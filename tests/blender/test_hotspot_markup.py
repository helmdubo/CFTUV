"""Blender 4.5 background smoke: режим Hotspot кнопки «Build Decal Mesh» (HOTSPOT_MARKUP_V1, HOTSPOT_ATLAS_MAPPING_V1).

Складка 90° (пол и стена, шов - ломаная из шести рёбер разной длины), выбран весь шов. Утверждения - на числах меша:

1. ВЫКЛЮЧЕН - меш без свойства `cftuv_hotspot`, дайджест тот же до и после прогонов с включённым режимом (режим не оставляет следа);
2. `trim` - UV ряда шва (UV ядра `v = 0`) ровно на нижнем крае метки, внешнего ряда (`v = 1`) - на верхнем, `u` вдоль длины - толщина метки на
   ширину; меш и UV ядра в остальном те же (число петель, порядок);
3. `corner` - шов на средней линии, крылья двух доменов - на верхней и нижней половинах, на КАЖДОЙ общей вершине шва крылья сходятся (допуск
   `MEET_TOLERANCE` в UV float32), `Swap wings` меняет половины;
4. отказы до прогона по именам: Uniform trim (`HOTSPOT_UV_POLICY_UNSUPPORTED`), нет метки (`HOTSPOT_MARK_NOT_FOUND`), непринятая разметка
   (`HOTSPOT_MARKUP_REFUSED`), нет ссылки материала (`MARKUP_NOT_LINKED`); меш при отказе не тронут;
5. ЖИВАЯ ШИРИНА: кадр модели и точный кадр той же ширины отображаются одной функцией; разрыв UV атласа напечатан и ограничен; узлы GPU
   affine drag под Hotspot не включаются (их формула двигает UV ядра, а в меше UV атласа);
6. «Mark Up» без окна (HOTSPOT_PANEL_EDITOR_V1): картинка из узлов материала декали, датаблок разметки создан и связан сам, отказы по именам.

Прогон: blender --background --factory-startup --python-exit-code 1 --python tests/blender/test_hotspot_markup.py
Последняя строка при успехе: HOTSPOT_MARKUP_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

# Общий модуль Blender-тестов ставит CFTUV_SNAPSHOT_BYTE_CHECK=all, пути репозитория и свежий импорт cftuv.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_envelope_debug_bridge import _reset_scene  # noqa: E402

import bmesh  # noqa: E402
import bpy  # noqa: E402
import numpy as np  # noqa: E402

SOURCE = "HotspotFold"
DECAL = SOURCE + ".CFTUV_Decal"
TEXT = "Sheet.hotspot"
MATERIAL = "CFTUV_Decal"
WIDTH, HEIGHT = 1024, 512
MEET_TOLERANCE = 1e-6
STATIONS = (0.0, 0.35, 0.6, 1.3, 2.05, 2.6, 3.2)


def _register():
    import cftuv

    cftuv.register()


def _build_fold():
    _reset_scene()
    vertices, faces = [], []
    for k, x in enumerate(STATIONS):
        bend = 1.0 + 0.06 * (-1) ** k  # шов - ломаная: грани полосы не сливаются, общих вершин шва столько же, сколько станций
        vertices += [(x, 0.0, 0.0), (x, bend, 0.0), (x, bend, 1.0)]
    for k in range(len(STATIONS) - 1):
        a, b = 3 * k, 3 * (k + 1)
        faces += [(a, b, b + 1, a + 1), (a + 1, b + 1, b + 2, a + 2)]
    mesh = bpy.data.meshes.new(SOURCE + "Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    seam = {tuple(sorted((3 * k + 1, 3 * (k + 1) + 1))) for k in range(len(STATIONS) - 1)}
    for edge in mesh.edges:
        edge.use_seam = tuple(sorted(edge.vertices)) in seam
    obj = bpy.data.objects.new(SOURCE, mesh)
    bpy.context.scene.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(mesh)
    for edge in bm.edges:
        edge.select = edge.seam
    bmesh.update_edit_mesh(mesh)
    return obj


def _markup(*marks, width=WIDTH, height=HEIGHT):
    from cftuv.hotspot_blender import link_markup, write_markup_text
    from cftuv.hotspot_markup import HotspotMarkupV1

    problems = write_markup_text(TEXT, HotspotMarkupV1(width, height, "sheet.png", tuple(marks)))
    assert problems == (), problems
    material = bpy.data.materials.get(MATERIAL) or bpy.data.materials.new(MATERIAL)
    link_markup(material, TEXT)


def _hotspot(enabled, mark_id=0, swap=False):
    settings = bpy.context.scene.hotspotuv_hotspot
    settings.enabled, settings.mark_id, settings.swap_wings = enabled, mark_id, swap


def _press(expect="FINISHED"):
    try:
        result = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    except RuntimeError:  # отчёт ERROR фонового Blender - исключение bpy.ops; оператор вернул CANCELLED
        result = {"CANCELLED"}
    status = bpy.context.scene.hotspotuv_decal_mesh.status
    assert result == {expect}, (result, status)
    return status


def _arrays():
    mesh = bpy.data.objects[DECAL].data
    loops = len(mesh.loops)
    uv = np.empty(loops * 2, dtype=np.float32)
    mesh.uv_layers["UVMap"].data.foreach_get("uv", uv)
    vertex = np.empty(loops, dtype=np.int32)
    mesh.loops.foreach_get("vertex_index", vertex)
    totals = np.empty(len(mesh.polygons), dtype=np.int32)
    mesh.polygons.foreach_get("loop_total", totals)
    domain = np.empty(len(mesh.polygons), dtype=np.int32)
    mesh.attributes["cftuv_domain"].data.foreach_get("value", domain)
    return uv.reshape(-1, 2).astype(np.float64), vertex, np.repeat(domain, totals)


def _digest():
    from cftuv.envelope_production_mesh import mesh_content_digest

    return mesh_content_digest(bpy.data.objects[DECAL].data)


def _check_off_and_trim(kernel):
    from cftuv.hotspot_markup import HotspotMarkV1

    uv, vertex, domain = kernel
    seam, outer = np.abs(uv[:, 1]) <= 1e-7, np.abs(uv[:, 1] - 1.0) <= 1e-7
    assert seam.any() and outer.any() and len(np.unique(domain)) == 2, "the fold gives two domains with seam and outer rows"
    _markup(HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, "+y"), HotspotMarkV1(1, "edge", "corner", 0, 256, 1024, 96, "+y", 12))
    _hotspot(True, 0)
    status = _press()
    mesh = bpy.data.objects[DECAL].data
    assert "cftuv_hotspot" in mesh.keys(), status
    atlas, vertex_on, _domain = _arrays()
    assert np.array_equal(vertex_on, vertex), "the mapping keeps the mesh: same loops in the same order"
    pixels = atlas * (WIDTH, HEIGHT)
    assert np.max(np.abs(pixels[seam, 1] - 64.0)) <= 1e-3, "the seam row is on the bottom edge of the mark"
    assert np.max(np.abs(pixels[outer, 1] - 192.0)) <= 1e-3, "the outer row is on the top edge of the mark"
    assert np.max(np.abs(pixels[:, 0] - uv[:, 0] * 128.0)) <= 2e-2, "one width of u is one mark thickness along the length"
    print(f"HOTSPOT TRIM seam rows {int(seam.sum())}, outer rows {int(outer.sum())}, max seam error {np.max(np.abs(pixels[seam, 1] - 64.0)):.2e} px")


def _check_corner(kernel, swap):
    uv, vertex, domain = kernel
    _hotspot(True, 1, swap)
    _press()
    atlas, _vertex, _domain = _arrays()
    seam, outer = np.abs(uv[:, 1]) <= 1e-7, np.abs(uv[:, 1] - 1.0) <= 1e-7
    pixels = atlas * (WIDTH, HEIGHT)
    assert np.max(np.abs(pixels[seam, 1] - 304.0)) <= 1e-3, "the seam is on the centre line"
    low, high = sorted(np.unique(domain))
    top, bottom = (high, low) if swap else (low, high)
    assert np.max(np.abs(pixels[outer & (domain == top), 1] - 352.0)) <= 1e-3, "the + wing goes to the upper half"
    assert np.max(np.abs(pixels[outer & (domain == bottom), 1] - 256.0)) <= 1e-3, "the - wing goes to the lower half"
    shared = 0
    gap = 0.0
    for item in np.unique(vertex[seam]):
        here = atlas[seam & (vertex == item)]
        if len(np.unique(domain[seam & (vertex == item)])) == 2:
            shared += 1
            gap = max(gap, float(np.max(np.abs(here - here[0]))))
    assert shared == len(STATIONS) and gap <= MEET_TOLERANCE, (shared, gap)
    from cftuv.hotspot_blender import LAST_COUNTERS

    print(f"HOTSPOT CORNER swap={swap}: {shared} shared seam vertices, max wing gap {gap:.2e} UV; {LAST_COUNTERS.get(bpy.data.objects[DECAL].data.session_uid)}")


def _check_refusals():
    from cftuv.hotspot_markup import HotspotMarkV1

    before = _digest()
    mesh_settings = bpy.context.scene.hotspotuv_decal_mesh
    mesh_settings.uv_policy_id = "UV_UNIFORM_TRIM_FAN_V1"
    assert "HOTSPOT_UV_POLICY_UNSUPPORTED" in _press("CANCELLED")
    mesh_settings.uv_policy_id = "UV_DIRECT_STRIP_V1"
    _hotspot(True, 7)
    assert "HOTSPOT_MARK_NOT_FOUND" in _press("CANCELLED")
    text = bpy.data.texts[TEXT]
    data = json.loads(text.as_string())
    data["marks"][1]["rect"]["y"] = 100  # перекрывает метку 0
    text.from_string(json.dumps(data))
    _hotspot(True, 0)
    assert "HOTSPOT_MARKUP_REFUSED" in _press("CANCELLED")
    from cftuv.hotspot_blender import link_markup

    link_markup(bpy.data.materials[MATERIAL], "")
    assert "MARKUP_NOT_LINKED" in _press("CANCELLED")
    assert _digest() == before, "a refused press leaves the mesh untouched"
    _markup(HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, "+y"), HotspotMarkV1(1, "edge", "corner", 0, 256, 1024, 96, "+y", 12))


def _check_live_width():
    """Кадр модели и точный кадр той же ширины под Hotspot: разрыв UV атласа - числом; GPU affine drag не включается."""

    from cftuv.envelope_width_adjust import KIND_CONFIRM, KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_session import ViewScaleV1, apply_step, begin_adjust, finish_adjust
    import time

    controller = bpy.context.window_manager._cftuv_envelope_debug_session
    bpy.context.scene.hotspotuv_decal_mesh.drag_gpu_affine = True
    view = ViewScaleV1(pivot=(0.0, 0.0), metres_per_pixel=1.0)
    runtime = begin_adjust(bpy.context, (0.0, 0.0), view)
    assert not isinstance(runtime, str), runtime
    assert controller.width_gpu_drag is False, "GPU affine drag stays off under Hotspot"
    for offset in (0.002, 0.004, 0.006):
        step = runtime.session.handle(WidthEventV1(KIND_MOVE, offset, 0.0))
        apply_step(bpy.context, runtime, step)
    preview, _vertex, _domain = _arrays()
    closing = runtime.session.handle(WidthEventV1(KIND_CONFIRM))
    apply_step(bpy.context, runtime, closing)
    assert finish_adjust(bpy.context, runtime, confirmed=True) == "FINISHED"
    end = time.perf_counter() + 120.0
    while controller.width_live is not None and controller.width_live.busy:
        controller.width_live._callback()
        time.sleep(0.01)
        assert time.perf_counter() < end
    exact, _vertex, _domain = _arrays()
    gap = float(np.max(np.abs(exact - preview)))
    print(f"HOTSPOT LIVE preview vs exact atlas uv: max {gap:.3e} UV ({gap * WIDTH:.3f} px of {WIDTH})")
    assert gap < 1e-3, gap
    bpy.context.scene.hotspotuv_decal_mesh.drag_gpu_affine = False
    return gap


def _check_mark_up_preparation():
    """HOTSPOT_PANEL_EDITOR_V1 без окна (фоновый Blender окон не открывает): картинка из узлов материала декали, датаблок разметки создан и
    связан сам, черновик загружен; отказы по именам - материал без картинки, непринятый черновик другой разметки."""

    from cftuv.hotspot_editor import MARKUP_DRAFT_UNSAVED, prepare_markup

    scene = bpy.context.scene
    mesh_settings = scene.hotspotuv_decal_mesh
    plain = bpy.data.materials.new("HotspotPlain")
    plain.use_nodes = True
    mesh_settings.material_name = plain.name
    assert prepare_markup(scene)[1] == "HOTSPOT_MATERIAL_HAS_NO_IMAGE"
    sheet = bpy.data.materials.new("HotspotSheet")
    sheet.use_nodes = True
    texture = sheet.node_tree.nodes.new("ShaderNodeTexImage")
    texture.image = bpy.data.images.new("panel_sheet.png", 512, 256)
    principled = next(node for node in sheet.node_tree.nodes if node.type == "BSDF_PRINCIPLED")
    sheet.node_tree.links.new(texture.outputs["Color"], principled.inputs["Base Color"])
    mesh_settings.material_name = sheet.name
    image, refusal, detail = prepare_markup(scene)
    editor = scene.hotspotuv_hotspot_editor
    assert image is not None and image.name == "panel_sheet.png" and refusal == "", (refusal, detail)
    assert sheet["cftuv_hotspot_markup"] == "panel_sheet.png.hotspot" and editor.draft_of == "panel_sheet.png.hotspot"
    assert (editor.width, editor.height, len(editor.marks)) == (512, 256, 0) and not editor.dirty
    editor.marks.add()  # пустой прямоугольник - черновик не принят
    from cftuv.hotspot_editor import commit_draft

    assert commit_draft(scene) and editor.dirty
    other = bpy.data.materials.new("HotspotOther")
    other.use_nodes = True
    other.node_tree.nodes.new("ShaderNodeTexImage").image = bpy.data.images.new("panel_other.png", 256, 256)
    mesh_settings.material_name = other.name
    assert prepare_markup(scene)[1] == MARKUP_DRAFT_UNSAVED, "an unsaved draft of another markup is never replaced"
    assert len(editor.marks) == 1 and editor.draft_of == "panel_sheet.png.hotspot" and "panel_other.png.hotspot" not in bpy.data.texts
    mesh_settings.material_name = sheet.name
    assert prepare_markup(scene)[1] == "" and len(editor.marks) == 1 and editor.dirty, "the same markup keeps its unsaved draft"
    from cftuv.hotspot_editor import load_draft

    assert load_draft(scene, editor.draft_of, discard=True) == "" and not editor.dirty
    mesh_settings.material_name = MATERIAL
    print("HOTSPOT MARK UP preparation: image from nodes, text created and linked, refusals named", flush=True)


def main():
    _register()
    _check_mark_up_preparation()
    bpy.context.scene.hotspotuv_settings.envelope_debug_alpha = 0.25
    _build_fold()
    _hotspot(False)
    _press()
    off = _digest()
    assert "cftuv_hotspot" not in bpy.data.objects[DECAL].data.keys()
    kernel = _arrays()
    _check_off_and_trim(kernel)
    _check_corner(kernel, swap=False)
    _check_corner(kernel, swap=True)
    _check_refusals()
    _hotspot(True, 0)
    _press()
    _check_live_width()
    _hotspot(True, 1)
    _press()
    _check_live_width()
    _hotspot(False)
    bpy.context.scene.hotspotuv_settings.envelope_debug_alpha = 0.25
    _press()
    assert _digest() == off, "off after on is the same mesh, bitwise"
    print("HOTSPOT_MARKUP_BLENDER_SMOKE_OK", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception:  # noqa: BLE001 - фоновый Blender: ненулевой код выхода
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
    sys.stdout.flush()
    os._exit(0)
