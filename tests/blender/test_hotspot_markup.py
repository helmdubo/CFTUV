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
6. «Mark Up» без окна (HOTSPOT_PANEL_EDITOR_V1): картинка из узлов материала декали, датаблок разметки создан и связан сам, отказы по именам;
7. МЕТКИ ОСТРОВОВ (HOTSPOT_ISLAND_MARKS_V1, HOTSPOT_APPLY_ON_PICK_V1): кнопка с меткой A, выбор метки B в меню панели при одном выделенном
   острове - его UV в B, другого в A, выделен ровно он (кнопки Apply нет); живая ширина (кадры и подтверждение) и повторная кнопка метки
   сохраняют; corner одной стороне шва - обе стороны (сосед по шву); Undo (если фоновый Blender его даёт) возвращает UV; отказы по именам,
   ничего применимого не выделено - метка только выбрана, присваивание настроек из кода (как при Undo/Redo) ничего не применяет, меш не
   тронут; режим выключен - ни записи, ни атрибутов `cftuv_hotspot_mark` и `cftuv_strip_uv`;
8. ОТДЕЛЬНОЕ ОКНО (HOTSPOT_EDITOR_WINDOW_V1): метки рисует только область мини-окна; второй Image Editor с той же картинкой - нет;
   подложка (HOTSPOT_BACKDROP_V1): другой размер при метках - отказ по имени, без меток разметка переходит в размер картинки;
   HOTSPOT_EDITOR_UX_V1: боковой панели нет, заголовок Image Editor обёрнут; поворот на соседа - MARK_ROTATE_BLOCKED, поворот и разворот
   стрелки, операторы меню правой кнопки на черновике сцены;
9. OBJECT MODE (HOTSPOT_OBJECT_APPLY_V1): выбор метки при двух выделенных декалях и не декали - все острова обеих декалей, не декаль не
   тронута; Swap wings кладёт то же самое с переставленными крыльями.

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


def _check_live_width(strict=True):
    """Кадр модели и точный кадр той же ширины под Hotspot: разрыв UV атласа - числом; GPU affine drag не включается. `strict=False` - после
    правки меша вне живой ширины (Apply Mark): модель снята (PREVIEW_DECAL_CHANGED_EXTERNALLY), и кадры до её новой сборки могут меш не двигать."""

    from cftuv.envelope_width_adjust import KIND_CONFIRM, KIND_MOVE, WidthEventV1
    from cftuv.envelope_width_session import ViewScaleV1, apply_step, begin_adjust, finish_adjust
    import time

    controller = bpy.context.window_manager._cftuv_envelope_debug_session
    bpy.context.scene.hotspotuv_decal_mesh.drag_gpu_affine = True
    view = ViewScaleV1(pivot=(0.0, 0.0), metres_per_pixel=1.0)
    before, _vertex, _domain = _arrays()
    runtime = begin_adjust(bpy.context, (0.0, 0.0), view)
    assert not isinstance(runtime, str), runtime
    assert controller.width_gpu_drag is False, "GPU affine drag stays off under Hotspot"
    for offset in (0.002, 0.004, 0.006):
        step = runtime.session.handle(WidthEventV1(KIND_MOVE, offset, 0.0))
        apply_step(bpy.context, runtime, step)
    preview, _vertex, _domain = _arrays()
    moved = float(np.max(np.abs(preview - before))) if preview.shape == before.shape else float("nan")
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
    print(f"HOTSPOT LIVE preview vs exact atlas uv: max {gap:.3e} UV ({gap * WIDTH:.3f} px of {WIDTH}); the frames moved the mesh by {moved:.3e} UV", flush=True)
    assert gap < 1e-3 or not strict, gap
    bpy.context.scene.hotspotuv_decal_mesh.drag_gpu_affine = False
    return gap


RUST = (400.0, 464.0)  # метка 2 (trim): шов и внешний ряд в пикселях
DRIPS = (64.0, 192.0)  # метка 0 (trim)


def _object_mode():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")


def _decal_edit(domain=None):
    """Edit Mode на декали, выделены грани домена `domain` (`None` - ничего)."""

    _object_mode()
    decal = bpy.data.objects[DECAL]
    for item in bpy.context.view_layer.objects:
        item.select_set(False)
    bpy.context.view_layer.objects.active = decal
    decal.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, False, True)
    bm = bmesh.from_edit_mesh(decal.data)
    layer = bm.faces.layers.int["cftuv_domain"]
    for item in (*bm.verts, *bm.edges, *bm.faces):
        item.select = False
    for face in bm.faces:
        if domain is not None and face[layer] == domain:
            face.select_set(True)
    bm.select_flush_mode()
    bmesh.update_edit_mesh(decal.data)


def _apply(note=False, op="hotspot_pick_mark"):
    """Выбор метки панели (HOTSPOT_APPLY_ON_PICK_V1): оператор пункта меню (или Swap wings) кладёт метку настроек на выделение. `note` -
    вернуть строку панели (отказ по имени или «метка только выбрана»), иначе - числа меша декали, и выбор обязан был что-то применить."""

    from cftuv.hotspot_blender import LAST_COUNTERS

    settings = bpy.context.scene.hotspotuv_hotspot
    result = bpy.ops.hotspotuv.hotspot_pick_mark(mark_id=settings.mark_id) if op == "hotspot_pick_mark" else getattr(bpy.ops.hotspotuv, op)()
    assert result == {"FINISHED"}, (result, settings.apply_note)
    if note:
        return settings.apply_note
    assert not settings.apply_note.startswith("Mark set for the next") and ":" in settings.apply_note, settings.apply_note
    return dict(LAST_COUNTERS.get(bpy.data.objects[DECAL].data.session_uid, {}))


def _current():
    """`(UV атласа, UV ядра обращением меток островов, домен угла)` меша декали сейчас (ширина могла смениться - маски по его же UV ядра)."""

    from cftuv.hotspot_blender import face_codes, mesh_record, recovered_strip

    _object_mode()
    atlas, _vertex, domain = _arrays()
    mesh = bpy.data.objects[DECAL].data
    record, _error = mesh_record(mesh)
    return atlas, recovered_strip(mesh, record, face_codes(mesh, len(mesh.polygons))), domain


def _rows(domain_rows, mark_rows, where):
    """Шов и внешний ряд углов домена `domain_rows` лежат на нижнем и верхнем крае trim-метки `mark_rows` (пиксели)."""

    atlas, strip, domain = _current()
    pixels = atlas * (WIDTH, HEIGHT)
    seam, outer = (np.abs(strip[:, 1]) <= 1e-7) & (domain == domain_rows), (np.abs(strip[:, 1] - 1.0) <= 1e-7) & (domain == domain_rows)
    assert seam.any() and outer.any(), where
    error = max(float(np.max(np.abs(pixels[seam, 1] - mark_rows[0]))), float(np.max(np.abs(pixels[outer, 1] - mark_rows[1]))))
    assert error <= 1e-3, (where, domain_rows, mark_rows, error)
    return error


def _undo_available():
    try:
        return bool(bpy.ops.ed.undo.poll())
    except RuntimeError:
        return False


def _check_island_marks(kernel):
    """HOTSPOT_ISLAND_MARKS_V1: метка островам выделения готовой декали, без пересборки; живая ширина и кнопка их сохраняют; отказы по именам."""

    from cftuv.hotspot_markup import HotspotMarkV1

    _uv, _vertex, domain = kernel
    low, high = sorted(np.unique(domain))
    source = bpy.data.objects[SOURCE]
    _markup(
        HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, "+y"),
        HotspotMarkV1(1, "edge", "corner", 0, 256, 1024, 96, "+y", 12),
        HotspotMarkV1(2, "rust", "trim", 0, 400, 1024, 64, "+y"),
    )
    _select_seam(source)
    _hotspot(True, 0)
    _press()
    _decal_edit(low)
    _hotspot(True, 2)
    counters = _apply()
    assert counters["HOTSPOT_ISLANDS_SELECTED"] == 1 and counters["HOTSPOT_CORNER_PAIR_EXPANDED"] == 0 and counters["HOTSPOT_ISLANDS_OFF_DEFAULT"] == 1, counters
    selected = [face.select for face in bmesh.from_edit_mesh(bpy.data.objects[DECAL].data).faces]
    _rows(low, RUST, "applied")
    _rows(high, DRIPS, "untouched")
    record = json.loads(bpy.data.objects[DECAL].data["cftuv_hotspot"])
    assert record["version"] == 2 and [item["mark"] for item in record["islands"]] == [2], record["islands"]
    assert all(selected[index] == (value == low) for index, value in enumerate(_face_domains())), "exactly the applied island is selected"
    undo = _undo_available()
    if undo:
        _decal_edit(low)
        _hotspot(True, 0)
        _apply()
        bpy.ops.ed.undo()
        _rows(low, RUST, "undo")
    print(f"HOTSPOT ISLANDS apply: {counters}; undo {'checked' if undo else 'not available in background (windowed proof)'}", flush=True)
    drift = _drift(low)
    _select_seam(source)
    gap = _check_live_width(strict=False)
    _rows(low, RUST, "after the live width")
    _rows(high, DRIPS, "after the live width")
    print(f"HOTSPOT ISLANDS live width keeps the marks: preview vs exact {gap:.2e} UV, {LAST_COUNTERS_OF()}; drift {drift}", flush=True)
    _decal_edit(high)
    _hotspot(True, 1, True)
    counters = _apply()
    assert counters["HOTSPOT_ISLANDS_SELECTED"] == 2 and counters["HOTSPOT_CORNER_PAIR_EXPANDED"] == 1 and counters["HOTSPOT_CORNER_WING_UNPAIRED"] == 0, counters
    atlas, strip, _domain = _current()
    seam = np.abs(strip[:, 1]) <= 1e-7
    assert np.max(np.abs(atlas[seam, 1] * HEIGHT - 304.0)) <= 1e-3, "both sides of the seam took the corner mark"
    _select_seam(source)
    _hotspot(True, 0)
    _press()
    record = json.loads(bpy.data.objects[DECAL].data["cftuv_hotspot"])
    assert record["default"] == {"mark": 0, "swap": False} and sorted((item["mark"], item["swap"]) for item in record["islands"]) == [(1, True), (1, True)], record
    atlas, strip, _domain = _current()
    assert np.max(np.abs(atlas[np.abs(strip[:, 1]) <= 1e-7, 1] * HEIGHT - 304.0)) <= 1e-3, "the press carries the island marks over"
    print(f"HOTSPOT ISLANDS corner pair: {counters}; press carried {len(record['islands'])} island marks", flush=True)
    _check_island_refusals(low)
    _check_object_apply()
    _select_seam(source)


def _drift(low):
    """50 нажатий «Apply Mark» по очереди rust <-> edge (corner, пара шва) на одном острове, через float32 слоя UV: накопление против первого круга."""

    from cftuv.hotspot_blender import KERNEL_UVS

    result = {}
    for session in (True, False):  # UV ядра сессии есть (точно) / нет, как после перезагрузки файла (обращение, развёртка ряда шва)
        seen = []
        for step in range(50):
            if not session:
                KERNEL_UVS.clear()
            _decal_edit(low)
            _hotspot(True, 1 if step % 2 == 0 else 2)
            counters = _apply()
            _object_mode()
            seen.append((_arrays()[0], next(key for key in counters if key.startswith("HOTSPOT_REMAP_"))))
        drift = max(float(np.max(np.abs(seen[-1][0] - seen[1][0]))), float(np.max(np.abs(seen[-2][0] - seen[0][0])))) * WIDTH
        result[sorted({item[1] for item in seen})[0] if session else "+".join(sorted({item[1] for item in seen}))] = drift
        assert drift < 1e-2, (session, drift)
    paths = sorted(result)
    drift = result
    print(f"HOTSPOT ISLANDS drift after 50 alternating applies, px of {WIDTH}: {result}", flush=True)
    _decal_edit(_other(low))  # corner ушёл и на пару шва: её возвращаем в drips
    _hotspot(True, 0)
    _apply()
    _rows(low, RUST, "after the alternating applies")
    _rows(_other(low), DRIPS, "after the alternating applies")
    return drift, paths


def _other(low):
    return next(value for value in set(_face_domains()) if value != low)


def _face_domains():
    mesh = bpy.data.objects[DECAL].data
    values = np.empty(len(mesh.polygons), dtype=np.int32)
    mesh.attributes["cftuv_domain"].data.foreach_get("value", values)
    return values.tolist()


def LAST_COUNTERS_OF():  # noqa: N802 - строка печати
    from cftuv.hotspot_blender import LAST_COUNTERS

    return LAST_COUNTERS.get(bpy.data.objects[DECAL].data.session_uid)


def _check_island_refusals(low):
    """Отказ декали - по имени в строке панели; ничего применимого не выделено - метка только выбрана (не ошибка); присваивание настроек из
    кода (так их возвращают Undo/Redo и чтение файла) ничего не применяет. Меш во всех случаях не тронут."""

    _object_mode()
    before = _digest()
    _decal_edit(low)
    _hotspot(True, 7)
    assert "HOTSPOT_MARK_NOT_FOUND" in _apply(note=True)
    _hotspot(True, 2)
    _decal_edit(None)
    assert "no faces of the decal selected" in _apply(note=True)
    _object_mode()
    bpy.ops.object.mode_set(mode="SCULPT")  # ни Object Mode (выделенные декали), ни Edit Mode (выделенные грани)
    assert "Sculpt applies no mark" in _apply(note=True)
    _object_mode()
    source = bpy.data.objects[SOURCE]
    _only_selected(source)
    assert "no decal selected" in _apply(note=True), "Object Mode with no decal selected"
    bpy.ops.object.mode_set(mode="EDIT")
    assert "Edit Mode is not on a decal" in _apply(note=True), "Edit Mode on the source before a build"
    _object_mode()
    assert _digest() == before, "a refused or idle pick leaves the mesh untouched"
    for mode, select in (("EDIT", low), ("OBJECT", None)):
        _decal_edit(select) if mode == "EDIT" else _only_selected(bpy.data.objects[DECAL])
        settings = bpy.context.scene.hotspotuv_hotspot
        settings.mark_id, settings.swap_wings = 1, not settings.swap_wings  # так значения возвращают Undo/Redo и чтение файла
        settings.mark_id, settings.swap_wings = 2, not settings.swap_wings
        _object_mode()
        assert _digest() == before, (mode, "assigning the settings applies nothing")
    _hotspot(False)
    _select_seam(source)
    _press()
    _decal_edit(low)
    _object_mode()
    mesh = bpy.data.objects[DECAL].data
    assert "cftuv_hotspot" not in mesh.keys() and "cftuv_hotspot_mark" not in mesh.attributes, "Hotspot off: no record, no attribute"
    kernel_off, _vertex, domain = _arrays()
    _decal_edit(low)
    _hotspot(True, 2)
    counters = _apply()  # декаль без Hotspot - тождественный случай: UV слоя и есть UV ядра
    _object_mode()
    atlas, _vertex, _domain = _arrays()
    mesh = bpy.data.objects[DECAL].data
    assert json.loads(mesh["cftuv_hotspot"])["default"]["mark"] is None and counters["HOTSPOT_ISLANDS_OFF_DEFAULT"] == 1, counters
    assert np.array_equal(atlas[domain != low].astype(np.float32), kernel_off[domain != low].astype(np.float32)), "islands without a mark keep their kernel UV bit for bit"
    _rows(low, RUST, "applied to a decal built with Hotspot off")
    print(f"HOTSPOT ISLANDS pick: HOTSPOT_MARK_NOT_FOUND named; no faces / Sculpt / no decal / Edit Mode on the source only set the mark; assigning the settings applies nothing; Hotspot-off decal mapped on pick: {counters}", flush=True)


def _only_selected(*objects):
    _object_mode()
    for item in bpy.context.view_layer.objects:
        item.select_set(False)
    for item in objects:
        item.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]


def _object_codes(obj):
    values = np.empty(len(obj.data.polygons), dtype=np.int32)
    obj.data.attributes["cftuv_hotspot_mark"].data.foreach_get("value", values)
    return values


def _check_object_apply():
    """HOTSPOT_OBJECT_APPLY_V1: Object Mode, выделены две декали и не декаль (она же активна): метка ложится на ВСЕ острова обеих, не декаль не
    тронута; corner - обе стороны шва парой у каждой декали (то же правило топологии); Undo (если фоновый Blender его даёт) - одним шагом."""

    from cftuv.hotspot_blender import LAST_COUNTERS, recovered_strip, face_codes, mesh_record

    _object_mode()
    decal, source = bpy.data.objects[DECAL], bpy.data.objects[SOURCE]
    twin = decal.copy()
    twin.data = decal.data.copy()
    twin.name = "Hand renamed decal"  # декаль узнаётся по маркеру, не по имени
    bpy.context.scene.collection.objects.link(twin)
    source_keys = set(source.data.keys())
    try:
        _only_selected(source, decal, twin)
        _hotspot(True, 0)
        _apply()
        for obj in (decal, twin):
            assert set(_object_codes(obj).tolist()) == {0}, (obj.name, "every island took mark 0")
            record, _error = mesh_record(obj.data)
            atlas = np.empty(len(obj.data.loops) * 2, dtype=np.float32)
            obj.data.uv_layers["UVMap"].data.foreach_get("uv", atlas)
            strip = recovered_strip(obj.data, record, face_codes(obj.data, len(obj.data.polygons)))
            seam = np.abs(strip[:, 1]) <= 1e-7
            assert np.max(np.abs(atlas.reshape(-1, 2)[seam, 1] * HEIGHT - DRIPS[0])) <= 1e-3, (obj.name, "seam rows on the trim's bottom edge")
        assert set(source.data.keys()) == source_keys and "cftuv_hotspot_mark" not in source.data.attributes, "the non-decal is skipped, untouched"
        _hotspot(True, 1)
        _apply()
        for obj in (decal, twin):
            counters = LAST_COUNTERS[obj.data.session_uid]
            assert counters["HOTSPOT_ISLANDS_SELECTED"] == 2 and counters["HOTSPOT_CORNER_WING_UNPAIRED"] == 0, (obj.name, counters)
            assert set((_object_codes(obj) >> 1).tolist()) == {1}
        undo = _undo_available()
        if undo:
            bpy.ops.ed.undo()
            assert set(_object_codes(bpy.data.objects[DECAL]).tolist()) == {0}, "one undo step returns both decals"
        _only_selected(source, decal, twin)
        _hotspot(True, 1)
        _apply()
        swapped = {obj.name: _object_codes(obj).copy() for obj in (decal, twin)}
        assert "HOTSPOT_NOT_A_DECAL_MESH=1" in _apply(note=True, op="hotspot_swap_wings"), "Swap wings applies like a pick and counts the non-decal"
        assert bpy.context.scene.hotspotuv_hotspot.swap_wings
        for obj in (decal, twin):
            codes = _object_codes(obj)
            assert set((codes >> 1).tolist()) == {1} and np.array_equal(codes ^ 1, swapped[obj.name]), (obj.name, "every wing swapped")
        _apply(op="hotspot_swap_wings")
        assert all(np.array_equal(_object_codes(obj), swapped[obj.name]) for obj in (decal, twin)), "swapping back"
        _only_selected(source)
        assert "no decal selected" in _apply(note=True)
    finally:
        _object_mode()
        twin = bpy.data.objects.get("Hand renamed decal")
        if twin is not None:
            mesh = twin.data
            bpy.data.objects.remove(twin)
            bpy.data.meshes.remove(mesh)
    print(f"HOTSPOT OBJECT apply: 2 decals + 1 non-decal, every island trim then corner pairs; undo {'checked' if undo else 'not available in background (windowed proof)'}", flush=True)


def _select_seam(source):
    """Источник активен в Edit Mode с выделенным швом (выделение сборки)."""

    _object_mode()
    for item in bpy.context.view_layer.objects:
        item.select_set(False)
    bpy.context.view_layer.objects.active = source
    source.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(source.data)
    for item in (*bm.verts, *bm.edges, *bm.faces):
        item.select = False
    for edge in bm.edges:
        edge.select = edge.seam
    bmesh.update_edit_mesh(source.data)


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
    _check_editor_edits(scene)
    _check_backdrop(scene, sheet, principled)
    _check_overlay_gate(editor)
    mesh_settings.material_name = MATERIAL
    print("HOTSPOT MARK UP preparation: image from nodes, text created and linked, refusals named", flush=True)


def _check_editor_edits(scene):
    """HOTSPOT_EDITOR_UX_V1 на черновике сцены (без окна): поворот на другую метку - отказ MARK_ROTATE_BLOCKED, черновик и текст прежние; без
    соседа - вертикальная полоса во всю высоту, проверена и записана; разворот стрелки; операторы меню правой кнопки (Rotate, Flip, Kind, Delete)."""

    from cftuv.hotspot_blender import read_markup_text
    from cftuv.hotspot_editor import commit_draft, flip_active, rotate_active

    editor = scene.hotspotuv_hotspot_editor
    for mark_id, y in ((0, 0), (1, 128)):
        item = editor.marks.add()
        item.mark_id, item.label, item.kind, item.up, item.x, item.y, item.w, item.h = mark_id, f"m{mark_id}", "trim", "+y", 0, y, 512, 64
    assert commit_draft(scene) == ()
    editor.active, editor.grid = 0, 8
    assert rotate_active(editor) == "MARK_ROTATE_BLOCKED" and "overlaps mark(s) 1" in editor.notice
    assert (editor.marks[0].up, editor.marks[0].w) == ("+y", 512), "a refused turn changes nothing"
    editor.active = 1
    assert bpy.ops.hotspotuv.hotspot_mark_delete() == {"FINISHED"} and len(editor.marks) == 1 and not editor.dirty
    editor.active = 0
    assert rotate_active(editor) == "" and not editor.notice
    mark = editor.marks[0]
    assert (mark.x, mark.y, mark.w, mark.h, mark.up) == (224, 0, 64, 256, "+x"), (mark.x, mark.y, mark.w, mark.h, mark.up)
    assert commit_draft(scene) == () and read_markup_text(editor.draft_of)[0].marks[0].up == "+x", "the turned strip is valid and saved"
    assert flip_active(editor) == "" and editor.marks[0].up == "-x"
    assert bpy.ops.hotspotuv.hotspot_mark_rotate() == {"FINISHED"} and editor.marks[0].up == "+y" and editor.marks[0].w == 512
    assert bpy.ops.hotspotuv.hotspot_mark_flip() == {"FINISHED"} and editor.marks[0].up == "-y"
    assert bpy.ops.hotspotuv.hotspot_mark_kind(kind="corner") == {"FINISHED"} and read_markup_text(editor.draft_of)[0].marks[0].kind == "corner"
    assert bpy.ops.hotspotuv.hotspot_mark_delete() == {"FINISHED"} and len(editor.marks) == 0 and not editor.dirty
    assert read_markup_text(editor.draft_of)[0].marks == ()
    print("HOTSPOT EDITOR edits: turn refused by name onto a neighbour, turned to a full-height strip, flipped, menu operators", flush=True)


def _check_backdrop(scene, sheet, principled):
    """HOTSPOT_BACKDROP_V1: подложка мини-окна из картинок материала; другой размер при метках - отказ по имени (подложка и разметка прежние),
    без меток разметка переходит в размер картинки; выбор помнится настройкой следующего «Mark Up»."""

    from cftuv.hotspot_blender import HOTSPOT_BACKDROP_SIZE_MISMATCH, read_markup_text
    from cftuv.hotspot_editor import backdrop_images, commit_draft, switch_backdrop
    from cftuv.hotspot_markup import HotspotMarkV1

    editor = scene.hotspotuv_hotspot_editor
    for name, size in (("panel_normal.png", (1024, 512)), ("panel_rough.png", (512, 256))):
        node = sheet.node_tree.nodes.new("ShaderNodeTexImage")
        node.image = bpy.data.images.new(name, *size)
    assert [image.name for image in backdrop_images(editor)] == ["panel_sheet.png", "panel_normal.png", "panel_rough.png"]
    from cftuv.hotspot_editor import _backdrop_items

    labels = [item[1] for item in _backdrop_items(editor, bpy.context)]
    assert labels == ["panel_sheet.png (512 x 256 px)", "panel_normal.png (1024 x 512 px) - not the markup's 512 x 256", "panel_rough.png (512 x 256 px)"], labels
    assert switch_backdrop(scene, "panel_rough.png") == "" and editor.backdrop == "panel_rough.png" and scene.hotspotuv_hotspot.image_name == "panel_rough.png"
    assert (editor.width, editor.height, editor.image) == (512, 256, "panel_sheet.png"), "a backdrop of the same size is display only"
    item = editor.marks.add()
    item.mark_id, item.label, item.kind, item.up, item.x, item.y, item.w, item.h = 0, "a", "trim", "+y", 0, 0, 512, 64
    assert commit_draft(scene) == () and not editor.dirty
    assert switch_backdrop(scene, "panel_normal.png") == HOTSPOT_BACKDROP_SIZE_MISMATCH and HOTSPOT_BACKDROP_SIZE_MISMATCH in editor.backdrop_problem
    assert editor.backdrop == "panel_rough.png" and (editor.width, editor.height) == (512, 256), "a refused switch changes nothing"
    editor.marks.remove(0)
    commit_draft(scene)
    assert switch_backdrop(scene, "panel_normal.png") == "" and (editor.width, editor.height, editor.image) == (1024, 512, "panel_normal.png") and not editor.backdrop_problem
    markup, _problems = read_markup_text(editor.draft_of)
    assert (markup.width, markup.height) == (1024, 512), "an empty markup takes the backdrop's size, written validated"
    assert switch_backdrop(scene, "panel_sheet.png") == "" and (editor.width, editor.height) == (512, 256)
    scene.hotspotuv_hotspot.image_name = ""
    print("HOTSPOT BACKDROP: same size shown, another size refused by name with marks, an empty markup rebased", flush=True)


def _check_overlay_gate(editor):
    """HOTSPOT_EDITOR_WINDOW_V1: метки рисует только область мини-окна; второй Image Editor с той же картинкой - нет (пока пользователь не включит
    «Show marks in UV Editor»); закрытое окно (запись очищена) - нигде; вкладка Hotspot и инструмент - тоже только там."""

    from cftuv import hotspot_editor
    from cftuv.hotspot_blender import OVERLAY_EDITOR, OVERLAY_ELSEWHERE

    window = bpy.context.window_manager.windows[0]
    screen = window.screen
    first, second = sorted(screen.areas, key=lambda area: -area.width * area.height)[:2]
    kinds = (first.type, second.type)
    image = bpy.data.images[editor.image]
    try:
        for area in (first, second):
            area.type = "IMAGE_EDITOR"
            area.spaces.active.image = image
        hotspot_editor._MINI.update(window=window.as_pointer(), screen=screen.as_pointer(), area=first.as_pointer())
        assert hotspot_editor.overlay_mode(window, screen, first, editor) == OVERLAY_EDITOR
        assert hotspot_editor.overlay_mode(window, screen, second, editor) == "", "a second Image Editor showing the same image draws no marks"
        with bpy.context.temp_override(window=window, area=second):
            assert not hotspot_editor.is_mini_context(bpy.context)
        with bpy.context.temp_override(window=window, area=first):
            assert hotspot_editor.is_mini_context(bpy.context)
        # HOTSPOT_EDITOR_UX_V1: боковой панели нет; заголовок мини-окна - свой, у других Image Editor - заголовок Blender.
        assert not hasattr(bpy.types, "HOTSPOTUV_PT_HotspotMarkup") and bpy.types.IMAGE_HT_header.draw is hotspot_editor._image_header_draw
        assert hotspot_editor._HEADER["draw"] is not hotspot_editor._image_header_draw
        editor.show_in_uv_editor = True
        assert hotspot_editor.overlay_mode(window, screen, second, editor) == OVERLAY_ELSEWHERE
        editor.show_in_uv_editor = False
        hotspot_editor._MINI.clear()
        assert hotspot_editor.overlay_mode(window, screen, first, editor) == "", "the closed mini editor draws nowhere"
    finally:
        hotspot_editor._MINI.clear()
        first.type, second.type = kinds
    print("HOTSPOT OVERLAY gate: the mini editor area only; a second Image Editor with the same image draws nothing", flush=True)


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
    _check_island_marks(kernel)
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
