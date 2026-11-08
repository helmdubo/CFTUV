"""Blender 4.5 background: «Build Decal Mesh» по выделенным граням бевела (BEVEL_FACES_INWARD_OUTWARD_V1).

Синтетика в памяти (ничего не сохраняется): куб, L-профиль и плита с отверстием; бевел модификатором (применён: `new_from_object`)
в 1, 2 и 4 сегмента с Face Strength и без, и ручной `bmesh.ops.bevel`. Выделены грани бевела (материал 1 - только чтобы их выбрать).

1. INWARD: каждый патч анализа (полосы И узлы) - MATERIALIZED, отказов нет, ни одна грань не осталась неразрешённой; анализ только выделенных
   граней, разрез «узел | полоса» есть (узлов > 0); источник разбиения - атрибут, если он есть, иначе геометрия;
2. OUTWARD: домены - только плоские стороны (патчи анализа невыделенных граней), все MATERIALIZED;
3. меш владельца не тронут: ни одного шва, отпечаток меша тот же, выделение граней то же; запись живой ширины сверяет меш по
   отпечатку МЕША (`mesh_digest`), и он равен отпечатку меша сейчас;
4. путь по рёбрам после нажатий по граням даёт тот же меш декали, что и в свежей сессии (кэши не смешаны);
5. замкнутый меш без швов по рёбрам - именованный отказ `SOURCE_HAS_NO_BOUNDARY_CHAIN`, а не исключение;
6. триангулированный бевел без атрибута - именованный отказ `BEVEL_SPLIT_UNRESOLVED` (меш декали не создан), с атрибутом - строится;
7. строка «Faces: Inward / Outward» панели рисуется только в режиме выделения граней;
8. живая ширина декали нажатия по граням доступна, и её точный пересчёт побитово равен нажатию на новой ширине;
9. OUTWARD - это обычная процедура на меше без граней бевела (BEVEL_FACES_OUTWARD_IS_THE_EDGE_PATH_V1, поле: шумный верх
   `rounded_wall_noise_top.001` без декали): четверть кольца с шумным триангулированным верхом, бевел обода, вертикальных рёбер
   торцов и низа; та же строка статуса и побитово тот же меш декали, что у нажатия по рёбрам открытой границы того же меша с
   удалёнными гранями бевела; умеренный шум - верх строится, сильный - домен верха не молчит (каждый патч анализа - домен со
   своим исходом, отказ назван в статусе).

Прогон: blender -b --factory-startup --python-exit-code 1 --python tests/blender/test_bevel_faces_build.py
Последняя строка при успехе: BEVEL_FACES_BUILD_BLENDER_OK
"""

from __future__ import annotations

import hashlib
import math
import random
import re
import sys
import time
from pathlib import Path

import bpy

try:
    import sympy  # noqa: F401
except ImportError:  # --factory-startup: sympy живёт в каталоге модулей пользователя
    sys.path.append(bpy.utils.user_resource("SCRIPTS", path="modules"))

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_envelope_debug_bridge import _enter_edge_selection, _reset_scene  # noqa: E402

import bmesh  # noqa: E402

WIDTH = 0.08
ALPHA = 0.05
SUMMARY = []


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _decal_settings():
    return bpy.context.scene.hotspotuv_decal_mesh


def _controller():
    return getattr(bpy.context.window_manager, "_cftuv_envelope_debug_session", None)


def _clean():
    _reset_scene()
    controller = _controller()
    if controller is not None:
        controller.clear()
    for mesh in list(bpy.data.meshes):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)


def _from_pydata(verts, faces):
    bm = bmesh.new()
    made = [bm.verts.new(item) for item in verts]
    for cycle in faces:
        bm.faces.new([made[index] for index in cycle])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


def _cube():
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=2.0)
    return bm


def _lprofile():
    poly = [(0, 0), (2, 0), (2, 0.6), (0.6, 0.6), (0.6, 2), (0, 2)]
    count = len(poly)
    verts = [(x, y, 0.0) for x, y in poly] + [(x, y, 2.0) for x, y in poly]
    faces = [tuple(reversed(range(count))), tuple(range(count, 2 * count))]
    faces += [(index, (index + 1) % count, count + (index + 1) % count, count + index) for index in range(count)]
    return _from_pydata(verts, faces)


def _plate_hole():
    outer = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    inner = [(-0.4, -0.4), (0.4, -0.4), (0.4, 0.4), (-0.4, 0.4)]
    verts = []
    for z in (0.0, 0.4):
        verts += [(x, y, z) for x, y in outer] + [(x, y, z) for x, y in inner]
    faces = []
    for k in range(4):
        m = (k + 1) % 4
        faces += [(8 + k, 8 + m, 12 + m, 12 + k), (m, k, 4 + k, 4 + m), (k, m, 8 + m, 8 + k), (4 + m, 4 + k, 12 + k, 12 + m)]
    return _from_pydata(verts, faces)


BASES = {"cube": _cube, "lprofile": _lprofile, "plate_hole": _plate_hole}


def _object(name, bm):
    mesh = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(mesh)
    bm.free()
    for slot in range(2):
        mesh.materials.append(bpy.data.materials.get(f"BevelTest{slot}") or bpy.data.materials.new(f"BevelTest{slot}"))
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _modifier_bevel(name, base, segments, face_strength):
    source = _object(name + "_base", BASES[base]())
    modifier = source.modifiers.new("Bevel", "BEVEL")
    modifier.width, modifier.segments, modifier.limit_method, modifier.material = WIDTH, segments, "ANGLE", 1
    modifier.face_strength_mode = "FSTR_NEW" if face_strength else "FSTR_NONE"
    depsgraph = bpy.context.evaluated_depsgraph_get()
    mesh = bpy.data.meshes.new_from_object(source.evaluated_get(depsgraph), preserve_all_data_layers=True, depsgraph=depsgraph)
    bpy.data.objects.remove(source, do_unlink=True)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _manual_bevel(name, base, segments):
    bm = BASES[base]()
    sharp = [edge for edge in bm.edges if len(edge.link_faces) == 2 and edge.calc_face_angle(0.0) > math.radians(30.0)]
    bmesh.ops.bevel(bm, geom=sharp, offset=WIDTH, offset_type="OFFSET", segments=segments, profile=0.5, affect="EDGES",
                    clamp_overlap=True, material=1, loop_slide=True, face_strength_mode="NONE")
    return _object(name, bm)


def _enter_face_selection(obj, faces):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, False, True)
    bm = bmesh.from_edit_mesh(obj.data)
    for element in (*bm.faces, *bm.edges, *bm.verts):
        element.select = False
    for face in bm.faces:
        if face.index in faces:
            face.select = True
    bm.select_flush_mode()
    bmesh.update_edit_mesh(obj.data)


def _mesh_state(obj):
    bm = bmesh.from_edit_mesh(obj.data)
    from cftuv.analysis_surface import source_revision_from_bmesh

    bm.faces.ensure_lookup_table()
    digest = source_revision_from_bmesh(bm, obj, tuple(face.index for face in bm.faces)).digest
    return digest, sum(1 for edge in bm.edges if edge.seam), frozenset(face.index for face in bm.faces if face.select)


def _decal_digest(name):
    decal = bpy.data.objects.get(name + ".CFTUV_Decal")
    assert decal is not None, name
    mesh = decal.data
    digest = hashlib.sha256()
    for vertex in mesh.vertices:
        digest.update(repr(tuple(round(value, 9) for value in vertex.co)).encode())
    for polygon in mesh.polygons:
        digest.update(repr(tuple(polygon.vertices)).encode())
    for loop in mesh.uv_layers.active.data:
        digest.update(repr(tuple(round(value, 9) for value in loop.uv)).encode())
    return digest.hexdigest(), len(mesh.polygons)


def _press(obj, side):
    _decal_settings().face_side = side
    started = time.perf_counter()
    result = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    seconds = time.perf_counter() - started
    status = _decal_settings().status
    match = re.match(r"MATERIALIZED (\d+) / refused (\d+)", status)
    assert "FINISHED" in result and match, (obj.name, side, result, status)
    return int(match.group(1)), int(match.group(2)), status, seconds


def _run_case(name, obj, expect_source):
    from cftuv.envelope_width_live import _current_digest, width_problem

    from cftuv.bevel_faces_blender import plan_face_build

    bevel = frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)
    _enter_face_selection(obj, bevel)
    events = dict(plan_face_build(bmesh.from_edit_mesh(obj.data), obj, "INWARD").plan.events)
    assert "BEVEL_CLASS_DISAGREES" not in events, (name, events)  # с атрибутом геометрия считается тоже и согласна
    before = _mesh_state(obj)
    assert before[1] == 0 and before[2] == bevel
    built, refused, status, seconds = _press(obj, "INWARD")
    record = _controller().width_build
    patches = len(record.analysis_bundle.patch_graph.nodes)
    analysed = {int(face) for face in record.analysis_bundle.patch_graph.face_to_patch}
    assert analysed == set(bevel), (name, "inward analyses only the selected faces")
    assert built == patches and refused == 0, (name, status)
    assert f"({expect_source})" in status and "BEVEL_SPLIT_UNRESOLVED" not in status, (name, status)
    nodes = int(re.search(r"(\d+) node", status).group(1))
    assert nodes > 0, (name, status)
    assert record.mesh_digest == before[0] == _current_digest(obj) and record.source_digest != before[0]
    assert width_problem(bpy.context) == "", (name, width_problem(bpy.context))  # живая ширина доступна декали нажатия по граням
    assert _mesh_state(obj) == before, (name, "the owner's mesh, seams and face selection are untouched")
    inward = (built, refused, round(seconds, 2), _decal_digest(name)[1])
    built, refused, status, seconds = _press(obj, "OUTWARD")
    record = _controller().width_build
    analysed = {int(face) for face in record.analysis_bundle.patch_graph.face_to_patch}
    assert analysed == {polygon.index for polygon in obj.data.polygons} - set(bevel), (name, "outward analyses only the faces around")
    assert built == len(record.analysis_bundle.patch_graph.nodes) and refused == 0, (name, status)
    assert _mesh_state(obj) == before
    SUMMARY.append(f"{name}: inward {inward[0]}/{inward[0] + inward[1]} built ({nodes} nodes) {inward[2]} s {inward[3]} faces | "
                   f"outward {built}/{built + refused} built {seconds:.2f} s {_decal_digest(name)[1]} faces")
    bpy.ops.object.mode_set(mode="OBJECT")


def _run_synthetic_cases():
    for base in BASES:
        for segments in (1, 2, 4):
            for face_strength in (True, False):
                _clean()
                name = f"{base}_s{segments}_{'fs' if face_strength else 'geo'}"
                obj = _modifier_bevel(name, base, segments, face_strength)
                _run_case(name, obj, "BEVEL_SPLIT_FACE_STRENGTH" if face_strength else "BEVEL_SPLIT_GEOMETRIC")
    for base in BASES:
        for segments in (1, 2):
            _clean()
            name = f"{base}_s{segments}_manual"
            _run_case(name, _manual_bevel(name, base, segments), "BEVEL_SPLIT_GEOMETRIC")


def _run_a_triangulated_bevel_needs_the_attribute():
    """Геометрия слепа на треугольниках: без атрибута - именованный отказ `BEVEL_SPLIT_UNRESOLVED`, с атрибутом (переживает Triangulate) - строится."""

    for face_strength in (False, True):
        _clean()
        name = f"tri_cube_{face_strength}"
        obj = _modifier_bevel(name, "cube", 2, face_strength)
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bmesh.ops.triangulate(bm, faces=[face for face in bm.faces if face.material_index == 1])
        bm.to_mesh(obj.data)
        bm.free()
        bevel = {polygon.index for polygon in obj.data.polygons if polygon.material_index == 1}
        _enter_face_selection(obj, bevel)
        _decal_settings().face_side = "INWARD"
        if face_strength:
            built, refused, status, _seconds = _press(obj, "INWARD")
            assert refused == 0 and built == len(_controller().width_build.analysis_bundle.patch_graph.nodes), status
            assert "BEVEL_SPLIT_FACE_STRENGTH" in status
        else:
            try:
                bpy.ops.hotspotuv.build_envelope_decal_mesh()
                raise AssertionError("a triangulated bevel without Face Strength must be refused")
            except RuntimeError as exc:
                assert "BEVEL_SPLIT_UNRESOLVED" in str(exc), exc
            assert _decal_settings().status == "Failed: BEVEL_SPLIT_UNRESOLVED", _decal_settings().status
            assert bpy.data.objects.get(name + ".CFTUV_Decal") is None
        bpy.ops.object.mode_set(mode="OBJECT")


def _pump_width_live(timeout=180.0):
    scheduler = _controller().width_live
    assert scheduler is not None, "the width field ordered no exact refresh"
    callback, end = scheduler._callback, time.perf_counter() + timeout
    while True:
        delay = callback()
        if delay is None:
            if bpy.app.timers.is_registered(callback):
                bpy.app.timers.unregister(callback)
            return scheduler
        assert time.perf_counter() < end, "width live did not settle"
        time.sleep(delay)


def _run_the_live_width_of_a_face_build_equals_a_press_at_that_width():
    """Поле ширины после нажатия по граням: точный пересчёт тем же анализом граней, меш побитово как у нажатия на новой ширине."""

    from cftuv.envelope_production_mesh import mesh_content_digest

    _clean()
    obj = _modifier_bevel("live_cube", "cube", 2, True)
    bevel = {polygon.index for polygon in obj.data.polygons if polygon.material_index == 1}
    _enter_face_selection(obj, bevel)
    _press(obj, "INWARD")
    _settings().envelope_debug_alpha = ALPHA * 1.25
    scheduler = _pump_width_live()
    assert scheduler.counters.applied == 1 and scheduler.counters.failed == 0, scheduler.counters
    live = mesh_content_digest(bpy.data.objects["live_cube.CFTUV_Decal"].data)
    _controller().clear()
    _enter_face_selection(obj, bevel)
    _press(obj, "INWARD")
    assert mesh_content_digest(bpy.data.objects["live_cube.CFTUV_Decal"].data) == live
    _settings().envelope_debug_alpha = ALPHA
    bpy.ops.object.mode_set(mode="OBJECT")


def _two_patch_fold():
    """Пол и стена через шов (путь по рёбрам): эталон «тот же меш» до и после нажатий по граням."""

    bm = _from_pydata(((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1)), ((0, 1, 2, 3), (0, 3, 5, 4)))
    obj = _object("BevelFold", bm)
    for edge in obj.data.edges:
        edge.use_seam = set(edge.vertices) == {0, 3}
    return obj


def _edge_press_digest():
    obj = bpy.data.objects.get("BevelFold") or _two_patch_fold()
    seam = [edge.index for edge in obj.data.edges if edge.use_seam]
    _enter_edge_selection(obj, seam)
    result = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    assert "FINISHED" in result, _decal_settings().status
    digest = _decal_digest("BevelFold")
    bpy.ops.object.mode_set(mode="OBJECT")
    return digest


def _run_edge_path_is_not_touched_by_face_presses():
    _clean()
    fresh = _edge_press_digest()
    obj = _modifier_bevel("cube_mixed", "cube", 2, True)
    _enter_face_selection(obj, {polygon.index for polygon in obj.data.polygons if polygon.material_index == 1})
    _press(obj, "INWARD")
    bpy.ops.object.mode_set(mode="OBJECT")
    assert _edge_press_digest() == fresh
    _clean()
    assert _edge_press_digest() == fresh


def _run_closed_mesh_without_seams_is_a_named_refusal():
    _clean()
    obj = _object("ClosedCube", _cube())
    _enter_edge_selection(obj, range(len(obj.data.edges)))
    try:
        bpy.ops.hotspotuv.build_envelope_decal_mesh()
        raise AssertionError("a closed mesh without seams must be refused")
    except RuntimeError as exc:  # отчёт ERROR оператора - исключение вызова в фоновом Blender
        assert "no BoundaryChain records" in str(exc), exc
    assert _decal_settings().status == "Failed: SOURCE_HAS_NO_BOUNDARY_CHAIN", _decal_settings().status
    bpy.ops.object.mode_set(mode="OBJECT")


def _noisy_wall(noise, arcs=16, rows=3, r_in=3.9, r_out=4.3, height=4.5):
    """Четверть кольца стены: верх - треугольники с шумом высоты `noise` (сид постоянный), бевел обода верха, вертикальных рёбер
    торцов и границы низа в 2 сегмента (материал 1). Грани бевела - в конце нумерации: у меша без них те же номера граней, рёбер, вершин."""

    rng = random.Random(7)
    bm = bmesh.new()
    radii = [r_in + (r_out - r_in) * k / rows for k in range(rows + 1)]
    top, bottom = [], []
    for i in range(arcs + 1):
        cos, sin = math.cos(0.5 * math.pi * i / arcs), math.sin(0.5 * math.pi * i / arcs)
        top.append([bm.verts.new((r * cos, r * sin, height + rng.uniform(-noise, noise))) for r in radii])
        bottom.append([bm.verts.new((r * cos, r * sin, 0.0)) for r in (r_in, r_out)])
    top_faces, bottom_faces = [], []
    for i in range(arcs):
        for k in range(rows):
            a, b, c, d = top[i][k], top[i][k + 1], top[i + 1][k + 1], top[i + 1][k]
            top_faces += [bm.faces.new(item) for item in (((a, b, c), (a, c, d)) if (i + k) % 2 else ((a, b, d), (b, c, d)))]
        bottom_faces.append(bm.faces.new((bottom[i][0], bottom[i + 1][0], bottom[i + 1][1], bottom[i][1])))
        bm.faces.new((top[i][0], top[i + 1][0], bottom[i + 1][0], bottom[i][0]))
        bm.faces.new((top[i][rows], bottom[i][1], bottom[i + 1][1], top[i + 1][rows]))
    for i in (0, arcs):
        ring = [bottom[i][0], bottom[i][1], *reversed(top[i])]
        bm.faces.new(ring if i == arcs else list(reversed(ring)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    vertical = {frozenset((top[i][0], bottom[i][0])) for i in (0, arcs)} | {frozenset((top[i][rows], bottom[i][1])) for i in (0, arcs)}
    sharp = [
        edge for edge in bm.edges
        if sum(1 for face in edge.link_faces if face in top_faces) == 1
        or sum(1 for face in edge.link_faces if face in bottom_faces) == 1
        or frozenset(edge.verts) in vertical
    ]
    bmesh.ops.bevel(bm, geom=sharp, offset=0.05, offset_type="OFFSET", segments=2, profile=0.5, affect="EDGES",
                    clamp_overlap=True, material=1, loop_slide=True, face_strength_mode="NONE")
    bm.faces.sort(key=lambda face: face.material_index)
    return bm


def _upward_decal(name):
    """Грани декали, смотрящие вверх (декаль верха), всего граней и отпечаток содержимого меша декали."""

    from cftuv.envelope_production_mesh import mesh_content_digest

    decal = bpy.data.objects.get(name + ".CFTUV_Decal")
    assert decal is not None, name
    mesh = decal.data
    return sum(1 for polygon in mesh.polygons if polygon.normal.z > 0.7), len(mesh.polygons), mesh_content_digest(mesh)


def _press_counted(obj, side):
    """Нажатие: (строка статуса без сводки граней, построено, отказано); каждый патч анализа - домен со своим исходом."""

    built, refused, status, _seconds = _press(obj, side)
    assert built + refused == len(_controller().width_build.analysis_bundle.patch_graph.nodes), (obj.name, side, status)
    return status.split(" | ")[0], built, refused


def _run_outward_is_the_edge_path_without_the_bevel_faces():
    """Поле (`rounded_wall_noise_top.001`, Outward): декали стен есть, верха нет. Строка `MATERIALIZED 4 / refused 1
    (DEVELOPABLE_CHART_SELF_OVERLAP)` называла отказ верха, а нажатие по рёбрам на том же верху без бевела отказывает тоже: это
    предел развёртки ядра на шумном верху, а не Outward. Здесь исполняется модель владельца: OUTWARD = обычная процедура на меше,
    где граней бевела нет (тот же меш с удалёнными гранями бевела, источники - его открытая граница, то же имя объекта)."""

    for noise, top_builds in ((0.04, True), (0.12, None)):
        _clean()
        name = f"noisy_wall_{round(noise * 100)}"
        obj = _object(name, _noisy_wall(noise))
        bevel = frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)
        _enter_face_selection(obj, bevel)
        outward = (*_press_counted(obj, "OUTWARD"), *_upward_decal(name))
        bpy.ops.object.mode_set(mode="OBJECT")
        bpy.data.objects.remove(bpy.data.objects[name + ".CFTUV_Decal"], do_unlink=True)
        obj.name = name + "_faces"
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bmesh.ops.delete(bm, geom=[face for face in bm.faces if face.material_index == 1], context="FACES_ONLY")
        border = [edge.index for edge in bm.edges if len(edge.link_faces) == 1]
        control = _object(name, bm)
        _enter_edge_selection(control, border)
        edges = (*_press_counted(control, "OUTWARD"), *_upward_decal(name))
        bpy.ops.object.mode_set(mode="OBJECT")
        assert outward == edges, (name, "outward is the edge path on the mesh without the bevel faces", outward, edges)
        status, built, refused, upward = outward[:4]
        if top_builds:
            assert refused == 0 and upward > 0, (name, status)
        elif upward == 0:
            assert refused > 0 and re.search(r"refused \d+ \([A-Z_]+", status), (name, "the top domain is refused by name", status)
        SUMMARY.append(f"{name}: outward = edge path without the bevel: {status}, {upward} top decal faces")


class _Layout:
    """Заглушка `UILayout`: любой вызов - заглушка, `prop` записывает имя свойства."""

    def __init__(self, log):
        self.log = log

    def __getattr__(self, name):
        def call(*args, **_kwargs):
            if name == "prop" and len(args) > 1:
                self.log.append(args[1])
            return _Layout(self.log)

        return call


def _run_the_face_side_row_is_drawn_only_in_face_select_mode():
    from cftuv.envelope_debug_panel import draw_decal_mesh_rows

    _clean()
    obj = _modifier_bevel("panel_cube", "cube", 1, True)
    _enter_face_selection(obj, {0})
    drawn = []
    draw_decal_mesh_rows(_Layout(drawn))
    assert "face_side" in drawn, drawn
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    drawn.clear()
    draw_decal_mesh_rows(_Layout(drawn))
    assert "face_side" not in drawn, drawn
    bpy.ops.object.mode_set(mode="OBJECT")


def _main():
    import cftuv

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован
        pass
    settings = _settings()
    settings.envelope_debug_alpha = ALPHA
    settings.envelope_debug_workers = 0
    _run_the_face_side_row_is_drawn_only_in_face_select_mode()
    _run_closed_mesh_without_seams_is_a_named_refusal()
    _run_a_triangulated_bevel_needs_the_attribute()
    _run_the_live_width_of_a_face_build_equals_a_press_at_that_width()
    _run_edge_path_is_not_touched_by_face_presses()
    _run_outward_is_the_edge_path_without_the_bevel_faces()
    _run_synthetic_cases()
    for line in SUMMARY:
        print("[BevelFacesTest]", line, flush=True)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("BEVEL_FACES_BUILD_BLENDER_OK", flush=True)


if __name__ == "__main__":
    _main()
