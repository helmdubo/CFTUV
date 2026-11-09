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

10. BOTH (по умолчанию, BEVEL_FACES_BOTH_V3, BEVEL_WINGS_V3): куб с бевелом в 1, 2 и 3 сегмента, узлы в углах; один объект декали; грани
    копии - ровно выделенные грани (те же кольца вершин с тем же обходом при смещении 0, то же число граней, вершин и петель), внешняя часть -
    все домены анализа невыделенных граней; каждая вершина копии на границе выделения - ОДНА вершина с внешней декалью, ни одной пары
    вершин в одной точке, ни одного открытого ребра копии; при смещении декали внутренняя вершина копии смещена вдоль своей нормали (сумма
    нормалей граней, взвешенных площадью) ровно на смещение, вершина рельса - митра (вдоль нормали копии то же смещение); UV СЕГМЕНТА: один
    остров UV на сегмент (полоса между узлами с юбками декали с обоих рельсов), острова = сегменты, узел - весь в острове одного сегмента;
    на каждом ребре рельса внутри сегмента UV копии и юбки равны (разрыв 0), шов меша - ровно рёбра, где UV расходится; рельсы полосы зеркальны
    около `V = 0.5` при той же `U`; V ровно [0, 1] (края юбок полной ширины - 0 и 1); ни одна грань UV не вывернута; одна плотность (разброс
    по граням при смещении 0 назван числом); кадр модели живой ширины не двигает копию и держит рельс сваренным, точный пересчёт не двигает
    копию и побитово равен нажатию на новой ширине; плита с отверстием и L-профиль - острова = сегменты, веера вогнутых углов раскрыты от
    юбки своего сегмента; стена-колонка (четверть кольца, стены и торцы поделены по высоте, шумный верх): грань
    юбки ядра над колонкой торца одна на всю высоту, а у рельса копии вершины на каждом уровне - они встают в ребро юбки (T-стык,
    BEVEL_COPY_RAIL_SPLIT_V3), открытых рёбер копии нет; строка частей «copy N faces, outward a/b refused (...)» называет отказ внешней части
    (шумный верх стены) и вершины рельса без пары; выделение всех граней - именованный отказ `BEVEL_FACES_NOTHING_OUTSIDE`.

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


def _noisy_wall(noise, arcs=16, rows=3, r_in=3.9, r_out=4.3, height=4.5, levels=1):
    """Четверть кольца стены: верх - треугольники с шумом высоты `noise` (сид постоянный), бевел обода верха, вертикальных рёбер
    торцов и границы низа в 2 сегмента (материал 1). Грани бевела - в конце нумерации: у меша без них те же номера граней, рёбер, вершин.
    `levels` - стены и торцы поделены по высоте на столько рядов (вертикальное ребро торца - цепь рёбер: у колонки бевела вершины на
    каждом уровне, а у плоского торца ядро оставляет только вершины силуэта)."""

    rng = random.Random(7)
    bm = bmesh.new()
    radii = [r_in + (r_out - r_in) * k / rows for k in range(rows + 1)]
    top, bottom, column = [], [], []
    for i in range(arcs + 1):
        cos, sin = math.cos(0.5 * math.pi * i / arcs), math.sin(0.5 * math.pi * i / arcs)
        top.append([bm.verts.new((r * cos, r * sin, height + rng.uniform(-noise, noise))) for r in radii])
        bottom.append([bm.verts.new((r * cos, r * sin, 0.0)) for r in (r_in, r_out)])
        middle = [[bm.verts.new((r * cos, r * sin, height * level / levels)) for level in range(1, levels)] for r in (r_in, r_out)]
        column.append([[bottom[i][side], *middle[side], top[i][0 if side == 0 else rows]] for side in (0, 1)])
    top_faces, bottom_faces = [], []
    for i in range(arcs):
        for k in range(rows):
            a, b, c, d = top[i][k], top[i][k + 1], top[i + 1][k + 1], top[i + 1][k]
            top_faces += [bm.faces.new(item) for item in (((a, b, c), (a, c, d)) if (i + k) % 2 else ((a, b, d), (b, c, d)))]
        bottom_faces.append(bm.faces.new((bottom[i][0], bottom[i + 1][0], bottom[i + 1][1], bottom[i][1])))
        for level in range(levels):
            inner, outer = column[i][0], column[i + 1][0]
            bm.faces.new((inner[level + 1], outer[level + 1], outer[level], inner[level]))
            inner, outer = column[i][1], column[i + 1][1]
            bm.faces.new((inner[level + 1], inner[level], outer[level], outer[level + 1]))
    for i in (0, arcs):
        ring = [*column[i][0][:1], *column[i][1][:-1], *reversed(top[i]), *reversed(column[i][0][1:-1])]
        bm.faces.new(ring if i == arcs else list(reversed(ring)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    vertical = {frozenset(pair) for i in (0, arcs) for side in (0, 1) for pair in zip(column[i][side], column[i][side][1:])}
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


BOTH_ALPHA = 0.1


def _copy_domain(mesh):
    import json

    record = mesh.get("cftuv_bevel_wings")
    assert record is not None, (mesh.name, "the Both mesh carries its wing record")
    return int(json.loads(str(record))["copy"])


def _parts(mesh, copy):
    """`(домен грани, грани копии, грани снаружи, вершины копии, вершины снаружи)` меша декали Both."""

    domain = [item.value for item in mesh.attributes["cftuv_domain"].data]
    copy_faces = [polygon for polygon in mesh.polygons if domain[polygon.index] == copy]
    outer_faces = [polygon for polygon in mesh.polygons if domain[polygon.index] != copy]
    return (domain, copy_faces, outer_faces, {v for polygon in copy_faces for v in polygon.vertices}, {v for polygon in outer_faces for v in polygon.vertices})


def _ring(cycle):
    """Кольцо вершин без выбора начала: наименьший поворот."""

    cycle = tuple(cycle)
    return min(cycle[at:] + cycle[:at] for at in range(len(cycle)))


def _uv_face_area(uv, polygon):
    points = [uv[loop].uv for loop in polygon.loop_indices]
    return sum(points[i].x * points[(i + 1) % len(points)].y - points[(i + 1) % len(points)].x * points[i].y for i in range(len(points)))


def _copy_normals(obj, bevel):
    """Закон смещения копии (BEVEL_COPY_OFFSET_V2): нормаль вершины - сумма нормалей её граней копии, взвешенных площадью."""

    from mathutils import Vector

    sums = {}
    for index in bevel:
        polygon = obj.data.polygons[index]
        for vertex in polygon.vertices:
            sums[vertex] = sums.get(vertex, Vector((0.0, 0.0, 0.0))) + polygon.normal * polygon.area
    return {vertex: value.normalized() for vertex, value in sums.items()}


def _islands(mesh, uv, loop_at, edge_faces):
    """Острова UV: грани, связанные рёбрами с непрерывной UV."""

    parent = list(range(len(mesh.polygons)))

    def find(item):
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    for (a, b), faces in edge_faces.items():
        if len(faces) == 2 and max((uv[loop_at[(faces[0], x)]].uv - uv[loop_at[(faces[1], x)]].uv).length for x in (a, b)) <= 1e-6:
            parent[find(faces[0])] = find(faces[1])
    found = {}
    for polygon in mesh.polygons:
        found.setdefault(find(polygon.index), []).append(polygon.index)
    return list(found.values())


def _uv_checks(name, mesh, copy, strict_islands=True, trim_tolerance=0.0):
    """UV сегментов (BEVEL_WINGS_V3): остров на сегмент, узел в одном сегменте, непрерывность через рельс внутри сегмента, швы - ровно разрывы UV,
    рельсы полосы зеркальны около средней линии (`V = 0.5`) при той же `U`, ни одна грань не вывернута, V в [0, 1] (шумный профиль шире полосы
    трима и узлы - в `trim_tolerance`), плотность по граням (разброс назван)."""

    domain, copy_faces, _outer, copy_vertices, outer_vertices = _parts(mesh, copy)
    uv = mesh.uv_layers["UVMap"].data
    owner = [item.value for item in mesh.attributes["cftuv_owner"].data]
    seams = {tuple(sorted(edge.vertices)) for edge in mesh.edges if edge.use_seam}
    loop_at, edge_faces = {}, {}
    for polygon in mesh.polygons:
        for loop in polygon.loop_indices:
            loop_at[(polygon.index, mesh.loops[loop].vertex_index)] = loop
        for key in polygon.edge_keys:
            edge_faces.setdefault(key, []).append(polygon.index)
    rail_gap, rail_edges, segment_seams, seam_mismatch = 0.0, 0, 0, 0
    for (a, b), faces in edge_faces.items():
        if len(faces) != 2:
            continue
        gap = max((uv[loop_at[(faces[0], x)]].uv - uv[loop_at[(faces[1], x)]].uv).length for x in (a, b))
        seam_mismatch += (gap > 1e-5) != ((a, b) in seams)
        if (domain[faces[0]] == copy) != (domain[faces[1]] == copy):
            if (a, b) in seams:
                segment_seams += 1
            else:
                rail_gap, rail_edges = max(rail_gap, gap), rail_edges + 1
    assert rail_edges > 0 and rail_gap <= 1e-6, (name, "UV is continuous across the rail inside a segment", rail_edges, rail_gap)
    assert seam_mismatch == 0, (name, "the seams are exactly the edges where UV breaks", seam_mismatch)
    islands = _islands(mesh, uv, loop_at, edge_faces)
    segments = {owner[polygon.index] // 2 for polygon in copy_faces}
    for faces in islands:
        mine = {owner[face] // 2 for face in faces if domain[face] == copy}
        assert len(mine) <= 1, (name, "an island carries one segment", mine)
    with_copy = sum(1 for faces in islands if any(domain[face] == copy for face in faces))
    assert with_copy == len(segments), (name, "every segment is one island", with_copy, len(segments))
    if strict_islands:
        assert len(islands) == len(segments), (name, "islands = segments", len(islands), len(segments))
    node_faces = [polygon for polygon in copy_faces if owner[polygon.index] % 2 == 1]
    for polygon in node_faces:  # узел целиком в одном сегменте: соседи-узлы по ребру - тот же сегмент
        for key in polygon.edge_keys:
            for other in edge_faces[key]:
                if other != polygon.index and domain[other] == copy and owner[other] % 2 == 1:
                    assert owner[other] // 2 == owner[polygon.index] // 2, (name, "a node is attached to one segment", polygon.index, other)
    rail = copy_vertices & outer_vertices
    by_segment = {}
    for polygon in copy_faces:
        if owner[polygon.index] % 2 == 0:
            for loop in polygon.loop_indices:
                if mesh.loops[loop].vertex_index in rail:
                    by_segment.setdefault(owner[polygon.index] // 2, set()).add((round(uv[loop].uv.x, 5), round(uv[loop].uv.y - 0.5, 5)))
    mirror = 0.0
    for points in by_segment.values():
        for u, h in points:
            mirror = max(mirror, min(abs(u - other_u) + abs(h + other_h) for other_u, other_h in points))
    flipped = sum(1 for polygon in mesh.polygons if _uv_face_area(uv, polygon) < 0.0)
    assert flipped == 0, (name, "no UV face is flipped", flipped)
    heights = [loop.uv.y for loop in uv]
    assert -trim_tolerance - 1e-6 <= min(heights) and max(heights) <= 1.0 + trim_tolerance + 1e-6, (name, "V in the trim [0, 1]", min(heights), max(heights))
    density, node_density = [], []  # узел - кусок двоякой кривизны, его развёртка подобием по граням меняет площадь: разброс узлов назван отдельно
    for polygon in mesh.polygons:
        if polygon.area > 1e-9:
            node = domain[polygon.index] == copy and owner[polygon.index] % 2 == 1
            (node_density if node else density).append(math.sqrt(abs(_uv_face_area(uv, polygon)) / 2.0 / polygon.area))
    density.sort()
    median = density[len(density) // 2]
    node_spread = (max(node_density) - min(node_density)) / median if node_density else 0.0
    return {
        "rail_edges": rail_edges, "rail_gap": rail_gap, "segment_seams": segment_seams, "mirror": mirror, "islands": len(islands),
        "segments": len(segments), "heights": (min(heights), max(heights)), "density": median, "spread": (density[-1] - density[0]) / median,
        "node_spread": node_spread,
        "open_copy": sum(1 for key, faces in edge_faces.items() if len(faces) == 1 and domain[faces[0]] == copy),
    }


def _both_checks(name, obj, bevel, offset):
    """Копия граней бевела и сварка меша декали Both: топология копии - топология выделенных граней, смещение, одна вершина на рельсе."""

    mesh = bpy.data.objects[name + ".CFTUV_Decal"].data
    copy = _copy_domain(mesh)
    _domain, copy_faces, _outer_faces, copy_vertices, outer_vertices = _parts(mesh, copy)
    source = obj.data
    selected_vertices = {vertex for index in bevel for vertex in source.polygons[index].vertices}
    boundary = {vertex for polygon in source.polygons if polygon.index not in bevel for vertex in polygon.vertices} & selected_vertices
    rail = copy_vertices & outer_vertices
    assert len(copy_faces) == len(bevel) and len(copy_vertices) == len(selected_vertices), (name, "the copy has the faces and vertices of the selection", len(copy_faces), len(copy_vertices))
    assert sum(polygon.loop_total for polygon in copy_faces) == sum(source.polygons[index].loop_total for index in bevel), (name, "the copy has the loops of the selection")
    assert len(rail) == len(boundary), (name, "every boundary vertex of the copy is one vertex with the outward decal", len(rail), len(boundary))
    points = {}
    for vertex in mesh.vertices:
        points.setdefault(tuple(round(value, 6) for value in vertex.co), []).append(vertex.index)
    doubled = sum(len(items) - 1 for items in points.values())
    assert doubled == 0, (name, "no two decal vertices at one point", doubled)
    normals = _copy_normals(obj, bevel)
    expected = {item: source.vertices[item].co + normals[item] * offset for item in selected_vertices}
    nearest = {}
    for vertex in copy_vertices:
        co = mesh.vertices[vertex].co
        nearest[vertex] = min(selected_vertices, key=lambda item: ((expected[item] - co).length, item))
        assert (expected[nearest[vertex]] - co).length <= 4.0 * offset + 1e-6, (name, "a copy vertex lies over a selected vertex", vertex)
    if offset == 0.0:
        assert len(set(nearest.values())) == len(copy_vertices), (name, "every copy vertex lies on its own source vertex at offset 0")
        rings = sorted(_ring(nearest[vertex] for vertex in polygon.vertices) for polygon in copy_faces)
        assert rings == sorted(_ring(source.polygons[index].vertices) for index in bevel), (name, "the copy faces are the selected faces, same vertices, same winding")
        return {"faces": len(copy_faces), "vertices": len(copy_vertices), "rail": len(rail), "doubled": doubled}
    interior = rail_error = 0.0
    for vertex in copy_vertices:
        shift = mesh.vertices[vertex].co - source.vertices[nearest[vertex]].co
        if vertex in rail:
            rail_error = max(rail_error, abs(shift.dot(normals[nearest[vertex]]) - offset))  # митра: вдоль нормали копии смещение то же
        else:
            interior = max(interior, (shift - normals[nearest[vertex]] * offset).length)
    assert interior <= 1e-6, (name, "an interior copy vertex is offset along its normal by the decal offset", interior)
    assert rail_error <= 1e-6, (name, "a rail vertex is mitred: its offset along the copy normal is the decal offset", rail_error)
    return {"faces": len(copy_faces), "vertices": len(copy_vertices), "rail": len(rail), "doubled": doubled, "interior": interior, "rail_error": rail_error}


def _width_problem_text():
    from cftuv.envelope_width_live import width_problem

    return width_problem(bpy.context)


def _press_both(name, segments=2, offset=None):
    obj = _modifier_bevel(name, "cube", segments, True)
    bevel = frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)
    _enter_face_selection(obj, bevel)
    default_offset = _decal_settings().offset
    if offset is not None:
        _decal_settings().offset = offset
    try:
        pressed = _press(obj, "BOTH")
    finally:
        _decal_settings().offset = default_offset
    return obj, bevel, pressed


BOTH_PARTS = re.compile(r"both: copy (\d+) faces, outward (\d+)/(\d+)(?: refused \(([^)]*)\))?")


def _run_both_is_the_default_one_welded_mesh_of_the_copy_and_the_outward_decal():
    """BOTH: одно нажатие, один меш - копия граней бевела (их топология, смещение декали) и декаль наружу, сваренные на рельсе; крылья UV."""

    from cftuv.bevel_wings_blender import LAST_COUNTERS

    _clean()
    face_side = _decal_settings().bl_rna.properties["face_side"]
    assert face_side.default == "BOTH" and [item.identifier for item in face_side.enum_items] == ["BOTH", "INWARD", "OUTWARD"], "Both is the default face side"
    for segments in (1, 2, 3):
        for offset in (0.0, None):
            _clean()
            _settings().envelope_debug_alpha = BOTH_ALPHA
            name = f"both_cube{segments}"
            obj, bevel, (built, refused, status, seconds) = _press_both(name, segments, offset)
            patches = _controller().width_build.analysis_bundle.patch_graph.nodes
            parts = BOTH_PARTS.search(status)
            assert parts and parts.groups()[:3] == (str(len(bevel)), str(len(patches)), str(len(patches))), (name, status)
            assert built == len(patches) and refused == 0 and "BEVEL_COPY_RAIL_UNPAIRED" not in status and "BEVEL_COPY_RAIL_SPLIT" not in status, (name, status)
            assert "ADAPTER_WELD" not in status, (name, "the weld names nothing", status)
            assert [item.name for item in bpy.data.objects if item.name.startswith(name + ".")] == [name + ".CFTUV_Decal"], "one decal mesh"
            assert _width_problem_text() == "", _width_problem_text()
            mesh = bpy.data.objects[name + ".CFTUV_Decal"].data
            counters = LAST_COUNTERS[mesh.session_uid]
            for key in ("BEVEL_WINGS_OUTER_UNPAIRED", "BEVEL_WINGS_NOT_TWO_SIDED", "BEVEL_WINGS_OUTER_MIRRORED", "BEVEL_WINGS_FOLDS", "BEVEL_WINGS_COPY_FLIPPED",
                        "BEVEL_WINGS_KNOT_BREAKS", "BEVEL_WINGS_KNOT_CONFLICTS", "BEVEL_WINGS_KERNEL_FLIPPED"):
                assert counters[key] == 0, (key, counters)
            assert "BEVEL_WINGS_ALPHA_UNKNOWN" not in counters and counters["BEVEL_WINGS_ALPHA_SPREAD_PPM"] <= 1000, counters
            numbers = _both_checks(name, obj, bevel, 0.0 if offset == 0.0 else _decal_settings().offset)
            uv = _uv_checks(name, mesh, _copy_domain(mesh))
            (low, high) = uv["heights"]
            assert abs(low) <= 1e-6 and abs(high - 1.0) <= 1e-6, (name, "the segments span the trim [0, 1]", low, high)
            assert uv["mirror"] <= 2e-5 and uv["open_copy"] == 0, (name, uv)
            if offset == 0.0:
                assert uv["spread"] <= 0.02, (name, "one texel density for the whole press", uv["spread"])
                flat_spread, node_spread = uv["spread"], uv["node_spread"]
            if offset is None:
                SUMMARY.append(
                    f"{name}: {status.split(' | ')[-1]} {seconds:.2f} s, {len(mesh.polygons)} faces {len(mesh.vertices)} vertices, "
                    f"{numbers['rail']} rail vertices welded, {numbers['doubled']} doubled, interior offset error {numbers['interior']:.2g}, "
                    f"rail offset error {numbers['rail_error']:.2g}, {uv['islands']} islands = {uv['segments']} segments, {uv['rail_edges']} rail edges UV gap "
                    f"{uv['rail_gap']:.2g}, {uv['segment_seams']} segment-seam rail edges, mirror {uv['mirror']:.2g}, density {uv['density']:.4g} UV/m "
                    f"spread {flat_spread:.2g} at offset 0 (node faces {node_spread:.2g}), alpha spread {counters['BEVEL_WINGS_ALPHA_SPREAD_PPM']} ppm"
                )
            bpy.ops.object.mode_set(mode="OBJECT")
    _clean()  # ширина возвращается без записи кнопки: живой пересчёт не заказывается
    _settings().envelope_debug_alpha = ALPHA


def _counts(scheduler):
    counters = getattr(scheduler, "counters", None)
    return (0, 0) if counters is None else (counters.applied, counters.failed)


def _copy_positions(mesh):
    _domain, _copy_faces, _outer, copy_vertices, _outer_vertices = _parts(mesh, _copy_domain(mesh))
    return {mesh.vertices[vertex].co.to_tuple() for vertex in copy_vertices}


def _run_the_live_width_of_both_equals_a_press_at_that_width():
    """Живая ширина Both: кадр модели не двигает копию и держит рельс сваренным; точный пересчёт не двигает копию и побитово равен нажатию на новой ширине."""

    from cftuv.envelope_production_mesh import mesh_content_digest
    from cftuv.envelope_width_mesh_preview import preview_mesh_now

    _clean()
    _settings().envelope_debug_alpha = BOTH_ALPHA
    obj, bevel, _pressed = _press_both("both_live")
    mesh = bpy.data.objects["both_live.CFTUV_Decal"].data
    pressed_copy = _copy_positions(mesh)
    controller = _controller()
    frame = preview_mesh_now(controller, BOTH_ALPHA * 1.1)
    frame_text = "no frame" if frame is None else frame.outcome
    if frame is not None:
        assert _copy_positions(mesh) == pressed_copy, "the model frame does not move the copy"
        _uv_checks("both_live frame", mesh, _copy_domain(mesh))
        _both_checks("both_live", obj, bevel, _decal_settings().offset)
    applied, failed = _counts(controller.width_live)
    _settings().envelope_debug_alpha = BOTH_ALPHA * 1.25
    scheduler = _pump_width_live()
    assert _counts(scheduler) == (applied + 1, failed), scheduler.counters
    live_mesh = bpy.data.objects["both_live.CFTUV_Decal"].data
    assert live_mesh.get("cftuv_bevel_wings") is not None
    assert _copy_positions(live_mesh) == pressed_copy, "the copy does not depend on the width"
    numbers = _both_checks("both_live", obj, bevel, _decal_settings().offset)
    _uv_checks("both_live", live_mesh, _copy_domain(live_mesh))
    live = mesh_content_digest(live_mesh)
    _controller().clear()
    _enter_face_selection(obj, bevel)
    _press(obj, "BOTH")
    assert mesh_content_digest(bpy.data.objects["both_live.CFTUV_Decal"].data) == live, "live width of Both equals a press at that width"
    SUMMARY.append(f"both_live: model frame {frame_text}, exact live width = press at {BOTH_ALPHA * 1.25:g}, {numbers['rail']} rail vertices welded, copy unchanged")
    bpy.ops.object.mode_set(mode="OBJECT")
    _clean()
    _settings().envelope_debug_alpha = ALPHA


def _run_both_plate_and_l_profile_are_one_island_per_segment():
    """Плита с отверстием и L-профиль (бевел модификатором в 2 сегмента, Face Strength): острова UV = сегменты, веера у вогнутых углов
    раскрыты от юбки своего сегмента (BEVEL_WINGS_FAN_UNFOLD_V3), разрыв на рельсе 0, вывернутых 0, V ровно [0, 1]."""

    from cftuv.bevel_wings_blender import LAST_COUNTERS

    for base in ("plate_hole", "lprofile"):
        _clean()
        _settings().envelope_debug_alpha = BOTH_ALPHA
        name = f"both_{base}"
        obj = _modifier_bevel(name, base, 2, True)
        bevel = frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)
        _enter_face_selection(obj, bevel)
        built, refused, status, seconds = _press(obj, "BOTH")
        assert refused == 0 and "BEVEL_COPY_RAIL_UNPAIRED" not in status, (name, status)
        mesh = bpy.data.objects[name + ".CFTUV_Decal"].data
        counters = LAST_COUNTERS[mesh.session_uid]
        for key in ("BEVEL_WINGS_OUTER_UNPAIRED", "BEVEL_WINGS_FOLDS", "BEVEL_WINGS_COPY_FLIPPED", "BEVEL_WINGS_OUTER_MIRRORED"):
            assert counters[key] == 0, (name, key, counters)
        _both_checks(name, obj, bevel, _decal_settings().offset)
        uv = _uv_checks(name, mesh, _copy_domain(mesh))
        assert uv["open_copy"] == 0 and abs(uv["heights"][0]) <= 1e-6 and abs(uv["heights"][1] - 1.0) <= 1e-6, (name, uv)
        SUMMARY.append(
            f"{name}: {BOTH_PARTS.search(status).group(0)} {seconds:.2f} s, {uv['islands']} islands = {uv['segments']} segments, "
            f"fans unfolded {counters['BEVEL_WINGS_FANS_UNFOLDED']}, rail UV gap {uv['rail_gap']:.2g} on {uv['rail_edges']} edges, "
            f"skirt stretch {counters['BEVEL_WINGS_SKIRT_STRETCH_PPM']} ppm, density spread {uv['spread']:.2g}"
        )
        bpy.ops.object.mode_set(mode="OBJECT")
    _clean()
    _settings().envelope_debug_alpha = ALPHA


def _run_both_welds_the_wall_columns_without_t_junctions():
    """Стена-колонка (поле: торцы стены, щели на рёбрах между копией и декалью): юбка ядра над колонкой торца - одна грань на всю высоту
    (ядро держит вершины силуэта), у рельса копии вершины на каждом уровне. Они встают в ребро юбки (BEVEL_COPY_RAIL_SPLIT_V3): ни одного
    открытого ребра копии, ни одной вершины рельса без пары; UV - острова сегментов, разрыв на рельсе 0, вывернутых 0."""

    from cftuv.bevel_wings_blender import LAST_COUNTERS

    _clean()
    _settings().envelope_debug_alpha = ALPHA
    obj = _object("both_columns", _noisy_wall(0.04, levels=3))
    bevel = frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)
    _enter_face_selection(obj, bevel)
    built, refused, status, seconds = _press(obj, "BOTH")
    parts = BOTH_PARTS.search(status)
    assert parts and refused == 0 and "BEVEL_COPY_RAIL_UNPAIRED" not in status, (status, "every rail vertex of the copy is welded")
    split = re.search(r"BEVEL_COPY_RAIL_SPLIT (\d+)", status)
    assert split and int(split.group(1)) > 0 and "BEVEL_COPY_RAIL_SPLIT_REFUSED" not in status, (status, "the column rail vertices stand in the wing edges")
    mesh = bpy.data.objects["both_columns.CFTUV_Decal"].data
    counters = LAST_COUNTERS[mesh.session_uid]
    numbers = _both_checks("both_columns", obj, bevel, _decal_settings().offset)
    uv = _uv_checks("both_columns", mesh, _copy_domain(mesh), strict_islands=False, trim_tolerance=0.5)
    assert uv["open_copy"] == 0, ("no open copy edge: the T-junctions are welded", uv)
    record = _controller().width_build
    SUMMARY.append(
        f"both_columns: {parts.group(0)} {seconds:.2f} s, {split.group(0)}, {numbers['rail']} rail vertices welded, open copy edges {uv['open_copy']}, "
        f"{uv['islands']} islands for {uv['segments']} segments, rail UV gap {uv['rail_gap']:.2g} on {uv['rail_edges']} edges, "
        f"{uv['segment_seams']} segment-seam rail edges, V {uv['heights'][0]:.3f}..{uv['heights'][1]:.3f}, density spread {uv['spread']:.2g}, "
        f"folds {counters['BEVEL_WINGS_FOLDS']} (kernel {counters['BEVEL_WINGS_KERNEL_FLIPPED']}), node skirts {counters['BEVEL_WINGS_NODE_SKIRTS']}, "
        f"skirt past trim {counters['BEVEL_WINGS_SKIRT_PAST_TRIM']}, "
        f"{len(record.analysis_bundle.patch_graph.nodes)} outward domains"
    )
    bpy.ops.object.mode_set(mode="OBJECT")
    _clean()


def _run_both_names_the_refused_part():
    """Отказ домена внешней части назван в строке частей, копия и остальное в меше (шумный верх стены: верх отказан ядром по имени);
    выделение без граней снаружи - именованный отказ нажатия."""

    _clean()
    _settings().envelope_debug_alpha = ALPHA
    obj = _object("both_noisy", _noisy_wall(0.12))
    bevel = frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)
    _enter_face_selection(obj, bevel)
    built, refused, status, _seconds = _press(obj, "BOTH")
    parts = BOTH_PARTS.search(status)
    assert parts, status
    copy_faces, outward_built, outward_all, named = parts.groups()
    assert int(copy_faces) == len(bevel) and int(outward_built) == built and int(outward_all) == built + refused, status
    assert refused == 0 or named, (status, "a refused outward domain is named")
    assert bpy.data.objects.get("both_noisy.CFTUV_Decal") is not None
    SUMMARY.append(f"both_noisy: {status.split(' | ')[0]} | {parts.group(0)}")
    bpy.ops.object.mode_set(mode="OBJECT")
    _clean()
    whole = _object("both_whole", _cube())
    _enter_face_selection(whole, {polygon.index for polygon in whole.data.polygons})
    _decal_settings().face_side = "BOTH"
    try:
        result = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    except RuntimeError as exc:  # отказ плана - ERROR отчёта оператора
        result = str(exc)
    assert "BEVEL_FACES_NOTHING_OUTSIDE" in _decal_settings().status, (result, _decal_settings().status)
    assert bpy.data.objects.get("both_whole.CFTUV_Decal") is None
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
    _run_both_is_the_default_one_welded_mesh_of_the_copy_and_the_outward_decal()
    _run_the_live_width_of_both_equals_a_press_at_that_width()
    _run_both_plate_and_l_profile_are_one_island_per_segment()
    _run_both_welds_the_wall_columns_without_t_junctions()
    _run_both_names_the_refused_part()
    for line in SUMMARY:
        print("[BevelFacesTest]", line, flush=True)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("BEVEL_FACES_BUILD_BLENDER_OK", flush=True)


if __name__ == "__main__":
    _main()
