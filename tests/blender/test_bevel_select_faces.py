"""Blender 4.5 background: кнопка «Select Bevel Faces» (BEVEL_SELECT_FACES_V1) на синтетике в памяти (ничего не сохраняется).

Правда - материал 1 (его ставит сам бевел; кнопка материал не читает). Меши: куб, L-профиль, плита с отверстием; бевел модификатором
(применён: `new_from_object`) в 1, 2 и 4 сегмента с Face Strength All, New и без него, и ручной `bmesh.ops.bevel`; контроли без бевела
(узкие плоские вставки, тонкая стенка, цилиндр, чистый куб).

1. Face Strength All - путь `BEVEL_SELECT_FACE_STRENGTH`, выделено ровно множество граней бевела (полосы 0, узлы -16384);
2. Face Strength New - путь `BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS` (геометрия с именем, строка называет режим All), узлы WEAK - точные;
3. без атрибута и ручной бевел - `BEVEL_SELECT_GEOMETRIC`; точность и полнота каждого случая печатаются и проверяются;
4. контроли - `BEVEL_SELECT_NONE_FOUND`: выделение и режим выделения не тронуты, оператор отменён (шага отмены нет);
5. замена и добавление (`extend`), режим выделения переключён на грани, меш не изменён (ни одного шва, те же грани);
6. несколько объектов в Edit Mode - каждый получает свои грани, строка отчёта называет каждый;
7. кнопка рисуется в Edit Mode в любом режиме выделения, строка «Faces» - только в режиме граней; оператор с шагом отмены;
8. углы обода у торцов стены с шумным треугольным верхом и своей фаской торца (BEVEL_SELECT_CORNERS_V1, поле
   `rounded_wall_noise_top`): бевел по углу в 1/2/3 сегмента, каждая грань бевела у торцов выделена, по стене - точность и полнота;
9. группа от семян (BEVEL_SELECT_SEED_V1): выделенная грань бевела - семя, кнопка выделяет её группу и ничего на втором кубе того же
   меша (Face Strength All - точно, без атрибута - геометрия), на стене поля - группа без своей фаски торца; семя - плоская грань:
   `BEVEL_SEED_NOT_BEVEL`, оператор отменён, выделение не тронуто. Без выделенных граней (выделено только ребро) - весь меш.
10. «Seam Bevel» (BEVEL_SEAM_MARK_V1): весь бевел двух кубов - рельсы и «узел | полоса» по атрибуту; без атрибута на стене поля
   («Grow Bevel» -> «Seam Bevel») узлы верхних углов торцов изолированы и шва между двумя полосами нет (BEVEL_SEAM_NODE_AGREEMENT_V1).

Прогон: blender -b --factory-startup --python-exit-code 1 --python tests/blender/test_bevel_select_faces.py
Последняя строка при успехе: BEVEL_SELECT_FACES_BLENDER_OK
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_bevel_faces_build import BASES, WIDTH, _clean, _object  # noqa: E402

import bmesh  # noqa: E402

SUMMARY = []


def _modifier_bevel(name, base, segments, mode):
    source = _object(name + "_base", BASES[base]())
    modifier = source.modifiers.new("Bevel", "BEVEL")
    modifier.width, modifier.segments, modifier.limit_method, modifier.material = WIDTH, segments, "ANGLE", 1
    modifier.face_strength_mode = mode
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


def _planar_cuts():
    """Куб, верх которого дважды вставлен узкими плоскими квадами (0.05): узкие грани без изгиба - не бевел."""

    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=2.0)
    for _round in range(2):
        top = max((face for face in bm.faces if face.normal.z > 0.9), key=lambda face: face.calc_area())
        bmesh.ops.inset_region(bm, faces=[top], thickness=0.05, depth=0.0)
    return bm


def _thin_ledge():
    """Стенка 2 x 0.1 x 1: узкие грани с выпуклыми рёбрами по 90 градусов с двух сторон - уступ, а не фаска."""

    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for vertex in bm.verts:
        vertex.co.x *= 2.0
        vertex.co.y *= 0.1
    return bm


def _cylinder():
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=32, radius1=1.0, radius2=1.0, depth=2.0)
    return bm


def _plain_cube():
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=2.0)
    return bm


def _grid_cube_bevel():
    """Куб 2 с сеткой 15 x 15 на грани, ручной бевел 0.03 в три сегмента по острым рёбрам (материал 1)."""

    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=2.0)
    bmesh.ops.subdivide_edges(bm, edges=bm.edges[:], cuts=14, use_grid_fill=True)
    sharp = [edge for edge in bm.edges if len(edge.link_faces) == 2 and edge.calc_face_angle(0.0) > math.radians(30.0)]
    bmesh.ops.bevel(bm, geom=sharp, offset=0.03, offset_type="OFFSET", segments=3, profile=0.5, affect="EDGES",
                    clamp_overlap=True, material=1, loop_slide=True, face_strength_mode="NONE")
    return bm


CONTROLS = {"planar_cuts": _planar_cuts, "thin_ledge": _thin_ledge, "cylinder32": _cylinder, "plain_cube": _plain_cube}

WALL_LENGTH, WALL_DEPTH, WALL_HEIGHT = 2.0, 0.3, 1.0
#: Замер (BEVEL_SELECT_CORNERS_V1): до правки R 0.88-0.91 и 8-28 пропусков у торцов, после - P 1.0, R >= 0.987, у торцов <= 2.
WALL_MIN_PRECISION, WALL_MIN_RECALL, WALL_MAX_END_MISSES = 0.99, 0.98, 2


def _noisy_wall(noise):
    """Стена поля в миниатюре (`rounded_wall_noise_top`): верх - треугольники с шумом высоты, торец x = длина со своей фаской 45 градусов
    0.02 на переднем вертикальном ребре. Бевел по углу снимает обод верха и вертикальные рёбра торцов; углы обода у торцов -
    то место, где кнопка пропускала фаски."""

    bm = bmesh.new()
    top, bottom, segments = {}, {}, 8
    for i in range(segments + 1):
        x = WALL_LENGTH * i / segments
        for j, y in enumerate((0.0, WALL_DEPTH / 2, WALL_DEPTH)):
            rise = noise * math.sin(1.7 * i + 2.3 * j) if 0 < i < segments or j == 1 else 0.0
            top[i, j] = bm.verts.new((x, y, WALL_HEIGHT + rise))
        for j, y in enumerate((0.0, WALL_DEPTH)):
            bottom[i, j] = bm.verts.new((x, y, 0.0))
    for i in range(segments):
        bm.faces.new((bottom[i, 0], bottom[i + 1, 0], top[i + 1, 0], top[i, 0]))
        bm.faces.new((bottom[i + 1, 1], bottom[i, 1], top[i, 2], top[i + 1, 2]))
        bm.faces.new((bottom[i, 1], bottom[i + 1, 1], bottom[i + 1, 0], bottom[i, 0]))
        for j in (0, 1):
            a, b, c, d = top[i, j], top[i + 1, j], top[i + 1, j + 1], top[i, j + 1]
            for cycle in (((a, b, c), (a, c, d)) if (i + j) % 2 else ((a, b, d), (b, c, d))):
                bm.faces.new(cycle)
    bm.faces.new((bottom[0, 1], bottom[0, 0], top[0, 0], top[0, 1], top[0, 2]))
    bm.faces.new((bottom[segments, 0], bottom[segments, 1], top[segments, 2], top[segments, 1], top[segments, 0]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    end = {(WALL_LENGTH, 0.0, 0.0), (WALL_LENGTH, 0.0, WALL_HEIGHT)}
    edge = next(item for item in bm.edges if {vertex.co.to_tuple(4) for vertex in item.verts} == end)
    bmesh.ops.bevel(bm, geom=[edge], offset=0.02, offset_type="OFFSET", segments=1, profile=0.5, affect="EDGES", clamp_overlap=True,
                    loop_slide=True)
    return bm


def _wall_bevel(name, noise, segments, width, mode="FSTR_NONE"):
    source = _object(name + "_base", _noisy_wall(noise))
    modifier = source.modifiers.new("Bevel", "BEVEL")
    modifier.width, modifier.segments, modifier.limit_method, modifier.material = width, segments, "ANGLE", 1
    modifier.face_strength_mode = mode
    depsgraph = bpy.context.evaluated_depsgraph_get()
    mesh = bpy.data.meshes.new_from_object(source.evaluated_get(depsgraph), preserve_all_data_layers=True, depsgraph=depsgraph)
    bpy.data.objects.remove(source, do_unlink=True)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _run_the_rim_corners_at_the_wall_ends_are_found():
    """BEVEL_SELECT_CORNERS_V1: на углах обода у торцов (щель, сжатая clamp overlap у фаски торца, трапеции и короткие куски полосы,
    угол трёх бевелей, ряд с крайним рельсом против профиля) выделена каждая грань бевела; по всей стене - точность и полнота."""

    from cftuv import bevel_faces as bf
    from cftuv.bevel_faces_blender import read_mesh_view

    for noise in (0.0, 0.03):
        for segments in (1, 2, 3):
            for width in (0.03, 0.05):
                _clean()
                name = f"wall_n{noise}_s{segments}_w{width}"
                obj = _wall_bevel(name, noise, segments, width)
                truth = _truth(obj)
                bm = bmesh.new()
                bm.from_mesh(obj.data)
                selection = bf.select_bevel_faces(read_mesh_view(bm, obj.matrix_world))
                bm.free()
                corner = frozenset(polygon.index for polygon in obj.data.polygons if polygon.index in truth
                                   and min(polygon.center.x, WALL_LENGTH - polygon.center.x) < 0.15)
                precision, recall = _score(selection.faces, truth)
                SUMMARY.append(f"{name}: {selection.outcome} truth {len(truth)}, ends {len(corner)}, missed at ends "
                               f"{len(corner - selection.faces)}, P {precision} R {recall} events {dict(selection.events)}")
                # у торцов допустимы лишь крошечные грани угла, что гнут к соседу меньше 2 градусов, - и они названы числом
                missed = corner - selection.faces
                named = dict(selection.events).get(bf.BEVEL_NODE_FLAT_EDGE, 0)
                assert selection.outcome == bf.BEVEL_SELECT_GEOMETRIC and len(missed) <= min(WALL_MAX_END_MISSES, named), (name, sorted(missed))
                assert precision >= WALL_MIN_PRECISION and recall >= WALL_MIN_RECALL, (name, precision, recall)


def _truth(obj):
    return frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)


def _enter(objects, mode=(False, False, True), faces=(), edges=()):
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = mode
    for obj in objects:
        bm = bmesh.from_edit_mesh(obj.data)
        for element in (*bm.faces, *bm.edges, *bm.verts):
            element.select = False
        bm.faces.ensure_lookup_table()
        for index in faces:
            bm.faces[index].select_set(True)
        bm.edges.ensure_lookup_table()
        for index in edges:  # ребро без целой грани: семян нет, поиск по всему мешу
            bm.edges[index].select_set(True)
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data)


def _state(obj):
    bm = bmesh.from_edit_mesh(obj.data)
    selected = frozenset(face.index for face in bm.faces if face.select)
    return selected, sum(1 for edge in bm.edges if edge.seam), len(bm.faces), len(bm.verts)


def _selected_edges(obj):
    return frozenset(edge.index for edge in bmesh.from_edit_mesh(obj.data).edges if edge.select)


def _press(**kwargs):
    from cftuv.bevel_faces import select_bevel_faces
    from cftuv.bevel_faces_blender import read_mesh_view

    obj = bpy.context.active_object
    selection = select_bevel_faces(read_mesh_view(bmesh.from_edit_mesh(obj.data), obj.matrix_world))
    result = bpy.ops.hotspotuv.select_bevel_faces(**kwargs)
    return result, selection


def _score(selected, truth):
    tp = len(selected & truth)
    precision = tp / len(selected) if selected else 1.0
    recall = tp / len(truth) if truth else 1.0
    return round(precision, 3), round(recall, 3)


def _run_case(name, obj, outcome, exact=True):
    """Нажатие на меше с бевелом: путь, выделенные грани против правды, режим граней, меш не изменён."""

    truth = _truth(obj)
    _enter([obj], mode=(False, True, False))
    before = _state(obj)
    result, selection = _press()
    after = _state(obj)
    assert "FINISHED" in result and selection.outcome == outcome, (name, result, selection.outcome, selection.events)
    assert tuple(bpy.context.tool_settings.mesh_select_mode) == (False, False, True), name
    assert after[1:] == before[1:] == (0, *before[2:]), (name, "the mesh is not changed")
    assert after[0] == selection.faces, (name, "the selection is the proposal")
    precision, recall = _score(after[0], truth)
    if exact:
        assert after[0] == truth, (name, sorted(after[0] - truth), sorted(truth - after[0]))
    SUMMARY.append(f"{name}: {selection.outcome} {len(selection.strip)} strip / {len(selection.node)} node, truth {len(truth)}, "
                   f"P {precision} R {recall} events {dict(selection.events)}")
    bpy.ops.object.mode_set(mode="OBJECT")
    return selection, precision, recall


def _run_bevel_cases():
    from cftuv import bevel_faces as bf

    for base in BASES:
        for segments in (1, 2, 4):
            for mode, outcome in (("FSTR_ALL", bf.BEVEL_SELECT_FACE_STRENGTH), ("FSTR_NEW", bf.BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS),
                                  ("FSTR_NONE", bf.BEVEL_SELECT_GEOMETRIC)):
                _clean()
                name = f"{base}_s{segments}_{mode[5:].lower()}"
                selection, _precision, _recall = _run_case(name, _modifier_bevel(name, base, segments, mode), outcome)
                if mode == "FSTR_NEW":
                    assert "Face Strength mode to All" in bf.selection_line(name, selection)
        for segments in (1, 2):
            _clean()
            name = f"{base}_s{segments}_manual"
            _run_case(name, _manual_bevel(name, base, segments), bf.BEVEL_SELECT_GEOMETRIC)
    _clean()  # Affected на полном бевеле: все исходные грани затронуты, STRONG - тот же точный ответ, что у All
    _run_case("cube_s2_affected", _modifier_bevel("cube_s2_affected", "cube", 2, "FSTR_AFFECTED"), bf.BEVEL_SELECT_FACE_STRENGTH)
    _clean()  # куб с сеткой: плоские обрезки сетки у фаски - не узлы
    _run_case("grid_cube_s3_manual", _object("grid_cube_s3_manual", _grid_cube_bevel()), bf.BEVEL_SELECT_GEOMETRIC)


def _run_a_triangulated_bevel_is_exact_only_with_face_strength_all():
    """Геометрия слепа на треугольниках: All - точно; без атрибута - ничего не найдено по имени, выделение не тронуто."""

    from cftuv import bevel_faces as bf

    for mode in ("FSTR_ALL", "FSTR_NONE"):
        _clean()
        obj = _modifier_bevel(f"tri_{mode}", "cube", 2, mode)
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bmesh.ops.triangulate(bm, faces=[face for face in bm.faces if face.material_index == 1])
        bm.to_mesh(obj.data)
        bm.free()
        if mode == "FSTR_ALL":
            _run_case(f"tri_cube_s2_{mode[5:].lower()}", obj, bf.BEVEL_SELECT_FACE_STRENGTH)
            continue
        _enter([obj], mode=(False, True, False), edges=(0,))
        before = _state(obj)
        result, selection = _press()
        assert "CANCELLED" in result and selection.outcome == bf.BEVEL_SELECT_NONE_FOUND, (result, selection)
        assert _state(obj) == before and tuple(bpy.context.tool_settings.mesh_select_mode) == (False, True, False)
        SUMMARY.append(f"tri_cube_s2_none: {selection.outcome}, truth {len(_truth(obj))} (geometry is blind on triangles)")
        bpy.ops.object.mode_set(mode="OBJECT")


def _run_controls_find_nothing_and_keep_the_selection():
    from cftuv import bevel_faces as bf

    for name, make in CONTROLS.items():
        _clean()
        obj = _object(name, make())
        _enter([obj], mode=(False, True, False), edges=(0,))
        before, edges = _state(obj), _selected_edges(obj)
        result, selection = _press()
        assert "CANCELLED" in result and selection.outcome == bf.BEVEL_SELECT_NONE_FOUND, (name, result, selection)
        assert _state(obj) == before and tuple(bpy.context.tool_settings.mesh_select_mode) == (False, True, False), name
        assert _selected_edges(obj) == edges, name
        SUMMARY.append(f"control {name}: {selection.outcome} events {dict(selection.events)}")
        bpy.ops.object.mode_set(mode="OBJECT")


def _run_replace_and_extend():
    """Без выделенных граней (выделено одно ребро посреди плоской грани куба с сеткой) - весь меш, и замена, и добавление дают те же
    грани. Одинокое ребро режим граней снимает и при добавлении: с семенами (BEVEL_SELECT_SEED_V1) выделение граней всегда семя, и
    добавлять, кроме него, нечего - добавление осталось ради Shift и совместимости."""

    _clean()
    obj = _object("extend_grid", _grid_cube_bevel())
    middle = min(obj.data.polygons, key=lambda polygon: (polygon.center - __import__("mathutils").Vector((0.0, 0.0, 1.0))).length)
    inner = obj.data.loops[middle.loop_start].edge_index  # ребро без целой грани: ни одна грань не выделена
    _enter([obj], mode=(False, True, False), edges=(inner,))
    bpy.ops.hotspotuv.select_bevel_faces()
    replaced = _state(obj)[0]
    assert replaced and inner not in _selected_edges(obj), "replace drops the previous selection"
    _enter([obj], mode=(False, True, False), edges=(inner,))
    bpy.ops.hotspotuv.select_bevel_faces(extend=True)
    assert _state(obj)[0] == replaced, "extend selects the same faces"
    bpy.ops.object.mode_set(mode="OBJECT")


def _run_the_max_width_bounds_the_geometric_guess_only():
    """Ширина полосы 0.113 (фаска 0.08): предел 0.05 - геометрия ничего не находит по имени; атрибут All предел не читает."""

    from cftuv import bevel_faces as bf

    for mode, outcome in (("FSTR_NONE", bf.BEVEL_SELECT_NONE_FOUND), ("FSTR_ALL", bf.BEVEL_SELECT_FACE_STRENGTH)):
        _clean()
        obj = _modifier_bevel(f"narrow_{mode}", "cube", 1, mode)
        _enter([obj], mode=(False, True, False))
        result = bpy.ops.hotspotuv.select_bevel_faces(max_width=0.05)
        assert ("CANCELLED" in result) == (outcome == bf.BEVEL_SELECT_NONE_FOUND), (mode, result)
        assert _state(obj)[0] == (frozenset() if outcome == bf.BEVEL_SELECT_NONE_FOUND else _truth(obj)), mode
        bpy.ops.object.mode_set(mode="OBJECT")


def _run_every_mesh_in_edit_mode():
    from cftuv import bevel_faces as bf

    _clean()
    exact = _modifier_bevel("multi_all", "cube", 2, "FSTR_ALL")
    guessed = _modifier_bevel("multi_none", "lprofile", 2, "FSTR_NONE")
    control = _object("multi_control", _plain_cube())
    _enter([exact, guessed, control], mode=(False, True, False))
    result = bpy.ops.hotspotuv.select_bevel_faces()
    assert "FINISHED" in result
    assert _state(exact)[0] == _truth(exact) and _state(guessed)[0] == _truth(guessed) and not _state(control)[0]
    bpy.ops.object.mode_set(mode="OBJECT")
    lines = []
    for obj in (exact, guessed, control):
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        from cftuv.bevel_faces_blender import read_mesh_view

        lines.append(bf.selection_line(obj.name, bf.select_bevel_faces(read_mesh_view(bm, obj.matrix_world))))
        bm.free()
    assert [line.split(": ")[1].split(" ")[0] for line in lines] == [
        bf.BEVEL_SELECT_FACE_STRENGTH, bf.BEVEL_SELECT_GEOMETRIC, bf.BEVEL_SELECT_NONE_FOUND
    ], lines


def _run_the_button_is_drawn_in_edit_mode_in_every_select_mode():
    from cftuv.envelope_debug_panel import draw_decal_mesh_rows
    from cftuv.bevel_select_operator import HOTSPOTUV_OT_SelectBevelFaces

    assert "UNDO" in HOTSPOTUV_OT_SelectBevelFaces.bl_options
    _clean()
    obj = _modifier_bevel("panel_cube", "cube", 1, "FSTR_ALL")
    drawn = []

    class _Logged:
        """Заглушка `UILayout`: `prop` пишет имя свойства, `operator` - имя оператора, остальное - заглушка."""

        def __init__(self, log):
            self.log = log

        def __getattr__(self, name):
            def call(*args, **_kwargs):
                if name == "prop" and len(args) > 1:
                    self.log.append(args[1])
                if name == "operator" and args:
                    self.log.append(args[0])
                return _Logged(self.log)

            return call

    for mode, side in (((False, False, True), True), ((False, True, False), False), ((True, False, False), False)):
        _enter([obj], mode=mode)
        drawn.clear()
        draw_decal_mesh_rows(_Logged(drawn))
        assert {"hotspotuv.select_bevel_faces", "hotspotuv.seam_bevel"} <= set(drawn) and ("face_side" in drawn) == side, (mode, drawn)
        bpy.ops.object.mode_set(mode="OBJECT")
    drawn.clear()
    draw_decal_mesh_rows(_Logged(drawn))
    assert "hotspotuv.select_bevel_faces" not in drawn, "Object Mode draws no select button"


def _two_cubes():
    """Два куба 2 в одном меше (второй сдвинут на 5 по x): две группы бевела, не связанные рёбрами."""

    bm = bmesh.new()
    for x in (0.0, 5.0):
        bmesh.ops.create_cube(bm, size=2.0, matrix=__import__("mathutils").Matrix.Translation((x, 0.0, 0.0)))
    return bm


def _modifier_bevel_bm(name, bm, segments, mode, width=WIDTH):
    source = _object(name + "_base", bm)
    modifier = source.modifiers.new("Bevel", "BEVEL")
    modifier.width, modifier.segments, modifier.limit_method, modifier.material = width, segments, "ANGLE", 1
    modifier.face_strength_mode = mode
    depsgraph = bpy.context.evaluated_depsgraph_get()
    mesh = bpy.data.meshes.new_from_object(source.evaluated_get(depsgraph), preserve_all_data_layers=True, depsgraph=depsgraph)
    bpy.data.objects.remove(source, do_unlink=True)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _whole_mesh_faces(obj):
    """Ответ поиска по всему мешу без семян (выделение не читается)."""

    from cftuv.bevel_faces import select_bevel_faces
    from cftuv.bevel_faces_blender import read_mesh_view

    return set(select_bevel_faces(read_mesh_view(bmesh.from_edit_mesh(obj.data), obj.matrix_world)).faces)


def _press_seeds(obj, seeds, **kwargs):
    """Семена выделены в режиме граней, нажатие; `(результат, предложение, выделение после)`."""

    from cftuv.bevel_faces_blender import read_mesh_view
    from cftuv.bevel_seed import select_from_seeds

    _enter([obj], faces=seeds)
    selection = select_from_seeds(read_mesh_view(bmesh.from_edit_mesh(obj.data), obj.matrix_world))
    result = bpy.ops.hotspotuv.select_bevel_faces(grow=True, **kwargs)
    return result, selection, _state(obj)[0]


def _run_a_seed_grows_its_own_bevel_group():
    """Два куба в одном меше: семя - полоса первого куба; выделена группа первого куба, второй не тронут (All - точно, без атрибута -
    геометрия с тем же ответом); добавление (Shift) даёт то же - семена уже в группе."""

    from cftuv import bevel_seed as bs

    for segments in (1, 2, 4):
        for mode, source in (("FSTR_ALL", "BEVEL_SELECT_FACE_STRENGTH"), ("FSTR_NONE", "BEVEL_SELECT_GEOMETRIC")):
            _clean()
            name = f"seed_cubes_s{segments}_{mode[5:].lower()}"
            obj = _modifier_bevel_bm(name, _two_cubes(), segments, mode)
            first = frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1 and polygon.center.x < 2.5)
            seed = min(index for index in first if len(obj.data.polygons[index].vertices) == 4)
            for extend in (False, True):
                result, selection, after = _press_seeds(obj, (seed,), extend=extend)
                assert "FINISHED" in result and selection.outcome == bs.BEVEL_SELECT_FROM_SEED and selection.source == source, (name, selection)
                assert after == first, (name, extend, sorted(after - first), sorted(first - after))
                bpy.ops.object.mode_set(mode="OBJECT")
            _enter([obj], faces=(seed,))  # BEVEL_SELECT_GROW_EXPLICIT_V1: без «Grow» выделение - не семена, весь меш
            whole = _whole_mesh_faces(obj)
            result = bpy.ops.hotspotuv.select_bevel_faces()
            assert "FINISHED" in result and _state(obj)[0] == whole and not first >= whole, (name, len(whole))
            bpy.ops.object.mode_set(mode="OBJECT")
            SUMMARY.append(f"{name}: {selection.outcome} ({selection.source}) {len(after)} faces = first cube, events {dict(selection.events)}; "
                           f"without Grow the whole mesh: {len(whole)} faces")


def _run_a_seed_that_is_not_a_bevel_face_keeps_the_selection():
    from cftuv import bevel_seed as bs

    for mode in ("FSTR_ALL", "FSTR_NONE"):
        _clean()
        obj = _modifier_bevel_bm(f"seed_flat_{mode}", _two_cubes(), 2, mode)
        flat = min(polygon.index for polygon in obj.data.polygons if polygon.material_index != 1)
        result, selection, after = _press_seeds(obj, (flat,))
        assert "CANCELLED" in result and selection.outcome == bs.BEVEL_SEED_NOT_BEVEL and after == {flat}, (mode, result, selection)
        assert dict(selection.events)[bs.BEVEL_SEED_NOT_BEVEL] == 1
        SUMMARY.append(f"seed_flat_{mode[5:].lower()}: {selection.outcome}, selection kept")
        bpy.ops.object.mode_set(mode="OBJECT")


def _run_a_seed_on_the_wall_leaves_the_source_chamfer_out():
    """Стена поля: семена - каждая полоса обода посередине стены по очереди. Группа - бевел стены с точностью 1.0 (своя фаска торца
    исходного меша не взята); полнота не ниже поиска по всему мешу (с допуском одной грани) у большинства семян. Остаток - семя,
    чей ряд на шумном верху в 3 сегмента сам частичный (одна грань с поворотом ~19 градусов): рост упирается в стык по имени
    (`BEVEL_SEED_OFFSET_MISMATCH`) - записан в сводку."""

    from cftuv import bevel_faces as bf
    from cftuv import bevel_seed as bs
    from cftuv.bevel_faces_blender import read_mesh_view

    for noise in (0.0, 0.03):
        for segments in (1, 2, 3):
            _clean()
            name = f"seed_wall_n{noise}_s{segments}"
            obj = _wall_bevel(name, noise, segments, 0.03)
            truth = _truth(obj)
            bm = bmesh.new()
            bm.from_mesh(obj.data)
            view = read_mesh_view(bm, obj.matrix_world)
            bm.free()
            _whole_precision, whole_recall = _score(bf.select_bevel_faces(view).faces, truth)
            rim = [polygon.index for polygon in obj.data.polygons if polygon.index in truth and len(polygon.vertices) == 4
                   and 0.5 < polygon.center.x < 1.5 and polygon.center.z > WALL_HEIGHT - 0.05]
            recalls = []
            for seed in rim:
                selection = bs.select_from_seeds(view, {seed})
                if selection.outcome == bs.BEVEL_SEED_NOT_BEVEL:  # кусок у торца шумного верха, не ставший рядом от семени, - по имени
                    continue
                precision, recall = _score(selection.faces | selection.seeds, truth)
                assert selection.outcome == bs.BEVEL_SELECT_FROM_SEED and precision == 1.0, (name, seed, selection.outcome, precision)
                recalls.append(recall)
            kept = sum(recall >= whole_recall - 1.0 / len(truth) - 1e-9 for recall in recalls)
            SUMMARY.append(f"{name}: {len(recalls)}/{len(rim)} seeds grown, P 1.0, R min {min(recalls)} max {max(recalls)}, "
                           f"{kept}/{len(recalls)} at whole-mesh R {whole_recall}")
            assert 2 * len(recalls) > len(rim) and 2 * kept > len(recalls), (name, len(rim), recalls, whole_recall)
            result, selection, after = _press_seeds(obj, (rim[recalls.index(max(recalls))],))
            assert "FINISHED" in result and after == selection.faces | selection.seeds and after <= truth, name
            bpy.ops.object.mode_set(mode="OBJECT")

def _run_seam_bevel_joins_the_rails_and_the_nodes():
    """BEVEL_SEAM_MARK_V1: весь бевел двух кубов (All, 2 сегмента) - швы ровно граница выделения и рёбра «узел | полоса» по атрибуту;
    один выделенный узел - швы по всему его контуру; старые швы не снимаются."""

    from cftuv.bevel_faces import FACE_STRENGTH_ATTRIBUTE

    _clean()
    obj = _modifier_bevel_bm("seam_cubes", _two_cubes(), 2, "FSTR_ALL")
    strength = {item.index: int(value.value) for item, value in zip(obj.data.polygons, obj.data.attributes[FACE_STRENGTH_ATTRIBUTE].data)}
    bevel = _truth(obj)
    node = frozenset(face for face in bevel if strength[face] < 0)
    expected = set()
    for edge in obj.data.edges:
        key = set(edge.vertices)
        faces = [polygon.index for polygon in obj.data.polygons if key <= set(polygon.vertices)]
        inside = [face in bevel for face in faces]
        kinds = {face in node for face in faces}
        if (any(inside) and not all(inside)) or (all(inside) and len(faces) == 2 and kinds == {True, False}):
            expected.add(edge.index)
    _enter([obj], faces=sorted(bevel))
    result = bpy.ops.hotspotuv.seam_bevel()
    seams = _seams(obj)
    assert "FINISHED" in result and node and seams == expected, (len(seams), len(expected), sorted(seams ^ expected)[:10])
    bpy.ops.object.mode_set(mode="OBJECT")
    for edge in obj.data.edges:
        edge.use_seam = False
    kept = obj.data.edges[0].index
    obj.data.edges[kept].use_seam = True
    one = min(node)
    _enter([obj], faces=(one,))
    assert "FINISHED" in bpy.ops.hotspotuv.seam_bevel()
    ring = {edge.index for edge in obj.data.edges if set(edge.vertices) <= set(obj.data.polygons[one].vertices)}
    assert _seams(obj) == ring | {kept}, sorted(_seams(obj) ^ (ring | {kept}))
    _enter([obj])
    assert "CANCELLED" in bpy.ops.hotspotuv.seam_bevel()
    bpy.ops.object.mode_set(mode="OBJECT")
    SUMMARY.append(f"seam_cubes: {len(expected)} seam edges = rails + node|strip ({len(node)} node faces); one node -> {len(ring)} edges")


def _run_seam_bevel_isolates_the_wall_end_top_corner_nodes_without_face_strength():
    """BEVEL_SPLIT_RAIL_LOCAL_PLANE_V1 + BEVEL_SEAM_NODE_AGREEMENT_V1 (поле: «Grow Bevel» -> «Seam Bevel» на `rounded_wall_noise_top`
    без атрибута, верхний угол торца, где обод встречает вертикальное ребро торца): узел угла был не изолирован - соседний кусок
    полосы уходил в узел. Правда узлов - та же стена с Face Strength All (та же топология); у верхних углов торцов каждый узел - свой
    патч, и ни один шов не лежит между двумя полосами правды."""

    from cftuv.bevel_faces import FACE_STRENGTH_ATTRIBUTE

    for noise in (0.0, 0.03):
        _clean()
        twin = _wall_bevel(f"seam_twin_n{noise}", noise, 1, 0.03, "FSTR_ALL")
        strength = {item.index: int(value.value) for item, value in zip(twin.data.polygons, twin.data.attributes[FACE_STRENGTH_ATTRIBUTE].data)}
        obj = _wall_bevel(f"seam_wall_n{noise}", noise, 1, 0.03)
        assert FACE_STRENGTH_ATTRIBUTE not in obj.data.attributes and len(obj.data.polygons) == len(strength)
        node = frozenset(face for face, value in strength.items() if value < 0)
        seed = min(polygon.index for polygon in obj.data.polygons if strength[polygon.index] == 0 and len(polygon.vertices) == 4
                   and 0.5 < polygon.center.x < 1.5 and polygon.center.z > WALL_HEIGHT - 0.05)
        _enter([obj], faces=(seed,))
        assert "FINISHED" in bpy.ops.hotspotuv.select_bevel_faces(grow=True)
        assert "FINISHED" in bpy.ops.hotspotuv.seam_bevel()
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        selected = frozenset(face.index for face in bm.faces if face.select)
        seams = frozenset(edge.index for edge in bm.edges if edge.seam)
        corner = {face for face in node & selected if bm.faces[face].calc_center_median().z > WALL_HEIGHT / 2
                  and min(bm.faces[face].calc_center_median().x, WALL_LENGTH - bm.faces[face].calc_center_median().x) < 0.15}
        open_edges = {edge.index for face in corner for edge in bm.faces[face].edges
                      if edge.index not in seams and not all(other.index in node for other in edge.link_faces)}
        across = {edge.index for edge in bm.edges if edge.seam and len(edge.link_faces) == 2
                  and all(other.index in selected and other.index not in node for other in edge.link_faces)}
        bpy.ops.object.mode_set(mode="OBJECT")
        assert corner and not open_edges and not across, (noise, sorted(corner), sorted(open_edges), sorted(across))
        SUMMARY.append(f"seam_wall_n{noise}: grown {len(selected)} faces, {len(seams)} seams, {len(corner)} top-corner nodes isolated, "
                       f"0 seams between two strips")


def _seams(obj):
    return frozenset(edge.index for edge in bmesh.from_edit_mesh(obj.data).edges if edge.seam)


def _main():
    import cftuv

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован
        pass
    _run_the_button_is_drawn_in_edit_mode_in_every_select_mode()
    _run_replace_and_extend()
    _run_every_mesh_in_edit_mode()
    _run_the_max_width_bounds_the_geometric_guess_only()
    _run_controls_find_nothing_and_keep_the_selection()
    _run_a_triangulated_bevel_is_exact_only_with_face_strength_all()
    _run_bevel_cases()
    _run_the_rim_corners_at_the_wall_ends_are_found()
    _run_a_seed_grows_its_own_bevel_group()
    _run_a_seed_that_is_not_a_bevel_face_keeps_the_selection()
    _run_a_seed_on_the_wall_leaves_the_source_chamfer_out()
    _run_seam_bevel_joins_the_rails_and_the_nodes()
    _run_seam_bevel_isolates_the_wall_end_top_corner_nodes_without_face_strength()
    for line in SUMMARY:
        print("[BevelSelectTest]", line, flush=True)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("BEVEL_SELECT_FACES_BLENDER_OK", flush=True)


if __name__ == "__main__":
    _main()
