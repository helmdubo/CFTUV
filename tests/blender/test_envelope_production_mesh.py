"""Blender 4.5 background smoke: «Build Decal Mesh» строит продуктовый меш.

Утверждения стоят на числах и содержимом меша, а не на виде картинки:

1. ХОЛОДНОЕ нажатие (без отладочной кнопки) создаёт ОДИН объект
   `<исходный>.CFTUV_Decal` — потомок исходного с единичным преобразованием:
   грани есть, слой UV `UVMap` есть, ВСЕ домены `MATERIALIZED`, атрибуты граней
   `cftuv_domain` и `cftuv_owner` лежат, слот материала один;
2. ПЕРЕСБОРКА ИДЕМПОТЕНТНА: тот же объект, тот же меш по дайджесту, ни одного
   лишнего объекта, меша и материала;
3. ТЁПЛОЕ нажатие после отладочной кнопки не собирает подготовки: счётчик
   `CONVEYOR_PREPARATION` контроллера не растёт (закон UV — параметр
   материализации, ключ кэша подготовок его не содержит);
4. 0 и 2 воркера дают ПОБИТОВО тот же меш (дайджест того, что лежит в Blender);
5. смещение над поверхностью — настройка сцены: позиции = локальная позиция
   батча + нормаль · смещение;
6. ДЛИННОЕ ИМЯ источника (60 символов): Blender режет имя объекта до 63 байт, и
   декаль ищется по маркеру, а не по имени — два нажатия дают один объект;
7. пропуск ПИСАТЕЛЯ (`ADAPTER_*`) виден в строке статуса, как и отказ пути;
8. ЗАКОН ТОПОЛОГИИ: кнопка просит `PLANAR_POLYGONS_V1`, поэтому полосы лежат в меше
   ЧЕТЫРЁХГРАННИКАМИ и выпуклыми многоугольниками (грань в 4 и более петель, без
   триангуляции Blender), веера — треугольниками; каждая грань от 4 петель плоская
   (её вершины в одной плоскости);
9. НЕВЫПУКЛАЯ грань закона (`PLANAR_AFFINE_UV_POLYGON_V1`) пишется ОДНОЙ гранью меша: петли
   = UV-петли, плоскость до 1e-5, площадь граней меша равна точной площади батча, а любая
   триангуляция, которую Blender выбрал для показа, даёт ту же UV-интерполяцию на всех
   петлях грани (аффинность UV, а не совпадение вершин);
10. СКЛАДКА 90°: два патча (пол и стена) через общий шов дают на общей цепи ОДНУ вершину
   на станцию (вершины общего шва сварены по семантической ссылке при побитово равных
   позициях), смещение общей вершины — митра (пересечение сдвинутых плоскостей двух
   доменов), каждая грань остаётся на сдвинутой плоскости СВОЕГО домена (плоская в
   пределах 1e-5), а общее ребро — шов UV между гранями двух доменов;
11. ДЕКАЛЬ ЧЕРЕЗ КОСУЮ СКЛАДКУ 90° (закон укладки `SOURCE_TRIANGLES_CLIPPED_V1`): каждая точка грани
   (вершины, середины рёбер, центр) отстоит от поверхности источника не дальше смещения, грани
   от четырёх вершин есть, а красный контроль (укладка без резки) уходит хордой в стену
   глубже смещения на сантиметры;
12. ОТМЕНА: оператор — REGISTER|UNDO (`UNDO_REQUIRED_REASON`). Нажатие в EDIT-режиме,
   выход в OBJECT БЕЗ шага отмены (`undo=False` — как Ctrl+Z владельца сразу после
   кнопки), затем `ed.undo`: декали нет, ни коллекции сцены, ни слой не ссылаются на
   освобождённый объект (сравнение указателей без разыменования), перестройка
   депсграфа (`view_layer.update()`, место падения) проходит; `ed.redo` возвращает
   декаль; следующее нажатие работает. То же — для кнопки отладки Envelope (GP-объект)
   и кнопки Clear. Без флага UNDO эта последовательность роняла Blender 4.5 в UI
   (`DepsgraphNodeBuilder::build_materials` на висячем объекте);
13. РЕБРО НУЛЕВОЙ ДЛИНЫ (`ZERO_LENGTH_EDGE`): обе кнопки ядра отказывают одним именем и строкой
   «run Merge by Distance» ДО анализа, виновное ребро остаётся выделенным, источник не тронут;
   после слияния вершин (bmesh `remove_doubles`, в памяти) та же кнопка строит декаль.

Прогон (без `--factory-startup`: sympy в 4.5 живёт в профиле пользователя):
blender --background --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_PRODUCTION_MESH_BLENDER_SMOKE_OK
"""

from __future__ import annotations

from pathlib import Path
import sys

import bpy


REPO_ROOT = Path(__file__).resolve().parents[2]
for path in (REPO_ROOT, REPO_ROOT / "kernel" / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_envelope_debug_bridge import (  # noqa: E402
    _build_two_patch_seam,
    _enter_edge_selection,
    _reset_scene,
)


SOURCE = "EnvelopeTwoPatch"
DECAL = SOURCE + ".CFTUV_Decal"


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _decal_settings():
    return bpy.context.scene.hotspotuv_decal_mesh


def _controller():
    return bpy.context.window_manager._cftuv_envelope_debug_session


def _fresh_scene(*, workers=0, alpha=0.25):
    _reset_scene()
    controller = _controller()
    if controller is not None:
        controller.clear()
    for mesh in list(bpy.data.meshes):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    source = _build_two_patch_seam()
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = alpha
    settings.envelope_debug_workers = workers
    _decal_settings().offset = 0.02
    _decal_settings().material_name = "CFTUV_Decal"
    return source


def _preparation_builds():
    controller = _controller()
    return 0 if controller is None else controller.build_counts["CONVEYOR_PREPARATION"]


def _press():
    result = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    assert result == {"FINISHED"}, result
    return bpy.data.objects[DECAL]


def _mesh_digest(decal):
    from cftuv.envelope_production_mesh import mesh_content_digest

    return mesh_content_digest(decal.data)


def _decal_objects():
    return [
        item for item in bpy.data.objects if "cftuv_source_revision" in item.keys()
    ]


def _off_plane(mesh, polygon):
    """Наибольшее расстояние вершины грани от плоскости грани (нормаль Ньюэлла, через первую вершину)."""

    points = [mesh.vertices[index].co for index in polygon.vertices]
    normal = points[0] * 0.0
    for index, current in enumerate(points):
        following = points[(index + 1) % len(points)]
        normal.x += (current.y - following.y) * (current.z + following.z)
        normal.y += (current.z - following.z) * (current.x + following.x)
        normal.z += (current.x - following.x) * (current.y + following.y)
    assert normal.length > 0.0
    return max(abs(normal.normalized().dot(point - points[0])) for point in points)


def _assert_faces_follow_the_polygon_law(mesh, *, require_quads, planar=True):
    """Грани меша — треугольники, четырёхгранники и многоугольники, и каждая грань от 4 петель плоская.

    `planar=False` — у домена развёртки смещение идёт вдоль нормали КАЖДОЙ вершины (закон ядра), и грань,
    плоская в батче (кусок в одном треугольнике источника), после смещения не плоская на записанную
    величину (`ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES`): тогда проверяется только, что число конечно и
    меньше смещения.
    """

    sizes = [len(polygon.vertices) for polygon in mesh.polygons]
    assert min(sizes) >= 3, sorted(set(sizes))
    assert len(mesh.loops) == sum(sizes)
    if require_quads:
        assert 4 in sizes, sizes
    quads = 0
    for polygon in mesh.polygons:
        count = len(polygon.vertices)
        if count < 4:
            continue
        quads += int(count == 4)
        points = [mesh.vertices[index].co for index in polygon.vertices]
        # Нормаль Ньюэлла: у многоугольника с вершинами на прямой первые три точки её не задают.
        normal = points[0] * 0.0
        for index, current in enumerate(points):
            following = points[(index + 1) % count]
            normal.x += (current.y - following.y) * (current.z + following.z)
            normal.y += (current.z - following.z) * (current.x + following.x)
            normal.z += (current.x - following.x) * (current.y + following.y)
        assert normal.length > 0.0
        if not planar:
            assert _off_plane(mesh, polygon) < 0.02, polygon.index
            continue
        for point in points:
            assert abs(normal.normalized().dot(point - points[0])) < 1e-5, polygon.index
    return quads


def _run_cold_press_builds_one_child_object():
    source = _fresh_scene()
    before = _preparation_builds()

    decal = _press()

    # Холодное нажатие наполнило кэши ТЕМ ЖЕ вычислителем, что и отладка.
    assert _preparation_builds() - before == 2

    assert decal.parent == source
    assert tuple(decal.location) == (0.0, 0.0, 0.0)
    assert tuple(decal.rotation_euler) == (0.0, 0.0, 0.0)
    assert tuple(decal.scale) == (1.0, 1.0, 1.0)
    mesh = decal.data
    assert len(mesh.polygons) > 0 and len(mesh.vertices) > 0
    assert mesh.uv_layers.get("UVMap") is not None
    assert len(mesh.uv_layers["UVMap"].data) == len(mesh.loops)
    domains = {item.value for item in mesh.attributes["cftuv_domain"].data}
    assert len(domains) == 2, domains
    assert mesh.attributes["cftuv_owner"].domain == "FACE"
    assert [item.name for item in mesh.materials] == ["CFTUV_Decal"]
    assert decal["cftuv_source_revision"]
    status = _decal_settings().status
    assert status == "MATERIALIZED 2 / refused 0", status
    assert "cold" in _decal_settings().timing, _decal_settings().timing
    # Смещение: плоскость патчей z = 0, нормаль +z или -z; позиции на 0.02.
    assert {round(abs(item.co.z), 6) for item in mesh.vertices} == {0.02}
    # Ни одной UV вне [0, 1] по v: закон V1 кладёт полосу в единичный квадрат поперёк.
    v_values = [item.uv[1] for item in mesh.uv_layers["UVMap"].data]
    assert min(v_values) >= -1e-6 and max(v_values) <= 1.0 + 1e-6
    # Закон топологии: полосы — четырёхгранники и многоугольники (в Blender они остались гранями в 4+ петли).
    quads = _assert_faces_follow_the_polygon_law(mesh, require_quads=True)
    print("PLANAR_POLYGONS observed:", quads, "quads of", len(mesh.polygons), "faces")
    # Исходный объект мешем декаля не тронут.
    assert len(source.data.polygons) == 2
    return source, decal


def _run_rebuild_is_idempotent(decal):
    digest = _mesh_digest(decal)
    objects = len(bpy.data.objects)
    meshes = len(bpy.data.meshes)
    materials = len(bpy.data.materials)

    again = _press()

    assert again == decal and again.name == DECAL
    assert _mesh_digest(again) == digest
    assert len(bpy.data.objects) == objects and len(_decal_objects()) == 1
    assert len(bpy.data.meshes) == meshes
    assert len(bpy.data.materials) == materials
    assert again.data.name == DECAL
    assert len(again.data.materials) == 1


def _run_offset_is_a_scene_setting():
    decal = bpy.data.objects[DECAL]
    _decal_settings().offset = 0.05
    decal = _press()
    assert {round(abs(item.co.z), 6) for item in decal.data.vertices} == {0.05}
    _decal_settings().offset = 0.02


def _run_warm_press_after_a_debug_build_reuses_the_preparations():
    source = _fresh_scene()
    before = _preparation_builds()
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug() == {"FINISHED"}
    )
    controller = _controller()
    builds = controller.build_counts
    keys = sorted(map(repr, controller._conveyor_preparation_cache))
    assert builds["CONVEYOR_PREPARATION"] - before == 2

    decal = _press()

    assert controller.build_counts == builds, (controller.build_counts, builds)
    assert sorted(map(repr, controller._conveyor_preparation_cache)) == keys
    assert "warm" in _decal_settings().timing, _decal_settings().timing
    assert "built 0" in _decal_settings().timing, _decal_settings().timing
    assert _decal_settings().status == "MATERIALIZED 2 / refused 0"
    assert decal.parent == source
    return _mesh_digest(decal)


def _run_workers_do_not_change_the_mesh(reference_digest):
    from cftuv import envelope_queue_pool

    original = envelope_queue_pool.COVERAGE_POOL_MIN_BYTES
    envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = 0
    try:
        _fresh_scene(workers=2)
        before = _preparation_builds()
        decal = _press()
        assert _preparation_builds() - before == 2
        pooled = _mesh_digest(decal)
        _fresh_scene(workers=0)
        sequential = _mesh_digest(_press())
    finally:
        envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = original
    assert pooled == sequential == reference_digest, (
        pooled,
        sequential,
        reference_digest,
    )


def _run_nothing_selected_is_refused_by_name():
    source = _fresh_scene()
    import bmesh

    bm = bmesh.from_edit_mesh(source.data)
    for edge in bm.edges:
        edge.select = False
    bmesh.update_edit_mesh(source.data)
    assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"CANCELLED"}
    assert "select a whole PhysicalChain" in _decal_settings().status
    assert bpy.data.objects.get(DECAL) is None


def _run_a_zero_length_edge_is_named_selected_and_never_repaired():
    """ZERO_LENGTH_EDGE: ребро с совпавшими концами названо ДО анализа обеими кнопками ядра.

    Шов двух патчей разбит вершиной 6, лежащей в одной точке с вершиной 4 (как вершины 6 и
    19 на `wall_noise_top`). Кнопка отказывает ОДНИМ именем и строкой «что делать», виновное
    ребро остаётся выделенным (режим рёбер), источник не тронут. После `Merge by Distance`
    на источнике (bmesh `remove_doubles`, в памяти) та же кнопка строит декаль.
    """

    import bmesh

    _reset_scene()
    controller = _controller()
    if controller is not None:
        controller.clear()
    mesh = bpy.data.meshes.new("EnvelopeZeroLengthSeamMesh")
    mesh.from_pydata(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 1.0, 0.0),
         (2.0, 1.0, 0.0), (1.0, 1.0, 0.0)],
        (),
        [(0, 1, 6, 4, 3), (1, 2, 5, 4, 6)],
    )
    mesh.update()
    for edge in mesh.edges:
        edge.use_seam = set(edge.vertices) in ({1, 6}, {4, 6})
    source = bpy.data.objects.new("EnvelopeZeroLengthSeam", mesh)
    bpy.context.scene.collection.objects.link(source)
    _enter_edge_selection(source, [edge.index for edge in mesh.edges if edge.use_seam])
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    settings.envelope_debug_workers = 0
    wanted = "ZERO_LENGTH_EDGE: 1 edges (e.g. vertices 4–6); run Merge by Distance"

    def refused(operator):
        try:
            operator()
        except RuntimeError as exc:
            assert wanted in str(exc), str(exc)
            return
        raise AssertionError("the button must refuse a zero-length edge")

    def offending():
        bm = bmesh.from_edit_mesh(source.data)
        return [tuple(sorted(item.index for item in edge.verts)) for edge in bm.edges if edge.select]

    refused(bpy.ops.hotspotuv.build_envelope_decal_mesh)
    assert _decal_settings().status == "Failed: " + wanted, _decal_settings().status
    assert offending() == [(4, 6)], offending()
    assert tuple(bpy.context.tool_settings.mesh_select_mode) == (False, True, False)
    assert bpy.data.objects.get(DECAL) is None

    _enter_edge_selection(source, [edge.index for edge in source.data.edges if edge.use_seam])
    refused(bpy.ops.hotspotuv.build_exact_reference_envelope_debug)
    assert settings.envelope_debug_outcome == "ZERO_LENGTH_EDGE", settings.envelope_debug_outcome
    assert settings.envelope_debug_status == "Failed: " + wanted, settings.envelope_debug_status
    assert offending() == [(4, 6)], offending()

    # Хост ничего не сваривает молча: вершин и рёбер столько же.
    assert (len(source.data.vertices), len(source.data.edges)) == (7, 8)

    bm = bmesh.from_edit_mesh(source.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-4)
    bmesh.update_edit_mesh(source.data)
    bm.edges.ensure_lookup_table()
    _enter_edge_selection(source, [edge.index for edge in bm.edges if edge.seam])
    decal = _press_named(source)
    assert decal.parent == source
    assert _decal_settings().status == "MATERIALIZED 2 / refused 0", _decal_settings().status


def _run_a_long_source_name_never_multiplies_the_decal():
    from cftuv.envelope_production_mesh import decal_object_name

    source = _fresh_scene()
    source.name = "L" * 60
    wanted = decal_object_name(source.name)
    assert len(wanted.encode("utf-8")) <= 63 and wanted != source.name + ".CFTUV_Decal"

    first = bpy.data.objects.get(_press_named(source).name)
    second = _press_named(source)

    assert second == first, (first.name, second.name)
    assert first.name == wanted and len(first.name.encode("utf-8")) <= 63
    assert len(_decal_objects()) == 1, [item.name for item in _decal_objects()]
    assert first.parent == source and first["cftuv_source_object"] == source.name
    assert first.data.name == wanted


def _press_named(source, *, undo=False):
    # `True` первым аргументом — как нажатие кнопки в UI: оператор кладёт свой шаг
    # отмены. Вызов из Python без него шага не кладёт (`bpy.ops`, `C_undo=False`).
    if undo:
        result = bpy.ops.hotspotuv.build_envelope_decal_mesh(True)
    else:
        result = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    assert result == {"FINISHED"}, result
    found = [item for item in _decal_objects() if item.parent == source]
    assert len(found) == 1, [item.name for item in _decal_objects()]
    return found[0]


def _run_an_adapter_skip_reaches_the_status_line():
    import dataclasses

    from cftuv import envelope_production_export as export

    source = _fresh_scene()
    original = export.run_production

    def damaged(*args, **kwargs):
        run = original(*args, **kwargs)
        first, *rest = run.results
        broken = dataclasses.replace(first, normal=(float("nan"), 0.0, 1.0))
        return dataclasses.replace(run, results=(broken, *rest))

    export.run_production = damaged
    try:
        _press_named(source)
    finally:
        export.run_production = original
    status = _decal_settings().status
    assert status == "MATERIALIZED 1 / refused 1 (ADAPTER_NORMAL_MISSING)", status
    assert export.receipt_report_level  # уровень отчёта — по квитанции (см. host-тест)


def _run_the_operator_declares_undo_with_a_named_reason():
    from cftuv.envelope_production_operator import (
        HOTSPOTUV_OT_BuildEnvelopeDecalMesh,
        UNDO_REQUIRED_REASON,
    )

    assert set(HOTSPOTUV_OT_BuildEnvelopeDecalMesh.bl_options) == {"REGISTER", "UNDO"}
    assert "EDIT mode" in UNDO_REQUIRED_REASON and "memfile" in UNDO_REQUIRED_REASON


def _walk_every_datablock():
    """Обход как при перерисовке: висячая ссылка здесь роняет Blender или бросает."""

    for item in bpy.data.objects:
        assert item.data is not None, item.name
        _ = (item.name, item.type, item.parent)
    for mesh in bpy.data.meshes:
        _ = (mesh.name, len(mesh.polygons), len(mesh.uv_layers))


def _linked_object_pointers():
    """Указатели объектов, на которые ссылаются коллекции сцены и слой: БЕЗ разыменования."""

    pointers = set()

    def visit(collection):
        pointers.update(item.as_pointer() for item in collection.objects)
        for child in collection.children:
            visit(child)

    visit(bpy.context.scene.collection)
    pointers.update(item.as_pointer() for item in bpy.context.view_layer.objects)
    return pointers


def _assert_scene_links_only_live_objects():
    """Висячий объект после отмены виден по указателю раньше, чем он уронит депсграф."""

    live = {item.as_pointer() for item in bpy.data.objects}
    dangling = _linked_object_pointers() - live
    assert not dangling, f"the scene links {len(dangling)} object(s) bpy.data no longer owns"


def _enable_background_undo(source):
    """Фоновый Blender включает систему отмены явным шагом, а отменять она начинает
    со ВТОРОГО: первый `undo_push` только инициализирует (`ed.undo.poll()`
    остаётся ложным), второй — настоящий шаг."""

    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.ed.undo_push(message="initialize")
    bpy.ops.ed.undo_push(message="scene ready")
    assert bpy.ops.ed.undo.poll()
    seam = [edge.index for edge in source.data.edges if edge.use_seam]
    assert seam
    _enter_edge_selection(source, seam)
    return seam


def _undo_from_the_state_the_button_left():
    """Выход из EDIT без шага отмены и отмена: как Ctrl+Z владельца сразу после
    кнопки (`ed.undo` в EDIT-режиме фоновый Blender отказывает; `mode_set` из Python
    шага не кладёт). Перестройка депсграфа — место падения без флага UNDO."""

    bpy.ops.object.mode_set(mode="OBJECT")
    assert bpy.ops.ed.undo() == {"FINISHED"}
    _assert_scene_links_only_live_objects()
    bpy.context.view_layer.update()
    _walk_every_datablock()


def _redo_and_check():
    assert bpy.ops.ed.redo() == {"FINISHED"}
    _assert_scene_links_only_live_objects()
    bpy.context.view_layer.update()
    _walk_every_datablock()


def _run_undo_after_a_press_in_edit_mode_keeps_the_scene_consistent():
    source = _fresh_scene()
    seam = _enable_background_undo(source)
    _press_named(source, undo=True)
    assert len(_decal_objects()) == 1

    _undo_from_the_state_the_button_left()
    assert not _decal_objects(), [item.name for item in _decal_objects()]
    source = bpy.data.objects[SOURCE]
    assert source.data is not None and len(source.data.polygons) == 2

    _redo_and_check()
    redone = _decal_objects()
    assert len(redone) == 1, [item.name for item in redone]
    print("UNDO_OBSERVED redo restored:", redone[0].name, "mode", bpy.context.mode)

    source = bpy.data.objects[SOURCE]
    bpy.context.view_layer.objects.active = source
    if source.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    _enter_edge_selection(source, seam)
    decal = _press_named(source)
    assert decal.parent == source and len(_decal_objects()) == 1
    assert decal.data.uv_layers.get("UVMap") is not None
    _walk_every_datablock()


def _run_undo_after_the_debug_button_and_after_clear_keeps_the_scene_consistent():
    """Кнопка отладки создаёт GP-объект, материалы и тексты из EDIT-режима; Clear удаляет их."""

    from cftuv.envelope_debug_renderer import envelope_debug_object_name

    source = _fresh_scene()
    _enable_background_undo(source)
    assert bpy.ops.hotspotuv.build_exact_reference_envelope_debug(True) == {"FINISHED"}
    gp_name = envelope_debug_object_name(source)
    assert gp_name in bpy.data.objects

    _undo_from_the_state_the_button_left()
    assert gp_name not in bpy.data.objects
    _redo_and_check()
    assert gp_name in bpy.data.objects

    assert bpy.ops.hotspotuv.clear_envelope_debug(True) == {"FINISHED"}
    assert gp_name not in bpy.data.objects
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    assert bpy.ops.ed.undo() == {"FINISHED"}
    _assert_scene_links_only_live_objects()
    bpy.context.view_layer.update()
    _walk_every_datablock()
    assert gp_name in bpy.data.objects, "Clear must be undoable"
    print("UNDO_OBSERVED debug button and Clear undo/redo clean; mode", bpy.context.mode)


def _run_an_unfolded_domain_is_written_with_a_vertex_normal_offset():
    """Изогнутый патч (лестница S1) разворачивается; смещение — по нормали каждой вершины.

    Второй патч поднят на 0.9 м: near-planar отказал бы по ширине, развёртка принимает
    (изогнутый квад из двух треугольников развёртывается точно). Меш пишется целиком
    (`MATERIALIZED 2`), а каждая вершина декали стоит от поверхности источника не дальше
    смещения: нормаль вершины на сгибе — биссектриса, расстояние до поверхности меньше.
    """

    from mathutils.bvhtree import BVHTree

    controller = _controller()
    if controller is not None:
        controller.clear()
    _reset_scene()
    source = _build_two_patch_seam(nonplanar_second_patch=True, second_patch_offset=0.9)
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    settings.envelope_debug_workers = 0
    _decal_settings().offset = 0.02
    decal = _press()
    status = _decal_settings().status
    assert status == "MATERIALIZED 2 / refused 0", status
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    depsgraph = bpy.context.evaluated_depsgraph_get()
    tree = BVHTree.FromObject(source, depsgraph)
    domain = decal.data.attributes["cftuv_domain"].data
    second = {
        vertex
        for polygon, value in zip(decal.data.polygons, domain)
        if value.value == 1
        for vertex in polygon.vertices
    }
    assert second
    distances = [
        tree.find_nearest(tuple(decal.data.vertices[index].co))[3] for index in second
    ]
    assert max(distances) <= 0.02 + 1e-5, distances
    assert min(distances) > 0.005, distances
    _assert_faces_follow_the_polygon_law(decal.data, require_quads=False, planar=False)
    return decal


def _run_a_band_chart_rescues_a_domain_the_whole_patch_refuses():
    """ПОЛОСА: пирамида (веер вокруг поднятой вершины) целиком не разворачивается, а полоса у одного шва - да.

    Один выбранный шов: носитель - три треугольника из четырёх (четвёртый дальше досягаемости 0.5 м), открытый веер
    разворачивается изометрично, кнопка строит оба домена. Красные контроли той же сцены: основание выбрано целиком -
    носитель весь патч, отказ метрики прежний и назван; alpha выше досягаемости - `REQUEST_ALPHA_EXCEEDS_CHART_REACH`.
    """

    from test_envelope_debug_bridge import _budget_refused_second_patch_offset

    settings = _settings()
    for base, alpha, expected in (
        (False, 0.25, "MATERIALIZED 2 / refused 0"),
        (True, 0.25, "MATERIALIZED 1 / refused 1 (DEVELOPABLE_STRETCH_BUDGET_EXCEEDED)"),
        (False, 0.75, "MATERIALIZED 1 / refused 1 (REQUEST_ALPHA_EXCEEDS_CHART_REACH)"),
    ):
        controller = _controller()
        if controller is not None:
            controller.clear()
        _reset_scene()
        _build_two_patch_seam(
            second_patch_apex=_budget_refused_second_patch_offset(), second_patch_whole_base=base
        )
        settings.envelope_debug_engine = "QUEUE"
        settings.envelope_debug_alpha = alpha
        settings.envelope_debug_workers = 0
        _decal_settings().offset = 0.02
        assert bpy.ops.hotspotuv.build_envelope_decal_mesh() == {"FINISHED"}
        assert _decal_settings().status == expected, (base, alpha, _decal_settings().status)
    print("BAND:", expected)


def _polygon_points(mesh, polygon):
    return [mesh.vertices[index].co.copy() for index in polygon.vertices]


def _is_concave(mesh, polygon):
    """Есть ли у грани строго правый поворот относительно её нормали Ньюэлла."""

    points = _polygon_points(mesh, polygon)
    count = len(points)
    if count < 4:
        return False
    normal = points[0] * 0.0
    for index, current in enumerate(points):
        following = points[(index + 1) % count]
        normal.x += (current.y - following.y) * (current.z + following.z)
        normal.y += (current.z - following.z) * (current.x + following.x)
        normal.z += (current.x - following.x) * (current.y + following.y)
    normal.normalize()
    return any(
        (points[index] - points[index - 1]).cross(
            points[(index + 1) % count] - points[index]
        ).dot(normal)
        < -1e-6
        for index in range(count)
    )


def _run_a_concave_polygon_is_one_face_with_the_same_uv_under_any_triangulation():
    """Корпус ядра `double_notch` на alpha 3/2: полоса с вырезом фронта соседа — невыпуклая грань."""

    from fractions import Fraction

    import mathutils

    kernel_tests = REPO_ROOT / "kernel" / "tests"
    if str(kernel_tests) not in sys.path:
        sys.path.insert(0, str(kernel_tests))
    import materialize_factories as factories
    from wavefront_cases import named_corpus

    from cftuv.envelope_production_export import MATERIALIZED, ProductionDomainResultV1
    from cftuv.envelope_production_mesh import write_decal_object
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1

    law = DecalTopologyLawV1.PLANAR_POLYGONS_V1
    batch, _frames = factories.assemble_polygon_batch(
        dict(named_corpus())["double_notch"], Fraction(3, 2), law=law
    )
    source = _fresh_scene()
    result = ProductionDomainResultV1(
        0,
        "concave",
        MATERIALIZED,
        batch,
        normal=(0.0, 0.0, 1.0),
        source_normal=(0.0, 0.0, 1.0),
        decal_topology_law=law.value,
    )
    receipt = write_decal_object(source, [result], offset=0.0, material_name="M")
    mesh = bpy.data.objects[receipt.object_name].data
    assert receipt.decal_topology_law == law.value
    assert [len(item.vertices) for item in mesh.polygons] == [
        len(face.ordered_vert_keys) for face in batch.faces
    ]
    assert len(mesh.uv_layers["UVMap"].data) == len(mesh.loops) == receipt.loops
    concave = [item for item in mesh.polygons if _is_concave(mesh, item)]
    assert len(concave) == 1, [len(item.vertices) for item in concave]
    # Площадь: Blender считает невыпуклую грань верно (сумма граней = точная площадь батча).
    position = {item.vert_key.value: item.position for item in batch.vertices}
    expected = 0.0
    for face in batch.faces:
        points = [position[key.value] for key in face.ordered_vert_keys]
        expected += abs(
            sum(
                points[index].x * points[(index + 1) % len(points)].y
                - points[(index + 1) % len(points)].x * points[index].y
                for index in range(len(points))
            )
        ) / 2.0
    assert abs(sum(item.area for item in mesh.polygons) - expected) < 1e-4, expected
    # Плоскость: все вершины граней от четырёх петель лежат в плоскости грани до 1e-5.
    for polygon in mesh.polygons:
        if len(polygon.vertices) < 4:
            continue
        origin = mesh.vertices[polygon.vertices[0]].co
        for index in polygon.vertices:
            assert abs(polygon.normal.dot(mesh.vertices[index].co - origin)) < 1e-5
    # Любая триангуляция показа даёт ту же UV-интерполяцию: карта первого треугольника
    # (положение -> UV) попадает в UV КАЖДОЙ петли той же грани.
    mesh.calc_loop_triangles()
    uv = mesh.uv_layers["UVMap"].data
    polygon = concave[0]
    triangles = [item for item in mesh.loop_triangles if item.polygon_index == polygon.index]
    assert len(triangles) == len(polygon.vertices) - 2 > 1
    assert abs(sum(item.area for item in triangles) - polygon.area) < 1e-4
    first = triangles[0]
    corners = [mesh.vertices[index].co for index in first.vertices]
    corner_uv = [mathutils.Vector((uv[loop].uv[0], uv[loop].uv[1], 0.0)) for loop in first.loops]
    for loop_index in polygon.loop_indices:
        point = mesh.vertices[mesh.loops[loop_index].vertex_index].co
        mapped = mathutils.geometry.barycentric_transform(point, *corners, *corner_uv)
        assert abs(mapped[0] - uv[loop_index].uv[0]) < 1e-4, loop_index
        assert abs(mapped[1] - uv[loop_index].uv[1]) < 1e-4, loop_index
    print("CONCAVE polygon of", len(polygon.vertices), "vertices:", len(triangles), "display triangles")


def _build_folded_two_patch_seam():
    """Пол `z = 0` и стена `x = 1` под 90°: складка внутрь комнаты, шов — общее ребро `(1,0,0)-(1,1,0)`.

    Обход таков, что общее ребро идёт в гранях в противоположные стороны (многообразие), а нормали
    обоих патчей — `+z` и `-x` — смотрят в одну сторону складки (внутрь угла).
    """

    mesh = bpy.data.meshes.new("EnvelopeFoldMesh")
    vertices = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 1.0),
        (1.0, 1.0, 1.0),
    ]
    mesh.from_pydata(vertices, (), [(0, 1, 2, 3), (1, 4, 5, 2)])
    mesh.update()
    shared = None
    for edge in mesh.edges:
        if set(edge.vertices) == {1, 2}:
            edge.use_seam = True
            shared = edge.index
    assert shared is not None
    obj = bpy.data.objects.new(SOURCE, mesh)
    bpy.context.scene.collection.objects.link(obj)
    _enter_edge_selection(obj, (shared,))
    return obj


def _run_a_fold_welds_the_shared_chain_into_single_vertices():
    controller = _controller()
    if controller is not None:
        controller.clear()
    _reset_scene()
    _build_folded_two_patch_seam()
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    settings.envelope_debug_workers = 0
    offset = 0.02
    _decal_settings().offset = offset
    decal = _press()
    status = _decal_settings().status
    assert status == "MATERIALIZED 2 / refused 0", status
    mesh = decal.data
    domain = mesh.attributes["cftuv_domain"].data
    assert {item.value for item in domain} == {0, 1}

    # Общая цепь: вершины исходника (1,0,0) и (1,1,0). Над каждой — ОДНА вершина декали,
    # в точке митры `(1 - d, y, d)`: пересечение плоскостей `z = d` (пол) и `x = 1 - d` (стена).
    shared = []
    for y in (0.0, 1.0):
        expected = (1.0 - offset, y, offset)
        hits = [
            item.index
            for item in mesh.vertices
            if all(abs(a - b) < 1e-6 for a, b in zip(item.co, expected))
        ]
        assert len(hits) == 1, (expected, hits)
        shared.append(hits[0])

    # Каждая грань лежит на сдвинутой плоскости СВОЕГО домена (и плоская).
    for polygon, value in zip(mesh.polygons, domain):
        for index in polygon.vertices:
            x, _y, z = mesh.vertices[index].co
            if value.value == 0:
                assert abs(z - offset) < 1e-5, (polygon.index, z)
            else:
                assert abs(x - (1.0 - offset)) < 1e-5, (polygon.index, x)
    _assert_faces_follow_the_polygon_law(mesh, require_quads=False)

    # Общее ребро — ребро двух граней РАЗНЫХ доменов и шов UV: дыры вдоль складки нет.
    edge = next(item for item in mesh.edges if set(item.vertices) == set(shared))
    users = [
        value.value
        for polygon, value in zip(mesh.polygons, domain)
        if set(shared) <= set(polygon.vertices)
    ]
    assert sorted(users) == [0, 1], users
    assert edge.use_seam
    # Ни одного ребра вдоль общей цепи, открытого с одной стороны.
    open_on_chain = [
        item
        for item in mesh.edges
        if set(item.vertices) <= set(shared)
        and sum(1 for polygon in mesh.polygons if set(item.vertices) <= set(polygon.vertices)) == 1
    ]
    assert not open_on_chain
    return decal


def _build_slanted_fold():
    """Одна складка 90° в одном патче: плоский квад и вертикальная стена вдоль КОСОГО ребра `(2, 0)-(2.5, 1)`.

    Шов — граничное ребро `x = 1`, откуда растёт декаль. Ребро складки не перпендикулярно полосе,
    поэтому без резки диагональ уха ленты уходит хордой в стену.
    """

    mesh = bpy.data.meshes.new("EnvelopeSlantFoldMesh")
    mesh.from_pydata(
        [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (2.5, 1.0, 0.0),
            (2.0, 0.0, 0.0),
            (2.5, 1.0, 1.0),
            (2.0, 0.0, 1.0),
        ],
        (),
        [(0, 3, 2, 1), (3, 5, 4, 2)],
    )
    mesh.update()
    seam = None
    for edge in mesh.edges:
        if set(edge.vertices) == {0, 1}:
            edge.use_seam = True
            seam = edge.index
    assert seam is not None
    obj = bpy.data.objects.new(SOURCE, mesh)
    bpy.context.scene.collection.objects.link(obj)
    _enter_edge_selection(obj, (seam,))
    return obj


def _fold_decal(*, lift_policy=None):
    """Нажатие на косой складке: `(объект декаля, исходный, наибольшее расстояние точек граней до источника)`."""

    from mathutils.bvhtree import BVHTree

    from cftuv import envelope_production_export, envelope_request_export

    controller = _controller()
    if controller is not None:
        controller.clear()
    _reset_scene()
    source = _build_slanted_fold()
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 1.3
    settings.envelope_debug_workers = 0
    _decal_settings().offset = 0.02
    saved = (
        envelope_production_export.HOST_NEAR_PLANAR_LIFT_POLICY,
        envelope_request_export.HOST_NEAR_PLANAR_LIFT_POLICY,
    )
    if lift_policy is not None:
        envelope_production_export.HOST_NEAR_PLANAR_LIFT_POLICY = lift_policy
        envelope_request_export.HOST_NEAR_PLANAR_LIFT_POLICY = lift_policy
    try:
        decal = _press()
    finally:
        envelope_production_export.HOST_NEAR_PLANAR_LIFT_POLICY = saved[0]
        envelope_request_export.HOST_NEAR_PLANAR_LIFT_POLICY = saved[1]
    assert _decal_settings().status == "MATERIALIZED 1 / refused 0", _decal_settings().status
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    tree = BVHTree.FromObject(source, bpy.context.evaluated_depsgraph_get())
    mesh = decal.data
    worst = 0.0
    for polygon in mesh.polygons:
        points = [mesh.vertices[index].co for index in polygon.vertices]
        samples = [*points, polygon.center]
        samples += [(points[i] + points[(i + 1) % len(points)]) * 0.5 for i in range(len(points))]
        worst = max(worst, *(tree.find_nearest(tuple(point))[3] for point in samples))
    return decal, source, worst


def _run_a_decal_across_a_fold_stays_on_the_surface():
    """Законы резки (`SOURCE_FACES_CLIPPED_V1` кнопки и `SOURCE_TRIANGLES_CLIPPED_V1`): куски лежат в гранях источника, хорды через складку нет."""

    from cftuv.surface_ir import HOST_NEAR_PLANAR_LIFT_POLICY, HostNearPlanarLiftPolicy

    assert HOST_NEAR_PLANAR_LIFT_POLICY is HostNearPlanarLiftPolicy.SOURCE_FACES_CLIPPED_V1
    decal, _source, worst = _fold_decal()
    sizes = [len(polygon.vertices) for polygon in decal.data.polygons]
    # Смещение вдоль нормалей вершин: точка грани отстоит от поверхности не дальше смещения.
    assert worst <= 0.02 + 1e-5, worst
    assert max(sizes) >= 4, sizes
    print("FOLD clipped:", sorted(sizes), "faces, worst distance", worst)
    # Прежний закон по треугольникам режет ту же сцену так же: складка по ребру меша, диагоналей нет.
    _by_triangles, _source, triangles_worst = _fold_decal(lift_policy=HostNearPlanarLiftPolicy.SOURCE_TRIANGLES_CLIPPED_V1)
    assert triangles_worst <= 0.02 + 1e-5, triangles_worst
    # Красный контроль: та же сцена без резки (закон `SOURCE_TRIANGLES_V1`) режет в стену.
    _plain, _source, plain_worst = _fold_decal(lift_policy=HostNearPlanarLiftPolicy.SOURCE_TRIANGLES_V1)
    assert plain_worst > 0.02 + 0.05, plain_worst
    print("FOLD plain control: worst distance", plain_worst)


def _main():
    import cftuv

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован установленной копией
        pass
    assert hasattr(bpy.ops.hotspotuv, "build_envelope_decal_mesh")
    _source, decal = _run_cold_press_builds_one_child_object()
    _run_rebuild_is_idempotent(decal)
    _run_offset_is_a_scene_setting()
    warm_digest = _run_warm_press_after_a_debug_build_reuses_the_preparations()
    _run_workers_do_not_change_the_mesh(warm_digest)
    _run_nothing_selected_is_refused_by_name()
    _run_a_zero_length_edge_is_named_selected_and_never_repaired()
    _run_a_long_source_name_never_multiplies_the_decal()
    _run_an_adapter_skip_reaches_the_status_line()
    _run_the_operator_declares_undo_with_a_named_reason()
    _run_undo_after_a_press_in_edit_mode_keeps_the_scene_consistent()
    _run_undo_after_the_debug_button_and_after_clear_keeps_the_scene_consistent()
    _run_an_unfolded_domain_is_written_with_a_vertex_normal_offset()
    _run_a_band_chart_rescues_a_domain_the_whole_patch_refuses()
    _run_a_concave_polygon_is_one_face_with_the_same_uv_under_any_triangulation()
    _run_a_fold_welds_the_shared_chain_into_single_vertices()
    _run_a_decal_across_a_fold_stays_on_the_surface()
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("ENVELOPE_PRODUCTION_MESH_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
