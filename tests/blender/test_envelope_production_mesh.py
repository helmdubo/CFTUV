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
9. отмена: оператор — только REGISTER (шаг BMesh в EDIT-режиме не отслеживает
   создание объекта), а после отката в OBJECT-режиме сцена цела и следующее
   нажатие работает. `ed.undo` в EDIT-режиме фоновый Blender отказывает
   («context is incorrect»), поэтому проверяется именно эта последовательность.

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


def _assert_faces_follow_the_polygon_law(mesh, *, require_quads):
    """Грани меша — треугольники, четырёхгранники и многоугольники, и каждая грань от 4 петель плоская."""

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


def _press_named(source):
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


def _run_the_operator_is_register_only_with_a_named_reason():
    from cftuv.envelope_production_operator import (
        HOTSPOTUV_OT_BuildEnvelopeDecalMesh,
        UNDO_DROPPED_REASON,
    )

    assert set(HOTSPOTUV_OT_BuildEnvelopeDecalMesh.bl_options) == {"REGISTER"}
    assert "BMesh" in UNDO_DROPPED_REASON


def _walk_every_datablock():
    """Обход как при перерисовке: висячая ссылка здесь роняет Blender или бросает."""

    for item in bpy.data.objects:
        assert item.data is not None, item.name
        _ = (item.name, item.type, item.parent)
    for mesh in bpy.data.meshes:
        _ = (mesh.name, len(mesh.polygons), len(mesh.uv_layers))


def _run_undo_leaves_a_consistent_scene_and_the_next_press_works():
    source = _fresh_scene()
    seam = [edge.index for edge in source.data.edges if edge.use_seam]
    assert seam
    bpy.ops.object.mode_set(mode="OBJECT")
    # Фоновый Blender включает систему отмены явным шагом, а отменять она начинает
    # со ВТОРОГО: первый `undo_push` только инициализирует (`ed.undo.poll()`
    # остаётся ложным), второй — настоящий шаг.
    bpy.ops.ed.undo_push(message="initialize")
    bpy.ops.ed.undo_push(message="scene ready")
    assert bpy.ops.ed.undo.poll()
    bpy.ops.object.mode_set(mode="EDIT")
    _press_named(source)
    assert len(_decal_objects()) == 1
    bpy.ops.object.mode_set(mode="OBJECT")

    assert bpy.ops.ed.undo() == {"FINISHED"}
    _walk_every_datablock()
    survivors = _decal_objects()
    print("UNDO_OBSERVED after_undo decals:", [item.name for item in survivors])
    source = bpy.data.objects[SOURCE]
    assert source.data is not None and len(source.data.polygons) == 2
    assert all(item.data is not None for item in survivors)

    try:
        redone = bpy.ops.ed.redo()
    except RuntimeError as exc:  # нечего повторять — тоже честный исход
        redone = str(exc)
    print("UNDO_OBSERVED redo:", redone, [item.name for item in _decal_objects()])
    print(
        "UNDO_OBSERVED meshes:",
        sorted((item.name, item.users) for item in bpy.data.meshes),
    )
    _walk_every_datablock()

    source = bpy.data.objects[SOURCE]
    from test_envelope_debug_bridge import _enter_edge_selection

    bpy.context.view_layer.objects.active = source
    if source.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    _enter_edge_selection(source, seam)
    decal = _press_named(source)
    assert decal.parent == source and len(_decal_objects()) == 1
    assert decal.data.uv_layers.get("UVMap") is not None
    _walk_every_datablock()


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
    _assert_faces_follow_the_polygon_law(decal.data, require_quads=False)
    return decal


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
    _run_a_long_source_name_never_multiplies_the_decal()
    _run_an_adapter_skip_reaches_the_status_line()
    _run_the_operator_is_register_only_with_a_named_reason()
    _run_undo_leaves_a_consistent_scene_and_the_next_press_works()
    _run_an_unfolded_domain_is_written_with_a_vertex_normal_offset()
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("ENVELOPE_PRODUCTION_MESH_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
