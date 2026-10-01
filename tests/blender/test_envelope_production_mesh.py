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
   батча + нормаль · смещение.

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
    return [item for item in bpy.data.objects if item.name.endswith(".CFTUV_Decal")]


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
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("ENVELOPE_PRODUCTION_MESH_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
