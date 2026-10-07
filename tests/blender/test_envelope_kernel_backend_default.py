"""Blender 4.5 background smoke: умолчание бэкенда ядра — NATIVE (решение владельца 2026-10-07), и это видно в самом Blender.

Утверждения стоят на свойстве сцены, записи бэкенда на каждом домене, строке журнала и дайджесте меша:

1. СВОЙСТВО СЦЕНЫ. Умолчание RNA `hotspotuv_decal_mesh.kernel_backend` — `NATIVE`; в сцене, где его не трогали, оно не хранится
   (`is_property_set` ложно), и `kernel_backend_of` читает `NATIVE`.
2. СТАРАЯ СЦЕНА НЕ ПЕРЕПИСЫВАЕТСЯ. Выбранный `PYTHON` хранится индексом 0 (так он лежит в старом файле) и после смены умолчания читается
   как `PYTHON`; присвоение `PYTHON` (то же значение, что было умолчанием) тоже хранится. Порядок пунктов — формат хранения: `PYTHON` = 0,
   `NATIVE` = 1. Миграции нет и не нужна: Blender хранит только присвоенное.
3. КНОПКА БЕЗ ЗАКАЗА считает `NATIVE`: прогон, запись живой ширины и запись бэкенда у каждого домена называют `NATIVE`, строка
   `[CFTUV][Production] BACKEND native ...` печатается. С колесом, пока порт не устарел, считает нативное ядро; без колеса — домен
   считает Python, и строка называет `NATIVE_UNAVAILABLE` (колесо прячется `sys.modules['cftuv_native'] = None`; настоящий
   `ModuleNotFoundError` проверяет запуск этого файла с убранным путём пользовательских модулей).
4. ОТВЕТ ТОТ ЖЕ. Меш (дайджест того, что лежит в Blender) одинаков для умолчания с колесом, умолчания без колеса и явного `PYTHON`;
   явный `PYTHON` молчит (записи нет, строки журнала нет).

Прогон (без `--factory-startup`: sympy в 4.5 живёт в профиле пользователя):
blender --background --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_KERNEL_BACKEND_DEFAULT_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import contextlib
import io
from pathlib import Path
import sys

import bpy


REPO_ROOT = Path(__file__).resolve().parents[2]


def _unregister_installed_copy() -> None:
    """Установленная копия аддона регистрирует СВОИ классы и умолчания: снимается до загрузки дерева репозитория."""

    installed = sys.modules.get("cftuv")
    if installed is not None and hasattr(installed, "unregister"):
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - копия могла быть не зарегистрирована
            print("installed cftuv unregister:", type(exc).__name__, exc)


_unregister_installed_copy()
for path in (REPO_ROOT, REPO_ROOT / "kernel" / "src"):
    if str(path) in sys.path:
        sys.path.remove(str(path))
    sys.path.insert(0, str(path))
for module_name in tuple(sys.modules):
    if module_name in {"cftuv", "cftuv_envelope"} or module_name.startswith(("cftuv.", "cftuv_envelope.")):
        del sys.modules[module_name]

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_envelope_debug_bridge import (  # noqa: E402
    _build_two_patch_seam,
    _reset_scene,
)


SOURCE = "EnvelopeTwoPatch"
DECAL = SOURCE + ".CFTUV_Decal"
BACKEND_LINE = "[CFTUV][Production] BACKEND native"


def _decal_settings():
    return bpy.context.scene.hotspotuv_decal_mesh


def _controller():
    return bpy.context.window_manager._cftuv_envelope_debug_session


def _register_repo_tree() -> None:
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (REPO_ROOT / "cftuv").resolve(), cftuv.__file__
    assert Path(cftuv_envelope.__file__).resolve().parent == (REPO_ROOT / "kernel" / "src" / "cftuv_envelope").resolve()
    cftuv.register()
    assert hasattr(bpy.ops.hotspotuv, "build_envelope_decal_mesh")


def _run_the_scene_property_defaults_to_native_and_stores_only_what_was_assigned():
    from cftuv import envelope_kernel_backend as host_backend

    scene = bpy.context.scene
    settings = scene.hotspotuv_decal_mesh
    property_ = settings.bl_rna.properties["kernel_backend"]
    # 1. умолчание RNA и непротронутая сцена
    assert property_.default == "NATIVE" == host_backend.DEFAULT_KERNEL_BACKEND, property_.default
    assert [item.identifier for item in property_.enum_items] == ["PYTHON", "NATIVE"]
    assert not settings.is_property_set("kernel_backend") and "kernel_backend" not in settings.keys()
    assert settings.kernel_backend == "NATIVE" and host_backend.kernel_backend_of(settings) == "NATIVE"

    # 2. старая сцена: хранимое не переписывается умолчанием; сцены создаются свежими, ни одна из них не трогалась
    old_python = bpy.data.scenes.new("KernelBackendOldPython")
    old_native = bpy.data.scenes.new("KernelBackendOldNative")
    assigned = bpy.data.scenes.new("KernelBackendAssignedPython")
    untouched = bpy.data.scenes.new("KernelBackendUntouched")
    try:
        old_python.hotspotuv_decal_mesh["kernel_backend"] = 0  # как лежит выбранный PYTHON в файле: индекс пункта
        old_native.hotspotuv_decal_mesh["kernel_backend"] = 1
        assigned.hotspotuv_decal_mesh.kernel_backend = "PYTHON"  # присвоено то же, что было прежним умолчанием
        assert old_python.hotspotuv_decal_mesh.kernel_backend == "PYTHON"
        assert host_backend.kernel_backend_of(old_python.hotspotuv_decal_mesh) == "PYTHON"
        assert old_native.hotspotuv_decal_mesh.kernel_backend == "NATIVE"
        assert assigned.hotspotuv_decal_mesh.is_property_set("kernel_backend")
        assert assigned.hotspotuv_decal_mesh["kernel_backend"] == 0, "PYTHON хранится индексом 0"
        assert assigned.hotspotuv_decal_mesh.kernel_backend == "PYTHON"
        assert host_backend.kernel_backend_of(assigned.hotspotuv_decal_mesh) == "PYTHON"
        # непротронутая сцена читает новое умолчание, и чтение ничего не записывает
        assert untouched.hotspotuv_decal_mesh.kernel_backend == "NATIVE"
        assert not untouched.hotspotuv_decal_mesh.is_property_set("kernel_backend")
        assert "kernel_backend" not in untouched.hotspotuv_decal_mesh.keys()
    finally:
        for item in (old_python, old_native, assigned, untouched):
            bpy.data.scenes.remove(item)


def _fresh_scene():
    _reset_scene()
    controller = _controller()
    if controller is not None:
        controller.clear()
    for mesh in list(bpy.data.meshes):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    source = _build_two_patch_seam()
    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    settings.envelope_debug_workers = 0
    _decal_settings().offset = 0.02
    _decal_settings().material_name = "CFTUV_Decal"
    return source


def _mesh_digest() -> str:
    from cftuv.envelope_production_mesh import mesh_content_digest

    return mesh_content_digest(bpy.data.objects[DECAL].data)


def _press():
    """Нажатие на пустой сессии: `(прогон, напечатанное, дайджест меша)`. Прогон снимается подменой `run_production` (оператор берёт её при вызове)."""

    from cftuv import envelope_production_export as export

    if _controller() is not None:
        _controller().clear()
    runs: list = []
    original = export.run_production

    def spy(*args, **kwargs):
        run = original(*args, **kwargs)
        runs.append(run)
        return run

    export.run_production = spy
    printed = io.StringIO()
    try:
        with contextlib.redirect_stdout(printed):
            result = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    finally:
        export.run_production = original
    assert result == {"FINISHED"}, (result, _decal_settings().status)
    assert len(runs) == 1
    return runs[0], printed.getvalue(), _mesh_digest()


def _computed(run):
    return [item for item in run.results if item.placement != "cached"]


def _run_the_default_press_is_native_names_the_executor_and_gives_the_same_mesh():
    from cftuv_envelope import backend as kernel_backend

    settings = _decal_settings()
    assert not settings.is_property_set("kernel_backend")  # до сих пор ни одна часть смока не выбирала бэкенд

    # --- умолчание, колесо как есть в этом процессе
    kernel_backend.refresh_native()
    status = kernel_backend.native_status()
    run, printed, digest = _press()
    assert run.kernel_backend == "NATIVE", run.kernel_backend
    assert _controller().width_build.kernel_backend == "NATIVE"  # запись живой ширины несёт тот же бэкенд
    records = [item.backend_record for item in _computed(run)]
    assert records and all(record is not None and record.requested == "NATIVE" for record in records)
    assert BACKEND_LINE in printed, printed
    if status.available:
        assert any(record.ran in ("native", "mixed") for record in records), [record.as_record() for record in records]
        assert not any("NATIVE_UNAVAILABLE" in record.outcomes for record in records)
        print("DEFAULT_WITH_WHEEL:", [record.ran for record in records], status.version, status.build_id[:10])
    else:
        # колеса нет либо порт устарел: считал Python, и каждый такой домен назван
        assert all(record.ran == "python" and record.outcomes for record in records)
        print("DEFAULT_WHEEL_NOT_AVAILABLE_HERE:", status.coverage, status.clip, sorted({o for r in records for o in r.outcomes}))
    default_digest = digest

    # --- умолчание, колесо спрятано: настоящий откат на Python, названный `NATIVE_UNAVAILABLE`
    hidden = sys.modules.get("cftuv_native", False)
    sys.modules["cftuv_native"] = None
    kernel_backend.refresh_native()
    try:
        run, printed, digest = _press()
    finally:
        if hidden is False:
            sys.modules.pop("cftuv_native", None)
        else:
            sys.modules["cftuv_native"] = hidden
        kernel_backend.refresh_native()
    assert run.kernel_backend == "NATIVE"
    records = [item.backend_record for item in _computed(run)]
    assert records and all(record.requested == "NATIVE" and record.ran == "python" for record in records)
    assert any("NATIVE_UNAVAILABLE" in record.outcomes for record in records)
    assert all(set(record.outcomes) <= {"NATIVE_UNAVAILABLE", "NATIVE_NOT_REACHED"} for record in records)
    assert BACKEND_LINE + " 0 / python" in printed and "NATIVE_UNAVAILABLE: patch" in printed, printed
    assert digest == default_digest, "ответ без колеса равен ответу с колесом"

    # --- явный PYTHON: молчит, и ответ тот же
    settings.kernel_backend = "PYTHON"
    assert settings.is_property_set("kernel_backend") and settings["kernel_backend"] == 0
    run, printed, digest = _press()
    assert run.kernel_backend == "PYTHON" and _controller().width_build.kernel_backend == "PYTHON"
    assert all(item.backend_record is None for item in run.results)
    assert "BACKEND" not in printed, printed
    assert digest == default_digest, "ответ явного PYTHON равен ответу умолчания"
    settings["kernel_backend"] = 1  # возврат на NATIVE тем же путём, каким его хранит файл
    assert settings.kernel_backend == "NATIVE"


def _main():
    _register_repo_tree()
    _run_the_scene_property_defaults_to_native_and_stores_only_what_was_assigned()
    _fresh_scene()
    _run_the_default_press_is_native_names_the_executor_and_gives_the_same_mesh()
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("ENVELOPE_KERNEL_BACKEND_DEFAULT_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
