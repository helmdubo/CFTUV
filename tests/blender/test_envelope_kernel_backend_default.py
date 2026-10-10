"""Blender 4.5 background smoke: главный переключатель бэкенда ядра — один, по умолчанию NATIVE (решение владельца 2026-10-07), и это видно в самом Blender.

Утверждения стоят на свойстве сцены, записи бэкенда на каждом домене, строке журнала, дайджесте меша и файле .blend:

1. СВОЙСТВО СЦЕНЫ. Умолчание RNA `hotspotuv_decal_mesh.kernel_backend` — `NATIVE`; в сцене, где его не трогали, оно не хранится
   (`is_property_set` ложно), и `kernel_backend_of` читает `NATIVE`. Отдельного свойства стадии скелета в RNA нет.
2. СТАРАЯ СЦЕНА НЕ ПЕРЕПИСЫВАЕТСЯ. Выбранный `PYTHON` хранится индексом 0 (так он лежит в старом файле) и после смены умолчания читается
   как `PYTHON`; присвоение `PYTHON` (то же значение, что было умолчанием) тоже хранится. Порядок пунктов — формат хранения: `PYTHON` = 0,
   `NATIVE` = 1.
3. КНОПКА БЕЗ ЗАКАЗА считает `NATIVE` на ВСЕХ стадиях: прогон, запись живой ширины и запись бэкенда у каждого домена называют `NATIVE` для покрытия и резки, скелета и вложения,
   строка `[CFTUV][Production] BACKEND native: coverage/clip ..., skeleton ..., embedding ...` печатается. С колесом, пока порт не устарел, считает нативное ядро; без колеса —
   домен считает Python, и строка называет `NATIVE_UNAVAILABLE` (колесо прячется `sys.modules['cftuv_native'] = None`).
4. ОТВЕТ ТОТ ЖЕ. Меш (дайджест того, что лежит в Blender) одинаков для умолчания с колесом, умолчания без колеса и явного `PYTHON`; явный `PYTHON` молчит на ВСЕХ стадиях
   (записи нет, строки журнала нет, прогон, запись живой ширины и порядки стадий — `PYTHON`).
5. МИГРАЦИЯ. Прежняя настройка стадии скелета (`skeleton_backend`) удалена; в сохранённых сценах её ключ ещё лежит индексом. Файл .blend во временной папке хранит каждое из девяти
   сочетаний (`kernel_backend`: не задан / PYTHON / NATIVE) x (прежний `skeleton_backend`: нет / PYTHON / NATIVE); после загрузки файла главный переключатель — `PYTHON`, если
   явный `PYTHON` лежал в ЛЮБОЙ из двух настроек, иначе `NATIVE` (не задан — умолчание), прежний ключ снят. Выбор владельца в списке снимает прежний ключ сразу, а кнопка в сцене
   с прежним `PYTHON` считает `PYTHON` на всех стадиях.

Прогон (без `--factory-startup`: sympy в 4.5 живёт в профиле пользователя):
blender --background --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_KERNEL_BACKEND_DEFAULT_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import contextlib
import io
import itertools
import tempfile
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
BACKEND_LINE = "[CFTUV][Production] BACKEND native: coverage/clip"


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
    # 1. умолчание RNA и непротронутая сцена; единственная настройка бэкенда в сцене
    assert property_.default == "NATIVE" == host_backend.DEFAULT_KERNEL_BACKEND, property_.default
    assert [item.identifier for item in property_.enum_items] == ["PYTHON", "NATIVE"]
    assert not settings.is_property_set("kernel_backend") and "kernel_backend" not in settings.keys()
    assert settings.kernel_backend == "NATIVE" and host_backend.kernel_backend_of(settings) == "NATIVE"
    assert [name for name in settings.bl_rna.properties.keys() if "backend" in name] == ["kernel_backend"]
    assert not hasattr(settings, "skeleton_backend")

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


def _every_stage(run, name):
    """Прогон и запись живой ширины называют `name` на покрытии и резке, скелете и вложении."""

    assert (run.kernel_backend, run.skeleton_backend, run.embedding_backend) == (name,) * 3, (run.kernel_backend, run.skeleton_backend, run.embedding_backend)
    record = _controller().width_build
    assert (record.kernel_backend, record.skeleton_backend, record.embedding_backend) == (name,) * 3


def _run_the_default_press_is_native_on_every_stage_names_the_executor_and_gives_the_same_mesh():
    from cftuv_envelope import backend as kernel_backend

    settings = _decal_settings()
    assert not settings.is_property_set("kernel_backend")  # до сих пор ни одна часть смока не выбирала бэкенд

    # --- умолчание, колесо как есть в этом процессе
    kernel_backend.refresh_native()
    status = kernel_backend.native_status()
    run, printed, digest = _press()
    _every_stage(run, "NATIVE")
    records = [item.backend_record for item in _computed(run)]
    assert records and all(record is not None for record in records)
    assert all((record.requested, record.skeleton_requested, record.embedding_requested) == ("NATIVE",) * 3 for record in records)
    assert BACKEND_LINE in printed and ", skeleton " in printed and ", embedding " in printed, printed
    if status.available and status.skeleton_available:
        assert any(record.ran in ("native", "mixed") for record in records), [record.as_record() for record in records]
        assert not any("NATIVE_UNAVAILABLE" in record.outcomes for record in records)
        assert all(record.skeleton_ran == "native" for record in records), [record.as_record() for record in records]
        print("DEFAULT_WITH_WHEEL:", [record.ran for record in records], status.version, status.build_id[:10])
    else:
        # колеса нет либо порт устарел: считал Python, и каждый такой домен назван
        assert all(record.ran == "python" and record.outcomes for record in records)
        print("DEFAULT_WHEEL_NOT_AVAILABLE_HERE:", status.coverage, status.clip, sorted({o for r in records for o in r.outcomes}))
    default_digest = digest

    # --- умолчание, колесо спрятано: настоящий откат на Python на КАЖДОЙ стадии, названный `NATIVE_UNAVAILABLE`
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
    _every_stage(run, "NATIVE")
    records = [item.backend_record for item in _computed(run)]
    assert records and all(record.requested == "NATIVE" and record.ran == "python" for record in records)
    assert any("NATIVE_UNAVAILABLE" in record.outcomes for record in records)
    assert all(set(record.outcomes) <= {"NATIVE_UNAVAILABLE", "NATIVE_NOT_REACHED"} for record in records)
    assert all(record.skeleton_requested == "NATIVE" and record.skeleton_outcomes == ("NATIVE_UNAVAILABLE",) for record in records)
    assert BACKEND_LINE + " 0, skeleton 0, embedding 0 | python: coverage/clip" in printed, printed
    assert ", skeleton " in printed and "(NATIVE_UNAVAILABLE: patch" in printed and printed.count("[CFTUV][Production] BACKEND ") == 1, printed
    assert digest == default_digest, "ответ без колеса равен ответу с колесом"

    # --- явный PYTHON: ОДИН переключатель, молчит на всех стадиях, и ответ тот же
    settings.kernel_backend = "PYTHON"
    assert settings.is_property_set("kernel_backend") and settings["kernel_backend"] == 0
    run, printed, digest = _press()
    _every_stage(run, "PYTHON")
    assert all(item.backend_record is None for item in run.results)
    assert "BACKEND" not in printed, printed
    assert digest == default_digest, "ответ явного PYTHON равен ответу умолчания"
    settings["kernel_backend"] = 1  # возврат на NATIVE тем же путём, каким его хранит файл
    assert settings.kernel_backend == "NATIVE"
    run, printed, digest = _press()
    _every_stage(run, "NATIVE")
    assert digest == default_digest


def _run_the_old_skeleton_key_in_the_active_scene_folds_into_the_switch_at_the_button():
    from cftuv import envelope_kernel_backend as host_backend

    settings = _decal_settings()
    settings.property_unset("kernel_backend")
    assert not settings.is_property_set("kernel_backend")
    settings["skeleton_backend"] = 0  # сцена, сохранённая прежней версией с выбранным PYTHON для скелета
    assert host_backend.kernel_backend_of(settings) == "PYTHON" and settings.kernel_backend == "NATIVE"  # чтение учитывает ключ, список ещё не перенесён
    run, printed, _digest = _press()
    _every_stage(run, "PYTHON")
    assert "BACKEND" not in printed and all(item.backend_record is None for item in run.results)
    assert "skeleton_backend" not in settings.keys() and settings.kernel_backend == "PYTHON" and settings["kernel_backend"] == 0
    settings.kernel_backend = "NATIVE"  # дальше решает владелец
    assert host_backend.kernel_backend_of(settings) == "NATIVE"
    settings.property_unset("kernel_backend")


def _run_a_choice_in_the_list_beats_a_not_yet_folded_old_key():
    from cftuv import envelope_kernel_backend as host_backend

    scene = bpy.data.scenes.new("KernelBackendChoiceBeatsOldKey")
    try:
        group = scene.hotspotuv_decal_mesh
        group["skeleton_backend"] = 0
        assert host_backend.kernel_backend_of(group) == "PYTHON"
        group.kernel_backend = "NATIVE"  # владелец выбрал сам: update-обработчик свойства снимает прежний ключ
        assert "skeleton_backend" not in group.keys() and host_backend.kernel_backend_of(group) == "NATIVE"
    finally:
        bpy.data.scenes.remove(scene)


def _run_a_saved_file_folds_each_combination_of_the_two_old_settings_into_the_master_switch():
    """Временный .blend хранит девять сочетаний; после загрузки файла главный переключатель — `PYTHON` при явном `PYTHON` в любой из двух настроек, иначе `NATIVE`."""

    from cftuv import envelope_kernel_backend as host_backend

    expected: dict = {}
    for kernel, legacy in itertools.product((None, 0, 1), (None, 0, 1)):
        name = f"Fold_k{kernel}_s{legacy}"
        group = bpy.data.scenes.new(name).hotspotuv_decal_mesh
        if kernel is not None:
            group["kernel_backend"] = kernel
        if legacy is not None:
            group["skeleton_backend"] = legacy
        expected[name] = (kernel, legacy, "PYTHON" if 0 in (kernel, legacy) else "NATIVE")
        assert host_backend.kernel_backend_of(group) == expected[name][2], (name, expected[name])  # до записи: чтение уже верно
    with tempfile.TemporaryDirectory() as folder:
        path = str(Path(folder) / "legacy_backend_combinations.blend")
        bpy.ops.wm.save_as_mainfile(filepath=path, copy=True)  # копия во временной папке; открытый файл не трогается
        bpy.ops.wm.open_mainfile(filepath=path)  # обработчик загрузки (`load_post`) переносит старый ключ
    for name, (kernel, legacy, master) in expected.items():
        group = bpy.data.scenes[name].hotspotuv_decal_mesh
        assert "skeleton_backend" not in group.keys(), (name, "прежний ключ снят")
        assert group.kernel_backend == master and host_backend.kernel_backend_of(group) == master, (name, master)
        assert group.is_property_set("kernel_backend") == (kernel is not None or legacy == 0), name  # переносится только явный PYTHON
        if master == "PYTHON":
            assert group["kernel_backend"] == 0
        print("FOLDED:", name, "->", master)
    from cftuv.envelope_production_operator import fold_scene_settings

    assert fold_scene_settings() == 0  # повтор ничего не находит


def _main():
    _register_repo_tree()
    _run_the_scene_property_defaults_to_native_and_stores_only_what_was_assigned()
    _fresh_scene()
    _run_the_default_press_is_native_on_every_stage_names_the_executor_and_gives_the_same_mesh()
    _run_the_old_skeleton_key_in_the_active_scene_folds_into_the_switch_at_the_button()
    _run_a_choice_in_the_list_beats_a_not_yet_folded_old_key()
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    _run_a_saved_file_folds_each_combination_of_the_two_old_settings_into_the_master_switch()
    print("ENVELOPE_KERNEL_BACKEND_DEFAULT_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
