"""Выбор бэкенда ядра в хосте (KERNEL-BACKEND): настройка, запись на домене, строка журнала, ключи кэшей.

Нативный бэкенд — переключатель, а не политика запроса: ответ побитово один. Что держат тесты:

1. УМОЛЧАНИЕ ПРОДУКТА — `NATIVE` (решение владельца 2026-10-07), и оно названо ОДНИМ местом (`DEFAULT_KERNEL_BACKEND`). Явный `PYTHON` —
   ответ тот же, записи нет, строки журнала нет, память резки не сбрасывается. ЭТАЛОН ОТВЕТА В ЭТОМ ФАЙЛЕ — ВСЕГДА ЯВНЫЙ `PYTHON`
   (`_python`): прогон «по умолчанию» эталоном быть не может, умолчание не Python.
2. ЗАКАЗ НАЗЫВАЕТ ИСПОЛНИТЕЛЯ. С `NATIVE` каждый посчитанный домен несёт `BackendRecordV1`; без колеса домен считает эталон,
   ответ равен ответу `PYTHON`, а запись называет причину. Запись и строка журнала едут из воркера пула.
3. КЛЮЧИ НЕ СМЕШИВАЮТСЯ. Результат одного бэкенда не берётся из кэша другого: ни в кэше ревизии, ни по содержимому.
4. СТРОКА ЖУРНАЛА: `BACKEND coverage/clip native 120 / python 2 (NATIVE_PORT_STALE: patch 7, 9); skeleton python`.
5. ЗАДАЧА ПУЛА НЕСЁТ БЭКЕНД, а внешний воркер берёт каталог `cftuv_native` у родителя.
"""

from __future__ import annotations

import pickle
import sys
import types
from types import SimpleNamespace

import pytest

from cftuv import envelope_content_key as content_key
from cftuv import envelope_domain_pool as pool_module
from cftuv import envelope_kernel_backend as host_backend
from cftuv import envelope_production_export as production
from cftuv.envelope_debug_session import EnvelopeDebugSessionController
from cftuv.envelope_production_export import PLACEMENT_CACHED, PLACEMENT_PARENT
from test_envelope_production_content import ROW, _press, pool, row  # noqa: F401 - фикстуры и помощник прогона
from content_equivalence import result_projection

from cftuv_envelope import backend as kernel_backend

#: Отпечаток сборки подставного нативного ядра (`native_build_id()`).
BUILD_ID = "ab" * 32


@pytest.fixture(autouse=True)
def _backend_state(monkeypatch):
    """Вердикт загрузки нативного ядра и «последний бэкенд процесса» не переживают тест; колеса в процессе нет."""

    monkeypatch.setitem(sys.modules, "cftuv_native", None)
    monkeypatch.setattr(host_backend, "_LAST_BACKEND", [host_backend.DEFAULT_KERNEL_BACKEND])
    kernel_backend.refresh_native()
    yield
    kernel_backend.refresh_native()


def _fake_native(**attributes):
    """Подставной `cftuv_native` с тем, что хост требует от колеса (`backend._REQUIRED`); `attributes` переопределяют и дополняют."""

    class Stale(RuntimeError):
        pass

    class Unsupported(RuntimeError):
        pass

    class UnsupportedPython(RuntimeError):
        pass

    class NativeDivisionDiverged(ArithmeticError):
        pass

    module = types.ModuleType("cftuv_native")
    module.NativePortStale, module.NativePortUnsupported, module.NativeUnsupportedPython = Stale, Unsupported, UnsupportedPython
    module.NativeDivisionDiverged = NativeDivisionDiverged
    module.NATIVE_REFUSALS = (Stale, UnsupportedPython, Unsupported, NativeDivisionDiverged)
    module.coverage_at = module.clip_geometry = module.native_status = lambda *a, **k: None
    module.native_version = lambda: "0.1.0"
    module.native_build_id = lambda: BUILD_ID
    for name, value in attributes.items():
        setattr(module, name, value)
    sys.modules["cftuv_native"] = module
    kernel_backend.refresh_native()
    return module


def _run(bundle, *, backend=None, controller=None, workers=0):
    controller = controller or EnvelopeDebugSessionController()
    if backend is None:
        return _press(bundle, controller, workers=workers)
    from cftuv.envelope_production_export import run_production

    return run_production(
        controller,
        bundle,
        frozenset(range(ROW)),
        0.25,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=workers,
        kernel_backend=backend,
        skeleton_backend="PYTHON",  # здесь изолируется coverage/clip; прогон без аргументов проверяет оба умолчания
    )


def _python(bundle, **kwargs):
    """Прогон на ЯВНОМ `PYTHON`: эталон ответа. Умолчание продукта — `NATIVE`, поэтому «прогон без заказа» эталоном быть не может."""

    return _run(bundle, backend="PYTHON", **kwargs)


def _projection(run):
    return [result_projection(item) for item in run.results]


# --------------------------------------------------------------------------
# 1. Умолчание продукта — NATIVE; явный PYTHON молчит
# --------------------------------------------------------------------------


def test_the_setting_defaults_to_native_a_chosen_python_stays_python_and_an_unknown_name_is_refused():
    assert host_backend.DEFAULT_KERNEL_BACKEND == "NATIVE"
    assert host_backend.kernel_backend_of(SimpleNamespace()) == "NATIVE"
    assert host_backend.kernel_backend_of(SimpleNamespace(kernel_backend="NATIVE")) == "NATIVE"
    assert host_backend.kernel_backend_of(SimpleNamespace(kernel_backend="PYTHON")) == "PYTHON"
    assert host_backend.normalize_kernel_backend(" native ") == "NATIVE"
    with pytest.raises(ValueError, match="unknown kernel backend"):
        host_backend.normalize_kernel_backend("RUST")
    # порядок пунктов — формат хранения в сцене (индекс): PYTHON = 0, NATIVE = 1; сцена с выбранным `PYTHON` читает его и после смены умолчания
    assert [item[0] for item in host_backend.KERNEL_BACKEND_ITEMS] == ["PYTHON", "NATIVE"]
    assert host_backend.KERNEL_BACKEND_ITEMS[1][1] == "Native (Rust)"
    assert "Default" in host_backend.KERNEL_BACKEND_ITEMS[1][2] and "Default" not in host_backend.KERNEL_BACKEND_ITEMS[0][2]


def test_the_default_press_is_the_native_press_and_names_who_computed_while_an_explicit_python_press_stays_silent(row):
    default = _run(row)
    python = _python(row)

    assert default.kernel_backend == "NATIVE" and python.kernel_backend == "PYTHON"
    # ответ побитово тот же, каким бэкендом ни считали
    assert _projection(default) == _projection(python)
    assert [item.content_digest for item in default.results] == [item.content_digest for item in python.results]
    # умолчание называет исполнителя: колеса в процессе нет, поэтому каждый домен назван откатом `NATIVE_UNAVAILABLE` (или не дошёл до операции)
    records = [item.backend_record for item in default.results]
    assert all(record is not None and record.requested == "NATIVE" and record.ran == "python" for record in records)
    assert all(record.outcomes and set(record.outcomes) <= {"NATIVE_UNAVAILABLE", "NATIVE_NOT_REACHED"} for record in records)
    line = host_backend.backend_console_lines(default.results, default.kernel_backend)[0]
    assert line.startswith("[CFTUV][Production] BACKEND coverage/clip native 0 / python " + str(ROW)) and f"; skeleton native 0 / python {ROW} (NATIVE_UNAVAILABLE:" in line
    assert "NATIVE_UNAVAILABLE: patch" in line or "NATIVE_NOT_REACHED: patch" in line
    assert f" | backend native 0 / python {ROW}" in production.production_timing_text(default)
    # явный `PYTHON`: записи нет, строки журнала нет, в строке панели бэкенда нет
    assert all(item.backend_record is None for item in python.results)
    assert host_backend.backend_console_lines(python.results, "PYTHON", "PYTHON") == []
    assert host_backend.backend_timing_suffix(python.results, "PYTHON", "PYTHON") == ""
    assert "backend" not in production.production_timing_text(python)


@pytest.mark.parametrize("name", ["PYTHON", "NATIVE"])
def test_a_press_of_the_backend_the_process_already_runs_never_clears_the_clip_memo(row, monkeypatch, name):
    from cftuv_envelope.materialize.clip_memo import MEMO

    cleared = []
    monkeypatch.setattr(host_backend, "_LAST_BACKEND", [name])
    monkeypatch.setattr(MEMO, "clear", lambda: cleared.append(1))
    _run(row, backend=name)
    _run(row, backend=name, controller=EnvelopeDebugSessionController())
    assert cleared == []


def test_every_backend_default_of_the_host_is_the_one_named_constant():
    """Умолчание бэкенда названо ОДНИМ местом (`DEFAULT_KERNEL_BACKEND`): литерал `"PYTHON"`/`"NATIVE"` в умолчании параметра, поля либо свойства сцены — дефект.

    Иначе прогон, задача пула, запись живой ширины и настройка сцены разошлись бы молча (одна часть продукта считает Python, другая Rust).
    Исключение одно: `backend_id` (идентичность бэкенда для ключа) без значения — `None`, и идентичность берётся у умолчания.
    """

    import ast
    from pathlib import Path

    names = {"backend", "kernel_backend", "backend_id", "skeleton_backend"}
    found: list = []
    for path in sorted((Path(__file__).resolve().parents[1] / "cftuv").glob("*.py")):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                arguments = node.args
                positional = [*arguments.posonlyargs, *arguments.args]
                pairs = list(zip(positional[len(positional) - len(arguments.defaults) :], arguments.defaults))
                pairs += [(arg, default) for arg, default in zip(arguments.kwonlyargs, arguments.kw_defaults) if default is not None]
                found += [(path.name, arg.arg, ast.unparse(default)) for arg, default in pairs if arg.arg in names]
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id in names:
                value = node.value
                if value is None and isinstance(node.annotation, ast.Call):  # свойство сцены: `kernel_backend: EnumProperty(..., default=...)`
                    value = next((item.value for item in node.annotation.keywords if item.arg == "default"), None)
                if value is not None:
                    found.append((path.name, node.target.id, ast.unparse(value)))
    # стадия скелета — своё единственное место умолчания (`DEFAULT_SKELETON_BACKEND`): покрытие с резкой и скелет переводятся на Rust порознь
    allowed = {"skeleton_backend": {"DEFAULT_SKELETON_BACKEND"}}
    plain = {"DEFAULT_KERNEL_BACKEND", "None"}
    unnamed = [item for item in found if item[2] not in allowed.get(item[1], plain)]
    assert not unnamed, unnamed
    # правило не пустое: оно видит каждое место проводки (прогон, задача пула, запись живой ширины, свойство сцены, ключи кэшей)
    seen = {(file, name) for file, name, _default in found}
    for site in (
        ("envelope_kernel_backend.py", "backend"),
        ("envelope_production_export.py", "kernel_backend"),
        ("envelope_production_export.py", "backend"),
        ("envelope_domain_pool.py", "backend"),
        ("envelope_width_live.py", "kernel_backend"),
        ("envelope_production_operator.py", "kernel_backend"),
        ("envelope_content_key.py", "backend"),
        ("envelope_content_key.py", "backend_id"),
        # стадия скелета: те же места проводки
        ("envelope_kernel_backend.py", "skeleton_backend"),
        ("envelope_production_export.py", "skeleton_backend"),
        ("envelope_domain_pool.py", "skeleton_backend"),
        ("envelope_queue_export.py", "skeleton_backend"),
        ("envelope_width_live.py", "skeleton_backend"),
        ("envelope_production_operator.py", "skeleton_backend"),
        ("envelope_content_key.py", "skeleton_backend"),
    ):
        assert site in seen, site


# --------------------------------------------------------------------------
# 2. Заказ называет исполнителя
# --------------------------------------------------------------------------


def test_a_native_press_without_the_wheel_gives_the_python_answer_and_names_every_computed_domain(row):
    reference = _python(row)
    native = _run(row, backend="NATIVE")

    assert native.kernel_backend == "NATIVE"
    assert _projection(native) == _projection(reference)
    assert [item.content_digest for item in native.results] == [item.content_digest for item in reference.results]
    records = [item.backend_record for item in native.results]
    assert all(record is not None and record.requested == "NATIVE" for record in records)
    assert all(record.ran == "python" and record.native_calls == 0 for record in records)
    # Точки диспетчеризации ядра подключены не везде: «колеса нет» и «домен не позвал ни одной операции» — разные названные исходы.
    allowed = {"NATIVE_UNAVAILABLE", "NATIVE_NOT_REACHED"}
    assert all(record.outcomes and set(record.outcomes) <= allowed for record in records)
    lines = host_backend.backend_console_lines(native.results, "NATIVE")
    assert len(lines) == 1 and lines[0].startswith("[CFTUV][Production] BACKEND coverage/clip native 0 / python " + str(ROW))
    assert "NATIVE_UNAVAILABLE: patch" in lines[0] or "NATIVE_NOT_REACHED: patch" in lines[0]
    assert f" | backend native 0 / python {ROW}" in production.production_timing_text(native)


def test_no_press_replaces_a_name_in_the_kernel_and_the_backends_give_the_same_answer(row):
    """Диспетчеры подключены в самом ядре: ни `PYTHON`, ни `NATIVE` ничего не ставит в процесс (подмены имён нет), воркер и главный процесс равны с первого домена."""

    from cftuv_envelope.materialize import clip
    from cftuv_envelope.wavefront import coverage

    oracles = (clip.clip_geometry, coverage._coverage_at)
    for name in ("install_dispatch", "uninstall_dispatch", "dispatch_installed"):
        assert not hasattr(kernel_backend, name) and not hasattr(host_backend, name), name
    python = _python(row)
    native = _run(row, backend="NATIVE", controller=EnvelopeDebugSessionController())
    again = _python(row, controller=EnvelopeDebugSessionController())
    assert (clip.clip_geometry, coverage._coverage_at) == oracles
    assert clip.clip_geometry is not kernel_backend.clip_compute
    assert _projection(again) == _projection(native) == _projection(python)


def test_a_native_press_after_a_python_press_recomputes_and_never_reads_the_python_cache(row):
    controller = EnvelopeDebugSessionController()
    python = _python(row, controller=controller)
    assert python.counter(production.PRODUCTION_RESULT_CACHE_MISS) == ROW

    native = _run(row, backend="NATIVE", controller=controller)
    assert (native.counter(production.PRODUCTION_RESULT_CACHE_HIT), native.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (0, ROW)
    assert all(item.placement != PLACEMENT_CACHED for item in native.results)

    again = _run(row, backend="NATIVE", controller=controller)
    assert (again.counter(production.PRODUCTION_RESULT_CACHE_HIT), again.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (ROW, 0)
    assert all(item.placement == PLACEMENT_CACHED for item in again.results)
    # Домен из кэша не считался в этом прогоне: в счёт исполнителей он не идёт, а назван отдельно.
    line = host_backend.backend_console_lines(again.results, "NATIVE")[0]
    assert f"native 0 / python 0 / cached {ROW}" in line

    back = _python(row, controller=controller)
    assert (back.counter(production.PRODUCTION_RESULT_CACHE_HIT), back.counter(production.PRODUCTION_RESULT_CACHE_MISS)) == (ROW, 0)


def test_the_backend_travels_with_the_task_and_the_record_comes_back_from_the_worker(row, pool):
    tasks: list = []
    original = pool.run

    def spy(batch):
        tasks.extend(batch)
        return original(batch)

    pool.run = spy
    run = _run(row, backend="NATIVE", workers=2)

    assert tasks and {task.backend for task in tasks} == {"NATIVE"}
    assert all(item.backend_record is not None and item.backend_record.requested == "NATIVE" for item in run.results)
    assert all(item.placement == production.PLACEMENT_WORKER for item in run.results)
    assert _projection(run) == _projection(_python(row))

    # заказ без слова — умолчание продукта, оно едет в задачу воркера; явный `PYTHON` едет как есть
    tasks.clear()
    _run(row, controller=EnvelopeDebugSessionController(), workers=2)
    assert tasks and {task.backend for task in tasks} == {"NATIVE"}
    tasks.clear()
    _python(row, controller=EnvelopeDebugSessionController(), workers=2)
    assert tasks and {task.backend for task in tasks} == {"PYTHON"}


def test_the_task_default_is_the_product_default_and_a_task_pickles_with_its_backend():
    task = pool_module.DomainTaskV1(1, 0, "d", None, None, "0.25", frozenset())
    assert task.backend == "NATIVE" == host_backend.DEFAULT_KERNEL_BACKEND
    assert pickle.loads(pickle.dumps(task)).backend == "NATIVE"
    assert pickle.loads(pickle.dumps(pool_module.DomainTaskV1(1, 0, "d", None, None, "0.25", frozenset(), backend="PYTHON"))).backend == "PYTHON"


def test_a_backend_switch_clears_the_clip_memo_once_per_switch(row, monkeypatch):
    from cftuv_envelope.materialize.clip_memo import MEMO

    cleared = []
    monkeypatch.setattr(host_backend, "_LAST_BACKEND", ["PYTHON"])
    monkeypatch.setattr(MEMO, "clear", lambda: cleared.append(host_backend._LAST_BACKEND[0]))
    _python(row)
    _run(row, backend="NATIVE")
    _run(row, backend="NATIVE", controller=EnvelopeDebugSessionController())
    _run(row, controller=EnvelopeDebugSessionController())  # без заказа — умолчание продукта, тот же `NATIVE`: сброса нет
    _python(row, controller=EnvelopeDebugSessionController())
    assert cleared == ["PYTHON", "NATIVE"]  # метка читается ДО записи: первый сброс — при уходе с PYTHON, второй — при возврате


def test_an_installed_native_module_changes_the_identity_not_the_answer(row):
    module = _fake_native()

    assert host_backend.backend_identity_of("NATIVE", "PYTHON") == f"NATIVE:{BUILD_ID}"
    assert host_backend.backend_identity_of("PYTHON", "PYTHON") == "PYTHON"
    assert _projection(_run(row, backend="NATIVE")) == _projection(_python(row))
    # другая сборка при том же номере колеса — другая идентичность (ключи кэшей не читают результат прежней сборки)
    module.native_build_id = lambda: "cd" * 32
    kernel_backend.refresh_native()
    assert host_backend.backend_identity_of("NATIVE", "PYTHON") == "NATIVE:" + "cd" * 32


# --------------------------------------------------------------------------
# 3. Ключи не смешиваются
# --------------------------------------------------------------------------


def test_the_execution_identity_adds_the_backend_to_the_code_fingerprint_and_leaves_the_fingerprint_alone():
    kernel_fingerprint, host_fingerprint = content_key.code_identity()
    assert content_key.execution_identity("PYTHON", "PYTHON") == (kernel_fingerprint, host_fingerprint, "PYTHON")
    assert content_key.execution_identity("NATIVE", "PYTHON") == (kernel_fingerprint, host_fingerprint, "NATIVE:unavailable")
    assert len(content_key.code_identity()) == 2
    # с колесом идентичность исполнения несёт отпечаток сборки (`native_build_id()`), а не номер колеса
    _fake_native()
    assert content_key.execution_identity("NATIVE", "PYTHON") == (kernel_fingerprint, host_fingerprint, f"NATIVE:{BUILD_ID}")


def test_the_content_key_and_the_result_slot_carry_the_backend(row):
    first = content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", "PYTHON")
    assert first != content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", f"NATIVE:{BUILD_ID}")
    # без идентичности слот несёт идентичность умолчания продукта (без колеса — `NATIVE:unavailable`), а не молчаливый `PYTHON`
    default = content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT")
    assert default == content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", host_backend.backend_identity_of(host_backend.DEFAULT_KERNEL_BACKEND))
    assert default == content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", "NATIVE:unavailable|skeleton=NATIVE:unavailable") != first
    assert content_key.execution_identity() == content_key.execution_identity(host_backend.DEFAULT_KERNEL_BACKEND)
    assert content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", f"NATIVE:{BUILD_ID}") != content_key.result_slot(
        "0.25", "UV", "TOPOLOGY", "LIFT", "NATIVE:" + "cd" * 32
    )


def test_a_native_press_after_a_python_press_does_not_take_results_from_the_content_store(row):
    """Хранилище по содержимому: ключ и слот результата несут бэкенд, поэтому другой бэкенд не читает чужую запись."""

    controller = EnvelopeDebugSessionController()
    _python(row, controller=controller)

    from cftuv.envelope_production_export import _slot

    python_run = SimpleNamespace(
        alpha_text="0.25", uv_policy_id="UV", topology_law="T", backend_id="PYTHON"
    )
    native_run = SimpleNamespace(
        alpha_text="0.25", uv_policy_id="UV", topology_law="T", backend_id=f"NATIVE:{BUILD_ID}"
    )
    assert _slot(python_run) != _slot(native_run)

    native = _run(row, backend="NATIVE", controller=controller)
    assert native.counter(production.PRODUCTION_CONTENT_RESULT_REUSED) == 0
    assert native.counter(production.PRODUCTION_RESULT_CACHE_MISS) == ROW


# --------------------------------------------------------------------------
# 4. Строка журнала
# --------------------------------------------------------------------------


def _record(ran, patch, *outcomes):
    from cftuv_envelope.backend import BackendRecordV1

    calls = {"native": (3, 0), "python": (0, 3), "mixed": (2, 1)}[ran]
    return SimpleNamespace(
        patch_id=patch,
        placement=PLACEMENT_PARENT,
        backend_record=BackendRecordV1("NATIVE", *calls, tuple((name, "clip", 1, "detail") for name in outcomes)),
    )


def test_the_journal_line_names_native_python_mixed_cached_and_the_patches_of_every_fallback():
    results = [_record("native", patch) for patch in range(120)]
    results += [_record("python", 7, "NATIVE_PORT_STALE"), _record("python", 9, "NATIVE_PORT_STALE")]
    results.pop(7)
    results.pop(8)  # патчи 7 и 9 заменены записями с откатом
    summary = host_backend.backend_summary(results, "NATIVE", "PYTHON")

    assert (summary.native, summary.python, summary.mixed, summary.cached) == (118, 2, 0, 0)
    assert host_backend.backend_text(summary) == "coverage/clip native 118 / python 2 (NATIVE_PORT_STALE: patch 7, 9); skeleton python"
    assert host_backend.backend_console_lines(results, "NATIVE", "PYTHON") == [
        "[CFTUV][Production] BACKEND coverage/clip native 118 / python 2 (NATIVE_PORT_STALE: patch 7, 9); skeleton python"
    ]


def test_the_journal_line_counts_mixed_and_cached_domains_and_caps_the_patch_list():
    results = [_record("mixed", patch, "NATIVE_PORT_UNSUPPORTED") for patch in range(15)]
    results.append(SimpleNamespace(patch_id=99, placement=PLACEMENT_CACHED, backend_record=None))
    results.append(SimpleNamespace(patch_id=98, placement=PLACEMENT_PARENT, backend_record=None))  # отказ входа: исполнителя не было
    text = host_backend.backend_text(host_backend.backend_summary(results, "NATIVE", "PYTHON"))

    assert text.startswith("coverage/clip native 0 / python 0 / mixed 15 / cached 1 (NATIVE_PORT_UNSUPPORTED: patch 0, 1, 2")
    assert "... (+3))" in text
    assert text.count("patch") == 1


def test_a_domain_that_called_no_native_operation_is_named_not_reached_in_the_line():
    from cftuv_envelope.backend import BackendRecordV1

    result = SimpleNamespace(patch_id=4, placement=PLACEMENT_PARENT, backend_record=BackendRecordV1("NATIVE", 0, 0))
    assert host_backend.backend_text(host_backend.backend_summary([result], "NATIVE", "PYTHON")) == (
        "coverage/clip native 0 / python 1 (NATIVE_NOT_REACHED: patch 4); skeleton python"
    )


# --------------------------------------------------------------------------
# 5. Внешний воркер берёт каталог нативного ядра у родителя
# --------------------------------------------------------------------------


def test_an_external_worker_gets_the_native_package_directory_when_the_parent_finds_it(monkeypatch, tmp_path):
    package = tmp_path / "modules" / "cftuv_native"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("", encoding="utf-8")
    real = pool_module.package_directory
    monkeypatch.setattr(
        pool_module, "package_directory", lambda name: str(package) if name == "cftuv_native" else real(name)
    )
    pool = pool_module.DomainPool(1, external_python=sys.executable)
    _python, specification, _host = pool._external_plan()
    assert str(tmp_path / "modules") in specification["sys_path"]
    assert specification["after_stdlib"] is True

    monkeypatch.setattr(pool_module, "package_directory", lambda name: None if name == "cftuv_native" else real(name))
    _python, without, _host = pool._external_plan()
    assert str(tmp_path / "modules") not in without["sys_path"]
    assert pool_module.OPTIONAL_HOST_PACKAGES == ("cftuv_native",)


# --------------------------------------------------------------------------
# 6. Поздний отказ порта после частичных эффектов называет домен
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "outcome, detail",
    [
        ("NATIVE_PARTIAL_EFFECTS_REFUSED", "NativePortUnsupported: late"),
        ("NATIVE_DIVISION_DIVERGED", "NativeDivisionDiverged: the generic division fallback did not finish"),
    ],
)
def test_a_domain_refusal_of_the_native_core_names_the_domain_whatever_the_stage_returned(outcome, detail):
    @host_backend.with_kernel_backend
    def produce(patch_id, domain_id):
        # стадию, проглотившую исключение, имитирует результат, вернувшийся как ни в чём не бывало
        kernel_backend._SCOPE.get().note_refusal("clip", kernel_backend.BackendOutcomeV1(outcome), detail)
        return production._refusal(patch_id, domain_id, "MATERIALIZED", "", 1.5)

    refused = produce(7, "domain-7", backend="NATIVE")

    assert (refused.outcome, refused.patch_id, refused.domain_id, refused.seconds) == (outcome, 7, "domain-7", 1.5)
    assert detail in refused.detail
    assert refused.backend_record.outcomes == (outcome,) and refused.backend_record.ran == "python"
    # под явным `PYTHON` ничего не записывается и ничего не называется; без слова — умолчание продукта, оно называет исполнителя
    @host_backend.with_kernel_backend
    def plain(patch_id, domain_id):
        return production._refusal(patch_id, domain_id, "MATERIALIZED", "")

    assert plain(7, "domain-7", backend="PYTHON", skeleton_backend="PYTHON").outcome == "MATERIALIZED" and plain(7, "domain-7", backend="PYTHON", skeleton_backend="PYTHON").backend_record is None
    assert plain(7, "domain-7").backend_record is not None and plain(7, "domain-7").backend_record.requested == "NATIVE"


def test_a_press_whose_native_coverage_refuses_late_names_every_domain_instead_of_computing_on_dirty_state(row):
    module = _fake_native()
    Unsupported = module.NativePortUnsupported
    asked = []

    def late(partition, alpha, work_budget=None, store=None, traces=None):
        asked.append(1)
        work_budget.gcd_operations += 1  # порт успел заплатить, а потом отказал по имени
        raise Unsupported("a sort of 64 nodes or more")

    module.coverage_at = late
    kernel_backend.refresh_native()

    native = _run(row, backend="NATIVE")

    assert asked
    assert {item.outcome for item in native.results} == {"NATIVE_PARTIAL_EFFECTS_REFUSED"}
    assert all(item.batch is None and item.detail.startswith("coverage: Unsupported: a sort of 64 nodes") for item in native.results)
    line = host_backend.backend_console_lines(native.results, "NATIVE")[0]
    assert "NATIVE_PARTIAL_EFFECTS_REFUSED: patch" in line
    # тот же заказ с честным откатом (ничего не тронуто) даёт ответ эталона
    def clean(partition, alpha, work_budget=None, store=None, traces=None):
        raise Unsupported("declined before any effect")

    module.coverage_at = clean
    kernel_backend.refresh_native()
    assert _projection(_run(row, backend="NATIVE", controller=EnvelopeDebugSessionController())) == _projection(_python(row))


def test_a_press_whose_native_division_diverges_refuses_every_domain_by_name_and_never_asks_the_oracle(row):
    """`NativeDivisionDiverged`: эталон на этом входе не завершился бы, поэтому домен отказан `NATIVE_DIVISION_DIVERGED`, а не откатом на Python."""

    module = _fake_native()
    asked = []

    def diverge(partition, alpha, work_budget=None, store=None, traces=None):
        asked.append(1)
        raise module.NativeDivisionDiverged("the generic division fallback did not finish")

    module.coverage_at = diverge
    kernel_backend.refresh_native()

    native = _run(row, backend="NATIVE")

    assert asked
    assert {item.outcome for item in native.results} == {"NATIVE_DIVISION_DIVERGED"}
    assert all(item.batch is None and item.detail.startswith("coverage: NativeDivisionDiverged") for item in native.results)
    assert all(item.backend_record.outcomes == ("NATIVE_DIVISION_DIVERGED",) for item in native.results)
    assert "NATIVE_DIVISION_DIVERGED: patch" in host_backend.backend_console_lines(native.results, "NATIVE")[0]


def test_a_press_whose_native_core_breaks_down_before_any_effect_gives_the_python_answer_and_names_the_defect(row):
    """Дефект порта до эффектов (вход, который расширение не несёт; паника Rust): домен считает эталон, ответ тот же, а в строке журнала — `NATIVE_INTERNAL_ERROR`."""

    module = _fake_native()

    def panic(partition, alpha, work_budget=None, store=None, traces=None):
        raise RuntimeError("Session.coverage_at: native panic: index out of bounds")

    module.coverage_at = panic
    kernel_backend.refresh_native()

    native = _run(row, backend="NATIVE", controller=EnvelopeDebugSessionController())

    assert _projection(native) == _projection(_python(row))
    assert all("NATIVE_INTERNAL_ERROR" in item.backend_record.outcomes for item in native.results if item.placement != PLACEMENT_CACHED)
    line = host_backend.backend_console_lines(native.results, "NATIVE")[0]
    assert "NATIVE_INTERNAL_ERROR: patch" in line


# --------------------------------------------------------------------------
# 7. Поток живой ширины задаёт бэкенд сам
# --------------------------------------------------------------------------


def test_a_worker_thread_gets_its_backend_from_its_own_argument_and_not_from_the_thread_that_started_it(row):
    import threading

    results: dict = {}

    def work(name, backend):
        results[name] = _run(row, backend=backend, controller=EnvelopeDebugSessionController())

    with kernel_backend.use_backend("NATIVE"):
        thread = threading.Thread(target=work, args=("python_inside_native", "PYTHON"))
        thread.start()
        thread.join()
    with kernel_backend.use_backend("PYTHON"):
        thread = threading.Thread(target=work, args=("native_inside_python", "NATIVE"))
        thread.start()
        thread.join()

    assert all(item.backend_record is None for item in results["python_inside_native"].results)
    assert all(item.backend_record is not None and item.backend_record.requested == "NATIVE" for item in results["native_inside_python"].results)
    assert _projection(results["python_inside_native"]) == _projection(results["native_inside_python"])


def test_the_live_width_thread_passes_the_backend_of_the_last_build_to_run_production():
    import ast
    import dataclasses
    from pathlib import Path

    from cftuv.envelope_width_live import LastProductionBuildV1

    assert {item.name: item.default for item in dataclasses.fields(LastProductionBuildV1)}["kernel_backend"] == "NATIVE"
    assert {item.name: item.default for item in dataclasses.fields(LastProductionBuildV1)}["skeleton_backend"] == "NATIVE"
    path =Path(__file__).resolve().parents[1] / "cftuv" / "envelope_width_live.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    begin = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "_begin")
    compute = next(node for node in ast.walk(begin) if isinstance(node, ast.FunctionDef) and node.name == "compute")
    # снято в главном потоке из записи кнопки, а не прочитано потоком из настроек
    captured = [
        node
        for node in begin.body
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id == "kernel_backend"
    ]
    assert len(captured) == 1 and ast.unparse(captured[0].value) == "record.kernel_backend"
    skeleton = [
        node
        for node in begin.body
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id == "skeleton_backend"
    ]
    assert len(skeleton) == 1 and ast.unparse(skeleton[0].value) == "record.skeleton_backend"
    calls = [node for node in ast.walk(compute) if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "run_production"]
    assert len(calls) == 1
    passed = {item.arg: ast.unparse(item.value) for item in calls[0].keywords}
    assert passed.get("kernel_backend") == "kernel_backend" and passed.get("skeleton_backend") == "skeleton_backend"
    assert "record" not in {node.id for node in ast.walk(compute) if isinstance(node, ast.Name)}
