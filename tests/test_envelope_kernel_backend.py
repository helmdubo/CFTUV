"""Выбор бэкенда ядра в хосте (KERNEL-BACKEND): настройка, запись на домене, строка журнала, ключи кэшей.

Нативный бэкенд — переключатель, а не политика запроса: ответ побитово один. Что держат тесты:

1. УМОЛЧАНИЕ НИЧЕГО НЕ МЕНЯЕТ. `PYTHON` — ответ тот же, записи нет, строки журнала нет, память резки не сбрасывается.
2. ЗАКАЗ НАЗЫВАЕТ ИСПОЛНИТЕЛЯ. С `NATIVE` каждый посчитанный домен несёт `BackendRecordV1`; без колеса домен считает эталон,
   ответ равен ответу `PYTHON`, а запись называет причину. Запись и строка журнала едут из воркера пула.
3. КЛЮЧИ НЕ СМЕШИВАЮТСЯ. Результат одного бэкенда не берётся из кэша другого: ни в кэше ревизии, ни по содержимому.
4. СТРОКА ЖУРНАЛА: `BACKEND native 120 / python 2 (NATIVE_PORT_STALE: patch 7, 9)`.
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


@pytest.fixture(autouse=True)
def _backend_state(monkeypatch):
    """Вердикт загрузки нативного ядра и «последний бэкенд процесса» не переживают тест; колеса в процессе нет."""

    monkeypatch.setitem(sys.modules, "cftuv_native", None)
    monkeypatch.setattr(host_backend, "_LAST_BACKEND", [host_backend.DEFAULT_KERNEL_BACKEND])
    kernel_backend.refresh_native()
    yield
    kernel_backend.refresh_native()


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
    )


def _projection(run):
    return [result_projection(item) for item in run.results]


# --------------------------------------------------------------------------
# 1. Умолчание ничего не меняет
# --------------------------------------------------------------------------


def test_the_setting_defaults_to_python_and_an_unknown_name_is_refused():
    assert host_backend.DEFAULT_KERNEL_BACKEND == "PYTHON"
    assert host_backend.kernel_backend_of(SimpleNamespace()) == "PYTHON"
    assert host_backend.kernel_backend_of(SimpleNamespace(kernel_backend="NATIVE")) == "NATIVE"
    assert host_backend.normalize_kernel_backend(" native ") == "NATIVE"
    with pytest.raises(ValueError, match="unknown kernel backend"):
        host_backend.normalize_kernel_backend("RUST")
    assert [item[0] for item in host_backend.KERNEL_BACKEND_ITEMS] == ["PYTHON", "NATIVE"]
    assert host_backend.KERNEL_BACKEND_ITEMS[1][1] == "Native (Rust)"


def test_the_default_press_is_the_python_press_with_no_record_and_no_journal_line(row):
    implicit = _run(row)
    explicit = _run(row, backend="PYTHON")

    assert implicit.kernel_backend == explicit.kernel_backend == "PYTHON"
    assert _projection(implicit) == _projection(explicit)
    assert [item.content_digest for item in implicit.results] == [item.content_digest for item in explicit.results]
    assert all(item.backend_record is None for item in (*implicit.results, *explicit.results))
    assert host_backend.backend_console_lines(explicit.results, "PYTHON") == []
    assert host_backend.backend_timing_suffix(explicit.results, "PYTHON") == ""
    assert "backend" not in production.production_timing_text(explicit)


def test_python_press_never_clears_the_clip_memo(row, monkeypatch):
    from cftuv_envelope.materialize.clip_memo import MEMO

    cleared = []
    monkeypatch.setattr(MEMO, "clear", lambda: cleared.append(1))
    _run(row)
    _run(row, backend="PYTHON")
    assert cleared == []


# --------------------------------------------------------------------------
# 2. Заказ называет исполнителя
# --------------------------------------------------------------------------


def test_a_native_press_without_the_wheel_gives_the_python_answer_and_names_every_computed_domain(row):
    reference = _run(row)
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
    assert len(lines) == 1 and lines[0].startswith("[CFTUV][Production] BACKEND native 0 / python " + str(ROW))
    assert "NATIVE_UNAVAILABLE: patch" in lines[0] or "NATIVE_NOT_REACHED: patch" in lines[0]
    assert f" | backend native 0 / python {ROW}" in production.production_timing_text(native)


def test_a_native_press_after_a_python_press_recomputes_and_never_reads_the_python_cache(row):
    controller = EnvelopeDebugSessionController()
    python = _run(row, controller=controller)
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

    back = _run(row, backend="PYTHON", controller=controller)
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
    assert _projection(run) == _projection(_run(row))

    tasks.clear()
    _run(row, controller=EnvelopeDebugSessionController(), workers=2)
    assert tasks and {task.backend for task in tasks} == {"PYTHON"}


def test_the_task_default_is_python_and_a_task_pickles_with_its_backend():
    task = pool_module.DomainTaskV1(1, 0, "d", None, None, "0.25", frozenset())
    assert task.backend == "PYTHON"
    assert pickle.loads(pickle.dumps(pool_module.DomainTaskV1(1, 0, "d", None, None, "0.25", frozenset(), backend="NATIVE"))).backend == "NATIVE"


def test_a_backend_switch_clears_the_clip_memo_once_per_switch(row, monkeypatch):
    from cftuv_envelope.materialize.clip_memo import MEMO

    cleared = []
    monkeypatch.setattr(MEMO, "clear", lambda: cleared.append(host_backend._LAST_BACKEND[0]))
    _run(row)
    _run(row, backend="NATIVE")
    _run(row, backend="NATIVE", controller=EnvelopeDebugSessionController())
    _run(row, backend="PYTHON", controller=EnvelopeDebugSessionController())
    assert cleared == ["PYTHON", "NATIVE"]  # метка читается ДО записи: первый сброс — при уходе с PYTHON, второй — при возврате


def test_an_installed_native_module_changes_the_identity_not_the_answer(row):
    module = types.ModuleType("cftuv_native")
    module.NativePortStale = module.NativePortUnsupported = module.NativeUnsupportedPython = RuntimeError
    module.coverage_at = module.clip_geometry = module.native_status = lambda *a, **k: None
    module.native_version = lambda: "0.1.0"
    sys.modules["cftuv_native"] = module
    kernel_backend.refresh_native()

    assert host_backend.backend_identity_of("NATIVE") == "NATIVE:0.1.0"
    assert host_backend.backend_identity_of("PYTHON") == "PYTHON"
    assert _projection(_run(row, backend="NATIVE")) == _projection(_run(row))


# --------------------------------------------------------------------------
# 3. Ключи не смешиваются
# --------------------------------------------------------------------------


def test_the_execution_identity_adds_the_backend_to_the_code_fingerprint_and_leaves_the_fingerprint_alone():
    kernel_fingerprint, host_fingerprint = content_key.code_identity()
    assert content_key.execution_identity("PYTHON") == (kernel_fingerprint, host_fingerprint, "PYTHON")
    assert content_key.execution_identity("NATIVE") == (kernel_fingerprint, host_fingerprint, "NATIVE:unavailable")
    assert len(content_key.code_identity()) == 2


def test_the_content_key_and_the_result_slot_carry_the_backend(row):
    first = content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT")
    assert first == content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", "PYTHON")
    assert first != content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", "NATIVE:0.1.0")
    assert content_key.result_slot("0.25", "UV", "TOPOLOGY", "LIFT", "NATIVE:0.1.0") != content_key.result_slot(
        "0.25", "UV", "TOPOLOGY", "LIFT", "NATIVE:0.2.0"
    )


def test_a_native_press_after_a_python_press_does_not_take_results_from_the_content_store(row):
    """Хранилище по содержимому: ключ и слот результата несут бэкенд, поэтому другой бэкенд не читает чужую запись."""

    controller = EnvelopeDebugSessionController()
    _run(row, controller=controller)

    from cftuv.envelope_production_export import _slot

    python_run = SimpleNamespace(
        alpha_text="0.25", uv_policy_id="UV", topology_law="T", backend_id="PYTHON"
    )
    native_run = SimpleNamespace(
        alpha_text="0.25", uv_policy_id="UV", topology_law="T", backend_id="NATIVE:0.1.0"
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
    summary = host_backend.backend_summary(results, "NATIVE")

    assert (summary.native, summary.python, summary.mixed, summary.cached) == (118, 2, 0, 0)
    assert host_backend.backend_text(summary) == "native 118 / python 2 (NATIVE_PORT_STALE: patch 7, 9)"
    assert host_backend.backend_console_lines(results, "NATIVE") == [
        "[CFTUV][Production] BACKEND native 118 / python 2 (NATIVE_PORT_STALE: patch 7, 9)"
    ]


def test_the_journal_line_counts_mixed_and_cached_domains_and_caps_the_patch_list():
    results = [_record("mixed", patch, "NATIVE_PORT_UNSUPPORTED") for patch in range(15)]
    results.append(SimpleNamespace(patch_id=99, placement=PLACEMENT_CACHED, backend_record=None))
    results.append(SimpleNamespace(patch_id=98, placement=PLACEMENT_PARENT, backend_record=None))  # отказ входа: исполнителя не было
    text = host_backend.backend_text(host_backend.backend_summary(results, "NATIVE"))

    assert text.startswith("native 0 / python 0 / mixed 15 / cached 1 (NATIVE_PORT_UNSUPPORTED: patch 0, 1, 2")
    assert "... (+3))" in text
    assert text.count("patch") == 1


def test_a_domain_that_called_no_native_operation_is_named_not_reached_in_the_line():
    from cftuv_envelope.backend import BackendRecordV1

    result = SimpleNamespace(patch_id=4, placement=PLACEMENT_PARENT, backend_record=BackendRecordV1("NATIVE", 0, 0))
    assert host_backend.backend_text(host_backend.backend_summary([result], "NATIVE")) == (
        "native 0 / python 1 (NATIVE_NOT_REACHED: patch 4)"
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
