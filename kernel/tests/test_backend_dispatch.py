"""Выбор бэкенда ядра (`cftuv_envelope.backend`): Python-эталон по умолчанию, нативное ядро — только именованно.

Утверждений восемь, и каждое стоит на проверке:

1. СТАТУС. Без `cftuv_native` статус — именованный `unavailable` с причиной, а не исключение; с ним — статус шима как есть; идентичность бэкенда несёт
   `native_build_id()` (отпечаток сборки), а не номер колеса. Шим без `NATIVE_REFUSALS` и `native_build_id` недоступен по имени.
2. PYTHON == ЭТАЛОН. Точки диспетчеризации под `PYTHON` (и вне блока) побитово равны вызову эталона: ответ домена, дайджесты,
   нормали и ЦЕНА (`EXACT_WORK_*`). Покрытие подключено в незакреплённых файлах (`conveyor.py`, `step.py`), резка — в `cut_domain`
   (`run_clip(backend.clip_compute, ...)`); имена в модулях ядра не подменяются.
3. NATIVE, КОТОРЫЙ СЧИТАЕТ. Нативное ядро, равное эталону, даёт тот же ответ и ту же цену, а журнал домена называет его
   исполнителем.
4. ОТКАТ ИМЕНОВАН. Члены `cftuv_native.NATIVE_REFUSALS` (`NativePortStale`, `NativeUnsupportedPython`, `NativePortUnsupported`), отсутствие колеса, вызов
   с `traces` и дефект порта до эффектов (`TypeError("cftuv_native: ...")`, паника Rust) — домен считает эталон, ответ тот же, а журнал несёт имя
   причины и счёт. `NativeDivisionDiverged` эталону не отдаётся: эталон на этом входе не завершился бы, домен отказан по имени. Тихого отката нет.
5. ИСХОД ЭТАЛОНА И ЧУЖОЕ ИСКЛЮЧЕНИЕ НЕ ГЛОТАЮТСЯ. Исход эталона нативный вызов применяет с частичными эффектами эталона и уходит вызывающему как есть;
   всё, что не названо выше, — тоже.
6. ТОЧКИ ПОДКЛЮЧЕНИЯ там, где о них говорит отчёт: покрытие подключено в `conveyor.py` и `step.py`, резка — в `clip.py`; закреплённый `coverage.py` зовёт эталон;
   обёртка `covered_at` повторяет `coverage.coverage_at`.
7. ПЕРВЫЙ ЗАКАЗ NATIVE из двух потоков: загрузка нативного ядра идёт под замком, подмены имён нет.
8. СТРАХОВОЧНЫЙ ПОЯС. Отказ, после которого видимое состояние сдвинулось, — не откат, а отказ домена; чистые отказы порта пояс не трогает.
"""

from __future__ import annotations

import re
import sys
import threading
import time
import types
from fractions import Fraction
from pathlib import Path

import pytest

import developable_factories as df
from developable_route import materialize_developable

from cftuv_envelope import backend
from cftuv_envelope import exact_sqrt_sum as exact
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import exact_work_budget
from cftuv_envelope.materialize import clip
from cftuv_envelope.materialize.clip_memo import memo_disabled
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.wavefront import coverage

POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
ROUTE = ("r0a", "r0b")
LAWS = {
    "triangles": NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1,
    "faces": NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
}
KERNEL = Path(backend.__file__).resolve().parent
BUILD_ID = "ab" * 32


@pytest.fixture(autouse=True)
def _backend_state_is_not_shared():
    """Вердикт загрузки нативного ядра живёт в процессе: тест не оставляет его следующему."""

    backend.refresh_native()
    yield
    backend.refresh_native()


class Stale(RuntimeError):
    pass


class Unsupported(RuntimeError):
    pass


class UnsupportedPython(RuntimeError):
    pass


class Diverged(ArithmeticError):
    pass


class FutureRefusal(RuntimeError):
    """Член `NATIVE_REFUSALS`, которого хост по имени не знает."""


def install_native(monkeypatch, *, coverage_at=None, clip_geometry=None, version="9.9.9", build_id=BUILD_ID, status=None, extra_refusals=()):
    """Подставной `cftuv_native` в `sys.modules`; возвращает модуль и список вызовов `[(операция, ...)]`."""

    calls: list = []
    module = types.ModuleType("cftuv_native")
    module.NativePortStale = Stale
    module.NativePortUnsupported = Unsupported
    module.NativeUnsupportedPython = UnsupportedPython
    module.NativeDivisionDiverged = Diverged
    module.NATIVE_REFUSALS = (Stale, UnsupportedPython, Unsupported, Diverged, *extra_refusals)
    module.native_version = lambda: version
    module.native_build_id = lambda: build_id
    module.native_status = lambda: dict(status or {"coverage": "available", "clip": "available"})

    def covered(*args, **kwargs):
        calls.append("coverage")
        return coverage_at(*args, **kwargs)

    def clipped(*args, **kwargs):
        calls.append("clip")
        return clip_geometry(*args, **kwargs)

    module.coverage_at = covered
    module.clip_geometry = clipped
    monkeypatch.setitem(sys.modules, "cftuv_native", module)
    backend.refresh_native()
    return module, calls


def no_native(monkeypatch):
    monkeypatch.setitem(sys.modules, "cftuv_native", None)
    backend.refresh_native()


def oracles():
    """Эталоны `(покрытие, резка)` — их зовёт тест и ими отвечает подставное нативное ядро (имена в модулях ядра не подменяются никогда)."""

    return coverage._coverage_at, clip.clip_geometry


def build(name, alpha, lift_law):
    make, _first, _saturated = FIXTURES[name]
    result, prepared = materialize_developable(
        make(), ROUTE, alpha=alpha, decal_topology_law=POLYGONS, near_planar_lift_law=lift_law
    )
    assert result.is_materialized, result.detail
    return result, prepared


FIXTURES = {"fold": (df.fold_strip, "3.5", "6"), "slant": (df.slant_fold, "3.0", "6")}


def answer(result):
    """Всё, что составляет ответ домена, ВМЕСТЕ с ценой: байты батча, дайджесты, нормали, диагностики, счётчики `EXACT_WORK_*`."""

    return (
        result.outcome.value,
        result.detail,
        tuple(result.counters),
        tuple(result.diagnostics),
        result.content_digest,
        result.offset_normals_digest,
        tuple(result.vertex_normals),
        result.offset_normal_law,
        canonical_json_bytes(result.batch),
    )


def baseline(monkeypatch, name, alpha, lift_law):
    """Ответ без диспетчера вообще: резка зовёт эталон напрямую (память резки выключена: резка считается, а не берётся)."""

    real = clip.run_clip

    def direct(_compute, *args, **kwargs):
        return real(clip.clip_geometry, *args, **kwargs)

    with monkeypatch.context() as scope:
        scope.setattr(clip, "run_clip", direct)
        with memo_disabled():
            return answer(build(name, alpha, lift_law)[0])


# --------------------------------------------------------------------------
# 1. Статус
# --------------------------------------------------------------------------


def test_backend_names_are_a_closed_set_and_an_unknown_name_is_refused():
    assert backend.normalize_backend("python") is backend.KernelBackendV1.PYTHON
    assert backend.normalize_backend(" NATIVE ") is backend.KernelBackendV1.NATIVE
    assert backend.normalize_backend(backend.KernelBackendV1.NATIVE) is backend.KernelBackendV1.NATIVE
    for bad in ("", "RUST", None, 3):
        with pytest.raises(ValueError, match="unknown kernel backend"):
            backend.normalize_backend(bad)


def test_an_absent_native_module_is_a_named_status_and_not_an_exception(monkeypatch):
    no_native(monkeypatch)
    status = backend.native_status()
    assert (status.coverage, status.clip, status.version, status.build_id) == (backend.UNAVAILABLE, backend.UNAVAILABLE, "", "")
    assert not status.available
    assert "cftuv_native" in status.detail
    assert status.as_record()["coverage"] == "unavailable"
    assert backend.backend_identity("PYTHON") == "PYTHON"
    assert backend.backend_identity("NATIVE") == "NATIVE:unavailable"


def test_a_module_without_the_named_refusals_is_unavailable_by_name(monkeypatch):
    module = types.ModuleType("cftuv_native")
    module.coverage_at = module.clip_geometry = module.native_status = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, "cftuv_native", module)
    backend.refresh_native()
    status = backend.native_status()
    assert status.coverage == status.clip == "unavailable"
    assert "NativePortStale" in status.detail and "NATIVE_REFUSALS" in status.detail and "native_build_id" in status.detail


@pytest.mark.parametrize("missing", ["NATIVE_REFUSALS", "native_build_id", "NativeDivisionDiverged"])
def test_a_wheel_older_than_the_refusal_contract_is_unavailable_and_names_what_it_lacks(monkeypatch, missing):
    module, _calls = install_native(monkeypatch)
    delattr(module, missing)
    backend.refresh_native()
    status = backend.native_status()
    assert status.coverage == status.clip == "unavailable" and missing in status.detail
    assert backend.backend_identity("NATIVE") == "NATIVE:unavailable"


@pytest.mark.parametrize(
    "refusals, text",
    [
        pytest.param([Stale, Unsupported], "tuple of exception classes", id="a-list"),
        pytest.param(("NativePortStale",), "tuple of exception classes", id="names-not-classes"),
        pytest.param((Stale, UnsupportedPython, Unsupported), "does not list NativeDivisionDiverged", id="division-not-listed"),
    ],
)
def test_a_malformed_refusal_list_is_unavailable_by_name(monkeypatch, refusals, text):
    module, _calls = install_native(monkeypatch)
    module.NATIVE_REFUSALS = refusals
    backend.refresh_native()
    status = backend.native_status()
    assert status.coverage == status.clip == "unavailable" and text in status.detail


def test_the_status_of_a_present_native_module_is_the_status_of_its_shim(monkeypatch):
    shim = {"coverage": "available", "clip": "stale(materialize/clip.py)"}
    install_native(monkeypatch, status=shim, version="0.1.0")
    status = backend.native_status()
    assert (status.coverage, status.clip, status.version, status.build_id) == ("available", "stale(materialize/clip.py)", "0.1.0", BUILD_ID)
    assert not status.available
    assert status.as_record()["build_id"] == BUILD_ID
    assert backend.backend_identity("NATIVE") == f"NATIVE:{BUILD_ID}"
    assert backend.backend_identity("PYTHON") == "PYTHON"


def test_the_identity_of_the_backend_is_the_build_and_not_the_wheel_number(monkeypatch):
    install_native(monkeypatch, version="0.1.0", build_id="11" * 32)
    first = backend.backend_identity("NATIVE")
    install_native(monkeypatch, version="0.1.0", build_id="22" * 32)  # тот же номер колеса, другая сборка
    assert backend.backend_identity("NATIVE") != first == "NATIVE:" + "11" * 32
    install_native(monkeypatch, version="0.2.0", build_id="22" * 32)  # другой номер, та же сборка
    assert backend.backend_identity("NATIVE") == "NATIVE:" + "22" * 32


def test_a_broken_status_call_is_named_and_not_raised(monkeypatch):
    module, _calls = install_native(monkeypatch)

    def broken():
        raise OSError("device gone")

    module.native_status = broken
    status = backend.native_status()
    assert status.coverage == status.clip == "unavailable"
    assert "OSError" in status.detail


def test_a_broken_build_id_call_is_a_named_status_and_an_unavailable_identity(monkeypatch):
    module, _calls = install_native(monkeypatch)

    def broken():
        raise OSError("sources gone")

    module.native_build_id = broken
    status = backend.native_status()
    assert status.coverage == status.clip == "unavailable" and "OSError" in status.detail
    assert backend.backend_identity("NATIVE") == "NATIVE:unavailable"


# --------------------------------------------------------------------------
# 2. PYTHON == эталон
# --------------------------------------------------------------------------


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("name", FIXTURES)
def test_python_dispatch_is_byte_identical_to_the_oracle(monkeypatch, name, law):
    lift_law = LAWS[law]
    _make, first, _saturated = FIXTURES[name]
    reference = baseline(monkeypatch, name, first, lift_law)
    with memo_disabled():
        outside = answer(build(name, first, lift_law)[0])
        with backend.use_backend("PYTHON") as ledger:
            inside = answer(build(name, first, lift_law)[0])
    assert ledger is None
    assert backend.active_backend() is backend.KernelBackendV1.PYTHON
    assert outside == reference
    assert inside == reference


def test_python_dispatch_of_the_coverage_is_the_oracle_value_and_price(monkeypatch):
    """Прямой вызов: тот же `CoverageV1` и те же шесть статей бюджета, что у эталона на равных входах."""

    _result, prepared = build("fold", "3.5", LAWS["triangles"])
    oracle, _clip = oracles()
    partition = prepared.regions[0].partition
    alpha = Fraction(7, 4)
    own, direct = exact_work_budget(stage="COVERAGE"), exact_work_budget(stage="COVERAGE")
    through = backend.coverage_compute(partition, alpha, own, {})
    reference = oracle(partition, alpha, direct, {})
    assert through == reference
    assert own.spent_by_article() == direct.spent_by_article()


def test_no_name_in_the_kernel_is_replaced_at_run_time():
    """Диспетчеры стоят у вызывающих: ни подмены имён, ни установщика, ни замка на неё в `backend.py` нет."""

    for name in ("install_dispatch", "uninstall_dispatch", "dispatch_installed", "_install_locked", "_INSTALL_LOCK", "_INSTALLED", "_ORACLES"):
        assert not hasattr(backend, name), name
    before = oracles()
    backend.refresh_native()
    with backend.use_backend("NATIVE"):
        pass
    assert oracles() == before
    assert coverage._coverage_at is not backend.coverage_compute and clip.clip_geometry is not backend.clip_compute
    text = (KERNEL / "backend.py").read_text(encoding="utf-8")
    assert "setattr(" not in text


# --------------------------------------------------------------------------
# 3. NATIVE, который считает
# --------------------------------------------------------------------------


@pytest.mark.parametrize("law", LAWS)
def test_a_native_equal_to_the_oracle_gives_the_same_answer_and_price_and_is_named_the_runner(monkeypatch, law):
    lift_law = LAWS[law]
    _make, first, _saturated = FIXTURES["fold"]
    reference = baseline(monkeypatch, "fold", first, lift_law)
    oracle_coverage, oracle_clip = oracles()
    _module, calls = install_native(monkeypatch, coverage_at=oracle_coverage, clip_geometry=oracle_clip)
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        native = answer(build("fold", first, lift_law)[0])
    record = ledger.record()
    assert native == reference
    assert record.requested == "NATIVE" and record.ran == "native"
    assert record.python_calls == 0 and record.fallbacks == () and record.outcomes == ()
    assert record.native_calls == len(calls) and set(calls) == {"coverage", "clip"}
    assert record.as_record()["ran"] == "native"


# --------------------------------------------------------------------------
# 4. Откат именован
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "refusal, outcome",
    [
        (Stale, "NATIVE_PORT_STALE"),
        (UnsupportedPython, "NATIVE_UNSUPPORTED_PYTHON"),
        (Unsupported, "NATIVE_PORT_UNSUPPORTED"),
    ],
)
def test_a_named_native_refusal_is_computed_by_the_oracle_and_recorded(monkeypatch, refusal, outcome):
    _make, first, _saturated = FIXTURES["fold"]
    reference = baseline(monkeypatch, "fold", first, LAWS["faces"])

    def refuse(*_args, **_kwargs):
        raise refusal("the oracle moved: materialize/clip.py")

    install_native(monkeypatch, coverage_at=refuse, clip_geometry=refuse)
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        fallen = answer(build("fold", first, LAWS["faces"])[0])
    record = ledger.record()
    assert fallen == reference
    assert record.ran == "python" and record.native_calls == 0 and record.python_calls > 0
    assert record.outcomes == (outcome,)
    assert {item[1] for item in record.fallbacks} == {"clip", "coverage"}
    assert all(item[0] == outcome and item[2] >= 1 and "the oracle moved" in item[3] for item in record.fallbacks)
    assert ledger.refusal is None


@pytest.mark.parametrize(
    "defect",
    [
        pytest.param(TypeError("cftuv_native: a Python object the extension does not carry: dict"), id="input-shape-type-error"),
        pytest.param(RuntimeError("Session.clip_geometry: native panic: index out of bounds"), id="rust-panic"),
        pytest.param(RuntimeError("Session.coverage_at: native panic: forced by the test knob"), id="rust-panic-coverage"),
    ],
)
def test_a_native_defect_before_any_effect_falls_back_and_is_named_internal_error(monkeypatch, defect):
    """`TypeError("cftuv_native: ...")` и пойманная паника Rust не из `NATIVE_REFUSALS`, но подняты до эффектов: домен считает эталон, дефект виден."""

    _make, first, _saturated = FIXTURES["fold"]
    reference = baseline(monkeypatch, "fold", first, LAWS["faces"])

    def break_down(*_args, **_kwargs):
        raise defect

    install_native(monkeypatch, coverage_at=break_down, clip_geometry=break_down)
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        fallen = answer(build("fold", first, LAWS["faces"])[0])
    record = ledger.record()
    assert fallen == reference
    assert record.ran == "python" and record.native_calls == 0 and record.python_calls > 0
    assert record.outcomes == ("NATIVE_INTERNAL_ERROR",)
    assert ledger.refusal is None
    assert all(item[0] == "NATIVE_INTERNAL_ERROR" and type(defect).__name__ in item[3] and str(defect)[:20] in item[3] for item in record.fallbacks)


def test_a_member_of_the_refusal_list_the_host_does_not_know_by_name_is_a_visible_internal_error(monkeypatch):
    plane, budget, inputs = _stub_clip_inputs()
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: "ORACLE"))

    def future(*_args, **_kwargs):
        raise FutureRefusal("a refusal added after this host was written")

    install_native(monkeypatch, clip_geometry=future, extra_refusals=(FutureRefusal,))
    with backend.use_backend("NATIVE") as ledger:
        assert backend.clip_compute(plane, budget, **inputs) == "ORACLE"
    record = ledger.record()
    assert record.outcomes == ("NATIVE_INTERNAL_ERROR",)
    assert "FutureRefusal" in record.fallbacks[0][3] and ledger.refusal is None


def test_a_division_that_diverged_refuses_the_domain_by_name_and_the_oracle_never_runs(monkeypatch):
    """`NativeDivisionDiverged`: деление эталона на этом входе не завершилось бы, поэтому ни отката, ни эталона."""

    plane, budget, inputs = _stub_clip_inputs()
    ran = []
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: ran.append("oracle")))

    def diverge(*_args, **_kwargs):
        raise Diverged("the generic division fallback did not finish")

    install_native(monkeypatch, clip_geometry=diverge)
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused, match="would not either") as raised:
            backend.clip_compute(plane, budget, **inputs)
    assert ran == []
    assert raised.value.outcome is backend.BackendOutcomeV1.NATIVE_DIVISION_DIVERGED
    assert isinstance(raised.value.__cause__, Diverged)
    record = ledger.record()
    assert record.outcomes == ("NATIVE_DIVISION_DIVERGED",)
    assert record.fallbacks[0][:3] == ("NATIVE_DIVISION_DIVERGED", "clip", 1)
    assert (record.native_calls, record.python_calls) == (0, 0)
    assert ledger.refusal[0] == "NATIVE_DIVISION_DIVERGED" and ledger.refusal[1].startswith("clip:")


def test_a_diverged_coverage_refuses_the_domain_by_name_too(monkeypatch):
    _result, prepared = build("fold", "3.5", LAWS["triangles"])
    partition = prepared.regions[0].partition
    ran = []
    monkeypatch.setattr(backend, "_python_coverage", lambda: (lambda *a, **k: ran.append("oracle")))

    def diverge(partition_, alpha, work_budget=None, store=None):
        raise Diverged("the generic division fallback did not finish")

    install_native(monkeypatch, coverage_at=diverge)
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused):
            backend.coverage_compute(partition, Fraction(7, 4), exact_work_budget(stage="COVERAGE"), {})
    assert ran == [] and ledger.record().outcomes == ("NATIVE_DIVISION_DIVERGED",)


def test_an_absent_native_module_falls_back_per_domain_and_is_recorded(monkeypatch):
    _make, first, _saturated = FIXTURES["fold"]
    reference = baseline(monkeypatch, "fold", first, LAWS["triangles"])
    no_native(monkeypatch)
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        fallen = answer(build("fold", first, LAWS["triangles"])[0])
    record = ledger.record()
    assert fallen == reference
    assert record.ran == "python" and record.outcomes == ("NATIVE_UNAVAILABLE",)
    assert "cftuv_native" in record.fallbacks[0][3]


def test_a_mixed_domain_is_named_mixed(monkeypatch):
    oracle_coverage, oracle_clip = oracles()

    def refuse(*_args, **_kwargs):
        raise Unsupported("a sort of 64 nodes or more")

    install_native(monkeypatch, coverage_at=oracle_coverage, clip_geometry=refuse)
    _make, first, _saturated = FIXTURES["fold"]
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        build("fold", first, LAWS["triangles"])
    record = ledger.record()
    assert record.ran == "mixed"
    assert record.native_calls > 0 and record.python_calls == 1
    assert record.outcomes == ("NATIVE_PORT_UNSUPPORTED",)
    assert [item[1] for item in record.fallbacks] == ["clip"]


def test_a_traced_coverage_of_a_wheel_without_traces_is_served_by_python_and_named(monkeypatch):
    """Старое колесо: у `coverage_at` нет параметра `traces` — запись шаблона считает эталон, и это названо."""

    _result, prepared = build("fold", "3.5", LAWS["triangles"])
    oracle, _clip = oracles()
    _module, calls = install_native(monkeypatch, coverage_at=oracle, clip_geometry=None)
    partition, alpha = prepared.regions[0].partition, Fraction(7, 4)
    own, direct = [], []
    with backend.use_backend("NATIVE") as ledger:
        through = backend.coverage_compute(partition, alpha, exact_work_budget(stage="COVERAGE"), {}, own)
    reference = oracle(partition, alpha, exact_work_budget(stage="COVERAGE"), {}, direct)
    assert through == reference and own == direct and own
    assert calls == []
    assert ledger.record().outcomes == ("NATIVE_TRACES_UNSUPPORTED",)


def test_a_traced_coverage_of_a_wheel_with_traces_is_computed_natively_and_fills_the_traces(monkeypatch):
    _result, prepared = build("fold", "3.5", LAWS["triangles"])
    oracle, _clip = oracles()
    seen: list = []

    def with_traces(partition, alpha, work_budget=None, store=None, traces=None):
        seen.append(traces)
        return oracle(partition, alpha, work_budget, store, traces)

    _module, calls = install_native(monkeypatch, coverage_at=with_traces, clip_geometry=None)
    # подставной шим берёт `*args`: подпись `covered` не называет traces, поэтому подменяем сам вход на функцию с подписью
    monkeypatch.setattr(sys.modules["cftuv_native"], "coverage_at", with_traces)
    backend.refresh_native()
    partition, alpha = prepared.regions[0].partition, Fraction(7, 4)
    own, direct = [], []
    with backend.use_backend("NATIVE") as ledger:
        through = backend.coverage_compute(partition, alpha, exact_work_budget(stage="COVERAGE"), {}, own)
        untraced = backend.coverage_compute(partition, alpha, exact_work_budget(stage="COVERAGE"), {})
    reference = oracle(partition, alpha, exact_work_budget(stage="COVERAGE"), {}, direct)
    assert through == reference == untraced and own == direct and own
    assert seen == [own, None]
    record = ledger.record()
    assert (record.native_calls, record.python_calls, record.outcomes) == (2, 0, ())


def test_a_domain_that_never_called_a_native_operation_is_named_not_reached(monkeypatch):
    install_native(monkeypatch)
    with backend.use_backend("NATIVE") as ledger:
        pass
    record = ledger.record()
    assert (record.native_calls, record.python_calls, record.fallbacks) == (0, 0, ())
    assert record.ran == "python" and record.outcomes == ("NATIVE_NOT_REACHED",)
    # заказ Python такого исхода не имеет: он ничего не заказывал
    assert backend.BackendRecordV1("PYTHON", 0, 0).outcomes == ()


def test_the_scope_is_not_inherited_by_a_thread(monkeypatch):
    install_native(monkeypatch)
    seen: list = []
    with backend.use_backend("NATIVE"):
        assert backend.active_backend() is backend.KernelBackendV1.NATIVE
        worker = threading.Thread(target=lambda: seen.append(backend.active_backend()))
        worker.start()
        worker.join()
    assert seen == [backend.KernelBackendV1.PYTHON]
    assert backend.active_backend() is backend.KernelBackendV1.PYTHON


# --------------------------------------------------------------------------
# 5. Исход эталона и чужое исключение не глотаются
# --------------------------------------------------------------------------


def _oracle_outcomes():
    return [
        pytest.param(lambda: exact.ExactCanonicalizationWorkBudgetExhausted("work budget exhausted"), id="work-budget-exhausted"),
        pytest.param(lambda: MaterializationRefusal(MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE, "the ring did not close"), id="materialization-refusal"),
        pytest.param(lambda: OverflowError("int too large to convert to float"), id="overflow"),
        pytest.param(lambda: ValueError("a native panic is not a refusal"), id="value-error"),
        pytest.param(lambda: KeyError("point"), id="key-error"),
        pytest.param(lambda: ZeroDivisionError("division by zero"), id="zero-division"),
        pytest.param(lambda: exact.NegativeRadicandError("under the root -1"), id="negative-radicand"),
        pytest.param(lambda: ArithmeticError("factorization did not reconstruct"), id="reconstruction"),
    ]


@pytest.mark.parametrize("make", _oracle_outcomes())
def test_an_oracle_outcome_reaches_the_caller_as_is_with_its_partial_effects(monkeypatch, make):
    """Исход эталона нативный вызов применяет с частичными эффектами эталона: откат, пояс и отказ домена к нему не относятся."""

    plane, budget, inputs = _stub_clip_inputs()
    ran = []
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: ran.append("oracle")))
    thrown = make()

    def oracle_like(plane_, budget_, **_inputs):
        budget_.gcd_operations += 7  # частичный эффект, который оставило бы исключение эталона
        plane_._normal_by_position["b"] = (1.0, 0.0, 0.0)
        raise thrown

    install_native(monkeypatch, clip_geometry=oracle_like)
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(type(thrown)) as raised:
            backend.clip_compute(plane, budget, **inputs)
    assert raised.value is thrown
    assert ran == []
    assert budget.gcd_operations == 7 and "b" in plane._normal_by_position  # эффекты остались: это ответ эталона, а не грязь
    assert ledger.refusal is None and ledger.record().fallbacks == ()
    assert (ledger.record().native_calls, ledger.record().python_calls) == (0, 0)


@pytest.mark.parametrize(
    "stranger",
    [
        pytest.param(lambda: TypeError("not from the shim"), id="plain-type-error"),
        pytest.param(lambda: RuntimeError("a plain runtime error"), id="plain-runtime-error"),
        pytest.param(lambda: type("NativeMirrorError", (RuntimeError,), {})("native panic of a subclass is not the caught panic"), id="runtime-error-subclass"),
        pytest.param(lambda: RuntimeError("cftuv_native: the native touch names a factorization the real table does not hold"), id="memory-log-failure"),
        pytest.param(lambda: OSError("device gone"), id="os-error"),
    ],
)
def test_an_exception_the_host_does_not_know_reaches_the_caller_as_is(monkeypatch, stranger):
    plane, budget, inputs = _stub_clip_inputs()
    ran = []
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: ran.append("oracle")))
    thrown = stranger()

    def boom(*_args, **_kwargs):
        raise thrown

    install_native(monkeypatch, clip_geometry=boom)
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(type(thrown)) as raised:
            backend.clip_compute(plane, budget, **inputs)
    assert raised.value is thrown and ran == []
    assert ledger.record().python_calls == 0 and ledger.record().fallbacks == () and ledger.refusal is None


def test_a_foreign_exception_stops_a_whole_domain_and_is_not_hidden_by_a_fallback(monkeypatch):
    def boom(*_args, **_kwargs):
        raise ValueError("a native panic is not a refusal")

    install_native(monkeypatch, coverage_at=boom, clip_geometry=boom)
    _make, first, _saturated = FIXTURES["fold"]
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        with pytest.raises(ValueError, match="a native panic is not a refusal"):
            build("fold", first, LAWS["triangles"])
    assert ledger.record().python_calls == 0


# --------------------------------------------------------------------------
# 6. Точки подключения
# --------------------------------------------------------------------------


def _text(*parts):
    return (KERNEL.joinpath(*parts)).read_text(encoding="utf-8").replace("\r\n", "\n")


def test_the_dispatch_hooks_stand_where_the_report_says():
    """Покрытие региона и запись шаблона идут через диспетчер в незакреплённых файлах; резка — в `cut_domain`; закреплённый `coverage.py` зовёт эталон."""

    conveyor = _text("wavefront", "conveyor.py")
    assert conveyor.count("covered = backend.covered_at(region.partition, lattice_alpha, work_budget, store)\n") == 1
    assert "coverage_at(region.partition" not in conveyor.replace("backend.covered_at(region.partition", "")
    stepper = _text("materialize", "step.py")
    assert stepper.count("result = backend.coverage_compute(partition, alpha, work_budget, store, traces)\n") == 1
    assert "_coverage_at(" not in stepper.replace("backend.coverage_compute(", "")
    # закреплённый файл покрытия: эталон зовёт эталон
    oracle_coverage = _text("wavefront", "coverage.py")
    assert oracle_coverage.count("    return _coverage_at(partition, alpha, work_budget, store)\n") == 1 and "backend" not in oracle_coverage
    # резка: ровно одно место, и оно зовёт диспетчер (правка закреплённого `clip.py` идёт вместе с перевыпуском закрепления: тест архитектуры)
    oracle_clip = _text("materialize", "clip.py")
    wired = len(re.findall(r"    clipped, memo = run_clip\(\n        backend\.clip_compute,\n        plane,\n", oracle_clip))
    plain = len(re.findall(r"    clipped, memo = run_clip\(\n        clip_geometry,\n        plane,\n", oracle_clip))
    assert (wired, plain) == (1, 0)
    assert oracle_clip.count("from .. import backend\n") == 1


def test_covered_at_mirrors_the_coverage_at_wrapper_by_behaviour():
    """`covered_at` = `coverage_at` с диспетчером вместо `_coverage_at`: без источника, с источником и с источником, не знающим разбиения."""

    _result, prepared = build("fold", "3.5", LAWS["triangles"])
    partition, alpha = prepared.regions[0].partition, Fraction(7, 4)
    produced = coverage.coverage_at(partition, alpha, exact_work_budget(stage="COVERAGE"), {})

    class Source:
        def __init__(self, answer):
            self.answer, self.asked = answer, []

        def coverage(self, partition, alpha, work_budget, store):
            self.asked.append((partition, alpha))
            return self.answer

    for answer in (None, produced):
        wrapper_source, mirror_source = Source(answer), Source(answer)
        own, direct = exact_work_budget(stage="COVERAGE"), exact_work_budget(stage="COVERAGE")
        with coverage.coverage_source(wrapper_source):
            expected = coverage.coverage_at(partition, Fraction(7, 4), direct, {})
        with coverage.coverage_source(mirror_source):
            got = backend.covered_at(partition, Fraction(7, 4), own, {})
        assert got == expected and own.spent_by_article() == direct.spent_by_article()
        assert wrapper_source.asked == mirror_source.asked == [(partition, Fraction(7, 4))]
    plain_own, plain_direct = exact_work_budget(stage="COVERAGE"), exact_work_budget(stage="COVERAGE")
    assert backend.covered_at(partition, Fraction(7, 4), plain_own, {}) == coverage.coverage_at(partition, Fraction(7, 4), plain_direct, {})
    assert plain_own.spent_by_article() == plain_direct.spent_by_article()
    assert backend.covered_at(partition, 2, None, None) == coverage.coverage_at(partition, 2, None, None)  # целая alpha приводится к дроби так же


def test_covered_at_mirrors_the_coverage_at_wrapper_by_text():
    """Обёртка эталона живёт в закреплённом файле и повторена в `backend.py`: правка обёртки красит тест, пока копию не догонят."""

    import ast
    import inspect

    def statements(function, fixed):
        tree = ast.parse(inspect.getsource(function).replace("\r\n", "\n"))
        body = tree.body[0].body
        if isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant):
            body = body[1:]  # докстрока
        return [ast.unparse(item).replace(fixed[0], fixed[1]) for item in body]

    oracle = statements(coverage.coverage_at, ("_SOURCE.get()", "current_coverage_source()"))
    mirror = statements(backend.covered_at, ("", ""))
    mirror = [line for line in mirror if "import current_coverage_source" not in line]
    oracle = [line.replace("_coverage_at(", "coverage_compute(") for line in oracle]
    assert oracle == mirror, (oracle, mirror)


# --------------------------------------------------------------------------
# 7. Первый заказ NATIVE из двух потоков
# --------------------------------------------------------------------------


def test_two_threads_placing_the_first_native_order_both_compute_natively_and_load_the_wheel_once(monkeypatch):
    """Главный поток и поток живой ширины делают первый заказ `NATIVE` одновременно: загрузка нативного ядра идёт под замком (один импорт на процесс), у каждого
    потока свой журнал, подмены имён нет, рекурсии нет."""

    plane, budget, inputs = _stub_clip_inputs()
    install_native(monkeypatch, clip_geometry=lambda *_a, **_k: "NATIVE")
    backend.refresh_native()  # вердикт загрузки забыт: первый вызов в потоке — первая загрузка
    imports: list = []
    real = backend._import_native

    def slow_import():
        imports.append(threading.current_thread().name)
        time.sleep(0.2)  # пока первый поток грузит, второй обязан ждать замок, а не грузить сам
        return real()

    monkeypatch.setattr(backend, "_import_native", slow_import)
    gate = threading.Barrier(2, timeout=10)
    results: dict = {}
    errors: list = []

    def order(name):
        try:
            gate.wait()
            with backend.use_backend("NATIVE") as ledger:
                results[name] = (backend.clip_compute(plane, budget, **inputs), ledger.record())
        except BaseException as exc:  # noqa: BLE001 - любая ошибка потока — ошибка теста
            errors.append(exc)

    threads = [threading.Thread(target=order, args=(name,), name=name) for name in ("main", "live")]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(30)

    assert errors == []
    assert len(imports) == 1
    assert {name: value[0] for name, value in results.items()} == {"main": "NATIVE", "live": "NATIVE"}
    assert all(record.native_calls == 1 and record.python_calls == 0 and record.fallbacks == () for _answer, record in results.values())
    assert oracles() == (coverage._coverage_at, clip.clip_geometry) and clip.clip_geometry is not backend.clip_compute


# --------------------------------------------------------------------------
# 8. Страховочный пояс: отказ после сдвига состояния — не откат
# --------------------------------------------------------------------------


def _stub_clip_inputs():
    plane = types.SimpleNamespace(_normal_by_position={"a": (0.0, 0.0, 1.0)})
    inputs = dict(points={}, cycles=[], polygons=[], law=None, seam=[], fans=[], flows=[], by_faces=False)
    return plane, exact_work_budget(stage="CLIP"), inputs


_DIRTY = [
    pytest.param(lambda plane, budget, traces, store: setattr(budget, "gcd_operations", budget.gcd_operations + 5), id="budget"),
    pytest.param(lambda plane, budget, traces, store: plane._normal_by_position.update(b=(1.0, 0.0, 0.0)), id="plane-normals"),
    pytest.param(lambda plane, budget, traces, store: plane._normal_by_position.update(a=(0.0, 1.0, 0.0)), id="plane-normal-overwritten"),
    pytest.param(lambda plane, budget, traces, store: exact._FACTORIZATION_MEMO.update({999_983: ((999_983, 1),)}), id="memory-table"),
    pytest.param(lambda plane, budget, traces, store: exact.SIGN_COUNTS.update(total=exact.SIGN_COUNTS["total"] + 1), id="sign-counts"),
]
_FAILURES = [
    pytest.param(lambda: Unsupported("a sort of 64 nodes or more"), id="port-unsupported"),
    pytest.param(lambda: Stale("moved"), id="port-stale"),
    pytest.param(lambda: TypeError("cftuv_native: a Python object the extension does not carry"), id="shim-type-error"),
    pytest.param(lambda: RuntimeError("Session.clip_geometry: native panic: boom"), id="rust-panic"),
]


@pytest.mark.parametrize("failure", _FAILURES)
@pytest.mark.parametrize("mutate", _DIRTY)
def test_a_native_failure_after_effects_is_a_named_refusal_and_the_oracle_never_runs_on_dirty_state(monkeypatch, mutate, failure):
    """Контракт порта: отказ состояния не двигает. Если пояс всё же видит сдвиг, откат запрещён: домен отказан `NATIVE_PARTIAL_EFFECTS_REFUSED`."""

    snapshot = dict(exact.SIGN_COUNTS)
    monkeypatch.setattr(exact, "SIGN_COUNTS", dict(snapshot))
    monkeypatch.setattr(exact, "_FACTORIZATION_MEMO", dict(exact._FACTORIZATION_MEMO))
    plane, budget, inputs = _stub_clip_inputs()
    ran = []
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: ran.append("oracle")))

    def late(plane_, budget_, **_inputs):
        mutate(plane_, budget_, None, None)
        raise failure()

    install_native(monkeypatch, clip_geometry=late)
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused, match="partial effects") as raised:
            backend.clip_compute(plane, budget, **inputs)
    assert ran == []
    assert raised.value.outcome is backend.BackendOutcomeV1.NATIVE_PARTIAL_EFFECTS_REFUSED
    record = ledger.record()
    assert record.outcomes == ("NATIVE_PARTIAL_EFFECTS_REFUSED",)
    assert record.fallbacks[0][:3] == ("NATIVE_PARTIAL_EFFECTS_REFUSED", "clip", 1)
    assert (record.native_calls, record.python_calls) == (0, 0)
    assert ledger.refusal[0] == "NATIVE_PARTIAL_EFFECTS_REFUSED" and ledger.refusal[1].startswith("clip:")


@pytest.mark.parametrize(
    "failure, outcome",
    [
        (lambda: Unsupported("declined before any effect"), "NATIVE_PORT_UNSUPPORTED"),
        (lambda: Stale("moved"), "NATIVE_PORT_STALE"),
        (lambda: UnsupportedPython("3.10"), "NATIVE_UNSUPPORTED_PYTHON"),
        (lambda: FutureRefusal("not known by name"), "NATIVE_INTERNAL_ERROR"),
        (lambda: TypeError("cftuv_native: a Python object the extension does not carry"), "NATIVE_INTERNAL_ERROR"),
        (lambda: RuntimeError("Session.clip_geometry: native panic: boom"), "NATIVE_INTERNAL_ERROR"),
    ],
)
def test_a_clean_native_refusal_passes_the_belt_and_falls_back_to_the_oracle(monkeypatch, failure, outcome):
    """Пояс не срабатывает на чистом отказе: нативный вызов успел только прочитать (плоскость, бюджет, таблицы памяти) — откат на эталон."""

    plane, budget, inputs = _stub_clip_inputs()
    spent = tuple(budget.spent_by_article())
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: "ORACLE"))

    def clean(plane_, budget_, **_inputs):
        assert plane_._normal_by_position == {"a": (0.0, 0.0, 1.0)}  # читает, ничего не пишет
        raise failure()

    install_native(monkeypatch, clip_geometry=clean, extra_refusals=(FutureRefusal,))
    with backend.use_backend("NATIVE") as ledger:
        assert backend.clip_compute(plane, budget, **inputs) == "ORACLE"
    assert ledger.record().outcomes == (outcome,) and ledger.refusal is None
    assert tuple(budget.spent_by_article()) == spent


def test_a_division_refusal_is_the_domain_refusal_even_when_the_state_is_clean_and_when_it_is_not(monkeypatch):
    plane, budget, inputs = _stub_clip_inputs()
    install_native(monkeypatch, clip_geometry=lambda *_a, **_k: (_ for _ in ()).throw(Diverged("did not finish")))
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused) as clean:
            backend.clip_compute(plane, budget, **inputs)
    assert clean.value.outcome is backend.BackendOutcomeV1.NATIVE_DIVISION_DIVERGED and ledger.record().outcomes == ("NATIVE_DIVISION_DIVERGED",)

    def dirty(plane_, budget_, **_inputs):
        budget_.gcd_operations += 3
        raise Diverged("did not finish")

    install_native(monkeypatch, clip_geometry=dirty)
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused) as unclean:
            backend.clip_compute(plane, budget, **inputs)
    assert unclean.value.outcome is backend.BackendOutcomeV1.NATIVE_DIVISION_DIVERGED  # имя точнее: эталон на этом входе не завершился бы в любом случае


def test_a_coverage_refusal_after_the_traces_were_filled_is_a_partial_effects_refusal(monkeypatch):
    _result, prepared = build("fold", "3.5", LAWS["triangles"])
    partition = prepared.regions[0].partition
    ran = []
    monkeypatch.setattr(backend, "_python_coverage", lambda: (lambda *a, **k: ran.append("oracle")))

    def late(partition_, alpha, work_budget=None, store=None, traces=None):
        traces.append("half a trace")
        raise Unsupported("late")

    install_native(monkeypatch, coverage_at=late)
    monkeypatch.setattr(sys.modules["cftuv_native"], "coverage_at", late)
    backend.refresh_native()
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused):
            backend.coverage_compute(partition, Fraction(7, 4), exact_work_budget(stage="COVERAGE"), {}, [])
    assert ran == [] and ledger.record().outcomes == ("NATIVE_PARTIAL_EFFECTS_REFUSED",)
