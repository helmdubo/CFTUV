"""Выбор бэкенда ядра (`cftuv_envelope.backend`): Python-эталон по умолчанию, нативное ядро — только именованно.

Утверждений шесть, и каждое стоит на проверке:

1. СТАТУС. Без `cftuv_native` статус — именованный `unavailable` с причиной, а не исключение; с ним — статус шима как есть.
2. PYTHON == ЭТАЛОН. Точки диспетчеризации под `PYTHON` (и вне блока) побитово равны вызову эталона: ответ домена, дайджесты,
   нормали и ЦЕНА (`EXACT_WORK_*`). Подключение точек в модули ядра здесь ИМИТИРУЕТСЯ подменой имени, которое зовёт вызывающий:
   замороженные файлы не тронуты, а проверяется ровно тот дифф, что записан в отчёте.
3. NATIVE, КОТОРЫЙ СЧИТАЕТ. Нативное ядро, равное эталону, даёт тот же ответ и ту же цену, а журнал домена называет его
   исполнителем.
4. ОТКАТ ИМЕНОВАН. `NativePortStale`, `NativeUnsupportedPython`, `NativePortUnsupported`, отсутствие колеса и вызов с `traces` —
   домен считает эталон, ответ тот же, а журнал несёт имя причины и счёт. Тихого отката нет.
5. ЧУЖОЕ ИСКЛЮЧЕНИЕ НЕ ГЛОТАЕТСЯ. Всё, что не из перечня выше, уходит вызывающему как есть.
6. ТОЧКИ ПОДКЛЮЧЕНИЯ там, где о них говорит отчёт: вызов эталона либо вызов диспетчера, ничего третьего.
"""

from __future__ import annotations

import re
import sys
import threading
import types
from fractions import Fraction
from pathlib import Path

import pytest

import developable_factories as df
from developable_route import materialize_developable

from cftuv_envelope import backend
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import exact_work_budget
from cftuv_envelope.materialize import clip, step
from cftuv_envelope.materialize.clip_memo import memo_disabled
from cftuv_envelope.wavefront import coverage

POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
ROUTE = ("r0a", "r0b")
LAWS = {
    "triangles": NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1,
    "faces": NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1,
}
KERNEL = Path(backend.__file__).resolve().parent


@pytest.fixture(autouse=True)
def _backend_state_is_not_shared():
    """Вердикт загрузки нативного ядра и подмена имён диспетчерами живут в процессе: тест не оставляет их следующему."""

    backend.uninstall_dispatch()
    backend.refresh_native()
    yield
    backend.uninstall_dispatch()
    backend.refresh_native()


class Stale(RuntimeError):
    pass


class Unsupported(RuntimeError):
    pass


class UnsupportedPython(RuntimeError):
    pass


def install_native(monkeypatch, *, coverage_at=None, clip_geometry=None, version="9.9.9", status=None):
    """Подставной `cftuv_native` в `sys.modules`; возвращает модуль и список вызовов `[(операция, ...)]`."""

    calls: list = []
    module = types.ModuleType("cftuv_native")
    module.NativePortStale = Stale
    module.NativePortUnsupported = Unsupported
    module.NativeUnsupportedPython = UnsupportedPython
    module.native_version = lambda: version
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
    """Эталоны `(покрытие, резка)` — их зовёт тест и ими отвечает подставное нативное ядро."""

    return coverage._coverage_at, clip.clip_geometry


def wire(monkeypatch=None):
    """Подключение через `install_dispatch`: на месте вызова эталона стоит диспетчер, файлы ядра не тронуты.

    Возвращает эталоны, снятые ДО подмены (`monkeypatch` не нужен: подмену снимает фикстура теста).
    """

    found = oracles()
    backend.install_dispatch()
    return found


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


def baseline(name, alpha, lift_law):
    """Ответ без диспетчера вообще (память резки выключена: резка считается, а не берётся)."""

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
    assert (status.coverage, status.clip, status.version) == (backend.UNAVAILABLE, backend.UNAVAILABLE, "")
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
    assert "NativePortStale" in status.detail


def test_the_status_of_a_present_native_module_is_the_status_of_its_shim(monkeypatch):
    shim = {"coverage": "available", "clip": "stale(materialize/clip.py)"}
    install_native(monkeypatch, status=shim, version="0.1.0")
    status = backend.native_status()
    assert (status.coverage, status.clip, status.version) == ("available", "stale(materialize/clip.py)", "0.1.0")
    assert not status.available
    assert backend.backend_identity("NATIVE") == "NATIVE:0.1.0"
    assert backend.backend_identity("PYTHON") == "PYTHON"


def test_a_broken_status_call_is_named_and_not_raised(monkeypatch):
    module, _calls = install_native(monkeypatch)

    def broken():
        raise OSError("device gone")

    module.native_status = broken
    status = backend.native_status()
    assert status.coverage == status.clip == "unavailable"
    assert "OSError" in status.detail


# --------------------------------------------------------------------------
# 2. PYTHON == эталон
# --------------------------------------------------------------------------


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("name", FIXTURES)
def test_python_dispatch_is_byte_identical_to_the_oracle(monkeypatch, name, law):
    lift_law = LAWS[law]
    _make, first, _saturated = FIXTURES[name]
    reference = baseline(name, first, lift_law)
    wire(monkeypatch)
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


# --------------------------------------------------------------------------
# 3. NATIVE, который считает
# --------------------------------------------------------------------------


@pytest.mark.parametrize("law", LAWS)
def test_a_native_equal_to_the_oracle_gives_the_same_answer_and_price_and_is_named_the_runner(monkeypatch, law):
    lift_law = LAWS[law]
    _make, first, _saturated = FIXTURES["fold"]
    reference = baseline("fold", first, lift_law)
    oracle_coverage, oracle_clip = wire(monkeypatch)
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
    reference = baseline("fold", first, LAWS["faces"])
    wire(monkeypatch)

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


def test_an_absent_native_module_falls_back_per_domain_and_is_recorded(monkeypatch):
    _make, first, _saturated = FIXTURES["fold"]
    reference = baseline("fold", first, LAWS["triangles"])
    wire(monkeypatch)
    no_native(monkeypatch)
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        fallen = answer(build("fold", first, LAWS["triangles"])[0])
    record = ledger.record()
    assert fallen == reference
    assert record.ran == "python" and record.outcomes == ("NATIVE_UNAVAILABLE",)
    assert "cftuv_native" in record.fallbacks[0][3]


def test_a_mixed_domain_is_named_mixed(monkeypatch):
    oracle_coverage, oracle_clip = wire(monkeypatch)

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


def test_a_traced_coverage_is_served_by_python_and_named(monkeypatch):
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
# 5. Чужое исключение не глотается
# --------------------------------------------------------------------------


def test_an_exception_outside_the_named_refusals_reaches_the_caller(monkeypatch):
    oracle_coverage, _oracle_clip = wire(monkeypatch)

    def boom(*_args, **_kwargs):
        raise ValueError("a native panic is not a refusal")

    install_native(monkeypatch, coverage_at=boom, clip_geometry=boom)
    _make, first, _saturated = FIXTURES["fold"]
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        with pytest.raises(ValueError, match="a native panic is not a refusal"):
            build("fold", first, LAWS["triangles"])
    assert ledger.record().python_calls == 0


def test_install_dispatch_replaces_exactly_the_three_names_and_uninstall_restores_them():
    before = (coverage._coverage_at, step._coverage_at, clip.clip_geometry)
    assert not backend.dispatch_installed()

    names = backend.install_dispatch()

    assert names == ("coverage._coverage_at", "step._coverage_at", "clip.clip_geometry")
    assert (coverage._coverage_at, step._coverage_at, clip.clip_geometry) == (
        backend.coverage_compute,
        backend.coverage_compute,
        backend.clip_compute,
    )
    assert backend.install_dispatch() == names and backend.dispatch_installed()  # повтор ничего не подменяет второй раз
    backend.uninstall_dispatch()
    assert (coverage._coverage_at, step._coverage_at, clip.clip_geometry) == before
    assert not backend.dispatch_installed()
    backend.uninstall_dispatch()  # без подмены - ничего не делает


def test_the_installed_dispatch_leaves_the_frozen_files_byte_for_byte_alone():
    """Подмена имён идёт при запуске: ни один файл ядра не правится (закрепления нативного порта по дайджесту остаются верными)."""

    import hashlib

    names = ("wavefront/coverage.py", "materialize/clip.py", "materialize/step.py")
    before = {name: hashlib.sha256((KERNEL / name).read_bytes()).hexdigest() for name in names}
    backend.install_dispatch()
    assert {name: hashlib.sha256((KERNEL / name).read_bytes()).hexdigest() for name in names} == before


# --------------------------------------------------------------------------
# 6. Точки подключения
# --------------------------------------------------------------------------


def _text(*parts):
    return (KERNEL.joinpath(*parts)).read_text(encoding="utf-8").replace("\r\n", "\n")


def test_the_hook_sites_are_where_the_report_names_them():
    """Три места вызова эталона: каждое — вызов эталона либо вызов диспетчера, ничего третьего; места не двоятся."""

    sites = (
        (
            _text("wavefront", "coverage.py"),
            r"    return (_coverage_at|backend\.coverage_compute)\(partition, alpha, work_budget, store\)\n",
        ),
        (
            _text("materialize", "step.py"),
            r"        result = (_coverage_at|backend\.coverage_compute)\(partition, alpha, work_budget, store, traces\)\n",
        ),
        (
            _text("materialize", "clip.py"),
            r"    clipped, memo = run_clip\(\n        (clip_geometry|backend\.clip_compute),\n        plane,\n",
        ),
    )
    for text, pattern in sites:
        assert len(re.findall(pattern, text)) == 1, pattern
