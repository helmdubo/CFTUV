"""Выбор бэкенда ядра (`cftuv_envelope.backend`): Python-эталон по умолчанию, нативное ядро — только именованно.

Утверждений шесть, и каждое стоит на проверке:

1. СТАТУС. Без `cftuv_native` статус — именованный `unavailable` с причиной, а не исключение; с ним — статус шима как есть.
2. PYTHON == ЭТАЛОН. Точки диспетчеризации под `PYTHON` (и вне блока) побитово равны вызову эталона: ответ домена, дайджесты,
   нормали и ЦЕНА (`EXACT_WORK_*`). Покрытие подключено в незакреплённых файлах (`conveyor.py`, `step.py`), резка — подменой имени
   `install_dispatch` (прямая правка `clip.py` сделала бы нативный порт резки `stale`): закреплённые файлы не тронуты.
3. NATIVE, КОТОРЫЙ СЧИТАЕТ. Нативное ядро, равное эталону, даёт тот же ответ и ту же цену, а журнал домена называет его
   исполнителем.
4. ОТКАТ ИМЕНОВАН. `NativePortStale`, `NativeUnsupportedPython`, `NativePortUnsupported`, отсутствие колеса и вызов с `traces` —
   домен считает эталон, ответ тот же, а журнал несёт имя причины и счёт. Тихого отката нет.
5. ЧУЖОЕ ИСКЛЮЧЕНИЕ НЕ ГЛОТАЕТСЯ. Всё, что не из перечня выше, уходит вызывающему как есть.
6. ТОЧКИ ПОДКЛЮЧЕНИЯ там, где о них говорит отчёт: покрытие подключено в `conveyor.py` и `step.py`, закреплённые файлы нативным портом
   (`coverage.py`, `clip.py`) зовут эталон, обёртка `covered_at` повторяет `coverage.coverage_at`.
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
    """Подключение резки через `install_dispatch` (покрытие подключено в самом ядре): файлы ядра не тронуты.

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


def test_install_dispatch_replaces_exactly_the_two_names_and_uninstall_restores_them():
    before = (coverage._coverage_at, clip.clip_geometry)
    assert not backend.dispatch_installed() and not hasattr(step, "_coverage_at")  # `step` зовёт диспетчер сам

    names = backend.install_dispatch()

    assert names == ("coverage._coverage_at", "clip.clip_geometry")
    assert (coverage._coverage_at, clip.clip_geometry) == (backend.coverage_compute, backend.clip_compute)
    assert backend.install_dispatch() == names and backend.dispatch_installed()  # повтор ничего не подменяет второй раз
    backend.uninstall_dispatch()
    assert (coverage._coverage_at, clip.clip_geometry) == before
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


def test_the_coverage_hooks_stand_in_the_unpinned_files_and_the_pinned_ones_still_call_the_oracle():
    """Покрытие региона и запись шаблона идут через диспетчер; закреплённые нативным портом файлы эталона не тронуты."""

    conveyor = _text("wavefront", "conveyor.py")
    assert conveyor.count("covered = backend.covered_at(region.partition, lattice_alpha, work_budget, store)\n") == 1
    assert "coverage_at(region.partition" not in conveyor.replace("backend.covered_at(region.partition", "")
    stepper = _text("materialize", "step.py")
    assert stepper.count("result = backend.coverage_compute(partition, alpha, work_budget, store, traces)\n") == 1
    assert "_coverage_at(" not in stepper.replace("backend.coverage_compute(", "")
    # закреплённые файлы: эталон зовёт эталон (подключение резки — подменой имени `install_dispatch` либо правкой с новым закреплением)
    oracle_coverage = _text("wavefront", "coverage.py")
    assert oracle_coverage.count("    return _coverage_at(partition, alpha, work_budget, store)\n") == 1 and "backend" not in oracle_coverage
    oracle_clip = _text("materialize", "clip.py")
    wired = len(re.findall(r"    clipped, memo = run_clip\(\n        backend\.clip_compute,\n        plane,\n", oracle_clip))
    plain = len(re.findall(r"    clipped, memo = run_clip\(\n        clip_geometry,\n        plane,\n", oracle_clip))
    assert wired + plain == 1  # ровно одно место резки; `wired` допустимо только вместе с новым закреплением (тест архитектуры)


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
# 7. Гонка первого заказа NATIVE: подмена имён идёт под замком
# --------------------------------------------------------------------------


class _Gate(list):
    """`_INSTALLED`, у которого первые две проверки «пусто ли» ждут друг друга, а поток «late» после этого отстаёт: оба потока проходят проверку
    ДО любой подмены, и «late» читает имена тогда, когда «early» уже всё подменил."""

    def __init__(self):
        super().__init__()
        self.barrier = threading.Barrier(2, timeout=5)
        self.checks = 0

    def __bool__(self):
        answer = len(self) > 0  # ответ проверки снят ДО ожидания: поток, отставший после неё, действует по устаревшему «пусто»
        self.checks += 1
        if self.checks <= 2:
            self.barrier.wait()
            if threading.current_thread().name == "late":
                time.sleep(0.3)
        return answer


def _race(monkeypatch, install):
    """Два потока одновременно делают первый заказ; `(эталоны, ошибки потоков)`."""

    monkeypatch.setattr(backend, "_INSTALLED", _Gate())
    monkeypatch.setattr(backend, "_ORACLES", {})
    errors: list = []

    def order():
        try:
            install()
        except BaseException as exc:  # noqa: BLE001 - любая ошибка потока — ошибка теста
            errors.append(exc)

    threads = [threading.Thread(target=order, name=name) for name in ("early", "late")]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(10)
    return dict(backend._ORACLES), errors


def test_two_threads_racing_the_first_native_order_capture_the_real_oracles_and_do_not_recurse(monkeypatch):
    real_coverage, real_clip = oracles()

    captured, errors = _race(monkeypatch, backend.install_dispatch)

    assert errors == []
    assert captured[backend.COVERAGE] is real_coverage and captured[backend.CLIP] is real_clip
    assert backend._python_clip() is real_clip and backend._python_coverage() is real_coverage
    # подмена одна: имена стоят один раз и возвращаются на место
    assert backend.dispatch_installed() and len(backend._INSTALLED) == 2
    assert (coverage._coverage_at, clip.clip_geometry) == (backend.coverage_compute, backend.clip_compute)
    # и рекурсии нет: домен под заказом NATIVE с отказывающим нативным ядром считается настоящим эталоном

    def refuse(*_args, **_kwargs):
        raise Stale("moved")

    install_native(monkeypatch, coverage_at=refuse, clip_geometry=refuse)
    _make, first, _saturated = FIXTURES["fold"]
    reference = baseline("fold", first, LAWS["triangles"])
    with memo_disabled(), backend.use_backend("NATIVE") as ledger:
        assert answer(build("fold", first, LAWS["triangles"])[0]) == reference
    assert ledger.record().outcomes == ("NATIVE_PORT_STALE",)
    backend.uninstall_dispatch()
    assert (coverage._coverage_at, clip.clip_geometry) == (real_coverage, real_clip)


def test_the_race_harness_catches_the_unlocked_check_then_act(monkeypatch):
    """Контроль: прежняя установка без замка в той же гонке снимает ПОДСТАВЛЕННЫЙ диспетчер как эталон (и позвала бы сама себя)."""

    real_coverage, real_clip = oracles()

    def unlocked():
        if not backend._INSTALLED:
            backend._ORACLES[backend.COVERAGE], backend._ORACLES[backend.CLIP] = coverage._coverage_at, clip.clip_geometry
            replaced = []
            for module, name, dispatcher in (
                (coverage, "_coverage_at", backend.coverage_compute),
                (clip, "clip_geometry", backend.clip_compute),
            ):
                replaced.append((module, name, getattr(module, name)))
                setattr(module, name, dispatcher)
            backend._INSTALLED.extend(replaced)

    try:
        captured, errors = _race(monkeypatch, unlocked)
        assert errors == []
        assert captured[backend.CLIP] is backend.clip_compute  # поток «late» снял диспетчер вместо эталона: рекурсия
        assert captured[backend.COVERAGE] is backend.coverage_compute
    finally:
        coverage._coverage_at, clip.clip_geometry = real_coverage, real_clip


# --------------------------------------------------------------------------
# 8. Поздний отказ порта после частичных эффектов — не откат
# --------------------------------------------------------------------------


def _stub_clip_inputs():
    plane = types.SimpleNamespace(_normal_by_position={"a": (0.0, 0.0, 1.0)})
    inputs = dict(points={}, cycles=[], polygons=[], law=None, seam=[], fans=[], flows=[], by_faces=False)
    return plane, exact_work_budget(stage="CLIP"), inputs


@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(lambda plane, budget, traces, store: setattr(budget, "gcd_operations", budget.gcd_operations + 5), id="budget"),
        pytest.param(lambda plane, budget, traces, store: plane._normal_by_position.update(b=(1.0, 0.0, 0.0)), id="plane-normals"),
        pytest.param(lambda plane, budget, traces, store: plane._normal_by_position.update(a=(0.0, 1.0, 0.0)), id="plane-normal-overwritten"),
        pytest.param(lambda plane, budget, traces, store: exact._FACTORIZATION_MEMO.update({999_983: ((999_983, 1),)}), id="memory-table"),
        pytest.param(lambda plane, budget, traces, store: exact.SIGN_COUNTS.update(total=exact.SIGN_COUNTS["total"] + 1), id="sign-counts"),
    ],
)
def test_a_late_native_refusal_after_effects_is_a_named_refusal_and_the_oracle_never_runs_on_dirty_state(monkeypatch, mutate):
    snapshot = dict(exact.SIGN_COUNTS)
    monkeypatch.setattr(exact, "SIGN_COUNTS", dict(snapshot))
    monkeypatch.setattr(exact, "_FACTORIZATION_MEMO", dict(exact._FACTORIZATION_MEMO))
    plane, budget, inputs = _stub_clip_inputs()
    ran = []
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: ran.append("oracle")))

    def late(plane_, budget_, **_inputs):
        mutate(plane_, budget_, None, None)
        raise Unsupported("a sort of 64 nodes or more")

    install_native(monkeypatch, clip_geometry=late)
    with backend.use_backend("NATIVE") as ledger:
        with pytest.raises(backend.NativePartialEffectsRefused, match="partial effects"):
            backend.clip_compute(plane, budget, **inputs)
    assert ran == []
    record = ledger.record()
    assert record.outcomes == ("NATIVE_PARTIAL_EFFECTS_REFUSED",)
    assert record.fallbacks[0][:3] == ("NATIVE_PARTIAL_EFFECTS_REFUSED", "clip", 1)
    assert (record.native_calls, record.python_calls) == (0, 0) and ledger.partial.startswith("clip:")


def test_a_clean_late_refusal_still_falls_back_to_the_oracle(monkeypatch):
    plane, budget, inputs = _stub_clip_inputs()
    monkeypatch.setattr(backend, "_python_clip", lambda: (lambda *a, **k: "ORACLE"))

    def clean(*_args, **_kwargs):
        raise Unsupported("declined before any effect")

    install_native(monkeypatch, clip_geometry=clean)
    with backend.use_backend("NATIVE") as ledger:
        assert backend.clip_compute(plane, budget, **inputs) == "ORACLE"
    assert ledger.record().outcomes == ("NATIVE_PORT_UNSUPPORTED",) and not ledger.partial


def test_a_coverage_refusal_after_the_traces_were_filled_is_a_partial_effects_refusal(monkeypatch):
    _result, prepared = build("fold", "3.5", LAWS["triangles"])
    oracle, _clip = oracles()
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
        with pytest.raises(backend.NativePartialEffectsRefused):
            backend.coverage_compute(partition, Fraction(7, 4), exact_work_budget(stage="COVERAGE"), {}, [])
    assert ran == [] and ledger.record().outcomes == ("NATIVE_PARTIAL_EFFECTS_REFUSED",)
