"""Скелет как третья операция бэкенда (`cftuv_envelope.backend.skeleton_compute`): диспетчер, стадия заказывается отдельно, откаты названы.

Утверждений семь, и каждое стоит на проверке (нативное ядро здесь подставное: `types.ModuleType`, отвечает эталоном):

1. СТАТУС. Ключ `skeleton` читается из `native_status()` шима; колесо без ключа (старое) и процесс без колеса — именованный `unavailable`, покрытие и резка при этом доступны.
2. СТАДИЯ ЗАКАЗЫВАЕТСЯ ОТДЕЛЬНО. `use_backend(backend, skeleton_backend)`: скелет считает нативное ядро только при заказе `skeleton_backend=NATIVE`, покрытие и резка — только при `backend=NATIVE`;
   обе на `PYTHON` — журнала нет, вызов побитово равен эталону.
3. NATIVE, КОТОРЫЙ СЧИТАЕТ. Подставное ядро, равное эталону, даёт тот же скелет и ту же цену; журнал называет исполнителя СКЕЛЕТА отдельно от покрытия и резки.
4. ОТКАТ ИМЕНОВАН. Отказ порта (`NATIVE_REFUSALS`), отсутствие колеса и операции в колесе, дефект порта до эффектов — эталон на том же бюджете, исход назван в `skeleton_outcomes`;
   `NativeDivisionDiverged` — отказ домена без эталона; сдвиг видимого состояния (в том числе строки `superlevel` бюджета) делает отказ порта отказом домена.
5. ИСХОД ЭТАЛОНА НЕ ГЛОТАЕТСЯ: исключение, которого порт не называет, уходит вызывающему как есть.
6. ТОЧКА ПОДКЛЮЧЕНИЯ: `_prepare_region` зовёт `backend.skeleton_compute` на месте `build_skeleton`, и подготовка под заказом `NATIVE` скелета считает его нативно.
7. ЗАПИСЬ ДОМЕНА. Записи подготовки и материализации сливаются (`BackendRecordV1.merged`), скелет остаётся отдельным; идентичность исполнения несёт обе стадии.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

import developable_factories as df
from developable_route import materialize_developable

from cftuv_envelope import backend
from cftuv_envelope import exact_sqrt_sum as exact
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import exact_work_budget
from cftuv_envelope.wavefront import coverage as coverage_module
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton as oracle_skeleton

POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
ROUTE = ("r0a", "r0b")
LIFT = NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1
KERNEL = Path(backend.__file__).resolve().parent
BUILD_ID = "ab" * 32


@pytest.fixture(autouse=True)
def _backend_state_is_not_shared():
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


def install(monkeypatch, *, skeleton="oracle", status=None, coverage_calls=None):
    """Подставной `cftuv_native` в `sys.modules`; `skeleton="oracle"` — отвечает эталоном и пишет вызовы, `None` — у колеса нет `build_skeleton`, иначе свой вызов."""

    calls: list = []
    module = types.ModuleType("cftuv_native")
    module.NativePortStale, module.NativePortUnsupported, module.NativeUnsupportedPython, module.NativeDivisionDiverged = Stale, Unsupported, UnsupportedPython, Diverged
    module.NATIVE_REFUSALS = (Stale, UnsupportedPython, Unsupported, Diverged)
    module.native_version = lambda: "9.9.9"
    module.native_build_id = lambda: BUILD_ID
    module.native_status = lambda: dict(status or {"coverage": "available", "clip": "available", "skeleton": "available"})

    def covered(*args, **kwargs):
        if coverage_calls is not None:
            coverage_calls.append("coverage")
        return coverage_module._coverage_at(*args, **kwargs)

    module.coverage_at = covered
    module.clip_geometry = lambda *a, **k: None
    if skeleton == "oracle":

        def native_skeleton(polygon, *, split_search=None, work_budget=None, dense_hydration=False):
            calls.append(("skeleton", split_search, dense_hydration))
            keywords = {"work_budget": work_budget, "dense_hydration": dense_hydration}
            if split_search is not None:
                keywords["split_search"] = split_search
            return oracle_skeleton(polygon, **keywords)

        module.build_skeleton = native_skeleton
    elif skeleton is not None:
        module.build_skeleton = skeleton
    monkeypatch.setitem(sys.modules, "cftuv_native", module)
    backend.refresh_native()
    return module, calls


def no_native(monkeypatch):
    monkeypatch.setitem(sys.modules, "cftuv_native", None)
    backend.refresh_native()


def prepared_fold():
    _result, prepared = materialize_developable(
        df.fold_strip(), ROUTE, alpha="3.5", decal_topology_law=POLYGONS, near_planar_lift_law=LIFT
    )
    return prepared


def polygon_of(prepared):
    return prepared.regions[0].bridge.polygon


def price(budget):
    return budget.spent_by_article(), budget.superlevel


def answer(skeleton):
    return (skeleton.outcome, semantic_digest(skeleton), skeleton.levels, skeleton.counters, len(skeleton.nodes))


# --------------------------------------------------------------------------
# 1. Статус
# --------------------------------------------------------------------------


def test_the_status_reads_the_skeleton_key_and_an_old_wheel_without_it_is_unavailable_by_name(monkeypatch):
    install(monkeypatch)
    status = backend.native_status()
    assert (status.coverage, status.clip, status.skeleton) == ("available", "available", "available")
    assert status.available and status.skeleton_available
    assert status.as_record()["skeleton"] == "available"

    install(monkeypatch, status={"coverage": "available", "clip": "available"})  # колесо до скелета: ключа нет
    old = backend.native_status()
    assert old.available and not old.skeleton_available and old.skeleton == backend.UNAVAILABLE
    assert old.as_record()["skeleton"] == "unavailable"

    install(monkeypatch, status={"coverage": "available", "clip": "available", "skeleton": "stale(wavefront/skeleton.py)"})
    stale = backend.native_status()
    assert stale.available and not stale.skeleton_available and stale.skeleton == "stale(wavefront/skeleton.py)"

    no_native(monkeypatch)
    absent = backend.native_status()
    assert absent.skeleton == backend.UNAVAILABLE and not absent.skeleton_available and absent.detail


def test_the_identity_names_both_stages_and_a_python_skeleton_leaves_the_old_string_alone(monkeypatch):
    install(monkeypatch)
    assert backend.stage_identity("PYTHON") == "PYTHON" and backend.stage_identity("NATIVE") == f"NATIVE:{BUILD_ID}"
    assert backend.backend_identity("NATIVE") == backend.backend_identity("NATIVE", "PYTHON") == f"NATIVE:{BUILD_ID}"
    assert backend.backend_identity("PYTHON") == "PYTHON"
    assert backend.backend_identity("NATIVE", "NATIVE") == f"NATIVE:{BUILD_ID}|skeleton=NATIVE:{BUILD_ID}"
    assert backend.backend_identity("PYTHON", "NATIVE") == f"PYTHON|skeleton=NATIVE:{BUILD_ID}"
    # другая сборка — другая идентичность стадии
    install(monkeypatch)
    sys.modules["cftuv_native"].native_build_id = lambda: "cd" * 32
    assert backend.stage_identity("NATIVE") == "NATIVE:" + "cd" * 32 != f"NATIVE:{BUILD_ID}"


# --------------------------------------------------------------------------
# 2. Стадия заказывается отдельно
# --------------------------------------------------------------------------


def test_a_block_with_both_stages_on_python_has_no_ledger_and_the_call_is_the_oracle_call(monkeypatch):
    _module, calls = install(monkeypatch)
    polygon = polygon_of(prepared_fold())
    plain, direct = exact_work_budget(stage="PREPARE"), exact_work_budget(stage="PREPARE")
    with backend.use_backend("PYTHON") as ledger:
        through = backend.skeleton_compute(polygon, work_budget=plain)
    assert ledger is None and calls == []
    assert answer(through) == answer(oracle_skeleton(polygon, work_budget=direct)) and price(plain) == price(direct)
    # и вне блока
    assert answer(backend.skeleton_compute(polygon)) == answer(oracle_skeleton(polygon))
    assert backend.active_backend() is backend.KernelBackendV1.PYTHON and backend.active_skeleton_backend() is backend.KernelBackendV1.PYTHON


@pytest.mark.parametrize(
    "coverage_clip, skeleton, native_skeleton, native_coverage_clip",
    [("PYTHON", "NATIVE", True, False), ("NATIVE", "PYTHON", False, True), ("NATIVE", "NATIVE", True, True), ("PYTHON", "PYTHON", False, False)],
)
def test_each_stage_is_computed_natively_only_when_it_is_the_one_ordered(monkeypatch, coverage_clip, skeleton, native_skeleton, native_coverage_clip):
    coverage_calls: list = []
    _module, calls = install(monkeypatch, coverage_calls=coverage_calls)
    polygon = polygon_of(prepared_fold())
    partition = prepared_fold().regions[0].partition
    with backend.use_backend(coverage_clip, skeleton) as ledger:
        backend.skeleton_compute(polygon)
        backend.coverage_compute(partition, 2, exact_work_budget(stage="COVERAGE"), {})
        assert backend.active_backend().value == coverage_clip and backend.active_skeleton_backend().value == skeleton
    assert bool(calls) is native_skeleton and bool(coverage_calls) is native_coverage_clip
    if ledger is None:
        assert (coverage_clip, skeleton) == ("PYTHON", "PYTHON")
        return
    record = ledger.record()
    assert (record.skeleton_native_calls, record.native_calls) == (int(native_skeleton), int(native_coverage_clip))
    assert (record.requested, record.skeleton_requested) == (coverage_clip, skeleton)
    # стадия, заказанная на `PYTHON`, журнала не пишет: её вызов не виден ни как счёт, ни как откат
    assert record.skeleton_python_calls == record.python_calls == 0 and not record.fallbacks and not record.skeleton_fallbacks


# --------------------------------------------------------------------------
# 3. NATIVE, который считает
# --------------------------------------------------------------------------


def test_a_native_equal_to_the_oracle_gives_the_same_skeleton_and_price_and_is_named_the_skeleton_runner(monkeypatch):
    _module, calls = install(monkeypatch)
    polygon = polygon_of(prepared_fold())
    own, direct = exact_work_budget(stage="PREPARE"), exact_work_budget(stage="PREPARE")
    with backend.use_backend("PYTHON", "NATIVE") as ledger:
        native = backend.skeleton_compute(polygon, work_budget=own)
    reference = oracle_skeleton(polygon, work_budget=direct)
    assert answer(native) == answer(reference) and price(own) == price(direct)
    assert len(calls) == 1
    record = ledger.record()
    assert (record.skeleton_ran, record.skeleton_native_calls, record.skeleton_python_calls) == ("native", 1, 0)
    assert record.skeleton_outcomes == () and record.skeleton_fallbacks == ()
    # покрытие и резка в этом блоке не заказаны: их счёта нет, как нет и `NATIVE_NOT_REACHED` (заказ `PYTHON`)
    assert (record.native_calls, record.python_calls, record.outcomes, record.ran) == (0, 0, (), "python")


def test_the_split_search_and_the_hydration_flag_reach_the_native_as_asked_and_the_oracle_default_is_not_passed(monkeypatch):
    _module, calls = install(monkeypatch)
    polygon = polygon_of(prepared_fold())
    with backend.use_backend("PYTHON", "NATIVE"):
        backend.skeleton_compute(polygon)
        backend.skeleton_compute(polygon, split_search=SplitSearch.EXHAUSTIVE, dense_hydration=True)
    assert calls == [("skeleton", None, False), ("skeleton", SplitSearch.EXHAUSTIVE, True)]


# --------------------------------------------------------------------------
# 4. Откат назван
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "failure, outcome",
    [
        (lambda: Stale("moved"), "NATIVE_PORT_STALE"),
        (lambda: Unsupported("python -O"), "NATIVE_PORT_UNSUPPORTED"),
        (lambda: UnsupportedPython("3.10"), "NATIVE_UNSUPPORTED_PYTHON"),
        (lambda: TypeError("cftuv_native: a Python object the extension does not carry"), "NATIVE_INTERNAL_ERROR"),
        (lambda: RuntimeError("Session.build_skeleton: native panic: boom"), "NATIVE_INTERNAL_ERROR"),
    ],
)
def test_a_named_native_refusal_is_computed_by_the_oracle_on_the_same_budget_and_recorded(monkeypatch, failure, outcome):
    polygon = polygon_of(prepared_fold())

    def refuse(_polygon, **_kwargs):
        raise failure()

    install(monkeypatch, skeleton=refuse)
    own, direct = exact_work_budget(stage="PREPARE"), exact_work_budget(stage="PREPARE")
    with backend.use_backend("NATIVE", "NATIVE") as ledger:
        fallen = backend.skeleton_compute(polygon, work_budget=own)
    assert answer(fallen) == answer(oracle_skeleton(polygon, work_budget=direct)) and price(own) == price(direct)
    record = ledger.record()
    assert (record.skeleton_ran, record.skeleton_native_calls, record.skeleton_python_calls) == ("python", 0, 1)
    assert record.skeleton_outcomes == (outcome,) and record.skeleton_fallbacks[0][:3] == (outcome, "skeleton", 1)
    # откат скелета не попадает в исходы покрытия и резки
    assert record.fallbacks == () and record.python_calls == 0


def test_an_absent_wheel_and_a_wheel_without_the_operation_fall_back_by_the_name_unavailable(monkeypatch):
    polygon = polygon_of(prepared_fold())
    reference = answer(oracle_skeleton(polygon))
    no_native(monkeypatch)
    with backend.use_backend("PYTHON", "NATIVE") as ledger:
        assert answer(backend.skeleton_compute(polygon)) == reference
    record = ledger.record()
    assert record.skeleton_outcomes == ("NATIVE_UNAVAILABLE",) and record.skeleton_ran == "python"
    assert record.skeleton_fallbacks[0][3]  # причина загрузки названа текстом

    install(monkeypatch, skeleton=None, status={"coverage": "available", "clip": "available"})
    assert backend.native_status().available and not backend.native_status().skeleton_available
    with backend.use_backend("PYTHON", "NATIVE") as ledger:
        assert answer(backend.skeleton_compute(polygon)) == reference
    record = ledger.record()
    assert record.skeleton_outcomes == ("NATIVE_UNAVAILABLE",) and "build_skeleton" in record.skeleton_fallbacks[0][3]
    assert (record.skeleton_native_calls, record.skeleton_python_calls) == (0, 1)


def test_a_diverged_division_refuses_the_domain_by_name_and_the_oracle_never_runs(monkeypatch):
    polygon = polygon_of(prepared_fold())
    ran: list = []
    monkeypatch.setattr(backend, "_python_skeleton", lambda: (lambda *a, **k: ran.append("oracle")))

    def diverge(_polygon, **_kwargs):
        raise Diverged("the generic division fallback did not finish")

    install(monkeypatch, skeleton=diverge)
    with backend.use_backend("NATIVE", "NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused, match="would not either") as raised:
            backend.skeleton_compute(polygon)
    assert ran == []
    assert raised.value.outcome is backend.BackendOutcomeV1.NATIVE_DIVISION_DIVERGED and isinstance(raised.value.__cause__, Diverged)
    record = ledger.record()
    assert record.skeleton_outcomes == ("NATIVE_DIVISION_DIVERGED",) and record.skeleton_fallbacks[0][:3] == ("NATIVE_DIVISION_DIVERGED", "skeleton", 1)
    assert ledger.refusal[0] == "NATIVE_DIVISION_DIVERGED" and ledger.refusal[1].startswith("skeleton:")


_DIRTY = [
    pytest.param(lambda budget: setattr(budget, "modular_squarings", budget.modular_squarings + 5), id="budget-article"),
    pytest.param(lambda budget: setattr(budget, "superlevel", "moved-by-the-port"), id="superlevel-row"),
    pytest.param(lambda budget: exact._FACTORIZATION_MEMO.__setitem__(-7, ((), 1)), id="memory-table"),
    pytest.param(lambda budget: exact.SIGN_COUNTS.__setitem__("late_sign", 1), id="sign-counts"),
]


@pytest.mark.parametrize("mutate", _DIRTY)
def test_a_native_refusal_after_visible_effects_is_a_domain_refusal_and_the_oracle_never_runs_on_dirty_state(monkeypatch, mutate):
    polygon = polygon_of(prepared_fold())
    monkeypatch.setattr(exact, "SIGN_COUNTS", dict(exact.SIGN_COUNTS))
    monkeypatch.setattr(exact, "_FACTORIZATION_MEMO", dict(exact._FACTORIZATION_MEMO))
    ran: list = []
    monkeypatch.setattr(backend, "_python_skeleton", lambda: (lambda *a, **k: ran.append("oracle")))

    def late(_polygon, *, work_budget=None, **_kwargs):
        mutate(work_budget)
        raise Stale("moved after the first effect")

    install(monkeypatch, skeleton=late)
    with backend.use_backend("NATIVE", "NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused, match="partial effects") as raised:
            backend.skeleton_compute(polygon, work_budget=exact_work_budget(stage="PREPARE"))
    assert ran == [] and raised.value.outcome is backend.BackendOutcomeV1.NATIVE_PARTIAL_EFFECTS_REFUSED
    assert ledger.record().skeleton_outcomes == ("NATIVE_PARTIAL_EFFECTS_REFUSED",)
    assert ledger.refusal[0] == "NATIVE_PARTIAL_EFFECTS_REFUSED" and ledger.refusal[1].startswith("skeleton:")


# --------------------------------------------------------------------------
# 5. Исход эталона и чужое исключение не глотаются
# --------------------------------------------------------------------------


@pytest.mark.parametrize("raised", [ValueError("the oracle text"), KeyError("k"), OverflowError("big"), TypeError("an internal TypeError of the oracle itself")])
def test_an_oracle_outcome_reaches_the_caller_as_is_and_is_not_counted_as_a_fallback(monkeypatch, raised):
    polygon = polygon_of(prepared_fold())

    def outcome(_polygon, **_kwargs):
        raise raised

    install(monkeypatch, skeleton=outcome)
    with backend.use_backend("NATIVE", "NATIVE") as ledger:
        with pytest.raises(type(raised)) as caught:
            backend.skeleton_compute(polygon)
    assert caught.value is raised
    record = ledger.record()
    assert record.skeleton_fallbacks == () and (record.skeleton_native_calls, record.skeleton_python_calls) == (0, 0) and ledger.refusal is None


# --------------------------------------------------------------------------
# 6. Точка подключения
# --------------------------------------------------------------------------


def test_the_prepare_region_hook_calls_the_dispatcher_in_place_of_the_oracle():
    text = (KERNEL / "wavefront" / "conveyor.py").read_text(encoding="utf-8").replace("\r\n", "\n")
    assert text.count("skeleton = backend.skeleton_compute(\n        report.polygon,\n        work_budget=work_budget,\n        dense_hydration=dense_hydration,\n    )\n") == 1
    assert "build_skeleton" not in text and "from .skeleton import SkeletonOutcome, SkeletonV1\n" in text
    # диспетчер стоит у вызывающего: имя эталона в модуле скелета не подменяется
    assert "setattr(" not in (KERNEL / "backend.py").read_text(encoding="utf-8")


def test_a_preparation_under_a_native_skeleton_computes_its_skeletons_natively_and_gives_the_same_preparation(monkeypatch):
    _module, calls = install(monkeypatch)
    from cftuv_envelope.wavefront import prepare_conveyor

    prepared_python = prepared_fold()
    assert calls == []
    snapshot, request = _snapshot_and_request()
    plain = prepare_conveyor(snapshot, request)
    with backend.use_backend("PYTHON", "NATIVE") as ledger:
        native = prepare_conveyor(snapshot, request)
    assert len(calls) == len(plain.regions) >= 1
    assert [(semantic_digest(item.skeleton), item.skeleton_outcome) for item in native.regions] == [
        (semantic_digest(item.skeleton), item.skeleton_outcome) for item in plain.regions
    ]
    assert native.work_budget.counters() == plain.work_budget.counters()
    assert ledger.record().skeleton_native_calls == len(plain.regions) and prepared_python.outcome == plain.outcome


def _snapshot_and_request():
    """Снапшот и запрос того же домена, из которого `materialize_developable` готовит сгиб: подготовка без материализации."""

    from developable_route import developable_domain

    return developable_domain(df.fold_strip(), ROUTE, alpha="3.5")


# --------------------------------------------------------------------------
# 7. Запись домена
# --------------------------------------------------------------------------


def test_the_records_of_two_blocks_merge_and_keep_the_skeleton_apart_from_coverage_and_clip():
    first = backend.BackendRecordV1("NATIVE", 0, 0, (), "NATIVE", 1, 2, (("NATIVE_PORT_STALE", "skeleton", 2, "first text"),))
    second = backend.BackendRecordV1(
        "NATIVE", 3, 1, (("NATIVE_PORT_UNSUPPORTED", "clip", 1, "clip text"),), "NATIVE", 0, 1, (("NATIVE_PORT_STALE", "skeleton", 1, "second text"),)
    )
    merged = first.merged(second)
    assert (merged.requested, merged.native_calls, merged.python_calls, merged.ran) == ("NATIVE", 3, 1, "mixed")
    assert merged.fallbacks == (("NATIVE_PORT_UNSUPPORTED", "clip", 1, "clip text"),) and merged.outcomes == ("NATIVE_PORT_UNSUPPORTED",)
    assert (merged.skeleton_requested, merged.skeleton_native_calls, merged.skeleton_python_calls, merged.skeleton_ran) == ("NATIVE", 1, 3, "mixed")
    assert merged.skeleton_fallbacks == (("NATIVE_PORT_STALE", "skeleton", 3, "first text"),) and merged.skeleton_outcomes == ("NATIVE_PORT_STALE",)
    assert first.merged(None) is first
    # скелет, которого в прогоне не было, — пусто, а не откат; заказ нативного покрытия без единой операции — по-прежнему `NATIVE_NOT_REACHED`
    empty = backend.BackendRecordV1("NATIVE", 0, 0, (), "NATIVE", 0, 0, ())
    assert empty.skeleton_ran == "" and empty.skeleton_outcomes == () and empty.outcomes == ("NATIVE_NOT_REACHED",)
    record = merged.as_record()
    assert record["skeleton_ran"] == "mixed" and record["skeleton_outcomes"] == ["NATIVE_PORT_STALE"] and record["ran"] == "mixed"
