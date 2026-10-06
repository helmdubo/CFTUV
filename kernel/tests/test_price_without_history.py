"""Цена и ответ вычисления домена - функция его ВХОДА: от состояния кэшей, истории, раскладки по воркерам и порядка вычислений не зависят.

Правило (в одной строке DECISIONS 2026-10-06, PRICE-WITHOUT-HISTORY): вход вычисления - подготовка (alpha-независимая), ширина и законы;
цена - шесть статей `ExactWorkBudgetV1` покрытия (копия состояния подготовки, `ConveyorCoverageV1.work_budget`) и материализации
(`EXACT_WORK_*` в `counters` ответа). До правила цена зависела от истории двумя способами, и обе зависимости убраны:

1. ПАМЯТЬ ПОКРЫТИЙ. `coverage_at` помнил недавние покрытия по `(тождество разбиения, alpha)`, а материализатор брал контуры вторым вызовом:
   попадание не платило, промах платил (до 133 единиц на поле), и при потолке, равном цене промаха минус единица, попадание
   материализовало, а промах отказывал по бюджету. Теперь контуры едут в записи региона из ТОГО ЖЕ вычисления.
2. БЮДЖЕТ ПОДГОТОВКИ. Покрытие тратило бюджет самой подготовки: холодная цена покрытия ездила в пикле, а тёплые шаги копились на ней
   в родителе (1724, 1726, 1728, 1730). Теперь каждое покрытие начинается с копии состояния подготовки и подготовку не меняет, а повтор
   вселенной простых платит записанную цену (`prime_universe_remembered`), как и шаблон шага ширины (`CoverageTemplateV1.price`).

Ничего не принимается на веру: каждый путь (холодный, тёплый, повторный, туда-обратно, развёрнутый из пикла воркера, шаг из шаблона,
контуры из записи и пересчитанные) сравнивается с полным путём на холодной подготовке тем же входом.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os
import pickle
import subprocess
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

import developable_factories as df
import materialize_factories as factories
import cftuv_envelope.wavefront as wavefront_package
from cftuv_envelope import exact_sqrt_sum as exact
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.interactions import policy_b
from cftuv_envelope.materialize import interval as interval_module
from cftuv_envelope.materialize import step as step_module
from cftuv_envelope.materialize.admit import materialization_request
from cftuv_envelope.materialize.coalesce import region_contours
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.step import ENVIRONMENT_SWITCH, ENVIRONMENT_VERIFY, STEP_COUNTERS, answer_differences, step_domain
from cftuv_envelope.reference.planar_types import ConstructionCertificate, ConstructionKind
from cftuv_envelope.wavefront import conveyor_coverage
from cftuv_envelope.wavefront.conveyor import ConveyorOutcome
from cftuv_envelope.wavefront.coverage import coverage_source

from test_interval_step import CASES, alpha_text, certificate_of, developable

HERE = Path(__file__).resolve().parent
KERNEL_SOURCE = HERE.parent / "src"

ARTICLES = 6


@pytest.fixture(autouse=True)
def _clean_environment(monkeypatch):
    monkeypatch.delenv(ENVIRONMENT_SWITCH, raising=False)
    monkeypatch.delenv(ENVIRONMENT_VERIFY, raising=False)


def request_of(prepared):
    return materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1")


def arguments_of(prepared, laws):
    return (request_of(prepared), laws[0], laws[1], False)


def pristine(build):
    """Подготовка, как её вернул `prepare_conveyor`, пиклом: каждый `pickle.loads` даёт холодную копию (память контактов и шагов пуста)."""

    return pickle.dumps(build())


def widths_of(prepared, given, offsets=(0, 1, -1, 2)):
    base = Fraction(str(alpha_text(prepared, given)))
    return [str(round(float(base * (1 + Fraction(offset, 1000))), 9)) for offset in offsets]


class Evaluation:
    """Что стоит вычисление и что оно отвечает: цена покрытия, цена материализации (в `counters` ответа), ответ."""

    def __init__(self, coverage, result):
        self.coverage = coverage
        self.result = result
        self.coverage_price = None if coverage.work_budget is None else coverage.work_budget.spent_by_article()
        self.materialize_price = tuple(item for item in result.counters if str(item[0]).startswith("EXACT_WORK_"))

    def differences(self, other):
        found = list(answer_differences(self.result, other.result))
        if self.coverage_price != other.coverage_price:
            found.append(f"coverage_price {self.coverage_price} != {other.coverage_price}")
        if self.materialize_price != other.materialize_price:
            found.append("materialize_price")
        if self.coverage.outcome != other.coverage.outcome or self.coverage.detail != other.coverage.detail:
            found.append("coverage_outcome")
        return tuple(found)


def evaluate_full(prepared, alpha, laws, *, work_budget=None):
    """Полный путь без источника покрытия: `conveyor_coverage`, затем `materialize_domain` на нём."""

    coverage = conveyor_coverage(prepared, str(alpha))
    result = materialize_domain(
        prepared,
        coverage,
        request=request_of(prepared),
        near_planar_lift_law=laws[0],
        decal_topology_law=laws[1],
        digests=False,
        certify=True,
        work_budget=work_budget,
    )
    return Evaluation(coverage, result)


def evaluate_step(prepared, alpha, laws, monkeypatch):
    """Шаг ширины (`step_domain`): быстрый путь внутри заверенного интервала, иначе полный с записью; покрытие записано шпионом."""

    seen = []
    original = wavefront_package.conveyor_coverage

    def spy(*args, **kwargs):
        found = original(*args, **kwargs)
        seen.append(found)
        return found

    monkeypatch.setattr(wavefront_package, "conveyor_coverage", spy)
    try:
        stepped = step_domain(
            prepared,
            str(alpha),
            request=request_of(prepared),
            near_planar_lift_law=laws[0],
            decal_topology_law=laws[1],
            digests=False,
        )
    finally:
        monkeypatch.setattr(wavefront_package, "conveyor_coverage", original)
    assert len(seen) == 1, "one coverage per step"
    return stepped, Evaluation(seen[0], stepped.result)


# ---------------------------------------------------------------- 1. бюджет подготовки неприкосновенен


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_the_coverage_works_on_a_copy_and_leaves_the_preparation_budget_alone(name, build, laws, given):
    prepared = build()
    before = (prepared.work_budget.spent_by_article(), prepared.work_budget.stage)
    assert before[1] == "PREPARE"
    seen = []
    for width in widths_of(prepared, given)[:3]:
        coverage = conveyor_coverage(prepared, width)
        assert coverage.outcome is ConveyorOutcome.EXACT, coverage.detail
        assert coverage.work_budget is not prepared.work_budget and coverage.work_budget.stage == "COVERAGE"
        assert coverage.work_budget.cap == prepared.work_budget.cap and coverage.work_budget.domain_id == prepared.work_budget.domain_id
        seen.append(coverage.work_budget.spent_by_article())
        assert (prepared.work_budget.spent_by_article(), prepared.work_budget.stage) == before, "the preparation is what PREPARE left"
    for article in range(ARTICLES):
        assert all(item[article] >= before[0][article] for item in seen), "the copy starts from the state of the preparation"


def test_a_forked_budget_is_an_independent_copy_on_its_own_stage():
    budget = exact.exact_work_budget(stage="PREPARE", domain_id="d", cap=1000)
    budget.superlevel = "3"
    budget.gcd_operations, budget.radical_materializations = 5, 2
    fork = budget.forked("COVERAGE")
    assert (fork.mode, fork.cap, fork.stage, fork.domain_id, fork.superlevel) == (budget.mode, 1000, "COVERAGE", "d", "3")
    assert fork.spent_by_article() == budget.spent_by_article() and budget.stage == "PREPARE"
    fork.gcd_operations += 7
    assert budget.gcd_operations == 5 and fork.gcd_operations == 12


# ---------------------------------------------------------------- 2. цена покрытия: холодное, тёплое, повторное, из пикла


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_the_coverage_price_is_the_same_cold_warm_repeated_and_unpickled(name, build, laws, given):
    blob = pristine(build)
    prepared = pickle.loads(blob)
    width = widths_of(prepared, given)[0]
    cold = conveyor_coverage(prepared, width)
    assert cold.outcome is ConveyorOutcome.EXACT, cold.detail
    price = cold.work_budget.spent_by_article()
    assert sum(price) > sum(prepared.work_budget.spent_by_article()) - 1
    # Тёплая подготовка (вселенная простых в памяти подготовки), тёплая память канонизации процесса, ещё раз, и копия из пикла воркера.
    warm = conveyor_coverage(prepared, width)
    again = conveyor_coverage(prepared, width)
    exact.reset_factorization_memory()
    after_reset = conveyor_coverage(prepared, width)
    worker = pickle.loads(pickle.dumps(prepared))
    from_pickle = conveyor_coverage(worker, width)
    for other in (warm, again, after_reset, from_pickle):
        assert other.work_budget.spent_by_article() == price, name
        assert other.doubled_area == cold.doubled_area and other.counters == cold.counters
    assert pickle.loads(pickle.dumps(prepared)).work_budget.spent_by_article() == prepared.work_budget.spent_by_article()


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_the_coverage_price_does_not_depend_on_the_memory_the_process_holds(name, build, laws, given):
    blob = pristine(build)
    prepared = pickle.loads(blob)
    width = widths_of(prepared, given)[0]
    exact.reset_factorization_memory()
    cold = conveyor_coverage(pickle.loads(blob), width)
    # Память канонизации процесса после покрытия та же, что до него (пустая), поэтому второе покрытие на СВЕЖЕЙ подготовке стоит так же.
    seen = conveyor_coverage(pickle.loads(blob), width)
    assert seen.work_budget.spent_by_article() == cold.work_budget.spent_by_article()
    # Память процесса тёплая чем-то другим: радикалы и разложения подготовки и материализации чужой ширины.
    exact.reset_factorization_memory()
    other = pickle.loads(blob)
    evaluate_full(other, widths_of(other, given)[1], laws)
    warm_marker = exact.factorization_memory_marker()
    held = conveyor_coverage(pickle.loads(blob), width)
    assert held.work_budget.spent_by_article() == cold.work_budget.spent_by_article()
    assert exact.factorization_memory_marker() == warm_marker, "the coverage leaves the memory of the caller as it found it"


# ---------------------------------------------------------------- 3. вычисление целиком: история ширин не меняет ни цену, ни ответ


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_a_width_history_there_and_back_changes_neither_the_price_nor_the_answer(name, build, laws, given, monkeypatch):
    blob = pristine(build)
    history = pickle.loads(blob)
    widths = widths_of(history, given, offsets=(0, 1, 0, 2, 1, 0, -1, 1))
    spent = history.work_budget.spent_by_article()
    reference = {}
    paths = []
    for width in widths:
        stepped, evaluated = evaluate_step(history, width, laws, monkeypatch)
        if width not in reference:
            reference[width] = evaluate_full(pickle.loads(blob), width, laws)
        assert stepped.result.is_materialized, stepped.result.detail
        assert evaluated.differences(reference[width]) == (), (name, width, stepped.path)
        assert history.work_budget.spent_by_article() == spent, "no evaluation changes the preparation"
        paths.append(stepped.path)
    assert any(path == step_module.PATH_FAST for path in paths), (name, paths)


@pytest.mark.parametrize("name,build,laws,given", CASES[:5], ids=[item[0] for item in CASES[:5]])
def test_the_evaluation_order_does_not_matter(name, build, laws, given, monkeypatch):
    blob = pristine(build)
    probe = pickle.loads(blob)
    widths = widths_of(probe, given, offsets=(0, 1, -1, 2))
    outcomes = {}
    for order in (widths, list(reversed(widths))):
        prepared = pickle.loads(blob)
        for width in order:
            evaluated = evaluate_step(prepared, width, laws, monkeypatch)[1]
            known = outcomes.setdefault(width, evaluated)
            assert evaluated.differences(known) == (), (name, width)


@pytest.mark.parametrize("name,build,laws,given", CASES[:5], ids=[item[0] for item in CASES[:5]])
def test_the_worker_image_of_a_warm_preparation_prices_like_the_parent(name, build, laws, given, monkeypatch):
    blob = pristine(build)
    parent = pickle.loads(blob)
    widths = widths_of(parent, given, offsets=(0, 1, 2))
    in_process = [evaluate_step(parent, width, laws, monkeypatch)[1] for width in widths]
    # Воркер получает пикл подготовки ПОСЛЕ холодного вычисления и считает на нём (память шага у воркера своя, у пикла пустая).
    worker = pickle.loads(pickle.dumps(parent))
    assert worker.work_budget.spent_by_article() == parent.work_budget.spent_by_article()
    for width, expected in zip(widths, in_process):
        assert evaluate_step(worker, width, laws, monkeypatch)[1].differences(expected) == (), (name, width)
    # Бюджет воркера не нужно возвращать в состояние разворота: покрытие его не трогает.
    assert worker.work_budget.spent_by_article() == parent.work_budget.spent_by_article()


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_the_fast_path_charges_exactly_what_the_full_path_charges(name, build, laws, given, monkeypatch):
    prepared = build()
    base = widths_of(prepared, given)[0]
    recorded, _ = evaluate_step(prepared, base, laws, monkeypatch)
    certificate = certificate_of(prepared, laws)
    assert certificate is not None and all(item.price is not None for item in certificate.templates.values())
    hits = 0
    for width in widths_of(prepared, given, offsets=(1, -1, 2, -2, 3)):
        if not certificate.covers(Fraction(width)):
            continue
        stepped, fast = evaluate_step(prepared, width, laws, monkeypatch)
        assert stepped.is_hit, (name, width, stepped.path)
        full = evaluate_full(prepared, width, laws)
        assert fast.coverage_price == full.coverage_price, (name, width, fast.coverage_price, full.coverage_price)
        assert fast.materialize_price == full.materialize_price
        assert step_module.coverage_price_differences(fast.coverage, full.coverage) == ()
        hits += 1
    assert hits >= 1, "no width inside the certified interval to compare"
    assert recorded.result.is_materialized


def test_the_verification_names_a_price_that_differs_between_the_fast_and_the_full_path(monkeypatch):
    prepared = developable(df.fold_strip)
    laws = CASES[0][2]
    step_module.step_domain(prepared, "0.3", request=request_of(prepared), near_planar_lift_law=laws[0], decal_topology_law=laws[1], digests=False)
    certificate = certificate_of(prepared, laws)
    certificate.templates = {key: dataclasses.replace(value, price=(0, 0, 0, 0, 0, 0)) for key, value in certificate.templates.items()}
    monkeypatch.setenv(ENVIRONMENT_VERIFY, "1")
    before = STEP_COUNTERS[step_module.VERIFY_MISMATCH]
    stepped = step_domain(prepared, "0.301", request=request_of(prepared), near_planar_lift_law=laws[0], decal_topology_law=laws[1], digests=False)
    assert stepped.path.startswith(step_module.VERIFY_MISMATCH + ":") and "coverage_price" in stepped.path, stepped.path
    assert STEP_COUNTERS[step_module.VERIFY_MISMATCH] == before + 1
    assert stepped.result.is_materialized, "the answer on a mismatch is the full one"


# ---------------------------------------------------------------- 4. потолок: окно у самой цены не зависит от пути


@pytest.mark.parametrize("name,build,laws,given", CASES[:6], ids=[item[0] for item in CASES[:6]])
def test_a_cap_one_below_the_coverage_price_refuses_on_every_path_and_the_exact_cap_answers_on_every_path(name, build, laws, given, monkeypatch):
    blob = pristine(build)
    probe = pickle.loads(blob)
    width = widths_of(probe, given)[0]
    priced = conveyor_coverage(probe, width)
    total = priced.work_budget.spent
    assert priced.outcome is ConveyorOutcome.EXACT and total > probe.work_budget.spent

    def at_cap(prepared, cap):
        prepared.work_budget.cap = cap
        return prepared

    tight, exactly = total - 1, total
    refused = conveyor_coverage(at_cap(pickle.loads(blob), tight), width)
    assert refused.outcome is ConveyorOutcome.EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED
    assert conveyor_coverage(at_cap(pickle.loads(blob), exactly), width).outcome is ConveyorOutcome.EXACT

    # Тёплая подготовка (вселенная простых записана): записанная цена не влезает в остаток потолка - счёт заново и тот же отказ.
    warm = pickle.loads(blob)
    conveyor_coverage(warm, width)
    again = conveyor_coverage(at_cap(warm, tight), width)
    assert again.outcome is refused.outcome and again.detail == refused.detail
    assert conveyor_coverage(at_cap(warm, exactly), width).outcome is ConveyorOutcome.EXACT

    # Подготовка из пикла после холодного вычисления (воркер).
    worker = at_cap(pickle.loads(pickle.dumps(warm)), tight)
    on_worker = conveyor_coverage(worker, width)
    assert on_worker.outcome is refused.outcome and on_worker.detail == refused.detail

    # Шаг ширины: покрытие из шаблона платит записанную цену, и в окне у цены отказывает так же, как полный путь.
    stepper = pickle.loads(blob)
    first, _ = evaluate_step(stepper, width, laws, monkeypatch)
    assert first.result.is_materialized
    certificate = certificate_of(stepper, laws)
    inside = next((item for item in widths_of(stepper, given, offsets=(1, -1, 2, -2)) if certificate.covers(Fraction(item))), None)
    if inside is not None:
        cold_inside = conveyor_coverage(pickle.loads(blob), inside).work_budget.spent
        fast_cap = at_cap(stepper, cold_inside - 1)
        source = step_module._Instantiator(certificate)
        with coverage_source(source):
            on_step = conveyor_coverage(fast_cap, inside)
        reference = conveyor_coverage(at_cap(pickle.loads(blob), cold_inside - 1), inside)
        assert on_step.outcome is reference.outcome is ConveyorOutcome.EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED
        assert on_step.detail == reference.detail
        fast_cap.work_budget.cap = cold_inside
        with coverage_source(step_module._Instantiator(certificate)):
            assert conveyor_coverage(fast_cap, inside).outcome is ConveyorOutcome.EXACT


@pytest.mark.parametrize("name,build,laws,given", CASES[:6], ids=[item[0] for item in CASES[:6]])
def test_a_materialization_cap_at_the_price_answers_the_same_on_every_path(name, build, laws, given, monkeypatch):
    blob = pristine(build)
    width = widths_of(pickle.loads(blob), given)[0]
    cold = evaluate_full(pickle.loads(blob), width, laws)
    price = dict(cold.materialize_price)["EXACT_WORK_SPENT"]
    assert cold.result.is_materialized and price > 0

    def run(prepared, coverage, cap):
        return Evaluation(
            coverage,
            materialize_domain(
                prepared,
                coverage,
                request=request_of(prepared),
                near_planar_lift_law=laws[0],
                decal_topology_law=laws[1],
                digests=False,
                certify=True,
                work_budget=exact.exact_work_budget(stage="MATERIALIZE", domain_id="cap", cap=cap),
            ),
        )

    for cap in (price - 1, price):
        flat = pickle.loads(blob)
        base = run(flat, conveyor_coverage(flat, width), cap)
        # Покрытие, у которого записи регионов собраны без контуров (пересчёт контуров вместо переданных), - тот же ответ и та же цена.
        stripped_source = pickle.loads(blob)
        coverage = conveyor_coverage(stripped_source, width)
        stripped = dataclasses.replace(coverage, regions=tuple(dataclasses.replace(item, contours=None) for item in coverage.regions))
        assert all(item.contours is None for item in stripped.regions)
        assert run(stripped_source, stripped, cap).differences(base) == (), (name, cap)
        # Тёплая подготовка, покрытие из шаблона и пикл воркера.
        warm = pickle.loads(blob)
        evaluate_step(warm, width, laws, monkeypatch)
        certificate = certificate_of(warm, laws)
        with coverage_source(step_module._Instantiator(certificate)):
            from_template = conveyor_coverage(warm, width)
        assert run(warm, from_template, cap).differences(base) == (), (name, cap)
        worker = pickle.loads(pickle.dumps(warm))
        assert run(worker, conveyor_coverage(worker, width), cap).differences(base) == (), (name, cap)


def test_contours_cost_nothing_whether_they_are_carried_or_recomputed():
    prepared = factories.field_domain("building_002_point_contact_v1")[0]
    coverage = conveyor_coverage(prepared, None)
    for region, covered in zip(prepared.regions, coverage.regions):
        assert covered.contours is not None and len(covered.contours) == len(covered.faces)
        assert [face.owner for face in covered.contours] == [face.owner for face in covered.faces]
        carried = exact.exact_work_budget(stage="MATERIALIZE")
        assert region_contours(region, coverage.lattice_alpha, carried, covered) is covered.contours
        assert carried.spent == 0, "carried contours are not paid for"
        recomputed = exact.exact_work_budget(stage="MATERIALIZE")
        marker = exact.factorization_memory_marker()
        again = region_contours(region, coverage.lattice_alpha, recomputed, dataclasses.replace(covered, contours=None))
        assert again == covered.contours
        assert recomputed.spent == 0, "recomputed contours leave the price of the evaluation alone too: hit == miss"
        assert exact.factorization_memory_marker() == marker, "and the memory the later stages see"


# ---------------------------------------------------------------- 5. вселенная простых и шаблон: цена записана и повторяется


def _q_values(prepared):
    partition = prepared.regions[0].partition
    return tuple(face.line.q for face in partition.faces)


def test_a_remembered_prime_universe_pays_the_recorded_price_on_every_repeat():
    prepared = factories.field_domain("building_002_point_contact_v1")[0]
    q_values = _q_values(prepared)
    store: dict = {}
    exact.reset_factorization_memory()
    cold = exact.exact_work_budget(stage="COVERAGE")
    universe = exact.prime_universe_remembered(q_values, cold, store)
    price = cold.spent_by_article()
    assert cold.spent > 0
    for _ in range(3):
        warm = exact.exact_work_budget(stage="COVERAGE")
        exact.reset_factorization_memory()
        assert exact.prime_universe_remembered(q_values, warm, store) == universe
        assert warm.spent_by_article() == price
    # Без бюджета повтор ничего не платит и отвечает тем же.
    assert exact.prime_universe_remembered(q_values, None, store) == universe


def test_a_recorded_price_that_does_not_fit_the_cap_is_recomputed_and_refuses_where_the_cold_run_would():
    prepared = factories.field_domain("building_002_point_contact_v1")[0]
    q_values = _q_values(prepared)
    store: dict = {}
    exact.reset_factorization_memory()
    cold = exact.exact_work_budget(stage="COVERAGE")
    exact.prime_universe_remembered(q_values, cold, store)
    tight = exact.exact_work_budget(stage="COVERAGE", cap=cold.spent - 1)
    exact.reset_factorization_memory()
    with pytest.raises(exact.ExactCanonicalizationWorkBudgetExhausted) as warm_refusal:
        exact.prime_universe_remembered(q_values, tight, store)
    exact.reset_factorization_memory()
    cold_tight = exact.exact_work_budget(stage="COVERAGE", cap=cold.spent - 1)
    with pytest.raises(exact.ExactCanonicalizationWorkBudgetExhausted) as cold_refusal:
        exact.prime_universe_remembered(q_values, cold_tight, {})
    assert str(warm_refusal.value) == str(cold_refusal.value)


def test_an_entry_recorded_without_a_budget_is_a_miss_for_a_budget_and_gets_its_price():
    prepared = factories.field_domain("building_002_point_contact_v1")[0]
    q_values = _q_values(prepared)
    store: dict = {}
    exact.reset_factorization_memory()
    exact.prime_universe_remembered(q_values, None, store)
    (entry,) = store.values()
    assert entry[2] is None
    paid = exact.exact_work_budget(stage="COVERAGE")
    exact.reset_factorization_memory()
    exact.prime_universe_remembered(q_values, paid, store)
    assert paid.spent > 0 and next(iter(store.values()))[2] == paid.spent_by_article()


@pytest.mark.parametrize("name,build,laws,given", CASES, ids=[item[0] for item in CASES])
def test_the_template_carries_the_price_of_the_full_coverage_of_its_partition(name, build, laws, given, monkeypatch):
    prepared = build()
    evaluate_step(prepared, widths_of(prepared, given)[0], laws, monkeypatch)
    certificate = certificate_of(prepared, laws)
    assert certificate is not None
    templates = list(certificate.templates.values())
    assert templates and all(item.price is not None and len(item.price) == ARTICLES for item in templates)
    assert sum(sum(item.price) for item in templates) > 0, "a coverage is not free"


# ---------------------------------------------------------------- 6. запас под потолком


def test_the_field_corpus_spends_at_most_a_sixteenth_of_the_cap(monkeypatch):
    """Цена домена на полевом корпусе <= cap/16 на каждой стадии: потолок - защита от патологии, а не граница обычной работы."""

    cap = exact._EXACT_CANONICALIZATION_WORK_CAP
    worst = {"prepare": 0, "coverage": 0, "materialize": 0}
    for name, build, laws, given in CASES:
        prepared = build()
        worst["prepare"] = max(worst["prepare"], prepared.work_budget.spent)
        evaluated = evaluate_full(prepared, widths_of(prepared, given)[0], laws)
        assert evaluated.result.is_materialized, name
        worst["coverage"] = max(worst["coverage"], evaluated.coverage.work_budget.spent)
        worst["materialize"] = max(worst["materialize"], dict(evaluated.materialize_price)["EXACT_WORK_SPENT"])
    assert all(value * 16 <= cap for value in worst.values()), (worst, cap // 16)


# ---------------------------------------------------------------- 7. процесс: PYTHONHASHSEED не меняет ни ответ, ни цену


_SCRIPT = r"""
import hashlib, json, sys
sys.path.insert(0, sys.argv[1])
import test_price_without_history as t
import pickle
out = {}
for name, build, laws, given in t.CASES:
    if name not in sys.argv[2].split(","):
        continue
    blob = pickle.dumps(build())
    prepared = pickle.loads(blob)
    for width in t.widths_of(prepared, given, offsets=(0, 1)):
        e = t.evaluate_full(pickle.loads(blob), width, laws)
        out[name + ":" + width] = [
            list(e.coverage_price), [list(item) for item in e.materialize_price],
            hashlib.sha256(t.canonical_json_bytes(e.result.batch)).hexdigest() if e.result.batch is not None else e.result.detail,
        ]
print("RESULT " + json.dumps(out, sort_keys=True))
"""

def test_the_price_and_the_answer_are_the_same_under_different_hash_seeds():
    """Три процесса с разными `PYTHONHASHSEED` дают одну и ту же цену покрытия и материализации и один и тот же батч."""

    names = "noise_top,mesh2_fans,fold"
    runs = {}
    for seed in ("1", "7", "42"):
        env = dict(os.environ, PYTHONHASHSEED=seed, PYTHONSAFEPATH="1", PYTHONPATH=str(KERNEL_SOURCE))
        done = subprocess.run(
            [sys.executable, "-c", _SCRIPT, str(HERE), names], capture_output=True, text=True, env=env, timeout=540
        )
        assert done.returncode == 0, done.stderr[-2000:]
        line = next(item for item in done.stdout.splitlines() if item.startswith("RESULT "))
        runs[seed] = json.loads(line[len("RESULT "):])
    assert runs["1"], "the script evaluated nothing"
    assert runs["1"] == runs["7"] == runs["42"]


# ---------------------------------------------------------------- 8. два места, где ответ зависел от процесса


def test_the_clip_event_key_is_the_least_key_whatever_the_order_of_the_constructions():
    def construction(key):
        return ConstructionCertificate(ConstructionKind.EVENT_ANCHOR, event_key=key)

    keys = ("m", "b", "z", None, "a2", "a10")
    for rotation in range(len(keys)):
        shuffled = keys[rotation:] + keys[:rotation]
        segment = SimpleNamespace(start_constructions=frozenset(construction(key) for key in shuffled))
        assert policy_b._clip_event_key(segment) == "a10", "the least string, not the first element of a hash-ordered set"
    assert policy_b._clip_event_key(SimpleNamespace(start_constructions=frozenset())) is None
    assert policy_b._clip_event_key(SimpleNamespace(start_constructions=frozenset({construction(None)}))) is None


def test_the_grid_step_is_the_left_fold_of_the_face_sizes_on_every_interpreter(monkeypatch):
    from cftuv_envelope._cpython311 import left_fold_sum

    monkeypatch.setattr(interval_module.float_filter, "centre_and_bound", lambda coordinate: (float(coordinate), 0.0))
    faces = [SimpleNamespace(points=((0.0, 0.0), (1.1, 0.5))) for _ in range(10)]
    sizes = [1.1] * 10
    expected = left_fold_sum(sizes) / len(sizes)
    assert expected != sum(sizes) / len(sizes) or sys.version_info < (3, 12), "the case separates the two sums on 3.12 and later"
    assert interval_module._grid_step(faces) == expected
