"""DEVELOPABLE-ARAP-PROPOSAL: второе предложение карты после ИМЕНОВАННОГО отказа шарнира.

Шарнирное предложение кладёт треугольник за треугольником и сваливает весь угловой дефект
вершин в последние (верх стены с шумом по `z`: квадрат растяжения 9.5 при лучшей достижимой
карте около 1.4). ARAP (local/global, binary64, фиксированный порядок операций, 80 итераций)
размазывает ту же невязку по карте; суд над ним тот же — точный сертификат растяжения.

Здесь проверяется: закон назван вместе с числом итераций; ARAP не трогает домен, который
принимал шарнир (его байты прежние); отказ шарнира по растяжению, перевороту и самонакрытию
порождает ARAP, а отказ по искажению-в-бюджете (спираль) — нет; отказ несёт числа ОБОИХ
предложений; сертификат называет закон и отказ шарнира, валидатор пересчитывает карту и
ловит подмену; результат воспроизводим побитово (золотой дайджест — защита от расхождения
платформ и версий Python); решатель по огибающей сверен с независимым плотным решателем.
"""

from __future__ import annotations

import hashlib
import random
import re
from dataclasses import replace
from fractions import Fraction

import pytest

import cftuv_envelope._developable as developable
import cftuv_envelope._arap as arap
from cftuv_envelope._stretch import measure_stretch, stretch_violations
from cftuv_envelope._unfold import exact_metres, hinge_proposal, owner_topology
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.metric import (
    DEFAULT_DEVELOPABLE_STRETCH_BUDGET,
    CurvatureLadderPolicyV1,
    DevelopableProposalLawV1,
    ExactRationalV1,
)
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
)

import developable_factories as factories
import developable_noise_fixtures as noise
from developable_factories import DOMAIN, PATCH, REVISION, developable_chart
from developable_route import build_metric

ON = CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1
ARAP = DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1
HINGE = DevelopableProposalLawV1.BINARY64_HINGE_V1
BAND_PATTERN = re.compile(r"worst_band_squared<=([0-9.]+e[+-][0-9]+)")

#: Невязка складки сеткой, при которой шарнир ЗА бюджетом 20 % (27.7 %), а ARAP в нём (10.2 %). Прежние 0.05 при
#: бюджете 2 % делали то же самое; при 20 % шарнир (5.8 %) принимает сам, и ARAP до него не доходит.
ARAP_FIXTURE_DROP = 0.3
#: Невязка, при которой не вмещает ни шарнир (63 %), ни ARAP (24.4 %): красный контроль над 20 %.
BEYOND_BOTH_DROP = 0.8


def _wall_noise_top():
    return noise.noise_surface(noise.WALL_NOISE_TOP_PATCH_3, patch=PATCH)


def _rounded_wall_noise_top():
    return noise.noise_surface(noise.ROUNDED_WALL_NOISE_TOP_PATCH_2, patch=PATCH)


def _perturbed_fold_grid(drop):
    """Складка сеткой, у вершины `g1_2` недостаёт `drop` по `x`: невязка веера у ОДНОЙ вершины."""

    vertices, faces, _triangles = factories.fold_grid()
    points = {
        item.vertex_id.value[2:]: (item.position.x, item.position.y, item.position.z)
        for item in vertices
    }
    x, y, z = points["g1_2"]
    points["g1_2"] = (x - drop, y, z)
    return factories.surface(
        points, [[v.value[2:] for v in face.vertex_cycle] for face in faces]
    )


def _refusal(parts, **overrides) -> PlanarMetricAdmissionError:
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        build_metric(parts, ladder=ON, **overrides)
    return failure.value


def _bands(text: str) -> list[float]:
    return [float(item) for item in BAND_PATTERN.findall(text)]


def _issues(record, parts):
    vertices, faces, triangles = parts
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PATCH,
        expected_source_revision=REVISION,
        expected_patch_domain_id=DOMAIN,
        expected_source_lineage=frozenset(),
        surface_triangles=triangles,
    )


def _tampered(record, **changes):
    certificate = replace(record.metric.planarity_certificate, **changes)
    return replace(record, metric=replace(record.metric, planarity_certificate=certificate))


# --------------------------------------------------------------------------
# Закон назван вместе со своим числом итераций
# --------------------------------------------------------------------------


def test_the_law_name_carries_its_iteration_count():
    """Другое число итераций — другой закон, а значит другое имя: число в имени, и оно одно."""

    assert ARAP.value == f"ARAP_LOCAL_GLOBAL_{arap.ARAP_PROPOSAL_ITERATIONS}_BINARY64_V1"


def test_the_arap_triggers_are_exactly_the_named_hinge_refusals_before_the_snap():
    assert developable.ARAP_TRIGGER_OUTCOMES == {
        NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED,
        NamedOutcome.DEVELOPABLE_CHART_TRIANGLE_FLIPPED,
        NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP,
    }


# --------------------------------------------------------------------------
# Принятый шарниром домен ARAP не видит
# --------------------------------------------------------------------------


def test_a_domain_the_hinge_accepts_never_reaches_arap(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("the ARAP proposal ran for a hinge-accepted domain")

    monkeypatch.setattr(developable, "arap_proposal", forbidden)
    for parts in (factories.fold_strip(), factories.bevel_strip(4), factories.quarter_cylinder(8)):
        certificate = developable_chart(parts).certificate
        assert certificate.proposal_law is HINGE
        assert certificate.previous_refusals == ()


def test_an_overlap_of_an_in_budget_hinge_is_a_surface_property_and_not_given_to_arap(
    monkeypatch,
):
    """Спираль 400°: развёртка не искажена (растяжение 1), накрывает себя — ARAP из неё не выходит."""

    def forbidden(*_args, **_kwargs):
        raise AssertionError("ARAP was tried for an overlap without distortion")

    monkeypatch.setattr(developable, "arap_proposal", forbidden)
    error = _refusal(factories.spiral_strip())
    assert error.outcome is NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP
    assert "ARAP" not in str(error)
    assert "the hinge unfolding covers itself before any snapping" in str(error)


# --------------------------------------------------------------------------
# ARAP принимает то, что шарнир отдать не мог, и сертификат это называет
# --------------------------------------------------------------------------


def test_the_arap_proposal_accepts_a_vertex_the_hinge_could_not_fit():
    certificate = developable_chart(_perturbed_fold_grid(ARAP_FIXTURE_DROP)).certificate
    assert certificate.proposal_law is ARAP
    assert certificate.previous_refusals == (
        NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED.value,
    )
    assert not stretch_violations(certificate.stretch)
    assert certificate.chart_boundary_overlap_count == 0


def test_a_hinge_refusal_by_a_flipped_triangle_is_a_trigger_too():
    """Растяжение шарнира в бюджете, но переворот: ARAP пробуется и называет ЭТОТ отказ."""

    parts = _wall_noise_top()
    certificate = developable_chart(parts, budget=Fraction(5, 2)).certificate
    assert certificate.proposal_law is ARAP
    assert certificate.previous_refusals == (
        NamedOutcome.DEVELOPABLE_CHART_TRIANGLE_FLIPPED.value,
    )


def test_the_hinge_flip_is_named_before_the_budget_allows_the_hinge_chart():
    """Шарнир `wall_noise_top` перевёрнут (и растянут в 9.5): бюджет 2.5 снимает растяжение, но не переворот."""

    parts = _wall_noise_top()
    vertices, faces, triangles = parts
    snapped = {
        item.vertex_id: tuple(Fraction(a) for a in (item.position.x, item.position.y, item.position.z))
        for item in vertices
    }
    topology = owner_topology(triangles, snapped)
    chart = exact_metres(hinge_proposal(topology, snapped).coordinates)
    facts = measure_stretch(topology.triangles, snapped, chart, Fraction(5, 2))
    assert facts.certificate.triangles_outside_budget == 0
    assert facts.certificate.chart_flipped_triangle_count == 1


# --------------------------------------------------------------------------
# Данные владельца: верх стены с шумом
# --------------------------------------------------------------------------


def test_wall_noise_top_patch_three_is_accepted_by_arap_within_the_owners_twenty_percent():
    """Решение владельца «до 20 %»: ARAP-карта `wall_noise_top` (18.7 %) принята, шарнир назван в трассе."""

    certificate = developable_chart(_wall_noise_top()).certificate
    band = certificate.stretch.worst_band_squared_upper
    assert certificate.proposal_law is ARAP
    assert certificate.previous_refusals == (
        NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED.value,
    )
    assert 1.0 < band.numerator / band.denominator <= 1.41
    assert not stretch_violations(certificate.stretch)
    assert certificate.chart_boundary_overlap_count == 0


def test_wall_noise_top_patch_three_reports_the_arap_numbers_and_the_hinge_numbers():
    """Отказ несёт числа обоих предложений; на пороге ниже 18.74 % (здесь 18 %) `wall_noise_top` ещё отказан."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(_wall_noise_top(), budget=Fraction(18, 100))
    error = failure.value
    text = str(error)
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    arap_band, hinge_band = _bands(text)[:2]
    assert arap_band <= 1.41
    assert hinge_band > 9.0
    assert text.index("the ARAP proposal before snapping") < text.index(
        "the hinge proposal before snapping"
    )
    assert "ARAP_LOCAL_GLOBAL_80_BINARY64_V1 after the hinge proposal was refused" in text


def test_rounded_wall_noise_top_patch_two_is_refused_by_both_and_names_both():
    """Шарнир: самонакрытие и растяжение 42 (число берётся из измерения шарнира); ARAP: 1.71."""

    error = _refusal(_rounded_wall_noise_top())
    text = str(error)
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    arap_band, hinge_band = _bands(text)[:2]
    assert arap_band <= 1.71
    assert hinge_band > 40.0
    assert "the hinge unfolding covers itself before any snapping" in text
    assert NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP.value in text


@pytest.mark.parametrize(
    ("parts", "budget", "worst_below"),
    (
        (_wall_noise_top, Fraction(19, 100), 1.41),
        (_rounded_wall_noise_top, Fraction(31, 100), 1.71),
    ),
    ids=("wall-noise-top", "rounded-wall-noise-top"),
)
def test_the_noisy_domains_pass_once_the_budget_admits_their_arap_chart(
    parts, budget, worst_below
):
    """Бюджет — решение владельца; здесь только порог: при 19 % и 31 % карта ARAP принята."""

    chart = developable_chart(parts(), budget=budget)
    certificate = chart.certificate
    band = certificate.stretch.worst_band_squared_upper
    assert certificate.proposal_law is ARAP
    assert band.numerator / band.denominator <= worst_below
    assert not stretch_violations(certificate.stretch)
    assert certificate.chart_boundary_overlap_count == 0


@pytest.mark.parametrize(
    ("parts", "budget"),
    ((_wall_noise_top, Fraction(18, 100)), (_rounded_wall_noise_top, Fraction(30, 100))),
    ids=("wall-noise-top", "rounded-wall-noise-top"),
)
def test_just_below_those_budgets_the_domains_are_still_refused(parts, budget):
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        developable_chart(parts(), budget=budget)
    assert failure.value.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED


# --------------------------------------------------------------------------
# Отказ несёт оба предложения; нехватка ARAP называется
# --------------------------------------------------------------------------


def test_a_refusal_after_both_proposals_names_both_with_their_worst_numbers():
    """Красный контроль над 20 %: ни шарнир (63 %), ни ARAP (24.4 %) — оба числа в отказе."""

    error = _refusal(_perturbed_fold_grid(BEYOND_BOTH_DROP))
    text = str(error)
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    arap_band, hinge_band = _bands(text)[:2]
    assert float((1 + DEFAULT_DEVELOPABLE_STRETCH_BUDGET) ** 2) < arap_band < hinge_band
    assert "worst_vertex=v:g1_2" in text


def test_the_work_cap_is_a_named_refusal_and_not_a_silent_skip(monkeypatch):
    monkeypatch.setattr(arap, "ARAP_PROPOSAL_WORK_CAP", 10)
    error = _refusal(_perturbed_fold_grid(ARAP_FIXTURE_DROP))
    assert error.outcome is NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "the ARAP proposal was not tried" in str(error)
    assert "exceeds the cap 10" in str(error)


def test_a_system_that_is_not_positive_definite_is_named_unavailable():
    with pytest.raises(arap.ArapProposalUnavailable, match="not positive definite"):
        arap._factor([{0: 1.0}, {0: 2.0, 1: 1.0}], [0, 0])


# --------------------------------------------------------------------------
# Предложение само: пин, итерации, снижение искажения
# --------------------------------------------------------------------------


def _fold_grid_inputs(drop):
    vertices, _faces, triangles = _perturbed_fold_grid(drop)
    snapped = {
        item.vertex_id: tuple(Fraction(a) for a in (item.position.x, item.position.y, item.position.z))
        for item in vertices
    }
    topology = owner_topology(triangles, snapped)
    return topology, hinge_proposal(topology, snapped), snapped


def test_the_proposal_pins_one_vertex_and_runs_the_declared_iterations():
    topology, hinge, snapped = _fold_grid_inputs(ARAP_FIXTURE_DROP)
    proposal = arap.arap_proposal(topology, hinge, snapped)
    pinned = proposal.pinned_vertex_id
    assert proposal.iterations == arap.ARAP_PROPOSAL_ITERATIONS == 80
    assert proposal.coordinates[pinned] == hinge.coordinates[pinned]
    assert proposal.unknown_count == len(hinge.coordinates) - 1
    assert set(proposal.coordinates) == set(hinge.coordinates)


def test_the_proposal_lowers_the_worst_stretch_of_the_hinge_chart():
    topology, hinge, snapped = _fold_grid_inputs(ARAP_FIXTURE_DROP)
    budget = DEFAULT_DEVELOPABLE_STRETCH_BUDGET
    proposal = arap.arap_proposal(topology, hinge, snapped)
    worst = []
    for coordinates in (hinge.coordinates, proposal.coordinates):
        band = measure_stretch(
            topology.triangles, snapped, exact_metres(coordinates), budget
        ).certificate.worst_band_squared_upper
        worst.append(Fraction(band.numerator, band.denominator))
    assert worst[1] < worst[0]


def _random_envelope_system(size: int, seed: int):
    """Разреженная симметрично-положительная матрица: узкая лента плюс лапласиан путей."""

    rng = random.Random(seed)
    rows = [dict() for _ in range(size)]
    for index in range(size):
        rows[index][index] = 1.0 + rng.random()
    for index in range(1, size):
        for column in range(max(0, index - 4), index):
            if rng.random() < 0.5:
                weight = rng.random()
                rows[index][column] = -weight
                rows[index][index] += weight
                rows[column][column] += weight
    return rows


def test_the_envelope_solver_agrees_with_an_independent_dense_solution():
    """Холецкий по огибающей против гауссова исключения в точных дробях на той же матрице."""

    size = 40
    rows = _random_envelope_system(size, seed=7)
    first = arap._profile(rows)
    factor = arap._factor(rows, first)
    rng = random.Random(11)
    right = [rng.random() - 0.5 for _ in range(size)]
    got, _ = arap._solve(factor, first, list(right), list(right))
    dense = [[Fraction(0)] * size for _ in range(size)]
    for index, row in enumerate(rows):
        for column, value in row.items():
            dense[index][column] = Fraction(value)
            dense[column][index] = Fraction(value)
    augmented = [[*dense[index], Fraction(right[index])] for index in range(size)]
    for pivot in range(size):
        for other in range(pivot + 1, size):
            factor_ = augmented[other][pivot] / augmented[pivot][pivot]
            for column in range(pivot, size + 1):
                augmented[other][column] -= factor_ * augmented[pivot][column]
    exact = [Fraction(0)] * size
    for index in range(size - 1, -1, -1):
        tail = sum(
            (augmented[index][column] * exact[column] for column in range(index + 1, size)),
            Fraction(0),
        )
        exact[index] = (augmented[index][size] - tail) / augmented[index][index]
    assert max(abs(float(exact[index]) - got[index]) for index in range(size)) < 1e-9


# --------------------------------------------------------------------------
# Воспроизводимость побитово и золотой дайджест
# --------------------------------------------------------------------------

#: Золотой дайджест записи метрики ARAP-карты складки с невязкой `ARAP_FIXTURE_DROP` (весь сертификат и узлы).
#: Закон бинарной арифметики с фиксированным порядком операций: дайджест не зависит ни от
#: платформы, ни от версии Python (в `sum` над float CPython 3.12 суммирует иначе, чем 3.11).
#: Перезаписан 2026-10-03 решением владельца «до 20 %»: другая фикстура (шарнир теперь отказывает при невязке 0.3, а не
#: 0.05) и записанный бюджет `1/5`; сам ARAP (80 итераций, порядок операций) не менялся.
#: Перезаписан 2026-10-03 (STRETCH-BUDGET-POLICY + BEST-PROPOSAL): четыре новых поля сертификата; вычёркивание этих полей тогда возвращало
#: прежнее значение (`0f5562b4...`), сам ARAP не менялся.
#: Перезаписан 2026-10-08 (APPROVED_EXACT_TWO_GOLDEN_MIGRATION, SURFACE_CONE_ANGLE_DETERMINISTIC_SEED_V1): запись включает численно изменённую
#: оболочку `v:g1_2`; вычёркивание четырёх полей теперь даёт `f3e15ebe...` (`test_developable_best_proposal`), а не `0f5562b4...`.
ARAP_RECORD_SHA256 = (
    "7920e15673bc2ae84d42164e1f685c75e3e9a69132bc477b56cef2aeae552a5c"
)


def _digest(record) -> str:
    return hashlib.sha256(canonical_json_bytes(record)).hexdigest()


def test_the_arap_chart_is_reproducible_bit_for_bit():
    parts = _perturbed_fold_grid(ARAP_FIXTURE_DROP)
    first = developable_chart(parts)
    second = developable_chart(parts)
    assert first.nodes == second.nodes
    assert first.certificate == second.certificate
    assert _digest(first.certificate) == _digest(second.certificate)


def test_the_arap_metric_record_matches_its_golden_digest():
    record = build_metric(_perturbed_fold_grid(ARAP_FIXTURE_DROP), ladder=ON)
    assert record.metric.planarity_certificate.proposal_law is ARAP
    assert _digest(record) == ARAP_RECORD_SHA256


# --------------------------------------------------------------------------
# Валидатор: пересчёт и красные контроли
# --------------------------------------------------------------------------


def test_the_validator_accepts_what_the_builder_wrote_with_the_arap_proposal():
    parts = _perturbed_fold_grid(ARAP_FIXTURE_DROP)
    record = build_metric(parts, ladder=ON)
    certificate = record.metric.planarity_certificate
    assert certificate.proposal_law is ARAP
    assert certificate.previous_refusals == (
        "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED",
        NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED.value,
    )
    assert _issues(record, parts) == ()


def test_the_validator_catches_a_proposal_law_swapped_for_the_hinge():
    parts = _perturbed_fold_grid(ARAP_FIXTURE_DROP)
    record = build_metric(parts, ladder=ON)
    forged = _tampered(record, proposal_law=HINGE)
    messages = [item.message for item in _issues(forged, parts)]
    assert "developable certificate differs from exact recomputation" in messages


def test_the_validator_catches_an_arap_certificate_without_the_hinge_refusal():
    parts = _perturbed_fold_grid(ARAP_FIXTURE_DROP)
    record = build_metric(parts, ladder=ON)
    trace = record.metric.planarity_certificate.previous_refusals
    for forged_trace in (
        trace[:-1],
        (*trace[:-1], "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"),
        (*trace[:-1], NamedOutcome.DEVELOPABLE_SOURCE_TRIANGLE_DEGENERATE.value),
    ):
        forged = _tampered(record, previous_refusals=forged_trace)
        messages = [item.message for item in _issues(forged, parts)]
        assert any(
            "the second (ARAP) proposal is tried only after a named refusal" in message
            or "the ladder trace must name a ladder trigger" in message
            for message in messages
        ), forged_trace


def test_the_validator_catches_an_arap_claim_on_a_domain_the_hinge_accepts():
    parts = factories.bevel_strip(3)
    record = build_metric(parts, ladder=ON)
    certificate = record.metric.planarity_certificate
    assert certificate.proposal_law is HINGE
    forged = _tampered(
        record,
        proposal_law=ARAP,
        previous_refusals=(
            *certificate.previous_refusals,
            NamedOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED.value,
        ),
    )
    assert _issues(forged, parts)


def test_the_validator_catches_a_moved_node_of_the_arap_chart():
    parts = _perturbed_fold_grid(ARAP_FIXTURE_DROP)
    record = build_metric(parts, ladder=ON)
    coordinates = sorted(
        record.metric.exact_source_vertex_coordinates,
        key=lambda item: item.source_vertex_id.value,
    )
    moved = replace(
        coordinates[1],
        domain_coordinate=replace(
            coordinates[1].domain_coordinate,
            x=ExactRationalV1(coordinates[1].domain_coordinate.x.numerator + 1, 1),
        ),
    )
    forged = replace(
        record,
        metric=replace(
            record.metric,
            exact_source_vertex_coordinates=frozenset(
                [moved, *(item for item in coordinates if item is not coordinates[1])]
            ),
        ),
    )
    assert any(
        "unfolded chart coordinates differ" in item.message for item in _issues(forged, parts)
    )


def _cone_seed_witness():
    import json
    from pathlib import Path

    return json.loads((Path(__file__).parents[1] / "fixtures/cone_angle_legacy_libm_v1.json").read_text())


def _legacy_cone_seed(platform):
    """Независимый старый libm-контроль из наблюдавшихся hex, без вызова нового кандидата."""
    values = {row["argument_hex"]: row[platform] for row in _cone_seed_witness()["arguments"]}

    def seed(argument):
        assert argument.hex() in values, f"UNMEASURED_LEGACY_ACOS_ARGUMENT: {argument.hex()}"
        return float.fromhex(values[argument.hex()])

    return seed


def _cone_decisions(classes):
    from cftuv_envelope._fan_closure import angle_defect_lower_bound, worst_defect_vertex

    labels = tuple(sorted((item.vertex_id.value, item.developability_class.value,
                           item.closure_law.value, item.fan_triangle_count) for item in classes))
    ranking = tuple(item.vertex_id.value for item in sorted(
        classes, key=lambda item: (-angle_defect_lower_bound(item), item.vertex_id.value)
    ))
    return labels, ranking, worst_defect_vertex(classes)


@pytest.mark.parametrize("budget", (Fraction(1, 5), Fraction(1, 50)))
def test_deterministic_seed_preserves_both_legacy_arap_decision_paths(monkeypatch, budget):
    """Оба старых seed независимы от нового: классы, порядок дефекта, отказ и карта прежние."""
    from cftuv_envelope import surface_cone_angle as cone

    new_seed = cone._acos_seed
    classify = developable.classify_interior_vertices
    observed = []

    def spy(*args, **kwargs):
        result = classify(*args, **kwargs)
        observed.append(result)
        return result

    monkeypatch.setattr(developable, "classify_interior_vertices", spy)
    records, decisions, refusals = {}, {}, {}
    for name, seed in (("windows", _legacy_cone_seed("windows")),
                       ("linux", _legacy_cone_seed("linux")), ("deterministic", new_seed)):
        developable.clear_developable_chart_memory()
        observed.clear()
        monkeypatch.setattr(cone, "_acos_seed", seed)
        try:
            records[name] = build_metric(_perturbed_fold_grid(ARAP_FIXTURE_DROP), ladder=ON,
                                         developable_stretch_budget=budget)
        except PlanarMetricAdmissionError as error:
            refusals[name] = (error.outcome, str(error))
        assert observed
        decisions[name] = tuple(_cone_decisions(classes) for classes in observed)
    assert decisions["windows"] == decisions["linux"] == decisions["deterministic"]
    if refusals:
        assert len(refusals) == 3 and not records
        assert refusals["windows"] == refusals["linux"] == refusals["deterministic"]
        return
    assert len(records) == 3
    witness = _cone_seed_witness()
    assert _digest(records["windows"]) == witness["record_sha256"]["windows"]
    assert _digest(records["linux"]) == witness["record_sha256"]["linux"]
    assert records["deterministic"] == records["linux"]
    original = records["windows"]
    for name in ("linux", "deterministic"):
        changed = records[name]
        old_cert, new_cert = original.metric.planarity_certificate, changed.metric.planarity_certificate
        old_classes = {item.vertex_id: item for item in old_cert.vertex_classes}
        new_classes = {item.vertex_id: item for item in new_cert.vertex_classes}
        differing = [key for key in old_classes if old_classes[key] != new_classes[key]]
        assert [key.value for key in differing] == ["v:g1_2"]
        for key in differing:
            assert replace(new_classes[key], angle_sum_enclosure=old_classes[key].angle_sum_enclosure) == old_classes[key]
        assert replace(new_cert, vertex_classes=old_cert.vertex_classes) == old_cert
        assert replace(changed, metric=replace(changed.metric, planarity_certificate=old_cert)) == original


def test_deterministic_seed_preserves_both_legacy_arap_materializations(monkeypatch):
    """Полный меш с UV/ценой сравнивается; различие сертификата не скрывается новым golden."""
    from cftuv_envelope import surface_cone_angle as cone
    from cftuv_envelope.materialize.admit import MaterializationOutcome
    from developable_route import materialize_developable

    new_seed = cone._acos_seed
    results = []
    for seed in (_legacy_cone_seed("windows"), _legacy_cone_seed("linux"), new_seed):
        developable.clear_developable_chart_memory()
        monkeypatch.setattr(cone, "_acos_seed", seed)
        result, _prepared = materialize_developable(
            _perturbed_fold_grid(ARAP_FIXTURE_DROP), ("g0_0", "g1_0"), alpha="0.2"
        )
        assert result.outcome is MaterializationOutcome.MATERIALIZED
        results.append((canonical_json_bytes(result.batch), result.vertex_normals,
                        result.counters, result.diagnostics, result.offset_normals_digest))
    assert results[0] == results[1] == results[2]
