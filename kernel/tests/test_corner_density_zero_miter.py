"""Митра при нулевой плотности (`CORNER_MITER_AT_DENSITY_ZERO_V1`, `SelectionLaw.CORNER_MITER_AT_DENSITY_ZERO_V1`).

Решение владельца 2026-10-17: на плотности веера 0 (`LINEAR_REFLEX_DENSITY_0_V1`) внешний угол декали вокруг вогнутого
угла источника не разбивается фаской (веер d0 — один луч, хорда срезает угол). Угол декали `alpha = pi - δ` не острее 89
градусов (`δ/π <= 91/180`, замкнуто) — митра со швом на биссектрисе (`k = 0`, `MITER_SEAM`); острее — фаска под именем
`DENSITY_ZERO_SHARP_CORNER_BEVEL`; интервал поверх предела — фаска под именем `DENSITY_ZERO_MITER_BOUND_UNPROVEN`.
Плотности 1..4 закон не видит.

Фикстура — `test_corner_join._snapshot`: две цепи `in` `(0,0)->(10,0)` и `out` `(10,0)->far` без общей записи хоста
(JOIN не берёт), поворот вправо; полоса `in` растёт вверх, полоса `out` — вправо от неё. Точка `far` выбрана так, что
угол поворота `δ` равен заявленному (числа — `math`, а не проверяемый код), интервалы — четыре знака вокруг `δ/π`.

| что проверяется                                                                        | тест |
|----------------------------------------------------------------------------------------|------|
| предел — 89 градусов как точная доля π; запись реестра им и объявлена                   | `..._bound_is_...` |
| прямой угол на d0 — митра, закон записи, сертификат `k = 0`, счётчики, проверки плана   | `..._a_right_angle_...` |
| таблица углов декали: тупой, прямой, 89.5, 89.05 — митра; 89 поверх интервала, 88.9, 73, 60 — фаска с именем | `..._decal_angle_table_...` |
| острый угол (60 градусов) — фаска d0 под именем, веер прежний                           | `..._a_sharp_decal_corner_...` |
| предел замкнут (верх на пределе — митра; низ на пределе закрытым концом — «не доказано») | `..._the_bound_is_closed_...` |
| плотности 1..4 — прежние записи и счёт на тех же углах                                  | `..._densities_one_to_four_...` |
| материализация: точка митры на `w / sin(alpha/2)`, шов, станции и диагностика назвали угол; изгиб шире четверти оборота строится | `..._the_miter_point_...`, `..._beyond_a_quarter_turn_...` |
| закон подъёма канонического счёта не судит k = 0 закона угла (тугой d0 на шуме вычисления) | `..._count_lift_law_...` |
| подделки: митра d0 под сертификатом d1, закон сертификата митры на изломе — отказ        | `..._forged_...` |
"""

from __future__ import annotations

import dataclasses
import math
from decimal import Decimal
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
from cftuv_envelope import _corner_treatment as law
from cftuv_envelope.contracts.envelopes import (
    ZERO_SUPPORT_SELECTION_LAWS,
    CornerTreatmentReasonV1,
    CornerTreatmentV1,
    SelectionLaw,
)
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.tolerance_policy import TolerancePolicyIdV1, tolerance_policy
from cftuv_envelope.materialize.stations import chain_station_table
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.reference.corner_treatment import corner_treatment_errors
from cftuv_envelope.validation_corner_treatment import (
    validate_plan_corner_treatments,
    validate_plan_corner_treatments_against_snapshot,
)

import materialize_factories as factories
from reference_factories import _interval
from test_corner_join import (
    _density_request,
    _materialized,
    _plan_like,
    _prepared,
    _record,
    _selection,
    _snapshot,
)

MITER = CornerTreatmentV1.MITER_SEAM
FAN = CornerTreatmentV1.ANGULAR_PROFILE
ZERO_MITER = CornerTreatmentReasonV1.DENSITY_ZERO_MITER
SHARP = CornerTreatmentReasonV1.DENSITY_ZERO_SHARP_CORNER_BEVEL
STRADDLE = CornerTreatmentReasonV1.DENSITY_ZERO_MITER_BOUND_UNPROVEN
UNPROVEN = CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN


def _far(turn_degrees: float, length: float = 10.0):
    """Конец цепи `out` для поворота вправо на `turn_degrees` от направления `+x` (вершина `(10, 0)`)."""

    radians = math.radians(turn_degrees)
    return (10.0 + length * math.cos(radians), -length * math.sin(radians))


def _bounds(turn_degrees: float):
    """Четыре знака вокруг `δ/π` (вниз и вверх) — интервал, который пишет хост."""

    value = Decimal(turn_degrees) / Decimal(180)
    lower = value.quantize(Decimal("0.0001"), rounding="ROUND_FLOOR")
    upper = value.quantize(Decimal("0.0001"), rounding="ROUND_CEILING")
    return str(lower), str(upper)


def _density_zero(request):
    return dataclasses.replace(
        _density_request(request, 1),
        max_subturn_value_id=kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
        max_subturn_exact_value=kernel.ExactAngleV1(kernel.ExactAngleSymbol.PI_OVER_2),
    )


def _case(turn_degrees, density=0, bounds=None):
    far = (10.0, -10.0) if turn_degrees == 90 else _far(turn_degrees)
    bounds = bounds or (("0.5000", "0.5000") if turn_degrees == 90 else _bounds(turn_degrees))
    snapshot, request = _snapshot(far, bounds, shared=False)
    request = _density_zero(request) if density == 0 else _density_request(request, density)
    return snapshot, request


def _plan_issues(prepared, snapshot):
    plan = _plan_like(prepared.compilation)
    issues: list = []
    validate_plan_corner_treatments(issues, plan)
    validate_plan_corner_treatments_against_snapshot(issues, plan, snapshot, ("plans", "p"))
    return issues


# --------------------------------------------------------------------------
# Предел и запись реестра
# --------------------------------------------------------------------------


def test_the_bound_is_eighty_nine_degrees_as_an_exact_fraction_of_pi_and_the_registry_declares_it():
    assert law.DENSITY_ZERO_MITER_BOUND_OVER_PI == Fraction(91, 180) == 1 - Fraction(89, 180)
    policy = tolerance_policy(TolerancePolicyIdV1.DENSITY_ZERO_MITER_BOUND_V1)
    assert Fraction(policy.value.numerator, policy.value.denominator) == law.DENSITY_ZERO_MITER_BOUND_OVER_PI
    assert policy.changes_topology
    assert policy.declaration_sites == ("cftuv_envelope._corner_treatment.DENSITY_ZERO_MITER_BOUND_OVER_PI",)
    assert SelectionLaw.CORNER_MITER_AT_DENSITY_ZERO_V1 in ZERO_SUPPORT_SELECTION_LAWS
    assert {law.law_of(reason) for reason in (ZERO_MITER, SHARP, STRADDLE)} == {law.DENSITY_ZERO_MITER_LAW}


# --------------------------------------------------------------------------
# Решение угла
# --------------------------------------------------------------------------


def test_a_right_angle_at_density_zero_miters():
    snapshot, request = _case(90)
    prepared = _prepared(snapshot, request)
    record, selection = _record(prepared), _selection(prepared)
    assert (record.treatment, record.reason, record.treatment_law) == (MITER, ZERO_MITER, law.DENSITY_ZERO_MITER_LAW)
    assert record.threshold_over_pi == kernel.ExactRatioV1(1, 2)
    assert selection.selection_law is SelectionLaw.CORNER_MITER_AT_DENSITY_ZERO_V1
    assert selection.resolved_hidden_edge_count == 0
    assert selection.max_subturn_value_id is kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_MITERED_CORNERS"] == 1 and counters["CONVEYOR_DENSITY_ZERO_MITERED_CORNERS"] == 1
    assert "CONVEYOR_FOLD_MITERED_CORNERS" not in counters
    assert counters.get("CONVEYOR_VERTEX_FANS", 0) == 0
    assert not corner_treatment_errors(prepared.compilation)
    assert not _plan_issues(prepared, snapshot)


@pytest.mark.parametrize(
    ("turn", "bounds", "treatment", "reason"),
    (
        (11.31, None, MITER, ZERO_MITER),  # угол декали 168.7
        (45.0, None, MITER, ZERO_MITER),  # 135
        (90, None, MITER, ZERO_MITER),  # прямой
        (90.5, None, MITER, ZERO_MITER),  # 89.5: шире четверти оборота, в пределе
        (90.95, ("0.5052", "0.5053"), MITER, ZERO_MITER),  # 89.05: верх 0.5053 < 91/180 = 0.50556
        (91.0, ("0.5055", "0.5056"), FAN, STRADDLE),  # ровно 89: интервал накрывает предел
        (91.1, ("0.5061", "0.5062"), FAN, SHARP),  # 88.9: низ 0.5061 > 91/180
        (106.7, None, FAN, SHARP),  # 73.3
        (120.0, None, FAN, SHARP),  # 60
    ),
    ids=("168.7", "135", "90", "89.5", "89.05", "89-straddle", "88.9", "73.3", "60"),
)
def test_the_decal_angle_table_at_density_zero(turn, bounds, treatment, reason):
    snapshot, request = _case(turn, bounds=bounds)
    prepared = _prepared(snapshot, request)
    record, selection = _record(prepared), _selection(prepared)
    assert (record.treatment, record.reason, record.treatment_law) == (treatment, reason, law.DENSITY_ZERO_MITER_LAW)
    if treatment is MITER:
        assert selection.selection_law is SelectionLaw.CORNER_MITER_AT_DENSITY_ZERO_V1
        assert selection.resolved_hidden_edge_count == 0
    else:
        assert selection.selection_law is SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1
        assert selection.resolved_hidden_edge_count == 1
    assert not corner_treatment_errors(prepared.compilation)
    assert not _plan_issues(prepared, snapshot)


def test_a_sharp_decal_corner_keeps_the_bevel_by_name():
    snapshot, request = _case(120.0)
    prepared = _prepared(snapshot, request)
    record, selection = _record(prepared), _selection(prepared)
    assert (record.treatment, record.reason) == (FAN, SHARP)
    assert law.law_of(record.reason) == law.DENSITY_ZERO_MITER_LAW
    # Фаска — прежний веер d0: один луч, счёт плотности.
    assert selection.selection_law is SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1
    assert selection.resolved_hidden_edge_count == 1
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_MITERED_CORNERS"] == 0 and "CONVEYOR_DENSITY_ZERO_MITERED_CORNERS" not in counters
    assert counters["CONVEYOR_DENSITY_ZERO_SHARP_BEVELS"] == 1 and "CONVEYOR_DENSITY_ZERO_UNPROVEN_BEVELS" not in counters
    straddle = dict(_prepared(*_case(91.0, bounds=("0.5055", "0.5056"))).counters)
    assert straddle["CONVEYOR_DENSITY_ZERO_UNPROVEN_BEVELS"] == 1 and "CONVEYOR_DENSITY_ZERO_SHARP_BEVELS" not in straddle
    # Станции домена называют фаску тем же числом (продукт видит счётчики материализации).
    _result, materialized = _materialized(prepared, request)
    assert materialized["STATION_DENSITY_ZERO_SHARP_BEVELS"] == 1
    assert "STATION_DENSITY_ZERO_MITER_CORNERS" not in materialized and "STATION_DENSITY_ZERO_UNPROVEN_BEVELS" not in materialized


def test_the_bound_is_closed_and_the_interval_alone_names_every_outcome():
    """`density_zero_decision`: верх на пределе (замкнуто) — митра; низ на пределе открытым концом — фаска острого угла."""

    def interval(lower, upper, lower_kind=IntervalEndpointKind.CLOSED, upper_kind=IntervalEndpointKind.CLOSED):
        return CertifiedDecimalIntervalV1(Decimal(lower), Decimal(upper), lower_kind, upper_kind, Decimal("0.0001"))

    def decide(value, bound=Fraction(1, 4)):
        # Предел подменён на 1/4: точная Decimal-граница (91/180 Decimal-ом не записывается).
        original = law.DENSITY_ZERO_MITER_BOUND_OVER_PI
        law.DENSITY_ZERO_MITER_BOUND_OVER_PI = bound
        try:
            fan = (FAN, UNPROVEN, frozenset())
            return law.density_zero_decision(fan, kernel.CertifiedReflexAngleMeasureV1(
                interval("1.2", "1.3"), value, kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION,
                kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
            ))[:2]
        finally:
            law.DENSITY_ZERO_MITER_BOUND_OVER_PI = original

    assert decide(interval("0.2400", "0.2500")) == (MITER, ZERO_MITER)
    assert decide(interval("0.2500", "0.2500")) == (MITER, ZERO_MITER)
    assert decide(interval("0.2500", "0.2600")) == (FAN, STRADDLE)
    assert decide(interval("0.2500", "0.2600", lower_kind=IntervalEndpointKind.OPEN)) == (FAN, SHARP)
    assert decide(interval("0.2400", "0.2600")) == (FAN, STRADDLE)
    assert decide(interval("0.2600", "0.2700")) == (FAN, SHARP)
    # Над JOIN и над митрой на изломе закон не работает: он переписывает только веер.
    join = (CornerTreatmentV1.JOIN_CONTINUATION, CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN, frozenset())
    fold = (MITER, CornerTreatmentReasonV1.FOLDED_NEIGHBOURHOOD_MITER, frozenset())
    measure = kernel.CertifiedReflexAngleMeasureV1(
        _interval("1.1", "1.2"), _interval("0.1000", "0.2000"),
        kernel.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION, kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
    )
    assert law.density_zero_decision(join, measure) == join
    assert law.density_zero_decision(fold, measure) == fold


@pytest.mark.parametrize("density", (1, 2, 3, 4))
@pytest.mark.parametrize("turn", (45.0, 90, 90.5, 106.7))
def test_densities_one_to_four_keep_their_records_and_counts(density, turn):
    snapshot, request = _case(turn, density=density)
    prepared = _prepared(snapshot, request)
    record, selection = _record(prepared), _selection(prepared)
    assert (record.treatment, record.reason, record.treatment_law) == (FAN, UNPROVEN, law.CORNER_TREATMENT_LAW)
    assert selection.selection_law is SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1
    assert selection.resolved_hidden_edge_count >= 1
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_MITERED_CORNERS"] == 0
    assert not {name for name in counters if name.startswith("CONVEYOR_DENSITY_ZERO_")}


# --------------------------------------------------------------------------
# Материализация
# --------------------------------------------------------------------------


def _points(result):
    return {(round(item.position.x, 9), round(item.position.y, 9)) for item in result.batch.vertices}


@pytest.mark.parametrize("decal_law", (DecalTopologyLawV1.TRIANGLES_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1))
def test_the_miter_point_fills_the_corner_and_the_seam_lies_on_the_bisector(decal_law):
    # Ширина 1, прямой угол: точка митры — `(10, 0) + (1, 1)` (расстояние `w / sin(45) = sqrt 2` по биссектрисе).
    snapshot, request = _case(90)
    prepared = _prepared(snapshot, request)
    result, counters = _materialized(prepared, request, decal_law)
    points = _points(result)
    assert (11.0, 1.0) in points
    # Фаски нет: концов лучей d0 на ширине 1 (`(10, 1)` и `(11, 0)` по нормалям и `(10 + 1/sqrt2, 1/sqrt2)` по лучу) в
    # контуре нет, кроме самих нормалей опор; конца биссекторного луча нет.
    assert (round(10 + 1 / math.sqrt(2), 9), round(1 / math.sqrt(2), 9)) not in points
    assert counters["STATION_DENSITY_ZERO_MITER_CORNERS"] == 1
    assert counters["STATION_FOLD_MITER_CORNERS"] == 0
    assert counters["STATION_FLOWS"] == 0 and counters["MATERIALIZE_REGIONS"] >= 2
    assert any(item.outcome is NamedOutcome.CORNER_MITER_AT_DENSITY_ZERO_V1 for item in result.batch.diagnostics)
    table = chain_station_table(prepared, factories.budget())
    assert table.density_zero_miters == ("v1",) and table.fold_miters == ()

    # Тот же угол на d1 (красный контроль) — веер: точки митры нет, станции углом митры его не называют.
    snapshot, request = _case(90, density=1)
    prepared = _prepared(snapshot, request)
    result, counters = _materialized(prepared, request, decal_law)
    assert (11.0, 1.0) not in _points(result)
    assert "STATION_DENSITY_ZERO_MITER_CORNERS" not in counters


def test_a_miter_beyond_a_quarter_turn_builds_where_the_join_ceiling_would_refuse(monkeypatch):
    """89.5 градуса декали: сектор шире `pi/2`, потолок JOIN (`q = 2`) его бы отверг; закон проверяет только ориентацию."""

    snapshot, request = _case(90.5)
    prepared = _prepared(snapshot, request)
    assert _selection(prepared).resolved_hidden_edge_count == 0
    _result, counters = _materialized(prepared, request)
    assert counters["STATION_DENSITY_ZERO_MITER_CORNERS"] == 1
    # Красный контроль: проверка опор с потолком JOIN `q = 2` отказывает доменом (прямой угол её ещё проходит).
    from cftuv_envelope.reference import angular
    from cftuv_envelope.reference.direction_binding import verify_huber_density_direction_bindings
    from cftuv_envelope.wavefront import prepare_conveyor

    monkeypatch.setattr(
        angular, "_verify_zero_support_turn",
        lambda metric, ideal, orientation: verify_huber_density_direction_bindings(metric, ideal, orientation, (), 2),
    )
    refused = prepare_conveyor(*_case(90.5))
    assert refused.outcome.value == "DOMAIN_GEOMETRY_REFUSED" and "BINDING_INSIDE_OWN_ORDINAL_WINDOW" in refused.detail
    assert prepare_conveyor(*_case(90)).outcome.value == "EXACT"


def test_the_canonical_count_lift_law_does_not_judge_a_corner_law_miter(monkeypatch):
    """Канонический прямой угол на d0 тугой (`u q = 1 = k + 1`); шум вычисления шире `pi/2` (поле: 13 доменов `building`,
    63 `cover.008`) делал его «счётом, который канон обязан был поднять». `k = 0` закона угла — не счёт плотности.

    Неосуществимость в геометрии вычисления навязана подменой; красный контроль — та же подмена без пропуска законов угла.
    """

    from cftuv_envelope.reference import angular, evaluation_binding_noise
    from cftuv_envelope.wavefront import prepare_conveyor

    monkeypatch.setattr(evaluation_binding_noise, "canonical_count_is_feasible", lambda *args, **kwargs: False)
    assert prepare_conveyor(*_case(90)).outcome.value == "EXACT"
    monkeypatch.setattr(angular, "ZERO_SUPPORT_SELECTION_LAWS", frozenset())
    refused = prepare_conveyor(*_case(90))
    assert refused.outcome.value == "DOMAIN_GEOMETRY_REFUSED"
    assert "canonical exact-limit count is not lifted" in refused.detail


# --------------------------------------------------------------------------
# Подделки
# --------------------------------------------------------------------------


def test_forged_density_zero_miters_are_refused_by_name():
    snapshot, request = _case(90)
    compilation = _prepared(snapshot, request).compilation
    (record,) = compilation.corner_treatments
    (selection,) = compilation.profile_selection_certificates

    def issues(**changes):
        found: list = []
        plan = _plan_like(compilation, **changes)
        validate_plan_corner_treatments(found, plan)
        validate_plan_corner_treatments_against_snapshot(found, plan, snapshot, ("plans", "p"))
        return [item.message for item in found]

    # Митра d0 под сертификатом d1: пересчёт по снапшоту на d1 даёт веер — запись и закон сертификата ложны.
    d1 = dataclasses.replace(
        selection,
        max_subturn_value_id=kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
    )
    messages = issues(certificates=frozenset({d1}))
    assert any("differs from the raw snapshot" in item for item in messages)
    assert any("contradicts the raw snapshot" in item for item in messages)
    # Митра d0 с законом сертификата митры на изломе: закон и обработка не согласны.
    fold_law = dataclasses.replace(selection, selection_law=SelectionLaw.CORNER_MITER_ON_FOLD_V1)
    assert any("selection law and treatment disagree" in item for item in issues(certificates=frozenset({fold_law})))
    # Причина митры d0 под законом записи JOIN.
    wrong_law = dataclasses.replace(record, treatment_law=law.CORNER_TREATMENT_LAW)
    assert any("treatment law is not the declared one" in item for item in issues(records=frozenset({wrong_law})))
    # Причина фаски d0 под обработкой митры.
    wrong_reason = dataclasses.replace(record, reason=SHARP)
    assert any("treatment reason does not belong" in item for item in issues(records=frozenset({wrong_reason})))
