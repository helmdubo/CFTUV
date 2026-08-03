"""Закон ячеек Density B («середины») и два его доказанных блокера.

Закон A: ячейка счёта `H` — это `u in ((C-1)/q, C/q]`, `H = max(1, C-1)`;
границы стоят на кратных `1/q`, то есть на поворотах `k*pi/q`.
Закон B: ячейка счёта `H` — это `u in ((2H+1)/(2q), (2H+3)/(2q)]` с нижней
ячейкой `H=0` от нуля; границы стоят на НЕЧЁТНЫХ кратных `1/(2q)`, а СЕРЕДИНА
каждой ячейки даёт подповорот ровно `pi/q`.

`u = theta/pi` — та же величина `reflex_excess_over_pi`, которой ядро уже
измеряет угол; `theta` — поворот, `phi = pi + theta` — угол сектора.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from fractions import Fraction
import json
import math
from pathlib import Path

import mpmath as mp
import pytest
import sympy as sp

from cftuv_envelope import (
    AdmissibilityUpperBound,
    AngularEnvelopeSpec,
    AngularProfileSelectionPolicyId,
    AnalysisSnapshotCodecV1,
    CompiledPlanCodecV1,
    ContractCodecError,
    DecalRequestCodecV1,
    ExactAngleSymbol,
    ExactAngleV1,
    HuberDensityMidpointSelectionIntervalCertificateV1,
    IntervalBoundKind,
    IntervalEndpointKind,
    MaxSubturnParameterId,
    MaxSubturnValueId,
    MinimalityLowerBound,
    SelectionLaw,
    ValidationCode,
    canonical_json_bytes,
    validate_compiled_plan,
    validate_decal_request,
)
from cftuv_envelope._density_policy import (
    density_cell_lower_turn,
    density_cell_threshold_turn,
    huber_density_certificate_bounds,
    is_huber_density_policy,
    is_midpoint_density_policy,
)
from cftuv_envelope.reference import (
    ReferenceOutcome,
    compile_reference_envelopes,
)
from cftuv_envelope.reference.angular import (
    _TURN_COS_SQUARED,
    _compare_turn_cos_squared,
    _turn_at_most,
    density_count_is_feasible,
)
from cftuv_envelope.reference.compile import (
    _resolve_huber_density_bucket_interval,
    _resolve_midpoint_density_cell_interval,
)


_FIELD_SNAPSHOTS = (
    Path(__file__).resolve().parents[2] / "artifacts" / "field_snapshots"
)
_FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"

_DENSITIES = (
    (0, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1, ExactAngleSymbol.PI_OVER_2, 2),
    (1, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1, ExactAngleSymbol.PI_OVER_3, 3),
    (2, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1, ExactAngleSymbol.PI_OVER_4, 4),
    (3, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_3_V1, ExactAngleSymbol.PI_OVER_5, 5),
    (4, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1, ExactAngleSymbol.PI_OVER_6, 6),
)


# ---------------------------------------------------------------------------
# Точные величины интервалов селекции
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _ExactInterval:
    lower: Fraction
    upper: Fraction
    lower_kind: IntervalEndpointKind = IntervalEndpointKind.CLOSED
    upper_kind: IntervalEndpointKind = IntervalEndpointKind.CLOSED


def _point(value: Fraction) -> _ExactInterval:
    return _ExactInterval(value, value)


def _cell_a(u: Fraction, q: int) -> int | None:
    bucket = _resolve_huber_density_bucket_interval(_point(u), q)
    return None if bucket is None else max(1, bucket - 1)


def _cell_b(u: Fraction, q: int) -> int | None:
    return _resolve_midpoint_density_cell_interval(_point(u), q)


# ---------------------------------------------------------------------------
# Замкнутое перечисление cos²
# ---------------------------------------------------------------------------


def _constant_value(constant) -> sp.Expr:
    a, b, c, d = constant
    return (sp.Integer(a) + sp.Integer(b) * sp.sqrt(c)) / sp.Integer(d)


def test_sealed_cos_squared_table_is_exact_for_every_entry():
    for (numerator, denominator), constant in _TURN_COS_SQUARED.items():
        expected = sp.cos(sp.Rational(numerator, denominator) * sp.pi) ** 2
        assert sp.simplify(expected - _constant_value(constant)) == 0, (
            f"cos^2({numerator}pi/{denominator}) mismatch"
        )


def test_sealed_cos_squared_table_covers_every_threshold_of_both_laws():
    required = set()
    for q in range(2, 7):
        for hidden_count in range(0, q):
            for midpoint in (False, True):
                threshold = density_cell_threshold_turn(
                    hidden_count,
                    q,
                    midpoint=midpoint,
                )
                if threshold < 1:
                    required.add(
                        (threshold.numerator, threshold.denominator)
                    )
    assert required <= set(_TURN_COS_SQUARED)
    # Ровно те знаменатели, которые обещаны законами: `q` у A и `2q` у B.
    assert {denominator for _, denominator in required} == {
        2,
        3,
        4,
        5,
        6,
        8,
        10,
        12,
    }


@pytest.mark.parametrize("entry", sorted(_TURN_COS_SQUARED))
def test_rational_comparison_with_quadratic_constant_is_exact(entry):
    """Знак сравнения выводится из ЗНАКА радикала, а не из приближения.

    Пробы строятся подстановкой рациональных обхватов `sqrt(c)`: если
    `r < sqrt(c)`, то `(a + b*r)/d - (a + b*sqrt(c))/d = b*(r - sqrt(c))/d`
    имеет знак `-sign(b)` ТОЧНО, без единого приближённого сравнения.
    """

    threshold = Fraction(*entry)
    a, b, c, d = _TURN_COS_SQUARED[entry]
    if b == 0:
        exactly_on = Fraction(a, d)
        assert _compare_turn_cos_squared(exactly_on, threshold) == 0
        assert _compare_turn_cos_squared(
            exactly_on - Fraction(1, 10**20), threshold
        ) == -1
        assert _compare_turn_cos_squared(
            exactly_on + Fraction(1, 10**20), threshold
        ) == 1
        return
    root = Fraction(math.isqrt(c * 10**40), 10**20)
    assert root * root < c
    below = Fraction(a + b * root, d)
    above = Fraction(a + b * (root + Fraction(1, 10**20)), d)
    assert (root + Fraction(1, 10**20)) ** 2 > c
    assert _compare_turn_cos_squared(below, threshold) == -_sign_of(b)
    assert _compare_turn_cos_squared(above, threshold) == _sign_of(b)


def _sign_of(value: int) -> int:
    return (value > 0) - (value < 0)


def test_threshold_outside_the_sealed_set_is_a_named_refusal():
    with pytest.raises(ValueError, match="sealed cos-squared set"):
        _compare_turn_cos_squared(Fraction(1, 3), Fraction(1, 7))


def test_turn_at_most_agrees_with_high_precision_arccos():
    mp.mp.prec = 300
    for entry in sorted(_TURN_COS_SQUARED):
        threshold = Fraction(*entry)
        for numerator in range(1, 400):
            theta = mp.pi * numerator / 400
            cosine = mp.cos(theta)
            sign = 1 if cosine > 0 else -1 if cosine < 0 else 0
            squared = Fraction(
                int(mp.floor(cosine * cosine * 10**30)), 10**30
            )
            # Отступ наружу, чтобы огрубление не решало за предикат.
            if abs(float(theta / mp.pi) - float(threshold)) < 1e-6:
                continue
            assert _turn_at_most(sign, squared, threshold) == (
                theta <= mp.pi * mp.mpf(threshold.numerator)
                / mp.mpf(threshold.denominator)
            )


# ---------------------------------------------------------------------------
# (а) Полевой прямой угол и его возмущённые близнецы
# ---------------------------------------------------------------------------


def _wall_corner_turns() -> dict[int, Fraction]:
    """Точный `u = theta/pi` каждого угла стены `2.001` из слепка владельца.

    Координаты слепка — двоичные float, то есть ТОЧНЫЕ двоичные рациональные;
    `dot` и `cross` пары рёбер поэтому точные дроби, и воспроизведение
    сверяется с `dot_exact`, записанным в полевом артефакте.
    """

    angles = json.loads(
        (_FIELD_SNAPSHOTS / "wall_2_001_corner_angles.json").read_text()
    )
    raw = json.loads(
        (_FIELD_SNAPSHOTS / "wall_2_001_snapshot.json").read_text()
    )["raw"]
    vertices = raw["vertices"]
    loop = angles["loop_order"]
    recorded = {item["vertex"]: item for item in angles["corners"]}

    def plane(index: int) -> tuple[Fraction, Fraction]:
        # Плоскость стены — `x = const`, поэтому карта это `(y, z)`.
        point = vertices[index]
        return Fraction(point[1]), Fraction(point[2])

    mp.mp.prec = 400
    turns: dict[int, Fraction] = {}
    for position, vertex in enumerate(loop):
        if vertex not in recorded:
            continue
        previous = loop[(position - 1) % len(loop)]
        following = loop[(position + 1) % len(loop)]
        px, py = plane(previous)
        ax, ay = plane(vertex)
        nx, ny = plane(following)
        ux, uy = ax - px, ay - py
        wx, wy = nx - ax, ny - ay
        dot = ux * wx + uy * wy
        cross = ux * wy - uy * wx
        assert str(dot) == recorded[vertex]["dot_exact"], vertex
        theta = mp.atan2(abs(mp.mpf(cross.numerator)) / mp.mpf(cross.denominator)
                         if cross.denominator != 1 else abs(mp.mpf(int(cross))),
                         mp.mpf(dot.numerator) / mp.mpf(dot.denominator))
        turns[vertex] = Fraction(
            mp.nstr(theta / mp.pi, 60)
        ).limit_denominator(10**40)
    return turns


# Полевые близнецы: v9 смоделирован ТОЧНО (`dot == 0`), v6 и v28 уведены выше
# 90 градусов на 8.4e-05 и 6.3e-05, v27 — ниже на 1.6e-04. Все четыре — «один
# и тот же» прямой угол для глаза владельца.
_FIELD_TWINS = (9, 6, 28, 27)

# Что даёт каждый закон на этой четвёрке. Таблица — не пожелание, а измерение.
#
# ЗАКОН A расщепляет четвёрку при ЧЁТНОМ `q` (плотности 2 и 4): граница
# `u = 1/2` стоит ровно на прямом угле, точный угол уходит в CLOSED-сторону,
# шумный — в верхнюю ячейку.
# ЗАКОН B расщепляет её при НЕЧЁТНОМ `q` (плотности 1 и 3) — ровно тот же
# дефект, переехавший на другие плотности. Плотность 1 — ДЕФОЛТ панели.
_TWIN_COUNTS = {
    #  q: (закон A по вершинам 9/6/28/27, закон B по ним же)
    2: ((1, 1, 1, 1), (0, 0, 0, 0)),
    3: ((1, 1, 1, 1), (0, 1, 1, 0)),
    4: ((1, 2, 2, 1), (1, 1, 1, 1)),
    5: ((2, 2, 2, 2), (1, 2, 2, 1)),
    6: ((2, 3, 3, 2), (2, 2, 2, 2)),
}


@pytest.mark.parametrize(("density", "value_id", "symbol", "q"), _DENSITIES)
def test_field_right_angle_twins_split_under_a_on_even_q_and_under_b_on_odd_q(
    density,
    value_id,
    symbol,
    q,
):
    del density, value_id, symbol
    turns = _wall_corner_turns()
    expected_a, expected_b = _TWIN_COUNTS[q]
    actual_a = tuple(_cell_a(turns[vertex], q) for vertex in _FIELD_TWINS)
    actual_b = tuple(_cell_b(turns[vertex], q) for vertex in _FIELD_TWINS)
    assert actual_a == expected_a
    assert actual_b == expected_b


def test_exactly_one_law_keeps_the_field_twins_together_at_each_density():
    """Ни один из двух законов не закрывает все пять плотностей.

    Это исполняемая форма блокера №1 карточки FAN-CELL-BOUNDARY-MIDPOINT:
    её ворота (а) — «одинаковый счёт на всех пяти плотностях» — недостижимы
    ни законом A, ни законом B. A держит нечётные `q`, B — чётные.
    """

    stable_a = {
        q for q, (a, _) in _TWIN_COUNTS.items() if len(set(a)) == 1
    }
    stable_b = {
        q for q, (_, b) in _TWIN_COUNTS.items() if len(set(b)) == 1
    }
    assert stable_a == {2, 3, 5}
    assert stable_b == {2, 4, 6}
    assert stable_a | stable_b == {2, 3, 4, 5, 6}
    assert stable_a != {2, 3, 4, 5, 6}
    assert stable_b != {2, 3, 4, 5, 6}


def test_field_reproduction_matches_the_recorded_source_law_counts():
    """Воспроизведение углов сверено с числами полевого артефакта."""

    angles = json.loads(
        (_FIELD_SNAPSHOTS / "wall_2_001_corner_angles.json").read_text()
    )
    turns = _wall_corner_turns()
    for corner in angles["corners"]:
        vertex = corner["vertex"]
        assert _cell_a(turns[vertex], 4) == corner["d2_hidden_count_source_law"]
        assert _cell_a(turns[vertex], 3) == corner["d1_hidden_count_source_law"]


# ---------------------------------------------------------------------------
# (б) Где счёт НЕ изменился, и (в) поведение на новой границе
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("q", range(2, 7))
def test_upper_half_of_every_old_cell_keeps_its_count(q):
    """Закон B отличается от A РОВНО на нижних полуполосах старых ячеек.

    Контрольные точки: середина верхней половины каждой старой ячейки,
    `u = (4C-1)/(4q)`. Там счёт обязан совпасть с законом A (с точностью до
    его собственного зажима `H >= 1`, которого у B нет).
    """

    for bucket_c in range(1, q + 1):
        upper_half = Fraction(4 * bucket_c - 1, 4 * q)
        if upper_half >= 1:
            continue
        assert _cell_b(upper_half, q) == max(0, bucket_c - 1)
        assert _cell_a(upper_half, q) == max(1, bucket_c - 1)


@pytest.mark.parametrize("q", range(2, 7))
def test_lower_half_of_every_old_cell_drops_the_count_by_one(q):
    """Обратная половина той же теоремы — единственный источник переездов."""

    for bucket_c in range(1, q + 1):
        lower_half = Fraction(4 * bucket_c - 3, 4 * q)
        if lower_half >= 1:
            continue
        assert _cell_b(lower_half, q) == max(0, bucket_c - 2)


@pytest.mark.parametrize("q", range(2, 7))
def test_new_cell_boundary_is_lower_open_upper_closed(q):
    """Граница `(2H+3)/(2q)` принадлежит НИЖНЕЙ ячейке; сторона заморожена."""

    epsilon = Fraction(1, 1000 * q)
    for hidden_count in range(0, q - 1):
        boundary = density_cell_threshold_turn(
            hidden_count,
            q,
            midpoint=True,
        )
        if boundary >= 1:
            continue
        assert _cell_b(boundary - epsilon, q) == hidden_count
        assert _cell_b(boundary, q) == hidden_count
        assert _cell_b(boundary + epsilon, q) == hidden_count + 1
        straddle = _ExactInterval(boundary - epsilon, boundary + epsilon)
        assert _resolve_midpoint_density_cell_interval(straddle, q) is None


@pytest.mark.parametrize("q", range(2, 7))
def test_every_exact_multiple_of_pi_over_q_is_strictly_inside_its_cell(q):
    """Свойство-цель закона B: `k*pi/q` — СЕРЕДИНА ячейки, не её граница.

    Возмущение до полуполосы `pi/(2q)` счёт не меняет, и это проверяется не
    рассуждением, а обоими краями полуполосы.
    """

    for k in range(1, q):
        centre = Fraction(k, q)
        cell = _cell_b(centre, q)
        assert cell == k - 1
        half_band = Fraction(1, 2 * q)
        for offset in (
            -half_band + Fraction(1, 10**9),
            Fraction(0),
            half_band - Fraction(1, 10**9),
        ):
            probe = centre + offset
            if not 0 < probe < 1:
                continue
            assert _cell_b(probe, q) == cell
        # Ровно на середине подповорот равен `pi/q` — это и есть смысл закона.
        assert Fraction(centre, cell + 1) == Fraction(1, q)


@pytest.mark.parametrize("q", range(2, 7))
def test_cell_bounds_are_a_partition_without_gaps_or_overlaps(q):
    for hidden_count in range(0, q):
        lower = density_cell_lower_turn(hidden_count, q, midpoint=True)
        upper = density_cell_threshold_turn(hidden_count, q, midpoint=True)
        assert lower < upper
        if hidden_count == 0:
            assert lower == 0
        else:
            assert lower == density_cell_threshold_turn(
                hidden_count - 1,
                q,
                midpoint=True,
            )
    assert density_cell_threshold_turn(q - 1, q, midpoint=True) > 1


# ---------------------------------------------------------------------------
# Контракт сертификата ячейки B
# ---------------------------------------------------------------------------


def _density_request(request, policy, parameter, value_id, symbol):
    return replace(
        request,
        angular_profile_selection_policy_id=policy,
        max_subturn_parameter_id=parameter,
        max_subturn_value_id=value_id,
        max_subturn_exact_value=ExactAngleV1(symbol),
    )


def _midpoint_request(request, value_id, symbol):
    return _density_request(
        request,
        AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1,
        MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_B_V1,
        value_id,
        symbol,
    )


@pytest.mark.parametrize(("density", "value_id", "symbol", "q"), _DENSITIES)
def test_midpoint_request_tuple_is_additive_and_strict(
    projections,
    density,
    value_id,
    symbol,
    q,
):
    del density, q
    legacy = next(
        item for item in projections if item.case_id == "EC0-C03"
    ).request
    request = _midpoint_request(legacy, value_id, symbol)
    assert validate_decal_request(request) == ()
    payload = DecalRequestCodecV1.dumps(request)
    assert (
        DecalRequestCodecV1.dumps(DecalRequestCodecV1.loads(payload))
        == payload
    )


def test_midpoint_request_rejects_the_density_a_parameter_id(projections):
    legacy = next(
        item for item in projections if item.case_id == "EC0-C03"
    ).request
    forged = _density_request(
        legacy,
        AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1,
        MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1,
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
        ExactAngleSymbol.PI_OVER_4,
    )
    issues = validate_decal_request(forged)
    assert any(
        issue.code is ValidationCode.POLICY_MISMATCH
        and issue.path == ("max_subturn_parameter_id",)
        for issue in issues
    )


def _midpoint_certificate(certificate, *, value_id, q, cell):
    return replace(
        certificate,
        selection_policy_id=(
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1
        ),
        max_subturn_value_id=value_id,
        resolved_hidden_edge_count=cell,
        resolved_subturn_count=cell + 1,
        local_profile_support_count=cell + 2,
        local_profile_segment_count=cell + 2,
        selection_law=SelectionLaw.HUBER_EMANATED_DENSITY_MIDPOINT_V1,
        minimality_lower_bound=(
            MinimalityLowerBound.HUBER_DENSITY_MIDPOINT_CELL_OPEN_LOWER
        ),
        admissibility_upper_bound=(
            AdmissibilityUpperBound.HUBER_DENSITY_MIDPOINT_CELL_CLOSED_UPPER
        ),
        selection_interval_certificate=(
            HuberDensityMidpointSelectionIntervalCertificateV1(
                q=q,
                cell_hidden_edge_count=cell,
                lower_bound_kind=IntervalBoundKind.OPEN,
                lower_bound_numerator=0 if cell == 0 else 2 * cell + 1,
                upper_bound_kind=IntervalBoundKind.CLOSED,
                upper_bound_numerator=2 * cell + 3,
            )
        ),
        regression_fixture_id=None,
    )


def test_midpoint_interval_tag_roundtrips_and_validator_seals_the_law(
    projections,
):
    plan = next(
        item for item in projections if item.case_id == "EC0-C03"
    ).plans[0]
    legacy = next(iter(plan.angular_profile_selection_certificates))
    density = _midpoint_certificate(
        legacy,
        value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        q=3,
        cell=1,
    )
    changed = replace(
        plan,
        angular_profile_selection_certificates=frozenset({density}),
    )
    payload = CompiledPlanCodecV1.dumps(changed)
    data = json.loads(payload)
    encoded = data["angular_profile_selection_certificates"][0][
        "selection_interval_certificate"
    ]
    assert encoded["$type"] == (
        "HuberDensityMidpointSelectionIntervalCertificateV1"
    )
    assert (
        CompiledPlanCodecV1.dumps(CompiledPlanCodecV1.loads(payload))
        == payload
    )
    assert huber_density_certificate_bounds(
        density.selection_interval_certificate
    ) == (Fraction(3, 6), Fraction(5, 6))


def test_midpoint_interval_forgery_is_rejected_by_the_validator(projections):
    plan = next(
        item for item in projections if item.case_id == "EC0-C03"
    ).plans[0]
    legacy = next(iter(plan.angular_profile_selection_certificates))
    density = _midpoint_certificate(
        legacy,
        value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        q=3,
        cell=1,
    )
    for forged_interval in (
        replace(
            density.selection_interval_certificate,
            lower_bound_kind=IntervalBoundKind.CLOSED,
        ),
        replace(
            density.selection_interval_certificate,
            upper_bound_numerator=4,
        ),
        replace(density.selection_interval_certificate, q=6),
    ):
        changed = replace(
            plan,
            angular_profile_selection_certificates=frozenset(
                {
                    replace(
                        density,
                        selection_interval_certificate=forged_interval,
                    )
                }
            ),
        )
        assert any(
            issue.code is ValidationCode.ANGULAR_CERTIFICATE
            and "Density B certificate" in issue.message
            for issue in validate_compiled_plan(changed)
        )


def test_midpoint_certificate_rejects_the_density_a_interval_tag(projections):
    plan = next(
        item for item in projections if item.case_id == "EC0-C03"
    ).plans[0]
    legacy = next(iter(plan.angular_profile_selection_certificates))
    density = _midpoint_certificate(
        legacy,
        value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        q=3,
        cell=1,
    )
    payload = CompiledPlanCodecV1.dumps(
        replace(
            plan,
            angular_profile_selection_certificates=frozenset({density}),
        )
    )
    data = json.loads(payload)
    data["angular_profile_selection_certificates"][0][
        "selection_interval_certificate"
    ]["$type"] = "HuberDensitySelectionIntervalCertificateV1"
    with pytest.raises(ContractCodecError, match="field mismatch"):
        CompiledPlanCodecV1.loads(json.dumps(data))


def test_policy_predicates_name_exactly_two_density_laws():
    assert is_huber_density_policy(
        AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
    )
    assert is_huber_density_policy(
        AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1
    )
    assert not is_huber_density_policy(
        AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1
    )
    assert not is_midpoint_density_policy(
        AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
    )
    assert is_midpoint_density_policy(
        AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1
    )


# ---------------------------------------------------------------------------
# ОДИН закон в двух местах: лифт в метрике карты
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("q", range(2, 7))
def test_lift_feasibility_uses_the_same_cell_bounds_as_selection(q):
    """`density_count_is_feasible` и селекция обязаны решать одну ячейку.

    Слева — закон по сертифицированному интервалу `u` источника, справа — тот
    же закон по точному signed-cos² в метрике карты. Второго закона нет.
    """

    mp.mp.prec = 300
    # `u = odd/480` не совпадает НИ С ОДНОЙ границей обоих законов: её
    # знаменатель делится на 32, а границы имеют знаменатель `q <= 6` или
    # `2q <= 12`. Поэтому огрубление cos² не решает за предикат.
    for numerator in range(1, 480, 2):
        u = Fraction(numerator, 480)
        theta = mp.pi * numerator / 480
        cosine = mp.cos(theta)
        sign = 1 if cosine > 0 else -1 if cosine < 0 else 0
        squared = Fraction(
            int(mp.floor(cosine * cosine * 10**30)), 10**30
        )
        for midpoint in (False, True):
            selected = (
                _cell_b(u, q) if midpoint else _cell_a(u, q)
            )
            if selected is None:
                continue
            feasible = [
                hidden_count
                for hidden_count in range(0, q)
                if density_count_is_feasible(
                    sign,
                    squared,
                    hidden_count,
                    q,
                    midpoint=midpoint,
                )
            ]
            assert feasible, (q, u, midpoint)
            # Осуществимость монотонна по счёту, а её минимум равен ячейке
            # селекции — с точностью до зажима `H >= 1` закона A.
            assert feasible == list(range(min(feasible), q))
            assert max(min(feasible), 1 if not midpoint else 0) == selected


def test_field_lift_carries_the_midpoint_thresholds_and_is_deterministic():
    """(г) Селекция источника на границе — решает МЕТРИКА КАРТЫ, с записью.

    На `building_002` при `q = 3` закон B кладёт все четыре угла в нижнюю
    ячейку (`H = 0`), а метрика карты одного из них поднимает счёт до 1 с
    сертификатом лифта. Пороги в сертификате — B-шные (`3/(2q) = 1/2` и
    `5/(2q) = 5/6`), а не A-шные (`1/q = 1/3` и `2/q = 2/3`): по закону A
    источник получил бы `H = 1` уже при селекции и подниматься было бы нечему.
    """

    from cftuv_envelope.reference.angular import _lift_count_is_feasible

    snapshot, legacy = _building_002()
    compiled = compile_reference_envelopes(
        snapshot,
        _midpoint_request(
            legacy,
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
            ExactAngleSymbol.PI_OVER_3,
        ),
    )
    assert compiled.outcome is ReferenceOutcome.EXACT
    lifted = [
        item
        for item in compiled.compilation.envelope_specs
        if getattr(item, "evaluation_subturn_count_lift", None) is not None
    ]
    assert len(lifted) == 1
    lift = lifted[0].evaluation_subturn_count_lift
    assert (
        lift.source_hidden_edge_count,
        lift.effective_hidden_edge_count,
        lift.max_subturn_q,
        lift.minimality_predecessor_hidden_edge_count,
    ) == (0, 1, 3, 0)
    assert _lift_count_is_feasible(lift, 1, midpoint=True)
    assert not _lift_count_is_feasible(lift, 0, midpoint=True)
    # Тот же свидетель, названные пороги: поворот карты вышел за `1/2` (верх
    # нижней B-ячейки) и не вышел за `5/6` (верх следующей).
    witness_sign = -1
    witness = Fraction(
        lift.evaluation_turn_cosine_squared.numerator,
        lift.evaluation_turn_cosine_squared.denominator,
    )
    assert not _turn_at_most(witness_sign, witness, Fraction(1, 2))
    assert _turn_at_most(witness_sign, witness, Fraction(5, 6))
    # Закон A на том же `q` уже при СЕЛЕКЦИИ даёт `H = 1` — лифту неоткуда
    # взяться, и его порог `2/q` этот же поворот накрывает.
    assert _lift_count_is_feasible(lift, 1, midpoint=False)

    again = compile_reference_envelopes(
        snapshot,
        _midpoint_request(
            legacy,
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
            ExactAngleSymbol.PI_OVER_3,
        ),
    )
    assert canonical_json_bytes(
        compiled.compilation.envelope_specs
    ) == canonical_json_bytes(again.compilation.envelope_specs)


def test_lift_certificate_thresholds_move_with_the_law():
    """Тот же поворот, тот же `q` — разный минимальный осуществимый счёт."""

    mp.mp.prec = 300
    # Поворот 0.62*pi: верхняя полуполоса старой ячейки `q=4`.
    theta = mp.pi * mp.mpf("0.62")
    cosine = mp.cos(theta)
    squared = Fraction(int(mp.floor(cosine * cosine * 10**30)), 10**30)
    assert not density_count_is_feasible(-1, squared, 1, 4, midpoint=False)
    assert density_count_is_feasible(-1, squared, 2, 4, midpoint=False)
    assert density_count_is_feasible(-1, squared, 1, 4, midpoint=True)


# ---------------------------------------------------------------------------
# БЛОКЕР 2: привязка направлений всё ещё доказывает подповорот <= pi/q
# ---------------------------------------------------------------------------


def _building_002():
    fixture = _FIXTURES / "building_002_full_selection_v1"
    return (
        AnalysisSnapshotCodecV1.loads(
            (fixture / "analysis_snapshot.json").read_bytes()
        ),
        DecalRequestCodecV1.loads(
            (fixture / "decal_request.json").read_bytes()
        ),
    )


# Наблюдаемый исход закона B на полевой стене `building_002` (четыре прямых
# угла, из них один уведён на 1.2e-04). Это ЗАМОРОЗКА факта, а не пожелание:
# у B подповорот законно доходит до `3*pi/(2q)`, а привязка направлений веера
# (`BINDING_SUBTURN_LE_DELTA_MAX` / `ADAPTIVE_FAN_SUBTURN`) доказывает
# `<= pi/q`. Как только эта граница будет отпущена до полуполосы, тест обязан
# быть переписан ВМЕСТЕ с законом, а не подкручен.
_BUILDING_002_MIDPOINT_OUTCOME = {
    2: (ReferenceOutcome.EXACT, (0, 0, 0, 0)),
    3: (ReferenceOutcome.EXACT, (0, 0, 1, 1)),
    4: (ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, None),
    5: (ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, None),
    6: (ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, None),
}


@pytest.mark.parametrize(("density", "value_id", "symbol", "q"), _DENSITIES)
def test_midpoint_law_on_building_002_is_frozen_with_its_binding_refusals(
    density,
    value_id,
    symbol,
    q,
):
    del density
    snapshot, legacy = _building_002()
    compiled = compile_reference_envelopes(
        snapshot,
        _midpoint_request(legacy, value_id, symbol),
    )
    expected_outcome, expected_counts = _BUILDING_002_MIDPOINT_OUTCOME[q]
    assert compiled.outcome is expected_outcome
    if expected_counts is None:
        assert compiled.compilation is None
        assert "BINDING_MONOTONE" in compiled.diagnostics[0].message
        return
    counts = tuple(
        sorted(
            item.resolved_hidden_edge_count
            for item in compiled.compilation.envelope_specs
            if isinstance(item, AngularEnvelopeSpec)
        )
    )
    assert counts == expected_counts


def test_zero_hidden_count_is_legal_only_under_the_midpoint_law():
    """`H = 0` — законный член семейства у B и подделка у A.

    У закона A счёт зажат снизу единицей (`max(1, C-1)`), поэтому нулевой
    веер при явной плотности мог быть только подделкой; у B нижняя ячейка
    даёт `H = 0` честно, и угол остаётся митрованным.
    """

    snapshot, legacy = _building_002()
    midpoint = compile_reference_envelopes(
        snapshot,
        _midpoint_request(
            legacy,
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
            ExactAngleSymbol.PI_OVER_2,
        ),
    )
    assert midpoint.outcome is ReferenceOutcome.EXACT
    angular = [
        item
        for item in midpoint.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    ]
    assert angular
    assert all(item.resolved_hidden_edge_count == 0 for item in angular)
    assert all(item.hidden_supports == frozenset() for item in angular)

    floor = compile_reference_envelopes(
        snapshot,
        _density_request(
            legacy,
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1,
            MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1,
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
            ExactAngleSymbol.PI_OVER_2,
        ),
    )
    assert floor.outcome is ReferenceOutcome.EXACT
    assert all(
        item.resolved_hidden_edge_count == 1
        for item in floor.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )


def test_conveyor_counts_a_zero_count_midpoint_corner_as_a_miter():
    """Очередь обязана назвать `H = 0` митром, а не отказом печати веера.

    Отказ `DENSITY_SEALED_FAN_INVALID` был инвариантом закона A (`H >= 1` по
    построению) и остаётся ровно там; у закона B нулевой веер — законный
    исход, и он попадает в `CONVEYOR_MITERED_CORNERS`, как legacy `k = 0`.
    """

    from cftuv_envelope.wavefront import conveyor as module

    snapshot, legacy = _building_002()
    prepared = module.prepare_conveyor(
        snapshot,
        _midpoint_request(
            legacy,
            MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1,
            ExactAngleSymbol.PI_OVER_2,
        ),
    )
    assert prepared.outcome.value == "EXACT", prepared.detail
    assert prepared.counter("CONVEYOR_MITERED_CORNERS") == 4
    assert prepared.counter("CONVEYOR_RATIONAL_VERTEX_FANS") == 0
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 0


# ---------------------------------------------------------------------------
# (д) Перестановка порядка углов
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("value_id", "symbol"),
    (
        (MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1, ExactAngleSymbol.PI_OVER_2),
        (MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1, ExactAngleSymbol.PI_OVER_3),
    ),
)
def test_corner_permutation_gives_bitwise_identical_certificates(
    value_id,
    symbol,
):
    snapshot, legacy = _building_002()
    request = _midpoint_request(legacy, value_id, symbol)
    first = compile_reference_envelopes(snapshot, request)
    assert first.outcome is ReferenceOutcome.EXACT
    permuted = replace(
        snapshot,
        corner_relations=frozenset(reversed(tuple(snapshot.corner_relations))),
        angular_owner_sectors=frozenset(
            reversed(tuple(snapshot.angular_owner_sectors))
        ),
        reflex_angle_certificates=frozenset(
            reversed(tuple(snapshot.reflex_angle_certificates))
        ),
    )
    second = compile_reference_envelopes(permuted, request)
    assert second.outcome is ReferenceOutcome.EXACT
    assert canonical_json_bytes(
        first.compilation.profile_selection_certificates
    ) == canonical_json_bytes(
        second.compilation.profile_selection_certificates
    )
    assert canonical_json_bytes(
        first.compilation.envelope_specs
    ) == canonical_json_bytes(second.compilation.envelope_specs)


@pytest.mark.parametrize("q", range(2, 7))
def test_cell_selection_equals_nearest_integer_subturn_count(q):
    """Закон B в одну строку: число подповоротов — БЛИЖАЙШЕЕ целое к `u*q`.

    Ничьи (`u*q` ровно в полуцелой точке) уходят вверх — это и есть
    lower-OPEN/upper-CLOSED сторона новой границы. Закон A той же строкой —
    ПОТОЛОК `ceil(u*q)`, зажатый снизу двойкой.
    """

    for numerator in range(1, 240):
        u = Fraction(numerator, 240)
        assert _cell_b(u, q) == max(0, math.ceil(u * q - Fraction(1, 2)) - 1)
        assert _cell_a(u, q) == max(1, math.ceil(u * q) - 1)
