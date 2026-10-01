"""D4-TIGHT-FAN-LIFT: веер ровно на пределе подшага с иррациональными лучами.

Полевой отказ, ради которого это существует: на Fan Density 4 прямой
вогнутый угол даёт ideal-веер из трёх шагов РОВНО по 30° (подшаг `pi/6`).
Остаток подшага тождественно нуль, скрытые лучи 30° и 60° иррациональны, и
допустимая область веера — точка. Прежде такой веер доходил до
`_termination_boxes`, сжимал ящик 96 раз и отказывал, называя причиной
предикат, который никто не проверял.

Закон теперь: подшаг ровно `pi/q` И хотя бы один скрытый луч иррационален —
счёт неосуществим точно (без допуска), существующий лифт поднимает `H` до
`H + 1` (прямой угол на d4 — 4 шага около 22.5° вместо 3 по 30°), и запись
лифта называется `EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1`.

Данные — выгрузка из живой сцены владельца (`artifacts/plan_not_compiled`):
домен меша `2` и два домена `building`; все 16 отказов d4 были этим классом.
"""

from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
from pathlib import Path

from mpmath import atan, mp, mpf, pi as mp_pi
import pytest

import cftuv_envelope as kernel
from cftuv_envelope import (
    AngularEnvelopeSpec,
    AngularProfileSelectionPolicyId,
    ExactAngleSymbol,
    ExactAngleV1,
    ExactRatioV1,
    ExactTurnSignV1,
    EvaluationGeometrySubturnCountLiftLawV1,
    MaxSubturnParameterId,
    MaxSubturnValueId,
)
from cftuv_envelope._density_policy import EVALUATION_SUBTURN_LIFT_PREDICATES
from cftuv_envelope.reference import ReferenceOutcome, compile_reference_envelopes
from cftuv_envelope.reference import adaptive_density_fan
from cftuv_envelope.reference.adaptive_density_fan import (
    _covectors,
    _subturn_boundary,
)
from cftuv_envelope.reference.angular import _ideal_angular_support_data
from cftuv_envelope.reference.common import (
    GeometryContext,
    ReferenceGeometryError,
)
from cftuv_envelope.reference.compile import (
    _density_ideal_is_subturn_feasible,
    _density_spec_with_hidden_count,
)
from cftuv_envelope.reference.subturn_exact_limit import (
    ideal_is_exact_limit_with_irrational_direction,
    turn_is_exactly_at_count_limit,
    verify_exact_limit_lift,
)
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)
from cftuv_envelope.wavefront import prepare_conveyor

import reference_factories as rf
from reference_factories import NEAR_RIGHT_ANGLE, angular_snapshot


FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "density4_exact_limit_v1"

EXACT_LIMIT = (
    EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1
)
STRICT = EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1

# (фикстура, число прямых вогнутых углов домена): у каждого из них счёт 2 -> 3.
DOMAINS = (
    ("mesh2_patch2", 1),
    ("building_patch3", 2),
    ("building_patch20", 8),
)

_DENSITY_VALUES = {
    2: (MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1, ExactAngleSymbol.PI_OVER_4),
    4: (MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1, ExactAngleSymbol.PI_OVER_6),
}


def _load(name: str, density: int):
    folder = FIXTURE / name
    return (
        kernel.AnalysisSnapshotCodecV1.loads(
            (folder / "analysis_snapshot.json").read_bytes()
        ),
        kernel.DecalRequestCodecV1.loads(
            (folder / f"decal_request_density{density}.json").read_bytes()
        ),
    )


def _angular(compilation):
    return tuple(
        item
        for item in compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )


def _lift(spec):
    """Запись лифта; у обычной (не adaptive) спеки поля нет вовсе."""

    return getattr(spec, "evaluation_subturn_count_lift", None)


def _context(snapshot, compilation):
    """Контекст с независимой проверкой всех лифтов (так строит и потребитель)."""

    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics
    return GeometryContext.build(compilation, frame), frame


def _ideal_at(context, spec, count):
    probe = _density_spec_with_hidden_count(spec, count)
    *_, ideal = _ideal_angular_support_data(context, probe)
    return ideal


@pytest.fixture(scope="module")
def compiled():
    cache = {}

    def get(name: str, density: int):
        key = (name, density)
        if key not in cache:
            snapshot, request = _load(name, density)
            cache[key] = (
                snapshot,
                request,
                compile_reference_envelopes(snapshot, request),
            )
        return cache[key]

    return get


@pytest.mark.parametrize(("name", "corners"), DOMAINS)
def test_density_4_right_corner_compiles_exact_with_four_steps(
    compiled, name, corners
):
    """Прямой вогнутый угол на d4: EXACT, лифт 2 -> 3, четыре шага вместо трёх."""

    snapshot, request, result = compiled(name, 4)
    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    specs = _angular(result.compilation)
    assert len(specs) == corners
    selections = {
        item.certificate_id: item
        for item in result.compilation.profile_selection_certificates
    }
    for spec in specs:
        lift = _lift(spec)
        assert lift is not None, spec.envelope_spec_id
        # Именованная запись, а не молчаливая подмена счёта.
        assert lift.lift_law is EXACT_LIMIT
        assert lift.proven_predicates == EVALUATION_SUBTURN_LIFT_PREDICATES[EXACT_LIMIT]
        assert (
            lift.source_hidden_edge_count,
            lift.effective_hidden_edge_count,
            lift.max_subturn_q,
            lift.minimality_predecessor_hidden_edge_count,
        ) == (2, 3, 6, 2)
        # Прямой угол: cos поворота ровно нуль, а не «почти».
        assert lift.evaluation_turn_sign is ExactTurnSignV1.ZERO
        assert lift.evaluation_turn_cosine_squared == ExactRatioV1(0, 1)
        # Селекция не тронута: лифт — evaluation-only.
        assert selections[spec.selection_certificate_id].resolved_hidden_edge_count == 2
        # Три скрытых луча = четыре шага по четверти поворота.
        assert spec.resolved_hidden_edge_count == 3
        assert len(spec.hidden_supports) == 3
        assert {
            Fraction(item.turn_fraction.numerator, item.turn_fraction.denominator)
            for item in spec.hidden_supports
        } == {Fraction(1, 4), Fraction(1, 2), Fraction(3, 4)}
    # Независимая проверка записи лифта проходит на настоящих данных.
    _context(snapshot, result.compilation)

    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    counters = dict(prepared.counters)
    assert counters["CONVEYOR_EXACT_LIMIT_LIFTED_FANS"] == corners
    assert counters["CONVEYOR_FAN_SUPPORTS"] == 3 * corners
    assert counters["CONVEYOR_BOUND_FAN_DIRECTIONS"] == 3 * corners


@pytest.mark.parametrize(("name", "corners"), DOMAINS)
def test_density_4_ideal_fan_is_exactly_at_the_limit_and_the_lift_leaves_it(
    compiled, name, corners
):
    """Признак назван на самих данных: предел при H=2, строгий запас при H=3."""

    snapshot, _, result = compiled(name, 4)
    context, _ = _context(snapshot, result.compilation)
    metric = context.metric
    for spec in _angular(result.compilation):
        at_limit = _ideal_at(context, spec, 2)
        lifted = _ideal_at(context, spec, 3)
        covectors = _covectors(metric, at_limit)
        # Прежний предикат считает такой веер ОСУЩЕСТВИМЫМ — в этом и ловушка.
        assert _density_ideal_is_subturn_feasible(metric, at_limit, 6)
        assert _subturn_boundary(metric, covectors[0], covectors[1], 6)
        assert ideal_is_exact_limit_with_irrational_direction(metric, at_limit, 6)
        # После лифта запас строгий: ни остатка ноль, ни точки.
        assert _density_ideal_is_subturn_feasible(metric, lifted, 6)
        assert not _subturn_boundary(
            metric,
            _covectors(metric, lifted)[0],
            _covectors(metric, lifted)[1],
            6,
        )
        assert not ideal_is_exact_limit_with_irrational_direction(metric, lifted, 6)


@pytest.mark.parametrize(("name", "corners"), DOMAINS)
def test_density_2_rational_exact_limit_is_not_touched(compiled, name, corners):
    """Детектор точен: предел БЕЗ иррационального луча не срабатывает.

    На d2 (`q = 4`) тот же прямой угол стоит ровно на пределе (два шага по
    45°, остаток подшага нуль), но биссектриса рациональна — допустимая точка
    существует, и ответ прежний: один скрытый луч, лифта нет.
    """

    snapshot, request, result = compiled(name, 2)
    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    context, _ = _context(snapshot, result.compilation)
    metric = context.metric
    for spec in _angular(result.compilation):
        assert _lift(spec) is None
        assert spec.resolved_hidden_edge_count == 1
        ideal = _ideal_at(context, spec, 1)
        covectors = _covectors(metric, ideal)
        assert _subturn_boundary(metric, covectors[0], covectors[1], 4)
        assert not ideal_is_exact_limit_with_irrational_direction(metric, ideal, 4)
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT"
    # Нулевой счётчик не печатается: структурные счётчики заморожены воротами.
    assert "CONVEYOR_EXACT_LIMIT_LIFTED_FANS" not in dict(prepared.counters)


def _forged(compilation, spec, **changes):
    lift = replace(spec.evaluation_subturn_count_lift, **changes)
    forged_spec = replace(spec, evaluation_subturn_count_lift=lift)
    return replace(
        compilation,
        envelope_specs=frozenset(
            forged_spec if item == spec else item
            for item in compilation.envelope_specs
        ),
    )


def test_exact_limit_lift_tampering_is_refused_by_the_independent_verifier(compiled):
    snapshot, _, result = compiled("building_patch3", 4)
    compilation = result.compilation
    spec = _angular(compilation)[0]
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None, diagnostics
    GeometryContext.build(compilation, frame)
    other_law = {
        EXACT_LIMIT: STRICT,
        STRICT: EXACT_LIMIT,
    }
    forgeries = (
        # Прежний закон на фактах предела: сырой предшественник осуществим.
        {
            "lift_law": STRICT,
            "proven_predicates": EVALUATION_SUBTURN_LIFT_PREDICATES[STRICT],
        },
        # Закон не обеспечен своим набором доказанных предикатов.
        {"proven_predicates": EVALUATION_SUBTURN_LIFT_PREDICATES[STRICT]},
        {"lift_law": other_law[EXACT_LIMIT]},
        # Знак и cos^2 свидетельства — отдельные факты геометрии.
        {"evaluation_turn_sign": ExactTurnSignV1.NEGATIVE},
        {"evaluation_turn_sign": ExactTurnSignV1.POSITIVE},
        {"evaluation_turn_cosine_squared": ExactRatioV1(1, 4)},
        # Предшественник — всегда `effective - 1`.
        {"minimality_predecessor_hidden_edge_count": 1},
        {"source_hidden_edge_count": 3},
    )
    for changes in forgeries:
        with pytest.raises(ReferenceGeometryError):
            GeometryContext.build(_forged(compilation, spec, **changes), frame)
    lifted_away = replace(spec, evaluation_subturn_count_lift=None)
    with pytest.raises(ReferenceGeometryError):
        GeometryContext.build(
            replace(
                compilation,
                envelope_specs=frozenset(
                    lifted_away if item == spec else item
                    for item in compilation.envelope_specs
                ),
            ),
            frame,
        )


def test_the_exact_limit_law_survives_the_wire_and_the_structural_validator(compiled):
    """Закон лежит в схеме: запись переживает кодек и структурную проверку."""

    from cftuv_envelope.adaptive_density_validation import (
        adaptive_density_effective_hidden_count,
    )

    _, _, result = compiled("mesh2_patch2", 4)
    spec = _angular(result.compilation)[0]
    selection = next(
        item
        for item in result.compilation.profile_selection_certificates
        if item.certificate_id == spec.selection_certificate_id
    )
    assert adaptive_density_effective_hidden_count(spec, selection) == (3, ())
    lift = _lift(spec)
    unproven = replace(
        spec,
        evaluation_subturn_count_lift=replace(
            lift, proven_predicates=EVALUATION_SUBTURN_LIFT_PREDICATES[STRICT]
        ),
    )
    count, errors = adaptive_density_effective_hidden_count(unproven, selection)
    assert count == 2 and errors, "закон без своих предикатов не авторизован"


_TABLE = (
    # (знак, cos^2, счёт, q) -> поворот ровно (счёт + 1) * pi / q
    (ExactTurnSignV1.ZERO, Fraction(0), 2, 6, True),
    (ExactTurnSignV1.POSITIVE, Fraction(0), 2, 6, False),
    (ExactTurnSignV1.POSITIVE, Fraction(1, 4), 1, 6, True),
    (ExactTurnSignV1.NEGATIVE, Fraction(1, 4), 1, 6, False),
    (ExactTurnSignV1.NEGATIVE, Fraction(1, 4), 3, 6, True),
    (ExactTurnSignV1.POSITIVE, Fraction(1, 4), 3, 6, False),
    (ExactTurnSignV1.POSITIVE, Fraction(1, 2), 0, 4, True),
    (ExactTurnSignV1.NEGATIVE, Fraction(1, 2), 2, 4, True),
    (ExactTurnSignV1.POSITIVE, Fraction(1, 2), 2, 4, False),
    (ExactTurnSignV1.ZERO, Fraction(0), 1, 4, True),
    (ExactTurnSignV1.POSITIVE, Fraction(3, 4), 4, 6, False),
    (ExactTurnSignV1.POSITIVE, Fraction(3, 4), 0, 6, True),
    (ExactTurnSignV1.NEGATIVE, Fraction(3, 4), 4, 6, True),
    (ExactTurnSignV1.POSITIVE, Fraction(1, 10), 1, 5, False),
    # Не равен ни на волос: cos^2 чуть выше нуля — это уже не предел.
    (ExactTurnSignV1.NEGATIVE, Fraction(1, 10**9), 2, 6, False),
    (ExactTurnSignV1.POSITIVE, Fraction(1, 10**9), 2, 6, False),
    # Порог `>= 1` — потолок поворота, а не предел.
    (ExactTurnSignV1.NEGATIVE, Fraction(1), 5, 6, False),
)


@pytest.mark.parametrize(("sign", "cosine_squared", "count", "q", "expected"), _TABLE)
def test_turn_is_exactly_at_count_limit_is_an_equality_not_a_tolerance(
    sign, cosine_squared, count, q, expected
):
    assert turn_is_exactly_at_count_limit(sign, cosine_squared, count, q) is expected


def _exact_twin(name: str = "exact-right-angle"):
    """Точный прямой угол: нормаль исходящей опоры ровно (0, -1)."""

    rf._ANGULAR_CASES.setdefault(
        name, ((0.0, -1.0), (-5.0, 0.0), ("0.5", "0.5"))
    )
    return name


def _tilted_right_angle(tilt: str, name: str):
    """Угол чуть МЕНЬШЕ прямого: запас положителен, предела нет.

    Нормаль исходящей опоры `(+tilt, -1)`: `delta/pi = 1/2 - atan(tilt)/pi`.
    Границы пары — те же 27 знаков, что у `NEAR_RIGHT_ANGLE`, но зеркально.
    """

    mp.dps = 80
    value = mpf(1) / 2 - atan(mpf(tilt)) / mp_pi
    scale = mpf(10) ** 27
    lower = int(value * scale) / Decimal(10**27)
    upper = (int(value * scale) + 1) / Decimal(10**27)
    rf._ANGULAR_CASES.setdefault(
        name,
        (
            (float(tilt), -1.0),
            (-5.0, -5.0 * float(tilt)),
            (format(lower, "f"), format(upper, "f")),
        ),
    )
    return name


def _synthetic(case, density: int):
    snapshot, legacy_request = angular_snapshot(case)
    value_id, symbol = _DENSITY_VALUES[density]
    request = replace(
        legacy_request,
        angular_profile_selection_policy_id=(
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        ),
        max_subturn_parameter_id=MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1,
        max_subturn_value_id=value_id,
        max_subturn_exact_value=ExactAngleV1(symbol),
    )
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT, (case, density, result.diagnostics)
    (spec,) = _angular(result.compilation)
    return snapshot, result.compilation, spec


def test_exact_right_angle_lifts_under_the_exact_limit_law_at_q6():
    _, compilation, spec = _synthetic(_exact_twin(), 4)
    lift = _lift(spec)
    assert lift.lift_law is EXACT_LIMIT
    assert (lift.source_hidden_edge_count, lift.effective_hidden_edge_count) == (2, 3)
    assert lift.evaluation_turn_sign is ExactTurnSignV1.ZERO
    assert spec.resolved_hidden_edge_count == 3
    assert compilation.canonical_subturn_fan_authorities == frozenset()


def test_restored_right_angle_at_q6_is_closed_with_the_same_four_steps():
    """Нож q=6 у восстановленных углов закрыт: тот же счёт, закон — строгий.

    Сырой угол чуть БОЛЬШЕ 90°, поэтому на предшественнике H=2 подшаг сырой
    геометрии СТРОГО больше `pi/6`: основание лифта — прежний закон, и оно
    проверяемо на сырой геометрии. Канонический веер (3 шага по 30° на
    каноническом угле) здесь невозможен — его лучи иррациональны, точка не
    имеет рационального представителя, — поэтому власть канонического подшага
    НЕ пишется, а счёт поднят так же, как у точного близнеца.
    """

    twin = _synthetic(_exact_twin(), 4)[2]
    snapshot, compilation, spec = _synthetic(NEAR_RIGHT_ANGLE, 4)
    lift = _lift(spec)
    assert lift.lift_law is STRICT
    assert lift.evaluation_turn_sign is ExactTurnSignV1.NEGATIVE
    assert (lift.source_hidden_edge_count, lift.effective_hidden_edge_count) == (2, 3)
    assert spec.resolved_hidden_edge_count == twin.resolved_hidden_edge_count == 3
    assert len(compilation.canonical_angle_restorations) == 1
    assert compilation.canonical_subturn_fan_authorities == frozenset()
    _context(snapshot, compilation)


@pytest.mark.parametrize(
    ("tilt", "restored"),
    (("1.5e-6", True), ("1e-4", False)),
)
def test_near_right_angle_with_positive_slack_keeps_three_steps(tilt, restored):
    """Околопрямой угол с положительным запасом — три шага, лифта нет.

    `1.5e-6` — класс восстановления (шум моделирования внутри допуска
    намерения), `1e-4` — вне его. Оба стоят ниже 90°: подшаг строго меньше
    `pi/6`, область веера открыта, и признак «на пределе» обязан молчать.
    """

    case = _tilted_right_angle(tilt, f"below-right-angle-{tilt}")
    snapshot, compilation, spec = _synthetic(case, 4)
    assert _lift(spec) is None
    assert spec.resolved_hidden_edge_count == 2
    assert compilation.canonical_subturn_fan_authorities == frozenset()
    assert bool(compilation.canonical_angle_restorations) is restored
    context, _ = _context(snapshot, compilation)
    ideal = _ideal_at(context, spec, 2)
    assert _density_ideal_is_subturn_feasible(context.metric, ideal, 6)
    assert not ideal_is_exact_limit_with_irrational_direction(
        context.metric, ideal, 6
    )


def test_exact_right_angle_at_q4_has_a_rational_limit_and_no_lift():
    snapshot, compilation, spec = _synthetic(_exact_twin(), 2)
    assert _lift(spec) is None
    assert spec.resolved_hidden_edge_count == 1
    context, _ = _context(snapshot, compilation)
    ideal = _ideal_at(context, spec, 1)
    covectors = _covectors(context.metric, ideal)
    assert _subturn_boundary(context.metric, covectors[0], covectors[1], 4)
    assert not ideal_is_exact_limit_with_irrational_direction(
        context.metric, ideal, 4
    )


def test_verifier_refuses_a_limit_claim_on_a_rational_fan_and_off_the_limit():
    """Две половины признака проверяются порознь, по геометрии, а не по записи."""

    snapshot, compilation, spec = _synthetic(_exact_twin(), 2)
    context, _ = _context(snapshot, compilation)
    selection = next(iter(compilation.profile_selection_certificates))

    def claim(**changes):
        base = dict(
            lift_law=EXACT_LIMIT,
            source_selection_certificate_id=selection.certificate_id,
            source_hidden_edge_count=1,
            effective_hidden_edge_count=2,
            max_subturn_q=4,
            evaluation_turn_sign=ExactTurnSignV1.ZERO,
            evaluation_turn_cosine_squared=ExactRatioV1(0, 1),
            minimality_predecessor_hidden_edge_count=1,
            proven_predicates=EVALUATION_SUBTURN_LIFT_PREDICATES[EXACT_LIMIT],
        )
        base.update(changes)
        return kernel.EvaluationGeometrySubturnCountLiftV1(**base)

    # Угол прямой, предел при H=1 настоящий, но луч 45° рационален.
    with pytest.raises(ValueError, match="only rational hidden directions"):
        verify_exact_limit_lift(context, spec, claim())
    # Предел заявлен не на том счёте: при H=0 поворот 90° не равен pi/4 * 1.
    with pytest.raises(ValueError, match="not exactly at the subturn limit"):
        verify_exact_limit_lift(
            context,
            spec,
            claim(
                source_hidden_edge_count=2,
                effective_hidden_edge_count=3,
                minimality_predecessor_hidden_edge_count=2,
            ),
        )
    # Исходный счёт выше предшественника — лифт назад, а не вперёд.
    with pytest.raises(ValueError, match="exceeds its predecessor"):
        verify_exact_limit_lift(
            context,
            spec,
            claim(source_hidden_edge_count=3),
        )


def _density_1_inputs():
    folder = Path(__file__).resolve().parents[1] / "fixtures" / "building_002_full_selection_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (folder / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (folder / "decal_request.json").read_bytes()
    )
    return snapshot, replace(
        request,
        angular_profile_selection_policy_id=(
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        ),
        max_subturn_parameter_id=MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1,
        max_subturn_value_id=MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        max_subturn_exact_value=ExactAngleV1(ExactAngleSymbol.PI_OVER_3),
    )


@pytest.mark.parametrize("route", ("lifted_fan", "binding_fallback"))
def test_box_refinement_exhaustion_has_its_own_honest_name(monkeypatch, route):
    """Исчерпанный кап уточнения ящика называется собой, а не «UNDECIDABLE».

    Предел с иррациональным лучом отсекается раньше, точным признаком, и в
    ящик не попадает; сюда доходит только то, что осталось, — нехватка
    уточнения. Ящик здесь принудительно не находит осуществимой точки, чтобы
    проверить оба пути в `_termination_boxes`: через лифтованный веер
    (`certify_adaptive_density_fan`) и через adaptive fallback после пустой B(w).
    """

    monkeypatch.setattr(
        adaptive_density_fan, "_box_is_feasible", lambda *args, **kwargs: False
    )
    if route == "lifted_fan":
        snapshot, request = _load("building_patch3", 4)
    else:
        snapshot, request = _density_1_inputs()
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED
    message = result.diagnostics[0].message
    assert "termination box refinement exhausted" in message
    assert "BINDING_MONOTONE" not in message
    if route == "lifted_fan":
        # После лифта предела нет: причина названа как нехватка уточнения.
        assert "subturn limit: False" in message
    assert result.outcome is not ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "PLAN_IS_NOT_COMPILED"
    assert prepared.detail == "DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED"

