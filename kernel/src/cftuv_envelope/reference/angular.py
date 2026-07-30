"""LINEAR_REFLEX_EQUAL_V1 construction: frozen legacy plus Density A fan."""

from __future__ import annotations

from dataclasses import dataclass, replace
from fractions import Fraction

import sympy as sp
from mpmath import iv

from ..contracts.analysis import TurnOrientation
from ..contracts.envelopes import (
    AdaptiveBoundHiddenSupportDirectionLawV2,
    AdaptiveBoundHiddenSupportSpecV2,
    AdaptiveDensityAngularEnvelopeSpecV2,
    AngularEnvelopeSpec,
    CertifiedBoundHiddenSupportDirectionLawV1,
    CertifiedBoundHiddenSupportSpecV1,
    DirectionBindingReasonV1,
    EvaluationGeometryDirectionBindingCertificateV1,
    EvaluationGeometrySubturnCountLiftLawV1,
    EvaluationGeometrySubturnCountLiftV1,
    ExactTurnSignV1,
    HiddenSupportDirectionLaw,
    HiddenSupportSpecV1,
    StripEnvelopeSpec,
)
from ..contracts.request import (
    AngularProfileSelectionPolicyId,
)
from .._density_policy import (
    DensityIntervalEnclosureUnsupported,
    density_interval_enclosure,
    huber_density_value_contract,
)
from ..numeric import ExactRatioV1, LocalLengthV1
from .common import (
    GeometryContext,
    ReferenceGeometryError,
    make_region,
    make_segment,
    source_vertex_certificate,
    stable_id,
    support_vertex_certificate,
)
from .contracts import ReferenceEnvelopeInstanceV1, ReferenceOutcome
from .direction_binding import (
    BINDING_MONOTONE,
    DirectionBindingCertificateUnproven,
    _verify_k1_recipe_direction_bindings,
    bound_unit_normal,
    bound_unit_normal_from_vector,
    has_rational_density_support_direction,
    has_rational_support_direction,
    verify_direction_bindings,
    verify_huber_density_direction_bindings,
    verify_adaptive_huber_density_direction_fan,
)
from .planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactPlanarVector,
    ExactScalar,
    ExactQuadraticFieldUnsupported,
    exact_quadratic_value,
    exact_normalize,
    exact_sign,
    point_sub,
    support_intersection,
)
from .metric import ExactPlanarMetric
from .provenance import make_reference_provenance, merge_provenance


_EVALUATION_SUBTURN_LIFT_PREDICATES = frozenset(
    {
        "SOURCE_SELECTION_CERTIFICATE_IMMUTABLE",
        "SOURCE_COUNT_EXACTLY_INFEASIBLE_IN_EVALUATION_GEOMETRY",
        "EFFECTIVE_COUNT_EXACTLY_FEASIBLE_IN_EVALUATION_GEOMETRY",
        "EFFECTIVE_COUNT_IS_MINIMAL",
    }
)


@dataclass(frozen=True, slots=True)
class _DensityRuntimeVector:
    """Транзакционный exact-вектор без wire srepr/reparse."""

    x: sp.Expr
    y: sp.Expr

    def expressions(self, transaction_memo=None):
        del transaction_memo
        return self.x, self.y


def _density_runtime_vector(x: sp.Expr, y: sp.Expr) -> _DensityRuntimeVector:
    return _DensityRuntimeVector(sp.sympify(x), sp.sympify(y))


def _incident_normal(
    context: GeometryContext, chain_use_id, anchor_vertex_id
) -> tuple[ExactPlanarVector, str]:
    strip = next(
        item
        for item in context.compilation.envelope_specs
        if isinstance(item, StripEnvelopeSpec)
        and next(
            seed.chain_use_id
            for seed in context.compilation.seeds
            if getattr(seed, "seed_id", None) == item.source_seed_id
        )
        == chain_use_id
    )
    segments = context.support_segments_for_use(
        chain_use_id, strip.envelope_spec_id.value
    )
    matches = [
        item
        for item in segments
        if anchor_vertex_id
        in (item.source_vertex_start_id, item.source_vertex_end_id)
    ]
    if len(matches) != 1:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_CHAIN_SUPPORT_NOT_LINEAR,
            f"angular anchor is not a unique physical endpoint for {chain_use_id}",
        )
    return matches[0].owner_normal, matches[0].support_id


def _interpolated_normals(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
    count: int,
    orientation: TurnOrientation,
    *,
    huber_density: bool = False,
) -> tuple[ExactPlanarVector, ...]:
    expected = 1 if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION else -1
    if huber_density:
        return _huber_density_interpolated_normals(
            metric,
            incoming,
            outgoing,
            count,
            expected,
        )
    incoming = metric.unit_g(incoming)
    outgoing = metric.unit_g(outgoing)
    turn_cross = exact_sign(metric.oriented_cross(incoming, outgoing))
    if turn_cross != expected:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "ordered support normals do not realize the certified owner-sector turn",
        )
    if count == 0:
        return incoming, outgoing
    ix, iy = incoming.expressions()
    lx, ly = metric.owner_normal_g(incoming, owner_left=True).expressions()
    if count == 1:
        hidden = metric.unit_g(
            ExactPlanarVector.from_values(
                ix + outgoing.x.as_expr(), iy + outgoing.y.as_expr()
            )
        )
        return incoming, hidden, outgoing
    if count != 2:  # v1 phi < 2*pi and delta_max=pi/3 imply k <= 2.
        raise ReferenceGeometryError(
            ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
            "LINEAR_REFLEX_EQUAL_V1 supports only the proven v1 K=0/1/2 range",
        )
    cosine_delta = exact_normalize(metric.dot_g(incoming, outgoing))
    root_symbol = sp.Symbol("linear_reflex_cos_subturn", real=True)
    candidates = sp.roots(
        4 * root_symbol**3 - 3 * root_symbol - cosine_delta,
        root_symbol,
        extension=True,
    ).keys()
    admissible = []
    for candidate in candidates:
        try:
            if exact_sign(candidate - sp.Rational(1, 2)) > 0 and exact_sign(
                candidate - 1
            ) <= 0:
                admissible.append(candidate)
        except Exception:
            continue
    if len(admissible) != 1:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "could not prove the unique oriented one-third support direction",
        )
    cosine_step = exact_normalize(admissible[0])
    sine_step = expected * sp.sqrt(exact_normalize(1 - cosine_step**2))
    hidden_one = ExactPlanarVector.from_values(
        cosine_step * ix + sine_step * lx,
        cosine_step * iy + sine_step * ly,
    )
    cosine_double = exact_normalize(2 * cosine_step**2 - 1)
    sine_double = exact_normalize(2 * cosine_step * sine_step)
    hidden_two = ExactPlanarVector.from_values(
        cosine_double * ix + sine_double * lx,
        cosine_double * iy + sine_double * ly,
    )
    for normal in (hidden_one, hidden_two):
        if exact_sign(exact_normalize(metric.dot_g(normal, normal) - 1)) != 0:
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
                "hidden support unit-speed proof failed",
            )
    return incoming, hidden_one, hidden_two, outgoing


def _density_exact_vector(
    x: sp.Expr,
    y: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> ExactPlanarVector:
    """Записать bounded Density-expression без global factor-canonicalizer."""

    def scalar(expression: sp.Expr) -> ExactScalar:
        expression = sp.sympify(expression)
        if isinstance(expression, sp.Rational):
            return ExactScalar.from_value(expression)
        return ExactScalar(_density_srepr(expression, metric))

    return ExactPlanarVector(scalar(x), scalar(y))


def _density_srepr(
    expression: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> str:
    """Сериализовать expression с memo конкретной metric-транзакции."""

    expression = sp.sympify(expression)
    if metric is None:
        return sp.srepr(expression)
    cached = metric._density_exact_memo.sreprs.get(expression)
    if cached is not None:
        return cached
    result = sp.srepr(expression)
    metric._density_exact_memo.sreprs[expression] = result
    metric._density_exact_memo.expressions[result] = expression
    return result


def _density_dot_expression(
    metric: ExactPlanarMetric,
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> sp.Expr:
    """Dual-free Gram dot без generic normalization/factor."""

    lx, ly = metric.density_expressions(left)
    rx, ry = metric.density_expressions(right)
    return (
        lx * (metric.gram[0][0] * rx + metric.gram[0][1] * ry)
        + ly * (metric.gram[1][0] * rx + metric.gram[1][1] * ry)
    )


def _density_exact_sign(
    expression: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> int:
    """Знак Density-факта с memo конкретной metric-транзакции."""

    expression = sp.sympify(expression)
    if metric is not None:
        cached = metric._density_exact_memo.signs.get(expression)
        if cached is not None:
            return cached
    if isinstance(expression, sp.Rational):
        return (expression.p > 0) - (expression.p < 0)
    if expression == 0:
        return 0
    saved = iv.prec
    iv.prec = 160
    try:
        enclosure = density_interval_enclosure(
            expression,
            (
                None
                if metric is None
                else metric._density_exact_memo.intervals
            ),
        )
    except (
        DensityIntervalEnclosureUnsupported,
        ArithmeticError,
        TypeError,
        ValueError,
    ):
        enclosure = None
    finally:
        iv.prec = saved
    if enclosure is not None:
        if enclosure.a > 0:
            result = 1
            if metric is not None:
                metric._density_exact_memo.signs[expression] = result
            return result
        if enclosure.b < 0:
            result = -1
            if metric is not None:
                metric._density_exact_memo.signs[expression] = result
            return result
    try:
        result = exact_quadratic_value(sp.expand(expression)).sign()
        if metric is not None:
            metric._density_exact_memo.signs[expression] = result
        return result
    except ExactQuadraticFieldUnsupported:
        pass
    raise ReferenceGeometryError(
        ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
        "Density A exact sign is not certified without generic factorization",
    )


def _density_unit_from_squared(
    vector: ExactPlanarVector,
    squared: sp.Expr,
    metric: ExactPlanarMetric | None = None,
) -> ExactPlanarVector:
    if _density_exact_sign(squared, metric) <= 0:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "Density A support direction has non-positive Gram norm",
        )
    x, y = (
        vector.expressions()
        if metric is None
        else metric.density_expressions(vector)
    )
    length = sp.sqrt(squared)
    return _density_runtime_vector(x / length, y / length)


def _density_left_unit_normal(
    metric: ExactPlanarMetric,
    tangent: ExactPlanarVector,
) -> ExactPlanarVector:
    tx, ty = metric.density_expressions(tangent)
    covector_x = metric.gram[0][0] * tx + metric.gram[0][1] * ty
    covector_y = metric.gram[1][0] * tx + metric.gram[1][1] * ty
    sign = metric.owner_orientation_sign
    raw = _density_runtime_vector(
        -sign * covector_y,
        sign * covector_x,
    )
    orthogonality = sp.expand(
        _density_dot_expression(metric, tangent, raw)
    )
    if orthogonality != 0:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "Density A Gram-orthogonal basis construction is not exact",
        )
    return _density_unit_from_squared(
        raw,
        sp.expand(_density_dot_expression(metric, raw, raw)),
        metric,
    )


def _huber_density_interpolated_normals(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
    count: int,
    orientation_sign: int,
) -> tuple[ExactPlanarVector, ...]:
    """Равноугольный веер H=1..5 без generic root solver.

    Власть ветви выводится из production Gram-фактов: знак `dot_g` и
    несократимый рациональный `dot_g²`.  `atan2(sqrt(1-c²), c)/(H+1)` —
    точная principal-ветвь того же корня `T_n(x)=c`; интервалы ниже лишь
    сертифицируют знаки/окна и никогда не подменяют конструкцию числом.
    """

    if count not in range(1, 6):
        raise ReferenceGeometryError(
            ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
            "Density A supports only the certified H=1..5 range",
        )
    incoming_squared = sp.expand(
        _density_dot_expression(metric, incoming, incoming)
    )
    outgoing_squared = sp.expand(
        _density_dot_expression(metric, outgoing, outgoing)
    )
    raw_dot = sp.expand(
        _density_dot_expression(metric, incoming, outgoing)
    )
    raw_cross = sp.expand(
        metric.owner_orientation_sign
        * (
            metric.density_expressions(incoming)[0]
            * metric.density_expressions(outgoing)[1]
            - metric.density_expressions(incoming)[1]
            * metric.density_expressions(outgoing)[0]
        )
    )
    if _density_exact_sign(raw_cross, metric) != orientation_sign:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "ordered support normals do not realize the certified owner-sector turn",
        )
    if (
        incoming_squared.is_Rational is not True
        or outgoing_squared.is_Rational is not True
    ):
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "Density A source Gram norms are not rational",
        )
    incoming = _density_unit_from_squared(
        incoming,
        incoming_squared,
        metric,
    )
    outgoing = _density_unit_from_squared(
        outgoing,
        outgoing_squared,
        metric,
    )
    raw_dot_squared = sp.expand(raw_dot * raw_dot)
    if raw_dot_squared.is_Rational is not True:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            "Density A signed-cos-squared is not rational in the declared Gram metric",
        )
    cosine_squared = raw_dot_squared / (
        incoming_squared * outgoing_squared
    )
    if (
        _density_exact_sign(cosine_squared, metric) < 0
        or _density_exact_sign(cosine_squared - 1, metric) >= 0
    ):
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "Density A requires a strict principal turn in (0, pi)",
        )
    turn_sign = _density_exact_sign(raw_dot, metric)
    cosine_total = turn_sign * sp.sqrt(cosine_squared)
    sine_squared = 1 - cosine_squared
    principal_turn = sp.atan2(sp.sqrt(sine_squared), cosine_total)
    subturn_count = count + 1
    ix, iy = metric.density_expressions(incoming)
    lx, ly = metric.density_expressions(
        _density_left_unit_normal(metric, incoming)
    )
    hidden = []
    for ordinal in range(1, subturn_count):
        angle = sp.Rational(ordinal, subturn_count) * principal_turn
        cosine = sp.cos(angle)
        sine = orientation_sign * sp.sin(angle)
        normal = _density_runtime_vector(
            cosine * ix + sine * lx,
            cosine * iy + sine * ly,
        )
        hidden.append(normal)
    # `principal_turn in (0, pi)` доказан signed-cos² и знаком cross.
    # Поэтому каждая разность соседних ordinal углов строго одного знака.
    return incoming, *hidden, outgoing


def _ideal_angular_support_data(
    context: GeometryContext,
    spec: AngularEnvelopeSpec,
):
    # Ключ включает всю immutable запись: context могут законно копировать
    # через `replace(..., compilation=forged)` adversarial-тесты/consumers.
    cache_key = spec
    cached = context.angular_ideal_cache.get(cache_key)
    if cached is not None:
        return cached
    relation = next(
        (
            item
            for item in context.snapshot.corner_relations
            if item.corner_relation_id == spec.source_relation_id
        ),
        None,
    )
    if relation is None:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID,
            (
                f"AngularEnvelope {spec.envelope_spec_id} references unknown "
                f"CornerRelation {spec.source_relation_id}"
            ),
        )
    sector = next(
        item
        for item in context.snapshot.angular_owner_sectors
        if item.owner_sector_id == spec.owner_sector_id
    )
    anchor = context.points_by_id[relation.source_vertex_id]
    hidden_by_ordinal = {item.ordinal: item for item in spec.hidden_supports}
    expected_ordinals = set(range(1, spec.resolved_hidden_edge_count + 1))
    if set(hidden_by_ordinal) != expected_ordinals or len(hidden_by_ordinal) != len(
        spec.hidden_supports
    ):
        raise ReferenceGeometryError(
            ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
            "AngularEnvelope hidden-support records do not match the selected K",
        )
    for support in hidden_by_ordinal.values():
        law_matches_tag = (
            type(support) is HiddenSupportSpecV1
            and support.direction_law
            is HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN
        ) or (
            type(support) is CertifiedBoundHiddenSupportSpecV1
            and support.direction_law
            is CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1
        ) or (
            type(support) is AdaptiveBoundHiddenSupportSpecV2
            and support.direction_law
            is AdaptiveBoundHiddenSupportDirectionLawV2.ADAPTIVE_MINIMAL_RATIONAL_FAN_V2
        )
        if not law_matches_tag:
            exc = DirectionBindingCertificateUnproven(BINDING_MONOTONE)
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
                f"direction binding certificate is not proven: {exc}",
            ) from exc
    incoming, incoming_support_id = _incident_normal(
        context,
        sector.ordered_incident_chain_use_ids[0],
        relation.source_vertex_id,
    )
    outgoing, outgoing_support_id = _incident_normal(
        context,
        sector.ordered_incident_chain_use_ids[-1],
        relation.source_vertex_id,
    )
    ideal = _interpolated_normals(
        context.metric,
        incoming,
        outgoing,
        spec.resolved_hidden_edge_count,
        sector.turn_orientation,
        huber_density=(
            next(
                item
                for item in context.compilation.profile_selection_certificates
                if item.certificate_id == spec.selection_certificate_id
            ).selection_policy_id
            is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        ),
    )
    support_ids = [incoming_support_id]
    support_ids.extend(
        hidden_by_ordinal[index].hidden_support_id.value
        for index in range(1, spec.resolved_hidden_edge_count + 1)
    )
    support_ids.append(outgoing_support_id)
    result = (
        relation,
        sector,
        anchor,
        hidden_by_ordinal,
        tuple(support_ids),
        ideal,
    )
    context.angular_ideal_cache[cache_key] = result
    return result


def _lift_probe_spec(spec, hidden_count):
    supports = frozenset(
        replace(
            support,
            ordinal=ordinal,
            turn_fraction=ExactRatioV1(ordinal, hidden_count + 1),
        )
        for ordinal, support in enumerate(
            sorted(spec.hidden_supports, key=lambda item: item.ordinal)[
                :hidden_count
            ],
            start=1,
        )
    )
    return replace(
        spec,
        resolved_hidden_edge_count=hidden_count,
        hidden_supports=supports,
    )


def _compare_q5_cos_squared(value: Fraction, numerator: int) -> int:
    """Сравнить rational value с cos²(n*pi/5) без materialized radical."""

    shifted = 8 * value - 3
    radical_sign = 1 if numerator in (1, 4) else -1
    if radical_sign > 0:
        if shifted < 0:
            return -1
        squared = shifted * shifted
        return 0 if squared == 5 else 1 if squared > 5 else -1
    if shifted >= 0:
        return 1
    squared = shifted * shifted
    return 0 if squared == 5 else -1 if squared > 5 else 1


def _compare_turn_cos_squared(
    value: Fraction,
    threshold_turn: Fraction,
) -> int:
    """Сравнить с cos²(threshold_turn*pi) на закрытом q<=6 наборе."""

    reduced = (threshold_turn.numerator, threshold_turn.denominator)
    rational = {
        (1, 6): Fraction(3, 4),
        (5, 6): Fraction(3, 4),
        (1, 4): Fraction(1, 2),
        (3, 4): Fraction(1, 2),
        (1, 3): Fraction(1, 4),
        (2, 3): Fraction(1, 4),
        (1, 2): Fraction(0),
    }.get(reduced)
    if rational is not None:
        return (value > rational) - (value < rational)
    if threshold_turn.denominator == 5:
        return _compare_q5_cos_squared(value, threshold_turn.numerator)
    raise ValueError("Density H-lift threshold is outside q<=6")


def _lift_count_is_feasible(lift, hidden_count: int) -> bool:
    """Проверить theta/(H+1)<=pi/q по sealed signed-cos² рационально."""

    threshold_turn = Fraction(hidden_count + 1, lift.max_subturn_q)
    if threshold_turn >= 1:
        return True
    sign = (
        1
        if lift.evaluation_turn_sign is ExactTurnSignV1.POSITIVE
        else -1
        if lift.evaluation_turn_sign is ExactTurnSignV1.NEGATIVE
        else 0
    )
    if threshold_turn == Fraction(1, 2):
        return sign >= 0
    cosine_squared = Fraction(
        lift.evaluation_turn_cosine_squared.numerator,
        lift.evaluation_turn_cosine_squared.denominator,
    )
    comparison = _compare_turn_cos_squared(
        cosine_squared,
        threshold_turn,
    )
    if threshold_turn < Fraction(1, 2):
        return sign > 0 and comparison >= 0
    return sign >= 0 or comparison <= 0


def _verify_evaluation_subturn_count_lift(
    context,
    spec,
    ideal,
) -> None:
    from .adaptive_density_fan import _covectors, _dual_dot, _subturn

    selection = next(
        item
        for item in context.compilation.profile_selection_certificates
        if item.certificate_id == spec.selection_certificate_id
    )
    lift = spec.evaluation_subturn_count_lift
    if lift is None:
        if (
            spec.resolved_hidden_edge_count
            != selection.resolved_hidden_edge_count
        ):
            raise ValueError("Density effective H lacks a lift authority")
        return
    q_contract = huber_density_value_contract(
        selection.max_subturn_value_id
    )
    if (
        type(lift) is not EvaluationGeometrySubturnCountLiftV1
        or lift.lift_law
        is not EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1
        or q_contract is None
        or lift.source_selection_certificate_id
        != selection.certificate_id
        or lift.source_hidden_edge_count
        != selection.resolved_hidden_edge_count
        or lift.effective_hidden_edge_count
        != spec.resolved_hidden_edge_count
        or lift.max_subturn_q != q_contract[0]
        or lift.effective_hidden_edge_count
        <= lift.source_hidden_edge_count
        or any(
            type(value) is not int
            for value in (
                lift.source_hidden_edge_count,
                lift.effective_hidden_edge_count,
                lift.max_subturn_q,
                lift.minimality_predecessor_hidden_edge_count,
            )
        )
        or lift.minimality_predecessor_hidden_edge_count
        != lift.effective_hidden_edge_count - 1
        or lift.proven_predicates
        != _EVALUATION_SUBTURN_LIFT_PREDICATES
        or type(lift.evaluation_turn_sign) is not ExactTurnSignV1
        or type(lift.evaluation_turn_cosine_squared)
        is not ExactRatioV1
        or type(lift.evaluation_turn_cosine_squared.numerator)
        is not int
        or type(lift.evaluation_turn_cosine_squared.denominator)
        is not int
    ):
        raise ValueError("Density evaluation H-lift tag is invalid")
    covectors = _covectors(context.metric, ideal)
    if not _lift_count_is_feasible(
        lift,
        lift.effective_hidden_edge_count,
    ):
        raise ValueError("Density lifted H is not exactly feasible")
    for count in (
        lift.source_hidden_edge_count,
        lift.minimality_predecessor_hidden_edge_count,
    ):
        if _lift_count_is_feasible(lift, count):
            raise ValueError("Density H-lift minimality witness is false")
    incoming = covectors[0]
    outgoing = covectors[-1]
    dot = sp.cancel(_dual_dot(context.metric, incoming, outgoing))
    cosine_squared = sp.cancel(
        dot * dot
        / (
            _dual_dot(context.metric, incoming, incoming)
            * _dual_dot(context.metric, outgoing, outgoing)
        )
    )
    sign = _density_exact_sign(dot, context.metric)
    expected_sign = (
        ExactTurnSignV1.POSITIVE
        if sign > 0
        else ExactTurnSignV1.NEGATIVE
        if sign < 0
        else ExactTurnSignV1.ZERO
    )
    if (
        cosine_squared.is_Rational is not True
        or lift.evaluation_turn_sign is not expected_sign
        or lift.evaluation_turn_cosine_squared
        != ExactRatioV1(int(cosine_squared.p), int(cosine_squared.q))
    ):
        raise ValueError("Density H-lift exact turn witness is false")


def _verify_evaluation_binding_reasons(
    context: GeometryContext,
    source_context: GeometryContext,
    spec: AngularEnvelopeSpec,
    hidden_by_ordinal,
    ideal,
) -> None:
    *_, source_ideal = _ideal_angular_support_data(source_context, spec)
    if type(spec) is AdaptiveDensityAngularEnvelopeSpecV2:
        try:
            _verify_evaluation_subturn_count_lift(
                context,
                spec,
                ideal,
            )
        except (TypeError, ValueError) as exc:
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
                f"evaluation subturn-count lift is not proven: {exc}",
            ) from exc
        authority = spec.direction_fan_authority
        for ordinal, reason in enumerate(authority.binding_reasons, start=1):
            source_rational = has_rational_density_support_direction(
                source_context.metric,
                source_ideal[ordinal],
            )
            bound_rational = has_rational_density_support_direction(
                context.metric,
                ideal[ordinal],
            )
            reason_matches = (
                reason is None
                and source_rational
                and bound_rational
            ) or (
                reason
                is DirectionBindingReasonV1.SOURCE_DIRECTION_IRRATIONAL
                and not source_rational
            ) or (
                reason
                is DirectionBindingReasonV1.EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL
                and source_rational
                and not bound_rational
            )
            if not reason_matches:
                raise ReferenceGeometryError(
                    ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
                    f"adaptive direction reason is false for ordinal {ordinal}",
                )
        return
    selection = next(
        item
        for item in context.compilation.profile_selection_certificates
        if item.certificate_id == spec.selection_certificate_id
    )
    rational_predicate = (
        has_rational_density_support_direction
        if selection.selection_policy_id
        is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        else has_rational_support_direction
    )
    for ordinal in range(1, spec.resolved_hidden_edge_count + 1):
        support = hidden_by_ordinal[ordinal]
        if type(support) is not CertifiedBoundHiddenSupportSpecV1:
            continue
        certificate = support.direction_binding
        if type(certificate) is not EvaluationGeometryDirectionBindingCertificateV1:
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
                "bound evaluation direction must carry its typed binding reason",
            )
        source_rational = rational_predicate(
            source_context.metric,
            source_ideal[ordinal],
        )
        bound_rational = rational_predicate(
            context.metric,
            ideal[ordinal],
        )
        reason_matches = (
            certificate.binding_reason
            is DirectionBindingReasonV1.SOURCE_DIRECTION_IRRATIONAL
            and not source_rational
        ) or (
            certificate.binding_reason
            is DirectionBindingReasonV1.EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL
            and source_rational
            and not bound_rational
        )
        if not reason_matches:
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_EVALUATION_GEOMETRY_BINDING_INVALID,
                f"bound direction reason is false for {support.hidden_support_id}",
            )


def verify_evaluation_direction_binding_reasons(
    context: GeometryContext,
) -> None:
    """Один раз проверить причины всех bound directions при сборке context."""

    if context.compilation.evaluation_geometry_binding is None:
        return
    source_context = GeometryContext.build(
        replace(
            context.compilation,
            evaluation_geometry_binding=None,
        ),
        context.frame,
        require_evaluation_binding=False,
        density_exact_memo=context.metric._density_exact_memo,
    )
    for spec in context.compilation.envelope_specs:
        if not isinstance(spec, AngularEnvelopeSpec):
            continue
        (
            _,
            _,
            _,
            hidden_by_ordinal,
            _,
            ideal,
        ) = _ideal_angular_support_data(context, spec)
        _verify_evaluation_binding_reasons(
            context,
            source_context,
            spec,
            hidden_by_ordinal,
            ideal,
        )


def _angular_support_data_uncached(
    context: GeometryContext,
    spec: AngularEnvelopeSpec,
):
    """Один раз проверить и материализовать plan-authority опоры Angular."""

    (
        relation,
        sector,
        anchor,
        hidden_by_ordinal,
        support_ids,
        ideal,
    ) = _ideal_angular_support_data(context, spec)
    if type(spec) is AdaptiveDensityAngularEnvelopeSpecV2:
        authority = spec.direction_fan_authority
        try:
            from .adaptive_density_fan import (
                verify_sealed_adaptive_density_fan,
            )

            verify_sealed_adaptive_density_fan(
                authority,
                ideal_count=len(ideal),
            )
        except (
            DirectionBindingCertificateUnproven,
            ValueError,
            TypeError,
        ) as exc:
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
                f"adaptive direction fan is not proven: {exc}",
            ) from exc
        ordered_supports = tuple(
            hidden_by_ordinal[index]
            for index in range(
                1,
                spec.resolved_hidden_edge_count + 1,
            )
        )
        if any(
            type(item) is not AdaptiveBoundHiddenSupportSpecV2
            or item.direction_fan_authority_id != authority.authority_id
            or item.bound_primitive_integer_vector
            != authority.bound_primitive_integer_vectors[item.ordinal - 1]
            for item in ordered_supports
        ):
            raise ReferenceGeometryError(
                ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
                "adaptive support does not match its sealed fan authority",
            )
        normals = list(ideal)
        for ordinal, vector in enumerate(
            authority.bound_primitive_integer_vectors,
            start=1,
        ):
            normals[ordinal] = bound_unit_normal_from_vector(
                context.metric,
                vector,
            )
        return relation, anchor, support_ids, tuple(normals)
    certificates = tuple(
        (
            hidden_by_ordinal[ordinal].direction_binding
            if isinstance(
                hidden_by_ordinal[ordinal],
                CertifiedBoundHiddenSupportSpecV1,
            )
            else None
        )
        for ordinal in range(1, spec.resolved_hidden_edge_count + 1)
    )
    selection = next(
        item
        for item in context.compilation.profile_selection_certificates
        if item.certificate_id == spec.selection_certificate_id
    )
    density_contract = huber_density_value_contract(
        selection.max_subturn_value_id
    )
    try:
        if (
            selection.selection_policy_id
            is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
            and density_contract is not None
        ):
            verify_huber_density_direction_bindings(
                context.metric,
                ideal,
                sector.turn_orientation,
                certificates,
                density_contract[0],
            )
        elif spec.resolved_hidden_edge_count == 1:
            _verify_k1_recipe_direction_bindings(
                context.metric,
                ideal[0],
                ideal[-1],
                ideal,
                sector.turn_orientation,
                certificates,
            )
        else:
            verify_direction_bindings(
                context.metric,
                ideal,
                sector.turn_orientation,
                certificates,
            )
    except DirectionBindingCertificateUnproven as exc:
        raise ReferenceGeometryError(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            f"direction binding certificate is not proven: {exc}",
        ) from exc
    normals = list(ideal)
    for ordinal, certificate in enumerate(certificates, start=1):
        if certificate is not None:
            normals[ordinal] = (
                bound_unit_normal_from_vector(
                    context.metric,
                    certificate.bound_primitive_integer_vector,
                )
                if density_contract is not None
                and selection.selection_policy_id
                is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
                else bound_unit_normal(context.metric, certificate)
            )
    return relation, anchor, support_ids, tuple(normals)


def angular_support_data(context: GeometryContext, spec: AngularEnvelopeSpec):
    """Вернуть уже проверенные опоры; verification не входит в hot path."""

    # Spec id не удостоверяет содержимое подделанной immutable записи.
    key = spec
    cached = context.angular_support_cache.get(key)
    if cached is None:
        cached = _angular_support_data_uncached(context, spec)
        context.angular_support_cache[key] = cached
    return cached


def seal_angular_support_cache(context: GeometryContext) -> None:
    """Одна bounded verification всех Angular-властей при сборке context."""

    for spec in sorted(
        (
            item
            for item in context.compilation.envelope_specs
            if isinstance(item, AngularEnvelopeSpec)
        ),
        key=lambda item: item.envelope_spec_id.value,
    ):
        angular_support_data(context, spec)


def evaluate_angular_envelope(
    context: GeometryContext,
    spec: AngularEnvelopeSpec,
    alpha_value: LocalLengthV1,
    effective_alpha: sp.Expr,
) -> ReferenceEnvelopeInstanceV1:
    relation, anchor, support_ids, normals = angular_support_data(context, spec)
    instance_id = stable_id(
        "envelope-instance",
        spec.envelope_spec_id,
        ExactScalar.from_value(effective_alpha).expression,
    )
    base_provenance = merge_provenance(
        context.provenance_by_spec_id[spec.envelope_spec_id.value],
        make_reference_provenance(
            envelope_instance_ids=frozenset({instance_id}),
            support_ids=frozenset(support_ids),
        ),
    )
    moving_lines = tuple(
        context.metric.line_through_point_g(
            support_id,
            anchor,
            normal,
            ConstructionCertificate(
                kind=ConstructionKind.SUPPORT_SUPPORT_INTERSECTION,
                source_vertex_ids=frozenset({relation.source_vertex_id.value}),
                support_ids=frozenset({support_id}),
            ),
        ).at_alpha(effective_alpha)
        for support_id, normal in zip(support_ids, normals, strict=True)
    )
    start = context.metric.offset_support_g(anchor, normals[0], effective_alpha)
    end = context.metric.offset_support_g(anchor, normals[-1], effective_alpha)
    intersections = tuple(
        support_intersection(left, right)
        for left, right in zip(moving_lines, moving_lines[1:])
    )
    profile_points = (start, *intersections, end)
    anchor_cert = source_vertex_certificate(relation.source_vertex_id)
    point_certificates = [
        support_vertex_certificate(support_ids[0], stable_id("angular-terminal", spec.envelope_spec_id, "incoming"))
    ]
    point_certificates.extend(
        support_vertex_certificate(left, right)
        for left, right in zip(support_ids, support_ids[1:])
    )
    point_certificates.append(
        support_vertex_certificate(support_ids[-1], stable_id("angular-terminal", spec.envelope_spec_id, "outgoing"))
    )
    segments = [
        make_segment(
            stable_id("angular-edge", instance_id, "incoming-terminal"),
            anchor,
            start,
            support_ids=frozenset({support_ids[0]}),
            provenance=base_provenance,
            start_certificates=frozenset({anchor_cert}),
            end_certificates=frozenset({point_certificates[0]}),
        )
    ]
    for ordinal, (left, right) in enumerate(
        zip(profile_points, profile_points[1:]), start=0
    ):
        support_id = support_ids[ordinal]
        segments.append(
            make_segment(
                stable_id("angular-profile-edge", instance_id, ordinal),
                left,
                right,
                support_ids=frozenset({support_id}),
                provenance=merge_provenance(
                    base_provenance,
                    make_reference_provenance(
                        support_ids=frozenset({support_id})
                    ),
                ),
                start_certificates=frozenset({point_certificates[ordinal]}),
                end_certificates=frozenset({point_certificates[ordinal + 1]}),
            )
        )
    segments.append(
        make_segment(
            stable_id("angular-edge", instance_id, "outgoing-terminal"),
            end,
            anchor,
            support_ids=frozenset({support_ids[-1]}),
            provenance=base_provenance,
            start_certificates=frozenset({point_certificates[-1]}),
            end_certificates=frozenset({anchor_cert}),
        )
    )
    region = make_region(
        stable_id("angular-region", instance_id),
        tuple(segments),
        instance_id=instance_id,
        spec_id=spec.envelope_spec_id.value,
    )
    return ReferenceEnvelopeInstanceV1(
        envelope_instance_id=instance_id,
        envelope_spec_id=spec.envelope_spec_id.value,
        envelope_variant="AngularEnvelope",
        requested_alpha=alpha_value,
        effective_alpha=ExactScalar.from_value(effective_alpha),
        regions=(region,) if region is not None else (),
        exposed_segments=region.outer.segments if region is not None else (),
        provenance=base_provenance,
    )
