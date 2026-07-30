"""LINEAR_REFLEX_EQUAL_V1 construction: frozen legacy plus Density A fan."""

from __future__ import annotations

from dataclasses import replace

import sympy as sp
from mpmath import iv

from ..contracts.analysis import TurnOrientation
from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    CertifiedBoundHiddenSupportDirectionLawV1,
    CertifiedBoundHiddenSupportSpecV1,
    DirectionBindingReasonV1,
    EvaluationGeometryDirectionBindingCertificateV1,
    HiddenSupportDirectionLaw,
    HiddenSupportSpecV1,
    StripEnvelopeSpec,
)
from ..contracts.request import (
    AngularProfileSelectionPolicyId,
)
from .._density_policy import huber_density_value_contract
from ..numeric import LocalLengthV1
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
    has_rational_support_direction,
    verify_direction_bindings,
    verify_huber_density_direction_bindings,
)
from .planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactPlanarVector,
    ExactScalar,
    IntervalEnclosureUnsupported,
    exact_normalize,
    exact_sign,
    interval_enclosure,
    point_sub,
    support_intersection,
)
from .metric import ExactPlanarMetric
from .provenance import make_reference_provenance, merge_provenance


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
) -> ExactPlanarVector:
    """Записать bounded Density-expression без global factor-canonicalizer."""

    def scalar(expression: sp.Expr) -> ExactScalar:
        if expression.is_Rational:
            return ExactScalar.from_value(expression)
        return ExactScalar(sp.srepr(expression))

    return ExactPlanarVector(scalar(x), scalar(y))


def _density_dot_expression(
    metric: ExactPlanarMetric,
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> sp.Expr:
    """Dual-free Gram dot без generic normalization/factor."""

    lx, ly = left.expressions()
    rx, ry = right.expressions()
    return (
        lx * (metric.gram[0][0] * rx + metric.gram[0][1] * ry)
        + ly * (metric.gram[1][0] * rx + metric.gram[1][1] * ry)
    )


def _density_exact_sign(expression: sp.Expr) -> int:
    """Знак Density-факта: rational либо строгая interval-сертификация."""

    if expression.is_Rational:
        return (expression.p > 0) - (expression.p < 0)
    if expression == 0 or expression.is_zero is True:
        return 0
    saved = iv.prec
    iv.prec = 160
    try:
        enclosure = interval_enclosure(expression)
    except (
        IntervalEnclosureUnsupported,
        ArithmeticError,
        TypeError,
        ValueError,
    ):
        enclosure = None
    finally:
        iv.prec = saved
    if enclosure is not None:
        if enclosure.a > 0:
            return 1
        if enclosure.b < 0:
            return -1
    raise ReferenceGeometryError(
        ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
        "Density A exact sign is not certified without generic factorization",
    )


def _density_unit_from_squared(
    vector: ExactPlanarVector,
    squared: sp.Expr,
) -> ExactPlanarVector:
    if _density_exact_sign(squared) <= 0:
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "Density A support direction has non-positive Gram norm",
        )
    x, y = vector.expressions()
    length = sp.sqrt(squared)
    return _density_exact_vector(x / length, y / length)


def _density_left_unit_normal(
    metric: ExactPlanarMetric,
    tangent: ExactPlanarVector,
) -> ExactPlanarVector:
    tx, ty = tangent.expressions()
    covector_x = metric.gram[0][0] * tx + metric.gram[0][1] * ty
    covector_y = metric.gram[1][0] * tx + metric.gram[1][1] * ty
    sign = metric.owner_orientation_sign
    raw = _density_exact_vector(
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
            incoming.x.as_expr() * outgoing.y.as_expr()
            - incoming.y.as_expr() * outgoing.x.as_expr()
        )
    )
    if _density_exact_sign(raw_cross) != orientation_sign:
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
    )
    outgoing = _density_unit_from_squared(
        outgoing,
        outgoing_squared,
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
        _density_exact_sign(cosine_squared) < 0
        or _density_exact_sign(cosine_squared - 1) >= 0
    ):
        raise ReferenceGeometryError(
            ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
            "Density A requires a strict principal turn in (0, pi)",
        )
    turn_sign = _density_exact_sign(raw_dot)
    cosine_total = turn_sign * sp.sqrt(cosine_squared)
    sine_squared = 1 - cosine_squared
    principal_turn = sp.atan2(sp.sqrt(sine_squared), cosine_total)
    subturn_count = count + 1
    ix, iy = incoming.expressions()
    lx, ly = _density_left_unit_normal(metric, incoming).expressions()
    hidden = []
    for ordinal in range(1, subturn_count):
        angle = sp.Rational(ordinal, subturn_count) * principal_turn
        cosine = sp.cos(angle)
        sine = orientation_sign * sp.sin(angle)
        normal = _density_exact_vector(
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
    relation = next(
        item
        for item in context.snapshot.corner_relations
        if item.corner_relation_id == spec.source_relation_id
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
    return (
        relation,
        sector,
        anchor,
        hidden_by_ordinal,
        tuple(support_ids),
        ideal,
    )


def _verify_evaluation_binding_reasons(
    context: GeometryContext,
    source_context: GeometryContext,
    spec: AngularEnvelopeSpec,
    hidden_by_ordinal,
    ideal,
) -> None:
    *_, source_ideal = _ideal_angular_support_data(source_context, spec)
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
        source_rational = has_rational_support_direction(
            source_context.metric,
            source_ideal[ordinal],
        )
        bound_rational = has_rational_support_direction(
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


def angular_support_data(context: GeometryContext, spec: AngularEnvelopeSpec):
    """Опоры Angular из одной plan-authority записи, с перепроверкой binding."""

    (
        relation,
        sector,
        anchor,
        hidden_by_ordinal,
        support_ids,
        ideal,
    ) = _ideal_angular_support_data(context, spec)
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
            normals[ordinal] = bound_unit_normal(context.metric, certificate)
    return relation, anchor, support_ids, tuple(normals)


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
