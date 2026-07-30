"""Структурные проверки sealed Density V2 без геометрических predicates."""

from __future__ import annotations

from math import gcd

from .contracts.envelopes import (
    AngularEnvelopeSpec,
    AdaptiveBoundHiddenSupportDirectionLawV2,
    AdaptiveBoundHiddenSupportSpecV2,
    AdaptiveDensityAngularEnvelopeSpecV2,
    EvaluationGeometrySubturnCountLiftLawV1,
    EvaluationGeometrySubturnCountLiftV1,
    ExactTurnSignV1,
)
from .contracts.events import InitialFrontFeatureKind
from .ids import HiddenSupportId
from .contracts.request import AngularProfileSelectionPolicyId
from .numeric import ExactRatioV1
from ._density_policy import huber_density_value_contract


_LIFT_PREDICATES = frozenset(
    {
        "SOURCE_SELECTION_CERTIFICATE_IMMUTABLE",
        "SOURCE_COUNT_EXACTLY_INFEASIBLE_IN_EVALUATION_GEOMETRY",
        "EFFECTIVE_COUNT_EXACTLY_FEASIBLE_IN_EVALUATION_GEOMETRY",
        "EFFECTIVE_COUNT_IS_MINIMAL",
    }
)


def adaptive_density_effective_hidden_count(
    spec,
    selection,
) -> tuple[int, tuple[str, ...]]:
    """Структурно разрешить effective H; exact geometry проверит consumer."""

    source_count = selection.resolved_hidden_edge_count
    if type(spec) is not AdaptiveDensityAngularEnvelopeSpecV2:
        return source_count, ()
    lift = spec.evaluation_subturn_count_lift
    if lift is None:
        return source_count, ()
    q_contract = huber_density_value_contract(
        selection.max_subturn_value_id
    )

    valid = (
        type(lift) is EvaluationGeometrySubturnCountLiftV1
        and lift.lift_law
        is EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1
        and selection.selection_policy_id
        is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        and q_contract is not None
        and lift.source_selection_certificate_id
        == selection.certificate_id
        and type(lift.source_hidden_edge_count) is int
        and type(lift.effective_hidden_edge_count) is int
        and type(lift.max_subturn_q) is int
        and type(lift.minimality_predecessor_hidden_edge_count) is int
        and lift.source_hidden_edge_count == source_count
        and lift.effective_hidden_edge_count
        == spec.resolved_hidden_edge_count
        and source_count < lift.effective_hidden_edge_count < q_contract[0]
        and lift.max_subturn_q == q_contract[0]
        and lift.minimality_predecessor_hidden_edge_count
        == lift.effective_hidden_edge_count - 1
        and type(lift.evaluation_turn_sign) is ExactTurnSignV1
        and type(lift.evaluation_turn_cosine_squared) is ExactRatioV1
        and type(
            lift.evaluation_turn_cosine_squared.numerator
        ) is int
        and type(
            lift.evaluation_turn_cosine_squared.denominator
        ) is int
        and lift.evaluation_turn_cosine_squared.denominator > 0
        and 0 <= lift.evaluation_turn_cosine_squared.numerator <= (
            lift.evaluation_turn_cosine_squared.denominator
        )
        and gcd(
            lift.evaluation_turn_cosine_squared.numerator,
            lift.evaluation_turn_cosine_squared.denominator,
        )
        == 1
        and lift.proven_predicates == _LIFT_PREDICATES
    )
    return (
        (lift.effective_hidden_edge_count, ())
        if valid
        else (
            source_count,
            ("evaluation subturn-count lift is not structurally authorized",),
        )
    )


def angular_hidden_feature_id_sets(plan):
    """Identity-множества supports/features для двусторонней полноты."""

    hidden = {
        support.hidden_support_id
        for spec in plan.envelope_specs
        if isinstance(spec, AngularEnvelopeSpec)
        for support in spec.hidden_supports
    }
    features = {
        item.source_id
        for item in plan.initial_front_spec.support_features
        if item.kind
        is InitialFrontFeatureKind.ANGULAR_HIDDEN_SUPPORT
        and isinstance(item.source_id, HiddenSupportId)
    }
    return hidden, features


def adaptive_density_structure_errors(
    spec,
) -> tuple[tuple[tuple[str, ...], str], ...]:
    """Вернуть structural V2-ошибки как suffix пути и сообщение."""

    if type(spec) is not AdaptiveDensityAngularEnvelopeSpecV2:
        return ()
    errors = []
    lift = spec.evaluation_subturn_count_lift
    if lift is not None and (
        type(lift) is not EvaluationGeometrySubturnCountLiftV1
        or lift.effective_hidden_edge_count
        != spec.resolved_hidden_edge_count
        or lift.source_selection_certificate_id
        != spec.selection_certificate_id
    ):
        errors.append(
            (
                ("evaluation_subturn_count_lift",),
                "adaptive H-lift must bind source selection and effective H",
            )
        )
    valid_supports = []
    for support in spec.hidden_supports:
        ordinal = getattr(support, "ordinal", "?")
        suffix = ("hidden_supports", str(ordinal))
        if type(support) is not AdaptiveBoundHiddenSupportSpecV2:
            errors.append(
                (
                    suffix,
                    "adaptive fan requires an exact V2 support record",
                )
            )
            continue
        valid_supports.append(support)
        if (
            support.direction_law
            is not AdaptiveBoundHiddenSupportDirectionLawV2.ADAPTIVE_MINIMAL_RATIONAL_FAN_V2
        ):
            errors.append(
                (
                    (*suffix, "direction_law"),
                    "adaptive support must reference the sealed V2 fan law",
                )
            )
        vector = support.bound_primitive_integer_vector
        if (
            len(vector) != 2
            or any(type(value) is not int for value in vector)
            or vector == (0, 0)
            or gcd(abs(vector[0]), abs(vector[1])) != 1
        ):
            errors.append(
                (
                    suffix,
                    "adaptive direction must be a primitive integer vector",
                )
            )
    if len(valid_supports) != len(spec.hidden_supports) or any(
        len(item.bound_primitive_integer_vector) != 2
        or any(
            type(value) is not int
            for value in item.bound_primitive_integer_vector
        )
        for item in valid_supports
    ):
        return tuple(errors)
    authority = spec.direction_fan_authority
    ordered = tuple(
        item.bound_primitive_integer_vector
        for item in sorted(
            valid_supports,
            key=lambda item: item.ordinal,
        )
    )
    if len(authority.ordinal_windows) != len(ordered):
        errors.append(
            (
                ("direction_fan_authority", "ordinal_windows"),
                "adaptive fan window cardinality must match its supports",
            )
        )
        return tuple(errors)
    if (
        any(
            item.direction_fan_authority_id != authority.authority_id
            for item in spec.hidden_supports
        )
        or ordered != authority.bound_primitive_integer_vectors
        or authority.minimal_common_height
        != max(
            (
                abs(
                    vector[0]
                    if window.use_x_denominator
                    else vector[1]
                )
                for vector, window in zip(
                    ordered,
                    authority.ordinal_windows,
                    strict=True,
                )
            ),
            default=0,
        )
    ):
        errors.append(
            (
                ("direction_fan_authority",),
                "adaptive supports must reference one matching minimal-height authority",
            )
        )
    return tuple(errors)
