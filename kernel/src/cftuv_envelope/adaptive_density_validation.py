"""Структурные проверки sealed Density V2 без геометрических predicates."""

from __future__ import annotations

from fractions import Fraction
from math import gcd

from .contracts.envelopes import (
    AngularEnvelopeSpec,
    AdaptiveBoundHiddenSupportDirectionLawV2,
    AdaptiveBoundHiddenSupportSpecV2,
    AdaptiveDensityAngularEnvelopeSpecV2,
    AdaptiveRationalFanOrdinalWindowAtlasV1,
    CanonicalFanRaysLawV1,
    CanonicalRationalRotationFanAuthorityV1,
    EvaluationGeometrySubturnCountLiftLawV1,
    EvaluationGeometrySubturnCountLiftV1,
    ExactTurnSignV1,
)
from .contracts.events import InitialFrontFeatureKind
from .ids import HiddenSupportId
from .contracts.request import AngularProfileSelectionPolicyId
from .numeric import ExactRatioV1
from ._density_policy import (
    CANONICAL_FAN_RAYS_PREDICATES,
    EVALUATION_SUBTURN_LIFT_PREDICATES,
    canonical_rotation_rays,
    huber_density_value_contract,
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
        and type(lift.lift_law) is EvaluationGeometrySubturnCountLiftLawV1
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
        and lift.proven_predicates
        == EVALUATION_SUBTURN_LIFT_PREDICATES[lift.lift_law]
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


def _canonical_rotation_structure_errors(spec, supports):
    """Структура власти поворотов: типы, закон, предикаты, тройка и ссылки опор.

    Геометрию (равенство лучей таблице, подшаг) проверяет эталон пересчётом;
    здесь — только то, что видно из самой записи.
    """

    authority = spec.direction_fan_authority
    path = ("direction_fan_authority",)
    ordered = tuple(
        item.bound_primitive_integer_vector
        for item in sorted(supports, key=lambda item: item.ordinal)
    )
    problems = []
    if (
        authority.ray_law
        is not CanonicalFanRaysLawV1.CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1
        or authority.proven_predicates != CANONICAL_FAN_RAYS_PREDICATES
    ):
        problems.append("canonical rotation fan must name its law and predicates")
    if (
        type(authority.hidden_edge_count) is not int
        or authority.hidden_edge_count != spec.resolved_hidden_edge_count
        or type(authority.max_subturn_q) is not int
        or authority.max_subturn_q not in range(2, 7)
        or authority.selection_certificate_id != spec.selection_certificate_id
    ):
        problems.append("canonical rotation fan must describe its own spec")
    canonical = authority.canonical_reflex_excess_over_pi
    if (
        type(canonical.numerator) is not int
        or type(canonical.denominator) is not int
        or canonical.denominator <= 0
        or canonical_rotation_rays(
            Fraction(canonical.numerator, canonical.denominator),
            authority.hidden_edge_count + 1,
            authority.max_subturn_q,
        )
        != authority.ray_rotation_pairs
    ):
        problems.append(
            "canonical rotation fan must carry the table row of its angle"
        )
    if (
        len(ordered) != len(authority.bound_primitive_integer_vectors)
        or ordered != authority.bound_primitive_integer_vectors
        or any(
            item.direction_fan_authority_id != authority.authority_id
            for item in supports
        )
    ):
        problems.append("canonical rotation supports must reference one matching authority")
    if any(
        len(vector) != 2
        or any(type(value) is not int for value in vector)
        or vector == (0, 0)
        or gcd(abs(vector[0]), abs(vector[1])) != 1
        for vector in authority.bound_primitive_integer_vectors
    ):
        problems.append("canonical rotation rays must be primitive integer vectors")
    return [(path, message) for message in problems]


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
    canonical_rays = (
        type(spec.direction_fan_authority)
        is CanonicalRationalRotationFanAuthorityV1
    )
    support_errors, valid_supports = _support_record_errors(spec, canonical_rays)
    errors.extend(support_errors)
    if len(valid_supports) != len(spec.hidden_supports) or any(
        len(item.bound_primitive_integer_vector) != 2
        or any(
            type(value) is not int
            for value in item.bound_primitive_integer_vector
        )
        for item in valid_supports
    ):
        return tuple(errors)
    if canonical_rays:
        errors.extend(_canonical_rotation_structure_errors(spec, valid_supports))
        return tuple(errors)
    errors.extend(_atlas_structure_errors(spec, valid_supports))
    return tuple(errors)


def _support_record_errors(spec, canonical_rays: bool):
    """Ошибки записей опор и сами годные записи: тип, закон власти, примитивность."""

    expected_law = (
        AdaptiveBoundHiddenSupportDirectionLawV2.CANONICAL_RATIONAL_ROTATION_FAN_V1
        if canonical_rays
        else AdaptiveBoundHiddenSupportDirectionLawV2.ADAPTIVE_MINIMAL_RATIONAL_FAN_V2
    )
    errors = []
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
        if support.direction_law is not expected_law:
            errors.append(
                (
                    (*suffix, "direction_law"),
                    "adaptive support must reference the law of its fan authority",
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
    return errors, valid_supports


def _atlas_structure_errors(spec, valid_supports):
    """Структура атласной власти: окна, ссылки опор и минимальная общая высота."""

    errors = []
    authority = spec.direction_fan_authority
    uses_atlas = any(
        type(item) is AdaptiveRationalFanOrdinalWindowAtlasV1
        for item in authority.ordinal_windows
    )
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
        return errors
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
                    max(vector, key=abs)
                    if uses_atlas
                    else (
                        vector[0]
                        if window.use_x_denominator
                        else vector[1]
                    )
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
    return errors
