"""Замкнутые внутренние policy/numeric authority Huber Density A."""

from __future__ import annotations

from mpmath import iv
import sympy as sp

from .contracts.envelopes import (
    AdmissibilityUpperBound,
    AngularProfileSelectionCertificateV1,
    HuberDensitySelectionIntervalCertificateV1,
    IntervalBoundKind,
    MinimalityLowerBound,
    SelectionIntervalCertificateV1,
    SelectionLaw,
)
from .contracts.request import (
    AngularProfileSelectionPolicyId,
    DecalRequestV1,
    MaxSubturnParameterId,
    MaxSubturnValueId,
)
from .numeric import ExactAngleSymbol


class DensityIntervalEnclosureUnsupported(Exception):
    """Узел вне закрытой Density-only interval whitelist."""


def density_interval_enclosure(expression: sp.Expr):
    """Outward enclosure только для точных bounded Density-выражений."""

    if expression.is_Integer:
        return iv.mpf(int(expression))
    if expression.is_Rational:
        return iv.mpf(int(expression.p)) / iv.mpf(int(expression.q))
    if expression is sp.pi:
        return iv.pi
    if expression.is_Add:
        total = iv.mpf(0)
        for term in expression.args:
            total = total + density_interval_enclosure(term)
        return total
    if expression.is_Mul:
        product = iv.mpf(1)
        for term in expression.args:
            product = product * density_interval_enclosure(term)
        return product
    if expression.is_Pow:
        base, exponent = expression.args
        enclosure = density_interval_enclosure(base)
        if exponent.is_Integer:
            return enclosure ** int(exponent)
        if exponent.is_Rational and exponent.q == 2:
            root = iv.sqrt(enclosure)
            return root if exponent.p == 1 else root ** int(exponent.p)
    if expression.func is sp.sin:
        return iv.sin(density_interval_enclosure(expression.args[0]))
    if expression.func is sp.cos:
        return iv.cos(density_interval_enclosure(expression.args[0]))
    if expression.func is sp.atan:
        return iv.atan2(
            density_interval_enclosure(expression.args[0]),
            iv.mpf(1),
        )
    if expression.func is sp.atan2:
        y, x = expression.args
        return iv.atan2(
            density_interval_enclosure(y),
            density_interval_enclosure(x),
        )
    raise DensityIntervalEnclosureUnsupported(str(expression))


def huber_density_value_contract(
    value_id: MaxSubturnValueId,
) -> tuple[int, ExactAngleSymbol] | None:
    """Вернуть `(q, pi/q)` только для закрытого Density A value-set."""

    if value_id is MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1:
        return 2, ExactAngleSymbol.PI_OVER_2
    if value_id is MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1:
        return 3, ExactAngleSymbol.PI_OVER_3
    if value_id is MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1:
        return 4, ExactAngleSymbol.PI_OVER_4
    if value_id is MaxSubturnValueId.LINEAR_REFLEX_DENSITY_3_V1:
        return 5, ExactAngleSymbol.PI_OVER_5
    if value_id is MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1:
        return 6, ExactAngleSymbol.PI_OVER_6
    return None


def angular_request_policy_mismatches(
    request: DecalRequestV1,
) -> tuple[str, ...]:
    """Имена полей несовместимой policy/value/exact-angle тройки."""

    policy = request.angular_profile_selection_policy_id
    if policy is AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1:
        checks = (
            (
                request.max_subturn_parameter_id
                is MaxSubturnParameterId.LINEAR_REFLEX_MAX_SUBTURN_V1,
                "max_subturn_parameter_id",
            ),
            (
                request.max_subturn_value_id
                is MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1,
                "max_subturn_value_id",
            ),
            (
                request.max_subturn_exact_value.symbol
                is ExactAngleSymbol.PI_OVER_3,
                "max_subturn_exact_value",
            ),
        )
    elif (
        policy
        is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
    ):
        value_contract = huber_density_value_contract(
            request.max_subturn_value_id
        )
        checks = (
            (
                request.max_subturn_parameter_id
                is MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1,
                "max_subturn_parameter_id",
            ),
            (value_contract is not None, "max_subturn_value_id"),
            (
                value_contract is not None
                and request.max_subturn_exact_value.symbol
                is value_contract[1],
                "max_subturn_exact_value",
            ),
        )
    else:
        return ("angular_profile_selection_policy_id",)
    return tuple(field for valid, field in checks if not valid)


def selection_certificate_contract_error(
    certificate: AngularProfileSelectionCertificateV1,
) -> str | None:
    """Проверить tag/law/cardinality-связь без чтения angle geometry."""

    hidden_count = certificate.resolved_hidden_edge_count
    interval = certificate.selection_interval_certificate
    if (
        certificate.selection_policy_id
        is AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1
    ):
        valid = (
            certificate.max_subturn_value_id
            is MaxSubturnValueId.LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1
            and certificate.selection_law
            is SelectionLaw.MIN_K_FOR_MAX_SUBTURN
            and certificate.minimality_lower_bound
            is MinimalityLowerBound.K_ZERO_OR_STRICT_LOWER
            and certificate.admissibility_upper_bound
            is AdmissibilityUpperBound.CLOSED_UPPER
            and type(interval) is SelectionIntervalCertificateV1
            and interval.lower_bound_kind is IntervalBoundKind.OPEN
            and interval.upper_bound_kind is IntervalBoundKind.CLOSED
            and interval.lower_bound_integer == hidden_count
            and interval.upper_bound_integer == hidden_count + 1
        )
        return (
            None
            if valid
            else "legacy certificate must encode open k < ratio <= k+1"
        )
    if (
        certificate.selection_policy_id
        is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
    ):
        value_contract = huber_density_value_contract(
            certificate.max_subturn_value_id
        )
        valid = (
            value_contract is not None
            and certificate.selection_law
            is SelectionLaw.HUBER_EMANATED_DENSITY_FLOOR_V1
            and certificate.minimality_lower_bound
            is MinimalityLowerBound.HUBER_DENSITY_BUCKET_OPEN_LOWER
            and certificate.admissibility_upper_bound
            is AdmissibilityUpperBound.HUBER_DENSITY_BUCKET_CLOSED_UPPER
            and type(interval)
            is HuberDensitySelectionIntervalCertificateV1
            and interval.q == value_contract[0]
            and 1 <= interval.bucket_c <= interval.q
            and interval.lower_bound_kind is IntervalBoundKind.OPEN
            and interval.upper_bound_kind is IntervalBoundKind.CLOSED
            and interval.lower_bound_numerator == interval.bucket_c - 1
            and interval.upper_bound_numerator == interval.bucket_c
            and hidden_count == max(1, interval.bucket_c - 1)
            and hidden_count <= 5
        )
        return (
            None
            if valid
            else (
                "Density A certificate must encode "
                "(C-1)/q < u <= C/q and H=max(1,C-1)"
            )
        )
    return "unsupported angular selection policy"
