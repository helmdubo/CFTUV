"""Замкнутые внутренние policy/numeric authority Huber Density A и B."""

from __future__ import annotations

from fractions import Fraction

from mpmath import iv
import sympy as sp

from .contracts.envelopes import (
    AdmissibilityUpperBound,
    AngularProfileSelectionCertificateV1,
    HuberDensityMidpointSelectionIntervalCertificateV1,
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


# Одна таблица «политика Density -> её parameter id». Никакой другой модуль не
# перечисляет плотностные политики списком: расхождение перечней было бы
# молчаливым отказом целой политики на одной из ступеней.
HUBER_DENSITY_PARAMETER_IDS = {
    AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1: (
        MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1
    ),
    AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1: (
        MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_B_V1
    ),
}


def is_huber_density_policy(policy_id) -> bool:
    """Идёт ли запрос по одной из плотностных политик (A или B)."""

    return policy_id in HUBER_DENSITY_PARAMETER_IDS


def is_midpoint_density_policy(policy_id) -> bool:
    """Стоят ли границы счётных ячеек на серединах (закон B)."""

    return (
        policy_id
        is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1
    )


def density_cell_threshold_turn(
    hidden_count: int,
    q: int,
    *,
    midpoint: bool,
) -> Fraction:
    """Верхняя граница ячейки счёта `H` в долях `pi` — ОДИН закон обеих ступеней.

    Закон A: `theta <= (H+1)*pi/q` — жёсткий потолок подповорота `pi/q`.
    Закон B: `theta <= (2H+3)*pi/(2q)` — та же ячейка, сдвинутая на полполосы,
    так что её СЕРЕДИНА `(H+1)*pi/q` даёт подповорот ровно `pi/q`.
    """

    if midpoint:
        return Fraction(2 * hidden_count + 3, 2 * q)
    return Fraction(hidden_count + 1, q)


def density_cell_lower_turn(
    hidden_count: int,
    q: int,
    *,
    midpoint: bool,
) -> Fraction:
    """Открытая нижняя граница ячейки счёта `H` в долях `pi`."""

    if not midpoint:
        return Fraction(hidden_count, q)
    if hidden_count <= 0:
        return Fraction(0)
    return Fraction(2 * hidden_count + 1, 2 * q)


def huber_density_certificate_bounds(interval):
    """Границы ячейки сертификата в долях `pi` — по его собственному тегу."""

    if type(interval) is HuberDensitySelectionIntervalCertificateV1:
        denominator = interval.q
    elif (
        type(interval) is HuberDensityMidpointSelectionIntervalCertificateV1
    ):
        denominator = 2 * interval.q
    else:
        return None
    return (
        Fraction(interval.lower_bound_numerator, denominator),
        Fraction(interval.upper_bound_numerator, denominator),
    )


class DensityIntervalEnclosureUnsupported(Exception):
    """Узел вне закрытой Density-only interval whitelist."""


def density_interval_enclosure(expression: sp.Expr, memo=None):
    """Outward enclosure только для точных bounded Density-выражений."""

    if memo is not None and expression in memo:
        return memo[expression]
    if expression.is_Integer:
        result = iv.mpf(int(expression))
    elif expression.is_Rational:
        result = iv.mpf(int(expression.p)) / iv.mpf(int(expression.q))
    elif expression is sp.pi:
        result = iv.pi
    elif expression.is_Add:
        total = iv.mpf(0)
        for term in expression.args:
            total = total + density_interval_enclosure(term, memo)
        result = total
    elif expression.is_Mul:
        product = iv.mpf(1)
        for term in expression.args:
            product = product * density_interval_enclosure(term, memo)
        result = product
    elif expression.is_Pow:
        base, exponent = expression.args
        enclosure = density_interval_enclosure(base, memo)
        if exponent.is_Integer:
            result = enclosure ** int(exponent)
        elif exponent.is_Rational and exponent.q == 2:
            root = iv.sqrt(enclosure)
            result = (
                root
                if exponent.p == 1
                else root ** int(exponent.p)
            )
        else:
            raise DensityIntervalEnclosureUnsupported(str(expression))
    elif expression.func is sp.sin:
        result = iv.sin(density_interval_enclosure(expression.args[0], memo))
    elif expression.func is sp.cos:
        result = iv.cos(density_interval_enclosure(expression.args[0], memo))
    elif expression.func is sp.atan:
        result = iv.atan2(
            density_interval_enclosure(expression.args[0], memo),
            iv.mpf(1),
        )
    elif expression.func is sp.atan2:
        y, x = expression.args
        result = iv.atan2(
            density_interval_enclosure(y, memo),
            density_interval_enclosure(x, memo),
        )
    else:
        raise DensityIntervalEnclosureUnsupported(str(expression))
    if memo is not None:
        memo[expression] = result
    return result


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
    elif policy in HUBER_DENSITY_PARAMETER_IDS:
        value_contract = huber_density_value_contract(
            request.max_subturn_value_id
        )
        checks = (
            (
                request.max_subturn_parameter_id
                is HUBER_DENSITY_PARAMETER_IDS[policy],
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
    if (
        certificate.selection_policy_id
        is AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_B_V1
    ):
        value_contract = huber_density_value_contract(
            certificate.max_subturn_value_id
        )
        cell = getattr(interval, "cell_hidden_edge_count", None)
        valid = (
            value_contract is not None
            and certificate.selection_law
            is SelectionLaw.HUBER_EMANATED_DENSITY_MIDPOINT_V1
            and certificate.minimality_lower_bound
            is MinimalityLowerBound.HUBER_DENSITY_MIDPOINT_CELL_OPEN_LOWER
            and certificate.admissibility_upper_bound
            is AdmissibilityUpperBound.HUBER_DENSITY_MIDPOINT_CELL_CLOSED_UPPER
            and type(interval)
            is HuberDensityMidpointSelectionIntervalCertificateV1
            and interval.q == value_contract[0]
            and 0 <= cell <= interval.q - 1
            and interval.lower_bound_kind is IntervalBoundKind.OPEN
            and interval.upper_bound_kind is IntervalBoundKind.CLOSED
            and interval.lower_bound_numerator
            == (0 if cell == 0 else 2 * cell + 1)
            and interval.upper_bound_numerator == 2 * cell + 3
            and hidden_count == cell
            and hidden_count <= 5
        )
        return (
            None
            if valid
            else (
                "Density B certificate must encode "
                "(2H+1)/(2q) < u <= (2H+3)/(2q) and H=cell"
            )
        )
    return "unsupported angular selection policy"
