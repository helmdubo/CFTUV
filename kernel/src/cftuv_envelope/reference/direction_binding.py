"""Сертифицированная рациональная привязка направлений скрытых опор."""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction
from math import gcd

from mpmath import iv, libmp
import sympy as sp

from ..contracts.analysis import TurnOrientation
from ..contracts.envelopes import DirectionBindingCertificateV1
from ..numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from .metric import ExactPlanarMetric, _floor_exact
from .planar_types import (
    CertifiedPredicateUndecidable,
    ExactPlanarVector,
    IntervalEnclosureUnsupported,
    exact_normalize,
    exact_sign,
    interval_enclosure,
)


BINDING_MONOTONE = "BINDING_MONOTONE"
BINDING_SUBTURN_LE_DELTA_MAX = "BINDING_SUBTURN_LE_DELTA_MAX"
BINDING_INSIDE_OWN_ORDINAL_WINDOW = "BINDING_INSIDE_OWN_ORDINAL_WINDOW"
BINDING_REFUSAL_REASON = "привязка: сертификат не доказан ({predicate})"

_PROVEN_PREDICATES = frozenset(
    {
        BINDING_MONOTONE,
        BINDING_SUBTURN_LE_DELTA_MAX,
        BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    }
)
_DECIMAL_DIGITS = 27
_DECIMAL_SCALE = 10**_DECIMAL_DIGITS


class DirectionBindingCertificateUnproven(ValueError):
    """Именованный отказ, если хотя бы один точный предикат не доказан."""

    def __init__(self, predicate: str):
        self.predicate = predicate
        super().__init__(BINDING_REFUSAL_REASON.format(predicate=predicate))


def has_rational_support_direction(
    metric: ExactPlanarMetric, unit_normal: ExactPlanarVector
) -> bool:
    """Рациональна ли проективная запись ковектора данной единичной нормали."""

    return _primitive_direction(metric.support_covector_g(unit_normal)) is not None


def certify_direction_bindings(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
) -> tuple[DirectionBindingCertificateV1, ...]:
    """Привязать все hidden ordinal одного профиля; `n = len(normals) - 1`."""

    if len(ideal_unit_normals) < 3:
        return ()
    ideal = tuple(
        metric.support_covector_g(normal) for normal in ideal_unit_normals
    )
    windows = [
        [
            _midpoint(metric, ideal[index - 1], ideal[index]),
            _midpoint(metric, ideal[index], ideal[index + 1]),
        ]
        for index in range(1, len(ideal) - 1)
    ]
    while True:
        certificates = tuple(
            _certificate_in_window(ideal[index], *windows[index - 1])
            for index in range(1, len(ideal) - 1)
        )
        predicate = _failed_predicate(
            metric, ideal, orientation, certificates, tuple(map(tuple, windows))
        )
        if predicate is None:
            return certificates
        if predicate != BINDING_SUBTURN_LE_DELTA_MAX:
            raise DirectionBindingCertificateUnproven(predicate)
        for index, window in enumerate(windows, start=1):
            window[0] = _midpoint(metric, window[0], ideal[index])
            window[1] = _midpoint(metric, ideal[index], window[1])


def verify_direction_bindings(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    certificates: tuple[DirectionBindingCertificateV1, ...],
) -> None:
    """Перепроверить записанные plan-authority направления без их перестроения."""

    ideal = tuple(
        metric.support_covector_g(normal) for normal in ideal_unit_normals
    )
    if len(certificates) != max(0, len(ideal) - 2):
        raise DirectionBindingCertificateUnproven(BINDING_MONOTONE)
    windows = tuple(
        (
            _midpoint(metric, ideal[index - 1], ideal[index]),
            _midpoint(metric, ideal[index], ideal[index + 1]),
        )
        for index in range(1, len(ideal) - 1)
    )
    predicate = _failed_predicate(
        metric, ideal, orientation, certificates, windows
    )
    if predicate is not None:
        raise DirectionBindingCertificateUnproven(predicate)


def bound_unit_normal(
    metric: ExactPlanarMetric, certificate: DirectionBindingCertificateV1
) -> ExactPlanarVector:
    """Восстановить G-единичную нормаль из plan-authority ковектора."""

    x, y = certificate.bound_primitive_integer_vector
    inverse = metric.inverse_gram
    normal = ExactPlanarVector.from_values(
        inverse[0][0] * x + inverse[0][1] * y,
        inverse[1][0] * x + inverse[1][1] * y,
    )
    return metric.unit_g(normal)


def _certificate_in_window(
    ideal: ExactPlanarVector,
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> DirectionBindingCertificateV1:
    use_x = _dominant_x(ideal)
    denominator_index = 0 if use_x else 1
    ideal_values = ideal.expressions()
    denominator_sign = exact_sign(ideal_values[denominator_index])
    if denominator_sign == 0:
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    slopes = (_slope(left, use_x), _slope(right, use_x))
    lower, upper = (
        slopes
        if exact_sign(slopes[1] - slopes[0]) > 0
        else (slopes[1], slopes[0])
    )
    fraction = _stern_brocot_open(lower, upper)
    vector = _vector_from_slope(fraction, use_x, denominator_sign)
    lower_envelope = _decimal_envelope(lower)
    upper_envelope = _decimal_envelope(upper)
    width = upper_envelope.lower - lower_envelope.upper
    if width <= 0:
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    return DirectionBindingCertificateV1(
        bound_primitive_integer_vector=vector,
        ideal_window_lower_slope_envelope=lower_envelope,
        ideal_window_upper_slope_envelope=upper_envelope,
        certified_window_width_lower_bound=width,
        proven_predicates=_PROVEN_PREDICATES,
    )


def _failed_predicate(metric, ideal, orientation, certificates, windows):
    if any(
        certificate.proven_predicates != _PROVEN_PREDICATES
        or not _primitive_integer(certificate.bound_primitive_integer_vector)
        for certificate in certificates
    ):
        return BINDING_MONOTONE
    bound = tuple(
        ExactPlanarVector.from_values(
            *certificate.bound_primitive_integer_vector
        )
        for certificate in certificates
    )
    expected = (
        1
        if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
        else -1
    )
    sequence = (ideal[0], *bound, ideal[-1])
    try:
        if any(
            exact_sign(_oriented_cross(metric, left, right)) != expected
            for left, right in zip(sequence, sequence[1:])
        ):
            return BINDING_MONOTONE
        if any(
            not _inside(metric, candidate, window, expected)
            for candidate, window in zip(bound, windows, strict=True)
        ):
            return BINDING_INSIDE_OWN_ORDINAL_WINDOW
        if not _certificate_intervals_hold(ideal, bound, certificates):
            return BINDING_INSIDE_OWN_ORDINAL_WINDOW
        if any(
            not _subturn_at_most_pi_over_three(metric, left, right)
            for left, right in zip(sequence, sequence[1:])
        ):
            return BINDING_SUBTURN_LE_DELTA_MAX
    except DirectionBindingCertificateUnproven as exc:
        return exc.predicate
    except (CertifiedPredicateUndecidable, ValueError, TypeError):
        return BINDING_MONOTONE
    return None


def _inside(metric, candidate, window, expected):
    left, right = window
    return (
        exact_sign(_oriented_cross(metric, left, candidate)) == expected
        and exact_sign(_oriented_cross(metric, candidate, right)) == expected
    )


def _subturn_at_most_pi_over_three(metric, left, right):
    left_primitive = _primitive_direction(left)
    right_primitive = _primitive_direction(right)
    if left_primitive is None or right_primitive is None:
        raise DirectionBindingCertificateUnproven(
            BINDING_SUBTURN_LE_DELTA_MAX
        )
    dot = _dual_dot(metric, left_primitive, right_primitive)
    if exact_sign(dot) <= 0:
        return False
    lhs = exact_normalize(4 * dot * dot)
    rhs = exact_normalize(
        _dual_dot(metric, left_primitive, left_primitive)
        * _dual_dot(metric, right_primitive, right_primitive)
    )
    return exact_sign(lhs - rhs) >= 0


def _certificate_intervals_hold(ideal, bound, certificates):
    for ideal_direction, candidate, certificate in zip(
        ideal[1:-1], bound, certificates, strict=True
    ):
        lower = certificate.ideal_window_lower_slope_envelope
        upper = certificate.ideal_window_upper_slope_envelope
        width = certificate.certified_window_width_lower_bound
        if width <= 0 or width > upper.lower - lower.upper:
            return False
        if lower.upper >= upper.lower:
            return False
        use_x = _dominant_x(ideal_direction)
        ideal_values = ideal_direction.expressions()
        candidate_values = candidate.expressions()
        denominator_index = 0 if use_x else 1
        if exact_sign(ideal_values[denominator_index]) != exact_sign(
            candidate_values[denominator_index]
        ):
            return False
        slope = _slope(candidate, use_x)
        if (
            exact_sign(slope - sp.Rational(Fraction(lower.upper))) <= 0
            or exact_sign(slope - sp.Rational(Fraction(upper.lower))) >= 0
        ):
            return False
    return True


def _midpoint(metric, left, right):
    # Ковекторы нормализуются в двойственной метрике через G^-1.
    left_unit = _dual_unit(metric, left)
    right_unit = _dual_unit(metric, right)
    lx, ly = left_unit.expressions()
    rx, ry = right_unit.expressions()
    return _dual_unit(
        metric, ExactPlanarVector.from_values(lx + rx, ly + ry)
    )


def _dual_unit(metric, vector):
    squared = _dual_dot(metric, vector, vector)
    if exact_sign(squared) <= 0:
        raise DirectionBindingCertificateUnproven(BINDING_MONOTONE)
    x, y = vector.expressions()
    length = sp.sqrt(squared)
    return ExactPlanarVector.from_values(x / length, y / length)


def _dual_dot(metric, left, right):
    lx, ly = left.expressions()
    rx, ry = right.expressions()
    inverse = metric.inverse_gram
    return exact_normalize(
        lx * (inverse[0][0] * rx + inverse[0][1] * ry)
        + ly * (inverse[1][0] * rx + inverse[1][1] * ry)
    )


def _oriented_cross(metric, left, right):
    lx, ly = left.expressions()
    rx, ry = right.expressions()
    return exact_normalize(
        metric.owner_orientation_sign * (lx * ry - ly * rx)
    )


def _dominant_x(vector):
    x, y = vector.expressions()
    return exact_sign(x * x - y * y) >= 0


def _slope(vector, use_x):
    x, y = vector.expressions()
    denominator = x if use_x else y
    if exact_sign(denominator) == 0:
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    return exact_normalize((y if use_x else x) / denominator)


def _stern_brocot_open(lower, upper):
    """Первая медианта Stern–Brocot внутри непустого открытого интервала."""

    if exact_sign(upper - lower) <= 0:
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    shift = 1 - _floor_exact(lower)
    low = exact_normalize(lower + shift)
    high = exact_normalize(upper + shift)
    left_p, left_q = 0, 1
    right_p, right_q = 1, 0
    while True:
        numerator = left_p + right_p
        denominator = left_q + right_q
        candidate = sp.Rational(numerator, denominator)
        if exact_sign(candidate - low) <= 0:
            left_p, left_q = numerator, denominator
            continue
        if exact_sign(candidate - high) >= 0:
            right_p, right_q = numerator, denominator
            continue
        return Fraction(numerator - shift * denominator, denominator)


def _vector_from_slope(value, use_x, denominator_sign):
    numerator = value.numerator * denominator_sign
    denominator = value.denominator * denominator_sign
    return (
        (denominator, numerator) if use_x else (numerator, denominator)
    )


def _primitive_direction(vector):
    x, y = vector.expressions()
    if exact_sign(x) == 0:
        sign = exact_sign(y)
        return None if sign == 0 else ExactPlanarVector.from_values(0, sign)
    ratio = exact_normalize(y / x)
    if ratio.is_Rational is not True:
        return None
    sign = exact_sign(x)
    return ExactPlanarVector.from_values(sign * int(ratio.q), sign * int(ratio.p))


def _primitive_integer(vector):
    if len(vector) != 2 or any(type(value) is not int for value in vector):
        return False
    x, y = vector
    return (x != 0 or y != 0) and gcd(abs(x), abs(y)) == 1


def _decimal_envelope(expression):
    saved = iv.prec
    iv.prec = 160
    try:
        enclosure = interval_enclosure(expression)
    except (IntervalEnclosureUnsupported, ArithmeticError, ValueError, TypeError):
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    finally:
        iv.prec = saved
    lower, upper = (Fraction(*libmp.to_rational(item)) for item in enclosure._mpi_)
    lower_units = lower.numerator * _DECIMAL_SCALE // lower.denominator
    upper_units = -(
        (-upper.numerator * _DECIMAL_SCALE) // upper.denominator
    )
    lower_decimal = Decimal(f"{lower_units}E-{_DECIMAL_DIGITS}")
    upper_decimal = Decimal(f"{upper_units}E-{_DECIMAL_DIGITS}")
    return CertifiedDecimalIntervalV1(
        lower_decimal,
        upper_decimal,
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        upper_decimal - lower_decimal,
    )
