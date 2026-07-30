"""Сертифицированная рациональная привязка направлений скрытых опор."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from fractions import Fraction
from math import gcd, isqrt

from mpmath import iv, libmp
import sympy as sp

from ..contracts.analysis import TurnOrientation
from ..contracts.envelopes import (
    DirectionBindingCertificateV1,
    EvaluationGeometryDirectionBindingCertificateV1,
)
from ..numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from .direction_window_exact import (
    _ExactWindowUnsupported,
    _MQField,
    _TowerField,
    _fraction,
    _interval_product,
)
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
BINDING_SUBTURN_RATIONAL_DIRECTION_REQUIRED = (
    "BINDING_SUBTURN_RATIONAL_DIRECTION_REQUIRED"
)
BINDING_INSIDE_OWN_ORDINAL_WINDOW = "BINDING_INSIDE_OWN_ORDINAL_WINDOW"
BINDING_REFINEMENT_LIMIT = "BINDING_REFINEMENT_LIMIT"
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
_MAX_WINDOW_REFINEMENTS = 256


@dataclass(frozen=True, slots=True)
class _ProjectiveRay:
    field: object
    x: object
    y: object
    squared: object
    length: object | None = None


@dataclass(frozen=True, slots=True)
class _FastK1Geometry:
    inverse: tuple[
        tuple[Fraction, Fraction],
        tuple[Fraction, Fraction],
    ]
    owner_orientation_sign: int
    incoming: _ProjectiveRay
    hidden: _ProjectiveRay
    outgoing: _ProjectiveRay
    left_window: _ProjectiveRay
    right_window: _ProjectiveRay
    primitive_incoming: tuple[int, int]
    primitive_outgoing: tuple[int, int]


def _support_covector_expressions(metric, normal) -> tuple[sp.Expr, sp.Expr]:
    x, y = normal.expressions()
    gram = metric.gram
    return (
        gram[0][0] * x + gram[0][1] * y,
        gram[1][0] * x + gram[1][1] * y,
    )


def _simple_exact_sign(expression: sp.Expr) -> int | None:
    """Знак rational/radical product без SymPy assumptions и factor."""

    if expression.is_Rational:
        return (expression.p > 0) - (expression.p < 0)
    if expression.is_Mul:
        sign = 1
        for factor in expression.args:
            factor_sign = _simple_exact_sign(factor)
            if factor_sign is None:
                return None
            sign *= factor_sign
        return sign
    if expression.is_Pow:
        base, exponent = expression.args
        base_sign = _simple_exact_sign(base)
        if base_sign is None or exponent.is_Rational is not True:
            return None
        if exponent.q % 2 == 0:
            return 1 if base_sign > 0 else None
        return base_sign if int(exponent.p) % 2 else 1
    return None


def _raw_primitive_direction(
    metric,
    normal,
) -> tuple[int, int] | None:
    x, y = _support_covector_expressions(metric, normal)
    if x == 0:
        sign = _simple_exact_sign(y)
        return None if not sign else (0, sign)
    ratio = y / x
    if ratio.is_Rational is not True:
        return None
    sign = _simple_exact_sign(x)
    if not sign:
        return None
    fraction = _fraction(ratio)
    return sign * fraction.denominator, sign * fraction.numerator


def _fraction_matrix(metric) -> tuple[
    tuple[Fraction, Fraction],
    tuple[Fraction, Fraction],
]:
    return tuple(
        tuple(_fraction(value) for value in row)
        for row in metric.inverse_gram
    )


def _fraction_dot(matrix, left, right) -> Fraction:
    return (
        left[0] * (matrix[0][0] * right[0] + matrix[0][1] * right[1])
        + left[1] * (matrix[1][0] * right[0] + matrix[1][1] * right[1])
    )


def _value_dot(matrix, left, right):
    return (
        left[0]
        * (
            right[0].scaled(matrix[0][0])
            + right[1].scaled(matrix[0][1])
        )
        + left[1]
        * (
            right[0].scaled(matrix[1][0])
            + right[1].scaled(matrix[1][1])
        )
    )


def _primitive_ray(field, matrix, vector) -> _ProjectiveRay:
    squared = _fraction_dot(matrix, vector, vector)
    if squared <= 0:
        raise _ExactWindowUnsupported("non-positive primitive direction")
    return _ProjectiveRay(
        field,
        field.rational(vector[0]),
        field.rational(vector[1]),
        field.rational(squared),
        field.radical(squared),
    )


def _promote_value(value, extension: _TowerField):
    return extension.promote(value)


def _promote_ray(
    ray: _ProjectiveRay,
    extension: _TowerField,
) -> _ProjectiveRay:
    if ray.field is not extension.base:
        raise _ExactWindowUnsupported("cannot promote unrelated ray")
    return _ProjectiveRay(
        extension,
        _promote_value(ray.x, extension),
        _promote_value(ray.y, extension),
        _promote_value(ray.squared, extension),
        (
            None
            if ray.length is None
            else _promote_value(ray.length, extension)
        ),
    )


def _ray_in_field(ray: _ProjectiveRay, field) -> _ProjectiveRay:
    if ray.field is field:
        return ray
    if isinstance(field, _TowerField) and ray.field is field.base:
        return _promote_ray(ray, field)
    raise _ExactWindowUnsupported("ray fields cannot be joined")


def _rational_root_in_field(field, value: Fraction):
    if isinstance(field, _MQField):
        return field.radical(value)
    return field.promote(_rational_root_in_field(field.base, value))


def _known_root(ray: _ProjectiveRay):
    if ray.length is not None:
        return ray.length
    rational = ray.squared.as_rational()
    if rational is not None:
        return _rational_root_in_field(ray.field, rational)
    if isinstance(ray.field, _TowerField):
        declared = ray.field.promote(ray.field.radicand)
        if (ray.squared - declared).is_zero:
            return ray.field.root()
    return None


def _with_length(
    ray: _ProjectiveRay,
    length,
) -> _ProjectiveRay:
    return _ProjectiveRay(
        ray.field,
        ray.x,
        ray.y,
        ray.squared,
        length,
    )


def _common_rays_with_lengths(left, right):
    if left.field is not right.field:
        raise _ExactWindowUnsupported("midpoint rays do not share a field")
    left_length = _known_root(left)
    right_length = _known_root(right)
    missing = int(left_length is None) + int(right_length is None)
    if missing > 1:
        raise _ExactWindowUnsupported("cross-root midpoint is unsupported")
    if not missing:
        return _with_length(left, left_length), _with_length(right, right_length)
    source = left if left_length is None else right
    extension = _TowerField(source.field, source.squared)
    promoted_left = _promote_ray(left, extension)
    promoted_right = _promote_ray(right, extension)
    root = extension.root()
    if left_length is None:
        promoted_left = _with_length(promoted_left, root)
    else:
        promoted_left = _with_length(
            promoted_left,
            extension.promote(left_length),
        )
    if right_length is None:
        promoted_right = _with_length(promoted_right, root)
    else:
        promoted_right = _with_length(
            promoted_right,
            extension.promote(right_length),
        )
    return promoted_left, promoted_right


def _midpoint_ray(matrix, left, right):
    left, right = _common_rays_with_lengths(left, right)
    x = left.x * right.length + right.x * left.length
    y = left.y * right.length + right.y * left.length
    squared = _value_dot(matrix, (x, y), (x, y))
    return _ProjectiveRay(left.field, x, y, squared), left, right


def _fast_k1_recipe_geometry(
    metric,
    incoming_normal,
    outgoing_normal,
) -> _FastK1Geometry | None:
    incoming_vector = _raw_primitive_direction(metric, incoming_normal)
    outgoing_vector = _raw_primitive_direction(metric, outgoing_normal)
    if incoming_vector is None or outgoing_vector is None:
        return None
    inverse = _fraction_matrix(metric)
    field = _MQField()
    incoming = _primitive_ray(field, inverse, incoming_vector)
    outgoing = _primitive_ray(field, inverse, outgoing_vector)
    hidden, incoming, outgoing = _midpoint_ray(inverse, incoming, outgoing)
    left_window, incoming, hidden = _midpoint_ray(inverse, incoming, hidden)
    outgoing = _ray_in_field(outgoing, left_window.field)
    outgoing = _with_length(
        outgoing,
        _rational_root_in_field(
            outgoing.field,
            _fraction_dot(inverse, outgoing_vector, outgoing_vector),
        ),
    )
    right_window, hidden, outgoing = _midpoint_ray(
        inverse,
        hidden,
        outgoing,
    )
    return _FastK1Geometry(
        inverse,
        metric.owner_orientation_sign,
        incoming,
        hidden,
        outgoing,
        left_window,
        right_window,
        incoming_vector,
        outgoing_vector,
    )


def _slope_parts(ray: _ProjectiveRay, use_x: bool):
    return (ray.y, ray.x) if use_x else (ray.x, ray.y)


def _compare_slope(
    ray: _ProjectiveRay,
    use_x: bool,
    candidate: Fraction,
) -> int:
    numerator, denominator = _slope_parts(ray, use_x)
    difference = (
        numerator.scaled(candidate.denominator)
        - denominator.scaled(candidate.numerator)
    )
    return (difference * denominator).sign()


def _slope_enclosure(
    ray: _ProjectiveRay,
    use_x: bool,
    bits: int,
) -> tuple[Fraction, Fraction]:
    numerator, denominator = _slope_parts(ray, use_x)
    numerator_interval = numerator.enclosure(bits)
    denominator_interval = denominator.enclosure(bits)
    if denominator_interval[0] <= 0 <= denominator_interval[1]:
        raise _ExactWindowUnsupported("slope denominator enclosure crosses zero")
    reciprocal = (
        Fraction(1, denominator_interval[1]),
        Fraction(1, denominator_interval[0]),
    )
    if reciprocal[0] > reciprocal[1]:
        reciprocal = reciprocal[1], reciprocal[0]
    return _interval_product(numerator_interval, reciprocal)


def _floor_scaled_slope(
    ray: _ProjectiveRay,
    use_x: bool,
    scale: int,
) -> int:
    for bits in (64, 128, 256, 512, 1024, 2048, 4096):
        try:
            lower, upper = _slope_enclosure(ray, use_x, bits)
        except _ExactWindowUnsupported:
            continue
        lower *= scale
        upper *= scale
        lower_floor = lower.numerator // lower.denominator
        upper_floor = upper.numerator // upper.denominator
        if lower_floor == upper_floor:
            return lower_floor
        if upper_floor - lower_floor <= 2:
            for candidate in range(lower_floor, upper_floor + 1):
                if _compare_slope(
                    ray,
                    use_x,
                    Fraction(candidate, scale),
                ) == 0:
                    return candidate
    raise _ExactWindowUnsupported("exact slope floor is undecidable")


def _dominant_x_ray(ray: _ProjectiveRay) -> bool:
    return (ray.x * ray.x - ray.y * ray.y).sign() >= 0


def _denominator_sign(ray: _ProjectiveRay, use_x: bool) -> int:
    _, denominator = _slope_parts(ray, use_x)
    sign = denominator.sign()
    if not sign:
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    return sign


def _ordered_windows(
    geometry: _FastK1Geometry,
    left: _ProjectiveRay,
    right: _ProjectiveRay,
    use_x: bool,
    expected: int,
) -> tuple[_ProjectiveRay, _ProjectiveRay]:
    left_sign = _denominator_sign(left, use_x)
    right_sign = _denominator_sign(right, use_x)
    raw_cross_sign = expected * geometry.owner_orientation_sign
    increasing = raw_cross_sign * left_sign * right_sign
    if not use_x:
        increasing = -increasing
    return (left, right) if increasing > 0 else (right, left)


def _stern_brocot_rays(
    lower: _ProjectiveRay,
    upper: _ProjectiveRay,
    use_x: bool,
) -> Fraction:
    lower_floor = _floor_scaled_slope(lower, use_x, 1)
    shift = 1 - lower_floor
    left_p, left_q = 0, 1
    right_p, right_q = 1, 0
    while True:
        numerator = left_p + right_p
        denominator = left_q + right_q
        candidate = Fraction(numerator, denominator)
        original = candidate - shift
        if _compare_slope(lower, use_x, original) >= 0:
            left_p, left_q = numerator, denominator
            continue
        if _compare_slope(upper, use_x, original) <= 0:
            right_p, right_q = numerator, denominator
            continue
        return original


def _decimal_envelope_ray(
    ray: _ProjectiveRay,
    use_x: bool,
) -> CertifiedDecimalIntervalV1:
    lower_units = _floor_scaled_slope(ray, use_x, _DECIMAL_SCALE)
    exact = _compare_slope(
        ray,
        use_x,
        Fraction(lower_units, _DECIMAL_SCALE),
    ) == 0
    upper_units = lower_units if exact else lower_units + 1
    lower = Decimal(f"{lower_units}E-{_DECIMAL_DIGITS}")
    upper = Decimal(f"{upper_units}E-{_DECIMAL_DIGITS}")
    return CertifiedDecimalIntervalV1(
        lower,
        upper,
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        upper - lower,
    )


def _fast_certificate_in_window(
    geometry: _FastK1Geometry,
    ideal: _ProjectiveRay,
    left: _ProjectiveRay,
    right: _ProjectiveRay,
    expected: int,
) -> DirectionBindingCertificateV1:
    use_x = _dominant_x_ray(ideal)
    denominator_sign = _denominator_sign(ideal, use_x)
    lower, upper = _ordered_windows(
        geometry,
        left,
        right,
        use_x,
        expected,
    )
    fraction = _stern_brocot_rays(lower, upper, use_x)
    vector = _vector_from_slope(fraction, use_x, denominator_sign)
    lower_envelope = _decimal_envelope_ray(lower, use_x)
    upper_envelope = _decimal_envelope_ray(upper, use_x)
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


def _raw_cross_with_candidate(
    ray: _ProjectiveRay,
    candidate: tuple[int, int],
):
    return ray.x.scaled(candidate[1]) - ray.y.scaled(candidate[0])


def _fast_inside(
    geometry: _FastK1Geometry,
    candidate: tuple[int, int],
    window: tuple[_ProjectiveRay, _ProjectiveRay],
    expected: int,
) -> bool:
    left, right = window
    left_sign = (
        _raw_cross_with_candidate(left, candidate).sign()
        * geometry.owner_orientation_sign
    )
    right_sign = (
        -_raw_cross_with_candidate(right, candidate).sign()
        * geometry.owner_orientation_sign
    )
    return left_sign == expected and right_sign == expected


def _fast_subturn(
    geometry: _FastK1Geometry,
    left: tuple[int, int],
    right: tuple[int, int],
) -> bool:
    dot = _fraction_dot(geometry.inverse, left, right)
    if dot <= 0:
        return False
    return 4 * dot * dot >= (
        _fraction_dot(geometry.inverse, left, left)
        * _fraction_dot(geometry.inverse, right, right)
    )


def _fast_certificate_intervals_hold(
    geometry: _FastK1Geometry,
    bound: tuple[tuple[int, int], ...],
    certificates,
) -> bool:
    ideal = geometry.hidden
    for candidate, certificate in zip(bound, certificates, strict=True):
        if certificate is None:
            continue
        lower = certificate.ideal_window_lower_slope_envelope
        upper = certificate.ideal_window_upper_slope_envelope
        width = certificate.certified_window_width_lower_bound
        if width <= 0 or width > upper.lower - lower.upper:
            return False
        if lower.upper >= upper.lower:
            return False
        use_x = _dominant_x_ray(ideal)
        denominator = candidate[0] if use_x else candidate[1]
        if _denominator_sign(ideal, use_x) != (
            (denominator > 0) - (denominator < 0)
        ):
            return False
        numerator = candidate[1] if use_x else candidate[0]
        slope = Fraction(numerator, denominator)
        if slope <= Fraction(lower.upper) or slope >= Fraction(upper.lower):
            return False
    return True


def _fast_failed_predicate(
    geometry: _FastK1Geometry,
    orientation: TurnOrientation,
    certificates,
    windows,
):
    if any(
        certificate is not None
        and (
            certificate.proven_predicates != _PROVEN_PREDICATES
            or not _primitive_integer(certificate.bound_primitive_integer_vector)
        )
        for certificate in certificates
    ):
        return BINDING_MONOTONE
    bound = tuple(
        certificate.bound_primitive_integer_vector
        for certificate in certificates
        if certificate is not None
    )
    if len(bound) != 1:
        return BINDING_MONOTONE
    expected = (
        1
        if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
        else -1
    )
    sequence = (
        geometry.primitive_incoming,
        bound[0],
        geometry.primitive_outgoing,
    )
    if any(
        geometry.owner_orientation_sign
        * (
            (
                left[0] * right[1] - left[1] * right[0] > 0
            )
            - (
                left[0] * right[1] - left[1] * right[0] < 0
            )
        )
        != expected
        for left, right in zip(sequence, sequence[1:])
    ):
        return BINDING_MONOTONE
    if not _fast_inside(geometry, bound[0], windows[0], expected):
        return BINDING_INSIDE_OWN_ORDINAL_WINDOW
    if not _fast_certificate_intervals_hold(
        geometry,
        bound,
        certificates,
    ):
        return BINDING_INSIDE_OWN_ORDINAL_WINDOW
    if any(
        not _fast_subturn(geometry, left, right)
        for left, right in zip(sequence, sequence[1:])
    ):
        return BINDING_SUBTURN_LE_DELTA_MAX
    return None


def _refine_fast_windows(
    geometry: _FastK1Geometry,
    windows,
):
    left, _, _ = _midpoint_ray(
        geometry.inverse,
        windows[0][0],
        geometry.hidden,
    )
    right, _, _ = _midpoint_ray(
        geometry.inverse,
        geometry.hidden,
        windows[0][1],
    )
    return ((left, right),)


def _certify_fast_k1(
    geometry: _FastK1Geometry,
    orientation: TurnOrientation,
) -> tuple[DirectionBindingCertificateV1, ...]:
    expected = (
        1
        if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
        else -1
    )
    windows = ((geometry.left_window, geometry.right_window),)
    # Поле K1 доказано для начального окна и одного сужения. Следующий уровень
    # возвращается в прежний символьный решатель, а не получает новый отказ.
    for refinement in range(2):
        certificate = _fast_certificate_in_window(
            geometry,
            geometry.hidden,
            windows[0][0],
            windows[0][1],
            expected,
        )
        certificates = (certificate,)
        predicate = _fast_failed_predicate(
            geometry,
            orientation,
            certificates,
            windows,
        )
        if predicate is None:
            return certificates
        if predicate != BINDING_SUBTURN_LE_DELTA_MAX:
            raise DirectionBindingCertificateUnproven(predicate)
        if refinement == 1:
            raise _ExactWindowUnsupported("K1 exact-window depth exceeds two")
        windows = _refine_fast_windows(geometry, windows)
    raise _ExactWindowUnsupported("K1 exact-window depth exceeds two")


def _verify_fast_k1(
    geometry: _FastK1Geometry,
    orientation: TurnOrientation,
    certificates,
) -> None:
    windows = ((geometry.left_window, geometry.right_window),)
    predicate = _fast_failed_predicate(
        geometry,
        orientation,
        certificates,
        windows,
    )
    if predicate is not None:
        raise DirectionBindingCertificateUnproven(predicate)


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
    """Привязать arbitrary ideal tuple прежним полным symbolic solver."""

    return _certify_direction_bindings_symbolic(
        metric,
        ideal_unit_normals,
        orientation,
    )


def certify_huber_density_direction_bindings(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    max_subturn_q: int,
) -> tuple[DirectionBindingCertificateV1 | None, ...]:
    """Каноническая B(w)-привязка Density A без неограниченного root solve."""

    if max_subturn_q not in range(2, 7):
        raise DirectionBindingCertificateUnproven(BINDING_SUBTURN_LE_DELTA_MAX)
    if len(ideal_unit_normals) < 3:
        return ()
    ideal = tuple(
        metric.support_covector_g(normal) for normal in ideal_unit_normals
    )
    windows = tuple(
        (
            _midpoint(metric, ideal[index - 1], ideal[index]),
            _midpoint(metric, ideal[index], ideal[index + 1]),
        )
        for index in range(1, len(ideal) - 1)
    )
    certificates = tuple(
        _huber_density_certificate_in_window(
            metric,
            ideal[index],
            ideal[index - 1],
            ideal[index + 1],
            windows[index - 1][0],
            windows[index - 1][1],
            max_subturn_q,
        )
        for index in range(1, len(ideal) - 1)
    )
    predicate = _failed_huber_density_predicate(
        metric,
        ideal,
        orientation,
        certificates,
        windows,
        max_subturn_q,
    )
    if predicate is not None:
        raise DirectionBindingCertificateUnproven(predicate)
    return certificates


def verify_huber_density_direction_bindings(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    certificates: tuple[
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
        | None,
        ...,
    ],
    max_subturn_q: int,
) -> None:
    """Перевывести B(w)-победителей и перепроверить записанные certificates."""

    if len(certificates) != max(0, len(ideal_unit_normals) - 2):
        raise DirectionBindingCertificateUnproven(BINDING_MONOTONE)
    ideal = tuple(
        metric.support_covector_g(normal) for normal in ideal_unit_normals
    )
    windows = tuple(
        (
            _midpoint(metric, ideal[index - 1], ideal[index]),
            _midpoint(metric, ideal[index], ideal[index + 1]),
        )
        for index in range(1, len(ideal) - 1)
    )
    canonical = tuple(
        _huber_density_certificate_in_window(
            metric,
            ideal[index],
            ideal[index - 1],
            ideal[index + 1],
            windows[index - 1][0],
            windows[index - 1][1],
            max_subturn_q,
        )
        for index in range(1, len(ideal) - 1)
    )
    for index, (supplied, expected) in enumerate(
        zip(certificates, canonical, strict=True),
        start=1,
    ):
        if supplied is None:
            if (
                expected is not None
                and _primitive_direction(ideal[index]) is None
            ):
                raise DirectionBindingCertificateUnproven(
                    BINDING_INSIDE_OWN_ORDINAL_WINDOW
                )
            continue
        if expected is None:
            raise DirectionBindingCertificateUnproven(
                BINDING_INSIDE_OWN_ORDINAL_WINDOW
            )
        if (
            supplied.bound_primitive_integer_vector
            != expected.bound_primitive_integer_vector
            or supplied.ideal_window_lower_slope_envelope
            != expected.ideal_window_lower_slope_envelope
            or supplied.ideal_window_upper_slope_envelope
            != expected.ideal_window_upper_slope_envelope
            or supplied.certified_window_width_lower_bound
            != expected.certified_window_width_lower_bound
            or supplied.proven_predicates != expected.proven_predicates
        ):
            raise DirectionBindingCertificateUnproven(
                BINDING_INSIDE_OWN_ORDINAL_WINDOW
            )
    predicate = _failed_huber_density_predicate(
        metric,
        ideal,
        orientation,
        certificates,
        windows,
        max_subturn_q,
    )
    if predicate is not None:
        raise DirectionBindingCertificateUnproven(predicate)


def _certify_k1_recipe_direction_bindings(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
    symbolic_ideal: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
) -> tuple[DirectionBindingCertificateV1, ...]:
    """Private fast path только для доказанного K1 interpolation recipe."""

    try:
        geometry = _fast_k1_recipe_geometry(metric, incoming, outgoing)
        if geometry is not None:
            return _certify_fast_k1(geometry, orientation)
    except _ExactWindowUnsupported:
        pass
    return _certify_direction_bindings_symbolic(
        metric,
        symbolic_ideal,
        orientation,
    )


def _certify_direction_bindings_symbolic(
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
    for refinement in range(_MAX_WINDOW_REFINEMENTS + 1):
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
        if refinement == _MAX_WINDOW_REFINEMENTS:
            raise DirectionBindingCertificateUnproven(
                BINDING_REFINEMENT_LIMIT
            )
        for index, window in enumerate(windows, start=1):
            window[0] = _midpoint(metric, window[0], ideal[index])
            window[1] = _midpoint(metric, ideal[index], window[1])
    raise DirectionBindingCertificateUnproven(BINDING_REFINEMENT_LIMIT)


def verify_direction_bindings(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    certificates: tuple[
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
        | None,
        ...,
    ],
) -> None:
    """Перепроверить arbitrary ideal tuple прежним полным symbolic solver."""

    _verify_direction_bindings_symbolic(
        metric,
        ideal_unit_normals,
        orientation,
        certificates,
    )


def _verify_k1_recipe_direction_bindings(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
    symbolic_ideal: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    certificates: tuple[
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
        | None,
        ...,
    ],
) -> None:
    """Private verifier того же доказанного K1 interpolation recipe."""

    if len(certificates) != 1:
        raise DirectionBindingCertificateUnproven(BINDING_MONOTONE)
    if not any(certificate is not None for certificate in certificates):
        return
    try:
        geometry = _fast_k1_recipe_geometry(metric, incoming, outgoing)
        if geometry is not None:
            _verify_fast_k1(geometry, orientation, certificates)
            return
    except _ExactWindowUnsupported:
        pass
    _verify_direction_bindings_symbolic(
        metric,
        symbolic_ideal,
        orientation,
        certificates,
    )


def _verify_direction_bindings_symbolic(
    metric: ExactPlanarMetric,
    ideal_unit_normals: tuple[ExactPlanarVector, ...],
    orientation: TurnOrientation,
    certificates: tuple[
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
        | None,
        ...,
    ],
) -> None:
    """Перепроверить записанные plan-authority направления без их перестроения."""

    ideal = tuple(
        metric.support_covector_g(normal) for normal in ideal_unit_normals
    )
    if len(certificates) != max(0, len(ideal) - 2):
        raise DirectionBindingCertificateUnproven(BINDING_MONOTONE)
    if not any(certificate is not None for certificate in certificates):
        return
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
    metric: ExactPlanarMetric,
    certificate: (
        DirectionBindingCertificateV1
        | EvaluationGeometryDirectionBindingCertificateV1
    ),
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


def _huber_density_certificate_in_window(
    metric: ExactPlanarMetric,
    ideal: ExactPlanarVector,
    left_ideal: ExactPlanarVector,
    right_ideal: ExactPlanarVector,
    left_window: ExactPlanarVector,
    right_window: ExactPlanarVector,
    max_subturn_q: int,
) -> DirectionBindingCertificateV1 | None:
    use_x = _dominant_x(ideal)
    denominator_index = 0 if use_x else 1
    denominator_sign = exact_sign(ideal.expressions()[denominator_index])
    if denominator_sign == 0:
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    slopes = (
        _slope(left_window, use_x),
        _slope(right_window, use_x),
    )
    lower_exact, upper_exact = (
        slopes
        if exact_sign(slopes[1] - slopes[0]) > 0
        else (slopes[1], slopes[0])
    )
    lower_envelope = _decimal_envelope(lower_exact)
    upper_envelope = _decimal_envelope(upper_exact)
    lower = Fraction(lower_envelope.upper)
    upper = Fraction(upper_envelope.lower)
    width = upper - lower
    if width <= 0:
        raise DirectionBindingCertificateUnproven(
            BINDING_INSIDE_OWN_ORDINAL_WINDOW
        )
    authority_budget = _huber_authority_budget(width)
    candidates = []
    ideal_unit = _dual_unit(metric, ideal)
    for denominator in range(1, authority_budget + 1):
        scaled_lower = lower * denominator
        scaled_upper = upper * denominator
        first = scaled_lower.numerator // scaled_lower.denominator + 1
        last = -(
            (-scaled_upper.numerator) // scaled_upper.denominator
        ) - 1
        for numerator in range(first, last + 1):
            if gcd(abs(numerator), denominator) != 1:
                continue
            slope = Fraction(numerator, denominator)
            vector = _vector_from_slope(
                slope,
                use_x,
                denominator_sign,
            )
            candidate = ExactPlanarVector.from_values(*vector)
            if not (
                _subturn_at_most_pi_over_q(
                    metric,
                    left_ideal,
                    candidate,
                    max_subturn_q,
                )
                and _subturn_at_most_pi_over_q(
                    metric,
                    candidate,
                    right_ideal,
                    max_subturn_q,
                )
            ):
                continue
            candidate_unit = _dual_unit(metric, candidate)
            ix, iy = ideal_unit.expressions()
            cx, cy = candidate_unit.expressions()
            difference = ExactPlanarVector.from_values(cx - ix, cy - iy)
            objective = _dual_dot(metric, difference, difference)
            candidates.append((objective, slope, vector))
    if not candidates:
        # A′-власть конечна и допускает EMPTY_AUTHORITY: в этом случае
        # канонической остаётся точная символическая ordinal-опора.
        return None
    winner = candidates[0]
    for candidate in candidates[1:]:
        comparison = exact_sign(candidate[0] - winner[0])
        if comparison < 0 or (
            comparison == 0 and candidate[1] > winner[1]
        ):
            winner = candidate
    return DirectionBindingCertificateV1(
        bound_primitive_integer_vector=winner[2],
        ideal_window_lower_slope_envelope=lower_envelope,
        ideal_window_upper_slope_envelope=upper_envelope,
        certified_window_width_lower_bound=(
            upper_envelope.lower - lower_envelope.upper
        ),
        proven_predicates=_PROVEN_PREDICATES,
    )


def _huber_authority_budget(width: Fraction) -> int:
    """Минимальный B: `B²*w >= 1`, без поискового цикла по кандидатам."""

    quotient_ceiling = (
        width.denominator + width.numerator - 1
    ) // width.numerator
    budget = isqrt(quotient_ceiling)
    if budget * budget < quotient_ceiling:
        budget += 1
    while budget > 1 and (
        (budget - 1) * (budget - 1) * width.numerator
        >= width.denominator
    ):
        budget -= 1
    if budget * budget * width.numerator < width.denominator:
        budget += 1
    return budget


def _failed_predicate(metric, ideal, orientation, certificates, windows):
    if any(
        certificate is not None
        and (
            certificate.proven_predicates != _PROVEN_PREDICATES
            or not _primitive_integer(certificate.bound_primitive_integer_vector)
        )
        for certificate in certificates
    ):
        return BINDING_MONOTONE
    resolved = tuple(
        ideal[index]
        if certificate is None
        else ExactPlanarVector.from_values(
            *certificate.bound_primitive_integer_vector
        )
        for index, certificate in enumerate(certificates, start=1)
    )
    expected = (
        1
        if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
        else -1
    )
    sequence = (ideal[0], *resolved, ideal[-1])
    try:
        if any(
            exact_sign(_oriented_cross(metric, left, right)) != expected
            for left, right in zip(sequence, sequence[1:])
        ):
            return BINDING_MONOTONE
        if any(
            certificate is not None
            and not _inside(metric, candidate, window, expected)
            for certificate, candidate, window in zip(
                certificates, resolved, windows, strict=True
            )
        ):
            return BINDING_INSIDE_OWN_ORDINAL_WINDOW
        if not _certificate_intervals_hold(ideal, resolved, certificates):
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


def _failed_huber_density_predicate(
    metric,
    ideal,
    orientation,
    certificates,
    windows,
    max_subturn_q,
):
    if any(
        certificate is not None
        and (
            certificate.proven_predicates != _PROVEN_PREDICATES
            or not _primitive_integer(
                certificate.bound_primitive_integer_vector
            )
        )
        for certificate in certificates
    ):
        return BINDING_MONOTONE
    resolved = tuple(
        ideal[index]
        if certificate is None
        else ExactPlanarVector.from_values(
            *certificate.bound_primitive_integer_vector
        )
        for index, certificate in enumerate(certificates, start=1)
    )
    expected = (
        1
        if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
        else -1
    )
    sequence = (ideal[0], *resolved, ideal[-1])
    try:
        if any(
            exact_sign(_oriented_cross(metric, left, right)) != expected
            for left, right in zip(sequence, sequence[1:])
        ):
            return BINDING_MONOTONE
        if any(
            certificate is not None
            and not _inside(metric, candidate, window, expected)
            for certificate, candidate, window in zip(
                certificates,
                resolved,
                windows,
                strict=True,
            )
        ):
            return BINDING_INSIDE_OWN_ORDINAL_WINDOW
        if not _certificate_intervals_hold(
            ideal,
            resolved,
            certificates,
        ):
            return BINDING_INSIDE_OWN_ORDINAL_WINDOW
        if any(
            not _subturn_at_most_pi_over_q(
                metric,
                left,
                right,
                max_subturn_q,
            )
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
            BINDING_SUBTURN_RATIONAL_DIRECTION_REQUIRED
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


def _subturn_at_most_pi_over_q(metric, left, right, q: int):
    """Точный dual-Gram predicate для q=2..6.

    Для q=5 порог `(3+sqrt(5))/8` намеренно остаётся quadratic exact
    expression; никакой Decimal/float-подмены здесь нет.
    """

    dot = _dual_dot(metric, left, right)
    dot_sign = exact_sign(dot)
    if q == 2:
        return dot_sign >= 0
    if dot_sign <= 0:
        return False
    norm_product = exact_normalize(
        _dual_dot(metric, left, left)
        * _dual_dot(metric, right, right)
    )
    dot_squared = exact_normalize(dot * dot)
    if q == 3:
        residual = exact_normalize(4 * dot_squared - norm_product)
    elif q == 4:
        residual = exact_normalize(2 * dot_squared - norm_product)
    elif q == 5:
        residual = exact_normalize(
            8 * dot_squared - (3 + sp.sqrt(5)) * norm_product
        )
    elif q == 6:
        residual = exact_normalize(
            4 * dot_squared - 3 * norm_product
        )
    else:
        raise DirectionBindingCertificateUnproven(
            BINDING_SUBTURN_LE_DELTA_MAX
        )
    return exact_sign(residual) >= 0


def _certificate_intervals_hold(ideal, bound, certificates):
    for ideal_direction, candidate, certificate in zip(
        ideal[1:-1], bound, certificates, strict=True
    ):
        if certificate is None:
            continue
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
