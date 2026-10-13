"""Целочисленный A2 value-path; не меняет legacy admission или DensityWork.

Источник алгоритма: native/cftuv-density/src/{angle_monotone,angle_profile,
angle,interval,angle_poly}. Фиксированная граница относится только к отдельным
компонентам луча; ширина произвольной композиции здесь не доказана.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from math import isqrt


COMPONENT_BITS = 160
_ROOT_BITS = (325, 327, 329, 331, 333)
_LIPSCHITZ = (1, 4, 15, 48, 145)
_U32_MAX = (1 << 32) - 1


class AngleV1ErrorCode(str, Enum):
    UNSUPPORTED_SUBDIVISION = "UNSUPPORTED_SUBDIVISION"
    RATIO_OUTSIDE_HALF_OPEN_UNIT_INTERVAL = "RATIO_OUTSIDE_HALF_OPEN_UNIT_INTERVAL"
    INCONSISTENT_DOT_SIGN = "INCONSISTENT_DOT_SIGN"
    NON_RATIONAL_INPUT = "NON_RATIONAL_INPUT"
    INVALID_BIT_COUNT = "INVALID_BIT_COUNT"
    BIT_COUNT_OVERFLOW = "BIT_COUNT_OVERFLOW"
    INVALID_ORDINAL = "INVALID_ORDINAL"
    NO_HIDDEN_ORDINAL = "NO_HIDDEN_ORDINAL"
    REVERSED_INTERVAL = "REVERSED_INTERVAL"
    EMPTY_INTERSECTION = "EMPTY_INTERSECTION"
    NEGATIVE_RADICAND = "NEGATIVE_RADICAND"
    ENCLOSURE_INVARIANT = "ENCLOSURE_INVARIANT"
    WIDTH_PROOF_VIOLATED = "WIDTH_PROOF_VIOLATED"


class AngleV1Error(ValueError):
    """Внутренняя ошибка прототипа; не продуктовый UNDECIDABLE."""

    def __init__(self, code: AngleV1ErrorCode):
        self.code = code
        super().__init__(code.value)


def _fraction(value: Fraction | int) -> Fraction:
    if type(value) not in (Fraction, int):
        raise AngleV1Error(AngleV1ErrorCode.NON_RATIONAL_INPUT)
    return Fraction(value)


def _bits(value: int, multiplier: int = 1, extra: int = 0) -> int:
    if type(value) is not int or value < 0:
        raise AngleV1Error(AngleV1ErrorCode.INVALID_BIT_COUNT)
    if value > _U32_MAX or value * multiplier + extra > _U32_MAX:
        raise AngleV1Error(AngleV1ErrorCode.BIT_COUNT_OVERFLOW)
    return value


@dataclass(frozen=True, slots=True)
class FractionIntervalV1:
    lower: Fraction
    upper: Fraction

    def __post_init__(self):
        object.__setattr__(self, "lower", _fraction(self.lower))
        object.__setattr__(self, "upper", _fraction(self.upper))
        if self.lower > self.upper:
            raise AngleV1Error(AngleV1ErrorCode.REVERSED_INTERVAL)

    @classmethod
    def point(cls, value: Fraction | int) -> FractionIntervalV1:
        value = _fraction(value)
        return cls(value, value)

    @property
    def width(self) -> Fraction:
        return self.upper - self.lower

    def add(self, other: FractionIntervalV1) -> FractionIntervalV1:
        return FractionIntervalV1(self.lower + other.lower, self.upper + other.upper)

    def sub(self, other: FractionIntervalV1) -> FractionIntervalV1:
        return FractionIntervalV1(self.lower - other.upper, self.upper - other.lower)

    def mul(self, other: FractionIntervalV1) -> FractionIntervalV1:
        products = (self.lower * other.lower, self.lower * other.upper,
                    self.upper * other.lower, self.upper * other.upper)
        return FractionIntervalV1(min(products), max(products))

    def square(self) -> FractionIntervalV1:
        left, right = self.lower**2, self.upper**2
        if self.lower >= 0:
            return FractionIntervalV1(left, right)
        if self.upper <= 0:
            return FractionIntervalV1(right, left)
        return FractionIntervalV1(Fraction(0), max(left, right))

    def intersect(self, other: FractionIntervalV1) -> FractionIntervalV1:
        lower, upper = max(self.lower, other.lower), min(self.upper, other.upper)
        if lower > upper:
            raise AngleV1Error(AngleV1ErrorCode.EMPTY_INTERSECTION)
        return FractionIntervalV1(lower, upper)

    def positive_sqrt(self, bits: int) -> FractionIntervalV1:
        if self.lower < 0:
            raise AngleV1Error(AngleV1ErrorCode.NEGATIVE_RADICAND)
        _bits(bits, multiplier=2)
        lower, _ = _sqrt_endpoint(self.lower, bits)
        _, upper = _sqrt_endpoint(self.upper, bits)
        return FractionIntervalV1(lower, upper)


def _sqrt_endpoint(value: Fraction, bits: int) -> tuple[Fraction, Fraction]:
    scaled = value.numerator << (2 * bits)
    root = isqrt(scaled // value.denominator)
    exact = root * root * value.denominator == scaled
    divisor = 1 << bits
    return Fraction(root, divisor), Fraction(root + int(not exact), divisor)


@dataclass(frozen=True, slots=True)
class AngleStatsV1:
    polynomial_degree: int
    bisections: int
    sturm_polynomials: int = 0
    variation_evaluations: int = 0


@dataclass(frozen=True, slots=True)
class RootEnclosureV1:
    lower: Fraction
    upper: Fraction
    stats: AngleStatsV1


@dataclass(frozen=True, slots=True)
class RayEnclosureV1:
    cosine: FractionIntervalV1
    sine: FractionIntervalV1
    root: RootEnclosureV1


def root_bits(subdivision: int) -> int:
    if type(subdivision) is not int or not 2 <= subdivision <= 6:
        raise AngleV1Error(AngleV1ErrorCode.NO_HIDDEN_ORDINAL)
    return _ROOT_BITS[subdivision - 2]


def profile_bound_is_valid(subdivision: int) -> bool:
    if type(subdivision) is not int or not 2 <= subdivision <= 6:
        return False
    bits = root_bits(subdivision)
    return (2 * _LIPSCHITZ[subdivision - 2] <= 1 << (bits - 2 * COMPONENT_BITS - 4)
            and bits > COMPONENT_BITS + 3)


def _principal_floor(n: int, a: int, d: int) -> bool:
    if n == 2:
        return a <= 0
    if n == 3:
        return 2 * a <= d
    if n == 4:
        return a <= 0 or 2 * a * a <= d * d
    if n == 5:
        y = 4 * a - d
        return y <= 0 or y * y <= 5 * d * d
    return a <= 0 or 4 * a * a <= 3 * d * d


def _signed_root_compare(value: int, divisor: int, ratio: Fraction, sign: int) -> int:
    value_sign = (value > 0) - (value < 0)
    if not ratio:
        return value_sign
    if value_sign != sign:
        return (value_sign > sign) - (value_sign < sign)
    residual = ratio.denominator * value * value - ratio.numerator * divisor * divisor
    return ((residual > 0) - (residual < 0)) * sign


def _chebyshev_coefficients(n: int) -> tuple[int, ...]:
    previous, current = (1,), (0, 1)
    for _ in range(1, n):
        coefficients = [0] + [2 * value for value in current]
        for power, value in enumerate(previous):
            coefficients[power] -= value
        previous, current = current, tuple(coefficients)
    return current


@dataclass(frozen=True, slots=True)
class AngleAtomV1:
    subdivision: int
    ratio: Fraction
    dot_sign: int

    def __post_init__(self):
        if type(self.subdivision) is not int or not 1 <= self.subdivision <= 6:
            raise AngleV1Error(AngleV1ErrorCode.UNSUPPORTED_SUBDIVISION)
        ratio = _fraction(self.ratio)
        object.__setattr__(self, "ratio", ratio)
        if not 0 <= ratio < 1:
            raise AngleV1Error(AngleV1ErrorCode.RATIO_OUTSIDE_HALF_OPEN_UNIT_INTERVAL)
        if type(self.dot_sign) is not int or not (
                (not ratio and self.dot_sign == 0) or (ratio and self.dot_sign in (-1, 1))):
            raise AngleV1Error(AngleV1ErrorCode.INCONSISTENT_DOT_SIGN)

    def compare(self, coordinate: Fraction | int) -> int:
        """Знак x-alpha; отсечение чужой ветви предшествует равенству."""
        x = _fraction(coordinate)
        a, d, n = x.numerator, x.denominator, self.subdivision
        if n == 1:
            return _signed_root_compare(a, d, self.ratio, self.dot_sign)
        if _principal_floor(n, a, d):
            return -1
        previous, current = 1, a
        for _ in range(1, n):
            previous, current = current, 2 * a * current - d * d * previous
        return _signed_root_compare(current, d**n, self.ratio, self.dot_sign)

    def enclose(self, bits: int) -> RootEnclosureV1:
        _bits(bits, extra=1)
        lower, upper = Fraction(-1), Fraction(1)
        degree = self.subdivision * (2 if self.ratio else 1)
        for steps in range(1, bits + 2):
            middle = (lower + upper) / 2
            comparison = self.compare(middle)
            if comparison == 0:
                return RootEnclosureV1(middle, middle, AngleStatsV1(degree, steps))
            if comparison < 0:
                lower = middle
            else:
                upper = middle
        if not 0 <= upper - lower <= Fraction(1, 1 << bits):
            raise AngleV1Error(AngleV1ErrorCode.ENCLOSURE_INVARIANT)
        return RootEnclosureV1(lower, upper, AngleStatsV1(degree, bits + 1))

    def ray_enclosure(self, ordinal: int, bits: int) -> RayEnclosureV1:
        if type(ordinal) is not int or not 0 < ordinal < self.subdivision:
            raise AngleV1Error(AngleV1ErrorCode.INVALID_ORDINAL)
        _bits(bits, multiplier=2)
        root = self.enclose(bits)
        interval = FractionIntervalV1(root.lower, root.upper)
        cosine = FractionIntervalV1.point(0)
        for coefficient in reversed(_chebyshev_coefficients(ordinal)):
            cosine = cosine.mul(interval).add(FractionIntervalV1.point(coefficient))
        cosine = cosine.intersect(FractionIntervalV1(-1, 1))
        nonnegative_unit = FractionIntervalV1(0, 1)
        square = FractionIntervalV1.point(1).sub(cosine.square()).intersect(nonnegative_unit)
        sine = square.positive_sqrt(bits).intersect(nonnegative_unit)
        return RayEnclosureV1(cosine, sine, root)

    def ray(self, ordinal: int) -> RayEnclosureV1:
        result = self.ray_enclosure(ordinal, root_bits(self.subdivision))
        limit = Fraction(1, 1 << COMPONENT_BITS)
        if result.cosine.width >= limit or result.sine.width >= limit:
            raise AngleV1Error(AngleV1ErrorCode.WIDTH_PROOF_VIOLATED)
        return result
