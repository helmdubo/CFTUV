"""Private exact algebra for direction-binding window predicates."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import isqrt

import sympy as sp


class _ExactWindowUnsupported(ValueError):
    """Private exact field does not represent the supplied construction."""


def _fraction(value: sp.Expr) -> Fraction:
    if value.is_Rational is not True:
        raise _ExactWindowUnsupported("direction-binding metric is not rational")
    return Fraction(int(value.p), int(value.q))


def _rational_square_root(value: Fraction) -> Fraction | None:
    if value < 0:
        raise _ExactWindowUnsupported("negative exact-window radicand")
    numerator = isqrt(value.numerator)
    denominator = isqrt(value.denominator)
    if (
        numerator * numerator != value.numerator
        or denominator * denominator != value.denominator
    ):
        return None
    return Fraction(numerator, denominator)


def _sqrt_fraction_enclosure(
    value: Fraction,
    bits: int,
) -> tuple[Fraction, Fraction]:
    """Наружная рациональная оболочка sqrt(value) на сетке 2**-bits."""

    if value < 0:
        raise _ExactWindowUnsupported("negative exact-window radicand")
    scale = 1 << bits
    scaled_numerator = value.numerator * scale * scale
    floor_root = isqrt(scaled_numerator // value.denominator)
    lower = Fraction(floor_root, scale)
    if (
        floor_root * floor_root * value.denominator
        == scaled_numerator
    ):
        return lower, lower
    return lower, Fraction(floor_root + 1, scale)


def _interval_product(
    left: tuple[Fraction, Fraction],
    right: tuple[Fraction, Fraction],
) -> tuple[Fraction, Fraction]:
    products = (
        left[0] * right[0],
        left[0] * right[1],
        left[1] * right[0],
        left[1] * right[1],
    )
    return min(products), max(products)


class _MQField:
    """Контекстный multiquadratic basis; process-global cache не нужен."""

    def __init__(self) -> None:
        self.generators: list[Fraction] = []

    def rational(self, value: Fraction | int) -> _MQValue:
        value = Fraction(value)
        return _MQValue(self, ((0, value),) if value else ())

    def radical(self, value: Fraction | int) -> _MQValue:
        value = Fraction(value)
        direct = _rational_square_root(value)
        if direct is not None:
            return self.rational(direct)
        for mask in range(1, 1 << len(self.generators)):
            product = Fraction(1)
            for index, generator in enumerate(self.generators):
                if mask & (1 << index):
                    product *= generator
            coefficient = _rational_square_root(value / product)
            if coefficient is not None:
                return _MQValue(self, ((mask, coefficient),))
        mask = 1 << len(self.generators)
        self.generators.append(value)
        return _MQValue(self, ((mask, Fraction(1)),))


@dataclass(frozen=True, slots=True)
class _MQValue:
    field: _MQField
    terms: tuple[tuple[int, Fraction], ...]

    @staticmethod
    def _from_map(
        field: _MQField,
        terms: dict[int, Fraction],
    ) -> _MQValue:
        return _MQValue(
            field,
            tuple(sorted((mask, value) for mask, value in terms.items() if value)),
        )

    @property
    def is_zero(self) -> bool:
        return not self.terms

    def as_rational(self) -> Fraction | None:
        if any(mask != 0 for mask, _ in self.terms):
            return None
        return dict(self.terms).get(0, Fraction(0))

    def __add__(self, other: _MQValue) -> _MQValue:
        _same_field(self, other)
        result = dict(self.terms)
        for mask, value in other.terms:
            result[mask] = result.get(mask, Fraction(0)) + value
        return _MQValue._from_map(self.field, result)

    def __neg__(self) -> _MQValue:
        return _MQValue(
            self.field,
            tuple((mask, -value) for mask, value in self.terms),
        )

    def __sub__(self, other: _MQValue) -> _MQValue:
        return self + (-other)

    def scaled(self, factor: Fraction | int) -> _MQValue:
        factor = Fraction(factor)
        return _MQValue._from_map(
            self.field,
            {mask: value * factor for mask, value in self.terms},
        )

    def __mul__(self, other: _MQValue) -> _MQValue:
        _same_field(self, other)
        result: dict[int, Fraction] = {}
        for left_mask, left_value in self.terms:
            for right_mask, right_value in other.terms:
                overlap = left_mask & right_mask
                factor = Fraction(1)
                for index, generator in enumerate(self.field.generators):
                    if overlap & (1 << index):
                        factor *= generator
                mask = left_mask ^ right_mask
                result[mask] = result.get(mask, Fraction(0)) + (
                    left_value * right_value * factor
                )
        return _MQValue._from_map(self.field, result)

    def enclosure(self, bits: int) -> tuple[Fraction, Fraction]:
        lower = upper = Fraction(0)
        for mask, coefficient in self.terms:
            radicand = Fraction(1)
            for index, generator in enumerate(self.field.generators):
                if mask & (1 << index):
                    radicand *= generator
            root = _sqrt_fraction_enclosure(radicand, bits)
            term = (
                (root[0] * coefficient, root[1] * coefficient)
                if coefficient > 0
                else (root[1] * coefficient, root[0] * coefficient)
            )
            lower += term[0]
            upper += term[1]
        return lower, upper

    def sign(self) -> int:
        return _mq_exact_sign(self)


def _same_field(left, right) -> None:
    if left.field is not right.field:
        raise _ExactWindowUnsupported("unrelated exact-window fields")


def _mq_exact_sign(value: _MQValue) -> int:
    if value.is_zero:
        return 0
    rational = value.as_rational()
    if rational is not None:
        return (rational > 0) - (rational < 0)
    highest = max(mask.bit_length() for mask, _ in value.terms) - 1
    bit = 1 << highest
    outside: dict[int, Fraction] = {}
    inside: dict[int, Fraction] = {}
    for mask, coefficient in value.terms:
        target = inside if mask & bit else outside
        key = mask ^ bit if mask & bit else mask
        target[key] = target.get(key, Fraction(0)) + coefficient
    left = _MQValue._from_map(value.field, outside)
    right = _MQValue._from_map(value.field, inside)
    left_sign = _mq_exact_sign(left)
    right_sign = _mq_exact_sign(right)
    if not left_sign:
        return right_sign
    if not right_sign or left_sign == right_sign:
        return left_sign
    discriminant = left * left - (right * right).scaled(
        value.field.generators[highest]
    )
    discriminant_sign = _mq_exact_sign(discriminant)
    if not discriminant_sign:
        return 0
    return left_sign if discriminant_sign > 0 else right_sign


class _TowerField:
    """Одна exact quadratic extension над MQ либо предыдущей ступенью."""

    def __init__(self, base, radicand) -> None:
        self.base = base
        self.radicand = radicand
        if radicand.sign() <= 0:
            raise _ExactWindowUnsupported("non-positive exact-window extension")

    def rational(self, value: Fraction | int) -> _TowerValue:
        return _TowerValue(
            self,
            self.base.rational(value),
            self.base.rational(0),
        )

    def promote(self, value) -> _TowerValue:
        if value.field is not self.base:
            raise _ExactWindowUnsupported("cannot promote unrelated exact value")
        return _TowerValue(self, value, self.base.rational(0))

    def root(self) -> _TowerValue:
        return _TowerValue(
            self,
            self.base.rational(0),
            self.base.rational(1),
        )


@dataclass(frozen=True, slots=True)
class _TowerValue:
    field: _TowerField
    rational_part: object
    radical_part: object

    @property
    def is_zero(self) -> bool:
        return self.rational_part.is_zero and self.radical_part.is_zero

    def as_rational(self) -> Fraction | None:
        if not self.radical_part.is_zero:
            return None
        return self.rational_part.as_rational()

    def __add__(self, other: _TowerValue) -> _TowerValue:
        _same_field(self, other)
        return _TowerValue(
            self.field,
            self.rational_part + other.rational_part,
            self.radical_part + other.radical_part,
        )

    def __neg__(self) -> _TowerValue:
        return _TowerValue(
            self.field,
            -self.rational_part,
            -self.radical_part,
        )

    def __sub__(self, other: _TowerValue) -> _TowerValue:
        return self + (-other)

    def scaled(self, factor: Fraction | int) -> _TowerValue:
        return _TowerValue(
            self.field,
            self.rational_part.scaled(factor),
            self.radical_part.scaled(factor),
        )

    def __mul__(self, other: _TowerValue) -> _TowerValue:
        _same_field(self, other)
        rational = (
            self.rational_part * other.rational_part
            + (self.radical_part * other.radical_part) * self.field.radicand
        )
        radical = (
            self.rational_part * other.radical_part
            + self.radical_part * other.rational_part
        )
        return _TowerValue(self.field, rational, radical)

    def enclosure(self, bits: int) -> tuple[Fraction, Fraction]:
        rational = self.rational_part.enclosure(bits)
        coefficient = self.radical_part.enclosure(bits)
        radicand = self.field.radicand.enclosure(bits)
        if radicand[0] < 0:
            raise _ExactWindowUnsupported("extension enclosure crossed zero")
        root = (
            _sqrt_fraction_enclosure(radicand[0], bits)[0],
            _sqrt_fraction_enclosure(radicand[1], bits)[1],
        )
        product = _interval_product(coefficient, root)
        return rational[0] + product[0], rational[1] + product[1]

    def sign(self) -> int:
        left_sign = self.rational_part.sign()
        right_sign = self.radical_part.sign()
        if not left_sign:
            return right_sign
        if not right_sign or left_sign == right_sign:
            return left_sign
        discriminant = (
            self.rational_part * self.rational_part
            - (
                self.radical_part * self.radical_part
            ) * self.field.radicand
        )
        discriminant_sign = discriminant.sign()
        if not discriminant_sign:
            return 0
        return left_sign if discriminant_sign > 0 else right_sign
