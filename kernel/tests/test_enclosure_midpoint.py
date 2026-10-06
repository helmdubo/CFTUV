"""Округление `SqrtSumV1` целыми (`lift.enclosure_midpoint`, `sqrt_sum_binary64`, `decimal_of`, `uv_direct_strip_v1`) побитово равно прежнему.

Прежний путь строил `value.scaled(factor)`, брал его оболочку `Fraction`ами и делил: три нормировки дроби на каждое число. Новый берёт
те же `isqrt` радикандов целыми над другим общим знаменателем (`L * den(factor)`), поэтому пределы оболочки и знаменатель отличаются
ОБЩИМ множителем, а середина - та же дробь. Тест сверяет два пути на случайных суммах корней (знаки, нули, рациональные члены, разные
разрядности и множители) и на значениях полевых доменов: `float`, `Decimal` (значение и запись), и пределы оболочки.
"""

from __future__ import annotations

import random
from decimal import Decimal, localcontext
from fractions import Fraction

import pytest

import developable_factories as df
from developable_route import developable_domain
from materialize_factories import prepare_and_cover

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize.assemble import DECIMAL_CONTEXT, decimal_of
from cftuv_envelope.materialize.lift import ENCLOSURE_BITS, enclosure_midpoint, sqrt_sum_binary64
from cftuv_envelope.materialize.uv_law import uv_direct_strip_v1
from cftuv_envelope.wavefront import conveyor_coverage


def _squarefree(number: int) -> bool:
    return all(number % (factor * factor) for factor in range(2, int(number**0.5) + 1))


SQUAREFREE = tuple(number for number in range(1, 700) if _squarefree(number))
FACTORS = (None, Fraction(1), Fraction(1, 7), Fraction(5, 3), Fraction(-3, 11), Fraction(0), Fraction(1, 10**9 + 7), Fraction(123456789, 1000))
BITS = (ENCLOSURE_BITS, 40, 100)


def reference(value: SqrtSumV1, bits: int, factor):
    """Прежний путь: `scaled(factor)`, оболочка `Fraction`ами, середина дробью."""

    scaled = value if factor is None else value.scaled(factor)
    low, high = scaled.enclosure(bits)
    return low, high, (low + high) / 2


def random_sum(generator: random.Random) -> SqrtSumV1:
    radicands = generator.sample(SQUAREFREE, generator.randint(0, 5))
    terms = []
    for radicand in sorted(radicands):
        numerator = generator.randint(-(10**generator.randint(1, 14)), 10**generator.randint(1, 14))
        if numerator:
            terms.append((radicand, Fraction(numerator, generator.randint(1, 10 ** generator.randint(0, 9)))))
    return SqrtSumV1(tuple(terms))


def old_decimal(value: SqrtSumV1, divisor: int) -> Decimal:
    low, high = value.scaled(Fraction(1, divisor)).enclosure(ENCLOSURE_BITS)
    middle = (low + high) / 2
    with localcontext(DECIMAL_CONTEXT):
        return Decimal(middle.numerator) / Decimal(middle.denominator)


def old_uv(s: SqrtSumV1, r: SqrtSumV1, lattice_alpha):
    inverse = Fraction(1) / Fraction(lattice_alpha)
    return (float((lambda b: (b[0] + b[1]) / 2)(s.scaled(inverse).enclosure(ENCLOSURE_BITS))), float((lambda b: (b[0] + b[1]) / 2)(r.scaled(inverse).enclosure(ENCLOSURE_BITS))))


def test_the_midpoint_is_the_same_fraction_as_the_old_path_on_random_sums():
    generator = random.Random(20261006)
    checked = 0
    for _ in range(4000):
        value = random_sum(generator)
        for factor in FACTORS:
            for bits in BITS:
                numerator, denominator = enclosure_midpoint(value, bits, factor)
                _low, _high, middle = reference(value, bits, factor)
                assert Fraction(numerator, denominator) == middle, (value, factor, bits)
                assert numerator / denominator == float(middle), (value, factor, bits)
                checked += 1
    assert checked == 4000 * len(FACTORS) * len(BITS)


def test_the_binary64_the_decimal_and_the_uv_are_the_same_on_random_sums():
    generator = random.Random(7)
    for _ in range(3000):
        value, other = random_sum(generator), random_sum(generator)
        assert sqrt_sum_binary64(value) == float(reference(value, ENCLOSURE_BITS, None)[2])
        divisor = generator.choice((1, 3, 1000, 10**6, 844687660141))
        new, old = decimal_of(value, divisor), old_decimal(value, divisor)
        assert new == old and repr(new) == repr(old) and str(new) == str(old), (value, divisor)
        alpha = Fraction(generator.randint(1, 10**7), generator.randint(1, 10**5))
        uv = uv_direct_strip_v1(value, other, alpha)
        assert (uv.u, uv.v) == old_uv(value, other, alpha)


def test_the_zero_the_rational_and_the_negative_edge_cases():
    zero = SqrtSumV1.zero()
    assert enclosure_midpoint(zero)[0] == 0 and sqrt_sum_binary64(zero) == 0.0
    assert sqrt_sum_binary64(SqrtSumV1.rational(Fraction(-7, 3))) == float(Fraction(-7, 3))
    root = SqrtSumV1(((2, Fraction(-1)),))
    assert sqrt_sum_binary64(root) == float(reference(root, ENCLOSURE_BITS, None)[2])
    assert sqrt_sum_binary64(root, factor=Fraction(0)) == 0.0
    assert repr(decimal_of(zero, 5)) == repr(old_decimal(zero, 5))


@pytest.mark.parametrize("make,alpha", [(df.fold_strip, "0.3"), (df.slant_fold, "0.7"), (df.quarter_cylinder, "0.7")])
def test_the_values_of_a_developable_domain_round_the_same(make, alpha):
    snapshot, request = developable_domain(make(), ("r0a", "r0b"), alpha="1")
    prepared, _ = prepare_and_cover(snapshot, request)
    coverage = conveyor_coverage(prepared, alpha)
    scale = 1 if prepared.lattice is None else prepared.lattice.scale
    count = 0
    for region in prepared.regions:
        for face in region.partition.faces:
            for point in face.points:
                for coordinate in point:
                    assert sqrt_sum_binary64(coordinate, factor=Fraction(1, scale)) == float(reference(coordinate, ENCLOSURE_BITS, Fraction(1, scale))[2])
                    assert decimal_of(coordinate, scale) == old_decimal(coordinate, scale)
                    count += 1
    assert count > 0 and coverage.outcome.value == "EXACT"
