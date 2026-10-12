"""`RadicalSumV2` (`reference/radical_v2.py`): the theorems the type stands on, as checks that fail.

* ZERO (Besicovitch): after the class merge a sum is zero iff it has no terms; equal values in different representations and
  cancellations give the empty sum, nonzero ones never do.
* SEPARATION BOUND: `|x| >= 1 / (L U^e)`, `e = 2^(k-1) - 1`, proved in the module docstring; here it is checked against the
  brute-force half-conjugate product (a nonzero rational integer) and on constructive near-zero families whose sign is known
  independently, including one that V1's fixed 4096-bit ladder refuses.
* INVERSE by conjugation in `m'` order, canonical form uniqueness (A0), text and wire round trips, V1 <-> V2 by value, and
  independence of `PYTHONHASHSEED`.
"""

from __future__ import annotations

import ast
import hashlib
import json
import os
import random
import subprocess
import sys
from fractions import Fraction
from math import isqrt

import mpmath
import pytest

from cftuv_envelope.reference import native_exact
from cftuv_envelope.reference.native_exact import NativeSignUndecided, RadicalSumV1
from cftuv_envelope.reference.radical_v2 import (
    EXACT_NATIVE_INVERSE_CLASS_LIMIT,
    EXACT_NATIVE_OUTSIDE_FIELD,
    EXACT_NATIVE_SIGN_UNDECIDED,
    NO_WORK,
    NativeAlgebraWorkV1,
    RadicalInverseClassLimit,
    RadicalOutsideField,
    RadicalSignUndecided,
    RadicalSumV2,
    _integer_form,
    canonical_text,
    from_v1,
    separation_bits,
    terms_wire,
    to_v1,
    value_of_text,
    value_of_wire,
)
from kernel_test_paths import PACKAGE_ROOT

SQUAREFREE = (2, 3, 5, 6, 7, 10, 11, 13, 14, 15, 17, 19, 21, 22, 23, 26, 29, 30, 31)


def value(*pairs) -> RadicalSumV2:
    return RadicalSumV2.from_terms(pairs)[0]


def random_value(rng: random.Random, classes: int, *, rational: bool = True, size: int = 9) -> RadicalSumV2:
    radicands = rng.sample(SQUAREFREE, classes)
    pairs = [(radicand, Fraction(rng.choice((-1, 1)) * rng.randint(1, size), rng.randint(1, size))) for radicand in radicands]
    if rational:
        pairs.append((1, Fraction(rng.randint(-size, size), rng.randint(1, size))))
    return value(*pairs)


def hidden_pairs(x: RadicalSumV2, rng: random.Random) -> list:
    """`(r, c)` pairs of the value `x` with squares hidden in the radicands and every class split in random parts, shuffled."""

    pairs = []
    for radicand, coefficient in x.pairs():
        parts = rng.randint(1, 3)
        remaining = coefficient
        for index in range(parts):
            share = remaining if index == parts - 1 else remaining * Fraction(rng.randint(-5, 5), rng.randint(1, 7))
            remaining -= share
            square = rng.choice((1, 2, 3, 5, 6, 10))
            pairs.append((radicand * square * square, share / square))
    rng.shuffle(pairs)
    return pairs


def identity(x: RadicalSumV2) -> tuple:
    """Everything that must be a function of the value: terms, text, wire, hash, the integer form and an enclosure."""

    common, items = _integer_form(x.terms)
    return (x.terms, canonical_text(x), terms_wire(x), hash(x), common, tuple(items), x.enclosure(160)[0])


# --------------------------------------------------------------------------
# ZERO (Besicovitch)
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "pairs",
    [
        [(2, 1), (8, 1), (18, -1)],  # sqrt2 + 2 sqrt2 - 3 sqrt2
        [(12, 1), (3, -2)],
        [(24, Fraction(1, 6)), (6, Fraction(-1, 3))],  # sqrt24/6 = sqrt6/3
        [(50, Fraction(3, 5)), (2, -3)],
        [(1, 5), (4, Fraction(-5, 2))],  # a square radicand is rational
        [(72, 1), (8, -3)],
    ],
)
def test_cancellations_across_representatives_are_the_empty_sum(pairs):
    x, _ = RadicalSumV2.from_terms(pairs)
    assert x.is_zero and x.terms == () and canonical_text(x) == "Integer(0)"
    assert x.signum() == (0, NO_WORK)


def test_products_that_cancel_are_the_empty_sum():
    one_plus, _ = RadicalSumV2.rational(1).add(value((2, 1)))
    one_minus, _ = RadicalSumV2.rational(1).sub(value((2, 1)))
    product, _ = one_plus.mul(one_minus)
    assert product == RadicalSumV2.rational(-1)
    s2, s3 = value((2, 1)), value((3, 1))
    square, _ = s2.add(s3)[0].mul(s2.add(s3)[0])  # 5 + 2 sqrt6
    rest, _ = square.sub(RadicalSumV2.rational(5))
    six, _ = s2.mul(s3)
    assert rest.sub(six.scaled(2))[0].is_zero


def test_equal_values_in_different_representations_are_one_value():
    forms = [
        [(18, 1)], [(2, 3)], [(8, Fraction(3, 2))], [(2, 1), (8, 1)], [(50, Fraction(3, 5))], [(2, 5), (8, -1)],
        [(32, Fraction(3, 4))], [(72, Fraction(1, 2))],
    ]
    values = [RadicalSumV2.from_terms(pairs)[0] for pairs in forms]
    assert len({identity(x) for x in values}) == 1
    assert values[0].terms == ((18, 1, 1),) and canonical_text(values[0]) == "Sqrt(Rational(18, 1))"


def test_the_zero_test_is_besicovitch_randomised():
    """`x - y` over random routes to the same value is empty; a random nonzero perturbation of one class never is."""

    rng = random.Random(1940)
    for _ in range(400):
        x = random_value(rng, rng.randint(1, 5))
        y, _ = RadicalSumV2.from_terms(hidden_pairs(x, rng))
        difference, _ = x.sub(y)
        assert difference.is_zero and x == y and hash(x) == hash(y)
        radicand = rng.choice(SQUAREFREE)
        bump, _ = RadicalSumV2.sqrt_of_rational(Fraction(radicand, rng.randint(1, 50)))
        moved, _ = y.add(bump.scaled(Fraction(1, 10**12)))
        assert not moved.sub(x)[0].is_zero and moved != x


# --------------------------------------------------------------------------
# SEPARATION BOUND
# --------------------------------------------------------------------------


def _formal_product(left: dict, right: dict, radicands: tuple) -> dict:
    """Product in `Z[z_1..z_k] / (z_i^2 - m_i)`: a monomial is a bit mask; no class merging, no `RadicalSumV2`."""

    out: dict = {}
    for mask_a, a in left.items():
        for mask_b, b in right.items():
            coefficient = a * b
            both = mask_a & mask_b
            index = 0
            while both:
                if both & 1:
                    coefficient *= radicands[index]
                both >>= 1
                index += 1
            mask = mask_a ^ mask_b
            out[mask] = out.get(mask, 0) + coefficient
    return {mask: c for mask, c in out.items() if c}


def half_conjugate_product(items: list) -> int:
    """`prod_{e in {+-1}^(k-1)} (a_1 sqrt m_1 + sum e_i a_i sqrt m_i)` expanded formally: the integer `N` of the bound's proof."""

    radicands = tuple(radicand for radicand, _ in items)
    total = {0: 1}
    for pattern in range(1 << (len(items) - 1)):
        factor = {1: items[0][1]}
        for index in range(1, len(items)):
            sign = -1 if pattern >> (index - 1) & 1 else 1
            factor[1 << index] = sign * items[index][1]
        total = _formal_product(total, factor, radicands)
    assert set(total) <= {0}, f"the half-conjugate product is not rational: {total}"
    return total.get(0, 0)


def test_the_separation_bound_holds_against_the_brute_force_conjugate_product():
    """For random sums of 3..5 classes: `N` is a nonzero integer, `|y| * U^e >= |N| >= 1`, and the sign at `separation_bits` is V1's."""

    rng = random.Random(2001)
    mpmath.mp.dps = 80
    for _ in range(120):
        classes = rng.randint(2, 4)
        x = random_value(rng, classes, rational=rng.random() < 0.7, size=40)
        if len(x.terms) < 3:
            continue
        common, items = _integer_form(x.terms)
        product = half_conjugate_product(items)
        assert isinstance(product, int) and product != 0
        upper = sum(abs(a) * (isqrt(m) + 1) if m != 1 else abs(a) for m, a in items)
        exponent = (1 << (len(items) - 1)) - 1
        y = mpmath.fsum(a * mpmath.sqrt(m) for m, a in items)
        assert abs(y) * mpmath.mpf(upper) ** exponent >= 1
        sign, work = x.signum()
        assert sign == (1 if y > 0 else -1) == to_v1(x).signum()
        bits = separation_bits(items)
        assert work.enclosure_rounds >= 1 and (2**bits) > sum(abs(a) for m, a in items if m != 1) * upper**exponent


def pell_powers(n: int, m: int) -> tuple[tuple[int, int], tuple[int, int]]:
    """`(sqrt2 - 1)^n = a + b sqrt2` and `(2 - sqrt3)^m = c + d sqrt3`, exact integers (both values in (0, 1))."""

    a, b = 1, 0
    for _ in range(n):
        a, b = -a + 2 * b, a - b
    c, d = 1, 0
    for _ in range(m):
        c, d = 2 * c - 3 * d, 2 * d - c
    return (a, b), (c, d)


@pytest.mark.parametrize("n,m", [(3, 2), (6, 4), (9, 6), (25, 17), (40, 27), (97, 65), (300, 201)])
def test_near_zero_sums_get_the_right_sign_within_the_bound(n, m):
    """Constructive near-zero family over three and four classes, with a sign known WITHOUT evaluating the sum.

    `u = (sqrt2 - 1)^n` and `v = (2 - sqrt3)^m` are tiny positive units (coefficients grow like their inverses):
    `u + v > 0` and `u v > 0` (classes 1, 2, 3 and 1, 2, 3, 6), while `sign(u - v) = sign(m ln(2 + sqrt3) - n ln(sqrt2 + 1))`,
    and `m/n` near `ln(sqrt2 + 1)/ln(2 + sqrt3)` makes `u - v` tiny relative to `u`. The ladder must reach the sign at no more
    than `separation_bits`, and the bigger members need more than the first 64-bit rung.
    """

    (a, b), (c, d) = pell_powers(n, m)
    mpmath.mp.dps = 60
    expected_difference = 1 if m * mpmath.log(2 + mpmath.sqrt(3)) > n * mpmath.log(mpmath.sqrt(2) + 1) else -1
    cases = [
        (value((1, a + c), (2, b), (3, d)), 1),
        (value((1, a - c), (2, b), (3, -d)), expected_difference),
        (value((1, a * c), (2, b * c), (3, a * d), (6, b * d)), 1),
    ]
    for x, expected in cases:
        _, items = _integer_form(x.terms)
        required = separation_bits(items)
        sign, work = x.signum()
        assert sign == expected
        if work.enclosure_rounds:
            assert len(x.terms) >= 3 and min(64 << (work.enclosure_rounds - 1), required) <= required
        else:
            assert len(x.terms) <= 2  # a rational part that cancelled: the squares decide
    if n >= 40:
        assert work.enclosure_rounds > 1, "the near-zero member must climb the ladder"


def test_v2_decides_where_the_v1_ladder_refuses():
    """`(sqrt2 - 1)^n + (2 - sqrt3)^m > 0` near `2^-3800` and `2^-5100`: V1's 4096-bit ladder refuses both by name; V2 decides both.

    The smaller one V2 decides inside V1's own seven rungs: a value-canonical term `s sqrt(m')/Q` puts the big coefficient INSIDE the
    root, so its enclosure is off by `1/Q` units instead of V1's `|c|`. The bigger one needs the eighth rung (8192 bits), past V1's cap
    and below the separation bound; a caller's cap of 4096 bits turns it into the named price refusal, carrying the work done.
    """

    rungs = len(native_exact._SIGN_BITS)
    for (n, m), beyond in (((3000, 2000), False), ((4000, 2700), True)):
        (a, b), (c, d) = pell_powers(n, m)
        pairs = ((1, Fraction(a + c)), (2, Fraction(b)), (3, Fraction(d)))
        with pytest.raises(NativeSignUndecided):
            RadicalSumV1(pairs).signum()
        x, _ = RadicalSumV2.from_terms(pairs)
        sign, work = x.signum()
        assert sign == 1 and (work.enclosure_rounds > rungs) == beyond
        assert (64 << (work.enclosure_rounds - 1)) < separation_bits(_integer_form(x.terms)[1])
    with pytest.raises(RadicalSignUndecided) as caught:
        x.signum(precision_cap_bits=4096)
    assert caught.value.code == EXACT_NATIVE_SIGN_UNDECIDED and caught.value.work.enclosure_rounds == rungs


# --------------------------------------------------------------------------
# INVERSE
# --------------------------------------------------------------------------


def test_inverse_by_conjugation_is_exact_and_canonical():
    rng = random.Random(1983)
    one = RadicalSumV2.rational(1)
    for _ in range(300):
        x = random_value(rng, rng.randint(1, 3), rational=rng.random() < 0.5)
        if not x.terms or len(x.terms) > 3:
            continue
        inverse, work = x.inverse()
        assert x.mul(inverse)[0] == one
        if len(inverse.terms) <= 3:  # the inverse of three classes can have four (1, 2, 3, 6): not invertible again by conjugation
            assert inverse.inverse()[0] == x
        assert inverse == RadicalSumV2.from_terms(inverse.pairs())[0]
        conjugations = 0 if len(x.terms) == 1 else (2 if len(x.terms) == 2 or _norm_is_rational(x) else 4)
        assert work.conjugation_products == conjugations
        assert from_v1(to_v1(x).inverse())[0] == inverse


def _norm_is_rational(x: RadicalSumV2) -> bool:
    rest = RadicalSumV2(x.terms[:-1])
    pivot = x.terms[-1]
    square = rest.mul(rest)[0]
    norm = square.sub(Fraction(pivot[0], pivot[2] * pivot[2]))[0]
    return norm.as_rational() is not None


def test_inverse_refusals_are_named():
    four = value((1, 1), (2, 1), (3, 1), (5, 1))
    with pytest.raises(RadicalInverseClassLimit) as caught:
        four.inverse()
    assert caught.value.code == EXACT_NATIVE_INVERSE_CLASS_LIMIT
    with pytest.raises(ZeroDivisionError):
        RadicalSumV2.zero().inverse()
    with pytest.raises(RadicalOutsideField) as outside:
        RadicalSumV2.sqrt_of_rational(Fraction(-1, 2))
    assert outside.value.code == EXACT_NATIVE_OUTSIDE_FIELD


# --------------------------------------------------------------------------
# CANONICAL FORM (A0)
# --------------------------------------------------------------------------


def test_the_representation_is_a_function_of_the_value():
    """Random operation orders over one value give ONE form: terms, text, wire, hash, integer form and enclosure (A0 acceptance)."""

    rng = random.Random(4096)
    for _ in range(150):
        x = random_value(rng, rng.randint(1, 4))
        helper = random_value(rng, rng.randint(1, 2), rational=False)
        factor = Fraction(rng.randint(1, 30), rng.randint(1, 30))
        routes = [
            RadicalSumV2.from_terms(hidden_pairs(x, rng))[0],
            RadicalSumV2.from_terms(hidden_pairs(x, rng))[0].add(helper)[0].sub(helper)[0],
            helper.add(RadicalSumV2.from_terms(hidden_pairs(x, rng))[0])[0].sub(helper)[0],
            x.scaled(factor).scaled(1 / factor),
            x.mul(helper)[0].div(helper)[0],
            value_of_text(canonical_text(x))[0],
            value_of_wire(terms_wire(x)),
            from_v1(to_v1(x))[0],
        ]
        assert {identity(route) for route in routes} == {identity(x)}


def test_terms_are_value_canonical_triples_in_m_order():
    rng = random.Random(7)
    for _ in range(200):
        x = random_value(rng, rng.randint(0, 5))
        radicands = [m for m, _, _ in x.terms]
        assert radicands == sorted(radicands) and len(set(radicands)) == len(radicands)
        for index, (m, n, d) in enumerate(x.terms):
            assert d > 0 and n
            if m == 1:
                assert index == 0
                continue
            assert abs(n) == 1 and m % d == 0
            p = m // d
            assert Fraction(p, d) == (Fraction(n, d) ** 2) * m and isqrt(m) ** 2 != m
        for left in range(len(x.terms)):
            for right in range(left + 1, len(x.terms)):
                product = x.terms[left][0] * x.terms[right][0]
                assert isqrt(product) ** 2 != product, "two terms of one class"


# --------------------------------------------------------------------------
# TEXT, WIRE, V1
# --------------------------------------------------------------------------


def test_text_and_wire_round_trip_and_the_single_term_text_is_canon_v2():
    rng = random.Random(5)
    for _ in range(300):
        x = random_value(rng, rng.randint(0, 4), rational=rng.random() < 0.5)
        text = canonical_text(x)
        assert canonical_text(value_of_text(text)[0]) == text and value_of_text(text)[0] == x
        assert value_of_wire(terms_wire(x)) == x
        if len(x.terms) <= 1:
            assert text == native_exact.canonical_text(to_v1(x))
        else:
            assert text.startswith("Sum(") and text.count("Sqrt(") + text.count("Integer(") + text.count("Rational(") - text.count("Sqrt(Rational(") == len(x.terms)
    # m' order: -sqrt2 is (2, -1, 1), sqrt3/2 = sqrt(3/4) is (12, 1, 4)
    assert canonical_text(value((1, Fraction(-3, 4)), (2, -1), (3, Fraction(1, 2)))) == "Sum(Rational(-3, 4), -Sqrt(Rational(2, 1)), Sqrt(Rational(3, 4)))"
    with pytest.raises(ValueError):
        value_of_wire((2, 1, 1, 8, 1, 1)[:5])
    with pytest.raises(ValueError):
        value_of_wire((8, 2, 1))  # a coefficient that is not +-1/Q
    with pytest.raises(ValueError):
        value_of_text("Sqrt(Rational(2, 1)) + Integer(1)")


def test_v1_operations_and_v2_operations_agree_by_value():
    """A small SHADOW in the suite: every V1 operation on random V1 values (random representatives) equals V2's by value."""

    rng = random.Random(311)
    for _ in range(250):
        left_v2 = random_value(rng, rng.randint(1, 3))
        right_v2 = random_value(rng, rng.randint(1, 2))
        left = RadicalSumV1.zero()
        for radicand, coefficient in hidden_pairs(left_v2, rng):
            left = left + RadicalSumV1.sqrt_of_rational(radicand).scaled(coefficient)
        right = RadicalSumV1.zero()
        for radicand, coefficient in hidden_pairs(right_v2, rng):
            right = right + RadicalSumV1.sqrt_of_rational(radicand).scaled(coefficient)
        assert from_v1(left)[0] == left_v2 and from_v1(right)[0] == right_v2
        assert from_v1(left + right)[0] == left_v2.add(right_v2)[0]
        assert from_v1(left - right)[0] == left_v2.sub(right_v2)[0]
        assert from_v1(left * right)[0] == left_v2.mul(right_v2)[0]
        assert from_v1(left.scaled(Fraction(3, 7)))[0] == left_v2.scaled(Fraction(3, 7))
        if right.terms and len(right.terms) <= 3:
            assert from_v1(left / right)[0] == left_v2.div(right_v2)[0]
        assert left.signum() == left_v2.signum()[0]
        assert (left.as_rational() is None) == (left_v2.as_rational() is None)
        assert to_v1(left_v2) == left


def test_the_price_is_returned_never_written():
    x = value((1, 1), (2, 1), (3, 1))
    y = value((8, 1), (27, 1))
    total, work = x.add(y)
    assert total == value((1, 1), (2, 3), (3, 4)) and work == NativeAlgebraWorkV1(class_merge_checks=2)
    assert work.articles() == {"CLASS_MERGE_CHECKS": 2, "CONJUGATION_PRODUCTS": 0, "ENCLOSURE_ROUNDS": 0}
    assert (work + NativeAlgebraWorkV1(0, 1, 2)).wire() == (2, 1, 2)
    assert x.enclosure(64)[1] == NativeAlgebraWorkV1(enclosure_rounds=1)
    assert x.scaled(5) == value((1, 5), (2, 5), (3, 5)) and x.neg().neg() == x


def _priced(name: str):
    one_plus_root2 = value((1, 1), (2, 1))
    three = value((1, 1), (2, 1), (3, 1))
    odd = value((1, Fraction(1, 2)), (5, Fraction(-3, 7)), (7, 2))
    return {
        "from_terms": lambda: RadicalSumV2.from_terms([(2, 1), (8, 1), (3, 1), (12, 1)]),
        "mul": lambda: one_plus_root2.mul(value((1, 1), (3, 1))),
        "inverse3": lambda: three.inverse(),
        "inverse3_norm2": lambda: odd.inverse(),
        "div": lambda: odd.div(one_plus_root2),
        "add_equal_m": lambda: value((2, 1), (3, 1)).add(value((8, Fraction(1, 2)), (27, Fraction(-1, 3)))),
        "sqrt": lambda: RadicalSumV2.sqrt_of_rational(Fraction(8, 9)),
        "sqrt_square": lambda: RadicalSumV2.sqrt_of_rational(Fraction(4, 9)),
    }[name]()


#: The mirror's vectors: canonical text and work wire `(CLASS_MERGE_CHECKS, CONJUGATION_PRODUCTS, ENCLOSURE_ROUNDS)` of fixed operations.
PRICE_VECTORS = {
    "from_terms": ("Sum(Sqrt(Rational(18, 1)), Sqrt(Rational(27, 1)))", (8, 0, 0)),
    "mul": ("Sum(Integer(1), Sqrt(Rational(2, 1)), Sqrt(Rational(3, 1)), Sqrt(Rational(6, 1)))", (4, 0, 0)),
    "inverse3": ("Sum(Rational(1, 2), Sqrt(Rational(1, 8)), -Sqrt(Rational(3, 8)))", (3, 2, 0)),
    "inverse3_norm2": (
        "Sum(Rational(-183554, 9207267), -Sqrt(Rational(4216540160, 84773765609289)), Sqrt(Rational(112494000080, 84773765609289)), "
        "Sqrt(Rational(3305475984832, 84773765609289)))",
        (6, 4, 0),
    ),
    "div": ("Sum(Rational(-1, 2), Sqrt(Rational(1, 2)), -Sqrt(Rational(28, 1)), Sqrt(Rational(56, 1)), Sqrt(Rational(45, 49)), -Sqrt(Rational(90, 49)))", (12, 2, 0)),
    "add_equal_m": ("Sqrt(Rational(8, 1))", (0, 0, 0)),
    "sqrt": ("Sqrt(Rational(8, 9))", (1, 0, 0)),
    "sqrt_square": ("Rational(2, 3)", (1, 0, 0)),
}


@pytest.mark.parametrize("name", sorted(PRICE_VECTORS))
def test_price_vectors_for_the_mirror(name):
    """The counting rules of the module docstring on fixed inputs: a mirror that answers the same value at another price is not the mirror."""

    result, work = _priced(name)
    assert (canonical_text(result), work.wire()) == PRICE_VECTORS[name]


# --------------------------------------------------------------------------
# DETERMINISM, NO SYMPY
# --------------------------------------------------------------------------

_DIGEST_SCRIPT = """
import hashlib, json, random
from fractions import Fraction
from cftuv_envelope.reference.radical_v2 import RadicalSumV2, canonical_text
rng = random.Random(9)
rows = []
for _ in range(200):
    pairs = [(rng.choice((2, 3, 5, 6, 8, 12, 18, 20, 45)), Fraction(rng.randint(-9, 9), rng.randint(1, 9))) for _ in range(rng.randint(1, 6))]
    x, w1 = RadicalSumV2.from_terms(pairs)
    y, w2 = RadicalSumV2.from_terms(list(reversed(pairs)))
    p, w3 = x.mul(y)
    s, w4 = p.signum()
    rows.append([canonical_text(x), canonical_text(p), s, w1.wire(), w2.wire(), w3.wire(), w4.wire(), hash(p)])
print(hashlib.sha256(json.dumps(rows).encode()).hexdigest())
"""


def test_answers_texts_and_prices_do_not_depend_on_pythonhashseed():
    digests = set()
    for seed in ("0", "1", "4242"):
        environment = dict(os.environ, PYTHONHASHSEED=seed, PYTHONPATH=str(PACKAGE_ROOT.parent))
        result = subprocess.run([sys.executable, "-c", _DIGEST_SCRIPT], env=environment, capture_output=True, text=True, check=True)
        digests.add(result.stdout.strip())
    assert len(digests) == 1


def test_the_module_imports_nothing_but_the_standard_library():
    """No sympy, no mpmath, no float math in the type; `native_exact` (which imports sympy) only inside `to_v1`."""

    tree = ast.parse((PACKAGE_ROOT / "reference" / "radical_v2.py").read_text(encoding="utf-8"))
    top = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            top.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            top.add(node.module.split(".")[0] if node.level == 0 else f".{node.module}")
    assert top <= {"__future__", "dataclasses", "fractions", "math", "re"}, top
    nested = [node for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node not in tree.body]
    assert [(node.module, node.level) for node in nested] == [("native_exact", 1)]
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)} | {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    assert not names & {"sympy", "sp", "mpmath", "float", "sqrt", "hash"}
