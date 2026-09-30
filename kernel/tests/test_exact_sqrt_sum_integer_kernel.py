"""Целочисленное ядро `SqrtSumV1`: те же ответы, те же счётчики, тот же бюджет.

Оболочка, знак, сложение, вычитание, произведение и сравнение времён события
считаются на целых с общим знаменателем вместо `Fraction`. Это ЦЕНА, а не
семантика, поэтому эталоном служат ИСХОДНЫЕ реализации на `Fraction` (копии
тел до замены): любое расхождение значения, типа коэффициента, `repr`,
порядка членов, счётчика `SIGN_COUNTS` или статьи бюджета — ошибка замены.

`repr` проверяется отдельно и нарочно: он служит ключом сортировки в десятках
мест волнового фронта и входит в дайджесты, а `Fraction(3, 1)` и `3` равны по
значению, но различны как строки.
"""

from __future__ import annotations

import random
from fractions import Fraction
from math import lcm

import pytest

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.exact_sqrt_sum import (
    SIGN_COUNTS,
    SqrtSumV1,
    _exact_sign,
    _integer_form,
    reset_factorization_memory,
    reset_sign_counts,
    unlimited_reference_budget,
)
from cftuv_envelope.wavefront.event_time import (
    EventTimeV1,
    compare_times,
    times_are_equal,
)


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    reset_sign_counts()
    yield
    reset_factorization_memory()
    reset_sign_counts()


# --------------------------------------------------------------------------
# Эталон: исходные тела на Fraction
# --------------------------------------------------------------------------


def ref_from_map(mapping):
    return SqrtSumV1(tuple(sorted((m, c) for m, c in mapping.items() if c)))


def ref_add(left, right):
    merged = dict(left.terms)
    for radicand, coefficient in right.terms:
        merged[radicand] = merged.get(radicand, Fraction(0)) + coefficient
    return ref_from_map(merged)


def ref_neg(value):
    return SqrtSumV1(tuple((m, -c) for m, c in value.terms))


def ref_sub(left, right):
    return ref_add(left, ref_neg(right))


def ref_scaled(value, factor):
    factor = Fraction(factor)
    if factor == 0:
        return SqrtSumV1(())
    return SqrtSumV1(tuple((m, c * factor) for m, c in value.terms))


def ref_mul(left, right):
    from math import gcd

    merged = {}
    for left_radicand, left_coefficient in left.terms:
        for right_radicand, right_coefficient in right.terms:
            common = gcd(left_radicand, right_radicand)
            radicand = (left_radicand // common) * (right_radicand // common)
            value = left_coefficient * right_coefficient * common
            merged[radicand] = merged.get(radicand, Fraction(0)) + value
    return ref_from_map(merged)


def ref_enclosure(value, bits):
    from math import isqrt

    scale = 1 << bits
    low = high = Fraction(0)
    for radicand, coefficient in value.terms:
        if radicand == 1:
            low += coefficient
            high += coefficient
            continue
        floor_root = isqrt(radicand << (2 * bits))
        lower = Fraction(floor_root, scale)
        upper = Fraction(floor_root + 1, scale)
        if coefficient > 0:
            low += coefficient * lower
            high += coefficient * upper
        else:
            low += coefficient * upper
            high += coefficient * lower
    return low, high


def ref_certified_sign(value, bits):
    low, high = ref_enclosure(value, bits)
    if low > 0:
        return 1
    if high < 0:
        return -1
    return None


def ref_sign(value, *, filter_bits=64, budget=None):
    SIGN_COUNTS["total"] += 1
    if not value.terms:
        SIGN_COUNTS["closed_rational_zero"] += 1
        return 0
    if len(value.terms) == 1 and value.terms[0][0] == 1:
        SIGN_COUNTS["closed_rational_nonzero"] += 1
        coefficient = value.terms[0][1]
        return (coefficient > 0) - (coefficient < 0)
    certified = ref_certified_sign(value, filter_bits)
    if certified is not None:
        SIGN_COUNTS["closed_by_enclosure"] += 1
        return certified
    SIGN_COUNTS["closed_by_conjugation"] += 1
    return _exact_sign(dict(value.terms), filter_bits, budget)


def ref_difference(left, right):
    return ref_sub(
        ref_scaled(right.divisor, left.dividend),
        ref_scaled(left.divisor, right.dividend),
    )


def ref_compare_times(left, right, budget=None):
    return ref_sign(ref_difference(left, right), budget=budget)


def ref_times_are_equal(left, right):
    return not ref_difference(left, right).terms


# --------------------------------------------------------------------------
# Генераторы
# --------------------------------------------------------------------------

SMALL_RADICANDS = (1, 2, 3, 5, 6, 7, 10, 11, 13, 14, 15, 21, 30, 35, 42, 77, 210)
LARGE_RADICANDS = (
    1_000_003 * 1_000_033,
    844_687_660_141,
    1_439_659_412_197,
    2 * 844_687_660_141,
    2 * 3 * 5 * 7 * 11 * 13 * 17 * 19 * 23 * 29 * 31 * 37,
    (1 << 61) - 1,
    3 * ((1 << 61) - 1),
)
DENOMINATORS = (
    1, 2, 3, 4, 6, 7, 12, 97, 844_687_660_141, 1_439_659_412_197, 10**20 + 39,
)


def random_fraction(rng, *, allow_zero=False):
    big = rng.random() < 0.3
    numerator = rng.randint(-(10**25), 10**25) if big else rng.randint(-60, 60)
    denominator = 1 if rng.random() < 0.3 else rng.choice(DENOMINATORS)
    value = Fraction(numerator, denominator)
    return value if (value or allow_zero) else Fraction(1, denominator)


def random_value(rng, *, allow_zero=False, int_coefficients=False):
    pool = SMALL_RADICANDS + LARGE_RADICANDS
    count = rng.choice((0, 1, 1, 2, 2, 3, 4, 5))
    terms = []
    for radicand in sorted(rng.sample(pool, count)):
        coefficient = random_fraction(rng, allow_zero=allow_zero)
        if int_coefficients and rng.random() < 0.4:
            coefficient = rng.randint(-9, 9) or 1
        terms.append((radicand, coefficient))
    return SqrtSumV1(tuple(terms))


def random_time(rng):
    dividend = Fraction(0) if rng.random() < 0.1 else random_fraction(rng)
    return EventTimeV1(dividend, random_value(rng))


def typed(value):
    """Члены вместе с ТИПОМ коэффициента: `3` и `Fraction(3, 1)` здесь различны."""

    return tuple((m, type(c), c) for m, c in value.terms)


def assert_same_value(actual, expected):
    assert typed(actual) == typed(expected)
    assert repr(actual) == repr(expected)


def assert_canonical_shape(value):
    radicands = [m for m, _ in value.terms]
    assert radicands == sorted(set(radicands))
    assert all(type(c) is Fraction and c != 0 for _, c in value.terms)


# --------------------------------------------------------------------------
# Оболочка и знак
# --------------------------------------------------------------------------


@pytest.mark.parametrize("bits", (0, 1, 5, 64, 100))
def test_enclosure_and_certified_sign_match_fraction_reference(bits):
    rng = random.Random(1000 + bits)
    for _ in range(600):
        value = random_value(rng)
        actual = value.enclosure(bits)
        expected = ref_enclosure(value, bits)
        assert actual == expected
        assert repr(actual) == repr(expected)
        assert all(type(part) is Fraction for part in actual)
        assert value.certified_sign(bits) == ref_certified_sign(value, bits)


def test_enclosure_of_noncanonical_input_matches_reference():
    """Нулевые коэффициенты, радикант 0, целые коэффициенты — как и раньше."""

    rng = random.Random(7)
    for _ in range(400):
        value = random_value(rng, allow_zero=True, int_coefficients=True)
        for bits in (0, 64):
            assert value.enclosure(bits) == ref_enclosure(value, bits)
            assert value.certified_sign(bits) == ref_certified_sign(value, bits)
    zero_radicand = SqrtSumV1(((0, Fraction(3, 2)), (1, Fraction(-1, 3))))
    assert zero_radicand.enclosure(8) == ref_enclosure(zero_radicand, 8)
    assert SqrtSumV1(()).enclosure(64) == (Fraction(0), Fraction(0))


def test_enclosure_of_huge_integers_matches_reference():
    rng = random.Random(11)
    for _ in range(60):
        terms = []
        radicands = {rng.randint(2, 10**60) for _ in range(rng.randint(1, 4))}
        for radicand in sorted(radicands):
            numerator = rng.randint(-(10**120), 10**120) or 1
            denominator = rng.randint(1, 10**90)
            terms.append((radicand, Fraction(numerator, denominator)))
        value = SqrtSumV1(tuple(terms))
        for bits in (0, 64, 200):
            assert value.enclosure(bits) == ref_enclosure(value, bits)
            assert value.certified_sign(bits) == ref_certified_sign(value, bits)


def test_negative_radicand_is_refused_exactly_as_before():
    value = SqrtSumV1(((-2, Fraction(1, 2)),))
    with pytest.raises(ValueError):
        ref_enclosure(value, 64)
    with pytest.raises(ValueError):
        value.enclosure(64)
    with pytest.raises(ValueError):
        value.certified_sign(64)


def test_integer_form_uses_the_least_common_denominator():
    rng = random.Random(3)
    for _ in range(300):
        value = random_value(rng)
        common, items = _integer_form(value.terms)
        denominators = [c.denominator for _, c in value.terms] or [1]
        assert common == lcm(*denominators)
        assert [m for m, _ in items] == [m for m, _ in value.terms]
        for (_, numerator), (_, coefficient) in zip(items, value.terms):
            assert type(numerator) is int
            assert Fraction(numerator, common) == coefficient


# --------------------------------------------------------------------------
# Сложение, вычитание, произведение
# --------------------------------------------------------------------------


def test_add_sub_mul_match_reference_value_type_repr_and_order():
    rng = random.Random(21)
    for _ in range(1500):
        left, right = random_value(rng), random_value(rng)
        for actual, expected in (
            (left + right, ref_add(left, right)),
            (left - right, ref_sub(left, right)),
            (left * right, ref_mul(left, right)),
        ):
            assert_same_value(actual, expected)
            assert_canonical_shape(actual)


def test_add_sub_mul_keep_the_reference_types_for_noncanonical_input():
    """Целый коэффициент на входе: тип результата тот же, что давал исходный код."""

    rng = random.Random(22)
    for _ in range(800):
        left = random_value(rng, int_coefficients=True)
        right = random_value(rng, int_coefficients=True)
        assert_same_value(left + right, ref_add(left, right))
        assert_same_value(left - right, ref_sub(left, right))
        assert_same_value(left * right, ref_mul(left, right))


def test_cancellation_and_zero_operands_give_the_empty_sum():
    rng = random.Random(23)
    zero = SqrtSumV1(())
    for _ in range(200):
        value = random_value(rng)
        assert (value - value).terms == ()
        assert (value + (-value)).terms == ()
        assert (value * zero).terms == ()
        assert (zero * value).terms == ()
        assert_same_value(value + zero, ref_add(value, zero))
        assert_same_value(zero - value, ref_sub(zero, value))
    assert (zero * zero).terms == ()


def test_product_of_conjugates_is_rational_and_stored_as_fraction():
    left = SqrtSumV1(((1, Fraction(1)), (2, Fraction(1))))
    right = SqrtSumV1(((1, Fraction(1)), (2, Fraction(-1))))
    product = left * right
    assert repr(product) == "SqrtSumV1(terms=((1, Fraction(-1, 1)),))"
    root = SqrtSumV1(((2, Fraction(1)),))
    assert repr(root * root) == "SqrtSumV1(terms=((1, Fraction(2, 1)),))"
    cross = SqrtSumV1(((6, Fraction(1, 3)),)) * SqrtSumV1(((10, Fraction(3, 5)),))
    assert repr(cross) == "SqrtSumV1(terms=((15, Fraction(2, 5)),))"


def test_sub_does_not_go_through_negation_or_fraction_zero_stubs():
    """`Fraction(0)` и промежуточная `-other` не нужны: сверка с `self + (-other)`."""

    rng = random.Random(24)
    for _ in range(400):
        left, right = random_value(rng), random_value(rng)
        assert_same_value(left - right, ref_add(left, -right))


# --------------------------------------------------------------------------
# Время события: сравнение и равенство
# --------------------------------------------------------------------------


def counts_after(function):
    reset_sign_counts()
    result = function()
    return result, dict(SIGN_COUNTS)


def test_compare_times_matches_reference_answer_and_sign_counts():
    rng = random.Random(31)
    for _ in range(2500):
        left, right = random_time(rng), random_time(rng)
        actual, actual_counts = counts_after(lambda: compare_times(left, right))
        expected, expected_counts = counts_after(
            lambda: ref_compare_times(left, right)
        )
        assert actual == expected
        assert actual_counts == expected_counts
        assert times_are_equal(left, right) == ref_times_are_equal(left, right)


def test_equal_times_with_proportional_pairs_are_zero_and_equal():
    rng = random.Random(32)
    checked = 0
    for _ in range(300):
        time = random_time(rng)
        if not time.divisor.terms:
            continue
        factor = random_fraction(rng)
        twin = EventTimeV1(time.dividend * factor, time.divisor.scaled(factor))
        assert times_are_equal(time, twin)
        assert times_are_equal(twin, time)
        actual, actual_counts = counts_after(lambda: compare_times(time, twin))
        expected, expected_counts = counts_after(
            lambda: ref_compare_times(time, twin)
        )
        assert actual == expected == 0
        assert actual_counts == expected_counts
        assert actual_counts["closed_rational_zero"] == 1
        checked += 1
    assert checked > 100


def test_compare_times_rational_nonzero_and_zero_dividends():
    rational = SqrtSumV1.rational(Fraction(3, 7))
    root = SqrtSumV1(((2, Fraction(5, 3)),))
    zero = EventTimeV1(Fraction(0), rational)
    cases = (
        (EventTimeV1(Fraction(1, 2), rational), EventTimeV1(Fraction(1, 3), rational)),
        (EventTimeV1(Fraction(1, 3), rational), EventTimeV1(Fraction(1, 2), rational)),
        (zero, EventTimeV1(Fraction(-4), root)),
        (EventTimeV1(Fraction(-4), root), zero),
        (zero, zero),
        (EventTimeV1(Fraction(7, 5), root), EventTimeV1(Fraction(3), rational)),
    )
    for left, right in cases:
        actual, actual_counts = counts_after(lambda: compare_times(left, right))
        expected, expected_counts = counts_after(
            lambda: ref_compare_times(left, right)
        )
        assert actual == expected
        assert actual_counts == expected_counts
        assert times_are_equal(left, right) == ref_times_are_equal(left, right)


# --------------------------------------------------------------------------
# Фильтр не доказал знак: прежний путь, прежний бюджет, прежний счёт
# --------------------------------------------------------------------------


def pell_pair(steps: int) -> tuple[int, int]:
    """`x^2 - 6*y^2 = 1`: `x - y*sqrt(6)` положительно и ~ 1/(2x) — глубже 64 бит."""

    x, y = 1, 0
    for _ in range(steps):
        x, y = 5 * x + 12 * y, 2 * x + 5 * y
    return x, y


def undecided_times(steps: int = 30):
    """`left - right` равно `x - y*sqrt(6)` (после домножения на положительное)."""

    x, y = pell_pair(steps)
    left = EventTimeV1(Fraction(x), SqrtSumV1(((6, Fraction(1)),)))
    right = EventTimeV1(Fraction(y), SqrtSumV1.rational(1))
    return left, right, x, y


def test_the_enclosure_really_cannot_decide_the_pell_difference():
    _, _, x, y = undecided_times()
    value = SqrtSumV1(((1, Fraction(x)), (6, Fraction(-y))))
    assert x.bit_length() > 90
    assert value.certified_sign(64) is None
    assert ref_certified_sign(value, 64) is None
    assert value.certified_sign(400) == 1


def test_undecided_filter_falls_back_with_the_same_budget_and_counts(monkeypatch):
    left, right, _, _ = undecided_times()
    calls = []
    original = canon._exact_sign

    def spy(terms, filter_bits, budget=None):
        calls.append(budget)
        return original(terms, filter_bits, budget)

    monkeypatch.setattr(canon, "_exact_sign", spy)
    for first, second, sign in ((left, right, 1), (right, left, -1)):
        reset_factorization_memory()
        new_budget = unlimited_reference_budget(stage="NEW")
        actual, actual_counts = counts_after(
            lambda: compare_times(first, second, new_budget)
        )
        assert calls and all(budget is new_budget for budget in calls)
        calls.clear()
        reset_factorization_memory()
        ref_budget = unlimited_reference_budget(stage="REF")
        expected, expected_counts = counts_after(
            lambda: ref_compare_times(first, second, ref_budget)
        )
        calls.clear()
        assert actual == expected == sign
        assert actual_counts == expected_counts
        assert actual_counts["closed_by_conjugation"] == 1
        assert actual_counts["closed_by_enclosure"] == 0
        assert new_budget.counters() == ref_budget.counters()
        assert new_budget.spent == ref_budget.spent > 0


def test_decided_comparisons_never_touch_the_exact_path(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("the exact path must not run for a decided filter")

    monkeypatch.setattr(canon, "_exact_sign", forbidden)
    rng = random.Random(41)
    for _ in range(800):
        left, right = random_time(rng), random_time(rng)
        assert compare_times(left, right) == ref_compare_times(left, right)


def test_exactly_equal_pell_times_are_zero_without_conjugation():
    left, _, x, y = undecided_times()
    twin = EventTimeV1(left.dividend * 3, left.divisor.scaled(3))
    assert times_are_equal(left, twin)
    actual, counts = counts_after(lambda: compare_times(left, twin))
    assert actual == 0
    assert counts["closed_by_conjugation"] == 0
    assert counts["closed_rational_zero"] == 1
    near = EventTimeV1(Fraction(y), SqrtSumV1.rational(1))
    assert not times_are_equal(left, near)
