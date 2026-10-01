"""`_divide_with_prime_universe` на целых: те же члены, типы, вызовы и бюджет.

Деление домножением на сопряжённые идёт теперь в целых с общим знаменателем:
сопряжённое — это знаменатель, у которого члены с радикандом, делящимся на
выбранное простое, сменили знак, а `Fraction` создаётся ровно один раз на член
результата. Это ЦЕНА, а не семантика, поэтому эталон — исходное тело функции на
`SqrtSumV1` и `Fraction` (копия ниже). Совпасть обязано всё, что видно снаружи:
члены, тип и `repr` коэффициента, порядок вызовов `_pick_prime_from_universe`
(их число и порядок радикандов заморожены в полевых тестах), статьи бюджета
и откат в `divided_by` на ИСХОДНЫХ операндах.
"""

from __future__ import annotations

import random
from fractions import Fraction
from math import gcd

import pytest

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.exact_sqrt_sum import (
    SqrtSumV1,
    ZeroSqrtSumDivisorError,
    _divide_with_prime_universe,
    _multiply_integer_items,
    _pick_prime_from_universe,
    _split_by_prime,
    exact_work_budget,
    reset_factorization_memory,
)

PRIMES = (2, 3, 5, 7, 11, 13)


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    yield
    reset_factorization_memory()


def ref_from_map(mapping):
    return SqrtSumV1(tuple(sorted((m, c) for m, c in mapping.items() if c)))


def ref_mul(left, right):
    merged = {}
    for left_radicand, left_coefficient in left.terms:
        for right_radicand, right_coefficient in right.terms:
            common = gcd(left_radicand, right_radicand)
            radicand = (left_radicand // common) * (right_radicand // common)
            value = left_coefficient * right_coefficient * common
            merged[radicand] = merged.get(radicand, Fraction(0)) + value
    return ref_from_map(merged)


def ref_sub(left, right):
    merged = dict(left.terms)
    for radicand, coefficient in right.terms:
        merged[radicand] = merged.get(radicand, Fraction(0)) - coefficient
    return ref_from_map(merged)


def ref_scaled(value, factor):
    factor = Fraction(factor)
    if factor == 0:
        return SqrtSumV1(())
    return SqrtSumV1(tuple((m, c * factor) for m, c in value.terms))


def ref_divide(numerator, denominator, prime_universe, budget, pick):
    """Исходное тело `_divide_with_prime_universe` до замены."""

    original_numerator = numerator
    original_denominator = denominator
    if denominator.is_zero:
        return numerator.divided_by(denominator, budget)
    while True:
        rational = denominator.as_rational()
        if rational is not None:
            return ref_scaled(numerator, Fraction(1) / rational)
        prime = pick(denominator.as_map(), prime_universe)
        if prime is None:
            return original_numerator.divided_by(original_denominator, budget)
        outside, inside = _split_by_prime(denominator.as_map(), prime)
        root = SqrtSumV1.radical(1, prime, budget)
        conjugate = ref_sub(
            ref_from_map(outside), ref_mul(ref_from_map(inside), root)
        )
        numerator = ref_mul(numerator, conjugate)
        denominator = ref_mul(denominator, conjugate)


def random_squarefree(rng, pool):
    radicand = 1
    for prime in rng.sample(pool, rng.randint(0, min(4, len(pool)))):
        radicand *= prime
    return radicand


def random_value(rng, pool, *, nonzero=False):
    count = rng.choice((0, 1, 2, 2, 3, 4, 5))
    radicands = {random_squarefree(rng, pool) for _ in range(count)}
    terms = []
    for radicand in sorted(radicands):
        denominator = rng.choice((1, 1, 2, 3, 6, 7, 10**12 + 39))
        numerator = rng.choice((-7, -3, -1, 1, 2, 5, 11, rng.randint(-10**15, 10**15) or 1))
        terms.append((radicand, Fraction(numerator, denominator)))
    if nonzero and not terms:
        terms.append((1, Fraction(3, 2)))
    return SqrtSumV1(tuple(terms))


class Recorder:
    """Журнал вызовов `_pick_prime_from_universe`: порядок радикандов и ответ."""

    def __init__(self):
        self.reference_calls = []
        self.new_calls = []

    def wrap(self, log):
        def pick(terms, prime_universe):
            answer = _pick_prime_from_universe(terms, prime_universe)
            log.append((tuple(terms), prime_universe, answer))
            return answer

        return pick


def run_pair(monkeypatch, numerator, denominator, prime_universe):
    recorder = Recorder()
    budget = exact_work_budget(stage="REF")
    reset_factorization_memory()
    expected = ref_divide(
        numerator,
        denominator,
        prime_universe,
        budget,
        recorder.wrap(recorder.reference_calls),
    )
    expected_counters = budget.counters()

    budget = exact_work_budget(stage="NEW")
    reset_factorization_memory()
    with monkeypatch.context() as local:
        local.setattr(
            canon,
            "_pick_prime_from_universe",
            recorder.wrap(recorder.new_calls),
        )
        actual = _divide_with_prime_universe(
            numerator, denominator, prime_universe, budget
        )
    return expected, expected_counters, actual, budget.counters(), recorder


def typed(value):
    return tuple((m, type(c), c) for m, c in value.terms)


def test_division_matches_the_fraction_reference(monkeypatch):
    rng = random.Random(2026)
    for _ in range(700):
        numerator = random_value(rng, PRIMES)
        denominator = random_value(rng, PRIMES, nonzero=True)
        if denominator.is_zero:
            continue
        universe = PRIMES
        expected, expected_counters, actual, counters, _ = run_pair(
            monkeypatch, numerator, denominator, universe
        )
        assert typed(actual) == typed(expected)
        assert repr(actual) == repr(expected)
        assert counters == expected_counters


def test_division_inverts_the_multiplication_exactly(monkeypatch):
    rng = random.Random(7)
    for _ in range(200):
        quotient = random_value(rng, PRIMES)
        denominator = random_value(rng, PRIMES, nonzero=True)
        if denominator.is_zero:
            continue
        numerator = ref_mul(quotient, denominator)
        actual = _divide_with_prime_universe(
            numerator, denominator, PRIMES, exact_work_budget(stage="T")
        )
        assert actual.terms == quotient.terms


def test_pick_calls_keep_their_number_order_and_answers(monkeypatch):
    rng = random.Random(11)
    checked = 0
    for _ in range(400):
        numerator = random_value(rng, PRIMES)
        denominator = random_value(rng, PRIMES, nonzero=True)
        if denominator.is_zero:
            continue
        # Вселенная беднее носителя: часть делений обязана уйти в откат.
        universe = PRIMES[: rng.randint(0, len(PRIMES))]
        expected, expected_counters, actual, counters, recorder = run_pair(
            monkeypatch, numerator, denominator, universe
        )
        assert recorder.new_calls == recorder.reference_calls
        assert typed(actual) == typed(expected)
        assert counters == expected_counters
        checked += 1
    assert checked > 100


def test_a_miss_restarts_the_legacy_division_on_the_original_operands(
    monkeypatch,
):
    numerator = SqrtSumV1.rational(5) + SqrtSumV1.radical(1, 2)
    denominator = (
        SqrtSumV1.rational(3)
        + SqrtSumV1.radical(1, 2)
        + SqrtSumV1.radical(1, 3)
    )
    original = SqrtSumV1.divided_by
    calls = []

    def tracking(left, right, budget=None):
        calls.append((left, right))
        return original(left, right, budget)

    monkeypatch.setattr(SqrtSumV1, "divided_by", tracking)
    actual = _divide_with_prime_universe(numerator, denominator, (2,))
    assert calls == [(numerator, denominator)]
    assert actual.terms == original(numerator, denominator, None).terms


def test_zero_denominator_is_the_named_refusal():
    with pytest.raises(ZeroSqrtSumDivisorError):
        _divide_with_prime_universe(
            SqrtSumV1.rational(1), SqrtSumV1(()), PRIMES
        )


def test_empty_numerator_divides_to_the_empty_sum():
    denominator = SqrtSumV1.rational(2) + SqrtSumV1.radical(1, 3)
    assert _divide_with_prime_universe(
        SqrtSumV1(()), denominator, (3,)
    ).terms == ()


def test_multiply_integer_items_matches_sqrt_sum_product():
    rng = random.Random(3)
    for _ in range(300):
        left = random_value(rng, PRIMES)
        right = random_value(rng, PRIMES)
        common_left, items_left = canon._integer_form(left.terms)
        common_right, items_right = canon._integer_form(right.terms)
        items = _multiply_integer_items(items_left, items_right)
        expected = ref_mul(left, right)
        assert [m for m, _ in items] == [m for m, _ in expected.terms]
        assert tuple(
            (m, Fraction(n, common_left * common_right)) for m, n in items
        ) == expected.terms
        assert all(type(n) is int and n != 0 for _, n in items)
