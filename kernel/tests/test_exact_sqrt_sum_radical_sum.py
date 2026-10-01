"""`radical_sum`: одна сумма корней вместо цепочки `radical(...) + radical(...)`.

Скорость события (`concurrency_time`, `sliding_time`) — сумма двух-трёх
слагаемых `c * sqrt(q)`. Раньше она собиралась цепочкой `radical` и `+`, то есть
тремя `SqrtSumV1`, тремя `Fraction` и двумя слияниями словарей на вызов; теперь
сумма копится в целых и нормируется один раз на член результата. Это ЦЕНА, а не
семантика, поэтому эталон — исходная цепочка: члены, тип и `repr` коэффициента,
порядок вызовов `squarefree_split` (он платит бюджет на промахе памяти),
статьи бюджета и счётчики знака обязаны совпасть.
"""

from __future__ import annotations

import random
from fractions import Fraction

import pytest

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.exact_sqrt_sum import (
    SIGN_COUNTS,
    NegativeRadicandError,
    SqrtSumV1,
    exact_work_budget,
    radical_sum,
    reset_factorization_memory,
    reset_sign_counts,
)
from cftuv_envelope.wavefront.event_time import (
    EventTimeOutcome,
    EventTimeV1,
    SupportLineV1,
    concurrency_time,
    sliding_time,
)


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    reset_sign_counts()
    yield
    reset_factorization_memory()
    reset_sign_counts()


def chain(parts, budget):
    """Исходная цепочка `radical(...) + radical(...) + ...`."""

    result = None
    for coefficient, radicand in parts:
        term = SqrtSumV1.radical(coefficient, radicand, budget)
        result = term if result is None else result + term
    return result


def typed(value):
    return tuple((m, type(c), c) for m, c in value.terms)


RADICANDS = (
    0, 1, 2, 3, 4, 8, 9, 12, 18, 25, 50, 72, 144, 1000, 844_687_660_141,
    Fraction(1, 4), Fraction(9, 4), Fraction(3, 5), Fraction(2, 7),
    Fraction(844_687_660_141, 9), Fraction(4), Fraction(25, 1),
)


def random_part(rng):
    coefficient = rng.choice(
        (0, 1, -1, 2, -3, 7, -12, 10**9 + 7, Fraction(1, 3), Fraction(-5, 8),
         Fraction(10**12 + 39, 6))
    )
    return coefficient, rng.choice(RADICANDS)


def test_radical_sum_matches_the_chain_value_type_repr_and_budget():
    rng = random.Random(404)
    for _ in range(1500):
        parts = tuple(random_part(rng) for _ in range(rng.randint(1, 4)))
        reset_factorization_memory()
        reference_budget = exact_work_budget(stage="REF")
        expected = chain(parts, reference_budget)
        reset_factorization_memory()
        budget = exact_work_budget(stage="NEW")
        actual = radical_sum(parts, budget)
        assert typed(actual) == typed(expected)
        assert repr(actual) == repr(expected)
        assert budget.counters() == reference_budget.counters()


def test_radical_sum_cancels_to_the_empty_sum_like_the_chain():
    parts = ((3, 8), (-6, 2), (5, Fraction(1, 4)), (-5, Fraction(1, 4)))
    assert radical_sum(parts, exact_work_budget(stage="T")).terms == ()
    assert radical_sum(parts, exact_work_budget(stage="T")) == chain(
        parts, exact_work_budget(stage="T")
    )
    assert radical_sum((), exact_work_budget(stage="T")).terms == ()


def test_radical_sum_makes_the_same_split_calls_in_the_same_order(monkeypatch):
    rng = random.Random(9)
    for _ in range(300):
        parts = tuple(random_part(rng) for _ in range(rng.randint(1, 4)))
        logs = {"chain": [], "sum": []}
        original = canon.squarefree_split

        def recording(log):
            def split(n, budget=None):
                log.append(n)
                return original(n, budget)

            return split

        with monkeypatch.context() as local:
            local.setattr(canon, "squarefree_split", recording(logs["chain"]))
            chain(parts, exact_work_budget(stage="REF"))
        with monkeypatch.context() as local:
            local.setattr(canon, "squarefree_split", recording(logs["sum"]))
            radical_sum(parts, exact_work_budget(stage="NEW"))
        assert logs["sum"] == logs["chain"]


def test_a_negative_radicand_is_refused_after_the_same_prefix():
    budget = exact_work_budget(stage="T")
    with pytest.raises(NegativeRadicandError):
        radical_sum(((1, 2), (1, -3)), budget)
    with pytest.raises(NegativeRadicandError):
        chain(((1, 2), (1, -3)), exact_work_budget(stage="T"))


# --------------------------------------------------------------------------
# События: исходные тела `concurrency_time` и `sliding_time` на цепочке
# --------------------------------------------------------------------------


def ref_concurrency_time(first, second, third, budget=None):
    cofactor_first = second.a * third.b - third.a * second.b
    cofactor_second = third.a * first.b - first.a * third.b
    cofactor_third = first.a * second.b - second.a * first.b
    offset = (
        first.c * cofactor_first
        + second.c * cofactor_second
        + third.c * cofactor_third
    )
    speed = (
        SqrtSumV1.radical(cofactor_first, first.q, budget)
        + SqrtSumV1.radical(cofactor_second, second.q, budget)
        + SqrtSumV1.radical(cofactor_third, third.q, budget)
    )
    if speed.is_zero:
        if offset == 0:
            return None, EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT
        return None, EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT
    return (
        EventTimeV1.normalized(-offset, speed, budget),
        EventTimeOutcome.EXACT,
    )


def ref_sliding_time(line, along, other, budget=None):
    weight = line.a * other.a + line.b * other.b
    cross = line.a * other.b - other.a * line.b
    norm = line.normal_squared
    numerator = (
        SqrtSumV1.rational(norm * other.c)
        + along.scaled(cross)
        - SqrtSumV1.rational(weight * line.c)
    )
    speed = SqrtSumV1.radical(weight, line.q, budget) - SqrtSumV1.radical(
        norm, other.q, budget
    )
    if speed.is_zero:
        if numerator.is_zero:
            return None, EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT
        return None, EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT
    if numerator.is_zero:
        return EventTimeV1(Fraction(0), SqrtSumV1.rational(1)), (
            EventTimeOutcome.EXACT
        )
    return (
        EventTimeV1.normalized(1, speed.divided_by(numerator, budget), budget),
        EventTimeOutcome.EXACT,
    )


SPEEDS = (0, 1, 2, 4, 5, 9, 8, 25, Fraction(1, 4), Fraction(9, 5), Fraction(3, 7))


def random_line(rng):
    a, b = rng.randint(-9, 9), rng.randint(-9, 9)
    if a == 0 and b == 0:
        a = 1
    q = rng.choice(SPEEDS + (a * a + b * b,))
    return SupportLineV1(a, b, rng.randint(-40, 40), q)


def snapshot_state():
    return dict(SIGN_COUNTS)


def run_both(function, reference, args):
    reset_sign_counts()
    reset_factorization_memory()
    reference_budget = exact_work_budget(stage="REF")
    expected = reference(*args, reference_budget)
    expected_signs = snapshot_state()
    reset_sign_counts()
    reset_factorization_memory()
    budget = exact_work_budget(stage="NEW")
    actual = function(*args, budget)
    return expected, expected_signs, actual, snapshot_state(), (
        reference_budget.counters(), budget.counters()
    )


def test_concurrency_time_matches_the_chain_implementation():
    rng = random.Random(77)
    exact = 0
    for _ in range(1200):
        args = tuple(random_line(rng) for _ in range(3))
        expected, expected_signs, actual, signs, (
            reference_counters, counters
        ) = run_both(concurrency_time, ref_concurrency_time, args)
        assert actual[1] is expected[1]
        assert repr(actual) == repr(expected)
        assert actual == expected
        assert signs == expected_signs
        assert counters == reference_counters
        exact += actual[1] is EventTimeOutcome.EXACT
    assert exact > 300


def test_sliding_time_matches_the_chain_implementation():
    rng = random.Random(78)
    exact = 0
    for _ in range(800):
        line, other = random_line(rng), random_line(rng)
        along = SqrtSumV1.radical(
            rng.randint(-5, 5), rng.choice((1, 2, 3, 5, 8)), None
        ) + SqrtSumV1.rational(rng.randint(-4, 4))
        expected, expected_signs, actual, signs, (
            reference_counters, counters
        ) = run_both(sliding_time, ref_sliding_time, (line, along, other))
        assert actual[1] is expected[1]
        assert repr(actual) == repr(expected)
        assert actual == expected
        assert signs == expected_signs
        assert counters == reference_counters
        exact += actual[1] is EventTimeOutcome.EXACT
    assert exact > 100
