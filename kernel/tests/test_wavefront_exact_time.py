"""Точная арифметика времени события: без порогов, без вычисления корня.

Здесь доказывается ровно то, что позволяет очереди быть точной: канонический
вид суммы корней единственен, поэтому ноль отличается от «маленького»
рационально, а знак — целочисленно.
"""

from __future__ import annotations

from fractions import Fraction
import math

import pytest

from cftuv_envelope.wavefront.event_time import (
    EventTimeOutcome,
    EventTimeV1,
    SupportLineV1,
    compare_times,
    concurrency_time,
    event_point,
    times_are_equal,
)
from cftuv_envelope.wavefront.sqrt_sum import (
    SqrtSumV1,
    ZeroSqrtSumDivisorError,
    prime_support,
    squarefree_split,
)


def test_squarefree_split_pulls_the_whole_square_out():
    assert squarefree_split(50) == (5, 2)
    assert squarefree_split(72) == (6, 2)
    assert squarefree_split(1) == (1, 1)
    assert squarefree_split(2**20 * 3) == (2**10, 3)


def test_the_canonical_form_proves_an_identity_rationally():
    """`sqrt(50) - 5*sqrt(2)` — ТОЧНЫЙ ноль, и это видно без единого корня."""

    value = SqrtSumV1.radical(1, 50) - SqrtSumV1.radical(5, 2)
    assert value.is_zero
    assert value.terms == ()
    assert value.sign() == 0


def test_a_tiny_but_nonzero_value_is_not_confused_with_zero():
    """Разность порядка 1e-17 не ноль, и порога, который бы её съел, нет."""

    value = SqrtSumV1.radical(1, 2) - SqrtSumV1.rational(
        Fraction(665857, 470832)
    )
    assert not value.is_zero
    assert value.sign() == -1
    low, high = value.enclosure(80)
    assert high < 0
    assert abs(float(low)) < 1e-11


@pytest.mark.parametrize(
    "coefficients,radicands",
    [
        ((1, 1, -1), (2, 3, 5)),
        ((3, -2, 1, -1), (2, 3, 5, 7)),
        ((-17, 5, 11, -3, 2), (2, 6, 10, 15, 30)),
        ((1, -1), (1000003, 1000033)),
    ],
)
def test_exact_sign_agrees_with_high_precision_arithmetic(coefficients, radicands):
    value = SqrtSumV1.zero()
    for coefficient, radicand in zip(coefficients, radicands):
        value = value + SqrtSumV1.radical(coefficient, radicand)
    reference = sum(
        coefficient * math.sqrt(radicand)
        for coefficient, radicand in zip(coefficients, radicands)
    )
    assert value.sign() == (reference > 0) - (reference < 0)


def test_conjugation_decides_when_the_enclosure_gives_up():
    """Фильтр с нулевой разрядностью бесполезен, а решатель всё равно отвечает.

    Это и есть доказательство, что оболочка — фильтр, а не решатель: убери её,
    и ответ не изменится, изменится только цена.
    """

    value = SqrtSumV1.radical(1, 2) + SqrtSumV1.radical(1, 3) - SqrtSumV1.radical(1, 5)
    assert value.certified_sign(1) is None
    assert value.sign(filter_bits=1) == 1
    assert value.sign(filter_bits=1) == value.sign(filter_bits=200)


def test_division_by_a_proven_zero_is_a_named_refusal():
    with pytest.raises(ZeroSqrtSumDivisorError):
        SqrtSumV1.rational(1) / (
            SqrtSumV1.radical(1, 50) - SqrtSumV1.radical(5, 2)
        )


def test_division_stays_canonical():
    value = SqrtSumV1.rational(1) / (
        SqrtSumV1.radical(1, 2) + SqrtSumV1.rational(1)
    )
    assert value == SqrtSumV1.radical(1, 2) - SqrtSumV1.rational(1)


def test_prime_support_is_deterministic():
    assert prime_support(30) == (2, 3, 5)
    assert prime_support(1) == ()


def test_a_rational_radicand_is_the_identity_and_not_an_approximation():
    """`sqrt(p/r) = (1/r)*sqrt(p*r)`, и это проверяется ВОЗВЕДЕНИЕМ В КВАДРАТ.

    Проверка идёт квадратом, а не сравнением с `math.sqrt`: возведение остаётся
    внутри той же точной алгебры, поэтому доказывает тождество, а не согласие с
    плавающей точкой на первых пятнадцати знаках.
    """

    for numerator, denominator in ((1, 2), (137438953472, 844687660141), (9, 4)):
        value = SqrtSumV1.radical(1, Fraction(numerator, denominator))
        assert (value * value).as_rational() == Fraction(numerator, denominator)
    # Дробь, у которой корень рационален, канонизируется в чистую дробь.
    assert SqrtSumV1.radical(1, Fraction(9, 4)).as_rational() == Fraction(3, 2)


def test_an_integer_valued_fraction_radicand_gives_the_integer_result_exactly():
    """`Fraction(50)` и `50` под корнем — ОДНО значение, а не два похожих.

    Это не педантизм про типы: `SqrtSumV1` хешируется и попадает в дайджест,
    поэтому два представления одного числа означали бы два дайджеста у одной
    геометрии. Ветка рационального радиканда обязана не выполняться вовсе,
    когда знаменатель равен единице.
    """

    assert SqrtSumV1.radical(3, Fraction(50)) == SqrtSumV1.radical(3, 50)
    assert SqrtSumV1.radical(3, Fraction(50)).terms == ((2, Fraction(15)),)


def test_a_rational_radicand_keeps_the_sign_decidable_without_a_threshold():
    """Знак дробного радиканда решается тем же путём, что и целого.

    Отрицательный контроль к тождеству выше: мало превратить дробь в целое под
    корнем — величина обязана остаться сравнимой. Берётся пара, разность которой
    ТОЧНЫЙ ноль, и пара, где она заведомо не ноль.
    """

    left = SqrtSumV1.radical(1, Fraction(1, 2))
    right = SqrtSumV1.radical(Fraction(1, 2), 2)
    assert (left - right).is_zero
    assert (left - SqrtSumV1.radical(1, Fraction(1, 3))).sign() > 0


# --------------------------------------------------------------------------
# Время события
# --------------------------------------------------------------------------


def _square(side: int) -> list[SupportLineV1]:
    points = [(0, 0), (side, 0), (side, side), (0, side)]
    return [
        SupportLineV1.through(points[index], points[(index + 1) % 4])
        for index in range(4)
    ]


def test_the_square_collapses_at_exactly_half_its_side():
    """Первая обязанность: квадрат схлопывается в точку, время ТОЧНО половина."""

    lines = _square(8)
    time, outcome = concurrency_time(lines[0], lines[1], lines[2])
    assert outcome is EventTimeOutcome.EXACT
    assert time.dividend / time.divisor.terms[0][1] == Fraction(4)
    assert time.divisor.is_rational()
    point = event_point(lines[0], lines[1], time)
    assert point.x == SqrtSumV1.rational(4)
    assert point.y == SqrtSumV1.rational(4)


def test_all_four_collapses_of_a_square_are_the_same_exact_instant():
    lines = _square(8)
    times = [
        concurrency_time(
            lines[index], lines[(index + 1) % 4], lines[(index + 2) % 4]
        )[0]
        for index in range(4)
    ]
    for other in times[1:]:
        assert times_are_equal(times[0], other)
        assert compare_times(times[0], other) == 0


def test_the_incircle_of_a_right_triangle_is_irrational_and_still_exact():
    """Треугольник 12/12: время равно `12 - 6*sqrt(2)`, и это НЕ дробь."""

    points = [(0, 0), (12, 0), (0, 12)]
    lines = [
        SupportLineV1.through(points[index], points[(index + 1) % 3])
        for index in range(3)
    ]
    time, outcome = concurrency_time(lines[0], lines[1], lines[2])
    assert outcome is EventTimeOutcome.EXACT
    assert not time.divisor.is_rational()
    expected = SqrtSumV1.rational(12) - SqrtSumV1.radical(6, 2)
    # Точное тождество `dividend = divisor * (12 - 6*sqrt(2))`, а не сверка с
    # округлённым эталоном: оболочка уже, чем binary64, и float здесь мерил бы
    # своё округление.
    assert (time.divisor * expected) == SqrtSumV1.rational(time.dividend)
    low, high = time.enclosure(120)
    assert high - low < Fraction(1, 10**20)
    assert abs(float(low) - (12 - 6 * math.sqrt(2))) < 1e-12


def test_parallel_wavefronts_never_meet_and_say_so_by_name():
    """Две противоположные стороны и одна из них же — события нет никогда."""

    lines = _square(8)
    time, outcome = concurrency_time(lines[0], lines[2], lines[0])
    assert time is None
    assert outcome is EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT


def test_a_negative_time_is_returned_not_swallowed():
    """Событие в прошлом — тоже ответ; отбрасывает его вызывающий, не формула."""

    points = [(0, 0), (10, 0), (10, 10), (0, 10)]
    lines = [
        SupportLineV1.through(points[index], points[(index + 1) % 4])
        for index in range(4)
    ]
    time, outcome = concurrency_time(lines[2], lines[1], lines[0])
    assert outcome is EventTimeOutcome.EXACT
    assert time.sign != 0


def test_comparison_never_computes_the_time():
    """Сравнение — линейная комбинация делителей, деления в нём нет.

    Проверяется тем, что сравнение верно и на паре, у которой ОБА времени
    иррациональны и близки: если бы сравнение шло через значение, ошибка
    представления решала бы за него.
    """

    left = EventTimeV1.normalized(1, SqrtSumV1.radical(1, 2))
    right = EventTimeV1.normalized(
        Fraction(665857, 941664), SqrtSumV1.rational(1)
    )
    assert compare_times(left, right) == -1
    assert compare_times(right, left) == 1
    assert not times_are_equal(left, right)


def test_canonical_time_is_the_same_for_proportional_representations():
    original = EventTimeV1.normalized(12, SqrtSumV1.radical(3, 2))
    scaled = EventTimeV1.normalized(48, SqrtSumV1.radical(12, 2))
    assert compare_times(original, scaled) == 0
    assert original.canonical() == scaled.canonical()
