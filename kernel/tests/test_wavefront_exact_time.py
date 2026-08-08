"""Точная арифметика времени события: без порогов, без вычисления корня.

Здесь доказывается ровно то, что позволяет очереди быть точной: канонический
вид суммы корней единственен, поэтому ноль отличается от «маленького»
рационально, а знак — целочисленно.
"""

from __future__ import annotations

from fractions import Fraction
import math

import pytest

from cftuv_envelope import exact_sqrt_sum as exact_sqrt_sum_module
from cftuv_envelope.wavefront import event_time as event_time_module
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


@pytest.mark.parametrize(
    "value,expected",
    (
        (1_000_003, {1_000_003: 1}),
        (1_000_003 * 1_000_033, {1_000_003: 1, 1_000_033: 1}),
        (10_007**2, {10_007: 2}),
        (10_007**5, {10_007: 5}),
        (10_007**3 * 10_009**2, {10_007: 3, 10_009: 2}),
    ),
)
def test_factorization_vectors_reconstruct_exactly(value, expected):
    factors = exact_sqrt_sum_module._factorize(value)
    reconstructed = math.prod(
        prime**power for prime, power in factors.items()
    )
    assert factors == expected
    assert reconstructed == value


@pytest.mark.parametrize(
    "composite",
    (
        1_000_003 * 1_000_033,
        10_007**3,
        10_007 * 10_009 * 10_037,
    ),
)
def test_pollard_rho_returns_a_deterministic_proper_divisor(composite):
    observed = [
        exact_sqrt_sum_module._pollard_rho(
            composite, exact_sqrt_sum_module.unlimited_reference_budget()
        )
        for _ in range(4)
    ]
    assert observed == [observed[0]] * 4
    divisor = observed[0]
    assert 1 < divisor < composite
    assert composite % divisor == 0


def test_batched_brent_replays_a_zero_product_from_its_checkpoint(monkeypatch):
    """Весь пакет даёт gcd(0, 9) == 9, replay находит делитель 3."""

    calls = []
    real_gcd = math.gcd

    def tracking_gcd(left, right):
        divisor = real_gcd(left, right)
        calls.append((left, right, divisor))
        return divisor

    monkeypatch.setattr(exact_sqrt_sum_module, "gcd", tracking_gcd)
    divisor = exact_sqrt_sum_module._pollard_rho_brent_attempt(
        9,
        2,
        6,
        batch_size=2,
        budget=exact_sqrt_sum_module.unlimited_reference_budget(),
    )
    batch_failure = calls.index((0, 9, 9))
    recovered = calls.index((6, 9, 3))
    assert divisor == 3
    assert batch_failure < recovered
    assert 1 < divisor < 9 and 9 % divisor == 0


def test_prime_universe_deduplicates_q_and_keeps_rational_radicals_exact(
    monkeypatch,
):
    calls = []
    original = exact_sqrt_sum_module._factorize

    def tracking_factorize(value, budget=None):
        calls.append(value)
        return original(value, budget)

    monkeypatch.setattr(exact_sqrt_sum_module, "_factorize", tracking_factorize)
    universe = exact_sqrt_sum_module._prime_universe_from_q_values(
        (
            Fraction(12, 5),
            Fraction(12, 5),
            18,
            0,
            18,
        )
    )
    assert calls == [18, 60]
    assert universe == (2, 3, 5)


def test_prime_universe_reconstructs_only_complete_squarefree_support():
    reconstruct = exact_sqrt_sum_module._support_from_prime_universe
    assert reconstruct(30, (2, 3, 5, 7)) == (2, 3, 5)
    assert reconstruct(30, (2, 3, 7)) is None
    assert reconstruct(12, (2, 3)) is None
    assert reconstruct(1, ()) == ()


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


def test_prime_universe_division_matches_legacy_terms_and_identity():
    numerator = (
        SqrtSumV1.rational(7)
        + SqrtSumV1.radical(2, 6)
        - SqrtSumV1.radical(1, 10)
    )
    denominator = (
        SqrtSumV1.rational(3)
        + SqrtSumV1.radical(1, 6)
        - SqrtSumV1.radical(2, 15)
    )
    legacy = numerator / denominator
    optimized = exact_sqrt_sum_module._divide_with_prime_universe(
        numerator,
        denominator,
        (2, 3, 5),
    )
    assert optimized.terms == legacy.terms
    assert optimized * denominator == numerator


def test_prime_universe_miss_restarts_legacy_on_whole_original_operands(
    monkeypatch,
):
    numerator = SqrtSumV1.rational(5) + SqrtSumV1.radical(1, 2)
    denominator = (
        SqrtSumV1.rational(3)
        + SqrtSumV1.radical(1, 2)
        + SqrtSumV1.radical(1, 3)
    )
    # Патчится `divided_by`, а не оператор: именно им ходит откат legacy —
    # оператор третьего аргумента не принимает и бюджет через себя не пустил бы.
    original = SqrtSumV1.divided_by
    expected = original(numerator, denominator, None)
    calls = []

    def tracking_legacy(left, right, budget=None):
        calls.append((left, right))
        return original(left, right, budget)

    monkeypatch.setattr(SqrtSumV1, "divided_by", tracking_legacy)
    actual = exact_sqrt_sum_module._divide_with_prime_universe(
        numerator,
        denominator,
        (2,),
    )
    assert actual.terms == expected.terms
    assert calls == [(numerator, denominator)]


def test_rational_prime_universe_division_never_picks_or_falls_back(monkeypatch):
    numerator = SqrtSumV1.rational(7)
    denominator = SqrtSumV1.rational(3)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("рациональное деление не выбирает простой")

    monkeypatch.setattr(
        exact_sqrt_sum_module,
        "_pick_prime_from_universe",
        forbidden,
    )
    monkeypatch.setattr(SqrtSumV1, "divided_by", forbidden)
    assert exact_sqrt_sum_module._divide_with_prime_universe(
        numerator,
        denominator,
        (),
    ) == SqrtSumV1.rational(Fraction(7, 3))


def test_prime_universe_is_operation_local_and_never_leaks_between_calls():
    first_numerator = SqrtSumV1.rational(1)
    first_denominator = (
        SqrtSumV1.rational(1) + SqrtSumV1.radical(1, 2)
    )
    second_numerator = SqrtSumV1.rational(1)
    second_denominator = (
        SqrtSumV1.rational(1) + SqrtSumV1.radical(1, 3)
    )
    first = exact_sqrt_sum_module._divide_with_prime_universe(
        first_numerator,
        first_denominator,
        (2,),
    )
    second = exact_sqrt_sum_module._divide_with_prime_universe(
        second_numerator,
        second_denominator,
        (3,),
    )
    assert first.terms == (first_numerator / first_denominator).terms
    assert second.terms == (second_numerator / second_denominator).terms
    assert not hasattr(exact_sqrt_sum_module, "_CURRENT_PRIME_UNIVERSE")


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


def test_event_point_distinguishes_legacy_none_from_certified_empty_basis(
    monkeypatch,
):
    lines = _square(8)
    time, outcome = concurrency_time(lines[0], lines[1], lines[2])
    assert outcome is EventTimeOutcome.EXACT
    observed_universes = []
    original = event_time_module._divide_with_prime_universe

    def capture(numerator, denominator, prime_universe, budget=None):
        observed_universes.append(prime_universe)
        return original(numerator, denominator, prime_universe, budget)

    monkeypatch.setattr(
        event_time_module,
        "_divide_with_prime_universe",
        capture,
    )
    legacy = event_point(lines[0], lines[1], time)
    assert observed_universes == []
    certified = event_time_module._event_point_with_prime_universe(
        lines[0],
        lines[1],
        time,
        (),
    )
    assert observed_universes == [(), ()]
    assert certified == legacy


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
