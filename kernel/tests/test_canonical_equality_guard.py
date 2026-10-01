"""Каноника сумм корней исполняется, и цена равенства времён названа числами.

`times_are_equal` читает равенство из ПУСТОТЫ разности и эквивалентна
`compare_times(...) == 0` только на канонических суммах: на `sqrt(8)` против
`2*sqrt(2)` знак скажет «равны», пустота — нет. Каноника держалась дисциплиной
конструкторов; здесь она исполняется на входе в машину времён (`EventTimeV1`,
`from_algebraic_sum`). Два слоя: дешёвый всегда включён (структура и квадраты
малых простых), полный (бесквадратность) — аудит набора тестов ядра.

Второй блок закрепляет названную цену: там, где знак требует сопряжения
(`closed_by_conjugation > 0`), `times_are_equal` и обход очереди с обрезкой
тратят статьи бюджета и счётчики знаков СТРОГО подмножеством прежнего пути.
Числа в тесте — те же, что получает прогон; расхождение значит, что цена
сместилась и это надо назвать, а не подогнать.
"""

from __future__ import annotations

from fractions import Fraction
from types import SimpleNamespace

import pytest

from cftuv_envelope import exact_sqrt_sum as sqrt_module
from cftuv_envelope.contracts.surface_arrival import (
    ExactAlgebraicSumV1,
    ExactAlgebraicTimeV1,
    ExactRadicalTermV1,
    exact_rational,
)
from cftuv_envelope.exact_sqrt_sum import (
    SIGN_COUNTS,
    UNBUDGETED_WORK,
    NonCanonicalSqrtSumError,
    SqrtSumV1,
    canonical_audit_enabled,
    exact_work_budget,
    require_canonical,
    reset_factorization_memory,
    reset_sign_counts,
    set_canonical_audit,
)
from cftuv_envelope.surface_arrival_planar_queue import (
    from_algebraic_sum,
    from_algebraic_time,
    to_algebraic_time,
)
from cftuv_envelope.wavefront.event_time import (
    EventTimeV1,
    compare_times,
    times_are_equal,
)
from cftuv_envelope.wavefront.events import EventQueueV1
from cftuv_envelope.wavefront.symbolic_overlay import _born_place


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    reset_sign_counts()
    previous = canonical_audit_enabled()
    yield
    set_canonical_audit(previous)
    reset_factorization_memory()
    reset_sign_counts()


def terms_of(*pairs):
    return SqrtSumV1(tuple((radicand, Fraction(c)) for radicand, c in pairs))


def unchecked_time(dividend, divisor):
    """Время мимо конструктора: единственный путь неканонической величины."""

    time = object.__new__(EventTimeV1)
    object.__setattr__(time, "dividend", Fraction(dividend))
    object.__setattr__(time, "divisor", divisor)
    return time


def record_of(radicand):
    return ExactAlgebraicSumV1(
        (ExactRadicalTermV1(radicand, exact_rational(1)),)
    )


# --------------------------------------------------------------------------
# Дешёвый слой: всегда включён, O(члены)
# --------------------------------------------------------------------------

STRUCTURAL_VIOLATIONS = {
    "unsorted": ((3, Fraction(1)), (2, Fraction(1))),
    "repeated": ((2, Fraction(1)), (2, Fraction(1))),
    "zero_coefficient": ((2, Fraction(0)),),
    "zero_int_coefficient": ((2, 0),),
    "zero_radicand": ((0, Fraction(1)),),
    "negative_radicand": ((-2, Fraction(1)),),
    "float_radicand": ((2.0, Fraction(1)),),
    "float_coefficient": ((2, 0.5),),
    "bool_coefficient": ((2, True),),
    "eight": ((8, Fraction(1)),),
    "twelve": ((3 * 4, Fraction(1)),),
    "nine_times_two": ((18, Fraction(1)),),
    "twenty_five_times_two": ((50, Fraction(1)),),
    "seven_squared_times_three": ((147, Fraction(1)),),
    "perfect_square": ((121, Fraction(1)),),
    "perfect_square_after_valid": ((1, Fraction(1)), (169, Fraction(2))),
}


@pytest.mark.parametrize("name", sorted(STRUCTURAL_VIOLATIONS))
def test_cheap_layer_refuses_with_the_audit_off(name):
    set_canonical_audit(False)
    divisor = SqrtSumV1(STRUCTURAL_VIOLATIONS[name])
    with pytest.raises(NonCanonicalSqrtSumError) as caught:
        require_canonical(divisor, "probe")
    assert caught.value.where == "probe"
    assert str(caught.value).startswith("NON_CANONICAL_SQRT_SUM:probe:")
    with pytest.raises(NonCanonicalSqrtSumError) as caught:
        EventTimeV1(Fraction(1), divisor)
    assert caught.value.where == "EventTimeV1.divisor"


def test_canonical_sums_pass_both_layers():
    for audit in (False, True):
        set_canonical_audit(audit)
        for value in (
            SqrtSumV1(()),
            SqrtSumV1.rational(Fraction(-3, 7)),
            terms_of((1, Fraction(1, 2)), (2, 3), (3, -1), (30030, 5)),
            SqrtSumV1.radical(Fraction(2, 3), Fraction(18, 5)),
        ):
            assert require_canonical(value, "probe") is value
            EventTimeV1(Fraction(1), value)


# --------------------------------------------------------------------------
# Аудит: то, чего дешёвый слой не видит
# --------------------------------------------------------------------------

HIDDEN_SQUARES = {
    "two_times_eleven_squared": 2 * 11**2,
    "three_times_thirteen_squared": 3 * 13**2,
    "two_times_big_prime_squared": 2 * 1000003**2,
}


@pytest.mark.parametrize("name", sorted(HIDDEN_SQUARES))
def test_audit_closes_what_the_cheap_layer_cannot_see(name):
    value = terms_of((HIDDEN_SQUARES[name], 1))
    set_canonical_audit(False)
    # Граница слоёв названа: без аудита эти радиканды проходят.
    assert require_canonical(value, "probe") is value
    set_canonical_audit(True)
    with pytest.raises(NonCanonicalSqrtSumError, match="не бесквадратный"):
        require_canonical(value, "probe")
    with pytest.raises(NonCanonicalSqrtSumError):
        EventTimeV1(Fraction(1), value)


def test_audit_accepts_squarefree_radicands_including_large_ones():
    set_canonical_audit(True)
    for radicand in (1, 2, 30030, 1000003 * 1000033, 10**9 + 7):
        require_canonical(terms_of((radicand, 1)), "probe")


def test_audit_leaves_no_trace_in_memory_budget_or_telemetry():
    """Аудит не двигает цену тестов, у которых она закреплена."""

    def snapshot():
        return (
            dict(sqrt_module._FACTORIZATION_MEMO),
            dict(sqrt_module._SQUAREFREE_MEMO),
            dict(sqrt_module._PRIME_SUPPORT_MEMO),
            list(sqrt_module._KNOWN_PRIMES),
            UNBUDGETED_WORK.counters(),
            dict(SIGN_COUNTS),
        )

    set_canonical_audit(True)
    before = snapshot()
    for radicand in (30030, 1000003 * 1000033, 2 * 1000003**2):
        try:
            require_canonical(terms_of((radicand, 1)), "probe")
        except NonCanonicalSqrtSumError:
            pass
    assert snapshot() == before


# --------------------------------------------------------------------------
# Входы в машину времён
# --------------------------------------------------------------------------


def test_contract_records_enter_the_kernel_only_canonical():
    for audit in (False, True):
        set_canonical_audit(audit)
        # Запись контракта допускает 8; на входе в ядро её нет.
        with pytest.raises(NonCanonicalSqrtSumError) as caught:
            from_algebraic_sum(record_of(8))
        assert caught.value.where == "from_algebraic_sum"
        with pytest.raises(NonCanonicalSqrtSumError):
            from_algebraic_time(
                ExactAlgebraicTimeV1(exact_rational(1), record_of(8))
            )
    set_canonical_audit(True)
    with pytest.raises(NonCanonicalSqrtSumError):
        from_algebraic_time(
            ExactAlgebraicTimeV1(exact_rational(1), record_of(2 * 11**2))
        )
    time = EventTimeV1(Fraction(3), terms_of((2, 1), (5, Fraction(1, 2))))
    assert from_algebraic_time(to_algebraic_time(time)) == time.canonical()


def test_noncanonical_values_split_the_two_questions_and_the_audit_names_it():
    """Зачем стража: на `sqrt(8)` против `2*sqrt(2)` два вопроса расходятся."""

    set_canonical_audit(False)
    eight = unchecked_time(1, terms_of((8, 1)))
    twice_two = unchecked_time(1, terms_of((2, 2)))
    budget = exact_work_budget(stage="T")
    assert compare_times(eight, twice_two, budget) == 0
    assert times_are_equal(eight, twice_two) is False
    # Каноническая форма той же величины согласна с обоими.
    canonical = EventTimeV1(Fraction(1), SqrtSumV1.radical(1, 8))
    assert compare_times(canonical, twice_two, budget) == 0
    assert times_are_equal(canonical, twice_two) is True
    # Под аудитом обход конструктора называется, а не даёт молчаливо другой ответ.
    set_canonical_audit(True)
    with pytest.raises(NonCanonicalSqrtSumError) as caught:
        times_are_equal(eight, twice_two)
    assert caught.value.where == "times_are_equal.left"


# --------------------------------------------------------------------------
# Названная цена: пара, которой знак требует сопряжения
# --------------------------------------------------------------------------

# `P/Q` — подходящая дробь `sqrt(M)`: `P*P - M*Q*Q == -101`, то есть
# `sqrt(M) - P/Q ~ 9e-25`, а оболочка знака (64 бита) шире этой разности.
M, P, Q, OTHER_M = 30030, 99648041611907, 575030792935, 39270
assert P * P - M * Q * Q == -101

ARTICLES = (
    "EXACT_WORK_MODULAR_SQUARINGS",
    "EXACT_WORK_GCD_OPERATIONS",
    "EXACT_WORK_MILLER_RABIN_ROUNDS",
    "EXACT_WORK_POLLARD_ATTEMPTS",
    "EXACT_WORK_RADICAL_MATERIALIZATIONS",
    "EXACT_WORK_EXACT_POSITION_HYDRATIONS",
    "EXACT_WORK_SPENT",
)
SIGN_KEYS = (
    "total",
    "closed_rational_zero",
    "closed_rational_nonzero",
    "closed_by_enclosure",
    "closed_by_conjugation",
)


def articles_row(modular, gcd_, rabin, pollard, radical, position):
    values = (
        modular,
        gcd_,
        rabin,
        pollard,
        radical,
        position,
        modular + gcd_ + rabin + pollard + radical + position,
    )
    return dict(zip(ARTICLES, values))


def signs_row(total, rational_zero, rational_nonzero, enclosure, conjugation):
    return dict(
        zip(
            SIGN_KEYS,
            (total, rational_zero, rational_nonzero, enclosure, conjugation),
        )
    )


FREE = articles_row(0, 0, 0, 0, 0, 0)
NO_SIGNS = signs_row(0, 0, 0, 0, 0)


def conjugation_pair():
    """`later` на ~9e-25 позже `earlier`; знак разности решает только сопряжение."""

    earlier = EventTimeV1(Fraction(1), terms_of((M, 1)))
    later = EventTimeV1(Fraction(1), SqrtSumV1.rational(Fraction(P, Q)))
    return later, earlier


def cold(fn, *, queue=None):
    """`(ответ, статьи бюджета, SIGN_COUNTS)` холодного вопроса.

    Очередь строится ДО обнуления счётчиков: цена вопроса — это цена вопроса, а
    не сборки очереди; бюджет вопроса подставляется в очередь после сборки.
    """

    reset_factorization_memory()
    built = queue() if queue is not None else None
    reset_sign_counts()
    budget = exact_work_budget(stage="T")
    if built is not None:
        built.work_budget = budget
        answer = fn(built, budget)
    else:
        answer = fn(budget)
    return answer, dict(budget.counters()), dict(SIGN_COUNTS)


def assert_strict_subset(old, new):
    """Статьи и счётчики знаков нового пути — подмножество прежнего, и строгое."""

    old_articles, new_articles = old[1], new[1]
    old_signs, new_signs = old[2], new[2]
    assert all(new_articles[key] <= old_articles[key] for key in ARTICLES)
    assert all(new_signs[key] <= old_signs[key] for key in SIGN_KEYS)
    assert (new_articles, new_signs) != (old_articles, old_signs)
    assert new_articles["EXACT_WORK_SPENT"] <= old_articles["EXACT_WORK_SPENT"]
    assert new_signs["total"] < old_signs["total"] or (
        new_articles["EXACT_WORK_SPENT"] < old_articles["EXACT_WORK_SPENT"]
    )


def test_the_pair_requires_conjugation_and_equality_pays_nothing():
    later, earlier = conjugation_pair()
    old = cold(lambda b: compare_times(later, earlier, b) == 0)
    new = cold(lambda b: times_are_equal(later, earlier))
    # Прежний путь: сопряжение, и оно платит.
    assert old[2]["closed_by_conjugation"] == 1 and old[2]["total"] == 1
    assert old == (
        False,
        articles_row(8, 4, 0, 4, 2, 0),
        signs_row(1, 0, 0, 0, 1),
    )
    # Новый путь: тот же ответ, ничего не потрачено, знак не спрошен.
    assert new == (False, FREE, NO_SIGNS)
    assert_strict_subset(old, new)


def test_equal_times_in_another_representation_cost_a_sign_only_on_the_old_path():
    _, earlier = conjugation_pair()
    same = EventTimeV1(Fraction(2), earlier.divisor.scaled(2))
    old = cold(lambda b: compare_times(same, earlier, b) == 0)
    new = cold(lambda b: times_are_equal(same, earlier))
    assert old == (True, FREE, signs_row(1, 1, 0, 0, 0))
    assert new == (True, FREE, NO_SIGNS)
    assert_strict_subset(old, new)


def born_overlay(birth, now):
    return SimpleNamespace(
        time=now,
        vertices={"v": SimpleNamespace(birth=birth, point=object())},
    )


def test_born_place_spends_strictly_less_than_the_sign_route():
    later, earlier = conjugation_pair()
    overlay = born_overlay(later, earlier)
    old = cold(
        lambda b: compare_times(overlay.vertices["v"].birth, overlay.time, b) != 0
    )
    new = cold(lambda b: _born_place(overlay, "v") is None)
    assert old == (
        True,
        articles_row(8, 4, 0, 4, 2, 0),
        signs_row(1, 0, 0, 0, 1),
    )
    assert new == (True, FREE, NO_SIGNS)
    assert_strict_subset(old, new)


def near_root_queue():
    """Куча из трёх записей, все строго позже `earlier`, и все — на сопряжении.

    Корень кучи — самая ранняя из них; обе другие — её потомки, и у каждой в
    разности с `earlier` есть ЕЩЁ ОДИН радикал (`OTHER_M`), то есть пропущенное
    сравнение пропускает ещё и его разложение.
    """

    def build():
        queue = EventQueueV1(work_budget=exact_work_budget(stage="SETUP"))
        for extra in (
            (),
            ((OTHER_M, Fraction(-1, 10**30)),),
            ((OTHER_M, Fraction(-2, 10**30)),),
        ):
            terms = ((1, Fraction(P, Q)),) + extra
            queue.push(SimpleNamespace(time=EventTimeV1(Fraction(1), SqrtSumV1(terms))))
        return queue

    return build


def test_count_at_time_spends_a_strict_subset_of_the_full_scan():
    _, earlier = conjugation_pair()
    old = cold(
        lambda queue, b: sum(
            compare_times(entry.event.time, earlier, b) == 0
            for entry in queue._heap
        ),
        queue=near_root_queue(),
    )
    new = cold(
        lambda queue, b: queue._count_at_time(earlier), queue=near_root_queue()
    )
    assert old == (
        0,
        articles_row(8, 4, 0, 4, 5, 0),
        signs_row(3, 0, 0, 0, 3),
    )
    assert new == (
        0,
        articles_row(8, 4, 0, 4, 2, 0),
        signs_row(1, 0, 0, 0, 1),
    )
    assert_strict_subset(old, new)
