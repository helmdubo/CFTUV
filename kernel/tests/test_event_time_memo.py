"""Память времён события на сборку скелета: тот же ответ, считается один раз.

Закон кандидата спрашивает время одной тройки прямых по десятку раз:
поколения exact-time замыкания и его повтор считают одни и те же пары
(вершина, цель). `concurrency_time_in` и `sliding_time_in` кладут ответ в память
мест сборки скелета (`PositionMemoV1`), ключ — идентичность прямых. Это ЦЕНА, а не
семантика: чистая функция трёх аргументов, повторный вопрос ничего не оплачивает
(память разложений уже ответила), поэтому ответ памяти и пересчёта один, а
статьи бюджета те же. Меняется только счётчик знаков `SIGN_COUNTS`: пропущенный
пересчёт не спрашивает знак второй раз.
"""

from __future__ import annotations

import random
from fractions import Fraction

import pytest

from cftuv_envelope.exact_sqrt_sum import (
    SIGN_COUNTS,
    SqrtSumV1,
    exact_work_budget,
    reset_factorization_memory,
    reset_sign_counts,
)
from cftuv_envelope.wavefront import exact_candidate_view as view_module
from cftuv_envelope.wavefront.candidate_law import evaluate_split_candidate
from cftuv_envelope.wavefront.event_time import (
    EventTimeV1,
    SupportLineV1,
    concurrency_time,
    sliding_time,
)
from cftuv_envelope.wavefront.exact_candidate_view import (
    CandidateSpanStateV1,
    CandidateVertexStateV1,
    ExactCandidateViewV1,
    PositionMemoV1,
    concurrency_time_in,
    sliding_time_in,
)

UNIVERSE = (2, 3, 5, 7)


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    reset_sign_counts()
    yield
    reset_factorization_memory()
    reset_sign_counts()


def random_line(rng):
    a, b = rng.randint(-9, 9), rng.randint(-9, 9)
    if a == 0 and b == 0:
        a = 1
    q = rng.choice((1, 2, 4, 5, 8, 25, Fraction(1, 4), Fraction(9, 5), a * a + b * b))
    return SupportLineV1(a, b, rng.randint(-30, 30), q)


def view_with(memo, budget=None):
    return ExactCandidateViewV1(
        UNIVERSE,
        lambda ref: None,
        lambda ref: None,
        lambda ref, time: None,
        budget if budget is not None else exact_work_budget(stage="T"),
        memo,
    )


class Counter:
    def __init__(self, monkeypatch, name):
        self.calls = 0
        original = getattr(view_module, name)

        def counting(*args, **kwargs):
            self.calls += 1
            return original(*args, **kwargs)

        monkeypatch.setattr(view_module, name, counting)


def test_memoized_times_equal_the_direct_functions():
    rng = random.Random(3)
    for _ in range(300):
        first, second, third = (random_line(rng) for _ in range(3))
        along = SqrtSumV1.radical(rng.randint(1, 5), rng.choice((2, 3, 5))) + (
            SqrtSumV1.rational(rng.randint(-3, 3))
        )
        for memo in (PositionMemoV1(UNIVERSE), None):
            view = view_with(memo)
            reset_factorization_memory()
            expected = concurrency_time(first, second, third, view.budget)
            reset_factorization_memory()
            actual = concurrency_time_in(view, first, second, third)
            assert actual == expected and repr(actual) == repr(expected)
            reset_factorization_memory()
            expected = sliding_time(first, along, third, view.budget)
            reset_factorization_memory()
            actual = sliding_time_in(view, first, along, third)
            assert actual == expected and repr(actual) == repr(expected)


def test_a_repeated_question_is_answered_by_the_memo_with_the_same_object(
    monkeypatch,
):
    counter = Counter(monkeypatch, "concurrency_time")
    sliding = Counter(monkeypatch, "sliding_time")
    rng = random.Random(4)
    first, second, third = (random_line(rng) for _ in range(3))
    along = SqrtSumV1.radical(2, 3) + SqrtSumV1.rational(1)
    view = view_with(PositionMemoV1(UNIVERSE))
    answers = [concurrency_time_in(view, first, second, third) for _ in range(5)]
    slid = [sliding_time_in(view, first, along, third) for _ in range(5)]
    assert counter.calls == 1 and sliding.calls == 1
    assert all(answer is answers[0] for answer in answers)
    assert all(answer is slid[0] for answer in slid)
    # Другой порядок прямых — другой вопрос.
    concurrency_time_in(view, second, first, third)
    assert counter.calls == 2


def test_no_memo_or_a_foreign_basis_or_a_fresh_memo_means_recomputation(monkeypatch):
    counter = Counter(monkeypatch, "concurrency_time")
    rng = random.Random(5)
    first, second, third = (random_line(rng) for _ in range(3))
    for memo in (None, PositionMemoV1((2, 3))):
        view = view_with(memo)
        for _ in range(3):
            concurrency_time_in(view, first, second, third)
    assert counter.calls == 6
    memo = PositionMemoV1(UNIVERSE)
    view = view_with(memo)
    concurrency_time_in(view, first, second, third)
    concurrency_time_in(view, first, second, third)
    assert counter.calls == 7
    # Память принадлежит ОДНОЙ сборке скелета: у другой сборки память своя, и она считает заново.
    concurrency_time_in(view_with(PositionMemoV1(UNIVERSE)), first, second, third)
    assert counter.calls == 8


def test_equal_lines_that_are_different_objects_are_asked_separately(
    monkeypatch,
):
    counter = Counter(monkeypatch, "concurrency_time")
    rng = random.Random(6)
    first, second, third = (random_line(rng) for _ in range(3))
    clone = SupportLineV1(first.a, first.b, first.c, first.q)
    view = view_with(PositionMemoV1(UNIVERSE))
    one = concurrency_time_in(view, first, second, third)
    two = concurrency_time_in(view, clone, second, third)
    assert counter.calls == 2
    assert one == two


def test_a_failed_question_is_not_remembered(monkeypatch):
    rng = random.Random(7)
    first, second, third = (random_line(rng) for _ in range(3))
    original = view_module.concurrency_time
    state = {"fail": True, "calls": 0}

    def flaky(*args, **kwargs):
        state["calls"] += 1
        if state["fail"]:
            raise RuntimeError("budget exhausted")
        return original(*args, **kwargs)

    monkeypatch.setattr(view_module, "concurrency_time", flaky)
    view = view_with(PositionMemoV1(UNIVERSE))
    with pytest.raises(RuntimeError):
        concurrency_time_in(view, first, second, third)
    state["fail"] = False
    assert concurrency_time_in(view, first, second, third) == original(
        first, second, third, view.budget
    )
    assert state["calls"] == 2


def test_memo_hits_do_not_spend_budget_and_only_lower_the_sign_counters():
    rng = random.Random(8)
    for _ in range(40):
        first, second, third = (random_line(rng) for _ in range(3))
        reset_factorization_memory()
        direct = exact_work_budget(stage="D")
        view = view_with(None, direct)
        reset_sign_counts()
        for _ in range(3):
            concurrency_time_in(view, first, second, third)
        direct_signs = dict(SIGN_COUNTS)

        reset_factorization_memory()
        memoized = exact_work_budget(stage="M")
        view = view_with(PositionMemoV1(UNIVERSE), memoized)
        reset_sign_counts()
        for _ in range(3):
            concurrency_time_in(view, first, second, third)
        assert memoized.counters() == direct.counters()
        assert SIGN_COUNTS["total"] <= direct_signs["total"]
        assert SIGN_COUNTS["closed_by_conjugation"] == direct_signs[
            "closed_by_conjugation"
        ]


# --------------------------------------------------------------------------
# Закон кандидата целиком: память и плотный режим дают одно решение
# --------------------------------------------------------------------------


def build_view(rng, memo, budget):
    lines = {name: random_line(rng) for name in ("p", "n", "T", "q", "r")}
    time = EventTimeV1.normalized(
        Fraction(rng.randint(0, 12), rng.choice((1, 2))),
        SqrtSumV1.radical(rng.randint(1, 4), rng.choice((2, 3, 5)))
        + SqrtSumV1.rational(rng.randint(1, 5)),
    )
    sliding = (
        SqrtSumV1.radical(1, rng.choice((2, 3))) + SqrtSumV1.rational(1)
        if rng.random() < 0.3
        else None
    )
    spans = {
        name: CandidateSpanStateV1(
            line,
            (0, 0, 3, 4),
            "s" if name == "T" else None,
            "e" if name == "T" else None,
        )
        for name, line in lines.items()
    }
    vertices = {
        "v": CandidateVertexStateV1("p", "n", time, sliding),
        "s": CandidateVertexStateV1("q", "T", time, None),
        "e": CandidateVertexStateV1("T", "r", time, None),
    }
    view = ExactCandidateViewV1(
        UNIVERSE,
        vertices.__getitem__,
        spans.__getitem__,
        lambda ref, when: None,
        budget,
        memo,
    )
    return view, time


def test_split_candidate_decisions_are_the_same_with_and_without_the_memo():
    rng = random.Random(21)
    candidates = refusals = 0
    for _ in range(400):
        seed = rng.getstate()
        zero = EventTimeV1(Fraction(0), SqrtSumV1.rational(1))
        runs = []
        for memo in (None, PositionMemoV1(UNIVERSE)):
            rng.setstate(seed)
            reset_factorization_memory()
            view, _ = build_view(rng, memo, exact_work_budget(stage="T"))
            budget_before = view.budget.counters()
            decisions = [
                evaluate_split_candidate(view, "v", "T", now=zero)
                for _ in range(3)
            ]
            runs.append((decisions, view.budget.counters(), budget_before))
        (expected, expected_counters, _), (actual, counters, _) = runs
        assert actual == expected
        assert repr(actual) == repr(expected)
        # Гидратации мест — статья ПАМЯТИ МЕСТ (она и раньше считала по
        # одной на промах), поэтому сравниваются остальные пять статей:
        # факторизация и радикалы память времён не трогает.
        assert counters[:5] == expected_counters[:5]
        candidates += expected[0].candidate is not None
        refusals += expected[0].candidate is None
    assert candidates > 10 and refusals > 10
