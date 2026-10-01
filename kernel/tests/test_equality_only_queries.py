"""Вопросы «равны ли времена» не ходят в знак: тот же ответ, дешевле.

Два места спрашивают у `compare_times` только `== 0`: счётчик равных времени
записей очереди (`EventQueueV1._count_at_time`, по записи очереди на каждый
уровень) и «родился ли порт в `now`» (`symbolic_overlay._born_place`). Равенство
времён — чисто рациональная проверка (`times_are_equal`), знак для него не
нужен, а обход очереди сверху обрезает поддеревья, чьё время строго позже.
Эталон — исходные выражения: те же числа и те же места, где был ответ.
"""

from __future__ import annotations

import heapq
import random
from fractions import Fraction
from types import SimpleNamespace

import pytest

from cftuv_envelope.exact_sqrt_sum import (
    SIGN_COUNTS,
    SqrtSumV1,
    exact_work_budget,
    reset_factorization_memory,
    reset_sign_counts,
)
from cftuv_envelope.wavefront.event_time import (
    EventPointV1,
    EventTimeV1,
    compare_times,
)
from cftuv_envelope.wavefront.events import EventQueueV1
from cftuv_envelope.wavefront.symbolic_overlay import _born_place


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    reset_sign_counts()
    yield
    reset_factorization_memory()
    reset_sign_counts()


def random_time(rng, base=None):
    if base is not None and rng.random() < 0.35:
        # То же значение другим представлением: пропорциональная пара.
        factor = Fraction(rng.randint(1, 7), rng.randint(1, 5))
        return EventTimeV1(base.dividend * factor, base.divisor.scaled(factor))
    divisor = SqrtSumV1.radical(rng.randint(1, 4), rng.choice((2, 3, 5))) + (
        SqrtSumV1.rational(rng.randint(1, 6))
    )
    return EventTimeV1(Fraction(rng.randint(0, 9), rng.choice((1, 2, 3))), divisor)


def brute_force_count(queue, time, budget):
    return sum(
        compare_times(entry.event.time, time, budget) == 0
        for entry in queue._heap
    )


def test_count_at_time_equals_the_full_scan_on_random_queues():
    rng = random.Random(14)
    nonzero = 0
    for _ in range(500):
        budget = exact_work_budget(stage="T")
        queue = EventQueueV1(work_budget=budget)
        pool = [random_time(rng) for _ in range(rng.randint(1, 5))]
        for _ in range(rng.randint(0, 40)):
            queue.push(SimpleNamespace(time=random_time(rng, rng.choice(pool))))
        for time in pool + [random_time(rng)]:
            expected = brute_force_count(queue, time, budget)
            assert queue._count_at_time(time) == expected
            nonzero += expected > 0
    assert nonzero > 200


def test_count_at_time_keeps_the_full_answer_for_entries_from_the_past():
    """Запись раньше `time` не обрезает обход: равные ниже неё считаются."""

    rng = random.Random(15)
    budget = exact_work_budget(stage="T")
    queue = EventQueueV1(work_budget=budget)
    early = random_time(rng)
    middle = EventTimeV1(early.dividend + 100, early.divisor)
    for time in (early, middle, middle, early, middle):
        queue.push(SimpleNamespace(time=time))
    assert queue._count_at_time(middle) == 3
    assert queue._count_at_time(early) == 2
    assert EventQueueV1(work_budget=budget)._count_at_time(early) == 0


def test_count_at_time_asks_far_fewer_questions_than_a_full_scan(monkeypatch):
    from cftuv_envelope.wavefront import events as events_module

    rng = random.Random(16)
    budget = exact_work_budget(stage="T")
    queue = EventQueueV1(work_budget=budget)
    base = random_time(rng)
    for step in range(200):
        queue.push(
            SimpleNamespace(
                time=EventTimeV1(base.dividend + step + 1, base.divisor)
            )
        )
    queue.push(SimpleNamespace(time=base))
    asked = []
    original = events_module.compare_times

    def counting(left, right, work_budget=None):
        asked.append(1)
        return original(left, right, work_budget)

    monkeypatch.setattr(events_module, "compare_times", counting)
    assert queue._count_at_time(base) == 1
    assert len(asked) < 20


def ref_born_place(overlay, ref, budget=None):
    vertex = overlay.vertices.get(ref)
    if vertex is None or vertex.point is None:
        return None
    if compare_times(vertex.birth, overlay.time, budget) != 0:
        return None
    return vertex.point


def test_born_place_matches_the_sign_based_original():
    rng = random.Random(17)
    born = other = 0
    for _ in range(600):
        time = random_time(rng)
        birth = random_time(rng, time)
        point = (
            None
            if rng.random() < 0.15
            else EventPointV1(SqrtSumV1.rational(1), SqrtSumV1.rational(2))
        )
        overlay = SimpleNamespace(
            time=time,
            vertices={"v": SimpleNamespace(birth=birth, point=point)},
        )
        for ref in ("v", "missing"):
            expected = ref_born_place(overlay, ref)
            assert _born_place(overlay, ref) is expected
        born += ref_born_place(overlay, "v") is not None
        other += ref_born_place(overlay, "v") is None
    assert born > 30 and other > 30


def test_born_place_never_asks_for_a_sign():
    rng = random.Random(18)
    time = random_time(rng)
    overlay = SimpleNamespace(
        time=time,
        vertices={
            "v": SimpleNamespace(
                birth=random_time(rng, time),
                point=EventPointV1(SqrtSumV1.rational(1), SqrtSumV1.rational(2)),
            )
        },
    )
    reset_sign_counts()
    _born_place(overlay, "v")
    assert SIGN_COUNTS["total"] == 0
