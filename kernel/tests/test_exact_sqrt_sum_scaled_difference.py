"""`scaled_difference`, `difference_sign`, `difference_is_zero` и их потребители.

Проекция точки на нормаль пролёта (`x*b - y*a`), разность проекций и её знак
считались цепочкой `scaled`, `-`, `sign`: три-пять `SqrtSumV1` и по `Fraction`
на каждый шаг. Теперь это один проход на целых с общим знаменателем, а знак
спрашивается у фильтра прямо из целых. Это ЦЕНА, а не семантика, поэтому
эталон — исходные выражения: члены, тип и `repr` коэффициента, счётчики
`SIGN_COUNTS` и статьи бюджета обязаны совпасть, в том числе когда фильтр знака
не решает и вопрос уходит в сопряжение.
"""

from __future__ import annotations

import random
from fractions import Fraction

import pytest

from cftuv_envelope.exact_sqrt_sum import (
    SIGN_COUNTS,
    ExactWorkOperationV1,
    SqrtSumV1,
    _named,
    exact_work_budget,
    reset_factorization_memory,
    reset_sign_counts,
)
from cftuv_envelope.wavefront import exact_candidate_view as view_module
from cftuv_envelope.wavefront.event_time import (
    EventPointV1,
    EventTimeV1,
    SupportLineV1,
    compare_times,
    concurrency_time,
)
from cftuv_envelope.wavefront.exact_candidate_view import (
    CandidateSpanStateV1,
    CandidateVertexStateV1,
    ExactCandidateViewV1,
    SpanContainmentV1,
    position,
    sliding_projection,
    span_containment,
    span_end,
)


@pytest.fixture(autouse=True)
def _cold_state():
    reset_factorization_memory()
    reset_sign_counts()
    yield
    reset_factorization_memory()
    reset_sign_counts()


def typed(value):
    return tuple((m, type(c), c) for m, c in value.terms)


def ref_scaled(value, factor):
    factor = Fraction(factor)
    if factor == 0:
        return SqrtSumV1(())
    return SqrtSumV1(tuple((m, c * factor) for m, c in value.terms))


def ref_sub(left, right):
    merged = dict(left.terms)
    for radicand, coefficient in right.terms:
        merged[radicand] = merged.get(radicand, Fraction(0)) - coefficient
    return SqrtSumV1(tuple(sorted((m, c) for m, c in merged.items() if c)))


POOL = (1, 2, 3, 5, 6, 7, 10, 15, 30, 42, 1_000_003 * 1_000_033, 844_687_660_141)
FACTORS = (0, 1, -1, 2, -3, 7, -12, 10**9 + 7, Fraction(1, 3), Fraction(-5, 8))


def random_value(rng, count=None):
    count = rng.choice((0, 1, 2, 2, 3, 4)) if count is None else count
    terms = []
    for radicand in sorted(rng.sample(POOL, count)):
        coefficient = Fraction(
            rng.choice((-9, -4, -1, 1, 2, 5, 11, 10**14 + 3)),
            rng.choice((1, 1, 2, 3, 6, 97)),
        )
        terms.append((radicand, coefficient))
    return SqrtSumV1(tuple(terms))


def test_scaled_difference_matches_scaled_minus_scaled():
    rng = random.Random(5)
    for _ in range(3000):
        left, right = random_value(rng), random_value(rng)
        left_factor, right_factor = rng.choice(FACTORS), rng.choice(FACTORS)
        actual = left.scaled_difference(left_factor, right, right_factor)
        expected = ref_sub(
            ref_scaled(left, left_factor), ref_scaled(right, right_factor)
        )
        assert typed(actual) == typed(expected)
        assert repr(actual) == repr(expected)


def test_scaled_difference_of_equal_projections_is_the_empty_sum():
    value = SqrtSumV1.radical(3, 2) + SqrtSumV1.rational(Fraction(1, 7))
    assert value.scaled_difference(2, value, 2).terms == ()
    assert SqrtSumV1(()).scaled_difference(5, SqrtSumV1(()), -3).terms == ()


def test_difference_sign_and_zero_match_the_subtraction_and_count_the_same():
    rng = random.Random(6)
    for _ in range(3000):
        left, right = random_value(rng), random_value(rng)
        if rng.random() < 0.2:
            right = left
        reset_sign_counts()
        reset_factorization_memory()
        reference_budget = exact_work_budget(stage="REF")
        expected = (left - right).sign(budget=reference_budget)
        expected_counts = dict(SIGN_COUNTS)
        reset_sign_counts()
        reset_factorization_memory()
        budget = exact_work_budget(stage="NEW")
        assert left.difference_sign(right, budget) == expected
        assert dict(SIGN_COUNTS) == expected_counts
        assert budget.counters() == reference_budget.counters()
        assert left.difference_is_zero(right) == (left - right).is_zero


def test_difference_sign_falls_back_to_conjugation_when_the_filter_cannot_decide():
    big = 10**20
    value = SqrtSumV1.radical(1, big * big + 1)
    bound = SqrtSumV1.rational(big)
    reset_sign_counts()
    budget = exact_work_budget(stage="T")
    assert value.difference_sign(bound, budget) == 1
    assert SIGN_COUNTS["closed_by_conjugation"] == 1
    assert bound.difference_sign(value, budget) == -1
    assert SIGN_COUNTS["closed_by_conjugation"] == 2


# --------------------------------------------------------------------------
# span_containment / span_end / sliding_projection против исходных выражений
# --------------------------------------------------------------------------


def ref_span_end(view, vertex_ref, span_ref, time, *, at_start):
    span = view.span_state(span_ref)
    place = position(view, vertex_ref, time)
    if place is not None:
        return ref_sub(
            ref_scaled(place.x, span.line.b), ref_scaled(place.y, span.line.a)
        )
    if not span.line.is_stationary:
        return None
    x0, y0, x1, y1 = span.source_span
    node_x, node_y = (x0, y0) if at_start else (x1, y1)
    return SqrtSumV1.rational(node_x * span.line.b - node_y * span.line.a)


def ref_span_bound(view, span, span_ref, time, *, at_start):
    vertex = span.start_vertex if at_start else span.end_vertex
    bound = ref_span_end(view, vertex, span_ref, time, at_start=at_start)
    if bound is not None:
        return bound
    place = span.frozen_start if at_start else span.frozen_end
    if (
        place is None
        or span.frozen_instant is None
        or compare_times(time, span.frozen_instant, view.budget) != 0
    ):
        return None
    return ref_sub(
        ref_scaled(place.x, span.line.b), ref_scaled(place.y, span.line.a)
    )


def ref_span_containment(view, span_ref, point, time):
    span = view.span_state(span_ref)
    if span.start_vertex is None or span.end_vertex is None:
        return SpanContainmentV1(False, False, False)
    here = ref_sub(
        ref_scaled(point.x, span.line.b), ref_scaled(point.y, span.line.a)
    )
    low = ref_span_bound(view, span, span_ref, time, at_start=True)
    high = ref_span_bound(view, span, span_ref, time, at_start=False)
    inside = not (
        (low is not None and (here - low).sign(budget=view.budget) < 0)
        or (high is not None and (high - here).sign(budget=view.budget) < 0)
    )
    return SpanContainmentV1(
        inside,
        low is not None and (here - low).is_zero,
        high is not None and (high - here).is_zero,
    )


def random_line(rng, *, stationary=False):
    a, b = rng.randint(-9, 9), rng.randint(-9, 9)
    if a == 0 and b == 0:
        a = 1
    q = 0 if stationary else rng.choice(
        (1, 2, 4, 5, 8, 9, 25, Fraction(1, 4), Fraction(9, 5), a * a + b * b)
    )
    return SupportLineV1(a, b, rng.randint(-30, 30), q)


def random_time(rng):
    divisor = SqrtSumV1.radical(rng.randint(1, 5), rng.choice((2, 3, 5, 7)))
    divisor = divisor + SqrtSumV1.rational(rng.randint(1, 6))
    return EventTimeV1.normalized(
        Fraction(rng.randint(0, 20), rng.choice((1, 2, 3))), divisor
    )


def random_point(rng):
    return EventPointV1(random_value(rng, 2), random_value(rng, 2))


def build_view(rng, budget):
    lines = {
        "p": random_line(rng),
        "n": random_line(rng),
        "T": random_line(rng, stationary=rng.random() < 0.15),
    }
    time = random_time(rng)
    frozen = rng.random() < 0.5
    spans = {
        name: CandidateSpanStateV1(
            line,
            (0, 0, 3, 4),
            "s" if name == "T" else None,
            "e" if name == "T" else None,
            time if frozen and rng.random() < 0.8 else random_time(rng) if frozen else None,
            random_point(rng) if frozen else None,
            random_point(rng) if frozen else None,
        )
        for name, line in lines.items()
    }
    vertices = {
        "s": CandidateVertexStateV1("p", "T", time, None),
        "e": CandidateVertexStateV1("T", "n", time, None),
    }
    view = ExactCandidateViewV1(
        (2, 3, 5, 7),
        vertices.__getitem__,
        spans.__getitem__,
        lambda ref, when: None,
        budget,
        None,
    )
    return view, time


def counters():
    return dict(SIGN_COUNTS)


def test_span_containment_matches_the_original_expressions():
    rng = random.Random(31)
    inside = outside = flagged = 0
    for _ in range(900):
        reset_factorization_memory()
        reference_budget = exact_work_budget(stage="REF")
        view, time = build_view(rng, reference_budget)
        point = random_point(rng)
        if rng.random() < 0.25:
            place = position(view, "s", time)
            if place is not None:
                point = place
        reset_sign_counts()
        reset_factorization_memory()
        reference_budget = exact_work_budget(stage="REF")
        view = view.__class__(
            view.prime_universe, view.vertex_state, view.span_state,
            view.trace_bounds, reference_budget, None,
        )
        expected = ref_span_containment(view, "T", point, time)
        expected_counts = counters()
        expected_budget = reference_budget.counters()

        reset_sign_counts()
        reset_factorization_memory()
        budget = exact_work_budget(stage="NEW")
        view = view.__class__(
            view.prime_universe, view.vertex_state, view.span_state,
            view.trace_bounds, budget, None,
        )
        actual = span_containment(view, "T", point, time)
        assert actual == expected
        assert counters() == expected_counts
        assert budget.counters() == expected_budget
        inside += actual.inside
        outside += not actual.inside
        flagged += actual.at_start or actual.at_end
    assert inside > 30 and outside > 30 and flagged > 5


def test_span_end_and_sliding_projection_match_the_original_expressions():
    rng = random.Random(32)
    for _ in range(600):
        budget = exact_work_budget(stage="T")
        view, time = build_view(rng, budget)
        for vertex, at_start in (("s", True), ("e", False)):
            expected = ref_span_end(view, vertex, "T", time, at_start=at_start)
            actual = span_end(view, vertex, "T", time, at_start=at_start)
            if expected is None:
                assert actual is None
            else:
                assert typed(actual) == typed(expected)
        first, second = random_line(rng), random_line(rng)
        point = random_point(rng)
        projected = sliding_projection(first, second, point)
        if projected is not None:
            expected = ref_sub(
                ref_scaled(point.x, first.b), ref_scaled(point.y, first.a)
            )
            assert typed(projected) == typed(expected)


def test_event_point_numerators_match_the_original_chain():
    """`_event_point` строит числители через `scaled_difference`: точка та же."""

    from cftuv_envelope.wavefront import event_time as event_module

    def ref_event_point(first, second, time, prime_universe, budget):
        determinant = first.a * second.b - second.a * first.b
        _named(budget).spend_exact_position_hydrations(
            1, operation=ExactWorkOperationV1.EXACT_POSITION, radicand=first.q
        )
        right_first = SqrtSumV1.rational(first.c) * time.divisor + (
            SqrtSumV1.radical(time.dividend, first.q, budget)
        )
        right_second = SqrtSumV1.rational(second.c) * time.divisor + (
            SqrtSumV1.radical(time.dividend, second.q, budget)
        )
        scale = time.divisor.scaled(determinant)
        x_numerator = ref_sub(
            ref_scaled(right_first, second.b), ref_scaled(right_second, first.b)
        )
        y_numerator = ref_sub(
            ref_scaled(right_second, first.a), ref_scaled(right_first, second.a)
        )
        x = event_module._divide_with_prime_universe(
            x_numerator, scale, prime_universe, budget
        )
        y = event_module._divide_with_prime_universe(
            y_numerator, scale, prime_universe, budget
        )
        return EventPointV1(x, y)

    rng = random.Random(33)
    checked = 0
    for _ in range(500):
        first, second = random_line(rng), random_line(rng)
        if first.a * second.b - second.a * first.b == 0:
            continue
        time = random_time(rng)
        reset_factorization_memory()
        reference_budget = exact_work_budget(stage="REF")
        expected = ref_event_point(
            first, second, time, (2, 3, 5, 7), reference_budget
        )
        reset_factorization_memory()
        budget = exact_work_budget(stage="NEW")
        actual = event_module._event_point(
            first, second, time, prime_universe=(2, 3, 5, 7), budget=budget
        )
        assert typed(actual.x) == typed(expected.x)
        assert typed(actual.y) == typed(expected.y)
        assert repr(actual) == repr(expected)
        assert budget.counters() == reference_budget.counters()
        checked += 1
    assert checked > 300
