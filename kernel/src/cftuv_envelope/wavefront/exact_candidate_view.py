"""Pure exact geometry view shared by runtime and symbolic candidates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .event_time import (
    EventPointV1,
    EventTimeOutcome,
    EventTimeV1,
    SupportLineV1,
    _event_point_with_prime_universe,
    compare_times,
    concurrency_time,
    sliding_point,
    sliding_time,
)
from .sqrt_sum import SqrtSumV1


@dataclass(frozen=True, slots=True)
class CandidateVertexStateV1:
    prev_span: object
    next_span: object
    birth: EventTimeV1
    sliding: SqrtSumV1 | None


@dataclass(frozen=True, slots=True)
class CandidateSpanStateV1:
    """Пролёт фронта, как его видит закон кандидатов.

    Четыре первых поля — рантаймовый вид: концы названы ВЕРШИНАМИ, и где они
    стоят, спрашивается у закона движения.

    `frozen_instant`/`frozen_start`/`frozen_end` — места концов, которые
    ИЗВЕСТНЫ ПАКЕТУ ТОЧНО в это самое мгновение и потому не выводятся из
    движения. Заполняет их символьный слой: порт, рождённый локусом ровно в
    `frozen_instant`, стоит в точке этого локуса. Без них закон движения на
    таком порту отвечает «места нет» — у двух СОВПАВШИХ антипараллельных прямых
    нет ни пересечения, ни скольжения, — и граница пролёта молча исчезает. А
    пролёт без обеих границ содержит ЛЮБУЮ точку: так рождается контакт вне
    пролёта (`walls.012`, четыре SPLIT в одном точном времени).

    Поля необязательные: рантаймовый вид их не заполняет и работает ровно как
    раньше. Заполненные действуют ТОЛЬКО в `frozen_instant` (см.
    `span_containment`) и ТОЛЬКО там, где закон движения промолчал: где он
    отвечает, ответ его.
    """

    line: SupportLineV1
    source_span: tuple[int, ...]
    start_vertex: object | None
    end_vertex: object | None
    frozen_instant: EventTimeV1 | None = None
    frozen_start: EventPointV1 | None = None
    frozen_end: EventPointV1 | None = None


@dataclass(frozen=True, slots=True)
class ExactCandidateViewV1:
    prime_universe: tuple[int, ...]
    vertex_state: Callable[[object], CandidateVertexStateV1]
    span_state: Callable[[object], CandidateSpanStateV1]
    trace_bounds: Callable[[object, EventTimeV1], bool | None]


@dataclass(frozen=True, slots=True)
class SpanContainmentV1:
    inside: bool
    at_start: bool
    at_end: bool


def position(
    view: ExactCandidateViewV1,
    vertex_ref: object,
    time: EventTimeV1,
) -> EventPointV1 | None:
    vertex = view.vertex_state(vertex_ref)
    first = view.span_state(vertex.prev_span).line
    second = view.span_state(vertex.next_span).line
    if first.a * second.b - second.a * first.b:
        return _event_point_with_prime_universe(
            first,
            second,
            time,
            view.prime_universe,
        )
    if vertex.sliding is None:
        return None
    return sliding_point(first, vertex.sliding, time)


def edge_event_time(
    view: ExactCandidateViewV1,
    vertex_ref: object,
    peer_ref: object,
    now: EventTimeV1,
) -> tuple[EventTimeV1 | None, EventTimeOutcome]:
    vertex = view.vertex_state(vertex_ref)
    peer = view.vertex_state(peer_ref)
    if vertex.sliding is not None and peer.sliding is not None:
        here = position(view, vertex_ref, now)
        there = position(view, peer_ref, now)
        if (here.x - there.x).is_zero and (here.y - there.y).is_zero:
            return now, EventTimeOutcome.EXACT
        return None, EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT
    if vertex.sliding is not None:
        return sliding_time(
            view.span_state(vertex.prev_span).line,
            vertex.sliding,
            view.span_state(peer.next_span).line,
        )
    if peer.sliding is not None:
        return sliding_time(
            view.span_state(peer.prev_span).line,
            peer.sliding,
            view.span_state(vertex.prev_span).line,
        )
    return concurrency_time(
        view.span_state(vertex.prev_span).line,
        view.span_state(vertex.next_span).line,
        view.span_state(peer.next_span).line,
    )


def is_future(
    view: ExactCandidateViewV1,
    time: EventTimeV1,
    *vertex_refs: object,
    now: EventTimeV1,
) -> bool:
    return time.sign >= 0 and compare_times(time, now) >= 0 and all(
        compare_times(time, view.vertex_state(ref).birth) >= 0
        for ref in vertex_refs
    )


def span_end(
    view: ExactCandidateViewV1,
    vertex_ref: object,
    span_ref: object,
    time: EventTimeV1,
    *,
    at_start: bool,
) -> SqrtSumV1 | None:
    span = view.span_state(span_ref)
    place = position(view, vertex_ref, time)
    if place is not None:
        return place.x.scaled(span.line.b) - place.y.scaled(span.line.a)
    if not span.line.is_stationary:
        return None
    x0, y0, x1, y1 = span.source_span
    node_x, node_y = (x0, y0) if at_start else (x1, y1)
    return SqrtSumV1.rational(
        node_x * span.line.b - node_y * span.line.a
    )


def collapsing_span(
    view: ExactCandidateViewV1,
    vertex_ref: object,
    peer_ref: object,
    time: EventTimeV1,
) -> SqrtSumV1 | None:
    shared = view.vertex_state(vertex_ref).next_span
    low = span_end(view, vertex_ref, shared, time, at_start=True)
    high = span_end(view, peer_ref, shared, time, at_start=False)
    if low is None or high is None:
        return None
    return high - low


def _span_bound(
    view: ExactCandidateViewV1,
    span: CandidateSpanStateV1,
    span_ref: object,
    time: EventTimeV1,
    *,
    at_start: bool,
) -> SqrtSumV1 | None:
    """Проекция СВОЕГО конца пролёта; молчание закона движения — не «нет границы».

    Сначала спрашивается закон движения — где он отвечает, ответ его, и ничего
    здесь не меняется. Молчит он ровно у порта, чьи два плеча легли на одну
    прямую противонаправленно: пересечения нет, скольжения нет. Раньше это
    молчание читалось как «границы нет», то есть пролёт объявлялся
    НЕОГРАНИЧЕННЫМ и содержал любую точку плоскости. Если такой порт РОДИЛСЯ в
    этом же мгновении, молчания нет по существу: пакет знает его место точно —
    это точка локуса, и она приходит сюда как `frozen_start`/`frozen_end`.
    """

    vertex = span.start_vertex if at_start else span.end_vertex
    bound = span_end(view, vertex, span_ref, time, at_start=at_start)
    if bound is not None:
        return bound
    place = span.frozen_start if at_start else span.frozen_end
    if (
        place is None
        or span.frozen_instant is None
        or compare_times(time, span.frozen_instant) != 0
    ):
        return None
    return place.x.scaled(span.line.b) - place.y.scaled(span.line.a)


def span_containment(
    view: ExactCandidateViewV1,
    span_ref: object,
    point: EventPointV1,
    time: EventTimeV1,
) -> SpanContainmentV1:
    span = view.span_state(span_ref)
    if span.start_vertex is None or span.end_vertex is None:
        return SpanContainmentV1(False, False, False)
    here = point.x.scaled(span.line.b) - point.y.scaled(span.line.a)
    low = _span_bound(view, span, span_ref, time, at_start=True)
    high = _span_bound(view, span, span_ref, time, at_start=False)
    inside = not (
        (low is not None and (here - low).sign() < 0)
        or (high is not None and (high - here).sign() < 0)
    )
    return SpanContainmentV1(
        inside,
        low is not None and (here - low).is_zero,
        high is not None and (high - here).is_zero,
    )


def span_contains(
    view: ExactCandidateViewV1,
    span_ref: object,
    point: EventPointV1,
    time: EventTimeV1,
) -> bool:
    return span_containment(view, span_ref, point, time).inside


def sliding_projection(
    first: SupportLineV1,
    second: SupportLineV1,
    point: EventPointV1,
) -> SqrtSumV1 | None:
    if not (
        first.a * second.b - second.a * first.b == 0
        and first.a * second.a + first.b * second.b > 0
        and first.q * second.normal_squared
        == second.q * first.normal_squared
    ):
        return None
    return point.x.scaled(first.b) - point.y.scaled(first.a)
