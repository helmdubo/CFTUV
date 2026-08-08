"""Pure exact candidate laws; scheduler owns ids, effects and queueing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .candidate_refusal import CandidateRefusal, joint_refusal
from .event_time import (
    EventPointV1,
    EventTimeOutcome,
    EventTimeV1,
    compare_times,
    concurrency_time,
    sliding_time,
)
from .exact_candidate_view import (
    ExactCandidateViewV1,
    collapsing_span,
    edge_event_time,
    is_future,
    position,
    span_containment,
)


@dataclass(frozen=True, slots=True)
class CandidateRefusalEffectV1:
    reason: Any
    proof_identity: tuple | None
    evaluation_level: EventTimeV1
    counter_deltas: tuple[tuple[str, int], ...] = ()


@dataclass(frozen=True, slots=True)
class EdgeCandidateV1:
    time: EventTimeV1
    point: EventPointV1
    span_unproven: bool


@dataclass(frozen=True, slots=True)
class EdgeCandidateDecisionV1:
    candidate: EdgeCandidateV1 | None
    effects: tuple[CandidateRefusalEffectV1, ...] = ()


@dataclass(frozen=True, slots=True)
class SplitCandidateV1:
    time: EventTimeV1
    point: EventPointV1
    at_start: bool
    at_end: bool


@dataclass(frozen=True, slots=True)
class SplitCandidateDecisionV1:
    candidate: SplitCandidateV1 | None
    effects: tuple[CandidateRefusalEffectV1, ...] = ()


def _effect(reason, identity, now, *counter_deltas):
    return (
        CandidateRefusalEffectV1(
            reason,
            identity,
            now,
            tuple(counter_deltas),
        ),
    )


def evaluate_edge_candidate(
    view: ExactCandidateViewV1,
    vertex_ref: object,
    peer_ref: object,
    *,
    now: EventTimeV1,
    same_vertex: bool,
    proof_identity_factory: Callable[[], tuple] | None = None,
) -> EdgeCandidateDecisionV1:
    """ID-free EDGE law with ordered observable effects as data."""

    def refuse(reason, *, needs_identity=False):
        identity = None
        if needs_identity and proof_identity_factory is not None:
            identity = proof_identity_factory()
        return EdgeCandidateDecisionV1(
            None,
            _effect(reason, identity, now),
        )

    if same_vertex:
        return refuse(CandidateRefusal.FILTER_SOLO_VERTEX)
    time, outcome = edge_event_time(view, vertex_ref, peer_ref, now)
    if outcome is EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT:
        return refuse(CandidateRefusal.FILTER_TRIPLE_NEVER_CONCURRENT)
    if outcome is not EventTimeOutcome.EXACT or time is None:
        return refuse(
            CandidateRefusal.NO_RULE_TRIPLE_ALWAYS_CONCURRENT,
            needs_identity=True,
        )
    if not is_future(view, time, vertex_ref, peer_ref, now=now):
        return refuse(CandidateRefusal.FILTER_EVENT_IN_THE_PAST)
    span = collapsing_span(view, vertex_ref, peer_ref, time)
    if span is not None and not span.is_zero:
        return refuse(CandidateRefusal.FILTER_SPAN_DOES_NOT_COLLAPSE)
    vertex = view.vertex_state(vertex_ref)
    peer = view.vertex_state(peer_ref)
    if (
        span is not None
        and compare_times(time, vertex.birth, view.budget) == 0
        and compare_times(time, peer.birth, view.budget) == 0
    ):
        return refuse(CandidateRefusal.FILTER_SPAN_IS_BORN_ZERO)
    point = position(view, vertex_ref, time)
    if point is None:
        return refuse(
            joint_refusal(
                view.span_state(vertex.prev_span).line,
                view.span_state(vertex.next_span).line,
            ),
            needs_identity=True,
        )
    return EdgeCandidateDecisionV1(
        EdgeCandidateV1(time, point, span is None)
    )


def evaluate_split_candidate(
    view: ExactCandidateViewV1,
    vertex_ref: object,
    target_ref: object,
    *,
    now: EventTimeV1,
    proof_identity_factory: Callable[[], tuple] | None = None,
) -> SplitCandidateDecisionV1:
    """ID-free SPLIT law with exact view-owned span and trace queries."""

    def refuse(reason, *, needs_identity=False, counter_deltas=()):
        identity = None
        if needs_identity and proof_identity_factory is not None:
            identity = proof_identity_factory()
        return SplitCandidateDecisionV1(
            None,
            _effect(reason, identity, now, *counter_deltas),
        )

    vertex = view.vertex_state(vertex_ref)
    first = view.span_state(vertex.prev_span).line
    second = view.span_state(vertex.next_span).line
    target = view.span_state(target_ref).line
    if vertex.sliding is None:
        time, outcome = concurrency_time(first, second, target, view.budget)
    else:
        time, outcome = sliding_time(
            first, vertex.sliding, target, view.budget
        )
    if outcome is EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT:
        return refuse(CandidateRefusal.FILTER_TRIPLE_NEVER_CONCURRENT)
    if outcome is not EventTimeOutcome.EXACT or time is None:
        return refuse(
            CandidateRefusal.NO_RULE_TRIPLE_ALWAYS_CONCURRENT,
            needs_identity=True,
        )
    if time.sign <= 0 or compare_times(time, vertex.birth, view.budget) <= 0:
        return refuse(CandidateRefusal.FILTER_EVENT_IN_THE_PAST)
    if compare_times(time, now, view.budget) < 0:
        return refuse(CandidateRefusal.FILTER_EVENT_IN_THE_PAST)
    bounded = view.trace_bounds(vertex_ref, time)
    if bounded is False:
        return refuse(
            CandidateRefusal.FILTER_BEYOND_TRACE,
            counter_deltas=(("split_candidates_beyond_trace", 1),),
        )
    point = position(view, vertex_ref, time)
    if point is None:
        return refuse(
            joint_refusal(first, second),
            needs_identity=True,
        )
    containment = span_containment(view, target_ref, point, time)
    if not containment.inside:
        return refuse(CandidateRefusal.FILTER_POINT_OUTSIDE_FRONT)
    return SplitCandidateDecisionV1(
        SplitCandidateV1(
            time,
            point,
            containment.at_start,
            containment.at_end,
        )
    )
