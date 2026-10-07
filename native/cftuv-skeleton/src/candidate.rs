//! The candidate laws (`wavefront/candidate_law.py`, `candidate_refusal.py`): id-free, pure exact decisions whether two neighbours of the front collapse
//! (`evaluate_edge_candidate`) or a reflex or sliding vertex meets a span of the front (`evaluate_split_candidate`), with the named refusals they end with
//! as ordered effects the builder applies.
//!
//! The order of the exact questions is the cost, and it is the oracle's. For a split: the three lines first, then the time of the triple (through the
//! identity memory), then its signs against the vertex's birth and `now`, then the trace bound (a filter that answers before any place is built), the place
//! of the vertex (hydrated once, through the value memory), and last the containment in the target span. For an edge: the time of the pair, whether it is in the
//! future, the length of the span that must collapse (both ends first), the births, and last the place. An early refusal leaves the later questions unasked.

use std::rc::Rc;

use cftuv_core::exact::ExactCtx;
use cftuv_core::rat::Rat;

use crate::error::SkelResult;
use crate::line::SupportLine;
use crate::time::{compare_times, EventTime, PointRef, TimeOutcome, TimeRef};
use crate::view::{collapsing_span, concurrency_time_in, edge_event_time, is_future, position, sliding_time_in, span_containment, CandidateView, PositionMemo, SpanRef, VertexRef};

/// `CandidateRefusal`: FILTER means proven absence, NO_RULE a named open seam. `value` is the enum's `.value`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CandidateRefusal {
    FilterSoloVertex,
    FilterTripleNeverConcurrent,
    FilterEventInThePast,
    FilterPointOutsideFront,
    FilterBeyondTrace,
    FilterEdgeIsOwn,
    FilterSpanDoesNotCollapse,
    FilterSpanIsBornZero,
    NoRuleTripleAlwaysConcurrent,
    NoRuleJointIsAntiparallel,
    NoRuleJointIsCodirectional,
    NoRuleJointIsCodirectionalAtDifferentSpeeds,
    NoRuleSpanVanished,
    NoRuleMeetingNotReconnectable,
}

impl CandidateRefusal {
    /// Declaration order of the Python enum (the order of `REFUSAL_COUNTERS`).
    pub const ALL: [CandidateRefusal; 14] = [
        CandidateRefusal::FilterSoloVertex,
        CandidateRefusal::FilterTripleNeverConcurrent,
        CandidateRefusal::FilterEventInThePast,
        CandidateRefusal::FilterPointOutsideFront,
        CandidateRefusal::FilterBeyondTrace,
        CandidateRefusal::FilterEdgeIsOwn,
        CandidateRefusal::FilterSpanDoesNotCollapse,
        CandidateRefusal::FilterSpanIsBornZero,
        CandidateRefusal::NoRuleTripleAlwaysConcurrent,
        CandidateRefusal::NoRuleJointIsAntiparallel,
        CandidateRefusal::NoRuleJointIsCodirectional,
        CandidateRefusal::NoRuleJointIsCodirectionalAtDifferentSpeeds,
        CandidateRefusal::NoRuleSpanVanished,
        CandidateRefusal::NoRuleMeetingNotReconnectable,
    ];

    pub fn value(self) -> &'static str {
        match self {
            CandidateRefusal::FilterSoloVertex => "FILTER_SOLO_VERTEX",
            CandidateRefusal::FilterTripleNeverConcurrent => "FILTER_TRIPLE_NEVER_CONCURRENT",
            CandidateRefusal::FilterEventInThePast => "FILTER_EVENT_IN_THE_PAST",
            CandidateRefusal::FilterPointOutsideFront => "FILTER_POINT_OUTSIDE_FRONT",
            CandidateRefusal::FilterBeyondTrace => "FILTER_BEYOND_TRACE",
            CandidateRefusal::FilterEdgeIsOwn => "FILTER_EDGE_IS_OWN",
            CandidateRefusal::FilterSpanDoesNotCollapse => "FILTER_SPAN_DOES_NOT_COLLAPSE",
            CandidateRefusal::FilterSpanIsBornZero => "FILTER_SPAN_IS_BORN_ZERO",
            CandidateRefusal::NoRuleTripleAlwaysConcurrent => "NO_RULE_TRIPLE_ALWAYS_CONCURRENT",
            CandidateRefusal::NoRuleJointIsAntiparallel => "NO_RULE_JOINT_IS_ANTIPARALLEL",
            CandidateRefusal::NoRuleJointIsCodirectional => "NO_RULE_JOINT_IS_CODIRECTIONAL",
            CandidateRefusal::NoRuleJointIsCodirectionalAtDifferentSpeeds => "NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS",
            CandidateRefusal::NoRuleSpanVanished => "NO_RULE_SPAN_VANISHED",
            CandidateRefusal::NoRuleMeetingNotReconnectable => "NO_RULE_MEETING_NOT_RECONNECTABLE",
        }
    }

    pub fn from_value(value: &str) -> Option<CandidateRefusal> {
        CandidateRefusal::ALL.into_iter().find(|reason| reason.value() == value)
    }

    /// `refusal_counter(reason)`: `refused_<value lowercased>`.
    pub fn counter(self) -> String {
        format!("refused_{}", self.value().to_lowercase())
    }
}

/// `joint_refusal(first, second)`: why the joint of two lines has no rule.
pub fn joint_refusal(first: &SupportLine, second: &SupportLine) -> CandidateRefusal {
    let dot = i128::from(first.b) * i128::from(second.b) + i128::from(first.a) * i128::from(second.a);
    if dot <= 0 {
        return CandidateRefusal::NoRuleJointIsAntiparallel;
    }
    let scaled = |q: &Rat, norm: i128| q.mul(&Rat::from_int(cftuv_core::num::IBig::from(norm)));
    if scaled(&first.q, second.normal_squared()) != scaled(&second.q, first.normal_squared()) {
        return CandidateRefusal::NoRuleJointIsCodirectionalAtDifferentSpeeds;
    }
    CandidateRefusal::NoRuleJointIsCodirectional
}

/// `CandidateRefusalEffectV1`: the reason, whether the host must supply the proof identity (`needs_identity`, asked of its factory), and the counter
/// increments, in order. The evaluation level is always the `now` the law was given.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RefusalEffect {
    pub reason: CandidateRefusal,
    pub needs_identity: bool,
    pub counter_deltas: Vec<(&'static str, i64)>,
}

/// `SplitCandidateV1`.
#[derive(Debug, Clone)]
pub struct SplitCandidate {
    pub time: TimeRef,
    pub point: PointRef,
    pub at_start: bool,
    pub at_end: bool,
}

/// `SplitCandidateDecisionV1`.
#[derive(Debug, Clone)]
pub struct SplitDecision {
    pub candidate: Option<SplitCandidate>,
    pub effects: Vec<RefusalEffect>,
}

fn refusal(reason: CandidateRefusal, needs_identity: bool, counter_deltas: Vec<(&'static str, i64)>) -> Vec<RefusalEffect> {
    vec![RefusalEffect { reason, needs_identity, counter_deltas }]
}

fn refused(reason: CandidateRefusal, needs_identity: bool, counter_deltas: Vec<(&'static str, i64)>) -> SkelResult<SplitDecision> {
    Ok(SplitDecision { candidate: None, effects: refusal(reason, needs_identity, counter_deltas) })
}

/// `EdgeCandidateV1`: `span_unproven` is set when the collapsing span had no proof (an end without a place on a moving line); the event is accepted with it.
#[derive(Debug, Clone)]
pub struct EdgeCandidate {
    pub time: TimeRef,
    pub point: PointRef,
    pub span_unproven: bool,
}

/// `EdgeCandidateDecisionV1`.
#[derive(Debug, Clone)]
pub struct EdgeDecision {
    pub candidate: Option<EdgeCandidate>,
    pub effects: Vec<RefusalEffect>,
}

fn edge_refused(reason: CandidateRefusal, needs_identity: bool) -> SkelResult<EdgeDecision> {
    Ok(EdgeDecision { candidate: None, effects: refusal(reason, needs_identity, Vec::new()) })
}

/// `evaluate_edge_candidate(view, vertex, peer, now=..., same_vertex=...)`.
pub fn evaluate_edge_candidate<V: CandidateView>(
    ctx: &mut ExactCtx<'_>,
    view: &V,
    memo: &mut PositionMemo,
    vertex_ref: VertexRef,
    peer_ref: VertexRef,
    now: &EventTime,
    same_vertex: bool,
) -> SkelResult<EdgeDecision> {
    if same_vertex {
        return edge_refused(CandidateRefusal::FilterSoloVertex, false);
    }
    let (time, outcome) = edge_event_time(ctx, view, memo, vertex_ref, peer_ref, now)?;
    if outcome == TimeOutcome::NeverConcurrent {
        return edge_refused(CandidateRefusal::FilterTripleNeverConcurrent, false);
    }
    let time = match (outcome, time) {
        (TimeOutcome::Exact, Some(time)) => time,
        _ => return edge_refused(CandidateRefusal::NoRuleTripleAlwaysConcurrent, true),
    };
    if !is_future(ctx, view, &time, &[vertex_ref, peer_ref], now)? {
        return edge_refused(CandidateRefusal::FilterEventInThePast, false);
    }
    let span = collapsing_span(ctx, view, memo, vertex_ref, peer_ref, &time)?;
    if span.as_ref().is_some_and(|length| !length.is_zero()) {
        return edge_refused(CandidateRefusal::FilterSpanDoesNotCollapse, false);
    }
    let vertex = view.vertex_state(vertex_ref)?;
    let peer = view.vertex_state(peer_ref)?;
    if span.is_some() && compare_times(ctx, &time, vertex.birth)? == 0 && compare_times(ctx, &time, peer.birth)? == 0 {
        return edge_refused(CandidateRefusal::FilterSpanIsBornZero, false);
    }
    let Some(point) = position(ctx, view, memo, vertex_ref, &time)? else {
        let first = view.span_state(vertex.prev_span)?.line;
        let second = view.span_state(vertex.next_span)?.line;
        return edge_refused(joint_refusal(first, second), true);
    };
    Ok(EdgeDecision { candidate: Some(EdgeCandidate { time, point, span_unproven: span.is_none() }), effects: Vec::new() })
}

/// `evaluate_split_candidate(view, vertex, target, now=...)`.
pub fn evaluate_split_candidate<V: CandidateView>(
    ctx: &mut ExactCtx<'_>,
    view: &V,
    memo: &mut PositionMemo,
    vertex_ref: VertexRef,
    target_ref: SpanRef,
    now: &EventTime,
) -> SkelResult<SplitDecision> {
    let vertex = view.vertex_state(vertex_ref)?;
    let first = view.span_state(vertex.prev_span)?.line;
    let second = view.span_state(vertex.next_span)?.line;
    let target = view.span_state(target_ref)?.line;
    let (time, outcome) = match vertex.sliding {
        None => concurrency_time_in(ctx, memo, first, second, target)?,
        Some(sliding) => sliding_time_in(ctx, memo, first, sliding, target)?,
    };
    if outcome == TimeOutcome::NeverConcurrent {
        return refused(CandidateRefusal::FilterTripleNeverConcurrent, false, Vec::new());
    }
    let time = match (outcome, time) {
        (TimeOutcome::Exact, Some(time)) => time,
        _ => return refused(CandidateRefusal::NoRuleTripleAlwaysConcurrent, true, Vec::new()),
    };
    if time.sign() <= 0 || compare_times(ctx, &time, vertex.birth)? <= 0 {
        return refused(CandidateRefusal::FilterEventInThePast, false, Vec::new());
    }
    if compare_times(ctx, &time, now)? < 0 {
        return refused(CandidateRefusal::FilterEventInThePast, false, Vec::new());
    }
    if view.trace_bounds(ctx, vertex_ref, &time)? == Some(false) {
        return refused(CandidateRefusal::FilterBeyondTrace, false, vec![("split_candidates_beyond_trace", 1)]);
    }
    let Some(point) = position(ctx, view, memo, vertex_ref, &time)? else {
        return refused(joint_refusal(first, second), true, Vec::new());
    };
    let containment = span_containment(ctx, view, memo, target_ref, &point, &time)?;
    if !containment.inside {
        return refused(CandidateRefusal::FilterPointOutsideFront, false, Vec::new());
    }
    Ok(SplitDecision { candidate: Some(SplitCandidate { time: Rc::clone(&time), point, at_start: containment.at_start, at_end: containment.at_end }), effects: Vec::new() })
}
