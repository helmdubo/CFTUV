//! The exact geometry view shared by the runtime and the symbolic candidates (`wavefront/exact_candidate_view.py`): where a vertex stands at a time,
//! when a triple of lines meets, whether a point lies inside a span of the front, and the memory of the superlevel those answers are given through.
//!
//! The oracle's view is a bundle of callbacks (`vertex_state`, `span_state`, `trace_bounds`); here it is the trait [`CandidateView`], and the port's
//! builder (or a recorded snapshot, in the seams) implements it. The memory is [`PositionMemo`], and it is cost: a place that is in it is not
//! hydrated again (no budget, no memory writes), a time that is in it is not computed again (no sign counters). Two kinds of key, as in the oracle:
//!
//! * a PLACE is keyed by the VALUE of its arguments: the two lines, the sliding projection and the time;
//! * a concurrency or sliding TIME is keyed by the IDENTITY of the line objects (`id()` in Python). Which lookups hit therefore depends on which
//!   Python objects are the same object, so the view hands out the identity of every line and of the sliding projection (`ident`); a projection the
//!   oracle makes anew on every `vertex_state` call is given a new identity on every call and so never hits.

use std::collections::HashMap;
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{IntForm, SqrtSum};

use crate::error::{SkelError, SkelResult};
use crate::line::{LineValue, SupportLine};
use crate::profile::{timed, Phase};
use crate::time::{
    combined, compare_times, concurrency_time, event_point, sliding_point, sliding_time, EventPoint, EventTime, PointRef, TimeOutcome, TimeRef,
};

pub type VertexRef = u32;
pub type SpanRef = u32;

/// `vertex.sliding`: the pinned projection along the line, and the identity of the object that holds it.
#[derive(Clone, Copy)]
pub struct Sliding<'a> {
    pub value: &'a SqrtSum,
    pub ident: u64,
}

/// `CandidateVertexStateV1`.
pub struct VertexState<'a> {
    pub prev_span: SpanRef,
    pub next_span: SpanRef,
    pub birth: &'a EventTime,
    pub sliding: Option<Sliding<'a>>,
}

/// `CandidateSpanStateV1`: the line of the span, the source nodes it spans, its end vertices, and the places the symbolic layer knows EXACTLY at
/// `frozen_instant`.
pub struct SpanState<'a> {
    pub line: &'a SupportLine,
    pub source_span: &'a [i64],
    pub start_vertex: Option<VertexRef>,
    pub end_vertex: Option<VertexRef>,
    pub frozen_instant: Option<&'a EventTime>,
    pub frozen_start: Option<&'a EventPoint>,
    pub frozen_end: Option<&'a EventPoint>,
}

/// `ExactCandidateViewV1` minus the memory (which is [`PositionMemo`], passed beside it) and the budget (the call's [`ExactCtx`]).
pub trait CandidateView {
    /// The proven local basis of primitive speeds (`view.prime_universe`).
    fn prime_universe(&self) -> &[UBig];
    fn vertex_state(&self, vertex: VertexRef) -> SkelResult<VertexState<'_>>;
    fn span_state(&self, span: SpanRef) -> SkelResult<SpanState<'_>>;
    /// `view.trace_bounds(vertex, time)`: `None` when the vertex has no trace, else `trace.bounds_time(time)` (the event is not later than the crash;
    /// `false` also for a trace that never crashes, which bounds nothing).
    fn trace_bounds(&self, ctx: &mut ExactCtx<'_>, vertex: VertexRef, time: &EventTime) -> SkelResult<Option<bool>>;
}

/// The key of a time memo entry: which question, and the identities of the objects it is asked about.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IdKey {
    pub sliding: bool,
    pub first: u64,
    pub second: u64,
    pub third: u64,
}

/// The value-key of a place: `(first, second, sliding, time)` of `ExactIdentityKeyV1`, each by value.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PlaceKey {
    pub first: LineValue,
    pub second: LineValue,
    pub sliding: Option<IntForm>,
    pub time: (Rat, IntForm),
}

/// The value of a time as a key: the dividend and the divisor in lowest terms (equal values, equal keys).
pub fn time_key(time: &EventTime) -> (Rat, IntForm) {
    (time.dividend.clone(), time.divisor.canonical_form().clone())
}

pub type TimeEntry = (Option<TimeRef>, TimeOutcome);

/// `PositionMemoV1`. `active` is `position_memo is not None and memo.admits(view.prime_universe)`: a view whose memory does not serve its basis
/// answers everything anew and writes nothing.
#[derive(Debug, Default)]
pub struct PositionMemo {
    pub active: bool,
    places: HashMap<PlaceKey, Option<PointRef>, FxBuild>,
    times: HashMap<IdKey, TimeEntry, FxBuild>,
}

impl PositionMemo {
    pub fn new(active: bool) -> PositionMemo {
        PositionMemo { active, places: HashMap::default(), times: HashMap::default() }
    }

    /// `clear()`: a new exact time begins.
    pub fn clear(&mut self) {
        self.places.clear();
        self.times.clear();
    }

    /// An entry the memory already held (the seams load the oracle's before-state this way).
    pub fn insert_place(&mut self, key: PlaceKey, place: Option<PointRef>) {
        self.places.insert(key, place);
    }

    pub fn insert_time(&mut self, key: IdKey, entry: TimeEntry) {
        self.times.insert(key, entry);
    }

    pub fn len(&self) -> (usize, usize) {
        (self.places.len(), self.times.len())
    }
}

fn rat_of(value: i128) -> Rat {
    Rat::from_int(IBig::from(value))
}

/// `_hydrate_position`: the crossing of the two lines when they are not parallel (through the universe), else the sliding point when the vertex slides,
/// else no place.
fn hydrate_position<V: CandidateView>(
    ctx: &mut ExactCtx<'_>,
    view: &V,
    first: &SupportLine,
    second: &SupportLine,
    sliding: Option<Sliding<'_>>,
    time: &EventTime,
) -> SkelResult<Option<PointRef>> {
    if first.determinant(second) != 0 {
        return Ok(Some(Rc::new(event_point(ctx, first, second, time, Some(view.prime_universe()))?)));
    }
    match sliding {
        None => Ok(None),
        Some(sliding) => Ok(Some(Rc::new(sliding_point(ctx, first, sliding.value, time)?))),
    }
}

/// `position(view, vertex, time)`: the exact place of the vertex at `time`; asked first, paid once (a hit costs nothing).
pub fn position<V: CandidateView>(ctx: &mut ExactCtx<'_>, view: &V, memo: &mut PositionMemo, vertex_ref: VertexRef, time: &EventTime) -> SkelResult<Option<PointRef>> {
    let vertex = view.vertex_state(vertex_ref)?;
    let first = view.span_state(vertex.prev_span)?.line;
    let second = view.span_state(vertex.next_span)?.line;
    if !memo.active {
        return hydrate_position(ctx, view, first, second, vertex.sliding, time);
    }
    let key = timed(Phase::MemoKey, || PlaceKey { first: first.value(), second: second.value(), sliding: vertex.sliding.map(|sliding| sliding.value.canonical_form().clone()), time: time_key(time) });
    if let Some(cached) = timed(Phase::MemoLookup, || memo.places.get(&key).cloned()) {
        return Ok(cached);
    }
    let place = hydrate_position(ctx, view, first, second, vertex.sliding, time)?;
    timed(Phase::MemoLookup, || memo.places.insert(key, place.clone()));
    Ok(place)
}

fn entry_of(result: (Option<EventTime>, TimeOutcome)) -> TimeEntry {
    (result.0.map(Rc::new), result.1)
}

/// `concurrency_time_in(view, first, second, third)`: through the memory when it serves, keyed by the identity of the three lines.
pub fn concurrency_time_in(ctx: &mut ExactCtx<'_>, memo: &mut PositionMemo, first: &SupportLine, second: &SupportLine, third: &SupportLine) -> SkelResult<TimeEntry> {
    if !memo.active {
        return Ok(entry_of(concurrency_time(ctx, first, second, third)?));
    }
    let key = IdKey { sliding: false, first: first.ident, second: second.ident, third: third.ident };
    if let Some(entry) = timed(Phase::MemoLookup, || memo.times.get(&key).cloned()) {
        return Ok(entry);
    }
    let entry = entry_of(concurrency_time(ctx, first, second, third)?);
    timed(Phase::MemoLookup, || memo.times.insert(key, entry.clone()));
    Ok(entry)
}

/// `sliding_time_in(view, line, along, other)`: the same memory, keyed by the identities of the line, the projection and the other line.
pub fn sliding_time_in(ctx: &mut ExactCtx<'_>, memo: &mut PositionMemo, line: &SupportLine, along: Sliding<'_>, other: &SupportLine) -> SkelResult<TimeEntry> {
    if !memo.active {
        return Ok(entry_of(sliding_time(ctx, line, along.value, other)?));
    }
    let key = IdKey { sliding: true, first: line.ident, second: along.ident, third: other.ident };
    if let Some(entry) = timed(Phase::MemoLookup, || memo.times.get(&key).cloned()) {
        return Ok(entry);
    }
    let entry = entry_of(sliding_time(ctx, line, along.value, other)?);
    timed(Phase::MemoLookup, || memo.times.insert(key, entry.clone()));
    Ok(entry)
}

/// The projection of a place on the direction of a span's line: `x*b - y*a`.
fn along_span(place: &EventPoint, line: &SupportLine) -> SqrtSum {
    timed(Phase::AlongSpan, || combined(&place.x, &rat_of(i128::from(line.b)), &place.y, &rat_of(i128::from(line.a))))
}

/// `span_end(view, vertex, span, time, at_start)`: the projection of the span's end on the span's line, or `None` when the end has no place and the line moves.
fn span_end<V: CandidateView>(
    ctx: &mut ExactCtx<'_>,
    view: &V,
    memo: &mut PositionMemo,
    vertex_ref: VertexRef,
    span_ref: SpanRef,
    time: &EventTime,
    at_start: bool,
) -> SkelResult<Option<SqrtSum>> {
    let span = view.span_state(span_ref)?;
    if let Some(place) = position(ctx, view, memo, vertex_ref, time)? {
        return Ok(Some(along_span(&place, span.line)));
    }
    if !span.line.is_stationary() {
        return Ok(None);
    }
    let [x0, y0, x1, y1] = *span.source_span else {
        let found = span.source_span.len();
        return Err(SkelError::Value(if found > 4 { "too many values to unpack (expected 4)".to_string() } else { format!("not enough values to unpack (expected 4, got {found})") }));
    };
    let (node_x, node_y) = if at_start { (x0, y0) } else { (x1, y1) };
    let value = i128::from(node_x) * i128::from(span.line.b) - i128::from(node_y) * i128::from(span.line.a);
    Ok(Some(SqrtSum::rational(&rat_of(value))))
}

/// `_span_bound`: the projection of the span's own end; where the law of motion is silent the places the symbolic layer froze at this very instant answer.
fn span_bound<V: CandidateView>(
    ctx: &mut ExactCtx<'_>,
    view: &V,
    memo: &mut PositionMemo,
    span: &SpanState<'_>,
    span_ref: SpanRef,
    time: &EventTime,
    at_start: bool,
) -> SkelResult<Option<SqrtSum>> {
    let vertex = if at_start { span.start_vertex } else { span.end_vertex };
    let vertex = vertex.ok_or_else(|| SkelError::Unsupported("a span bound asked of a span without that end vertex".to_string()))?;
    if let Some(bound) = span_end(ctx, view, memo, vertex, span_ref, time, at_start)? {
        return Ok(Some(bound));
    }
    let place = if at_start { span.frozen_start } else { span.frozen_end };
    let (Some(place), Some(instant)) = (place, span.frozen_instant) else {
        return Ok(None);
    };
    if compare_times(ctx, time, instant)? != 0 {
        return Ok(None);
    }
    Ok(Some(along_span(place, span.line)))
}

/// `SpanContainmentV1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpanContainment {
    pub inside: bool,
    pub at_start: bool,
    pub at_end: bool,
}

/// `span_containment(view, span, point, time)`: both bounds are asked first (each may hydrate a place); the sign `here - low` whenever there is a lower
/// bound, the sign `high - here` only when the first did not reject the point (the counters of the old short circuit).
pub fn span_containment<V: CandidateView>(ctx: &mut ExactCtx<'_>, view: &V, memo: &mut PositionMemo, span_ref: SpanRef, point: &EventPoint, time: &EventTime) -> SkelResult<SpanContainment> {
    let span = view.span_state(span_ref)?;
    if span.start_vertex.is_none() || span.end_vertex.is_none() {
        return Ok(SpanContainment { inside: false, at_start: false, at_end: false });
    }
    let here = along_span(point, span.line);
    let low = span_bound(ctx, view, memo, &span, span_ref, time, true)?;
    let high = span_bound(ctx, view, memo, &span, span_ref, time, false)?;
    let low_sign = match &low {
        None => None,
        Some(low) => Some(timed(Phase::DifferenceSign, || exact::difference_sign(ctx, &here, low))?),
    };
    let mut high_sign = None;
    if low_sign.map_or(true, |sign| sign >= 0) {
        if let Some(high) = &high {
            high_sign = Some(timed(Phase::DifferenceSign, || exact::difference_sign(ctx, high, &here))?);
        }
    }
    let inside = !(low_sign.is_some_and(|sign| sign < 0) || high_sign.is_some_and(|sign| sign < 0));
    let at_start = low.is_some() && low_sign == Some(0);
    let at_end = match (&high, high_sign) {
        (None, _) => false,
        (Some(_), Some(sign)) => sign == 0,
        (Some(high), None) => high.difference_is_zero(&here),
    };
    Ok(SpanContainment { inside, at_start, at_end })
}
