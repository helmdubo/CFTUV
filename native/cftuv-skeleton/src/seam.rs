//! The differential seams of the skeleton port (test-only): every function of the event layer behind ONE entry (`run`), so the Python harness can
//! call the native function on exactly the arguments the oracle saw and compare the answers.
//!
//! Wire: a request is one boundary value `[header, opcode, [argument, ...]]` (`cftuv_core::codec`), the answer one boundary value
//! `[outcome, counts, articles, log, state, nanoseconds, extras...]` (the cost answer of `cftuv_core::session`, then the time of the seam). `header` is
//! the cost header `[options, sync, budget]` or none. Outcome codes: 0 ok, 1..7 as `cftuv_core::session`, 10 `ValueError` (`[10, text]`), 12 unsupported
//! by the port (`[12, text]`), 14 `ZeroDivisorTimeError`, 15 `ParallelSupportLinesError`, 16 `DegenerateEdgeError` (`[16, text]`), 17
//! `NegativeSpeedError` (`[17, text]`).
//!
//! The view of a candidate call is a SNAPSHOT of what the oracle's callbacks answered (`SnapshotView`): the vertices and spans the call touched, the
//! traces of its vertices, the proven prime universe, and the entries the superlevel memory already held that the call hit (`memo`). A reference the
//! snapshot does not hold is a refusal of the harness (`Unsupported`), never a guess.

use std::rc::Rc;
use std::time::Instant;

use cftuv_core::codec::{Reader, Value, Writer};
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::UBig;
use cftuv_core::session::{outcome_value, CostRun, Session};
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};

use crate::candidate::{evaluate_split_candidate, SplitDecision};
use crate::error::{SkelError, SkelResult};
use crate::line::SupportLine;
use crate::queue::{CandidateEvent, EventKind, EventQueue};
use crate::repr::{py_repr, Repr};
use crate::time::{compare_times, concurrency_time, event_point, sliding_point, sliding_time, times_are_equal, EventPoint, EventTime, TimeOutcome};
use crate::view::{span_containment, time_key, CandidateView, IdKey, PlaceKey, PositionMemo, Sliding, SpanState, VertexRef, VertexState};
use crate::wire::{
    bad, fixed, flag_of, i64_of, int, int_of, line_of, list, optional, point_of, point_value, rat_of, speed_value, str_of, str_value, sum_of, time_of, time_value,
    u32_of, u64_of, ubig_of, SeamError, Wire,
};

/// `(opcode, name)` of the seams, in the order the Python harness uses.
pub const SEAMS: &[(u16, &str)] = &[
    (200, "PY_REPR"),
    (201, "SUPPORT_LINE"),
    (202, "COMPARE_TIMES"),
    (203, "TIMES_ARE_EQUAL"),
    (204, "TIME_NORMALIZED"),
    (205, "TIME_CANONICAL"),
    (206, "CONCURRENCY_TIME"),
    (207, "SLIDING_TIME"),
    (208, "SLIDING_POINT"),
    (209, "EVENT_POINT"),
    (210, "QUEUE_SCRIPT"),
    (220, "EVALUATE_SPLIT_CANDIDATE"),
    (221, "POSITION"),
    (222, "SPAN_CONTAINMENT"),
];

pub const STATUS_VALUE: u8 = 10;
pub const STATUS_UNSUPPORTED: u8 = 12;
pub const STATUS_ZERO_DIVISOR_TIME: u8 = 14;
pub const STATUS_PARALLEL_LINES: u8 = 15;
pub const STATUS_DEGENERATE_EDGE: u8 = 16;
pub const STATUS_NEGATIVE_SPEED: u8 = 17;
pub const STATUS_CELL_GRID_REJECTED: u8 = 18;

pub(crate) fn outcome_code(outcome: TimeOutcome) -> u8 {
    match outcome {
        TimeOutcome::Exact => 0,
        TimeOutcome::NeverConcurrent => 1,
        TimeOutcome::AlwaysConcurrent => 2,
    }
}

pub(crate) fn outcome_of(value: &Value) -> Wire<TimeOutcome> {
    match u32_of(value, "a time outcome")? {
        0 => Ok(TimeOutcome::Exact),
        1 => Ok(TimeOutcome::NeverConcurrent),
        2 => Ok(TimeOutcome::AlwaysConcurrent),
        _ => Err(bad("a time outcome")),
    }
}

fn answer_outcome(result: SkelResult<Value>) -> Value {
    let coded = |code: u8, parts: Vec<Value>| {
        let mut entry = vec![int(code)];
        entry.extend(parts);
        Value::List(entry)
    };
    match result {
        Ok(value) => Value::List(vec![int(0u8), value]),
        Err(SkelError::Exact(error)) => outcome_value(Err(error)),
        Err(SkelError::ZeroDivisorTime) => coded(STATUS_ZERO_DIVISOR_TIME, Vec::new()),
        Err(SkelError::ParallelSupportLines) => coded(STATUS_PARALLEL_LINES, Vec::new()),
        Err(SkelError::DegenerateEdge(text)) => coded(STATUS_DEGENERATE_EDGE, vec![str_value(&text)]),
        Err(SkelError::NegativeSpeed(text)) => coded(STATUS_NEGATIVE_SPEED, vec![str_value(&text)]),
        Err(SkelError::Value(text)) => coded(STATUS_VALUE, vec![str_value(&text)]),
        Err(SkelError::CellGridRejected(text)) => coded(STATUS_CELL_GRID_REJECTED, vec![str_value(&text)]),
        Err(SkelError::Unsupported(text)) => coded(STATUS_UNSUPPORTED, vec![str_value(&text)]),
    }
}

// --------------------------------------------------------------------------
// the snapshot view
// --------------------------------------------------------------------------

struct VertexData {
    prev_span: u32,
    next_span: u32,
    birth: EventTime,
    sliding: Option<(SqrtSum, u64)>,
}

struct SpanData {
    line: SupportLine,
    source_span: Vec<i64>,
    start: Option<u32>,
    end: Option<u32>,
    frozen_instant: Option<EventTime>,
    frozen_start: Option<EventPoint>,
    frozen_end: Option<EventPoint>,
    occurrence: Option<[SqrtSum; 4]>,
}

/// What the oracle's callbacks answered during one call, as a [`CandidateView`]. The references are the dense numbers the harness gave (`CallRecorder`), so
/// the tables are indexed by them: a lookup in the view costs what the builder's arena costs, not what a hash map costs.
pub(crate) struct SnapshotView {
    universe: Vec<UBig>,
    vertices: Vec<Option<VertexData>>,
    spans: Vec<Option<SpanData>>,
    /// `vertex -> crash time` of the vertices that have a trace (`None`: a trace that never crashes). A vertex absent here has no trace.
    traces: Vec<Option<Option<EventTime>>>,
}

fn slot<T>(table: &mut Vec<Option<T>>, reference: u32, value: T) -> Wire<()> {
    let index = reference as usize;
    if index > (1 << 20) {
        return Err(bad("a reference beyond a million: the harness numbers them densely"));
    }
    if table.len() <= index {
        table.resize_with(index + 1, || None);
    }
    table[index] = Some(value);
    Ok(())
}

fn missing(what: &str, reference: u32) -> SkelError {
    SkelError::Unsupported(format!("the snapshot holds no {what} {reference}: the harness recorded an incomplete view"))
}

impl CandidateView for SnapshotView {
    fn prime_universe(&self) -> &[UBig] {
        &self.universe
    }

    fn vertex_state(&self, vertex: VertexRef) -> SkelResult<VertexState<'_>> {
        let data = self.vertices.get(vertex as usize).and_then(Option::as_ref).ok_or_else(|| missing("vertex", vertex))?;
        Ok(VertexState { prev_span: data.prev_span, next_span: data.next_span, birth: &data.birth, sliding: data.sliding.as_ref().map(|(value, ident)| Sliding { value, ident: *ident }) })
    }

    fn span_state(&self, span: u32) -> SkelResult<SpanState<'_>> {
        let data = self.spans.get(span as usize).and_then(Option::as_ref).ok_or_else(|| missing("span", span))?;
        Ok(SpanState {
            line: &data.line,
            source_span: &data.source_span,
            start_vertex: data.start,
            end_vertex: data.end,
            frozen_instant: data.frozen_instant.as_ref(),
            frozen_start: data.frozen_start.as_ref(),
            frozen_end: data.frozen_end.as_ref(),
            occurrence: data.occurrence.as_ref(),
        })
    }

    fn trace_bounds(&self, ctx: &mut ExactCtx<'_>, vertex: VertexRef, time: &EventTime) -> SkelResult<Option<bool>> {
        match self.traces.get(vertex as usize).and_then(Option::as_ref) {
            None => Ok(None),
            Some(None) => Ok(Some(false)),
            Some(Some(crash)) => Ok(Some(compare_times(ctx, time, crash)? <= 0)),
        }
    }
}

pub(crate) fn view_of(value: &Value) -> Wire<SnapshotView> {
    let [universe, vertices, spans, traces] = fixed::<4>(value, "a view")?;
    let universe = list(universe, "a prime universe")?.iter().map(|prime| ubig_of(prime, "a prime")).collect::<Wire<_>>()?;
    let mut view = SnapshotView { universe, vertices: Vec::new(), spans: Vec::new(), traces: Vec::new() };
    for entry in list(vertices, "vertices")? {
        let [reference, prev, next, birth, sliding] = fixed::<5>(entry, "a vertex")?;
        let sliding = optional(sliding, |found| {
            let [value, ident] = fixed::<2>(found, "a sliding projection")?;
            Ok((sum_of(value, "a sliding value")?.clone(), u64_of(ident, "a sliding identity")?))
        })?;
        let data = VertexData { prev_span: u32_of(prev, "a previous span")?, next_span: u32_of(next, "a next span")?, birth: time_of(birth)?, sliding };
        slot(&mut view.vertices, u32_of(reference, "a vertex reference")?, data)?;
    }
    for entry in list(spans, "spans")? {
        let [reference, line, source, start, end, instant, frozen_start, frozen_end, occurrence] = fixed::<9>(entry, "a span")?;
        let data = SpanData {
            line: line_of(line)?,
            source_span: list(source, "a source span")?.iter().map(|node| i64_of(node, "a source node")).collect::<Wire<_>>()?,
            start: optional(start, |found| u32_of(found, "a start vertex"))?,
            end: optional(end, |found| u32_of(found, "an end vertex"))?,
            frozen_instant: optional(instant, time_of)?,
            frozen_start: optional(frozen_start, point_of)?,
            frozen_end: optional(frozen_end, point_of)?,
            occurrence: optional(occurrence, |found| {
                let [start_x, start_y, end_x, end_y] = fixed::<4>(found, "a span occurrence")?;
                Ok([sum_of(start_x, "an occurrence x")?.clone(), sum_of(start_y, "an occurrence y")?.clone(), sum_of(end_x, "an occurrence x")?.clone(), sum_of(end_y, "an occurrence y")?.clone()])
            })?,
        };
        slot(&mut view.spans, u32_of(reference, "a span reference")?, data)?;
    }
    for entry in list(traces, "traces")? {
        let [reference, crash] = fixed::<2>(entry, "a trace")?;
        slot(&mut view.traces, u32_of(reference, "a trace vertex")?, optional(crash, time_of)?)?;
    }
    Ok(view)
}

/// `[active, entries]`; an entry is `[0, first line, second line, sliding sum or none, time, place or none]` (a place),
/// `[1, id, id, id, time or none, outcome]` (a concurrency time) or `[2, ...]` (a sliding time).
pub(crate) fn memo_of(value: &Value) -> Wire<PositionMemo> {
    let [active, entries] = fixed::<2>(value, "a memo")?;
    let mut memo = PositionMemo::new(flag_of(active, "the memo flag")?);
    for entry in list(entries, "memo entries")? {
        let items = list(entry, "a memo entry")?;
        match items {
            [Value::Int(number), first, second, sliding, time, place] if *number == 0.into() => {
                let key = PlaceKey {
                    first: line_of(first)?.value(),
                    second: line_of(second)?.value(),
                    sliding: optional(sliding, |found| Ok(sum_of(found, "a memo sliding")?.canonical_form().clone()))?,
                    time: time_key(&time_of(time)?),
                };
                memo.insert_place(key, optional(place, point_of)?.map(Rc::new));
            }
            [kind, first, second, third, time, outcome] => {
                let sliding = match u32_of(kind, "a memo kind")? {
                    1 => false,
                    2 => true,
                    _ => return Err(bad("a memo kind")),
                };
                let key = IdKey { sliding, first: u64_of(first, "an identity")?, second: u64_of(second, "an identity")?, third: u64_of(third, "an identity")? };
                memo.insert_time(key, (optional(time, time_of)?.map(Rc::new), outcome_of(outcome)?));
            }
            _ => return Err(bad("a memo entry")),
        }
    }
    Ok(memo)
}

pub(crate) fn effects_value(effects: &[crate::candidate::RefusalEffect]) -> Value {
    Value::List(
        effects
            .iter()
            .map(|effect| {
                let deltas = effect.counter_deltas.iter().map(|(name, delta)| Value::List(vec![str_value(name), int(*delta)])).collect();
                Value::List(vec![str_value(effect.reason.value()), Value::Bool(effect.needs_identity), Value::List(deltas)])
            })
            .collect(),
    )
}

pub(crate) fn growth_value(memo_before: (usize, usize), memo_after: (usize, usize)) -> Value {
    let added = |after: usize, before: usize| int(after.saturating_sub(before) as u64);
    Value::List(vec![added(memo_after.0, memo_before.0), added(memo_after.1, memo_before.1)])
}

fn decision_value(decision: &SplitDecision, memo_before: (usize, usize), memo_after: (usize, usize)) -> Value {
    let candidate = match &decision.candidate {
        None => Value::None,
        Some(candidate) => Value::List(vec![time_value(&candidate.time), point_value(&candidate.point), Value::Bool(candidate.at_start), Value::Bool(candidate.at_end)]),
    };
    Value::List(vec![candidate, effects_value(&decision.effects), growth_value(memo_before, memo_after)])
}

// --------------------------------------------------------------------------
// `repr`
// --------------------------------------------------------------------------

/// An owned node of a repr request: `[tag, payload]`.
enum Node {
    None,
    Bool(bool),
    Int(cftuv_core::num::IBig),
    Str(String),
    Frac(cftuv_core::rat::Rat),
    Tuple(Vec<Node>),
    List(Vec<Node>),
    Data(String, Vec<(String, Node)>),
    Member(String, String, Box<Node>),
    Sum(SqrtSum),
    Time(EventTime),
    Point(EventPoint),
}

fn node_of(value: &Value) -> Wire<Node> {
    let [tag, payload] = fixed::<2>(value, "a repr node")?;
    let nodes = |items: &Value| -> Wire<Vec<Node>> { list(items, "repr items")?.iter().map(node_of).collect() };
    Ok(match u32_of(tag, "a repr tag")? {
        0 => Node::None,
        1 => Node::Bool(flag_of(payload, "a repr bool")?),
        2 => Node::Int(int_of(payload, "a repr int")?),
        3 => Node::Str(str_of(payload, "a repr string")?),
        4 => Node::Frac(rat_of(payload, "a repr fraction")?),
        5 => Node::Tuple(nodes(payload)?),
        6 => Node::List(nodes(payload)?),
        7 => {
            let [name, fields] = fixed::<2>(payload, "a repr dataclass")?;
            let fields = list(fields, "repr fields")?
                .iter()
                .map(|field| {
                    let [field_name, field_value] = fixed::<2>(field, "a repr field")?;
                    Ok((str_of(field_name, "a repr field name")?, node_of(field_value)?))
                })
                .collect::<Wire<_>>()?;
            Node::Data(str_of(name, "a repr class name")?, fields)
        }
        8 => {
            let [class, member, inner] = fixed::<3>(payload, "a repr member")?;
            Node::Member(str_of(class, "a repr enum class")?, str_of(member, "a repr enum member")?, Box::new(node_of(inner)?))
        }
        9 => Node::Sum(sum_of(payload, "a repr sum")?.clone()),
        10 => Node::Time(time_of(payload)?),
        11 => Node::Point(point_of(payload)?),
        _ => return Err(bad("a repr tag")),
    })
}

fn borrowed(node: &Node) -> Repr<'_> {
    match node {
        Node::None => Repr::None,
        Node::Bool(flag) => Repr::Bool(*flag),
        Node::Int(number) => Repr::Int(number.clone()),
        Node::Str(text) => Repr::Str(text),
        Node::Frac(number) => Repr::Frac(number),
        Node::Tuple(items) => Repr::Tuple(items.iter().map(borrowed).collect()),
        Node::List(items) => Repr::List(items.iter().map(borrowed).collect()),
        Node::Data(name, fields) => Repr::Data(name, fields.iter().map(|(field, item)| (field.as_str(), borrowed(item))).collect()),
        Node::Member(class, member, inner) => Repr::Member(class, member, Box::new(borrowed(inner))),
        Node::Sum(sum) => Repr::Sum(sum),
        Node::Time(time) => Repr::Time(time),
        Node::Point(point) => Repr::Point(point),
    }
}

// --------------------------------------------------------------------------
// the seams
// --------------------------------------------------------------------------

pub(crate) fn time_entry_value(entry: (Option<EventTime>, TimeOutcome)) -> Value {
    Value::List(vec![entry.0.as_ref().map_or(Value::None, time_value), int(outcome_code(entry.1))])
}

fn queue_script(ctx: &mut ExactCtx<'_>, script: &Value) -> Wire<SkelResult<Value>> {
    let zero = Rc::new(EventPoint { x: SqrtSum::zero(), y: SqrtSum::zero() });
    let mut queue = EventQueue::new();
    let mut results = Vec::new();
    for op in list(script, "a queue script")? {
        let items = list(op, "a queue op")?;
        let step: SkelResult<Value> = match items {
            [code, time, tag] if u32_of(code, "a queue opcode")? == 0 => {
                let event = CandidateEvent { kind: EventKind::Split, time: Rc::new(time_of(time)?), point: Rc::clone(&zero), vertex: i64_of(tag, "an event tag")?, peer: -1, edge: -1, span_unproven: false };
                queue.push(ctx, event).map(|()| Value::None)
            }
            [code] if u32_of(code, "a queue opcode")? == 1 => queue.pop_level(ctx).map(|level| Value::List(level.iter().map(|event| int(event.vertex)).collect())),
            [code] if u32_of(code, "a queue opcode")? == 2 => Ok(queue.peek_time().map_or(Value::None, |time| time_value(time))),
            [code, time] if u32_of(code, "a queue opcode")? == 3 => queue.count_at_time(ctx, &time_of(time)?).map(|count| int(count as u64)),
            _ => return Err(bad("a queue op")),
        };
        match step {
            Ok(value) => results.push(value),
            Err(error) => return Ok(Err(error)),
        }
    }
    let arrangement = queue.arrangement().into_iter().map(int).collect();
    Ok(Ok(Value::List(vec![Value::List(results), Value::List(arrangement), int(queue.pushed), int(queue.popped)])))
}

fn dispatch(code: u16, args: &[Value], ctx: &mut ExactCtx<'_>, extras: &mut Vec<Value>) -> Wire<SkelResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    let expect = |count: usize| if args.len() == count { Ok(()) } else { Err(bad("the argument count")) };
    Ok(match code {
        200 => {
            expect(1)?;
            let node = node_of(at(0)?)?;
            match py_repr(&borrowed(&node)) {
                Ok(text) => Ok(str_value(&text)),
                Err(error) => Err(SkelError::Unsupported(error.0)),
            }
        }
        201 => {
            expect(6)?;
            let (start, end) = ((i64_of(at(0)?, "sx")?, i64_of(at(1)?, "sy")?), (i64_of(at(2)?, "ex")?, i64_of(at(3)?, "ey")?));
            let line = if u32_of(at(5)?, "a mode")? == 0 { SupportLine::with_speed(start, end, rat_of(at(4)?, "a speed")?, 0) } else { SupportLine::through(start, end, 0) };
            line.map(|line| Value::List(vec![int(line.a), int(line.b), int(line.c), speed_value(&line.q)]))
        }
        202 => {
            expect(2)?;
            compare_times(ctx, &time_of(at(0)?)?, &time_of(at(1)?)?).map(|sign| int(i64::from(sign)))
        }
        203 => {
            expect(2)?;
            Ok(Value::Bool(times_are_equal(&time_of(at(0)?)?, &time_of(at(1)?)?)))
        }
        204 => {
            expect(2)?;
            EventTime::normalized(ctx, rat_of(at(0)?, "a dividend")?, sum_of(at(1)?, "a divisor")?.clone()).map(|time| time_value(&time))
        }
        205 => {
            expect(1)?;
            time_of(at(0)?)?.canonical().map(|time| time_value(&time))
        }
        206 => {
            expect(3)?;
            concurrency_time(ctx, &line_of(at(0)?)?, &line_of(at(1)?)?, &line_of(at(2)?)?).map(time_entry_value)
        }
        207 => {
            expect(3)?;
            sliding_time(ctx, &line_of(at(0)?)?, sum_of(at(1)?, "along")?, &line_of(at(2)?)?).map(time_entry_value)
        }
        208 => {
            expect(3)?;
            sliding_point(ctx, &line_of(at(0)?)?, sum_of(at(1)?, "along")?, &time_of(at(2)?)?).map(|point| point_value(&point))
        }
        209 => {
            expect(4)?;
            let universe = match at(3)? {
                Value::None => None,
                other => Some(list(other, "a prime universe")?.iter().map(|prime| ubig_of(prime, "a prime")).collect::<Wire<Vec<UBig>>>()?),
            };
            event_point(ctx, &line_of(at(0)?)?, &line_of(at(1)?)?, &time_of(at(2)?)?, universe.as_deref()).map(|point| point_value(&point))
        }
        210 => {
            expect(1)?;
            return queue_script(ctx, at(0)?);
        }
        220 => {
            expect(5)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (vertex, target, now) = (u32_of(at(2)?, "a vertex")?, u32_of(at(3)?, "a target span")?, time_of(at(4)?)?);
            let before = memo.len();
            crate::profile::reset();
            let started = Instant::now();
            let decision = evaluate_split_candidate(ctx, &view, &mut memo, vertex, target, &now);
            extras.push(int(started.elapsed().as_nanos() as u64));
            let phases = crate::profile::take();
            if !phases.is_empty() {
                extras.push(Value::List(phases.into_iter().map(int).collect()));
            }
            decision.map(|decision| decision_value(&decision, before, memo.len()))
        }
        221 => {
            expect(4)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (vertex, time) = (u32_of(at(2)?, "a vertex")?, time_of(at(3)?)?);
            let before = memo.len();
            let started = Instant::now();
            let place = crate::view::position(ctx, &view, &mut memo, vertex, &time);
            extras.push(int(started.elapsed().as_nanos() as u64));
            place.map(|place| Value::List(vec![place.as_ref().map_or(Value::None, |place| point_value(place)), Value::List(vec![int((memo.len().0.saturating_sub(before.0)) as u64), int((memo.len().1.saturating_sub(before.1)) as u64)])]))
        }
        222 => {
            expect(5)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (span, point, time) = (u32_of(at(2)?, "a span")?, point_of(at(3)?)?, time_of(at(4)?)?);
            let before = memo.len();
            let started = Instant::now();
            let found = span_containment(ctx, &view, &mut memo, span, &point, &time);
            extras.push(int(started.elapsed().as_nanos() as u64));
            found.map(|found| {
                Value::List(vec![
                    Value::Bool(found.inside),
                    Value::Bool(found.at_start),
                    Value::Bool(found.at_end),
                    Value::List(vec![int((memo.len().0.saturating_sub(before.0)) as u64), int((memo.len().1.saturating_sub(before.1)) as u64)]),
                ])
            })
        }
        other => return crate::seam_graph::dispatch(other, args, ctx, extras),
    })
}

fn default_header() -> Value {
    let table = || Value::List(vec![Value::Bool(false), Value::List(Vec::new()), Value::List(Vec::new())]);
    Value::List(vec![int(0u8), Value::List(vec![table(), table(), table(), table()]), Value::None])
}

/// Runs one seam on `session` and answers the cost answer buffer.
pub fn run(session: &mut Session, request: &[u8]) -> Result<Vec<u8>, SeamError> {
    let mut reader = Reader::new(request, true);
    let parsed = reader.get_value()?;
    reader.finish()?;
    let [header, code, args] = fixed::<3>(&parsed, "a request")?;
    let code = u16::try_from(&int_of(code, "an opcode")?).map_err(|_| bad("an opcode"))?;
    let args = list(args, "the arguments")?;
    let header = if matches!(header, Value::None) { default_header() } else { header.clone() };
    let mut cost = CostRun::begin(session, &header).map_err(|error| SeamError(error.to_string()))?;
    let mut counts = SignCounts::default();
    let mut extras: Vec<Value> = Vec::new();
    let started = Instant::now();
    let result = {
        let mut ctx = ExactCtx { memory: &mut session.memory, budget: cost.budget_mut(), counts: &mut counts, products: &mut session.products };
        dispatch(code, args, &mut ctx, &mut extras)?
    };
    let elapsed = started.elapsed().as_nanos() as u64;
    let mut answer = cost.answer(session, answer_outcome(result), &counts);
    if let Value::List(parts) = &mut answer {
        parts.push(int(elapsed));
        parts.extend(extras);
    }
    let mut writer = Writer::new();
    writer.put_value(&answer);
    Ok(writer.into_bytes())
}

/// The table [`SEAMS`] as the harness reads it (kept beside `run` so a new opcode is one edit).
pub fn table() -> Vec<(u16, &'static str)> {
    SEAMS.iter().chain(crate::seam_graph::SEAMS).chain(crate::seam_builder::SEAMS).chain(crate::seam_primitive::SEAMS).copied().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_seam_table_has_no_duplicate_opcode() {
        let mut codes: Vec<u16> = table().iter().map(|(code, _)| *code).collect();
        codes.sort_unstable();
        codes.dedup();
        assert_eq!(codes.len(), table().len());
    }

    #[test]
    fn a_whole_speed_is_an_int_and_a_dividend_stays_a_fraction() {
        let whole = cftuv_core::rat::Rat::from_int(cftuv_core::num::IBig::from(3));
        assert!(matches!(crate::wire::frac_value(&whole), Value::Frac(_)));
        assert!(matches!(speed_value(&whole), Value::Int(_)));
    }
}
