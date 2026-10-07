//! The state and the event loop of the skeleton (`wavefront/skeleton.py::_Builder`): the front as lists of runtime edges and vertices, the queue of candidate events,
//! the motorcycle graph with its index, the proof ledger and the counters, in the oracle's exact order of cost-bearing operations.
//!
//! * `Builder::new` is `__init__` with `_seed`: the prime universe of the speeds, the loops with their fans, the traces of the reflex vertices, and the first
//!   candidates of every vertex (`_enqueue_for`), each in the oracle's order;
//! * `Builder::run` is `run`: pop a level, clear the memory of places on a NEW exact time, count the level, hand the packet to the TRANSACTION, count what it enqueued
//!   at the same time, ask whether the next packet is of the same time, and close the short LAVs and discharge the proof debt when the exact time is done;
//! * the primitives the transaction uses (`twin`, `new_vertex`, `register`, `emit`, `refuse`, `enqueue_*`, `front_vertex_met_by`, the liveness of a candidate) are
//!   public methods.
//!
//! THE TRANSACTION BOUNDARY. The oracle's `_apply_level` calls `superlevel.apply_superlevel_transaction(builder, level)`: a frozen snapshot of the front, a plan of
//! every component of the packet, the symbolic closure, and the commit that mutates the builder. The port keeps that call behind the trait [`Transaction`], which is
//! handed the builder, the exact context and the level; the loop does not know what is behind it. `snapshot` and `plans` (the first two stages) are ported; the closure
//! and the commit implement the trait.
//!
//! Identity. `exact_candidate_view` keys two of its memories by the identity of Python objects (`id()`), and which lookups hit decides the sign counters. The builder
//! gives every support line and every sliding projection an identity (`fresh_ident`) and keeps it as the oracle's object keeps its address: a twin shares its edge's
//! line, a vertex keeps its projection. Nothing else is identity-keyed.

use std::cell::Cell;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::UBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::candidate::{evaluate_edge_candidate, evaluate_split_candidate, CandidateRefusal, RefusalEffect};
use crate::error::{SkelError, SkelResult};
use crate::line::{LineValue, SupportLine};
use crate::motorcycle::{build_motorcycle_graph_with, speed_universe, MotorcycleGraph, Trace, TraceCandidateIndex};
use crate::polygon::{Point, Polygon};
use crate::proof::{EdgeKey, ProofBranch, ProofCause, ProofDisposition, ProofLedger};
use crate::queue::{CandidateEvent, EventKind, EventQueue};
use crate::skeleton::{accumulate_nodes, duplicate_node_counts, has_same_time_residual, Skeleton, SkeletonNode, SkeletonOutcome};
use crate::time::{compare_times, EventPoint, EventTime, PointRef, TimeRef};
use crate::view::{position, span_contains, CandidateView, PositionMemo, Sliding, SpanRef, SpanState, VertexRef, VertexState};

// --------------------------------------------------------------------------
// counters
// --------------------------------------------------------------------------

/// The counters the builder starts with, in the order of the oracle's dictionary (the refusal counters follow them, see [`CandidateRefusal::counter`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Counter {
    EdgeEvents,
    SplitEvents,
    StartEvents,
    SwitchEvents,
    MultiParticipantNodes,
    DiscardedStaleCandidates,
    SplitCandidatesExamined,
    SplitCandidatesBeyondTrace,
    SplitSearchExhaustiveVertices,
    SplitSearchExhaustiveSegments,
    CoincidentSplitTargets,
    Peaks,
    Ridges,
    VertexMeetingEvents,
    EdgeCollapseSpanUnprovenButAccepted,
    UnsupportedEventKindDropped,
    SameTimeEventsEnqueuedDuringLevel,
    SameTimeResidualAfterLevel,
    DuplicateExactTimePointNodes,
    MixedKindExactTimePointNodes,
    SuperlevelUnresolvableComponents,
    SuperlevelContactJunctionResolutions,
}

impl Counter {
    pub const ALL: [Counter; 22] = [
        Counter::EdgeEvents,
        Counter::SplitEvents,
        Counter::StartEvents,
        Counter::SwitchEvents,
        Counter::MultiParticipantNodes,
        Counter::DiscardedStaleCandidates,
        Counter::SplitCandidatesExamined,
        Counter::SplitCandidatesBeyondTrace,
        Counter::SplitSearchExhaustiveVertices,
        Counter::SplitSearchExhaustiveSegments,
        Counter::CoincidentSplitTargets,
        Counter::Peaks,
        Counter::Ridges,
        Counter::VertexMeetingEvents,
        Counter::EdgeCollapseSpanUnprovenButAccepted,
        Counter::UnsupportedEventKindDropped,
        Counter::SameTimeEventsEnqueuedDuringLevel,
        Counter::SameTimeResidualAfterLevel,
        Counter::DuplicateExactTimePointNodes,
        Counter::MixedKindExactTimePointNodes,
        Counter::SuperlevelUnresolvableComponents,
        Counter::SuperlevelContactJunctionResolutions,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Counter::EdgeEvents => "edge_events",
            Counter::SplitEvents => "split_events",
            Counter::StartEvents => "start_events",
            Counter::SwitchEvents => "switch_events",
            Counter::MultiParticipantNodes => "multi_participant_nodes",
            Counter::DiscardedStaleCandidates => "discarded_stale_candidates",
            Counter::SplitCandidatesExamined => "split_candidates_examined",
            Counter::SplitCandidatesBeyondTrace => "split_candidates_beyond_trace",
            Counter::SplitSearchExhaustiveVertices => "split_search_exhaustive_vertices",
            Counter::SplitSearchExhaustiveSegments => "split_search_exhaustive_segments",
            Counter::CoincidentSplitTargets => "coincident_split_targets",
            Counter::Peaks => "peaks",
            Counter::Ridges => "ridges",
            Counter::VertexMeetingEvents => "vertex_meeting_events",
            Counter::EdgeCollapseSpanUnprovenButAccepted => "edge_collapse_span_unproven_but_accepted",
            Counter::UnsupportedEventKindDropped => "unsupported_event_kind_dropped",
            Counter::SameTimeEventsEnqueuedDuringLevel => "same_time_events_enqueued_during_level",
            Counter::SameTimeResidualAfterLevel => "same_time_residual_after_level",
            Counter::DuplicateExactTimePointNodes => "duplicate_exact_time_point_nodes",
            Counter::MixedKindExactTimePointNodes => "mixed_kind_exact_time_point_nodes",
            Counter::SuperlevelUnresolvableComponents => "superlevel_unresolvable_components",
            Counter::SuperlevelContactJunctionResolutions => "superlevel_contact_junction_resolutions",
        }
    }
}

/// The prefix of the counter a refusal reason of the superlevel is counted under (`superlevel_unresolvable_reason::<REASON>`).
pub const REASON_COUNTER_PREFIX: &str = "superlevel_unresolvable_reason::";

/// `self.counters`: the fixed counters, the refusal counters, and the counters named at run time (the unresolvable reasons).
#[derive(Debug, Clone, Default)]
pub struct Counters {
    base: [i64; 22],
    refusals: [i64; 14],
    reasons: BTreeMap<String, i64>,
}

impl Counters {
    pub fn bump(&mut self, counter: Counter, by: i64) {
        self.base[counter as usize] += by;
    }

    pub fn get(&self, counter: Counter) -> i64 {
        self.base[counter as usize]
    }

    pub fn bump_refusal(&mut self, reason: CandidateRefusal) {
        self.refusals[refusal_index(reason)] += 1;
    }

    pub fn refusal(&self, reason: CandidateRefusal) -> i64 {
        self.refusals[refusal_index(reason)]
    }

    /// `counters[name] += by` for a name the oracle's dictionary holds from the start; any other name is the oracle's `KeyError`.
    pub fn add_named(&mut self, name: &str, by: i64) -> SkelResult<()> {
        if let Some(counter) = Counter::ALL.into_iter().find(|counter| counter.name() == name) {
            self.bump(counter, by);
            return Ok(());
        }
        if let Some(reason) = CandidateRefusal::ALL.into_iter().find(|reason| reason.counter() == name) {
            self.refusals[refusal_index(reason)] += by;
            return Ok(());
        }
        Err(SkelError::Unsupported(format!("KeyError: the counter {name:?} is not in the oracle's dictionary")))
    }

    /// `counters[f"superlevel_unresolvable_reason::{reason}"] = counters.get(..., 0) + 1`.
    pub fn bump_reason(&mut self, reason: &str) {
        *self.reasons.entry(format!("{REASON_COUNTER_PREFIX}{reason}")).or_insert(0) += 1;
    }

    pub fn reasons(&self) -> &BTreeMap<String, i64> {
        &self.reasons
    }

    /// Every counter by name, sorted by name (`tuple(sorted(counters.items()))`).
    pub fn sorted(&self) -> Vec<(String, i64)> {
        let mut items: Vec<(String, i64)> = Counter::ALL.iter().map(|counter| (counter.name().to_string(), self.get(*counter))).collect();
        items.extend(CandidateRefusal::ALL.iter().map(|reason| (reason.counter(), self.refusal(*reason))));
        items.extend(self.reasons.iter().map(|(name, value)| (name.clone(), *value)));
        items.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
        items
    }

    /// The counters put back from a list of `(name, value)` (the seams restore the oracle's state with it).
    pub fn restore(&mut self, items: &[(String, i64)]) -> SkelResult<()> {
        for (name, value) in items {
            if let Some(counter) = Counter::ALL.into_iter().find(|counter| counter.name() == name) {
                self.base[counter as usize] = *value;
            } else if let Some(reason) = CandidateRefusal::ALL.into_iter().find(|reason| &reason.counter() == name) {
                self.refusals[refusal_index(reason)] = *value;
            } else if name.starts_with(REASON_COUNTER_PREFIX) {
                self.reasons.insert(name.clone(), *value);
            } else {
                return Err(SkelError::Unsupported(format!("an unknown counter {name:?}")));
            }
        }
        Ok(())
    }
}

fn refusal_index(reason: CandidateRefusal) -> usize {
    CandidateRefusal::ALL.iter().position(|each| *each == reason).unwrap_or(0)
}

// --------------------------------------------------------------------------
// the front
// --------------------------------------------------------------------------

/// `_Edge`: a runtime edge, the carrier line (shared by the twins of a cut) and the occurrence of the SOURCE edge `(x0, y0, x1, y1)` (a hidden support of a fan
/// `(x, y, x, y, ordinal)`); the occurrence is the identity of a participant of an event.
#[derive(Debug, Clone)]
pub struct Edge {
    pub ident: i64,
    pub line: Rc<SupportLine>,
    pub span: Rc<Vec<i64>>,
}

impl Edge {
    /// `edge.key`.
    pub fn key(&self) -> &[i64] {
        &self.span
    }

    /// `edge.line_key`: `(a, b, c, q)`, what the index of candidates is asked by.
    pub fn line_key(&self) -> LineValue {
        self.line.value()
    }
}

/// `vertex.sliding`: the projection along the common line of a straight joint, and the identity of the object that holds it.
#[derive(Debug, Clone)]
pub struct SlidingValue {
    pub value: SqrtSum,
    pub ident: u64,
}

/// `_Vertex`.
#[derive(Debug, Clone)]
pub struct Vertex {
    pub ident: i64,
    pub prev_edge: i64,
    pub next_edge: i64,
    pub prev: i64,
    pub next: i64,
    pub birth: TimeRef,
    pub point: PointRef,
    pub reflex: bool,
    pub alive: bool,
    pub sliding: Option<SlidingValue>,
}

/// The fields `_new_vertex(**fields)` is given.
#[derive(Debug, Clone)]
pub struct NewVertex {
    pub prev_edge: i64,
    pub next_edge: i64,
    pub prev: i64,
    pub next: i64,
    pub birth: TimeRef,
    pub point: PointRef,
}

/// What the host decides about the run: the memory of places (`dense_hydration` is the reference mode with none), the number of march steps of the live oracle's
/// `march_budget` (a test that replaces it forces the exhaustion of the march), and whether the oracle has a named budget (`work_budget is not None`: only then the
/// string `superlevel` of the budget is written).
#[derive(Debug, Clone, Copy, Default)]
pub struct BuilderOptions {
    pub dense_hydration: bool,
    pub march_steps: Option<i64>,
    pub budgeted: bool,
}

/// `(vertex ids, participant edge keys, target edge keys)`: the proof identity of an observation.
pub type ProofIdentity = (Vec<i64>, Vec<EdgeKey>, Vec<EdgeKey>);

/// How much of the oracle's `level_budget(polygon)` the run may use: `4 n + 4 n r + 16` with the fan supports counted as vertices and as reflex vertices.
pub fn level_budget(polygon: &Polygon) -> i64 {
    let fan: i64 = polygon.fans.iter().map(|fan| fan.supports.len() as i64).sum();
    let reflex: i64 = polygon.loops.iter().map(|each| each.reflex_flags().iter().filter(|flag| **flag).count() as i64).sum();
    let n = polygon.vertex_count() as i64 + fan;
    let r = reflex + fan;
    4 * n + 4 * n * r + 16
}

/// `_is_reflex(first, second)`: the cross product of the directions is negative.
fn is_reflex(first: &SupportLine, second: &SupportLine) -> bool {
    let cross = i128::from(first.b) * (-i128::from(second.a)) - (-i128::from(first.a)) * i128::from(second.b);
    cross < 0
}

/// `_project(line, point)`: the projection on the direction `(b, -a)` of the line.
pub fn project(line: &SupportLine, point: &EventPoint) -> SqrtSum {
    point.x.scaled_difference(&Rat::from_i64(line.b), &point.y, &Rat::from_i64(line.a))
}

fn rational_point(point: Point) -> EventPoint {
    EventPoint { x: SqrtSum::rational(&Rat::from_i64(point.0)), y: SqrtSum::rational(&Rat::from_i64(point.1)) }
}

/// What the loop does at a call of the transaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    Continue,
    /// The run stops before it has applied the packet (the tests of the loop up to its first transaction).
    Stop,
}

/// `apply_superlevel_transaction(builder, level)`: the one place where the front changes.
pub trait Transaction {
    fn apply(&mut self, builder: &mut Builder, ctx: &mut ExactCtx<'_>, level: &[CandidateEvent]) -> SkelResult<Step>;
}

/// Why `run` returned.
#[derive(Debug, Clone)]
pub enum RunEnd {
    Finished(Box<Skeleton>),
    /// The transaction asked to stop: `levels` counted so far, the packet it was handed.
    Stopped { levels: i64, level: Vec<CandidateEvent> },
}

/// Where a run starts: at the top of the loop, or right after a transaction (the tests of the loop resume a recorded run there).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResumeAt {
    Top { levels: i64 },
    AfterTransaction { levels: i64 },
}

pub struct Builder {
    pub polygon: Polygon,
    pub options: BuilderOptions,
    /// `_prime_universe`: the proven local basis of the primitive speeds.
    pub prime_universe: Vec<UBig>,
    pub edges: Vec<Edge>,
    pub vertices: Vec<Vertex>,
    pub queue: EventQueue,
    pub edge_start: HashMap<i64, i64, FxBuild>,
    pub edge_end: HashMap<i64, i64, FxBuild>,
    pub nodes: Vec<SkeletonNode>,
    pub node_vertex_ids: Vec<Vec<i64>>,
    pub proof: ProofLedger,
    pub refusal: Option<SkeletonOutcome>,
    pub graph: Option<MotorcycleGraph>,
    pub index: Option<TraceCandidateIndex>,
    pub traces: BTreeMap<i64, Trace>,
    /// `line_id`: the identity of every distinct `(a, b, c, q)`, in order of first registration.
    pub line_id: HashMap<LineValue, usize, FxBuild>,
    pub line_order: Vec<LineValue>,
    pub edges_by_line: Vec<Vec<i64>>,
    pub unindexed_reflex: BTreeSet<i64>,
    pub sliding_vertices: BTreeSet<i64>,
    pub fan_vertices: BTreeSet<i64>,
    pub origin_vertex: BTreeMap<i64, i64>,
    pub origin_count: i64,
    pub now: TimeRef,
    pub memo: PositionMemo,
    pub counters: Counters,
    /// The oracle's `_FutureQueueV1` while the commit enqueues: a candidate is pushed only if its time is later than this one (a `compare_times` per push).
    pub future_only: Option<TimeRef>,
    next_ident: Cell<u64>,
}

fn index_of(ident: i64, len: usize, what: &str) -> SkelResult<usize> {
    usize::try_from(ident).ok().filter(|index| *index < len).ok_or_else(|| SkelError::Unsupported(format!("IndexError: {what} {ident} is not in the front of {len}")))
}

fn narrow(ident: i64, what: &str) -> SkelResult<u32> {
    u32::try_from(ident).map_err(|_| SkelError::Unsupported(format!("{what} {ident} is beyond the references of the view")))
}

impl Builder {
    /// `_Builder(polygon, MOTORCYCLE, work_budget=..., dense_hydration=...)`: everything `__init__` does, `_seed` included.
    pub fn new(ctx: &mut ExactCtx<'_>, polygon: Polygon, options: BuilderOptions) -> SkelResult<Builder> {
        let prime_universe = speed_universe(ctx, &polygon)?;
        let mut builder = Builder::empty(polygon, options, prime_universe);
        builder.seed(ctx)?;
        Ok(builder)
    }

    /// The builder of `__init__` before `_seed`: the state with nothing seeded.
    pub fn empty(polygon: Polygon, options: BuilderOptions, prime_universe: Vec<UBig>) -> Builder {
        Builder {
            polygon,
            options,
            prime_universe,
            edges: Vec::new(),
            vertices: Vec::new(),
            queue: EventQueue::new(),
            edge_start: HashMap::default(),
            edge_end: HashMap::default(),
            nodes: Vec::new(),
            node_vertex_ids: Vec::new(),
            proof: ProofLedger::new(),
            refusal: None,
            graph: None,
            index: None,
            traces: BTreeMap::new(),
            line_id: HashMap::default(),
            line_order: Vec::new(),
            edges_by_line: Vec::new(),
            unindexed_reflex: BTreeSet::new(),
            sliding_vertices: BTreeSet::new(),
            fan_vertices: BTreeSet::new(),
            origin_vertex: BTreeMap::new(),
            origin_count: 0,
            now: Rc::new(EventTime::zero()),
            memo: PositionMemo::new(!options.dense_hydration),
            counters: Counters::default(),
            future_only: None,
            next_ident: Cell::new(1),
        }
    }

    /// A new identity for a support line or a sliding projection (the address of a Python object).
    pub fn fresh_ident(&self) -> u64 {
        let ident = self.next_ident.get();
        self.next_ident.set(ident + 1);
        ident
    }

    /// The next identity `fresh_ident` will give (the tests restore the oracle's numbering with it).
    pub fn next_ident(&self) -> u64 {
        self.next_ident.get()
    }

    pub fn set_next_ident(&self, next: u64) {
        self.next_ident.set(next);
    }

    pub fn vertex_at(&self, ident: i64) -> SkelResult<&Vertex> {
        Ok(&self.vertices[index_of(ident, self.vertices.len(), "the vertex")?])
    }

    pub fn edge_at(&self, ident: i64) -> SkelResult<&Edge> {
        Ok(&self.edges[index_of(ident, self.edges.len(), "the edge")?])
    }

    // ---- the proof ----------------------------------------------------------------------------------------------------

    /// `_refuse(reason, ...)`: a named refusal of a candidate, counted and recorded at the level `now`. Nothing passes without a name.
    pub fn refuse(&mut self, reason: CandidateRefusal, identity: &ProofIdentity) -> SkelResult<()> {
        self.counters.bump_refusal(reason);
        self.proof.record_refusal(reason, &identity.0, &identity.1, &identity.2, &self.now)
    }

    /// `_record_obligation(...)`.
    #[allow(clippy::too_many_arguments)]
    pub fn record_obligation(
        &mut self,
        cause: ProofBranchOrRefusal,
        disposition: ProofDisposition,
        identity: &ProofIdentity,
        level: &EventTime,
        event_kind: Option<EventKind>,
    ) -> SkelResult<()> {
        let cause = match cause {
            ProofBranchOrRefusal::Branch(branch) => ProofCause::Branch(branch),
            ProofBranchOrRefusal::Refusal(reason) => ProofCause::Refusal(reason),
        };
        self.proof.record(cause, disposition, &identity.0, &identity.1, &identity.2, level, event_kind)
    }

    /// `_edge_keys(*edge_ids)`: the keys of the edges that exist.
    pub fn edge_keys(&self, edge_ids: &[i64]) -> Vec<EdgeKey> {
        edge_ids.iter().filter(|ident| **ident >= 0 && (**ident as usize) < self.edges.len()).map(|ident| self.edges[*ident as usize].span.to_vec()).collect()
    }

    /// `_proof_edge_endpoint_ids(edge_id)`: the runtime ids of the ends of an edge, without the liveness filter.
    pub fn proof_edge_endpoint_ids(&self, edge_id: i64) -> Vec<i64> {
        [self.edge_start.get(&edge_id), self.edge_end.get(&edge_id)].into_iter().flatten().copied().filter(|ident| *ident >= 0 && (*ident as usize) < self.vertices.len()).collect()
    }

    /// `_edge_obligation_identity(vertex, peer)`.
    pub fn edge_obligation_identity(&self, vertex: &Vertex, peer: &Vertex) -> ProofIdentity {
        let shared = vertex.next_edge;
        (vec![vertex.ident, peer.ident], self.edge_keys(&[vertex.prev_edge, shared, peer.next_edge]), self.edge_keys(&[shared]))
    }

    /// `_split_obligation_identity(vertex, edge)`.
    pub fn split_obligation_identity(&self, vertex: &Vertex, edge: &Edge) -> ProofIdentity {
        let mut ids = vec![vertex.ident];
        ids.extend(self.proof_edge_endpoint_ids(edge.ident));
        (ids, self.edge_keys(&[vertex.prev_edge, vertex.next_edge, edge.ident]), self.edge_keys(&[edge.ident]))
    }

    // ---- the seed -------------------------------------------------------------------------------------------------------

    /// `_seed`: the loops, the traces, and the first candidates of every vertex.
    fn seed(&mut self, ctx: &mut ExactCtx<'_>) -> SkelResult<()> {
        self.seed_loops()?;
        self.seed_traces(ctx)?;
        for ident in 0..self.vertices.len() as i64 {
            self.enqueue_for(ctx, ident)?;
        }
        Ok(())
    }

    fn seed_loops(&mut self) -> SkelResult<()> {
        let loops = self.polygon.loops.clone();
        for each in &loops {
            self.seed_one_loop(each)?;
        }
        for index in 0..self.edges.len() {
            let edge = self.edges[index].clone();
            self.register_line(&edge);
        }
        Ok(())
    }

    /// `_seed_one_loop(loop)`: the edges of a loop, its fans, and the vertices between them; a vertex with a fan of `k` supports is `k + 1` vertices at one point.
    fn seed_one_loop(&mut self, each: &crate::polygon::Loop) -> SkelResult<()> {
        let points = &each.points;
        let reflex = each.reflex_flags();
        let size = points.len();
        let first_edge = self.edges.len() as i64;
        for index in 0..size {
            let (start, end) = (points[index], points[(index + 1) % size]);
            let line = SupportLine::with_speed(start, end, each.speeds[index].clone(), self.fresh_ident())?;
            self.edges.push(Edge { ident: first_edge + index as i64, line: Rc::new(line), span: Rc::new(vec![start.0, start.1, end.0, end.1]) });
        }
        let first_vertex = self.vertices.len() as i64;
        // (prev edge, next edge, point, the flag of a corner without a fan)
        let mut stops: Vec<(i64, i64, Point, Option<bool>)> = Vec::new();
        for index in 0..size {
            let chain = self.seed_fan_edges(points[index])?;
            let mut ring = vec![first_edge + ((index + size - 1) % size) as i64];
            ring.extend(&chain);
            ring.push(first_edge + index as i64);
            for pair in ring.windows(2) {
                stops.push((pair[0], pair[1], points[index], if chain.is_empty() { Some(reflex[index]) } else { None }));
            }
            if chain.is_empty() {
                self.origin_vertex.insert(self.origin_count, first_vertex + stops.len() as i64 - 1);
            }
            self.origin_count += 1;
        }
        let total = stops.len() as i64;
        let zero = Rc::clone(&self.now);
        for (offset, (prev_edge, next_edge, point, corner)) in stops.into_iter().enumerate() {
            let offset = offset as i64;
            let reflex = match corner {
                None => is_reflex(&self.edges[prev_edge as usize].line, &self.edges[next_edge as usize].line),
                Some(flag) => flag,
            };
            let mut vertex = Vertex {
                ident: first_vertex + offset,
                prev_edge,
                next_edge,
                prev: first_vertex + (offset + total - 1) % total,
                next: first_vertex + (offset + 1) % total,
                birth: Rc::clone(&zero),
                point: Rc::new(rational_point(point)),
                reflex,
                alive: true,
                sliding: None,
            };
            self.classify_sliding(&mut vertex);
            let fan = corner.is_none();
            let ident = vertex.ident;
            self.vertices.push(vertex);
            self.register(ident);
            if fan {
                self.fan_vertices.insert(ident);
            }
        }
        Ok(())
    }

    /// `_seed_fan_edges(point)`: the zero-length edges of the fan of this vertex, in the order of its input.
    fn seed_fan_edges(&mut self, point: Point) -> SkelResult<Vec<i64>> {
        let Some(fan) = self.polygon.fan_at(point).cloned() else {
            return Ok(Vec::new());
        };
        let mut idents = Vec::new();
        for (ordinal, support) in fan.supports.iter().enumerate() {
            let ident = self.edges.len() as i64;
            let constant = i128::from(support.normal.0) * i128::from(point.0) + i128::from(support.normal.1) * i128::from(point.1);
            let line = SupportLine::new(support.normal.0, support.normal.1, constant, support.speed.clone(), self.fresh_ident())?;
            self.edges.push(Edge { ident, line: Rc::new(line), span: Rc::new(vec![point.0, point.1, point.0, point.1, ordinal as i64 + 1]) });
            idents.push(ident);
        }
        Ok(idents)
    }

    /// `_register_line(edge)`: the edge under its carrier line; the twins of a cut share one.
    pub fn register_line(&mut self, edge: &Edge) -> usize {
        let key = edge.line_key();
        let line = match self.line_id.get(&key) {
            Some(found) => *found,
            None => {
                let line = self.line_id.len();
                self.line_id.insert(key.clone(), line);
                self.line_order.push(key);
                self.edges_by_line.push(Vec::new());
                line
            }
        };
        self.edges_by_line[line].push(edge.ident);
        line
    }

    /// `_seed_traces`: the motorcycle graph of the input, the index over its traces, and the trace of every reflex vertex (the born-at-start ones of a fan march by themselves).
    fn seed_traces(&mut self, ctx: &mut ExactCtx<'_>) -> SkelResult<()> {
        let graph = build_motorcycle_graph_with(ctx, &self.polygon, self.options.march_steps)?;
        let mut index = TraceCandidateIndex::covering(&self.polygon, &graph.traces)?;
        for (line, key) in self.line_order.iter().enumerate() {
            index.register_line(line as i64, &SupportLine::new(key.0, key.1, key.2, key.3.clone(), 0)?);
        }
        self.graph = Some(graph);
        self.index = Some(index);
        let origins: Vec<(i64, i64)> = self.origin_vertex.iter().map(|(origin, vertex)| (*origin, *vertex)).collect();
        for (origin, ident) in origins {
            if !self.vertices[ident as usize].reflex {
                continue;
            }
            let trace = self.graph.as_ref().and_then(|graph| graph.trace(origin)).cloned();
            self.adopt_trace(ident, trace)?;
        }
        let fans: Vec<i64> = self.fan_vertices.iter().copied().collect();
        for ident in fans {
            let vertex = self.vertices[ident as usize].clone();
            if !vertex.reflex {
                continue;
            }
            let (left, right) = (Rc::clone(&self.edges[vertex.prev_edge as usize].line), Rc::clone(&self.edges[vertex.next_edge as usize].line));
            let Some(graph) = self.graph.as_mut() else {
                return Err(SkelError::Unsupported("the graph is gone".to_string()));
            };
            let trace = graph.trace_for(ctx, &left, &right, &vertex.birth, &vertex.point)?;
            self.adopt_trace(ident, Some(trace))?;
        }
        Ok(())
    }

    /// `_adopt_trace(vertex, trace)`: take the trace of a vertex, or declare it unindexable.
    pub fn adopt_trace(&mut self, ident: i64, trace: Option<Trace>) -> SkelResult<()> {
        let Some(trace) = trace.filter(|trace| trace.crash_time.is_some()) else {
            self.unindexed_reflex.insert(ident);
            return Ok(());
        };
        let registered = match self.index.as_mut() {
            Some(index) => index.register_trace(ident, &trace)?,
            None => return Err(SkelError::Unsupported("AttributeError: the index is None".to_string())),
        };
        self.traces.insert(ident, trace);
        if !registered {
            self.unindexed_reflex.insert(ident);
        }
        Ok(())
    }

    // ---- the front: reading ------------------------------------------------------------------------------------------

    /// `_register(vertex)`: the vertex starts the edge it goes along and ends the one it comes from.
    pub fn register(&mut self, ident: i64) {
        let (next_edge, prev_edge) = {
            let vertex = &self.vertices[ident as usize];
            (vertex.next_edge, vertex.prev_edge)
        };
        self.edge_start.insert(next_edge, ident);
        self.edge_end.insert(prev_edge, ident);
    }

    /// `_edge_start_vertex(edge_id)`: the live vertex a front span of this edge starts at, if the entry is not stale.
    pub fn edge_start_vertex(&self, edge_id: i64) -> Option<&Vertex> {
        let vertex = self.vertices.get(*self.edge_start.get(&edge_id)? as usize)?;
        (vertex.alive && vertex.next_edge == edge_id).then_some(vertex)
    }

    /// `_edge_end_vertex(edge_id)`.
    pub fn edge_end_vertex(&self, edge_id: i64) -> Option<&Vertex> {
        let vertex = self.vertices.get(*self.edge_end.get(&edge_id)? as usize)?;
        (vertex.alive && vertex.prev_edge == edge_id).then_some(vertex)
    }

    /// `_classify_sliding(vertex)`: a straight joint of one physical line (parallel, the same way, at one speed) slides.
    pub fn classify_sliding(&mut self, vertex: &mut Vertex) {
        let first = &self.edges[vertex.prev_edge as usize].line;
        let second = &self.edges[vertex.next_edge as usize].line;
        let dot = i128::from(first.a) * i128::from(second.a) + i128::from(first.b) * i128::from(second.b);
        let same_speed = first.q.mul(&Rat::from_int(cftuv_core::num::IBig::from(second.normal_squared()))) == second.q.mul(&Rat::from_int(cftuv_core::num::IBig::from(first.normal_squared())));
        if !(first.determinant(second) == 0 && dot > 0 && same_speed) {
            return;
        }
        let value = project(first, &vertex.point);
        vertex.sliding = Some(SlidingValue { value, ident: self.fresh_ident() });
        self.sliding_vertices.insert(vertex.ident);
    }

    /// `_position(vertex, time)`: where a vertex stands at `time` (asked first, paid once).
    pub fn position(&mut self, ctx: &mut ExactCtx<'_>, vertex: i64, time: &EventTime) -> SkelResult<Option<PointRef>> {
        let reference = narrow(vertex, "the vertex")?;
        let mut memo = std::mem::take(&mut self.memo);
        let answer = position(ctx, &*self, &mut memo, reference, time);
        self.memo = memo;
        answer
    }

    /// `_edge_span_contains(edge, point, time)`: the point lies inside the CURRENT span of the edge.
    pub fn edge_span_contains(&mut self, ctx: &mut ExactCtx<'_>, edge: i64, point: &EventPoint, time: &EventTime) -> SkelResult<bool> {
        let reference = narrow(edge, "the edge")?;
        let mut memo = std::mem::take(&mut self.memo);
        let answer = span_contains(ctx, &*self, &mut memo, reference, point, time);
        self.memo = memo;
        answer
    }

    /// `_front_vertex_met_by(event)`: the vertex of the front that stands exactly at the point of a split candidate, and whether it is a neighbour on the LAV (the
    /// meeting of a neighbour is the collapse of the edge between them, not a reconnection).
    pub fn front_vertex_met_by(&mut self, ctx: &mut ExactCtx<'_>, event: &CandidateEvent) -> SkelResult<(Option<i64>, bool)> {
        let vertex = self.vertex_at(event.vertex)?.clone();
        let candidates = [self.edge_start_vertex(event.edge).map(|other| other.ident), self.edge_end_vertex(event.edge).map(|other| other.ident)];
        for other in candidates.into_iter().flatten() {
            if other == vertex.ident {
                continue;
            }
            let Some(place) = self.position(ctx, other, &event.time)? else {
                continue;
            };
            if place.x.difference_is_zero(&event.point.x) && place.y.difference_is_zero(&event.point.y) {
                let neighbour = self.vertex_at(other)?.next;
                return Ok((Some(other), vertex.next == other || neighbour == vertex.ident));
            }
        }
        Ok((None, false))
    }

    /// `_edge_event_is_live(event)`: both vertices live and still neighbours in this order.
    pub fn edge_event_is_live(&self, event: &CandidateEvent) -> SkelResult<bool> {
        let (vertex, peer) = (self.vertex_at(event.vertex)?, self.vertex_at(event.peer)?);
        Ok(vertex.alive && peer.alive && vertex.next == peer.ident && peer.prev == vertex.ident)
    }

    /// `_split_is_live(event)`: the emitter is live, the edge is not its own, and the point is still inside the current span of the edge.
    pub fn split_is_live(&mut self, ctx: &mut ExactCtx<'_>, event: &CandidateEvent) -> SkelResult<bool> {
        let vertex = self.vertex_at(event.vertex)?.clone();
        if !vertex.alive {
            return Ok(false);
        }
        let edge = self.edge_at(event.edge)?.ident;
        if edge == vertex.prev_edge || edge == vertex.next_edge {
            return Ok(false);
        }
        self.edge_span_contains(ctx, edge, &event.point, &event.time)
    }

    // ---- candidates -----------------------------------------------------------------------------------------------------

    /// `_enqueue_for(vertex)`: the edge event of the vertex and, for a reflex or sliding one, its split events.
    pub fn enqueue_for(&mut self, ctx: &mut ExactCtx<'_>, ident: i64) -> SkelResult<()> {
        self.enqueue_edge_event(ctx, ident)?;
        let vertex = self.vertex_at(ident)?;
        if vertex.reflex || vertex.sliding.is_some() {
            self.enqueue_split_events(ctx, ident)?;
        }
        Ok(())
    }

    /// The refusals of a decision, as `_enqueue_edge_event` and `_split_candidate` apply them: the assert of the evaluation level (a `compare_times` of `now` with
    /// itself: the sign counters move by one), the counter increments, the proof identity when the law asks for one, and the named refusal.
    fn apply_effects(&mut self, ctx: &mut ExactCtx<'_>, effects: &[RefusalEffect], identity: impl Fn(&Builder) -> ProofIdentity) -> SkelResult<()> {
        for effect in effects {
            let now = Rc::clone(&self.now);
            if compare_times(ctx, &now, &now)? != 0 {
                return Err(SkelError::Unsupported("AssertionError: the evaluation level is not `now`".to_string()));
            }
            for (name, increment) in &effect.counter_deltas {
                self.counters.add_named(name, *increment)?;
            }
            let found = if effect.needs_identity { identity(self) } else { (Vec::new(), Vec::new(), Vec::new()) };
            self.refuse(effect.reason, &found)?;
        }
        Ok(())
    }

    /// `_enqueue_edge_event(vertex)`.
    pub fn enqueue_edge_event(&mut self, ctx: &mut ExactCtx<'_>, ident: i64) -> SkelResult<()> {
        let vertex = self.vertex_at(ident)?.clone();
        let peer = self.vertex_at(vertex.next)?.clone();
        let now = Rc::clone(&self.now);
        let mut memo = std::mem::take(&mut self.memo);
        let decision = evaluate_edge_candidate(ctx, &*self, &mut memo, vertex.ident as u32, peer.ident as u32, &now, peer.ident == vertex.ident);
        self.memo = memo;
        let decision = decision?;
        self.apply_effects(ctx, &decision.effects, |builder| builder.edge_obligation_identity(&vertex, &peer))?;
        if let Some(candidate) = decision.candidate {
            let event = CandidateEvent {
                kind: EventKind::Edge,
                time: candidate.time,
                point: candidate.point,
                vertex: vertex.ident,
                peer: peer.ident,
                edge: -1,
                span_unproven: candidate.span_unproven,
            };
            self.push_event(ctx, event)?;
        }
        Ok(())
    }

    /// `queue.push(event)` as the oracle's queue answers it: through the filter of the commit (`_FutureQueueV1`) when it is on.
    pub fn push_event(&mut self, ctx: &mut ExactCtx<'_>, event: CandidateEvent) -> SkelResult<()> {
        if let Some(now) = self.future_only.clone() {
            if compare_times(ctx, &event.time, &now)? <= 0 {
                return Ok(());
            }
        }
        self.queue.push(ctx, event)
    }

    /// `_enqueue_split_events(vertex)`: the candidates from the trace of the vertex when the index answers for it, else every edge.
    pub fn enqueue_split_events(&mut self, ctx: &mut ExactCtx<'_>, ident: i64) -> SkelResult<()> {
        let vertex = self.vertex_at(ident)?;
        let indexed = vertex.sliding.is_none() && self.index.as_ref().is_some_and(|index| index.knows_vertex(ident));
        if indexed {
            let lines = match self.index.as_ref() {
                Some(index) => index.lines_near(ident)?,
                None => Vec::new(),
            };
            for line in lines {
                let edges = self.edges_by_line.get(line as usize).cloned().ok_or_else(|| SkelError::Unsupported(format!("KeyError: the line {line} has no edges")))?;
                for edge in edges {
                    self.try_split(ctx, ident, edge)?;
                }
            }
            return Ok(());
        }
        self.counters.bump(Counter::SplitSearchExhaustiveVertices, 1);
        for edge in 0..self.edges.len() as i64 {
            self.try_split(ctx, ident, edge)?;
        }
        Ok(())
    }

    /// `_try_split(vertex, edge)`.
    fn try_split(&mut self, ctx: &mut ExactCtx<'_>, ident: i64, edge: i64) -> SkelResult<()> {
        let vertex = self.vertex_at(ident)?;
        if edge == vertex.prev_edge || edge == vertex.next_edge {
            return self.refuse(CandidateRefusal::FilterEdgeIsOwn, &(Vec::new(), Vec::new(), Vec::new()));
        }
        self.counters.bump(Counter::SplitCandidatesExamined, 1);
        if let Some(event) = self.split_candidate(ctx, ident, edge)? {
            self.push_event(ctx, event)?;
        }
        Ok(())
    }

    /// `_split_candidate(vertex, edge)`: the SPLIT event of a vertex against an edge, or none (every refusal named).
    pub fn split_candidate(&mut self, ctx: &mut ExactCtx<'_>, ident: i64, edge: i64) -> SkelResult<Option<CandidateEvent>> {
        let vertex = self.vertex_at(ident)?.clone();
        let target = self.edge_at(edge)?.clone();
        let now = Rc::clone(&self.now);
        let mut memo = std::mem::take(&mut self.memo);
        let decision = evaluate_split_candidate(ctx, &*self, &mut memo, vertex.ident as u32, target.ident as u32, &now);
        self.memo = memo;
        let decision = decision?;
        self.apply_effects(ctx, &decision.effects, |builder| builder.split_obligation_identity(&vertex, &target))?;
        Ok(decision.candidate.map(|candidate| CandidateEvent {
            kind: EventKind::Split,
            time: candidate.time,
            point: candidate.point,
            vertex: vertex.ident,
            peer: -1,
            edge: target.ident,
            span_unproven: false,
        }))
    }

    /// `_enqueue_splits_against(edge, excluded_vertex_ids)`: the candidates of the live reflex or sliding vertices against a span that has just appeared.
    pub fn enqueue_splits_against(&mut self, ctx: &mut ExactCtx<'_>, edge: i64, excluded: &BTreeSet<i64>) -> SkelResult<()> {
        for ident in self.split_partners(edge)? {
            if excluded.contains(&ident) {
                continue;
            }
            let vertex = self.vertex_at(ident)?;
            if !vertex.alive || !(vertex.reflex || vertex.sliding.is_some()) {
                continue;
            }
            self.counters.bump(Counter::SplitCandidatesExamined, 1);
            if let Some(event) = self.split_candidate(ctx, ident, edge)? {
                self.push_event(ctx, event)?;
            }
        }
        Ok(())
    }

    /// `_split_partners(edge)`: the vertices that may split this edge: those the index pairs with its line, the unindexed reflex vertices and the sliding ones; all of
    /// them when the index does not know the line.
    pub fn split_partners(&mut self, edge: i64) -> SkelResult<Vec<i64>> {
        let key = self.edge_at(edge)?.line_key();
        let line = self.line_id.get(&key).copied();
        let known = match (&self.index, line) {
            (Some(index), Some(line)) => index.knows_line(line as i64),
            _ => false,
        };
        if !known {
            self.counters.bump(Counter::SplitSearchExhaustiveSegments, 1);
            return Ok((0..self.vertices.len() as i64).collect());
        }
        let (Some(index), Some(line)) = (&self.index, line) else {
            return Ok(Vec::new());
        };
        let mut partners: BTreeSet<i64> = index.vertices_near(line as i64).into_iter().collect();
        partners.extend(self.unindexed_reflex.iter().copied());
        partners.extend(self.sliding_vertices.iter().copied());
        Ok(partners.into_iter().collect())
    }

    // ---- the front: writing --------------------------------------------------------------------------------------------

    /// `_twin(edge)`: a copy of the edge under a new identifier, with the same occurrence and the same carrier line (the pieces of a cut edge have one face).
    pub fn twin(&mut self, edge: i64) -> SkelResult<i64> {
        let source = self.edge_at(edge)?.clone();
        let twin = Edge { ident: self.edges.len() as i64, line: source.line, span: source.span };
        let ident = twin.ident;
        self.edges.push(twin.clone());
        self.register_line(&twin);
        Ok(ident)
    }

    /// `_new_vertex(**fields)`: a vertex born during the count; a reflex one takes the trace of the weakest bound (walls only), which is true without any theorem.
    pub fn new_vertex(&mut self, ctx: &mut ExactCtx<'_>, fields: NewVertex) -> SkelResult<i64> {
        let ident = self.vertices.len() as i64;
        let mut vertex = Vertex {
            ident,
            prev_edge: fields.prev_edge,
            next_edge: fields.next_edge,
            prev: fields.prev,
            next: fields.next,
            birth: fields.birth,
            point: fields.point,
            reflex: false,
            alive: true,
            sliding: None,
        };
        vertex.reflex = is_reflex(&self.edge_at(vertex.prev_edge)?.line, &self.edge_at(vertex.next_edge)?.line);
        self.classify_sliding(&mut vertex);
        self.vertices.push(vertex.clone());
        self.register(ident);
        if vertex.reflex && self.graph.is_some() {
            let (left, right) = (Rc::clone(&self.edge_at(vertex.prev_edge)?.line), Rc::clone(&self.edge_at(vertex.next_edge)?.line));
            let Some(graph) = self.graph.as_mut() else {
                return Ok(ident);
            };
            let trace = graph.trace_for(ctx, &left, &right, &vertex.birth, &vertex.point)?;
            self.adopt_trace(ident, Some(trace))?;
        }
        Ok(ident)
    }

    /// `_emit(kind, event, participants, converged_vertex_ids)`: a node of the skeleton, with the vertices that converged in it (sorted, repeats dropped).
    pub fn emit(&mut self, kind: EventKind, event: &CandidateEvent, participants: Vec<EdgeKey>, converged: &[i64]) {
        let converged: Vec<i64> = converged.iter().copied().collect::<BTreeSet<i64>>().into_iter().collect();
        self.nodes.push(SkeletonNode {
            kind,
            time: Rc::clone(&event.time),
            point: Rc::clone(&event.point),
            participants,
            converging_vertices: converged.len() as i64,
            kinds: Vec::new(),
            incidences: Vec::new(),
        });
        self.node_vertex_ids.push(converged);
    }

    /// `_emit_split_node(vertex, edge, event)`: the node of a split (the participants are the keys of the two edges of the vertex and of the cut edge).
    pub fn emit_split_node(&mut self, vertex: i64, edge: i64, event: &CandidateEvent) -> SkelResult<()> {
        let vertex = self.vertex_at(vertex)?;
        let keys: BTreeSet<EdgeKey> = [self.edge_at(vertex.prev_edge)?.span.to_vec(), self.edge_at(vertex.next_edge)?.span.to_vec(), self.edge_at(edge)?.span.to_vec()].into_iter().collect();
        let ident = vertex.ident;
        self.emit(EventKind::Split, event, keys.into_iter().collect(), &[ident]);
        self.counters.bump(Counter::SplitEvents, 1);
        Ok(())
    }

    /// `_close_short_lavs`: a LAV of two vertices is a ridge of the roof; no event follows it.
    pub fn close_short_lavs(&mut self) {
        for ident in 0..self.vertices.len() {
            if !self.vertices[ident].alive {
                continue;
            }
            let peer = self.vertices[ident].next as usize;
            if peer == ident || self.vertices[self.vertices[peer].next as usize].ident != ident as i64 {
                continue;
            }
            self.vertices[ident].alive = false;
            self.vertices[peer].alive = false;
            self.counters.bump(Counter::Ridges, 1);
        }
    }

    /// `_discharge_observed_obligations`: the debt of the vertices that are dead.
    pub fn discharge_observed_obligations(&mut self) {
        let dead: Vec<i64> = self.vertices.iter().filter(|vertex| !vertex.alive).map(|vertex| vertex.ident).collect();
        self.proof.discharge(&dead);
    }

    // ---- the loop ------------------------------------------------------------------------------------------------------------

    /// `run()`: the event loop. `level_limit` is the oracle's `level_budget(polygon)` read live by the host.
    pub fn run(&mut self, ctx: &mut ExactCtx<'_>, level_limit: i64, transaction: &mut dyn Transaction) -> SkelResult<RunEnd> {
        self.run_from(ctx, level_limit, ResumeAt::Top { levels: 0 }, transaction)
    }

    /// `run()` from a point of the loop: the top of an iteration, or right after the transaction of a level (what is asked of the queue next).
    pub fn run_from(&mut self, ctx: &mut ExactCtx<'_>, level_limit: i64, at: ResumeAt, transaction: &mut dyn Transaction) -> SkelResult<RunEnd> {
        let (mut levels, mut resume) = match at {
            ResumeAt::Top { levels } => (levels, false),
            ResumeAt::AfterTransaction { levels } => (levels, true),
        };
        'outer: loop {
            let mut level: Vec<CandidateEvent>;
            if !resume {
                if self.queue.is_empty() {
                    break;
                }
                if levels >= level_limit {
                    return self.finish(SkeletonOutcome::LevelBudgetExhausted, levels);
                }
                level = self.queue.pop_level(ctx)?;
                self.now = Rc::clone(&level[0].time);
                self.memo.clear();
            } else {
                level = Vec::new();
            }
            loop {
                if !resume {
                    levels += 1;
                    if self.options.budgeted {
                        ctx.budget.superlevel = levels.to_string();
                    }
                    if transaction.apply(self, ctx, &level)? == Step::Stop {
                        return Ok(RunEnd::Stopped { levels, level });
                    }
                }
                resume = false;
                let now = Rc::clone(&self.now);
                let count = self.queue.count_at_time(ctx, &now)?;
                self.counters.bump(Counter::SameTimeEventsEnqueuedDuringLevel, count as i64);
                if let Some(refusal) = self.refusal {
                    return self.finish(refusal, levels);
                }
                if !has_same_time_residual(ctx, &self.queue, &now)? {
                    break;
                }
                if levels >= level_limit {
                    return self.finish(SkeletonOutcome::LevelBudgetExhausted, levels);
                }
                level = self.queue.pop_level(ctx)?;
            }
            self.close_short_lavs();
            self.discharge_observed_obligations();
            let now = Rc::clone(&self.now);
            let residual = has_same_time_residual(ctx, &self.queue, &now)?;
            self.counters.bump(Counter::SameTimeResidualAfterLevel, i64::from(residual));
            continue 'outer;
        }
        let outcome = if self.vertices.iter().any(|vertex| vertex.alive) { SkeletonOutcome::WavefrontLeftUnresolved } else { SkeletonOutcome::Exact };
        self.finish(outcome, levels)
    }

    /// `_finish(outcome, levels)`: the proof status, the accumulated nodes, the duplicate counters and the cost of the graph, as a result.
    pub fn finish(&mut self, outcome: SkeletonOutcome, levels: i64) -> SkelResult<RunEnd> {
        let dead: Vec<i64> = self.vertices.iter().filter(|vertex| !vertex.alive).map(|vertex| vertex.ident).collect();
        let (proof_status, proof_obligations) = self.proof.finalize(&dead);
        let nodes = accumulate_nodes(&self.nodes, &self.node_vertex_ids)?;
        let mut counters = self.counters.clone();
        let (duplicates, mixed) = duplicate_node_counts(&nodes)?;
        counters.base[Counter::DuplicateExactTimePointNodes as usize] = duplicates;
        counters.base[Counter::MixedKindExactTimePointNodes as usize] = mixed;
        let mut sorted = counters.sorted();
        if let Some(graph) = &self.graph {
            for (name, value) in crate::motorcycle::GraphCounters::NAMES.iter().zip(graph.counters.as_array()) {
                sorted.push((name.to_string(), value as i64));
            }
            sorted.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
        }
        Ok(RunEnd::Finished(Box::new(Skeleton { outcome, nodes, levels, counters: sorted, proof_status, proof_obligations })))
    }
}

/// The cause of an obligation the builder records: a refusal or a branch of the proof axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProofBranchOrRefusal {
    Branch(ProofBranch),
    Refusal(CandidateRefusal),
}

// --------------------------------------------------------------------------
// the view of the front
// --------------------------------------------------------------------------

impl CandidateView for Builder {
    fn prime_universe(&self) -> &[UBig] {
        &self.prime_universe
    }

    fn vertex_state(&self, vertex: VertexRef) -> SkelResult<VertexState<'_>> {
        let vertex = self.vertex_at(i64::from(vertex))?;
        Ok(VertexState {
            prev_span: narrow(vertex.prev_edge, "the edge")?,
            next_span: narrow(vertex.next_edge, "the edge")?,
            birth: &vertex.birth,
            sliding: vertex.sliding.as_ref().map(|sliding| Sliding { value: &sliding.value, ident: sliding.ident }),
        })
    }

    fn span_state(&self, span: SpanRef) -> SkelResult<SpanState<'_>> {
        let edge = self.edge_at(i64::from(span))?;
        let end_of = |vertex: Option<&Vertex>| vertex.map(|found| narrow(found.ident, "the vertex")).transpose();
        Ok(SpanState {
            line: &edge.line,
            source_span: &edge.span,
            start_vertex: end_of(self.edge_start_vertex(edge.ident))?,
            end_vertex: end_of(self.edge_end_vertex(edge.ident))?,
            frozen_instant: None,
            frozen_start: None,
            frozen_end: None,
            occurrence: None,
        })
    }

    fn trace_bounds(&self, ctx: &mut ExactCtx<'_>, vertex: VertexRef, time: &EventTime) -> SkelResult<Option<bool>> {
        match self.traces.get(&i64::from(vertex)) {
            None => Ok(None),
            Some(trace) => Ok(Some(trace.bounds_time(ctx, time)?)),
        }
    }
}
