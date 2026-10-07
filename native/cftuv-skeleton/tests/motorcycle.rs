//! The motorcycle graph, the edge law and the proof ledger on small worlds whose answers were read off the Python oracle (the differential against the oracle itself is
//! `tests/test_native_skeleton_motorcycle.py`, over the corpora, the field polygons and the kernel tests).

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::{ExactCtx, ExactError};
use cftuv_core::num::IBig;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_skeleton::candidate::{evaluate_edge_candidate, CandidateRefusal};
use cftuv_skeleton::error::{SkelError, SkelResult};
use cftuv_skeleton::line::SupportLine;
use cftuv_skeleton::motorcycle::{build_motorcycle_graph, CrashKind, MotorcycleGraph, TraceCandidateIndex, TraceOutcome};
use cftuv_skeleton::polygon::{unit_speed_squared, Loop, Polygon};
use cftuv_skeleton::time::EventTime;
use cftuv_skeleton::view::{CandidateView, PositionMemo, Sliding, SpanState, VertexRef, VertexState};

struct World {
    memory: CanonMemory,
    budget: WorkBudget,
    counts: SignCounts,
    products: ProductMemo,
}

impl World {
    fn new(budget: WorkBudget) -> World {
        World { memory: CanonMemory::new(), budget, counts: SignCounts::default(), products: ProductMemo::new() }
    }

    fn ctx(&mut self) -> ExactCtx<'_> {
        ExactCtx { memory: &mut self.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.products }
    }
}

fn rat(numerator: i64, denominator: i64) -> Rat {
    Rat::new(IBig::from(numerator), IBig::from(denominator)).unwrap()
}

fn unit_polygon(points: &[(i64, i64)]) -> Polygon {
    let size = points.len();
    let speeds = (0..size).map(|index| unit_speed_squared(points[index], points[(index + 1) % size])).collect();
    Polygon::new(vec![Loop { points: points.to_vec(), speeds }], Vec::new()).unwrap()
}

fn ell() -> Polygon {
    unit_polygon(&[(0, 0), (12, 0), (12, 6), (6, 6), (6, 12), (0, 12)])
}

fn double_notch() -> Polygon {
    unit_polygon(&[(0, 0), (20, 0), (20, 10), (14, 10), (14, 4), (11, 4), (11, 9), (5, 9), (5, 3), (2, 3), (2, 8), (0, 8)])
}

fn cross() -> Polygon {
    unit_polygon(&[(4, 0), (10, 0), (10, 4), (18, 4), (18, 8), (10, 8), (10, 20), (4, 20), (4, 8), (0, 8), (0, 4), (4, 4)])
}

fn built(polygon: &Polygon) -> (MotorcycleGraph, [u64; 6]) {
    let mut world = World::new(WorkBudget::bounded(1 << 23));
    let graph = build_motorcycle_graph(&mut world.ctx(), polygon).unwrap();
    (graph, world.budget.articles())
}

fn terms(sum: &SqrtSum) -> Vec<(u32, Rat)> {
    sum.terms().iter().map(|term| (u32::try_from(&term.radicand).unwrap(), term.coef.value().clone())).collect()
}

fn crashes(graph: &MotorcycleGraph) -> Vec<(i64, CrashKind, i64)> {
    graph.traces.iter().map(|trace| (trace.ident, trace.crash_kind, trace.crash_target)).collect()
}

#[test]
fn the_ell_has_one_trace_that_crashes_into_the_first_wall() {
    let (graph, articles) = built(&ell());
    assert_eq!((graph.grid.x_min, graph.grid.y_min, graph.grid.x_max, graph.grid.y_max, graph.grid.cell), (0, 0, 12, 12, 8));
    assert_eq!(graph.counters.as_array(), [1, 8, 2, 1, 0, 0, 0, 0]);
    assert_eq!(articles, [0, 3, 0, 0, 1, 4]);
    assert_eq!(graph.wall_index.buckets().len(), 3);
    let trace = graph.trace(3).unwrap();
    assert_eq!((trace.outcome, trace.crash_kind, trace.crash_target), (TraceOutcome::Exact, CrashKind::Wall, 0));
    // EventTimeV1(dividend=Fraction(2592, 1), divisor=SqrtSumV1(terms=((1, Fraction(432, 1)),)))
    let time = trace.crash_time.as_ref().unwrap();
    assert_eq!(time.dividend, rat(2592, 1));
    assert_eq!(terms(&time.divisor), vec![(1, rat(432, 1))]);
    assert!(trace.crash_point.as_ref().unwrap().x.is_zero() && trace.crash_point.as_ref().unwrap().y.is_zero());
    assert_eq!((terms(&trace.velocity.0), terms(&trace.velocity.1)), (vec![(1, rat(-1, 1))], vec![(1, rat(-1, 1))]));
    assert_eq!(trace.reach, Some(rat(6, 1)));
}

#[test]
fn two_traces_that_arrive_together_crash_simultaneously_into_each_other() {
    let (graph, articles) = built(&cross());
    assert_eq!(graph.counters.as_array(), [4, 72, 8, 4, 4, 4, 0, 4]);
    assert_eq!(articles, [0, 13, 0, 0, 3, 38]);
    assert_eq!(crashes(&graph), vec![(2, CrashKind::Simultaneous, 5), (5, CrashKind::Simultaneous, 2), (8, CrashKind::Simultaneous, 11), (11, CrashKind::Simultaneous, 8)]);
    let first = graph.trace(2).unwrap();
    let time = first.crash_time.as_ref().unwrap();
    assert_eq!((time.dividend.clone(), terms(&time.divisor)), (rat(1, 1), vec![(1, rat(1, 2))]));
    let point = first.crash_point.as_ref().unwrap();
    assert_eq!((terms(&point.x), terms(&point.y)), (vec![(1, rat(8, 1))], vec![(1, rat(6, 1))]));
}

#[test]
fn a_trace_that_meets_another_trace_before_its_wall_is_shortened_to_that_meeting() {
    let (graph, articles) = built(&double_notch());
    assert_eq!(graph.counters.as_array(), [4, 29, 4, 4, 1, 1, 0, 1]);
    assert_eq!(articles, [0, 22, 0, 0, 3, 12]);
    assert_eq!(crashes(&graph), vec![(4, CrashKind::Wall, 0), (5, CrashKind::Trace, 8), (8, CrashKind::Wall, 0), (9, CrashKind::Wall, 11)]);
    let shortened = graph.trace(5).unwrap();
    let time = shortened.crash_time.as_ref().unwrap();
    assert_eq!((time.dividend.clone(), terms(&time.divisor)), (rat(1, 1), vec![(1, rat(2, 7))]));
    let point = shortened.crash_point.as_ref().unwrap();
    assert_eq!((terms(&point.x), terms(&point.y)), (vec![(1, rat(15, 2))], vec![(1, rat(1, 2))]));
}

#[test]
fn a_budget_under_what_the_build_spends_refuses_by_name_at_the_operation_that_ran_out() {
    let (_graph, spent) = built(&double_notch());
    let total: u64 = spent.iter().sum();
    let mut world = World::new(WorkBudget::bounded(total / 2));
    match build_motorcycle_graph(&mut world.ctx(), &double_notch()) {
        Err(SkelError::Exact(ExactError::Canon(cftuv_canon::CanonError::Exhausted(_)))) => {}
        other => panic!("expected an exhaustion, got {:?}", other.map(|graph| graph.counters)),
    }
    assert!(world.budget.articles().iter().sum::<u64>() <= total / 2 + 1);
}

#[test]
fn the_trace_index_answers_what_it_registered_and_knows_nothing_else() {
    let polygon = ell();
    let (graph, _) = built(&polygon);
    let mut index = TraceCandidateIndex::covering(&polygon, &graph.traces).unwrap();
    assert_eq!(index.speed_bound, 1);
    for (key, (start, end, speed)) in polygon.edges().into_iter().enumerate() {
        index.register_line(key as i64, &SupportLine::with_speed(start, end, speed.clone(), 0).unwrap());
    }
    assert!(index.register_trace(3, graph.trace(3).unwrap()).unwrap());
    assert!(index.knows_vertex(3) && !index.knows_vertex(4) && index.knows_line(0) && !index.knows_line(99));
    let near = index.lines_near(3).unwrap();
    assert!(near.windows(2).all(|pair| pair[0] < pair[1]) && !near.is_empty());
    assert!(matches!(index.lines_near(4), Err(SkelError::Unsupported(_))));
    assert!(index.vertices_near(99).is_empty());
}

/// Three spans and two vertices that share the middle one: the edge law on lines of the right triangle through one point at one time.
struct TwoVertices {
    lines: Vec<SupportLine>,
    birth: EventTime,
}

impl CandidateView for TwoVertices {
    fn prime_universe(&self) -> &[cftuv_core::num::UBig] {
        &[]
    }

    fn vertex_state(&self, vertex: VertexRef) -> SkelResult<VertexState<'_>> {
        Ok(VertexState { prev_span: vertex, next_span: vertex + 1, birth: &self.birth, sliding: None::<Sliding<'_>> })
    }

    fn span_state(&self, span: u32) -> SkelResult<SpanState<'_>> {
        Ok(SpanState { line: &self.lines[span as usize], source_span: &[0, 0, 0, 0], start_vertex: None, end_vertex: None, frozen_instant: None, frozen_start: None, frozen_end: None, occurrence: None })
    }

    fn trace_bounds(&self, _ctx: &mut ExactCtx<'_>, _vertex: VertexRef, _time: &EventTime) -> SkelResult<Option<bool>> {
        Ok(None)
    }
}

#[test]
fn the_edge_law_refuses_a_solo_vertex_and_a_triple_that_never_meets_by_name_without_a_question() {
    let view = TwoVertices { lines: (0..3).map(|offset| SupportLine::new(1, 0, offset, rat(1, 1), 0).unwrap()).collect(), birth: EventTime::zero() };
    let mut world = World::new(WorkBudget::unlimited());
    let mut memo = PositionMemo::new(true);
    let now = EventTime::zero();
    let solo = evaluate_edge_candidate(&mut world.ctx(), &view, &mut memo, 0, 0, &now, true).unwrap();
    assert!(solo.candidate.is_none() && solo.effects[0].reason == CandidateRefusal::FilterSoloVertex && !solo.effects[0].needs_identity);
    assert_eq!(world.counts.total, 0);
    // three parallel lines at one speed: every cofactor is zero, the triple is always concurrent in the determinant's sense and the law has no rule for it (named, asks the host
    // for the proof identity)
    let parallel = evaluate_edge_candidate(&mut world.ctx(), &view, &mut memo, 0, 1, &now, false).unwrap();
    assert!(parallel.candidate.is_none());
    assert!(parallel.effects[0].reason == CandidateRefusal::NoRuleTripleAlwaysConcurrent && parallel.effects[0].needs_identity);
}
