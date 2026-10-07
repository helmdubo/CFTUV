//! The leaf on small worlds whose answers are known by hand (the differential against the Python oracle is `tests/test_native_skeleton_parts.py`).

use std::rc::Rc;

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::num::{IBig, UBig};
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_skeleton::candidate::{evaluate_split_candidate, evaluate_split_candidate_gated, CandidateRefusal, NowGate};
use cftuv_skeleton::error::SkelResult;
use cftuv_skeleton::line::SupportLine;
use cftuv_skeleton::queue::{CandidateEvent, EventKind, EventQueue};
use cftuv_skeleton::time::{compare_times, concurrency_time, times_are_equal, EventPoint, EventTime, TimeOutcome};
use cftuv_skeleton::view::{CandidateView, PositionMemo, Sliding, SpanState, VertexRef, VertexState};

struct World {
    memory: CanonMemory,
    budget: WorkBudget,
    counts: SignCounts,
    products: ProductMemo,
}

impl World {
    fn new() -> World {
        World { memory: CanonMemory::new(), budget: WorkBudget::unlimited(), counts: SignCounts::default(), products: ProductMemo::new() }
    }

    fn ctx(&mut self) -> ExactCtx<'_> {
        ExactCtx { memory: &mut self.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.products }
    }
}

fn rat(numerator: i64, denominator: i64) -> Rat {
    Rat::new(IBig::from(numerator), IBig::from(denominator)).unwrap()
}

fn rational_time(numerator: i64, denominator: i64) -> EventTime {
    EventTime::new(rat(numerator, 1), SqrtSum::rational(&rat(denominator, 1)))
}

fn lines_of_the_right_triangle() -> [SupportLine; 3] {
    [
        SupportLine::through((0, 0), (4, 0), 1).unwrap(),
        SupportLine::through((4, 0), (4, 4), 2).unwrap(),
        SupportLine::through((4, 4), (0, 0), 3).unwrap(),
    ]
}

#[test]
fn three_lines_of_a_right_triangle_meet_at_the_time_the_oracle_prints() {
    let mut world = World::new();
    let [first, second, third] = lines_of_the_right_triangle();
    let (time, outcome) = concurrency_time(&mut world.ctx(), &first, &second, &third).unwrap();
    assert_eq!(outcome, TimeOutcome::Exact);
    let time = time.unwrap();
    // EventTimeV1(dividend=Fraction(256, 1), divisor=SqrtSumV1(terms=((1, Fraction(128, 1)), (2, Fraction(64, 1)))))
    assert_eq!(time.dividend, rat(256, 1));
    let terms: Vec<(u32, Rat)> = time.divisor.terms().iter().map(|term| (u32::try_from(&term.radicand).unwrap(), term.coef.value().clone())).collect();
    assert_eq!(terms, vec![(1, rat(128, 1)), (2, rat(64, 1))]);
    assert_eq!(world.counts.total, 1, "one sign: the divisor's, for the sign of the time");
}

#[test]
fn compare_times_is_exact_on_a_root_and_equality_is_an_empty_difference() {
    let mut world = World::new();
    let root_of_two = exact::radical(&mut world.ctx(), &rat(1, 1), &rat(2, 1)).unwrap();
    let one = rational_time(1, 1);
    let one_over_root_two = EventTime::new(rat(1, 1), root_of_two.clone());
    let two_over_two_root_two = EventTime::new(rat(2, 1), root_of_two.scaled(&rat(2, 1)));
    assert_eq!(compare_times(&mut world.ctx(), &one, &one_over_root_two).unwrap(), 1);
    assert_eq!(compare_times(&mut world.ctx(), &one_over_root_two, &one).unwrap(), -1);
    assert_eq!(compare_times(&mut world.ctx(), &one_over_root_two, &two_over_two_root_two).unwrap(), 0);
    assert!(times_are_equal(&one_over_root_two, &two_over_two_root_two));
    assert!(!times_are_equal(&one, &one_over_root_two));
    assert_eq!(world.counts.total, 3, "the equality test pays no sign");
    assert_eq!(world.counts.closed_by_conjugation, 0);
}

#[test]
fn the_queue_takes_a_whole_level_and_keeps_the_counts() {
    let mut world = World::new();
    let mut queue = EventQueue::new();
    let point = Rc::new(EventPoint { x: SqrtSum::zero(), y: SqrtSum::zero() });
    for (tag, time) in [(0, 3), (1, 1), (2, 2), (3, 1), (4, 2), (5, 5)] {
        let event = CandidateEvent { kind: EventKind::Edge, time: Rc::new(rational_time(time, 1)), point: Rc::clone(&point), vertex: tag, peer: -1, edge: -1, span_unproven: false };
        queue.push(&mut world.ctx(), event).unwrap();
    }
    assert_eq!(queue.pushed, 6);
    let first = queue.pop_level(&mut world.ctx()).unwrap();
    let mut tags: Vec<i64> = first.iter().map(|event| event.vertex).collect();
    tags.sort_unstable();
    assert_eq!(tags, vec![1, 3]);
    assert_eq!(queue.count_at_time(&mut world.ctx(), &rational_time(2, 1)).unwrap(), 2);
    assert_eq!(queue.count_at_time(&mut world.ctx(), &rational_time(4, 1)).unwrap(), 0);
    let second = queue.pop_level(&mut world.ctx()).unwrap();
    assert_eq!(second.len(), 2);
    assert_eq!((queue.popped, queue.len()), (4, 2));
    assert_eq!(queue.pop_level(&mut world.ctx()).unwrap().len(), 1);
    assert_eq!(queue.pop_level(&mut world.ctx()).unwrap().len(), 1);
    assert!(queue.pop_level(&mut world.ctx()).unwrap().is_empty());
    assert_eq!((queue.popped, queue.len()), (6, 0));
}

/// One reflex-like vertex between the first two lines, a target span of the third line without end vertices.
struct TriangleView {
    lines: [SupportLine; 3],
    birth: EventTime,
    universe: Vec<UBig>,
}

impl CandidateView for TriangleView {
    fn prime_universe(&self) -> &[UBig] {
        &self.universe
    }

    fn vertex_state(&self, _vertex: VertexRef) -> SkelResult<VertexState<'_>> {
        Ok(VertexState { prev_span: 0, next_span: 1, birth: &self.birth, sliding: None::<Sliding<'_>> })
    }

    fn span_state(&self, span: u32) -> SkelResult<SpanState<'_>> {
        Ok(SpanState { line: &self.lines[span as usize], source_span: &[0, 0, 0, 0], start_vertex: None, end_vertex: None, frozen_instant: None, frozen_start: None, frozen_end: None, occurrence: None })
    }

    fn trace_bounds(&self, _ctx: &mut ExactCtx<'_>, _vertex: VertexRef, _time: &EventTime) -> SkelResult<Option<bool>> {
        Ok(None)
    }
}

fn triangle_view(birth: EventTime) -> TriangleView {
    TriangleView { lines: lines_of_the_right_triangle(), birth, universe: vec![UBig::from(2u8)] }
}

#[test]
fn a_candidate_born_before_its_time_gets_a_place_and_is_refused_outside_the_front() {
    let mut world = World::new();
    let view = triangle_view(rational_time(0, 1));
    let mut memo = PositionMemo::new(true);
    let decision = evaluate_split_candidate(&mut world.ctx(), &view, &mut memo, 0, 2, &rational_time(0, 1)).unwrap();
    assert!(decision.candidate.is_none());
    assert_eq!(decision.effects.len(), 1);
    assert_eq!(decision.effects[0].reason, CandidateRefusal::FilterPointOutsideFront);
    assert!(!decision.effects[0].needs_identity && decision.effects[0].counter_deltas.is_empty());
    assert_eq!(memo.len(), (1, 1), "the time of the triple and the place of the vertex are remembered");
    assert_eq!(world.budget.exact_position_hydrations, 1);
    // the same question again: both answers come from the memory, nothing is paid
    let (signs, hydrations) = (world.counts.total, world.budget.exact_position_hydrations);
    let again = evaluate_split_candidate(&mut world.ctx(), &view, &mut memo, 0, 2, &rational_time(0, 1)).unwrap();
    assert_eq!(again.effects[0].reason, CandidateRefusal::FilterPointOutsideFront);
    assert_eq!(world.budget.exact_position_hydrations, hydrations);
    assert!(world.counts.total > signs, "the signs against the birth and `now` are asked every time; only the time and the place are remembered");
}

#[test]
fn a_gate_of_exactly_now_answers_a_later_time_with_no_candidate_and_no_effects_before_the_trace_the_place_and_the_containment() {
    let mut world = World::new();
    let view = triangle_view(rational_time(0, 1));
    let mut memo = PositionMemo::new(true);
    let now = rational_time(0, 1);
    let decision = evaluate_split_candidate_gated(&mut world.ctx(), &view, &mut memo, 0, 2, &now, NowGate::ExactlyNow).unwrap();
    assert!(decision.candidate.is_none());
    // the oracle's `at_now_only`: `SplitCandidateDecisionV1(None)`, not a named refusal (the time is AFTER `now`, which only the front's own gate would carry on with)
    assert!(decision.effects.is_empty());
    assert_eq!(world.budget.exact_position_hydrations, 0, "no place is asked of a time that is not now");
    assert_eq!(memo.len(), (0, 1));
    // the gate of the front lets the same question through to the place
    let through = evaluate_split_candidate_gated(&mut world.ctx(), &view, &mut memo, 0, 2, &now, NowGate::NotBefore).unwrap();
    assert_eq!(through.effects[0].reason, CandidateRefusal::FilterPointOutsideFront);
    assert_eq!(world.budget.exact_position_hydrations, 1);
    // a time BEFORE `now` is refused by name by both gates
    let after = rational_time(100, 1);
    let past = evaluate_split_candidate_gated(&mut world.ctx(), &view, &mut memo, 0, 2, &after, NowGate::ExactlyNow).unwrap();
    assert!(past.candidate.is_none());
    assert_eq!(past.effects[0].reason, CandidateRefusal::FilterEventInThePast);
}

#[test]
fn a_vertex_born_after_the_event_refuses_it_as_the_past_without_asking_for_a_place() {
    let mut world = World::new();
    let view = triangle_view(rational_time(1000, 1));
    let mut memo = PositionMemo::new(true);
    let decision = evaluate_split_candidate(&mut world.ctx(), &view, &mut memo, 0, 2, &rational_time(0, 1)).unwrap();
    assert_eq!(decision.effects[0].reason, CandidateRefusal::FilterEventInThePast);
    assert_eq!(world.budget.exact_position_hydrations, 0);
    assert_eq!(memo.len(), (0, 1));
}

#[test]
fn a_view_whose_memory_does_not_serve_remembers_nothing() {
    let mut world = World::new();
    let view = triangle_view(rational_time(0, 1));
    let mut memo = PositionMemo::new(false);
    evaluate_split_candidate(&mut world.ctx(), &view, &mut memo, 0, 2, &rational_time(0, 1)).unwrap();
    assert_eq!(memo.len(), (0, 0));
}
