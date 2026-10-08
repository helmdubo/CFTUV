//! The builder, its loop and the planning layer on small worlds whose answers were read off the Python oracle (the differential against the oracle itself is
//! `tests/test_native_skeleton_builder.py`, over the corpora, the field polygons and the kernel tests).

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::ExactCtx;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SignCounts;
use cftuv_skeleton::builder::{level_budget, Builder, BuilderOptions, Counter, Counters, RunEnd, Step, Transaction};
use cftuv_skeleton::candidate::CandidateRefusal;
use cftuv_skeleton::error::SkelResult;
use cftuv_skeleton::plans::{plan_superlevel_components, Resolution};
use cftuv_skeleton::polygon::{unit_speed_squared, FanSupport, Loop, Polygon, VertexFan};
use cftuv_skeleton::queue::{CandidateEvent, EventKind};
use cftuv_skeleton::skeleton::SkeletonOutcome;
use cftuv_skeleton::snapshot::collect_superlevel_snapshot;
use cftuv_skeleton::superlevel::{transaction_prefix, Prefix};
use cftuv_skeleton::time::times_are_equal;

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

fn polygon(points: &[(i64, i64)]) -> Polygon {
    let speeds = (0..points.len()).map(|index| unit_speed_squared(points[index], points[(index + 1) % points.len()])).collect();
    Polygon::new(vec![Loop { points: points.to_vec(), speeds }], Vec::new()).unwrap()
}

fn square() -> Polygon {
    polygon(&[(0, 0), (4, 0), (4, 4), (0, 4)])
}

fn ell() -> Polygon {
    polygon(&[(0, 0), (8, 0), (8, 4), (4, 4), (4, 8), (0, 8)])
}

struct Stop;

impl Transaction for Stop {
    fn apply(&mut self, _: &mut Builder, _: &mut ExactCtx<'_>, _: &[CandidateEvent]) -> SkelResult<Step> {
        Ok(Step::Stop)
    }
}

/// A transaction that kills the vertices of the packet and emits one node per event: enough to drive the loop to its end.
struct Kill;

impl Transaction for Kill {
    fn apply(&mut self, builder: &mut Builder, _: &mut ExactCtx<'_>, level: &[CandidateEvent]) -> SkelResult<Step> {
        for event in level {
            let vertex = builder.vertex_at(event.vertex)?.clone();
            let mut keys = vec![builder.edge_at(vertex.prev_edge)?.span.to_vec(), builder.edge_at(vertex.next_edge)?.span.to_vec()];
            keys.sort();
            builder.emit(event.kind, event, keys, &[event.vertex, event.peer]);
            for ident in [event.vertex, event.peer] {
                if ident >= 0 {
                    builder.vertices[ident as usize].alive = false;
                }
            }
        }
        Ok(Step::Continue)
    }
}

#[test]
fn a_square_seeds_four_vertices_four_edges_and_one_edge_candidate_each() {
    let mut world = World::new();
    let builder = Builder::new(&mut world.ctx(), square(), BuilderOptions::default()).unwrap();
    assert_eq!((builder.vertices.len(), builder.edges.len(), builder.queue.len()), (4, 4, 4));
    assert!(builder.vertices.iter().all(|vertex| !vertex.reflex && vertex.alive && vertex.sliding.is_none()));
    assert!(builder.traces.is_empty() && builder.unindexed_reflex.is_empty());
    assert_eq!(builder.line_order.len(), 4);
    assert_eq!(builder.edge_start.len(), 4);
    assert_eq!(builder.counters.get(Counter::SplitCandidatesExamined), 0);
}

#[test]
fn the_ell_has_one_reflex_vertex_with_a_trace_and_split_candidates_from_it() {
    let mut world = World::new();
    let builder = Builder::new(&mut world.ctx(), ell(), BuilderOptions::default()).unwrap();
    let reflex: Vec<i64> = builder.vertices.iter().filter(|vertex| vertex.reflex).map(|vertex| vertex.ident).collect();
    assert_eq!(reflex.len(), 1);
    assert!(builder.traces.contains_key(&reflex[0]) || builder.unindexed_reflex.contains(&reflex[0]));
    assert!(builder.counters.get(Counter::SplitCandidatesExamined) > 0);
    assert!(builder.queue.len() >= 6);
}

#[test]
fn the_loop_stops_at_the_first_transaction_with_the_whole_level_and_writes_the_level_counter() {
    let mut world = World::new();
    let options = BuilderOptions { budgeted: true, ..BuilderOptions::default() };
    let mut builder = Builder::new(&mut world.ctx(), square(), options).unwrap();
    let future = builder.queue.peek_time().unwrap().clone();
    let mut memo = std::mem::take(&mut builder.memo);
    cftuv_skeleton::view::position(&mut world.ctx(), &builder, &mut memo, 0, &future).unwrap();
    builder.memo = memo;
    let seeded_memory = builder.memo.len();
    assert!(seeded_memory.0 > 0);
    let limit = level_budget(&builder.polygon);
    let RunEnd::Stopped { levels, level } = builder.run(&mut world.ctx(), limit, &mut Stop).unwrap() else {
        panic!("the loop must stop at the transaction");
    };
    assert_eq!((levels, level.len()), (1, 4));
    assert_eq!(world.budget.superlevel, "1");
    assert!(builder.queue.is_empty());
    // Будущие места, оплаченные при посеве кандидатов, переживают вход в уровень.
    assert_eq!(builder.memo.len(), seeded_memory);
    assert!(times_are_equal(&builder.now, &level[0].time));
    let hydrations = world.budget.exact_position_hydrations;
    let mut memo = std::mem::take(&mut builder.memo);
    let place = cftuv_skeleton::view::position(&mut world.ctx(), &builder, &mut memo, 0, &builder.now).unwrap();
    assert!(place.is_some());
    assert_eq!(world.budget.exact_position_hydrations, hydrations, "место кандидата повторно не оплачивается на своём уровне");
    builder.memo = memo;
}

#[test]
fn a_transaction_that_kills_everything_ends_the_run_exact_with_the_accumulated_nodes() {
    let mut world = World::new();
    let mut builder = Builder::new(&mut world.ctx(), square(), BuilderOptions::default()).unwrap();
    let limit = level_budget(&builder.polygon);
    let RunEnd::Finished(skeleton) = builder.run(&mut world.ctx(), limit, &mut Kill).unwrap() else {
        panic!("the loop must finish");
    };
    assert_eq!(skeleton.outcome, SkeletonOutcome::Exact);
    assert_eq!(skeleton.levels, 1);
    // four edge nodes at one place share their participants: one MULTIWAY node
    assert_eq!(skeleton.nodes.len(), 1);
    assert_eq!(skeleton.nodes[0].kind, EventKind::Multiway);
    let names: Vec<&str> = skeleton.counters.iter().map(|(name, _)| name.as_str()).collect();
    let mut sorted = names.clone();
    sorted.sort_unstable();
    assert_eq!(names, sorted, "the counters are sorted by name");
    assert!(names.contains(&"motorcycle_traces") && names.contains(&"refused_filter_solo_vertex"));
    assert_eq!(skeleton.counters.iter().find(|(name, _)| name == "same_time_residual_after_level").map(|(_, value)| *value), Some(0));
}

#[test]
fn a_level_limit_below_the_first_level_is_the_named_outcome() {
    let mut world = World::new();
    let mut builder = Builder::new(&mut world.ctx(), square(), BuilderOptions::default()).unwrap();
    let RunEnd::Finished(skeleton) = builder.run(&mut world.ctx(), 0, &mut Kill).unwrap() else {
        panic!("finished");
    };
    assert_eq!(skeleton.outcome, SkeletonOutcome::LevelBudgetExhausted);
    assert_eq!(skeleton.levels, 0);
}

#[test]
fn the_level_budget_counts_the_supports_of_a_fan_as_vertices_and_as_reflex_vertices() {
    let plain = square();
    assert_eq!(level_budget(&plain), 4 * 4 + 16);
    let fanned = Polygon::new(
        plain.loops.clone(),
        vec![VertexFan { point: (0, 0), supports: vec![FanSupport { normal: (1, 1), speed: Rat::one() }, FanSupport { normal: (1, 2), speed: Rat::one() }] }],
    )
    .unwrap();
    assert_eq!(level_budget(&fanned), 4 * 6 + 4 * 6 * 2 + 16);
}

#[test]
fn the_snapshot_of_the_first_packet_of_a_square_is_one_component_that_all_four_vertices_die_in() {
    let mut world = World::new();
    let mut builder = Builder::new(&mut world.ctx(), square(), BuilderOptions::default()).unwrap();
    let RunEnd::Stopped { level, .. } = builder.run(&mut world.ctx(), 100, &mut Stop).unwrap() else {
        panic!("stopped");
    };
    let snapshot = collect_superlevel_snapshot(&mut world.ctx(), &mut builder, &level).unwrap();
    assert_eq!((snapshot.incidents.len(), snapshot.vertices.len(), snapshot.stale_candidates), (4, 4, 0));
    assert!(snapshot.duplicate_live_owner_edge_ids.is_empty() && snapshot.unsupported.is_empty());
    let plans = plan_superlevel_components(&mut world.ctx(), &snapshot).unwrap();
    assert_eq!(plans.len(), 1);
    assert_eq!(plans[0].resolution, Resolution::Edge);
    assert_eq!(plans[0].dead_vertex_ids, vec![0, 1, 2, 3]);
    assert_eq!((plans[0].edge_contacts.len(), plans[0].closed_chain_count), (1, 1));
    assert!(plans[0].births.is_empty());
}

#[test]
fn the_head_of_the_transaction_hands_a_live_packet_to_the_closure_with_the_oracles_budget() {
    let mut world = World::new();
    let mut builder = Builder::new(&mut world.ctx(), square(), BuilderOptions::default()).unwrap();
    let RunEnd::Stopped { level, .. } = builder.run(&mut world.ctx(), 100, &mut Stop).unwrap() else {
        panic!("stopped");
    };
    match transaction_prefix(&mut world.ctx(), &mut builder, &level).unwrap() {
        Prefix::Continue { snapshot, budget } => assert_eq!(budget, (2 * 4 + 4).max(8) + 0 * snapshot.incidents.len() as i64),
        Prefix::Done => panic!("a live packet continues"),
    }
    // a packet whose only event is stale changes nothing but the counter
    let mut stale = level.clone();
    stale.truncate(1);
    builder.vertices[stale[0].vertex as usize].alive = false;
    assert!(matches!(transaction_prefix(&mut world.ctx(), &mut builder, &stale).unwrap(), Prefix::Done));
    assert_eq!(builder.counters.get(Counter::DiscardedStaleCandidates), 1);
}

#[test]
fn the_counters_round_trip_and_an_unknown_name_is_the_oracles_key_error() {
    let mut counters = Counters::default();
    counters.bump(Counter::Peaks, 3);
    counters.bump_refusal(CandidateRefusal::FilterEdgeIsOwn);
    counters.bump_reason("SOME_REASON");
    counters.add_named("split_candidates_beyond_trace", 2).unwrap();
    assert!(counters.add_named("no_such_counter", 1).is_err());
    let mut copy = Counters::default();
    copy.restore(&counters.sorted()).unwrap();
    assert_eq!(copy.sorted(), counters.sorted());
    assert!(counters.sorted().iter().any(|(name, value)| name == "superlevel_unresolvable_reason::SOME_REASON" && *value == 1));
}
