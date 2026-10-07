//! The whole operation (`transaction::build_skeleton`) on small worlds: outcome, nodes, levels and the number of signs the oracle paid for the same polygon.
//!
//! The numbers are the Python oracle's (`kernel/src/cftuv_envelope/wavefront/skeleton.py::build_skeleton`, cold memory, no budget), taken with the script that prints
//! `outcome, nodes, levels, SIGN_COUNTS`; the differential against the oracle itself over the field and synthetic corpora is `tests/test_native_skeleton_whole.py`.

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::ExactCtx;
use cftuv_core::products::ProductMemo;
use cftuv_core::sqrt_sum::SignCounts;
use cftuv_skeleton::builder::{level_budget, BuilderOptions};
use cftuv_skeleton::polygon::{unit_speed_squared, Loop, Polygon};
use cftuv_skeleton::proof::ProofStatus;
use cftuv_skeleton::queue::EventKind;
use cftuv_skeleton::skeleton::{Skeleton, SkeletonOutcome};
use cftuv_skeleton::transaction::build_skeleton;

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

    fn build(&mut self, polygon: Polygon) -> Skeleton {
        self.build_with(polygon, BuilderOptions::default())
    }

    fn build_with(&mut self, polygon: Polygon, options: BuilderOptions) -> Skeleton {
        let limit = level_budget(&polygon);
        let mut ctx = ExactCtx { memory: &mut self.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.products };
        build_skeleton(&mut ctx, polygon, options, limit).expect("a polygon of the corpus builds")
    }
}

fn each(points: &[(i64, i64)]) -> Loop {
    let speeds = (0..points.len()).map(|index| unit_speed_squared(points[index], points[(index + 1) % points.len()])).collect();
    Loop { points: points.to_vec(), speeds }
}

fn polygon(points: &[(i64, i64)]) -> Polygon {
    Polygon::new(vec![each(points)], Vec::new()).unwrap()
}

fn kinds(skeleton: &Skeleton) -> Vec<EventKind> {
    skeleton.nodes.iter().map(|node| node.kind).collect()
}

#[test]
fn a_square_is_one_edge_node_and_costs_what_the_oracle_pays() {
    let mut world = World::new();
    let skeleton = world.build(polygon(&[(0, 0), (4, 0), (4, 4), (0, 4)]));
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels, skeleton.proof_status, skeleton.proof_obligations.len()), (SkeletonOutcome::Exact, 1, 1, ProofStatus::Complete, 0));
    assert_eq!(kinds(&skeleton), vec![EventKind::Edge]);
    assert_eq!(world.counts.as_array(), [29, 9, 20, 0, 0]);
}

#[test]
fn a_rectangle_is_two_edge_nodes() {
    let mut world = World::new();
    let skeleton = world.build(polygon(&[(0, 0), (8, 0), (8, 4), (0, 4)]));
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels, skeleton.proof_obligations.len()), (SkeletonOutcome::Exact, 2, 2, 2));
    assert_eq!(world.counts.as_array(), [35, 6, 29, 0, 0]);
}

#[test]
fn an_ell_splits_once_at_its_reflex_corner() {
    let mut world = World::new();
    let skeleton = world.build(polygon(&[(0, 0), (8, 0), (8, 4), (4, 4), (4, 8), (0, 8)]));
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels, skeleton.proof_obligations.len()), (SkeletonOutcome::Exact, 3, 2, 4));
    assert_eq!(kinds(&skeleton), vec![EventKind::Edge, EventKind::Edge, EventKind::Split]);
    assert_eq!(world.counts.as_array(), [84, 28, 56, 0, 0]);
}

#[test]
fn a_cross_meets_its_four_reflex_corners_in_one_level() {
    let mut world = World::new();
    let skeleton = world.build(polygon(&[(4, 0), (8, 0), (8, 4), (12, 4), (12, 8), (8, 8), (8, 12), (4, 12), (4, 8), (0, 8), (0, 4), (4, 4)]));
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels, skeleton.proof_obligations.len()), (SkeletonOutcome::Exact, 5, 1, 16));
    assert_eq!(skeleton.proof_status, ProofStatus::Complete);
    assert_eq!(world.counts.as_array(), [525, 243, 282, 0, 0]);
}

#[test]
fn a_square_with_two_holes_is_nine_nodes_in_three_levels() {
    let mut world = World::new();
    let outer = each(&[(0, 0), (20, 0), (20, 20), (0, 20)]);
    let first = each(&[(4, 4), (4, 8), (8, 8), (8, 4)]);
    let second = each(&[(12, 12), (12, 16), (16, 16), (16, 12)]);
    let skeleton = world.build(Polygon::new(vec![outer, first, second], Vec::new()).unwrap());
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels, skeleton.proof_obligations.len()), (SkeletonOutcome::Exact, 9, 3, 8));
    assert_eq!(world.counts.as_array(), [853, 234, 619, 0, 0]);
}

#[test]
fn a_run_is_a_function_of_its_polygon_and_the_memory_it_started_with() {
    let points = [(0, 0), (8, 0), (8, 4), (4, 4), (4, 8), (0, 8)];
    let mut cold = World::new();
    let first = cold.build(polygon(&points));
    let mut warm = World::new();
    warm.build(polygon(&points));
    let before = warm.counts.as_array();
    let second = warm.build(polygon(&points));
    // the same answer on warm memory; the signs are paid again, to the digit (the memory of factorizations never changes a sign count, only the budget)
    assert_eq!(first.nodes.len(), second.nodes.len());
    assert_eq!(first.counters, second.counters);
    let paid: Vec<u64> = warm.counts.as_array().iter().zip(before).map(|(after, start)| after - start).collect();
    assert_eq!(paid, cold.counts.as_array().to_vec());
}

fn exhaustive() -> BuilderOptions {
    BuilderOptions { exhaustive: true, ..BuilderOptions::default() }
}

fn counter(skeleton: &Skeleton, name: &str) -> i64 {
    skeleton.counters.iter().find(|(found, _)| found == name).map_or(0, |(_, value)| *value)
}

#[test]
fn the_exhaustive_search_builds_the_same_skeleton_at_the_price_of_the_oracles_exhaustive_search() {
    // `SplitSearch.EXHAUSTIVE` of the oracle: no graph, no index; every reflex vertex against every edge (the counters tell it)
    let mut world = World::new();
    let skeleton = world.build_with(polygon(&[(0, 0), (8, 0), (8, 4), (4, 4), (4, 8), (0, 8)]), exhaustive());
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels, skeleton.proof_obligations.len()), (SkeletonOutcome::Exact, 3, 2, 4));
    assert_eq!((counter(&skeleton, "split_search_exhaustive_vertices"), counter(&skeleton, "split_candidates_examined"), counter(&skeleton, "motorcycle_traces")), (1, 4, 0));
    assert_eq!(world.counts.as_array(), [64, 22, 42, 0, 0]);

    let mut world = World::new();
    let skeleton = world.build_with(polygon(&[(4, 0), (8, 0), (8, 4), (12, 4), (12, 8), (8, 8), (8, 12), (4, 12), (4, 8), (0, 8), (0, 4), (4, 4)]), exhaustive());
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels), (SkeletonOutcome::Exact, 5, 1));
    assert_eq!((counter(&skeleton, "split_search_exhaustive_vertices"), counter(&skeleton, "split_candidates_examined")), (4, 40));
    assert_eq!(world.counts.as_array(), [299, 167, 132, 0, 0]);

    let mut world = World::new();
    let outer = each(&[(0, 0), (20, 0), (20, 20), (0, 20)]);
    let first = each(&[(4, 4), (4, 8), (8, 8), (8, 4)]);
    let second = each(&[(12, 12), (12, 16), (16, 16), (16, 12)]);
    let skeleton = world.build_with(Polygon::new(vec![outer, first, second], Vec::new()).unwrap(), exhaustive());
    assert_eq!((skeleton.outcome, skeleton.nodes.len(), skeleton.levels), (SkeletonOutcome::Exact, 9, 3));
    assert_eq!((counter(&skeleton, "split_search_exhaustive_vertices"), counter(&skeleton, "split_candidates_examined")), (8, 80));
    assert_eq!(world.counts.as_array(), [638, 189, 449, 0, 0]);
}
