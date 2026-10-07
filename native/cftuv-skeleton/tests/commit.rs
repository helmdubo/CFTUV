//! The plan of the runtime commit on a spoiled closure: every named reason the oracle's `plan_symbolic_runtime_commit` returns before it writes anything, and the plan of the
//! unspoiled closure of the first packet of an ell. The reasons are those of `wavefront/symbolic_runtime_commit.py` (the validators `_validated_existing_vertices`,
//! `_validated_active_leaf_groups`, `_validated_birth_context`); the corpora reach the Q-08 reasons (`SYMBOLIC_POSTSTATE_*`) but no packet of theirs breaks one of the references, so
//! these tests hold what the differential cannot: that each reference is checked, in the oracle's order, and that a refusal writes nothing.

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::ExactCtx;
use cftuv_core::products::ProductMemo;
use cftuv_core::sqrt_sum::SignCounts;
use cftuv_skeleton::builder::{Builder, BuilderOptions, RunEnd, Step, Transaction};
use cftuv_skeleton::commit::{materialize_symbolic_runtime_commit, plan_symbolic_runtime_commit};
use cftuv_skeleton::coordinator::{plan_symbolic_superlevel_closure, SymbolicSuperlevelClosure};
use cftuv_skeleton::error::SkelResult;
use cftuv_skeleton::overlay::JRef;
use cftuv_skeleton::polygon::{unit_speed_squared, Loop, Polygon};
use cftuv_skeleton::pyval::Val;
use cftuv_skeleton::queue::CandidateEvent;
use cftuv_skeleton::snapshot::Snapshot;
use cftuv_skeleton::superlevel::{transaction_prefix, Prefix};

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

struct Stop;

impl Transaction for Stop {
    fn apply(&mut self, _: &mut Builder, _: &mut ExactCtx<'_>, _: &[CandidateEvent]) -> SkelResult<Step> {
        Ok(Step::Stop)
    }
}

fn each(points: &[(i64, i64)]) -> Loop {
    let speeds = (0..points.len()).map(|index| unit_speed_squared(points[index], points[(index + 1) % points.len()])).collect();
    Loop { points: points.to_vec(), speeds }
}

fn ell() -> Polygon {
    Polygon::new(vec![each(&[(0, 0), (8, 0), (8, 4), (4, 4), (4, 8), (0, 8)])], Vec::new()).unwrap()
}

/// A square with two square holes: the first packet splits the spans of the outer edges and leaves existing junctions of the holes in the overlay.
fn holes() -> Polygon {
    Polygon::new(vec![each(&[(0, 0), (20, 0), (20, 20), (0, 20)]), each(&[(4, 4), (4, 8), (8, 8), (8, 4)]), each(&[(12, 12), (12, 16), (16, 16), (16, 12)])], Vec::new()).unwrap()
}

/// The builder stopped at the first packet of the ell, its frozen snapshot, and the closure of that packet.
fn first_closure() -> (World, Builder, Snapshot, SymbolicSuperlevelClosure) {
    closure_of(ell())
}

fn closure_of(polygon: Polygon) -> (World, Builder, Snapshot, SymbolicSuperlevelClosure) {
    let mut world = World::new();
    let mut builder = Builder::new(&mut world.ctx(), polygon, BuilderOptions::default()).unwrap();
    let RunEnd::Stopped { level, .. } = builder.run(&mut world.ctx(), 10_000, &mut Stop).unwrap() else {
        panic!("the loop must stop at the first packet");
    };
    let (snapshot, budget) = match transaction_prefix(&mut world.ctx(), &mut builder, &level).unwrap() {
        Prefix::Continue { snapshot, budget } => (*snapshot, budget),
        Prefix::Done => panic!("the first packet of the ell has a live candidate"),
    };
    let closure = plan_symbolic_superlevel_closure(&mut world.ctx(), &mut builder, &snapshot, budget, budget).unwrap();
    assert_eq!(closure.unresolved_reason, None);
    (world, builder, snapshot, closure)
}

fn reason_of(world: &mut World, builder: &mut Builder, snapshot: &Snapshot, closure: &SymbolicSuperlevelClosure) -> Option<&'static str> {
    let (plan, reason) = plan_symbolic_runtime_commit(&mut world.ctx(), builder, snapshot, closure).unwrap();
    assert_eq!(plan.is_none(), reason.is_some(), "a plan or a reason, never both and never neither");
    reason
}

#[test]
fn the_closure_of_the_first_packet_of_an_ell_makes_a_plan_that_the_materialization_writes() {
    let (mut world, mut builder, snapshot, closure) = first_closure();
    let (plan, reason) = plan_symbolic_runtime_commit(&mut world.ctx(), &mut builder, &snapshot, &closure).unwrap();
    assert_eq!(reason, None);
    let plan = plan.expect("the unspoiled closure is committed");
    assert!(!plan.active_leaves_by_edge.is_empty());
    let (edges, vertices) = (builder.edges.len(), builder.vertices.len());
    materialize_symbolic_runtime_commit(&mut world.ctx(), &mut builder, &snapshot, &plan).unwrap();
    assert!(builder.edges.len() >= edges && builder.vertices.len() >= vertices + plan.births.len());
    assert!(builder.future_only.is_none(), "the queue of the commit is the real queue again");
    for ident in &plan.dead_existing {
        assert!(!builder.vertices[*ident as usize].alive);
    }
}

#[test]
fn a_closure_without_a_final_overlay_or_with_a_reason_of_its_own_is_refused_by_that_name() {
    let (mut world, mut builder, snapshot, closure) = first_closure();
    let mut spoiled = closure.clone();
    spoiled.overlay = None;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_FINAL_OVERLAY_UNAVAILABLE"));
    let mut spoiled = closure.clone();
    spoiled.materialization = None;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_FINAL_OVERLAY_UNAVAILABLE"));
    let mut spoiled = closure;
    spoiled.materialization.as_mut().unwrap().unresolved_reason = Some("SYMBOLIC_SOMETHING_THE_PLANNER_NAMED");
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_SOMETHING_THE_PLANNER_NAMED"));
}

#[test]
fn an_existing_junction_without_a_runtime_id_or_with_the_id_of_another_is_refused() {
    let (mut world, mut builder, snapshot, closure) = closure_of(holes());
    let existing: Vec<JRef> = closure.overlay.as_ref().unwrap().vertices.values().filter(|vertex| vertex.reference.is_existing()).map(|vertex| vertex.reference.clone()).collect();
    assert!(existing.len() >= 2);
    let mut spoiled = closure.clone();
    spoiled.overlay.as_mut().unwrap().vertices.get_mut(&existing[0]).unwrap().runtime_id = None;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_EXISTING_REF_UNRESOLVABLE"));
    let mut spoiled = closure.clone();
    let first = spoiled.overlay.as_ref().unwrap().vertices.get(&existing[0]).unwrap().runtime_id;
    spoiled.overlay.as_mut().unwrap().vertices.get_mut(&existing[1]).unwrap().runtime_id = first;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_EXISTING_REF_UNRESOLVABLE"));
    let mut spoiled = closure;
    spoiled.overlay.as_mut().unwrap().vertices.get_mut(&existing[0]).unwrap().runtime_id = Some(10_000);
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_EXISTING_REF_UNRESOLVABLE"));
}

#[test]
fn a_front_that_moved_since_the_snapshot_is_refused_before_anything_is_written() {
    let (mut world, mut builder, snapshot, closure) = first_closure();
    let ident = snapshot.vertices.iter().find(|vertex| vertex.alive).unwrap().ident as usize;
    let (edges, vertices) = (builder.edges.len(), builder.vertices.len());
    builder.vertices[ident].next_edge += 1;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &closure), Some("SYMBOLIC_RUNTIME_FROZEN_PRESTATE_CHANGED"));
    assert_eq!((builder.edges.len(), builder.vertices.len()), (edges, vertices));
}

#[test]
fn the_neighbours_of_the_alive_junctions_must_be_alive_and_reciprocal_and_every_span_has_one_owner_and_one_binding() {
    let (mut world, mut builder, snapshot, closure) = first_closure();
    let alive: Vec<JRef> = closure.overlay.as_ref().unwrap().vertices.values().filter(|vertex| vertex.alive).map(|vertex| vertex.reference.clone()).collect();
    assert!(alive.len() >= 2, "the first packet of the ell leaves a ring to commit");
    // a neighbour that is not in the overlay
    let mut spoiled = closure.clone();
    spoiled.overlay.as_mut().unwrap().vertices.get_mut(&alive[0]).unwrap().next = Some(JRef::new("EXISTING", Val::str("nowhere")));
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_RECIPROCITY_UNRESOLVABLE"));
    // a junction that has no neighbour
    let mut spoiled = closure.clone();
    spoiled.overlay.as_mut().unwrap().vertices.get_mut(&alive[0]).unwrap().prev = None;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_RECIPROCITY_UNRESOLVABLE"));
    // two junctions that start the span of one leaf
    let mut spoiled = closure.clone();
    let leaf = spoiled.overlay.as_ref().unwrap().vertices.get(&alive[0]).unwrap().next_leaf.clone();
    spoiled.overlay.as_mut().unwrap().vertices.get_mut(&alive[1]).unwrap().next_leaf = leaf.clone();
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_SPAN_OWNER_AMBIGUOUS"));
    // a binding that names another start than the one the junctions give
    let mut spoiled = closure.clone();
    spoiled.overlay.as_mut().unwrap().spans.get_mut(&leaf).unwrap().start = None;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_SPAN_BINDING_UNRESOLVABLE"));
    // a binding to an edge the runtime does not have
    let mut spoiled = closure.clone();
    spoiled.overlay.as_mut().unwrap().spans.get_mut(&leaf).unwrap().physical_edge_id = 10_000;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_SPAN_BINDING_UNRESOLVABLE"));
    // a binding to another edge than the one the leaf was cut from: the source key of the edge is the authority
    let mut spoiled = closure;
    let bound = spoiled.overlay.as_ref().unwrap().spans.get(&leaf).unwrap().physical_edge_id;
    spoiled.overlay.as_mut().unwrap().spans.get_mut(&leaf).unwrap().physical_edge_id = (bound + 1) % builder.edges.len() as i64;
    assert_eq!(reason_of(&mut world, &mut builder, &snapshot, &spoiled), Some("SYMBOLIC_RUNTIME_EDGE_AUTHORITY_MISMATCH"));
}

#[test]
fn a_refused_plan_pays_nothing_and_writes_nothing() {
    let (mut world, mut builder, snapshot, closure) = first_closure();
    let mut spoiled = closure;
    spoiled.overlay = None;
    let (signs, edges, vertices, nodes) = (world.counts.as_array(), builder.edges.len(), builder.vertices.len(), builder.nodes.len());
    assert!(reason_of(&mut world, &mut builder, &snapshot, &spoiled).is_some());
    assert_eq!((world.counts.as_array(), builder.edges.len(), builder.vertices.len(), builder.nodes.len()), (signs, edges, vertices, nodes));
}
