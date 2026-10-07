//! The symbolic closure on small worlds: the first packet of a polygon, driven to the closure through the head of the transaction, and the pieces of the overlay on it. The
//! differential against the oracle itself is `tests/test_native_skeleton_closure.py` (over the corpora, the field polygons, the contacts made for the generations, the spoiled
//! overlays, both interpreters); these tests hold what a change of the port must not move without the oracle in the room.

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SignCounts;
use cftuv_skeleton::builder::{Builder, BuilderOptions, RunEnd, Step, Transaction};
use cftuv_skeleton::component::{apply_component_deltas, normalize_dead_component, overlay_signature, point_from_key, SignatureMemo};
use cftuv_skeleton::coordinator::{closure_stage, plan_symbolic_superlevel_closure, SymbolicSuperlevelClosure};
use cftuv_skeleton::error::SkelResult;
use cftuv_skeleton::overlay::{refreshed_span_bindings, OverlayView};
use cftuv_skeleton::polygon::{unit_speed_squared, Loop, Polygon};
use cftuv_skeleton::pyval::Val;
use cftuv_skeleton::queue::CandidateEvent;
use cftuv_skeleton::snapshot::Snapshot;
use cftuv_skeleton::superlevel::{transaction_prefix, Prefix};
use cftuv_skeleton::view::CandidateView;

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

struct Stop;

impl Transaction for Stop {
    fn apply(&mut self, _: &mut Builder, _: &mut ExactCtx<'_>, _: &[CandidateEvent]) -> SkelResult<Step> {
        Ok(Step::Stop)
    }
}

/// The builder of a polygon stopped at its first packet, with the frozen snapshot and the budget of the closure.
fn first_packet(world: &mut World, polygon: Polygon) -> (Builder, Snapshot, i64) {
    let mut builder = Builder::new(&mut world.ctx(), polygon, BuilderOptions::default()).unwrap();
    let RunEnd::Stopped { level, .. } = builder.run(&mut world.ctx(), 10_000, &mut Stop).unwrap() else {
        panic!("the loop must stop at the first packet");
    };
    match transaction_prefix(&mut world.ctx(), &mut builder, &level).unwrap() {
        Prefix::Continue { snapshot, budget } => (builder, *snapshot, budget),
        Prefix::Done => panic!("the first packet of the polygon has a live candidate"),
    }
}

fn closure_of(polygon: Polygon) -> (World, Builder, Snapshot, SymbolicSuperlevelClosure) {
    let mut world = World::new();
    let (mut builder, snapshot, budget) = first_packet(&mut world, polygon);
    let closure = plan_symbolic_superlevel_closure(&mut world.ctx(), &mut builder, &snapshot, budget, budget).unwrap();
    (world, builder, snapshot, closure)
}

fn ell() -> Polygon {
    polygon(&[(0, 0), (8, 0), (8, 4), (4, 4), (4, 8), (0, 8)])
}

fn square() -> Polygon {
    polygon(&[(0, 0), (4, 0), (4, 4), (0, 4)])
}

#[test]
fn the_closure_of_the_first_packet_of_a_square_is_resolved_with_one_overlay_and_two_signatures() {
    let (_world, _builder, snapshot, closure) = closure_of(square());
    assert_eq!(closure.unresolved_reason, None);
    let overlay = closure.overlay.as_ref().expect("a resolved closure has an overlay");
    assert_eq!(snapshot.incidents.len(), 4);
    // the four vertices of the square die in one contact, and the leaves of the four edges stay
    assert!(overlay.vertices.values().all(|vertex| !vertex.alive));
    assert_eq!(overlay.spans.len(), 4);
    // the signatures are the split overlay, one per generation, and the closed one
    let junction = closure.junction.as_ref().expect("a resolved closure has a junction fixed point");
    assert_eq!(closure.signatures.len(), 2 + junction.generations.len());
    assert!(closure.canonical_batch_count > junction.generations.len() as i64);
}

#[test]
fn the_closure_is_a_function_of_the_front_it_pays_the_same_signs_and_answers_the_same_text_twice() {
    let first = {
        let (world, _builder, _snapshot, closure) = closure_of(ell());
        (world.counts, cftuv_skeleton::coordinator::closure_val(&closure).repr().to_string())
    };
    let second = {
        let (world, _builder, _snapshot, closure) = closure_of(ell());
        (world.counts, cftuv_skeleton::coordinator::closure_val(&closure).repr().to_string())
    };
    assert_eq!(first.1, second.1);
    assert_eq!(first.0.as_array(), second.0.as_array());
}

#[test]
fn the_replay_of_the_generations_is_in_the_cost_a_second_closure_pays_what_the_first_pays_beyond_its_memory() {
    let mut world = World::new();
    let (mut builder, snapshot, budget) = first_packet(&mut world, ell());
    let before = world.counts.as_array();
    let first = plan_symbolic_superlevel_closure(&mut world.ctx(), &mut builder, &snapshot, budget, budget).unwrap();
    let paid_first: Vec<u64> = world.counts.as_array().iter().zip(before).map(|(after, start)| after - start).collect();
    let middle = world.counts.as_array();
    let second = plan_symbolic_superlevel_closure(&mut world.ctx(), &mut builder, &snapshot, budget, budget).unwrap();
    let paid_second: Vec<u64> = world.counts.as_array().iter().zip(middle).map(|(after, start)| after - start).collect();
    // the memory of places, filled by the first, answers the second: it never pays more signs than the first did
    assert!(paid_second.iter().zip(&paid_first).all(|(second, first)| second <= first), "{paid_first:?} against {paid_second:?}");
    assert_eq!(cftuv_skeleton::coordinator::closure_val(&first).repr(), cftuv_skeleton::coordinator::closure_val(&second).repr());
}

#[test]
fn an_overlay_binds_every_leaf_to_one_start_and_one_end_and_its_view_answers_the_ends_by_slot() {
    let (_world, builder, _snapshot, closure) = closure_of(ell());
    let overlay = closure.overlay.expect("a resolved closure has an overlay");
    let (bindings, reason) = refreshed_span_bindings(&overlay);
    assert_eq!(reason, None);
    let bindings = bindings.expect("the bindings of a closed overlay refresh");
    assert_eq!(bindings.len(), overlay.spans.len());
    let view = OverlayView::new(&builder, &overlay);
    for (leaf, binding) in overlay.spans.iter() {
        let slot = view.span_ref(leaf).unwrap();
        let state = view.span_state(slot).unwrap();
        assert_eq!(state.start_vertex.is_some(), binding.start.is_some());
        assert_eq!(state.end_vertex.is_some(), binding.end.is_some());
        assert_eq!(state.line.ident, builder.edge_at(binding.physical_edge_id).unwrap().line.ident, "the span of a leaf carries the line object of its edge (the identity the memory of times is keyed by)");
    }
}

#[test]
fn a_junction_that_is_not_an_existing_port_gets_a_new_identity_of_its_sliding_projection_on_every_question() {
    let (_world, builder, _snapshot, closure) = closure_of(ell());
    let overlay = closure.overlay.unwrap();
    // an overlay vertex that was born (not an existing port) has no sliding value of its own; whatever its projection is, asking twice must not give the same identity
    let view = OverlayView::new(&builder, &overlay);
    for (reference, _) in overlay.vertices.iter().filter(|(reference, _)| !reference.is_existing()) {
        let slot = view.vertex_ref(reference).unwrap();
        let (first, second) = (view.vertex_state(slot).unwrap().sliding.map(|sliding| sliding.ident), view.vertex_state(slot).unwrap().sliding.map(|sliding| sliding.ident));
        assert_eq!(first.is_some(), second.is_some());
        if let (Some(left), Some(right)) = (first, second) {
            assert_ne!(left, right);
        }
    }
}

#[test]
fn a_point_key_makes_its_place_back_with_the_python_type_of_every_coefficient() {
    let whole = Val::big(IBig::from(3));
    let fraction = Val::frac(Rat::new(IBig::from(1), IBig::from(2)).unwrap());
    let key = Val::tuple(vec![
        Val::tuple(vec![Val::tuple(vec![Val::int(1), whole.clone()]), Val::tuple(vec![Val::int(2), fraction.clone()])]),
        Val::tuple(vec![Val::tuple(vec![Val::int(1), Val::frac(Rat::from_int(IBig::from(3)))])]),
    ]);
    let point = point_from_key(&key).unwrap();
    assert!(point.x.terms()[0].coef.is_py_int() && !point.x.terms()[1].coef.is_py_int());
    assert!(!point.y.terms()[0].coef.is_py_int(), "a Fraction that is whole stays a Fraction");
    assert_eq!(Val::terms_of(&point.x).repr().to_string(), "((1, 3), (2, Fraction(1, 2)))");
    let broken = Val::tuple(vec![Val::tuple(vec![Val::tuple(vec![Val::int(2), whole]), Val::tuple(vec![Val::int(1), fraction])]), Val::tuple(Vec::new())]);
    assert!(point_from_key(&broken).is_err(), "terms out of order are not a canonical sum");
}

#[test]
fn the_dead_junctions_of_a_closed_overlay_pair_their_boundary_arms_and_the_births_are_applied_as_one_generation() {
    let (_world, _builder, _snapshot, closure) = closure_of(ell());
    let overlay = closure.overlay.expect("a resolved closure has an overlay");
    let alive: Vec<_> = overlay.vertices.values().filter(|vertex| vertex.alive).collect();
    if alive.len() < 3 {
        return;
    }
    // two neighbours of a ring die together: one incoming and one outgoing arm remain, so one junction is born between them
    let first = alive[0];
    let Some(second) = first.next.as_ref().and_then(|next| overlay.vertices.get(next)).filter(|peer| peer.alive) else {
        return;
    };
    let dead = vec![first.reference.clone(), second.reference.clone()];
    let point_key = Val::tuple(vec![Val::tuple(Vec::new()), Val::tuple(Vec::new())]);
    let (delta, reason) = normalize_dead_component(&overlay, &[Val::str("k")], &dead, &point_key, "JUNCTION", "STALE", "AMBIGUOUS");
    let Some(delta) = delta else {
        assert!(reason.is_some());
        return;
    };
    assert_eq!(delta.rewires.len(), usize::from(delta.birth_ref.is_some()));
    let (applied, why) = apply_component_deltas(&overlay, std::slice::from_ref(&delta), "OVERLAP").unwrap();
    match applied {
        Some(result) => {
            assert_eq!(result.vertices.len(), overlay.vertices.len() + delta.rewires.len());
            assert!(result.changed.len() >= overlay.changed.len());
            let signature = overlay_signature(&result, &mut SignatureMemo::new()).unwrap();
            assert_eq!(signature.items().map(<[Val]>::len), Some(3));
        }
        None => assert!(why.is_some()),
    }
    // the same delta twice kills the same junctions twice: a named overlap, never a silent last-wins
    let (twice, overlap) = apply_component_deltas(&overlay, &[delta.clone(), delta], "OVERLAP").unwrap();
    assert!(twice.is_none());
    assert_eq!(overlap, Some("OVERLAP"));
}

#[test]
fn the_closure_stage_refuses_by_name_what_the_closure_cannot_resolve_and_records_it() {
    let mut world = World::new();
    let (mut builder, snapshot, budget) = first_packet(&mut world, ell());
    let counted = builder.counters.get(cftuv_skeleton::builder::Counter::SuperlevelUnresolvableComponents);
    let staged = closure_stage(&mut world.ctx(), &mut builder, &snapshot, budget).unwrap();
    if staged.is_some() {
        assert_eq!(builder.counters.get(cftuv_skeleton::builder::Counter::SuperlevelUnresolvableComponents), counted);
        assert!(builder.refusal.is_none());
    }
    // a packet with no incident has no closure to plan: the empty packet is a named reason of the closure itself
    let empty = Snapshot { incidents: Vec::new(), vertices: snapshot.vertices.clone(), unsupported: Vec::new(), stale_candidates: 0, duplicate_live_owner_edge_ids: Vec::new() };
    let refused = plan_symbolic_superlevel_closure(&mut world.ctx(), &mut builder, &empty, 8, 8).unwrap();
    assert_eq!(refused.unresolved_reason, Some("SYMBOLIC_SUPERLEVEL_EMPTY_PACKET"));
    assert!(refused.materialization.is_none() && refused.overlay.is_none() && refused.junction.is_none());
    let staged = closure_stage(&mut world.ctx(), &mut builder, &empty, 8).unwrap();
    assert!(staged.is_none());
    assert_eq!(builder.counters.get(cftuv_skeleton::builder::Counter::SuperlevelUnresolvableComponents), counted + 1);
    assert!(builder.counters.reasons().contains_key("superlevel_unresolvable_reason::SYMBOLIC_SUPERLEVEL_EMPTY_PACKET"));
}
