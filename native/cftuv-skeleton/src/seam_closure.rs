//! The differential seams of the symbolic closure (test-only, WP-S5): the whole closure of a packet on the front the oracle has at its call, and its parts on the overlays and
//! materializations the oracle had at theirs. Opcodes 310..316, the wire of `seam.rs`; every answer is the `repr` TEXT of what the oracle's function returned (with the sets
//! and the dictionaries the oracle keeps in hash order brought to the canonical form of `pyval`), compared byte for byte, plus the growth of the memory of places.
//!
//! ```text
//! 310 PLAN_SYMBOLIC_CLOSURE        [state, options, snapshot, outer budget, junction budget]          -> [text, growth]
//! 311 BUILD_SYMBOLIC_OVERLAY       [state, options, vertices, materialization, include line ports]    -> [text, growth]
//! 312 DISCOVER_INTERIOR_CONTACTS   [state, options, overlay]                                          -> [text, growth]
//! 313 PLAN_MIXED_GENERATIONS       [state, options, overlay, budget, script | none]                   -> [text, growth]
//! 314 DISCOVER_JUNCTION_CONTACTS   [state, options, overlay]                                          -> [text, growth]
//! 315 APPLY_COMPONENT_DELTAS       [state, options, overlay, deltas, collision reason]                -> [text, growth]
//! 317 NORMALIZE_MIXED_GENERATION   [state, options, overlay, junction contacts, interior contacts]          -> [text, growth]  (normalize, then apply)
//! 316 CLOSURE_PART                 [state, options, op, ...]  0 with_line_ports [vertices, time]  1 build_f0_overlay [vertices, time]
//!                                                             2 initial_interior_contacts [snapshot]  3 overlay_signature [overlay]
//!
//! state    the state of `seam_builder` with the traces: the front, the memory of places, the prime universe, the traces
//! overlay  [vertices, spans, changed, time]
//!          vertex  [ref, prev | none, next | none, prev leaf, next leaf, birth, point | none, [sliding sum, identity] | none, [provenance keys], runtime id | none,
//!                   trace | none (absent) | [] (no crash) | [crash time], alive]
//!          span    [leaf, physical edge, start | none, end | none]     (refs, leaves and keys are repr nodes)
//! materialization  [plans, families]
//!          plan    [time, events, dead ids, births, wiring, rewrites, cuts, contacts]
//!          birth   [point key, prev occurrence, next occurrence, key, replaces]    wire  [key, [existing | none, birth key | none], [existing | none, birth key | none]]
//!          rewrite [vertex, prev occurrence, next occurrence]    cut  [edge, target occurrence, [segment occurrences]]    contact  [births, participants, [kinds]]
//!          family  [[segments], [births]]
//! script   [[junction contacts | none, junction reason | none, interior contacts | none, interior reason | none], ...]: the answers the discoveries of the round with that
//!          number give instead of the front's (none: the front answers); the oracle side makes them by monkeypatching its two discoveries
//! interior contact  [key, time, point, projection, leaf | none]
//! junction contact  [kind, key, [dead refs], [families], edge | none]    edge  [key, prev leaf, shared leaf, next leaf, span unproven, [participant keys]]
//! delta    [contact keys, dead refs, incoming | none, outgoing | none, birth ref | none, point key, [leaf resources], [rewires]]    port  [ref | none, leaf]    rewire  [port, port, ref]
//! ```

use std::rc::Rc;
use std::time::Instant;

use cftuv_core::codec::Value;
use cftuv_core::exact::ExactCtx;

use crate::builder::{Builder, SlidingValue};
use crate::closure::{FamilyNormalForm, Materialization};
use crate::component::{apply_component_deltas, overlay_signature, Delta, Port, SignatureMemo};
use crate::contacts::{discover_interior_split_contacts, discover_junction_contacts, initial_interior_contacts, ContactKind, EdgeContact, EdgeContactKey, EndpointKey, JunctionContact, SplitKey, SymSplitContact};
use crate::coordinator::{closure_val, plan_symbolic_superlevel_closure};
use crate::error::SkelResult;
use crate::generations::{apply_mixed_generation, fixed_point_val, normalize_mixed_generation, plan_mixed_generations_with, Discovery, Natural};
use crate::omap::OrderedMap;
use crate::overlay::{build_f0_overlay, build_symbolic_overlay, frozen_keys, overlay_val, with_line_ports, Binding, JRef, Leaf, Overlay, SymVertex, TraceInfo};
use crate::plans::{BoundaryBirth, ComponentPlan, PlanVal, Resolution, VertexReference};
use crate::pyval::Val;
use crate::seam::growth_value;
use crate::seam_builder::{builder_of, event_of, options_of, snapshot_of, val_of, vertex_snapshot_of};
use crate::seam_graph::{ints_of, keys_of, nanoseconds};
use crate::snapshot::VertexSnapshot;
use crate::time::EventTime;
use crate::wire::{bad, fixed, flag_of, i64_of, list, optional, point_of, str_of, str_value, sum_of, time_of, u64_of, Wire};

pub(crate) const SEAMS: &[(u16, &str)] = &[
    (310, "PLAN_SYMBOLIC_CLOSURE"),
    (311, "BUILD_SYMBOLIC_OVERLAY"),
    (312, "DISCOVER_INTERIOR_CONTACTS"),
    (313, "PLAN_MIXED_GENERATIONS"),
    (314, "DISCOVER_JUNCTION_CONTACTS"),
    (315, "APPLY_COMPONENT_DELTAS"),
    (316, "CLOSURE_PART"),
    (317, "NORMALIZE_MIXED_GENERATION"),
];

fn refusal(error: impl std::fmt::Debug) -> crate::wire::SeamError {
    bad(&format!("{error:?}"))
}

fn vals_of(value: &Value, what: &str) -> Wire<Vec<Val>> {
    list(value, what)?.iter().map(val_of).collect()
}

fn jref_of(value: &Value) -> Wire<JRef> {
    JRef::from_val(&val_of(value)?).map_err(refusal)
}

fn leaf_of(value: &Value) -> Wire<Leaf> {
    Leaf::from_val(&val_of(value)?).map_err(refusal)
}

fn opt_jref(value: &Value) -> Wire<Option<JRef>> {
    optional(value, jref_of)
}

fn port_of(value: &Value) -> Wire<Port> {
    let [reference, leaf] = fixed::<2>(value, "a port")?;
    Ok((opt_jref(reference)?, leaf_of(leaf)?))
}

// --------------------------------------------------------------------------
// the overlay
// --------------------------------------------------------------------------

fn vertex_of(value: &Value) -> Wire<SymVertex> {
    let [reference, prev, next, prev_leaf, next_leaf, birth, point, sliding, provenance, runtime_id, trace, alive] = fixed::<12>(value, "an overlay vertex")?;
    Ok(SymVertex {
        reference: jref_of(reference)?,
        prev: opt_jref(prev)?,
        next: opt_jref(next)?,
        prev_leaf: leaf_of(prev_leaf)?,
        next_leaf: leaf_of(next_leaf)?,
        birth: Rc::new(time_of(birth)?),
        point: optional(point, |found| Ok(Rc::new(point_of(found)?)))?,
        sliding: optional(sliding, |found| {
            let [sum, ident] = fixed::<2>(found, "a sliding projection")?;
            Ok(Rc::new(SlidingValue { value: sum_of(sum, "a sliding value")?.clone(), ident: u64_of(ident, "a sliding identity")? }))
        })?,
        provenance: frozen_keys(vals_of(provenance, "provenance")?),
        runtime_id: optional(runtime_id, |found| i64_of(found, "a runtime id"))?,
        trace: match trace {
            Value::None => TraceInfo::Absent,
            other => match list(other, "a trace authority")? {
                [] => TraceInfo::WithoutCrash,
                [crash] => TraceInfo::Crash(Rc::new(time_of(crash)?)),
                _ => return Err(bad("a trace authority")),
            },
        },
        alive: flag_of(alive, "alive")?,
    })
}

pub(crate) fn overlay_of(value: &Value) -> Wire<Overlay> {
    let [vertices, spans, changed, time] = fixed::<4>(value, "an overlay")?;
    let mut overlay_vertices: OrderedMap<JRef, SymVertex> = OrderedMap::new();
    for item in list(vertices, "overlay vertices")? {
        let vertex = vertex_of(item)?;
        overlay_vertices.insert(vertex.reference.clone(), vertex);
    }
    let mut overlay_spans: OrderedMap<Leaf, Binding> = OrderedMap::new();
    for item in list(spans, "overlay spans")? {
        let [leaf, edge, start, end] = fixed::<4>(item, "an overlay span")?;
        let leaf = leaf_of(leaf)?;
        overlay_spans.insert(leaf.clone(), Binding { leaf, physical_edge_id: i64_of(edge, "a physical edge")?, start: opt_jref(start)?, end: opt_jref(end)? });
    }
    let leaves = list(changed, "changed leaves")?.iter().map(leaf_of).collect::<Wire<Vec<_>>>()?;
    Ok(Overlay { vertices: overlay_vertices, spans: overlay_spans, changed: leaves.into_iter().collect(), time: Rc::new(time_of(time)?) })
}

// --------------------------------------------------------------------------
// the materialization
// --------------------------------------------------------------------------

fn birth_of(value: &Value) -> Wire<BoundaryBirth> {
    let [point_key, prev, next, key, replaces] = fixed::<5>(value, "a birth")?;
    Ok(BoundaryBirth { point_key: val_of(point_key)?, prev_occurrence: val_of(prev)?, next_occurrence: val_of(next)?, key: val_of(key)?, replaces: ints_of(replaces, "the load of a birth")? })
}

fn reference_of(value: &Value) -> Wire<VertexReference> {
    let [existing, birth_key] = fixed::<2>(value, "a vertex reference")?;
    Ok(VertexReference { existing: optional(existing, |found| i64_of(found, "an existing vertex"))?, birth_key: optional(birth_key, val_of)? })
}

fn plan_of(value: &Value) -> Wire<ComponentPlan> {
    let [time, events, dead, births, wiring, rewrites, cuts, contacts] = fixed::<8>(value, "a plan")?;
    let events = list(events, "plan events")?.iter().map(event_of).collect::<Wire<Vec<_>>>()?;
    let time = Rc::new(time_of(time)?);
    let first = events.first();
    Ok(ComponentPlan {
        event_kinds: Vec::new(),
        point: first.map_or_else(|| Rc::new(crate::time::EventPoint { x: cftuv_core::sqrt_sum::SqrtSum::zero(), y: cftuv_core::sqrt_sum::SqrtSum::zero() }), |event| Rc::clone(&event.point)),
        time,
        point_keys: Vec::new(),
        resolution: Resolution::Split,
        events,
        participants: Vec::new(),
        target_participants: Vec::new(),
        dead_vertex_ids: ints_of(dead, "dead vertices")?,
        chains: Vec::new(),
        births: list(births, "plan births")?.iter().map(birth_of).collect::<Wire<Vec<_>>>()?,
        queue_seed_vertex_ids: Vec::new(),
        enqueue_born_vertices: false,
        edge_contacts: list(contacts, "plan contacts")?
            .iter()
            .map(|contact| -> Wire<crate::plans::EdgeContactPlan> {
                let [births, participants, kinds] = fixed::<3>(contact, "a plan contact")?;
                Ok(crate::plans::EdgeContactPlan {
                    events: Vec::new(),
                    time: Rc::new(EventTime::zero()),
                    point: Rc::new(crate::time::EventPoint { x: cftuv_core::sqrt_sum::SqrtSum::zero(), y: cftuv_core::sqrt_sum::SqrtSum::zero() }),
                    point_key: Val::none(),
                    participants: keys_of(participants, "contact participants")?,
                    dead_vertex_ids: Vec::new(),
                    chains: Vec::new(),
                    births: list(births, "contact births")?.iter().map(birth_of).collect::<Wire<Vec<_>>>()?,
                    kinds: list(kinds, "contact kinds")?.iter().map(crate::seam_builder::kind_of).collect::<Wire<Vec<_>>>()?,
                })
            })
            .collect::<Wire<Vec<_>>>()?,
        split_cuts: list(cuts, "plan cuts")?
            .iter()
            .map(|cut| -> Wire<crate::plans::SplitCutPlan> {
                let [edge, target, segments] = fixed::<3>(cut, "a plan cut")?;
                Ok(crate::plans::SplitCutPlan {
                    edge_id: i64_of(edge, "a cut edge")?,
                    target_occurrence: val_of(target)?,
                    events: Vec::new(),
                    segment_occurrences: vals_of(segments, "segments")?,
                    births: Vec::new(),
                    final_birth_ports: Vec::new(),
                })
            })
            .collect::<Wire<Vec<_>>>()?,
        vertex_meetings: Vec::new(),
        proof_fallbacks: Vec::new(),
        coincident_split_targets: 0,
        closed_chain_count: 0,
        suppressed_candidates: 0,
        birth_wiring: list(wiring, "plan wiring")?
            .iter()
            .map(|wire| -> Wire<(Val, VertexReference, VertexReference)> {
                let [key, predecessor, successor] = fixed::<3>(wire, "a birth wire")?;
                Ok((val_of(key)?, reference_of(predecessor)?, reference_of(successor)?))
            })
            .collect::<Wire<Vec<_>>>()?,
        existing_port_rewrites: list(rewrites, "plan rewrites")?
            .iter()
            .map(|rewrite| -> Wire<(i64, Val, Val)> {
                let [ident, prev, next] = fixed::<3>(rewrite, "a rewrite")?;
                Ok((i64_of(ident, "a rewritten vertex")?, val_of(prev)?, val_of(next)?))
            })
            .collect::<Wire<Vec<_>>>()?,
        birth_components: Vec::new(),
        terminal_birth_cycles: Vec::new(),
    })
}

fn materialization_of(value: &Value) -> Wire<Materialization> {
    let [plans, families] = fixed::<2>(value, "a materialization")?;
    let families = list(families, "families")?
        .iter()
        .map(|family| -> Wire<FamilyNormalForm> {
            let [segments, births] = fixed::<2>(family, "a family")?;
            Ok(FamilyNormalForm { family: Val::none(), contacts: Vec::new(), segments: vals_of(segments, "segments")?, births: vals_of(births, "family births")? })
        })
        .collect::<Wire<Vec<_>>>()?;
    Ok(Materialization { plans: list(plans, "plans")?.iter().map(plan_of).collect::<Wire<Vec<_>>>()?, families, signature: Val::tuple(Vec::new()), unresolved_reason: None })
}

fn delta_of(value: &Value) -> Wire<Delta> {
    let [keys, dead, incoming, outgoing, birth, point_key, leaves, rewires] = fixed::<8>(value, "a delta")?;
    Ok(Delta {
        contact_keys: vals_of(keys, "contact keys")?,
        dead_refs: list(dead, "dead refs")?.iter().map(jref_of).collect::<Wire<Vec<_>>>()?,
        incoming: optional(incoming, port_of)?,
        outgoing: optional(outgoing, port_of)?,
        birth_ref: opt_jref(birth)?,
        point_key: val_of(point_key)?,
        leaf_resources: list(leaves, "leaf resources")?.iter().map(leaf_of).collect::<Wire<Vec<_>>>()?,
        rewires: list(rewires, "rewires")?
            .iter()
            .map(|rewire| -> Wire<(Port, Port, JRef)> {
                let [first, second, reference] = fixed::<3>(rewire, "a rewire")?;
                Ok((port_of(first)?, port_of(second)?, jref_of(reference)?))
            })
            .collect::<Wire<Vec<_>>>()?,
    })
}

fn interior_contact_of(value: &Value) -> Wire<SymSplitContact> {
    let [key, time, point, projection, leaf] = fixed::<5>(value, "an interior contact")?;
    Ok(SymSplitContact {
        key: SplitKey::from_val(&val_of(key)?).map_err(refusal)?,
        time: Rc::new(time_of(time)?),
        point: Rc::new(point_of(point)?),
        projection: optional(projection, |found| Ok(sum_of(found, "a projection")?.clone()))?,
        leaf: optional(leaf, leaf_of)?,
    })
}

fn junction_contact_of(value: &Value) -> Wire<JunctionContact> {
    let [kind, key, dead, families, edge] = fixed::<5>(value, "a junction contact")?;
    let key = val_of(key)?;
    let dead_refs = list(dead, "dead refs")?.iter().map(jref_of).collect::<Wire<Vec<_>>>()?;
    let families = vals_of(families, "families")?;
    match str_of(kind, "a contact kind")?.as_str() {
        "EDGE" => {
            let [_, prev, shared, next, unproven, participants] = fixed::<6>(edge, "an edge contact")?;
            let contact = EdgeContact {
                key: EdgeContactKey::from_val(&key).map_err(refusal)?,
                prev_leaf: leaf_of(prev)?,
                shared_leaf: leaf_of(shared)?,
                next_leaf: leaf_of(next)?,
                span_unproven: flag_of(unproven, "span_unproven")?,
                participant_keys: keys_of(participants, "participant keys")?,
            };
            Ok(JunctionContact {
                kind: ContactKind::Edge,
                identity: Val::tuple(vec![Val::str("EDGE"), key.clone()]),
                time_key: contact.key.time_key.clone(),
                point_key: contact.key.point_key.clone(),
                dead_refs,
                families,
                key,
                edge: Some(contact),
                endpoint: None,
            })
        }
        "ENDPOINT" => {
            let endpoint = EndpointKey::from_val(&key).map_err(refusal)?;
            Ok(JunctionContact {
                kind: ContactKind::Endpoint,
                identity: Val::tuple(vec![Val::str("ENDPOINT"), key.clone()]),
                time_key: endpoint.time_key.clone(),
                point_key: endpoint.point_key.clone(),
                dead_refs,
                families,
                key,
                edge: None,
                endpoint: Some(endpoint),
            })
        }
        _ => Err(bad("a contact kind")),
    }
}

/// What the discoveries of one round answer instead of the front's: `None` leaves the question to the front.
struct Round {
    junction: Option<(Vec<JunctionContact>, Option<&'static str>)>,
    interior: Option<(Vec<SymSplitContact>, Option<&'static str>)>,
}

/// The reasons a scripted discovery may name (the two the oracle's discoveries of the closure name).
fn known_reason(text: &str) -> Wire<&'static str> {
    const REASONS: [&str; 2] = ["SYMBOLIC_INTERIOR_SPLIT_CONTACT_METADATA_CONFLICT", "SYMBOLIC_ENDPOINT_REFERENCE_AMBIGUOUS"];
    REASONS.into_iter().find(|reason| *reason == text).ok_or_else(|| bad("a reason a discovery does not name"))
}

fn round_of(value: &Value) -> Wire<Round> {
    let [junction, junction_reason, interior, interior_reason] = fixed::<4>(value, "a round of a script")?;
    let reason = |found: &Value| -> Wire<Option<&'static str>> { optional(found, |text| known_reason(&str_of(text, "a reason")?)) };
    Ok(Round {
        junction: match junction {
            Value::None => None,
            found => Some((list(found, "junction contacts")?.iter().map(junction_contact_of).collect::<Wire<Vec<_>>>()?, reason(junction_reason)?)),
        },
        interior: match interior {
            Value::None => None,
            found => Some((list(found, "interior contacts")?.iter().map(interior_contact_of).collect::<Wire<Vec<_>>>()?, reason(interior_reason)?)),
        },
    })
}

/// The discovery of a script: the round with the number of the call answers what the script says, and the front answers every other.
struct Scripted {
    rounds: Vec<Round>,
    junction_calls: usize,
    interior_calls: usize,
}

impl Discovery for Scripted {
    fn junction(&mut self, ctx: &mut ExactCtx<'_>, builder: &mut Builder, overlay: &Overlay) -> SkelResult<(Vec<JunctionContact>, Option<&'static str>)> {
        let call = self.junction_calls;
        self.junction_calls += 1;
        match self.rounds.get(call).and_then(|round| round.junction.clone()) {
            Some(answer) => Ok(answer),
            None => Natural.junction(ctx, builder, overlay),
        }
    }

    fn interior(&mut self, ctx: &mut ExactCtx<'_>, builder: &mut Builder, overlay: &Overlay) -> SkelResult<(Vec<SymSplitContact>, Option<&'static str>)> {
        let call = self.interior_calls;
        self.interior_calls += 1;
        match self.rounds.get(call).and_then(|round| round.interior.clone()) {
            Some(answer) => Ok(answer),
            None => Natural.interior(ctx, builder, overlay),
        }
    }
}

// --------------------------------------------------------------------------
// the seams
// --------------------------------------------------------------------------

fn text_value(val: &Val) -> Value {
    str_value(&val.repr())
}

fn none_text() -> Val {
    Val::none()
}

fn reason_val(reason: Option<&'static str>) -> Val {
    reason.map_or_else(Val::none, Val::str)
}

fn answered(val: &Val, before: (usize, usize), after: (usize, usize)) -> Value {
    Value::List(vec![text_value(val), growth_value(before, after)])
}

/// What a seam runs once its arguments are read: the stopwatch of the seam starts after the decoding (the time of a seam is the compute of the closure, not of the wire).
type Run = Box<dyn for<'a, 'b> FnOnce(&'a mut ExactCtx<'b>, &mut Builder) -> SkelResult<Val>>;

fn decode_call(code: u16, args: &[Value]) -> Wire<Run> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    Ok(match code {
        310 => {
            let snapshot = snapshot_of(at(2)?)?;
            let (outer, junction) = (i64_of(at(3)?, "the outer budget")?, i64_of(at(4)?, "the junction budget")?);
            Box::new(move |ctx, builder| plan_symbolic_superlevel_closure(ctx, builder, &snapshot, outer, junction).map(|found| closure_val(&found)))
        }
        311 => {
            let vertices = list(at(2)?, "vertices")?.iter().map(vertex_snapshot_of).collect::<Wire<Vec<VertexSnapshot>>>()?;
            let materialization = materialization_of(at(3)?)?;
            let ported = flag_of(at(4)?, "include line ports")?;
            Box::new(move |ctx, builder| build_symbolic_overlay(ctx, builder, &vertices, &materialization, ported).map(|found| found.map_or_else(none_text, |overlay| overlay_val(&overlay))))
        }
        312 => {
            let overlay = overlay_of(at(2)?)?;
            Box::new(move |ctx, builder| discover_interior_split_contacts(ctx, builder, &overlay).map(|(contacts, reason)| contacts_val(&contacts, reason)))
        }
        313 => {
            let (overlay, budget) = (overlay_of(at(2)?)?, i64_of(at(3)?, "the budget")?);
            let script = match args.get(4) {
                None | Some(Value::None) => Vec::new(),
                Some(found) => list(found, "a script")?.iter().map(round_of).collect::<Wire<Vec<_>>>()?,
            };
            Box::new(move |ctx, builder| {
                let mut memo = SignatureMemo::new();
                let mut scripted = Scripted { rounds: script, junction_calls: 0, interior_calls: 0 };
                plan_mixed_generations_with(ctx, builder, &overlay, budget, &mut memo, &mut scripted).map(|(found, later)| Val::tuple(vec![fixed_point_val(&found), Val::tuple(later.iter().map(PlanVal::to_val).collect())]))
            })
        }
        314 => {
            let overlay = overlay_of(at(2)?)?;
            Box::new(move |ctx, builder| discover_junction_contacts(ctx, builder, &overlay).map(|(contacts, reason)| contacts_val(&contacts, reason)))
        }
        315 => {
            let overlay = overlay_of(at(2)?)?;
            let deltas = list(at(3)?, "deltas")?.iter().map(delta_of).collect::<Wire<Vec<_>>>()?;
            let reason = match str_of(at(4)?, "the collision reason")?.as_str() {
                "SYMBOLIC_MIXED_COMPONENT_DELTAS_OVERLAP" => "SYMBOLIC_MIXED_COMPONENT_DELTAS_OVERLAP",
                "SYMBOLIC_JUNCTION_BIRTH_COLLISION" => "SYMBOLIC_JUNCTION_BIRTH_COLLISION",
                _ => return Err(bad("a collision reason the oracle does not use")),
            };
            Box::new(move |_ctx, _builder| apply_component_deltas(&overlay, &deltas, reason).map(|(found, why)| Val::tuple(vec![found.map_or_else(none_text, |result| overlay_val(&result)), reason_val(why)])))
        }
        316 => decode_part(args)?,
        317 => {
            let overlay = overlay_of(at(2)?)?;
            let junction = list(at(3)?, "junction contacts")?.iter().map(junction_contact_of).collect::<Wire<Vec<_>>>()?;
            let interior = list(at(4)?, "interior contacts")?.iter().map(interior_contact_of).collect::<Wire<Vec<_>>>()?;
            Box::new(move |ctx, builder| {
                let normalized = normalize_mixed_generation(ctx, builder, &overlay, &junction, &interior)?;
                let (expanded, generation) = match normalized {
                    Err(reason) => return Ok(Val::tuple(vec![Val::none(), Val::none(), Val::str(reason), Val::none(), Val::none()])),
                    Ok(found) => found,
                };
                let (applied, why) = apply_mixed_generation(&expanded, &generation)?;
                Ok(Val::tuple(vec![overlay_val(&expanded), generation.to_val(), Val::none(), applied.map_or_else(none_text, |result| overlay_val(&result)), reason_val(why)]))
            })
        }
        other => return Err(bad(&format!("unknown closure seam opcode {other}"))),
    })
}

fn contacts_val<T: PlanVal>(contacts: &[T], reason: Option<&'static str>) -> Val {
    Val::tuple(vec![Val::tuple(contacts.iter().map(PlanVal::to_val).collect()), reason_val(reason)])
}

/// `CLOSURE_PART`: the small functions of the closure on their own.
fn decode_part(args: &[Value]) -> Wire<Run> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    Ok(match i64_of(at(2)?, "the operation")? {
        0 => {
            let vertices = list(at(3)?, "vertices")?.iter().map(vertex_snapshot_of).collect::<Wire<Vec<VertexSnapshot>>>()?;
            let time = time_of(at(4)?)?;
            Box::new(move |ctx, builder| with_line_ports(ctx, builder, &vertices, &time).map(|found| found.map_or_else(none_text, |ported| Val::tuple(ported.iter().map(PlanVal::to_val).collect()))))
        }
        1 => {
            let vertices = list(at(3)?, "vertices")?.iter().map(vertex_snapshot_of).collect::<Wire<Vec<VertexSnapshot>>>()?;
            let time = Rc::new(time_of(at(4)?)?);
            Box::new(move |ctx, builder| build_f0_overlay(ctx, builder, &vertices, &time).map(|found| found.map_or_else(none_text, |overlay| overlay_val(&overlay))))
        }
        2 => {
            let snapshot = snapshot_of(at(3)?)?;
            Box::new(move |_ctx, _builder| initial_interior_contacts(&snapshot).map(|(contacts, reason)| contacts_val(&contacts, reason)))
        }
        3 => {
            let overlay = overlay_of(at(3)?)?;
            Box::new(move |_ctx, _builder| overlay_signature(&overlay, &mut SignatureMemo::new()))
        }
        _ => return Err(bad("the closure part")),
    })
}

pub(crate) fn dispatch(code: u16, args: &[Value], ctx: &mut ExactCtx<'_>, extras: &mut Vec<Value>) -> Wire<SkelResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    let mut builder: Builder = builder_of(at(0)?, options_of(at(1)?)?)?;
    let run = decode_call(code, args)?;
    let before = builder.memo.len();
    crate::profile::reset();
    let started = Instant::now();
    let answer = run(ctx, &mut builder);
    extras.push(nanoseconds(started));
    let phases = crate::profile::take();
    if !phases.is_empty() {
        extras.push(Value::List(phases.into_iter().map(crate::wire::int).collect()));
    }
    Ok(answer.map(|val| answered(&val, before, builder.memo.len())))
}
