//! The single commit of a closed symbolic superlevel overlay (`wavefront/symbolic_runtime_commit.py`): the plan that validates every reference of the final overlay
//! BEFORE the first runtime allocation or write ([`plan_symbolic_runtime_commit`]), and the materialization that applies it to the builder
//! ([`materialize_symbolic_runtime_commit`]).
//!
//! The plan is the last place the transaction pays for exact questions before it writes: the Q-08 guard classifies every changed adjacency of the overlay by its affine
//! birth law (`poststate::classify_poststate_span`, one signed length and slope each), and asks the exact EDGE event of every one of them, the first of the two passes
//! over the alive junctions in the order of their `repr`. A refusal is a named reason (`&'static str`); nothing mutates the builder until the plan is whole.
//!
//! The materialization writes in the oracle's order: the runtime edges of every split edge (twins of the SOURCE edge, in the order of the physical ids), the born vertices,
//! the named fallbacks, the nodes of every component, the nodes of the later contacts, the death of the existing vertices, the new links of the alive ones, and the
//! candidates that follow the new topology, enqueued only if they lie in the future of the packet (`Builder::future_only`: a `compare_times` per push).
//!
//! What is not ported, by name: the fingerprint of the physical edges that the oracle takes at the end of the plan and compares at the start of the materialization
//! (`SYMBOLIC_RUNTIME_EDGE_FINGERPRINT_CHANGED`). Both run inside one `apply_superlevel_transaction` with no write in between, so the comparison cannot differ; it asks no
//! exact question and has no effect to keep.
//!
//! Where the oracle's answer depends on the iteration order of a `set` of objects that hold text (a hash order that changes between runs), the port takes the order of
//! the `repr`: the leaves referenced by the commit are checked in that order, so that the REASON of a refusal of a packet with two different faults is fixed (the oracle's
//! is not). A packet with ONE fault names it either way.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::ExactCtx;

use crate::builder::{Builder, Counter, NewVertex, ProofBranchOrRefusal};
use crate::candidate::CandidateRefusal;
use crate::component::point_from_key;
use crate::contacts::ContactKind;
use crate::coordinator::SymbolicSuperlevelClosure;
use crate::error::{SkelError, SkelResult};
use crate::overlay::{edge_key_of, with_view, JRef, Leaf, Overlay, SymVertex};
use crate::poststate::{classify_poststate_span, PoststateDisposition};
use crate::proof::{EdgeKey, ProofBranch, ProofDisposition};
use crate::pyval::Val;
use crate::queue::{CandidateEvent, EventKind};
use crate::snapshot::{same_event, time_key, Snapshot};
use crate::superlevel::emit_component_nodes;
use crate::time::{compare_times, PointRef, TimeOutcome, TimeRef};
use crate::view::{edge_event_time, position};

/// `SymbolicNodePayloadV1`: a node of a contact that was found after the first packet of the closure (a generation of the junction fixed point).
#[derive(Clone)]
pub struct NodePayload {
    pub kind: EventKind,
    pub time: TimeRef,
    pub point: PointRef,
    pub participants: Vec<EdgeKey>,
    pub dead_refs: Vec<JRef>,
    pub span_unproven: bool,
    pub target_participants: Vec<EdgeKey>,
    pub vertex_meeting: bool,
}

/// `SymbolicFallbackPayloadV1`: a candidate of the packet that no causal endpoint consumed, kept as a named refusal.
#[derive(Clone)]
pub struct FallbackPayload {
    pub vertex_ids: Vec<i64>,
    pub participant_edge_keys: Vec<EdgeKey>,
    pub target_edge_keys: Vec<EdgeKey>,
}

/// `SymbolicRuntimeCommitPlanV1` (without the fingerprint, see the module note): the final overlay projected on the runtime, every reference validated.
pub struct CommitPlan<'c> {
    pub closure: &'c SymbolicSuperlevelClosure,
    /// `(physical edge id, its leaves in the order of their occurrence)`, by edge id.
    pub active_leaves_by_edge: Vec<(i64, Vec<Leaf>)>,
    /// The alive existing junctions, by runtime id.
    pub alive_existing: Vec<&'c SymVertex>,
    /// The runtime ids of the existing vertices that die (those the plans kill, and those the overlay ends dead), ascending.
    pub dead_existing: Vec<i64>,
    /// The junctions the overlay gave birth to, in the order of their `repr`.
    pub births: Vec<&'c SymVertex>,
    pub affected_edges: Vec<i64>,
    pub later_nodes: Vec<NodePayload>,
    pub unresolved_fallbacks: Vec<FallbackPayload>,
}

type Planned<'c> = (Option<CommitPlan<'c>>, Option<&'static str>);

fn by_repr<T>(items: &mut [T], text: impl Fn(&T) -> Rc<str>) {
    items.sort_by(|left, right| text(left).as_bytes().cmp(text(right).as_bytes()));
}

fn key_error(what: &str) -> SkelError {
    SkelError::Unsupported(format!("KeyError in the oracle: {what}"))
}

/// `_later_node_payloads(closure)`.
fn later_node_payloads(closure: &SymbolicSuperlevelClosure, overlay: &Overlay) -> SkelResult<Vec<NodePayload>> {
    let mut payloads = Vec::new();
    let Some(junction) = &closure.junction else {
        return Err(SkelError::Unsupported("AttributeError in the oracle: a closure without a junction fixed point".to_string()));
    };
    for generation in &junction.generations {
        for contact in &generation.junction_contacts {
            let point = Rc::new(point_from_key(&contact.point_key)?);
            if contact.kind == ContactKind::Edge {
                let edge = contact.edge.as_ref().ok_or_else(|| SkelError::Unsupported("AttributeError in the oracle: an edge contact without an edge".to_string()))?;
                payloads.push(NodePayload {
                    kind: EventKind::Edge,
                    time: Rc::clone(&overlay.time),
                    point,
                    participants: edge.participant_keys.clone(),
                    dead_refs: contact.dead_refs.clone(),
                    span_unproven: edge.span_unproven,
                    target_participants: edge.shared_leaf.participant_keys(),
                    vertex_meeting: false,
                });
            } else {
                let participants: Option<Vec<EdgeKey>> = contact.endpoint.as_ref().and_then(|key| key.val.field("participants")).and_then(|found| found.items()).map(|items| items.iter().filter_map(edge_key_of).collect());
                payloads.push(NodePayload {
                    kind: EventKind::Split,
                    time: Rc::clone(&overlay.time),
                    point,
                    participants: participants.ok_or_else(|| SkelError::Unsupported("AttributeError in the oracle: an endpoint contact without participants".to_string()))?,
                    dead_refs: contact.dead_refs.clone(),
                    span_unproven: false,
                    target_participants: Vec::new(),
                    vertex_meeting: true,
                });
            }
        }
        for contact in &generation.interior_contacts {
            payloads.push(NodePayload {
                kind: EventKind::Split,
                time: Rc::clone(&contact.time),
                point: Rc::clone(&contact.point),
                participants: contact.key.participants.clone(),
                dead_refs: vec![contact.key.emitter.clone()],
                span_unproven: false,
                target_participants: contact.key.family_participants(),
                vertex_meeting: false,
            });
        }
    }
    Ok(payloads)
}

/// `_unresolved_fallbacks(snapshot, closure)`: the fallbacks of the plans that no endpoint contact of the junction generations consumed.
fn unresolved_fallbacks(snapshot: &Snapshot, closure: &SymbolicSuperlevelClosure) -> SkelResult<Vec<FallbackPayload>> {
    let (Some(junction), Some(materialization)) = (&closure.junction, &closure.materialization) else {
        return Err(SkelError::Unsupported("AttributeError in the oracle: a closure without a junction or a materialization".to_string()));
    };
    let endpoints: Vec<_> = junction.generations.iter().flat_map(|generation| generation.junction_contacts.iter()).filter(|contact| contact.kind == ContactKind::Endpoint).filter_map(|contact| contact.endpoint.as_ref()).collect();
    let mut unresolved = Vec::new();
    for component in &materialization.plans {
        for event in &component.proof_fallbacks {
            // `{incident.event: incident ...}`: the last incident of an equal event wins
            let incident = snapshot.incidents.iter().rev().find(|incident| same_event(&incident.event, event));
            let mut resolved = false;
            if let Some(incident) = incident {
                let time = time_key(&incident.event.time)?;
                let participants = crate::contacts::keys_val(&incident.participants);
                resolved = endpoints.iter().any(|key| {
                    key.time_key == time
                        && key.point_key == incident.point_key
                        && key.emitter.key() == incident.emitter_key
                        && incident.target_occurrence.as_ref().is_some_and(|occurrence| key.family.field("occurrence") == Some(occurrence))
                        && key.val.field("participants") == Some(&participants)
                });
            }
            if resolved {
                continue;
            }
            let target = match incident.and_then(|incident| incident.target_occurrence.as_ref()) {
                Some(occurrence) => vec![occurrence.get(0).and_then(edge_key_of).ok_or_else(|| SkelError::Unsupported("an occurrence without an edge key".to_string()))?],
                None => Vec::new(),
            };
            let vertex_ids = match incident {
                None => vec![event.vertex],
                Some(incident) => {
                    let mut found: BTreeSet<i64> = BTreeSet::from([event.vertex]);
                    found.extend(incident.target_start_id);
                    found.extend(incident.target_end_id);
                    found.into_iter().collect()
                }
            };
            unresolved.push(FallbackPayload { vertex_ids, participant_edge_keys: incident.map_or_else(Vec::new, |incident| incident.participants.clone()), target_edge_keys: target });
        }
    }
    Ok(unresolved)
}

/// `_ordered_lineage(leaves)`: the leaves of one physical root family by their occurrence; `None` when they are of several families or two have one occurrence.
fn ordered_lineage(leaves: &[Leaf]) -> Option<Vec<Leaf>> {
    let first = leaves.first()?;
    if leaves.iter().any(|leaf| leaf.family() != first.family()) {
        return None;
    }
    let occurrences: HashSet<_> = leaves.iter().map(Leaf::occurrence).collect();
    if occurrences.len() != leaves.len() {
        return None;
    }
    let mut ordered = leaves.to_vec();
    by_repr(&mut ordered, |leaf| leaf.occurrence().repr());
    Some(ordered)
}

/// The alive junctions of the overlay in the order of the `repr` of their reference (`sorted(alive, key=repr)`).
fn alive_by_repr(overlay: &Overlay) -> Vec<&SymVertex> {
    let mut alive: Vec<&SymVertex> = overlay.vertices.values().filter(|vertex| vertex.alive).collect();
    by_repr(&mut alive, |vertex| vertex.reference.val().repr());
    alive
}

fn alive_peer<'a>(overlay: &'a Overlay, vertex: &SymVertex) -> Option<&'a SymVertex> {
    let next = vertex.next.as_ref()?;
    overlay.vertices.get(next).filter(|peer| peer.alive)
}

/// `changed_adjacency`: the pair is not the one the frozen front had.
fn changed_adjacency(snapshot: &Snapshot, vertex: &SymVertex, peer: &SymVertex) -> bool {
    !vertex.reference.is_existing() || !peer.reference.is_existing() || vertex.runtime_id.and_then(|ident| snapshot.vertices.get(ident as usize)).map(|frozen| frozen.next) != peer.runtime_id
}

/// What `changed_poststate_edge_events` saw of one changed adjacency: the time of its EDGE event and the births of its two ends.
struct PastEdgeEvent {
    event_time: TimeRef,
    vertex_birth: TimeRef,
    peer_birth: TimeRef,
}

/// `changed_poststate_edge_events(builder, snapshot, overlay)`: the exact EDGE event of every changed adjacency (a pair with an EXACT time), keyed by the two junctions.
fn changed_poststate_edge_events(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, overlay: &Overlay) -> SkelResult<HashMap<(JRef, JRef), PastEdgeEvent, FxBuild>> {
    let alive = alive_by_repr(overlay);
    with_view(builder, overlay, |_, view, memo| {
        let mut witnesses: HashMap<(JRef, JRef), PastEdgeEvent, FxBuild> = HashMap::default();
        for vertex in alive {
            let Some(peer) = alive_peer(overlay, vertex) else {
                continue;
            };
            if !changed_adjacency(snapshot, vertex, peer) {
                continue;
            }
            let (vertex_ref, peer_ref) = (view.vertex_ref(&vertex.reference)?, view.vertex_ref(&peer.reference)?);
            let (event_time, outcome) = edge_event_time(ctx, view, memo, vertex_ref, peer_ref, &overlay.time)?;
            let (TimeOutcome::Exact, Some(event_time)) = (outcome, event_time) else {
                continue;
            };
            // the oracle takes the place of the vertex at that time for the record of the witness (a hydration, paid)
            position(ctx, view, memo, vertex_ref, &event_time)?;
            witnesses.insert((vertex.reference.clone(), peer.reference.clone()), PastEdgeEvent { event_time, vertex_birth: Rc::clone(&vertex.birth), peer_birth: Rc::clone(&peer.birth) });
        }
        Ok(witnesses)
    })
}

/// `_poststate_span_refusal(builder, snapshot, overlay)`: Q-08, the last guard after the affine law has classified every changed span.
fn poststate_span_refusal(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, overlay: &Overlay) -> SkelResult<Option<&'static str>> {
    let alive = alive_by_repr(overlay);
    let spans: Vec<((JRef, JRef), PoststateDisposition)> = with_view(builder, overlay, |_, view, memo| {
        let mut spans = Vec::new();
        for vertex in alive {
            let Some(peer) = alive_peer(overlay, vertex) else {
                continue;
            };
            // a two-vertex terminal cycle has two shared leaves and no unique oriented newborn span: the terminal certificate owns its closure
            if peer.next.as_ref() == Some(&vertex.reference) {
                continue;
            }
            if !changed_adjacency(snapshot, vertex, peer) {
                continue;
            }
            let (vertex_ref, peer_ref) = (view.vertex_ref(&vertex.reference)?, view.vertex_ref(&peer.reference)?);
            let classification = classify_poststate_span(ctx, view, memo, vertex_ref, peer_ref, &overlay.time)?;
            spans.push(((vertex.reference.clone(), peer.reference.clone()), classification.disposition));
        }
        Ok::<_, SkelError>(spans)
    })?;
    if spans.iter().any(|(_, disposition)| *disposition == PoststateDisposition::Inverted) {
        return Ok(Some("SYMBOLIC_POSTSTATE_SPAN_INVERTED"));
    }
    if spans.iter().any(|(_, disposition)| *disposition == PoststateDisposition::AffineClassificationUnproven) {
        return Ok(Some("SYMBOLIC_POSTSTATE_SPAN_AFFINE_CLASSIFICATION_UNPROVEN"));
    }
    let exact_events = changed_poststate_edge_events(ctx, builder, snapshot, overlay)?;
    for (pair, disposition) in &spans {
        if *disposition != PoststateDisposition::ClosingWithFutureEvent {
            continue;
        }
        let proven = match exact_events.get(pair) {
            None => false,
            Some(event) => {
                compare_times(ctx, &event.event_time, &overlay.time)? > 0 && compare_times(ctx, &event.event_time, &event.vertex_birth)? > 0 && compare_times(ctx, &event.event_time, &event.peer_birth)? > 0
            }
        };
        if !proven {
            return Ok(Some("SYMBOLIC_POSTSTATE_CLOSING_EVENT_UNPROVEN"));
        }
    }
    Ok(None)
}

/// `_validated_existing_vertices`: the existing junctions of the overlay and the vertices the plans kill, against the frozen front and the runtime.
fn validated_existing_vertices<'c>(builder: &Builder, snapshot: &Snapshot, overlay: &'c Overlay, closure: &SymbolicSuperlevelClosure) -> Result<(Vec<&'c SymVertex>, BTreeSet<i64>), &'static str> {
    let existing: Vec<&SymVertex> = overlay.vertices.values().filter(|vertex| vertex.reference.is_existing()).collect();
    let frozen_live: BTreeSet<i64> = snapshot.vertices.iter().filter(|vertex| vertex.alive).map(|vertex| vertex.ident).collect();
    let initially_dead: BTreeSet<i64> = closure.materialization.iter().flat_map(|found| found.plans.iter()).flat_map(|plan| plan.dead_vertex_ids.iter().copied()).collect();
    const UNRESOLVABLE: &str = "SYMBOLIC_RUNTIME_EXISTING_REF_UNRESOLVABLE";
    let mut runtime_ids: Vec<i64> = Vec::with_capacity(existing.len());
    for vertex in &existing {
        match vertex.runtime_id {
            Some(ident) => runtime_ids.push(ident),
            None => return Err(UNRESOLVABLE),
        }
    }
    let unique: BTreeSet<i64> = runtime_ids.iter().copied().collect();
    if unique.len() != runtime_ids.len() || !unique.is_disjoint(&initially_dead) || unique.union(&initially_dead).copied().collect::<BTreeSet<i64>>() != frozen_live {
        return Err(UNRESOLVABLE);
    }
    if runtime_ids.iter().any(|ident| *ident < 0 || *ident as usize >= builder.vertices.len() || !builder.vertices[*ident as usize].alive) {
        return Err(UNRESOLVABLE);
    }
    let changed = snapshot.vertices.iter().any(|item| {
        let runtime = &builder.vertices[item.ident as usize];
        runtime.prev != item.prev || runtime.next != item.next || runtime.prev_edge != item.prev_edge || runtime.next_edge != item.next_edge || runtime.alive != item.alive
    });
    if changed {
        return Err("SYMBOLIC_RUNTIME_FROZEN_PRESTATE_CHANGED");
    }
    Ok((existing, initially_dead))
}

/// What `_validated_active_leaf_groups` answers: the active leaves, and them grouped by physical edge.
type ActiveLeaves = (HashSet<Leaf, FxBuild>, HashMap<i64, Vec<Leaf>, FxBuild>);

/// `_validated_active_leaf_groups`: reciprocity of the alive junctions, one owner per active leaf, the binding of every active leaf.
fn validated_active_leaf_groups(builder: &Builder, overlay: &Overlay) -> Result<ActiveLeaves, &'static str> {
    let alive: HashMap<&JRef, &SymVertex, FxBuild> = overlay.vertices.iter().filter(|(_, vertex)| vertex.alive).collect();
    let reciprocal = alive.iter().all(|(reference, vertex)| {
        let (Some(prev), Some(next)) = (&vertex.prev, &vertex.next) else {
            return false;
        };
        match (alive.get(prev), alive.get(next)) {
            (Some(before), Some(after)) => before.next.as_ref() == Some(*reference) && after.prev.as_ref() == Some(*reference),
            _ => false,
        }
    });
    if !reciprocal {
        return Err("SYMBOLIC_RUNTIME_RECIPROCITY_UNRESOLVABLE");
    }
    let mut starts: HashMap<Leaf, Vec<&JRef>, FxBuild> = HashMap::default();
    let mut ends: HashMap<Leaf, Vec<&JRef>, FxBuild> = HashMap::default();
    for (reference, vertex) in overlay.vertices.iter().filter(|(_, vertex)| vertex.alive) {
        starts.entry(vertex.next_leaf.clone()).or_default().push(reference);
        ends.entry(vertex.prev_leaf.clone()).or_default().push(reference);
    }
    let active: HashSet<Leaf, FxBuild> = starts.keys().chain(ends.keys()).cloned().collect();
    if starts.len() != ends.len() || starts.keys().any(|leaf| !ends.contains_key(leaf)) || starts.values().chain(ends.values()).any(|refs| refs.len() != 1) {
        return Err("SYMBOLIC_RUNTIME_SPAN_OWNER_AMBIGUOUS");
    }
    let mut grouped: HashMap<i64, Vec<Leaf>, FxBuild> = HashMap::default();
    for leaf in &active {
        let binding = match overlay.spans.get(leaf) {
            Some(binding) if binding.start.as_ref() == Some(starts[leaf][0]) && binding.end.as_ref() == Some(ends[leaf][0]) && 0 <= binding.physical_edge_id && (binding.physical_edge_id as usize) < builder.edges.len() => binding,
            _ => return Err("SYMBOLIC_RUNTIME_SPAN_BINDING_UNRESOLVABLE"),
        };
        grouped.entry(binding.physical_edge_id).or_default().push(leaf.clone());
    }
    Ok((active, grouped))
}

/// `_validated_birth_context`: the births (never a virtual boundary), their leaves in the overlay, and every referenced leaf against the source key of its physical edge.
fn validated_birth_context<'c>(builder: &Builder, overlay: &'c Overlay, active: &HashSet<Leaf, FxBuild>) -> Result<Vec<&'c SymVertex>, &'static str> {
    let mut births: Vec<&SymVertex> = overlay.vertices.values().filter(|vertex| !vertex.reference.is_existing()).collect();
    by_repr(&mut births, |vertex| vertex.reference.val().repr());
    if births.iter().any(|vertex| vertex.reference.is_virtual_boundary()) {
        return Err("SYMBOLIC_RUNTIME_VIRTUAL_VERTEX_UNRESOLVABLE");
    }
    if births.iter().any(|vertex| !overlay.spans.contains_key(&vertex.prev_leaf) || !overlay.spans.contains_key(&vertex.next_leaf)) {
        return Err("SYMBOLIC_RUNTIME_BIRTH_SPAN_UNRESOLVABLE");
    }
    let mut referenced: HashSet<Leaf, FxBuild> = active.clone();
    for vertex in &births {
        referenced.insert(vertex.prev_leaf.clone());
        referenced.insert(vertex.next_leaf.clone());
    }
    let mut referenced: Vec<Leaf> = referenced.into_iter().collect();
    by_repr(&mut referenced, |leaf| leaf.val().repr());
    for leaf in &referenced {
        let Some(binding) = overlay.spans.get(leaf) else {
            return Err("SYMBOLIC_RUNTIME_BIRTH_SPAN_UNRESOLVABLE");
        };
        if binding.physical_edge_id < 0 || binding.physical_edge_id as usize >= builder.edges.len() {
            return Err("SYMBOLIC_RUNTIME_PHYSICAL_EDGE_UNRESOLVABLE");
        }
        let source = &builder.edges[binding.physical_edge_id as usize].span;
        let source_key = Val::ints(source);
        let occurrence_source = leaf.occurrence().get(0).cloned();
        let family_source = leaf.family().field("occurrence").and_then(|occurrence| occurrence.get(0)).cloned();
        let listed = leaf.participant_keys().iter().any(|key| key.as_slice() == source.as_slice());
        if occurrence_source.as_ref() != Some(&source_key) || family_source.as_ref() != Some(&source_key) || !listed {
            return Err("SYMBOLIC_RUNTIME_EDGE_AUTHORITY_MISMATCH");
        }
    }
    Ok(births)
}

/// `plan_symbolic_runtime_commit(builder, snapshot, closure)`: validate every reference before the first runtime allocation or write; the plan, or the named reason.
pub fn plan_symbolic_runtime_commit<'c>(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, closure: &'c SymbolicSuperlevelClosure) -> SkelResult<Planned<'c>> {
    let (Some(overlay), Some(materialization)) = (&closure.overlay, &closure.materialization) else {
        return Ok((None, Some("SYMBOLIC_RUNTIME_FINAL_OVERLAY_UNAVAILABLE")));
    };
    if let Some(reason) = materialization.unresolved_reason {
        return Ok((None, Some(reason)));
    }
    let (existing, initially_dead) = match validated_existing_vertices(builder, snapshot, overlay, closure) {
        Ok(found) => found,
        Err(reason) => return Ok((None, Some(reason))),
    };
    let (active, grouped) = match validated_active_leaf_groups(builder, overlay) {
        Ok(found) => found,
        Err(reason) => return Ok((None, Some(reason))),
    };
    let births = match validated_birth_context(builder, overlay, &active) {
        Ok(found) => found,
        Err(reason) => return Ok((None, Some(reason))),
    };
    let mut alive_existing: Vec<&SymVertex> = existing.iter().copied().filter(|vertex| vertex.alive).collect();
    alive_existing.sort_by_key(|vertex| vertex.runtime_id);
    let mut dead: BTreeSet<i64> = initially_dead;
    dead.extend(existing.iter().filter(|vertex| !vertex.alive).filter_map(|vertex| vertex.runtime_id));
    let affected: BTreeSet<i64> = overlay.changed.iter().filter(|leaf| active.contains(*leaf)).map(|leaf| overlay.spans.get(leaf).map(|binding| binding.physical_edge_id).ok_or_else(|| key_error("a changed leaf without a binding"))).collect::<SkelResult<_>>()?;
    let known: HashSet<&JRef, FxBuild> = existing.iter().map(|vertex| &vertex.reference).chain(births.iter().map(|vertex| &vertex.reference)).collect();
    let later_nodes = later_node_payloads(closure, overlay)?;
    if later_nodes.iter().any(|payload| payload.dead_refs.iter().any(|reference| !known.contains(reference))) {
        return Ok((None, Some("SYMBOLIC_RUNTIME_NODE_REF_UNRESOLVABLE")));
    }
    let mut lineages: Vec<(i64, Vec<Leaf>)> = Vec::with_capacity(grouped.len());
    for (edge_id, leaves) in &grouped {
        match ordered_lineage(leaves) {
            Some(ordered) => lineages.push((*edge_id, ordered)),
            None => return Ok((None, Some("SYMBOLIC_RUNTIME_EDGE_LINEAGE_UNRESOLVABLE"))),
        }
    }
    lineages.sort_by_key(|(edge_id, _)| *edge_id);
    if let Some(reason) = poststate_span_refusal(ctx, builder, snapshot, overlay)? {
        return Ok((None, Some(reason)));
    }
    let plan = CommitPlan {
        closure,
        active_leaves_by_edge: lineages,
        alive_existing,
        dead_existing: dead.into_iter().collect(),
        births,
        affected_edges: affected.into_iter().collect(),
        later_nodes,
        unresolved_fallbacks: unresolved_fallbacks(snapshot, closure)?,
    };
    Ok((Some(plan), None))
}

/// `_runtime_ids(refs, runtime_by_ref)`: the sorted distinct runtime ids of the junctions.
fn runtime_ids(refs: &[JRef], runtime_by_ref: &HashMap<JRef, i64, FxBuild>) -> SkelResult<Vec<i64>> {
    let mut found = BTreeSet::new();
    for reference in refs {
        found.insert(*runtime_by_ref.get(reference).ok_or_else(|| key_error("a junction without a runtime vertex"))?);
    }
    Ok(found.into_iter().collect())
}

/// `_emit_later_nodes(builder, plan, runtime_by_ref)`: the causal symbolic contacts become nodes; the final accumulator owns the unions of nodes at one point.
fn emit_later_nodes(builder: &mut Builder, plan: &CommitPlan<'_>, runtime_by_ref: &HashMap<JRef, i64, FxBuild>) -> SkelResult<()> {
    for payload in &plan.later_nodes {
        let ids = runtime_ids(&payload.dead_refs, runtime_by_ref)?;
        let event = CandidateEvent {
            kind: payload.kind,
            time: Rc::clone(&payload.time),
            point: Rc::clone(&payload.point),
            vertex: ids.first().copied().unwrap_or(-1),
            peer: ids.get(1).copied().unwrap_or(-1),
            edge: -1,
            span_unproven: false,
        };
        builder.emit(payload.kind, &event, payload.participants.clone(), &ids);
        if payload.kind == EventKind::Edge {
            builder.counters.bump(Counter::EdgeEvents, 1);
            if payload.span_unproven {
                builder.counters.bump(Counter::EdgeCollapseSpanUnprovenButAccepted, 1);
                builder.record_obligation(
                    ProofBranchOrRefusal::Branch(ProofBranch::EdgeCollapseSpanUnproven),
                    ProofDisposition::EventAcceptedWithUnprovenSpan,
                    &(ids.clone(), payload.participants.clone(), payload.target_participants.clone()),
                    &payload.time,
                    None,
                )?;
            }
        } else {
            builder.counters.bump(Counter::SplitEvents, 1);
            if payload.vertex_meeting {
                builder.counters.bump(Counter::VertexMeetingEvents, 1);
            }
        }
        if payload.participants.len() > 3 {
            builder.counters.bump(Counter::MultiParticipantNodes, 1);
        }
    }
    Ok(())
}

/// `materialize_symbolic_runtime_commit(builder, snapshot, plan)`: commit the already validated final topology; there is no fallback path and no refusal left (the one
/// the oracle has, the fingerprint, cannot differ: see the module note).
pub fn materialize_symbolic_runtime_commit(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, plan: &CommitPlan<'_>) -> SkelResult<()> {
    let closure = plan.closure;
    let (Some(overlay), Some(materialization), Some(junction)) = (&closure.overlay, &closure.materialization, &closure.junction) else {
        return Err(SkelError::Unsupported("AttributeError in the oracle: a commit of a closure without its parts".to_string()));
    };
    let mut edge_by_leaf: HashMap<Leaf, i64, FxBuild> = HashMap::default();
    for (edge_id, leaves) in &plan.active_leaves_by_edge {
        let mut runtime_edges = vec![*edge_id];
        for _ in leaves.iter().skip(1) {
            runtime_edges.push(builder.twin(*edge_id)?);
        }
        for (leaf, edge) in leaves.iter().zip(runtime_edges) {
            edge_by_leaf.insert(leaf.clone(), edge);
        }
    }
    for symbolic in &plan.births {
        for leaf in [&symbolic.prev_leaf, &symbolic.next_leaf] {
            let binding = overlay.spans.get(leaf).ok_or_else(|| key_error("a birth leaf without a binding"))?;
            edge_by_leaf.entry(leaf.clone()).or_insert(binding.physical_edge_id);
        }
    }
    let edge_of = |leaf: &Leaf| -> SkelResult<i64> { edge_by_leaf.get(leaf).copied().ok_or_else(|| key_error("a leaf without a runtime edge")) };

    let mut runtime_by_ref: HashMap<JRef, i64, FxBuild> = HashMap::default();
    for vertex in overlay.vertices.values().filter(|vertex| vertex.reference.is_existing()) {
        runtime_by_ref.insert(vertex.reference.clone(), vertex.runtime_id.ok_or_else(|| key_error("an existing junction without a runtime id"))?);
    }
    for symbolic in &plan.births {
        let placeholder = builder.vertices.len() as i64;
        let point = symbolic.point.as_ref().map(Rc::clone).ok_or_else(|| SkelError::Unsupported("AttributeError in the oracle: a born junction without a place".to_string()))?;
        let ident = builder.new_vertex(
            ctx,
            NewVertex { prev_edge: edge_of(&symbolic.prev_leaf)?, next_edge: edge_of(&symbolic.next_leaf)?, prev: placeholder, next: placeholder, birth: Rc::clone(&symbolic.birth), point },
        )?;
        builder.vertices[ident as usize].alive = symbolic.alive;
        runtime_by_ref.insert(symbolic.reference.clone(), ident);
    }

    for fallback in &plan.unresolved_fallbacks {
        builder.refuse(CandidateRefusal::NoRuleMeetingNotReconnectable, &(fallback.vertex_ids.clone(), fallback.participant_edge_keys.clone(), fallback.target_edge_keys.clone()))?;
    }
    for component in &materialization.plans {
        emit_component_nodes(builder, component)?;
        builder.counters.bump(Counter::CoincidentSplitTargets, component.coincident_split_targets);
        builder.counters.bump(Counter::DiscardedStaleCandidates, component.suppressed_candidates);
        builder.counters.bump(Counter::Peaks, component.closed_chain_count);
    }
    emit_later_nodes(builder, plan, &runtime_by_ref)?;

    for ident in &plan.dead_existing {
        builder.vertices[*ident as usize].alive = false;
    }
    let alive_births: Vec<&SymVertex> = plan.births.iter().copied().filter(|vertex| vertex.alive).collect();
    let survivors: Vec<&SymVertex> = plan.alive_existing.iter().copied().chain(alive_births.iter().copied()).collect();
    for symbolic in &survivors {
        let ident = runtime_by_ref[&symbolic.reference] as usize;
        builder.vertices[ident].prev_edge = edge_of(&symbolic.prev_leaf)?;
        builder.vertices[ident].next_edge = edge_of(&symbolic.next_leaf)?;
    }
    for symbolic in &survivors {
        let ident = runtime_by_ref[&symbolic.reference];
        let (Some(prev), Some(next)) = (&symbolic.prev, &symbolic.next) else {
            return Err(key_error("an alive junction without a neighbour"));
        };
        let (prev, next) = (*runtime_by_ref.get(prev).ok_or_else(|| key_error("a neighbour without a runtime vertex"))?, *runtime_by_ref.get(next).ok_or_else(|| key_error("a neighbour without a runtime vertex"))?);
        builder.vertices[ident as usize].prev = prev;
        builder.vertices[ident as usize].next = next;
        builder.register(ident);
    }

    let born_ids: BTreeSet<i64> = plan.births.iter().map(|vertex| runtime_by_ref[&vertex.reference]).collect();
    builder.future_only = Some(Rc::clone(&overlay.time));
    let enqueued = enqueue_after_commit(ctx, builder, snapshot, plan, &runtime_by_ref, &alive_births, &edge_by_leaf, &born_ids);
    builder.future_only = None;
    enqueued?;

    let later_births: i64 = junction
        .generations
        .iter()
        .flat_map(|generation| generation.deltas.iter())
        .map(|delta| delta.rewires.len() as i64 + i64::from(delta.rewires.is_empty() && delta.birth_ref.is_some()))
        .sum();
    let earlier_births: i64 = materialization.plans.iter().map(|component| component.births.len() as i64).sum();
    builder.counters.bump(Counter::SuperlevelContactJunctionResolutions, earlier_births + later_births);
    Ok(())
}

/// The candidates that follow the new topology (the queue is `_FutureQueueV1` while this runs): the born vertices against everything, the existing ones whose
/// neighbours changed against their new neighbour, and every vertex that may split a span the commit created or changed.
#[allow(clippy::too_many_arguments)]
fn enqueue_after_commit(
    ctx: &mut ExactCtx<'_>,
    builder: &mut Builder,
    snapshot: &Snapshot,
    plan: &CommitPlan<'_>,
    runtime_by_ref: &HashMap<JRef, i64, FxBuild>,
    alive_births: &[&SymVertex],
    edge_by_leaf: &HashMap<Leaf, i64, FxBuild>,
    born_ids: &BTreeSet<i64>,
) -> SkelResult<()> {
    for symbolic in alive_births {
        builder.enqueue_for(ctx, runtime_by_ref[&symbolic.reference])?;
    }
    for symbolic in &plan.alive_existing {
        let ident = runtime_by_ref[&symbolic.reference];
        let frozen = snapshot.vertices.get(symbolic.runtime_id.ok_or_else(|| key_error("an existing junction without a runtime id"))? as usize).ok_or_else(|| key_error("a runtime id beyond the frozen front"))?;
        let runtime = builder.vertex_at(ident)?;
        if runtime.prev != frozen.prev || runtime.next != frozen.next || runtime.prev_edge != frozen.prev_edge || runtime.next_edge != frozen.next_edge {
            builder.enqueue_edge_event(ctx, ident)?;
        }
    }
    let affected: HashSet<i64, FxBuild> = plan.affected_edges.iter().copied().collect();
    let mut active_edges: BTreeSet<i64> = BTreeSet::new();
    for (physical, leaves) in &plan.active_leaves_by_edge {
        if !affected.contains(physical) {
            continue;
        }
        for leaf in leaves {
            active_edges.insert(*edge_by_leaf.get(leaf).ok_or_else(|| key_error("a leaf without a runtime edge"))?);
        }
    }
    for edge_id in active_edges {
        builder.enqueue_splits_against(ctx, edge_id, born_ids)?;
    }
    Ok(())
}
