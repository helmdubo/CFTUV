//! The generations of the symbolic closure (`wavefront/symbolic_junction_normalize.py`, `symbolic_mixed_generation.py`, the dataclasses of `symbolic_junction_fixed_point.py`).
//!
//! A GENERATION is what dies and what is born at one exact time on one overlay: the junction contacts (edges that collapse, vertices that reach an end of a span) normalised to
//! one delta per component, and the interior splits (a vertex that meets the inside of a leaf) which first cut the leaf, in the order of the projections of their points (CPython
//! 3.11's sequence of questions, each comparison a paid sign), then die as one junction and are born as two. [`plan_mixed_generations`] grows the causal chain of generations
//! until the overlay has no contact left: every round REPLAYS the chain from the initial overlay (the oracle's cost is the cost of the replay), then discovers.

use std::collections::{HashMap, HashSet};

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::ExactCtx;

use crate::builder::Builder;
use crate::closure::sort_by_projection;
use crate::component::{apply_component_deltas, by_repr, clone_overlay, normalize_dead_component, overlay_signature, Delta, SignatureMemo};
use crate::contacts::{discover_interior_split_contacts, discover_junction_contacts, merge_symbolic_split_contacts, valid_edge_contact, ContactKind, JunctionContact, SymSplitContact};
use crate::error::{SkelError, SkelResult};
use crate::omap::OrderedMap;
use crate::overlay::{JRef, Leaf, Overlay};
use crate::plans::PlanVal;
use crate::pyval::Val;
use crate::snapshot::direction;

fn repr_order(left: &Val, right: &Val) -> std::cmp::Ordering {
    left.repr().as_bytes().cmp(right.repr().as_bytes())
}

// --------------------------------------------------------------------------
// the delta of a junction component
// --------------------------------------------------------------------------

/// A resource a contact writes (`_resources`): the vertex rewritten, or the family touched.
#[derive(Clone, PartialEq, Eq, Hash)]
enum Resource {
    VertexWrite(Option<JRef>),
    Family(Val),
}

fn resources_of(overlay: &Overlay, contact: &JunctionContact) -> HashSet<Resource> {
    let mut result: HashSet<Resource> = contact.dead_refs.iter().map(|reference| Resource::VertexWrite(Some(reference.clone()))).collect();
    for reference in &contact.dead_refs {
        if let Some(vertex) = overlay.vertices.get(reference) {
            result.insert(Resource::VertexWrite(vertex.prev.clone()));
            result.insert(Resource::VertexWrite(vertex.next.clone()));
        }
    }
    result.extend(contact.families.iter().cloned().map(Resource::Family));
    result
}

fn identity_sorted(contacts: &[JunctionContact]) -> Vec<JunctionContact> {
    let mut sorted = contacts.to_vec();
    sorted.sort_by(|left, right| repr_order(&left.identity, &right.identity));
    sorted
}

/// `_components(overlay, contacts)`: the contacts of one time and one point that share a resource, transitively; each component by the `repr` of its identities.
fn components(overlay: &Overlay, contacts: &[JunctionContact]) -> Vec<Vec<JunctionContact>> {
    let mut pending: Vec<JunctionContact> = identity_sorted(contacts);
    let mut result = Vec::new();
    while !pending.is_empty() {
        let first = pending.remove(0);
        let mut resources = resources_of(overlay, &first);
        let time_point = (first.time_key.clone(), first.point_key.clone());
        let mut component = vec![first];
        let mut changed = true;
        while changed {
            changed = false;
            let mut rest = Vec::new();
            for contact in pending {
                let found = resources_of(overlay, &contact);
                if (contact.time_key.clone(), contact.point_key.clone()) == time_point && resources.intersection(&found).next().is_some() {
                    resources.extend(found);
                    component.push(contact);
                    changed = true;
                } else {
                    rest.push(contact);
                }
            }
            pending = rest;
        }
        result.push(identity_sorted(&component));
    }
    result
}

/// `_valid_endpoint(overlay, key)`: both junctions alive, and the endpoint is an end of a leaf of the family.
fn valid_endpoint(overlay: &Overlay, contact: &JunctionContact) -> bool {
    let Some(key) = &contact.endpoint else {
        return false;
    };
    let (Some(emitter), Some(endpoint)) = (overlay.vertices.get(&key.emitter), overlay.vertices.get(&key.endpoint)) else {
        return false;
    };
    emitter.alive && endpoint.alive && overlay.spans.iter().any(|(leaf, binding)| leaf.family() == key.family && (binding.start.as_ref() == Some(&key.endpoint) || binding.end.as_ref() == Some(&key.endpoint)))
}

/// `_ray(builder, overlay, leaf, incoming=...)`: the primitive direction of the carrier line of a leaf, reversed for an incoming arm.
fn ray(builder: &Builder, overlay: &Overlay, leaf: &Leaf, incoming: bool) -> SkelResult<(i64, i64)> {
    let binding = overlay.spans.get(leaf).ok_or_else(|| SkelError::Unsupported("KeyError in the oracle: a leaf that is not in the overlay".to_string()))?;
    let found = direction(&builder.edge_at(binding.physical_edge_id)?.line)?;
    Ok(if incoming { (-found.0, -found.1) } else { found })
}

type Arm = (Option<JRef>, Leaf, JRef);

fn arm_port(arm: &Arm) -> (Option<JRef>, Leaf) {
    (arm.0.clone(), arm.1.clone())
}

/// `_multi_delta(builder, overlay, component)`: one delta for a component of junction contacts: the terminal 0/0 or 1/1 of `normalize_dead_component`, the cross pairing of two
/// dead ports, or the pairing by rays of three or more; any other shape is named as ambiguous.
fn multi_delta(builder: &Builder, overlay: &Overlay, component: &[JunctionContact]) -> SkelResult<(Option<Delta>, Option<&'static str>)> {
    const AMBIGUOUS: &str = "SYMBOLIC_JUNCTION_PORT_MATCHING_AMBIGUOUS";
    let mut dead: Vec<JRef> = Vec::new();
    for reference in component.iter().flat_map(|item| item.dead_refs.iter()) {
        if !dead.contains(reference) {
            dead.push(reference.clone());
        }
    }
    if dead.iter().any(|reference| !overlay.vertices.get(reference).is_some_and(|vertex| vertex.alive)) {
        return Ok((None, Some("SYMBOLIC_JUNCTION_CONTACT_STALE")));
    }
    let dead = by_repr(&dead, |reference| reference.val());
    let outside = |reference: &Option<JRef>| reference.as_ref().is_none_or(|found| !dead.contains(found));
    let (mut incoming, mut outgoing): (Vec<Arm>, Vec<Arm>) = (Vec::new(), Vec::new());
    for reference in &dead {
        let Some(vertex) = overlay.vertices.get(reference) else {
            continue;
        };
        if outside(&vertex.prev) {
            incoming.push((vertex.prev.clone(), vertex.prev_leaf.clone(), reference.clone()));
        }
        if outside(&vertex.next) {
            outgoing.push((vertex.next.clone(), vertex.next_leaf.clone(), reference.clone()));
        }
    }
    let unknown = |arm: &Arm| arm.0.as_ref().is_none_or(|found| !overlay.vertices.get(found).is_some_and(|vertex| vertex.alive));
    if incoming.iter().chain(outgoing.iter()).any(unknown) {
        return Ok((None, Some(AMBIGUOUS)));
    }
    let identities: Vec<Val> = by_repr(&component.iter().map(|item| item.identity.clone()).collect::<Vec<Val>>(), |value| value);
    let point_key = &component[0].point_key;
    if matches!((incoming.len(), outgoing.len()), (0, 0) | (1, 1)) {
        return Ok(normalize_dead_component(overlay, &identities, &dead, point_key, "JUNCTION", "SYMBOLIC_JUNCTION_CONTACT_STALE", AMBIGUOUS));
    }
    if incoming.len() != outgoing.len() {
        return Ok((None, Some(AMBIGUOUS)));
    }
    let mut pairs: Vec<(Arm, Arm)> = Vec::new();
    if dead.len() == 2 && incoming.len() == 2 {
        let mut by_dead: Vec<&Arm> = Vec::new();
        for arm in &outgoing {
            if !by_dead.iter().any(|known| known.2 == arm.2) {
                by_dead.push(arm);
            }
        }
        if by_dead.len() != 2 {
            return Ok((None, Some(AMBIGUOUS)));
        }
        for arm in &incoming {
            let peer = dead.iter().find(|reference| **reference != arm.2).ok_or_else(|| SkelError::Unsupported("StopIteration in the oracle: a pair of dead ports without a peer".to_string()))?;
            let partner = by_dead.iter().find(|known| known.2 == *peer).ok_or_else(|| SkelError::Unsupported("KeyError in the oracle: a dead port without an outgoing arm".to_string()))?;
            pairs.push((arm.clone(), (*partner).clone()));
        }
    } else {
        let mut incoming_by_ray: Vec<((i64, i64), Vec<Arm>)> = Vec::new();
        let mut outgoing_by_ray: Vec<((i64, i64), Vec<Arm>)> = Vec::new();
        for (arms, table, is_incoming) in [(&incoming, &mut incoming_by_ray, true), (&outgoing, &mut outgoing_by_ray, false)] {
            for arm in arms {
                let found = ray(builder, overlay, &arm.1, is_incoming)?;
                match table.iter_mut().find(|(known, _)| *known == found) {
                    Some((_, group)) => group.push(arm.clone()),
                    None => table.push((found, vec![arm.clone()])),
                }
            }
        }
        if incoming_by_ray.iter().chain(outgoing_by_ray.iter()).any(|(_, group)| group.len() != 1) {
            return Ok((None, Some(AMBIGUOUS)));
        }
        let mut common: Vec<(i64, i64)> = incoming_by_ray.iter().map(|(found, _)| *found).filter(|found| outgoing_by_ray.iter().any(|(known, _)| known == found)).collect();
        common.sort_unstable();
        for found in common {
            let from_incoming = incoming_by_ray.remove(incoming_by_ray.iter().position(|(known, _)| *known == found).unwrap_or(0));
            let from_outgoing = outgoing_by_ray.remove(outgoing_by_ray.iter().position(|(known, _)| *known == found).unwrap_or(0));
            pairs.push((from_incoming.1[0].clone(), from_outgoing.1[0].clone()));
        }
        let rest_in: Vec<Arm> = incoming_by_ray.into_iter().flat_map(|(_, group)| group).collect();
        let rest_out: Vec<Arm> = outgoing_by_ray.into_iter().flat_map(|(_, group)| group).collect();
        if rest_in.len() != rest_out.len() || rest_in.len() > 1 {
            return Ok((None, Some(AMBIGUOUS)));
        }
        if let (Some(first), Some(second)) = (rest_in.first(), rest_out.first()) {
            pairs.push((first.clone(), second.clone()));
        }
    }
    let keys = Val::tuple(identities.clone());
    let mut rewires: Vec<((Option<JRef>, Leaf), (Option<JRef>, Leaf), JRef)> = pairs
        .iter()
        .map(|(first, second)| (arm_port(first), arm_port(second), JRef::new("JUNCTION", Val::tuple(vec![keys.clone(), first.1.val().clone(), second.1.val().clone()]))))
        .collect();
    let text = |item: &((Option<JRef>, Leaf), (Option<JRef>, Leaf), JRef)| {
        let port = |found: &(Option<JRef>, Leaf)| Val::tuple(vec![found.0.as_ref().map_or_else(Val::none, |reference| reference.val().clone()), found.1.val().clone()]);
        Val::tuple(vec![port(&item.0), port(&item.1), item.2.val().clone()]).repr()
    };
    rewires.sort_by_cached_key(|item| text(item).to_string());
    let leaf_values: Vec<Val> = dead.iter().filter_map(|reference| overlay.vertices.get(reference)).flat_map(|vertex| [vertex.prev_leaf.val().clone(), vertex.next_leaf.val().clone()]).collect();
    let leaf_resources: Vec<Leaf> = Val::set(leaf_values, true).set_items().unwrap_or(&[]).iter().map(Leaf::from_val).collect::<SkelResult<Vec<_>>>()?;
    Ok((
        Some(Delta { contact_keys: identities, dead_refs: dead, incoming: None, outgoing: None, birth_ref: None, point_key: point_key.clone(), leaf_resources, rewires }),
        None,
    ))
}

/// `SymbolicJunctionGenerationV1(contacts, deltas)`.
#[derive(Clone)]
pub struct JunctionGeneration {
    pub contacts: Vec<JunctionContact>,
    pub deltas: Vec<Delta>,
}

/// `normalize_junction_generation(builder, overlay, contacts, generation_type)`: one delta per component of the contacts, or the named reason (a stale contact, two contacts of
/// one identity, two components that kill one junction).
pub fn normalize_junction_generation(builder: &Builder, overlay: &Overlay, contacts: &[JunctionContact]) -> SkelResult<(Option<JunctionGeneration>, Option<&'static str>)> {
    let mut unique: OrderedMap<Val, JunctionContact> = OrderedMap::new();
    for contact in contacts {
        unique.insert(contact.identity.clone(), contact.clone());
    }
    if unique.len() != contacts.len() {
        return Ok((None, Some("SYMBOLIC_JUNCTION_DUPLICATE_CONTACT")));
    }
    let ordered = identity_sorted(&unique.values().cloned().collect::<Vec<_>>());
    let mut deltas: Vec<Delta> = Vec::new();
    let mut occupied: HashSet<JRef> = HashSet::new();
    for component in components(overlay, &ordered) {
        let stale = component.iter().any(|item| match item.kind {
            ContactKind::Edge => !item.edge.as_ref().is_some_and(|found| valid_edge_contact(overlay, found)),
            ContactKind::Endpoint => !valid_endpoint(overlay, item),
        });
        if stale {
            return Ok((None, Some("SYMBOLIC_JUNCTION_CONTACT_STALE")));
        }
        let (delta, reason) = multi_delta(builder, overlay, &component)?;
        if let Some(reason) = reason {
            return Ok((None, Some(reason)));
        }
        let Some(delta) = delta else {
            return Ok((None, Some("SYMBOLIC_JUNCTION_CONTACT_STALE")));
        };
        if delta.dead_refs.iter().any(|reference| occupied.contains(reference)) {
            return Ok((None, Some("SYMBOLIC_JUNCTION_COMPONENT_DELTAS_OVERLAP")));
        }
        occupied.extend(delta.dead_refs.iter().cloned());
        deltas.push(delta);
    }
    deltas.sort_by_cached_key(|item| Val::tuple(item.contact_keys.clone()).repr().to_string());
    Ok((Some(JunctionGeneration { contacts: ordered, deltas }), None))
}

// --------------------------------------------------------------------------
// the mixed generation
// --------------------------------------------------------------------------

/// `SymbolicMixedGenerationV1(junction_contacts, interior_contacts, deltas)`.
#[derive(Clone)]
pub struct MixedGeneration {
    pub junction_contacts: Vec<JunctionContact>,
    pub interior_contacts: Vec<SymSplitContact>,
    pub deltas: Vec<Delta>,
}

impl PlanVal for MixedGeneration {
    fn to_val(&self) -> Val {
        Val::data(
            "SymbolicMixedGenerationV1",
            vec![
                ("junction_contacts", Val::tuple(self.junction_contacts.iter().map(PlanVal::to_val).collect())),
                ("interior_contacts", Val::tuple(self.interior_contacts.iter().map(PlanVal::to_val).collect())),
                ("deltas", Val::tuple(self.deltas.iter().map(PlanVal::to_val).collect())),
            ],
        )
    }
}

/// What `_expand_target_leaves` answers: the overlay with every cut leaf replaced by its pieces, and the two pieces on each side of every contact.
type Expanded = (Overlay, HashMap<Val, (Leaf, Leaf), FxBuild>);

/// `_expand_target_leaves(overlay, contacts, budget)`: each leaf the interior contacts cut is replaced by the pieces between the contacts, taken in the order of their
/// projections (paid signs); the named reason when a contact has no leaf, a leaf is stale, two contacts are one, or two cut at one point.
fn expand_target_leaves(ctx: &mut ExactCtx<'_>, overlay: &Overlay, contacts: &[SymSplitContact]) -> SkelResult<Result<Expanded, &'static str>> {
    let mut result = clone_overlay(overlay);
    let mut children_by_contact: HashMap<Val, (Leaf, Leaf), FxBuild> = HashMap::default();
    let mut grouped: OrderedMap<Leaf, Vec<SymSplitContact>> = OrderedMap::new();
    for contact in contacts {
        let Some(leaf) = &contact.leaf else {
            return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_LEAF_UNAVAILABLE"));
        };
        match grouped.get_mut(leaf) {
            Some(group) => group.push(contact.clone()),
            None => grouped.insert(leaf.clone(), vec![contact.clone()]),
        }
    }
    let mut leaves: Vec<(&Leaf, &Vec<SymSplitContact>)> = grouped.iter().collect();
    leaves.sort_by(|left, right| repr_order(left.0.val(), right.0.val()));
    for (leaf, group) in leaves {
        let binding = match result.spans.get(leaf) {
            Some(found) if found.start.is_some() && found.end.is_some() => found.clone(),
            _ => return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_TARGET_STALE")),
        };
        let ordered = sort_by_projection(ctx, group, |contact| (&contact.key.val, &contact.projection))?;
        let distinct: HashSet<&Val> = ordered.iter().map(|item| &item.key.val).collect();
        if distinct.len() != ordered.len() {
            return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_CONTACT_DUPLICATE"));
        }
        if ordered.windows(2).any(|pair| pair[0].key.point_key == pair[1].key.point_key) {
            return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_POINT_MULTIPLICITY_UNRESOLVABLE"));
        }
        let occurrence = leaf.occurrence();
        let part = |index: usize| occurrence.get(index).cloned().unwrap_or_else(Val::none);
        let mut boundaries = vec![part(1)];
        boundaries.extend(ordered.iter().map(|item| item.key.point_key.clone()));
        boundaries.push(part(2));
        let mut keys = vec![leaf.start()];
        keys.extend(ordered.iter().map(|item| item.key.val.clone()));
        keys.push(leaf.end());
        let children: Vec<Leaf> = (0..=ordered.len())
            .map(|index| Leaf::new(leaf.family(), keys[index].clone(), keys[index + 1].clone(), Val::tuple(vec![part(0), boundaries[index].clone(), boundaries[index + 1].clone()])))
            .collect();
        let distinct_children: HashSet<&Leaf> = children.iter().collect();
        if distinct_children.len() != children.len() {
            return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_SEGMENT_COLLISION"));
        }
        result.spans.remove(leaf);
        for child in &children {
            result.spans.insert(child.clone(), crate::overlay::Binding { leaf: child.clone(), physical_edge_id: binding.physical_edge_id, start: None, end: None });
        }
        let (Some(start_ref), Some(end_ref)) = (binding.start.as_ref(), binding.end.as_ref()) else {
            return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_TARGET_STALE"));
        };
        let alive = |reference: &JRef| result.vertices.get(reference).is_some_and(|vertex| vertex.alive);
        if !alive(start_ref) || !alive(end_ref) {
            return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_TARGET_STALE"));
        }
        if let Some(start) = result.vertices.get_mut(start_ref) {
            start.next_leaf = children[0].clone();
        }
        if let Some(end) = result.vertices.get_mut(end_ref) {
            end.prev_leaf = children[children.len() - 1].clone();
        }
        for (index, contact) in ordered.iter().enumerate() {
            children_by_contact.insert(contact.key.val.clone(), (children[index].clone(), children[index + 1].clone()));
        }
        result.changed.remove(leaf);
        result.changed.extend(children);
    }
    Ok(Ok((result, children_by_contact)))
}

/// `_interior_deltas(overlay, contacts, children)`: for every interior contact, in the order of the `repr` of its key, the emitter dies and two junctions are born: the one that
/// joins the emitter's incoming arm to the piece after the contact, the one that joins the piece before it to the emitter's outgoing arm.
fn interior_deltas(overlay: &Overlay, contacts: &[SymSplitContact], children: &HashMap<Val, (Leaf, Leaf), FxBuild>) -> SkelResult<Result<Vec<Delta>, &'static str>> {
    let mut deltas = Vec::new();
    let mut ordered: Vec<&SymSplitContact> = contacts.iter().collect();
    ordered.sort_by(|left, right| repr_order(&left.key.val, &right.key.val));
    for contact in ordered {
        let Some(emitter) = overlay.vertices.get(&contact.key.emitter).filter(|vertex| vertex.alive) else {
            return Ok(Err("SYMBOLIC_INTERIOR_SPLIT_EMITTER_STALE"));
        };
        let (before, after) = children.get(&contact.key.val).ok_or_else(|| SkelError::Unsupported("KeyError in the oracle: a contact without its pieces".to_string()))?;
        let first = JRef::new("INTERIOR_SPLIT", Val::tuple(vec![contact.key.val.clone(), Val::str("EMITTER_TO_TARGET"), emitter.prev_leaf.val().clone(), after.val().clone()]));
        let second = JRef::new("INTERIOR_SPLIT", Val::tuple(vec![contact.key.val.clone(), Val::str("TARGET_TO_EMITTER"), before.val().clone(), emitter.next_leaf.val().clone()]));
        let leaf_values = vec![emitter.prev_leaf.val().clone(), emitter.next_leaf.val().clone(), before.val().clone(), after.val().clone()];
        deltas.push(Delta {
            contact_keys: vec![contact.key.val.clone()],
            dead_refs: vec![emitter.reference.clone()],
            incoming: None,
            outgoing: None,
            birth_ref: None,
            point_key: contact.key.point_key.clone(),
            leaf_resources: Val::set(leaf_values, true).set_items().unwrap_or(&[]).iter().map(Leaf::from_val).collect::<SkelResult<Vec<_>>>()?,
            rewires: vec![
                ((emitter.prev.clone(), emitter.prev_leaf.clone()), (None, after.clone()), first),
                ((None, before.clone()), (emitter.next.clone(), emitter.next_leaf.clone()), second),
            ],
        });
    }
    Ok(Ok(deltas))
}

/// `_translated_junction_contacts(overlay, contacts)`: an edge contact carries the leaves its two junctions have in THIS overlay (the leaves that interior cuts of the same
/// generation may have replaced).
fn translated_junction_contacts(overlay: &Overlay, contacts: &[JunctionContact]) -> Result<Vec<JunctionContact>, &'static str> {
    let mut translated = Vec::with_capacity(contacts.len());
    for contact in contacts {
        let Some(edge) = contact.edge.as_ref().filter(|_| contact.kind == ContactKind::Edge) else {
            translated.push(contact.clone());
            continue;
        };
        let (Some(start), Some(end)) = (overlay.vertices.get(&edge.key.start), overlay.vertices.get(&edge.key.end)) else {
            return Err("SYMBOLIC_JUNCTION_CONTACT_STALE");
        };
        let mut moved = contact.clone();
        let mut moved_edge = edge.clone();
        moved_edge.prev_leaf = start.prev_leaf.clone();
        moved_edge.shared_leaf = start.next_leaf.clone();
        moved_edge.next_leaf = end.next_leaf.clone();
        moved.edge = Some(moved_edge);
        translated.push(moved);
    }
    Ok(translated)
}

/// `normalize_mixed_generation(builder, overlay, junction, interior)`: the overlay with the interior leaves cut, and the generation (every delta in the order of the `repr` of its
/// contact keys); the named reason otherwise.
pub fn normalize_mixed_generation(
    ctx: &mut ExactCtx<'_>,
    builder: &Builder,
    overlay: &Overlay,
    junction: &[JunctionContact],
    interior: &[SymSplitContact],
) -> SkelResult<Result<(Overlay, MixedGeneration), &'static str>> {
    let (expanded, children) = match expand_target_leaves(ctx, overlay, interior)? {
        Ok(found) => found,
        Err(reason) => return Ok(Err(reason)),
    };
    let interior_deltas = match interior_deltas(&expanded, interior, &children)? {
        Ok(found) => found,
        Err(reason) => return Ok(Err(reason)),
    };
    let translated = match translated_junction_contacts(&expanded, junction) {
        Ok(found) => found,
        Err(reason) => return Ok(Err(reason)),
    };
    let mut junction_deltas = Vec::new();
    if !translated.is_empty() {
        let (normalized, reason) = normalize_junction_generation(builder, &expanded, &translated)?;
        if let Some(reason) = reason {
            return Ok(Err(reason));
        }
        junction_deltas = normalized.map(|found| found.deltas).unwrap_or_default();
    }
    let mut deltas: Vec<Delta> = junction_deltas;
    deltas.extend(interior_deltas);
    deltas.sort_by_cached_key(|item| Val::tuple(item.contact_keys.clone()).repr().to_string());
    let mut interior_sorted = interior.to_vec();
    interior_sorted.sort_by(|left, right| repr_order(&left.key.val, &right.key.val));
    Ok(Ok((expanded, MixedGeneration { junction_contacts: translated, interior_contacts: interior_sorted, deltas })))
}

/// `apply_mixed_generation(expanded, generation)`.
pub fn apply_mixed_generation(expanded: &Overlay, generation: &MixedGeneration) -> SkelResult<(Option<Overlay>, Option<&'static str>)> {
    apply_component_deltas(expanded, &generation.deltas, "SYMBOLIC_MIXED_COMPONENT_DELTAS_OVERLAP")
}

/// `SymbolicJunctionFixedPointV1(generations, overlay, signatures, unresolved_reason)`.
#[derive(Clone)]
pub struct JunctionFixedPoint {
    pub generations: Vec<MixedGeneration>,
    pub overlay: Option<Overlay>,
    pub signatures: Vec<Val>,
    pub unresolved_reason: Option<&'static str>,
}

/// `plan_mixed_generations(builder, initial, discover_interior, budget=...)`: the causal chain of generations of an overlay until it holds no contact; the interior contacts of
/// every generation of the chain as the second answer (none unless the chain closed). Every round replays the chain from the initial overlay.
pub fn plan_mixed_generations(ctx: &mut ExactCtx<'_>, builder: &mut Builder, initial: &Overlay, budget: i64, memo: &mut SignatureMemo) -> SkelResult<(JunctionFixedPoint, Vec<SymSplitContact>)> {
    let mut causal: Vec<(Vec<JunctionContact>, Vec<SymSplitContact>)> = Vec::new();
    let refused = |generations: Vec<MixedGeneration>, overlay: Option<Overlay>, signatures: Vec<Val>, reason: &'static str| {
        (JunctionFixedPoint { generations, overlay, signatures, unresolved_reason: Some(reason) }, Vec::new())
    };
    for iteration in 0..=budget {
        let mut overlay = clone_overlay(initial);
        let (mut generations, mut signatures): (Vec<MixedGeneration>, Vec<Val>) = (Vec::new(), Vec::new());
        for (junction, interior) in &causal {
            let (expanded, generation) = match normalize_mixed_generation(ctx, builder, &overlay, junction, interior)? {
                Ok(found) => found,
                Err(reason) => return Ok(refused(generations, None, signatures, reason)),
            };
            let (applied, reason) = apply_mixed_generation(&expanded, &generation)?;
            if let Some(reason) = reason {
                return Ok(refused(generations, None, signatures, reason));
            }
            overlay = applied.ok_or_else(|| SkelError::Unsupported("a generation applied without an overlay".to_string()))?;
            generations.push(generation);
            signatures.push(overlay_signature(&overlay, memo)?);
        }
        let (junction, reason) = discover_junction_contacts(ctx, builder, &overlay)?;
        if let Some(reason) = reason {
            return Ok(refused(generations, Some(overlay), signatures, reason));
        }
        let (interior, reason) = discover_interior_split_contacts(ctx, builder, &overlay)?;
        if let Some(reason) = reason {
            return Ok(refused(generations, Some(overlay), signatures, reason));
        }
        if junction.is_empty() && interior.is_empty() {
            let later: Vec<SymSplitContact> = causal.iter().flat_map(|(_, batch)| batch.iter().cloned()).collect();
            return Ok((JunctionFixedPoint { generations, overlay: Some(overlay), signatures, unresolved_reason: None }, later));
        }
        let (interior, reason) = merge_symbolic_split_contacts(&[&interior]);
        if let Some(reason) = reason {
            return Ok(refused(generations, Some(overlay), signatures, reason));
        }
        if iteration == budget {
            return Ok(refused(generations, Some(overlay), signatures, "SYMBOLIC_MIXED_GENERATION_BUDGET_EXHAUSTED"));
        }
        causal.push((junction, interior));
    }
    Err(SkelError::Unsupported("AssertionError in the oracle: unreachable mixed symbolic generation loop".to_string()))
}

/// `repr` of a fixed point as the seams compare it.
pub fn fixed_point_val(found: &JunctionFixedPoint) -> Val {
    Val::data(
        "SymbolicJunctionFixedPointV1",
        vec![
            ("generations", Val::tuple(found.generations.iter().map(PlanVal::to_val).collect())),
            ("overlay", found.overlay.as_ref().map_or_else(Val::none, crate::overlay::overlay_val)),
            ("signatures", Val::tuple(found.signatures.clone())),
            ("unresolved_reason", found.unresolved_reason.map_or_else(Val::none, Val::str)),
        ],
    )
}
