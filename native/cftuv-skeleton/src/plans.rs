//! The plan of a packet (`wavefront/superlevel.py`, the plan half): from a frozen snapshot to one delta per resource-connected component of contacts, without touching a
//! runtime object.
//!
//! A packet is split into components (transitive closure over shared vertices and edge occurrences), and each component is planned by three paths that must agree on one
//! frozen prestate: the DEATH of the ports of colliding edges (`edge_contact_plans`), the RECONNECTION of vertices that met (`meeting_plans`) and the CUT of a span
//! by a reflex vertex (`split_cut_plans`), composed where the proof says they overlap (`composition`). Their births are wired into one partial bijection of ports
//! (`wire_births`) and decomposed into chains and cycles. Every path that cannot prove its case ends in the named resolution `UNRESOLVABLE`, never in a guess.
//!
//! The order of everything is the oracle's: dictionaries keep their insertion order, a set that is iterated is sorted by what the oracle sorts it by (`repr` of the key,
//! the tuple of an incident), a comparator sort is CPython 3.11's with its signs paid for (`split_cut_plans` orders the cuts of a span by a SIGN of the difference of their
//! projections), and a sort by a key that Python cannot order (a `None` against a tuple) is the oracle's `TypeError`, answered as the refusal [`UnorderedPair::refusal`].
//!
//! Plans carry [`Val`]s for keys: births, occurrences and points are compared by value, hashed and printed as the oracle's tuples are. [`PlanVal`] turns a plan into the
//! `repr` the oracle's dataclass would print, which the seams compare byte for byte.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::{SkelError, SkelResult};
use crate::germ::{absorbed_by_locus, locus_ends, GermLedger};
use crate::proof::EdgeKey;
use crate::pyset::PySet;
use crate::pyval::{identity_order_val, sorted_by_repr, sorted_by_val_or_refuse, Val};
use crate::queue::{CandidateEvent, EventKind};
use crate::snapshot::{exact_point_key, Incident, IncidentSortKey, Snapshot, VertexSnapshot};
use crate::time::{EventPoint, EventTime, PointRef, TimeRef};

// --------------------------------------------------------------------------
// the plan types
// --------------------------------------------------------------------------

/// `SuperlevelResolution`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Resolution {
    Edge,
    Split,
    BoundaryPortPairing,
    Unresolvable,
}

impl Resolution {
    pub fn value(self) -> &'static str {
        match self {
            Resolution::Edge => "EDGE",
            Resolution::Split => "SPLIT",
            Resolution::BoundaryPortPairing => "BOUNDARY_PORT_PAIRING",
            Resolution::Unresolvable => "UNRESOLVABLE",
        }
    }
}

/// `BoundaryBirthV1`: a contact junction between two geometry-derived edge occurrences; `key` is `(time key, point key, prev occurrence, next occurrence)`.
#[derive(Debug, Clone)]
pub struct BoundaryBirth {
    pub point_key: Val,
    pub prev_occurrence: Val,
    pub next_occurrence: Val,
    pub key: Val,
    pub replaces: Vec<i64>,
}

/// `VertexReferenceV1`: an existing vertex or the symbolic birth of a frozen delta.
#[derive(Debug, Clone)]
pub struct VertexReference {
    pub existing: Option<i64>,
    pub birth_key: Option<Val>,
}

/// `EdgeContactPlanV1`.
#[derive(Debug, Clone)]
pub struct EdgeContactPlan {
    pub events: Vec<CandidateEvent>,
    pub time: TimeRef,
    pub point: PointRef,
    pub point_key: Val,
    pub participants: Vec<EdgeKey>,
    pub dead_vertex_ids: Vec<i64>,
    pub chains: Vec<Vec<i64>>,
    pub births: Vec<BoundaryBirth>,
    pub kinds: Vec<EventKind>,
}

/// `SplitCutPlanV1`.
#[derive(Debug, Clone)]
pub struct SplitCutPlan {
    pub edge_id: i64,
    pub target_occurrence: Val,
    pub events: Vec<CandidateEvent>,
    pub segment_occurrences: Vec<Val>,
    pub births: Vec<BoundaryBirth>,
    /// `(birth key, keep prev, keep next)`.
    pub final_birth_ports: Vec<(Val, bool, bool)>,
}

/// `VertexMeetingPlanV1`.
#[derive(Debug, Clone)]
pub struct VertexMeetingPlan {
    pub events: Vec<CandidateEvent>,
    pub time: TimeRef,
    pub point: PointRef,
    pub meeting_vertex_ids: Vec<i64>,
    pub pairs: Vec<(i64, i64)>,
    pub participants: Vec<EdgeKey>,
    pub births: Vec<BoundaryBirth>,
}

/// `(birth key, predecessor, successor)`.
pub type BirthWire = (Val, VertexReference, VertexReference);

/// `SuperlevelComponentPlanV1`: the pure delta of one resource-connected component.
#[derive(Debug, Clone)]
pub struct ComponentPlan {
    pub event_kinds: Vec<EventKind>,
    pub time: TimeRef,
    pub point_keys: Vec<Val>,
    pub point: PointRef,
    pub resolution: Resolution,
    pub events: Vec<CandidateEvent>,
    pub participants: Vec<EdgeKey>,
    pub target_participants: Vec<EdgeKey>,
    pub dead_vertex_ids: Vec<i64>,
    pub chains: Vec<Vec<i64>>,
    pub births: Vec<BoundaryBirth>,
    pub queue_seed_vertex_ids: Vec<i64>,
    pub enqueue_born_vertices: bool,
    pub edge_contacts: Vec<EdgeContactPlan>,
    pub split_cuts: Vec<SplitCutPlan>,
    pub vertex_meetings: Vec<VertexMeetingPlan>,
    pub proof_fallbacks: Vec<CandidateEvent>,
    pub coincident_split_targets: i64,
    pub closed_chain_count: i64,
    pub suppressed_candidates: i64,
    pub birth_wiring: Vec<BirthWire>,
    /// `(vertex id, prev occurrence, next occurrence)`.
    pub existing_port_rewrites: Vec<(i64, Val, Val)>,
    pub birth_components: Vec<Vec<Val>>,
    pub terminal_birth_cycles: Vec<Vec<Val>>,
}

// --------------------------------------------------------------------------
// births and the ledger
// --------------------------------------------------------------------------

/// `_fold_germ(previous, repeated)`: two presentations of one germ become one object. The load (`replaces`) is the union; the representative of two DIFFERENT keys is
/// the canonical minimum by `repr` (and an ambiguity is for the caller to name, not for the ledger).
pub fn fold_germ(previous: &BoundaryBirth, repeated: &BoundaryBirth) -> BoundaryBirth {
    if previous.key != repeated.key {
        return if previous.key.repr().as_bytes() <= repeated.key.repr().as_bytes() { previous.clone() } else { repeated.clone() };
    }
    let merged: BTreeSet<i64> = previous.replaces.iter().chain(&repeated.replaces).copied().collect();
    BoundaryBirth { replaces: merged.into_iter().collect(), ..previous.clone() }
}

/// `_birth(time, point_key, prev_occurrence, next_occurrence, replaces=..., ledger=...)`: the one door of the materialisation of a contact junction; the runtime id is not
/// in the key. A germ presented again answers the same object.
pub fn birth(
    time: &TimeRef,
    point_key: &Val,
    prev_occurrence: Option<&Val>,
    next_occurrence: Option<&Val>,
    replaces: Vec<i64>,
    ledger: &mut GermLedger,
) -> SkelResult<Option<BoundaryBirth>> {
    let (Some(prev), Some(next)) = (prev_occurrence, next_occurrence) else {
        return Ok(None);
    };
    let time_key = ledger.time_key(time)?;
    let key = Val::tuple(vec![time_key.clone(), point_key.clone(), prev.clone(), next.clone()]);
    let germ = BoundaryBirth { point_key: point_key.clone(), prev_occurrence: prev.clone(), next_occurrence: next.clone(), key, replaces };
    match ledger.key(&time_key, point_key, prev, next) {
        None => Ok(Some(germ)),
        Some(germ_key) => Ok(Some(ledger.materialize(germ_key, germ))),
    }
}

/// THE order of births: the oracle sorts them by `item.order_key` (`superlevel.BoundaryBirthV1.order_key`, `exact_identity.identity_order_key`: the slots in order, a `None` end of an
/// occurrence (a vertex without a place BY the law) AFTER any value of the same slot, the order of keys without a `None` the plain one), never by the bare key, whose `<` between a
/// `None` and a point key is a `TypeError` (oracle commits 3da8cdd, d6b2c49). The one place the port mirrors it is this function and [`port_order_key`].
pub fn birth_order_key(item: &BoundaryBirth) -> Val {
    identity_order_val(&item.key)
}

/// The order of the final birth ports `(birth key, keep prev, keep next)`: the oracle sorts them with `key=identity_order_key` (see [`birth_order_key`]).
pub fn port_order_key(key: &Val, keep_prev: bool, keep_next: bool) -> Val {
    identity_order_val(&Val::tuple(vec![key.clone(), Val::boolean(keep_prev), Val::boolean(keep_next)]))
}

pub fn births_by_key(births: &[BoundaryBirth], site: &str) -> SkelResult<Vec<BoundaryBirth>> {
    sorted_by_val_or_refuse(births, birth_order_key, site)
}

// --------------------------------------------------------------------------
// components
// --------------------------------------------------------------------------

/// `_connected_components(incidents)`: the transitive closure over a shared vertex or edge occurrence; the members of each, by the geometric order of the incident.
/// The oracle builds the members as a `set` of indices and iterates it, and the sort by the geometric key is stable: incidents with EQUAL keys stay in the iteration order of
/// that set, which is the order of its hash table (`pyset`), not the ascending one.
pub fn connected_components(incidents: &[Incident]) -> Vec<Vec<&Incident>> {
    let mut pending = PySet::from_range(incidents.len());
    let mut components = Vec::new();
    while let Some(seed) = pending.min() {
        let mut component = PySet::new();
        component.add(seed);
        let mut vertices: HashSet<i64> = incidents[seed].vertex_ids.iter().copied().collect();
        let mut edges: HashSet<i64> = incidents[seed].edge_occurrences.iter().copied().collect();
        loop {
            let mut joined = PySet::new();
            for index in pending.difference(&component).iter() {
                if incidents[index].vertex_ids.iter().any(|ident| vertices.contains(ident)) || incidents[index].edge_occurrences.iter().any(|ident| edges.contains(ident)) {
                    joined.add(index);
                }
            }
            let changed = !joined.is_empty();
            component.update(&joined);
            for index in joined.iter() {
                vertices.extend(incidents[index].vertex_ids.iter().copied());
                edges.extend(incidents[index].edge_occurrences.iter().copied());
            }
            if !changed {
                break;
            }
        }
        pending.difference_update(&component);
        let mut members: Vec<&Incident> = component.iter().map(|index| &incidents[index]).collect();
        members.sort_by(|left, right| left.sort_key().cmp(right.sort_key()));
        components.push(members);
    }
    components
}

/// `_snapshot_chains(idents, vertices)`: the connected runs of the dead vertices of the frozen LAV (runtime ids stay inside).
pub fn snapshot_chains(idents: &BTreeSet<i64>, vertices: &[VertexSnapshot]) -> Vec<Vec<i64>> {
    let live: BTreeSet<i64> = idents.iter().copied().filter(|ident| vertices[*ident as usize].alive).collect();
    let mut chains = Vec::new();
    let mut seen: BTreeSet<i64> = BTreeSet::new();
    for ident in &live {
        if seen.contains(ident) {
            continue;
        }
        let mut start = *ident;
        let mut guard = 0usize;
        while live.contains(&vertices[start as usize].prev) && vertices[start as usize].prev != *ident && guard <= live.len() {
            start = vertices[start as usize].prev;
            guard += 1;
        }
        let mut chain = vec![start];
        seen.insert(start);
        let mut cursor = vertices[start as usize].next;
        while live.contains(&cursor) && !seen.contains(&cursor) {
            chain.push(cursor);
            seen.insert(cursor);
            cursor = vertices[cursor as usize].next;
        }
        chains.push(chain);
    }
    chains
}

/// The fields common to every resolution (`_component_fields`).
pub struct ComponentFields {
    pub event_kinds: Vec<EventKind>,
    pub time: TimeRef,
    pub point_keys: Vec<Val>,
    pub point: PointRef,
    pub events: Vec<CandidateEvent>,
    pub participants: Vec<EdgeKey>,
    pub target_participants: Vec<EdgeKey>,
}

fn sorted_kinds(kinds: impl IntoIterator<Item = EventKind>) -> Vec<EventKind> {
    let mut found: Vec<EventKind> = kinds.into_iter().collect();
    found.sort_by_key(|kind| kind.value());
    found.dedup();
    found
}

fn sorted_unique<T: Ord>(items: impl IntoIterator<Item = T>) -> Vec<T> {
    items.into_iter().collect::<BTreeSet<T>>().into_iter().collect()
}

/// `_component_fields(component)`.
pub fn component_fields(component: &[&Incident]) -> ComponentFields {
    let point_keys: Vec<Val> = {
        let mut seen: HashSet<Val> = HashSet::new();
        let unique: Vec<Val> = component.iter().map(|incident| incident.point_key.clone()).filter(|key| seen.insert(key.clone())).collect();
        sorted_by_repr(&unique, Val::clone)
    };
    // `min(component, key=repr(point_key))`: the FIRST of the equal keys
    let sample = component.iter().skip(1).fold(component[0], |best, incident| if incident.point_key.repr().as_bytes() < best.point_key.repr().as_bytes() { incident } else { best });
    ComponentFields {
        event_kinds: sorted_kinds(component.iter().map(|incident| incident.event.kind)),
        time: Rc::clone(&sample.event.time),
        point_keys,
        point: Rc::clone(&sample.event.point),
        events: component.iter().map(|incident| incident.event.clone()).collect(),
        participants: sorted_unique(component.iter().flat_map(|incident| incident.participants.iter().cloned())),
        target_participants: sorted_unique(component.iter().flat_map(|incident| incident.target_participants.iter().cloned())),
    }
}

// --------------------------------------------------------------------------
// the three paths
// --------------------------------------------------------------------------

/// Groups in the order of first insertion, each under a value key (a Python `dict` of lists).
struct Grouped<'a, T> {
    slots: HashMap<Val, usize, FxBuild>,
    groups: Vec<(Val, Vec<&'a T>)>,
}

impl<'a, T> Grouped<'a, T> {
    fn new() -> Grouped<'a, T> {
        Grouped { slots: HashMap::default(), groups: Vec::new() }
    }

    fn push(&mut self, key: &Val, item: &'a T) {
        match self.slots.get(key) {
            Some(slot) => self.groups[*slot].1.push(item),
            None => {
                self.slots.insert(key.clone(), self.groups.len());
                self.groups.push((key.clone(), vec![item]));
            }
        }
    }

    /// The keys sorted by `repr` (`sorted(grouped, key=repr)`), each with its group.
    fn by_repr(&self) -> Vec<(&Val, &Vec<&'a T>)> {
        let mut found: Vec<(&Val, &Vec<&'a T>)> = self.groups.iter().map(|(key, group)| (key, group)).collect();
        found.sort_by(|left, right| left.0.repr().as_bytes().cmp(right.0.repr().as_bytes()));
        found
    }
}

fn sorted_incidents<'a>(group: &[&'a Incident]) -> Vec<&'a Incident> {
    let mut sorted = group.to_vec();
    sorted.sort_by(|left, right| left.sort_key().cmp(right.sort_key()));
    sorted
}

fn participants_of(incidents: &[&Incident]) -> Vec<EdgeKey> {
    sorted_unique(incidents.iter().flat_map(|incident| incident.participants.iter().cloned()))
}

/// What `_edge_contact_plans` answers: the contacts, the splits that no contact absorbed, and whether every contact was provable.
pub type EdgeContacts<'a> = (Vec<EdgeContactPlan>, Vec<&'a Incident>, bool);

/// `_edge_contact_plans(edges, splits, vertices, ledger)`: the death of the ports of colliding edges and the births at their ends; a split whose ends all lie in the locus
/// the contact occupies is the SAME locus and is absorbed by the contact.
pub fn edge_contact_plans<'a>(edges: &[&'a Incident], splits: &[&'a Incident], vertices: &[VertexSnapshot], ledger: &mut GermLedger) -> SkelResult<EdgeContacts<'a>> {
    let mut grouped: Grouped<'_, Incident> = Grouped::new();
    for incident in edges.iter().copied() {
        grouped.push(&incident.point_key, incident);
    }
    let mut contacts = Vec::new();
    let mut absorbed: HashSet<crate::snapshot::EventIdentity> = HashSet::new();
    let mut valid = true;
    for (point_key, group) in grouped.by_repr() {
        let local_edges = sorted_incidents(group);
        let dead: BTreeSet<i64> = local_edges.iter().flat_map(|incident| [incident.event.vertex, incident.event.peer]).collect();
        let chains = snapshot_chains(&dead, vertices);
        let occupied = locus_ends(&dead, vertices);
        let mut local_splits: Vec<&Incident> = Vec::new();
        for incident in splits {
            if incident.point_key == *point_key && dead.contains(&incident.event.vertex) && absorbed_by_locus(incident, vertices, &occupied) && !local_splits.iter().any(|seen| seen.same(incident)) {
                local_splits.push(incident);
            }
        }
        absorbed.extend(local_splits.iter().map(|incident| incident.identity().clone()));
        if !local_splits.is_empty() && chains.len() != 1 {
            valid = false;
        }
        let mut births = Vec::new();
        for chain in &chains {
            let (head, tail) = (&vertices[chain[0] as usize], &vertices[chain[chain.len() - 1] as usize]);
            if vertices[tail.next as usize].ident == head.ident {
                continue;
            }
            match birth(&local_edges[0].event.time, point_key, head.prev_occurrence.as_ref(), tail.next_occurrence.as_ref(), chain.clone(), ledger)? {
                None => valid = false,
                Some(found) => births.push(found),
            }
        }
        let every: Vec<&Incident> = local_edges.iter().chain(local_splits.iter()).copied().collect();
        contacts.push(EdgeContactPlan {
            events: every.iter().map(|incident| incident.event.clone()).collect(),
            time: Rc::clone(&local_edges[0].event.time),
            point: Rc::clone(&local_edges[0].event.point),
            point_key: point_key.clone(),
            participants: participants_of(&every),
            dead_vertex_ids: dead.iter().copied().collect(),
            chains,
            births: births_by_key(&births, "superlevel._edge_contact_plans")?,
            kinds: sorted_kinds(every.iter().map(|incident| incident.event.kind)),
        });
    }
    let remaining = splits.iter().filter(|incident| !absorbed.contains(incident.identity())).copied().collect();
    Ok((contacts, remaining, valid))
}

/// `_reconnect_snapshot(meeting, vertices)`: how the vertices of a meeting are stitched again. Two non-adjacent ports have the one cross pair; three or more are paired by
/// ray (the incoming ray of one with the outgoing ray of another), with at most one pair left over.
pub fn reconnect_snapshot(meeting: &[i64], vertices: &[VertexSnapshot]) -> Option<Vec<(i64, i64)>> {
    if meeting.len() == 2 {
        return Some(vec![(meeting[0], meeting[1]), (meeting[1], meeting[0])]);
    }
    let mut incoming: Vec<((i64, i64), Vec<i64>)> = Vec::new();
    let mut outgoing: Vec<((i64, i64), Vec<i64>)> = Vec::new();
    let put = |table: &mut Vec<((i64, i64), Vec<i64>)>, ray: (i64, i64), ident: i64| match table.iter_mut().find(|(known, _)| *known == ray) {
        Some((_, group)) => group.push(ident),
        None => table.push((ray, vec![ident])),
    };
    for ident in meeting {
        let vertex = &vertices[*ident as usize];
        put(&mut incoming, vertex.incoming_ray, *ident);
        put(&mut outgoing, vertex.outgoing_ray, *ident);
    }
    if incoming.len() != meeting.len() || outgoing.len() != meeting.len() {
        return None;
    }
    let common: BTreeSet<(i64, i64)> = incoming.iter().map(|(ray, _)| *ray).filter(|ray| outgoing.iter().any(|(known, _)| known == ray)).collect();
    let mut pairs = Vec::new();
    for ray in common {
        let from_incoming = incoming.remove(incoming.iter().position(|(known, _)| *known == ray)?);
        let from_outgoing = outgoing.remove(outgoing.iter().position(|(known, _)| *known == ray)?);
        pairs.push((from_incoming.1[0], from_outgoing.1[0]));
    }
    let rest_in: Vec<i64> = incoming.iter().flat_map(|(_, group)| group.iter().copied()).collect();
    let rest_out: Vec<i64> = outgoing.iter().flat_map(|(_, group)| group.iter().copied()).collect();
    if rest_in.len() != rest_out.len() || rest_in.len() > 1 {
        return None;
    }
    if let (Some(first), Some(second)) = (rest_in.first(), rest_out.first()) {
        pairs.push((*first, *second));
    }
    Some(pairs)
}

/// What `_meeting_plans` answers: the meetings, the incidents that stay cuts, and the events whose meeting could not be reconnected.
pub type Meetings<'a> = (Vec<VertexMeetingPlan>, Vec<&'a Incident>, Vec<CandidateEvent>);

/// `_meeting_plans(splits, vertices, ledger)`: the vertices that met a vertex (not an edge) are reconnected by rays; a meeting that cannot be, or that touches itself,
/// goes back to the cuts and its events are named fallbacks.
pub fn meeting_plans<'a>(splits: &[&'a Incident], vertices: &[VertexSnapshot], ledger: &mut GermLedger) -> SkelResult<Meetings<'a>> {
    let mut grouped: Grouped<'_, Incident> = Grouped::new();
    let (mut cuts, mut fallbacks): (Vec<&Incident>, Vec<CandidateEvent>) = (Vec::new(), Vec::new());
    for incident in splits.iter().copied() {
        match (incident.met_vertex_id, incident.met_adjacent) {
            (None, _) => cuts.push(incident),
            (Some(_), true) => {
                cuts.push(incident);
                fallbacks.push(incident.event.clone());
            }
            (Some(_), false) => grouped.push(&incident.point_key, incident),
        }
    }
    let mut meetings = Vec::new();
    for (point_key, group) in grouped.by_repr() {
        let incidents = sorted_incidents(group);
        let meeting: Vec<i64> = sorted_unique(incidents.iter().map(|incident| incident.event.vertex).chain(incidents.iter().filter_map(|incident| incident.met_vertex_id)));
        let touches_self = meeting.iter().any(|ident| meeting.contains(&vertices[*ident as usize].prev) || meeting.contains(&vertices[*ident as usize].next));
        let pairs = if touches_self { None } else { reconnect_snapshot(&meeting, vertices) };
        let Some(pairs) = pairs else {
            cuts.extend(incidents.iter().copied());
            fallbacks.extend(incidents.iter().map(|incident| incident.event.clone()));
            continue;
        };
        let mut planned = Vec::new();
        for (incoming, outgoing) in &pairs {
            planned.push(birth(
                &incidents[0].event.time,
                point_key,
                vertices[*incoming as usize].prev_occurrence.as_ref(),
                vertices[*outgoing as usize].next_occurrence.as_ref(),
                Vec::new(),
                ledger,
            )?);
        }
        if planned.iter().any(Option::is_none) {
            cuts.extend(incidents.iter().copied());
            fallbacks.extend(incidents.iter().map(|incident| incident.event.clone()));
            continue;
        }
        let planned: Vec<BoundaryBirth> = planned.into_iter().flatten().collect();
        meetings.push(VertexMeetingPlan {
            events: incidents.iter().map(|incident| incident.event.clone()).collect(),
            time: Rc::clone(&incidents[0].event.time),
            point: Rc::clone(&incidents[0].event.point),
            meeting_vertex_ids: meeting,
            pairs,
            participants: participants_of(&incidents),
            births: births_by_key(&planned, "superlevel._meeting_plans")?,
        });
    }
    Ok((meetings, cuts, fallbacks))
}

/// `repr(item)` of a possibly absent value (`repr(None)` is `None`).
fn repr_option(value: &Option<Val>) -> String {
    value.as_ref().map_or_else(|| "None".to_string(), |found| found.repr().to_string())
}

fn repr_projection(incident: &Incident) -> String {
    incident.target_projection.as_ref().map_or_else(|| "None".to_string(), crate::snapshot::sum_repr)
}

/// `_projection_order(left, right, budget)`: the order of two cuts of one span, by the SIGN of the difference of their projections (paid for), then by their geometry.
fn projection_order(ctx: &mut ExactCtx<'_>, left: &Incident, right: &Incident) -> SkelResult<std::cmp::Ordering> {
    let (Some(first), Some(second)) = (&left.target_projection, &right.target_projection) else {
        return Err(SkelError::Unsupported("TypeError in the oracle: a cut without a projection has no order".to_string()));
    };
    let difference = first.sub(second);
    let order = exact::sign(ctx, &difference, SIGN_FILTER_BITS)?;
    if order != 0 {
        return Ok(if order < 0 { std::cmp::Ordering::Less } else { std::cmp::Ordering::Greater });
    }
    let key = |incident: &Incident| (incident.participants.clone(), incident.target_participants.clone(), repr_option(&incident.target_occurrence), incident.emitter_key.repr().to_string());
    Ok(key(left).cmp(&key(right)))
}

/// What `_dedupe_split_incidents` answers: the chosen incident of every emitting vertex, the number dropped, and whether the choice was unambiguous.
pub type Deduped<'a> = (Vec<&'a Incident>, i64, bool);

/// `_dedupe_split_incidents(splits)`: one incident per emitting vertex (the same event repeated, or the same vertex cutting two spans of one geometry, is dropped).
pub fn dedupe_split_incidents<'a>(splits: &[&'a Incident]) -> Deduped<'a> {
    let mut by_vertex: Vec<(i64, Vec<&'a Incident>)> = Vec::new();
    for incident in splits {
        match by_vertex.iter_mut().find(|(vertex, _)| *vertex == incident.event.vertex) {
            Some((_, group)) => group.push(incident),
            None => by_vertex.push((incident.event.vertex, vec![incident])),
        }
    }
    let mut chosen: Vec<&Incident> = Vec::new();
    let mut dropped = 0i64;
    for (_, candidates) in &by_vertex {
        let mut unique: Vec<&Incident> = Vec::new();
        for item in candidates {
            if !unique.iter().any(|seen| seen.identity() == item.identity()) {
                unique.push(item);
            }
        }
        dropped += (candidates.len() - unique.len()) as i64;
        if unique.iter().map(|candidate| &candidate.point_key).collect::<HashSet<&Val>>().len() != 1 {
            return (Vec::new(), 0, false);
        }
        let key = |incident: &Incident| (incident.target_participants.clone(), repr_projection(incident), repr_option(&incident.target_occurrence));
        let mut ordered = unique;
        ordered.sort_by_cached_key(|incident| key(incident));
        if ordered.len() > 1 && key(ordered[0]) == key(ordered[1]) {
            return (Vec::new(), 0, false);
        }
        dropped += ordered.len() as i64 - 1;
        chosen.push(ordered[0]);
    }
    chosen.sort_by(|left, right| left.sort_key().cmp(right.sort_key()));
    (chosen, dropped, true)
}

/// What `_split_cut_plans` answers: the cuts, the number of dropped candidates, whether every cut was provable.
pub type SplitCuts = (Vec<SplitCutPlan>, i64, bool);

/// `_split_cut_plans(splits, vertices, ledger, budget)`: the cut of a span by one or more reflex vertices, the pieces between the cuts and the births at each cut.
/// Only a true interior cut is planned; an end, a zero length, a repeated or an unchanged occurrence stays fail-closed.
pub fn split_cut_plans(ctx: &mut ExactCtx<'_>, splits: &[&Incident], vertices: &[VertexSnapshot], ledger: &mut GermLedger) -> SkelResult<SplitCuts> {
    let (chosen, dropped, valid) = dedupe_split_incidents(splits);
    if !valid {
        return Ok((Vec::new(), 0, false));
    }
    let mut grouped: Grouped<'_, Incident> = Grouped::new();
    for incident in chosen.iter().copied() {
        match &incident.target_occurrence {
            None => return Ok((Vec::new(), 0, false)),
            Some(occurrence) => grouped.push(occurrence, incident),
        }
    }
    let mut plans: Vec<SplitCutPlan> = Vec::new();
    for (target_occurrence, incidents) in grouped.by_repr() {
        let ordered = order_cuts(ctx, incidents)?;
        let first = ordered[0];
        if ordered.iter().any(|incident| incident.event.edge != first.event.edge || incident.target_occurrence.as_ref() != Some(target_occurrence)) {
            return Ok((Vec::new(), 0, false));
        }
        let Some([owner_key, span_start, span_end]) = target_occurrence.items() else {
            return Ok((Vec::new(), 0, false));
        };
        let points: Vec<Val> = ordered.iter().map(|item| item.point_key.clone()).collect();
        let mut lefts = vec![span_start.clone()];
        lefts.extend(points.iter().cloned());
        let mut rights = points.clone();
        rights.push(span_end.clone());
        let segment_occurrences: Vec<Val> = lefts.iter().zip(&rights).map(|(left, right)| Val::tuple(vec![owner_key.clone(), left.clone(), right.clone()])).collect();
        let distinct: HashSet<&Val> = segment_occurrences.iter().collect();
        if lefts.iter().zip(&rights).any(|(left, right)| left == right) || distinct.len() != segment_occurrences.len() || segment_occurrences.contains(target_occurrence) {
            return Ok((Vec::new(), 0, false));
        }
        let (mut births, mut final_ports): (Vec<BoundaryBirth>, Vec<(Val, bool, bool)>) = (Vec::new(), Vec::new());
        for (index, item) in ordered.iter().enumerate() {
            let emitter = &vertices[item.event.vertex as usize];
            let left = birth(&item.event.time, &item.point_key, emitter.prev_occurrence.as_ref(), Some(&segment_occurrences[index + 1]), vec![emitter.ident], ledger)?;
            let right = birth(&item.event.time, &item.point_key, Some(&segment_occurrences[index]), emitter.next_occurrence.as_ref(), vec![emitter.ident], ledger)?;
            let (Some(left), Some(right)) = (left, right) else {
                return Ok((Vec::new(), 0, false));
            };
            final_ports.push((left.key.clone(), false, true));
            final_ports.push((right.key.clone(), true, false));
            births.push(left);
            births.push(right);
        }
        let port_text = |port: &(Val, bool, bool)| Val::tuple(vec![port.0.clone(), Val::boolean(port.1), Val::boolean(port.2)]);
        final_ports.sort_by_cached_key(|port| port_text(port).repr().to_string());
        plans.push(SplitCutPlan {
            edge_id: first.event.edge,
            target_occurrence: target_occurrence.clone(),
            events: ordered.iter().map(|item| item.event.clone()).collect(),
            segment_occurrences,
            births: births_by_key(&births, "superlevel._split_cut_plans")?,
            final_birth_ports: final_ports,
        });
    }
    let order_key = |plan: &SplitCutPlan| {
        let item = chosen.iter().find(|item| item.target_occurrence.as_ref() == Some(&plan.target_occurrence));
        let head = item.map(|item| (item.target_participants.clone(), repr_option(&item.target_occurrence)));
        let tail: Vec<String> = plan.events.iter().map(|event| exact_point_key(&event.point).repr().to_string()).collect();
        (head, tail)
    };
    plans.sort_by_cached_key(order_key);
    Ok((plans, dropped, true))
}

/// `sorted_as_cpython311(incidents, lambda l, r: _projection_order(l, r, budget))`: the cuts of one span in the order of their projections, in CPython 3.11's sequence of
/// questions (each comparison is a sign).
fn order_cuts<'a>(ctx: &mut ExactCtx<'_>, incidents: &[&'a Incident]) -> SkelResult<Vec<&'a Incident>> {
    let mut failure: Option<SkelError> = None;
    let sorted = {
        let mut less = |left: &usize, right: &usize| -> cftuv_clip::error::ClipResult<bool> {
            match projection_order(ctx, incidents[*left], incidents[*right]) {
                Ok(order) => Ok(order == std::cmp::Ordering::Less),
                Err(error) => {
                    failure = Some(error);
                    Err(cftuv_clip::error::ClipError::Value("refused"))
                }
            }
        };
        cftuv_clip::cpython311::sort_by_less((0..incidents.len()).collect(), &mut less)
    };
    match sorted {
        Ok(order) => Ok(order.into_iter().map(|index| incidents[index]).collect()),
        Err(_) => Err(failure.unwrap_or(SkelError::Unsupported("the comparator sort failed without a cause".to_string()))),
    }
}

// --------------------------------------------------------------------------
// wiring the births
// --------------------------------------------------------------------------

fn rewritten_occurrence(occurrence: &Val, rewrites: &HashMap<Val, Val, FxBuild>) -> Val {
    rewrites.get(occurrence).cloned().unwrap_or_else(|| occurrence.clone())
}

/// `_rewrite_birth(birth, prev_rewrites, next_rewrites, keep_prev, keep_next)`: the birth with the occurrences of the cut edge replaced by the pieces.
pub fn rewrite_birth(
    birth: &BoundaryBirth,
    prev_rewrites: &HashMap<Val, Val, FxBuild>,
    next_rewrites: &HashMap<Val, Val, FxBuild>,
    keep_prev: bool,
    keep_next: bool,
) -> BoundaryBirth {
    let prev_occurrence = if keep_prev { birth.prev_occurrence.clone() } else { rewritten_occurrence(&birth.prev_occurrence, prev_rewrites) };
    let next_occurrence = if keep_next { birth.next_occurrence.clone() } else { rewritten_occurrence(&birth.next_occurrence, next_rewrites) };
    let key = Val::tuple(vec![birth.key.get(0).cloned().unwrap_or_else(Val::none), birth.point_key.clone(), prev_occurrence.clone(), next_occurrence.clone()]);
    BoundaryBirth { point_key: birth.point_key.clone(), prev_occurrence, next_occurrence, key, replaces: birth.replaces.clone() }
}

fn tuple_of(items: &[Val]) -> Val {
    Val::tuple(items.to_vec())
}

/// `_decompose_birth_function(births)`: the partial bijection of births into anchored chains and cycles; `false` when it is not one (a port used twice, a cycle that joins a
/// chain) or does not account for every birth.
pub fn decompose_birth_function(births: &[BoundaryBirth]) -> (Vec<Vec<Val>>, bool) {
    let mut by_prev: HashMap<&Val, Vec<usize>, FxBuild> = HashMap::default();
    let mut by_next: HashMap<&Val, Vec<usize>, FxBuild> = HashMap::default();
    let mut by_key: HashMap<&Val, Vec<usize>, FxBuild> = HashMap::default();
    let mut key_order: Vec<&Val> = Vec::new();
    for (index, item) in births.iter().enumerate() {
        by_prev.entry(&item.prev_occurrence).or_default().push(index);
        by_next.entry(&item.next_occurrence).or_default().push(index);
        let group = by_key.entry(&item.key).or_default();
        if group.is_empty() {
            key_order.push(&item.key);
        }
        group.push(index);
    }
    let single = |table: &HashMap<&Val, Vec<usize>, FxBuild>| table.values().all(|group| group.len() == 1);
    if !(single(&by_prev) && single(&by_next) && single(&by_key)) {
        return (Vec::new(), false);
    }
    let mut successor: HashMap<&Val, &Val, FxBuild> = HashMap::default();
    for item in births {
        if let Some(group) = by_prev.get(&item.next_occurrence) {
            successor.insert(&item.key, &births[group[0]].key);
        }
    }
    let incoming: HashSet<&Val> = successor.values().copied().collect();
    let mut seen: HashSet<&Val> = HashSet::new();
    let mut components: Vec<Vec<Val>> = Vec::new();
    let mut roots: Vec<&Val> = key_order.iter().copied().filter(|key| !incoming.contains(key)).collect();
    roots.sort_by(|left, right| left.repr().as_bytes().cmp(right.repr().as_bytes()));
    for root in roots {
        let mut chain: Vec<Val> = Vec::new();
        let mut cursor = root;
        while !seen.contains(cursor) {
            seen.insert(cursor);
            chain.push(cursor.clone());
            match successor.get(cursor) {
                None => break,
                Some(next) => cursor = next,
            }
        }
        components.push(chain);
    }
    let mut rest: Vec<&Val> = key_order.iter().copied().filter(|key| !seen.contains(key)).collect();
    rest.sort_by(|left, right| left.repr().as_bytes().cmp(right.repr().as_bytes()));
    for root in rest {
        if seen.contains(root) {
            continue;
        }
        let mut cycle: Vec<&Val> = Vec::new();
        let mut cursor = root;
        while !seen.contains(cursor) {
            seen.insert(cursor);
            cycle.push(cursor);
            match successor.get(cursor) {
                None => return (Vec::new(), false),
                Some(next) => cursor = next,
            }
        }
        if !cycle.contains(&cursor) {
            return (Vec::new(), false);
        }
        let offset = cycle.iter().enumerate().fold(0usize, |best, (index, key)| if key.repr().as_bytes() < cycle[best].repr().as_bytes() { index } else { best });
        let rotated: Vec<Val> = cycle[offset..].iter().chain(cycle[..offset].iter()).map(|key| (*key).clone()).collect();
        components.push(rotated);
    }
    let sorted = sorted_by_repr(&components, |component| tuple_of(component));
    (sorted, seen.len() == births.len())
}

/// `_quotient_by_germ(births, ledger)`: births that are one germ presented twice become one (their loads united); one germ key with TWO different representatives is an
/// ambiguity, answered `false`.
pub fn quotient_by_germ(births: &[BoundaryBirth], ledger: &GermLedger) -> (Vec<BoundaryBirth>, bool) {
    let mut slots: HashMap<Val, usize, FxBuild> = HashMap::default();
    let mut grouped: Vec<BoundaryBirth> = Vec::new();
    for item in births {
        let time_key = item.key.get(0).cloned().unwrap_or_else(Val::none);
        let key = ledger.key(&time_key, &item.point_key, &item.prev_occurrence, &item.next_occurrence).unwrap_or_else(|| Val::tuple(vec![Val::str("UNKEYED"), item.key.clone()]));
        match slots.get(&key) {
            None => {
                slots.insert(key, grouped.len());
                grouped.push(item.clone());
            }
            Some(slot) => {
                if grouped[*slot].key != item.key {
                    return (births.to_vec(), false);
                }
                grouped[*slot] = fold_germ(&grouped[*slot], item);
            }
        }
    }
    (grouped, true)
}

/// What `_wire_births` answers: the births (rewritten), the wiring of each into the front, the rewrites of existing ports, the components of the bijection, and whether it
/// all held.
pub type Wired = (Vec<BoundaryBirth>, Vec<BirthWire>, Vec<(i64, Val, Val)>, Vec<Vec<Val>>, bool);

fn unwired(births: Vec<BoundaryBirth>) -> Wired {
    (births, Vec::new(), Vec::new(), Vec::new(), false)
}

/// `_wire_births(births, dead, vertices, split_cuts, ledger)`: proves the port matching on the frozen prestate before any mutation: every birth has exactly one predecessor
/// and one successor among the surviving ports and the other births.
pub fn wire_births(births: Vec<BoundaryBirth>, dead: &BTreeSet<i64>, vertices: &[VertexSnapshot], split_cuts: &[SplitCutPlan], ledger: &GermLedger) -> SkelResult<Wired> {
    let mut prev_rewrites: HashMap<Val, Val, FxBuild> = HashMap::default();
    let mut next_rewrites: HashMap<Val, Val, FxBuild> = HashMap::default();
    let mut final_ports: HashMap<Val, (bool, bool), FxBuild> = HashMap::default();
    for cut in split_cuts {
        prev_rewrites.insert(cut.target_occurrence.clone(), cut.segment_occurrences[cut.segment_occurrences.len() - 1].clone());
        next_rewrites.insert(cut.target_occurrence.clone(), cut.segment_occurrences[0].clone());
    }
    for cut in split_cuts {
        for (key, keep_prev, keep_next) in &cut.final_birth_ports {
            final_ports.insert(key.clone(), (*keep_prev, *keep_next));
        }
    }
    let (quotient, unambiguous) = quotient_by_germ(&births, ledger);
    if !unambiguous {
        return Ok(unwired(births));
    }
    let rewritten: Vec<BoundaryBirth> = quotient
        .iter()
        .map(|item| {
            let (keep_prev, keep_next) = final_ports.get(&item.key).copied().unwrap_or((false, false));
            rewrite_birth(item, &prev_rewrites, &next_rewrites, keep_prev, keep_next)
        })
        .collect();
    let rewritten = births_by_key(&rewritten, "superlevel._wire_births")?;
    let (components, decomposable) = decompose_birth_function(&rewritten);
    if !decomposable {
        return Ok(unwired(rewritten));
    }
    let mut starts: HashMap<Val, Vec<VertexReference>, FxBuild> = HashMap::default();
    let mut ends: HashMap<Val, Vec<VertexReference>, FxBuild> = HashMap::default();
    let mut rewrites: Vec<(i64, Val, Val)> = Vec::new();
    for vertex in vertices {
        if !vertex.alive || dead.contains(&vertex.ident) {
            continue;
        }
        let prev_occurrence = vertex.prev_occurrence.as_ref().map(|found| rewritten_occurrence(found, &prev_rewrites));
        let next_occurrence = vertex.next_occurrence.as_ref().map(|found| rewritten_occurrence(found, &next_rewrites));
        let reference = VertexReference { existing: Some(vertex.ident), birth_key: None };
        if let Some(next) = &next_occurrence {
            starts.entry(next.clone()).or_default().push(reference.clone());
        }
        if let Some(prev) = &prev_occurrence {
            ends.entry(prev.clone()).or_default().push(reference.clone());
        }
        if prev_occurrence != vertex.prev_occurrence || next_occurrence != vertex.next_occurrence {
            let (Some(prev), Some(next)) = (prev_occurrence, next_occurrence) else {
                return Ok(unwired(rewritten));
            };
            rewrites.push((vertex.ident, prev, next));
        }
    }
    for item in &rewritten {
        let reference = VertexReference { existing: None, birth_key: Some(item.key.clone()) };
        starts.entry(item.next_occurrence.clone()).or_default().push(reference.clone());
        ends.entry(item.prev_occurrence.clone()).or_default().push(reference);
    }
    let mut wiring: Vec<BirthWire> = Vec::new();
    for item in &rewritten {
        let predecessors = starts.get(&item.prev_occurrence).map_or(&[][..], Vec::as_slice);
        let successors = ends.get(&item.next_occurrence).map_or(&[][..], Vec::as_slice);
        if predecessors.len() != 1 || successors.len() != 1 {
            return Ok(unwired(rewritten));
        }
        wiring.push((item.key.clone(), predecessors[0].clone(), successors[0].clone()));
    }
    wiring.sort_by_cached_key(|wire| wire.0.repr().to_string());
    rewrites.sort_by_key(|(ident, _, _)| *ident);
    Ok((rewritten, wiring, rewrites, components, true))
}

/// `_terminal_two_birth_cycles(births, components, wiring)`: a fully born reciprocal two-cycle without an existing anchor (a witness only, not an authority to retire).
pub fn terminal_two_birth_cycles(births: &[BoundaryBirth], components: &[Vec<Val>], wiring: &[BirthWire]) -> Vec<Vec<Val>> {
    let by_key: HashMap<&Val, &BoundaryBirth, FxBuild> = births.iter().map(|item| (&item.key, item)).collect();
    let wired: HashMap<&Val, (&VertexReference, &VertexReference), FxBuild> = wiring.iter().map(|(key, predecessor, successor)| (key, (predecessor, successor))).collect();
    let mut terminal = Vec::new();
    for component in components {
        if component.len() != 2 || component[0] == component[1] {
            continue;
        }
        let (first, second) = (&component[0], &component[1]);
        let (Some(first_birth), Some(second_birth)) = (by_key.get(first), by_key.get(second)) else {
            continue;
        };
        if second_birth.next_occurrence != first_birth.prev_occurrence {
            continue;
        }
        let mut reciprocal = true;
        for (key, other) in [(first, second), (second, first)] {
            let Some((predecessor, successor)) = wired.get(key) else {
                reciprocal = false;
                break;
            };
            if predecessor.existing.is_some() || successor.existing.is_some() || predecessor.birth_key.as_ref() != Some(other) || successor.birth_key.as_ref() != Some(other) {
                reciprocal = false;
                break;
            }
        }
        if reciprocal {
            terminal.push(component.clone());
        }
    }
    terminal
}

// --------------------------------------------------------------------------
// the component, the packet
// --------------------------------------------------------------------------

/// What `_component_stages` answers.
pub struct Stages {
    pub contacts: Vec<EdgeContactPlan>,
    pub meetings: Vec<VertexMeetingPlan>,
    pub split_cuts: Vec<SplitCutPlan>,
    pub fallbacks: Vec<CandidateEvent>,
    pub dropped: i64,
    pub dead: BTreeSet<i64>,
    pub valid: bool,
}

/// `_component_stages(component, vertices, ledger, budget)`: the three paths on one frozen prestate and whether they are compatible: the dying ports are shared only where
/// the composition proved it, any other intersection is an ambiguity, not a choice.
pub fn component_stages(ctx: &mut ExactCtx<'_>, component: &[&Incident], vertices: &[VertexSnapshot], ledger: &mut GermLedger) -> SkelResult<Stages> {
    let mut geometric: Vec<(&IncidentSortKey, Vec<&Incident>)> = Vec::new();
    for incident in component {
        let key = incident.sort_key();
        match geometric.iter_mut().find(|(known, _)| *known == key) {
            Some((_, group)) => {
                if !group.iter().any(|seen| seen.identity() == incident.identity()) {
                    group.push(incident);
                }
            }
            None => geometric.push((key, vec![incident])),
        }
    }
    let unique_incidents = geometric.iter().all(|(_, group)| group.len() == 1);
    let edges: Vec<&Incident> = component.iter().filter(|incident| incident.event.kind == EventKind::Edge).copied().collect();
    let splits: Vec<&Incident> = component.iter().filter(|incident| incident.event.kind == EventKind::Split).copied().collect();
    let (contacts, remaining, valid) = edge_contact_plans(&edges, &splits, vertices, ledger)?;
    let (meetings, cuts, fallbacks) = meeting_plans(&remaining, vertices, ledger)?;
    let (split_cuts, dropped, cuts_valid) = split_cut_plans(ctx, &cuts, vertices, ledger)?;
    let (contacts, split_cuts, composed_overlap, composition_valid) = crate::composition::compose_edge_split_overlap(contacts, split_cuts, vertices, ledger)?;
    let contact_dead: BTreeSet<i64> = contacts.iter().flat_map(|contact| contact.dead_vertex_ids.iter().copied()).collect();
    let meeting_dead: BTreeSet<i64> = meetings.iter().flat_map(|meeting| meeting.meeting_vertex_ids.iter().copied()).collect();
    let cut_dead: BTreeSet<i64> = split_cuts.iter().flat_map(|cut| cut.events.iter().map(|event| event.vertex)).collect();
    let overlap: BTreeSet<i64> = contact_dead.intersection(&cut_dead).copied().collect();
    let valid = valid
        && unique_incidents
        && cuts_valid
        && composition_valid
        && contact_dead.is_disjoint(&meeting_dead)
        && meeting_dead.is_disjoint(&cut_dead)
        && overlap == composed_overlap
        && (composed_overlap.is_empty() || meetings.is_empty());
    let dead: BTreeSet<i64> = contact_dead.union(&meeting_dead).chain(cut_dead.iter()).copied().collect();
    Ok(Stages { contacts, meetings, split_cuts, fallbacks, dropped, dead, valid })
}

/// `_planned_component(component, vertices, base, ledger, budget)`.
pub fn planned_component(ctx: &mut ExactCtx<'_>, component: &[&Incident], vertices: &[VertexSnapshot], mut base: ComponentFields, ledger: &mut GermLedger) -> SkelResult<ComponentPlan> {
    let kinds = base.event_kinds.clone();
    let Stages { contacts, meetings, split_cuts, fallbacks, dropped, dead, valid } = component_stages(ctx, component, vertices, ledger)?;
    let raw_births: Vec<BoundaryBirth> = contacts
        .iter()
        .flat_map(|contact| contact.births.iter().cloned())
        .chain(meetings.iter().flat_map(|meeting| meeting.births.iter().cloned()))
        .chain(split_cuts.iter().flat_map(|cut| cut.births.iter().cloned()))
        .collect();
    let (births, birth_wiring, existing_port_rewrites, birth_components, ports_valid) = wire_births(raw_births, &dead, vertices, &split_cuts, ledger)?;
    let terminal_birth_cycles = terminal_two_birth_cycles(&births, &birth_components, &birth_wiring);
    let valid = valid && ports_valid;
    let chains = snapshot_chains(&dead, vertices);
    if let Some(first) = contacts.first() {
        base.point = Rc::clone(&first.point);
    }
    let resolution = if !valid {
        Resolution::Unresolvable
    } else if kinds == [EventKind::Edge] {
        Resolution::Edge
    } else if kinds == [EventKind::Split] {
        Resolution::Split
    } else {
        Resolution::BoundaryPortPairing
    };
    let closed_chain_count = if births.is_empty() { chains.iter().filter(|chain| vertices[chain[chain.len() - 1] as usize].next == chain[0]).count() as i64 } else { 0 };
    Ok(ComponentPlan {
        event_kinds: base.event_kinds,
        time: base.time,
        point_keys: base.point_keys,
        point: base.point,
        resolution,
        events: base.events,
        participants: base.participants,
        target_participants: base.target_participants,
        dead_vertex_ids: dead.iter().copied().collect(),
        chains,
        enqueue_born_vertices: !births.is_empty(),
        births,
        queue_seed_vertex_ids: Vec::new(),
        edge_contacts: contacts,
        split_cuts,
        vertex_meetings: meetings,
        proof_fallbacks: fallbacks,
        coincident_split_targets: dropped,
        closed_chain_count,
        suppressed_candidates: 0,
        birth_wiring,
        existing_port_rewrites,
        birth_components,
        terminal_birth_cycles,
    })
}

/// `_snapshot_rays(snapshot)`: the primitive ray of every occurrence of the packet, read from the carrier lines.
pub fn snapshot_rays(snapshot: &Snapshot) -> HashMap<Val, (i64, i64), FxBuild> {
    let mut rays: HashMap<Val, (i64, i64), FxBuild> = HashMap::default();
    let key_of = |occurrence: &Val| occurrence.get(0).cloned().unwrap_or_else(Val::none);
    for vertex in &snapshot.vertices {
        if let Some(next) = &vertex.next_occurrence {
            rays.entry(key_of(next)).or_insert(vertex.outgoing_ray);
        }
        if let Some(prev) = &vertex.prev_occurrence {
            rays.entry(key_of(prev)).or_insert((-vertex.incoming_ray.0, -vertex.incoming_ray.1));
        }
    }
    for incident in &snapshot.incidents {
        if let (Some(occurrence), Some(ray)) = (&incident.target_occurrence, incident.target_ray) {
            rays.entry(key_of(occurrence)).or_insert(ray);
        }
    }
    rays
}

/// `plan_superlevel_components(snapshot, budget)`: every component delta, without changing a runtime object. The ledger of germs is the packet's, not the component's: a
/// locus presented by two components must be one object.
pub fn plan_superlevel_components(ctx: &mut ExactCtx<'_>, snapshot: &Snapshot) -> SkelResult<Vec<ComponentPlan>> {
    let mut ledger = GermLedger::new(snapshot_rays(snapshot));
    let mut plans = Vec::new();
    for component in connected_components(&snapshot.incidents) {
        let base = component_fields(&component);
        plans.push(planned_component(ctx, &component, &snapshot.vertices, base, &mut ledger)?);
    }
    let order_key = |plan: &ComponentPlan| {
        (
            plan.point_keys.iter().map(|key| key.repr().to_string()).collect::<Vec<String>>(),
            plan.event_kinds.iter().map(|kind| kind.value()).collect::<Vec<&'static str>>(),
            plan.participants.clone(),
        )
    };
    plans.sort_by_cached_key(order_key);
    Ok(plans)
}

// --------------------------------------------------------------------------
// the values the seams print
// --------------------------------------------------------------------------

/// The `repr` of the objects of this layer as the oracle's dataclasses print them: the plan types become [`Val`] dataclass instances.
pub trait PlanVal {
    fn to_val(&self) -> Val;
}

fn ints_val(items: &[i64]) -> Val {
    Val::ints(items)
}

fn keys_val(keys: &[EdgeKey]) -> Val {
    Val::tuple(keys.iter().map(|key| Val::ints(key)).collect())
}

fn sum_val(sum: &SqrtSum) -> Val {
    Val::data("SqrtSumV1", vec![("terms", Val::terms_of(sum))])
}

pub fn time_val(time: &EventTime) -> Val {
    Val::data("EventTimeV1", vec![("dividend", Val::frac(time.dividend.clone())), ("divisor", sum_val(&time.divisor))])
}

pub fn point_val(point: &EventPoint) -> Val {
    Val::data("EventPointV1", vec![("x", sum_val(&point.x)), ("y", sum_val(&point.y))])
}

pub fn kind_val(kind: EventKind) -> Val {
    Val::member("EventKind", kind_name(kind), Val::str(kind.value()))
}

fn kind_name(kind: EventKind) -> &'static str {
    kind.value()
}

fn kinds_val(kinds: &[EventKind]) -> Val {
    Val::tuple(kinds.iter().map(|kind| kind_val(*kind)).collect())
}

fn vals(items: &[Val]) -> Val {
    Val::tuple(items.to_vec())
}

impl PlanVal for CandidateEvent {
    fn to_val(&self) -> Val {
        Val::data(
            "CandidateEventV1",
            vec![
                ("kind", kind_val(self.kind)),
                ("time", time_val(&self.time)),
                ("point", point_val(&self.point)),
                ("vertex", Val::int(self.vertex)),
                ("peer", Val::int(self.peer)),
                ("edge", Val::int(self.edge)),
            ],
        )
    }
}

fn events_val(events: &[CandidateEvent]) -> Val {
    Val::tuple(events.iter().map(PlanVal::to_val).collect())
}

impl PlanVal for BoundaryBirth {
    fn to_val(&self) -> Val {
        Val::data(
            "BoundaryBirthV1",
            vec![
                ("point_key", self.point_key.clone()),
                ("prev_occurrence", self.prev_occurrence.clone()),
                ("next_occurrence", self.next_occurrence.clone()),
                ("key", self.key.clone()),
                ("replaces", ints_val(&self.replaces)),
            ],
        )
    }
}

fn births_val(births: &[BoundaryBirth]) -> Val {
    Val::tuple(births.iter().map(PlanVal::to_val).collect())
}

impl PlanVal for VertexReference {
    fn to_val(&self) -> Val {
        Val::data("VertexReferenceV1", vec![("existing", self.existing.map_or_else(Val::none, Val::int)), ("birth_key", self.birth_key.clone().unwrap_or_else(Val::none))])
    }
}

impl PlanVal for EdgeContactPlan {
    fn to_val(&self) -> Val {
        Val::data(
            "EdgeContactPlanV1",
            vec![
                ("events", events_val(&self.events)),
                ("time", time_val(&self.time)),
                ("point", point_val(&self.point)),
                ("point_key", self.point_key.clone()),
                ("participants", keys_val(&self.participants)),
                ("dead_vertex_ids", ints_val(&self.dead_vertex_ids)),
                ("chains", Val::tuple(self.chains.iter().map(|chain| ints_val(chain)).collect())),
                ("births", births_val(&self.births)),
                ("kinds", kinds_val(&self.kinds)),
            ],
        )
    }
}

impl PlanVal for SplitCutPlan {
    fn to_val(&self) -> Val {
        Val::data(
            "SplitCutPlanV1",
            vec![
                ("edge_id", Val::int(self.edge_id)),
                ("target_occurrence", self.target_occurrence.clone()),
                ("events", events_val(&self.events)),
                ("segment_occurrences", vals(&self.segment_occurrences)),
                ("births", births_val(&self.births)),
                ("final_birth_ports", Val::tuple(self.final_birth_ports.iter().map(|(key, prev, next)| Val::tuple(vec![key.clone(), Val::boolean(*prev), Val::boolean(*next)])).collect())),
            ],
        )
    }
}

impl PlanVal for VertexMeetingPlan {
    fn to_val(&self) -> Val {
        Val::data(
            "VertexMeetingPlanV1",
            vec![
                ("events", events_val(&self.events)),
                ("time", time_val(&self.time)),
                ("point", point_val(&self.point)),
                ("meeting_vertex_ids", ints_val(&self.meeting_vertex_ids)),
                ("pairs", Val::tuple(self.pairs.iter().map(|(first, second)| ints_val(&[*first, *second])).collect())),
                ("participants", keys_val(&self.participants)),
                ("births", births_val(&self.births)),
            ],
        )
    }
}

impl PlanVal for ComponentPlan {
    fn to_val(&self) -> Val {
        Val::data(
            "SuperlevelComponentPlanV1",
            vec![
                ("event_kinds", kinds_val(&self.event_kinds)),
                ("time", time_val(&self.time)),
                ("point_keys", vals(&self.point_keys)),
                ("point", point_val(&self.point)),
                ("resolution", Val::member("SuperlevelResolution", self.resolution.value(), Val::str(self.resolution.value()))),
                ("events", events_val(&self.events)),
                ("participants", keys_val(&self.participants)),
                ("target_participants", keys_val(&self.target_participants)),
                ("dead_vertex_ids", ints_val(&self.dead_vertex_ids)),
                ("chains", Val::tuple(self.chains.iter().map(|chain| ints_val(chain)).collect())),
                ("births", births_val(&self.births)),
                ("queue_seed_vertex_ids", ints_val(&self.queue_seed_vertex_ids)),
                ("enqueue_born_vertices", Val::boolean(self.enqueue_born_vertices)),
                ("edge_contacts", Val::tuple(self.edge_contacts.iter().map(PlanVal::to_val).collect())),
                ("split_cuts", Val::tuple(self.split_cuts.iter().map(PlanVal::to_val).collect())),
                ("vertex_meetings", Val::tuple(self.vertex_meetings.iter().map(PlanVal::to_val).collect())),
                ("proof_fallbacks", events_val(&self.proof_fallbacks)),
                ("coincident_split_targets", Val::int(self.coincident_split_targets)),
                ("closed_chain_count", Val::int(self.closed_chain_count)),
                ("suppressed_candidates", Val::int(self.suppressed_candidates)),
                (
                    "birth_wiring",
                    Val::tuple(self.birth_wiring.iter().map(|(key, predecessor, successor)| Val::tuple(vec![key.clone(), predecessor.to_val(), successor.to_val()])).collect()),
                ),
                (
                    "existing_port_rewrites",
                    Val::tuple(self.existing_port_rewrites.iter().map(|(ident, prev, next)| Val::tuple(vec![Val::int(*ident), prev.clone(), next.clone()])).collect()),
                ),
                ("birth_components", Val::tuple(self.birth_components.iter().map(|component| vals(component)).collect())),
                ("terminal_birth_cycles", Val::tuple(self.terminal_birth_cycles.iter().map(|component| vals(component)).collect())),
            ],
        )
    }
}
