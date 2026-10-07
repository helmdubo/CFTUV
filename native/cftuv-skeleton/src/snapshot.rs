//! The frozen prestate of one exact-time packet (`wavefront/superlevel_snapshot.py`): everything the transaction reads from the builder before its first mutation, and
//! nothing that mutates.
//!
//! `collect_superlevel_snapshot(builder, level)` asks the front, in this order: which events of the packet are still live (a split asks the span of its edge, which may
//! hydrate places), which edge occurrences the packet must carry (all live ones, or none when no event is live), the exact place of every end of every carried span
//! (`_sparse_occurrences`, through the memory of places), one [`VertexSnapshot`] per runtime vertex, and one [`Incident`] per live event (a split also asks which vertex of
//! the front stands at its point). The order is the cost: the memory of places is hit or filled in exactly the oracle's sequence.
//!
//! The identities of the snapshot are values: a point key `(x terms, y terms)`, an occurrence `(edge key, start point key | None, end point key | None)`, a port
//! `(prev edge key | None, next edge key | None)`. They are [`Val`]s (the oracle's `ExactIdentityKeyV1` is a tuple that remembers its hash), compared, hashed and printed
//! as Python does.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::rc::Rc;

use cftuv_core::exact::ExactCtx;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::builder::{project, Builder};
use crate::error::{SkelError, SkelResult};
use crate::line::SupportLine;
use crate::proof::EdgeKey;
use crate::pyval::Val;
use crate::queue::{CandidateEvent, EventKind};
use crate::time::{EventPoint, EventTime};

/// `_VertexSnapshot`.
#[derive(Debug, Clone)]
pub struct VertexSnapshot {
    pub ident: i64,
    pub prev: i64,
    pub next: i64,
    pub prev_edge: i64,
    pub next_edge: i64,
    pub alive: bool,
    pub incoming_ray: (i64, i64),
    pub outgoing_ray: (i64, i64),
    pub point_key: Option<Val>,
    pub prev_occurrence: Option<Val>,
    pub next_occurrence: Option<Val>,
}

/// `_incident_sort_key(incident)`: only geometry, never a runtime id; every item has an order, so the derived order is the oracle's tuple order.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct IncidentSortKey {
    pub kind: &'static str,
    pub point_key: String,
    pub emitter_key: String,
    pub peer_key: String,
    pub target_occurrence: String,
    pub participants: Vec<EdgeKey>,
    pub target_participants: Vec<EdgeKey>,
    pub target_projection: String,
    pub span_unproven: bool,
}

/// `SuperlevelIncidentV1`: one live candidate with its geometric identity and its runtime payload.
#[derive(Debug, Clone)]
pub struct Incident {
    pub event: CandidateEvent,
    pub vertex_ids: Vec<i64>,
    pub edge_occurrences: Vec<i64>,
    pub participants: Vec<EdgeKey>,
    pub target_participants: Vec<EdgeKey>,
    pub point_key: Val,
    pub met_vertex_id: Option<i64>,
    pub met_adjacent: bool,
    pub target_projection: Option<SqrtSum>,
    pub target_start_id: Option<i64>,
    pub target_end_id: Option<i64>,
    pub emitter_key: Val,
    pub peer_key: Val,
    pub target_occurrence: Option<Val>,
    pub target_ray: Option<(i64, i64)>,
}

/// `SuperlevelSnapshotV1`.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub incidents: Vec<Incident>,
    pub vertices: Vec<VertexSnapshot>,
    pub unsupported: Vec<CandidateEvent>,
    pub stale_candidates: i64,
    pub duplicate_live_owner_edge_ids: Vec<i64>,
}

/// The identity of a candidate event as the oracle's `==` and `hash` see it: kind, time, point, vertex, peer and edge (the flag `span_unproven` is not part of it).
pub type EventIdentity = (EventKind, (cftuv_core::rat::Rat, cftuv_core::sqrt_sum::IntForm), (cftuv_core::sqrt_sum::IntForm, cftuv_core::sqrt_sum::IntForm), i64, i64, i64);

pub fn event_identity(event: &CandidateEvent) -> EventIdentity {
    (
        event.kind,
        crate::view::time_key(&event.time),
        (event.point.x.canonical_form().clone(), event.point.y.canonical_form().clone()),
        event.vertex,
        event.peer,
        event.edge,
    )
}

/// `left == right` of two events.
pub fn same_event(left: &CandidateEvent, right: &CandidateEvent) -> bool {
    left.kind == right.kind && left.vertex == right.vertex && left.peer == right.peer && left.edge == right.edge && event_identity(left) == event_identity(right)
}

impl Incident {
    /// `_incident_sort_key(incident)`.
    pub fn sort_key(&self) -> IncidentSortKey {
        IncidentSortKey {
            kind: self.event.kind.value(),
            point_key: self.point_key.repr().to_string(),
            emitter_key: self.emitter_key.repr().to_string(),
            peer_key: self.peer_key.repr().to_string(),
            target_occurrence: self.target_occurrence.as_ref().map_or_else(|| "None".to_string(), |value| value.repr().to_string()),
            participants: self.participants.clone(),
            target_participants: self.target_participants.clone(),
            target_projection: self.target_projection.as_ref().map_or_else(|| "None".to_string(), |sum| sum_repr(sum)),
            span_unproven: self.event.span_unproven,
        }
    }

    /// `left == right` of two incidents (every field; the events by their identity).
    pub fn same(&self, other: &Incident) -> bool {
        same_event(&self.event, &other.event)
            && self.vertex_ids == other.vertex_ids
            && self.edge_occurrences == other.edge_occurrences
            && self.participants == other.participants
            && self.target_participants == other.target_participants
            && self.point_key == other.point_key
            && self.met_vertex_id == other.met_vertex_id
            && self.met_adjacent == other.met_adjacent
            && self.target_projection.as_ref().map(SqrtSum::canonical_form) == other.target_projection.as_ref().map(SqrtSum::canonical_form)
            && self.target_start_id == other.target_start_id
            && self.target_end_id == other.target_end_id
            && self.emitter_key == other.emitter_key
            && self.peer_key == other.peer_key
            && self.target_occurrence == other.target_occurrence
            && self.target_ray == other.target_ray
    }
}

/// `repr(sum)` of a `SqrtSumV1`.
pub fn sum_repr(sum: &SqrtSum) -> String {
    let mut out = String::new();
    crate::repr::repr_sum(sum, &mut out);
    out
}

/// `_event_point_key(event)` / `exact_point_key(point)`: `(x.terms, y.terms)`.
pub fn exact_point_key(point: &EventPoint) -> Val {
    Val::tuple(vec![Val::terms_of(&point.x), Val::terms_of(&point.y)])
}

/// `_time_key(time)`: `(canonical dividend, canonical divisor terms)`.
pub fn time_key(time: &EventTime) -> SkelResult<Val> {
    let canonical = time.canonical()?;
    Ok(Val::tuple(vec![Val::frac(canonical.dividend.clone()), Val::terms_of(&canonical.divisor)]))
}

/// `_direction(line)`: the primitive integer direction `(b, -a) / gcd` of the carrier line.
pub fn direction(line: &SupportLine) -> SkelResult<(i64, i64)> {
    let (dx, dy) = (i128::from(line.b), -i128::from(line.a));
    let (mut left, mut right) = (dx.unsigned_abs(), dy.unsigned_abs());
    while right != 0 {
        (left, right) = (right, left % right);
    }
    if left == 0 {
        return Err(SkelError::Unsupported("ZeroDivisionError: the direction of a line without a normal".to_string()));
    }
    let narrow = |value: i128| i64::try_from(value).map_err(|_| SkelError::Unsupported("a direction beyond the machine range".to_string()));
    Ok((narrow(dx / left as i128)?, narrow(dy / left as i128)?))
}

/// `_port_identity(vertex)`: the two source edges of a joint, which survive the hydration of a sparse port.
pub fn port_identity(vertex: &VertexSnapshot) -> Val {
    let key = |occurrence: &Option<Val>| occurrence.as_ref().and_then(|value| value.get(0).cloned()).unwrap_or_else(Val::none);
    Val::tuple(vec![key(&vertex.prev_occurrence), key(&vertex.next_occurrence)])
}

/// `_live_level(builder, level)`: the live events, the events of a kind the transaction does not carry, and the number of stale ones.
pub fn live_level(ctx: &mut ExactCtx<'_>, builder: &mut Builder, level: &[CandidateEvent]) -> SkelResult<(Vec<CandidateEvent>, Vec<CandidateEvent>, i64)> {
    let (mut live, mut unsupported, mut stale) = (Vec::new(), Vec::new(), 0);
    for event in level {
        if !matches!(event.kind, EventKind::Split | EventKind::Edge) {
            unsupported.push(event.clone());
            continue;
        }
        let is_live = if event.kind == EventKind::Edge { builder.edge_event_is_live(event)? } else { builder.split_is_live(ctx, event)? };
        if is_live {
            live.push(event.clone());
        } else {
            stale += 1;
        }
    }
    Ok((live, unsupported, stale))
}

/// `_required_physical_edge_keys(builder, events)`: the keys of the edge occurrences the packet must carry: every live one, or none when no event is live.
pub fn required_physical_edge_keys(builder: &Builder, events: &[CandidateEvent]) -> SkelResult<HashSet<Vec<i64>>> {
    if events.is_empty() {
        return Ok(HashSet::new());
    }
    let mut keys = HashSet::new();
    for vertex in builder.vertices.iter().filter(|vertex| vertex.alive) {
        keys.insert(builder.edge_at(vertex.next_edge)?.span.to_vec());
    }
    for vertex in builder.vertices.iter().filter(|vertex| vertex.alive) {
        keys.insert(builder.edge_at(vertex.prev_edge)?.span.to_vec());
    }
    Ok(keys)
}

/// What `_sparse_occurrences` answers: the point key of every vertex that is an end of a carried span, the occurrence of every carried span by its edge, and the
/// edges that have more than one live owner.
pub struct SparseOccurrences {
    pub point_keys: Vec<Option<Val>>,
    pub occurrences: HashMap<i64, Val>,
    pub duplicate_owner_ids: Vec<i64>,
}

/// `_sparse_occurrences(builder, time, required_keys)`.
pub fn sparse_occurrences(ctx: &mut ExactCtx<'_>, builder: &mut Builder, time: &EventTime, required: &HashSet<Vec<i64>>) -> SkelResult<SparseOccurrences> {
    let mut span_starts: Vec<(i64, i64)> = Vec::new();
    let mut point_vertex_ids: BTreeSet<i64> = BTreeSet::new();
    let mut live_owner_counts: HashMap<i64, i64> = HashMap::new();
    for vertex in &builder.vertices {
        if !vertex.alive {
            continue;
        }
        let end = builder.vertex_at(vertex.next)?;
        if !end.alive {
            continue;
        }
        *live_owner_counts.entry(vertex.next_edge).or_insert(0) += 1;
        if required.contains(&builder.edge_at(vertex.next_edge)?.span.to_vec()) {
            span_starts.push((vertex.ident, end.ident));
            point_vertex_ids.insert(vertex.ident);
            point_vertex_ids.insert(end.ident);
        }
    }
    let mut point_keys: Vec<Option<Val>> = vec![None; builder.vertices.len()];
    for ident in point_vertex_ids {
        let point = builder.position(ctx, ident, time)?;
        point_keys[ident as usize] = point.map(|point| exact_point_key(&point));
    }
    let mut occurrences = HashMap::new();
    for (start, end) in span_starts {
        let next_edge = builder.vertex_at(start)?.next_edge;
        let key = Val::ints(builder.edge_at(next_edge)?.span.as_slice());
        let part = |ident: i64| point_keys[ident as usize].clone().unwrap_or_else(Val::none);
        occurrences.insert(next_edge, Val::tuple(vec![key, part(start), part(end)]));
    }
    let mut duplicate_owner_ids: Vec<i64> = live_owner_counts.into_iter().filter(|(_, count)| *count != 1).map(|(edge, _)| edge).collect();
    duplicate_owner_ids.sort_unstable();
    Ok(SparseOccurrences { point_keys, occurrences, duplicate_owner_ids })
}

/// `_vertex_snapshot(builder, vertex, point_key, occurrences)`.
pub fn vertex_snapshot(builder: &Builder, ident: i64, point_key: Option<Val>, occurrences: &HashMap<i64, Val>) -> SkelResult<VertexSnapshot> {
    let vertex = builder.vertex_at(ident)?;
    let outgoing = direction(&builder.edge_at(vertex.next_edge)?.line)?;
    let incoming = direction(&builder.edge_at(vertex.prev_edge)?.line)?;
    Ok(VertexSnapshot {
        ident: vertex.ident,
        prev: vertex.prev,
        next: vertex.next,
        prev_edge: vertex.prev_edge,
        next_edge: vertex.next_edge,
        alive: vertex.alive,
        incoming_ray: (-incoming.0, -incoming.1),
        outgoing_ray: outgoing,
        point_key,
        prev_occurrence: occurrences.get(&vertex.prev_edge).cloned(),
        next_occurrence: occurrences.get(&vertex.next_edge).cloned(),
    })
}

fn sorted_set(items: impl IntoIterator<Item = i64>) -> Vec<i64> {
    items.into_iter().collect::<BTreeSet<i64>>().into_iter().collect()
}

fn sorted_keys(keys: Vec<EdgeKey>) -> Vec<EdgeKey> {
    keys.into_iter().collect::<BTreeSet<EdgeKey>>().into_iter().collect()
}

/// `_incident(builder, event, vertices)`: the incident of one live event. A split asks which vertex of the front stands at its point (`_front_vertex_met_by`).
pub fn incident(ctx: &mut ExactCtx<'_>, builder: &mut Builder, event: &CandidateEvent, vertices: &[VertexSnapshot]) -> SkelResult<Incident> {
    let vertex = builder.vertex_at(event.vertex)?.clone();
    let (mut met_vertex_id, mut met_adjacent) = (None, false);
    let (mut target_projection, mut target_ray, mut target_start_id, mut target_end_id) = (None, None, None, None);
    let (core_vertex_ids, participant_edge_ids, target_occurrence): (Vec<i64>, Vec<i64>, Option<Val>);
    if event.kind == EventKind::Edge {
        let peer = builder.vertex_at(event.peer)?.clone();
        core_vertex_ids = vec![vertex.ident, peer.ident];
        participant_edge_ids = vec![vertex.prev_edge, vertex.next_edge, peer.prev_edge, peer.next_edge];
        target_occurrence = None;
    } else {
        let endpoint_ids = builder.proof_edge_endpoint_ids(event.edge);
        target_start_id = builder.edge_start.get(&event.edge).copied();
        target_end_id = builder.edge_end.get(&event.edge).copied();
        let (met, adjacent) = builder.front_vertex_met_by(ctx, event)?;
        met_vertex_id = met;
        met_adjacent = adjacent;
        let mut core = vec![vertex.ident];
        core.extend(endpoint_ids);
        core.extend(met_vertex_id);
        core_vertex_ids = core;
        participant_edge_ids = vec![vertex.prev_edge, vertex.next_edge, event.edge];
        let emitter = vertices.get(event.vertex as usize).ok_or_else(|| SkelError::Unsupported("IndexError: the emitter is not in the snapshot".to_string()))?;
        let mut occurrence = emitter.next_occurrence.clone();
        if 0 <= event.edge && (event.edge as usize) < builder.edges.len() {
            occurrence = vertices.iter().find(|item| item.alive && item.next_edge == event.edge).and_then(|item| item.next_occurrence.clone());
        }
        target_occurrence = occurrence;
        let line = &builder.edge_at(event.edge)?.line;
        target_ray = Some(direction(line)?);
        target_projection = Some(project(line, &event.point));
    }
    let mut resource: BTreeSet<i64> = core_vertex_ids.iter().copied().collect();
    let neighbours: Vec<i64> = resource.iter().copied().collect();
    for ident in neighbours {
        let found = builder.vertex_at(ident)?;
        resource.insert(found.prev);
        resource.insert(found.next);
    }
    let keys = sorted_keys(builder.edge_keys(&participant_edge_ids));
    let emitter_state = vertices.get(event.vertex as usize).ok_or_else(|| SkelError::Unsupported("IndexError: the emitter is not in the snapshot".to_string()))?;
    let peer_key = if event.kind == EventKind::Edge {
        port_identity(vertices.get(event.peer as usize).ok_or_else(|| SkelError::Unsupported("IndexError: the peer is not in the snapshot".to_string()))?)
    } else {
        Val::tuple(Vec::new())
    };
    Ok(Incident {
        event: event.clone(),
        vertex_ids: resource.into_iter().collect(),
        edge_occurrences: sorted_set(participant_edge_ids.iter().copied()),
        participants: keys.clone(),
        target_participants: keys,
        point_key: exact_point_key(&event.point),
        met_vertex_id,
        met_adjacent,
        target_projection,
        target_start_id,
        target_end_id,
        emitter_key: port_identity(emitter_state),
        peer_key,
        target_occurrence,
        target_ray,
    })
}

/// `collect_superlevel_snapshot(builder, level)`: the frozen prestate of the packet.
pub fn collect_superlevel_snapshot(ctx: &mut ExactCtx<'_>, builder: &mut Builder, level: &[CandidateEvent]) -> SkelResult<Snapshot> {
    let time: Rc<EventTime> = level.first().map_or_else(|| Rc::new(EventTime::zero()), |event| Rc::clone(&event.time));
    let (live, unsupported, stale) = live_level(ctx, builder, level)?;
    let required = required_physical_edge_keys(builder, &live)?;
    let SparseOccurrences { point_keys, occurrences, duplicate_owner_ids } = sparse_occurrences(ctx, builder, &time, &required)?;
    let mut vertices = Vec::with_capacity(builder.vertices.len());
    for ident in 0..builder.vertices.len() as i64 {
        vertices.push(vertex_snapshot(builder, ident, point_keys[ident as usize].clone(), &occurrences)?);
    }
    let mut incidents = Vec::with_capacity(live.len());
    for event in &live {
        incidents.push(incident(ctx, builder, event, &vertices)?);
    }
    Ok(Snapshot { incidents, vertices, unsupported, stale_candidates: stale, duplicate_live_owner_edge_ids: duplicate_owner_ids })
}
