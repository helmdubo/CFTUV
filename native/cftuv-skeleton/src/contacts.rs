//! The contacts of an exact time over a symbolic overlay: what a vertex of the front meets at exactly `now`, by keys that hold no runtime id
//! (`superlevel_fixed_point.py`: the live half, `symbolic_edge_closure.py`, `symbolic_split_endpoint.py`: `discover_endpoint_contacts`, `symbolic_junction_contacts.py`).
//!
//! * an INTERIOR split contact ([`SymSplitContact`]) is a reflex or sliding vertex meeting the inside of a leaf: a stable key (time, point, emitter, family, participants), the
//!   projection of the point on the leaf, and the leaf it was found on. Equal keys are one contact; equal keys of different payload are a named conflict;
//! * an ENDPOINT contact ([`EndpointKey`]) is the same vertex meeting an END of a leaf, an EDGE contact ([`EdgeContact`]) two neighbours that collapse. Both are junction contacts
//!   ([`JunctionContact`]) with the junctions that die in them and the families they touch;
//! * THE PASSES are separate units, each of which can go (or be merged into one pair loop) without touching the others: the interior pass ([`discover_interior_split_contacts`]),
//!   the endpoint pass ([`discover_endpoint_contacts`]) and the edge pass ([`discover_symbolic_edge_contacts`]); [`discover_junction_contacts`] only calls the last two. The pairs the
//!   interior and the endpoint passes evaluate are the same pairs; each routes the candidate by `at_start` / `at_end`;
//! * every discovery asks the candidate laws over the view of the overlay, in the oracle's order (the leaves by `repr`, the emitters by `repr` of their junction, a contact found
//!   only when its time is exactly `now`), and so pays what the oracle pays.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::ExactCtx;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::builder::Builder;
use crate::candidate::{evaluate_edge_candidate, evaluate_split_candidate_gated, NowGate};
use crate::closure::span_family;
use crate::error::{SkelError, SkelResult};
use crate::omap::OrderedMap;
use crate::overlay::{edge_key_of, is_symbolic_split_emitter, with_view, JRef, Leaf, Overlay, SymVertex};
use crate::plans::{point_val, time_val, PlanVal};
use crate::proof::EdgeKey;
use crate::pyval::Val;
use crate::queue::{CandidateEvent, EventKind};
use crate::snapshot::{exact_point_key, incident, sparse_occurrences, time_key, Incident, Snapshot, VertexSnapshot};
use crate::time::{compare_times, EventPoint, EventTime, PointRef, TimeRef};

fn sum_val(sum: &SqrtSum) -> Val {
    Val::data("SqrtSumV1", vec![("terms", Val::terms_of(sum))])
}

/// The field of a key dataclass, or the refusal.
fn field_of(val: &Val, class: &str, name: &str) -> SkelResult<Val> {
    match val.as_data() {
        Some((found, _)) if found == class => val.field(name).cloned().ok_or_else(|| SkelError::Unsupported(format!("a {class} without the field {name}"))),
        _ => Err(SkelError::Unsupported(format!("a {class} expected, found {val:?}"))),
    }
}

fn keys_of_val(val: &Val) -> SkelResult<Vec<EdgeKey>> {
    val.items().ok_or_else(|| SkelError::Unsupported("a tuple of edge keys expected".to_string()))?.iter().map(|item| edge_key_of(item).ok_or_else(|| SkelError::Unsupported("an edge key expected".to_string()))).collect()
}

/// `a == b` of two `SqrtSumV1` (the value, whatever the Python type of a coefficient).
pub fn sum_eq(left: &SqrtSum, right: &SqrtSum) -> bool {
    left.canonical_form() == right.canonical_form()
}

/// `a == b` of two `EventTimeV1` as a dataclass compares them: the dividend and the divisor field by field (two proportional times are NOT equal).
pub fn time_eq(left: &EventTime, right: &EventTime) -> bool {
    left.dividend == right.dividend && sum_eq(&left.divisor, &right.divisor)
}

pub fn point_eq(left: &EventPoint, right: &EventPoint) -> bool {
    sum_eq(&left.x, &right.x) && sum_eq(&left.y, &right.y)
}

/// The gate of the law of a split candidate on the symbolic call path (the discoveries of one exact time). The oracle gates a time BEFORE `now` and then keeps only the contacts
/// found exactly AT `now` ([`found_at_now`]); a version of the oracle that gates `!= 0` in the law moves this one constant and drops the second check.
pub const SYMBOLIC_GATE: NowGate = NowGate::NotBefore;

/// The check after the law that a contact of a discovery is at exactly `now` (`compare_times(candidate.time, overlay.time) != 0` skips the candidate): one comparison, paid.
fn found_at_now(ctx: &mut ExactCtx<'_>, time: &EventTime, now: &EventTime) -> SkelResult<bool> {
    Ok(compare_times(ctx, time, now)? == 0)
}

fn repr_order(left: &Val, right: &Val) -> std::cmp::Ordering {
    left.repr().as_bytes().cmp(right.repr().as_bytes())
}

pub fn keys_val(keys: &[EdgeKey]) -> Val {
    Val::tuple(keys.iter().map(|key| Val::ints(key)).collect())
}

/// `_participants(*leaves)`: the sorted distinct keys of the edges the families of the leaves stand for.
pub fn participants_of(leaves: &[&Leaf]) -> Vec<EdgeKey> {
    leaves.iter().flat_map(|leaf| leaf.participant_keys()).collect::<BTreeSet<EdgeKey>>().into_iter().collect()
}

// --------------------------------------------------------------------------
// interior split contacts
// --------------------------------------------------------------------------

/// `SymbolicSplitContactKeyV1(time_key, point_key, emitter, family, participants)`.
#[derive(Clone)]
pub struct SplitKey {
    pub val: Val,
    pub point_key: Val,
    pub emitter: JRef,
    pub family: Val,
    pub participants: Vec<EdgeKey>,
}

impl SplitKey {
    pub fn new(time_key: Val, point_key: Val, emitter: JRef, family: Val, participants: Vec<EdgeKey>) -> SplitKey {
        let val = Val::data(
            "SymbolicSplitContactKeyV1",
            vec![("time_key", time_key), ("point_key", point_key.clone()), ("emitter", emitter.val().clone()), ("family", family.clone()), ("participants", keys_val(&participants))],
        );
        SplitKey { val, point_key, emitter, family, participants }
    }

    /// A key read back from a value (`SymbolicSplitContactKeyV1`).
    pub fn from_val(val: &Val) -> SkelResult<SplitKey> {
        const CLASS: &str = "SymbolicSplitContactKeyV1";
        Ok(SplitKey {
            val: val.clone(),
            point_key: field_of(val, CLASS, "point_key")?,
            emitter: JRef::from_val(&field_of(val, CLASS, "emitter")?)?,
            family: field_of(val, CLASS, "family")?,
            participants: keys_of_val(&field_of(val, CLASS, "participants")?)?,
        })
    }

    /// `key.family.occurrence`.
    pub fn occurrence(&self) -> Val {
        self.family.field("occurrence").cloned().unwrap_or_else(Val::none)
    }

    /// `key.family.participant_keys`.
    pub fn family_participants(&self) -> Vec<EdgeKey> {
        self.family.field("participant_keys").and_then(Val::items).map(|items| items.iter().filter_map(edge_key_of).collect()).unwrap_or_default()
    }
}

/// `SymbolicSplitContactV1(key, time, point, projection, leaf=None)`.
#[derive(Clone)]
pub struct SymSplitContact {
    pub key: SplitKey,
    pub time: TimeRef,
    pub point: PointRef,
    pub projection: SqrtSum,
    pub leaf: Option<Leaf>,
}

/// `left == right` of two contacts (a dataclass: every field, the times raw).
pub fn contacts_equal(left: &SymSplitContact, right: &SymSplitContact) -> bool {
    left.key.val == right.key.val && time_eq(&left.time, &right.time) && point_eq(&left.point, &right.point) && sum_eq(&left.projection, &right.projection) && left.leaf == right.leaf
}

impl PlanVal for SymSplitContact {
    fn to_val(&self) -> Val {
        Val::data(
            "SymbolicSplitContactV1",
            vec![
                ("key", self.key.val.clone()),
                ("time", time_val(&self.time)),
                ("point", point_val(&self.point)),
                ("projection", sum_val(&self.projection)),
                ("leaf", self.leaf.as_ref().map_or_else(Val::none, |leaf| leaf.val().clone())),
            ],
        )
    }
}

/// `merge_symbolic_split_contacts(*groups)`: equal stable contacts once, in the order of the `repr` of their keys; two contacts of one key that are not equal are the named
/// conflict (and then no contacts).
pub fn merge_symbolic_split_contacts(groups: &[&[SymSplitContact]]) -> (Vec<SymSplitContact>, Option<&'static str>) {
    let mut unique: OrderedMap<Val, SymSplitContact> = OrderedMap::new();
    for contact in groups.iter().flat_map(|group| group.iter()) {
        if unique.get(&contact.key.val).is_some_and(|previous| !contacts_equal(previous, contact)) {
            return (Vec::new(), Some("SYMBOLIC_INTERIOR_SPLIT_CONTACT_METADATA_CONFLICT"));
        }
        unique.insert(contact.key.val.clone(), contact.clone());
    }
    let mut ordered: Vec<SymSplitContact> = unique.values().cloned().collect();
    ordered.sort_by(|left, right| repr_order(&left.key.val, &right.key.val));
    (ordered, None)
}

/// `initial_interior_contacts(snapshot)`: the interior cuts of the frozen packet as stable contacts (a cut at an end of its span is not an interior one).
pub fn initial_interior_contacts(snapshot: &Snapshot) -> SkelResult<(Vec<SymSplitContact>, Option<&'static str>)> {
    let mut contacts = Vec::new();
    for found in &snapshot.incidents {
        let Some(occurrence) = &found.target_occurrence else {
            continue;
        };
        if found.event.kind != EventKind::Split || occurrence.get(1) == Some(&found.point_key) || occurrence.get(2) == Some(&found.point_key) {
            continue;
        }
        let projection = found.target_projection.clone().ok_or_else(|| SkelError::Unsupported("TypeError in the oracle: a split incident without a projection".to_string()))?;
        let key = SplitKey::new(time_key(&found.event.time)?, found.point_key.clone(), JRef::new("EXISTING", found.emitter_key.clone()), span_family(occurrence), found.participants.clone());
        contacts.push(SymSplitContact { key, time: Rc::clone(&found.event.time), point: Rc::clone(&found.event.point), projection, leaf: None });
    }
    Ok(merge_symbolic_split_contacts(&[&contacts]))
}

/// `_required_keys(contacts)`: the keys of the edges whose ends a compilation of the contacts must hydrate (the family of every contact and the two edges of an existing emitter).
fn required_keys(contacts: &[SymSplitContact]) -> HashSet<EdgeKey> {
    let mut required: HashSet<EdgeKey> = HashSet::new();
    for contact in contacts {
        required.extend(contact.key.occurrence().get(0).and_then(edge_key_of));
        if !contact.key.emitter.is_existing() {
            continue;
        }
        if let Some(items) = contact.key.emitter.key().items() {
            required.extend(items.iter().filter_map(edge_key_of));
        }
    }
    required
}

/// `_unique_map(pairs)`: the one value of every key, or none when a key has two.
fn unique_map<K: std::hash::Hash + Eq + Clone, V: Ord + Copy>(pairs: impl IntoIterator<Item = (K, V)>) -> Option<HashMap<K, V, FxBuild>> {
    let mut grouped: HashMap<K, BTreeSet<V>, FxBuild> = HashMap::default();
    for (key, value) in pairs {
        grouped.entry(key).or_default().insert(value);
    }
    let mut resolved: HashMap<K, V, FxBuild> = HashMap::default();
    for (key, values) in grouped {
        if values.len() != 1 {
            return None;
        }
        resolved.insert(key, *values.iter().next()?);
    }
    Some(resolved)
}

/// `_compile_contacts(builder, snapshot, contacts)`: the stable contacts as incidents of the frozen packet again, every root family rebound to the runtime edge that owns it and
/// every emitter to the runtime vertex that is that port; a reason when it cannot be done.
pub fn compile_contacts(ctx: &mut ExactCtx<'_>, builder: &mut Builder, vertices: &[VertexSnapshot], contacts: &[SymSplitContact]) -> SkelResult<(Vec<Incident>, Option<&'static str>)> {
    const REBIND: &str = "SYMBOLIC_SPLIT_ROOT_REBIND_UNRESOLVABLE";
    let Some(first) = contacts.first() else {
        return Ok((Vec::new(), None));
    };
    let found = sparse_occurrences(ctx, builder, &first.time, &required_keys(contacts))?;
    if !found.duplicate_owner_ids.is_empty() {
        return Ok((Vec::new(), Some(REBIND)));
    }
    let hydrated = vertices
        .iter()
        .map(|vertex| -> SkelResult<VertexSnapshot> {
            let point_key = found.point_keys.get(vertex.ident as usize).ok_or_else(|| SkelError::Unsupported("IndexError in the oracle: a vertex beyond the point keys".to_string()))?;
            Ok(VertexSnapshot { point_key: point_key.clone(), prev_occurrence: found.occurrences.get(&vertex.prev_edge).cloned(), next_occurrence: found.occurrences.get(&vertex.next_edge).cloned(), ..vertex.clone() })
        })
        .collect::<SkelResult<Vec<_>>>()?;
    let wanted: HashSet<&JRef> = contacts.iter().map(|contact| &contact.key.emitter).collect();
    let named: Vec<(JRef, i64)> = hydrated.iter().filter(|vertex| vertex.point_key.is_some()).map(|vertex| (JRef::new("EXISTING", crate::snapshot::port_identity(vertex)), vertex.ident)).filter(|(reference, _)| wanted.contains(reference)).collect();
    let emitters = unique_map(named);
    let roots = unique_map(hydrated.iter().flat_map(|vertex| [(vertex.prev_edge, &vertex.prev_occurrence), (vertex.next_edge, &vertex.next_occurrence)]).filter_map(|(edge, occurrence)| occurrence.as_ref().map(|found| (found.clone(), edge))));
    let (Some(emitters), Some(roots)) = (emitters, roots) else {
        return Ok((Vec::new(), Some(REBIND)));
    };
    let mut ordered: Vec<&SymSplitContact> = contacts.iter().collect();
    ordered.sort_by(|left, right| repr_order(&left.key.val, &right.key.val));
    let mut compiled = Vec::with_capacity(ordered.len());
    for contact in ordered {
        let (Some(emitter), Some(root)) = (emitters.get(&contact.key.emitter), roots.get(&contact.key.occurrence())) else {
            return Ok((Vec::new(), Some(REBIND)));
        };
        let event = CandidateEvent { kind: EventKind::Split, time: Rc::clone(&contact.time), point: Rc::clone(&contact.point), vertex: *emitter, peer: -1, edge: *root, span_unproven: false };
        let base = incident(ctx, builder, &event, &hydrated)?;
        compiled.push(Incident {
            participants: contact.key.participants.clone(),
            target_participants: contact.key.family_participants(),
            target_projection: Some(contact.projection.clone()),
            target_occurrence: Some(contact.key.occurrence()),
            emitter_key: contact.key.emitter.key(),
            sort_cache: std::cell::OnceCell::new(),
            identity_cache: std::cell::OnceCell::new(),
            ..base
        });
    }
    Ok((compiled, None))
}

/// The alive vertices of the overlay that emit splits (`is_symbolic_split_emitter`), in the order of the `repr` of their junctions: what every pass of the discoveries pairs with the
/// changed leaves.
fn emitters_of<'a>(builder: &Builder, overlay: &'a Overlay) -> SkelResult<Vec<&'a SymVertex>> {
    let mut emitters: Vec<&SymVertex> = Vec::new();
    for vertex in overlay.vertices.values() {
        if vertex.alive && is_symbolic_split_emitter(builder, overlay, vertex)? {
            emitters.push(vertex);
        }
    }
    emitters.sort_by(|left, right| repr_order(left.reference.val(), right.reference.val()));
    Ok(emitters)
}

/// `discover_interior_split_contacts(builder, overlay)`: every reflex or sliding vertex that meets the inside of a CHANGED leaf at exactly `now`, as stable contacts.
pub fn discover_interior_split_contacts(ctx: &mut ExactCtx<'_>, builder: &mut Builder, overlay: &Overlay) -> SkelResult<(Vec<SymSplitContact>, Option<&'static str>)> {
    let found = with_view(builder, overlay, |owner, view, memo| -> SkelResult<Vec<SymSplitContact>> {
        let emitters = emitters_of(owner, overlay)?;
        let mut contacts = Vec::new();
        for leaf in overlay.changed_by_repr() {
            for emitter in &emitters {
                if leaf == emitter.prev_leaf || leaf == emitter.next_leaf {
                    continue;
                }
                let decision = evaluate_split_candidate_gated(ctx, view, memo, view.vertex_ref(&emitter.reference)?, view.span_ref(&leaf)?, &overlay.time, SYMBOLIC_GATE)?;
                let Some(candidate) = decision.candidate else {
                    continue;
                };
                if !found_at_now(ctx, &candidate.time, &overlay.time)? || candidate.at_start || candidate.at_end {
                    continue;
                }
                let binding = overlay.spans.get(&leaf).ok_or_else(|| SkelError::Unsupported("KeyError in the oracle: a leaf that is not in the overlay".to_string()))?;
                let line = &owner.edge_at(binding.physical_edge_id)?.line;
                let projection = candidate.point.x.scaled_difference(&Rat::from_i64(line.b), &candidate.point.y, &Rat::from_i64(line.a));
                let key = SplitKey::new(time_key(&candidate.time)?, exact_point_key(&candidate.point), emitter.reference.clone(), leaf.family(), participants_of(&[&emitter.prev_leaf, &emitter.next_leaf, &leaf]));
                contacts.push(SymSplitContact { key, time: candidate.time, point: candidate.point, projection, leaf: Some(leaf.clone()) });
            }
        }
        Ok(contacts)
    })?;
    Ok(merge_symbolic_split_contacts(&[&found]))
}

// --------------------------------------------------------------------------
// endpoint contacts
// --------------------------------------------------------------------------

/// `EndpointContactKeyV1(time_key, point_key, emitter, endpoint, family, participants)`: a vertex meets the end of a span.
#[derive(Clone)]
pub struct EndpointKey {
    pub val: Val,
    pub time_key: Val,
    pub point_key: Val,
    pub emitter: JRef,
    pub endpoint: JRef,
    pub family: Val,
}

impl EndpointKey {
    pub fn new(time_key: Val, point_key: Val, emitter: JRef, endpoint: JRef, family: Val, participants: &[EdgeKey]) -> EndpointKey {
        let val = Val::data(
            "EndpointContactKeyV1",
            vec![
                ("time_key", time_key.clone()),
                ("point_key", point_key.clone()),
                ("emitter", emitter.val().clone()),
                ("endpoint", endpoint.val().clone()),
                ("family", family.clone()),
                ("participants", keys_val(participants)),
            ],
        );
        EndpointKey { val, time_key, point_key, emitter, endpoint, family }
    }
}

impl EndpointKey {
    /// A key read back from a value (`EndpointContactKeyV1`).
    pub fn from_val(val: &Val) -> SkelResult<EndpointKey> {
        const CLASS: &str = "EndpointContactKeyV1";
        Ok(EndpointKey {
            val: val.clone(),
            time_key: field_of(val, CLASS, "time_key")?,
            point_key: field_of(val, CLASS, "point_key")?,
            emitter: JRef::from_val(&field_of(val, CLASS, "emitter")?)?,
            endpoint: JRef::from_val(&field_of(val, CLASS, "endpoint")?)?,
            family: field_of(val, CLASS, "family")?,
        })
    }
}

/// `discover_endpoint_contacts(builder, overlay)`: every vertex that meets an END of a changed leaf at exactly `now`; the keys in the order of their `repr`, or the named
/// reason when the end that was met has no junction to name it.
pub fn discover_endpoint_contacts(ctx: &mut ExactCtx<'_>, builder: &mut Builder, overlay: &Overlay) -> SkelResult<(Vec<EndpointKey>, Option<&'static str>)> {
    with_view(builder, overlay, |owner, view, memo| -> SkelResult<(Vec<EndpointKey>, Option<&'static str>)> {
        let emitters = emitters_of(owner, overlay)?;
        let mut contacts: OrderedMap<Val, EndpointKey> = OrderedMap::new();
        for leaf in overlay.changed_by_repr() {
            let binding = overlay.spans.get(&leaf).ok_or_else(|| SkelError::Unsupported("KeyError in the oracle: a leaf that is not in the overlay".to_string()))?;
            for emitter in &emitters {
                if leaf == emitter.prev_leaf || leaf == emitter.next_leaf {
                    continue;
                }
                let decision = evaluate_split_candidate_gated(ctx, view, memo, view.vertex_ref(&emitter.reference)?, view.span_ref(&leaf)?, &overlay.time, SYMBOLIC_GATE)?;
                let Some(candidate) = decision.candidate else {
                    continue;
                };
                if !found_at_now(ctx, &candidate.time, &overlay.time)? || !(candidate.at_start || candidate.at_end) {
                    continue;
                }
                let mut endpoints: Vec<&JRef> = Vec::new();
                for (flag, endpoint) in [(candidate.at_start, &binding.start), (candidate.at_end, &binding.end)] {
                    if let (true, Some(found)) = (flag, endpoint) {
                        if !endpoints.contains(&found) {
                            endpoints.push(found);
                        }
                    }
                }
                if endpoints.is_empty() {
                    return Ok((Vec::new(), Some("SYMBOLIC_ENDPOINT_REFERENCE_AMBIGUOUS")));
                }
                for endpoint in endpoints {
                    let key = EndpointKey::new(
                        time_key(&candidate.time)?,
                        exact_point_key(&candidate.point),
                        emitter.reference.clone(),
                        endpoint.clone(),
                        leaf.family(),
                        &participants_of(&[&emitter.prev_leaf, &emitter.next_leaf, &leaf]),
                    );
                    contacts.insert(key.val.clone(), key);
                }
            }
        }
        let mut ordered: Vec<EndpointKey> = contacts.values().cloned().collect();
        ordered.sort_by(|left, right| repr_order(&left.val, &right.val));
        Ok((ordered, None))
    })
}

// --------------------------------------------------------------------------
// edge contacts
// --------------------------------------------------------------------------

/// `SymbolicEdgeContactKeyV1(time_key, point_key, start, end, prev_family, shared_family, next_family)`.
#[derive(Clone)]
pub struct EdgeContactKey {
    pub val: Val,
    pub time_key: Val,
    pub point_key: Val,
    pub start: JRef,
    pub end: JRef,
    pub prev_family: Val,
    pub shared_family: Val,
    pub next_family: Val,
}

impl EdgeContactKey {
    /// A key read back from a value (`SymbolicEdgeContactKeyV1`).
    pub fn from_val(val: &Val) -> SkelResult<EdgeContactKey> {
        const CLASS: &str = "SymbolicEdgeContactKeyV1";
        Ok(EdgeContactKey {
            val: val.clone(),
            time_key: field_of(val, CLASS, "time_key")?,
            point_key: field_of(val, CLASS, "point_key")?,
            start: JRef::from_val(&field_of(val, CLASS, "start")?)?,
            end: JRef::from_val(&field_of(val, CLASS, "end")?)?,
            prev_family: field_of(val, CLASS, "prev_family")?,
            shared_family: field_of(val, CLASS, "shared_family")?,
            next_family: field_of(val, CLASS, "next_family")?,
        })
    }
}

/// `SymbolicEdgeContactV1(key, prev_leaf, shared_leaf, next_leaf, span_unproven, participant_keys)`: two neighbours that collapse.
#[derive(Clone)]
pub struct EdgeContact {
    pub key: EdgeContactKey,
    pub prev_leaf: Leaf,
    pub shared_leaf: Leaf,
    pub next_leaf: Leaf,
    pub span_unproven: bool,
    pub participant_keys: Vec<EdgeKey>,
}

impl PlanVal for EdgeContact {
    fn to_val(&self) -> Val {
        Val::data(
            "SymbolicEdgeContactV1",
            vec![
                ("key", self.key.val.clone()),
                ("prev_leaf", self.prev_leaf.val().clone()),
                ("shared_leaf", self.shared_leaf.val().clone()),
                ("next_leaf", self.next_leaf.val().clone()),
                ("span_unproven", Val::boolean(self.span_unproven)),
                ("participant_keys", keys_val(&self.participant_keys)),
            ],
        )
    }
}

/// `discover_symbolic_edge_contacts(builder, overlay)`: every pair of neighbours of the overlay that collapses at exactly `now` (a pair of existing ports that no changed leaf
/// separates is skipped), in the order of the `repr` of their keys.
pub fn discover_symbolic_edge_contacts(ctx: &mut ExactCtx<'_>, builder: &mut Builder, overlay: &Overlay) -> SkelResult<Vec<EdgeContact>> {
    with_view(builder, overlay, |_owner, view, memo| -> SkelResult<Vec<EdgeContact>> {
        let mut references: Vec<&JRef> = overlay.vertices.keys().collect();
        references.sort_by(|left, right| repr_order(left.val(), right.val()));
        let mut contacts: Vec<EdgeContact> = Vec::new();
        for reference in references {
            let Some(vertex) = overlay.vertices.get(reference) else {
                continue;
            };
            let Some(next) = vertex.next.as_ref().filter(|_| vertex.alive) else {
                continue;
            };
            let Some(peer) = overlay.vertices.get(next).filter(|peer| peer.alive && peer.prev.as_ref() == Some(reference)) else {
                continue;
            };
            if !overlay.changed.contains(&vertex.next_leaf) && vertex.reference.is_existing() && peer.reference.is_existing() {
                continue;
            }
            let decision = evaluate_edge_candidate(ctx, view, memo, view.vertex_ref(&vertex.reference)?, view.vertex_ref(&peer.reference)?, &overlay.time, vertex.reference == peer.reference)?;
            let Some(candidate) = decision.candidate else {
                continue;
            };
            if !found_at_now(ctx, &candidate.time, &overlay.time)? {
                continue;
            }
            let (prev_family, shared_family, next_family) = (vertex.prev_leaf.family(), vertex.next_leaf.family(), peer.next_leaf.family());
            let (time_key, point_key) = (time_key(&candidate.time)?, exact_point_key(&candidate.point));
            let val = Val::data(
                "SymbolicEdgeContactKeyV1",
                vec![
                    ("time_key", time_key.clone()),
                    ("point_key", point_key.clone()),
                    ("start", vertex.reference.val().clone()),
                    ("end", peer.reference.val().clone()),
                    ("prev_family", prev_family.clone()),
                    ("shared_family", shared_family.clone()),
                    ("next_family", next_family.clone()),
                ],
            );
            contacts.push(EdgeContact {
                key: EdgeContactKey { val, time_key, point_key, start: vertex.reference.clone(), end: peer.reference.clone(), prev_family, shared_family, next_family },
                prev_leaf: vertex.prev_leaf.clone(),
                shared_leaf: vertex.next_leaf.clone(),
                next_leaf: peer.next_leaf.clone(),
                span_unproven: candidate.span_unproven,
                participant_keys: participants_of(&[&vertex.prev_leaf, &vertex.next_leaf, &peer.next_leaf]),
            });
        }
        contacts.sort_by(|left, right| repr_order(&left.key.val, &right.key.val));
        Ok(contacts)
    })
}

// --------------------------------------------------------------------------
// junction contacts
// --------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactKind {
    Edge,
    Endpoint,
}

impl ContactKind {
    pub fn value(self) -> &'static str {
        match self {
            ContactKind::Edge => "EDGE",
            ContactKind::Endpoint => "ENDPOINT",
        }
    }
}

/// `SymbolicJunctionContactV1(kind, key, dead_refs, families, edge=None)`.
#[derive(Clone)]
pub struct JunctionContact {
    pub kind: ContactKind,
    pub key: Val,
    /// `contact_identity(contact)`: `(kind, key)`.
    pub identity: Val,
    pub time_key: Val,
    pub point_key: Val,
    pub dead_refs: Vec<JRef>,
    pub families: Vec<Val>,
    pub edge: Option<EdgeContact>,
    pub endpoint: Option<EndpointKey>,
}

impl PlanVal for JunctionContact {
    fn to_val(&self) -> Val {
        Val::data(
            "SymbolicJunctionContactV1",
            vec![
                ("kind", Val::str(self.kind.value())),
                ("key", self.key.clone()),
                ("dead_refs", Val::tuple(self.dead_refs.iter().map(|found| found.val().clone()).collect())),
                ("families", Val::set(self.families.clone(), true)),
                ("edge", self.edge.as_ref().map_or_else(Val::none, PlanVal::to_val)),
            ],
        )
    }
}

fn sorted_pair(first: &JRef, second: &JRef) -> Vec<JRef> {
    let mut pair = vec![first.clone(), second.clone()];
    pair.sort_by(|left, right| repr_order(left.val(), right.val()));
    pair
}

/// `edge_contact(contact)`.
pub fn edge_contact(contact: EdgeContact) -> JunctionContact {
    let key = contact.key.val.clone();
    JunctionContact {
        kind: ContactKind::Edge,
        identity: Val::tuple(vec![Val::str("EDGE"), key.clone()]),
        time_key: contact.key.time_key.clone(),
        point_key: contact.key.point_key.clone(),
        dead_refs: sorted_pair(&contact.key.start, &contact.key.end),
        families: vec![contact.key.prev_family.clone(), contact.key.shared_family.clone(), contact.key.next_family.clone()],
        key,
        edge: Some(contact),
        endpoint: None,
    }
}

/// `endpoint_contact(overlay, key)`.
pub fn endpoint_contact(overlay: &Overlay, key: EndpointKey) -> JunctionContact {
    let mut families = vec![key.family.clone()];
    for reference in [&key.emitter, &key.endpoint] {
        if let Some(vertex) = overlay.vertices.get(reference) {
            families.extend([vertex.prev_leaf.family(), vertex.next_leaf.family()]);
        }
    }
    JunctionContact {
        kind: ContactKind::Endpoint,
        identity: Val::tuple(vec![Val::str("ENDPOINT"), key.val.clone()]),
        time_key: key.time_key.clone(),
        point_key: key.point_key.clone(),
        dead_refs: sorted_pair(&key.emitter, &key.endpoint),
        families,
        key: key.val.clone(),
        edge: None,
        endpoint: Some(key),
    }
}

/// `discover_junction_contacts(builder, overlay)`: the endpoint contacts, then the edge contacts, each once, in the order of the `repr` of their identity; or the named reason.
pub fn discover_junction_contacts(ctx: &mut ExactCtx<'_>, builder: &mut Builder, overlay: &Overlay) -> SkelResult<(Vec<JunctionContact>, Option<&'static str>)> {
    let (endpoints, reason) = discover_endpoint_contacts(ctx, builder, overlay)?;
    if reason.is_some() {
        return Ok((Vec::new(), reason));
    }
    let mut unique: OrderedMap<Val, JunctionContact> = OrderedMap::new();
    for contact in discover_symbolic_edge_contacts(ctx, builder, overlay)?.into_iter().map(edge_contact).chain(endpoints.into_iter().map(|key| endpoint_contact(overlay, key))) {
        unique.insert(contact.identity.clone(), contact);
    }
    let mut ordered: Vec<JunctionContact> = unique.values().cloned().collect();
    ordered.sort_by(|left, right| repr_order(&left.identity, &right.identity));
    Ok((ordered, None))
}

/// `_valid_contact(overlay, contact)` of the edge contacts: both junctions alive and still neighbours, along the same leaves and families the contact was found on.
pub fn valid_edge_contact(overlay: &Overlay, contact: &EdgeContact) -> bool {
    let (Some(start), Some(end)) = (overlay.vertices.get(&contact.key.start), overlay.vertices.get(&contact.key.end)) else {
        return false;
    };
    start.alive
        && end.alive
        && start.next.as_ref() == Some(&end.reference)
        && end.prev.as_ref() == Some(&start.reference)
        && start.prev_leaf == contact.prev_leaf
        && start.next_leaf == contact.shared_leaf
        && end.next_leaf == contact.next_leaf
        && start.prev_leaf.family() == contact.key.prev_family
        && start.next_leaf.family() == contact.key.shared_family
        && end.next_leaf.family() == contact.key.next_family
}
