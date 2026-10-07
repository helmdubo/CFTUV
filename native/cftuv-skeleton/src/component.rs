//! The order-free topology delta of a symbolic contact component and the signature of an overlay (`wavefront/symbolic_component.py`).
//!
//! * a [`Delta`] says which junctions of the overlay die, which two ports (an incoming and an outgoing arm of the boundary of the dead component) are joined, and by which
//!   junction; [`normalize_dead_component`] derives it for one component of dead junctions (the terminal 0/0 or the unique 1/1 pairing), [`apply_component_deltas`] applies disjoint
//!   deltas as ONE simultaneous generation and checks the reciprocity of what is left;
//! * the SIGNATURE of an overlay ([`overlay_signature`]) is its alive vertices, its spans and its changed leaves in the order of their `repr`, with the births in canonical time and
//!   the trace of a vertex as its authority. The oracle compares two signatures with `==` (the repeat of a closure must give the same one) and keeps them as the `signatures` of the
//!   result; a vertex row is ordered by the text of its row, which is decided by the junction (the first item of a row, whose `repr` is never a prefix of another's), so the text of
//!   the provenance, the one item the oracle prints in hash order, never decides.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::num::UBig;
use cftuv_core::rat::Coef;
use cftuv_core::sqrt_sum::{SqrtSum, Term};

use crate::builder::SlidingValue;
use crate::error::{SkelError, SkelResult};
use crate::overlay::{frozen_keys, refreshed_span_bindings, JRef, Leaf, Overlay, SymVertex, TraceInfo};
use crate::plans::{point_val, time_val, PlanVal};
use crate::pyval::Val;
use crate::time::{EventPoint, EventTime, PointRef, TimeRef};

/// `(prev junction | None, leaf)`: one arm of the boundary of a component: the junction it comes from and the leaf it runs along.
pub type Port = (Option<JRef>, Leaf);

/// `SymbolicComponentDeltaV1`.
#[derive(Clone)]
pub struct Delta {
    pub contact_keys: Vec<Val>,
    pub dead_refs: Vec<JRef>,
    pub incoming: Option<Port>,
    pub outgoing: Option<Port>,
    pub birth_ref: Option<JRef>,
    pub point_key: Val,
    pub leaf_resources: Vec<Leaf>,
    pub rewires: Vec<(Port, Port, JRef)>,
}

fn repr_order(left: &Val, right: &Val) -> std::cmp::Ordering {
    left.repr().as_bytes().cmp(right.repr().as_bytes())
}

fn port_val(port: &Option<Port>) -> Val {
    match port {
        None => Val::none(),
        Some(found) => port_pair(found),
    }
}

fn port_pair(port: &Port) -> Val {
    Val::tuple(vec![port.0.as_ref().map_or_else(Val::none, |found| found.val().clone()), port.1.val().clone()])
}

impl PlanVal for Delta {
    fn to_val(&self) -> Val {
        Val::data(
            "SymbolicComponentDeltaV1",
            vec![
                ("contact_keys", Val::tuple(self.contact_keys.clone())),
                ("dead_refs", Val::tuple(self.dead_refs.iter().map(|found| found.val().clone()).collect())),
                ("incoming", port_val(&self.incoming)),
                ("outgoing", port_val(&self.outgoing)),
                ("birth_ref", self.birth_ref.as_ref().map_or_else(Val::none, |found| found.val().clone())),
                ("point_key", self.point_key.clone()),
                ("leaf_resources", Val::set(self.leaf_resources.iter().map(|leaf| leaf.val().clone()).collect(), true)),
                ("rewires", Val::tuple(self.rewires.iter().map(|(incoming, outgoing, birth)| Val::tuple(vec![port_pair(incoming), port_pair(outgoing), birth.val().clone()])).collect())),
            ],
        )
    }
}

/// `sorted(items, key=repr)` of values that are refs or leaves.
pub fn by_repr<T: Clone>(items: &[T], val: impl Fn(&T) -> &Val) -> Vec<T> {
    let mut sorted: Vec<T> = items.to_vec();
    sorted.sort_by(|left, right| repr_order(val(left), val(right)));
    sorted
}

/// `point_from_key(key)`: the place a point key `(x terms, y terms)` names, the terms as they stand (a coefficient keeps its Python type).
pub fn point_from_key(key: &Val) -> SkelResult<EventPoint> {
    let sum = |terms: Option<&Val>| -> SkelResult<SqrtSum> {
        let refusal = || SkelError::Unsupported("a point key whose terms are not (radicand, coefficient) pairs".to_string());
        let mut found = Vec::new();
        for term in terms.and_then(Val::items).ok_or_else(refusal)? {
            let (radicand, coefficient) = (term.get(0).and_then(Val::as_int).ok_or_else(refusal)?, term.get(1).ok_or_else(refusal)?);
            let coef = match (coefficient.as_int(), coefficient.as_frac()) {
                (Some(whole), _) => Coef::int(whole.clone()),
                (None, Some(fraction)) => Coef::fraction(fraction.clone()),
                (None, None) => return Err(refusal()),
            };
            found.push(Term { radicand: UBig::try_from(radicand.clone()).map_err(|_| refusal())?, coef });
        }
        SqrtSum::from_terms(found).map_err(|error| SkelError::Unsupported(format!("a point key that is not canonical: {error:?}")))
    };
    Ok(EventPoint { x: sum(key.get(0))?, y: sum(key.get(1))? })
}

/// `normalize_dead_component(overlay, contact_keys=..., dead_refs=..., point_key=..., birth_kind=..., stale_reason=..., ambiguity_reason=...)`: the terminal 0/0 or the unique
/// 1/1 boundary-arm pairing of a component of dead junctions, or the named reason.
pub fn normalize_dead_component(
    overlay: &Overlay,
    contact_keys: &[Val],
    dead_refs: &[JRef],
    point_key: &Val,
    birth_kind: &str,
    stale_reason: &'static str,
    ambiguity_reason: &'static str,
) -> (Option<Delta>, Option<&'static str>) {
    let dead: HashSet<&JRef> = dead_refs.iter().collect();
    if dead.iter().any(|reference| !overlay.vertices.get(reference).is_some_and(|vertex| vertex.alive)) {
        return (None, Some(stale_reason));
    }
    let outside = |reference: &Option<JRef>| reference.as_ref().is_none_or(|found| !dead.contains(found));
    let mut incoming: Vec<Port> = Vec::new();
    let mut outgoing: Vec<Port> = Vec::new();
    for reference in &dead {
        let Some(vertex) = overlay.vertices.get(reference) else {
            continue;
        };
        if outside(&vertex.prev) && !incoming.iter().any(|(known, leaf)| *known == vertex.prev && *leaf == vertex.prev_leaf) {
            incoming.push((vertex.prev.clone(), vertex.prev_leaf.clone()));
        }
        if outside(&vertex.next) && !outgoing.iter().any(|(known, leaf)| *known == vertex.next && *leaf == vertex.next_leaf) {
            outgoing.push((vertex.next.clone(), vertex.next_leaf.clone()));
        }
    }
    if !matches!((incoming.len(), outgoing.len()), (0, 0) | (1, 1)) {
        return (None, Some(ambiguity_reason));
    }
    let (in_port, out_port) = (incoming.into_iter().next(), outgoing.into_iter().next());
    if let (Some(inbound), Some(outbound)) = (&in_port, &out_port) {
        let predecessor = inbound.0.as_ref().and_then(|found| overlay.vertices.get(found));
        let successor = outbound.0.as_ref().and_then(|found| overlay.vertices.get(found));
        let consistent = match (predecessor, successor) {
            (Some(before), Some(after)) => before.alive && after.alive && !outside(&before.next) && !outside(&after.prev),
            _ => false,
        };
        if !consistent {
            return (None, Some(ambiguity_reason));
        }
    }
    let ordered_keys = by_repr(contact_keys, |key| key);
    let birth = in_port.as_ref().map(|_| JRef::new(birth_kind, Val::tuple(ordered_keys.clone())));
    let leaves: Vec<Leaf> = dead.iter().filter_map(|reference| overlay.vertices.get(reference)).flat_map(|vertex| [vertex.prev_leaf.clone(), vertex.next_leaf.clone()]).collect();
    let leaf_set = Val::set(leaves.iter().map(|leaf| leaf.val().clone()).collect(), true);
    let leaf_resources: Vec<Leaf> = leaf_set.set_items().unwrap_or(&[]).iter().filter_map(|item| Leaf::from_val(item).ok()).collect();
    let rewires = match (&in_port, &out_port, &birth) {
        (Some(inbound), Some(outbound), Some(born)) => vec![(inbound.clone(), outbound.clone(), born.clone())],
        _ => Vec::new(),
    };
    let mut unique_dead: Vec<JRef> = Vec::new();
    for reference in dead_refs {
        if !unique_dead.contains(reference) {
            unique_dead.push(reference.clone());
        }
    }
    let sorted_dead = by_repr(&unique_dead, |reference| reference.val());
    (Some(Delta { contact_keys: ordered_keys, dead_refs: sorted_dead, incoming: in_port, outgoing: out_port, birth_ref: birth, point_key: point_key.clone(), leaf_resources, rewires }), None)
}

/// `rewires(delta)` of `apply_component_deltas`: the delta's own, or the one pairing a birth with its two ports.
fn rewires_of(delta: &Delta) -> SkelResult<Vec<(Port, Port, JRef)>> {
    if !delta.rewires.is_empty() {
        return Ok(delta.rewires.clone());
    }
    match (&delta.birth_ref, &delta.incoming, &delta.outgoing) {
        (None, _, _) => Ok(Vec::new()),
        (Some(birth), Some(incoming), Some(outgoing)) => Ok(vec![(incoming.clone(), outgoing.clone(), birth.clone())]),
        _ => Err(SkelError::Unsupported("TypeError in the oracle: a delta with a birth and without its ports".to_string())),
    }
}

/// `apply_component_deltas(overlay, deltas, collision_reason=...)`: disjoint normalised components as one simultaneous generation; the new overlay, or the named reason.
pub fn apply_component_deltas(overlay: &Overlay, deltas: &[Delta], collision_reason: &'static str) -> SkelResult<(Option<Overlay>, Option<&'static str>)> {
    let mut result = overlay.clone();
    let mut dead: HashSet<&JRef> = HashSet::new();
    let mut dead_count = 0usize;
    for delta in deltas {
        dead.extend(delta.dead_refs.iter());
        dead_count += delta.dead_refs.len();
    }
    if dead_count != dead.len() {
        return Ok((None, Some(collision_reason)));
    }
    let mut births: Vec<JRef> = Vec::new();
    for delta in deltas {
        births.extend(rewires_of(delta)?.into_iter().map(|(_, _, birth)| birth));
    }
    let distinct: HashSet<&JRef> = births.iter().collect();
    if distinct.len() != births.len() || births.iter().any(|birth| result.vertices.contains_key(birth)) {
        return Ok((None, Some(collision_reason)));
    }
    for delta in deltas {
        for reference in &delta.dead_refs {
            result.vertices.get_mut(reference).ok_or_else(|| SkelError::Unsupported("KeyError in the oracle: a dead junction that is not in the overlay".to_string()))?.alive = false;
        }
    }
    let point_keys = deltas.iter().map(|delta| point_from_key(&delta.point_key)).collect::<SkelResult<Vec<_>>>()?;
    for (delta, point) in deltas.iter().zip(point_keys) {
        let mut provenance: Vec<Val> = Vec::new();
        for reference in &delta.dead_refs {
            provenance.extend(result.vertices.get(reference).ok_or_else(|| SkelError::Unsupported("KeyError in the oracle: a dead junction that is not in the overlay".to_string()))?.provenance.iter().cloned());
        }
        let provenance = frozen_keys(provenance);
        let point: PointRef = Rc::new(point);
        for (incoming, outgoing, birth) in rewires_of(delta)? {
            let (prev_leaf, next_leaf) = (incoming.1, outgoing.1);
            result.vertices.insert(
                birth.clone(),
                SymVertex {
                    reference: birth,
                    prev: None,
                    next: None,
                    prev_leaf: prev_leaf.clone(),
                    next_leaf: next_leaf.clone(),
                    birth: Rc::clone(&result.time),
                    point: Some(Rc::clone(&point)),
                    sliding: None,
                    provenance: Rc::clone(&provenance),
                    runtime_id: None,
                    trace: TraceInfo::Absent,
                    alive: true,
                },
            );
            result.changed.insert(prev_leaf);
            result.changed.insert(next_leaf);
        }
    }
    let mut starts: HashMap<Leaf, Vec<JRef>, FxBuild> = HashMap::default();
    let mut ends: HashMap<Leaf, Vec<JRef>, FxBuild> = HashMap::default();
    for vertex in result.vertices.values() {
        if !vertex.alive {
            continue;
        }
        starts.entry(vertex.next_leaf.clone()).or_default().push(vertex.reference.clone());
        ends.entry(vertex.prev_leaf.clone()).or_default().push(vertex.reference.clone());
    }
    if starts.values().chain(ends.values()).any(|refs| refs.len() != 1) {
        return Ok((None, Some("SYMBOLIC_EDGE_SPAN_OWNER_AMBIGUOUS")));
    }
    for vertex in result.vertices.values_mut() {
        if !vertex.alive {
            continue;
        }
        let (Some(previous), Some(following)) = (starts.get(&vertex.prev_leaf), ends.get(&vertex.next_leaf)) else {
            return Ok((None, Some("SYMBOLIC_JUNCTION_RECIPROCITY_UNRESOLVABLE")));
        };
        vertex.prev = Some(previous[0].clone());
        vertex.next = Some(following[0].clone());
    }
    let (bindings, reason) = refreshed_span_bindings(&result);
    if let Some(reason) = reason {
        return Ok((None, Some(reason)));
    }
    result.spans = bindings.unwrap_or_default();
    for vertex in result.vertices.values() {
        if !vertex.alive {
            continue;
        }
        let lookup = |reference: &Option<JRef>| reference.as_ref().and_then(|found| result.vertices.get(found));
        let (previous, following) = (lookup(&vertex.prev), lookup(&vertex.next));
        let consistent = match (previous, following) {
            (Some(before), Some(after)) => before.alive && after.alive && before.next.as_ref() == Some(&vertex.reference) && after.prev.as_ref() == Some(&vertex.reference),
            _ => false,
        };
        if !consistent {
            return Ok((None, Some("SYMBOLIC_JUNCTION_RECIPROCITY_UNRESOLVABLE")));
        }
    }
    Ok((Some(result), None))
}

// --------------------------------------------------------------------------
// the signature
// --------------------------------------------------------------------------

/// `_SignaturePartsMemo`: the value of an immutable part of a signature once per object (a pure cache: the signatures of one transaction are taken from clones of one overlay,
/// whose births, points and projections are the same objects).
#[derive(Default)]
pub struct SignatureMemo {
    births: HashMap<*const EventTime, (TimeRef, Val)>,
    points: HashMap<*const EventPoint, (PointRef, Val)>,
    slides: HashMap<*const SlidingValue, (Rc<SlidingValue>, Val)>,
    crashes: HashMap<*const EventTime, (TimeRef, Val)>,
}

impl SignatureMemo {
    pub fn new() -> SignatureMemo {
        SignatureMemo::default()
    }

    /// `birth.canonical()` as a value.
    fn birth(&mut self, birth: &TimeRef) -> SkelResult<Val> {
        if let Some((_, found)) = self.births.get(&Rc::as_ptr(birth)) {
            return Ok(found.clone());
        }
        let value = time_val(&birth.canonical()?);
        self.births.insert(Rc::as_ptr(birth), (Rc::clone(birth), value.clone()));
        Ok(value)
    }

    fn point(&mut self, point: &PointRef) -> Val {
        if let Some((_, found)) = self.points.get(&Rc::as_ptr(point)) {
            return found.clone();
        }
        let value = point_val(point);
        self.points.insert(Rc::as_ptr(point), (Rc::clone(point), value.clone()));
        value
    }

    fn slide(&mut self, sliding: &Rc<SlidingValue>) -> Val {
        if let Some((_, found)) = self.slides.get(&Rc::as_ptr(sliding)) {
            return found.clone();
        }
        let value = Val::data("SqrtSumV1", vec![("terms", Val::terms_of(&sliding.value))]);
        self.slides.insert(Rc::as_ptr(sliding), (Rc::clone(sliding), value.clone()));
        value
    }

    /// `trace_authority(trace)`: what a vertex's trace says about its life.
    fn authority(&mut self, trace: &TraceInfo) -> SkelResult<Val> {
        let crash = match trace {
            TraceInfo::Absent => return Ok(Val::tuple(vec![Val::str("UNAVAILABLE")])),
            TraceInfo::WithoutCrash => return Ok(Val::tuple(vec![Val::str("TRACE_WITHOUT_CRASH")])),
            TraceInfo::Crash(crash) => crash,
        };
        if let Some((_, found)) = self.crashes.get(&Rc::as_ptr(crash)) {
            return Ok(found.clone());
        }
        let value = Val::tuple(vec![Val::str("BOUNDED"), time_val(&crash.canonical()?)]);
        self.crashes.insert(Rc::as_ptr(crash), (Rc::clone(crash), value.clone()));
        Ok(value)
    }
}

fn shown(value: &Option<Val>) -> String {
    value.as_ref().map_or_else(|| "None".to_string(), |found| found.repr().to_string())
}

fn joined(texts: &[String]) -> String {
    format!("({})", texts.join(", "))
}

/// `overlay_signature(overlay)`: `(vertices, spans, changed)`: the alive vertices as rows ordered by their text, every span as a row ordered by its text, the changed leaves
/// by `repr`. A value that compares equal to the oracle's tuple exactly when the oracle's tuples do.
pub fn overlay_signature(overlay: &Overlay, memo: &mut SignatureMemo) -> SkelResult<Val> {
    let mut rows: Vec<(String, Val)> = Vec::new();
    for vertex in overlay.vertices.values() {
        if !vertex.alive {
            continue;
        }
        let birth = memo.birth(&vertex.birth)?;
        let authority = memo.authority(&vertex.trace)?;
        let point = vertex.point.as_ref().map(|found| memo.point(found));
        let sliding = vertex.sliding.as_ref().map(|found| memo.slide(found));
        let provenance = Val::set(vertex.provenance.to_vec(), true);
        let prev = vertex.prev.as_ref().map(|found| found.val().clone());
        let next = vertex.next.as_ref().map(|found| found.val().clone());
        let text = joined(&[
            vertex.reference.val().repr().to_string(),
            shown(&prev),
            shown(&next),
            vertex.prev_leaf.val().repr().to_string(),
            vertex.next_leaf.val().repr().to_string(),
            birth.repr().to_string(),
            shown(&point),
            shown(&sliding),
            provenance.repr().to_string(),
            authority.repr().to_string(),
        ]);
        let orphan = |value: Option<Val>| value.unwrap_or_else(Val::none);
        rows.push((
            text,
            Val::tuple(vec![
                vertex.reference.val().clone(),
                orphan(prev),
                orphan(next),
                vertex.prev_leaf.val().clone(),
                vertex.next_leaf.val().clone(),
                birth,
                orphan(point),
                orphan(sliding),
                provenance,
                authority,
            ]),
        ));
    }
    rows.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
    let vertices = Val::tuple(rows.into_iter().map(|(_, row)| row).collect());
    let mut spans: Vec<(String, Val)> = overlay
        .spans
        .iter()
        .map(|(leaf, binding)| {
            let (start, end) = (binding.start.as_ref().map(|found| found.val().clone()), binding.end.as_ref().map(|found| found.val().clone()));
            let text = joined(&[leaf.val().repr().to_string(), binding.physical_edge_id.to_string(), shown(&start), shown(&end)]);
            (text, Val::tuple(vec![leaf.val().clone(), Val::int(binding.physical_edge_id), start.unwrap_or_else(Val::none), end.unwrap_or_else(Val::none)]))
        })
        .collect();
    spans.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
    let spans = Val::tuple(spans.into_iter().map(|(_, row)| row).collect());
    let changed = Val::tuple(overlay.changed_by_repr().iter().map(|leaf| leaf.val().clone()).collect());
    Ok(Val::tuple(vec![vertices, spans, changed]))
}

/// The overlay of the oracle's `clone_overlay`: the dictionaries are copied, the immutable parts are shared.
pub fn clone_overlay(overlay: &Overlay) -> Overlay {
    overlay.clone()
}
