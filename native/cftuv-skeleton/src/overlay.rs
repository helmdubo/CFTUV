//! The symbolic overlay of one exact-time packet (`wavefront/symbolic_overlay.py`, `symbolic_f0_overlay.py`, `symbolic_sparse_ports.py`, and `_existing_edges_by_occurrence` of
//! `superlevel.py`): the junctions of the front by an identity that holds no runtime id, the leaves (spans) they run along, and the exact view of the candidate laws over both.
//!
//! * a JUNCTION is [`JRef`] (`JunctionRefV1(kind, key)`): an existing port is named by the pair of its source edges, a port that a contact will birth by what the contact is;
//! * a LEAF is [`Leaf`] (`SegmentRefV1`): the span of one edge occurrence, the key of every span of the overlay;
//! * the OVERLAY is two insertion-ordered dictionaries ([`OrderedMap`]) and a set of changed leaves: the vertices and the bindings of the leaves to their physical edge and to the
//!   two junctions that start and end them. Its order is the oracle's wherever the oracle builds a new overlay from an old one (`refreshed_span_bindings`, `clone_overlay`);
//! * the VIEW ([`OverlayView`]) answers the candidate laws what the oracle's `exact_overlay_view` answers: the references of a vertex and of a span are the slots of the two
//!   dictionaries, a vertex that is not an existing port has its sliding projection made anew on EVERY `vertex_state` call (a new object in Python, so it never hits the
//!   identity-keyed memory of times: the view hands it a fresh identity each time), and an end of a span born at exactly `now` stands at its birth (`_born_place`).
//!
//! The order in which the oracle builds `spans` from the SET of occurrences (`_occurrences`) is the hash order of tuples that hold `None`, and nothing reads it: every
//! iteration over `spans` that decides something sorts by `repr` or asks `any`. The port builds them in the order the occurrences are met.

use std::cell::OnceCell;
use std::collections::{BTreeSet, HashMap, HashSet};
use std::hash::{Hash, Hasher};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::UBig;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::builder::{is_reflex, Builder, SlidingValue};
use crate::closure::{span_family, Materialization};
use crate::error::{SkelError, SkelResult};
use crate::line::SupportLine;
use crate::omap::OrderedMap;
use crate::plans::{point_val, time_val, ComponentPlan};
use crate::pyval::Val;
use crate::snapshot::{exact_point_key, port_identity, sparse_occurrences, VertexSnapshot};
use crate::time::{times_are_equal, EventPoint, EventTime, PointRef, TimeRef};
use crate::view::{sliding_projection, CandidateView, PositionMemo, Sliding, SpanRef, SpanState, VertexRef, VertexState};

fn key_error(what: &str) -> SkelError {
    SkelError::Unsupported(format!("KeyError in the oracle: {what}"))
}

/// The ints of a key that is a tuple of ints (an edge key), or none for any other value.
pub fn edge_key_of(value: &Val) -> Option<Vec<i64>> {
    value.items()?.iter().map(Val::as_i64).collect()
}

// --------------------------------------------------------------------------
// junctions and leaves
// --------------------------------------------------------------------------

/// `JunctionRefV1(kind, key)`: a junction of the overlay. `kind` is `EXISTING`, `BIRTH`, `INTERIOR_SPLIT`, `JUNCTION`, `EDGE_JUNCTION` or `VIRTUAL_BOUNDARY`; two refs are equal
/// when their kind and key are.
#[derive(Clone)]
pub struct JRef {
    val: Val,
    existing: bool,
    boundary: bool,
}

impl JRef {
    pub fn new(kind: &str, key: Val) -> JRef {
        JRef { existing: kind == "EXISTING", boundary: kind == "VIRTUAL_BOUNDARY", val: Val::data("JunctionRefV1", vec![("kind", Val::str(kind)), ("key", key)]) }
    }

    /// A ref read back from a value (`JunctionRefV1` with its two fields), or the refusal.
    pub fn from_val(val: &Val) -> SkelResult<JRef> {
        match (val.as_data(), val.field("kind").and_then(Val::as_str), val.field("key")) {
            (Some(("JunctionRefV1", _)), Some(kind), Some(_)) => Ok(JRef { existing: kind == "EXISTING", boundary: kind == "VIRTUAL_BOUNDARY", val: val.clone() }),
            _ => Err(SkelError::Unsupported(format!("a junction reference expected, found {:?}", val))),
        }
    }

    pub fn val(&self) -> &Val {
        &self.val
    }

    pub fn key(&self) -> Val {
        self.val.field("key").cloned().unwrap_or_else(Val::none)
    }

    pub fn kind(&self) -> &str {
        self.val.field("kind").and_then(Val::as_str).unwrap_or("")
    }

    pub fn is_existing(&self) -> bool {
        self.existing
    }

    pub fn is_virtual_boundary(&self) -> bool {
        self.boundary
    }
}

impl PartialEq for JRef {
    fn eq(&self, other: &JRef) -> bool {
        self.val == other.val
    }
}

impl Eq for JRef {}

impl Hash for JRef {
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write_u64(self.val.hash_value());
    }
}

impl std::fmt::Debug for JRef {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.val.repr())
    }
}

/// `SegmentRefV1(family, start, end, occurrence)`: the span of one edge occurrence (a leaf), by value.
#[derive(Clone)]
pub struct Leaf(Val);

impl Leaf {
    pub fn new(family: Val, start: Val, end: Val, occurrence: Val) -> Leaf {
        Leaf(Val::data("SegmentRefV1", vec![("family", family), ("start", start), ("end", end), ("occurrence", occurrence)]))
    }

    /// The leaf of an occurrence no split has refined: `SegmentRefV1(SpanFamilyRefV1(occurrence, (occurrence[0],)), None, None, occurrence)`.
    pub fn of_occurrence(occurrence: &Val) -> Leaf {
        Leaf::new(span_family(occurrence), Val::none(), Val::none(), occurrence.clone())
    }

    pub fn from_val(val: &Val) -> SkelResult<Leaf> {
        let complete = matches!(val.as_data(), Some(("SegmentRefV1", _))) && ["family", "start", "end", "occurrence"].iter().all(|name| val.field(name).is_some());
        if complete {
            Ok(Leaf(val.clone()))
        } else {
            Err(SkelError::Unsupported(format!("a leaf (SegmentRefV1) expected, found {:?}", val)))
        }
    }

    pub fn val(&self) -> &Val {
        &self.0
    }

    pub fn family(&self) -> Val {
        self.0.field("family").cloned().unwrap_or_else(Val::none)
    }

    pub fn occurrence(&self) -> Val {
        self.0.field("occurrence").cloned().unwrap_or_else(Val::none)
    }

    pub fn start(&self) -> Val {
        self.0.field("start").cloned().unwrap_or_else(Val::none)
    }

    pub fn end(&self) -> Val {
        self.0.field("end").cloned().unwrap_or_else(Val::none)
    }

    /// `leaf.family.participant_keys`: the keys of the edges the family stands for.
    pub fn participant_keys(&self) -> Vec<Vec<i64>> {
        self.family().field("participant_keys").and_then(Val::items).map(|items| items.iter().filter_map(edge_key_of).collect()).unwrap_or_default()
    }
}

impl PartialEq for Leaf {
    fn eq(&self, other: &Leaf) -> bool {
        self.0 == other.0
    }
}

impl Eq for Leaf {}

impl Hash for Leaf {
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write_u64(self.0.hash_value());
    }
}

impl std::fmt::Debug for Leaf {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0.repr())
    }
}

/// `frozenset` of keys as the overlay keeps it: unique, in the canonical order of `repr`.
pub fn frozen_keys(items: impl IntoIterator<Item = Val>) -> Rc<Vec<Val>> {
    let set = Val::set(items.into_iter().collect(), true);
    Rc::new(set.set_items().map(<[Val]>::to_vec).unwrap_or_default())
}

// --------------------------------------------------------------------------
// the overlay
// --------------------------------------------------------------------------

/// What a vertex says about the `trace` it holds (`SymbolicVertexV1.trace`): an existing port has the trace the builder gave it, any other vertex none. Only the crash time of
/// a trace is part of a signature; the trace itself is read from the builder by `runtime_id` where its bound is asked.
#[derive(Clone)]
pub enum TraceInfo {
    Absent,
    WithoutCrash,
    Crash(TimeRef),
}

impl TraceInfo {
    /// The trace of the builder that a vertex of this runtime id holds.
    pub fn of_builder(builder: &Builder, runtime_id: Option<i64>) -> TraceInfo {
        match runtime_id.and_then(|ident| builder.traces.get(&ident)) {
            None => TraceInfo::Absent,
            Some(trace) => trace.crash_time.as_ref().map_or(TraceInfo::WithoutCrash, |crash| TraceInfo::Crash(Rc::clone(crash))),
        }
    }
}

/// `SymbolicVertexV1`.
#[derive(Clone)]
pub struct SymVertex {
    pub reference: JRef,
    pub prev: Option<JRef>,
    pub next: Option<JRef>,
    pub prev_leaf: Leaf,
    pub next_leaf: Leaf,
    pub birth: TimeRef,
    pub point: Option<PointRef>,
    pub sliding: Option<Rc<SlidingValue>>,
    pub provenance: Rc<Vec<Val>>,
    pub runtime_id: Option<i64>,
    pub trace: TraceInfo,
    pub alive: bool,
}

/// `SymbolicSpanBindingV1`.
#[derive(Clone)]
pub struct Binding {
    pub leaf: Leaf,
    pub physical_edge_id: i64,
    pub start: Option<JRef>,
    pub end: Option<JRef>,
}

/// `SymbolicOverlayV1`.
#[derive(Clone)]
pub struct Overlay {
    pub vertices: OrderedMap<JRef, SymVertex>,
    pub spans: OrderedMap<Leaf, Binding>,
    pub changed: HashSet<Leaf, FxBuild>,
    pub time: TimeRef,
}

impl Overlay {
    /// `sorted(overlay.changed, key=repr)`.
    pub fn changed_by_repr(&self) -> Vec<Leaf> {
        let mut leaves: Vec<Leaf> = self.changed.iter().cloned().collect();
        leaves.sort_by(|left, right| left.val().repr().as_bytes().cmp(right.val().repr().as_bytes()));
        leaves
    }
}

/// `refreshed_span_bindings(overlay)`: the active leaf owners rebound, refusing multiplicity instead of the last one winning.
pub fn refreshed_span_bindings(overlay: &Overlay) -> (Option<OrderedMap<Leaf, Binding>>, Option<&'static str>) {
    let mut starts: HashMap<Leaf, Vec<JRef>, FxBuild> = HashMap::default();
    let mut ends: HashMap<Leaf, Vec<JRef>, FxBuild> = HashMap::default();
    for vertex in overlay.vertices.values() {
        if !vertex.alive {
            continue;
        }
        starts.entry(vertex.next_leaf.clone()).or_default().push(vertex.reference.clone());
        ends.entry(vertex.prev_leaf.clone()).or_default().push(vertex.reference.clone());
    }
    if starts.values().chain(ends.values()).any(|refs| refs.len() != 1) {
        return (None, Some("SYMBOLIC_EDGE_SPAN_OWNER_AMBIGUOUS"));
    }
    let mut bindings = OrderedMap::new();
    for (leaf, binding) in overlay.spans.iter() {
        bindings.insert(
            leaf.clone(),
            Binding { leaf: leaf.clone(), physical_edge_id: binding.physical_edge_id, start: starts.get(leaf).map(|refs| refs[0].clone()), end: ends.get(leaf).map(|refs| refs[0].clone()) },
        );
    }
    (Some(bindings), None)
}

// --------------------------------------------------------------------------
// sparse ports
// --------------------------------------------------------------------------

fn line_occurrence(builder: &Builder, edge: i64) -> SkelResult<Val> {
    Ok(Val::tuple(vec![Val::ints(builder.edge_at(edge)?.span.as_slice()), Val::none(), Val::none()]))
}

/// `with_line_ports(builder, snapshot, time)`: the vertices of the frozen front with a line-only occurrence where a live vertex has none (an end stays `None`; its exact place
/// is asked when somebody needs it), and the twin edges of one physical key told apart by hydrating those families. `None` when two live vertices own one edge.
pub fn with_line_ports(ctx: &mut ExactCtx<'_>, builder: &mut Builder, vertices: &[VertexSnapshot], time: &EventTime) -> SkelResult<Option<Vec<VertexSnapshot>>> {
    let mut line_only: Vec<VertexSnapshot> = Vec::with_capacity(vertices.len());
    for vertex in vertices {
        if !vertex.alive {
            line_only.push(vertex.clone());
            continue;
        }
        let prev = match &vertex.prev_occurrence {
            Some(found) => found.clone(),
            None => line_occurrence(builder, vertex.prev_edge)?,
        };
        let next = match &vertex.next_occurrence {
            Some(found) => found.clone(),
            None => line_occurrence(builder, vertex.next_edge)?,
        };
        line_only.push(VertexSnapshot { prev_occurrence: Some(prev), next_occurrence: Some(next), ..vertex.clone() });
    }
    let mut owners: Vec<(Val, BTreeSet<i64>)> = Vec::new();
    let mut slots: HashMap<Val, usize, FxBuild> = HashMap::default();
    for vertex in &line_only {
        let Some(occurrence) = vertex.next_occurrence.as_ref().filter(|_| vertex.alive) else {
            continue;
        };
        let slot = *slots.entry(occurrence.clone()).or_insert_with(|| {
            owners.push((occurrence.clone(), BTreeSet::new()));
            owners.len() - 1
        });
        owners[slot].1.insert(vertex.next_edge);
    }
    let ambiguous: HashSet<Vec<i64>> = owners
        .iter()
        .filter(|(occurrence, edge_ids)| edge_ids.len() > 1 && occurrence.get(1).is_some_and(Val::is_none) && occurrence.get(2).is_some_and(Val::is_none))
        .filter_map(|(occurrence, _)| occurrence.get(0).and_then(edge_key_of))
        .collect();
    if ambiguous.is_empty() {
        return Ok(Some(line_only));
    }
    let found = sparse_occurrences(ctx, builder, time, &ambiguous)?;
    if !found.duplicate_owner_ids.is_empty() {
        return Ok(None);
    }
    let hydrated = line_only
        .iter()
        .map(|vertex| -> SkelResult<VertexSnapshot> {
            let point_key = found.point_keys.get(vertex.ident as usize).ok_or_else(|| SkelError::Unsupported("IndexError in the oracle: a vertex beyond the point keys".to_string()))?;
            Ok(VertexSnapshot {
                point_key: point_key.clone().or_else(|| vertex.point_key.clone()),
                prev_occurrence: found.occurrences.get(&vertex.prev_edge).cloned().or_else(|| vertex.prev_occurrence.clone()),
                next_occurrence: found.occurrences.get(&vertex.next_edge).cloned().or_else(|| vertex.next_occurrence.clone()),
                ..vertex.clone()
            })
        })
        .collect::<SkelResult<Vec<_>>>()?;
    Ok(Some(hydrated))
}

// --------------------------------------------------------------------------
// the leaves of an overlay
// --------------------------------------------------------------------------

/// `_hydrate_f0(builder, snapshot, plans, time)`: the frozen vertices with the exact end points of every occurrence the packet names (and of the two edges of every existing
/// port a wiring or a rewrite refers to); `None` when two live vertices own one edge.
fn hydrate_f0(ctx: &mut ExactCtx<'_>, builder: &mut Builder, vertices: &[VertexSnapshot], plans: &[ComponentPlan], time: &EventTime) -> SkelResult<Option<Vec<VertexSnapshot>>> {
    let mut required: HashSet<Vec<i64>> = HashSet::new();
    for vertex in vertices {
        for occurrence in [&vertex.prev_occurrence, &vertex.next_occurrence].into_iter().flatten() {
            required.extend(occurrence.get(0).and_then(edge_key_of));
        }
    }
    let mut referenced: BTreeSet<i64> = BTreeSet::new();
    for plan in plans {
        for (_, predecessor, successor) in &plan.birth_wiring {
            referenced.extend([predecessor.existing, successor.existing].into_iter().flatten());
        }
        referenced.extend(plan.existing_port_rewrites.iter().map(|(ident, _, _)| *ident));
    }
    for ident in referenced {
        let vertex = builder.vertex_at(ident)?;
        required.insert(builder.edge_at(vertex.prev_edge)?.span.to_vec());
        required.insert(builder.edge_at(vertex.next_edge)?.span.to_vec());
    }
    let found = sparse_occurrences(ctx, builder, time, &required)?;
    if !found.duplicate_owner_ids.is_empty() {
        return Ok(None);
    }
    let hydrated = vertices
        .iter()
        .map(|vertex| -> SkelResult<VertexSnapshot> {
            let point_key = found.point_keys.get(vertex.ident as usize).ok_or_else(|| SkelError::Unsupported("IndexError in the oracle: a vertex beyond the point keys".to_string()))?;
            Ok(VertexSnapshot {
                point_key: point_key.clone(),
                prev_occurrence: found.occurrences.get(&vertex.prev_edge).cloned(),
                next_occurrence: found.occurrences.get(&vertex.next_edge).cloned(),
                ..vertex.clone()
            })
        })
        .collect::<SkelResult<Vec<_>>>()?;
    Ok(Some(hydrated))
}

/// `_occurrences(snapshot, plans)`: every occurrence the packet names, each once (the oracle's is a set; the order of the port is the order they are met in).
fn occurrences_of(vertices: &[VertexSnapshot], plans: &[ComponentPlan]) -> Vec<Val> {
    let mut seen: HashSet<Val> = HashSet::new();
    let mut found: Vec<Val> = Vec::new();
    let mut add = |occurrence: &Val| {
        if seen.insert(occurrence.clone()) {
            found.push(occurrence.clone());
        }
    };
    for vertex in vertices {
        for occurrence in [&vertex.prev_occurrence, &vertex.next_occurrence].into_iter().flatten() {
            add(occurrence);
        }
    }
    for plan in plans {
        for cut in &plan.split_cuts {
            cut.segment_occurrences.iter().for_each(&mut add);
        }
        for birth in &plan.births {
            add(&birth.prev_occurrence);
            add(&birth.next_occurrence);
        }
        for (_, prev_occurrence, next_occurrence) in &plan.existing_port_rewrites {
            add(prev_occurrence);
            add(next_occurrence);
        }
    }
    found
}

/// `_existing_edges_by_occurrence(snapshot, plans)`: the one runtime edge every occurrence the plans need already has, or none when any has not exactly one owner.
pub fn existing_edges_by_occurrence(vertices: &[VertexSnapshot], plans: &[ComponentPlan]) -> Option<HashMap<Val, i64, FxBuild>> {
    let new_occurrences: HashSet<&Val> = plans.iter().flat_map(|plan| plan.split_cuts.iter().flat_map(|cut| cut.segment_occurrences.iter())).collect();
    let mut required: Vec<&Val> = Vec::new();
    for plan in plans {
        for birth in &plan.births {
            required.extend([&birth.prev_occurrence, &birth.next_occurrence].into_iter().filter(|occurrence| !new_occurrences.contains(occurrence)));
        }
    }
    required.extend(plans.iter().flat_map(|plan| plan.split_cuts.iter().map(|cut| &cut.target_occurrence)));
    for plan in plans {
        for (_, prev_occurrence, next_occurrence) in &plan.existing_port_rewrites {
            required.extend([prev_occurrence, next_occurrence].into_iter().filter(|occurrence| !new_occurrences.contains(occurrence)));
        }
    }
    let mut grouped: HashMap<&Val, BTreeSet<i64>, FxBuild> = HashMap::default();
    for vertex in vertices {
        if let (true, Some(occurrence)) = (vertex.alive, vertex.next_occurrence.as_ref()) {
            grouped.entry(occurrence).or_default().insert(vertex.next_edge);
        }
    }
    let mut resolved: HashMap<Val, i64, FxBuild> = HashMap::default();
    for occurrence in required {
        let owners = grouped.get(occurrence)?;
        if owners.len() != 1 {
            return None;
        }
        resolved.insert(occurrence.clone(), *owners.iter().next()?);
    }
    Some(resolved)
}

/// The leaves of an overlay by occurrence (in the order they were made) and the runtime edge each occurrence is a span of.
pub struct LeafBindings {
    pub leaves: OrderedMap<Val, Leaf>,
    pub physical: HashMap<Val, i64, FxBuild>,
}

/// `_leaf_bindings(snapshot, materialization)`.
pub fn leaf_bindings(vertices: &[VertexSnapshot], materialization: &Materialization) -> SkelResult<Option<LeafBindings>> {
    let mut leaves: OrderedMap<Val, Leaf> = OrderedMap::new();
    for family in &materialization.families {
        for segment in &family.segments {
            let leaf = Leaf::from_val(segment)?;
            leaves.insert(leaf.occurrence(), leaf);
        }
    }
    for occurrence in occurrences_of(vertices, &materialization.plans) {
        if !leaves.contains_key(&occurrence) {
            leaves.insert(occurrence.clone(), Leaf::of_occurrence(&occurrence));
        }
    }
    let Some(mut physical) = existing_edges_by_occurrence(vertices, &materialization.plans) else {
        return Ok(None);
    };
    for vertex in vertices {
        for (occurrence, edge_id) in [(&vertex.prev_occurrence, vertex.prev_edge), (&vertex.next_occurrence, vertex.next_edge)] {
            let Some(occurrence) = occurrence else {
                continue;
            };
            if physical.get(occurrence).is_some_and(|previous| *previous != edge_id) {
                return Ok(None);
            }
            physical.insert(occurrence.clone(), edge_id);
        }
    }
    for plan in &materialization.plans {
        for cut in &plan.split_cuts {
            for occurrence in &cut.segment_occurrences {
                physical.insert(occurrence.clone(), cut.edge_id);
            }
        }
    }
    if leaves.keys().any(|item| !physical.contains_key(item)) {
        return Ok(None);
    }
    Ok(Some(LeafBindings { leaves, physical }))
}

// --------------------------------------------------------------------------
// the vertices of an overlay
// --------------------------------------------------------------------------

/// `_consumed_by_locus(vertex)`: a port both arms of which collapse to one point exactly at `now`: the interior of a contact locus, not a port of the poststate.
fn consumed_by_locus(vertex: &VertexSnapshot) -> bool {
    [&vertex.prev_occurrence, &vertex.next_occurrence].into_iter().all(|occurrence| match occurrence {
        Some(found) => found.get(1).is_some_and(|start| !start.is_none()) && found.get(1) == found.get(2),
        None => false,
    })
}

/// `_existing_ref(vertex)`: a surviving port is named by the pair of its source edges (the same across rebuilds: a contact names its emitter so); a port a locus consumed
/// carries its full identity (point and both occurrences).
pub fn existing_ref(vertex: &VertexSnapshot) -> JRef {
    if consumed_by_locus(vertex) {
        let part = |value: &Option<Val>| value.clone().unwrap_or_else(Val::none);
        return JRef::new("EXISTING", Val::tuple(vec![Val::str("CONSUMED_BY_LOCUS"), part(&vertex.point_key), part(&vertex.prev_occurrence), part(&vertex.next_occurrence)]));
    }
    JRef::new("EXISTING", port_identity(vertex))
}

fn leaf_of<'a>(leaves: &'a OrderedMap<Val, Leaf>, occurrence: &Val) -> SkelResult<&'a Leaf> {
    leaves.get(occurrence).ok_or_else(|| key_error("an occurrence without a leaf"))
}

/// What `_initial_vertices` answers: the existing ports as vertices, and the junction of every eligible frozen vertex by its runtime id.
pub struct InitialVertices {
    pub vertices: OrderedMap<JRef, SymVertex>,
    pub refs: HashMap<i64, JRef, FxBuild>,
}

/// `_initial_vertices(builder, snapshot, materialization, leaves)`: `None` when two surviving ports are indistinguishable.
pub fn initial_vertices(builder: &Builder, frozen: &[VertexSnapshot], plans: &[ComponentPlan], leaves: &OrderedMap<Val, Leaf>) -> SkelResult<Option<InitialVertices>> {
    let dead: HashSet<i64> = plans.iter().flat_map(|plan| plan.dead_vertex_ids.iter().copied()).collect();
    let mut rewrites: HashMap<i64, (&Val, &Val), FxBuild> = HashMap::default();
    for plan in plans {
        for (ident, prev_occurrence, next_occurrence) in &plan.existing_port_rewrites {
            rewrites.insert(*ident, (prev_occurrence, next_occurrence));
        }
    }
    let eligible: Vec<&VertexSnapshot> = frozen.iter().filter(|vertex| vertex.alive && vertex.prev_occurrence.is_some() && vertex.next_occurrence.is_some()).collect();
    let refs: HashMap<i64, JRef, FxBuild> = eligible.iter().map(|vertex| (vertex.ident, existing_ref(vertex))).collect();
    let survivors: Vec<i64> = eligible.iter().filter(|vertex| !consumed_by_locus(vertex)).map(|vertex| vertex.ident).collect();
    let distinct_survivors: HashSet<&JRef> = survivors.iter().map(|ident| &refs[ident]).collect();
    if distinct_survivors.len() != survivors.len() {
        return Ok(None);
    }
    let distinct: HashSet<&JRef> = refs.values().collect();
    if distinct.len() != refs.len() {
        return Ok(None);
    }
    let mut vertices: OrderedMap<JRef, SymVertex> = OrderedMap::new();
    for vertex in &eligible {
        if dead.contains(&vertex.ident) {
            continue;
        }
        let (prev_occurrence, next_occurrence) = match rewrites.get(&vertex.ident) {
            Some((prev, next)) => ((*prev).clone(), (*next).clone()),
            None => (vertex.prev_occurrence.clone().unwrap_or_else(Val::none), vertex.next_occurrence.clone().unwrap_or_else(Val::none)),
        };
        let runtime = builder.vertex_at(vertex.ident)?;
        let reference = refs[&vertex.ident].clone();
        vertices.insert(
            reference.clone(),
            SymVertex {
                reference: reference.clone(),
                prev: refs.get(&vertex.prev).cloned(),
                next: refs.get(&vertex.next).cloned(),
                prev_leaf: leaf_of(leaves, &prev_occurrence)?.clone(),
                next_leaf: leaf_of(leaves, &next_occurrence)?.clone(),
                birth: Rc::clone(&runtime.birth),
                point: Some(Rc::clone(&runtime.point)),
                sliding: runtime.sliding.as_ref().map(|sliding| Rc::new(sliding.clone())),
                provenance: frozen_keys([reference.key()]),
                runtime_id: Some(vertex.ident),
                trace: TraceInfo::of_builder(builder, Some(vertex.ident)),
                alive: true,
            },
        );
    }
    Ok(Some(InitialVertices { vertices, refs }))
}

// --------------------------------------------------------------------------
// the overlay
// --------------------------------------------------------------------------

/// `build_f0_overlay(builder, snapshot, time)`: the sparse overlay of the frozen state before the contact: only the occurrences asked for, with their line-only ports.
pub fn build_f0_overlay(ctx: &mut ExactCtx<'_>, builder: &mut Builder, vertices: &[VertexSnapshot], time: &TimeRef) -> SkelResult<Option<Overlay>> {
    let Some(ported) = with_line_ports(ctx, builder, vertices, time)? else {
        return Ok(None);
    };
    let empty = Materialization { plans: Vec::new(), families: Vec::new(), signature: Val::tuple(Vec::new()), unresolved_reason: None };
    let Some(LeafBindings { leaves, physical }) = leaf_bindings(&ported, &empty)? else {
        return Ok(None);
    };
    let Some(initial) = initial_vertices(builder, &ported, &empty.plans, &leaves)? else {
        return Ok(None);
    };
    let mut spans: OrderedMap<Leaf, Binding> = OrderedMap::new();
    for (occurrence, leaf) in leaves.iter() {
        let edge = *physical.get(occurrence).ok_or_else(|| key_error("an occurrence without a runtime edge"))?;
        spans.insert(leaf.clone(), Binding { leaf: leaf.clone(), physical_edge_id: edge, start: None, end: None });
    }
    let mut overlay = Overlay { vertices: initial.vertices, spans, changed: HashSet::default(), time: Rc::clone(time) };
    let (bindings, reason) = refreshed_span_bindings(&overlay);
    if reason.is_some() {
        return Ok(None);
    }
    overlay.spans = bindings.unwrap_or_default();
    Ok(Some(overlay))
}

/// `_birth_ref(birth, materialization, leaves)`: the junction a birth of the plans stands for.
fn birth_ref(birth: &crate::plans::BoundaryBirth, materialization: &Materialization, leaves: &OrderedMap<Val, Leaf>) -> SkelResult<JRef> {
    let split_birth = materialization.families.iter().flat_map(|family| family.births.iter()).find(|item| item.field("key") == Some(&birth.key));
    let prev_family = leaf_of(leaves, &birth.prev_occurrence)?.family();
    let next_family = leaf_of(leaves, &birth.next_occurrence)?.family();
    let key = match split_birth {
        Some(found) => Val::tuple(vec![Val::str("SPLIT"), found.field("contact").cloned().unwrap_or_else(Val::none), prev_family, next_family]),
        None => {
            let contact = materialization.plans.iter().flat_map(|plan| plan.edge_contacts.iter()).find(|contact| contact.births.iter().any(|candidate| candidate.key == birth.key));
            Val::tuple(vec![
                Val::str("CONTACT"),
                birth.point_key.clone(),
                prev_family,
                next_family,
                Val::tuple(contact.map_or_else(Vec::new, |found| found.participants.iter().map(|key| Val::ints(key)).collect())),
                Val::tuple(contact.map_or_else(Vec::new, |found| found.kinds.iter().map(|kind| Val::str(kind.value())).collect())),
            ])
        }
    };
    Ok(JRef::new("BIRTH", key))
}

/// `build_symbolic_overlay(builder, snapshot, materialization, include_line_ports=...)`: the overlay of the packet's materialization, or none when it cannot be built (the
/// oracle answers `None` and the caller names the refusal).
pub fn build_symbolic_overlay(ctx: &mut ExactCtx<'_>, builder: &mut Builder, vertices: &[VertexSnapshot], materialization: &Materialization, include_line_ports: bool) -> SkelResult<Option<Overlay>> {
    let first = materialization.plans.first().ok_or_else(|| SkelError::Unsupported("IndexError in the oracle: a materialization without a plan".to_string()))?;
    let time = Rc::clone(&first.time);
    let Some(mut hydrated) = hydrate_f0(ctx, builder, vertices, &materialization.plans, &time)? else {
        return Ok(None);
    };
    if include_line_ports {
        match with_line_ports(ctx, builder, &hydrated, &time)? {
            Some(ported) => hydrated = ported,
            None => return Ok(None),
        }
    }
    let Some(LeafBindings { leaves, physical }) = leaf_bindings(&hydrated, materialization)? else {
        return Ok(None);
    };
    let Some(initial) = initial_vertices(builder, &hydrated, &materialization.plans, &leaves)? else {
        return Ok(None);
    };
    let InitialVertices { vertices: mut overlay_vertices, refs: existing_refs } = initial;
    let mut birth_refs: HashMap<Val, JRef, FxBuild> = HashMap::default();
    let mut points: HashMap<Val, PointRef, FxBuild> = HashMap::default();
    for plan in &materialization.plans {
        for event in &plan.events {
            points.insert(exact_point_key(&event.point), Rc::clone(&event.point));
        }
    }
    for plan in &materialization.plans {
        for birth in &plan.births {
            let reference = birth_ref(birth, materialization, &leaves)?;
            if overlay_vertices.contains_key(&reference) || birth_refs.contains_key(&birth.key) {
                return Ok(None);
            }
            birth_refs.insert(birth.key.clone(), reference.clone());
            let point = points.get(&birth.point_key).ok_or_else(|| key_error("the point of a birth is not the point of an event"))?;
            overlay_vertices.insert(
                reference.clone(),
                SymVertex {
                    reference: reference.clone(),
                    prev: None,
                    next: None,
                    prev_leaf: leaf_of(&leaves, &birth.prev_occurrence)?.clone(),
                    next_leaf: leaf_of(&leaves, &birth.next_occurrence)?.clone(),
                    birth: Rc::clone(&plan.time),
                    point: Some(Rc::clone(point)),
                    sliding: None,
                    provenance: frozen_keys([reference.key()]),
                    runtime_id: None,
                    trace: TraceInfo::Absent,
                    alive: true,
                },
            );
        }
    }
    let resolve = |reference: &crate::plans::VertexReference| -> Option<JRef> {
        match (reference.existing, &reference.birth_key) {
            (Some(existing), _) => existing_refs.get(&existing).cloned(),
            (None, Some(key)) => birth_refs.get(key).cloned(),
            (None, None) => None,
        }
    };
    for plan in &materialization.plans {
        for (key, predecessor, successor) in &plan.birth_wiring {
            let reference = birth_refs.get(key).cloned().ok_or_else(|| key_error("a wired birth that is not a birth of the plans"))?;
            let (prev_ref, next_ref) = (resolve(predecessor), resolve(successor));
            let (Some(prev_ref), Some(next_ref)) = (prev_ref, next_ref) else {
                return Ok(None);
            };
            if !overlay_vertices.contains_key(&prev_ref) || !overlay_vertices.contains_key(&next_ref) {
                return Ok(None);
            }
            if let Some(vertex) = overlay_vertices.get_mut(&reference) {
                vertex.prev = Some(prev_ref.clone());
                vertex.next = Some(next_ref.clone());
            }
            if let Some(vertex) = overlay_vertices.get_mut(&prev_ref) {
                vertex.next = Some(reference.clone());
            }
            if let Some(vertex) = overlay_vertices.get_mut(&next_ref) {
                vertex.prev = Some(reference.clone());
            }
        }
    }
    if overlay_vertices.values().any(|item| (item.prev.is_none() || item.next.is_none()) && item.reference.kind() == "BIRTH") {
        return Ok(None);
    }
    let mut starts: HashMap<Leaf, JRef, FxBuild> = HashMap::default();
    let mut ends: HashMap<Leaf, JRef, FxBuild> = HashMap::default();
    for vertex in overlay_vertices.values() {
        starts.insert(vertex.next_leaf.clone(), vertex.reference.clone());
        ends.insert(vertex.prev_leaf.clone(), vertex.reference.clone());
    }
    let mut spans: OrderedMap<Leaf, Binding> = OrderedMap::new();
    for (occurrence, leaf) in leaves.iter() {
        let edge = *physical.get(occurrence).ok_or_else(|| key_error("an occurrence without a runtime edge"))?;
        spans.insert(leaf.clone(), Binding { leaf: leaf.clone(), physical_edge_id: edge, start: starts.get(leaf).cloned(), end: ends.get(leaf).cloned() });
    }
    let mut changed: HashSet<Leaf, FxBuild> = HashSet::default();
    for family in &materialization.families {
        for segment in &family.segments {
            changed.insert(Leaf::from_val(segment)?);
        }
    }
    for vertex in overlay_vertices.values() {
        if vertex.reference.kind() == "BIRTH" {
            changed.insert(vertex.prev_leaf.clone());
            changed.insert(vertex.next_leaf.clone());
        }
    }
    Ok(Some(Overlay { vertices: overlay_vertices, spans, changed, time }))
}

// --------------------------------------------------------------------------
// the view
// --------------------------------------------------------------------------

/// The part of the state of a span the view does not make anew on every question (`SpanStateMemoV1`): the slots of its two end junctions, and whether an end was born at
/// exactly `now` (then it stands at its birth).
struct SpanCache {
    start: Option<u32>,
    end: Option<u32>,
    born_start: bool,
    born_end: bool,
}

/// Where a junction named by a binding is in the overlay: a slot, or the number no slot has (a junction the overlay does not hold: the oracle's `KeyError` when it is asked
/// where that end stands).
const ABSENT: u32 = u32::MAX;

/// `exact_overlay_view(builder, overlay)`: the exact candidate view over an overlay and the builder that owns its physical edges.
pub struct OverlayView<'a> {
    builder: &'a Builder,
    overlay: &'a Overlay,
    slides: Vec<OnceCell<Option<SqrtSum>>>,
    spans: Vec<OnceCell<SpanCache>>,
}

fn same_motion(first: &SupportLine, second: &SupportLine) -> bool {
    let dot = i128::from(first.a) * i128::from(second.a) + i128::from(first.b) * i128::from(second.b);
    let scaled = |q: &cftuv_core::rat::Rat, norm: i128| q.mul(&cftuv_core::rat::Rat::from_int(cftuv_core::num::IBig::from(norm)));
    first.determinant(second) == 0 && dot > 0 && scaled(&first.q, second.normal_squared()) == scaled(&second.q, first.normal_squared())
}

impl<'a> OverlayView<'a> {
    pub fn new(builder: &'a Builder, overlay: &'a Overlay) -> OverlayView<'a> {
        OverlayView { builder, overlay, slides: (0..overlay.vertices.slot_count()).map(|_| OnceCell::new()).collect(), spans: (0..overlay.spans.slot_count()).map(|_| OnceCell::new()).collect() }
    }

    /// The reference of a junction in this view.
    pub fn vertex_ref(&self, reference: &JRef) -> SkelResult<VertexRef> {
        self.overlay.vertices.slot_of(reference).and_then(|slot| u32::try_from(slot).ok()).ok_or_else(|| key_error("a junction that is not in the overlay"))
    }

    /// The reference of a leaf in this view.
    pub fn span_ref(&self, leaf: &Leaf) -> SkelResult<SpanRef> {
        self.overlay.spans.slot_of(leaf).and_then(|slot| u32::try_from(slot).ok()).ok_or_else(|| key_error("a leaf that is not in the overlay"))
    }

    fn vertex_at(&self, vertex: VertexRef) -> SkelResult<&SymVertex> {
        self.overlay.vertices.at(vertex as usize).map(|(_, found)| found).ok_or_else(|| key_error("a junction that is not in the overlay"))
    }

    fn leaf_line(&self, leaf: &Leaf) -> SkelResult<&SupportLine> {
        let binding = self.overlay.spans.get(leaf).ok_or_else(|| key_error("a leaf that is not in the overlay"))?;
        Ok(&self.builder.edge_at(binding.physical_edge_id)?.line)
    }

    /// The sliding projection of a vertex that is not an existing port, from its two lines and its place (made once; its identity is made anew on every question).
    fn projection(&self, vertex: &SymVertex, slot: usize) -> SkelResult<&Option<SqrtSum>> {
        if let Some(found) = self.slides[slot].get() {
            return Ok(found);
        }
        let (first, second) = (self.leaf_line(&vertex.prev_leaf)?, self.leaf_line(&vertex.next_leaf)?);
        let value = match &vertex.point {
            Some(point) => sliding_projection(first, second, point),
            None if same_motion(first, second) => return Err(SkelError::Unsupported("AttributeError in the oracle: a sliding projection of a junction without a place".to_string())),
            None => None,
        };
        Ok(self.slides[slot].get_or_init(|| value))
    }

    fn span_cache(&self, slot: usize, binding: &Binding) -> &SpanCache {
        self.spans[slot].get_or_init(|| {
            let locate = |reference: &Option<JRef>| match reference {
                None => None,
                Some(found) => Some(self.overlay.vertices.slot_of(found).and_then(|place| u32::try_from(place).ok()).unwrap_or(ABSENT)),
            };
            let born = |place: Option<u32>| match place.and_then(|found| self.overlay.vertices.at(found as usize)) {
                Some((_, vertex)) => vertex.point.is_some() && times_are_equal(&vertex.birth, &self.overlay.time),
                None => false,
            };
            let (start, end) = (locate(&binding.start), locate(&binding.end));
            SpanCache { start, end, born_start: born(start), born_end: born(end) }
        })
    }

    fn born_point(&self, place: Option<u32>) -> Option<&EventPoint> {
        self.overlay.vertices.at(place? as usize)?.1.point.as_deref()
    }
}

/// Runs `run` with a view of the overlay and the builder's memory of places taken out of the builder (the view reads the builder; the memory is what it writes), and puts the
/// memory back. The candidate laws are called inside.
pub fn with_view<R>(builder: &mut Builder, overlay: &Overlay, run: impl FnOnce(&Builder, &OverlayView<'_>, &mut PositionMemo) -> R) -> R {
    let mut memo = std::mem::take(&mut builder.memo);
    let answer = {
        let owner: &Builder = builder;
        let view = OverlayView::new(owner, overlay);
        run(owner, &view, &mut memo)
    };
    builder.memo = memo;
    answer
}

impl CandidateView for OverlayView<'_> {
    fn prime_universe(&self) -> &[UBig] {
        &self.builder.prime_universe
    }

    fn vertex_state(&self, vertex: VertexRef) -> SkelResult<VertexState<'_>> {
        let found = self.vertex_at(vertex)?;
        let sliding = if found.reference.is_existing() {
            found.sliding.as_ref().map(|sliding| Sliding { value: &sliding.value, ident: sliding.ident })
        } else {
            self.projection(found, vertex as usize)?.as_ref().map(|value| Sliding { value, ident: self.builder.fresh_ident() })
        };
        // a leaf the overlay does not hold is the oracle's `KeyError` when the SPAN is asked for, not when the vertex is (a pair of neighbours asks the lines of three of its four
        // leaves): the reference of such a leaf is the number no slot has
        let leaf_slot = |leaf: &Leaf| self.overlay.spans.slot_of(leaf).and_then(|slot| u32::try_from(slot).ok()).unwrap_or(ABSENT);
        Ok(VertexState { prev_span: leaf_slot(&found.prev_leaf), next_span: leaf_slot(&found.next_leaf), birth: &found.birth, sliding })
    }

    fn span_state(&self, span: SpanRef) -> SkelResult<SpanState<'_>> {
        let (_, binding) = self.overlay.spans.at(span as usize).ok_or_else(|| key_error("a leaf that is not in the overlay"))?;
        let edge = self.builder.edge_at(binding.physical_edge_id)?;
        let cache = self.span_cache(span as usize, binding);
        Ok(SpanState {
            line: &edge.line,
            source_span: &edge.span,
            start_vertex: cache.start,
            end_vertex: cache.end,
            frozen_instant: Some(&self.overlay.time),
            frozen_start: if cache.born_start { self.born_point(cache.start) } else { None },
            frozen_end: if cache.born_end { self.born_point(cache.end) } else { None },
            occurrence: None,
        })
    }

    fn trace_bounds(&self, ctx: &mut ExactCtx<'_>, vertex: VertexRef, time: &EventTime) -> SkelResult<Option<bool>> {
        let found = self.vertex_at(vertex)?;
        match found.runtime_id.and_then(|ident| self.builder.traces.get(&ident)) {
            None => Ok(None),
            Some(trace) => Ok(Some(trace.bounds_time(ctx, time)?)),
        }
    }
}

/// `is_symbolic_split_emitter(builder, overlay, vertex)`: the runtime's own law of a reflex or sliding vertex, without requiring a runtime id.
pub fn is_symbolic_split_emitter(builder: &Builder, overlay: &Overlay, vertex: &SymVertex) -> SkelResult<bool> {
    if vertex.reference.is_virtual_boundary() {
        return Ok(false);
    }
    if let Some(ident) = vertex.runtime_id {
        let runtime = builder.vertex_at(ident)?;
        return Ok(runtime.reflex || runtime.sliding.is_some());
    }
    let line_of = |leaf: &Leaf| -> SkelResult<&SupportLine> {
        let binding = overlay.spans.get(leaf).ok_or_else(|| key_error("a leaf that is not in the overlay"))?;
        Ok(&builder.edge_at(binding.physical_edge_id)?.line)
    };
    let (first, second) = (line_of(&vertex.prev_leaf)?, line_of(&vertex.next_leaf)?);
    let slides = match (&vertex.sliding, &vertex.point) {
        (Some(_), _) => true,
        (None, Some(point)) => sliding_projection(first, second, point).is_some(),
        (None, None) if same_motion(first, second) => return Err(SkelError::Unsupported("AttributeError in the oracle: a sliding projection of a junction without a place".to_string())),
        (None, None) => false,
    };
    Ok(is_reflex(first, second) || slides)
}

// --------------------------------------------------------------------------
// the values the seams print
// --------------------------------------------------------------------------

fn sum_val(sum: &SqrtSum) -> Val {
    Val::data("SqrtSumV1", vec![("terms", Val::terms_of(sum))])
}

fn opt_ref(reference: &Option<JRef>) -> Val {
    reference.as_ref().map_or_else(Val::none, |found| found.val().clone())
}

/// `repr` of an overlay as the seams compare it: the vertices in the insertion order of the oracle, the spans and the changed leaves in the order of their `repr` (the two
/// that the oracle keeps in the order of a hash table), the trace of a vertex as a marker (the oracle prints the whole `TraceV1`).
pub fn overlay_val(overlay: &Overlay) -> Val {
    let vertices: Vec<(Val, Val)> = overlay
        .vertices
        .iter()
        .map(|(reference, vertex)| {
            let has_trace = !matches!(vertex.trace, TraceInfo::Absent);
            (
                reference.val().clone(),
                Val::data(
                    "SymbolicVertexV1",
                    vec![
                        ("ref", reference.val().clone()),
                        ("prev", opt_ref(&vertex.prev)),
                        ("next", opt_ref(&vertex.next)),
                        ("prev_leaf", vertex.prev_leaf.val().clone()),
                        ("next_leaf", vertex.next_leaf.val().clone()),
                        ("birth", time_val(&vertex.birth)),
                        ("point", vertex.point.as_ref().map_or_else(Val::none, |point| point_val(point))),
                        ("sliding", vertex.sliding.as_ref().map_or_else(Val::none, |sliding| sum_val(&sliding.value))),
                        ("provenance", Val::set(vertex.provenance.to_vec(), true)),
                        ("runtime_id", vertex.runtime_id.map_or_else(Val::none, Val::int)),
                        ("trace", if has_trace { Val::str("<trace>") } else { Val::none() }),
                        ("alive", Val::boolean(vertex.alive)),
                    ],
                ),
            )
        })
        .collect();
    let mut spans: Vec<(Val, Val)> = overlay
        .spans
        .iter()
        .map(|(leaf, binding)| {
            (
                leaf.val().clone(),
                Val::data(
                    "SymbolicSpanBindingV1",
                    vec![("leaf", binding.leaf.val().clone()), ("physical_edge_id", Val::int(binding.physical_edge_id)), ("start", opt_ref(&binding.start)), ("end", opt_ref(&binding.end))],
                ),
            )
        })
        .collect();
    spans.sort_by(|left, right| left.0.repr().as_bytes().cmp(right.0.repr().as_bytes()));
    Val::data(
        "SymbolicOverlayV1",
        vec![
            ("vertices", Val::dict(vertices)),
            ("spans", Val::dict(spans)),
            ("changed", Val::set(overlay.changed.iter().map(|leaf| leaf.val().clone()).collect(), false)),
            ("time", time_val(&overlay.time)),
        ],
    )
}
