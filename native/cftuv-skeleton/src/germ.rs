//! The once-only ledger of the germs of a contact of one superlevel (`wavefront/superlevel_germ.py`, AUTH Q-10-ADD).
//!
//! A germ is a LOCUS of contact, not an event: its key is the canonical time, the canonical point and the canonical SET OF ENDS, an end being the pair
//! `(edge occurrence key, primitive integer ray FROM the contact point along that occurrence)`. The kind of the event is not part of the key (a duplicate of another
//! kind would otherwise pass as a new germ). The twins of a cut share an occurrence key and are told apart ONLY by their rays, which come from the carrier lines of the
//! frozen prestate (`VertexSnapshot::incoming_ray`/`outgoing_ray`, `Incident::target_ray`), not from the key (a hidden support of a fan has a degenerate key
//! `(x, y, x, y, ordinal)` that carries no direction).
//!
//! The scope of a ledger is the whole packet: 35 of the 63 corpus figures carry more than one component per level, and a ledger per unit would bring back the double
//! materialisation it exists to prevent.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;

use crate::plans::{fold_germ, BoundaryBirth};
use crate::pyval::{sorted_by_repr, Val};
use crate::error::SkelResult;
use crate::queue::EventKind;
use crate::time::{EventTime, TimeRef};
use crate::snapshot::{Incident, VertexSnapshot};

fn ray_val(ray: (i64, i64)) -> Val {
    Val::ints(&[ray.0, ray.1])
}

fn end_of(occurrence: &Val, ray: (i64, i64)) -> Val {
    Val::tuple(vec![occurrence.get(0).cloned().unwrap_or_else(Val::none), ray_val(ray)])
}

fn unique(ends: Vec<Val>) -> Vec<Val> {
    let mut seen: HashSet<Val> = HashSet::new();
    ends.into_iter().filter(|end| seen.insert(end.clone())).collect()
}

/// `port_ends(vertex)`: the ends of an existing port of the LAV, backwards along `prev` and forwards along `next`.
pub fn port_ends(vertex: &VertexSnapshot) -> Vec<Val> {
    let mut ends = Vec::new();
    if let Some(prev) = &vertex.prev_occurrence {
        ends.push(end_of(prev, vertex.incoming_ray));
    }
    if let Some(next) = &vertex.next_occurrence {
        ends.push(end_of(next, vertex.outgoing_ray));
    }
    unique(ends)
}

/// `target_ends(incident)`: the ends the cut occurrence brings to the contact point. A contact at the end of a span gives ONE ray, inward; a true cut gives TWO (the
/// locus lies between the two future twins); a degenerate span whose two ends stand at the contact brings both.
pub fn target_ends(incident: &Incident) -> Vec<Val> {
    let (Some(occurrence), Some(ray)) = (&incident.target_occurrence, incident.target_ray) else {
        return Vec::new();
    };
    let backward = (-ray.0, -ray.1);
    let mut ends = Vec::new();
    if occurrence.get(1) == Some(&incident.point_key) {
        ends.push(end_of(occurrence, ray));
    }
    if occurrence.get(2) == Some(&incident.point_key) {
        ends.push(end_of(occurrence, backward));
    }
    if ends.is_empty() {
        ends.push(end_of(occurrence, ray));
        ends.push(end_of(occurrence, backward));
    }
    unique(ends)
}

fn is_subset(inner: &[Val], outer: &HashSet<Val>) -> bool {
    inner.iter().all(|end| outer.contains(end))
}

/// `absorbed_by_locus(incident, vertices, occupied, edge_kind=EDGE)`: the incident describes the SAME locus the plan of a death already occupies, when every end it
/// brings lies in the set of ends of that locus. For a split the occurrence of the cut edge is checked first (necessary: its key is always in the prestate), then the
/// exact ends (rays come from the lines, so in the product every split has one).
pub fn absorbed_by_locus(incident: &Incident, vertices: &[VertexSnapshot], occupied: &HashSet<Val>) -> bool {
    let Some(emitter) = vertices.get(incident.event.vertex as usize) else {
        return false;
    };
    if !is_subset(&port_ends(emitter), occupied) {
        return false;
    }
    if incident.event.kind == EventKind::Edge {
        return vertices.get(incident.event.peer as usize).is_some_and(|peer| is_subset(&port_ends(peer), occupied));
    }
    let Some(occurrence) = &incident.target_occurrence else {
        return false;
    };
    let owner = occurrence.get(0).cloned().unwrap_or_else(Val::none);
    if !occupied.iter().any(|end| end.get(0) == Some(&owner)) {
        return false;
    }
    if incident.target_ray.is_none() {
        return true;
    }
    is_subset(&target_ends(incident), occupied)
}

/// `locus_ends(idents, vertices)`: the ends of the locus that the plan of the death of these ports already occupies.
pub fn locus_ends(idents: &BTreeSet<i64>, vertices: &[VertexSnapshot]) -> HashSet<Val> {
    idents.iter().filter_map(|ident| vertices.get(*ident as usize)).flat_map(port_ends).collect()
}

/// `junction_ends(prev_occurrence, next_occurrence, rays)`: the canonical set of ends of one contact junction. The ray of the prev end looks BACK along the incoming arm,
/// the ray of the next end FORWARD along the outgoing one; the sign is what tells the twins of one cut apart. `None` when a ray is not known.
pub fn junction_ends(prev_occurrence: &Val, next_occurrence: &Val, rays: &HashMap<Val, (i64, i64), FxBuild>) -> Option<Val> {
    let prev_key = prev_occurrence.get(0)?.clone();
    let next_key = next_occurrence.get(0)?.clone();
    let prev_ray = *rays.get(&prev_key)?;
    let next_ray = *rays.get(&next_key)?;
    let pair = [end_of(prev_occurrence, (-prev_ray.0, -prev_ray.1)), end_of(next_occurrence, next_ray)];
    Some(Val::tuple(sorted_by_repr(&pair, Val::clone)))
}

/// `SuperlevelGermLedgerV1`: a germ is materialised once; a repeated key answers the same object.
#[derive(Debug, Default)]
pub struct GermLedger {
    by_key: HashMap<Val, BoundaryBirth, FxBuild>,
    rays: HashMap<Val, (i64, i64), FxBuild>,
    /// `_time_key` of the times the packet has met, by the identity of the shared time object (the plans of one component all cut through the time of its sample).
    time_keys: HashMap<*const EventTime, Val, FxBuild>,
}

impl GermLedger {
    pub fn new(rays: HashMap<Val, (i64, i64), FxBuild>) -> GermLedger {
        GermLedger { by_key: HashMap::default(), rays, time_keys: HashMap::default() }
    }

    /// `_time_key(time)`, computed once per time object (the object is held by the plan, so the address is not reused while the ledger lives).
    pub fn time_key(&mut self, time: &TimeRef) -> SkelResult<Val> {
        let address = Rc::as_ptr(time);
        if let Some(found) = self.time_keys.get(&address) {
            return Ok(found.clone());
        }
        let key = crate::snapshot::time_key(time)?;
        self.time_keys.insert(address, key.clone());
        Ok(key)
    }

    pub fn rays(&self) -> &HashMap<Val, (i64, i64), FxBuild> {
        &self.rays
    }

    pub fn len(&self) -> usize {
        self.by_key.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_key.is_empty()
    }

    /// `key(time_key, point_key, prev_occurrence, next_occurrence)`: `GermKeyV1(time_key, point_key, ends)`, or none when a ray is unknown.
    pub fn key(&self, time_key: &Val, point_key: &Val, prev_occurrence: &Val, next_occurrence: &Val) -> Option<Val> {
        let ends = junction_ends(prev_occurrence, next_occurrence, &self.rays)?;
        Some(Val::data("GermKeyV1", vec![("time_key", time_key.clone()), ("point_key", point_key.clone()), ("ends", ends)]))
    }

    /// `materialize(key, germ, fold)`: the germ already materialised under the key, folded with the new one, or the new one.
    pub fn materialize(&mut self, key: Val, germ: BoundaryBirth) -> BoundaryBirth {
        match self.by_key.get(&key) {
            None => {
                self.by_key.insert(key, germ.clone());
                germ
            }
            Some(previous) => {
                let merged = fold_germ(previous, &germ);
                self.by_key.insert(key, merged.clone());
                merged
            }
        }
    }
}
