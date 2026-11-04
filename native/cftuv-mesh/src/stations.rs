//! `station_values` (`mesh.py`) with `station_of`, `transverse_root`, `transverse_of` (`stations.py`): the exact `(s, r)` of every vertex of every
//! region, in units of the lattice.
//!
//! A vertex of a region gets ONE answer. A second answer is legal in one case only: the rung of a JOIN corner (`_rung_station`: the two runs of one
//! flow give the bisector point two `s`, `s_v + d sin(d/2)` and `s_v - d sin(d/2)`, and the same `r`; their mean is the station of the chain
//! vertex, EXACTLY, and exactly that is checked), the boundary between two frames of an opened ring (`_unify_across_frames`), and the skeleton edge
//! of two runs of a flow with a consumed piece of chain between them (`_consumed_piece_facts`, law `RUNG_STATION_CONSUMED_PIECE_V1`), which is settled
//! after every face because its proof is another vertex. Any other second answer is a refusal that carries the answers of the runs.
//!
//! [`station_values`] is the first call (`chords=None`). [`chord_station_values`] is the call of the clip stage (`clip.cut_domain`: the new vertices of
//! the clip with `chords` and `anchors`, `rungs` and `plain` not read back): a second answer on a SHARED edge of two faces of a flow is the
//! interpolation of the facts of the edge's ends (`_chord_station`, law `RUNG_CHORD_STATION_V1`).
//!
//! The order of the questions is the cost: one `radical` per distinct `q` (the first time it is met), at the face, before its vertices.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::point::Point;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::data::{Line, MeshInput, Run, Table};
use crate::error::{MeshError, MeshResult};
use crate::tally::Tally;

/// Number of the law `RUNG_STATION_FROM_CHAIN_VERTEX_V1`: vertices of a JOIN rung that got the station of the chain vertex instead of two answers.
pub const RUNG_STATIONS_FROM_CHAIN_VERTEX: &str = "MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX";
/// Number of the law `RUNG_CHORD_STATION_V1`: new clip vertices on a SHARED edge of two faces of a flow that got the interpolation of the facts of its ends.
pub const RUNG_CHORD_STATIONS: &str = "MATERIALIZE_RUNG_CHORD_STATIONS";
/// Number of the law `RUNG_STATION_CONSUMED_PIECE_V1`: vertices answered by two runs of a flow with a CONSUMED piece of chain between them, which got the mean of the two answers.
pub const RUNG_STATIONS_CONSUMED_PIECE: &str = "MATERIALIZE_RUNG_STATIONS_CONSUMED_PIECE";

/// `(s, r)` of one vertex, shared between the answers and the facts.
pub type Value = Rc<(SqrtSum, SqrtSum)>;

/// `facts`: the `(s, r)` of every `(layout region, key)` in the order of first touch, with the answers of the runs that gave them.
///
/// The oracle keeps two dictionaries (`facts`, `answers`) and always creates a slot in both at once, so they have one order and one index.
pub struct Stations<'a> {
    index: HashMap<(u32, u32), usize, FxBuild>,
    pub slots: Vec<(u32, u32)>,
    pub facts: Vec<Value>,
    given: Vec<Vec<(&'a str, Value)>>,
    /// `rungs` in the order the oracle's set was filled (a set has no order a reader can see).
    pub rungs: Vec<(u32, u32)>,
    /// `plain`: the merged faces whose facts equal their own values at every vertex.
    pub plain: Vec<bool>,
}

impl Stations<'_> {
    /// `facts[(region, key)]`.
    pub fn fact(&self, region: u32, key: u32) -> Option<&Value> {
        self.index.get(&(region, key)).map(|at| &self.facts[*at])
    }

    pub fn is_rung(&self, region: u32, key: u32) -> bool {
        self.rungs.iter().any(|slot| *slot == (region, key))
    }
}

/// `station_of(run, point)`: `s_origin + <p - origin, direction>_G / |direction|_G`.
pub fn station_of(ctx: &mut ExactCtx<'_>, run: &Run, point: &Point) -> SqrtSum {
    let relative_x = point.0.sub(&run.origin.0);
    let relative_y = point.1.sub(&run.origin.1);
    let projection = relative_x.scaled(&run.covector.0).add(&relative_y.scaled(&run.covector.1));
    run.s_origin.add(&projection.mul(&run.inverse_length, ctx.products))
}

/// `transverse_root(line, budget)`: `sqrt(q) / q`, the arrival-time factor (`SqrtSumV1.radical(1, 1/q, budget)`).
pub fn transverse_root(ctx: &mut ExactCtx<'_>, line: &Line) -> MeshResult<SqrtSum> {
    let inverse = Rat::one().div(&line.q).map_err(|_| MeshError::Unsupported("a support line with q == 0 (ZeroDivisionError in the oracle)".to_string()))?;
    Ok(exact::radical(ctx, &Rat::one(), &inverse)?)
}

/// `transverse_of(line, point, root)`: `r = (a x + b y - c) / sqrt(q)`, as a product by `root = sqrt(q) / q`.
pub fn transverse_of(ctx: &mut ExactCtx<'_>, line: &Line, point: &Point, root: &SqrtSum) -> SqrtSum {
    let numerator = point.0.scaled(&line.a).add(&point.1.scaled(&line.b)).sub(&line.c);
    numerator.mul(root, ctx.products)
}

/// Equality of two `(s, r)` by VALUE (Python's `==`; the coefficient types are not part of it).
pub fn same_value(left: &Value, right: &Value) -> bool {
    Rc::ptr_eq(left, right) || (left.0.canonical_form() == right.0.canonical_form() && left.1.canonical_form() == right.1.canonical_form())
}

fn same_sum(left: &SqrtSum, right: &SqrtSum) -> bool {
    left.canonical_form() == right.canonical_form()
}

/// `_rung_station(table, answers, run_id, value)`: the `(s, r)` of a vertex on the bisector of a JOIN corner, or `None`.
fn rung_station(table: &Table, answers: &[(&str, Value)], run_id: &str, value: &Value) -> Option<Value> {
    if answers.len() != 1 || answers.iter().any(|(id, _)| *id == run_id) {
        return None;
    }
    let (other_run, other) = &answers[0];
    let station = table.join_station(other_run, run_id);
    let station = match station {
        Some(station) if other.1.difference_is_zero(&value.1) => station,
        _ => return None,
    };
    // `other_s + value_s - station - station` is zero
    if !other.0.add(&value.0).sub(station).sub(station).is_zero() {
        return None;
    }
    Some(Rc::new((station.clone(), value.1.clone())))
}

/// `_flow_positions(table)`: `run -> (head, number)` of the runs tied by the JOIN stations, in the order of their flow. The name of a head is only an identity.
fn flow_positions(table: &Table) -> HashMap<&str, (&str, usize), FxBuild> {
    let mut following: HashMap<&str, &str, FxBuild> = HashMap::default();
    let mut entered: HashSet<&str, FxBuild> = HashSet::default();
    for (before, after) in table.joins.keys() {
        following.insert(before, after);
        entered.insert(after);
    }
    let mut positions: HashMap<&str, (&str, usize), FxBuild> = HashMap::default();
    for head in following.keys().copied().filter(|run| !entered.contains(run)) {
        let (mut cursor, mut number) = (Some(head), 0usize);
        while let Some(run) = cursor {
            positions.insert(run, (head, number));
            cursor = following.get(run).copied();
            number += 1;
        }
    }
    positions
}

/// A second answer `station_values` could not reduce: the slot, the conflict as it stood at that moment, and whether the law may read the slot at all.
struct Pending {
    at: usize,
    key: u32,
    region: u32,
    face: usize,
    answered: Vec<(String, SqrtSum, SqrtSum)>,
    poisoned: bool,
}

/// `_consumed_piece_facts(table, pending, answers)`: `(slot number, (s, r))` of the slots the law proves, or the first unresolved conflict in the order of discovery.
fn consumed_piece_facts<'p>(table: &Table, stations: &Stations, pending: &'p [Pending], fan_slots: &HashSet<usize, FxBuild>) -> Result<Vec<(usize, Value)>, &'p Pending> {
    let positions = flow_positions(table);
    // per pending slot: the answering runs in the order of their flow with the sum of the two answers at the ends (`None`: the law cannot read it)
    let mut entries: Vec<Option<(Vec<usize>, SqrtSum)>> = Vec::with_capacity(pending.len());
    for item in pending {
        let given = &stations.given[item.at];
        let mut entry = None;
        if !item.poisoned && !fan_slots.contains(&item.at) && given.iter().all(|(run, _)| positions.contains_key(run)) {
            let head = positions[given[0].0].0;
            let first_r = &given[0].1 .1;
            if given.iter().all(|(run, value)| positions[run].0 == head && value.1.difference_is_zero(first_r)) {
                let mut placed: Vec<usize> = (0..given.len()).collect();
                placed.sort_by_key(|at| positions[given[*at].0].1);
                let (first, last) = (placed[0], placed[placed.len() - 1]);
                if positions[given[last].0].1 - positions[given[first].0].1 >= 2 {
                    let total = given[first].1 .0.add(&given[last].1 .0);
                    entry = Some((placed, total));
                }
            }
        }
        entries.push(entry);
    }
    let mut proven: HashMap<(&str, &str), Vec<SqrtSum>, FxBuild> = HashMap::default();
    let mut changed = true;
    while changed {
        changed = false;
        for (item, entry) in pending.iter().zip(&entries) {
            let Some((placed, total)) = entry else { continue };
            if placed.len() < 3 {
                continue;
            }
            let given = &stations.given[item.at];
            let pair = (given[placed[0]].0, given[placed[placed.len() - 1]].0);
            if proven.get(&pair).is_some_and(|known| known.iter().any(|sum| total.difference_is_zero(sum))) {
                continue;
            }
            let chained = placed.windows(2).all(|step| {
                let (before, after) = (given[step[0]].0, given[step[1]].0);
                positions[after].1 - positions[before].1 == 1 || proven.contains_key(&(before, after))
            });
            if chained {
                proven.entry(pair).or_default().push(total.clone());
                changed = true;
            }
        }
    }
    let half = Rat::one().div(&Rat::from_i64(2)).expect("two is not zero");
    let mut found = Vec::with_capacity(pending.len());
    for (item, entry) in pending.iter().zip(&entries) {
        let Some((placed, total)) = entry else { return Err(item) };
        let given = &stations.given[item.at];
        let pair = (given[placed[0]].0, given[placed[placed.len() - 1]].0);
        if !proven.get(&pair).is_some_and(|known| known.iter().any(|sum| total.difference_is_zero(sum))) {
            return Err(item);
        }
        found.push((item.at, Rc::new((total.scaled(&half), given[placed[0]].1 .1.clone()))));
    }
    Ok(found)
}

/// `_unify_across_frames(facts, answers, table, tally, rungs)`: the answers of the two runs of one corner in two regions of an opened ring are reduced to
/// the station of the chain vertex, exactly, by the condition of the rung inside a region.
fn unify_across_frames(stations: &mut Stations, table: &Table, tally: &mut Tally) {
    // key -> [(region, run, value)], in the order the keys and the answers first appear
    let mut order: Vec<u32> = Vec::new();
    let mut by_key: HashMap<u32, Vec<(u32, &str, Value)>, FxBuild> = HashMap::default();
    for (slot, given) in stations.slots.iter().zip(&stations.given) {
        for (run_id, value) in given {
            by_key
                .entry(slot.1)
                .or_insert_with(|| {
                    order.push(slot.1);
                    Vec::new()
                })
                .push((slot.0, run_id, value.clone()));
        }
    }
    for key in order {
        let entries = &by_key[&key];
        for first in 0..entries.len() {
            for second in first + 1..entries.len() {
                let (region_a, run_a, value_a) = &entries[first];
                let (region_b, run_b, value_b) = &entries[second];
                let station = table.cut_join_station(run_a, run_b);
                let station = match station {
                    Some(station) if region_a != region_b => station,
                    _ => continue,
                };
                if !same_sum(&value_a.1, &value_b.1) {
                    continue;
                }
                if !value_a.0.add(&value_b.0).sub(station).sub(station).is_zero() {
                    continue;
                }
                for (region, value) in [(*region_a, value_a), (*region_b, value_b)] {
                    let slot = (region, key);
                    let wanted: Value = Rc::new((station.clone(), value.1.clone()));
                    let at = stations.index[&slot];
                    if !same_value(&stations.facts[at], &wanted) {
                        stations.facts[at] = wanted;
                        stations.rungs.push(slot);
                        tally.add(RUNG_STATIONS_FROM_CHAIN_VERTEX, 1);
                    }
                }
            }
        }
    }
}

/// `station_values(frame_faces, cycles, layout, table, lattice_alpha, budget, tally, rungs, plain=plain)` (first call: no chords).
pub fn station_values<'a>(ctx: &mut ExactCtx<'_>, input: &MeshInput<'a>, cycles: &[Vec<(u32, u32)>], tally: &mut Tally) -> MeshResult<Stations<'a>> {
    let table = input.table;
    let mut stations = Stations { index: HashMap::default(), slots: Vec::new(), facts: Vec::new(), given: Vec::new(), rungs: Vec::new(), plain: Vec::new() };
    let mut roots: HashMap<Rat, SqrtSum, FxBuild> = HashMap::default();
    let mut own: Vec<Vec<Value>> = Vec::with_capacity(input.faces.len());
    let mut pending: Vec<Pending> = Vec::new();
    let mut fan_slots: HashSet<usize, FxBuild> = HashSet::default();
    for (index, (face, cycle)) in input.faces.iter().zip(cycles).enumerate() {
        let region = face.region;
        if !roots.contains_key(&face.line.q) {
            let root = transverse_root(ctx, &face.line)?;
            roots.insert(face.line.q.clone(), root);
        }
        let root = &roots[&face.line.q];
        let run_id: &'a str = &face.run.id;
        let mut values: Vec<Value> = Vec::with_capacity(cycle.len());
        for (key, vertex) in cycle {
            let point = &face.points[*vertex as usize];
            let station = match &face.fan_station {
                Some(station) => station.clone(),
                None => station_of(ctx, &face.run, point),
            };
            let value: Value = Rc::new((station, transverse_of(ctx, &face.line, point, root)));
            values.push(value.clone());
            let slot = (region, *key);
            // `known = facts.setdefault(slot, value)`, `given = answers.setdefault(slot, {})`
            let at = match stations.index.get(&slot) {
                Some(at) => *at,
                None => {
                    let at = stations.slots.len();
                    stations.index.insert(slot, at);
                    stations.slots.push(slot);
                    stations.facts.push(value.clone());
                    stations.given.push(Vec::new());
                    at
                }
            };
            if face.is_fan() {
                fan_slots.insert(at);
            }
            let same_run_answer = stations.given[at].iter().find(|(id, _)| *id == run_id).map(|(_, held)| held);
            if same_value(&stations.facts[at], &value) || same_run_answer.is_some_and(|held| same_value(held, &value)) {
                // `given.setdefault(run_id, value)`
                if !stations.given[at].iter().any(|(id, _)| *id == run_id) {
                    stations.given[at].push((run_id, value));
                }
                continue;
            }
            let rung = if face.is_fan() { None } else { rung_station(table, &stations.given[at], run_id, &value) };
            let Some(rung) = rung else {
                // settled after every face (`_consumed_piece_facts`), or the refusal with the text of this moment
                if !input.consumed_piece {
                    let mut answered: Vec<(String, SqrtSum, SqrtSum)> = stations.given[at].iter().map(|(id, held)| (id.to_string(), held.0.clone(), held.1.clone())).collect();
                    answered.push((run_id.to_string(), value.0.clone(), value.1.clone()));
                    return Err(MeshError::StationConflict { key: *key, region, face: index, answered });
                }
                let poisoned = face.is_fan() || stations.given[at].iter().any(|(id, _)| *id == run_id);
                match pending.iter_mut().find(|item| item.at == at) {
                    Some(item) => item.poisoned |= poisoned,
                    None => {
                        let mut answered: Vec<(String, SqrtSum, SqrtSum)> = stations.given[at].iter().map(|(id, held)| (id.to_string(), held.0.clone(), held.1.clone())).collect();
                        answered.push((run_id.to_string(), value.0.clone(), value.1.clone()));
                        pending.push(Pending { at, key: *key, region, face: index, answered, poisoned });
                    }
                }
                match stations.given[at].iter_mut().find(|(id, _)| *id == run_id) {
                    Some(held) => held.1 = value,
                    None => stations.given[at].push((run_id, value)),
                }
                continue;
            };
            // `given[run_id] = value`
            match stations.given[at].iter_mut().find(|(id, _)| *id == run_id) {
                Some(held) => held.1 = value,
                None => stations.given[at].push((run_id, value)),
            }
            stations.facts[at] = rung;
            stations.rungs.push(slot);
            tally.add(RUNG_STATIONS_FROM_CHAIN_VERTEX, 1);
        }
        own.push(values);
    }
    if !pending.is_empty() {
        match consumed_piece_facts(table, &stations, &pending, &fan_slots) {
            Err(item) => return Err(MeshError::StationConflict { key: item.key, region: item.region, face: item.face, answered: item.answered.clone() }),
            Ok(found) => {
                for (at, value) in found {
                    stations.facts[at] = value;
                    stations.rungs.push(stations.slots[at]);
                    tally.add(RUNG_STATIONS_CONSUMED_PIECE, 1);
                }
            }
        }
    }
    if !table.frame_of_run.is_empty() {
        unify_across_frames(&mut stations, table, tally);
    }
    for ((face, cycle), values) in input.faces.iter().zip(cycles).zip(&own) {
        let mut all = true;
        for ((key, _), value) in cycle.iter().zip(values) {
            let fact = stations.fact(face.region, *key).expect("every vertex of a cycle has a fact");
            if !same_value(fact, value) {
                all = false;
                break;
            }
        }
        stations.plain.push(all);
    }
    Ok(stations)
}

// --------------------------------------------------------------------------
// the call of the clip stage: chords and anchors
// --------------------------------------------------------------------------

/// One merged face as the call of the clip stage reads it: the region of the layout, the support line, the run and the fan station (the contour
/// is the clip's).
#[derive(Debug, Clone)]
pub struct Frame {
    pub region: u32,
    pub line: Line,
    pub run: Rc<Run>,
    /// `fan_station` (`Some` makes the face a fan).
    pub fan_station: Option<SqrtSum>,
}

/// `chords[i][key]`: the edge `(u, v, point u, point v)` of the contour of face `i` the new vertex `key` lies on (`clip._chords_of`).
#[derive(Debug, Clone)]
pub struct Chord {
    pub u: u32,
    pub v: u32,
    pub u_point: Point,
    pub v_point: Point,
}

impl Chord {
    /// `frozenset(chord[:2])`.
    fn edge(&self) -> (u32, u32) {
        (self.u.min(self.v), self.u.max(self.v))
    }
}

/// `anchors.get((region, key))`: the fact of a vertex of the contour before the clip (read only when a chord needs it), or `None`.
pub type Anchors<'f> = dyn FnMut(u32, u32) -> MeshResult<Option<Value>> + 'f;

/// `_chord_station(chord, point, region, anchors, budget)`: `(s, r)` of a new vertex on the edge `(u, v)` of the contour, the interpolation of the facts
/// of its ends along it, or `None`.
fn chord_station(ctx: &mut ExactCtx<'_>, chord: &Chord, point: &Point, region: u32, anchors: &mut Anchors<'_>) -> MeshResult<Option<Value>> {
    let u_fact = anchors(region, chord.u)?;
    let v_fact = anchors(region, chord.v)?;
    let (Some(u_fact), Some(v_fact)) = (u_fact, v_fact) else {
        return Ok(None);
    };
    let (u, v) = (&chord.u_point, &chord.v_point);
    let along_x = !v.0.sub(&u.0).is_zero();
    let (axis_u, axis_v, axis_p, other_u, other_v, other_p) = if along_x { (&u.0, &v.0, &point.0, &u.1, &v.1, &point.1) } else { (&u.1, &v.1, &point.1, &u.0, &v.0, &point.0) };
    let span = axis_v.sub(axis_u);
    if span.is_zero() {
        return Ok(None);
    }
    let share = exact::divided_by(ctx, &axis_p.sub(axis_u), &span)?;
    if !other_p.sub(other_u).sub(&other_v.sub(other_u).mul(&share, ctx.products)).is_zero() {
        return Ok(None);
    }
    if exact::sign(ctx, &share, SIGN_FILTER_BITS)? < 0 || exact::sign(ctx, &SqrtSum::rational(&Rat::one()).sub(&share), SIGN_FILTER_BITS)? < 0 {
        return Ok(None);
    }
    let s = u_fact.0.add(&v_fact.0.sub(&u_fact.0).mul(&share, ctx.products));
    let r = u_fact.1.add(&v_fact.1.sub(&u_fact.1).mul(&share, ctx.products));
    Ok(Some(Rc::new((s, r))))
}

/// `station_values(frame_faces, cycles, layout, table, lattice_alpha, budget, tally, chords=chords, anchors=anchors)`: the call of the clip stage. `cycles[i]`
/// is `(key, point)` of the new vertices of face `i` (`ClippedV1.extra_lists`), `chords[i]` the edges they lie on. The answer is `facts` in the oracle's
/// order (`Stations::slots`); `rungs` and `plain` are not part of it.
pub fn chord_station_values<'a>(
    ctx: &mut ExactCtx<'_>,
    frames: &'a [Frame],
    cycles: &[Vec<(u32, Point)>],
    chords: &[HashMap<u32, Chord, FxBuild>],
    anchors: &mut Anchors<'_>,
    table: &Table,
    tally: &mut Tally,
) -> MeshResult<Stations<'a>> {
    let mut stations = Stations { index: HashMap::default(), slots: Vec::new(), facts: Vec::new(), given: Vec::new(), rungs: Vec::new(), plain: Vec::new() };
    let mut roots: HashMap<Rat, SqrtSum, FxBuild> = HashMap::default();
    // `edges_of[slot]`: the edges (`None`: no chord) the faces that answered for the slot name
    let mut edges_of: Vec<HashSet<Option<(u32, u32)>, FxBuild>> = Vec::new();
    for (index, (frame, cycle)) in frames.iter().zip(cycles).enumerate() {
        let region = frame.region;
        let face_chords = &chords[index];
        if !roots.contains_key(&frame.line.q) {
            let root = transverse_root(ctx, &frame.line)?;
            roots.insert(frame.line.q.clone(), root);
        }
        let root = roots[&frame.line.q].clone();
        let run_id: &'a str = &frame.run.id;
        let is_fan = frame.fan_station.is_some();
        for (key, point) in cycle {
            let station = match &frame.fan_station {
                Some(station) => station.clone(),
                None => station_of(ctx, &frame.run, point),
            };
            let value: Value = Rc::new((station, transverse_of(ctx, &frame.line, point, &root)));
            let slot = (region, *key);
            let chord = face_chords.get(key);
            let at = match stations.index.get(&slot) {
                Some(at) => *at,
                None => {
                    let at = stations.slots.len();
                    stations.index.insert(slot, at);
                    stations.slots.push(slot);
                    stations.facts.push(value.clone());
                    stations.given.push(Vec::new());
                    edges_of.push(HashSet::default());
                    at
                }
            };
            edges_of[at].insert(chord.map(Chord::edge));
            let same_run_answer = stations.given[at].iter().find(|(id, _)| *id == run_id).map(|(_, held)| held);
            if same_value(&stations.facts[at], &value) || same_run_answer.is_some_and(|held| same_value(held, &value)) {
                if !stations.given[at].iter().any(|(id, _)| *id == run_id) {
                    stations.given[at].push((run_id, value));
                }
                continue;
            }
            let rung = if is_fan { None } else { rung_station(table, &stations.given[at], run_id, &value) };
            let on_one_edge = chord.is_some() && edges_of[at].len() == 1;
            if rung.is_none() && on_one_edge && !is_fan {
                let chord = chord.expect("on one edge");
                if let Some(interpolated) = chord_station(ctx, chord, point, region, anchors)? {
                    match stations.given[at].iter_mut().find(|(id, _)| *id == run_id) {
                        Some(held) => held.1 = value,
                        None => stations.given[at].push((run_id, value)),
                    }
                    if !same_value(&stations.facts[at], &interpolated) {
                        stations.facts[at] = interpolated;
                        tally.add(RUNG_CHORD_STATIONS, 1);
                    }
                    continue;
                }
            }
            let Some(rung) = rung else {
                let mut answered: Vec<(String, SqrtSum, SqrtSum)> = stations.given[at].iter().map(|(id, held)| (id.to_string(), held.0.clone(), held.1.clone())).collect();
                answered.push((run_id.to_string(), value.0.clone(), value.1.clone()));
                return Err(MeshError::StationConflict { key: *key, region, face: index, answered });
            };
            match stations.given[at].iter_mut().find(|(id, _)| *id == run_id) {
                Some(held) => held.1 = value,
                None => stations.given[at].push((run_id, value)),
            }
            stations.facts[at] = rung;
            stations.rungs.push(slot);
            tally.add(RUNG_STATIONS_FROM_CHAIN_VERTEX, 1);
        }
    }
    if !table.frame_of_run.is_empty() {
        unify_across_frames(&mut stations, table, tally);
    }
    Ok(stations)
}
