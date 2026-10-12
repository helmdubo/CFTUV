//! `station_values` (`mesh.py`) with `station_of`, `transverse_root`, `transverse_of` (`stations.py`): the exact `(s, r)` of every vertex of every
//! region, in units of the lattice.
//!
//! A vertex of a region gets ONE answer. A second answer is legal in one case only: the rung of a JOIN corner (`_rung_station`: the two runs of one
//! flow give the bisector point two `s`, `s_v + d sin(d/2)` and `s_v - d sin(d/2)`, and the same `r`; their mean is the station of the chain
//! vertex, EXACTLY, and exactly that is checked), and the boundary between two frames of an opened ring (`_unify_across_frames`). Any other second
//! answer is a refusal that carries the answers of the runs.
//!
//! The chord mode of the oracle (`chords`, `anchors`: the second call of the clip) is not here: this entry is the first call, `chords=None`.
//!
//! The order of the questions is the cost: one `radical` per distinct `q` (the first time it is met), at the face, before its vertices.

use std::collections::HashMap;
use std::rc::Rc;

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::point::Point;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::data::{Line, MeshInput, Run, Table};
use crate::error::{MeshError, MeshResult};
use crate::tally::Tally;

/// Number of the law `RUNG_STATION_FROM_CHAIN_VERTEX_V1`: vertices of a JOIN rung that got the station of the chain vertex instead of two answers.
pub const RUNG_STATIONS_FROM_CHAIN_VERTEX: &str = "MATERIALIZE_RUNG_STATIONS_FROM_CHAIN_VERTEX";

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
                let mut answered: Vec<(String, SqrtSum, SqrtSum)> = stations.given[at].iter().map(|(id, held)| (id.to_string(), held.0.clone(), held.1.clone())).collect();
                answered.push((run_id.to_string(), value.0.clone(), value.1.clone()));
                return Err(MeshError::StationConflict { key: *key, region, face: index, answered });
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
