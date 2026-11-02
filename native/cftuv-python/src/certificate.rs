//! The side cuts of the drag certificate in ONE native call (CERTIFICATE_SIDE_CUTS_NATIVE_V1): `parametric._clip_slopes` whole, from the inputs of the
//! domain's clip stage to the slopes of the clip nodes and of the facts the clip added.
//!
//! Per attempt (`_STEP_ATTEMPTS`, the step halved after a refusal or a change of structure) and per side (`+step`, `-step`): the points and facts before
//! the cut moved along their slopes (`_side_inputs`), the seam edges (`clip.seam_edges`), the geometric clip (`cftuv_clip::geometry::clip_geometry`),
//! the chords of the new vertices (`clip._chords_of`) and their facts (`cftuv_mesh::stations::chord_station_values`, law `RUNG_CHORD_STATION_V1`), merged as
//! `clip.cut_domain` merges them; then the structure of both side cuts against the domain's own cut (`parametric._shape`) and the exact second
//! differences (`parametric._clip_slope_values`). Every intermediate stays native: the inputs are read once, the answer is two small dicts.
//!
//! Status: 0 the slopes; 21 `PARAM_CLIP_NODE`, 22 `PARAM_CLIP_FACT`, 23 `PARAM_CLIP_STRUCTURE_CHANGED`, 24 `PARAM_CLIP_REFUSED` (detail: the name of the
//! `MaterializationOutcome`); 12 the port declines (detail: why) and NOTHING of the call is applied, so the caller runs the Python chain on the same state.
//! On every other status the memory log of the call is replayed on the host's tables and the sign counts are returned for the shim to settle, as the
//! Python side cuts leave them. What the side cuts leave that no answer reads is not reproduced (NATIVE_PATH_PRODUCT_CONTRACT_V1): the offset normals the
//! side lifts would write into the plane's `_normal_by_position` (the batch's normals were read before the certificate; a later lift writes its own), the
//! articles of the throwaway unlimited budgets and the throwaway `tally`.
//!
//! The order of the answer dicts is not part of the contract: the certificate only looks slopes up by key and counts them.

use std::collections::{HashMap, HashSet};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::rc::Rc;
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList, PyString, PyTuple};

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::emit::{Clipped, Law};
use cftuv_clip::error::ClipError;
use cftuv_clip::geometry::{clip_geometry, ClipInput};
use cftuv_clip::point::Point;
use cftuv_core::codec::Reader;
use cftuv_core::exact::ExactCtx;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::session::{CostRun, Session};
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_mesh::error::MeshResult;
use cftuv_mesh::stations::{chord_station_values, Chord, Value};
use cftuv_mesh::tally::Tally;
use cftuv_silhouette::assemble_law::uniform_numerator_slope;

use crate::materialize_hot::{carried, pair_of, rat_of, unsupported, Fail, Read};
use crate::memlog::{apply_log, Tables};
use crate::pyobj::{sqrt_sum_from_py, sqrt_sum_to_py, Pool};
use crate::refusal::Forced;

pub const STATUS_OK: u8 = 0;
pub const STATUS_DECLINED: u8 = 12;
pub const STATUS_CLIP_NODE: u8 = 21;
pub const STATUS_CLIP_FACT: u8 = 22;
pub const STATUS_STRUCTURE_CHANGED: u8 = 23;
pub const STATUS_CLIP_REFUSED: u8 = 24;

/// `(status, detail, point slopes, fact slopes, sign-counter deltas, changed-tables bits, (read, compute, build, memory log) nanoseconds, attempts run)`.
pub type Answer<'py> = (u8, Option<String>, Option<Bound<'py, PyAny>>, Option<Bound<'py, PyAny>>, [u64; 5], u8, [u64; 4], u32);

/// What the shim hands over: the Python objects of one certificate's side cuts.
pub struct SideArgs<'r, 'py> {
    pub triangles: &'r Bound<'py, PyAny>,
    pub points_before: &'r Bound<'py, PyAny>,
    pub cycles: &'r Bound<'py, PyAny>,
    pub facts_before: &'r Bound<'py, PyAny>,
    pub key_slope: &'r Bound<'py, PyAny>,
    pub fact_slope: &'r Bound<'py, PyAny>,
    pub frame_faces: &'r Bound<'py, PyAny>,
    pub regions: &'r Bound<'py, PyAny>,
    pub fans: &'r Bound<'py, PyAny>,
    pub flows: &'r Bound<'py, PyAny>,
    pub polygons: &'r Bound<'py, PyAny>,
    pub law: u8,
    pub by_faces: bool,
    pub inert: &'r Bound<'py, PyAny>,
    pub table: &'r Bound<'py, PyAny>,
    pub lattice_alpha: &'r Bound<'py, PyAny>,
    pub scale: &'r Bound<'py, PyAny>,
    pub step: &'r Bound<'py, PyAny>,
    pub attempts: u32,
    pub cut: &'r Bound<'py, PyAny>,
    pub facts_after: &'r Bound<'py, PyAny>,
}

type Slot = (u32, String);

/// A keyed point of a cut as the Python dict holds it: the key object, its text, the value.
struct Keyed<'py> {
    object: Bound<'py, PyAny>,
    name: String,
    point: Point,
}

/// The domain's own cut (`inputs.cut`): its shape (`parametric._shape`) and its nodes.
struct BaseCut<'py> {
    polygons: Vec<Vec<Vec<String>>>,
    cycles: Vec<Vec<String>>,
    vertex_lists: Vec<Vec<String>>,
    extra_lists: Vec<Vec<String>>,
    points: Vec<Keyed<'py>>,
    snapped: Vec<Keyed<'py>>,
}

/// Everything the side cuts read, converted once.
struct Inputs<'py> {
    points: Vec<(String, Point)>,
    point_slopes: Vec<Option<(SqrtSum, SqrtSum)>>,
    cycle_keys: Vec<Vec<String>>,
    facts: Vec<(Slot, Point)>,
    fact_slopes: Vec<Option<(SqrtSum, SqrtSum)>>,
    polygons: Vec<Vec<Vec<String>>>,
    regions: Vec<u32>,
    fans: Vec<bool>,
    flows: Vec<bool>,
    inert: Vec<(String, String)>,
    lattice_alpha: Rat,
    scale: Rat,
    step: Rat,
    base: BaseCut<'py>,
    /// The facts of `facts_after` that are not in `facts_before` (the clip's own), with their slot objects.
    added: Vec<(Bound<'py, PyAny>, Slot, Point)>,
}

fn text(object: &Bound<'_, PyAny>, what: &str) -> Read<String> {
    match object.cast_exact::<PyString>() {
        Ok(found) => Ok(found.to_str()?.to_string()),
        Err(_) => unsupported(format!("{what} is not a str")),
    }
}

fn dict<'a, 'py>(object: &'a Bound<'py, PyAny>, what: &str) -> Read<&'a Bound<'py, PyDict>> {
    object.cast_exact::<PyDict>().map_err(|_| Fail::Unsupported(format!("{what} is not a dict")))
}

fn list<'a, 'py>(object: &'a Bound<'py, PyAny>, what: &str) -> Read<&'a Bound<'py, PyList>> {
    object.cast_exact::<PyList>().map_err(|_| Fail::Unsupported(format!("{what} is not a list")))
}

/// `(region, key)` of a fact slot (`(int, str)`).
fn slot_of(object: &Bound<'_, PyAny>) -> Read<Slot> {
    let Ok(pair) = object.cast_exact::<PyTuple>() else {
        return unsupported("a fact slot is not a tuple");
    };
    if pair.len() != 2 {
        return unsupported("a fact slot is not a pair");
    }
    let region: u32 = carried(pair.get_item(0)?.extract::<u32>(), "a fact region")?;
    Ok((region, text(&pair.get_item(1)?, "a fact key")?))
}

/// The keys of a list of `(key, point)` lists (`_shape`'s `keys`), `None` for a `None` (declined by the caller).
fn keyed_lists(object: &Bound<'_, PyAny>, what: &str) -> Read<Vec<Vec<String>>> {
    let mut found = Vec::new();
    for entries in object.try_iter().map_err(|_| Fail::Unsupported(format!("{what} is not iterable")))? {
        let mut keys = Vec::new();
        for entry in entries?.try_iter()? {
            let entry = entry?;
            let Ok(pair) = entry.cast_exact::<PyTuple>() else {
                return unsupported(format!("an entry of {what} is not a (key, point) tuple"));
            };
            if pair.len() != 2 {
                return unsupported(format!("an entry of {what} is not a (key, point) pair"));
            }
            keys.push(text(&pair.get_item(0)?, "a key")?);
        }
        found.push(keys);
    }
    Ok(found)
}

/// Faces of polygons of keys (a list of faces, each an iterable of tuples or lists of `str`).
fn polygons_of(object: &Bound<'_, PyAny>, what: &str) -> Read<Vec<Vec<Vec<String>>>> {
    let mut faces = Vec::new();
    for face in object.try_iter().map_err(|_| Fail::Unsupported(format!("{what} is not iterable")))? {
        let mut loops = Vec::new();
        for polygon in face?.try_iter()? {
            let polygon = polygon?;
            if polygon.cast_exact::<PyTuple>().is_err() && polygon.cast_exact::<PyList>().is_err() {
                return unsupported(format!("a polygon of {what} is not a tuple or a list"));
            }
            let mut keys = Vec::new();
            for key in polygon.try_iter()? {
                keys.push(text(&key?, "a polygon key")?);
            }
            loops.push(keys);
        }
        faces.push(loops);
    }
    Ok(faces)
}

fn keyed_dict<'py>(pool: &Pool, object: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Keyed<'py>>> {
    let found = dict(object, what)?;
    let mut items = Vec::with_capacity(found.len());
    for (key, value) in found.iter() {
        let name = text(&key, "a node key")?;
        items.push(Keyed { object: key, name, point: pair_of(pool, &value, "a node point")? });
    }
    Ok(items)
}

fn flags(object: &Bound<'_, PyAny>, what: &str) -> Read<Vec<bool>> {
    let mut found = Vec::new();
    for item in list(object, what)?.iter() {
        found.push(item.is_truthy()?);
    }
    Ok(found)
}

fn read<'py>(pool: &Pool, args: &SideArgs<'_, 'py>) -> Read<Inputs<'py>> {
    let points_before = dict(args.points_before, "points_before")?;
    let key_slope = dict(args.key_slope, "key_slope")?;
    let mut points = Vec::with_capacity(points_before.len());
    let mut point_slopes = Vec::with_capacity(points_before.len());
    for (key, point) in points_before.iter() {
        let slope = match key_slope.get_item(&key)? {
            Some(slope) if !slope.is_none() => Some(pair_of(pool, &slope, "a key slope")?),
            _ => None,
        };
        points.push((text(&key, "a point key")?, pair_of(pool, &point, "a point before the cut")?));
        point_slopes.push(slope);
    }
    let cycle_keys = keyed_lists(list(args.cycles, "the cycles")?.as_any(), "the cycles")?;
    let facts_before = dict(args.facts_before, "facts_before")?;
    let fact_slope = dict(args.fact_slope, "fact_slope")?;
    let mut facts = Vec::with_capacity(facts_before.len());
    let mut fact_slopes = Vec::with_capacity(facts_before.len());
    let mut before: HashSet<Slot, FxBuild> = HashSet::default();
    for (slot, value) in facts_before.iter() {
        // `_nonzero(fact_slope.get(slot))`
        let slope = match fact_slope.get_item(&slot)? {
            Some(slope) if !slope.is_none() => {
                let pair = pair_of(pool, &slope, "a fact slope")?;
                if pair.0.is_zero() && pair.1.is_zero() {
                    None
                } else {
                    Some(pair)
                }
            }
            _ => None,
        };
        let native = slot_of(&slot)?;
        before.insert(native.clone());
        facts.push((native, pair_of(pool, &value, "a fact")?));
        fact_slopes.push(slope);
    }
    let mut regions = Vec::new();
    for item in list(args.regions, "regions")?.iter() {
        regions.push(carried(item.extract::<u32>(), "a layout region")?);
    }
    let cut = args.cut;
    let shape = |name: &str| -> Read<Bound<'py, PyAny>> {
        let found = cut.getattr(name)?;
        if found.is_none() {
            return unsupported(format!("the domain's cut has no {name}"));
        }
        Ok(found)
    };
    let base = BaseCut {
        polygons: polygons_of(&shape("polygons")?, "the cut's polygons")?,
        cycles: keyed_lists(&shape("cycles")?, "the cut's cycles")?,
        vertex_lists: keyed_lists(&shape("vertex_lists")?, "the cut's vertex lists")?,
        extra_lists: keyed_lists(&shape("extra_lists")?, "the cut's extra lists")?,
        points: keyed_dict(pool, &shape("points")?, "the cut's points")?,
        snapped: keyed_dict(pool, &shape("snapped")?, "the cut's snapped points")?,
    };
    let facts_after = dict(args.facts_after, "facts_after")?;
    let mut added = Vec::new();
    for (slot, value) in facts_after.iter() {
        let native = slot_of(&slot)?;
        if before.contains(&native) {
            continue;
        }
        let point = pair_of(pool, &value, "a fact")?;
        added.push((slot, native, point));
    }
    Ok(Inputs {
        points,
        point_slopes,
        cycle_keys,
        facts,
        fact_slopes,
        polygons: polygons_of(list(args.polygons, "polygons")?.as_any(), "polygons")?,
        regions,
        fans: flags(args.fans, "fans")?,
        flows: flags(args.flows, "flows")?,
        inert: carried(crate::clip::inert_of(args.inert), "inert")?,
        lattice_alpha: rat_of(pool, args.lattice_alpha, "lattice_alpha")?,
        scale: rat_of(pool, args.scale, "the scale")?,
        step: rat_of(pool, args.step, "the step")?,
        base,
        added,
    })
}

/// Why one side cut has no answer: the oracle's `MaterializationRefusal` (the outcome's name), or an input or outcome the port does not carry.
enum SideFail {
    Refused(String),
    Declined(String),
}

/// A dict filled in order with the oracle's semantics (an overwrite keeps the first position).
struct Facts {
    slots: Vec<Slot>,
    values: Vec<Point>,
    index: HashMap<Slot, usize, FxBuild>,
}

impl Facts {
    fn get(&self, slot: &Slot) -> Option<&Point> {
        self.index.get(slot).map(|at| &self.values[*at])
    }
}

struct Side {
    clipped: Clipped,
    facts: Facts,
}

fn shifted(point: &Point, slope: &(SqrtSum, SqrtSum), shift: &Rat) -> Point {
    (point.0.add(&slope.0.scaled(shift)), point.1.add(&slope.1.scaled(shift)))
}

/// `clip.seam_edges(frame_faces, polygons, facts, layout, lattice_alpha)` on the native inputs: the key pairs of the boundary edges that are not a front.
fn seam_edges(polygons: &[Vec<Vec<String>>], regions: &[u32], facts: &Facts, front: &SqrtSum) -> Result<Vec<(String, String)>, SideFail> {
    let mut numbers: HashMap<&str, u32, FxBuild> = HashMap::default();
    let mut names: Vec<&str> = Vec::new();
    let mut slots: HashMap<(u32, u32), usize, FxBuild> = HashMap::default();
    let mut pairs: Vec<((u32, u32), u32, usize)> = Vec::new();
    for (index, face) in polygons.iter().enumerate() {
        for polygon in face {
            let ids: Vec<u32> = polygon
                .iter()
                .map(|key| {
                    *numbers.entry(key.as_str()).or_insert_with(|| {
                        names.push(key.as_str());
                        (names.len() - 1) as u32
                    })
                })
                .collect();
            for position in 0..ids.len() {
                let (a, b) = (ids[position], ids[(position + 1) % ids.len()]);
                let pair = (a.min(b), a.max(b));
                match slots.get(&pair) {
                    Some(at) => {
                        pairs[*at].1 += 1;
                        pairs[*at].2 = index;
                    }
                    None => {
                        slots.insert(pair, pairs.len());
                        pairs.push(((a, b), 1, index));
                    }
                }
            }
        }
    }
    let mut found = Vec::new();
    for ((a, b), count, owner) in &pairs {
        if *count != 1 || a == b {
            continue;
        }
        let region = *regions.get(*owner).ok_or_else(|| SideFail::Declined("more polygon lists than frame faces (IndexError in the oracle)".to_string()))?;
        let r_of = |key: &str| -> Result<&SqrtSum, SideFail> {
            facts.get(&(region, key.to_string())).map(|fact| &fact.1).ok_or_else(|| SideFail::Declined("a vertex of a boundary edge without a fact (KeyError in the oracle)".to_string()))
        };
        let (first, second) = (names[*a as usize], names[*b as usize]);
        let (ra, rb) = (r_of(first)?, r_of(second)?);
        let rim = !(ra.is_zero() && rb.is_zero()) && ra.sub(front).is_zero() && rb.sub(front).is_zero();
        if !rim {
            found.push((first.to_string(), second.to_string()));
        }
    }
    Ok(found)
}

/// Interned key numbers of one side cut (`chord_station_values` speaks numbers).
#[derive(Default)]
struct Keys {
    index: HashMap<Rc<str>, u32, FxBuild>,
    names: Vec<Rc<str>>,
}

impl Keys {
    fn number(&mut self, key: &Rc<str>) -> u32 {
        if let Some(found) = self.index.get(key) {
            return *found;
        }
        let number = self.names.len() as u32;
        self.index.insert(key.clone(), number);
        self.names.push(key.clone());
        number
    }
}

#[allow(clippy::too_many_arguments)]
fn side_cut(
    ctx: &mut ExactCtx<'_>,
    warm: &mut cftuv_clip::warm::Warm,
    plane: &cftuv_clip::plane::Plane,
    inputs: &Inputs<'_>,
    frames: &[cftuv_mesh::stations::Frame],
    table: &cftuv_mesh::data::Table,
    law: Law,
    by_faces: bool,
    shift: &Rat,
) -> Result<Side, SideFail> {
    // `_side_inputs`: the points and facts before the cut, moved along their slopes
    let points: Vec<(String, Point)> = inputs
        .points
        .iter()
        .zip(&inputs.point_slopes)
        .map(|((key, point), slope)| (key.clone(), match slope {
            Some(slope) => shifted(point, slope, shift),
            None => point.clone(),
        }))
        .collect();
    let mut facts = Facts { slots: Vec::with_capacity(inputs.facts.len()), values: Vec::with_capacity(inputs.facts.len()), index: HashMap::default() };
    for ((slot, fact), slope) in inputs.facts.iter().zip(&inputs.fact_slopes) {
        let value = match slope {
            Some(slope) => shifted(fact, slope, shift),
            None => fact.clone(),
        };
        match facts.index.get(slot) {
            Some(at) => facts.values[*at] = value,
            None => {
                facts.index.insert(slot.clone(), facts.slots.len());
                facts.slots.push(slot.clone());
                facts.values.push(value);
            }
        }
    }
    let lattice_alpha = inputs.lattice_alpha.add(shift);
    let seam = seam_edges(&inputs.polygons, &inputs.regions, &facts, &SqrtSum::rational(&lattice_alpha))?;
    let input = ClipInput {
        points: &points,
        cycles: &inputs.cycle_keys,
        polygons: &inputs.polygons,
        law,
        seam: &seam,
        fans: Some(&inputs.fans),
        flows: Some(&inputs.flows),
        by_faces,
        inert: &inputs.inert,
    };
    // the offset normals of the side lifts (`ran.writes`) are not written anywhere (module note)
    let ran = clip_geometry(ctx, warm, plane, &input);
    let clipped = match ran.result {
        Ok(clipped) => clipped,
        Err(ClipError::Refusal { outcome, .. }) => return Err(SideFail::Refused(outcome.to_string())),
        Err(_) => return Err(SideFail::Declined("the side clip ended in an outcome the certificate does not catch".to_string())),
    };
    // `_chords_of(cycles, clipped.cycles)`, `station_values(..., chords=..., anchors=facts)`
    if clipped.extra_lists.len() != frames.len() || clipped.cycles.len().min(inputs.cycle_keys.len()) != frames.len() {
        return Err(SideFail::Declined("frame_faces, the cycles of the clip and the chords do not have one length".to_string()));
    }
    let mut keys = Keys::default();
    let mut chords: Vec<HashMap<u32, Chord, FxBuild>> = Vec::with_capacity(frames.len());
    for (cycle, contour) in inputs.cycle_keys.iter().zip(&clipped.cycles) {
        let original: HashSet<&str, FxBuild> = cycle.iter().map(String::as_str).collect();
        let mut found: HashMap<u32, Chord, FxBuild> = HashMap::default();
        if contour.iter().any(|(key, _)| original.contains(&**key)) {
            let count = contour.len() as isize;
            let at = |position: isize| &contour[position.rem_euclid(count) as usize];
            for (position, (key, _point)) in contour.iter().enumerate() {
                if original.contains(&**key) {
                    continue;
                }
                let (mut before, mut after) = (position as isize, position as isize);
                while !original.contains(&*at(before).0) {
                    before -= 1;
                }
                while !original.contains(&*at(after).0) {
                    after += 1;
                }
                let (first, last) = (at(before), at(after));
                let number = keys.number(key);
                let chord = Chord { u: keys.number(&first.0), v: keys.number(&last.0), u_point: (*first.1).clone(), v_point: (*last.1).clone() };
                found.insert(number, chord);
            }
        }
        chords.push(found);
    }
    let lists: Vec<Vec<(u32, Point)>> = clipped.extra_lists.iter().map(|entries| entries.iter().map(|(key, point)| (keys.number(key), (**point).clone())).collect()).collect();
    let names = keys.names.clone();
    let mut lookup = |region: u32, key: u32| -> MeshResult<Option<Value>> {
        Ok(facts.get(&(region, names[key as usize].to_string())).map(|fact| Rc::new(fact.clone())))
    };
    let mut tally = Tally::new();
    let stations = match chord_station_values(ctx, frames, &lists, &chords, &mut lookup, table, &mut tally) {
        Ok(stations) => stations.slots.iter().zip(stations.facts.iter()).map(|(slot, value)| (*slot, value.clone())).collect::<Vec<((u32, u32), Value)>>(),
        Err(_) => return Err(SideFail::Declined("the side stations ended in an outcome the port leaves to the oracle".to_string())),
    };
    // `cut_domain`: `facts.setdefault(slot, value)`, a second answer is `STATION_VALUE_CONFLICT`
    for ((region, key), value) in stations {
        let slot = (region, names[key as usize].to_string());
        match facts.index.get(&slot) {
            Some(at) => {
                if facts.values[*at] != *value {
                    return Err(SideFail::Refused("BATCH_DID_NOT_VALIDATE".to_string()));
                }
            }
            None => {
                facts.index.insert(slot.clone(), facts.slots.len());
                facts.slots.push(slot);
                facts.values.push((*value).clone());
            }
        }
    }
    Ok(Side { clipped, facts })
}

/// Keys in dict order (first position, as `dict(...)` of the entries would keep them).
fn dict_keys<'a>(entries: impl Iterator<Item = &'a str>) -> Vec<&'a str> {
    let mut seen: HashSet<&str, FxBuild> = HashSet::default();
    entries.filter(|key| seen.insert(key)).collect()
}

fn same_lists(native: &[Vec<cftuv_clip::emit::KeyedPoint>], base: &[Vec<String>]) -> bool {
    native.len() == base.len() && native.iter().zip(base).all(|(entries, keys)| entries.len() == keys.len() && entries.iter().zip(keys).all(|((key, _), name)| **key == **name))
}

/// `_shape(side) == _shape(inputs.cut)`.
fn same_shape(clipped: &Clipped, base: &BaseCut<'_>) -> bool {
    let polygons = clipped.polygons.len() == base.polygons.len()
        && clipped.polygons.iter().zip(&base.polygons).all(|(face, keys)| {
            face.len() == keys.len() && face.iter().zip(keys).all(|(polygon, names)| polygon.len() == names.len() && polygon.iter().zip(names).all(|(key, name)| **key == **name))
        });
    let points = dict_keys(clipped.points.iter().map(|(key, _)| &**key));
    let snapped = dict_keys(clipped.snapped.iter().map(|(key, _)| key.as_str()));
    polygons
        && same_lists(&clipped.cycles, &base.cycles)
        && same_lists(&clipped.vertex_lists, &base.vertex_lists)
        && same_lists(&clipped.extra_lists, &base.extra_lists)
        && points.len() == base.points.len()
        && points.iter().zip(&base.points).all(|(key, item)| *key == item.name)
        && snapped.len() == base.snapped.len()
        && snapped.iter().zip(&base.snapped).all(|(key, item)| *key == item.name)
}

/// `_second_difference_slope`: `Some(slope)`, `None` when the second difference is not exactly zero.
fn second_difference(low: &SqrtSum, middle: &SqrtSum, high: &SqrtSum, inverse: &Rat) -> Option<SqrtSum> {
    let up = high.sub(middle);
    let down = middle.sub(low);
    if up != down {
        return None;
    }
    Some(up.scaled(inverse))
}

/// `dict(entries)[key]` of a side cut's nodes (the last value of a key, as a dict keeps it).
fn last_values<'a>(entries: impl Iterator<Item = (&'a str, &'a Point)>) -> HashMap<&'a str, &'a Point, FxBuild> {
    let mut found: HashMap<&str, &Point, FxBuild> = HashMap::default();
    for (key, point) in entries {
        found.insert(key, point);
    }
    found
}

/// The slopes of the clip nodes (`(key object, slope)`) and of the clip's facts (`(slot object, slope)`), or the refusal status.
type NodeSlopes<'a, 'py> = Vec<(&'a Keyed<'py>, (SqrtSum, SqrtSum))>;
type FactSlopes<'a, 'py> = Vec<(&'a (Bound<'py, PyAny>, Slot, Point), (SqrtSum, SqrtSum))>;
type Slopes<'a, 'py> = Result<(NodeSlopes<'a, 'py>, FactSlopes<'a, 'py>), u8>;

fn slope_values<'a, 'py>(inputs: &'a Inputs<'py>, up: &Side, down: &Side, delta: &Rat) -> Result<Slopes<'a, 'py>, String> {
    let inverse = Rat::one().div(delta).map_err(|_| "delta is zero (ZeroDivisionError in the oracle)".to_string())?;
    let up_points = last_values(up.clipped.points.iter().map(|(key, point)| (&**key, &**point)));
    let down_points = last_values(down.clipped.points.iter().map(|(key, point)| (&**key, &**point)));
    let up_snapped = last_values(up.clipped.snapped.iter().map(|(key, point)| (key.as_str(), point)));
    let down_snapped = last_values(down.clipped.snapped.iter().map(|(key, point)| (key.as_str(), point)));
    let mut points = Vec::new();
    for (source, high_of, low_of) in [(&inputs.base.points, &up_points, &down_points), (&inputs.base.snapped, &up_snapped, &down_snapped)] {
        for item in source.iter() {
            let (Some(high), Some(low)) = (high_of.get(item.name.as_str()), low_of.get(item.name.as_str())) else {
                return Err("a node of the cut is missing from a side cut (KeyError in the oracle)".to_string());
            };
            let Some(x) = second_difference(&low.0, &item.point.0, &high.0, &inverse) else {
                return Ok(Err(STATUS_CLIP_NODE));
            };
            let Some(y) = second_difference(&low.1, &item.point.1, &high.1, &inverse) else {
                return Ok(Err(STATUS_CLIP_NODE));
            };
            if !(x.is_zero() && y.is_zero()) {
                points.push((item, (x, y)));
            }
        }
    }
    let mut facts = Vec::new();
    for entry in &inputs.added {
        let (_object, slot, middle) = entry;
        let (Some(high), Some(low)) = (up.facts.get(slot), down.facts.get(slot)) else {
            return Err("a fact of the clip is missing from a side cut (KeyError in the oracle)".to_string());
        };
        let Some(s) = second_difference(&low.0, &middle.0, &high.0, &inverse) else {
            return Ok(Err(STATUS_CLIP_FACT));
        };
        let Some(r) = second_difference(&low.1, &middle.1, &high.1, &inverse) else {
            return Ok(Err(STATUS_CLIP_FACT));
        };
        facts.push((entry, (s, r)));
    }
    Ok(Ok((points, facts)))
}

/// The answer of the attempts: the slopes or the oracle's refusal `(status, detail)`; `Err(why)` is a decline of the port.
type Found<'a, 'py> = Result<Result<(NodeSlopes<'a, 'py>, FactSlopes<'a, 'py>), (u8, Option<String>)>, String>;

/// `_clip_slopes` after `_step_for`: up to `attempts` pairs of side cuts, the step halved after a refusal or a change of structure, then the second differences.
#[allow(clippy::too_many_arguments)]
fn attempt_loop<'a, 'py>(
    ctx: &mut ExactCtx<'_>,
    warm: &mut cftuv_clip::warm::Warm,
    plane: &cftuv_clip::plane::Plane,
    inputs: &'a Inputs<'py>,
    frames: &[cftuv_mesh::stations::Frame],
    table: &cftuv_mesh::data::Table,
    law: Law,
    by_faces: bool,
    attempts: u32,
) -> (Found<'a, 'py>, u32) {
    let two = Rat::from_i64(2);
    let mut step = inputs.step.clone();
    let mut last: Option<(u8, Option<String>)> = None;
    let mut ran = 0u32;
    for _attempt in 0..attempts {
        ran += 1;
        let shift = step.mul(&inputs.scale);
        let up = match side_cut(ctx, warm, plane, inputs, frames, table, law, by_faces, &shift) {
            Ok(up) => up,
            Err(SideFail::Declined(text)) => return (Err(text), ran),
            Err(SideFail::Refused(outcome)) => {
                last = Some((STATUS_CLIP_REFUSED, Some(outcome)));
                step = step.div(&two).expect("two is not zero");
                continue;
            }
        };
        let down = match side_cut(ctx, warm, plane, inputs, frames, table, law, by_faces, &shift.neg()) {
            Ok(down) => down,
            Err(SideFail::Declined(text)) => return (Err(text), ran),
            Err(SideFail::Refused(outcome)) => {
                last = Some((STATUS_CLIP_REFUSED, Some(outcome)));
                step = step.div(&two).expect("two is not zero");
                continue;
            }
        };
        if !same_shape(&up.clipped, &inputs.base) || !same_shape(&down.clipped, &inputs.base) {
            last = Some((STATUS_STRUCTURE_CHANGED, None));
            step = step.div(&two).expect("two is not zero");
            continue;
        }
        return match slope_values(inputs, &up, &down, &shift) {
            Ok(Ok(slopes)) => (Ok(Ok(slopes)), ran),
            Ok(Err(status)) => (Ok(Err((status, None))), ran),
            Err(text) => (Err(text), ran),
        };
    }
    match last {
        Some(refused) => (Ok(Err(refused)), ran),
        None => (Err("no attempt ran (attempts is zero)".to_string()), ran),
    }
}

fn nanos(since: Instant) -> u64 {
    u64::try_from(since.elapsed().as_nanos()).unwrap_or(u64::MAX)
}

fn declined<'py>(detail: String, timings: [u64; 4]) -> Answer<'py> {
    (STATUS_DECLINED, Some(detail), None, None, [0; 5], 0, timings, 0)
}

/// The whole chain (module note). `sync` is the memory sync in the wire format (`None`: unchanged), `tables` the real memory tables the memory log is replayed on.
#[allow(clippy::too_many_arguments)]
pub fn side_slopes<'py>(
    py: Python<'py>,
    session: &mut Session,
    clip: &mut crate::clip::Host,
    w1: &mut crate::materialize_w1::Host,
    args: &SideArgs<'_, 'py>,
    sync: Option<&[u8]>,
    tables: &Tables<'py>,
    forced: Option<Forced>,
) -> PyResult<Answer<'py>> {
    let started = Instant::now();
    let Ok(frame_faces) = args.frame_faces.cast_exact::<PyList>() else {
        return Ok(declined("frame_faces is not a list".to_string(), [nanos(started), 0, 0, 0]));
    };
    let (plane, warm, pool, insort) = clip.certificate_parts(py, args.triangles)?;
    let inputs = match read(pool, args) {
        Ok(inputs) => inputs,
        Err(Fail::Py(error)) => return Err(error),
        Err(Fail::Unsupported(text)) => return Ok(declined(text, [nanos(started), 0, 0, 0])),
    };
    let (table, frames) = match w1.certificate_frames(args.table, frame_faces, &inputs.regions) {
        Ok(found) => found,
        Err(Err(error)) => return Err(error),
        Err(Ok(text)) => return Ok(declined(text, [nanos(started), 0, 0, 0])),
    };
    if inputs.regions.len() < inputs.polygons.len() {
        return Ok(declined("more polygon lists than frame faces (IndexError in the oracle)".to_string(), [nanos(started), 0, 0, 0]));
    }
    let law = match args.law {
        0 => Law::PlanarPolygons,
        1 => Law::QuadStrips,
        _ => Law::Ears,
    };
    let sync_value = match sync {
        Some(bytes) => Some(Reader::new(bytes, true).get_value().map_err(|error| PyValueError::new_err(format!("the memory sync is not in the wire format: {error}")))?),
        None => None,
    };
    // the side cuts run on throwaway unlimited budgets in the oracle: no cap, and the articles are not reported
    let mut run = CostRun::begin_parts(session, sync_value.as_ref(), None, None).map_err(|error| PyValueError::new_err(error.to_string()))?;
    let read_ns = nanos(started);

    let computing = Instant::now();
    let outcome = catch_unwind(AssertUnwindSafe(|| {
        let mut counts = SignCounts::default();
        let (found, attempts) = {
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            attempt_loop(&mut ctx, warm, plane, &inputs, &frames, &table, law, args.by_faces, args.attempts)
        };
        (found, counts, session.memory.take_log(), attempts)
    }));
    let compute_ns = nanos(computing);
    let (found, counts, log, attempts) = match outcome {
        Ok(done) => done,
        Err(_) => return Err(PyRuntimeError::new_err("Session.certificate_side_slopes: native panic")),
    };
    let found = match (found, forced) {
        (_, Some(Forced::Unsupported)) => Err("forced by the test knob".to_string()),
        (found, _) => found,
    };
    let found = match found {
        Ok(found) => found,
        Err(text) => return Ok(declined(text, [read_ns, compute_ns, 0, 0])),
    };
    let building = Instant::now();
    let (status, detail, point_slopes, fact_slopes) = match found {
        Ok((points, facts)) => {
            let pair = |value: &(SqrtSum, SqrtSum)| -> PyResult<Bound<'py, PyAny>> { Ok(PyTuple::new(py, [sqrt_sum_to_py(py, pool, &value.0)?, sqrt_sum_to_py(py, pool, &value.1)?])?.into_any()) };
            let point_dict = PyDict::new(py);
            for (key, value) in &points {
                point_dict.set_item(&key.object, pair(value)?)?;
            }
            let fact_dict = PyDict::new(py);
            for (slot, value) in &facts {
                fact_dict.set_item(&slot.0, pair(value)?)?;
            }
            (STATUS_OK, None, Some(point_dict.into_any()), Some(fact_dict.into_any()))
        }
        Err((status, detail)) => (status, detail, None, None),
    };
    let build_ns = nanos(building);
    let applying = Instant::now();
    let changed = apply_log(py, pool, insort, tables, log)?;
    let log_ns = nanos(applying);
    Ok((status, detail, point_slopes, fact_slopes, counts.as_array(), changed, [read_ns, compute_ns, build_ns, log_ns], attempts))
}

// --------------------------------------------------------------------------
// the whole certificate after the slopes of the facts (CERTIFICATE_CORE_NATIVE_V1)
// --------------------------------------------------------------------------

pub const STATUS_LIFT_UNSUPPORTED: u8 = 31;
pub const STATUS_LIFT_ON_EDGE: u8 = 32;
pub const STATUS_FACT_UNRESOLVED: u8 = 33;

/// What the shim hands over for the arrays of one certificate (`parametric._certificate_tail` on a bound surface, not exact).
pub struct ArrayArgs<'r, 'py> {
    pub vertices: &'r Bound<'py, PyAny>,
    pub faces: &'r Bound<'py, PyAny>,
    pub points_after: &'r Bound<'py, PyAny>,
    pub key_slope: &'r Bound<'py, PyAny>,
    pub lifted_names: &'r Bound<'py, PyAny>,
    /// `{vertex key: (low, high)}` of the cut points of the coverage templates (`_cut_point_limits` by the identity of the slope's objects, resolved by the shim).
    pub reach: &'r Bound<'py, PyAny>,
    pub window: (f64, f64),
    pub developable: bool,
    pub scale: &'r Bound<'py, PyAny>,
    /// `{("region:N", key): (float s, float r)}` of the facts before the cut (the slope memory's floats).
    pub fact_floats: &'r Bound<'py, PyAny>,
}

/// `(status, detail, (position slopes, loop slopes, limits, floors) as float64 bytes, normal rate, (vertices, loops, moving vertices, clip node slopes, clip fact slopes),
/// sign-counter deltas, changed-tables bits, (read, compute, build, memory log) nanoseconds, attempts)`.
pub type CoreAnswer<'py> = (u8, Option<String>, Option<[Bound<'py, PyBytes>; 4]>, f64, [u64; 5], [u64; 5], u8, [u64; 4], u32);

fn core_declined<'py>(detail: String, timings: [u64; 4]) -> CoreAnswer<'py> {
    (STATUS_DECLINED, Some(detail), None, 0.0, [0; 5], [0; 5], 0, timings, 0)
}

fn core_refused<'py>(status: u8, detail: Option<String>, counts: [u64; 5], changed: u8, timings: [u64; 4], attempts: u32) -> CoreAnswer<'py> {
    (status, detail, None, 0.0, [0; 5], counts, changed, timings, attempts)
}

fn attr<'py>(object: &Bound<'py, PyAny>, name: &Bound<'py, PyString>) -> Read<Bound<'py, PyAny>> {
    Ok(object.getattr(name)?)
}

/// `_triangle_slope(triangle, slope, scale)`: the slope of the lifted position in the source triangle (metres per metre of width).
fn triangle_slope(triangle: &cftuv_clip::plane::Triangle, slope: &Point, scale: &Rat) -> Result<[SqrtSum; 3], String> {
    let (x1, y1) = slope;
    let weight = |edge: usize| {
        let (start, end) = (&triangle.chart[edge], &triangle.chart[(edge + 1) % 3]);
        y1.scaled(&end.0.sub(&start.0)).sub(&x1.scaled(&end.1.sub(&start.1)))
    };
    let ordered = [weight(1), weight(2), weight(0)];
    let mut found = [SqrtSum::zero(), SqrtSum::zero(), SqrtSum::zero()];
    for (axis, total) in found.iter_mut().enumerate() {
        for (item, corner) in ordered.iter().zip(&triangle.corners) {
            let factor = corner[axis].mul(scale).div(&triangle.twice_area).map_err(|_| "a triangle of zero area (ZeroDivisionError in the oracle)".to_string())?;
            *total = total.add(&item.scaled(&factor));
        }
    }
    Ok(found)
}

/// `_normal_rate(triangle, numbers, scale)`: the binary64 operations of the oracle in its order.
fn normal_rate(triangle: &cftuv_clip::plane::Triangle, numbers: &([f64; 3], [f64; 3]), scale: f64) -> Result<f64, String> {
    let Some(normals) = &triangle.normals else {
        return Ok(0.0);
    };
    let divisor = cftuv_core::pyfloat::rat_to_f64(&triangle.twice_area).map_err(|_| "the doubled area overflows a float".to_string())?;
    let (values, slopes) = numbers;
    let order = [1usize, 2, 0];
    let weights = order.map(|index| values[index] / divisor);
    let rates = order.map(|index| slopes[index] * scale / divisor);
    let blend = |factors: &[f64; 3], axis: usize| {
        let mut total = 0.0f64;
        for (factor, normal) in factors.iter().zip(normals) {
            total += factor * normal[axis];
        }
        total
    };
    let mixed = [blend(&weights, 0), blend(&weights, 1), blend(&weights, 2)];
    let moved = [blend(&rates, 0), blend(&rates, 1), blend(&rates, 2)];
    let mut squares = 0.0f64;
    for item in &mixed {
        squares += item * item;
    }
    let length = squares.sqrt();
    if !(length > 0.0) {
        return Ok(f64::INFINITY);
    }
    let unit = mixed.map(|item| item / length);
    let mut along = 0.0f64;
    for (a, b) in unit.iter().zip(&moved) {
        along += a * b;
    }
    let mut rest = 0.0f64;
    for (m, u) in moved.iter().zip(&unit) {
        let item = m - u * along;
        rest += item * item;
    }
    Ok(rest.sqrt() / length)
}

fn float_of(value: &SqrtSum) -> Result<f64, String> {
    cftuv_clip::lift::sqrt_sum_binary64(value).map_err(|_| "an exact value without a binary64 (OverflowError in the oracle)".to_string())
}

fn bytes_of<'py>(py: Python<'py>, values: &[f64]) -> Bound<'py, PyBytes> {
    let mut raw = Vec::with_capacity(values.len() * 8);
    for value in values {
        raw.extend_from_slice(&value.to_le_bytes());
    }
    PyBytes::new(py, &raw)
}

struct UniformSlopes {
    physical: HashMap<Slot, Point, FxBuild>,
    forms: HashMap<Slot, Vec<[SqrtSum; 4]>, FxBuild>,
    products: ProductMemo,
}

impl UniformSlopes {
    fn read(pool: &Pool, physical: &Bound<'_, PyAny>, forms: &Bound<'_, PyAny>, clipped: &FactSlopes<'_, '_>) -> Read<Self> {
        let mut slopes = HashMap::default();
        for (slot, value) in dict(physical, "physical fact slopes")?.iter() {
            slopes.insert(carried(slot.extract::<Slot>(), "a physical slope slot")?, pair_of(pool, &value, "a physical slope")?);
        }
        for (entry, value) in clipped {
            slopes.insert(entry.1.clone(), value.clone());
        }
        let mut charts = HashMap::default();
        for (slot, candidates) in dict(forms, "UV affine forms")?.iter() {
            let mut rows = Vec::new();
            for candidate in candidates.try_iter()? {
                let candidate = candidate?;
                let row: Vec<_> = candidate.try_iter()?.collect::<PyResult<_>>()?;
                if row.len() != 4 {
                    return unsupported("a UV affine form is not (K, Tx, Ty, B)");
                }
                let mut values = Vec::with_capacity(4);
                for value in row {
                    values.push(carried(sqrt_sum_from_py(pool, &value), "a UV affine coefficient")?);
                }
                rows.push(values.try_into().map_err(|_| Fail::Unsupported("a UV affine form is not four coefficients".into()))?);
            }
            charts.insert(carried(slot.extract::<Slot>(), "a UV affine slot")?, rows);
        }
        Ok(Self { physical: slopes, forms: charts, products: ProductMemo::default() })
    }

    fn at(&mut self, slot: &Slot, point: &Point) -> Result<Point, String> {
        let label = format!("region:{}/{}", slot.0, slot.1);
        let forms = self.forms.get(slot).ok_or_else(|| format!("UV_UNIFORM_SLOPE_FORM_MISSING: {label}"))?;
        let physical = self.physical.get(slot).ok_or_else(|| format!("UV_UNIFORM_PHYSICAL_SLOPE_MISSING: {label}"))?;
        let mut numerator = None;
        for form in forms {
            let value = uniform_numerator_slope(form, point, &physical.1, &mut self.products);
            if numerator.as_ref().is_some_and(|known| known != &value) {
                return Err(format!("UV_UNIFORM_SLOPE_OWNER_CONFLICT: {label}"));
            }
            numerator = Some(value);
        }
        Ok((numerator.ok_or_else(|| format!("UV_UNIFORM_SLOPE_OWNER_CONFLICT: {label}"))?, physical.1.clone()))
    }
}

/// The arrays of `_certificate` after the slopes: `Ok(Ok(arrays))`, `Ok(Err((status, detail)))` an oracle refusal, `Err(why)` a decline.
#[allow(clippy::type_complexity)]
fn arrays<'py>(
    py: Python<'py>,
    pool: &Pool,
    plane: &cftuv_clip::plane::Plane,
    args: &ArrayArgs<'_, 'py>,
    nodes: &NodeSlopes<'_, 'py>,
    facts: &FactSlopes<'_, 'py>,
) -> Read<Result<([Vec<f64>; 4], f64, [u64; 5]), (u8, Option<String>)>> {
    let names = (PyString::intern(py, "vert_key"), PyString::intern(py, "value"), PyString::intern(py, "semantic_region_id"), PyString::intern(py, "uv_facts"));
    let scale = rat_of(pool, args.scale, "the scale")?;
    let scale_float = carried(args.scale.extract::<f64>(), "the scale")?;
    let mut clip_points: HashMap<&str, &(SqrtSum, SqrtSum), FxBuild> = HashMap::default();
    for (item, slope) in nodes {
        clip_points.insert(item.name.as_str(), slope);
    }
    let mut triangles: HashMap<&str, usize, FxBuild> = HashMap::default();
    for (index, triangle) in plane.triangles.iter().enumerate() {
        triangles.insert(triangle.name.as_str(), index);
    }
    let points_after = dict(args.points_after, "points_after")?;
    let key_slope = dict(args.key_slope, "key_slope")?;
    let lifted = dict(args.lifted_names, "lifted_names")?;
    let reach = dict(args.reach, "the reach of the cut points")?;
    let mut keys = Vec::new();
    for vertex in args.vertices.try_iter()? {
        keys.push(text(&attr(&attr(&vertex?, &names.0)?, &names.1)?, "a vertex key")?);
    }
    keys.sort();
    let unbounded = (f64::NEG_INFINITY, f64::INFINITY);
    let (mut positions, mut limits, mut floors) = (Vec::with_capacity(keys.len() * 3), Vec::with_capacity(keys.len()), Vec::with_capacity(keys.len()));
    let mut moving = 0u64;
    let mut rate = 0.0f64;
    for key in &keys {
        let Some(point) = points_after.get_item(key)? else {
            return unsupported("a vertex of the batch without its point (KeyError in the oracle)");
        };
        let native;
        let (slope, clipped) = match clip_points.get(key.as_str()) {
            Some(found) => (Some(*found), true),
            None => match key_slope.get_item(key)? {
                Some(found) if !found.is_none() => {
                    native = pair_of(pool, &found, "a key slope")?;
                    (Some(&native), false)
                }
                _ => (None, false),
            },
        };
        let (floor, limit) = match slope {
            None => unbounded,
            Some(_) if clipped => args.window,
            Some(_) => match reach.get_item(key)? {
                Some(found) => carried(found.extract::<(f64, f64)>(), "the reach of a cut point")?,
                None => args.window,
            },
        };
        floors.push(floor);
        limits.push(limit);
        let Some(slope) = slope else {
            positions.extend_from_slice(&[0.0, 0.0, 0.0]);
            continue;
        };
        moving += 1;
        let name = match lifted.get_item(key)? {
            Some(found) => {
                let first = found.get_item(0)?;
                if first.is_none() {
                    None
                } else {
                    Some(text(&first, "a triangle name")?)
                }
            }
            None => None,
        };
        let Some(index) = name.as_deref().and_then(|name| triangles.get(name)).copied() else {
            return Ok(Err((STATUS_LIFT_UNSUPPORTED, Some(format!("vertex {key}: no source triangle")))));
        };
        let point = pair_of(pool, &point, "a point after the cut")?;
        let Ok((leaves, numbers)) = cftuv_clip::surface::vertex_facts(plane, index, &point, slope, args.developable) else {
            return unsupported("the facts of a moving vertex have no binary64");
        };
        if leaves {
            return Ok(Err((STATUS_LIFT_ON_EDGE, Some(format!("vertex {key}")))));
        }
        let triangle = &plane.triangles[index];
        for value in triangle_slope(triangle, slope, &scale).map_err(Fail::Unsupported)? {
            positions.push(float_of(&value).map_err(Fail::Unsupported)?);
        }
        if args.developable {
            let numbers = numbers.ok_or_else(|| Fail::Unsupported("the floats of a developable vertex are missing".to_string()))?;
            let found = normal_rate(triangle, &numbers, scale_float).map_err(Fail::Unsupported)?;
            // `max(normal_rate, found)`: the first of equals, a NaN never wins
            if found > rate {
                rate = found;
            }
        }
    }
    // the loops: the floats of the facts before the cut (the memory's), the clip's facts over them
    let (physical_floats, mut uniform) = if let Ok(payload) = args.fact_floats.cast_exact::<PyTuple>() {
        if payload.len() != 3 {
            return unsupported("the uniform slope payload is not (floats, physical slopes, affine forms)");
        }
        (payload.get_item(0)?, Some(UniformSlopes::read(pool, &payload.get_item(1)?, &payload.get_item(2)?, facts)?))
    } else {
        (args.fact_floats.clone(), None)
    };
    let mut floats: HashMap<(String, String), (f64, f64), FxBuild> = HashMap::default();
    for (slot, pair) in dict(&physical_floats, "fact_floats")?.iter() {
        let slot: (String, String) = carried(slot.extract(), "a fact slot of the floats")?;
        floats.insert(slot, carried(pair.extract::<(f64, f64)>(), "the floats of a fact")?);
    }
    let mut clip_facts: HashSet<&Slot, FxBuild> = HashSet::default();
    for (entry, (s, r)) in facts {
        clip_facts.insert(&entry.1);
        floats.insert((format!("region:{}", entry.1 .0), entry.1 .1.clone()), (float_of(s).map_err(Fail::Unsupported)?, float_of(r).map_err(Fail::Unsupported)?));
    }
    let mut loops = Vec::new();
    let mut count = 0u64;
    for face in args.faces.try_iter()? {
        let face = face?;
        let region = text(&attr(&attr(&face, &names.2)?, &names.1)?, "a region id")?;
        let mut lookup = (region, String::new());
        for fact in attr(&face, &names.3)?.try_iter()? {
            lookup.1 = text(&attr(&attr(&fact?, &names.0)?, &names.1)?, "a vertex key")?;
            if let Some(uniform) = uniform.as_mut() {
                let region = lookup.0.strip_prefix("region:").and_then(|text| text.parse::<u32>().ok()).ok_or_else(|| Fail::Unsupported("a malformed UV region".into()))?;
                let mut point = (SqrtSum::zero(), SqrtSum::zero());
                if let Some(found) = clip_points.get(lookup.1.as_str()) {
                    point = (*found).clone();
                } else if let Some(found) = key_slope.get_item(&lookup.1)? {
                    if !found.is_none() {
                        point = pair_of(pool, &found, "a key slope")?;
                    }
                }
                let pair = match uniform.at(&(region, lookup.1.clone()), &point) {
                    Ok(pair) => pair,
                    Err(detail) => return Ok(Err((STATUS_FACT_UNRESOLVED, Some(detail)))),
                };
                loops.push(float_of(&pair.0).map_err(Fail::Unsupported)?);
                loops.push(float_of(&pair.1).map_err(Fail::Unsupported)?);
                count += 1;
                continue;
            }
            let Some(pair) = floats.get(&lookup) else {
                return Ok(Err((STATUS_FACT_UNRESOLVED, Some(format!("{}/{}", lookup.0, lookup.1)))));
            };
            loops.push(pair.0);
            loops.push(pair.1);
            count += 1;
        }
    }
    let unique_nodes = clip_points.len() as u64;
    Ok(Ok(([positions, loops, limits, floors], rate, [keys.len() as u64, count, moving, unique_nodes, clip_facts.len() as u64])))
}

/// `_certificate` after the slopes of the facts, whole (CERTIFICATE_CORE_NATIVE_V1): the side cuts when the domain has a cut (`side`), then the arrays.
#[allow(clippy::too_many_arguments)]
pub fn core<'py>(
    py: Python<'py>,
    session: &mut Session,
    clip: &mut crate::clip::Host,
    w1: &mut crate::materialize_w1::Host,
    triangles: &Bound<'py, PyAny>,
    side: Option<&SideArgs<'_, 'py>>,
    array_args: &ArrayArgs<'_, 'py>,
    sync: Option<&[u8]>,
    tables: &Tables<'py>,
    forced: Option<Forced>,
) -> PyResult<CoreAnswer<'py>> {
    let started = Instant::now();
    let (plane, warm, pool, insort) = clip.certificate_parts(py, triangles)?;
    let inputs = match side.map(|args| read(pool, args)).transpose() {
        Ok(inputs) => inputs,
        Err(Fail::Py(error)) => return Err(error),
        Err(Fail::Unsupported(text)) => return Ok(core_declined(text, [nanos(started), 0, 0, 0])),
    };
    let mut prepared = None;
    if let (Some(args), Some(inputs)) = (side, inputs.as_ref()) {
        let Ok(frame_faces) = args.frame_faces.cast_exact::<PyList>() else {
            return Ok(core_declined("frame_faces is not a list".to_string(), [nanos(started), 0, 0, 0]));
        };
        match w1.certificate_frames(args.table, frame_faces, &inputs.regions) {
            Ok(found) => prepared = Some(found),
            Err(Err(error)) => return Err(error),
            Err(Ok(text)) => return Ok(core_declined(text, [nanos(started), 0, 0, 0])),
        }
        if inputs.regions.len() < inputs.polygons.len() {
            return Ok(core_declined("more polygon lists than frame faces (IndexError in the oracle)".to_string(), [nanos(started), 0, 0, 0]));
        }
    }
    let read_ns = nanos(started);
    let computing = Instant::now();
    let mut counts = SignCounts::default();
    let mut log = Vec::new();
    let mut attempts = 0u32;
    let mut found: Found<'_, 'py> = Ok(Ok((Vec::new(), Vec::new())));
    if let (Some(args), Some(inputs), Some((table, frames))) = (side, inputs.as_ref(), prepared.as_ref()) {
        let law = match args.law {
            0 => Law::PlanarPolygons,
            1 => Law::QuadStrips,
            _ => Law::Ears,
        };
        let sync_value = match sync {
            Some(bytes) => Some(Reader::new(bytes, true).get_value().map_err(|error| PyValueError::new_err(format!("the memory sync is not in the wire format: {error}")))?),
            None => None,
        };
        let mut run = CostRun::begin_parts(session, sync_value.as_ref(), None, None).map_err(|error| PyValueError::new_err(error.to_string()))?;
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut counts = SignCounts::default();
            let (found, attempts) = {
                let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
                attempt_loop(&mut ctx, warm, plane, inputs, frames, table, law, args.by_faces, args.attempts)
            };
            (found, counts, session.memory.take_log(), attempts)
        }));
        (found, counts, log, attempts) = match outcome {
            Ok(done) => done,
            Err(_) => return Err(PyRuntimeError::new_err("Session.certificate_core: native panic")),
        };
    }
    if forced == Some(Forced::Unsupported) {
        found = Err("forced by the test knob".to_string());
    }
    let slopes = match found {
        Ok(slopes) => slopes,
        Err(text) => return Ok(core_declined(text, [read_ns, nanos(computing), 0, 0])),
    };
    let built = match &slopes {
        Ok((nodes, facts)) => match arrays(py, pool, plane, array_args, nodes, facts) {
            Ok(built) => built,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(core_declined(text, [read_ns, nanos(computing), 0, 0])),
        },
        Err((status, detail)) => Err((*status, detail.clone())),
    };
    let compute_ns = nanos(computing);
    let building = Instant::now();
    let (columns, rate, sizes) = match built {
        Ok(built) => built,
        Err((status, detail)) => {
            let applying = Instant::now();
            let changed = apply_log(py, pool, insort, tables, log)?;
            return Ok(core_refused(status, detail, counts.as_array(), changed, [read_ns, compute_ns, 0, nanos(applying)], attempts));
        }
    };
    let [positions, loops, limits, floors] = &columns;
    let columns = [bytes_of(py, positions), bytes_of(py, loops), bytes_of(py, limits), bytes_of(py, floors)];
    let build_ns = nanos(building);
    let applying = Instant::now();
    let changed = apply_log(py, pool, insort, tables, log)?;
    let log_ns = nanos(applying);
    Ok((STATUS_OK, None, Some(columns), rate, sizes, counts.as_array(), changed, [read_ns, compute_ns, build_ns, log_ns], attempts))
}
