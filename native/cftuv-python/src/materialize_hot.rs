//! The hot part of the materializer's SILHOUETTE stage behind the persistent session (`cftuv-silhouette`):
//!
//! * `silhouette`: `silhouette.apply_silhouette` (the pass `dissolve_silhouette` and the check `verify_silhouette`) with the chains of the final
//!   faces. The answer is `(result or None, status, detail, counts, articles, changed tables, timings)` as for W1; the result is
//!   `(kind, faces, gone, counters, note, changed, runs, fits, boundary, interface, problems)` with the key strings of the input, and the shim
//!   builds `SilhouetteV1` (and raises the oracle's refusal when `problems` is not empty). The budget the call runs on is the one the shim hands
//!   over: the INNER budget of the pass (cap = what the domain has left, articles zero) when the domain's budget has a cap, so an exhaustion is
//!   the pass's `SKIPPED_WORK_BUDGET` (kind 2) and the domain's budget is untouched, as in the oracle.
//! * `plane_lift`: `PlaneLiftV1.lift` of every point of a dict (no budget, no memory).
//!
//! An input the port does not carry (a class, a missing key, a value the oracle would raise on) is status 12 (`STATUS_UNSUPPORTED`): nothing of
//! the call is applied, the session forgets its mirror, and the shim runs the oracle.

use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyFloat, PyFrozenSet, PyInt, PyList, PyString, PyTuple};
use pyo3::{ffi, IntoPyObjectExt};

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::codec::Reader;
use cftuv_core::exact::ExactCtx;
use cftuv_core::rat::Rat;
use cftuv_core::session::{CostRun, Session};
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_clip::error::ClipError;
use cftuv_silhouette::plane_lift::Plane;
use cftuv_silhouette::silhouette::{apply_silhouette, Key, SilError, SilInput, SilOutput};
use cftuv_silhouette::source_lift::{lift_source_vertices, settle_emitted_faces, LiftInput, LiftOutput, Placed, Settled};

use crate::coverage::exact_status;
use crate::memlog::{apply_log, insort, Tables};
use crate::refusal::is_native_only;
use crate::pyobj::{alloc, ibig_from_int, note_attr_build, note_attr_reads, rat_from_number, set_slot, sqrt_sum_from_py, Pool, Raw, SlotMode};

const STATUS_UNSUPPORTED: u8 = 12;

/// `(result, status, detail, counts, articles, changed, timings)`.
pub type Answer<'py> = (Option<Bound<'py, PyAny>>, u8, Option<Bound<'py, PyTuple>>, [u64; 5], [u64; 6], u8, [u64; 8]);

pub(crate) enum Fail {
    Py(PyErr),
    Unsupported(String),
}

impl From<PyErr> for Fail {
    fn from(error: PyErr) -> Fail {
        Fail::Py(error)
    }
}

pub(crate) type Read<T> = Result<T, Fail>;

pub(crate) fn unsupported<T>(text: impl Into<String>) -> Read<T> {
    Err(Fail::Unsupported(text.into()))
}

/// A conversion of the boundary that refuses (`TypeError("cftuv_native: ...")`) is an input the port does not carry, not an error of the call.
pub(crate) fn carried<T>(found: PyResult<T>, what: &str) -> Read<T> {
    found.map_err(|error| Fail::Unsupported(format!("{what}: {error}")))
}

pub(crate) fn nanos(since: Instant) -> u64 {
    u64::try_from(since.elapsed().as_nanos()).unwrap_or(u64::MAX)
}

pub(crate) struct Classes {
    pub(crate) pool: Pool,
    pub(crate) local_point: Py<PyAny>,
    local_raw: Option<Raw>,
    x: Py<PyString>,
    y: Py<PyString>,
    z: Py<PyString>,
    insort: Py<PyAny>,
}

#[derive(Default)]
pub struct Host {
    classes: Option<Classes>,
    pub(crate) assemble: Option<crate::assemble_hot::AssembleClasses>,
}

impl Host {
    /// `SqrtSumV1`, `Fraction`, `LocalPoint3V1`.
    pub fn bind(&mut self, py: Python<'_>, sqrt_sum: &Bound<'_, PyAny>, fraction: &Bound<'_, PyAny>, local_point: &Bound<'_, PyAny>, mode: SlotMode) -> PyResult<()> {
        let x = PyString::intern(py, "x").unbind();
        let y = PyString::intern(py, "y").unbind();
        let z = PyString::intern(py, "z").unbind();
        let local_point = local_point.clone().unbind();
        let local_raw = Raw::select(py, mode, "LocalPoint3V1", &local_point, &[&x, &y, &z])?;
        self.classes = Some(Classes { pool: Pool::new(py, fraction, sqrt_sum, mode)?, local_point, local_raw, x, y, z, insort: insort(py)? });
        Ok(())
    }

    pub(crate) fn classes(&self) -> PyResult<&Classes> {
        self.classes.as_ref().ok_or_else(|| PyRuntimeError::new_err("cftuv_native: the materialize_hot classes were not bound (`bind_hot`)"))
    }

    pub fn disable_raw(&mut self) {
        if let Some(classes) = self.classes.as_mut() {
            classes.pool.disable_raw();
            classes.local_raw = None;
        }
    }
}

// --------------------------------------------------------------------------
// reading
// --------------------------------------------------------------------------

/// The items of an exact `list` or `tuple`.
pub(crate) fn sequence<'py>(object: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Bound<'py, PyAny>>> {
    if let Ok(list) = object.cast_exact::<PyList>() {
        return Ok(list.iter().collect());
    }
    if let Ok(tuple) = object.cast_exact::<PyTuple>() {
        return Ok(tuple.iter().collect());
    }
    unsupported(format!("{what} is not a list or a tuple"))
}

pub(crate) fn exact_dict<'a, 'py>(object: &'a Bound<'py, PyAny>, what: &str) -> Read<&'a Bound<'py, PyDict>> {
    match object.cast_exact::<PyDict>() {
        Ok(dict) => Ok(dict),
        Err(_) => unsupported(format!("{what} is not a dict")),
    }
}

pub(crate) fn utf8<'a>(py: Python<'_>, object: &Bound<'_, PyAny>) -> Read<&'a [u8]> {
    if !object.is_exact_instance_of::<PyString>() {
        return unsupported("a vertex key is not a str");
    }
    let mut size: ffi::Py_ssize_t = 0;
    // SAFETY: an exact `str`; its UTF-8 buffer lives as long as the string, which the caller's containers keep alive for the whole call.
    let data = unsafe { ffi::PyUnicode_AsUTF8AndSize(object.as_ptr(), &mut size) };
    if data.is_null() {
        return Err(Fail::Py(PyErr::fetch(py)));
    }
    // SAFETY: borrowed from a live immutable string for the lifetime of the call.
    Ok(unsafe { std::slice::from_raw_parts(data.cast::<u8>(), size as usize) })
}

pub(crate) fn small_int(object: &Bound<'_, PyAny>, what: &str) -> Read<u32> {
    if !object.is_exact_instance_of::<PyInt>() {
        return unsupported(format!("{what} is not an int"));
    }
    let value = ibig_from_int(object)?;
    u32::try_from(&value).map_err(|_| Fail::Unsupported(format!("{what} is not a 32-bit non-negative int")))
}

/// The vertex keys of one call, numbered by first sight, with the objects (the result hands back the input strings).
pub(crate) struct Keys<'a, 'py> {
    map: HashMap<&'a [u8], Key, FxBuild>,
    pub(crate) bytes: Vec<&'a [u8]>,
    pub(crate) objects: Vec<Bound<'py, PyAny>>,
}

impl<'a, 'py> Keys<'a, 'py> {
    pub(crate) fn new() -> Keys<'a, 'py> {
        Keys { map: HashMap::default(), bytes: Vec::new(), objects: Vec::new() }
    }

    pub(crate) fn key(&mut self, py: Python<'py>, object: &Bound<'py, PyAny>) -> Read<Key> {
        let bytes: &'a [u8] = utf8(py, object)?;
        if let Some(found) = self.map.get(bytes) {
            return Ok(*found);
        }
        let next = self.bytes.len() as Key;
        self.map.insert(bytes, next);
        self.bytes.push(bytes);
        self.objects.push(object.clone());
        Ok(next)
    }

    /// The rank of each key's name in Python's `str` order (the byte order of UTF-8).
    pub(crate) fn ranks(&self) -> Vec<u32> {
        let mut order: Vec<u32> = (0..self.bytes.len() as u32).collect();
        order.sort_by(|a, b| self.bytes[*a as usize].cmp(self.bytes[*b as usize]));
        let mut rank = vec![0u32; order.len()];
        for (position, index) in order.into_iter().enumerate() {
            rank[index as usize] = position as u32;
        }
        rank
    }

    /// `audit.location_key` of every key, numbered.
    pub(crate) fn places(&self, mark: &[u8]) -> Vec<u32> {
        let mut map: HashMap<&[u8], u32, FxBuild> = HashMap::default();
        let mut found = Vec::with_capacity(self.bytes.len());
        for bytes in &self.bytes {
            let place = if !mark.is_empty() && bytes.ends_with(mark) { &bytes[..bytes.len() - mark.len()] } else { bytes };
            let next = map.len() as u32;
            found.push(*map.entry(place).or_insert(next));
        }
        found
    }
}

pub(crate) fn float_of(object: &Bound<'_, PyAny>) -> Read<f64> {
    if !object.is_exact_instance_of::<PyFloat>() {
        return unsupported("a position coordinate is not a float");
    }
    // SAFETY: an exact float.
    Ok(unsafe { ffi::PyFloat_AsDouble(object.as_ptr()) })
}

pub(crate) fn position_of(classes: &Classes, object: &Bound<'_, PyAny>) -> Read<[f64; 3]> {
    if object.get_type().as_ptr() != classes.local_point.as_ptr() {
        return unsupported("a position is not exactly a LocalPoint3V1");
    }
    if let Some(raw) = &classes.local_raw {
        if let (Some(x), Some(y), Some(z)) = (raw.read(object, 0), raw.read(object, 1), raw.read(object, 2)) {
            return Ok([float_of(&x)?, float_of(&y)?, float_of(&z)?]);
        }
    }
    note_attr_reads(3);
    let py = object.py();
    Ok([float_of(&object.getattr(classes.x.bind(py))?)?, float_of(&object.getattr(classes.y.bind(py))?)?, float_of(&object.getattr(classes.z.bind(py))?)?])
}

pub(crate) fn pair_of(pool: &Pool, object: &Bound<'_, PyAny>, what: &str) -> Read<(SqrtSum, SqrtSum)> {
    let Ok(pair) = object.cast_exact::<PyTuple>() else {
        return unsupported(format!("{what} is not a tuple"));
    };
    if pair.len() != 2 {
        return unsupported(format!("{what} is not a pair"));
    }
    Ok((carried(sqrt_sum_from_py(pool, &pair.get_item(0)?), what)?, carried(sqrt_sum_from_py(pool, &pair.get_item(1)?), what)?))
}

pub(crate) fn rat_of(pool: &Pool, object: &Bound<'_, PyAny>, what: &str) -> Read<Rat> {
    carried(rat_from_number(pool, object), what)
}

/// What the shim hands over for one `apply_silhouette`.
pub struct SilhouetteArgs<'r, 'py> {
    pub polygons: &'r Bound<'py, PyAny>,
    pub cycles: &'r Bound<'py, PyAny>,
    pub positions: &'r Bound<'py, PyAny>,
    pub points: &'r Bound<'py, PyAny>,
    pub facts: &'r Bound<'py, PyAny>,
    pub regions: &'r Bound<'py, PyAny>,
    pub cut_pairs: &'r Bound<'py, PyAny>,
    pub mark: &'r str,
    pub lattice_alpha: &'r Bound<'py, PyAny>,
    pub uv_slide: &'r Bound<'py, PyAny>,
    pub chord: &'r Bound<'py, PyAny>,
    pub reverse: bool,
    pub free: &'r Bound<'py, PyAny>,
}

fn read_silhouette<'a, 'py>(py: Python<'py>, classes: &Classes, args: &SilhouetteArgs<'_, 'py>, keys: &mut Keys<'a, 'py>) -> Read<SilInput> {
    let pool = &classes.pool;
    let mut polygons = Vec::new();
    for face in sequence(args.polygons, "polygons")? {
        let mut rings = Vec::new();
        for ring in sequence(&face, "the polygons of a face")? {
            let mut found = Vec::new();
            for key in sequence(&ring, "a ring")? {
                found.push(keys.key(py, &key)?);
            }
            rings.push(found);
        }
        polygons.push(rings);
    }
    let mut cycles = Vec::new();
    for cycle in sequence(args.cycles, "cycles")? {
        let mut found = Vec::new();
        for item in sequence(&cycle, "a cycle")? {
            let Ok(item) = item.cast_exact::<PyTuple>() else {
                return unsupported("a cycle item is not a tuple");
            };
            if item.is_empty() {
                return unsupported("a cycle item is empty");
            }
            found.push(keys.key(py, &item.get_item(0)?)?);
        }
        cycles.push(found);
    }
    let regions: Vec<u32> = sequence(args.regions, "regions")?.iter().map(|item| small_int(item, "a region")).collect::<Read<_>>()?;
    if regions.len() != polygons.len() || cycles.len() != polygons.len() {
        return unsupported("polygons, cycles and regions must have one length");
    }
    let mut cut_pairs = std::collections::HashSet::<(u32, u32), FxBuild>::default();
    for pair in sequence(args.cut_pairs, "cut_pairs")? {
        let items = sequence(&pair, "a cut pair")?;
        if items.len() != 2 {
            return unsupported("a cut pair is not two regions");
        }
        let (a, b) = (small_int(&items[0], "a cut region")?, small_int(&items[1], "a cut region")?);
        cut_pairs.insert((a.min(b), a.max(b)));
    }
    let mut position_rows = Vec::new();
    for (key, value) in exact_dict(args.positions, "positions")?.iter() {
        position_rows.push((keys.key(py, &key)?, position_of(classes, &value)?));
    }
    let mut point_rows = Vec::new();
    for (key, value) in exact_dict(args.points, "points")?.iter() {
        point_rows.push((keys.key(py, &key)?, pair_of(pool, &value, "a chart point")?));
    }
    let mut facts = HashMap::with_capacity_and_hasher(0, FxBuild::default());
    for (slot, value) in exact_dict(args.facts, "facts")?.iter() {
        let Ok(slot) = slot.cast_exact::<PyTuple>() else {
            return unsupported("a fact slot is not a tuple");
        };
        if slot.len() != 2 {
            return unsupported("a fact slot is not (region, key)");
        }
        let region = small_int(&slot.get_item(0)?, "a fact region")?;
        let key = keys.key(py, &slot.get_item(1)?)?;
        facts.insert((region, key), pair_of(pool, &value, "a fact")?);
    }
    let Ok(free_set) = args.free.cast_exact::<PyFrozenSet>() else {
        return unsupported("free is not a frozenset");
    };
    let mut free_keys = Vec::new();
    for key in free_set.iter() {
        free_keys.push(keys.key(py, &key)?);
    }
    let alpha = rat_of(pool, args.lattice_alpha, "lattice_alpha")?;
    if alpha.is_zero() {
        return unsupported("lattice_alpha is zero");
    }
    let inverse_alpha = Rat::one().div(&alpha).map_err(|_| Fail::Unsupported("lattice_alpha is zero".to_string()))?;
    let uv_slide = rat_of(pool, args.uv_slide, "uv_slide")?;
    let chord_budget = rat_of(pool, args.chord, "the chord budget")?;
    let count = keys.bytes.len();
    let mut positions = vec![None; count];
    for (key, value) in position_rows {
        positions[key as usize] = Some(value);
    }
    let mut points = vec![None; count];
    for (key, value) in point_rows {
        points[key as usize] = Some(value);
    }
    let mut free = vec![false; count];
    for key in &free_keys {
        free[*key as usize] = true;
    }
    Ok(SilInput {
        rank: keys.ranks(),
        place: keys.places(args.mark.as_bytes()),
        polygons,
        cycles,
        regions,
        cut_pairs,
        positions,
        points,
        facts,
        front: SqrtSum::rational(&alpha),
        inverse_alpha,
        uv_slide,
        chord_budget,
        reverse: args.reverse,
        free,
        free_any: !free_keys.is_empty(),
    })
}

fn key_tuple<'py>(py: Python<'py>, objects: &[Bound<'py, PyAny>], keys: &[Key]) -> PyResult<Bound<'py, PyTuple>> {
    PyTuple::new(py, keys.iter().map(|key| &objects[*key as usize]))
}

fn build_silhouette<'py>(py: Python<'py>, objects: &[Bound<'py, PyAny>], out: &SilOutput) -> PyResult<Bound<'py, PyAny>> {
    let mut faces = Vec::with_capacity(out.faces.len());
    for face in &out.faces {
        faces.push((face.index, key_tuple(py, objects, &face.ring)?, PyTuple::new(py, &face.others)?).into_bound_py_any(py)?);
    }
    let gone = key_tuple(py, objects, &out.gone)?;
    let counters = PyTuple::new(py, out.counters.iter().map(|(name, value)| (*name, *value)))?;
    let mut runs = Vec::with_capacity(out.runs.len());
    for (before, after, run) in &out.runs {
        runs.push((&objects[*before as usize], &objects[*after as usize], key_tuple(py, objects, run)?).into_bound_py_any(py)?);
    }
    let mut fits = Vec::with_capacity(out.fits.len());
    for (region, ring) in &out.fits {
        fits.push((*region, key_tuple(py, objects, ring)?).into_bound_py_any(py)?);
    }
    let chains = match &out.chains {
        None => py.None().into_bound(py),
        Some((boundary, interface)) => {
            let mut lists = Vec::with_capacity(2);
            for chains in [boundary, interface] {
                let mut items = Vec::with_capacity(chains.len());
                for (id, keys) in chains {
                    items.push((id.as_str(), key_tuple(py, objects, keys)?).into_bound_py_any(py)?);
                }
                lists.push(PyTuple::new(py, items)?);
            }
            PyTuple::new(py, lists)?.into_any()
        }
    };
    let problems = PyTuple::new(py, out.problems.iter().copied())?;
    (out.kind, PyTuple::new(py, faces)?, gone, counters, out.note.as_str(), out.changed, PyTuple::new(py, runs)?, PyTuple::new(py, fits)?, chains, problems).into_bound_py_any(py)
}

/// What the shim hands over for one `lift_source_vertices` + `settle_emitted_faces`.
pub struct SourceLiftArgs<'r, 'py> {
    pub positions: &'r Bound<'py, PyAny>,
    pub triangles: &'r Bound<'py, PyAny>,
    pub host_positions: &'r Bound<'py, PyAny>,
    pub step: &'r Bound<'py, PyAny>,
    pub mark: &'r str,
    pub polygons: &'r Bound<'py, PyAny>,
    pub points: &'r Bound<'py, PyAny>,
    pub reverse: bool,
}

type SourceRead<'py> = (LiftInput, Vec<Bound<'py, PyAny>>, Vec<Option<Bound<'py, PyAny>>>, Vec<Vec<Vec<usize>>>);

fn read_source_lift<'a, 'py>(py: Python<'py>, classes: &Classes, args: &SourceLiftArgs<'_, 'py>, keys: &mut Keys<'a, 'py>) -> Read<SourceRead<'py>> {
    let positions_dict = exact_dict(args.positions, "positions")?;
    let mut positions = Vec::with_capacity(positions_dict.len());
    for (key, value) in positions_dict.iter() {
        let index = keys.key(py, &key)?;
        if index as usize != positions.len() {
            return unsupported("a key twice in positions");
        }
        positions.push(position_of(classes, &value)?);
    }
    let count = positions.len();
    let known = |index: Key| -> Read<usize> {
        if (index as usize) < count {
            Ok(index as usize)
        } else {
            unsupported("a vertex of a face or a triangle has no lifted position (the oracle raises KeyError)")
        }
    };
    let mut triangles = Vec::new();
    for triangle in sequence(args.triangles, "triangles")? {
        let mut found = Vec::new();
        for key in sequence(&triangle, "a triangle")? {
            let index = keys.key(py, &key)?;
            found.push(known(index)?);
        }
        triangles.push(found);
    }
    let mut polygons = Vec::new();
    for face in sequence(args.polygons, "polygons")? {
        let mut rings = Vec::new();
        for ring in sequence(&face, "the polygons of a face")? {
            let mut found = Vec::new();
            for key in sequence(&ring, "a polygon")? {
                let index = keys.key(py, &key)?;
                found.push(known(index)?);
            }
            rings.push(found);
        }
        polygons.push(rings);
    }
    let hosts_dict = exact_dict(args.host_positions, "host_positions")?;
    let mark = args.mark.as_bytes();
    let mut source = vec![false; count];
    let mut node = vec![false; count];
    let mut host = vec![None; count];
    let mut host_objects: Vec<Option<Bound<'py, PyAny>>> = vec![None; count];
    for index in 0..count {
        let bytes = keys.bytes[index];
        node[index] = bytes.starts_with(b"node:");
        if !bytes.starts_with(b"src:") {
            continue;
        }
        source[index] = true;
        let name = &bytes[4..];
        let name = if !mark.is_empty() && name.ends_with(mark) { &name[..name.len() - mark.len()] } else { name };
        let Ok(text) = std::str::from_utf8(name) else {
            return unsupported("a source key is not UTF-8 after its prefix");
        };
        let Some(found) = hosts_dict.get_item(text)? else {
            continue;
        };
        if found.get_type().as_ptr() == classes.local_point.as_ptr() {
            host[index] = Some(position_of(classes, &found)?);
            host_objects[index] = Some(found);
        } else if found.is_instance(classes.local_point.bind(py))? {
            return unsupported("a host position of a subclass of LocalPoint3V1");
        }
    }
    let step = if args.step.is_none() { None } else { Some(rat_of(&classes.pool, args.step, "the source step")?) };
    let objects = keys.objects.clone();
    Ok((LiftInput { positions, source, node, host, triangles, step }, objects, host_objects, polygons))
}

/// `(final positions, lifted, moved, displaced, unavailable, kept, total, worst or None, max lifted, budget or None, followed, followed drivers, max follow)`.
fn build_lift<'py>(
    py: Python<'py>,
    classes: &Classes,
    input: &LiftInput,
    lift: &LiftOutput,
    objects: &[Bound<'py, PyAny>],
    hosts: &[Option<Bound<'py, PyAny>>],
    positions: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let positions = positions.cast_exact::<PyDict>().map_err(|_| PyRuntimeError::new_err("positions changed during the call"))?;
    let final_positions = PyDict::new(py);
    for (index, (key, value)) in positions.iter().enumerate() {
        let placed = match lift.placed[index] {
            Placed::Lift => value,
            Placed::Host => hosts[index].clone().ok_or_else(|| PyRuntimeError::new_err("a host placement without its host"))?,
            Placed::Followed(position) => match &classes.local_raw {
                Some(raw) => raw.build(py, &classes.local_point, [PyFloat::new(py, position[0]).into_any(), PyFloat::new(py, position[1]).into_any(), PyFloat::new(py, position[2]).into_any()])?,
                None => {
                    note_attr_build();
                    let built = alloc(py, &classes.local_point)?;
                    for (slot, coordinate) in [(&classes.x, position[0]), (&classes.y, position[1]), (&classes.z, position[2])] {
                        set_slot(&built, slot, PyFloat::new(py, coordinate).as_any())?;
                    }
                    built
                }
            },
        };
        final_positions.set_item(key, placed)?;
    }
    let _ = input;
    let worst = match lift.worst {
        None => py.None().into_bound(py),
        Some((key, distance)) => (&objects[key], distance).into_bound_py_any(py)?,
    };
    (
        final_positions,
        (lift.lifted, lift.moved, lift.displaced, lift.unavailable, lift.kept, lift.total),
        (worst, lift.max_lifted, lift.budget, lift.followed, lift.followed_drivers, lift.max_follow),
    )
        .into_bound_py_any(py)
}

pub(crate) fn unsupported_answer<'py>(py: Python<'py>, text: &str, given: [u64; 6]) -> PyResult<Answer<'py>> {
    Ok((None, STATUS_UNSUPPORTED, Some(PyTuple::new(py, [PyString::new(py, text)])?), [0; 5], given, 0, [0; 8]))
}

impl Host {
    /// One `apply_silhouette`: see the module note. `budget` is `(cap, six articles)` of the budget the pass runs on.
    pub fn silhouette<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        args: &SilhouetteArgs<'_, 'py>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
    ) -> PyResult<Answer<'py>> {
        let given = budget.map_or([0; 6], |(_, articles)| articles);
        let started = Instant::now();
        let classes = self.classes()?;
        let mut keys = Keys::new();
        let input = match read_silhouette(py, classes, args, &mut keys) {
            Ok(input) => input,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return unsupported_answer(py, &text, given),
        };
        let arguments_ns = nanos(started);
        let sync_value = match sync {
            Some(bytes) => Some(Reader::new(bytes, true).get_value().map_err(|error| PyValueError::new_err(format!("the memory sync is not in the wire format: {error}")))?),
            None => None,
        };
        let (cap, articles) = match budget {
            Some((cap, articles)) => (cap, Some(articles)),
            None => (None, None),
        };
        let mut run = CostRun::begin_parts(session, sync_value.as_ref(), cap, articles).map_err(|error| PyValueError::new_err(error.to_string()))?;
        let computing = Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut counts = SignCounts::default();
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            let result = apply_silhouette(&mut ctx, &input);
            let articles = run.budget_mut().articles();
            (result, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (result, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.silhouette: native panic: {}", crate::panic_message(&panic)))),
        };
        let out = match result {
            Ok(out) => out,
            Err(SilError::Exhausted) => return unsupported_answer(py, "an exhaustion outside the pass", given),
            Err(SilError::HalfEdgeShared(..)) => return unsupported_answer(py, "HALF_EDGE_SHARED_IN_ONE_DIRECTION: the oracle names it", given),
            Err(SilError::Unsupported(text)) => return unsupported_answer(py, &text, given),
        };
        let building = Instant::now();
        let built = build_silhouette(py, &keys.objects, &out)?;
        let building_ns = nanos(building);
        let applying = Instant::now();
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);
        Ok((Some(built), 0, None, counts.as_array(), articles, changed, [0, arguments_ns, compute_ns, building_ns, log_ns, 0, 0, 0]))
    }

    /// `lift_source_vertices(positions, triangles, host_positions, step)` and then `settle_emitted_faces(polygons, positions, sourced, points, budget, reverse)`: the answer of
    /// `silhouette` with the result `(lift, faces)`. `lift` is `(final positions, lifted, moved, displaced, unavailable, kept, total, worst or None, max lifted, budget or None, followed,
    /// followed drivers, max follow)`; `faces` is `None` when no position changed (the oracle returns `polygons` itself), else `(settled, max off plane, triangulated, flipped)`. An exact
    /// outcome of the oracle in `settle_emitted_faces` (exhaustion, ...) is its status with the effects applied, and `lift` is still answered (the shim rebinds the normals first, as the
    /// oracle does, then raises).
    #[allow(clippy::too_many_arguments)]
    pub fn source_lift<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        args: &SourceLiftArgs<'_, 'py>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
    ) -> PyResult<Answer<'py>> {
        let given = budget.map_or([0; 6], |(_, articles)| articles);
        let started = Instant::now();
        let classes = self.classes()?;
        let mut keys = Keys::new();
        let read = read_source_lift(py, classes, args, &mut keys);
        let (input, objects, hosts, polygons) = match read {
            Ok(found) => found,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return unsupported_answer(py, &text, given),
        };
        let Some(lift) = catch_unwind(AssertUnwindSafe(|| lift_source_vertices(&input))).map_err(|panic| PyRuntimeError::new_err(format!("Session.source_lift: native panic: {}", crate::panic_message(&panic))))? else {
            return unsupported_answer(py, "lift_source_vertices: a value the oracle would raise on (a followed position that is not finite, a distance beyond a float)", given);
        };
        let after: Vec<[f64; 3]> = (0..input.positions.len())
            .map(|key| match lift.placed[key] {
                Placed::Lift => input.positions[key],
                Placed::Host => input.host[key].expect("a placed host"),
                Placed::Followed(position) => position,
            })
            .collect();
        // the chart points the settle may read: the vertices of the faces of four or more with a moved vertex
        let moved: Vec<bool> = input.positions.iter().zip(&after).map(|(old, new)| old != new).collect();
        let mut chart: Vec<Option<(SqrtSum, SqrtSum)>> = vec![None; input.positions.len()];
        let points = match exact_dict(args.points, "points") {
            Ok(points) => points,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return unsupported_answer(py, &text, given),
        };
        for face in &polygons {
            for polygon in face {
                if polygon.len() < 4 || !polygon.iter().any(|key| moved[*key]) {
                    continue;
                }
                for &key in polygon {
                    if chart[key].is_some() {
                        continue;
                    }
                    if let Some(value) = points.get_item(&objects[key])? {
                        match pair_of(&classes.pool, &value, "a chart point") {
                            Ok(pair) => chart[key] = Some(pair),
                            Err(Fail::Py(error)) => return Err(error),
                            Err(Fail::Unsupported(text)) => return unsupported_answer(py, &text, given),
                        }
                    }
                }
            }
        }
        let arguments_ns = nanos(started);
        let sync_value = match sync {
            Some(bytes) => Some(Reader::new(bytes, true).get_value().map_err(|error| PyValueError::new_err(format!("the memory sync is not in the wire format: {error}")))?),
            None => None,
        };
        let (cap, articles) = match budget {
            Some((cap, articles)) => (cap, Some(articles)),
            None => (None, None),
        };
        let mut run = CostRun::begin_parts(session, sync_value.as_ref(), cap, articles).map_err(|error| PyValueError::new_err(error.to_string()))?;
        let computing = Instant::now();
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut counts = SignCounts::default();
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            let result = settle_emitted_faces(&mut ctx, &polygons, &input.positions, &after, &chart, args.reverse);
            let articles = run.budget_mut().articles();
            (result, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (result, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.source_lift: native panic: {}", crate::panic_message(&panic)))),
        };
        let (faces, status, detail) = match result {
            Ok(faces) => (Some(faces), 0u8, None),
            Err(None) => return unsupported_answer(py, "settle_emitted_faces: a vertex of a face without its chart point (the oracle raises KeyError)", given),
            Err(Some(ClipError::Exact(error))) => {
                let (status, detail) = exact_status(py, &classes.pool, &error)?;
                if is_native_only(status) {
                    return unsupported_answer(py, &format!("settle_emitted_faces: the exact layer refused ({error:?})"), given);
                }
                (None, status, detail)
            }
            Err(Some(other)) => return unsupported_answer(py, &format!("settle_emitted_faces: {other:?}"), given),
        };
        let building = Instant::now();
        let lift_tuple = build_lift(py, classes, &input, &lift, &objects, &hosts, args.positions)?;
        let faces_tuple = match &faces {
            None => py.None().into_bound(py),
            Some(found) if !found.changed_any => py.None().into_bound(py),
            Some(found) => {
                let polygon_objects = sequence(args.polygons, "polygons").map_err(|_| PyRuntimeError::new_err("polygons changed during the call"))?;
                let mut settled = Vec::with_capacity(found.settled.len());
                for (face, fates) in polygon_objects.iter().zip(&found.settled) {
                    let rings = sequence(face, "a face").map_err(|_| PyRuntimeError::new_err("polygons changed during the call"))?;
                    let mut kept: Vec<Bound<'py, PyAny>> = Vec::with_capacity(rings.len());
                    for (ring, fate) in rings.iter().zip(fates) {
                        match fate {
                            Settled::Whole => kept.push(ring.clone()),
                            Settled::Ears(ears) => {
                                let items = sequence(ring, "a polygon").map_err(|_| PyRuntimeError::new_err("polygons changed during the call"))?;
                                for ear in ears {
                                    kept.push(PyTuple::new(py, [&items[ear[0]], &items[ear[1]], &items[ear[2]]])?.into_any());
                                }
                            }
                        }
                    }
                    settled.push(PyTuple::new(py, kept)?);
                }
                (PyList::new(py, settled)?, found.max_off_plane, found.triangulated, found.flipped).into_bound_py_any(py)?
            }
        };
        let built = (lift_tuple, faces_tuple).into_bound_py_any(py)?;
        let building_ns = nanos(building);
        let applying = Instant::now();
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);
        Ok((Some(built), status, detail, counts.as_array(), articles, changed, [0, arguments_ns, compute_ns, building_ns, log_ns, 0, 0, 0]))
    }

    /// `verify_silhouette(source, result)` of the Rust law on a result in the native form (`(kind, faces, gone, counters, note, changed, runs, fits, chains)` as
    /// `silhouette` answers it): the names of the violations, or `None` when the input is not carried. A check of the check: the tests compare it with the oracle's
    /// `verify_silhouette` on corrupted results.
    pub fn verify<'py>(&mut self, py: Python<'py>, args: &SilhouetteArgs<'_, 'py>, result: &Bound<'py, PyAny>) -> PyResult<Option<Vec<&'static str>>> {
        let classes = self.classes()?;
        let mut keys = Keys::new();
        let read = (|| -> Read<(SilInput, SilOutput)> {
            let input = read_silhouette(py, classes, args, &mut keys)?;
            let items = sequence(result, "the result")?;
            if items.len() != 9 {
                return unsupported("the result is not nine items");
            }
            let mut faces = Vec::new();
            for face in sequence(&items[1], "faces")? {
                let parts = sequence(&face, "a face")?;
                if parts.len() != 3 {
                    return unsupported("a face is not (index, ring, others)");
                }
                let mut ring = Vec::new();
                for key in sequence(&parts[1], "a ring")? {
                    ring.push(keys.key(py, &key)?);
                }
                let others = sequence(&parts[2], "others")?.iter().map(|item| small_int(item, "a merged face")).collect::<Read<Vec<u32>>>()?;
                faces.push(cftuv_silhouette::silhouette::FinalFace { index: small_int(&parts[0], "a face index")?, ring, others });
            }
            let mut gone = Vec::new();
            for key in sequence(&items[2], "gone")? {
                gone.push(keys.key(py, &key)?);
            }
            let mut counters = Vec::new();
            for pair in sequence(&items[3], "counters")? {
                let parts = sequence(&pair, "a counter")?;
                let name: String = parts[0].extract()?;
                let Some(known) = cftuv_silhouette::silhouette::COUNTER_NAMES.iter().find(|item| **item == name) else {
                    return unsupported(format!("a counter the law does not write: {name}"));
                };
                let value: u64 = parts[1].extract()?;
                counters.push((*known, value));
            }
            let mut runs = Vec::new();
            for run in sequence(&items[6], "runs")? {
                let parts = sequence(&run, "a run")?;
                let mut inner = Vec::new();
                for key in sequence(&parts[2], "a run")? {
                    inner.push(keys.key(py, &key)?);
                }
                runs.push((keys.key(py, &parts[0])?, keys.key(py, &parts[1])?, inner));
            }
            let mut fits = Vec::new();
            for fit in sequence(&items[7], "fits")? {
                let parts = sequence(&fit, "a fit")?;
                let mut ring = Vec::new();
                for key in sequence(&parts[1], "a fit ring")? {
                    ring.push(keys.key(py, &key)?);
                }
                fits.push((small_int(&parts[0], "a fit region")?, ring));
            }
            let chains = if items[8].is_none() {
                None
            } else {
                let mut lists = Vec::new();
                for side in sequence(&items[8], "chains")? {
                    let mut chains = Vec::new();
                    for chain in sequence(&side, "a side of the chains")? {
                        let parts = sequence(&chain, "a chain")?;
                        let mut path = Vec::new();
                        for key in sequence(&parts[1], "a chain path")? {
                            path.push(keys.key(py, &key)?);
                        }
                        chains.push((parts[0].extract::<String>()?, path));
                    }
                    lists.push(chains);
                }
                if lists.len() != 2 {
                    return unsupported("chains is not (boundary, interface)");
                }
                let interface = lists.pop().unwrap_or_default();
                let boundary = lists.pop().unwrap_or_default();
                Some((boundary, interface))
            };
            // the keys first seen in the result: the input tables of the law grow to the same length
            let mut input = input;
            let count = keys.bytes.len();
            input.rank = keys.ranks();
            input.place = keys.places(args.mark.as_bytes());
            input.positions.resize(count, None);
            input.points.resize(count, None);
            input.free.resize(count, false);
            let output = SilOutput {
                kind: small_int(&items[0], "the kind")? as u8,
                faces,
                gone,
                counters,
                note: items[4].extract()?,
                changed: items[5].extract()?,
                runs,
                fits,
                chains,
                problems: Vec::new(),
            };
            Ok((input, output))
        })();
        let (input, output) = match read {
            Ok(found) => found,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(_)) => return Ok(None),
        };
        match catch_unwind(AssertUnwindSafe(|| cftuv_silhouette::silhouette::verify_output(&input, &output))) {
            Ok(Ok(problems)) => Ok(Some(problems)),
            Ok(Err(_)) => Ok(None),
            Err(panic) => Err(PyRuntimeError::new_err(format!("Session.hot_verify: native panic: {}", crate::panic_message(&panic)))),
        }
    }

    /// `{key: PlaneLiftV1(origin, basis_a, basis_b, scale).lift(point)}` over a dict of points, in its order; `None` when the port does not carry it.
    #[allow(clippy::too_many_arguments)]
    pub fn plane_lift<'py>(&self, py: Python<'py>, points: &Bound<'py, PyAny>, origin: &Bound<'py, PyAny>, basis_a: &Bound<'py, PyAny>, basis_b: &Bound<'py, PyAny>, scale: &Bound<'py, PyAny>) -> PyResult<Option<Bound<'py, PyDict>>> {
        let classes = self.classes()?;
        let pool = &classes.pool;
        let read = (|| -> Read<Plane> {
            let triple = |object: &Bound<'py, PyAny>, what: &str| -> Read<[Rat; 3]> {
                let items = sequence(object, what)?;
                if items.len() != 3 {
                    return unsupported(format!("{what} is not three numbers"));
                }
                Ok([rat_of(pool, &items[0], what)?, rat_of(pool, &items[1], what)?, rat_of(pool, &items[2], what)?])
            };
            if !scale.is_exact_instance_of::<PyInt>() {
                return unsupported("the scale is not an int");
            }
            let scale = Rat::from_int(ibig_from_int(scale)?);
            match Plane::new(triple(origin, "origin")?, triple(basis_a, "basis_a")?, triple(basis_b, "basis_b")?, &scale) {
                Some(plane) => Ok(plane),
                None => unsupported("a zero scale"),
            }
        })();
        let plane = match read {
            Ok(plane) => plane,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(_)) => return Ok(None),
        };
        let Ok(points) = points.cast_exact::<PyDict>() else {
            return Ok(None);
        };
        let found = PyDict::new(py);
        for (key, value) in points.iter() {
            let Ok((x, y)) = pair_of(pool, &value, "a chart point") else {
                return Ok(None);
            };
            let Some(lifted) = catch_unwind(AssertUnwindSafe(|| plane.lift(&x, &y))).map_err(|panic| PyRuntimeError::new_err(format!("Session.plane_lift: native panic: {}", crate::panic_message(&panic))))? else {
                return Ok(None);
            };
            let position = match &classes.local_raw {
                Some(raw) => raw.build(py, &classes.local_point, [PyFloat::new(py, lifted[0]).into_any(), PyFloat::new(py, lifted[1]).into_any(), PyFloat::new(py, lifted[2]).into_any()])?,
                None => {
                    note_attr_build();
                    let position = alloc(py, &classes.local_point)?;
                    for (slot, value) in [(&classes.x, lifted[0]), (&classes.y, lifted[1]), (&classes.z, lifted[2])] {
                        set_slot(&position, slot, PyFloat::new(py, value).as_any())?;
                    }
                    position
                }
            };
            found.set_item(key, position)?;
        }
        Ok(Some(found))
    }
}
