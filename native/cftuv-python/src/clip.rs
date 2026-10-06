//! `clip.clip_geometry` behind the persistent session: the production-shaped boundary of the clip port.
//!
//! What crosses per call, and what does not:
//!
//! * the PLANE (`plane.triangles`, a tuple of frozen `LiftTriangleV1`) is converted ONCE and kept by the identity of that
//!   tuple together with a strong reference to it (the same discipline as the coverage partitions); the least recently
//!   used plane goes out at [`PLANE_LIMIT`]. The triangle names stay as the Python strings the oracle returns.
//! * the call's own arguments (`points`, the keys of `cycles`, `polygons`, `seam`, `fans`, `flows`) are read from the
//!   Python containers directly, no intermediate buffer. A point's `SqrtSumV1` pair is converted per call.
//! * the RESULT is built from Rust: `ClippedV1`, `LocalPoint3V1`, `SqrtSumV1`, `Fraction`, the containers. Where the oracle
//!   returns an object it was given (a point that is a node representative straight from the input `points`, a key string,
//!   a triangle name) the same object is returned; a point that appears in several places of the result is ONE object, as
//!   in the oracle (every list holds `node.point`).
//! * the normal writes of `lift_known` go straight into `plane._normal_by_position`, in call order, also when the
//!   operation fails afterwards (the writes before the failure stay in the oracle).
//!
//! * the memory log of the call (what `prime_support` and `squarefree_split` wrote into the canonicalization tables) is replayed IN PLACE on
//!   the process's real tables, in the order Python writes them: no log crosses the boundary.
//! * the session keeps a cross-call cache of exact results (`cftuv_clip::warm`: crossings, line values, lifts) that depend on their operands alone;
//!   what a hit would have cost is paid again against the current memory and budget, so it changes no answer and no cost.
//!
//! The answer is `(result or None, status, detail, counts, articles, changed tables, timings)`. `status` 0 is ok; the others are
//! the outcome codes of `cost.OpResult` (1..7) and of the clip seams (8 `OverflowError`, 9 `ZeroDivisionError`, 10 `ValueError`,
//! 11 `MaterializationRefusal`, 12 unsupported by the port, 13 `KeyError`); the shim applies the budget, the counters and the snapshot of the
//! tables first and raises the oracle's exception last. `timings` are nanoseconds `(plane, arguments, compute, result, memory log)`.
//!
//! The GIL stays held: the operation is one thread and the product parallelizes by process.

use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::rc::Rc;
use std::sync::Arc;
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyFloat, PyList, PyString, PyTuple};

use cftuv_clip::emit::{Clipped, Law};
use cftuv_clip::fxhash::FxBuild;
use cftuv_clip::error::ClipError;
use cftuv_clip::geometry::{clip_geometry, ClipInput, ClipRun};
use cftuv_clip::plane::{ChartPoint, Plane, Triangle};
use cftuv_clip::point::Point;
use cftuv_clip::pyemu::PyVersion;
use cftuv_clip::stage::NormalWrite;
use cftuv_clip::warm::Warm;
use cftuv_core::codec::Reader;
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::session::{CostRun, Session};
use cftuv_core::sqrt_sum::SignCounts;

use crate::coverage::exact_status;
use crate::memlog::{apply_log, insort, Tables};
use crate::pyobj::{alloc, int_from_ibig, int_from_ubig, rat_from_number, refuse, set_slot, sqrt_sum_from_py, sqrt_sum_to_py, Pool, Raw};

/// Planes kept converted per session.
pub const PLANE_LIMIT: usize = 16;

const STATUS_OVERFLOW: u8 = 8;
const STATUS_ZERO_DIVISION: u8 = 9;
const STATUS_VALUE: u8 = 10;
const STATUS_REFUSAL: u8 = 11;
const STATUS_UNSUPPORTED: u8 = 12;
const STATUS_MISSING_KEY: u8 = 13;

/// `(result, status, detail, counts, articles, changed tables, timings)`.
pub type Answer<'py> = (Option<Bound<'py, PyAny>>, u8, Option<Bound<'py, PyTuple>>, [u64; 5], [u64; 6], u8, [u64; 5]);

struct Names {
    polygons: Py<PyString>,
    cycles: Py<PyString>,
    vertex_lists: Py<PyString>,
    extra_lists: Py<PyString>,
    points: Py<PyString>,
    snapped: Py<PyString>,
    lifted: Py<PyString>,
    counters: Py<PyString>,
    note: Py<PyString>,
    memo: Py<PyString>,
    x: Py<PyString>,
    y: Py<PyString>,
    z: Py<PyString>,
    name: Py<PyString>,
    chart: Py<PyString>,
    corners: Py<PyString>,
    twice_area: Py<PyString>,
    bbox: Py<PyString>,
    normals: Py<PyString>,
    face: Py<PyString>,
}

/// What `bind_clip` hands over: the kernel classes the result is built from.
struct Classes {
    pool: Pool,
    clipped: Py<PyAny>,
    local_point: Py<PyAny>,
    /// Where the slots x, y, z of a `LocalPoint3V1` sit (see [`Raw`]).
    local_raw: Option<Raw>,
    empty_str: Py<PyAny>,
    /// `bisect.insort`: the registry list is sorted by the comparison of Python itself.
    insort: Py<PyAny>,
    names: Names,
}

/// A converted plane: the native triangles, the strong reference that keeps its identity valid, the names as Python strings.
struct PreparedPlane {
    id: usize,
    stamp: u64,
    _holder: Py<PyAny>,
    plane: Plane,
    names: HashMap<String, Py<PyString>>,
}

pub struct Host {
    classes: Option<Classes>,
    planes: Vec<PreparedPlane>,
    clock: u64,
    /// The cross-call cache (`cftuv_clip::warm`): exact results kept for the next call of the session.
    warm: Warm,
}

impl Default for Host {
    fn default() -> Host {
        Host { classes: None, planes: Vec::new(), clock: 0, warm: Warm::new() }
    }
}

fn nanos(since: Instant) -> u64 {
    u64::try_from(since.elapsed().as_nanos()).unwrap_or(u64::MAX)
}

fn panic_text(panic: &Box<dyn std::any::Any + Send>) -> String {
    if let Some(text) = panic.downcast_ref::<&str>() {
        (*text).to_string()
    } else if let Some(text) = panic.downcast_ref::<String>() {
        text.clone()
    } else {
        "non-text panic payload".to_string()
    }
}

// --------------------------------------------------------------------------
// the arguments of one call
// --------------------------------------------------------------------------

/// The call's arguments in native form plus the Python objects the result reuses.
struct Arguments<'py> {
    points: Vec<(String, Point)>,
    /// The input tuples, by position in `points`.
    point_objects: Vec<Bound<'py, PyAny>>,
    /// The input key strings, by content.
    key_objects: HashMap<String, Bound<'py, PyAny>, FxBuild>,
    cycles: Vec<Vec<String>>,
    polygons: Vec<Vec<Vec<String>>>,
    seam: Vec<(String, String)>,
    fans: Option<Vec<bool>>,
    flows: Option<Vec<bool>>,
    /// The chain station plan's pairs of faces, in the iteration order of the caller's frozenset (pairs that are not two names are dropped: the oracle skips them).
    inert: Vec<(String, String)>,
}

fn text_of(item: &Bound<'_, PyAny>, what: &str) -> PyResult<String> {
    match item.cast::<PyString>() {
        Ok(text) => Ok(text.to_str()?.to_string()),
        Err(_) => Err(refuse(format!("{what} must be a str, not {}", item.get_type().name()?))),
    }
}

fn keys_of(sequence: &Bound<'_, PyAny>, what: &str) -> PyResult<Vec<String>> {
    sequence.try_iter()?.map(|item| text_of(&item?, what)).collect()
}

fn flags_of(value: &Bound<'_, PyAny>) -> PyResult<Option<Vec<bool>>> {
    if value.is_none() {
        return Ok(None);
    }
    let mut flags = Vec::new();
    for item in value.try_iter()? {
        flags.push(item?.is_truthy()?);
    }
    Ok(Some(flags))
}

/// `inert`: an iterable of pairs of face names (`frozenset[frozenset[str]]`). A pair that is not two items is skipped, as `_plan_groups` skips it;
/// an item that is no `str` is a named refusal (the oracle would compare it to the face names and sort it with a name).
fn inert_of(value: &Bound<'_, PyAny>) -> PyResult<Vec<(String, String)>> {
    // `None` is as falsy as the empty frozenset: the oracle's `if inert` takes the same branch
    if value.is_none() {
        return Ok(Vec::new());
    }
    let mut pairs = Vec::new();
    for pair in value.try_iter()? {
        let mut names = Vec::with_capacity(2);
        for item in pair?.try_iter()? {
            names.push(text_of(&item?, "an inert face name")?);
        }
        if names.len() == 2 {
            let second = names.pop().expect("two names");
            let first = names.pop().expect("two names");
            pairs.push((first, second));
        }
    }
    Ok(pairs)
}

fn arguments_of<'py>(
    pool: &Pool,
    points: &Bound<'py, PyDict>,
    cycles: &Bound<'py, PyAny>,
    polygons: &Bound<'py, PyAny>,
    seam: &Bound<'py, PyAny>,
    fans: &Bound<'py, PyAny>,
    flows: &Bound<'py, PyAny>,
    inert: &Bound<'py, PyAny>,
) -> PyResult<Arguments<'py>> {
    let mut named = Vec::with_capacity(points.len());
    let mut objects = Vec::with_capacity(points.len());
    let mut key_objects: HashMap<String, Bound<'py, PyAny>, FxBuild> = HashMap::with_capacity_and_hasher(points.len(), FxBuild::default());
    for (key, value) in points.iter() {
        let text = text_of(&key, "a point key")?;
        let pair = value.cast::<PyTuple>().map_err(|_| refuse("a point must be a tuple of two SqrtSumV1"))?;
        if pair.len() != 2 {
            return Err(refuse("a point must be a tuple of two SqrtSumV1"));
        }
        named.push((text.clone(), (sqrt_sum_from_py(pool, &pair.get_item(0)?)?, sqrt_sum_from_py(pool, &pair.get_item(1)?)?)));
        objects.push(value.clone());
        key_objects.insert(text, key);
    }
    let mut cycle_keys = Vec::new();
    for cycle in cycles.try_iter()? {
        let mut keys = Vec::new();
        for entry in cycle?.try_iter()? {
            keys.push(text_of(&entry?.get_item(0)?, "a cycle key")?);
        }
        cycle_keys.push(keys);
    }
    let mut polygon_keys = Vec::new();
    for face in polygons.try_iter()? {
        let mut loops = Vec::new();
        for keys in face?.try_iter()? {
            loops.push(keys_of(&keys?, "a polygon key")?);
        }
        polygon_keys.push(loops);
    }
    let mut pairs = Vec::new();
    for pair in seam.try_iter()? {
        let keys = keys_of(&pair?, "a seam key")?;
        match (keys.first(), keys.last()) {
            (Some(first), Some(last)) => pairs.push((first.clone(), last.clone())),
            _ => return Err(refuse("a seam pair without keys")),
        }
    }
    Ok(Arguments {
        points: named,
        point_objects: objects,
        key_objects,
        cycles: cycle_keys,
        polygons: polygon_keys,
        seam: pairs,
        fans: flags_of(fans)?,
        flows: flags_of(flows)?,
        inert: inert_of(inert)?,
    })
}

// --------------------------------------------------------------------------
// converting a plane once
// --------------------------------------------------------------------------

fn float_of(item: &Bound<'_, PyAny>, what: &str) -> PyResult<f64> {
    match item.cast_exact::<PyFloat>() {
        Ok(number) => Ok(number.value()),
        Err(_) => Err(refuse(format!("{what} must be a float, not {}", item.get_type().name()?))),
    }
}

fn tuple_of<'py>(item: &Bound<'py, PyAny>, size: usize, what: &str) -> PyResult<Bound<'py, PyTuple>> {
    let tuple = item.cast::<PyTuple>().map_err(|_| refuse(format!("{what} must be a tuple")))?;
    if tuple.len() != size {
        return Err(refuse(format!("{what} must have {size} items, not {}", tuple.len())));
    }
    Ok(tuple.clone())
}

fn chart_point_of(pool: &Pool, item: &Bound<'_, PyAny>) -> PyResult<ChartPoint> {
    let pair = tuple_of(item, 2, "a chart point")?;
    Ok((rat_from_number(pool, &pair.get_item(0)?)?, rat_from_number(pool, &pair.get_item(1)?)?))
}

fn triangle_of(py: Python<'_>, classes: &Classes, item: &Bound<'_, PyAny>) -> PyResult<(Triangle, Py<PyString>)> {
    let pool = &classes.pool;
    let names = &classes.names;
    let name = item.getattr(names.name.bind(py))?;
    let name_text = text_of(&name, "a triangle name")?;
    let chart = tuple_of(&item.getattr(names.chart.bind(py))?, 3, "a chart")?;
    let chart: [ChartPoint; 3] = [chart_point_of(pool, &chart.get_item(0)?)?, chart_point_of(pool, &chart.get_item(1)?)?, chart_point_of(pool, &chart.get_item(2)?)?];
    let corners = tuple_of(&item.getattr(names.corners.bind(py))?, 3, "the corners")?;
    let corner = |index: usize| -> PyResult<[Rat; 3]> {
        let axes = tuple_of(&corners.get_item(index)?, 3, "a corner")?;
        Ok([rat_from_number(pool, &axes.get_item(0)?)?, rat_from_number(pool, &axes.get_item(1)?)?, rat_from_number(pool, &axes.get_item(2)?)?])
    };
    let corners = [corner(0)?, corner(1)?, corner(2)?];
    let twice_area = rat_from_number(pool, &item.getattr(names.twice_area.bind(py))?)?;
    let bbox = tuple_of(&item.getattr(names.bbox.bind(py))?, 4, "a box")?;
    let bbox = [float_of(&bbox.get_item(0)?, "a box edge")?, float_of(&bbox.get_item(1)?, "a box edge")?, float_of(&bbox.get_item(2)?, "a box edge")?, float_of(&bbox.get_item(3)?, "a box edge")?];
    let normals = item.getattr(names.normals.bind(py))?;
    let normals = if normals.is_truthy()? {
        let rows = tuple_of(&normals, 3, "the offset normals")?;
        let row = |index: usize| -> PyResult<[f64; 3]> {
            let axes = tuple_of(&rows.get_item(index)?, 3, "an offset normal")?;
            Ok([float_of(&axes.get_item(0)?, "a normal axis")?, float_of(&axes.get_item(1)?, "a normal axis")?, float_of(&axes.get_item(2)?, "a normal axis")?])
        };
        Some([row(0)?, row(1)?, row(2)?])
    } else {
        None
    };
    let face = text_of(&item.getattr(names.face.bind(py))?, "a triangle face")?;
    let name_object = name.cast::<PyString>()?.clone().unbind();
    Ok((Triangle { name: name_text, chart, corners, twice_area, bbox, normals, face }, name_object))
}

fn convert_plane(py: Python<'_>, classes: &Classes, triangles: &Bound<'_, PyAny>, id: usize, stamp: u64) -> PyResult<PreparedPlane> {
    let tuple = triangles.cast::<PyTuple>().map_err(|_| refuse("plane.triangles must be a tuple of LiftTriangleV1"))?;
    let mut native = Vec::with_capacity(tuple.len());
    let mut names = HashMap::with_capacity(tuple.len());
    for item in tuple.iter() {
        let (triangle, name) = triangle_of(py, classes, &item)?;
        names.insert(triangle.name.clone(), name);
        native.push(triangle);
    }
    Ok(PreparedPlane { id, stamp, _holder: triangles.clone().unbind(), plane: Plane::new(native), names })
}

// --------------------------------------------------------------------------
// building the result
// --------------------------------------------------------------------------

/// The Python objects of one result: strings and points are made once and shared wherever the oracle shares them.
struct Builder<'py, 'a> {
    py: Python<'py>,
    classes: &'a Classes,
    keys: HashMap<String, Bound<'py, PyAny>, FxBuild>,
    /// `Rc` address of a node's point -> the Python tuple (an input object or one built here).
    points: HashMap<usize, Bound<'py, PyAny>, FxBuild>,
    inputs: &'a [Bound<'py, PyAny>],
}

impl<'py, 'a> Builder<'py, 'a> {
    fn key(&mut self, key: &str) -> Bound<'py, PyAny> {
        if let Some(found) = self.keys.get(key) {
            return found.clone();
        }
        let made = PyString::new(self.py, key).into_any();
        self.keys.insert(key.to_string(), made.clone());
        made
    }

    fn sum_pair(&self, point: &Point) -> PyResult<Bound<'py, PyAny>> {
        let pool = &self.classes.pool;
        Ok(PyTuple::new(self.py, [sqrt_sum_to_py(self.py, pool, &point.0)?, sqrt_sum_to_py(self.py, pool, &point.1)?])?.into_any())
    }

    fn point(&mut self, point: &Arc<Point>) -> PyResult<Bound<'py, PyAny>> {
        let address = Arc::as_ptr(point) as usize;
        if let Some(found) = self.points.get(&address) {
            return Ok(found.clone());
        }
        let made = self.sum_pair(point)?;
        self.points.insert(address, made.clone());
        Ok(made)
    }

    fn keyed(&mut self, entries: &[(Rc<str>, Arc<Point>)]) -> PyResult<Bound<'py, PyAny>> {
        let mut items = Vec::with_capacity(entries.len());
        for (key, point) in entries {
            items.push(PyTuple::new(self.py, [self.key(key), self.point(point)?])?);
        }
        Ok(PyList::new(self.py, items)?.into_any())
    }

    fn keyed_lists(&mut self, lists: &[Vec<(Rc<str>, Arc<Point>)>]) -> PyResult<Bound<'py, PyAny>> {
        let mut out = Vec::with_capacity(lists.len());
        for entries in lists {
            out.push(self.keyed(entries)?);
        }
        Ok(PyList::new(self.py, out)?.into_any())
    }

    fn polygons(&mut self, polygons: &[Vec<Vec<Rc<str>>>]) -> PyResult<Bound<'py, PyAny>> {
        let mut faces = Vec::with_capacity(polygons.len());
        for face in polygons {
            let mut loops = Vec::with_capacity(face.len());
            for keys in face {
                let items: Vec<Bound<'py, PyAny>> = keys.iter().map(|key| self.key(key)).collect();
                loops.push(PyTuple::new(self.py, items)?);
            }
            faces.push(PyTuple::new(self.py, loops)?);
        }
        Ok(PyList::new(self.py, faces)?.into_any())
    }

    fn lifted(&mut self, clipped: &Clipped, names: &HashMap<String, Py<PyString>>, normals: &[Bound<'py, PyAny>]) -> PyResult<Bound<'py, PyAny>> {
        let classes = self.classes;
        let slots = &classes.names;
        let dict = PyDict::new(self.py);
        let mut next_normal = 0;
        for (key, lifted) in &clipped.lifted {
            let position = match &classes.local_raw {
                Some(raw) => raw.build(
                    self.py,
                    &classes.local_point,
                    [PyFloat::new(self.py, lifted.position[0]).into_any(), PyFloat::new(self.py, lifted.position[1]).into_any(), PyFloat::new(self.py, lifted.position[2]).into_any()],
                )?,
                None => {
                    let position = alloc(self.py, &classes.local_point)?;
                    for (slot, value) in [(&slots.x, lifted.position[0]), (&slots.y, lifted.position[1]), (&slots.z, lifted.position[2])] {
                        set_slot(&position, slot, PyFloat::new(self.py, value).as_any())?;
                    }
                    position
                }
            };
            let normal = match &lifted.normal {
                Some(_) => {
                    let shared = normals.get(next_normal).ok_or_else(|| refuse("a lifted normal without its write"))?.clone();
                    next_normal += 1;
                    shared
                }
                None => self.py.None().into_bound(self.py),
            };
            let name = match names.get(&lifted.triangle) {
                Some(found) => found.bind(self.py).clone().into_any(),
                None => PyString::new(self.py, &lifted.triangle).into_any(),
            };
            dict.set_item(self.key(key), PyTuple::new(self.py, [position, PyTuple::new(self.py, [name, normal])?.into_any()])?)?;
        }
        Ok(dict.into_any())
    }

    fn counters(&self, clipped: &Clipped) -> PyResult<Bound<'py, PyAny>> {
        let pool = &self.classes.pool;
        let mut items = Vec::with_capacity(clipped.counters.len());
        for (name, value) in &clipped.counters {
            items.push(PyTuple::new(self.py, [PyString::intern(self.py, name).into_any(), int_from_ubig(self.py, pool, value)?])?);
        }
        Ok(PyTuple::new(self.py, items)?.into_any())
    }

    fn build(&mut self, clipped: &Clipped, names: &HashMap<String, Py<PyString>>, normals: &[Bound<'py, PyAny>]) -> PyResult<Bound<'py, PyAny>> {
        let classes = self.classes;
        let slots = &classes.names;
        let py = self.py;
        // the nodes that ARE input objects: their points are the input tuples
        for (point, index) in &clipped.origins {
            if let Some(object) = self.inputs.get(*index as usize) {
                self.points.insert(Arc::as_ptr(point) as usize, object.clone());
            }
        }
        let polygons = self.polygons(&clipped.polygons)?;
        let cycles = self.keyed_lists(&clipped.cycles)?;
        let vertex_lists = self.keyed_lists(&clipped.vertex_lists)?;
        let extra_lists = self.keyed_lists(&clipped.extra_lists)?;
        let points = PyDict::new(py);
        for (key, point) in &clipped.points {
            points.set_item(self.key(key), self.point(point)?)?;
        }
        let snapped = PyDict::new(py);
        for (key, point) in &clipped.snapped {
            snapped.set_item(self.key(key), self.sum_pair(point)?)?;
        }
        let lifted = self.lifted(clipped, names, normals)?;
        let result = alloc(py, &classes.clipped)?;
        set_slot(&result, &slots.polygons, &polygons)?;
        set_slot(&result, &slots.cycles, &cycles)?;
        set_slot(&result, &slots.vertex_lists, &vertex_lists)?;
        set_slot(&result, &slots.extra_lists, &extra_lists)?;
        set_slot(&result, &slots.points, points.as_any())?;
        set_slot(&result, &slots.snapped, snapped.as_any())?;
        set_slot(&result, &slots.lifted, &lifted)?;
        set_slot(&result, &slots.counters, &self.counters(clipped)?)?;
        set_slot(&result, &slots.note, PyString::new(py, &clipped.note).as_any())?;
        set_slot(&result, &slots.memo, classes.empty_str.bind(py))?;
        Ok(result)
    }
}

// --------------------------------------------------------------------------
// the host
// --------------------------------------------------------------------------

/// The outcome code and detail of a refused clip: the exact-layer codes (1..7) and the clip codes (8..13).
fn error_status<'py>(py: Python<'py>, pool: &Pool, error: &ClipError) -> PyResult<(u8, Option<Bound<'py, PyTuple>>)> {
    let strings = |items: &[&str]| -> PyResult<Option<Bound<'py, PyTuple>>> { PyTuple::new(py, items.iter().map(|item| PyString::new(py, item))).map(Some) };
    Ok(match error {
        ClipError::Exact(error) => exact_status(py, pool, error)?,
        ClipError::Overflow(kind) => (STATUS_OVERFLOW, Some(PyTuple::new(py, [int_from_ibig(py, pool, &IBig::from(*kind as u64))?])?)),
        ClipError::ZeroDivision(text) => (STATUS_ZERO_DIVISION, strings(&[*text])?),
        ClipError::Value(text) => (STATUS_VALUE, strings(&[*text])?),
        ClipError::Refusal { outcome, detail } => (STATUS_REFUSAL, strings(&[*outcome, detail.as_str()])?),
        ClipError::MissingKey(key) => (STATUS_MISSING_KEY, strings(&[key.as_str()])?),
        ClipError::Unsupported(text) => (STATUS_UNSUPPORTED, strings(&[text.as_str()])?),
    })
}

/// `plane._normal_by_position[(x, y, z)] = normal` for every write, in order; the tuples are returned for the result to share.
fn write_normals<'py>(py: Python<'py>, table: Option<&Bound<'py, PyDict>>, writes: &[NormalWrite]) -> PyResult<Vec<Bound<'py, PyAny>>> {
    let tuple = |values: &[f64; 3]| PyTuple::new(py, values.iter().map(|value| PyFloat::new(py, *value)));
    let mut normals = Vec::with_capacity(writes.len());
    for write in writes {
        let normal = tuple(&write.normal)?;
        if let Some(table) = table {
            table.set_item(tuple(&write.position)?, &normal)?;
        }
        normals.push(normal.into_any());
    }
    Ok(normals)
}

impl Host {
    pub fn bind(&mut self, py: Python<'_>, sqrt_sum: &Bound<'_, PyAny>, fraction: &Bound<'_, PyAny>, clipped: &Bound<'_, PyAny>, local_point: &Bound<'_, PyAny>) -> PyResult<()> {
        let intern = |text: &str| PyString::intern(py, text).unbind();
        let local_point = local_point.clone().unbind();
        let (x, y, z) = (intern("x"), intern("y"), intern("z"));
        let local_raw = Raw::probe(py, &local_point, &[&x, &y, &z])?;
        self.classes = Some(Classes {
            pool: Pool::new(py, fraction, sqrt_sum)?,
            clipped: clipped.clone().unbind(),
            local_point,
            local_raw,
            empty_str: PyString::intern(py, "").into_any().unbind(),
            insort: insort(py)?,
            names: Names {
                polygons: intern("polygons"),
                cycles: intern("cycles"),
                vertex_lists: intern("vertex_lists"),
                extra_lists: intern("extra_lists"),
                points: intern("points"),
                snapped: intern("snapped"),
                lifted: intern("lifted"),
                counters: intern("counters"),
                note: intern("note"),
                memo: intern("memo"),
                x,
                y,
                z,
                name: intern("name"),
                chart: intern("chart"),
                corners: intern("corners"),
                twice_area: intern("twice_area"),
                bbox: intern("box"),
                normals: intern("normals"),
                face: intern("face"),
            },
        });
        self.planes.clear();
        Ok(())
    }

    fn classes(&self) -> PyResult<&Classes> {
        self.classes.as_ref().ok_or_else(|| PyRuntimeError::new_err("cftuv_native: the clip classes were not bound (`bind_clip`)"))
    }

    /// `(Fraction, SqrtSumV1, LocalPoint3V1)` slots read and written raw; `None` before `bind_clip`.
    pub fn raw_layouts(&self) -> Option<(bool, bool, bool)> {
        self.classes.as_ref().map(|classes| {
            let (fraction, sqrt_sum) = classes.pool.raw_layouts();
            (fraction, sqrt_sum, classes.local_raw.is_some())
        })
    }

    /// Test-only: the attribute protocol everywhere (and for the planes converted from now on).
    pub fn disable_raw(&mut self) {
        if let Some(classes) = self.classes.as_mut() {
            classes.pool.disable_raw();
            classes.local_raw = None;
        }
        self.planes.clear();
    }

    pub fn cache_size(&self) -> usize {
        self.planes.len()
    }

    pub fn forget(&mut self) {
        self.planes.clear();
        self.warm.clear();
    }

    /// Switches the cross-call cache on or off (off: every call computes everything; the answers and the cost are the same either way).
    pub fn set_warm_enabled(&mut self, enabled: bool) {
        self.warm.enabled = enabled;
    }

    /// Drops the cross-call results only (the converted planes stay).
    pub fn clear_warm(&mut self) {
        self.warm.clear();
    }

    /// Lowers (or restores) the size at which the cross-call cache drops everything: a test knob, a cache has no answer to change.
    pub fn set_warm_limit(&mut self, limit: usize) {
        self.warm.limit = limit;
    }

    /// `(crossing hits, crossings stored, crossing entries, value hits, lift hits)` of the cross-call cache.
    pub fn warm_stats(&self) -> (u64, u64, usize, u64, u64) {
        (self.warm.hits, self.warm.stored, self.warm.len(), self.warm.value_hits, self.warm.lift_hits)
    }

    /// Index of the prepared plane (converted now when this tuple of triangles is new to the session).
    fn prepare(&mut self, py: Python<'_>, triangles: &Bound<'_, PyAny>) -> PyResult<usize> {
        let id = triangles.as_ptr() as usize;
        self.clock += 1;
        if let Some(index) = self.planes.iter().position(|entry| entry.id == id) {
            self.planes[index].stamp = self.clock;
            return Ok(index);
        }
        let entry = convert_plane(py, self.classes()?, triangles, id, self.clock)?;
        if self.planes.len() >= PLANE_LIMIT {
            if let Some(oldest) = self.planes.iter().enumerate().min_by_key(|(_, entry)| entry.stamp).map(|(index, _)| index) {
                self.planes.swap_remove(oldest);
            }
        }
        self.planes.push(entry);
        Ok(self.planes.len() - 1)
    }

    /// One `clip_geometry`: see the module note. `sync` is the memory sync (`None`: unchanged), `budget` the cap and the six
    /// articles (`None`: no budget), `normals` the plane's `_normal_by_position`.
    #[allow(clippy::too_many_arguments)]
    pub fn clip_geometry<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        triangles: &Bound<'py, PyAny>,
        points: &Bound<'py, PyDict>,
        cycles: &Bound<'py, PyAny>,
        polygons: &Bound<'py, PyAny>,
        law: u8,
        seam: &Bound<'py, PyAny>,
        fans: &Bound<'py, PyAny>,
        flows: &Bound<'py, PyAny>,
        by_faces: bool,
        inert: &Bound<'py, PyAny>,
        version: (u32, u32),
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        normals: Option<&Bound<'py, PyDict>>,
        tables: &Tables<'py>,
    ) -> PyResult<Answer<'py>> {
        let started = Instant::now();
        let index = self.prepare(py, triangles)?;
        let plane_ns = nanos(started);
        let began = Instant::now();
        let mut arguments = arguments_of(&self.classes()?.pool, points, cycles, polygons, seam, fans, flows, inert)?;
        let law = match law {
            0 => Law::PlanarPolygons,
            1 => Law::QuadStrips,
            _ => Law::Ears,
        };
        let sync_value = match sync {
            Some(bytes) => Some(Reader::new(bytes, true).get_value().map_err(|error| PyValueError::new_err(format!("the memory sync is not in the wire format: {error}")))?),
            None => None,
        };
        let (cap, articles) = match budget {
            Some((cap, articles)) => (cap, Some(articles)),
            None => (None, None),
        };
        let mut run = CostRun::begin_parts(session, sync_value.as_ref(), cap, articles).map_err(|error| PyValueError::new_err(error.to_string()))?;
        let arguments_ns = nanos(began);

        let computing = Instant::now();
        let prepared = &self.planes[index];
        let warm = &mut self.warm;
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut counts = SignCounts::default();
            let ran = match PyVersion::from_version(version.0, version.1) {
                Err(error) => ClipRun { result: Err(error), writes: Vec::new() },
                Ok(version) => {
                    let input = ClipInput {
                        points: &arguments.points,
                        cycles: &arguments.cycles,
                        polygons: &arguments.polygons,
                        law,
                        seam: &arguments.seam,
                        fans: arguments.fans.as_deref(),
                        flows: arguments.flows.as_deref(),
                        by_faces,
                        inert: &arguments.inert,
                    };
                    let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
                    clip_geometry(&mut ctx, warm, version, &prepared.plane, &input)
                }
            };
            let articles = run.budget_mut().articles();
            (ran, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (ran, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.clip_geometry: native panic: {}", panic_text(&panic)))),
        };

        let applying = Instant::now();
        let classes = self.classes()?;
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);

        let building = Instant::now();
        let shared = write_normals(py, normals, &ran.writes)?;
        let (result, status, detail) = match &ran.result {
            Ok(clipped) => {
                let keys = std::mem::take(&mut arguments.key_objects);
                let mut builder = Builder { py, classes, keys, points: HashMap::default(), inputs: &arguments.point_objects };
                (Some(builder.build(clipped, &self.planes[index].names, &shared)?), 0, None)
            }
            Err(error) => {
                let (status, detail) = error_status(py, &classes.pool, error)?;
                (None, status, detail)
            }
        };
        Ok((result, status, detail, counts.as_array(), articles, changed, [plane_ns, arguments_ns, compute_ns, nanos(building), log_ns]))
    }
}
