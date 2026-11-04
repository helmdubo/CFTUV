//! `mesh.exact_mesh` and `mesh.canonical_triangles` (W1 of the MATERIALIZE chain) behind the persistent session.
//!
//! What crosses per call, and what does not:
//!
//! * the TABLE (`ChainStationTableV1`: the vertex names of the lattice nodes, the JOIN stations, the frames of the runs) is converted once and kept by
//!   the identity of the table object together with a strong reference to it (a retry round builds a new table, so a new entry); the least recently used
//!   goes out at [`TABLE_LIMIT`].
//! * the faces (`FrameFaceV1` with its `CoveredFaceV1`, `SupportLineV1`, `StationRunV1`) are read from the Python objects directly. Their points are
//!   converted per call; the objects the result reuses (the point pairs, the key strings) are the inputs themselves.
//! * the RESULT (`ExactMeshV1`, the key strings, the `(s, r)` pairs, the containers) is built from Rust. A point that the oracle returns (`cycles[i][j][1]`,
//!   `points[key]`) is the input object.
//!
//! The answer is `(result or None, status, detail, counts, articles, changed tables, timings)`. `status` 0 is ok; 1..4 the exact layer's, 11
//! `MaterializationRefusal` (the detail names which, and the shim composes the text with the Python objects), 12 a refusal of the PORT (an input it does not
//! carry), anything else is a refusal of the port too. An ORACLE outcome is applied whole (articles, counters, the memory log, the lines already appended to
//! `notes`) and the shim raises last; a refusal of the PORT is applied NOWHERE (`refusal.rs`).
//!
//! `timings` are nanoseconds `(tables, arguments, compute, result, memory log)`.

use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::rc::Rc;
use std::sync::Arc;
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyFrozenSet, PyList, PySet, PyString, PyTuple};

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::faces::Pt;
use cftuv_clip::point::Point;
use cftuv_core::codec::Reader;
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::session::{CostRun, Session};
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_mesh::data::{ChordNames, Face, Law, Line, MeshInput, MeshOutput, Part, Run, Table};
use cftuv_mesh::error::{MeshError, MeshResult};
use cftuv_mesh::stations::{chord_station_values, Chord, Frame, Value};
use cftuv_mesh::mesh::{canonical_triangles, exact_mesh};
use cftuv_mesh::tally::Tally;
use cftuv_mesh::tessellate::{contour_polygons, fan_faces, Contour, Piece, PieceContext, UvMap};

use crate::coverage::exact_status;
use crate::memlog::{apply_log, insort, Tables};
use crate::pyobj::{alloc, coef_from_number, ibig_from_int, int_from_ibig, is_exactly, set_slot, sqrt_sum_from_py, sqrt_sum_to_py, Pool, SlotMode};
use crate::refusal::{is_native_only, Forced};

/// Tables kept converted per session.
pub const TABLE_LIMIT: usize = 8;

const STATUS_REFUSAL: u8 = 11;
const STATUS_UNSUPPORTED: u8 = 12;
const STATUS_MISSING_KEY: u8 = 13;

/// `(result, status, detail, counts, articles, changed, timings)`.
pub type Answer<'py> = (Option<Bound<'py, PyAny>>, u8, Option<Bound<'py, PyTuple>>, [u64; 5], [u64; 6], u8, [u64; 8]);

/// A failure of reading the inputs: a Python error, or an input the port does not carry (a named refusal of the port, never an answer).
enum Fail {
    Py(PyErr),
    Unsupported(String),
}

impl From<PyErr> for Fail {
    fn from(error: PyErr) -> Fail {
        Fail::Py(error)
    }
}

type Read<T> = Result<T, Fail>;

fn unsupported<T>(text: impl Into<String>) -> Read<T> {
    Err(Fail::Unsupported(text.into()))
}

struct Names {
    face: Py<PyString>,
    line: Py<PyString>,
    run: Py<PyString>,
    fan_station: Py<PyString>,
    flow_key: Py<PyString>,
    points: Py<PyString>,
    doubled_area: Py<PyString>,
    owner: Py<PyString>,
    parts: Py<PyString>,
    a: Py<PyString>,
    b: Py<PyString>,
    c: Py<PyString>,
    q: Py<PyString>,
    run_id: Py<PyString>,
    origin: Py<PyString>,
    covector: Py<PyString>,
    s_origin: Py<PyString>,
    inverse_length: Py<PyString>,
    node_vertex_ids: Py<PyString>,
    joins: Py<PyString>,
    frame_of_run: Py<PyString>,
    cycles: Py<PyString>,
    points_slot: Py<PyString>,
    facts: Py<PyString>,
    polygons: Py<PyString>,
    tally: Py<PyString>,
    rungs: Py<PyString>,
}

struct Classes {
    pool: Pool,
    result: Py<PyAny>,
    counter: Py<PyAny>,
    insort: Py<PyAny>,
    names: Names,
}

struct CachedTable {
    id: usize,
    stamp: u64,
    /// Keeps the identity of the table valid, and the dictionaries it was read from alive.
    _holder: Py<PyAny>,
    table: Arc<Table>,
}

pub struct Host {
    classes: Option<Classes>,
    tables: Vec<CachedTable>,
    clock: u64,
}

impl Default for Host {
    fn default() -> Host {
        Host { classes: None, tables: Vec::new(), clock: 0 }
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
// reading the inputs
// --------------------------------------------------------------------------

fn field<'py>(object: &Bound<'py, PyAny>, name: &Py<PyString>) -> PyResult<Bound<'py, PyAny>> {
    object.getattr(name.bind(object.py()))
}

fn text_of(item: &Bound<'_, PyAny>, what: &str) -> Read<String> {
    match item.cast_exact::<PyString>() {
        Ok(text) => Ok(text.to_str()?.to_string()),
        Err(_) => unsupported(format!("{what} must be a str, not {}", item.get_type().name()?)),
    }
}

fn exact_int(pool: &Pool, item: &Bound<'_, PyAny>, what: &str) -> Read<IBig> {
    if !is_exactly(item, &pool.int_type) {
        return unsupported(format!("{what} must be an int, not {}", item.get_type().name()?));
    }
    Ok(ibig_from_int(item)?)
}

fn rat_of(pool: &Pool, item: &Bound<'_, PyAny>, what: &str) -> Read<Rat> {
    match coef_from_number(pool, item) {
        Ok(coef) => Ok(coef.into_value()),
        Err(_) => unsupported(format!("{what} must be an int or a Fraction, not {}", item.get_type().name()?)),
    }
}

fn sum_of(pool: &Pool, item: &Bound<'_, PyAny>, what: &str) -> Read<SqrtSum> {
    if !is_exactly(item, &pool.sqrt_sum) {
        return unsupported(format!("{what} must be a SqrtSumV1, not {}", item.get_type().name()?));
    }
    Ok(sqrt_sum_from_py(pool, item)?)
}

/// A tuple of two `SqrtSumV1` as a native point.
fn point_of(pool: &Pool, item: &Bound<'_, PyAny>) -> Read<Point> {
    let Ok(pair) = item.cast_exact::<PyTuple>() else {
        return unsupported(format!("a point must be a tuple of two SqrtSumV1, not {}", item.get_type().name()?));
    };
    if pair.len() != 2 {
        return unsupported("a point must be a tuple of two SqrtSumV1");
    }
    Ok((sum_of(pool, &pair.get_item(0)?, "a point coordinate")?, sum_of(pool, &pair.get_item(1)?, "a point coordinate")?))
}

/// The points of a contour (a tuple of point pairs): native points and the pair objects the result reuses.
fn contour_of<'py>(pool: &Pool, points: &Bound<'py, PyAny>) -> Read<(Vec<Point>, Vec<Bound<'py, PyAny>>)> {
    let Ok(tuple) = points.cast_exact::<PyTuple>() else {
        return unsupported(format!("the points of a face must be a tuple, not {}", points.get_type().name()?));
    };
    let mut native = Vec::with_capacity(tuple.len());
    let mut objects = Vec::with_capacity(tuple.len());
    for item in tuple.iter() {
        native.push(point_of(pool, &item)?);
        objects.push(item);
    }
    Ok((native, objects))
}

fn run_of(classes: &Classes, object: &Bound<'_, PyAny>) -> Read<Run> {
    let pool = &classes.pool;
    let names = &classes.names;
    let origin = field(object, &names.origin)?;
    let Ok(origin) = origin.cast_exact::<PyTuple>() else {
        return unsupported("the origin of a run must be a tuple");
    };
    if origin.len() != 2 {
        return unsupported("the origin of a run must have two items");
    }
    let lattice = |item: Bound<'_, PyAny>| -> Read<SqrtSum> {
        let value = exact_int(pool, &item, "a run origin")?;
        Ok(SqrtSum::rational(&Rat::from_int(value)))
    };
    let origin = (lattice(origin.get_item(0)?)?, lattice(origin.get_item(1)?)?);
    let covector = field(object, &names.covector)?;
    let Ok(covector) = covector.cast_exact::<PyTuple>() else {
        return unsupported("the covector of a run must be a tuple");
    };
    if covector.len() != 2 {
        return unsupported("the covector of a run must have two items");
    }
    let covector = (rat_of(pool, &covector.get_item(0)?, "a covector item")?, rat_of(pool, &covector.get_item(1)?, "a covector item")?);
    Ok(Run {
        id: text_of(&field(object, &names.run_id)?, "a run id")?,
        origin,
        covector,
        s_origin: sum_of(pool, &field(object, &names.s_origin)?, "s_origin")?,
        inverse_length: sum_of(pool, &field(object, &names.inverse_length)?, "inverse_length")?,
    })
}

fn line_of(classes: &Classes, object: &Bound<'_, PyAny>) -> Read<Line> {
    let pool = &classes.pool;
    let names = &classes.names;
    Ok(Line {
        a: rat_of(pool, &field(object, &names.a)?, "line.a")?,
        b: rat_of(pool, &field(object, &names.b)?, "line.b")?,
        c: SqrtSum::rational(&rat_of(pool, &field(object, &names.c)?, "line.c")?),
        q: rat_of(pool, &field(object, &names.q)?, "line.q")?,
    })
}

/// The first two items of an owner as integers (the node of a fan), or `None`.
fn apex_of(pool: &Pool, owner: &Bound<'_, PyAny>) -> Read<Option<(IBig, IBig)>> {
    let Ok(owner) = owner.cast_exact::<PyTuple>() else {
        return unsupported("an owner must be a tuple");
    };
    if owner.len() < 2 {
        return Ok(None);
    }
    let (x, y) = (owner.get_item(0)?, owner.get_item(1)?);
    if !is_exactly(&x, &pool.int_type) || !is_exactly(&y, &pool.int_type) {
        return Ok(None);
    }
    Ok(Some((ibig_from_int(&x)?, ibig_from_int(&y)?)))
}

/// One merged face with its frame, and the point pairs the result reuses.
fn face_of<'py>(classes: &Classes, runs: &mut HashMap<usize, Rc<Run>, FxBuild>, frame_face: &Bound<'py, PyAny>, region: u32) -> Read<(Face, Vec<Bound<'py, PyAny>>)> {
    let pool = &classes.pool;
    let names = &classes.names;
    let covered = field(frame_face, &names.face)?;
    let (points, objects) = contour_of(pool, &field(&covered, &names.points)?)?;
    let area = sum_of(pool, &field(&covered, &names.doubled_area)?, "the doubled area")?;
    let apex = apex_of(pool, &field(&covered, &names.owner)?)?;
    let mut parts = Vec::new();
    let held = field(&covered, &names.parts)?;
    let Ok(held) = held.cast_exact::<PyTuple>() else {
        return unsupported("the parts of a face must be a tuple");
    };
    if held.len() >= 2 {
        for part in held.iter() {
            let (points, _) = contour_of(pool, &field(&part, &names.points)?)?;
            parts.push(Part { points, area: sum_of(pool, &field(&part, &names.doubled_area)?, "the doubled area of a part")? });
        }
    }
    let run_object = field(frame_face, &names.run)?;
    let address = run_object.as_ptr() as usize;
    let run = match runs.get(&address) {
        Some(run) => run.clone(),
        None => {
            let run = Rc::new(run_of(classes, &run_object)?);
            runs.insert(address, run.clone());
            run
        }
    };
    let fan = field(frame_face, &names.fan_station)?;
    let fan_station = if fan.is_none() { None } else { Some(sum_of(pool, &fan, "fan_station")?) };
    let in_flow = !field(frame_face, &names.flow_key)?.is_none();
    let line = line_of(classes, &field(frame_face, &names.line)?)?;
    Ok((Face { region, points, area, apex, parts, line, run, fan_station, in_flow }, objects))
}

fn table_of(classes: &Classes, table: &Bound<'_, PyAny>) -> Read<Table> {
    let pool = &classes.pool;
    let names = &classes.names;
    let mut native = Table::default();
    let nodes = field(table, &names.node_vertex_ids)?;
    let Ok(nodes) = nodes.cast_exact::<PyDict>() else {
        return unsupported("node_vertex_ids must be a dict");
    };
    for (key, value) in nodes.iter() {
        let Ok(key) = key.cast_exact::<PyTuple>() else {
            return unsupported("a key of node_vertex_ids must be (region, node)");
        };
        if key.len() != 2 {
            return unsupported("a key of node_vertex_ids must be (region, node)");
        }
        let region = text_of(&key.get_item(0)?, "a region id")?;
        let node = key.get_item(1)?;
        let Ok(node) = node.cast_exact::<PyTuple>() else {
            return unsupported("a node must be a tuple");
        };
        if node.len() != 2 {
            return unsupported("a node must have two coordinates");
        }
        let node = (exact_int(pool, &node.get_item(0)?, "a node coordinate")?, exact_int(pool, &node.get_item(1)?, "a node coordinate")?);
        native.node_vertex_ids.entry(region).or_default().insert(node, text_of(&value, "a vertex id")?);
    }
    let joins = field(table, &names.joins)?;
    let Ok(joins) = joins.cast_exact::<PyDict>() else {
        return unsupported("joins must be a dict");
    };
    for (key, value) in joins.iter() {
        let Ok(key) = key.cast_exact::<PyTuple>() else {
            return unsupported("a key of joins must be (run, run)");
        };
        if key.len() != 2 {
            return unsupported("a key of joins must be (run, run)");
        }
        native.joins.insert((text_of(&key.get_item(0)?, "a run id")?, text_of(&key.get_item(1)?, "a run id")?), sum_of(pool, &value, "a join station")?);
    }
    let frames = field(table, &names.frame_of_run)?;
    let Ok(frames) = frames.cast_exact::<PyDict>() else {
        return unsupported("frame_of_run must be a dict");
    };
    for (run, frame) in frames.iter() {
        native.frame_of_run.insert(text_of(&run, "a run id")?, text_of(&frame, "a frame key")?);
    }
    Ok(native)
}

fn chords_of(classes: &Classes, chord_names: &Bound<'_, PyAny>) -> Read<ChordNames> {
    let mut native = ChordNames::default();
    let Ok(chord_names) = chord_names.cast_exact::<PyDict>() else {
        return unsupported("chord_names must be a dict");
    };
    for (key, value) in chord_names.iter() {
        let Ok(key) = key.cast_exact::<PyTuple>() else {
            return unsupported("a key of chord_names must be (region, point key)");
        };
        if key.len() != 2 {
            return unsupported("a key of chord_names must be (region, point key)");
        }
        let region = text_of(&key.get_item(0)?, "a region id")?;
        let identity = key.get_item(1)?;
        let Ok(identity) = identity.cast_exact::<PyTuple>() else {
            return unsupported("a point key must be a pair of term tuples");
        };
        if identity.len() != 2 {
            return unsupported("a point key must be a pair of term tuples");
        }
        let mut coordinates = Vec::with_capacity(2);
        for terms in identity.iter() {
            // a `SqrtSumV1` made from the terms of the key, read the usual way
            let built = classes.pool.sqrt_sum.bind(identity.py()).call1((terms,))?;
            coordinates.push(sum_of(&classes.pool, &built, "a chord point coordinate")?);
        }
        let second = coordinates.pop().expect("two coordinates");
        let first = coordinates.pop().expect("two coordinates");
        native.entries.push((region, (first, second), text_of(&value, "a chord vertex name")?));
    }
    Ok(native)
}

// --------------------------------------------------------------------------
// building the result
// --------------------------------------------------------------------------

fn key_objects<'py>(py: Python<'py>, keys: &[cftuv_mesh::data::Key]) -> Vec<Bound<'py, PyAny>> {
    keys.iter().map(|key| PyString::new(py, &key.name).into_any()).collect()
}

fn polygons_list<'py>(py: Python<'py>, polygons: &[Vec<Vec<u32>>], keys: &[Bound<'py, PyAny>]) -> PyResult<Bound<'py, PyAny>> {
    let mut faces = Vec::with_capacity(polygons.len());
    for face in polygons {
        let mut loops = Vec::with_capacity(face.len());
        for polygon in face {
            let items: Vec<Bound<'py, PyAny>> = polygon.iter().map(|key| keys[*key as usize].clone()).collect();
            loops.push(PyTuple::new(py, items)?);
        }
        faces.push(PyTuple::new(py, loops)?);
    }
    Ok(PyList::new(py, faces)?.into_any())
}

/// The `Exact`/refusal detail of a refused mesh: `(status, detail)`.
fn error_status<'py>(py: Python<'py>, pool: &Pool, error: &MeshError, keys: &[cftuv_mesh::data::Key]) -> PyResult<(u8, Option<Bound<'py, PyTuple>>)> {
    let strings = |items: &[&str]| -> PyResult<Option<Bound<'py, PyTuple>>> { PyTuple::new(py, items.iter().map(|item| PyString::new(py, item))).map(Some) };
    let int = |value: i64| int_from_ibig(py, pool, &IBig::from(value));
    Ok(match error {
        MeshError::Exact(error) => exact_status(py, pool, error)?,
        MeshError::VertexKeyCollision { key, region, place } => (STATUS_REFUSAL, strings(&["collision", key.as_str(), region.as_str(), place.as_str()])?),
        MeshError::StationConflict { key, region, face, answered } => {
            let mut rows = Vec::with_capacity(answered.len());
            for (run, s, r) in answered {
                rows.push(PyTuple::new(py, [PyString::new(py, run).into_any(), sqrt_sum_to_py(py, pool, s)?, sqrt_sum_to_py(py, pool, r)?])?.into_any());
            }
            let detail = PyTuple::new(
                py,
                [
                    PyString::new(py, "conflict").into_any(),
                    PyString::new(py, &keys[*key as usize].name).into_any(),
                    int(i64::from(*region))?,
                    int(*face as i64)?,
                    PyTuple::new(py, rows)?.into_any(),
                ],
            )?;
            (STATUS_REFUSAL, Some(detail))
        }
        MeshError::NotClosed { whose, what } => {
            let detail = PyTuple::new(py, [PyString::new(py, "not_closed").into_any(), int(whose.face as i64)?, int(whose.part.map_or(-1, |part| part as i64))?, PyString::new(py, what.parts()).into_any()])?;
            (STATUS_REFUSAL, Some(detail))
        }
        MeshError::NoTriangulation { whose, points } => {
            let detail = PyTuple::new(py, [PyString::new(py, "no_triangulation").into_any(), int(whose.face as i64)?, int(whose.part.map_or(-1, |part| part as i64))?, int(*points as i64)?])?;
            (STATUS_REFUSAL, Some(detail))
        }
        MeshError::MissingFact { region, key } => (STATUS_MISSING_KEY, Some(PyTuple::new(py, [int(i64::from(*region))?, PyString::new(py, &keys[*key as usize].name).into_any()])?)),
        MeshError::Unsupported(text) => (STATUS_UNSUPPORTED, strings(&[text.as_str()])?),
    })
}

// --------------------------------------------------------------------------
// the host
// --------------------------------------------------------------------------

impl Host {
    #[allow(clippy::too_many_arguments)]
    pub fn bind(&mut self, py: Python<'_>, sqrt_sum: &Bound<'_, PyAny>, fraction: &Bound<'_, PyAny>, result: &Bound<'_, PyAny>, counter: &Bound<'_, PyAny>, mode: SlotMode) -> PyResult<()> {
        let intern = |text: &str| PyString::intern(py, text).unbind();
        self.classes = Some(Classes {
            pool: Pool::new(py, fraction, sqrt_sum, mode)?,
            result: result.clone().unbind(),
            counter: counter.clone().unbind(),
            insort: insort(py)?,
            names: Names {
                face: intern("face"),
                line: intern("line"),
                run: intern("run"),
                fan_station: intern("fan_station"),
                flow_key: intern("flow_key"),
                points: intern("points"),
                doubled_area: intern("doubled_area"),
                owner: intern("owner"),
                parts: intern("parts"),
                a: intern("a"),
                b: intern("b"),
                c: intern("c"),
                q: intern("q"),
                run_id: intern("run_id"),
                origin: intern("origin"),
                covector: intern("covector"),
                s_origin: intern("s_origin"),
                inverse_length: intern("inverse_length"),
                node_vertex_ids: intern("node_vertex_ids"),
                joins: intern("joins"),
                frame_of_run: intern("frame_of_run"),
                cycles: intern("cycles"),
                points_slot: intern("points"),
                facts: intern("facts"),
                polygons: intern("polygons"),
                tally: intern("tally"),
                rungs: intern("rungs"),
            },
        });
        self.tables.clear();
        Ok(())
    }

    fn classes(&self) -> PyResult<&Classes> {
        self.classes.as_ref().ok_or_else(|| PyRuntimeError::new_err("cftuv_native: the materialize_w1 classes were not bound (`bind_w1`)"))
    }

    pub fn cache_size(&self) -> usize {
        self.tables.len()
    }

    pub fn forget(&mut self) {
        self.tables.clear();
    }

    /// Test-only: the attribute protocol everywhere.
    pub fn disable_raw(&mut self) {
        if let Some(classes) = self.classes.as_mut() {
            classes.pool.disable_raw();
        }
        self.tables.clear();
    }

    pub fn raw_layouts(&self) -> Option<(bool, bool)> {
        self.classes.as_ref().map(|classes| classes.pool.raw_layouts())
    }

    /// CERTIFICATE_SIDE_CUTS_NATIVE_V1 (`certificate.rs`): the converted table and the frames of `frame_faces` (layout region `regions[i]` for face `i`), or why the port
    /// does not carry them (`Err(Ok(text))`); a Python error is `Err(Err(error))`.
    pub(crate) fn certificate_frames(&mut self, table: &Bound<'_, PyAny>, frame_faces: &Bound<'_, PyList>, regions: &[u32]) -> Result<(Arc<Table>, Vec<Frame>), Result<String, PyErr>> {
        let lift = |fail: Fail| match fail {
            Fail::Py(error) => Err(error),
            Fail::Unsupported(text) => Ok(text),
        };
        let table = self.table(table).map_err(lift)?;
        let classes = self.classes.as_ref().ok_or_else(|| Ok("the materialize_w1 classes were not bound".to_string()))?;
        if regions.len() != frame_faces.len() {
            return Err(Ok("frame_faces and regions must have one length".to_string()));
        }
        let mut runs: HashMap<usize, Rc<Run>, FxBuild> = HashMap::default();
        let mut frames = Vec::with_capacity(regions.len());
        for (frame_face, region) in frame_faces.iter().zip(regions) {
            frames.push(frame_of(classes, &mut runs, &frame_face, *region).map_err(lift)?);
        }
        Ok((table, frames))
    }

    /// The converted table (read now when this table object is new to the session).
    fn table(&mut self, table: &Bound<'_, PyAny>) -> Read<Arc<Table>> {
        let id = table.as_ptr() as usize;
        self.clock += 1;
        if let Some(found) = self.tables.iter_mut().find(|entry| entry.id == id) {
            found.stamp = self.clock;
            return Ok(found.table.clone());
        }
        let native = Arc::new(table_of(self.classes.as_ref().ok_or_else(|| Fail::Unsupported("the classes were not bound".to_string()))?, table)?);
        if self.tables.len() >= TABLE_LIMIT {
            if let Some(oldest) = self.tables.iter().enumerate().min_by_key(|(_, entry)| entry.stamp).map(|(index, _)| index) {
                self.tables.swap_remove(oldest);
            }
        }
        self.tables.push(CachedTable { id, stamp: self.clock, _holder: table.clone().unbind(), table: native.clone() });
        Ok(native)
    }

    /// One `exact_mesh`: see the module note.
    #[allow(clippy::too_many_arguments)]
    pub fn exact_mesh<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        frame_faces: &Bound<'py, PyAny>,
        region_ids: &Bound<'py, PyAny>,
        regions: &Bound<'py, PyAny>,
        table: &Bound<'py, PyAny>,
        notes: &Bound<'py, PyList>,
        chord_names: &Bound<'py, PyAny>,
        lattice_alpha: &Bound<'py, PyAny>,
        chart_cw: bool,
        law: u8,
        exact_plane: bool,
        consumed_piece: bool,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
        forced: Option<Forced>,
    ) -> PyResult<Answer<'py>> {
        let given = budget.map_or([0; 6], |(_, articles)| articles);
        let started = Instant::now();
        let table_native = match self.table(table) {
            Ok(table) => table,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(unsupported_answer(py, &text, given)?),
        };
        let tables_ns = nanos(started);
        let began = Instant::now();
        let classes = self.classes()?;
        let read = (|| -> Read<(Vec<Face>, Vec<Vec<Bound<'py, PyAny>>>, Vec<String>, ChordNames, Rat)> {
            let pool = &classes.pool;
            let items = frame_faces.cast::<PyList>().map_err(|_| Fail::Unsupported("frame_faces must be a list".to_string()))?;
            let ids = region_ids.cast::<PyList>().map_err(|_| Fail::Unsupported("region_ids must be a list".to_string()))?;
            let numbers = regions.cast::<PyList>().map_err(|_| Fail::Unsupported("regions must be a list".to_string()))?;
            if ids.len() != items.len() || numbers.len() != items.len() {
                return unsupported("frame_faces, region_ids and regions must have one length");
            }
            let mut runs: HashMap<usize, Rc<Run>, FxBuild> = HashMap::default();
            let mut faces = Vec::with_capacity(items.len());
            let mut objects = Vec::with_capacity(items.len());
            let mut names = Vec::with_capacity(items.len());
            for ((frame_face, region), number) in items.iter().zip(ids.iter()).zip(numbers.iter()) {
                let number = exact_int(pool, &number, "a layout region")?;
                let number = u32::try_from(&number).map_err(|_| Fail::Unsupported("a layout region beyond 32 bits".to_string()))?;
                names.push(text_of(&region, "a region id")?);
                let (face, held) = face_of(classes, &mut runs, &frame_face, number)?;
                faces.push(face);
                objects.push(held);
            }
            let chords = chords_of(classes, chord_names)?;
            let alpha = rat_of(pool, lattice_alpha, "lattice_alpha")?;
            Ok((faces, objects, names, chords, alpha))
        })();
        let (faces, objects, region_names, chords, alpha) = match read {
            Ok(read) => read,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(unsupported_answer(py, &text, given)?),
        };
        let law = match law {
            0 => Law::PlanarPolygons,
            1 => Law::QuadStrips,
            _ => Law::Triangles,
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
        let input = MeshInput { faces: &faces, region_ids: &region_names, table: &table_native, chords: &chords, lattice_alpha: &alpha, chart_cw, law, exact_plane, consumed_piece };
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut counts = SignCounts::default();
            let mut out = MeshOutput::default();
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            let mut result = exact_mesh(&mut ctx, &input, &mut out);
            if let Some(knob) = forced {
                assert!(!knob.panics(), "forced by the test knob");
                if let Some(error) = knob.exact_error() {
                    result = Err(MeshError::Exact(error));
                } else if knob.clip_error().is_some() {
                    result = Err(MeshError::Unsupported("forced by the test knob".to_string()));
                }
            }
            let articles = run.budget_mut().articles();
            (result, out, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (result, out, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.exact_mesh: native panic: {}", panic_text(&panic)))),
        };

        // Classify first: a refusal of the port is applied nowhere, so nothing below it may have run.
        let classes = self.classes()?;
        let failure = match &result {
            Ok(()) => None,
            Err(error) => Some(error_status(py, &classes.pool, error, &out.keys)?),
        };
        if let Some((status, detail)) = &failure {
            if is_native_only(*status) {
                return Ok((None, *status, detail.clone(), [0; 5], given, 0, [tables_ns, arguments_ns, compute_ns, 0, 0, 0, 0, 0]));
            }
        }

        let building = Instant::now();
        let built = match (&result, failure) {
            (Ok(()), _) => Some(build_result(py, classes, &objects, &out)?),
            (Err(_), Some(_)) => None,
            (Err(_), None) => return Err(PyRuntimeError::new_err("cftuv_native: a refused mesh without an outcome code")),
        };
        let (status, detail) = match (&built, &result) {
            (Some(_), _) => (0, None),
            (None, Err(error)) => error_status(py, &classes.pool, error, &out.keys)?,
            (None, Ok(())) => unreachable!("a mesh that answered has a result"),
        };
        let building_ns = nanos(building);

        // Commit: the memory log on the real tables, then the lines appended to `notes` (what the oracle leaves behind, also when it raised afterwards).
        let applying = Instant::now();
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);
        for line in &out.notes {
            notes.append(line)?;
        }
        Ok((built, status, detail, counts.as_array(), articles, changed, [tables_ns, arguments_ns, compute_ns, building_ns, log_ns, out.phase_ns[0], out.phase_ns[1], out.phase_ns[2]]))
    }
}

// --------------------------------------------------------------------------
// station_values of the clip stage (chords and anchors)
// --------------------------------------------------------------------------

/// What the call of the clip stage reads of the Python objects: the frames of the faces, the new vertices by key number, the chords, the key objects.
type ChordRead<'py> = (Vec<Frame>, Vec<Vec<(u32, Point)>>, Vec<HashMap<u32, Chord, FxBuild>>, Vec<Bound<'py, PyAny>>);

fn frame_of(classes: &Classes, runs: &mut HashMap<usize, Rc<Run>, FxBuild>, frame_face: &Bound<'_, PyAny>, region: u32) -> Read<Frame> {
    let names = &classes.names;
    let run_object = field(frame_face, &names.run)?;
    let address = run_object.as_ptr() as usize;
    let run = match runs.get(&address) {
        Some(run) => run.clone(),
        None => {
            let run = Rc::new(run_of(classes, &run_object)?);
            runs.insert(address, run.clone());
            run
        }
    };
    let fan = field(frame_face, &names.fan_station)?;
    let fan_station = if fan.is_none() { None } else { Some(sum_of(&classes.pool, &fan, "fan_station")?) };
    let line = line_of(classes, &field(frame_face, &names.line)?)?;
    Ok(Frame { region, line, run, fan_station })
}

/// The number of a key string, interned in the order met; the object is kept for the result.
fn key_number<'py>(index: &mut HashMap<String, u32, FxBuild>, objects: &mut Vec<Bound<'py, PyAny>>, key: Bound<'py, PyAny>) -> Read<u32> {
    let name = text_of(&key, "a vertex key")?;
    if let Some(found) = index.get(&name) {
        return Ok(*found);
    }
    let found = u32::try_from(objects.len()).map_err(|_| Fail::Unsupported("more than 2^32 keys".to_string()))?;
    index.insert(name, found);
    objects.push(key);
    Ok(found)
}

fn read_chord_inputs<'py>(classes: &Classes, frame_faces: &Bound<'py, PyAny>, regions: &Bound<'py, PyAny>, lists: &Bound<'py, PyAny>, chords: &Bound<'py, PyAny>) -> Read<ChordRead<'py>> {
    let pool = &classes.pool;
    let items = frame_faces.cast::<PyList>().map_err(|_| Fail::Unsupported("frame_faces must be a list".to_string()))?;
    let numbers = regions.cast::<PyList>().map_err(|_| Fail::Unsupported("regions must be a list".to_string()))?;
    let lists = lists.cast::<PyList>().map_err(|_| Fail::Unsupported("the cycles of the clip must be a list".to_string()))?;
    let chords = chords.cast::<PyList>().map_err(|_| Fail::Unsupported("chords must be a list".to_string()))?;
    if numbers.len() != items.len() || lists.len() != items.len() || chords.len() != items.len() {
        return unsupported("frame_faces, regions, the cycles of the clip and chords must have one length");
    }
    let mut runs: HashMap<usize, Rc<Run>, FxBuild> = HashMap::default();
    let mut index: HashMap<String, u32, FxBuild> = HashMap::default();
    let mut objects: Vec<Bound<'py, PyAny>> = Vec::new();
    let mut frames = Vec::with_capacity(items.len());
    let mut cycles = Vec::with_capacity(items.len());
    let mut edges = Vec::with_capacity(items.len());
    for (((frame_face, number), cycle), face_chords) in items.iter().zip(numbers.iter()).zip(lists.iter()).zip(chords.iter()) {
        let number = exact_int(pool, &number, "a layout region")?;
        let number = u32::try_from(&number).map_err(|_| Fail::Unsupported("a layout region beyond 32 bits".to_string()))?;
        frames.push(frame_of(classes, &mut runs, &frame_face, number)?);
        let Ok(cycle) = cycle.cast_exact::<PyList>() else {
            return unsupported("a cycle of the clip must be a list");
        };
        let mut entries = Vec::with_capacity(cycle.len());
        for entry in cycle.iter() {
            let Ok(pair) = entry.cast_exact::<PyTuple>() else {
                return unsupported("a cycle entry must be (key, point)");
            };
            if pair.len() != 2 {
                return unsupported("a cycle entry must be (key, point)");
            }
            let key = key_number(&mut index, &mut objects, pair.get_item(0)?)?;
            entries.push((key, point_of(pool, &pair.get_item(1)?)?));
        }
        cycles.push(entries);
        let Ok(face_chords) = face_chords.cast_exact::<PyDict>() else {
            return unsupported("the chords of a face must be a dict");
        };
        let mut found: HashMap<u32, Chord, FxBuild> = HashMap::default();
        for (key, chord) in face_chords.iter() {
            let Ok(chord) = chord.cast_exact::<PyTuple>() else {
                return unsupported("a chord must be (u, v, point u, point v)");
            };
            if chord.len() != 4 {
                return unsupported("a chord must be (u, v, point u, point v)");
            }
            let key = key_number(&mut index, &mut objects, key)?;
            let u = key_number(&mut index, &mut objects, chord.get_item(0)?)?;
            let v = key_number(&mut index, &mut objects, chord.get_item(1)?)?;
            found.insert(key, Chord { u, v, u_point: point_of(pool, &chord.get_item(2)?)?, v_point: point_of(pool, &chord.get_item(3)?)? });
        }
        edges.push(found);
    }
    Ok((frames, cycles, edges, objects))
}

/// `anchors[(region, key)]` read now: the fact as a native value, `None` when the slot is missing.
fn anchor_of<'py>(pool: &Pool, anchors: &Bound<'py, PyDict>, region: u32, key: &Bound<'py, PyAny>) -> Read<Option<Value>> {
    let py = anchors.py();
    let slot = PyTuple::new(py, [int_from_ibig(py, pool, &IBig::from(u64::from(region)))?, key.clone()])?;
    let Some(fact) = anchors.get_item(slot)? else {
        return Ok(None);
    };
    let Ok(pair) = fact.cast_exact::<PyTuple>() else {
        return unsupported("a fact must be a tuple (s, r)");
    };
    if pair.len() != 2 {
        return unsupported("a fact must be a tuple (s, r)");
    }
    Ok(Some(Rc::new((sum_of(pool, &pair.get_item(0)?, "a fact")?, sum_of(pool, &pair.get_item(1)?, "a fact")?))))
}

impl Host {
    /// One `station_values(frame_faces, lists, layout, table, lattice_alpha, budget, tally, chords=chords, anchors=anchors)` of the clip stage
    /// (`clip.cut_domain`). The answer is the exact-mesh answer with `(facts or None, tally items)` as the result: `facts` the dict of the oracle in its order,
    /// the tally items what the oracle adds to the caller's Counter (also when it refused afterwards).
    #[allow(clippy::too_many_arguments)]
    pub fn chord_stations<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        frame_faces: &Bound<'py, PyAny>,
        regions: &Bound<'py, PyAny>,
        lists: &Bound<'py, PyAny>,
        chords: &Bound<'py, PyAny>,
        anchors: &Bound<'py, PyAny>,
        table: &Bound<'py, PyAny>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
        forced: Option<Forced>,
    ) -> PyResult<Answer<'py>> {
        let given = budget.map_or([0; 6], |(_, articles)| articles);
        let started = Instant::now();
        let table_native = match self.table(table) {
            Ok(table) => table,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(unsupported_answer(py, &text, given)?),
        };
        let tables_ns = nanos(started);
        let began = Instant::now();
        let classes = self.classes()?;
        let pool = &classes.pool;
        let Ok(anchors) = anchors.cast_exact::<PyDict>() else {
            return Ok(unsupported_answer(py, "anchors must be a dict", given)?);
        };
        let (frames, cycles, edges, objects) = match read_chord_inputs(classes, frame_faces, regions, lists, chords) {
            Ok(read) => read,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(unsupported_answer(py, &text, given)?),
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
        let mut python_error: Option<PyErr> = None;
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut counts = SignCounts::default();
            let mut tally = Tally::new();
            let mut lookup = |region: u32, key: u32| -> MeshResult<Option<Value>> {
                match anchor_of(pool, anchors, region, &objects[key as usize]) {
                    Ok(found) => Ok(found),
                    Err(Fail::Unsupported(text)) => Err(MeshError::Unsupported(text)),
                    Err(Fail::Py(error)) => {
                        python_error = Some(error);
                        Err(MeshError::Unsupported("reading an anchor raised".to_string()))
                    }
                }
            };
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            let mut result = chord_station_values(&mut ctx, &frames, &cycles, &edges, &mut lookup, &table_native, &mut tally).map(|stations| {
                stations.slots.iter().zip(stations.facts.iter()).map(|(slot, value)| (*slot, value.clone())).collect::<Vec<((u32, u32), Value)>>()
            });
            if let Some(knob) = forced {
                assert!(!knob.panics(), "forced by the test knob");
                if let Some(error) = knob.exact_error() {
                    result = Err(MeshError::Exact(error));
                } else if knob.clip_error().is_some() {
                    result = Err(MeshError::Unsupported("forced by the test knob".to_string()));
                }
            }
            let articles = run.budget_mut().articles();
            (result, tally, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (result, tally, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.chord_stations: native panic: {}", panic_text(&panic)))),
        };
        if let Some(error) = python_error {
            return Err(error);
        }
        let keys: Vec<cftuv_mesh::data::Key> =
            objects.iter().map(|object| cftuv_mesh::data::Key { name: object.extract::<String>().unwrap_or_default(), first: (0, 0) }).collect();
        let failure = match &result {
            Ok(_) => None,
            Err(error) => Some(error_status(py, pool, error, &keys)?),
        };
        if let Some((status, detail)) = &failure {
            if is_native_only(*status) {
                return Ok((None, *status, detail.clone(), [0; 5], given, 0, [tables_ns, arguments_ns, compute_ns, 0, 0, 0, 0, 0]));
            }
        }
        let building = Instant::now();
        let mut tally_items = Vec::new();
        for (name, value) in tally.items() {
            tally_items.push(PyTuple::new(py, [PyString::new(py, name).into_any(), int_from_ibig(py, pool, &IBig::from(value))?])?);
        }
        let tally_items = PyTuple::new(py, tally_items)?.into_any();
        let (built, status, detail) = match (&result, failure) {
            (Ok(facts), _) => {
                let found = PyDict::new(py);
                let mut cache: HashMap<usize, Bound<'py, PyAny>, FxBuild> = HashMap::default();
                for ((region, key), value) in facts {
                    let slot = PyTuple::new(py, [int_from_ibig(py, pool, &IBig::from(u64::from(*region)))?, objects[*key as usize].clone()])?;
                    let address = Rc::as_ptr(value) as usize;
                    let pair = match cache.get(&address) {
                        Some(pair) => pair.clone(),
                        None => {
                            let pair = PyTuple::new(py, [sqrt_sum_to_py(py, pool, &value.0)?, sqrt_sum_to_py(py, pool, &value.1)?])?.into_any();
                            cache.insert(address, pair.clone());
                            pair
                        }
                    };
                    found.set_item(slot, pair)?;
                }
                (Some(PyTuple::new(py, [found.into_any(), tally_items])?.into_any()), 0, None)
            }
            (Err(_), Some((status, detail))) => (Some(PyTuple::new(py, [py.None().into_bound(py), tally_items])?.into_any()), status, detail),
            (Err(_), None) => return Err(PyRuntimeError::new_err("cftuv_native: refused stations without an outcome code")),
        };
        let building_ns = nanos(building);
        let applying = Instant::now();
        let changed = apply_log(py, pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);
        Ok((built, status, detail, counts.as_array(), articles, changed, [tables_ns, arguments_ns, compute_ns, building_ns, log_ns, 0, 0, 0]))
    }
}

// --------------------------------------------------------------------------
// the clip stage around the clip (CLIP_STAGE_NATIVE_V1): the seam edges and the triangles of the emitted pieces
// --------------------------------------------------------------------------

/// One emitted polygon of `piece_triangles`: a triangle as it is, or a contour to ear-clip (its points by number in the converted table).
enum Emitted<'py> {
    Triangle(Vec<Bound<'py, PyAny>>),
    Contour(Vec<Bound<'py, PyAny>>, Vec<usize>),
}

fn polygon_keys<'py>(polygon: &Bound<'py, PyAny>) -> Read<Vec<Bound<'py, PyAny>>> {
    if let Ok(tuple) = polygon.cast_exact::<PyTuple>() {
        return Ok(tuple.iter().collect());
    }
    if let Ok(list) = polygon.cast_exact::<PyList>() {
        return Ok(list.iter().collect());
    }
    unsupported("an emitted polygon must be a tuple or a list of keys")
}

impl Host {
    /// `clip.piece_triangles(polygons, points, budget)`: the triangles (keys) of the emitted pieces, a triangle as it is and every other polygon by the exact ear
    /// triangulation of its chart points. The answer is the exact-mesh answer with the list of key triples as the result; a polygon without a triangulation is
    /// status 11 with `("no_piece_triangulation", vertices)`.
    #[allow(clippy::too_many_arguments)]
    pub fn piece_triangles<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        polygons: &Bound<'py, PyAny>,
        points: &Bound<'py, PyAny>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
        forced: Option<Forced>,
    ) -> PyResult<Answer<'py>> {
        let given = budget.map_or([0; 6], |(_, articles)| articles);
        let started = Instant::now();
        let classes = self.classes()?;
        let pool = &classes.pool;
        let read = (|| -> Read<(Vec<Emitted<'py>>, Vec<Point>)> {
            let Ok(faces) = polygons.cast_exact::<PyList>() else {
                return unsupported("polygons must be a list");
            };
            let Ok(points) = points.cast_exact::<PyDict>() else {
                return unsupported("points must be a dict");
            };
            let mut index: HashMap<String, usize, FxBuild> = HashMap::default();
            let mut table: Vec<Point> = Vec::new();
            let mut emitted = Vec::new();
            for face in faces.iter() {
                for polygon in face.try_iter()? {
                    let keys = polygon_keys(&polygon?)?;
                    if keys.len() == 3 {
                        emitted.push(Emitted::Triangle(keys));
                        continue;
                    }
                    let mut numbers = Vec::with_capacity(keys.len());
                    for key in &keys {
                        let name = text_of(key, "a piece key")?;
                        let number = match index.get(&name) {
                            Some(found) => *found,
                            None => {
                                let Some(point) = points.get_item(key)? else {
                                    return unsupported("a piece key without a point (KeyError in the oracle)");
                                };
                                table.push(point_of(pool, &point)?);
                                index.insert(name, table.len() - 1);
                                table.len() - 1
                            }
                        };
                        numbers.push(number);
                    }
                    emitted.push(Emitted::Contour(keys, numbers));
                }
            }
            Ok((emitted, table))
        })();
        let (emitted, table) = match read {
            Ok(read) => read,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(unsupported_answer(py, &text, given)?),
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
        let arguments_ns = nanos(started);
        let computing = Instant::now();
        // per contour: its ears, or the number of its vertices when it has none (the oracle's refusal, after which nothing is asked)
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut counts = SignCounts::default();
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            let mut ears: Vec<Vec<[usize; 3]>> = Vec::new();
            let mut result: Result<Option<usize>, MeshError> = Ok(None);
            for item in &emitted {
                let Emitted::Contour(_, numbers) = item else { continue };
                let contour: Vec<Pt> = numbers.iter().map(|number| (&table[*number].0, &table[*number].1)).collect();
                match cftuv_clip::tessellate::triangulate_exact(&mut ctx, &contour) {
                    Ok(Some(found)) => ears.push(found),
                    Ok(None) => {
                        result = Ok(Some(numbers.len()));
                        break;
                    }
                    Err(error) => {
                        result = Err(MeshError::from(error));
                        break;
                    }
                }
            }
            if let Some(knob) = forced {
                assert!(!knob.panics(), "forced by the test knob");
                if let Some(error) = knob.exact_error() {
                    result = Err(MeshError::Exact(error));
                } else if knob.clip_error().is_some() {
                    result = Err(MeshError::Unsupported("forced by the test knob".to_string()));
                }
            }
            let articles = run.budget_mut().articles();
            (result, ears, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (result, ears, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.piece_triangles: native panic: {}", panic_text(&panic)))),
        };
        let classes = self.classes()?;
        let pool = &classes.pool;
        let (built, status, detail) = match &result {
            Ok(None) => {
                let mut found = Vec::new();
                let mut contours = ears.iter();
                for item in &emitted {
                    match item {
                        Emitted::Triangle(keys) => found.push(PyTuple::new(py, keys)?),
                        Emitted::Contour(keys, _) => {
                            for [a, b, c] in contours.next().expect("one answer per contour") {
                                found.push(PyTuple::new(py, [&keys[*a], &keys[*b], &keys[*c]])?);
                            }
                        }
                    }
                }
                (Some(PyList::new(py, found)?.into_any()), 0, None)
            }
            Ok(Some(vertices)) => {
                let detail = PyTuple::new(py, [PyString::new(py, "no_piece_triangulation").into_any(), int_from_ibig(py, pool, &IBig::from(*vertices as u64))?])?;
                (None, STATUS_REFUSAL, Some(detail))
            }
            Err(error) => {
                let (status, detail) = error_status(py, pool, error, &[])?;
                if is_native_only(status) {
                    return Ok((None, status, detail, [0; 5], given, 0, [0, arguments_ns, compute_ns, 0, 0, 0, 0, 0]));
                }
                (None, status, detail)
            }
        };
        let applying = Instant::now();
        let changed = apply_log(py, pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);
        Ok((built, status, detail, counts.as_array(), articles, changed, [0, arguments_ns, compute_ns, 0, log_ns, 0, 0, 0]))
    }

    /// `clip.seam_edges(frame_faces, polygons, facts, layout, lattice_alpha)` with `regions[i]` the layout region of face `i`: the frozenset of the key pairs of the
    /// boundary edges of the polygons that are not a front (`edge_kind` not `RIM`), built as the oracle builds it (`frozenset` of a set filled in its order), or
    /// `None` when the port declines (a missing fact, a class it does not carry).
    pub fn seam_edges<'py>(&self, py: Python<'py>, polygons: &Bound<'py, PyAny>, regions: &Bound<'py, PyAny>, facts: &Bound<'py, PyAny>, lattice_alpha: &Bound<'py, PyAny>) -> PyResult<Option<Bound<'py, PyAny>>> {
        let pool = &self.classes()?.pool;
        let found = (|| -> Read<Bound<'py, PyAny>> {
            let Ok(faces) = polygons.cast_exact::<PyList>() else {
                return unsupported("polygons must be a list");
            };
            let Ok(regions) = regions.cast_exact::<PyList>() else {
                return unsupported("regions must be a list");
            };
            let Ok(facts) = facts.cast_exact::<PyDict>() else {
                return unsupported("facts must be a dict");
            };
            let front = SqrtSum::rational(&rat_of(pool, lattice_alpha, "lattice_alpha")?);
            // `count` and `owner` by the unordered pair of key numbers, in the order of first insertion
            let mut numbers: HashMap<String, u32, FxBuild> = HashMap::default();
            let mut objects: Vec<Bound<'py, PyAny>> = Vec::new();
            let mut slots: HashMap<(u32, u32), usize, FxBuild> = HashMap::default();
            let mut pairs: Vec<((u32, u32), u32, usize)> = Vec::new();
            // (first oriented edge, count, owner): the pair object of the oracle is the frozenset of the first edge met
            for (index, face) in faces.iter().enumerate() {
                for polygon in face.try_iter()? {
                    let keys = polygon_keys(&polygon?)?;
                    let mut ids = Vec::with_capacity(keys.len());
                    for key in &keys {
                        let name = text_of(key, "a polygon key")?;
                        let id = match numbers.get(&name) {
                            Some(id) => *id,
                            None => {
                                let id = objects.len() as u32;
                                numbers.insert(name, id);
                                objects.push(key.clone());
                                id
                            }
                        };
                        ids.push(id);
                    }
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
            let r_of = |region: &Bound<'py, PyAny>, key: &Bound<'py, PyAny>| -> Read<SqrtSum> {
                let slot = PyTuple::new(py, [region.clone(), key.clone()])?;
                let Some(fact) = facts.get_item(slot)? else {
                    return unsupported("a vertex of a boundary edge without a fact (KeyError in the oracle)");
                };
                let Ok(fact) = fact.cast_exact::<PyTuple>() else {
                    return unsupported("a fact must be a tuple (s, r)");
                };
                if fact.len() != 2 {
                    return unsupported("a fact must be a tuple (s, r)");
                }
                Ok(sum_of(pool, &fact.get_item(1)?, "the r of a fact")?)
            };
            let found = PySet::empty(py)?;
            for ((a, b), number, owner) in &pairs {
                if *number != 1 || a == b {
                    continue;
                }
                let region = regions.get_item(*owner)?;
                let (first, second) = (&objects[*a as usize], &objects[*b as usize]);
                let (ra, rb) = (r_of(&region, first)?, r_of(&region, second)?);
                // `edge_kind`: SOURCE (r = 0 at both ends) and WALL are seams, RIM (r = alpha at both ends) is not
                let rim = !(ra.is_zero() && rb.is_zero()) && ra.sub(&front).is_zero() && rb.sub(&front).is_zero();
                if !rim {
                    found.add(PyFrozenSet::new(py, [first, second])?)?;
                }
            }
            Ok(py.get_type::<PyFrozenSet>().call1((found,))?)
        })();
        match found {
            Ok(found) => Ok(Some(found)),
            Err(Fail::Unsupported(_)) => Ok(None),
            Err(Fail::Py(error)) => Err(error),
        }
    }
}

// --------------------------------------------------------------------------
// the seams of the tessellation of one contour (test-only differential entries)
// --------------------------------------------------------------------------

impl Host {
    /// ONE contour under the polygon law (`kind` 0: `mesh._contour_polygons`; 1: `mesh._fan_faces`) with the `(s, r)` of its vertices handed over as a map: the part seam of the tessellation, whose
    /// oracle takes a `uv_of` callable. The answer is the exact-mesh answer with `(polygons, tally items)` as the result. A test entry: no bound classes are needed beyond `SqrtSumV1` and `Fraction`.
    #[allow(clippy::too_many_arguments)]
    pub fn seam<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        kind: u8,
        keys: &Bound<'py, PyAny>,
        points: &Bound<'py, PyAny>,
        uv: &Bound<'py, PyAny>,
        area: &Bound<'py, PyAny>,
        owner: &Bound<'py, PyAny>,
        reverse: bool,
        exact_plane: bool,
        unit: &Bound<'py, PyAny>,
        in_flow: bool,
        partition: bool,
        affine_known: bool,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
    ) -> PyResult<Answer<'py>> {
        let given = budget.map_or([0; 6], |(_, articles)| articles);
        let classes = self.classes()?;
        let pool = &classes.pool;
        let read = (|| -> Read<(Vec<String>, Vec<Point>, HashMap<u32, (SqrtSum, SqrtSum), FxBuild>, SqrtSum, Option<(IBig, IBig)>, Rat)> {
            let names: Vec<String> = keys.try_iter()?.map(|item| text_of(&item?, "a key")).collect::<Read<_>>()?;
            let mut native = Vec::with_capacity(names.len());
            for item in points.try_iter()? {
                native.push(point_of(pool, &item?)?);
            }
            if native.len() != names.len() {
                return unsupported("keys and points must have one length");
            }
            let Ok(uv) = uv.cast_exact::<PyDict>() else {
                return unsupported("uv must be a dict");
            };
            let mut index: HashMap<String, u32, FxBuild> = HashMap::default();
            for (number, name) in names.iter().enumerate() {
                index.entry(name.clone()).or_insert(number as u32);
            }
            let mut values: HashMap<u32, (SqrtSum, SqrtSum), FxBuild> = HashMap::default();
            for (key, value) in uv.iter() {
                let name = text_of(&key, "a uv key")?;
                let Some(number) = index.get(&name) else { continue };
                values.insert(*number, point_of(pool, &value)?);
            }
            let apex = apex_of(pool, owner)?;
            Ok((names, native, values, sum_of(pool, area, "the doubled area")?, apex, rat_of(pool, unit, "unit")?))
        })();
        let (names, native_points, values, area, apex, unit) = match read {
            Ok(read) => read,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(unsupported_answer(py, &text, given)?),
        };
        // the key numbers are those of the first occurrence of each name (a repeated name is one key, as in the oracle)
        let mut first: HashMap<&str, u32, FxBuild> = HashMap::default();
        let ids: Vec<u32> = names.iter().enumerate().map(|(number, name)| *first.entry(name.as_str()).or_insert(number as u32)).collect();
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
            let mut tally = Tally::new();
            let uv = UvMap { values };
            let cx = PieceContext { uv: &uv, unit: &unit, reverse, exact_plane, partition };
            let whose = cftuv_mesh::error::Whose { face: 0, part: None };
            let contour_points: Vec<Pt> = native_points.iter().map(|point| (&point.0, &point.1)).collect();
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            let result = if kind == 0 {
                let piece = Piece { keys: ids.clone(), points: contour_points, area: &area, whose };
                contour_polygons(&mut ctx, &piece, &cx, &mut tally, in_flow, affine_known)
            } else {
                let face = Face {
                    region: 0,
                    points: Vec::new(),
                    area: area.clone(),
                    apex,
                    parts: Vec::new(),
                    line: Line { a: Rat::zero(), b: Rat::zero(), c: SqrtSum::zero(), q: Rat::one() },
                    run: Rc::new(Run { id: String::new(), origin: (SqrtSum::zero(), SqrtSum::zero()), covector: (Rat::zero(), Rat::zero()), s_origin: SqrtSum::zero(), inverse_length: SqrtSum::zero() }),
                    fan_station: Some(SqrtSum::zero()),
                    in_flow: false,
                };
                let contour = Contour { keys: ids.clone(), points: contour_points };
                fan_faces(&mut ctx, &face, whose, &contour, &cx, &mut tally, affine_known)
            };
            let articles = run.budget_mut().articles();
            (result, tally, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (result, tally, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.seam: native panic: {}", panic_text(&panic)))),
        };
        let keys_out: Vec<cftuv_mesh::data::Key> = names.iter().enumerate().map(|(number, name)| cftuv_mesh::data::Key { name: name.clone(), first: (0, number as u32) }).collect();
        let failure = match &result {
            Ok(_) => None,
            Err(error) => Some(error_status(py, &classes.pool, error, &keys_out)?),
        };
        if let Some((status, detail)) = &failure {
            if is_native_only(*status) {
                return Ok((None, *status, detail.clone(), [0; 5], given, 0, [0, 0, compute_ns, 0, 0, 0, 0, 0]));
            }
        }
        // the tally is the caller's Counter, written in place: what a refusal of the oracle leaves in it stays (the answer carries it either way)
        let mut tally_items = Vec::with_capacity(tally.items().count());
        for (name, value) in tally.items() {
            tally_items.push(PyTuple::new(py, [PyString::new(py, name).into_any(), int_from_ibig(py, &classes.pool, &IBig::from(value))?])?);
        }
        let tally_items = PyTuple::new(py, tally_items)?.into_any();
        let (built, status, detail) = match (&result, failure) {
            (Ok(faces), _) => {
                let key_objects: Vec<Bound<'py, PyAny>> = names.iter().map(|name| PyString::new(py, name).into_any()).collect();
                let mut polygons = Vec::with_capacity(faces.len());
                for polygon in faces {
                    let items: Vec<Bound<'py, PyAny>> = polygon.iter().map(|id| key_objects[*id as usize].clone()).collect();
                    polygons.push(PyTuple::new(py, items)?);
                }
                (Some(PyTuple::new(py, [PyTuple::new(py, polygons)?.into_any(), tally_items])?.into_any()), 0, None)
            }
            (Err(_), Some((status, detail))) => (Some(PyTuple::new(py, [py.None().into_bound(py), tally_items])?.into_any()), status, detail),
            (Err(_), None) => return Err(PyRuntimeError::new_err("cftuv_native: a refused seam without an outcome code")),
        };
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        Ok((built, status, detail, counts.as_array(), articles, changed, [0, 0, compute_ns, 0, 0, 0, 0, 0]))
    }
}

fn unsupported_answer<'py>(py: Python<'py>, text: &str, given: [u64; 6]) -> PyResult<Answer<'py>> {
    Ok((None, STATUS_UNSUPPORTED, Some(PyTuple::new(py, [PyString::new(py, text)])?), [0; 5], given, 0, [0; 8]))
}

/// `ExactMeshV1(cycles, points, facts, polygons, tally, rungs)` from the native answer.
fn build_result<'py>(py: Python<'py>, classes: &Classes, objects: &[Vec<Bound<'py, PyAny>>], out: &MeshOutput) -> PyResult<Bound<'py, PyAny>> {
    let pool = &classes.pool;
    let names = &classes.names;
    let keys = key_objects(py, &out.keys);
    let mut cycles = Vec::with_capacity(out.cycles.len());
    for (held, cycle) in objects.iter().zip(&out.cycles) {
        let mut pairs = Vec::with_capacity(cycle.len());
        for (key, vertex) in cycle {
            pairs.push(PyTuple::new(py, [keys[*key as usize].clone(), held[*vertex as usize].clone()])?);
        }
        cycles.push(PyList::new(py, pairs)?);
    }
    let points = PyDict::new(py);
    for (key, object) in out.keys.iter().zip(&keys) {
        let (face, vertex) = key.first;
        points.set_item(object, objects[face as usize][vertex as usize].clone())?;
    }
    let facts = PyDict::new(py);
    let mut cache: HashMap<usize, Bound<'py, PyAny>, FxBuild> = HashMap::default();
    for ((region, key), value) in &out.facts {
        let slot = PyTuple::new(py, [int_from_ibig(py, pool, &IBig::from(u64::from(*region)))?, keys[*key as usize].clone()])?;
        let address = Rc::as_ptr(value) as usize;
        let pair = match cache.get(&address) {
            Some(pair) => pair.clone(),
            None => {
                let pair = PyTuple::new(py, [sqrt_sum_to_py(py, pool, &value.0)?, sqrt_sum_to_py(py, pool, &value.1)?])?.into_any();
                cache.insert(address, pair.clone());
                pair
            }
        };
        facts.set_item(slot, pair)?;
    }
    let polygons = polygons_list(py, &out.polygons, &keys)?;
    let tally = classes.counter.bind(py).call0()?;
    for (name, value) in out.tally.items() {
        tally.set_item(PyString::new(py, name), int_from_ibig(py, pool, &IBig::from(value))?)?;
    }
    let rungs = PySet::empty(py)?;
    for (region, key) in &out.rungs {
        rungs.add(PyTuple::new(py, [int_from_ibig(py, pool, &IBig::from(u64::from(*region)))?, keys[*key as usize].clone()])?)?;
    }
    let result = alloc(py, &classes.result)?;
    set_slot(&result, &names.cycles, PyList::new(py, cycles)?.as_any())?;
    set_slot(&result, &names.points_slot, points.as_any())?;
    set_slot(&result, &names.facts, facts.as_any())?;
    set_slot(&result, &names.polygons, &polygons)?;
    set_slot(&result, &names.tally, &tally)?;
    set_slot(&result, &names.rungs, rungs.as_any())?;
    Ok(result)
}

// --------------------------------------------------------------------------
// canonical_triangles
// --------------------------------------------------------------------------

impl Host {
    /// One `canonical_triangles(frame_faces, cycles, budget, reverse)`: the ear-clipped triangles of every merged face by keys.
    #[allow(clippy::too_many_arguments)]
    pub fn canonical<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        frame_faces: &Bound<'py, PyAny>,
        cycles: &Bound<'py, PyAny>,
        reverse: bool,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
        forced: Option<Forced>,
    ) -> PyResult<Answer<'py>> {
        let given = budget.map_or([0; 6], |(_, articles)| articles);
        let started = Instant::now();
        let classes = self.classes()?;
        let pool = &classes.pool;
        let read = (|| -> Read<(Vec<SqrtSum>, Vec<Vec<Bound<'py, PyAny>>>, Vec<Vec<Point>>, Vec<Vec<u32>>)> {
            let items = frame_faces.cast::<PyList>().map_err(|_| Fail::Unsupported("frame_faces must be a list".to_string()))?;
            let lists = cycles.cast::<PyList>().map_err(|_| Fail::Unsupported("cycles must be a list".to_string()))?;
            if items.len() != lists.len() {
                return unsupported("frame_faces and cycles must have one length");
            }
            let mut areas = Vec::with_capacity(items.len());
            let mut key_objects: Vec<Bound<'py, PyAny>> = Vec::new();
            let mut index: HashMap<String, u32, FxBuild> = HashMap::default();
            let mut key_lists = Vec::with_capacity(items.len());
            let mut point_lists = Vec::with_capacity(items.len());
            for (frame_face, cycle) in items.iter().zip(lists.iter()) {
                let covered = field(&frame_face, &classes.names.face)?;
                areas.push(sum_of(pool, &field(&covered, &classes.names.doubled_area)?, "the doubled area")?);
                let Ok(cycle) = cycle.cast::<PyList>() else {
                    return unsupported("a cycle must be a list");
                };
                let mut keys = Vec::with_capacity(cycle.len());
                let mut points = Vec::with_capacity(cycle.len());
                for entry in cycle.iter() {
                    let Ok(pair) = entry.cast_exact::<PyTuple>() else {
                        return unsupported("a cycle entry must be (key, point)");
                    };
                    if pair.len() != 2 {
                        return unsupported("a cycle entry must be (key, point)");
                    }
                    let key = pair.get_item(0)?;
                    let name = text_of(&key, "a cycle key")?;
                    let number = match index.get(&name) {
                        Some(found) => *found,
                        None => {
                            let found = key_objects.len() as u32;
                            index.insert(name, found);
                            key_objects.push(key);
                            found
                        }
                    };
                    keys.push(number);
                    points.push(point_of(pool, &pair.get_item(1)?)?);
                }
                key_lists.push(keys);
                point_lists.push(points);
            }
            Ok((areas, vec![key_objects], point_lists, key_lists))
        })();
        let (areas, key_object_lists, point_lists, key_lists) = match read {
            Ok(read) => read,
            Err(Fail::Py(error)) => return Err(error),
            Err(Fail::Unsupported(text)) => return Ok(unsupported_answer(py, &text, given)?),
        };
        let key_objects = &key_object_lists[0];
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
            let contours: Vec<(Vec<u32>, Vec<Pt>)> =
                key_lists.iter().zip(&point_lists).map(|(keys, points)| (keys.clone(), points.iter().map(|point| (&point.0, &point.1)).collect())).collect();
            let area_refs: Vec<&SqrtSum> = areas.iter().collect();
            let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
            let mut result = canonical_triangles(&mut ctx, &contours, &area_refs, reverse);
            if let Some(knob) = forced {
                assert!(!knob.panics(), "forced by the test knob");
                if let Some(error) = knob.exact_error() {
                    result = Err(MeshError::Exact(error));
                } else if knob.clip_error().is_some() {
                    result = Err(MeshError::Unsupported("forced by the test knob".to_string()));
                }
            }
            let articles = run.budget_mut().articles();
            (result, counts, articles, session.memory.take_log())
        }));
        let compute_ns = nanos(computing);
        let (result, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.canonical: native panic: {}", panic_text(&panic)))),
        };
        let failure = match &result {
            Ok(_) => None,
            Err(error) => Some(error_status(py, &classes.pool, error, &[])?),
        };
        if let Some((status, detail)) = &failure {
            if is_native_only(*status) {
                return Ok((None, *status, detail.clone(), [0; 5], given, 0, [0, arguments_ns, compute_ns, 0, 0, 0, 0, 0]));
            }
        }
        let building = Instant::now();
        let (built, status, detail) = match (&result, failure) {
            (Ok(polygons), _) => (Some(polygons_list(py, polygons, key_objects)?), 0, None),
            (Err(_), Some((status, detail))) => (None, status, detail),
            (Err(_), None) => return Err(PyRuntimeError::new_err("cftuv_native: a refused canonical pass without an outcome code")),
        };
        let building_ns = nanos(building);
        let applying = Instant::now();
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);
        Ok((built, status, detail, counts.as_array(), articles, changed, [0, arguments_ns, compute_ns, building_ns, log_ns, 0, 0, 0]))
    }
}
