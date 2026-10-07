//! `skeleton.build_skeleton` behind the persistent session: the whole operation of the skeleton port (WP-S6), at the boundary of the product.
//!
//! What crosses per call, and what does not:
//!
//! * the POLYGON (`PolygonV1`: loops of lattice points with their squared speeds, the vertex fans) is read from the Python objects directly, into the native polygon of the port;
//!   the lattice is the machine range of the port (a coordinate beyond it is a named refusal of the port, never a wrap);
//! * the live bounds of the oracle's module functions (`skeleton.level_budget(polygon)`, the number of march steps when `motorcycle.march_budget` is replaced) arrive as integers: the
//!   shim reads them from the live module, a patched function is therefore seen as the oracle sees it;
//! * the RESULT is built from Rust: `SkeletonV1`, `SkeletonNodeV1`, `EventTimeV1`, `EventPointV1`, `SqrtSumV1`, `Fraction`, `ProofObligationV1`, the enum members and the tuples. A time or a point
//!   that two nodes share is ONE object, as in the oracle; coefficients keep the Python type the oracle holds (`pyobj::sqrt_sum_to_py`);
//! * the memory log of the call (what the canonicalization tables received) is replayed IN PLACE on the process's real tables, in the order Python writes them, as for `clip_geometry`.
//!
//! The answer is `(result or None, status, detail, counts, articles, changed tables, timings, superlevel)`. `status` 0 is ok; the others are the outcome codes of the exact layer (1..7, as
//! `coverage.rs`) and of the skeleton (10 `ValueError`, 12 unsupported by the port, 14 `ZeroDivisorTimeError`, 15 `ParallelSupportLinesError`, 16 `DegenerateEdgeError`, 17 `NegativeSpeedError`,
//! 18 `CellGridRejected`). An ORACLE outcome ([`ORACLE_STATUSES`]) is applied whole: the memory log, the counters and the articles of the call, `superlevel` (the string the oracle writes into the
//! budget at every level), and the shim raises last, as the oracle's exception leaves its partial effects. A refusal of the PORT (5, 6, 7, 12, anything unknown) is applied NOWHERE: the log is dropped,
//! the answer carries zero counts and the articles it was given, and the session forgets its mirror (`lib.rs`); the host can run the oracle on the very same state. The order inside the call is
//! compute (native state only), classify, build the result (new objects only), and only then commit (the memory log on the real tables).
//!
//! The GIL stays HELD while the port computes, as for `clip_geometry`: the real memory tables are replayed from a log made against the state the call started from, so no Python code of this
//! process may mutate them in between (the oracle's own fallback in another thread would), and the thread that waits for `NATIVE_LOCK` waits without it. The shim holds `NATIVE_LOCK` for the whole
//! call, so no other native operation reads or writes the mirror or the tables in that time.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyString, PyTuple};

use cftuv_canon::fxhash::FxBuild;
use cftuv_canon::MemOp;
use cftuv_core::codec::Reader;
use cftuv_core::exact::ExactCtx;
use cftuv_core::rat::Rat;
use cftuv_core::session::{CostRun, Session};
use cftuv_core::sqrt_sum::SignCounts;
use cftuv_skeleton::builder::BuilderOptions;
use cftuv_skeleton::candidate::CandidateRefusal;
use cftuv_skeleton::error::SkelError;
use cftuv_skeleton::polygon::{FanSupport, Loop, Polygon, VertexFan};
use cftuv_skeleton::proof::{ProofBranch, ProofCause, ProofDisposition, ProofObligation, ProofStatus};
use cftuv_skeleton::queue::EventKind;
use cftuv_skeleton::skeleton::{Skeleton, SkeletonNode, SkeletonOutcome};
use cftuv_skeleton::time::{EventPoint, EventTime};
use cftuv_skeleton::transaction::build_skeleton;

use crate::coverage::exact_status;
use crate::memlog::{apply_log, insort, Tables};
use crate::pyobj::{alloc, fraction_from_rat, ibig_from_int, is_exactly, note_attr_build, rat_from_number, refuse, set_slot, sqrt_sum_to_py, Pool, Raw, SlotMode};
use crate::refusal::Forced;

const STATUS_VALUE: u8 = 10;
const STATUS_UNSUPPORTED: u8 = 12;
const STATUS_ZERO_DIVISOR_TIME: u8 = 14;
const STATUS_PARALLEL_LINES: u8 = 15;
const STATUS_DEGENERATE_EDGE: u8 = 16;
const STATUS_NEGATIVE_SPEED: u8 = 17;
const STATUS_CELL_GRID_REJECTED: u8 = 18;

/// The status codes of this operation that are outcomes of the ORACLE: 0 ok, 1 exhaustion, 2 negative radicand, 3 zero divisor, 4 failed reconstruction, 10 `ValueError`, 14 `ZeroDivisorTimeError`,
/// 15 `ParallelSupportLinesError`, 16 `DegenerateEdgeError`, 17 `NegativeSpeedError`, 18 `CellGridRejected`. Every other code (5, 6, 7, 12, an unknown one) is a refusal of the port.
pub const ORACLE_STATUSES: [u8; 11] = [0, 1, 2, 3, 4, STATUS_VALUE, STATUS_ZERO_DIVISOR_TIME, STATUS_PARALLEL_LINES, STATUS_DEGENERATE_EDGE, STATUS_NEGATIVE_SPEED, STATUS_CELL_GRID_REJECTED];

/// A status the oracle has no outcome of THIS operation for.
pub fn is_native_only(status: u8) -> bool {
    !ORACLE_STATUSES.contains(&status)
}

/// `skeleton_oracle_statuses() -> list[int]`: [`ORACLE_STATUSES`], for the shim's table to be compared with.
#[pyfunction]
pub fn skeleton_oracle_statuses() -> Vec<u8> {
    ORACLE_STATUSES.to_vec()
}

/// `(result, status, detail, counts, articles, changed tables, timings, superlevel)`.
pub type Answer<'py> = (Option<Bound<'py, PyAny>>, u8, Option<Bound<'py, PyTuple>>, [u64; 5], [u64; 6], u8, [u64; 4], Option<String>);

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
// the classes of the result
// --------------------------------------------------------------------------

/// A slot class whose instances are built from Rust: the layout probe (see [`Raw`]) and the names of the slots in order, for the fallback of the attribute protocol.
struct Shape {
    class: Py<PyAny>,
    raw: Option<Raw>,
    names: Vec<Py<PyString>>,
}

impl Shape {
    /// `label` names the class in the refusal a forced `raw` mode makes of a layout the probe does not confirm; `Attr` does not probe (see [`Raw::select`]).
    fn new(py: Python<'_>, mode: SlotMode, label: &str, class: &Bound<'_, PyAny>, names: &[&str]) -> PyResult<Shape> {
        let class = class.clone().unbind();
        let names: Vec<Py<PyString>> = names.iter().map(|name| PyString::intern(py, name).unbind()).collect();
        let raw = Raw::select(py, mode, label, &class, &names.iter().collect::<Vec<_>>())?;
        Ok(Shape { class, raw, names })
    }

    /// An instance with every slot given, in the order of the names.
    fn make<'py, const N: usize>(&self, py: Python<'py>, values: [Bound<'py, PyAny>; N]) -> PyResult<Bound<'py, PyAny>> {
        if let Some(raw) = &self.raw {
            return raw.build(py, &self.class, values);
        }
        note_attr_build();
        let object = alloc(py, &self.class)?;
        for (name, value) in self.names.iter().zip(values.iter()) {
            set_slot(&object, name, value)?;
        }
        Ok(object)
    }

    fn raw_engaged(&self) -> bool {
        self.raw.is_some()
    }
}

/// What `bind_skeleton` hands over: the kernel classes the result is built from and the members of the enums it names.
struct Classes {
    pool: Pool,
    skeleton: Shape,
    node: Shape,
    time: Shape,
    point: Shape,
    obligation: Shape,
    outcomes: Vec<Py<PyAny>>,
    kinds: Vec<Py<PyAny>>,
    statuses: Vec<Py<PyAny>>,
    branches: Vec<Py<PyAny>>,
    dispositions: Vec<Py<PyAny>>,
    refusals: Vec<Py<PyAny>>,
    insort: Py<PyAny>,
    names: Names,
}

/// The attributes the polygon is read through.
struct Names {
    loops: Py<PyString>,
    points: Py<PyString>,
    speeds: Py<PyString>,
    fans: Py<PyString>,
    point: Py<PyString>,
    supports: Py<PyString>,
    normal_x: Py<PyString>,
    normal_y: Py<PyString>,
    speed_squared: Py<PyString>,
}

#[derive(Default)]
pub struct Host {
    classes: Option<Classes>,
}

/// The members of an enum, from the dictionary `{value: member}` the shim built, in the order of `values`.
fn members(table: &Bound<'_, PyAny>, values: &[&str], what: &str) -> PyResult<Vec<Py<PyAny>>> {
    values
        .iter()
        .map(|value| match table.get_item(*value) {
            Ok(found) => Ok(found.unbind()),
            Err(_) => Err(refuse(format!("the enum {what} has no member {value:?}: the oracle moved past the port"))),
        })
        .collect()
}

// --------------------------------------------------------------------------
// reading the arguments
// --------------------------------------------------------------------------

fn lattice(pool: &Pool, value: &Bound<'_, PyAny>, what: &str) -> Result<i64, String> {
    if !is_exactly(value, &pool.int_type) {
        return Err(format!("{what} must be an int, not {}", value.get_type().name().map(|name| name.to_string()).unwrap_or_default()));
    }
    let number = ibig_from_int(value).map_err(|error| error.to_string())?;
    i64::try_from(&number).map_err(|_| format!("{what} is beyond the machine range of the port"))
}

fn point_of(pool: &Pool, value: &Bound<'_, PyAny>, what: &str) -> Result<(i64, i64), String> {
    let pair = value.cast::<PyTuple>().map_err(|_| format!("{what} must be a tuple of two ints"))?;
    if pair.len() != 2 {
        return Err(format!("{what} must be a tuple of two ints"));
    }
    let item = |index: usize| pair.get_item(index).map_err(|error| error.to_string());
    Ok((lattice(pool, &item(0)?, what)?, lattice(pool, &item(1)?, what)?))
}

fn speed_of(pool: &Pool, value: &Bound<'_, PyAny>) -> Result<Rat, String> {
    rat_from_number(pool, value).map_err(|error| error.to_string())
}

/// The native polygon of a `PolygonV1` (`loops`, `edge_speeds_squared`, `vertex_fans`); any shape the port does not carry is the text of the refusal.
fn polygon_of(py: Python<'_>, classes: &Classes, polygon: &Bound<'_, PyAny>) -> Result<Polygon, String> {
    let pool = &classes.pool;
    let names = &classes.names;
    fn read<'py>(py: Python<'py>, object: &Bound<'py, PyAny>, name: &Py<PyString>) -> Result<Bound<'py, PyAny>, String> {
        object.getattr(name.bind(py)).map_err(|error| error.to_string())
    }
    fn tuple_of<'py>(value: Bound<'py, PyAny>, what: &str) -> Result<Bound<'py, PyTuple>, String> {
        value.cast_into::<PyTuple>().map_err(|_| format!("{what} must be a tuple"))
    }
    let mut loops = Vec::new();
    for each in tuple_of(read(py, polygon, &names.loops)?, "polygon.loops")?.iter() {
        let points = tuple_of(read(py, &each, &names.points)?, "loop.points")?;
        let speeds = tuple_of(read(py, &each, &names.speeds)?, "loop.edge_speeds_squared")?;
        loops.push(Loop {
            points: points.iter().map(|point| point_of(pool, &point, "a loop point")).collect::<Result<Vec<_>, _>>()?,
            speeds: speeds.iter().map(|speed| speed_of(pool, &speed)).collect::<Result<Vec<_>, _>>()?,
        });
    }
    let mut fans = Vec::new();
    for fan in tuple_of(read(py, polygon, &names.fans)?, "polygon.vertex_fans")?.iter() {
        let point = point_of(pool, &read(py, &fan, &names.point)?, "a fan point")?;
        let mut supports = Vec::new();
        for support in tuple_of(read(py, &fan, &names.supports)?, "fan.supports")?.iter() {
            let normal = (lattice(pool, &read(py, &support, &names.normal_x)?, "a fan normal")?, lattice(pool, &read(py, &support, &names.normal_y)?, "a fan normal")?);
            supports.push(FanSupport { normal, speed: speed_of(pool, &read(py, &support, &names.speed_squared)?)? });
        }
        fans.push(VertexFan { point, supports });
    }
    match Polygon::new(loops, fans) {
        Ok(found) => Ok(found),
        Err(SkelError::Unsupported(text)) => Err(text),
        Err(other) => Err(format!("{other:?}")),
    }
}

// --------------------------------------------------------------------------
// building the result
// --------------------------------------------------------------------------

/// The Python objects of one result. A time or a point shared by several nodes is made once.
struct Maker<'py, 'a> {
    py: Python<'py>,
    classes: &'a Classes,
    times: std::collections::HashMap<usize, Bound<'py, PyAny>, FxBuild>,
    points: std::collections::HashMap<usize, Bound<'py, PyAny>, FxBuild>,
}

impl<'py, 'a> Maker<'py, 'a> {
    fn int(&self, value: i64) -> PyResult<Bound<'py, PyAny>> {
        Ok(value.into_pyobject(self.py)?.into_any())
    }

    fn key(&self, key: &[i64]) -> PyResult<Bound<'py, PyAny>> {
        Ok(PyTuple::new(self.py, key.iter().map(|value| value.into_pyobject(self.py)).collect::<Result<Vec<_>, _>>()?)?.into_any())
    }

    fn keys(&self, keys: &[Vec<i64>]) -> PyResult<Bound<'py, PyAny>> {
        let items = keys.iter().map(|key| self.key(key)).collect::<PyResult<Vec<_>>>()?;
        Ok(PyTuple::new(self.py, items)?.into_any())
    }

    fn time_value(&self, time: &EventTime) -> PyResult<Bound<'py, PyAny>> {
        let pool = &self.classes.pool;
        self.classes.time.make(self.py, [fraction_from_rat(self.py, pool, &time.dividend)?, sqrt_sum_to_py(self.py, pool, &time.divisor)?])
    }

    fn time(&mut self, time: &std::rc::Rc<EventTime>) -> PyResult<Bound<'py, PyAny>> {
        let address = std::rc::Rc::as_ptr(time) as usize;
        if let Some(found) = self.times.get(&address) {
            return Ok(found.clone());
        }
        let made = self.time_value(time)?;
        self.times.insert(address, made.clone());
        Ok(made)
    }

    fn point(&mut self, point: &std::rc::Rc<EventPoint>) -> PyResult<Bound<'py, PyAny>> {
        let address = std::rc::Rc::as_ptr(point) as usize;
        if let Some(found) = self.points.get(&address) {
            return Ok(found.clone());
        }
        let pool = &self.classes.pool;
        let made = self.classes.point.make(self.py, [sqrt_sum_to_py(self.py, pool, &point.x)?, sqrt_sum_to_py(self.py, pool, &point.y)?])?;
        self.points.insert(address, made.clone());
        Ok(made)
    }

    fn kind(&self, kind: EventKind) -> Bound<'py, PyAny> {
        let index = EventKind::ALL.iter().position(|found| *found == kind).unwrap_or(0);
        self.classes.kinds[index].bind(self.py).clone()
    }

    fn kinds(&self, kinds: &[EventKind]) -> PyResult<Bound<'py, PyAny>> {
        Ok(PyTuple::new(self.py, kinds.iter().map(|kind| self.kind(*kind)))?.into_any())
    }

    fn node(&mut self, node: &SkeletonNode) -> PyResult<Bound<'py, PyAny>> {
        let incidences = node.incidences.iter().map(|incidence| self.keys(incidence)).collect::<PyResult<Vec<_>>>()?;
        self.classes.node.make(
            self.py,
            [
                self.kind(node.kind),
                self.time(&node.time)?,
                self.point(&node.point)?,
                self.keys(&node.participants)?,
                self.int(node.converging_vertices)?,
                self.kinds(&node.kinds)?,
                PyTuple::new(self.py, incidences)?.into_any(),
            ],
        )
    }

    fn cause(&self, cause: ProofCause) -> Bound<'py, PyAny> {
        let object = match cause {
            ProofCause::Refusal(reason) => &self.classes.refusals[CandidateRefusal::ALL.iter().position(|found| *found == reason).unwrap_or(0)],
            ProofCause::Branch(branch) => &self.classes.branches[ProofBranch::ALL.iter().position(|found| *found == branch).unwrap_or(0)],
        };
        object.bind(self.py).clone()
    }

    fn obligation(&self, obligation: &ProofObligation) -> PyResult<Bound<'py, PyAny>> {
        let disposition = ProofDisposition::ALL.iter().position(|found| *found == obligation.disposition).unwrap_or(0);
        let ids = PyTuple::new(self.py, obligation.vertex_ids.iter().map(|value| value.into_pyobject(self.py)).collect::<Result<Vec<_>, _>>()?)?.into_any();
        self.classes.obligation.make(
            self.py,
            [
                self.cause(obligation.cause),
                self.classes.dispositions[disposition].bind(self.py).clone(),
                ids,
                self.keys(&obligation.participant_edge_keys)?,
                self.keys(&obligation.target_edge_keys)?,
                self.time_value(&obligation.level)?,
                match obligation.event_kind {
                    Some(kind) => self.kind(kind),
                    None => self.py.None().into_bound(self.py),
                },
            ],
        )
    }

    fn build(&mut self, skeleton: &Skeleton) -> PyResult<Bound<'py, PyAny>> {
        let classes = self.classes;
        let outcome = SkeletonOutcome::ALL.iter().position(|found| *found == skeleton.outcome).unwrap_or(0);
        let status = match skeleton.proof_status {
            ProofStatus::Complete => 0,
            ProofStatus::Incomplete => 1,
        };
        let nodes = skeleton.nodes.iter().map(|node| self.node(node)).collect::<PyResult<Vec<_>>>()?;
        let counters = skeleton
            .counters
            .iter()
            .map(|(name, value)| Ok(PyTuple::new(self.py, [PyString::new(self.py, name).into_any(), self.int(*value)?])?.into_any()))
            .collect::<PyResult<Vec<_>>>()?;
        let obligations = skeleton.proof_obligations.iter().map(|obligation| self.obligation(obligation)).collect::<PyResult<Vec<_>>>()?;
        classes.skeleton.make(
            self.py,
            [
                classes.outcomes[outcome].bind(self.py).clone(),
                PyTuple::new(self.py, nodes)?.into_any(),
                self.int(skeleton.levels)?,
                PyTuple::new(self.py, counters)?.into_any(),
                classes.statuses[status].bind(self.py).clone(),
                PyTuple::new(self.py, obligations)?.into_any(),
            ],
        )
    }
}

// --------------------------------------------------------------------------
// the host
// --------------------------------------------------------------------------

/// The outcome code and detail of a refused skeleton.
fn error_status<'py>(py: Python<'py>, pool: &Pool, error: &SkelError) -> PyResult<(u8, Option<Bound<'py, PyTuple>>)> {
    let text = |value: &str| PyTuple::new(py, [PyString::new(py, value)]).map(Some);
    Ok(match error {
        SkelError::Exact(error) => exact_status(py, pool, error)?,
        SkelError::ZeroDivisorTime => (STATUS_ZERO_DIVISOR_TIME, None),
        SkelError::ParallelSupportLines => (STATUS_PARALLEL_LINES, None),
        SkelError::DegenerateEdge(message) => (STATUS_DEGENERATE_EDGE, text(message)?),
        SkelError::NegativeSpeed(message) => (STATUS_NEGATIVE_SPEED, text(message)?),
        SkelError::CellGridRejected(message) => (STATUS_CELL_GRID_REJECTED, text(message)?),
        SkelError::Value(message) => (STATUS_VALUE, text(message)?),
        SkelError::Unsupported(message) => (STATUS_UNSUPPORTED, text(message)?),
    })
}

/// What the computation hands back: the result or the refusal, the sign counters, the articles and the memory log of the call, the string the oracle would have written into the budget.
type Ran = (Result<Skeleton, SkelError>, SignCounts, [u64; 6], Vec<MemOp>, String);

impl Host {
    #[allow(clippy::too_many_arguments)]
    pub fn bind(
        &mut self,
        py: Python<'_>,
        sqrt_sum: &Bound<'_, PyAny>,
        fraction: &Bound<'_, PyAny>,
        classes: [&Bound<'_, PyAny>; 5],
        enums: [&Bound<'_, PyAny>; 6],
        mode: SlotMode,
    ) -> PyResult<()> {
        let [skeleton, node, time, point, obligation] = classes;
        let [outcomes, kinds, statuses, branches, dispositions, refusals] = enums;
        let intern = |text: &str| PyString::intern(py, text).unbind();
        self.classes = Some(Classes {
            pool: Pool::new(py, fraction, sqrt_sum, mode)?,
            skeleton: Shape::new(py, mode, "SkeletonV1", skeleton, &["outcome", "nodes", "levels", "counters", "proof_status", "proof_obligations"])?,
            node: Shape::new(py, mode, "SkeletonNodeV1", node, &["kind", "time", "point", "participants", "converging_vertices", "kinds", "incidences"])?,
            time: Shape::new(py, mode, "EventTimeV1", time, &["dividend", "divisor"])?,
            point: Shape::new(py, mode, "EventPointV1", point, &["x", "y"])?,
            obligation: Shape::new(py, mode, "ProofObligationV1", obligation, &["cause", "disposition", "vertex_ids", "participant_edge_keys", "target_edge_keys", "level", "event_kind"])?,
            outcomes: members(outcomes, &SkeletonOutcome::ALL.map(SkeletonOutcome::value), "SkeletonOutcome")?,
            kinds: members(kinds, &EventKind::ALL.map(EventKind::value), "EventKind")?,
            statuses: members(statuses, &[ProofStatus::Complete.value(), ProofStatus::Incomplete.value()], "ProofStatus")?,
            branches: members(branches, &ProofBranch::ALL.map(ProofBranch::value), "ProofObligationBranch")?,
            dispositions: members(dispositions, &ProofDisposition::ALL.map(ProofDisposition::value), "ProofObligationDisposition")?,
            refusals: members(refusals, &CandidateRefusal::ALL.map(CandidateRefusal::value), "CandidateRefusal")?,
            insort: insort(py)?,
            names: Names {
                loops: intern("loops"),
                points: intern("points"),
                speeds: intern("edge_speeds_squared"),
                fans: intern("vertex_fans"),
                point: intern("point"),
                supports: intern("supports"),
                normal_x: intern("normal_x"),
                normal_y: intern("normal_y"),
                speed_squared: intern("speed_squared"),
            },
        });
        Ok(())
    }

    fn classes(&self) -> PyResult<&Classes> {
        self.classes.as_ref().ok_or_else(|| PyRuntimeError::new_err("cftuv_native: the skeleton classes were not bound (`bind_skeleton`)"))
    }

    /// `(SkeletonV1, SkeletonNodeV1, EventTimeV1, EventPointV1, ProofObligationV1)` built by the raw layout (a layout the probe did not confirm is built through the attribute protocol).
    pub fn raw_layouts(&self) -> Option<[bool; 5]> {
        self.classes.as_ref().map(|classes| [classes.skeleton.raw_engaged(), classes.node.raw_engaged(), classes.time.raw_engaged(), classes.point.raw_engaged(), classes.obligation.raw_engaged()])
    }

    /// Test-only: the attribute protocol everywhere (the fallback of the raw access).
    pub fn disable_raw(&mut self) {
        if let Some(classes) = self.classes.as_mut() {
            classes.pool.disable_raw();
            for shape in [&mut classes.skeleton, &mut classes.node, &mut classes.time, &mut classes.point, &mut classes.obligation] {
                shape.raw = None;
            }
        }
    }

    /// One `build_skeleton`: see the module note. `exhaustive` is `split_search is EXHAUSTIVE`, `level_limit` is the live `skeleton.level_budget(polygon)`, `march_steps` the live number of march steps when the oracle's
    /// `march_budget` was replaced (`None`: the declared one), `sync` the memory sync (`None`: unchanged), `budget` the cap and the six articles (`None`: no budget).
    #[allow(clippy::too_many_arguments)]
    pub fn build_skeleton<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        polygon: &Bound<'py, PyAny>,
        dense_hydration: bool,
        exhaustive: bool,
        level_limit: i64,
        march_steps: Option<i64>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: &Tables<'py>,
        forced: Option<Forced>,
    ) -> PyResult<Answer<'py>> {
        let started = Instant::now();
        let classes = self.classes()?;
        let (cap, articles) = match budget {
            Some((cap, articles)) => (cap, Some(articles)),
            None => (None, None),
        };
        // what a refusal of the port answers for the articles: the ones it was given (nothing was spent as far as the host is concerned)
        let given = articles.unwrap_or([0; 6]);
        let refused = |text: String, arguments_ns: u64| -> PyResult<Answer<'py>> {
            Ok((None, STATUS_UNSUPPORTED, Some(PyTuple::new(py, [PyString::new(py, &text)])?), [0; 5], given, 0, [arguments_ns, 0, 0, 0], None))
        };
        let native = match polygon_of(py, classes, polygon) {
            Ok(found) => found,
            Err(text) => return refused(text, nanos(started)),
        };
        let sync_value = match sync {
            Some(bytes) => Some(Reader::new(bytes, true).get_value().map_err(|error| PyValueError::new_err(format!("the memory sync is not in the wire format: {error}")))?),
            None => None,
        };
        let mut run = CostRun::begin_parts(session, sync_value.as_ref(), cap, articles).map_err(|error| PyValueError::new_err(error.to_string()))?;
        let budgeted = articles.is_some();
        let options = BuilderOptions { dense_hydration, march_steps, budgeted, exhaustive };
        let arguments_ns = nanos(started);

        let computing = Instant::now();
        let outcome = {
            catch_unwind(AssertUnwindSafe(|| {
                let mut counts = SignCounts::default();
                let mut ran = {
                    let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
                    build_skeleton(&mut ctx, native, options, level_limit)
                };
                if let Some(knob) = forced {
                    // the test knob: the computation DID run (the call has effects to drop), then the port refuses
                    assert!(!knob.panics(), "forced by the test knob");
                    if let Some(error) = knob.exact_error() {
                        ran = Err(SkelError::Exact(error));
                    } else if matches!(knob, Forced::Unsupported) {
                        ran = Err(SkelError::Unsupported("forced by the test knob".to_string()));
                    }
                }
                let articles = run.budget_mut().articles();
                let superlevel = run.budget_mut().superlevel.clone();
                (ran, counts, articles, session.memory.take_log(), superlevel)
            }))
        };
        let compute_ns = nanos(computing);
        let (ran, counts, articles, log, superlevel): Ran = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.build_skeleton: native panic: {}", panic_text(&panic)))),
        };

        // Classify first: a refusal of the port is applied nowhere (module note), so nothing below it may have run.
        let failure = match &ran {
            Ok(_) => None,
            Err(error) => Some(error_status(py, &classes.pool, error)?),
        };
        if let Some((status, detail)) = &failure {
            if is_native_only(*status) {
                return Ok((None, *status, detail.clone(), [0; 5], given, 0, [arguments_ns, compute_ns, 0, 0], None));
            }
        }

        // Build everything the host receives: new objects only, no host object is written yet.
        let building = Instant::now();
        let (result, status, detail) = match (&ran, failure) {
            (Ok(skeleton), _) => {
                let mut maker = Maker { py, classes, times: Default::default(), points: Default::default() };
                (Some(maker.build(skeleton)?), 0, None)
            }
            (Err(_), Some((status, detail))) => (None, status, detail),
            (Err(_), None) => return Err(refuse("a refused skeleton without an outcome code")),
        };
        let result_ns = nanos(building);

        // Commit: the memory log on the real tables (what the oracle leaves behind, also when it raised afterwards).
        let applying = Instant::now();
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);
        // the string the oracle writes into the named budget at every level: only a budgeted call has one, and only a call that reached a level
        let written = (budgeted && !superlevel.is_empty()).then_some(superlevel);
        Ok((result, status, detail, counts.as_array(), articles, changed, [arguments_ns, compute_ns, result_ns, log_ns], written))
    }
}
