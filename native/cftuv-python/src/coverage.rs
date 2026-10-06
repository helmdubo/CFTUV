//! `coverage._coverage_at` behind the persistent session: partitions converted ONCE and kept by identity, the
//! result built from Rust, the store and the cost answer exchanged with the shim.
//!
//! A `FacePartitionV1` is a frozen record, so `id(partition)` plus a strong reference identifies its content for as
//! long as the entry lives; the session keeps at most [`PARTITION_LIMIT`] partitions (least recently used out). The
//! same holds for the `(universe, delta, price, memory)` records of the store (`id` of the tuple the store hands back). Per call
//! only `alpha`, the partition handle, the cost header (memory sync + budget) and the store cross the boundary.
//!
//! The answer is `(result, status, detail, counts, articles, changed, timings)`: `result` is the `CoverageV1` (or `None`
//! when the arithmetic refused), `status` and `detail` are the outcome codes of `session.rs` (0 ok; the shim raises
//! the exception of a refusal after it applied every effect), `counts` the five sign-counter deltas, `articles` the
//! six budget articles after the call, `changed` the bits of the memory tables the call changed (its memory log was
//! replayed on the host's real tables here, in the one crossing, see `memlog.rs`), `timings` are nanoseconds
//! `(prepare, arguments, compute, result, memory log)` measured here.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Arc;
use std::time::Instant;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyTuple};

use cftuv_canon::{CanonError, Operation, Pairs, UniverseRecord};
use cftuv_core::codec::Reader;
use cftuv_core::coverage::{self, Answer, Area, Clipped, CoverageError, Face, Line, Partition, Vertex};
use cftuv_core::exact::{ExactCtx, ExactError, UniverseStore};
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::session::{CostRun, Session};
use cftuv_core::sqrt_sum::SignCounts;

use crate::memlog::{apply_log, insort, Tables};
use crate::pyobj::{self, alloc, int_from_ibig, int_from_ubig, refuse, set_slot, sqrt_sum_from_py, sqrt_sum_to_py, Pool, Raw};

/// Partitions kept converted per session.
pub const PARTITION_LIMIT: usize = 64;
/// `(universe, delta, price, memory)` store records kept converted per session.
pub const RECORD_LIMIT: usize = 16;
/// `status` of the cost answer for a face without a supporting line (the shim raises `ValueError`).
const STATUS_MISSING_LINE: u64 = 8;

struct Names {
    outcome: Py<pyo3::types::PyString>,
    alpha: Py<pyo3::types::PyString>,
    faces: Py<pyo3::types::PyString>,
    doubled_area: Py<pyo3::types::PyString>,
    polygon_doubled_area: Py<pyo3::types::PyString>,
    detail: Py<pyo3::types::PyString>,
    work_budget: Py<pyo3::types::PyString>,
    owner: Py<pyo3::types::PyString>,
    points: Py<pyo3::types::PyString>,
}

/// What `bind_coverage` hands over: the kernel classes and enum members this module builds and recognises.
struct Classes {
    pool: Pool,
    coverage: Py<PyAny>,
    face_coverage: Py<PyAny>,
    /// Where the three slots of a `FaceCoverageV1` (owner, points, doubled_area) sit (see [`Raw`]).
    face_raw: Option<Raw>,
    outcome_exact: Py<PyAny>,
    outcome_not_exact: Py<PyAny>,
    outcome_negative: Py<PyAny>,
    face_exact: Py<PyAny>,
    /// `cost.StoreKey`: the lookup key of a store, whose hash is taken once (see `prepare_lookup`).
    store_key: Py<PyAny>,
    /// `FactorizationMemoryDeltaV1`: the fourth part of a store record.
    memory_delta: Py<PyAny>,
    /// `bisect.insort`, for the replay of the memory log (see `memlog.rs`).
    insort: Py<PyAny>,
    empty_str: Py<PyAny>,
    names: Names,
}

/// The Python objects of one face that a result reuses.
struct PyFace {
    owner: Py<PyAny>,
    /// The original `points` tuple (what an unclipped face returns, by identity).
    points: Py<PyAny>,
    /// The original point tuples (what a clipped contour keeps, by identity).
    point_items: Vec<Py<PyAny>>,
    /// The `SqrtSumV1` of the unclipped doubled area, built once.
    area: Option<Py<PyAny>>,
}

struct Prepared {
    id: usize,
    stamp: u64,
    /// Keeps `id` valid (the identity of a frozen record is its content).
    _partition: Py<PyAny>,
    core: Arc<Partition>,
    faces: Vec<PyFace>,
    polygon_area: Py<PyAny>,
    /// `("prime-universe", (Fraction(q), ...))`: the store key, equal to the one `prime_universe_remembered` builds.
    key: Py<PyAny>,
    /// The same key as a `cost.StoreKey` (hash cached): what a lookup in a plain `dict` store presents; `None` when there is no key.
    lookup: Option<Py<PyAny>>,
}

struct Remembered {
    id: usize,
    stamp: u64,
    /// Keeps `id` valid.
    _holder: Py<PyAny>,
    record: Arc<UniverseRecord>,
}

/// `(result, status, detail, counts, articles, changed tables, timings)`.
pub type Answer7<'py> = (Option<Bound<'py, PyAny>>, u8, Option<Bound<'py, PyTuple>>, [u64; 5], [u64; 6], u8, [u64; 5]);

#[derive(Default)]
pub struct Host {
    classes: Option<Classes>,
    prepared: Vec<Prepared>,
    records: Vec<Remembered>,
    clock: u64,
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

impl Host {
    #[allow(clippy::too_many_arguments)]
    pub fn bind(
        &mut self,
        py: Python<'_>,
        sqrt_sum: &Bound<'_, PyAny>,
        fraction: &Bound<'_, PyAny>,
        coverage: &Bound<'_, PyAny>,
        face_coverage: &Bound<'_, PyAny>,
        outcomes: [&Bound<'_, PyAny>; 3],
        face_exact: &Bound<'_, PyAny>,
        store_key: &Bound<'_, PyAny>,
        memory_delta: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let intern = |text: &str| pyo3::types::PyString::intern(py, text).unbind();
        let face_coverage = face_coverage.clone().unbind();
        let (owner, points, doubled_area) = (intern("owner"), intern("points"), intern("doubled_area"));
        let face_raw = Raw::probe(py, &face_coverage, &[&owner, &points, &doubled_area])?;
        self.classes = Some(Classes {
            pool: Pool::new(py, fraction, sqrt_sum)?,
            coverage: coverage.clone().unbind(),
            face_coverage,
            face_raw,
            outcome_exact: outcomes[0].clone().unbind(),
            outcome_not_exact: outcomes[1].clone().unbind(),
            outcome_negative: outcomes[2].clone().unbind(),
            face_exact: face_exact.clone().unbind(),
            store_key: store_key.clone().unbind(),
            memory_delta: memory_delta.clone().unbind(),
            insort: insort(py)?,
            empty_str: pyo3::types::PyString::intern(py, "").into_any().unbind(),
            names: Names {
                outcome: intern("outcome"),
                alpha: intern("alpha"),
                faces: intern("faces"),
                doubled_area: intern("doubled_area"),
                polygon_doubled_area: intern("polygon_doubled_area"),
                detail: intern("detail"),
                work_budget: intern("work_budget"),
                owner: intern("owner"),
                points: intern("points"),
            },
        });
        self.prepared.clear();
        self.records.clear();
        Ok(())
    }

    fn classes(&self) -> PyResult<&Classes> {
        self.classes.as_ref().ok_or_else(|| PyRuntimeError::new_err("cftuv_native: the coverage classes were not bound (`bind_coverage`)"))
    }

    /// `(Fraction, SqrtSumV1, FaceCoverageV1)` slots read and written raw; `None` before `bind_coverage`.
    pub fn raw_layouts(&self) -> Option<(bool, bool, bool)> {
        self.classes.as_ref().map(|classes| {
            let (fraction, sqrt_sum) = classes.pool.raw_layouts();
            (fraction, sqrt_sum, classes.face_raw.is_some())
        })
    }

    /// Test-only: the attribute protocol everywhere (and for the partitions converted from now on).
    pub fn disable_raw(&mut self) {
        if let Some(classes) = self.classes.as_mut() {
            classes.pool.disable_raw();
            classes.face_raw = None;
        }
    }

    /// Test-only: `Fraction`/`SqrtSumV1` through the conversions of the boundary and back.
    pub fn round_trip<'py>(&self, py: Python<'py>, value: &Bound<'py, PyAny>, sum: bool) -> PyResult<Bound<'py, PyAny>> {
        let pool = &self.classes()?.pool;
        if sum {
            sqrt_sum_to_py(py, pool, &sqrt_sum_from_py(pool, value)?)
        } else {
            pyobj::fraction_from_rat(py, pool, &pyobj::rat_from_number(pool, value)?)
        }
    }

    pub fn cache_sizes(&self) -> (usize, usize) {
        (self.prepared.len(), self.records.len())
    }

    pub fn forget(&mut self) {
        self.prepared.clear();
        self.records.clear();
    }

    // ---- the two refusals --------------------------------------------------------------------------------------

    /// `CoverageV1(PARTITION_IS_NOT_EXACT / ALPHA_IS_NEGATIVE, alpha, (), zero, polygon_doubled_area, detail)`.
    pub fn refused<'py>(&self, py: Python<'py>, partition: &Bound<'py, PyAny>, alpha: &Bound<'py, PyAny>, negative: bool) -> PyResult<Bound<'py, PyAny>> {
        let classes = self.classes()?;
        let (outcome, detail) = if negative {
            (&classes.outcome_negative, alpha.str()?.into_any())
        } else {
            (&classes.outcome_not_exact, partition.getattr("outcome")?.getattr("value")?)
        };
        let area = partition.getattr("polygon_doubled_area")?;
        let result = alloc(py, &classes.coverage)?;
        let names = &classes.names;
        set_slot(&result, &names.outcome, outcome.bind(py))?;
        set_slot(&result, &names.alpha, alpha)?;
        set_slot(&result, &names.faces, PyTuple::empty(py).as_any())?;
        set_slot(&result, &names.doubled_area, classes.pool.zero_sum.bind(py))?;
        set_slot(&result, &names.polygon_doubled_area, &area)?;
        set_slot(&result, &names.detail, &detail)?;
        set_slot(&result, &names.work_budget, py.None().bind(py))?;
        Ok(result)
    }

    // ---- partitions --------------------------------------------------------------------------------------------

    /// Index of the prepared partition (converting it first when the session has not seen this object) and whether
    /// this call converted it.
    fn prepare(&mut self, py: Python<'_>, partition: &Bound<'_, PyAny>) -> PyResult<(usize, bool)> {
        let id = partition.as_ptr() as usize;
        self.clock += 1;
        if let Some(index) = self.prepared.iter().position(|entry| entry.id == id) {
            self.prepared[index].stamp = self.clock;
            return Ok((index, false));
        }
        let entry = convert_partition(py, self.classes()?, partition, id, self.clock)?;
        if self.prepared.len() >= PARTITION_LIMIT {
            if let Some(oldest) = self.prepared.iter().enumerate().min_by_key(|(_, entry)| entry.stamp).map(|(index, _)| index) {
                self.prepared.swap_remove(oldest);
            }
        }
        self.prepared.push(entry);
        Ok((self.prepared.len() - 1, true))
    }

    // ---- store records -----------------------------------------------------------------------------------------

    /// The record a store holds under the key, converted once (by the identity of its tuple); `None` for a tuple that is not a 4-tuple: the
    /// oracle's `len(found) != 4` is a miss, and the miss replaces it.
    fn remembered(&mut self, found: &Bound<'_, PyAny>) -> PyResult<Option<Arc<UniverseRecord>>> {
        let id = found.as_ptr() as usize;
        self.clock += 1;
        if let Some(entry) = self.records.iter_mut().find(|entry| entry.id == id) {
            entry.stamp = self.clock;
            return Ok(Some(entry.record.clone()));
        }
        let Some(record) = record_from_py(found)? else {
            return Ok(None);
        };
        let record = Arc::new(record);
        self.remember(found, record.clone());
        Ok(Some(record))
    }

    fn remember(&mut self, holder: &Bound<'_, PyAny>, record: Arc<UniverseRecord>) {
        if self.records.len() >= RECORD_LIMIT {
            if let Some(oldest) = self.records.iter().enumerate().min_by_key(|(_, entry)| entry.stamp).map(|(index, _)| index) {
                self.records.swap_remove(oldest);
            }
        }
        self.records.push(Remembered { id: holder.as_ptr() as usize, stamp: self.clock, _holder: holder.clone().unbind(), record });
    }

    // ---- the call ----------------------------------------------------------------------------------------------

    /// One `_coverage_at` on an exact partition and a non-negative `alpha`: see the module note. `sync` is the memory
    /// sync in the wire format (`None`: unchanged), `budget` the cap and the six articles (`None`: no budget).
    #[allow(clippy::too_many_arguments)]
    pub fn coverage_at<'py>(
        &mut self,
        py: Python<'py>,
        session: &mut Session,
        partition: &Bound<'py, PyAny>,
        alpha: &Bound<'py, PyAny>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        store: Option<&Bound<'py, PyAny>>,
        work_budget: &Bound<'py, PyAny>,
        tables: &Tables<'py>,
        traces: Option<&Bound<'py, PyList>>,
    ) -> PyResult<Answer7<'py>> {
        let started = Instant::now();
        let (index, converted) = self.prepare(py, partition)?;
        let prepare_ns = if converted { nanos(started) } else { 0 };
        let began = Instant::now();
        let alpha_rat = pyobj::rat_from_number(&self.classes()?.pool, alpha)?;
        let (core, key, lookup) = (self.prepared[index].core.clone(), self.prepared[index].key.clone_ref(py), self.prepared[index].lookup.as_ref().map(|lookup| lookup.clone_ref(py)));
        // The store: absent, a miss (nothing under the key) or a hit (the record, converted once).
        let mut hit: Option<Arc<UniverseRecord>> = None;
        let mut miss = false;
        if let Some(store) = store {
            let found = match store.cast_exact::<PyDict>() {
                Ok(dict) => dict.get_item(lookup.as_ref().unwrap_or(&key).bind(py))?,
                Err(_) => {
                    let found = store.call_method1("get", (key.bind(py),))?;
                    if found.is_none() { None } else { Some(found) }
                }
            };
            if let Some(found) = found.filter(|found| !found.is_none()) {
                hit = self.remembered(&found)?;
            }
            miss = hit.is_none();
        }
        let sync_value = match sync {
            Some(bytes) => Some(Reader::new(bytes, true).get_value().map_err(|error| PyValueError::new_err(format!("the memory sync is not in the wire format: {error}")))?),
            None => None,
        };
        let budgeted = budget.is_some();
        let traced = traces.is_some();
        let (cap, articles) = match budget {
            Some((cap, articles)) => (cap, Some(articles)),
            None => (None, None),
        };
        let mut run = CostRun::begin_parts(session, sync_value.as_ref(), cap, articles).map_err(|error| PyValueError::new_err(error.to_string()))?;
        let arguments_ns = nanos(began);

        let computing = Instant::now();
        let outcome = py.detach(|| {
            catch_unwind(AssertUnwindSafe(|| {
                let mut counts = SignCounts::default();
                let run_result = {
                    let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
                    let universe = match (&hit, miss) {
                        (Some(record), _) => UniverseStore::Hit(record),
                        (None, true) => UniverseStore::Miss,
                        (None, false) => UniverseStore::Absent,
                    };
                    coverage::coverage_at(&mut ctx, &core, &alpha_rat, universe, budgeted, traced)
                };
                let articles = run.budget_mut().articles();
                (run_result, counts, articles, session.memory.take_log())
            }))
        });
        let compute_ns = nanos(computing);
        let (run_result, counts, articles, log) = match outcome {
            Ok(done) => done,
            Err(panic) => return Err(PyRuntimeError::new_err(format!("Session.coverage_at: native panic: {}", panic_text(&panic)))),
        };

        let applying = Instant::now();
        let classes = self.classes()?;
        let changed = apply_log(py, &classes.pool, &classes.insort, tables, log)?;
        let log_ns = nanos(applying);

        let building = Instant::now();
        let result = self.finish(py, &run_result, index, &key, store, alpha, work_budget, traces)?;
        let (status, detail) = self.status_of(py, &run_result.outcome)?;
        Ok((result, status, detail, counts.as_array(), articles, changed, [prepare_ns, arguments_ns, compute_ns, nanos(building), log_ns]))
    }

    /// The outcome code and detail of a run (`cost.OpResult`): 0 ok, 1 exhaustion `(operation index, radicand)`, 2 negative
    /// radicand `(numerator, denominator)`, 3 zero divisor, 4 failed reconstruction `(radicand,)`, 6 diverged, 7 internal, 8 a
    /// face without a line `(face,)`.
    fn status_of<'py>(&self, py: Python<'py>, outcome: &Result<Answer, CoverageError>) -> PyResult<(u8, Option<Bound<'py, PyTuple>>)> {
        let pool = &self.classes()?.pool;
        Ok(match outcome {
            Ok(_) => (0, None),
            Err(CoverageError::MissingLine { face }) => (STATUS_MISSING_LINE as u8, Some(PyTuple::new(py, [int_from_ibig(py, pool, &IBig::from(*face as u64))?])?)),
            Err(CoverageError::Exact(error)) => exact_status(py, pool, error)?,
        })
    }

    /// Writes the miss record into the store (the oracle does it before the faces are touched) and builds the result.
    #[allow(clippy::too_many_arguments)]
    fn finish<'py>(
        &mut self,
        py: Python<'py>,
        run: &coverage::Run,
        index: usize,
        key: &Py<PyAny>,
        store: Option<&Bound<'py, PyAny>>,
        alpha: &Bound<'py, PyAny>,
        work_budget: &Bound<'py, PyAny>,
        traces: Option<&Bound<'py, PyList>>,
    ) -> PyResult<Option<Bound<'py, PyAny>>> {
        if let (Some(record), Some(store)) = (&run.record, store) {
            let stored = record_to_py(py, self.classes()?, record)?;
            store.set_item(key.bind(py), &stored)?;
            self.remember(&stored, Arc::new(record.clone()));
        }
        if let Some(list) = traces {
            // the oracle's `traces.append((signs, values))`, one per face whose signs were computed, whatever happened after
            let pool = &self.classes()?.pool;
            for trace in &run.traces {
                let signs = PyList::new(py, trace.signs.iter().map(|sign| i64::from(*sign)))?;
                let values: Vec<Bound<'py, PyAny>> = trace.values.iter().map(|value| sqrt_sum_to_py(py, pool, value)).collect::<PyResult<_>>()?;
                list.append(PyTuple::new(py, [signs.into_any(), PyList::new(py, values)?.into_any()])?)?;
            }
        }
        let Ok(Answer::Exact { faces, total }) = &run.outcome else {
            return Ok(None);
        };
        let classes = self.classes.as_ref().ok_or_else(|| refuse("the coverage classes were not bound"))?;
        let prepared = &mut self.prepared[index];
        let pool = &classes.pool;
        let names = &classes.names;
        let mut covered = Vec::with_capacity(faces.len());
        for (position, out) in faces.iter().enumerate() {
            let face = &mut prepared.faces[position];
            let points = match &out.clipped {
                Clipped::Unchanged => face.points.bind(py).clone(),
                Clipped::Empty => PyTuple::empty(py).into_any(),
                Clipped::Polygon(vertices) => {
                    let mut items = Vec::with_capacity(vertices.len());
                    for vertex in vertices {
                        items.push(match vertex {
                            Vertex::Kept(original) => face.point_items[*original].bind(py).clone(),
                            Vertex::Cut(point) => PyTuple::new(py, [sqrt_sum_to_py(py, pool, &point.0)?, sqrt_sum_to_py(py, pool, &point.1)?])?.into_any(),
                        });
                    }
                    PyTuple::new(py, items)?.into_any()
                }
            };
            let doubled = match &out.area {
                Area::Original => {
                    if face.area.is_none() {
                        let original = prepared.core.faces()[position].cached_area().ok_or_else(|| refuse("the unclipped area was not computed"))?;
                        face.area = Some(sqrt_sum_to_py(py, pool, original)?.unbind());
                    }
                    face.area.as_ref().map(|area| area.bind(py).clone()).ok_or_else(|| refuse("the unclipped area is missing"))?
                }
                Area::Fresh(value) => sqrt_sum_to_py(py, pool, value)?,
            };
            let item = match &classes.face_raw {
                Some(raw) => raw.build(py, &classes.face_coverage, [face.owner.bind(py).clone(), points, doubled])?,
                None => {
                    let item = alloc(py, &classes.face_coverage)?;
                    set_slot(&item, &names.owner, face.owner.bind(py))?;
                    set_slot(&item, &names.points, &points)?;
                    set_slot(&item, &names.doubled_area, &doubled)?;
                    item
                }
            };
            covered.push(item);
        }
        let result = alloc(py, &classes.coverage)?;
        set_slot(&result, &names.outcome, classes.outcome_exact.bind(py))?;
        set_slot(&result, &names.alpha, alpha)?;
        set_slot(&result, &names.faces, PyTuple::new(py, covered)?.as_any())?;
        set_slot(&result, &names.doubled_area, &sqrt_sum_to_py(py, pool, total)?)?;
        set_slot(&result, &names.polygon_doubled_area, prepared.polygon_area.bind(py))?;
        set_slot(&result, &names.detail, classes.empty_str.bind(py))?;
        set_slot(&result, &names.work_budget, work_budget)?;
        Ok(Some(result))
    }
}

/// The outcome code and detail of an exact-layer refusal (the codes of `cost.OpResult`): 1 exhaustion `(operation index,
/// radicand)`, 2 negative radicand `(numerator, denominator)`, 3 zero divisor, 4 failed reconstruction `(radicand,)`, 5 invalid
/// input, 6 diverged, 7 internal. Shared by the whole operations that spend a budget.
pub(crate) fn exact_status<'py>(py: Python<'py>, pool: &Pool, error: &ExactError) -> PyResult<(u8, Option<Bound<'py, PyTuple>>)> {
    let pair = |first: Bound<'py, PyAny>, second: Bound<'py, PyAny>| PyTuple::new(py, [first, second]).map(Some);
    Ok(match error {
        ExactError::Canon(CanonError::Exhausted(exhausted)) => {
            let operation = Operation::ALL.iter().position(|operation| *operation == exhausted.operation).unwrap_or(0);
            (1, pair(int_from_ibig(py, pool, &IBig::from(operation as u64))?, int_from_ubig(py, pool, &exhausted.radicand)?)?)
        }
        ExactError::Canon(CanonError::NegativeRadicand { numerator, denominator }) => (2, pair(int_from_ibig(py, pool, numerator)?, int_from_ubig(py, pool, denominator)?)?),
        ExactError::ZeroDivisor => (3, None),
        ExactError::Canon(CanonError::ReconstructionFailed { radicand }) => (4, Some(PyTuple::new(py, [int_from_ubig(py, pool, radicand)?])?)),
        ExactError::Canon(CanonError::InvalidInput(_)) => (5, None),
        ExactError::Diverged => (6, None),
        ExactError::Internal(_) => (7, None),
    })
}

// --------------------------------------------------------------------------
// converting a partition once
// --------------------------------------------------------------------------

fn convert_partition(py: Python<'_>, classes: &Classes, partition: &Bound<'_, PyAny>, id: usize, stamp: u64) -> PyResult<Prepared> {
    let pool = &classes.pool;
    let outcome = partition.getattr(pyo3::intern!(py, "outcome"))?;
    if !outcome.is(classes.face_exact.bind(py)) {
        return Err(refuse("a partition whose outcome is not EXACT has no faces to prepare"));
    }
    let faces = partition.getattr(pyo3::intern!(py, "faces"))?;
    let faces = faces.cast::<PyTuple>()?;
    let mut native = Vec::with_capacity(faces.len());
    let mut objects = Vec::with_capacity(faces.len());
    for face in faces.iter() {
        let owner = face.getattr(pyo3::intern!(py, "owner"))?;
        let points = face.getattr(pyo3::intern!(py, "points"))?;
        let tuple = points.cast::<PyTuple>()?;
        let mut contour = Vec::with_capacity(tuple.len());
        let mut items = Vec::with_capacity(tuple.len());
        for point in tuple.iter() {
            let pair = point.cast::<PyTuple>().map_err(|_| refuse("a face point must be a tuple of two SqrtSumV1"))?;
            if pair.len() != 2 {
                return Err(refuse("a face point must be a tuple of two SqrtSumV1"));
            }
            contour.push((sqrt_sum_from_py(pool, &pair.get_item(0)?)?, sqrt_sum_from_py(pool, &pair.get_item(1)?)?));
            items.push(point.unbind());
        }
        let line = face.getattr(pyo3::intern!(py, "line"))?;
        let line = if line.is_none() {
            None
        } else {
            let part = |name: &Bound<'_, pyo3::types::PyString>| -> PyResult<Rat> { pool_rat(pool, &line.getattr(name)?) };
            Some(Line {
                a: part(pyo3::intern!(py, "a"))?,
                b: part(pyo3::intern!(py, "b"))?,
                c: part(pyo3::intern!(py, "c"))?,
                q: part(pyo3::intern!(py, "q"))?,
            })
        };
        native.push(Face::new(contour, line));
        objects.push(PyFace { owner: owner.unbind(), points: points.unbind(), point_items: items, area: None });
    }
    let core = Partition::new(true, native);
    let key = if core.faces().iter().all(|face| face.line().is_some()) {
        let mut q_values = Vec::with_capacity(core.faces().len());
        for face in core.faces() {
            if let Some(line) = face.line() {
                q_values.push(pyobj::fraction_from_rat(py, pool, &line.q)?);
            }
        }
        PyTuple::new(py, [pyo3::types::PyString::intern(py, "prime-universe").into_any(), PyTuple::new(py, q_values)?.into_any()])?.into_any().unbind()
    } else {
        py.None()
    };
    let lookup = prepare_lookup(py, classes, &key)?;
    Ok(Prepared {
        id,
        stamp,
        _partition: partition.clone().unbind(),
        core: Arc::new(core),
        faces: objects,
        polygon_area: partition.getattr(pyo3::intern!(py, "polygon_doubled_area"))?.unbind(),
        key,
        lookup,
    })
}

/// The `cost.StoreKey` of a store key: `hash(key)` of a tuple of `Fraction`s is Python code per fraction (a lookup of 54 of them cost 80 us on 3.11), so it is
/// taken once here and the lookup presents a key that answers `hash` from memory and `==` by identity with `key` first (the shim's class does both, and falls back to
/// the tuple comparison, so a store holding an equal key made by anyone else is found exactly as before). The key itself is what a miss writes.
fn prepare_lookup(py: Python<'_>, classes: &Classes, key: &Py<PyAny>) -> PyResult<Option<Py<PyAny>>> {
    let plain = key.bind(py);
    if plain.is_none() {
        return Ok(None);
    }
    let hash = plain.hash()?;
    let parts = PyTuple::new(py, [plain.get_item(0)?, plain.get_item(1)?, hash.into_pyobject(py)?.into_any(), plain.clone(), pyo3::types::PyList::empty(py).into_any()])?;
    Ok(Some(classes.store_key.bind(py).call1((parts,))?.unbind()))
}

fn pool_rat(pool: &Pool, value: &Bound<'_, PyAny>) -> PyResult<Rat> {
    pyobj::rat_from_number(pool, value)
}

// --------------------------------------------------------------------------
// store records
// --------------------------------------------------------------------------

/// The ints of a tuple of `(prime, power)` pairs of one factorization.
fn pairs_from_py(found: &Bound<'_, PyAny>, bad: &dyn Fn() -> PyErr) -> PyResult<Pairs> {
    let mut pairs = Pairs::new();
    for pair in found.try_iter()? {
        let pair = pair?;
        let pair = pair.cast::<PyTuple>().map_err(|_| bad())?;
        if pair.len() != 2 {
            return Err(bad());
        }
        let power = pyobj::ibig_from_int(&pair.get_item(1)?)?;
        pairs.push((pyobj::ubig_from_int(&pair.get_item(0)?)?, u64::try_from(&power).map_err(|_| bad())?));
    }
    Ok(pairs)
}

/// The entries `(key, value)` of one table of the memory delta, `value` read by `read`.
fn entries_from_py<V>(found: &Bound<'_, PyAny>, bad: &dyn Fn() -> PyErr, read: impl Fn(&Bound<'_, PyAny>) -> PyResult<V>) -> PyResult<Vec<(cftuv_core::num::UBig, V)>> {
    let mut entries = Vec::new();
    for entry in found.try_iter()? {
        let entry = entry?;
        let entry = entry.cast::<PyTuple>().map_err(|_| bad())?;
        if entry.len() != 2 {
            return Err(bad());
        }
        entries.push((pyobj::ubig_from_int(&entry.get_item(0)?)?, read(&entry.get_item(1)?)?));
    }
    Ok(entries)
}

fn ubigs_from_py(found: &Bound<'_, PyAny>) -> PyResult<Vec<cftuv_core::num::UBig>> {
    found.try_iter()?.map(|item| pyobj::ubig_from_int(&item?)).collect()
}

/// `(universe, delta, price, memory)` of the store: `universe` a tuple of ints, `delta` a tuple of `(number, ((prime, power), ...))`, `price` `None`
/// or the six articles, `memory` a `FactorizationMemoryDeltaV1`. A tuple of another length is no record of this kind (`None`: the oracle's
/// `len(found) != 4` is a miss); a value that is no tuple, or a part of the wrong shape, is a named refusal.
fn record_from_py(found: &Bound<'_, PyAny>) -> PyResult<Option<UniverseRecord>> {
    let bad = || refuse("a store record must be (universe, delta, price, memory) of ints, (prime, power) pairs and a FactorizationMemoryDeltaV1");
    let found = found.cast::<PyTuple>().map_err(|_| bad())?;
    if found.len() != 4 {
        return Ok(None);
    }
    let universe = ubigs_from_py(&found.get_item(0)?)?;
    let mut delta = Vec::new();
    for (number, pairs) in entries_from_py(&found.get_item(1)?, &bad, |pairs| pairs_from_py(pairs, &bad))? {
        delta.push((number, pairs));
    }
    let price = found.get_item(2)?;
    let price = if price.is_none() {
        None
    } else {
        let articles: Vec<u64> = price
            .try_iter()?
            .map(|article| u64::try_from(&pyobj::ibig_from_int(&article?)?).map_err(|_| bad()))
            .collect::<PyResult<_>>()?;
        Some(<[u64; 6]>::try_from(articles).map_err(|_| bad())?)
    };
    let memory = found.get_item(3)?;
    let memory = cftuv_canon::MemoryDelta {
        factorizations: entries_from_py(&memory.getattr("factorizations")?, &bad, |pairs| pairs_from_py(pairs, &bad))?,
        squarefree: entries_from_py(&memory.getattr("squarefree")?, &bad, |split| {
            let split = split.cast::<PyTuple>().map_err(|_| bad())?;
            if split.len() != 2 {
                return Err(bad());
            }
            Ok((pyobj::ubig_from_int(&split.get_item(0)?)?, pyobj::ubig_from_int(&split.get_item(1)?)?))
        })?,
        supports: entries_from_py(&memory.getattr("supports")?, &bad, ubigs_from_py)?,
        primes: ubigs_from_py(&memory.getattr("primes")?)?,
    };
    Ok(Some(UniverseRecord { universe, delta, price, memory }))
}

fn pairs_to_py<'py>(py: Python<'py>, pool: &Pool, pairs: &Pairs) -> PyResult<Bound<'py, PyTuple>> {
    let mut items = Vec::with_capacity(pairs.len());
    for (prime, power) in pairs {
        items.push(PyTuple::new(py, [int_from_ubig(py, pool, prime)?, int_from_ibig(py, pool, &IBig::from(*power))?])?);
    }
    PyTuple::new(py, items)
}

fn ubigs_to_py<'py>(py: Python<'py>, pool: &Pool, values: &[cftuv_core::num::UBig]) -> PyResult<Bound<'py, PyTuple>> {
    let items: Vec<Bound<'py, PyAny>> = values.iter().map(|value| int_from_ubig(py, pool, value)).collect::<PyResult<_>>()?;
    PyTuple::new(py, items)
}

/// The record a miss writes: the same tuples of ints (and the `FactorizationMemoryDeltaV1`) the oracle stores.
fn record_to_py<'py>(py: Python<'py>, classes: &Classes, record: &UniverseRecord) -> PyResult<Bound<'py, PyAny>> {
    let pool = &classes.pool;
    let mut delta = Vec::with_capacity(record.delta.len());
    for (number, pairs) in &record.delta {
        delta.push(PyTuple::new(py, [int_from_ubig(py, pool, number)?, pairs_to_py(py, pool, pairs)?.into_any()])?);
    }
    let price = match &record.price {
        None => py.None().into_bound(py),
        Some(price) => {
            let articles: Vec<Bound<'py, PyAny>> = price.iter().map(|article| int_from_ibig(py, pool, &IBig::from(*article))).collect::<PyResult<_>>()?;
            PyTuple::new(py, articles)?.into_any()
        }
    };
    let memory = &record.memory;
    let mut factorizations = Vec::with_capacity(memory.factorizations.len());
    for (key, pairs) in &memory.factorizations {
        factorizations.push(PyTuple::new(py, [int_from_ubig(py, pool, key)?, pairs_to_py(py, pool, pairs)?.into_any()])?);
    }
    let mut squarefree = Vec::with_capacity(memory.squarefree.len());
    for (key, (outside, inside)) in &memory.squarefree {
        let split = PyTuple::new(py, [int_from_ubig(py, pool, outside)?, int_from_ubig(py, pool, inside)?])?;
        squarefree.push(PyTuple::new(py, [int_from_ubig(py, pool, key)?, split.into_any()])?);
    }
    let mut supports = Vec::with_capacity(memory.supports.len());
    for (key, support) in &memory.supports {
        supports.push(PyTuple::new(py, [int_from_ubig(py, pool, key)?, ubigs_to_py(py, pool, support)?.into_any()])?);
    }
    let memory = classes.memory_delta.bind(py).call1((
        PyTuple::new(py, factorizations)?,
        PyTuple::new(py, squarefree)?,
        PyTuple::new(py, supports)?,
        ubigs_to_py(py, pool, &memory.primes)?,
    ))?;
    Ok(PyTuple::new(py, [ubigs_to_py(py, pool, &record.universe)?.into_any(), PyTuple::new(py, delta)?.into_any(), price, memory])?.into_any())
}
