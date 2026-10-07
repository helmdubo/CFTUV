//! Thin PyO3 layer: whole-operation entry points over byte buffers. No arithmetic here.
//!
//! Every entry point is panic-proof: a panic in the core is caught and raised as `RuntimeError` naming the
//! entry point, so it never unwinds across the boundary. A request the core refuses (a buffer it cannot
//! decode, an unknown opcode) is a `ValueError` with the core's own message.

mod clip;
mod clip_seams;
mod coverage;
mod memlog;
mod pyobj;
mod refusal;
mod skeleton;
mod skeleton_seams;
mod view;

// The digest of the Rust sources, shared with `build.rs` (which embeds it) and exposed for the tests to check the algorithm on a tree of their own.
#[path = "../digest.rs"]
mod digest;

use std::panic::{catch_unwind, AssertUnwindSafe};

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList, PyTuple};

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

#[pyfunction]
fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// `run_number_ops(request: bytes) -> bytes`: a whole script of number operations (test-only differential
/// entry, format in `cftuv_core::script`).
#[pyfunction]
fn run_number_ops<'py>(py: Python<'py>, request: &[u8]) -> PyResult<Bound<'py, PyBytes>> {
    let outcome = py.detach(|| catch_unwind(AssertUnwindSafe(|| cftuv_core::script::run_number_ops(request))));
    match outcome {
        Ok(Ok(response)) => Ok(PyBytes::new(py, &response)),
        Ok(Err(error)) => Err(PyValueError::new_err(error.to_string())),
        Err(panic) => Err(PyRuntimeError::new_err(format!("run_number_ops: native panic: {}", panic_message(&panic)))),
    }
}

/// Test-only: a Python `int` through the boundary conversions and back.
#[pyfunction]
fn int_round_trip<'py>(value: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    pyobj::int_round_trip(value)
}

/// `source_digest() -> str`: the sha256 of the Rust sources this extension was built from (`build.rs`, rule in `digest.rs`).
#[pyfunction]
fn source_digest() -> &'static str {
    env!("CFTUV_NATIVE_SOURCE_DIGEST")
}

/// `tree_digest(native_root: str) -> str`: the same digest over the workspace at `native_root` (test-only: the algorithm on a tree of the test's own).
#[pyfunction]
fn tree_digest(native_root: &str) -> PyResult<String> {
    digest::tree_digest(std::path::Path::new(native_root)).map(|found| found.hex).map_err(|error| PyValueError::new_err(format!("tree_digest({native_root:?}): {error}")))
}

/// `process_slot_mode() -> str`: the mode of the raw slot access the extension read from `CFTUV_NATIVE_SLOTS` when it was imported (`raw`, `attr` or `auto`).
#[pyfunction]
fn process_slot_mode() -> &'static str {
    pyobj::process_mode().name()
}

/// `slot_counters() -> (raw reads, attribute reads, raw builds, attribute builds)`: which path the slot accesses of the whole operations actually took (`pyobj::slot_counters`).
#[pyfunction]
fn slot_counters() -> (u64, u64, u64, u64) {
    let [raw_reads, attr_reads, raw_builds, attr_builds] = pyobj::slot_counters();
    (raw_reads, attr_reads, raw_builds, attr_builds)
}

/// Zeroes the four slot counters.
#[pyfunction]
fn reset_slot_counters() {
    pyobj::reset_slot_counters();
}

/// `number_op_table() -> list[tuple[int, str]]`: the opcode table, for the harness to compare with its own.
#[pyfunction]
fn number_op_table() -> Vec<(u8, &'static str)> {
    cftuv_core::script::OPS.to_vec()
}

/// A persistent native session: the mirror of the canonicalization memory and the product cache. `run` takes the
/// same buffer as `run_number_ops` (with the cost flag, a sync of the host's memory and a budget).
///
/// Any refusal or panic resets the session to empty: the host's real tables were not updated by a failed call,
/// so the mirror must not keep what the aborted call did to it. The host shim drops its own mirror state on
/// every exception for the same reason. The whole operations (`coverage_at`, `clip_geometry`) do the same for a refusal of the PORT that they
/// ANSWER instead of raising (`refusal.rs`): the call's effects are dropped, nothing of the host is touched, the mirror is forgotten.
#[pyclass(module = "cftuv_native._core")]
struct Session {
    pub(crate) inner: cftuv_core::session::Session,
    coverage: coverage::Host,
    clip: clip::Host,
    /// The host's memory tables as this session's mirror last saw them (see `view.rs`).
    view: view::View,
    /// Test knob (`force_refusal`): a refusal of the port the NEXT whole operation takes after it computed.
    forced: Option<refusal::Forced>,
    /// The whole `build_skeleton` (WP-S6): the bound classes of its result.
    skeleton: skeleton::Host,
    /// How the slots of the bound classes are read and written (`pyobj::SlotMode`): the process-wide mode (`CFTUV_NATIVE_SLOTS`) unless the session was made with its own.
    slots: pyobj::SlotMode,
}

#[pymethods]
impl Session {
    /// `slots`: `raw`, `attr` or `auto` for this session alone (tests); `None` takes the process-wide mode the extension read from `CFTUV_NATIVE_SLOTS` when it was imported.
    #[new]
    #[pyo3(signature = (slots=None))]
    fn new(slots: Option<&str>) -> PyResult<Session> {
        let slots = match slots {
            Some(text) => pyobj::SlotMode::parse(text).map_err(|why| pyobj::refuse(format!("Session(slots=...): {why}")))?,
            None => pyobj::process_mode(),
        };
        Ok(Session { inner: cftuv_core::session::Session::new(), coverage: coverage::Host::default(), clip: clip::Host::default(), view: view::View::default(), forced: None, skeleton: skeleton::Host::default(), slots })
    }

    fn run<'py>(&mut self, py: Python<'py>, request: &[u8]) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &mut self.inner;
        let outcome = py.detach(|| catch_unwind(AssertUnwindSafe(|| cftuv_core::script::run_script(inner, request))));
        match outcome {
            Ok(Ok(response)) => Ok(PyBytes::new(py, &response)),
            Ok(Err(error)) => {
                self.reset_memory();
                Err(PyValueError::new_err(error.to_string()))
            }
            Err(panic) => {
                self.reset_memory();
                Err(PyRuntimeError::new_err(format!("Session.run: native panic: {}", panic_message(&panic))))
            }
        }
    }

    /// Hands the kernel classes to the coverage entry points (`SqrtSumV1`, `Fraction`, `CoverageV1`, `FaceCoverageV1`,
    /// the three `CoverageOutcome` members the entry points build, `FaceOutcome.EXACT`, the shim's `StoreKey`, `FactorizationMemoryDeltaV1`). Forgets every
    /// prepared partition and every converted store record.
    #[allow(clippy::too_many_arguments)]
    fn bind_coverage(
        &mut self,
        py: Python<'_>,
        sqrt_sum: &Bound<'_, PyAny>,
        fraction: &Bound<'_, PyAny>,
        coverage: &Bound<'_, PyAny>,
        face_coverage: &Bound<'_, PyAny>,
        outcome_exact: &Bound<'_, PyAny>,
        outcome_not_exact: &Bound<'_, PyAny>,
        outcome_negative: &Bound<'_, PyAny>,
        face_exact: &Bound<'_, PyAny>,
        store_key: &Bound<'_, PyAny>,
        memory_delta: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        self.coverage.bind(py, sqrt_sum, fraction, coverage, face_coverage, [outcome_exact, outcome_not_exact, outcome_negative], face_exact, store_key, memory_delta, self.slots)
    }

    /// The `CoverageV1` of a refused call (`negative`: `ALPHA_IS_NEGATIVE`, else `PARTITION_IS_NOT_EXACT`).
    fn refused_coverage<'py>(&self, py: Python<'py>, partition: &Bound<'py, PyAny>, alpha: &Bound<'py, PyAny>, negative: bool) -> PyResult<Bound<'py, PyAny>> {
        self.coverage.refused(py, partition, alpha, negative)
    }

    /// `_coverage_at` on an exact partition and `alpha >= 0`: `(result or None, status, detail, sign-counter deltas, budget
    /// articles after, changed-tables bits, (prepare, arguments, compute, result, memory log) nanoseconds)`. `sync` is the memory sync
    /// in the wire format (`None`: unchanged since the last call), `budget` is `(cap, six articles)` or `None`, `tables` the real memory tables
    /// `(registry list, registry set, factorizations, squarefree splits, supports)` the memory log of the call is replayed on, in place (as
    /// for `clip_geometry`), `traces` the list the oracle's `traces` argument is (`None`: not asked for): the `(signs, values)` of every face
    /// whose signs were computed are appended to it. Any error resets the session, as `run` does, and so does a status that is a refusal of the port (`refusal.rs`): then nothing of the call is applied to `tables`, `store` or `traces`.
    #[allow(clippy::too_many_arguments)]
    fn coverage_at<'py>(
        &mut self,
        py: Python<'py>,
        partition: &Bound<'py, PyAny>,
        alpha: &Bound<'py, PyAny>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        store: Option<Bound<'py, PyAny>>,
        work_budget: &Bound<'py, PyAny>,
        tables: memlog::Tables<'py>,
        traces: Option<Bound<'py, pyo3::types::PyList>>,
    ) -> PyResult<coverage::Answer7<'py>> {
        let forced = self.forced.take();
        let outcome = self.coverage.coverage_at(py, &mut self.inner, partition, alpha, sync, budget, store.as_ref(), work_budget, &tables, traces.as_ref(), forced);
        // an error, or an answer that is a refusal of the port: nothing of the call reached the host, and the mirror holds what the call did to it
        if outcome.as_ref().map_or(true, |answer| refusal::is_native_only(answer.1)) {
            self.reset_memory();
        }
        outcome
    }

    /// `(partitions, store records)` the coverage side keeps converted.
    fn coverage_cache(&self) -> (usize, usize) {
        self.coverage.cache_sizes()
    }

    /// Drops every converted partition and store record.
    fn forget_coverage(&mut self) {
        self.coverage.forget();
    }

    /// Hands the kernel classes to `clip_geometry` (`SqrtSumV1`, `Fraction`, `ClippedV1`, `LocalPoint3V1`). Forgets every converted plane.
    fn bind_clip(&mut self, py: Python<'_>, sqrt_sum: &Bound<'_, PyAny>, fraction: &Bound<'_, PyAny>, clipped: &Bound<'_, PyAny>, local_point: &Bound<'_, PyAny>) -> PyResult<()> {
        self.clip.bind(py, sqrt_sum, fraction, clipped, local_point, self.slots)
    }

    /// `clip.clip_geometry` whole (see `clip.rs`): `(result or None, status, detail, sign-counter deltas, budget articles after,
    /// changed-tables bits, (plane, arguments, compute, result, memory log) nanoseconds)`. `triangles` is `plane.triangles`, `law` the code of
    /// the topology law (0 planar polygons, 1 quad strips, 2 any other), `inert` the chain station plan's pairs of faces (a frozenset of frozensets of names), `sync` the memory sync
    /// in the wire format (`None`: unchanged), `budget` `(cap, six articles)` or `None`, `normals` the plane's
    /// `_normal_by_position`, `tables` the real memory tables `(registry list, registry set, factorizations, squarefree splits,
    /// supports)` the memory log of the call is replayed on, in place. Any error resets the session, as `run` does, and so does a status that is a refusal of the port (`refusal.rs`): then nothing of the call is applied to `tables` or `normals`.
    #[allow(clippy::too_many_arguments)]
    fn clip_geometry<'py>(
        &mut self,
        py: Python<'py>,
        triangles: &Bound<'py, PyAny>,
        points: &Bound<'py, pyo3::types::PyDict>,
        cycles: &Bound<'py, PyAny>,
        polygons: &Bound<'py, PyAny>,
        law: u8,
        seam: &Bound<'py, PyAny>,
        fans: &Bound<'py, PyAny>,
        flows: &Bound<'py, PyAny>,
        by_faces: bool,
        inert: &Bound<'py, PyAny>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        normals: Option<&Bound<'py, pyo3::types::PyDict>>,
        tables: memlog::Tables<'py>,
    ) -> PyResult<clip::Answer<'py>> {
        let forced = self.forced.take();
        let outcome = self.clip.clip_geometry(py, &mut self.inner, triangles, points, cycles, polygons, law, seam, fans, flows, by_faces, inert, sync, budget, normals, &tables, forced);
        // an error, or an answer that is a refusal of the port: nothing of the call reached the host, and the mirror holds what the call did to it
        if outcome.as_ref().map_or(true, |answer| refusal::is_native_only(answer.1)) {
            self.reset_memory();
        }
        outcome
    }

    /// Hands the kernel classes to `build_skeleton`: `SqrtSumV1`, `Fraction`, then `SkeletonV1`, `SkeletonNodeV1`, `EventTimeV1`, `EventPointV1`, `ProofObligationV1`, then the tables
    /// `{value: member}` of `SkeletonOutcome`, `EventKind`, `ProofStatus`, `ProofObligationBranch`, `ProofObligationDisposition`, `CandidateRefusal`.
    #[allow(clippy::too_many_arguments)]
    fn bind_skeleton(
        &mut self,
        py: Python<'_>,
        sqrt_sum: &Bound<'_, PyAny>,
        fraction: &Bound<'_, PyAny>,
        skeleton: &Bound<'_, PyAny>,
        node: &Bound<'_, PyAny>,
        time: &Bound<'_, PyAny>,
        point: &Bound<'_, PyAny>,
        obligation: &Bound<'_, PyAny>,
        outcomes: &Bound<'_, PyAny>,
        kinds: &Bound<'_, PyAny>,
        statuses: &Bound<'_, PyAny>,
        branches: &Bound<'_, PyAny>,
        dispositions: &Bound<'_, PyAny>,
        refusals: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        self.skeleton.bind(py, sqrt_sum, fraction, [skeleton, node, time, point, obligation], [outcomes, kinds, statuses, branches, dispositions, refusals], self.slots)
    }

    /// `skeleton.build_skeleton` whole (see `skeleton.rs`; `exhaustive`: `split_search is EXHAUSTIVE`): `(result or None, status, detail, sign-counter deltas, budget articles after, changed-tables bits,
    /// (arguments, compute, result, memory log) nanoseconds, the `superlevel` string the oracle wrote into the budget or None)`. `level_limit` is the live `level_budget(polygon)`, `march_steps` the live
    /// number of march steps when the oracle's `march_budget` was replaced (`None`: the declared one), `sync` the memory sync in the wire format (`None`: unchanged), `budget` `(cap, six
    /// articles)` or `None`, `tables` the real memory tables the memory log of the call is replayed on, in place. Any error resets the session, as `run` does, and so does a status that is a refusal
    /// of the port (`skeleton::is_native_only`): then nothing of the call is applied to `tables`.
    #[allow(clippy::too_many_arguments)]
    fn build_skeleton<'py>(
        &mut self,
        py: Python<'py>,
        polygon: &Bound<'py, PyAny>,
        dense_hydration: bool,
        exhaustive: bool,
        replay_check: bool,
        level_limit: i64,
        march_steps: Option<i64>,
        sync: Option<&[u8]>,
        budget: Option<(Option<u64>, [u64; 6])>,
        tables: memlog::Tables<'py>,
    ) -> PyResult<skeleton::Answer<'py>> {
        let forced = self.forced.take();
        let outcome = self.skeleton.build_skeleton(py, &mut self.inner, polygon, dense_hydration, exhaustive, replay_check, level_limit, march_steps, sync, budget, &tables, forced);
        // an error, or an answer that is a refusal of the port: nothing of the call reached the host, and the mirror holds what the call did to it
        if outcome.as_ref().map_or(true, |answer| skeleton::is_native_only(answer.1)) {
            self.reset_memory();
        }
        outcome
    }

    /// Test-only: arms ONE refusal of the port (`unsupported` (a clip), `invalid_input`, `diverged`, `internal`, `panic`; `None` disarms) that the next whole operation
    /// takes AFTER it computed, instead of its own outcome: the call then has real effects (articles, counters, memory log, normal writes, store record, traces) that a refusal
    /// must not let reach the host. Consumed by the first `coverage_at` or `clip_geometry` that reaches its computation.
    fn force_refusal(&mut self, kind: Option<&str>) -> PyResult<()> {
        self.forced = kind.map(refusal::Forced::parse).transpose()?;
        Ok(())
    }

    /// `{slot class: raw access engaged}` of the two bound entry points (a debugging view: a layout the probe did not confirm is read through the
    /// attribute protocol, which is slower, not wrong).
    fn raw_layouts(&self) -> Vec<(&'static str, bool)> {
        let mut found = Vec::new();
        if let Some((fraction, sqrt_sum, face)) = self.coverage.raw_layouts() {
            found.extend([("coverage Fraction", fraction), ("coverage SqrtSumV1", sqrt_sum), ("FaceCoverageV1", face)]);
        }
        if let Some((fraction, sqrt_sum, point)) = self.clip.raw_layouts() {
            found.extend([("clip Fraction", fraction), ("clip SqrtSumV1", sqrt_sum), ("LocalPoint3V1", point)]);
        }
        found
    }

    /// `{result class: raw access engaged}` of `build_skeleton` (a layout the probe did not confirm is built through the attribute protocol, which is slower, not wrong).
    fn skeleton_raw_layouts(&self) -> Vec<(&'static str, bool)> {
        match self.skeleton.raw_layouts() {
            Some([skeleton, node, time, point, obligation]) => vec![("SkeletonV1", skeleton), ("SkeletonNodeV1", node), ("EventTimeV1", time), ("EventPointV1", point), ("ProofObligationV1", obligation)],
            None => Vec::new(),
        }
    }

    /// Test-only: every slot of the result classes and of the inputs through the attribute protocol (the fallback of the raw access); the session's mode becomes `attr`.
    fn disable_raw(&mut self) {
        self.slots = pyobj::SlotMode::Attr;
        self.coverage.disable_raw();
        self.clip.disable_raw();
        self.skeleton.disable_raw();
    }

    /// `raw`, `attr` or `auto`: how this session reads and writes the slots of the bound classes.
    fn slot_mode(&self) -> &'static str {
        self.slots.name()
    }

    /// Test-only: a `Fraction` (`sum` false) or a `SqrtSumV1` (`sum` true) through the conversions of the boundary and back (needs `bind_coverage`).
    fn round_trip<'py>(&self, py: Python<'py>, value: &Bound<'py, PyAny>, sum: bool) -> PyResult<Bound<'py, PyAny>> {
        self.coverage.round_trip(py, value, sum)
    }

    /// Planes the clip side keeps converted.
    fn clip_cache(&self) -> usize {
        self.clip.cache_size()
    }

    /// Drops every converted plane and every cross-call result.
    fn forget_clip(&mut self) {
        self.clip.forget();
    }

    /// Switches the clip's cross-call cache on or off (a harness knob: the answers and the cost do not depend on it).
    fn set_clip_warm_enabled(&mut self, enabled: bool) {
        self.clip.set_warm_enabled(enabled);
    }

    /// Drops the clip's cross-call results; the converted planes stay.
    fn clear_clip_warm(&mut self) {
        self.clip.clear_warm();
    }

    /// Test knob: the size at which the clip's cross-call cache drops everything.
    fn set_clip_warm_limit(&mut self, limit: usize) {
        self.clip.set_warm_limit(limit);
    }

    /// `(crossing hits, crossings stored, crossing entries, value hits, lift hits)` of the clip's cross-call cache of exact results.
    fn clip_warm_stats(&self) -> (u64, u64, usize, u64, u64) {
        self.clip.warm_stats()
    }

    /// Lengths of the mirrored tables: registry, factorizations, squarefree splits, supports.
    fn lengths(&self) -> (usize, usize, usize, usize) {
        self.inner.memory.lengths()
    }

    /// Back to the empty session (the host reloads its tables on the next call).
    fn clear(&mut self) {
        self.reset_memory();
    }

    /// Whether the host's four tables are, entry by entry and in order, the very objects the session's mirror saw after the last call (`view.rs`); `false` is "not
    /// provably unchanged": the shim then compares the tables with `view_lists` as it always did.
    fn view_matches(&self, primes: &Bound<'_, PyList>, factorization: &Bound<'_, PyDict>, squarefree: &Bound<'_, PyDict>, support: &Bound<'_, PyDict>) -> bool {
        self.view.matches(primes, [factorization, squarefree, support])
    }

    /// Takes the view of the tables in `mask` (1 registry, 2 factorizations, 4 squarefree splits, 8 supports) from the host's tables as they are now.
    fn view_capture(&mut self, py: Python<'_>, primes: &Bound<'_, PyList>, factorization: &Bound<'_, PyDict>, squarefree: &Bound<'_, PyDict>, support: &Bound<'_, PyDict>, mask: u8) {
        self.view.capture(py, primes, [factorization, squarefree, support], mask);
    }

    /// The view as lists: `(primes, (factorization keys, values), (squarefree keys, values), (support keys, values))`.
    fn view_lists<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        self.view.lists(py)
    }
}

impl Session {
    /// The mirror and the view of the host's tables forgotten together: the host reloads its tables on the next call.
    fn reset_memory(&mut self) {
        self.inner = cftuv_core::session::Session::new();
        self.view = view::View::default();
    }
}

pub(crate) fn panic_message(panic: &Box<dyn std::any::Any + Send>) -> String {
    if let Some(text) = panic.downcast_ref::<&str>() {
        (*text).to_string()
    } else if let Some(text) = panic.downcast_ref::<String>() {
        text.clone()
    } else {
        "non-text panic payload".to_string()
    }
}

#[pymodule]
fn _core(module: &Bound<'_, PyModule>) -> PyResult<()> {
    // the slot mode is read ONCE, here: an unknown `CFTUV_NATIVE_SLOTS` is a named refusal and the extension does not import
    pyobj::init_process_mode()?;
    module.add_function(wrap_pyfunction!(version, module)?)?;
    module.add_function(wrap_pyfunction!(process_slot_mode, module)?)?;
    module.add_function(wrap_pyfunction!(slot_counters, module)?)?;
    module.add_function(wrap_pyfunction!(reset_slot_counters, module)?)?;
    module.add_function(wrap_pyfunction!(run_number_ops, module)?)?;
    module.add_function(wrap_pyfunction!(number_op_table, module)?)?;
    module.add_function(wrap_pyfunction!(int_round_trip, module)?)?;
    module.add_function(wrap_pyfunction!(source_digest, module)?)?;
    module.add_function(wrap_pyfunction!(tree_digest, module)?)?;
    module.add_function(wrap_pyfunction!(refusal::oracle_statuses, module)?)?;
    module.add_function(wrap_pyfunction!(skeleton::skeleton_oracle_statuses, module)?)?;
    module.add_class::<Session>()?;
    module.add_function(wrap_pyfunction!(clip_seams::clip_seam_run, module)?)?;
    module.add_function(wrap_pyfunction!(clip_seams::clip_seam_table, module)?)?;
    module.add_function(wrap_pyfunction!(skeleton_seams::skeleton_seam_run, module)?)?;
    module.add_function(wrap_pyfunction!(skeleton_seams::skeleton_seam_table, module)?)?;
    Ok(())
}
