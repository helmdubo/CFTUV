//! Thin PyO3 layer: whole-operation entry points over byte buffers. No arithmetic here.
//!
//! Every entry point is panic-proof: a panic in the core is caught and raised as `RuntimeError` naming the
//! entry point, so it never unwinds across the boundary. A request the core refuses (a buffer it cannot
//! decode, an unknown opcode) is a `ValueError` with the core's own message.

mod coverage;
mod pyobj;

use std::panic::{catch_unwind, AssertUnwindSafe};

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyBytes;

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
/// every exception for the same reason.
#[pyclass(module = "cftuv_native._core")]
struct Session {
    inner: cftuv_core::session::Session,
    coverage: coverage::Host,
}

#[pymethods]
impl Session {
    #[new]
    fn new() -> Session {
        Session { inner: cftuv_core::session::Session::new(), coverage: coverage::Host::default() }
    }

    fn run<'py>(&mut self, py: Python<'py>, request: &[u8]) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &mut self.inner;
        let outcome = py.detach(|| catch_unwind(AssertUnwindSafe(|| cftuv_core::script::run_script(inner, request))));
        match outcome {
            Ok(Ok(response)) => Ok(PyBytes::new(py, &response)),
            Ok(Err(error)) => {
                self.inner = cftuv_core::session::Session::new();
                Err(PyValueError::new_err(error.to_string()))
            }
            Err(panic) => {
                self.inner = cftuv_core::session::Session::new();
                Err(PyRuntimeError::new_err(format!("Session.run: native panic: {}", panic_message(&panic))))
            }
        }
    }

    /// Hands the kernel classes to the coverage entry points (`SqrtSumV1`, `Fraction`, `CoverageV1`, `FaceCoverageV1`,
    /// the three `CoverageOutcome` members the entry points build, `FaceOutcome.EXACT`). Forgets every prepared partition.
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
    ) -> PyResult<()> {
        self.coverage.bind(py, sqrt_sum, fraction, coverage, face_coverage, [outcome_exact, outcome_not_exact, outcome_negative], face_exact)
    }

    /// The `CoverageV1` of a refused call (`negative`: `ALPHA_IS_NEGATIVE`, else `PARTITION_IS_NOT_EXACT`).
    fn refused_coverage<'py>(&self, py: Python<'py>, partition: &Bound<'py, PyAny>, alpha: &Bound<'py, PyAny>, negative: bool) -> PyResult<Bound<'py, PyAny>> {
        self.coverage.refused(py, partition, alpha, negative)
    }

    /// `_coverage_at` on an exact partition and `alpha >= 0`: `(result or None, status, detail, sign-counter deltas, budget
    /// articles after, memory log or None, (prepare, arguments, compute, result) nanoseconds)`. `sync` is the memory sync
    /// in the wire format (`None`: unchanged since the last call), `budget` is `(cap, six articles)` or `None`. Any error
    /// resets the session, as `run` does.
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
    ) -> PyResult<coverage::Answer7<'py>> {
        let outcome = self.coverage.coverage_at(py, &mut self.inner, partition, alpha, sync, budget, store.as_ref(), work_budget);
        if outcome.is_err() {
            self.inner = cftuv_core::session::Session::new();
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

    /// Lengths of the mirrored tables: registry, factorizations, squarefree splits, supports.
    fn lengths(&self) -> (usize, usize, usize, usize) {
        self.inner.memory.lengths()
    }

    /// Back to the empty session (the host reloads its tables on the next call).
    fn clear(&mut self) {
        self.inner = cftuv_core::session::Session::new();
    }
}

fn panic_message(panic: &Box<dyn std::any::Any + Send>) -> String {
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
    module.add_function(wrap_pyfunction!(version, module)?)?;
    module.add_function(wrap_pyfunction!(run_number_ops, module)?)?;
    module.add_function(wrap_pyfunction!(number_op_table, module)?)?;
    module.add_class::<Session>()?;
    Ok(())
}
