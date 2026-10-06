//! Thin PyO3 layer: whole-operation entry points over byte buffers. No arithmetic here.
//!
//! Every entry point is panic-proof: a panic in the core is caught and raised as `RuntimeError` naming the
//! entry point, so it never unwinds across the boundary. A request the core refuses (a buffer it cannot
//! decode, an unknown opcode) is a `ValueError` with the core's own message.

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
}

#[pymethods]
impl Session {
    #[new]
    fn new() -> Session {
        Session { inner: cftuv_core::session::Session::new() }
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
