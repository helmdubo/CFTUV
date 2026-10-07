//! Test-only PyO3 entries of the skeleton port: the differential seams of `cftuv-skeleton` behind one function.
//!
//! `skeleton_seam_run(session, request) -> bytes` runs ONE seam (format in `cftuv_skeleton::seam`) on a native session and answers the cost answer
//! buffer. Like every entry point it is panic-proof: a refusal of the buffer is a `ValueError`, a panic a `RuntimeError`, and both reset the session (the
//! host's tables were not updated).

use std::panic::{catch_unwind, AssertUnwindSafe};

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::{panic_message, Session};

#[pyfunction]
pub fn skeleton_seam_run<'py>(py: Python<'py>, session: &Bound<'py, Session>, request: &[u8]) -> PyResult<Bound<'py, PyBytes>> {
    let mut guard = session.borrow_mut();
    let inner = &mut guard.inner;
    let outcome = py.detach(|| catch_unwind(AssertUnwindSafe(|| cftuv_skeleton::seam::run(inner, request))));
    match outcome {
        Ok(Ok(response)) => Ok(PyBytes::new(py, &response)),
        Ok(Err(error)) => {
            guard.inner = cftuv_core::session::Session::new();
            Err(PyValueError::new_err(error.to_string()))
        }
        Err(panic) => {
            guard.inner = cftuv_core::session::Session::new();
            Err(PyRuntimeError::new_err(format!("skeleton_seam_run: native panic: {}", panic_message(&panic))))
        }
    }
}

/// `skeleton_seam_table() -> list[tuple[int, str]]`: the opcodes of the seams, for the harness to compare with its own.
#[pyfunction]
pub fn skeleton_seam_table() -> Vec<(u16, &'static str)> {
    cftuv_skeleton::seam::table()
}
