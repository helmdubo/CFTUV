//! ОДИН pure raw batch; ни Python callbacks, ни выражений, ни общего состояния.
#[path = "density_input.rs"]
mod input;
#[path = "density_memo_input.rs"]
mod memo_input;
#[path = "density_memo_output.rs"]
mod memo_output;
#[path = "density_output.rs"]
mod output;

use pyo3::exceptions::{PyRuntimeError, PyTypeError, PyValueError};
use pyo3::prelude::*;
use std::panic::{catch_unwind, AssertUnwindSafe};

#[pyfunction]
pub fn density_compute<'py>(
    py: Python<'py>,
    batch: &Bound<'py, PyAny>,
    fraction: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let decoder = input::Decoder::new(py, fraction)?;
    let raw = decoder.batch(batch)?;
    let report = py.detach(|| {
        catch_unwind(AssertUnwindSafe(|| {
            cftuv_density::density_raw_frontend_v1::evaluate_raw_density_batch(&raw)
        }))
    });
    match report {
        Ok(report) => output::Encoder::new(py, &decoder.fraction)?.report(&report),
        Err(panic) => Err(PyRuntimeError::new_err(format!(
            "density_compute: native panic: {}",
            crate::panic_message(&panic)
        ))),
    }
}

#[pyfunction]
pub fn density_compute_with_memo<'py>(
    py: Python<'py>,
    batch: &Bound<'py, PyAny>,
    snapshot: &Bound<'py, PyAny>,
    fraction: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let decoder = input::Decoder::new(py, fraction)?;
    let raw = decoder.batch(batch)?;
    let snapshot = memo_input::snapshot(&decoder, snapshot, &raw.canon_ids).map_err(|error| {
        if error.is_instance_of::<PyTypeError>(py) || error.is_instance_of::<PyValueError>(py) {
            error
        } else {
            PyTypeError::new_err(error.to_string())
        }
    })?;
    let report = py.detach(|| {
        catch_unwind(AssertUnwindSafe(|| {
            cftuv_density::density_raw_frontend_v1::evaluate_raw_density_batch_with_memo(
                &raw, &snapshot,
            )
        }))
    });
    match report {
        Ok(report) => {
            let encoder = output::Encoder::new(py, &decoder.fraction)?;
            memo_output::report(&encoder, &report)
        }
        Err(panic) => Err(PyRuntimeError::new_err(format!(
            "density_compute: native panic: {}",
            crate::panic_message(&panic)
        ))),
    }
}
