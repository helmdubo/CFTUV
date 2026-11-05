//! `materialize.uv_unfold.unfold(data)` at the boundary (UNFOLD_ALONG_U_V1): the numbers of `UnfoldInputV1` read once, the solve in `cftuv-unfold`, the arrays of
//! `UnfoldResultV1` back as a tuple the shim wraps. Pure: no budget, no memory tables. An input the port cannot read is `ValueError("cftuv_native: unfold declines: ...")`,
//! which the shim turns into the named refusal of the port (`NativePortUnsupported`), and the reference answers.

use cftuv_unfold::{unfold, UnfoldInput, COUNTER_NAMES, DEGENERATE_SINE, FLAT_GRADIENT, GUARD_CEILING, GUARD_FACTOR, GUARD_FLOOR, UNORIENTED_SINE};
use std::panic::{catch_unwind, AssertUnwindSafe};

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyTuple;
use pyo3::IntoPyObjectExt;

fn declined(name: &str, error: PyErr) -> PyErr {
    PyValueError::new_err(format!("cftuv_native: unfold declines: `{name}`: {error}"))
}

fn floats(data: &Bound<'_, PyAny>, name: &str) -> PyResult<Vec<f64>> {
    data.getattr(name)?.extract::<Vec<f64>>().map_err(|error| declined(name, error))
}

fn indices(data: &Bound<'_, PyAny>, name: &str) -> PyResult<Vec<usize>> {
    data.getattr(name)?.extract::<Vec<usize>>().map_err(|error| declined(name, error))
}

fn flags(data: &Bound<'_, PyAny>, name: &str) -> PyResult<Vec<bool>> {
    data.getattr(name)?.extract::<Vec<bool>>().map_err(|error| declined(name, error))
}

/// `unfold_u(data)` -> `(u, slope_x, slope_y, held, counters)`.
#[pyfunction]
pub fn unfold_u<'py>(py: Python<'py>, data: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyTuple>> {
    let input = UnfoldInput {
        positions: floats(data, "positions")?,
        chart: floats(data, "chart")?,
        r: floats(data, "r")?,
        s: floats(data, "s")?,
        source: flags(data, "source")?,
        ring_starts: indices(data, "ring_starts")?,
        corners: indices(data, "corners")?,
        corner_station: floats(data, "corner_station")?,
        hold_source: data.getattr("hold_source")?.extract::<bool>().map_err(|error| declined("hold_source", error))?,
        iterations: data.getattr("iterations")?.extract::<usize>().map_err(|error| declined("iterations", error))?,
        guard_rounds: data.getattr("guard_rounds")?.extract::<usize>().map_err(|error| declined("guard_rounds", error))?,
    };
    let outcome = py.detach(|| catch_unwind(AssertUnwindSafe(|| unfold(&input))));
    let result = match outcome {
        Ok(Ok(result)) => result,
        Ok(Err(error)) => return Err(PyValueError::new_err(format!("cftuv_native: unfold declines: {}", error.0))),
        Err(_) => return Err(PyRuntimeError::new_err("cftuv_native: the unfold port panicked")),
    };
    let counters: Vec<(&str, i64)> = result.counters;
    (result.u, result.slope_x, result.slope_y, result.held, counters).into_pyobject(py)
}

/// The constants of the port, for the shim to compare with the reference's when it binds: `(DEGENERATE_SINE, FLAT_GRADIENT, UNORIENTED_SINE, GUARD_FLOOR, GUARD_CEILING,
/// GUARD_FACTOR, COUNTER_NAMES)`.
#[pyfunction]
pub fn unfold_constants(py: Python<'_>) -> PyResult<Py<PyAny>> {
    (DEGENERATE_SINE, FLAT_GRADIENT, UNORIENTED_SINE, GUARD_FLOOR, GUARD_CEILING, GUARD_FACTOR, COUNTER_NAMES.to_vec()).into_py_any(py)
}
