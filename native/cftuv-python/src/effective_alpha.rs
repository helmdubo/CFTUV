//! The contact leaf of the effective-alpha resolver at the boundary: `alpha_frame` converts the data of one source segment ONCE (the numbers the oracle's `SourceContactFrame` holds), and
//! `AlphaFrame.pair` answers one barrier segment against it (`cftuv-alpha`).
//!
//! The leaf has no state, no budget and no memory table; a pair that the port does not carry is an answer, `(2, reason)`, and nothing of the process was touched either way, so the shim
//! lets the oracle compute the pair. What the shim hands over is checked by exact type before anything is computed: the three `RadicalSumV1` of a covector pair and the length as FLAT
//! TUPLES OF `int` (`native_exact.terms_wire`: `(radicand, numerator, denominator, ...)` in the order of `terms`), the seven `(centre, bound)` floats of the prefilter, the texts of the points,
//! and the four classes the contacts are made of (`Fraction`, `RadicalSumV1`, `ExactScalar`, `ExactPlanarPoint`), whose instances are built here without their `__init__`, slot by slot
//! (`pyobj::Raw`, the attribute protocol where the probe does not confirm the layout), as the oracle's own constructors would build them.
//!
//! Answers: `alpha_frame` -> `(0, frame)` or `(2, reason)`; `AlphaFrame.pair` -> `(0, prefilter_rejected, prefilter_passed, contacts)` with `contacts` a tuple of
//! `(alpha, station, point)` (the oracle's tuple), or `(2, reason)`.

use std::panic::{catch_unwind, AssertUnwindSafe};

use cftuv_alpha::{terms_from_wire, Answer, Entry, Frame, Radical};
use cftuv_core::num::{IBig, UBig};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyInt, PyString, PyTuple};
use pyo3::IntoPyObjectExt;

use crate::pyobj::{alloc, fraction_from_rat, ibig_from_int, int_of_ubig, note_attr_build, process_mode, set_slot, ubig_from_int, Pool, Raw};

/// The reason the port declines the frame (a message for the shim to name, never an exception of the oracle).
struct Declined(String);

impl From<PyErr> for Declined {
    fn from(error: PyErr) -> Declined {
        Declined(format!("reading the arguments failed: {error}"))
    }
}

fn decline<T>(message: impl Into<String>) -> Result<T, Declined> {
    Err(Declined(message.into()))
}

/// `(radicand, numerator, denominator)` triples of an exact tuple of exact `int`.
fn wire_of(object: &Bound<'_, PyAny>, what: &str) -> Result<Vec<(UBig, IBig, UBig)>, Declined> {
    if !object.is_exact_instance_of::<PyTuple>() {
        return decline(format!("{what} is not a tuple"));
    }
    let tuple = object.cast::<PyTuple>().map_err(|_| Declined(format!("{what} is not a tuple")))?;
    if tuple.len() % 3 != 0 {
        return decline(format!("{what} is not a whole number of (radicand, numerator, denominator) triples"));
    }
    let mut numbers = Vec::with_capacity(tuple.len());
    for item in tuple.iter() {
        if !item.is_exact_instance_of::<PyInt>() {
            return decline(format!("{what} holds something that is not an int"));
        }
        numbers.push(item);
    }
    let mut triples = Vec::with_capacity(numbers.len() / 3);
    for chunk in numbers.chunks(3) {
        triples.push((ubig_from_int(&chunk[0])?, ibig_from_int(&chunk[1])?, ubig_from_int(&chunk[2])?));
    }
    Ok(triples)
}

fn terms_of(object: &Bound<'_, PyAny>, what: &str) -> Result<cftuv_alpha::Terms, Declined> {
    terms_from_wire(&wire_of(object, what)?).map_err(|invalid| Declined(format!("{what}: {}", invalid.0)))
}

/// The seven `(centre, bound)` floats of the prefilter, or `None` when the oracle's `_float_parts()` is `False`.
fn floats_of(floats: Option<Vec<(f64, f64)>>) -> Result<Option<[Entry; 7]>, Declined> {
    match floats {
        None => Ok(None),
        Some(list) => match <[Entry; 7]>::try_from(list) {
            Ok(array) => Ok(Some(array)),
            Err(list) => decline(format!("the prefilter floats are {} pairs, not seven", list.len())),
        },
    }
}

/// The classes the contacts are made of and the raw access to their slots.
struct Builders {
    /// `Fraction` and `RadicalSumV1` (one slot, `terms`, like `SqrtSumV1`).
    pool: Pool,
    scalar: Py<PyAny>,
    scalar_raw: Option<Raw>,
    expression: Py<PyString>,
    point: Py<PyAny>,
    point_raw: Option<Raw>,
    point_x: Py<PyString>,
    point_y: Py<PyString>,
}

impl Builders {
    fn new(py: Python<'_>, fraction: &Bound<'_, PyAny>, radical: &Bound<'_, PyAny>, scalar: &Bound<'_, PyAny>, point: &Bound<'_, PyAny>) -> PyResult<Builders> {
        let mode = process_mode();
        let pool = Pool::new(py, fraction, radical, mode)?;
        let expression = PyString::intern(py, "expression").unbind();
        let point_x = PyString::intern(py, "x").unbind();
        let point_y = PyString::intern(py, "y").unbind();
        let scalar = scalar.clone().unbind();
        let point = point.clone().unbind();
        let scalar_raw = Raw::select(py, mode, "ExactScalar", &scalar, &[&expression])?;
        let point_raw = Raw::select(py, mode, "ExactPlanarPoint", &point, &[&point_x, &point_y])?;
        Ok(Builders { pool, scalar, scalar_raw, expression, point, point_raw, point_x, point_y })
    }

    /// `RadicalSumV1(((radicand, Fraction), ...))` with the terms as they are.
    fn radical<'py>(&self, py: Python<'py>, value: &Radical) -> PyResult<Bound<'py, PyAny>> {
        let mut terms: Vec<Bound<'py, PyAny>> = Vec::with_capacity(value.terms().len());
        for (radicand, coefficient) in value.terms() {
            terms.push(PyTuple::new(py, [int_of_ubig(py, radicand)?, fraction_from_rat(py, &self.pool, coefficient)?])?.into_any());
        }
        let terms = PyTuple::new(py, terms)?.into_any();
        if let Some(raw) = &self.pool.sqrt_raw {
            return raw.build(py, &self.pool.sqrt_sum, [terms]);
        }
        note_attr_build();
        let object = alloc(py, &self.pool.sqrt_sum)?;
        set_slot(&object, &self.pool.terms, &terms)?;
        Ok(object)
    }

    /// `ExactScalar(text)`.
    fn scalar<'py>(&self, py: Python<'py>, text: &str) -> PyResult<Bound<'py, PyAny>> {
        let text = PyString::new(py, text).into_any();
        if let Some(raw) = &self.scalar_raw {
            return raw.build(py, &self.scalar, [text]);
        }
        note_attr_build();
        let object = alloc(py, &self.scalar)?;
        set_slot(&object, &self.expression, &text)?;
        Ok(object)
    }

    /// `ExactPlanarPoint(ExactScalar(x), ExactScalar(y))`.
    fn point<'py>(&self, py: Python<'py>, x: &str, y: &str) -> PyResult<Bound<'py, PyAny>> {
        let x = self.scalar(py, x)?;
        let y = self.scalar(py, y)?;
        if let Some(raw) = &self.point_raw {
            return raw.build(py, &self.point, [x, y]);
        }
        note_attr_build();
        let object = alloc(py, &self.point)?;
        set_slot(&object, &self.point_x, &x)?;
        set_slot(&object, &self.point_y, &y)?;
        Ok(object)
    }
}

/// The data of a source segment, converted: the oracle's `SourceContactFrame` after `ready()`, in numbers.
#[pyclass(module = "cftuv_native._core", frozen)]
pub struct AlphaFrame {
    inner: Frame,
    builders: Builders,
}

/// `alpha_frame(start, length, tangent, normal, floats, fraction, radical, scalar, point)`: `(0, frame)` or `(2, reason)`.
///
/// `start`: the two texts of the source start; `length`: `terms_wire` of the length; `tangent`, `normal`: `(terms_wire x, terms_wire y)` of the covectors `G·t`, `G·n`; `floats`: the seven
/// `(centre, bound)` pairs of `SourceContactFrame._float_parts()` in its order (start x, y, tangent x, y, normal x, y, length), or `None`; the four classes the contacts are made of.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn alpha_frame<'py>(
    py: Python<'py>,
    start: (String, String),
    length: &Bound<'py, PyAny>,
    tangent: (Bound<'py, PyAny>, Bound<'py, PyAny>),
    normal: (Bound<'py, PyAny>, Bound<'py, PyAny>),
    floats: Option<Vec<(f64, f64)>>,
    fraction: &Bound<'py, PyAny>,
    radical: &Bound<'py, PyAny>,
    scalar: &Bound<'py, PyAny>,
    point: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let built = (|| -> Result<Frame, Declined> {
        Ok(Frame::new(
            Some((start.0.as_str(), start.1.as_str())),
            terms_of(length, "the length")?,
            (terms_of(&tangent.0, "the tangent covector x")?, terms_of(&tangent.1, "the tangent covector y")?),
            (terms_of(&normal.0, "the normal covector x")?, terms_of(&normal.1, "the normal covector y")?),
            floats_of(floats)?,
        ))
    })();
    match built {
        Ok(inner) => {
            let builders = Builders::new(py, fraction, radical, scalar, point)?;
            (0u8, Py::new(py, AlphaFrame { inner, builders })?).into_bound_py_any(py)
        }
        Err(Declined(reason)) => (2u8, reason).into_bound_py_any(py),
    }
}

#[pymethods]
impl AlphaFrame {
    /// The barrier segment `(start_x, start_y) -> (end_x, end_y)` (the texts of its points) against this frame.
    fn pair<'py>(&self, py: Python<'py>, start_x: &str, start_y: &str, end_x: &str, end_y: &str) -> PyResult<Bound<'py, PyAny>> {
        match catch_unwind(AssertUnwindSafe(|| self.inner.pair(start_x, start_y, end_x, end_y))) {
            Ok(Answer::Contacts { rejected, passed, contacts }) => {
                let mut items: Vec<Bound<'py, PyAny>> = Vec::with_capacity(contacts.len());
                for contact in &contacts {
                    let alpha = self.builders.radical(py, &contact.alpha)?;
                    let station = self.builders.radical(py, &contact.station)?;
                    let point = self.builders.point(py, &contact.x, &contact.y)?;
                    items.push(PyTuple::new(py, [alpha, station, point])?.into_any());
                }
                (0u8, rejected, passed, PyTuple::new(py, items)?).into_bound_py_any(py)
            }
            Ok(Answer::Declined(reason)) => (2u8, reason).into_bound_py_any(py),
            Err(panic) => Err(PyRuntimeError::new_err(format!("effective_alpha: native panic: {}", crate::panic_message(&panic)))),
        }
    }
}
