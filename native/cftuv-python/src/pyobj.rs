//! Python object plumbing for the whole-operation entry points: big integers, `Fraction`, `SqrtSumV1`, and
//! instances of frozen `slots` dataclasses built without their `__init__`.
//!
//! The extension is `abi3` (one wheel for 3.11 and 3.13), so only the limited API is used. Big integers go in as
//! little-endian bytes (`int.to_bytes`) and come out as hexadecimal text (`PyLong_FromString`); machine-sized ones
//! through `PyLong_*LongLong`.
//!
//! Building a frozen dataclass instance: `PyType_GenericAlloc` (what `object.__new__` does) and then
//! `PyObject_GenericSetAttr` per field. The generic setter goes straight to the slot's member descriptor and does
//! not call the class's `__setattr__`, which is what `frozen=True` overrides; the generated `__init__` of such a
//! class does the same through `object.__setattr__`. That is the only `unsafe` in the module besides the raw
//! integer constructors, and each use is a single FFI call with its result checked.

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyString, PyTuple};
use pyo3::{ffi, intern};

use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::{Coef, Rat};
use cftuv_core::sqrt_sum::{SqrtSum, Term};

/// A named refusal of the boundary (a Python object the extension does not carry): `TypeError`, never a fallback.
pub fn refuse(message: impl AsRef<str>) -> PyErr {
    PyTypeError::new_err(format!("cftuv_native: {}", message.as_ref()))
}

/// The classes and constants the conversions need, looked up once when the shim binds them.
pub struct Pool {
    pub int_type: Py<PyAny>,
    pub fraction: Py<PyAny>,
    pub sqrt_sum: Py<PyAny>,
    pub zero_sum: Py<PyAny>,
    pub numerator: Py<PyString>,
    pub denominator: Py<PyString>,
    pub terms: Py<PyString>,
}

impl Pool {
    pub fn new(py: Python<'_>, fraction: &Bound<'_, PyAny>, sqrt_sum: &Bound<'_, PyAny>) -> PyResult<Pool> {
        let int_type = py.import("builtins")?.getattr("int")?;
        let zero_sum = sqrt_sum.call1((PyTuple::empty(py),))?;
        Ok(Pool {
            int_type: int_type.unbind(),
            fraction: fraction.clone().unbind(),
            sqrt_sum: sqrt_sum.clone().unbind(),
            zero_sum: zero_sum.unbind(),
            numerator: PyString::intern(py, "_numerator").unbind(),
            denominator: PyString::intern(py, "_denominator").unbind(),
            terms: PyString::intern(py, "terms").unbind(),
        })
    }
}

// --------------------------------------------------------------------------
// instances of slot classes
// --------------------------------------------------------------------------

/// `object.__new__(cls)` for a class with `__slots__`: nothing set yet.
pub fn alloc<'py>(py: Python<'py>, class: &Py<PyAny>) -> PyResult<Bound<'py, PyAny>> {
    // SAFETY: `class` is a type object (checked when it was bound); the result is a new reference or NULL.
    unsafe { Bound::from_owned_ptr_or_err(py, ffi::PyType_GenericAlloc(class.as_ptr().cast(), 0)) }
}

/// `object.__setattr__(obj, name, value)`: the slot's member descriptor, bypassing a frozen class's `__setattr__`.
pub fn set_slot(obj: &Bound<'_, PyAny>, name: &Py<PyString>, value: &Bound<'_, PyAny>) -> PyResult<()> {
    // SAFETY: three valid object pointers; a negative status is an error with the exception set.
    let status = unsafe { ffi::PyObject_GenericSetAttr(obj.as_ptr(), name.as_ptr(), value.as_ptr()) };
    if status < 0 {
        Err(PyErr::fetch(obj.py()))
    } else {
        Ok(())
    }
}

pub fn is_exactly(obj: &Bound<'_, PyAny>, class: &Py<PyAny>) -> bool {
    obj.get_type().as_ptr() == class.as_ptr()
}

// --------------------------------------------------------------------------
// integers
// --------------------------------------------------------------------------

/// The value of a Python `int` (the caller checked the type).
pub fn ibig_from_int(obj: &Bound<'_, PyAny>) -> PyResult<IBig> {
    let mut overflow = 0;
    // SAFETY: `obj` is an `int`; `overflow` is a valid out parameter.
    let value = unsafe { ffi::PyLong_AsLongLongAndOverflow(obj.as_ptr(), &mut overflow) };
    if overflow == 0 {
        if value == -1 {
            if let Some(error) = PyErr::take(obj.py()) {
                return Err(error);
            }
        }
        return Ok(IBig::from(value));
    }
    let magnitude = if overflow < 0 { obj.call_method0(intern!(obj.py(), "__abs__"))? } else { obj.clone() };
    let bits: usize = magnitude.call_method0(intern!(obj.py(), "bit_length"))?.extract()?;
    let bytes = magnitude.call_method1(intern!(obj.py(), "to_bytes"), (bits.div_ceil(8), intern!(obj.py(), "little")))?;
    let bytes = bytes.cast::<PyBytes>()?;
    Ok(num::from_sign_le_bytes(overflow < 0, bytes.as_bytes()))
}

pub fn ubig_from_int(obj: &Bound<'_, PyAny>) -> PyResult<UBig> {
    let value = ibig_from_int(obj)?;
    if num::is_negative(&value) {
        return Err(refuse(format!("a negative number where a radicand or a denominator is expected: {value}")));
    }
    Ok(num::magnitude(&value))
}

/// A Python `int` from the little-endian 64-bit words of its magnitude: the words are written as hexadecimal digits and
/// parsed by `PyLong_FromString` (a power-of-two base has no digit limit), which costs one allocation where
/// `int.from_bytes` costs a bytes object, an argument tuple and a call.
fn int_from_words<'py>(py: Python<'py>, negative: bool, words: &[u64]) -> PyResult<Bound<'py, PyAny>> {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut text: Vec<u8> = Vec::with_capacity(words.len() * 16 + 2);
    if negative {
        text.push(b'-');
    }
    let mut started = false;
    for word in words.iter().rev() {
        for shift in (0..16).rev() {
            let digit = ((word >> (4 * shift)) & 0xf) as usize;
            if started || digit != 0 {
                started = true;
                text.push(DIGITS[digit]);
            }
        }
    }
    if !started {
        text.push(b'0');
    }
    text.push(0);
    // SAFETY: `text` is a NUL-terminated ASCII string that lives through the call; the result is a new reference or NULL.
    unsafe { Bound::from_owned_ptr_or_err(py, ffi::PyLong_FromString(text.as_ptr().cast(), std::ptr::null_mut(), 16)) }
}

pub fn int_from_ibig<'py>(py: Python<'py>, _pool: &Pool, value: &IBig) -> PyResult<Bound<'py, PyAny>> {
    if let Ok(small) = i64::try_from(value) {
        // SAFETY: a new reference or NULL.
        return unsafe { Bound::from_owned_ptr_or_err(py, ffi::PyLong_FromLongLong(small)) };
    }
    let (sign, words) = value.as_sign_words();
    int_from_words(py, sign == num::Sign::Negative, words)
}

pub fn int_from_ubig<'py>(py: Python<'py>, _pool: &Pool, value: &UBig) -> PyResult<Bound<'py, PyAny>> {
    if let Ok(small) = u64::try_from(value) {
        // SAFETY: a new reference or NULL.
        return unsafe { Bound::from_owned_ptr_or_err(py, ffi::PyLong_FromUnsignedLongLong(small)) };
    }
    int_from_words(py, false, value.as_words())
}

// --------------------------------------------------------------------------
// Fraction, coefficients, SqrtSumV1
// --------------------------------------------------------------------------

/// A `Fraction` from a canonical rational: no gcd, the two slots are set directly (as `codec.make_fraction` does).
pub fn fraction_from_rat<'py>(py: Python<'py>, pool: &Pool, value: &Rat) -> PyResult<Bound<'py, PyAny>> {
    let fraction = alloc(py, &pool.fraction)?;
    set_slot(&fraction, &pool.numerator, &int_from_ibig(py, pool, value.numerator())?)?;
    set_slot(&fraction, &pool.denominator, &int_from_ubig(py, pool, value.denominator())?)?;
    Ok(fraction)
}

/// The rational of a Python `int` or `Fraction` (exact types only: anything else is refused by name).
pub fn rat_from_number(pool: &Pool, obj: &Bound<'_, PyAny>) -> PyResult<Rat> {
    coef_from_number(pool, obj).map(Coef::into_value)
}

pub fn coef_from_number(pool: &Pool, obj: &Bound<'_, PyAny>) -> PyResult<Coef> {
    let py = obj.py();
    if is_exactly(obj, &pool.int_type) {
        return Ok(Coef::int(ibig_from_int(obj)?));
    }
    if is_exactly(obj, &pool.fraction) {
        let numerator = ibig_from_int(&obj.getattr(pool.numerator.bind(py))?)?;
        let denominator = ubig_from_int(&obj.getattr(pool.denominator.bind(py))?)?;
        if denominator.is_zero() {
            return Err(refuse("a Fraction with a zero denominator"));
        }
        return Ok(Coef::fraction(Rat::reduced(numerator, denominator)));
    }
    Err(refuse(format!("a coefficient must be an int or a Fraction, not {}", obj.get_type().name()?)))
}

fn coef_to_py<'py>(py: Python<'py>, pool: &Pool, coef: &Coef) -> PyResult<Bound<'py, PyAny>> {
    if coef.is_py_int() {
        int_from_ibig(py, pool, coef.value().numerator())
    } else {
        fraction_from_rat(py, pool, coef.value())
    }
}

/// A `SqrtSumV1` object into a native sum (canonical form checked: a non-canonical one is refused by name).
pub fn sqrt_sum_from_py(pool: &Pool, obj: &Bound<'_, PyAny>) -> PyResult<SqrtSum> {
    let py = obj.py();
    if !is_exactly(obj, &pool.sqrt_sum) {
        return Err(refuse(format!("a point coordinate must be a SqrtSumV1, not {}", obj.get_type().name()?)));
    }
    let terms = obj.getattr(pool.terms.bind(py))?;
    let terms = terms.cast::<PyTuple>()?;
    let mut out = Vec::with_capacity(terms.len());
    for term in terms.iter() {
        let term = term.cast::<PyTuple>()?;
        if term.len() != 2 {
            return Err(refuse("a SqrtSumV1 term must be a (radicand, coefficient) pair"));
        }
        let radicand = term.get_item(0)?;
        if !is_exactly(&radicand, &pool.int_type) {
            return Err(refuse("a SqrtSumV1 radicand must be an int"));
        }
        out.push(Term { radicand: ubig_from_int(&radicand)?, coef: coef_from_number(pool, &term.get_item(1)?)? });
    }
    SqrtSum::from_terms(out).map_err(|error| refuse(format!("a SqrtSumV1 outside canonical form (term {}: {})", error.index, error.reason)))
}

/// A native sum as a `SqrtSumV1` with exactly the coefficient types the oracle would hold (`int` stays `int`).
pub fn sqrt_sum_to_py<'py>(py: Python<'py>, pool: &Pool, sum: &SqrtSum) -> PyResult<Bound<'py, PyAny>> {
    if sum.terms().is_empty() {
        return Ok(pool.zero_sum.bind(py).clone());
    }
    let mut terms = Vec::with_capacity(sum.terms().len());
    for term in sum.terms() {
        let pair = [int_from_ubig(py, pool, &term.radicand)?, coef_to_py(py, pool, &term.coef)?];
        terms.push(PyTuple::new(py, pair)?);
    }
    let value = alloc(py, &pool.sqrt_sum)?;
    set_slot(&value, &pool.terms, PyTuple::new(py, terms)?.as_any())?;
    Ok(value)
}
