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
use pyo3::types::{PyFloat, PyString, PyTuple};
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
    /// Where `Fraction._numerator` and `._denominator` sit in an instance (found by probing, see [`Raw`]); `None`: use the attribute protocol.
    pub fraction_raw: Option<Raw>,
    /// Where `SqrtSumV1.terms` sits in an instance.
    pub sqrt_raw: Option<Raw>,
}

impl Pool {
    pub fn new(py: Python<'_>, fraction: &Bound<'_, PyAny>, sqrt_sum: &Bound<'_, PyAny>) -> PyResult<Pool> {
        let int_type = py.import("builtins")?.getattr("int")?;
        let zero_sum = sqrt_sum.call1((PyTuple::empty(py),))?;
        let numerator = PyString::intern(py, "_numerator").unbind();
        let denominator = PyString::intern(py, "_denominator").unbind();
        let terms = PyString::intern(py, "terms").unbind();
        let fraction = fraction.clone().unbind();
        let sqrt_sum = sqrt_sum.clone().unbind();
        let fraction_raw = Raw::probe(py, &fraction, &[&numerator, &denominator])?;
        let sqrt_raw = Raw::probe(py, &sqrt_sum, &[&terms])?;
        Ok(Pool { int_type: int_type.unbind(), fraction, sqrt_sum, zero_sum: zero_sum.unbind(), numerator, denominator, terms, fraction_raw, sqrt_raw })
    }
}

impl Pool {
    /// Whether the raw slot access is engaged: `(Fraction, SqrtSumV1)`.
    pub fn raw_layouts(&self) -> (bool, bool) {
        (self.fraction_raw.is_some(), self.sqrt_raw.is_some())
    }

    /// Test-only: use the attribute protocol everywhere (the fallback of the raw access).
    pub fn disable_raw(&mut self) {
        self.fraction_raw = None;
        self.sqrt_raw = None;
    }
}

// --------------------------------------------------------------------------
// raw access to the slots of `__slots__` classes
// --------------------------------------------------------------------------

/// The byte offsets of the slots of a class whose instances hold nothing but pointer-sized slots after the object header
/// (`__slots__` and no `__dict__`, no `__weakref__`): the generic attribute protocol costs a type lookup and a descriptor call per
/// slot, a pointer read or store costs nothing. The layout is NOT assumed: [`Raw::probe`] builds a real instance through the
/// generic setter, finds where each value landed and refuses (`None`: the callers use the attribute protocol) unless the class has
/// exactly the expected shape; a read is only done on an instance of exactly the probed class.
pub struct Raw {
    offsets: Vec<usize>,
}

impl Raw {
    /// The layout of `class` for `names` (in this order), or `None` when it is not the plain pointer-slot layout.
    pub fn probe(py: Python<'_>, class: &Py<PyAny>, names: &[&Py<PyString>]) -> PyResult<Option<Raw>> {
        let bound = class.bind(py);
        let size: usize = bound.getattr(intern!(py, "__basicsize__"))?.extract()?;
        let items: usize = bound.getattr(intern!(py, "__itemsize__"))?.extract()?;
        let header = 2 * std::mem::size_of::<usize>();
        if items != 0 || size != header + names.len() * std::mem::size_of::<usize>() {
            return Ok(None);
        }
        let instance = alloc(py, class)?;
        let mut values = Vec::with_capacity(names.len());
        for (index, name) in names.iter().enumerate() {
            let value = PyFloat::new(py, index as f64 + 0.25).into_any();
            set_slot(&instance, name, &value)?;
            values.push(value);
        }
        let mut offsets = Vec::with_capacity(names.len());
        for value in &values {
            let mut found = None;
            for index in 0..names.len() {
                let at = header + index * std::mem::size_of::<usize>();
                // SAFETY: `at + 8 <= basicsize`: inside the instance that was just allocated.
                let held = unsafe { *instance.as_ptr().cast::<u8>().add(at).cast::<*mut ffi::PyObject>() };
                if held == value.as_ptr() {
                    if found.is_some() {
                        return Ok(None);
                    }
                    found = Some(at);
                }
            }
            match found {
                Some(at) => offsets.push(at),
                None => return Ok(None),
            }
        }
        Ok(Some(Raw { offsets }))
    }

    /// `object.__new__(class)` with every slot set to the given object (ownership moves into the instance), in the order of the probe's names.
    pub fn build<'py, const N: usize>(&self, py: Python<'py>, class: &Py<PyAny>, values: [Bound<'py, PyAny>; N]) -> PyResult<Bound<'py, PyAny>> {
        debug_assert_eq!(self.offsets.len(), N);
        let object = alloc(py, class)?;
        for (offset, value) in self.offsets.iter().zip(values) {
            // SAFETY: `offset` is a slot of exactly this class (probed), the slot is still empty after `alloc`, and the instance owns the reference it is given.
            unsafe { *object.as_ptr().cast::<u8>().add(*offset).cast::<*mut ffi::PyObject>() = value.into_ptr() };
        }
        Ok(object)
    }

    /// The object in slot `index` of `obj`, borrowed; `None` when the slot is empty. `obj` must be an instance of exactly the probed class.
    pub fn read<'a, 'py>(&self, obj: &'a Bound<'py, PyAny>, index: usize) -> Option<Borrowed<'a, 'py, PyAny>> {
        // SAFETY: the offset is a slot of the probed class and `obj` is an instance of it (the caller checked the type); a slot holds NULL or a valid object that lives as long as `obj`.
        unsafe { Borrowed::from_ptr_or_opt(obj.py(), *obj.as_ptr().cast::<u8>().add(self.offsets[index]).cast::<*mut ffi::PyObject>()) }
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
    big_from_int(obj, overflow < 0)
}

/// An `int` beyond 64 bits as little-endian words: the magnitude is taken 64 bits at a time (`& (2**64 - 1)` through `PyLong_AsUnsignedLongLongMask`, then
/// `>> 64`) with C calls only; a method call per number (`__abs__`, `bit_length`, `to_bytes`) costs several times more.
fn big_from_int(obj: &Bound<'_, PyAny>, negative: bool) -> PyResult<IBig> {
    let py = obj.py();
    // SAFETY: each call gets valid objects and its result is checked; `64` is a small integer (a cached object).
    unsafe {
        let shift = Bound::from_owned_ptr_or_err(py, ffi::PyLong_FromLong(64))?;
        let mut current = if negative { Bound::from_owned_ptr_or_err(py, ffi::PyNumber_Absolute(obj.as_ptr()))? } else { obj.clone() };
        let mut words: Vec<u64> = Vec::with_capacity(4);
        loop {
            let word = ffi::PyLong_AsUnsignedLongLongMask(current.as_ptr());
            if word == u64::MAX {
                if let Some(error) = PyErr::take(py) {
                    return Err(error);
                }
            }
            words.push(word);
            let rest = Bound::from_owned_ptr_or_err(py, ffi::PyNumber_Rshift(current.as_ptr(), shift.as_ptr()))?;
            let mut overflow = 0;
            let value = ffi::PyLong_AsLongLongAndOverflow(rest.as_ptr(), &mut overflow);
            if overflow == 0 {
                if value == -1 {
                    if let Some(error) = PyErr::take(py) {
                        return Err(error);
                    }
                }
                if value != 0 {
                    words.push(value as u64);
                }
                break;
            }
            current = rest;
        }
        Ok(num::from_sign_magnitude(negative, UBig::from_words(&words)))
    }
}

pub fn ubig_from_int(obj: &Bound<'_, PyAny>) -> PyResult<UBig> {
    let value = ibig_from_int(obj)?;
    if num::is_negative(&value) {
        return Err(refuse(format!("a negative number where a radicand or a denominator is expected: {value}")));
    }
    Ok(num::magnitude(&value))
}

/// Two hexadecimal digits per byte value.
const HEX_PAIRS: [[u8; 2]; 256] = {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut table = [[0u8; 2]; 256];
    let mut byte = 0;
    while byte < 256 {
        table[byte] = [DIGITS[byte >> 4], DIGITS[byte & 0xf]];
        byte += 1;
    }
    table
};

/// A Python `int` from the little-endian 64-bit words of its magnitude: the words are written as hexadecimal digits and
/// parsed by `PyLong_FromString` (a power-of-two base has no digit limit), which costs one allocation where
/// `int.from_bytes` costs a bytes object, an argument tuple and a call. The text is `-` (when negative) and the digits of the
/// non-zero bytes, two per byte (`PyLong_FromString` takes a leading zero digit).
fn int_from_words<'py>(py: Python<'py>, negative: bool, words: &[u64]) -> PyResult<Bound<'py, PyAny>> {
    let mut text: Vec<u8> = Vec::with_capacity(words.len() * 16 + 3);
    text.push(b'-');
    let mut started = false;
    for word in words.iter().rev() {
        for byte in word.to_be_bytes() {
            if started || byte != 0 {
                started = true;
                text.extend_from_slice(&HEX_PAIRS[byte as usize]);
            }
        }
    }
    if !started {
        text.extend_from_slice(b"00");
    }
    text.push(0);
    let begin = usize::from(!negative);
    // SAFETY: `text` is a NUL-terminated ASCII string that lives through the call; the result is a new reference or NULL.
    unsafe { Bound::from_owned_ptr_or_err(py, ffi::PyLong_FromString(text.as_ptr().add(begin).cast(), std::ptr::null_mut(), 16)) }
}

pub fn int_from_ibig<'py>(py: Python<'py>, _pool: &Pool, value: &IBig) -> PyResult<Bound<'py, PyAny>> {
    int_of_ibig(py, value)
}

/// Test-only: a Python `int` through [`ibig_from_int`] and [`int_of_ibig`] (both directions of the boundary conversion).
pub fn int_round_trip<'py>(value: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    if !value.is_instance_of::<pyo3::types::PyInt>() {
        return Err(refuse("int_round_trip takes an int"));
    }
    int_of_ibig(value.py(), &ibig_from_int(value)?)
}

fn int_of_ibig<'py>(py: Python<'py>, value: &IBig) -> PyResult<Bound<'py, PyAny>> {
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
    let numerator = int_from_ibig(py, pool, value.numerator())?;
    let denominator = int_from_ubig(py, pool, value.denominator())?;
    if let Some(raw) = &pool.fraction_raw {
        return raw.build(py, &pool.fraction, [numerator, denominator]);
    }
    let fraction = alloc(py, &pool.fraction)?;
    set_slot(&fraction, &pool.numerator, &numerator)?;
    set_slot(&fraction, &pool.denominator, &denominator)?;
    Ok(fraction)
}

/// The rational of a Python `int` or `Fraction` (exact types only: anything else is refused by name).
pub fn rat_from_number(pool: &Pool, obj: &Bound<'_, PyAny>) -> PyResult<Rat> {
    coef_from_number(pool, obj).map(Coef::into_value)
}

/// `Fraction(numerator, denominator)` of a Python `Fraction`'s two parts, which are in lowest terms unless someone built the object by hand: a pair that fits in
/// 64 bits is checked for a common factor by a binary gcd on machine words (the usual case, no allocation), anything else (a common factor, bigger numbers) goes
/// through `Rat::reduced`, which divides it out.
fn rat_of_parts(numerator: IBig, denominator: UBig) -> Rat {
    if let (Ok(small_numerator), Ok(small_denominator)) = (i64::try_from(&numerator), u64::try_from(&denominator)) {
        if gcd_is_one(small_numerator.unsigned_abs(), small_denominator) {
            return Rat::from_canonical(numerator, denominator);
        }
    }
    Rat::reduced(numerator, denominator)
}

/// `math.gcd(a, b) == 1` (Stein's binary algorithm; `gcd(0, b) = b`).
fn gcd_is_one(mut a: u64, mut b: u64) -> bool {
    if a == 0 || b == 0 {
        return a | b == 1;
    }
    if (a | b) & 1 == 0 {
        return false;
    }
    a >>= a.trailing_zeros();
    loop {
        b >>= b.trailing_zeros();
        if a > b {
            std::mem::swap(&mut a, &mut b);
        }
        b -= a;
        if b == 0 {
            return a == 1;
        }
    }
}

pub fn coef_from_number(pool: &Pool, obj: &Bound<'_, PyAny>) -> PyResult<Coef> {
    let py = obj.py();
    if is_exactly(obj, &pool.int_type) {
        return Ok(Coef::int(ibig_from_int(obj)?));
    }
    if is_exactly(obj, &pool.fraction) {
        let (numerator, denominator) = match pool.fraction_raw.as_ref().and_then(|raw| raw.read(obj, 0).zip(raw.read(obj, 1))) {
            Some((numerator, denominator)) => (ibig_from_int(&numerator)?, ubig_from_int(&denominator)?),
            None => (ibig_from_int(&obj.getattr(pool.numerator.bind(py))?)?, ubig_from_int(&obj.getattr(pool.denominator.bind(py))?)?),
        };
        if denominator.is_zero() {
            return Err(refuse("a Fraction with a zero denominator"));
        }
        return Ok(Coef::fraction(rat_of_parts(numerator, denominator)));
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
    match pool.sqrt_raw.as_ref().and_then(|raw| raw.read(obj, 0)) {
        Some(terms) => sqrt_sum_from_terms(pool, &terms),
        None => sqrt_sum_from_terms(pool, &obj.getattr(pool.terms.bind(py))?),
    }
}

fn sqrt_sum_from_terms(pool: &Pool, terms: &Bound<'_, PyAny>) -> PyResult<SqrtSum> {
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
    let terms = PyTuple::new(py, terms)?.into_any();
    if let Some(raw) = &pool.sqrt_raw {
        return raw.build(py, &pool.sqrt_sum, [terms]);
    }
    let value = alloc(py, &pool.sqrt_sum)?;
    set_slot(&value, &pool.terms, &terms)?;
    Ok(value)
}
