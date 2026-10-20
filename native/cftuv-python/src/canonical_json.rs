//! `codec.canonical_json_bytes(value)` natively: `to_canonical_data` and the C JSON encoder of CPython (`ensure_ascii=False, allow_nan=False, sort_keys=True,
//! separators=(",", ":")`) in one walk, the bytes written directly.
//!
//! The walk takes the oracle's branches in the oracle's order:
//!
//! * exact `str`, `int`, `bool`, `None` are scalars; exact `tuple` is an array; exact `frozenset` is an array of its encoded items sorted by their bytes
//!   (`_json_sort_key`: the UTF-8 of the item's own canonical text);
//! * an `OpaqueId` is `{"$id_type": <class name>, "value": <its value>}`; an `Enum` is its `value`; a `Decimal` is `_canonical_decimal` (normalized under the
//!   CALLER's context precision, then plain notation without trailing zeros);
//! * a dataclass instance is `{"$type": <class name>, <fields>}` without a wire-default field equal to its default (`schema.is_wire_default_field`; the plan
//!   of each class comes from the shim once and is kept by the class);
//! * a `tuple`/`frozenset` subclass as the exact ones, a `dict` with `str` keys as an object (keys sorted as Python sorts `str`: by code point, which is the
//!   order of their UTF-8), a finite `float` by `float.__repr__`, a `str`/`int` subclass as the encoder writes it.
//!
//! Anything else — a NaN or infinity, a non-`str` key, a `list`, a non-finite `Decimal`, a `Decimal` with more digits than the context's precision (its
//! `normalize` would round), an enum or id value that is not a plain `str`/`int`, a string with a lone surrogate (the UTF-8 encoding of the oracle raises),
//! nesting deeper than [`DEPTH_LIMIT`] — is a refusal of the PORT: the dispatcher runs the oracle, which answers or raises as it does.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::sync::Arc;

use cftuv_canon::fxhash::FxBuild;

use pyo3::prelude::*;
use pyo3::types::{PyBool, PyBytes, PyDict, PyFloat, PyFrozenSet, PyInt, PyString, PyTuple, PyType};

/// Deeper nesting is declined (the oracle would recurse as deep; no record of the kernel is near it).
pub const DEPTH_LIMIT: usize = 200;

enum Fail {
    Py(PyErr),
    Unsupported(String),
}

impl From<PyErr> for Fail {
    fn from(error: PyErr) -> Fail {
        Fail::Py(error)
    }
}

type Walk<T> = Result<T, Fail>;

fn unsupported<T>(text: impl Into<String>) -> Walk<T> {
    Err(Fail::Unsupported(text.into()))
}

/// One field of a record class: its name (Python and Rust) and, for a wire-default field, the default it is omitted at.
struct Field {
    name: Py<PyString>,
    text: String,
    default: Option<Py<PyAny>>,
}

enum Kind {
    Record { name: String, fields: Vec<Field> },
    Opaque(String),
    Enum,
    Decimal,
    TupleLike,
    FrozensetLike,
    Dict,
    Float,
    Str,
    Int,
    Unsupported(String),
}

/// The bound classes and the per-class kinds of one session.
pub struct Host {
    opaque: Option<Py<PyType>>,
    enumeration: Option<Py<PyType>>,
    decimal: Option<Py<PyType>>,
    /// `plan(class) -> ((name, is_wire_default, default), ...)` in the order of `dataclasses.fields` (the shim's).
    plan: Option<Py<PyAny>>,
    dataclass_fields: Option<Py<PyString>>,
    value_name: Option<Py<PyString>>,
    kinds: HashMap<usize, (Py<PyType>, Arc<Kind>), FxBuild>,
}

impl Default for Host {
    fn default() -> Host {
        Host { opaque: None, enumeration: None, decimal: None, plan: None, dataclass_fields: None, value_name: None, kinds: HashMap::default() }
    }
}

/// `encode_basestring` of CPython's `_json` (ensure_ascii off): the quote, the backslash and the controls escaped, everything else as it is.
fn put_str(out: &mut Vec<u8>, text: &str) {
    out.push(b'"');
    if !text.bytes().any(|b| b == b'"' || b == b'\\' || b < 0x20) {
        out.extend_from_slice(text.as_bytes());
        out.push(b'"');
        return;
    }
    for ch in text.chars() {
        match ch {
            '"' => out.extend_from_slice(b"\\\""),
            '\\' => out.extend_from_slice(b"\\\\"),
            '\u{8}' => out.extend_from_slice(b"\\b"),
            '\u{c}' => out.extend_from_slice(b"\\f"),
            '\n' => out.extend_from_slice(b"\\n"),
            '\r' => out.extend_from_slice(b"\\r"),
            '\t' => out.extend_from_slice(b"\\t"),
            c if (c as u32) < 0x20 => {
                let mut text = String::new();
                let _ = write!(text, "\\u{:04x}", c as u32);
                out.extend_from_slice(text.as_bytes());
            }
            c => {
                let mut buffer = [0u8; 4];
                out.extend_from_slice(c.encode_utf8(&mut buffer).as_bytes());
            }
        }
    }
    out.push(b'"');
}

fn text_of(value: &Bound<'_, PyAny>) -> Walk<String> {
    let Ok(text) = value.cast::<PyString>() else {
        return unsupported("not a str");
    };
    match text.to_str() {
        Ok(found) => Ok(found.to_string()),
        Err(_) => unsupported("a str with a lone surrogate (its UTF-8 encoding raises)"),
    }
}

/// `int.__repr__`: an `i64` directly, a larger one through Python.
fn put_int(out: &mut Vec<u8>, value: &Bound<'_, PyAny>) -> Walk<()> {
    if let Ok(small) = value.extract::<i64>() {
        out.extend_from_slice(small.to_string().as_bytes());
        return Ok(());
    }
    let exact = value.py().get_type::<PyInt>().call1((value,))?;
    out.extend_from_slice(exact.repr()?.to_str()?.as_bytes());
    Ok(())
}

/// `_canonical_decimal` from the scientific string of the Decimal (`str(value)`): the coefficient and exponent are exact there.
pub fn canonical_decimal(text: &str, precision: usize) -> Option<String> {
    let (negative, body) = match text.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, text.strip_prefix('+').unwrap_or(text)),
    };
    let (mantissa, exponent) = match body.find(['E', 'e']) {
        Some(at) => (&body[..at], body[at + 1..].parse::<i64>().ok()?),
        None => (body, 0),
    };
    let (whole, fraction) = match mantissa.find('.') {
        Some(at) => (&mantissa[..at], &mantissa[at + 1..]),
        None => (mantissa, ""),
    };
    if whole.is_empty() && fraction.is_empty() || !whole.bytes().chain(fraction.bytes()).all(|b| b.is_ascii_digit()) {
        return None; // NaN, Infinity, sNaN, anything unexpected
    }
    let mut digits: String = format!("{whole}{fraction}");
    let mut exponent = exponent.checked_sub(fraction.len() as i64)?;
    let lead = digits.bytes().take_while(|b| *b == b'0').count();
    digits.drain(..lead);
    if digits.is_empty() {
        return Some("0".to_string());
    }
    if digits.len() > precision {
        return None; // `normalize` would round under the context
    }
    let trail = digits.bytes().rev().take_while(|b| *b == b'0').count();
    digits.truncate(digits.len() - trail);
    exponent = exponent.checked_add(trail as i64)?;
    if exponent.abs() > 100_000 {
        return None; // near the exponent limits of the context: `normalize` may clamp
    }
    let mut text = String::new();
    if negative {
        text.push('-');
    }
    if exponent >= 0 {
        text.push_str(&digits);
        text.extend(std::iter::repeat_n('0', exponent as usize));
    } else {
        let shift = (-exponent) as usize;
        if digits.len() > shift {
            let (int_part, frac_part) = digits.split_at(digits.len() - shift);
            text.push_str(int_part);
            text.push('.');
            text.push_str(frac_part);
        } else {
            text.push_str("0.");
            text.extend(std::iter::repeat_n('0', shift - digits.len()));
            text.push_str(&digits);
        }
        // `format(normalized, "f")` has no trailing zero after normalize; the oracle's rstrip has nothing to take.
    }
    Some(text)
}

impl Host {
    /// Binds `ids.OpaqueId`, `enum.Enum`, `decimal.Decimal` and the shim's plan function; forgets the kinds of the previous binding.
    pub fn bind(&mut self, py: Python<'_>, opaque: &Bound<'_, PyType>, enumeration: &Bound<'_, PyType>, decimal: &Bound<'_, PyType>, plan: &Bound<'_, PyAny>) {
        self.opaque = Some(opaque.clone().unbind());
        self.enumeration = Some(enumeration.clone().unbind());
        self.decimal = Some(decimal.clone().unbind());
        self.plan = Some(plan.clone().unbind());
        self.dataclass_fields = Some(PyString::intern(py, "__dataclass_fields__").unbind());
        self.value_name = Some(PyString::intern(py, "value").unbind());
        self.kinds.clear();
    }

    pub fn bound(&self) -> bool {
        self.plan.is_some()
    }

    fn classify(&self, py: Python<'_>, kind: &Bound<'_, PyType>) -> PyResult<Kind> {
        let (Some(opaque), Some(enumeration), Some(decimal), Some(plan), Some(fields)) = (&self.opaque, &self.enumeration, &self.decimal, &self.plan, &self.dataclass_fields) else {
            return Ok(Kind::Unsupported("the canonical classes are not bound".to_string()));
        };
        let name = kind.name()?.to_str()?.to_string();
        if kind.is_subclass(opaque.bind(py))? {
            return Ok(Kind::Opaque(name));
        }
        if kind.is_subclass(enumeration.bind(py))? {
            return Ok(Kind::Enum);
        }
        if kind.is_subclass(decimal.bind(py))? {
            return Ok(Kind::Decimal);
        }
        if kind.hasattr(fields.bind(py))? {
            let mut found = Vec::new();
            for row in plan.bind(py).call1((kind,))?.try_iter()? {
                let row = row?;
                let field_name = row.get_item(0)?;
                let text = field_name.extract::<String>()?;
                let wire = row.get_item(1)?.is_truthy()?;
                found.push(Field {
                    name: PyString::intern(py, &text).unbind(),
                    text,
                    default: if wire { Some(row.get_item(2)?.unbind()) } else { None },
                });
            }
            found.sort_by(|one, two| one.text.cmp(&two.text));
            if found.iter().any(|field| field.text.as_str() < "$type") {
                return Ok(Kind::Unsupported(format!("a field of {name} sorts before $type")));
            }
            return Ok(Kind::Record { name, fields: found });
        }
        if kind.is_subclass_of::<PyTuple>()? {
            return Ok(Kind::TupleLike);
        }
        if kind.is_subclass_of::<PyFrozenSet>()? {
            return Ok(Kind::FrozensetLike);
        }
        if kind.is_subclass_of::<PyDict>()? {
            return Ok(Kind::Dict);
        }
        if kind.is_subclass_of::<PyFloat>()? {
            return Ok(Kind::Float);
        }
        if kind.is_subclass_of::<PyBool>()? {
            return Ok(Kind::Unsupported(format!("a bool subclass {name}")));
        }
        if kind.is_subclass_of::<PyString>()? {
            return Ok(Kind::Str);
        }
        if kind.is_subclass_of::<PyInt>()? {
            return Ok(Kind::Int);
        }
        Ok(Kind::Unsupported(format!("unsupported canonical value: {name}")))
    }

    fn kind_of(&mut self, py: Python<'_>, kind: &Bound<'_, PyType>) -> PyResult<Arc<Kind>> {
        let key = kind.as_ptr() as usize;
        if let Some((_, found)) = self.kinds.get(&key) {
            return Ok(found.clone());
        }
        let found = Arc::new(self.classify(py, kind)?);
        self.kinds.insert(key, (kind.clone().unbind(), found.clone()));
        Ok(found)
    }

    /// A raw value under an id or an enum (the oracle hands it to the encoder as it is): a plain `str` or `int`.
    fn put_raw(out: &mut Vec<u8>, value: &Bound<'_, PyAny>) -> Walk<()> {
        if value.is_exact_instance_of::<PyString>() {
            put_str(out, &text_of(value)?);
            Ok(())
        } else if value.is_exact_instance_of::<PyInt>() {
            put_int(out, value)
        } else {
            unsupported("an id or enum value that is not a plain str or int")
        }
    }

    fn put_items<'py>(&mut self, py: Python<'py>, out: &mut Vec<u8>, items: Vec<Bound<'py, PyAny>>, depth: usize, sorted: bool, precision: usize) -> Walk<()> {
        if sorted {
            let mut encoded: Vec<Vec<u8>> = Vec::new();
            for item in items {
                let mut one = Vec::new();
                self.put(py, &mut one, &item, depth + 1, precision)?;
                encoded.push(one);
            }
            encoded.sort();
            out.push(b'[');
            for (index, one) in encoded.iter().enumerate() {
                if index > 0 {
                    out.push(b',');
                }
                out.extend_from_slice(one);
            }
            out.push(b']');
        } else {
            out.push(b'[');
            for (index, item) in items.iter().enumerate() {
                if index > 0 {
                    out.push(b',');
                }
                self.put(py, out, item, depth + 1, precision)?;
            }
            out.push(b']');
        }
        Ok(())
    }

    fn put<'py>(&mut self, py: Python<'py>, out: &mut Vec<u8>, value: &Bound<'py, PyAny>, depth: usize, precision: usize) -> Walk<()> {
        if depth > DEPTH_LIMIT {
            return unsupported("nesting deeper than the port's limit");
        }
        if value.is_exact_instance_of::<PyString>() {
            put_str(out, &text_of(value)?);
            return Ok(());
        }
        if value.is_exact_instance_of::<PyBool>() {
            out.extend_from_slice(if value.is_truthy()? { b"true" } else { b"false" });
            return Ok(());
        }
        if value.is_exact_instance_of::<PyInt>() {
            return put_int(out, value);
        }
        if value.is_none() {
            out.extend_from_slice(b"null");
            return Ok(());
        }
        if let Ok(tuple) = value.cast_exact::<PyTuple>() {
            return self.put_items(py, out, tuple.iter().collect(), depth, false, precision);
        }
        if let Ok(set) = value.cast_exact::<PyFrozenSet>() {
            return self.put_items(py, out, set.iter().collect(), depth, true, precision);
        }
        let class = value.get_type();
        let kind = self.kind_of(py, &class)?;
        match &*kind {
            Kind::Opaque(name) => {
                out.extend_from_slice(b"{\"$id_type\":");
                put_str(out, name);
                out.extend_from_slice(b",\"value\":");
                let Some(value_name) = &self.value_name else { return unsupported("not bound") };
                Self::put_raw(out, &value.getattr(value_name.bind(py))?)?;
                out.push(b'}');
                Ok(())
            }
            Kind::Enum => {
                let Some(value_name) = &self.value_name else { return unsupported("not bound") };
                Self::put_raw(out, &value.getattr(value_name.bind(py))?)
            }
            Kind::Decimal => {
                let text = value.str()?;
                match canonical_decimal(text.to_str()?, precision) {
                    Some(found) => {
                        put_str(out, &found);
                        Ok(())
                    }
                    None => unsupported(format!("a Decimal the port does not normalize: {}", text.to_str()?)),
                }
            }
            Kind::Record { name, fields } => {
                out.extend_from_slice(b"{\"$type\":");
                put_str(out, name);
                for field in fields {
                    let held = value.getattr(field.name.bind(py))?;
                    if let Some(default) = &field.default {
                        if held.eq(default.bind(py))? {
                            continue;
                        }
                    }
                    out.push(b',');
                    put_str(out, &field.text);
                    out.push(b':');
                    self.put(py, out, &held, depth + 1, precision)?;
                }
                out.push(b'}');
                Ok(())
            }
            Kind::TupleLike => {
                let items: Vec<Bound<'py, PyAny>> = value.try_iter()?.collect::<PyResult<_>>()?;
                self.put_items(py, out, items, depth, false, precision)
            }
            Kind::FrozensetLike => {
                let items: Vec<Bound<'py, PyAny>> = value.try_iter()?.collect::<PyResult<_>>()?;
                self.put_items(py, out, items, depth, true, precision)
            }
            Kind::Dict => {
                let dict = value.cast::<PyDict>().map_err(|_| Fail::Unsupported("not a dict".to_string()))?;
                let mut entries: Vec<(String, Bound<'py, PyAny>)> = Vec::with_capacity(dict.len());
                for (key, item) in dict.iter() {
                    if !key.is_instance_of::<PyString>() {
                        return unsupported("canonical mappings require string keys");
                    }
                    entries.push((text_of(&key)?, item));
                }
                entries.sort_by(|one, two| one.0.cmp(&two.0));
                out.push(b'{');
                for (index, (key, item)) in entries.iter().enumerate() {
                    if index > 0 {
                        out.push(b',');
                    }
                    put_str(out, key);
                    out.push(b':');
                    self.put(py, out, item, depth + 1, precision)?;
                }
                out.push(b'}');
                Ok(())
            }
            Kind::Float => {
                let number: f64 = value.extract()?;
                if !number.is_finite() {
                    return unsupported("NaN and infinity are forbidden");
                }
                // `float.__repr__` itself (the encoder calls it on a float subclass too): CPython's dtoa is the oracle's. Re-deriving it in Rust
                // (shortest length, then the half-even digits of that length) measured no faster and has its own tie rule to get right.
                let exact = if value.is_exact_instance_of::<PyFloat>() { value.clone() } else { PyFloat::new(py, number).into_any() };
                out.extend_from_slice(exact.repr()?.to_str()?.as_bytes());
                Ok(())
            }
            Kind::Str => {
                put_str(out, &text_of(value)?);
                Ok(())
            }
            Kind::Int => {
                let exact = py.get_type::<PyInt>().call1((value,))?;
                out.extend_from_slice(exact.repr()?.to_str()?.as_bytes());
                Ok(())
            }
            Kind::Unsupported(text) => unsupported(text.clone()),
        }
    }

    /// `(True, bytes)` or `(False, why the port declined)`; a Python error raised while reading goes out as it is.
    pub fn encode<'py>(&mut self, py: Python<'py>, value: &Bound<'py, PyAny>, precision: usize) -> PyResult<(bool, Bound<'py, PyAny>)> {
        if !self.bound() {
            return Ok((false, PyString::new(py, "the canonical classes are not bound").into_any()));
        }
        let mut out = Vec::with_capacity(4096);
        match self.put(py, &mut out, value, 0, precision) {
            Ok(()) => Ok((true, PyBytes::new(py, &out).into_any())),
            Err(Fail::Unsupported(text)) => Ok((false, PyString::new(py, &text).into_any())),
            Err(Fail::Py(error)) => Err(error),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::canonical_decimal;

    #[test]
    fn decimals_are_written_as_the_oracle_writes_them() {
        // `_canonical_decimal(Decimal(text))` under prec 28 (checked against CPython 3.11)
        for (text, wanted) in [
            ("0", "0"),
            ("-0", "0"),
            ("0E-8", "0"),
            ("1.500", "1.5"),
            ("-0.00012", "-0.00012"),
            ("1.2E+5", "120000"),
            ("1E-7", "0.0000001"),
            ("100", "100"),
            ("1.499950382858514785766601562", "1.499950382858514785766601562"),
            ("12345678901234567890123456.78", "12345678901234567890123456.78"),
            ("0.000", "0"),
            ("5E+2", "500"),
        ] {
            assert_eq!(canonical_decimal(text, 28).as_deref(), Some(wanted), "{text}");
        }
        for text in ["NaN", "-Infinity", "sNaN", "Infinity", "1234567890123456789012345678.9"] {
            assert_eq!(canonical_decimal(text, 28), None, "{text}");
        }
    }
}
