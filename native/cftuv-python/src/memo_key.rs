//! `clip_memo.clip_key(inputs, triangles, policy)` natively (CLIP_MEMO_KEY_NATIVE_V1): the canonical record of the arguments of the clip stage
//! (`clip_memo._encode`) written in one walk, and its sha256.
//!
//! The walk takes the oracle's branches in the oracle's order, by the EXACT class of the value: `int`, `Fraction`, `str`, `bool`, `None`, `float`, `tuple`
//! and `list`, `frozenset` and `set` (the records of the items sorted as Python sorts `str`: by code point, which is the order of their UTF-8), `dict`
//! (in its order), then an `Enum` member (`isinstance`), then a dataclass instance (its fields in the order of `dataclasses.fields`, read once per class
//! from the shim's function). Anything else, a class among them, is a refusal of the PORT: the dispatcher runs the oracle, which raises
//! `ClipKeyUnsupported` as it does, and the clip goes past the memory.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::sync::Arc;

use cftuv_canon::fxhash::FxBuild;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyFrozenSet, PyInt, PyList, PySet, PyString, PyTuple, PyType};
use sha2::{Digest, Sha256};

/// Deeper nesting is declined (the oracle would recurse as deep; no argument of the clip is near it).
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

enum Kind {
    Int,
    Fraction,
    Str,
    Bool,
    NoneValue,
    Float,
    Tuple,
    List,
    FrozenSet,
    Set,
    Dict,
    Enum(String),
    Record { name: String, fields: Vec<(Py<PyString>, String)> },
    Unsupported(String),
}

/// The bound classes and the kind of every class met.
#[derive(Default)]
pub struct Host {
    fraction: Option<Py<PyType>>,
    enumeration: Option<Py<PyType>>,
    /// `fields(class) -> (name, ...)`: the names of `dataclasses.fields(class)` in their order (the shim's).
    fields: Option<Py<PyAny>>,
    dataclass_fields: Option<Py<PyString>>,
    numerator: Option<Py<PyString>>,
    denominator: Option<Py<PyString>>,
    name: Option<Py<PyString>>,
    kinds: HashMap<usize, (Py<PyType>, Arc<Kind>), FxBuild>,
}

fn qualname(kind: &Bound<'_, PyType>) -> PyResult<String> {
    Ok(kind.qualname()?.to_str()?.to_string())
}

impl Host {
    /// Binds `fractions.Fraction`, `enum.Enum` and the shim's function of the field names; forgets the kinds of the previous binding.
    pub fn bind(&mut self, py: Python<'_>, fraction: &Bound<'_, PyType>, enumeration: &Bound<'_, PyType>, fields: &Bound<'_, PyAny>) {
        self.fraction = Some(fraction.clone().unbind());
        self.enumeration = Some(enumeration.clone().unbind());
        self.fields = Some(fields.clone().unbind());
        self.dataclass_fields = Some(PyString::intern(py, "__dataclass_fields__").unbind());
        self.numerator = Some(PyString::intern(py, "numerator").unbind());
        self.denominator = Some(PyString::intern(py, "denominator").unbind());
        self.name = Some(PyString::intern(py, "name").unbind());
        self.kinds.clear();
    }

    fn classify(&self, py: Python<'_>, kind: &Bound<'_, PyType>) -> PyResult<Kind> {
        let (Some(fraction), Some(enumeration), Some(fields), Some(marker)) = (&self.fraction, &self.enumeration, &self.fields, &self.dataclass_fields) else {
            return Ok(Kind::Unsupported("the memo key classes are not bound".to_string()));
        };
        let is = |other: &Bound<'_, PyType>| kind.is(other);
        if is(&py.get_type::<PyInt>()) {
            return Ok(Kind::Int);
        }
        if kind.is(fraction.bind(py)) {
            return Ok(Kind::Fraction);
        }
        if is(&py.get_type::<PyString>()) {
            return Ok(Kind::Str);
        }
        if is(&py.get_type::<PyBool>()) {
            return Ok(Kind::Bool);
        }
        if kind.is(&py.None().bind(py).get_type()) {
            return Ok(Kind::NoneValue);
        }
        if is(&py.get_type::<PyFloat>()) {
            return Ok(Kind::Float);
        }
        if is(&py.get_type::<PyTuple>()) {
            return Ok(Kind::Tuple);
        }
        if is(&py.get_type::<PyList>()) {
            return Ok(Kind::List);
        }
        if is(&py.get_type::<PyFrozenSet>()) {
            return Ok(Kind::FrozenSet);
        }
        if is(&py.get_type::<PySet>()) {
            return Ok(Kind::Set);
        }
        if is(&py.get_type::<PyDict>()) {
            return Ok(Kind::Dict);
        }
        if kind.is_subclass(enumeration.bind(py))? {
            return Ok(Kind::Enum(qualname(kind)?));
        }
        // `dataclasses.is_dataclass(value) and not isinstance(value, type)`: the class of a class is a subclass of `type`
        if !kind.is_subclass(&py.get_type::<PyType>())? && kind.hasattr(marker.bind(py))? {
            let names = fields.bind(py).call1((kind,))?;
            let mut found = Vec::new();
            for item in names.try_iter()? {
                let item = item?;
                let text = item.cast::<PyString>()?.to_str()?.to_string();
                found.push((PyString::intern(py, &text).unbind(), text));
            }
            return Ok(Kind::Record { name: qualname(kind)?, fields: found });
        }
        Ok(Kind::Unsupported(format!("{} is not encodable in a clip memo key", qualname(kind)?)))
    }

    fn kind_of(&mut self, py: Python<'_>, value: &Bound<'_, PyAny>) -> PyResult<Arc<Kind>> {
        let kind = value.get_type();
        let address = kind.as_ptr() as usize;
        if let Some((_, found)) = self.kinds.get(&address) {
            return Ok(found.clone());
        }
        let found = Arc::new(self.classify(py, &kind)?);
        self.kinds.insert(address, (kind.unbind(), found.clone()));
        Ok(found)
    }

    fn put(&mut self, py: Python<'_>, out: &mut String, value: &Bound<'_, PyAny>, depth: usize) -> Walk<()> {
        if depth > DEPTH_LIMIT {
            return Err(Fail::Unsupported(format!("nesting deeper than {DEPTH_LIMIT}")));
        }
        let kind = self.kind_of(py, value)?;
        match &*kind {
            Kind::Int => {
                out.push('i');
                put_int(out, value)?;
                out.push(';');
            }
            Kind::Fraction => {
                let (numerator, denominator) = (self.numerator.as_ref().expect("bound"), self.denominator.as_ref().expect("bound"));
                out.push('q');
                put_int(out, &value.getattr(numerator.bind(py))?)?;
                out.push('/');
                put_int(out, &value.getattr(denominator.bind(py))?)?;
                out.push(';');
            }
            Kind::Str => {
                let Ok(text) = value.cast::<PyString>().map_err(PyErr::from)?.to_str() else {
                    return Err(Fail::Unsupported("a str that is not valid UTF-8 (a lone surrogate)".to_string()));
                };
                let _ = write!(out, "s{}:", text.chars().count());
                out.push_str(text);
                out.push(';');
            }
            Kind::Bool => out.push_str(if value.is_truthy()? { "T;" } else { "F;" }),
            Kind::NoneValue => out.push_str("N;"),
            Kind::Float => {
                out.push('f');
                out.push_str(value.call_method0("hex")?.cast::<PyString>().map_err(PyErr::from)?.to_str()?);
                out.push(';');
            }
            Kind::Tuple => {
                out.push('(');
                for item in value.cast::<PyTuple>().map_err(PyErr::from)?.iter() {
                    self.put(py, out, &item, depth + 1)?;
                }
                out.push(')');
            }
            Kind::List => {
                out.push('[');
                // a snapshot of the items: the walk calls no Python code that could change the list, but the borrow stays simple
                let items: Vec<Bound<'_, PyAny>> = value.cast::<PyList>().map_err(PyErr::from)?.iter().collect();
                for item in &items {
                    self.put(py, out, item, depth + 1)?;
                }
                out.push(']');
            }
            Kind::FrozenSet | Kind::Set => {
                let mut parts: Vec<String> = Vec::new();
                for item in value.try_iter()? {
                    let mut piece = String::new();
                    self.put(py, &mut piece, &item?, depth + 1)?;
                    parts.push(piece);
                }
                parts.sort_unstable();
                out.push('{');
                for part in &parts {
                    out.push_str(part);
                }
                out.push('}');
            }
            Kind::Dict => {
                out.push('<');
                let items: Vec<(Bound<'_, PyAny>, Bound<'_, PyAny>)> = value.cast::<PyDict>().map_err(PyErr::from)?.iter().collect();
                for (key, item) in &items {
                    self.put(py, out, key, depth + 1)?;
                    self.put(py, out, item, depth + 1)?;
                }
                out.push('>');
            }
            Kind::Enum(name) => {
                let member = value.getattr(self.name.as_ref().expect("bound").bind(py))?;
                let member = member.cast::<PyString>().map_err(PyErr::from)?.to_str()?.to_string();
                let _ = write!(out, "e{name}.{member};");
            }
            Kind::Record { name, fields } => {
                let _ = write!(out, "D{name}[");
                for (field, text) in fields {
                    out.push_str(text);
                    out.push('=');
                    let item = value.getattr(field.bind(py))?;
                    self.put(py, out, &item, depth + 1)?;
                }
                out.push(']');
            }
            Kind::Unsupported(text) => return Err(Fail::Unsupported(text.clone())),
        }
        Ok(())
    }

    /// `(True, hexdigest)` of `prefix` followed by the records of `policy`, `"triangles="` and `triangles`, and `"{name}="` and the record of every
    /// argument in `inputs`; `(False, why the port declined)`. A Python error raised while reading goes out as it is.
    pub fn key<'py>(&mut self, py: Python<'py>, prefix: &str, policy: &Bound<'py, PyAny>, triangles: &Bound<'py, PyAny>, inputs: &Bound<'py, PyDict>) -> PyResult<(bool, Bound<'py, PyAny>)> {
        if self.fields.is_none() {
            return Ok((false, PyString::new(py, "the memo key classes are not bound").into_any()));
        }
        let mut out = String::with_capacity(1 << 15);
        out.push_str(prefix);
        let walked = (|| -> Walk<()> {
            self.put(py, &mut out, policy, 0)?;
            out.push_str("triangles=");
            self.put(py, &mut out, triangles, 0)?;
            let items: Vec<(Bound<'_, PyAny>, Bound<'_, PyAny>)> = inputs.iter().collect();
            for (name, value) in &items {
                let name = name.cast::<PyString>().map_err(PyErr::from)?;
                out.push_str(name.to_str()?);
                out.push('=');
                self.put(py, &mut out, value, 0)?;
            }
            Ok(())
        })();
        match walked {
            Ok(()) => {
                let digest = Sha256::digest(out.as_bytes());
                let mut text = String::with_capacity(64);
                for byte in digest {
                    let _ = write!(text, "{byte:02x}");
                }
                Ok((true, PyString::new(py, &text).into_any()))
            }
            Err(Fail::Unsupported(text)) => Ok((false, PyString::new(py, &text).into_any())),
            Err(Fail::Py(error)) => Err(error),
        }
    }
}

/// The decimal text of a Python `int` (`f"{value}"`): `i64` written here, a larger one by `str`.
fn put_int(out: &mut String, value: &Bound<'_, PyAny>) -> Walk<()> {
    match value.extract::<i64>() {
        Ok(small) => {
            let _ = write!(out, "{small}");
        }
        Err(_) => out.push_str(value.str()?.to_str()?),
    }
    Ok(())
}
