//! `graph_pack(value)` / `graph_unpack(data)` (COMMIT_BATCH_PACK_V1): the object graph a pool worker sends home on a COMMIT (the domain's `GeometryBatchV1`
//! and its offset normals, `envelope_production_result.deferred_result`) as one compact byte buffer instead of `pickle.dumps`.
//!
//! The oracle is pickle itself: `graph_unpack(graph_pack(x))` builds the graph `pickle.loads(pickle.dumps(x, protocol=5))` builds, object for object. The
//! same classes, the same field values (floats bit for bit, `Decimal` through its `str`, as `Decimal.__reduce__` does), the same insertion order of every
//! `frozenset`, and the same SHARING: an object pickle memoizes (`str`, `bytes`, a non-empty `tuple`, `frozenset`, `Decimal`, an enum member, a record) is
//! written once and referenced after, in the order pickle puts it in its memo (a record before its fields, a container after its items). What pickle
//! does not memoize (`None`, `bool`, `int`, `float`, the empty tuple) is written by value. So `pickle.dumps` of the unpacked graph is the bytes of
//! `pickle.dumps` of the unpickled one (the differential test checks exactly that).
//!
//! Carried: exact `None`, `bool`, `int` within `i64`, `float`, `str` (valid UTF-8), `bytes`, `tuple`, `frozenset`, `decimal.Decimal`; a member of an
//! `enum.Enum` subclass that keeps `Enum.__reduce_ex__` (unpacked as `cls(value)`, which is pickle's `REDUCE`); an instance of a FROZEN `slots` dataclass that
//! keeps `dataclasses._dataclass_getstate`/`_dataclass_setstate` and the default `__new__`/`__reduce_ex__`/`__reduce__` (unpacked as `object.__new__(cls)`
//! and `object.__setattr__` per field, which is pickle's `NEWOBJ` + `BUILD`). Anything else (a bigger int, a lone surrogate, a list, a dict, a subclass of
//! a builtin, a class with its own pickling, nesting deeper than [`DEPTH_LIMIT`]) is a DECLINE: `graph_pack` answers the reason as a `str`, and the host
//! pickles as before (named, never silent). A buffer `graph_unpack` cannot read is a `ValueError`.
//!
//! Format: `b"CGP1"`, the class table (varint count; per class: kind byte, module, qualname, varint field count, field names - each text a varint length and
//! UTF-8), then one value. A value is a tag byte and its payload (see the `T_*` constants); lengths and indices are LEB128 varints.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use cftuv_canon::fxhash::FxBuild;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyBytes, PyFloat, PyFrozenSet, PyInt, PyString, PyTuple, PyType};

use crate::pyobj;

/// Deeper nesting is declined (pickle would recurse as deep; no batch is near it).
pub const DEPTH_LIMIT: usize = 200;
pub const MAGIC: &[u8; 4] = b"CGP1";

const T_NONE: u8 = 0;
const T_FALSE: u8 = 1;
const T_TRUE: u8 = 2;
const T_INT: u8 = 3;
const T_FLOAT: u8 = 4;
const T_STR: u8 = 5;
const T_BYTES: u8 = 6;
const T_TUPLE: u8 = 7;
const T_FROZENSET: u8 = 8;
const T_DECIMAL: u8 = 9;
const T_ENUM: u8 = 10;
const T_RECORD: u8 = 11;
const T_REF: u8 = 12;
const T_EMPTY_TUPLE: u8 = 13;

const KIND_RECORD: u8 = 0;
const KIND_ENUM: u8 = 1;

type Decline = String;

enum Fail {
    Py(PyErr),
    Decline(Decline),
}

impl From<PyErr> for Fail {
    fn from(error: PyErr) -> Fail {
        Fail::Py(error)
    }
}

fn decline<T>(text: impl Into<String>) -> Result<T, Fail> {
    Err(Fail::Decline(text.into()))
}

fn put_varint(out: &mut Vec<u8>, mut value: u64) {
    while value >= 0x80 {
        out.push((value as u8) | 0x80);
        value >>= 7;
    }
    out.push(value as u8);
}

fn put_text(out: &mut Vec<u8>, text: &str) {
    put_varint(out, text.len() as u64);
    out.extend_from_slice(text.as_bytes());
}

/// The pickling classes of the running interpreter, looked up when a class is met for the first time in the process.
struct Stdlib<'py> {
    decimal: Bound<'py, PyAny>,
    enumeration: Bound<'py, PyAny>,
    enum_reduce_ex: Bound<'py, PyAny>,
    getstate: Option<Bound<'py, PyAny>>,
    setstate: Option<Bound<'py, PyAny>>,
    fields: Bound<'py, PyAny>,
    object_new: Bound<'py, PyAny>,
    object_reduce_ex: Bound<'py, PyAny>,
    object_reduce: Bound<'py, PyAny>,
}

impl<'py> Stdlib<'py> {
    fn load(py: Python<'py>) -> PyResult<Stdlib<'py>> {
        let dataclasses = py.import("dataclasses")?;
        let enumeration = py.import("enum")?.getattr("Enum")?;
        let object = py.get_type::<pyo3::types::PyAny>();
        Ok(Stdlib {
            decimal: py.import("decimal")?.getattr("Decimal")?,
            enum_reduce_ex: enumeration.getattr("__reduce_ex__")?,
            enumeration,
            getstate: dataclasses.getattr("_dataclass_getstate").ok(),
            setstate: dataclasses.getattr("_dataclass_setstate").ok(),
            fields: dataclasses.getattr("fields")?,
            object_new: object.getattr("__new__")?,
            object_reduce_ex: object.getattr("__reduce_ex__")?,
            object_reduce: object.getattr("__reduce__")?,
        })
    }

    fn same(class: &Bound<'py, PyType>, name: &str, wanted: &Bound<'py, PyAny>) -> PyResult<bool> {
        Ok(class.getattr(name)?.is(wanted))
    }

    /// The field names of a frozen slots dataclass pickled by `_dataclass_getstate`/`_dataclass_setstate`, or why the class is not carried.
    fn record_fields(&self, class: &Bound<'py, PyType>) -> PyResult<Result<Vec<String>, Decline>> {
        let name = class.qualname()?.to_string();
        let (Some(getstate), Some(setstate)) = (&self.getstate, &self.setstate) else {
            return Ok(Err("this interpreter's dataclasses has no _dataclass_getstate/_dataclass_setstate".to_string()));
        };
        if !class.hasattr("__dataclass_fields__")? || !class.hasattr("__slots__")? {
            return Ok(Err(format!("{name} is not a slots dataclass")));
        }
        if !Self::same(class, "__getstate__", getstate)? || !Self::same(class, "__setstate__", setstate)? {
            return Ok(Err(format!("{name} does not pickle by _dataclass_getstate/_dataclass_setstate (not frozen, or its own)")));
        }
        if !Self::same(class, "__new__", &self.object_new)? || !Self::same(class, "__reduce_ex__", &self.object_reduce_ex)? || !Self::same(class, "__reduce__", &self.object_reduce)? {
            return Ok(Err(format!("{name} has its own __new__ or __reduce_ex__ or __reduce__")));
        }
        if class.hasattr("__getnewargs_ex__")? || class.hasattr("__getnewargs__")? {
            return Ok(Err(format!("{name} has __getnewargs__")));
        }
        if class.is_subclass_of::<pyo3::types::PyList>()? || class.is_subclass_of::<pyo3::types::PyDict>()? || class.is_subclass_of::<PyTuple>()? {
            return Ok(Err(format!("{name} subclasses a builtin container")));
        }
        let mut names = Vec::new();
        for field in self.fields.call1((class,))?.try_iter()? {
            names.push(field?.getattr("name")?.extract::<String>()?);
        }
        Ok(Ok(names))
    }

    fn is_enum(&self, class: &Bound<'py, PyType>) -> PyResult<Result<(), Decline>> {
        if !class.is_subclass(&self.enumeration)? {
            return Ok(Err(format!("unsupported value: {}", class.qualname()?)));
        }
        if !Self::same(class, "__reduce_ex__", &self.enum_reduce_ex)? {
            return Ok(Err(format!("the enum {} has its own __reduce_ex__", class.qualname()?)));
        }
        Ok(Ok(()))
    }
}

/// What the packer does with the instances of one class, decided once per process: the class's entry of the table (kind, module, qualname, field names) and its interned
/// field names. The class itself is held by the cache, so its address stays its key.
enum Plan {
    Record { entry: Vec<u8>, names: Vec<Py<PyString>> },
    Enum { entry: Vec<u8> },
    Decimal,
    Unsupported(Decline),
}

/// A class of a buffer's table resolved and checked against the running interpreter, kept per process by the entry's bytes.
struct Resolved {
    kind: u8,
    class: Py<PyAny>,
    names: Vec<Py<PyString>>,
}

#[derive(Default)]
struct Cache {
    plans: HashMap<usize, (Py<PyType>, Arc<Plan>), FxBuild>,
    resolved: HashMap<Vec<u8>, Arc<Resolved>, FxBuild>,
}

/// The class plans and the resolved classes of the process. Never held across a call into Python (another thread could take the GIL and wait here).
static CACHE: Mutex<Option<Cache>> = Mutex::new(None);

fn with_cache<T>(action: impl FnOnce(&mut Cache) -> T) -> T {
    let mut guard = CACHE.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
    action(guard.get_or_insert_with(Cache::default))
}

fn entry_of(class: &Bound<'_, PyType>, kind: u8, names: &[String]) -> PyResult<Vec<u8>> {
    let mut entry = vec![kind];
    put_text(&mut entry, &class.module()?.to_string());
    put_text(&mut entry, &class.qualname()?.to_string());
    put_varint(&mut entry, names.len() as u64);
    for name in names {
        put_text(&mut entry, name);
    }
    Ok(entry)
}

fn classify(py: Python<'_>, class: &Bound<'_, PyType>) -> PyResult<Plan> {
    let lib = Stdlib::load(py)?;
    Ok(if class.is(&lib.decimal) {
        Plan::Decimal
    } else if class.hasattr("__dataclass_fields__")? {
        match lib.record_fields(class)? {
            Ok(names) => Plan::Record { entry: entry_of(class, KIND_RECORD, &names)?, names: names.iter().map(|name| PyString::intern(py, name).unbind()).collect() },
            Err(reason) => Plan::Unsupported(reason),
        }
    } else {
        match lib.is_enum(class)? {
            Ok(()) => Plan::Enum { entry: entry_of(class, KIND_ENUM, &[])? },
            Err(reason) => Plan::Unsupported(reason),
        }
    })
}

/// The plan of a class: from the process cache, or classified (outside the lock) and kept.
fn plan_of(py: Python<'_>, class: &Bound<'_, PyType>) -> PyResult<Arc<Plan>> {
    let key = class.as_ptr() as usize;
    if let Some(found) = with_cache(|cache| cache.plans.get(&key).map(|(_, plan)| plan.clone())) {
        return Ok(found);
    }
    let plan = Arc::new(classify(py, class)?);
    let held = class.clone().unbind();
    Ok(with_cache(|cache| cache.plans.entry(key).or_insert((held, plan)).1.clone()))
}

struct Packer<'py> {
    py: Python<'py>,
    value_name: Bound<'py, PyString>,
    /// The classes met in THIS buffer: their plan and their index in its table.
    kinds: HashMap<usize, (Arc<Plan>, u64), FxBuild>,
    classes: Vec<u8>,
    class_count: u64,
    memo: HashMap<usize, u64, FxBuild>,
    /// Containers being written: met again inside themselves would be a reference pickle resolves differently; declined.
    open: std::collections::HashSet<usize, FxBuild>,
    next: u64,
    /// The memoized objects stay alive (their addresses are the memo's keys) until the walk ends.
    held: Vec<Bound<'py, PyAny>>,
}

impl<'py> Packer<'py> {
    fn remember(&mut self, value: &Bound<'py, PyAny>) {
        self.memo.insert(value.as_ptr() as usize, self.next);
        self.next += 1;
        self.held.push(value.clone());
    }

    fn kind_of(&mut self, class: &Bound<'py, PyType>) -> PyResult<(Arc<Plan>, u64)> {
        let key = class.as_ptr() as usize;
        if let Some((plan, index)) = self.kinds.get(&key) {
            return Ok((plan.clone(), *index));
        }
        let plan = plan_of(self.py, class)?;
        let index = self.class_count;
        if let Plan::Record { entry, .. } | Plan::Enum { entry } = &*plan {
            self.classes.extend_from_slice(entry);
            self.class_count += 1;
        }
        self.kinds.insert(key, (plan.clone(), index));
        Ok((plan, index))
    }

    fn put(&mut self, out: &mut Vec<u8>, value: &Bound<'py, PyAny>, depth: usize) -> Result<(), Fail> {
        if depth > DEPTH_LIMIT {
            return decline("nesting deeper than the packer's limit");
        }
        if value.is_none() {
            out.push(T_NONE);
            return Ok(());
        }
        if value.is_exact_instance_of::<PyBool>() {
            out.push(if value.is_truthy()? { T_TRUE } else { T_FALSE });
            return Ok(());
        }
        if value.is_exact_instance_of::<PyInt>() {
            let Ok(small) = value.extract::<i64>() else {
                return decline("an int beyond 64 bits");
            };
            out.push(T_INT);
            out.extend_from_slice(&small.to_le_bytes());
            return Ok(());
        }
        if value.is_exact_instance_of::<PyFloat>() {
            out.push(T_FLOAT);
            out.extend_from_slice(&value.extract::<f64>()?.to_bits().to_le_bytes());
            return Ok(());
        }
        let key = value.as_ptr() as usize;
        if let Some(index) = self.memo.get(&key) {
            out.push(T_REF);
            put_varint(out, *index);
            return Ok(());
        }
        if self.open.contains(&key) {
            return decline("a container that contains itself");
        }
        if let Ok(text) = value.cast_exact::<PyString>() {
            let Ok(found) = text.to_str() else {
                return decline("a str with a lone surrogate");
            };
            out.push(T_STR);
            put_text(out, found);
            self.remember(value);
            return Ok(());
        }
        if let Ok(bytes) = value.cast_exact::<PyBytes>() {
            out.push(T_BYTES);
            put_varint(out, bytes.as_bytes().len() as u64);
            out.extend_from_slice(bytes.as_bytes());
            self.remember(value);
            return Ok(());
        }
        if let Ok(tuple) = value.cast_exact::<PyTuple>() {
            if tuple.is_empty() {
                out.push(T_EMPTY_TUPLE);
                return Ok(());
            }
            out.push(T_TUPLE);
            put_varint(out, tuple.len() as u64);
            self.open.insert(key);
            for item in tuple.iter() {
                self.put(out, &item, depth + 1)?;
            }
            self.open.remove(&key);
            self.remember(value);
            return Ok(());
        }
        if let Ok(set) = value.cast_exact::<PyFrozenSet>() {
            out.push(T_FROZENSET);
            put_varint(out, set.len() as u64);
            self.open.insert(key);
            for item in set.iter() {
                self.put(out, &item, depth + 1)?;
            }
            self.open.remove(&key);
            self.remember(value);
            return Ok(());
        }
        let class = value.get_type();
        let (plan, index) = self.kind_of(&class)?;
        match &*plan {
            Plan::Decimal => {
                out.push(T_DECIMAL);
                put_text(out, value.str()?.to_str()?);
                self.remember(value);
                Ok(())
            }
            Plan::Enum { .. } => {
                out.push(T_ENUM);
                put_varint(out, index);
                let held = value.getattr(&self.value_name)?;
                self.put(out, &held, depth + 1)?;
                self.remember(value);
                Ok(())
            }
            Plan::Record { names, .. } => {
                out.push(T_RECORD);
                put_varint(out, index);
                self.remember(value);
                for name in names {
                    let held = match value.getattr(name.bind(self.py)) {
                        Ok(found) => found,
                        Err(_) => return decline(format!("{} has a field that is not set", class.qualname()?)),
                    };
                    self.put(out, &held, depth + 1)?;
                }
                Ok(())
            }
            Plan::Unsupported(reason) => decline(reason.clone()),
        }
    }
}

/// `bytes` of the graph, or the reason (`str`) the packer declined it.
#[pyfunction]
pub fn graph_pack<'py>(py: Python<'py>, value: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let mut packer = Packer {
        py,
        value_name: PyString::intern(py, "_value_"),
        kinds: HashMap::default(),
        classes: Vec::new(),
        class_count: 0,
        memo: HashMap::default(),
        open: Default::default(),
        next: 0,
        held: Vec::new(),
    };
    let mut body = Vec::with_capacity(16 * 1024);
    match packer.put(&mut body, value, 0) {
        Ok(()) => {}
        Err(Fail::Decline(reason)) => return Ok(PyString::new(py, &reason).into_any()),
        Err(Fail::Py(error)) => return Err(error),
    }
    let mut out = Vec::with_capacity(body.len() + packer.classes.len() + 16);
    out.extend_from_slice(MAGIC);
    put_varint(&mut out, packer.class_count);
    out.extend_from_slice(&packer.classes);
    out.extend_from_slice(&body);
    Ok(PyBytes::new(py, &out).into_any())
}

// --------------------------------------------------------------------------
// unpacking
// --------------------------------------------------------------------------

fn malformed(text: impl Into<String>) -> PyErr {
    PyValueError::new_err(format!("COMMIT_BATCH_PACK_V1: {}", text.into()))
}

struct Reader<'a> {
    data: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn byte(&mut self) -> PyResult<u8> {
        let Some(found) = self.data.get(self.at) else {
            return Err(malformed("the buffer ends early"));
        };
        self.at += 1;
        Ok(*found)
    }

    fn varint(&mut self) -> PyResult<u64> {
        let mut value = 0u64;
        let mut shift = 0;
        loop {
            let byte = self.byte()?;
            if shift >= 64 || (shift == 63 && byte > 1) {
                return Err(malformed("a varint beyond 64 bits"));
            }
            value |= u64::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                return Ok(value);
            }
            shift += 7;
        }
    }

    fn take(&mut self, count: usize) -> PyResult<&'a [u8]> {
        let end = self.at.checked_add(count).filter(|end| *end <= self.data.len()).ok_or_else(|| malformed("the buffer ends early"))?;
        let found = &self.data[self.at..end];
        self.at = end;
        Ok(found)
    }

    fn length(&mut self) -> PyResult<usize> {
        let value = self.varint()?;
        if value > (self.data.len() - self.at) as u64 {
            return Err(malformed("a length beyond the buffer"));
        }
        Ok(value as usize)
    }

    fn text(&mut self) -> PyResult<&'a str> {
        let count = self.length()?;
        std::str::from_utf8(self.take(count)?).map_err(|_| malformed("a text that is not UTF-8"))
    }
}


fn resolve<'py>(py: Python<'py>, module: &str, qualname: &str) -> PyResult<Bound<'py, PyAny>> {
    let mut found = py.import(module)?.into_any();
    for part in qualname.split('.') {
        found = found.getattr(part)?;
    }
    Ok(found)
}

struct Unpacker<'py, 'a> {
    py: Python<'py>,
    reader: Reader<'a>,
    classes: Vec<Arc<Resolved>>,
    memo: Vec<Bound<'py, PyAny>>,
    decimal: Bound<'py, PyAny>,
}

impl<'py, 'a> Unpacker<'py, 'a> {
    fn read_classes(&mut self) -> PyResult<()> {
        let count = self.reader.length()?;
        for _ in 0..count {
            let begin = self.reader.at;
            let kind = self.reader.byte()?;
            let module = self.reader.text()?;
            let qualname = self.reader.text()?;
            let fields = self.reader.length()?;
            let mut names = Vec::with_capacity(fields);
            for _ in 0..fields {
                names.push(self.reader.text()?.to_string());
            }
            let entry = &self.reader.data[begin..self.reader.at];
            if let Some(found) = with_cache(|cache| cache.resolved.get(entry).cloned()) {
                self.classes.push(found);
                continue;
            }
            let resolved = Arc::new(self.resolve_entry(kind, module, qualname, names)?);
            let entry = entry.to_vec();
            self.classes.push(with_cache(|cache| cache.resolved.entry(entry).or_insert(resolved).clone()));
        }
        Ok(())
    }

    fn resolve_entry(&self, kind: u8, module: &str, qualname: &str, names: Vec<String>) -> PyResult<Resolved> {
        let lib = Stdlib::load(self.py)?;
        let class = resolve(self.py, module, qualname)?;
        let Ok(class_type) = class.cast::<PyType>() else {
            return Err(malformed(format!("{module}.{qualname} is not a class")));
        };
        match kind {
            KIND_RECORD => {
                let wanted = lib.record_fields(class_type)?.map_err(|reason| malformed(format!("{module}.{qualname}: {reason}")))?;
                if wanted != names {
                    return Err(malformed(format!("the fields of {module}.{qualname} are {wanted:?} here, {names:?} in the buffer")));
                }
            }
            KIND_ENUM => {
                lib.is_enum(class_type)?.map_err(|reason| malformed(format!("{module}.{qualname}: {reason}")))?;
            }
            other => return Err(malformed(format!("an unknown class kind {other}"))),
        }
        let names = names.iter().map(|name| PyString::intern(self.py, name).unbind()).collect();
        Ok(Resolved { kind, class: class.unbind(), names })
    }

    fn class(&self, index: u64, kind: u8) -> PyResult<Arc<Resolved>> {
        let found = self.classes.get(index as usize).ok_or_else(|| malformed("a class index beyond the table"))?;
        if found.kind != kind {
            return Err(malformed(if kind == KIND_ENUM { "an enum member of a record class" } else { "a record of an enum class" }));
        }
        Ok(found.clone())
    }

    fn value(&mut self, depth: usize) -> PyResult<Bound<'py, PyAny>> {
        if depth > DEPTH_LIMIT {
            return Err(malformed("nesting deeper than the packer's limit"));
        }
        let py = self.py;
        let tag = self.reader.byte()?;
        Ok(match tag {
            T_NONE => py.None().into_bound(py),
            T_FALSE => PyBool::new(py, false).to_owned().into_any(),
            T_TRUE => PyBool::new(py, true).to_owned().into_any(),
            T_INT => {
                let bytes: [u8; 8] = self.reader.take(8)?.try_into().map_err(|_| malformed("a short int"))?;
                PyInt::new(py, i64::from_le_bytes(bytes)).into_any()
            }
            T_FLOAT => {
                let bytes: [u8; 8] = self.reader.take(8)?.try_into().map_err(|_| malformed("a short float"))?;
                PyFloat::new(py, f64::from_bits(u64::from_le_bytes(bytes))).into_any()
            }
            T_STR => {
                let found = PyString::new(py, self.reader.text()?).into_any();
                self.memo.push(found.clone());
                found
            }
            T_BYTES => {
                let count = self.reader.length()?;
                let found = PyBytes::new(py, self.reader.take(count)?).into_any();
                self.memo.push(found.clone());
                found
            }
            T_EMPTY_TUPLE => PyTuple::empty(py).into_any(),
            T_TUPLE | T_FROZENSET => {
                let count = self.reader.length()?;
                let mut items = Vec::with_capacity(count);
                for _ in 0..count {
                    items.push(self.value(depth + 1)?);
                }
                let found = if tag == T_TUPLE { PyTuple::new(py, items)?.into_any() } else { PyFrozenSet::new(py, items)?.into_any() };
                self.memo.push(found.clone());
                found
            }
            T_DECIMAL => {
                let text = self.reader.text()?;
                let found = self.decimal.call1((text,))?;
                self.memo.push(found.clone());
                found
            }
            T_ENUM => {
                let index = self.reader.varint()?;
                let class = self.class(index, KIND_ENUM)?;
                let held = self.value(depth + 1)?;
                let found = class.class.bind(py).call1((held,))?;
                self.memo.push(found.clone());
                found
            }
            T_RECORD => {
                let index = self.reader.varint()?;
                let class = self.class(index, KIND_RECORD)?;
                let found = pyobj::alloc(py, &class.class)?;
                self.memo.push(found.clone());
                for name in &class.names {
                    let held = self.value(depth + 1)?;
                    pyobj::set_slot(&found, name, &held)?;
                }
                found
            }
            T_REF => {
                let index = self.reader.varint()?;
                self.memo.get(index as usize).cloned().ok_or_else(|| malformed("a reference beyond the memo"))?
            }
            other => return Err(malformed(format!("an unknown tag {other}"))),
        })
    }
}

/// The graph `graph_pack` wrote; `ValueError` for a buffer it did not write (a class that moved or changed its fields included).
#[pyfunction]
pub fn graph_unpack<'py>(py: Python<'py>, data: &[u8]) -> PyResult<Bound<'py, PyAny>> {
    if data.len() < MAGIC.len() || &data[..MAGIC.len()] != MAGIC {
        return Err(malformed("not a COMMIT_BATCH_PACK_V1 buffer"));
    }
    let decimal = py.import("decimal")?.getattr("Decimal")?;
    let mut unpacker = Unpacker { py, reader: Reader { data, at: MAGIC.len() }, classes: Vec::new(), memo: Vec::new(), decimal };
    unpacker.read_classes()?;
    let found = unpacker.value(0)?;
    if unpacker.reader.at != data.len() {
        return Err(malformed("bytes after the value"));
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::{put_varint, Reader};

    #[test]
    fn varints_round_trip_at_their_edges() {
        for value in [0u64, 1, 127, 128, 300, 16383, 16384, u32::MAX as u64, u64::MAX] {
            let mut out = Vec::new();
            put_varint(&mut out, value);
            let mut reader = Reader { data: &out, at: 0 };
            assert_eq!(reader.varint().unwrap(), value);
            assert_eq!(reader.at, out.len());
        }
    }
}
