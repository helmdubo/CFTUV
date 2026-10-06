//! The memory log of a whole operation, replayed on the host's real canonicalization tables.
//!
//! The native memory (`cftuv_canon`) records what it wrote while an operation ran; the host's tables are Python objects the operation must leave exactly as the
//! Python oracle would have, so the log is replayed on them in place, in the order Python performs the writes. Both whole operations (`clip_geometry`,
//! `coverage_at`) do it inside the one crossing: no log is encoded, shipped and decoded in Python.

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PySet, PyTuple};

use cftuv_canon::MemOp;
use cftuv_core::num::IBig;

use crate::pyobj::{int_from_ibig, int_from_ubig, Pool};

/// Which memory tables a call changed (the bits of the answer's `changed`): the shim takes its view of those again.
pub const CHANGED_REGISTRY: u8 = 1;
pub const CHANGED_FACTORIZATION: u8 = 2;
pub const CHANGED_SQUAREFREE: u8 = 4;
pub const CHANGED_SUPPORT: u8 = 8;

/// The process's real canonicalization tables, handed in by the shim: the sorted registry list and its set, and the three dictionaries.
pub type Tables<'py> = (Bound<'py, PyList>, Bound<'py, PySet>, Bound<'py, PyDict>, Bound<'py, PyDict>, Bound<'py, PyDict>);

/// `bisect.insort` (the registry list is sorted by the comparison of Python itself).
pub fn insort(py: Python<'_>) -> PyResult<Py<PyAny>> {
    Ok(py.import("bisect")?.getattr("insort")?.unbind())
}

/// The mutations of the canonicalization memory a whole operation made (`clip_geometry`, `coverage_at`), replayed IN PLACE on the process's real tables, in the order Python
/// performs them (`insort`, `d[key] = v`, `d[key] = d.pop(key)`, `del d[oldest]`, `clear`): returns which tables changed. Values
/// have exactly Python's types (tuples of int pairs, `(outside, inside)`, tuples of ints). The oldest key an eviction names is
/// checked against the real one: a mirror that disagrees with the real table is a named error, never a silent repair.
pub fn apply_log<'py>(py: Python<'py>, pool: &Pool, insort: &Py<PyAny>, tables: &Tables<'py>, log: Vec<MemOp>) -> PyResult<u8> {
    let (primes, prime_set, factorization, squarefree, support) = tables;
    let int = |value: &cftuv_core::num::UBig| int_from_ubig(py, pool, value);
    let mut changed = 0u8;
    for op in log {
        match op {
            MemOp::ResetAll => return Err(PyRuntimeError::new_err("cftuv_native: a memory reset inside a whole operation is not part of it")),
            MemOp::RegistryClear => {
                primes.call_method0("clear")?;
                prime_set.clear();
                changed |= CHANGED_REGISTRY;
            }
            MemOp::RegistryInsert(prime) => {
                let prime = int(&prime)?;
                insort.bind(py).call1((primes, &prime))?;
                prime_set.add(&prime)?;
                changed |= CHANGED_REGISTRY;
            }
            MemOp::FactorizationEvictOldest { key } => {
                let key = int(&key)?;
                let oldest = factorization.iter().next().map(|(oldest, _)| oldest);
                if !oldest.is_some_and(|oldest| oldest.eq(&key).unwrap_or(false)) {
                    return Err(PyRuntimeError::new_err("cftuv_native: the native eviction names an oldest factorization the real table does not hold"));
                }
                factorization.del_item(&key)?;
                changed |= CHANGED_FACTORIZATION;
            }
            MemOp::FactorizationInsert { key, pairs } => {
                let mut items = Vec::with_capacity(pairs.len());
                for (prime, power) in &pairs {
                    items.push(PyTuple::new(py, [int(prime)?, int_from_ibig(py, pool, &IBig::from(*power))?])?);
                }
                factorization.set_item(int(&key)?, PyTuple::new(py, items)?)?;
                changed |= CHANGED_FACTORIZATION;
            }
            MemOp::FactorizationTouch { key } => {
                let key = int(&key)?;
                let value = factorization.get_item(&key)?.ok_or_else(|| PyRuntimeError::new_err("cftuv_native: the native touch names a factorization the real table does not hold"))?;
                factorization.del_item(&key)?;
                factorization.set_item(&key, value)?;
                changed |= CHANGED_FACTORIZATION;
            }
            MemOp::SquarefreeInsert { key, value } => {
                squarefree.set_item(int(&key)?, PyTuple::new(py, [int(&value.0)?, int(&value.1)?])?)?;
                changed |= CHANGED_SQUAREFREE;
            }
            MemOp::SupportInsert { key, value } => {
                let items: Vec<Bound<'py, PyAny>> = value.iter().map(&int).collect::<PyResult<_>>()?;
                support.set_item(int(&key)?, PyTuple::new(py, items)?)?;
                changed |= CHANGED_SUPPORT;
            }
        }
    }
    Ok(changed)
}
