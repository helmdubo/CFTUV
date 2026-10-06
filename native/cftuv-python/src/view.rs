//! The identity view of the host's canonicalization tables: what the session's mirror held after the last call, as the very objects of the real tables.
//!
//! A call must know whether the real tables (`_KNOWN_PRIMES`, `_FACTORIZATION_MEMO`, `_SQUAREFREE_MEMO`, `_PRIME_SUPPORT_MEMO`) changed since the mirror last
//! saw them, and Python code mutates them freely between native calls, so the answer is a comparison of every entry, order included. Doing it as lists (a copy of
//! each table and two comparisons per call) costs about ten nanoseconds per entry and the calls that do the least work pay the most for it. Here the view keeps a
//! strong reference to every key and value it saw, so the check is one pointer comparison per entry with no allocation: a table whose entries are the same objects in
//! the same order is unchanged, which is also what the full comparison would say. Anything else (a new, moved, removed or replaced entry) is only "not provably
//! unchanged": the shim then runs its full comparison over the lists of this view (`lists`), exactly as before, and takes the view anew after the call.

use std::ptr;

use pyo3::ffi;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyTuple};

type Entries = Vec<(Py<PyAny>, Py<PyAny>)>;

#[derive(Default)]
pub struct View {
    primes: Vec<Py<PyAny>>,
    tables: [Entries; 3],
}

impl View {
    /// Whether every table of the host is, entry by entry and in order, made of the objects this view holds.
    pub fn matches(&self, primes: &Bound<'_, PyList>, tables: [&Bound<'_, PyDict>; 3]) -> bool {
        if primes.len() != self.primes.len() {
            return false;
        }
        for (index, held) in self.primes.iter().enumerate() {
            // SAFETY: `index` is below the length just checked; `PyList_GetItem` gives a borrowed reference, nothing runs Python code while it is compared.
            if unsafe { ffi::PyList_GetItem(primes.as_ptr(), index as ffi::Py_ssize_t) } != held.as_ptr() {
                return false;
            }
        }
        tables.iter().zip(&self.tables).all(|(table, held)| Self::same_entries(table, held))
    }

    fn same_entries(table: &Bound<'_, PyDict>, held: &Entries) -> bool {
        if table.len() != held.len() {
            return false;
        }
        let (mut position, mut key, mut value): (ffi::Py_ssize_t, *mut ffi::PyObject, *mut ffi::PyObject) = (0, ptr::null_mut(), ptr::null_mut());
        let mut index = 0;
        // SAFETY: the dictionary is not changed while it is walked (the GIL is held and no Python code runs); `key` and `value` are borrowed references.
        while unsafe { ffi::PyDict_Next(table.as_ptr(), &mut position, &mut key, &mut value) } != 0 {
            match held.get(index) {
                Some((held_key, held_value)) if held_key.as_ptr() == key && held_value.as_ptr() == value => index += 1,
                _ => return false,
            }
        }
        index == held.len()
    }

    /// Takes the view of the tables in `mask` (bit 0 the registry, bits 1..3 the dictionaries in the order of `matches`) from the host's tables as they are now.
    pub fn capture(&mut self, py: Python<'_>, primes: &Bound<'_, PyList>, tables: [&Bound<'_, PyDict>; 3], mask: u8) {
        if mask & 1 != 0 {
            self.primes = primes.iter().map(Bound::unbind).collect();
        }
        for (index, table) in tables.iter().enumerate() {
            if mask & (2 << index) != 0 {
                self.tables[index] = Self::entries_of(py, table);
            }
        }
    }

    fn entries_of(py: Python<'_>, table: &Bound<'_, PyDict>) -> Entries {
        let mut out = Vec::with_capacity(table.len());
        let (mut position, mut key, mut value): (ffi::Py_ssize_t, *mut ffi::PyObject, *mut ffi::PyObject) = (0, ptr::null_mut(), ptr::null_mut());
        // SAFETY: as in `same_entries`; the references are taken (`from_borrowed_ptr`), so the view keeps them alive.
        while unsafe { ffi::PyDict_Next(table.as_ptr(), &mut position, &mut key, &mut value) } != 0 {
            out.push(unsafe { (Bound::from_borrowed_ptr(py, key).unbind(), Bound::from_borrowed_ptr(py, value).unbind()) });
        }
        out
    }

    /// The view as Python lists: `(primes, (factorization keys, values), (squarefree keys, values), (support keys, values))`.
    pub fn lists<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        let primes = PyList::new(py, self.primes.iter().map(|item| item.bind(py).clone()))?;
        let mut parts = vec![primes.into_any()];
        for entries in &self.tables {
            let keys = PyList::new(py, entries.iter().map(|(key, _)| key.bind(py).clone()))?;
            let values = PyList::new(py, entries.iter().map(|(_, value)| value.bind(py).clone()))?;
            parts.push(PyTuple::new(py, [keys.into_any(), values.into_any()])?.into_any());
        }
        PyTuple::new(py, parts)
    }
}
