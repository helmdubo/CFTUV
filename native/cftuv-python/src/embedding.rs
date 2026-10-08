//! `_embedding._compute_source_snap_embedding_certificate` at the boundary: the arguments of the oracle read from Python objects, the numbers handed to `cftuv-embedding`, the counts back.
//!
//! The leaf is a pure function (no budget, no memory tables, no counters), so this entry point keeps no state: the shim hands the three kernel classes of every call, and a refusal
//! of the port is just an answer (`(2, reason)`): nothing of the process is touched either way. What the port carries is checked before anything is computed, by exact type:
//!
//! * `before` is a `dict` of `SourceVertexId -> (x, y, z)`; `after` is the same dict object (`UNSNAPPED_EXACT_V1`) or a `dict` over the same vertices; a coordinate is an exact `int` or `Fraction`;
//! * `faces` is an iterable of objects with `face_id.value` (a `str`), `vertex_cycle` and `edge_cycle` (tuples of ids of equal length); vertex ids are exactly `SourceVertexId`, edge ids exactly
//!   `PhysicalEdgeId`; corners are tuples or lists of three `SourceVertexId`.
//!
//! Anything else (a subclass, a float, a vertex of `after` that `before` lacks, ...) is `(2, reason)` and the caller lets the oracle answer. The answers are `(0, vertex ids, counts...)`,
//! `(1, edge id value)` for the oracle's `ValueError("physical edge ... has inconsistent endpoints")`, and `(2, reason)`; a vertex the positions lack is also `(2, reason)` (the oracle's `KeyError`).

use std::collections::HashMap;
use std::panic::{catch_unwind, AssertUnwindSafe};

use cftuv_core::num::UBig;
use cftuv_embedding::{compute, After, Corner, Face, Failure, Input, RPoint, Rational};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyInt, PyList, PyString, PyTuple};
use pyo3::{intern, IntoPyObjectExt};

use crate::pyobj::{ibig_from_int, ubig_from_int};

/// The reason the port declines a call (a message for the shim to name, never an exception of the oracle).
struct Declined(String);

impl From<PyErr> for Declined {
    fn from(error: PyErr) -> Declined {
        Declined(format!("reading the arguments failed: {error}"))
    }
}

fn decline<T>(message: impl Into<String>) -> Result<T, Declined> {
    Err(Declined(message.into()))
}

/// The classes the arguments must be instances of, exactly.
struct Classes<'a, 'py> {
    vertex: &'a Bound<'py, PyAny>,
    edge: &'a Bound<'py, PyAny>,
    fraction: &'a Bound<'py, PyAny>,
    int: Bound<'py, PyAny>,
}

fn is_exactly(object: &Bound<'_, PyAny>, class: &Bound<'_, PyAny>) -> bool {
    object.get_type().as_ptr() == class.as_ptr()
}

fn exact_str<'a>(object: &'a Bound<'_, PyAny>, what: &str) -> Result<&'a str, Declined> {
    if !object.is_exact_instance_of::<PyString>() {
        return decline(format!("{what} is not a str"));
    }
    match object.cast::<PyString>() {
        Ok(text) => text.to_str().map_err(|_| Declined(format!("{what} is not valid UTF-8"))),
        Err(_) => decline(format!("{what} is not a str")),
    }
}

fn value_of<'py>(object: &Bound<'py, PyAny>, what: &str) -> Result<String, Declined> {
    let value = object.getattr(intern!(object.py(), "value"))?;
    Ok(exact_str(&value, what)?.to_string())
}

fn rational_of(classes: &Classes<'_, '_>, object: &Bound<'_, PyAny>) -> Result<Rational, Declined> {
    if is_exactly(object, &classes.int) {
        return Ok(Rational::new(ibig_from_int(object)?, UBig::ONE));
    }
    if is_exactly(object, classes.fraction) {
        let py = object.py();
        let numerator = ibig_from_int(&object.getattr(intern!(py, "_numerator"))?)?;
        let denominator = ubig_from_int(&object.getattr(intern!(py, "_denominator"))?)?;
        if denominator == UBig::ZERO {
            return decline("a Fraction with a zero denominator");
        }
        return Ok(Rational::new(numerator, denominator));
    }
    decline(format!("a coordinate must be an int or a Fraction, not {}", object.get_type().name()?))
}

fn point_of(classes: &Classes<'_, '_>, object: &Bound<'_, PyAny>) -> Result<RPoint, Declined> {
    if !object.is_exact_instance_of::<PyTuple>() {
        return decline("a position must be a tuple of three coordinates");
    }
    let tuple = object.cast::<PyTuple>().map_err(|_| Declined("a position must be a tuple".into()))?;
    if tuple.len() != 3 {
        return decline(format!("a position must have three coordinates, not {}", tuple.len()));
    }
    Ok([rational_of(classes, &tuple.get_item(0)?)?, rational_of(classes, &tuple.get_item(1)?)?, rational_of(classes, &tuple.get_item(2)?)?])
}

/// One `dict` of positions as `(vertex id object, its value, its position)` rows in the dict's order.
fn rows_of<'py>(classes: &Classes<'_, 'py>, positions: &Bound<'py, PyAny>, what: &str) -> Result<Vec<(Bound<'py, PyAny>, String, RPoint)>, Declined> {
    if !positions.is_exact_instance_of::<PyDict>() {
        return decline(format!("{what} is not a dict"));
    }
    let dict = positions.cast::<PyDict>().map_err(|_| Declined(format!("{what} is not a dict")))?;
    let mut rows = Vec::with_capacity(dict.len());
    for (key, value) in dict.iter() {
        if !is_exactly(&key, classes.vertex) {
            return decline(format!("a key of {what} is not exactly a SourceVertexId"));
        }
        let name = value_of(&key, "a vertex id value")?;
        rows.push((key, name, point_of(classes, &value)?));
    }
    Ok(rows)
}

/// Vertex and edge ids of the faces and corners, numbered; the pointer cache keeps the objects alive so an address is never reused for another object during the call.
struct Interner<'py> {
    ranks: HashMap<String, u32>,
    unknown: HashMap<String, u32>,
    by_pointer: HashMap<usize, u32>,
    edge_slots: HashMap<String, u32>,
    edge_by_pointer: HashMap<usize, u32>,
    edge_names: Vec<String>,
    alive: Vec<Bound<'py, PyAny>>,
}

impl<'py> Interner<'py> {
    fn vertex(&mut self, classes: &Classes<'_, 'py>, object: &Bound<'py, PyAny>) -> Result<u32, Declined> {
        if let Some(found) = self.by_pointer.get(&(object.as_ptr() as usize)) {
            return Ok(*found);
        }
        if !is_exactly(object, classes.vertex) {
            return decline("a vertex id of a face or corner is not exactly a SourceVertexId");
        }
        let name = value_of(object, "a vertex id value")?;
        let known = self.ranks.len() as u32;
        let slot = match self.ranks.get(&name) {
            Some(rank) => *rank,
            None => {
                let next = known + self.unknown.len() as u32;
                *self.unknown.entry(name).or_insert(next)
            }
        };
        self.by_pointer.insert(object.as_ptr() as usize, slot);
        self.alive.push(object.clone());
        Ok(slot)
    }

    fn edge(&mut self, classes: &Classes<'_, 'py>, object: &Bound<'py, PyAny>) -> Result<u32, Declined> {
        if let Some(found) = self.edge_by_pointer.get(&(object.as_ptr() as usize)) {
            return Ok(*found);
        }
        if !is_exactly(object, classes.edge) {
            return decline("an edge id of a face is not exactly a PhysicalEdgeId");
        }
        let name = value_of(object, "an edge id value")?;
        let slot = match self.edge_slots.get(&name) {
            Some(slot) => *slot,
            None => {
                self.edge_names.push(name.clone());
                let slot = self.edge_names.len() as u32 - 1;
                self.edge_slots.insert(name, slot);
                slot
            }
        };
        self.edge_by_pointer.insert(object.as_ptr() as usize, slot);
        self.alive.push(object.clone());
        Ok(slot)
    }
}

fn cycle_of<'py>(object: Bound<'py, PyAny>, what: &str) -> Result<Bound<'py, PyTuple>, Declined> {
    if !object.is_exact_instance_of::<PyTuple>() {
        return decline(format!("{what} is not a tuple"));
    }
    object.cast_into::<PyTuple>().map_err(|_| Declined(format!("{what} is not a tuple")))
}

fn faces_of<'py>(classes: &Classes<'_, 'py>, interner: &mut Interner<'py>, faces: &Bound<'py, PyAny>) -> Result<Vec<Face>, Declined> {
    let py = faces.py();
    if !(faces.is_exact_instance_of::<PyTuple>() || faces.is_exact_instance_of::<PyList>()) {
        return decline("`faces` is not a tuple or a list");
    }
    let mut out = Vec::new();
    for face in faces.try_iter()? {
        let face = face?;
        let key = value_of(&face.getattr(intern!(py, "face_id"))?, "a face id value")?;
        let vertex_cycle = cycle_of(face.getattr(intern!(py, "vertex_cycle"))?, "a vertex_cycle")?;
        let edge_cycle = cycle_of(face.getattr(intern!(py, "edge_cycle"))?, "an edge_cycle")?;
        if vertex_cycle.len() != edge_cycle.len() {
            return decline("a face whose vertex and edge cycles differ in length");
        }
        let mut vertices = Vec::with_capacity(vertex_cycle.len());
        let mut edges = Vec::with_capacity(edge_cycle.len());
        for (vertex, edge) in vertex_cycle.iter().zip(edge_cycle.iter()) {
            vertices.push(interner.vertex(classes, &vertex)?);
            edges.push(interner.edge(classes, &edge)?);
        }
        out.push(Face { key, vertices, edges });
    }
    Ok(out)
}

fn corners_of<'py>(classes: &Classes<'_, 'py>, interner: &mut Interner<'py>, corners: &Bound<'py, PyAny>, what: &str) -> Result<Vec<Corner>, Declined> {
    if !(corners.is_exact_instance_of::<PyTuple>() || corners.is_exact_instance_of::<PyList>()) {
        return decline(format!("{what} is not a tuple or a list"));
    }
    let mut out = Vec::new();
    for corner in corners.try_iter()? {
        let corner = corner?;
        let items: Vec<Bound<'py, PyAny>> = if corner.is_exact_instance_of::<PyTuple>() || corner.is_exact_instance_of::<PyList>() {
            corner.try_iter()?.collect::<PyResult<_>>()?
        } else {
            return decline(format!("a corner of {what} is not a tuple or a list"));
        };
        if items.len() != 3 {
            return decline(format!("a corner of {what} does not have three vertices"));
        }
        let mut slots: Corner = [None; 3];
        for (slot, item) in slots.iter_mut().zip(items.iter()) {
            let found = interner.vertex(classes, item)?;
            *slot = (found < interner.ranks.len() as u32).then_some(found);
        }
        out.push(slots);
    }
    Ok(out)
}

/// The whole conversion: the numbered input and the vertex id objects in sorted order.
fn convert<'py>(classes: &Classes<'_, 'py>, before: &Bound<'py, PyAny>, after: &Bound<'py, PyAny>, faces: &Bound<'py, PyAny>, intended: &Bound<'py, PyAny>, unclassifiable: &Bound<'py, PyAny>) -> Result<(Input, Vec<Bound<'py, PyAny>>), Declined> {
    let mut rows = rows_of(classes, before, "`before`")?;
    rows.sort_by(|left, right| left.1.as_bytes().cmp(right.1.as_bytes()));
    let ranks: HashMap<String, u32> = rows.iter().enumerate().map(|(rank, row)| (row.1.clone(), rank as u32)).collect();
    if ranks.len() != rows.len() {
        return decline("two vertices of `before` have the same id value");
    }
    let after_points = if after.is(before) {
        After::Same
    } else {
        let mut found: Vec<Option<RPoint>> = vec![None; rows.len()];
        for (_, name, point) in rows_of(classes, after, "`after`")? {
            match ranks.get(&name) {
                Some(rank) if found[*rank as usize].is_none() => found[*rank as usize] = Some(point),
                _ => return decline("`after` has a vertex that `before` lacks"),
            }
        }
        if found.iter().any(Option::is_none) {
            return decline("`before` has a vertex that `after` lacks");
        }
        After::Points(found.into_iter().flatten().collect())
    };
    let mut interner = Interner { ranks, unknown: HashMap::new(), by_pointer: HashMap::new(), edge_slots: HashMap::new(), edge_by_pointer: HashMap::new(), edge_names: Vec::new(), alive: Vec::new() };
    for (rank, row) in rows.iter().enumerate() {
        interner.by_pointer.insert(row.0.as_ptr() as usize, rank as u32);
    }
    let faces = faces_of(classes, &mut interner, faces)?;
    let intended = corners_of(classes, &mut interner, intended, "`intended_corners`")?;
    let unclassifiable = corners_of(classes, &mut interner, unclassifiable, "`unclassifiable_corners`")?;
    let keys = rows.iter().map(|row| row.0.clone()).collect();
    let input = Input { before: rows.into_iter().map(|row| row.2).collect(), after: after_points, faces, edge_names: interner.edge_names, intended, unclassifiable };
    Ok((input, keys))
}

/// `snap_embedding(vertex_class, edge_class, fraction_class, before, after, faces, intended_corners, unclassifiable_corners)`:
/// `(0, vertex ids sorted, edge count, coincident pairs, collapsed edges, new intersections, unchanged unclassifiable corners, degenerated corners, pair tests)`,
/// `(1, edge id value)` (the oracle's `ValueError`) or `(2, reason)` (the port declines).
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn snap_embedding<'py>(
    py: Python<'py>,
    vertex_class: &Bound<'py, PyAny>,
    edge_class: &Bound<'py, PyAny>,
    fraction_class: &Bound<'py, PyAny>,
    before: &Bound<'py, PyAny>,
    after: &Bound<'py, PyAny>,
    faces: &Bound<'py, PyAny>,
    intended_corners: &Bound<'py, PyAny>,
    unclassifiable_corners: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let classes = Classes { vertex: vertex_class, edge: edge_class, fraction: fraction_class, int: py.get_type::<PyInt>().into_any() };
    let (input, keys) = match convert(&classes, before, after, faces, intended_corners, unclassifiable_corners) {
        Ok(found) => found,
        Err(Declined(reason)) => return (2u8, reason).into_bound_py_any(py),
    };
    let outcome = py.detach(|| catch_unwind(AssertUnwindSafe(|| compute(&input))));
    match outcome {
        Ok(Ok(report)) => {
            let counts = report.counts;
            let ids = PyTuple::new(py, keys)?;
            (
                0u8,
                ids,
                counts.source_edge_count,
                counts.newly_coincident_vertex_pair_count,
                counts.collapsed_nonzero_source_edge_count,
                counts.new_nonadjacent_edge_intersection_count,
                counts.unchanged_unclassifiable_source_corner_count,
                counts.degenerated_intended_right_corner_count,
                counts.exact_pair_test_count,
            )
                .into_bound_py_any(py)
        }
        Ok(Err(Failure::InconsistentEndpoints { edge })) => (1u8, input.edge_names[edge as usize].clone()).into_bound_py_any(py),
        Ok(Err(Failure::MissingVertex)) => (2u8, "a vertex of a face is not in the positions (the oracle raises KeyError)").into_bound_py_any(py),
        Err(panic) => Err(PyRuntimeError::new_err(format!("snap_embedding: native panic: {}", crate::panic_message(&panic)))),
    }
}
