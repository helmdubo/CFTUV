//! The checks of the assembled decal mesh in the main process (MESH_ASSEMBLY_NATIVE_V1, slice B1): `kernel/src/cftuv_envelope/mesh_checks.py` is the oracle.
//!
//! * `mesh_off_plane(positions, faces)`: `off_plane_measure`, `(worst metres, faces off by a nanometre)`;
//! * `mesh_cross_seams(faces, uvs, face_domain)`: `cross_domain_seams`, the sorted edges between two faces of different domains with a UV jump;
//! * `mesh_half_edges(faces)`: `half_edge_conflicts`, the directed edges of two faces.
//!
//! CPython semantics, bit for bit: binary64 subtraction and sums left to right (Rust never contracts to FMA), `max` keeps the first of equals, `round` to
//! even (`round_ties_even`), `!=` over UV numbers is numeric (`-0.0 == 0.0`). What the port does not carry it declines by a text (the shim raises
//! `NativePortUnsupported` and the oracle answers): a coordinate or UV that is not an exact `float` (an `int` subtracts exactly in Python, a subclass may
//! override the operators), a non-finite number (`max` over a NaN depends on order), an index outside the arrays or negative (Python wraps it or raises),
//! a vertex or domain id that is not an `int` in `i64`. Declining is always safe: the oracle answers the same input.

use std::collections::HashMap;

use pyo3::prelude::*;
use pyo3::types::{PyFloat, PyList, PyTuple};

type Decline = String;

fn exact_float(item: &Bound<'_, PyAny>, what: &str) -> Result<f64, Decline> {
    if !item.is_exact_instance_of::<PyFloat>() {
        return Err(format!("{what} is not an exact float"));
    }
    let value = item.cast::<PyFloat>().map_err(|_| format!("{what} is not a float"))?.value();
    if !value.is_finite() {
        return Err(format!("{what} is not finite"));
    }
    Ok(value)
}

fn items<'py>(sequence: &Bound<'py, PyAny>, what: &str) -> Result<Vec<Bound<'py, PyAny>>, Decline> {
    if let Ok(list) = sequence.cast::<PyList>() {
        return Ok(list.iter().collect());
    }
    if let Ok(tuple) = sequence.cast::<PyTuple>() {
        return Ok(tuple.iter().collect());
    }
    Err(format!("{what} is not a list or a tuple"))
}

fn floats(item: &Bound<'_, PyAny>, size: usize, what: &str) -> Result<Vec<f64>, Decline> {
    let parts = items(item, what)?;
    if parts.len() != size {
        return Err(format!("{what} has {} numbers, not {size}", parts.len()));
    }
    parts.iter().map(|part| exact_float(part, what)).collect()
}

fn read_faces(faces: &Bound<'_, PyAny>) -> Result<Vec<Vec<i64>>, Decline> {
    items(faces, "faces")?
        .iter()
        .map(|loop_| {
            items(loop_, "a face")?
                .iter()
                .map(|vertex| {
                    if !vertex.is_exact_instance_of::<pyo3::types::PyInt>() {
                        return Err("a face vertex is not an exact int".to_string());
                    }
                    vertex.extract::<i64>().map_err(|_| "a face vertex is outside i64".to_string())
                })
                .collect()
        })
        .collect()
}

type Point = [f64; 3];

fn sub(a: &Point, b: &Point) -> Point {
    [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
}

fn cross(l: &Point, r: &Point) -> Point {
    [l[1] * r[2] - l[2] * r[1], l[2] * r[0] - l[0] * r[2], l[0] * r[1] - l[1] * r[0]]
}

fn dot(l: &Point, r: &Point) -> f64 {
    l[0] * r[0] + l[1] * r[1] + l[2] * r[2]
}

/// `off_plane_measure` over read arrays; `Err` declines.
pub fn off_plane(positions: &[Point], faces: &[Vec<i64>]) -> Result<(f64, u64), Decline> {
    let mut worst = 0.0f64;
    let mut off = 0u64;
    for face in faces {
        if face.len() < 4 {
            continue;
        }
        let mut points = Vec::with_capacity(face.len());
        for &index in face {
            if index < 0 || index as u64 >= positions.len() as u64 {
                return Err(format!("face vertex {index} is outside the positions"));
            }
            points.push(positions[index as usize]);
        }
        let origin = points[0];
        let mut normal: Point = [0.0, 0.0, 0.0];
        for index in 1..points.len() - 1 {
            let part = cross(&sub(&origin, &points[index]), &sub(&origin, &points[index + 1]));
            normal = [normal[0] + part[0], normal[1] + part[1], normal[2] + part[2]];
        }
        let length = dot(&normal, &normal).sqrt();
        if !length.is_finite() {
            return Err("a face normal is not finite".to_string());
        }
        if length == 0.0 {
            continue;
        }
        let mut deviation = f64::NEG_INFINITY;
        for (at, point) in points[1..].iter().enumerate() {
            let value = dot(&sub(&origin, point), &normal).abs() / length;
            if !value.is_finite() {
                return Err("a face deviation is not finite".to_string());
            }
            if at == 0 || value > deviation {
                deviation = value;
            }
        }
        if deviation > worst {
            worst = deviation;
        }
        let scaled = deviation * 1e9;
        if !scaled.is_finite() {
            return Err("a face deviation in nanometres is not finite (Python's `round` raises)".to_string());
        }
        if scaled.round_ties_even() > 0.0 {
            off += 1;
        }
    }
    Ok((worst, off))
}

type Uv = [f64; 2];

/// `cross_domain_seams` over read arrays; `Err` declines.
pub fn cross_seams(faces: &[Vec<i64>], uvs: &[Uv], face_domain: &[i64]) -> Result<Vec<(i64, i64)>, Decline> {
    // edge -> (sides seen, first side, second side); a side is (domain, uv at the smaller vertex, uv at the larger)
    let mut edges: HashMap<(i64, i64), (u32, (i64, Uv, Uv), (i64, Uv, Uv))> = HashMap::new();
    let mut cursor = 0usize;
    for (number, face) in faces.iter().enumerate() {
        let size = face.len();
        let domain = *face_domain.get(number).ok_or("a face has no domain")?;
        if cursor + size > uvs.len() {
            return Err("the faces have more loops than the uvs".to_string());
        }
        for position in 0..size {
            let next = (position + 1) % size;
            let (mut first, mut second) = (face[position], face[next]);
            let (mut uv_first, mut uv_second) = (uvs[cursor + position], uvs[cursor + next]);
            if first > second {
                std::mem::swap(&mut first, &mut second);
                std::mem::swap(&mut uv_first, &mut uv_second);
            }
            let side = (domain, uv_first, uv_second);
            edges
                .entry((first, second))
                .and_modify(|entry| {
                    if entry.0 == 1 {
                        entry.2 = side;
                    }
                    entry.0 += 1;
                })
                .or_insert((1, side, side));
        }
        cursor += size;
    }
    let mut found: Vec<(i64, i64)> = edges
        .into_iter()
        .filter(|(_, (count, a, b))| *count == 2 && a.0 != b.0 && (a.1 != b.1 || a.2 != b.2))
        .map(|(edge, _)| edge)
        .collect();
    found.sort_unstable();
    Ok(found)
}

/// `half_edge_conflicts` over read faces.
pub fn half_edges(faces: &[Vec<i64>]) -> u64 {
    let mut seen: HashMap<(i64, i64), u32> = HashMap::new();
    for face in faces {
        let size = face.len();
        for position in 0..size {
            *seen.entry((face[position], face[(position + 1) % size])).or_insert(0) += 1;
        }
    }
    seen.values().filter(|count| **count > 1).count() as u64
}

fn declined<'py>(py: Python<'py>, reason: Decline) -> PyResult<Bound<'py, PyAny>> {
    Ok(pyo3::types::PyString::new(py, &reason).into_any())
}

/// `(worst, off)` or the text of a decline (a `str`).
#[pyfunction]
pub fn mesh_off_plane<'py>(py: Python<'py>, positions: &Bound<'py, PyAny>, faces: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let read = || -> Result<(Vec<Point>, Vec<Vec<i64>>), Decline> {
        let points = items(positions, "positions")?
            .iter()
            .map(|item| floats(item, 3, "a position").map(|v| [v[0], v[1], v[2]]))
            .collect::<Result<Vec<Point>, Decline>>()?;
        Ok((points, read_faces(faces)?))
    };
    match read().and_then(|(points, faces)| off_plane(&points, &faces)) {
        Ok((worst, off)) => Ok((worst, off).into_pyobject(py)?.into_any()),
        Err(reason) => declined(py, reason),
    }
}

/// The sorted fold edges `((a, b), ...)` or the text of a decline.
#[pyfunction]
pub fn mesh_cross_seams<'py>(py: Python<'py>, faces: &Bound<'py, PyAny>, uvs: &Bound<'py, PyAny>, face_domain: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let read = || -> Result<(Vec<Vec<i64>>, Vec<Uv>, Vec<i64>), Decline> {
        let loops = read_faces(faces)?;
        let uv = items(uvs, "uvs")?
            .iter()
            .map(|item| floats(item, 2, "a uv").map(|v| [v[0], v[1]]))
            .collect::<Result<Vec<Uv>, Decline>>()?;
        let domains = items(face_domain, "face_domain")?
            .iter()
            .map(|item| {
                if !item.is_exact_instance_of::<pyo3::types::PyInt>() {
                    return Err("a face domain is not an exact int".to_string());
                }
                item.extract::<i64>().map_err(|_| "a face domain is outside i64".to_string())
            })
            .collect::<Result<Vec<i64>, Decline>>()?;
        Ok((loops, uv, domains))
    };
    match read().and_then(|(loops, uv, domains)| cross_seams(&loops, &uv, &domains)) {
        Ok(edges) => Ok(PyTuple::new(py, edges)?.into_any()),
        Err(reason) => declined(py, reason),
    }
}

/// The number of directed edges of two faces or the text of a decline.
#[pyfunction]
pub fn mesh_half_edges<'py>(py: Python<'py>, faces: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    match read_faces(faces) {
        Ok(loops) => Ok(half_edges(&loops).into_pyobject(py)?.into_any()),
        Err(reason) => declined(py, reason),
    }
}
