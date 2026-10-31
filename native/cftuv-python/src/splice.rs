//! COMPOSITE_SPLICE_NATIVE_V1: the composite sample of the stream tick (`cftuv/envelope_width_composite.py`) over the VIEW_BYTES_V1 of its group domains.
//!
//! * `mesh_splice_weld(views, rebases, alpha, offset, law)`: `weld_vertices` over the group views at the width of the tick - a served domain's view moved by
//!   its certificate first (`rebased_view`), a recomputed one as it is - for `_weld_into` and `_splice_group`;
//! * `mesh_splice_rebuilt(views, rebases, alpha, mappings, patch_ids)`: what `_splice_faces` and `_splice_seams` read from the views of the domains whose
//!   layout is rebuilt - face sizes, corners in the new numbering, view owners, view seams in the new numbering and the fold seams (`cross_domain_seams`).
//!
//! The oracle is the host's Python path on the same views (`rebased_view`, `weld_vertices`, the face and seam loops): the same binary64 operations in the
//! same order (`p + s * dt`, `uv + (l - uv) * (dt / alpha)`; Rust never contracts to FMA), a zero `dt` leaves the view as it is (bits, `-0.0` included).
//! What the port would not repeat exactly it declines by a text and the Python path answers: bytes that are not VIEW_BYTES_V1, slopes that are not one per
//! vertex or loop (numpy would broadcast or raise), a number that is not finite, a local vertex outside its mapping, a negative mesh vertex, an owner
//! outside `int32`.

use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyTuple};

use crate::mesh_assembly::{bytes_of, cross_seams, declined, exact_float, floats, items, parse_view, weld, weld_domains, Decline, Law, ParsedView, Uv};

/// The slopes of one served domain's certificate: `position_slopes`, `loop_slopes` (float64 little-endian) and the width they were taken at.
struct Rebase<'a> {
    positions: &'a [u8],
    loops: &'a [u8],
    alpha: f64,
}

fn f64s(data: &[u8], what: &str) -> Result<Vec<f64>, Decline> {
    if data.len() % 8 != 0 {
        return Err(format!("the {what} are not whole float64 numbers"));
    }
    Ok(data.chunks_exact(8).map(|chunk| f64::from_le_bytes(chunk.try_into().unwrap())).collect())
}

/// `rebased_view` on a parsed view: positions `p + s * dt`, uvs `uv + (l - uv) * (dt / alpha)`, `dt = alpha - certificate alpha`.
fn rebase(view: &mut ParsedView<'_>, rebase: &Rebase<'_>, alpha: f64) -> Result<(), Decline> {
    let dt = alpha - rebase.alpha;
    if dt == 0.0 {
        return Ok(());
    }
    let slopes = f64s(rebase.positions, "position slopes")?;
    if slopes.len() != 3 * view.positions.len() {
        return Err("the position slopes are not one per vertex".to_string());
    }
    for (point, slope) in view.positions.iter_mut().zip(slopes.chunks_exact(3)) {
        for axis in 0..3 {
            point[axis] = point[axis] + slope[axis] * dt;
        }
    }
    let loops = f64s(rebase.loops, "loop slopes")?;
    if loops.len() != 2 * view.uvs.len() {
        return Err("the loop slopes are not one per loop".to_string());
    }
    let share = dt / alpha;
    for (uv, slope) in view.uvs.iter_mut().zip(loops.chunks_exact(2)) {
        for axis in 0..2 {
            uv[axis] = uv[axis] + (slope[axis] - uv[axis]) * share;
        }
    }
    if view.positions.iter().flatten().chain(view.uvs.iter().flatten()).any(|value| !value.is_finite()) {
        return Err("a rebased position or uv is not finite".to_string());
    }
    Ok(())
}

/// The bytes of the views and their rebases: `None` - the view as it is, `(position slopes, loop slopes, certificate alpha)` - moved to the tick.
fn read_group<'py>(views: &Bound<'py, PyAny>, rebases: &Bound<'py, PyAny>, alpha: &Bound<'py, PyAny>) -> Result<(Vec<Bound<'py, PyBytes>>, Vec<Option<(Bound<'py, PyBytes>, Bound<'py, PyBytes>, f64)>>, f64), Decline> {
    let data = items(views, "views")?
        .into_iter()
        .map(|item| item.cast_into::<PyBytes>().map_err(|_| "a view is not bytes (it travels as an object)".to_string()))
        .collect::<Result<Vec<_>, Decline>>()?;
    let mut moves = Vec::with_capacity(data.len());
    for item in items(rebases, "rebases")? {
        if item.is_none() {
            moves.push(None);
            continue;
        }
        let parts = items(&item, "a rebase")?;
        if parts.len() != 3 {
            return Err("a rebase is not (position slopes, loop slopes, alpha)".to_string());
        }
        let slopes = |at: usize| parts[at].clone().cast_into::<PyBytes>().map_err(|_| "the slopes of a rebase are not bytes".to_string());
        moves.push(Some((slopes(0)?, slopes(1)?, exact_float(&parts[2], "the alpha of a certificate")?)));
    }
    if moves.len() != data.len() {
        return Err("one rebase per view is expected".to_string());
    }
    let alpha = exact_float(alpha, "the alpha of the tick")?;
    if alpha <= 0.0 {
        return Err("the alpha of the tick is not positive".to_string());
    }
    Ok((data, moves, alpha))
}

/// The parsed views of the group at the width of the tick.
fn moved_views<'a>(slices: &[&'a [u8]], moves: &[Option<(&'a [u8], &'a [u8], f64)>], alpha: f64) -> Result<Vec<ParsedView<'a>>, Decline> {
    let mut parsed = Vec::with_capacity(slices.len());
    for (bytes, step) in slices.iter().zip(moves) {
        let mut view = parse_view(bytes)?;
        if let Some((positions, loops, at)) = step {
            rebase(&mut view, &Rebase { positions, loops, alpha: *at }, alpha)?;
        }
        parsed.push(view);
    }
    Ok(parsed)
}

fn borrowed<'a>(data: &'a [Bound<'_, PyBytes>], moves: &'a [Option<(Bound<'_, PyBytes>, Bound<'_, PyBytes>, f64)>]) -> (Vec<&'a [u8]>, Vec<Option<(&'a [u8], &'a [u8], f64)>>) {
    (
        data.iter().map(|item| item.as_bytes()).collect(),
        moves.iter().map(|step| step.as_ref().map(|(positions, loops, at)| (positions.as_bytes(), loops.as_bytes(), *at))).collect(),
    )
}

/// `(positions f64x3, index u32 (the local vertices of every domain in turn), vertex counts, partners)` of the weld of the group, or the text of a decline.
#[pyfunction]
pub fn mesh_splice_weld<'py>(py: Python<'py>, views: &Bound<'py, PyAny>, rebases: &Bound<'py, PyAny>, alpha: &Bound<'py, PyAny>, offset: &Bound<'py, PyAny>, law: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let read = || -> Result<_, Decline> {
        let numbers = floats(law, 4, "the weld law")?;
        let law = Law { limit: numbers[0], pivot: numbers[1], residual: numbers[2], sine: numbers[3] };
        let (data, moves, alpha) = read_group(views, rebases, alpha)?;
        Ok((data, moves, alpha, exact_float(offset, "the offset")?, law))
    };
    let (data, moves, alpha, offset, law) = match read() {
        Ok(found) => found,
        Err(reason) => return declined(py, reason),
    };
    let (slices, steps) = borrowed(&data, &moves);
    let computed = py.detach(|| -> Result<_, Decline> {
        let parsed = moved_views(&slices, &steps, alpha)?;
        let answer = weld(&weld_domains(&parsed), offset, &law);
        if answer.positions.iter().flatten().any(|value| !value.is_finite()) {
            return Err("a welded position is not finite".to_string());
        }
        Ok(answer)
    });
    let answer = match computed {
        Ok(found) => found,
        Err(reason) => return declined(py, reason),
    };
    let flat: Vec<f64> = answer.positions.iter().flatten().copied().collect();
    let positions = bytes_of(py, flat.into_iter(), f64::to_le_bytes)?;
    let numbers: Vec<u32> = answer.index.iter().flatten().map(|item| *item as u32).collect();
    let index = bytes_of(py, numbers.into_iter(), u32::to_le_bytes)?;
    let counts = PyTuple::new(py, answer.index.iter().map(|mapping| mapping.len()))?;
    let partners = PyTuple::new(py, answer.partners.iter().copied())?;
    Ok(PyTuple::new(py, [positions.into_any(), index.into_any(), counts.into_any(), partners.into_any()])?.into_any())
}

/// The faces, owners and seams of the rebuilt views in the new numbering (`mappings`: int64 little-endian, the mesh vertex of every local vertex).
struct Rebuilt {
    face_counts: Vec<usize>,
    sizes: Vec<i64>,
    corners: Vec<i64>,
    owners: Vec<i64>,
    seams: Vec<i64>,
    folds: Vec<i64>,
}

fn rebuild(parsed: &[ParsedView<'_>], mappings: &[Vec<i64>], patch_ids: &[i64]) -> Result<Rebuilt, Decline> {
    let mut found = Rebuilt { face_counts: Vec::new(), sizes: Vec::new(), corners: Vec::new(), owners: Vec::new(), seams: Vec::new(), folds: Vec::new() };
    let mut faces: Vec<Vec<i64>> = Vec::new();
    let mut uvs: Vec<Uv> = Vec::new();
    let mut face_domain: Vec<i64> = Vec::new();
    for ((view, mapping), patch_id) in parsed.iter().zip(mappings).zip(patch_ids) {
        let at = |local: usize| mapping.get(local).copied().ok_or_else(|| "a local vertex is outside its mapping".to_string());
        if view.owners.len() != view.faces.len() {
            return Err("the face owners of a view are not one per face".to_string());
        }
        if view.owners.iter().any(|owner| i32::try_from(*owner).is_err()) {
            return Err("a face owner is outside int32".to_string());
        }
        found.face_counts.push(view.faces.len());
        found.owners.extend_from_slice(&view.owners);
        for face in &view.faces {
            found.sizes.push(face.len() as i64);
            let corners = face.iter().map(|local| at(*local)).collect::<Result<Vec<i64>, Decline>>()?;
            found.corners.extend_from_slice(&corners);
            faces.push(corners);
        }
        for (a, b) in &view.seams {
            found.seams.push(at(*a)?);
            found.seams.push(at(*b)?);
        }
        uvs.extend_from_slice(&view.uvs);
        face_domain.extend(std::iter::repeat_n(*patch_id, view.faces.len()));
    }
    for (a, b) in cross_seams(&faces, &uvs, &face_domain)? {
        found.folds.push(a);
        found.folds.push(b);
    }
    Ok(found)
}

/// `(face counts, face sizes i64, corners i64, view owners i64, view seams i64x2, fold seams i64x2)` of the rebuilt domains, or the text of a decline.
#[pyfunction]
pub fn mesh_splice_rebuilt<'py>(
    py: Python<'py>,
    views: &Bound<'py, PyAny>,
    rebases: &Bound<'py, PyAny>,
    alpha: &Bound<'py, PyAny>,
    mappings: &Bound<'py, PyAny>,
    patch_ids: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let read = || -> Result<_, Decline> {
        let (data, moves, alpha) = read_group(views, rebases, alpha)?;
        let mut maps = Vec::with_capacity(data.len());
        for item in items(mappings, "mappings")? {
            let bytes = item.cast_into::<PyBytes>().map_err(|_| "a mapping is not bytes".to_string())?;
            let raw = bytes.as_bytes();
            if raw.len() % 8 != 0 {
                return Err("a mapping is not whole int64 numbers".to_string());
            }
            let values: Vec<i64> = raw.chunks_exact(8).map(|chunk| i64::from_le_bytes(chunk.try_into().unwrap())).collect();
            if values.iter().any(|value| *value < 0) {
                return Err("a mapping holds a negative mesh vertex".to_string());
            }
            maps.push(values);
        }
        let mut ids = Vec::new();
        for item in items(patch_ids, "patch_ids")? {
            if !item.is_exact_instance_of::<pyo3::types::PyInt>() {
                return Err("a patch id is not an exact int".to_string());
            }
            ids.push(item.extract::<i64>().map_err(|_| "a patch id is outside i64".to_string())?);
        }
        if maps.len() != data.len() || ids.len() != data.len() {
            return Err("one mapping and one patch id per view are expected".to_string());
        }
        Ok((data, moves, alpha, maps, ids))
    };
    let (data, moves, alpha, maps, ids) = match read() {
        Ok(found) => found,
        Err(reason) => return declined(py, reason),
    };
    let (slices, steps) = borrowed(&data, &moves);
    let computed = py.detach(|| -> Result<Rebuilt, Decline> { rebuild(&moved_views(&slices, &steps, alpha)?, &maps, &ids) });
    let found = match computed {
        Ok(found) => found,
        Err(reason) => return declined(py, reason),
    };
    let parts = [
        PyTuple::new(py, found.face_counts.iter().copied())?.into_any(),
        bytes_of(py, found.sizes.iter().copied(), i64::to_le_bytes)?.into_any(),
        bytes_of(py, found.corners.iter().copied(), i64::to_le_bytes)?.into_any(),
        bytes_of(py, found.owners.iter().copied(), i64::to_le_bytes)?.into_any(),
        bytes_of(py, found.seams.iter().copied(), i64::to_le_bytes)?.into_any(),
        bytes_of(py, found.folds.iter().copied(), i64::to_le_bytes)?.into_any(),
    ];
    Ok(PyTuple::new(py, parts)?.into_any())
}
