//! VIEW_BYTES_NATIVE_V1: the bytes of the view of a materialized domain, `envelope_view_bytes.encode_view(build_domain_view(result), structure_token(view))`,
//! read from the batch in one pass (the host's Python is the oracle, its digest is pinned by `tests/test_native_domain_view.py`).
//!
//! The answer is the oracle's bytes or the text of a decline (a `str`): every view the oracle would refuse to the writer (`ADAPTER_*`: empty batch, normal
//! missing or opposing the source, a value that is not finite, a face vertex missing), every value the encoder would not repeat exactly (a number that is not a
//! `float`, an index past `uint32`, a text that is not a `str`) and every input the port does not read like Python (an ownership claim whose ordinal is not plain
//! digits, a reference whose `repr` is not plain ASCII, a chain name without `:`) is declined: the oracle then answers, as it did. A Python error met while
//! reading is a decline too (nothing was written).
//!
//! The structure token is `blake2b(repr((faces, seams, refs, len(uvs))), digest_size=10)`: the `repr` is written here (tuples of `int`, `str` and `None`), the
//! hash is the host's `hashlib.blake2b` (passed in by the shim).

use std::collections::{BTreeSet, HashMap};
use std::fmt::Write as _;

use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyFloat, PyFrozenSet, PyInt, PyList, PyString, PyTuple};

use cftuv_canon::fxhash::FxBuild;

enum Fail {
    Decline(String),
    Py(PyErr),
}

impl From<PyErr> for Fail {
    fn from(error: PyErr) -> Fail {
        Fail::Py(error)
    }
}

type Read<T> = Result<T, Fail>;

fn decline<T>(reason: impl Into<String>) -> Read<T> {
    Err(Fail::Decline(reason.into()))
}

const MAGIC: &[u8; 4] = b"CFVB";
const VERSION: u16 = 1;
const TOKEN_BYTES: usize = 20;

fn exact_float(value: &Bound<'_, PyAny>, what: &str) -> Read<f64> {
    match value.cast_exact::<PyFloat>() {
        Ok(number) => Ok(number.value()),
        Err(_) => decline(format!("{what} is not a float")),
    }
}

/// A number of the source normal (`float` or `int`, as Python multiplies it).
fn number(value: &Bound<'_, PyAny>, what: &str) -> Read<f64> {
    if let Ok(found) = value.cast_exact::<PyFloat>() {
        return Ok(found.value());
    }
    if value.cast_exact::<PyInt>().is_ok() {
        if let Ok(small) = value.extract::<i64>() {
            if small.unsigned_abs() <= (1u64 << 53) {
                return Ok(small as f64);
            }
        }
    }
    decline(format!("{what} is not a float or a small int"))
}

fn text(value: &Bound<'_, PyAny>, what: &str) -> Read<String> {
    match value.cast_exact::<PyString>() {
        Ok(found) => Ok(found.to_str()?.to_string()),
        Err(_) => decline(format!("{what} is not a str")),
    }
}

fn items<'py>(value: &Bound<'py, PyAny>, what: &str) -> Read<Vec<Bound<'py, PyAny>>> {
    if let Ok(tuple) = value.cast_exact::<PyTuple>() {
        return Ok(tuple.iter().collect());
    }
    if let Ok(list) = value.cast_exact::<PyList>() {
        return Ok(list.iter().collect());
    }
    if value.cast_exact::<PyFrozenSet>().is_ok() {
        // the order of the oracle's own iteration of the same object
        let mut found = Vec::new();
        for item in value.try_iter()? {
            found.push(item?);
        }
        return Ok(found);
    }
    decline(format!("{what} is not a tuple, a list or a frozenset"))
}

/// `value.value` of an id record, as a `str`.
fn id_text(py: Python<'_>, record: &Bound<'_, PyAny>, what: &str) -> Read<String> {
    text(&record.getattr(pyo3::intern!(py, "value"))?, what)
}

/// The strings of the encoder in the order of their first number (`number_of`).
#[derive(Default)]
struct Strings {
    order: Vec<String>,
    index: HashMap<String, u32, FxBuild>,
}

impl Strings {
    fn number_of(&mut self, value: &str) -> Read<u32> {
        if let Some(found) = self.index.get(value) {
            return Ok(*found);
        }
        let Ok(next) = u32::try_from(self.order.len()) else {
            return decline("too many strings");
        };
        self.index.insert(value.to_string(), next);
        self.order.push(value.to_string());
        Ok(next)
    }
}

fn u32_of(value: usize, what: &str) -> Read<u32> {
    match u32::try_from(value) {
        Ok(found) => Ok(found),
        Err(_) => decline(format!("{what} is past uint32")),
    }
}

/// `repr` of a reference (`str`), when it is the plain single-quoted ASCII form.
fn put_text_repr(out: &mut String, value: &str) -> Read<()> {
    if !value.bytes().all(|byte| (0x20..0x7f).contains(&byte) && byte != b'\'' && byte != b'\\') {
        return decline("a vertex reference whose repr is not plain ASCII");
    }
    out.push('\'');
    out.push_str(value);
    out.push('\'');
    Ok(())
}

fn put_int_tuple(out: &mut String, values: &[u32]) {
    out.push('(');
    for (at, value) in values.iter().enumerate() {
        if at > 0 {
            out.push_str(", ");
        }
        let _ = write!(out, "{value}");
    }
    if values.len() == 1 {
        out.push(',');
    }
    out.push(')');
}

fn put_rows(out: &mut String, rows: &[Vec<u32>]) {
    out.push('(');
    for (at, row) in rows.iter().enumerate() {
        if at > 0 {
            out.push_str(", ");
        }
        put_int_tuple(out, row);
    }
    if rows.len() == 1 {
        out.push(',');
    }
    out.push(')');
}

/// `_claim_ordinals`: `claim:N` -> N for every claim when every suffix is plain digits; any other suffix is declined (Python's `int` reads more forms).
fn claim_ordinals(names: &BTreeSet<String>) -> Read<HashMap<String, i64, FxBuild>> {
    let mut found = HashMap::default();
    for name in names {
        let Some((_, suffix)) = name.split_once(':') else {
            return decline("an ownership claim without ':'");
        };
        if suffix.is_empty() || suffix.len() > 18 || !suffix.bytes().all(|byte| byte.is_ascii_digit()) {
            return decline("an ownership claim whose ordinal is not plain digits");
        }
        found.insert(name.clone(), suffix.parse::<i64>().map_err(|_| Fail::Decline("an ownership ordinal".to_string()))?);
    }
    Ok(found)
}

struct Vertex {
    key: String,
    position: [f64; 3],
    reference: Option<String>,
}

#[allow(clippy::too_many_arguments)]
fn build<'py>(
    py: Python<'py>,
    batch: &Bound<'py, PyAny>,
    normal: &Bound<'py, PyAny>,
    vertex_normals: &Bound<'py, PyAny>,
    source_normal: &Bound<'py, PyAny>,
    patch_id: &Bound<'py, PyAny>,
    blake2b: &Bound<'py, PyAny>,
) -> Read<Bound<'py, PyBytes>> {
    let revision = id_text(py, &batch.getattr(pyo3::intern!(py, "source_revision"))?, "the source revision")?;
    let face_objects = items(&batch.getattr(pyo3::intern!(py, "faces"))?, "the faces")?;
    let vertex_objects = items(&batch.getattr(pyo3::intern!(py, "vertices"))?, "the vertices")?;
    if face_objects.is_empty() || vertex_objects.is_empty() {
        return decline("the batch has no faces or no vertices (ADAPTER_EMPTY_BATCH)");
    }
    if normal.is_none() {
        return decline("the plane normal is absent (ADAPTER_NORMAL_MISSING)");
    }
    let normal_items = items(normal, "the plane normal")?;
    if normal_items.len() != 3 {
        return decline("the plane normal is not three numbers");
    }
    let plane = [exact_float(&normal_items[0], "the plane normal")?, exact_float(&normal_items[1], "the plane normal")?, exact_float(&normal_items[2], "the plane normal")?];
    if !plane.iter().all(|value| value.is_finite()) {
        return decline("the plane normal is not finite (ADAPTER_NORMAL_MISSING)");
    }
    // `dict(getattr(result, "vertex_normals", ()) or ())`
    let mut shifts: HashMap<String, [f64; 3], FxBuild> = HashMap::default();
    let pairs: Vec<(Bound<'py, PyAny>, Bound<'py, PyAny>)> = if vertex_normals.is_none() {
        Vec::new()
    } else if let Ok(dict) = vertex_normals.cast_exact::<PyDict>() {
        dict.iter().collect()
    } else {
        let mut found = Vec::new();
        for pair in items(vertex_normals, "the vertex normals")? {
            let pair = items(&pair, "a vertex normal")?;
            if pair.len() != 2 {
                return decline("a vertex normal is not a (key, normal) pair");
            }
            found.push((pair[0].clone(), pair[1].clone()));
        }
        found
    };
    for (key, value) in &pairs {
        let row = items(value, "a vertex normal")?;
        if row.len() != 3 {
            return decline("a vertex normal is not three numbers");
        }
        shifts.insert(text(key, "a vertex normal key")?, [exact_float(&row[0], "a vertex normal")?, exact_float(&row[1], "a vertex normal")?, exact_float(&row[2], "a vertex normal")?]);
    }
    if !source_normal.is_none() && shifts.is_empty() {
        let source = items(source_normal, "the source normal")?;
        let mut truthy = false;
        for item in &source {
            truthy |= item.is_truthy()?;
        }
        if truthy {
            if source.len() != 3 {
                return decline("the source normal is not three numbers");
            }
            let (a, b, c) = (number(&source[0], "the source normal")?, number(&source[1], "the source normal")?, number(&source[2], "the source normal")?);
            let dot = plane[0] * a + plane[1] * b + plane[2] * c;
            if !(dot > 0.0) {
                return decline("the plane normal opposes the source (ADAPTER_NORMAL_OPPOSES_SOURCE)");
            }
        }
    }
    let Ok(patch) = patch_id.cast_exact::<PyInt>() else {
        return decline("the patch id is not an int");
    };
    let Ok(patch) = patch.extract::<i64>() else {
        return decline("the patch id is past int64");
    };
    // vertices in the order of their keys
    let mut vertices = Vec::with_capacity(vertex_objects.len());
    for vertex in &vertex_objects {
        let key = id_text(py, &vertex.getattr(pyo3::intern!(py, "vert_key"))?, "a vertex key")?;
        let point = vertex.getattr(pyo3::intern!(py, "position"))?;
        let position = [
            exact_float(&point.getattr(pyo3::intern!(py, "x"))?, "a vertex position")?,
            exact_float(&point.getattr(pyo3::intern!(py, "y"))?, "a vertex position")?,
            exact_float(&point.getattr(pyo3::intern!(py, "z"))?, "a vertex position")?,
        ];
        let reference = match vertex.getattr_opt(pyo3::intern!(py, "semantic_location_ref"))? {
            None => None,
            Some(location) if location.is_none() => None,
            Some(location) => Some(id_text(py, &location, "a vertex reference")?),
        };
        vertices.push(Vertex { key, position, reference });
    }
    vertices.sort_by(|a, b| a.key.cmp(&b.key));
    let mut index: HashMap<&str, u32, FxBuild> = HashMap::default();
    for (number, vertex) in vertices.iter().enumerate() {
        index.insert(vertex.key.as_str(), u32_of(number, "a vertex number")?);
    }
    let count = vertices.len();
    let mut positions = Vec::with_capacity(count * 3);
    let mut normals = Vec::with_capacity(count * 3);
    for vertex in &vertices {
        if !vertex.position.iter().all(|value| value.is_finite()) {
            return decline("a vertex position is not finite (ADAPTER_NON_FINITE_BATCH)");
        }
        let shift = if shifts.is_empty() {
            plane
        } else {
            match shifts.get(&vertex.key) {
                Some(found) => *found,
                None => return decline("the offset normal of a vertex is absent (ADAPTER_NORMAL_MISSING)"),
            }
        };
        positions.extend_from_slice(&vertex.position);
        normals.extend_from_slice(&shift);
    }
    // faces: loops, UV, owners
    let mut claims = Vec::with_capacity(face_objects.len());
    let mut claim_names = BTreeSet::new();
    let mut faces: Vec<Vec<u32>> = Vec::with_capacity(face_objects.len());
    let mut uvs: Vec<f64> = Vec::new();
    for face in &face_objects {
        let mut ring = Vec::new();
        for key in items(&face.getattr(pyo3::intern!(py, "ordered_vert_keys"))?, "a face loop")? {
            let key = id_text(py, &key, "a face vertex key")?;
            match index.get(key.as_str()) {
                Some(found) => ring.push(*found),
                None => return decline("a face vertex is missing (ADAPTER_FACE_VERTEX_MISSING)"),
            }
        }
        for fact in items(&face.getattr(pyo3::intern!(py, "uv_facts"))?, "the UV facts of a face")? {
            let uv = fact.getattr(pyo3::intern!(py, "uv"))?;
            let (u, v) = (exact_float(&uv.getattr(pyo3::intern!(py, "u"))?, "a UV")?, exact_float(&uv.getattr(pyo3::intern!(py, "v"))?, "a UV")?);
            if !(u.is_finite() && v.is_finite()) {
                return decline("a UV is not finite (ADAPTER_NON_FINITE_BATCH)");
            }
            uvs.push(u);
            uvs.push(v);
        }
        let claim = id_text(py, &face.getattr(pyo3::intern!(py, "ownership_claim_id"))?, "an ownership claim")?;
        claim_names.insert(claim.clone());
        claims.push(claim);
        faces.push(ring);
    }
    let ordinals = claim_ordinals(&claim_names)?;
    let owners: Vec<i64> = claims.iter().map(|claim| ordinals[claim]).collect();
    // seams: every consecutive pair of an interface chain whose two keys are vertices of the view
    let mut seams: BTreeSet<(u32, u32)> = BTreeSet::new();
    let interface = batch.getattr(pyo3::intern!(py, "interface_chains"))?;
    for chain in interface.try_iter()? {
        let mut keys = Vec::new();
        for key in items(&chain?.getattr(pyo3::intern!(py, "ordered_vert_keys"))?, "an interface chain")? {
            keys.push(id_text(py, &key, "an interface chain key")?);
        }
        for pair in keys.windows(2) {
            if let (Some(first), Some(second)) = (index.get(pair[0].as_str()), index.get(pair[1].as_str())) {
                if pair[0] != pair[1] {
                    seams.insert(if first <= second { (*first, *second) } else { (*second, *first) });
                }
            }
        }
    }
    // `boundary_chain_names(batch)`: by chain name
    let mut chains: Vec<(String, Vec<String>)> = Vec::new();
    if let Some(boundary) = batch.getattr_opt(pyo3::intern!(py, "boundary_chains"))? {
        if boundary.is_truthy()? {
            for chain in boundary.try_iter()? {
                let chain = chain?;
                let name = id_text(py, &chain.getattr(pyo3::intern!(py, "semantic_boundary_id"))?, "a boundary chain name")?;
                let mut keys = Vec::new();
                for key in items(&chain.getattr(pyo3::intern!(py, "ordered_vert_keys"))?, "a boundary chain")? {
                    keys.push(id_text(py, &key, "a boundary chain key")?);
                }
                chains.push((name, keys));
            }
        }
    }
    chains.sort_by(|a, b| a.0.cmp(&b.0));
    let mut walls: Vec<Vec<u32>> = Vec::new();
    for (name, keys) in &chains {
        let mut parts = name.split(':');
        parts.next();
        let Some(kind) = parts.next() else {
            return decline("a boundary chain name without ':'");
        };
        if kind == "WALL" {
            walls.push(keys.iter().filter_map(|key| index.get(key.as_str()).copied()).collect());
        }
    }
    // the structure token: blake2b(repr((faces, seams, refs, len(uvs))), digest_size=10)
    let mut shown = String::with_capacity(faces.len() * 24 + count * 24);
    shown.push('(');
    put_rows(&mut shown, &faces);
    shown.push_str(", ");
    let seam_rows: Vec<Vec<u32>> = seams.iter().map(|(a, b)| vec![*a, *b]).collect();
    put_rows(&mut shown, &seam_rows);
    shown.push_str(", (");
    for (at, vertex) in vertices.iter().enumerate() {
        if at > 0 {
            shown.push_str(", ");
        }
        match &vertex.reference {
            None => shown.push_str("None"),
            Some(reference) => put_text_repr(&mut shown, reference)?,
        }
    }
    if count == 1 {
        shown.push(',');
    }
    let uv_pairs = uvs.len() / 2;
    let _ = write!(shown, "), {uv_pairs})");
    let keywords = PyDict::new(py);
    keywords.set_item(pyo3::intern!(py, "digest_size"), 10)?;
    let token = text(&blake2b.call((PyBytes::new(py, shown.as_bytes()),), Some(&keywords))?.call_method0(pyo3::intern!(py, "hexdigest"))?, "the token")?;
    if token.len() != TOKEN_BYTES || !token.is_ascii() {
        return decline("the structure token is not 20 characters");
    }
    // the encoder: strings numbered in the order of `_encode`
    let mut strings = Strings::default();
    let revision_number = strings.number_of(&revision)?;
    let mut refs = Vec::with_capacity(count);
    for vertex in &vertices {
        refs.push(match &vertex.reference {
            None => 0,
            Some(reference) => strings.number_of(reference)?.checked_add(1).ok_or_else(|| Fail::Decline("a reference number".to_string()))?,
        });
    }
    let mut wall_starts = vec![0u32];
    let mut wall_items: Vec<u32> = Vec::new();
    for wall in &walls {
        wall_items.extend_from_slice(wall);
        wall_starts.push(u32_of(wall_items.len(), "the wall items")?);
    }
    let mut chain_names = Vec::with_capacity(chains.len());
    for (name, _keys) in &chains {
        chain_names.push(strings.number_of(name)?);
    }
    let mut chain_starts = vec![0u32];
    let mut chain_items: Vec<u32> = Vec::new();
    for (_name, keys) in &chains {
        for key in keys {
            chain_items.push(strings.number_of(key)?);
        }
        chain_starts.push(u32_of(chain_items.len(), "the chain items")?);
    }
    let mut face_starts = vec![0u32];
    let mut corners: Vec<u32> = Vec::new();
    for face in &faces {
        corners.extend_from_slice(face);
        face_starts.push(u32_of(corners.len(), "the face corners")?);
    }
    let seam_flat: Vec<u32> = seams.iter().flat_map(|(a, b)| [*a, *b]).collect();
    let mut offsets = vec![0u32];
    let mut blob: Vec<u8> = Vec::new();
    for text in &strings.order {
        blob.extend_from_slice(text.as_bytes());
        offsets.push(u32_of(blob.len(), "the string bytes")?);
    }
    let counts = [
        u32_of(count, "the vertices")?,
        u32_of(faces.len(), "the faces")?,
        u32_of(corners.len(), "the corners")?,
        u32_of(uv_pairs, "the UV")?,
        u32_of(seams.len(), "the seams")?,
        u32_of(walls.len(), "the walls")?,
        u32_of(wall_items.len(), "the wall items")?,
        u32_of(chains.len(), "the chains")?,
        u32_of(chain_items.len(), "the chain items")?,
        u32_of(strings.order.len(), "the strings")?,
        u32_of(blob.len(), "the string bytes")?,
        revision_number,
        0,
        0,
    ];
    let mut out: Vec<u8> = Vec::with_capacity(80 + positions.len() * 16 + uvs.len() * 8 + corners.len() * 4 + blob.len());
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&patch.to_le_bytes());
    for value in counts {
        out.extend_from_slice(&value.to_le_bytes());
    }
    out.extend_from_slice(token.as_bytes());
    for value in positions.iter().chain(&normals).chain(&uvs) {
        out.extend_from_slice(&value.to_le_bytes());
    }
    for value in &owners {
        out.extend_from_slice(&value.to_le_bytes());
    }
    for part in [&face_starts, &corners, &seam_flat, &refs, &wall_starts, &wall_items, &chain_names, &chain_starts, &chain_items, &offsets] {
        for value in part.iter() {
            out.extend_from_slice(&value.to_le_bytes());
        }
    }
    out.extend_from_slice(&blob);
    Ok(PyBytes::new(py, &out))
}

/// The bytes of the view of a materialized domain, or the text of a decline (a `str`).
#[pyfunction]
pub fn domain_view_bytes<'py>(
    py: Python<'py>,
    batch: &Bound<'py, PyAny>,
    normal: &Bound<'py, PyAny>,
    vertex_normals: &Bound<'py, PyAny>,
    source_normal: &Bound<'py, PyAny>,
    patch_id: &Bound<'py, PyAny>,
    blake2b: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    match build(py, batch, normal, vertex_normals, source_normal, patch_id, blake2b) {
        Ok(found) => Ok(found.into_any()),
        Err(Fail::Decline(reason)) => Ok(PyString::new(py, &reason).into_any()),
        Err(Fail::Py(error)) => Ok(PyString::new(py, &format!("a Python error while reading: {error}")).into_any()),
    }
}
