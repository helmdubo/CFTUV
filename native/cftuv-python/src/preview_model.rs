//! PREVIEW_MODEL_NATIVE_V1: `envelope_width_preview_model.build_model` over the domain rows of a sample (`DomainSampleV1`), in one call.
//!
//! The oracle is the host's Python (`build_model` with `_domain_reason`, `_partner_state`, `_weld_conflicts`, `_entries_of`, `_scatter_last`, `_trusted_reach`,
//! `_display_reach`, `_normal_sources`, `_vertex_windows`): the same binary64 operations in the same order, the Python `max` / `min` of two floats (the first
//! unless the second is strictly greater / smaller) where the oracle calls them, the numpy `maximum` / `minimum` (NaN propagates) where it reduces arrays.
//! The answer is the model's arrays as `bytearray` (writable, the dtypes of the oracle), the rows tuple and the layout entries (`_EntriesV1`).
//! What the port would not repeat exactly it declines by a text and the Python path answers: a row whose slopes are not one per vertex or loop (numpy would
//! broadcast or raise), a mesh vertex outside the sample, loops that do not tile the sample, a wall partner outside the rows, a buffer of another type.

use pyo3::buffer::PyBuffer;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyByteArray, PyDict, PyFloat, PyInt, PyTuple};

use crate::mesh_assembly::{declined, Decline};

/// The Python `max(a, b)` of two floats: `a` unless `b > a`.
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// The Python `min(a, b)` of two floats: `a` unless `b < a`.
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `np.maximum(a, b)`: NaN propagates, else `a` when `a >= b`.
fn np_max(a: f64, b: f64) -> f64 {
    if a >= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// `np.minimum(a, b)`: NaN propagates, else `a` when `a <= b`.
fn np_min(a: f64, b: f64) -> f64 {
    if a <= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// The numbers of the law the Python module defines (passed in, so the module stays their one source).
struct Law {
    readmit_reach: f64,
    reach_ratio: f64,
    error_limit: f64,
    error_safety: f64,
    weld_tolerance: f64,
}

fn display_reach(law: &Law, offset: f64, rate: f64, alpha: f64) -> f64 {
    let per_width = offset * rate * law.error_safety;
    if per_width != per_width {
        return 0.0;
    }
    if per_width <= 0.0 {
        return law.reach_ratio;
    }
    py_min(law.reach_ratio, law.error_limit / (per_width * alpha))
}

#[derive(Clone, Copy, PartialEq)]
enum Reason {
    None,
    NotMaterialized,
    NoInterval,
    Marker,
    WeldPartner,
    WeldDisagrees,
}

struct Row<'py> {
    patch: Bound<'py, PyAny>,
    domain: Bound<'py, PyAny>,
    marker: Option<Bound<'py, PyAny>>,
    reason: Reason,
    no_certificate: bool,
    approximate: bool,
    rate: f64,
    partners: Vec<usize>,
    low: f64,
    high: f64,
    normal_alpha: f64,
    slopes: Option<Vec<f64>>,
    uv_slopes: Option<Vec<f64>>,
    loop_start: i64,
    loop_count: i64,
    vertices: Vec<i64>,
}

fn f64_buffer(item: &Bound<'_, PyAny>, what: &str) -> Result<Vec<f64>, Decline> {
    let buffer = PyBuffer::<f64>::get(item).map_err(|_| format!("{what} is not a float64 buffer"))?;
    buffer.to_vec(item.py()).map_err(|_| format!("{what} is not a contiguous float64 buffer"))
}

fn index_buffer(item: &Bound<'_, PyAny>, what: &str) -> Result<Vec<i64>, Decline> {
    if let Ok(buffer) = PyBuffer::<i32>::get(item) {
        return Ok(buffer.to_vec(item.py()).map_err(|_| format!("{what} is not a contiguous int32 buffer"))?.into_iter().map(i64::from).collect());
    }
    let buffer = PyBuffer::<i64>::get(item).map_err(|_| format!("{what} is not an int32 or int64 buffer"))?;
    buffer.to_vec(item.py()).map_err(|_| format!("{what} is not a contiguous int64 buffer"))
}

struct Failure(Result<Decline, PyErr>);

impl From<PyErr> for Failure {
    /// A row the port cannot read (a field of another type, a lookup that raised) is a decline: the Python path answers it, with whatever it raises.
    fn from(error: PyErr) -> Failure {
        Failure(Ok(format!("a row is not read: {error}")))
    }
}

impl From<Decline> for Failure {
    fn from(reason: Decline) -> Failure {
        Failure(Ok(reason))
    }
}

fn read_row<'py>(py: Python<'py>, item: &Bound<'py, PyAny>, certified: &str, no_certificate: &str) -> Result<Row<'py>, Failure> {
    let get = |name: &Bound<'py, pyo3::types::PyString>| item.getattr(name);
    let token = get(pyo3::intern!(py, "token"))?;
    let status: String = get(pyo3::intern!(py, "status"))?.extract()?;
    let interval = get(pyo3::intern!(py, "interval"))?;
    let markers = get(pyo3::intern!(py, "markers"))?;
    let marker_items: Vec<Bound<'py, PyAny>> = markers.try_iter()?.collect::<PyResult<_>>()?;
    let reason = if !token.is_truthy()? {
        Reason::NotMaterialized
    } else if status != certified || interval.is_none() {
        Reason::NoInterval
    } else if !marker_items.is_empty() {
        Reason::Marker
    } else {
        Reason::None
    };
    let mut no_cert = false;
    for marker in &marker_items {
        if let Ok(text) = marker.extract::<String>() {
            no_cert |= text == no_certificate;
        }
    }
    let partners_object = get(pyo3::intern!(py, "partners"))?;
    let mut partners = Vec::new();
    for other in partners_object.try_iter()? {
        let other: i64 = other?.extract()?;
        if other < 0 {
            return Err(Decline::from("a wall partner is negative").into());
        }
        partners.push(other as usize);
    }
    let high = get(pyo3::intern!(py, "high"))?;
    let slopes = get(pyo3::intern!(py, "slopes"))?;
    let uv_slopes = get(pyo3::intern!(py, "uv_slopes"))?;
    let rate: f64 = get(pyo3::intern!(py, "approximate_rate"))?.extract()?;
    Ok(Row {
        patch: get(pyo3::intern!(py, "patch_id"))?,
        domain: get(pyo3::intern!(py, "domain_id"))?,
        marker: marker_items.into_iter().next(),
        reason,
        no_certificate: no_cert,
        approximate: get(pyo3::intern!(py, "approximate"))?.is_truthy()?,
        rate,
        partners,
        low: get(pyo3::intern!(py, "low"))?.extract()?,
        high: if high.is_none() { f64::INFINITY } else { high.extract()? },
        normal_alpha: get(pyo3::intern!(py, "normal_alpha"))?.extract()?,
        slopes: if slopes.is_none() { None } else { Some(f64_buffer(&slopes, "the position slopes")?) },
        uv_slopes: if uv_slopes.is_none() { None } else { Some(f64_buffer(&uv_slopes, "the loop slopes")?) },
        loop_start: get(pyo3::intern!(py, "loop_start"))?.extract()?,
        loop_count: get(pyo3::intern!(py, "loop_count"))?.extract()?,
        vertices: index_buffer(&get(pyo3::intern!(py, "vertices"))?, "the mesh vertices of a domain")?,
    })
}

struct Trust {
    quarantined: bool,
    abandoned: bool,
    scale: f64,
    reach_cap: f64,
}

/// The entries of the layout (`_EntriesV1`): records `(domain, mesh vertex)` domain by domain, and their stable order by vertex.
struct Entries {
    domain: Vec<i64>,
    vertex: Vec<i64>,
    bounds: Vec<i64>,
    total: Vec<i64>,
    order: Vec<i64>,
    starts: Vec<i64>,
    groups: Vec<i64>,
}

fn entries_of(rows: &[Row<'_>], vertex_count: usize) -> Result<Entries, Decline> {
    let mut domain = Vec::new();
    let mut vertex = Vec::new();
    let mut bounds = vec![0i64];
    for (number, row) in rows.iter().enumerate() {
        for &item in &row.vertices {
            if item < 0 || item as usize >= vertex_count {
                return Err("a mesh vertex of a domain is outside the sample".to_string());
            }
            domain.push(number as i64);
            vertex.push(item);
        }
        bounds.push(vertex.len() as i64);
    }
    let mut total = vec![0i64; vertex_count];
    for &item in &vertex {
        total[item as usize] += 1;
    }
    // the stable argsort by vertex: a counting sort keeps the order of the records of one vertex
    let mut first = vec![0i64; vertex_count + 1];
    for v in 0..vertex_count {
        first[v + 1] = first[v] + total[v];
    }
    let mut cursor = first.clone();
    let mut order = vec![0i64; vertex.len()];
    for (at, &item) in vertex.iter().enumerate() {
        let slot = &mut cursor[item as usize];
        order[*slot as usize] = at as i64;
        *slot += 1;
    }
    let mut starts = Vec::new();
    let mut groups = Vec::new();
    for v in 0..vertex_count {
        if total[v] > 0 {
            starts.push(first[v]);
            groups.push(v as i64);
        }
    }
    Ok(Entries { domain, vertex, bounds, total, order, starts, groups })
}

/// `_weld_conflicts`: the domains (of `live`) with a welded vertex whose slopes from the live domains differ above the tolerance.
fn weld_conflicts(rows: &[Row<'_>], entries: &Entries, live: &[bool], tolerance: f64) -> Vec<bool> {
    let count = rows.len();
    let mut found = vec![false; count];
    let valid: Vec<bool> = entries.domain.iter().map(|&d| live[d as usize]).collect();
    if !valid.iter().any(|&v| v) {
        return found;
    }
    let slope = |at: usize| -> [f64; 3] {
        let d = entries.domain[at] as usize;
        let local = at - entries.bounds[d] as usize;
        let s = rows[d].slopes.as_ref().expect("a live domain has slopes");
        [s[3 * local], s[3 * local + 1], s[3 * local + 2]]
    };
    let mut broken_vertex = vec![false; entries.total.len()];
    let size = entries.order.len();
    for (group, &start) in entries.starts.iter().enumerate() {
        let end = if group + 1 < entries.starts.len() { entries.starts[group + 1] as usize } else { size };
        let mut low = [0.0f64; 3];
        let mut high = [0.0f64; 3];
        for (k, position) in (start as usize..end).enumerate() {
            let at = entries.order[position] as usize;
            let (lo, hi) = if valid[at] { let s = slope(at); (s, s) } else { ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]) };
            for axis in 0..3 {
                if k == 0 {
                    low[axis] = lo[axis];
                    high[axis] = hi[axis];
                } else {
                    low[axis] = np_min(low[axis], lo[axis]);
                    high[axis] = np_max(high[axis], hi[axis]);
                }
            }
        }
        let mut widest = high[0] - low[0];
        for axis in 1..3 {
            widest = np_max(widest, high[axis] - low[axis]);
        }
        if widest > tolerance {
            broken_vertex[entries.groups[group] as usize] = true;
        }
    }
    for (at, &v) in entries.vertex.iter().enumerate() {
        if valid[at] && broken_vertex[v as usize] {
            found[entries.domain[at] as usize] = true;
        }
    }
    found
}

fn bytearray_f64<'py>(py: Python<'py>, values: &[f64]) -> PyResult<Bound<'py, PyByteArray>> {
    PyByteArray::new_with(py, values.len() * 8, |buffer| {
        for (chunk, value) in buffer.chunks_exact_mut(8).zip(values) {
            chunk.copy_from_slice(&value.to_le_bytes());
        }
        Ok(())
    })
}

fn bytearray_i64<'py>(py: Python<'py>, values: &[i64]) -> PyResult<Bound<'py, PyByteArray>> {
    PyByteArray::new_with(py, values.len() * 8, |buffer| {
        for (chunk, value) in buffer.chunks_exact_mut(8).zip(values) {
            chunk.copy_from_slice(&value.to_le_bytes());
        }
        Ok(())
    })
}

fn bytearray_bool<'py>(py: Python<'py>, values: &[bool]) -> PyResult<Bound<'py, PyByteArray>> {
    PyByteArray::new_with(py, values.len(), |buffer| {
        for (byte, &value) in buffer.iter_mut().zip(values) {
            *byte = value as u8;
        }
        Ok(())
    })
}

/// `build_model` (the arrays, the rows and the layout entries) or the text of a decline (a `str`).
///
/// `names`: `(MODELLED, NOT_MATERIALIZED, NO_INTERVAL, WELD_PARTNER, WELD_DISAGREES, DISPLAY_APPROXIMATE, QUARANTINED, ABANDONED, PARAM_NO_CERTIFICATE,
/// CERTIFIED, TRUST_QUARANTINED, TRUST_ABANDONED)`; `law`: `(READMIT_NOMINAL_REACH_RATIO, DISPLAY_APPROXIMATE_REACH_RATIO, DISPLAY_APPROXIMATE_ERROR_LIMIT,
/// DISPLAY_APPROXIMATE_ERROR_SAFETY, WELD_SLOPE_TOLERANCE)`; `trust`: the entries of the ledger of the sample's key (`{(patch, domain): DomainTrustV1}`) or `None`.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn preview_model_build<'py>(
    py: Python<'py>,
    domains: &Bound<'py, PyAny>,
    vertex_count: usize,
    uvs: &Bound<'py, PyAny>,
    alpha0: f64,
    offset: f64,
    trust: &Bound<'py, PyAny>,
    names: &Bound<'py, PyTuple>,
    law: (f64, f64, f64, f64, f64),
) -> PyResult<Bound<'py, PyAny>> {
    match build(py, domains, vertex_count, uvs, alpha0, offset, trust, names, law) {
        Ok(answer) => Ok(answer),
        Err(Failure(Ok(reason))) => declined(py, reason),
        Err(Failure(Err(error))) => Err(error),
    }
}

#[allow(clippy::too_many_arguments)]
fn build<'py>(
    py: Python<'py>,
    domains: &Bound<'py, PyAny>,
    vertex_count: usize,
    uvs: &Bound<'py, PyAny>,
    alpha0: f64,
    offset: f64,
    trust: &Bound<'py, PyAny>,
    names: &Bound<'py, PyTuple>,
    law: (f64, f64, f64, f64, f64),
) -> Result<Bound<'py, PyAny>, Failure> {
    let law = Law { readmit_reach: law.0, reach_ratio: law.1, error_limit: law.2, error_safety: law.3, weld_tolerance: law.4 };
    let name = |at: usize| names.get_item(at);
    let certified: String = name(9)?.extract()?;
    let no_certificate: String = name(8)?.extract()?;
    let trust_quarantined: String = name(10)?.extract()?;
    let trust_abandoned: String = name(11)?.extract()?;
    let mut rows = Vec::new();
    for item in domains.try_iter()? {
        rows.push(read_row(py, &item?, &certified, &no_certificate)?);
    }
    let count = rows.len();
    let base_uvs = f64_buffer(uvs, "the sample uvs")?;
    let loop_total = base_uvs.len() / 2;
    for row in &rows {
        if row.partners.iter().any(|&other| other >= count) {
            return Err(Decline::from("a wall partner is outside the rows").into());
        }
    }
    // the first pass: the reason of each domain, its partners' state
    let mut approximate = vec![false; count];
    let mut normal_rate = vec![0.0f64; count];
    let mut reasons: Vec<Reason> = rows.iter().map(|row| row.reason).collect();
    for number in 0..count {
        if reasons[number] != Reason::None {
            continue;
        }
        let row = &rows[number];
        let (mut blocked, mut near, mut rate) = (false, false, 0.0f64);
        for &other in &row.partners {
            let neighbour = &rows[other];
            if neighbour.no_certificate {
                blocked = true;
            } else if neighbour.approximate {
                near = true;
                rate = py_max(rate, neighbour.rate);
            }
        }
        if blocked {
            reasons[number] = Reason::WeldPartner;
        }
        approximate[number] = row.approximate || near;
        normal_rate[number] = py_max(if row.approximate { row.rate } else { 0.0 }, rate);
    }
    let live: Vec<bool> = reasons.iter().map(|&reason| reason == Reason::None).collect();
    for (number, row) in rows.iter().enumerate() {
        if live[number] {
            match &row.slopes {
                Some(slopes) if slopes.len() == 3 * row.vertices.len() => {}
                _ => return Err(Decline::from("the position slopes of a domain are not one per vertex").into()),
            }
        }
    }
    let entries = entries_of(&rows, vertex_count)?;
    if live.iter().any(|&v| v) {
        let conflicts = weld_conflicts(&rows, &entries, &live, law.weld_tolerance);
        for number in 0..count {
            if conflicts[number] {
                reasons[number] = Reason::WeldDisagrees;
            }
        }
    }
    // the second pass: the windows and the rows
    let trust_entries = if trust.is_none() { None } else { Some(trust.cast::<PyDict>().map_err(|_| Decline::from("the trust entries are not a dict"))?.clone()) };
    let trust_of = |row: &Row<'py>| -> Result<Option<Trust>, Failure> {
        let Some(entries) = &trust_entries else { return Ok(None) };
        if entries.is_empty() {
            return Ok(None);
        }
        let key = PyTuple::new(py, [row.patch.clone(), row.domain.clone()])?;
        let Some(entry) = entries.get_item(key)? else { return Ok(None) };
        let state: String = entry.getattr(pyo3::intern!(py, "state"))?.extract()?;
        Ok(Some(Trust {
            quarantined: state == trust_quarantined,
            abandoned: state == trust_abandoned,
            scale: entry.getattr(pyo3::intern!(py, "scale"))?.extract()?,
            reach_cap: entry.getattr(pyo3::intern!(py, "reach_cap"))?.extract()?,
        }))
    };
    let mut dlow = vec![f64::INFINITY; count];
    let mut dhigh = vec![f64::NEG_INFINITY; count];
    let mut quarantined = vec![false; count];
    let mut shadow_low = vec![f64::INFINITY; count];
    let mut shadow_high = vec![f64::NEG_INFINITY; count];
    let mut out_rows = Vec::with_capacity(count);
    let mut modelled = vec![false; count];
    let none = py.None().into_bound(py);
    for number in 0..count {
        let row = &rows[number];
        let reason = reasons[number];
        if reason != Reason::None {
            let text = match reason {
                Reason::NotMaterialized => name(1)?,
                Reason::NoInterval => name(2)?,
                Reason::Marker => row.marker.clone().expect("a marker reason has a marker"),
                Reason::WeldPartner => name(3)?,
                Reason::WeldDisagrees => name(4)?,
                Reason::None => unreachable!(),
            };
            out_rows.push(PyTuple::new(py, [row.patch.clone(), row.domain.clone(), text, PyInt::new(py, 0i64).into_any(), none.clone()])?.into_any());
            continue;
        }
        let entry = trust_of(row)?;
        let near = approximate[number];
        let base_reach = if near { Some(display_reach(&law, offset, normal_rate[number], alpha0)) } else { None };
        let reach = match &entry {
            None => base_reach,
            Some(entry) => {
                let nominal = base_reach.unwrap_or(law.readmit_reach);
                Some(py_min(nominal * entry.scale, entry.reach_cap))
            }
        };
        let mut outcome = if near { name(5)? } else { name(0)? };
        let mut low = row.low;
        let mut high = row.high;
        if let Some(entry) = &entry {
            if entry.quarantined || entry.abandoned {
                outcome = if entry.quarantined { name(6)? } else { name(7)? };
                quarantined[number] = true;
                shadow_low[number] = py_max(low, alpha0 * (1.0 - law.readmit_reach));
                shadow_high[number] = py_min(high, alpha0 * (1.0 + law.readmit_reach));
            }
        }
        if let Some(reach) = reach {
            low = py_max(low, alpha0 * (1.0 - reach));
            high = py_min(high, alpha0 * (1.0 + reach));
        }
        if near {
            let sources = std::iter::once(row).chain(row.partners.iter().map(|&other| &rows[other]));
            for item in sources {
                if !item.approximate {
                    continue;
                }
                let frozen = if item.normal_alpha != item.normal_alpha { alpha0 } else { item.normal_alpha };
                if frozen != alpha0 {
                    let span = display_reach(&law, offset, item.rate, frozen);
                    low = py_max(low, frozen * (1.0 - span));
                    high = py_min(high, frozen * (1.0 + span));
                }
            }
        }
        dlow[number] = low;
        dhigh[number] = high;
        let reach_object = match reach {
            None => none.clone(),
            Some(value) => PyFloat::new(py, value).into_any(),
        };
        out_rows.push(PyTuple::new(py, [row.patch.clone(), row.domain.clone(), outcome, PyInt::new(py, 1i64).into_any(), reach_object])?.into_any());
        modelled[number] = true;
    }
    let rows_tuple = PyTuple::new(py, out_rows)?;
    let entries_tuple = PyTuple::new(
        py,
        [
            bytearray_i64(py, &entries.domain)?,
            bytearray_i64(py, &entries.vertex)?,
            bytearray_i64(py, &entries.bounds)?,
            bytearray_i64(py, &entries.total)?,
            bytearray_i64(py, &entries.order)?,
            bytearray_i64(py, &entries.starts)?,
            bytearray_i64(py, &entries.groups)?,
        ],
    )?;
    if !modelled.iter().any(|&m| m) {
        return Ok(PyTuple::new(py, [rows_tuple.into_any(), entries_tuple.into_any()])?.into_any());
    }
    // the arrays
    let loop_counts: Vec<i64> = rows.iter().map(|row| row.loop_count).collect();
    if loop_counts.iter().any(|&c| c < 0) || loop_counts.iter().sum::<i64>() as usize != loop_total || base_uvs.len() != 2 * loop_total {
        return Err(Decline::from("the loops of the domains do not tile the sample").into());
    }
    let mut uv_slopes = vec![0.0f64; 2 * loop_total];
    for (number, row) in rows.iter().enumerate() {
        if !modelled[number] {
            continue;
        }
        let (start, size) = (row.loop_start, row.loop_count);
        let values = match &row.uv_slopes {
            Some(values) if values.len() == 2 * size as usize => values,
            _ => return Err(Decline::from("the loop slopes of a domain are not one per loop").into()),
        };
        if start < 0 || (start + size) as usize > loop_total {
            return Err(Decline::from("the loops of a domain are outside the sample").into());
        }
        uv_slopes[2 * start as usize..2 * (start + size) as usize].copy_from_slice(values);
    }
    let slopes_of: Vec<&[f64]> = (0..count).map(|number| if modelled[number] { rows_slopes(&rows, number) } else { &[][..] }).collect();
    let (entries, modelled, quarantined, dlow_ref, dhigh_ref) = (&entries, &modelled, &quarantined, &dlow, &dhigh);
    let (loop_counts_ref, uv_slopes_ref, base_uvs_ref, slopes_ref) = (&loop_counts, &uv_slopes, &base_uvs, &slopes_of);
    let answer = py.detach(move || {
        let (dlow, dhigh, loop_counts, uv_slopes, base_uvs) = (dlow_ref, dhigh_ref, loop_counts_ref, uv_slopes_ref, base_uvs_ref);
        let mut a1 = vec![0.0f64; 3 * vertex_count];
        for (at, &v) in entries.vertex.iter().enumerate() {
            let d = entries.domain[at] as usize;
            if modelled[d] {
                let local = at - entries.bounds[d] as usize;
                let s = slopes_ref[d];
                a1[3 * v as usize..3 * v as usize + 3].copy_from_slice(&s[3 * local..3 * local + 3]);
            }
        }
        let mut e1 = vec![0.0f64; 2 * loop_total];
        let mut at = 0usize;
        for (number, &size) in loop_counts.iter().enumerate() {
            for _ in 0..size {
                if modelled[number] {
                    e1[2 * at] = uv_slopes[2 * at] - base_uvs[2 * at];
                    e1[2 * at + 1] = uv_slopes[2 * at + 1] - base_uvs[2 * at + 1];
                }
                at += 1;
            }
        }
        // `_vertex_windows`
        let mut alive = vec![0i64; vertex_count];
        let mut vertex_quarantined = vec![false; vertex_count];
        for (at, &v) in entries.vertex.iter().enumerate() {
            let d = entries.domain[at] as usize;
            if modelled[d] {
                alive[v as usize] += 1;
                if quarantined[d] {
                    vertex_quarantined[v as usize] = true;
                }
            }
        }
        let mut vlow = vec![f64::NEG_INFINITY; vertex_count];
        let mut vhigh = vec![f64::INFINITY; vertex_count];
        let size = entries.order.len();
        for (group, &start) in entries.starts.iter().enumerate() {
            let end = if group + 1 < entries.starts.len() { entries.starts[group + 1] as usize } else { size };
            let (mut low, mut high) = (0.0f64, 0.0f64);
            for (k, position) in (start as usize..end).enumerate() {
                let at = entries.order[position] as usize;
                let d = entries.domain[at] as usize;
                let (lo, hi) = if modelled[d] { (dlow[d], dhigh[d]) } else { (f64::NEG_INFINITY, f64::INFINITY) };
                if k == 0 {
                    low = lo;
                    high = hi;
                } else {
                    low = np_max(low, lo);
                    high = np_min(high, hi);
                }
            }
            let v = entries.groups[group] as usize;
            vlow[v] = low;
            vhigh[v] = high;
        }
        for v in 0..vertex_count {
            if alive[v] != entries.total[v] {
                vlow[v] = f64::INFINITY;
                vhigh[v] = f64::NEG_INFINITY;
                a1[3 * v] = 0.0;
                a1[3 * v + 1] = 0.0;
                a1[3 * v + 2] = 0.0;
            }
        }
        (a1, e1, vlow, vhigh, vertex_quarantined)
    });
    let (a1, e1, vlow, vhigh, vertex_quarantined) = answer;
    let any_approximate = (0..count).any(|number| modelled[number] && approximate[number]);
    let parts: Vec<Bound<'py, PyAny>> = vec![
        rows_tuple.into_any(),
        entries_tuple.into_any(),
        bytearray_f64(py, &a1)?.into_any(),
        bytearray_f64(py, &e1)?.into_any(),
        bytearray_f64(py, &vlow)?.into_any(),
        bytearray_f64(py, &vhigh)?.into_any(),
        bytearray_f64(py, &dlow)?.into_any(),
        bytearray_f64(py, &dhigh)?.into_any(),
        bytearray_bool(py, &quarantined)?.into_any(),
        bytearray_bool(py, &vertex_quarantined)?.into_any(),
        bytearray_f64(py, &shadow_low)?.into_any(),
        bytearray_f64(py, &shadow_high)?.into_any(),
        bytearray_bool(py, &approximate)?.into_any(),
        PyBool::new(py, quarantined.iter().any(|&q| q)).to_owned().into_any(),
        PyBool::new(py, any_approximate).to_owned().into_any(),
    ];
    Ok(PyTuple::new(py, parts)?.into_any())
}

fn rows_slopes<'a>(rows: &'a [Row<'_>], number: usize) -> &'a [f64] {
    rows[number].slopes.as_deref().expect("a modelled domain has slopes")
}
