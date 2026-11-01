//! The checks of the assembled decal mesh in the main process (MESH_ASSEMBLY_NATIVE_V1, slice B1): `kernel/src/cftuv_envelope/mesh_checks.py` is the oracle.
//!
//! * `mesh_off_plane(positions, faces)`: `off_plane_measure`, `(worst metres, faces off by a nanometre)`;
//! * `mesh_cross_seams(faces, uvs, face_domain)`: `cross_domain_seams`, the sorted edges between two faces of different domains with a UV jump;
//! * `mesh_half_edges(faces)`: `half_edge_conflicts`, the directed edges of two faces;
//! * `mesh_weld(domains, offset)`: `mesh_weld.weld_core` (slice B2), the welded positions with the miter offset and the numbers of the receipt;
//! * `mesh_reports(...)`: the reports of the assembled mesh in one read (slice B3): `arrays_digest`, `seam_report_numbers`, `off_plane_measure`, `half_edge_conflicts`.
//!
//! CPython semantics, bit for bit: binary64 subtraction and sums left to right (Rust never contracts to FMA), `max` keeps the first of equals, `round` to
//! even (`round_ties_even`), `!=` over UV numbers is numeric (`-0.0 == 0.0`). What the port does not carry it declines by a text (the shim raises
//! `NativePortUnsupported` and the oracle answers): a coordinate or UV that is not an exact `float` (an `int` subtracts exactly in Python, a subclass may
//! override the operators), a non-finite number (`max` over a NaN depends on order), an index outside the arrays or negative (Python wraps it or raises),
//! a vertex or domain id that is not an `int` in `i64`. Declining is always safe: the oracle answers the same input.

use std::collections::HashMap;

use pyo3::prelude::*;
use pyo3::types::{PyFloat, PyList, PyTuple};

pub(crate) type Decline = String;

pub(crate) fn exact_float(item: &Bound<'_, PyAny>, what: &str) -> Result<f64, Decline> {
    if !item.is_exact_instance_of::<PyFloat>() {
        return Err(format!("{what} is not an exact float"));
    }
    let value = item.cast::<PyFloat>().map_err(|_| format!("{what} is not a float"))?.value();
    if !value.is_finite() {
        return Err(format!("{what} is not finite"));
    }
    Ok(value)
}

pub(crate) fn items<'py>(sequence: &Bound<'py, PyAny>, what: &str) -> Result<Vec<Bound<'py, PyAny>>, Decline> {
    if let Ok(list) = sequence.cast::<PyList>() {
        return Ok(list.iter().collect());
    }
    if let Ok(tuple) = sequence.cast::<PyTuple>() {
        return Ok(tuple.iter().collect());
    }
    Err(format!("{what} is not a list or a tuple"))
}

pub(crate) fn floats(item: &Bound<'_, PyAny>, size: usize, what: &str) -> Result<Vec<f64>, Decline> {
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

pub(crate) type Point = [f64; 3];

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

pub(crate) type Uv = [f64; 2];

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

pub(crate) fn declined<'py>(py: Python<'py>, reason: Decline) -> PyResult<Bound<'py, PyAny>> {
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

// ---------------------------------------------------------------------------------------------------------------------------------------------
// `mesh_weld(domains, offset)`: `kernel/src/cftuv_envelope/mesh_weld.py` `weld_core` (slice B2). Positions compare by their bits (`float.hex` of a finite
// float is its bits), normals by `==`; the Gauss-Jordan pivot is the first of equal magnitudes (`max` with a key), the weights fold left from zero.
// ---------------------------------------------------------------------------------------------------------------------------------------------

const WELD_LOCATION_PREFIX: &str = "location:src:";
/// The host's law of the weld (`envelope_production_weld.WELD_LAW`): the tolerances are the adapter's decisions, passed in, not constants of the port.
#[derive(Clone, Copy)]
pub struct Law {
    pub limit: f64,
    pub pivot: f64,
    pub residual: f64,
    pub sine: f64,
}
const REASON_NOT_MEET: &str = "the offset planes do not meet in one point";
const REASON_EXCEEDS: &str = "the miter exceeds the limit";

pub struct WeldDomain {
    pub positions: Vec<Point>,
    pub normals: Vec<Point>,
    /// The id of the `location:src:` reference of each vertex (`None`: no reference, or one that does not weld).
    pub refs: Vec<Option<u32>>,
    pub walls: Vec<Vec<usize>>,
}

type Member = (usize, usize);

fn bits(point: &Point) -> [u64; 3] {
    [point[0].to_bits(), point[1].to_bits(), point[2].to_bits()]
}

/// `miter_offset`: `(offset, factor, reason)`; `offset` is `None` on a refusal, `factor` is `None` where Python's is.
pub fn miter(normals: &[Point], law: &Law) -> (Option<Point>, Option<f64>, &'static str) {
    let first = normals[0];
    if normals.iter().all(|item| item[0] == first[0] && item[1] == first[1] && item[2] == first[2]) {
        return (Some(first), Some(1.0), "");
    }
    let size = normals.len();
    let mut rows: Vec<Vec<f64>> = normals
        .iter()
        .map(|a| {
            let mut row: Vec<f64> = normals.iter().map(|b| dot(a, b)).collect();
            row.push(1.0);
            row
        })
        .collect();
    let mut pivots = Vec::new();
    let mut row = 0usize;
    for column in 0..size {
        if row == size {
            break;
        }
        let mut best = row;
        for index in row + 1..size {
            if rows[index][column].abs() > rows[best][column].abs() {
                best = index;
            }
        }
        if rows[best][column].abs() <= law.pivot {
            continue;
        }
        rows.swap(row, best);
        let scale = rows[row][column];
        for value in rows[row].iter_mut() {
            *value /= scale;
        }
        let pivot_row = rows[row].clone();
        for other in 0..size {
            if other != row && rows[other][column] != 0.0 {
                let factor = rows[other][column];
                for (a, b) in rows[other].iter_mut().zip(pivot_row.iter()) {
                    *a -= factor * b;
                }
            }
        }
        pivots.push(column);
        row += 1;
    }
    for rest in rows.iter().skip(pivots.len()) {
        if rest[size].abs() > law.residual {
            return (None, None, REASON_NOT_MEET);
        }
    }
    let mut weights = vec![0.0f64; size];
    for (row, column) in pivots.iter().enumerate() {
        weights[*column] = rows[row][size];
    }
    let mut offset: Point = [0.0, 0.0, 0.0];
    for (axis, slot) in offset.iter_mut().enumerate() {
        let mut total = 0.0f64;
        for (weight, normal) in weights.iter().zip(normals.iter()) {
            total += weight * normal[axis];
        }
        *slot = total;
    }
    if normals.iter().any(|normal| (dot(normal, &offset) - 1.0).abs() > law.residual) {
        return (None, None, REASON_NOT_MEET);
    }
    let factor = dot(&offset, &offset).sqrt();
    if factor > law.limit {
        return (None, Some(factor), REASON_EXCEEDS);
    }
    (Some(offset), Some(factor), "")
}

fn normalised(vector: Point) -> Option<Point> {
    let length = dot(&vector, &vector).sqrt();
    if length == 0.0 {
        return None;
    }
    Some([vector[0] / length, vector[1] / length, vector[2] / length])
}

pub struct WeldAnswer {
    pub positions: Vec<Point>,
    pub index: Vec<Vec<usize>>,
    pub groups: usize,
    pub merged: usize,
    pub mismatches: usize,
    pub gaps: Vec<Vec<Point>>,
    pub fallbacks: usize,
    pub factor: f64,
    pub reasons: Vec<&'static str>,
    pub lifted: usize,
    pub welded: usize,
    pub alone: usize,
    pub wall_fallbacks: usize,
    pub wall_factor: f64,
    pub wall_reasons: Vec<&'static str>,
    pub partners: Vec<(usize, usize)>,
}

fn sorted_reasons(reasons: &std::collections::BTreeSet<&'static str>) -> Vec<&'static str> {
    reasons.iter().copied().collect()
}

fn wall_heading(other: &WeldDomain, chain: &[usize], at: usize, direction: &Point, sine: f64) -> Option<(usize, Point, f64)> {
    let length = dot(direction, direction).sqrt();
    for step in [-1i64, 1] {
        let position = at as i64 + step;
        if position < 0 || position as usize >= chain.len() {
            continue;
        }
        let heading = sub(&other.positions[chain[at]], &other.positions[chain[position as usize]]);
        let reach = dot(&heading, &heading).sqrt();
        if reach == 0.0 || length == 0.0 {
            continue;
        }
        let turn = cross(direction, &heading);
        if dot(direction, &heading) > 0.0 && dot(&turn, &turn).sqrt() <= sine * length * reach {
            return Some((chain[position as usize], heading, reach));
        }
    }
    None
}

fn wall_partner(domains: &[WeldDomain], table: &HashMap<u32, Vec<(usize, usize, usize)>>, ordinal: usize, chain: &[usize], at: usize, sine: f64) -> Option<(usize, usize, Point)> {
    let domain = &domains[ordinal];
    for step in [-1i64, 1] {
        let position = at as i64 + step;
        if position < 0 || position as usize >= chain.len() {
            continue;
        }
        let anchor = chain[position as usize];
        let Some(reference) = domain.refs[anchor] else { continue };
        let direction = sub(&domain.positions[anchor], &domain.positions[chain[at]]);
        for &(other_ordinal, number, place) in table.get(&reference).map(Vec::as_slice).unwrap_or(&[]) {
            let other = &domains[other_ordinal];
            let other_chain = &other.walls[number];
            if other_ordinal == ordinal || bits(&other.positions[other_chain[place]]) != bits(&domain.positions[anchor]) {
                continue;
            }
            let Some((beyond, heading, reach)) = wall_heading(other, other_chain, place, &direction, sine) else { continue };
            let ratio = dot(&direction, &heading) / (reach * reach);
            let share = if ratio < 1.0 { ratio } else { 1.0 };
            let a = other.normals[other_chain[place]];
            let b = other.normals[beyond];
            let mixed = [(1.0 - share) * a[0] + share * b[0], (1.0 - share) * a[1] + share * b[1], (1.0 - share) * a[2] + share * b[2]];
            if let Some(blended) = normalised(mixed) {
                return Some((other_ordinal, beyond, blended));
            }
        }
    }
    None
}

/// `weld_core` over read domains (indices already checked against the arrays).
pub fn weld(domains: &[WeldDomain], offset: f64, law: &Law) -> WeldAnswer {
    use std::collections::{BTreeSet, HashSet};

    // _groups_by_reference: the references in the order they are first met (their ids are assigned in that order)
    let mut groups: Vec<Vec<Member>> = Vec::new();
    for (ordinal, domain) in domains.iter().enumerate() {
        for (local, reference) in domain.refs.iter().enumerate() {
            if let Some(id) = reference {
                let id = *id as usize;
                if id >= groups.len() {
                    groups.resize_with(id + 1, Vec::new);
                }
                groups[id].push((ordinal, local));
            }
        }
    }
    let mut classes: HashMap<Member, Member> = HashMap::new();
    let mut miters: HashMap<Member, Point> = HashMap::new();
    let (mut mismatches, mut fallbacks) = (0usize, 0usize);
    let mut gaps: Vec<Vec<Point>> = Vec::new();
    let mut worst_factor = 0.0f64;
    let mut reasons: BTreeSet<&'static str> = BTreeSet::new();
    for members in groups.iter().filter(|members| members.len() > 1) {
        let mut by_position: Vec<([u64; 3], Vec<Member>)> = Vec::new();
        for &(ordinal, local) in members {
            let key = bits(&domains[ordinal].positions[local]);
            match by_position.iter_mut().find(|(found, _)| *found == key) {
                Some((_, class)) => class.push((ordinal, local)),
                None => by_position.push((key, vec![(ordinal, local)])),
            }
        }
        if by_position.len() > 1 {
            mismatches += 1;
            gaps.push(by_position.iter().map(|(_, class)| domains[class[0].0].positions[class[0].1]).collect());
        }
        for (_, class) in &by_position {
            if class.len() < 2 {
                continue;
            }
            let normals: Vec<Point> = class.iter().map(|&(o, l)| domains[o].normals[l]).collect();
            match miter(&normals, law) {
                (Some(shift), _, _) => {
                    for member in class {
                        classes.insert(*member, class[0]);
                    }
                    miters.insert(class[0], shift);
                }
                (None, factor, reason) => {
                    fallbacks += 1;
                    reasons.insert(reason);
                    let factor = factor.unwrap_or(0.0);
                    if factor > worst_factor {
                        worst_factor = factor;
                    }
                }
            }
        }
    }
    let merged = classes.len() - miters.len();
    let plan_groups = miters.len();

    // plan_wall_nodes
    let mut table: HashMap<u32, Vec<(usize, usize, usize)>> = HashMap::new();
    for (ordinal, domain) in domains.iter().enumerate() {
        for (number, chain) in domain.walls.iter().enumerate() {
            for (at, &local) in chain.iter().enumerate() {
                if let Some(reference) = domain.refs[local] {
                    table.entry(reference).or_default().push((ordinal, number, at));
                }
            }
        }
    }
    let mut lifts: HashMap<Member, Point> = HashMap::new();
    let mut wall_classes: HashMap<Member, Member> = HashMap::new();
    let mut wall_miters: HashMap<Member, Point> = HashMap::new();
    let mut seen: HashSet<Member> = HashSet::new();
    let (mut lifted, mut alone, mut wall_fallbacks) = (0usize, 0usize, 0usize);
    let mut wall_worst = 0.0f64;
    let mut wall_reasons: BTreeSet<&'static str> = BTreeSet::new();
    let mut partners: BTreeSet<(usize, usize)> = BTreeSet::new();
    for (ordinal, domain) in domains.iter().enumerate() {
        for chain in &domain.walls {
            for (at, &local) in chain.iter().enumerate() {
                if domain.refs[local].is_some() || seen.contains(&(ordinal, local)) {
                    continue;
                }
                seen.insert((ordinal, local));
                let Some((other_ordinal, beyond, blended)) = wall_partner(domains, &table, ordinal, chain, at, law.sine) else {
                    alone += 1;
                    continue;
                };
                let other = &domains[other_ordinal];
                let twin = other.refs[beyond].is_none() && !seen.contains(&(other_ordinal, beyond)) && bits(&other.positions[beyond]) == bits(&domain.positions[local]);
                let pair = [domain.normals[local], if twin { other.normals[beyond] } else { blended }];
                let (shift, factor, reason) = miter(&pair, law);
                let Some(shift) = shift else {
                    wall_fallbacks += 1;
                    wall_reasons.insert(reason);
                    let factor = factor.unwrap_or(0.0);
                    if factor > wall_worst {
                        wall_worst = factor;
                    }
                    continue;
                };
                if !twin {
                    lifts.insert((ordinal, local), shift);
                    partners.insert((ordinal, other_ordinal));
                    lifted += 1;
                    continue;
                }
                let head = (ordinal, local);
                seen.insert((other_ordinal, beyond));
                wall_classes.insert(head, head);
                wall_classes.insert((other_ordinal, beyond), head);
                wall_miters.insert(head, shift);
                lifted += 2;
            }
        }
    }
    let welded = wall_classes.len() - wall_miters.len();
    classes.extend(wall_classes);
    miters.extend(wall_miters);

    let mut positions: Vec<Point> = Vec::new();
    let mut placed: HashMap<Member, usize> = HashMap::new();
    let mut index: Vec<Vec<usize>> = Vec::with_capacity(domains.len());
    for (ordinal, domain) in domains.iter().enumerate() {
        let mut mapping = Vec::with_capacity(domain.positions.len());
        for (local, point) in domain.positions.iter().enumerate() {
            let head = classes.get(&(ordinal, local)).copied();
            if let Some(found) = head.and_then(|head| placed.get(&head).copied()) {
                mapping.push(found);
                continue;
            }
            let shift = match head {
                Some(head) => miters[&head],
                None => lifts.get(&(ordinal, local)).copied().unwrap_or(domain.normals[local]),
            };
            positions.push([point[0] + offset * shift[0], point[1] + offset * shift[1], point[2] + offset * shift[2]]);
            mapping.push(positions.len() - 1);
            if let Some(head) = head {
                placed.insert(head, positions.len() - 1);
            }
        }
        index.push(mapping);
    }
    WeldAnswer {
        positions,
        index,
        groups: plan_groups,
        merged,
        mismatches,
        gaps,
        fallbacks,
        factor: worst_factor,
        reasons: sorted_reasons(&reasons),
        lifted,
        welded,
        alone,
        wall_fallbacks,
        wall_factor: wall_worst,
        wall_reasons: sorted_reasons(&wall_reasons),
        partners: partners.into_iter().collect(),
    }
}

fn read_points(sequence: &Bound<'_, PyAny>, what: &str) -> Result<Vec<Point>, Decline> {
    items(sequence, what)?.iter().map(|item| floats(item, 3, what).map(|v| [v[0], v[1], v[2]])).collect()
}

fn read_weld_domains(domains: &Bound<'_, PyAny>) -> Result<Vec<WeldDomain>, Decline> {
    let mut ids: HashMap<String, u32> = HashMap::new();
    let mut found = Vec::new();
    for domain in items(domains, "domains")? {
        let attribute = |name: &str| domain.getattr(name).map_err(|_| format!("a domain has no `{name}`"));
        let positions = read_points(&attribute("positions")?, "a position")?;
        let normals = read_points(&attribute("normals")?, "a normal")?;
        let mut refs = Vec::new();
        for reference in items(&attribute("refs")?, "refs")? {
            if reference.is_none() {
                refs.push(None);
                continue;
            }
            if !reference.is_exact_instance_of::<pyo3::types::PyString>() {
                return Err("a vertex reference is not an exact str".to_string());
            }
            let text = reference.cast::<pyo3::types::PyString>().map_err(|_| "a vertex reference is not a str".to_string())?.to_str().map_err(|_| "a vertex reference is not UTF-8".to_string())?;
            if !text.starts_with(WELD_LOCATION_PREFIX) {
                refs.push(None);
                continue;
            }
            let next = ids.len() as u32;
            refs.push(Some(*ids.entry(text.to_string()).or_insert(next)));
        }
        if normals.len() != positions.len() || refs.len() != positions.len() {
            return Err("the vertex rows of a domain differ in length".to_string());
        }
        let walls_object = match domain.getattr("walls") {
            Ok(walls) => walls,
            Err(_) => PyTuple::empty(domain.py()).into_any(),
        };
        let mut walls = Vec::new();
        for chain in items(&walls_object, "walls")? {
            let mut locals = Vec::new();
            for vertex in items(&chain, "a wall")? {
                if !vertex.is_exact_instance_of::<pyo3::types::PyInt>() {
                    return Err("a wall vertex is not an exact int".to_string());
                }
                let local = vertex.extract::<i64>().map_err(|_| "a wall vertex is outside i64".to_string())?;
                if local < 0 || local as u64 >= positions.len() as u64 {
                    return Err(format!("wall vertex {local} is outside the domain"));
                }
                locals.push(local as usize);
            }
            walls.push(locals);
        }
        found.push(WeldDomain { positions, normals, refs, walls });
    }
    Ok(found)
}

fn point_tuple<'py>(py: Python<'py>, point: &Point) -> PyResult<Bound<'py, PyTuple>> {
    PyTuple::new(py, point.iter().copied())
}

/// The tuple of `weld_core` or the text of a decline.
#[pyfunction]
pub fn mesh_weld<'py>(py: Python<'py>, domains: &Bound<'py, PyAny>, offset: &Bound<'py, PyAny>, law: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyAny>> {
    let read = || -> Result<(Vec<WeldDomain>, f64, Law), Decline> {
        let numbers = floats(law, 4, "the weld law")?;
        let law = Law { limit: numbers[0], pivot: numbers[1], residual: numbers[2], sine: numbers[3] };
        Ok((read_weld_domains(domains)?, exact_float(offset, "the offset")?, law))
    };
    let (domains, offset, law) = match read() {
        Ok(found) => found,
        Err(reason) => return declined(py, reason),
    };
    let answer = weld(&domains, offset, &law);
    let positions = PyTuple::new(py, answer.positions.iter().map(|point| point_tuple(py, point)).collect::<PyResult<Vec<_>>>()?)?;
    let index = PyTuple::new(py, answer.index.iter().map(|mapping| PyTuple::new(py, mapping.iter().copied())).collect::<PyResult<Vec<_>>>()?)?;
    let gaps = PyTuple::new(
        py,
        answer.gaps.iter().map(|points| PyTuple::new(py, points.iter().map(|point| point_tuple(py, point)).collect::<PyResult<Vec<_>>>()?)).collect::<PyResult<Vec<_>>>()?,
    )?;
    let reasons = PyTuple::new(py, answer.reasons.iter().copied())?;
    let wall_reasons = PyTuple::new(py, answer.wall_reasons.iter().copied())?;
    let partners = PyTuple::new(py, answer.partners.iter().copied())?;
    let parts: Vec<Bound<'py, PyAny>> = vec![
        positions.into_any(),
        index.into_any(),
        answer.groups.into_pyobject(py)?.into_any(),
        answer.merged.into_pyobject(py)?.into_any(),
        answer.mismatches.into_pyobject(py)?.into_any(),
        gaps.into_any(),
        answer.fallbacks.into_pyobject(py)?.into_any(),
        PyFloat::new(py, answer.factor).into_any(),
        reasons.into_any(),
        answer.lifted.into_pyobject(py)?.into_any(),
        answer.welded.into_pyobject(py)?.into_any(),
        answer.alone.into_pyobject(py)?.into_any(),
        answer.wall_fallbacks.into_pyobject(py)?.into_any(),
        PyFloat::new(py, answer.wall_factor).into_any(),
        wall_reasons.into_any(),
        partners.into_any(),
    ];
    Ok(PyTuple::new(py, parts)?.into_any())
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// `mesh_reports(positions, faces, uvs, face_domain, face_owner, seams, chains)` (slice B3): the reports of the assembled mesh in one read of the arrays —
// `mesh_checks.arrays_digest` (the JSON of `json.dumps(sort_keys=True, separators=(",", ":"))`, floats by CPython's own `float.__repr__` through
// `PyOS_double_to_string(x, 'r', 0, Py_DTSF_ADD_DOT_0)`), `seam_report_numbers`, `off_plane_measure` and `half_edge_conflicts`.
// ---------------------------------------------------------------------------------------------------------------------------------------------

fn float_repr(out: &mut Vec<u8>, value: f64) -> Result<(), Decline> {
    // SAFETY: the GIL is held (the caller is a #[pyfunction]); the buffer is CPython's, freed with PyMem_Free after the copy.
    unsafe {
        let text = pyo3::ffi::PyOS_double_to_string(value, b'r' as std::ffi::c_char, 0, pyo3::ffi::Py_DTSF_ADD_DOT_0, std::ptr::null_mut());
        if text.is_null() {
            return Err("float.__repr__ failed".to_string());
        }
        out.extend_from_slice(std::ffi::CStr::from_ptr(text).to_bytes());
        pyo3::ffi::PyMem_Free(text as *mut std::ffi::c_void);
    }
    Ok(())
}

fn json_ints<I: IntoIterator<Item = i64>>(out: &mut Vec<u8>, values: I) {
    out.push(b'[');
    let mut first = true;
    for value in values {
        if !first {
            out.push(b',');
        }
        first = false;
        out.extend_from_slice(value.to_string().as_bytes());
    }
    out.push(b']');
}

fn json_floats(out: &mut Vec<u8>, values: &[f64]) -> Result<(), Decline> {
    out.push(b'[');
    for (at, value) in values.iter().enumerate() {
        if at > 0 {
            out.push(b',');
        }
        float_repr(out, *value)?;
    }
    out.push(b']');
    Ok(())
}

/// `mesh_checks.arrays_digest` over read arrays.
pub fn arrays_digest(positions: &[Point], faces: &[Vec<i64>], uvs: &[Uv], face_domain: &[i64], face_owner: &[i64], seams: &[(i64, i64)]) -> Result<String, Decline> {
    use sha2::Digest;

    let mut out: Vec<u8> = Vec::with_capacity(64 * (positions.len() + uvs.len()) + 16 * faces.len());
    out.extend_from_slice(b"{\"face_domain\":");
    json_ints(&mut out, face_domain.iter().copied());
    out.extend_from_slice(b",\"face_owner\":");
    json_ints(&mut out, face_owner.iter().copied());
    out.extend_from_slice(b",\"faces\":[");
    for (at, face) in faces.iter().enumerate() {
        if at > 0 {
            out.push(b',');
        }
        json_ints(&mut out, face.iter().copied());
    }
    out.extend_from_slice(b"],\"positions\":[");
    for (at, point) in positions.iter().enumerate() {
        if at > 0 {
            out.push(b',');
        }
        json_floats(&mut out, point)?;
    }
    out.extend_from_slice(b"],\"seams\":[");
    for (at, (a, b)) in seams.iter().enumerate() {
        if at > 0 {
            out.push(b',');
        }
        json_ints(&mut out, [*a, *b]);
    }
    out.extend_from_slice(b"],\"uvs\":[");
    for (at, uv) in uvs.iter().enumerate() {
        if at > 0 {
            out.push(b',');
        }
        json_floats(&mut out, uv)?;
    }
    out.extend_from_slice(b"]}");
    let digest = sha2::Sha256::digest(&out);
    Ok(digest.iter().map(|byte| format!("{byte:02x}")).collect())
}

/// `_run_between`: a path along `anchors` from one end of `pair` to the other through `src:` keys the pair's domain does not hold.
fn run_between(anchors: &[u32], pair: (u32, u32), own: &std::collections::HashSet<u32>) -> bool {
    let inside = |key: u32| key == pair.0 || key == pair.1;
    for (at, &key) in anchors.iter().enumerate() {
        if !inside(key) {
            continue;
        }
        for stop in at + 1..anchors.len() {
            if anchors[stop] == key {
                break;
            }
            if inside(anchors[stop]) {
                let between = &anchors[at + 1..stop];
                if !between.is_empty() && between.iter().all(|item| !own.contains(item)) {
                    return true;
                }
                break;
            }
        }
    }
    false
}

/// `seam_report_numbers` over interned chains: per domain `(kind, keys)` with kind 0 SOURCE, 1 WALL, 2 other; keys as ids, `src`/`clip` flags per id.
pub fn seam_numbers(domains: &[Vec<(u8, Vec<u32>)>], is_src: &[bool], is_clip: &[bool]) -> (u64, u64) {
    use std::collections::{HashMap, HashSet};

    let pair_of = |a: u32, b: u32| if a <= b { (a, b) } else { (b, a) };
    let mut by_pair: HashMap<(u32, u32), Vec<i64>> = HashMap::new();
    let mut clip_vertices = 0u64;
    let mut anchored: Vec<(usize, Vec<u32>)> = Vec::new();
    let mut held: HashMap<usize, HashSet<u32>> = HashMap::new();
    for (number, chains) in domains.iter().enumerate() {
        for (kind, keys) in chains {
            if *kind > 1 {
                continue;
            }
            let anchors: Vec<usize> = (0..keys.len()).filter(|&index| is_src[keys[index] as usize]).collect();
            clip_vertices += keys.iter().filter(|key| is_clip[**key as usize]).count() as u64;
            held.entry(number).or_default().extend(anchors.iter().map(|&index| keys[index]));
            if *kind != 0 {
                continue;
            }
            anchored.push((number, anchors.iter().map(|&index| keys[index]).collect()));
            for window in anchors.windows(2) {
                by_pair.entry(pair_of(keys[window[0]], keys[window[1]])).or_default().push((window[1] - window[0] - 1) as i64);
            }
        }
    }
    let mut junctions = by_pair
        .values()
        .filter(|counts| counts.len() > 1 && counts.iter().any(|count| *count != counts[0]))
        .count() as u64;
    // _anchors_between
    let mut runs: HashMap<usize, Vec<&Vec<u32>>> = HashMap::new();
    let mut consecutive: HashMap<usize, HashSet<(u32, u32)>> = HashMap::new();
    let mut where_: HashMap<u32, HashSet<(usize, usize)>> = HashMap::new();
    for (number, anchors) in &anchored {
        let chains = runs.entry(*number).or_default();
        for key in anchors {
            where_.entry(*key).or_default().insert((*number, chains.len()));
        }
        chains.push(anchors);
        consecutive.entry(*number).or_default().extend(anchors.windows(2).map(|window| pair_of(window[0], window[1])));
    }
    let empty: HashSet<u32> = HashSet::new();
    let mut open_pairs: HashSet<(u32, u32)> = HashSet::new();
    for (number, pairs) in &consecutive {
        for &pair in pairs {
            if pair.0 == pair.1 || open_pairs.contains(&pair) {
                continue;
            }
            let (Some(first), Some(second)) = (where_.get(&pair.0), where_.get(&pair.1)) else { continue };
            let own = held.get(number).unwrap_or(&empty);
            for &(other, at) in first.intersection(second) {
                if other != *number && !consecutive[&other].contains(&pair) && run_between(runs[&other][at], pair, own) {
                    open_pairs.insert(pair);
                    break;
                }
            }
        }
    }
    junctions += open_pairs.len() as u64;
    (junctions, clip_vertices)
}

fn read_int_list(sequence: &Bound<'_, PyAny>, what: &str) -> Result<Vec<i64>, Decline> {
    items(sequence, what)?
        .iter()
        .map(|item| {
            if !item.is_exact_instance_of::<pyo3::types::PyInt>() {
                return Err(format!("{what} holds a value that is not an exact int"));
            }
            item.extract::<i64>().map_err(|_| format!("{what} holds an int outside i64"))
        })
        .collect()
}

type Chains = (Vec<Vec<(u8, Vec<u32>)>>, Vec<bool>, Vec<bool>);

fn read_chains(chains: &Bound<'_, PyAny>) -> Result<Chains, Decline> {
    let mut ids: HashMap<String, u32> = HashMap::new();
    let (mut is_src, mut is_clip) = (Vec::new(), Vec::new());
    let mut domains = Vec::new();
    for domain in items(chains, "chains")? {
        let mut found = Vec::new();
        for chain in items(&domain, "the chains of a domain")? {
            let parts = items(&chain, "a chain")?;
            if parts.len() != 2 || !parts[0].is_exact_instance_of::<pyo3::types::PyString>() {
                return Err("a chain is not a (name, keys) pair".to_string());
            }
            let name = parts[0].cast::<pyo3::types::PyString>().map_err(|_| "a chain name is not a str".to_string())?.to_str().map_err(|_| "a chain name is not UTF-8".to_string())?;
            let kind = match name.split(':').nth(1) {
                Some("SOURCE") => 0u8,
                Some("WALL") => 1u8,
                Some(_) => 2u8,
                None => return Err("a chain name has no kind (Python's split raises)".to_string()),
            };
            let mut keys = Vec::new();
            for key in items(&parts[1], "the keys of a chain")? {
                if !key.is_exact_instance_of::<pyo3::types::PyString>() {
                    return Err("a chain key is not an exact str".to_string());
                }
                let text = key.cast::<pyo3::types::PyString>().map_err(|_| "a chain key is not a str".to_string())?.to_str().map_err(|_| "a chain key is not UTF-8".to_string())?;
                let next = ids.len() as u32;
                let id = *ids.entry(text.to_string()).or_insert_with(|| {
                    is_src.push(text.starts_with("src:"));
                    is_clip.push(text.starts_with("clip:"));
                    next
                });
                keys.push(id);
            }
            found.push((kind, keys));
        }
        domains.push(found);
    }
    Ok((domains, is_src, is_clip))
}

/// `(digest, (junctions, clip vertices), (worst, off), conflicts)` or the text of a decline.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn mesh_reports<'py>(
    py: Python<'py>,
    positions: &Bound<'py, PyAny>,
    faces: &Bound<'py, PyAny>,
    uvs: &Bound<'py, PyAny>,
    face_domain: &Bound<'py, PyAny>,
    face_owner: &Bound<'py, PyAny>,
    seams: &Bound<'py, PyAny>,
    chains: &Bound<'py, PyAny>,
) -> PyResult<Bound<'py, PyAny>> {
    let answer = || -> Result<(String, (u64, u64), (f64, u64), u64), Decline> {
        let points = read_points(positions, "a position")?;
        let loops = read_faces(faces)?;
        let uv = items(uvs, "uvs")?.iter().map(|item| floats(item, 2, "a uv").map(|v| [v[0], v[1]])).collect::<Result<Vec<Uv>, Decline>>()?;
        let domains = read_int_list(face_domain, "face_domain")?;
        let owners = read_int_list(face_owner, "face_owner")?;
        let mut pairs = Vec::new();
        for seam in items(seams, "seams")? {
            let pair = read_int_list(&seam, "a seam")?;
            if pair.len() != 2 {
                return Err("a seam is not a pair".to_string());
            }
            pairs.push((pair[0], pair[1]));
        }
        let (chain_domains, is_src, is_clip) = read_chains(chains)?;
        let digest = arrays_digest(&points, &loops, &uv, &domains, &owners, &pairs)?;
        Ok((digest, seam_numbers(&chain_domains, &is_src, &is_clip), off_plane(&points, &loops)?, half_edges(&loops)))
    };
    match answer() {
        Ok(found) => Ok(found.into_pyobject(py)?.into_any()),
        Err(reason) => declined(py, reason),
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// `mesh_assemble(views, patch_ids, offset, law, reports)` (slice B4a, MESH_ASSEMBLY_BUFFERS_V1): the whole `build_mesh_arrays` of the materialized domains
// straight from their VIEW_BYTES_V1 (`cftuv/envelope_view_bytes.py`, `_HEADER = "<4sHHq14I20s"`) into flat buffers — the parse (`decode_view`), the weld
// (`weld_core`), the glue (`mesh_glue.glue_domains`), the fold seams (`cross_domain_seams`) and, when asked, the reports (`mesh_reports`), in one pass. The
// views are read without the GIL; only the digest (CPython's own float repr) holds it. What `decode_view` or the oracles would not answer the same way is
// declined by a text: a view that is not VIEW_BYTES_V1, a failed view, a truncated section, an offset run that goes backwards, an index outside its table,
// a text that is not UTF-8 (or a token that is not ASCII), a non-finite position, normal, UV or welded position.
// ---------------------------------------------------------------------------------------------------------------------------------------------

const VIEW_MAGIC: &[u8; 4] = b"CFVB";
const VIEW_VERSION: u16 = 1;
const VIEW_FAILED: u16 = 1;
const VIEW_HEADER_BYTES: usize = 4 + 2 + 2 + 8 + 14 * 4 + 20;

/// One parsed VIEW_BYTES_V1: the arrays `decode_view` builds its tuples from, with the texts borrowed from the bytes.
pub struct ParsedView<'a> {
    pub positions: Vec<Point>,
    pub normals: Vec<Point>,
    pub uvs: Vec<Uv>,
    pub owners: Vec<i64>,
    pub faces: Vec<Vec<usize>>,
    pub seams: Vec<(usize, usize)>,
    /// `0` - `None`, else the number of the text + 1.
    pub refs: Vec<u32>,
    pub walls: Vec<Vec<usize>>,
    pub chains: Vec<(usize, Vec<usize>)>,
    pub texts: Vec<&'a str>,
    pub revision: usize,
    pub token: &'a str,
}

struct Reader<'a> {
    data: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, size: usize, what: &str) -> Result<&'a [u8], Decline> {
        let end = self.at.checked_add(size).ok_or_else(|| format!("the {what} of a view overflow"))?;
        if end > self.data.len() {
            return Err(format!("the view bytes end inside the {what}"));
        }
        let part = &self.data[self.at..end];
        self.at = end;
        Ok(part)
    }

    fn f64s(&mut self, count: usize, what: &str) -> Result<Vec<f64>, Decline> {
        let part = self.take(count.checked_mul(8).ok_or_else(|| format!("the {what} of a view overflow"))?, what)?;
        Ok(part.chunks_exact(8).map(|chunk| f64::from_le_bytes(chunk.try_into().unwrap())).collect())
    }

    fn i64s(&mut self, count: usize, what: &str) -> Result<Vec<i64>, Decline> {
        let part = self.take(count.checked_mul(8).ok_or_else(|| format!("the {what} of a view overflow"))?, what)?;
        Ok(part.chunks_exact(8).map(|chunk| i64::from_le_bytes(chunk.try_into().unwrap())).collect())
    }

    fn u32s(&mut self, count: usize, what: &str) -> Result<Vec<u32>, Decline> {
        let part = self.take(count.checked_mul(4).ok_or_else(|| format!("the {what} of a view overflow"))?, what)?;
        Ok(part.chunks_exact(4).map(|chunk| u32::from_le_bytes(chunk.try_into().unwrap())).collect())
    }
}

fn finite_points(values: Vec<f64>, what: &str) -> Result<Vec<Point>, Decline> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(format!("{what} is not finite"));
    }
    Ok(values.chunks_exact(3).map(|chunk| [chunk[0], chunk[1], chunk[2]]).collect())
}

/// `(starts, flat)` into rows; the starts begin at 0, never go back and end at the flat length (else `decode_view`'s slices would differ).
fn rows(starts: &[u32], flat: &[u32], bound: usize, what: &str) -> Result<Vec<Vec<usize>>, Decline> {
    if starts.first() != Some(&0) || *starts.last().unwrap() as usize != flat.len() || starts.windows(2).any(|pair| pair[1] < pair[0]) {
        return Err(format!("the starts of the {what} are not a run over its items"));
    }
    let mut found = Vec::with_capacity(starts.len() - 1);
    for pair in starts.windows(2) {
        let row = &flat[pair[0] as usize..pair[1] as usize];
        if row.iter().any(|item| *item as usize >= bound) {
            return Err(format!("an item of the {what} is outside its table"));
        }
        found.push(row.iter().map(|item| *item as usize).collect());
    }
    Ok(found)
}

/// One VIEW_BYTES_V1 parsed as `decode_view` reads it; `Err` declines.
pub fn parse_view(data: &[u8]) -> Result<ParsedView<'_>, Decline> {
    if data.len() < VIEW_HEADER_BYTES || &data[0..4] != VIEW_MAGIC || u16::from_le_bytes([data[4], data[5]]) != VIEW_VERSION {
        return Err("not VIEW_BYTES_V1 bytes".to_string());
    }
    if u16::from_le_bytes([data[6], data[7]]) & VIEW_FAILED != 0 {
        return Err("a failed view reached the assembly".to_string());
    }
    let count = |at: usize| u32::from_le_bytes(data[16 + 4 * at..20 + 4 * at].try_into().unwrap()) as usize;
    let (vertices, faces, corners, loops, seams, walls, wall_items, chains, chain_items, strings, blob) =
        (count(0), count(1), count(2), count(3), count(4), count(5), count(6), count(7), count(8), count(9), count(10));
    let (revision, token_bytes) = (count(11), &data[72..92]);
    let mut reader = Reader { data, at: VIEW_HEADER_BYTES };
    let positions = finite_points(reader.f64s(3 * vertices, "positions")?, "a position")?;
    let normals = finite_points(reader.f64s(3 * vertices, "normals")?, "a normal")?;
    let uv_values = reader.f64s(2 * loops, "uvs")?;
    if uv_values.iter().any(|value| !value.is_finite()) {
        return Err("a uv is not finite".to_string());
    }
    let uvs = uv_values.chunks_exact(2).map(|chunk| [chunk[0], chunk[1]]).collect();
    let owners = reader.i64s(faces, "owners")?;
    let face_starts = reader.u32s(faces + 1, "face starts")?;
    let face_corners = reader.u32s(corners, "face corners")?;
    let seam_flat = reader.u32s(2 * seams, "seams")?;
    let refs = reader.u32s(vertices, "vertex references")?;
    let wall_starts = reader.u32s(walls + 1, "wall starts")?;
    let wall_flat = reader.u32s(wall_items, "wall vertices")?;
    let chain_names = reader.u32s(chains, "chain names")?;
    let chain_starts = reader.u32s(chains + 1, "chain starts")?;
    let chain_flat = reader.u32s(chain_items, "chain keys")?;
    let offsets = reader.u32s(strings + 1, "text offsets")?;
    let text_blob = reader.take(blob, "texts")?;
    let mut texts = Vec::with_capacity(strings);
    if offsets.windows(2).any(|pair| pair[1] < pair[0]) || offsets.iter().any(|offset| *offset as usize > blob) {
        return Err("the text offsets are not a run over the texts".to_string());
    }
    for pair in offsets.windows(2) {
        texts.push(std::str::from_utf8(&text_blob[pair[0] as usize..pair[1] as usize]).map_err(|_| "a text of a view is not UTF-8".to_string())?);
    }
    if revision >= strings || refs.iter().any(|item| *item as usize > strings) || chain_names.iter().any(|item| *item as usize >= strings) {
        return Err("a text number of a view is outside its table".to_string());
    }
    let end = token_bytes.iter().rposition(|byte| *byte != 0).map_or(0, |at| at + 1);
    let token = std::str::from_utf8(&token_bytes[..end]).ok().filter(|text| text.is_ascii() && !text.is_empty()).ok_or("the structure token is not ASCII or is empty (the host would compute it)")?;
    let seams = seam_flat
        .chunks_exact(2)
        .map(|pair| if (pair[0] as usize) < vertices && (pair[1] as usize) < vertices { Ok((pair[0] as usize, pair[1] as usize)) } else { Err("a seam vertex is outside the domain".to_string()) })
        .collect::<Result<Vec<_>, Decline>>()?;
    let chain_rows = rows(&chain_starts, &chain_flat, strings, "chain keys")?;
    Ok(ParsedView {
        positions,
        normals,
        uvs,
        owners,
        faces: rows(&face_starts, &face_corners, vertices, "face corners")?,
        seams,
        refs,
        walls: rows(&wall_starts, &wall_flat, vertices, "wall vertices")?,
        chains: chain_names.iter().map(|name| *name as usize).zip(chain_rows).collect(),
        texts,
        revision,
        token,
    })
}

/// What `mesh_assemble` answers, before the reports' digest (which needs the GIL).
pub struct Assembled {
    pub weld: WeldAnswer,
    pub faces: Vec<Vec<i64>>,
    pub uvs: Vec<Uv>,
    pub face_domain: Vec<i64>,
    pub face_owner: Vec<i64>,
    pub seams: Vec<(i64, i64)>,
    pub folds: usize,
    pub vertex_counts: Vec<usize>,
    pub loop_counts: Vec<usize>,
    /// `(junctions, clip vertices)`, `(worst, off)`, conflicts: only when the reports are asked.
    pub reports: Option<((u64, u64), (f64, u64), u64)>,
}

/// The chains of the parsed views interned as `read_chains` does: the keys numbered in the order they are first met over all domains.
fn intern_chains(views: &[ParsedView<'_>]) -> Result<Chains, Decline> {
    let mut ids: HashMap<&str, u32> = HashMap::new();
    let (mut is_src, mut is_clip) = (Vec::new(), Vec::new());
    let mut domains = Vec::with_capacity(views.len());
    for view in views {
        let mut found = Vec::with_capacity(view.chains.len());
        for (name, keys) in &view.chains {
            let kind = match view.texts[*name].split(':').nth(1) {
                Some("SOURCE") => 0u8,
                Some("WALL") => 1u8,
                Some(_) => 2u8,
                None => return Err("a chain name has no kind (Python's split raises)".to_string()),
            };
            let mut numbers = Vec::with_capacity(keys.len());
            for key in keys {
                let text = view.texts[*key];
                let next = ids.len() as u32;
                let id = *ids.entry(text).or_insert_with(|| {
                    is_src.push(text.starts_with("src:"));
                    is_clip.push(text.starts_with("clip:"));
                    next
                });
                numbers.push(id);
            }
            found.push((kind, numbers));
        }
        domains.push(found);
    }
    Ok((domains, is_src, is_clip))
}

/// The weld input of parsed views as `read_weld_domains` reads the domain objects: the `location:src:` references numbered in the order they are first met.
pub fn weld_domains(views: &[ParsedView<'_>]) -> Vec<WeldDomain> {
    let mut ids: HashMap<&str, u32> = HashMap::new();
    let mut domains = Vec::with_capacity(views.len());
    for view in views {
        let refs = view
            .refs
            .iter()
            .map(|item| {
                if *item == 0 {
                    return None;
                }
                let text = view.texts[*item as usize - 1];
                if !text.starts_with(WELD_LOCATION_PREFIX) {
                    return None;
                }
                let next = ids.len() as u32;
                Some(*ids.entry(text).or_insert(next))
            })
            .collect();
        domains.push(WeldDomain { positions: view.positions.clone(), normals: view.normals.clone(), refs, walls: view.walls.clone() });
    }
    domains
}

/// `build_mesh_arrays` over parsed views (without the digest); `Err` declines.
pub fn assemble(views: &[ParsedView<'_>], patch_ids: &[i64], offset: f64, law: &Law, reports: bool) -> Result<Assembled, Decline> {
    let domains = weld_domains(views);
    let weld = weld(&domains, offset, law);
    if weld.positions.iter().any(|point| point.iter().any(|value| !value.is_finite())) {
        return Err("a welded position is not finite".to_string());
    }
    // mesh_glue.glue_domains
    let total_faces: usize = views.iter().map(|view| view.faces.len()).sum();
    let mut faces: Vec<Vec<i64>> = Vec::with_capacity(total_faces);
    let mut uvs: Vec<Uv> = Vec::with_capacity(views.iter().map(|view| view.uvs.len()).sum());
    let (mut face_domain, mut face_owner) = (Vec::with_capacity(total_faces), Vec::with_capacity(total_faces));
    let mut pairs: std::collections::BTreeSet<(i64, i64)> = std::collections::BTreeSet::new();
    for ((view, mapping), patch_id) in views.iter().zip(&weld.index).zip(patch_ids) {
        if view.owners.len() != view.faces.len() {
            return Err("the face owners of a view are not one per face".to_string());
        }
        for face in &view.faces {
            faces.push(face.iter().map(|local| mapping[*local] as i64).collect());
        }
        uvs.extend_from_slice(&view.uvs);
        face_domain.extend(std::iter::repeat_n(*patch_id, view.faces.len()));
        face_owner.extend_from_slice(&view.owners);
        for (a, b) in &view.seams {
            let (a, b) = (mapping[*a] as i64, mapping[*b] as i64);
            pairs.insert(if a <= b { (a, b) } else { (b, a) });
        }
    }
    let folds = cross_seams(&faces, &uvs, &face_domain)?;
    let fold_count = folds.len();
    pairs.extend(folds);
    let seams: Vec<(i64, i64)> = pairs.into_iter().collect();
    let found_reports = if reports {
        let (chain_domains, is_src, is_clip) = intern_chains(views)?;
        Some((seam_numbers(&chain_domains, &is_src, &is_clip), off_plane(&weld.positions, &faces)?, half_edges(&faces)))
    } else {
        None
    };
    Ok(Assembled {
        vertex_counts: views.iter().map(|view| view.positions.len()).collect(),
        loop_counts: views.iter().map(|view| view.uvs.len()).collect(),
        weld,
        faces,
        uvs,
        face_domain,
        face_owner,
        seams,
        folds: fold_count,
        reports: found_reports,
    })
}

pub(crate) fn bytes_of<'py, T: Copy, const N: usize>(py: Python<'py>, values: impl ExactSizeIterator<Item = T>, to_bytes: impl Fn(T) -> [u8; N]) -> PyResult<Bound<'py, pyo3::types::PyBytes>> {
    let size = values.len() * N;
    let mut values = values;
    pyo3::types::PyBytes::new_with(py, size, |buffer| {
        for chunk in buffer.chunks_exact_mut(N) {
            chunk.copy_from_slice(&to_bytes(values.next().expect("the size was counted")));
        }
        Ok(())
    })
}

/// The buffers of the assembled mesh, or the text of a decline.
///
/// `(positions f64x3, face starts u32 (F+1), face corners u32, uvs f64x2, face domain i64, face owner i64, seams u32x2, weld index u32 (the local
/// vertices of every domain in turn), vertex counts, loop counts, tokens, revisions, weld numbers, folds, reports | None)`; the weld numbers are the
/// tuple of `weld_core` without its positions and index, the reports `(digest, (junctions, clip vertices), (worst, off), conflicts)`.
/// `digest=False` (MESH_DIGEST_OFF_CONFIRM_V1, the commit of the live width): the reports without `arrays_digest` (an empty text in its place; the other reports the same).
#[pyfunction]
#[pyo3(signature = (views, patch_ids, offset, law, reports, digest=true))]
pub fn mesh_assemble<'py>(py: Python<'py>, views: &Bound<'py, PyAny>, patch_ids: &Bound<'py, PyAny>, offset: &Bound<'py, PyAny>, law: &Bound<'py, PyAny>, reports: bool, digest: bool) -> PyResult<Bound<'py, PyAny>> {
    let read = || -> Result<(Vec<Bound<'py, pyo3::types::PyBytes>>, Vec<i64>, f64, Law), Decline> {
        let numbers = floats(law, 4, "the weld law")?;
        let law = Law { limit: numbers[0], pivot: numbers[1], residual: numbers[2], sine: numbers[3] };
        let data = items(views, "views")?
            .into_iter()
            .map(|item| item.cast_into::<pyo3::types::PyBytes>().map_err(|_| "a view is not bytes (it travels as an object)".to_string()))
            .collect::<Result<Vec<_>, Decline>>()?;
        let ids = read_int_list(patch_ids, "patch_ids")?;
        if ids.len() != data.len() {
            return Err("one patch id per view is expected".to_string());
        }
        Ok((data, ids, exact_float(offset, "the offset")?, law))
    };
    let (data, ids, offset, law) = match read() {
        Ok(found) => found,
        Err(reason) => return declined(py, reason),
    };
    let slices: Vec<&[u8]> = data.iter().map(|item| item.as_bytes()).collect();
    let computed = py.detach(|| -> Result<(Assembled, Vec<String>, Vec<String>), Decline> {
        let parsed = slices.iter().map(|bytes| parse_view(bytes)).collect::<Result<Vec<_>, Decline>>()?;
        let tokens = parsed.iter().map(|view| view.token.to_string()).collect();
        let revisions = parsed.iter().map(|view| view.texts[view.revision].to_string()).collect();
        Ok((assemble(&parsed, &ids, offset, &law, reports)?, tokens, revisions))
    });
    let (answer, tokens, revisions) = match computed {
        Ok(found) => found,
        Err(reason) => return declined(py, reason),
    };
    let digest = if reports && !digest {
        Some(String::new())
    } else if reports {
        match arrays_digest(&answer.weld.positions, &answer.faces, &answer.uvs, &answer.face_domain, &answer.face_owner, &answer.seams) {
            Ok(text) => Some(text),
            Err(reason) => return declined(py, reason),
        }
    } else {
        None
    };
    let mut face_starts: Vec<u32> = Vec::with_capacity(answer.faces.len() + 1);
    face_starts.push(0);
    for face in &answer.faces {
        face_starts.push(face_starts.last().unwrap() + face.len() as u32);
    }
    let weld = &answer.weld;
    let gaps = PyTuple::new(
        py,
        weld.gaps.iter().map(|points| PyTuple::new(py, points.iter().map(|point| point_tuple(py, point)).collect::<PyResult<Vec<_>>>()?)).collect::<PyResult<Vec<_>>>()?,
    )?;
    let numbers: Vec<Bound<'py, PyAny>> = vec![
        weld.groups.into_pyobject(py)?.into_any(),
        weld.merged.into_pyobject(py)?.into_any(),
        weld.mismatches.into_pyobject(py)?.into_any(),
        gaps.into_any(),
        weld.fallbacks.into_pyobject(py)?.into_any(),
        PyFloat::new(py, weld.factor).into_any(),
        PyTuple::new(py, weld.reasons.iter().copied())?.into_any(),
        weld.lifted.into_pyobject(py)?.into_any(),
        weld.welded.into_pyobject(py)?.into_any(),
        weld.alone.into_pyobject(py)?.into_any(),
        weld.wall_fallbacks.into_pyobject(py)?.into_any(),
        PyFloat::new(py, weld.wall_factor).into_any(),
        PyTuple::new(py, weld.wall_reasons.iter().copied())?.into_any(),
        PyTuple::new(py, weld.partners.iter().copied())?.into_any(),
    ];
    let found_reports = match (digest, answer.reports) {
        (Some(digest), Some((seam, plane, conflicts))) => (digest, seam, plane, conflicts).into_pyobject(py)?.into_any(),
        _ => py.None().into_bound(py),
    };
    let parts: Vec<Bound<'py, PyAny>> = vec![
        bytes_of(py, weld.positions.iter().flat_map(|point| point.iter().copied()).collect::<Vec<f64>>().into_iter(), f64::to_le_bytes)?.into_any(),
        bytes_of(py, face_starts.iter().copied(), u32::to_le_bytes)?.into_any(),
        bytes_of(py, answer.faces.iter().flat_map(|face| face.iter().map(|item| *item as u32)).collect::<Vec<u32>>().into_iter(), u32::to_le_bytes)?.into_any(),
        bytes_of(py, answer.uvs.iter().flat_map(|uv| uv.iter().copied()).collect::<Vec<f64>>().into_iter(), f64::to_le_bytes)?.into_any(),
        bytes_of(py, answer.face_domain.iter().copied(), i64::to_le_bytes)?.into_any(),
        bytes_of(py, answer.face_owner.iter().copied(), i64::to_le_bytes)?.into_any(),
        bytes_of(py, answer.seams.iter().flat_map(|(a, b)| [*a as u32, *b as u32]).collect::<Vec<u32>>().into_iter(), u32::to_le_bytes)?.into_any(),
        bytes_of(py, weld.index.iter().flat_map(|mapping| mapping.iter().map(|item| *item as u32)).collect::<Vec<u32>>().into_iter(), u32::to_le_bytes)?.into_any(),
        PyTuple::new(py, answer.vertex_counts.iter().copied())?.into_any(),
        PyTuple::new(py, answer.loop_counts.iter().copied())?.into_any(),
        PyTuple::new(py, tokens)?.into_any(),
        PyTuple::new(py, revisions)?.into_any(),
        PyTuple::new(py, numbers)?.into_any(),
        answer.folds.into_pyobject(py)?.into_any(),
        found_reports,
    ];
    Ok(PyTuple::new(py, parts)?.into_any())
}
