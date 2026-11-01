//! BATCH_READERS_NATIVE_V1: the worker's three readers of a materialized domain's `GeometryBatchV1`, in one pass over the batch:
//!
//! * `validation.validate_geometry_batch` as a PROOF: `True` is "the oracle finds no issue" (every check of the oracle holds), `False` is "not proven" and the
//!   host runs the oracle for the named issues (the port never writes an issue, so it never names one differently);
//! * `materialize.audit.audit_batch`: the fields of `BatchAuditV1` (the same binary64 operations in the same order: the sums are CPython 3.11 left folds);
//! * `materialize.structure.batch_structure`: the digest and the part digests of `StructureSignatureV1` (the same text, the same sha256).
//!
//! The answer is a tuple `(proved clean | None, audit fields | None, (digest, parts) | None)` or the text of a decline (a `str`): every input the oracle would raise on
//! (a missing key, a face with fewer than three corners, a chain name without `:`, no reference normal) and every input the port does not read like Python (a number that
//! is not a `float`, an id whose `value` is not a `str`, a text that is not UTF-8) is declined, and the host's Python answers it, as it did. A Python error met while
//! reading is a decline too. The oracle's sources are pinned by `tests/test_native_batch_checks.py`.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::fmt::Write as _;

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyFloat, PyList, PyString, PyTuple};
use sha2::{Digest, Sha256};

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

/// An id record as Python compares it: a frozen dataclass whose one field is `value` - equal when the class is the same and the text is the same.
type Id = (usize, String);

const SEPARATOR: &str = "\x1f";
const RECORD: &str = "\x1e";

fn float_of(value: &Bound<'_, PyAny>, what: &str) -> Read<f64> {
    match value.cast_exact::<PyFloat>() {
        Ok(number) => Ok(number.value()),
        Err(_) => decline(format!("{what} is not a float")),
    }
}

fn text(value: &Bound<'_, PyAny>, what: &str) -> Read<String> {
    match value.cast_exact::<PyString>() {
        Ok(found) => Ok(found.to_str()?.to_string()),
        Err(_) => decline(format!("{what} is not a str")),
    }
}

fn items<'py>(value: &Bound<'py, PyAny>) -> Read<Vec<Bound<'py, PyAny>>> {
    if let Ok(tuple) = value.cast_exact::<PyTuple>() {
        return Ok(tuple.iter().collect());
    }
    if let Ok(list) = value.cast_exact::<PyList>() {
        return Ok(list.iter().collect());
    }
    let mut found = Vec::new();
    for item in value.try_iter()? {
        found.push(item?);
    }
    Ok(found)
}

fn id_of(py: Python<'_>, record: &Bound<'_, PyAny>, what: &str) -> Read<Id> {
    let value = text(&record.getattr(pyo3::intern!(py, "value"))?, what)?;
    Ok((record.get_type().as_ptr() as usize, value))
}

fn attr_id(py: Python<'_>, owner: &Bound<'_, PyAny>, name: &Bound<'_, PyString>, what: &str) -> Read<Id> {
    id_of(py, &owner.getattr(name)?, what)
}

fn keys_of(py: Python<'_>, owner: &Bound<'_, PyAny>, what: &str) -> Read<Vec<Id>> {
    let mut found = Vec::new();
    for key in items(&owner.getattr(pyo3::intern!(py, "ordered_vert_keys"))?)? {
        found.push(id_of(py, &key, what)?);
    }
    Ok(found)
}

fn triple(value: &Bound<'_, PyAny>, what: &str) -> Read<[f64; 3]> {
    let row = items(value)?;
    if row.len() != 3 {
        return decline(format!("{what} is not three numbers"));
    }
    Ok([float_of(&row[0], what)?, float_of(&row[1], what)?, float_of(&row[2], what)?])
}

struct Uv {
    vert: Id,
    class: usize,
    u: f64,
    v: f64,
}

struct Face {
    id: Id,
    keys: Vec<Id>,
    uv: Vec<Uv>,
    region: Id,
    claim: Id,
    material: Id,
}

struct Station {
    id: Id,
    vert: Id,
    region: Id,
    claim: Id,
}

struct Region {
    id: Id,
    claim: Id,
    material: Id,
}

struct Batch {
    schema_ok: bool,
    digest_ok: bool,
    vertices: Vec<(Id, [f64; 3])>,
    faces: Vec<Face>,
    stations: Vec<Station>,
    regions: Vec<Region>,
    boundary: Vec<(String, Vec<Id>)>,
    interface: Vec<Vec<Id>>,
    diagnostics: Vec<String>,
}

fn read_batch(py: Python<'_>, batch: &Bound<'_, PyAny>, schema: &Bound<'_, PyAny>, digest_class: &Bound<'_, PyAny>, positions: bool, diagnostics: bool) -> Read<Batch> {
    let schema_ok = batch.getattr(pyo3::intern!(py, "schema_version"))?.eq(schema)?;
    // `batch.semantic_digest != SemanticDigestValue(batch.semantic_digest.value)` (`check_semantic_digest=False`): unequal only for a digest record of another class
    let declared = batch.getattr(pyo3::intern!(py, "semantic_digest"))?;
    let digest_ok = declared.get_type().is(digest_class) && declared.getattr(pyo3::intern!(py, "value"))?.cast_exact::<PyString>().is_ok();
    let mut vertices = Vec::new();
    for vertex in items(&batch.getattr(pyo3::intern!(py, "vertices"))?)? {
        let key = attr_id(py, &vertex, pyo3::intern!(py, "vert_key"), "a vertex key")?;
        let position = if positions {
            let point = vertex.getattr(pyo3::intern!(py, "position"))?;
            [
                float_of(&point.getattr(pyo3::intern!(py, "x"))?, "a vertex position")?,
                float_of(&point.getattr(pyo3::intern!(py, "y"))?, "a vertex position")?,
                float_of(&point.getattr(pyo3::intern!(py, "z"))?, "a vertex position")?,
            ]
        } else {
            [0.0; 3]
        };
        vertices.push((key, position));
    }
    let mut faces = Vec::new();
    for face in items(&batch.getattr(pyo3::intern!(py, "faces"))?)? {
        let mut uv = Vec::new();
        for fact in items(&face.getattr(pyo3::intern!(py, "uv_facts"))?)? {
            let point = fact.getattr(pyo3::intern!(py, "uv"))?;
            uv.push(Uv {
                vert: attr_id(py, &fact, pyo3::intern!(py, "vert_key"), "a UV fact key")?,
                class: point.get_type().as_ptr() as usize,
                u: float_of(&point.getattr(pyo3::intern!(py, "u"))?, "a UV")?,
                v: float_of(&point.getattr(pyo3::intern!(py, "v"))?, "a UV")?,
            });
        }
        faces.push(Face {
            id: attr_id(py, &face, pyo3::intern!(py, "face_id"), "a face id")?,
            keys: keys_of(py, &face, "a face vertex key")?,
            uv,
            region: attr_id(py, &face, pyo3::intern!(py, "semantic_region_id"), "a face region")?,
            claim: attr_id(py, &face, pyo3::intern!(py, "ownership_claim_id"), "a face claim")?,
            material: attr_id(py, &face, pyo3::intern!(py, "material_id"), "a face material")?,
        });
    }
    let mut stations = Vec::new();
    for fact in items(&batch.getattr(pyo3::intern!(py, "station_facts"))?)? {
        stations.push(Station {
            id: attr_id(py, &fact, pyo3::intern!(py, "station_fact_id"), "a station fact id")?,
            vert: attr_id(py, &fact, pyo3::intern!(py, "vert_key"), "a station fact key")?,
            region: attr_id(py, &fact, pyo3::intern!(py, "semantic_region_id"), "a station fact region")?,
            claim: attr_id(py, &fact, pyo3::intern!(py, "ownership_claim_id"), "a station fact claim")?,
        });
    }
    let mut regions = Vec::new();
    for region in items(&batch.getattr(pyo3::intern!(py, "semantic_regions"))?)? {
        regions.push(Region {
            id: attr_id(py, &region, pyo3::intern!(py, "semantic_region_id"), "a region id")?,
            claim: attr_id(py, &region, pyo3::intern!(py, "ownership_claim_id"), "a region claim")?,
            material: attr_id(py, &region, pyo3::intern!(py, "material_id"), "a region material")?,
        });
    }
    let mut boundary = Vec::new();
    for chain in items(&batch.getattr(pyo3::intern!(py, "boundary_chains"))?)? {
        let name = attr_id(py, &chain, pyo3::intern!(py, "semantic_boundary_id"), "a boundary chain name")?.1;
        boundary.push((name, keys_of(py, &chain, "a boundary chain key")?));
    }
    let mut interface = Vec::new();
    for chain in items(&batch.getattr(pyo3::intern!(py, "interface_chains"))?)? {
        interface.push(keys_of(py, &chain, "an interface chain key")?);
    }
    let mut notes = Vec::new();
    if diagnostics {
        for item in items(&batch.getattr(pyo3::intern!(py, "diagnostics"))?)? {
            notes.push(attr_id(py, &item, pyo3::intern!(py, "diagnostic_id"), "a diagnostic id")?.1);
        }
    }
    Ok(Batch { schema_ok, digest_ok, vertices, faces, stations, regions, boundary, interface, diagnostics: notes })
}

/// `validate_geometry_batch(batch, check_semantic_digest=False)` finds no issue: every check of the oracle holds.
fn proved_clean(batch: &Batch) -> bool {
    if !batch.schema_ok || !batch.digest_ok {
        return false;
    }
    let mut vertex_keys: HashSet<&Id, FxBuild> = HashSet::default();
    for (key, _) in &batch.vertices {
        if !vertex_keys.insert(key) {
            return false; // duplicate vert_key
        }
        // `vert_key.value.lower().startswith(("coord:", "xyz:", "rounded:"))`: a key that is not ASCII up front is left to the oracle (Unicode case mapping)
        let head = &key.1.as_bytes()[..key.1.len().min(8)];
        if !head.is_ascii() {
            return false;
        }
        let lowered = head.to_ascii_lowercase();
        if lowered.starts_with(b"coord:") || lowered.starts_with(b"xyz:") || lowered.starts_with(b"rounded:") {
            return false;
        }
    }
    let mut region_ids: HashSet<&Id, FxBuild> = HashSet::default();
    let mut region_by_id: HashMap<&Id, &Region, FxBuild> = HashMap::default();
    let mut claim_ids: HashSet<&Id, FxBuild> = HashSet::default();
    for region in &batch.regions {
        if !region_ids.insert(&region.id) {
            return false;
        }
        region_by_id.insert(&region.id, region);
        claim_ids.insert(&region.claim);
    }
    let mut fact_ids: HashSet<&Id, FxBuild> = HashSet::default();
    for fact in &batch.stations {
        if !fact_ids.insert(&fact.id) {
            return false;
        }
    }
    let mut face_ids: HashSet<&Id, FxBuild> = HashSet::default();
    let mut semantic_uv: HashMap<(&Id, &Id, &Id), (usize, f64, f64), FxBuild> = HashMap::default();
    for face in &batch.faces {
        if !face_ids.insert(&face.id) {
            return false;
        }
        if !face.keys.iter().all(|key| vertex_keys.contains(key)) || !region_ids.contains(&face.region) || !claim_ids.contains(&face.claim) {
            return false;
        }
        if let Some(region) = region_by_id.get(&face.region) {
            if region.claim != face.claim || region.material != face.material {
                return false;
            }
        }
        for fact in &face.uv {
            let value = (fact.class, fact.u, fact.v);
            match semantic_uv.get(&(&face.region, &face.claim, &fact.vert)) {
                // `previous != uv`: the same class and equal binary64 coordinates are equal in Python too
                Some(previous) => {
                    if previous.0 != value.0 || previous.1 != value.1 || previous.2 != value.2 {
                        return false;
                    }
                }
                None => {
                    semantic_uv.insert((&face.region, &face.claim, &fact.vert), value);
                }
            }
        }
    }
    let mut station_keys: HashSet<(&Id, &Id, &Id), FxBuild> = HashSet::default();
    let mut station_vertex_keys: HashSet<&Id, FxBuild> = HashSet::default();
    for fact in &batch.stations {
        station_vertex_keys.insert(&fact.vert);
        if !station_keys.insert((&fact.region, &fact.claim, &fact.vert)) {
            return false; // a duplicated (or conflicting) station key
        }
        if !vertex_keys.contains(&fact.vert) || !region_ids.contains(&fact.region) || !claim_ids.contains(&fact.claim) {
            return false;
        }
        if let Some(region) = region_by_id.get(&fact.region) {
            if region.claim != fact.claim {
                return false;
            }
        }
    }
    if station_vertex_keys != vertex_keys {
        return false;
    }
    if station_keys.len() != semantic_uv.len() || !semantic_uv.keys().all(|key| station_keys.contains(key)) {
        return false;
    }
    batch.boundary.iter().all(|(_, keys)| keys.iter().all(|key| vertex_keys.contains(key))) && batch.interface.iter().all(|keys| keys.iter().all(|key| vertex_keys.contains(key)))
}

fn location_key<'a>(key: &'a str, mark: &str) -> &'a str {
    key.strip_suffix(mark).unwrap_or(key)
}

fn cross(a: &[f64; 3], b: &[f64; 3], c: &[f64; 3]) -> [f64; 3] {
    let (ux, uy, uz) = (b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    let (vx, vy, vz) = (c[0] - a[0], c[1] - a[1], c[2] - a[2]);
    [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx]
}

fn degrees(radians: f64) -> f64 {
    // CPython: `radToDeg * x`, `radToDeg = 180.0 / Py_MATH_PI`
    (180.0_f64 / std::f64::consts::PI) * radians
}

/// `_normal_spread(normals)`: the largest angle between two of the unit normals, degrees.
fn normal_spread(normals: &[[f64; 3]]) -> f64 {
    let mut worst = 0.0_f64;
    for first in 0..normals.len() {
        for second in first + 1..normals.len() {
            let mut dot = 0.0_f64;
            for axis in 0..3 {
                dot += normals[first][axis] * normals[second][axis];
            }
            let low = if dot < 1.0 { dot } else { 1.0 };
            let clamped = if low > -1.0 { low } else { -1.0 };
            let angle = degrees(clamped.acos());
            if angle > worst {
                worst = angle;
            }
        }
    }
    worst
}

struct Audit {
    faces: usize,
    boundary_edges: usize,
    overshared: usize,
    duplicated: usize,
    mismatch: usize,
    v_min: f64,
    v_max: f64,
    flipped: usize,
    degenerate: usize,
    reversed: usize,
    spread: Option<f64>,
    seam_clip: usize,
}

fn audit(batch: &Batch, source: Option<[f64; 3]>, normals: &HashMap<String, [f64; 3], FxBuild>, mark: &str) -> Read<Audit> {
    let mut position: HashMap<&Id, &[f64; 3], FxBuild> = HashMap::default();
    for (key, point) in &batch.vertices {
        if position.insert(key, point).is_some() {
            return decline("a vertex key twice (the oracle keeps the last of a set's order)");
        }
    }
    let developed = !normals.is_empty();
    if !developed && source.is_none() {
        return decline("no source normal and no vertex normals (the oracle raises)");
    }
    let mut directed: HashMap<(&str, &str), usize, FxBuild> = HashMap::default();
    let (mut flipped, mut degenerate) = (0usize, 0usize);
    let mut signs: Vec<i64> = Vec::new();
    let (mut v_min, mut v_max) = (1.0_f64, 0.0_f64);
    let mut spread = 0.0_f64;
    for face in &batch.faces {
        let keys = &face.keys;
        if keys.len() < 3 || face.uv.len() < 3 {
            return decline("a face with fewer than three corners (the oracle raises)");
        }
        let mut face_normals = Vec::new();
        if developed {
            for key in keys {
                match normals.get(&key.1) {
                    Some(found) => face_normals.push(*found),
                    None => return decline("a face vertex without an offset normal (the oracle raises)"),
                }
            }
            if keys.len() >= 4 {
                let found = normal_spread(&face_normals);
                if found > spread {
                    spread = found;
                }
            }
        }
        for index in 0..keys.len() {
            let pair = (location_key(&keys[index].1, mark), location_key(&keys[(index + 1) % keys.len()].1, mark));
            *directed.entry(pair).or_insert(0) += 1;
        }
        let mut points = Vec::with_capacity(keys.len());
        for key in keys {
            match position.get(key) {
                Some(found) => points.push(*found),
                None => return decline("a face vertex is not a vertex (the oracle raises)"),
            }
        }
        let mut total = cross(points[0], points[1], points[2]);
        for index in 2..points.len() - 1 {
            let part = cross(points[0], points[index], points[index + 1]);
            total = [total[0] + part[0], total[1] + part[1], total[2] + part[2]];
        }
        let reference = match source {
            Some(normal) if !developed => normal,
            _ => {
                let mut sum = [0.0_f64; 3];
                for (axis, slot) in sum.iter_mut().enumerate() {
                    for normal in &face_normals {
                        *slot += normal[axis];
                    }
                }
                sum
            }
        };
        let facing = total[0] * reference[0] + total[1] * reference[1] + total[2] * reference[2];
        if facing <= 0.0 {
            flipped += 1;
        }
        let uv = &face.uv;
        let term = |first: &Uv, second: &Uv| (second.u - uv[0].u) * (first.v - uv[0].v) - (second.v - uv[0].v) * (first.u - uv[0].u);
        let mut area = term(&uv[2], &uv[1]);
        for index in 2..uv.len() - 1 {
            area += term(&uv[index + 1], &uv[index]);
        }
        if area == 0.0 {
            degenerate += 1;
        } else {
            signs.push(if area > 0.0 { 1 } else { -1 });
        }
        for item in uv {
            if item.v < v_min {
                v_min = item.v;
            }
            if item.v > v_max {
                v_max = item.v;
            }
        }
    }
    let mut undirected: HashMap<(&str, &str), usize, FxBuild> = HashMap::default();
    for ((first, second), count) in &directed {
        let edge = if first <= second { (*first, *second) } else { (*second, *first) };
        *undirected.entry(edge).or_insert(0) += count;
    }
    let boundary: HashSet<(&str, &str), FxBuild> = undirected.iter().filter(|(_, count)| **count == 1).map(|(edge, _)| *edge).collect();
    let mut chain_edges: HashSet<(&str, &str), FxBuild> = HashSet::default();
    let mut seam_clip = 0usize;
    for (name, keys) in &batch.boundary {
        for pair in keys.windows(2) {
            let (first, second) = (location_key(&pair[0].1, mark), location_key(&pair[1].1, mark));
            chain_edges.insert(if first <= second { (first, second) } else { (second, first) });
        }
        let Some(kind) = name.split(':').nth(1) else {
            return decline("a boundary chain name without ':' (the oracle raises)");
        };
        if kind == "SOURCE" || kind == "WALL" {
            seam_clip += keys.iter().filter(|key| key.1.starts_with("clip:")).count();
        }
    }
    let mismatch = boundary.symmetric_difference(&chain_edges).count();
    let majority = if signs.iter().sum::<i64>() >= 0 { 1 } else { -1 };
    let any = !batch.faces.is_empty();
    Ok(Audit {
        faces: batch.faces.len(),
        boundary_edges: boundary.len(),
        overshared: undirected.values().filter(|count| **count > 2).count(),
        duplicated: directed.values().filter(|count| **count > 1).count(),
        mismatch,
        v_min: if any { v_min } else { 0.0 },
        v_max: if any { v_max } else { 0.0 },
        flipped,
        degenerate,
        reversed: signs.iter().filter(|sign| **sign != majority).count(),
        spread: if developed { Some(spread) } else { None },
        seam_clip,
    })
}

fn part_digest<S: AsRef<str>>(records: &[S]) -> String {
    let mut hasher = Sha256::new();
    for (at, record) in records.iter().enumerate() {
        if at > 0 {
            hasher.update(RECORD.as_bytes());
        }
        hasher.update(record.as_ref().as_bytes());
    }
    let mut out = String::with_capacity(16);
    for byte in &hasher.finalize()[..8] {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

fn chain_records(chains: &[&Vec<Id>]) -> Vec<String> {
    let mut found: Vec<String> = chains
        .iter()
        .map(|keys| {
            let mut edges: Vec<(&str, &str)> = keys
                .windows(2)
                .map(|pair| {
                    let (a, b) = (pair[0].1.as_str(), pair[1].1.as_str());
                    if a <= b { (a, b) } else { (b, a) }
                })
                .collect();
            edges.sort();
            edges.iter().map(|(a, b)| format!("{a}{SEPARATOR}{b}")).collect::<Vec<_>>().join(RECORD)
        })
        .collect();
    found.sort();
    found
}

/// `batch_structure(batch)`: `(digest, ((part, digest), ...))`.
fn structure(batch: &Batch) -> Read<(String, Vec<(&'static str, String)>)> {
    let mut rings: Vec<(Vec<&str>, &str)> = Vec::with_capacity(batch.faces.len());
    for face in &batch.faces {
        let keys: Vec<&str> = face.keys.iter().map(|key| key.1.as_str()).collect();
        let Some(least) = keys.iter().min() else {
            return decline("a face without corners (the oracle raises)");
        };
        let start = keys.iter().position(|key| key == least).unwrap_or(0);
        let ring: Vec<&str> = keys[start..].iter().chain(&keys[..start]).copied().collect();
        rings.push((ring, face.region.1.as_str()));
    }
    let mut label_of: HashMap<&str, &Vec<&str>, FxBuild> = HashMap::default();
    for (ring, region) in &rings {
        match label_of.get(region) {
            Some(known) if !(ring < *known) => {}
            _ => {
                label_of.insert(region, ring);
            }
        }
    }
    let mut faces: Vec<(&Vec<&str>, &Vec<&str>)> = rings.iter().map(|(ring, region)| (ring, label_of[region])).collect();
    faces.sort();
    let face_records: Vec<String> = faces.iter().map(|(ring, label)| format!("{}@{}", ring.join(SEPARATOR), label.join(SEPARATOR))).collect();
    let mut vertices: Vec<&str> = batch.vertices.iter().map(|(key, _)| key.1.as_str()).collect();
    vertices.sort();
    let unknown = vec!["?"];
    let mut facts: Vec<String> = batch
        .stations
        .iter()
        .map(|fact| format!("{}@{}", fact.vert.1, label_of.get(fact.region.1.as_str()).copied().unwrap_or(&unknown).join(SEPARATOR)))
        .collect();
    facts.sort();
    let boundary = chain_records(&batch.boundary.iter().map(|(_, keys)| keys).collect::<Vec<_>>());
    let interface = chain_records(&batch.interface.iter().collect::<Vec<_>>());
    let labels: BTreeSet<&Vec<&str>> = label_of.values().copied().collect();
    let mut regions: Vec<String> = labels.iter().map(|label| label.join(SEPARATOR)).collect();
    regions.sort();
    let mut diagnostics: Vec<&str> = batch.diagnostics.iter().map(String::as_str).collect();
    diagnostics.sort();
    let parts = vec![
        ("vertices", part_digest(&vertices)),
        ("faces", part_digest(&face_records)),
        ("facts", part_digest(&facts)),
        ("boundary_chains", part_digest(&boundary)),
        ("interface_chains", part_digest(&interface)),
        ("regions", part_digest(&regions)),
        ("diagnostics", part_digest(&diagnostics)),
    ];
    let named: Vec<String> = parts.iter().map(|(name, digest)| format!("{name}={digest}")).collect();
    Ok((part_digest(&named), parts))
}

#[allow(clippy::too_many_arguments)]
fn run<'py>(
    py: Python<'py>,
    batch: &Bound<'py, PyAny>,
    schema: &Bound<'py, PyAny>,
    digest_class: &Bound<'py, PyAny>,
    source_normal: &Bound<'py, PyAny>,
    vertex_normals: &Bound<'py, PyAny>,
    mark: &str,
    want: (bool, bool, bool),
) -> Read<Bound<'py, PyTuple>> {
    let (want_validate, want_audit, want_structure) = want;
    let read = read_batch(py, batch, schema, digest_class, want_audit, want_structure)?;
    let clean = if want_validate { Some(proved_clean(&read)) } else { None };
    let audited = if want_audit {
        let source = if source_normal.is_none() { None } else { Some(triple(source_normal, "the source normal")?) };
        let mut normals: HashMap<String, [f64; 3], FxBuild> = HashMap::default();
        if !vertex_normals.is_none() {
            let Ok(dict) = vertex_normals.cast_exact::<PyDict>() else {
                return decline("the vertex normals are not a dict");
            };
            for (key, value) in dict.iter() {
                normals.insert(text(&key, "a vertex normal key")?, triple(&value, "a vertex normal")?);
            }
        }
        Some(audit(&read, source, &normals, mark)?)
    } else {
        None
    };
    let signature = if want_structure { Some(structure(&read)?) } else { None };
    let audit_value: Bound<'py, PyAny> = match audited {
        None => py.None().into_bound(py),
        Some(found) => (
            found.faces,
            found.boundary_edges,
            found.overshared,
            found.duplicated,
            found.mismatch,
            found.v_min,
            found.v_max,
            found.flipped,
            found.degenerate,
            found.reversed,
            found.spread,
            found.seam_clip,
        )
            .into_pyobject(py)?
            .into_any(),
    };
    let structure_value: Bound<'py, PyAny> = match signature {
        None => py.None().into_bound(py),
        Some((digest, parts)) => (digest, PyTuple::new(py, parts)?).into_pyobject(py)?.into_any(),
    };
    let clean_value: Bound<'py, PyAny> = match clean {
        None => py.None().into_bound(py),
        Some(found) => pyo3::types::PyBool::new(py, found).to_owned().into_any(),
    };
    Ok(PyTuple::new(py, [clean_value, audit_value, structure_value])?)
}

/// `(proved clean | None, audit fields | None, (digest, parts) | None)` of a materialized batch, or the text of a decline (a `str`).
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn batch_checks<'py>(
    py: Python<'py>,
    batch: &Bound<'py, PyAny>,
    schema: &Bound<'py, PyAny>,
    digest_class: &Bound<'py, PyAny>,
    source_normal: &Bound<'py, PyAny>,
    vertex_normals: &Bound<'py, PyAny>,
    copy_mark: &str,
    validate: bool,
    audit: bool,
    structure: bool,
) -> PyResult<Bound<'py, PyAny>> {
    if copy_mark.is_empty() {
        return Ok(PyString::new(py, "the copy mark of a cut vertex is empty").into_any());
    }
    match run(py, batch, schema, digest_class, source_normal, vertex_normals, copy_mark, (validate, audit, structure)) {
        Ok(found) => Ok(found.into_any()),
        Err(Fail::Decline(reason)) => Ok(PyString::new(py, &reason).into_any()),
        Err(Fail::Py(error)) => Ok(PyString::new(py, &format!("a Python error while reading: {error}")).into_any()),
    }
}
