//! `silhouette.dissolve_silhouette` and `silhouette.verify_silhouette` (the law `SILHOUETTE_TOPOLOGY_V1`), with the chains of the final faces
//! (`assemble.chains_of_faces`, `assemble.chains_of`, `assemble.paths_of`, `assemble.edge_kind`), bit for bit.
//!
//! Vertices are numbered (`Key`); `rank[k]` is the position of the name of `k` among all names sorted as Python sorts `str` (by code point,
//! which is the byte order of UTF-8), so every `sorted(...)` and every comparison of names of the oracle is a comparison of ranks. Places
//! (`audit.location_key`: the right copy of a cut vertex is the place of the left one) are numbered as well.
//!
//! The ORDER of the exact questions is the cost: the edges are merged in the oracle's sorted order, the vertices dissolved in its sorted order,
//! the face numbers of a vertex are visited in the iteration order of the CPython `set` that holds them (`pyset`), and the other vertices of the
//! domain in the insertion order of the oracle's `incident` dictionary (first appearance in the faces, in face order, minus the dissolved ones).

use std::collections::{HashMap, HashSet};

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::error::ClipError;
use cftuv_clip::faces::{orientation, Pt};
use cftuv_clip::lift::sqrt_sum_binary64;
use cftuv_core::exact::{self, ExactCtx, ExactError};
use cftuv_core::float_filter::{affine_map_violated, centre_and_bound};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};
use cftuv_mesh::affine::{affine_frame, uv_vertex_on_affine_map, Frame};
use cftuv_mesh::error::MeshError;
use cftuv_mesh::predicates::segments_cross;

use crate::pymath::{atan2, ceil_scaled, hypot, left_fold_sum, py_max, py_min, square, within};
use crate::pyset::PySmallIntSet;

pub type Key = u32;
type FxMap<K, V> = HashMap<K, V, FxBuild>;
type FxSet<K> = HashSet<K, FxBuild>;

pub const LAW: &str = "SILHOUETTE_TOPOLOGY_V1";
pub const EDGES_DISSOLVED: &str = "MATERIALIZE_SILHOUETTE_EDGES_DISSOLVED";
pub const VERTICES_DISSOLVED: &str = "MATERIALIZE_SILHOUETTE_VERTICES_DISSOLVED";
pub const KEPT_UV: &str = "MATERIALIZE_SILHOUETTE_KEPT_UV";
pub const KEPT_CHORD: &str = "MATERIALIZE_SILHOUETTE_KEPT_CHORD";
pub const KEPT_NOT_AFFINE: &str = "MATERIALIZE_SILHOUETTE_KEPT_NOT_AFFINE";
pub const KEPT_NOT_SIMPLE: &str = "MATERIALIZE_SILHOUETTE_KEPT_NOT_SIMPLE";
pub const EDGES_WITHIN_UV_TOLERANCE: &str = "MATERIALIZE_SILHOUETTE_EDGES_WITHIN_UV_TOLERANCE";
pub const MAX_UV_RESIDUAL_MILLI_ALPHA: &str = "MATERIALIZE_SILHOUETTE_MAX_UV_RESIDUAL_MILLI_ALPHA";
pub const MAX_UV_SLIDE_MILLI_ALPHA: &str = "MATERIALIZE_SILHOUETTE_MAX_UV_SLIDE_MILLI_ALPHA";
pub const MAX_CHORD_NM: &str = "MATERIALIZE_SILHOUETTE_MAX_CHORD_NM";
pub const SKIPPED_WORK_BUDGET: &str = "MATERIALIZE_SILHOUETTE_SKIPPED_WORK_BUDGET";
pub const SKIPPED_NOT_MANIFOLD: &str = "MATERIALIZE_SILHOUETTE_SKIPPED_NOT_MANIFOLD";
pub const PLAN_DISSOLVED: &str = "MATERIALIZE_STATION_PLAN_DISSOLVED";
pub const PLAN_KEPT_TWINNED: &str = "MATERIALIZE_STATION_PLAN_KEPT_TWINNED";
pub const PLAN_KEPT_ATTACHED: &str = "MATERIALIZE_STATION_PLAN_KEPT_ATTACHED";
pub const PLAN_KEPT_CHORD: &str = "MATERIALIZE_STATION_PLAN_KEPT_CHORD";
pub const PLAN_KEPT_NOT_SIMPLE: &str = "MATERIALIZE_STATION_PLAN_KEPT_NOT_SIMPLE";
pub const PLAN_KEPT_NO_UV_FACTS: &str = "MATERIALIZE_STATION_PLAN_KEPT_NO_UV_FACTS";
pub const PLAN_MAX_UV_SLIDE_MILLI_ALPHA: &str = "MATERIALIZE_STATION_PLAN_MAX_UV_SLIDE_MILLI_ALPHA";
pub const CONTINUOUS_INTERFACE_EDGES: &str = "MATERIALIZE_SILHOUETTE_CONTINUOUS_INTERFACE_EDGES";
pub const STATION_PLAN_VIOLATED: &str = "STATION_PLAN_VIOLATED";

pub const PLAN_KEPT_NAMES: [&str; 5] = [PLAN_KEPT_TWINNED, PLAN_KEPT_ATTACHED, PLAN_KEPT_CHORD, PLAN_KEPT_NOT_SIMPLE, PLAN_KEPT_NO_UV_FACTS];

/// `COUNTER_NAMES` in the oracle's order.
pub const COUNTER_NAMES: [&str; 20] = [
    EDGES_DISSOLVED,
    VERTICES_DISSOLVED,
    KEPT_UV,
    KEPT_CHORD,
    KEPT_NOT_AFFINE,
    KEPT_NOT_SIMPLE,
    EDGES_WITHIN_UV_TOLERANCE,
    MAX_UV_RESIDUAL_MILLI_ALPHA,
    MAX_UV_SLIDE_MILLI_ALPHA,
    MAX_CHORD_NM,
    SKIPPED_WORK_BUDGET,
    SKIPPED_NOT_MANIFOLD,
    PLAN_DISSOLVED,
    PLAN_KEPT_TWINNED,
    PLAN_KEPT_ATTACHED,
    PLAN_KEPT_CHORD,
    PLAN_KEPT_NOT_SIMPLE,
    PLAN_KEPT_NO_UV_FACTS,
    PLAN_MAX_UV_SLIDE_MILLI_ALPHA,
    CONTINUOUS_INTERFACE_EDGES,
];

const NANOMETRES_PER_METRE: u64 = 1_000_000_000;
const MILLI_ALPHA_PER_ALPHA: u64 = 1000;

/// What the pass refuses with.
#[derive(Debug, Clone)]
pub enum SilError {
    /// The budget of the pass ran out (`ExactCanonicalizationWorkBudgetExhausted`): the host keeps the mesh as it was, `SKIPPED_WORK_BUDGET`.
    Exhausted,
    /// `MaterializationRefusal(BATCH_DID_NOT_VALIDATE, "HALF_EDGE_SHARED_IN_ONE_DIRECTION: (place_a, place_b)")` of `assemble._own_half_edge`.
    HalfEdgeShared(u32, u32),
    /// The port does not carry this input; the host runs the oracle.
    Unsupported(String),
}

pub type SilResult<T> = Result<T, SilError>;

fn unsupported<T>(text: impl Into<String>) -> SilResult<T> {
    Err(SilError::Unsupported(text.into()))
}

impl From<ClipError> for SilError {
    fn from(error: ClipError) -> SilError {
        match error {
            ClipError::Exact(error) => SilError::from(error),
            other => SilError::Unsupported(format!("an exact predicate failed: {other:?}")),
        }
    }
}

impl From<ExactError> for SilError {
    fn from(error: ExactError) -> SilError {
        match error {
            ExactError::Canon(cftuv_canon::CanonError::Exhausted(_)) => SilError::Exhausted,
            other => SilError::Unsupported(format!("the exact layer refused: {other:?}")),
        }
    }
}

impl From<MeshError> for SilError {
    fn from(error: MeshError) -> SilError {
        match error {
            MeshError::Exact(error) => SilError::from(error),
            other => SilError::Unsupported(format!("a mesh predicate failed: {other:?}")),
        }
    }
}

/// Everything the pass reads (`SilhouetteInputV1` in native form).
pub struct SilInput {
    /// `rank[k]`: the place of the name of `k` among all names in Python's `str` order.
    pub rank: Vec<u32>,
    /// `place[k]`: the number of `location_key(name of k)`.
    pub place: Vec<u32>,
    /// `polygons[i]`: the rings of merged face `i`.
    pub polygons: Vec<Vec<Vec<Key>>>,
    /// `cycles[i]`: the contour keys of merged face `i` (`source.cycles[i][j][0]`).
    pub cycles: Vec<Vec<Key>>,
    /// `regions[i]`: `layout.region_of(frame_faces[i])`.
    pub regions: Vec<u32>,
    /// `layout.cut_pairs(frame_faces)` as ordered pairs `(low, high)`.
    pub cut_pairs: FxSet<(u32, u32)>,
    /// `positions[k]` (`None`: not a key of `positions`).
    pub positions: Vec<Option<[f64; 3]>>,
    /// `points[k]` (`None`: not a key of `points`).
    pub points: Vec<Option<(SqrtSum, SqrtSum)>>,
    /// `facts[(region, k)]`.
    pub facts: FxMap<(u32, Key), (SqrtSum, SqrtSum)>,
    /// `Fraction(1) / Fraction(lattice_alpha)` (the UV law's factor) and `SqrtSumV1.rational(lattice_alpha)` (the front of `edge_kind`).
    pub inverse_alpha: Rat,
    pub front: SqrtSum,
    pub uv_slide: Rat,
    /// `CHORD_BUDGET` (`clip_cells.CLIP_DIAGONAL_CHORD_BUDGET`, read live by the host).
    pub chord_budget: Rat,
    pub reverse: bool,
    /// `free[k]`: the name of `k` is in `source.free`; `free_any`: `source.free` is not empty.
    pub free: Vec<bool>,
    pub free_any: bool,
}

/// One final face.
#[derive(Debug, Clone)]
pub struct FinalFace {
    pub index: u32,
    pub ring: Vec<Key>,
    /// `merged[(index, ring)]`: the other merged faces it carries (empty: not merged).
    pub others: Vec<u32>,
}

/// A chain of the batch: its id text and its vertex keys.
pub type Chain = (String, Vec<Key>);

/// `SilhouetteV1` in native form plus the verdict of `verify_silhouette`.
#[derive(Debug)]
pub struct SilOutput {
    /// 0: the pass ran; 1: not manifold (`SKIPPED_NOT_MANIFOLD`, the mesh as it was, no chains); 2: the budget ran out (`SKIPPED_WORK_BUDGET`).
    pub kind: u8,
    pub faces: Vec<FinalFace>,
    pub gone: Vec<Key>,
    pub counters: Vec<(&'static str, u64)>,
    pub note: String,
    pub changed: bool,
    pub runs: Vec<(Key, Key, Vec<Key>)>,
    pub fits: Vec<(u32, Vec<Key>)>,
    pub chains: Option<(Vec<Chain>, Vec<Chain>)>,
    pub problems: Vec<&'static str>,
}

// --------------------------------------------------------------------------
// the mesh
// --------------------------------------------------------------------------

#[derive(Clone)]
enum Cert {
    Unknown,
    NotAffine,
    Map([Key; 3], std::rc::Rc<Frame>),
}

#[derive(Clone, Copy)]
enum PlaneState {
    Unknown,
    None,
    Some(([f64; 3], [f64; 3], f64)),
}

struct Face {
    ring: Vec<Key>,
    region: u32,
    frames: Vec<u32>,
    order: (u32, u32),
    cert: Cert,
    plane: PlaneState,
    repeats: bool,
}

fn has_repeats(ring: &[Key]) -> bool {
    let mut seen: FxSet<Key> = FxSet::default();
    !ring.iter().all(|key| seen.insert(*key))
}

fn pairs(ring: &[Key]) -> impl Iterator<Item = (Key, Key)> + '_ {
    let size = ring.len();
    (0..size).map(move |at| (ring[at], ring[(at + 1) % size]))
}

/// `_plane_of(points)`: `(centre, unit normal, twice the area)` (Newell from the centre), or `None`.
fn plane_of(points: &[[f64; 3]]) -> PlaneState {
    let count = points.len() as f64;
    let cx = left_fold_sum(points.iter().map(|point| point[0])) / count;
    let cy = left_fold_sum(points.iter().map(|point| point[1])) / count;
    let cz = left_fold_sum(points.iter().map(|point| point[2])) / count;
    let (mut nx, mut ny, mut nz) = (0.0f64, 0.0f64, 0.0f64);
    let size = points.len();
    for at in 0..size {
        let (first, second) = (points[at], points[(at + 1) % size]);
        let (ax, ay, az) = (first[0] - cx, first[1] - cy, first[2] - cz);
        let (bx, by, bz) = (second[0] - cx, second[1] - cy, second[2] - cz);
        nx += ay * bz - az * by;
        ny += az * bx - ax * bz;
        nz += ax * by - ay * bx;
    }
    let length = (nx * nx + ny * ny + nz * nz).sqrt();
    if !(length > 0.0) || !length.is_finite() {
        return PlaneState::None;
    }
    PlaneState::Some(([cx, cy, cz], [nx / length, ny / length, nz / length], length))
}

/// `big, small = (one, two) if self._twice_area(one) >= self._twice_area(two) else (two, one)`: a tie keeps the first face as the bigger one.
fn first_is_bigger(one: f64, two: f64) -> bool {
    one >= two
}

fn distance_to_plane(plane: &([f64; 3], [f64; 3], f64), point: &[f64; 3]) -> f64 {
    let ((cx, cy, cz), (nx, ny, nz)) = ((plane.0[0], plane.0[1], plane.0[2]), (plane.1[0], plane.1[1], plane.1[2]));
    ((point[0] - cx) * nx + (point[1] - cy) * ny + (point[2] - cz) * nz).abs()
}

/// `_along(first, second, point)`: `(parameter clamped to [0, 1], distance)`.
fn along(first: &[f64; 3], second: &[f64; 3], point: &[f64; 3]) -> (f64, f64) {
    let (dx, dy, dz) = (second[0] - first[0], second[1] - first[1], second[2] - first[2]);
    let length_squared = dx * dx + dy * dy + dz * dz;
    let (px, py, pz) = (point[0] - first[0], point[1] - first[1], point[2] - first[2]);
    let share = if length_squared == 0.0 { 0.0 } else { py_max(0.0, py_min(1.0, (px * dx + py * dy + pz * dz) / length_squared)) };
    let (ex, ey, ez) = (px - dx * share, py - dy * share, pz - dz * share);
    (share, (ex * ex + ey * ey + ez * ez).sqrt())
}

/// `uv_fit_residual(chart, uvs)`.
pub fn uv_fit_residual(chart: &[(f64, f64)], uvs: &[(f64, f64)]) -> Option<f64> {
    let count = chart.len() as f64;
    let mean_x = left_fold_sum(chart.iter().map(|point| point.0)) / count;
    let mean_y = left_fold_sum(chart.iter().map(|point| point.1)) / count;
    let mut scale: Option<f64> = None;
    for point in chart {
        let item = py_max((point.0 - mean_x).abs(), (point.1 - mean_y).abs());
        scale = Some(match scale {
            None => item,
            Some(found) => py_max(found, item),
        });
    }
    let scale = scale?;
    if !(scale > 0.0) {
        return None;
    }
    let xs: Vec<f64> = chart.iter().map(|point| (point.0 - mean_x) / scale).collect();
    let ys: Vec<f64> = chart.iter().map(|point| (point.1 - mean_y) / scale).collect();
    let sxx = left_fold_sum(xs.iter().map(|x| x * x));
    let sxy = left_fold_sum(xs.iter().zip(&ys).map(|(x, y)| x * y));
    let syy = left_fold_sum(ys.iter().map(|y| y * y));
    let determinant = sxx * syy - sxy * sxy;
    if !(determinant > 0.0) {
        return None;
    }
    let mut residuals: [Vec<f64>; 2] = [Vec::new(), Vec::new()];
    for component in 0..2 {
        let values: Vec<f64> = uvs.iter().map(|uv| if component == 0 { uv.0 } else { uv.1 }).collect();
        let centre = left_fold_sum(values.iter().copied()) / count;
        let shifted: Vec<f64> = values.iter().map(|value| value - centre).collect();
        let right_x = left_fold_sum(xs.iter().zip(&shifted).map(|(x, value)| x * value));
        let right_y = left_fold_sum(ys.iter().zip(&shifted).map(|(y, value)| y * value));
        let slope_x = (right_x * syy - right_y * sxy) / determinant;
        let slope_y = (sxx * right_y - sxy * right_x) / determinant;
        residuals[component] = shifted.iter().zip(xs.iter().zip(&ys)).map(|(value, (x, y))| value - (slope_x * x + slope_y * y)).collect();
    }
    let mut worst: Option<f64> = None;
    for (first, second) in residuals[0].iter().zip(&residuals[1]) {
        let item = hypot(*first, *second);
        worst = Some(match worst {
            None => item,
            Some(found) => py_max(found, item),
        });
    }
    let worst = worst?;
    if worst.is_finite() {
        Some(worst)
    } else {
        None
    }
}

/// The insertion-ordered `incident` dictionary of the oracle: `key -> set of face numbers`, with `pop`.
struct Incident {
    order: Vec<Key>,
    sets: FxMap<Key, PySmallIntSet>,
}

impl Incident {
    fn add(&mut self, key: Key, number: u32) {
        let set = self.sets.entry(key).or_insert_with(|| {
            self.order.push(key);
            PySmallIntSet::default()
        });
        set.add(u64::from(number));
    }

    fn get(&self, key: Key) -> Option<&PySmallIntSet> {
        self.sets.get(&key)
    }

    fn pop(&mut self, key: Key) -> Option<PySmallIntSet> {
        let found = self.sets.remove(&key)?;
        if let Some(at) = self.order.iter().position(|item| *item == key) {
            self.order.remove(at);
        }
        Some(found)
    }

    fn keys(&self) -> impl Iterator<Item = Key> + '_ {
        self.order.iter().copied()
    }
}

struct Tally {
    values: FxMap<&'static str, u64>,
}

impl Tally {
    fn new() -> Tally {
        Tally { values: FxMap::default() }
    }

    fn add(&mut self, name: &'static str, amount: u64) {
        *self.values.entry(name).or_insert(0) += amount;
    }

    fn get(&self, name: &str) -> u64 {
        self.values.get(name).copied().unwrap_or(0)
    }
}

struct Mesh<'s> {
    source: &'s SilInput,
    tally: Tally,
    max_chord: f64,
    max_slide: f64,
    max_residual: f64,
    fits: Vec<(u32, Vec<Key>)>,
    planned: FxSet<Key>,
    plan_slide: f64,
    faces: std::collections::BTreeMap<u32, Face>,
    half: FxMap<(Key, Key), u32>,
    manifold: bool,
    uv: FxMap<(u32, Key), (f64, f64)>,
    next_id: u32,
}

impl<'s> Mesh<'s> {
    /// `_Mesh(source, budget)` over the rings `polygons` (the source's, or those of a result).
    fn new(source: &'s SilInput, polygons: &[Vec<Vec<Key>>]) -> Mesh<'s> {
        let mut mesh = Mesh {
            source,
            tally: Tally::new(),
            max_chord: 0.0,
            max_slide: 0.0,
            max_residual: 0.0,
            fits: Vec::new(),
            planned: FxSet::default(),
            plan_slide: 0.0,
            faces: std::collections::BTreeMap::new(),
            half: FxMap::default(),
            manifold: true,
            uv: FxMap::default(),
            next_id: 0,
        };
        let mut number = 0u32;
        for (index, face_polygons) in polygons.iter().enumerate() {
            let region = source.regions[index];
            for (place, ring) in face_polygons.iter().enumerate() {
                let face = Face { ring: ring.clone(), region, frames: vec![index as u32], order: (index as u32, place as u32), cert: Cert::Unknown, plane: PlaneState::Unknown, repeats: has_repeats(ring) };
                for edge in pairs(&face.ring) {
                    mesh.manifold = mesh.manifold && !mesh.half.contains_key(&edge);
                    mesh.half.insert(edge, number);
                }
                mesh.faces.insert(number, face);
                number += 1;
            }
        }
        mesh.next_id = number;
        mesh
    }

    fn xyz(&self, key: Key) -> SilResult<[f64; 3]> {
        match self.source.positions.get(key as usize).copied().flatten() {
            Some(found) => Ok(found),
            None => unsupported("a vertex of the faces has no position"),
        }
    }

    fn point(&self, key: Key) -> SilResult<&'s (SqrtSum, SqrtSum)> {
        match self.source.points.get(key as usize).and_then(Option::as_ref) {
            Some(found) => Ok(found),
            None => unsupported("a vertex of the faces has no chart point"),
        }
    }

    fn fact(&self, region: u32, key: Key) -> SilResult<&'s (SqrtSum, SqrtSum)> {
        match self.source.facts.get(&(region, key)) {
            Some(found) => Ok(found),
            None => unsupported("a vertex of a face has no fact in its region"),
        }
    }

    fn uv(&mut self, region: u32, key: Key) -> SilResult<(f64, f64)> {
        if let Some(found) = self.uv.get(&(region, key)) {
            return Ok(*found);
        }
        let (s, r) = self.fact(region, key)?;
        let factor = &self.source.inverse_alpha;
        let u = sqrt_sum_binary64(&s.scaled(factor)).map_err(|_| SilError::Unsupported("the UV of a vertex overflows".to_string()))?;
        let v = sqrt_sum_binary64(&r.scaled(factor)).map_err(|_| SilError::Unsupported("the UV of a vertex overflows".to_string()))?;
        if !u.is_finite() || !v.is_finite() {
            return unsupported("the UV of a vertex is not finite");
        }
        self.uv.insert((region, key), (u, v));
        Ok((u, v))
    }

    /// `box(keys)`.
    fn bbox(&self, keys: &[Key]) -> SilResult<Option<(f64, f64, f64, f64)>> {
        let mut xs = Vec::with_capacity(keys.len());
        let mut ys = Vec::with_capacity(keys.len());
        for &key in keys {
            let point = self.point(key)?;
            let Some(x) = centre_and_bound(&point.0) else {
                return Ok(None);
            };
            let Some(y) = centre_and_bound(&point.1) else {
                return Ok(None);
            };
            xs.push(x);
            ys.push(y);
        }
        let low = |items: &[(f64, f64)]| items.iter().skip(1).fold(items[0].0 - items[0].1, |found, (c, e)| py_min(found, c - e));
        let high = |items: &[(f64, f64)]| items.iter().skip(1).fold(items[0].0 + items[0].1, |found, (c, e)| py_max(found, c + e));
        Ok(Some((low(&xs), high(&xs), low(&ys), high(&ys))))
    }

    fn plane(&mut self, number: u32) -> SilResult<PlaneState> {
        let state = self.faces[&number].plane;
        if !matches!(state, PlaneState::Unknown) {
            return Ok(state);
        }
        let ring = self.faces[&number].ring.clone();
        let mut points = Vec::with_capacity(ring.len());
        for key in ring {
            points.push(self.xyz(key)?);
        }
        let found = plane_of(&points);
        self.faces.get_mut(&number).expect("a live face").plane = found;
        Ok(found)
    }

    fn dihedral(&mut self, first: u32, second: u32) -> SilResult<f64> {
        let (one, two) = (self.plane(first)?, self.plane(second)?);
        let (PlaneState::Some(one), PlaneState::Some(two)) = (one, two) else {
            return Ok(f64::INFINITY);
        };
        let ([ax, ay, az], [bx, by, bz]) = (one.1, two.1);
        let squares = (square(ay * bz - az * by), square(az * bx - ax * bz), square(ax * by - ay * bx));
        let (Some(p), Some(q), Some(r)) = squares else {
            return unsupported("a dihedral product overflows");
        };
        let cross = (p + q + r).sqrt();
        Ok(atan2(cross, ax * bx + ay * by + az * bz))
    }

    fn pt(&self, key: Key) -> SilResult<Pt<'s>> {
        let point = self.point(key)?;
        Ok((&point.0, &point.1))
    }

    /// `certificate(face)`.
    fn certificate(&mut self, ctx: &mut ExactCtx<'_>, number: u32) -> SilResult<Cert> {
        if matches!(self.faces[&number].cert, Cert::Unknown) {
            let found = self.solve(ctx, number)?;
            self.faces.get_mut(&number).expect("a live face").cert = found;
        }
        Ok(self.faces[&number].cert.clone())
    }

    fn solve(&mut self, ctx: &mut ExactCtx<'_>, number: u32) -> SilResult<Cert> {
        let ring = self.faces[&number].ring.clone();
        let region = self.faces[&number].region;
        let mut base = None;
        'search: for second in 1..ring.len() {
            for third in second + 1..ring.len() {
                if orientation(ctx, self.pt(ring[0])?, self.pt(ring[second])?, self.pt(ring[third])?)? != 0 {
                    base = Some([ring[0], ring[second], ring[third]]);
                    break 'search;
                }
            }
        }
        let Some(base) = base else {
            return Ok(Cert::NotAffine);
        };
        let base_points = [self.pt(base[0])?, self.pt(base[1])?, self.pt(base[2])?];
        let frame = affine_frame(ctx, &base_points, [0, 1, 2]);
        for &key in &ring {
            if base.contains(&key) {
                continue;
            }
            if !self.on_map(ctx, region, base, &frame, key)? {
                return Ok(Cert::NotAffine);
            }
        }
        Ok(Cert::Map(base, std::rc::Rc::new(frame)))
    }

    /// `_on_map(points, values, base, frame, key)`.
    fn on_map(&self, ctx: &mut ExactCtx<'_>, region: u32, base: [Key; 3], frame: &Frame, key: Key) -> SilResult<bool> {
        let points = [self.pt(base[0])?, self.pt(base[1])?, self.pt(base[2])?, self.pt(key)?];
        let mut values_owned = Vec::with_capacity(4);
        for item in [base[0], base[1], base[2], key] {
            values_owned.push(self.fact(region, item)?);
        }
        let values = [(&values_owned[0].0, &values_owned[0].1), (&values_owned[1].0, &values_owned[1].1), (&values_owned[2].0, &values_owned[2].1), (&values_owned[3].0, &values_owned[3].1)];
        if affine_map_violated(&points, &values) {
            return Ok(false);
        }
        Ok(uv_vertex_on_affine_map(ctx, &points, &values, [0, 1, 2], frame, 3))
    }

    // -- the edges --------------------------------------------------------------------------------

    fn dissolve_edges(&mut self, ctx: &mut ExactCtx<'_>) -> SilResult<()> {
        let mut candidates: Vec<(f64, Key, Key)> = Vec::new();
        let halves: Vec<((Key, Key), u32)> = self.half.iter().map(|(edge, number)| (*edge, *number)).collect();
        for ((first, second), number) in halves {
            let Some(&other) = self.half.get(&(second, first)) else {
                continue;
            };
            if self.source.rank[first as usize] >= self.source.rank[second as usize] || other == number {
                continue;
            }
            if self.faces[&number].region == self.faces[&other].region {
                candidates.push((self.dihedral(number, other)?, first, second));
            }
        }
        let rank = &self.source.rank;
        candidates.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(rank[a.1 as usize].cmp(&rank[b.1 as usize]))
                .then(rank[a.2 as usize].cmp(&rank[b.2 as usize]))
        });
        if candidates.iter().any(|item| item.0.is_nan()) {
            return unsupported("a dihedral angle is NaN");
        }
        for (_angle, first, second) in candidates {
            self.try_merge(ctx, first, second)?;
        }
        Ok(())
    }

    fn try_merge(&mut self, ctx: &mut ExactCtx<'_>, first: Key, second: Key) -> SilResult<()> {
        let (Some(&one_id), Some(&two_id)) = (self.half.get(&(first, second)), self.half.get(&(second, first))) else {
            return Ok(());
        };
        if one_id == two_id {
            return Ok(());
        }
        let shared_is_edge = {
            let (one, two) = (&self.faces[&one_id], &self.faces[&two_id]);
            if one.repeats || two.repeats {
                false
            } else {
                let left: FxSet<Key> = one.ring.iter().copied().collect();
                let right: FxSet<Key> = two.ring.iter().copied().collect();
                let common: FxSet<Key> = left.intersection(&right).copied().collect();
                common.len() == 2 && common.contains(&first) && common.contains(&second)
            }
        };
        if !shared_is_edge {
            self.tally.add(KEPT_NOT_SIMPLE, 1);
            return Ok(());
        }
        let union = {
            let (one, two) = (&self.faces[&one_id], &self.faces[&two_id]);
            union_of(&one.ring, &two.ring, first, second)
        };
        let depth = self.merge_depth(one_id, two_id, &union)?;
        let Some(depth) = depth.filter(|value| within(*value, &self.source.chord_budget)) else {
            self.tally.add(KEPT_CHORD, 1);
            return Ok(());
        };
        let cert = self.affine_union(ctx, one_id, two_id)?;
        let region = self.faces[&one_id].region;
        if matches!(cert, Cert::NotAffine) {
            let Some(residual) = self.tolerated(region, &union)? else {
                self.tally.add(KEPT_NOT_AFFINE, 1);
                return Ok(());
            };
            self.tally.add(EDGES_WITHIN_UV_TOLERANCE, 1);
            self.max_residual = py_max(self.max_residual, residual);
            self.fits.push((region, union.clone()));
        }
        let (one, two) = (self.faces.remove(&one_id).expect("a live face"), self.faces.remove(&two_id).expect("a live face"));
        for face in [&one, &two] {
            for edge in pairs(&face.ring) {
                self.half.remove(&edge);
            }
        }
        let mut frames: Vec<u32> = one.frames.iter().chain(&two.frames).copied().collect();
        frames.sort_unstable();
        frames.dedup();
        let merged = Face { repeats: has_repeats(&union), ring: union, region: one.region, frames, order: one.order.min(two.order), cert, plane: PlaneState::Unknown };
        let id = self.next_id;
        for edge in pairs(&merged.ring) {
            self.half.insert(edge, id);
        }
        self.faces.insert(id, merged);
        self.next_id += 1;
        self.tally.add(EDGES_DISSOLVED, 1);
        self.max_chord = py_max(self.max_chord, depth);
        Ok(())
    }

    /// `_merge_depth(one, two, union)`.
    fn merge_depth(&mut self, one_id: u32, two_id: u32, union: &[Key]) -> SilResult<Option<f64>> {
        let (big, small) = if first_is_bigger(self.twice_area(one_id)?, self.twice_area(two_id)?) { (one_id, two_id) } else { (two_id, one_id) };
        let PlaneState::Some(plane) = self.plane(big)? else {
            return Ok(None);
        };
        let mut depth: Option<f64> = None;
        for key in self.faces[&small].ring.clone() {
            let item = distance_to_plane(&plane, &self.xyz(key)?);
            depth = Some(depth.map_or(item, |found| py_max(found, item)));
        }
        let Some(depth) = depth else {
            return unsupported("max() of an empty ring");
        };
        let mut points = Vec::with_capacity(union.len());
        for &key in union {
            points.push(self.xyz(key)?);
        }
        let PlaneState::Some(fit) = plane_of(&points) else {
            return Ok(None);
        };
        let mut away: Option<f64> = None;
        for point in &points {
            let item = distance_to_plane(&fit, point);
            away = Some(away.map_or(item, |found| py_max(found, item)));
        }
        Ok(Some(py_max(depth, away.expect("a union has vertices"))))
    }

    fn twice_area(&mut self, number: u32) -> SilResult<f64> {
        Ok(match self.plane(number)? {
            PlaneState::Some(plane) => plane.2,
            _ => 0.0,
        })
    }

    /// `_affine_union(one, two)`.
    fn affine_union(&mut self, ctx: &mut ExactCtx<'_>, one_id: u32, two_id: u32) -> SilResult<Cert> {
        let (mut one_id, mut two_id) = (one_id, two_id);
        {
            let (one, two) = (&self.faces[&one_id], &self.faces[&two_id]);
            if matches!(one.cert, Cert::Unknown) && (matches!(two.cert, Cert::Map(..)) || two.ring.len() > one.ring.len()) {
                std::mem::swap(&mut one_id, &mut two_id);
            }
        }
        let cert = self.certificate(ctx, one_id)?;
        let Cert::Map(base, frame) = &cert else {
            return Ok(Cert::NotAffine);
        };
        let region = self.faces[&one_id].region;
        let known: FxSet<Key> = self.faces[&one_id].ring.iter().copied().collect();
        for key in self.faces[&two_id].ring.clone() {
            if known.contains(&key) {
                continue;
            }
            if !self.on_map(ctx, region, *base, frame, key)? {
                return Ok(Cert::NotAffine);
            }
        }
        Ok(cert)
    }

    /// `_tolerated(region, union)`.
    fn tolerated(&mut self, region: u32, union: &[Key]) -> SilResult<Option<f64>> {
        if self.source.uv_slide.is_zero() {
            return Ok(None);
        }
        let mut chart = Vec::with_capacity(union.len());
        for &key in union {
            let point = self.point(key)?;
            let (Some(x), Some(y)) = (centre_and_bound(&point.0), centre_and_bound(&point.1)) else {
                return Ok(None);
            };
            chart.push((x.0, y.0));
        }
        let mut uvs = Vec::with_capacity(union.len());
        for &key in union {
            uvs.push(self.uv(region, key)?);
        }
        let residual = uv_fit_residual(&chart, &uvs);
        Ok(residual.filter(|value| within(*value, &self.source.uv_slide)))
    }

    // -- the vertices -----------------------------------------------------------------------------

    fn incident(&self) -> Incident {
        let mut incident = Incident { order: Vec::new(), sets: FxMap::default() };
        for (number, face) in &self.faces {
            for &key in &face.ring {
                incident.add(key, *number);
            }
        }
        incident
    }

    fn edge_kind(&self, region: u32, first: Key, second: Key) -> SilResult<u8> {
        edge_kind(self.source, region, first, second)
    }

    /// `_fixed_vertices(incident)`.
    fn fixed_vertices(&mut self, incident: &Incident) -> SilResult<FxSet<Key>> {
        let mut fixed: FxSet<Key> = FxSet::default();
        for (&(first, second), &number) in &self.half {
            if self.half.contains_key(&(second, first)) {
                continue;
            }
            let region = self.faces[&number].region;
            if self.edge_kind(region, first, second)? != RIM {
                fixed.insert(first);
                fixed.insert(second);
            }
        }
        let mut by_place: FxMap<u32, Vec<Key>> = FxMap::default();
        for key in incident.keys() {
            by_place.entry(self.source.place[key as usize]).or_default().push(key);
        }
        let twinned: FxSet<Key> = by_place.values().filter(|keys| keys.len() > 1).flatten().copied().collect();
        let interior = self.chain_interior(incident)?;
        self.planned = interior.difference(&twinned).copied().collect();
        self.tally.add(PLAN_KEPT_TWINNED, interior.intersection(&twinned).count() as u64);
        for key in &self.planned {
            fixed.remove(key);
        }
        fixed.extend(twinned);
        Ok(fixed)
    }

    /// `chain_interior(incident)`.
    fn chain_interior(&self, incident: &Incident) -> SilResult<FxSet<Key>> {
        if !self.source.free_any {
            return Ok(FxSet::default());
        }
        let mut into: FxMap<Key, Vec<u8>> = FxMap::default();
        let mut out: FxMap<Key, Vec<u8>> = FxMap::default();
        for (&(first, second), &number) in &self.half {
            if self.half.contains_key(&(second, first)) {
                continue;
            }
            let kind = self.edge_kind(self.faces[&number].region, first, second)?;
            out.entry(first).or_default().push(kind);
            into.entry(second).or_default().push(kind);
        }
        let mut found = FxSet::default();
        for key in incident.keys() {
            if !self.source.free[key as usize] {
                continue;
            }
            let (Some(entering), Some(leaving)) = (into.get(&key), out.get(&key)) else {
                continue;
            };
            if entering.len() == 1 && leaving.len() == 1 && entering == leaving && entering[0] != RIM {
                found.insert(key);
            }
        }
        Ok(found)
    }

    /// `_line(key, incident)`.
    fn line(&self, key: Key, incident: &Incident) -> Option<(Key, Key)> {
        let numbers = incident.get(key)?;
        if !(1..=2).contains(&numbers.len()) {
            return None;
        }
        let mut around = Vec::with_capacity(2);
        for number in numbers.sorted() {
            let face = &self.faces[&(number as u32)];
            if face.repeats {
                return None;
            }
            let at = face.ring.iter().position(|item| *item == key)?;
            let size = face.ring.len();
            around.push((face.ring[(at + size - 1) % size], face.ring[(at + 1) % size]));
        }
        let (before, after) = around[0];
        if around.len() == 2 && around[1] != (after, before) {
            return None;
        }
        Some((before, after))
    }

    fn dissolve_vertices(&mut self, ctx: &mut ExactCtx<'_>) -> SilResult<(Vec<Key>, FxMap<(Key, Key), Vec<Key>>)> {
        let mut incident = self.incident();
        let fixed = self.fixed_vertices(&incident)?;
        let mut covered: FxMap<(Key, Key), Vec<Key>> = FxMap::default();
        let mut order: Vec<(f64, Key)> = Vec::new();
        let rank = &self.source.rank;
        let mut sorted_keys: Vec<Key> = incident.keys().collect();
        sorted_keys.sort_by_key(|key| rank[*key as usize]);
        for key in sorted_keys {
            let found = if fixed.contains(&key) { None } else { self.line(key, &incident) };
            if let Some((before, after)) = found {
                order.push((along(&self.xyz(before)?, &self.xyz(after)?, &self.xyz(key)?).1, key));
            } else if self.planned.contains(&key) && !fixed.contains(&key) {
                self.tally.add(PLAN_KEPT_ATTACHED, 1);
            }
        }
        if order.iter().any(|item| item.0.is_nan()) {
            return unsupported("a sag is NaN");
        }
        order.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal).then(rank[a.1 as usize].cmp(&rank[b.1 as usize])));
        let mut gone: Vec<Key> = Vec::new();
        let mut gone_set: FxSet<Key> = FxSet::default();
        for (_sag, key) in order {
            if self.try_dissolve(ctx, key, &mut incident, &mut covered, &gone_set)? {
                gone.push(key);
                gone_set.insert(key);
            }
        }
        Ok((gone, covered))
    }

    fn try_dissolve(&mut self, ctx: &mut ExactCtx<'_>, key: Key, incident: &mut Incident, covered: &mut FxMap<(Key, Key), Vec<Key>>, gone: &FxSet<Key>) -> SilResult<bool> {
        let line = self.line(key, incident);
        let planned = self.planned.contains(&key);
        let Some((before, after)) = line else {
            if planned {
                self.tally.add(PLAN_KEPT_ATTACHED, 1);
            }
            return Ok(false);
        };
        let left = unordered(before, key);
        let right = unordered(key, after);
        let mut run: Vec<Key> = covered.get(&left).cloned().unwrap_or_default();
        run.extend(covered.get(&right).cloned().unwrap_or_default());
        run.push(key);
        let rank = &self.source.rank;
        let (low, high) = if rank[before as usize] <= rank[after as usize] { (before, after) } else { (after, before) };
        let (start, end) = (self.xyz(low)?, self.xyz(high)?);
        let mut worst: Option<f64> = None;
        for &item in &run {
            let value = along(&start, &end, &self.xyz(item)?).1;
            worst = Some(worst.map_or(value, |found| py_max(found, value)));
        }
        let worst = worst.expect("a run has its vertex");
        if !within(worst, &self.source.chord_budget) {
            self.tally.add(if planned { PLAN_KEPT_CHORD } else { KEPT_CHORD }, 1);
            return Ok(false);
        }
        let numbers: Vec<u32> = incident.get(key).expect("a line vertex is incident").iter().map(|n| n as u32).collect();
        let slide = self.slide(&numbers, low, high, &run)?;
        let Some(slide) = slide else {
            self.tally.add(if planned { PLAN_KEPT_NO_UV_FACTS } else { KEPT_UV }, 1);
            return Ok(false);
        };
        if !planned && !within(slide, &self.source.uv_slide) {
            self.tally.add(KEPT_UV, 1);
            return Ok(false);
        }
        if !self.removable(ctx, key, before, after, incident, &numbers, gone)? {
            self.tally.add(if planned { PLAN_KEPT_NOT_SIMPLE } else { KEPT_NOT_SIMPLE }, 1);
            return Ok(false);
        }
        let popped = incident.pop(key).expect("a line vertex is incident");
        for number in popped.iter() {
            let number = number as u32;
            let ring = self.faces[&number].ring.clone();
            for edge in pairs(&ring) {
                self.half.remove(&edge);
            }
            let kept: Vec<Key> = ring.into_iter().filter(|item| *item != key).collect();
            for edge in pairs(&kept) {
                self.half.insert(edge, number);
            }
            self.faces.get_mut(&number).expect("a live face").ring = kept;
        }
        covered.remove(&left);
        covered.remove(&right);
        covered.insert(unordered(before, after), run);
        self.max_chord = py_max(self.max_chord, worst);
        if planned {
            self.tally.add(PLAN_DISSOLVED, 1);
            self.plan_slide = py_max(self.plan_slide, slide);
        } else {
            self.tally.add(VERTICES_DISSOLVED, 1);
            self.max_slide = py_max(self.max_slide, slide);
        }
        Ok(true)
    }

    /// `_slide(numbers, before, after, run)`.
    fn slide(&mut self, numbers: &[u32], before: Key, after: Key, run: &[Key]) -> SilResult<Option<f64>> {
        let (start, end) = (self.xyz(before)?, self.xyz(after)?);
        let mut worst = 0.0f64;
        for &number in numbers {
            let region = self.faces[&number].region;
            if [before, after].iter().chain(run).any(|item| !self.source.facts.contains_key(&(region, *item))) {
                return Ok(None);
            }
            let ((u0, v0), (u1, v1)) = (self.uv(region, before)?, self.uv(region, after)?);
            for &item in run {
                let share = along(&start, &end, &self.xyz(item)?).0;
                let (u, v) = self.uv(region, item)?;
                worst = py_max(worst, hypot(u - (u0 + (u1 - u0) * share), v - (v0 + (v1 - v0) * share)));
            }
        }
        Ok(Some(worst))
    }

    /// `_removable(key, before, after, incident, gone)`.
    #[allow(clippy::too_many_arguments)]
    fn removable(&self, ctx: &mut ExactCtx<'_>, key: Key, before: Key, after: Key, incident: &Incident, numbers: &[u32], gone: &FxSet<Key>) -> SilResult<bool> {
        for &number in numbers {
            if !self.shortcut_is_simple(ctx, &self.faces[&number].ring, key)? {
                return Ok(false);
            }
        }
        let corner = [self.pt(before)?, self.pt(key)?, self.pt(after)?];
        if orientation(ctx, corner[0], corner[1], corner[2])? == 0 {
            return Ok(true);
        }
        let around = self.bbox(&[before, key, after])?;
        for other in incident.keys() {
            if other == before || other == key || other == after || incident.get(other).map_or(true, PySmallIntSet::is_empty) || gone.contains(&other) {
                continue;
            }
            let far = match around {
                None => None,
                Some(_) => self.bbox(&[other])?,
            };
            if let (Some(far), Some(around)) = (far, around) {
                if far.1 < around.0 || far.0 > around.1 || far.3 < around.2 || far.2 > around.3 {
                    continue;
                }
            }
            if self.in_corner(ctx, &corner, self.pt(other)?)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// `_shortcut_is_simple(ring, key)`.
    fn shortcut_is_simple(&self, ctx: &mut ExactCtx<'_>, ring: &[Key], key: Key) -> SilResult<bool> {
        let Some(at) = ring.iter().position(|item| *item == key) else {
            return unsupported("ring.index(key) of a vertex not in the ring");
        };
        let size = ring.len();
        let (before, after) = (ring[(at + size - 1) % size], ring[(at + 1) % size]);
        let rest: Vec<Key> = ring.iter().copied().filter(|item| *item != key).collect();
        if rest.len() < 3 {
            return Ok(false);
        }
        let (start, end) = (self.pt(before)?, self.pt(after)?);
        if rest.len() == 3 {
            return Ok(orientation(ctx, self.pt(rest[0])?, self.pt(rest[1])?, self.pt(rest[2])?)? != 0);
        }
        for (first, second) in pairs(&rest) {
            if before == first || before == second || after == first || after == second {
                continue;
            }
            if segments_cross(ctx, start, end, self.pt(first)?, self.pt(second)?)? {
                return Ok(false);
            }
        }
        for &item in &rest {
            if item == before || item == after {
                continue;
            }
            if on_open_segment(ctx, start, end, self.pt(item)?)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn in_corner(&self, ctx: &mut ExactCtx<'_>, corner: &[Pt<'s>; 3], point: Pt<'s>) -> SilResult<bool> {
        let mut signs = [0i8; 3];
        for at in 0..3 {
            signs[at] = orientation(ctx, corner[at], corner[(at + 1) % 3], point)?;
        }
        Ok(signs.iter().all(|sign| *sign >= 0) || signs.iter().all(|sign| *sign <= 0))
    }
}

fn unordered(a: Key, b: Key) -> (Key, Key) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

/// `_union(one, two, first, second)`.
fn union_of(one: &[Key], two: &[Key], first: Key, second: Key) -> Vec<Key> {
    let at = one.iter().position(|item| *item == second).expect("the shared edge is in the first ring");
    let mut start: Vec<Key> = one[at..].iter().chain(&one[..at]).copied().collect();
    let at = two.iter().position(|item| *item == first).expect("the shared edge is in the second ring");
    let rest: Vec<Key> = two[at..].iter().chain(&two[..at]).copied().collect();
    if rest.len() > 2 {
        start.extend_from_slice(&rest[1..rest.len() - 1]);
    }
    start
}

/// `_on_open_segment(first, second, other, budget)`.
fn on_open_segment(ctx: &mut ExactCtx<'_>, first: Pt, second: Pt, other: Pt) -> SilResult<bool> {
    if orientation(ctx, first, second, other)? != 0 {
        return Ok(false);
    }
    let along = second.0.sub(first.0).mul(&other.0.sub(first.0), ctx.products).add(&second.1.sub(first.1).mul(&other.1.sub(first.1), ctx.products));
    let along = exact::sign(ctx, &along, SIGN_FILTER_BITS)?;
    let beyond = first.0.sub(second.0).mul(&other.0.sub(second.0), ctx.products).add(&first.1.sub(second.1).mul(&other.1.sub(second.1), ctx.products));
    let beyond = exact::sign(ctx, &beyond, SIGN_FILTER_BITS)?;
    Ok(along > 0 && beyond > 0)
}

// --------------------------------------------------------------------------
// the chains (`assemble.py`)
// --------------------------------------------------------------------------

const RIM: u8 = 0;
const SOURCE: u8 = 1;
const WALL: u8 = 2;

fn kind_name(kind: u8) -> &'static str {
    match kind {
        RIM => "RIM",
        SOURCE => "SOURCE",
        _ => "WALL",
    }
}

/// Value equality of two `(s, r)` pairs (the dataclass `==` of `SqrtSumV1`: `int` and `Fraction` coefficients of one value are equal).
fn same_value(a: &SqrtSum, b: &SqrtSum) -> bool {
    let (left, right) = (a.terms(), b.terms());
    left.len() == right.len() && left.iter().zip(right).all(|(x, y)| x.radicand == y.radicand && x.coef.value() == y.coef.value())
}

fn same_fact(a: &(SqrtSum, SqrtSum), b: &(SqrtSum, SqrtSum)) -> bool {
    same_value(&a.0, &b.0) && same_value(&a.1, &b.1)
}

/// `edge_kind(facts, region, a, b, lattice_alpha)`.
fn edge_kind(source: &SilInput, region: u32, a: Key, b: Key) -> SilResult<u8> {
    edge_kind_in(&source.facts, &source.front, region, a, b)
}

fn edge_kind_in(facts: &FxMap<(u32, Key), (SqrtSum, SqrtSum)>, front: &SqrtSum, region: u32, a: Key, b: Key) -> SilResult<u8> {
    let (Some(fa), Some(fb)) = (facts.get(&(region, a)), facts.get(&(region, b))) else {
        return unsupported("edge_kind: a boundary vertex has no fact");
    };
    let (ra, rb) = (&fa.1, &fb.1);
    if ra.is_zero() && rb.is_zero() {
        return Ok(SOURCE);
    }
    if ra.sub(front).is_zero() && rb.sub(front).is_zero() {
        return Ok(RIM);
    }
    Ok(WALL)
}

/// `paths_of(edges)`: the maximal non-branching paths over the half-edges, by name order.
fn paths_of(edges: &[(Key, Key)], rank: &[u32]) -> Vec<Vec<Key>> {
    let key_of = |edge: &(Key, Key)| (rank[edge.0 as usize], rank[edge.1 as usize]);
    let mut sorted_edges: Vec<(Key, Key)> = edges.to_vec();
    sorted_edges.sort_by_key(key_of);
    let mut successors: FxMap<Key, Vec<Key>> = FxMap::default();
    let mut starts: Vec<Key> = Vec::new();
    let mut in_degree: FxMap<Key, usize> = FxMap::default();
    for &(a, b) in &sorted_edges {
        let entry = successors.entry(a).or_insert_with(|| {
            starts.push(a);
            Vec::new()
        });
        entry.push(b);
        *in_degree.entry(b).or_insert(0) += 1;
    }
    let interior = |node: Key, successors: &FxMap<Key, Vec<Key>>| in_degree.get(&node).copied().unwrap_or(0) == 1 && successors.get(&node).map_or(0, Vec::len) == 1;
    let mut seen: FxSet<(Key, Key)> = FxSet::default();
    let mut paths: Vec<Vec<Key>> = Vec::new();
    let walk = |start: Key, first: Key, seen: &mut FxSet<(Key, Key)>| {
        let mut path = vec![start, first];
        seen.insert((start, first));
        let mut node = first;
        while interior(node, &successors) && node != start {
            let following = successors[&node][0];
            seen.insert((node, following));
            path.push(following);
            node = following;
        }
        path
    };
    let mut ordered_starts = starts.clone();
    ordered_starts.sort_by_key(|key| rank[*key as usize]);
    for start in ordered_starts {
        if interior(start, &successors) {
            continue;
        }
        for &first in &successors[&start] {
            paths.push(walk(start, first, &mut seen));
        }
    }
    for &(a, b) in &sorted_edges {
        if !seen.contains(&(a, b)) {
            paths.push(walk(a, b, &mut seen));
        }
    }
    paths
}

/// `(owner_of, insertion order)`: `_own_half_edge` over the half-edges `(index, first, second)` given in the oracle's order.
fn own_half_edges(source: &SilInput, halves: impl Iterator<Item = (u32, Key, Key)>) -> SilResult<(FxMap<(u32, u32), (u32, Key, Key)>, Vec<(u32, u32)>)> {
    let mut owner_of: FxMap<(u32, u32), (u32, Key, Key)> = FxMap::default();
    let mut order = Vec::new();
    for (index, first, second) in halves {
        let half = (source.place[first as usize], source.place[second as usize]);
        if owner_of.contains_key(&half) {
            return Err(SilError::HalfEdgeShared(half.0, half.1));
        }
        owner_of.insert(half, (index, first, second));
        order.push(half);
    }
    Ok((owner_of, order))
}

/// `_chains_from(owner_of, frame_faces, layout, facts, lattice_alpha, continuous)`: `(boundary chains, interface chains, continuous edges)`;
/// `continuous` `None` is the call without the counter (`chains_of`).
fn chains_from(
    source: &SilInput,
    facts: &FxMap<(u32, Key), (SqrtSum, SqrtSum)>,
    owner_of: &FxMap<(u32, u32), (u32, Key, Key)>,
    order: &[(u32, u32)],
    count_continuous: bool,
) -> SilResult<(Vec<(u8, u32, Vec<(Key, Key)>)>, Vec<((u32, u32), Vec<(Key, Key)>)>, u64)> {
    let mut boundary: FxMap<(u8, u32), Vec<(Key, Key)>> = FxMap::default();
    let mut interface: FxMap<(u32, u32), Vec<(Key, Key)>> = FxMap::default();
    let mut continuous = 0u64;
    for half in order {
        let (index, a, b) = owner_of[half];
        let region = source.regions[index as usize];
        let Some(&(other, other_start, other_end)) = owner_of.get(&(half.1, half.0)) else {
            let kind = edge_kind_in(facts, &source.front, region, a, b)?;
            boundary.entry((kind, region)).or_default().push((a, b));
            continue;
        };
        let other_region = source.regions[other as usize];
        if (region, index) < (other_region, other) && (other_region != region || (a, b) != (other_end, other_start)) {
            let pair = unordered(region, other_region);
            let cut = source.cut_pairs.contains(&pair);
            if other_region != region && (count_continuous || cut) {
                let mut equal = true;
                for (key, twin) in [(a, other_end), (b, other_start)] {
                    let (Some(left), Some(right)) = (facts.get(&(region, key)), facts.get(&(other_region, twin))) else {
                        return unsupported("an interface vertex has no fact");
                    };
                    if !same_fact(left, right) {
                        equal = false;
                        break;
                    }
                }
                if equal {
                    if count_continuous && !cut {
                        continuous += 1;
                    }
                    continue;
                }
            }
            interface.entry((region, other_region)).or_default().push((a, b));
        }
    }
    let mut boundary: Vec<(u8, u32, Vec<(Key, Key)>)> = boundary.into_iter().map(|((kind, region), edges)| (kind, region, edges)).collect();
    boundary.sort_by(|a, b| kind_name(a.0).cmp(kind_name(b.0)).then(a.1.cmp(&b.1)));
    let mut interface: Vec<((u32, u32), Vec<(Key, Key)>)> = interface.into_iter().collect();
    interface.sort_by_key(|item| item.0);
    Ok((boundary, interface, continuous))
}

/// The chains as `(id text, keys)`, in the oracle's order.
fn chain_records(source: &SilInput, boundary: &[(u8, u32, Vec<(Key, Key)>)], interface: &[((u32, u32), Vec<(Key, Key)>)]) -> (Vec<Chain>, Vec<Chain>) {
    let mut boundary_chains = Vec::new();
    for (kind, region, edges) in boundary {
        for (number, path) in paths_of(edges, &source.rank).into_iter().enumerate() {
            boundary_chains.push((format!("boundary:{}:{}:{}", kind_name(*kind), region, number), path));
        }
    }
    let mut interface_chains = Vec::new();
    for ((left, right), edges) in interface {
        for (number, path) in paths_of(edges, &source.rank).into_iter().enumerate() {
            interface_chains.push((format!("interface:{left}:{right}:{number}"), path));
        }
    }
    (boundary_chains, interface_chains)
}

/// `chains_of_faces(frame_faces, polygons, layout, facts, lattice_alpha, continuous, reverse)`: `(boundary, interface, continuous edges)`.
fn chains_of_faces(source: &SilInput, polygons: &[Vec<Vec<Key>>], facts: &FxMap<(u32, Key), (SqrtSum, SqrtSum)>) -> SilResult<(Vec<Chain>, Vec<Chain>, u64)> {
    let mut halves = Vec::new();
    for (index, face_polygons) in polygons.iter().enumerate() {
        for ring in face_polygons {
            for (first, second) in pairs(ring) {
                halves.push(if source.reverse { (index as u32, second, first) } else { (index as u32, first, second) });
            }
        }
    }
    let (owner_of, order) = own_half_edges(source, halves.into_iter())?;
    let (boundary, interface, continuous) = chains_from(source, facts, &owner_of, &order, true)?;
    let (boundary, interface) = chain_records(source, &boundary, &interface);
    Ok((boundary, interface, continuous))
}

/// The keys of the SOURCE and WALL boundary chains of `chains_of(frame_faces, cycles, ...)` (what `verify_silhouette` reads of it).
fn walls_of_cycles(source: &SilInput) -> SilResult<FxSet<Key>> {
    let mut halves = Vec::new();
    for (index, cycle) in source.cycles.iter().enumerate() {
        let size = cycle.len();
        for position in 0..size {
            halves.push((index as u32, cycle[position], cycle[(position + 1) % size]));
        }
    }
    let (owner_of, order) = own_half_edges(source, halves.into_iter())?;
    let (boundary, _interface, _continuous) = chains_from(source, &source.facts, &owner_of, &order, false)?;
    let mut walls = FxSet::default();
    for (kind, _region, edges) in &boundary {
        if *kind == SOURCE || *kind == WALL {
            for (a, b) in edges {
                walls.insert(*a);
                walls.insert(*b);
            }
        }
    }
    Ok(walls)
}

// --------------------------------------------------------------------------
// the pass, the result and the verification
// --------------------------------------------------------------------------

fn counters_of(tally: &Tally, maxima: Option<[u64; 4]>) -> Vec<(&'static str, u64)> {
    let mut found = Vec::new();
    for name in COUNTER_NAMES.iter() {
        let value = match (maxima, *name) {
            (Some(m), MAX_CHORD_NM) => m[0],
            (Some(m), MAX_UV_SLIDE_MILLI_ALPHA) => m[1],
            (Some(m), MAX_UV_RESIDUAL_MILLI_ALPHA) => m[2],
            (Some(m), PLAN_MAX_UV_SLIDE_MILLI_ALPHA) => m[3],
            _ => tally.get(name),
        };
        if value != 0 {
            found.push((*name, value));
        }
    }
    found
}

fn nanometres(metres: f64) -> SilResult<u64> {
    ceil_scaled(metres, NANOMETRES_PER_METRE).ok_or_else(|| SilError::Unsupported("a chord depth beyond the counter".to_string()))
}

fn milli_alpha(slide: f64) -> SilResult<u64> {
    ceil_scaled(slide, MILLI_ALPHA_PER_ALPHA).ok_or_else(|| SilError::Unsupported("a UV slide beyond the counter".to_string()))
}

/// `dissolve_silhouette(source, budget)` and `verify_silhouette(source, result)`. `ctx` carries the budget of the pass (the host makes the inner
/// budget of the oracle): an exhaustion answers `kind` 2 with the mesh as it was.
pub fn apply_silhouette(ctx: &mut ExactCtx<'_>, source: &SilInput) -> SilResult<SilOutput> {
    let mut mesh = Mesh::new(source, &source.polygons);
    if !mesh.manifold {
        let mut output = unchanged(source, vec![(SKIPPED_NOT_MANIFOLD, 1)], false, 1)?;
        output.problems = verify(source, &output)?;
        return Ok(output);
    }
    let faces_before = mesh.faces.len();
    let passed = (|| -> SilResult<(Vec<Key>, FxMap<(Key, Key), Vec<Key>>)> {
        mesh.dissolve_edges(ctx)?;
        mesh.dissolve_vertices(ctx)
    })();
    let (gone, covered) = match passed {
        Ok(found) => found,
        Err(SilError::Exhausted) => {
            let mut output = unchanged(source, vec![(SKIPPED_WORK_BUDGET, 1)], true, 2)?;
            output.problems = verify(source, &output)?;
            return Ok(output);
        }
        Err(other) => return Err(other),
    };
    let mut output = result(&mut mesh, gone, covered, faces_before)?;
    output.problems = verify(source, &output)?;
    Ok(output)
}

fn unchanged(source: &SilInput, mut counters: Vec<(&'static str, u64)>, manifold: bool, kind: u8) -> SilResult<SilOutput> {
    let mut chains = None;
    if manifold {
        let (boundary, interface, continuous) = chains_of_faces(source, &source.polygons, &source.facts)?;
        if continuous != 0 {
            counters.push((CONTINUOUS_INTERFACE_EDGES, continuous));
        }
        chains = Some((boundary, interface));
    }
    let mut faces = Vec::new();
    for (index, face_polygons) in source.polygons.iter().enumerate() {
        for ring in face_polygons {
            faces.push(FinalFace { index: index as u32, ring: ring.clone(), others: Vec::new() });
        }
    }
    Ok(SilOutput { kind, faces, gone: Vec::new(), counters, note: String::new(), changed: false, runs: Vec::new(), fits: Vec::new(), chains, problems: Vec::new() })
}

fn result(mesh: &mut Mesh<'_>, gone: Vec<Key>, covered: FxMap<(Key, Key), Vec<Key>>, faces_before: usize) -> SilResult<SilOutput> {
    let source = mesh.source;
    let mut numbers: Vec<u32> = mesh.faces.keys().copied().collect();
    numbers.sort_by_key(|number| mesh.faces[number].order);
    let mut faces = Vec::with_capacity(numbers.len());
    let mut polygons: Vec<Vec<Vec<Key>>> = vec![Vec::new(); source.polygons.len()];
    for number in numbers {
        let face = &mesh.faces[&number];
        let index = face.order.0;
        polygons[index as usize].push(face.ring.clone());
        let others: Vec<u32> = if face.frames.len() > 1 { face.frames.iter().copied().filter(|item| *item != index).collect() } else { Vec::new() };
        faces.push(FinalFace { index, ring: face.ring.clone(), others });
    }
    let gone_set: FxSet<Key> = gone.iter().copied().collect();
    let facts: FxMap<(u32, Key), (SqrtSum, SqrtSum)> = source.facts.iter().filter(|((_, key), _)| !gone_set.contains(key)).map(|(slot, value)| (*slot, value.clone())).collect();
    let (boundary, interface, continuous) = chains_of_faces(source, &polygons, &facts)?;
    mesh.tally.values.insert(CONTINUOUS_INTERFACE_EDGES, continuous);
    let tally = &mesh.tally;
    let changed = tally.get(EDGES_DISSOLVED) > 0 || tally.get(VERTICES_DISSOLVED) > 0 || tally.get(PLAN_DISSOLVED) > 0;
    let maxima = if changed {
        Some([nanometres(mesh.max_chord)?, milli_alpha(mesh.max_slide)?, milli_alpha(mesh.max_residual)?, milli_alpha(mesh.plan_slide)?])
    } else {
        None
    };
    let counters = counters_of(tally, maxima);
    let plan_kept: u64 = PLAN_KEPT_NAMES.iter().map(|name| tally.get(name)).sum();
    let plan = if tally.get(PLAN_DISSOLVED) > 0 || plan_kept > 0 { format!("plan_dissolved={} plan_kept={} ", tally.get(PLAN_DISSOLVED), plan_kept) } else { String::new() };
    let note = if !changed {
        String::new()
    } else {
        format!(
            "{LAW}: faces {} -> {} edges_dissolved={} vertices_dissolved={} {}kept_uv={} kept_chord={} kept_not_affine={} kept_not_simple={} max_chord_nm={} max_uv_slide_milli_alpha={} within_uv_tolerance={} max_uv_residual_milli_alpha={}",
            faces_before,
            mesh.faces.len(),
            tally.get(EDGES_DISSOLVED),
            tally.get(VERTICES_DISSOLVED),
            plan,
            tally.get(KEPT_UV),
            tally.get(KEPT_CHORD),
            tally.get(KEPT_NOT_AFFINE),
            tally.get(KEPT_NOT_SIMPLE),
            nanometres(mesh.max_chord)?,
            milli_alpha(mesh.max_slide)?,
            tally.get(EDGES_WITHIN_UV_TOLERANCE),
            milli_alpha(mesh.max_residual)?,
        )
    };
    let rank = &source.rank;
    let mut runs: Vec<(Key, Key, Vec<Key>)> = covered
        .into_iter()
        .map(|((a, b), run)| if rank[a as usize] <= rank[b as usize] { (a, b, run) } else { (b, a, run) })
        .collect();
    runs.sort_by(|x, y| {
        rank[x.0 as usize]
            .cmp(&rank[y.0 as usize])
            .then(rank[x.1 as usize].cmp(&rank[y.1 as usize]))
            .then_with(|| x.2.iter().map(|k| rank[*k as usize]).cmp(y.2.iter().map(|k| rank[*k as usize])))
    });
    Ok(SilOutput {
        kind: 0,
        faces,
        gone,
        counters,
        note,
        changed,
        runs,
        fits: std::mem::take(&mut mesh.fits),
        chains: Some((boundary, interface)),
        problems: Vec::new(),
    })
}

fn result_polygons(source: &SilInput, output: &SilOutput) -> Vec<Vec<Vec<Key>>> {
    let mut polygons: Vec<Vec<Vec<Key>>> = vec![Vec::new(); source.polygons.len()];
    for face in &output.faces {
        polygons[face.index as usize].push(face.ring.clone());
    }
    polygons
}

fn counter(output: &SilOutput, name: &str) -> u64 {
    output.counters.iter().find(|(found, _)| *found == name).map_or(0, |(_, value)| *value)
}

/// `verify_silhouette(source, result)`: the names of the violations (empty: the law held).
fn verify(source: &SilInput, output: &SilOutput) -> SilResult<Vec<&'static str>> {
    let polygons = result_polygons(source, output);
    let gone: FxSet<Key> = output.gone.iter().copied().collect();
    let facts: FxMap<(u32, Key), (SqrtSum, SqrtSum)> = if gone.is_empty() {
        source.facts.clone()
    } else {
        source.facts.iter().filter(|((_, key), _)| !gone.contains(key)).map(|(slot, value)| (*slot, value.clone())).collect()
    };
    let mut problems = Vec::new();
    if !output.changed {
        problems.extend(verify_plan(source, output, &polygons, &facts)?);
        problems.extend(verify_chains(source, output, &polygons, &facts)?);
        return Ok(problems);
    }
    let walls = walls_of_cycles(source)?;
    if walls.iter().any(|key| gone.contains(key) && !source.free[*key as usize]) {
        problems.push("SOURCE_OR_WALL_VERTEX_DISSOLVED");
    }
    problems.extend(verify_runs(source, output, &polygons)?);
    problems.extend(verify_fits(source, output)?);
    problems.extend(verify_topology(source, output, &polygons, &gone));
    problems.extend(verify_plan(source, output, &polygons, &facts)?);
    problems.extend(verify_chains(source, output, &polygons, &facts)?);
    Ok(problems)
}

fn verify_plan(source: &SilInput, output: &SilOutput, polygons: &[Vec<Vec<Key>>], facts: &FxMap<(u32, Key), (SqrtSum, SqrtSum)>) -> SilResult<Vec<&'static str>> {
    if !source.free_any || counter(output, SKIPPED_WORK_BUDGET) != 0 || counter(output, SKIPPED_NOT_MANIFOLD) != 0 {
        return Ok(Vec::new());
    }
    let final_mesh = Mesh::new(source, polygons);
    let incident = final_mesh.incident();
    // `chain_interior` of the final mesh reads the RESULT's facts
    let mut into: FxMap<Key, Vec<u8>> = FxMap::default();
    let mut out: FxMap<Key, Vec<u8>> = FxMap::default();
    for (&(first, second), &number) in &final_mesh.half {
        if final_mesh.half.contains_key(&(second, first)) {
            continue;
        }
        let kind = edge_kind_in(facts, &source.front, final_mesh.faces[&number].region, first, second)?;
        out.entry(first).or_default().push(kind);
        into.entry(second).or_default().push(kind);
    }
    let mut interior = 0usize;
    let gone: FxSet<Key> = output.gone.iter().copied().collect();
    let mut overlap = false;
    for key in incident.keys() {
        if !source.free[key as usize] {
            continue;
        }
        let (Some(entering), Some(leaving)) = (into.get(&key), out.get(&key)) else {
            continue;
        };
        if entering.len() == 1 && leaving.len() == 1 && entering == leaving && entering[0] != RIM {
            interior += 1;
            overlap |= gone.contains(&key);
        }
    }
    let named: u64 = PLAN_KEPT_NAMES.iter().map(|name| counter(output, name)).sum();
    if interior as u64 != named || overlap {
        return Ok(vec![STATION_PLAN_VIOLATED]);
    }
    Ok(Vec::new())
}

fn verify_runs(source: &SilInput, output: &SilOutput, polygons: &[Vec<Vec<Key>>]) -> SilResult<Vec<&'static str>> {
    let mut mesh = Mesh::new(source, &source.polygons);
    let mut regions: FxMap<(Key, Key), Vec<u32>> = FxMap::default();
    for (index, face_polygons) in polygons.iter().enumerate() {
        let region = source.regions[index];
        for ring in face_polygons {
            for (a, b) in pairs(ring) {
                let entry = regions.entry(unordered(a, b)).or_default();
                if !entry.contains(&region) {
                    entry.push(region);
                }
            }
        }
    }
    let mut found = Vec::new();
    let mut covered: Vec<Key> = Vec::new();
    let (mut worst_chord, mut worst_slide, mut plan_slide) = (0.0f64, 0.0f64, 0.0f64);
    for (before, after, run) in &output.runs {
        covered.extend(run);
        let (start, end) = (mesh.xyz(*before)?, mesh.xyz(*after)?);
        let planned = run.iter().all(|item| source.free[*item as usize]);
        let edge = unordered(*before, *after);
        for region in regions.get(&edge).cloned().unwrap_or_default() {
            if [*before, *after].iter().chain(run).any(|item| !source.facts.contains_key(&(region, *item))) {
                found.push("RUN_VERTEX_HAS_NO_FACT");
                continue;
            }
            let ((u0, v0), (u1, v1)) = (mesh.uv(region, *before)?, mesh.uv(region, *after)?);
            for &item in run {
                let share = along(&start, &end, &mesh.xyz(item)?).0;
                let (u, v) = mesh.uv(region, item)?;
                let slide = hypot(u - (u0 + (u1 - u0) * share), v - (v0 + (v1 - v0) * share));
                if planned {
                    plan_slide = py_max(plan_slide, slide);
                } else {
                    worst_slide = py_max(worst_slide, slide);
                }
            }
        }
        if !regions.contains_key(&edge) {
            found.push("RUN_EDGE_IS_NOT_IN_THE_RESULT");
        }
        for &item in run {
            worst_chord = py_max(worst_chord, along(&start, &end, &mesh.xyz(item)?).1);
        }
    }
    let mut left = covered.clone();
    left.sort_unstable();
    let mut right = output.gone.clone();
    right.sort_unstable();
    if left != right {
        found.push("RUNS_DO_NOT_COVER_THE_DISSOLVED_VERTICES");
    }
    if nanometres(worst_chord)? > counter(output, MAX_CHORD_NM) || !within(worst_chord, &source.chord_budget) {
        found.push("CHORD_DEPTH_BEYOND_RECORDED_MAXIMUM");
    }
    if milli_alpha(worst_slide)? > counter(output, MAX_UV_SLIDE_MILLI_ALPHA) || !within(worst_slide, &source.uv_slide) {
        found.push("UV_SLIDE_BEYOND_RECORDED_MAXIMUM");
    }
    if milli_alpha(plan_slide)? > counter(output, PLAN_MAX_UV_SLIDE_MILLI_ALPHA) {
        found.push("PLAN_UV_SLIDE_BEYOND_RECORDED_MAXIMUM");
    }
    Ok(found)
}

fn verify_fits(source: &SilInput, output: &SilOutput) -> SilResult<Vec<&'static str>> {
    if output.fits.is_empty() {
        return Ok(Vec::new());
    }
    let mut mesh = Mesh::new(source, &source.polygons);
    let mut worst = 0.0f64;
    for (region, ring) in &output.fits {
        let mut chart = Vec::with_capacity(ring.len());
        for &key in ring {
            let point = mesh.point(key)?;
            let (Some(x), Some(y)) = (centre_and_bound(&point.0), centre_and_bound(&point.1)) else {
                return Ok(vec!["UV_FIT_UNVERIFIABLE"]);
            };
            chart.push((x.0, y.0));
        }
        let mut uvs = Vec::with_capacity(ring.len());
        for &key in ring {
            uvs.push(mesh.uv(*region, key)?);
        }
        let Some(residual) = uv_fit_residual(&chart, &uvs) else {
            return Ok(vec!["UV_FIT_HAS_NO_AFFINE_MAP"]);
        };
        worst = py_max(worst, residual);
    }
    if milli_alpha(worst)? > counter(output, MAX_UV_RESIDUAL_MILLI_ALPHA) || !within(worst, &source.uv_slide) {
        return Ok(vec!["UV_RESIDUAL_BEYOND_RECORDED_MAXIMUM"]);
    }
    Ok(Vec::new())
}

/// `_directed(polygons)`: the directed edges by places, counted.
fn directed(source: &SilInput, polygons: &[Vec<Vec<Key>>]) -> FxMap<(u32, u32), u64> {
    let mut found: FxMap<(u32, u32), u64> = FxMap::default();
    for face_polygons in polygons {
        for ring in face_polygons {
            for (a, b) in pairs(ring) {
                *found.entry((source.place[a as usize], source.place[b as usize])).or_insert(0) += 1;
            }
        }
    }
    found
}

/// `_outline(directed)`: the undirected place edges whose reverse is absent.
fn outline(directed: &FxMap<(u32, u32), u64>) -> FxSet<(u32, u32)> {
    directed.keys().filter(|edge| !directed.contains_key(&(edge.1, edge.0))).map(|edge| unordered(edge.0, edge.1)).collect()
}

fn verify_topology(source: &SilInput, output: &SilOutput, polygons: &[Vec<Vec<Key>>], gone: &FxSet<Key>) -> Vec<&'static str> {
    let mut found = Vec::new();
    let live = |key: Key| source.positions[key as usize].is_some() && !gone.contains(&key);
    for face_polygons in polygons {
        for ring in face_polygons {
            if ring.len() < 3 || has_repeats(ring) || ring.iter().any(|key| !live(*key)) {
                found.push("FACE_IS_DEGENERATE_OR_USES_A_DISSOLVED_VERTEX");
                break;
            }
        }
    }
    let (after, before) = (directed(source, polygons), directed(source, &source.polygons));
    if after.values().any(|count| *count > 1) {
        found.push("HALF_EDGE_DUPLICATED");
    }
    let (outline_before, outline_after) = (outline(&before), outline(&after));
    let places: FxSet<u32> = outline_before.iter().flat_map(|edge| [edge.0, edge.1]).collect();
    let dissolved: FxSet<u32> = output.gone.iter().map(|key| source.place[*key as usize]).collect();
    let on_outline = places.intersection(&dissolved).count();
    if outline_after.len() + on_outline != outline_before.len() || outline_before.len() < on_outline {
        found.push("OUTLINE_DOES_NOT_CONTRACT_BY_DISSOLVED_VERTICES");
    }
    found
}

fn chain_edges(source: &SilInput, chains: &[Chain]) -> FxSet<(u32, u32)> {
    let mut found = FxSet::default();
    for (_id, keys) in chains {
        for at in 0..keys.len().saturating_sub(1) {
            found.insert(unordered(source.place[keys[at] as usize], source.place[keys[at + 1] as usize]));
        }
    }
    found
}

fn verify_chains(source: &SilInput, output: &SilOutput, polygons: &[Vec<Vec<Key>>], facts: &FxMap<(u32, Key), (SqrtSum, SqrtSum)>) -> SilResult<Vec<&'static str>> {
    let Some((boundary, interface)) = &output.chains else {
        return Ok(Vec::new());
    };
    let mut found = Vec::new();
    if outline(&directed(source, polygons)) != chain_edges(source, boundary) {
        found.push("OUTLINE_DOES_NOT_MATCH_THE_BOUNDARY_CHAINS");
    }
    let mut owner: FxMap<(u32, u32), (u32, u32, Key, Key)> = FxMap::default();
    for (index, face_polygons) in polygons.iter().enumerate() {
        let region = source.regions[index];
        for ring in face_polygons {
            for (a, b) in pairs(ring) {
                owner.insert((source.place[a as usize], source.place[b as usize]), (region, index as u32, a, b));
            }
        }
    }
    let mut expected: FxSet<(u32, u32)> = FxSet::default();
    let mut continuous: FxSet<(u32, u32)> = FxSet::default();
    for (&(place_a, place_b), &(region, index, a, b)) in &owner {
        let Some(&(other_region, other_index, other_a, other_b)) = owner.get(&(place_b, place_a)) else {
            continue;
        };
        let edge = unordered(place_a, place_b);
        if region == other_region {
            if index != other_index && (a, b) != (other_b, other_a) {
                expected.insert(edge);
            }
            continue;
        }
        let mut equal = true;
        for (key, twin) in [(a, other_b), (b, other_a)] {
            let (Some(left), Some(right)) = (facts.get(&(region, key)), facts.get(&(other_region, twin))) else {
                return unsupported("an edge between regions has a vertex without a fact");
            };
            if !same_fact(left, right) {
                equal = false;
                break;
            }
        }
        if equal {
            if !source.cut_pairs.contains(&unordered(region, other_region)) {
                continuous.insert(edge);
            }
        } else {
            expected.insert(edge);
        }
    }
    if expected != chain_edges(source, interface) {
        found.push("INTERFACE_DOES_NOT_MATCH_THE_UV_DISCONTINUITIES");
    }
    if continuous.len() as u64 != counter(output, CONTINUOUS_INTERFACE_EDGES) {
        found.push("CONTINUOUS_INTERFACE_EDGES_DO_NOT_MATCH_THE_RECORDED_NUMBER");
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::first_is_bigger;

    #[test]
    fn a_tie_of_areas_keeps_the_first_face_as_the_bigger_one() {
        assert!(first_is_bigger(0.5, 0.5));
        assert!(first_is_bigger(0.0, 0.0));
        assert!(!first_is_bigger(0.25, 0.5));
        assert!(first_is_bigger(0.5, 0.25));
    }
}
