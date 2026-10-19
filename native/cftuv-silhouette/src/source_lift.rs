//! `source_lift.lift_source_vertices` (the law `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1`) and `source_lift.settle_emitted_faces`, bit for bit.
//!
//! The first is pure binary64 and `Fraction` arithmetic (squared distances compared exactly); the second asks `tessellate.triangulate_exact`
//! (exact, priced) for a face of four or more vertices with a moved vertex, in the oracle's face order.

use cftuv_clip::faces::Pt;
use cftuv_clip::tessellate::triangulate_exact;
use cftuv_core::exact::ExactCtx;
use cftuv_core::pyfloat::rat_to_f64;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::pymath::{py_max, rat_of_float};

pub type P3 = [f64; 3];

fn eq3(a: &P3, b: &P3) -> bool {
    a[0] == b[0] && a[1] == b[1] && a[2] == b[2]
}

/// `_area_vector(points)`: the fan from the first point.
pub fn area_vector(points: &[P3]) -> P3 {
    let first = points[0];
    let mut total = [0.0f64; 3];
    if points.len() >= 3 {
        for index in 1..points.len() - 1 {
            let (second, third) = (points[index], points[index + 1]);
            let (ux, uy, uz) = (second[0] - first[0], second[1] - first[1], second[2] - first[2]);
            let (vx, vy, vz) = (third[0] - first[0], third[1] - first[1], third[2] - first[2]);
            total = [total[0] + (uy * vz - uz * vy), total[1] + (uz * vx - ux * vz), total[2] + (ux * vy - uy * vx)];
        }
    }
    total
}

pub fn dot(left: &P3, right: &P3) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

/// `_distance_squared(first, second)`: exact.
fn distance_squared(first: &P3, second: &P3) -> Option<Rat> {
    let mut total = Rat::zero();
    for axis in 0..3 {
        let difference = rat_of_float(first[axis])?.sub(&rat_of_float(second[axis])?);
        total = total.add(&difference.mul(&difference));
    }
    Some(total)
}

/// `off_plane_distance(before, after)`.
pub fn off_plane_distance(before: &[P3], after: &[P3]) -> f64 {
    let normal = area_vector(before);
    let length = dot(&normal, &normal).sqrt();
    if length == 0.0 {
        return 0.0;
    }
    let origin = before[0];
    let mut worst = 0.0f64;
    for (old, new) in before.iter().zip(after) {
        if eq3(old, new) {
            continue;
        }
        let away = [new[0] - origin[0], new[1] - origin[1], new[2] - origin[2]];
        worst = py_max(worst, dot(&away, &normal).abs() / length);
    }
    worst
}

/// The input of `lift_source_vertices`: the vertices in the order of `positions`, their names' kinds and the host positions of the `src:` ones.
pub struct LiftInput {
    pub positions: Vec<P3>,
    /// `key.startswith("src:")`.
    pub source: Vec<bool>,
    /// `key.startswith("node:")`.
    pub node: Vec<bool>,
    /// The host position of a `src:` vertex (`isinstance(host, LocalPoint3V1)`), else `None`.
    pub host: Vec<Option<P3>>,
    /// The canonical triangles (any contour) by vertex index.
    pub triangles: Vec<Vec<usize>>,
    /// `Fraction(step)`, or `None`.
    pub step: Option<Rat>,
}

/// Where the final position of a vertex comes from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Placed {
    Lift,
    Host,
    Followed(P3),
}

/// `SourceLiftV1` in native form.
#[derive(Debug)]
pub struct LiftOutput {
    pub placed: Vec<Placed>,
    pub lifted: usize,
    pub moved: usize,
    pub displaced: usize,
    pub unavailable: usize,
    pub kept: usize,
    pub total: usize,
    pub worst: Option<(usize, f64)>,
    pub max_lifted: f64,
    pub budget: Option<f64>,
    pub followed: usize,
    pub followed_drivers: usize,
    pub max_follow: f64,
}

/// A small insertion-ordered map `vertex -> value` over vertex indices.
struct Ordered<T: Copy> {
    order: Vec<usize>,
    value: Vec<Option<T>>,
}

impl<T: Copy> Ordered<T> {
    fn new(size: usize) -> Ordered<T> {
        Ordered { order: Vec::new(), value: vec![None; size] }
    }

    fn get(&self, key: usize) -> Option<T> {
        self.value[key]
    }

    fn contains(&self, key: usize) -> bool {
        self.value[key].is_some()
    }

    fn insert(&mut self, key: usize, value: T) {
        if self.value[key].is_none() {
            self.order.push(key);
        }
        self.value[key] = Some(value);
    }

    fn remove(&mut self, key: usize) {
        if self.value[key].take().is_some() {
            self.order.retain(|item| *item != key);
        }
    }

    fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    fn items(&self) -> Vec<(usize, T)> {
        self.order.iter().map(|key| (*key, self.value[*key].expect("an ordered key has a value"))).collect()
    }
}

/// `None`: a followed position is not finite (`LocalPoint3V1` would raise) or a coordinate is not finite.
pub fn lift_source_vertices(input: &LiftInput) -> Option<LiftOutput> {
    let count = input.positions.len();
    let budget = input.step.clone();
    let budget_squared = budget.as_ref().map(|value| value.mul(value));
    let mut allowed: Ordered<P3> = Ordered::new(count);
    let (mut displaced, mut unavailable) = (0usize, 0usize);
    let mut worst: Option<(usize, Rat)> = None;
    let names: Vec<usize> = (0..count).filter(|key| input.source[*key]).collect();
    for &key in &names {
        let (Some(limit), Some(host)) = (budget_squared.as_ref(), input.host[key]) else {
            unavailable += 1;
            continue;
        };
        let squared = distance_squared(&input.positions[key], &host)?;
        if squared <= *limit {
            allowed.insert(key, host);
        } else {
            displaced += 1;
            if worst.as_ref().map_or(true, |(_, found)| squared > *found) {
                worst = Some((key, squared));
            }
        }
    }
    let before: Vec<P3> = input.triangles.iter().map(|keys| area_vector(&keys.iter().map(|key| input.positions[*key]).collect::<Vec<_>>())).collect();
    let (kept, followers) = settle_orientation(input, &before, &mut allowed)?;
    let followed = followed_positions(input, &allowed, &followers)?;
    let mut placed = vec![Placed::Lift; count];
    for (key, _host) in allowed.items() {
        placed[key] = Placed::Host;
    }
    for (key, position) in &followed {
        placed[*key] = Placed::Followed(*position);
    }
    let changed: Vec<usize> = allowed.items().into_iter().filter(|(key, host)| !eq3(&input.positions[*key], host)).map(|(key, _)| key).collect();
    let mut biggest = Rat::zero();
    let mut first = true;
    for &key in &changed {
        let value = distance_squared(&input.positions[key], &allowed.get(key).expect("changed is allowed"))?;
        if first || value > biggest {
            biggest = value;
            first = false;
        }
    }
    let mut carried: Vec<usize> = Vec::new();
    for (_key, driver) in followers.items() {
        if changed.contains(&driver) && !carried.contains(&driver) {
            carried.push(driver);
        }
    }
    let mut follow_biggest = Rat::zero();
    let mut first = true;
    for &driver in &carried {
        let value = distance_squared(&input.positions[driver], &allowed.get(driver).expect("a carried driver is allowed"))?;
        if first || value > follow_biggest {
            follow_biggest = value;
            first = false;
        }
    }
    let sqrt_of = |value: &Rat| rat_to_f64(value).ok().map(f64::sqrt);
    Some(LiftOutput {
        placed,
        lifted: allowed.order.len(),
        moved: changed.len(),
        displaced,
        unavailable,
        kept,
        total: names.len(),
        worst: match worst {
            None => None,
            Some((key, squared)) => Some((key, sqrt_of(&squared)?)),
        },
        max_lifted: sqrt_of(&biggest)?,
        budget: match budget {
            None => None,
            Some(value) => Some(rat_to_f64(&value).ok()?),
        },
        followed: followers.order.len(),
        followed_drivers: carried.len(),
        max_follow: sqrt_of(&follow_biggest)?,
    })
}

fn followed_positions(input: &LiftInput, allowed: &Ordered<P3>, followers: &Ordered<usize>) -> Option<Vec<(usize, P3)>> {
    let mut found = Vec::with_capacity(followers.order.len());
    for (key, driver) in followers.items() {
        let (here, there) = (input.positions[key], input.positions[driver]);
        let moved = allowed.get(driver)?;
        let position = [here[0] + (moved[0] - there[0]), here[1] + (moved[1] - there[1]), here[2] + (moved[2] - there[2])];
        if !position.iter().all(|value| value.is_finite()) {
            return None;
        }
        found.push((key, position));
    }
    Some(found)
}

/// `_settle_orientation`: `(reverted, followers)`; `allowed` changes in place.
fn settle_orientation(input: &LiftInput, before: &[P3], allowed: &mut Ordered<P3>) -> Option<(usize, Ordered<usize>)> {
    let count = input.positions.len();
    let mut followers: Ordered<usize> = Ordered::new(count);
    let mut kept = 0usize;
    while !allowed.is_empty() {
        let mut proposed = input.positions.clone();
        for (key, host) in allowed.items() {
            proposed[key] = host;
        }
        for (key, position) in followed_positions(input, allowed, &followers)? {
            proposed[key] = position;
        }
        // `_flipped_contours(contours, before, proposed, {*allowed, *followers})`
        let mut flipped = Vec::new();
        for (index, keys) in input.triangles.iter().enumerate() {
            if !keys.iter().any(|key| allowed.contains(*key) || followers.contains(*key)) {
                continue;
            }
            let after = area_vector(&keys.iter().map(|key| proposed[*key]).collect::<Vec<_>>());
            if !(dot(&before[index], &after) > 0.0) {
                flipped.push(index);
            }
        }
        if flipped.is_empty() {
            break;
        }
        let mut reverted: Vec<usize> = Vec::new();
        for index in flipped {
            let keys = &input.triangles[index];
            let mut drivers: Vec<usize> = Vec::new();
            for &key in keys {
                if let Some(host) = allowed.get(key) {
                    if !eq3(&host, &input.positions[key]) && !drivers.contains(&key) {
                        drivers.push(key);
                    }
                }
            }
            for &key in keys {
                if let Some(driver) = followers.get(key) {
                    if !drivers.contains(&driver) {
                        drivers.push(driver);
                    }
                }
            }
            let free: Vec<usize> = keys.iter().copied().filter(|key| input.node[*key] && !followers.contains(*key)).collect();
            if drivers.len() == 1 && !free.is_empty() {
                let driver = drivers[0];
                for key in free {
                    followers.insert(key, driver);
                }
            } else {
                let chosen: Vec<usize> = if !drivers.is_empty() { drivers } else { keys.iter().copied().filter(|key| allowed.contains(*key)).collect() };
                for key in chosen {
                    if !reverted.contains(&key) {
                        reverted.push(key);
                    }
                }
            }
        }
        for &key in &reverted {
            allowed.remove(key);
        }
        kept += reverted.len();
        let survivors: Vec<(usize, usize)> = followers.items().into_iter().filter(|(_key, driver)| allowed.contains(*driver)).collect();
        followers = Ordered::new(count);
        for (key, driver) in survivors {
            followers.insert(key, driver);
        }
    }
    Some((kept, followers))
}

/// One emitted face after the lift: kept whole, or cut into its ears (indices into the polygon, in the order the oracle emits them).
#[derive(Debug, Clone)]
pub enum Settled {
    Whole,
    Ears(Vec<[usize; 3]>),
}

/// `SourceFacesV1` in native form and the fate of every polygon (by face, by polygon).
#[derive(Debug)]
pub struct FacesOutput {
    pub changed_any: bool,
    pub settled: Vec<Vec<Settled>>,
    pub max_off_plane: f64,
    pub triangulated: usize,
    pub flipped: usize,
}

/// `settle_emitted_faces(polygons, before, sourced, chart_of, budget, reverse)`: `polygons` by vertex index, `before`/`after` the positions by
/// vertex index, `chart[k]` the chart point of vertex `k` (`None`: no point; asked only for a face of four or more with a moved vertex, where the
/// oracle would raise `KeyError`, and then the answer is `Err(None)`).
pub fn settle_emitted_faces(
    ctx: &mut ExactCtx<'_>,
    polygons: &[Vec<Vec<usize>>],
    before: &[P3],
    after: &[P3],
    chart: &[Option<(SqrtSum, SqrtSum)>],
    reverse: bool,
) -> Result<FacesOutput, Option<cftuv_clip::error::ClipError>> {
    let changed: Vec<bool> = before.iter().zip(after).map(|(old, new)| !eq3(old, new)).collect();
    if !changed.iter().any(|item| *item) {
        return Ok(FacesOutput { changed_any: false, settled: Vec::new(), max_off_plane: 0.0, triangulated: 0, flipped: 0 });
    }
    let area = |source: &[P3], keys: &[usize]| area_vector(&keys.iter().map(|key| source[*key]).collect::<Vec<_>>());
    let mut worst = 0.0f64;
    let (mut cut, mut flipped) = (0usize, 0usize);
    let mut settled = Vec::with_capacity(polygons.len());
    for face_polygons in polygons {
        let mut kept = Vec::with_capacity(face_polygons.len());
        for polygon in face_polygons {
            if !polygon.iter().any(|key| changed[*key]) {
                kept.push(Settled::Whole);
                continue;
            }
            let (ears, exact) = if polygon.len() == 3 {
                (vec![[0usize, 1, 2]], true)
            } else {
                let mut points: Vec<Pt> = Vec::with_capacity(polygon.len());
                for key in polygon {
                    let Some(point) = chart[*key].as_ref() else {
                        return Err(None);
                    };
                    points.push((&point.0, &point.1));
                }
                match triangulate_exact(ctx, &points).map_err(Some)? {
                    Some(ears) => (ears, true),
                    None => ((1..polygon.len() - 1).map(|index| [0, index, index + 1]).collect(), false),
                }
            };
            let mut lost = 0usize;
            for ear in &ears {
                let keys = [polygon[ear[0]], polygon[ear[1]], polygon[ear[2]]];
                if !(dot(&area(before, &keys), &area(after, &keys)) > 0.0) {
                    lost += 1;
                }
            }
            if polygon.len() == 3 {
                flipped += lost;
                kept.push(Settled::Whole);
            } else if lost > 0 || !exact {
                cut += 1;
                flipped += lost;
                let swap = reverse && exact;
                kept.push(Settled::Ears(ears.iter().map(|&[a, b, c]| if swap { [a, c, b] } else { [a, b, c] }).collect()));
            } else {
                kept.push(Settled::Whole);
                let old: Vec<P3> = polygon.iter().map(|key| before[*key]).collect();
                let new: Vec<P3> = polygon.iter().map(|key| after[*key]).collect();
                worst = py_max(worst, off_plane_distance(&old, &new));
            }
        }
        settled.push(kept);
    }
    Ok(FacesOutput { changed_any: true, settled, max_off_plane: worst, triangulated: cut, flipped })
}
