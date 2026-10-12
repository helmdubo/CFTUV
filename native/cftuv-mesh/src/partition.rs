//! `convex_partition.py`: the law `CONVEX_PARTITION_BY_DIAGONALS_V1`. A piece with no affine or bilinear face as a whole is cut into admissible
//! faces by diagonals between its own vertices (no new vertex): the exact planner (the fewest parts, memoized over sub-rings) up to
//! [`EXACT_VERTICES`] vertices, the greedy Hertel-Mehlhorn merge above, and the proof that the parts subdivide the piece.
//!
//! The ORDER of the exact questions is the cost, and it is the oracle's: diagonals by ascending pair of indices, the first part count to win, the
//! memo of the sub-rings (`lru_cache`) and of the verdicts (`allowed`) kept per call.

use std::collections::{HashMap, HashSet};

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::faces::{orientation, shoelace_sign, Pt};
use cftuv_clip::tessellate::triangulate_exact;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::sqrt_sum::SIGN_FILTER_BITS;

use crate::error::MeshResult;
use crate::predicates::{contour_is_simple, counter_clockwise_ring, segments_cross};

/// A piece up to this many vertices is searched exactly (the fewest parts), a longer one by the greedy merge: the bound of the method, not a refusal.
pub const EXACT_VERTICES: usize = 8;

/// The numbers of the law (and the keys of the counters of the materializer). Zero numbers are not written.
pub const PIECES_PARTITIONED: &str = "MATERIALIZE_CONVEX_PARTITION_PIECES";
pub const PIECES_EXACT: &str = "MATERIALIZE_CONVEX_PARTITION_PIECES_EXACT";
pub const PIECES_GREEDY: &str = "MATERIALIZE_CONVEX_PARTITION_PIECES_GREEDY";
pub const FACES_EMITTED: &str = "MATERIALIZE_CONVEX_PARTITION_FACES_EMITTED";
pub const DIAGONALS: &str = "MATERIALIZE_CONVEX_PARTITION_DIAGONALS";
pub const PIECES_REFUSED: &str = "MATERIALIZE_CONVEX_PARTITION_PIECES_REFUSED";
pub const REFUSED_PREFIX: &str = "MATERIALIZE_CONVEX_PARTITION_REFUSED_";

pub const REASON_NO_TRIANGULATION: &str = "NO_TRIANGULATION";
pub const REASON_NO_PARTITION: &str = "NO_ADMISSIBLE_PARTITION";
pub const REASON_NO_GAIN: &str = "NO_GAIN_OVER_EARS";
pub const REASON_NOT_A_SUBDIVISION: &str = "NOT_A_SUBDIVISION";
pub const REASON_UV_FOLD: &str = "UV_FOLD";
pub const REASON_AREA: &str = "AREA_DOES_NOT_CLOSE";

/// `PartitionPlanV1`: the parts as rings of indices of the input contour, counter-clockwise.
pub struct Plan {
    pub pieces: Vec<Vec<usize>>,
    pub order: Vec<usize>,
    pub exact: bool,
}

/// The decision of the admissibility of a part, asked once per distinct ring of indices.
pub type Admissible<'f> = dyn FnMut(&mut ExactCtx<'_>, &[usize]) -> MeshResult<bool> + 'f;

/// `_on_open_segment(first, second, other, budget)`: `other` lies strictly between the ends of the segment, on its line. Both signs are asked once
/// the point is collinear.
fn on_open_segment(ctx: &mut ExactCtx<'_>, first: Pt, second: Pt, other: Pt) -> MeshResult<bool> {
    if orientation(ctx, first, second, other)? != 0 {
        return Ok(false);
    }
    let dot = |ctx: &mut ExactCtx<'_>, from: Pt, to: Pt, at: Pt| {
        let x = to.0.sub(from.0).mul(&at.0.sub(from.0), ctx.products);
        let y = to.1.sub(from.1).mul(&at.1.sub(from.1), ctx.products);
        x.add(&y)
    };
    let along = dot(ctx, first, second, other);
    let along = exact::sign(ctx, &along, SIGN_FILTER_BITS)?;
    let beyond = dot(ctx, second, first, other);
    let beyond = exact::sign(ctx, &beyond, SIGN_FILTER_BITS)?;
    Ok(along > 0 && beyond > 0)
}

/// `_enters_interior(ring, index, target, budget)`: the direction from vertex `index` to `target` enters the angle of the ring strictly inside.
fn enters_interior(ctx: &mut ExactCtx<'_>, ring: &[Pt], index: usize, target: Pt) -> MeshResult<bool> {
    let size = ring.len();
    let (before, here, after) = (ring[(index + size - 1) % size], ring[index], ring[(index + 1) % size]);
    let toward_after = orientation(ctx, here, after, target)?;
    let from_before = orientation(ctx, before, here, target)?;
    let turn = orientation(ctx, before, here, after)?;
    Ok(if turn > 0 {
        from_before > 0 && toward_after > 0
    } else if turn < 0 {
        from_before > 0 || toward_after > 0
    } else {
        from_before > 0
    })
}

/// `_valid_diagonal(ring, first, second, budget)`: the diagonal lies inside the ring: no crossing, no vertex on it, both ends enter inside.
fn valid_diagonal(ctx: &mut ExactCtx<'_>, ring: &[Pt], first: usize, second: usize) -> MeshResult<bool> {
    let size = ring.len();
    let (start, end) = (ring[first], ring[second]);
    for index in 0..size {
        if index == first || index == second {
            continue;
        }
        if on_open_segment(ctx, start, end, ring[index])? {
            return Ok(false);
        }
        let next = (index + 1) % size;
        if next == first || next == second {
            continue;
        }
        if segments_cross(ctx, start, end, ring[index], ring[next])? {
            return Ok(false);
        }
    }
    Ok(enters_interior(ctx, ring, first, end)? && enters_interior(ctx, ring, second, start)?)
}

/// The state of one `plan_partition` call: the verdicts of `allowed` and the memo of `solve`.
struct Planner<'p, 'a, 'f> {
    points: &'p [Pt<'a>],
    admissible: &'p mut Admissible<'f>,
    verdicts: HashMap<Vec<usize>, bool, FxBuild>,
    solved: HashMap<Vec<usize>, (Option<u32>, Vec<Vec<usize>>), FxBuild>,
}

impl Planner<'_, '_, '_> {
    /// `allowed(verts)`: a triangle, or what `admissible` says, remembered.
    fn allowed(&mut self, ctx: &mut ExactCtx<'_>, verts: &[usize]) -> MeshResult<bool> {
        if let Some(found) = self.verdicts.get(verts) {
            return Ok(*found);
        }
        let found = verts.len() == 3 || (self.admissible)(ctx, verts)?;
        self.verdicts.insert(verts.to_vec(), found);
        Ok(found)
    }

    /// `solve(verts)`: `(number of parts, parts)` of the smallest partition of the sub-ring, or `(None, [])`.
    fn solve(&mut self, ctx: &mut ExactCtx<'_>, verts: &[usize]) -> MeshResult<(Option<u32>, Vec<Vec<usize>>)> {
        if let Some(found) = self.solved.get(verts) {
            return Ok(found.clone());
        }
        let answer = self.solve_uncached(ctx, verts)?;
        self.solved.insert(verts.to_vec(), answer.clone());
        Ok(answer)
    }

    fn solve_uncached(&mut self, ctx: &mut ExactCtx<'_>, verts: &[usize]) -> MeshResult<(Option<u32>, Vec<Vec<usize>>)> {
        if self.allowed(ctx, verts)? {
            return Ok((Some(1), vec![verts.to_vec()]));
        }
        let subring: Vec<Pt> = verts.iter().map(|index| self.points[*index]).collect();
        let size = verts.len();
        let mut best: (Option<u32>, Vec<Vec<usize>>) = (None, Vec::new());
        for first in 0..size {
            for second in first + 2..size {
                if first == 0 && second == size - 1 {
                    continue;
                }
                if !valid_diagonal(ctx, &subring, first, second)? {
                    continue;
                }
                let (left_count, left) = self.solve(ctx, &verts[first..=second])?;
                let Some(left_count) = left_count else {
                    continue;
                };
                let mut rest: Vec<usize> = verts[second..].to_vec();
                rest.extend_from_slice(&verts[..=first]);
                let (right_count, right) = self.solve(ctx, &rest)?;
                let Some(right_count) = right_count else {
                    continue;
                };
                if best.0.is_none_or(|count| left_count + right_count < count) {
                    let mut parts = left;
                    parts.extend(right);
                    best = (Some(left_count + right_count), parts);
                    if best.0 == Some(2) {
                        return Ok(best);
                    }
                }
            }
        }
        Ok(best)
    }
}

/// `_merged_ring(first, second, start, end)`: the union of two parts across their common diagonal.
fn merged_ring(first: &[usize], second: &[usize], start: usize, end: usize) -> Vec<usize> {
    let at = first.iter().position(|item| *item == start).expect("the diagonal is an edge of the first part");
    let mut rotated_first = first[at..].to_vec();
    rotated_first.extend_from_slice(&first[..at]);
    let at = second.iter().position(|item| *item == end).expect("the diagonal is an edge of the second part");
    let mut rotated_second = second[at..].to_vec();
    rotated_second.extend_from_slice(&second[..at]);
    let mut out: Vec<usize> = rotated_first[1..].to_vec();
    out.push(start);
    out.extend_from_slice(&rotated_second[2..]);
    out
}

fn edges_of(face: &[usize]) -> Vec<(usize, usize)> {
    (0..face.len()).map(|at| (face[at], face[(at + 1) % face.len()])).collect()
}

impl Planner<'_, '_, '_> {
    /// `_plan_greedy(triangles, allowed)`: the Hertel-Mehlhorn merge from the ear triangulation.
    fn greedy(&mut self, ctx: &mut ExactCtx<'_>, triangles: &[[usize; 3]]) -> MeshResult<Vec<Vec<usize>>> {
        let mut faces: Vec<Option<Vec<usize>>> = triangles.iter().map(|triangle| Some(triangle.to_vec())).collect();
        let mut owner: HashMap<(usize, usize), usize, FxBuild> = HashMap::default();
        for (number, face) in faces.iter().enumerate() {
            for edge in edges_of(face.as_ref().expect("a face")) {
                owner.insert(edge, number);
            }
        }
        let mut diagonals: Vec<(usize, usize)> =
            owner.keys().filter(|(a, b)| owner.contains_key(&(*b, *a))).map(|(a, b)| (*a.min(b), *a.max(b))).collect::<HashSet<_>>().into_iter().collect();
        diagonals.sort_unstable();
        let mut merged = true;
        while merged {
            merged = false;
            for (low, high) in &diagonals {
                let (low, high) = (*low, *high);
                if !owner.contains_key(&(low, high)) || !owner.contains_key(&(high, low)) {
                    continue;
                }
                let (first, second) = (owner[&(low, high)], owner[&(high, low)]);
                let union = merged_ring(faces[first].as_ref().expect("a live face"), faces[second].as_ref().expect("a live face"), low, high);
                if !self.allowed(ctx, &union)? {
                    continue;
                }
                for number in [first, second] {
                    let face = faces[number].take().expect("a live face");
                    for edge in edges_of(&face) {
                        owner.remove(&edge);
                    }
                }
                let created = faces.len();
                for edge in edges_of(&union) {
                    owner.insert(edge, created);
                }
                faces.push(Some(union));
                merged = true;
            }
        }
        let mut result: Vec<Vec<usize>> = faces
            .into_iter()
            .flatten()
            .map(|face| {
                let smallest = face.iter().enumerate().min_by_key(|(_, item)| **item).map(|(at, _)| at).expect("a face is not empty");
                let mut rotated = face[smallest..].to_vec();
                rotated.extend_from_slice(&face[..smallest]);
                rotated
            })
            .collect();
        result.sort();
        Ok(result)
    }
}

/// `plan_partition(points, budget, admissible)`: the plan, or the name of the refusal.
pub fn plan_partition(ctx: &mut ExactCtx<'_>, points: &[Pt], admissible: &mut Admissible<'_>) -> MeshResult<Result<Plan, &'static str>> {
    let ring = counter_clockwise_ring(ctx, points)?;
    let triangles = match &ring {
        None => None,
        Some(_) => triangulate_exact(ctx, points)?,
    };
    let (Some(ring), Some(triangles)) = (ring, triangles) else {
        return Ok(Err(REASON_NO_TRIANGULATION));
    };
    let mut planner = Planner { points, admissible, verdicts: HashMap::default(), solved: HashMap::default() };
    let exact_search = points.len() <= EXACT_VERTICES;
    if planner.allowed(ctx, &ring)? {
        return Ok(Ok(Plan { pieces: vec![ring.clone()], order: ring, exact: exact_search }));
    }
    let pieces = if exact_search {
        let (count, pieces) = planner.solve(ctx, &ring)?;
        count.map(|_| pieces)
    } else {
        Some(planner.greedy(ctx, &triangles)?)
    };
    Ok(match pieces {
        None => Err(REASON_NO_PARTITION),
        Some(pieces) => Ok(Plan { pieces, order: ring, exact: exact_search }),
    })
}

/// `verify_partition(points, plan, budget)`: `None` when the parts are simple, positive, and their half-edges cancel in pairs to the contour;
/// else the reason (`NOT_A_SUBDIVISION`).
pub fn verify_partition(ctx: &mut ExactCtx<'_>, points: &[Pt], plan: &Plan) -> MeshResult<Option<&'static str>> {
    let mut used: Vec<((usize, usize), u32)> = Vec::new();
    let mut at: HashMap<(usize, usize), usize, FxBuild> = HashMap::default();
    for piece in &plan.pieces {
        let ring_points: Vec<Pt> = piece.iter().map(|index| points[*index]).collect();
        let distinct: HashSet<&usize> = piece.iter().collect();
        if distinct.len() != piece.len() || shoelace_sign(ctx, &ring_points)? <= 0 {
            return Ok(Some(REASON_NOT_A_SUBDIVISION));
        }
        if !contour_is_simple(ctx, &ring_points)? {
            return Ok(Some(REASON_NOT_A_SUBDIVISION));
        }
        for edge in edges_of(piece) {
            match at.get(&edge) {
                Some(found) => used[*found].1 += 1,
                None => {
                    at.insert(edge, used.len());
                    used.push((edge, 1));
                }
            }
        }
    }
    let boundary: HashSet<(usize, usize)> = edges_of(&plan.order).into_iter().collect();
    for ((first, second), times) in &used {
        if *times != 1 {
            return Ok(Some(REASON_NOT_A_SUBDIVISION));
        }
        let paired = at.contains_key(&(*second, *first));
        if paired == boundary.contains(&(*first, *second)) {
            return Ok(Some(REASON_NOT_A_SUBDIVISION));
        }
    }
    if boundary.iter().any(|edge| !at.contains_key(edge)) {
        return Ok(Some(REASON_NOT_A_SUBDIVISION));
    }
    Ok(None)
}

/// The names of the counters of the law in the order of `partition_counters` (the reasons after the five numbers).
pub const REASONS: [&str; 6] = [REASON_NO_TRIANGULATION, REASON_NO_PARTITION, REASON_NO_GAIN, REASON_NOT_A_SUBDIVISION, REASON_UV_FOLD, REASON_AREA];
