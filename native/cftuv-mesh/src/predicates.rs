//! The contour predicates of `tessellate.py` and `wavefront/faces.py` the clip port does not carry: `counter_clockwise_ring`,
//! `convex_polygon_ring`, `triangulate_from_apex`, `segments_cross`, `contour_crossings`, `contour_is_simple`.
//!
//! The ORDER of the exact questions is the cost (an `orientation` the binary64 filter cannot prove is a `.sign()`: SIGN_COUNTS, budget, memory), so
//! the loops keep the oracle's order and its short circuits; and where the oracle computes a whole list before it tests it (`contour_crossings`),
//! so does this.

use cftuv_clip::faces::{orientation, shoelace_sign, Pt};
use cftuv_clip::tessellate::{has_right_turn, triangulate_exact};
use cftuv_core::exact::ExactCtx;

use crate::error::MeshResult;

/// `counter_clockwise_ring(points, budget)`: the indices counter-clockwise by the sign of the area; `None` for a zero area.
pub fn counter_clockwise_ring(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> MeshResult<Option<Vec<usize>>> {
    let total = shoelace_sign(ctx, points)?;
    if total == 0 {
        return Ok(None);
    }
    let count = points.len();
    Ok(Some(if total > 0 { (0..count).collect() } else { (0..count).rev().collect() }))
}

/// `convex_polygon_ring(points, budget)`: the counter-clockwise ring of a contour of four points or more without a right turn (points on a
/// straight line allowed), else `None`.
pub fn convex_polygon_ring(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> MeshResult<Option<Vec<usize>>> {
    if points.len() < 4 {
        return Ok(None);
    }
    let Some(ring) = counter_clockwise_ring(ctx, points)? else {
        return Ok(None);
    };
    if has_right_turn(ctx, points, &ring)? {
        return Ok(None);
    }
    Ok(Some(ring))
}

/// `triangulate_from_apex(points, apex, budget)`: the fan `(apex, i, j)` from the vertex `apex`, counter-clockwise whatever the input orientation,
/// or `None` (fewer than three points, an apex outside the contour, a zero area, a triangle that is not strictly positive).
pub fn triangulate_from_apex(ctx: &mut ExactCtx<'_>, points: &[Pt], apex: usize) -> MeshResult<Option<Vec<[usize; 3]>>> {
    let count = points.len();
    if count < 3 || apex >= count {
        return Ok(None);
    }
    let total = shoelace_sign(ctx, points)?;
    if total == 0 {
        return Ok(None);
    }
    let mut ring: Vec<usize> = (0..count).collect();
    if total < 0 {
        ring.reverse();
    }
    let start = ring.iter().position(|item| *item == apex).expect("the apex is a vertex of the ring");
    ring.rotate_left(start);
    let mut triangles = Vec::with_capacity(count - 2);
    for window in ring[1..].windows(2) {
        let (first, second) = (window[0], window[1]);
        if orientation(ctx, points[apex], points[first], points[second])? <= 0 {
            return Ok(None);
        }
        triangles.push([apex, first, second]);
    }
    Ok(Some(triangles))
}

/// `segments_cross(first_start, first_end, second_start, second_end, budget)`: the segments cross TRANSVERSALLY. Both orientations of the first
/// conjunct are always asked (the product of two signs is taken); the second conjunct only when the first holds.
pub fn segments_cross(ctx: &mut ExactCtx<'_>, first_start: Pt, first_end: Pt, second_start: Pt, second_end: Pt) -> MeshResult<bool> {
    let a = orientation(ctx, second_start, second_end, first_start)?;
    let b = orientation(ctx, second_start, second_end, first_end)?;
    if i32::from(a) * i32::from(b) >= 0 {
        return Ok(false);
    }
    let c = orientation(ctx, first_start, first_end, second_start)?;
    let d = orientation(ctx, first_start, first_end, second_end)?;
    Ok(i32::from(c) * i32::from(d) < 0)
}

/// Whether `contour_crossings(points, budget)` is non-empty. The oracle builds the WHOLE list before `not list` is taken, so every pair is asked.
pub fn has_crossings(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> MeshResult<bool> {
    let size = points.len();
    let mut found = false;
    for first in 0..size {
        for second in first + 1..size {
            if (second + 1) % size == first || (first + 1) % size == second {
                continue;
            }
            if segments_cross(ctx, points[first], points[(first + 1) % size], points[second], points[(second + 1) % size])? {
                found = true;
            }
        }
    }
    Ok(found)
}

/// `contour_is_simple(points, budget)`: no transversal crossing and an admissible triangulation (the triangulation only when there is no crossing).
pub fn contour_is_simple(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> MeshResult<bool> {
    if points.len() < 3 {
        return Ok(false);
    }
    if has_crossings(ctx, points)? {
        return Ok(false);
    }
    Ok(triangulate_exact(ctx, points)?.is_some())
}
