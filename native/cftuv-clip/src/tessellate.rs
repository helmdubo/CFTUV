//! `materialize/tessellate.py`: the tessellation predicates the clip stage calls (`triangulate_exact`,
//! `convex_quad_ring`, `has_right_turn`). The ORDER of the exact predicates is the cost (each `orientation` that the
//! binary64 filter cannot prove is a `.sign()`), so the loops below keep the oracle's order, short-circuits and
//! incremental flag updates exactly.

use cftuv_core::exact::ExactCtx;

use crate::error::ClipResult;
use crate::faces::{orientation, shoelace_sign, Pt};

/// `_ear_contains_vertex`: whether vertex `w` lies in the CLOSED triangle `(a, b, c)` (counter-clockwise). The three
/// orientations short-circuit on the first negative.
fn ear_contains_vertex(ctx: &mut ExactCtx<'_>, points: &[Pt], a: usize, b: usize, c: usize, w: usize) -> ClipResult<bool> {
    Ok(orientation(ctx, points[a], points[b], points[w])? >= 0
        && orientation(ctx, points[b], points[c], points[w])? >= 0
        && orientation(ctx, points[c], points[a], points[w])? >= 0)
}

/// `convex(position)` on the current ring: the turn at `ring[position]` is strictly to the left. Python's negative
/// index `ring[-1]` at position 0 is the last element.
fn convex(ctx: &mut ExactCtx<'_>, points: &[Pt], ring: &[usize], position: usize) -> ClipResult<bool> {
    let size = ring.len();
    Ok(orientation(ctx, points[ring[(position + size - 1) % size]], points[ring[position]], points[ring[(position + 1) % size]])? > 0)
}

/// `triangulate_exact(points, budget)`: the ear triangulation as index triples, counter-clockwise whatever the input
/// orientation; `None` when the contour has no triangulation (fewer than three points, zero area, no ear).
pub fn triangulate_exact(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> ClipResult<Option<Vec<[usize; 3]>>> {
    let count = points.len();
    if count < 3 {
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
    let mut flags = Vec::with_capacity(count);
    for position in 0..count {
        flags.push(convex(ctx, points, &ring, position)?);
    }
    let mut triangles: Vec<[usize; 3]> = Vec::with_capacity(count.saturating_sub(2));
    while ring.len() > 3 {
        let size = ring.len();
        let mut found = None;
        'candidates: for position in 0..size {
            if !flags[position] {
                continue;
            }
            let previous = ring[(position + size - 1) % size];
            let current = ring[position];
            let following = ring[(position + 1) % size];
            // only a non-convex vertex can lie inside an ear
            for other in 0..size {
                if !flags[other] && ![previous, current, following].contains(&ring[other]) && ear_contains_vertex(ctx, points, previous, current, following, ring[other])? {
                    continue 'candidates;
                }
            }
            found = Some((position, [previous, current, following]));
            break;
        }
        let Some((position, triangle)) = found else {
            return Ok(None);
        };
        triangles.push(triangle);
        ring.remove(position);
        flags.remove(position);
        let size = size - 1;
        let before = (position + size - 1) % size;
        flags[before] = convex(ctx, points, &ring, before)?;
        let after = position % size;
        flags[after] = convex(ctx, points, &ring, after)?;
    }
    if orientation(ctx, points[ring[0]], points[ring[1]], points[ring[2]])? <= 0 {
        return Ok(None);
    }
    triangles.push([ring[0], ring[1], ring[2]]);
    Ok(Some(triangles))
}

/// `convex_quad_ring(points, budget)`: the counter-clockwise ring of four points when they form a STRICTLY convex
/// quadrilateral, else `None` (not four points, zero area, or any turn that is not strictly left).
pub fn convex_quad_ring(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> ClipResult<Option<[usize; 4]>> {
    if points.len() != 4 {
        return Ok(None);
    }
    let total = shoelace_sign(ctx, points)?;
    if total == 0 {
        return Ok(None);
    }
    let ring = if total > 0 { [0, 1, 2, 3] } else { [3, 2, 1, 0] };
    for position in 0..4 {
        let turn = orientation(ctx, points[ring[(position + 3) % 4]], points[ring[position]], points[ring[(position + 1) % 4]])?;
        if turn <= 0 {
            return Ok(None);
        }
    }
    Ok(Some(ring))
}

/// `has_right_turn(points, ring, budget)`: some vertex of the counter-clockwise `ring` turns strictly right.
/// `any(...)` stops at the first one.
pub fn has_right_turn(ctx: &mut ExactCtx<'_>, points: &[Pt], ring: &[usize]) -> ClipResult<bool> {
    let count = ring.len();
    for position in 0..count {
        let turn = orientation(ctx, points[ring[(position + count - 1) % count]], points[ring[position]], points[ring[(position + 1) % count]])?;
        if turn < 0 {
            return Ok(true);
        }
    }
    Ok(false)
}
