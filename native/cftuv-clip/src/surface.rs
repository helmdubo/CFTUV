//! `BoundSurfaceLiftV1.lift_named` for many points (`assemble.lift_vertices` on a bound surface, SURFACE_LIFT_NATIVE_V1) and the exact
//! facts of a moving vertex the drag certificate reads (`parametric._surface_vertex_facts`, the same law).
//!
//! The oracle is `materialize/lift_surface.py`: `locate` (the window, the box prefilter, `_inside` with its signs paid on the plane's
//! budget), `lift_known` (position, triangle name, offset normal) and the counters `LOCATIONS`, `CANDIDATES`, `PREDICATES`, `ON_EDGE`.
//! A point that lies in no triangle (`_nearest`, the continuation within `EXTRAPOLATION_CELL_BOUND`) is NOT ported: the operation
//! refuses by name ([`ClipError::Unsupported`]) and the host runs the oracle on the untouched state. Nothing here writes a host
//! object: the normal writes are returned in call order with the positions.

use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::{ClipError, ClipResult};
use crate::lift::{lift_known_with, sqrt_sum_binary64, Lifted};
use crate::numeric::window;
use crate::plane::{edge_value, Plane};
use crate::point::Point;

/// What one call adds to the plane's `_tally` (the counters `locate` moves).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SurfaceTally {
    pub locations: u64,
    pub candidates: u64,
    pub predicates: u64,
    pub on_edge: u64,
}

/// The name of the refusal of a point outside every triangle (the oracle continues it or names its own refusal).
pub const OUTSIDE_ALL_TRIANGLES: &str = "SURFACE_LIFT_NATIVE_V1: a point in no triangle of the plane (the continuation `_nearest` is the oracle's)";

/// `_inside(triangle, point)`: the three orientation values when the point is in the closed triangle, else `None`. Each value's sign is paid
/// in the oracle's order and the scan stops at the first value on the wrong side, as the oracle's does.
fn inside(ctx: &mut ExactCtx<'_>, plane: &Plane, index: usize, x: &SqrtSum, y: &SqrtSum, tally: &mut SurfaceTally) -> ClipResult<Option<[SqrtSum; 3]>> {
    let triangle = &plane.triangles[index];
    let direction: i32 = if triangle.twice_area.signum() > 0 { 1 } else { -1 };
    let mut values: Vec<SqrtSum> = Vec::with_capacity(3);
    for edge in 0..3 {
        let value = edge_value(&triangle.chart[edge], &triangle.chart[(edge + 1) % 3], x, y);
        tally.predicates += 1;
        if i32::from(exact::sign(ctx, &value, SIGN_FILTER_BITS)?) * direction < 0 {
            return Ok(None);
        }
        values.push(value);
    }
    let [a, b, c]: [SqrtSum; 3] = values.try_into().map_err(|_| ClipError::Unsupported("three orientation values".into()))?;
    Ok(Some([a, b, c]))
}

/// `locate(point)` without the continuation: the first triangle (in the plane's order) whose box meets the point's window and that holds
/// the point, with its three values.
pub fn locate(ctx: &mut ExactCtx<'_>, plane: &Plane, x: &SqrtSum, y: &SqrtSum, tally: &mut SurfaceTally) -> ClipResult<(usize, [SqrtSum; 3])> {
    tally.locations += 1;
    let [x_low, x_high, y_low, y_high] = window(x, y)?;
    for (index, triangle) in plane.triangles.iter().enumerate() {
        let [x_min, x_max, y_min, y_max] = triangle.bbox;
        if x_high < x_min || x_low > x_max || y_high < y_min || y_low > y_max {
            continue;
        }
        tally.candidates += 1;
        if let Some(values) = inside(ctx, plane, index, x, y, tally)? {
            if values.iter().any(SqrtSum::is_zero) {
                tally.on_edge += 1;
            }
            return Ok((index, values));
        }
    }
    Err(ClipError::Unsupported(OUTSIDE_ALL_TRIANGLES.to_string()))
}

/// `{key: plane.lift_named(point)}` for every point in order: the lifts (position, triangle name, offset normal; the normal is also the
/// write `_normal_by_position[position] = normal` the caller applies in this order) and what the counters gained.
pub fn lift_points(ctx: &mut ExactCtx<'_>, plane: &Plane, points: &[(String, Point)]) -> ClipResult<(Vec<Lifted>, SurfaceTally)> {
    let mut tally = SurfaceTally::default();
    let mut lifted = Vec::with_capacity(points.len());
    for (_key, (x, y)) in points {
        let (index, values) = locate(ctx, plane, x, y, &mut tally)?;
        lifted.push(lift_known_with(&plane.triangles[index], plane.lift_factors(index)?, [&values[0], &values[1], &values[2]])?);
    }
    Ok((lifted, tally))
}

/// `parametric._surface_vertex_facts(triangle, point, slope, floats)`: whether the moving point lies on an edge of its triangle that it
/// leaves (`_leaves_edge`), and, when `floats`, the binary64 of the three orientation values (`values_in`) and of their three slopes
/// (`_edge_slope`), each the midpoint of its 64-bit enclosure (`parametric._float`). Exact arithmetic only: no sign, no budget, no memory.
pub fn vertex_facts(plane: &Plane, index: usize, point: &Point, slope: &Point, floats: bool) -> ClipResult<(bool, Option<([f64; 3], [f64; 3])>)> {
    let triangle = &plane.triangles[index];
    let (x1, y1) = slope;
    let mut leaves = false;
    let mut values = [0.0f64; 3];
    let mut slopes = [0.0f64; 3];
    for edge in 0..3 {
        let start = &triangle.chart[edge];
        let end = &triangle.chart[(edge + 1) % 3];
        let dx = end.0.sub(&start.0);
        let dy = end.1.sub(&start.1);
        // `_edge_slope`: `y1.scaled(dx) - x1.scaled(dy)`
        let moving = y1.scaled(&dx).sub(&x1.scaled(&dy));
        let value = edge_value(start, end, &point.0, &point.1);
        if !moving.is_zero() && value.is_zero() {
            leaves = true;
        }
        if floats {
            values[edge] = sqrt_sum_binary64(&value)?;
            slopes[edge] = sqrt_sum_binary64(&moving)?;
        }
    }
    Ok((leaves, floats.then_some((values, slopes))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cftuv_core::num::IBig;
    use cftuv_core::rat::Rat;

    use crate::plane::Triangle;

    fn rat(n: i64) -> Rat {
        Rat::from_int(IBig::from(n))
    }

    fn triangle(name: &str, chart: [(i64, i64); 3]) -> Triangle {
        let xs = chart.map(|point| point.0 as f64);
        let ys = chart.map(|point| point.1 as f64);
        let bbox = [xs.iter().cloned().fold(f64::INFINITY, f64::min), xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max), ys.iter().cloned().fold(f64::INFINITY, f64::min), ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max)];
        let chart = chart.map(|(x, y)| (rat(x), rat(y)));
        let twice_area = chart[1].0.sub(&chart[0].0).mul(&chart[2].1.sub(&chart[0].1)).sub(&chart[1].1.sub(&chart[0].1).mul(&chart[2].0.sub(&chart[0].0)));
        let corners = [[rat(0), rat(0), rat(0)], [rat(1), rat(0), rat(0)], [rat(0), rat(1), rat(0)]];
        Triangle { name: name.to_string(), chart, corners, twice_area, bbox, normals: None, face: String::new() }
    }

    #[test]
    fn a_point_on_an_edge_it_does_not_leave_is_not_leaving() {
        let plane = Plane::new(vec![triangle("t", [(0, 0), (4, 0), (0, 4)])]);
        let on_bottom = (SqrtSum::rational(&rat(2)), SqrtSum::rational(&rat(0)));
        let along = (SqrtSum::rational(&rat(1)), SqrtSum::rational(&rat(0)));
        let up = (SqrtSum::rational(&rat(0)), SqrtSum::rational(&rat(1)));
        assert!(!vertex_facts(&plane, 0, &on_bottom, &along, false).unwrap().0);
        assert!(vertex_facts(&plane, 0, &on_bottom, &up, false).unwrap().0);
        let (_, floats) = vertex_facts(&plane, 0, &on_bottom, &up, true).unwrap();
        let (values, slopes) = floats.unwrap();
        assert_eq!(values[0], 0.0);
        assert_eq!(slopes[0], 4.0);
    }
}
