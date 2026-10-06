//! Binary64 sign filters (`float_filter.py`): either prove a sign or yield to the exact path.
//!
//! Bit-exact to CPython: the same operations in the same order (Rust never fuses a multiply and an add), the
//! same constants, the same `OverflowError` and denormal branches, the same finiteness test. The Python
//! id()-keyed table is a pure cache; here the `(centre, bound)` of a coordinate is cached on the immutable
//! [`SqrtSum`] itself, which changes cost only.

use crate::pyfloat::{math_sqrt, ratio_to_f64};
use crate::sqrt_sum::{IntForm, SqrtSum};

/// Twice the binary64 unit roundoff, the slack every bound is computed with (`2.0 ** -52`).
const SLACK: f64 = f64::EPSILON;
/// The smallest normal binary64 (`float_info.min`, `2.2250738585072014e-308`).
const FLOOR: f64 = f64::MIN_POSITIVE;
/// The error bound the value must beat; `1.0 + 1e-9` as Python computes it.
const MARGIN: f64 = 1.0 + 1e-9;

/// `(centre, bound)`: `|value - centre| <= bound`.
pub type Entry = (f64, f64);

/// A point with two sqrt-sum coordinates.
pub type Point<'a> = (&'a SqrtSum, &'a SqrtSum);

/// The measure of a value from its integer form: `float(coefficient)` is the correctly rounded `a_m / L`, which is the same binary64 whether or not
/// the fraction is in lowest terms.
fn measure(form: &IntForm) -> Option<Entry> {
    let mut total = 0.0f64;
    let mut weight = 0.0f64;
    for (radicand, numerator) in &form.items {
        // OverflowError in `float(coefficient)` or in `sqrt(radicand)` sends the value to the exact path.
        let centre = ratio_to_f64(numerator, &form.common).ok()?;
        // A coefficient below the smallest normal has an absolute (not relative) conversion error.
        if centre.abs() < FLOOR {
            return None;
        }
        let term_value = centre * math_sqrt(radicand).ok()?;
        total += term_value;
        weight += term_value.abs();
    }
    let bound = (form.items.len() + 6) as f64 * SLACK * weight + FLOOR;
    // Python's `bound - bound == 0.0 and total - total == 0.0` (`inf - inf` and `nan - nan` are not zero):
    // centre and bound must both be finite.
    if bound.is_finite() && total.is_finite() {
        Some((total, bound))
    } else {
        None
    }
}

/// `centre_and_bound`: `None` when binary64 does not take the value.
pub fn centre_and_bound(value: &SqrtSum) -> Option<Entry> {
    value.float_measure(measure)
}

/// The sign of the doubled oriented area `(b - a) x (c - a)`, or `None` (not proved).
pub fn orientation_sign(first: Point, second: Point, third: Point) -> Option<i8> {
    let mut entries = [(0.0, 0.0); 6];
    for (slot, coordinate) in [first.0, first.1, second.0, second.1, third.0, third.1].into_iter().enumerate() {
        entries[slot] = centre_and_bound(coordinate)?;
    }
    let [(ax, eax), (ay, eay), (bx, ebx), (by, eby), (cx, ecx), (cy, ecy)] = entries;
    let slack = SLACK;
    // (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    let d1 = bx - ax;
    let e1 = ebx + eax + slack * d1.abs();
    let d2 = cy - ay;
    let e2 = ecy + eay + slack * d2.abs();
    let d3 = by - ay;
    let e3 = eby + eay + slack * d3.abs();
    let d4 = cx - ax;
    let e4 = ecx + eax + slack * d4.abs();
    let left = d1 * d2;
    let left_bound = d1.abs() * e2 + d2.abs() * e1 + e1 * e2 + slack * left.abs() + FLOOR;
    let right = d3 * d4;
    let right_bound = d3.abs() * e4 + d4.abs() * e3 + e3 * e4 + slack * right.abs() + FLOOR;
    let value = left - right;
    let bound = (left_bound + right_bound + slack * value.abs()) * MARGIN;
    if value.abs() > bound {
        Some(if value > 0.0 { 1 } else { -1 })
    } else {
        None
    }
}

/// `(value, bound)` of the orientation of a point against the line through `(start_x, start_y)` with step
/// `(step_x, step_y)`: `step_x * (y - start_y) - step_y * (x - start_x)`. The line data are floats that hold
/// integers exactly (no error of their own). `None`: binary64 does not take a coordinate.
pub fn line_estimate(point: Point, start_x: f64, start_y: f64, step_x: f64, step_y: f64) -> Option<Entry> {
    let (centre_x, error_x_base) = centre_and_bound(point.0)?;
    let (centre_y, error_y_base) = centre_and_bound(point.1)?;
    let slack = SLACK;
    let along_y = centre_y - start_y;
    let error_y = error_y_base + slack * along_y.abs();
    let along_x = centre_x - start_x;
    let error_x = error_x_base + slack * along_x.abs();
    let first = step_x * along_y;
    let second = step_y * along_x;
    let value = first - second;
    let bound = (step_x.abs() * error_y + step_y.abs() * error_x + slack * (first.abs() + second.abs() + value.abs()) + FLOOR) * MARGIN;
    Some((value, bound))
}

/// The sign of the doubled oriented area of a polygon (a fan from the first point), or `None`.
pub fn polygon_sign(points: &[Point]) -> Option<i8> {
    if points.len() < 3 {
        return None;
    }
    let mut flat: Vec<Entry> = Vec::with_capacity(points.len() * 2);
    for point in points {
        flat.push(centre_and_bound(point.0)?);
        flat.push(centre_and_bound(point.1)?);
    }
    let slack = SLACK;
    let (px, epx) = flat[0];
    let (py, epy) = flat[1];
    let mut ux = flat[2].0 - px;
    let mut eux = flat[2].1 + epx + slack * ux.abs();
    let mut uy = flat[3].0 - py;
    let mut euy = flat[3].1 + epy + slack * uy.abs();
    let mut total = 0.0f64;
    let mut total_bound = 0.0f64;
    for index in 2..points.len() {
        let vx = flat[2 * index].0 - px;
        let evx = flat[2 * index].1 + epx + slack * vx.abs();
        let vy = flat[2 * index + 1].0 - py;
        let evy = flat[2 * index + 1].1 + epy + slack * vy.abs();
        let left = ux * vy;
        let left_bound = ux.abs() * evy + vy.abs() * eux + eux * evy + slack * left.abs() + FLOOR;
        let right = uy * vx;
        let right_bound = uy.abs() * evx + vx.abs() * euy + euy * evx + slack * right.abs() + FLOOR;
        let area = left - right;
        total += area;
        total_bound += left_bound + right_bound + slack * area.abs() + slack * total.abs();
        ux = vx;
        eux = evx;
        uy = vy;
        euy = evy;
    }
    let bound = total_bound * MARGIN;
    if total.abs() > bound {
        Some(if total > 0.0 { 1 } else { -1 })
    } else {
        None
    }
}

fn sub(first: Entry, second: Entry) -> Entry {
    let value = first.0 - second.0;
    (value, first.1 + second.1 + SLACK * value.abs())
}

fn add(first: Entry, second: Entry) -> Entry {
    let value = first.0 + second.0;
    (value, first.1 + second.1 + SLACK * value.abs())
}

fn mul(first: Entry, second: Entry) -> Entry {
    let value = first.0 * second.0;
    (value, first.0.abs() * second.1 + second.0.abs() * first.1 + first.1 * second.1 + SLACK * value.abs() + FLOOR)
}

/// Whether binary64 proves that the vertex `index` does NOT lie on the affine map of values solved from the
/// three base vertices. `points` and `values` hold the base vertices first, the checked vertex last; each
/// vertex gives two coordinates and two values. `true`: a violation is proved; `false`: not proved.
pub fn affine_map_violated(points: &[Point; 4], values: &[Point; 4]) -> bool {
    let mut found = [(0.0, 0.0); 16];
    for (vertex, (point, value)) in points.iter().zip(values.iter()).enumerate() {
        for (offset, coordinate) in [point.0, point.1, value.0, value.1].into_iter().enumerate() {
            match centre_and_bound(coordinate) {
                Some(entry) => found[4 * vertex + offset] = entry,
                None => return false,
            }
        }
    }
    // four entries per vertex: x, y, f0, f1
    let [x0, y0, a0, b0] = [found[0], found[1], found[2], found[3]];
    let [x1, y1, a1, b1] = [found[4], found[5], found[6], found[7]];
    let [x2, y2, a2, b2] = [found[8], found[9], found[10], found[11]];
    let [xq, yq, aq, bq] = [found[12], found[13], found[14], found[15]];
    let (ux, uy) = (sub(x1, x0), sub(y1, y0));
    let (vx, vy) = (sub(x2, x0), sub(y2, y0));
    let (qx, qy) = (sub(xq, x0), sub(yq, y0));
    let det = sub(mul(ux, vy), mul(uy, vx));
    let first_weight = sub(mul(qx, vy), mul(qy, vx));
    let second_weight = sub(mul(ux, qy), mul(uy, qx));
    for (f0, f1, f2, fq) in [(a0, a1, a2, aq), (b0, b1, b2, bq)] {
        let left = mul(det, sub(fq, f0));
        let right = add(mul(first_weight, sub(f1, f0)), mul(second_weight, sub(f2, f0)));
        let residual = sub(left, right);
        if residual.0.abs() > residual.1 * MARGIN {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::num::{IBig, UBig};
    use crate::rat::{Coef, Rat};
    use crate::sqrt_sum::Term;

    fn term(radicand: u64, n: i64, d: i64) -> Term {
        Term { radicand: UBig::from(radicand), coef: Coef::fraction(Rat::new(IBig::from(n), IBig::from(d)).unwrap()) }
    }

    fn sum(terms: Vec<Term>) -> SqrtSum {
        SqrtSum::from_terms(terms).unwrap()
    }

    #[test]
    fn the_constants_are_the_ones_python_computes() {
        assert_eq!(SLACK.to_bits(), 0x3cb0_0000_0000_0000);
        assert_eq!(FLOOR.to_bits(), 0x0010_0000_0000_0000);
        assert_eq!(MARGIN.to_bits(), 0x3ff0_0000_0044_b830, "1.0 + 1e-9");
    }

    #[test]
    fn a_coordinate_gets_a_centre_and_an_error_bound() {
        let value = sum(vec![term(1, 1, 2), term(2, 3, 1)]);
        let (centre, bound) = centre_and_bound(&value).unwrap();
        assert!((centre - (0.5 + 3.0 * 2.0f64.sqrt())).abs() < 1e-15);
        assert!(bound > 0.0 && bound < 1e-13);
        // the zero sum measures as (0, FLOOR)
        assert_eq!(centre_and_bound(&SqrtSum::zero()), Some((0.0, FLOOR)));
        // a coefficient below the smallest normal, and one too large for binary64, go to the exact path
        let tiny = sum(vec![Term { radicand: UBig::ONE, coef: Coef::fraction(Rat::new(IBig::ONE, IBig::ONE << 1100usize).unwrap()) }]);
        assert_eq!(centre_and_bound(&tiny), None);
        let huge = sum(vec![Term { radicand: UBig::ONE, coef: Coef::int(IBig::ONE << 1100usize) }]);
        assert_eq!(centre_and_bound(&huge), None);
    }

    #[test]
    fn orientation_sign_proves_clear_cases_and_yields_on_touching_ones() {
        let rational = |n: i64| SqrtSum::rational(&Rat::from_i64(n));
        let (zero, four, three, eight) = (rational(0), rational(4), rational(3), rational(8));
        let a = (&zero, &zero);
        let b = (&four, &zero);
        let c = (&zero, &three);
        assert_eq!(orientation_sign(a, b, c), Some(1));
        assert_eq!(orientation_sign(a, c, b), Some(-1));
        let collinear = (&eight, &zero);
        assert_eq!(orientation_sign(a, b, collinear), None);
        let irrational = sum(vec![term(2, 1, 1)]);
        let d = (&irrational, &zero);
        assert_eq!(orientation_sign(a, d, c), Some(1));
    }

    #[test]
    fn line_estimate_and_polygon_sign_agree_with_the_orientation() {
        let rational = |n: i64| SqrtSum::rational(&Rat::from_i64(n));
        let (x, y) = (rational(3), rational(5));
        let (value, bound) = line_estimate((&x, &y), 0.0, 0.0, 1.0, 0.0).unwrap();
        assert!((value - 5.0).abs() < 1e-12 && bound > 0.0, "step_x * (y - 0) - step_y * (x - 0)");
        let (zero, one, two) = (rational(0), rational(1), rational(2));
        let square = [(&zero, &zero), (&two, &zero), (&two, &two), (&zero, &two)];
        assert_eq!(polygon_sign(&square), Some(1));
        let reversed = [square[3], square[2], square[1], square[0]];
        assert_eq!(polygon_sign(&reversed), Some(-1));
        assert_eq!(polygon_sign(&square[..2]), None);
        let degenerate = [(&zero, &zero), (&one, &one), (&two, &two)];
        assert_eq!(polygon_sign(&degenerate), None);
    }

    #[test]
    fn the_affine_map_check_proves_violations_only() {
        let r = |n: i64| SqrtSum::rational(&Rat::from_i64(n));
        let (zero, one) = (r(0), r(1));
        let (two, five) = (r(2), r(5));
        // the map f = (x, y) on the base (0,0), (1,0), (0,1); the vertex (2, 2) with values (2, 2) lies on it
        let points = [(&zero, &zero), (&one, &zero), (&zero, &one), (&two, &two)];
        let on_map = [(&zero, &zero), (&one, &zero), (&zero, &one), (&two, &two)];
        assert!(!affine_map_violated(&points, &on_map));
        let off_map = [(&zero, &zero), (&one, &zero), (&zero, &one), (&five, &two)];
        assert!(affine_map_violated(&points, &off_map));
    }
}
