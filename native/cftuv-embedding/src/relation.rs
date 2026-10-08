//! `_embedding._segment_relation3`: the exact relation of two closed 3D segments, over integers (`num.rs`).
//!
//! A line-by-line port, quirks included: a segment of length zero on the left (`a == b`) meets another segment only at its two endpoints (`a == c or a == d`), so a point in the
//! INTERIOR of the other segment is `NONE`, while the same pair the other way round (`c == d` on the left of `ab`) is `POINT`. The answer is only ever compared with `NONE`
//! by the certificate, but the three values are kept so the unit tests can hold the function to the oracle's.

use crate::num::Num;

pub const NONE: u8 = 0;
pub const POINT: u8 = 1;
pub const OVERLAP: u8 = 2;

pub type Point<N> = [N; 3];

fn sub3<N: Num>(left: &Point<N>, right: &Point<N>) -> Point<N> {
    [left[0].sub(&right[0]), left[1].sub(&right[1]), left[2].sub(&right[2])]
}

/// One component of the cross product, as `_cross3` computes it.
fn cross_component<N: Num>(left: &Point<N>, right: &Point<N>, axis: usize) -> N {
    let (i, j) = match axis {
        0 => (1, 2),
        1 => (2, 0),
        _ => (0, 1),
    };
    left[i].mul(&right[j]).sub(&left[j].mul(&right[i]))
}

fn cross3<N: Num>(left: &Point<N>, right: &Point<N>) -> Point<N> {
    [cross_component(left, right, 0), cross_component(left, right, 1), cross_component(left, right, 2)]
}

fn any_nonzero<N: Num>(vector: &Point<N>) -> bool {
    vector.iter().any(|item| !item.is_zero())
}

fn first_nonzero<N: Num>(vector: &Point<N>) -> Option<usize> {
    vector.iter().position(|item| !item.is_zero())
}

fn dot3<N: Num>(left: &Point<N>, right: &Point<N>) -> N {
    left[0].mul(&right[0]).add(&left[1].mul(&right[1])).add(&left[2].mul(&right[2]))
}

/// `0 <= numerator / denominator <= 1` for a non-zero denominator, without dividing.
fn in_unit_interval<N: Num>(numerator: &N, denominator: &N) -> bool {
    if denominator.sign() > 0 {
        numerator.sign() >= 0 && numerator <= denominator
    } else {
        numerator.sign() <= 0 && numerator >= denominator
    }
}

/// `_segment_relation3(a, b, c, d)`.
pub fn relation3<N: Num>(a: &Point<N>, b: &Point<N>, c: &Point<N>, d: &Point<N>) -> u8 {
    let r = sub3(b, a);
    let s = sub3(d, c);
    let r_cross_s = cross3(&r, &s);
    let offset = sub3(c, a);
    if !any_nonzero(&r_cross_s) {
        if any_nonzero(&cross3(&offset, &r)) {
            return NONE;
        }
        let Some(axis) = first_nonzero(&r) else {
            return if a == c || a == d { POINT } else { NONE };
        };
        let (left_low, left_high) = if a[axis] <= b[axis] { (&a[axis], &b[axis]) } else { (&b[axis], &a[axis]) };
        let (right_low, right_high) = if c[axis] <= d[axis] { (&c[axis], &d[axis]) } else { (&d[axis], &c[axis]) };
        let low = if left_low >= right_low { left_low } else { right_low };
        let high = if left_high <= right_high { left_high } else { right_high };
        return match low.compare(high) {
            std::cmp::Ordering::Greater => NONE,
            std::cmp::Ordering::Equal => POINT,
            std::cmp::Ordering::Less => OVERLAP,
        };
    }
    if !dot3(&offset, &r_cross_s).is_zero() {
        return NONE;
    }
    let axis = first_nonzero(&r_cross_s).expect("the cross product is not zero here");
    let denominator = &r_cross_s[axis];
    let t = cross_component(&offset, &s, axis);
    let u = cross_component(&offset, &r, axis);
    if in_unit_interval(&t, denominator) && in_unit_interval(&u, denominator) {
        POINT
    } else {
        NONE
    }
}

/// The axis-aligned box of a segment: two boxes that are apart on one axis cannot hold segments that meet, so `relation3` of those segments is `NONE` in every branch above
/// (a shared point of the two segments is a point of both boxes; the zero-length quirk only narrows `POINT`).
#[derive(Clone)]
pub struct Bounds<N: Num> {
    low: Point<N>,
    high: Point<N>,
}

impl<N: Num> Bounds<N> {
    pub fn of(a: &Point<N>, b: &Point<N>) -> Bounds<N> {
        let pick = |k: usize| if a[k] <= b[k] { (a[k].clone(), b[k].clone()) } else { (b[k].clone(), a[k].clone()) };
        let (x, y, z) = (pick(0), pick(1), pick(2));
        Bounds { low: [x.0, y.0, z.0], high: [x.1, y.1, z.1] }
    }

    pub fn apart_from(&self, other: &Bounds<N>) -> bool {
        (0..3).any(|k| self.high[k] < other.low[k] || other.high[k] < self.low[k])
    }
}

/// The differences and cross product `_corner_degenerated` tests: `True` when `previous - vertex` and `following - vertex` are parallel (or one is zero).
pub fn corner_is_flat<N: Num>(previous: &Point<N>, vertex: &Point<N>, following: &Point<N>) -> bool {
    !any_nonzero(&cross3(&sub3(previous, vertex), &sub3(following, vertex)))
}
