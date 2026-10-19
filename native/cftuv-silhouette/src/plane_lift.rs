//! `lift.PlaneLiftV1.lift` (through `assemble.lift_vertices` on an exact plane): `origin + a * (X / scale) + b * (Y / scale)` per axis, exactly,
//! then one rounding per axis (`lift.sqrt_sum_binary64`, the midpoint of the 64-bit enclosure). No budget, no memory: a pure function.

use cftuv_clip::lift::sqrt_sum_binary64;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

/// The plane of a domain: `(origin, basis_a, basis_b)` as exact rationals and the lattice scale.
pub struct Plane {
    origin: [SqrtSum; 3],
    factor_a: [Rat; 3],
    factor_b: [Rat; 3],
}

impl Plane {
    /// `None` for a zero scale (the oracle's `Fraction` division would raise).
    pub fn new(origin: [Rat; 3], basis_a: [Rat; 3], basis_b: [Rat; 3], scale: &Rat) -> Option<Plane> {
        let divide = |value: &Rat| value.div(scale).ok();
        Some(Plane {
            origin: [SqrtSum::rational(&origin[0]), SqrtSum::rational(&origin[1]), SqrtSum::rational(&origin[2])],
            factor_a: [divide(&basis_a[0])?, divide(&basis_a[1])?, divide(&basis_a[2])?],
            factor_b: [divide(&basis_b[0])?, divide(&basis_b[1])?, divide(&basis_b[2])?],
        })
    }

    /// The lifted position of one chart point; `None` when a coordinate is not a finite binary64 (`LocalPoint3V1` would raise).
    pub fn lift(&self, x: &SqrtSum, y: &SqrtSum) -> Option<[f64; 3]> {
        let mut found = [0.0f64; 3];
        for axis in 0..3 {
            let value = x.scaled(&self.factor_a[axis]).add(&y.scaled(&self.factor_b[axis])).add(&self.origin[axis]);
            let coordinate = sqrt_sum_binary64(&value).ok()?;
            if !coordinate.is_finite() {
                return None;
            }
            found[axis] = coordinate;
        }
        Some(found)
    }
}
