//! The lift of a clip vertex in its source triangle: `BoundSurfaceLiftV1.lift_in` / `lift_known`, `lift.sqrt_sum_binary64`,
//! `offset_normal.blend` and the finiteness check of `LocalPoint3V1`.
//!
//! `lift_known` WRITES `plane._normal_by_position[(x, y, z)] = normal` in the oracle; here the write is the RETURNED
//! `(position, normal)` of [`Lifted`] (when the triangle has normals): the caller applies it, in call order, so a call
//! that fails leaves nothing behind, as in Python where the write is the last statement. The oracle's dictionary has
//! float-tuple keys: `-0.0` and `0.0` are one key and the first spelling stays; the caller must keep that.

use cftuv_core::sqrt_sum::SqrtSum;

use crate::error::{ClipError, ClipResult, BLEND_ZERO_DETAIL, BLEND_ZERO_OUTCOME, FLOAT_DIVISION_BY_ZERO, NON_FINITE_POINT};
use crate::numeric::{float_of, float_of_ratio, ENCLOSURE_BITS};
use crate::plane::{lift_factors, LiftFactors, Triangle};
use crate::cpython311::left_fold_sum;

/// What `lift_known` answers: the position, the triangle name and the offset normal. `(position, normal)` is also the
/// entry the oracle writes into `plane._normal_by_position`, exactly when the triangle carries normals.
#[derive(Clone, Debug, PartialEq)]
pub struct Lifted {
    pub position: [f64; 3],
    pub triangle: String,
    pub normal: Option<[f64; 3]>,
}

/// `lift_in(triangle, values)`: `(e1*A + e2*B + e0*C) / D` per axis; `values` are the three edge orientation values.
pub fn lift_in(triangle: &Triangle, values: &[SqrtSum; 3]) -> ClipResult<[SqrtSum; 3]> {
    lift_in_with(&lift_factors(triangle)?, [&values[0], &values[1], &values[2]])
}

/// [`lift_in`] with the factors `corner / twice_area` of the triangle already at hand (they are a function of the triangle:
/// the plane keeps them). `a.scaled(f) + b.scaled(g) + c.scaled(h)` is one weighted sum with one normalisation per term.
pub fn lift_in_with(factors: &LiftFactors, values: [&SqrtSum; 3]) -> ClipResult<[SqrtSum; 3]> {
    let weights = [values[1], values[2], values[0]];
    let axis = |index: usize| SqrtSum::scaled_sum(&[(weights[0], &factors[0][index]), (weights[1], &factors[1][index]), (weights[2], &factors[2][index])]);
    Ok([axis(0), axis(1), axis(2)])
}

/// `lift.sqrt_sum_binary64(value)`: the midpoint of the 64-bit enclosure as a binary64, one rounding. The midpoint is built from
/// the unreduced endpoints: `float(Fraction)` rounds the exact ratio, which is the same however it is written.
pub fn sqrt_sum_binary64(value: &SqrtSum) -> ClipResult<f64> {
    let (low, high, denominator) = value.enclosure_parts(ENCLOSURE_BITS);
    float_of_ratio(&(low + high), &(denominator << 1usize))
}

fn dot(left: &[f64; 3], right: &[f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

/// `offset_normal.blend(weights, normals)`: the normalized barycentric mix of the three corner normals. The three-term
/// sums are the kernel's `left_fold_sum`: a left fold from the `int` zero, without compensation.
pub fn blend(weights: &[f64; 3], normals: &[[f64; 3]; 3]) -> ClipResult<[f64; 3]> {
    let mut mixed = [0.0f64; 3];
    for axis in 0..3 {
        let terms = [weights[0] * normals[0][axis], weights[1] * normals[1][axis], weights[2] * normals[2][axis]];
        mixed[axis] = left_fold_sum(&terms);
    }
    let length = dot(&mixed, &mixed).sqrt();
    // `not _length(mixed)`: zero (either sign) is a refusal, NaN is not
    if length == 0.0 {
        return Err(ClipError::Refusal { outcome: BLEND_ZERO_OUTCOME, detail: BLEND_ZERO_DETAIL.to_string() });
    }
    Ok([mixed[0] / length, mixed[1] / length, mixed[2] / length])
}

/// `BoundSurfaceLiftV1.lift_known(triangle, values)`: the lifted position (finite binary64s), the triangle name and the
/// offset normal when the triangle has normals. The caller writes `normal` at `position` into the normal table.
pub fn lift_known(triangle: &Triangle, values: &[SqrtSum; 3]) -> ClipResult<Lifted> {
    lift_known_with(triangle, &lift_factors(triangle)?, [&values[0], &values[1], &values[2]])
}

/// [`lift_known`] with the factors of the triangle at hand (the plane caches them).
pub fn lift_known_with(triangle: &Triangle, factors: &LiftFactors, values: [&SqrtSum; 3]) -> ClipResult<Lifted> {
    let [x, y, z] = lift_in_with(factors, values)?;
    let position = [sqrt_sum_binary64(&x)?, sqrt_sum_binary64(&y)?, sqrt_sum_binary64(&z)?];
    if !position.iter().all(|axis| axis.is_finite()) {
        return Err(ClipError::Value(NON_FINITE_POINT));
    }
    let mut normal = None;
    if let Some(corner_normals) = &triangle.normals {
        let divisor = float_of(&triangle.twice_area)?;
        let mut weights = [0.0f64; 3];
        for (slot, index) in [1usize, 2, 0].into_iter().enumerate() {
            let converted = sqrt_sum_binary64(&values[index])?;
            if divisor == 0.0 {
                return Err(ClipError::ZeroDivision(FLOAT_DIVISION_BY_ZERO));
            }
            weights[slot] = converted / divisor;
        }
        normal = Some(blend(&weights, corner_normals)?);
    }
    Ok(Lifted { position, triangle: triangle.name.clone(), normal })
}
