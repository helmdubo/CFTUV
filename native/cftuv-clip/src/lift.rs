//! The lift of a clip vertex in its source triangle: `BoundSurfaceLiftV1.lift_in` / `lift_known`, `lift.sqrt_sum_binary64`,
//! `offset_normal.blend` and the finiteness check of `LocalPoint3V1`.
//!
//! `lift_known` WRITES `plane._normal_by_position[(x, y, z)] = normal` in the oracle; here the write is the RETURNED
//! `(position, normal)` of [`Lifted`] (when the triangle has normals): the caller applies it, in call order, so a call
//! that fails leaves nothing behind, as in Python where the write is the last statement. The oracle's dictionary has
//! float-tuple keys: `-0.0` and `0.0` are one key and the first spelling stays; the caller must keep that.

use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::error::{ClipError, ClipResult, BLEND_ZERO_DETAIL, BLEND_ZERO_OUTCOME, FLOAT_DIVISION_BY_ZERO, NON_FINITE_POINT};
use crate::numeric::{float_of, ENCLOSURE_BITS};
use crate::plane::Triangle;
use crate::pyemu::{float_sum, PyVersion};

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
    let weights = [&values[1], &values[2], &values[0]];
    let mut out = Vec::with_capacity(3);
    for axis in 0..3 {
        let mut total = SqrtSum::zero();
        for (position, weight) in weights.iter().enumerate() {
            let factor = triangle.corners[position][axis].div(&triangle.twice_area).map_err(|_| ClipError::Value("a degenerate source triangle"))?;
            let term = weight.scaled(&factor);
            // `a.scaled() + b.scaled() + c.scaled()` is `(a + b) + c`; the first term stands alone
            total = if position == 0 { term } else { total.add(&term) };
        }
        out.push(total);
    }
    let [x, y, z]: [SqrtSum; 3] = out.try_into().expect("three axes");
    Ok([x, y, z])
}

/// `lift.sqrt_sum_binary64(value)`: the midpoint of the 64-bit enclosure as a binary64, one rounding.
pub fn sqrt_sum_binary64(value: &SqrtSum) -> ClipResult<f64> {
    let (low, high) = value.enclosure(ENCLOSURE_BITS);
    let midpoint = low.add(&high).div(&Rat::from_i64(2)).expect("two is not zero");
    float_of(&midpoint)
}

fn dot(left: &[f64; 3], right: &[f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

/// `offset_normal.blend(weights, normals)`: the normalized barycentric mix of the three corner normals. The three-term
/// sums are Python's `sum()`, hence the interpreter version.
pub fn blend(version: PyVersion, weights: &[f64; 3], normals: &[[f64; 3]; 3]) -> ClipResult<[f64; 3]> {
    let mut mixed = [0.0f64; 3];
    for axis in 0..3 {
        let terms = [weights[0] * normals[0][axis], weights[1] * normals[1][axis], weights[2] * normals[2][axis]];
        mixed[axis] = float_sum(version, &terms);
    }
    let length = dot(&mixed, &mixed).sqrt();
    // `not _length(mixed)`: zero (either sign) is a refusal, NaN is not
    if length == 0.0 {
        return Err(ClipError::Refusal { outcome: BLEND_ZERO_OUTCOME, detail: BLEND_ZERO_DETAIL });
    }
    Ok([mixed[0] / length, mixed[1] / length, mixed[2] / length])
}

/// `BoundSurfaceLiftV1.lift_known(triangle, values)`: the lifted position (finite binary64s), the triangle name and the
/// offset normal when the triangle has normals. The caller writes `normal` at `position` into the normal table.
pub fn lift_known(version: PyVersion, triangle: &Triangle, values: &[SqrtSum; 3]) -> ClipResult<Lifted> {
    let [x, y, z] = lift_in(triangle, values)?;
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
        normal = Some(blend(version, &weights, corner_normals)?);
    }
    Ok(Lifted { position, triangle: triangle.name.clone(), normal })
}
