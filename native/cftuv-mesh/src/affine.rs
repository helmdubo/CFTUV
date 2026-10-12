//! `tessellate.py`: the exact tests of whether the UV of a contour is an affine function of the position on the chart
//! (`uv_is_affine_in_chart`, `affine_frame`, `uv_vertex_on_affine_map`) and how far from affine it is (`uv_affine_defect_milli`).
//!
//! The products and sums here are free (`SqrtSumV1.__mul__`, `__sub__`, `.is_zero` pay no budget); what costs is the base search by
//! `orientation`, which is asked in the oracle's order and stops at the first non-zero.

use cftuv_clip::faces::{orientation, Pt};
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::error::{MeshError, MeshResult};

/// The `(s, r)` of a vertex.
pub type Uv<'a> = (&'a SqrtSum, &'a SqrtSum);

/// `affine_frame(points, base)`: `(origin, (ux, uy, vx, vy), det)`.
pub struct Frame {
    origin: (SqrtSum, SqrtSum),
    ux: SqrtSum,
    uy: SqrtSum,
    vx: SqrtSum,
    vy: SqrtSum,
    det: SqrtSum,
}

pub fn affine_frame(ctx: &mut ExactCtx<'_>, points: &[Pt], base: [usize; 3]) -> Frame {
    let origin = points[base[0]];
    let ux = points[base[1]].0.sub(origin.0);
    let uy = points[base[1]].1.sub(origin.1);
    let vx = points[base[2]].0.sub(origin.0);
    let vy = points[base[2]].1.sub(origin.1);
    let det = ux.mul(&vy, ctx.products).sub(&uy.mul(&vx, ctx.products));
    Frame { origin: (origin.0.clone(), origin.1.clone()), ux, uy, vx, vy, det }
}

fn component<'a>(uv: Uv<'a>, which: usize) -> &'a SqrtSum {
    if which == 0 {
        uv.0
    } else {
        uv.1
    }
}

/// `uv_vertex_on_affine_map(points, values, base, frame, index)`: vertex `index` lies on the affine map solved by `base`, exactly.
pub fn uv_vertex_on_affine_map(ctx: &mut ExactCtx<'_>, points: &[Pt], values: &[Uv], base: [usize; 3], frame: &Frame, index: usize) -> bool {
    let qx = points[index].0.sub(&frame.origin.0);
    let qy = points[index].1.sub(&frame.origin.1);
    let first_weight = qx.mul(&frame.vy, ctx.products).sub(&qy.mul(&frame.vx, ctx.products));
    let second_weight = frame.ux.mul(&qy, ctx.products).sub(&frame.uy.mul(&qx, ctx.products));
    for which in 0..2 {
        let f0 = component(values[base[0]], which);
        let left = frame.det.mul(&component(values[index], which).sub(f0), ctx.products);
        let first = first_weight.mul(&component(values[base[1]], which).sub(f0), ctx.products);
        let second = second_weight.mul(&component(values[base[2]], which).sub(f0), ctx.products);
        let right = first.add(&second);
        if !left.sub(&right).is_zero() {
            return false;
        }
    }
    true
}

/// `uv_is_affine_in_chart(points, values, budget)`: the UV is an affine function of the chart position over the whole contour. All points on a
/// line is `False`.
pub fn uv_is_affine_in_chart(ctx: &mut ExactCtx<'_>, points: &[Pt], values: &[Uv]) -> MeshResult<bool> {
    let count = points.len();
    let mut base = None;
    'search: for second in 1..count {
        for third in second + 1..count {
            if orientation(ctx, points[0], points[second], points[third])? != 0 {
                base = Some([0, second, third]);
                break 'search;
            }
        }
    }
    let Some(base) = base else {
        return Ok(false);
    };
    let frame = affine_frame(ctx, points, base);
    for index in 1..count {
        if base.contains(&index) {
            continue;
        }
        if !uv_vertex_on_affine_map(ctx, points, values, base, &frame, index) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn abs_rat(value: &Rat) -> Rat {
    if value.signum() < 0 {
        value.neg()
    } else {
        value.clone()
    }
}

/// `uv_affine_defect_milli(points, values, unit, budget, bits=64)`: the largest deviation of the UV of the fourth vertex from the affine
/// continuation of three, in thousandths of `unit`, from strict enclosures rounded up. A contour on one line is zero.
pub fn uv_affine_defect_milli(ctx: &mut ExactCtx<'_>, points: &[Pt], values: &[Uv], unit: &Rat, bits: usize) -> MeshResult<IBig> {
    let count = points.len();
    let mut base = None;
    'search: for second in 1..count {
        for third in second + 1..count {
            if orientation(ctx, points[0], points[second], points[third])? != 0 {
                base = Some((second, third));
                break 'search;
            }
        }
    }
    let Some((second, third)) = base else {
        return Ok(IBig::ZERO);
    };
    let origin = points[0];
    let ux = points[second].0.sub(origin.0);
    let uy = points[second].1.sub(origin.1);
    let vx = points[third].0.sub(origin.0);
    let vy = points[third].1.sub(origin.1);
    let det = ux.mul(&vy, ctx.products).sub(&uy.mul(&vx, ctx.products));
    let (det_low, det_high) = det.enclosure(bits);
    let det_abs = abs_rat(&det_low).min(abs_rat(&det_high));
    if (det_low.signum() <= 0 && det_high.signum() >= 0) || det_abs.is_zero() {
        return Ok(IBig::ZERO);
    }
    let mut worst = Rat::zero();
    for index in 1..count {
        if index == second || index == third {
            continue;
        }
        let qx = points[index].0.sub(origin.0);
        let qy = points[index].1.sub(origin.1);
        let first_weight = qx.mul(&vy, ctx.products).sub(&qy.mul(&vx, ctx.products));
        let second_weight = ux.mul(&qy, ctx.products).sub(&uy.mul(&qx, ctx.products));
        for which in 0..2 {
            let f0 = component(values[0], which);
            let lhs = det.mul(&component(values[index], which).sub(f0), ctx.products);
            let first = first_weight.mul(&component(values[second], which).sub(f0), ctx.products);
            let second_part = second_weight.mul(&component(values[third], which).sub(f0), ctx.products);
            let residual = lhs.sub(&first.add(&second_part));
            let (low, high) = residual.enclosure(bits);
            let magnitude = abs_rat(&low).max(abs_rat(&high));
            let ratio = magnitude.div(&det_abs).map_err(|_| MeshError::Unsupported("a zero determinant bound".to_string()))?;
            if ratio > worst {
                worst = ratio;
            }
        }
    }
    // `-(-(worst * 1000) // Fraction(unit))`: the ceiling of `worst * 1000 / unit`
    let scaled = worst.mul(&Rat::from_i64(1000));
    let quotient = scaled.div(unit).map_err(|_| MeshError::Unsupported("a zero unit of the UV defect".to_string()))?;
    Ok(ceil_rat(&quotient))
}

/// `-(-n // d)` of a rational: its ceiling.
fn ceil_rat(value: &Rat) -> IBig {
    let denominator = IBig::from(value.denominator().clone());
    let (quotient, remainder) = dashu_int::ops::DivRemEuclid::div_rem_euclid(value.numerator().clone(), denominator);
    if remainder.is_zero() {
        quotient
    } else {
        quotient + IBig::ONE
    }
}
