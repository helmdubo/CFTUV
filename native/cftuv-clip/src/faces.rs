//! The orientation family of `wavefront/faces.py` the clip stage calls: `orientation`, `shoelace_sign`,
//! `doubled_shoelace`. Each asks the binary64 filter first and goes to the exact `SqrtSum` path only when the filter
//! cannot prove the sign, so the exact signs (which move `SIGN_COUNTS`, may pay budget and may factor) are the same
//! ones the oracle asks, in the same order.

use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::float_filter;
use cftuv_core::fused::sum_of_products;
use cftuv_core::num::IBig;
use cftuv_core::products::ProductMemo;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::ClipResult;

/// A point of the plane: two sqrt-sum coordinates (borrowed).
pub type Pt<'a> = (&'a SqrtSum, &'a SqrtSum);

/// `faces.orientation(first, second, third, budget)`: the sign of `(b - a) x (c - a)`.
pub fn orientation(ctx: &mut ExactCtx<'_>, first: Pt, second: Pt, third: Pt) -> ClipResult<i8> {
    if let Some(decided) = float_filter::orientation_sign(first, second, third) {
        return Ok(decided);
    }
    let left = second.0.sub(first.0).mul(&third.1.sub(first.1), ctx.products);
    let right = second.1.sub(first.1).mul(&third.0.sub(first.0), ctx.products);
    Ok(exact::sign(ctx, &left.sub(&right), SIGN_FILTER_BITS)?)
}

/// `faces.shoelace_sign(points, budget)`: the sign of the doubled area, the filter first (three points or more).
pub fn shoelace_sign(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> ClipResult<i8> {
    if let Some(decided) = float_filter::polygon_sign(points) {
        return Ok(decided);
    }
    let area = doubled_shoelace(ctx.products, points);
    Ok(exact::sign(ctx, &area, SIGN_FILTER_BITS)?)
}

/// `faces.doubled_shoelace(points)`: the doubled oriented area. From three points a fan from the first
/// (`sum (p_i - p_0) x (p_i+1 - p_0)`, one fused pass); fewer points the trapezoid sum.
pub fn doubled_shoelace(products: &mut ProductMemo, points: &[Pt]) -> SqrtSum {
    let size = points.len();
    if size >= 3 {
        let (origin_x, origin_y) = points[0];
        let mut previous_x = points[1].0.sub(origin_x);
        let mut previous_y = points[1].1.sub(origin_y);
        let mut differences: Vec<(SqrtSum, SqrtSum, SqrtSum, SqrtSum)> = Vec::with_capacity(size - 2);
        for point in &points[2..] {
            let next_x = point.0.sub(origin_x);
            let next_y = point.1.sub(origin_y);
            differences.push((previous_x.clone(), next_y.clone(), previous_y.clone(), next_x.clone()));
            previous_x = next_x;
            previous_y = next_y;
        }
        let mut terms: Vec<(&SqrtSum, &SqrtSum, IBig)> = Vec::with_capacity(2 * differences.len());
        for (px, ny, py, nx) in &differences {
            terms.push((px, ny, IBig::ONE));
            terms.push((py, nx, -IBig::ONE));
        }
        return sum_of_products(&terms, products);
    }
    let mut total = SqrtSum::zero();
    for index in 0..size {
        let (x0, y0) = points[index];
        let (x1, y1) = points[(index + 1) % size];
        total = total.add(&x0.mul(y1, products)).sub(&x1.mul(y0, products));
    }
    total
}
