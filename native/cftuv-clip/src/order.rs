//! `ClipStageV1._ordered`: the nodes of a segment from `first` to `second`, sorted by an exact comparison along the
//! first axis on which the ends differ. Every comparison is a `SqrtSumV1.sign()` (cost) asked in the order CPython's
//! `list.sort` asks it, which depends on the interpreter version (`pyemu`).

use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::ClipResult;
use crate::point::Point;
use crate::pyemu::{sort_by_less, PyVersion};

fn along(point: &Point, axis: usize) -> &SqrtSum {
    if axis == 0 {
        &point.0
    } else {
        &point.1
    }
}

/// `_ordered(first, second, nodes)`: the permutation (indices into `nodes`) that sorts them from `first` towards
/// `second`. The direction sign is asked first, even for one node; then one sign per comparison of the sort.
pub fn ordered(ctx: &mut ExactCtx<'_>, version: PyVersion, first: &Point, second: &Point, nodes: &[&Point]) -> ClipResult<Vec<usize>> {
    let axis = if !second.0.sub(&first.0).is_zero() { 0 } else { 1 };
    let direction = exact::sign(ctx, &along(second, axis).sub(along(first, axis)), SIGN_FILTER_BITS)?;
    sort_by_less(version, (0..nodes.len()).collect::<Vec<usize>>(), |left, right| {
        let sign = exact::sign(ctx, &along(nodes[*left], axis).sub(along(nodes[*right], axis)), SIGN_FILTER_BITS)?;
        Ok(i32::from(direction) * i32::from(sign) < 0)
    })
}
