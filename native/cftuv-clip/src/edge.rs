//! The cheap sign of a point against an edge of a cut region (`ClipStageV1._edge_constants` and `_cheap_sign`): integer
//! arithmetic for a rational point, the binary64 line filter for any other. Every decision must be bit-identical to
//! the oracle's: a 1-ulp difference changes WHICH signs go through `SqrtSumV1.sign()` (and so the cost) and the
//! tolerance zeroing rule of law 2.

use cftuv_core::float_filter::line_estimate;
use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;

use crate::numeric::true_divide;
use crate::plane::ChartPoint;
use crate::point::{rational_pair, Point, RationalPair};

/// `1 + 1e-9`: the margin of the float estimate of the tolerance.
pub const FILTER_MARGIN: f64 = 1.0 + 1e-9;
/// Integer chart coordinates below this are exact in binary64 together with their differences.
pub const FILTER_COORDINATE_LIMIT: i64 = 1 << 40;
/// `float(NODE_EDGE_SNAP_CELLS * NODE_EDGE_SNAP_CELLS)`: the squared tolerance of law 2 in cells.
pub const GAP_CELLS_SQUARE: f64 = 1.0;

/// `_edge_constants(ti, index)`: the integer start and step of an edge, their floats, and the tolerance of law 2
/// (`sqrt(cells^2 * |edge|^2)`, float).
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeConstants {
    pub x0: i64,
    pub y0: i64,
    pub dx: i64,
    pub dy: i64,
    pub fx0: f64,
    pub fy0: f64,
    pub fdx: f64,
    pub fdy: f64,
    pub tolerance: f64,
}

/// `_edge_constants` of the edge `index` of a chart loop; `None` when the ends are not integers or too large (the
/// float estimate is not exact in its constants then, and the exact path decides).
pub fn edge_constants(chart: &[ChartPoint], index: usize) -> Option<EdgeConstants> {
    let (start, end) = (&chart[index], &chart[(index + 1) % chart.len()]);
    let (x0, y0, x1, y1) = (&start.0, &start.1, &end.0, &end.1);
    if !(x0.is_integer() && y0.is_integer() && x1.is_integer() && y1.is_integer()) {
        return None;
    }
    let limit = IBig::from(FILTER_COORDINATE_LIMIT);
    let biggest = [x0, y0, x1, y1].into_iter().map(|value| num::magnitude(value.numerator())).max().expect("four values");
    if IBig::from(biggest) >= limit {
        return None;
    }
    let small = |value: &Rat| -> i64 { i64::try_from(value.numerator()).expect("below 2^40") };
    let (x0, y0) = (small(x0), small(y0));
    let (dx, dy) = (small(x1) - x0, small(y1) - y0);
    // `float(dx*dx + dy*dy)`: the integer is below 2^83, `as f64` rounds half to even like `PyLong_AsDouble`
    let squared = i128::from(dx) * i128::from(dx) + i128::from(dy) * i128::from(dy);
    let tolerance = (GAP_CELLS_SQUARE * (squared as f64)).sqrt();
    Some(EdgeConstants { x0, y0, dx, dy, fx0: x0 as f64, fy0: y0 as f64, fdx: dx as f64, fdy: dy as f64, tolerance })
}

/// `_cheap_sign(node, constants, watch)`: `(sign or None, farther than the tolerance)`. The sign is EXACT whenever it
/// is not `None`: integer arithmetic for a rational point (zero included), a proof by the float error bound for any
/// other (zero is never proved: `None`). `farther` is proved only for a watched node.
pub fn cheap_sign(point: &Point, constants: &EdgeConstants, watch: bool) -> (Option<i8>, bool) {
    cheap_sign_with(point, rational_pair(point).as_ref(), constants, watch)
}

/// [`cheap_sign`] with the integer record of the point (`_Node.rational`) already at hand: the stage keeps it on its nodes.
pub fn cheap_sign_with(point: &Point, rational: Option<&RationalPair>, constants: &EdgeConstants, watch: bool) -> (Option<i8>, bool) {
    if let Some((x_numerator, x_denominator, y_numerator, y_denominator)) = rational {
        let (x0, y0) = (IBig::from(constants.x0), IBig::from(constants.y0));
        let (dx, dy) = (IBig::from(constants.dx), IBig::from(constants.dy));
        let x_den = IBig::from(x_denominator.clone());
        let y_den = IBig::from(y_denominator.clone());
        let numerator = &dx * (y_numerator - &y0 * &y_den) * &x_den - &dy * (x_numerator - &x0 * &x_den) * &y_den;
        let sign = num::signum(&numerator);
        if sign == 0 || !watch {
            return (Some(sign), false);
        }
        let magnitude = num::magnitude(&numerator);
        let denominator: UBig = x_denominator * y_denominator;
        // `abs(numerator) / (x_denominator * y_denominator)`: an overflow means binary64 cannot take the value
        let far = match true_divide(&IBig::from(magnitude), &denominator) {
            Ok(ratio) => ratio > constants.tolerance * FILTER_MARGIN,
            Err(_) => false,
        };
        return (Some(sign), far);
    }
    let Some((value, bound)) = line_estimate((&point.0, &point.1), constants.fx0, constants.fy0, constants.fdx, constants.fdy) else {
        return (None, false);
    };
    let magnitude = value.abs();
    if magnitude <= bound {
        return (None, false);
    }
    let sign = if value > 0.0 { 1 } else { -1 };
    (Some(sign), watch && magnitude > (bound + constants.tolerance) * FILTER_MARGIN)
}
