//! The binary64 prefilter "no contacts" of `SourceContactFrame` (`boundary_native.py`: `excludes`, `_station_and_alpha`) and the measure it stands on (`float_filter.py`: `_measure`).
//!
//! The filter can only PROVE emptiness (both ends of the barrier segment lie strictly on the inadmissible side of one of three lines, in binary64 with an error bound); a zero, a touch, a number
//! outside binary64, anything not proved, goes to the exact leaf. It adds no tolerance: it does not change the answer, only decides whether to pay for it. The operations and their order are
//! the Python expressions' (Rust never fuses a multiply and an add, `abs`, `+ - *` and `sqrt` are IEEE), so the float proof of Rust is the proof of Python bit for bit, and so are the two counters
//! (`prefilter_rejected`, `prefilter_passed`) that move with it.
//!
//! The frame's own seven `(centre, bound)` pairs (source start, the two covectors, the length) come from Python as they are (`SourceContactFrame._float_parts`); the barrier ends are
//! rational here (a rational coordinate is `RadicalSumV1.rational(value)`: no terms for zero, one term of radicand 1 otherwise), and their measure is computed.

use cftuv_core::pyfloat::rat_to_f64;
use cftuv_core::rat::Rat;

/// `_SLACK = 2.0 ** -52`.
const SLACK: f64 = f64::EPSILON;
/// `_FLOOR = float_info.min`.
const FLOOR: f64 = f64::MIN_POSITIVE;
/// `_MARGIN = 1.0 + 1e-9`.
const MARGIN: f64 = 1.0 + 1e-9;

/// `(centre, bound)`: `|value - centre| <= bound`.
pub type Entry = (f64, f64);

/// `float_filter._measure` of a rational value; `None` is `centre_and_bound(...) is None` (binary64 does not take the value).
pub fn rational_entry(value: &Rat) -> Option<Entry> {
    let mut total = 0.0f64;
    let mut weight = 0.0f64;
    let mut count = 0u32;
    if !value.is_zero() {
        // OverflowError of `float(Fraction)` sends the value to the exact path
        let centre = rat_to_f64(value).ok()?;
        // a coefficient below the smallest normal has an absolute (not relative) conversion error
        if centre.abs() < FLOOR {
            return None;
        }
        // `math.sqrt(1)` is exactly 1.0
        let term = centre * 1.0f64;
        total += term;
        weight += term.abs();
        count = 1;
    }
    let bound = (count + 6) as f64 * SLACK * weight + FLOOR;
    // `bound - bound == 0.0 and total - total == 0.0`
    if bound.is_finite() && total.is_finite() {
        Some((total, bound))
    } else {
        None
    }
}

/// `(station, its bound, alpha, its bound)` of a point in binary64, or `None`: `_station_and_alpha`.
fn station_and_alpha(floats: &[Entry; 7], point: [&Rat; 2]) -> Option<(f64, f64, f64, f64)> {
    let [(sx, sx_error), (sy, sy_error), (gx, gx_error), (gy, gy_error), (hx, hx_error), (hy, hy_error), _] = *floats;
    let entry_x = rational_entry(point[0])?;
    let entry_y = rational_entry(point[1])?;
    let dx = entry_x.0 - sx;
    let dx_error = entry_x.1 + sx_error + SLACK * dx.abs();
    let dy = entry_y.0 - sy;
    let dy_error = entry_y.1 + sy_error + SLACK * dy.abs();
    let left = dx * gx;
    let right = dy * gy;
    let station = left + right;
    let station_error = (dx.abs() * gx_error + gx.abs() * dx_error + dx_error * gx_error + SLACK * left.abs() + FLOOR + dy.abs() * gy_error + gy.abs() * dy_error + dy_error * gy_error + SLACK * right.abs() + FLOOR + SLACK * station.abs()) * MARGIN;
    let left = dx * hx;
    let right = dy * hy;
    let alpha = left + right;
    let alpha_error = (dx.abs() * hx_error + hx.abs() * dx_error + dx_error * hx_error + SLACK * left.abs() + FLOOR + dy.abs() * hy_error + hy.abs() * dy_error + dy_error * hy_error + SLACK * right.abs() + FLOOR + SLACK * alpha.abs()) * MARGIN;
    Some((station, station_error, alpha, alpha_error))
}

/// `SourceContactFrame.excludes(boundary)` for a barrier segment with rational ends: `true` ONLY when emptiness is proved.
pub fn excludes(floats: &[Entry; 7], first: [&Rat; 2], second: [&Rat; 2]) -> bool {
    let Some((station0, station_error0, alpha0, alpha_error0)) = station_and_alpha(floats, first) else {
        return false;
    };
    let Some((station1, station_error1, alpha1, alpha_error1)) = station_and_alpha(floats, second) else {
        return false;
    };
    if alpha0 < -alpha_error0 && alpha1 < -alpha_error1 {
        return true;
    }
    if station0 < -station_error0 && station1 < -station_error1 {
        return true;
    }
    let (length, length_error) = floats[6];
    let beyond0 = station0 - length;
    let beyond1 = station1 - length;
    beyond0 > (station_error0 + length_error + SLACK * beyond0.abs()) * MARGIN && beyond1 > (station_error1 + length_error + SLACK * beyond1.abs()) * MARGIN
}

#[cfg(test)]
mod tests {
    use super::*;
    use cftuv_core::num::IBig;

    fn q(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    #[test]
    fn the_measure_of_a_rational_is_the_python_measure() {
        // zero: no terms, `(0 + 6) * 2^-52 * 0.0 + float_info.min`
        assert_eq!(rational_entry(&Rat::zero()), Some((0.0, FLOOR)));
        // 1/3: centre float(Fraction(1, 3)), bound (1 + 6) * 2^-52 * |centre| + float_info.min
        let (centre, bound) = rational_entry(&q(1, 3)).unwrap();
        assert_eq!(centre.to_bits(), (1.0f64 / 3.0).to_bits());
        assert_eq!(bound.to_bits(), (7.0 * f64::EPSILON * (1.0f64 / 3.0) + f64::MIN_POSITIVE).to_bits());
        // a coefficient below the smallest normal, and one beyond binary64, are not taken
        let tiny = Rat::new(IBig::ONE, IBig::ONE << 1100usize).unwrap();
        assert_eq!(rational_entry(&tiny), None);
        let huge = Rat::from_int(IBig::ONE << 1100usize);
        assert_eq!(rational_entry(&huge), None);
    }

    #[test]
    fn both_ends_behind_the_alpha_line_are_proved_empty_and_a_touch_is_not() {
        // source at the origin, station direction (1, 0), alpha direction (0, 1), length 1
        let one = (1.0, SLACK);
        let zero = (0.0, FLOOR);
        let floats = [zero, zero, one, zero, zero, one, one];
        let below = q(-3, 1);
        assert!(excludes(&floats, [&q(0, 1), &below], [&q(1, 2), &below]));
        // one end on the line: not proved
        assert!(!excludes(&floats, [&q(0, 1), &below], [&q(1, 2), &q(0, 1)]));
        // both ends past the far end of the source
        assert!(excludes(&floats, [&q(5, 1), &q(1, 1)], [&q(7, 1), &q(2, 1)]));
        assert!(!excludes(&floats, [&q(1, 2), &q(1, 1)], [&q(7, 1), &q(2, 1)]));
    }
}
