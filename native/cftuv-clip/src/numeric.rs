//! Small exact and binary64 helpers of the clip stage: ceilings of square roots (`nanometres`, `milli_cells`,
//! `_upper_root`), the outward-rounded float window of a point, CPython float conversions with their named overflow.
//!
//! Everything here is a pure function of its operands (no cost): the exact parts are integer arithmetic, the float
//! parts follow CPython bit for bit (`float(Fraction)` is a correctly rounded `int / int`, `math.nextafter`).

use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::pyfloat::{self, ratio_to_f64};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::error::{fraction_overflow, ClipError, ClipResult};

/// The bits of the strict enclosure every float conversion here reads (`lift.ENCLOSURE_BITS`).
pub const ENCLOSURE_BITS: usize = 64;

/// `float(Fraction)`: a correctly rounded true division, `OverflowError` with the division text.
pub fn float_of(value: &Rat) -> ClipResult<f64> {
    pyfloat::rat_to_f64(value).map_err(fraction_overflow)
}

/// `-(-numerator // denominator)`: the ceiling of a quotient, for any sign.
pub fn ceil_div(numerator: &IBig, denominator: &UBig) -> IBig {
    use cftuv_core::num::IBig as Int;
    let divisor = Int::from(denominator.clone());
    let (quotient, remainder) = dashu_int::ops::DivRemEuclid::div_rem_euclid(numerator.clone(), divisor);
    if remainder.is_zero() {
        quotient
    } else {
        quotient + IBig::ONE
    }
}

/// `ceil(sqrt(value * factor))`, exactly: `whole = ceil(value * factor)`, `root = isqrt(whole)`, `root` when it is
/// exact, else `root + 1` (a negative `whole` is `ValueError` in `math.isqrt`).
pub fn ceil_root_scaled(value: &Rat, factor: &UBig) -> ClipResult<UBig> {
    let scaled = value.mul(&Rat::from_int(IBig::from(factor.clone())));
    let whole = ceil_div(scaled.numerator(), scaled.denominator());
    if num::is_negative(&whole) {
        return Err(ClipError::Value("isqrt() argument must be nonnegative"));
    }
    let whole = num::magnitude(&whole);
    let root = num::isqrt(&whole);
    Ok(if &root * &root == whole { root } else { root + UBig::ONE })
}

fn power_of_ten(exponent: u32) -> UBig {
    UBig::from(10u8).pow(exponent as usize)
}

/// `clip_cells.nanometres`: the integer upper bound of a depth in nanometres from its square in metres.
pub fn nanometres(depth_square: &Rat) -> ClipResult<UBig> {
    ceil_root_scaled(depth_square, &power_of_ten(18))
}

/// `clip_snap.milli_cells`: the integer upper bound of a distance in thousandths of a cell from its square in cells.
pub fn milli_cells(square: &Rat) -> ClipResult<UBig> {
    ceil_root_scaled(square, &UBig::from(1_000_000u32))
}

/// `lift_surface._upper_root`: a rational upper bound of `sqrt(value)` with relative precision about `2^-24`.
pub fn upper_root(value: &Rat) -> ClipResult<Rat> {
    let scale = UBig::ONE << 24usize;
    let scaled = value.mul(&Rat::from_int(IBig::from(&scale * &scale)));
    let whole = ceil_div(scaled.numerator(), scaled.denominator());
    if num::is_negative(&whole) {
        return Err(ClipError::Value("isqrt() argument must be nonnegative"));
    }
    let whole = num::magnitude(&whole);
    let root = num::isqrt(&whole);
    let root = if &root * &root == whole { root } else { root + UBig::ONE };
    Ok(Rat::reduced(IBig::from(root), scale))
}

/// `lift_surface._down`: `math.nextafter(float(value), -inf)`.
pub fn float_down(value: &Rat) -> ClipResult<f64> {
    Ok(float_of(value)?.next_down())
}

/// `lift_surface._up`: `math.nextafter(float(value), inf)`.
pub fn float_up(value: &Rat) -> ClipResult<f64> {
    Ok(float_of(value)?.next_up())
}

/// `BoundSurfaceLiftV1.window(point)`: the outward-rounded box `(xmin, xmax, ymin, ymax)` of a point from the strict
/// enclosures of its coordinates. Evaluated in the oracle's order (the first overflow wins).
pub fn window(x: &SqrtSum, y: &SqrtSum) -> ClipResult<[f64; 4]> {
    let (x_low, x_high) = x.enclosure(ENCLOSURE_BITS);
    let (y_low, y_high) = y.enclosure(ENCLOSURE_BITS);
    let x_min = float_down(&x_low)?;
    let x_max = float_up(&x_high)?;
    let y_min = float_down(&y_low)?;
    let y_max = float_up(&y_high)?;
    Ok([x_min, x_max, y_min, y_max])
}

/// `int(value) / int(1)` for a lattice integer given as `i128`: CPython's `float(int)` (round half to even).
pub fn float_of_int(value: &IBig) -> ClipResult<f64> {
    pyfloat::int_to_f64(value).map_err(crate::error::int_overflow)
}

/// `a / b` for Python ints (`b > 0`): the correctly rounded quotient or the named overflow.
pub fn true_divide(numerator: &IBig, denominator: &UBig) -> Result<f64, pyfloat::Overflow> {
    ratio_to_f64(numerator, denominator)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rat(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    #[test]
    fn ceil_div_matches_python_for_every_sign() {
        let cases = [(7, 2, 4), (-7, 2, -3), (6, 2, 3), (-6, 2, -3), (0, 5, 0), (1, 3, 1), (-1, 3, 0)];
        for (n, d, expected) in cases {
            assert_eq!(ceil_div(&IBig::from(n), &UBig::from(d as u32)), IBig::from(expected), "{n}/{d}");
        }
    }

    #[test]
    fn ceiling_roots_are_exact_upper_bounds() {
        // sqrt(2 * 10^18) = 1414213562.37...: 1414213563 nanometres
        assert_eq!(nanometres(&rat(2, 1)).unwrap(), UBig::from(1_414_213_563u64));
        assert_eq!(nanometres(&rat(4, 1)).unwrap(), UBig::from(2_000_000_000u64));
        assert_eq!(nanometres(&rat(0, 1)).unwrap(), UBig::ZERO);
        assert_eq!(milli_cells(&rat(1, 4)).unwrap(), UBig::from(500u32));
        assert_eq!(milli_cells(&rat(2, 1)).unwrap(), UBig::from(1415u32));
        assert!(nanometres(&rat(-1, 3)).is_err());
        let root = upper_root(&rat(2, 1)).unwrap();
        assert!(root.mul(&root) >= rat(2, 1));
        assert_eq!(upper_root(&rat(4, 1)).unwrap(), rat(2, 1));
    }

    #[test]
    fn the_float_window_is_widened_by_one_ulp_each_side() {
        let x = SqrtSum::rational(&rat(1, 2));
        let y = SqrtSum::rational(&rat(-3, 1));
        let [x_min, x_max, y_min, y_max] = window(&x, &y).unwrap();
        assert!(x_min < 0.5 && 0.5 < x_max && y_min < -3.0 && -3.0 < y_max);
    }
}
