//! CPython-exact conversions to binary64.
//!
//! `float(int)`, `float(Fraction)` (which is `int(n) / int(d)`) and `math.sqrt(int)` are all built on the
//! same correctly rounded integer true division: the nearest binary64 with ties to even, subnormals rounded
//! once (no double rounding), and `OverflowError` exactly when the rounded value would be `>= 2^1024`.
//! No crate's `to_f64` is trusted here: the rounding is done on exact integers and the result is assembled
//! from its bit pattern.

use dashu_int::ops::BitTest;

use crate::num::{self, IBig, UBig};
use crate::rat::Rat;

/// `OverflowError`: the correctly rounded value does not fit in binary64.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Overflow;

const MANTISSA_BITS: i64 = 52;
const LOWEST_NORMAL_EXPONENT: i64 = -1022;
/// Shift that turns the value into units of the smallest subnormal, `2^-1074`.
const SUBNORMAL_SHIFT: i64 = 1074;
const INFINITY_BITS: u64 = 0x7FF0_0000_0000_0000;

/// `int(num) / int(den)` for a positive denominator, rounded half to even; `-0.0` for a tiny negative.
pub fn ratio_to_f64(num: &IBig, den: &UBig) -> Result<f64, Overflow> {
    debug_assert!(!den.is_zero());
    if num.is_zero() {
        return Ok(0.0);
    }
    let negative = num::is_negative(num);
    let magnitude = num::magnitude(num);
    let value = magnitude_ratio(&magnitude, den)?;
    Ok(if negative { -value } else { value })
}

/// `n / d` for positive `n` and `d`; the exact quotient of two integers below `2^53` needs no more than the
/// hardware division, which CPython also takes for such operands.
fn magnitude_ratio(n: &UBig, d: &UBig) -> Result<f64, Overflow> {
    let n_bits = n.bit_len();
    let d_bits = d.bit_len();
    if n_bits <= 53 && d_bits <= 53 {
        let (Ok(top), Ok(bottom)) = (u64::try_from(n), u64::try_from(d)) else {
            unreachable!("operands below 2^53 fit in u64");
        };
        return Ok(top as f64 / bottom as f64);
    }
    // exponent of the quotient: 2^exp <= n / d < 2^(exp + 1)
    let gap = n_bits as i64 - d_bits as i64;
    let at_least_two_to_gap = if gap >= 0 { *n >= (d << gap as usize) } else { (n << (-gap) as usize) >= *d };
    let exponent = if at_least_two_to_gap { gap } else { gap - 1 };
    if exponent > 1023 {
        return Err(Overflow);
    }
    if exponent < -1075 {
        return Ok(0.0);
    }
    let shift = (MANTISSA_BITS - exponent).min(SUBNORMAL_SHIFT);
    let (numerator, denominator) = if shift >= 0 { (n << shift as usize, d.clone()) } else { (n.clone(), d << (-shift) as usize) };
    let (quotient, remainder) = div_rem(&numerator, &denominator);
    let mut mantissa = u64::try_from(&quotient).expect("the scaled quotient has at most 53 bits");
    let twice = &remainder << 1usize;
    if twice > denominator || (twice == denominator && mantissa & 1 == 1) {
        mantissa += 1;
    }
    let bits = if exponent >= LOWEST_NORMAL_EXPONENT { (((exponent - LOWEST_NORMAL_EXPONENT) as u64) << 52) + mantissa } else { mantissa };
    if bits >= INFINITY_BITS {
        return Err(Overflow);
    }
    Ok(f64::from_bits(bits))
}

fn div_rem(numerator: &UBig, denominator: &UBig) -> (UBig, UBig) {
    use dashu_int::ops::DivRem;
    numerator.div_rem(denominator)
}

/// `float(int)`.
pub fn int_to_f64(value: &IBig) -> Result<f64, Overflow> {
    if value.is_zero() {
        return Ok(0.0);
    }
    if let Ok(small) = i64::try_from(value) {
        return Ok(small as f64);
    }
    ratio_to_f64(value, &UBig::ONE)
}

/// `float(Fraction)`; a rational with a denominator of one goes through the integer conversion, as a `Fraction`
/// numerator over `1` divides to the same correctly rounded value.
pub fn rat_to_f64(value: &Rat) -> Result<f64, Overflow> {
    if value.denominator().is_one() {
        return int_to_f64(value.numerator());
    }
    ratio_to_f64(value.numerator(), value.denominator())
}

/// `math.sqrt(int)` for a non-negative integer: the binary64 square root of `float(int)`.
pub fn math_sqrt(value: &UBig) -> Result<f64, Overflow> {
    if let Ok(small) = u64::try_from(value) {
        return Ok((small as f64).sqrt());
    }
    Ok(ratio_to_f64(&IBig::from(value.clone()), &UBig::ONE)?.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ubig(text: &str) -> UBig {
        text.parse().unwrap()
    }

    fn ratio(n: &str, d: &str) -> Result<f64, Overflow> {
        ratio_to_f64(&n.parse::<IBig>().unwrap(), &ubig(d))
    }

    #[test]
    fn small_operands_use_the_hardware_division() {
        assert_eq!(ratio("1", "3").unwrap().to_bits(), (1.0f64 / 3.0).to_bits());
        assert_eq!(ratio("-7", "5").unwrap().to_bits(), (-7.0f64 / 5.0).to_bits());
        assert_eq!(ratio("0", "5").unwrap().to_bits(), 0.0f64.to_bits());
    }

    #[test]
    fn powers_of_two_are_exact_across_the_whole_range() {
        for exponent in -1074i64..=1023 {
            let expected = f64::from_bits(if exponent >= -1022 { ((exponent + 1023) as u64) << 52 } else { 1u64 << (exponent + 1074) });
            let value = if exponent >= 0 {
                ratio_to_f64(&(IBig::ONE << exponent as usize), &UBig::ONE).unwrap()
            } else {
                ratio_to_f64(&IBig::ONE, &(UBig::ONE << (-exponent) as usize)).unwrap()
            };
            assert_eq!(value.to_bits(), expected.to_bits(), "2^{exponent}");
        }
    }

    #[test]
    fn rounding_is_half_to_even_at_the_53_bit_boundary() {
        // 2^53 + 1 is the tie between 2^53 and 2^53 + 2: the even neighbour is 2^53.
        assert_eq!(int_to_f64(&IBig::from((1i64 << 53) + 1)).unwrap(), 9007199254740992.0);
        // 2^53 + 3 is the tie between 2^53 + 2 and 2^53 + 4: the even neighbour is 2^53 + 4.
        assert_eq!(int_to_f64(&IBig::from((1i64 << 53) + 3)).unwrap(), 9007199254740996.0);
        // beyond i64: 2^64 + 2^11 is a tie (53-bit spacing is 2^12): rounds to even 2^64
        let tie: IBig = "18446744073709553664".parse().unwrap();
        assert_eq!(int_to_f64(&tie).unwrap(), 18446744073709551616.0);
        let above: IBig = "18446744073709553665".parse().unwrap();
        assert_eq!(int_to_f64(&above).unwrap(), 18446744073709555712.0);
    }

    #[test]
    fn overflow_starts_where_the_rounded_value_reaches_two_to_the_1024() {
        let max_double = (IBig::ONE << 1024usize) - (IBig::ONE << 971usize);
        assert_eq!(int_to_f64(&max_double).unwrap(), f64::MAX);
        let halfway = (IBig::ONE << 1024usize) - (IBig::ONE << 970usize);
        assert_eq!(int_to_f64(&halfway), Err(Overflow));
        let below_halfway = &halfway - IBig::ONE;
        assert_eq!(int_to_f64(&below_halfway).unwrap(), f64::MAX);
        assert_eq!(int_to_f64(&(IBig::ONE << 1024usize)), Err(Overflow));
        assert_eq!(int_to_f64(&-(IBig::ONE << 1100usize)), Err(Overflow));
        assert_eq!(math_sqrt(&(UBig::ONE << 1024usize)), Err(Overflow));
    }

    #[test]
    fn subnormals_are_rounded_once() {
        let tiny = |k: usize| ratio_to_f64(&IBig::ONE, &(UBig::ONE << k)).unwrap();
        assert_eq!(tiny(1074).to_bits(), 1);
        assert_eq!(tiny(1075).to_bits(), 0, "exactly half of the smallest subnormal ties to even (zero)");
        assert_eq!(tiny(1076).to_bits(), 0);
        // 3 * 2^-1075 = 1.5 * 2^-1074 ties to the even neighbour 2 * 2^-1074
        assert_eq!(ratio_to_f64(&IBig::from(3), &(UBig::ONE << 1075usize)).unwrap().to_bits(), 2);
        // 2^-1075 + epsilon rounds up to the smallest subnormal
        let just_above = ratio_to_f64(&((IBig::ONE << 100usize) + IBig::ONE), &(UBig::ONE << 1175usize)).unwrap();
        assert_eq!(just_above.to_bits(), 1);
        // the largest subnormal plus half a unit, (2^53 - 1) * 2^-1075, ties up (to the even side) into the smallest normal
        let up = ratio_to_f64(&((IBig::ONE << 53usize) - IBig::ONE), &(UBig::ONE << 1075usize)).unwrap();
        assert_eq!(up.to_bits(), 1u64 << 52);
        // negative underflow keeps the sign of zero
        let negative_zero = ratio_to_f64(&IBig::from(-1), &(UBig::ONE << 1200usize)).unwrap();
        assert_eq!(negative_zero.to_bits(), 0x8000_0000_0000_0000);
    }

    #[test]
    fn math_sqrt_is_the_sqrt_of_the_rounded_float() {
        assert_eq!(math_sqrt(&UBig::from(2u8)).unwrap(), 2.0f64.sqrt());
        let big = ubig("340282366920938463463374607431768211456");
        assert_eq!(math_sqrt(&big).unwrap(), 18446744073709551616.0);
    }

    #[test]
    fn a_fraction_with_huge_terms_divides_like_python() {
        // both terms far beyond binary64, the quotient 1/3
        let num: IBig = (IBig::ONE << 3000usize) - IBig::ONE;
        let den: UBig = (UBig::ONE << 3002usize) - UBig::from(3u8);
        let value = ratio_to_f64(&num, &den).unwrap();
        assert!((value - 0.25).abs() < 1e-15);
        let value = rat_to_f64(&Rat::new(IBig::from(1), IBig::from(3)).unwrap()).unwrap();
        assert_eq!(value.to_bits(), (1.0f64 / 3.0).to_bits());
    }
}
