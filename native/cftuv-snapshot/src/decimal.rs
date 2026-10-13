//! Python `Decimal` values and binary64 numbers as exact numbers.
//!
//! A `Decimal` reaches the port as the text `str(value)` shows (`1.5`, `1E-7`, `-0`, `0.500000000000000000000000000`): a signed coefficient and a power of ten. Only finite values are read
//! (`Infinity`, `NaN`, `sNaN` are not numbers a certified interval may hold, and a comparison with them raises in Python), and only the exact arithmetic is mirrored: a subtraction whose exact
//! result needs more digits than the ambient context holds is rounded by Python, and the port declines it instead of imitating the rounding.

use std::cmp::Ordering;

use cftuv_core::num::{self, IBig, UBig};
use dashu_int::ops::BitTest;
use cftuv_core::rat::Rat;

use crate::model::Dec;

/// The largest difference of exponents two decimals are aligned across (a coefficient is multiplied by `10^difference`); beyond it the comparison is declined, not computed.
const ALIGN_LIMIT: u64 = 4096;

fn pow10(exponent: u64) -> IBig {
    IBig::from(10u8).pow(exponent as usize)
}

/// The decimal digits of `|coefficient|` (`Decimal` counts the digits of a zero coefficient as one).
pub fn digit_count(coefficient: &IBig) -> usize {
    let magnitude = num::magnitude(coefficient);
    if magnitude.is_zero() {
        return 1;
    }
    // bit_len * log10(2) is an estimate within one digit; fix it up by comparing with powers of ten
    let bits = magnitude.bit_len();
    let mut estimate = ((bits as f64) * std::f64::consts::LOG10_2).floor() as usize + 1;
    let ten = UBig::from(10u8);
    while estimate > 1 && magnitude < ten.pow(estimate - 1) {
        estimate -= 1;
    }
    while magnitude >= ten.pow(estimate) {
        estimate += 1;
    }
    estimate
}

/// A finite decimal from the text of `str(Decimal)`; `None` for anything else.
pub fn parse(text: &str) -> Option<Dec> {
    let bytes = text.as_bytes();
    let mut index = 0;
    let negative = match bytes.first()? {
        b'-' => {
            index = 1;
            true
        }
        b'+' => {
            index = 1;
            false
        }
        _ => false,
    };
    let mut digits = String::new();
    let mut fraction_digits: i64 = 0;
    let mut seen_point = false;
    let mut any_digit = false;
    while index < bytes.len() {
        match bytes[index] {
            b'0'..=b'9' => {
                digits.push(bytes[index] as char);
                any_digit = true;
                if seen_point {
                    fraction_digits += 1;
                }
            }
            b'.' if !seen_point => seen_point = true,
            b'E' | b'e' => break,
            _ => return None,
        }
        index += 1;
    }
    if !any_digit {
        return None;
    }
    let mut exponent: i64 = 0;
    if index < bytes.len() {
        // `E`, an optional sign, digits
        let rest = &text[index + 1..];
        if rest.is_empty() || rest.len() > 12 {
            return None;
        }
        let (sign, body) = match rest.as_bytes()[0] {
            b'-' => (-1i64, &rest[1..]),
            b'+' => (1i64, &rest[1..]),
            _ => (1i64, rest),
        };
        if body.is_empty() || !body.bytes().all(|byte| byte.is_ascii_digit()) {
            return None;
        }
        exponent = sign * body.parse::<i64>().ok()?;
    }
    let magnitude: IBig = digits.parse().ok()?;
    let coefficient = if negative { -magnitude } else { magnitude };
    Some(Dec { coefficient, exponent: exponent - fraction_digits })
}

/// The value as an exact rational.
pub fn to_rat(value: &Dec) -> Rat {
    if value.exponent >= 0 {
        Rat::from_int(&value.coefficient * pow10(value.exponent as u64))
    } else {
        Rat::new(value.coefficient.clone(), pow10(value.exponent.unsigned_abs())).expect("a power of ten is not zero")
    }
}

/// The numeric order of two decimals (`Decimal.__lt__`...: by value, `0.50 == 0.5`, `-0 == 0`); `None` when the exponents are too far apart to align.
pub fn compare(left: &Dec, right: &Dec) -> Option<Ordering> {
    let exponent = left.exponent.min(right.exponent);
    let left_shift = (left.exponent - exponent) as u64;
    let right_shift = (right.exponent - exponent) as u64;
    if left_shift > ALIGN_LIMIT || right_shift > ALIGN_LIMIT {
        return None;
    }
    let raised_left = &left.coefficient * pow10(left_shift);
    let raised_right = &right.coefficient * pow10(right_shift);
    Some(raised_left.cmp(&raised_right))
}

/// `value - 1` computed the way `Decimal.__sub__` computes it when the result is EXACT: the coefficient of the smaller exponent, and the result is returned only if it needs no more
/// than `precision` digits (otherwise the context would round it and the port declines).
pub fn minus_one_exact(value: &Dec, precision: u32) -> Option<Dec> {
    let exponent = value.exponent.min(0);
    let shift = (value.exponent - exponent) as u64;
    if shift > ALIGN_LIMIT {
        return None;
    }
    let one_shift = exponent.unsigned_abs();
    if one_shift > ALIGN_LIMIT {
        return None;
    }
    let coefficient = &value.coefficient * pow10(shift) - pow10(one_shift);
    if digit_count(&coefficient) > precision as usize {
        return None;
    }
    Some(Dec { coefficient, exponent })
}

/// The exact value of a finite binary64 (`Fraction(*value.as_integer_ratio())`); `None` for infinities and NaN.
pub fn rat_of_f64(value: f64) -> Option<Rat> {
    if !value.is_finite() {
        return None;
    }
    let bits = value.to_bits();
    let negative = bits >> 63 == 1;
    let biased = ((bits >> 52) & 0x7ff) as i64;
    let fraction = bits & ((1u64 << 52) - 1);
    let (mantissa, exponent) = if biased == 0 { (fraction, -1074i64) } else { (fraction | (1u64 << 52), biased - 1075) };
    let mut numerator = IBig::from(mantissa);
    if negative {
        numerator = -numerator;
    }
    if mantissa == 0 {
        return Some(Rat::zero());
    }
    if exponent >= 0 {
        Some(Rat::from_int(numerator << exponent as usize))
    } else {
        // reduce by the common power of two first (the mantissa may be even)
        let trailing = mantissa.trailing_zeros() as i64;
        let cancel = trailing.min(-exponent);
        let reduced = numerator >> cancel as usize;
        let denominator = UBig::ONE << ((-exponent - cancel) as usize);
        Some(Rat::reduced(reduced, denominator))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dec(text: &str) -> Dec {
        parse(text).unwrap()
    }

    #[test]
    fn the_shapes_of_str_decimal_parse() {
        assert_eq!(dec("1.5"), Dec { coefficient: IBig::from(15), exponent: -1 });
        assert_eq!(dec("-0"), Dec { coefficient: IBig::ZERO, exponent: 0 });
        assert_eq!(dec("1E-7"), Dec { coefficient: IBig::ONE, exponent: -7 });
        assert_eq!(dec("1.5E+3"), Dec { coefficient: IBig::from(15), exponent: 2 });
        assert_eq!(dec("0.500000000000000000000000000").exponent, -27);
        for bad in ["", "-", "NaN", "Infinity", "sNaN", "1e", "1E+", "1..2", "x", "1 "] {
            assert!(parse(bad).is_none(), "{bad:?}");
        }
    }

    #[test]
    fn values_compare_by_value_not_by_form() {
        assert_eq!(compare(&dec("0.50"), &dec("0.5")), Some(Ordering::Equal));
        assert_eq!(compare(&dec("-0"), &dec("0")), Some(Ordering::Equal));
        assert_eq!(compare(&dec("1.5"), &dec("1.49999")), Some(Ordering::Greater));
        assert_eq!(compare(&dec("1E-5"), &dec("0.00002")), Some(Ordering::Less));
        assert_eq!(compare(&dec("1E5000"), &dec("1")), None);
    }

    #[test]
    fn minus_one_is_exact_while_the_digits_fit() {
        let result = minus_one_exact(&dec("1.500000000000000000000000000"), 28).unwrap();
        assert_eq!(compare(&result, &dec("0.5")), Some(Ordering::Equal));
        // 35 digits: Python's context would round the difference to 28
        assert!(minus_one_exact(&dec("1.1104059866156777258442802201234567"), 28).is_none());
        assert!(minus_one_exact(&dec("1.1104059866156777258442802201234567"), 40).is_some());
        // the difference may need FEWER digits than the operand
        let cancelled = minus_one_exact(&dec("1.0000000000000000000000000000001"), 28).unwrap();
        assert_eq!(compare(&cancelled, &dec("1E-31")), Some(Ordering::Equal));
    }

    #[test]
    fn digits_are_counted_like_decimal() {
        assert_eq!(digit_count(&IBig::ZERO), 1);
        assert_eq!(digit_count(&IBig::from(9)), 1);
        assert_eq!(digit_count(&IBig::from(10)), 2);
        assert_eq!(digit_count(&IBig::from(-999)), 3);
        assert_eq!(digit_count(&IBig::from(1000)), 4);
        let big: IBig = "1000000000000000000000000000".parse().unwrap();
        assert_eq!(digit_count(&big), 28);
    }

    #[test]
    fn a_binary64_is_its_integer_ratio() {
        let half = rat_of_f64(0.5).unwrap();
        assert_eq!((half.numerator().clone(), half.denominator().clone()), (IBig::ONE, UBig::from(2u8)));
        let tenth = rat_of_f64(0.1).unwrap();
        assert_eq!(tenth.numerator().to_string(), "3602879701896397");
        assert_eq!(tenth.denominator().to_string(), "36028797018963968");
        assert_eq!(rat_of_f64(-3.0).unwrap(), Rat::from_i64(-3));
        assert_eq!(rat_of_f64(0.0).unwrap(), Rat::zero());
        assert_eq!(rat_of_f64(-0.0).unwrap(), Rat::zero());
        assert_eq!(rat_of_f64(1e300).unwrap().denominator(), &UBig::ONE);
        let tiny = rat_of_f64(5e-324).unwrap();
        assert_eq!(*tiny.numerator(), IBig::ONE);
        assert_eq!(tiny.denominator().bit_len(), 1075);
        assert!(rat_of_f64(f64::NAN).is_none() && rat_of_f64(f64::INFINITY).is_none());
    }
}
