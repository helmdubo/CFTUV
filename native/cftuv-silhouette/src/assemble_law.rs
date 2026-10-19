//! The two number laws of `assemble.assemble_batch` (the oracle, bit for bit):
//!
//! * [`uv_coordinate`]: one coordinate of `uv_law.uv_direct_strip_v1(s, r, lattice_alpha)`: `sqrt_sum_binary64(value, factor=1/alpha)`, the midpoint of the 64-bit
//!   enclosure of `value / alpha` rounded once;
//! * [`decimal_of`]: `mesh.decimal_of(value, divisor)`: `Decimal(numerator) / Decimal(denominator)` of the midpoint of the 64-bit enclosure of `value / divisor`, under
//!   `mesh.DECIMAL_CONTEXT` (28 digits, `ROUND_HALF_EVEN`, exponent limits that a station never reaches). The quotient is the General Decimal Arithmetic division
//!   (`decimal.Decimal.__truediv__`, the same as libmpdec): the exact quotient when it fits in 28 digits, with the ideal exponent `0` as far as trailing zeros allow,
//!   else rounded half-even to 28 digits. The answer is the triple `(negative, coefficient digits, exponent)`, which `Decimal("<digits>E<exponent>")` rebuilds
//!   exactly.

use cftuv_clip::lift::sqrt_sum_binary64;
use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

/// `ENCLOSURE_BITS`.
const BITS: usize = 64;

/// `mesh.DECIMAL_DIGITS`.
pub const DECIMAL_DIGITS: usize = 28;

/// One coordinate of `uv_direct_strip_v1`: `None` when the value is not a finite binary64 (`UvPoint2V1` would raise).
pub fn uv_coordinate(value: &SqrtSum, inverse_alpha: &Rat) -> Option<f64> {
    let found = sqrt_sum_binary64(&value.scaled(inverse_alpha)).ok()?;
    found.is_finite().then_some(found)
}

fn pow10(exponent: usize) -> UBig {
    UBig::from(10u8).pow(exponent)
}

fn digits(value: &UBig) -> usize {
    value.to_string().len()
}

/// `Decimal(numerator) / Decimal(denominator)` under the materializer's context: `(negative, coefficient, exponent)`; `None` for a zero denominator (`DivisionByZero` is trapped).
pub fn decimal_quotient(numerator: &IBig, denominator: &IBig) -> Option<(bool, UBig, i64)> {
    if num::signum(denominator) == 0 {
        return None;
    }
    let negative = num::is_negative(numerator) != num::is_negative(denominator);
    let top = num::magnitude(numerator);
    let bottom = num::magnitude(denominator);
    if top == UBig::ZERO {
        // `Decimal(0) / Decimal(d)`: coefficient 0 at the ideal exponent 0 (the sign is the xor of the signs)
        return Some((negative, UBig::ZERO, 0));
    }
    let prec = DECIMAL_DIGITS as i64;
    // `shift = len(other._int) - len(self._int) + context.prec + 1`; `exp = self._exp - other._exp - shift`
    let shift = digits(&bottom) as i64 - digits(&top) as i64 + prec + 1;
    let mut exponent = -shift;
    let (mut coefficient, remainder) = if shift >= 0 {
        let scaled = &top * pow10(shift as usize);
        (&scaled / &bottom, &scaled % &bottom)
    } else {
        let scaled = &bottom * pow10((-shift) as usize);
        (&top / &scaled, &top % &scaled)
    };
    if remainder != UBig::ZERO {
        // the sticky digit: an inexact quotient never ends in 0 or 5
        if &coefficient % UBig::from(5u8) == UBig::ZERO {
            coefficient += UBig::ONE;
        }
    } else {
        // exact: trailing zeros down to the ideal exponent 0
        let ten = UBig::from(10u8);
        while exponent < 0 && &coefficient % &ten == UBig::ZERO {
            coefficient /= &ten;
            exponent += 1;
        }
    }
    // `_fix`: round half even to `prec` digits
    let length = digits(&coefficient);
    if length > DECIMAL_DIGITS {
        let excess = length - DECIMAL_DIGITS;
        let unit = pow10(excess);
        let (mut kept, rest) = (&coefficient / &unit, &coefficient % &unit);
        let twice = &rest << 1usize;
        let odd = &kept % UBig::from(2u8) == UBig::ONE;
        if twice > unit || (twice == unit && odd) {
            kept += UBig::ONE;
        }
        exponent += excess as i64;
        if digits(&kept) > DECIMAL_DIGITS {
            kept /= UBig::from(10u8);
            exponent += 1;
        }
        coefficient = kept;
    }
    Some((negative, coefficient, exponent))
}

/// `mesh.decimal_of(value, divisor)`: `None` for a zero divisor.
pub fn decimal_of(value: &SqrtSum, divisor: &IBig) -> Option<(bool, UBig, i64)> {
    if num::signum(divisor) == 0 {
        return None;
    }
    if value.is_zero() {
        // `enclosure_midpoint` answers `(0, 1)` for a value without terms: `Decimal(0) / Decimal(1)`
        return Some((false, UBig::ZERO, 0));
    }
    let factor = Rat::new(IBig::ONE, divisor.clone()).ok()?;
    let scaled = value.scaled(&factor);
    if scaled.is_zero() {
        return Some((false, UBig::ZERO, 0));
    }
    let (low, high, denominator) = scaled.enclosure_parts(BITS);
    decimal_quotient(&(low + high), &IBig::from(denominator << 1usize))
}

/// The text `Decimal` reads back into exactly `(negative, coefficient, exponent)`.
pub fn decimal_text(negative: bool, coefficient: &UBig, exponent: i64) -> String {
    format!("{}{}E{}", if negative { "-" } else { "" }, coefficient, exponent)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quotient(a: i64, b: i64) -> String {
        let (negative, coefficient, exponent) = decimal_quotient(&IBig::from(a), &IBig::from(b)).unwrap();
        decimal_text(negative, &coefficient, exponent)
    }

    #[test]
    fn the_division_of_python_decimal() {
        // `str(Decimal(a) / Decimal(b))` written as coefficient and exponent (taken from CPython 3.11, prec 28, ROUND_HALF_EVEN)
        assert_eq!(quotient(6, 3), "2E0");
        assert_eq!(quotient(1, 4), "25E-2");
        assert_eq!(quotient(10, 1), "10E0");
        assert_eq!(quotient(100, 4), "25E0");
        assert_eq!(quotient(1, 3), "3333333333333333333333333333E-28");
        assert_eq!(quotient(2, 3), "6666666666666666666666666667E-28");
        assert_eq!(quotient(-1, 8), "-125E-3");
        assert_eq!(quotient(0, 7), "0E0");
        assert_eq!(quotient(1, 1 << 40), "9094947017729282379150390625E-40");
        assert_eq!(quotient(1, 1 << 50), "8881784197001252323389053345E-43");
        assert_eq!(quotient(5, 10), "5E-1");
        assert_eq!(quotient(99999, 7), "1428557142857142857142857143E-23");
        assert_eq!(quotient(25, 2), "125E-1");
        assert_eq!(quotient(15, 2), "75E-1");
        let big = |a: &str, b: &str| {
            let (negative, coefficient, exponent) = decimal_quotient(&a.parse::<IBig>().unwrap(), &b.parse::<IBig>().unwrap()).unwrap();
            decimal_text(negative, &coefficient, exponent)
        };
        assert_eq!(big("1000000000000000000000000000000", "3"), "3333333333333333333333333333E2");
        assert_eq!(big("10000000000000000000000000000000000000000", "1"), "1000000000000000000000000000E13");
        assert_eq!(big("1", "10000000000000000000000000000000000000000"), "1E-40");
        assert_eq!(big("123456789012345678901234567895", "10"), "1234567890123456789012345679E1");
    }
}
