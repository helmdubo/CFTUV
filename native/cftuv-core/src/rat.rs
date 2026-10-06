//! Canonical rationals, identical to `fractions.Fraction`, and the coefficient wrapper that remembers whether
//! the Python object was an `int`.
//!
//! A [`Rat`] is `(numerator, positive denominator)` in lowest terms, zero is `0/1`: the same canonical form
//! `Fraction` keeps, so two equal values are always equal field by field. Python additionally lets sqrt-sum
//! coefficients be plain `int` objects (inputs, and whatever `+`, `-`, `-x` and `product_added` pass through);
//! that distinction is observable in the oracle's output, so a [`Coef`] carries it.

use std::cmp::Ordering;

use crate::num::{self, IBig, UBig};

/// Division by zero (`ZeroDivisionError`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZeroDivision;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Rat {
    num: IBig,
    den: UBig,
}

impl Rat {
    pub fn zero() -> Rat {
        Rat { num: IBig::ZERO, den: UBig::ONE }
    }

    pub fn one() -> Rat {
        Rat { num: IBig::ONE, den: UBig::ONE }
    }

    pub fn from_int(num: IBig) -> Rat {
        Rat { num, den: UBig::ONE }
    }

    pub fn from_i64(num: i64) -> Rat {
        Rat::from_int(IBig::from(num))
    }

    /// `Fraction(num, den)`: any sign, any common factor; a zero denominator is a `ZeroDivision`.
    pub fn new(num: IBig, den: IBig) -> Result<Rat, ZeroDivision> {
        if den.is_zero() {
            return Err(ZeroDivision);
        }
        if num::is_negative(&den) {
            Ok(Rat::reduced(-num, num::magnitude(&den)))
        } else {
            Ok(Rat::reduced(num, num::magnitude(&den)))
        }
    }

    /// `Fraction(num, den)` for a positive denominator: divides out the common factor.
    pub fn reduced(num: IBig, den: UBig) -> Rat {
        debug_assert!(!den.is_zero());
        if den.is_one() {
            return Rat { num, den };
        }
        let common = num::gcd_mixed(&num, &den);
        if common.is_one() {
            return Rat { num, den };
        }
        Rat { num: num / &common, den: den / common }
    }

    /// A value already in lowest terms with a positive denominator (the caller's invariant; checked in debug).
    pub fn from_canonical(num: IBig, den: UBig) -> Rat {
        debug_assert!(Rat::is_canonical(&num, &den));
        Rat { num, den }
    }

    /// Whether `(num, den)` is exactly the form `Fraction` keeps.
    pub fn is_canonical(num: &IBig, den: &UBig) -> bool {
        !den.is_zero() && (den.is_one() || num::gcd_mixed(num, den).is_one())
    }

    pub fn numerator(&self) -> &IBig {
        &self.num
    }

    pub fn denominator(&self) -> &UBig {
        &self.den
    }

    pub fn into_parts(self) -> (IBig, UBig) {
        (self.num, self.den)
    }

    pub fn is_zero(&self) -> bool {
        self.num.is_zero()
    }

    /// The denominator is one (`Fraction(n, 1)` as well as an `int`).
    pub fn is_integer(&self) -> bool {
        self.den.is_one()
    }

    pub fn signum(&self) -> i8 {
        num::signum(&self.num)
    }

    pub fn neg(&self) -> Rat {
        Rat { num: -&self.num, den: self.den.clone() }
    }

    pub fn add(&self, other: &Rat) -> Rat {
        self.combine(other, false)
    }

    pub fn sub(&self, other: &Rat) -> Rat {
        self.combine(other, true)
    }

    /// `Fraction._add` / `_sub` (Knuth): the same canonical result as the textbook formula.
    fn combine(&self, other: &Rat, subtract: bool) -> Rat {
        let signed = |value: &IBig| if subtract { -value } else { value.clone() };
        if self.den.is_one() && other.den.is_one() {
            let right = signed(&other.num);
            return Rat { num: &self.num + right, den: UBig::ONE };
        }
        // equal denominators share all of themselves: the gcd of the two is known without taking it
        let common = if self.den == other.den { self.den.clone() } else { num::gcd(&self.den, &other.den) };
        if common.is_one() {
            let num = &self.num * IBig::from(other.den.clone()) + signed(&other.num) * IBig::from(self.den.clone());
            return Rat { num, den: &self.den * &other.den };
        }
        let scaled_self = &self.den / &common;
        let scaled_other = &other.den / &common;
        let total = &self.num * IBig::from(scaled_other.clone()) + signed(&other.num) * IBig::from(scaled_self.clone());
        let second = num::gcd_mixed(&total, &common);
        if second.is_one() {
            Rat { num: total, den: scaled_self * &other.den }
        } else {
            Rat { num: total / &second, den: scaled_self * (&other.den / second) }
        }
    }

    /// `Fraction._mul`: cross-cancelled, so the result is canonical without a final gcd. A denominator of one cancels
    /// nothing, and the gcd that would prove it is not taken.
    pub fn mul(&self, other: &Rat) -> Rat {
        let first = if other.den.is_one() { UBig::ONE } else { num::gcd_mixed(&self.num, &other.den) };
        let second = if self.den.is_one() { UBig::ONE } else { num::gcd_mixed(&other.num, &self.den) };
        let cancels = |common: &UBig| !common.is_one() && !common.is_zero();
        let (left, right_den) = if cancels(&first) {
            let mut left = self.num.clone();
            left /= &first;
            (Some(left), Some(&other.den / &first))
        } else {
            (None, None)
        };
        let (right, left_den) = if cancels(&second) {
            let mut right = other.num.clone();
            right /= &second;
            (Some(right), Some(&self.den / &second))
        } else {
            (None, None)
        };
        let left: &IBig = left.as_ref().unwrap_or(&self.num);
        let right_den: &UBig = right_den.as_ref().unwrap_or(&other.den);
        let right: &IBig = right.as_ref().unwrap_or(&other.num);
        let left_den: &UBig = left_den.as_ref().unwrap_or(&self.den);
        Rat { num: left * right, den: left_den * right_den }
    }

    /// `Fraction._div`; a zero divisor is a `ZeroDivision`.
    pub fn div(&self, other: &Rat) -> Result<Rat, ZeroDivision> {
        if other.num.is_zero() {
            return Err(ZeroDivision);
        }
        let inverse = if num::is_negative(&other.num) {
            Rat { num: -IBig::from(other.den.clone()), den: num::magnitude(&other.num) }
        } else {
            Rat { num: IBig::from(other.den.clone()), den: num::magnitude(&other.num) }
        };
        Ok(self.mul(&inverse))
    }
}

impl PartialOrd for Rat {
    fn partial_cmp(&self, other: &Rat) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Rat {
    fn cmp(&self, other: &Rat) -> Ordering {
        if self.den == other.den {
            return self.num.cmp(&other.num);
        }
        (&self.num * IBig::from(other.den.clone())).cmp(&(&other.num * IBig::from(self.den.clone())))
    }
}

/// A sqrt-sum coefficient: the canonical value plus whether Python holds it as an `int` (`py_int`, which
/// implies a denominator of one) or as a `Fraction`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Coef {
    value: Rat,
    py_int: bool,
}

impl Coef {
    /// A Python `int` coefficient.
    pub fn int(value: IBig) -> Coef {
        Coef { value: Rat::from_int(value), py_int: true }
    }

    /// A Python `Fraction` coefficient (which may well be integral).
    pub fn fraction(value: Rat) -> Coef {
        Coef { value, py_int: false }
    }

    pub fn value(&self) -> &Rat {
        &self.value
    }

    pub fn into_value(self) -> Rat {
        self.value
    }

    pub fn is_py_int(&self) -> bool {
        self.py_int
    }

    pub fn is_zero(&self) -> bool {
        self.value.is_zero()
    }

    /// `-c`: neither the type nor the value class changes.
    pub fn neg(&self) -> Coef {
        Coef { value: self.value.neg(), py_int: self.py_int }
    }

    /// `old + other`: `int + int` stays `int`, anything with a `Fraction` is a `Fraction`.
    pub fn add(&self, other: &Coef) -> Coef {
        Coef { value: self.value.add(&other.value), py_int: self.py_int && other.py_int }
    }

    /// `old - other`, with the same typing as [`Coef::add`].
    pub fn sub(&self, other: &Coef) -> Coef {
        Coef { value: self.value.sub(&other.value), py_int: self.py_int && other.py_int }
    }

    /// `Fraction(0) + c`: any coefficient promoted to a `Fraction`.
    pub fn promoted(&self) -> Coef {
        Coef { value: self.value.clone(), py_int: false }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rat(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    #[test]
    fn construction_is_canonical() {
        assert_eq!(rat(6, -4), Rat::from_canonical(IBig::from(-3), UBig::from(2u8)));
        assert_eq!(rat(0, -7), Rat::zero());
        assert_eq!(Rat::new(IBig::ONE, IBig::ZERO), Err(ZeroDivision));
        assert!(Rat::is_canonical(&IBig::from(-3), &UBig::from(2u8)));
        assert!(!Rat::is_canonical(&IBig::from(2), &UBig::from(4u8)));
        assert!(!Rat::is_canonical(&IBig::ONE, &UBig::ZERO));
    }

    #[test]
    fn arithmetic_matches_the_textbook_formulas() {
        let mut state = 0x1234_5678_9abc_def0u64;
        let mut next = |bound: i64| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state % (2 * bound as u64)) as i64 - bound
        };
        for _ in 0..4000 {
            let (a, b, c, d) = (next(60), next(60).max(1).abs().max(1), next(60), next(60).abs().max(1));
            let left = rat(a, b);
            let right = rat(c, d);
            assert_eq!(left.add(&right), rat(a * d + c * b, b * d));
            assert_eq!(left.sub(&right), rat(a * d - c * b, b * d));
            assert_eq!(left.mul(&right), rat(a * c, b * d));
            if c != 0 {
                assert_eq!(left.div(&right).unwrap(), rat(a * d, b * c));
            } else {
                assert_eq!(left.div(&right), Err(ZeroDivision));
            }
            assert_eq!(left.cmp(&right), (a * d).cmp(&(c * b)));
            assert_eq!(left.neg(), rat(-a, b));
            assert!(Rat::is_canonical(left.add(&right).numerator(), left.add(&right).denominator()));
        }
    }

    #[test]
    fn coefficient_typing_follows_python() {
        let int = |n: i64| Coef::int(IBig::from(n));
        let frac = |n: i64, d: i64| Coef::fraction(rat(n, d));
        assert!(int(2).add(&int(3)).is_py_int());
        assert!(!int(2).add(&frac(3, 1)).is_py_int());
        assert!(!frac(2, 1).add(&int(3)).is_py_int());
        assert!(int(2).sub(&int(3)).is_py_int());
        assert!(int(2).neg().is_py_int());
        assert!(!frac(2, 1).neg().is_py_int());
        assert!(!int(2).promoted().is_py_int());
        assert_eq!(int(5).value(), &Rat::from_i64(5));
    }
}
