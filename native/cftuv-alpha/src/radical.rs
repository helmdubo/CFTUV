//! `RadicalSumV1` (`kernel/src/cftuv_envelope/reference/native_exact.py`): the subset the contact leaf needs, mirrored OPERATION FOR OPERATION.
//!
//! This is not `SqrtSumV1`: a value is `sum c_k * sqrt(r_k)` with one coefficient per SQUARE CLASS (`isqrt(keep * new)` is a perfect square), and the radicand that stands for a class is
//! whichever of its members entered the sum first. It is NOT the squarefree part and NOT a function of the value: the representative, and with it the order of the terms (they are sorted by
//! representative), the integer enclosure and the binary64 measure, depend on the order the operations were done in. The leaf's answer carries those terms out to the host (the
//! enclosure of every alpha enters the step window), so a bit-equal port reproduces the Python loops, not just the value: the `dict` of `_accumulate` is a `Vec` here (insertion
//! order, linear search), `_freeze` keeps the non-zero entries and sorts them by radicand, `__mul__` is left-outer / right-inner, `inverse` pivots on the largest representative.
//!
//! Python raises `NativeExactError` (`OutsideNativeField`, `NativeSignUndecided`) and, by contract only, `ZeroDivisionError`; here they are the one [`Refused`], and the caller DECLINES the
//! pair (the oracle then computes it, with its own counts and its own sympy fallback).

use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::integer_enclosure;

/// `_SIGN_BITS`: the precision ladder of the sign of three or more classes.
pub const SIGN_BITS: [usize; 7] = [64, 128, 256, 512, 1024, 2048, 4096];
/// `_INVERSE_CLASS_LIMIT`: more classes than this are not inverted by conjugation.
const INVERSE_CLASS_LIMIT: usize = 3;

/// What the Python class raises instead of an answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refused {
    /// `OutsideNativeField`: too many square classes to invert.
    OutsideField,
    /// `NativeSignUndecided`: the enclosure did not separate the value from zero up to `SIGN_BITS[-1]` bits.
    SignUndecided,
    /// `ZeroDivisionError` (an inverse of an exact zero; unreachable by contract).
    ZeroDivision,
}

pub type Res<T> = Result<T, Refused>;
pub type Terms = Vec<(UBig, Rat)>;

/// `RadicalSumV1(terms)`; the terms are held as the Python tuple holds them (no hidden normalisation).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Radical {
    terms: Terms,
}

/// `_class_ratio(keep, new)`: `sqrt(new) = ratio * sqrt(keep)` when `keep * new` is a perfect square.
fn class_ratio(keep: &UBig, new: &UBig) -> Option<Rat> {
    let product = keep * new;
    let root = num::isqrt(&product);
    if &root * &root == product {
        // `keep` is a positive radicand (the wire is checked), so the denominator is never zero
        Rat::new(IBig::from(root), IBig::from(keep.clone())).ok()
    } else {
        None
    }
}

/// `_accumulate(into, radicand, coefficient)` on the insertion-ordered `dict`.
fn accumulate(into: &mut Terms, radicand: UBig, coefficient: Rat) {
    if let Some(slot) = into.iter_mut().find(|slot| slot.0 == radicand) {
        slot.1 = slot.1.add(&coefficient);
        return;
    }
    if !radicand.is_one() {
        for slot in into.iter_mut() {
            if slot.0.is_one() {
                continue;
            }
            if let Some(ratio) = class_ratio(&slot.0, &radicand) {
                slot.1 = slot.1.add(&coefficient.mul(&ratio));
                return;
            }
        }
    }
    into.push((radicand, coefficient));
}

/// `_freeze(into)`: the non-zero entries, sorted by radicand (radicands are unique, so a coefficient is never compared).
fn freeze(mut into: Terms) -> Radical {
    into.retain(|slot| !slot.1.is_zero());
    into.sort_by(|left, right| left.0.cmp(&right.0));
    Radical { terms: into }
}

/// `_product_term(left, right)`: `sqrt(left) * sqrt(right) = multiplier * sqrt(radicand)`; a perfect square goes to the multiplier.
fn product_term(left: &UBig, right: &UBig) -> (UBig, UBig) {
    if left.is_one() {
        return (right.clone(), UBig::ONE);
    }
    if right.is_one() {
        return (left.clone(), UBig::ONE);
    }
    let common = num::gcd(left, right);
    let mut radicand = (left / &common) * (right / &common);
    let mut multiplier = common;
    if !radicand.is_one() {
        let root = num::isqrt(&radicand);
        if &root * &root == radicand {
            multiplier = multiplier * root;
            radicand = UBig::ONE;
        }
    }
    (radicand, multiplier)
}

impl Radical {
    pub fn zero() -> Radical {
        Radical { terms: Vec::new() }
    }

    pub fn one() -> Radical {
        Radical { terms: vec![(UBig::ONE, Rat::one())] }
    }

    /// `RadicalSumV1.rational(value)`.
    pub fn rational(value: Rat) -> Radical {
        if value.is_zero() {
            Radical::zero()
        } else {
            Radical { terms: vec![(UBig::ONE, value)] }
        }
    }

    /// `RadicalSumV1(terms)` as given (the wire of a frame; the caller checked radicands and denominators).
    pub fn from_terms(terms: Terms) -> Radical {
        Radical { terms }
    }

    pub fn terms(&self) -> &[(UBig, Rat)] {
        &self.terms
    }

    /// `as_rational()`.
    pub fn as_rational(&self) -> Option<Rat> {
        match self.terms.as_slice() {
            [] => Some(Rat::zero()),
            [(radicand, value)] if radicand.is_one() => Some(value.clone()),
            _ => None,
        }
    }

    pub fn neg(&self) -> Radical {
        Radical { terms: self.terms.iter().map(|(radicand, value)| (radicand.clone(), value.neg())).collect() }
    }

    /// `__add__`: an empty operand returns the other as it is.
    pub fn add(&self, other: &Radical) -> Radical {
        if other.terms.is_empty() {
            return self.clone();
        }
        if self.terms.is_empty() {
            return other.clone();
        }
        let mut total = self.terms.clone();
        for (radicand, value) in &other.terms {
            accumulate(&mut total, radicand.clone(), value.clone());
        }
        freeze(total)
    }

    /// `__sub__`: an empty subtrahend returns `self` as it is; otherwise the terms of `other` are accumulated NEGATED into `dict(self.terms)`.
    pub fn sub(&self, other: &Radical) -> Radical {
        if other.terms.is_empty() {
            return self.clone();
        }
        let mut total = self.terms.clone();
        for (radicand, value) in &other.terms {
            accumulate(&mut total, radicand.clone(), value.neg());
        }
        freeze(total)
    }

    /// `scaled(factor)`.
    pub fn scaled(&self, factor: &Rat) -> Radical {
        if factor.is_zero() || self.terms.is_empty() {
            return Radical::zero();
        }
        Radical { terms: self.terms.iter().map(|(radicand, value)| (radicand.clone(), value.mul(factor))).collect() }
    }

    /// `__mul__` of two sums: left-outer, right-inner, each product into the class dict.
    pub fn mul(&self, other: &Radical) -> Radical {
        if self.terms.is_empty() || other.terms.is_empty() {
            return Radical::zero();
        }
        let mut total: Terms = Vec::new();
        for (left, left_value) in &self.terms {
            for (right, right_value) in &other.terms {
                let (radicand, multiplier) = product_term(left, right);
                let coefficient = left_value.mul(right_value).mul(&Rat::from_int(IBig::from(multiplier)));
                accumulate(&mut total, radicand, coefficient);
            }
        }
        freeze(total)
    }

    /// `inverse()`: one term directly, up to three classes by conjugation, else `OutsideNativeField`.
    pub fn inverse(&self) -> Res<Radical> {
        let terms = &self.terms;
        if terms.is_empty() {
            return Err(Refused::ZeroDivision);
        }
        if terms.len() == 1 {
            let (radicand, value) = &terms[0];
            let denominator = value.mul(&Rat::from_int(IBig::from(radicand.clone())));
            let inverse = Rat::one().div(&denominator).map_err(|_| Refused::ZeroDivision)?;
            return Ok(Radical { terms: vec![(radicand.clone(), inverse)] });
        }
        if terms.len() > INVERSE_CLASS_LIMIT {
            return Err(Refused::OutsideField);
        }
        let (pivot_radicand, pivot_value) = &terms[terms.len() - 1];
        let rest = Radical { terms: terms[..terms.len() - 1].to_vec() };
        let mut conjugate_terms = terms[..terms.len() - 1].to_vec();
        conjugate_terms.push((pivot_radicand.clone(), pivot_value.neg()));
        let conjugate = Radical { terms: conjugate_terms };
        let pivot_square = pivot_value.mul(pivot_value).mul(&Rat::from_int(IBig::from(pivot_radicand.clone())));
        let norm = rest.mul(&rest).sub(&Radical { terms: vec![(UBig::ONE, pivot_square)] });
        Ok(conjugate.mul(&norm.inverse()?))
    }

    /// `self * other.inverse()` (`__truediv__` by a sum).
    pub fn div(&self, other: &Radical) -> Res<Radical> {
        Ok(self.mul(&other.inverse()?))
    }

    /// `signum()`: zero is the empty sum; two classes by squares; three or more by the integer enclosure ladder.
    pub fn signum(&self) -> Res<i8> {
        match self.terms.as_slice() {
            [] => Ok(0),
            [(_, value)] => Ok(value.signum()),
            [(first, first_value), (second, second_value)] => {
                let first_sign = first_value.signum();
                let second_sign = second_value.signum();
                if first_sign == second_sign {
                    return Ok(first_sign);
                }
                let left = first_value.mul(first_value).mul(&Rat::from_int(IBig::from(first.clone())));
                let right = second_value.mul(second_value).mul(&Rat::from_int(IBig::from(second.clone())));
                let difference = left.sub(&right);
                Ok(if difference.signum() > 0 { first_sign } else { second_sign })
            }
            terms => {
                let items = integer_items(terms);
                for bits in SIGN_BITS {
                    let (low, high) = integer_enclosure(&items, bits);
                    if low > IBig::ZERO {
                        return Ok(1);
                    }
                    if high < IBig::ZERO {
                        return Ok(-1);
                    }
                }
                Err(Refused::SignUndecided)
            }
        }
    }
}

/// `_integer_form(terms)[1]`: `(r, a)` with `c = a / L`, `L` the least common denominator of the coefficients.
fn integer_items(terms: &[(UBig, Rat)]) -> Vec<(UBig, IBig)> {
    let mut common = UBig::ONE;
    for (_, value) in terms {
        if !value.denominator().is_one() {
            common = num::lcm(&common, value.denominator());
        }
    }
    terms
        .iter()
        .map(|(radicand, value)| (radicand.clone(), value.numerator() * IBig::from(&common / value.denominator())))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn q(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    fn sum(terms: &[(u64, i64, i64)]) -> Radical {
        Radical::from_terms(terms.iter().map(|&(r, n, d)| (UBig::from(r), q(n, d))).collect())
    }

    fn shape(value: &Radical) -> Vec<(u64, i64, i64)> {
        value
            .terms()
            .iter()
            .map(|(r, c)| (u64::try_from(r).unwrap(), i64::try_from(c.numerator()).unwrap(), i64::try_from(&IBig::from(c.denominator().clone())).unwrap()))
            .collect()
    }

    #[test]
    fn the_representative_is_the_first_radicand_that_entered_the_sum() {
        // sqrt(12) + sqrt(3) = 2 sqrt(3) + sqrt(3): the class keeps 12 (the left operand's), the right one is folded in as ratio 1/2
        let left = sum(&[(12, 1, 1)]);
        let right = sum(&[(3, 1, 1)]);
        assert_eq!(shape(&left.add(&right)), vec![(12, 3, 2)]);
        assert_eq!(shape(&right.add(&left)), vec![(3, 3, 1)]);
    }

    #[test]
    fn a_class_that_cancelled_keeps_its_slot_until_the_freeze() {
        // 1*sqrt(2) - 1*sqrt(2) + 1*sqrt(8): the zero entry for 2 absorbs sqrt(8) (ratio 2) inside ONE subtraction chain only through the dict;
        // sub drops zero entries at the freeze, so a later add starts fresh
        let two = sum(&[(2, 1, 1)]);
        assert!(two.sub(&two).terms().is_empty());
        let eight = sum(&[(8, 1, 1)]);
        assert_eq!(shape(&two.add(&eight)), vec![(2, 3, 1)]);
    }

    #[test]
    fn products_and_inverses_agree_with_the_python_loops() {
        let a = sum(&[(1, 1, 1), (2, 1, 1)]); // 1 + sqrt(2)
        let inverse = a.inverse().unwrap(); // sqrt(2) - 1
        assert_eq!(shape(&a.mul(&inverse)), vec![(1, 1, 1)]);
        assert_eq!(shape(&inverse), vec![(1, -1, 1), (2, 1, 1)]);
        // an exact square of a root lands in the rational part
        let root = sum(&[(5, 1, 1)]);
        assert_eq!(shape(&root.mul(&root)), vec![(1, 5, 1)]);
        // four classes cannot be inverted
        let four = sum(&[(1, 1, 1), (2, 1, 1), (3, 1, 1), (5, 1, 1)]);
        assert_eq!(four.inverse(), Err(Refused::OutsideField));
        assert_eq!(Radical::zero().inverse(), Err(Refused::ZeroDivision));
    }

    #[test]
    fn signs_by_squares_and_by_the_enclosure_ladder() {
        assert_eq!(sum(&[(1, 3, 2), (2, -1, 1)]).signum(), Ok(1)); // 1.5 - 1.414
        assert_eq!(sum(&[(1, 7, 5), (2, -1, 1)]).signum(), Ok(-1)); // 1.4 - 1.414
        assert_eq!(sum(&[(1, 1, 1), (2, 1, 1), (3, -3, 1)]).signum(), Ok(-1)); // 1 + 1.414 - 5.196
        assert_eq!(sum(&[(2, 1, 1), (3, 1, 1), (5, -2, 1)]).signum(), Ok(-1)); // 1.414 + 1.732 - 4.472
    }
}
