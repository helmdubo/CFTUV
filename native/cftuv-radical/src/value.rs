//! `RadicalSumV2`: the value type. A thin, allocation-honest layer over `canon.rs` (the term algebra) and `ladder.rs` (the sign and the enclosure); the methods are the oracle's.

use dashu_int::ops::SquareRoot;

use crate::arith::{exact_root, reduced, signed, IBig, UBig};
use crate::canon::{self, Term};
use crate::ladder;
use crate::work::{Failure, Priced, Refusal, RefusalCode, Work};

/// A sum of square roots of rationals, one value-canonical term per square class, sorted by `m'`.
///
/// The terms are a function of the VALUE: `==` and `hash` compare and hash them (exact value equality, by the Besicovitch theorem the value is zero iff it has no terms).
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Radical {
    terms: Vec<Term>,
}

impl Radical {
    // ---- construction -------------------------------------------------------

    pub fn zero() -> Radical {
        Radical { terms: Vec::new() }
    }

    pub fn one() -> Radical {
        Radical { terms: vec![Term::rational(IBig::ONE, UBig::ONE)] }
    }

    /// `RadicalSumV2.rational(value)` for the canonical fraction `numerator/denominator` (`denominator > 0`; reduced here if it is not).
    pub fn rational(numerator: IBig, denominator: UBig) -> Radical {
        if numerator.is_zero() {
            return Radical::zero();
        }
        let (numerator, denominator) = reduced(numerator, denominator);
        Radical { terms: vec![Term::rational(numerator, denominator)] }
    }

    /// The bare constructor of the oracle: the caller vouches that `terms` are canonical (wire readers check, see `text.rs`).
    pub fn from_canonical_terms(terms: Vec<Term>) -> Radical {
        Radical { terms }
    }

    /// `RadicalSumV2.sqrt_of_rational(value)` for the fraction `numerator/denominator` (any sign and common factor, `denominator > 0`).
    ///
    /// `sqrt(p/q) = sqrt(p q) / q`: the term `(p q, 1, q)`, or a rational when `p q` is a square (one class check).
    pub fn sqrt_of_rational(numerator: IBig, denominator: UBig) -> Result<Priced<Radical>, Refusal> {
        let (numerator, denominator) = reduced(numerator, denominator);
        if numerator < IBig::ZERO {
            return Err(Refusal::new(RefusalCode::OutsideField, "root of a negative number", Work::NONE));
        }
        if numerator.is_zero() {
            return Ok((Radical::zero(), Work::NONE));
        }
        let numerator = crate::arith::magnitude(&numerator);
        let radicand = &numerator * &denominator;
        if radicand.is_one() {
            return Ok((Radical::one(), Work::NONE));
        }
        Ok(match exact_root(&radicand) {
            Some(root) => (Radical::rational(signed(&root), denominator), Work::checks(1)),
            None => (Radical { terms: vec![Term { radicand, numerator: IBig::ONE, denominator }] }, Work::checks(1)),
        })
    }

    /// `RadicalSumV2.from_terms(pairs)`: `sum c * sqrt(r)` over `(r, c)` pairs (`c = numerator/denominator`, any sign and common factor, `denominator > 0`) in ANY representation, read by value.
    ///
    /// A radicand that is a perfect square (one class check unless it is 1) goes to the rational part; the others are merged by class and normalised. A negative radicand is `OutsideField` (carrying the checks so far).
    pub fn from_terms(pairs: &[(IBig, IBig, UBig)]) -> Result<Priced<Radical>, Refusal> {
        let mut rational = (IBig::ZERO, UBig::ONE);
        let mut slots = Vec::new();
        let mut checks = 0u64;
        for (radicand, numerator, denominator) in pairs {
            let (numerator, denominator) = reduced(numerator.clone(), denominator.clone());
            if *radicand < IBig::ZERO {
                return Err(Refusal::new(RefusalCode::OutsideField, "root of a negative number", Work::checks(checks)));
            }
            if numerator.is_zero() || radicand.is_zero() {
                continue;
            }
            let radicand = crate::arith::magnitude(radicand);
            let mut numerator = numerator;
            if !radicand.is_one() {
                checks += 1;
                match exact_root(&radicand) {
                    None => {
                        checks += canon::accumulate(&mut slots, radicand, numerator, denominator);
                        continue;
                    }
                    Some(root) => numerator *= signed(&root),
                }
            }
            rational = reduced(std::mem::replace(&mut rational.0, IBig::ZERO) * signed(&denominator) + numerator * signed(&rational.1), std::mem::replace(&mut rational.1, UBig::ONE) * denominator);
        }
        Ok((Radical { terms: canon::freeze(rational, slots) }, Work::checks(checks)))
    }

    // ---- reading --------------------------------------------------------------

    /// The exact zero test (Besicovitch): no terms.
    pub fn is_zero(&self) -> bool {
        self.terms.is_empty()
    }

    pub fn terms(&self) -> &[Term] {
        &self.terms
    }

    /// The value if it is rational, else `None` (an irrational class never cancels to a rational): `(numerator, denominator)`.
    pub fn as_rational(&self) -> Option<(IBig, UBig)> {
        match self.terms.as_slice() {
            [] => Some((IBig::ZERO, UBig::ONE)),
            [only] if only.is_rational() => Some((only.numerator.clone(), only.denominator.clone())),
            _ => None,
        }
    }

    // ---- arithmetic (priced) --------------------------------------------------

    pub fn neg(&self) -> Radical {
        Radical { terms: self.terms.iter().map(|term| Term { radicand: term.radicand.clone(), numerator: -&term.numerator, denominator: term.denominator.clone() }).collect() }
    }

    /// `self * numerator/denominator` for a rational factor: no class can merge, so it costs nothing.
    pub fn scaled(&self, numerator: &IBig, denominator: &UBig) -> Radical {
        if numerator.is_zero() || self.terms.is_empty() {
            return Radical::zero();
        }
        let (numerator, denominator) = reduced(numerator.clone(), denominator.clone());
        Radical { terms: canon::scaled_terms(&self.terms, &numerator, &denominator) }
    }

    pub fn add(&self, other: &Radical) -> Priced<Radical> {
        let (terms, checks) = canon::add_terms(&self.terms, &other.terms, false);
        (Radical { terms }, Work::checks(checks))
    }

    pub fn sub(&self, other: &Radical) -> Priced<Radical> {
        let (terms, checks) = canon::add_terms(&self.terms, &other.terms, true);
        (Radical { terms }, Work::checks(checks))
    }

    pub fn mul(&self, other: &Radical) -> Priced<Radical> {
        let (terms, checks) = canon::mul_terms(&self.terms, &other.terms);
        (Radical { terms }, Work::checks(checks))
    }

    /// `1 / self`; `ZeroDivision` for the exact zero; more than three classes: `InverseClassLimit`.
    pub fn inverse(&self) -> Result<Priced<Radical>, Failure> {
        let (terms, work) = canon::inverse_terms(&self.terms)?;
        Ok((Radical { terms }, work))
    }

    pub fn div(&self, other: &Radical) -> Result<Priced<Radical>, Failure> {
        let (inverse, work) = other.inverse()?;
        let (value, product) = self.mul(&inverse);
        Ok((value, work + product))
    }

    // ---- decisions ------------------------------------------------------------

    /// The exact sign and its price; `precision_cap_bits` is the only way to get `SignUndecided`.
    pub fn signum(&self, precision_cap_bits: Option<i64>) -> Result<(i8, Work), Failure> {
        let (sign, rounds) = ladder::signum_terms(&self.terms, precision_cap_bits)?;
        Ok((sign, if rounds != 0 { Work::rounds(rounds) } else { Work::NONE }))
    }

    /// Outward `[low, high]` as reduced fractions, and its price (one round).
    pub fn enclosure(&self, bits: usize) -> Priced<((IBig, UBig), (IBig, UBig))> {
        (ladder::enclosure(&self.terms, bits), Work::rounds(1))
    }

    /// The integer form of the terms (`_integer_form`): the least common denominator and the items `(m', a)`.
    pub fn integer_form(&self) -> ladder::IntegerForm {
        ladder::integer_form(&self.terms)
    }

    /// `isqrt(m')` of a term, for callers that build their own bounds.
    pub fn floor_root(term: &Term) -> UBig {
        term.radicand.sqrt()
    }
}
