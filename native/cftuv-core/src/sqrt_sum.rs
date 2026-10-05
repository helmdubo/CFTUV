//! `SqrtSumV1` (`exact_sqrt_sum.py`): `sum c_m * sqrt(m)` over distinct squarefree `m >= 1` with non-zero
//! rational `c_m`, and the integer machinery under it.
//!
//! The canonical value is what leaves this module: sorted radicands, coefficients as canonical [`Rat`] with the
//! Python `int`-versus-`Fraction` type of each coefficient preserved wherever the oracle preserves it
//! (see [`Coef`]). Inside, a sum may be worked on as an *integer form* `(L, [(m, a_m)])`, `c_m = a_m / L`;
//! every decision taken on it (enclosure sign, zero-ness, support) is invariant under positive scaling, so the
//! forms here are the same ones the oracle builds and are only reduced where the oracle reduces them.
//!
//! Not in this module (they need the work budget and the canonicalization memory of `cftuv-canon`):
//! `sign` past the enclosure filter (`_exact_sign`, conjugation), `divided_by`, `radical`, `radical_sum`,
//! `_pick_prime`. They slot in on top of [`SqrtSum::sign_prefilter`], [`integer_form`], [`reduced_form`],
//! [`multiply_integer_items`], [`scaled_by_reciprocal`] and [`conjugate_items`].

use std::cmp::Ordering;
use std::sync::OnceLock;

use crate::num::{self, IBig, UBig};
use crate::products::{accumulate_products, Accumulator, Items, ProductMemo};
use crate::rat::{Coef, Rat, ZeroDivision};

/// `SqrtSumV1.sign` / `_filtered_sign` counters (`SIGN_COUNTS`), kept as a value so a caller can take a delta.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct SignCounts {
    pub total: u64,
    pub closed_rational_zero: u64,
    pub closed_rational_nonzero: u64,
    pub closed_by_enclosure: u64,
    pub closed_by_conjugation: u64,
}

impl SignCounts {
    /// The five counters in the order of the Python dictionary.
    pub fn as_array(&self) -> [u64; 5] {
        [self.total, self.closed_rational_zero, self.closed_rational_nonzero, self.closed_by_enclosure, self.closed_by_conjugation]
    }
}

/// The width of the enclosure filter shared by `SqrtSumV1.sign` and `_filtered_sign` (`SIGN_FILTER_BITS`).
pub const SIGN_FILTER_BITS: usize = 64;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Term {
    pub radicand: UBig,
    pub coef: Coef,
}

/// A sum that is not in canonical form (`require_canonical`'s cheap layer): radicands must be strictly
/// increasing from one and coefficients non-zero.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NonCanonical {
    pub index: usize,
    pub reason: &'static str,
}

/// `(L, [(m, a_m)])` with `c_m = a_m / L`, `L` the least common denominator of the coefficients.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IntForm {
    pub common: UBig,
    pub items: Items,
}

impl IntForm {
    /// The canonical value `sum a_m sqrt(m) / L`: radicands ascending, zero numerators dropped, one
    /// `Fraction(a_m, L)` normalisation per term. Every value that leaves the core goes through here (or its
    /// twin in `fused`), which is what lets everything before it stay unreduced.
    pub fn into_sqrt_sum(self) -> SqrtSum {
        let IntForm { common, mut items } = self;
        if !items.windows(2).all(|pair| pair[0].0 < pair[1].0) {
            items.sort_by(|left, right| left.0.cmp(&right.0));
        }
        let terms = items
            .into_iter()
            .filter(|(_, value)| !value.is_zero())
            .map(|(radicand, value)| Term { radicand, coef: Coef::fraction(Rat::reduced(value, common.clone())) })
            .collect();
        SqrtSum::from_terms_unchecked(terms)
    }
}

/// A sqrt-sum value. Immutable: the integer form and the binary64 measure are cached on first use, which
/// affects nothing but cost.
#[derive(Debug, Clone)]
pub struct SqrtSum {
    terms: Vec<Term>,
    int_form: OnceLock<IntForm>,
    measure: OnceLock<Option<(f64, f64)>>,
}

/// Strict equality: same terms, same coefficient values and the same Python types.
impl PartialEq for SqrtSum {
    fn eq(&self, other: &SqrtSum) -> bool {
        self.terms == other.terms
    }
}

impl Eq for SqrtSum {}

/// Outcome of the part of `SqrtSumV1.sign` that needs no conjugation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignStage {
    Decided(i8),
    /// The enclosure did not decide; the oracle counts `closed_by_conjugation` and calls `_exact_sign`.
    NeedsConjugation,
}

impl SqrtSum {
    // ---- construction ----------------------------------------------------

    pub fn zero() -> SqrtSum {
        SqrtSum::from_terms_unchecked(Vec::new())
    }

    /// `SqrtSumV1.rational(value)`: `Fraction(value)` at radicand one (or the zero sum).
    pub fn rational(value: &Rat) -> SqrtSum {
        if value.is_zero() {
            return SqrtSum::zero();
        }
        SqrtSum::from_terms_unchecked(vec![Term { radicand: UBig::ONE, coef: Coef::fraction(value.clone()) }])
    }

    /// Terms the caller guarantees canonical (sorted, distinct, non-zero): no check beyond a debug assertion.
    pub fn from_terms_unchecked(terms: Vec<Term>) -> SqrtSum {
        let value = SqrtSum { terms, int_form: OnceLock::new(), measure: OnceLock::new() };
        debug_assert!(value.check_canonical().is_ok());
        value
    }

    /// Terms from outside the core: refused with a named reason unless canonical.
    pub fn from_terms(terms: Vec<Term>) -> Result<SqrtSum, NonCanonical> {
        let value = SqrtSum { terms, int_form: OnceLock::new(), measure: OnceLock::new() };
        value.check_canonical()?;
        Ok(value)
    }

    fn check_canonical(&self) -> Result<(), NonCanonical> {
        let mut previous: Option<&UBig> = None;
        for (index, term) in self.terms.iter().enumerate() {
            if term.radicand.is_zero() {
                return Err(NonCanonical { index, reason: "radicand zero" });
            }
            if let Some(before) = previous {
                if term.radicand <= *before {
                    return Err(NonCanonical { index, reason: "radicands not strictly increasing" });
                }
            }
            if term.coef.is_zero() {
                return Err(NonCanonical { index, reason: "zero coefficient" });
            }
            previous = Some(&term.radicand);
        }
        Ok(())
    }

    // ---- reading ---------------------------------------------------------

    pub fn terms(&self) -> &[Term] {
        &self.terms
    }

    pub fn into_terms(self) -> Vec<Term> {
        self.terms
    }

    /// The integer form of the terms, computed once.
    pub fn int_form(&self) -> &IntForm {
        self.int_form.get_or_init(|| integer_form(&self.terms))
    }

    /// The cached binary64 `(centre, bound)` of the value (`float_filter` owns the computation).
    pub fn float_measure(&self, compute: impl FnOnce(&[Term]) -> Option<(f64, f64)>) -> Option<(f64, f64)> {
        *self.measure.get_or_init(|| compute(&self.terms))
    }

    /// `not self.terms`.
    pub fn is_zero(&self) -> bool {
        self.terms.is_empty()
    }

    /// `all(radicand == 1 ...)` (true for zero as well).
    pub fn is_rational(&self) -> bool {
        self.terms.iter().all(|term| term.radicand.is_one())
    }

    /// `as_rational()`: the coefficient object at radicand one (type kept; `Fraction(0)` when there is none),
    /// or `None` when some radicand is not one.
    pub fn as_rational(&self) -> Option<Coef> {
        if !self.is_rational() {
            return None;
        }
        Some(self.terms.first().map_or_else(|| Coef::fraction(Rat::zero()), |term| term.coef.clone()))
    }

    // ---- arithmetic (SqrtSumV1.__add__, __sub__, __neg__, scaled, ...) -----

    /// `self + other`. Coefficients present on both sides add (`int + int` stays an `int`), those only in
    /// `other` become `Fraction`s, those only in `self` are kept as they are; zeros are dropped.
    pub fn add(&self, other: &SqrtSum) -> SqrtSum {
        self.merged(other, false)
    }

    /// `self - other`, typed like [`SqrtSum::add`].
    pub fn sub(&self, other: &SqrtSum) -> SqrtSum {
        self.merged(other, true)
    }

    fn merged(&self, other: &SqrtSum, subtract: bool) -> SqrtSum {
        let mut out: Vec<Term> = Vec::with_capacity(self.terms.len() + other.terms.len());
        let keep = |out: &mut Vec<Term>, term: &Term| {
            if !term.coef.is_zero() {
                out.push(term.clone());
            }
        };
        let take_other = |out: &mut Vec<Term>, term: &Term| {
            let coef = if subtract { term.coef.neg().promoted() } else { term.coef.promoted() };
            if !coef.is_zero() {
                out.push(Term { radicand: term.radicand.clone(), coef });
            }
        };
        let (mut left, mut right) = (0, 0);
        while left < self.terms.len() && right < other.terms.len() {
            let (own, theirs) = (&self.terms[left], &other.terms[right]);
            match own.radicand.cmp(&theirs.radicand) {
                Ordering::Less => {
                    keep(&mut out, own);
                    left += 1;
                }
                Ordering::Greater => {
                    take_other(&mut out, theirs);
                    right += 1;
                }
                Ordering::Equal => {
                    let coef = if subtract { own.coef.sub(&theirs.coef) } else { own.coef.add(&theirs.coef) };
                    if !coef.is_zero() {
                        out.push(Term { radicand: own.radicand.clone(), coef });
                    }
                    left += 1;
                    right += 1;
                }
            }
        }
        for own in &self.terms[left..] {
            keep(&mut out, own);
        }
        for theirs in &other.terms[right..] {
            take_other(&mut out, theirs);
        }
        SqrtSum::from_terms_unchecked(out)
    }

    /// `-self`: no filtering, types kept.
    pub fn neg(&self) -> SqrtSum {
        let terms = self.terms.iter().map(|term| Term { radicand: term.radicand.clone(), coef: term.coef.neg() }).collect();
        SqrtSum::from_terms_unchecked(terms)
    }

    /// `self.scaled(factor)`: every coefficient becomes a `Fraction`; a zero factor gives the zero sum.
    pub fn scaled(&self, factor: &Rat) -> SqrtSum {
        if factor.is_zero() {
            return SqrtSum::zero();
        }
        let terms = self
            .terms
            .iter()
            .map(|term| Term { radicand: term.radicand.clone(), coef: Coef::fraction(term.coef.value().mul(factor)) })
            .collect();
        SqrtSum::from_terms_unchecked(terms)
    }

    /// `self*factor - other*other_factor` in one pass over the integer forms.
    pub fn scaled_difference(&self, factor: &Rat, other: &SqrtSum, other_factor: &Rat) -> SqrtSum {
        scaled_difference_parts(self, factor, other, other_factor).into_sqrt_sum()
    }

    /// `(self - other).is_zero` without building the difference.
    pub fn difference_is_zero(&self, other: &SqrtSum) -> bool {
        let one = Rat::one();
        scaled_difference_parts(self, &one, other, &one).items.is_empty()
    }

    /// The integer filter of `SqrtSumV1.difference_sign`: `Some(sign)` when it decides (counters bumped as the
    /// oracle bumps them), `None` when the question goes on to the exact path (counters untouched).
    pub fn difference_filtered_sign(&self, other: &SqrtSum, counts: &mut SignCounts) -> Option<i8> {
        let one = Rat::one();
        filtered_sign(&scaled_difference_parts(self, &one, other, &one).items, SIGN_FILTER_BITS, counts)
    }

    /// `self * other`. Radicands: `sqrt(a)*sqrt(b) = g*sqrt(a*b/g^2)`; every coefficient a `Fraction`.
    pub fn mul(&self, other: &SqrtSum, memo: &mut ProductMemo) -> SqrtSum {
        if self.terms.is_empty() || other.terms.is_empty() {
            return SqrtSum::zero();
        }
        let (left, right) = (self.int_form(), other.int_form());
        let common = &left.common * &right.common;
        IntForm { common, items: multiply_integer_items(&left.items, &right.items, memo) }.into_sqrt_sum()
    }

    // ---- decisions -------------------------------------------------------

    /// `enclosure(bits)`: `(lo, hi)` with `lo <= value <= hi`, exact rationals.
    pub fn enclosure(&self, bits: usize) -> (Rat, Rat) {
        let form = self.int_form();
        let (low, high) = integer_enclosure(&form.items, bits);
        let denominator = &form.common << bits;
        (Rat::reduced(low, denominator.clone()), Rat::reduced(high, denominator))
    }

    /// `certified_sign(bits)`: the sign the enclosure proves, or `None`.
    pub fn certified_sign(&self, bits: usize) -> Option<i8> {
        integer_certified_sign(&self.int_form().items, bits)
    }

    /// `SqrtSumV1.sign` up to (not including) the conjugation: the counters are bumped exactly where the
    /// oracle bumps them, `closed_by_conjugation` included when the enclosure fails.
    pub fn sign_prefilter(&self, filter_bits: usize, counts: &mut SignCounts) -> SignStage {
        counts.total += 1;
        if self.terms.is_empty() {
            counts.closed_rational_zero += 1;
            return SignStage::Decided(0);
        }
        if self.terms.len() == 1 && self.terms[0].radicand.is_one() {
            counts.closed_rational_nonzero += 1;
            return SignStage::Decided(self.terms[0].coef.value().signum());
        }
        if let Some(certified) = self.certified_sign(filter_bits) {
            counts.closed_by_enclosure += 1;
            return SignStage::Decided(certified);
        }
        counts.closed_by_conjugation += 1;
        SignStage::NeedsConjugation
    }
}

// --------------------------------------------------------------------------
// The integer core (`_integer_form` and friends)
// --------------------------------------------------------------------------

/// `_integer_form`: `(L, [(m, a_m)])`, `L` the least common denominator.
pub fn integer_form(terms: &[Term]) -> IntForm {
    let mut common = UBig::ONE;
    for term in terms {
        let denominator = term.coef.value().denominator();
        if !denominator.is_one() {
            common = num::lcm(&common, denominator);
        }
    }
    let items = if common.is_one() {
        terms.iter().map(|term| (term.radicand.clone(), term.coef.value().numerator().clone())).collect()
    } else {
        terms
            .iter()
            .map(|term| {
                let value = term.coef.value();
                let factor = &common / value.denominator();
                (term.radicand.clone(), value.numerator() * factor)
            })
            .collect()
    };
    IntForm { common, items }
}

/// `_integer_enclosure`: bounds of `2^bits * sum a_m sqrt(m)` from the same `isqrt` as `enclosure`.
pub fn integer_enclosure(items: &[(UBig, IBig)], bits: usize) -> (IBig, IBig) {
    let shift = 2 * bits;
    let mut low = IBig::ZERO;
    let mut high = IBig::ZERO;
    for (radicand, numerator) in items {
        if radicand.is_one() {
            let exact = numerator << bits;
            low += &exact;
            high += exact;
            continue;
        }
        let floor_root = IBig::from(num::isqrt(&(radicand << shift)));
        let ceiling_root = &floor_root + IBig::ONE;
        if numerator > &IBig::ZERO {
            low += numerator * &floor_root;
            high += numerator * ceiling_root;
        } else {
            low += numerator * ceiling_root;
            high += numerator * &floor_root;
        }
    }
    (low, high)
}

/// `_integer_certified_sign`.
pub fn integer_certified_sign(items: &[(UBig, IBig)], bits: usize) -> Option<i8> {
    let (low, high) = integer_enclosure(items, bits);
    if low > IBig::ZERO {
        Some(1)
    } else if high < IBig::ZERO {
        Some(-1)
    } else {
        None
    }
}

/// `_filtered_sign`: the sign of `sum a_m sqrt(m)` if what `sign` decides before conjugation decides it.
/// The counters move as in `SqrtSumV1.sign`; `None` leaves them alone.
pub fn filtered_sign(items: &[(UBig, IBig)], filter_bits: usize, counts: &mut SignCounts) -> Option<i8> {
    if items.is_empty() {
        counts.total += 1;
        counts.closed_rational_zero += 1;
        return Some(0);
    }
    if items.len() == 1 && items[0].0.is_one() {
        counts.total += 1;
        counts.closed_rational_nonzero += 1;
        return Some(num::signum(&items[0].1));
    }
    let certified = integer_certified_sign(items, filter_bits);
    if certified.is_some() {
        counts.total += 1;
        counts.closed_by_enclosure += 1;
    }
    certified
}

/// `_scaled_difference_parts`: `plus*plus_factor - minus*minus_factor = sum a_m sqrt(m) / D`.
///
/// `D` is positive, so sign and zero-ness read off the integers. The merge repeats
/// `scaled(...) - scaled(...)`: `plus` terms first (in their order), then the radicands only `minus` has;
/// zero numerators are dropped. The factors may be integral.
pub fn scaled_difference_parts(plus: &SqrtSum, plus_factor: &Rat, minus: &SqrtSum, minus_factor: &Rat) -> IntForm {
    let (plus_form, minus_form) = (plus.int_form(), minus.int_form());
    let plus_scale = &plus_form.common * plus_factor.denominator();
    let minus_scale = &minus_form.common * minus_factor.denominator();
    let big = num::lcm(&plus_scale, &minus_scale);
    let plus_multiplier = plus_factor.numerator() * (&big / &plus_scale);
    let minus_multiplier = minus_factor.numerator() * (&big / &minus_scale);
    let mut merged: Items = plus_form.items.iter().map(|(radicand, numerator)| (radicand.clone(), numerator * &plus_multiplier)).collect();
    let plus_count = merged.len();
    for (radicand, numerator) in &minus_form.items {
        let amount = numerator * &minus_multiplier;
        match merged[..plus_count].binary_search_by(|(known, _)| known.cmp(radicand)) {
            Ok(index) => merged[index].1 -= amount,
            Err(_) => merged.push((radicand.clone(), -amount)),
        }
    }
    merged.retain(|(_, value)| !value.is_zero());
    IntForm { common: big, items: merged }
}

/// `_multiply_integer_items`: the product of two `sum a_m sqrt(m)`; radicands ascending, zeros dropped.
pub fn multiply_integer_items(left: &[(UBig, IBig)], right: &[(UBig, IBig)], memo: &mut ProductMemo) -> Items {
    let mut merged = Accumulator::with_capacity(left.len() + right.len());
    accumulate_products(&mut merged, left, right, &IBig::ONE, memo);
    merged.into_nonzero_items()
}

/// `_reduced_form`: the same set `(L, a_m)` divided by their common divisor; the value is unchanged.
pub fn reduced_form(common: &UBig, items: &[(UBig, IBig)]) -> IntForm {
    let mut divisor = common.clone();
    for (_, value) in items {
        if divisor.is_one() {
            break;
        }
        divisor = num::gcd_mixed(value, &divisor);
    }
    if divisor.is_one() {
        return IntForm { common: common.clone(), items: items.to_vec() };
    }
    IntForm {
        common: common / &divisor,
        items: items.iter().map(|(radicand, value)| (radicand.clone(), value / &divisor)).collect(),
    }
}

/// `_scaled_by_reciprocal`: `numerator / rational` for a rational denominator, one fraction per term.
pub fn scaled_by_reciprocal(
    numerator_common: &UBig,
    numerator_items: &[(UBig, IBig)],
    denominator_common: &UBig,
    denominator_items: &[(UBig, IBig)],
) -> Result<SqrtSum, ZeroDivision> {
    let head = denominator_items.first().map_or(IBig::ZERO, |(_, value)| value.clone());
    if denominator_common.is_zero() {
        return Err(ZeroDivision);
    }
    let rational = Rat::reduced(head, denominator_common.clone());
    let factor = Rat::one().div(&rational)?;
    if numerator_common.is_zero() {
        return Err(ZeroDivision);
    }
    let terms = numerator_items
        .iter()
        .map(|(radicand, value)| Term {
            radicand: radicand.clone(),
            coef: Coef::fraction(Rat::reduced(value * factor.numerator(), numerator_common * factor.denominator())),
        })
        .collect();
    Ok(SqrtSum::from_terms_unchecked(terms))
}

/// The conjugate of an integer form by the prime `p` (`E = A + B*sqrt(p)  ->  A - B*sqrt(p)`): the numerators
/// of the radicands divisible by `p` change sign, the radicands stay.
pub fn conjugate_items(items: &[(UBig, IBig)], prime: &UBig) -> Items {
    items
        .iter()
        .map(|(radicand, value)| {
            let divisible = (radicand % prime).is_zero();
            (radicand.clone(), if divisible { -value } else { value.clone() })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rat(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    fn frac(radicand: u64, n: i64, d: i64) -> Term {
        Term { radicand: UBig::from(radicand), coef: Coef::fraction(rat(n, d)) }
    }

    fn int(radicand: u64, n: i64) -> Term {
        Term { radicand: UBig::from(radicand), coef: Coef::int(IBig::from(n)) }
    }

    fn sum(terms: Vec<Term>) -> SqrtSum {
        SqrtSum::from_terms(terms).unwrap()
    }

    #[test]
    fn canonical_form_is_checked_at_the_door() {
        assert!(SqrtSum::from_terms(vec![frac(2, 1, 1), frac(3, 1, 1)]).is_ok());
        assert_eq!(SqrtSum::from_terms(vec![frac(3, 1, 1), frac(2, 1, 1)]).unwrap_err().reason, "radicands not strictly increasing");
        assert_eq!(SqrtSum::from_terms(vec![frac(2, 1, 1), frac(2, 2, 1)]).unwrap_err().index, 1);
        assert_eq!(SqrtSum::from_terms(vec![frac(2, 0, 1)]).unwrap_err().reason, "zero coefficient");
        assert_eq!(SqrtSum::from_terms(vec![frac(0, 1, 1)]).unwrap_err().reason, "radicand zero");
    }

    #[test]
    fn integer_form_uses_the_least_common_denominator() {
        let value = sum(vec![frac(1, 1, 2), frac(2, 1, 3), frac(3, 5, 1)]);
        let form = value.int_form();
        assert_eq!(form.common, UBig::from(6u8));
        assert_eq!(form.items, vec![(UBig::from(1u8), IBig::from(3)), (UBig::from(2u8), IBig::from(2)), (UBig::from(3u8), IBig::from(30))]);
        let integral = sum(vec![int(2, 4), frac(5, -3, 1)]);
        assert_eq!(integral.int_form().common, UBig::ONE);
    }

    #[test]
    fn add_sub_neg_keep_the_python_coefficient_types() {
        let left = sum(vec![int(1, 2), int(2, 3)]);
        let right = sum(vec![int(2, 4), int(5, 1)]);
        let added = left.add(&right);
        // radicand 1 only in left: untouched int; 2 in both: int + int = int; 5 only in right: promoted
        assert_eq!(added.terms()[0], int(1, 2));
        assert_eq!(added.terms()[1], int(2, 7));
        assert_eq!(added.terms()[2], frac(5, 1, 1));
        let subtracted = left.sub(&right);
        assert_eq!(subtracted.terms()[1], int(2, -1));
        assert_eq!(subtracted.terms()[2], frac(5, -1, 1));
        let mixed = sum(vec![frac(2, 4, 1)]);
        assert_eq!(left.add(&mixed).terms()[1], frac(2, 7, 1));
        assert_eq!(left.neg().terms()[0], int(1, -2));
        // cancellation drops the term
        assert!(sum(vec![int(2, 3)]).sub(&sum(vec![int(2, 3)])).is_zero());
        assert!(sum(vec![int(2, 3)]).add(&sum(vec![int(2, -3)])).is_zero());
    }

    #[test]
    fn scaled_and_scaled_difference_give_fractions() {
        let value = sum(vec![int(1, 2), int(2, 3)]);
        let scaled = value.scaled(&rat(1, 2));
        assert_eq!(scaled.terms()[0], frac(1, 1, 1));
        assert_eq!(scaled.terms()[1], frac(2, 3, 2));
        assert!(value.scaled(&Rat::zero()).is_zero());
        let other = sum(vec![int(2, 1), int(3, 1)]);
        let difference = value.scaled_difference(&rat(1, 2), &other, &rat(1, 3));
        // 1/2*(2 + 3 r2) - 1/3*(r2 + r3) = 1 + 7/6 r2 - 1/3 r3
        assert_eq!(difference, sum(vec![frac(1, 1, 1), frac(2, 7, 6), frac(3, -1, 3)]));
        assert_eq!(difference, value.scaled(&rat(1, 2)).sub(&other.scaled(&rat(1, 3))));
    }

    #[test]
    fn products_follow_the_radicand_gcd_rule() {
        let mut memo = ProductMemo::new();
        // (1 + sqrt(2)) * (1 - sqrt(2)) = -1
        let product = sum(vec![int(1, 1), int(2, 1)]).mul(&sum(vec![int(1, 1), int(2, -1)]), &mut memo);
        assert_eq!(product, sum(vec![frac(1, -1, 1)]));
        // sqrt(6) * sqrt(10) = 2 sqrt(15)
        let product = sum(vec![int(6, 1)]).mul(&sum(vec![int(10, 1)]), &mut memo);
        assert_eq!(product, sum(vec![frac(15, 2, 1)]));
        assert!(SqrtSum::zero().mul(&product, &mut memo).is_zero());
    }

    #[test]
    fn enclosure_brackets_and_decides_signs() {
        // sqrt(2) - 1.5 < 0, sqrt(2) - 1.4 > 0
        let below = sum(vec![frac(1, -3, 2), frac(2, 1, 1)]);
        assert_eq!(below.certified_sign(64), Some(-1));
        let above = sum(vec![frac(1, -7, 5), frac(2, 1, 1)]);
        assert_eq!(above.certified_sign(64), Some(1));
        let (low, high) = above.enclosure(64);
        assert!(low <= high);
        assert!(low.signum() > 0);
        // an exact cancellation leaves no term at all, so there is nothing for an enclosure to decide
        let tight = sum(vec![frac(2, 1, 1)]).add(&sum(vec![frac(2, -1, 1)]));
        assert!(tight.is_zero());
    }

    #[test]
    fn sign_prefilter_bumps_the_oracle_counters() {
        let mut counts = SignCounts::default();
        assert_eq!(SqrtSum::zero().sign_prefilter(64, &mut counts), SignStage::Decided(0));
        assert_eq!(sum(vec![frac(1, -5, 7)]).sign_prefilter(64, &mut counts), SignStage::Decided(-1));
        assert_eq!(sum(vec![frac(1, -3, 2), frac(2, 1, 1)]).sign_prefilter(64, &mut counts), SignStage::Decided(-1));
        assert_eq!(counts.as_array(), [3, 1, 1, 1, 0]);
        // sqrt(2) - 6369051672525773/2^52 is about -1e-16: a 2-bit enclosure cannot decide it
        let tiny = sum(vec![frac(1, -6369051672525773, 4503599627370496), frac(2, 1, 1)]);
        let stage = tiny.sign_prefilter(2, &mut counts);
        assert_eq!(stage, SignStage::NeedsConjugation);
        assert_eq!(counts.as_array(), [4, 1, 1, 1, 1]);
    }

    #[test]
    fn filtered_sign_leaves_the_counters_alone_when_it_gives_up() {
        let mut counts = SignCounts::default();
        let items: Items = vec![(UBig::ONE, IBig::from(1)), (UBig::from(2u8), IBig::from(-1))];
        // 1 - sqrt(2): decided by the enclosure at 64 bits
        assert_eq!(filtered_sign(&items, 64, &mut counts), Some(-1));
        assert_eq!(counts.closed_by_enclosure, 1);
        let before = counts;
        assert_eq!(filtered_sign(&items, 0, &mut counts), None, "a zero-bit enclosure cannot decide 1 - sqrt(2)");
        assert_eq!(counts, before);
        assert_eq!(filtered_sign(&[], 64, &mut counts), Some(0));
        assert_eq!(filtered_sign(&[(UBig::ONE, IBig::from(-9))], 64, &mut counts), Some(-1));
    }

    #[test]
    fn scaled_difference_parts_keep_the_plus_then_minus_order() {
        let plus = sum(vec![frac(3, 1, 2), frac(7, 1, 1)]);
        let minus = sum(vec![frac(2, 1, 1), frac(7, 1, 3)]);
        let form = scaled_difference_parts(&plus, &rat(1, 1), &minus, &rat(1, 1));
        // 1/2 r3 + r7 - r2 - 1/3 r7 over 6: radicand 3, 7 (plus order) then 2
        assert_eq!(form.common, UBig::from(6u8));
        assert_eq!(form.items, vec![(UBig::from(3u8), IBig::from(3)), (UBig::from(7u8), IBig::from(4)), (UBig::from(2u8), IBig::from(-6))]);
        assert!(plus.difference_is_zero(&plus));
        assert!(!plus.difference_is_zero(&minus));
    }

    #[test]
    fn reduced_form_and_reciprocal_agree() {
        let items: Items = vec![(UBig::ONE, IBig::from(6)), (UBig::from(2u8), IBig::from(-9))];
        let reduced = reduced_form(&UBig::from(12u8), &items);
        assert_eq!(reduced.common, UBig::from(4u8));
        assert_eq!(reduced.items, vec![(UBig::ONE, IBig::from(2)), (UBig::from(2u8), IBig::from(-3))]);
        // (2 + 3 sqrt(2)) / 4  divided by the rational 3/2
        let numerator: Items = vec![(UBig::ONE, IBig::from(2)), (UBig::from(2u8), IBig::from(3))];
        let denominator: Items = vec![(UBig::ONE, IBig::from(3))];
        let quotient = scaled_by_reciprocal(&UBig::from(4u8), &numerator, &UBig::from(2u8), &denominator).unwrap();
        assert_eq!(quotient, sum(vec![frac(1, 1, 3), frac(2, 1, 2)]));
        assert_eq!(scaled_by_reciprocal(&UBig::ONE, &numerator, &UBig::ONE, &[]), Err(ZeroDivision));
    }

    #[test]
    fn conjugation_flips_the_terms_divisible_by_the_prime() {
        let items: Items = vec![(UBig::ONE, IBig::from(1)), (UBig::from(3u8), IBig::from(2)), (UBig::from(6u8), IBig::from(-5)), (UBig::from(5u8), IBig::from(7))];
        let conjugate = conjugate_items(&items, &UBig::from(3u8));
        assert_eq!(conjugate.iter().map(|(_, value)| value.clone()).collect::<Vec<_>>(), vec![IBig::from(1), IBig::from(-2), IBig::from(5), IBig::from(7)]);
    }
}
