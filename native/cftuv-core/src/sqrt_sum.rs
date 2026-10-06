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

use crate::fx::{self, FxItems};
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

/// `(L, [(m, a_m)])` with `c_m = a_m / L`, `L` the least common denominator of the coefficients (or any common denominator, for a form
/// that was only ever worked on as integers).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct IntForm {
    pub common: UBig,
    pub items: Items,
}

impl IntForm {
    /// The value `sum a_m sqrt(m) / L` as a [`SqrtSum`]: radicands ascending, zero numerators dropped. Nothing is normalised here: the
    /// canonical `Fraction(a_m, L)` terms of the value are made when somebody asks for them ([`SqrtSum::terms`]), and its identity
    /// (lowest terms, one gcd chain for the whole value) when somebody asks for that ([`SqrtSum::canonical_form`]).
    pub fn into_sqrt_sum(self) -> SqrtSum {
        let IntForm { mut common, mut items } = self;
        if !items.windows(2).all(|pair| pair[0].0 < pair[1].0) {
            items.sort_by(|left, right| left.0.cmp(&right.0));
        }
        items.retain(|(_, value)| !value.is_zero());
        // Taken to lowest terms here, at the price of one gcd chain (the canonical terms would cost a gcd per term, and only the values that
        // leave the core pay for those): every later operation works on the smallest numbers the value has, and its identity is at hand.
        reduce_in_place(&mut common, &mut items);
        SqrtSum::from_sorted_form(common, items, true)
    }
}

/// A sqrt-sum value. Immutable. It is held as canonical `Fraction` terms (what the oracle's `SqrtSumV1` is) or as an integer form
/// `sum a_m sqrt(m) / L` (what every fused operation produces), and the other one is derived on first use: the terms by reducing
/// each `a_m / L` (a gcd per term), the form by taking the least common denominator. A value that is only ever added, multiplied, divided
/// and signed never pays for its terms. Everything else cached (the identity form, the binary64 measure, the proved sign) affects nothing
/// but cost.
#[derive(Debug, Clone)]
pub struct SqrtSum {
    terms: OnceLock<Vec<Term>>,
    form: OnceLock<IntForm>,
    /// The form in lowest terms (`gcd(L, a_m...) = 1`), derived from `form` when that one is not minimal already.
    canonical: OnceLock<IntForm>,
    /// `form` is in lowest terms the moment it exists (it is derived from canonical terms, or the caller proved it).
    minimal: bool,
    /// Some coefficient is a Python `int` object (only a value made of terms can have one: every form-made coefficient is a `Fraction`).
    py_ints: bool,
    measure: OnceLock<Option<(f64, f64)>>,
    /// `certified_sign(SIGN_FILTER_BITS)`, remembered: a pure function of the value, asked again whenever the same object is signed again.
    certified: OnceLock<Option<i8>>,
}

/// Strict equality: same terms, same coefficient values and the same Python types.
impl PartialEq for SqrtSum {
    fn eq(&self, other: &SqrtSum) -> bool {
        self.terms() == other.terms()
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
        let value = SqrtSum::of_terms(terms);
        debug_assert!(value.check_canonical().is_ok());
        value
    }

    /// Terms from outside the core: refused with a named reason unless canonical.
    pub fn from_terms(terms: Vec<Term>) -> Result<SqrtSum, NonCanonical> {
        let value = SqrtSum::of_terms(terms);
        value.check_canonical()?;
        Ok(value)
    }

    fn of_terms(terms: Vec<Term>) -> SqrtSum {
        let py_ints = terms.iter().any(|term| term.coef.is_py_int());
        SqrtSum { terms: OnceLock::from(terms), form: OnceLock::new(), canonical: OnceLock::new(), minimal: true, py_ints, measure: OnceLock::new(), certified: OnceLock::new() }
    }

    /// A value from an integer form: radicands strictly ascending, numerators non-zero, `common` positive; `minimal` says the form is in
    /// lowest terms already. A value without terms has the common denominator one.
    pub fn from_sorted_form(common: UBig, items: Items, minimal: bool) -> SqrtSum {
        debug_assert!(!common.is_zero());
        debug_assert!(items.windows(2).all(|pair| pair[0].0 < pair[1].0) && items.iter().all(|(radicand, value)| !radicand.is_zero() && !value.is_zero()));
        let common = if items.is_empty() { UBig::ONE } else { common };
        SqrtSum {
            terms: OnceLock::new(),
            form: OnceLock::from(IntForm { common, items }),
            canonical: OnceLock::new(),
            minimal,
            py_ints: false,
            measure: OnceLock::new(),
            certified: OnceLock::new(),
        }
    }

    fn check_canonical(&self) -> Result<(), NonCanonical> {
        let mut previous: Option<&UBig> = None;
        for (index, term) in self.terms().iter().enumerate() {
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

    /// The canonical terms: kept, or made from the integer form on the first ask (one gcd per term, after one gcd chain that takes the
    /// content of the whole form out).
    pub fn terms(&self) -> &[Term] {
        self.terms.get_or_init(|| {
            let form = self.canonical_form();
            let coefficients = lowest_terms(form);
            form.items.iter().zip(coefficients).map(|((radicand, _), rat)| Term { radicand: radicand.clone(), coef: Coef::fraction(rat) }).collect()
        })
    }

    pub fn into_terms(self) -> Vec<Term> {
        self.terms();
        self.terms.into_inner().expect("the terms were just made")
    }

    /// An integer form of the value, computed once: the least common denominator of the terms, or whatever scale the operation that made
    /// the value left it at.
    pub fn int_form(&self) -> &IntForm {
        self.form.get_or_init(|| integer_form(self.terms.get().expect("a sum has terms or a form")))
    }

    /// The identity of the value: its integer form in lowest terms (`gcd(L, a_m...) = 1`, `L` positive), which is unique, so two values are
    /// equal exactly when their canonical forms are. For a form that is not minimal it costs one gcd chain, once.
    pub fn canonical_form(&self) -> &IntForm {
        if self.minimal {
            return self.int_form();
        }
        self.canonical.get_or_init(|| content_reduced(self.int_form()))
    }

    /// The cached binary64 `(centre, bound)` of the value (`float_filter` owns the computation).
    pub fn float_measure(&self, compute: impl FnOnce(&IntForm) -> Option<(f64, f64)>) -> Option<(f64, f64)> {
        *self.measure.get_or_init(|| compute(self.int_form()))
    }

    /// The value has a Python `int` coefficient somewhere (so its terms are not interchangeable with the form's `Fraction`s).
    pub fn has_py_int(&self) -> bool {
        self.py_ints
    }

    /// `not self.terms`.
    pub fn is_zero(&self) -> bool {
        match self.terms.get() {
            Some(terms) => terms.is_empty(),
            None => self.int_form().items.is_empty(),
        }
    }

    /// `all(radicand == 1 ...)` (true for zero as well).
    pub fn is_rational(&self) -> bool {
        match self.terms.get() {
            Some(terms) => terms.iter().all(|term| term.radicand.is_one()),
            None => self.int_form().items.iter().all(|(radicand, _)| radicand.is_one()),
        }
    }

    /// `as_rational()`: the coefficient object at radicand one (type kept; `Fraction(0)` when there is none),
    /// or `None` when some radicand is not one.
    pub fn as_rational(&self) -> Option<Coef> {
        if !self.is_rational() {
            return None;
        }
        Some(self.terms().first().map_or_else(|| Coef::fraction(Rat::zero()), |term| term.coef.clone()))
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
        if !self.py_ints {
            // every coefficient of the result is a `Fraction` (a term only in `self` is one already, the others are promoted or summed with
            // one), so the sum is the same value as one over a common denominator, with nothing to normalise
            let one = Rat::one();
            let other_factor = if subtract { Rat::one() } else { Rat::from_i64(-1) };
            return scaled_difference_parts(self, &one, other, &other_factor).into_sqrt_sum();
        }
        let (own_terms, other_terms) = (self.terms(), other.terms());
        let mut out: Vec<Term> = Vec::with_capacity(own_terms.len() + other_terms.len());
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
        while left < own_terms.len() && right < other_terms.len() {
            let (own, theirs) = (&own_terms[left], &other_terms[right]);
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
        for own in &own_terms[left..] {
            keep(&mut out, own);
        }
        for theirs in &other_terms[right..] {
            take_other(&mut out, theirs);
        }
        SqrtSum::from_terms_unchecked(out)
    }

    /// `-self`: no filtering, types kept.
    pub fn neg(&self) -> SqrtSum {
        if !self.py_ints {
            let form = self.int_form();
            let items = form.items.iter().map(|(radicand, value)| (radicand.clone(), -value)).collect();
            return SqrtSum::from_sorted_form(form.common.clone(), items, self.minimal);
        }
        let terms = self.terms().iter().map(|term| Term { radicand: term.radicand.clone(), coef: term.coef.neg() }).collect();
        SqrtSum::from_terms_unchecked(terms)
    }

    /// `self.scaled(factor)`: every coefficient becomes a `Fraction`; a zero factor gives the zero sum.
    pub fn scaled(&self, factor: &Rat) -> SqrtSum {
        if factor.is_zero() {
            return SqrtSum::zero();
        }
        let form = self.int_form();
        let items = form.items.iter().map(|(radicand, value)| (radicand.clone(), value * factor.numerator())).collect();
        SqrtSum::from_sorted_form(&form.common * factor.denominator(), items, false)
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
        if self.is_zero() || other.is_zero() {
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

    /// [`SqrtSum::enclosure`] without the normalisation of its two endpoints: `(low, high, denominator)` with the endpoints
    /// `low / denominator` and `high / denominator` (the same rationals, not in lowest terms). For a caller that only turns
    /// them into a binary64 (`float(Fraction)` is a correctly rounded division, which does not care how a ratio is written).
    pub fn enclosure_parts(&self, bits: usize) -> (IBig, IBig, UBig) {
        let form = self.int_form();
        let (low, high) = integer_enclosure(&form.items, bits);
        (low, high, &form.common << bits)
    }

    /// `sum(part.scaled(factor) for ...)` taken left to right (`a.scaled(f) + b.scaled(g) + ...`): the canonical value (every
    /// coefficient a `Fraction`, radicands ascending, zeros dropped) with ONE normalisation per result term instead of one per
    /// product and one per sum. A part with a zero factor or no terms contributes nothing, as `scaled` makes it zero.
    pub fn scaled_sum(parts: &[(&SqrtSum, &Rat)]) -> SqrtSum {
        let mut scale = UBig::ONE;
        let mut live = Vec::with_capacity(parts.len());
        for (value, factor) in parts {
            if factor.is_zero() || value.is_zero() {
                continue;
            }
            let form = value.int_form();
            // the coefficient of radicand m in this part is `a_m * p / (L * q)` for the factor `p / q`
            let denominator = &form.common * factor.denominator();
            scale = if scale == denominator { scale } else { num::lcm(&scale, &denominator) };
            live.push((form, factor, denominator));
        }
        let mut merged = Accumulator::new();
        for (form, factor, denominator) in live {
            let weight = IBig::from(&scale / &denominator) * factor.numerator();
            for (radicand, numerator) in &form.items {
                merged.add(radicand, numerator * &weight);
            }
        }
        IntForm { common: scale, items: merged.into_nonzero_items() }.into_sqrt_sum()
    }

    /// `certified_sign(bits)`: the sign the enclosure proves, or `None`.
    pub fn certified_sign(&self, bits: usize) -> Option<i8> {
        if bits == SIGN_FILTER_BITS {
            return *self.certified.get_or_init(|| integer_certified_sign(&self.int_form().items, bits));
        }
        integer_certified_sign(&self.int_form().items, bits)
    }

    /// `SqrtSumV1.sign` up to (not including) the conjugation: the counters are bumped exactly where the
    /// oracle bumps them, `closed_by_conjugation` included when the enclosure fails.
    pub fn sign_prefilter(&self, filter_bits: usize, counts: &mut SignCounts) -> SignStage {
        counts.total += 1;
        let form = self.int_form();
        if form.items.is_empty() {
            counts.closed_rational_zero += 1;
            return SignStage::Decided(0);
        }
        if form.items.len() == 1 && form.items[0].0.is_one() {
            counts.closed_rational_nonzero += 1;
            return SignStage::Decided(num::signum(&form.items[0].1));
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
    if let (Some(fast_left), Some(fast_right)) = (FxItems::from_items(left), FxItems::from_items(right)) {
        if let Some(product) = fx::multiply(&fast_left, &fast_right, memo) {
            return product.to_items();
        }
    }
    multiply_integer_items_dashu(left, right, memo)
}

/// [`multiply_integer_items`] on `dashu-int` integers only (what the stack road falls back to, and what it is held equal to).
pub fn multiply_integer_items_dashu(left: &[(UBig, IBig)], right: &[(UBig, IBig)], memo: &mut ProductMemo) -> Items {
    let mut merged = Accumulator::with_capacity(left.len() + right.len());
    accumulate_products(&mut merged, left, right, &IBig::ONE, memo);
    merged.into_nonzero_items()
}

/// `_reduced_form`: the same set `(L, a_m)` divided by their common divisor; the value is unchanged.
pub fn reduced_form(common: &UBig, items: &[(UBig, IBig)]) -> IntForm {
    let (mut common, mut items) = (common.clone(), items.to_vec());
    reduce_in_place(&mut common, &mut items);
    IntForm { common, items }
}

/// The coefficients `a_m / L` of a form in lowest terms (`gcd(L, a_m...) = 1`) as canonical fractions. Each fraction needs `gcd(a_m, L)`; for a wide
/// form that is a gcd of two wide numbers per term, so the terms are first tried together: `gcd(a_m, L)` divides `G = gcd(prod a_m mod L, L)`, which
/// costs a modular product per term and one gcd, and when `G` is one (or small) every term's gcd is known without taking it.
pub fn lowest_terms(form: &IntForm) -> Vec<Rat> {
    let wide = form.common.as_words().len() >= BATCH_WORDS && form.items.len() >= 3;
    if !wide || form.common.is_one() {
        return form.items.iter().map(|(_, value)| Rat::reduced(value.clone(), form.common.clone())).collect();
    }
    let mut product = num::magnitude(&form.items[0].1) % &form.common;
    for (_, value) in &form.items[1..] {
        product = (product * num::magnitude(value)) % &form.common;
    }
    let shared = num::gcd(&product, &form.common);
    if shared.is_one() {
        return form.items.iter().map(|(_, value)| Rat::from_canonical(value.clone(), form.common.clone())).collect();
    }
    form.items
        .iter()
        .map(|(_, value)| {
            let divisor = num::gcd_mixed(value, &shared);
            if divisor.is_one() {
                Rat::from_canonical(value.clone(), form.common.clone())
            } else {
                Rat::from_canonical(value / IBig::from(divisor.clone()), &form.common / divisor)
            }
        })
        .collect()
}

/// Forms whose common denominator is at least this many words wide take the batch road of [`lowest_terms`].
const BATCH_WORDS: usize = 4;

/// The form in lowest terms: `gcd(L, a_m...)` taken out of `L` and every numerator (the identity of the value).
pub fn content_reduced(form: &IntForm) -> IntForm {
    reduced_form(&form.common, &form.items)
}

/// [`reduced_form`] in place: `gcd(L, *a_m)` divided out of `L` and every numerator (a divisor of one, or of zero
/// for the degenerate all-zero input the oracle refuses, changes nothing). The gcd is the same number whatever order it is taken in, so it is
/// taken from the shortest operand outwards: once it is a word long, every later step is a single-word remainder.
pub fn reduce_in_place(common: &mut UBig, items: &mut Items) {
    let words = |value: &IBig| value.as_sign_words().1.len();
    let mut shortest: Option<usize> = None; // `None`: the common denominator
    let mut length = common.as_words().len();
    for (index, (_, value)) in items.iter().enumerate() {
        if words(value) < length {
            length = words(value);
            shortest = Some(index);
        }
    }
    let mut divisor = match shortest {
        None => common.clone(),
        Some(index) => {
            let start = num::magnitude(&items[index].1);
            if start.is_one() {
                return;
            }
            num::gcd(&start, common)
        }
    };
    for (index, (_, value)) in items.iter().enumerate() {
        if divisor.is_one() {
            break;
        }
        if Some(index) != shortest {
            divisor = num::gcd_mixed(value, &divisor);
        }
    }
    if divisor.is_one() || divisor.is_zero() {
        return;
    }
    *common = &*common / &divisor;
    for (_, value) in items.iter_mut() {
        *value = &*value / &divisor;
    }
}

/// `_scaled_by_reciprocal`: `numerator / rational` for a rational denominator, one fraction per term.
///
/// The oracle builds `Fraction(a * fn, L * fd)` per term (`fn/fd` the reciprocal in lowest terms, `L` the common
/// denominator) and lets `Fraction` take the gcd of the two big products. The lowest-terms result is unique, so it
/// is reached by a cheaper road: with `g1 = gcd(|fn|, L)`, `fn = g1*b`, `L = g1*c` and `M = c*fd` (all coprime
/// to `b`, since `gcd(b, c) = 1` by the choice of `g1` and `gcd(fn, fd) = 1`), `gcd(a*fn, L*fd) = g1 * gcd(a, M)`,
/// so the term is `(a / g2) * b  /  (M / g2)` with `g2 = gcd(a, M)`: one gcd of the SMALL operands per term and
/// nothing to multiply before it.
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
    if numerator_items.is_empty() {
        // `Fraction(...)` is built per term: with no term there is nothing to refuse
        return Ok(SqrtSum::zero());
    }
    if numerator_common.is_zero() {
        return Err(ZeroDivision);
    }
    let shared = num::gcd(&num::magnitude(factor.numerator()), numerator_common);
    let scaled_factor = factor.numerator() / IBig::from(shared.clone());
    let modulus = (numerator_common / &shared) * factor.denominator();
    let terms = numerator_items
        .iter()
        .map(|(radicand, value)| {
            let divisor = num::gcd_mixed(value, &modulus);
            let numerator = (value / IBig::from(divisor.clone())) * &scaled_factor;
            Term { radicand: radicand.clone(), coef: Coef::fraction(Rat::from_canonical(numerator, &modulus / divisor)) }
        })
        .collect();
    Ok(SqrtSum::from_terms_unchecked(terms))
}

/// [`scaled_by_reciprocal`] without the per-term normalisation: the same value as ONE integer form
/// `(numerator_common * |head|, a_m * denominator_common * sign(head))`, reduced by one common divisor. The
/// canonical value is `form.into_sqrt_sum()`; a caller that only multiplies it on (`fused::product_added_form`)
/// never needs the intermediate canonical terms. The refusals are those of [`scaled_by_reciprocal`].
pub fn scaled_by_reciprocal_form(
    numerator_common: &UBig,
    numerator_items: &[(UBig, IBig)],
    denominator_common: &UBig,
    denominator_items: &[(UBig, IBig)],
) -> Result<IntForm, ZeroDivision> {
    let head = denominator_items.first().map_or(IBig::ZERO, |(_, value)| value.clone());
    if denominator_common.is_zero() || head.is_zero() {
        return Err(ZeroDivision);
    }
    if numerator_items.is_empty() {
        return Ok(IntForm { common: UBig::ONE, items: Items::new() });
    }
    if numerator_common.is_zero() {
        return Err(ZeroDivision);
    }
    let negative = num::is_negative(&head);
    let multiplier = IBig::from(denominator_common.clone());
    let mut items: Items = numerator_items.iter().map(|(radicand, value)| (radicand.clone(), value * &multiplier)).collect();
    if negative {
        for (_, value) in items.iter_mut() {
            *value = -std::mem::take(value);
        }
    }
    let mut common = numerator_common * num::magnitude(&head);
    reduce_in_place(&mut common, &mut items);
    Ok(IntForm { common, items })
}

/// `items * conjugate(items)` for the conjugate by `prime`: `(the terms the prime does not divide)^2 - (the terms it divides)^2`. The cross terms
/// of the product cancel exactly, so each square is formed from the products of its pairs taken once (doubled off the diagonal): the same items as
/// `multiply_integer_items(items, conjugate_items(items, prime))`, with about a third of its products.
pub fn norm_items(items: &[(UBig, IBig)], prime: &UBig, memo: &mut ProductMemo) -> Items {
    let inside: Vec<bool> = items.iter().map(|(radicand, _)| (radicand % prime).is_zero()).collect();
    let mut merged = Accumulator::with_capacity(items.len() * (items.len() + 1) / 2);
    for first in 0..items.len() {
        for second in first..items.len() {
            if inside[second] != inside[first] {
                continue;
            }
            let ((left_radicand, left_value), (right_radicand, right_value)) = (&items[first], &items[second]);
            let mut value = left_value * right_value;
            if second != first {
                value <<= 1usize;
            }
            if inside[first] {
                value = -value;
            }
            if left_radicand.is_one() {
                merged.add(right_radicand, value);
            } else if right_radicand.is_one() {
                merged.add(left_radicand, value);
            } else {
                let (common, radicand) = memo.product_ref(left_radicand, right_radicand);
                merged.add(radicand, if common.is_one() { value } else { value * IBig::from(common.clone()) });
            }
        }
    }
    merged.into_nonzero_items()
}

/// Divides both lists by the gcd of all their numerators (a quotient of the two sums does not change), the gcd taken from the shortest numerator
/// outwards.
pub fn reduce_items_together(first: &mut Items, second: &mut Items) {
    let words = |value: &IBig| value.as_sign_words().1.len();
    let mut shortest: Option<(bool, usize)> = None;
    let mut length = usize::MAX;
    for (which, list) in [(false, &*first), (true, &*second)] {
        for (index, (_, value)) in list.iter().enumerate() {
            if words(value) < length {
                length = words(value);
                shortest = Some((which, index));
            }
        }
    }
    let Some((which, position)) = shortest else { return };
    let start_list = if which { &*second } else { &*first };
    let mut divisor = num::magnitude(&start_list[position].1);
    for (side, list) in [(false, &*first), (true, &*second)] {
        for (index, (_, value)) in list.iter().enumerate() {
            if divisor.is_one() {
                break;
            }
            if side == which && index == position {
                continue;
            }
            divisor = num::gcd_mixed(value, &divisor);
        }
    }
    if divisor.is_one() || divisor.is_zero() {
        return;
    }
    for list in [first, second] {
        for (_, value) in list.iter_mut() {
            *value = &*value / &divisor;
        }
    }
}

/// The lists of a quotient `(sum N_i sqrt(m_i) / n) / (sum D_j sqrt(m_j) / d)` over ONE common denominator: the quotient of the two sums of the
/// returned lists is the same value. Equal denominators change nothing, one that divides the other scales its list by the quotient, otherwise each
/// list is scaled by the other's denominator and the common factor is divided out.
pub fn over_one_denominator_items(numerator_common: &UBig, numerator: &[(UBig, IBig)], denominator_common: &UBig, denominator: &[(UBig, IBig)]) -> (Items, Items) {
    let scaled = |items: &[(UBig, IBig)], factor: &UBig| -> Items {
        let factor = IBig::from(factor.clone());
        items.iter().map(|(radicand, value)| (radicand.clone(), value * &factor)).collect()
    };
    if numerator_common == denominator_common {
        return (numerator.to_vec(), denominator.to_vec());
    }
    if (denominator_common % numerator_common).is_zero() {
        return (scaled(numerator, &(denominator_common / numerator_common)), denominator.to_vec());
    }
    if (numerator_common % denominator_common).is_zero() {
        return (numerator.to_vec(), scaled(denominator, &(numerator_common / denominator_common)));
    }
    let (mut first, mut second) = (scaled(numerator, denominator_common), scaled(denominator, numerator_common));
    reduce_items_together(&mut first, &mut second);
    (first, second)
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

    /// The textbook road of the oracle: one `Fraction(a * fn, L * fd)` per term.
    fn reciprocal_by_the_book(nc: &UBig, ni: &[(UBig, IBig)], dc: &UBig, di: &[(UBig, IBig)]) -> Vec<Rat> {
        let rational = Rat::reduced(di[0].1.clone(), dc.clone());
        let factor = Rat::one().div(&rational).unwrap();
        ni.iter().map(|(_, value)| Rat::reduced(value * factor.numerator(), nc * factor.denominator())).collect()
    }

    #[test]
    fn the_cheaper_reciprocal_road_reaches_the_same_lowest_terms_as_the_textbook_one() {
        let mut state = 0x9e37_79b9_7f4a_7c15u64;
        let mut next = |bound: u64| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state % bound
        };
        for round in 0..3000 {
            // small primes make shared factors between the numerators, the common denominators and the divisor certain
            let pick = |next: &mut dyn FnMut(u64) -> u64| -> UBig { [1u64, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 18, 25, 30, 36, 45][next(16) as usize].into() };
            let nc = &pick(&mut next) * &pick(&mut next) * UBig::from(1 + next(7));
            let dc = &pick(&mut next) * &pick(&mut next) * UBig::from(1 + next(5));
            let head = IBig::from(1 + next(60) as i64) * if next(2) == 0 { IBig::ONE } else { -IBig::ONE };
            let count = 1 + next(5) as usize;
            let items: Items = (0..count)
                .map(|index| {
                    let value = IBig::from(1 + next(2000) as i64) * IBig::from(&pick(&mut next) * &pick(&mut next)) * if next(2) == 0 { IBig::ONE } else { -IBig::ONE };
                    (UBig::from(index as u64 + 1), value)
                })
                .collect();
            let divisor: Items = vec![(UBig::ONE, head)];
            let got = scaled_by_reciprocal(&nc, &items, &dc, &divisor).unwrap();
            let want = reciprocal_by_the_book(&nc, &items, &dc, &divisor);
            let got_rats: Vec<Rat> = got.terms().iter().map(|term| term.coef.value().clone()).collect();
            assert_eq!(got_rats, want, "round {round}");
            assert!(got.terms().iter().all(|term| Rat::is_canonical(term.coef.value().numerator(), term.coef.value().denominator())));
        }
    }

    #[test]
    fn the_batch_road_of_lowest_terms_gives_the_fractions_the_per_term_road_gives() {
        let mut state = 0x1357_9bdf_0246_8ace_u64;
        let mut next = |bound: u64| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state % bound
        };
        let wide = |next: &mut dyn FnMut(u64) -> u64| -> UBig {
            let mut value = UBig::ONE;
            for _ in 0..(5 + next(4)) {
                value = value * UBig::from(next(u64::MAX - 1) | 1) + UBig::from(next(1000));
            }
            value
        };
        for round in 0..400 {
            // a wide common denominator, numerators that share a small factor with it (sometimes) or a big one (rarely)
            let common = wide(&mut next) * UBig::from(1 + next(60));
            let shared = [UBig::ONE, UBig::from(1 + next(60)), wide(&mut next)][next(3) as usize].clone();
            let items: Items = (0..(3 + next(5)))
                .map(|index| {
                    let base = wide(&mut next);
                    let numerator = if next(3) == 0 { base * &shared } else { base };
                    (UBig::from(index + 1), IBig::from(numerator) * if next(2) == 0 { IBig::ONE } else { -IBig::ONE })
                })
                .collect();
            let form = content_reduced(&IntForm { common, items });
            if form.common.as_words().len() < BATCH_WORDS {
                continue;
            }
            let expected: Vec<Rat> = form.items.iter().map(|(_, value)| Rat::reduced(value.clone(), form.common.clone())).collect();
            assert_eq!(lowest_terms(&form), expected, "round {round}");
        }
    }

    /// A random sum of `Fraction` terms over a small pool of radicands, and the same value as a map for the reference arithmetic.
    fn random_sum(state: &mut u64) -> (SqrtSum, std::collections::BTreeMap<UBig, Rat>) {
        let mut next = |bound: u64| {
            *state ^= *state << 13;
            *state ^= *state >> 7;
            *state ^= *state << 17;
            *state % bound
        };
        const POOL: [u64; 8] = [1, 2, 3, 5, 6, 7, 10, 15];
        let mut map = std::collections::BTreeMap::new();
        for _ in 0..(next(5) + 1) {
            let radicand = UBig::from(POOL[next(POOL.len() as u64) as usize]);
            let numerator = IBig::from(next(2000) as i64 - 1000) * if next(4) == 0 { IBig::from(next(u64::MAX - 1) | 1) } else { IBig::ONE };
            let value = Rat::new(numerator, IBig::from(1 + next(90) as i64)).unwrap();
            if !value.is_zero() {
                map.insert(radicand, value);
            }
        }
        let terms = map.iter().map(|(radicand, value)| Term { radicand: radicand.clone(), coef: Coef::fraction(value.clone()) }).collect();
        (SqrtSum::from_terms(terms).unwrap(), map)
    }

    fn add_to(map: &mut std::collections::BTreeMap<UBig, Rat>, radicand: UBig, value: Rat) {
        let total = map.get(&radicand).map_or(value.clone(), |old| old.add(&value));
        if total.is_zero() {
            map.remove(&radicand);
        } else {
            map.insert(radicand, total);
        }
    }

    fn as_sum(map: &std::collections::BTreeMap<UBig, Rat>) -> SqrtSum {
        SqrtSum::from_terms(map.iter().map(|(radicand, value)| Term { radicand: radicand.clone(), coef: Coef::fraction(value.clone()) }).collect()).unwrap()
    }

    #[test]
    fn values_made_from_forms_equal_the_values_made_from_terms() {
        let mut state = 0x6a09_e667_f3bc_c908u64;
        let mut memo = ProductMemo::new();
        for round in 0..1500 {
            let ((a, left), (b, right)) = (random_sum(&mut state), random_sum(&mut state));
            let factor = Rat::new(IBig::from(round % 17 - 8), IBig::from(1 + round % 11)).unwrap();
            // sums and differences: the terms are exactly the reference terms, every coefficient a `Fraction`
            let mut sum = left.clone();
            let mut difference = left.clone();
            for (radicand, value) in &right {
                add_to(&mut sum, radicand.clone(), value.clone());
                add_to(&mut difference, radicand.clone(), value.neg());
            }
            assert_eq!(a.add(&b), as_sum(&sum), "round {round}: add");
            assert_eq!(a.sub(&b), as_sum(&difference), "round {round}: sub");
            assert_eq!(a.neg(), as_sum(&left.iter().map(|(radicand, value)| (radicand.clone(), value.neg())).collect()));
            // scaling
            let scaled: std::collections::BTreeMap<UBig, Rat> = left.iter().filter(|_| !factor.is_zero()).map(|(radicand, value)| (radicand.clone(), value.mul(&factor))).collect();
            assert_eq!(a.scaled(&factor), as_sum(&scaled), "round {round}: scaled");
            // products
            let mut product = std::collections::BTreeMap::new();
            for (left_radicand, left_value) in &left {
                for (right_radicand, right_value) in &right {
                    let (common, radicand) = crate::products::radicand_product(left_radicand, right_radicand);
                    add_to(&mut product, radicand, left_value.mul(right_value).mul(&Rat::from_int(IBig::from(common))));
                }
            }
            let multiplied = a.mul(&b, &mut memo);
            assert_eq!(multiplied, as_sum(&product), "round {round}: mul");
            // the identity of a value is its form in lowest terms whatever made it: the same form for the same value
            assert_eq!(multiplied.canonical_form(), as_sum(&product).canonical_form());
            assert_eq!(a.add(&b).canonical_form(), as_sum(&sum).canonical_form());
            let scaled_back = a.scaled(&factor).scaled(&Rat::one().div(&if factor.is_zero() { Rat::one() } else { factor.clone() }).unwrap());
            if !factor.is_zero() {
                assert_eq!(scaled_back.canonical_form(), a.canonical_form(), "round {round}: scaled twice");
                assert_eq!(scaled_back, a);
            }
            // a combined sum over the same radicands
            assert_eq!(SqrtSum::scaled_sum(&[(&a, &factor), (&b, &Rat::one())]), as_sum(&{
                let mut total = scaled.clone();
                for (radicand, value) in &right {
                    add_to(&mut total, radicand.clone(), value.clone());
                }
                total
            }));
        }
    }

    #[test]
    fn conjugation_flips_the_terms_divisible_by_the_prime() {
        let items: Items = vec![(UBig::ONE, IBig::from(1)), (UBig::from(3u8), IBig::from(2)), (UBig::from(6u8), IBig::from(-5)), (UBig::from(5u8), IBig::from(7))];
        let conjugate = conjugate_items(&items, &UBig::from(3u8));
        assert_eq!(conjugate.iter().map(|(_, value)| value.clone()).collect::<Vec<_>>(), vec![IBig::from(1), IBig::from(-2), IBig::from(5), IBig::from(7)]);
    }
}
