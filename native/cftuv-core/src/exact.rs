//! The cost-bearing operations of `exact_sqrt_sum.py`: `sign` past the enclosure filter (`_exact_sign`),
//! `divided_by` (the integer conjugation loop and its generic fallback), `_divide_with_prime_universe`,
//! `radical`, `radical_sum`, `prime_universe_remembered`, and the two memory-backed primitives under them.
//!
//! Equality with the oracle covers the answer AND the cost, so every function here follows the Python call order
//! wherever that order is observable: which radicands reach `prime_support` / `squarefree_split` and when
//! (a miss pays the budget and may factor), which sign counters move before an exhaustion, which table entries
//! exist when an exhaustion cuts the operation short. The numbers in between are free: the recursion and the
//! conjugation loop work on unreduced integer forms (the common denominator of a sign question is irrelevant,
//! the enclosure and every other decision are invariant under positive scaling) and reduce only where Python
//! reduces (`_reduced_form`, one `Fraction` per result term).
//!
//! Budget, memory, sign counters and the product memory are explicit `&mut` parameters ([`ExactCtx`]). An
//! exhaustion is a `Result::Err` carrying `(operation, radicand)`; every partial effect (articles, table entries,
//! counters) stays exactly where Python's exception would have left it.

use cftuv_canon::{pick_prime_from_universe, CanonError, CanonMemory, QValue, UniverseRecord, WorkBudget};

use crate::fused::Share;
use crate::fx::{self, ubig_of_u128, FxItems};
use crate::num::{self, IBig, UBig};
use crate::products::{Items, ProductMemo};
use crate::rat::{Coef, Rat};
use crate::sqrt_sum::{
    conjugate_items, integer_certified_sign, multiply_integer_items, multiply_integer_items_dashu, reduce_in_place, scaled_by_reciprocal,
    scaled_by_reciprocal_form, IntForm, SignCounts, SignStage, SqrtSum, Term, SIGN_FILTER_BITS,
};
use crate::sqrt_sum::{norm_items, over_one_denominator_items, reduce_items_together};
use crate::wide::Wide;

/// Rounds and coefficient size the generic division fallback may reach before it is refused by name. The oracle
/// loops without a bound: on a denominator that is not canonical (a radicand with a square) the loop never reaches
/// a rational denominator and its numbers DOUBLE in size every round, so Python runs out of memory. A legitimate
/// division needs about one round per prime of the support and stays far below both limits.
pub const MAX_GENERIC_ROUNDS: usize = 64;
pub const MAX_GENERIC_BITS: usize = 1 << 20;

/// What an exact operation can refuse with. The host raises the matching Python exception.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExactError {
    /// Budget exhaustion, negative radicand, failed reconstruction, invalid mirror input.
    Canon(CanonError),
    /// `ZeroSqrtSumDivisorError`: division by an exact zero.
    ZeroDivisor,
    /// The generic division fallback passed [`MAX_GENERIC_ROUNDS`] or [`MAX_GENERIC_BITS`] (Python would loop on).
    Diverged,
    /// A state the oracle can only reach by an exception that is not part of its contract (`TypeError`,
    /// `KeyError`, `ZeroDivisionError` from a `Fraction`).
    Internal(&'static str),
}

impl From<CanonError> for ExactError {
    fn from(error: CanonError) -> ExactError {
        ExactError::Canon(error)
    }
}

impl From<cftuv_canon::Exhausted> for ExactError {
    fn from(error: cftuv_canon::Exhausted) -> ExactError {
        ExactError::Canon(CanonError::Exhausted(error))
    }
}

/// Everything an exact operation reads and writes besides its operands.
pub struct ExactCtx<'a> {
    pub memory: &'a mut CanonMemory,
    pub budget: &'a mut WorkBudget,
    pub counts: &'a mut SignCounts,
    pub products: &'a mut ProductMemo,
}

// --------------------------------------------------------------------------
// `_pick_prime`, `_split_by_prime`
// --------------------------------------------------------------------------

/// `_pick_prime(terms)`: the smallest prime of the support of every non-zero radicand above one. Each radicand
/// goes through `prime_support` in the order given (ascending for every dictionary the oracle builds), so memory
/// misses, their budget and their factorizations fall exactly where Python's fall.
pub fn pick_prime(ctx: &mut ExactCtx<'_>, items: &[(UBig, IBig)]) -> Result<Option<UBig>, ExactError> {
    pick_prime_over(ctx, items.iter().filter(|(_, value)| !value.is_zero()).map(|(radicand, _)| radicand.clone()))
}

/// [`pick_prime`] over the radicands alone (their numerators are known to be non-zero).
fn pick_prime_over(ctx: &mut ExactCtx<'_>, radicands: impl Iterator<Item = UBig>) -> Result<Option<UBig>, ExactError> {
    let mut smallest: Option<UBig> = None;
    for radicand in radicands {
        if radicand <= UBig::ONE {
            continue;
        }
        if let Some(first) = ctx.memory.smallest_support_prime(&radicand, ctx.budget)? {
            if smallest.as_ref().is_none_or(|current| first < *current) {
                smallest = Some(first);
            }
        }
    }
    Ok(smallest)
}

/// `_split_by_prime`: `E = A + B*sqrt(p)`, returned as `(A, B)` with `B` keyed by `radicand / p`. The radicands
/// of a dictionary are distinct, hence so are the quotients: nothing merges.
fn split_by_prime(items: &[(UBig, IBig)], prime: &UBig) -> (Items, Items) {
    let (mut outside, mut inside) = (Items::new(), Items::new());
    for (radicand, value) in items {
        if (radicand % prime).is_zero() {
            inside.push((radicand / prime, value.clone()));
        } else {
            outside.push((radicand.clone(), value.clone()));
        }
    }
    (outside, inside)
}

/// The same split on real terms (the generic division fallback needs the actual coefficients).
fn split_terms_by_prime(terms: &[Term], prime: &UBig) -> (Vec<Term>, Vec<Term>) {
    let (mut outside, mut inside) = (Vec::new(), Vec::new());
    for term in terms {
        let coef = Coef::fraction(term.coef.value().clone());
        if (&term.radicand % prime).is_zero() {
            inside.push(Term { radicand: &term.radicand / prime, coef });
        } else {
            outside.push(Term { radicand: term.radicand.clone(), coef });
        }
    }
    (outside, inside)
}

/// `left - factor * right` for two ascending item lists; zeros dropped, ascending kept.
fn subtract_scaled(left: &[(UBig, IBig)], right: &[(UBig, IBig)], factor: &UBig) -> Items {
    let factor = IBig::from(factor.clone());
    let mut out = Items::with_capacity(left.len() + right.len());
    let (mut first, mut second) = (0, 0);
    while first < left.len() || second < right.len() {
        let order = match (left.get(first), right.get(second)) {
            (Some((own, _)), Some((theirs, _))) => own.cmp(theirs),
            (Some(_), None) => std::cmp::Ordering::Less,
            _ => std::cmp::Ordering::Greater,
        };
        match order {
            std::cmp::Ordering::Less => {
                out.push(left[first].clone());
                first += 1;
            }
            std::cmp::Ordering::Greater => {
                out.push((right[second].0.clone(), -(&right[second].1 * &factor)));
                second += 1;
            }
            std::cmp::Ordering::Equal => {
                let value = &left[first].1 - &right[second].1 * &factor;
                if !value.is_zero() {
                    out.push((left[first].0.clone(), value));
                }
                first += 1;
                second += 1;
            }
        }
    }
    out
}

// --------------------------------------------------------------------------
// sign
// --------------------------------------------------------------------------

/// `_exact_sign(terms)` on integer items (the common denominator is dropped: it is positive and every decision
/// here is invariant under positive scaling). `enclosure_failed` says the caller already ran the same enclosure
/// on the same items and it did not decide, so the repeat is skipped (it could only fail again).
///
/// Order (cost-visible): zero filter, `_pick_prime` over all radicands, the enclosure, the split, BOTH
/// recursive signs (outside first), then the discriminant only when the two signs are opposite.
fn exact_sign(ctx: &mut ExactCtx<'_>, items: &[(UBig, IBig)], bits: usize, enclosure_failed: bool) -> Result<i8, ExactError> {
    let filtered: Items;
    let items = if items.iter().any(|(_, value)| value.is_zero()) {
        filtered = items.iter().filter(|(_, value)| !value.is_zero()).cloned().collect();
        &filtered[..]
    } else {
        items
    };
    if items.is_empty() {
        return Ok(0);
    }
    let Some(prime) = pick_prime(ctx, items)? else {
        return match items.iter().find(|(radicand, _)| radicand.is_one()) {
            Some((_, value)) => Ok(num::signum(value)),
            None => Err(ExactError::Internal("a sum without a prime and without a rational term")),
        };
    };
    if !enclosure_failed {
        if let Some(certified) = integer_certified_sign(items, bits) {
            return Ok(certified);
        }
    }
    let (outside, inside) = split_by_prime(items, &prime);
    let outside_sign = exact_sign(ctx, &outside, bits, false)?;
    let inside_sign = exact_sign(ctx, &inside, bits, false)?;
    if inside_sign == 0 {
        return Ok(outside_sign);
    }
    if outside_sign == 0 {
        return Ok(inside_sign);
    }
    if outside_sign == inside_sign {
        return Ok(outside_sign);
    }
    let left = multiply_integer_items(&outside, &outside, ctx.products);
    let right = multiply_integer_items(&inside, &inside, ctx.products);
    let discriminant = subtract_scaled(&left, &right, &prime);
    let discriminant_sign = exact_sign(ctx, &discriminant, bits, false)?;
    if discriminant_sign == 0 {
        return Ok(0);
    }
    Ok(if discriminant_sign > 0 { outside_sign } else { inside_sign })
}

/// The sign of `sum a_m sqrt(m)` on integer items, decided past the enclosure: the conjugation of `_exact_sign`
/// (the caller counted `closed_by_conjugation` and ran the enclosure that did not decide).
pub fn exact_sign_after_enclosure(ctx: &mut ExactCtx<'_>, items: &[(UBig, IBig)], bits: usize) -> Result<i8, ExactError> {
    exact_sign(ctx, items, bits, true)
}

/// `SqrtSumV1.sign` on a value given as integer items (positive scaling of the value changes no decision): the
/// counters, then the enclosure through the memory of floor roots, then the conjugation. Zero numerators must not be
/// among the items.
pub fn sign_items(ctx: &mut ExactCtx<'_>, items: &[(UBig, IBig)], filter_bits: usize) -> Result<i8, ExactError> {
    ctx.counts.total += 1;
    if items.is_empty() {
        ctx.counts.closed_rational_zero += 1;
        return Ok(0);
    }
    if items.len() == 1 && items[0].0.is_one() {
        ctx.counts.closed_rational_nonzero += 1;
        return Ok(num::signum(&items[0].1));
    }
    let shift = 2 * filter_bits;
    let products = &mut *ctx.products;
    let (mut low, mut high) = (IBig::ZERO, IBig::ZERO);
    for (radicand, numerator) in items {
        if radicand.is_one() {
            let exact = numerator << filter_bits;
            low += &exact;
            high += exact;
            continue;
        }
        let floor_root = IBig::from(products.floor_root(radicand, shift));
        let ceiling_root = &floor_root + IBig::ONE;
        if numerator > &IBig::ZERO {
            low += numerator * &floor_root;
            high += numerator * ceiling_root;
        } else {
            low += numerator * ceiling_root;
            high += numerator * &floor_root;
        }
    }
    if let Some(certified) = certify(&low, &high) {
        ctx.counts.closed_by_enclosure += 1;
        return Ok(certified);
    }
    ctx.counts.closed_by_conjugation += 1;
    exact_sign(ctx, items, filter_bits, true)
}

/// The sign an enclosure `[low, high]` proves, or `None`.
pub fn certify(low: &IBig, high: &IBig) -> Option<i8> {
    if *low > IBig::ZERO {
        Some(1)
    } else if *high < IBig::ZERO {
        Some(-1)
    } else {
        None
    }
}

/// `SqrtSumV1.sign(filter_bits=..., budget=...)`: the counters move as the oracle moves them, including
/// `closed_by_conjugation` BEFORE the exact work (it survives an exhaustion).
pub fn sign(ctx: &mut ExactCtx<'_>, value: &SqrtSum, filter_bits: usize) -> Result<i8, ExactError> {
    match value.sign_prefilter(filter_bits, ctx.counts) {
        SignStage::Decided(sign) => Ok(sign),
        SignStage::NeedsConjugation => exact_sign(ctx, &value.int_form().items, filter_bits, true),
    }
}

/// `SqrtSumV1.difference_sign(other, budget)`: the sign of `self - other`. The integer filter reads it off the two
/// integer forms without the difference (counters bumped as `sign` bumps them); when it gives up it leaves every
/// counter alone and the question goes the whole way as `(self - other).sign(budget=budget)`.
pub fn difference_sign(ctx: &mut ExactCtx<'_>, left: &SqrtSum, right: &SqrtSum) -> Result<i8, ExactError> {
    if let Some(decided) = left.difference_filtered_sign(right, ctx.counts) {
        return Ok(decided);
    }
    sign(ctx, &left.sub(right), SIGN_FILTER_BITS)
}

// --------------------------------------------------------------------------
// division
// --------------------------------------------------------------------------

/// Where the conjugation loop takes its prime from.
enum PrimeSource<'a> {
    /// `_pick_prime`: `prime_support` per radicand (factoring on a miss).
    Factorized,
    /// `_pick_prime_from_universe`: divisibility by a known set of primes, no factoring.
    Universe(&'a [UBig]),
}

/// The two integer forms the conjugation loop ends with: `numerator / denominator` where the denominator became
/// rational (`(common, items)` of each).
enum Rationalized {
    Big(BigState),
    Fx(FxState),
}

impl Rationalized {
    fn into_big(self) -> BigState {
        match self {
            Rationalized::Big(state) => state,
            Rationalized::Fx(state) => state.into_big(),
        }
    }
}

/// The loop's integers on the stack: the item lists of the numerator and of the denominator over ONE common denominator. The quotient
/// `(sum N_i sqrt(m_i) / L) / (sum D_j sqrt(m_j) / L)` is `sum N_i sqrt(m_i) / sum D_j sqrt(m_j)`, so that denominator is never carried: the
/// rounds multiply both lists by the same conjugate and may divide both by any common factor.
struct FxState {
    numerator_items: FxItems,
    denominator_items: FxItems,
}

/// The same forms as `dashu-int` integers (the road of operands that do not fit the stack).
struct BigState {
    numerator_common: UBig,
    numerator_items: Items,
    denominator_common: UBig,
    denominator_items: Items,
}

impl FxState {
    fn into_big(self) -> BigState {
        BigState {
            numerator_common: UBig::ONE,
            numerator_items: self.numerator_items.to_items(),
            denominator_common: UBig::ONE,
            denominator_items: self.denominator_items.to_items(),
        }
    }

    /// One conjugation round on the stack: `None` when anything leaves the capacity (the state is untouched, the caller redoes the round on
    /// `dashu-int` integers).
    fn step(&self, prime: &UBig, products: &mut ProductMemo) -> Option<FxState> {
        let word = fx::radicand_u128(prime)?;
        let conjugate = self.denominator_items.flipped(word);
        let mut numerator_items = fx::multiply(&self.numerator_items, &conjugate, products)?;
        let mut denominator_items = fx::norm(&self.denominator_items, word, products)?;
        fx::reduce_together(&mut numerator_items, &mut denominator_items);
        Some(FxState { numerator_items, denominator_items })
    }
}

impl BigState {
    /// One round over lists with ONE common denominator (carried as one: it cancels in the quotient, see `FxState`).
    fn step(&mut self, prime: &UBig, products: &mut ProductMemo) {
        let conjugate = conjugate_items(&self.denominator_items, prime);
        self.numerator_items = multiply_integer_items_dashu(&self.numerator_items, &conjugate, products);
        self.denominator_items = norm_items(&self.denominator_items, prime, products);
        reduce_items_together(&mut self.numerator_items, &mut self.denominator_items);
    }
}

/// Where the rounds are: on the stack while everything fits, on `dashu-int` integers from the first round that does not.
enum RoundState {
    Fx(FxState),
    Big(BigState),
}

impl RoundState {
    fn start(numerator: &SqrtSum, denominator: &SqrtSum) -> RoundState {
        let (numerator_form, denominator_form) = (numerator.int_form(), denominator.int_form());
        if let (Some(numerator_common), Some(numerator_items), Some(denominator_common), Some(denominator_items)) = (
            Wide::from_ubig(&numerator_form.common),
            FxItems::from_items(&numerator_form.items),
            Wide::from_ubig(&denominator_form.common),
            FxItems::from_items(&denominator_form.items),
        ) {
            // over one common denominator: the two are equal, or one divides the other (the dearer list is scaled by the quotient), or each list is
            // taken times the other's and the common factor that makes is divided out
            if let Some((numerator_items, denominator_items)) = fx::over_one_denominator(numerator_items, &numerator_common, denominator_items, &denominator_common) {
                return RoundState::Fx(FxState { numerator_items, denominator_items });
            }
        }
        let (numerator_items, denominator_items) = over_one_denominator_items(&numerator_form.common, &numerator_form.items, &denominator_form.common, &denominator_form.items);
        RoundState::Big(BigState { numerator_common: UBig::ONE, numerator_items, denominator_common: UBig::ONE, denominator_items })
    }

    /// The loop's exit test: the denominator has no term or only the rational one.
    fn denominator_is_rational(&self) -> bool {
        match self {
            RoundState::Fx(state) => state.denominator_items.is_rational(),
            RoundState::Big(state) => state.denominator_items.len() <= 1 && state.denominator_items.iter().all(|(radicand, _)| radicand.is_one()),
        }
    }

    /// The radicands of the denominator, ascending (every numerator is non-zero).
    fn denominator_radicands(&self) -> Vec<UBig> {
        match self {
            RoundState::Fx(state) => (0..state.denominator_items.len()).map(|index| ubig_of_u128(state.denominator_items.key(index))).collect(),
            RoundState::Big(state) => state.denominator_items.iter().map(|(radicand, _)| radicand.clone()).collect(),
        }
    }

    fn step(self, prime: &UBig, products: &mut ProductMemo) -> RoundState {
        match self {
            RoundState::Fx(state) => match state.step(prime, products) {
                Some(next) => RoundState::Fx(next),
                None => {
                    let mut big = state.into_big();
                    big.step(prime, products);
                    RoundState::Big(big)
                }
            },
            RoundState::Big(mut state) => {
                state.step(prime, products);
                RoundState::Big(state)
            }
        }
    }

    fn finish(self) -> Rationalized {
        match self {
            RoundState::Fx(state) => Rationalized::Fx(state),
            RoundState::Big(state) => Rationalized::Big(state),
        }
    }
}

/// The integer conjugation loop shared by `divided_by` and `_divide_with_prime_universe`.
///
/// `Ok(Some(quotient))` when the denominator became rational; `Ok(None)` when the oracle leaves the loop for
/// its fallback (no prime from the universe, or `squarefree_split(prime) != (1, prime)`).
fn conjugation_loop(ctx: &mut ExactCtx<'_>, numerator: &SqrtSum, denominator: &SqrtSum, source: PrimeSource<'_>) -> Result<Option<SqrtSum>, ExactError> {
    match rationalize(ctx, numerator, denominator, source)? {
        Some(done) => {
            let state = done.into_big();
            scaled_by_reciprocal(&state.numerator_common, &state.numerator_items, &state.denominator_common, &state.denominator_items)
                .map(Some)
                .map_err(|_| ExactError::Internal("a rational divisor of zero"))
        }
        None => Ok(None),
    }
}

/// The conjugation rounds themselves (every memory question and budget payment of the oracle's loop, in its order).
fn rationalize(ctx: &mut ExactCtx<'_>, numerator: &SqrtSum, denominator: &SqrtSum, source: PrimeSource<'_>) -> Result<Option<Rationalized>, ExactError> {
    let mut state = RoundState::start(numerator, denominator);
    loop {
        if state.denominator_is_rational() {
            return Ok(Some(state.finish()));
        }
        let radicands = state.denominator_radicands();
        let prime = match &source {
            PrimeSource::Factorized => match pick_prime_over(ctx, radicands.into_iter())? {
                Some(prime) => prime,
                None => return Err(ExactError::Internal("a conjugation without a prime")),
            },
            PrimeSource::Universe(universe) => match pick_prime_from_universe(radicands.iter().map(|radicand| (radicand, true)), universe) {
                Some(prime) => prime,
                None => return Ok(None),
            },
        };
        let (outside, inside) = ctx.memory.squarefree_split_unsigned(&prime, ctx.budget)?;
        if outside != UBig::ONE || inside != prime {
            return Ok(None);
        }
        state = state.step(&prime, ctx.products);
    }
}

/// The widest numerator or denominator among the coefficients of a sum, in bits.
fn largest_bits(value: &SqrtSum) -> usize {
    value.terms().iter().map(|term| num::bit_length(&num::magnitude(term.coef.value().numerator())).max(num::bit_length(term.coef.value().denominator()))).max().unwrap_or(0)
}

/// `_divided_by_generic`: the fallback of the integer loop, on real sums (also an operation of its own, so the
/// fallback is compared with the oracle on inputs where Python reaches it by a direct call).
pub fn divided_by_generic(ctx: &mut ExactCtx<'_>, numerator: &SqrtSum, denominator: &SqrtSum) -> Result<SqrtSum, ExactError> {
    let (mut numerator, mut denominator) = (numerator.clone(), denominator.clone());
    for _ in 0..MAX_GENERIC_ROUNDS {
        if let Some(rational) = denominator.as_rational() {
            let factor = Rat::one().div(rational.value()).map_err(|_| ExactError::Internal("a rational divisor of zero"))?;
            return Ok(numerator.scaled(&factor));
        }
        let items = denominator.int_form().items.clone();
        let Some(prime) = pick_prime(ctx, &items)? else {
            return Err(ExactError::Internal("a conjugation without a prime"));
        };
        let (outside, inside) = split_terms_by_prime(denominator.terms(), &prime);
        let root = radical(ctx, &Rat::one(), &Rat::from_int(IBig::from(prime)))?;
        let conjugate = SqrtSum::from_terms_unchecked(outside).sub(&SqrtSum::from_terms_unchecked(inside).mul(&root, ctx.products));
        numerator = numerator.mul(&conjugate, ctx.products);
        denominator = denominator.mul(&conjugate, ctx.products);
        if largest_bits(&numerator).max(largest_bits(&denominator)) > MAX_GENERIC_BITS {
            return Err(ExactError::Diverged);
        }
    }
    Err(ExactError::Diverged)
}

/// The quotient of [`divided_by_form`]: an unreduced integer form, or (the oracle's generic fallback, dead in
/// practice) the sum the fallback built.
pub enum Quotient {
    Form(Share),
    Sum(SqrtSum),
}

/// `divided_by` that stops before the per-term normalisation of its result: the same loop (so the same memory
/// questions, budget payments and refusals, in the same order), the quotient as an integer form whose canonical value
/// is `divided_by`'s answer. For a caller that multiplies the quotient on at once (`fused::product_added_form`) and
/// so normalises once, not twice.
pub fn divided_by_form(ctx: &mut ExactCtx<'_>, numerator: &SqrtSum, denominator: &SqrtSum) -> Result<Quotient, ExactError> {
    if denominator.is_zero() {
        return Err(ExactError::ZeroDivisor);
    }
    match rationalize(ctx, numerator, denominator, PrimeSource::Factorized)? {
        Some(done) => {
            if let Rationalized::Fx(state) = &done {
                if let Some((common, items)) = fx::scaled_by_reciprocal_form(&state.numerator_items, &state.denominator_items) {
                    return Ok(Quotient::Form(Share::from_stack(common, items)));
                }
            }
            let state = done.into_big();
            scaled_by_reciprocal_form(&state.numerator_common, &state.numerator_items, &state.denominator_common, &state.denominator_items)
                .map(|form| Quotient::Form(Share::from_form(form)))
                .map_err(|_| ExactError::Internal("a rational divisor of zero"))
        }
        None => divided_by_generic(ctx, numerator, denominator).map(Quotient::Sum),
    }
}

/// `SqrtSumV1.divided_by(other, budget)`: division by the conjugates, exact; `ZeroDivisor` for an exact zero.
pub fn divided_by(ctx: &mut ExactCtx<'_>, numerator: &SqrtSum, denominator: &SqrtSum) -> Result<SqrtSum, ExactError> {
    if denominator.is_zero() {
        return Err(ExactError::ZeroDivisor);
    }
    match conjugation_loop(ctx, numerator, denominator, PrimeSource::Factorized)? {
        Some(quotient) => Ok(quotient),
        None => divided_by_generic(ctx, numerator, denominator),
    }
}

/// `_divide_with_prime_universe`: the same conjugations with the prime taken from a proven universe; any radical
/// the universe does not reproduce sends the WHOLE division to `divided_by` on the ORIGINAL operands (partially
/// conjugated values are never mixed with the legacy path), with the budget and memory as the aborted attempt
/// left them.
pub fn divide_with_prime_universe(ctx: &mut ExactCtx<'_>, numerator: &SqrtSum, denominator: &SqrtSum, universe: &[UBig]) -> Result<SqrtSum, ExactError> {
    if denominator.is_zero() {
        return divided_by(ctx, numerator, denominator);
    }
    match conjugation_loop(ctx, numerator, denominator, PrimeSource::Universe(universe))? {
        Some(quotient) => Ok(quotient),
        None => divided_by(ctx, numerator, denominator),
    }
}

/// The part of `_divide_with_prime_universe` that depends on the DENOMINATOR alone: the primes the conjugation loop
/// picks in order, the product `C` of the conjugates it multiplies in, and the rational `N = denominator * C`
/// the loop ends with. The quotient of any numerator is then `numerator * C / N`; the memory and the budget are not
/// touched (the oracle's loop does `squarefree_split(prime)` for each prime, a host that replays [`primes`] must do
/// the same). `None` where the oracle leaves the loop for its fallback: no prime from the universe.
///
/// [`primes`]: ConjugationPlan::primes
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConjugationPlan {
    pub primes: Vec<UBig>,
    pub conjugate: SqrtSum,
    pub rational: Rat,
}

pub fn conjugation_plan(denominator: &SqrtSum, universe: &[UBig], products: &mut ProductMemo) -> Option<ConjugationPlan> {
    if denominator.is_zero() {
        return None;
    }
    let form = denominator.int_form();
    let (mut common, mut items) = (form.common.clone(), form.items.clone());
    let mut conjugate_common = UBig::ONE;
    let mut conjugate: Items = vec![(UBig::ONE, IBig::ONE)];
    let mut primes = Vec::new();
    while !(items.len() <= 1 && items.iter().all(|(radicand, _)| radicand.is_one())) {
        if primes.len() > universe.len() {
            return None;
        }
        let prime = pick_prime_from_universe(items.iter().map(|(radicand, value)| (radicand, !value.is_zero())), universe)?;
        let flipped = conjugate_items(&items, &prime);
        conjugate = multiply_integer_items(&conjugate, &flipped, products);
        conjugate_common = &conjugate_common * &common;
        items = multiply_integer_items(&items, &flipped, products);
        common = &common * &common;
        reduce_in_place(&mut common, &mut items);
        primes.push(prime);
    }
    let head = items.first().map_or(IBig::ZERO, |(_, value)| value.clone());
    if head.is_zero() {
        return None;
    }
    Some(ConjugationPlan { primes, conjugate: IntForm { common: conjugate_common, items: conjugate }.into_sqrt_sum(), rational: Rat::reduced(head, common) })
}

// --------------------------------------------------------------------------
// radicals
// --------------------------------------------------------------------------

/// The integer radicand of `radicand` and the divisor that moves into the coefficient: `sqrt(p/r) = sqrt(p*r)/r`.
fn integral_radicand(radicand: &Rat) -> (IBig, Option<&UBig>) {
    if radicand.denominator().is_one() {
        (radicand.numerator().clone(), None)
    } else {
        (radicand.numerator() * IBig::from(radicand.denominator().clone()), Some(radicand.denominator()))
    }
}

/// `SqrtSumV1.radical(coefficient, radicand, budget)`: `coefficient * sqrt(radicand)` in canonical form. A zero
/// coefficient or radicand is the zero sum before any memory is touched; a negative radicand is
/// `NegativeRadicandError` (naming the integer radicand after the rational reduction).
pub fn radical(ctx: &mut ExactCtx<'_>, coefficient: &Rat, radicand: &Rat) -> Result<SqrtSum, ExactError> {
    if coefficient.is_zero() || radicand.is_zero() {
        return Ok(SqrtSum::zero());
    }
    let (integral, divisor) = integral_radicand(radicand);
    let coefficient = match divisor {
        Some(divisor) => coefficient.div(&Rat::from_int(IBig::from(divisor.clone()))).map_err(|_| ExactError::Internal("a zero denominator"))?,
        None => coefficient.clone(),
    };
    let (outside, inside) = ctx.memory.squarefree_split(&integral, ctx.budget)?;
    let term = Term { radicand: inside, coef: Coef::fraction(coefficient.mul(&Rat::from_int(IBig::from(outside)))) };
    Ok(SqrtSum::from_terms_unchecked(vec![term]))
}

struct Accumulated {
    radicand: UBig,
    numerator: IBig,
    denominator: UBig,
}

/// `radical_sum(parts, budget)`: `sum coefficient_i * sqrt(radicand_i)`; `squarefree_split` runs on the same
/// non-zero radicands in the same order as the chain `radical(...) + ...`.
pub fn radical_sum(ctx: &mut ExactCtx<'_>, parts: &[(Rat, Rat)]) -> Result<SqrtSum, ExactError> {
    let mut merged: Vec<Accumulated> = Vec::new();
    for (coefficient, radicand) in parts {
        if coefficient.is_zero() || radicand.is_zero() {
            continue;
        }
        let (mut numerator, mut denominator) = (coefficient.numerator().clone(), coefficient.denominator().clone());
        let (integral, divisor) = integral_radicand(radicand);
        if let Some(divisor) = divisor {
            denominator *= divisor;
        }
        let (outside, inside) = ctx.memory.squarefree_split(&integral, ctx.budget)?;
        numerator *= IBig::from(outside);
        match merged.binary_search_by(|entry| entry.radicand.cmp(&inside)) {
            Ok(position) => {
                let old = &mut merged[position];
                old.numerator = &numerator * IBig::from(old.denominator.clone()) + &old.numerator * IBig::from(denominator.clone());
                old.denominator = &old.denominator * &denominator;
            }
            Err(position) => merged.insert(position, Accumulated { radicand: inside, numerator, denominator }),
        }
    }
    let terms = merged
        .into_iter()
        .filter(|entry| !entry.numerator.is_zero())
        .map(|entry| Term { radicand: entry.radicand, coef: Coef::fraction(Rat::reduced(entry.numerator, entry.denominator)) })
        .collect();
    Ok(SqrtSum::from_terms_unchecked(terms))
}

// --------------------------------------------------------------------------
// prime universe
// --------------------------------------------------------------------------

/// How `prime_universe_remembered` meets its `store` (the host owns the dictionary).
#[derive(Debug, Clone, Copy)]
pub enum UniverseStore<'a> {
    /// `store is None`: build, remember nothing.
    Absent,
    /// The key is not in the store: build and hand back the record to write.
    Miss,
    /// The key is in the store: put the recorded factorizations back, spend nothing.
    Hit(&'a UniverseRecord),
}

/// `prime_universe_remembered(q_values, budget, store)`: the universe and, on a miss, the `(universe, delta)`
/// record the host writes into its store (only on success, as Python does).
pub fn prime_universe(ctx: &mut ExactCtx<'_>, q_values: &[QValue], store: UniverseStore<'_>) -> Result<(Vec<UBig>, Option<UniverseRecord>), ExactError> {
    match store {
        UniverseStore::Absent => Ok((ctx.memory.prime_universe_from_q_values(q_values, ctx.budget)?, None)),
        UniverseStore::Miss => {
            let record = ctx.memory.prime_universe_miss(q_values, ctx.budget)?;
            Ok((record.universe.clone(), Some(record)))
        }
        UniverseStore::Hit(record) => Ok((ctx.memory.prime_universe_hit(record), None)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sqrt_sum::integer_certified_sign;
    use cftuv_canon::Operation;

    struct World {
        memory: CanonMemory,
        budget: WorkBudget,
        counts: SignCounts,
        products: ProductMemo,
    }

    impl World {
        fn new(budget: WorkBudget) -> World {
            World { memory: CanonMemory::new(), budget, counts: SignCounts::default(), products: ProductMemo::new() }
        }

        fn ctx(&mut self) -> ExactCtx<'_> {
            ExactCtx { memory: &mut self.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.products }
        }
    }

    fn term(radicand: u64, numerator: i64, denominator: i64) -> Term {
        Term { radicand: UBig::from(radicand), coef: Coef::fraction(Rat::new(IBig::from(numerator), IBig::from(denominator)).unwrap()) }
    }

    fn sum(terms: Vec<Term>) -> SqrtSum {
        SqrtSum::from_terms(terms).unwrap()
    }

    fn rat(value: i64) -> Rat {
        Rat::from_i64(value)
    }

    /// `sqrt(2) + sqrt(3) - r` with `r` the binary approximation to `2^-bits`: undecidable by a 64-bit enclosure.
    fn near_zero(bits: usize, shift: i64) -> SqrtSum {
        let scaled = |radicand: u64| num::isqrt(&(UBig::from(radicand) << (2 * bits)));
        let total = IBig::from(scaled(2)) + IBig::from(scaled(3)) + IBig::from(shift);
        let approximation = Rat::new(total, IBig::ONE << bits).unwrap();
        sum(vec![Term { radicand: UBig::ONE, coef: Coef::fraction(approximation.neg()) }, term(2, 1, 1), term(3, 1, 1)])
    }

    #[test]
    fn a_tiny_difference_goes_through_the_conjugation_and_counts_it_before_the_work() {
        let value = near_zero(90, 1);
        let mut world = World::new(WorkBudget::unlimited());
        let answer = sign(&mut world.ctx(), &value, 64).unwrap();
        // the value is (frac - shift) / 2^90 with 0 < frac < 2: the 64-bit enclosure cannot tell, the exact path can
        assert_eq!(Some(answer), integer_certified_sign(&value.int_form().items, 400));
        assert_eq!(world.counts.as_array(), [1, 0, 0, 0, 1]);
        assert!(world.memory.lengths().3 >= 2, "the supports of 2 and 3 were materialized");
        // shift 3 puts the approximation above the value (frac - 3 < 0); shift -1 puts it below (frac + 1 > 0)
        assert_eq!(sign(&mut world.ctx(), &near_zero(90, 3), 64).unwrap(), -1);
        assert_eq!(sign(&mut world.ctx(), &near_zero(90, -1), 64).unwrap(), 1);
    }

    #[test]
    fn an_exhaustion_keeps_the_counter_and_the_partial_effects() {
        let value = near_zero(90, 1);
        let mut world = World::new(WorkBudget::bounded(0));
        let error = sign(&mut world.ctx(), &value, 64).unwrap_err();
        match error {
            ExactError::Canon(CanonError::Exhausted(exhausted)) => {
                assert_eq!(exhausted.operation, Operation::PrimeSupport);
                assert_eq!(exhausted.radicand, UBig::from(2u8), "the first radicand above one is asked first");
            }
            other => panic!("expected an exhaustion, got {other:?}"),
        }
        assert_eq!(world.counts.closed_by_conjugation, 1, "counted before the exact work");
        assert_eq!(world.budget.articles(), [0, 0, 0, 0, 1, 0], "the failing spend stays incremented");
        assert_eq!(world.memory.lengths(), (0, 0, 0, 0), "nothing was stored by the refused miss");
    }

    #[test]
    fn division_gives_a_quotient_that_multiplies_back() {
        let mut world = World::new(WorkBudget::unlimited());
        let numerator = sum(vec![term(1, 3, 2), term(2, -1, 1), term(6, 5, 3)]);
        let denominator = sum(vec![term(1, 1, 1), term(2, 1, 1), term(3, -2, 1), term(6, 1, 5)]);
        let quotient = divided_by(&mut world.ctx(), &numerator, &denominator).unwrap();
        let product = quotient.mul(&denominator, &mut ProductMemo::new());
        assert_eq!(product, numerator);
        // the same quotient through the generic fallback and through a complete prime universe
        assert_eq!(divided_by_generic(&mut world.ctx(), &numerator, &denominator).unwrap(), quotient);
        let universe = [UBig::from(2u8), UBig::from(3u8)];
        assert_eq!(divide_with_prime_universe(&mut world.ctx(), &numerator, &denominator, &universe).unwrap(), quotient);
        // an incomplete universe falls back to the full division on the original operands: same answer
        assert_eq!(divide_with_prime_universe(&mut world.ctx(), &numerator, &denominator, &universe[..1]).unwrap(), quotient);
    }

    #[test]
    fn dividing_by_zero_is_named_before_any_memory_is_touched() {
        let mut world = World::new(WorkBudget::bounded(5));
        let value = sum(vec![term(2, 1, 1)]);
        assert_eq!(divided_by(&mut world.ctx(), &value, &SqrtSum::zero()), Err(ExactError::ZeroDivisor));
        assert_eq!(divide_with_prime_universe(&mut world.ctx(), &value, &SqrtSum::zero(), &[]), Err(ExactError::ZeroDivisor));
        assert_eq!(world.budget.articles(), [0; 6]);
        assert_eq!(world.memory.lengths(), (0, 0, 0, 0));
        // a rational divisor needs no conjugation at all
        let rational = SqrtSum::rational(&rat(-4));
        let quotient = divided_by(&mut world.ctx(), &value, &rational).unwrap();
        assert_eq!(quotient, sum(vec![term(2, -1, 4)]));
        assert_eq!(world.memory.lengths(), (0, 0, 0, 0));
    }

    #[test]
    fn radicals_reduce_rational_radicands_and_name_the_negative_integer() {
        let mut world = World::new(WorkBudget::unlimited());
        // 5 * sqrt(3/4) = (5/4) * sqrt(12), 12 = 2^2 * 3: coefficient 5/4 * 2, radicand 3
        let value = radical(&mut world.ctx(), &rat(5), &Rat::new(IBig::from(3), IBig::from(4)).unwrap()).unwrap();
        assert_eq!(value, sum(vec![term(3, 5, 2)]));
        assert_eq!(radical(&mut world.ctx(), &rat(0), &rat(7)).unwrap(), SqrtSum::zero());
        assert_eq!(radical(&mut world.ctx(), &rat(2), &rat(0)).unwrap(), SqrtSum::zero());
        let negative = radical(&mut world.ctx(), &rat(1), &Rat::new(IBig::from(-3), IBig::from(5)).unwrap()).unwrap_err();
        assert_eq!(negative, ExactError::Canon(CanonError::NegativeRadicand { numerator: IBig::from(-15), denominator: UBig::ONE }));
    }

    #[test]
    fn radical_sum_merges_like_the_chain_of_radicals() {
        let mut world = World::new(WorkBudget::unlimited());
        let parts = [(rat(1), rat(2)), (rat(1), rat(8)), (rat(-3), rat(18)), (Rat::new(IBig::ONE, IBig::from(2)).unwrap(), rat(2)), (rat(0), rat(5))];
        let merged = radical_sum(&mut world.ctx(), &parts).unwrap();
        // 1 + 2 - 9 + 1/2 = -11/2 times sqrt(2)
        assert_eq!(merged, sum(vec![term(2, -11, 2)]));
        let mut chain = SqrtSum::zero();
        let mut other = World::new(WorkBudget::unlimited());
        for (coefficient, radicand) in &parts {
            let part = radical(&mut other.ctx(), coefficient, radicand).unwrap();
            chain = chain.add(&part);
        }
        assert_eq!(chain, merged);
        assert_eq!(world.budget.articles(), other.budget.articles(), "the same misses, the same price");
        assert_eq!(world.memory.export_state(), other.memory.export_state());
        let cancelled = radical_sum(&mut world.ctx(), &[(rat(2), rat(50)), (rat(-10), rat(2))]).unwrap();
        assert!(cancelled.is_zero());
    }

    #[test]
    fn the_prime_universe_store_modes() {
        let mut world = World::new(WorkBudget::unlimited());
        let q = |n: i64, d: u64| QValue { numerator: IBig::from(n), denominator: UBig::from(d) };
        let values = [q(6, 5), q(15, 1), q(0, 1)];
        let (universe, record) = prime_universe(&mut world.ctx(), &values, UniverseStore::Miss).unwrap();
        let record = record.expect("a miss hands the record back");
        assert_eq!(record.universe, universe);
        assert!(!universe.is_empty());
        assert!(world.budget.spent() > 0);
        // a hit on a cold memory puts the recorded factorizations back and pays nothing
        let mut cold = World::new(WorkBudget::bounded(0));
        let (again, none) = prime_universe(&mut cold.ctx(), &values, UniverseStore::Hit(&record)).unwrap();
        assert_eq!(again, universe);
        assert!(none.is_none());
        assert_eq!(cold.budget.articles(), [0; 6]);
        let mut plain_world = World::new(WorkBudget::unlimited());
        let (plain, no_record) = prime_universe(&mut plain_world.ctx(), &values, UniverseStore::Absent).unwrap();
        assert_eq!(plain, universe);
        assert!(no_record.is_none());
        let refused = prime_universe(&mut world.ctx(), &[q(-3, 7)], UniverseStore::Miss).unwrap_err();
        assert_eq!(refused, ExactError::Canon(CanonError::NegativeRadicand { numerator: IBig::from(-3), denominator: UBig::from(7u8) }));
    }
}
