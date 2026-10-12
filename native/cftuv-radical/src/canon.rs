//! The value-canonical terms of `RadicalSumV2` and the algebra on term lists (`radical_v2.py`, "terms" section), operation for operation.
//!
//! A term `t = c sqrt(r)` with `t^2 = P/Q` in lowest terms is the triple `(m', s, Q)`, `m' = P Q`, `s = sign(t)`: `t = s sqrt(m') / Q`. The rational class is
//! `(1, n, d)`. A value is the list of its terms sorted by `m'` (two classes never share an `m'`: `m'^2` is a square), so every representation is a function of the
//! value. The counting rules of the price (`CLASS_MERGE_CHECKS`, `CONJUGATION_PRODUCTS`) are the contract of the mirror: where Python asks a class question, so does this
//! file, in the same order, and no more.

use crate::arith::{exact_root, gcd, magnitude, reduced, signed, IBig, UBig};
use crate::work::{Failure, Refusal, RefusalCode, Work, INVERSE_CLASS_LIMIT};

/// `(m', n, d)`: `n/d * sqrt(m')`; for an irrational class `n = +-1` and `d = Q`, for the rational class `m' = 1`.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Term {
    pub radicand: UBig,
    pub numerator: IBig,
    pub denominator: UBig,
}

impl Term {
    #[inline]
    pub fn rational(numerator: IBig, denominator: UBig) -> Term {
        Term { radicand: UBig::ONE, numerator, denominator }
    }

    #[inline]
    pub fn is_rational(&self) -> bool {
        self.radicand.is_one()
    }

    #[inline]
    pub fn is_negative(&self) -> bool {
        self.numerator < IBig::ZERO
    }

    #[inline]
    fn negated(&self) -> Term {
        Term { radicand: self.radicand.clone(), numerator: -&self.numerator, denominator: self.denominator.clone() }
    }
}

/// `_irrational_term(radicand, numerator, denominator)`: the value-canonical triple of `numerator/denominator * sqrt(radicand)` (`radicand` not a square, `denominator > 0`); `None` for zero.
///
/// With `numerator/denominator = n/d` in lowest terms, `t^2 = n^2 r / d^2` and `gcd(n^2 r, d^2) = gcd(r, d^2) = h`, so `P = n^2 (r/h)`, `Q = d^2/h` are coprime: `m' = P Q`, `s = sign(n)`.
pub(crate) fn irrational_term(radicand: &UBig, numerator: IBig, denominator: UBig) -> Option<Term> {
    if numerator.is_zero() {
        return None;
    }
    let negative = numerator < IBig::ZERO;
    let mut n = magnitude(&numerator);
    let mut d = denominator;
    let sign = if negative { IBig::NEG_ONE } else { IBig::ONE };
    if d.is_one() {
        // an integer coefficient: t^2 = n^2 r, nothing to cancel (Q = 1, P = n^2 r)
        return Some(Term { radicand: &n * &n * radicand, numerator: sign, denominator: UBig::ONE });
    }
    if !n.is_one() {
        let common = gcd(&n, &d);
        if !common.is_one() {
            n /= &common;
            d /= common;
        }
    }
    let square = &d * &d;
    let shared = gcd(radicand, &square);
    let (quotient, kept) = if shared.is_one() { (square, radicand.clone()) } else { (&square / &shared, radicand / &shared) };
    let m = if n.is_one() { &kept * &quotient } else { ((&n * &n) * kept) * &quotient };
    Some(Term { radicand: m, numerator: sign, denominator: quotient })
}

/// `_class_root(keep, new)`: `isqrt(keep * new)` when the product is a perfect square (one class).
#[inline]
fn class_root(keep: &UBig, new: &UBig) -> Option<UBig> {
    exact_root(&(keep * new))
}

fn sort_terms(terms: &mut [Term]) {
    terms.sort_by(|left, right| left.radicand.cmp(&right.radicand));
}

// --------------------------------------------------------------------------
// class slots (from_terms, mul)
// --------------------------------------------------------------------------

/// `[r, n, d]` of the oracle's slots: the class so far as `n/d * sqrt(r)` (not necessarily reduced).
pub(crate) struct Slot {
    radicand: UBig,
    numerator: IBig,
    denominator: UBig,
}

/// `_merge_into(slot, radicand, numerator, denominator, root)`: adds `numerator/denominator * sqrt(radicand)` to the class, `root = isqrt(r * radicand)`, on the smaller radicand.
fn merge_into(slot: &mut Slot, radicand: &UBig, numerator: IBig, denominator: UBig, root: &UBig) {
    let (n, d, numerator, denominator) = if *radicand < slot.radicand {
        // sqrt(keep) = root / radicand * sqrt(radicand)
        let n = std::mem::replace(&mut slot.numerator, IBig::ZERO) * signed(root);
        let d = std::mem::replace(&mut slot.denominator, UBig::ONE) * radicand;
        slot.radicand = radicand.clone();
        (n, d, numerator, denominator)
    } else {
        // sqrt(radicand) = root / keep * sqrt(keep)
        let n = std::mem::replace(&mut slot.numerator, IBig::ZERO);
        let d = std::mem::replace(&mut slot.denominator, UBig::ONE);
        (n, d, numerator * signed(root), denominator * &slot.radicand)
    };
    let top = n * signed(&denominator) + numerator * signed(&d);
    let (top, bottom) = reduced(top, d * denominator);
    slot.numerator = top;
    slot.denominator = bottom;
}

/// `_accumulate(slots, index, radicand, numerator, denominator)`: adds an irrational `numerator/denominator * sqrt(radicand)` to the class slots (creation order); returns the class checks made.
///
/// An equal radicand merges without a check; otherwise every slot, in creation order, is asked once until one is the same class.
pub(crate) fn accumulate(slots: &mut Vec<Slot>, radicand: UBig, numerator: IBig, denominator: UBig) -> u64 {
    if let Some(slot) = slots.iter_mut().find(|slot| slot.radicand == radicand) {
        let top = std::mem::replace(&mut slot.numerator, IBig::ZERO) * signed(&denominator) + numerator * signed(&slot.denominator);
        let (top, bottom) = reduced(top, std::mem::replace(&mut slot.denominator, UBig::ONE) * denominator);
        slot.numerator = top;
        slot.denominator = bottom;
        return 0;
    }
    let mut checks = 0;
    for slot in slots.iter_mut() {
        checks += 1;
        if let Some(root) = class_root(&slot.radicand, &radicand) {
            merge_into(slot, &radicand, numerator, denominator, &root);
            return checks;
        }
    }
    slots.push(Slot { radicand, numerator, denominator });
    checks
}

/// `_freeze(rational_numerator, rational_denominator, slots)`: the canonical terms of a rational part and class slots: each class normalised to its triple, zero classes dropped, `m'` order.
pub(crate) fn freeze(rational: (IBig, UBig), slots: Vec<Slot>) -> Vec<Term> {
    let mut terms: Vec<Term> = Vec::with_capacity(slots.len() + 1);
    for slot in slots {
        if let Some(term) = irrational_term(&slot.radicand, slot.numerator, slot.denominator) {
            terms.push(term);
        }
    }
    sort_terms(&mut terms);
    if !rational.0.is_zero() {
        // the rational part is the result of `reduced` in every loop that builds it (and `(0, 1)` before the first)
        debug_assert!(gcd(&magnitude(&rational.0), &rational.1).is_one());
        terms.insert(0, Term::rational(rational.0, rational.1));
    }
    terms
}

// --------------------------------------------------------------------------
// add, sub
// --------------------------------------------------------------------------

/// `_combine(left, right, root)`: `left + right` of two irrational terms of ONE class: equal `m'`, or `root = isqrt(a b)` and the sum on the smaller radicand.
fn combine(left: &Term, right: &Term, root: Option<&UBig>) -> Option<Term> {
    let (a, b) = (&left.radicand, &right.radicand);
    if a == b {
        return irrational_term(a, &left.numerator * signed(&right.denominator) + &right.numerator * signed(&left.denominator), &left.denominator * &right.denominator);
    }
    let root = root.expect("two different radicands of one class come with their root");
    if a < b {
        // sqrt(b) = root / a * sqrt(a)
        let top = &left.numerator * signed(&right.denominator) * signed(a) + &right.numerator * signed(root) * signed(&left.denominator);
        irrational_term(a, top, &left.denominator * &right.denominator * a)
    } else {
        let top = &right.numerator * signed(&left.denominator) * signed(b) + &left.numerator * signed(root) * signed(&right.denominator);
        irrational_term(b, top, &left.denominator * &right.denominator * b)
    }
}

fn ordered_pair(left: &Term, right: &Term) -> Vec<Term> {
    if left.radicand < right.radicand {
        vec![left.clone(), right.clone()]
    } else {
        vec![right.clone(), left.clone()]
    }
}

/// `_add_one(left, right)`: `left + right` of two single canonical terms (the common case): the same `m'`, two classes, or one class by one check.
fn add_one(left: &Term, right: &Term) -> (Vec<Term>, u64) {
    let (a, b) = (&left.radicand, &right.radicand);
    if a.is_one() && b.is_one() {
        let (numerator, denominator) = reduced(&left.numerator * signed(&right.denominator) + &right.numerator * signed(&left.denominator), &left.denominator * &right.denominator);
        return (if numerator.is_zero() { Vec::new() } else { vec![Term::rational(numerator, denominator)] }, 0);
    }
    if a == b {
        return (combine(left, right, None).into_iter().collect(), 0);
    }
    if a.is_one() || b.is_one() {
        return (ordered_pair(left, right), 0);
    }
    match class_root(a, b) {
        None => (ordered_pair(left, right), 1),
        Some(root) => (combine(left, right, Some(&root)).into_iter().collect(), 1),
    }
}

/// `_add_terms(left, right, sign)`: `left + sign * right` of two canonical term lists, and the class checks.
///
/// The terms of each operand are pairwise different classes, so a term of `right` meets at most one term of `left` (an equal `m'` first, without a check; else the open terms of
/// `left` in `m'` order, one check each), and a term of `left` that met one is never asked again; a term that met none is kept as it is (already canonical).
pub(crate) fn add_terms(left: &[Term], right: &[Term], subtract: bool) -> (Vec<Term>, u64) {
    if right.is_empty() {
        return (left.to_vec(), 0);
    }
    let negated: Vec<Term>;
    let right: &[Term] = if subtract {
        negated = right.iter().map(Term::negated).collect();
        &negated
    } else {
        right
    };
    if left.is_empty() {
        return (right.to_vec(), 0);
    }
    if left.len() == 1 && right.len() == 1 {
        return add_one(&left[0], &right[0]);
    }
    let mut rational = (IBig::ZERO, UBig::ONE);
    let mut left = left;
    if left[0].is_rational() {
        rational = (left[0].numerator.clone(), left[0].denominator.clone());
        left = &left[1..];
    }
    let mut open: Vec<Option<&Term>> = left.iter().map(Some).collect();
    let mut out: Vec<Term> = Vec::with_capacity(left.len() + right.len());
    let mut checks = 0u64;
    for term in right {
        if term.is_rational() {
            rational = reduced(
                std::mem::replace(&mut rational.0, IBig::ZERO) * signed(&term.denominator) + &term.numerator * signed(&rational.1),
                std::mem::replace(&mut rational.1, UBig::ONE) * &term.denominator,
            );
            continue;
        }
        // an equal m' first, without a check
        let mut found: Option<(usize, Option<UBig>)> = open.iter().position(|candidate| candidate.is_some_and(|candidate| candidate.radicand == term.radicand)).map(|at| (at, None));
        if found.is_none() {
            for (at, candidate) in open.iter().enumerate() {
                let Some(candidate) = candidate else { continue };
                checks += 1;
                if let Some(root) = class_root(&candidate.radicand, &term.radicand) {
                    found = Some((at, Some(root)));
                    break;
                }
            }
        }
        let Some((at, root)) = found else {
            out.push(term.clone());
            continue;
        };
        let merged = combine(open[at].expect("an open term"), term, root.as_ref());
        open[at] = None;
        if let Some(merged) = merged {
            out.push(merged);
        }
    }
    out.extend(open.into_iter().flatten().cloned());
    sort_terms(&mut out);
    if !rational.0.is_zero() {
        out.insert(0, Term::rational(rational.0, rational.1));
    }
    (out, checks)
}

// --------------------------------------------------------------------------
// scaled, mul
// --------------------------------------------------------------------------

/// `_scaled_terms(terms, numerator, denominator)`: `terms * numerator/denominator` (nonzero, `denominator > 0`): every class stays a class, its `m'` moves, the order is re-sorted.
pub(crate) fn scaled_terms(terms: &[Term], numerator: &IBig, denominator: &UBig) -> Vec<Term> {
    let mut out: Vec<Term> = Vec::with_capacity(terms.len());
    for term in terms {
        if term.is_rational() {
            let (n, d) = reduced(&term.numerator * numerator, &term.denominator * denominator);
            out.push(Term::rational(n, d));
        } else {
            out.push(irrational_term(&term.radicand, &term.numerator * numerator, &term.denominator * denominator).expect("a nonzero factor of a nonzero term"));
        }
    }
    sort_terms(&mut out);
    out
}

#[inline]
fn is_rational_terms(terms: &[Term]) -> bool {
    terms.len() == 1 && terms[0].is_rational()
}

/// `_mul_terms(left, right)`: `left * right`, and the class checks: products left-outer, right-inner, each into the class slots.
///
/// A rational operand only scales the other (no class can merge). `sqrt(a) sqrt(b) = gcd * sqrt((a/g)(b/g))`; whether that radicand is a square is one class check; equal radicands give the rational `a` without one.
pub(crate) fn mul_terms(left: &[Term], right: &[Term]) -> (Vec<Term>, u64) {
    if left.is_empty() || right.is_empty() {
        return (Vec::new(), 0);
    }
    if is_rational_terms(left) {
        return (scaled_terms(right, &left[0].numerator, &left[0].denominator), 0);
    }
    if is_rational_terms(right) {
        return (scaled_terms(left, &right[0].numerator, &right[0].denominator), 0);
    }
    let mut rational = (IBig::ZERO, UBig::ONE);
    let mut slots: Vec<Slot> = Vec::new();
    let mut checks = 0u64;
    for a in left {
        for b in right {
            let mut numerator = &a.numerator * &b.numerator;
            let denominator = &a.denominator * &b.denominator;
            let radicand;
            if a.is_rational() {
                radicand = b.radicand.clone();
            } else if b.is_rational() {
                radicand = a.radicand.clone();
            } else if a.radicand == b.radicand {
                radicand = UBig::ONE;
                numerator *= signed(&a.radicand);
            } else {
                let common = gcd(&a.radicand, &b.radicand);
                let product = (&a.radicand / &common) * (&b.radicand / &common);
                numerator *= signed(&common);
                checks += 1;
                match exact_root(&product) {
                    Some(root) => {
                        numerator *= signed(&root);
                        radicand = UBig::ONE;
                    }
                    None => radicand = product,
                }
            }
            if radicand.is_one() {
                rational = reduced(
                    std::mem::replace(&mut rational.0, IBig::ZERO) * signed(&denominator) + numerator * signed(&rational.1),
                    std::mem::replace(&mut rational.1, UBig::ONE) * denominator,
                );
            } else {
                checks += accumulate(&mut slots, radicand, numerator, denominator);
            }
        }
    }
    (freeze(rational, slots), checks)
}

// --------------------------------------------------------------------------
// inverse
// --------------------------------------------------------------------------

/// `_inverse_terms(terms)`: one term directly; two or three classes by conjugating the term of the LARGEST `m'`, recursively; more: `InverseClassLimit`; the empty sum: `ZeroDivision`.
pub(crate) fn inverse_terms(terms: &[Term]) -> Result<(Vec<Term>, Work), Failure> {
    let count = terms.len();
    if count == 0 {
        return Err(Failure::ZeroDivision);
    }
    if count == 1 {
        let term = &terms[0];
        if term.is_rational() {
            let numerator = if term.numerator > IBig::ZERO { signed(&term.denominator) } else { -signed(&term.denominator) };
            return Ok((vec![Term::rational(numerator, magnitude(&term.numerator))], Work::NONE));
        }
        // 1/(s sqrt(m')/Q) = s sqrt(m')/(m'/Q): the same m'
        return Ok((vec![Term { radicand: term.radicand.clone(), numerator: term.numerator.clone(), denominator: &term.radicand / &term.denominator }], Work::NONE));
    }
    if count > INVERSE_CLASS_LIMIT {
        return Err(Failure::Refused(Refusal::new(RefusalCode::InverseClassLimit, format!("{count} classes, limit {INVERSE_CLASS_LIMIT}"), Work::NONE)));
    }
    let pivot = &terms[count - 1];
    let rest = &terms[..count - 1];
    let mut conjugate = rest.to_vec();
    conjugate.push(Term { radicand: pivot.radicand.clone(), numerator: -&pivot.numerator, denominator: pivot.denominator.clone() });
    let (square, first) = mul_terms(rest, rest);
    // pivot^2 = m'/Q^2 = P/Q (lowest terms)
    let minus_pivot_square = [Term::rational(-signed(&(&pivot.radicand / &pivot.denominator)), pivot.denominator.clone())];
    let (norm, second) = add_terms(&square, &minus_pivot_square, false);
    let (inverse_norm, inner) = inverse_terms(&norm)?;
    let (result, third) = mul_terms(&conjugate, &inverse_norm);
    Ok((result, Work { class_merge_checks: first + second + third, conjugation_products: 2, enclosure_rounds: 0 } + inner))
}
