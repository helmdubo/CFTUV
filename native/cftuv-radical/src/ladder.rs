//! The integer form, the outward enclosure, the separation bound and the sign ladder of `radical_v2.py` (`_integer_form`, `_integer_enclosure`, `separation_bits`, `_signum_terms`).
//!
//! SIGN. No terms: 0. One term: its sign. Two terms of opposite signs: compare the squares `t^2` in Q (never equal: different classes). Three or more: the integer enclosure on the
//! rungs 64, then min(128, B), min(256, B), ... bits, where `B` is the COMPUTED separation bound, evaluated only when the 64-bit rung fails. At the bound the sign is certain, so
//! `SignUndecided` is only the outcome of an explicit price cap, never of the mathematics. The bound is exact (a [`UBig`]): it is exponential in the number of classes.

use dashu_int::ops::{BitTest, SquareRoot};

use crate::arith::{lcm, magnitude, reduced, signed, IBig, UBig};
use crate::canon::Term;
use crate::work::{Failure, Refusal, RefusalCode, Work, SIGN_FIRST_BITS};

/// `_integer_form(terms)`: `L` and `[(m', a)]` with `t = a sqrt(m') / L` (irrational) or `t = a / L` (rational), `L` the least common denominator.
pub struct IntegerForm {
    pub common: UBig,
    pub items: Vec<(UBig, IBig)>,
}

pub fn integer_form(terms: &[Term]) -> IntegerForm {
    let mut common = UBig::ONE;
    for term in terms {
        if !term.denominator.is_one() {
            common = lcm(&common, &term.denominator);
        }
    }
    let items = if common.is_one() {
        terms.iter().map(|term| (term.radicand.clone(), term.numerator.clone())).collect()
    } else {
        terms.iter().map(|term| (term.radicand.clone(), &term.numerator * signed(&(&common / &term.denominator)))).collect()
    };
    IntegerForm { common, items }
}

/// `_integer_enclosure(items, bits)`: integer bounds of `2^bits * sum a sqrt(m')`, outward: `isqrt` is the lower node of each root, the next node the upper.
///
/// For a term `a sqrt(m')` with `f = isqrt(m' 4^bits)` the bounds are `a f` and `a (f + 1)` (the smaller first when `a < 0`), so `low = sum a f + sum_{a<0} a` and
/// `high = sum a f + sum_{a>0} a`: the same integers, one product per term.
pub fn integer_enclosure(items: &[(UBig, IBig)], bits: usize) -> (IBig, IBig) {
    let shift = 2 * bits;
    let mut centre = IBig::ZERO;
    let mut below = IBig::ZERO;
    let mut above = IBig::ZERO;
    for (radicand, numerator) in items {
        if radicand.is_one() {
            centre += numerator << bits;
            continue;
        }
        let floor_root = IBig::from((radicand << shift).sqrt());
        centre += numerator * floor_root;
        if *numerator > IBig::ZERO {
            above += numerator;
        } else {
            below += numerator;
        }
    }
    (&centre + below, centre + above)
}

/// `separation_bits(items)`: bits at which the integer enclosure of `sum a sqrt(m')` (>= 1 term, all `a != 0`) cannot hold zero.
///
/// `b = bitlen(W) + (2^(k-1) - 1) * bitlen(U)` with `W = sum_{irrational} |a|` and `U = sum |a| (isqrt(m') + 1)` (rational terms: `|a|`); see the proof in `radical_v2.py`.
pub fn separation_bits(items: &[(UBig, IBig)]) -> UBig {
    let mut upper = UBig::ZERO;
    let mut width = UBig::ZERO;
    for (radicand, numerator) in items {
        let size = magnitude(numerator);
        if radicand.is_one() {
            upper += size;
        } else {
            upper += &size * (radicand.sqrt() + UBig::ONE);
            width += size;
        }
    }
    let exponent = (UBig::ONE << (items.len() - 1)) - UBig::ONE;
    UBig::from(width.bit_len()) + exponent * UBig::from(upper.bit_len())
}

/// `t^2` of a term as `(numerator, denominator)`: `n^2/d^2` for the rational term, `m'/Q^2` for an irrational one.
fn square_of(term: &Term) -> (UBig, UBig) {
    let numerator = if term.is_rational() { magnitude(&term.numerator) * magnitude(&term.numerator) } else { term.radicand.clone() };
    (numerator, &term.denominator * &term.denominator)
}

fn sign_of(term: &Term) -> i8 {
    if term.numerator > IBig::ZERO {
        1
    } else {
        -1
    }
}

fn pretty(value: &UBig) -> String {
    value.to_string()
}

/// `_signum_terms(terms, cap)`: `(sign, enclosure rounds)`. `cap` is the only way to get `SignUndecided`.
pub(crate) fn signum_terms(terms: &[Term], cap: Option<i64>) -> Result<(i8, u64), Failure> {
    let count = terms.len();
    match count {
        0 => return Ok((0, 0)),
        1 => return Ok((sign_of(&terms[0]), 0)),
        2 => {
            let (first, second) = (sign_of(&terms[0]), sign_of(&terms[1]));
            if first == second {
                return Ok((first, 0));
            }
            let ((p1, q1), (p2, q2)) = (square_of(&terms[0]), square_of(&terms[1]));
            // equal squares would mean one class; the classes are different
            return Ok((if &p1 * &q2 > &p2 * &q1 { first } else { second }, 0));
        }
        _ => {}
    }
    let form = integer_form(terms);
    // Rungs: 64, then min(128, bound), min(256, bound), ...; the bound is computed only when the first rung fails (a rung at or above the bound always decides, so a bound
    // below 64 bits can never be the reason the first rung failed).
    let mut required: Option<UBig> = None;
    let mut rung: u64 = SIGN_FIRST_BITS;
    let mut rounds: u64 = 0;
    loop {
        if let Some(cap) = cap {
            if i128::from(rung) > i128::from(cap) {
                let needs = match &required {
                    Some(bits) => pretty(bits),
                    None => format!("more than {cap}"),
                };
                return Err(Failure::Refused(Refusal::new(
                    RefusalCode::SignUndecided,
                    format!("{count} classes: the separation bound needs {needs} bits, the cap is {cap}"),
                    Work::rounds(rounds),
                )));
            }
        }
        rounds += 1;
        let (low, high) = integer_enclosure(&form.items, rung as usize);
        if low > IBig::ZERO {
            return Ok((1, rounds));
        }
        if high < IBig::ZERO {
            return Ok((-1, rounds));
        }
        let bound = required.get_or_insert_with(|| separation_bits(&form.items));
        if UBig::from(rung) >= *bound {
            return Err(Failure::SeparationBoundViolated(format!("{count} classes, enclosure [{low}, {high}] at {rung} bits, bound {bound}")));
        }
        let doubled = UBig::from(rung) * UBig::from(2u8);
        rung = if doubled <= *bound { rung * 2 } else { u64::try_from(&*bound).expect("a bound below the doubled rung fits the rung") };
    }
}

/// `enclosure(bits)`: outward `[low, high]` as reduced fractions `(numerator, denominator)`: each term `s sqrt(P/Q)` within `[isqrt(PQ 4^bits), +1] / (Q 2^bits)`, summed over the common denominator.
pub fn enclosure(terms: &[Term], bits: usize) -> ((IBig, UBig), (IBig, UBig)) {
    let form = integer_form(terms);
    let (low, high) = integer_enclosure(&form.items, bits);
    let denominator = &form.common << bits;
    (reduced(low, denominator.clone()), reduced(high, denominator))
}
