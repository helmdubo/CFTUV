//! `boundary_native.contact_candidates_native` for a pair whose three points are rational: the contacts of one source segment with one barrier segment.
//!
//! Same function as the Python leaf, same order of operations (the representative of a square class, hence the terms of every alpha and station, depends on it): the four scalar products of
//! `metric.dot_covector_native` (`_product(lx, gx) + _product(ly, gy)`), the five candidate parameters `[0, 1, -s0/ds, (L - s0)/ds, -a0/da]` (the divisions only for a non-zero `ds` / `da`), the
//! value-distinct filter, the three admissibility filters, the point written as texts of the rational coordinates, and the sort by alpha with the SEQUENCE of questions of CPython 3.11's
//! `list.sort` (`_cpython311.sorted_as_cpython311`): up to 63 elements that is `count_run` and a binary insertion, and a comparison that cannot be answered (`NativeSignUndecided`) ends the sort,
//! so the same comparisons must be asked.
//!
//! Everything the oracle would do through sympy (an irrational parameter reaching `place`) or refuse (`Refused`) is DECLINED: the function returns the reason and touches nothing.

use cftuv_core::rat::Rat;

use crate::radical::{Radical, Refused};
use crate::text::rational_text;

/// One contact: `(alpha, station, point)` with the point as the two texts of `ExactScalar`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Contact {
    pub alpha: Radical,
    pub station: Radical,
    pub x: String,
    pub y: String,
}

/// Why the leaf does not answer (the oracle computes the pair).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decline {
    /// an operation of the algebra refused (`NativeExactError`)
    Refused(Refused),
    /// a contact whose parameter is not rational: the oracle writes its point through sympy (`point_add`, `vector_scale`, `to_sympy`)
    IrrationalParameter,
    /// a sort of 64 or more contacts (five at most exist; the port does not claim the merge strategy)
    SortTooLong,
}

impl Decline {
    pub fn reason(self) -> &'static str {
        match self {
            Decline::Refused(Refused::OutsideField) => "a value outside the native field (too many square classes to invert)",
            Decline::Refused(Refused::SignUndecided) => "a sign the enclosure did not decide",
            Decline::Refused(Refused::ZeroDivision) => "a division by an exact zero",
            Decline::IrrationalParameter => "a contact with an irrational parameter (its point is written through sympy)",
            Decline::SortTooLong => "more contacts than the port sorts",
        }
    }
}

impl From<Refused> for Decline {
    fn from(refused: Refused) -> Decline {
        Decline::Refused(refused)
    }
}

type Pair = (Radical, Radical);

/// `metric._product(left, right)`: a rational left factor rewrites the terms of `right` in place; anything else is the general product.
fn product(left: &Radical, right: &Radical) -> Radical {
    if let [(radicand, factor)] = left.terms() {
        if radicand.is_one() {
            return if right.terms().is_empty() {
                Radical::zero()
            } else {
                Radical::from_terms(right.terms().iter().map(|(r, value)| (r.clone(), value.mul(factor))).collect())
            };
        }
    }
    left.mul(right)
}

/// `metric.dot_covector_native(left, covector)`.
fn dot_covector(left: &Pair, covector: &Pair) -> Radical {
    product(&left.0, &covector.0).add(&product(&left.1, &covector.1))
}

fn terms_are_rational(value: &Radical) -> bool {
    let terms = value.terms();
    terms.is_empty() || (terms.len() == 1 && terms[0].0.is_one())
}

/// `_same_value(left, right)`: `not (left - right).terms`, with the cheap layers of the oracle.
fn same_value(left: &Radical, right: &Radical) -> bool {
    if left.terms() == right.terms() {
        return true;
    }
    if terms_are_rational(left) && terms_are_rational(right) {
        return false;
    }
    left.sub(right).terms().is_empty()
}

/// `_distinct(values)`: the first of each value, in order.
fn distinct(values: Vec<Radical>) -> Vec<Radical> {
    let mut unique: Vec<Radical> = Vec::new();
    for value in values {
        if !unique.iter().any(|kept| same_value(&value, kept)) {
            unique.push(value);
        }
    }
    unique
}

/// `less(a, b)` of the sort: `(a.alpha - b.alpha).signum() < 0`.
fn less(alphas: &[Radical], left: usize, right: usize) -> Result<bool, Refused> {
    Ok(alphas[left].sub(&alphas[right]).signum()? < 0)
}

/// `sorted_as_cpython311` for fewer than 64 items: `min_run` is the length, so the sort is one `count_run` (a strictly descending run is reversed) and a binary insertion of the rest.
fn sorted_order(alphas: &[Radical]) -> Result<Vec<usize>, Refused> {
    let size = alphas.len();
    let mut keys: Vec<usize> = (0..size).collect();
    if size < 2 {
        return Ok(keys);
    }
    // count_run(keys, 0, size)
    let mut at = 1usize;
    let mut run = 2usize;
    let mut descending = false;
    if less(alphas, keys[at], keys[at - 1])? {
        at += 1;
        while at < size && less(alphas, keys[at], keys[at - 1])? {
            at += 1;
            run += 1;
        }
        descending = true;
    } else {
        at += 1;
        while at < size && !less(alphas, keys[at], keys[at - 1])? {
            at += 1;
            run += 1;
        }
    }
    if descending {
        keys[..run].reverse();
    }
    // binary_insertion(keys, 0, size, run)
    let mut start = run;
    while start < size {
        let (mut left, mut right) = (0usize, start);
        let pivot = keys[start];
        while left < right {
            let middle = left + ((right - left) >> 1);
            if less(alphas, pivot, keys[middle])? {
                right = middle;
            } else {
                left = middle + 1;
            }
        }
        keys.copy_within(left..start, left + 1);
        keys[left] = pivot;
        start += 1;
    }
    Ok(keys)
}

/// The frame data the leaf reads: `length`, `G·t`, `G·n` and the source start.
pub struct LeafFrame<'a> {
    pub origin: &'a (Rat, Rat),
    pub length: &'a Radical,
    pub tangent: &'a Pair,
    pub normal: &'a Pair,
}

/// `contact_candidates_native` for the barrier segment `start -> end`.
pub fn contacts(frame: &LeafFrame<'_>, start: &(Rat, Rat), end: &(Rat, Rat)) -> Result<Vec<Contact>, Decline> {
    let step = (end.0.sub(&start.0), end.1.sub(&start.1));
    let direction: Pair = (Radical::rational(step.0.clone()), Radical::rational(step.1.clone()));
    let offset: Pair = (Radical::rational(start.0.sub(&frame.origin.0)), Radical::rational(start.1.sub(&frame.origin.1)));
    let s0 = dot_covector(&offset, frame.tangent);
    let ds = dot_covector(&direction, frame.tangent);
    let a0 = dot_covector(&offset, frame.normal);
    let da = dot_covector(&direction, frame.normal);
    let length = frame.length;
    let one = Radical::one();
    let mut candidates = vec![Radical::zero(), Radical::one()];
    if ds.signum()? != 0 {
        candidates.push(s0.neg().div(&ds)?);
        candidates.push(length.sub(&s0).div(&ds)?);
    }
    if da.signum()? != 0 {
        candidates.push(a0.neg().div(&da)?);
    }
    let mut found: Vec<Contact> = Vec::new();
    for parameter in distinct(candidates) {
        if parameter.signum()? < 0 || parameter.sub(&one).signum()? > 0 {
            continue;
        }
        let station = s0.add(&parameter.mul(&ds));
        let alpha = a0.add(&parameter.mul(&da));
        if station.signum()? < 0 || station.sub(length).signum()? > 0 {
            continue;
        }
        if alpha.signum()? < 0 {
            continue;
        }
        // `place(parameter)`: the point of a rational parameter is written straight from the fractions
        let Some(rational) = parameter.as_rational() else {
            return Err(Decline::IrrationalParameter);
        };
        let x = start.0.add(&step.0.mul(&rational));
        let y = start.1.add(&step.1.mul(&rational));
        found.push(Contact { alpha, station, x: rational_text(&x), y: rational_text(&y) });
    }
    if found.len() >= 64 {
        return Err(Decline::SortTooLong);
    }
    let alphas: Vec<Radical> = found.iter().map(|contact| contact.alpha.clone()).collect();
    let order = sorted_order(&alphas)?;
    let mut slots: Vec<Option<Contact>> = found.into_iter().map(Some).collect();
    Ok(order.into_iter().map(|index| slots[index].take().expect("a sort order is a permutation")).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cftuv_core::num::IBig;

    fn q(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    fn rational(n: i64) -> Radical {
        Radical::rational(q(n, 1))
    }

    #[test]
    fn a_unit_square_corner_with_the_identity_metric() {
        // source on the x axis from (0,0), tangent covector (1,0), normal covector (0,1), length 1; barrier segment (0,1) -> (1,1): alpha 1 along the whole segment
        let tangent: Pair = (rational(1), Radical::zero());
        let normal: Pair = (Radical::zero(), rational(1));
        let length = rational(1);
        let origin = (Rat::zero(), Rat::zero());
        let frame = LeafFrame { origin: &origin, length: &length, tangent: &tangent, normal: &normal };
        let found = contacts(&frame, &(q(0, 1), q(1, 1)), &(q(1, 1), q(1, 1))).unwrap();
        // da = 0, ds = 1: candidates 0, 1, -s0/ds = 0, (L - s0)/ds = 1; distinct 0 and 1
        assert_eq!(found.len(), 2);
        assert_eq!(found[0].x, "Integer(0)");
        assert_eq!(found[1].x, "Integer(1)");
        assert_eq!(found[0].y, "Integer(1)");
        assert_eq!(found[0].alpha, rational(1));
        // a tie in alpha keeps the order of insertion (a stable sort)
        assert_eq!(found[0].station, rational(0));
        assert_eq!(found[1].station, rational(1));
    }

    #[test]
    fn an_irrational_parameter_is_declined_not_guessed() {
        // G t = (sqrt(2), 0), L = 1: the parameter (L - s0) / ds = 1 / sqrt(2) is irrational and inside [0, 1]; its station is 1 and its alpha is y0 + p * dy >= 0
        let root = Radical::from_terms(vec![(cftuv_core::num::UBig::from(2u8), Rat::one())]);
        let tangent: Pair = (root, Radical::zero());
        let normal: Pair = (Radical::zero(), rational(1));
        let length = rational(1);
        let origin = (Rat::zero(), Rat::zero());
        let frame = LeafFrame { origin: &origin, length: &length, tangent: &tangent, normal: &normal };
        let declined = contacts(&frame, &(q(0, 1), q(1, 1)), &(q(1, 1), q(2, 1)));
        assert_eq!(declined.unwrap_err(), Decline::IrrationalParameter);
    }

    #[test]
    fn four_classes_are_declined_by_the_algebra() {
        // ds with four square classes: the inverse refuses
        let many = Radical::from_terms(vec![
            (cftuv_core::num::UBig::from(1u8), Rat::one()),
            (cftuv_core::num::UBig::from(2u8), Rat::one()),
            (cftuv_core::num::UBig::from(3u8), Rat::one()),
            (cftuv_core::num::UBig::from(5u8), Rat::one()),
        ]);
        let tangent: Pair = (many, Radical::zero());
        let normal: Pair = (Radical::zero(), rational(1));
        let length = rational(1);
        let origin = (Rat::zero(), Rat::zero());
        let frame = LeafFrame { origin: &origin, length: &length, tangent: &tangent, normal: &normal };
        let declined = contacts(&frame, &(q(0, 1), q(1, 1)), &(q(1, 1), q(2, 1)));
        assert_eq!(declined.unwrap_err(), Decline::Refused(Refused::OutsideField));
    }

    #[test]
    fn the_sort_asks_the_questions_of_count_run_and_binary_insertion() {
        let alphas = vec![rational(3), rational(1), rational(2)];
        // descending run 3 > 1 (length 2), then 2 inserted by bisection: order 1, 2, 3
        assert_eq!(sorted_order(&alphas).unwrap(), vec![1, 2, 0]);
        let ties = vec![rational(1), rational(1), rational(0), rational(1)];
        assert_eq!(sorted_order(&ties).unwrap(), vec![2, 0, 1, 3]);
    }
}
