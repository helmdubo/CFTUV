//! The angle certificate of one corner: `reference/validation._interval_contains_oriented_support_delta` for rational supports, as far as the port carries it.
//!
//! The Python predicate asks whether the certified interval of the reflex excess contains the oriented turn between two supports, i.e. two signs: that of the oriented cross of the unit supports
//! (a rational number, `owner_sign * sign(cross)`), and that of `cos(angle) - cos(pi r)` at each end `r` of the interval, which is `sign(dot - c * sqrt(Na * Nb))` with `c = cos(pi r)`, `Na`, `Nb`
//! the squared norms in the metric. `angle_certificate_sign.certified_oriented_support_delta` decides the second on a box (see `enclosure.rs`); an angle whose box straddles zero goes to the
//! exact (sympy) path, which this module does NOT carry, except for the one case that is the common one on architectural meshes and is exact in rational arithmetic:
//!
//! * `dot == 0` and `c == 0` exactly (a right angle against an interval end `r = k + 1/2`, e.g. the degenerate interval `[0.5, 0.5]` of an exact 270 degree reflex corner), with both squared
//!   norms perfect squares of rationals, so that the unit supports are rational and the exact path is rational arithmetic too (no radical arithmetic, hence none of its counters). The sign is
//!   then exactly 0 and the end is inside the interval iff it is CLOSED.
//!
//! Anything else the box does not decide is [`Reason::Undecided`]: the port declines and Python runs its exact path.

use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;
use dashu_int::ops::SquareRoot;

use crate::decimal;
use crate::enclosure::{cos_pi_enclosure, sqrt_enclosure};
use crate::model::{Endpoint, Interval, Turn};
use crate::verdict::{decline, issue, Check, Reason};

/// How Python's counters see the corner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Class {
    /// The box decided both ends (`angle_certificate_sign.certified`).
    Certified,
    /// The box did not, and the exact path decided (`angle_certificate_sign.exact_fallback.straddles_zero`).
    Straddled,
}

/// `+-1` for a box wholly on one side of `residual`, `None` when it covers it (`_proven_sign`).
fn proven_sign(residual: &Rat, root: &(Rat, Rat), cosine: &(Rat, Rat)) -> Option<i8> {
    let products = [cosine.0.mul(&root.0), cosine.0.mul(&root.1), cosine.1.mul(&root.0), cosine.1.mul(&root.1)];
    let largest = products.iter().max().expect("four products");
    let smallest = products.iter().min().expect("four products");
    if residual.sub(largest).signum() > 0 {
        return Some(1);
    }
    if residual.sub(smallest).signum() < 0 {
        return Some(-1);
    }
    None
}

/// `numerator` and `denominator` of a reduced rational are both squares.
fn is_rational_square(value: &Rat) -> bool {
    let numerator = num::magnitude(value.numerator());
    let root = numerator.sqrt();
    if &root * &root != numerator {
        return false;
    }
    let denominator_root = value.denominator().sqrt();
    &denominator_root * &denominator_root == *value.denominator()
}

/// `r = k + 1/2` for an integer `k`: `cos(pi r)` is exactly 0.
fn cosine_is_exactly_zero(end: &Rat) -> bool {
    end.denominator() == &UBig::from(2u8) && !end.numerator().is_zero()
}

fn quadratic(gram: &[[Rat; 2]; 2], a: &[Rat; 2], b: &[Rat; 2]) -> Rat {
    // a . G b  =  a0 (g00 b0 + g01 b1) + a1 (g10 b0 + g11 b1)
    a[0].mul(&gram[0][0].mul(&b[0]).add(&gram[0][1].mul(&b[1]))).add(&a[1].mul(&gram[1][0].mul(&b[0]).add(&gram[1][1].mul(&b[1]))))
}

/// `matches` of the Python predicate is TRUE for these supports and this interval, and which counter it leaves; anything else is a decline (`Issue` if Python says no, `Undecided` if the
/// port cannot say).
pub fn contains_turn(gram: &[[Rat; 2]; 2], owner_sign: i8, incoming: &[Rat; 2], outgoing: &[Rat; 2], turn: Turn, interval: &Interval) -> Check<Class> {
    let incoming_squared = quadratic(gram, incoming, incoming);
    let outgoing_squared = quadratic(gram, outgoing, outgoing);
    let cross = incoming[0].mul(&outgoing[1]).sub(&incoming[1].mul(&outgoing[0]));
    if incoming_squared.signum() <= 0 || outgoing_squared.signum() <= 0 || cross.is_zero() {
        return decline(Reason::Undecided, "a degenerate support pair: the exact path raises or answers by itself");
    }
    let turn_sign: i8 = owner_sign * if cross.signum() > 0 { 1 } else { -1 };
    let expected_sign: i8 = if turn == Turn::Ccw { 1 } else { -1 };
    if turn_sign != expected_sign {
        return issue("the oriented turn of the supports has the other orientation than the certificate");
    }
    let ends = [decimal::to_rat(&interval.lower), decimal::to_rat(&interval.upper)];
    let dot = quadratic(gram, incoming, outgoing);
    let radicand = incoming_squared.mul(&outgoing_squared);
    let root = sqrt_enclosure(&radicand);
    let mut signs = [0i8; 2];
    let mut straddled = false;
    for (index, end) in ends.iter().enumerate() {
        let cosine = cos_pi_enclosure(end.numerator(), end.denominator());
        match proven_sign(&dot, &root, &cosine) {
            Some(sign) => signs[index] = sign,
            None => {
                straddled = true;
                if dot.is_zero() && cosine_is_exactly_zero(end) && is_rational_square(&incoming_squared) && is_rational_square(&outgoing_squared) {
                    signs[index] = 0;
                } else {
                    return decline(Reason::Undecided, "the sign of cos(angle) - cos(pi r) is not decided by the box, and the exact path is not carried for this angle");
                }
            }
        }
    }
    let lower_ok = signs[0] < 0 || (signs[0] == 0 && interval.lower_kind == Endpoint::Closed);
    let upper_ok = signs[1] > 0 || (signs[1] == 0 && interval.upper_kind == Endpoint::Closed);
    if !(lower_ok && upper_ok) {
        return issue("the certified reflex excess does not contain the exact oriented support turn");
    }
    Ok(if straddled { Class::Straddled } else { Class::Certified })
}

#[allow(dead_code)]
fn unused(_: &IBig) {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Dec;

    fn rat(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    fn identity() -> [[Rat; 2]; 2] {
        [[rat(1, 1), rat(0, 1)], [rat(0, 1), rat(1, 1)]]
    }

    fn closed(text: &str) -> Interval {
        let value = decimal::parse(text).unwrap();
        Interval { lower: value.clone(), upper: value, lower_kind: Endpoint::Closed, upper_kind: Endpoint::Closed }
    }

    fn open_at(text: &str) -> Interval {
        let value: Dec = decimal::parse(text).unwrap();
        Interval { lower: value.clone(), upper: value, lower_kind: Endpoint::Open, upper_kind: Endpoint::Open }
    }

    #[test]
    fn an_exact_right_angle_against_a_closed_half_is_straddled_and_inside() {
        // supports e_x and e_y, G = I: dot = 0, cross = 1 (counter-clockwise); interval [0.5, 0.5]
        let outcome = contains_turn(&identity(), 1, &[rat(1, 1), rat(0, 1)], &[rat(0, 1), rat(1, 1)], Turn::Ccw, &closed("0.500000000000000000000000000"));
        assert_eq!(outcome, Ok(Class::Straddled));
    }

    #[test]
    fn an_open_end_at_the_exact_right_angle_is_not_inside() {
        let outcome = contains_turn(&identity(), 1, &[rat(1, 1), rat(0, 1)], &[rat(0, 1), rat(1, 1)], Turn::Ccw, &open_at("0.5"));
        assert!(matches!(outcome, Err(ref decline) if decline.reason == Reason::Issue), "{outcome:?}");
    }

    #[test]
    fn the_other_orientation_is_an_issue() {
        let outcome = contains_turn(&identity(), 1, &[rat(1, 1), rat(0, 1)], &[rat(0, 1), rat(1, 1)], Turn::Cw, &closed("0.5"));
        assert!(matches!(outcome, Err(ref decline) if decline.reason == Reason::Issue));
    }

    #[test]
    fn a_right_angle_with_irrational_unit_supports_is_left_to_python() {
        // |(1,1)|^2 = 2 is not a rational square: the exact path would use radicals
        let outcome = contains_turn(&identity(), 1, &[rat(1, 1), rat(1, 1)], &[rat(-1, 1), rat(1, 1)], Turn::Ccw, &closed("0.5"));
        assert!(matches!(outcome, Err(ref decline) if decline.reason == Reason::Undecided), "{outcome:?}");
    }

    #[test]
    fn an_ordinary_angle_inside_its_interval_is_certified() {
        // the supports (1,0) and (3,4) on G = I turn counter-clockwise by acos(3/5) = 0.29516723530086... pi
        let inside = Interval { lower: decimal::parse("0.29516").unwrap(), upper: decimal::parse("0.29517").unwrap(), lower_kind: Endpoint::Closed, upper_kind: Endpoint::Closed };
        let outcome = contains_turn(&identity(), 1, &[rat(1, 1), rat(0, 1)], &[rat(3, 1), rat(4, 1)], Turn::Ccw, &inside);
        assert_eq!(outcome, Ok(Class::Certified));
        // an interval that does not contain the angle is an issue
        let away = Interval { lower: decimal::parse("0.30").unwrap(), upper: decimal::parse("0.31").unwrap(), lower_kind: Endpoint::Closed, upper_kind: Endpoint::Closed };
        let outcome = contains_turn(&identity(), 1, &[rat(1, 1), rat(0, 1)], &[rat(3, 1), rat(4, 1)], Turn::Ccw, &away);
        assert!(matches!(outcome, Err(ref decline) if decline.reason == Reason::Issue), "{outcome:?}");
    }
}
