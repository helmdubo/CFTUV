//! Point facts the stage reads: the identity key of a point (`coalesce.point_key`), its integer record when both
//! coordinates are rational (`clip._rational_pair`), and the owned point type.

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

/// A point of the plane that owns its coordinates (the values of the `points` dictionaries).
pub type Point = (SqrtSum, SqrtSum);

/// `point_key(point)`: `(x.terms, y.terms)`, equality by value (`int 3 == Fraction(3, 1)`, so the Python type of a
/// coefficient is not part of the identity; the first object interned stays the representative, see the stage).
pub type PointKey = (Vec<(UBig, Rat)>, Vec<(UBig, Rat)>);

fn terms_of(value: &SqrtSum) -> Vec<(UBig, Rat)> {
    value.terms().iter().map(|term| (term.radicand.clone(), term.coef.value().clone())).collect()
}

pub fn point_key(point: &Point) -> PointKey {
    (terms_of(&point.0), terms_of(&point.1))
}

/// Mixes the words of a rational (sign, numerator, denominator) into a hash state of the same recipe as [`point_hash`].
pub fn mix_rat(state: &mut u64, value: &Rat) {
    const SEED: u64 = 0x517c_c1b7_2722_0a95;
    let mut mix = |word: u64| *state = (state.rotate_left(5) ^ word).wrapping_mul(SEED);
    let (sign, words) = value.numerator().as_sign_words();
    mix(sign as u64);
    words.iter().for_each(|word| mix(*word));
    mix(3);
    value.denominator().as_words().iter().for_each(|word| mix(*word));
    mix(5);
}

/// A hash of the identity of a point (the words of every radicand and coefficient, in term order): equal by [`same_point`] implies
/// equal here, and nothing is copied (`point_key` clones every term to build its key).
pub fn point_hash(point: &Point) -> u64 {
    const SEED: u64 = 0x517c_c1b7_2722_0a95;
    let mut state = 0u64;
    let mut mix = |word: u64| state = (state.rotate_left(5) ^ word).wrapping_mul(SEED);
    for coordinate in [&point.0, &point.1] {
        mix(coordinate.terms().len() as u64 | (1 << 62));
        for term in coordinate.terms() {
            let value = term.coef.value();
            term.radicand.as_words().iter().for_each(|word| mix(*word));
            mix(2);
            let (sign, words) = value.numerator().as_sign_words();
            mix(sign as u64);
            words.iter().for_each(|word| mix(*word));
            mix(3);
            value.denominator().as_words().iter().for_each(|word| mix(*word));
        }
    }
    state
}

/// `point_key(a) == point_key(b)` without building the keys: the same radicands with the same coefficient VALUES (an `int` coefficient
/// and the `Fraction` of the same value are the same identity).
pub fn same_point(left: &Point, right: &Point) -> bool {
    let same = |a: &SqrtSum, b: &SqrtSum| {
        a.terms().len() == b.terms().len() && a.terms().iter().zip(b.terms()).all(|(x, y)| x.radicand == y.radicand && x.coef.value() == y.coef.value())
    };
    same(&left.0, &right.0) && same(&left.1, &right.1)
}

/// `(x numerator, x denominator, y numerator, y denominator)` of a rational point: a coordinate is rational when it
/// has no term or one term at radicand 1.
pub type RationalPair = (IBig, UBig, IBig, UBig);

/// `clip._rational_pair(point)`.
pub fn rational_pair(point: &Point) -> Option<RationalPair> {
    let mut parts: Vec<(IBig, UBig)> = Vec::with_capacity(2);
    for coordinate in [&point.0, &point.1] {
        let terms = coordinate.terms();
        match terms {
            [] => parts.push((IBig::ZERO, UBig::ONE)),
            [only] if only.radicand.is_one() => {
                let value = only.coef.value();
                parts.push((value.numerator().clone(), value.denominator().clone()));
            }
            _ => return None,
        }
    }
    let (second, first) = (parts.pop()?, parts.pop()?);
    Some((first.0, first.1, second.0, second.1))
}
