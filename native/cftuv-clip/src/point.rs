//! Point facts the stage reads: the identity key of a point (`coalesce.point_key`), its integer record when both
//! coordinates are rational (`clip._rational_pair`), and the owned point type.
//!
//! The identity of a coordinate is its integer form in lowest terms (`SqrtSum::canonical_form`): unique per value, so equal values have equal
//! forms whichever way they were made, and a coordinate that was only ever computed with never has its per-term fractions normalised.

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{IntForm, SqrtSum};

/// A point of the plane that owns its coordinates (the values of the `points` dictionaries).
pub type Point = (SqrtSum, SqrtSum);

/// `point_key(point)`: the value of `(x, y)`, equality by value (`int 3 == Fraction(3, 1)`, so the Python type of a
/// coefficient is not part of the identity; the first object interned stays the representative, see the stage).
pub type PointKey = (IntForm, IntForm);

pub fn point_key(point: &Point) -> PointKey {
    (point.0.canonical_form().clone(), point.1.canonical_form().clone())
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

/// Mixes the words of a form (its common denominator, then every radicand and numerator in order) into a hash state of the same recipe as
/// [`point_hash`].
pub fn mix_form(state: &mut u64, form: &IntForm) {
    const SEED: u64 = 0x517c_c1b7_2722_0a95;
    let mut mix = |word: u64| *state = (state.rotate_left(5) ^ word).wrapping_mul(SEED);
    mix(form.items.len() as u64 | (1 << 62));
    form.common.as_words().iter().for_each(|word| mix(*word));
    mix(5);
    for (radicand, numerator) in &form.items {
        radicand.as_words().iter().for_each(|word| mix(*word));
        mix(2);
        let (sign, words) = numerator.as_sign_words();
        mix(sign as u64);
        words.iter().for_each(|word| mix(*word));
        mix(3);
    }
}

/// A hash of the identity of a point (the words of the forms of its two coordinates in lowest terms): equal by [`same_point`] implies equal
/// here, and nothing is copied.
pub fn point_hash(point: &Point) -> u64 {
    let mut state = 0u64;
    mix_form(&mut state, point.0.canonical_form());
    mix_form(&mut state, point.1.canonical_form());
    state
}

/// `point_key(a) == point_key(b)` without building the keys: the same value in each coordinate (an `int` coefficient and the `Fraction` of the
/// same value are the same identity).
pub fn same_point(left: &Point, right: &Point) -> bool {
    left.0.canonical_form() == right.0.canonical_form() && left.1.canonical_form() == right.1.canonical_form()
}

/// `(x numerator, x denominator, y numerator, y denominator)` of a rational point: a coordinate is rational when it
/// has no term or one term at radicand 1.
pub type RationalPair = (IBig, UBig, IBig, UBig);

/// `clip._rational_pair(point)`.
pub fn rational_pair(point: &Point) -> Option<RationalPair> {
    let mut parts: Vec<(IBig, UBig)> = Vec::with_capacity(2);
    for coordinate in [&point.0, &point.1] {
        // the form in lowest terms: a lone rational term `a / L` has `gcd(a, L) = 1`, which is the fraction `Fraction` keeps
        let form = coordinate.canonical_form();
        match form.items.as_slice() {
            [] => parts.push((IBig::ZERO, UBig::ONE)),
            [(radicand, numerator)] if radicand.is_one() => parts.push((numerator.clone(), form.common.clone())),
            _ => return None,
        }
    }
    let (second, first) = (parts.pop()?, parts.pop()?);
    Some((first.0, first.1, second.0, second.1))
}
