//! Integerising a map of positions: every coordinate becomes a numerator over the ONE common denominator of the map.
//!
//! `before` is dyadic (a binary64 read exactly) and `after` is `k / scale`, so the common denominator is the largest denominator when all of them are powers of two, and the `lcm` otherwise
//! (computed once per call, `O(V)` gcds at most, never in the pair loops). Scaling every coordinate by the same positive factor changes no zero test, no sign and no comparison of the
//! predicates, and equal values stay equal: two coordinates are equal exactly when their numerators are.

use dashu_int::ops::{Gcd, PowerOfTwo};
use dashu_int::{IBig, UBig};

/// A coordinate as the host holds it: `numerator / denominator` with a positive denominator (not necessarily in lowest terms).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rational {
    pub numerator: IBig,
    pub denominator: UBig,
}

impl Rational {
    pub fn new(numerator: IBig, denominator: UBig) -> Rational {
        assert!(denominator != UBig::ZERO, "a rational needs a positive denominator");
        Rational { numerator, denominator }
    }

    pub fn from_i64(value: i64) -> Rational {
        Rational { numerator: IBig::from(value), denominator: UBig::ONE }
    }
}

pub type RPoint = [Rational; 3];

/// A map in integers: `coords[i][k] / denominator` is the `k`-th coordinate of vertex `i`; `bits` bounds every magnitude (`|numerator| < 2^bits`).
#[derive(Clone, Debug)]
pub struct Scaled {
    pub denominator: UBig,
    pub coords: Vec<[IBig; 3]>,
    pub bits: usize,
}

/// The bit length of `|value|` (zero for zero), from its words: no allocation.
fn magnitude_bits(value: &IBig) -> usize {
    let (_, words) = value.as_sign_words();
    match words.last() {
        Some(top) => words.len() * 64 - top.leading_zeros() as usize,
        None => 0,
    }
}

fn lcm(left: &UBig, right: &UBig) -> UBig {
    let common = left.gcd(right);
    (left / &common) * right
}

/// The common denominator: the largest one when every denominator is a power of two (no gcd), the `lcm` of all of them otherwise.
fn common_denominator(points: &[RPoint]) -> UBig {
    let all = || points.iter().flat_map(|point| point.iter().map(|item| &item.denominator));
    if all().all(|item| item.is_power_of_two()) {
        return all().max().cloned().unwrap_or(UBig::ONE);
    }
    all().fold(UBig::ONE, |common, item| if item == &UBig::ONE || item == &common { common } else { lcm(&common, item) })
}

pub fn scale(points: &[RPoint]) -> Scaled {
    let denominator = common_denominator(points);
    let mut bits = 0usize;
    let mut cached: Option<(UBig, IBig)> = None;
    let coords = points
        .iter()
        .map(|point| {
            let mut out = [IBig::ZERO, IBig::ZERO, IBig::ZERO];
            for (slot, item) in out.iter_mut().zip(point.iter()) {
                let value = if item.denominator == denominator {
                    item.numerator.clone()
                } else {
                    let factor = match &cached {
                        Some((key, factor)) if *key == item.denominator => factor.clone(),
                        _ => {
                            let factor = IBig::from(&denominator / &item.denominator);
                            cached = Some((item.denominator.clone(), factor.clone()));
                            factor
                        }
                    };
                    &item.numerator * factor
                };
                bits = bits.max(magnitude_bits(&value));
                *slot = value;
            }
            out
        })
        .collect();
    Scaled { denominator, coords, bits }
}

/// Whether `left` and `right` (two scaled maps of the same vertices) hold equal values at vertex `index`.
pub fn same_value(left: &Scaled, right: &Scaled, index: usize) -> bool {
    if left.denominator == right.denominator {
        return left.coords[index] == right.coords[index];
    }
    let (left_factor, right_factor) = (IBig::from(right.denominator.clone()), IBig::from(left.denominator.clone()));
    (0..3).all(|k| &left.coords[index][k] * &left_factor == &right.coords[index][k] * &right_factor)
}
