//! The integer a geometric pass runs on: `i128` when the numbers provably fit, `IBig` (dashu) when they might not.
//!
//! The pass never divides and never reduces: a coordinate is an integer numerator over the common denominator of its map (`scale.rs`), so every predicate below is a sign or a zero test
//! of a polynomial in those integers. With `|coordinate| < 2^B`, a difference is `< 2^(B+1)`, a component of a cross product `< 2^(2B+3)`, and the dot product of a difference with a
//! cross product `< 2^(3B+6)`; `i128` holds that for `B <= FIXED_BITS` and the arithmetic below is then plain (a debug build, and so every `cargo test`, panics on an overflow).

use std::cmp::Ordering;

use dashu_int::IBig;

/// The widest coordinate (in bits of the magnitude) for which every intermediate of `relation3` fits an `i128`: `3 * 40 + 6 = 126 <= 127`.
pub const FIXED_BITS: usize = 40;

/// The operations the predicates need. `Ord` is the order of the integers.
pub trait Num: Clone + Ord {
    fn zero() -> Self;
    /// The value in this representation, or `None` when it does not fit.
    fn from_big(value: &IBig) -> Option<Self>;
    fn add(&self, other: &Self) -> Self;
    fn sub(&self, other: &Self) -> Self;
    fn mul(&self, other: &Self) -> Self;
    /// `-1`, `0` or `1`.
    fn sign(&self) -> i8;

    fn is_zero(&self) -> bool {
        self.sign() == 0
    }

    fn compare(&self, other: &Self) -> Ordering {
        self.cmp(other)
    }
}

impl Num for i128 {
    #[inline]
    fn zero() -> i128 {
        0
    }

    #[inline]
    fn from_big(value: &IBig) -> Option<i128> {
        i128::try_from(value).ok()
    }

    #[inline]
    fn add(&self, other: &i128) -> i128 {
        self + other
    }

    #[inline]
    fn sub(&self, other: &i128) -> i128 {
        self - other
    }

    #[inline]
    fn mul(&self, other: &i128) -> i128 {
        self * other
    }

    #[inline]
    fn sign(&self) -> i8 {
        self.signum() as i8
    }
}

impl Num for IBig {
    fn zero() -> IBig {
        IBig::ZERO
    }

    fn from_big(value: &IBig) -> Option<IBig> {
        Some(value.clone())
    }

    fn add(&self, other: &IBig) -> IBig {
        self + other
    }

    fn sub(&self, other: &IBig) -> IBig {
        self - other
    }

    fn mul(&self, other: &IBig) -> IBig {
        self * other
    }

    fn sign(&self) -> i8 {
        if *self == IBig::ZERO {
            0
        } else if *self < IBig::ZERO {
            -1
        } else {
            1
        }
    }
}
