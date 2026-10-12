//! The integer functions `radical_v2.py` takes from `math` (`gcd`, `isqrt`, `lcm`) and its `_reduced`, over `dashu-int`.
//!
//! `UBig` holds radicands, denominators and magnitudes, `IBig` numerators. Everything here is exact; there is no float anywhere in the crate.

use dashu_base::SquareRootRem;
use dashu_int::ops::{Gcd, UnsignedAbs};

pub use dashu_int::{IBig, UBig};

/// The binary gcd of two machine words (`gcd(0, x) = x`): the whole of the common case, where numerators, denominators and radicands are small. It is exact for every input; the cost is
/// the point -- about half of the multi-word Lehmer gcd of `dashu-int` on 62-bit operands, a third on 30-bit ones.
#[inline]
pub fn gcd_u64(mut a: u64, mut b: u64) -> u64 {
    if a == 0 {
        return b;
    }
    if b == 0 {
        return a;
    }
    if a == 1 || b == 1 {
        return 1;
    }
    let shift = (a | b).trailing_zeros();
    a >>= a.trailing_zeros();
    loop {
        b >>= b.trailing_zeros();
        if a > b {
            std::mem::swap(&mut a, &mut b);
        }
        b -= a;
        if b == 0 {
            return a << shift;
        }
    }
}

/// The gcd of two double words when at least one of them is a single word: one remainder (the Euclid step), then the binary gcd of words. `None` when both need two words.
#[inline]
fn gcd_u128(a: u128, b: u128) -> Option<u128> {
    let (big, small) = if a >= b { (a, b) } else { (b, a) };
    if small >> 64 != 0 {
        return None;
    }
    if small == 0 {
        return Some(big);
    }
    // big % small < small < 2^64
    Some(u128::from(gcd_u64(small as u64, (big % small) as u64)))
}

/// `math.gcd` of two non-negative integers (`gcd(0, 0) = 0`): machine words on the machine (two words when one of them is small), `dashu-int` beyond.
#[inline]
pub fn gcd(a: &UBig, b: &UBig) -> UBig {
    if let (Ok(x), Ok(y)) = (u64::try_from(a), u64::try_from(b)) {
        return UBig::from(gcd_u64(x, y));
    }
    if let (Ok(x), Ok(y)) = (u128::try_from(a), u128::try_from(b)) {
        if let Some(found) = gcd_u128(x, y) {
            return UBig::from(found);
        }
    }
    if a.is_one() || b.is_one() {
        return UBig::ONE;
    }
    if a.is_zero() && b.is_zero() {
        return UBig::ZERO;
    }
    a.gcd(b)
}

/// `math.gcd(a, b)` for a signed `a` and a non-negative `b`.
#[inline]
pub fn gcd_signed(a: &IBig, b: &UBig) -> UBig {
    gcd(&magnitude(a), b)
}

/// `math.lcm` of two positive integers.
pub fn lcm(a: &UBig, b: &UBig) -> UBig {
    let common = gcd(a, b);
    (a / &common) * b
}

/// `abs(n)`.
#[inline]
pub fn magnitude(n: &IBig) -> UBig {
    n.unsigned_abs()
}

/// The signed integer of a non-negative one.
#[inline]
pub fn signed(n: &UBig) -> IBig {
    IBig::from(n.clone())
}

/// `-1`, `0` or `1`.
#[inline]
pub fn signum(n: &IBig) -> i8 {
    if n.is_zero() {
        0
    } else if *n < IBig::ZERO {
        -1
    } else {
        1
    }
}

/// `_reduced(numerator, denominator)` for a positive denominator: lowest terms (`0/d` is `0/1`).
pub fn reduced(numerator: IBig, denominator: UBig) -> (IBig, UBig) {
    debug_assert!(!denominator.is_zero());
    if denominator.is_one() {
        return (numerator, denominator);
    }
    let common = gcd_signed(&numerator, &denominator);
    if common.is_one() {
        return (numerator, denominator);
    }
    (numerator / &common, denominator / common)
}

/// `Fraction(numerator, denominator)`: any sign, any common factor; `None` for a zero denominator (`ZeroDivisionError`).
pub fn fraction(numerator: IBig, denominator: IBig) -> Option<(IBig, UBig)> {
    if denominator.is_zero() {
        return None;
    }
    if denominator < IBig::ZERO {
        Some(reduced(-numerator, magnitude(&denominator)))
    } else {
        Some(reduced(numerator, magnitude(&denominator)))
    }
}

/// Squares modulo 64 are only these 12 residues: a number whose low six bits are not among them is certainly not a perfect square (a filter in front of the root, never a decision).
const SQUARES_MOD_64: u64 = {
    let mut mask = 0u64;
    let mut n = 0u64;
    while n < 64 {
        mask |= 1 << ((n * n) & 63);
        n += 1;
    }
    mask
};

/// `isqrt(n)` when `n` is a perfect square, else `None` (`n >= 1` in the callers; `0` and `1` are their own roots).
pub fn exact_root(n: &UBig) -> Option<UBig> {
    let low = n.as_words().first().copied().unwrap_or(0);
    if SQUARES_MOD_64 >> (low & 63) & 1 == 0 {
        return None;
    }
    let (root, rest) = n.sqrt_rem();
    if rest.is_zero() {
        Some(root)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_square_filter_never_rejects_a_square() {
        for n in 0u64..4000 {
            let square = UBig::from(n * n);
            assert_eq!(exact_root(&square), Some(UBig::from(n)), "{n}^2");
            if n > 0 {
                assert_eq!(exact_root(&(square.clone() + UBig::ONE)), None, "{n}^2 + 1");
            }
        }
        let big = UBig::from_str_radix("123456789012345678901234567890123456789", 10).unwrap();
        assert_eq!(exact_root(&(&big * &big)), Some(big.clone()));
        assert_eq!(exact_root(&(&big * &big + UBig::ONE)), None);
    }

    #[test]
    fn the_machine_word_gcds_are_the_gcd_of_dashu_at_every_size() {
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let sizes = [0u32, 1, 2, 7, 31, 32, 63, 64, 65, 96, 127, 128, 129, 200];
        for &left in &sizes {
            for &right in &sizes {
                for _ in 0..40 {
                    let mut build = |bits: u32| -> UBig {
                        if bits == 0 {
                            return UBig::ZERO;
                        }
                        let mut value = UBig::ONE;
                        for _ in 0..bits.div_ceil(64) {
                            value = (value << 64usize) | UBig::from(next());
                        }
                        // a random common factor now and then, and a power of two
                        let value = value >> ((bits.div_ceil(64) * 64 - bits + 1) as usize);
                        value | UBig::ONE
                    };
                    let (a, b) = (build(left), build(right));
                    let common = UBig::from(next() % 1000 + 1) << (next() % 70) as usize;
                    for (x, y) in [(a.clone(), b.clone()), (&a * &common, &b * &common), (a.clone(), a.clone()), (UBig::ZERO, b.clone()), (a.clone(), UBig::ZERO), (UBig::ONE, b.clone())] {
                        let expected = if x.is_zero() && y.is_zero() { UBig::ZERO } else { dashu_int::ops::Gcd::gcd(&x, &y) };
                        assert_eq!(gcd(&x, &y), expected, "gcd({x}, {y})");
                        assert_eq!(gcd_signed(&-signed(&x), &y), expected, "gcd(-{x}, {y})");
                    }
                }
            }
        }
        assert_eq!(gcd_u64(0, 0), 0);
        assert_eq!(gcd_u64(0, 9), 9);
        assert_eq!(gcd_u64(1 << 63, 1 << 40), 1 << 40);
    }

    #[test]
    fn reduced_follows_fraction_at_the_edges() {
        assert_eq!(reduced(IBig::ZERO, UBig::from(12u8)), (IBig::ZERO, UBig::ONE));
        assert_eq!(reduced(IBig::from(-6), UBig::from(4u8)), (IBig::from(-3), UBig::from(2u8)));
        assert_eq!(fraction(IBig::from(6), IBig::from(-4)), Some((IBig::from(-3), UBig::from(2u8))));
        assert_eq!(fraction(IBig::ONE, IBig::ZERO), None);
        assert_eq!(lcm(&UBig::from(4u8), &UBig::from(6u8)), UBig::from(12u8));
    }
}
