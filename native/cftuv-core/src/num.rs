//! Integer helpers over `dashu-int` (the representation chosen by `examples/bigint_bench.rs`).
//!
//! `UBig` is a non-negative integer (radicands, denominators, magnitudes), `IBig` a signed one (numerators,
//! sqrt-sum coefficients over a common denominator). Everything here mirrors the Python integer function it is
//! named after, including the edge cases that matter for exactness (`gcd(0, 0) = 0`, `lcm(0, x) = 0`).

use dashu_int::ops::{BitTest, Gcd, SquareRoot};

pub use dashu_int::{IBig, UBig};
pub use dashu_int::Sign;

/// `math.isqrt` for a non-negative integer: the floor of the square root.
pub fn isqrt(n: &UBig) -> UBig {
    n.sqrt()
}

/// `math.gcd` of two non-negative integers.
pub fn gcd(a: &UBig, b: &UBig) -> UBig {
    if a.is_zero() && b.is_zero() {
        return UBig::ZERO;
    }
    a.gcd(b)
}

/// `math.gcd` of two signed integers (the result is non-negative).
pub fn gcd_signed(a: &IBig, b: &IBig) -> UBig {
    if a.is_zero() && b.is_zero() {
        return UBig::ZERO;
    }
    a.gcd(b)
}

/// `math.gcd(a, b)` for a signed `a` and a non-negative `b`.
pub fn gcd_mixed(a: &IBig, b: &UBig) -> UBig {
    if a.is_zero() && b.is_zero() {
        return UBig::ZERO;
    }
    a.gcd(b)
}

/// `math.lcm` of two non-negative integers (`lcm(0, x) = 0`, as in Python).
pub fn lcm(a: &UBig, b: &UBig) -> UBig {
    if a.is_zero() || b.is_zero() {
        return UBig::ZERO;
    }
    let common = gcd(a, b);
    (a / &common) * b
}

/// `int.bit_length` of a non-negative integer.
pub fn bit_length(n: &UBig) -> usize {
    n.bit_len()
}

/// `abs(n)` as an unsigned integer.
pub fn magnitude(n: &IBig) -> UBig {
    let (_, words) = n.as_sign_words();
    UBig::from_words(words)
}

/// `-1`, `0` or `1`.
pub fn signum(n: &IBig) -> i8 {
    if n.is_zero() {
        0
    } else if n.sign() == Sign::Negative {
        -1
    } else {
        1
    }
}

pub fn is_negative(n: &IBig) -> bool {
    !n.is_zero() && n.sign() == Sign::Negative
}

/// A signed integer from a sign flag and a magnitude (`negative` with a zero magnitude is zero).
pub fn from_sign_magnitude(negative: bool, magnitude: UBig) -> IBig {
    if negative {
        -IBig::from(magnitude)
    } else {
        IBig::from(magnitude)
    }
}

/// Little-endian magnitude bytes without trailing zero bytes (zero is the empty slice).
pub fn magnitude_le_bytes(n: &UBig) -> Vec<u8> {
    let mut bytes: Vec<u8> = Vec::with_capacity(n.as_words().len() * 8);
    for word in n.as_words() {
        bytes.extend_from_slice(&word.to_le_bytes());
    }
    while bytes.last() == Some(&0) {
        bytes.pop();
    }
    bytes
}

/// `abs(n)` as little-endian bytes together with the sign flag.
pub fn to_sign_le_bytes(n: &IBig) -> (bool, Vec<u8>) {
    let (sign, words) = n.as_sign_words();
    let mut bytes: Vec<u8> = Vec::with_capacity(words.len() * 8);
    for word in words {
        bytes.extend_from_slice(&word.to_le_bytes());
    }
    while bytes.last() == Some(&0) {
        bytes.pop();
    }
    (!bytes.is_empty() && sign == Sign::Negative, bytes)
}

/// The inverse of [`to_sign_le_bytes`]; trailing zero bytes are tolerated.
pub fn from_sign_le_bytes(negative: bool, bytes: &[u8]) -> IBig {
    from_sign_magnitude(negative, UBig::from_le_bytes(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn big(text: &str) -> IBig {
        text.parse().unwrap()
    }

    #[test]
    fn isqrt_is_the_floor_of_the_square_root() {
        for n in 0u64..2000 {
            let root = isqrt(&UBig::from(n));
            let expected = (n as f64).sqrt().floor() as u64;
            assert_eq!(root, UBig::from(expected), "isqrt({n})");
        }
        let huge: UBig = "123456789012345678901234567890123456789012345678901234567890".parse().unwrap();
        let root = isqrt(&huge);
        assert!(&root * &root <= huge);
        let next = &root + UBig::ONE;
        assert!(&next * &next > huge);
        let square = &huge * &huge;
        assert_eq!(isqrt(&square), huge);
        assert_eq!(isqrt(&(&square - UBig::ONE)), &huge - UBig::ONE);
    }

    #[test]
    fn gcd_and_lcm_follow_python_at_the_edges() {
        assert_eq!(gcd(&UBig::ZERO, &UBig::ZERO), UBig::ZERO);
        assert_eq!(gcd(&UBig::ZERO, &UBig::from(12u8)), UBig::from(12u8));
        assert_eq!(gcd_signed(&IBig::from(-12), &IBig::from(18)), UBig::from(6u8));
        assert_eq!(gcd_signed(&IBig::from(-12), &IBig::from(-18)), UBig::from(6u8));
        assert_eq!(lcm(&UBig::ZERO, &UBig::from(5u8)), UBig::ZERO);
        assert_eq!(lcm(&UBig::from(4u8), &UBig::from(6u8)), UBig::from(12u8));
        assert_eq!(lcm(&UBig::from(7u8), &UBig::from(7u8)), UBig::from(7u8));
    }

    #[test]
    fn bytes_round_trip_beyond_i128() {
        for text in [
            "0",
            "1",
            "-1",
            "255",
            "256",
            "-256",
            "170141183460469231731687303715884105727",
            "170141183460469231731687303715884105728",
            "-170141183460469231731687303715884105729",
            "340282366920938463463374607431768211456",
            "-1234567890123456789012345678901234567890123456789012345678901234567890",
        ] {
            let value = big(text);
            let (negative, bytes) = to_sign_le_bytes(&value);
            assert_eq!(from_sign_le_bytes(negative, &bytes), value, "{text}");
            assert_eq!(negative, value < IBig::ZERO);
            assert!(bytes.last().is_none_or(|&byte| byte != 0));
            assert_eq!(magnitude(&value), UBig::from_le_bytes(&bytes));
        }
    }

    #[test]
    fn signum_and_bit_length() {
        assert_eq!(signum(&IBig::ZERO), 0);
        assert_eq!(signum(&IBig::from(-5)), -1);
        assert_eq!(signum(&IBig::from(5)), 1);
        assert_eq!(bit_length(&UBig::ZERO), 0);
        assert_eq!(bit_length(&UBig::from(255u8)), 8);
        assert_eq!(bit_length(&UBig::from(256u16)), 9);
    }
}
