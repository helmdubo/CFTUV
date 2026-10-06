//! Fixed-capacity signed integers kept on the stack, for the hot arithmetic of sqrt-sum integer forms.
//!
//! A [`Wide`] is a sign and at most [`LIMBS`] little-endian 64-bit limbs. Every operation that could outgrow the capacity says so
//! (`None` / `false`) and the caller answers with the `dashu-int` road it always had: nothing here is an approximation, and a
//! result that does not fit is never truncated. The point is to take the allocator and the representation dispatch out of
//! the operands that are almost always small: of the products the conjugation loop forms, 99 % are at most 512 bits wide.
//!
//! Invariants: `limbs[len..]` are zero, `limbs[len - 1] != 0` for `len > 0`, zero is `len = 0` and never negative.

use dashu_int::ops::Gcd;

use crate::num::{IBig, Sign, UBig};

/// The capacity in limbs (640 bits).
pub const LIMBS: usize = 10;

#[derive(Clone, Copy)]
pub struct Wide {
    limbs: [u64; LIMBS],
    len: u8,
    neg: bool,
}

impl std::fmt::Debug for Wide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Wide({}{:?})", if self.neg { "-" } else { "" }, &self.limbs[..self.len as usize])
    }
}

impl PartialEq for Wide {
    fn eq(&self, other: &Wide) -> bool {
        self.neg == other.neg && self.len == other.len && self.limbs[..self.len as usize] == other.limbs[..other.len as usize]
    }
}

impl Eq for Wide {}

// ---- limb helpers (little-endian magnitudes) -------------------------------------------------------------------------

fn trimmed(words: &[u64]) -> usize {
    let mut len = words.len();
    while len > 0 && words[len - 1] == 0 {
        len -= 1;
    }
    len
}

/// `out[..a.len() + b.len()] = a * b` (schoolbook); `out` must be at least that long.
fn mul_limbs(a: &[u64], b: &[u64], out: &mut [u64]) {
    let (la, lb) = (a.len(), b.len());
    out[..la + lb].fill(0);
    for (i, &ai) in a.iter().enumerate() {
        if ai == 0 {
            continue;
        }
        let mut carry = 0u128;
        for (j, &bj) in b.iter().enumerate() {
            let total = ai as u128 * bj as u128 + out[i + j] as u128 + carry;
            out[i + j] = total as u64;
            carry = total >> 64;
        }
        out[i + lb] = carry as u64;
    }
}

/// `x^-1 mod 2^64` for an odd `x` (Newton: each step doubles the correct low bits, `x*x = 1 mod 8` seeds three).
fn inverse_mod_2_64(x: u64) -> u64 {
    debug_assert!(x & 1 == 1);
    let mut inverse = x;
    for _ in 0..5 {
        inverse = inverse.wrapping_mul(2u64.wrapping_sub(x.wrapping_mul(inverse)));
    }
    inverse
}

/// `gcd` of two `u64` (binary, branch-light); `gcd(0, x) = x`.
pub fn gcd_u64(mut a: u64, mut b: u64) -> u64 {
    if a == 0 {
        return b;
    }
    if b == 0 {
        return a;
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

/// `gcd` of two `u128` (binary).
fn gcd_u128(mut a: u128, mut b: u128) -> u128 {
    if a == 0 {
        return b;
    }
    if b == 0 {
        return a;
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
        if (a >> 64) == 0 && (b >> 64) == 0 {
            return (gcd_u64(a as u64, b as u64) as u128) << shift;
        }
    }
}

/// A word divisor with its reciprocal precomputed (Moller and Granlund, "Improved division by invariant integers", algorithm 4): the
/// remainder of a limb sequence is then a multiplication and a few adds per limb, with no hardware division and no call. Built once for a
/// divisor that is used against many numbers (the running gcd of a reduction).
#[derive(Clone, Copy, Debug)]
pub struct Divisor {
    shift: u32,
    normalized: u64,
    inverse: u64,
}

impl Divisor {
    pub fn new(word: u64) -> Divisor {
        assert!(word != 0, "a zero divisor");
        let shift = word.leading_zeros();
        let normalized = word << shift;
        // floor((2^128 - 1) / normalized) - 2^64
        let inverse = ((u128::MAX - ((normalized as u128) << 64)) / normalized as u128) as u64;
        Divisor { shift, normalized, inverse }
    }

    /// `(u1 * 2^64 + u0) mod normalized` for `u1 < normalized`.
    #[inline(always)]
    fn step(&self, u1: u64, u0: u64) -> u64 {
        let product = self.inverse as u128 * u1 as u128 + (((u1 as u128) << 64) | u0 as u128);
        let (high, low) = ((product >> 64) as u64, product as u64);
        let mut quotient = high.wrapping_add(1);
        let mut remainder = u0.wrapping_sub(quotient.wrapping_mul(self.normalized));
        if remainder > low {
            quotient = quotient.wrapping_sub(1);
            remainder = remainder.wrapping_add(self.normalized);
        }
        let _ = quotient;
        if remainder >= self.normalized {
            remainder -= self.normalized;
        }
        remainder
    }

    /// The magnitude `limbs` (little-endian) modulo the divisor.
    #[inline]
    pub fn rem(&self, limbs: &[u64]) -> u64 {
        let shift = self.shift;
        let mut remainder = 0u64;
        if shift == 0 {
            for &limb in limbs.iter().rev() {
                remainder = self.step(remainder, limb);
            }
            return remainder;
        }
        // the limbs of `x << shift` from the top: the one above the number holds only the bits that move out of it
        for index in (0..=limbs.len()).rev() {
            let high = limbs.get(index).copied().unwrap_or(0);
            let low = if index > 0 { limbs[index - 1] } else { 0 };
            remainder = self.step(remainder, (high << shift) | (low >> (64 - shift)));
        }
        remainder >> shift
    }
}

impl Wide {
    pub const ZERO: Wide = Wide { limbs: [0; LIMBS], len: 0, neg: false };

    pub fn from_u64(value: u64) -> Wide {
        let mut limbs = [0u64; LIMBS];
        limbs[0] = value;
        Wide { limbs, len: (value != 0) as u8, neg: false }
    }

    pub fn from_u128(value: u128) -> Wide {
        let mut limbs = [0u64; LIMBS];
        limbs[0] = value as u64;
        limbs[1] = (value >> 64) as u64;
        let len = if limbs[1] != 0 { 2 } else { (limbs[0] != 0) as u8 };
        Wide { limbs, len, neg: false }
    }

    /// From a sign and little-endian words (leading zero words are tolerated); `None` when the value needs more than [`LIMBS`] limbs.
    pub fn from_words(negative: bool, words: &[u64]) -> Option<Wide> {
        let len = trimmed(words);
        if len > LIMBS {
            return None;
        }
        let mut limbs = [0u64; LIMBS];
        limbs[..len].copy_from_slice(&words[..len]);
        Some(Wide { limbs, len: len as u8, neg: negative && len > 0 })
    }

    pub fn from_ibig(value: &IBig) -> Option<Wide> {
        let (sign, words) = value.as_sign_words();
        Wide::from_words(sign == Sign::Negative, words)
    }

    pub fn from_ubig(value: &UBig) -> Option<Wide> {
        Wide::from_words(false, value.as_words())
    }

    pub fn to_ibig(&self) -> IBig {
        if self.len == 0 {
            return IBig::ZERO;
        }
        IBig::from_sign_words(if self.neg { Sign::Negative } else { Sign::Positive }, &self.limbs[..self.len as usize])
    }

    /// The magnitude as an unsigned integer.
    pub fn to_ubig(&self) -> UBig {
        UBig::from_words(&self.limbs[..self.len as usize])
    }

    /// The limbs of the magnitude, least significant first (no leading zeros).
    pub fn magnitude(&self) -> &[u64] {
        &self.limbs[..self.len as usize]
    }

    pub fn limb_count(&self) -> usize {
        self.len as usize
    }

    pub fn is_zero(&self) -> bool {
        self.len == 0
    }

    pub fn is_negative(&self) -> bool {
        self.neg
    }

    pub fn is_one(&self) -> bool {
        self.len == 1 && self.limbs[0] == 1 && !self.neg
    }

    pub fn trailing_zeros(&self) -> u32 {
        let mut zeros = 0;
        for limb in self.magnitude() {
            if *limb == 0 {
                zeros += 64;
            } else {
                return zeros + limb.trailing_zeros();
            }
        }
        zeros
    }

    pub fn neg(&self) -> Wide {
        let mut out = *self;
        out.neg = !out.neg && out.len > 0;
        out
    }

    pub fn abs(&self) -> Wide {
        let mut out = *self;
        out.neg = false;
        out
    }

    // ---- arithmetic -----------------------------------------------------------------------------------------------

    /// `self * other`, or `None` when the product may need more than [`LIMBS`] limbs.
    pub fn mul(&self, other: &Wide) -> Option<Wide> {
        if self.len == 0 || other.len == 0 {
            return Some(Wide::ZERO);
        }
        let (la, lb) = (self.len as usize, other.len as usize);
        if la + lb > LIMBS {
            return None;
        }
        let mut limbs = [0u64; LIMBS];
        mul_limbs(&self.limbs[..la], &other.limbs[..lb], &mut limbs);
        let len = trimmed(&limbs[..la + lb]);
        Some(Wide { limbs, len: len as u8, neg: self.neg != other.neg })
    }

    /// `self >>= bits` on the magnitude (the sign is kept; a zero result is zero).
    pub fn shr_assign(&mut self, bits: u32) {
        if self.len == 0 || bits == 0 {
            return;
        }
        let (limb_shift, bit_shift) = ((bits / 64) as usize, bits % 64);
        let len = self.len as usize;
        if limb_shift >= len {
            *self = Wide::ZERO;
            return;
        }
        for index in 0..len - limb_shift {
            let low = self.limbs[index + limb_shift] >> bit_shift;
            let high = if bit_shift > 0 && index + limb_shift + 1 < len { self.limbs[index + limb_shift + 1] << (64 - bit_shift) } else { 0 };
            self.limbs[index] = low | high;
        }
        for index in len - limb_shift..len {
            self.limbs[index] = 0;
        }
        self.len = trimmed(&self.limbs[..len - limb_shift]) as u8;
        if self.len == 0 {
            self.neg = false;
        }
    }

    /// The exact quotient `self / divisor` (signed, truncating nothing: the caller guarantees `divisor | self`, `divisor != 0`).
    /// Hensel division: no quotient estimation, one multiply-subtract pass per quotient limb.
    pub fn div_exact(&self, divisor: &Wide) -> Wide {
        debug_assert!(divisor.len > 0);
        if self.len == 0 {
            return Wide::ZERO;
        }
        let shift = divisor.trailing_zeros();
        let mut remainder = *self;
        remainder.neg = false;
        remainder.shr_assign(shift);
        let mut odd = *divisor;
        odd.neg = false;
        odd.shr_assign(shift);
        let (n, d) = (remainder.len as usize, odd.len as usize);
        debug_assert!(n >= d, "a divisor longer than the dividend divides only zero");
        if n < d {
            return Wide::ZERO;
        }
        let inverse = inverse_mod_2_64(odd.limbs[0]);
        let mut quotient = [0u64; LIMBS];
        let count = n - d + 1;
        for index in 0..count {
            let digit = remainder.limbs[index].wrapping_mul(inverse);
            quotient[index] = digit;
            if digit == 0 {
                continue;
            }
            // remainder[index..] -= digit * odd
            let mut carry = 0u128;
            for column in 0..d {
                let product = digit as u128 * odd.limbs[column] as u128 + carry;
                let (value, borrow) = remainder.limbs[index + column].overflowing_sub(product as u64);
                remainder.limbs[index + column] = value;
                carry = (product >> 64) + borrow as u128;
            }
            let mut column = index + d;
            while carry != 0 && column < n {
                let (value, borrow) = remainder.limbs[column].overflowing_sub(carry as u64);
                remainder.limbs[column] = value;
                carry = (carry >> 64) + borrow as u128;
                column += 1;
            }
        }
        debug_assert!(remainder.limbs[..n].iter().all(|limb| *limb == 0), "the division was not exact");
        let len = trimmed(&quotient[..count]);
        Wide { limbs: quotient, len: len as u8, neg: self.neg != divisor.neg && len > 0 }
    }

    /// The exact quotient `self / divisor` for a non-zero word that divides `self` (signed like `self`): Hensel division, one multiplication
    /// and one subtraction per limb.
    pub fn div_exact_u64(&self, divisor: u64) -> Wide {
        debug_assert!(divisor != 0);
        let shift = divisor.trailing_zeros();
        let odd = divisor >> shift;
        let mut dividend = *self;
        dividend.shr_assign(shift);
        let n = dividend.len as usize;
        let inverse = inverse_mod_2_64(odd);
        let mut quotient = [0u64; LIMBS];
        let mut borrow = 0u64;
        for index in 0..n {
            let (difference, underflow) = dividend.limbs[index].overflowing_sub(borrow);
            let digit = difference.wrapping_mul(inverse);
            quotient[index] = digit;
            borrow = ((digit as u128 * odd as u128) >> 64) as u64 + underflow as u64;
        }
        debug_assert!(borrow == 0, "the division was not exact");
        let len = trimmed(&quotient[..n]);
        Wide { limbs: quotient, len: len as u8, neg: self.neg && len > 0 }
    }

    /// `gcd(|self|, |other|)`; `gcd(0, 0) = 0`. Operands up to two limbs stay on the stack; wider ones go to `dashu-int`.
    pub fn gcd(&self, other: &Wide) -> Wide {
        if self.len == 0 {
            return other.abs();
        }
        if other.len == 0 {
            return self.abs();
        }
        let (small, large) = if self.len <= other.len { (self, other) } else { (other, self) };
        if small.len == 1 {
            let divisor = small.limbs[0];
            let remainder = if large.len == 1 { large.limbs[0] % divisor } else { Divisor::new(divisor).rem(large.magnitude()) };
            return Wide::from_u64(gcd_u64(divisor, remainder));
        }
        if large.len <= 2 {
            let to_u128 = |wide: &Wide| wide.limbs[0] as u128 | (wide.limbs[1] as u128) << 64;
            return Wide::from_u128(gcd_u128(to_u128(small), to_u128(large)));
        }
        let common = small.to_ubig().gcd(&large.to_ubig());
        Wide::from_ubig(&common).expect("a divisor fits where its operand fits")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        /// A limb biased towards the edges that break carries: zero, one, all ones, powers of two, near-max.
        fn limb(&mut self) -> u64 {
            match self.next() % 8 {
                0 => 0,
                1 => 1,
                2 => u64::MAX,
                3 => u64::MAX - (self.next() % 4),
                4 => 1u64 << (self.next() % 64),
                5 => (1u64 << (self.next() % 64)).wrapping_sub(1),
                _ => self.next(),
            }
        }

        fn wide(&mut self, max_limbs: usize) -> Wide {
            let len = (self.next() as usize) % (max_limbs + 1);
            let words: Vec<u64> = (0..len).map(|_| self.limb()).collect();
            Wide::from_words(self.next() % 2 == 0, &words).unwrap()
        }
    }

    fn big(value: &Wide) -> IBig {
        value.to_ibig()
    }

    fn signed(value: i64) -> Wide {
        Wide::from_words(value < 0, &[value.unsigned_abs()]).unwrap()
    }

    #[test]
    fn conversions_round_trip_across_limb_boundaries() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        for _ in 0..20000 {
            let value = rng.wide(LIMBS);
            let back = Wide::from_ibig(&value.to_ibig()).unwrap();
            assert_eq!(back, value);
            assert_eq!(value.to_ubig(), UBig::from_words(value.magnitude()));
            assert_eq!(value.is_zero(), value.to_ibig().is_zero());
        }
        // one limb too many is refused, not truncated
        let words = vec![1u64; LIMBS + 1];
        assert!(Wide::from_words(false, &words).is_none());
        assert!(Wide::from_ibig(&IBig::from_sign_words(Sign::Negative, &words)).is_none());
        // leading zero words are tolerated
        let padded = Wide::from_words(true, &[5, 0, 0]).unwrap();
        assert_eq!(big(&padded), IBig::from(-5));
        assert!(Wide::from_words(true, &[0, 0]).unwrap().is_zero());
        assert!(!Wide::from_words(true, &[0]).unwrap().is_negative());
        assert_eq!(Wide::from_u128((1u128 << 64) + 5).magnitude(), &[5, 1]);
        assert!(Wide::from_u64(1).is_one() && !signed(-1).is_one() && !Wide::ZERO.is_one());
    }

    #[test]
    fn multiplication_matches_dashu_and_refuses_overflow() {
        let mut rng = Rng(0x1234_5678_9abc_def1);
        let mut refused = 0;
        for _ in 0..30000 {
            let (a, b) = (rng.wide(6), rng.wide(6));
            match a.mul(&b) {
                Some(product) => assert_eq!(big(&product), big(&a) * big(&b), "{a:?} * {b:?}"),
                None => {
                    refused += 1;
                    assert!(a.limb_count() + b.limb_count() > LIMBS);
                }
            }
        }
        assert!(refused > 0, "the refusal path must be exercised");
        let max = Wide::from_words(false, &[u64::MAX; LIMBS / 2]).unwrap();
        let square = max.mul(&max).unwrap();
        assert_eq!(big(&square), big(&max) * big(&max));
        // LIMBS + 1 limbs are refused even when the product would fit
        let full = Wide::from_words(false, &[u64::MAX; LIMBS]).unwrap();
        assert!(full.mul(&Wide::from_u64(1)).is_none());
        assert_eq!(big(&full.neg()), -big(&full));
        assert_eq!(big(&full.abs()), big(&full));
        assert!(Wide::ZERO.neg().is_zero() && !Wide::ZERO.neg().is_negative());
    }

    #[test]
    fn shifts_and_bit_facts_match_dashu() {
        let mut rng = Rng(0x0bad_cafe_dead_beef);
        for _ in 0..20000 {
            let value = rng.wide(LIMBS);
            let bits = (rng.next() % (64 * LIMBS as u64 + 70)) as u32;
            let mut shifted = value;
            shifted.shr_assign(bits);
            let magnitude = value.to_ubig() >> bits as usize;
            assert_eq!(shifted.to_ubig(), magnitude);
            assert_eq!(shifted.is_negative(), value.is_negative() && !magnitude.is_zero());
            if !value.is_zero() {
                assert_eq!(Some(value.trailing_zeros() as usize), value.to_ubig().trailing_zeros());
            }
        }
    }

    #[test]
    fn the_invariant_divisor_gives_the_remainder_the_hardware_gives() {
        let mut rng = Rng(0x7e57_ab1e_0dd5_eed5);
        for _ in 0..40000 {
            let value = rng.wide(LIMBS);
            let word = match rng.next() % 5 {
                0 => rng.limb() | 1,
                1 => (rng.next() >> (rng.next() % 60)) | 1,
                2 => 1u64 << (rng.next() % 64),
                3 => u64::MAX - (rng.next() % 3),
                _ => rng.next() | (1 << 63),
            }
            .max(1);
            let divisor = Divisor::new(word);
            let expected = value.to_ubig() % UBig::from(word);
            assert_eq!(UBig::from(divisor.rem(value.magnitude())), expected, "{value:?} mod {word}");
        }
        assert_eq!(Divisor::new(1).rem(&[u64::MAX, u64::MAX, 7]), 0);
        assert_eq!(Divisor::new(10).rem(&[]), 0);
        assert_eq!(Divisor::new(u64::MAX).rem(&[u64::MAX]), 0);
        assert_eq!(Divisor::new(3).rem(&[1, 1]), (((1u128 << 64) + 1) % 3) as u64);
    }

    #[test]
    fn exact_division_by_a_word_recovers_the_factor() {
        let mut rng = Rng(0x0ddc_0ffe_e0dd_f00d);
        for _ in 0..40000 {
            let quotient = rng.wide(LIMBS - 1);
            let word = match rng.next() % 4 {
                0 => rng.next(),
                1 => (rng.next() >> (rng.next() % 60)).max(1),
                2 => (1u64 << (rng.next() % 63)) | (rng.next() % 16),
                _ => rng.limb(),
            }
            .max(1);
            let Some(dividend) = quotient.mul(&Wide::from_u64(word)) else { continue };
            assert_eq!(dividend.div_exact_u64(word), quotient, "{dividend:?} / {word}");
        }
        assert!(Wide::ZERO.div_exact_u64(7).is_zero());
        assert_eq!(signed(-48).div_exact_u64(16), signed(-3));
    }

    #[test]
    fn exact_division_recovers_the_factor() {
        let mut rng = Rng(0x5eed_1234_5678_9abc);
        for _ in 0..30000 {
            let divisor = rng.wide(4);
            if divisor.is_zero() {
                continue;
            }
            let quotient = rng.wide(5);
            let Some(dividend) = divisor.mul(&quotient) else { continue };
            assert_eq!(dividend.div_exact(&divisor), quotient, "{dividend:?} / {divisor:?}");
        }
        // even divisors, powers of two, one, minus one, zero dividend
        assert_eq!(Wide::from_u64(48).div_exact(&Wide::from_u64(16)), Wide::from_u64(3));
        assert_eq!(big(&signed(-48).div_exact(&signed(-16))), IBig::from(3));
        assert_eq!(big(&signed(48).div_exact(&signed(-1))), IBig::from(-48));
        assert!(Wide::ZERO.div_exact(&Wide::from_u64(7)).is_zero());
        let power = Wide::from_words(false, &[0, 0, 1]).unwrap();
        assert_eq!(power.div_exact(&power), Wide::from_u64(1));
    }

    #[test]
    fn gcd_matches_dashu_for_every_size_mix() {
        let mut rng = Rng(0xc0ff_ee00_1234_5678);
        for _ in 0..30000 {
            let (a, b) = (rng.wide(5), rng.wide(5));
            let expected = crate::num::gcd_signed(&big(&a), &big(&b));
            let found = a.gcd(&b);
            assert_eq!(found.to_ubig(), expected, "gcd({a:?}, {b:?})");
            assert!(!found.is_negative());
            // a shared factor the generator does not otherwise produce
            let common = rng.wide(2).abs();
            if let (Some(x), Some(y)) = (a.mul(&common), b.mul(&common)) {
                assert_eq!(x.gcd(&y).to_ubig(), crate::num::gcd_signed(&big(&x), &big(&y)));
            }
        }
        assert!(Wide::ZERO.gcd(&Wide::ZERO).is_zero());
        assert_eq!(signed(-12).gcd(&Wide::ZERO), Wide::from_u64(12));
        assert_eq!(Wide::ZERO.gcd(&signed(-12)), Wide::from_u64(12));
        assert_eq!(signed(-12).gcd(&signed(18)), Wide::from_u64(6));
        for _ in 0..2000 {
            let (a, b) = (rng.next(), rng.next());
            assert_eq!(UBig::from(gcd_u64(a, b)), crate::num::gcd(&UBig::from(a), &UBig::from(b)));
            let (c, d) = (((a as u128) << 64) | b as u128, ((b as u128) << 62) | a as u128);
            assert_eq!(UBig::from(gcd_u128(c, d)), crate::num::gcd(&UBig::from(c), &UBig::from(d)));
        }
    }
}
