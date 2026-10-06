//! Fixed-limb Montgomery arithmetic for odd moduli of `K` machine words (`K` = 1..=`MAX_LIMBS`).
//!
//! Speed lever of the Brent orbit and of the Miller-Rabin exponentiation. It never changes a decision: the
//! budget is charged by the callers per Python's rules, and every value that influences control flow
//! (a gcd, a comparison against `1` / `n - 1`) is invariant under the Montgomery factor `R = 2^(64 K)`
//! because `gcd(R, n) = 1`.
//!
//! All arrays are little-endian words. Residues are kept fully reduced (`< n`).

use dashu_base::BitTest;
use dashu_int::UBig;

/// Largest supported modulus: 8 words = 512 bits. Wider moduli use the plain-integer fallback.
pub const MAX_LIMBS: usize = 8;

/// `a >= b` for equal-length little-endian arrays.
#[inline(always)]
fn ge<const K: usize>(a: &[u64; K], b: &[u64; K]) -> bool {
    let mut i = K;
    while i > 0 {
        i -= 1;
        if a[i] != b[i] {
            return a[i] > b[i];
        }
    }
    true
}

/// `a - b` with the final borrow.
#[inline(always)]
#[allow(clippy::needless_range_loop)]
fn sub_with_borrow<const K: usize>(a: &[u64; K], b: &[u64; K]) -> ([u64; K], bool) {
    let mut out = [0u64; K];
    let mut borrow = false;
    for i in 0..K {
        let (d1, b1) = a[i].overflowing_sub(b[i]);
        let (d2, b2) = d1.overflowing_sub(u64::from(borrow));
        out[i] = d2;
        borrow = b1 | b2;
    }
    (out, borrow)
}

/// `a + b` with the final carry.
#[inline(always)]
#[allow(clippy::needless_range_loop)]
fn add_with_carry<const K: usize>(a: &[u64; K], b: &[u64; K]) -> ([u64; K], bool) {
    let mut out = [0u64; K];
    let mut carry = false;
    for i in 0..K {
        let (s1, c1) = a[i].overflowing_add(b[i]);
        let (s2, c2) = s1.overflowing_add(u64::from(carry));
        out[i] = s2;
        carry = c1 | c2;
    }
    (out, carry)
}

/// Words of `value` as a fixed array (callers guarantee `value < 2^(64 K)`).
pub fn to_array<const K: usize>(value: &UBig) -> [u64; K] {
    let mut out = [0u64; K];
    for (slot, word) in out.iter_mut().zip(value.as_words()) {
        *slot = *word;
    }
    out
}

/// The array as an integer.
pub fn from_array<const K: usize>(value: &[u64; K]) -> UBig {
    UBig::from_words(value)
}

/// Montgomery context of one odd modulus `n` with `K` words (`n >= 3`, top word non-zero).
#[derive(Clone)]
pub struct Mont<const K: usize> {
    pub n: [u64; K],
    /// `-n^{-1} mod 2^64`.
    ninv: u64,
    /// `R mod n`: Montgomery form of 1.
    pub one: [u64; K],
    /// `R^2 mod n`.
    r2: [u64; K],
}

impl<const K: usize> Mont<K> {
    /// `None` unless `n` is odd, `n >= 3` and occupies exactly `K` words.
    pub fn new(n: &UBig) -> Option<Mont<K>> {
        let words = n.as_words();
        if words.len() != K || K == 0 || K > MAX_LIMBS || words[0] & 1 == 0 || *n < UBig::from(3u8) {
            return None;
        }
        let n0 = words[0];
        let mut inverse = n0;
        for _ in 0..5 {
            inverse = inverse.wrapping_mul(2u64.wrapping_sub(n0.wrapping_mul(inverse)));
        }
        let one = (UBig::ONE << (64 * K)) % n;
        let r2 = (UBig::ONE << (128 * K)) % n;
        Some(Mont { n: to_array::<K>(n), ninv: inverse.wrapping_neg(), one: to_array::<K>(&one), r2: to_array::<K>(&r2) })
    }

    /// Montgomery product `a * b / R mod n` (CIOS), result fully reduced.
    #[inline(always)]
    #[allow(clippy::needless_range_loop)]
    pub fn mul(&self, a: &[u64; K], b: &[u64; K]) -> [u64; K] {
        let n = &self.n;
        let mut t = [0u64; K];
        let mut t_hi = 0u64;
        for i in 0..K {
            let bi = u128::from(b[i]);
            let mut carry = 0u64;
            for j in 0..K {
                let sum = u128::from(t[j]) + u128::from(a[j]) * bi + u128::from(carry);
                t[j] = sum as u64;
                carry = (sum >> 64) as u64;
            }
            let sum = u128::from(t_hi) + u128::from(carry);
            t_hi = sum as u64;
            let t_over = (sum >> 64) as u64;
            let m = u128::from(t[0].wrapping_mul(self.ninv));
            let sum = u128::from(t[0]) + m * u128::from(n[0]);
            let mut carry = (sum >> 64) as u64;
            for j in 1..K {
                let sum = u128::from(t[j]) + m * u128::from(n[j]) + u128::from(carry);
                t[j - 1] = sum as u64;
                carry = (sum >> 64) as u64;
            }
            let sum = u128::from(t_hi) + u128::from(carry);
            t[K - 1] = sum as u64;
            t_hi = t_over + ((sum >> 64) as u64);
        }
        if t_hi != 0 || ge(&t, n) {
            sub_with_borrow(&t, n).0
        } else {
            t
        }
    }

    /// `a + b mod n` for residues.
    #[inline(always)]
    pub fn add(&self, a: &[u64; K], b: &[u64; K]) -> [u64; K] {
        let (sum, carry) = add_with_carry(a, b);
        if carry || ge(&sum, &self.n) {
            sub_with_borrow(&sum, &self.n).0
        } else {
            sum
        }
    }

    /// `a - b mod n` for residues.
    #[inline(always)]
    pub fn sub(&self, a: &[u64; K], b: &[u64; K]) -> [u64; K] {
        let (difference, borrow) = sub_with_borrow(a, b);
        if borrow {
            add_with_carry(&difference, &self.n).0
        } else {
            difference
        }
    }

    /// Montgomery form of `value mod n`.
    pub fn to_mont(&self, value: &UBig) -> [u64; K] {
        let n = from_array(&self.n);
        let reduced = if *value < n { value.clone() } else { value % &n };
        self.mul(&to_array::<K>(&reduced), &self.r2)
    }

    /// Plain residue of a Montgomery-form element.
    #[cfg(test)]
    #[allow(clippy::wrong_self_convention)]
    pub fn to_plain(&self, value: &[u64; K]) -> UBig {
        let mut plain = [0u64; K];
        plain[0] = 1;
        from_array(&self.mul(value, &plain))
    }

    /// `base^exponent` in Montgomery form for `exponent >= 1` (left-to-right binary).
    pub fn pow(&self, base: &[u64; K], exponent: &UBig) -> [u64; K] {
        let bits = exponent.bit_len();
        let words = exponent.as_words();
        let mut result = *base;
        let mut position = bits.saturating_sub(1);
        while position > 0 {
            position -= 1;
            result = self.mul(&result, &result);
            if (words[position / 64] >> (position % 64)) & 1 == 1 {
                result = self.mul(&result, base);
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lcg(state: &mut u64) -> u64 {
        *state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        *state
    }

    fn random_odd(words: usize, state: &mut u64) -> UBig {
        let mut limbs: Vec<u64> = (0..words).map(|_| lcg(state)).collect();
        limbs[0] |= 1;
        if limbs[words - 1] == 0 {
            limbs[words - 1] = 1;
        }
        UBig::from_words(&limbs)
    }

    fn check<const K: usize>() {
        let mut state = 0x1234_5678_9abc_def0u64 ^ (K as u64);
        for _ in 0..40 {
            let n = random_odd(K, &mut state);
            if n < UBig::from(3u8) {
                continue;
            }
            let ctx = Mont::<K>::new(&n).expect("odd modulus");
            let a = random_odd(K, &mut state) % &n;
            let b = random_odd(K, &mut state) % &n;
            let am = ctx.to_mont(&a);
            let bm = ctx.to_mont(&b);
            assert_eq!(ctx.to_plain(&am), a);
            assert_eq!(ctx.to_plain(&ctx.mul(&am, &bm)), (&a * &b) % &n);
            assert_eq!(ctx.to_plain(&ctx.add(&am, &bm)), (&a + &b) % &n);
            assert_eq!(ctx.to_plain(&ctx.sub(&am, &bm)), (&a + &n - &b) % &n);
            let exponent = UBig::from(lcg(&mut state) | 1);
            let expected = {
                let mut result = UBig::ONE;
                let mut base = a.clone();
                let mut e = exponent.clone();
                while e > UBig::ZERO {
                    if &e % 2u8 == 1u8 {
                        result = (&result * &base) % &n;
                    }
                    base = (&base * &base) % &n;
                    e >>= 1usize;
                }
                result
            };
            assert_eq!(ctx.to_plain(&ctx.pow(&am, &exponent)), expected);
        }
    }

    #[test]
    fn montgomery_matches_plain_integers_for_every_width() {
        check::<1>();
        check::<2>();
        check::<3>();
        check::<4>();
        check::<5>();
        check::<6>();
        check::<7>();
        check::<8>();
    }

    #[test]
    fn full_width_top_bit_moduli_survive_the_carry_paths() {
        let n = (UBig::ONE << 256usize) - UBig::from(189u8);
        let ctx = Mont::<4>::new(&n).expect("odd");
        let a = &n - UBig::ONE;
        let am = ctx.to_mont(&a);
        assert_eq!(ctx.to_plain(&ctx.mul(&am, &am)), UBig::ONE);
        assert_eq!(ctx.to_plain(&ctx.add(&am, &am)), &n - UBig::from(2u8));
    }
}
