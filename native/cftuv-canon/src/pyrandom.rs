//! CPython's `random.Random(seed)` for non-negative big-integer seeds: MT19937 seeded through `init_by_array`,
//! `getrandbits`, `_randbelow_with_getrandbits` and `randrange(start, stop)`. Identical on CPython 3.11 and 3.13.
//!
//! Only the integer side of the module is reproduced; `random()`, `gauss` and friends are not used by the kernel.

use dashu_base::BitTest;
use dashu_int::UBig;

const N: usize = 624;
const M: usize = 397;
const MATRIX_A: u32 = 0x9908_b0df;
const UPPER_MASK: u32 = 0x8000_0000;
const LOWER_MASK: u32 = 0x7fff_ffff;

/// Refusals of the integer helpers; they mirror the `ValueError`s CPython raises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RandomError {
    /// `randrange(start, stop)` with `stop <= start`.
    EmptyRange,
    /// `_randbelow(0)`: returns 0 on CPython >= 3.12 and loops on older ones; the kernel never asks, so it is
    /// refused instead of guessed.
    ZeroBound,
}

/// The generator state: 624 words and the read index.
#[derive(Clone)]
pub struct PyRandom {
    mt: [u32; N],
    index: usize,
}

impl PyRandom {
    /// `random.Random(abs(seed))`: the key is the little-endian `u32` words of the magnitude, at least one word.
    pub fn from_seed(seed: &UBig) -> PyRandom {
        let mut key: Vec<u32> = Vec::with_capacity(seed.as_words().len() * 2);
        for word in seed.as_words() {
            key.push(*word as u32);
            key.push((*word >> 32) as u32);
        }
        while key.len() > 1 && key.last() == Some(&0) {
            key.pop();
        }
        if key.is_empty() {
            key.push(0);
        }
        PyRandom::from_key(&key)
    }

    /// `init_by_array(key)` of mt19937ar.c (`init_genrand(19650218)` first).
    pub fn from_key(key: &[u32]) -> PyRandom {
        let key: &[u32] = if key.is_empty() { &[0] } else { key };
        let mut generator = PyRandom { mt: [0; N], index: N };
        generator.init_genrand(19_650_218);
        let key_length = key.len();
        let (mut i, mut j) = (1usize, 0usize);
        let mut k = N.max(key_length);
        while k > 0 {
            let previous = generator.mt[i - 1];
            generator.mt[i] = (generator.mt[i] ^ (previous ^ (previous >> 30)).wrapping_mul(1_664_525))
                .wrapping_add(key[j])
                .wrapping_add(j as u32);
            i += 1;
            j += 1;
            if i >= N {
                generator.mt[0] = generator.mt[N - 1];
                i = 1;
            }
            if j >= key_length {
                j = 0;
            }
            k -= 1;
        }
        k = N - 1;
        while k > 0 {
            let previous = generator.mt[i - 1];
            generator.mt[i] = (generator.mt[i] ^ (previous ^ (previous >> 30)).wrapping_mul(1_566_083_941))
                .wrapping_sub(i as u32);
            i += 1;
            if i >= N {
                generator.mt[0] = generator.mt[N - 1];
                i = 1;
            }
            k -= 1;
        }
        generator.mt[0] = 0x8000_0000;
        generator.index = N;
        generator
    }

    fn init_genrand(&mut self, seed: u32) {
        self.mt[0] = seed;
        for i in 1..N {
            let previous = self.mt[i - 1];
            self.mt[i] = 1_812_433_253u32.wrapping_mul(previous ^ (previous >> 30)).wrapping_add(i as u32);
        }
        self.index = N;
    }

    fn regenerate(&mut self) {
        let mag = |y: u32| if y & 1 == 0 { 0 } else { MATRIX_A };
        for kk in 0..N - M {
            let y = (self.mt[kk] & UPPER_MASK) | (self.mt[kk + 1] & LOWER_MASK);
            self.mt[kk] = self.mt[kk + M] ^ (y >> 1) ^ mag(y);
        }
        for kk in N - M..N - 1 {
            let y = (self.mt[kk] & UPPER_MASK) | (self.mt[kk + 1] & LOWER_MASK);
            self.mt[kk] = self.mt[kk + M - N] ^ (y >> 1) ^ mag(y);
        }
        let y = (self.mt[N - 1] & UPPER_MASK) | (self.mt[0] & LOWER_MASK);
        self.mt[N - 1] = self.mt[M - 1] ^ (y >> 1) ^ mag(y);
        self.index = 0;
    }

    /// `genrand_uint32`.
    pub fn next_u32(&mut self) -> u32 {
        if self.index >= N {
            self.regenerate();
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^= y >> 18;
        y
    }

    /// `getrandbits(k)`: `k <= 32` takes the top `k` bits of one word; larger `k` fills little-endian words and
    /// drops the low bits of the last one.
    pub fn getrandbits(&mut self, k: u32) -> UBig {
        if k == 0 {
            return UBig::ZERO;
        }
        if k <= 32 {
            return UBig::from(self.next_u32() >> (32 - k));
        }
        let words = (k - 1) / 32 + 1;
        let mut remaining = k;
        let mut packed: Vec<u64> = Vec::with_capacity((words as usize).div_ceil(2));
        let mut pending: Option<u32> = None;
        for _ in 0..words {
            let mut word = self.next_u32();
            if remaining < 32 {
                word >>= 32 - remaining;
            }
            remaining = remaining.saturating_sub(32);
            match pending.take() {
                None => pending = Some(word),
                Some(low) => packed.push(u64::from(low) | (u64::from(word) << 32)),
            }
        }
        if let Some(low) = pending {
            packed.push(u64::from(low));
        }
        UBig::from_words(&packed)
    }

    /// `_randbelow_with_getrandbits(n)` for `n > 0`: `k = n.bit_length()`, resample while `r >= n`.
    pub fn randbelow(&mut self, bound: &UBig) -> Result<UBig, RandomError> {
        if *bound == UBig::ZERO {
            return Err(RandomError::ZeroBound);
        }
        let k = bound.bit_len() as u32;
        loop {
            let candidate = self.getrandbits(k);
            if candidate < *bound {
                return Ok(candidate);
            }
        }
    }

    /// `randrange(start, stop)` with the default step: `start + _randbelow(stop - start)`.
    pub fn randrange(&mut self, start: &UBig, stop: &UBig) -> Result<UBig, RandomError> {
        if stop <= start {
            return Err(RandomError::EmptyRange);
        }
        let width = stop - start;
        Ok(start + self.randbelow(&width)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seed_words_are_little_endian_u32_with_a_minimum_of_one_word() {
        let zero = PyRandom::from_seed(&UBig::ZERO);
        let explicit = PyRandom::from_key(&[0]);
        assert_eq!(zero.mt[..4], explicit.mt[..4]);
        let wide = PyRandom::from_seed(&((UBig::ONE << 64usize) + UBig::from(7u8)));
        assert_eq!(wide.mt[..4], PyRandom::from_key(&[7, 0, 1]).mt[..4]);
    }

    #[test]
    fn getrandbits_shifts_only_the_last_word() {
        let mut a = PyRandom::from_key(&[1, 2, 3]);
        let mut b = a.clone();
        let wide = a.getrandbits(65);
        let w0 = u64::from(b.next_u32());
        let w1 = u64::from(b.next_u32());
        let w2 = u64::from(b.next_u32() >> 31);
        assert_eq!(wide, UBig::from(w0 | (w1 << 32)) + (UBig::from(w2) << 64usize));
    }

    #[test]
    fn randrange_rejects_the_empty_range() {
        let mut generator = PyRandom::from_key(&[1]);
        assert_eq!(generator.randrange(&UBig::from(3u8), &UBig::from(3u8)), Err(RandomError::EmptyRange));
    }
}
