//! The accumulator of the stack road: integer sums keyed by radicand, built from products of limb slices.
//!
//! A slot keeps the sum of its positive products and the sum of its negative ones apart, as unsigned limb arrays, so adding a product is a
//! multiplication into a small temporary and an unsigned add with carry into one of the two arrays, with no comparison and no sign
//! logic; the difference is taken once per slot, in [`FxAcc::finish`]. The slots live in a scratch that is reused from call to call
//! (a slot is cleared when it is claimed, so nothing is zeroed in bulk).
//!
//! Everything answers `false` / `None` when a product or a sum would not fit [`SLOT`] limbs; the caller runs the `dashu-int` road then.

use std::cmp::Ordering;

use crate::wide::Wide;

/// Limbs of one sum (a product of `a` and `b` limbs needs `a + b`, and sums of them one more).
pub const SLOT: usize = 12;
/// Distinct radicands one accumulation holds.
pub const SLOTS: usize = 32;

/// The widest product the unrolled kernel takes: a left numerator of at most [`NARROW`] limbs times a right one of at most [`WIDE`].
const NARROW: usize = 2;
const WIDE: usize = 6;

pub struct FxAcc {
    n: usize,
    key: [u128; SLOTS],
    pos: [[u64; SLOT]; SLOTS],
    neg: [[u64; SLOT]; SLOTS],
    /// Limbs in use of each array (a bound: everything above is zero).
    pos_top: [u8; SLOTS],
    neg_top: [u8; SLOTS],
}

impl FxAcc {
    pub fn new() -> Box<FxAcc> {
        Box::new(FxAcc { n: 0, key: [0; SLOTS], pos: [[0; SLOT]; SLOTS], neg: [[0; SLOT]; SLOTS], pos_top: [0; SLOTS], neg_top: [0; SLOTS] })
    }

    /// Starts a new accumulation.
    pub fn clear(&mut self) {
        self.n = 0;
    }

    /// The slot of a radicand, claimed (and cleared) if it is new; `None` when every slot is taken.
    #[inline]
    fn slot(&mut self, radicand: u128) -> Option<usize> {
        for index in 0..self.n {
            if self.key[index] == radicand {
                return Some(index);
            }
        }
        if self.n == SLOTS {
            return None;
        }
        let index = self.n;
        self.n += 1;
        self.key[index] = radicand;
        self.pos[index] = [0; SLOT];
        self.neg[index] = [0; SLOT];
        self.pos_top[index] = 0;
        self.neg_top[index] = 0;
        Some(index)
    }

    /// `sum[radicand] += (-1)^negative * a * b * scale` for magnitudes `a`, `b` (little-endian limbs, no leading zeros) and a word `scale`;
    /// `false` when it does not fit.
    #[inline]
    pub fn add_product(&mut self, radicand: u128, a: &[u64], b: &[u64], negative: bool, scale: u64) -> bool {
        if a.is_empty() || b.is_empty() {
            return true;
        }
        let Some(slot) = self.slot(radicand) else { return false };
        let (target, top) = if negative { (&mut self.neg[slot], &mut self.neg_top[slot]) } else { (&mut self.pos[slot], &mut self.pos_top[slot]) };
        let (la, lb) = (a.len(), b.len());
        if la <= NARROW && lb <= WIDE {
            // the unrolled kernel: both operands padded with zeros to the class, the product is at most NARROW + WIDE limbs
            let mut left = [0u64; NARROW];
            left[..la].copy_from_slice(a);
            let mut right = [0u64; WIDE];
            right[..lb].copy_from_slice(b);
            let mut product = [0u64; NARROW + WIDE + 1];
            for i in 0..NARROW {
                let mut carry = 0u64;
                for j in 0..WIDE {
                    let total = left[i] as u128 * right[j] as u128 + product[i + j] as u128 + carry as u128;
                    product[i + j] = total as u64;
                    carry = (total >> 64) as u64;
                }
                product[i + WIDE] = carry;
            }
            if scale != 1 {
                let mut carry = 0u64;
                for limb in product.iter_mut() {
                    let total = *limb as u128 * scale as u128 + carry as u128;
                    *limb = total as u64;
                    carry = (total >> 64) as u64;
                }
                if carry != 0 {
                    return false;
                }
            }
            return add_limbs(target, top, &product);
        }
        if la + lb > SLOT - 2 {
            return false;
        }
        let mut product = [0u64; SLOT];
        for (i, &ai) in a.iter().enumerate() {
            let mut carry = 0u64;
            for (j, &bj) in b.iter().enumerate() {
                let total = ai as u128 * bj as u128 + product[i + j] as u128 + carry as u128;
                product[i + j] = total as u64;
                carry = (total >> 64) as u64;
            }
            product[i + lb] = carry;
        }
        if scale != 1 {
            let mut carry = 0u64;
            for limb in product.iter_mut() {
                let total = *limb as u128 * scale as u128 + carry as u128;
                *limb = total as u64;
                carry = (total >> 64) as u64;
            }
            if carry != 0 {
                return false;
            }
        }
        add_limbs(target, top, &product)
    }

    /// The non-zero sums by ascending radicand, written into `keys` / `values`; `None` when more survive than they hold, or one does not
    /// fit a [`Wide`].
    pub fn finish(&self, keys: &mut [u128], values: &mut [Wide]) -> Option<usize> {
        let mut order = [0u8; SLOTS];
        let mut count = 0;
        for slot in 0..self.n {
            let (pos, neg) = (trim(&self.pos[slot][..self.pos_top[slot] as usize]), trim(&self.neg[slot][..self.neg_top[slot] as usize]));
            if cmp_limbs(pos, neg) == Ordering::Equal {
                continue;
            }
            let mut at = count;
            while at > 0 && self.key[order[at - 1] as usize] > self.key[slot] {
                order[at] = order[at - 1];
                at -= 1;
            }
            order[at] = slot as u8;
            count += 1;
        }
        if count > keys.len() {
            return None;
        }
        for (target, slot) in order[..count].iter().enumerate() {
            keys[target] = self.key[*slot as usize];
            values[target] = self.difference(*slot as usize)?;
        }
        Some(count)
    }

    /// `positives - negatives` of a slot as a [`Wide`]; `None` when it does not fit a `Wide`.
    fn difference(&self, slot: usize) -> Option<Wide> {
        let (pos, neg) = (&self.pos[slot][..self.pos_top[slot] as usize], &self.neg[slot][..self.neg_top[slot] as usize]);
        let (pos, neg) = (trim(pos), trim(neg));
        match cmp_limbs(pos, neg) {
            Ordering::Equal => Some(Wide::ZERO),
            Ordering::Greater => magnitude_difference(pos, neg, false),
            Ordering::Less => magnitude_difference(neg, pos, true),
        }
    }
}

fn trim(limbs: &[u64]) -> &[u64] {
    let mut len = limbs.len();
    while len > 0 && limbs[len - 1] == 0 {
        len -= 1;
    }
    &limbs[..len]
}

fn cmp_limbs(left: &[u64], right: &[u64]) -> Ordering {
    if left.len() != right.len() {
        return left.len().cmp(&right.len());
    }
    for index in (0..left.len()).rev() {
        if left[index] != right[index] {
            return left[index].cmp(&right[index]);
        }
    }
    Ordering::Equal
}

/// `larger - smaller` for `larger >= smaller`, as a signed [`Wide`].
fn magnitude_difference(larger: &[u64], smaller: &[u64], negative: bool) -> Option<Wide> {
    let mut limbs = [0u64; SLOT];
    let mut borrow = false;
    for index in 0..larger.len() {
        let rhs = smaller.get(index).copied().unwrap_or(0);
        let (partial, first) = larger[index].overflowing_sub(rhs);
        let (total, second) = partial.overflowing_sub(borrow as u64);
        limbs[index] = total;
        borrow = first || second;
    }
    Wide::from_words(negative, &limbs[..larger.len()])
}

/// `target += addend` where the addend is a [`NARROW`] + [`WIDE`] + 1 limb product (or a `SLOT` one): unsigned add with carry, the top
/// bound raised to what was touched. `false` on a carry out of the last limb.
#[inline]
fn add_limbs(target: &mut [u64; SLOT], top: &mut u8, addend: &[u64]) -> bool {
    let width = addend.len();
    let mut carry = false;
    for index in 0..width {
        let (partial, first) = target[index].overflowing_add(addend[index]);
        let (total, second) = partial.overflowing_add(carry as u64);
        target[index] = total;
        carry = first || second;
    }
    let mut index = width;
    while carry {
        if index == SLOT {
            return false;
        }
        let (total, overflow) = target[index].overflowing_add(1);
        target[index] = total;
        carry = overflow;
        index += 1;
    }
    *top = (*top).max(index.max(width) as u8);
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::num::{IBig, Sign};

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn limbs(&mut self, max: usize) -> Vec<u64> {
            let len = 1 + (self.next() as usize) % max;
            let mut out: Vec<u64> = (0..len)
                .map(|_| match self.next() % 6 {
                    0 => u64::MAX,
                    1 => 1,
                    2 => 0,
                    _ => self.next(),
                })
                .collect();
            if *out.last().unwrap() == 0 {
                *out.last_mut().unwrap() = 7;
            }
            out
        }
    }

    fn big(negative: bool, limbs: &[u64]) -> IBig {
        IBig::from_sign_words(if negative { Sign::Negative } else { Sign::Positive }, limbs)
    }

    #[test]
    fn sums_of_products_equal_dashu_per_radicand() {
        let mut rng = Rng(0x0dd_ba11_f00d_cafe);
        let mut acc = FxAcc::new();
        let mut checked = 0;
        for _ in 0..3000 {
            acc.clear();
            let mut expected: std::collections::BTreeMap<u128, IBig> = std::collections::BTreeMap::new();
            let mut fits = true;
            for _ in 0..(1 + rng.next() % 14) {
                let radicand = 2 + (rng.next() % 9) as u128;
                let narrow = if rng.next() % 4 == 0 { 6 } else { 2 };
                let (a, b) = (rng.limbs(narrow), rng.limbs(6));
                let negative = rng.next() % 2 == 0;
                let scale = [1u64, 1, 2, 6, u64::MAX][(rng.next() % 5) as usize];
                if !acc.add_product(radicand, &a, &b, negative, scale) {
                    fits = false;
                    break;
                }
                *expected.entry(radicand).or_insert(IBig::ZERO) += big(negative, &a) * big(false, &b) * IBig::from(scale);
            }
            if !fits {
                continue;
            }
            let mut keys = [0u128; 16];
            let mut values = [Wide::ZERO; 16];
            let Some(count) = acc.finish(&mut keys, &mut values) else { continue }; // a sum wider than a `Wide` is refused, not truncated
            let live: Vec<(u128, IBig)> = expected.into_iter().filter(|(_, value)| !value.is_zero()).collect();
            assert_eq!(count, live.len());
            for (index, (radicand, value)) in live.iter().enumerate() {
                assert_eq!(keys[index], *radicand);
                assert_eq!(values[index].to_ibig(), *value);
            }
            checked += 1;
        }
        assert!(checked > 2000, "the stack road must carry most random sums ({checked})");
    }

    #[test]
    fn a_product_that_does_not_fit_is_refused() {
        let mut acc = FxAcc::new();
        acc.clear();
        let wide = vec![u64::MAX; 6];
        assert!(!acc.add_product(2, &wide, &wide, false, 1), "twelve limbs leave no room for the carries of a sum");
        // a carry that runs out of the last limb is refused too
        acc.clear();
        let almost = vec![u64::MAX; 5];
        assert!(acc.add_product(2, &almost, &almost, false, 1));
        for _ in 0..4 {
            let _ = acc.add_product(2, &almost, &almost, false, 1);
        }
        // more distinct radicands than slots
        acc.clear();
        for radicand in 0..SLOTS as u128 {
            assert!(acc.add_product(radicand + 2, &[3], &[5], false, 1));
        }
        assert!(!acc.add_product(1000, &[3], &[5], false, 1));
    }
}
