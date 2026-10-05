//! Products of sqrt-sum integer forms (`_radicand_products.py`).
//!
//! `sqrt(a) * sqrt(b) = g * sqrt(a*b / g^2)` with `g = gcd(a, b)`. For squarefree `a` and `b` the radicand
//! `a*b / g^2` is squarefree too, so the product needs no factorization. The pair `(a, b) -> (g, a*b/g^2)` is a
//! pure function, which makes the [`ProductMemo`] a cache with no effect on any answer or on any counted cost
//! (the Python table is a pure cache as well, and is private here).

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

use crate::num::{self, IBig, UBig};

/// `[(radicand, numerator)]`: the integer numerators of `sum a_m * sqrt(m) / L`.
pub type Items = Vec<(UBig, IBig)>;

/// Integer sums keyed by radicand, kept sorted by radicand (the order every Python caller sorts to).
#[derive(Debug, Default, Clone)]
pub struct Accumulator {
    keys: Vec<UBig>,
    values: Vec<IBig>,
}

impl Accumulator {
    pub fn new() -> Accumulator {
        Accumulator::default()
    }

    pub fn with_capacity(capacity: usize) -> Accumulator {
        Accumulator { keys: Vec::with_capacity(capacity), values: Vec::with_capacity(capacity) }
    }

    /// `merged[radicand] = merged.get(radicand, 0) + value`.
    pub fn add(&mut self, radicand: &UBig, value: IBig) {
        match self.keys.binary_search(radicand) {
            Ok(index) => self.values[index] += value,
            Err(index) => {
                self.keys.insert(index, radicand.clone());
                self.values.insert(index, value);
            }
        }
    }

    pub fn get(&self, radicand: &UBig) -> Option<&IBig> {
        self.keys.binary_search(radicand).ok().map(|index| &self.values[index])
    }

    pub fn len(&self) -> usize {
        self.keys.len()
    }

    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&UBig, &IBig)> {
        self.keys.iter().zip(self.values.iter())
    }

    /// The entries with a non-zero value, by radicand: `sorted((m, v) for m, v in merged.items() if v)`.
    pub fn into_nonzero_items(self) -> Items {
        self.keys.into_iter().zip(self.values).filter(|(_, value)| !value.is_zero()).collect()
    }
}

/// `(g, a*b/g^2)` for radicands `a`, `b` (both above one).
pub fn radicand_product(left: &UBig, right: &UBig) -> (UBig, UBig) {
    let common = num::gcd(left, right);
    let radicand = (left / &common) * (right / &common);
    (common, radicand)
}

#[derive(Default)]
struct IdentityHasher(u64);

impl Hasher for IdentityHasher {
    fn finish(&self) -> u64 {
        self.0
    }

    fn write(&mut self, _: &[u8]) {
        unreachable!("the memo only hashes u64 keys");
    }

    fn write_u64(&mut self, value: u64) {
        self.0 = value;
    }
}

struct MemoEntry {
    left: UBig,
    right: UBig,
    common: UBig,
    radicand: UBig,
}

fn word_hash(value: &UBig) -> u64 {
    let mut state = 0u64;
    for word in value.as_words() {
        state = (state.rotate_left(5) ^ word).wrapping_mul(0x517c_c1b7_2722_0a95);
    }
    state
}

/// Memory of radicand-pair products. Both orders of a pair share one entry (the hash is symmetric).
pub struct ProductMemo {
    table: HashMap<u64, Vec<MemoEntry>, BuildHasherDefault<IdentityHasher>>,
    entries: usize,
    limit: usize,
    enabled: bool,
}

impl ProductMemo {
    /// Same bound as the Python table (65536 keys, two per pair); the whole table is dropped on overflow.
    const DEFAULT_PAIRS: usize = 1 << 15;

    pub fn new() -> ProductMemo {
        ProductMemo { table: HashMap::default(), entries: 0, limit: ProductMemo::DEFAULT_PAIRS, enabled: true }
    }

    /// No memory at all: every pair is computed from scratch.
    pub fn disabled() -> ProductMemo {
        ProductMemo { table: HashMap::default(), entries: 0, limit: 0, enabled: false }
    }

    pub fn clear(&mut self) {
        self.table.clear();
        self.entries = 0;
    }

    pub fn len(&self) -> usize {
        self.entries
    }

    pub fn is_empty(&self) -> bool {
        self.entries == 0
    }

    /// `(g, a*b/g^2)` from the memory or computed and remembered.
    pub fn product(&mut self, left: &UBig, right: &UBig) -> (UBig, UBig) {
        if !self.enabled {
            return radicand_product(left, right);
        }
        let key = word_hash(left).wrapping_add(word_hash(right));
        if let Some(bucket) = self.table.get(&key) {
            for entry in bucket {
                if (entry.left == *left && entry.right == *right) || (entry.left == *right && entry.right == *left) {
                    return (entry.common.clone(), entry.radicand.clone());
                }
            }
        }
        let (common, radicand) = radicand_product(left, right);
        if self.entries >= self.limit {
            self.clear();
        }
        self.table.entry(key).or_default().push(MemoEntry {
            left: left.clone(),
            right: right.clone(),
            common: common.clone(),
            radicand: radicand.clone(),
        });
        self.entries += 1;
        (common, radicand)
    }
}

impl Default for ProductMemo {
    fn default() -> ProductMemo {
        ProductMemo::new()
    }
}

/// Adds `weight * (sum a_m sqrt(m)) * (sum b_m sqrt(m))` to `merged` (integers, no normalisation).
///
/// The weight is multiplied into the left numerator once per left term, not once per pair.
pub fn accumulate_products(merged: &mut Accumulator, left: &[(UBig, IBig)], right: &[(UBig, IBig)], weight: &IBig, memo: &mut ProductMemo) {
    let weighted = !weight.is_one();
    for (left_radicand, left_numerator) in left {
        let left_numerator = if weighted { left_numerator * weight } else { left_numerator.clone() };
        if left_radicand.is_one() {
            for (right_radicand, right_numerator) in right {
                merged.add(right_radicand, &left_numerator * right_numerator);
            }
            continue;
        }
        for (right_radicand, right_numerator) in right {
            if right_radicand.is_one() {
                merged.add(left_radicand, &left_numerator * right_numerator);
                continue;
            }
            let (common, radicand) = memo.product(left_radicand, right_radicand);
            merged.add(&radicand, &left_numerator * right_numerator * IBig::from(common));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(radicand: u64, numerator: i64) -> (UBig, IBig) {
        (UBig::from(radicand), IBig::from(numerator))
    }

    #[test]
    fn product_of_radicands_pulls_out_the_common_factor() {
        // sqrt(6) * sqrt(10) = 2 * sqrt(15)
        assert_eq!(radicand_product(&UBig::from(6u8), &UBig::from(10u8)), (UBig::from(2u8), UBig::from(15u8)));
        // sqrt(6) * sqrt(6) = 6 * sqrt(1)
        assert_eq!(radicand_product(&UBig::from(6u8), &UBig::from(6u8)), (UBig::from(6u8), UBig::ONE));
    }

    #[test]
    fn the_memory_returns_the_same_pairs_in_both_orders_and_never_changes_an_answer() {
        let mut memo = ProductMemo::new();
        let mut off = ProductMemo::disabled();
        for (left, right) in [(6u64, 10u64), (10, 6), (15, 35), (35, 15), (6, 6), (2, 3)] {
            let (left, right) = (UBig::from(left), UBig::from(right));
            let expected = radicand_product(&left, &right);
            assert_eq!(memo.product(&left, &right), expected);
            assert_eq!(memo.product(&left, &right), expected);
            assert_eq!(off.product(&left, &right), expected);
        }
        assert_eq!(memo.len(), 4, "(6,10) and (10,6) share an entry, so do (15,35) and (35,15)");
        assert_eq!(off.len(), 0);
    }

    #[test]
    fn a_full_memory_is_dropped_whole() {
        let mut memo = ProductMemo::new();
        memo.limit = 3;
        for value in 2u64..10 {
            memo.product(&UBig::from(value), &UBig::from(value + 1));
            assert!(memo.len() <= 3);
        }
    }

    #[test]
    fn products_accumulate_like_the_python_dict_loop() {
        // (1 + 2 sqrt(2)) * (3 - sqrt(2)) = 3 - sqrt(2) + 6 sqrt(2) - 4 = -1 + 5 sqrt(2)
        let mut merged = Accumulator::new();
        let mut memo = ProductMemo::new();
        accumulate_products(&mut merged, &[item(1, 1), item(2, 2)], &[item(1, 3), item(2, -1)], &IBig::ONE, &mut memo);
        assert_eq!(merged.into_nonzero_items(), vec![item(1, -1), item(2, 5)]);
        // weight multiplies every product term
        let mut merged = Accumulator::new();
        accumulate_products(&mut merged, &[item(6, 1)], &[item(10, 1)], &IBig::from(-7), &mut memo);
        assert_eq!(merged.into_nonzero_items(), vec![item(15, -14)]);
    }

    #[test]
    fn zero_sums_are_dropped_and_order_is_by_radicand() {
        let mut merged = Accumulator::new();
        merged.add(&UBig::from(7u8), IBig::from(3));
        merged.add(&UBig::from(2u8), IBig::from(5));
        merged.add(&UBig::from(7u8), IBig::from(-3));
        assert_eq!(merged.len(), 2);
        assert_eq!(merged.into_nonzero_items(), vec![item(2, 5)]);
    }
}
