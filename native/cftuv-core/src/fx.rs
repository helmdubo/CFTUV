//! The stack road of the integer-form arithmetic: sums `sum a_m sqrt(m)` over [`Wide`] numerators and `u128` radicands.
//!
//! Every function here answers `None` when an operand or a result leaves the fixed capacity (a numerator over [`crate::wide::LIMBS`]
//! limbs, a radicand over 128 bits, too many distinct radicands); the caller then runs the `dashu-int` road it always had, from the
//! same inputs, so a refusal costs time and never an answer. The two roads are the same arithmetic on the same integers, which the unit
//! tests below and the differential suites hold together.

use crate::num::{IBig, UBig};
use crate::fxacc::FxAcc;
use crate::products::{Items, ProductMemo};
use crate::sqrt_sum::SqrtSum;
use crate::wide::{gcd_u64, Divisor, Wide};

/// Items one list holds (the lists of the conjugation loop have a handful).
pub const CAP: usize = 16;

/// A radicand as `u128`: `None` above 128 bits.
#[inline]
pub fn radicand_u128(value: &UBig) -> Option<u128> {
    match value.as_words() {
        [] => Some(0),
        [low] => Some(*low as u128),
        [low, high] => Some(*low as u128 | (*high as u128) << 64),
        _ => None,
    }
}

#[inline]
pub fn ubig_of_u128(value: u128) -> UBig {
    UBig::from(value)
}

/// The items of an integer form on the stack: radicands ascending, numerators non-zero.
#[derive(Clone)]
pub struct FxItems {
    n: usize,
    key: [u128; CAP],
    val: [Wide; CAP],
}

impl FxItems {
    pub fn new() -> FxItems {
        FxItems { n: 0, key: [0; CAP], val: [Wide::ZERO; CAP] }
    }

    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    pub fn key(&self, index: usize) -> u128 {
        self.key[index]
    }

    pub fn value(&self, index: usize) -> &Wide {
        &self.val[index]
    }

    /// The items of a `dashu-int` list (ascending, non-zero numerators): `None` when one does not fit.
    pub fn from_items(items: &[(UBig, IBig)]) -> Option<FxItems> {
        if items.len() > CAP {
            return None;
        }
        let mut out = FxItems::new();
        for (radicand, numerator) in items {
            out.key[out.n] = radicand_u128(radicand)?;
            out.val[out.n] = Wide::from_ibig(numerator)?;
            out.n += 1;
        }
        Some(out)
    }

    pub fn to_items(&self) -> Items {
        (0..self.n).map(|index| (ubig_of_u128(self.key[index]), self.val[index].to_ibig())).collect()
    }

    /// The denominator test of the conjugation loop: no term, or only the rational one.
    pub fn is_rational(&self) -> bool {
        self.n == 0 || (self.n == 1 && self.key[0] == 1)
    }

    /// The longest numerator, in limbs.
    pub fn max_limbs(&self) -> usize {
        self.val[..self.n].iter().map(Wide::limb_count).max().unwrap_or(0)
    }

    /// Every numerator times a positive factor; `None` when one does not fit.
    pub fn scaled(&self, factor: &Wide) -> Option<FxItems> {
        let mut out = FxItems::new();
        out.n = self.n;
        for index in 0..self.n {
            out.key[index] = self.key[index];
            out.val[index] = self.val[index].mul(factor)?;
        }
        Some(out)
    }

    /// The conjugate by `prime` (`A + B*sqrt(p)  ->  A - B*sqrt(p)`): the numerators of the radicands the prime divides change sign.
    pub fn flipped(&self, prime: u128) -> FxItems {
        let mut out = self.clone();
        for index in 0..self.n {
            let divisible = if prime >> 64 == 0 && self.key[index] >> 64 == 0 { (self.key[index] as u64) % (prime as u64) == 0 } else { self.key[index] % prime == 0 };
            if divisible {
                out.val[index] = self.val[index].neg();
            }
        }
        out
    }
}

impl Default for FxItems {
    fn default() -> FxItems {
        FxItems::new()
    }
}

/// `a * b * scale` accumulated into the scratch: the products of two item lists by radicand (`scale` the word the radicand pair factors
/// into the coefficient). `false` when something does not fit.
#[inline]
fn add_pair(acc: &mut FxAcc, memo: &mut ProductMemo, left_radicand: u128, left_value: &Wide, right_radicand: u128, right_value: &Wide) -> bool {
    let negative = left_value.is_negative() != right_value.is_negative();
    let (a, b) = (left_value.magnitude(), right_value.magnitude());
    if left_radicand == 1 {
        return acc.add_product(right_radicand, a, b, negative, 1);
    }
    if right_radicand == 1 {
        return acc.add_product(left_radicand, a, b, negative, 1);
    }
    let Some((shared, radicand)) = memo.product_u128(left_radicand, right_radicand) else { return false };
    if shared >> 64 != 0 {
        return false;
    }
    acc.add_product(radicand, a, b, negative, shared as u64)
}

/// [`add_pair`] with the product multiplied by a small word and its sign flipped when asked.
#[inline]
fn add_pair_scaled(acc: &mut FxAcc, memo: &mut ProductMemo, left_radicand: u128, left_value: &Wide, right_radicand: u128, right_value: &Wide, multiplier: u64, flip: bool) -> bool {
    let negative = (left_value.is_negative() != right_value.is_negative()) != flip;
    let (a, b) = (left_value.magnitude(), right_value.magnitude());
    if left_radicand == 1 {
        return acc.add_product(right_radicand, a, b, negative, multiplier);
    }
    if right_radicand == 1 {
        return acc.add_product(left_radicand, a, b, negative, multiplier);
    }
    let Some((shared, radicand)) = memo.product_u128(left_radicand, right_radicand) else { return false };
    match shared.checked_mul(multiplier as u128) {
        Some(scale) if scale >> 64 == 0 => acc.add_product(radicand, a, b, negative, scale as u64),
        _ => false,
    }
}

/// `items * conjugate(items)` for the conjugate by `prime` on the stack road: the squares of the terms the prime does not divide and of those it
/// divides, subtracted, from the products of their pairs taken once (see `sqrt_sum::norm_items`).
pub fn norm(items: &FxItems, prime: u128, memo: &mut ProductMemo) -> Option<FxItems> {
    let mut inside = [false; CAP];
    for index in 0..items.n {
        inside[index] = if prime >> 64 == 0 && items.key[index] >> 64 == 0 { (items.key[index] as u64) % (prime as u64) == 0 } else { items.key[index] % prime == 0 };
    }
    memo.with_acc(|acc, memo| {
        for first in 0..items.n {
            for second in first..items.n {
                if inside[second] != inside[first] {
                    continue;
                }
                let multiplier = if second == first { 1 } else { 2 };
                if !add_pair_scaled(acc, memo, items.key[first], &items.val[first], items.key[second], &items.val[second], multiplier, inside[first]) {
                    return None;
                }
            }
        }
        collect(acc)
    })
}

/// The sums of the scratch as a list: `None` when more than [`CAP`] survive or one does not fit a `Wide`.
fn collect(acc: &FxAcc) -> Option<FxItems> {
    let mut out = FxItems::new();
    out.n = acc.finish(&mut out.key, &mut out.val)?;
    Some(out)
}

/// `_multiply_integer_items` on the stack road: the product of two item lists, radicands ascending, zeros dropped.
pub fn multiply(left: &FxItems, right: &FxItems, memo: &mut ProductMemo) -> Option<FxItems> {
    memo.with_acc(|acc, memo| {
        for index in 0..left.n {
            for other in 0..right.n {
                if !add_pair(acc, memo, left.key[index], &left.val[index], right.key[other], &right.val[other]) {
                    return None;
                }
            }
        }
        collect(acc)
    })
}

/// `reduce_in_place` on the stack road: `gcd(L, *a_m)` divided out of `L` and every numerator. The gcd is taken from the shortest
/// operand outwards (it is the same number in any order, and a short start keeps every later step on one limb).
pub fn reduce(common: &mut Wide, items: &mut FxItems) {
    let mut start: Option<usize> = None; // `None`: the common denominator
    let mut shortest = (common.limb_count(), common.magnitude().last().copied().unwrap_or(0));
    for index in 0..items.n {
        let candidate = (items.val[index].limb_count(), items.val[index].magnitude().last().copied().unwrap_or(0));
        if candidate < shortest {
            shortest = candidate;
            start = Some(index);
        }
    }
    let mut divisor = match start {
        None => common.abs(),
        Some(index) => items.val[index].abs(),
    };
    if !divisor.is_one() {
        if start.is_some() {
            divisor = divisor.gcd(common);
        }
        divisor = gcd_chain(divisor, items, start);
    }
    if divisor.is_one() || divisor.is_zero() {
        return;
    }
    if divisor.limb_count() == 1 {
        let word = divisor.magnitude()[0];
        *common = common.div_exact_u64(word);
        for index in 0..items.n {
            items.val[index] = items.val[index].div_exact_u64(word);
        }
        return;
    }
    *common = common.div_exact(&divisor);
    for index in 0..items.n {
        items.val[index] = items.val[index].div_exact(&divisor);
    }
}

/// The two item lists of a quotient `(sum N_i sqrt(m_i) / n) / (sum D_j sqrt(m_j) / d)` over ONE common denominator, as the lists of the
/// quotient `sum N'_i sqrt(m_i) / sum D'_j sqrt(m_j)` of the same value: equal denominators change nothing, a denominator that divides the other
/// scales its list by the quotient, and otherwise each list is scaled by the other's denominator and their common factor is divided out. `None`
/// when a number does not fit.
pub fn over_one_denominator(numerator: FxItems, numerator_common: &Wide, denominator: FxItems, denominator_common: &Wide) -> Option<(FxItems, FxItems)> {
    if numerator_common == denominator_common {
        return Some((numerator, denominator));
    }
    // a one-word denominator that divides the other: scale the list over the other
    if numerator_common.limb_count() == 1 {
        let word = numerator_common.magnitude()[0];
        if Divisor::new(word).rem(denominator_common.magnitude()) == 0 {
            return Some((numerator.scaled(&denominator_common.div_exact_u64(word))?, denominator));
        }
    }
    if denominator_common.limb_count() == 1 {
        let word = denominator_common.magnitude()[0];
        if Divisor::new(word).rem(numerator_common.magnitude()) == 0 {
            return Some((numerator, denominator.scaled(&numerator_common.div_exact_u64(word))?));
        }
    }
    let (mut numerator, mut denominator) = (numerator.scaled(denominator_common)?, denominator.scaled(numerator_common)?);
    reduce_together(&mut numerator, &mut denominator);
    Some((numerator, denominator))
}

/// Divides both lists by the gcd of all their numerators (the quotient of the two sums does not change). The gcd is taken from the
/// shortest numerator outwards, as in [`reduce`].
pub fn reduce_together(first: &mut FxItems, second: &mut FxItems) {
    let mut shortest: Option<(bool, usize)> = None;
    let mut length = (usize::MAX, 0u64);
    for (which, list) in [(false, &*first), (true, &*second)] {
        for index in 0..list.n {
            let candidate = (list.val[index].limb_count(), list.val[index].magnitude().last().copied().unwrap_or(0));
            if candidate < length {
                length = candidate;
                shortest = Some((which, index));
            }
        }
    }
    let Some((which, index)) = shortest else { return };
    let start = if which { second.val[index].abs() } else { first.val[index].abs() };
    if start.is_one() {
        return;
    }
    // the other numerators of the list the start is in, then every numerator of the other list
    let own = if which { &*second } else { &*first };
    let other = if which { &*first } else { &*second };
    let mut divisor = gcd_chain(start, own, Some(index));
    divisor = gcd_chain(divisor, other, None);
    if divisor.is_one() || divisor.is_zero() {
        return;
    }
    for list in [&mut *first, &mut *second] {
        for index in 0..list.n {
            list.val[index] = if divisor.limb_count() == 1 { list.val[index].div_exact_u64(divisor.magnitude()[0]) } else { list.val[index].div_exact(&divisor) };
        }
    }
}

/// `gcd(start, every item but `skip`)`: while the running gcd is wider than a word the numbers are taken with `Wide::gcd`; once it is a word
/// long, each item costs one remainder by an invariant divisor and one word gcd.
fn gcd_chain(start: Wide, items: &FxItems, skip: Option<usize>) -> Wide {
    let mut divisor = start;
    let mut index = 0;
    while index < items.n && divisor.limb_count() > 1 {
        if Some(index) != skip {
            divisor = divisor.gcd(&items.val[index]);
        }
        index += 1;
    }
    if divisor.limb_count() != 1 || divisor.is_one() {
        return divisor;
    }
    let mut word = divisor.magnitude()[0];
    let mut invariant = Divisor::new(word);
    while index < items.n && word != 1 {
        if Some(index) != skip {
            let limbs = items.val[index].magnitude();
            let remainder = if limbs.len() == 1 { limbs[0] % word } else { invariant.rem(limbs) };
            if remainder != 0 {
                let next = gcd_u64(word, remainder);
                if next != word {
                    word = next;
                    if word != 1 {
                        invariant = Divisor::new(word);
                    }
                }
            }
        }
        index += 1;
    }
    Wide::from_u64(word)
}

/// `scaled_by_reciprocal_form` without its final reduction, on the stack road: the quotient of two lists over one common denominator whose denominator list is rational,
/// as one integer form `(|head|, a_m * sign(head))`, NOT reduced (whoever takes the quotient on reduces what comes out). `None` where the `dashu-int` road
/// has a refusal to make (a zero divisor) or where something does not fit: the caller runs that road.
pub fn scaled_by_reciprocal_form(numerator_items: &FxItems, denominator_items: &FxItems) -> Option<(Wide, FxItems)> {
    if denominator_items.is_empty() {
        return None;
    }
    let head = denominator_items.val[0];
    if numerator_items.is_empty() {
        return Some((Wide::from_u64(1), FxItems::new()));
    }
    let mut items = numerator_items.clone();
    if head.is_negative() {
        for index in 0..items.n {
            items.val[index] = items.val[index].neg();
        }
    }
    Some((head.abs(), items))
}

/// `base + left * right` on the stack road, as one value in lowest terms: `right` is an integer form `(right_common, right_items)`. `None`
/// when anything does not fit, and the caller runs the `dashu-int` road. The base has no Python `int` coefficient (the caller checked).
pub fn product_added(base: &SqrtSum, left: &SqrtSum, right_common: &Wide, right: &FxItems, memo: &mut ProductMemo) -> Option<SqrtSum> {
    let (base_form, left_form) = (base.int_form(), left.int_form());
    let base_common = Wide::from_ubig(&base_form.common)?;
    let common = Wide::from_ubig(&left_form.common)?.mul(right_common)?;
    let base_items = FxItems::from_items(&base_form.items)?;
    let left_items = FxItems::from_items(&left_form.items)?;
    // scale = lcm(base_common, common); the product is weighted by scale / common, the base by scale / base_common
    let (weight, factor, mut scale) = if base_common == common {
        (Wide::from_u64(1), Wide::from_u64(1), common)
    } else {
        let divisor = base_common.gcd(&common);
        let factor = common.div_exact(&divisor);
        (base_common.div_exact(&divisor), factor, base_common.mul(&factor)?)
    };
    let mut items = memo.with_acc(|acc, memo| {
        for index in 0..left_items.n {
            let weighted = if weight.is_one() { left_items.val[index] } else { left_items.val[index].mul(&weight)? };
            for other in 0..right.n {
                if !add_pair(acc, memo, left_items.key[index], &weighted, right.key[other], &right.val[other]) {
                    return None;
                }
            }
        }
        for index in 0..base_items.n {
            let value = &base_items.val[index];
            if !acc.add_product(base_items.key[index], value.magnitude(), factor.magnitude(), value.is_negative(), 1) {
                return None;
            }
        }
        collect(acc)
    })?;
    reduce(&mut scale, &mut items);
    Some(SqrtSum::from_sorted_form(scale.to_ubig(), items.to_items(), true))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::num;
    use crate::products::{accumulate_products, Accumulator};
    use crate::sqrt_sum::{reduce_in_place, IntForm};

    /// The `dashu-int` road, as `multiply_integer_items` runs it.
    fn multiply_integer_items_dashu(left: &[(UBig, IBig)], right: &[(UBig, IBig)], memo: &mut ProductMemo) -> Items {
        let mut merged = Accumulator::with_capacity(left.len() + right.len());
        accumulate_products(&mut merged, left, right, &IBig::ONE, memo);
        merged.into_nonzero_items()
    }

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }

        fn number(&mut self, max_limbs: usize) -> IBig {
            let len = 1 + (self.next() as usize) % max_limbs;
            let words: Vec<u64> = (0..len)
                .map(|_| match self.next() % 6 {
                    0 => u64::MAX,
                    1 => 0,
                    2 => 1,
                    _ => self.next(),
                })
                .collect();
            let value = IBig::from_sign_words(if self.next() % 2 == 0 { num::Sign::Negative } else { num::Sign::Positive }, &words);
            if value.is_zero() {
                IBig::from(3)
            } else {
                value
            }
        }

        /// Squarefree-ish radicands from a small pool (so products collide, merge and cancel) plus a few wide ones.
        fn radicand(&mut self) -> UBig {
            const POOL: [u64; 12] = [1, 2, 3, 5, 6, 7, 10, 14, 15, 21, 30, 35];
            match self.next() % 9 {
                0 => UBig::from(self.next() | 1),
                1 => UBig::from(self.next() as u128 * 6u128 + 1),
                _ => UBig::from(POOL[(self.next() as usize) % POOL.len()]),
            }
        }

        fn items(&mut self, max_items: usize, max_limbs: usize) -> Items {
            let count = 1 + (self.next() as usize) % max_items;
            let mut items: Items = (0..count).map(|_| (self.radicand(), self.number(max_limbs))).collect();
            items.sort_by(|a, b| a.0.cmp(&b.0));
            items.dedup_by(|a, b| a.0 == b.0);
            items
        }
    }

    #[test]
    fn products_equal_the_dashu_road_or_are_refused() {
        let mut rng = Rng(0x2545_f491_4f6c_dd1d);
        let mut memo = ProductMemo::new();
        let mut checked = 0;
        let mut refused = 0;
        for _ in 0..4000 {
            let left = rng.items(7, 5);
            let right = rng.items(7, 5);
            let (Some(fast_left), Some(fast_right)) = (FxItems::from_items(&left), FxItems::from_items(&right)) else { continue };
            let expected = multiply_integer_items_dashu(&left, &right, &mut memo);
            match multiply(&fast_left, &fast_right, &mut memo) {
                Some(found) => {
                    assert_eq!(found.to_items(), expected);
                    checked += 1;
                }
                None => refused += 1,
            }
        }
        assert!(checked > 1000, "the fast road must carry most random products ({checked} of the cases, {refused} refused)");
    }

    #[test]
    fn the_norm_equals_the_product_with_the_conjugate() {
        use crate::sqrt_sum::{conjugate_items, norm_items};
        let mut rng = Rng(0x00c0_ffee_1234_5678);
        let mut memo = ProductMemo::new();
        let mut checked = 0;
        for _ in 0..4000 {
            let items = rng.items(8, 4);
            let prime = UBig::from([2u64, 3, 5, 7][(rng.next() % 4) as usize]);
            let expected = multiply_integer_items_dashu(&items, &conjugate_items(&items, &prime), &mut memo);
            assert_eq!(norm_items(&items, &prime, &mut memo), expected, "the dashu norm");
            let Some(fast) = FxItems::from_items(&items) else { continue };
            if let Some(found) = norm(&fast, fx_prime(&prime), &mut memo) {
                assert_eq!(found.to_items(), expected, "the stack norm");
                checked += 1;
            }
        }
        assert!(checked > 1500);
    }

    fn fx_prime(prime: &UBig) -> u128 {
        radicand_u128(prime).unwrap()
    }

    #[test]
    fn a_square_of_a_conjugate_pair_cancels_its_cross_terms() {
        // (3 + 2 sqrt(2) + sqrt(3)) * (3 - 2 sqrt(2) + sqrt(3)) has no sqrt(2)-by-constant cross terms left
        let mut memo = ProductMemo::new();
        let make = |items: &[(u64, i64)]| -> Items { items.iter().map(|(r, v)| (UBig::from(*r), IBig::from(*v))).collect() };
        let left = make(&[(1, 3), (2, 2), (3, 1)]);
        let right = make(&[(1, 3), (2, -2), (3, 1)]);
        let (a, b) = (FxItems::from_items(&left).unwrap(), FxItems::from_items(&right).unwrap());
        let found = multiply(&a, &b, &mut memo).unwrap().to_items();
        assert_eq!(found, multiply_integer_items_dashu(&left, &right, &mut memo));
        // a product that is entirely zero is an empty list
        let zero = multiply(&FxItems::from_items(&make(&[(2, 5)])).unwrap(), &FxItems::new(), &mut memo).unwrap();
        assert!(zero.is_empty());
    }

    #[test]
    fn reduction_divides_out_exactly_the_common_divisor() {
        let mut rng = Rng(0x1357_9bdf_2468_ace0);
        for _ in 0..6000 {
            let mut items = rng.items(6, 4);
            // plant a common factor of the numerators and the denominator
            let factor = rng.number(2).abs_value();
            let common_base = UBig::from(1 + rng.next() % 1000);
            let common = &common_base * &factor;
            for (_, value) in items.iter_mut() {
                *value = &*value * IBig::from(factor.clone());
            }
            let (mut slow_common, mut slow_items) = (common.clone(), items.clone());
            reduce_in_place(&mut slow_common, &mut slow_items);
            let (Some(mut fast_common), Some(mut fast_items)) = (Wide::from_ubig(&common), FxItems::from_items(&items)) else { continue };
            reduce(&mut fast_common, &mut fast_items);
            assert_eq!(fast_common.to_ubig(), slow_common);
            assert_eq!(fast_items.to_items(), slow_items);
        }
    }

    #[test]
    fn the_reciprocal_form_equals_the_dashu_road() {
        use crate::sqrt_sum::scaled_by_reciprocal_form as dashu_road;
        let mut rng = Rng(0x7777_1234_abcd_ef01);
        let mut compared = 0;
        for _ in 0..6000 {
            let numerator_items = rng.items(6, 4);
            let numerator_common = UBig::from(1 + rng.next() % 5000) * UBig::from(1 + rng.next() % 90);
            let denominator_common = UBig::from(1 + rng.next() % 4000);
            let head = rng.number(3);
            let denominator_items: Items = vec![(UBig::ONE, head)];
            let expected = dashu_road(&numerator_common, &numerator_items, &denominator_common, &denominator_items).unwrap();
            let (Some(nc), Some(ni), Some(dc), Some(di)) =
                (Wide::from_ubig(&numerator_common), FxItems::from_items(&numerator_items), Wide::from_ubig(&denominator_common), FxItems::from_items(&denominator_items))
            else {
                continue;
            };
            // over one common denominator: each list times the other's common denominator
            let (Some(over_numerator), Some(over_denominator)) = (ni.scaled(&dc), di.scaled(&nc)) else { continue };
            if let Some((mut common, mut items)) = scaled_by_reciprocal_form(&over_numerator, &over_denominator) {
                // both are the quotient in lowest terms, which is one form
                reduce(&mut common, &mut items);
                assert_eq!(IntForm { common: common.to_ubig(), items: items.to_items() }, expected);
                compared += 1;
            }
        }
        assert!(compared > 1000);
        // an empty numerator is the zero form, and an empty denominator is left to the dashu road to refuse
        let empty = FxItems::new();
        let head = FxItems::from_items(&[(UBig::ONE, IBig::from(-4))]).unwrap();
        let (zero_common, zero_items) = scaled_by_reciprocal_form(&empty, &head).unwrap();
        assert_eq!((zero_common, zero_items.len()), (Wide::from_u64(1), 0));
        assert!(scaled_by_reciprocal_form(&head, &empty).is_none());
    }

    #[test]
    fn lists_over_one_denominator_make_the_same_quotient() {
        use crate::sqrt_sum::scaled_by_reciprocal_form as dashu_road;
        let mut rng = Rng(0x0ddb_a11_cafe_f00d);
        let mut compared = 0;
        for _ in 0..8000 {
            let numerator_items = rng.items(5, 3);
            let head = rng.number(2);
            let base = 1 + rng.next() % 600;
            // equal denominators, a denominator that divides the other (either way), and unrelated ones
            let (numerator_common, denominator_common) = match rng.next() % 4 {
                0 => (base, base),
                1 => (base, base * (1 + rng.next() % 40)),
                2 => (base * (1 + rng.next() % 40), base),
                _ => (base, 1 + rng.next() % 9000),
            };
            let (nc, dc) = (UBig::from(numerator_common), UBig::from(denominator_common));
            let denominator_items: Items = vec![(UBig::ONE, head)];
            let expected = dashu_road(&nc, &numerator_items, &dc, &denominator_items).unwrap();
            let (Some(ni), Some(di)) = (FxItems::from_items(&numerator_items), FxItems::from_items(&denominator_items)) else { continue };
            let Some((over_numerator, over_denominator)) = over_one_denominator(ni, &Wide::from_ubig(&nc).unwrap(), di, &Wide::from_ubig(&dc).unwrap()) else { continue };
            let (mut common, mut items) = scaled_by_reciprocal_form(&over_numerator, &over_denominator).unwrap();
            reduce(&mut common, &mut items);
            assert_eq!(IntForm { common: common.to_ubig(), items: items.to_items() }, expected);
            compared += 1;
        }
        assert!(compared > 4000);
    }

    #[test]
    fn dividing_both_lists_by_their_common_factor_keeps_the_quotient() {
        let mut rng = Rng(0x5eed_f00d_5eed_f00d);
        for _ in 0..6000 {
            let (numerator, denominator) = (rng.items(5, 3), rng.items(5, 3));
            let factor = IBig::from(1 + rng.next() % 100_000) * IBig::from(1 + rng.next() % 1000);
            let scale = |items: &Items| -> Items { items.iter().map(|(radicand, value)| (radicand.clone(), value * &factor)).collect() };
            let (Some(mut first), Some(mut second)) = (FxItems::from_items(&scale(&numerator)), FxItems::from_items(&scale(&denominator))) else { continue };
            reduce_together(&mut first, &mut second);
            // what is left has no common factor any more, and the factor was taken out of every numerator
            let mut all = first.to_items();
            all.extend(second.to_items());
            let content = all.iter().fold(UBig::ZERO, |content, (_, value)| num::gcd(&content, &num::magnitude(value)));
            assert!(content.is_one() || content.is_zero(), "content {content}");
            let original: Vec<IBig> = scale(&numerator).into_iter().chain(scale(&denominator)).map(|(_, value)| value).collect();
            let reduced: Vec<IBig> = all.into_iter().map(|(_, value)| value).collect();
            let divisor = &original[0] / &reduced[0];
            assert!(original.iter().zip(&reduced).all(|(before, after)| *before == after * &divisor), "every numerator was divided by the same number");
        }
    }

    trait AbsValue {
        fn abs_value(&self) -> UBig;
    }

    impl AbsValue for IBig {
        fn abs_value(&self) -> UBig {
            num::magnitude(self)
        }
    }
}
