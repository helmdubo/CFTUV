//! Integer factorization with Python-exact budget accounting (`exact_sqrt_sum.py` lines 472-634 and 869-913):
//! deterministic Miller-Rabin, Pollard-Brent with the `random.Random(n)` seeding, the `_rho_factors` stack
//! discipline and the coprime basis.
//!
//! The VALUES (gcds, divisors, witnesses) equal Python's; how `x*y mod n` is computed is free. Hot paths run in
//! fixed-limb Montgomery form (`mont.rs`) for odd moduli up to `MAX_LIMBS` words; anything else takes the plain
//! `UBig` path, which is a line-by-line transcription of the oracle and doubles as the differential reference.

use dashu_base::{BitTest, Gcd, SquareRoot};
use dashu_int::{IBig, UBig};

use crate::budget::{Exhausted, Operation, WorkBudget};
use crate::mont::{from_array, Mont, MAX_LIMBS};
use crate::pyrandom::{PyRandom, RandomError};

/// Everything a canonicalization call can refuse with. The host formats the messages.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CanonError {
    /// The budget cap was crossed (`ExactCanonicalizationWorkBudgetExhausted`).
    Exhausted(Exhausted),
    /// `NegativeRadicandError(f"под корнем {value}")`, `value = numerator / denominator` (`denominator = 1` for ints).
    NegativeRadicand { numerator: IBig, denominator: UBig },
    /// `ArithmeticError`: the factorization did not rebuild the radicand. Unreachable unless the prover is wrong.
    ReconstructionFailed { radicand: UBig },
    /// A Python `ValueError` / empty range / state that cannot be mirrored.
    InvalidInput(&'static str),
}

impl From<Exhausted> for CanonError {
    fn from(value: Exhausted) -> Self {
        CanonError::Exhausted(value)
    }
}

impl From<RandomError> for CanonError {
    fn from(_: RandomError) -> Self {
        CanonError::InvalidInput("random range is empty")
    }
}

/// `_MILLER_RABIN_BASES`.
pub const MILLER_RABIN_BASES: [u64; 12] = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37];

/// The Brent batch of `_pollard_rho`.
pub const BRENT_BATCH_SIZE: u64 = 64;

/// A factorization as `(prime, power)` pairs.
pub type Pairs = Vec<(UBig, u64)>;

/// Binary gcd of machine words (`gcd(0, x) = x`).
fn gcd_word(mut a: u64, mut b: u64) -> u64 {
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

/// `math.gcd(a, b)` for non-negative integers. Most real operands fit a machine word (a coprime-basis atom, a
/// small prime): when one side does, reduce the other modulo it and finish on words.
pub fn gcd_values(a: &UBig, b: &UBig) -> UBig {
    match (u64::try_from(a), u64::try_from(b)) {
        (Ok(x), Ok(y)) => return UBig::from(gcd_word(x, y)),
        (Ok(x), Err(_)) if x != 0 => return UBig::from(gcd_word(x, b % x)),
        (Err(_), Ok(y)) if y != 0 => return UBig::from(gcd_word(y, a % y)),
        _ => {}
    }
    if *a == UBig::ZERO {
        return b.clone();
    }
    if *b == UBig::ZERO {
        return a.clone();
    }
    a.gcd(b)
}

// ---- Miller-Rabin ----------------------------------------------------------------------------------------------

/// `_is_prime`: one round plus `d.bit_length()` squarings are paid before each `pow`, the witness loop is paid
/// after it, all against the budget with `PRIMALITY` and radicand `n`.
pub fn is_prime(n: &UBig, budget: &mut WorkBudget) -> Result<bool, Exhausted> {
    if *n < UBig::from(2u8) {
        return Ok(false);
    }
    for small in MILLER_RABIN_BASES {
        if n % small == 0 {
            return Ok(*n == UBig::from(small));
        }
    }
    let n_minus_one = n - UBig::ONE;
    let r = n_minus_one.trailing_zeros().unwrap_or(0) as u64;
    let d = &n_minus_one >> (r as usize);
    match n.as_words().len() {
        1 => is_prime_mont::<1>(n, &d, r, budget),
        2 => is_prime_mont::<2>(n, &d, r, budget),
        3 => is_prime_mont::<3>(n, &d, r, budget),
        4 => is_prime_mont::<4>(n, &d, r, budget),
        5 => is_prime_mont::<5>(n, &d, r, budget),
        6 => is_prime_mont::<6>(n, &d, r, budget),
        7 => is_prime_mont::<7>(n, &d, r, budget),
        8 => is_prime_mont::<8>(n, &d, r, budget),
        _ => is_prime_plain(n, &d, r, budget),
    }
}

fn is_prime_mont<const K: usize>(n: &UBig, d: &UBig, r: u64, budget: &mut WorkBudget) -> Result<bool, Exhausted> {
    let Some(ctx) = Mont::<K>::new(n) else {
        return is_prime_plain(n, d, r, budget);
    };
    let exponent_cost = d.bit_len() as u64;
    let one = ctx.one;
    let minus_one = ctx.sub(&[0u64; K], &one);
    for base in MILLER_RABIN_BASES {
        budget.spend_miller_rabin_rounds(1, Operation::Primality, n)?;
        budget.spend_modular_squarings(exponent_cost, Operation::Primality, n)?;
        let base_m = ctx.to_mont(&UBig::from(base));
        let mut x = ctx.pow(&base_m, d);
        if x == one || x == minus_one {
            continue;
        }
        let mut witnessed = false;
        let mut steps = 0u64;
        for _ in 0..r.saturating_sub(1) {
            x = ctx.mul(&x, &x);
            steps += 1;
            if x == minus_one {
                witnessed = true;
                break;
            }
        }
        budget.spend_modular_squarings(steps, Operation::Primality, n)?;
        if !witnessed {
            return Ok(false);
        }
    }
    Ok(true)
}

/// `pow(base, exponent, n)` on plain integers, left-to-right binary (`exponent >= 1`).
fn pow_mod_plain(base: &UBig, exponent: &UBig, n: &UBig) -> UBig {
    let mut result = base % n;
    let mut position = exponent.bit_len().saturating_sub(1);
    while position > 0 {
        position -= 1;
        result = &result * &result % n;
        if (exponent.as_words()[position / 64] >> (position % 64)) & 1 == 1 {
            result = &result * base % n;
        }
    }
    result
}

/// The oracle transcribed on plain integers: the reference for the Montgomery path and the fallback for wide `n`.
pub fn is_prime_plain(n: &UBig, d: &UBig, r: u64, budget: &mut WorkBudget) -> Result<bool, Exhausted> {
    let exponent_cost = d.bit_len() as u64;
    let n_minus_one = n - UBig::ONE;
    for base in MILLER_RABIN_BASES {
        budget.spend_miller_rabin_rounds(1, Operation::Primality, n)?;
        budget.spend_modular_squarings(exponent_cost, Operation::Primality, n)?;
        let mut x = pow_mod_plain(&UBig::from(base), d, n);
        if x == UBig::ONE || x == n_minus_one {
            continue;
        }
        let mut witnessed = false;
        let mut steps = 0u64;
        for _ in 0..r.saturating_sub(1) {
            x = &x * &x % n;
            steps += 1;
            if x == n_minus_one {
                witnessed = true;
                break;
            }
        }
        budget.spend_modular_squarings(steps, Operation::Primality, n)?;
        if !witnessed {
            return Ok(false);
        }
    }
    Ok(true)
}

// ---- Pollard-Brent ---------------------------------------------------------------------------------------------

/// `_pollard_rho(n)`: a nontrivial divisor of a composite `n`. `random.Random(n)` seeds the orbit parameters.
pub fn pollard_rho(n: &UBig, budget: &mut WorkBudget) -> Result<UBig, CanonError> {
    if n.as_words().first().is_none_or(|word| word & 1 == 0) {
        return Ok(UBig::from(2u8));
    }
    let mut rng = PyRandom::from_seed(n);
    loop {
        budget.spend_pollard_attempts(1, Operation::PollardRhoBrent, n)?;
        let c = rng.randrange(&UBig::ONE, n)?;
        let y = rng.randrange(&UBig::ZERO, n)?;
        if let Some(divisor) = pollard_rho_brent_attempt(n, &y, &c, BRENT_BATCH_SIZE, budget)? {
            return Ok(divisor);
        }
    }
}

/// `_pollard_rho_brent_attempt(n, y, c, batch_size=...)`. `y` and `c` are reduced mod `n` first, which changes
/// no gcd of the orbit. `None` asks for the next parameter pair.
pub fn pollard_rho_brent_attempt(
    n: &UBig,
    y: &UBig,
    c: &UBig,
    batch_size: u64,
    budget: &mut WorkBudget,
) -> Result<Option<UBig>, CanonError> {
    if batch_size < 1 {
        return Err(CanonError::InvalidInput("Brent batch size must be positive"));
    }
    if *n == UBig::ZERO {
        return Err(CanonError::InvalidInput("Brent attempt needs a positive modulus"));
    }
    let attempt = match n.as_words().len() {
        1 => brent_mont::<1>(n, y, c, batch_size, budget),
        2 => brent_mont::<2>(n, y, c, batch_size, budget),
        3 => brent_mont::<3>(n, y, c, batch_size, budget),
        4 => brent_mont::<4>(n, y, c, batch_size, budget),
        5 => brent_mont::<5>(n, y, c, batch_size, budget),
        6 => brent_mont::<6>(n, y, c, batch_size, budget),
        7 => brent_mont::<7>(n, y, c, batch_size, budget),
        8 => brent_mont::<8>(n, y, c, batch_size, budget),
        _ => brent_plain(n, y, c, batch_size, budget),
    };
    Ok(attempt?)
}

/// Speculative batches per gcd group (see `brent_mont`).
const MAX_GROUP: usize = 8;

/// One computed-but-not-yet-charged batch of the orbit.
#[derive(Clone, Copy)]
struct Batch<const K: usize> {
    steps: u64,
    checkpoint: [u64; K],
    product: [u64; K],
}

/// The Brent orbit in Montgomery form.
///
/// Charges, returns and the replay follow Python batch by batch. The one liberty is WHEN a batch is computed
/// and its gcd taken: in a long round up to `MAX_GROUP` batches are computed ahead (their charges are paid only
/// when their turn comes) and one gcd of their combined product replaces the per-batch gcds. If that gcd is 1,
/// every batch gcd is 1; otherwise each batch gcd is taken in order, exactly as Python does. Work computed
/// beyond a returning or exhausted batch is discarded, so nothing observable changes.
fn brent_mont<const K: usize>(
    n: &UBig,
    y: &UBig,
    c: &UBig,
    batch_size: u64,
    budget: &mut WorkBudget,
) -> Result<Option<UBig>, Exhausted> {
    let Some(ctx) = Mont::<K>::new(n) else {
        return brent_plain(n, y, c, batch_size, budget);
    };
    debug_assert!(K <= MAX_LIMBS);
    let operation = Operation::PollardRhoBrent;
    let c_m = ctx.to_mont(c);
    let mut y = ctx.to_mont(y);
    let step = |value: &[u64; K]| ctx.add(&ctx.mul(value, value), &c_m);
    let mut power = 1u64;
    loop {
        let x = y;
        budget.spend_modular_squarings(power, operation, n)?;
        for _ in 0..power {
            y = step(&y);
        }
        let group_limit = ((power / 256) as usize).clamp(1, MAX_GROUP);
        let mut offset = 0u64;
        while offset < power {
            let mut batches = [Batch { steps: 0, checkpoint: x, product: x }; MAX_GROUP];
            let mut count = 0;
            let mut cursor = offset;
            while count < group_limit && cursor < power {
                let steps = batch_size.min(power - cursor);
                let checkpoint = y;
                let mut product = ctx.one;
                for _ in 0..steps {
                    y = step(&y);
                    product = ctx.mul(&product, &ctx.sub(&x, &y));
                }
                batches[count] = Batch { steps, checkpoint, product };
                cursor += steps;
                count += 1;
            }
            let all_coprime = count > 1 && {
                let combined = batches[..count].iter().fold(ctx.one, |total, batch| ctx.mul(&total, &batch.product));
                gcd_values(&from_array(&combined), n) == UBig::ONE
            };
            for batch in &batches[..count] {
                budget.spend_modular_squarings(batch.steps, operation, n)?;
                budget.spend_gcd_operations(1, operation, n)?;
                let divisor = if all_coprime { UBig::ONE } else { gcd_values(&from_array(&batch.product), n) };
                if divisor == UBig::ONE {
                    offset += batch.steps;
                    continue;
                }
                if divisor < *n {
                    return Ok(Some(divisor));
                }
                budget.spend_modular_squarings(batch.steps, operation, n)?;
                budget.spend_gcd_operations(batch.steps, operation, n)?;
                let mut replay = batch.checkpoint;
                for _ in 0..batch.steps {
                    replay = step(&replay);
                    let divisor = gcd_values(&from_array(&ctx.sub(&x, &replay)), n);
                    if divisor == UBig::ONE {
                        continue;
                    }
                    if divisor < *n {
                        return Ok(Some(divisor));
                    }
                    return Ok(None);
                }
                return Ok(None);
            }
        }
        power = power.saturating_mul(2);
    }
}

/// The oracle transcribed on plain integers.
pub fn brent_plain(
    n: &UBig,
    y: &UBig,
    c: &UBig,
    batch_size: u64,
    budget: &mut WorkBudget,
) -> Result<Option<UBig>, Exhausted> {
    let operation = Operation::PollardRhoBrent;
    let reduce = |value: UBig| value % n;
    let step = |value: &UBig| reduce(value * value + c);
    let mut y = y.clone();
    let mut power = 1u64;
    loop {
        let x = y.clone();
        budget.spend_modular_squarings(power, operation, n)?;
        for _ in 0..power {
            y = step(&y);
        }
        let mut offset = 0u64;
        while offset < power {
            let checkpoint = y.clone();
            let steps = batch_size.min(power - offset);
            budget.spend_modular_squarings(steps, operation, n)?;
            budget.spend_gcd_operations(1, operation, n)?;
            let mut product = UBig::ONE;
            for _ in 0..steps {
                y = step(&y);
                let difference = if x >= y { &x - &y } else { &y - &x };
                product = reduce(product * difference);
            }
            let divisor = gcd_values(&product, n);
            if divisor == UBig::ONE {
                offset += steps;
                continue;
            }
            if UBig::ONE < divisor && divisor < *n {
                return Ok(Some(divisor));
            }
            budget.spend_modular_squarings(steps, operation, n)?;
            budget.spend_gcd_operations(steps, operation, n)?;
            let mut replay = checkpoint;
            for _ in 0..steps {
                replay = step(&replay);
                let difference = if x >= replay { &x - &replay } else { &replay - &x };
                let divisor = gcd_values(&difference, n);
                if divisor == UBig::ONE {
                    continue;
                }
                if UBig::ONE < divisor && divisor < *n {
                    return Ok(Some(divisor));
                }
                return Ok(None);
            }
            return Ok(None);
        }
        power = power.saturating_mul(2);
    }
}

// ---- _rho_factors ----------------------------------------------------------------------------------------------

/// `factors[prime] += power`, keeping the dict's first-insertion order.
pub fn add_factor(factors: &mut Pairs, prime: &UBig, power: u64) {
    match factors.iter_mut().find(|(known, _)| known == prime) {
        Some((_, count)) => *count += power,
        None => factors.push((prime.clone(), power)),
    }
}

/// `_rho_factors(n)`: factorization from scratch; the result is in dict INSERTION order (stack discipline), which
/// is the order `_register_prime` sees.
pub fn rho_factors(n: &UBig, budget: &mut WorkBudget) -> Result<Pairs, CanonError> {
    if *n == UBig::ZERO {
        return Err(CanonError::InvalidInput("zero has no factorization"));
    }
    let mut factors: Pairs = Vec::new();
    let mut stack = vec![n.clone()];
    while let Some(value) = stack.pop() {
        if value == UBig::ONE {
            continue;
        }
        if is_prime(&value, budget)? {
            add_factor(&mut factors, &value, 1);
            continue;
        }
        let root = value.sqrt();
        if &root * &root == value {
            stack.push(root.clone());
            stack.push(root);
            continue;
        }
        let divisor = pollard_rho(&value, budget)?;
        let cofactor = &value / &divisor;
        stack.push(divisor);
        stack.push(cofactor);
    }
    Ok(factors)
}

// ---- _coprime_basis --------------------------------------------------------------------------------------------

/// `_COPRIME_BASIS_SPLIT_BUDGET`.
pub const COPRIME_BASIS_SPLIT_BUDGET: u64 = 1 << 12;

/// `_coprime_basis(values)`: only gcds, paid one per atom comparison with `COPRIME_BASIS` and the value being
/// placed as radicand. Stack order and basis order are the oracle's.
pub fn coprime_basis(values: &[UBig], budget: &mut WorkBudget) -> Result<Vec<UBig>, Exhausted> {
    let mut basis: Vec<UBig> = Vec::new();
    let mut stack: Vec<UBig> = values.iter().filter(|value| **value > UBig::ONE).cloned().collect();
    let mut splits = 0u64;
    while let Some(value) = stack.pop() {
        let mut placed = false;
        for index in 0..basis.len() {
            budget.spend_gcd_operations(1, Operation::CoprimeBasis, &value)?;
            let atom = &basis[index];
            let common = gcd_values(&value, atom);
            if common == UBig::ONE {
                continue;
            }
            if common == value && common == *atom {
                placed = true;
                break;
            }
            if splits >= COPRIME_BASIS_SPLIT_BUDGET {
                break;
            }
            splits += 1;
            let atom_rest = atom / &common;
            let value_rest = &value / &common;
            basis.remove(index);
            stack.push(common);
            if atom_rest > UBig::ONE {
                stack.push(atom_rest);
            }
            if value_rest > UBig::ONE {
                stack.push(value_rest);
            }
            placed = true;
            break;
        }
        if !placed {
            basis.push(value);
        }
    }
    Ok(basis)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unlimited() -> WorkBudget {
        WorkBudget::unlimited()
    }

    fn big(value: u128) -> UBig {
        UBig::from(value)
    }

    #[test]
    fn miller_rabin_agrees_with_trial_division_on_small_numbers() {
        let mut budget = unlimited();
        for n in 0u64..3000 {
            let expected = n >= 2 && (2..).take_while(|p| p * p <= n).all(|p| n % p != 0);
            assert_eq!(is_prime(&UBig::from(n), &mut budget).unwrap(), expected, "n = {n}");
        }
    }

    #[test]
    fn montgomery_and_plain_miller_rabin_charge_the_same_and_agree() {
        let samples = [
            big(1_000_000_007),
            big(1_000_000_007) * big(998_244_353),
            big(3_215_031_751),
            big(0xffff_ffff_ffff_ffc5),
            (UBig::ONE << 127usize) - UBig::ONE,
            ((UBig::ONE << 89usize) - UBig::ONE) * ((UBig::ONE << 61usize) - UBig::ONE),
            (UBig::ONE << 521usize) - UBig::ONE,
        ];
        for n in samples {
            let n_minus_one = &n - UBig::ONE;
            let r = n_minus_one.trailing_zeros().unwrap_or(0) as u64;
            let d = &n_minus_one >> (r as usize);
            let mut fast = unlimited();
            let mut plain = unlimited();
            let a = is_prime(&n, &mut fast).unwrap();
            let b = is_prime_plain(&n, &d, r, &mut plain).unwrap();
            assert_eq!(a, b);
            assert_eq!(fast.articles(), plain.articles());
        }
    }

    #[test]
    fn brent_montgomery_matches_the_plain_transcription() {
        let semiprimes = [
            big(1_000_003) * big(999_983),
            big(2_147_483_647) * big(1_000_000_007),
            ((UBig::ONE << 89usize) - UBig::ONE) * big(1_000_003),
            ((UBig::ONE << 127usize) - UBig::ONE) * big(100_003) * big(10_007),
        ];
        for n in semiprimes {
            let mut rng = PyRandom::from_seed(&n);
            for _ in 0..6 {
                let c = rng.randrange(&UBig::ONE, &n).unwrap();
                let y = rng.randrange(&UBig::ZERO, &n).unwrap();
                let mut fast = unlimited();
                let mut plain = unlimited();
                let a = pollard_rho_brent_attempt(&n, &y, &c, 64, &mut fast).unwrap();
                let b = brent_plain(&n, &y, &c, 64, &mut plain).unwrap();
                assert_eq!(a, b);
                assert_eq!(fast.articles(), plain.articles());
            }
        }
    }

    fn lcg(state: &mut u64) -> u64 {
        *state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        *state >> 11
    }

    /// Random odd composite `small * large` of `words` limbs, `small` below 2^20 so the orbit closes quickly.
    fn random_modulus(words: usize, state: &mut u64) -> UBig {
        let small = UBig::from((lcg(state) % 1_000_000) * 2 + 3);
        let mut limbs: Vec<u64> = (0..words).map(|_| lcg(state) << 11 ^ lcg(state)).collect();
        limbs[words - 1] |= 1 << 63;
        limbs[0] |= 1;
        let large = (UBig::from_words(&limbs) / &small) | UBig::ONE;
        let n = small * large;
        if n.as_words().len() == words { n } else { random_modulus(words, state) }
    }

    #[test]
    fn brent_montgomery_matches_plain_under_random_caps_batches_and_parameters() {
        let mut state = 0x5eed_u64;
        for words in 1..=8usize {
            for _ in 0..25 {
                let n = random_modulus(words, &mut state);
                if n.as_words()[0] & 1 == 0 {
                    continue;
                }
                let c = UBig::from(lcg(&mut state)) % &n;
                let y = UBig::from(lcg(&mut state)) * UBig::from(lcg(&mut state)) % &n;
                let batch = [1u64, 3, 64, 100][(lcg(&mut state) % 4) as usize];
                let mut full = unlimited();
                let reference = brent_plain(&n, &y, &c, batch, &mut full).unwrap();
                let total = full.spent();
                for cap in [0, 1, total / 3, total.saturating_sub(1), total, total + 1] {
                    let mut fast = WorkBudget::bounded(cap);
                    let mut plain = WorkBudget::bounded(cap);
                    let a = pollard_rho_brent_attempt(&n, &y, &c, batch, &mut fast);
                    let b = brent_plain(&n, &y, &c, batch, &mut plain);
                    assert_eq!(a, b.map_err(CanonError::from), "n={n} c={c} y={y} batch={batch} cap={cap}");
                    assert_eq!(fast.articles(), plain.articles(), "n={n} batch={batch} cap={cap}");
                }
                let _ = reference;
            }
        }
    }

    fn prime_near(bits: u32, state: &mut u64) -> UBig {
        loop {
            let candidate = UBig::from((lcg(state) & ((1u64 << bits) - 1)) | 1 << (bits - 1) | 1);
            if is_prime(&candidate, &mut unlimited()).unwrap() {
                return candidate;
            }
        }
    }

    #[test]
    fn brent_groups_of_batches_match_plain_in_long_orbits_under_random_caps() {
        let mut state = 0xface_u64;
        for (small_bits, large_words) in [(27u32, 1usize), (28, 2), (29, 4), (26, 6)] {
            let small = prime_near(small_bits, &mut state);
            let mut limbs: Vec<u64> = (0..large_words).map(|_| lcg(&mut state) << 11 ^ lcg(&mut state)).collect();
            limbs[large_words - 1] |= 1 << 63;
            let mut large = UBig::from_words(&limbs) | UBig::ONE;
            while !is_prime(&large, &mut unlimited()).unwrap() {
                large += 2u8;
            }
            let n = &small * &large;
            let mut rng = PyRandom::from_seed(&n);
            for _ in 0..2 {
                let c = rng.randrange(&UBig::ONE, &n).unwrap();
                let y = rng.randrange(&UBig::ZERO, &n).unwrap();
                let mut full = unlimited();
                let reference = brent_plain(&n, &y, &c, 64, &mut full).unwrap();
                let total = full.spent();
                assert!(total > 5000, "the orbit must be long enough for grouped batches");
                let mut caps: Vec<u64> = (0..14).map(|_| lcg(&mut state) % (total + 2)).collect();
                caps.extend([total - 1, total, total + 1]);
                for cap in caps {
                    let mut fast = WorkBudget::bounded(cap);
                    let mut plain = WorkBudget::bounded(cap);
                    let a = pollard_rho_brent_attempt(&n, &y, &c, 64, &mut fast);
                    let b = brent_plain(&n, &y, &c, 64, &mut plain);
                    assert_eq!(a, b.map_err(CanonError::from), "n={n} cap={cap}");
                    assert_eq!(fast.articles(), plain.articles(), "n={n} cap={cap}");
                }
                let mut unbounded = unlimited();
                assert_eq!(pollard_rho_brent_attempt(&n, &y, &c, 64, &mut unbounded), Ok(reference));
            }
        }
    }

    #[test]
    fn rho_factors_keeps_stack_insertion_order_and_counts_repeats() {
        let n = big(7) * big(7) * big(1_000_003) * big(999_983) * big(999_983);
        let mut budget = unlimited();
        let factors = rho_factors(&n, &mut budget).unwrap();
        let mut sorted = factors.clone();
        sorted.sort();
        assert_eq!(
            sorted,
            vec![(big(7), 2), (big(999_983), 2), (big(1_000_003), 1)]
        );
    }

    #[test]
    fn gcd_fast_paths_agree_with_the_general_gcd() {
        let mut rng = PyRandom::from_key(&[3, 1, 4, 1, 5]);
        for (bits_a, bits_b) in [(0u32, 5u32), (1, 1), (5, 0), (20, 30), (64, 64), (63, 130), (130, 63), (200, 64), (64, 200), (250, 120), (300, 300)] {
            for _ in 0..40 {
                let shared = rng.getrandbits(17);
                let a = rng.getrandbits(bits_a) * &shared;
                let b = rng.getrandbits(bits_b) * &shared;
                if a == UBig::ZERO && b == UBig::ZERO {
                    assert_eq!(gcd_values(&a, &b), UBig::ZERO);
                    continue;
                }
                let expected = if a == UBig::ZERO { b.clone() } else if b == UBig::ZERO { a.clone() } else { (&a).gcd(&b) };
                assert_eq!(gcd_values(&a, &b), expected, "{a} {b}");
            }
        }
    }

    #[test]
    fn coprime_basis_splits_shared_factors() {
        let mut budget = unlimited();
        let basis = coprime_basis(&[big(12), big(18), big(1), big(35)], &mut budget).unwrap();
        let product: UBig = basis.iter().fold(UBig::ONE, |total, atom| total * atom);
        assert!(product >= big(6));
        for (i, a) in basis.iter().enumerate() {
            for b in basis.iter().skip(i + 1) {
                assert_eq!(a.gcd(b), UBig::ONE);
            }
        }
        assert!(budget.gcd_operations > 0);
    }
}
