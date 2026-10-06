//! Where the time of a sqrt-sum operation goes, on operands of the size the corpus holds (5 terms, ~56-bit
//! radicands built from a small prime universe, ~200-bit numerators and denominators).
//! Run: `cargo run --release --example sqrt_sum_bench -p cftuv-core`.
//!
//! It answers two questions for the whole-operation ports: how much of `mul` is the final per-term
//! normalisation (`Fraction(numerator, denominator)`), and whether the radicand-product memory pays.
//!
//! Measured (2026-10-06, release): `mul` of two 5-term sums with 200-bit coefficients takes 92-100 us, of which the
//! integer products are 11-13 us; the per-term normalisation (a gcd over a ~1000-bit common denominator for each of
//! ~15 result terms) is almost all the rest. The memory of radicand products makes no measurable difference for
//! 20-40-bit radicands and saves about 9% of `mul` for 100-250-bit radicands (so it stays, on by default). The
//! speed-up of a whole operation therefore cannot come from a faster `mul`: it has to come from keeping values as
//! unreduced integer forms (`IntForm`) inside the operation and normalising once, at the boundary
//! (`IntForm::into_sqrt_sum`).

use std::hint::black_box;
use std::time::Instant;

use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::{Coef, Rat};
use cftuv_core::sqrt_sum::{multiply_integer_items, SqrtSum, Term};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn big(&mut self, bits: usize) -> UBig {
        let bytes: Vec<u8> = (0..bits.div_ceil(8)).map(|_| self.next() as u8).collect();
        let value = UBig::from_le_bytes(&bytes);
        (value >> (bytes.len() * 8 - bits)) | (UBig::ONE << (bits - 1))
    }
}

const SMALL_PRIMES: [u64; 12] = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37];
/// Primes near 2^34 .. 2^40: products of about half of them are the 200-bit radicands of the heavy domains.
const LARGE_PRIMES: [u64; 12] = [17179869143, 34359738337, 68719476731, 137438953447, 274877906899, 549755813881, 1099511627689, 2147483647, 4294967291, 1000000007, 998244353, 1000000009];

fn random_sum(rng: &mut Rng, terms: usize, primes: &[u64]) -> SqrtSum {
    let mut radicands: Vec<UBig> = Vec::new();
    while radicands.len() < terms {
        let mut product = UBig::ONE;
        for &prime in primes {
            if rng.next().is_multiple_of(3) {
                product *= UBig::from(prime);
            }
        }
        if !radicands.contains(&product) {
            radicands.push(product);
        }
    }
    radicands.sort();
    let terms = radicands
        .into_iter()
        .map(|radicand| {
            let numerator = IBig::from(rng.big(200)) * if rng.next().is_multiple_of(2) { IBig::ONE } else { IBig::NEG_ONE };
            let denominator = rng.big(200);
            Term { radicand, coef: Coef::fraction(Rat::reduced(numerator, denominator)) }
        })
        .collect();
    SqrtSum::from_terms(terms).unwrap()
}

fn per_op<F: FnMut()>(iterations: usize, mut body: F) -> f64 {
    for _ in 0..iterations.min(200) {
        body();
    }
    let start = Instant::now();
    for _ in 0..iterations {
        body();
    }
    start.elapsed().as_nanos() as f64 / iterations as f64 / 1000.0
}

fn main() {
    println!("-- radicands from the 12 smallest primes (about 20-40 bits)");
    report(&SMALL_PRIMES);
    println!("-- radicands from 12 primes of 30-40 bits (about 100-250 bits)");
    report(&LARGE_PRIMES);
}

fn report(primes: &[u64]) {
    let mut rng = Rng(0x1234_5678_9ABC_DEF1);
    let pairs: Vec<(SqrtSum, SqrtSum)> = (0..64).map(|_| (random_sum(&mut rng, 5, primes), random_sum(&mut rng, 5, primes))).collect();
    let mut index = 0usize;
    let mut next_pair = || {
        index = (index + 1) % pairs.len();
        &pairs[index]
    };
    let mut warm = ProductMemo::new();
    let mut cold = ProductMemo::disabled();
    let mul_warm = per_op(20_000, || {
        let (a, b) = next_pair();
        black_box(a.mul(b, &mut warm));
    });
    let mul_cold = per_op(20_000, || {
        let (a, b) = next_pair();
        black_box(a.mul(b, &mut cold));
    });
    let items = per_op(20_000, || {
        let (a, b) = next_pair();
        black_box(multiply_integer_items(&a.int_form().items, &b.int_form().items, &mut warm));
    });
    let mut normalised = 0usize;
    let normalise = per_op(20_000, || {
        let (a, b) = next_pair();
        let form = a.int_form();
        let denominator = &form.common * &b.int_form().common;
        for (_, numerator) in &a.int_form().items {
            normalised += 1;
            black_box(Rat::reduced(numerator * numerator, denominator.clone()));
        }
    });
    let gcd = per_op(20_000, || {
        let (a, b) = next_pair();
        black_box(num::gcd(&a.int_form().common, &b.int_form().common));
    });
    println!("mul (5x5 terms, ~200-bit coefficients): memo {mul_warm:.1} us, no memo {mul_cold:.1} us");
    println!("  integer product only (no normalisation): {items:.1} us");
    println!("  one normalisation of the 5 numerators (reduced over a ~400-bit denominator): {normalise:.1} us");
    println!("  gcd of the two common denominators: {gcd:.2} us ({normalised} normalised numerators)");
}
