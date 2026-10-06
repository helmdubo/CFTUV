//! Day-1 microbench behind the choice of the big-integer representation (dashu-int vs num-bigint).
//!
//! Sizes are the ones the kernel's exact arithmetic actually uses: gcd of ~300-bit numbers (fraction
//! normalisation), 600x600-bit products (sqrt-sum products over a common denominator), 1000/500-bit division
//! with remainder, 250-bit modular multiplication (the budgeted Pollard orbit) and the integer square root of
//! `n << 128` (the enclosure). Run: `cargo run --release --example bigint_bench -p cftuv-core`.
//!
//! Decision (2026-10-06, Windows 11, rustc 1.99, CPython 3.13.1; mean ns per op, interpreter loop overhead of
//! about 70 ns not subtracted from the CPython column): dashu-int wins or ties on every row, so it is the
//! representation (`UBig`/`IBig`); num-bigint is only kept here as the comparison point.
//!
//! ```text
//! op                    dashu   num-bigint   CPython
//! gcd 300-bit             815         4990      1231
//! mul 600x600             145          158       550
//! divrem 1000/500         364          333       732
//! mulmod 250              226          291       559
//! isqrt 260<<128          303         1196       768
//! gcd 60-bit               53         1093       311
//! gcd 120-bit             184         1877       520
//! gcd 1000-bit           3241        18843      4760
//! mul 60x60                 6           45       151
//! divrem 120/60             9           46       226
//! ```
//!
//! The per-operation gain over CPython is only 1.4x..3.3x on the large rows: the speed-up of a whole kernel
//! operation has to come from removing the interpreter around the arithmetic (no per-op boundary, no object
//! churn), not from a faster bignum.

use std::hint::black_box;
use std::time::Instant;

use dashu_int::ops::{DivRem, Gcd, SquareRoot};
use dashu_int::UBig;
use num_bigint::BigUint;
use num_integer::Integer;

const POOL: usize = 64;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// Random bytes of an exact bit length (top bit set), little-endian.
    fn bytes(&mut self, bits: usize) -> Vec<u8> {
        let n = bits.div_ceil(8);
        let mut out: Vec<u8> = (0..n).map(|_| self.next() as u8).collect();
        let top = (bits - 1) % 8;
        out[n - 1] &= (1u16 << (top + 1)).wrapping_sub(1) as u8;
        out[n - 1] |= 1 << top;
        out
    }
}

fn pools(rng: &mut Rng, bits: usize) -> (Vec<UBig>, Vec<BigUint>) {
    let raw: Vec<Vec<u8>> = (0..POOL).map(|_| rng.bytes(bits)).collect();
    (
        raw.iter().map(|b| UBig::from_le_bytes(b)).collect(),
        raw.iter().map(|b| BigUint::from_bytes_le(b)).collect(),
    )
}

fn per_op<F: FnMut(usize)>(iterations: usize, mut body: F) -> f64 {
    for index in 0..iterations.min(2000) {
        body(index);
    }
    let start = Instant::now();
    for index in 0..iterations {
        body(index);
    }
    start.elapsed().as_nanos() as f64 / iterations as f64
}

fn row(name: &str, dashu: f64, num: f64) {
    println!("{name:<34} dashu {dashu:>9.0} ns   num-bigint {num:>9.0} ns   ratio num/dashu {:>5.2}", num / dashu);
}

fn bench_gcd(rng: &mut Rng, bits: usize, iterations: usize) {
    let (da, na) = pools(rng, bits);
    let (db, nb) = pools(rng, bits);
    let dashu = per_op(iterations, |i| {
        black_box(black_box(&da[i % POOL]).gcd(black_box(&db[(i * 7 + 3) % POOL])));
    });
    let num = per_op(iterations, |i| {
        black_box(black_box(&na[i % POOL]).gcd(black_box(&nb[(i * 7 + 3) % POOL])));
    });
    row(&format!("gcd {bits}-bit"), dashu, num);
}

fn bench_mul(rng: &mut Rng, bits: usize, iterations: usize) {
    let (da, na) = pools(rng, bits);
    let (db, nb) = pools(rng, bits);
    let dashu = per_op(iterations, |i| {
        black_box(black_box(&da[i % POOL]) * black_box(&db[(i * 7 + 3) % POOL]));
    });
    let num = per_op(iterations, |i| {
        black_box(black_box(&na[i % POOL]) * black_box(&nb[(i * 7 + 3) % POOL]));
    });
    row(&format!("mul {bits}x{bits}-bit"), dashu, num);
}

fn bench_divrem(rng: &mut Rng, big: usize, small: usize, iterations: usize) {
    let (da, na) = pools(rng, big);
    let (db, nb) = pools(rng, small);
    let dashu = per_op(iterations, |i| {
        black_box(black_box(&da[i % POOL]).div_rem(black_box(&db[(i * 7 + 3) % POOL])));
    });
    let num = per_op(iterations, |i| {
        black_box(black_box(&na[i % POOL]).div_rem(black_box(&nb[(i * 7 + 3) % POOL])));
    });
    row(&format!("divrem {big}/{small}-bit"), dashu, num);
}

fn bench_mulmod(rng: &mut Rng, bits: usize, iterations: usize) {
    let (da, na) = pools(rng, bits);
    let (db, nb) = pools(rng, bits);
    let (dn, nn) = pools(rng, bits);
    let dashu = per_op(iterations, |i| {
        let product = black_box(&da[i % POOL]) * black_box(&db[(i * 7 + 3) % POOL]);
        black_box(product % black_box(&dn[(i * 5 + 1) % POOL]));
    });
    let num = per_op(iterations, |i| {
        let product = black_box(&na[i % POOL]) * black_box(&nb[(i * 7 + 3) % POOL]);
        black_box(product % black_box(&nn[(i * 5 + 1) % POOL]));
    });
    row(&format!("mulmod {bits}-bit"), dashu, num);
}

fn bench_isqrt(rng: &mut Rng, bits: usize, shift: usize, iterations: usize) {
    let (da, na) = pools(rng, bits);
    let dashu = per_op(iterations, |i| {
        let value = black_box(&da[i % POOL]) << shift;
        black_box(value.sqrt());
    });
    let num = per_op(iterations, |i| {
        let value = black_box(&na[i % POOL]) << shift;
        black_box(value.sqrt());
    });
    row(&format!("isqrt {bits}<<{shift}"), dashu, num);
}

fn main() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    println!("-- corpus-like sizes (mean ns per op, rotating pool of {POOL} operand sets) --");
    bench_gcd(&mut rng, 300, 200_000);
    bench_mul(&mut rng, 600, 200_000);
    bench_divrem(&mut rng, 1000, 500, 200_000);
    bench_mulmod(&mut rng, 250, 200_000);
    bench_isqrt(&mut rng, 260, 128, 100_000);
    println!("-- small and mid sizes (coefficients near the inline limit) --");
    bench_gcd(&mut rng, 60, 500_000);
    bench_gcd(&mut rng, 120, 500_000);
    bench_gcd(&mut rng, 1000, 50_000);
    bench_mul(&mut rng, 60, 500_000);
    bench_mul(&mut rng, 250, 500_000);
    bench_mul(&mut rng, 2000, 50_000);
    bench_divrem(&mut rng, 120, 60, 500_000);
    bench_divrem(&mut rng, 600, 300, 200_000);
    bench_isqrt(&mut rng, 60, 128, 200_000);
}
