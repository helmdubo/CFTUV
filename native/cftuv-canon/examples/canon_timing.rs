//! Micro-timing of the budget unit: nanoseconds per charged unit of a Brent orbit (steps, gcds, pre-paid
//! squarings) and of a Miller-Rabin run, on 120- and 250-bit moduli.
//!
//! `cargo run --release -p cftuv-canon --example canon_timing` (the Python side:
//! `PYTHONSAFEPATH=1 python tools/native_canon_vectors.py --timing`).

use std::time::Instant;

use cftuv_canon::factor::{brent_plain, is_prime, is_prime_plain, pollard_rho_brent_attempt};
use cftuv_canon::pyrandom::PyRandom;
use cftuv_canon::WorkBudget;
use dashu_int::UBig;

fn random_prime(bits: u32, rng: &mut PyRandom) -> UBig {
    loop {
        let candidate = rng.getrandbits(bits) | (UBig::ONE << (bits as usize - 1)) | UBig::ONE;
        if is_prime(&candidate, &mut WorkBudget::unlimited()).unwrap_or(false) {
            return candidate;
        }
    }
}

fn brent_ns_per_unit(n: &UBig, plain: bool, cap: u64) -> (f64, u64) {
    let mut rng = PyRandom::from_seed(n);
    let c = rng.randrange(&UBig::ONE, n).unwrap();
    let y = rng.randrange(&UBig::ZERO, n).unwrap();
    let mut budget = WorkBudget::bounded(cap);
    let started = Instant::now();
    let outcome = if plain {
        brent_plain(n, &y, &c, 64, &mut budget).map_err(|_| ())
    } else {
        pollard_rho_brent_attempt(n, &y, &c, std::env::var("BATCH").ok().and_then(|v| v.parse().ok()).unwrap_or(64), &mut budget).map_err(|_| ())
    };
    let elapsed = started.elapsed().as_secs_f64();
    assert!(outcome.is_err(), "the attempt was expected to run into the cap");
    (elapsed * 1e9 / budget.spent() as f64, budget.spent())
}

fn mr_ns_per_unit(p: &UBig, plain: bool, repeats: u32) -> (f64, u64) {
    let n_minus_one = p - UBig::ONE;
    let r = n_minus_one.trailing_zeros().unwrap_or(0) as u64;
    let d = &n_minus_one >> (r as usize);
    let mut spent = 0u64;
    let started = Instant::now();
    for _ in 0..repeats {
        let mut budget = WorkBudget::unlimited();
        let verdict = if plain { is_prime_plain(p, &d, r, &mut budget) } else { is_prime(p, &mut budget) };
        assert_eq!(verdict, Ok(true));
        spent += budget.spent();
    }
    (started.elapsed().as_secs_f64() * 1e9 / spent as f64, spent / u64::from(repeats))
}

fn main() {
    let mut rng = PyRandom::from_seed(&UBig::from(20_261_006u32));
    println!("{:<34} {:>12} {:>12} {:>10}", "case", "ns/unit", "units/call", "path");
    for (label, half_bits) in [("120-bit", 60u32), ("250-bit", 125u32)] {
        let n = random_prime(half_bits, &mut rng) * random_prime(half_bits, &mut rng);
        for plain in [false, true] {
            let (ns, units) = brent_ns_per_unit(&n, plain, 1_000_000);
            println!("{:<34} {:>12.1} {:>12} {:>10}", format!("brent orbit {label} semiprime"), ns, units, if plain { "dashu" } else { "montgomery" });
        }
        let prime = random_prime(half_bits * 2, &mut rng);
        for plain in [false, true] {
            let (ns, units) = mr_ns_per_unit(&prime, plain, 200);
            println!("{:<34} {:>12.1} {:>12} {:>10}", format!("miller-rabin {label} prime"), ns, units, if plain { "dashu" } else { "montgomery" });
        }
    }
}
