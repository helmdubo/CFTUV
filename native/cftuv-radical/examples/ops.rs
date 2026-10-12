//! The cost of one public operation of `Radical` on 60-bit radicands and small fractions (the operands of the field are smaller, the sums of the kernel suite bigger), nanoseconds per call:
//!
//!     cargo run --release -p cftuv-radical --example ops
//!
//! Under mimalloc, the allocator the extension ships with. `replay.rs` times recorded programs instead.

use std::hint::black_box;
use std::time::Instant;

use cftuv_radical::{IBig, Radical, UBig};

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

fn time<F: FnMut()>(label: &str, count: usize, mut f: F) {
    for _ in 0..count / 10 {
        f();
    }
    let started = Instant::now();
    for _ in 0..count {
        f();
    }
    println!("{label:<52}{:>9.1} ns", started.elapsed().as_nanos() as f64 / count as f64);
}

fn pair(r: u64, n: i64, d: u64) -> (IBig, IBig, UBig) {
    (IBig::from(r), IBig::from(n), UBig::from(d))
}

fn main() {
    let n = 400_000;
    let r60 = 1_152_921_504_606_846_977u64; // not a square
    let one = vec![pair(r60, 1, 1)];
    let frac = vec![pair(r60, 3, 5)];
    let hidden = vec![pair(r60 / 7 * 7 * 9, 3, 10)];
    time("from_terms [(r60, 1, 1)]", n, || {
        black_box(Radical::from_terms(black_box(&one)).unwrap());
    });
    time("from_terms [(r60, 3, 5)]", n, || {
        black_box(Radical::from_terms(black_box(&frac)).unwrap());
    });
    time("from_terms [(r60 hidden square, 3, 10)]", n, || {
        black_box(Radical::from_terms(black_box(&hidden)).unwrap());
    });
    let a = Radical::from_terms(&frac).unwrap().0;
    let b = Radical::from_terms(&[pair(r60, 7, 3)]).unwrap().0;
    let c = Radical::from_terms(&[pair(r60 + 2, 7, 3)]).unwrap().0;
    time("clone of a one-term value", n, || {
        black_box(black_box(&a).clone());
    });
    time("neg", n, || {
        black_box(black_box(&a).neg());
    });
    time("add same class", n, || {
        black_box(black_box(&a).add(black_box(&b)));
    });
    time("add different class (one check)", n, || {
        black_box(black_box(&a).add(black_box(&c)));
    });
    time("mul same class (rational result)", n, || {
        black_box(black_box(&a).mul(black_box(&b)));
    });
    time("mul different class", n, || {
        black_box(black_box(&a).mul(black_box(&c)));
    });
    time("signum of one term", n, || {
        black_box(black_box(&a).signum(None).unwrap());
    });
    time("inverse of one term", n, || {
        black_box(black_box(&a).inverse().unwrap());
    });
    let s = a.add(&c).0;
    let t = b.add(&Radical::rational(IBig::from(5), UBig::from(3u8))).0;
    time("add two-term + two-term", n, || {
        black_box(black_box(&s).add(black_box(&t)));
    });
    time("mul two-term x two-term", n, || {
        black_box(black_box(&s).mul(black_box(&t)));
    });
    let three = s.add(&Radical::rational(IBig::from(5), UBig::from(3u8))).0;
    time("signum of three classes", n / 4, || {
        black_box(black_box(&three).signum(None).unwrap());
    });
    time("text of one term", n, || {
        black_box(black_box(&a).text());
    });
    time("wire of one term", n, || {
        black_box(black_box(&a).wire());
    });
}
