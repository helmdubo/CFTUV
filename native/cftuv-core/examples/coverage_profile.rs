//! Where the time of the native `_coverage_at` goes, phase by phase, on a real partition a Python harness dumped
//! (`python tools/native_bench_native.py --dump <record id> --dump-to <file>`).
//! Run: `cargo run --release --example coverage_profile -p cftuv-core -- <file> [warm calls]`.
//!
//! It runs the real `coverage::coverage_at` (one cold call that builds the prime universe, then warm calls at other
//! alphas) and prints the phase timers the operation keeps while profiling is on (`coverage::set_profiling`).

use std::time::Instant;

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

use cftuv_canon::{QValue, WorkBudget};
use cftuv_core::codec::{Reader, Value};
use cftuv_core::coverage::{self, Face, Line, Partition, Point};
use cftuv_core::exact::{ExactCtx, UniverseStore};
use cftuv_core::num::IBig;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SignCounts;

fn rat_of(value: &Value) -> Rat {
    match value {
        Value::Int(number) => Rat::from_int(number.clone()),
        Value::Frac(rat) => rat.clone(),
        other => panic!("a number, got {other:?}"),
    }
}

fn load(path: &str) -> (Rat, Partition) {
    let bytes = std::fs::read(path).unwrap();
    let Value::List(top) = Reader::new(&bytes, true).get_value().unwrap() else { panic!("list") };
    let alpha = rat_of(&top[0]);
    let Value::List(faces) = &top[1] else { panic!("faces") };
    let mut out = Vec::new();
    for face in faces {
        let Value::List(parts) = face else { panic!("face") };
        let Value::List(points) = &parts[0] else { panic!("points") };
        let Value::List(line) = &parts[1] else { panic!("line") };
        let points: Vec<Point> = points
            .iter()
            .map(|point| match point {
                Value::List(pair) => match (&pair[0], &pair[1]) {
                    (Value::Sum(x), Value::Sum(y)) => (x.clone(), y.clone()),
                    _ => panic!("sums"),
                },
                _ => panic!("point"),
            })
            .collect();
        out.push(Face::new(points, Some(Line { a: rat_of(&line[0]), b: rat_of(&line[1]), c: rat_of(&line[2]), q: rat_of(&line[3]) })));
    }
    (alpha, Partition::new(true, out))
}

fn main() {
    let mut arguments = std::env::args().skip(1);
    let path = arguments.next().expect("dump file");
    let calls: usize = arguments.next().map_or(40, |text| text.parse().unwrap());
    let (alpha, partition) = load(&path);
    let mut memory = cftuv_canon::CanonMemory::new();
    let mut budget = WorkBudget::unlimited();
    let mut counts = SignCounts::default();
    let mut products = ProductMemo::new();
    let q_values: Vec<QValue> = partition.q_values().to_vec();
    // the real operation, cold then warm
    let started = Instant::now();
    let record = {
        let mut ctx = ExactCtx { memory: &mut memory, budget: &mut budget, counts: &mut counts, products: &mut products };
        let run = coverage::coverage_at(&mut ctx, &partition, &alpha, UniverseStore::Miss, true);
        assert!(run.outcome.is_ok());
        run.record.expect("a miss makes a record")
    };
    println!("cold coverage_at (conversion excluded; prime universe miss {} q values): {:.3} ms", q_values.len(), started.elapsed().as_secs_f64() * 1e3);
    let factors: Vec<Rat> = [(15, 16), (17, 16), (7, 8), (9, 8), (3, 4), (5, 4), (1, 2)].iter().map(|(n, d)| Rat::new(IBig::from(*n), IBig::from(*d)).unwrap()).collect();
    let mut best = f64::MAX;
    let mut sum = 0.0;
    for call in 0..calls {
        let alpha = alpha.mul(&factors[call % factors.len()]);
        let started = Instant::now();
        {
            let mut ctx = ExactCtx { memory: &mut memory, budget: &mut budget, counts: &mut counts, products: &mut products };
            let run = coverage::coverage_at(&mut ctx, &partition, &alpha, UniverseStore::Hit(&record), true);
            std::hint::black_box(&run);
        }
        let elapsed = started.elapsed().as_secs_f64() * 1e3;
        best = best.min(elapsed);
        sum += elapsed;
    }
    println!("warm coverage_at: best {:.3} ms, mean {:.3} ms over {calls} calls", best, sum / calls as f64);
    // the phases of the real operation, on the same calls
    coverage::set_profiling(true);
    for call in 0..calls {
        let alpha = alpha.mul(&factors[call % factors.len()]);
        let mut ctx = ExactCtx { memory: &mut memory, budget: &mut budget, counts: &mut counts, products: &mut products };
        std::hint::black_box(coverage::coverage_at(&mut ctx, &partition, &alpha, UniverseStore::Hit(&record), true));
    }
    coverage::set_profiling(false);
    let per = |slot: usize| coverage::PHASE_NANOS[slot].load(std::sync::atomic::Ordering::Relaxed) as f64 / 1e3 / calls as f64;
    let phases: Vec<String> = coverage::PHASES.iter().enumerate().map(|(slot, name)| format!("{name} {:.0}", per(slot))).collect();
    println!("per warm call (us), real operation: {}", phases.join("  "));
}
