//! Times a text program (the grammar of `program.rs`, one instruction per line) on the crate alone, without Python:
//!
//!     python tools/native_radical_v2_corpus.py program --seed --out program.txt
//!     cargo run --release -p cftuv-radical --example replay -- program.txt [passes] [inner]
//!
//! Prints, per instruction name, the number of executions and the mean / median / 95th percentile nanoseconds per execution (the best of `passes` passes, each instruction executed `inner`
//! times in a row on the outcomes before it), and the time of one plain run of the whole program.

use std::collections::BTreeMap;
use std::time::Instant;

use cftuv_radical::program::{parse_instruction, run, run_timed};

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

fn main() {
    let mut arguments = std::env::args().skip(1);
    let path = arguments.next().expect("usage: replay PROGRAM_FILE [passes] [inner]");
    let passes: usize = arguments.next().map_or(3, |text| text.parse().expect("passes"));
    let inner: usize = arguments.next().map_or(8, |text| text.parse().expect("inner"));
    let text = std::fs::read_to_string(&path).expect("the program file");
    let program: Vec<_> = text.lines().enumerate().map(|(number, line)| parse_instruction(line).unwrap_or_else(|why| panic!("line {}: {why}", number + 1))).collect();
    let started = Instant::now();
    let done = run(&program);
    println!("{} instructions, one plain run {:.3} s ({} outcomes)", program.len(), started.elapsed().as_secs_f64(), done.len());
    let (_, timings) = run_timed(&program, passes, inner);
    let mut by_name: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    for (line, nanoseconds) in text.lines().zip(&timings) {
        by_name.entry(line.split(' ').next().unwrap_or("").to_string()).or_default().push(*nanoseconds);
    }
    println!("{:<14}{:>8}{:>12}{:>12}{:>12}   (ns per execution)", "op", "n", "mean", "p50", "p95");
    for (name, mut values) in by_name {
        values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mean = values.iter().sum::<f64>() / values.len() as f64;
        println!("{name:<14}{:>8}{mean:>12.0}{:>12.0}{:>12.0}", values.len(), values[values.len() / 2], values[(values.len() * 95 / 100).min(values.len() - 1)]);
    }
}
