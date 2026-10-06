//! Where the time of the native `clip_geometry` goes, phase by phase, on real calls a Python harness dumped
//! (`python tools/native_clip_geometry.py dump <dir> --top 6`: the request of the `CLIP_GEOMETRY` seam of the slowest
//! records, memory state and arguments included).
//!
//! Run: `cargo run --release --example clip_profile -p cftuv-clip -- <request files...> [--calls N]` for plain timings, add
//! `--features profile` for the phase table (inclusive and exclusive nanoseconds per phase of one warm call; the exclusive
//! column sums to the compute time of that call, the scopes cost a little themselves).
//!
//! Timings are the compute time the seam measures inside Rust (arguments decoded, result not yet encoded); the first call
//! of a file is cold (empty product cache), the others warm like a long-lived session.

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

use cftuv_clip::profile;
use cftuv_clip::seam;
use cftuv_core::codec::{Reader, Value};
use cftuv_core::session::Session;

fn compute_nanoseconds(response: &[u8]) -> u64 {
    let Value::List(parts) = Reader::new(response, true).get_value().expect("a decodable answer") else {
        panic!("the answer is a list");
    };
    match parts.last() {
        Some(Value::Int(nanoseconds)) => u64::try_from(nanoseconds).expect("nanoseconds fit"),
        other => panic!("the compute time is the last item, got {other:?}"),
    }
}

fn outcome_label(response: &[u8]) -> String {
    let Value::List(parts) = Reader::new(response, true).get_value().expect("a decodable answer") else {
        panic!("the answer is a list");
    };
    match &parts[0] {
        Value::List(outcome) => format!("{:?}", outcome.first()),
        other => format!("{other:?}"),
    }
}

fn main() {
    let mut files = Vec::new();
    let mut calls = 12usize;
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == "--calls" {
            calls = arguments.next().and_then(|text| text.parse().ok()).expect("--calls N");
        } else {
            files.push(argument);
        }
    }
    for file in &files {
        let request = std::fs::read(file).expect("a readable request");
        let mut session = Session::new();
        let mut times = Vec::new();
        let mut label = String::new();
        for _ in 0..calls {
            let response = seam::run(&mut session, &request).expect("the seam answers");
            label = outcome_label(&response);
            times.push(compute_nanoseconds(&response));
        }
        let cold = times[0];
        let mut warm: Vec<u64> = times[1..].to_vec();
        warm.sort_unstable();
        let median = warm.get(warm.len() / 2).copied().unwrap_or(cold);
        println!("{file}: outcome {label}, cold {:.2} ms, warm min {:.2} ms, median {:.2} ms", cold as f64 * 1e-6, warm.first().copied().unwrap_or(cold) as f64 * 1e-6, median as f64 * 1e-6);
        profile::take();
        let response = seam::run(&mut session, &request).expect("the seam answers");
        let total = compute_nanoseconds(&response);
        let mut rows = profile::take();
        if rows.is_empty() {
            continue;
        }
        rows.retain(|row| row.1 > 0);
        rows.sort_by_key(|row| std::cmp::Reverse(row.2));
        let covered: u64 = rows.iter().map(|row| row.2).sum();
        println!("  profiled call: {:.2} ms (scopes included), exclusive time covered {:.1} %", total as f64 * 1e-6, 100.0 * covered as f64 / total.max(1) as f64);
        println!("  {:<34} {:>9} {:>12} {:>7} {:>12}", "phase", "calls", "exclusive us", "share", "inclusive us");
        for (name, count, exclusive, inclusive) in rows {
            println!("  {name:<34} {count:>9} {:>12.1} {:>6.1}% {:>12.1}", exclusive as f64 * 1e-3, 100.0 * exclusive as f64 / total.max(1) as f64, inclusive as f64 * 1e-3);
        }
    }
}
