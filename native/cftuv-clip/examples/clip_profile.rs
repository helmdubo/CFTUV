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
//!
//! `--chain` runs the files as the steps of one session, once each and in the order given, with ONE cross-call cache shared by the
//! calls (`python tools/native_clip_geometry.py chain <dir> <mesh> <patch>` writes the neighbouring alphas of a patch in order):
//! the total compute of the chain and its phases summed (`--cold`: the same chain with no cache between the calls). With `--features profile` the phase
//! table is printed.

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
    let mut chain = false;
    let mut cold = false;
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == "--cold" {
            cold = true;
        } else if argument == "--chain" {
            chain = true;
        } else if argument == "--calls" {
            calls = arguments.next().and_then(|text| text.parse().ok()).expect("--calls N");
        } else {
            files.push(argument);
        }
    }
    if chain {
        // one call per file, in the order given, on one thread with ONE warm cache shared by the calls (a session across the steps of a width slider);
        // phases summed over the chain
        if !cold {
            cftuv_clip::geometry_seam::enable_warm();
        }
        let mut totals: std::collections::BTreeMap<&'static str, (u64, u64)> = std::collections::BTreeMap::new();
        let mut sum_ns = 0u64;
        for file in &files {
            let request = std::fs::read(file).expect("a readable request");
            let mut session = Session::new();
            profile::take();
            let response = seam::run(&mut session, &request).expect("the seam answers");
            let total = compute_nanoseconds(&response);
            sum_ns += total;
            for (name, count, exclusive, _inclusive) in profile::take() {
                let entry = totals.entry(name).or_default();
                entry.0 += count;
                entry.1 += exclusive;
            }
            println!("  {} {:.2} ms", file.rsplit('/').next().unwrap_or(file), total as f64 * 1e-6);
        }
        let mut rows: Vec<_> = totals.into_iter().filter(|row| row.1 .1 > 0).collect();
        rows.sort_by_key(|row| std::cmp::Reverse(row.1 .1));
        println!("chain total {:.1} ms", sum_ns as f64 * 1e-6);
        for (name, (count, exclusive)) in rows.iter().take(22) {
            println!("  {name:<34} {count:>9} {:>10.1} us {:>6.1}%", *exclusive as f64 * 1e-3, 100.0 * *exclusive as f64 / sum_ns as f64);
        }
        return;
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
