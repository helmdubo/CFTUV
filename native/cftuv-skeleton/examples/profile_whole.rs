//! Where the time of a whole `build_skeleton` goes: `cargo run --release --features profile --example profile_whole -p cftuv-skeleton -- <polygons.txt> [repeat]`.
//!
//! The file holds polygons of the field corpus in a plain text (written by the harness that dumps a record's polygon): `polygon <name> <mesh> level_budget <n>`, one `loop x,y;x,y;... |
//! n/d;n/d;...` per loop (points, then the squared speed of every edge), one `fan x,y | nx,ny,n/d;...` per vertex fan, `end`. Each polygon is built `repeat` times on a cold memory
//! (as a preparation starts) and the phases of `profile` are printed (the time of every named phase, `other` being the glue and the exact arithmetic not named).

use std::time::Instant;

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SignCounts;
use cftuv_skeleton::builder::BuilderOptions;
use cftuv_skeleton::polygon::{FanSupport, Loop, Polygon, VertexFan};
use cftuv_skeleton::profile::{reset, take, PHASES};
use cftuv_skeleton::transaction::build_skeleton;

fn rat(text: &str) -> Rat {
    let (numerator, denominator) = text.split_once('/').expect("a speed is n/d");
    Rat::new(numerator.parse::<IBig>().expect("a numerator"), denominator.parse::<IBig>().expect("a denominator")).expect("a denominator that is not zero")
}

fn point(text: &str) -> (i64, i64) {
    let (x, y) = text.split_once(',').expect("a point is x,y");
    (x.parse().expect("an integer x"), y.parse().expect("an integer y"))
}

fn main() {
    let arguments: Vec<String> = std::env::args().collect();
    let text = std::fs::read_to_string(&arguments[1]).expect("the polygons file");
    let repeat: usize = arguments.get(2).map_or(3, |found| found.parse().expect("repeat"));
    let mut name = String::new();
    let mut limit = 0;
    let (mut loops, mut fans): (Vec<Loop>, Vec<VertexFan>) = (Vec::new(), Vec::new());
    for line in text.lines() {
        let mut words = line.splitn(2, ' ');
        match (words.next(), words.next()) {
            (Some("polygon"), Some(rest)) => {
                let parts: Vec<&str> = rest.split(' ').collect();
                name = format!("{} {}", parts[0], parts[1]);
                limit = parts[3].parse().expect("a level budget");
            }
            (Some("loop"), Some(rest)) => {
                let (points, speeds) = rest.split_once(" | ").expect("points | speeds");
                loops.push(Loop { points: points.split(';').map(point).collect(), speeds: speeds.split(';').map(rat).collect() });
            }
            (Some("fan"), Some(rest)) => {
                let (at, supports) = rest.split_once(" | ").expect("point | supports");
                let supports = supports
                    .split(';')
                    .map(|support| {
                        let parts: Vec<&str> = support.splitn(3, ',').collect();
                        FanSupport { normal: (parts[0].parse().unwrap(), parts[1].parse().unwrap()), speed: rat(parts[2]) }
                    })
                    .collect();
                fans.push(VertexFan { point: point(at), supports });
            }
            (Some("end"), _) => {
                let polygon = Polygon::new(std::mem::take(&mut loops), std::mem::take(&mut fans)).expect("a polygon of the corpus");
                let mut best = f64::MAX;
                let mut profile: Vec<u64> = Vec::new();
                let mut nodes = 0;
                for _ in 0..repeat {
                    let (mut memory, mut budget, mut counts, mut products) = (CanonMemory::new(), WorkBudget::unlimited(), SignCounts::default(), ProductMemo::new());
                    let mut ctx = ExactCtx { memory: &mut memory, budget: &mut budget, counts: &mut counts, products: &mut products };
                    reset();
                    let started = Instant::now();
                    let skeleton = build_skeleton(&mut ctx, polygon.clone(), BuilderOptions::default(), limit).expect("a polygon of the corpus builds");
                    let seconds = started.elapsed().as_secs_f64();
                    nodes = skeleton.nodes.len();
                    if seconds < best {
                        best = seconds;
                        profile = take();
                    }
                }
                println!("{name}: {nodes} nodes, best of {repeat}: {:.1} ms", best * 1e3);
                let total: u64 = profile.iter().sum();
                let mut rows: Vec<(&str, u64)> = PHASES.iter().zip(&profile).map(|((label, _), nanoseconds)| (*label, *nanoseconds)).collect();
                rows.sort_by_key(|row| std::cmp::Reverse(row.1));
                for (label, nanoseconds) in rows.iter().filter(|row| row.1 > 0).take(14) {
                    println!("    {label:34} {:9.2} ms {:5.1} %", *nanoseconds as f64 * 1e-6, 100.0 * *nanoseconds as f64 / total.max(1) as f64);
                }
            }
            _ => {}
        }
    }
}
