//! Timing of the cost operations on operands a Python harness dumped as a cost script (`cargo run --release
//! --example exact_bench -p cftuv-core -- <script.bin> [passes]`): whole-script time per operation and, for the
//! divisions, the split of the conjugation loop into its steps. The script carries an empty sync and no budget,
//! so pass one pays the memory misses and the later passes are the steady state.

use std::time::{Duration, Instant};

use cftuv_core::codec::{Reader, Value};
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::num::UBig;
use cftuv_core::products::ProductMemo;
use cftuv_core::script::run_script;
use cftuv_core::session::Session;
use cftuv_core::sqrt_sum::{conjugate_items, multiply_integer_items, reduce_in_place, scaled_by_reciprocal, SignCounts, SqrtSum};
use cftuv_canon::{CanonMemory, WorkBudget};

fn operands(bytes: &[u8]) -> Vec<(u8, Vec<Value>)> {
    let mut reader = Reader::new(&bytes[5..], true);
    let _header = reader.get_value().unwrap();
    let count = reader.get_uint().unwrap();
    let mut ops = Vec::new();
    for _ in 0..count {
        let code = reader.get_u8().unwrap();
        let argc = reader.get_uint().unwrap();
        ops.push((code, (0..argc).map(|_| reader.get_value().unwrap()).collect()));
    }
    ops
}

fn sum_of(value: &Value) -> &SqrtSum {
    match value {
        Value::Sum(sum) => sum,
        other => panic!("a sum, got {other:?}"),
    }
}

fn main() {
    let mut arguments = std::env::args().skip(1);
    let path = arguments.next().expect("script file");
    let passes: usize = arguments.next().map_or(5, |text| text.parse().unwrap());
    let bytes = std::fs::read(&path).unwrap();
    let ops = operands(&bytes);
    let mut session = Session::new();
    for pass in 0..passes {
        let started = Instant::now();
        let response = run_script(&mut session, &bytes).unwrap();
        let elapsed = started.elapsed();
        println!("pass {pass}: {} ops, {:.1} us/op (script incl. buffer decode/encode), response {} bytes", ops.len(), elapsed.as_secs_f64() * 1e6 / ops.len() as f64, response.len());
    }
    // pure compute on decoded operands, warm memory
    let mut memory = CanonMemory::new();
    let mut budget = WorkBudget::unlimited();
    let mut counts = SignCounts::default();
    let mut products = ProductMemo::new();
    let mut pass_time = |label: &str, run: &mut dyn FnMut(&mut ExactCtx<'_>)| {
        for round in 0..3 {
            let started = Instant::now();
            let mut ctx = ExactCtx { memory: &mut memory, budget: &mut budget, counts: &mut counts, products: &mut products };
            run(&mut ctx);
            if round > 0 {
                println!("{label}: warm compute {:.1} us/op", started.elapsed().as_secs_f64() * 1e6 / ops.len() as f64);
            }
        }
    };
    match ops[0].0 {
        71 => pass_time("divided_by", &mut |ctx| {
            for (_, args) in &ops {
                exact::divided_by(ctx, sum_of(&args[0]), sum_of(&args[1])).unwrap();
            }
        }),
        72 => pass_time("divide_with_prime_universe", &mut |ctx| {
            for (_, args) in &ops {
                let Value::List(universe) = &args[2] else { panic!("universe") };
                let universe: Vec<UBig> = universe.iter().map(|value| match value { Value::Int(n) => cftuv_core::num::magnitude(n), _ => panic!("int") }).collect();
                exact::divide_with_prime_universe(ctx, sum_of(&args[0]), sum_of(&args[1]), &universe).unwrap();
            }
        }),
        70 => pass_time("sign", &mut |ctx| {
            for (_, args) in &ops {
                exact::sign(ctx, sum_of(&args[0]), 64).unwrap();
            }
        }),
        _ => {}
    }
    if ops[0].0 == 71 || ops[0].0 == 72 {
        loop_profile(&ops, &mut memory, &mut budget);
    }
}

/// The conjugation loop of `divided_by`, step by step, with the time of each kind of step.
fn loop_profile(ops: &[(u8, Vec<Value>)], memory: &mut CanonMemory, budget: &mut WorkBudget) {
    let (mut forms, mut multiplying, mut reducing, mut finishing, mut picking) = (Duration::ZERO, Duration::ZERO, Duration::ZERO, Duration::ZERO, Duration::ZERO);
    let mut rounds = 0usize;
    let mut products = ProductMemo::new();
    let mut counts = SignCounts::default();
    for _ in 0..2 {
        (forms, multiplying, reducing, finishing, picking, rounds) = (Duration::ZERO, Duration::ZERO, Duration::ZERO, Duration::ZERO, Duration::ZERO, 0);
        for (_, args) in ops {
            let started = Instant::now();
            let (numerator, denominator) = (sum_of(&args[0]), sum_of(&args[1]));
            let (mut nc, mut ni) = (numerator.int_form().common.clone(), numerator.int_form().items.clone());
            let (mut dc, mut di) = (denominator.int_form().common.clone(), denominator.int_form().items.clone());
            forms += started.elapsed();
            loop {
                if di.len() <= 1 && di.iter().all(|(radicand, _)| radicand.is_one()) {
                    let started = Instant::now();
                    scaled_by_reciprocal(&nc, &ni, &dc, &di).unwrap();
                    finishing += started.elapsed();
                    break;
                }
                let started = Instant::now();
                let mut ctx = ExactCtx { memory, budget, counts: &mut counts, products: &mut products };
                let prime = exact::pick_prime(&mut ctx, &di).unwrap().unwrap();
                ctx.memory.squarefree_split_unsigned(&prime, ctx.budget).unwrap();
                picking += started.elapsed();
                rounds += 1;
                let started = Instant::now();
                let conjugate = conjugate_items(&di, &prime);
                ni = multiply_integer_items(&ni, &conjugate, &mut products);
                di = multiply_integer_items(&di, &conjugate, &mut products);
                nc = &nc * &dc;
                dc = &dc * &dc;
                multiplying += started.elapsed();
                let started = Instant::now();
                reduce_in_place(&mut nc, &mut ni);
                reduce_in_place(&mut dc, &mut di);
                reducing += started.elapsed();
            }
        }
    }
    let per = |time: Duration| time.as_secs_f64() * 1e6 / ops.len() as f64;
    println!(
        "loop profile ({} ops, {} rounds): integer forms {:.1} us/op, pick prime + split {:.1}, multiply {:.1}, reduce {:.1}, final fractions {:.1}",
        ops.len(),
        rounds,
        per(forms),
        per(picking),
        per(multiplying),
        per(reducing),
        per(finishing)
    );
}
