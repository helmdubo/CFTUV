//! Call sequences over the canonicalization memory against the Python oracle: after EVERY call the answer, the six
//! budget articles (and the unbudgeted telemetry), and the full ordered memory (or its digest at the 8192-entry
//! thresholds); cap sweeps compare the exhaustion point and the partial memory for every cap `0..=total+1`.

mod common;

use std::collections::{BTreeMap, HashMap};

use cftuv_canon::{
    CanonError, CanonMemory, MemOp, MemoryDelta, MemoryMarker, MemoryState, QValue, UniverseRecord, WorkBudget,
};
use common::*;
use dashu_int::UBig;

// ---- JSON encodings shared with tools/native_canon_vectors.py ---------------------------------------------------

fn obj(items: Vec<(&str, Json)>) -> Json {
    Json::Obj(items.into_iter().map(|(key, value)| (key.to_owned(), value)).collect::<BTreeMap<_, _>>())
}

fn hex_list(values: &[UBig]) -> Json {
    jarr(values.iter().map(jhex).collect())
}

fn pairs_json(pairs: &[(UBig, u64)]) -> Json {
    jarr(pairs.iter().map(|(prime, power)| jarr(vec![jhex(prime), jint(*power)])).collect())
}

fn split_json(split: &(UBig, UBig)) -> Json {
    jarr(vec![jhex(&split.0), jhex(&split.1)])
}

fn articles_json(budget: &WorkBudget) -> Json {
    jarr(budget.articles().iter().map(|article| jint(*article)).collect())
}

fn state_json(state: &MemoryState) -> Json {
    obj(vec![
        ("p", hex_list(&state.known_primes)),
        ("f", jarr(state.factorization.iter().map(|(key, pairs)| jarr(vec![jhex(key), pairs_json(pairs)])).collect())),
        ("s", jarr(state.squarefree.iter().map(|(key, split)| jarr(vec![jhex(key), split_json(split)])).collect())),
        ("u", jarr(state.support.iter().map(|(key, support)| jarr(vec![jhex(key), hex_list(support)])).collect())),
    ])
}

fn state_texts(state: &MemoryState) -> [String; 4] {
    let join_pairs = |pairs: &[(UBig, u64)]| pairs.iter().map(|(p, e)| format!("{}^{e}", hx(p))).collect::<Vec<_>>().join(",");
    [
        state.known_primes.iter().map(hx).collect::<Vec<_>>().join(","),
        state.factorization.iter().map(|(key, pairs)| format!("{}:{}", hx(key), join_pairs(pairs))).collect::<Vec<_>>().join(";"),
        state.squarefree.iter().map(|(key, (a, b))| format!("{}:{},{}", hx(key), hx(a), hx(b))).collect::<Vec<_>>().join(";"),
        state
            .support
            .iter()
            .map(|(key, support)| format!("{}:{}", hx(key), support.iter().map(hx).collect::<Vec<_>>().join(",")))
            .collect::<Vec<_>>()
            .join(";"),
    ]
}

fn head_tail(items: Vec<String>) -> Json {
    let head = items.iter().take(3);
    let tail = items.iter().skip(items.len().saturating_sub(3));
    jarr(head.chain(tail).map(|item| jstr(item.clone())).collect())
}

fn digest_json(state: &MemoryState) -> Json {
    digest_with(state, true)
}

fn digest_with(state: &MemoryState, heads: bool) -> Json {
    let texts = state_texts(state);
    let mut items = vec![
        (
            "n",
            jarr(vec![
                jint(state.known_primes.len() as u64),
                jint(state.factorization.len() as u64),
                jint(state.squarefree.len() as u64),
                jint(state.support.len() as u64),
            ]),
        ),
        ("d", jarr(texts.iter().map(|text| jstr(format!("{:x}", fnv1a64(text.as_bytes())))).collect())),
    ];
    if heads {
        items.push(("pk", head_tail(state.known_primes.iter().map(hx).collect())));
        items.push(("fk", head_tail(state.factorization.iter().map(|(key, _)| hx(key)).collect())));
    }
    obj(items)
}

fn delta_json(delta: &MemoryDelta) -> Json {
    obj(vec![
        ("f", jarr(delta.factorizations.iter().map(|(key, pairs)| jarr(vec![jhex(key), pairs_json(pairs)])).collect())),
        ("s", jarr(delta.squarefree.iter().map(|(key, split)| jarr(vec![jhex(key), split_json(split)])).collect())),
        ("u", jarr(delta.supports.iter().map(|(key, support)| jarr(vec![jhex(key), hex_list(support)])).collect())),
        ("p", hex_list(&delta.primes)),
    ])
}

fn error_json(error: &CanonError) -> Json {
    match error {
        CanonError::Exhausted(error) => jarr(vec![jstr("exh"), jstr(error.operation.as_str()), jhex(&error.radicand)]),
        CanonError::NegativeRadicand { numerator, denominator } => {
            jarr(vec![jstr("neg"), jstr(hx_signed(numerator)), jhex(denominator)])
        }
        other => panic!("unexpected refusal {other:?}"),
    }
}

// ---- before-state and the interpreter ----------------------------------------------------------------------------

fn pairs_of(json: &Json) -> Vec<(UBig, u64)> {
    json.arr().iter().map(|item| (ub(item.arr()[0].str()), item.arr()[1].int() as u64)).collect()
}

fn apply_fill(state: &mut MemoryState, fill: &Json) {
    if fill.has("p") {
        let rule = fill.get("p");
        let (start, step) = (ub(rule.get("start").str()), UBig::from(rule.get("step").int() as u64));
        for index in 0..rule.get("count").int() as u64 {
            state.known_primes.push(&start + &step * UBig::from(index));
        }
    }
    if fill.has("f") {
        let rule = fill.get("f");
        let start = ub(rule.get("start").str());
        for index in 0..rule.get("count").int() as u64 {
            let key = &start + UBig::from(index);
            state.factorization.push((key.clone(), vec![(key, 1)]));
        }
    }
}

fn before_state(before: &Json) -> MemoryState {
    let mut state = MemoryState {
        known_primes: before.get("p").arr().iter().map(|item| ub(item.str())).collect(),
        factorization: before.get("f").arr().iter().map(|item| (ub(item.arr()[0].str()), pairs_of(&item.arr()[1]))).collect(),
        squarefree: before
            .get("s")
            .arr()
            .iter()
            .map(|item| (ub(item.arr()[0].str()), (ub(item.arr()[1].arr()[0].str()), ub(item.arr()[1].arr()[1].str()))))
            .collect(),
        support: before
            .get("u")
            .arr()
            .iter()
            .map(|item| (ub(item.arr()[0].str()), item.arr()[1].arr().iter().map(|prime| ub(prime.str())).collect()))
            .collect(),
    };
    apply_fill(&mut state, before.get("fill"));
    state
}

struct Context {
    budget: WorkBudget,
    unbudgeted: WorkBudget,
    track_unbudgeted: bool,
    mode: String,
    stores: HashMap<(String, String), UniverseRecord>,
    marks: HashMap<String, MemoryMarker>,
    deltas: HashMap<String, MemoryDelta>,
}

fn q_values(call: &Json) -> Vec<QValue> {
    call.get("q")
        .arr()
        .iter()
        .map(|item| QValue { numerator: ib(item.arr()[0].str()), denominator: ub(item.arr()[1].str()) })
        .collect()
}

fn radicands(items: &Json) -> Vec<UBig> {
    items.arr().iter().map(|item| ub(item.str())).collect()
}

fn remembered(memory: &mut CanonMemory, context: &mut Context, call: &Json, none: bool) -> Result<Json, CanonError> {
    let q = q_values(call);
    let key = (call.get("store").str().to_owned(), call.get("q").text());
    let budget = if none { &mut context.unbudgeted } else { &mut context.budget };
    let (universe, written) = memory.prime_universe_remembered(&q, budget, !none, context.stores.get(&key))?;
    let Some(record) = written else {
        return Ok(jarr(vec![jstr("hit"), hex_list(&universe), Json::Null]));
    };
    let delta = jarr(record.delta.iter().map(|(number, pairs)| jarr(vec![jhex(number), pairs_json(pairs)])).collect());
    let price = record.price.map_or(Json::Null, |price| jarr(price.iter().map(|article| jint(*article)).collect()));
    let encoded = jarr(vec![delta, price, delta_json(&record.memory)]);
    context.stores.insert(key, record);
    Ok(jarr(vec![jstr("miss"), hex_list(&universe), encoded]))
}

fn fill_op(memory: &mut CanonMemory, call: &Json) {
    let mut state = memory.export_state();
    let rule = call.get("rule").clone();
    let fill = obj(vec![(call.get("table").str(), rule)]);
    let mut extra = MemoryState::default();
    apply_fill(&mut extra, &fill);
    state.known_primes.extend(extra.known_primes);
    state.factorization.extend(extra.factorization);
    memory.load_state(state).expect("a fill keeps the registry ascending");
}

fn execute(memory: &mut CanonMemory, context: &mut Context, call: &Json) -> Json {
    let none = call.get("none").flag();
    match run_op(memory, context, call, none) {
        Ok(result) => result,
        Err(error) => error_json(&error),
    }
}

fn run_op(memory: &mut CanonMemory, context: &mut Context, call: &Json, none: bool) -> Result<Json, CanonError> {
    if call.get("op").str() == "remembered" {
        return remembered(memory, context, call, none);
    }
    let budget = if none { &mut context.unbudgeted } else { &mut context.budget };
    Ok(match call.get("op").str() {
        "split" => split_json(&memory.squarefree_split(&ib(call.get("n").str()), budget)?),
        "support" => hex_list(&memory.prime_support(&ib(call.get("n").str()), budget)?),
        "pairs" => {
            let n = ib(call.get("n").str());
            match UBig::try_from(n) {
                Ok(unsigned) => pairs_json(&memory.factorization_pairs(&unsigned, budget)?),
                Err(_) => jarr(vec![]),
            }
        }
        "seed" => {
            memory.seed_factorization_basis(&radicands(call.get("radicands")), budget)?;
            Json::Null
        }
        "basis" => hex_list(&cftuv_canon::factor::coprime_basis(&radicands(call.get("values")), budget)?),
        "universe" => hex_list(&memory.prime_universe_from_q_values(&q_values(call), budget)?),
        "reset" => {
            memory.reset();
            Json::Null
        }
        "fill" => {
            fill_op(memory, call);
            Json::Null
        }
        "mark" => {
            context.marks.insert(call.get("name").str().to_owned(), memory.marker());
            Json::Null
        }
        "delta" => {
            let delta = memory.delta_since(&context.marks[call.get("name").str()]);
            let encoded = delta_json(&delta);
            context.deltas.insert(call.get("as").str().to_owned(), delta);
            encoded
        }
        "replay" => {
            let delta = context.deltas[call.get("name").str()].clone();
            memory.replay_delta(&delta);
            Json::Null
        }
        "isolated" => {
            let calls = call.get("calls").arr().to_vec();
            let entries = memory.with_isolated(|inner| calls.iter().map(|item| record(inner, context, item)).collect::<Vec<_>>());
            jarr(entries)
        }
        other => panic!("unknown op {other}"),
    })
}

fn state_entry(memory: &CanonMemory, mode: &str) -> Json {
    match mode {
        "none" => Json::Null,
        "digest" => digest_json(&memory.export_state()),
        _ => state_json(&memory.export_state()),
    }
}

fn record(memory: &mut CanonMemory, context: &mut Context, call: &Json) -> Json {
    let result = execute(memory, context, call);
    let mut items = vec![("r", result), ("a", articles_json(&context.budget)), ("s", state_entry(memory, &context.mode))];
    if context.track_unbudgeted {
        items.push(("u", articles_json(&context.unbudgeted)));
    }
    obj(items)
}

/// `state` of the program; a run under a cap uses `state_capped` when the program names one.
fn state_mode(program: &Json, cap: &Json) -> String {
    let base = if program.has("state") { program.get("state").str() } else { "full" };
    match cap {
        Json::Int(_) if program.has("state_capped") => program.get("state_capped").str().to_owned(),
        _ => base.to_owned(),
    }
}

fn start(program: &Json, cap: &Json) -> (CanonMemory, Context) {
    let mut memory = CanonMemory::new();
    memory.load_state(before_state(program.get("before"))).expect("valid before-state");
    let budget = match cap {
        Json::Int(cap) => WorkBudget::bounded(*cap as u64),
        _ => WorkBudget::unlimited(),
    };
    let context = Context {
        budget,
        unbudgeted: WorkBudget::unlimited(),
        track_unbudgeted: program.get("unbudgeted").flag(),
        mode: state_mode(program, cap),
        stores: HashMap::new(),
        marks: HashMap::new(),
        deltas: HashMap::new(),
    };
    (memory, context)
}

// ---- a host mirror driven only by the op-log ---------------------------------------------------------------------

/// What a Python shim would keep: plain lists with dict semantics, mutated ONLY by replaying `MemOp`s.
struct Mirror {
    state: MemoryState,
}

fn put<V>(table: &mut Vec<(UBig, V)>, key: &UBig, value: V) {
    match table.iter_mut().find(|(known, _)| known == key) {
        Some(entry) => entry.1 = value,
        None => table.push((key.clone(), value)),
    }
}

impl Mirror {
    fn apply(&mut self, op: &MemOp) {
        let state = &mut self.state;
        match op {
            MemOp::ResetAll => *state = MemoryState::default(),
            MemOp::RegistryClear => state.known_primes.clear(),
            MemOp::RegistryInsert(prime) => {
                let at = state.known_primes.partition_point(|known| known <= prime);
                state.known_primes.insert(at, prime.clone());
            }
            MemOp::FactorizationEvictOldest { key } => {
                assert_eq!(&state.factorization[0].0, key, "the log must name the oldest key");
                state.factorization.remove(0);
            }
            MemOp::FactorizationInsert { key, pairs } => put(&mut state.factorization, key, pairs.clone()),
            MemOp::FactorizationTouch { key } => {
                let at = state.factorization.iter().position(|(known, _)| known == key).expect("touched key is present");
                let entry = state.factorization.remove(at);
                state.factorization.push(entry);
            }
            MemOp::SquarefreeInsert { key, value } => put(&mut state.squarefree, key, value.clone()),
            MemOp::SupportInsert { key, value } => put(&mut state.support, key, value.clone()),
        }
    }
}

// ---- the tests ---------------------------------------------------------------------------------------------------

fn check_runs(name: &str) -> usize {
    let vectors = load_vectors(name);
    let mut calls_checked = 0;
    for case in vectors.get("cases").arr() {
        let program = case.get("program");
        for run in case.get("runs").arr() {
            let (mut memory, mut context) = start(program, run.get("cap"));
            memory.start_log();
            let mut mirror = Mirror { state: memory.export_state() };
            for (index, (call, expected)) in program.get("calls").arr().iter().zip(run.get("calls").arr()).enumerate() {
                let entry = record(&mut memory, &mut context, call);
                if call.get("op").str() == "fill" {
                    mirror = Mirror { state: memory.export_state() };
                }
                for op in memory.take_log() {
                    mirror.apply(&op);
                }
                assert!(mirror.state == memory.export_state(), "{name}/{} call {index}: the op-log does not reproduce the state", program.get("name").str());
                assert_eq!(
                    entry,
                    *expected,
                    "{name}/{} cap {} call {index} {}",
                    program.get("name").str(),
                    run.get("cap").text(),
                    call.text()
                );
                calls_checked += 1;
            }
            let final_state = state_entry(&memory, &context.mode);
            assert_eq!(&final_state, run.get("final"), "{name}/{} cap {} final state", program.get("name").str(), run.get("cap").text());
        }
    }
    calls_checked
}

#[test]
fn sequences_match_the_oracle_after_every_call() {
    assert!(check_runs("sequences") > 300);
}

#[test]
fn corpus_programs_match_the_oracle() {
    assert!(check_runs("corpus") > 100);
}

#[test]
fn thresholds_match_the_oracle() {
    assert!(check_runs("thresholds") > 40);
}

#[test]
fn cap_sweeps_match_the_oracle_for_every_cap() {
    let vectors = load_vectors("sweeps");
    let mut caps_checked = 0u64;
    for case in vectors.get("cases").arr() {
        let program = case.get("program");
        let calls = program.get("calls").arr();
        for group in case.get("sweep").arr() {
            let (low, high) = (group.get("caps").arr()[0].int(), group.get("caps").arr()[1].int());
            for cap in low..=high {
                let (mut memory, mut context) = start(program, &Json::Int(cap));
                let mut last = Json::Null;
                let mut failed = -1i64;
                for (index, call) in calls.iter().enumerate() {
                    last = record(&mut memory, &mut context, call);
                    if matches!(last.get("r").arr().first(), Some(Json::Str(tag)) if tag == "exh") {
                        failed = index as i64;
                        break;
                    }
                }
                let context_label = format!("{} cap {cap}", program.get("name").str());
                assert_eq!(Json::Int(failed), *group.get("i"), "{context_label}: exhausted call");
                assert_eq!(last.get("r"), group.get("r"), "{context_label}: result");
                assert_eq!(last.get("a"), group.get("a"), "{context_label}: articles");
                assert_eq!(digest_with(&memory.export_state(), false), *group.get("s"), "{context_label}: partial memory");
                caps_checked += 1;
            }
        }
    }
    assert!(caps_checked > 3000, "only {caps_checked} caps were checked");
}
