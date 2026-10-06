//! The cost session: the native mirror of the Python canonicalization memory, the budget of a call, and the wire
//! shapes of the cost-bearing operations (`exact.rs`, opcodes 70..80) inside the number script (`script.rs`).
//!
//! A [`Session`] persists across calls (the product memory is a pure cache; the memory mirror is what makes
//! costs equal). Before the operations of a call, the host sends the cost header
//!
//! ```text
//! header   [ options, sync, budget ]
//! options  int; bit 0: answer every operation with the full ordered memory (differential tests)
//! sync     [ registry, factorization, squarefree, support ]
//! registry [ clear, removed primes, added primes ]
//! table    [ clear, deleted keys, tail ]     tail entries: [key, value...] appended in order
//!            factorization  [key, [[prime, power], ...]]
//!            squarefree     [key, outside, inside]
//!            support        [key, [prime, ...]]
//! budget   none (an unlimited budget starting at zero: Python's `budget=None` / `UNBUDGETED_WORK` path)
//!          | [ cap or none, article x 6 ]          (articles in `spent_by_article` order)
//! ```
//!
//! and every cost operation answers `[ outcome, sign-count delta (5), articles after (6), log, state ]`:
//!
//! ```text
//! outcome  [0, value]                      ok
//!          [1, operation index, radicand]  budget exhausted (`Operation::ALL` order)
//!          [2, numerator, denominator]     negative radicand
//!          [3]                             zero divisor           [4, radicand] reconstruction failed
//!          [5] invalid mirror input        [6] generic division diverged        [7] internal state
//! log      the mutations of the memory in Python's order (see `MemOp`): [0] reset, [1] registry clear,
//!          [2, p] registry insert, [3, key] evict oldest, [4, key, pairs] factorization insert,
//!          [5, key] LRU touch, [6, key, outside, inside] split insert, [7, key, support] support insert
//! state    none, or [primes, [[key, pairs]...], [[key, outside, inside]...], [[key, support]...]] (bit 0 of options)
//! ```
//!
//! The host applies the log to its real Python containers in place; the budget articles are cumulative inside
//! one call (operations of a script evolve them exactly like consecutive Python calls).

use cftuv_canon::{CanonError, CanonMemory, MemOp, MemorySync, MemoryState, Operation, Pairs, QValue, TableSync, UniverseRecord, WorkBudget};

use crate::codec::Value;
use crate::exact::{self, ExactCtx, ExactError, UniverseStore};
use crate::num::{self, IBig, UBig};
use crate::products::ProductMemo;
use crate::rat::Rat;
use crate::script::{Args, ScriptError};
use crate::sqrt_sum::SignCounts;

/// `(opcode, name)` of the cost-bearing operations; they live in the table of `script::OPS` too.
pub const COST_OPS: &[(u8, &str)] = &[
    (70, "EXACT_SIGN"),
    (71, "EXACT_DIVIDED_BY"),
    (72, "EXACT_DIVIDE_WITH_UNIVERSE"),
    (73, "EXACT_RADICAL"),
    (74, "EXACT_RADICAL_SUM"),
    (75, "EXACT_PRIME_UNIVERSE"),
    (76, "EXACT_SQUAREFREE_SPLIT"),
    (77, "EXACT_PRIME_SUPPORT"),
    (78, "EXACT_RESET_MEMORY"),
    (79, "EXACT_DIVIDED_BY_GENERIC"),
    (80, "EXACT_DIFFERENCE_SIGN"),
];

/// Options bit: return the full ordered memory after every operation.
pub const OPTION_FULL_STATE: u64 = 1;

/// What survives between calls: the memory mirror and the (pure) product cache.
pub struct Session {
    pub memory: CanonMemory,
    pub products: ProductMemo,
}

impl Session {
    pub fn new() -> Session {
        Session { memory: CanonMemory::new(), products: ProductMemo::new() }
    }
}

impl Default for Session {
    fn default() -> Session {
        Session::new()
    }
}

/// The per-call state of a cost script: the budget and the answer options.
pub struct CostRun {
    budget: WorkBudget,
    full_state: bool,
}

fn refuse(what: &'static str) -> ScriptError {
    ScriptError::CostHeader(what)
}

// --------------------------------------------------------------------------
// decoding the header and the arguments
// --------------------------------------------------------------------------

fn list<'a>(value: &'a Value, what: &'static str) -> Result<&'a [Value], ScriptError> {
    match value {
        Value::List(items) => Ok(items),
        _ => Err(refuse(what)),
    }
}

fn ubig(value: &Value, what: &'static str) -> Result<UBig, ScriptError> {
    match value {
        Value::Int(number) if !num::is_negative(number) => Ok(num::magnitude(number)),
        _ => Err(refuse(what)),
    }
}

fn int_of(value: &Value, what: &'static str) -> Result<u64, ScriptError> {
    match value {
        Value::Int(number) => u64::try_from(number).map_err(|_| refuse(what)),
        _ => Err(refuse(what)),
    }
}

fn flag(value: &Value, what: &'static str) -> Result<bool, ScriptError> {
    match value {
        Value::Bool(flag) => Ok(*flag),
        _ => Err(refuse(what)),
    }
}

fn ubigs(value: &Value, what: &'static str) -> Result<Vec<UBig>, ScriptError> {
    list(value, what)?.iter().map(|item| ubig(item, what)).collect()
}

fn pairs(value: &Value, what: &'static str) -> Result<Pairs, ScriptError> {
    let mut out = Pairs::new();
    for entry in list(value, what)? {
        match list(entry, what)? {
            [prime, power] => out.push((ubig(prime, what)?, int_of(power, what)?)),
            _ => return Err(refuse(what)),
        }
    }
    Ok(out)
}

fn table_sync<V>(
    value: &Value,
    what: &'static str,
    entry: impl Fn(&[Value]) -> Result<(UBig, V), ScriptError>,
) -> Result<TableSync<V>, ScriptError> {
    let [clear, deleted, tail] = list(value, what)? else {
        return Err(refuse(what));
    };
    let tail = list(tail, what)?.iter().map(|item| entry(list(item, what)?)).collect::<Result<Vec<_>, _>>()?;
    Ok(TableSync { clear: flag(clear, what)?, deleted: ubigs(deleted, what)?, tail })
}

fn decode_sync(value: &Value) -> Result<MemorySync, ScriptError> {
    let [registry, factorization, squarefree, support] = list(value, "sync")? else {
        return Err(refuse("sync must hold the registry and three tables"));
    };
    let [registry_clear, removed, added] = list(registry, "registry")? else {
        return Err(refuse("registry sync must be [clear, removed, added]"));
    };
    let what = "table entry";
    Ok(MemorySync {
        registry_clear: flag(registry_clear, "registry")?,
        registry_removed: ubigs(removed, "registry")?,
        registry_added: ubigs(added, "registry")?,
        factorization: table_sync(factorization, "factorization sync", |item| match item {
            [key, pairs_value] => Ok((ubig(key, what)?, pairs(pairs_value, what)?)),
            _ => Err(refuse(what)),
        })?,
        squarefree: table_sync(squarefree, "squarefree sync", |item| match item {
            [key, outside, inside] => Ok((ubig(key, what)?, (ubig(outside, what)?, ubig(inside, what)?))),
            _ => Err(refuse(what)),
        })?,
        support: table_sync(support, "support sync", |item| match item {
            [key, primes] => Ok((ubig(key, what)?, ubigs(primes, what)?)),
            _ => Err(refuse(what)),
        })?,
    })
}

fn decode_budget(value: &Value) -> Result<WorkBudget, ScriptError> {
    let what = "budget";
    if matches!(value, Value::None) {
        return Ok(WorkBudget::unlimited());
    }
    let [cap, articles @ ..] = list(value, what)? else {
        return Err(refuse(what));
    };
    let mut budget = match cap {
        Value::None => WorkBudget::unlimited(),
        other => WorkBudget::bounded(int_of(other, what)?),
    };
    let articles: [u64; 6] = articles.iter().map(|article| int_of(article, what)).collect::<Result<Vec<_>, _>>()?.try_into().map_err(|_| refuse(what))?;
    budget.set_articles(articles);
    Ok(budget)
}

impl CostRun {
    /// Reads the cost header, brings the memory mirror to the host's content, and starts the op-log.
    pub fn begin(session: &mut Session, header: &Value) -> Result<CostRun, ScriptError> {
        let [options, sync, budget] = list(header, "cost header")? else {
            return Err(refuse("the cost header must be [options, sync, budget]"));
        };
        let options = int_of(options, "options")?;
        session.memory.apply_sync(decode_sync(sync)?).map_err(|error| match error {
            CanonError::InvalidInput(message) => ScriptError::CostSync(message),
            _ => ScriptError::CostSync("the memory sync was refused"),
        })?;
        session.memory.start_log();
        Ok(CostRun { budget: decode_budget(budget)?, full_state: options & OPTION_FULL_STATE != 0 })
    }
}

fn rat_list(args: &Args, index: usize) -> Result<Vec<Rat>, ScriptError> {
    args.list(index)?.iter().map(|value| rat_of(args, value)).collect()
}

fn rat_of(args: &Args, value: &Value) -> Result<Rat, ScriptError> {
    match value {
        Value::Int(number) => Ok(Rat::from_int(number.clone())),
        Value::Frac(rat) => Ok(rat.clone()),
        _ => Err(args.bad()),
    }
}

fn universe_record(args: &Args, value: &Value) -> Result<UniverseRecord, ScriptError> {
    let bad = || args.bad();
    let [universe, delta] = list(value, "record").map_err(|_| bad())? else {
        return Err(bad());
    };
    let mut entries = Vec::new();
    for entry in list(delta, "record").map_err(|_| bad())? {
        match list(entry, "record").map_err(|_| bad())? {
            [number, recorded] => entries.push((ubig(number, "record").map_err(|_| bad())?, pairs(recorded, "record").map_err(|_| bad())?)),
            _ => return Err(bad()),
        }
    }
    Ok(UniverseRecord { universe: ubigs(universe, "record").map_err(|_| bad())?, delta: entries })
}

// --------------------------------------------------------------------------
// encoding the answers
// --------------------------------------------------------------------------

fn int(number: impl Into<IBig>) -> Value {
    Value::Int(number.into())
}

fn ubig_value(value: &UBig) -> Value {
    Value::Int(IBig::from(value.clone()))
}

fn ubig_list(values: &[UBig]) -> Value {
    Value::List(values.iter().map(ubig_value).collect())
}

fn pairs_value(pairs: &Pairs) -> Value {
    Value::List(pairs.iter().map(|(prime, power)| Value::List(vec![ubig_value(prime), int(*power)])).collect())
}

fn record_value(record: &UniverseRecord) -> Value {
    let delta = record.delta.iter().map(|(number, pairs)| Value::List(vec![ubig_value(number), pairs_value(pairs)])).collect();
    Value::List(vec![ubig_list(&record.universe), Value::List(delta)])
}

fn log_entry(op: MemOp) -> Value {
    let entry = match op {
        MemOp::ResetAll => vec![int(0u8)],
        MemOp::RegistryClear => vec![int(1u8)],
        MemOp::RegistryInsert(prime) => vec![int(2u8), ubig_value(&prime)],
        MemOp::FactorizationEvictOldest { key } => vec![int(3u8), ubig_value(&key)],
        MemOp::FactorizationInsert { key, pairs } => vec![int(4u8), ubig_value(&key), pairs_value(&pairs)],
        MemOp::FactorizationTouch { key } => vec![int(5u8), ubig_value(&key)],
        MemOp::SquarefreeInsert { key, value } => vec![int(6u8), ubig_value(&key), ubig_value(&value.0), ubig_value(&value.1)],
        MemOp::SupportInsert { key, value } => vec![int(7u8), ubig_value(&key), ubig_list(&value)],
    };
    Value::List(entry)
}

fn state_value(state: &MemoryState) -> Value {
    let factorization = state.factorization.iter().map(|(key, pairs)| Value::List(vec![ubig_value(key), pairs_value(pairs)])).collect();
    let squarefree = state.squarefree.iter().map(|(key, split)| Value::List(vec![ubig_value(key), ubig_value(&split.0), ubig_value(&split.1)])).collect();
    let support = state.support.iter().map(|(key, support)| Value::List(vec![ubig_value(key), ubig_list(support)])).collect();
    Value::List(vec![ubig_list(&state.known_primes), Value::List(factorization), Value::List(squarefree), Value::List(support)])
}

fn outcome_value(result: Result<Value, ExactError>) -> Value {
    let entry = match result {
        Ok(value) => vec![int(0u8), value],
        Err(ExactError::Canon(CanonError::Exhausted(exhausted))) => {
            let index = Operation::ALL.iter().position(|operation| *operation == exhausted.operation).unwrap_or(0);
            vec![int(1u8), int(index as u64), ubig_value(&exhausted.radicand)]
        }
        Err(ExactError::Canon(CanonError::NegativeRadicand { numerator, denominator })) => vec![int(2u8), Value::Int(numerator), ubig_value(&denominator)],
        Err(ExactError::ZeroDivisor) => vec![int(3u8)],
        Err(ExactError::Canon(CanonError::ReconstructionFailed { radicand })) => vec![int(4u8), ubig_value(&radicand)],
        Err(ExactError::Canon(CanonError::InvalidInput(_))) => vec![int(5u8)],
        Err(ExactError::Diverged) => vec![int(6u8)],
        Err(ExactError::Internal(_)) => vec![int(7u8)],
    };
    Value::List(entry)
}

// --------------------------------------------------------------------------
// the operations
// --------------------------------------------------------------------------

fn run_exact(args: &Args, ctx: &mut ExactCtx<'_>) -> Result<Result<Value, ExactError>, ScriptError> {
    Ok(match args.code() {
        70 => {
            args.expect(2)?;
            exact::sign(ctx, args.sum(0)?, args.bits(1)?).map(int)
        }
        71 => {
            args.expect(2)?;
            exact::divided_by(ctx, args.sum(0)?, args.sum(1)?).map(Value::Sum)
        }
        72 => {
            args.expect(3)?;
            let universe = args.ubig_list(2)?;
            exact::divide_with_prime_universe(ctx, args.sum(0)?, args.sum(1)?, &universe).map(Value::Sum)
        }
        73 => {
            args.expect(2)?;
            exact::radical(ctx, &args.rat(0)?, &args.rat(1)?).map(Value::Sum)
        }
        74 => {
            args.expect(1)?;
            let mut parts = Vec::new();
            for entry in args.list(0)? {
                match list(entry, "part").map_err(|_| args.bad())? {
                    [coefficient, radicand] => parts.push((rat_of(args, coefficient)?, rat_of(args, radicand)?)),
                    _ => return Err(args.bad()),
                }
            }
            exact::radical_sum(ctx, &parts).map(Value::Sum)
        }
        75 => {
            args.expect(2)?;
            let q_values: Vec<QValue> =
                rat_list(args, 0)?.into_iter().map(|rat| { let (numerator, denominator) = rat.into_parts(); QValue { numerator, denominator } }).collect();
            let record;
            let store = match &args.values()[1] {
                Value::None => UniverseStore::Absent,
                Value::Bool(false) => UniverseStore::Miss,
                other => {
                    record = universe_record(args, other)?;
                    UniverseStore::Hit(&record)
                }
            };
            exact::prime_universe(ctx, &q_values, store)
                .map(|(universe, record)| Value::List(vec![ubig_list(&universe), record.as_ref().map_or(Value::None, record_value)]))
        }
        76 => {
            args.expect(1)?;
            ctx.memory
                .squarefree_split(args.int(0)?, ctx.budget)
                .map(|(outside, inside)| Value::List(vec![ubig_value(&outside), ubig_value(&inside)]))
                .map_err(ExactError::from)
        }
        77 => {
            args.expect(1)?;
            ctx.memory.prime_support(args.int(0)?, ctx.budget).map(|support| ubig_list(&support)).map_err(ExactError::from)
        }
        78 => {
            args.expect(0)?;
            ctx.memory.reset();
            Ok(Value::None)
        }
        79 => {
            args.expect(2)?;
            exact::divided_by_generic(ctx, args.sum(0)?, args.sum(1)?).map(Value::Sum)
        }
        80 => {
            args.expect(2)?;
            exact::difference_sign(ctx, args.sum(0)?, args.sum(1)?).map(int)
        }
        other => return Err(ScriptError::UnknownOpcode(other)),
    })
}

/// One cost operation of a script: runs it on the session, answers `[outcome, counts, articles, log, state]`.
/// An argument of the wrong shape is a `ScriptError`; every refusal of the arithmetic is an outcome.
pub(crate) fn execute_cost_op(session: &mut Session, run: &mut CostRun, args: &Args) -> Result<Value, ScriptError> {
    let mut counts = SignCounts::default();
    let result = {
        let mut ctx = ExactCtx { memory: &mut session.memory, budget: &mut run.budget, counts: &mut counts, products: &mut session.products };
        run_exact(args, &mut ctx)?
    };
    let log = Value::List(session.memory.take_log().into_iter().map(log_entry).collect());
    let state = if run.full_state { state_value(&session.memory.export_state()) } else { Value::None };
    Ok(Value::List(vec![
        outcome_value(result),
        Value::List(counts.as_array().iter().map(|count| int(*count)).collect()),
        Value::List(run.budget.articles().iter().map(|article| int(*article)).collect()),
        log,
        state,
    ]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codec::{Reader, Writer};
    use crate::rat::Coef;
    use crate::script::{run_script, FLAG_COST, FLAG_STRICT, MAGIC};
    use crate::sqrt_sum::{SqrtSum, Term};

    fn n(value: i64) -> Value {
        Value::Int(IBig::from(value))
    }

    fn vals(items: Vec<Value>) -> Value {
        Value::List(items)
    }

    fn empty_table() -> Value {
        vals(vec![Value::Bool(false), vals(vec![]), vals(vec![])])
    }

    fn header(options: i64, budget: Value) -> Value {
        vals(vec![n(options), vals(vec![empty_table(), empty_table(), empty_table(), empty_table()]), budget])
    }

    fn request(flags: u8, header: Option<Value>, ops: &[(u8, Vec<Value>)]) -> Vec<u8> {
        let mut writer = Writer::new();
        for byte in MAGIC {
            writer.put_u8(*byte);
        }
        writer.put_u8(flags);
        if let Some(header) = header {
            writer.put_value(&header);
        }
        writer.put_uint(ops.len() as u64);
        for (code, args) in ops {
            writer.put_u8(*code);
            writer.put_uint(args.len() as u64);
            for arg in args {
                writer.put_value(arg);
            }
        }
        writer.into_bytes()
    }

    fn answers(response: &[u8]) -> Vec<Vec<Value>> {
        let mut reader = Reader::new(response, true);
        let count = reader.get_uint().unwrap();
        let answers = (0..count)
            .map(|_| match reader.get_value().unwrap() {
                Value::List(items) => items,
                other => panic!("a cost answer is a list, got {other:?}"),
            })
            .collect();
        reader.finish().unwrap();
        answers
    }

    /// `sqrt(2) + sqrt(3) - r`, `r` a 2^-90 approximation: the 64-bit enclosure cannot decide it.
    fn hard_sum() -> SqrtSum {
        let scaled = |radicand: u64| num::isqrt(&(UBig::from(radicand) << 180usize));
        let total = IBig::from(scaled(2)) + IBig::from(scaled(3)) + IBig::ONE;
        let approximation = Rat::new(total, IBig::ONE << 90usize).unwrap();
        let term = |radicand: u64| Term { radicand: UBig::from(radicand), coef: Coef::fraction(Rat::one()) };
        SqrtSum::from_terms(vec![Term { radicand: UBig::ONE, coef: Coef::fraction(approximation.neg()) }, term(2), term(3)]).unwrap()
    }

    fn outcome(answer: &[Value]) -> &[Value] {
        match &answer[0] {
            Value::List(items) => items,
            other => panic!("an outcome is a list, got {other:?}"),
        }
    }

    #[test]
    fn a_sign_answers_with_its_cost_log_and_state_and_the_session_remembers() {
        let mut session = Session::new();
        let sign = (70u8, vec![Value::Sum(hard_sum()), n(64)]);
        let budget = vals(vec![Value::None, n(0), n(0), n(0), n(0), n(0), n(0)]);
        let first = run_script(&mut session, &request(FLAG_STRICT | FLAG_COST, Some(header(OPTION_FULL_STATE as i64, budget.clone())), std::slice::from_ref(&sign))).unwrap();
        let first = answers(&first);
        assert_eq!(outcome(&first[0])[0], n(0), "ok");
        assert_eq!(first[0][1], vals(vec![n(1), n(0), n(0), n(0), n(1)]), "total and closed_by_conjugation");
        let Value::List(articles) = &first[0][2] else { panic!("articles") };
        assert_eq!(articles.len(), 6);
        assert_ne!(articles[4], n(0), "radical materializations were paid for the supports");
        let Value::List(log) = &first[0][3] else { panic!("log") };
        assert!(log.iter().any(|entry| matches!(entry, Value::List(items) if items[0] == n(7))), "support inserts are logged");
        assert!(matches!(&first[0][4], Value::List(parts) if parts.len() == 4), "full state requested");
        // the next call sends an empty sync: the mirror kept the memory, the same sign costs nothing new
        let second = run_script(&mut session, &request(FLAG_STRICT | FLAG_COST, Some(header(0, budget)), &[sign])).unwrap();
        let second = answers(&second);
        assert_eq!(outcome(&second[0])[0], n(0));
        assert_eq!(second[0][2], vals(vec![n(0); 6]), "memory hits are free");
        assert_eq!(second[0][3], vals(vec![]), "and log nothing");
        assert_eq!(second[0][4], Value::None);
    }

    #[test]
    fn an_exhaustion_is_an_outcome_with_the_operation_and_the_radicand() {
        let mut session = Session::new();
        let budget = vals(vec![n(0), n(0), n(0), n(0), n(0), n(0), n(0)]);
        let sign = (70u8, vec![Value::Sum(hard_sum()), n(64)]);
        let response = run_script(&mut session, &request(FLAG_STRICT | FLAG_COST, Some(header(0, budget)), &[sign])).unwrap();
        let response = answers(&response);
        let refused = outcome(&response[0]);
        assert_eq!(refused[0], n(1));
        assert_eq!(refused[1], n(Operation::ALL.iter().position(|operation| *operation == Operation::PrimeSupport).unwrap() as i64));
        assert_eq!(refused[2], n(2));
        assert_eq!(response[0][1], vals(vec![n(1), n(0), n(0), n(0), n(1)]), "the conjugation counter survives");
        let Value::List(articles) = &response[0][2] else { panic!("articles") };
        assert_eq!(articles[4], n(1), "the failing spend stays incremented");
    }

    #[test]
    fn malformed_cost_requests_are_refused_not_run() {
        let mut session = Session::new();
        let sign = (70u8, vec![Value::Sum(hard_sum()), n(64)]);
        // a cost operation without the cost flag
        let result = run_script(&mut session, &request(FLAG_STRICT, None, std::slice::from_ref(&sign)));
        assert!(matches!(result, Err(ScriptError::CostHeader(_))), "{result:?}");
        // a header that is not a list of three
        let bad = request(FLAG_STRICT | FLAG_COST, Some(vals(vec![n(0)])), std::slice::from_ref(&sign));
        assert!(matches!(run_script(&mut session, &bad), Err(ScriptError::CostHeader(_))));
        // a budget with five articles
        let short = vals(vec![Value::None, n(0), n(0), n(0), n(0), n(0)]);
        assert!(matches!(run_script(&mut session, &request(FLAG_STRICT | FLAG_COST, Some(header(0, short)), std::slice::from_ref(&sign))), Err(ScriptError::CostHeader(_))));
        // a sync that names a key the mirror does not hold
        let mut deleted = header(0, Value::None);
        if let Value::List(parts) = &mut deleted {
            parts[1] = vals(vec![empty_table(), vals(vec![Value::Bool(false), vals(vec![n(5)]), vals(vec![])]), empty_table(), empty_table()]);
        }
        assert!(matches!(run_script(&mut session, &request(FLAG_STRICT | FLAG_COST, Some(deleted), std::slice::from_ref(&sign))), Err(ScriptError::CostSync(_))));
        // an opcode outside every table with the cost flag on is still an unknown opcode
        let unknown = request(FLAG_STRICT | FLAG_COST, Some(header(0, Value::None)), &[(99u8, vec![])]);
        assert_eq!(run_script(&mut session, &unknown), Err(ScriptError::UnknownOpcode(99)));
    }

    #[test]
    fn number_operations_and_cost_operations_share_one_script() {
        let mut session = Session::new();
        let ops = [(1u8, vec![n(99)]), (70u8, vec![Value::Sum(hard_sum()), n(64)]), (2u8, vec![n(12), n(18)])];
        let response = run_script(&mut session, &request(FLAG_STRICT | FLAG_COST, Some(header(0, Value::None)), &ops)).unwrap();
        let mut reader = Reader::new(&response, true);
        assert_eq!(reader.get_uint().unwrap(), 3);
        assert_eq!(reader.get_value().unwrap(), n(9));
        assert!(matches!(reader.get_value().unwrap(), Value::List(_)));
        assert_eq!(reader.get_value().unwrap(), n(6));
    }
}
