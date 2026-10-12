//! The differential entry: an operation PROGRAM over `RadicalSumV2`, run here and by `tools/native_radical_v2_corpus.py` on the Python oracle, outcome for outcome.
//!
//! A program is a list of [`Instruction`]s. An instruction that produces a value defines the register of its own index; the others read registers of EARLIER instructions. Every
//! instruction has one outcome ([`Done`]): the value, the sign, the enclosure, a text, ... or the named failure of the oracle (a refusal with its work, `ZeroDivisionError`,
//! `ValueError`). The same outcomes have a text form (one line each), which the vector files written from the oracle (`tests/vectors.txt`) are made of; the extension returns them as
//! Python tuples. Nothing here is a product entry: the density core calls [`Radical`] directly.
//!
//! Text grammar (tokens separated by one space; a wire is `r,n,d,...` or `-` for the empty one; pairs are `r,n,d,...` triples of `from_terms`):
//!
//! ```text
//! zero | rational N D | sqrt N D | terms PAIRS | wire WIRE | text TEXT...
//! add I J | sub I J | mul I J | div I J | inverse I | neg I | scaled I N D | signum I CAP|- | enclosure I BITS
//! text_of I | wire_of I | as_rational I | eq I J | sep_bits I
//! ```

use std::time::Instant;

use crate::arith::{fraction, IBig, UBig};
use crate::ladder;
use crate::text::ReadError;
use crate::value::Radical;
use crate::work::{Failure, Refusal, Work};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Instruction {
    Zero,
    /// `RadicalSumV2.rational(Fraction(n, d))`.
    Rational(IBig, IBig),
    /// `RadicalSumV2.sqrt_of_rational(Fraction(n, d))`.
    Sqrt(IBig, IBig),
    /// `RadicalSumV2.from_terms([(r, Fraction(n, d)), ...])`.
    FromTerms(Vec<(IBig, IBig, IBig)>),
    /// `value_of_wire(wire)`.
    Wire(Vec<IBig>),
    /// `value_of_text(text)`.
    Text(String),
    Add(usize, usize),
    Sub(usize, usize),
    Mul(usize, usize),
    Div(usize, usize),
    Inverse(usize),
    Neg(usize),
    /// `x.scaled(Fraction(n, d))`.
    Scaled(usize, IBig, IBig),
    /// `x.signum(precision_cap_bits)`.
    Signum(usize, Option<IBig>),
    /// `x.enclosure(bits)`; `bits` is a Python int (a negative one is the oracle's `ValueError`).
    Enclosure(usize, IBig),
    TextOf(usize),
    WireOf(usize),
    AsRational(usize),
    Equals(usize, usize),
    /// `separation_bits(_integer_form(x.terms)[1])`.
    SeparationBits(usize),
}

/// The outcome of one instruction.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Done {
    Value(Radical, Work),
    Sign(i8, Work),
    Enclosure((IBig, UBig), (IBig, UBig), Work),
    Text(String),
    Wire(Vec<IBig>),
    Rational(Option<(IBig, UBig)>),
    Bool(bool),
    Int(UBig),
    Refused(Refusal),
    ZeroDivision(String),
    ValueError(String),
    BoundViolated(String),
    /// A defect of the program (a register that holds no value): not an outcome of the oracle.
    Invalid(String),
}

fn register(done: &[Done], index: usize) -> Result<&Radical, Done> {
    match done.get(index) {
        Some(Done::Value(value, _)) => Ok(value),
        Some(_) => Err(Done::Invalid(format!("register {index} holds no value"))),
        None => Err(Done::Invalid(format!("register {index} is not defined yet"))),
    }
}

fn failure(failure: Failure) -> Done {
    match failure {
        Failure::Refused(refusal) => Done::Refused(refusal),
        Failure::ZeroDivision => Done::ZeroDivision("division by an exact zero".to_string()),
        Failure::SeparationBoundViolated(message) => Done::BoundViolated(message),
    }
}

fn read_error(error: ReadError) -> Done {
    match error {
        ReadError::Value(message) => Done::ValueError(message),
        ReadError::ZeroDivision(message) => Done::ZeroDivision(message),
    }
}

/// `Fraction(numerator, denominator)`, or the `ZeroDivisionError` of its constructor.
fn ratio(numerator: &IBig, denominator: &IBig) -> Result<(IBig, UBig), Done> {
    fraction(numerator.clone(), denominator.clone()).ok_or_else(|| Done::ZeroDivision(format!("Fraction({numerator}, 0)")))
}

/// Executes ONE instruction against the outcomes before it.
pub fn execute(instruction: &Instruction, done: &[Done]) -> Done {
    execute_inner(instruction, done).unwrap_or_else(|early| early)
}

fn execute_inner(instruction: &Instruction, done: &[Done]) -> Result<Done, Done> {
    Ok(match instruction {
        Instruction::Zero => Done::Value(Radical::zero(), Work::NONE),
        Instruction::Rational(numerator, denominator) => {
            let (numerator, denominator) = ratio(numerator, denominator)?;
            Done::Value(Radical::rational(numerator, denominator), Work::NONE)
        }
        Instruction::Sqrt(numerator, denominator) => {
            let (numerator, denominator) = ratio(numerator, denominator)?;
            match Radical::sqrt_of_rational(numerator, denominator) {
                Ok((value, work)) => Done::Value(value, work),
                Err(refusal) => Done::Refused(refusal),
            }
        }
        Instruction::FromTerms(pairs) => {
            let mut read = Vec::with_capacity(pairs.len());
            for (radicand, numerator, denominator) in pairs {
                let (numerator, denominator) = ratio(numerator, denominator)?;
                read.push((radicand.clone(), numerator, denominator));
            }
            match Radical::from_terms(&read) {
                Ok((value, work)) => Done::Value(value, work),
                Err(refusal) => Done::Refused(refusal),
            }
        }
        Instruction::Wire(wire) => match Radical::from_wire(wire) {
            Ok(value) => Done::Value(value, Work::NONE),
            Err(error) => read_error(error),
        },
        Instruction::Text(text) => match Radical::from_text(text) {
            Ok((value, work)) => Done::Value(value, work),
            Err(error) => read_error(error),
        },
        Instruction::Add(a, b) => {
            let (value, work) = register(done, *a)?.add(register(done, *b)?);
            Done::Value(value, work)
        }
        Instruction::Sub(a, b) => {
            let (value, work) = register(done, *a)?.sub(register(done, *b)?);
            Done::Value(value, work)
        }
        Instruction::Mul(a, b) => {
            let (value, work) = register(done, *a)?.mul(register(done, *b)?);
            Done::Value(value, work)
        }
        Instruction::Div(a, b) => match register(done, *a)?.div(register(done, *b)?) {
            Ok((value, work)) => Done::Value(value, work),
            Err(error) => failure(error),
        },
        Instruction::Inverse(a) => match register(done, *a)?.inverse() {
            Ok((value, work)) => Done::Value(value, work),
            Err(error) => failure(error),
        },
        Instruction::Neg(a) => Done::Value(register(done, *a)?.neg(), Work::NONE),
        Instruction::Scaled(a, numerator, denominator) => {
            let (numerator, denominator) = ratio(numerator, denominator)?;
            Done::Value(register(done, *a)?.scaled(&numerator, &denominator), Work::NONE)
        }
        Instruction::Signum(a, cap) => {
            let cap = match cap {
                None => None,
                Some(cap) => Some(i64::try_from(cap).map_err(|_| Done::Invalid(format!("a precision cap outside i64: {cap}")))?),
            };
            match register(done, *a)?.signum(cap) {
                Ok((sign, work)) => Done::Sign(sign, work),
                Err(error) => failure(error),
            }
        }
        Instruction::Enclosure(a, bits) => {
            let value = register(done, *a)?;
            if *bits < IBig::ZERO {
                return Ok(Done::ValueError("negative shift count".to_string()));
            }
            let bits = usize::try_from(bits).map_err(|_| Done::Invalid(format!("an enclosure of {bits} bits does not fit in memory")))?;
            let ((low_n, low_d), (high_n, high_d)) = value.enclosure(bits).0;
            Done::Enclosure((low_n, low_d), (high_n, high_d), Work::rounds(1))
        }
        Instruction::TextOf(a) => Done::Text(register(done, *a)?.text()),
        Instruction::WireOf(a) => Done::Wire(register(done, *a)?.wire()),
        Instruction::AsRational(a) => Done::Rational(register(done, *a)?.as_rational()),
        Instruction::Equals(a, b) => Done::Bool(register(done, *a)? == register(done, *b)?),
        Instruction::SeparationBits(a) => {
            let value = register(done, *a)?;
            if value.is_zero() {
                return Ok(Done::ValueError("negative shift count".to_string()));
            }
            Done::Int(ladder::separation_bits(&value.integer_form().items))
        }
    })
}

/// Runs a program: one outcome per instruction.
pub fn run(program: &[Instruction]) -> Vec<Done> {
    let mut done: Vec<Done> = Vec::with_capacity(program.len());
    for instruction in program {
        let outcome = execute(instruction, &done);
        done.push(outcome);
    }
    done
}

/// Runs a program and times every instruction: nanoseconds per execution, the best of `passes` passes, each instruction executed `inner` times in a row on the outcomes before it (so a
/// cheap instruction is measured well below the timer's resolution). Returns the outcomes of the first pass and the timings.
pub fn run_timed(program: &[Instruction], passes: usize, inner: usize) -> (Vec<Done>, Vec<f64>) {
    let inner = inner.max(1);
    let done = run(program);
    let mut best = vec![f64::INFINITY; program.len()];
    for _ in 0..passes.max(1) {
        for (index, instruction) in program.iter().enumerate() {
            let started = Instant::now();
            for _ in 0..inner {
                std::hint::black_box(execute(std::hint::black_box(instruction), std::hint::black_box(&done[..index])));
            }
            let nanoseconds = started.elapsed().as_nanos() as f64 / inner as f64;
            if nanoseconds < best[index] {
                best[index] = nanoseconds;
            }
        }
    }
    (done, best)
}

// --------------------------------------------------------------------------
// text form
// --------------------------------------------------------------------------

fn int(token: &str) -> Result<IBig, String> {
    IBig::from_str_radix(token, 10).map_err(|_| format!("not an integer: {token:?}"))
}

fn index(token: &str) -> Result<usize, String> {
    token.parse::<usize>().map_err(|_| format!("not a register: {token:?}"))
}

fn ints(token: &str) -> Result<Vec<IBig>, String> {
    if token == "-" {
        return Ok(Vec::new());
    }
    token.split(',').map(int).collect()
}

/// One instruction from its text.
pub fn parse_instruction(line: &str) -> Result<Instruction, String> {
    let (name, rest) = line.split_once(' ').unwrap_or((line, ""));
    let tokens: Vec<&str> = rest.split(' ').filter(|token| !token.is_empty()).collect();
    let want = |count: usize| if tokens.len() == count { Ok(()) } else { Err(format!("{name}: {count} operands expected, got {}", tokens.len())) };
    Ok(match name {
        "zero" => {
            want(0)?;
            Instruction::Zero
        }
        "rational" => {
            want(2)?;
            Instruction::Rational(int(tokens[0])?, int(tokens[1])?)
        }
        "sqrt" => {
            want(2)?;
            Instruction::Sqrt(int(tokens[0])?, int(tokens[1])?)
        }
        "terms" => {
            want(1)?;
            let flat = ints(tokens[0])?;
            if flat.len() % 3 != 0 {
                return Err("terms: a whole number of triples expected".to_string());
            }
            Instruction::FromTerms(flat.chunks(3).map(|triple| (triple[0].clone(), triple[1].clone(), triple[2].clone())).collect())
        }
        "wire" => {
            want(1)?;
            Instruction::Wire(ints(tokens[0])?)
        }
        "text" => Instruction::Text(rest.to_string()),
        "add" | "sub" | "mul" | "div" | "eq" => {
            want(2)?;
            let (a, b) = (index(tokens[0])?, index(tokens[1])?);
            match name {
                "add" => Instruction::Add(a, b),
                "sub" => Instruction::Sub(a, b),
                "mul" => Instruction::Mul(a, b),
                "div" => Instruction::Div(a, b),
                _ => Instruction::Equals(a, b),
            }
        }
        "inverse" | "neg" | "text_of" | "wire_of" | "as_rational" | "sep_bits" => {
            want(1)?;
            let a = index(tokens[0])?;
            match name {
                "inverse" => Instruction::Inverse(a),
                "neg" => Instruction::Neg(a),
                "text_of" => Instruction::TextOf(a),
                "wire_of" => Instruction::WireOf(a),
                "as_rational" => Instruction::AsRational(a),
                _ => Instruction::SeparationBits(a),
            }
        }
        "scaled" => {
            want(3)?;
            Instruction::Scaled(index(tokens[0])?, int(tokens[1])?, int(tokens[2])?)
        }
        "signum" => {
            want(2)?;
            Instruction::Signum(index(tokens[0])?, if tokens[1] == "-" { None } else { Some(int(tokens[1])?) })
        }
        "enclosure" => {
            want(2)?;
            Instruction::Enclosure(index(tokens[0])?, int(tokens[1])?)
        }
        other => return Err(format!("unknown instruction {other:?}")),
    })
}

fn wire_text(wire: &[IBig]) -> String {
    if wire.is_empty() {
        "-".to_string()
    } else {
        wire.iter().map(|item| item.to_string()).collect::<Vec<_>>().join(",")
    }
}

fn work_text(work: &Work) -> String {
    format!("{},{},{}", work.class_merge_checks, work.conjugation_products, work.enclosure_rounds)
}

/// The text of one outcome: `v WIRE WORK`, `s SIGN WORK`, `e LOW HIGH WORK` (fractions `n/d`), `t TEXT`, `w WIRE`, `q n/d|-`, `b 0|1`, `i N`, `r CODE WORK DETAIL`, `z MESSAGE`, `x MESSAGE`, `!` , `?`.
pub fn render(done: &Done) -> String {
    match done {
        Done::Value(value, work) => format!("v {} {}", wire_text(&value.wire()), work_text(work)),
        Done::Sign(sign, work) => format!("s {sign} {}", work_text(work)),
        Done::Enclosure(low, high, work) => format!("e {}/{} {}/{} {}", low.0, low.1, high.0, high.1, work_text(work)),
        Done::Text(text) => format!("t {text}"),
        Done::Wire(wire) => format!("w {}", wire_text(wire)),
        Done::Rational(None) => "q -".to_string(),
        Done::Rational(Some((numerator, denominator))) => format!("q {numerator}/{denominator}"),
        Done::Bool(value) => format!("b {}", u8::from(*value)),
        Done::Int(value) => format!("i {value}"),
        Done::Refused(refusal) => format!("r {} {} {}", refusal.code.code(), work_text(&refusal.work), refusal.detail),
        Done::ZeroDivision(message) => format!("z {message}"),
        Done::ValueError(message) => format!("x {message}"),
        Done::BoundViolated(message) => format!("! {message}"),
        Done::Invalid(message) => format!("? {message}"),
    }
}
