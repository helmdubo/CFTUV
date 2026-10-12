//! `radical_v2_eval`: the test and bench entry of `cftuv-radical` (`RadicalSumV2`). A capability of the extension, not a product stage: nothing in the host calls it.
//!
//! The argument is a PROGRAM, a sequence of instruction tuples whose first item names the operation (`cftuv_radical::program`; the same tuples drive the Python oracle in
//! `tools/native_radical_v2_corpus.py`):
//!
//! ```text
//! ("zero",) ("rational", n, d) ("sqrt", n, d) ("terms", ((r, n, d), ...)) ("wire", (int, ...)) ("text", str)
//! ("add"|"sub"|"mul"|"div"|"eq", i, j) ("inverse"|"neg"|"text_of"|"wire_of"|"as_rational"|"sep_bits", i)
//! ("scaled", i, n, d) ("signum", i, cap or None) ("enclosure", i, bits)
//! ```
//!
//! An instruction that makes a value defines the register of its own index; the others read earlier registers. The answer is `(outcomes, run_ns, timings)`: one outcome tuple per
//! instruction (`("v", wire, work)`, `("s", sign, work)`, `("e", low, high, work)`, `("t", text)`, `("w", wire)`, `("q", None | (n, d))`, `("b", bool)`, `("i", int)`,
//! `("r", code, work, detail)`, `("z", message)`, `("x", message)`, `("!", message)`), the nanoseconds the whole program took to RUN (decoding and encoding excluded), and with `passes > 0`
//! the nanoseconds of each instruction (the best of `passes` passes, each instruction executed `inner` times in a row). A defect of the program is a `ValueError` naming the instruction.

use std::fmt::Display;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::time::Instant;

use cftuv_radical::program::{run, run_timed, Done, Instruction};
use cftuv_radical::{IBig, Work};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyInt, PyList, PyString, PyTuple};
use pyo3::IntoPyObjectExt;

use crate::pyobj::{ibig_from_int, int_of_ibig, int_of_ubig};

fn defect(index: usize, why: impl Display) -> PyErr {
    PyValueError::new_err(format!("radical_v2_eval: instruction {index}: {why}"))
}

fn int_of(index: usize, item: &Bound<'_, PyAny>) -> PyResult<IBig> {
    if !item.is_instance_of::<PyInt>() {
        return Err(defect(index, format!("an int expected, not {}", item.get_type().name()?)));
    }
    ibig_from_int(item)
}

fn ints_of(index: usize, item: &Bound<'_, PyAny>) -> PyResult<Vec<IBig>> {
    let mut out = Vec::new();
    for entry in item.try_iter()? {
        out.push(int_of(index, &entry?)?);
    }
    Ok(out)
}

fn register_of(index: usize, item: &Bound<'_, PyAny>) -> PyResult<usize> {
    item.extract::<usize>().map_err(|_| defect(index, "a register (a non-negative int) expected"))
}

fn decode(index: usize, item: &Bound<'_, PyAny>) -> PyResult<Instruction> {
    let tuple = item.cast::<PyTuple>().map_err(|_| defect(index, "not a tuple"))?;
    if tuple.is_empty() {
        return Err(defect(index, "empty"));
    }
    let name_item = tuple.get_item(0)?;
    let name = name_item.cast::<PyString>().map_err(|_| defect(index, "the operation is not a str"))?.to_str()?.to_string();
    let count = tuple.len() - 1;
    let want = |expected: usize| if count == expected { Ok(()) } else { Err(defect(index, format!("{name}: {expected} operands expected, got {count}"))) };
    let at = |position: usize| tuple.get_item(position);
    Ok(match name.as_str() {
        "zero" => {
            want(0)?;
            Instruction::Zero
        }
        "rational" => {
            want(2)?;
            Instruction::Rational(int_of(index, &at(1)?)?, int_of(index, &at(2)?)?)
        }
        "sqrt" => {
            want(2)?;
            Instruction::Sqrt(int_of(index, &at(1)?)?, int_of(index, &at(2)?)?)
        }
        "terms" => {
            want(1)?;
            let mut pairs = Vec::new();
            for pair in at(1)?.try_iter()? {
                let pair = pair?;
                let triple = ints_of(index, &pair)?;
                let [radicand, numerator, denominator]: [IBig; 3] = triple.try_into().map_err(|_| defect(index, "terms: each pair is (r, n, d)"))?;
                pairs.push((radicand, numerator, denominator));
            }
            Instruction::FromTerms(pairs)
        }
        "wire" => {
            want(1)?;
            Instruction::Wire(ints_of(index, &at(1)?)?)
        }
        "text" => {
            want(1)?;
            let text = at(1)?;
            Instruction::Text(text.cast::<PyString>().map_err(|_| defect(index, "text: a str expected"))?.to_str()?.to_string())
        }
        "add" | "sub" | "mul" | "div" | "eq" => {
            want(2)?;
            let (a, b) = (register_of(index, &at(1)?)?, register_of(index, &at(2)?)?);
            match name.as_str() {
                "add" => Instruction::Add(a, b),
                "sub" => Instruction::Sub(a, b),
                "mul" => Instruction::Mul(a, b),
                "div" => Instruction::Div(a, b),
                _ => Instruction::Equals(a, b),
            }
        }
        "inverse" | "neg" | "text_of" | "wire_of" | "as_rational" | "sep_bits" => {
            want(1)?;
            let a = register_of(index, &at(1)?)?;
            match name.as_str() {
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
            Instruction::Scaled(register_of(index, &at(1)?)?, int_of(index, &at(2)?)?, int_of(index, &at(3)?)?)
        }
        "signum" => {
            want(2)?;
            let cap = at(2)?;
            Instruction::Signum(register_of(index, &at(1)?)?, if cap.is_none() { None } else { Some(int_of(index, &cap)?) })
        }
        "enclosure" => {
            want(2)?;
            Instruction::Enclosure(register_of(index, &at(1)?)?, int_of(index, &at(2)?)?)
        }
        other => return Err(defect(index, format!("unknown operation {other:?}"))),
    })
}

fn work_tuple<'py>(py: Python<'py>, work: &Work) -> PyResult<Bound<'py, PyTuple>> {
    PyTuple::new(py, work.wire())
}

fn wire_tuple<'py>(py: Python<'py>, wire: &[IBig]) -> PyResult<Bound<'py, PyTuple>> {
    let mut items = Vec::with_capacity(wire.len());
    for item in wire {
        items.push(int_of_ibig(py, item)?);
    }
    PyTuple::new(py, items)
}

fn fraction_tuple<'py>(py: Python<'py>, numerator: &IBig, denominator: &cftuv_radical::UBig) -> PyResult<Bound<'py, PyTuple>> {
    PyTuple::new(py, [int_of_ibig(py, numerator)?, int_of_ubig(py, denominator)?])
}

fn encode<'py>(py: Python<'py>, done: &Done) -> PyResult<Bound<'py, PyAny>> {
    match done {
        Done::Value(value, work) => ("v", wire_tuple(py, &value.wire())?, work_tuple(py, work)?).into_bound_py_any(py),
        Done::Sign(sign, work) => ("s", *sign, work_tuple(py, work)?).into_bound_py_any(py),
        Done::Enclosure(low, high, work) => ("e", fraction_tuple(py, &low.0, &low.1)?, fraction_tuple(py, &high.0, &high.1)?, work_tuple(py, work)?).into_bound_py_any(py),
        Done::Text(text) => ("t", text.as_str()).into_bound_py_any(py),
        Done::Wire(wire) => ("w", wire_tuple(py, wire)?).into_bound_py_any(py),
        Done::Rational(None) => ("q", py.None()).into_bound_py_any(py),
        Done::Rational(Some((numerator, denominator))) => ("q", fraction_tuple(py, numerator, denominator)?).into_bound_py_any(py),
        Done::Bool(value) => ("b", *value).into_bound_py_any(py),
        Done::Int(value) => ("i", int_of_ubig(py, value)?).into_bound_py_any(py),
        Done::Refused(refusal) => ("r", refusal.code.code(), work_tuple(py, &refusal.work)?, refusal.detail.as_str()).into_bound_py_any(py),
        Done::ZeroDivision(message) => ("z", message.as_str()).into_bound_py_any(py),
        Done::ValueError(message) => ("x", message.as_str()).into_bound_py_any(py),
        Done::BoundViolated(message) => ("!", message.as_str()).into_bound_py_any(py),
        Done::Invalid(message) => ("?", message.as_str()).into_bound_py_any(py),
    }
}

/// `radical_v2_eval(program, passes=0, inner=1) -> (outcomes, run_ns, timings or None)`; see the module note.
#[pyfunction]
#[pyo3(signature = (program, passes=0, inner=1))]
pub fn radical_v2_eval<'py>(py: Python<'py>, program: &Bound<'py, PyAny>, passes: usize, inner: usize) -> PyResult<Bound<'py, PyTuple>> {
    let mut decoded: Vec<Instruction> = Vec::new();
    for (index, item) in program.try_iter()?.enumerate() {
        decoded.push(decode(index, &item?)?);
    }
    let outcome = py.detach(|| {
        catch_unwind(AssertUnwindSafe(|| {
            let started = Instant::now();
            let done = run(&decoded);
            let run_ns = started.elapsed().as_nanos() as u64;
            let timings = if passes > 0 { Some(run_timed(&decoded, passes, inner).1) } else { None };
            (done, run_ns, timings)
        }))
    });
    match outcome {
        Ok((done, run_ns, timings)) => {
            let mut outcomes = Vec::with_capacity(done.len());
            for item in &done {
                outcomes.push(encode(py, item)?);
            }
            let timings = match timings {
                Some(values) => PyList::new(py, values)?.into_any(),
                None => py.None().into_bound(py),
            };
            PyTuple::new(py, [PyList::new(py, outcomes)?.into_any(), run_ns.into_bound_py_any(py)?, timings])
        }
        Err(panic) => Err(PyRuntimeError::new_err(format!("radical_v2_eval: native panic: {}", crate::panic_message(&panic)))),
    }
}
