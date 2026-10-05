//! Test-only differential entry: a whole script of number operations in one buffer, so the harness never
//! crosses the language boundary per operation.
//!
//! ```text
//! request   magic "CFN1", flags u8 (bit 0: strict decoding, bit 1: product memory off),
//!           uint op count, then per op: u8 opcode, uint argument count, argument values (see `codec`)
//! response  uint result count, then one value per op (`Value::Error` for a named failure of the operation)
//! ```
//!
//! Operations are independent: arguments are inline values, results never feed the next operation (the
//! Python harness feeds earlier oracle results forward itself). A buffer the format cannot carry, an unknown
//! opcode or an argument of the wrong shape is a [`ScriptError`], not a result.

use crate::codec::{DecodeError, ErrorCode, Reader, Value, Writer};
use crate::float_filter;
use crate::fused;
use crate::num::{self, IBig, UBig};
use crate::products::{Items, ProductMemo};
use crate::pyfloat;
use crate::rat::{Coef, Rat, ZeroDivision};
use crate::sqrt_sum::{self, IntForm, SignCounts, SignStage, SqrtSum};

pub const MAGIC: &[u8; 4] = b"CFN1";
pub const FLAG_STRICT: u8 = 1;
pub const FLAG_NO_MEMO: u8 = 2;
/// The widest enclosure the script accepts (the oracle shifts by it twice).
pub const MAX_BITS: usize = 1 << 20;

/// `(opcode, name)` of every operation; `cftuv_native/codec.py` carries the same table and a test compares them.
pub const OPS: &[(u8, &str)] = &[
    (1, "ISQRT"),
    (2, "GCD"),
    (3, "LCM"),
    (4, "BIT_LENGTH"),
    (5, "FLOAT_OF_INT"),
    (6, "FLOAT_OF_FRACTION"),
    (7, "MATH_SQRT_INT"),
    (8, "RAT_NEW"),
    (9, "RAT_ADD"),
    (10, "RAT_SUB"),
    (11, "RAT_MUL"),
    (12, "RAT_DIV"),
    (13, "RAT_NEG"),
    (14, "RAT_CMP"),
    (20, "SUM_RATIONAL"),
    (21, "SUM_ADD"),
    (22, "SUM_SUB"),
    (23, "SUM_NEG"),
    (24, "SUM_SCALED"),
    (25, "SUM_SCALED_DIFFERENCE"),
    (26, "SUM_DIFFERENCE_IS_ZERO"),
    (27, "SUM_MUL"),
    (28, "SUM_IS_ZERO"),
    (29, "SUM_IS_RATIONAL"),
    (30, "SUM_AS_RATIONAL"),
    (31, "SUM_ENCLOSURE"),
    (32, "SUM_CERTIFIED_SIGN"),
    (33, "SUM_SIGN_PREFILTER"),
    (34, "INTEGER_FORM"),
    (35, "INTEGER_ENCLOSURE"),
    (36, "INTEGER_CERTIFIED_SIGN"),
    (37, "SCALED_DIFFERENCE_PARTS"),
    (38, "MULTIPLY_INTEGER_ITEMS"),
    (39, "REDUCED_FORM"),
    (40, "SCALED_BY_RECIPROCAL"),
    (41, "FILTERED_SIGN"),
    (42, "DIFFERENCE_FILTERED_SIGN"),
    (50, "ORIENTED_SUM"),
    (51, "PRODUCT_ADDED"),
    (52, "SUM_OF_PRODUCTS"),
    (60, "FF_CENTRE_AND_BOUND"),
    (61, "FF_ORIENTATION_SIGN"),
    (62, "FF_LINE_ESTIMATE"),
    (63, "FF_POLYGON_SIGN"),
    (64, "FF_AFFINE_MAP_VIOLATED"),
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScriptError {
    BadMagic,
    Decode(DecodeError),
    UnknownOpcode(u8),
    /// The arguments of the operation have the wrong count or shape.
    BadArguments(u8),
}

impl std::fmt::Display for ScriptError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScriptError::BadMagic => write!(formatter, "number script: bad magic"),
            ScriptError::Decode(error) => write!(formatter, "number script: cannot decode ({error})"),
            ScriptError::UnknownOpcode(code) => write!(formatter, "number script: unknown opcode {code}"),
            ScriptError::BadArguments(code) => write!(formatter, "number script: bad arguments for opcode {code}"),
        }
    }
}

impl std::error::Error for ScriptError {}

impl From<DecodeError> for ScriptError {
    fn from(error: DecodeError) -> ScriptError {
        ScriptError::Decode(error)
    }
}

/// Runs a request buffer and returns the response buffer.
pub fn run_number_ops(request: &[u8]) -> Result<Vec<u8>, ScriptError> {
    if request.len() < 5 || &request[..4] != MAGIC {
        return Err(ScriptError::BadMagic);
    }
    let flags = request[4];
    let mut reader = Reader::new(&request[5..], flags & FLAG_STRICT != 0);
    let count = reader.get_uint()?;
    let mut memo = if flags & FLAG_NO_MEMO != 0 { ProductMemo::disabled() } else { ProductMemo::new() };
    let mut writer = Writer::new();
    writer.put_uint(count);
    for _ in 0..count {
        let code = reader.get_u8()?;
        let argument_count = reader.get_uint()?;
        let mut arguments = Vec::new();
        for _ in 0..argument_count {
            arguments.push(reader.get_value()?);
        }
        let result = execute(code, &arguments, &mut memo)?;
        writer.put_value(&result);
    }
    reader.finish()?;
    Ok(writer.into_bytes())
}

// --------------------------------------------------------------------------
// argument and result shapes
// --------------------------------------------------------------------------

struct Args<'a> {
    code: u8,
    values: &'a [Value],
}

impl<'a> Args<'a> {
    fn bad(&self) -> ScriptError {
        ScriptError::BadArguments(self.code)
    }

    fn expect(&self, count: usize) -> Result<(), ScriptError> {
        if self.values.len() == count {
            Ok(())
        } else {
            Err(self.bad())
        }
    }

    fn int(&self, index: usize) -> Result<&'a IBig, ScriptError> {
        match self.values.get(index) {
            Some(Value::Int(value)) => Ok(value),
            _ => Err(self.bad()),
        }
    }

    /// A non-negative integer argument.
    fn ubig(&self, index: usize) -> Result<UBig, ScriptError> {
        let value = self.int(index)?;
        if num::is_negative(value) {
            return Err(self.bad());
        }
        Ok(num::magnitude(value))
    }

    /// An enclosure width: a shift count, bounded so that a hostile buffer cannot ask for gigabytes.
    fn bits(&self, index: usize) -> Result<usize, ScriptError> {
        match usize::try_from(self.int(index)?) {
            Ok(bits) if bits <= MAX_BITS => Ok(bits),
            _ => Err(self.bad()),
        }
    }

    /// An `int` or a `Fraction`, read as the rational value (the Python type is irrelevant to the callee).
    fn rat(&self, index: usize) -> Result<Rat, ScriptError> {
        match self.values.get(index) {
            Some(Value::Int(value)) => Ok(Rat::from_int(value.clone())),
            Some(Value::Frac(value)) => Ok(value.clone()),
            _ => Err(self.bad()),
        }
    }

    fn sum(&self, index: usize) -> Result<&'a SqrtSum, ScriptError> {
        match self.values.get(index) {
            Some(Value::Sum(value)) => Ok(value),
            _ => Err(self.bad()),
        }
    }

    fn float(&self, index: usize) -> Result<f64, ScriptError> {
        match self.values.get(index) {
            Some(Value::Float(value)) => Ok(*value),
            _ => Err(self.bad()),
        }
    }

    fn list(&self, index: usize) -> Result<&'a [Value], ScriptError> {
        match self.values.get(index) {
            Some(Value::List(items)) => Ok(items),
            _ => Err(self.bad()),
        }
    }

    /// `[[radicand, numerator], ...]` with radicands of one and above (a canonical sum has no others)
    fn items(&self, index: usize) -> Result<Items, ScriptError> {
        let mut items = Items::new();
        for entry in self.list(index)? {
            match entry {
                Value::List(pair) => match pair.as_slice() {
                    [Value::Int(radicand), Value::Int(numerator)] if radicand > &IBig::ZERO => {
                        items.push((num::magnitude(radicand), numerator.clone()));
                    }
                    _ => return Err(self.bad()),
                },
                _ => return Err(self.bad()),
            }
        }
        Ok(items)
    }

    fn point(&self, value: &'a Value) -> Result<(&'a SqrtSum, &'a SqrtSum), ScriptError> {
        match value {
            Value::List(pair) => match pair.as_slice() {
                [Value::Sum(x), Value::Sum(y)] => Ok((x, y)),
                _ => Err(self.bad()),
            },
            _ => Err(self.bad()),
        }
    }

    fn points(&self, index: usize) -> Result<Vec<(&'a SqrtSum, &'a SqrtSum)>, ScriptError> {
        self.list(index)?.iter().map(|value| self.point(value)).collect()
    }
}

fn error(code: ErrorCode) -> Value {
    Value::Error(code)
}

fn int_list(values: impl IntoIterator<Item = IBig>) -> Value {
    Value::List(values.into_iter().map(Value::Int).collect())
}

fn items_value(items: &Items) -> Value {
    Value::List(
        items
            .iter()
            .map(|(radicand, numerator)| Value::List(vec![Value::Int(IBig::from(radicand.clone())), Value::Int(numerator.clone())]))
            .collect(),
    )
}

fn form_value(form: &IntForm) -> Value {
    Value::List(vec![Value::Int(IBig::from(form.common.clone())), items_value(&form.items)])
}

fn sign_value(sign: Option<i8>) -> Value {
    sign.map_or(Value::None, |sign| Value::Int(IBig::from(sign)))
}

fn counts_value(counts: &SignCounts) -> Value {
    int_list(counts.as_array().iter().map(|count| IBig::from(*count)))
}

fn float_value(result: Result<f64, pyfloat::Overflow>) -> Value {
    result.map_or(error(ErrorCode::Overflow), Value::Float)
}

fn entry_value(entry: Option<(f64, f64)>) -> Value {
    entry.map_or(Value::None, |(first, second)| Value::List(vec![Value::Float(first), Value::Float(second)]))
}

fn coef_value(coef: &Coef) -> Value {
    if coef.is_py_int() {
        Value::Int(coef.value().numerator().clone())
    } else {
        Value::Frac(coef.value().clone())
    }
}

fn zero_division(result: Result<Rat, ZeroDivision>) -> Value {
    result.map_or(error(ErrorCode::ZeroDivision), Value::Frac)
}

// --------------------------------------------------------------------------
// the operations
// --------------------------------------------------------------------------

fn execute(code: u8, values: &[Value], memo: &mut ProductMemo) -> Result<Value, ScriptError> {
    let args = Args { code, values };
    Ok(match code {
        1 => {
            args.expect(1)?;
            if num::is_negative(args.int(0)?) {
                error(ErrorCode::Value)
            } else {
                Value::Int(IBig::from(num::isqrt(&args.ubig(0)?)))
            }
        }
        2 => {
            args.expect(2)?;
            Value::Int(IBig::from(num::gcd_signed(args.int(0)?, args.int(1)?)))
        }
        3 => {
            args.expect(2)?;
            let (left, right) = (num::magnitude(args.int(0)?), num::magnitude(args.int(1)?));
            Value::Int(IBig::from(num::lcm(&left, &right)))
        }
        4 => {
            args.expect(1)?;
            Value::Int(IBig::from(num::bit_length(&num::magnitude(args.int(0)?)) as u64))
        }
        5 => {
            args.expect(1)?;
            float_value(pyfloat::int_to_f64(args.int(0)?))
        }
        6 => {
            args.expect(1)?;
            float_value(pyfloat::rat_to_f64(&args.rat(0)?))
        }
        7 => {
            args.expect(1)?;
            let value = args.int(0)?;
            if num::is_negative(value) {
                // `math.sqrt` converts to float first: a huge negative int is an OverflowError, not a domain error
                match pyfloat::int_to_f64(value) {
                    Err(pyfloat::Overflow) => error(ErrorCode::Overflow),
                    Ok(_) => error(ErrorCode::Value),
                }
            } else {
                float_value(pyfloat::math_sqrt(&args.ubig(0)?))
            }
        }
        8 => {
            args.expect(2)?;
            zero_division(Rat::new(args.int(0)?.clone(), args.int(1)?.clone()))
        }
        9 => {
            args.expect(2)?;
            Value::Frac(args.rat(0)?.add(&args.rat(1)?))
        }
        10 => {
            args.expect(2)?;
            Value::Frac(args.rat(0)?.sub(&args.rat(1)?))
        }
        11 => {
            args.expect(2)?;
            Value::Frac(args.rat(0)?.mul(&args.rat(1)?))
        }
        12 => {
            args.expect(2)?;
            zero_division(args.rat(0)?.div(&args.rat(1)?))
        }
        13 => {
            args.expect(1)?;
            Value::Frac(args.rat(0)?.neg())
        }
        14 => {
            args.expect(2)?;
            Value::Int(IBig::from(args.rat(0)?.cmp(&args.rat(1)?) as i8))
        }
        20..=42 => execute_sums(&args, memo)?,
        50..=52 => execute_fused(&args, memo)?,
        60..=64 => execute_filters(&args)?,
        other => return Err(ScriptError::UnknownOpcode(other)),
    })
}

fn execute_sums(args: &Args, memo: &mut ProductMemo) -> Result<Value, ScriptError> {
    Ok(match args.code {
        20 => {
            args.expect(1)?;
            Value::Sum(SqrtSum::rational(&args.rat(0)?))
        }
        21 => {
            args.expect(2)?;
            Value::Sum(args.sum(0)?.add(args.sum(1)?))
        }
        22 => {
            args.expect(2)?;
            Value::Sum(args.sum(0)?.sub(args.sum(1)?))
        }
        23 => {
            args.expect(1)?;
            Value::Sum(args.sum(0)?.neg())
        }
        24 => {
            args.expect(2)?;
            Value::Sum(args.sum(0)?.scaled(&args.rat(1)?))
        }
        25 => {
            args.expect(4)?;
            Value::Sum(args.sum(0)?.scaled_difference(&args.rat(1)?, args.sum(2)?, &args.rat(3)?))
        }
        26 => {
            args.expect(2)?;
            Value::Bool(args.sum(0)?.difference_is_zero(args.sum(1)?))
        }
        27 => {
            args.expect(2)?;
            Value::Sum(args.sum(0)?.mul(args.sum(1)?, memo))
        }
        28 => {
            args.expect(1)?;
            Value::Bool(args.sum(0)?.is_zero())
        }
        29 => {
            args.expect(1)?;
            Value::Bool(args.sum(0)?.is_rational())
        }
        30 => {
            args.expect(1)?;
            args.sum(0)?.as_rational().map_or(Value::None, |coef| coef_value(&coef))
        }
        31 => {
            args.expect(2)?;
            let (low, high) = args.sum(0)?.enclosure(args.bits(1)?);
            Value::List(vec![Value::Frac(low), Value::Frac(high)])
        }
        32 => {
            args.expect(2)?;
            sign_value(args.sum(0)?.certified_sign(args.bits(1)?))
        }
        33 => {
            args.expect(2)?;
            let mut counts = SignCounts::default();
            let stage = args.sum(0)?.sign_prefilter(args.bits(1)?, &mut counts);
            let decided = match stage {
                SignStage::Decided(sign) => Some(sign),
                SignStage::NeedsConjugation => None,
            };
            Value::List(vec![sign_value(decided), counts_value(&counts)])
        }
        34 => {
            args.expect(1)?;
            form_value(args.sum(0)?.int_form())
        }
        35 => {
            args.expect(2)?;
            let (low, high) = sqrt_sum::integer_enclosure(&args.items(0)?, args.bits(1)?);
            int_list([low, high])
        }
        36 => {
            args.expect(2)?;
            sign_value(sqrt_sum::integer_certified_sign(&args.items(0)?, args.bits(1)?))
        }
        37 => {
            args.expect(4)?;
            form_value(&sqrt_sum::scaled_difference_parts(args.sum(0)?, &args.rat(1)?, args.sum(2)?, &args.rat(3)?))
        }
        38 => {
            args.expect(2)?;
            items_value(&sqrt_sum::multiply_integer_items(&args.items(0)?, &args.items(1)?, memo))
        }
        39 => {
            args.expect(2)?;
            let (common, items) = (args.ubig(0)?, args.items(1)?);
            if common.is_zero() && items.iter().all(|(_, value)| value.is_zero()) {
                // `gcd(0)` is zero and `0 // 0` raises in the oracle
                error(ErrorCode::ZeroDivision)
            } else {
                form_value(&sqrt_sum::reduced_form(&common, &items))
            }
        }
        40 => {
            args.expect(4)?;
            match sqrt_sum::scaled_by_reciprocal(&args.ubig(0)?, &args.items(1)?, &args.ubig(2)?, &args.items(3)?) {
                Ok(sum) => Value::Sum(sum),
                Err(ZeroDivision) => error(ErrorCode::ZeroDivision),
            }
        }
        41 => {
            args.expect(2)?;
            let mut counts = SignCounts::default();
            let sign = sqrt_sum::filtered_sign(&args.items(0)?, args.bits(1)?, &mut counts);
            Value::List(vec![sign_value(sign), counts_value(&counts)])
        }
        42 => {
            args.expect(2)?;
            let mut counts = SignCounts::default();
            let sign = args.sum(0)?.difference_filtered_sign(args.sum(1)?, &mut counts);
            Value::List(vec![sign_value(sign), counts_value(&counts)])
        }
        other => return Err(ScriptError::UnknownOpcode(other)),
    })
}

fn execute_fused(args: &Args, memo: &mut ProductMemo) -> Result<Value, ScriptError> {
    Ok(match args.code {
        50 => {
            args.expect(5)?;
            Value::Sum(fused::oriented_sum(args.sum(0)?, args.sum(1)?, args.int(2)?, args.int(3)?, args.int(4)?))
        }
        51 => {
            args.expect(3)?;
            Value::Sum(fused::product_added(args.sum(0)?, args.sum(1)?, args.sum(2)?, memo))
        }
        52 => {
            args.expect(1)?;
            let mut products = Vec::new();
            for entry in args.list(0)? {
                match entry {
                    Value::List(triple) => match triple.as_slice() {
                        [Value::Sum(left), Value::Sum(right), Value::Int(sign)] => products.push((left, right, sign.clone())),
                        _ => return Err(args.bad()),
                    },
                    _ => return Err(args.bad()),
                }
            }
            Value::Sum(fused::sum_of_products(&products, memo))
        }
        other => return Err(ScriptError::UnknownOpcode(other)),
    })
}

fn execute_filters(args: &Args) -> Result<Value, ScriptError> {
    Ok(match args.code {
        60 => {
            args.expect(1)?;
            entry_value(float_filter::centre_and_bound(args.sum(0)?))
        }
        61 => {
            args.expect(3)?;
            let (first, second, third) = (args.point(&args.values[0])?, args.point(&args.values[1])?, args.point(&args.values[2])?);
            sign_value(float_filter::orientation_sign(first, second, third))
        }
        62 => {
            args.expect(5)?;
            let point = args.point(&args.values[0])?;
            entry_value(float_filter::line_estimate(point, args.float(1)?, args.float(2)?, args.float(3)?, args.float(4)?))
        }
        63 => {
            args.expect(1)?;
            sign_value(float_filter::polygon_sign(&args.points(0)?))
        }
        64 => {
            args.expect(2)?;
            let points: [_; 4] = args.points(0)?.try_into().map_err(|_| args.bad())?;
            let values: [_; 4] = args.points(1)?.try_into().map_err(|_| args.bad())?;
            Value::Bool(float_filter::affine_map_violated(&points, &values))
        }
        other => return Err(ScriptError::UnknownOpcode(other)),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codec::Writer;

    fn request(flags: u8, ops: &[(u8, Vec<Value>)]) -> Vec<u8> {
        let mut writer = Writer::new();
        for byte in MAGIC {
            writer.put_u8(*byte);
        }
        writer.put_u8(flags);
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

    fn run(flags: u8, ops: &[(u8, Vec<Value>)]) -> Vec<Value> {
        let response = run_number_ops(&request(flags, ops)).unwrap();
        let mut reader = Reader::new(&response, true);
        let count = reader.get_uint().unwrap();
        let values = (0..count).map(|_| reader.get_value().unwrap()).collect();
        reader.finish().unwrap();
        values
    }

    #[test]
    fn the_opcode_table_has_no_duplicates() {
        let mut codes: Vec<u8> = OPS.iter().map(|(code, _)| *code).collect();
        codes.sort_unstable();
        codes.dedup();
        assert_eq!(codes.len(), OPS.len());
        let mut names: Vec<&str> = OPS.iter().map(|(_, name)| *name).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), OPS.len());
    }

    #[test]
    fn every_table_entry_is_dispatched() {
        for (code, name) in OPS {
            let result = execute(*code, &[], &mut ProductMemo::new());
            assert!(!matches!(result, Err(ScriptError::UnknownOpcode(_))), "{name} has no handler");
        }
        assert_eq!(execute(250, &[], &mut ProductMemo::new()), Err(ScriptError::UnknownOpcode(250)));
    }

    #[test]
    fn a_script_runs_integer_and_float_operations() {
        let results = run(
            FLAG_STRICT,
            &[
                (1, vec![Value::Int(IBig::from(99))]),
                (1, vec![Value::Int(IBig::from(-1))]),
                (2, vec![Value::Int(IBig::from(0)), Value::Int(IBig::from(0))]),
                (5, vec![Value::Int(IBig::ONE << 1024usize)]),
                (8, vec![Value::Int(IBig::ONE), Value::Int(IBig::ZERO)]),
                (7, vec![Value::Int(IBig::from(2))]),
            ],
        );
        assert_eq!(results[0], Value::Int(IBig::from(9)));
        assert_eq!(results[1], Value::Error(ErrorCode::Value));
        assert_eq!(results[2], Value::Int(IBig::ZERO));
        assert_eq!(results[3], Value::Error(ErrorCode::Overflow));
        assert_eq!(results[4], Value::Error(ErrorCode::ZeroDivision));
        assert_eq!(results[5], Value::Float(2.0f64.sqrt()));
    }

    #[test]
    fn malformed_requests_are_errors_not_results() {
        assert_eq!(run_number_ops(b"nope!"), Err(ScriptError::BadMagic));
        assert_eq!(run_number_ops(&request(0, &[(1, vec![])])), Err(ScriptError::BadArguments(1)));
        assert_eq!(run_number_ops(&request(0, &[(200, vec![])])), Err(ScriptError::UnknownOpcode(200)));
        let mut truncated = request(0, &[(1, vec![Value::Int(IBig::from(4))])]);
        truncated.pop();
        assert!(matches!(run_number_ops(&truncated), Err(ScriptError::Decode(_))));
    }
}
