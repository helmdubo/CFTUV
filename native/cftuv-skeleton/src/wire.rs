//! Reading and writing the boundary values of the skeleton seams (`cftuv_core::codec::Value`): lines, times, points, strings, lists.
//!
//! A string travels as `[None, int]` (UTF-8 bytes plus a final 0x01 sentinel, little endian) because the codec has no tag for strings. A line is
//! `[a, b, c, q, ident]` (`ident` is the Python `id()` of the object), a time `[dividend, divisor]`, a point `[x, y]`.

use cftuv_core::codec::Value;
use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::line::SupportLine;
use crate::time::{EventPoint, EventTime};

/// A request the seam cannot read (a bug of the caller, never an answer).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeamError(pub String);

impl std::fmt::Display for SeamError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "skeleton seam: {}", self.0)
    }
}

impl std::error::Error for SeamError {}

impl From<cftuv_core::codec::DecodeError> for SeamError {
    fn from(error: cftuv_core::codec::DecodeError) -> SeamError {
        SeamError(format!("cannot decode the request ({error})"))
    }
}

pub fn bad(what: &str) -> SeamError {
    SeamError(format!("bad argument: {what}"))
}

pub type Wire<T> = Result<T, SeamError>;

pub fn list<'a>(value: &'a Value, what: &str) -> Wire<&'a [Value]> {
    match value {
        Value::List(items) => Ok(items),
        _ => Err(bad(what)),
    }
}

pub fn fixed<'a, const N: usize>(value: &'a Value, what: &str) -> Wire<&'a [Value; N]> {
    list(value, what)?.try_into().map_err(|_| bad(what))
}

pub fn int_of(value: &Value, what: &str) -> Wire<IBig> {
    match value {
        Value::Int(number) => Ok(number.clone()),
        _ => Err(bad(what)),
    }
}

pub fn i64_of(value: &Value, what: &str) -> Wire<i64> {
    i64::try_from(&int_of(value, what)?).map_err(|_| bad(what))
}

pub fn u64_of(value: &Value, what: &str) -> Wire<u64> {
    u64::try_from(&int_of(value, what)?).map_err(|_| bad(what))
}

pub fn u32_of(value: &Value, what: &str) -> Wire<u32> {
    u32::try_from(&int_of(value, what)?).map_err(|_| bad(what))
}

pub fn flag_of(value: &Value, what: &str) -> Wire<bool> {
    match value {
        Value::Bool(flag) => Ok(*flag),
        _ => Err(bad(what)),
    }
}

/// A rational given as an `int` or a `Fraction`.
pub fn rat_of(value: &Value, what: &str) -> Wire<Rat> {
    match value {
        Value::Int(number) => Ok(Rat::from_int(number.clone())),
        Value::Frac(number) => Ok(number.clone()),
        _ => Err(bad(what)),
    }
}

pub fn sum_of<'a>(value: &'a Value, what: &str) -> Wire<&'a SqrtSum> {
    match value {
        Value::Sum(found) => Ok(found),
        _ => Err(bad(what)),
    }
}

pub fn ubig_of(value: &Value, what: &str) -> Wire<UBig> {
    let number = int_of(value, what)?;
    if num::is_negative(&number) {
        return Err(bad(what));
    }
    Ok(num::magnitude(&number))
}

/// A string: `[None, int]`, the int holding the UTF-8 bytes and a final `0x01`.
pub fn str_of(value: &Value, what: &str) -> Wire<String> {
    let [Value::None, Value::Int(number)] = fixed::<2>(value, what)? else {
        return Err(bad(what));
    };
    if num::is_negative(number) {
        return Err(bad(what));
    }
    let mut bytes = num::magnitude_le_bytes(&num::magnitude(number));
    if bytes.pop() != Some(1) {
        return Err(bad(what));
    }
    String::from_utf8(bytes).map_err(|_| bad(what))
}

pub fn str_value(text: &str) -> Value {
    let mut bytes = text.as_bytes().to_vec();
    bytes.push(1);
    Value::List(vec![Value::None, Value::Int(IBig::from(UBig::from_le_bytes(&bytes)))])
}

pub fn int(number: impl Into<IBig>) -> Value {
    Value::Int(number.into())
}

/// An optional value: `None` stays none.
pub fn optional<T>(value: &Value, read: impl FnOnce(&Value) -> Wire<T>) -> Wire<Option<T>> {
    match value {
        Value::None => Ok(None),
        other => read(other).map(Some),
    }
}

/// `[a, b, c, q, ident]`.
pub fn line_of(value: &Value) -> Wire<SupportLine> {
    let [a, b, c, q, ident] = fixed::<5>(value, "a support line")?;
    let q = rat_of(q, "a line speed")?;
    SupportLine::new(i64_of(a, "line a")?, i64_of(b, "line b")?, i128::try_from(&int_of(c, "line c")?).map_err(|_| bad("line c"))?, q, u64_of(ident, "a line identity")?).map_err(|error| SeamError(format!("unsupported: {error:?}")))
}

/// `[dividend, divisor]`.
pub fn time_of(value: &Value) -> Wire<EventTime> {
    let [dividend, divisor] = fixed::<2>(value, "a time")?;
    Ok(EventTime::new(rat_of(dividend, "a time dividend")?, sum_of(divisor, "a time divisor")?.clone()))
}

/// `[x, y]`.
pub fn point_of(value: &Value) -> Wire<EventPoint> {
    let [x, y] = fixed::<2>(value, "a point")?;
    Ok(EventPoint { x: sum_of(x, "a point x")?.clone(), y: sum_of(y, "a point y")?.clone() })
}

/// A rational as the oracle holds it: a `Fraction` (the dividend of a time is one even when it is whole).
pub fn frac_value(value: &Rat) -> Value {
    Value::Frac(value.clone())
}

/// A speed as the oracle holds it after `normalized_speed`: an `int` when whole, else a `Fraction`.
pub fn speed_value(value: &Rat) -> Value {
    if value.is_integer() {
        Value::Int(value.numerator().clone())
    } else {
        Value::Frac(value.clone())
    }
}

pub fn time_value(time: &EventTime) -> Value {
    Value::List(vec![frac_value(&time.dividend), Value::Sum(time.divisor.clone())])
}

pub fn point_value(point: &EventPoint) -> Value {
    Value::List(vec![Value::Sum(point.x.clone()), Value::Sum(point.y.clone())])
}

/// A line without its identity as the seams answer it: `[a, b, c, q]` (`q` as the oracle holds it, an `int` when whole).
pub fn line_answer(line: &SupportLine) -> Value {
    Value::List(vec![int(line.a), int(line.b), int(line.c), speed_value(&line.q)])
}

/// A line of four elements (`[a, b, c, q]`, identity zero) or five (`[a, b, c, q, ident]`).
pub fn line_any_of(value: &Value) -> Wire<SupportLine> {
    match list(value, "a support line")? {
        [a, b, c, q] => SupportLine::new(i64_of(a, "line a")?, i64_of(b, "line b")?, i128::try_from(&int_of(c, "line c")?).map_err(|_| bad("line c"))?, rat_of(q, "a line speed")?, 0)
            .map_err(|error| SeamError(format!("unsupported: {error:?}"))),
        _ => line_of(value),
    }
}

/// An optional sum.
pub fn sum_option_value(sum: Option<SqrtSum>) -> Value {
    sum.map_or(Value::None, Value::Sum)
}
