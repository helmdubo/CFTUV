//! Exact finite decimal/scientific input text supplied by Python str(float(x)).
//! No f64 conversion: 0.1 is 1/10, not the binary64 exact ratio.
use cftuv_core::{num::{self, UBig}, rat::Rat};
pub const MAX_DECIMAL_TEXT_BYTES: usize = 128;
pub const MAX_DECIMAL_EXPONENT_ABS: i32 = 1024;
pub const MAX_DECIMAL_POWER_OF_TEN: usize = 1024;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecimalTextDecline { InputTooLong, NonFinite, InvalidDecimal, ExponentOutOfRange, ScaleOutOfRange }
fn power10(n: usize) -> UBig {
    let mut out = UBig::ONE;
    for _ in 0..n { out = out * UBig::from(10u8); }
    out
}
/// ASCII grammar: sign? (digits ('.' digits*)? | '.' digits+) ([eE] sign? digits+)?
/// Bounds deliberately exceed every finite Python binary64 str spelling.
pub fn decimal_text_to_rat(text: &str) -> Result<Rat, DecimalTextDecline> {
    use DecimalTextDecline::*;
    if text.len() > MAX_DECIMAL_TEXT_BYTES { return Err(InputTooLong); }
    let body = text.strip_prefix('-').or_else(|| text.strip_prefix('+')).unwrap_or(text);
    if body.eq_ignore_ascii_case("nan") || body.eq_ignore_ascii_case("inf") || body.eq_ignore_ascii_case("infinity") { return Err(NonFinite); }
    if body.is_empty() || !body.is_ascii() { return Err(InvalidDecimal); }
    let negative = text.starts_with('-');
    let exponent_at = body.bytes().position(|b| b == b'e' || b == b'E');
    let (mantissa, exponent) = if let Some(at) = exponent_at { (&body[..at], Some(&body[at+1..])) } else { (body, None) };
    let mut exp = 0i32;
    if let Some(raw) = exponent {
        let neg = raw.starts_with('-');
        let digits = raw.strip_prefix('-').or_else(||raw.strip_prefix('+')).unwrap_or(raw);
        if digits.is_empty() || !digits.bytes().all(|b|b.is_ascii_digit()) { return Err(InvalidDecimal); }
        for b in digits.bytes() {
            exp = exp.checked_mul(10).and_then(|x|x.checked_add(i32::from(b-b'0'))).ok_or(ExponentOutOfRange)?;
            if exp > MAX_DECIMAL_EXPONENT_ABS { return Err(ExponentOutOfRange); }
        }
        if neg { exp = -exp; }
    }
    let mut digits = String::with_capacity(mantissa.len());
    let mut dot = false;
    let mut fractional = 0i32;
    for b in mantissa.bytes() {
        if b == b'.' && !dot { dot = true; }
        else if b.is_ascii_digit() { digits.push(char::from(b)); if dot { fractional += 1; } }
        else { return Err(InvalidDecimal); }
    }
    if digits.is_empty() { return Err(InvalidDecimal); }
    let scale = exp - fractional;
    if scale.unsigned_abs() as usize > MAX_DECIMAL_POWER_OF_TEN { return Err(ScaleOutOfRange); }
    let magnitude = UBig::from_str_radix(&digits, 10).map_err(|_|InvalidDecimal)?;
    // Rat::reduced canonicalizes common factors and signed zero to 0/1.
    let (numerator, denominator) = if scale >= 0 { (magnitude * power10(scale as usize), UBig::ONE) }
        else { (magnitude, power10((-scale) as usize)) };
    Ok(Rat::reduced(num::from_sign_magnitude(negative, numerator), denominator))
}
