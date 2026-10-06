//! The boundary buffer format shared with `cftuv_native/codec.py`: compact, little-endian, no decimal strings.
//!
//! ```text
//! uint      LEB128 (7 bits per byte, high bit = more)
//! int       uint header = (byte_count << 1) | negative, then the magnitude as little-endian bytes (zero: header 0)
//! rational  int numerator, int denominator (header sign bit 0, denominator > 0)
//! coef      flag u8: 0 = Python int            -> int
//!                    1 = Fraction, denominator 1 -> int numerator
//!                    2 = Fraction                -> rational
//! sum       uint term count, then per term: int radicand (>= 0), coef
//! value     tag u8 then payload:
//!           0 None | 1 False | 2 True | 3 int | 4 rational (a Fraction) | 5 float (8 bytes, bit pattern)
//!           6 sum | 7 list (uint count, values) | 8 error (u8 code)
//! ```
//!
//! Python decodes this without re-normalising a Fraction: the numerator and denominator go straight into
//! the `Fraction` slots. Rust decodes trusting that a rational is canonical (Python's always are); `strict`
//! mode checks the lowest-terms invariant too and is what the differential harness runs under.

use crate::num::{self, IBig, UBig};
use crate::rat::{Coef, Rat};
use crate::sqrt_sum::{NonCanonical, SqrtSum, Term};

pub const TAG_NONE: u8 = 0;
pub const TAG_FALSE: u8 = 1;
pub const TAG_TRUE: u8 = 2;
pub const TAG_INT: u8 = 3;
pub const TAG_FRAC: u8 = 4;
pub const TAG_FLOAT: u8 = 5;
pub const TAG_SUM: u8 = 6;
pub const TAG_LIST: u8 = 7;
pub const TAG_ERROR: u8 = 8;

pub const COEF_INT: u8 = 0;
pub const COEF_FRACTION_INTEGRAL: u8 = 1;
pub const COEF_FRACTION: u8 = 2;

/// Named failures a computation hands back instead of raising (the Python exception classes of the oracle).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    /// `OverflowError`
    Overflow = 1,
    /// `ZeroDivisionError`
    ZeroDivision = 2,
    /// `ValueError`
    Value = 3,
}

impl ErrorCode {
    pub fn from_u8(code: u8) -> Option<ErrorCode> {
        match code {
            1 => Some(ErrorCode::Overflow),
            2 => Some(ErrorCode::ZeroDivision),
            3 => Some(ErrorCode::Value),
            _ => None,
        }
    }
}

/// A decoded boundary value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    None,
    Bool(bool),
    Int(IBig),
    Frac(Rat),
    Float(f64),
    Sum(SqrtSum),
    List(Vec<Value>),
    Error(ErrorCode),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeError {
    Truncated,
    BadTag(u8),
    BadFlag(u8),
    BadErrorCode(u8),
    ZeroDenominator,
    NegativeDenominator,
    NotInLowestTerms,
    NegativeRadicand,
    NonCanonicalSum(NonCanonical),
    TooDeep,
    TrailingBytes,
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for DecodeError {}

const MAX_DEPTH: usize = 64;

// --------------------------------------------------------------------------
// writing
// --------------------------------------------------------------------------

#[derive(Debug, Default)]
pub struct Writer {
    bytes: Vec<u8>,
}

impl Writer {
    pub fn new() -> Writer {
        Writer::default()
    }

    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }

    pub fn put_u8(&mut self, byte: u8) {
        self.bytes.push(byte);
    }

    pub fn put_uint(&mut self, mut value: u64) {
        while value >= 0x80 {
            self.bytes.push((value & 0x7f) as u8 | 0x80);
            value >>= 7;
        }
        self.bytes.push(value as u8);
    }

    fn put_magnitude(&mut self, negative: bool, magnitude: &[u8]) {
        self.put_uint(((magnitude.len() as u64) << 1) | negative as u64);
        self.bytes.extend_from_slice(magnitude);
    }

    pub fn put_int(&mut self, value: &IBig) {
        let (negative, bytes) = num::to_sign_le_bytes(value);
        self.put_magnitude(negative, &bytes);
    }

    pub fn put_ubig(&mut self, value: &UBig) {
        self.put_magnitude(false, &num::magnitude_le_bytes(value));
    }

    pub fn put_rat(&mut self, value: &Rat) {
        self.put_int(value.numerator());
        self.put_ubig(value.denominator());
    }

    pub fn put_coef(&mut self, coef: &Coef) {
        let value = coef.value();
        if coef.is_py_int() {
            self.put_u8(COEF_INT);
            self.put_int(value.numerator());
        } else if value.is_integer() {
            self.put_u8(COEF_FRACTION_INTEGRAL);
            self.put_int(value.numerator());
        } else {
            self.put_u8(COEF_FRACTION);
            self.put_rat(value);
        }
    }

    pub fn put_sum(&mut self, value: &SqrtSum) {
        self.put_uint(value.terms().len() as u64);
        for term in value.terms() {
            self.put_ubig(&term.radicand);
            self.put_coef(&term.coef);
        }
    }

    pub fn put_value(&mut self, value: &Value) {
        match value {
            Value::None => self.put_u8(TAG_NONE),
            Value::Bool(false) => self.put_u8(TAG_FALSE),
            Value::Bool(true) => self.put_u8(TAG_TRUE),
            Value::Int(number) => {
                self.put_u8(TAG_INT);
                self.put_int(number);
            }
            Value::Frac(number) => {
                self.put_u8(TAG_FRAC);
                self.put_rat(number);
            }
            Value::Float(number) => {
                self.put_u8(TAG_FLOAT);
                self.bytes.extend_from_slice(&number.to_bits().to_le_bytes());
            }
            Value::Sum(sum) => {
                self.put_u8(TAG_SUM);
                self.put_sum(sum);
            }
            Value::List(items) => {
                self.put_u8(TAG_LIST);
                self.put_uint(items.len() as u64);
                for item in items {
                    self.put_value(item);
                }
            }
            Value::Error(code) => {
                self.put_u8(TAG_ERROR);
                self.put_u8(*code as u8);
            }
        }
    }
}

// --------------------------------------------------------------------------
// reading
// --------------------------------------------------------------------------

#[derive(Debug)]
pub struct Reader<'a> {
    bytes: &'a [u8],
    position: usize,
    strict: bool,
}

impl<'a> Reader<'a> {
    pub fn new(bytes: &'a [u8], strict: bool) -> Reader<'a> {
        Reader { bytes, position: 0, strict }
    }

    pub fn is_finished(&self) -> bool {
        self.position == self.bytes.len()
    }

    pub fn finish(&self) -> Result<(), DecodeError> {
        if self.is_finished() {
            Ok(())
        } else {
            Err(DecodeError::TrailingBytes)
        }
    }

    pub fn take(&mut self, count: usize) -> Result<&'a [u8], DecodeError> {
        let end = self.position.checked_add(count).ok_or(DecodeError::Truncated)?;
        let slice = self.bytes.get(self.position..end).ok_or(DecodeError::Truncated)?;
        self.position = end;
        Ok(slice)
    }

    pub fn get_u8(&mut self) -> Result<u8, DecodeError> {
        Ok(self.take(1)?[0])
    }

    pub fn get_uint(&mut self) -> Result<u64, DecodeError> {
        let mut value = 0u64;
        let mut shift = 0u32;
        loop {
            let byte = self.get_u8()?;
            if shift >= 64 || (shift == 63 && byte > 1) {
                return Err(DecodeError::Truncated);
            }
            value |= u64::from(byte & 0x7f) << shift;
            if byte & 0x80 == 0 {
                return Ok(value);
            }
            shift += 7;
        }
    }

    fn get_magnitude(&mut self) -> Result<(bool, UBig), DecodeError> {
        let header = self.get_uint()?;
        let count = usize::try_from(header >> 1).map_err(|_| DecodeError::Truncated)?;
        let bytes = self.take(count)?;
        Ok((header & 1 == 1, UBig::from_le_bytes(bytes)))
    }

    pub fn get_int(&mut self) -> Result<IBig, DecodeError> {
        let (negative, magnitude) = self.get_magnitude()?;
        Ok(num::from_sign_magnitude(negative, magnitude))
    }

    /// A non-negative integer (a radicand or a denominator): the sign bit must be clear.
    pub fn get_ubig(&mut self) -> Result<UBig, DecodeError> {
        let (negative, magnitude) = self.get_magnitude()?;
        if negative && !magnitude.is_zero() {
            return Err(DecodeError::NegativeRadicand);
        }
        Ok(magnitude)
    }

    fn rational_from(&self, numerator: IBig, denominator: UBig) -> Result<Rat, DecodeError> {
        if denominator.is_zero() {
            return Err(DecodeError::ZeroDenominator);
        }
        if self.strict && !Rat::is_canonical(&numerator, &denominator) {
            return Err(DecodeError::NotInLowestTerms);
        }
        Ok(Rat::from_canonical(numerator, denominator))
    }

    pub fn get_rat(&mut self) -> Result<Rat, DecodeError> {
        let numerator = self.get_int()?;
        let (negative, denominator) = self.get_magnitude()?;
        if negative {
            return Err(DecodeError::NegativeDenominator);
        }
        self.rational_from(numerator, denominator)
    }

    pub fn get_coef(&mut self) -> Result<Coef, DecodeError> {
        match self.get_u8()? {
            COEF_INT => Ok(Coef::int(self.get_int()?)),
            COEF_FRACTION_INTEGRAL => Ok(Coef::fraction(Rat::from_int(self.get_int()?))),
            COEF_FRACTION => Ok(Coef::fraction(self.get_rat()?)),
            other => Err(DecodeError::BadFlag(other)),
        }
    }

    pub fn get_sum(&mut self) -> Result<SqrtSum, DecodeError> {
        let count = self.get_uint()?;
        // every term takes at least three bytes: refuse a count the buffer cannot hold before allocating
        if count > (self.bytes.len() - self.position) as u64 {
            return Err(DecodeError::Truncated);
        }
        let mut terms = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let radicand = self.get_ubig()?;
            let coef = self.get_coef()?;
            terms.push(Term { radicand, coef });
        }
        SqrtSum::from_terms(terms).map_err(DecodeError::NonCanonicalSum)
    }

    pub fn get_value(&mut self) -> Result<Value, DecodeError> {
        self.get_value_at(0)
    }

    fn get_value_at(&mut self, depth: usize) -> Result<Value, DecodeError> {
        if depth > MAX_DEPTH {
            return Err(DecodeError::TooDeep);
        }
        match self.get_u8()? {
            TAG_NONE => Ok(Value::None),
            TAG_FALSE => Ok(Value::Bool(false)),
            TAG_TRUE => Ok(Value::Bool(true)),
            TAG_INT => Ok(Value::Int(self.get_int()?)),
            TAG_FRAC => Ok(Value::Frac(self.get_rat()?)),
            TAG_FLOAT => {
                let bytes: [u8; 8] = self.take(8)?.try_into().expect("eight bytes were taken");
                Ok(Value::Float(f64::from_bits(u64::from_le_bytes(bytes))))
            }
            TAG_SUM => Ok(Value::Sum(self.get_sum()?)),
            TAG_LIST => {
                let count = self.get_uint()?;
                if count > (self.bytes.len() - self.position) as u64 {
                    return Err(DecodeError::Truncated);
                }
                let mut items = Vec::with_capacity(count as usize);
                for _ in 0..count {
                    items.push(self.get_value_at(depth + 1)?);
                }
                Ok(Value::List(items))
            }
            TAG_ERROR => {
                let code = self.get_u8()?;
                ErrorCode::from_u8(code).map(Value::Error).ok_or(DecodeError::BadErrorCode(code))
            }
            other => Err(DecodeError::BadTag(other)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn big(text: &str) -> IBig {
        text.parse().unwrap()
    }

    fn rat(n: &str, d: &str) -> Rat {
        Rat::new(big(n), big(d)).unwrap()
    }

    fn round_trip(value: &Value) -> Value {
        let mut writer = Writer::new();
        writer.put_value(value);
        let bytes = writer.into_bytes();
        let mut reader = Reader::new(&bytes, true);
        let decoded = reader.get_value().unwrap();
        reader.finish().unwrap();
        decoded
    }

    fn sample_sum() -> SqrtSum {
        let huge = "-123456789012345678901234567890123456789012345678901234567890123456789";
        SqrtSum::from_terms(vec![
            Term { radicand: UBig::ONE, coef: Coef::int(big(huge)) },
            Term { radicand: UBig::from(6u8), coef: Coef::fraction(rat("7", "1")) },
            Term { radicand: big("1000000000000000000000000000000000000000000000000000001").try_into().unwrap(), coef: Coef::fraction(rat(huge, "340282366920938463463374607431768211457")) },
        ])
        .unwrap()
    }

    #[test]
    fn varuint_round_trips_across_the_range() {
        for value in [0u64, 1, 127, 128, 255, 16383, 16384, u32::MAX as u64, u64::MAX] {
            let mut writer = Writer::new();
            writer.put_uint(value);
            let bytes = writer.into_bytes();
            let mut reader = Reader::new(&bytes, true);
            assert_eq!(reader.get_uint().unwrap(), value);
            assert!(reader.is_finished());
        }
    }

    #[test]
    fn every_value_kind_round_trips_beyond_i128() {
        let values = [
            Value::None,
            Value::Bool(true),
            Value::Bool(false),
            Value::Int(IBig::ZERO),
            Value::Int(big("-170141183460469231731687303715884105729")),
            Value::Int(big("340282366920938463463374607431768211456")),
            Value::Frac(rat("-6", "4")),
            Value::Frac(rat("5", "1")),
            Value::Float(-0.0),
            Value::Float(f64::MIN_POSITIVE / 4.0),
            Value::Float(f64::NAN),
            Value::Sum(sample_sum()),
            Value::Sum(SqrtSum::zero()),
            Value::List(vec![Value::Int(IBig::ONE), Value::List(vec![]), Value::Error(ErrorCode::Overflow)]),
        ];
        for value in &values {
            let decoded = round_trip(value);
            match (value, &decoded) {
                (Value::Float(a), Value::Float(b)) => assert_eq!(a.to_bits(), b.to_bits()),
                _ => assert_eq!(&decoded, value),
            }
        }
    }

    #[test]
    fn the_coefficient_flag_keeps_int_apart_from_an_integral_fraction() {
        let sum = SqrtSum::from_terms(vec![
            Term { radicand: UBig::ONE, coef: Coef::int(IBig::from(3)) },
            Term { radicand: UBig::from(2u8), coef: Coef::fraction(Rat::from_i64(3)) },
        ])
        .unwrap();
        let Value::Sum(decoded) = round_trip(&Value::Sum(sum.clone())) else { panic!("a sum") };
        assert!(decoded.terms()[0].coef.is_py_int());
        assert!(!decoded.terms()[1].coef.is_py_int());
        assert_eq!(decoded, sum);
    }

    #[test]
    fn malformed_buffers_are_named_not_swallowed() {
        let mut reader = Reader::new(&[TAG_INT, 0x08], true);
        assert_eq!(reader.get_value(), Err(DecodeError::Truncated));
        assert_eq!(Reader::new(&[99], true).get_value(), Err(DecodeError::BadTag(99)));
        assert_eq!(Reader::new(&[TAG_ERROR, 9], true).get_value(), Err(DecodeError::BadErrorCode(9)));
        // 2/4 is not in lowest terms: strict mode refuses it (the lenient mode trusts the sender, and a debug
        // build still asserts the invariant in `Rat::from_canonical`)
        let bytes = [TAG_FRAC, 0x02, 2, 0x02, 4];
        assert_eq!(Reader::new(&bytes, true).get_value(), Err(DecodeError::NotInLowestTerms));
        // zero and negative denominators
        assert_eq!(Reader::new(&[TAG_FRAC, 0x02, 1, 0x00], false).get_value(), Err(DecodeError::ZeroDenominator));
        assert_eq!(Reader::new(&[TAG_FRAC, 0x02, 1, 0x03, 1], false).get_value(), Err(DecodeError::NegativeDenominator));
        // a sum that is not sorted
        let mut writer = Writer::new();
        writer.put_u8(TAG_SUM);
        writer.put_uint(2);
        writer.put_ubig(&UBig::from(3u8));
        writer.put_coef(&Coef::int(IBig::ONE));
        writer.put_ubig(&UBig::from(2u8));
        writer.put_coef(&Coef::int(IBig::ONE));
        let bytes = writer.into_bytes();
        assert!(matches!(Reader::new(&bytes, true).get_value(), Err(DecodeError::NonCanonicalSum(_))));
        // a list count far beyond the buffer must not allocate
        assert_eq!(Reader::new(&[TAG_LIST, 0xff, 0xff, 0xff, 0x7f], true).get_value(), Err(DecodeError::Truncated));
        // nesting depth is bounded
        let mut deep = vec![];
        for _ in 0..80 {
            deep.extend_from_slice(&[TAG_LIST, 1]);
        }
        deep.push(TAG_NONE);
        assert_eq!(Reader::new(&deep, true).get_value(), Err(DecodeError::TooDeep));
    }
}
