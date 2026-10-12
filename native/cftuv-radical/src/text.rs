//! The text and the wire of a `RadicalSumV2` (`canonical_text`, `terms_wire`, `value_of_wire`, `value_of_text`), and the exceptions of the readers with the oracle's messages.
//!
//! TEXT. `Integer(n)` / `Rational(p, q)` for a rational value, `[-]Sqrt(Rational(P, Q))` for one irrational term (identical to `native_exact.canonical_text`,
//! EXACT_SCALAR_TEXT_CANON_V2), `Sum(<term>, <term>, ...)` in `m'` order for more. WIRE: the canonical terms as one flat list of ints `(m', n, d, ...)`; the empty list is zero.
//!
//! The reader of a text mirrors the regular expression of the oracle with ASCII digits (`\d` also takes the Unicode decimal digits, which this reader does not: a text written by
//! `canonical_text` never has one). The `repr` of a text inside a message is Python's for ASCII text; a non-ASCII character is kept unless it is a control character.

use std::fmt::Write;

use crate::arith::{gcd, gcd_signed, magnitude, signed, IBig, UBig};
use crate::canon::Term;
use crate::value::Radical;
use crate::work::{Priced, RefusalCode};

pub const RADICAL_SUM_V2: &str = "RADICAL_SUM_V2";

/// What a reader raises instead of an answer: `ValueError(message)` or `ZeroDivisionError(message)` (a `Rational(p, 0)` text).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReadError {
    Value(String),
    ZeroDivision(String),
}

fn rational_text(numerator: &IBig, denominator: &UBig) -> String {
    if denominator.is_one() {
        format!("Integer({numerator})")
    } else {
        format!("Rational({numerator}, {denominator})")
    }
}

fn term_text(term: &Term) -> String {
    if term.is_rational() {
        return rational_text(&term.numerator, &term.denominator);
    }
    let sign = if term.is_negative() { "-" } else { "" };
    format!("{sign}Sqrt(Rational({}, {}))", &term.radicand / &term.denominator, term.denominator)
}

/// Python's `repr(str)` (see the module note for non-ASCII).
pub fn py_repr(text: &str) -> String {
    let quote = if text.contains('\'') && !text.contains('"') { '"' } else { '\'' };
    let mut out = String::with_capacity(text.len() + 2);
    out.push(quote);
    for c in text.chars() {
        match c {
            c if c == quote || c == '\\' => {
                out.push('\\');
                out.push(c);
            }
            '\t' => out.push_str("\\t"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            c if (c as u32) < 0x20 || c as u32 == 0x7f => {
                let _ = write!(out, "\\x{:02x}", c as u32);
            }
            c if (c as u32) < 0x7f => out.push(c),
            c if c.is_control() => {
                let code = c as u32;
                if code < 0x100 {
                    let _ = write!(out, "\\x{code:02x}");
                } else if code < 0x10000 {
                    let _ = write!(out, "\\u{code:04x}");
                } else {
                    let _ = write!(out, "\\U{code:08x}");
                }
            }
            c => out.push(c),
        }
    }
    out.push(quote);
    out
}

/// `repr(tuple of ints)`.
fn tuple_repr(wire: &[IBig]) -> String {
    match wire {
        [] => "()".to_string(),
        [only] => format!("({only},)"),
        many => format!("({})", many.iter().map(|item| item.to_string()).collect::<Vec<_>>().join(", ")),
    }
}

impl Radical {
    /// `canonical_text(value)`: `Integer(0)`; one term: its EXACT_SCALAR_TEXT_CANON_V2 text; more: `Sum(<term>, ...)` in `m'` order. A function of the value.
    pub fn text(&self) -> String {
        match self.terms() {
            [] => "Integer(0)".to_string(),
            [only] => term_text(only),
            many => format!("Sum({})", many.iter().map(term_text).collect::<Vec<_>>().join(", ")),
        }
    }

    /// `terms_wire(value)`: the canonical terms as one flat list of ints `(m', n, d, ...)` in `m'` order.
    pub fn wire(&self) -> Vec<IBig> {
        let mut flat = Vec::with_capacity(self.terms().len() * 3);
        for term in self.terms() {
            flat.push(signed(&term.radicand));
            flat.push(term.numerator.clone());
            flat.push(signed(&term.denominator));
        }
        flat
    }

    /// `value_of_wire(wire)`, for a wire `wire` wrote: the SHAPE is checked (a `ValueError` with the oracle's text), the class distinctness is trusted.
    ///
    /// Shape: triples; `m'` strictly increasing; `(1, n, d)` with `n != 0`, `d > 0`, `gcd(n, d) = 1`; `(m', +-1, Q)` with `m' > 1`, `Q | m'`, `gcd(m'/Q, Q) = 1`.
    pub fn from_wire(wire: &[IBig]) -> Result<Radical, ReadError> {
        #[allow(clippy::manual_is_multiple_of)] // `is_multiple_of` needs a toolchain newer than the one the workflow may run
        if wire.len() % 3 != 0 {
            return Err(ReadError::Value(format!("{RADICAL_SUM_V2} wire of {} ints is not a list of triples", wire.len())));
        }
        let mut terms: Vec<Term> = Vec::with_capacity(wire.len() / 3);
        let mut previous = IBig::ZERO;
        for triple in wire.chunks(3) {
            let (radicand, numerator, denominator) = (&triple[0], &triple[1], &triple[2]);
            if *radicand <= previous || *denominator <= IBig::ZERO || numerator.is_zero() {
                return Err(ReadError::Value(format!("{RADICAL_SUM_V2} wire {} is not canonical", tuple_repr(wire))));
            }
            let (radicand_u, denominator_u) = (magnitude(radicand), magnitude(denominator));
            if radicand.is_one() {
                if !gcd_signed(numerator, &denominator_u).is_one() {
                    return Err(ReadError::Value(format!("{RADICAL_SUM_V2} wire {} has an unreduced rational term", tuple_repr(wire))));
                }
            } else {
                let quotient = &radicand_u / &denominator_u;
                let remainder_zero = &quotient * &denominator_u == radicand_u;
                if !(*numerator == IBig::ONE || *numerator == IBig::NEG_ONE) || !remainder_zero || !gcd(&quotient, &denominator_u).is_one() {
                    return Err(ReadError::Value(format!("{RADICAL_SUM_V2} wire {} has a non-canonical irrational term", tuple_repr(wire))));
                }
            }
            previous = radicand.clone();
            terms.push(Term { radicand: radicand_u, numerator: numerator.clone(), denominator: denominator_u });
        }
        Ok(Radical::from_canonical_terms(terms))
    }

    /// `value_of_text(text)`: the value of a text made of the `canonical_text` shapes, read BY VALUE (`from_terms`).
    pub fn from_text(text: &str) -> Result<Priced<Radical>, ReadError> {
        let chars: Vec<char> = text.chars().collect();
        let pairs = if chars.starts_with(&['S', 'u', 'm', '(']) && chars.last() == Some(&')') {
            text_pairs(&chars[4..chars.len() - 1])?
        } else {
            let pairs = text_pairs(&chars)?;
            if pairs.len() != 1 {
                return Err(ReadError::Value(format!("{RADICAL_SUM_V2} text {}: several terms outside Sum(...)", py_repr(text))));
            }
            pairs
        };
        match Radical::from_terms(&pairs) {
            Ok(priced) => Ok(priced),
            Err(refusal) => {
                debug_assert!(refusal.code != RefusalCode::OutsideField, "a text has no negative radicand");
                Err(ReadError::Value(refusal.message()))
            }
        }
    }
}

// --------------------------------------------------------------------------
// the text reader (`_TERM_TEXT`, `_text_pairs`)
// --------------------------------------------------------------------------

struct Cursor<'a> {
    chars: &'a [char],
    at: usize,
}

impl Cursor<'_> {
    fn literal(&mut self, text: &str) -> bool {
        let mut at = self.at;
        for expected in text.chars() {
            if self.chars.get(at) != Some(&expected) {
                return false;
            }
            at += 1;
        }
        self.at = at;
        true
    }

    /// `\d+` (ASCII digits).
    fn digits(&mut self) -> Option<UBig> {
        let start = self.at;
        while self.chars.get(self.at).is_some_and(|c| c.is_ascii_digit()) {
            self.at += 1;
        }
        if self.at == start {
            return None;
        }
        let text: String = self.chars[start..self.at].iter().collect();
        Some(UBig::from_str_radix(&text, 10).expect("ASCII digits"))
    }

    /// `-?\d+`.
    fn integer(&mut self) -> Option<IBig> {
        let start = self.at;
        let negative = self.literal("-");
        match self.digits() {
            Some(magnitude) => Some(if negative { -signed(&magnitude) } else { signed(&magnitude) }),
            None => {
                self.at = start;
                None
            }
        }
    }
}

/// One `(r, c)` pair of `from_terms` as `(radicand, numerator, denominator)`.
type Pair = (IBig, IBig, UBig);

enum Matched {
    Pair(Pair),
    ZeroDivision(String),
    DividesByZero,
}

/// `_TERM_TEXT.match(body, at)`: the three shapes tried in the regular expression's order; `None` when none matches at `at`.
fn match_term(chars: &[char], at: usize) -> Option<(Matched, usize)> {
    let mut cursor = Cursor { chars, at };
    if cursor.literal("Integer(") {
        if let Some(value) = cursor.integer() {
            if cursor.literal(")") {
                return Some((Matched::Pair((IBig::ONE, value, UBig::ONE)), cursor.at));
            }
        }
        return None;
    }
    let mut cursor = Cursor { chars, at };
    if cursor.literal("Rational(") {
        if let Some(numerator) = cursor.integer() {
            if cursor.literal(", ") {
                if let Some(denominator) = cursor.digits() {
                    if cursor.literal(")") {
                        let end = cursor.at;
                        if denominator.is_zero() {
                            return Some((Matched::ZeroDivision(format!("Fraction({numerator}, 0)")), end));
                        }
                        let (numerator, denominator) = crate::arith::reduced(numerator, denominator);
                        return Some((Matched::Pair((IBig::ONE, numerator, denominator)), end));
                    }
                }
            }
        }
        return None;
    }
    let mut cursor = Cursor { chars, at };
    let negative = cursor.literal("-");
    if cursor.literal("Sqrt(Rational(") {
        if let Some(p) = cursor.digits() {
            if cursor.literal(", ") {
                if let Some(q) = cursor.digits() {
                    if cursor.literal("))") {
                        if q.is_zero() {
                            return Some((Matched::DividesByZero, cursor.at));
                        }
                        // sqrt(p/q) = sqrt(p q) / q
                        let radicand = signed(&(&p * &q));
                        return Some((Matched::Pair((radicand, if negative { IBig::NEG_ONE } else { IBig::ONE }, q)), cursor.at));
                    }
                }
            }
        }
    }
    None
}

fn text_pairs(body: &[char]) -> Result<Vec<Pair>, ReadError> {
    let whole: String = body.iter().collect();
    let cannot_read = |at: usize| ReadError::Value(format!("{RADICAL_SUM_V2} cannot read {} at {at}", py_repr(&whole)));
    let mut pairs = Vec::new();
    let mut at = 0usize;
    loop {
        let Some((matched, end)) = match_term(body, at) else {
            return Err(cannot_read(at));
        };
        match matched {
            Matched::Pair(pair) => pairs.push(pair),
            Matched::ZeroDivision(message) => return Err(ReadError::ZeroDivision(message)),
            Matched::DividesByZero => return Err(ReadError::Value(format!("{RADICAL_SUM_V2} text {} divides by zero", py_repr(&whole)))),
        }
        at = end;
        if at == body.len() {
            return Ok(pairs);
        }
        if !(body.get(at) == Some(&',') && body.get(at + 1) == Some(&' ')) {
            return Err(cannot_read(at));
        }
        at += 2;
    }
}
