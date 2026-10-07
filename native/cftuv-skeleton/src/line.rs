//! `SupportLineV1` (`event_time.py`): the supporting line `a*x + b*y = c + t*sqrt(q)` of a front edge, with INTEGER `a`, `b`, `c` and a rational
//! non-negative `q` (the squared speed in units of the normal).
//!
//! `a`, `b` are machine integers (|v| < 2^62) and `c` a 128-bit one (|v| < 2^124): the field polygons reach edges of ~2^49 and `c` of ~2^65, and the
//! determinants the laws take of them (`a*b - a*b`, `a*a + b*b`, `a*a' + b*b'`) then fit `i128` exactly; a Python int beyond the limits is a named
//! `Unsupported`, never a wrap. `ident` is the IDENTITY of the Python object (`id(line)`): `exact_candidate_view` keys two of its memos by it, and which lookups hit decides the
//! sign counters, so the port keeps the identity the host gave the line instead of comparing by value.

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;

use crate::error::{SkelError, SkelResult};

/// `|a|, |b| < LINE_LIMIT`.
pub const LINE_LIMIT: i64 = 1 << 62;
/// `|c| < OFFSET_LIMIT`.
pub const OFFSET_LIMIT: i128 = 1 << 124;

#[derive(Debug, Clone)]
pub struct SupportLine {
    pub a: i64,
    pub b: i64,
    pub c: i128,
    /// The normalised speed: an `int` stays one, a `Fraction` with denominator 1 became one (the type is not observable here, the value is).
    pub q: Rat,
    /// `int(q)`: the radicand a position hydration names when the budget runs out (`exhaustion_detail` prints its bit length).
    pub q_int: UBig,
    pub ident: u64,
}

/// The Python `str` of a `Fraction` or `int` (`3`, `-1/2`).
pub fn rat_text(value: &Rat) -> String {
    if value.is_integer() {
        value.numerator().to_string()
    } else {
        format!("{}/{}", value.numerator(), value.denominator())
    }
}

/// The value of a line (identity left out): what `PositionMemoV1` compares keys by.
pub type LineValue = (i64, i64, i128, Rat);

impl SupportLine {
    pub fn new(a: i64, b: i64, c: i128, q: Rat, ident: u64) -> SkelResult<SupportLine> {
        for (name, value) in [("a", a), ("b", b)] {
            if value <= -LINE_LIMIT || value >= LINE_LIMIT {
                return Err(SkelError::Unsupported(format!("the support line coefficient {name} = {value} is beyond the machine range of the port")));
            }
        }
        if c <= -OFFSET_LIMIT || c >= OFFSET_LIMIT {
            return Err(SkelError::Unsupported(format!("the support line offset c = {c} is beyond the machine range of the port")));
        }
        let q_int = q.numerator() / IBig::from(q.denominator().clone());
        let q_int = cftuv_core::num::magnitude(&q_int);
        Ok(SupportLine { a, b, c, q, q_int, ident })
    }

    /// `SupportLineV1.with_speed(start, end, q)`: the normal looks LEFT of the walk; a zero-length edge has no line and a negative `q` is
    /// `NegativeSpeedError` (the degenerate edge is asked first, as in the oracle).
    pub fn with_speed(start: (i64, i64), end: (i64, i64), q: Rat, ident: u64) -> SkelResult<SupportLine> {
        let (dx, dy) = (i128::from(end.0) - i128::from(start.0), i128::from(end.1) - i128::from(start.1));
        if dx == 0 && dy == 0 {
            return Err(SkelError::DegenerateEdge(format!("ребро нулевой длины в ({}, {})", start.0, start.1)));
        }
        if q.signum() < 0 {
            return Err(SkelError::NegativeSpeed(format!("q отрицательно: {}", rat_text(&q))));
        }
        let (a, b) = (-dy, dx);
        let c = a * i128::from(start.0) + b * i128::from(start.1);
        let narrow = |value: i128, what: &str| i64::try_from(value).map_err(|_| SkelError::Unsupported(format!("the support line coefficient {what} = {value} is beyond the machine range of the port")));
        SupportLine::new(narrow(a, "a")?, narrow(b, "b")?, c, q, ident)
    }

    /// `SupportLineV1.through(start, end)`: unit speed, `q = |d|^2`.
    pub fn through(start: (i64, i64), end: (i64, i64), ident: u64) -> SkelResult<SupportLine> {
        let (dx, dy) = (i128::from(end.0) - i128::from(start.0), i128::from(end.1) - i128::from(start.1));
        SupportLine::with_speed(start, end, Rat::from_int(IBig::from(dx * dx + dy * dy)), ident)
    }

    /// `a^2 + b^2`: the squared length of the normal (not `q`).
    pub fn normal_squared(&self) -> i128 {
        i128::from(self.a) * i128::from(self.a) + i128::from(self.b) * i128::from(self.b)
    }

    /// `q == 0`: the line does not move.
    pub fn is_stationary(&self) -> bool {
        self.q.is_zero()
    }

    pub fn value(&self) -> LineValue {
        (self.a, self.b, self.c, self.q.clone())
    }

    /// `first.a * second.b - second.a * first.b`.
    pub fn determinant(&self, other: &SupportLine) -> i128 {
        i128::from(self.a) * i128::from(other.b) - i128::from(other.a) * i128::from(self.b)
    }
}
