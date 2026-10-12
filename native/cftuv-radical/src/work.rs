//! `NativeAlgebraWorkV1` and the named refusals of `radical_v2.py`.
//!
//! Nothing here is a counter in a global: every operation RETURNS its [`Work`] (a delta), and a refusal carries the work done before it.

use std::ops::Add;

pub const EXACT_NATIVE_OUTSIDE_FIELD: &str = "EXACT_NATIVE_OUTSIDE_FIELD";
pub const EXACT_NATIVE_SIGN_UNDECIDED: &str = "EXACT_NATIVE_SIGN_UNDECIDED";
pub const EXACT_NATIVE_INVERSE_CLASS_LIMIT: &str = "EXACT_NATIVE_INVERSE_CLASS_LIMIT";

/// More classes than this are not inverted by conjugation (a cost limit, not mathematics).
pub const INVERSE_CLASS_LIMIT: usize = 3;
/// The first rung of the sign ladder; each next rung doubles, the last one is the separation bound.
pub const SIGN_FIRST_BITS: u64 = 64;

/// The price of one exact operation, by article, in the order of the wire `(CLASS_MERGE_CHECKS, CONJUGATION_PRODUCTS, ENCLOSURE_ROUNDS)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Work {
    pub class_merge_checks: u64,
    pub conjugation_products: u64,
    pub enclosure_rounds: u64,
}

impl Work {
    pub const NONE: Work = Work { class_merge_checks: 0, conjugation_products: 0, enclosure_rounds: 0 };

    #[inline]
    pub const fn checks(count: u64) -> Work {
        Work { class_merge_checks: count, conjugation_products: 0, enclosure_rounds: 0 }
    }

    #[inline]
    pub const fn rounds(count: u64) -> Work {
        Work { class_merge_checks: 0, conjugation_products: 0, enclosure_rounds: count }
    }

    /// `NativeAlgebraWorkV1.wire()`.
    #[inline]
    pub const fn wire(&self) -> [u64; 3] {
        [self.class_merge_checks, self.conjugation_products, self.enclosure_rounds]
    }
}

impl Add for Work {
    type Output = Work;

    #[inline]
    fn add(self, other: Work) -> Work {
        Work {
            class_merge_checks: self.class_merge_checks + other.class_merge_checks,
            conjugation_products: self.conjugation_products + other.conjugation_products,
            enclosure_rounds: self.enclosure_rounds + other.enclosure_rounds,
        }
    }
}

/// An answer and its price.
pub type Priced<T> = (T, Work);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RefusalCode {
    /// The value is not a sum of square roots of rationals (a root of a negative number).
    OutsideField,
    /// The caller's precision cap is below the separation bound: a PRICE outcome, the sign itself is always decidable.
    SignUndecided,
    /// More than [`INVERSE_CLASS_LIMIT`] classes to invert by conjugation.
    InverseClassLimit,
}

impl RefusalCode {
    pub const fn code(self) -> &'static str {
        match self {
            RefusalCode::OutsideField => EXACT_NATIVE_OUTSIDE_FIELD,
            RefusalCode::SignUndecided => EXACT_NATIVE_SIGN_UNDECIDED,
            RefusalCode::InverseClassLimit => EXACT_NATIVE_INVERSE_CLASS_LIMIT,
        }
    }
}

/// `RadicalRefusal`: the code, the detail text and the work done before the refusal.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Refusal {
    pub code: RefusalCode,
    pub detail: String,
    pub work: Work,
}

impl Refusal {
    pub fn new(code: RefusalCode, detail: impl Into<String>, work: Work) -> Refusal {
        Refusal { code, detail: detail.into(), work }
    }

    /// `str(exception)`: `CODE: detail`, or the bare code for an empty detail.
    pub fn message(&self) -> String {
        if self.detail.is_empty() {
            self.code.code().to_string()
        } else {
            format!("{}: {}", self.code.code(), self.detail)
        }
    }
}

/// What an operation that can fail raises instead of an answer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Failure {
    /// A named refusal of the algebra.
    Refused(Refusal),
    /// `ZeroDivisionError("division by an exact zero")`: the inverse of the empty sum.
    ZeroDivision,
    /// `RadicalSeparationBoundViolated`: the enclosure at the separation bound still holds zero -- the proof or the code is broken. Never an answer.
    SeparationBoundViolated(String),
}

impl From<Refusal> for Failure {
    fn from(refusal: Refusal) -> Failure {
        Failure::Refused(refusal)
    }
}
