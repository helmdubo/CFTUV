//! What a clip operation can refuse with. The host shim raises the matching Python exception; the native side never
//! formats a text Python compares except the fixed ones of the oracle (listed here, one place).
//!
//! Every refusal is NAMED (rule 4 of `AGENTS.md`): there is no silent fallback. `Unsupported` is the one refusal
//! the oracle does not have: the native code was asked for something it does not port (a corner grid beyond the lattice
//! range, a hinge cell without a second triangle, ...); the host decides what to do, it is never an answer.

use cftuv_core::exact::ExactError;
use cftuv_core::pyfloat::Overflow;

/// Which Python conversion overflowed (`OverflowError` has a different text for each).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverflowKind {
    /// `float(int)`, `math.sqrt(int)`: "int too large to convert to float".
    IntToFloat,
    /// `int / int` and `float(Fraction)` (a `Fraction` converts as `numerator / denominator`):
    /// "integer division result too large for a float".
    TrueDivision,
}

impl OverflowKind {
    pub fn message(self) -> &'static str {
        match self {
            OverflowKind::IntToFloat => "int too large to convert to float",
            OverflowKind::TrueDivision => "integer division result too large for a float",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ClipError {
    /// Budget exhaustion and the other refusals of the exact layer.
    Exact(ExactError),
    /// `OverflowError` from a float conversion.
    Overflow(OverflowKind),
    /// `ZeroDivisionError` (a float divided by zero).
    ZeroDivision(&'static str),
    /// `ValueError`.
    Value(&'static str),
    /// `MaterializationRefusal(outcome, detail)`: `outcome` is the enum name, the exception text is `outcome: detail`.
    Refusal { outcome: &'static str, detail: String },
    /// `KeyError`: a key of a polygon, a cycle or a seam pair that is no vertex of the domain (`str(exc)` is the `repr` of the key).
    MissingKey(String),
    /// The native port does not cover this input (never an answer; the host must see it).
    Unsupported(String),
}

impl From<ExactError> for ClipError {
    fn from(error: ExactError) -> ClipError {
        ClipError::Exact(error)
    }
}

pub type ClipResult<T> = Result<T, ClipError>;

/// `float(Fraction)` / `float(a / b)` failed: the `Fraction` text of the overflow.
pub fn fraction_overflow(_: Overflow) -> ClipError {
    ClipError::Overflow(OverflowKind::TrueDivision)
}

/// `float(int)` failed.
pub fn int_overflow(_: Overflow) -> ClipError {
    ClipError::Overflow(OverflowKind::IntToFloat)
}

pub const BLEND_ZERO_OUTCOME: &str = "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE";
pub const BLEND_ZERO_DETAIL: &str = "the blended offset normal of a mesh vertex is zero";
pub const NON_FINITE_POINT: &str = "LocalPoint3V1 requires finite coordinates";
pub const FLOAT_DIVISION_BY_ZERO: &str = "float division by zero";
