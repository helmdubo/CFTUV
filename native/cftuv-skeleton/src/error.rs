//! What a skeleton operation can refuse with. The host raises the matching Python exception; nothing here formats a text Python
//! compares except the fixed texts of the oracle (one place each).
//!
//! Every refusal is NAMED (rule 4 of `AGENTS.md`). `Unsupported` is the one that is not the oracle's: the port was asked for
//! something it does not carry (a lattice coefficient beyond the machine range, a reference that is not in the snapshot it was
//! given); the host decides what to do with it, it is never an answer.

use cftuv_canon::Exhausted;
use cftuv_core::exact::ExactError;

/// `ZeroDivisorTimeError` text of `event_time.py`.
pub const ZERO_DIVISOR_TIME_TEXT: &str = "знаменатель времени доказанно нулевой";
/// `ParallelSupportLinesError` text of `event_time.py`.
pub const PARALLEL_LINES_TEXT: &str = "прямые параллельны, точки пересечения нет";

#[derive(Debug, Clone, PartialEq)]
pub enum SkelError {
    /// Budget exhaustion and the other refusals of the exact layer.
    Exact(ExactError),
    /// `ZeroDivisorTimeError`: a time whose divisor is proven zero.
    ZeroDivisorTime,
    /// `ParallelSupportLinesError`.
    ParallelSupportLines,
    /// `DegenerateEdgeError`; the text is the whole message (`ребро нулевой длины в (x, y)`).
    DegenerateEdge(String),
    /// `NegativeSpeedError`; the text is the whole message (`q отрицательно: <the str of the number>`).
    NegativeSpeed(String),
    /// `ValueError` with the exact text of the interpreter (`too many values to unpack`).
    Value(String),
    /// The port does not carry this input (never an answer; the host must see it).
    Unsupported(String),
}

impl From<ExactError> for SkelError {
    fn from(error: ExactError) -> SkelError {
        SkelError::Exact(error)
    }
}

impl From<Exhausted> for SkelError {
    fn from(error: Exhausted) -> SkelError {
        SkelError::Exact(error.into())
    }
}

pub type SkelResult<T> = Result<T, SkelError>;
