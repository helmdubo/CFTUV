//! Which outcomes of a whole operation are the ORACLE's and which are the PORT's own refusals, and the knob that forces a port refusal in tests.
//!
//! A whole operation (`clip_geometry`, `coverage_at`) answers with a status code (`clip.rs`, `coverage.rs`). The oracle's outcomes are
//! [`ORACLE_STATUSES`]: the Python kernel would have raised the exception the shim builds from them, AFTER the partial effects it leaves behind (budget articles,
//! sign counters, memory tables, offset normals, the store record, the traces), so the extension applies every effect and the shim raises last.
//!
//! Every other code is a refusal of the PORT: the answer is no answer, the oracle can still produce its own (and its own effects) on the same state, and
//! the host falls back to it on the SAME budget, plane and memory tables. Such a refusal must leave every Python-visible state exactly as it was before the
//! call, so the extension drops the whole outcome of the call (memory log, counters, articles, normal writes, store record, traces) before it touches any
//! host object, and resets the session's mirror (`Session::reset_memory`: the next call reloads the tables whole). The set is closed the other way round
//! ([`is_native_only`] is "not an oracle outcome"): a code this table does not know is a refusal, never an outcome the host would apply.
//!
//! `cftuv_native/cost.py` keeps the same table (`ORACLE_STATUSES`); `tests/test_native_dropin_refusal.py` compares the two through [`oracle_statuses`].

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use cftuv_canon::CanonError;
use cftuv_clip::error::ClipError;
use cftuv_core::exact::ExactError;

/// The status codes that are outcomes of the oracle: 0 ok, 1 exhaustion, 2 negative radicand, 3 zero divisor, 4 failed reconstruction (`cost.OpResult`), 8 a
/// face without a supporting line in a coverage / `OverflowError` in a clip, 9 `ZeroDivisionError`, 10 `ValueError`, 11 `MaterializationRefusal`, 13 `KeyError`.
pub const ORACLE_STATUSES: [u8; 10] = [0, 1, 2, 3, 4, 8, 9, 10, 11, 13];

/// A status the oracle has no outcome for: 5 invalid mirror input, 6 the generic division did not finish, 7 an internal state, 12 unsupported by the port, and any
/// code this table does not know.
pub fn is_native_only(status: u8) -> bool {
    !ORACLE_STATUSES.contains(&status)
}

/// `oracle_statuses() -> list[int]`: [`ORACLE_STATUSES`], for the shim's table to be compared with.
#[pyfunction]
pub fn oracle_statuses() -> Vec<u8> {
    ORACLE_STATUSES.to_vec()
}

/// A refusal of the port that a test can force to happen AFTER the whole operation computed (so the call has real effects to discard), see `Session::force_refusal`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Forced {
    /// Status 12 (`ClipError::Unsupported`); a clip only.
    Unsupported,
    /// Status 5.
    InvalidInput,
    /// Status 6.
    Diverged,
    /// Status 7.
    Internal,
    /// A panic inside the computation (the extension catches it and raises `RuntimeError`).
    Panic,
}

const FORCED_TEXT: &str = "forced by the test knob";

impl Forced {
    pub fn parse(name: &str) -> PyResult<Forced> {
        Ok(match name {
            "unsupported" => Forced::Unsupported,
            "invalid_input" => Forced::InvalidInput,
            "diverged" => Forced::Diverged,
            "internal" => Forced::Internal,
            "panic" => Forced::Panic,
            other => return Err(PyValueError::new_err(format!("unknown forced refusal {other:?}: unsupported, invalid_input, diverged, internal or panic"))),
        })
    }

    /// The exact-layer refusal this knob stands for (`None` for the ones that are not one).
    pub fn exact_error(self) -> Option<ExactError> {
        match self {
            Forced::InvalidInput => Some(ExactError::Canon(CanonError::InvalidInput(FORCED_TEXT))),
            Forced::Diverged => Some(ExactError::Diverged),
            Forced::Internal => Some(ExactError::Internal(FORCED_TEXT)),
            Forced::Unsupported | Forced::Panic => None,
        }
    }

    /// The clip refusal this knob stands for (`None` for a panic).
    pub fn clip_error(self) -> Option<ClipError> {
        match self {
            Forced::Unsupported => Some(ClipError::Unsupported(FORCED_TEXT.to_string())),
            Forced::Panic => None,
            other => other.exact_error().map(ClipError::Exact),
        }
    }

    pub fn panics(self) -> bool {
        self == Forced::Panic
    }
}
