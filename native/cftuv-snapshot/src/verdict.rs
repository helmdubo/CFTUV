//! The two answers of the port: CLEAN (an attestation) or DECLINED (a name). The port never refuses a snapshot itself.
//!
//! A snapshot with an issue is not "refused" here: the port declines it, and the Python validators, which run exactly as they ran before the port existed, produce the real
//! refusal with its text. So every place where the Python validator would add an issue, raise, or loop is a [`Decline`] here, and a decline is always safe (the cost of the call
//! is paid twice, nothing else changes). What must never happen is the other direction: a clean verdict for a snapshot the Python validators do not pass.

use std::fmt;

/// Why the port did not attest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// A condition under which the Python validator adds an issue or a diagnostic (or raises): the validator is to run and say it.
    Issue,
    /// A record or a law the port does not carry (a developable chart, a near-planar certificate, a float direction, an intrinsic metric...): the validator is to run.
    Unsupported,
    /// A value that is not the canonical shape of its field (the reader; or a number outside what an operation of the oracle accepts).
    Shape,
    /// The process is not in the state the port mirrors (the symbolic backend is not `NATIVE_EXACT`, the decimal context is not the one the subtraction is exact in).
    Mode,
    /// The certified sign did not decide and the exact path is not carried for it.
    Undecided,
}

impl Reason {
    pub const fn name(self) -> &'static str {
        match self {
            Reason::Issue => "ISSUE",
            Reason::Unsupported => "UNSUPPORTED",
            Reason::Shape => "SHAPE",
            Reason::Mode => "MODE",
            Reason::Undecided => "UNDECIDED",
        }
    }

    pub const fn code(self) -> u8 {
        match self {
            Reason::Issue => 1,
            Reason::Unsupported => 2,
            Reason::Shape => 3,
            Reason::Mode => 4,
            Reason::Undecided => 5,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Decline {
    pub reason: Reason,
    pub detail: String,
}

impl fmt::Display for Decline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.reason.name(), self.detail)
    }
}

pub type Check<T> = Result<T, Decline>;

pub fn decline<T>(reason: Reason, detail: impl Into<String>) -> Check<T> {
    Err(Decline { reason, detail: detail.into() })
}

/// The Python validator would add an issue here (`what` names the check).
pub fn issue<T>(what: &str) -> Check<T> {
    decline(Reason::Issue, what)
}

pub fn unsupported<T>(what: impl Into<String>) -> Check<T> {
    decline(Reason::Unsupported, what)
}

/// What a clean verdict carries: the side effects the Python validators leave on the process counters, so that the caller can leave the same ones.
///
/// `certified` and `straddled` are the two counters `reference/angle_certificate_sign.py` writes to `symbolic_backend.BACKEND_COUNTS` for a corner whose angle certificate it checks:
/// `angle_certificate_sign.certified` (the cheap interval path decided both ends) and `angle_certificate_sign.exact_fallback.straddles_zero` (it did not, and the exact path decided).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Attestation {
    pub domains: u32,
    pub corners: u32,
    pub certified: u32,
    pub straddled: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    /// `validate_analysis_snapshot(snapshot, developable_stretch_budget=budget)` alone.
    Analysis,
    /// That, then `validate_reference_geometry_payload(snapshot, domain)` for every domain (`envelope_angle_certificate._reject_snapshot`).
    AnalysisAndReference,
}

/// The state of the process the verdict is a statement about.
#[derive(Clone, Debug)]
pub struct Context {
    pub scope: Scope,
    /// `symbolic_backend.backend_mode() is NATIVE_EXACT` (the cheap angle path is tried only then; under `SYMPY` or `SHADOW` the Python validator takes another road).
    pub native_exact: bool,
    /// `decimal.getcontext().prec`: the subtraction `phi.lower - Decimal(1)` is exact only while its digits fit.
    pub decimal_precision: u32,
    /// The text of `ANALYSIS_SNAPSHOT_SCHEMA_V1` the oracle compares the schema with.
    pub analysis_schema: Vec<u8>,
}
