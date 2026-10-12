//! `RadicalSumV2` in Rust: the value-canonical sum of square roots of rationals, a bit-for-bit mirror of the Python oracle
//! (`kernel/src/cftuv_envelope/reference/radical_v2.py`, slice S2), prices, refusals, texts and wires included.
//!
//! A leaf crate on purpose: `dashu-int` and nothing else. It does not touch the `SqrtSumV1` stack (`cftuv-core`: sessions, budgets, the canonicalization memory), so the
//! density/fan core that is ported next can depend on it alone. The oracle imports no sympy and no float; neither does this crate.
//!
//! Module map:
//! - `arith`: `gcd`, `isqrt` (the perfect-square decision), `lcm`, `_reduced` over `dashu-int`;
//! - `work`: `NativeAlgebraWorkV1` ([`Work`]) and the named refusals ([`Refusal`], [`Failure`]);
//! - `canon`: the value-canonical term `(m', +-1, Q)` and the term-list algebra (class merge, add/sub, mul, scaled, inverse by conjugation), counting the class questions as the oracle does;
//! - `ladder`: the integer form, the outward enclosure, the separation bound and the sign ladder (64 bits, then min(2 rung, bound), ...);
//! - `value`: [`Radical`], the public type; `text`: its text, its wire and their readers;
//! - `program`: operation programs and their outcomes, the differential and bench entry shared with the Python oracle (`tools/native_radical_v2_corpus.py`).

pub mod arith;
pub mod canon;
pub mod ladder;
pub mod program;
pub mod text;
pub mod value;
pub mod work;

pub use arith::{IBig, UBig};
pub use canon::Term;
pub use text::ReadError;
pub use value::Radical;
pub use work::{
    Failure, Priced, Refusal, RefusalCode, Work, EXACT_NATIVE_INVERSE_CLASS_LIMIT, EXACT_NATIVE_OUTSIDE_FIELD, EXACT_NATIVE_SIGN_UNDECIDED, INVERSE_CLASS_LIMIT, SIGN_FIRST_BITS,
};
