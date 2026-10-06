//! Native core of the CFTUV envelope kernel: a bit-for-bit port of the Python oracle
//! (`kernel/src/cftuv_envelope`). No Python here; the PyO3 layer lives in `cftuv-python`.
//!
//! Module map (one owner per module while R1 is in flight):
//! - `num`, `rat`, `pyfloat`: big integers, canonical rationals, CPython-exact float conversions;
//! - `sqrt_sum`, `fused`, `products`: `SqrtSumV1` arithmetic (`exact_sqrt_sum.py`, `_fused`, `_radicand_products`);
//! - `exact`: the cost-bearing operations on top of it and of the canonicalization memory (`sign` past the filter,
//!   `divided_by`, `_divide_with_prime_universe`, `radical`, `radical_sum`, `prime_universe_remembered`), with the
//!   budget, the memory, the sign counters and the product cache as explicit `&mut` parameters;
//! - `session`: the persistent native mirror of the canonicalization memory, the budget of a call and the wire
//!   shapes of those operations (the cost header, the answer `[outcome, counts, articles, log, state]`);
//! - `coverage`: the whole `wavefront.coverage._coverage_at` (faces, fronts, clipping, areas) in the oracle's order of
//!   cost-bearing calls, over a partition prepared once;
//! - `float_filter`: certified binary64 filters (`float_filter.py`);
//! - the work budget, CPython `random.Random`, factorization and the canonicalization memory live in the
//!   sibling crate `cftuv-canon`;
//! - `codec`: the boundary buffer format shared with `cftuv_native/codec.py`;
//! - `script`: the differential entry (a whole script of number and cost operations in one buffer); the shim's
//!   `Session.run` is the same entry on a persistent session.

pub mod codec;
pub mod coverage;
pub mod exact;
pub mod float_filter;
pub mod fused;
pub mod num;
pub mod products;
pub mod pyfloat;
pub mod rat;
pub mod script;
pub mod session;
pub mod sqrt_sum;
