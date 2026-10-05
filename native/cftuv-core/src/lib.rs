//! Native core of the CFTUV envelope kernel: a bit-for-bit port of the Python oracle
//! (`kernel/src/cftuv_envelope`). No Python here; the PyO3 layer lives in `cftuv-python`.
//!
//! Module map (one owner per module while R1 is in flight):
//! - `num`, `rat`, `pyfloat`: big integers, canonical rationals, CPython-exact float conversions;
//! - `sqrt_sum`, `fused`, `products`: `SqrtSumV1` arithmetic (`exact_sqrt_sum.py`, `_fused`, `_radicand_products`);
//! - `float_filter`: certified binary64 filters (`float_filter.py`);
//! - the work budget, CPython `random.Random`, factorization and the canonicalization memory live in the
//!   sibling crate `cftuv-canon`;
//! - `codec`: the boundary buffer format shared with `cftuv_native/codec.py`.

pub mod codec;
pub mod float_filter;
pub mod fused;
pub mod num;
pub mod products;
pub mod pyfloat;
pub mod rat;
pub mod sqrt_sum;
