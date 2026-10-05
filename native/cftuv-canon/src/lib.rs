//! Canonicalization of radicals: the deterministic exact-work budget, CPython's `random.Random`, integer
//! factorization and the process canonicalization memory (`exact_sqrt_sum.py` lines 88-1135).
//! Separate crate so it builds and tests independently of the `SqrtSum` arithmetic in `cftuv-core`.

pub mod budget;
pub mod factor;
pub mod memory;
pub mod pyrandom;
