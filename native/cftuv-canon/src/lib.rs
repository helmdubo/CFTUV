//! Canonicalization of radicals: the deterministic exact-work budget, CPython's `random.Random`, integer
//! factorization and the process canonicalization memory (`exact_sqrt_sum.py` lines 88-1135).
//! Separate crate so it builds and tests independently of the `SqrtSum` arithmetic in `cftuv-core`.
//!
//! Python is the oracle. Equality covers the answers AND the cost: the six budget articles after any call
//! sequence, the point of exhaustion, and the four memory tables with their insertion order.

pub mod budget;
pub mod factor;
mod fxhash;
pub mod memory;
mod mont;
pub mod ordered;
pub mod pyrandom;

pub use budget::{BudgetConfigError, BudgetMode, Exhausted, Operation, WorkBudget, ARTICLES};
pub use factor::{CanonError, Pairs};
pub use memory::{
    pick_prime_from_universe, support_from_prime_universe, CanonMemory, MemOp, MemoryDelta, MemoryMarker, MemorySync, MemoryState, QValue, Request,
    Split, Support, TableSync, UniverseRecord,
};
