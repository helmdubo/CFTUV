//! The hash tables of the planning layer: `std`'s, with the hasher the rest of the port uses (`FxBuild`: no `SipHash` of a key whose hash is already made).
//!
//! Nothing here depends on the order a table is iterated in: a table with the default hasher has a different order in every run, so a result that depended on it would not be
//! one. The tables are only asked for membership and for their size, or they are iterated into something that is ordered afterwards.

use cftuv_canon::fxhash::FxBuild;

pub type HashMap<K, V, S = FxBuild> = std::collections::HashMap<K, V, S>;
pub type HashSet<T, S = FxBuild> = std::collections::HashSet<T, S>;
