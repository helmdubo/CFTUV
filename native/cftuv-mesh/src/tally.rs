//! The `collections.Counter` of the oracle (`tally`): numbers keyed by name, in the order the names were first touched.
//!
//! The ORDER and the ZERO entries are part of the contract: `tally[name] += 0` creates the entry (a Counter is a dict), and the shim builds the
//! Python `Counter` from this list entry by entry, so a Counter equal to the oracle's in everything a reader can see (keys, order, values).

use cftuv_canon::fxhash::FxBuild;
use std::collections::HashMap;

#[derive(Debug, Clone, Default)]
pub struct Tally {
    names: Vec<String>,
    values: Vec<u64>,
    index: HashMap<String, usize, FxBuild>,
}

impl Tally {
    pub fn new() -> Tally {
        Tally::default()
    }

    fn slot(&mut self, name: &str) -> usize {
        if let Some(found) = self.index.get(name) {
            return *found;
        }
        let at = self.names.len();
        self.names.push(name.to_string());
        self.values.push(0);
        self.index.insert(name.to_string(), at);
        at
    }

    /// `tally[name]` (a missing name reads as zero and creates nothing, as `Counter.__missing__`).
    pub fn get(&self, name: &str) -> u64 {
        self.index.get(name).map_or(0, |at| self.values[*at])
    }

    /// `tally[name] += by` (the entry exists afterwards, also for `by == 0`).
    pub fn add(&mut self, name: &str, by: u64) {
        let at = self.slot(name);
        self.values[at] += by;
    }

    /// `tally[name] = max(tally[name], value)`.
    pub fn raise_to(&mut self, name: &str, value: u64) {
        let at = self.slot(name);
        let current = self.values[at];
        self.values[at] = current.max(value);
    }

    /// `tally[name]` read for the `max` of a scratch merge and nothing else: `(name, value)` in first-touch order.
    pub fn items(&self) -> impl Iterator<Item = (&str, u64)> {
        self.names.iter().map(String::as_str).zip(self.values.iter().copied())
    }

    pub fn is_empty(&self) -> bool {
        self.names.is_empty()
    }
}
