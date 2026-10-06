//! The cross-call cache of the clip stage: exact results that depend only on their operands, kept in the session so the next call (the
//! next alpha of the same patch) finds them.
//!
//! Three caches, all exact (a hit compares its operands STRICTLY, coefficient types included: the representative of a point takes its
//! types from the arithmetic, so a result is handed only to operands that would have produced it):
//!
//! * a CROSSING `first + (second - first) * v0 / (v0 - v1)` is a function of the two end points and the line of the edge: nothing else. The
//!   entry keeps the result AND the ordered questions the original computation asked of the canonicalization memory (`prime_support`,
//!   `squarefree_split`): a hit asks them again against the CURRENT memory and budget, so the budget articles, the tables, their LRU order and
//!   the point of an exhaustion are exactly what a computation would leave;
//! * a VALUE, the orientation of a point against the line of an edge (`oriented_sum`), and a LIFT, `lift_known` of a point in a triangle
//!   (a function of the point and the triangle), cost nothing observable: no budget, no memory, no counter.
//!
//! Each cache is two generations: lookups see both, a store goes to the young one, and when the young one is full it becomes the old one and
//! the old one is dropped; a hit in the old generation is stored young again, so what every call uses stays and what no call uses goes. A cache
//! has no answer to change: dropping any of it only costs the time of computing it again.

use std::sync::Arc;

use cftuv_canon::Request;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::fxhash::FxMap;
use crate::lift::Lifted;
use crate::plane::Triangle;
use crate::point::Point;
use crate::regions::EdgeLine;

/// Entries per generation of the crossing cache (the others are scaled from it): the neighbouring alphas of a width slider
/// reuse the last call or two, and the entries are big (a crossing coordinate has up to 16 terms of 1000+ bits).
pub const CROSSING_LIMIT: usize = 1 << 11;

#[derive(Clone, Debug)]
pub struct CrossingEntry {
    pub first: Arc<Point>,
    pub second: Arc<Point>,
    pub line: EdgeLine,
    /// The result: the crossing point and the hash of its identity.
    pub point: Arc<Point>,
    pub hash: u64,
    /// The questions the computation asked of the memory, in order.
    pub requests: Vec<Request>,
}

/// The exact orientation value of a point against the line of an edge: a function of the two and nothing else (no budget, no memory).
#[derive(Clone, Debug)]
pub struct ValueEntry {
    pub point: Arc<Point>,
    pub line: EdgeLine,
    pub value: Arc<SqrtSum>,
}

/// The lift of a point in a triangle (`lift_known`): a function of the point and the triangle (the left fold of the
/// blended normal included), with no cost of its own. A failing lift is never kept.
#[derive(Clone, Debug)]
pub struct LiftEntry {
    pub point: Arc<Point>,
    pub triangle: Arc<Triangle>,
    pub lifted: Lifted,
}

/// Two generations of buckets keyed by a 64-bit hash (the entries of a bucket are told apart by strict comparison of their operands).
#[derive(Debug)]
struct Generations<T> {
    young: FxMap<u64, Vec<T>>,
    old: FxMap<u64, Vec<T>>,
    /// Entries in the young generation.
    count: usize,
}

impl<T> Default for Generations<T> {
    fn default() -> Generations<T> {
        Generations { young: FxMap::default(), old: FxMap::default(), count: 0 }
    }
}

impl<T: Clone> Generations<T> {
    /// The entry of `key` that `same` accepts. One found in the old generation is stored young again (`limit`: the size of a generation).
    fn find(&mut self, key: u64, limit: usize, same: impl Fn(&T) -> bool) -> Option<T> {
        if let Some(found) = self.young.get(&key).and_then(|bucket| bucket.iter().find(|entry| same(entry))) {
            return Some(found.clone());
        }
        let found = self.old.get(&key).and_then(|bucket| bucket.iter().find(|entry| same(entry)))?.clone();
        self.store(key, found.clone(), limit);
        Some(found)
    }

    fn store(&mut self, key: u64, entry: T, limit: usize) {
        if self.count >= limit {
            self.old = std::mem::take(&mut self.young);
            self.count = 0;
        }
        self.young.entry(key).or_default().push(entry);
        self.count += 1;
    }

    fn len(&self) -> usize {
        self.count + self.old.values().map(Vec::len).sum::<usize>()
    }

    fn clear(&mut self) {
        self.young.clear();
        self.old.clear();
        self.count = 0;
    }
}

#[derive(Debug, Default)]
pub struct Warm {
    crossings: Generations<CrossingEntry>,
    values: Generations<ValueEntry>,
    lifts: Generations<LiftEntry>,
    /// Entries per generation of the crossing cache (a test lowers it to exercise the drop); the value cache holds four times as many, the lift cache as many.
    pub limit: usize,
    /// Crossings found, crossings stored, values found, lifts found (what the session reports).
    pub hits: u64,
    pub stored: u64,
    pub value_hits: u64,
    pub lift_hits: u64,
    pub enabled: bool,
}

impl Warm {
    pub fn new() -> Warm {
        Warm { enabled: true, limit: CROSSING_LIMIT, ..Warm::default() }
    }

    pub fn disabled() -> Warm {
        Warm::default()
    }

    /// Entries held (both generations of the crossing cache).
    pub fn len(&self) -> usize {
        self.crossings.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// `(crossing, value, lift)` entries held.
    pub fn sizes(&self) -> (usize, usize, usize) {
        (self.crossings.len(), self.values.len(), self.lifts.len())
    }

    pub fn clear(&mut self) {
        self.crossings.clear();
        self.values.clear();
        self.lifts.clear();
    }

    pub fn find(&mut self, key: u64, first: &Point, second: &Point, line: &EdgeLine) -> Option<CrossingEntry> {
        let limit = self.limit;
        self.crossings.find(key, limit, |entry| *entry.first == *first && *entry.second == *second && entry.line == *line)
    }

    pub fn store(&mut self, key: u64, entry: CrossingEntry) {
        let limit = self.limit;
        self.crossings.store(key, entry, limit);
        self.stored += 1;
    }

    pub fn find_value(&mut self, key: u64, point: &Point, line: &EdgeLine) -> Option<Arc<SqrtSum>> {
        let limit = self.limit.saturating_mul(4);
        let found = self.values.find(key, limit, |entry| *entry.point == *point && entry.line == *line).map(|entry| entry.value);
        self.value_hits += u64::from(found.is_some());
        found
    }

    pub fn store_value(&mut self, key: u64, entry: ValueEntry) {
        let limit = self.limit.saturating_mul(4);
        self.values.store(key, entry, limit);
    }

    pub fn find_lift(&mut self, key: u64, point: &Point, triangle: &Triangle) -> Option<Lifted> {
        let limit = self.limit;
        let found = self.lifts.find(key, limit, |entry| *entry.point == *point && *entry.triangle == *triangle).map(|entry| entry.lifted);
        self.lift_hits += u64::from(found.is_some());
        found
    }

    pub fn store_lift(&mut self, key: u64, entry: LiftEntry) {
        let limit = self.limit;
        self.lifts.store(key, entry, limit);
    }
}

/// The key of a crossing: the hashes of the two end points and of the line.
pub fn crossing_key(first_hash: u64, second_hash: u64, line_hash: u64) -> u64 {
    const SEED: u64 = 0x517c_c1b7_2722_0a95;
    let mix = |state: u64, word: u64| (state.rotate_left(5) ^ word).wrapping_mul(SEED);
    mix(mix(mix(0, first_hash), second_hash), line_hash)
}

/// The key of a value: the hashes of the point and of the line.
pub fn value_key(point_hash: u64, line_hash: u64) -> u64 {
    crossing_key(point_hash, line_hash, 0x76616c75)
}

/// The key of a lift: the hashes of the point and of the triangle.
pub fn lift_key(point_hash: u64, triangle_hash: u64) -> u64 {
    crossing_key(point_hash, triangle_hash, 0x6c696674)
}
