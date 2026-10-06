//! The process canonicalization memory and every function that reads or writes it
//! (`exact_sqrt_sum.py` lines 636-1135): the proven-prime registry, the factorization / squarefree-split /
//! prime-support tables, and the Python-exact call graph above `rho_factors`.
//!
//! What "equal" means here is stronger than answers: the CONTENT AND INSERTION ORDER of the four tables, the LRU
//! touch of the factorization table, eviction at 8192 entries checked at insert, registry clear-before-insert at
//! 8192 and `insort` order. Every mutation therefore happens at the same point of the same call chain as in
//! Python, so a budget exhaustion in the middle leaves the same partial state.
//!
//! The budget is always passed explicitly. Python's `budget=None` path is `UNBUDGETED_WORK`, an unlimited budget
//! whose deltas the host adds to its own `UNBUDGETED_WORK`: pass a `WorkBudget::unlimited()` for it.
//!
//! Not part of this crate (and cleared by Python's `reset_factorization_memory` in the same call): the float
//! centre table of `float_filter` and the radicand-product cache. The integration layer clears those itself.

use std::collections::{BTreeSet, HashSet};

use dashu_int::{IBig, UBig};

use crate::budget::{Operation, WorkBudget};
use crate::factor::{add_factor, coprime_basis, rho_factors, CanonError, Pairs};
use crate::ordered::OrderedMap;

/// `_FACTORIZATION_MEMO_ENTRIES`.
pub const FACTORIZATION_MEMO_ENTRIES: usize = 1 << 13;
/// `_KNOWN_PRIME_REGISTRY_ENTRIES`.
pub const KNOWN_PRIME_REGISTRY_ENTRIES: usize = 1 << 13;

/// `_SQUAREFREE_MEMO` value: `(outside, inside)` with `n = outside^2 * inside`.
pub type Split = (UBig, UBig);
/// `_PRIME_SUPPORT_MEMO` value: the primes of a squarefree radicand, ascending.
pub type Support = Vec<UBig>;

/// One mutation of the memory, in the order Python performs it. A host that mirrors the tables as real Python
/// containers replays these in place (`d[key] = v`, `d[key] = d.pop(key)`, `del d[key]`, `insort`, `clear`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MemOp {
    /// `reset_factorization_memory`: all four tables cleared.
    ResetAll,
    /// `_register_prime` found the registry full: list and set cleared before the insert.
    RegistryClear,
    /// `insort(_KNOWN_PRIMES, prime); _KNOWN_PRIME_SET.add(prime)`.
    RegistryInsert(UBig),
    /// `del _FACTORIZATION_MEMO[next(iter(_FACTORIZATION_MEMO))]`; `key` is that oldest key.
    FactorizationEvictOldest { key: UBig },
    /// `_FACTORIZATION_MEMO[key] = pairs` (position: end, or in place when the key was present).
    FactorizationInsert { key: UBig, pairs: Pairs },
    /// LRU hit: `value = d.pop(key); d[key] = value`.
    FactorizationTouch { key: UBig },
    /// `_SQUAREFREE_MEMO[key] = value` (new key).
    SquarefreeInsert { key: UBig, value: Split },
    /// `_PRIME_SUPPORT_MEMO[key] = value` (new key).
    SupportInsert { key: UBig, value: Support },
}

/// Full ordered content of the memory: the interchange form for loading a before-state / exporting an after-state.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MemoryState {
    pub known_primes: Vec<UBig>,
    pub factorization: Vec<(UBig, Pairs)>,
    pub squarefree: Vec<(UBig, Split)>,
    pub support: Vec<(UBig, Support)>,
}

/// An incremental update of one insertion-ordered table: optionally clear it, delete the listed keys wherever they
/// are, then append `tail` in order. The host derives it from the real Python dict and the key order it last
/// mirrored: kept entries keep their relative order, so "oldest evicted", "touched to the end" and "appended" all
/// reduce to `deleted` plus `tail`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TableSync<V> {
    pub clear: bool,
    pub deleted: Vec<UBig>,
    pub tail: Vec<(UBig, V)>,
}

/// The incremental update of the whole memory (`CanonMemory::apply_sync`); the registry is a sorted set, so it
/// travels as removed and added primes.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MemorySync {
    pub registry_clear: bool,
    pub registry_removed: Vec<UBig>,
    pub registry_added: Vec<UBig>,
    pub factorization: TableSync<Pairs>,
    pub squarefree: TableSync<Split>,
    pub support: TableSync<Support>,
}

/// `FactorizationMemoryDeltaV1`: what a stage added, in insertion order (primes: registry, i.e. ascending, order).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MemoryDelta {
    pub factorizations: Vec<(UBig, Pairs)>,
    pub squarefree: Vec<(UBig, Split)>,
    pub supports: Vec<(UBig, Support)>,
    pub primes: Vec<UBig>,
}

/// `factorization_memory_marker`: the key sets present NOW.
#[derive(Clone, Debug, Default)]
pub struct MemoryMarker {
    factorizations: HashSet<UBig>,
    squarefree: HashSet<UBig>,
    supports: HashSet<UBig>,
    primes: HashSet<UBig>,
}

/// A `Fraction` `numerator / denominator` in lowest terms with a positive denominator (as Python normalizes it);
/// ints pass `denominator = 1`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QValue {
    pub numerator: IBig,
    pub denominator: UBig,
}

impl QValue {
    pub fn from_int(value: IBig) -> QValue {
        QValue { numerator: value, denominator: UBig::ONE }
    }
}

/// The `(universe, delta)` pair `prime_universe_remembered` writes into its `store`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UniverseRecord {
    pub universe: Vec<UBig>,
    pub delta: Vec<(UBig, Pairs)>,
}

/// One question asked of the memory (the cost-bearing ones): `prime_support(radicand)` or `squarefree_split(n)`. A computation that
/// asks the same questions in the same order pays the same budget and leaves the same tables and the same LRU order, whatever it does
/// with the answers: recording the questions of a computation and asking them again IS its cost.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Request {
    Support(UBig),
    Squarefree(UBig),
}

/// The four tables and the registry.
#[derive(Clone, Debug, Default)]
pub struct CanonMemory {
    requests: Option<Vec<Request>>,
    known_primes: Vec<UBig>,
    known_prime_set: HashSet<UBig>,
    factorization: OrderedMap<UBig, Pairs>,
    squarefree: OrderedMap<UBig, Split>,
    support: OrderedMap<UBig, Support>,
    log: Option<Vec<MemOp>>,
}

/// The magnitude of a non-negative integer; `None` for a negative one.
fn unsigned_of(value: &IBig) -> Option<UBig> {
    UBig::try_from(value.clone()).ok()
}

fn strictly_ascending(values: &[UBig]) -> bool {
    values.windows(2).all(|pair| pair[0] < pair[1])
}

fn has_duplicates<'a>(keys: impl Iterator<Item = &'a UBig>) -> bool {
    let mut seen = HashSet::new();
    keys.into_iter().any(|key| !seen.insert(key))
}

fn sync_table<V>(table: &mut OrderedMap<UBig, V>, sync: TableSync<V>) -> Result<(), CanonError> {
    if sync.clear {
        table.clear();
    }
    for key in &sync.deleted {
        if table.remove(key).is_none() {
            return Err(CanonError::InvalidInput("sync deletes a key the mirror does not hold"));
        }
    }
    for (key, value) in sync.tail {
        if !table.insert_if_absent(key, value) {
            return Err(CanonError::InvalidInput("sync appends a key the mirror already holds"));
        }
    }
    Ok(())
}

impl CanonMemory {
    pub fn new() -> CanonMemory {
        CanonMemory::default()
    }

    // ---- observation, interchange and the op-log ---------------------------------------------------------------

    /// Table lengths: registry, factorizations, squarefree splits, supports.
    pub fn lengths(&self) -> (usize, usize, usize, usize) {
        (self.known_primes.len(), self.factorization.len(), self.squarefree.len(), self.support.len())
    }

    /// The whole ordered content.
    pub fn export_state(&self) -> MemoryState {
        MemoryState {
            known_primes: self.known_primes.clone(),
            factorization: self.factorization.iter().map(|(key, pairs)| (key.clone(), pairs.clone())).collect(),
            squarefree: self.squarefree.iter().map(|(key, split)| (key.clone(), split.clone())).collect(),
            support: self.support.iter().map(|(key, support)| (key.clone(), support.clone())).collect(),
        }
    }

    /// Replace the content by a full ordered state (the registry must be strictly ascending, keys unique). An
    /// active op-log is emptied: the load is a baseline, not an operation.
    pub fn load_state(&mut self, state: MemoryState) -> Result<(), CanonError> {
        if !strictly_ascending(&state.known_primes) {
            return Err(CanonError::InvalidInput("registry must be strictly ascending"));
        }
        if has_duplicates(state.factorization.iter().map(|(key, _)| key))
            || has_duplicates(state.squarefree.iter().map(|(key, _)| key))
            || has_duplicates(state.support.iter().map(|(key, _)| key))
        {
            return Err(CanonError::InvalidInput("duplicate memory key"));
        }
        self.known_prime_set = state.known_primes.iter().cloned().collect();
        self.known_primes = state.known_primes;
        self.factorization = OrderedMap::new();
        for (key, pairs) in state.factorization {
            self.factorization.set(key, pairs);
        }
        self.squarefree = OrderedMap::new();
        for (key, split) in state.squarefree {
            self.squarefree.set(key, split);
        }
        self.support = OrderedMap::new();
        for (key, support) in state.support {
            self.support.set(key, support);
        }
        if let Some(log) = &mut self.log {
            log.clear();
        }
        Ok(())
    }

    /// Bring the memory to the host's current content by an incremental update (see [`MemorySync`]). A host that
    /// mirrors correctly never trips the errors; one that does not gets a named refusal, never a silent repair.
    /// An active op-log is emptied: the sync is a baseline, not an operation.
    pub fn apply_sync(&mut self, sync: MemorySync) -> Result<(), CanonError> {
        if sync.registry_clear {
            self.known_primes.clear();
            self.known_prime_set.clear();
        }
        for prime in &sync.registry_removed {
            if !self.known_prime_set.remove(prime) {
                return Err(CanonError::InvalidInput("sync removes a prime the mirror does not hold"));
            }
        }
        if !sync.registry_removed.is_empty() {
            let removed: HashSet<&UBig> = sync.registry_removed.iter().collect();
            self.known_primes.retain(|prime| !removed.contains(prime));
        }
        for prime in sync.registry_added {
            if !self.known_prime_set.insert(prime.clone()) {
                return Err(CanonError::InvalidInput("sync adds a prime the mirror already holds"));
            }
            let position = self.known_primes.partition_point(|known| *known < prime);
            self.known_primes.insert(position, prime);
        }
        sync_table(&mut self.factorization, sync.factorization)?;
        sync_table(&mut self.squarefree, sync.squarefree)?;
        sync_table(&mut self.support, sync.support)?;
        if let Some(log) = &mut self.log {
            log.clear();
        }
        Ok(())
    }

    /// Start (or restart) recording mutations.
    pub fn start_log(&mut self) {
        self.log = Some(Vec::new());
    }

    /// Start recording the questions asked of the memory ([`Request`]), dropping any earlier recording.
    pub fn start_requests(&mut self) {
        self.requests = Some(Vec::new());
    }

    /// Stop recording and hand over the questions asked since [`CanonMemory::start_requests`].
    pub fn take_requests(&mut self) -> Vec<Request> {
        self.requests.take().unwrap_or_default()
    }

    /// Asks a recorded question again: the same budget payment, memory write or LRU touch as the first time, the answer dropped.
    pub fn replay_request(&mut self, request: &Request, budget: &mut WorkBudget) -> Result<(), CanonError> {
        match request {
            Request::Support(radicand) => self.prime_support_unsigned(radicand, budget).map(drop),
            Request::Squarefree(n) => self.squarefree_split_unsigned(n, budget).map(drop),
        }
    }

    /// Take the mutations recorded since the last `start_log` / `take_log`; recording continues.
    pub fn take_log(&mut self) -> Vec<MemOp> {
        match &mut self.log {
            Some(log) => std::mem::take(log),
            None => Vec::new(),
        }
    }

    pub fn stop_log(&mut self) {
        self.log = None;
    }

    fn record(&mut self, make: impl FnOnce() -> MemOp) {
        if let Some(log) = &mut self.log {
            log.push(make());
        }
    }

    // ---- reset / isolation / marker / delta --------------------------------------------------------------------

    /// `reset_factorization_memory` (the four tables; see the module note for the two host-side caches).
    pub fn reset(&mut self) {
        self.known_primes.clear();
        self.known_prime_set.clear();
        self.factorization.clear();
        self.squarefree.clear();
        self.support.clear();
        self.record(|| MemOp::ResetAll);
    }

    /// `isolated_factorization_memory`: run `body` on a COLD memory, then put the caller's memory back verbatim
    /// (order included). The block is invisible to a mirrored host: its mutations are not logged.
    pub fn with_isolated<R>(&mut self, body: impl FnOnce(&mut CanonMemory) -> R) -> R {
        let saved = std::mem::take(self);
        let result = body(self);
        *self = saved;
        result
    }

    /// `factorization_memory_marker`.
    pub fn marker(&self) -> MemoryMarker {
        MemoryMarker {
            factorizations: self.factorization.keys().cloned().collect(),
            squarefree: self.squarefree.keys().cloned().collect(),
            supports: self.support.keys().cloned().collect(),
            primes: self.known_prime_set.clone(),
        }
    }

    /// `factorization_memory_delta(marker)`.
    pub fn delta_since(&self, marker: &MemoryMarker) -> MemoryDelta {
        MemoryDelta {
            factorizations: self
                .factorization
                .iter()
                .filter(|(key, _)| !marker.factorizations.contains(*key))
                .map(|(key, pairs)| (key.clone(), pairs.clone()))
                .collect(),
            squarefree: self
                .squarefree
                .iter()
                .filter(|(key, _)| !marker.squarefree.contains(*key))
                .map(|(key, split)| (key.clone(), split.clone()))
                .collect(),
            supports: self
                .support
                .iter()
                .filter(|(key, _)| !marker.supports.contains(*key))
                .map(|(key, support)| (key.clone(), support.clone()))
                .collect(),
            primes: self.known_primes.iter().filter(|prime| !marker.primes.contains(*prime)).cloned().collect(),
        }
    }

    /// `replay_factorization_memory(delta)`: entries already present are not touched; limits as in a real count.
    pub fn replay_delta(&mut self, delta: &MemoryDelta) {
        for (key, pairs) in &delta.factorizations {
            if !self.factorization.contains_key(key) {
                self.insert_factorization(key, pairs);
            }
        }
        for (key, split) in &delta.squarefree {
            self.insert_squarefree_if_absent(key, split);
        }
        for (key, support) in &delta.supports {
            self.insert_support_if_absent(key, support);
        }
        for prime in &delta.primes {
            self.register_prime(prime);
        }
    }

    // ---- primitive mutations (each mirrors one Python statement) -----------------------------------------------

    /// `if len(memo) >= LIMIT: del memo[next(iter(memo))]; memo[key] = pairs`.
    fn insert_factorization(&mut self, key: &UBig, pairs: &Pairs) {
        if self.factorization.len() >= FACTORIZATION_MEMO_ENTRIES {
            if let Some((oldest, _)) = self.factorization.pop_front() {
                self.record(|| MemOp::FactorizationEvictOldest { key: oldest });
            }
        }
        self.factorization.set(key.clone(), pairs.clone());
        self.record(|| MemOp::FactorizationInsert { key: key.clone(), pairs: pairs.clone() });
    }

    fn insert_squarefree_if_absent(&mut self, key: &UBig, split: &Split) {
        if self.squarefree.insert_if_absent(key.clone(), split.clone()) {
            self.record(|| MemOp::SquarefreeInsert { key: key.clone(), value: split.clone() });
        }
    }

    fn insert_support_if_absent(&mut self, key: &UBig, support: &Support) {
        if self.support.insert_if_absent(key.clone(), support.clone()) {
            self.record(|| MemOp::SupportInsert { key: key.clone(), value: support.clone() });
        }
    }

    /// `_register_prime`: ascending registry without repeats; a full registry is cleared BEFORE the insert.
    pub fn register_prime(&mut self, prime: &UBig) {
        if self.known_prime_set.contains(prime) {
            return;
        }
        if self.known_primes.len() >= KNOWN_PRIME_REGISTRY_ENTRIES {
            self.known_primes.clear();
            self.known_prime_set.clear();
            self.record(|| MemOp::RegistryClear);
        }
        let position = self.known_primes.partition_point(|known| known <= prime);
        self.known_primes.insert(position, prime.clone());
        self.known_prime_set.insert(prime.clone());
        self.record(|| MemOp::RegistryInsert(prime.clone()));
    }

    // ---- factorization -----------------------------------------------------------------------------------------

    /// `_strip_known_primes`: divide the registry primes out of `n` (ascending, stopping at the first prime
    /// above the remainder); returns the stripped pairs and the cofactor. No proof, no mutation.
    pub fn strip_known_primes(&self, n: &UBig) -> (Pairs, UBig) {
        let mut factors: Pairs = Vec::new();
        let mut remainder = n.clone();
        for prime in &self.known_primes {
            if *prime > remainder {
                break;
            }
            if &remainder % prime != UBig::ZERO {
                continue;
            }
            let mut power = 0u64;
            while &remainder % prime == UBig::ZERO {
                remainder /= prime;
                power += 1;
            }
            factors.push((prime.clone(), power));
        }
        (factors, remainder)
    }

    /// `_factorization_pairs`: ascending `(prime, power)` pairs; a hit is an LRU touch, a miss strips the
    /// registry, factors the cofactor, registers primes in dict insertion order and then stores the entry.
    pub fn factorization_pairs(&mut self, n: &UBig, budget: &mut WorkBudget) -> Result<Pairs, CanonError> {
        if *n < UBig::from(2u8) {
            return Ok(Vec::new());
        }
        if let Some(cached) = self.factorization.get(n).cloned() {
            self.factorization.move_to_end(n);
            self.record(|| MemOp::FactorizationTouch { key: n.clone() });
            return Ok(cached);
        }
        let (mut factors, remainder) = self.strip_known_primes(n);
        if remainder > UBig::ONE {
            for (prime, power) in rho_factors(&remainder, budget)? {
                add_factor(&mut factors, &prime, power);
            }
        }
        for (prime, _) in &factors {
            self.register_prime(prime);
        }
        factors.sort();
        self.insert_factorization(n, &factors);
        Ok(factors)
    }

    /// `_coprime_basis` + factor each atom: `_seed_factorization_basis(radicands)`.
    pub fn seed_factorization_basis(&mut self, radicands: &[UBig], budget: &mut WorkBudget) -> Result<(), CanonError> {
        let mut residues: Vec<UBig> = Vec::new();
        for radicand in radicands {
            let (_, remainder) = self.strip_known_primes(radicand);
            if remainder > UBig::ONE {
                residues.push(remainder);
            }
        }
        if residues.len() < 2 {
            return Ok(());
        }
        for atom in coprime_basis(&residues, budget)? {
            self.factorization_pairs(&atom, budget)?;
        }
        Ok(())
    }

    // ---- squarefree split / prime support ----------------------------------------------------------------------

    /// `squarefree_split(n)`; a negative `n` is `NegativeRadicandError`.
    pub fn squarefree_split(&mut self, n: &IBig, budget: &mut WorkBudget) -> Result<Split, CanonError> {
        match unsigned_of(n) {
            Some(unsigned) => self.squarefree_split_unsigned(&unsigned, budget),
            None => Err(CanonError::NegativeRadicand { numerator: n.clone(), denominator: UBig::ONE }),
        }
    }

    /// `squarefree_split(n)` for `n >= 0`. A miss pays one radical materialization BEFORE factoring.
    pub fn squarefree_split_unsigned(&mut self, n: &UBig, budget: &mut WorkBudget) -> Result<Split, CanonError> {
        if *n == UBig::ZERO {
            return Ok((UBig::ZERO, UBig::ZERO));
        }
        if *n == UBig::ONE {
            return Ok((UBig::ONE, UBig::ONE));
        }
        if let Some(requests) = &mut self.requests {
            requests.push(Request::Squarefree(n.clone()));
        }
        if let Some(cached) = self.squarefree.get(n) {
            return Ok(cached.clone());
        }
        budget.spend_radical_materializations(1, Operation::SquarefreeSplit, n)?;
        let mut outside = UBig::ONE;
        let mut inside = UBig::ONE;
        for (prime, power) in self.factorization_pairs(n, budget)? {
            outside *= prime.pow((power / 2) as usize);
            if power % 2 == 1 {
                inside *= prime;
            }
        }
        let result = (outside, inside);
        self.squarefree.set(n.clone(), result.clone());
        self.record(|| MemOp::SquarefreeInsert { key: n.clone(), value: result.clone() });
        Ok(result)
    }

    /// `prime_support(radicand)`; `radicand <= 1` has no support.
    pub fn prime_support(&mut self, radicand: &IBig, budget: &mut WorkBudget) -> Result<Support, CanonError> {
        match unsigned_of(radicand) {
            Some(unsigned) => self.prime_support_unsigned(&unsigned, budget),
            None => Ok(Vec::new()),
        }
    }

    /// `prime_support(radicand)` for `radicand >= 0`. A miss pays one radical materialization BEFORE factoring.
    pub fn prime_support_unsigned(&mut self, radicand: &UBig, budget: &mut WorkBudget) -> Result<Support, CanonError> {
        if *radicand <= UBig::ONE {
            return Ok(Vec::new());
        }
        if let Some(requests) = &mut self.requests {
            requests.push(Request::Support(radicand.clone()));
        }
        if let Some(cached) = self.support.get(radicand) {
            return Ok(cached.clone());
        }
        budget.spend_radical_materializations(1, Operation::PrimeSupport, radicand)?;
        let support: Support = self.factorization_pairs(radicand, budget)?.into_iter().map(|(prime, _)| prime).collect();
        self.support.set(radicand.clone(), support.clone());
        self.record(|| MemOp::SupportInsert { key: radicand.clone(), value: support.clone() });
        Ok(support)
    }

    // ---- prime universe ----------------------------------------------------------------------------------------

    /// `_prime_universe_from_q_values`: primes of odd power of every primitive `q` (radicand `p*r` for `q = p/r`).
    /// The coprime basis is built over the SORTED radicand set, then each radicand is factored once, ascending.
    pub fn prime_universe_from_q_values(&mut self, q_values: &[QValue], budget: &mut WorkBudget) -> Result<Vec<UBig>, CanonError> {
        let mut transformed: BTreeSet<UBig> = BTreeSet::new();
        for q in q_values {
            let Some(numerator) = unsigned_of(&q.numerator) else {
                return Err(CanonError::NegativeRadicand { numerator: q.numerator.clone(), denominator: q.denominator.clone() });
            };
            if numerator == UBig::ZERO {
                continue;
            }
            transformed.insert(numerator * &q.denominator);
        }
        let radicands: Vec<UBig> = transformed.into_iter().collect();
        self.seed_factorization_basis(&radicands, budget)?;
        let mut primes: BTreeSet<UBig> = BTreeSet::new();
        for radicand in &radicands {
            let factors = self.factorization_pairs(radicand, budget)?;
            let mut reconstructed = UBig::ONE;
            for (prime, power) in &factors {
                reconstructed *= prime.pow(*power as usize);
                if power % 2 == 1 {
                    primes.insert(prime.clone());
                }
            }
            if reconstructed != *radicand {
                return Err(CanonError::ReconstructionFailed { radicand: radicand.clone() });
            }
        }
        Ok(primes.into_iter().collect())
    }

    /// `prime_universe_remembered`, store MISS: build, then record the universe and the factorizations the call
    /// added or found (`numbers`: keys new since the call began plus every `q` radicand that is in the table),
    /// plus `(p, ((p, 1),))` for each universe prime. The caller writes the record into its `store` only on `Ok`.
    pub fn prime_universe_miss(&mut self, q_values: &[QValue], budget: &mut WorkBudget) -> Result<UniverseRecord, CanonError> {
        let before: HashSet<UBig> = self.factorization.keys().cloned().collect();
        let universe = self.prime_universe_from_q_values(q_values, budget)?;
        let mut numbers: BTreeSet<UBig> =
            self.factorization.keys().filter(|key| !before.contains(*key)).cloned().collect();
        for q in q_values {
            if let Some(numerator) = unsigned_of(&q.numerator) {
                numbers.insert(numerator * &q.denominator);
            }
        }
        let mut delta: Vec<(UBig, Pairs)> = Vec::new();
        for number in numbers {
            if let Some(pairs) = self.factorization.get(&number) {
                delta.push((number, pairs.clone()));
            }
        }
        for prime in &universe {
            delta.push((prime.clone(), vec![(prime.clone(), 1)]));
        }
        Ok(UniverseRecord { universe, delta })
    }

    /// `prime_universe_remembered`, store HIT: put the recorded factorizations back as if just computed (no
    /// budget spent), registering every prime of every recorded pair list.
    pub fn prime_universe_hit(&mut self, record: &UniverseRecord) -> Vec<UBig> {
        for (number, pairs) in &record.delta {
            if !self.factorization.contains_key(number) {
                self.insert_factorization(number, pairs);
            }
            for (prime, _) in pairs {
                self.register_prime(prime);
            }
        }
        record.universe.clone()
    }
}

// ---- pure helpers over a universe ----------------------------------------------------------------------------

/// `_support_from_prime_universe`: the exact support of a squarefree radicand, or `None` if the universe does
/// not reproduce it (remainder not 1, product not the radicand, or a repeated prime).
pub fn support_from_prime_universe(radicand: &UBig, prime_universe: &[UBig]) -> Option<Vec<UBig>> {
    if *radicand <= UBig::ONE {
        return if *radicand == UBig::ONE { Some(Vec::new()) } else { None };
    }
    let mut remainder = radicand.clone();
    let mut product = UBig::ONE;
    let mut support: Vec<UBig> = Vec::new();
    for prime in prime_universe {
        if &remainder % prime != UBig::ZERO {
            continue;
        }
        remainder /= prime;
        product *= prime;
        support.push(prime.clone());
        if &remainder % prime == UBig::ZERO {
            return None;
        }
        if remainder == UBig::ONE {
            break;
        }
    }
    if remainder != UBig::ONE || product != *radicand {
        return None;
    }
    Some(support)
}

/// `_pick_prime_from_universe`: the minimal support prime if every nonzero, non-unit radicand is reproduced.
/// `terms` yields `(radicand, coefficient_is_nonzero)` in the dict's order.
pub fn pick_prime_from_universe<'a>(
    terms: impl IntoIterator<Item = (&'a UBig, bool)>,
    prime_universe: &[UBig],
) -> Option<UBig> {
    let mut smallest: Option<UBig> = None;
    for (radicand, nonzero) in terms {
        if !nonzero || *radicand == UBig::ONE {
            continue;
        }
        let support = support_from_prime_universe(radicand, prime_universe)?;
        let first = support.first()?;
        if smallest.as_ref().is_none_or(|current| first < current) {
            smallest = Some(first.clone());
        }
    }
    smallest
}
