//! The incremental sync a host sends before a call: `deleted` plus an appended `tail` per table, removed and
//! added primes for the registry. The mirror must end in exactly the content (and order) the host has, and a
//! disagreement is a named refusal, never a repair.

use cftuv_canon::{CanonError, CanonMemory, MemOp, MemorySync, MemoryState, TableSync, WorkBudget};
use dashu_int::UBig;

fn u(value: u64) -> UBig {
    UBig::from(value)
}

fn pairs(items: &[(u64, u64)]) -> Vec<(UBig, u64)> {
    items.iter().map(|(prime, power)| (u(*prime), *power)).collect()
}

fn keys(state: &MemoryState) -> Vec<u64> {
    state.factorization.iter().map(|(key, _)| u64::try_from(key).unwrap()).collect()
}

fn append(items: &[u64]) -> TableSync<Vec<(UBig, u64)>> {
    TableSync { clear: false, deleted: vec![], tail: items.iter().map(|key| (u(*key), pairs(&[(*key, 1)]))).collect() }
}

#[test]
fn a_tail_appends_in_order_and_a_delete_keeps_the_relative_order_of_the_rest() {
    let mut memory = CanonMemory::new();
    memory.apply_sync(MemorySync { factorization: append(&[7, 11, 13, 17]), ..Default::default() }).unwrap();
    assert_eq!(keys(&memory.export_state()), vec![7, 11, 13, 17]);
    // the host touched 11 (moved to the end), evicted 7 and appended 19: kept 13, 17 in order, tail = 11, 19
    let sync = MemorySync {
        factorization: TableSync {
            clear: false,
            deleted: vec![u(7), u(11)],
            tail: vec![(u(11), pairs(&[(11, 1)])), (u(19), pairs(&[(19, 1)]))],
        },
        ..Default::default()
    };
    memory.apply_sync(sync).unwrap();
    assert_eq!(keys(&memory.export_state()), vec![13, 17, 11, 19]);
    // a clear followed by a tail replaces everything
    let sync = MemorySync { factorization: TableSync { clear: true, deleted: vec![], tail: append(&[2, 3]).tail }, ..Default::default() };
    memory.apply_sync(sync).unwrap();
    assert_eq!(keys(&memory.export_state()), vec![2, 3]);
}

#[test]
fn the_registry_is_a_sorted_set_updated_by_removed_and_added_primes() {
    let mut memory = CanonMemory::new();
    memory.apply_sync(MemorySync { registry_added: vec![u(13), u(2), u(7)], ..Default::default() }).unwrap();
    assert_eq!(memory.export_state().known_primes, vec![u(2), u(7), u(13)]);
    memory.apply_sync(MemorySync { registry_removed: vec![u(7)], registry_added: vec![u(5), u(17)], ..Default::default() }).unwrap();
    assert_eq!(memory.export_state().known_primes, vec![u(2), u(5), u(13), u(17)]);
    memory.apply_sync(MemorySync { registry_clear: true, registry_added: vec![u(3)], ..Default::default() }).unwrap();
    assert_eq!(memory.export_state().known_primes, vec![u(3)]);
}

#[test]
fn a_sync_the_mirror_cannot_apply_is_refused_by_name() {
    let mut memory = CanonMemory::new();
    memory.apply_sync(MemorySync { factorization: append(&[7]), registry_added: vec![u(7)], ..Default::default() }).unwrap();
    let missing_delete = MemorySync { factorization: TableSync { clear: false, deleted: vec![u(99)], tail: vec![] }, ..Default::default() };
    assert!(matches!(memory.apply_sync(missing_delete), Err(CanonError::InvalidInput(_))));
    let repeated_key = MemorySync { factorization: append(&[7]), ..Default::default() };
    assert!(matches!(memory.apply_sync(repeated_key), Err(CanonError::InvalidInput(_))));
    let repeated_prime = MemorySync { registry_added: vec![u(7)], ..Default::default() };
    assert!(matches!(memory.apply_sync(repeated_prime), Err(CanonError::InvalidInput(_))));
    let missing_prime = MemorySync { registry_removed: vec![u(11)], ..Default::default() };
    assert!(matches!(memory.apply_sync(missing_prime), Err(CanonError::InvalidInput(_))));
}

#[test]
fn a_synced_mirror_computes_like_a_mirror_that_built_the_same_content_itself() {
    // build real content by computing, export it, load it into a second memory THROUGH a sync, then continue both
    let mut built = CanonMemory::new();
    let mut budget = WorkBudget::unlimited();
    for n in [360u64, 1_000_003 * 7, 49 * 121, 600_851_475_143] {
        built.squarefree_split_unsigned(&u(n), &mut budget).unwrap();
        built.prime_support_unsigned(&u(n), &mut budget).unwrap();
    }
    let state = built.export_state();
    let mut synced = CanonMemory::new();
    synced
        .apply_sync(MemorySync {
            registry_added: state.known_primes.clone(),
            factorization: TableSync { clear: false, deleted: vec![], tail: state.factorization.clone() },
            squarefree: TableSync { clear: false, deleted: vec![], tail: state.squarefree.clone() },
            support: TableSync { clear: false, deleted: vec![], tail: state.support.clone() },
            ..Default::default()
        })
        .unwrap();
    assert_eq!(synced.export_state(), state);
    // the next calls cost and log the same on both
    built.start_log();
    synced.start_log();
    let (mut first, mut second) = (WorkBudget::unlimited(), WorkBudget::unlimited());
    for n in [360u64, 2 * 3 * 5 * 999_983, 600_851_475_143, 7 * 7 * 13] {
        assert_eq!(built.squarefree_split_unsigned(&u(n), &mut first), synced.squarefree_split_unsigned(&u(n), &mut second));
    }
    assert_eq!(first.articles(), second.articles());
    let (left, right): (Vec<MemOp>, Vec<MemOp>) = (built.take_log(), synced.take_log());
    assert_eq!(left, right);
    assert_eq!(built.export_state(), synced.export_state());
}
