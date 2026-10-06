//! Insertion-ordered map with Python-dict semantics: assignment to an existing key keeps its position,
//! `del` + re-insert moves a key to the end, `next(iter(d))` is the oldest key. Backs the canonicalization tables,
//! whose ORDER is observable (LRU eviction) and therefore part of the equivalence contract.

use std::collections::HashMap;
use std::hash::Hash;

/// Slots in insertion order with tombstones; compacted when more than half of the slots are dead.
#[derive(Clone, Debug)]
pub struct OrderedMap<K, V> {
    slots: Vec<Option<(K, V)>>,
    index: HashMap<K, usize>,
    head: usize,
}

impl<K: Hash + Eq + Clone, V> Default for OrderedMap<K, V> {
    fn default() -> Self {
        OrderedMap::new()
    }
}

impl<K: Hash + Eq + Clone, V> OrderedMap<K, V> {
    pub fn new() -> OrderedMap<K, V> {
        OrderedMap { slots: Vec::new(), index: HashMap::new(), head: 0 }
    }

    pub fn len(&self) -> usize {
        self.index.len()
    }

    pub fn contains_key(&self, key: &K) -> bool {
        self.index.contains_key(key)
    }

    pub fn get(&self, key: &K) -> Option<&V> {
        let slot = *self.index.get(key)?;
        self.slots[slot].as_ref().map(|(_, value)| value)
    }

    pub fn clear(&mut self) {
        self.slots.clear();
        self.index.clear();
        self.head = 0;
    }

    /// `d[key] = value`: an existing key keeps its place, a new key goes to the end.
    pub fn set(&mut self, key: K, value: V) {
        if let Some(slot) = self.index.get(&key) {
            self.slots[*slot] = Some((key, value));
            return;
        }
        self.push_back(key, value);
    }

    /// `d.setdefault(key, value)`: insert at the end only if absent. Returns whether it inserted.
    pub fn insert_if_absent(&mut self, key: K, value: V) -> bool {
        if self.index.contains_key(&key) {
            return false;
        }
        self.push_back(key, value);
        true
    }

    /// `del d[key]; d[key] = value` with the value kept: the LRU touch. `false` if the key is absent.
    pub fn move_to_end(&mut self, key: &K) -> bool {
        let Some(slot) = self.index.remove(key) else {
            return false;
        };
        if let Some(entry) = self.slots[slot].take() {
            self.push_back(entry.0, entry.1);
        }
        true
    }

    /// `del d[key]`: drop an entry wherever it is; `None` if the key is absent.
    pub fn remove(&mut self, key: &K) -> Option<V> {
        let slot = self.index.remove(key)?;
        self.slots[slot].take().map(|(_, value)| value)
    }

    /// `del d[next(iter(d))]`: drop and return the oldest entry.
    pub fn pop_front(&mut self) -> Option<(K, V)> {
        while self.head < self.slots.len() {
            let taken = self.slots[self.head].take();
            self.head += 1;
            if let Some((key, value)) = taken {
                self.index.remove(&key);
                return Some((key, value));
            }
        }
        None
    }

    /// Entries from the oldest to the newest.
    pub fn iter(&self) -> impl Iterator<Item = (&K, &V)> {
        self.slots[self.head..].iter().filter_map(|slot| slot.as_ref().map(|(key, value)| (key, value)))
    }

    pub fn keys(&self) -> impl Iterator<Item = &K> {
        self.iter().map(|(key, _)| key)
    }

    fn push_back(&mut self, key: K, value: V) {
        if self.slots.len() > 2 * self.index.len() + 64 {
            self.compact();
        }
        self.index.insert(key.clone(), self.slots.len());
        self.slots.push(Some((key, value)));
    }

    fn compact(&mut self) {
        let old = std::mem::take(&mut self.slots);
        self.slots.reserve(self.index.len() + 64);
        for entry in old.into_iter().skip(self.head).flatten() {
            self.index.insert(entry.0.clone(), self.slots.len());
            self.slots.push(Some(entry));
        }
        self.head = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn keys(map: &OrderedMap<u32, u32>) -> Vec<u32> {
        map.keys().copied().collect()
    }

    #[test]
    fn behaves_like_a_python_dict_for_order() {
        let mut map = OrderedMap::new();
        for key in [5, 3, 9, 1] {
            map.set(key, key * 10);
        }
        assert_eq!(keys(&map), vec![5, 3, 9, 1]);
        map.set(3, 31);
        assert_eq!(keys(&map), vec![5, 3, 9, 1]);
        assert_eq!(map.get(&3), Some(&31));
        assert!(map.move_to_end(&5));
        assert_eq!(keys(&map), vec![3, 9, 1, 5]);
        assert_eq!(map.pop_front(), Some((3, 31)));
        assert_eq!(keys(&map), vec![9, 1, 5]);
        assert!(!map.insert_if_absent(9, 0));
        assert!(map.insert_if_absent(7, 70));
        assert_eq!(keys(&map), vec![9, 1, 5, 7]);
        assert!(!map.contains_key(&3));
        assert_eq!(map.remove(&1), Some(10));
        assert_eq!(map.remove(&1), None);
        assert_eq!(keys(&map), vec![9, 5, 7]);
        assert!(map.insert_if_absent(1, 11));
        assert_eq!(keys(&map), vec![9, 5, 7, 1]);
    }

    #[test]
    fn long_churn_stays_consistent_through_compaction() {
        let mut map = OrderedMap::new();
        let mut model: Vec<u32> = Vec::new();
        for step in 0..5000u32 {
            let key = step % 300;
            if model.contains(&key) {
                model.retain(|candidate| *candidate != key);
                model.push(key);
                assert!(map.move_to_end(&key));
            } else {
                if model.len() >= 100 {
                    let oldest = model.remove(0);
                    assert_eq!(map.pop_front().map(|(key, _)| key), Some(oldest));
                }
                model.push(key);
                map.set(key, step);
            }
            assert_eq!(map.len(), model.len());
        }
        assert_eq!(keys(&map), model);
    }
}
