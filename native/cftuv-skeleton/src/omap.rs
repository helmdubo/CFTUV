//! A `dict` that remembers its insertion order, the way Python's does: assigning to a key that is there keeps its place, a new key goes to the end, and a deleted key leaves
//! the order of the others alone (a key put back after a deletion goes to the end).
//!
//! The symbolic overlay is made of three of them (`vertices`, `spans`, and the refs a vertex keeps), and the oracle iterates them in insertion order wherever it builds a new
//! one from an old one (`refreshed_span_bindings`, `clone_overlay`). A slot keeps its number for the life of the map, so a number is a stable reference to an entry: the
//! exact view of the overlay hands them out as the references of its vertices and spans.

use std::collections::HashMap;
use std::hash::Hash;

use cftuv_canon::fxhash::FxBuild;

#[derive(Debug)]
pub struct OrderedMap<K, V> {
    slots: Vec<Option<(K, V)>>,
    index: HashMap<K, usize, FxBuild>,
}

impl<K: Clone + Eq + Hash, V: Clone> Clone for OrderedMap<K, V> {
    fn clone(&self) -> OrderedMap<K, V> {
        OrderedMap { slots: self.slots.clone(), index: self.index.clone() }
    }
}

impl<K: Clone + Eq + Hash, V> Default for OrderedMap<K, V> {
    fn default() -> OrderedMap<K, V> {
        OrderedMap::new()
    }
}

impl<K: Clone + Eq + Hash, V> OrderedMap<K, V> {
    pub fn new() -> OrderedMap<K, V> {
        OrderedMap { slots: Vec::new(), index: HashMap::default() }
    }

    pub fn len(&self) -> usize {
        self.index.len()
    }

    pub fn is_empty(&self) -> bool {
        self.index.is_empty()
    }

    pub fn contains_key(&self, key: &K) -> bool {
        self.index.contains_key(key)
    }

    pub fn get(&self, key: &K) -> Option<&V> {
        let slot = *self.index.get(key)?;
        self.slots[slot].as_ref().map(|(_, value)| value)
    }

    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        let slot = *self.index.get(key)?;
        self.slots[slot].as_mut().map(|(_, value)| value)
    }

    /// The slot of a key: its stable number (`at` answers the entry).
    pub fn slot_of(&self, key: &K) -> Option<usize> {
        self.index.get(key).copied()
    }

    /// The entry of a slot, or none for a deleted one and for a number beyond the map.
    pub fn at(&self, slot: usize) -> Option<(&K, &V)> {
        self.slots.get(slot)?.as_ref().map(|(key, value)| (key, value))
    }

    pub fn at_mut(&mut self, slot: usize) -> Option<(&K, &mut V)> {
        self.slots.get_mut(slot)?.as_mut().map(|(key, value)| (&*key, value))
    }

    /// `map[key] = value`: an existing key keeps its place.
    pub fn insert(&mut self, key: K, value: V) {
        match self.index.get(&key) {
            Some(slot) => self.slots[*slot] = Some((key, value)),
            None => {
                self.index.insert(key.clone(), self.slots.len());
                self.slots.push(Some((key, value)));
            }
        }
    }

    /// `del map[key]` (`pop`): the others keep their places.
    pub fn remove(&mut self, key: &K) -> Option<V> {
        let slot = self.index.remove(key)?;
        self.slots[slot].take().map(|(_, value)| value)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&K, &V)> {
        self.slots.iter().flatten().map(|(key, value)| (key, value))
    }

    pub fn keys(&self) -> impl Iterator<Item = &K> {
        self.slots.iter().flatten().map(|(key, _)| key)
    }

    pub fn values(&self) -> impl Iterator<Item = &V> {
        self.slots.iter().flatten().map(|(_, value)| value)
    }

    pub fn values_mut(&mut self) -> impl Iterator<Item = &mut V> {
        self.slots.iter_mut().flatten().map(|(_, value)| value)
    }

    /// The number of slots ever given (deleted ones included): the bound of the slot numbers.
    pub fn slot_count(&self) -> usize {
        self.slots.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insertion_order_survives_assignment_and_deletion_like_a_python_dict() {
        let mut map: OrderedMap<i64, &str> = OrderedMap::new();
        for (key, value) in [(5, "a"), (1, "b"), (9, "c")] {
            map.insert(key, value);
        }
        map.insert(1, "B");
        assert_eq!(map.iter().map(|(key, value)| (*key, *value)).collect::<Vec<_>>(), vec![(5, "a"), (1, "B"), (9, "c")]);
        assert_eq!(map.remove(&1), Some("B"));
        map.insert(1, "again");
        assert_eq!(map.keys().copied().collect::<Vec<_>>(), vec![5, 9, 1]);
        assert_eq!(map.slot_of(&9), Some(2));
        assert_eq!(map.at(1), None);
        assert_eq!(map.len(), 3);
    }
}
