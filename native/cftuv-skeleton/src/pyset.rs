//! The iteration order of a CPython `set` of small non-negative integers (`Objects/setobject.c`, the same in 3.11 and 3.13).
//!
//! The oracle's `_connected_components` builds the members of a component as a `set` of indices and iterates it: incidents with an EQUAL geometric sort key keep the order
//! of that iteration (the sort is stable), and the iteration order of a set of ints is the order of the slots of its hash table, which is NOT ascending once an index
//! exceeds the table (`{10, 3}` iterates 10 first). The order depends on the history of the set (the insertions in the order the comprehension yields them, the resizes,
//! the merges, the dummies a `difference_update` leaves), so it is reproduced by the same algorithm: open addressing with linear probes and the perturbation of the hash,
//! `hash(i) == i`. Only what the oracle does with these sets is here: `set(range(n))`, a set display, a comprehension (insertions), `update`, `-`, `difference_update`, `min`
//! and iteration.

const MINSIZE: usize = 8;
const LINEAR_PROBES: usize = 9;
const PERTURB_SHIFT: u32 = 5;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Slot {
    Empty,
    Dummy,
    Key(usize),
}

#[derive(Clone, Debug)]
pub struct PySet {
    table: Vec<Slot>,
    fill: usize,
    used: usize,
}

impl Default for PySet {
    fn default() -> PySet {
        PySet::new()
    }
}

fn insert_clean(table: &mut [Slot], key: usize) {
    let mask = table.len() - 1;
    let mut perturb = key;
    let mut index = key & mask;
    loop {
        if table[index] == Slot::Empty {
            table[index] = Slot::Key(key);
            return;
        }
        if index + LINEAR_PROBES <= mask {
            for offset in 1..=LINEAR_PROBES {
                if table[index + offset] == Slot::Empty {
                    table[index + offset] = Slot::Key(key);
                    return;
                }
            }
        }
        perturb >>= PERTURB_SHIFT;
        index = (index.wrapping_mul(5).wrapping_add(1).wrapping_add(perturb)) & mask;
    }
}

impl PySet {
    pub fn new() -> PySet {
        PySet { table: vec![Slot::Empty; MINSIZE], fill: 0, used: 0 }
    }

    /// `set(range(count))`: the insertions in ascending order.
    pub fn from_range(count: usize) -> PySet {
        let mut set = PySet::new();
        for key in 0..count {
            set.add(key);
        }
        set
    }

    pub fn len(&self) -> usize {
        self.used
    }

    pub fn is_empty(&self) -> bool {
        self.used == 0
    }

    fn mask(&self) -> usize {
        self.table.len() - 1
    }

    /// `set_table_resize(so, minused)`: the smallest power of two above `minused` (at least eight), the active entries inserted again in slot order.
    fn resize(&mut self, minused: usize) {
        let mut size = MINSIZE;
        while size <= minused {
            size <<= 1;
        }
        let mut table = vec![Slot::Empty; size];
        for slot in &self.table {
            if let Slot::Key(key) = slot {
                insert_clean(&mut table, *key);
            }
        }
        self.table = table;
        self.fill = self.used;
    }

    /// `set_add_entry`.
    pub fn add(&mut self, key: usize) {
        let mask = self.mask();
        let mut index = key & mask;
        let mut perturb = key;
        let mut free: Option<usize> = None;
        loop {
            let probes = if index + LINEAR_PROBES <= mask { LINEAR_PROBES } else { 0 };
            for offset in 0..=probes {
                match self.table[index + offset] {
                    Slot::Empty => {
                        match free {
                            Some(slot) => {
                                self.used += 1;
                                self.table[slot] = Slot::Key(key);
                            }
                            None => {
                                self.fill += 1;
                                self.used += 1;
                                self.table[index + offset] = Slot::Key(key);
                                if self.fill * 5 >= mask * 3 {
                                    let minused = if self.used > 50_000 { self.used * 2 } else { self.used * 4 };
                                    self.resize(minused);
                                }
                            }
                        }
                        return;
                    }
                    Slot::Key(found) if found == key => return,
                    Slot::Dummy if free.is_none() => free = Some(index + offset),
                    _ => {}
                }
            }
            perturb >>= PERTURB_SHIFT;
            index = (index.wrapping_mul(5).wrapping_add(1).wrapping_add(perturb)) & mask;
        }
    }

    /// `set_lookkey`: the slot of a key.
    fn find(&self, key: usize) -> Option<usize> {
        let mask = self.mask();
        let mut index = key & mask;
        let mut perturb = key;
        loop {
            let probes = if index + LINEAR_PROBES <= mask { LINEAR_PROBES } else { 0 };
            for offset in 0..=probes {
                match self.table[index + offset] {
                    Slot::Empty => return None,
                    Slot::Key(found) if found == key => return Some(index + offset),
                    _ => {}
                }
            }
            perturb >>= PERTURB_SHIFT;
            index = (index.wrapping_mul(5).wrapping_add(1).wrapping_add(perturb)) & mask;
        }
    }

    pub fn contains(&self, key: usize) -> bool {
        self.find(key).is_some()
    }

    /// `set_discard_entry`: the slot becomes a dummy (`fill` stays).
    pub fn discard(&mut self, key: usize) {
        if let Some(slot) = self.find(key) {
            self.table[slot] = Slot::Dummy;
            self.used -= 1;
        }
    }

    /// The keys in the order of the slots (what a `for` over the set yields).
    pub fn iter(&self) -> impl Iterator<Item = usize> + '_ {
        self.table.iter().filter_map(|slot| if let Slot::Key(key) = slot { Some(*key) } else { None })
    }

    pub fn min(&self) -> Option<usize> {
        self.iter().min()
    }

    /// `so.update(other)` for a set `other` (`set_merge`).
    pub fn update(&mut self, other: &PySet) {
        if other.used == 0 {
            return;
        }
        if (self.fill + other.used) * 5 >= self.mask() * 3 {
            self.resize((self.used + other.used) * 2);
        }
        if self.fill == 0 && self.mask() == other.mask() && other.fill == other.used {
            for (slot, taken) in self.table.iter_mut().zip(&other.table) {
                if let Slot::Key(key) = taken {
                    *slot = Slot::Key(*key);
                }
            }
            self.fill = other.fill;
            self.used = other.used;
            return;
        }
        if self.fill == 0 {
            self.fill = other.used;
            self.used = other.used;
            for slot in &other.table {
                if let Slot::Key(key) = slot {
                    insert_clean(&mut self.table, *key);
                }
            }
            return;
        }
        for slot in &other.table {
            if let Slot::Key(key) = slot {
                self.add(*key);
            }
        }
    }

    /// `set(so)`: `set_copy`, a merge into an empty set.
    pub fn copy(&self) -> PySet {
        let mut result = PySet::new();
        result.update(self);
        result
    }

    /// `so - other` (`set_difference`): a copy with the common keys discarded when `so` is much larger than `other`, else the keys of `so` not in `other`, added in slot order.
    pub fn difference(&self, other: &PySet) -> PySet {
        if (self.used >> 2) > other.used {
            let mut result = self.copy();
            result.difference_update(other);
            return result;
        }
        let mut result = PySet::new();
        for key in self.iter() {
            if !other.contains(key) {
                result.add(key);
            }
        }
        result
    }

    /// `so.difference_update(other)` (`set_difference_update_internal`): every key of `other` discarded, and when more than a quarter of the table is dummies the table is
    /// rebuilt without them (a smaller table when the keys left fit one).
    pub fn difference_update(&mut self, other: &PySet) {
        let keys: Vec<usize> = other.iter().collect();
        for key in keys {
            self.discard(key);
        }
        if self.fill - self.used > self.mask() / 4 {
            self.resize(if self.used > 50_000 { self.used * 2 } else { self.used * 4 });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_set_of_small_ints_iterates_ascending_while_they_fit_the_table() {
        let set = PySet::from_range(8);
        assert_eq!(set.iter().collect::<Vec<_>>(), (0..8).collect::<Vec<_>>());
        let mut set = PySet::new();
        for key in [3, 1, 2] {
            set.add(key);
        }
        assert_eq!(set.iter().collect::<Vec<_>>(), vec![1, 2, 3]);
    }

    #[test]
    fn an_index_beyond_the_table_lands_by_its_hash_and_not_ascending() {
        // {3} then 10: 10 & 7 = 2, a slot before 3 (this is CPython's own answer: `list({3, 10})` is `[10, 3]`)
        let mut set = PySet::new();
        set.add(3);
        set.add(10);
        assert_eq!(set.iter().collect::<Vec<_>>(), vec![10, 3]);
    }

    #[test]
    fn a_table_grows_when_it_is_three_fifths_full_and_keeps_the_keys() {
        let mut set = PySet::new();
        for key in 0..20 {
            set.add(key);
        }
        assert_eq!(set.len(), 20);
        let mut keys: Vec<usize> = set.iter().collect();
        keys.sort_unstable();
        assert_eq!(keys, (0..20).collect::<Vec<_>>());
        assert!((0..20).all(|key| set.contains(key)) && !set.contains(20));
    }

    #[test]
    fn discarding_leaves_a_dummy_and_the_next_insertion_may_reuse_it() {
        let mut set = PySet::from_range(5);
        set.discard(2);
        assert_eq!(set.len(), 4);
        assert!(!set.contains(2));
        set.add(2);
        assert_eq!(set.iter().collect::<Vec<_>>(), vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn a_difference_update_that_leaves_many_dummies_rebuilds_the_table_smaller() {
        // CPython: `s = set(range(39)); s.difference_update(<33 of them>)` gives `[1, 36, 37, 17, 24, 26]` (the table is rebuilt into 32 slots)
        let mut pending = PySet::from_range(39);
        let mut gone = PySet::new();
        for key in [0, 5, 8, 10, 11, 12, 13, 14, 19, 22, 23, 25, 28, 32, 34, 3, 4, 7, 9, 15, 18, 20, 21, 27, 30, 31, 35, 38, 2, 33, 6, 16, 29] {
            gone.add(key);
        }
        pending.difference_update(&gone);
        assert_eq!(pending.iter().collect::<Vec<_>>(), vec![1, 36, 37, 17, 24, 26]);
    }

    #[test]
    fn update_difference_and_copy_keep_the_membership() {
        let mut pending = PySet::from_range(12);
        let mut component = PySet::new();
        component.add(4);
        let mut joined = PySet::new();
        for key in [9, 11, 6] {
            joined.add(key);
        }
        component.update(&joined);
        assert_eq!(component.len(), 4);
        let rest = pending.difference(&component);
        assert_eq!(rest.len(), 8);
        assert!(!rest.contains(9) && rest.contains(10));
        pending.difference_update(&component);
        assert_eq!(pending.len(), 8);
        assert_eq!(pending.min(), Some(0));
        let copied = pending.copy();
        assert_eq!(copied.len(), 8);
    }
}
