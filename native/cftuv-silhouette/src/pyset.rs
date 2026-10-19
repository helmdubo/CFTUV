//! The iteration order of a CPython 3.11 `set` of small non-negative `int`s built by `add` alone (no removal): the order a `for` over it
//! sees. The silhouette law iterates such sets (`incident[key]`, the face numbers of a vertex) where the order decides which exact predicate is
//! asked first, so the order is part of the cost.
//!
//! `hash(n) == n` for `0 <= n < 2**61 - 1`; the table starts at eight slots, an insertion probes `set_add_entry` (linear probes only when nine
//! slots fit after the start, then the perturbed jump), and the table grows (`set_table_resize`, entries reinserted in slot order by
//! `set_insert_clean`) once `fill * 5 >= mask * 3`.

const MINSIZE: usize = 8;
const LINEAR_PROBES: usize = 9;
const PERTURB_SHIFT: u32 = 5;

/// A simulated `set` of small non-negative ints, in table layout.
#[derive(Debug, Clone)]
pub struct PySmallIntSet {
    table: Vec<Option<u64>>,
    fill: usize,
}

impl Default for PySmallIntSet {
    fn default() -> PySmallIntSet {
        PySmallIntSet { table: vec![None; MINSIZE], fill: 0 }
    }
}

impl PySmallIntSet {
    pub fn len(&self) -> usize {
        self.fill
    }

    pub fn is_empty(&self) -> bool {
        self.fill == 0
    }

    pub fn contains(&self, value: u64) -> bool {
        self.table.iter().any(|slot| *slot == Some(value))
    }

    /// `s.add(value)`.
    pub fn add(&mut self, value: u64) {
        let mask = self.table.len() - 1;
        let hash = value;
        let mut index = (hash as usize) & mask;
        let mut perturb = hash;
        loop {
            let probes = if index + LINEAR_PROBES <= mask { LINEAR_PROBES } else { 0 };
            let mut at = index;
            let mut remaining = probes as isize;
            loop {
                match self.table[at] {
                    None => {
                        self.table[at] = Some(value);
                        self.fill += 1;
                        if self.fill * 5 >= mask * 3 {
                            self.resize(self.fill * 4);
                        }
                        return;
                    }
                    Some(existing) if existing == value => return,
                    Some(_) => {}
                }
                at += 1;
                if remaining == 0 {
                    break;
                }
                remaining -= 1;
            }
            perturb >>= PERTURB_SHIFT;
            index = (index.wrapping_mul(5).wrapping_add(1).wrapping_add(perturb as usize)) & mask;
        }
    }

    fn resize(&mut self, minused: usize) {
        let mut size = MINSIZE;
        while size <= minused {
            size <<= 1;
        }
        let old = std::mem::replace(&mut self.table, vec![None; size]);
        let mask = size - 1;
        for value in old.into_iter().flatten() {
            let mut index = (value as usize) & mask;
            let mut perturb = value;
            'place: loop {
                if self.table[index].is_none() {
                    self.table[index] = Some(value);
                    break 'place;
                }
                if index + LINEAR_PROBES <= mask {
                    for step in 1..=LINEAR_PROBES {
                        if self.table[index + step].is_none() {
                            self.table[index + step] = Some(value);
                            break 'place;
                        }
                    }
                }
                perturb >>= PERTURB_SHIFT;
                index = (index.wrapping_mul(5).wrapping_add(1).wrapping_add(perturb as usize)) & mask;
            }
        }
    }

    /// The order of `for item in s`.
    pub fn iter(&self) -> impl Iterator<Item = u64> + '_ {
        self.table.iter().filter_map(|slot| *slot)
    }

    /// `sorted(s)`.
    pub fn sorted(&self) -> Vec<u64> {
        let mut items: Vec<u64> = self.iter().collect();
        items.sort_unstable();
        items
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_sets_iterate_by_slot() {
        let mut set = PySmallIntSet::default();
        set.add(9);
        set.add(2);
        assert_eq!(set.iter().collect::<Vec<_>>(), vec![9, 2], "9 & 7 = 1 comes before 2");
        let mut colliding = PySmallIntSet::default();
        colliding.add(3);
        colliding.add(11);
        // 11 & 7 = 3 is taken: (3 * 5 + 1 + (11 >> 5)) & 7 = 0
        assert_eq!(colliding.iter().collect::<Vec<_>>(), vec![11, 3]);
    }
}
