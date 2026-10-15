//! CPython64 set(tuple[int,int]): только add/update(iterable), без удаления.
//!
//! Алгоритмы и hash constants сверены с CPython v3.11.11:
//! https://github.com/python/cpython/blob/v3.11.11/Objects/setobject.c
//! https://github.com/python/cpython/blob/v3.11.11/Objects/longobject.c
//! https://github.com/python/cpython/blob/v3.11.11/Objects/tupleobject.c
//! Порядок — slot order; rebuild — прежний slot order, не Rust HashSet/sort.

use cftuv_radical::arith::magnitude;

use crate::farey::PrimitiveVector;
use crate::{IBig, UBig};

const HASH_MODULUS: u64 = (1 << 61) - 1;
const PRIME_1: u64 = 11_400_714_785_074_694_791;
const PRIME_2: u64 = 14_029_467_366_897_019_727;
const PRIME_5: u64 = 2_870_177_450_012_600_261;
const MINSIZE: usize = 8;
const LINEAR_PROBES: usize = 9;
const PERTURB_SHIFT: u32 = 5;

/// long_hash: signed magnitude modulo 2^61-1, затем -1 -> -2.
pub fn integer_hash(value: &IBig) -> i64 {
    let remainder = magnitude(value) % UBig::from(HASH_MODULUS);
    let magnitude = u64::try_from(remainder).expect("hash residue fits 61 bits") as i64;
    let hash = if value < &IBig::ZERO {
        -magnitude
    } else {
        magnitude
    };
    if hash == -1 { -2 } else { hash }
}

/// tuplehash длины два; unsigned wrapping arithmetic и rotate_left(31).
pub fn tuple_hash(value: &PrimitiveVector) -> i64 {
    let mut accumulator = PRIME_5;
    for lane in [integer_hash(&value.0), integer_hash(&value.1)] {
        accumulator = accumulator.wrapping_add((lane as u64).wrapping_mul(PRIME_2));
        accumulator = accumulator.rotate_left(31).wrapping_mul(PRIME_1);
    }
    accumulator = accumulator.wrapping_add(2 ^ (PRIME_5 ^ 3_527_539));
    if accumulator == u64::MAX {
        1_546_275_796
    } else {
        accumulator as i64
    }
}

#[derive(Clone, Copy, Debug)]
struct Entry {
    hash: i64,
    index: usize,
}

/// Arena сохраняет unique insertion order; таблица хранит только индексы.
#[derive(Clone, Debug)]
pub struct TupleSet {
    table: Vec<Option<Entry>>,
    keys: Vec<PrimitiveVector>,
}

impl Default for TupleSet {
    fn default() -> Self {
        Self::new()
    }
}

fn insert_clean(table: &mut [Option<Entry>], entry: Entry) {
    let mask = table.len() - 1;
    let mut perturb = entry.hash as u64;
    let mut index = (perturb as usize) & mask;
    loop {
        if table[index].is_none() {
            table[index] = Some(entry);
            return;
        }
        if index + LINEAR_PROBES <= mask {
            for offset in 1..=LINEAR_PROBES {
                if table[index + offset].is_none() {
                    table[index + offset] = Some(entry);
                    return;
                }
            }
        }
        perturb >>= PERTURB_SHIFT;
        index = (index
            .wrapping_mul(5)
            .wrapping_add(1)
            .wrapping_add(perturb as usize))
            & mask;
    }
}

impl TupleSet {
    pub fn new() -> Self {
        Self {
            table: vec![None; MINSIZE],
            keys: Vec::new(),
        }
    }
    pub fn len(&self) -> usize {
        self.keys.len()
    }
    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }
    pub fn capacity(&self) -> usize {
        self.table.len()
    }

    /// true только для новой пары; duplicate не меняет ни table, ни resize.
    pub fn add(&mut self, key: PrimitiveVector) -> bool {
        let hash = tuple_hash(&key);
        let mask = self.table.len() - 1;
        let mut perturb = hash as u64;
        let mut index = (perturb as usize) & mask;
        loop {
            let probes = if index + LINEAR_PROBES <= mask {
                LINEAR_PROBES
            } else {
                0
            };
            for offset in 0..=probes {
                match self.table[index + offset] {
                    Some(entry) if entry.hash == hash && self.keys[entry.index] == key => {
                        return false;
                    }
                    None => {
                        let key_index = self.keys.len();
                        self.keys.push(key);
                        self.table[index + offset] = Some(Entry {
                            hash,
                            index: key_index,
                        });
                        // Без deletions fill == used, как у исходного search.
                        if self.len() * 5 >= mask * 3 {
                            self.resize(if self.len() > 50_000 {
                                self.len() * 2
                            } else {
                                self.len() * 4
                            });
                        }
                        return true;
                    }
                    _ => {}
                }
            }
            perturb >>= PERTURB_SHIFT;
            index = (index
                .wrapping_mul(5)
                .wrapping_add(1)
                .wrapping_add(perturb as usize))
                & mask;
        }
    }

    /// Source .update(tuple_of_shell_candidates): без set-merge preallocation.
    pub fn update<'a>(&mut self, iterable: impl IntoIterator<Item = &'a PrimitiveVector>) {
        for key in iterable {
            self.add(key.clone());
        }
    }

    pub fn insertion_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.table
            .iter()
            .filter_map(|slot| slot.map(|entry| entry.index))
    }

    pub fn iter(&self) -> impl Iterator<Item = &PrimitiveVector> + '_ {
        self.insertion_indices().map(|index| &self.keys[index])
    }

    fn resize(&mut self, minimum: usize) {
        let mut size = MINSIZE;
        while size <= minimum {
            size <<= 1;
        }
        let mut table = vec![None; size];
        for entry in self.table.iter().flatten() {
            insert_clean(&mut table, *entry);
        }
        self.table = table;
    }
}
