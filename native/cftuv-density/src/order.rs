//! Generic fallible CPython 3.11 сортировка, стабильная и comparison-for-comparison.
//!
//! Алгоритм перенесён из `cftuv-clip/src/cpython311.rs`, без зависимости на Clip
//! или алгебру V1: только тип ошибки и generic оболочка отделены от прежнего
//! проверенного алгоритма. Неперенесённые binary64 helpers здесь отсутствуют.
//! CPython source: https://github.com/python/cpython/blob/v3.11.11/Objects/listobject.c
//!
//! ARCHITECTURAL_DEBT: R3_DENSITY_CPYTHON311_SORT_COPY
//! Локальная копия нужна стене V2: зависимость на cftuv-clip притащила бы V1.
//! Origin — native/cftuv-clip/src/cpython311.rs; gate — tests/order.rs,
//! full comparator traces CPython311/313, stable ties и ранние отказы.
//! Trigger pay-down: согласованный standalone generic sorting crate; тогда
//! обе Rust-копии свести в него вместе с их trace-gates, без изменения V1 сейчас.

use crate::farey::{FareyOrdering, FareyPredicates, PrimitiveVector};
use crate::order_set::TupleSet;

const MIN_GALLOP: isize = 7;

/// Sorted copy без требования Copy/Clone к T; comparator может отказать.
/// Порядок вопросов — CPython311, в том числе при 0 для различных объектов.
pub fn sorted_as_cpython311<T, E, F>(items: Vec<T>, mut compare: F) -> Result<Vec<T>, E>
where
    F: FnMut(&T, &T) -> Result<i8, E>,
{
    let indices = sort_copy_by_less((0..items.len()).collect(), |left, right| {
        compare(&items[*left], &items[*right]).map(|order| order < 0)
    })?;
    let mut slots: Vec<Option<T>> = items.into_iter().map(Some).collect();
    Ok(indices
        .into_iter()
        .map(|index| slots[index].take().expect("sort permutation"))
        .collect())
}

/// Canonical tuple-set iteration и actual CPython311 sort для Farey search.
#[derive(Default)]
pub struct CanonicalOrdering;

impl<P: FareyPredicates> FareyOrdering<P> for CanonicalOrdering {
    fn iteration_order(
        &mut self,
        _ordinal: usize,
        insertion_order: &[PrimitiveVector],
    ) -> Result<Vec<usize>, P::Error> {
        // Farey даёт уникальный insertion log. Пропущенный duplicate add
        // не меняет таблицу и не вызывает resize, поэтому rebuild эквивалентен
        // всем source .update(tuple_of_shell_candidates) без deletions.
        let mut set = TupleSet::new();
        set.update(insertion_order.iter());
        Ok(set.insertion_indices().collect())
    }

    fn sort(
        &mut self,
        ordinal: usize,
        items: &[PrimitiveVector],
        predicates: &mut P,
    ) -> Result<Vec<usize>, P::Error> {
        sorted_as_cpython311((0..items.len()).collect(), |left, right| {
            predicates.candidate_compare(ordinal, &items[*left], &items[*right])
        })
    }
}

/// `sorted_as_cpython311(items, compare)` with `less(a, b)` = `compare(a, b) < 0`: the stable sorted order, asking CPython's questions in CPython's order.
fn sort_copy_by_less<T, F, E>(items: Vec<T>, mut less: F) -> Result<Vec<T>, E>
where
    T: Copy,
    F: FnMut(&T, &T) -> Result<bool, E>,
{
    let mut sorter = Sorter {
        keys: items,
        less: &mut less,
        min_gallop: MIN_GALLOP,
        _error: std::marker::PhantomData,
    };
    sorter.run()?;
    Ok(sorter.keys)
}

struct Sorter<'a, T, F, E> {
    keys: Vec<T>,
    less: &'a mut F,
    /// `MergeState.min_gallop`: lives between the merges of one sort.
    min_gallop: isize,
    _error: std::marker::PhantomData<E>,
}

/// A run of the stack: `[start, length, power]`.
type Run = [isize; 3];

fn min_run(mut count: isize) -> isize {
    let mut carry = 0;
    while count >= 64 {
        carry |= count & 1;
        count >>= 1;
    }
    count + carry
}

/// `(length, descending)` of the natural run at `low` of `keys[..high]` (`count_run` of 3.11).
fn count_run<T, F, E>(keys: &[T], low: isize, high: isize, less: &mut F) -> Result<(isize, bool), E>
where
    F: FnMut(&T, &T) -> Result<bool, E>,
{
    let mut low = low + 1;
    if low == high {
        return Ok((1, false));
    }
    let mut count = 2;
    if less(&keys[low as usize], &keys[(low - 1) as usize])? {
        low += 1;
        while low < high && less(&keys[low as usize], &keys[(low - 1) as usize])? {
            low += 1;
            count += 1;
        }
        return Ok((count, true));
    }
    low += 1;
    while low < high && !less(&keys[low as usize], &keys[(low - 1) as usize])? {
        low += 1;
        count += 1;
    }
    Ok((count, false))
}

/// The leftmost position of `key` in the sorted `array[base..base + count]`, galloping from `hint` (`gallop_left` of 3.11).
fn gallop_left<T, F, E>(
    key: &T,
    array: &[T],
    base: isize,
    count: isize,
    hint: isize,
    less: &mut F,
) -> Result<isize, E>
where
    F: FnMut(&T, &T) -> Result<bool, E>,
{
    let at = |index: isize| &array[index as usize];
    let (mut last, mut offset, position) = (0isize, 1isize, base + hint);
    if less(at(position), key)? {
        let limit = count - hint;
        while offset < limit {
            if less(at(position + offset), key)? {
                last = offset;
                offset = (offset << 1) + 1;
            } else {
                break;
            }
        }
        offset = offset.min(limit);
        last += hint;
        offset += hint;
    } else {
        let limit = hint + 1;
        while offset < limit {
            if less(at(position - offset), key)? {
                break;
            }
            last = offset;
            offset = (offset << 1) + 1;
        }
        offset = offset.min(limit);
        (last, offset) = (hint - offset, hint - last);
    }
    last += 1;
    while last < offset {
        let middle = last + ((offset - last) >> 1);
        if less(at(base + middle), key)? {
            last = middle + 1;
        } else {
            offset = middle;
        }
    }
    Ok(offset)
}

/// The rightmost position of `key` in the sorted `array[base..base + count]`, galloping from `hint` (`gallop_right` of 3.11).
fn gallop_right<T, F, E>(
    key: &T,
    array: &[T],
    base: isize,
    count: isize,
    hint: isize,
    less: &mut F,
) -> Result<isize, E>
where
    F: FnMut(&T, &T) -> Result<bool, E>,
{
    let at = |index: isize| &array[index as usize];
    let (mut last, mut offset, position) = (0isize, 1isize, base + hint);
    if less(key, at(position))? {
        let limit = hint + 1;
        while offset < limit {
            if less(key, at(position - offset))? {
                last = offset;
                offset = (offset << 1) + 1;
            } else {
                break;
            }
        }
        offset = offset.min(limit);
        (last, offset) = (hint - offset, hint - last);
    } else {
        let limit = count - hint;
        while offset < limit {
            if less(key, at(position + offset))? {
                break;
            }
            last = offset;
            offset = (offset << 1) + 1;
        }
        offset = offset.min(limit);
        last += hint;
        offset += hint;
    }
    last += 1;
    while last < offset {
        let middle = last + ((offset - last) >> 1);
        if less(key, at(base + middle))? {
            offset = middle;
        } else {
            last = middle + 1;
        }
    }
    Ok(offset)
}

/// The range of a Python slice `[start:start + count]` (`count <= 0` is an empty slice).
fn span(start: isize, count: isize) -> std::ops::Range<usize> {
    let count = count.max(0) as usize;
    let start = start.max(0) as usize;
    start..start + count
}

impl<T, F, E> Sorter<'_, T, F, E>
where
    T: Copy,
    F: FnMut(&T, &T) -> Result<bool, E>,
{
    fn run(&mut self) -> Result<(), E> {
        let size = self.keys.len() as isize;
        let mut remaining = size;
        if remaining < 2 {
            return Ok(());
        }
        let mut pending: Vec<Run> = Vec::new();
        let mut low = 0isize;
        let minimum = min_run(remaining);
        while remaining > 0 {
            let (mut run, descending) =
                count_run(&self.keys, low, low + remaining, &mut *self.less)?;
            if descending {
                self.keys[span(low, run)].reverse();
            }
            if run < minimum {
                let forced = remaining.min(minimum);
                self.binary_insertion(low, low + forced, low + run)?;
                run = forced;
            }
            self.found_new_run(&mut pending, run, size)?;
            pending.push([low, run, 0]);
            low += run;
            remaining -= run;
        }
        self.merge_force_collapse(&mut pending)
    }

    /// `binarysort` of 3.11: `keys[low..start]` is sorted, `keys[start..high]` is inserted into it one by one.
    fn binary_insertion(&mut self, low: isize, high: isize, start: isize) -> Result<(), E> {
        let mut start = if low == start { start + 1 } else { start };
        while start < high {
            let (mut left, mut right) = (low, start);
            let pivot = self.keys[start as usize];
            while left < right {
                let middle = left + ((right - left) >> 1);
                if (self.less)(&pivot, &self.keys[middle as usize])? {
                    right = middle;
                } else {
                    left = middle + 1;
                }
            }
            self.keys
                .copy_within(span(left, start - left), (left + 1) as usize);
            self.keys[left as usize] = pivot;
            start += 1;
        }
        Ok(())
    }

    /// `found_new_run` of 3.11: a new run of length `count` is found; merge the runs of the stack with a greater power of their boundary.
    fn found_new_run(
        &mut self,
        pending: &mut Vec<Run>,
        count: isize,
        size: isize,
    ) -> Result<(), E> {
        if let Some(&last) = pending.last() {
            let depth = power(last[0], last[1], count, size);
            while pending.len() > 1 && pending[pending.len() - 2][2] > depth {
                let index = pending.len() - 2;
                self.merge_at(pending, index)?;
            }
            pending.last_mut().expect("the stack was not empty")[2] = depth;
        }
        Ok(())
    }

    fn merge_force_collapse(&mut self, pending: &mut Vec<Run>) -> Result<(), E> {
        while pending.len() > 1 {
            let mut index = pending.len() - 2;
            if index > 0 && pending[index - 1][1] < pending[index + 1][1] {
                index -= 1;
            }
            self.merge_at(pending, index)?;
        }
        Ok(())
    }

    /// Merges `pending[index]` and `pending[index + 1]` (`merge_at` of 3.11).
    fn merge_at(&mut self, pending: &mut Vec<Run>, index: usize) -> Result<(), E> {
        let (mut base_a, mut count_a) = (pending[index][0], pending[index][1]);
        let (base_b, mut count_b) = (pending[index + 1][0], pending[index + 1][1]);
        pending[index][1] = count_a + count_b;
        if index + 3 == pending.len() {
            pending[index + 1] = pending[index + 2];
        }
        pending.pop();
        let skip = gallop_right(
            &self.keys[base_b as usize],
            &self.keys,
            base_a,
            count_a,
            0,
            &mut *self.less,
        )?;
        base_a += skip;
        count_a -= skip;
        if count_a == 0 {
            return Ok(());
        }
        count_b = gallop_left(
            &self.keys[(base_a + count_a - 1) as usize],
            &self.keys,
            base_b,
            count_b,
            count_b - 1,
            &mut *self.less,
        )?;
        if count_b <= 0 {
            return Ok(());
        }
        if count_a <= count_b {
            self.merge_low(base_a, count_a, base_b, count_b)
        } else {
            self.merge_high(base_a, count_a, base_b, count_b)
        }
    }

    /// Writes `held[from..from + count]` at `dest` (a Python slice assignment from the held copy).
    fn put_held(&mut self, dest: isize, held: &[T], from: isize, count: isize) {
        let source = span(from, count);
        self.keys[span(dest, count)].copy_from_slice(&held[source]);
    }

    /// `keys[dest:dest + count] = keys[from:from + count]` (the right side is a copy, so the ranges may overlap).
    fn shift(&mut self, dest: isize, from: isize, count: isize) {
        if count > 0 {
            self.keys.copy_within(span(from, count), dest as usize);
        }
    }

    /// Merges `a` (the shorter, held as a copy) and `b` from left to right (`merge_lo` of 3.11).
    fn merge_low(
        &mut self,
        base_a: isize,
        mut count_a: isize,
        base_b: isize,
        mut count_b: isize,
    ) -> Result<(), E> {
        let held: Vec<T> = self.keys[span(base_a, count_a)].to_vec();
        let (mut dest, mut in_a, mut in_b) = (base_a, 0isize, base_b);
        self.keys[dest as usize] = self.keys[in_b as usize];
        dest += 1;
        in_b += 1;
        count_b -= 1;
        if count_b == 0 {
            self.put_held(dest, &held, in_a, count_a);
            return Ok(());
        }
        if count_a == 1 {
            self.shift(dest, in_b, count_b);
            self.keys[(dest + count_b) as usize] = held[in_a as usize];
            return Ok(());
        }
        let mut min_gallop = self.min_gallop;
        loop {
            let (mut wins_a, mut wins_b) = (0isize, 0isize);
            loop {
                if (self.less)(&self.keys[in_b as usize], &held[in_a as usize])? {
                    self.keys[dest as usize] = self.keys[in_b as usize];
                    dest += 1;
                    in_b += 1;
                    wins_b += 1;
                    wins_a = 0;
                    count_b -= 1;
                    if count_b == 0 {
                        self.put_held(dest, &held, in_a, count_a);
                        return Ok(());
                    }
                    if wins_b >= min_gallop {
                        break;
                    }
                } else {
                    self.keys[dest as usize] = held[in_a as usize];
                    dest += 1;
                    in_a += 1;
                    wins_a += 1;
                    wins_b = 0;
                    count_a -= 1;
                    if count_a == 1 {
                        self.shift(dest, in_b, count_b);
                        self.keys[(dest + count_b) as usize] = held[in_a as usize];
                        return Ok(());
                    }
                    if wins_a >= min_gallop {
                        break;
                    }
                }
            }
            min_gallop += 1;
            loop {
                min_gallop -= isize::from(min_gallop > 1);
                self.min_gallop = min_gallop;
                let key = self.keys[in_b as usize];
                let run = gallop_right(&key, &held, in_a, count_a, 0, &mut *self.less)?;
                wins_a = run;
                if run != 0 {
                    self.put_held(dest, &held, in_a, run);
                    dest += run;
                    in_a += run;
                    count_a -= run;
                    if count_a == 1 {
                        self.shift(dest, in_b, count_b);
                        self.keys[(dest + count_b) as usize] = held[in_a as usize];
                        return Ok(());
                    }
                    if count_a == 0 {
                        return Ok(());
                    }
                }
                self.keys[dest as usize] = self.keys[in_b as usize];
                dest += 1;
                in_b += 1;
                count_b -= 1;
                if count_b == 0 {
                    self.put_held(dest, &held, in_a, count_a);
                    return Ok(());
                }
                let key = held[in_a as usize];
                let run = gallop_left(&key, &self.keys, in_b, count_b, 0, &mut *self.less)?;
                wins_b = run;
                if run != 0 {
                    self.shift(dest, in_b, run);
                    dest += run;
                    in_b += run;
                    count_b -= run;
                    if count_b == 0 {
                        self.put_held(dest, &held, in_a, count_a);
                        return Ok(());
                    }
                }
                self.keys[dest as usize] = held[in_a as usize];
                dest += 1;
                in_a += 1;
                count_a -= 1;
                if count_a == 1 {
                    self.shift(dest, in_b, count_b);
                    self.keys[(dest + count_b) as usize] = held[in_a as usize];
                    return Ok(());
                }
                if !(wins_a >= MIN_GALLOP || wins_b >= MIN_GALLOP) {
                    break;
                }
            }
            min_gallop += 1;
            self.min_gallop = min_gallop;
        }
    }

    /// Merges `a` and `b` (the shorter, held as a copy) from right to left (`merge_hi` of 3.11).
    fn merge_high(
        &mut self,
        base_a: isize,
        mut count_a: isize,
        base_b: isize,
        mut count_b: isize,
    ) -> Result<(), E> {
        let held: Vec<T> = self.keys[span(base_b, count_b)].to_vec();
        let (mut dest, mut in_a, mut in_b) =
            (base_b + count_b - 1, base_a + count_a - 1, count_b - 1);
        self.keys[dest as usize] = self.keys[in_a as usize];
        dest -= 1;
        in_a -= 1;
        count_a -= 1;
        if count_a == 0 {
            self.put_held(dest - count_b + 1, &held, 0, count_b);
            return Ok(());
        }
        if count_b == 1 {
            self.shift(dest - count_a + 1, in_a - count_a + 1, count_a);
            self.keys[(dest - count_a) as usize] = held[in_b as usize];
            return Ok(());
        }
        let mut min_gallop = self.min_gallop;
        loop {
            let (mut wins_a, mut wins_b) = (0isize, 0isize);
            loop {
                if (self.less)(&held[in_b as usize], &self.keys[in_a as usize])? {
                    self.keys[dest as usize] = self.keys[in_a as usize];
                    dest -= 1;
                    in_a -= 1;
                    wins_a += 1;
                    wins_b = 0;
                    count_a -= 1;
                    if count_a == 0 {
                        self.put_held(dest - count_b + 1, &held, 0, count_b);
                        return Ok(());
                    }
                    if wins_a >= min_gallop {
                        break;
                    }
                } else {
                    self.keys[dest as usize] = held[in_b as usize];
                    dest -= 1;
                    in_b -= 1;
                    wins_b += 1;
                    wins_a = 0;
                    count_b -= 1;
                    if count_b == 1 {
                        self.shift(dest - count_a + 1, in_a - count_a + 1, count_a);
                        self.keys[(dest - count_a) as usize] = held[in_b as usize];
                        return Ok(());
                    }
                    if wins_b >= min_gallop {
                        break;
                    }
                }
            }
            min_gallop += 1;
            loop {
                min_gallop -= isize::from(min_gallop > 1);
                self.min_gallop = min_gallop;
                let key = held[in_b as usize];
                let run = count_a
                    - gallop_right(
                        &key,
                        &self.keys,
                        base_a,
                        count_a,
                        count_a - 1,
                        &mut *self.less,
                    )?;
                wins_a = run;
                if run != 0 {
                    dest -= run;
                    in_a -= run;
                    self.shift(dest + 1, in_a + 1, run);
                    count_a -= run;
                    if count_a == 0 {
                        self.put_held(dest - count_b + 1, &held, 0, count_b);
                        return Ok(());
                    }
                }
                self.keys[dest as usize] = held[in_b as usize];
                dest -= 1;
                in_b -= 1;
                count_b -= 1;
                if count_b == 1 {
                    self.shift(dest - count_a + 1, in_a - count_a + 1, count_a);
                    self.keys[(dest - count_a) as usize] = held[in_b as usize];
                    return Ok(());
                }
                let key = self.keys[in_a as usize];
                let run =
                    count_b - gallop_left(&key, &held, 0, count_b, count_b - 1, &mut *self.less)?;
                wins_b = run;
                if run != 0 {
                    dest -= run;
                    in_b -= run;
                    self.put_held(dest + 1, &held, in_b + 1, run);
                    count_b -= run;
                    if count_b == 1 {
                        self.shift(dest - count_a + 1, in_a - count_a + 1, count_a);
                        self.keys[(dest - count_a) as usize] = held[in_b as usize];
                        return Ok(());
                    }
                    if count_b == 0 {
                        return Ok(());
                    }
                }
                self.keys[dest as usize] = self.keys[in_a as usize];
                dest -= 1;
                in_a -= 1;
                count_a -= 1;
                if count_a == 0 {
                    self.put_held(dest - count_b + 1, &held, 0, count_b);
                    return Ok(());
                }
                if !(wins_a >= MIN_GALLOP || wins_b >= MIN_GALLOP) {
                    break;
                }
            }
            min_gallop += 1;
            self.min_gallop = min_gallop;
        }
    }
}

/// The power of the boundary of two neighbouring runs (`powerloop` of 3.11): its depth in the binary tree of merges of Munro and Wild.
fn power(start: isize, first: isize, second: isize, size: isize) -> isize {
    let mut result = 0;
    let mut left = 2 * start + first;
    let mut right = left + first + second;
    loop {
        result += 1;
        if left >= size {
            left -= size;
            right -= size;
        } else if right >= size {
            break;
        }
        left <<= 1;
        right <<= 1;
    }
    result
}
