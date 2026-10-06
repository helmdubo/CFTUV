//! Port of `kernel/src/cftuv_envelope/_cpython311.py`: the semantics of CPython 3.11 that the kernel names explicitly, so that
//! the native cost (and a native float answer) does not depend on the interpreter that happens to run the host.
//!
//! 1. [`sort_by_less`] is `sorted_as_cpython311(items, compare)`: `list.sort` of CPython 3.11 (`Objects/listobject.c`) for EVERY
//!    length, comparison for comparison: `count_run` (a strictly descending run is reversed, a non-descending one is not), binary
//!    insertion up to `minrun`, the Powersort merge strategy (`found_new_run`, `powerloop`), `merge_force_collapse`, `merge_at`
//!    with `gallop_left` / `gallop_right`, `merge_lo` / `merge_hi` with the `min_gallop` that lives between the merges of one
//!    sort. Every comparison in `ClipStageV1._ordered` is a `SqrtSumV1.sign()` (it moves `SIGN_COUNTS` and may pay budget), so
//!    the SEQUENCE of questions is observable and must be CPython's. `less(a, b)` is `compare(a, b) < 0`. A failing comparison
//!    (budget exhaustion) ends the sort with that error, the comparisons asked so far having been asked.
//! 2. [`left_fold_sum`] is `left_fold_sum(terms)`: `0 + t0 + t1 + ...` from left to right, without compensation (`0 + a` is
//!    `0.0 + a`, so `-0.0` becomes `0.0`).
//!
//! The kernel's functions have no state and no classes; this port keeps the same shape (a struct holds the list, the comparison
//! and the `min_gallop` field of `MergeState`; the stack of runs is `[start, length, power]`). Indices are `isize` because the
//! Python code lets a pointer step one before the start of a run when the count reaches zero.
//! Verified against the Python function (comparison logs, random inputs of 0..300 elements with ties and runs, both
//! interpreters): `tests/test_native_clip_parts.py`.

use crate::error::ClipResult;

const MIN_GALLOP: isize = 7;

/// `left_fold_sum(terms)` over binary64 floats (a non-empty list: the Python function returns the `int` 0 for none).
pub fn left_fold_sum(terms: &[f64]) -> f64 {
    let mut total = 0.0f64;
    for term in terms {
        total += term;
    }
    total
}

/// `sorted_as_cpython311(items, compare)` with `less(a, b)` = `compare(a, b) < 0`: the stable sorted order, asking CPython's questions in CPython's order.
pub fn sort_by_less<T, F>(items: Vec<T>, mut less: F) -> ClipResult<Vec<T>>
where
    T: Copy,
    F: FnMut(&T, &T) -> ClipResult<bool>,
{
    let mut sorter = Sorter { keys: items, less: &mut less, min_gallop: MIN_GALLOP };
    sorter.run()?;
    Ok(sorter.keys)
}

struct Sorter<'a, T, F> {
    keys: Vec<T>,
    less: &'a mut F,
    /// `MergeState.min_gallop`: lives between the merges of one sort.
    min_gallop: isize,
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
fn count_run<T, F>(keys: &[T], low: isize, high: isize, less: &mut F) -> ClipResult<(isize, bool)>
where
    F: FnMut(&T, &T) -> ClipResult<bool>,
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
fn gallop_left<T, F>(key: &T, array: &[T], base: isize, count: isize, hint: isize, less: &mut F) -> ClipResult<isize>
where
    F: FnMut(&T, &T) -> ClipResult<bool>,
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
fn gallop_right<T, F>(key: &T, array: &[T], base: isize, count: isize, hint: isize, less: &mut F) -> ClipResult<isize>
where
    F: FnMut(&T, &T) -> ClipResult<bool>,
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

impl<T, F> Sorter<'_, T, F>
where
    T: Copy,
    F: FnMut(&T, &T) -> ClipResult<bool>,
{
    fn run(&mut self) -> ClipResult<()> {
        let size = self.keys.len() as isize;
        let mut remaining = size;
        if remaining < 2 {
            return Ok(());
        }
        let mut pending: Vec<Run> = Vec::new();
        let mut low = 0isize;
        let minimum = min_run(remaining);
        while remaining > 0 {
            let (mut run, descending) = count_run(&self.keys, low, low + remaining, &mut *self.less)?;
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
    fn binary_insertion(&mut self, low: isize, high: isize, start: isize) -> ClipResult<()> {
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
            self.keys.copy_within(span(left, start - left), (left + 1) as usize);
            self.keys[left as usize] = pivot;
            start += 1;
        }
        Ok(())
    }

    /// `found_new_run` of 3.11: a new run of length `count` is found; merge the runs of the stack with a greater power of their boundary.
    fn found_new_run(&mut self, pending: &mut Vec<Run>, count: isize, size: isize) -> ClipResult<()> {
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

    fn merge_force_collapse(&mut self, pending: &mut Vec<Run>) -> ClipResult<()> {
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
    fn merge_at(&mut self, pending: &mut Vec<Run>, index: usize) -> ClipResult<()> {
        let (mut base_a, mut count_a) = (pending[index][0], pending[index][1]);
        let (base_b, mut count_b) = (pending[index + 1][0], pending[index + 1][1]);
        pending[index][1] = count_a + count_b;
        if index + 3 == pending.len() {
            pending[index + 1] = pending[index + 2];
        }
        pending.pop();
        let skip = gallop_right(&self.keys[base_b as usize], &self.keys, base_a, count_a, 0, &mut *self.less)?;
        base_a += skip;
        count_a -= skip;
        if count_a == 0 {
            return Ok(());
        }
        count_b = gallop_left(&self.keys[(base_a + count_a - 1) as usize], &self.keys, base_b, count_b, count_b - 1, &mut *self.less)?;
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
    fn merge_low(&mut self, base_a: isize, mut count_a: isize, base_b: isize, mut count_b: isize) -> ClipResult<()> {
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
    fn merge_high(&mut self, base_a: isize, mut count_a: isize, base_b: isize, mut count_b: isize) -> ClipResult<()> {
        let held: Vec<T> = self.keys[span(base_b, count_b)].to_vec();
        let (mut dest, mut in_a, mut in_b) = (base_b + count_b - 1, base_a + count_a - 1, count_b - 1);
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
                let run = count_a - gallop_right(&key, &self.keys, base_a, count_a, count_a - 1, &mut *self.less)?;
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
                let run = count_b - gallop_left(&key, &held, 0, count_b, count_b - 1, &mut *self.less)?;
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A xorshift sequence: the same inputs on every run and every machine.
    struct Random(u64);

    impl Random {
        fn below(&mut self, bound: u64) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0 % bound
        }
    }

    fn sorted_with_log(values: &[u64]) -> (Vec<usize>, Vec<(usize, usize)>) {
        let mut log = Vec::new();
        let order = sort_by_less((0..values.len()).collect::<Vec<usize>>(), |a, b| {
            log.push((*a, *b));
            Ok(values[*a] < values[*b])
        })
        .unwrap();
        (order, log)
    }

    fn assert_sorts(values: &[u64]) {
        let (order, log) = sorted_with_log(values);
        let mut expected: Vec<usize> = (0..values.len()).collect();
        expected.sort_by_key(|index| values[*index]);
        assert_eq!(order, expected, "{values:?}");
        assert!(log.iter().all(|(a, b)| *a < values.len() && *b < values.len()));
        if values.len() < 2 {
            assert!(log.is_empty(), "nothing is asked below two elements");
        }
    }

    #[test]
    fn every_length_sorts_stably_whatever_the_ties_and_the_runs() {
        let mut random = Random(0x2545_f491_4f6c_dd1d);
        for size in 0..=300usize {
            for alphabet in [1u64, 2, 3, 7, 1000] {
                let values: Vec<u64> = (0..size).map(|_| random.below(alphabet)).collect();
                assert_sorts(&values);
            }
        }
    }

    #[test]
    fn long_inputs_with_runs_blocks_and_sawtooth_reach_the_merges_and_the_galloping() {
        let mut random = Random(0x9e37_79b9_7f4a_7c15);
        for round in 0..400usize {
            let size = 64 + round % 500;
            let mut values: Vec<u64> = Vec::with_capacity(size);
            while values.len() < size {
                let length = 1 + random.below(90) as usize;
                let start = random.below(50);
                let step = random.below(3);
                for position in 0..length {
                    let value = match round % 4 {
                        0 => start + step * position as u64,
                        1 => start + 3 * (length - position) as u64,
                        2 => random.below(6),
                        _ => start + (position as u64 * 7) % 11,
                    };
                    values.push(value);
                }
            }
            values.truncate(size);
            assert_sorts(&values);
        }
    }

    #[test]
    fn ascending_and_descending_inputs_ask_the_least() {
        let ascending: Vec<u64> = (0..200).collect();
        assert_eq!(sorted_with_log(&ascending).1.len(), 199);
        let descending: Vec<u64> = (0..200).rev().collect();
        assert_eq!(sorted_with_log(&descending).1.len(), 199);
        // CPython 3.11 reverses only a STRICTLY descending run: equal elements stay in order
        let (order, _) = sorted_with_log(&[2, 2, 1, 1]);
        assert_eq!(order, vec![2, 3, 0, 1]);
    }

    #[test]
    fn a_failing_comparison_ends_the_sort_with_its_error() {
        let mut asked = 0;
        let result = sort_by_less((0..100usize).rev().collect::<Vec<usize>>(), |_a, _b| {
            asked += 1;
            if asked == 5 {
                Err(crate::error::ClipError::Value("exhausted".into()))
            } else {
                Ok(false)
            }
        });
        assert!(result.is_err());
        assert_eq!(asked, 5);
    }

    #[test]
    fn the_left_fold_has_no_compensation() {
        assert_eq!(left_fold_sum(&[-0.0]).to_bits(), 0.0f64.to_bits());
        assert_eq!(left_fold_sum(&[1e100, 1.0, -1e100]), 0.0);
        assert_eq!(left_fold_sum(&[0.1, 0.2, 0.3]), 0.6000000000000001);
    }
}
