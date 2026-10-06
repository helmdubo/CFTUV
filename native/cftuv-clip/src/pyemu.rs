//! Version-pinned emulation of the two places where the Python result (here: the COST) depends on the minor
//! version of the interpreter that runs the oracle.
//!
//! 1. `list.sort` / `sorted(key=cmp_to_key(cmp))`: every comparison is a user call (in `ClipStageV1._ordered` a
//!    `SqrtSumV1.sign()`, which moves `SIGN_COUNTS` and may pay budget), so the COMPARISON SEQUENCE is observable.
//!    CPython 3.13 changed `count_run` (long descending runs with equal elements are one run); 3.11 and 3.13
//!    therefore ask different questions on the same input. Below 64 elements the whole sort is `count_run` followed
//!    by one binary insertion (`minrun == n`); for 64 or more the merge machinery of Timsort would be needed and is
//!    NOT ported: that is a named [`ClipError::Unsupported`], never a guess.
//! 2. float `sum()` (`offset_normal.blend`): 3.11 folds left from an `int` zero (`0 + a` is `0.0 + a`, so `-0.0`
//!    becomes `0.0`); 3.12+ keeps a Neumaier compensation after the first item.
//!
//! The interpreter version is an explicit parameter ([`PyVersion`]); a version this module does not emulate is a
//! named refusal. Both emulations are verified against the real interpreters (comparison logs, random inputs
//! with ties and descending runs): `tests/test_native_clip_parts.py`.

use crate::error::{ClipError, ClipResult};

/// An interpreter whose behaviour is emulated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PyVersion {
    /// CPython 3.11 (Blender 4.5, the product runtime).
    V311,
    /// CPython 3.13 (the dev venv and the external pool Python).
    V313,
}

impl PyVersion {
    /// `sys.version_info[:2]`; any other version is a named refusal (3.12 differs from both in details nobody verified).
    pub fn from_version(major: u32, minor: u32) -> ClipResult<PyVersion> {
        match (major, minor) {
            (3, 11) => Ok(PyVersion::V311),
            (3, 13) => Ok(PyVersion::V313),
            _ => Err(ClipError::Unsupported(format!("python {major}.{minor} is not emulated (3.11 and 3.13 are)"))),
        }
    }
}

/// Lists of this length or more need the merge machinery of Timsort (`minrun < n`): not ported.
pub const SORT_LIMIT: usize = 64;

/// `sorted(items, key=cmp_to_key(cmp))` where `less(a, b)` is `cmp(a, b) < 0`, asking CPython's questions in
/// CPython's order. The result is the stable sorted order; a failing comparison (exhaustion) ends the sort with
/// that error, the comparisons asked so far having been asked (their costs paid), as in Python.
pub fn sort_by_less<T, F>(version: PyVersion, items: Vec<T>, mut less: F) -> ClipResult<Vec<T>>
where
    F: FnMut(&T, &T) -> ClipResult<bool>,
{
    let mut items = items;
    let count = items.len();
    if count < 2 {
        return Ok(items);
    }
    if count >= SORT_LIMIT {
        return Err(ClipError::Unsupported(format!("a sort of {count} elements needs the merge machinery of list.sort (limit {SORT_LIMIT})")));
    }
    let run = match version {
        PyVersion::V311 => count_run_311(&mut items, &mut less)?,
        PyVersion::V313 => count_run_313(&mut items, &mut less)?,
    };
    binary_insertion(&mut items, run, &mut less)?;
    Ok(items)
}

/// CPython 3.11 `count_run`: a strictly descending run is reversed (strictness keeps the sort stable), an
/// ascending run extends while the next is not smaller. Returns the run length (>= 2 for two or more elements).
fn count_run_311<T, F>(items: &mut [T], less: &mut F) -> ClipResult<usize>
where
    F: FnMut(&T, &T) -> ClipResult<bool>,
{
    let count = items.len();
    let mut run = 2;
    let mut next = 2;
    if less(&items[1], &items[0])? {
        while next < count && less(&items[next], &items[next - 1])? {
            next += 1;
            run += 1;
        }
        items[..run].reverse();
    } else {
        while next < count && !less(&items[next], &items[next - 1])? {
            next += 1;
            run += 1;
        }
    }
    Ok(run)
}

/// CPython 3.13 `count_run`: an ascending prefix first; if it is all equal and followed by a smaller element the
/// run is descending (the equal prefix is reversed in place); a descending run tolerates equal elements, each block
/// of equals being reversed back to keep stability, and the run is finally reversed to ascending, then extended by a
/// naturally ascending suffix.
fn count_run_313<T, F>(items: &mut [T], less: &mut F) -> ClipResult<usize>
where
    F: FnMut(&T, &T) -> ClipResult<bool>,
{
    let remaining = items.len();
    let mut n = 1;
    while n < remaining {
        if less(&items[n], &items[n - 1])? {
            break;
        }
        n += 1;
    }
    if n == remaining {
        return Ok(n);
    }
    if n > 1 {
        if less(&items[0], &items[n - 1])? {
            return Ok(n);
        }
        items[..n].reverse();
    }
    n += 1;
    let mut equal = 0usize;
    while n < remaining {
        if less(&items[n], &items[n - 1])? {
            reverse_last_equals(items, n, &mut equal);
        } else if less(&items[n - 1], &items[n])? {
            break;
        } else {
            equal += 1;
        }
        n += 1;
    }
    reverse_last_equals(items, n, &mut equal);
    items[..n].reverse();
    while n < remaining {
        if less(&items[n], &items[n - 1])? {
            break;
        }
        n += 1;
    }
    Ok(n)
}

/// `REVERSE_LAST_NEQ`: the last `equal + 1` elements before `n` are all equal; reverse them and reset the counter.
fn reverse_last_equals<T>(items: &mut [T], n: usize, equal: &mut usize) {
    if *equal > 0 {
        let size = *equal + 1;
        items[n - size..n].reverse();
        *equal = 0;
    }
}

/// `binarysort`: the elements before `sorted` are in order; each next one is inserted after the last element
/// that is not greater (`pivot < a[mid]` goes left), which keeps equal elements in their original order.
fn binary_insertion<T, F>(items: &mut [T], sorted: usize, less: &mut F) -> ClipResult<()>
where
    F: FnMut(&T, &T) -> ClipResult<bool>,
{
    for position in sorted.max(1)..items.len() {
        let (mut low, mut high) = (0, position);
        while low < high {
            let middle = (low + high) >> 1;
            if less(&items[position], &items[middle])? {
                high = middle;
            } else {
                low = middle + 1;
            }
        }
        items[low..=position].rotate_right(1);
    }
    Ok(())
}

/// `sum(terms)` over binary64 floats, starting from the `int` 0 (the default `start`), for a non-empty list.
///
/// 3.11: a left fold, `((0 + a) + b) + c` where `0 + a` is `0.0 + a`. 3.12+: the first item is added the same way,
/// then Neumaier (improved Kahan-Babuska) compensation over the rest, the correction added at the end when it is
/// non-zero and finite. Never fused, never reordered.
pub fn float_sum(version: PyVersion, terms: &[f64]) -> f64 {
    let Some((first, rest)) = terms.split_first() else {
        return 0.0;
    };
    let mut total = 0.0 + first;
    match version {
        PyVersion::V311 => {
            for term in rest {
                total += term;
            }
            total
        }
        PyVersion::V313 => {
            let mut compensation = 0.0f64;
            for &term in rest {
                let next = total + term;
                if total.abs() >= term.abs() {
                    compensation += (total - next) + term;
                } else {
                    compensation += (term - next) + total;
                }
                total = next;
            }
            if compensation != 0.0 && compensation.is_finite() {
                total += compensation;
            }
            total
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sorted_with_log(version: PyVersion, values: &[u8]) -> (Vec<usize>, Vec<(usize, usize)>) {
        let mut log = Vec::new();
        let items: Vec<usize> = (0..values.len()).collect();
        let order = sort_by_less(version, items, |a, b| {
            log.push((*a, *b));
            Ok(values[*a] < values[*b])
        })
        .unwrap();
        (order, log)
    }

    #[test]
    fn both_versions_sort_stably_and_ask_only_questions_about_their_input() {
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut next = move |bound: u64| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state % bound) as u8
        };
        for round in 0..3000 {
            let size = 2 + (round % 61);
            let alphabet = 1 + (round % 5) as u64;
            let values: Vec<u8> = (0..size).map(|_| next(alphabet)).collect();
            for version in [PyVersion::V311, PyVersion::V313] {
                let (order, log) = sorted_with_log(version, &values);
                let mut expected: Vec<usize> = (0..size).collect();
                expected.sort_by_key(|index| values[*index]);
                assert_eq!(order, expected, "{version:?} {values:?}");
                assert!(log.iter().all(|(a, b)| *a < size && *b < size));
            }
        }
    }

    #[test]
    fn three_point_eleven_and_three_point_thirteen_ask_different_questions() {
        // [2, 2, 1, 1]: 3.13 sees ONE descending run with equal blocks, 3.11 a run of two and an insertion.
        let (_, old) = sorted_with_log(PyVersion::V311, &[2, 2, 1, 1]);
        let (_, new) = sorted_with_log(PyVersion::V313, &[2, 2, 1, 1]);
        assert_ne!(old, new);
        // below two elements nothing is asked
        assert!(sorted_with_log(PyVersion::V313, &[7]).1.is_empty());
    }

    #[test]
    fn a_long_list_is_a_named_refusal() {
        let items: Vec<usize> = (0..SORT_LIMIT).collect();
        let result = sort_by_less(PyVersion::V311, items, |a, b| Ok(a < b));
        assert!(matches!(result, Err(ClipError::Unsupported(_))));
        assert!(PyVersion::from_version(3, 12).is_err());
    }

    #[test]
    fn float_sum_follows_each_version() {
        assert_eq!(float_sum(PyVersion::V311, &[-0.0]).to_bits(), 0.0f64.to_bits());
        assert_eq!(float_sum(PyVersion::V313, &[-0.0]).to_bits(), 0.0f64.to_bits());
        assert_eq!(float_sum(PyVersion::V311, &[1e100, 1.0, -1e100]), 0.0);
        assert_eq!(float_sum(PyVersion::V313, &[1e100, 1.0, -1e100]), 1.0);
        assert_eq!(float_sum(PyVersion::V311, &[0.1, 0.2, 0.3]), 0.6000000000000001);
        assert_eq!(float_sum(PyVersion::V313, &[0.1, 0.2, 0.3]), 0.6);
    }
}
