//! CPython's `heapq` (the C implementation of `_heapqmodule.c`, which `heapq.heappush` / `heappop` call) over a FALLIBLE `less`.
//!
//! The comparison SEQUENCE is part of the cost: `EventQueueV1` compares entries with `compare_times`, each comparison pays sign counters (and may
//! run the conjugation), so the port must ask exactly the questions CPython asks, in the same order and with the same operands. The shapes, as the
//! C code has them:
//!
//! * push: append, then `siftdown(0, last)`: while the new item is not at the root, compare `item < parent`, stop at the first `false`, else swap with
//!   the parent;
//! * pop: remove the last element; if the heap is now empty it is the answer; otherwise the root is the answer, the last element takes the root's place
//!   and `siftup(0)` runs: walk down to a leaf choosing, at each level, the left child when `left < right` and the right one otherwise (a lone left child
//!   is taken without a comparison), swapping the item down with each step; then `siftdown(start, leaf)` brings it back up.
//!
//! `less(a, b)` is `a < b` (a tie is `false`, so equal entries keep the structure order, as the oracle's `_QueueEntry.__lt__` does). A refusal
//! inside `less` stops the operation where CPython's exception would: the heap is left as it is at that moment (the oracle's builder is discarded after
//! an exhaustion, so the partial arrangement is never read).

/// `heapq.heappush(heap, item)`.
pub fn heappush<T, E>(heap: &mut Vec<T>, item: T, less: &mut impl FnMut(&T, &T) -> Result<bool, E>) -> Result<(), E> {
    heap.push(item);
    let last = heap.len() - 1;
    siftdown(heap, 0, last, less)
}

/// `heapq.heappop(heap)`; `None` for an empty heap (the oracle guards it before calling).
pub fn heappop<T, E>(heap: &mut Vec<T>, less: &mut impl FnMut(&T, &T) -> Result<bool, E>) -> Result<Option<T>, E> {
    let Some(last) = heap.pop() else {
        return Ok(None);
    };
    if heap.is_empty() {
        return Ok(Some(last));
    }
    let answer = std::mem::replace(&mut heap[0], last);
    siftup(heap, 0, less)?;
    Ok(Some(answer))
}

fn siftdown<T, E>(heap: &mut [T], start: usize, mut position: usize, less: &mut impl FnMut(&T, &T) -> Result<bool, E>) -> Result<(), E> {
    while position > start {
        let parent = (position - 1) >> 1;
        if !less(&heap[position], &heap[parent])? {
            break;
        }
        heap.swap(position, parent);
        position = parent;
    }
    Ok(())
}

fn siftup<T, E>(heap: &mut [T], mut position: usize, less: &mut impl FnMut(&T, &T) -> Result<bool, E>) -> Result<(), E> {
    let end = heap.len();
    let start = position;
    let limit = end >> 1;
    while position < limit {
        let mut child = 2 * position + 1;
        if child + 1 < end && !less(&heap[child], &heap[child + 1])? {
            child += 1;
        }
        heap.swap(position, child);
        position = child;
    }
    siftdown(heap, start, position, less)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    /// The pure-Python reference of `heapq.py`, transcribed independently (item held aside, parents moved down, not swapped): the same arrangement and the same questions.
    fn reference_push(heap: &mut Vec<i64>, item: i64, log: &mut Vec<(i64, i64)>) {
        heap.push(item);
        let mut position = heap.len() - 1;
        let newitem = heap[position];
        while position > 0 {
            let parent = (position - 1) >> 1;
            log.push((newitem, heap[parent]));
            if newitem < heap[parent] {
                heap[position] = heap[parent];
                position = parent;
                continue;
            }
            break;
        }
        heap[position] = newitem;
    }

    fn reference_pop(heap: &mut Vec<i64>, log: &mut Vec<(i64, i64)>) -> Option<i64> {
        let last = heap.pop()?;
        if heap.is_empty() {
            return Some(last);
        }
        let answer = heap[0];
        heap[0] = last;
        let end = heap.len();
        let mut position = 0;
        let newitem = heap[0];
        let mut child = 1;
        while child < end {
            let right = child + 1;
            if right < end {
                log.push((heap[child], heap[right]));
                if !(heap[child] < heap[right]) {
                    child = right;
                }
            }
            heap[position] = heap[child];
            position = child;
            child = 2 * position + 1;
        }
        heap[position] = newitem;
        let mut current = position;
        while current > 0 {
            let parent = (current - 1) >> 1;
            log.push((newitem, heap[parent]));
            if newitem < heap[parent] {
                heap[current] = heap[parent];
                current = parent;
                continue;
            }
            break;
        }
        heap[current] = newitem;
        Some(answer)
    }

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
    }

    #[test]
    fn the_comparison_sequence_and_the_arrangement_equal_the_reference_on_tie_heavy_input() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        for round in 0..400 {
            let alphabet = 2 + (round % 9) as u64;
            let (mut heap, mut reference) = (Vec::new(), Vec::new());
            let log = RefCell::new(Vec::new());
            let mut reference_log = Vec::new();
            let mut less = |left: &i64, right: &i64| -> Result<bool, ()> {
                log.borrow_mut().push((*left, *right));
                Ok(left < right)
            };
            for _ in 0..(5 + rng.next() % 120) {
                if rng.next() % 4 == 0 && !heap.is_empty() {
                    let popped = heappop(&mut heap, &mut less).unwrap();
                    assert_eq!(popped, reference_pop(&mut reference, &mut reference_log));
                } else {
                    let item = (rng.next() % alphabet) as i64;
                    heappush(&mut heap, item, &mut less).unwrap();
                    reference_push(&mut reference, item, &mut reference_log);
                }
                assert_eq!(heap, reference);
            }
            while !heap.is_empty() {
                let popped = heappop(&mut heap, &mut less).unwrap();
                assert_eq!(popped, reference_pop(&mut reference, &mut reference_log));
                assert_eq!(heap, reference);
            }
            assert_eq!(*log.borrow(), reference_log, "round {round}");
        }
    }

    #[test]
    fn a_refusal_stops_at_the_comparison_that_raised() {
        let mut heap = vec![1, 5, 2, 9, 7];
        let mut calls = 0;
        let mut less = |_: &i64, _: &i64| -> Result<bool, &'static str> {
            calls += 1;
            if calls == 2 {
                Err("exhausted")
            } else {
                Ok(true)
            }
        };
        assert_eq!(heappush(&mut heap, 0, &mut less), Err("exhausted"));
        assert_eq!(calls, 2);
    }
}
