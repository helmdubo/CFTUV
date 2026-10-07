//! `EventQueueV1` (`wavefront/events.py`): events ordered by their exact time, a whole level taken at once.
//!
//! The Python queue is `heapq` over entries whose `__lt__` is `compare_times(...) < 0` (a tie is `false`); the comparison SEQUENCE of that heap
//! is cost (each comparison is a sign), so the heap is the transcription in `heap.rs` and the comparator is [`compare_times`] on the entries' times with
//! the call's context. `count_at_time` (telemetry the builder asks every level) walks the heap tree by an explicit stack and its comparisons count too.

use cftuv_core::exact::ExactCtx;

use crate::error::SkelResult;
use crate::heap::{heappop, heappush};
use crate::time::{compare_times, EventTime, PointRef, TimeRef};

/// `EventKind`: the value is the enum's `.value`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EventKind {
    Edge,
    Multiway,
    Split,
    Start,
    Switch,
}

impl EventKind {
    pub const ALL: [EventKind; 5] = [EventKind::Edge, EventKind::Multiway, EventKind::Split, EventKind::Start, EventKind::Switch];

    pub fn from_value(value: &str) -> Option<EventKind> {
        EventKind::ALL.into_iter().find(|kind| kind.value() == value)
    }

    pub fn value(self) -> &'static str {
        match self {
            EventKind::Edge => "EDGE",
            EventKind::Multiway => "MULTIWAY",
            EventKind::Split => "SPLIT",
            EventKind::Start => "START",
            EventKind::Switch => "SWITCH",
        }
    }
}

/// `CandidateEventV1` (the flag `span_unproven` is not part of its identity).
#[derive(Debug, Clone)]
pub struct CandidateEvent {
    pub kind: EventKind,
    pub time: TimeRef,
    pub point: PointRef,
    pub vertex: i64,
    pub peer: i64,
    pub edge: i64,
    pub span_unproven: bool,
}

#[derive(Debug, Clone)]
struct Entry {
    event: CandidateEvent,
    sequence: u64,
}

/// `EventQueueV1`.
#[derive(Debug, Default)]
pub struct EventQueue {
    heap: Vec<Entry>,
    counter: u64,
    pub pushed: u64,
    pub popped: u64,
}

fn entry_less<'a, 'b>(ctx: &'a mut ExactCtx<'b>) -> impl FnMut(&Entry, &Entry) -> SkelResult<bool> + use<'a, 'b> {
    move |left, right| Ok(compare_times(ctx, &left.event.time, &right.event.time)? < 0)
}

impl EventQueue {
    pub fn new() -> EventQueue {
        EventQueue::default()
    }

    pub fn len(&self) -> usize {
        self.heap.len()
    }

    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    /// `push`: the sequence number is taken before the heap is touched (as `next(self._counter)` is an argument of the call), `pushed` counts a push that finished.
    pub fn push(&mut self, ctx: &mut ExactCtx<'_>, event: CandidateEvent) -> SkelResult<()> {
        let sequence = self.counter;
        self.counter += 1;
        heappush(&mut self.heap, Entry { event, sequence }, &mut entry_less(ctx))?;
        self.pushed += 1;
        Ok(())
    }

    /// `peek_time`: the minimum time without taking it.
    pub fn peek_time(&self) -> Option<&TimeRef> {
        self.heap.first().map(|entry| &entry.event.time)
    }

    /// `_count_at_time`: how many queued events have exactly this time. The heap puts no child before its parent, so a node strictly later than `time`
    /// has no equal below it and its subtree is cut; the nodes walked are the nodes compared.
    pub fn count_at_time(&self, ctx: &mut ExactCtx<'_>, time: &EventTime) -> SkelResult<usize> {
        let size = self.heap.len();
        let mut count = 0;
        let mut pending: Vec<usize> = if size > 0 { vec![0] } else { Vec::new() };
        while let Some(index) = pending.pop() {
            let order = compare_times(ctx, &self.heap[index].event.time, time)?;
            if order > 0 {
                continue;
            }
            count += usize::from(order == 0);
            pending.extend([2 * index + 1, 2 * index + 2].into_iter().filter(|child| *child < size));
        }
        Ok(count)
    }

    /// `pop_level`: the head and every event whose time is EXACTLY the head's.
    pub fn pop_level(&mut self, ctx: &mut ExactCtx<'_>) -> SkelResult<Vec<CandidateEvent>> {
        let Some(head) = heappop(&mut self.heap, &mut entry_less(ctx))? else {
            return Ok(Vec::new());
        };
        let mut level = vec![head.event.clone()];
        while let Some(next) = self.heap.first() {
            if compare_times(ctx, &next.event.time, &head.event.time)? != 0 {
                break;
            }
            let Some(taken) = heappop(&mut self.heap, &mut entry_less(ctx))? else {
                break;
            };
            level.push(taken.event);
        }
        self.popped += level.len() as u64;
        Ok(level)
    }

    /// The sequence numbers of the heap array in its order (the arrangement the oracle's `_heap` has).
    pub fn arrangement(&self) -> Vec<u64> {
        self.heap.iter().map(|entry| entry.sequence).collect()
    }
}
