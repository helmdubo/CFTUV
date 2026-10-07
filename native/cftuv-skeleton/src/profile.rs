//! Phase timers of the leaf behind the cargo feature `profile` (off by default; the tools build a separate wheel with it, see `tools/native_leaf_gate.py profile`).
//!
//! `timed(phase, || work)` charges the time of `work` to `phase` and takes it away from the phase it was called in, so the phases add up to the whole and a
//! nested phase is not counted twice. Without the feature `timed` is the call of `work` and nothing else; `take()` answers an empty list.

/// What the time of an evaluate call (or of a graph build) is spent on. `Other` is everything not named (the memory lookups, the view, the glue).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Other,
    CompareTimes,
    ConcurrencyTime,
    SlidingTime,
    SlidingPoint,
    EventPointRadical,
    EventPointArithmetic,
    EventPointDivide,
    DifferenceSign,
    MemoKey,
    MemoLookup,
    AlongSpan,
    GraphVelocity,
    GraphExtent,
    GraphCells,
    GraphProjection,
    GraphReach,
    GraphCrashes,
}

/// `(name, phase)` in the order of the answer of [`take`].
pub const PHASES: [(&str, Phase); 18] = [
    ("other", Phase::Other),
    ("compare_times", Phase::CompareTimes),
    ("concurrency_time", Phase::ConcurrencyTime),
    ("sliding_time", Phase::SlidingTime),
    ("sliding_point", Phase::SlidingPoint),
    ("event_point radical", Phase::EventPointRadical),
    ("event_point arithmetic", Phase::EventPointArithmetic),
    ("event_point divide", Phase::EventPointDivide),
    ("difference_sign", Phase::DifferenceSign),
    ("memo keys", Phase::MemoKey),
    ("memo lookups", Phase::MemoLookup),
    ("projections on a span", Phase::AlongSpan),
    ("graph: bisector velocity", Phase::GraphVelocity),
    ("graph: boxes of the march", Phase::GraphExtent),
    ("graph: cells and walls of a step", Phase::GraphCells),
    ("graph: projection into a wall", Phase::GraphProjection),
    ("graph: reach test", Phase::GraphReach),
    ("graph: crashes into traces", Phase::GraphCrashes),
];

#[cfg(feature = "profile")]
mod on {
    use super::{Phase, PHASES};
    use std::cell::RefCell;
    use std::time::Instant;

    struct State {
        current: Phase,
        last: Instant,
        nanoseconds: [u64; PHASES.len()],
    }

    thread_local! {
        static STATE: RefCell<State> = RefCell::new(State { current: Phase::Other, last: Instant::now(), nanoseconds: [0; PHASES.len()] });
    }

    fn charge(state: &mut State, now: Instant) {
        let index = state.current as usize;
        state.nanoseconds[index] += now.duration_since(state.last).as_nanos() as u64;
        state.last = now;
    }

    pub fn timed<T>(phase: Phase, work: impl FnOnce() -> T) -> T {
        let parent = STATE.with(|state| {
            let mut state = state.borrow_mut();
            charge(&mut state, Instant::now());
            std::mem::replace(&mut state.current, phase)
        });
        let answer = work();
        STATE.with(|state| {
            let mut state = state.borrow_mut();
            charge(&mut state, Instant::now());
            state.current = parent;
        });
        answer
    }

    /// Starts a measurement: everything charged so far is dropped.
    pub fn reset() {
        STATE.with(|state| *state.borrow_mut() = State { current: Phase::Other, last: Instant::now(), nanoseconds: [0; PHASES.len()] });
    }

    /// The nanoseconds of every phase since [`reset`], in the order of [`PHASES`].
    pub fn take() -> Vec<u64> {
        STATE.with(|state| {
            let mut state = state.borrow_mut();
            charge(&mut state, Instant::now());
            state.nanoseconds.to_vec()
        })
    }
}

#[cfg(feature = "profile")]
pub use on::{reset, take, timed};

#[cfg(not(feature = "profile"))]
#[inline(always)]
pub fn timed<T>(_phase: Phase, work: impl FnOnce() -> T) -> T {
    work()
}

#[cfg(not(feature = "profile"))]
pub fn reset() {}

#[cfg(not(feature = "profile"))]
pub fn take() -> Vec<u64> {
    Vec::new()
}
