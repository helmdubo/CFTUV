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
    ClosurePlans,
    ClosureCompile,
    ClosureOverlay,
    ClosureDiscover,
    ClosureGenerations,
    ClosureSignature,
    ClosureClone,
    ClosureKeys,
    ClosureSortKey,
    OverlayHydrate,
    OverlayPorts,
    OverlayLeaves,
    OverlayVertices,
    OverlayWiring,
}

/// `(name, phase)` in the order of the answer of [`take`].
pub const PHASES: [(&str, Phase); 32] = [
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
    ("closure: plans of the packet", Phase::ClosurePlans),
    ("closure: compile contacts", Phase::ClosureCompile),
    ("closure: overlays", Phase::ClosureOverlay),
    ("closure: discoveries", Phase::ClosureDiscover),
    ("closure: generations", Phase::ClosureGenerations),
    ("closure: signatures", Phase::ClosureSignature),
    ("closure: clones", Phase::ClosureClone),
    ("closure: point and time keys", Phase::ClosureKeys),
    ("closure: sort keys of incidents", Phase::ClosureSortKey),
    ("overlay: hydrate f0", Phase::OverlayHydrate),
    ("overlay: line ports", Phase::OverlayPorts),
    ("overlay: leaf bindings", Phase::OverlayLeaves),
    ("overlay: initial vertices", Phase::OverlayVertices),
    ("overlay: wiring and spans", Phase::OverlayWiring),
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

/// Counters of the leaf (events that are not time): how many lookups hit, how many keys were built twice. Only the feature `profile` counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stat {
    PositionLookups,
    PositionHits,
    PointKeys,
    PointKeysRepeated,
    TermsOf,
    ReprBuilds,
    ReprBytes,
    ValNew,
    EventPoints,
    PlanDivisors,
    PlanDivisorsRepeated,
    CompareTimes,
    CompareTimesDecidedByFilter,
    CompareTimesRepeated,
    TimeKeys,
    TimeKeysRepeated,
    ReprRoots,
    ReprRootsRepeated,
    ReprHeads,
    ReprHeadTies,
    ValNone,
    ValBool,
    ValSmallInt,
    ValBigInt,
    ValStr,
    ValFrac,
    ValTuple,
    ValData,
    ValOther,
}

pub const STATS: [(&str, Stat); 29] = [
    ("position lookups", Stat::PositionLookups),
    ("position hits", Stat::PositionHits),
    ("exact_point_key calls", Stat::PointKeys),
    ("exact_point_key of an already keyed point", Stat::PointKeysRepeated),
    ("Val::terms_of calls", Stat::TermsOf),
    ("repr texts built", Stat::ReprBuilds),
    ("repr bytes", Stat::ReprBytes),
    ("Val::new", Stat::ValNew),
    ("event_point calls", Stat::EventPoints),
    ("conjugation plans (pair divisions)", Stat::PlanDivisors),
    ("... of a divisor planned before", Stat::PlanDivisorsRepeated),
    ("compare_times", Stat::CompareTimes),
    ("... decided by the filter", Stat::CompareTimesDecidedByFilter),
    ("... of a pair of times compared before", Stat::CompareTimesRepeated),
    ("time keys (memo keys)", Stat::TimeKeys),
    ("... of a time keyed before", Stat::TimeKeysRepeated),
    ("repr roots", Stat::ReprRoots),
    ("... structurally equal to an earlier root", Stat::ReprRootsRepeated),
    ("repr heads built (bounded text)", Stat::ReprHeads),
    ("repr comparisons tied on the heads", Stat::ReprHeadTies),
    ("Val None", Stat::ValNone),
    ("Val bool", Stat::ValBool),
    ("Val int < 256", Stat::ValSmallInt),
    ("Val int", Stat::ValBigInt),
    ("Val str", Stat::ValStr),
    ("Val Fraction", Stat::ValFrac),
    ("Val tuple", Stat::ValTuple),
    ("Val dataclass", Stat::ValData),
    ("Val other", Stat::ValOther),
];

#[cfg(feature = "profile")]
mod stats_on {
    use super::{Stat, STATS};
    use std::cell::RefCell;
    use std::collections::HashSet;

    thread_local! {
        static COUNTS: RefCell<[u64; STATS.len()]> = const { RefCell::new([0; STATS.len()]) };
        static SEEN: RefCell<HashSet<(u8, u64)>> = RefCell::new(HashSet::new());
    }

    pub fn count(stat: Stat, by: u64) {
        COUNTS.with(|counts| counts.borrow_mut()[stat as usize] += by);
    }

    /// Counts `stat` and, when `key` was seen before in this measurement, `repeated` too.
    pub fn count_seen(stat: Stat, repeated: Stat, key: impl FnOnce() -> u64) {
        let key = key();
        count(stat, 1);
        if !SEEN.with(|seen| seen.borrow_mut().insert((stat as u8, key))) {
            count(repeated, 1);
        }
    }

    thread_local! {
        static SITES: RefCell<std::collections::HashMap<(&'static str, u32), u64>> = RefCell::new(std::collections::HashMap::new());
    }

    /// Counts a value made at `site` (the caller of the constructor).
    pub fn count_site(site: &'static std::panic::Location<'static>) {
        SITES.with(|sites| *sites.borrow_mut().entry((site.file(), site.line())).or_default() += 1);
    }

    /// The sites that made values, most first.
    pub fn take_sites() -> Vec<(String, u64)> {
        SITES.with(|sites| {
            let mut rows: Vec<(String, u64)> = sites.borrow().iter().map(|((file, line), count)| (format!("{file}:{line}"), *count)).collect();
            rows.sort_by_key(|row| std::cmp::Reverse(row.1));
            rows
        })
    }

    pub fn reset_stats() {
        COUNTS.with(|counts| *counts.borrow_mut() = [0; STATS.len()]);
        SEEN.with(|seen| seen.borrow_mut().clear());
        SITES.with(|sites| sites.borrow_mut().clear());
    }

    pub fn take_stats() -> Vec<u64> {
        COUNTS.with(|counts| counts.borrow().to_vec())
    }
}

#[cfg(feature = "profile")]
pub use stats_on::{count, count_seen, count_site, reset_stats, take_sites, take_stats};

#[cfg(not(feature = "profile"))]
#[inline(always)]
pub fn count(_stat: Stat, _by: u64) {}

#[cfg(not(feature = "profile"))]
#[inline(always)]
pub fn count_seen(_stat: Stat, _repeated: Stat, _key: impl FnOnce() -> u64) {}

#[cfg(not(feature = "profile"))]
pub fn take_sites() -> Vec<(String, u64)> {
    Vec::new()
}

#[cfg(not(feature = "profile"))]
pub fn reset_stats() {}

#[cfg(not(feature = "profile"))]
pub fn take_stats() -> Vec<u64> {
    Vec::new()
}
