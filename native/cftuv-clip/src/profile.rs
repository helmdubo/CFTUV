//! Phase timers of the clip stage, compiled in only with the cargo feature `profile` (the example `clip_profile` turns
//! it on). A scope measures its own wall time and hands it to its parent as child time, so every phase reports both its
//! INCLUSIVE time and its EXCLUSIVE (self) time: the exclusive column sums to the compute time and says where it goes.
//!
//! Without the feature a scope is a zero-sized value with no `Drop`: nothing is measured and nothing is paid.

/// What a scope measures. The names are the oracle's functions (or the exact operation they spend their time in).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Snap,
    BuildCells,
    StageNew,
    Cut,
    Slot,
    CheapSign,
    LineValue,
    ExactSign,
    WithinEdgeGap,
    CornerOfGap,
    Crossing,
    DividedBy,
    ProductAdded,
    Intern,
    Candidates,
    SegmentIn,
    EdgePoints,
    Ordered,
    Clip,
    Merged,
    Closed,
    Covers,
    ShoelaceSign,
    HasRightTurn,
    Triangulate,
    PieceChord,
    ChordOf,
    Emit,
    KeyOf,
    HomeOf,
    LiftKnown,
    Prove,
    SplitForLaw,
    Glued,
    Refined,
    BoundaryIs,
    Suppress,
    Contour,
    Finish,
}

pub const PHASES: [(Phase, &str); 39] = [
    (Phase::Snap, "snap_source_vertices"),
    (Phase::BuildCells, "build_cells"),
    (Phase::StageNew, "Stage::new (rest)"),
    (Phase::Cut, "_cut (rest)"),
    (Phase::Slot, "_slot (rest)"),
    (Phase::CheapSign, "_cheap_sign"),
    (Phase::LineValue, "line_value (_value)"),
    (Phase::ExactSign, "exact::sign"),
    (Phase::WithinEdgeGap, "within_edge_gap"),
    (Phase::CornerOfGap, "_corner_of_gap (rest)"),
    (Phase::Crossing, "_crossing (rest)"),
    (Phase::DividedBy, "exact::divided_by"),
    (Phase::ProductAdded, "product_added"),
    (Phase::Intern, "_node (point_key)"),
    (Phase::Candidates, "_candidates (+window)"),
    (Phase::SegmentIn, "_segment_in (rest)"),
    (Phase::EdgePoints, "edge_points (rest)"),
    (Phase::Ordered, "_ordered (sort)"),
    (Phase::Clip, "_clip (rest)"),
    (Phase::Merged, "_merged"),
    (Phase::Closed, "_closed (rest)"),
    (Phase::Covers, "_covers_by_construction"),
    (Phase::ShoelaceSign, "shoelace_sign"),
    (Phase::HasRightTurn, "has_right_turn"),
    (Phase::Triangulate, "triangulate_exact"),
    (Phase::PieceChord, "piece_chord (rest)"),
    (Phase::ChordOf, "chord_of"),
    (Phase::Emit, "_emit (rest)"),
    (Phase::KeyOf, "_key (rest)"),
    (Phase::HomeOf, "_home (rest)"),
    (Phase::LiftKnown, "lift_known"),
    (Phase::Prove, "_prove (rest)"),
    (Phase::SplitForLaw, "_split_for_law (rest)"),
    (Phase::Glued, "_glued (rest)"),
    (Phase::Refined, "_refined (rest)"),
    (Phase::BoundaryIs, "_boundary_is"),
    (Phase::Suppress, "_suppress_noise (rest)"),
    (Phase::Contour, "_contour (rest)"),
    (Phase::Finish, "run: lists, counters, note"),
];

/// `(phase name, calls, exclusive nanoseconds, inclusive nanoseconds)` since the last [`take`].
pub type Row = (&'static str, u64, u64, u64);

#[cfg(feature = "profile")]
mod on {
    use std::cell::RefCell;
    use std::time::Instant;

    use super::{Phase, Row, PHASES};

    struct Frame {
        phase: usize,
        start: Instant,
        child: u64,
    }

    struct State {
        stack: Vec<Frame>,
        calls: [u64; PHASES.len()],
        exclusive: [u64; PHASES.len()],
        inclusive: [u64; PHASES.len()],
    }

    thread_local! {
        static STATE: RefCell<State> = const { RefCell::new(State { stack: Vec::new(), calls: [0; PHASES.len()], exclusive: [0; PHASES.len()], inclusive: [0; PHASES.len()] }) };
    }

    pub struct Scope;

    pub fn scope(phase: Phase) -> Scope {
        STATE.with(|state| state.borrow_mut().stack.push(Frame { phase: phase as usize, start: Instant::now(), child: 0 }));
        Scope
    }

    impl Drop for Scope {
        fn drop(&mut self) {
            STATE.with(|state| {
                let mut state = state.borrow_mut();
                if let Some(frame) = state.stack.pop() {
                    let elapsed = frame.start.elapsed().as_nanos() as u64;
                    state.calls[frame.phase] += 1;
                    state.inclusive[frame.phase] += elapsed;
                    state.exclusive[frame.phase] += elapsed.saturating_sub(frame.child);
                    if let Some(parent) = state.stack.last_mut() {
                        parent.child += elapsed;
                    }
                }
            })
        }
    }

    pub fn take() -> Vec<Row> {
        STATE.with(|state| {
            let mut state = state.borrow_mut();
            let rows = PHASES.iter().enumerate().map(|(index, (_, name))| (*name, state.calls[index], state.exclusive[index], state.inclusive[index])).collect();
            state.calls = [0; PHASES.len()];
            state.exclusive = [0; PHASES.len()];
            state.inclusive = [0; PHASES.len()];
            rows
        })
    }
}

#[cfg(not(feature = "profile"))]
mod off {
    use super::{Phase, Row};

    pub struct Scope;

    #[inline(always)]
    pub fn scope(_: Phase) -> Scope {
        Scope
    }

    pub fn take() -> Vec<Row> {
        Vec::new()
    }
}

#[cfg(feature = "profile")]
pub use on::{scope, take, Scope};
#[cfg(not(feature = "profile"))]
pub use off::{scope, take, Scope};
