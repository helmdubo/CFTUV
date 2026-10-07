//! The motorcycle graph (`wavefront/motorcycle.py`): one trace per reflex vertex of the input, marched cell by cell along the bisector until it crashes into
//! a wall or into another trace, and the two-sided index that turns traces into split candidates (Huber's theorem 2.11 as a filter).
//!
//! The order of the exact questions is the cost, and it is the oracle's, operation by operation: the prime universe of the speeds first, then per reflex
//! vertex four radicals (the bisector velocity), one doubling search for the step, and per march step the box, the cells, the walls of the cells in
//! ascending order (`concurrency_time`, `compare_times`, `event_point`, the projection signs) and the reach test. The crash queue is CPython's `heapq`
//! over `compare_times` (the comparison sequence is cost, `heap.rs`); the pairs of traces that share a cell are made lazily, each pair's meeting times
//! computed between the pushes of the pair before it, exactly as the oracle's generator interleaves them.
//!
//! Everything is exact: no float anywhere. The grid is a filter (`grid.rs`); a box or a margin beyond the machine range of the grid is a named
//! `Unsupported`, never a wrap.

use std::collections::HashMap;
use std::rc::Rc;

use cftuv_canon::QValue;
use cftuv_core::exact::{self, ExactCtx, UniverseStore};
use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::{SkelError, SkelResult};
use crate::grid::{ceil_of, ceil_ratio, floor_ratio, narrow_to_grid, Cell, CellGrid, CellIndex};
use crate::heap::{heappop, heappush};
use crate::line::SupportLine;
use crate::polygon::{Point, Polygon};
use crate::profile::{timed, Phase};
use crate::time::{compare_times, concurrency_time, event_point, EventPoint, EventTime, PointRef, TimeOutcome, TimeRef};

/// `ENCLOSURE_BITS`.
pub const ENCLOSURE_BITS: usize = 64;
/// `UPPER_BOUND_DOUBLINGS`: how often the enclosure of a divisor is refined before a rational upper bound of a time is given up.
pub const UPPER_BOUND_DOUBLINGS: usize = 12;
/// The first identity `trace_for` gives (`1_000_000 + next(counter)`).
pub const BORN_TRACE_BASE: i64 = 1_000_000;

/// `CrashKind`: what a trace ended in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CrashKind {
    None,
    Wall,
    Trace,
    Simultaneous,
}

impl CrashKind {
    pub fn value(self) -> &'static str {
        match self {
            CrashKind::None => "NONE",
            CrashKind::Wall => "WALL",
            CrashKind::Trace => "TRACE",
            CrashKind::Simultaneous => "SIMULTANEOUS",
        }
    }
}

/// `TraceOutcome`: why a trace is not bounded (there is no silent return).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TraceOutcome {
    Exact,
    HasNoBisector,
    NeverMeetsAWall,
    MarchBudgetExhausted,
}

impl TraceOutcome {
    pub fn value(self) -> &'static str {
        match self {
            TraceOutcome::Exact => "EXACT",
            TraceOutcome::HasNoBisector => "MOTORCYCLE_HAS_NO_BISECTOR",
            TraceOutcome::NeverMeetsAWall => "MOTORCYCLE_NEVER_MEETS_A_WALL",
            TraceOutcome::MarchBudgetExhausted => "MOTORCYCLE_MARCH_BUDGET_EXHAUSTED",
        }
    }
}

/// `WallV1`: an input edge as a line that does not move (`q = 0`) plus its ends.
#[derive(Debug, Clone)]
pub struct Wall {
    pub ident: i64,
    pub start: Point,
    pub end: Point,
    pub line: SupportLine,
}

/// `walls_of(polygon)`: in the order the builder makes its edges.
pub fn walls_of(polygon: &Polygon) -> SkelResult<Vec<Wall>> {
    let mut walls = Vec::with_capacity(polygon.vertex_count());
    for each in &polygon.loops {
        let size = each.points.len();
        for index in 0..size {
            let (start, end) = (each.points[index], each.points[(index + 1) % size]);
            let moving = SupportLine::through(start, end, 0)?;
            walls.push(Wall { ident: walls.len() as i64, start, end, line: SupportLine::new(moving.a, moving.b, moving.c, Rat::zero(), 0)? });
        }
    }
    Ok(walls)
}

/// `bisector_velocity(left, right)`: `dp/dt` of the vertex between two moving lines, written out without a division by a `SqrtSum`. `None` for parallel lines.
pub fn bisector_velocity(ctx: &mut ExactCtx<'_>, left: &SupportLine, right: &SupportLine) -> SkelResult<Option<(SqrtSum, SqrtSum)>> {
    let determinant = i128::from(left.a) * i128::from(right.b) - i128::from(right.a) * i128::from(left.b);
    if determinant == 0 {
        return Ok(None);
    }
    let scale = Rat::new(IBig::ONE, IBig::from(determinant)).map_err(|_| exact_internal("a zero determinant"))?;
    let coefficient = |value: i64| Rat::from_i64(value);
    let first = exact::radical(ctx, &coefficient(right.b), &left.q)?;
    let second = exact::radical(ctx, &coefficient(left.b), &right.q)?;
    let x = first.sub(&second).scaled(&scale);
    let third = exact::radical(ctx, &coefficient(left.a), &right.q)?;
    let fourth = exact::radical(ctx, &coefficient(right.a), &left.q)?;
    let y = third.sub(&fourth).scaled(&scale);
    Ok(Some((x, y)))
}

fn exact_internal(what: &'static str) -> SkelError {
    SkelError::Exact(exact::ExactError::Internal(what))
}

/// The integer extent of one point: the floor of the least and the ceiling of the greatest of its two enclosures, on each axis (`floor` and `ceil` are monotone, so the
/// floor of the least of several lows is the least of their floors: the box of several points is the merge of their extents).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Extent {
    x_floor: i64,
    x_ceil: i64,
    y_floor: i64,
    y_ceil: i64,
}

impl Extent {
    /// `(x_low, x_high, y_low, y_high)` of the box of two extents.
    pub fn merged(self, other: Extent) -> (i64, i64, i64, i64) {
        (self.x_floor.min(other.x_floor), self.x_ceil.max(other.x_ceil), self.y_floor.min(other.y_floor), self.y_ceil.max(other.y_ceil))
    }
}

/// The extent of a point from the enclosures of its coordinates at [`ENCLOSURE_BITS`], taken as unreduced ratios (no gcd: only the floor and the ceiling are asked).
pub fn extent_of(point: &EventPoint) -> SkelResult<Extent> {
    let (x_low, x_high, x_denominator) = point.x.enclosure_parts(ENCLOSURE_BITS);
    let (y_low, y_high, y_denominator) = point.y.enclosure_parts(ENCLOSURE_BITS);
    Ok(Extent {
        x_floor: narrow_to_grid(&floor_ratio(&x_low, &x_denominator), "a box edge")?,
        x_ceil: narrow_to_grid(&ceil_ratio(&x_high, &x_denominator), "a box edge")?,
        y_floor: narrow_to_grid(&floor_ratio(&y_low, &y_denominator), "a box edge")?,
        y_ceil: narrow_to_grid(&ceil_ratio(&y_high, &y_denominator), "a box edge")?,
    })
}

/// `_point_box(points)`: the integer box `(x_low, x_high, y_low, y_high)` of the enclosures, rounded OUTWARD (floor of the least, ceiling of the greatest).
pub fn point_box(points: &[&EventPoint]) -> SkelResult<(i64, i64, i64, i64)> {
    let mut found: Option<(i64, i64, i64, i64)> = None;
    for point in points {
        let extent = extent_of(point)?;
        found = Some(match found {
            None => (extent.x_floor, extent.x_ceil, extent.y_floor, extent.y_ceil),
            Some((x_low, x_high, y_low, y_high)) => (x_low.min(extent.x_floor), x_high.max(extent.x_ceil), y_low.min(extent.y_floor), y_high.max(extent.y_ceil)),
        });
    }
    found.ok_or_else(|| SkelError::Value("min() arg is an empty sequence".to_string()))
}

/// `_upper_bound_of_root(value)`: the power of two `s` with `s >= sqrt(value)` proven by the enclosure.
pub fn upper_bound_of_root(value: &SqrtSum) -> IBig {
    let (_low, high, denominator) = value.enclosure_parts(ENCLOSURE_BITS);
    let denominator = IBig::from(denominator);
    let mut bound = IBig::ONE;
    while &bound * &bound * &denominator < high {
        bound *= IBig::from(2);
    }
    bound
}

/// `_upper_bound_of_time(time)`: a rational upper bound of a non-negative time, or `None` (a negative dividend, or no positive lower bound of the divisor after
/// [`UPPER_BOUND_DOUBLINGS`] refinements).
pub fn upper_bound_of_time(time: &EventTime) -> SkelResult<Option<Rat>> {
    if time.dividend.signum() < 0 {
        return Ok(None);
    }
    let mut bits = ENCLOSURE_BITS;
    for _ in 0..UPPER_BOUND_DOUBLINGS {
        let (low, _high) = time.divisor.enclosure(bits);
        if low.signum() > 0 {
            return Ok(Some(time.dividend.div(&low).map_err(|_| exact_internal("a zero lower bound"))?));
        }
        bits *= 2;
    }
    Ok(None)
}

/// `march_budget(grid)`: the declared bound of the number of march steps.
pub fn march_budget(grid: &CellGrid) -> i64 {
    2 * (grid.columns() + grid.rows()) + 8
}

fn rat_of(value: i128) -> Rat {
    Rat::from_int(IBig::from(value))
}

/// `_projection_is_inside(point, wall)`: the point projects into the wall's segment (two exact signs, the second asked only when the first did not reject).
pub fn projection_is_inside(ctx: &mut ExactCtx<'_>, point: &EventPoint, wall: &Wall) -> SkelResult<bool> {
    let dx = i128::from(wall.end.0) - i128::from(wall.start.0);
    let dy = i128::from(wall.end.1) - i128::from(wall.start.1);
    let here = point.x.scaled(&rat_of(dx)).add(&point.y.scaled(&rat_of(dy)));
    let low = dx * i128::from(wall.start.0) + dy * i128::from(wall.start.1);
    let high = dx * i128::from(wall.end.0) + dy * i128::from(wall.end.1);
    if exact::sign(ctx, &here.sub(&SqrtSum::rational(&rat_of(low))), SIGN_FILTER_BITS)? < 0 {
        return Ok(false);
    }
    Ok(exact::sign(ctx, &SqrtSum::rational(&rat_of(high)).sub(&here), SIGN_FILTER_BITS)? >= 0)
}

/// `_reaches(origin, velocity, point, offset)`: the point is not farther than the ray gets in `offset` (squares, no roots).
pub fn reaches(ctx: &mut ExactCtx<'_>, origin: &EventPoint, velocity: &(SqrtSum, SqrtSum), point: &EventPoint, offset: &Rat) -> SkelResult<bool> {
    let dx = point.x.sub(&origin.x);
    let dy = point.y.sub(&origin.y);
    let travelled = dx.mul(&dx, ctx.products).add(&dy.mul(&dy, ctx.products));
    let speed = velocity.0.mul(&velocity.0, ctx.products).add(&velocity.1.mul(&velocity.1, ctx.products));
    let reach = speed.scaled(&offset.mul(offset));
    Ok(exact::sign(ctx, &reach.sub(&travelled), SIGN_FILTER_BITS)? >= 0)
}

/// `TraceV1`: a trace from its start to its crash. Nothing is evaluated, everything exact.
#[derive(Debug, Clone)]
pub struct Trace {
    pub ident: i64,
    pub outcome: TraceOutcome,
    pub left_line: SupportLine,
    pub right_line: SupportLine,
    pub start_time: TimeRef,
    pub origin: PointRef,
    pub velocity: (SqrtSum, SqrtSum),
    pub crash_time: Option<TimeRef>,
    pub crash_point: Option<PointRef>,
    pub crash_kind: CrashKind,
    pub crash_target: i64,
    pub reach: Option<Rat>,
}

impl Trace {
    /// `bounds_time(time)`: the event is not later than the crash (theorem 2.11 as a predicate). A trace without a crash bounds nothing and says so with `false`.
    pub fn bounds_time(&self, ctx: &mut ExactCtx<'_>, time: &EventTime) -> SkelResult<bool> {
        match &self.crash_time {
            None => Ok(false),
            Some(crash) => Ok(compare_times(ctx, time, crash)? <= 0),
        }
    }

    /// `box()`: the integer box of the trace (origin and crash point), rounded outward; `None` without a crash point.
    pub fn bounding_box(&self) -> SkelResult<Option<(i64, i64, i64, i64)>> {
        match &self.crash_point {
            None => Ok(None),
            Some(crash) => Ok(Some(point_box(&[&self.origin, crash])?)),
        }
    }
}

/// The eight counters of the graph, in the order of the oracle's dictionary.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GraphCounters {
    pub traces: u64,
    pub wall_tests: u64,
    pub march_steps: u64,
    pub wall_crashes: u64,
    pub trace_pairs: u64,
    pub trace_crashes: u64,
    pub unbounded_traces: u64,
    pub resting_steiner_vertices: u64,
}

impl GraphCounters {
    pub const NAMES: [&'static str; 8] = [
        "motorcycle_traces",
        "motorcycle_wall_tests",
        "motorcycle_march_steps",
        "motorcycle_wall_crashes",
        "motorcycle_trace_pairs",
        "motorcycle_trace_crashes",
        "motorcycle_unbounded_traces",
        "resting_steiner_vertices",
    ];

    pub fn as_array(&self) -> [u64; 8] {
        [self.traces, self.wall_tests, self.march_steps, self.wall_crashes, self.trace_pairs, self.trace_crashes, self.unbounded_traces, self.resting_steiner_vertices]
    }

    pub fn from_array(values: [u64; 8]) -> GraphCounters {
        let [traces, wall_tests, march_steps, wall_crashes, trace_pairs, trace_crashes, unbounded_traces, resting_steiner_vertices] = values;
        GraphCounters { traces, wall_tests, march_steps, wall_crashes, trace_pairs, trace_crashes, unbounded_traces, resting_steiner_vertices }
    }
}

/// `MotorcycleGraphV1`.
#[derive(Debug, Clone)]
pub struct MotorcycleGraph {
    pub walls: Vec<Wall>,
    pub grid: CellGrid,
    pub wall_index: CellIndex,
    /// The traces in the order the oracle's dictionary holds them (ascending identity: they are put in as the vertices are walked; a crash replaces in place).
    pub traces: Vec<Trace>,
    pub counters: GraphCounters,
    next_ident: u64,
    /// The number of march steps to use INSTEAD of [`march_budget`] of the grid. A host that must keep parity with an oracle whose `march_budget` was replaced (a test that
    /// forces the exhaustion of the march) asks the live function and passes its answer here; `None` is the declared bound.
    pub march_steps_override: Option<i64>,
}

/// `_wall_hit`'s answer: when, where, which wall.
pub type WallHit = (EventTime, EventPoint, i64);

impl MotorcycleGraph {
    /// A graph over the given parts (the seams restore the oracle's state with it); `next_ident` is how many traces `trace_for` has made.
    pub fn from_parts(walls: Vec<Wall>, grid: CellGrid, wall_index: CellIndex, traces: Vec<Trace>, counters: GraphCounters, next_ident: u64) -> MotorcycleGraph {
        MotorcycleGraph { walls, grid, wall_index, traces, counters, next_ident, march_steps_override: None }
    }

    pub fn next_ident(&self) -> u64 {
        self.next_ident
    }

    /// `traces.get(ident)`.
    pub fn trace(&self, ident: i64) -> Option<&Trace> {
        self.traces.binary_search_by_key(&ident, |trace| trace.ident).ok().map(|position| &self.traces[position])
    }

    /// `trace_for(left, right, start_time, origin)`: the trace of a vertex born during the count; it crashes only into walls (the weakest bound, true without any theorem).
    pub fn trace_for(&mut self, ctx: &mut ExactCtx<'_>, left: &SupportLine, right: &SupportLine, start_time: &TimeRef, origin: &PointRef) -> SkelResult<Trace> {
        let ident = BORN_TRACE_BASE + self.next_ident as i64;
        self.next_ident += 1;
        self.march(ctx, ident, left, right, start_time, origin)
    }

    #[allow(clippy::too_many_arguments)]
    fn blank(&self, ident: i64, outcome: TraceOutcome, left: &SupportLine, right: &SupportLine, start_time: &TimeRef, origin: &PointRef, velocity: (SqrtSum, SqrtSum)) -> Trace {
        Trace {
            ident,
            outcome,
            left_line: left.clone(),
            right_line: right.clone(),
            start_time: Rc::clone(start_time),
            origin: Rc::clone(origin),
            velocity,
            crash_time: None,
            crash_point: None,
            crash_kind: CrashKind::None,
            crash_target: -1,
            reach: None,
        }
    }

    /// `_march`: the velocity, the step (a cell divided by a power of two over the speed), then the walk.
    fn march(&mut self, ctx: &mut ExactCtx<'_>, ident: i64, left: &SupportLine, right: &SupportLine, start_time: &TimeRef, origin: &PointRef) -> SkelResult<Trace> {
        let Some(velocity) = timed(Phase::GraphVelocity, || bisector_velocity(ctx, left, right))? else {
            return Ok(self.blank(ident, TraceOutcome::HasNoBisector, left, right, start_time, origin, (SqrtSum::zero(), SqrtSum::zero())));
        };
        let speed_squared = velocity.0.mul(&velocity.0, ctx.products).add(&velocity.1.mul(&velocity.1, ctx.products));
        let step = Rat::new(IBig::from(self.grid.cell), upper_bound_of_root(&speed_squared)).map_err(|_| exact_internal("a zero step"))?;
        self.march_steps(ctx, ident, left, right, start_time, origin, velocity, &step)
    }

    #[allow(clippy::too_many_arguments)]
    fn march_steps(&mut self, ctx: &mut ExactCtx<'_>, ident: i64, left: &SupportLine, right: &SupportLine, start_time: &TimeRef, origin: &PointRef, velocity: (SqrtSum, SqrtSum), step: &Rat) -> SkelResult<Trace> {
        let budget = self.march_steps_override.unwrap_or_else(|| march_budget(&self.grid));
        let mut best: Option<WallHit> = None;
        let mut offset = Rat::zero();
        let mut near = timed(Phase::GraphExtent, || extent_of(&point_at(origin, &velocity, &offset)))?;
        for _ in 0..budget {
            let far = offset.add(step);
            let end = timed(Phase::GraphExtent, || extent_of(&point_at(origin, &velocity, &far)))?;
            let (x_low, x_high, y_low, y_high) = near.merged(end);
            near = end;
            let cells = timed(Phase::GraphCells, || self.grid.box_cells_int(x_low, x_high, y_low, y_high));
            if cells.is_empty() && best.is_none() && offset.signum() > 0 {
                return Ok(self.blank(ident, TraceOutcome::NeverMeetsAWall, left, right, start_time, origin, velocity));
            }
            for wall_ident in timed(Phase::GraphCells, || self.wall_index.lookup(&cells)) {
                self.counters.wall_tests += 1;
                let hit = wall_hit(ctx, left, right, start_time, &self.walls[wall_ident as usize])?;
                if let Some(hit) = hit {
                    let earlier = match &best {
                        None => true,
                        Some(found) => compare_times(ctx, &hit.0, &found.0)? < 0,
                    };
                    if earlier {
                        best = Some(hit);
                    }
                }
            }
            self.counters.march_steps += 1;
            let reached = match &best {
                Some(found) => timed(Phase::GraphReach, || reaches(ctx, origin, &velocity, &found.1, &far))?,
                None => false,
            };
            if reached {
                let (time, point, wall_ident) = best.take().expect("a reached hit was found");
                let reach = upper_bound_of_time(&time)?;
                return Ok(Trace {
                    ident,
                    outcome: TraceOutcome::Exact,
                    left_line: left.clone(),
                    right_line: right.clone(),
                    start_time: Rc::clone(start_time),
                    origin: Rc::clone(origin),
                    velocity,
                    crash_time: Some(Rc::new(time)),
                    crash_point: Some(Rc::new(point)),
                    crash_kind: CrashKind::Wall,
                    crash_target: wall_ident,
                    reach,
                });
            }
            offset = far;
        }
        Ok(self.blank(ident, TraceOutcome::MarchBudgetExhausted, left, right, start_time, origin, velocity))
    }
}

/// The point of the ray at an offset: `origin + velocity * offset` (the stretch `_segment_box` boxes is the one between the points of two offsets).
fn point_at(origin: &EventPoint, velocity: &(SqrtSum, SqrtSum), offset: &Rat) -> EventPoint {
    EventPoint { x: origin.x.add(&velocity.0.scaled(offset)), y: origin.y.add(&velocity.1.scaled(offset)) }
}

/// `_wall_hit(left, right, start_time, wall)`: when and where the vertex between the two lines meets the wall's segment after the start, or nothing.
pub fn wall_hit(ctx: &mut ExactCtx<'_>, left: &SupportLine, right: &SupportLine, start_time: &EventTime, wall: &Wall) -> SkelResult<Option<WallHit>> {
    let (time, outcome) = concurrency_time(ctx, left, right, &wall.line)?;
    let time = match (outcome, time) {
        (TimeOutcome::Exact, Some(time)) => time,
        _ => return Ok(None),
    };
    if compare_times(ctx, &time, start_time)? <= 0 {
        return Ok(None);
    }
    let point = event_point(ctx, left, right, &time, None)?;
    if !timed(Phase::GraphProjection, || projection_is_inside(ctx, &point, wall))? {
        return Ok(None);
    }
    Ok(Some((time, point, wall.ident)))
}

/// One crash candidate of the queue: ordered by the exact time alone (a tie is `false`, the heap structure decides).
struct CrashEntry {
    time: TimeRef,
    kind: CrashKind,
    motorcycle: i64,
    target: i64,
    peer_time: Option<TimeRef>,
}

fn crash_less<'a, 'b>(ctx: &'a mut ExactCtx<'b>) -> impl FnMut(&CrashEntry, &CrashEntry) -> SkelResult<bool> + use<'a, 'b> {
    move |left, right| Ok(compare_times(ctx, &left.time, &right.time)? < 0)
}

/// `_arrival(rider, host)`: when `rider` crosses the carrier line of `host`'s trace, as a normalised time.
pub fn arrival(ctx: &mut ExactCtx<'_>, rider: &Trace, host: &Trace) -> SkelResult<Option<EventTime>> {
    let normal_x = &host.velocity.1;
    let normal_y = host.velocity.0.neg();
    let offset = normal_x.mul(&host.origin.x, ctx.products).add(&normal_y.mul(&host.origin.y, ctx.products));
    let divisor = normal_x.mul(&rider.velocity.0, ctx.products).add(&normal_y.mul(&rider.velocity.1, ctx.products));
    if divisor.is_zero() {
        return Ok(None);
    }
    let dividend = offset.sub(&normal_x.mul(&rider.origin.x, ctx.products).add(&normal_y.mul(&rider.origin.y, ctx.products)));
    if dividend.is_zero() {
        return Ok(None);
    }
    let quotient = exact::divided_by(ctx, &divisor, &dividend)?;
    Ok(Some(EventTime::normalized(ctx, Rat::one(), quotient)?))
}

/// `_meeting_times(first, second)`: the time each trace reaches the meeting of the two bisectors, both after the trace's own start.
pub fn meeting_times(ctx: &mut ExactCtx<'_>, first: &Trace, second: &Trace) -> SkelResult<Option<(EventTime, EventTime)>> {
    let at_first = arrival(ctx, first, second)?;
    let at_second = arrival(ctx, second, first)?;
    let (Some(at_first), Some(at_second)) = (at_first, at_second) else {
        return Ok(None);
    };
    if compare_times(ctx, &at_first, &first.start_time)? <= 0 || compare_times(ctx, &at_second, &second.start_time)? <= 0 {
        return Ok(None);
    }
    Ok(Some((at_first, at_second)))
}

/// `_as_sqrt_sum(time)`: the value of a time as a sum of roots, one division through the proven universe.
fn as_sqrt_sum(ctx: &mut ExactCtx<'_>, time: &EventTime, universe: &[UBig]) -> SkelResult<SqrtSum> {
    Ok(exact::divide_with_prime_universe(ctx, &SqrtSum::rational(&time.dividend), &time.divisor, universe)?)
}

/// `_apply_trace_crash(graph, entry)`: shortens the trace to the earlier crash; `false` when the entry is not earlier than what the trace already has.
fn apply_trace_crash(ctx: &mut ExactCtx<'_>, graph: &mut MotorcycleGraph, entry: &CrashEntry, universe: &[UBig]) -> SkelResult<bool> {
    let position = graph.traces.binary_search_by_key(&entry.motorcycle, |trace| trace.ident).map_err(|_| SkelError::Unsupported(format!("the crash names the motorcycle {} that has no trace", entry.motorcycle)))?;
    let trace = graph.traces[position].clone();
    let current = trace.crash_time.as_ref().ok_or_else(|| SkelError::Unsupported("a live trace without a crash time".to_string()))?;
    if compare_times(ctx, &entry.time, current)? >= 0 {
        return Ok(false);
    }
    // TWO divisions, not one shared: the oracle's `_as_sqrt_sum` is a pure function whose price is counted (a frozen counter caught a hoisting once)
    let along_x = as_sqrt_sum(ctx, &entry.time, universe)?;
    let x = trace.origin.x.add(&trace.velocity.0.mul(&along_x, ctx.products));
    let along_y = as_sqrt_sum(ctx, &entry.time, universe)?;
    let y = trace.origin.y.add(&trace.velocity.1.mul(&along_y, ctx.products));
    let reach = upper_bound_of_time(&entry.time)?;
    graph.traces[position] = Trace {
        crash_time: Some(Rc::clone(&entry.time)),
        crash_point: Some(Rc::new(EventPoint { x, y })),
        crash_kind: entry.kind,
        crash_target: entry.target,
        reach,
        outcome: TraceOutcome::Exact,
        ..trace
    };
    graph.counters.trace_crashes += 1;
    graph.counters.resting_steiner_vertices += 1;
    Ok(true)
}

/// `_drain_crashes`: the queue only shrinks; a cancelled candidate (the neighbour crashed before the meeting) is dropped.
fn drain_crashes(ctx: &mut ExactCtx<'_>, graph: &mut MotorcycleGraph, heap: &mut Vec<CrashEntry>, universe: &[UBig]) -> SkelResult<()> {
    let mut settled: HashMap<i64, TimeRef> = HashMap::new();
    loop {
        let popped = heappop(heap, &mut crash_less(ctx))?;
        let Some(entry) = popped else {
            break;
        };
        if settled.contains_key(&entry.motorcycle) {
            continue;
        }
        if entry.kind == CrashKind::Wall {
            settled.insert(entry.motorcycle, Rc::clone(&entry.time));
            continue;
        }
        if let Some(peer) = settled.get(&entry.target) {
            let peer_time = entry.peer_time.as_ref().ok_or_else(|| SkelError::Unsupported("a trace crash entry without the peer's time".to_string()))?;
            if compare_times(ctx, peer, peer_time)? < 0 {
                continue;
            }
        }
        if apply_trace_crash(ctx, graph, &entry, universe)? {
            settled.insert(entry.motorcycle, Rc::clone(&entry.time));
        }
    }
    Ok(())
}

/// `_resolve_trace_crashes(graph, prime_universe)`: the crashes into other traces, in the order of rising time (an entry that leaves the queue first is final).
fn resolve_trace_crashes(ctx: &mut ExactCtx<'_>, graph: &mut MotorcycleGraph, universe: &[UBig]) -> SkelResult<()> {
    let live: Vec<Trace> = graph.traces.iter().filter(|trace| trace.outcome == TraceOutcome::Exact).cloned().collect();
    if live.len() < 2 {
        return Ok(());
    }
    let mut heap: Vec<CrashEntry> = Vec::new();
    for trace in &live {
        let time = trace.crash_time.as_ref().ok_or_else(|| SkelError::Unsupported("a live trace without a crash time".to_string()))?;
        heappush(&mut heap, CrashEntry { time: Rc::clone(time), kind: CrashKind::Wall, motorcycle: trace.ident, target: trace.crash_target, peer_time: None }, &mut crash_less(ctx))?;
    }
    // the pairs: traces whose boxes share a cell, made lazily (the meeting times of a pair are computed between the pushes of the pair before it)
    let mut index = CellIndex::new();
    let mut boxes: Vec<(i64, (i64, i64, i64, i64))> = Vec::new();
    for trace in &live {
        if let Some(found) = trace.bounding_box()? {
            boxes.push((trace.ident, found));
            index.add(trace.ident, &graph.grid.box_cells_int(found.0, found.1, found.2, found.3));
        }
    }
    boxes.sort_by_key(|(ident, _)| *ident);
    let position: HashMap<i64, usize> = live.iter().enumerate().map(|(slot, trace)| (trace.ident, slot)).collect();
    let find = |ident: i64| &live[position[&ident]];
    let mut seen: std::collections::HashSet<(i64, i64)> = std::collections::HashSet::new();
    for (ident, found) in &boxes {
        for other in index.lookup(&graph.grid.box_cells_int(found.0, found.1, found.2, found.3)) {
            if other <= *ident || !seen.insert((*ident, other)) {
                continue;
            }
            let Some((at_first, at_second)) = meeting_times(ctx, find(*ident), find(other))? else {
                continue;
            };
            graph.counters.trace_pairs += 1;
            let order = compare_times(ctx, &at_first, &at_second)?;
            let kind = if order == 0 { CrashKind::Simultaneous } else { CrashKind::Trace };
            let (at_first, at_second) = (Rc::new(at_first), Rc::new(at_second));
            if order >= 0 {
                heappush(&mut heap, CrashEntry { time: Rc::clone(&at_first), kind, motorcycle: *ident, target: other, peer_time: Some(Rc::clone(&at_second)) }, &mut crash_less(ctx))?;
            }
            if order <= 0 {
                heappush(&mut heap, CrashEntry { time: at_second, kind, motorcycle: other, target: *ident, peer_time: Some(at_first) }, &mut crash_less(ctx))?;
            }
        }
    }
    drain_crashes(ctx, graph, &mut heap, universe)
}

/// `_seed_traces(polygon, walls, graph)`: a motorcycle for every reflex vertex of the input, starting at time zero.
fn seed_traces(ctx: &mut ExactCtx<'_>, polygon: &Polygon, graph: &mut MotorcycleGraph) -> SkelResult<()> {
    let zero = Rc::new(EventTime::zero());
    let mut vertex: i64 = 0;
    for each in &polygon.loops {
        let size = each.points.len();
        let reflex = each.reflex_flags();
        for (index, _) in reflex.iter().enumerate().filter(|(_, flag)| **flag) {
            // the speeds of the neighbouring edges come from the loop: the bisector of a reflex vertex between a wall and a source runs ALONG the wall
            let before = (index + size - 1) % size;
            let left = SupportLine::with_speed(each.points[before], each.points[index], each.speeds[before].clone(), 0)?;
            let right = SupportLine::with_speed(each.points[index], each.points[(index + 1) % size], each.speeds[index].clone(), 0)?;
            let origin = Rc::new(EventPoint { x: SqrtSum::rational(&Rat::from_i64(each.points[index].0)), y: SqrtSum::rational(&Rat::from_i64(each.points[index].1)) });
            let trace = graph.march(ctx, vertex + index as i64, &left, &right, &zero, &origin)?;
            let exact = trace.outcome == TraceOutcome::Exact;
            graph.traces.push(trace);
            graph.counters.traces += 1;
            if exact {
                graph.counters.wall_crashes += 1;
            } else {
                graph.counters.unbounded_traces += 1;
            }
        }
        vertex += size as i64;
    }
    Ok(())
}

/// `build_motorcycle_graph(polygon, budget)`: the traces of every reflex vertex of the input, with their crashes into walls and into each other.
pub fn build_motorcycle_graph(ctx: &mut ExactCtx<'_>, polygon: &Polygon) -> SkelResult<MotorcycleGraph> {
    build_motorcycle_graph_with(ctx, polygon, None)
}

/// [`build_motorcycle_graph`] with the number of march steps given by the host (see [`MotorcycleGraph::march_steps_override`]).
pub fn build_motorcycle_graph_with(ctx: &mut ExactCtx<'_>, polygon: &Polygon, march_steps_override: Option<i64>) -> SkelResult<MotorcycleGraph> {
    let universe = speed_universe(ctx, polygon)?;
    let walls = walls_of(polygon)?;
    let (x_min, y_min, x_max, y_max) = polygon.bounding_box().ok_or_else(|| SkelError::Value("min() arg is an empty sequence".to_string()))?;
    let grid = CellGrid::covering((x_min, y_min, x_max, y_max), (walls.len() as i64).max(4))?;
    let mut wall_index = CellIndex::new();
    for wall in &walls {
        wall_index.add(wall.ident, &grid.segment_cells(wall.start, wall.end));
    }
    let mut graph = MotorcycleGraph { walls, grid, wall_index, traces: Vec::new(), counters: GraphCounters::default(), next_ident: 0, march_steps_override };
    seed_traces(ctx, polygon, &mut graph)?;
    timed(Phase::GraphCrashes, || resolve_trace_crashes(ctx, &mut graph, &universe))?;
    Ok(graph)
}

/// `_prime_universe_from_q_values(edge speeds + fan speeds, budget)`: the primes of odd power of every primitive speed (the factorization is the cost).
pub fn speed_universe(ctx: &mut ExactCtx<'_>, polygon: &Polygon) -> SkelResult<Vec<UBig>> {
    let mut speeds: Vec<QValue> = polygon.edges().into_iter().map(|(_, _, speed)| QValue { numerator: speed.numerator().clone(), denominator: speed.denominator().clone() }).collect();
    for (_, _, line) in polygon.fan_edges()? {
        speeds.push(QValue { numerator: line.q.numerator().clone(), denominator: line.q.denominator().clone() });
    }
    Ok(exact::prime_universe(ctx, &speeds, UniverseStore::Absent, true)?.0)
}

/// `speed_bound_of(polygon)`: the integer `s` with `s >= sqrt(q / |n|^2)` proven for every edge, fan support included (exactly 1 when every speed is the unit one).
pub fn speed_bound_of(polygon: &Polygon) -> SkelResult<i64> {
    let mut bound = IBig::ONE;
    let mut raise = |normal_squared: i128, speed: &Rat| {
        while Rat::from_int(&bound * &bound * IBig::from(normal_squared)) < *speed {
            bound = &bound * IBig::from(2);
        }
    };
    for (start, end, speed) in polygon.edges() {
        let (dx, dy) = (i128::from(end.0) - i128::from(start.0), i128::from(end.1) - i128::from(start.1));
        raise(dx * dx + dy * dy, speed);
    }
    for (_, _, line) in polygon.fan_edges()? {
        raise(line.normal_squared(), &line.q);
    }
    i64::try_from(&bound).map_err(|_| SkelError::Unsupported("the speed bound of the polygon is beyond the machine range".to_string()))
}

/// `(cell, ...)` lists of an index keyed by identity, in the order the oracle's dictionary keeps them (a repeated key replaces in place).
#[derive(Debug, Clone, Default)]
struct CellsByKey {
    slots: HashMap<i64, usize>,
    entries: Vec<(i64, Vec<Cell>)>,
}

impl CellsByKey {
    fn insert(&mut self, key: i64, cells: Vec<Cell>) {
        match self.slots.get(&key) {
            Some(slot) => self.entries[*slot].1 = cells,
            None => {
                self.slots.insert(key, self.entries.len());
                self.entries.push((key, cells));
            }
        }
    }

    fn get(&self, key: i64) -> Option<&[Cell]> {
        self.slots.get(&key).map(|slot| self.entries[*slot].1.as_slice())
    }
}

/// `TraceCandidateIndexV1`: which vertex may split which line, symmetric by construction (a pair is a candidate exactly when the vertex's trace box, widened
/// by what the trace can reach, shares a cell with the line).
#[derive(Debug, Clone)]
pub struct TraceCandidateIndex {
    pub grid: CellGrid,
    /// An integer upper bound of the Euclidean speed of any edge (one for unit speeds).
    pub speed_bound: i64,
    lines: CellIndex,
    vertices: CellIndex,
    line_cells: CellsByKey,
    vertex_cells: CellsByKey,
}

impl TraceCandidateIndex {
    /// `TraceCandidateIndexV1.covering(polygon, graph)`: the grid over the polygon widened by the longest reach (the nearest point of a carrier line to a trace
    /// may lie outside the polygon).
    pub fn covering(polygon: &Polygon, traces: &[Trace]) -> SkelResult<TraceCandidateIndex> {
        let (x_min, y_min, x_max, y_max) = polygon.bounding_box().ok_or_else(|| SkelError::Value("min() arg is an empty sequence".to_string()))?;
        let bound = speed_bound_of(polygon)?;
        let longest = traces.iter().filter_map(|trace| trace.reach.as_ref()).max();
        let margin = match longest {
            None => 0,
            Some(reach) => i128::from(narrow_to_grid(&ceil_of(&reach.mul(&Rat::from_i64(bound))), "the margin of the trace index")?),
        };
        let widened = |value: i64, shift: i128| -> SkelResult<i64> {
            i64::try_from(i128::from(value) + shift).map_err(|_| SkelError::Unsupported("the widened box of the trace index is beyond the machine range".to_string()))
        };
        let grid = CellGrid::covering((widened(x_min, -margin)?, widened(y_min, -margin)?, widened(x_max, margin)?, widened(y_max, margin)?), (polygon.vertex_count() as i64).max(4))?;
        Ok(TraceCandidateIndex { grid, speed_bound: bound, lines: CellIndex::new(), vertices: CellIndex::new(), line_cells: CellsByKey::default(), vertex_cells: CellsByKey::default() })
    }

    /// `register_line(key, line)`: once per key.
    pub fn register_line(&mut self, key: i64, line: &SupportLine) {
        if self.line_cells.get(key).is_some() {
            return;
        }
        let cells = self.grid.line_cells(i128::from(line.a), i128::from(line.b), line.c, None);
        self.lines.add(key, &cells);
        self.line_cells.insert(key, cells);
    }

    /// `register_trace(vertex, trace)`: `false` when the trace's box does not fit the area (the index then answers nothing for the vertex).
    pub fn register_trace(&mut self, vertex: i64, trace: &Trace) -> SkelResult<bool> {
        let Some((x_low, x_high, y_low, y_high)) = trace.bounding_box()? else {
            return Ok(false);
        };
        let Some(reach) = &trace.reach else {
            return Ok(false);
        };
        let margin = reach.mul(&Rat::from_i64(self.speed_bound));
        let (x_low, x_high) = (Rat::from_i64(x_low).sub(&margin), Rat::from_i64(x_high).add(&margin));
        let (y_low, y_high) = (Rat::from_i64(y_low).sub(&margin), Rat::from_i64(y_high).add(&margin));
        if !self.grid.contains_box(&x_low, &x_high, &y_low, &y_high) {
            return Ok(false);
        }
        let cells = self.grid.box_cells(&x_low, &x_high, &y_low, &y_high);
        if cells.is_empty() {
            return Ok(false);
        }
        self.vertices.add(vertex, &cells);
        self.vertex_cells.insert(vertex, cells);
        Ok(true)
    }

    /// `lines_near(vertex)`: the lines sharing a cell with the vertex (`KeyError` in the oracle for a vertex it does not know: a named refusal here).
    pub fn lines_near(&self, vertex: i64) -> SkelResult<Vec<i64>> {
        let cells = self.vertex_cells.get(vertex).ok_or_else(|| SkelError::Unsupported(format!("KeyError: the index knows no vertex {vertex}")))?;
        Ok(self.lines.lookup(cells))
    }

    /// `vertices_near(key)`: the vertices sharing a cell with the line (none for a line the index does not know).
    pub fn vertices_near(&self, key: i64) -> Vec<i64> {
        self.vertices.lookup(self.line_cells.get(key).unwrap_or(&[]))
    }

    pub fn knows_vertex(&self, vertex: i64) -> bool {
        self.vertex_cells.get(vertex).is_some()
    }

    pub fn knows_line(&self, key: i64) -> bool {
        self.line_cells.get(key).is_some()
    }

    /// `(key, cells)` of the registered lines, in registration order.
    pub fn line_cell_table(&self) -> &[(i64, Vec<Cell>)] {
        &self.line_cells.entries
    }

    /// `(vertex, cells)` of the registered vertices, in registration order.
    pub fn vertex_cell_table(&self) -> &[(i64, Vec<Cell>)] {
        &self.vertex_cells.entries
    }

    /// The buckets of the line index and of the vertex index (`(cell, identities)` in the order of the oracle's dictionaries).
    pub fn buckets(&self) -> (&[(Cell, Vec<i64>)], &[(Cell, Vec<i64>)]) {
        (self.lines.buckets(), self.vertices.buckets())
    }

    /// An index in the state the oracle's is in (the seams restore it): the tables of cells replayed in their registration order give the buckets.
    pub fn restore(grid: CellGrid, speed_bound: i64, line_cells: Vec<(i64, Vec<Cell>)>, vertex_cells: Vec<(i64, Vec<Cell>)>) -> TraceCandidateIndex {
        let mut index = TraceCandidateIndex { grid, speed_bound, lines: CellIndex::new(), vertices: CellIndex::new(), line_cells: CellsByKey::default(), vertex_cells: CellsByKey::default() };
        for (key, cells) in line_cells {
            index.lines.add(key, &cells);
            index.line_cells.insert(key, cells);
        }
        for (vertex, cells) in vertex_cells {
            index.vertices.add(vertex, &cells);
            index.vertex_cells.insert(vertex, cells);
        }
        index
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polygon::{unit_speed_squared, Loop};

    #[test]
    fn the_root_bound_is_a_power_of_two_above_the_root() {
        for (value, bound) in [(0, 1), (1, 1), (2, 2), (4, 2), (5, 4), (16, 4), (17, 8)] {
            let sum = SqrtSum::rational(&Rat::from_i64(value));
            assert_eq!(upper_bound_of_root(&sum), IBig::from(bound), "bound of sqrt({value})");
        }
    }

    #[test]
    fn a_time_has_an_upper_bound_only_when_it_is_not_negative() {
        let positive = EventTime::new(Rat::from_i64(3), SqrtSum::rational(&Rat::from_i64(2)));
        assert_eq!(upper_bound_of_time(&positive).unwrap(), Some(Rat::new(IBig::from(3), IBig::from(2)).unwrap()));
        let negative = EventTime::new(Rat::from_i64(-1), SqrtSum::rational(&Rat::one()));
        assert_eq!(upper_bound_of_time(&negative).unwrap(), None);
    }

    #[test]
    fn the_speed_bound_of_unit_speeds_is_one_and_a_fast_edge_raises_it() {
        let points = vec![(0, 0), (4, 0), (4, 4), (0, 4)];
        let unit: Vec<Rat> = (0..4).map(|index| unit_speed_squared(points[index], points[(index + 1) % 4])).collect();
        let polygon = Polygon::new(vec![Loop { points: points.clone(), speeds: unit.clone() }], Vec::new()).unwrap();
        assert_eq!(speed_bound_of(&polygon).unwrap(), 1);
        let mut fast = unit;
        fast[0] = Rat::from_i64(16 * 16 * 10);
        let polygon = Polygon::new(vec![Loop { points, speeds: fast }], Vec::new()).unwrap();
        assert_eq!(speed_bound_of(&polygon).unwrap(), 16);
    }

    #[test]
    fn counters_keep_the_order_of_the_oracles_dictionary() {
        let counters = GraphCounters::from_array([1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(counters.as_array(), [1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(GraphCounters::NAMES[7], "resting_steiner_vertices");
    }
}
