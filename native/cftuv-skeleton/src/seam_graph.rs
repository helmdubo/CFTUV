//! The differential seams of the second slice (test-only): the cell grid and the motorcycle graph (WP-S1), the edge law, the rest of the exact candidate
//! view, the poststate classification and the proof ledger (WP-S2b). Same wire as `seam.rs` (a request is `[header, opcode, [argument, ...]]`, the answer the
//! cost answer plus the nanoseconds of the seam and the extras); the opcodes run from 230.
//!
//! Wire shapes, besides those of `wire.rs`:
//!
//! ```text
//! polygon  [loops, fans]        loop [points x0 y0 x1 y1 ..., speeds]        fan [x, y, [[nx, ny, q], ...]]
//! trace    [ident, outcome, left line, right line, start time, origin point, [vx, vy], crash time | none, crash point | none, crash kind, target, reach | none]
//! graph    [walls, grid, buckets, traces, counters, next identity]
//!          wall [ident, sx, sy, ex, ey, line]    grid [x_min, y_min, x_max, y_max, cell]    bucket [column, row, [identity, ...]]
//! ```
//!
//! `BUILD_MOTORCYCLE_GRAPH` takes `[polygon]` or `[polygon, march steps | none]`, `TRACE_FOR` `[graph, left, right, start time, origin]` plus the same optional last argument: the
//! number of march steps the host read from the oracle's live `march_budget` (a test that replaces it forces the exhaustion of the march; the native side cannot see the patch).
//!
//! Outcome codes of a trace: 0 EXACT, 1 no bisector, 2 never meets a wall, 3 march budget exhausted; crash kinds 0 NONE, 1 WALL, 2 TRACE, 3 SIMULTANEOUS.

use std::rc::Rc;
use std::time::Instant;

use cftuv_core::codec::Value;
use cftuv_core::exact::ExactCtx;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::candidate::{evaluate_edge_candidate, CandidateRefusal, EdgeDecision};
use crate::error::{SkelError, SkelResult};
use crate::grid::{ceil_of, floor_of, Cell, CellGrid, CellIndex};
use crate::motorcycle::{
    arrival, bisector_velocity, build_motorcycle_graph_with, march_budget, meeting_times, point_box, projection_is_inside, reaches, speed_bound_of, upper_bound_of_root, upper_bound_of_time, wall_hit,
    walls_of, CrashKind, GraphCounters, MotorcycleGraph, Trace, TraceCandidateIndex, TraceOutcome, Wall,
};
use crate::polygon::{FanSupport, Loop, Polygon, VertexFan};
use crate::poststate::{classify_poststate_span, PoststateDisposition};
use crate::proof::{ProofBranch, ProofCause, ProofDisposition, ProofLedger, ProofObligation};
use crate::queue::EventKind;
use crate::seam::{effects_value, growth_value, memo_of, time_entry_value, view_of};
use crate::time::EventPoint;
use crate::view::{collapsing_span, edge_event_time, is_future, sliding_projection, span_end};
use crate::wire::{
    bad, fixed, flag_of, i64_of, int, int_of, line_answer, line_any_of, list, optional, point_of, point_value, rat_of, str_of, str_value, sum_of, sum_option_value, time_of, time_value, u32_of,
    SeamError, Wire,
};

/// `(opcode, name)` of the seams of this slice.
pub(crate) const SEAMS: &[(u8, &str)] = &[
    (230, "CELL_GRID"),
    (231, "BUILD_MOTORCYCLE_GRAPH"),
    (232, "TRACE_FOR"),
    (233, "TRACE_INDEX_SCRIPT"),
    (234, "MOTORCYCLE_PART"),
    (240, "EVALUATE_EDGE_CANDIDATE"),
    (241, "EDGE_EVENT_TIME"),
    (242, "IS_FUTURE"),
    (243, "COLLAPSING_SPAN"),
    (244, "SPAN_END"),
    (245, "SLIDING_PROJECTION"),
    (246, "CLASSIFY_POSTSTATE_SPAN"),
    (250, "PROOF_SCRIPT"),
];

fn unsupported_wire(error: SkelError) -> SeamError {
    SeamError(format!("unsupported: {error:?}"))
}

// --------------------------------------------------------------------------
// readers and writers
// --------------------------------------------------------------------------

fn ints_of(value: &Value, what: &str) -> Wire<Vec<i64>> {
    list(value, what)?.iter().map(|item| i64_of(item, what)).collect()
}

fn ints_value(values: &[i64]) -> Value {
    Value::List(values.iter().map(|value| int(*value)).collect())
}

fn keys_of(value: &Value, what: &str) -> Wire<Vec<Vec<i64>>> {
    list(value, what)?.iter().map(|key| ints_of(key, what)).collect()
}

fn keys_value(keys: &[Vec<i64>]) -> Value {
    Value::List(keys.iter().map(|key| ints_value(key)).collect())
}

fn point2_of(first: &Value, second: &Value, what: &str) -> Wire<(i64, i64)> {
    Ok((i64_of(first, what)?, i64_of(second, what)?))
}

pub(crate) fn polygon_of(value: &Value) -> Wire<Polygon> {
    let [loops, fans] = fixed::<2>(value, "a polygon")?;
    let loops = list(loops, "loops")?
        .iter()
        .map(|each| {
            let [points, speeds] = fixed::<2>(each, "a loop")?;
            let flat = ints_of(points, "loop points")?;
            if flat.len() % 2 != 0 {
                return Err(bad("loop points"));
            }
            let speeds = list(speeds, "loop speeds")?.iter().map(|speed| rat_of(speed, "a loop speed")).collect::<Wire<Vec<Rat>>>()?;
            Ok(Loop { points: flat.chunks(2).map(|pair| (pair[0], pair[1])).collect(), speeds })
        })
        .collect::<Wire<Vec<Loop>>>()?;
    let fans = list(fans, "fans")?
        .iter()
        .map(|each| {
            let [x, y, supports] = fixed::<3>(each, "a fan")?;
            let supports = list(supports, "fan supports")?
                .iter()
                .map(|support| {
                    let [nx, ny, q] = fixed::<3>(support, "a fan support")?;
                    Ok(FanSupport { normal: point2_of(nx, ny, "a fan normal")?, speed: rat_of(q, "a fan speed")? })
                })
                .collect::<Wire<Vec<FanSupport>>>()?;
            Ok(VertexFan { point: point2_of(x, y, "a fan vertex")?, supports })
        })
        .collect::<Wire<Vec<VertexFan>>>()?;
    Polygon::new(loops, fans).map_err(unsupported_wire)
}

fn outcome_code(outcome: TraceOutcome) -> u8 {
    match outcome {
        TraceOutcome::Exact => 0,
        TraceOutcome::HasNoBisector => 1,
        TraceOutcome::NeverMeetsAWall => 2,
        TraceOutcome::MarchBudgetExhausted => 3,
    }
}

fn kind_code(kind: CrashKind) -> u8 {
    match kind {
        CrashKind::None => 0,
        CrashKind::Wall => 1,
        CrashKind::Trace => 2,
        CrashKind::Simultaneous => 3,
    }
}

fn trace_value(trace: &Trace) -> Value {
    Value::List(vec![
        int(trace.ident),
        int(outcome_code(trace.outcome)),
        line_answer(&trace.left_line),
        line_answer(&trace.right_line),
        time_value(&trace.start_time),
        point_value(&trace.origin),
        Value::List(vec![Value::Sum(trace.velocity.0.clone()), Value::Sum(trace.velocity.1.clone())]),
        trace.crash_time.as_ref().map_or(Value::None, |time| time_value(time)),
        trace.crash_point.as_ref().map_or(Value::None, |point| point_value(point)),
        int(kind_code(trace.crash_kind)),
        int(trace.crash_target),
        trace.reach.as_ref().map_or(Value::None, |reach| Value::Frac(reach.clone())),
    ])
}

fn trace_of(value: &Value) -> Wire<Trace> {
    let [ident, outcome, left, right, start, origin, velocity, crash_time, crash_point, kind, target, reach] = fixed::<12>(value, "a trace")?;
    let [vx, vy] = fixed::<2>(velocity, "a velocity")?;
    Ok(Trace {
        ident: i64_of(ident, "a trace identity")?,
        outcome: match u32_of(outcome, "a trace outcome")? {
            0 => TraceOutcome::Exact,
            1 => TraceOutcome::HasNoBisector,
            2 => TraceOutcome::NeverMeetsAWall,
            3 => TraceOutcome::MarchBudgetExhausted,
            _ => return Err(bad("a trace outcome")),
        },
        left_line: line_any_of(left)?,
        right_line: line_any_of(right)?,
        start_time: Rc::new(time_of(start)?),
        origin: Rc::new(point_of(origin)?),
        velocity: (sum_of(vx, "a velocity x")?.clone(), sum_of(vy, "a velocity y")?.clone()),
        crash_time: optional(crash_time, time_of)?.map(Rc::new),
        crash_point: optional(crash_point, point_of)?.map(Rc::new),
        crash_kind: match u32_of(kind, "a crash kind")? {
            0 => CrashKind::None,
            1 => CrashKind::Wall,
            2 => CrashKind::Trace,
            3 => CrashKind::Simultaneous,
            _ => return Err(bad("a crash kind")),
        },
        crash_target: i64_of(target, "a crash target")?,
        reach: optional(reach, |found| rat_of(found, "a reach"))?,
    })
}

fn cell_value(cell: &Cell) -> Value {
    Value::List(vec![int(cell.0), int(cell.1)])
}

fn cells_value(cells: &[Cell]) -> Value {
    Value::List(cells.iter().map(cell_value).collect())
}

fn cells_of(value: &Value) -> Wire<Vec<Cell>> {
    list(value, "cells")?
        .iter()
        .map(|cell| {
            let [column, row] = fixed::<2>(cell, "a cell")?;
            point2_of(column, row, "a cell")
        })
        .collect()
}

fn grid_value(grid: &CellGrid) -> Value {
    ints_value(&[grid.x_min, grid.y_min, grid.x_max, grid.y_max, grid.cell])
}

fn grid_of(value: &Value) -> Wire<CellGrid> {
    let [x_min, y_min, x_max, y_max, cell] = fixed::<5>(value, "a grid")?;
    let grid = CellGrid { x_min: i64_of(x_min, "a grid limit")?, y_min: i64_of(y_min, "a grid limit")?, x_max: i64_of(x_max, "a grid limit")?, y_max: i64_of(y_max, "a grid limit")?, cell: i64_of(cell, "a grid cell")? };
    if grid.cell <= 0 {
        return Err(bad("a grid cell must be positive"));
    }
    Ok(grid)
}

fn buckets_value(index: &CellIndex) -> Value {
    Value::List(index.buckets().iter().map(|(cell, idents)| Value::List(vec![int(cell.0), int(cell.1), ints_value(idents)])).collect())
}

fn wall_value(wall: &Wall) -> Value {
    Value::List(vec![int(wall.ident), int(wall.start.0), int(wall.start.1), int(wall.end.0), int(wall.end.1), line_answer(&wall.line)])
}

fn wall_of(value: &Value) -> Wire<Wall> {
    let [ident, sx, sy, ex, ey, line] = fixed::<6>(value, "a wall")?;
    Ok(Wall { ident: i64_of(ident, "a wall identity")?, start: point2_of(sx, sy, "a wall start")?, end: point2_of(ex, ey, "a wall end")?, line: line_any_of(line)? })
}

fn counters_value(counters: &GraphCounters) -> Value {
    Value::List(counters.as_array().iter().map(|count| int(*count)).collect())
}

fn graph_value(graph: &MotorcycleGraph) -> Value {
    Value::List(vec![
        Value::List(graph.walls.iter().map(wall_value).collect()),
        grid_value(&graph.grid),
        buckets_value(&graph.wall_index),
        Value::List(graph.traces.iter().map(trace_value).collect()),
        counters_value(&graph.counters),
        int(graph.next_ident()),
    ])
}

fn graph_of(value: &Value) -> Wire<MotorcycleGraph> {
    let [walls, grid, buckets, traces, counters, next] = fixed::<6>(value, "a graph")?;
    let walls = list(walls, "walls")?.iter().map(wall_of).collect::<Wire<Vec<Wall>>>()?;
    let mut index = CellIndex::new();
    for bucket in list(buckets, "buckets")? {
        let [column, row, idents] = fixed::<3>(bucket, "a bucket")?;
        index.restore(point2_of(column, row, "a bucket cell")?, ints_of(idents, "bucket identities")?);
    }
    let values = list(counters, "counters")?.iter().map(|count| u64::try_from(&int_of(count, "a counter")?).map_err(|_| bad("a counter"))).collect::<Wire<Vec<u64>>>()?;
    let counters = GraphCounters::from_array(values.try_into().map_err(|_| bad("eight counters"))?);
    let traces = list(traces, "traces")?.iter().map(trace_of).collect::<Wire<Vec<Trace>>>()?;
    let next_ident = u64::try_from(&int_of(next, "the next identity")?).map_err(|_| bad("the next identity"))?;
    Ok(MotorcycleGraph::from_parts(walls, grid_of(grid)?, index, traces, counters, next_ident))
}

fn nanoseconds(started: Instant) -> Value {
    int(started.elapsed().as_nanos() as u64)
}

// --------------------------------------------------------------------------
// the cell grid
// --------------------------------------------------------------------------

fn window_of(value: &Value) -> Wire<Option<[Rat; 4]>> {
    optional(value, |found| {
        let [x_low, x_high, y_low, y_high] = fixed::<4>(found, "a window")?;
        Ok([rat_of(x_low, "a window edge")?, rat_of(x_high, "a window edge")?, rat_of(y_low, "a window edge")?, rat_of(y_high, "a window edge")?])
    })
}

/// `[0, x_min, y_min, x_max, y_max, targets]` covering; `[1, grid]` the four measures; `[2, grid, x, y]` the column and the row; `[3, grid, x_low, x_high, y_low, y_high, by_integers]`
/// the cells of a box (by the integer entry when `by_integers` and every edge is a whole number); `[4, grid, ...]` containment; `[5, grid, a, b, c, window]` the cells of a line;
/// `[6, grid, sx, sy, ex, ey]` of a segment; `[7, ops]` a cell index (`[0, ident, cells]` adds, `[1, cells]` looks up); `[8, value]` floor and ceiling.
fn cell_grid(args: &[Value]) -> Wire<SkelResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    Ok(match u32_of(at(0)?, "a grid operation")? {
        0 => {
            let bounds = (i64_of(at(1)?, "a limit")?, i64_of(at(2)?, "a limit")?, i64_of(at(3)?, "a limit")?, i64_of(at(4)?, "a limit")?);
            CellGrid::covering(bounds, i64_of(at(5)?, "targets")?).map(|grid| grid_value(&grid))
        }
        1 => {
            let grid = grid_of(at(1)?)?;
            Ok(ints_value(&[grid.columns(), grid.rows(), grid.x_limit(), grid.y_limit()]))
        }
        2 => {
            let grid = grid_of(at(1)?)?;
            Ok(Value::List(vec![Value::Int(grid.column(&rat_of(at(2)?, "x")?)), Value::Int(grid.row(&rat_of(at(3)?, "y")?))]))
        }
        3 => {
            let grid = grid_of(at(1)?)?;
            let edges = [at(2)?, at(3)?, at(4)?, at(5)?];
            let whole = flag_of(at(6)?, "the entry")? && edges.iter().all(|edge| matches!(edge, Value::Int(_)));
            if whole {
                let [x_low, x_high, y_low, y_high] = [i64_of(edges[0], "an edge")?, i64_of(edges[1], "an edge")?, i64_of(edges[2], "an edge")?, i64_of(edges[3], "an edge")?];
                Ok(cells_value(&grid.box_cells_int(x_low, x_high, y_low, y_high)))
            } else {
                Ok(cells_value(&grid.box_cells(&rat_of(edges[0], "an edge")?, &rat_of(edges[1], "an edge")?, &rat_of(edges[2], "an edge")?, &rat_of(edges[3], "an edge")?)))
            }
        }
        4 => {
            let grid = grid_of(at(1)?)?;
            Ok(Value::Bool(grid.contains_box(&rat_of(at(2)?, "an edge")?, &rat_of(at(3)?, "an edge")?, &rat_of(at(4)?, "an edge")?, &rat_of(at(5)?, "an edge")?)))
        }
        5 => {
            let grid = grid_of(at(1)?)?;
            let (a, b) = (i128::try_from(&int_of(at(2)?, "a")?).map_err(|_| bad("a"))?, i128::try_from(&int_of(at(3)?, "b")?).map_err(|_| bad("b"))?);
            let c = i128::try_from(&int_of(at(4)?, "c")?).map_err(|_| bad("c"))?;
            Ok(cells_value(&grid.line_cells(a, b, c, window_of(at(5)?)?)))
        }
        6 => {
            let grid = grid_of(at(1)?)?;
            Ok(cells_value(&grid.segment_cells(point2_of(at(2)?, at(3)?, "a start")?, point2_of(at(4)?, at(5)?, "an end")?)))
        }
        7 => {
            let mut index = CellIndex::new();
            let mut results = Vec::new();
            for op in list(at(1)?, "index operations")? {
                match list(op, "an index operation")? {
                    [code, ident, cells] if u32_of(code, "an index code")? == 0 => {
                        index.add(i64_of(ident, "an identity")?, &cells_of(cells)?);
                        results.push(Value::None);
                    }
                    [code, cells] if u32_of(code, "an index code")? == 1 => results.push(ints_value(&index.lookup(&cells_of(cells)?))),
                    _ => return Err(bad("an index operation")),
                }
            }
            Ok(Value::List(vec![Value::List(results), buckets_value(&index)]))
        }
        8 => {
            let value = rat_of(at(1)?, "a number")?;
            Ok(Value::List(vec![Value::Int(floor_of(&value)), Value::Int(ceil_of(&value))]))
        }
        _ => return Err(bad("a grid operation")),
    })
}

// --------------------------------------------------------------------------
// the motorcycle graph
// --------------------------------------------------------------------------

/// `[0, left, right]` the bisector velocity; `[1, sum]` the root bound; `[2, time]` the upper bound of a time; `[3, [point, ...]]` the box of points; `[4, polygon]` the speed bound;
/// `[5, polygon]` the walls; `[6, grid]` the march budget; `[7, point, wall]` the projection test; `[8, origin, [vx, vy], point, offset]` the reach test;
/// `[9, left, right, start time, wall]` a wall hit; `[10, rider, host]` an arrival; `[11, first, second]` the meeting times.
fn motorcycle_part(args: &[Value], ctx: &mut ExactCtx<'_>) -> Wire<SkelResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    let velocity_of = |value: &Value| -> Wire<(SqrtSum, SqrtSum)> {
        let [x, y] = fixed::<2>(value, "a velocity")?;
        Ok((sum_of(x, "a velocity x")?.clone(), sum_of(y, "a velocity y")?.clone()))
    };
    Ok(match u32_of(at(0)?, "a part")? {
        0 => bisector_velocity(ctx, &line_any_of(at(1)?)?, &line_any_of(at(2)?)?).map(|found| found.map_or(Value::None, |(x, y)| Value::List(vec![Value::Sum(x), Value::Sum(y)]))),
        1 => Ok(Value::Int(upper_bound_of_root(sum_of(at(1)?, "a sum")?))),
        2 => upper_bound_of_time(&time_of(at(1)?)?).map(|found| found.map_or(Value::None, Value::Frac)),
        3 => {
            let points = list(at(1)?, "points")?.iter().map(point_of).collect::<Wire<Vec<EventPoint>>>()?;
            let refs: Vec<&EventPoint> = points.iter().collect();
            point_box(&refs).map(|(a, b, c, d)| ints_value(&[a, b, c, d]))
        }
        4 => speed_bound_of(&polygon_of(at(1)?)?).map(int),
        5 => walls_of(&polygon_of(at(1)?)?).map(|walls| Value::List(walls.iter().map(wall_value).collect())),
        6 => Ok(int(march_budget(&grid_of(at(1)?)?))),
        7 => projection_is_inside(ctx, &point_of(at(1)?)?, &wall_of(at(2)?)?).map(Value::Bool),
        8 => reaches(ctx, &point_of(at(1)?)?, &velocity_of(at(2)?)?, &point_of(at(3)?)?, &rat_of(at(4)?, "an offset")?).map(Value::Bool),
        9 => wall_hit(ctx, &line_any_of(at(1)?)?, &line_any_of(at(2)?)?, &time_of(at(3)?)?, &wall_of(at(4)?)?)
            .map(|found| found.map_or(Value::None, |(time, point, ident)| Value::List(vec![time_value(&time), point_value(&point), int(ident)]))),
        10 => arrival(ctx, &trace_of(at(1)?)?, &trace_of(at(2)?)?).map(|found| found.map_or(Value::None, |time| time_value(&time))),
        11 => meeting_times(ctx, &trace_of(at(1)?)?, &trace_of(at(2)?)?).map(|found| found.map_or(Value::None, |(first, second)| Value::List(vec![time_value(&first), time_value(&second)]))),
        _ => return Err(bad("a part")),
    })
}

fn trace_index_script(args: &[Value]) -> Wire<SkelResult<Value>> {
    let polygon = polygon_of(args.first().ok_or_else(|| bad("too few arguments"))?)?;
    let traces = list(args.get(1).ok_or_else(|| bad("too few arguments"))?, "traces")?.iter().map(trace_of).collect::<Wire<Vec<Trace>>>()?;
    let mut index = match TraceCandidateIndex::covering(&polygon, &traces) {
        Ok(index) => index,
        Err(error) => return Ok(Err(error)),
    };
    let mut results = Vec::new();
    for op in list(args.get(2).ok_or_else(|| bad("too few arguments"))?, "index operations")? {
        let step: SkelResult<Value> = match list(op, "an index operation")? {
            [code, key, line] if u32_of(code, "an index code")? == 0 => {
                index.register_line(i64_of(key, "a key")?, &line_any_of(line)?);
                Ok(Value::None)
            }
            [code, vertex, trace] if u32_of(code, "an index code")? == 1 => index.register_trace(i64_of(vertex, "a vertex")?, &trace_of(trace)?).map(Value::Bool),
            [code, vertex] if u32_of(code, "an index code")? == 2 => Ok(match index.lines_near(i64_of(vertex, "a vertex")?) {
                Ok(lines) => Value::List(vec![int(0u8), ints_value(&lines)]),
                Err(_) => Value::List(vec![int(1u8)]),
            }),
            [code, key] if u32_of(code, "an index code")? == 3 => Ok(ints_value(&index.vertices_near(i64_of(key, "a key")?))),
            [code, vertex] if u32_of(code, "an index code")? == 4 => Ok(Value::Bool(index.knows_vertex(i64_of(vertex, "a vertex")?))),
            [code, key] if u32_of(code, "an index code")? == 5 => Ok(Value::Bool(index.knows_line(i64_of(key, "a key")?))),
            _ => return Err(bad("an index operation")),
        };
        match step {
            Ok(value) => results.push(value),
            Err(error) => return Ok(Err(error)),
        }
    }
    let table = |entries: &[(i64, Vec<Cell>)]| Value::List(entries.iter().map(|(key, cells)| Value::List(vec![int(*key), cells_value(cells)])).collect());
    Ok(Ok(Value::List(vec![Value::List(results), grid_value(&index.grid), int(index.speed_bound), table(index.line_cell_table()), table(index.vertex_cell_table())])))
}

// --------------------------------------------------------------------------
// the candidate laws
// --------------------------------------------------------------------------

fn edge_decision_value(decision: &EdgeDecision, before: (usize, usize), after: (usize, usize)) -> Value {
    let candidate = match &decision.candidate {
        None => Value::None,
        Some(candidate) => Value::List(vec![time_value(&candidate.time), point_value(&candidate.point), Value::Bool(candidate.span_unproven)]),
    };
    Value::List(vec![candidate, effects_value(&decision.effects), growth_value(before, after)])
}

fn disposition_index(disposition: PoststateDisposition) -> u8 {
    PoststateDisposition::ALL.iter().position(|found| *found == disposition).unwrap_or(0) as u8
}

// --------------------------------------------------------------------------
// the proof ledger
// --------------------------------------------------------------------------

fn obligation_value(obligation: &ProofObligation) -> Value {
    let (kind, cause) = match obligation.cause {
        ProofCause::Refusal(reason) => (0u8, reason.value()),
        ProofCause::Branch(branch) => (1u8, branch.value()),
    };
    Value::List(vec![
        int(kind),
        str_value(cause),
        str_value(obligation.disposition.value()),
        ints_value(&obligation.vertex_ids),
        keys_value(&obligation.participant_edge_keys),
        keys_value(&obligation.target_edge_keys),
        time_value(&obligation.level),
        obligation.event_kind.map_or(Value::None, |kind| str_value(kind.value())),
    ])
}

fn cause_of(kind: &Value, name: &Value) -> Wire<ProofCause> {
    let name = str_of(name, "a cause")?;
    match u32_of(kind, "a cause kind")? {
        0 => CandidateRefusal::from_value(&name).map(ProofCause::Refusal).ok_or_else(|| bad("a refusal name")),
        1 => ProofBranch::ALL.into_iter().find(|branch| branch.value() == name).map(ProofCause::Branch).ok_or_else(|| bad("a branch name")),
        _ => Err(bad("a cause kind")),
    }
}

/// `[0, cause kind, cause, disposition, vertices, participant keys, target keys, level, event kind | none]` records; `[1, reason, vertices, participant keys, target keys, level]`
/// records a refusal; `[2, dead]` discharges; `[3, dead]` finalizes (answers `[status, obligations]`). The seam's answer: the results, then the ledger at the end.
fn proof_script(args: &[Value]) -> Wire<SkelResult<Value>> {
    let mut ledger = ProofLedger::new();
    let mut results = Vec::new();
    for op in list(args.first().ok_or_else(|| bad("too few arguments"))?, "a proof script")? {
        let step: SkelResult<Value> = match list(op, "a proof operation")? {
            [code, kind, cause, disposition, vertices, participants, targets, level, event_kind] if u32_of(code, "a proof code")? == 0 => {
                let disposition = ProofDisposition::from_value(&str_of(disposition, "a disposition")?).ok_or_else(|| bad("a disposition"))?;
                let event_kind = optional(event_kind, |found| EventKind::from_value(&str_of(found, "an event kind")?).ok_or_else(|| bad("an event kind")))?;
                ledger
                    .record(cause_of(kind, cause)?, disposition, &ints_of(vertices, "vertices")?, &keys_of(participants, "keys")?, &keys_of(targets, "keys")?, &time_of(level)?, event_kind)
                    .map(|()| Value::None)
            }
            [code, reason, vertices, participants, targets, level] if u32_of(code, "a proof code")? == 1 => {
                let reason = CandidateRefusal::from_value(&str_of(reason, "a reason")?).ok_or_else(|| bad("a refusal name"))?;
                ledger.record_refusal(reason, &ints_of(vertices, "vertices")?, &keys_of(participants, "keys")?, &keys_of(targets, "keys")?, &time_of(level)?).map(|()| Value::None)
            }
            [code, dead] if u32_of(code, "a proof code")? == 2 => {
                ledger.discharge(&ints_of(dead, "dead vertices")?);
                Ok(Value::None)
            }
            [code, dead] if u32_of(code, "a proof code")? == 3 => {
                let (status, obligations) = ledger.finalize(&ints_of(dead, "dead vertices")?);
                Ok(Value::List(vec![str_value(status.value()), Value::List(obligations.iter().map(obligation_value).collect())]))
            }
            _ => return Err(bad("a proof operation")),
        };
        match step {
            Ok(value) => results.push(value),
            Err(error) => return Ok(Err(error)),
        }
    }
    Ok(Ok(Value::List(vec![Value::List(results), Value::List(ledger.obligations().iter().map(obligation_value).collect())])))
}

// --------------------------------------------------------------------------
// dispatch
// --------------------------------------------------------------------------

pub(crate) fn dispatch(code: u8, args: &[Value], ctx: &mut ExactCtx<'_>, extras: &mut Vec<Value>) -> Wire<SkelResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    let expect = |count: usize| if args.len() == count { Ok(()) } else { Err(bad("the argument count")) };
    Ok(match code {
        230 => return cell_grid(args),
        231 => {
            let polygon = polygon_of(at(0)?)?;
            let override_steps = args.get(1).map(|value| optional(value, |found| i64_of(found, "the march steps"))).transpose()?.flatten();
            crate::profile::reset();
            let started = Instant::now();
            let graph = build_motorcycle_graph_with(ctx, &polygon, override_steps);
            extras.push(nanoseconds(started));
            let phases = crate::profile::take();
            if !phases.is_empty() {
                extras.push(Value::List(phases.into_iter().map(int).collect()));
            }
            graph.map(|graph| graph_value(&graph))
        }
        232 => {
            let mut graph = graph_of(at(0)?)?;
            graph.march_steps_override = args.get(5).map(|value| optional(value, |found| i64_of(found, "the march steps"))).transpose()?.flatten();
            let (left, right) = (line_any_of(at(1)?)?, line_any_of(at(2)?)?);
            let (start, origin) = (Rc::new(time_of(at(3)?)?), Rc::new(point_of(at(4)?)?));
            let started = Instant::now();
            let trace = graph.trace_for(ctx, &left, &right, &start, &origin);
            extras.push(nanoseconds(started));
            trace.map(|trace| Value::List(vec![trace_value(&trace), counters_value(&graph.counters), int(graph.next_ident())]))
        }
        233 => return trace_index_script(args),
        234 => return motorcycle_part(args, ctx),
        240 => {
            expect(6)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (vertex, peer, now, same_vertex) = (u32_of(at(2)?, "a vertex")?, u32_of(at(3)?, "a peer")?, time_of(at(4)?)?, flag_of(at(5)?, "the same-vertex flag")?);
            let before = memo.len();
            let started = Instant::now();
            let decision = evaluate_edge_candidate(ctx, &view, &mut memo, vertex, peer, &now, same_vertex);
            extras.push(nanoseconds(started));
            decision.map(|decision| edge_decision_value(&decision, before, memo.len()))
        }
        241 => {
            expect(5)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (vertex, peer, now) = (u32_of(at(2)?, "a vertex")?, u32_of(at(3)?, "a peer")?, time_of(at(4)?)?);
            let before = memo.len();
            let entry = edge_event_time(ctx, &view, &mut memo, vertex, peer, &now);
            entry.map(|(time, outcome)| Value::List(vec![time_entry_value((time.map(|found| (*found).clone()), outcome)), growth_value(before, memo.len())]))
        }
        242 => {
            expect(5)?;
            let view = view_of(at(0)?)?;
            let (time, refs, now) = (time_of(at(2)?)?, list(at(3)?, "vertices")?.iter().map(|vertex| u32_of(vertex, "a vertex")).collect::<Wire<Vec<u32>>>()?, time_of(at(4)?)?);
            is_future(ctx, &view, &time, &refs, &now).map(Value::Bool)
        }
        243 => {
            expect(5)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (vertex, peer, time) = (u32_of(at(2)?, "a vertex")?, u32_of(at(3)?, "a peer")?, time_of(at(4)?)?);
            let before = memo.len();
            let span = collapsing_span(ctx, &view, &mut memo, vertex, peer, &time);
            span.map(|span| Value::List(vec![sum_option_value(span), growth_value(before, memo.len())]))
        }
        244 => {
            expect(6)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (vertex, span, time, at_start) = (u32_of(at(2)?, "a vertex")?, u32_of(at(3)?, "a span")?, time_of(at(4)?)?, flag_of(at(5)?, "at_start")?);
            let before = memo.len();
            let end = span_end(ctx, &view, &mut memo, vertex, span, &time, at_start);
            end.map(|end| Value::List(vec![sum_option_value(end), growth_value(before, memo.len())]))
        }
        245 => {
            expect(3)?;
            Ok(sum_option_value(sliding_projection(&line_any_of(at(0)?)?, &line_any_of(at(1)?)?, &point_of(at(2)?)?)))
        }
        246 => {
            expect(5)?;
            let (view, mut memo) = (view_of(at(0)?)?, memo_of(at(1)?)?);
            let (vertex, peer, birth) = (u32_of(at(2)?, "a vertex")?, u32_of(at(3)?, "a peer")?, time_of(at(4)?)?);
            let before = memo.len();
            let started = Instant::now();
            let found = classify_poststate_span(ctx, &view, &mut memo, vertex, peer, &birth);
            extras.push(nanoseconds(started));
            found.map(|found| {
                Value::List(vec![
                    int(disposition_index(found.disposition)),
                    sum_option_value(found.birth_length),
                    sum_option_value(found.slope),
                    found.orientation_sign.map_or(Value::None, |sign| int(i64::from(sign))),
                    growth_value(before, memo.len()),
                ])
            })
        }
        250 => return proof_script(args),
        other => return Err(SeamError(format!("unknown skeleton seam opcode {other}"))),
    })
}
