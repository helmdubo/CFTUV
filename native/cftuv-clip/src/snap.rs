//! `materialize/clip_snap.py`: the two tolerance laws of the stage (a `src:` vertex within a few cells of a chart corner
//! snaps to it before the cut; a `node:` vertex within one cell of an interior source edge has a zero sign there) and
//! their counters. Exact squares of distances on `SqrtSum` under the budget decide; binary64 only filters.
//!
//! Dictionary orders are semantic: the `src:` candidates in `points` order, the corners x-major then y inside the
//! grid cells, the proposals sorted by key, `moved` in that order.

use std::collections::{HashMap, HashSet};

use cftuv_canon::ordered::OrderedMap;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::num::UBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::{ClipError, ClipResult};
use crate::numeric::{float_of, milli_cells, nanometres, window, ENCLOSURE_BITS};
use crate::plane::{ChartPoint, Plane};
use crate::point::{point_key, Point, PointKey};

/// `SOURCE_VERTEX_CORNER_SNAP_CELLS`.
pub const SOURCE_VERTEX_CORNER_SNAP_CELLS: i64 = 4;

pub const COUNTER_NAMES: [&str; 5] = [
    "MATERIALIZE_CLIP_SOURCE_VERTICES_SNAPPED_TO_CORNER",
    "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_MAX_GAP_NANOMETRES",
    "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_MAX_GAP_MILLICELLS",
    "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_REFUSED_CORNER_TAKEN",
    "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_REFUSED_CORNERS_AMBIGUOUS",
];

/// `CornerSnapV1`.
#[derive(Clone, Debug)]
pub struct CornerSnap {
    /// Every vertex of the domain after the snap, in the order of the input; moved `src:` vertices stand in corners.
    pub points: Vec<(String, Point)>,
    /// Only the moved vertices, sorted by key.
    pub moved: Vec<(String, Point)>,
    /// The five counters in the order of [`COUNTER_NAMES`].
    pub counters: [UBig; 5],
}

/// The most grid cells a corner scan may visit (a lattice chart needs a handful).
const MAX_SCANNED_CELLS: i128 = 50_000_000;

/// `math.floor(value)` as a grid coordinate; beyond what a lattice chart can hold is a named refusal.
fn floor_cell(value: f64) -> ClipResult<i64> {
    let floored = value.floor();
    if !floored.is_finite() || floored.abs() > 4.0e18 {
        return Err(ClipError::Unsupported(format!("a corner grid coordinate {value} is beyond the lattice range")));
    }
    Ok(floored as i64)
}

/// `{(grid x, grid y): [(corner index, x, y)]}`.
type GridCells = HashMap<(i64, i64), Vec<(usize, f64, f64)>>;

/// The moved vertices, the largest move in nanometres and the largest squared move in cells.
type Moves = (Vec<(String, Point)>, UBig, Rat);

struct CornerGrid {
    corners: Vec<ChartPoint>,
    /// `{(grid x, grid y): [(corner index, x, y)]}`, corners in sorted order.
    cells: GridCells,
    /// The triangles of each corner, in triangle order.
    owners: Vec<Vec<usize>>,
}

/// `_corner_grid(plane)`.
fn corner_grid(plane: &Plane) -> ClipResult<CornerGrid> {
    let step = SOURCE_VERTEX_CORNER_SNAP_CELLS as f64;
    let mut seen: HashMap<ChartPoint, Vec<usize>> = HashMap::new();
    for (index, triangle) in plane.triangles.iter().enumerate() {
        for corner in &triangle.chart {
            seen.entry(corner.clone()).or_default().push(index);
        }
    }
    let mut corners: Vec<ChartPoint> = seen.keys().cloned().collect();
    corners.sort();
    let mut cells: GridCells = HashMap::new();
    let mut owners = Vec::with_capacity(corners.len());
    for (number, corner) in corners.iter().enumerate() {
        let x = float_of(&corner.0)?;
        let y = float_of(&corner.1)?;
        cells.entry((floor_cell(x / step)?, floor_cell(y / step)?)).or_default().push((number, x, y));
        owners.push(seen.remove(corner).expect("a seen corner"));
    }
    Ok(CornerGrid { corners, cells, owners })
}

/// `_near_corners(grid, point, plane)`: the corners whose float distance estimate is within the tolerance (with a
/// margin): a filter, never an answer.
fn near_corners(grid: &CornerGrid, point: &Point) -> ClipResult<Vec<usize>> {
    let [x_low, x_high, y_low, y_high] = window(&point.0, &point.1)?;
    let step = SOURCE_VERTEX_CORNER_SNAP_CELLS as f64;
    let reach = step + 1.0;
    let mut found = Vec::new();
    let first_x = floor_cell((x_low - reach) / step)?;
    let last_x = floor_cell((x_high + reach) / step)?;
    let first_y = floor_cell((y_low - reach) / step)?;
    let last_y = floor_cell((y_high + reach) / step)?;
    // a window this wide would make the oracle itself scan forever: a named refusal, not a hang
    let scanned = (i128::from(last_x) - i128::from(first_x) + 1).saturating_mul(i128::from(last_y) - i128::from(first_y) + 1);
    if scanned > MAX_SCANNED_CELLS {
        return Err(ClipError::Unsupported(format!("a corner scan over {scanned} grid cells")));
    }
    for cell_x in first_x..last_x + 1 {
        for cell_y in first_y..last_y + 1 {
            if let Some(bucket) = grid.cells.get(&(cell_x, cell_y)) {
                for (corner, cx, cy) in bucket {
                    if x_low - reach <= *cx && *cx <= x_high + reach && y_low - reach <= *cy && *cy <= y_high + reach {
                        found.push(*corner);
                    }
                }
            }
        }
    }
    Ok(found)
}

/// `_gap_square(point, corner)`: the exact squared distance in cells.
fn gap_square(point: &Point, corner: &ChartPoint, products: &mut cftuv_core::products::ProductMemo) -> SqrtSum {
    let dx = point.0.sub(&SqrtSum::rational(&corner.0));
    let dy = point.1.sub(&SqrtSum::rational(&corner.1));
    dx.mul(&dx, products).add(&dy.mul(&dy, products))
}

/// `_corner_stretch(plane, owners, corner)`: the largest stretch bound among the triangles of the corner (zero floor).
fn corner_stretch(plane: &Plane, owners: &[usize]) -> ClipResult<Rat> {
    let mut stretch = Rat::zero();
    for &triangle in owners {
        let candidate = plane.stretch_square(triangle)?;
        if candidate > stretch {
            stretch = candidate;
        }
    }
    Ok(stretch)
}

fn corner_target(corner: &ChartPoint) -> Point {
    (SqrtSum::rational(&corner.0), SqrtSum::rational(&corner.1))
}

/// `_snap_to_corners(plane, budget, points, tally)`: the moved vertices (sorted by key), the largest move in nanometres
/// and the largest squared move in cells; `tally` gets the ambiguity, taken and snapped counts.
fn snap_to_corners(
    ctx: &mut ExactCtx<'_>,
    plane: &Plane,
    points: &[(String, Point)],
    tally: &mut [UBig; 5],
) -> ClipResult<Moves> {
    let candidates: Vec<usize> = points.iter().enumerate().filter(|(_, (key, _))| key.starts_with("src:")).map(|(index, _)| index).collect();
    if candidates.is_empty() {
        return Ok((Vec::new(), UBig::ZERO, Rat::zero()));
    }
    let limit = SqrtSum::rational(&Rat::from_i64(SOURCE_VERTEX_CORNER_SNAP_CELLS * SOURCE_VERTEX_CORNER_SNAP_CELLS));
    let grid = corner_grid(plane)?;
    let taken: HashSet<PointKey> = points.iter().map(|(_, point)| point_key(point)).collect();
    // `proposals` in candidate order: key -> (corner index, gap square)
    let mut proposals: Vec<(&str, usize, SqrtSum)> = Vec::new();
    for &candidate in &candidates {
        let (key, point) = (&points[candidate].0, &points[candidate].1);
        let mut within: Vec<(usize, SqrtSum)> = Vec::new();
        for corner in near_corners(&grid, point)? {
            let gap = gap_square(point, &grid.corners[corner], ctx.products);
            if exact::sign(ctx, &limit.sub(&gap), SIGN_FILTER_BITS)? >= 0 {
                within.push((corner, gap));
            }
        }
        if within.is_empty() || within.iter().any(|(_, gap)| gap.is_zero()) {
            continue;
        }
        if within.len() > 1 {
            tally[4] = &tally[4] + UBig::ONE;
            continue;
        }
        let (corner, gap) = within.pop().expect("one corner");
        proposals.push((key.as_str(), corner, gap));
    }
    let mut aimed: OrderedMap<usize, usize> = OrderedMap::new();
    for (_, corner, _) in &proposals {
        let count = aimed.get(corner).copied().unwrap_or(0);
        aimed.set(*corner, count + 1);
    }
    proposals.sort_by(|left, right| left.0.cmp(right.0));
    let mut moved: Vec<(String, Point)> = Vec::new();
    let mut widest = UBig::ZERO;
    let mut widest_square = Rat::zero();
    for (key, corner, gap) in &proposals {
        let target = corner_target(&grid.corners[*corner]);
        if aimed.get(corner).copied().unwrap_or(0) > 1 || taken.contains(&point_key(&target)) {
            tally[3] = &tally[3] + UBig::ONE;
            continue;
        }
        moved.push(((*key).to_string(), target));
        tally[0] = &tally[0] + UBig::ONE;
        let upper = gap.enclosure(ENCLOSURE_BITS).1;
        let reach = nanometres(&upper.mul(&corner_stretch(plane, &grid.owners[*corner])?))?;
        if reach > widest {
            widest = reach;
        }
        if upper > widest_square {
            widest_square = upper;
        }
    }
    Ok((moved, widest, widest_square))
}

/// `snap_source_vertices(plane, budget, points)`: the `src:` vertices near a chart corner stand in it; nothing else moves.
pub fn snap_source_vertices(ctx: &mut ExactCtx<'_>, plane: &Plane, points: &[(String, Point)]) -> ClipResult<CornerSnap> {
    let mut tally: [UBig; 5] = [UBig::ZERO, UBig::ZERO, UBig::ZERO, UBig::ZERO, UBig::ZERO];
    let (moved, widest, widest_square) = snap_to_corners(ctx, plane, points, &mut tally)?;
    tally[1] = widest;
    tally[2] = milli_cells(&widest_square)?;
    let replaced: HashMap<&str, &Point> = moved.iter().map(|(key, point)| (key.as_str(), point)).collect();
    let merged = points
        .iter()
        .map(|(key, point)| (key.clone(), replaced.get(key.as_str()).map_or_else(|| point.clone(), |target| (*target).clone())))
        .collect();
    Ok(CornerSnap { points: merged, moved, counters: tally })
}

/// `within_edge_gap(value, edge_square, budget)`: `(within the tolerance, upper bound of the squared distance in cells)`
/// of a vertex from the line of an edge (law 2). The enclosure of `value` is a strict filter; the exact sign of
/// `cells^2 |edge|^2 - value^2` decides the rest.
pub fn within_edge_gap(ctx: &mut ExactCtx<'_>, value: &SqrtSum, edge_square: &Rat) -> ClipResult<(bool, Rat)> {
    let (low, high) = value.enclosure(ENCLOSURE_BITS);
    // `NODE_EDGE_SNAP_CELLS * NODE_EDGE_SNAP_CELLS * edge_square` with the tolerance of one cell
    let limit = edge_square.clone();
    let low_square = low.mul(&low);
    let high_square = high.mul(&high);
    let nearest = if low.signum() <= 0 && high.signum() >= 0 {
        Rat::zero()
    } else if low_square < high_square {
        low_square.clone()
    } else {
        high_square.clone()
    };
    if nearest > limit {
        return Ok((false, Rat::zero()));
    }
    let difference = SqrtSum::rational(&limit).sub(&value.mul(value, ctx.products));
    if exact::sign(ctx, &difference, SIGN_FILTER_BITS)? < 0 {
        return Ok((false, Rat::zero()));
    }
    let widest = if high_square > low_square { high_square } else { low_square };
    let gap = widest.div(edge_square).map_err(|_| ClipError::Value("a zero-length edge"))?;
    Ok((true, gap))
}
