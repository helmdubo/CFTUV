//! The lifted surface the clip stage reads (`BoundSurfaceLiftV1` of `lift_surface.py`): the source triangles with
//! their chart, their 3D corners and their offset normals, and the exact orientation value of a point against an
//! edge of the chart (`_edge_value` / `line_value`), the stretch bound (`stretch_square`, `_lipschitz_square`) and the
//! float window of a point.
//!
//! `Triangle` is `LiftTriangleV1`; `chart` coordinates are `Fraction`s (lattice integers in practice, but a
//! non-integer chart is handled by the second branch of `_edge_value`, as in the oracle). Nothing here pays cost:
//! the arithmetic of `SqrtSum` is free, only `.sign()` is not and the callers own it.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, OnceLock};

use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::cells::{build_cells, single, CellMemo};
use crate::error::{ClipError, ClipResult};
use crate::numeric::upper_root;
use crate::regions::{EdgeLine, RegionSet};
use crate::snap::{corner_grid, CornerGrid};

/// A point of the chart: two `Fraction`s.
pub type ChartPoint = (Rat, Rat);

/// `LiftTriangleV1` (the fields the clip reads).
#[derive(Clone, Debug, PartialEq)]
pub struct Triangle {
    pub name: String,
    pub chart: [ChartPoint; 3],
    pub corners: [[Rat; 3]; 3],
    pub twice_area: Rat,
    /// `(xmin, xmax, ymin, ymax)`, outward-rounded binary64.
    pub bbox: [f64; 4],
    /// The three unit offset normals of the corners, or none (`normals == ()`).
    pub normals: Option<[[f64; 3]; 3]>,
    /// The source face the triangle comes from; empty: the triangle is its own face.
    pub face: String,
}

/// A hash of everything about a triangle the clip reads (a warm-cache key; equal triangles hash alike).
pub fn triangle_hash(triangle: &Triangle) -> u64 {
    const SEED: u64 = 0x517c_c1b7_2722_0a95;
    let mut state = 0u64;
    {
        let mut mix = |word: u64| state = (state.rotate_left(5) ^ word).wrapping_mul(SEED);
        triangle.name.bytes().for_each(|byte| mix(u64::from(byte)));
        triangle.face.bytes().for_each(|byte| mix(u64::from(byte) | (1 << 40)));
        triangle.bbox.iter().for_each(|edge| mix(edge.to_bits()));
        if let Some(normals) = &triangle.normals {
            normals.iter().flatten().for_each(|axis| mix(axis.to_bits()));
        }
    }
    for rat in triangle.chart.iter().flat_map(|(x, y)| [x, y]).chain(triangle.corners.iter().flatten()).chain(std::iter::once(&triangle.twice_area)) {
        crate::point::mix_rat(&mut state, rat);
    }
    state
}

/// The three edge lines of a triangle (`edge_value` of each chart edge) with their hashes.
pub type TriangleLines = [(EdgeLine, u64); 3];

fn triangle_lines(triangle: &Triangle) -> TriangleLines {
    [0, 1, 2].map(|index| {
        let line = EdgeLine::new(&triangle.chart[index], &triangle.chart[(index + 1) % 3]);
        let hash = line.hash();
        (line, hash)
    })
}

/// `corners[position][axis] / twice_area` of a triangle: the factors `lift_in` weighs the orientation values with.
pub type LiftFactors = [[Rat; 3]; 3];

/// The factors of a triangle; a zero `twice_area` (a degenerate triangle the oracle's division refuses) is a named refusal.
pub fn lift_factors(triangle: &Triangle) -> ClipResult<LiftFactors> {
    let factor = |position: usize, axis: usize| triangle.corners[position][axis].div(&triangle.twice_area).map_err(|_| crate::error::ClipError::Value("a degenerate source triangle"));
    let row = |position: usize| -> ClipResult<[Rat; 3]> { Ok([factor(position, 0)?, factor(position, 1)?, factor(position, 2)?]) };
    Ok([row(0)?, row(1)?, row(2)?])
}

/// What the first stage of `_cut_by_faces` takes from the plane alone: the regions (the merged cells of the law by faces, built with an
/// empty split), the faces that merged into no cell, and the cell memo the second stage continues from.
#[derive(Debug)]
pub struct FirstStage {
    pub regions: Arc<RegionSet>,
    pub unmergeable: Vec<(String, &'static str)>,
    pub memo: CellMemo,
}

/// `BoundSurfaceLiftV1` without the budget (the budget is a parameter of the operations that spend it).
#[derive(Debug, Default)]
pub struct Plane {
    pub triangles: Vec<Triangle>,
    /// One cell per triangle: its [`LiftFactors`], computed on first use (`None`: the triangle is degenerate). Pure functions of the
    /// triangle, like `stretch`.
    factors: Vec<OnceLock<Option<LiftFactors>>>,
    /// The lines of the edges of each triangle, built on first use.
    lines: Vec<OnceLock<TriangleLines>>,
    /// `triangle_hash` of each triangle.
    pub hashes: Vec<u64>,
    /// The first stage of a clip by faces (`build_cells(triangles, {}, memo)`), built once per plane.
    first: OnceLock<ClipResult<FirstStage>>,
    /// The regions of a clip by triangles: one single-triangle region per triangle.
    triangle_regions: OnceLock<Arc<RegionSet>>,
    /// The grid of the chart corners the corner snap scans (`_corner_grid`), built on first use.
    grid: OnceLock<ClipResult<CornerGrid>>,
    /// `plane._stretch`: a pure cache (`stretch_square` is a function of the triangle). A `Mutex` only so that a cached plane
    /// can live in the Python-owned session (`Sync`); the operation is single-threaded and the lock is never contended.
    stretch: Mutex<HashMap<usize, Rat>>,
}

impl Plane {
    pub fn new(triangles: Vec<Triangle>) -> Plane {
        let factors = triangles.iter().map(|_| OnceLock::new()).collect();
        let lines = triangles.iter().map(|_| OnceLock::new()).collect();
        let hashes = triangles.iter().map(triangle_hash).collect();
        Plane { triangles, factors, lines, hashes, first: OnceLock::new(), triangle_regions: OnceLock::new(), grid: OnceLock::new(), stretch: Mutex::new(HashMap::new()) }
    }

    /// `plane.stretch_square(triangle)` for the triangle at `index` (cached per triangle, as the oracle caches by name).
    pub fn stretch_square(&self, index: usize) -> ClipResult<Rat> {
        if let Some(found) = self.stretch.lock().expect("the stretch cache lock is never poisoned").get(&index) {
            return Ok(found.clone());
        }
        let computed = lipschitz_square(&self.triangles[index])?;
        self.stretch.lock().expect("the stretch cache lock is never poisoned").insert(index, computed.clone());
        Ok(computed)
    }

    /// The first stage of a clip by faces: a pure function of the triangles (`build_cells` reads nothing else), so the plane builds it once
    /// and every call shares it; a refusal of the build is the refusal of every call.
    pub fn first_stage(&self) -> ClipResult<&FirstStage> {
        self.first
            .get_or_init(|| {
                let mut memo = CellMemo::new();
                let plan = build_cells(&self.triangles, &HashSet::new(), &mut memo)?;
                Ok(FirstStage { regions: Arc::new(RegionSet::new(plan.cells)), unmergeable: plan.unmergeable, memo })
            })
            .as_ref()
            .map_err(ClipError::clone)
    }

    /// The regions of a clip by triangles (the triangles themselves as cells), built once per plane.
    pub fn triangle_regions(&self) -> Arc<RegionSet> {
        self.triangle_regions.get_or_init(|| Arc::new(RegionSet::new((0..self.triangles.len()).map(|index| single(&self.triangles, index, None, None)).collect()))).clone()
    }

    /// The corner grid of the snap (a function of the triangles), built when the first `src:` vertex asks for it; a refusal of the
    /// build is the refusal of every call, as it is when the oracle builds it per call.
    pub fn corner_grid(&self) -> ClipResult<&CornerGrid> {
        self.grid.get_or_init(|| corner_grid(self)).as_ref().map_err(ClipError::clone)
    }

    /// The edge lines of the triangle at `index` (built once per plane).
    pub fn triangle_lines(&self, index: usize) -> &TriangleLines {
        self.lines[index].get_or_init(|| triangle_lines(&self.triangles[index]))
    }

    /// The lift factors of the triangle at `index` (computed once per plane).
    pub fn lift_factors(&self, index: usize) -> ClipResult<&LiftFactors> {
        self.factors[index]
            .get_or_init(|| lift_factors(&self.triangles[index]).ok())
            .as_ref()
            .ok_or(crate::error::ClipError::Value("a degenerate source triangle"))
    }

    /// `plane.line_value(triangle, index, point)`.
    pub fn line_value(&self, triangle: usize, index: usize, x: &SqrtSum, y: &SqrtSum) -> SqrtSum {
        line_value(&self.triangles[triangle].chart, index, x, y)
    }

    /// `plane.values_in(triangle, point)`: the three orientation values of a point against the edges of a triangle.
    pub fn values_in(&self, triangle: usize, x: &SqrtSum, y: &SqrtSum) -> [SqrtSum; 3] {
        values_in(&self.triangles[triangle], x, y)
    }
}

/// `lift_surface._edge_value(start, end, point)`: `(end - start) x (point - start)`, exact (see [`EdgeLine`]).
pub fn edge_value(start: &ChartPoint, end: &ChartPoint, x: &SqrtSum, y: &SqrtSum) -> SqrtSum {
    EdgeLine::new(start, end).value(x, y)
}

/// `line_value` on any chart loop (a triangle's or a cell's): the edge `index` runs to the next corner, wrapping.
pub fn line_value(chart: &[ChartPoint], index: usize, x: &SqrtSum, y: &SqrtSum) -> SqrtSum {
    edge_value(&chart[index], &chart[(index + 1) % chart.len()], x, y)
}

/// `values_in(triangle, point)`.
pub fn values_in(triangle: &Triangle, x: &SqrtSum, y: &SqrtSum) -> [SqrtSum; 3] {
    [0, 1, 2].map(|index| edge_value(&triangle.chart[index], &triangle.chart[(index + 1) % 3], x, y))
}

/// `lift_surface._lipschitz_square(triangle)`: an upper bound of the largest eigenvalue of `J^T J` of the affine lift
/// of the triangle (`J` the 3x2 Jacobian), exact up to the rational root `upper_root`.
pub fn lipschitz_square(triangle: &Triangle) -> ClipResult<Rat> {
    let [(ax, ay), (bx, by), (cx, cy)] = &triangle.chart;
    let e1x = bx.sub(ax);
    let e1y = by.sub(ay);
    let e2x = cx.sub(ax);
    let e2y = cy.sub(ay);
    let det = e1x.mul(&e2y).sub(&e1y.mul(&e2x));
    let first: Vec<Rat> = (0..3).map(|axis| triangle.corners[1][axis].sub(&triangle.corners[0][axis])).collect();
    let second: Vec<Rat> = (0..3).map(|axis| triangle.corners[2][axis].sub(&triangle.corners[0][axis])).collect();
    let dot = |left: &[Rat], right: &[Rat]| -> Rat {
        // `sum(q * r for ...)` starts from the int 0: the left fold of exact rationals
        let mut total = Rat::zero();
        for (q, r) in left.iter().zip(right.iter()) {
            total = total.add(&q.mul(r));
        }
        total
    };
    let cross = dot(&first, &second);
    let gram = [[dot(&first, &first), cross.clone()], [cross, dot(&second, &second)]];
    // `M^-1 = [[e2y, -e2x], [-e1y, e1x]] / det`; `J^T J = M^-T G M^-1` is a symmetric 2x2 `[[p, q], [q, r]]`.
    let rows = [[e2y.clone(), e2x.neg()], [e1y.neg(), e1x.clone()]];
    let denominator = det.mul(&det);
    let mut entries = Vec::with_capacity(3);
    for (i, j) in [(0usize, 0usize), (0, 1), (1, 1)] {
        let mut total = Rat::zero();
        for k in 0..2 {
            for m in 0..2 {
                total = total.add(&rows[k][i].mul(&gram[k][m]).mul(&rows[m][j]));
            }
        }
        entries.push(total.div(&denominator).map_err(|_| crate::error::ClipError::Value("a degenerate chart triangle"))?);
    }
    let (p, q, r) = (&entries[0], &entries[1], &entries[2]);
    let two = Rat::from_int(IBig::from(2));
    let half = p.sub(r).div(&two).expect("two is not zero");
    let root = upper_root(&half.mul(&half).add(&q.mul(q)))?;
    Ok(p.add(r).div(&two).expect("two is not zero").add(&root))
}
