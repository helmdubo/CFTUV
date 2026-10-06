//! The lifted surface the clip stage reads (`BoundSurfaceLiftV1` of `lift_surface.py`): the source triangles with
//! their chart, their 3D corners and their offset normals, and the exact orientation value of a point against an
//! edge of the chart (`_edge_value` / `line_value`), the stretch bound (`stretch_square`, `_lipschitz_square`) and the
//! float window of a point.
//!
//! `Triangle` is `LiftTriangleV1`; `chart` coordinates are `Fraction`s (lattice integers in practice, but a
//! non-integer chart is handled by the second branch of `_edge_value`, as in the oracle). Nothing here pays cost:
//! the arithmetic of `SqrtSum` is free, only `.sign()` is not and the callers own it.

use std::cell::RefCell;
use std::collections::HashMap;

use cftuv_core::fused::oriented_sum;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::error::ClipResult;
use crate::numeric::upper_root;

/// A point of the chart: two `Fraction`s.
pub type ChartPoint = (Rat, Rat);

/// `LiftTriangleV1` (the fields the clip reads).
#[derive(Clone, Debug)]
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

/// `BoundSurfaceLiftV1` without the budget (the budget is a parameter of the operations that spend it).
#[derive(Debug, Default)]
pub struct Plane {
    pub triangles: Vec<Triangle>,
    /// `plane._stretch`: a pure cache (`stretch_square` is a function of the triangle).
    stretch: RefCell<HashMap<usize, Rat>>,
}

impl Plane {
    pub fn new(triangles: Vec<Triangle>) -> Plane {
        Plane { triangles, stretch: RefCell::new(HashMap::new()) }
    }

    /// `plane.stretch_square(triangle)` for the triangle at `index` (cached per triangle, as the oracle caches by name).
    pub fn stretch_square(&self, index: usize) -> ClipResult<Rat> {
        if let Some(found) = self.stretch.borrow().get(&index) {
            return Ok(found.clone());
        }
        let computed = lipschitz_square(&self.triangles[index])?;
        self.stretch.borrow_mut().insert(index, computed.clone());
        Ok(computed)
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

/// `lift_surface._edge_value(start, end, point)`: `(end - start) x (point - start)`, exact. Lattice-integer ends take
/// the fused single-normalisation kernel; anything else the plain chain (`scaled`, `-`, `+`).
pub fn edge_value(start: &ChartPoint, end: &ChartPoint, x: &SqrtSum, y: &SqrtSum) -> SqrtSum {
    let dx = end.0.sub(&start.0);
    let dy = end.1.sub(&start.1);
    if dx.is_integer() && dy.is_integer() && start.0.is_integer() && start.1.is_integer() {
        let offset = dy.numerator() * start.0.numerator() - dx.numerator() * start.1.numerator();
        return oriented_sum(x, y, dx.numerator(), dy.numerator(), &offset);
    }
    let offset = dy.mul(&start.0).sub(&dx.mul(&start.1));
    y.scaled(&dx).sub(&x.scaled(&dy)).add(&SqrtSum::rational(&offset))
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
