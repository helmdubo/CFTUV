//! The cut regions of a stage and everything about them that is a function of the regions alone: the direction of each, the
//! integer constants and the squared length of each edge, and the exact line of each edge (`_edge_value`'s constants).
//!
//! A [`RegionSet`] is immutable and shared (`Arc`): the plane builds the set of the first stage (the merged cells of the law by faces,
//! or the triangles of the lift) ONCE and every call of a session clips against it, where the oracle rebuilds it per call. Nothing in
//! it depends on the points, the budget or the memory, so no answer and no counted cost can.

use std::ops::Index;

use cftuv_core::fused::oriented_sum;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::cells::ClipCell;
use crate::edge::{edge_constants, EdgeConstants};
use crate::plane::ChartPoint;

/// The exact orientation value of a point against the line of one edge of a chart: `(end - start) x (point - start)`
/// (`lift_surface._edge_value`). A lattice-integer edge takes the fused single-normalisation kernel with its three integers
/// (computed once here, not per point); anything else the plain chain (`scaled`, `-`, `+`).
#[derive(Clone, Debug, PartialEq)]
pub enum EdgeLine {
    Lattice { dx: IBig, dy: IBig, offset: IBig },
    Plain { start: ChartPoint, end: ChartPoint },
}

impl EdgeLine {
    pub fn new(start: &ChartPoint, end: &ChartPoint) -> EdgeLine {
        let dx = end.0.sub(&start.0);
        let dy = end.1.sub(&start.1);
        if dx.is_integer() && dy.is_integer() && start.0.is_integer() && start.1.is_integer() {
            let offset = dy.numerator() * start.0.numerator() - dx.numerator() * start.1.numerator();
            return EdgeLine::Lattice { dx: dx.numerator().clone(), dy: dy.numerator().clone(), offset };
        }
        EdgeLine::Plain { start: start.clone(), end: end.clone() }
    }

    /// A hash of the line (its integers, or its two ends): equal lines hash alike.
    pub fn hash(&self) -> u64 {
        const SEED: u64 = 0x517c_c1b7_2722_0a95;
        let mut state = 0u64;
        let mut mix = |word: u64| state = (state.rotate_left(5) ^ word).wrapping_mul(SEED);
        let int = |value: &IBig, mix: &mut dyn FnMut(u64)| {
            let (sign, words) = value.as_sign_words();
            mix(sign as u64);
            words.iter().for_each(|word| mix(*word));
        };
        match self {
            EdgeLine::Lattice { dx, dy, offset } => {
                mix(1);
                int(dx, &mut mix);
                int(dy, &mut mix);
                int(offset, &mut mix);
            }
            EdgeLine::Plain { start, end } => {
                mix(2);
                for rat in [&start.0, &start.1, &end.0, &end.1] {
                    int(rat.numerator(), &mut mix);
                    rat.denominator().as_words().iter().for_each(|word| mix(*word));
                }
            }
        }
        state
    }

    pub fn value(&self, x: &SqrtSum, y: &SqrtSum) -> SqrtSum {
        match self {
            EdgeLine::Lattice { dx, dy, offset } => oriented_sum(x, y, dx, dy, offset),
            EdgeLine::Plain { start, end } => {
                let dx = end.0.sub(&start.0);
                let dy = end.1.sub(&start.1);
                let offset = dy.mul(&start.0).sub(&dx.mul(&start.1));
                y.scaled(&dx).sub(&x.scaled(&dy)).add(&SqrtSum::rational(&offset))
            }
        }
    }
}

/// The regions of a stage with their derived constants.
#[derive(Debug)]
pub struct RegionSet {
    pub cells: Vec<ClipCell>,
    /// `1` for a counter-clockwise region (`twice_area > 0`), else `-1`.
    pub directions: Vec<i8>,
    pub has_groups: bool,
    /// `_edge_constants(ti, index)`.
    pub constants: Vec<Vec<Option<EdgeConstants>>>,
    /// `_edge_square(ti, index)`: `dx^2 + dy^2` of the chart edge.
    pub edge_squares: Vec<Vec<Rat>>,
    /// The line of each edge.
    pub lines: Vec<Vec<EdgeLine>>,
    /// The hash of each line (the key of the warm cache).
    pub line_hashes: Vec<Vec<u64>>,
}

impl RegionSet {
    pub fn new(cells: Vec<ClipCell>) -> RegionSet {
        let directions = cells.iter().map(|item| if item.twice_area.signum() > 0 { 1 } else { -1 }).collect();
        let has_groups = cells.iter().any(|item| item.group.is_some());
        let constants = cells.iter().map(|item| (0..item.chart.len()).map(|index| edge_constants(&item.chart, index)).collect()).collect();
        let edge_squares = cells
            .iter()
            .map(|item| {
                let size = item.chart.len();
                (0..size)
                    .map(|index| {
                        let (first, second) = (&item.chart[index], &item.chart[(index + 1) % size]);
                        let dx = second.0.sub(&first.0);
                        let dy = second.1.sub(&first.1);
                        dx.mul(&dx).add(&dy.mul(&dy))
                    })
                    .collect()
            })
            .collect();
        let lines: Vec<Vec<EdgeLine>> = cells
            .iter()
            .map(|item| {
                let size = item.chart.len();
                (0..size).map(|index| EdgeLine::new(&item.chart[index], &item.chart[(index + 1) % size])).collect()
            })
            .collect();
        let line_hashes = lines.iter().map(|row| row.iter().map(EdgeLine::hash).collect()).collect();
        RegionSet { cells, directions, has_groups, constants, edge_squares, lines, line_hashes }
    }

    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    pub fn iter(&self) -> std::slice::Iter<'_, ClipCell> {
        self.cells.iter()
    }
}

impl Index<usize> for RegionSet {
    type Output = ClipCell;

    fn index(&self, index: usize) -> &ClipCell {
        &self.cells[index]
    }
}
