//! The cell grid over the lattice (`wavefront/cell_grid.py`): a FILTER that answers which cells an object may touch, never a geometric question.
//!
//! The grid works on exact rationals and rounds OUTWARD: `math.floor` and `math.ceil` of a `Fraction` are floor and ceiling division, toward minus infinity
//! for a negative value too, and NOT the truncation of the integer `/` of Rust (`floor(-7/3) = -3`, `ceil(-7/3) = -2`). Everything here goes through
//! [`floor_of`] and [`ceil_of`], which are tested against that definition on negative origins and negative values.
//!
//! A window or a box is a [`Rat`] (the margin of a trace is a rational) or, on the march, plain machine integers: the two entry points of `box_cells`
//! make the same cells (a property test says so) and the integer one pays no big-number arithmetic. The cells come out in the oracle's order, column
//! first, rows inside it. The coordinates of a grid are machine integers; a box beyond them is a named `Unsupported`.

use std::collections::HashMap;

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;

use crate::error::{SkelError, SkelResult};

pub type Cell = (i64, i64);

/// The grid limits and boxes stay below this in magnitude, so a width, a cell count and a product of the two fit `i128` with room.
pub const GRID_LIMIT: i64 = 1 << 61;

/// `floor(n / d)` for a positive `d`: the truncated quotient, one less when the division was inexact and the dividend negative.
pub fn floor_ratio(n: &IBig, d: &UBig) -> IBig {
    let divisor = IBig::from(d.clone());
    let quotient = n / &divisor;
    if num::is_negative(n) && &quotient * &divisor != *n {
        quotient - IBig::ONE
    } else {
        quotient
    }
}

/// `math.floor(Fraction)`.
pub fn floor_of(value: &Rat) -> IBig {
    if value.is_integer() {
        return value.numerator().clone();
    }
    floor_ratio(value.numerator(), value.denominator())
}

/// `ceil(n / d)` for a positive `d`: `-floor(-n / d)`.
pub fn ceil_ratio(n: &IBig, d: &UBig) -> IBig {
    -floor_ratio(&-n, d)
}

/// `math.ceil(Fraction)`: `-floor(-value)`.
pub fn ceil_of(value: &Rat) -> IBig {
    -floor_of(&value.neg())
}

/// An integer narrowed to the machine range of the grid, saturating: the callers clip the index to the grid right after, so an index beyond the range
/// clips to the same end it would have clipped to unsaturated.
fn saturate(value: &IBig) -> i64 {
    i64::try_from(value).unwrap_or(if num::is_negative(value) { i64::MIN } else { i64::MAX })
}

/// `floor(n / d)` for machine integers and a positive `d`.
fn floor_div_i128(n: i128, d: i128) -> i128 {
    n.div_euclid(d)
}

/// `ceil(n / d)` for machine integers and a positive `d`.
fn ceil_div_i128(n: i128, d: i128) -> i128 {
    -(-n).div_euclid(d)
}

/// An integer of a box that must fit the grid's machine range (a named `Unsupported` otherwise).
pub fn narrow_to_grid(value: &IBig, what: &str) -> SkelResult<i64> {
    match i64::try_from(value) {
        Ok(found) if found.abs() < GRID_LIMIT => Ok(found),
        _ => Err(SkelError::Unsupported(format!("{what} {value} is beyond the machine range of the cell grid"))),
    }
}

/// `CellGridV1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CellGrid {
    pub x_min: i64,
    pub y_min: i64,
    pub x_max: i64,
    pub y_max: i64,
    pub cell: i64,
}

fn rat_of(value: i128) -> Rat {
    Rat::from_int(IBig::from(value))
}

impl CellGrid {
    /// `CellGridV1.covering(box, targets=...)`: the smallest power-of-two cell for which the grid has no more than about `targets` cells. `bounds` is
    /// `(x_min, y_min, x_max, y_max)`. An empty box is the named `CellGridRejected`.
    pub fn covering(bounds: (i64, i64, i64, i64), targets: i64) -> SkelResult<CellGrid> {
        let (x_min, y_min, x_max, y_max) = bounds;
        if x_max < x_min || y_max < y_min {
            return Err(SkelError::CellGridRejected(format!("пустая область ({x_min}, {y_min}, {x_max}, {y_max})")));
        }
        if [x_min, y_min, x_max, y_max].iter().any(|value| value.abs() >= GRID_LIMIT) {
            return Err(SkelError::Unsupported(format!("the box ({x_min}, {y_min}, {x_max}, {y_max}) is beyond the machine range of the cell grid")));
        }
        let width = i128::from(x_max - x_min).max(1);
        let height = i128::from(y_max - y_min).max(1);
        let budget = i128::from(targets).max(1);
        let mut cell: i128 = 1;
        while (width / cell + 1) * (height / cell + 1) > budget {
            cell *= 2;
        }
        Ok(CellGrid { x_min, y_min, x_max, y_max, cell: cell as i64 })
    }

    pub fn columns(&self) -> i64 {
        (self.x_max - self.x_min) / self.cell + 1
    }

    pub fn rows(&self) -> i64 {
        (self.y_max - self.y_min) / self.cell + 1
    }

    /// The right edge of the area the cells cover, not of the declared box.
    pub fn x_limit(&self) -> i64 {
        self.x_min + self.columns() * self.cell
    }

    pub fn y_limit(&self) -> i64 {
        self.y_min + self.rows() * self.cell
    }

    /// `column(x)`: `floor((x - x_min) / cell)`.
    pub fn column(&self, x: &Rat) -> IBig {
        floor_of(&x.sub(&rat_of(i128::from(self.x_min))).div(&rat_of(i128::from(self.cell))).expect("a cell is positive"))
    }

    pub fn row(&self, y: &Rat) -> IBig {
        floor_of(&y.sub(&rat_of(i128::from(self.y_min))).div(&rat_of(i128::from(self.cell))).expect("a cell is positive"))
    }

    /// `ceil((value - origin) / cell) - 1`: the cells are CLOSED, so a value on a border belongs to the cell before it too.
    fn low_index(&self, value: &Rat, origin: i64) -> i64 {
        let scaled = value.sub(&rat_of(i128::from(origin))).div(&rat_of(i128::from(self.cell))).expect("a cell is positive");
        saturate(&(ceil_of(&scaled) - IBig::ONE))
    }

    fn high_index(&self, value: &Rat, origin: i64) -> i64 {
        let scaled = value.sub(&rat_of(i128::from(origin))).div(&rat_of(i128::from(self.cell))).expect("a cell is positive");
        saturate(&floor_of(&scaled))
    }

    fn low_index_int(&self, value: i64, origin: i64) -> i64 {
        (ceil_div_i128(i128::from(value) - i128::from(origin), i128::from(self.cell)) - 1).clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64
    }

    fn high_index_int(&self, value: i64, origin: i64) -> i64 {
        floor_div_i128(i128::from(value) - i128::from(origin), i128::from(self.cell)).clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64
    }

    /// `contains_box`: the rectangle lies wholly inside the declared area (if not, a query through the grid is incomplete and the caller goes to the full search).
    pub fn contains_box(&self, x_low: &Rat, x_high: &Rat, y_low: &Rat, y_high: &Rat) -> bool {
        *x_low >= rat_of(i128::from(self.x_min)) && *x_high <= rat_of(i128::from(self.x_max)) && *y_low >= rat_of(i128::from(self.y_min)) && *y_high <= rat_of(i128::from(self.y_max))
    }

    fn cells_of(&self, first_column: i64, last_column: i64, first_row: i64, last_row: i64) -> Vec<Cell> {
        let first_column = first_column.max(0);
        let last_column = last_column.min(self.columns() - 1);
        let first_row = first_row.max(0);
        let last_row = last_row.min(self.rows() - 1);
        if last_column < first_column || last_row < first_row {
            return Vec::new();
        }
        let mut cells = Vec::with_capacity(((last_column - first_column + 1) * (last_row - first_row + 1)) as usize);
        for column in first_column..=last_column {
            for row in first_row..=last_row {
                cells.push((column, row));
            }
        }
        cells
    }

    /// `box_cells(x_low, x_high, y_low, y_high)`: every cell the rectangle touches, clipped to the area.
    pub fn box_cells(&self, x_low: &Rat, x_high: &Rat, y_low: &Rat, y_high: &Rat) -> Vec<Cell> {
        self.cells_of(self.low_index(x_low, self.x_min), self.high_index(x_high, self.x_min), self.low_index(y_low, self.y_min), self.high_index(y_high, self.y_min))
    }

    /// [`CellGrid::box_cells`] of a box of machine integers (the march's boxes: the floor and ceiling of an enclosure).
    pub fn box_cells_int(&self, x_low: i64, x_high: i64, y_low: i64, y_high: i64) -> Vec<Cell> {
        self.cells_of(self.low_index_int(x_low, self.x_min), self.high_index_int(x_high, self.x_min), self.low_index_int(y_low, self.y_min), self.high_index_int(y_high, self.y_min))
    }

    /// `_clipped(window)`: the window cut to the covered area (the whole area for `None`).
    fn clipped(&self, window: Option<[Rat; 4]>) -> [Rat; 4] {
        let [x_low, x_high, y_low, y_high] = match window {
            None => return [rat_of(i128::from(self.x_min)), rat_of(i128::from(self.x_limit())), rat_of(i128::from(self.y_min)), rat_of(i128::from(self.y_limit()))],
            Some(window) => window,
        };
        [
            x_low.max(rat_of(i128::from(self.x_min))),
            x_high.min(rat_of(i128::from(self.x_limit()))),
            y_low.max(rat_of(i128::from(self.y_min))),
            y_high.min(rat_of(i128::from(self.y_limit()))),
        ]
    }

    /// `line_cells(a, b, c, window=...)`: the cells the line `a*x + b*y = c` may cross, inside its own window when it has one (a segment has).
    pub fn line_cells(&self, a: i128, b: i128, c: i128, window: Option<[Rat; 4]>) -> Vec<Cell> {
        let [x_low, x_high, y_low, y_high] = self.clipped(window);
        if x_high < x_low || y_high < y_low || (a == 0 && b == 0) {
            return Vec::new();
        }
        if b == 0 {
            let x = Rat::new(IBig::from(c), IBig::from(a)).expect("a is not zero");
            if !(x_low <= x && x <= x_high) {
                return Vec::new();
            }
            return self.box_cells(&x, &x, &y_low, &y_high);
        }
        let (mut x_low, mut x_high) = (x_low, x_high);
        if a != 0 {
            let at = |y: &Rat| rat_of(c).sub(&rat_of(b).mul(y)).div(&rat_of(a)).expect("a is not zero");
            let (at_low, at_high) = (at(&y_low), at(&y_high));
            x_low = x_low.max(at_low.clone().min(at_high.clone()));
            x_high = x_high.min(at_low.max(at_high));
            if x_high < x_low {
                return Vec::new();
            }
        }
        self.columns_of_line(a, b, c, &x_low, &x_high, &y_low, &y_high)
    }

    /// `segment_cells(start, end)`: the same marking with the segment's own box as its window.
    pub fn segment_cells(&self, start: (i64, i64), end: (i64, i64)) -> Vec<Cell> {
        let ((x0, y0), (x1, y1)) = (start, end);
        let (a, b) = (i128::from(y0) - i128::from(y1), i128::from(x1) - i128::from(x0));
        if a == 0 && b == 0 {
            return self.box_cells_int(x0, x0, y0, y0);
        }
        let window = [rat_of(i128::from(x0.min(x1))), rat_of(i128::from(x0.max(x1))), rat_of(i128::from(y0.min(y1))), rat_of(i128::from(y0.max(y1)))];
        self.line_cells(a, b, a * i128::from(x0) + b * i128::from(y0), Some(window))
    }

    #[allow(clippy::too_many_arguments)]
    fn columns_of_line(&self, a: i128, b: i128, c: i128, x_low: &Rat, x_high: &Rat, y_low: &Rat, y_high: &Rat) -> Vec<Cell> {
        let mut cells = Vec::new();
        let first_column = self.low_index(x_low, self.x_min).max(0);
        let last_column = self.high_index(x_high, self.x_min).min(self.columns() - 1);
        let cell = i128::from(self.cell);
        for column in first_column..=last_column {
            let left = x_low.clone().max(rat_of(i128::from(self.x_min) + i128::from(column) * cell));
            let right = x_high.clone().min(rat_of(i128::from(self.x_min) + (i128::from(column) + 1) * cell));
            if right < left {
                continue;
            }
            let at = |x: &Rat| rat_of(c).sub(&rat_of(a).mul(x)).div(&rat_of(b)).expect("b is not zero");
            let (at_left, at_right) = (at(&left), at(&right));
            let low = y_low.clone().max(at_left.clone().min(at_right.clone()));
            let high = y_high.clone().min(at_left.max(at_right));
            if high < low {
                continue;
            }
            let first_row = self.low_index(&low, self.y_min).max(0);
            let last_row = self.high_index(&high, self.y_min).min(self.rows() - 1);
            for row in first_row..=last_row {
                cells.push((column, row));
            }
        }
        cells
    }
}

/// `CellIndexV1`: cell -> identifiers, in the order the cells were first used (a Python dictionary), the identifiers in the order they were added.
#[derive(Debug, Clone, Default)]
pub struct CellIndex {
    slots: HashMap<Cell, usize, FxBuild>,
    buckets: Vec<(Cell, Vec<i64>)>,
}

impl CellIndex {
    pub fn new() -> CellIndex {
        CellIndex::default()
    }

    pub fn add(&mut self, ident: i64, cells: &[Cell]) {
        for cell in cells {
            let slot = *self.slots.entry(*cell).or_insert_with(|| {
                self.buckets.push((*cell, Vec::new()));
                self.buckets.len() - 1
            });
            self.buckets[slot].1.push(ident);
        }
    }

    /// `lookup(cells)`: the identifiers of the cells, each once, ascending.
    pub fn lookup(&self, cells: &[Cell]) -> Vec<i64> {
        let mut found: Vec<i64> = Vec::new();
        for cell in cells {
            if let Some(slot) = self.slots.get(cell) {
                found.extend_from_slice(&self.buckets[*slot].1);
            }
        }
        found.sort_unstable();
        found.dedup();
        found
    }

    /// `buckets` as the dictionary the oracle keeps: `(cell, identifiers)` in insertion order.
    pub fn buckets(&self) -> &[(Cell, Vec<i64>)] {
        &self.buckets
    }

    /// A bucket of a table the harness restores (`buckets` of the oracle's index, in its order).
    pub fn restore(&mut self, cell: Cell, idents: Vec<i64>) {
        let slot = self.buckets.len();
        self.slots.insert(cell, slot);
        self.buckets.push((cell, idents));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frac(numerator: i64, denominator: i64) -> Rat {
        Rat::new(IBig::from(numerator), IBig::from(denominator)).unwrap()
    }

    #[test]
    fn floor_and_ceiling_go_toward_minus_infinity_not_toward_zero() {
        for (numerator, denominator, floor, ceil) in [(-1, 2, -1, 0), (-7, 3, -3, -2), (7, 3, 2, 3), (-6, 3, -2, -2), (0, 5, 0, 0), (-1, 1000, -1, 0), (13, 4, 3, 4), (-13, 4, -4, -3)] {
            let value = frac(numerator, denominator);
            assert_eq!(floor_of(&value), IBig::from(floor), "floor({numerator}/{denominator})");
            assert_eq!(ceil_of(&value), IBig::from(ceil), "ceil({numerator}/{denominator})");
        }
        let huge = Rat::new(IBig::from(-1) * (IBig::from(1) << 200) - IBig::from(1), IBig::from(1) << 100).unwrap();
        assert_eq!(floor_of(&huge), -(IBig::from(1) << 100) - IBig::ONE);
        assert_eq!(ceil_of(&huge), -(IBig::from(1) << 100));
    }

    #[test]
    fn the_integer_indexes_equal_the_rational_ones_on_negative_origins() {
        for (x_min, cell) in [(-37, 4), (-5, 1), (-64, 8), (11, 2), (0, 16)] {
            let grid = CellGrid { x_min, y_min: x_min, x_max: x_min + 200, y_max: x_min + 200, cell };
            for value in (-300..300).step_by(7) {
                assert_eq!(grid.low_index_int(value, x_min), grid.low_index(&Rat::from_i64(value), x_min));
                assert_eq!(grid.high_index_int(value, x_min), grid.high_index(&Rat::from_i64(value), x_min));
            }
        }
    }

    #[test]
    fn a_box_of_integers_marks_the_same_cells_either_way_and_closed_cells_take_both_sides_of_a_border() {
        let grid = CellGrid { x_min: -10, y_min: -10, x_max: 10, y_max: 10, cell: 4 };
        let by_rat = grid.box_cells(&Rat::from_i64(-2), &Rat::from_i64(2), &Rat::from_i64(-2), &Rat::from_i64(2));
        assert_eq!(by_rat, grid.box_cells_int(-2, 2, -2, 2));
        // x = -2 is 8 from the origin: the border of the cells 1 and 2, so both are marked
        assert!(by_rat.contains(&(1, 1)) && by_rat.contains(&(2, 2)));
        assert!(grid.box_cells_int(100, 120, 0, 0).is_empty());
    }

    #[test]
    fn covering_picks_the_smallest_power_of_two_cell() {
        let grid = CellGrid::covering((0, 0, 100, 100), 16).unwrap();
        assert!((100 / grid.cell + 1) * (100 / grid.cell + 1) <= 16);
        assert!(grid.cell.count_ones() == 1);
        assert!(grid.cell == 1 || (100 / (grid.cell / 2) + 1) * (100 / (grid.cell / 2) + 1) > 16);
        assert!(matches!(CellGrid::covering((1, 0, 0, 5), 4), Err(SkelError::CellGridRejected(_))));
    }

    #[test]
    fn a_vertical_a_horizontal_and_a_slanted_line_mark_the_cells_they_cross() {
        let grid = CellGrid::covering((0, 0, 16, 16), 25).unwrap();
        assert_eq!(grid.cell, 4);
        // x = 8: the border of the columns 1 and 2
        let vertical = grid.line_cells(1, 0, 8, None);
        assert!(vertical.contains(&(1, 0)) && vertical.contains(&(2, 3)) && !vertical.contains(&(0, 0)));
        let horizontal = grid.line_cells(0, 1, 6, None);
        assert!(horizontal.iter().all(|(_, row)| *row == 1) && horizontal.len() == 5);
        let slanted = grid.line_cells(1, -1, 0, None);
        assert!(slanted.contains(&(0, 0)) && slanted.contains(&(4, 4)));
        // a segment has its own window
        let segment = grid.segment_cells((0, 0), (4, 0));
        assert!(segment.iter().all(|(column, _)| *column <= 1));
    }

    #[test]
    fn the_index_keeps_insertion_order_and_looks_up_sorted_without_repeats() {
        let mut index = CellIndex::new();
        index.add(5, &[(1, 1), (0, 0)]);
        index.add(3, &[(0, 0), (2, 2)]);
        assert_eq!(index.buckets().iter().map(|(cell, _)| *cell).collect::<Vec<_>>(), vec![(1, 1), (0, 0), (2, 2)]);
        assert_eq!(index.lookup(&[(0, 0), (1, 1), (9, 9)]), vec![3, 5]);
    }
}
