//! The polygon the skeleton is built over (`wavefront/polygon.py::PolygonV1`), as the builder reads it: loops of lattice points with a squared speed per
//! edge, and the hidden supports of the vertex fans.
//!
//! Only what the wavefront reads is here: `loops`, `edges()`, `fan_edges()`, `vertex_count` and the reflex flags. The validation of `LoopV1` (distinct points, a
//! non-zero area, normalised speeds) happens where the polygon is made (the bridge), outside the unit; the port takes what it is given. The one condition it
//! adds is the machine range: a coordinate is below `2^60` in magnitude, so that every edge direction is below `2^61` (a [`SupportLine`] coefficient is
//! below `2^62`) and every determinant of three lattice points fits `i128`; a polygon beyond it is a named `Unsupported`, never a wrap.

use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;

use crate::error::{SkelError, SkelResult};
use crate::line::SupportLine;

pub type Point = (i64, i64);

/// `|x|, |y| < COORDINATE_LIMIT` for every lattice point of a polygon.
pub const COORDINATE_LIMIT: i64 = 1 << 60;

/// `LoopV1`: `points` and `edge_speeds_squared` (the default `|d|^2` already spelt out).
#[derive(Debug, Clone, PartialEq)]
pub struct Loop {
    pub points: Vec<Point>,
    pub speeds: Vec<Rat>,
}

/// `FanSupportV1`: the normal of a hidden support through the vertex and its squared speed.
#[derive(Debug, Clone, PartialEq)]
pub struct FanSupport {
    pub normal: Point,
    pub speed: Rat,
}

/// `VertexFanV1`: the hidden supports at one vertex, from the incoming edge to the outgoing one.
#[derive(Debug, Clone, PartialEq)]
pub struct VertexFan {
    pub point: Point,
    pub supports: Vec<FanSupport>,
}

/// `(start, end, q)` of an edge of a loop.
pub type Edge<'a> = (Point, Point, &'a Rat);

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    pub loops: Vec<Loop>,
    pub fans: Vec<VertexFan>,
}

/// `orient2d(a, b, c)`: the sign of the determinant of the turn `a -> b -> c` (positive: counter-clockwise).
pub fn orient2d(a: Point, b: Point, c: Point) -> i8 {
    let value = (i128::from(b.0) - i128::from(a.0)) * (i128::from(c.1) - i128::from(a.1)) - (i128::from(b.1) - i128::from(a.1)) * (i128::from(c.0) - i128::from(a.0));
    (value > 0) as i8 - (value < 0) as i8
}

impl Loop {
    /// `LoopV1.reflex_flags()`: vertex `i` is the joint of edges `i - 1` and `i`; it is reflex when the turn is clockwise.
    pub fn reflex_flags(&self) -> Vec<bool> {
        let size = self.points.len();
        (0..size).map(|index| orient2d(self.points[(index + size - 1) % size], self.points[index], self.points[(index + 1) % size]) < 0).collect()
    }
}

impl Polygon {
    /// A polygon after the range check (see the module note) and the shape check the oracle's constructors make before the unit runs.
    pub fn new(loops: Vec<Loop>, fans: Vec<VertexFan>) -> SkelResult<Polygon> {
        for each in &loops {
            if each.speeds.len() != each.points.len() || each.points.len() < 3 {
                return Err(SkelError::Unsupported("a loop has fewer than three points or a speed count that does not match its edges".to_string()));
            }
            for point in &each.points {
                if point.0.abs() >= COORDINATE_LIMIT || point.1.abs() >= COORDINATE_LIMIT {
                    return Err(SkelError::Unsupported(format!("the lattice point {point:?} is beyond the machine range of the port")));
                }
            }
        }
        for fan in &fans {
            if fan.point.0.abs() >= COORDINATE_LIMIT || fan.point.1.abs() >= COORDINATE_LIMIT {
                return Err(SkelError::Unsupported(format!("the fan vertex {:?} is beyond the machine range of the port", fan.point)));
            }
        }
        Ok(Polygon { loops, fans })
    }

    /// `PolygonV1.vertex_count`.
    pub fn vertex_count(&self) -> usize {
        self.loops.iter().map(|each| each.points.len()).sum()
    }

    /// `PolygonV1.edges()`: every loop's edges in walking order, with their squared speeds.
    pub fn edges(&self) -> Vec<Edge<'_>> {
        let mut records = Vec::with_capacity(self.vertex_count());
        for each in &self.loops {
            let size = each.points.len();
            for index in 0..size {
                records.push((each.points[index], each.points[(index + 1) % size], &each.speeds[index]));
            }
        }
        records
    }

    /// `PolygonV1.fan_edges()`: `(vertex, ordinal from one, support line)` of every hidden support, the fans taken in the order of their points (a stable sort).
    /// The line's identity is not the oracle's (it makes a fresh object on every call); nothing here is keyed by it.
    pub fn fan_edges(&self) -> SkelResult<Vec<(Point, u32, SupportLine)>> {
        let mut fans: Vec<&VertexFan> = self.fans.iter().collect();
        fans.sort_by_key(|fan| fan.point);
        let mut records = Vec::new();
        for fan in fans {
            for (ordinal, support) in fan.supports.iter().enumerate() {
                let constant = i128::from(support.normal.0) * i128::from(fan.point.0) + i128::from(support.normal.1) * i128::from(fan.point.1);
                records.push((fan.point, ordinal as u32 + 1, SupportLine::new(support.normal.0, support.normal.1, constant, support.speed.clone(), 0)?));
            }
        }
        Ok(records)
    }

    /// `fan_at(point)`: the fan of that vertex, by a linear search.
    pub fn fan_at(&self, point: Point) -> Option<&VertexFan> {
        self.fans.iter().find(|fan| fan.point == point)
    }

    /// `polygon_box(polygon)`: `(x_min, y_min, x_max, y_max)` of every loop point.
    pub fn bounding_box(&self) -> Option<(i64, i64, i64, i64)> {
        let mut points = self.loops.iter().flat_map(|each| each.points.iter());
        let first = points.next()?;
        Some(points.fold((first.0, first.1, first.0, first.1), |found, point| (found.0.min(point.0), found.1.min(point.1), found.2.max(point.0), found.3.max(point.1))))
    }
}

/// `|d|^2` of an edge as the rational the oracle's default speed is.
pub fn unit_speed_squared(start: Point, end: Point) -> Rat {
    let (dx, dy) = (i128::from(end.0) - i128::from(start.0), i128::from(end.1) - i128::from(start.1));
    Rat::from_int(IBig::from(dx * dx + dy * dy))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square() -> Loop {
        let points = vec![(0, 0), (4, 0), (4, 4), (0, 4)];
        let speeds = (0..4).map(|index| unit_speed_squared(points[index], points[(index + 1) % 4])).collect();
        Loop { points, speeds }
    }

    #[test]
    fn a_convex_loop_has_no_reflex_vertex_and_a_notch_has_one() {
        assert_eq!(square().reflex_flags(), vec![false; 4]);
        let ell = Loop { points: vec![(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)], speeds: vec![Rat::one(); 6] };
        assert_eq!(ell.reflex_flags(), vec![false, false, false, true, false, false]);
    }

    #[test]
    fn edges_and_fan_edges_come_in_the_oracles_order() {
        let polygon = Polygon::new(
            vec![square()],
            vec![
                VertexFan { point: (4, 4), supports: vec![FanSupport { normal: (1, 1), speed: Rat::from_i64(2) }] },
                VertexFan { point: (0, 0), supports: vec![FanSupport { normal: (1, 0), speed: Rat::one() }, FanSupport { normal: (0, 1), speed: Rat::one() }] },
            ],
        )
        .unwrap();
        assert_eq!(polygon.vertex_count(), 4);
        assert_eq!(polygon.edges()[1].0, (4, 0));
        let fans = polygon.fan_edges().unwrap();
        assert_eq!(fans.iter().map(|(point, ordinal, _)| (*point, *ordinal)).collect::<Vec<_>>(), vec![((0, 0), 1), ((0, 0), 2), ((4, 4), 1)]);
        assert_eq!(fans[2].2.c, 8);
        assert_eq!(polygon.bounding_box(), Some((0, 0, 4, 4)));
    }

    #[test]
    fn a_coordinate_beyond_the_range_is_unsupported_not_wrong() {
        let loop_ = Loop { points: vec![(0, 0), (1 << 60, 0), (0, 1)], speeds: vec![Rat::one(); 3] };
        assert!(matches!(Polygon::new(vec![loop_], Vec::new()), Err(SkelError::Unsupported(_))));
    }
}
