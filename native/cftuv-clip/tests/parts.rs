//! Small exact checks of the first third of the clip port. The bit-for-bit equality with the Python oracle is
//! tested from Python (`tests/test_native_clip_parts.py`: corpora, targeted cases, budget sweeps); here are the
//! hand-computed cases that must stay true whatever the oracle says, and the refusals the port names itself.

use std::collections::HashSet;

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_clip::cells::{build_cells, CellKey, CellMemo, NOT_ONE_LOOP};
use cftuv_clip::edge::{cheap_sign, edge_constants};
use cftuv_clip::error::ClipError;
use cftuv_clip::faces::{doubled_shoelace, orientation, Pt};
use cftuv_clip::lift::blend;
use cftuv_clip::plane::{ChartPoint, Plane, Triangle};
use cftuv_clip::point::Point;
use cftuv_clip::pyemu::PyVersion;
use cftuv_clip::tessellate::{convex_quad_ring, has_right_turn, triangulate_exact};
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};

fn rat(value: i64) -> Rat {
    Rat::from_i64(value)
}

fn point(x: i64, y: i64) -> Point {
    (SqrtSum::rational(&rat(x)), SqrtSum::rational(&rat(y)))
}

fn refs(points: &[Point]) -> Vec<Pt<'_>> {
    points.iter().map(|point| (&point.0, &point.1)).collect()
}

struct World {
    memory: CanonMemory,
    budget: WorkBudget,
    counts: SignCounts,
    products: ProductMemo,
}

impl World {
    fn new() -> World {
        World { memory: CanonMemory::new(), budget: WorkBudget::unlimited(), counts: SignCounts::default(), products: ProductMemo::new() }
    }

    fn ctx(&mut self) -> ExactCtx<'_> {
        ExactCtx { memory: &mut self.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.products }
    }
}

fn triangle(name: &str, chart: [(i64, i64); 3], face: &str, height: i64) -> Triangle {
    let corner = |x: i64, y: i64| [Rat::new(IBig::from(x), IBig::from(100)).unwrap(), Rat::new(IBig::from(y), IBig::from(100)).unwrap(), rat(height * x * y)];
    let twice_area = {
        let [(ax, ay), (bx, by), (cx, cy)] = chart;
        rat((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    };
    Triangle {
        name: name.to_string(),
        chart: chart.map(|(x, y)| -> ChartPoint { (rat(x), rat(y)) }),
        corners: chart.map(|(x, y)| corner(x, y)),
        twice_area,
        bbox: [0.0, 8.0, 0.0, 8.0],
        normals: None,
        face: face.to_string(),
    }
}

#[test]
fn the_ears_of_a_square_and_of_a_concave_pentagon_follow_the_ear_clipping_order() {
    let mut world = World::new();
    let square = [point(0, 0), point(4, 0), point(4, 4), point(0, 4)];
    let found = triangulate_exact(&mut world.ctx(), &refs(&square)).unwrap().unwrap();
    assert_eq!(found, vec![[3, 0, 1], [1, 2, 3]], "the first ear of the list is taken, the rest closes the contour");
    // clockwise input: the ring is reversed, the triangles are still counter-clockwise
    let clockwise = [point(0, 4), point(4, 4), point(4, 0), point(0, 0)];
    assert_eq!(triangulate_exact(&mut world.ctx(), &refs(&clockwise)).unwrap().unwrap().len(), 2);
    let arrow = [point(0, 0), point(4, 2), point(8, 0), point(8, 8), point(0, 8)];
    let found = triangulate_exact(&mut world.ctx(), &refs(&arrow)).unwrap().unwrap();
    assert_eq!(found.len(), 3);
    let collinear = [point(0, 0), point(1, 0), point(2, 0)];
    assert_eq!(triangulate_exact(&mut world.ctx(), &refs(&collinear)).unwrap(), None, "zero area has no triangulation");
    assert_eq!(triangulate_exact(&mut world.ctx(), &refs(&square[..2])).unwrap(), None);
}

#[test]
fn a_quadrilateral_ring_needs_four_strictly_left_turns() {
    let mut world = World::new();
    let square = [point(0, 0), point(4, 0), point(4, 4), point(0, 4)];
    assert_eq!(convex_quad_ring(&mut world.ctx(), &refs(&square)).unwrap(), Some([0, 1, 2, 3]));
    let reversed = [point(0, 4), point(4, 4), point(4, 0), point(0, 0)];
    assert_eq!(convex_quad_ring(&mut world.ctx(), &refs(&reversed)).unwrap(), Some([3, 2, 1, 0]));
    let flat_angle = [point(0, 0), point(2, 0), point(4, 0), point(4, 4)];
    assert_eq!(convex_quad_ring(&mut world.ctx(), &refs(&flat_angle)).unwrap(), None, "a straight angle is not strictly convex");
    let dart = [point(0, 0), point(4, 0), point(1, 1), point(0, 4)];
    assert_eq!(convex_quad_ring(&mut world.ctx(), &refs(&dart)).unwrap(), None);
    assert!(has_right_turn(&mut world.ctx(), &refs(&dart), &[0, 1, 2, 3]).unwrap());
    assert!(!has_right_turn(&mut world.ctx(), &refs(&square), &[0, 1, 2, 3]).unwrap());
    assert_eq!(world.counts.total, 1, "only the exactly collinear triple reaches a sign: the binary64 filter never proves zero");
}

#[test]
fn the_exact_orientation_of_collinear_radical_points_is_a_counted_sign() {
    let mut world = World::new();
    let root2 = SqrtSum::from_terms(vec![cftuv_core::sqrt_sum::Term { radicand: 2u8.into(), coef: cftuv_core::rat::Coef::fraction(rat(1)) }]).unwrap();
    let origin = point(0, 0);
    let on_line = (root2.clone(), root2.clone());
    let twice = (root2.add(&root2), root2.add(&root2));
    let sign = orientation(&mut world.ctx(), (&origin.0, &origin.1), (&on_line.0, &on_line.1), (&twice.0, &twice.1)).unwrap();
    assert_eq!(sign, 0);
    assert_eq!(world.counts.closed_rational_zero, 1, "an exact zero is counted, the filter never proves zero");
}

#[test]
fn the_doubled_shoelace_of_a_square_and_of_short_lists() {
    let mut products = ProductMemo::new();
    let square = [point(0, 0), point(4, 0), point(4, 4), point(0, 4)];
    let area = doubled_shoelace(&mut products, &refs(&square));
    assert_eq!(area.as_rational().unwrap().value(), &rat(32));
    assert!(doubled_shoelace(&mut products, &refs(&square[..2])).is_zero());
    assert!(doubled_shoelace(&mut products, &[]).is_zero());
}

#[test]
fn cells_merge_a_convex_face_group_a_concave_one_and_name_what_is_neither() {
    let triangles = vec![
        triangle("a0", [(0, 0), (4, 0), (4, 4)], "quad", 1),
        triangle("a1", [(0, 0), (4, 4), (0, 4)], "quad", 1),
        triangle("b0", [(10, 0), (14, 0), (10, 4)], "apart", 0),
        triangle("b1", [(30, 30), (34, 30), (30, 34)], "apart", 0),
        triangle("c0", [(20, 0), (28, 0), (28, 4)], "", 0),
    ];
    let mut memo = CellMemo::new();
    let plan = build_cells(&triangles, &HashSet::new(), &mut memo).unwrap();
    assert_eq!(plan.cells.len(), 4, "the quad merges, the others stay triangles");
    assert_eq!(plan.cells[0].key, CellKey::Face("quad".into(), 0));
    assert_eq!(plan.cells[0].members, vec![0, 1]);
    assert!(plan.cells[0].hinge.is_some() && plan.cells[0].flat_square.is_none());
    assert_eq!(plan.unmergeable, vec![("apart".to_string(), NOT_ONE_LOOP)]);
    assert_eq!(memo.len(), 2, "one memo entry per multi-triangle face");
    // the cell asked to split stays two triangles, from the memo and without recomputation
    let split: HashSet<CellKey> = [CellKey::Face("quad".into(), 0)].into_iter().collect();
    let again = build_cells(&triangles, &split, &mut memo).unwrap();
    assert_eq!(again.cells.len(), 5);
    assert!(again.cells.iter().all(|cell| cell.members.len() == 1));
}

#[test]
fn the_edge_constants_refuse_fractions_and_far_coordinates_and_the_cheap_sign_is_exact_for_rationals() {
    let lattice: Vec<ChartPoint> = vec![(rat(0), rat(0)), (rat(4), rat(0)), (rat(0), rat(4))];
    let found = edge_constants(&lattice, 0).unwrap();
    assert_eq!((found.x0, found.y0, found.dx, found.dy), (0, 0, 4, 0));
    assert_eq!(found.tolerance, 4.0, "sqrt(1 * 16)");
    let half: Vec<ChartPoint> = vec![(Rat::new(IBig::ONE, IBig::from(2)).unwrap(), rat(0)), (rat(4), rat(0)), (rat(0), rat(4))];
    assert!(edge_constants(&half, 0).is_none());
    let far: Vec<ChartPoint> = vec![(rat(1 << 40), rat(0)), (rat(4), rat(0)), (rat(0), rat(4))];
    assert!(edge_constants(&far, 0).is_none());
    // above the edge (0,0)-(4,0): positive; on it: zero; far above a watched node: farther than the tolerance
    assert_eq!(cheap_sign(&point(1, 2), &found, false), (Some(1), false));
    assert_eq!(cheap_sign(&point(1, 0), &found, true), (Some(0), false));
    assert_eq!(cheap_sign(&point(1, 100), &found, true), (Some(1), true));
    assert_eq!(cheap_sign(&point(1, 100), &found, false), (Some(1), false));
}

#[test]
fn a_zero_blend_is_a_named_refusal_and_the_version_decides_the_sum() {
    let normals = [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 0.0, 0.0]];
    let refused = blend(PyVersion::V311, &[0.5, 0.5, 0.0], &normals);
    assert!(matches!(refused, Err(ClipError::Refusal { outcome: "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE", .. })));
    let ok = blend(PyVersion::V313, &[0.2, 0.3, 0.5], &[[0.0, 0.0, 1.0], [0.0, 0.6, 0.8], [0.6, 0.0, 0.8]]).unwrap();
    let length = (ok[0] * ok[0] + ok[1] * ok[1] + ok[2] * ok[2]).sqrt();
    assert!((length - 1.0).abs() < 1e-15);
}

#[test]
fn the_stretch_bound_of_a_planar_lattice_triangle_is_its_scale_squared() {
    // one cell = 1/100 m in the plane, so the largest singular value squared of the lift is (1/100)^2 = 1e-4
    let plane = Plane::new(vec![triangle("t", [(0, 0), (4, 0), (0, 4)], "", 0)]);
    let bound = plane.stretch_square(0).unwrap();
    let exact = Rat::new(IBig::ONE, IBig::from(10_000)).unwrap();
    assert!(bound >= exact, "an upper bound never undershoots");
    let slack = Rat::new(IBig::ONE, IBig::from(10_000_000)).unwrap();
    assert!(bound.sub(&exact) < slack, "and stays within the 2^-24 relative slack of the root");
}
