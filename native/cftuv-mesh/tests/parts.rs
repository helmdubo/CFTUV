//! Small exact checks of the mesh port. The bit-for-bit equality with the Python oracle is tested from Python (`tests/test_native_materialize_w1.py`: the seed, the generated contours, the field corpora);
//! here are the hand-computed cases that must stay true whatever the oracle says.

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_clip::faces::Pt;
use cftuv_clip::point::Point;
use cftuv_core::exact::ExactCtx;
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_mesh::partition::{plan_partition, verify_partition, REASON_NO_TRIANGULATION};
use cftuv_mesh::predicates::{contour_is_simple, convex_polygon_ring, counter_clockwise_ring, has_crossings, segments_cross, triangulate_from_apex};
use cftuv_mesh::tally::Tally;

fn point(x: i64, y: i64) -> Point {
    (SqrtSum::rational(&Rat::from_i64(x)), SqrtSum::rational(&Rat::from_i64(y)))
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

#[test]
fn the_tally_keeps_first_touch_order_and_the_zero_entries_of_a_counter() {
    let mut tally = Tally::new();
    tally.add("B", 0);
    tally.add("A", 2);
    tally.add("B", 1);
    tally.raise_to("M_MAX_MILLI_ALPHA", 5);
    tally.raise_to("M_MAX_MILLI_ALPHA", 3);
    assert_eq!(tally.get("never touched"), 0, "a missing name reads as zero and creates nothing");
    assert_eq!(tally.items().collect::<Vec<_>>(), vec![("B", 1), ("A", 2), ("M_MAX_MILLI_ALPHA", 5)]);
    let mut zero = Tally::new();
    zero.add("Z", 0);
    assert_eq!(zero.items().collect::<Vec<_>>(), vec![("Z", 0)], "`tally[name] += 0` creates the entry");
}

#[test]
fn rings_follow_the_sign_of_the_area_and_a_right_turn_is_not_convex() {
    let mut world = World::new();
    let square = [point(0, 0), point(4, 0), point(4, 4), point(0, 4)];
    let reversed = [point(0, 4), point(4, 4), point(4, 0), point(0, 0)];
    assert_eq!(counter_clockwise_ring(&mut world.ctx(), &refs(&square)).unwrap(), Some(vec![0, 1, 2, 3]));
    assert_eq!(counter_clockwise_ring(&mut world.ctx(), &refs(&reversed)).unwrap(), Some(vec![3, 2, 1, 0]));
    let flat = [point(0, 0), point(1, 0), point(2, 0), point(3, 0)];
    assert_eq!(counter_clockwise_ring(&mut world.ctx(), &refs(&flat)).unwrap(), None);
    let arrow = [point(0, 0), point(4, 0), point(2, 1), point(4, 4), point(0, 4)];
    assert_eq!(convex_polygon_ring(&mut world.ctx(), &refs(&arrow)).unwrap(), None);
    assert!(convex_polygon_ring(&mut world.ctx(), &refs(&square)).unwrap().is_some());
}

#[test]
fn a_bow_tie_crosses_itself_and_a_square_does_not() {
    let mut world = World::new();
    let bow = [point(0, 0), point(4, 4), point(4, 0), point(0, 4)];
    let square = [point(0, 0), point(4, 0), point(4, 4), point(0, 4)];
    assert!(has_crossings(&mut world.ctx(), &refs(&bow)).unwrap());
    assert!(!contour_is_simple(&mut world.ctx(), &refs(&bow)).unwrap());
    assert!(!has_crossings(&mut world.ctx(), &refs(&square)).unwrap());
    assert!(contour_is_simple(&mut world.ctx(), &refs(&square)).unwrap());
    let (a, b, c, d) = (point(0, 0), point(4, 4), point(4, 0), point(0, 4));
    assert!(segments_cross(&mut world.ctx(), (&a.0, &a.1), (&b.0, &b.1), (&c.0, &c.1), (&d.0, &d.1)).unwrap());
    // touching at an end is not a transversal crossing
    let f = point(8, 8);
    assert!(!segments_cross(&mut world.ctx(), (&a.0, &a.1), (&b.0, &b.1), (&b.0, &b.1), (&f.0, &f.1)).unwrap());
}

#[test]
fn the_fan_from_an_apex_is_counter_clockwise_and_refused_when_a_triangle_is_not_positive() {
    let mut world = World::new();
    let square = [point(0, 0), point(4, 0), point(4, 4), point(0, 4)];
    let fan = triangulate_from_apex(&mut world.ctx(), &refs(&square), 2).unwrap().expect("a convex contour is a star from any vertex");
    assert_eq!(fan, vec![[2, 3, 0], [2, 0, 1]]);
    let outside = triangulate_from_apex(&mut world.ctx(), &refs(&square), 9).unwrap();
    assert!(outside.is_none(), "an apex outside the contour has no fan");
}

#[test]
fn the_exact_partition_of_an_arrow_takes_one_diagonal_and_the_proof_accepts_it() {
    let mut world = World::new();
    // an arrowhead: one reflex vertex; a part is admissible when it is a triangle or has no right turn (convex)
    let arrow = [point(0, 0), point(6, 0), point(6, 3), point(3, 2), point(0, 3)];
    let points = refs(&arrow);
    let mut admissible = |ctx: &mut ExactCtx<'_>, verts: &[usize]| -> cftuv_mesh::error::MeshResult<bool> {
        let sub: Vec<Pt> = verts.iter().map(|index| points[*index]).collect();
        Ok(convex_polygon_ring(ctx, &sub)?.is_some())
    };
    let plan = plan_partition(&mut world.ctx(), &points, &mut admissible).unwrap().expect("a plan");
    assert!(plan.exact && plan.pieces.len() == 2, "{:?}", plan.pieces);
    assert_eq!(verify_partition(&mut world.ctx(), &points, &plan).unwrap(), None);
    let degenerate = [point(0, 0), point(1, 0), point(2, 0), point(3, 0)];
    let mut anything = |_: &mut ExactCtx<'_>, _: &[usize]| -> cftuv_mesh::error::MeshResult<bool> { Ok(true) };
    let refused = plan_partition(&mut world.ctx(), &refs(&degenerate), &mut anything).unwrap();
    assert_eq!(refused.err(), Some(REASON_NO_TRIANGULATION));
}
