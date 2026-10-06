//! Hand-checked cases of the whole `clip_geometry` port. The bit-for-bit equality with the Python oracle (field and
//! synthetic corpora, cap sweeps, fuzz, both interpreters) is tested from Python (`tests/test_native_clip_geometry.py`);
//! here is one golden cut read off the oracle, the refusals the port names itself, and the shape of the profile table.

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_clip::emit::Law;
use cftuv_clip::error::ClipError;
use cftuv_clip::geometry::{clip_geometry, ClipInput};
use cftuv_clip::plane::{ChartPoint, Plane, Triangle};
use cftuv_clip::point::Point;
use cftuv_clip::profile::{Phase, PHASES};
use cftuv_clip::pyemu::PyVersion;
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::{IBig, UBig};
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};

fn rat(value: i64) -> Rat {
    Rat::from_i64(value)
}

fn point(x: i64, y: i64) -> Point {
    (SqrtSum::rational(&rat(x)), SqrtSum::rational(&rat(y)))
}

struct World {
    memory: CanonMemory,
    budget: WorkBudget,
    counts: SignCounts,
    products: ProductMemo,
}

impl World {
    fn new(budget: WorkBudget) -> World {
        World { memory: CanonMemory::new(), budget, counts: SignCounts::default(), products: ProductMemo::new() }
    }

    fn ctx(&mut self) -> ExactCtx<'_> {
        ExactCtx { memory: &mut self.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.products }
    }
}

/// `native_clip_generated.lift_square()`: an 8 x 8 square of the map (scale 4, flat) cut by the diagonal `(0,0)-(8,8)`.
fn square() -> Plane {
    let quarter = |value: i64| Rat::new(IBig::from(value), IBig::from(4)).unwrap();
    let triangle = |name: &str, chart: [(i64, i64); 3]| Triangle {
        name: name.to_string(),
        chart: chart.map(|(x, y)| -> ChartPoint { (rat(x), rat(y)) }),
        corners: chart.map(|(x, y)| [quarter(x), quarter(y), rat(0)]),
        twice_area: rat(64),
        bbox: [-5e-324, 8.000000000000002, -5e-324, 8.000000000000002],
        normals: None,
        face: String::new(),
    };
    Plane::new(vec![triangle("t0", [(0, 0), (8, 0), (8, 8)]), triangle("t1", [(0, 0), (8, 8), (0, 8)])])
}

fn keys_of(count: usize) -> Vec<String> {
    (0..count).map(|index| format!("node:{index}")).collect()
}

struct Case {
    points: Vec<(String, Point)>,
    cycles: Vec<Vec<String>>,
    polygons: Vec<Vec<Vec<String>>>,
    seam: Vec<(String, String)>,
}

fn case(vertices: &[Point]) -> Case {
    let keys = keys_of(vertices.len());
    Case {
        points: keys.iter().cloned().zip(vertices.iter().cloned()).collect(),
        cycles: vec![keys.clone()],
        polygons: vec![vec![keys]],
        seam: Vec::new(),
    }
}

fn input(case: &Case) -> ClipInput<'_> {
    ClipInput { points: &case.points, cycles: &case.cycles, polygons: &case.polygons, law: Law::PlanarPolygons, seam: &case.seam, fans: None, flows: None, by_faces: false }
}

fn counter(clipped: &cftuv_clip::emit::Clipped, name: &str) -> UBig {
    clipped.counters.iter().find(|(known, _)| *known == name).map(|(_, value)| value.clone()).expect("a counter of that name")
}

#[test]
fn a_rectangle_over_the_diagonal_is_cut_into_two_faces_with_one_new_vertex() {
    let plane = square();
    let case = case(&[point(2, 3), point(7, 3), point(7, 5), point(2, 5)]);
    let mut world = World::new(WorkBudget::unlimited());
    let run = clip_geometry(&mut world.ctx(), PyVersion::V313, &plane, &input(&case));
    let clipped = run.result.expect("the oracle cuts this rectangle");
    let faces: Vec<Vec<Vec<&str>>> = clipped.polygons.iter().map(|face| face.iter().map(|keys| keys.iter().map(|key| &**key).collect()).collect()).collect();
    assert_eq!(faces, vec![vec![vec!["node:0", "node:1", "node:2", "clip:0"], vec!["node:0", "clip:0", "node:3"]]]);
    assert_eq!(clipped.points.len(), 1);
    assert_eq!(&*clipped.points[0].0, "clip:0");
    assert_eq!(clipped.points[0].1 .0, SqrtSum::rational(&rat(5)));
    assert_eq!(clipped.points[0].1 .1, SqrtSum::rational(&rat(5)));
    assert_eq!(clipped.lifted.len(), 1);
    assert_eq!(clipped.lifted[0].1.position, [1.25, 1.25, 0.0]);
    assert_eq!(clipped.lifted[0].1.triangle, "t0");
    assert!(run.writes.is_empty(), "the triangles carry no offset normals");
    assert_eq!(counter(&clipped, "MATERIALIZE_CLIP_PREDICATES"), UBig::from(26u8));
    assert_eq!(counter(&clipped, "MATERIALIZE_CLIP_DIVISIONS"), UBig::from(4u8));
    assert_eq!(counter(&clipped, "MATERIALIZE_CLIP_NODE_SIGNS_ZEROED_BY_EDGE_GAP"), UBig::from(2u8));
    assert_eq!(counter(&clipped, "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_NANOMETRES"), UBig::from(176_776_696u64));
    assert_eq!(counter(&clipped, "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_MILLICELLS"), UBig::from(708u32));
    assert_eq!(
        clipped.note,
        "clip_vertices=1 (at_source_vertices=0) refined_edges=1 faces_in_one_triangle=0 faces_cut=1 (by_ears=0) pieces=2 merged_groups=0 \
         kept_separate_groups=0 overhang_faces=0 boundary_mismatch_faces=0 seam_crossings_suppressed_faces=0 off_corner_source_vertex_faces=0 \
         predicates=26 divisions=4"
    );
    assert_eq!(clipped.counters.len(), 24, "16 stage + 5 snap + 3 node-gap counters, no diagonal ones by triangles");
}

#[test]
fn the_same_input_is_the_same_answer_on_a_second_run_and_in_either_interpreter_model() {
    let plane = square();
    let case = case(&[point(2, 3), point(7, 3), point(7, 5), point(2, 5)]);
    let mut notes = Vec::new();
    for version in [PyVersion::V311, PyVersion::V313, PyVersion::V313] {
        let mut world = World::new(WorkBudget::unlimited());
        notes.push(clip_geometry(&mut world.ctx(), version, &plane, &input(&case)).result.expect("a cut").note);
    }
    assert_eq!(notes[0], notes[1], "no sort of two or more equal-keyed nodes here: the interpreter does not matter");
    assert_eq!(notes[1], notes[2]);
}

#[test]
fn a_key_that_is_no_vertex_of_the_domain_is_a_named_missing_key() {
    let plane = square();
    let mut case = case(&[point(2, 3), point(7, 3), point(7, 5)]);
    case.polygons = vec![vec![vec!["node:0".to_string(), "node:1".to_string(), "node:9".to_string()]]];
    let mut world = World::new(WorkBudget::unlimited());
    let run = clip_geometry(&mut world.ctx(), PyVersion::V313, &plane, &input(&case));
    assert_eq!(run.result.err(), Some(ClipError::MissingKey("node:9".to_string())));
    assert!(run.writes.is_empty());
}

#[test]
fn a_rational_cut_spends_no_radical_work_so_a_cap_of_zero_does_not_stop_it() {
    let plane = square();
    let case = case(&[point(2, 3), point(7, 3), point(7, 5), point(2, 5)]);
    let mut world = World::new(WorkBudget::bounded(0));
    let run = clip_geometry(&mut world.ctx(), PyVersion::V313, &plane, &input(&case));
    // the budget counts radicals: a polygon of rational points pays none (the exhaustion paths are swept from Python, cap by cap)
    assert!(run.result.is_ok());
}

#[test]
fn an_interpreter_the_port_does_not_emulate_is_refused_by_name() {
    assert!(matches!(PyVersion::from_version(3, 12), Err(ClipError::Unsupported(_))));
}

#[test]
fn the_phase_table_is_in_the_order_of_its_enum() {
    for (index, (phase, name)) in PHASES.iter().enumerate() {
        assert_eq!(*phase as usize, index, "{name}");
    }
    assert_eq!(PHASES[Phase::Finish as usize].0, Phase::Finish);
    assert_eq!(PHASES.len(), Phase::Finish as usize + 1);
}
