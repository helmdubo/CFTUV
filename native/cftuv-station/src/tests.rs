//! Unit facts of the port; the bit-for-bit evidence is the corpus replay against the Python oracle (`tests/test_native_chain_station.py`).

use super::*;

#[test]
fn split_is_exact() {
    for x in [0.0f64, -0.0, 1.0, -1.5, 0.1, 3.0e-310, 1.0e300, 0.004999999888241291] {
        let (negative, m, e) = split(x);
        let back = (m as f64) * 2f64.powi(e);
        assert_eq!(if negative { -back } else { back }, if x == 0.0 { 0.0 } else { x });
        if m != 0 {
            assert_eq!(m % 2, 1);
        }
    }
    assert_eq!(scaled(0.75, 2), IBig::from(3));
    assert_eq!(scaled(-0.75, 3), IBig::from(-6));
}

/// A small deterministic generator (no dependency): the cases are the same on every run.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
}

fn three_points(a: [f64; 3], b: [f64; 3], p: [f64; 3], float_limit: f64) -> Input {
    Input {
        vertex_rank: vec![0, 1, 2],
        face_rank: Vec::new(),
        chain_rank: Vec::new(),
        full: true,
        source_positions: vec![(0, a), (1, b), (2, p)],
        surface_faces: Vec::new(),
        neighbours: Vec::new(),
        corner_vertices: Vec::new(),
        chains: Vec::new(),
        domain_chains: Vec::new(),
        budget: (IBig::from(1), IBig::from(200)),
        bend: (IBig::from(1745), IBig::from(1_000_000)),
        float_limit,
    }
}

/// The exact distance to a SEGMENT (both ends and the inside) agrees with the binary64 filter wherever the filter decides: points placed at the chord budget times
/// `1 +- 10^-k` from the line, before, inside and past the segment. With the filter switched off (`float_limit = NaN` abstains), only the exact path answers.
#[test]
fn the_exact_segment_distance_agrees_with_the_filter_wherever_the_filter_decides() {
    let mut rng = Lcg(7);
    let mut decided = 0;
    for _ in 0..4000 {
        let a = [rng.next() * 4.0 - 2.0, rng.next() * 4.0 - 2.0, rng.next() * 4.0 - 2.0];
        let length = [0.01, 0.3, 2.0, 7.0][(rng.next() * 4.0) as usize];
        let u = {
            let v = [rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5];
            let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            [v[0] / n, v[1] / n, v[2] / n]
        };
        let w = {
            let t = if u[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
            let c = [u[1] * t[2] - u[2] * t[1], u[2] * t[0] - u[0] * t[2], u[0] * t[1] - u[1] * t[0]];
            let n = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
            [c[0] / n, c[1] / n, c[2] / n]
        };
        let b = [a[0] + length * u[0], a[1] + length * u[1], a[2] + length * u[2]];
        let t = [-0.3, -1e-9, 0.0, 0.2, 0.5, 1.0, 1.0 + 1e-9, 1.4][(rng.next() * 8.0) as usize];
        let relative = [1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-7, 1e-9][(rng.next() * 7.0) as usize] * if rng.next() < 0.5 { -1.0 } else { 1.0 };
        let d = 0.005 * (1.0 + relative);
        let p = [a[0] + t * length * u[0] + d * w[0], a[1] + t * length * u[1] + d * w[1], a[2] + t * length * u[2] + d * w[2]];
        let exact_input = three_points(a, b, p, f64::NAN);
        let exact = Facts::new(&exact_input).unwrap().beyond_segment(0, 1, 2).unwrap();
        let limit = 1.0 / 40000.0;
        if let Some(fast) = beyond_by_floats(&a, &b, &p, limit) {
            decided += 1;
            assert_eq!(exact, fast, "a={a:?} b={b:?} p={p:?}");
        }
        let both_input = three_points(a, b, p, limit);
        assert_eq!(Facts::new(&both_input).unwrap().beyond_segment(0, 1, 2).unwrap(), exact);
    }
    assert!(decided > 1000, "{decided}");
}

fn quad_strip(offset: f64) -> Input {
    // vertices 0..=2 on the chain (x axis), 3..=5 above them: two quads of one patch glued along the transverse edge 1-4
    let positions = vec![
        (0, [0.0, 0.0, 0.0]),
        (1, [1.0, 0.0, 0.0]),
        (2, [2.0, 0.0, 0.0]),
        (3, [0.0, 1.0, 0.0]),
        (4, [1.0, 1.0, offset]),
        (5, [2.0, 1.0, 0.0]),
    ];
    Input {
        vertex_rank: (0..6).collect(),
        face_rank: vec![0, 1],
        chain_rank: vec![0],
        full: true,
        source_positions: positions,
        surface_faces: vec![SurfaceFace { id: 0, patch: 0, cycle: vec![0, 1, 4, 3] }, SurfaceFace { id: 1, patch: 0, cycle: vec![1, 2, 5, 4] }],
        neighbours: Vec::new(),
        corner_vertices: Vec::new(),
        chains: vec![ChainIn { id: 0, kind: ChainKind::DecalSource, closed: false, vertices: vec![0, 1, 2] }],
        domain_chains: vec![0],
        budget: (IBig::from(1), IBig::from(200)),
        bend: (IBig::from(1745), IBig::from(1_000_000)),
        float_limit: 1.0 / 40000.0,
    }
}

#[test]
fn flat_strip_frees_the_middle_vertex() {
    let answer = chain_station_plans(&quad_strip(0.0)).unwrap();
    assert_eq!(answer.plans.len(), 1);
    let station = &answer.plans[0].stations[0];
    assert_eq!((station.ordinal, station.disposition, station.reason), (1, Disposition::Free, Reason::TransverseEdgesInert));
    assert_eq!(station.pairs, vec![(0, 1)]);
}

#[test]
fn folded_strip_keeps_the_middle_vertex() {
    let answer = chain_station_plans(&quad_strip(0.25)).unwrap();
    let station = &answer.plans[0].stations[0];
    assert_eq!((station.disposition, station.reason), (Disposition::Required, Reason::Fold));
}

#[test]
fn corner_vertex_is_a_junction() {
    let mut input = quad_strip(0.0);
    input.corner_vertices = vec![1];
    let station = chain_station_plans(&input).unwrap().plans[0].stations[0].clone();
    assert_eq!((station.disposition, station.reason), (Disposition::Required, Reason::Junction));
}

#[test]
fn float_filter_is_sound_at_the_margin() {
    let limit = 1.0 / 40000.0;
    assert_eq!(beyond_by_floats(&[0.0; 3], &[1.0, 0.0, 0.0], &[0.5, 0.01, 0.0], limit), Some(true));
    assert_eq!(beyond_by_floats(&[0.0; 3], &[1.0, 0.0, 0.0], &[0.5, 0.001, 0.0], limit), Some(false));
    assert_eq!(beyond_by_floats(&[0.0; 3], &[1.0, 0.0, 0.0], &[0.5, 0.005, 0.0], limit), None);
}

#[test]
fn a_neighbour_face_without_positions_declines_where_the_oracle_raises() {
    let mut input = quad_strip(0.0);
    input.face_rank = vec![0, 1, 2];
    input.neighbours.push(NeighbourFace { id: 2, patch: 1, vertices: vec![0, 1, 2], positions: Vec::new() });
    assert!(chain_station_plans(&input).is_err());
}

#[test]
fn a_station_next_to_a_vertex_without_a_position_names_it() {
    // the neighbour face names more vertices than it has positions (zip truncates): the end of the chain has no position, the window reads it
    let mut input = quad_strip(0.0);
    input.vertex_rank = (0..7).collect();
    input.face_rank = vec![0, 1, 2];
    input.chains[0].vertices = vec![0, 1, 2, 6];
    input.surface_faces.push(SurfaceFace { id: 2, patch: 0, cycle: vec![2, 6, 5] });
    input.neighbours.push(NeighbourFace { id: 2, patch: 0, vertices: vec![2, 6, 5], positions: vec![[2.0, 0.0, 0.0]] });
    // the face 2 of the surface has no points (vertex 6 has no position), so the station at 2 names POSITIONS_UNAVAILABLE and nothing reads 6 through it
    let answer = chain_station_plans(&input).unwrap();
    assert_eq!(answer.plans[0].stations.iter().find(|item| item.ordinal == 2).unwrap().reason, Reason::PositionsUnavailable);
}

#[test]
fn a_face_without_all_positions_makes_its_station_name_it() {
    let mut input = quad_strip(0.0);
    input.source_positions.retain(|(vertex, _)| *vertex != 2);
    // vertex 2 has no position: the face 1 has no points, the station names POSITIONS_UNAVAILABLE and nothing reads the window
    let station = chain_station_plans(&input).unwrap().plans[0].stations[0].clone();
    assert_eq!(station.reason, Reason::PositionsUnavailable);
}
