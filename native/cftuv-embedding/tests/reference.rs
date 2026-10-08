//! The port against an independent transcription of the oracle (`_embedding.py`) on exact reduced rationals, over generated cases.
//!
//! The reference below follows the Python line by line (names as strings, groups in order of first sight, `Fraction`-like `Q`), shares no code with the port, and answers a vertex the positions
//! lack with `KeyError` the way the oracle does. The generator makes planar and spatial meshes on small lattices (so vertices coincide, edges collapse and cross after a snap), inconsistent
//! physical edges, corners with missing vertices, and numbers of every width: small integers, dyadics at both sides of the `i128` bound, denominators that are not powers of two, and
//! wide ones that need `IBig`.

use std::collections::HashMap;

use cftuv_embedding::relation::{relation3, NONE, OVERLAP, POINT};
use cftuv_embedding::{compute, After, Corner, Counts, Face, Failure, Input, Rational, Tier};
use dashu_int::ops::{DivEuclid, Gcd};
use dashu_int::{IBig, UBig};

// ---- exact rationals (reduced) ------------------------------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Eq)]
struct Q {
    n: IBig,
    d: IBig,
}

impl Q {
    fn new(n: IBig, d: IBig) -> Q {
        assert!(d != IBig::ZERO);
        let g = IBig::from(n.clone().gcd(&d));
        let (mut n, mut d) = if g == IBig::ZERO { (n, d) } else { (&n / &g, &d / &g) };
        if d < IBig::ZERO {
            n = -n;
            d = -d;
        }
        Q { n, d }
    }
    fn int(v: i64) -> Q {
        Q { n: IBig::from(v), d: IBig::ONE }
    }
    fn zero() -> Q {
        Q::int(0)
    }
    fn is_zero(&self) -> bool {
        self.n == IBig::ZERO
    }
    fn add(&self, o: &Q) -> Q {
        Q::new(&self.n * &o.d + &o.n * &self.d, &self.d * &o.d)
    }
    fn sub(&self, o: &Q) -> Q {
        Q::new(&self.n * &o.d - &o.n * &self.d, &self.d * &o.d)
    }
    fn mul(&self, o: &Q) -> Q {
        Q::new(&self.n * &o.n, &self.d * &o.d)
    }
    fn div(&self, o: &Q) -> Q {
        Q::new(&self.n * &o.d, &self.d * &o.n)
    }
}

impl PartialOrd for Q {
    fn partial_cmp(&self, o: &Q) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

impl Ord for Q {
    fn cmp(&self, o: &Q) -> std::cmp::Ordering {
        (&self.n * &o.d).cmp(&(&o.n * &self.d))
    }
}

type P3 = [Q; 3];

fn sub3(l: &P3, r: &P3) -> P3 {
    [l[0].sub(&r[0]), l[1].sub(&r[1]), l[2].sub(&r[2])]
}

fn cross3(l: &P3, r: &P3) -> P3 {
    [l[1].mul(&r[2]).sub(&l[2].mul(&r[1])), l[2].mul(&r[0]).sub(&l[0].mul(&r[2])), l[0].mul(&r[1]).sub(&l[1].mul(&r[0]))]
}

fn dot3(l: &P3, r: &P3) -> Q {
    l.iter().zip(r.iter()).fold(Q::zero(), |sum, (a, b)| sum.add(&a.mul(b)))
}

fn any(v: &P3) -> bool {
    v.iter().any(|x| !x.is_zero())
}

fn ref_relation3(a: &P3, b: &P3, c: &P3, d: &P3) -> u8 {
    let r = sub3(b, a);
    let s = sub3(d, c);
    let rxs = cross3(&r, &s);
    let offset = sub3(c, a);
    if !any(&rxs) {
        if any(&cross3(&offset, &r)) {
            return NONE;
        }
        let Some(axis) = r.iter().position(|x| !x.is_zero()) else {
            return if a == c || a == d { POINT } else { NONE };
        };
        let mut left = [a[axis].clone(), b[axis].clone()];
        left.sort();
        let mut right = [c[axis].clone(), d[axis].clone()];
        right.sort();
        let low = left[0].clone().max(right[0].clone());
        let high = left[1].clone().min(right[1].clone());
        return if low > high {
            NONE
        } else if low == high {
            POINT
        } else {
            OVERLAP
        };
    }
    if !dot3(&offset, &rxs).is_zero() {
        return NONE;
    }
    let axis = rxs.iter().position(|x| !x.is_zero()).unwrap();
    let t = cross3(&offset, &s)[axis].div(&rxs[axis]);
    let u = cross3(&offset, &r)[axis].div(&rxs[axis]);
    let (zero, one) = (Q::zero(), Q::int(1));
    if zero <= t && t <= one && zero <= u && u <= one {
        POINT
    } else {
        NONE
    }
}

// ---- the case and the reference ----------------------------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct Case {
    before: Vec<(String, P3)>,
    after: Option<Vec<(String, P3)>>,
    faces: Vec<(String, Vec<String>, Vec<String>)>,
    intended: Vec<[String; 3]>,
    unclassifiable: Vec<[String; 3]>,
}

#[derive(Debug, PartialEq, Eq)]
enum Answer {
    Counts(Counts),
    Inconsistent(String),
    KeyError,
}

#[derive(Clone)]
struct Occ {
    key: (String, usize, String, String, String),
    start: String,
    end: String,
}

fn ref_edges(case: &Case) -> Result<Vec<Occ>, String> {
    let mut groups: Vec<(String, Vec<Occ>)> = Vec::new();
    for (face, vertices, edges) in &case.faces {
        for (index, start) in vertices.iter().enumerate() {
            let end = &vertices[(index + 1) % vertices.len()];
            let occ = Occ { key: (face.clone(), index, edges[index].clone(), start.clone(), end.clone()), start: start.clone(), end: end.clone() };
            match groups.iter_mut().find(|(name, _)| *name == edges[index]) {
                Some((_, list)) => list.push(occ),
                None => groups.push((edges[index].clone(), vec![occ])),
            }
        }
    }
    for (name, list) in &groups {
        let mut ends: Vec<(String, String)> = list.iter().map(|o| if o.start <= o.end { (o.start.clone(), o.end.clone()) } else { (o.end.clone(), o.start.clone()) }).collect();
        ends.sort();
        ends.dedup();
        if ends.len() != 1 {
            return Err(name.clone());
        }
    }
    let mut out: Vec<Occ> = groups.into_iter().map(|(_, list)| list.into_iter().min_by(|x, y| x.key.cmp(&y.key)).unwrap()).collect();
    out.sort_by(|x, y| x.key.cmp(&y.key));
    Ok(out)
}

fn ref_compute(case: &Case) -> Answer {
    let before: HashMap<&str, &P3> = case.before.iter().map(|(n, p)| (n.as_str(), p)).collect();
    let after: HashMap<&str, &P3> = match &case.after {
        Some(rows) => rows.iter().map(|(n, p)| (n.as_str(), p)).collect(),
        None => before.clone(),
    };
    let mut names: Vec<&str> = before.keys().copied().collect();
    names.sort();
    let edges = match ref_edges(case) {
        Ok(edges) => edges,
        Err(name) => return Answer::Inconsistent(name),
    };
    let get = |map: &HashMap<&str, &P3>, name: &str| -> Option<P3> { map.get(name).map(|p| (*p).clone()) };
    let (mut coincident, mut pair_tests) = (0u64, 0u64);
    for (index, left) in names.iter().enumerate() {
        for right in &names[index + 1..] {
            pair_tests += 1;
            if before[left] != before[right] && after[left] == after[right] {
                coincident += 1;
            }
        }
    }
    let mut collapsed = 0u64;
    for edge in &edges {
        let (Some(bs), Some(be)) = (get(&before, &edge.start), get(&before, &edge.end)) else { return Answer::KeyError };
        let (Some(as_), Some(ae)) = (get(&after, &edge.start), get(&after, &edge.end)) else { return Answer::KeyError };
        if bs != be && as_ == ae {
            collapsed += 1;
        }
    }
    let mut fresh = 0u64;
    for (index, left) in edges.iter().enumerate() {
        for right in &edges[index + 1..] {
            if left.start == right.start || left.start == right.end || left.end == right.start || left.end == right.end {
                continue;
            }
            pair_tests += 1;
            let at = |map: &HashMap<&str, &P3>, edge: &Occ, which: bool| get(map, if which { &edge.start } else { &edge.end }).unwrap();
            let prior = ref_relation3(&at(&before, left, true), &at(&before, left, false), &at(&before, right, true), &at(&before, right, false));
            let snapped = ref_relation3(&at(&after, left, true), &at(&after, left, false), &at(&after, right, true), &at(&after, right, false));
            if prior == NONE && snapped != NONE {
                fresh += 1;
            }
        }
    }
    let degenerated = case
        .intended
        .iter()
        .filter(|corner| match (get(&after, &corner[0]), get(&after, &corner[1]), get(&after, &corner[2])) {
            (Some(p), Some(v), Some(f)) => !any(&cross3(&sub3(&p, &v), &sub3(&f, &v))),
            _ => true,
        })
        .count() as u64;
    let unchanged = case
        .unclassifiable
        .iter()
        .filter(|corner| corner.iter().all(|name| matches!((get(&before, name), get(&after, name)), (Some(b), Some(a)) if a == b)))
        .count() as u64;
    Answer::Counts(Counts {
        source_edge_count: edges.len() as u64,
        newly_coincident_vertex_pair_count: coincident,
        collapsed_nonzero_source_edge_count: collapsed,
        new_nonadjacent_edge_intersection_count: fresh,
        unchanged_unclassifiable_source_corner_count: unchanged,
        degenerated_intended_right_corner_count: degenerated,
        exact_pair_test_count: pair_tests,
    })
}

// ---- the port's input from a case (what the Python boundary does) ------------------------------------------------------------

fn rational(q: &Q) -> Rational {
    let (sign, magnitude) = q.d.clone().into_parts();
    assert!(sign == dashu_int::Sign::Positive);
    Rational::new(q.n.clone(), magnitude)
}

fn rpoint(p: &P3) -> [Rational; 3] {
    [rational(&p[0]), rational(&p[1]), rational(&p[2])]
}

fn to_input(case: &Case) -> Input {
    let mut rows: Vec<&(String, P3)> = case.before.iter().collect();
    rows.sort_by(|a, b| a.0.as_bytes().cmp(b.0.as_bytes()));
    let rank: HashMap<&str, u32> = rows.iter().enumerate().map(|(i, r)| (r.0.as_str(), i as u32)).collect();
    let mut unknown: HashMap<String, u32> = HashMap::new();
    let slot = |name: &str, unknown: &mut HashMap<String, u32>| -> u32 {
        match rank.get(name) {
            Some(r) => *r,
            None => {
                let next = rank.len() as u32 + unknown.len() as u32;
                *unknown.entry(name.to_string()).or_insert(next)
            }
        }
    };
    let mut edge_names: Vec<String> = Vec::new();
    let mut edge_slot: HashMap<String, u32> = HashMap::new();
    let faces = case
        .faces
        .iter()
        .map(|(key, vertices, edges)| Face {
            key: key.clone(),
            vertices: vertices.iter().map(|v| slot(v, &mut unknown)).collect(),
            edges: edges
                .iter()
                .map(|e| {
                    *edge_slot.entry(e.clone()).or_insert_with(|| {
                        edge_names.push(e.clone());
                        edge_names.len() as u32 - 1
                    })
                })
                .collect(),
        })
        .collect();
    let corners = |list: &Vec<[String; 3]>| -> Vec<Corner> { list.iter().map(|c| [0, 1, 2].map(|k| rank.get(c[k].as_str()).copied())).collect() };
    let after = match &case.after {
        None => After::Same,
        Some(rows_after) => {
            let by: HashMap<&str, &P3> = rows_after.iter().map(|(n, p)| (n.as_str(), p)).collect();
            After::Points(rows.iter().map(|r| rpoint(by[r.0.as_str()])).collect())
        }
    };
    Input {
        before: rows.iter().map(|r| rpoint(&r.1)).collect(),
        after,
        faces,
        edge_names,
        intended: corners(&case.intended),
        unclassifiable: corners(&case.unclassifiable),
    }
}

fn check(case: &Case) -> Option<cftuv_embedding::Report> {
    let input = to_input(case);
    let expected = ref_compute(case);
    let got = compute(&input);
    match (&expected, &got) {
        (Answer::Counts(want), Ok(report)) => assert_eq!(*want, report.counts, "counts differ for {case:#?}"),
        (Answer::Inconsistent(name), Err(Failure::InconsistentEndpoints { edge })) => assert_eq!(*name, input.edge_names[*edge as usize]),
        (Answer::KeyError, Err(Failure::MissingVertex)) => {}
        _ => panic!("reference {expected:?} but the port answered {got:?} for {case:#?}"),
    }
    got.ok()
}

// ---- the generator ------------------------------------------------------------------------------------------------------------

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn chance(&mut self, percent: u64) -> bool {
        self.below(100) < percent
    }
}

/// How the lattice coordinate `i` becomes a number: the width classes of the port.
#[derive(Clone, Copy, Debug)]
enum Width {
    Small,
    Dyadic(u32),
    Thirds,
    Wide(u32),
}

fn number(rng: &mut Rng, lattice: i64, width: Width) -> Q {
    match width {
        Width::Small => Q::int(lattice),
        Width::Dyadic(shift) => Q::new(IBig::from(lattice) << shift as usize, IBig::ONE << (shift as usize / 2)),
        Width::Thirds => Q::new(IBig::from(lattice), IBig::from([1, 3, 7, 10][rng.below(4) as usize])),
        Width::Wide(shift) => Q::new((IBig::from(lattice) << shift as usize) + IBig::from(rng.below(5) as i64), IBig::ONE << (shift as usize + 3)),
    }
}

fn snap(q: &Q, scale: i64) -> Q {
    // floor(x * scale + 1/2) / scale
    let scaled = q.mul(&Q::int(scale)).add(&Q::new(IBig::ONE, IBig::from(2)));
    let floor = &scaled.n.clone().div_euclid(scaled.d.clone());
    Q::new(floor.clone(), IBig::from(scale))
}

fn random_case(rng: &mut Rng) -> Case {
    let vertices = 3 + rng.below(9) as usize;
    let spatial = rng.chance(40);
    let range = [2i64, 3, 6, 12][rng.below(4) as usize];
    let width = match rng.below(8) {
        0 => Width::Dyadic(30 + rng.below(30) as u32),
        1 => Width::Thirds,
        2 => Width::Wide(60 + rng.below(80) as u32),
        3 => Width::Dyadic(rng.below(12) as u32),
        _ => Width::Small,
    };
    let name = |i: usize| format!("v{i:02}");
    let mut before = Vec::new();
    for i in 0..vertices {
        let mut coordinate = |flat: bool| {
            let lattice = if flat { 0 } else { rng.below(range as u64 * 2 + 1) as i64 - range };
            number(rng, lattice, width)
        };
        let point = [coordinate(false), coordinate(false), coordinate(!spatial)];
        before.push((name(i), point));
    }
    if rng.chance(30) {
        let (a, b) = (rng.below(vertices as u64) as usize, rng.below(vertices as u64) as usize);
        before[a].1 = before[b].1.clone();
    }
    let mut faces = Vec::new();
    let face_count = 1 + rng.below(6) as usize;
    for f in 0..face_count {
        let size = 3 + rng.below(3) as usize;
        let mut cycle: Vec<usize> = Vec::new();
        while cycle.len() < size {
            let v = rng.below(vertices as u64) as usize;
            if rng.chance(95) && cycle.contains(&v) {
                continue;
            }
            cycle.push(v);
        }
        let edges: Vec<String> = (0..size)
            .map(|i| {
                let (a, b) = (cycle[i], cycle[(i + 1) % size]);
                let (lo, hi) = (a.min(b), a.max(b));
                if rng.chance(2) { format!("e{lo}_{}", hi + 1) } else { format!("e{lo}_{hi}") }
            })
            .collect();
        let key = if rng.chance(5) { "f0".to_string() } else { format!("f{f}") };
        faces.push((key, cycle.iter().map(|v| if rng.chance(1) { "ghost".to_string() } else { name(*v) }).collect(), edges));
    }
    let after = match rng.below(5) {
        0 => None,
        1 => Some(before.clone()),
        _ => {
            let scale = [1i64, 2, 3, 5, 8, 100, 1 << 20, 1 << 44, (1 << 60) + 7][rng.below(9) as usize];
            let mut rows: Vec<(String, P3)> = before.iter().map(|(n, p)| (n.clone(), [snap(&p[0], scale), snap(&p[1], scale), snap(&p[2], scale)])).collect();
            if rng.chance(30) {
                let (a, b) = (rng.below(vertices as u64) as usize, rng.below(vertices as u64) as usize);
                rows[a].1 = rows[b].1.clone();
            }
            Some(rows)
        }
    };
    let corner = |rng: &mut Rng| -> [String; 3] { [0, 1, 2].map(|_| if rng.chance(3) { "nowhere".to_string() } else { name(rng.below(vertices as u64) as usize) }) };
    let intended = (0..rng.below(5)).map(|_| corner(rng)).collect();
    let unclassifiable = (0..rng.below(5)).map(|_| corner(rng)).collect();
    Case { before, after, faces, intended, unclassifiable }
}

// ---- the tests ---------------------------------------------------------------------------------------------------------------

#[test]
fn the_port_equals_the_reference_on_generated_cases() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let (mut counts, mut tiers, mut violating) = (0usize, [0usize; 4], [0usize; 3]);
    for _ in 0..6000 {
        let case = random_case(&mut rng);
        if let Some(report) = check(&case) {
            counts += 1;
            tiers[(report.before_tier == Tier::Big) as usize * 2 + (report.after_tier == Tier::Big) as usize] += 1;
            violating[0] += (report.counts.newly_coincident_vertex_pair_count > 0) as usize;
            violating[1] += (report.counts.collapsed_nonzero_source_edge_count > 0) as usize;
            violating[2] += (report.counts.new_nonadjacent_edge_intersection_count > 0) as usize;
        }
    }
    assert!(counts > 3000, "most generated cases are answered, not declined ({counts})");
    assert!(tiers.iter().all(|n| *n > 0), "every tier pair is exercised: {tiers:?}");
    assert!(violating.iter().all(|n| *n > 20), "every violation counter is non-zero often enough: {violating:?}");
}

#[test]
fn the_fixed_width_bound_holds_at_the_widest_coordinates() {
    // |x| < 2^40 on every axis, signs and magnitudes chosen so the boxes overlap and the relation runs in full on the i128 road (a debug build panics on an overflow).
    let top = (IBig::ONE << 40usize) - IBig::ONE;
    let mut rng = Rng(0xD1B5_4A32_D192_ED03);
    let mut fixed = 0;
    for _ in 0..400 {
        let pick = |rng: &mut Rng| -> Q {
            let sign = if rng.chance(50) { IBig::ONE } else { -IBig::ONE };
            let value = if rng.chance(60) { top.clone() } else { IBig::from(rng.below(1 << 20) as i64) };
            Q::new(sign * value, IBig::ONE)
        };
        let flat = rng.chance(50);
        let random_point = |rng: &mut Rng| -> P3 { [pick(rng), pick(rng), if flat { Q::zero() } else { pick(rng) }] };
        let before: Vec<(String, P3)> = (0..5).map(|i| (format!("v{i}"), random_point(&mut rng))).collect();
        let faces = vec![
            ("f".to_string(), vec!["v0".into(), "v1".into(), "v2".into()], vec!["a".into(), "b".into(), "c".into()]),
            ("g".to_string(), vec!["v3".into(), "v4".into(), "v1".into(), "v2".into()], vec!["d".into(), "e".into(), "b".into(), "f".into()]),
        ];
        // the snap moves vertex 0 onto vertex 3 (and sometimes 2 onto 4): segments that did not meet now do, and the relation of the ORIGINAL segments runs on the widest numbers
        let mut after: Vec<(String, P3)> = before.clone();
        after[0].1 = after[3].1.clone();
        if rng.chance(50) {
            after[2].1 = after[4].1.clone();
        }
        let after = Some(after);
        let case = Case { before, after, faces, intended: vec![], unclassifiable: vec![] };
        if let Some(report) = check(&case) {
            fixed += (report.before_tier == Tier::Fixed) as usize;
        }
    }
    assert!(fixed > 100, "the widest coordinates still run on the fixed road ({fixed})");
}

#[test]
fn relation3_on_the_widest_fixed_coordinates_equals_the_big_road_and_the_reference() {
    let top = (1i128 << 40) - 1;
    let mut rng = Rng(0x94D0_49BB_1331_11EB);
    let mut met = [0usize; 3];
    for round in 0..30000 {
        let pick = |rng: &mut Rng| -> i128 {
            let magnitude = match rng.below(4) {
                0 => top,
                1 => 0,
                2 => rng.below(1 << 20) as i128,
                _ => top - rng.below(3) as i128,
            };
            if rng.chance(50) { magnitude } else { -magnitude }
        };
        let flat = rng.chance(50);
        let point = |rng: &mut Rng| -> [i128; 3] { [pick(rng), pick(rng), if flat { 0 } else { pick(rng) }] };
        let (a, b, mut c, mut d) = (point(&mut rng), point(&mut rng), point(&mut rng), point(&mut rng));
        // coplanar and touching constructions: c on the line ab, or d = a
        match rng.below(4) {
            0 => c = [a[0] + (b[0] - a[0]) / 2 * 2, a[1] + (b[1] - a[1]) / 2 * 2, a[2] + (b[2] - a[2]) / 2 * 2].map(|x| x.clamp(-top, top)),
            1 => d = a,
            2 => c = b,
            _ => {}
        }
        let fixed = relation3(&a, &b, &c, &d);
        let big = |x: &[i128; 3]| -> [IBig; 3] { x.map(IBig::from) };
        assert_eq!(fixed, relation3(&big(&a), &big(&b), &big(&c), &big(&d)), "{a:?} {b:?} {c:?} {d:?}");
        if round % 20 == 0 {
            let q = |x: &[i128; 3]| -> P3 { x.map(|v| Q::new(IBig::from(v), IBig::ONE)) };
            assert_eq!(fixed, ref_relation3(&q(&a), &q(&b), &q(&c), &q(&d)), "{a:?} {b:?} {c:?} {d:?}");
        }
        met[fixed as usize] += 1;
    }
    assert!(met.iter().all(|n| *n > 50), "NONE, POINT and OVERLAP all occur at the widest coordinates: {met:?}");
}

#[test]
fn relation3_matches_the_reference_on_small_integer_segments() {
    let mut rng = Rng(0x2545_F491_4F6C_DD1D);
    let q = |v: i64| Q::int(v);
    for _ in 0..20000 {
        let mut p = || -> [i128; 3] { [rng.below(4) as i128 - 1, rng.below(4) as i128 - 1, rng.below(3) as i128 - 1] };
        let (a, b, c, d) = (p(), p(), p(), p());
        let as_q = |x: &[i128; 3]| -> P3 { [q(x[0] as i64), q(x[1] as i64), q(x[2] as i64)] };
        assert_eq!(relation3(&a, &b, &c, &d), ref_relation3(&as_q(&a), &as_q(&b), &as_q(&c), &as_q(&d)), "{a:?} {b:?} {c:?} {d:?}");
        let big = |x: &[i128; 3]| -> [IBig; 3] { [IBig::from(x[0]), IBig::from(x[1]), IBig::from(x[2])] };
        assert_eq!(relation3(&big(&a), &big(&b), &big(&c), &big(&d)), relation3(&a, &b, &c, &d));
    }
}

#[test]
fn the_zero_length_segment_quirk_of_the_oracle_is_kept() {
    // a == b is a point strictly inside c..d: the oracle says NONE (it only compares the point with the endpoints), and POINT when the roles are swapped.
    let point = [1i128, 0, 0];
    let (c, d) = ([0i128, 0, 0], [2i128, 0, 0]);
    assert_eq!(relation3(&point, &point, &c, &d), NONE);
    assert_eq!(relation3(&c, &d, &point, &point), POINT);
    assert_eq!(relation3(&c, &d, &c, &c), POINT);
}

#[test]
fn a_vertex_the_positions_lack_is_declined_and_an_inconsistent_edge_is_named_in_order_of_first_sight() {
    let p = |x: i64| -> P3 { [Q::int(x), Q::int(0), Q::int(0)] };
    let before = vec![("a".to_string(), p(0)), ("b".to_string(), p(1)), ("c".to_string(), p(2)), ("d".to_string(), p(3))];
    // edge "x" is first seen before edge "y"; "y" is inconsistent earlier in the walk than "x" is, the oracle names "x"
    let faces = vec![
        ("f0".to_string(), vec!["a".into(), "b".into(), "c".into()], vec!["x".into(), "y".into(), "z".into()]),
        ("f1".to_string(), vec!["a".into(), "d".into(), "c".into()], vec!["y".into(), "w".into(), "x".into()]),
    ];
    let case = Case { before: before.clone(), after: None, faces, intended: vec![], unclassifiable: vec![] };
    assert_eq!(ref_compute(&case), Answer::Inconsistent("x".to_string()));
    assert!(check(&case).is_none());
    let case = Case { before, after: None, faces: vec![("f".to_string(), vec!["a".into(), "b".into(), "ghost".into()], vec!["x".into(), "y".into(), "z".into()])], intended: vec![], unclassifiable: vec![] };
    assert_eq!(ref_compute(&case), Answer::KeyError);
    assert!(check(&case).is_none());
}

#[test]
fn non_reduced_and_mixed_denominators_are_values_not_representations() {
    let wide = |n: i64, d: u64| Rational::new(IBig::from(n), UBig::from(d));
    let input = |after: [i64; 4]| Input {
        before: vec![[wide(1, 2), wide(0, 1), wide(0, 1)], [wide(2, 4), wide(0, 3), wide(0, 1)], [wide(3, 6), wide(1, 1), wide(0, 1)], [wide(1, 1), wide(2, 5), wide(0, 1)]],
        after: After::Points((0..4).map(|i| [wide(after[i], 8), wide(0, 1), wide(0, 1)]).collect()),
        faces: vec![],
        edge_names: vec![],
        intended: vec![],
        unclassifiable: vec![[Some(0), Some(1), Some(2)]],
    };
    // vertices 0 and 1 hold 1/2 and 2/4: the same value before the snap, so they are not "newly" coincident whatever the snap does
    let report = compute(&input([4, 4, 4, 4])).unwrap();
    assert_eq!(report.counts.newly_coincident_vertex_pair_count, 5, "six pairs, one of them equal before");
    let report = compute(&input([4, 4, 5, 5])).unwrap();
    assert_eq!(report.counts.newly_coincident_vertex_pair_count, 1, "{{0,1}} were equal before, {{2,3}} are new");
    assert_eq!(report.counts.unchanged_unclassifiable_source_corner_count, 0, "vertex 2 moved");
    assert!(!report.unchanged);
    // the snap that moves nothing, written with other denominators: the same values
    let mut same = input([4, 4, 4, 4]);
    same.after = After::Points(vec![[wide(2, 4), wide(0, 9), wide(0, 2)], [wide(3, 6), wide(0, 1), wide(0, 1)], [wide(1, 2), wide(1, 1), wide(0, 1)], [wide(10, 10), wide(4, 10), wide(0, 1)]]);
    let report = compute(&same).unwrap();
    assert!(report.unchanged);
    assert_eq!((report.counts.newly_coincident_vertex_pair_count, report.counts.unchanged_unclassifiable_source_corner_count), (0, 1));
}

#[test]
fn the_bound_tests_run_with_overflow_checks_on() {
    // the i128 road is plain arithmetic whose safety is the bound `3 * 40 + 6 < 127`; these tests prove it only if an overflow would panic
    let top = std::hint::black_box(i128::MAX);
    assert!(std::panic::catch_unwind(|| top + std::hint::black_box(1)).is_err(), "run the tests in the test profile (overflow checks on)");
}
