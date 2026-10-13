//! The port against an independent transcription of the oracle (`source_grid.py`, `planar_metric.py`, the exact-plane branch) on exact reduced rationals, over generated patches.
//!
//! The reference follows the Python line by line with a `Fraction`-like `Q`, shares no code with the port (it does not use the integer lifting, the common denominators, the area-vector
//! orientation, nor the closed forms of the inverse and of the coordinates), and sees the same patch as the port does: positions as exact rationals, faces as cycles of ranks. The generator makes
//! patches in the planes the product meets (axis-aligned planes and slanted planes through lattice points), with authoring noise below half a grid cell (so the snap restores the relations, or
//! does not, and the port has to decide the same), with right angles, oblique angles and degenerate faces. Two outcomes are compared: the frame, number by number, or the kind of decline.

use cftuv_core::rat::Rat;
use cftuv_frame::model::{FaceIn, Input, Reason};
use cftuv_frame::resolve;
use dashu_int::ops::Gcd;
use dashu_int::IBig;

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
    fn neg(&self) -> Q {
        Q { n: -&self.n, d: self.d.clone() }
    }
    /// `floor(self)`
    fn floor(&self) -> IBig {
        let (q, r) = (&self.n / &self.d, &self.n % &self.d);
        if r < IBig::ZERO {
            q - IBig::ONE
        } else {
            q
        }
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

// ---- the reference -------------------------------------------------------------------------------------------------------------

#[derive(Debug)]
struct RefFrame {
    scale_exponent: u32,
    trials: Vec<(u32, u32, bool)>,
    intended: u32,
    extent: Q,
    lower: Q,
    normal: [IBig; 3],
    origin: P3,
    a: P3,
    b: P3,
    gram: [Q; 3],
    inverse: [Q; 3],
    coordinates: Vec<[Q; 2]>,
    sign: i8,
}

#[derive(Debug)]
enum Expected {
    Frame(Box<RefFrame>),
    Decline(Reason),
}

fn corners(faces: &[FaceIn]) -> Vec<[u32; 3]> {
    let mut out = Vec::new();
    for face in faces {
        let n = face.vertices.len();
        for i in 0..n {
            out.push([face.vertices[(i + n - 1) % n], face.vertices[i], face.vertices[(i + 1) % n]]);
        }
    }
    out
}

/// `_corner_pair`: `(cosine, squared sine)` or `None` for a degenerate corner.
fn corner_pair(positions: &[P3], corner: &[u32; 3]) -> Option<(Q, Q)> {
    let (p, v, f) = (&positions[corner[0] as usize], &positions[corner[1] as usize], &positions[corner[2] as usize]);
    let left = sub3(p, v);
    let right = sub3(f, v);
    let normal = cross3(&left, &right);
    let squared = dot3(&normal, &normal);
    if squared.is_zero() {
        return None;
    }
    Some((dot3(&left, &right), squared))
}

fn snap(positions: &[P3], scale: &Q) -> Vec<P3> {
    let half = Q::new(IBig::ONE, IBig::from(2));
    positions
        .iter()
        .map(|p| {
            let one = |x: &Q| Q::new(x.mul(scale).add(&half).floor(), IBig::ONE).div(scale);
            [one(&p[0]), one(&p[1]), one(&p[2])]
        })
        .collect()
}

fn reference(positions: &[P3], faces: &[FaceIn]) -> Expected {
    let axis_span = |axis: usize| {
        let high = positions.iter().map(|p| p[axis].clone()).max().unwrap();
        let low = positions.iter().map(|p| p[axis].clone()).min().unwrap();
        high.sub(&low)
    };
    let extent = (0..3).map(axis_span).max().unwrap();
    if extent <= Q::zero() {
        return Expected::Decline(Reason::Frame);
    }
    let error = Q::new(IBig::from(7), IBig::from(1_000_000));
    let lower = Q::int(2).mul(&error).mul(&extent);
    let upper = Q::new(IBig::ONE, IBig::from(100));
    if lower > upper {
        return Expected::Decline(Reason::Grid);
    }
    let mut scale: u128 = 1;
    while Q::new(IBig::ONE, IBig::from(scale)) >= lower {
        scale *= 2;
    }
    scale /= 2;
    if Q::new(IBig::ONE, IBig::from(scale)) > upper {
        return Expected::Decline(Reason::Grid);
    }
    // admissible scales, descending (finest first)
    let mut candidates = Vec::new();
    let mut s = scale;
    while Q::new(IBig::ONE, IBig::from(s)) <= upper {
        candidates.push(s);
        if s == 1 {
            break;
        }
        s /= 2;
    }
    // intended corners (on the unsnapped positions)
    let all = corners(faces);
    let bound = error.mul(&error);
    let mut intended = Vec::new();
    for corner in &all {
        let Some((cosine, squared)) = corner_pair(positions, corner) else { continue };
        if cosine.is_zero() {
            continue;
        }
        if cosine.mul(&cosine) <= bound.mul(&squared) {
            intended.push(*corner);
        }
    }
    let mut trials = Vec::new();
    let mut chosen = None;
    for scale in candidates {
        let q = Q::int(0).add(&Q::new(IBig::from(scale), IBig::ONE));
        let snapped = snap(positions, &q);
        let restored = intended.iter().filter(|corner| matches!(corner_pair(&snapped, corner), Some((c, _)) if c.is_zero())).count() as u32;
        let passed = restored as usize == intended.len();
        trials.push((scale.trailing_zeros(), restored, passed));
        if passed {
            chosen = Some((scale, snapped));
            break;
        }
    }
    let Some((scale, snapped)) = chosen else { return Expected::Decline(Reason::Grid) };
    // the plane: Newell, canonical primitive form
    let mut sum = [Q::zero(), Q::zero(), Q::zero()];
    for face in faces {
        let n = face.vertices.len();
        for i in 0..n {
            let head = &snapped[face.vertices[i] as usize];
            let tail = &snapped[face.vertices[(i + 1) % n] as usize];
            sum[0] = sum[0].add(&head[1].sub(&tail[1]).mul(&head[2].add(&tail[2])));
            sum[1] = sum[1].add(&head[2].sub(&tail[2]).mul(&head[0].add(&tail[0])));
            sum[2] = sum[2].add(&head[0].sub(&tail[0]).mul(&head[1].add(&tail[1])));
        }
    }
    let mut denominator = IBig::ONE;
    for item in &sum {
        let g = IBig::from(denominator.clone().gcd(&item.d));
        denominator = &denominator * &item.d / g;
    }
    let mut ints: Vec<IBig> = sum.iter().map(|item| item.n.clone() * (&denominator / &item.d)).collect();
    let mut divisor = IBig::ZERO;
    for item in &ints {
        divisor = if divisor == IBig::ZERO && *item == IBig::ZERO { IBig::ZERO } else { IBig::from(divisor.gcd(item)) };
    }
    if divisor == IBig::ZERO {
        return Expected::Decline(Reason::Plane);
    }
    ints = ints.iter().map(|item| item / &divisor).collect();
    if ints.iter().find(|item| **item != IBig::ZERO).unwrap() < &IBig::ZERO {
        ints = ints.iter().map(|item| -item).collect();
    }
    let normal = [ints[0].clone(), ints[1].clone(), ints[2].clone()];
    let normal_q: P3 = [Q::new(normal[0].clone(), IBig::ONE), Q::new(normal[1].clone(), IBig::ONE), Q::new(normal[2].clone(), IBig::ONE)];
    let anchor = &snapped[0];
    if snapped.iter().any(|p| !dot3(&sub3(p, anchor), &normal_q).is_zero()) {
        return Expected::Decline(Reason::Plane);
    }
    // the frame
    let origin = snapped[0].clone();
    let candidates: Vec<P3> = snapped.iter().skip(1).map(|p| sub3(p, &origin)).collect();
    let Some(first) = candidates.iter().find(|v| any(v)).cloned() else { return Expected::Decline(Reason::Frame) };
    let Some(second) = candidates.iter().find(|v| any(&cross3(&first, v))).cloned() else { return Expected::Decline(Reason::Frame) };
    let (g00, g01, g11) = (dot3(&first, &first), dot3(&first, &second), dot3(&second, &second));
    let determinant = g00.mul(&g11).sub(&g01.mul(&g01));
    if determinant <= Q::zero() {
        return Expected::Decline(Reason::Frame);
    }
    let inverse = [g11.div(&determinant), g01.neg().div(&determinant), g00.div(&determinant)];
    let mut coordinates = Vec::new();
    for position in &snapped {
        let relative = sub3(position, &origin);
        let (rhs_a, rhs_b) = (dot3(&first, &relative), dot3(&second, &relative));
        let u = inverse[0].mul(&rhs_a).add(&inverse[1].mul(&rhs_b));
        let v = inverse[1].mul(&rhs_a).add(&inverse[2].mul(&rhs_b));
        let rebuilt: P3 = [0, 1, 2].map(|k| origin[k].add(&first[k].mul(&u)).add(&second[k].mul(&v)));
        if &rebuilt != position {
            return Expected::Decline(Reason::Frame);
        }
        coordinates.push([u, v]);
    }
    let mut sign = 0i8;
    for face in faces {
        let cycle: Vec<&[Q; 2]> = face.vertices.iter().map(|v| &coordinates[*v as usize]).collect();
        let mut twice = Q::zero();
        for i in 0..cycle.len() {
            let next = cycle[(i + 1) % cycle.len()];
            twice = twice.add(&cycle[i][0].mul(&next[1]).sub(&cycle[i][1].mul(&next[0])));
        }
        let this = (twice > Q::zero()) as i8 - (twice < Q::zero()) as i8;
        if this != 0 {
            sign = this;
            break;
        }
    }
    if sign == 0 {
        return Expected::Decline(Reason::Frame);
    }
    Expected::Frame(Box::new(RefFrame {
        scale_exponent: scale.trailing_zeros(),
        trials,
        intended: intended.len() as u32,
        extent,
        lower,
        normal,
        origin,
        a: first,
        b: second,
        gram: [g00, g01, g11],
        inverse,
        coordinates,
        sign,
    }))
}

// ---- the generator -------------------------------------------------------------------------------------------------------------

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
    fn chance(&mut self, num: u64, den: u64) -> bool {
        self.below(den) < num
    }
}

/// A patch: a `width x height` lattice of cells in the `(s, t)` coordinates of a plane `origin + s * e1 + t * e2`, some cells removed, vertices numbered by the sort of their names.
struct Patch {
    positions: Vec<[f64; 3]>,
    faces: Vec<FaceIn>,
    edge_names: Vec<String>,
}

fn dyadic(numerator: i64, shift: u32) -> f64 {
    numerator as f64 / (1u64 << shift) as f64
}

fn generate(rng: &mut Rng) -> Patch {
    let width = 1 + rng.below(4) as i64;
    let height = 1 + rng.below(4) as i64;
    // the plane: axis-aligned most of the time, a slanted lattice plane otherwise
    let (e1, e2): ([i64; 3], [i64; 3]) = match rng.below(6) {
        0 => ([1, 0, 0], [0, 1, 0]),
        1 => ([1, 0, 0], [0, 0, 1]),
        2 => ([0, 1, 0], [0, 0, 1]),
        3 => ([1, 1, 0], [0, 0, 1]),
        4 => ([1, 0, 1], [0, 1, 0]),
        _ => ([1, 2, 2], [2, 1, -2]),
    };
    let cell: i64 = 1 + rng.below(5) as i64;
    let unit_shift = rng.below(3) as u32; // cell size in 2^-shift
    let offset = [rng.below(4000) as i64 - 2000, rng.below(4000) as i64 - 2000, rng.below(4000) as i64 - 2000];
    let noise_shift = 24 + rng.below(4) as u32; // the authoring noise: up to 1000 * 2^-(shift + 3), a few millionths of a cell
    let noisy = rng.chance(2, 3);
    let skew = rng.chance(1, 8);
    let mut names: Vec<(i64, i64)> = Vec::new();
    for s in 0..=width {
        for t in 0..=height {
            names.push((s, t));
        }
    }
    let mut removed = vec![false; (width * height) as usize];
    if width * height > 2 && rng.chance(1, 3) {
        removed[rng.below((width * height) as u64) as usize] = true;
    }
    let used_cells: Vec<(i64, i64)> = (0..width).flat_map(|s| (0..height).map(move |t| (s, t))).filter(|(s, t)| !removed[(s * height + t) as usize]).collect();
    if used_cells.is_empty() {
        return generate(rng);
    }
    let mut used = std::collections::BTreeSet::new();
    for (s, t) in &used_cells {
        for (ds, dt) in [(0, 0), (1, 0), (1, 1), (0, 1)] {
            used.insert((s + ds, t + dt));
        }
    }
    let ordered: Vec<(i64, i64)> = {
        let mut list: Vec<(i64, i64)> = used.iter().copied().collect();
        list.sort_by_key(|(s, t)| format!("v{s:03}-{t:03}"));
        list
    };
    let rank = |s: i64, t: i64| ordered.iter().position(|item| *item == (s, t)).unwrap() as u32;
    let mut positions = Vec::new();
    for (s, t) in &ordered {
        let mut p = [0.0f64; 3];
        for axis in 0..3 {
            let lattice = (s * cell * e1[axis] + t * cell * e2[axis]) * (1 << 10) + offset[axis] * (1 << (10 - unit_shift));
            let mut value = dyadic(lattice, 10 + unit_shift);
            if skew && axis == 0 && *s == width {
                value += dyadic(37, 14);
            }
            if noisy {
                let jitter = rng.below(2001) as i64 - 1000;
                value += dyadic(jitter, noise_shift + 3);
            }
            p[axis] = value;
        }
        positions.push(p);
    }
    let mut faces = Vec::new();
    let mut edge_names: Vec<String> = Vec::new();
    let edge_of = |a: u32, b: u32, names: &mut Vec<String>| -> u32 {
        let name = format!("e{}-{}", a.min(b), a.max(b));
        match names.iter().position(|item| *item == name) {
            Some(slot) => slot as u32,
            None => {
                names.push(name);
                names.len() as u32 - 1
            }
        }
    };
    for (s, t) in &used_cells {
        let mut cycle = vec![rank(*s, *t), rank(s + 1, *t), rank(s + 1, t + 1), rank(*s, t + 1)];
        if rng.chance(1, 5) {
            cycle.reverse();
        }
        let edges = (0..4).map(|i| edge_of(cycle[i], cycle[(i + 1) % 4], &mut edge_names)).collect();
        faces.push(FaceIn { key: format!("f{s:03}-{t:03}"), vertices: cycle, edges });
    }
    Patch { positions, faces, edge_names }
}

fn q_of(x: f64) -> Q {
    // an f64 of the generator is `n / 2^k`: read it exactly through repeated doubling
    let mut k = 0u32;
    let mut y = x;
    while y.fract() != 0.0 {
        y *= 2.0;
        k += 1;
        assert!(k < 1100);
    }
    Q::new(IBig::from(y as i128), IBig::from(1u128 << k.min(120)) << (k.saturating_sub(120) as usize))
}

fn equal(rat: &Rat, q: &Q, what: &str) {
    assert_eq!(rat.numerator(), &q.n, "{what}: numerator");
    assert_eq!(IBig::from(rat.denominator().clone()), q.d, "{what}: denominator");
}

#[test]
fn the_port_equals_the_reference_on_generated_patches() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let (mut frames, mut declined) = (0u32, std::collections::BTreeMap::<&'static str, u32>::new());
    let mut skipped = 0u32;
    let (mut searched, mut with_corners) = (0u32, 0u32);
    for case in 0..6000 {
        let patch = generate(&mut rng);
        let positions: Vec<P3> = patch.positions.iter().map(|p| [q_of(p[0]), q_of(p[1]), q_of(p[2])]).collect();
        let input = Input { positions: patch.positions.clone(), faces: patch.faces.clone(), edge_names: patch.edge_names.clone(), snaps_source: true };
        let got = resolve(&input);
        let want = reference(&positions, &patch.faces);
        match (got, want) {
            (Ok(frame), Expected::Frame(want)) => {
                frames += 1;
                searched += (frame.grid.trials.len() > 1) as u32;
                with_corners += (frame.grid.intended_right_corners > 0) as u32;
                let tag = format!("case {case}");
                assert_eq!(frame.grid.exponent, want.scale_exponent, "{tag}: scale");
                assert_eq!(frame.grid.intended_right_corners, want.intended, "{tag}: intended");
                let trials: Vec<(u32, u32, bool)> = frame.grid.trials.iter().map(|t| (t.exponent, t.restored, t.passed)).collect();
                assert_eq!(trials, want.trials, "{tag}: trials");
                equal(&frame.grid.extent, &want.extent, "extent");
                equal(&frame.grid.window_lower, &want.lower, "lower");
                assert_eq!(frame.normal, want.normal, "{tag}: normal");
                for k in 0..3 {
                    equal(&frame.origin[k], &want.origin[k], "origin");
                    equal(&frame.basis_a[k], &want.a[k], "a");
                    equal(&frame.basis_b[k], &want.b[k], "b");
                }
                for k in 0..3 {
                    equal(&frame.gram[k], &want.gram[k], "gram");
                    equal(&frame.inverse[k], &want.inverse[k], "inverse");
                }
                assert_eq!(frame.coordinates.len(), want.coordinates.len());
                for (rank, (got, want)) in frame.coordinates.iter().zip(want.coordinates.iter()).enumerate() {
                    equal(&got[0], &want[0], &format!("{tag}: u of {rank}"));
                    equal(&got[1], &want[1], &format!("{tag}: v of {rank}"));
                }
                assert_eq!(frame.orientation_sign, want.sign, "{tag}: orientation");
            }
            (Err(decline), Expected::Decline(reason)) => {
                assert_eq!(decline.reason, reason, "case {case}: {decline}");
                *declined.entry(reason.name()).or_insert(0) += 1;
            }
            (Err(decline), Expected::Frame(_)) if decline.detail == "SOURCE_SNAP_EMBEDDING_VIOLATED" => skipped += 1,
            (got, want) => panic!("case {case}: the port said {got:?}, the reference {want:?}"),
        }
    }
    eprintln!("frames {frames} (with intended corners {with_corners}, with a search of more than one scale {searched}), declined {declined:?}, skipped for the embedding gate {skipped}");
    assert!(frames > 1500, "the generator reaches the accepted road: {frames}");
    assert!(declined.get("PLANE").copied().unwrap_or(0) > 50, "and the near-planar decline: {declined:?}");
}
