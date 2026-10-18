//! `CHAIN_STATION_PLAN_V1` (`kernel/src/cftuv_envelope/_chain_station.py`, the frozen oracle): which vertices of the physical chains of a domain the decal keeps.
//!
//! A bit-for-bit mirror of `chain_station_plans(snapshot, patch_domain_id)`: the same records (chain, ordinal, kind, disposition, reason, inert face pairs) for every input the oracle answers,
//! and a named `Decline` for every input on which the oracle would raise (a position missing where a window reads it, a face without points) or which the reader cannot hold. No Python here:
//! the PyO3 reader (`cftuv-python/src/chain_station.rs`) hands over the snapshot as plain data, in the iteration order of the Python containers.
//!
//! Exactness. The oracle works on `Fraction(float)` of the binary64 positions; this port works on integers: every coordinate is `m * 2^e` exactly, and all of them are multiplied by ONE
//! power of two `S = 2^E` (the largest `-e`), so a coordinate is the integer `X = S x`. Every predicate of the law is a comparison of polynomials in the coordinates, and each is rewritten
//! as an equivalent integer comparison (the degrees and the powers of `S` are written next to each). An exact comparison has one answer, so the rewrite cannot change it. The centroid
//! of a face (`sum / count`) is never divided: `count * (p - c) = count * p - sum`.
//!
//! The binary64 filter of `beyond_segment` is mirrored on the same floats (`float(Fraction(x))` is `x`, with `-0.0` read as `0.0`); `x ** 2` of CPython is `pow(x, 2)`, here `x * x`.
//! The filter is sound (a margin of 1e-6 on both sides) and decides nothing the exact path would decide otherwise, and no counter reads it: a differing filter verdict could only move
//! work between the two paths, never change an answer.

use std::cell::OnceCell;
use std::collections::{BTreeMap, HashMap, HashSet};

use dashu_int::IBig;

/// `PhysicalChainKind` (the members the law distinguishes).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ChainKind {
    DecalSource,
    Seam,
    SeamSelf,
}

/// `ChainStationKindV1`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StationKind {
    Interior = 0,
    Joint = 1,
}

/// `ChainStationDispositionV1`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Disposition {
    Free = 0,
    Required = 1,
}

/// `ChainStationReasonV1`, in the order of its members.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Reason {
    TransverseEdgesInert = 0,
    FreeUnderChart = 1,
    Fold = 2,
    GroupNotFlat = 3,
    BendBeyondStraight = 4,
    ChordBeyondBudget = 5,
    RunChordBeyondBudget = 6,
    Junction = 7,
    NeighbourSideUnknown = 8,
    NotManifold = 9,
    PositionsUnavailable = 10,
    ClosedChain = 11,
}

/// The oracle would raise here (or the input is outside what the port holds): the caller runs the oracle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Decline(pub String);

fn decline<T>(text: &str) -> Result<T, Decline> {
    Err(Decline(text.to_string()))
}

/// A vertex, face, patch or chain: an index into the reader's table of that id class (equal ids, equal index).
pub type Ix = u32;

pub struct SurfaceFace {
    pub id: Ix,
    pub patch: Ix,
    pub cycle: Vec<Ix>,
}

pub struct NeighbourFace {
    pub id: Ix,
    pub patch: Ix,
    pub vertices: Vec<Ix>,
    pub positions: Vec<[f64; 3]>,
}

pub struct ChainIn {
    pub id: Ix,
    pub kind: ChainKind,
    pub closed: bool,
    pub vertices: Vec<Ix>,
}

/// The snapshot as the law reads it, every container in the iteration order of its Python original.
pub struct Input {
    /// The rank of each vertex / face / chain id by its `value` (ids of one class sort by the text of the value).
    pub vertex_rank: Vec<u32>,
    pub face_rank: Vec<u32>,
    pub chain_rank: Vec<u32>,
    /// `surface_ir.payload_mode is FULL_HOST_SURFACE`.
    pub full: bool,
    /// `source_vertices` with a `LocalPoint3V1` position.
    pub source_positions: Vec<(Ix, [f64; 3])>,
    pub surface_faces: Vec<SurfaceFace>,
    pub neighbours: Vec<NeighbourFace>,
    /// `source_vertex_id` of every corner and junction relation.
    pub corner_vertices: Vec<Ix>,
    /// `physical_chains`.
    pub chains: Vec<ChainIn>,
    /// `physical_chain_id` of every chain use of the domain.
    pub domain_chains: Vec<Ix>,
    /// The chord budget `B` and the bend limit `L` as `(numerator, denominator)`, denominators positive; `float_limit = float(B * B)`.
    pub budget: (IBig, IBig),
    pub bend: (IBig, IBig),
    pub float_limit: f64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Station {
    pub ordinal: usize,
    pub kind: StationKind,
    pub disposition: Disposition,
    pub reason: Reason,
    /// Pairs of face indices into `Facts::faces` (`Answer::face_ids`).
    pub pairs: Vec<(usize, usize)>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Plan {
    /// Index into `Input::chains`.
    pub chain: usize,
    pub stations: Vec<Station>,
}

pub struct Answer {
    pub plans: Vec<Plan>,
    /// The id (index) of each face a pair names.
    pub face_ids: Vec<Ix>,
}

// --------------------------------------------------------------------------
// exact integers
// --------------------------------------------------------------------------

type V3 = [IBig; 3];

fn sub(a: &V3, b: &V3) -> V3 {
    [&a[0] - &b[0], &a[1] - &b[1], &a[2] - &b[2]]
}

fn dot(a: &V3, b: &V3) -> IBig {
    &a[0] * &b[0] + &a[1] * &b[1] + &a[2] * &b[2]
}

fn cross(a: &V3, b: &V3) -> V3 {
    [&a[1] * &b[2] - &a[2] * &b[1], &a[2] * &b[0] - &a[0] * &b[2], &a[0] * &b[1] - &a[1] * &b[0]]
}

fn zero() -> V3 {
    [IBig::ZERO, IBig::ZERO, IBig::ZERO]
}

/// `x = sign * m * 2^e` with `m` odd (or zero).
fn split(x: f64) -> (bool, u64, i32) {
    let bits = x.to_bits();
    let negative = bits >> 63 == 1;
    let field = ((bits >> 52) & 0x7ff) as i32;
    let fraction = bits & ((1u64 << 52) - 1);
    let (mut m, mut e) = if field == 0 { (fraction, -1074) } else { (fraction | (1u64 << 52), field - 1075) };
    if m == 0 {
        return (false, 0, 0);
    }
    let zeros = m.trailing_zeros();
    m >>= zeros;
    e += zeros as i32;
    (negative, m, e)
}

/// `x * 2^scale` as an integer (`scale >= -e` of every coordinate).
fn scaled(x: f64, scale: i32) -> IBig {
    let (negative, m, e) = split(x);
    let value = IBig::from(m) << ((e + scale) as usize);
    if negative {
        -value
    } else {
        value
    }
}

/// The comparison `lhs > B^2 * S^2 * rhs` of a squared length against the chord budget: `lhs * bd^2 > bn^2 * rhs * 2^(2E)`.
struct Budget {
    numerator2: IBig,
    denominator2: IBig,
    shift: usize,
}

impl Budget {
    fn beyond(&self, lhs: &IBig, rhs: &IBig) -> bool {
        lhs * &self.denominator2 > &self.numerator2 * (rhs << self.shift)
    }
}

// --------------------------------------------------------------------------
// the facts (`StationFacts`)
// --------------------------------------------------------------------------

/// `(sum of the points, count, Newell normal, |normal|^2, B^2 S^2 count^2 |normal|^2 as the right side of `_within`)`.
struct Plane {
    sum: V3,
    count: IBig,
    normal: V3,
    squared: IBig,
    limit: IBig,
}

struct Face {
    id: Ix,
    patch: Ix,
    cycle: Vec<Ix>,
    points: Option<Vec<V3>>,
    plane: OnceCell<Option<Plane>>,
}

struct Facts<'a> {
    input: &'a Input,
    /// Per vertex: `(binary64 position, exact scaled position)`.
    position: Vec<Option<([f64; 3], V3)>>,
    faces: Vec<Face>,
    at: Vec<Vec<usize>>,
    corner: Vec<bool>,
    budget: Budget,
    bend_numerator2: IBig,
    bend_denominator2: IBig,
    inert: HashMap<(usize, usize), bool>,
}

fn positive_zero(point: [f64; 3]) -> [f64; 3] {
    [point[0] + 0.0, point[1] + 0.0, point[2] + 0.0]
}

impl<'a> Facts<'a> {
    fn new(input: &'a Input) -> Result<Facts<'a>, Decline> {
        let vertices = input.vertex_rank.len();
        let mut scale = 0i32;
        let mut note = |point: &[f64; 3]| -> Result<(), Decline> {
            for &x in point {
                if !x.is_finite() {
                    return decline("a position that is not finite");
                }
                let (_, m, e) = split(x);
                if m != 0 {
                    scale = scale.max(-e);
                }
            }
            Ok(())
        };
        for (_, point) in &input.source_positions {
            note(point)?;
        }
        for face in &input.neighbours {
            for point in &face.positions {
                note(point)?;
            }
        }
        let exact = |point: &[f64; 3]| -> V3 { [scaled(point[0], scale), scaled(point[1], scale), scaled(point[2], scale)] };
        let mut position: Vec<Option<([f64; 3], V3)>> = vec![None; vertices];
        for (vertex, point) in &input.source_positions {
            position[*vertex as usize] = Some((positive_zero(*point), exact(point)));
        }
        let mut faces: Vec<Face> = Vec::with_capacity(input.surface_faces.len() + input.neighbours.len());
        let mut slot: HashMap<Ix, usize> = HashMap::with_capacity(faces.capacity());
        for face in &input.surface_faces {
            let points = if input.full {
                let mut found = Vec::with_capacity(face.cycle.len());
                let mut missing = false;
                for vertex in &face.cycle {
                    match &position[*vertex as usize] {
                        Some((_, point)) => found.push(point.clone()),
                        None => missing = true,
                    }
                }
                if missing {
                    None
                } else {
                    Some(found)
                }
            } else {
                None
            };
            let record = Face { id: face.id, patch: face.patch, cycle: face.cycle.clone(), points, plane: OnceCell::new() };
            match slot.get(&face.id) {
                Some(&at) => faces[at] = record, // a dict keeps the place of the first key and the value of the last
                None => {
                    slot.insert(face.id, faces.len());
                    faces.push(record);
                }
            }
        }
        for face in &input.neighbours {
            if slot.contains_key(&face.id) {
                continue;
            }
            let points: Vec<V3> = face.positions.iter().map(&exact).collect();
            for (vertex, point) in face.vertices.iter().zip(face.positions.iter()) {
                let entry = &mut position[*vertex as usize];
                if entry.is_none() {
                    *entry = Some((positive_zero(*point), exact(point)));
                }
            }
            slot.insert(face.id, faces.len());
            faces.push(Face { id: face.id, patch: face.patch, cycle: face.vertices.clone(), points: Some(points), plane: OnceCell::new() });
        }
        let mut at: Vec<Vec<usize>> = vec![Vec::new(); vertices];
        for (index, face) in faces.iter().enumerate() {
            for vertex in &face.cycle {
                at[*vertex as usize].push(index);
            }
        }
        for list in at.iter_mut() {
            list.sort_by_key(|index| input.face_rank[faces[*index].id as usize]);
        }
        let mut corner = vec![false; vertices];
        for vertex in &input.corner_vertices {
            corner[*vertex as usize] = true;
        }
        let (bn, bd) = &input.budget;
        let (ln, ld) = &input.bend;
        Ok(Facts {
            input,
            position,
            faces,
            at,
            corner,
            budget: Budget { numerator2: bn * bn, denominator2: bd * bd, shift: 2 * scale as usize },
            bend_numerator2: ln * ln,
            bend_denominator2: ld * ld,
            inert: HashMap::new(),
        })
    }

    /// `_Face.plane`: `None` without positions or without area; the oracle raises (`points[0]`) on a face with an empty point tuple.
    fn plane(&self, index: usize) -> Result<Option<&Plane>, Decline> {
        let face = &self.faces[index];
        if face.plane.get().is_none() {
            let value = match &face.points {
                None => None,
                Some(points) => self.newell(points)?,
            };
            let _ = face.plane.set(value);
        }
        Ok(face.plane.get().and_then(|plane| plane.as_ref()))
    }

    fn newell(&self, points: &[V3]) -> Result<Option<Plane>, Decline> {
        if points.is_empty() {
            return decline("a seam neighbour face without positions (the oracle raises on points[0])");
        }
        let origin = &points[0];
        let mut normal = zero();
        for index in 1..points.len().saturating_sub(1) {
            let part = cross(&sub(&points[index], origin), &sub(&points[index + 1], origin));
            for axis in 0..3 {
                normal[axis] = &normal[axis] + &part[axis];
            }
        }
        let squared = dot(&normal, &normal);
        if squared == IBig::ZERO {
            return Ok(None);
        }
        let mut sum = zero();
        for point in points {
            for axis in 0..3 {
                sum[axis] = &sum[axis] + &point[axis];
            }
        }
        let count = IBig::from(points.len());
        // degrees: H = n_X . (count p_X - sum_X) = S^3 count h; h^2 <= B^2 |n|^2  <=>  H^2 bd^2 <= bn^2 count^2 |n_X|^2 2^(2E)
        let limit = &self.budget.numerator2 * ((&count * &count * &squared) << self.budget.shift);
        Ok(Some(Plane { sum, count, normal, squared, limit }))
    }

    /// `_within(plane, points, budget)`.
    fn within(&self, plane: &Plane, points: &[V3]) -> bool {
        for point in points {
            let lifted = [&plane.count * &point[0] - &plane.sum[0], &plane.count * &point[1] - &plane.sum[1], &plane.count * &point[2] - &plane.sum[2]];
            let height = dot(&plane.normal, &lifted);
            if &height * &height * &self.budget.denominator2 > plane.limit {
                return false;
            }
        }
        true
    }

    fn points(&self, index: usize) -> &[V3] {
        self.faces[index].points.as_deref().unwrap_or(&[])
    }

    fn inert(&mut self, first: usize, second: usize) -> Result<bool, Decline> {
        let rank = |index: usize| self.input.face_rank[self.faces[index].id as usize];
        let key = if rank(first) <= rank(second) { (first, second) } else { (second, first) };
        if let Some(found) = self.inert.get(&key) {
            return Ok(*found);
        }
        let found = {
            let one = self.plane(first)?.ok_or_else(|| Decline("a transverse face without a plane".into()))?;
            if !self.within(one, self.points(second)) {
                false
            } else {
                let two = self.plane(second)?.ok_or_else(|| Decline("a transverse face without a plane".into()))?;
                self.within(two, self.points(first))
            }
        };
        self.inert.insert(key, found);
        Ok(found)
    }

    /// `flat_together`: every point of every face within the budget of the plane of the biggest face (by `|n|^2`, then by name).
    fn flat_together(&self, faces: &[usize]) -> Result<bool, Decline> {
        let mut biggest: Option<(usize, &Plane)> = None;
        for &index in faces {
            let plane = self.plane(index)?.ok_or_else(|| Decline("a grouped face without a plane".into()))?;
            let better = match biggest {
                None => true,
                Some((best, top)) => {
                    (&plane.squared, self.input.face_rank[self.faces[index].id as usize]) > (&top.squared, self.input.face_rank[self.faces[best].id as usize])
                }
            };
            if better {
                biggest = Some((index, plane));
            }
        }
        let Some((_, plane)) = biggest else { return decline("an empty face group") };
        Ok(faces.iter().all(|&index| self.within(plane, self.points(index))))
    }

    fn at(&self, vertex: Ix) -> Result<&([f64; 3], V3), Decline> {
        self.position[vertex as usize].as_ref().ok_or_else(|| Decline("a run window reads a vertex without a position (the oracle raises KeyError)".into()))
    }

    /// `beyond_segment(first, second, point)`.
    fn beyond_segment(&self, first: Ix, second: Ix, point: Ix) -> Result<bool, Decline> {
        let (a, start) = self.at(first)?;
        let (b, end) = self.at(second)?;
        let (p, middle) = self.at(point)?;
        if let Some(fast) = beyond_by_floats(a, b, p, self.input.float_limit) {
            return Ok(fast);
        }
        let whole = sub(end, start);
        let along = sub(middle, start);
        let reach = dot(&along, &whole);
        // degrees: reach and |whole|^2 are S^2; |along|^2 against B^2 is S^2 a > B^2 S^2; |cross|^2 (S^4) against B^2 |whole|^2 (S^2)
        if reach <= IBig::ZERO {
            return Ok(self.budget.beyond(&dot(&along, &along), &IBig::ONE));
        }
        let length = dot(&whole, &whole);
        if reach >= length {
            let far = sub(middle, end);
            return Ok(self.budget.beyond(&dot(&far, &far), &IBig::ONE));
        }
        let off = cross(&along, &whole);
        Ok(self.budget.beyond(&dot(&off, &off), &length))
    }

    /// `not_straight(before, vertex, after)`.
    fn not_straight(&self, before: Ix, vertex: Ix, after: Ix) -> Option<Reason> {
        let (Some((_, first)), Some((_, second)), Some((_, middle))) = (&self.position[before as usize], &self.position[after as usize], &self.position[vertex as usize]) else {
            return Some(Reason::PositionsUnavailable);
        };
        let one = sub(middle, first);
        let two = sub(second, middle);
        let whole = sub(second, first);
        let (one_square, two_square, whole_square) = (dot(&one, &one), dot(&two, &two), dot(&whole, &whole));
        if one_square == IBig::ZERO || two_square == IBig::ZERO || whole_square == IBig::ZERO {
            return Some(Reason::NotManifold);
        }
        let turn = cross(&one, &two);
        // degrees: |turn|^2 and |one|^2 |two|^2 are both S^4
        if dot(&one, &two) <= IBig::ZERO || dot(&turn, &turn) * &self.bend_denominator2 > &self.bend_numerator2 * &one_square * &two_square {
            return Some(Reason::BendBeyondStraight);
        }
        let off = cross(&one, &whole);
        if self.budget.beyond(&dot(&off, &off), &whole_square) {
            return Some(Reason::ChordBeyondBudget);
        }
        None
    }
}

/// `_beyond_by_floats`: `Some` when binary64 decides with the margin, `None` for the exact path.
fn beyond_by_floats(a: &[f64; 3], b: &[f64; 3], p: &[f64; 3], float_limit: f64) -> Option<bool> {
    let w = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let reach = v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
    let whole = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
    let (square, limit) = if reach <= 0.0 {
        (v[0] * v[0] + v[1] * v[1] + v[2] * v[2], float_limit)
    } else if reach >= whole {
        let d = [p[0] - b[0], p[1] - b[1], p[2] - b[2]];
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2], float_limit)
    } else {
        let c = [v[1] * w[2] - v[2] * w[1], v[2] * w[0] - v[0] * w[2], v[0] * w[1] - v[1] * w[0]];
        (c[0] * c[0] + c[1] * c[1] + c[2] * c[2], float_limit * whole)
    };
    if square <= limit * (1.0 - 1e-6) {
        return Some(false);
    }
    if square >= limit * (1.0 + 1e-6) {
        return Some(true);
    }
    None
}

// --------------------------------------------------------------------------
// the law
// --------------------------------------------------------------------------

type Decided = (Disposition, Reason, Vec<(usize, usize)>);

fn expected_faces(kind: ChainKind) -> usize {
    if kind == ChainKind::DecalSource {
        1
    } else {
        2
    }
}

/// `_station(facts, vertex, arms)`.
fn station(facts: &mut Facts<'_>, vertex: Ix, arms: [(Ix, ChainKind); 2]) -> Result<Decided, Decline> {
    let required = |reason| Ok((Disposition::Required, reason, Vec::new()));
    if facts.corner[vertex as usize] {
        return required(Reason::Junction);
    }
    let faces = facts.at[vertex as usize].clone();
    if faces.is_empty() || faces.iter().any(|&index| facts.faces[index].points.is_none()) {
        return required(Reason::PositionsUnavailable);
    }
    for &index in &faces {
        if facts.faces[index].cycle.iter().filter(|item| **item == vertex).count() != 1 || facts.plane(index)?.is_none() {
            return required(Reason::NotManifold);
        }
    }
    // `edges`: neighbour vertex -> faces, in the order of `faces`
    let mut edges: Vec<(Ix, Vec<usize>)> = Vec::new();
    for &index in &faces {
        let cycle = &facts.faces[index].cycle;
        let at = cycle.iter().position(|item| *item == vertex).expect("the vertex is in the cycle");
        let n = cycle.len();
        for other in [cycle[(at + n - 1) % n], cycle[(at + 1) % n]] {
            match edges.iter_mut().find(|(key, _)| *key == other) {
                Some((_, list)) => list.push(index),
                None => edges.push((other, vec![index])),
            }
        }
    }
    let ends = [arms[0].0, arms[1].0];
    for (other, kind) in arms {
        let found = edges.iter().find(|(key, _)| *key == other).map_or(0, |(_, list)| list.len());
        let wanted = expected_faces(kind);
        if found > wanted {
            return required(Reason::NotManifold);
        }
        if found < wanted {
            return required(if kind == ChainKind::Seam { Reason::NeighbourSideUnknown } else { Reason::NotManifold });
        }
    }
    let input = facts.input;
    edges.sort_by_key(|(other, _)| input.vertex_rank[*other as usize]);
    let face_rank = |facts: &Facts<'_>, index: usize| input.face_rank[facts.faces[index].id as usize];
    let mut pairs = Vec::new();
    let mut folded = false;
    for (other, sides) in &edges {
        if ends.contains(other) {
            continue;
        }
        if sides.len() > 2 {
            return required(Reason::NotManifold);
        }
        if sides.len() < 2 || facts.faces[sides[0]].patch != facts.faces[sides[1]].patch {
            return required(Reason::Junction);
        }
        let (first, second) = if face_rank(facts, sides[1]) < face_rank(facts, sides[0]) { (sides[1], sides[0]) } else { (sides[0], sides[1]) };
        if facts.inert(first, second)? {
            pairs.push((first, second));
        } else {
            folded = true;
        }
    }
    if let Some(bent) = facts.not_straight(ends[0], vertex, ends[1]) {
        return required(bent);
    }
    if folded {
        return required(Reason::Fold);
    }
    pairs.sort_by_key(|(first, second)| (face_rank(facts, *first), face_rank(facts, *second)));
    Ok((Disposition::Free, Reason::TransverseEdgesInert, pairs))
}

type Stations = BTreeMap<usize, (StationKind, Disposition, Reason, Vec<(usize, usize)>)>;

/// `_demote_deep_groups`.
fn demote_deep_groups(facts: &Facts<'_>, found: &mut [(usize, Stations)]) -> Result<(), Decline> {
    for (_, stations) in found.iter_mut() {
        let mut parent: HashMap<usize, usize> = HashMap::new();
        let mut order: Vec<usize> = Vec::new();
        fn root(parent: &mut HashMap<usize, usize>, order: &mut Vec<usize>, item: usize) -> usize {
            if let std::collections::hash_map::Entry::Vacant(entry) = parent.entry(item) {
                entry.insert(item);
                order.push(item);
            }
            let mut item = item;
            while parent[&item] != item {
                let grand = parent[&parent[&item]];
                parent.insert(item, grand);
                item = grand;
            }
            item
        }
        for (_, disposition, _, pairs) in stations.values() {
            if *disposition == Disposition::Free {
                for &(first, second) in pairs {
                    let to = root(&mut parent, &mut order, second);
                    let from = root(&mut parent, &mut order, first);
                    parent.insert(from, to);
                }
            }
        }
        let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
        for item in order.clone() {
            let key = root(&mut parent, &mut order, item);
            match groups.iter_mut().find(|(group, _)| *group == key) {
                Some((_, list)) => list.push(item),
                None => groups.push((key, vec![item])),
            }
        }
        let mut deep: HashSet<usize> = HashSet::new();
        for (key, faces) in &groups {
            if !facts.flat_together(faces)? {
                deep.insert(*key);
            }
        }
        for value in stations.values_mut() {
            if value.1 == Disposition::Free && value.3.iter().any(|&(first, _)| deep.contains(&root(&mut parent, &mut order, first))) {
                *value = (value.0, Disposition::Required, Reason::GroupNotFlat, Vec::new());
            }
        }
    }
    Ok(())
}

/// `_agree_on_joints`.
fn agree_on_joints(found: &mut [(usize, Stations)], chains: &[ChainIn]) {
    let mut demoted: HashSet<Ix> = HashSet::new();
    for (chain, stations) in found.iter() {
        for (ordinal, (kind, _, reason, _)) in stations {
            if *kind == StationKind::Joint && *reason == Reason::GroupNotFlat {
                demoted.insert(chains[*chain].vertices[*ordinal]);
            }
        }
    }
    if demoted.is_empty() {
        return;
    }
    for (chain, stations) in found.iter_mut() {
        for (ordinal, value) in stations.iter_mut() {
            if value.0 == StationKind::Joint && demoted.contains(&chains[*chain].vertices[*ordinal]) && value.1 == Disposition::Free {
                *value = (value.0, Disposition::Required, Reason::GroupNotFlat, Vec::new());
            }
        }
    }
}

type Line = Vec<(Ix, Vec<(usize, usize)>)>;

/// `_lines`: the chains of the domain joined at their joints, `(vertex, ((chain, ordinal), ...))` in walking order.
fn lines(found: &[(usize, Stations)], input: &Input) -> Vec<Line> {
    let vertices_of = |chain: usize| &input.chains[chain].vertices;
    let mut joined: Vec<(Ix, Vec<usize>)> = Vec::new();
    for (chain, stations) in found {
        for (ordinal, (kind, ..)) in stations {
            if *kind == StationKind::Joint {
                let vertex = vertices_of(*chain)[*ordinal];
                match joined.iter_mut().find(|(key, _)| *key == vertex) {
                    Some((_, list)) => list.push(*chain),
                    None => joined.push((vertex, vec![*chain])),
                }
            }
        }
    }
    let mut link: HashMap<(usize, u8), (usize, u8)> = HashMap::new();
    for (vertex, ids) in &joined {
        if ids.len() == 2 && ids[0] != ids[1] {
            let (one, two) = (ids[0], ids[1]);
            let end_one = u8::from(vertices_of(one)[0] != *vertex);
            let end_two = u8::from(vertices_of(two)[0] != *vertex);
            link.insert((one, end_one), (two, end_two));
            link.insert((two, end_two), (one, end_one));
        }
    }
    let rank = |chain: usize| input.chain_rank[input.chains[chain].id as usize];
    let mut starts: Vec<usize> = found.iter().map(|(chain, _)| *chain).collect();
    starts.sort_by_key(|chain| rank(*chain));
    let mut visited: HashSet<usize> = HashSet::new();
    let mut out = Vec::new();
    for start in starts {
        if visited.contains(&start) || vertices_of(start).len() < 2 {
            continue;
        }
        let (mut chain, mut flip) = (start, false);
        let mut seen: Vec<usize> = vec![start];
        let mut ahead;
        loop {
            ahead = link.get(&(chain, u8::from(flip))).copied();
            match ahead {
                Some((next, _)) if !seen.contains(&next) => {
                    chain = next;
                    flip = ahead.unwrap().1 == 0;
                    seen.push(chain);
                }
                _ => break,
            }
        }
        if ahead.is_some() {
            chain = *seen.iter().min_by_key(|item| rank(**item)).expect("seen holds the start");
            flip = false;
        }
        let mut line: Line = Vec::new();
        while !visited.contains(&chain) {
            visited.insert(chain);
            let vertices = vertices_of(chain);
            let n = vertices.len();
            let first = if flip { n - 1 } else { 0 };
            for step in 0..n {
                let ordinal = if flip { n - 1 - step } else { step };
                let reference = (chain, ordinal);
                if !line.is_empty() && ordinal == first && line.last().unwrap().0 == vertices[ordinal] {
                    line.last_mut().unwrap().1.push(reference);
                } else {
                    line.push((vertices[ordinal], vec![reference]));
                }
            }
            match link.get(&(chain, if flip { 0 } else { 1 })) {
                None => break,
                Some(&(next, end)) => {
                    chain = next;
                    flip = end == 1;
                }
            }
        }
        out.push(line);
    }
    out
}

/// `_demote_runs`.
fn demote_runs(facts: &Facts<'_>, found: &mut [(usize, Stations)], input: &Input) -> Result<(), Decline> {
    let slot: HashMap<usize, usize> = found.iter().enumerate().map(|(at, (chain, _))| (*chain, at)).collect();
    for line in lines(found, input) {
        let mut carried: Vec<bool> = line
            .iter()
            .map(|(_, refs)| refs.iter().any(|(chain, ordinal)| found[slot[chain]].1.get(ordinal).is_none_or(|value| value.1 == Disposition::Required)))
            .collect();
        let mut anchor = 0usize;
        for index in 1..line.len().saturating_sub(1) {
            if carried[index] {
                anchor = index;
                continue;
            }
            let (first, second) = (line[anchor].0, line[index + 1].0);
            let mut beyond = false;
            for (point, _) in &line[anchor + 1..=index] {
                if facts.beyond_segment(first, second, *point)? {
                    beyond = true;
                    break;
                }
            }
            if beyond {
                for (chain, ordinal) in &line[index].1 {
                    let value = found[slot[chain]].1.get_mut(ordinal).expect("a free station is recorded");
                    *value = (value.0, Disposition::Required, Reason::RunChordBeyondBudget, Vec::new());
                }
                carried[index] = true;
                anchor = index;
            }
        }
    }
    Ok(())
}

/// `_joint_arms`.
fn joint_arms(chain: &ChainIn, other: &ChainIn, vertex: Ix, ends_at: usize) -> Option<[(Ix, ChainKind); 2]> {
    let (first, second) = (&chain.vertices, &other.vertices);
    if ends_at != 2 || chain.closed || other.closed {
        return None;
    }
    let near_first = if first[0] == vertex { first[1] } else { first[first.len() - 2] };
    let near_second = if second[0] == vertex { second[1] } else { second[second.len() - 2] };
    if near_first == near_second {
        return None;
    }
    Some([(near_first, chain.kind), (near_second, other.kind)])
}

/// `chain_station_plans(snapshot, patch_domain_id)`: the plans in no particular order (the oracle answers a frozenset), stations by ordinal. The reason `TRANSVERSE_EDGES_INERT`
/// is answered as such; the caller names it `FREE_UNDER_CHART` when the domain's chart is developed (`_chart_is_developed`).
pub fn chain_station_plans(input: &Input) -> Result<Answer, Decline> {
    let mut ids: HashSet<Ix> = HashSet::with_capacity(input.chains.len());
    for chain in &input.chains {
        if !ids.insert(chain.id) {
            return decline("two physical chains share an id");
        }
    }
    let mut facts = Facts::new(input)?;
    let in_domain: HashSet<Ix> = input.domain_chains.iter().copied().collect();
    let mut chains: Vec<usize> = (0..input.chains.len()).filter(|index| in_domain.contains(&input.chains[*index].id)).collect();
    chains.sort_by_key(|index| input.chain_rank[input.chains[*index].id as usize]);
    let mut ends: HashMap<Ix, Vec<usize>> = HashMap::new();
    for (index, chain) in input.chains.iter().enumerate() {
        let n = chain.vertices.len();
        if !chain.closed && n >= 2 {
            ends.entry(chain.vertices[0]).or_default().push(index);
            if chain.vertices[n - 1] != chain.vertices[0] {
                ends.entry(chain.vertices[n - 1]).or_default().push(index);
            }
        }
    }
    let mut found: Vec<(usize, Stations)> = Vec::new();
    let mut joints: HashMap<Ix, Decided> = HashMap::new();
    for &index in &chains {
        let chain = &input.chains[index];
        let vertices = &chain.vertices;
        let n = vertices.len();
        if chain.closed {
            let closed = if n > 1 && vertices[0] == vertices[n - 1] { n - 1 } else { n };
            found.push((index, (0..closed).map(|ordinal| (ordinal, (StationKind::Interior, Disposition::Required, Reason::ClosedChain, Vec::new()))).collect()));
            continue;
        }
        let mut stations: Stations = BTreeMap::new();
        for ordinal in 1..n.saturating_sub(1) {
            let (disposition, reason, pairs) = station(&mut facts, vertices[ordinal], [(vertices[ordinal - 1], chain.kind), (vertices[ordinal + 1], chain.kind)])?;
            stations.insert(ordinal, (StationKind::Interior, disposition, reason, pairs));
        }
        if n >= 2 {
            let mut ordinals = vec![0];
            if n - 1 != 0 {
                ordinals.push(n - 1);
            }
            for ordinal in ordinals {
                let vertex = vertices[ordinal];
                let at = ends.get(&vertex).map(Vec::as_slice).unwrap_or(&[]);
                let partner = at.iter().copied().find(|item| *item != index && in_domain.contains(&input.chains[*item].id));
                let arms = partner.and_then(|other| joint_arms(chain, &input.chains[other], vertex, at.len()));
                if let Some(arms) = arms {
                    if facts.not_straight(arms[0].0, vertex, arms[1].0).is_none() {
                        if let std::collections::hash_map::Entry::Vacant(entry) = joints.entry(vertex) {
                            entry.insert(station(&mut facts, vertex, arms)?);
                        }
                        let (disposition, reason, pairs) = joints[&vertex].clone();
                        stations.insert(ordinal, (StationKind::Joint, disposition, reason, pairs));
                    }
                }
            }
        }
        if !stations.is_empty() {
            found.push((index, stations));
        }
    }
    demote_deep_groups(&facts, &mut found)?;
    agree_on_joints(&mut found, &input.chains);
    demote_runs(&facts, &mut found, input)?;
    let face_ids = facts.faces.iter().map(|face| face.id).collect();
    let plans = found
        .into_iter()
        .map(|(chain, stations)| Plan {
            chain,
            stations: stations
                .into_iter()
                .map(|(ordinal, (kind, disposition, reason, pairs))| Station { ordinal, kind, disposition, reason, pairs })
                .collect(),
        })
        .collect();
    Ok(Answer { plans, face_ids })
}

#[cfg(test)]
mod tests;
