//! `materialize.uv_unfold.unfold` (UNFOLD_ALONG_U_V1): the `u` of the islands of Straight strips with `v` held, the Rust product of the law.
//!
//! The Python module is the reference and its docstring is the law; this file mirrors it statement for statement: the triangles Blender interpolates (`polyfill`), the
//! per-triangle frame `(t̂, n̂r)` in the 3D plane (`triangle`), the Poisson system with the source held or the gauge pinned, the reverse Cuthill-McKee order, the profile
//! Cholesky, the local/global steps and the guard rounds on the same island, the slopes over the chart and the flips. Only `+ - * /` and `sqrt` of binary64 in the
//! reference's order (no fused multiply-add, no reassociation), so the answer equals the reference's to the last bit on one machine; the registry still names the tolerance
//! the differential tests compare with (`UNFOLD_NATIVE_EQUALITY_V1`), because the reference is CPython and the port is LLVM.

use std::collections::{BTreeMap, BTreeSet};

/// `uv_unfold.DEGENERATE_SINE`.
pub const DEGENERATE_SINE: f64 = 1e-9;
/// `uv_unfold.FLAT_GRADIENT`.
pub const FLAT_GRADIENT: f64 = 1e-6;
/// `uv_unfold.UNORIENTED_SINE`.
pub const UNORIENTED_SINE: f64 = 1e-6;
/// `uv_unfold.GUARD_FLOOR`.
pub const GUARD_FLOOR: f64 = 0.5;
/// `uv_unfold.GUARD_CEILING`.
pub const GUARD_CEILING: f64 = 2.0;
/// `uv_unfold.GUARD_FACTOR`.
pub const GUARD_FACTOR: f64 = 4.0;

/// `uv_unfold.COUNTER_NAMES`, in that order.
pub const COUNTER_NAMES: [&str; 13] = [
    "MATERIALIZE_STRAIGHTEN_UNFOLD_TRIANGLES",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_DEGENERATE_TRIANGLES",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_FLAT_R_TRIANGLES",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_UNORIENTED_TRIANGLES",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_ISLANDS",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_ISLANDS_WITHOUT_SOURCE",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_ISOLATED_VERTICES",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_SINGULAR_ISLANDS",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_REWEIGHTED_TRIANGLES",
    "MATERIALIZE_STRAIGHTEN_UV_FLIPPED_TRIANGLES",
    "MATERIALIZE_STRAIGHTEN_UV_FLIPPED_AREA_MM2",
    "MATERIALIZE_STRAIGHTEN_UV_ZERO_AREA",
    "MATERIALIZE_STRAIGHTEN_UNFOLD_SLOPE_SKIPPED_TRIANGLES",
];

const TRIANGLES: usize = 0;
const DEGENERATE: usize = 1;
const FLAT: usize = 2;
const UNORIENTED: usize = 3;
const ISLANDS: usize = 4;
const WITHOUT_SOURCE: usize = 5;
const ISOLATED: usize = 6;
const SINGULAR: usize = 7;
const GUARDED: usize = 8;
const FLIPPED: usize = 9;
const FLIPPED_AREA: usize = 10;
const ZERO_AREA: usize = 11;
const SLOPE_SKIPPED: usize = 12;

/// `uv_unfold.UnfoldInputV1`.
#[derive(Clone, Debug, Default)]
pub struct UnfoldInput {
    pub positions: Vec<f64>,
    pub chart: Vec<f64>,
    pub r: Vec<f64>,
    pub s: Vec<f64>,
    pub source: Vec<bool>,
    pub ring_starts: Vec<usize>,
    pub corners: Vec<usize>,
    pub corner_station: Vec<f64>,
    pub hold_source: bool,
    pub iterations: usize,
    pub guard_rounds: usize,
}

/// `uv_unfold.UnfoldResultV1`: the counters are the nonzero ones, in the order of `COUNTER_NAMES`.
#[derive(Clone, Debug, PartialEq)]
pub struct UnfoldResult {
    pub u: Vec<f64>,
    pub slope_x: Vec<f64>,
    pub slope_y: Vec<f64>,
    pub held: Vec<bool>,
    pub counters: Vec<(&'static str, i64)>,
}

/// Why the input cannot be read (lengths that disagree, an index outside the vertices): the shim declines by name and the reference answers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Malformed(pub String);

fn cross(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ax * by - ay * bx
}

fn sign(value: f64) -> i8 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

fn doubled_area(points: &[(f64, f64)]) -> f64 {
    let size = points.len();
    let mut doubled = 0.0;
    for at in 0..size {
        let following = points[(at + 1) % size];
        doubled += points[at].0 * following.1 - following.0 * points[at].1;
    }
    doubled
}

/// `uv_unfold.polyfill`: the triangles `(i, j, k)` of a ring of chart points as Blender cuts it.
pub fn polyfill(points: &[(f64, f64)]) -> Vec<[usize; 3]> {
    let size = points.len();
    if size < 3 {
        return Vec::new();
    }
    if size == 3 {
        return vec![[0, 1, 2]];
    }
    if size == 4 {
        let (v1, v2, v3, v4) = (points[0], points[1], points[2], points[3]);
        let d12 = (v2.0 - v1.0, v2.1 - v1.1);
        let d13 = (v3.0 - v1.0, v3.1 - v1.1);
        let d14 = (v4.0 - v1.0, v4.1 - v1.1);
        if cross(d12.0, d12.1, d13.0, d13.1) * cross(d14.0, d14.1, d13.0, d13.1) > 0.0 {
            return vec![[0, 1, 3], [1, 2, 3]];
        }
        return vec![[0, 1, 2], [0, 2, 3]];
    }
    let orient = if doubled_area(points) >= 0.0 { 1.0 } else { -1.0 };
    let span = |a: usize, b: usize, c: usize| -> i8 {
        let (pa, pb, pc) = (points[a], points[b], points[c]);
        sign(orient * cross(pb.0 - pa.0, pb.1 - pa.1, pc.0 - pb.0, pc.1 - pb.1))
    };
    let mut following: Vec<usize> = (0..size).map(|at| (at + 1) % size).collect();
    let mut previous: Vec<usize> = (0..size).map(|at| (at + size - 1) % size).collect();
    let mut signs: Vec<i8> = (0..size).map(|at| span(previous[at], at, following[at])).collect();
    let mut concave = signs.iter().filter(|value| **value != 1).count();
    let mut head = 0usize;
    let mut count = size;
    let mut found = Vec::with_capacity(size - 2);
    let mut ear_init = head;
    let mut reverse = false;
    while count > 3 {
        let tip = |ear: usize, accept: i8, signs: &[i8], following: &[usize], previous: &[usize], concave: usize| -> bool {
            if concave == 0 {
                return true;
            }
            if signs[ear] != accept {
                return false;
            }
            let (a, b, c) = (previous[ear], ear, following[ear]);
            let mut at = following[c];
            while at != a {
                if signs[at] != 1 && span(b, c, at) != -1 && span(c, a, at) != -1 && span(a, b, at) != -1 {
                    return false;
                }
                at = following[at];
            }
            true
        };
        let mut chosen = None;
        for accept in [1i8, 0i8] {
            let mut ear = ear_init;
            for _ in 0..count {
                if tip(ear, accept, &signs, &following, &previous, concave) {
                    chosen = Some(ear);
                    break;
                }
                ear = if reverse { previous[ear] } else { following[ear] };
            }
            if chosen.is_some() {
                break;
            }
        }
        let chosen = match chosen {
            Some(ear) => ear,
            None => {
                let mut ear = ear_init;
                let mut picked = None;
                for _ in 0..count {
                    if signs[ear] != -1 {
                        picked = Some(ear);
                        break;
                    }
                    ear = following[ear];
                }
                picked.unwrap_or(ear)
            }
        };
        if signs[chosen] != 1 {
            concave -= 1;
        }
        let (before, after) = (previous[chosen], following[chosen]);
        found.push([before, chosen, after]);
        following[before] = after;
        previous[after] = before;
        if head == chosen {
            head = after;
        }
        count -= 1;
        for near in [before, after] {
            if signs[near] != 1 {
                signs[near] = span(previous[near], near, following[near]);
                if signs[near] == 1 {
                    concave -= 1;
                }
            }
        }
        ear_init = if reverse { previous[before] } else { following[after] };
        if signs[ear_init] != 1 {
            ear_init = if reverse { previous[ear_init] } else { following[ear_init] };
            reverse = !reverse;
        }
    }
    found.push([head, following[head], following[following[head]]]);
    found
}

/// `uv_unfold._Profile`.
struct Profile {
    first: Vec<usize>,
    rows: Vec<Vec<f64>>,
}

impl Profile {
    fn factor(first: Vec<usize>, entries: &[BTreeMap<usize, f64>]) -> Option<Profile> {
        let size = first.len();
        let mut rows: Vec<Vec<f64>> = Vec::with_capacity(size);
        for i in 0..size {
            let low = first[i];
            let mut row = vec![0.0f64; i - low + 1];
            let known = &entries[i];
            for j in low..=i {
                let mut total = known.get(&j).copied().unwrap_or(0.0);
                let other_low = first[j];
                let start = if low > other_low { low } else { other_low };
                if j < i {
                    let other = &rows[j];
                    for k in start..j {
                        total -= row[k - low] * other[k - other_low];
                    }
                    row[j - low] = total / other[j - other_low];
                } else {
                    for k in start..j {
                        total -= row[k - low] * row[k - other_low];
                    }
                    if !(total > 0.0) {
                        return None;
                    }
                    row[i - low] = total.sqrt();
                }
            }
            rows.push(row);
        }
        Some(Profile { first, rows })
    }

    fn solve(&self, right: &[f64]) -> Vec<f64> {
        let size = self.rows.len();
        let mut y = right.to_vec();
        for i in 0..size {
            let (low, row) = (self.first[i], &self.rows[i]);
            let mut total = y[i];
            for k in low..i {
                total -= row[k - low] * y[k];
            }
            y[i] = total / row[i - low];
        }
        for i in (0..size).rev() {
            let (low, row) = (self.first[i], &self.rows[i]);
            y[i] /= row[i - low];
            let value = y[i];
            for k in low..i {
                y[k] -= row[k - low] * value;
            }
        }
        y
    }
}

/// `uv_unfold._order`: reverse Cuthill-McKee over `nodes` (ascending), neighbours only from `nodes`.
fn order_of(nodes: &[usize], neighbours: &BTreeMap<usize, Vec<usize>>) -> Vec<usize> {
    let inside: BTreeSet<usize> = nodes.iter().copied().collect();
    let degree: BTreeMap<usize, usize> =
        nodes.iter().map(|node| (*node, neighbours[node].iter().filter(|other| inside.contains(other) && **other != *node).count())).collect();
    let mut starts = nodes.to_vec();
    starts.sort_by_key(|node| (degree[node], *node));
    let mut seen: BTreeSet<usize> = BTreeSet::new();
    let mut order = Vec::with_capacity(nodes.len());
    for start in starts {
        if seen.contains(&start) {
            continue;
        }
        seen.insert(start);
        let mut queue = vec![start];
        let mut at = 0;
        while at < queue.len() {
            let node = queue[at];
            at += 1;
            order.push(node);
            let mut next: Vec<usize> = neighbours[&node].iter().copied().filter(|item| inside.contains(item) && !seen.contains(item)).collect();
            next.sort_by_key(|item| (degree[item], *item));
            for other in next {
                seen.insert(other);
                queue.push(other);
            }
        }
    }
    order.reverse();
    order
}

type Framed = [(f64, f64); 3];

/// `uv_unfold._triangle`: `(area, gradients in the frame (t̂, n̂r), kind)`; kind 0 ordinary, 1 degenerate, 2 flat `r`, 3 no direction.
fn triangle(p: [[f64; 3]; 3], values_r: [f64; 3], values_s: [f64; 3]) -> (f64, Option<Framed>, u8) {
    let (p0, p1, p2) = (p[0], p[1], p[2]);
    let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
    let l1 = (e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]).sqrt();
    let l2 = (e2[0] * e2[0] + e2[1] * e2[1] + e2[2] * e2[2]).sqrt();
    if !(l1 > 0.0 && l2 > 0.0) {
        return (0.0, None, 1);
    }
    let x2 = (e2[0] * e1[0] + e2[1] * e1[1] + e2[2] * e1[2]) / l1;
    let rest = l2 * l2 - x2 * x2;
    let y2 = if rest > 0.0 { rest.sqrt() } else { 0.0 };
    if !(y2 > DEGENERATE_SINE * l2) {
        return (0.0, None, 1);
    }
    let doubled = l1 * y2;
    let gradients: Framed = [((0.0 - y2) / doubled, (x2 - l1) / doubled), (y2 / doubled, (0.0 - x2) / doubled), (0.0, l1 / doubled)];
    let rx = values_r[0] * gradients[0].0 + values_r[1] * gradients[1].0 + values_r[2] * gradients[2].0;
    let ry = values_r[0] * gradients[0].1 + values_r[1] * gradients[1].1 + values_r[2] * gradients[2].1;
    let sx = values_s[0] * gradients[0].0 + values_s[1] * gradients[1].0 + values_s[2] * gradients[2].0;
    let sy = values_s[0] * gradients[0].1 + values_s[1] * gradients[1].1 + values_s[2] * gradients[2].1;
    let length_r = (rx * rx + ry * ry).sqrt();
    let length_s = (sx * sx + sy * sy).sqrt();
    let (tx, ty, nx, ny, kind);
    if length_r < FLAT_GRADIENT {
        if !(length_s > 0.0) {
            return (0.0, None, 3);
        }
        tx = sx / length_s;
        ty = sy / length_s;
        nx = 0.0 - ty;
        ny = tx;
        kind = 2;
    } else {
        nx = rx / length_r;
        ny = ry / length_r;
        let across = sx * (0.0 - ny) + sy * nx;
        if !(across.abs() > UNORIENTED_SINE * length_s) {
            return (0.0, None, 3);
        }
        if across > 0.0 {
            tx = 0.0 - ny;
            ty = nx;
        } else {
            tx = ny;
            ty = 0.0 - nx;
        }
        kind = 0;
    }
    let framed: Framed = [
        (gradients[0].0 * tx + gradients[0].1 * ty, gradients[0].0 * nx + gradients[0].1 * ny),
        (gradients[1].0 * tx + gradients[1].1 * ty, gradients[1].0 * nx + gradients[1].1 * ny),
        (gradients[2].0 * tx + gradients[2].1 * ty, gradients[2].0 * nx + gradients[2].1 * ny),
    ];
    (0.5 * doubled, Some(framed), kind)
}

struct Triangle {
    nodes: [usize; 3],
    area: f64,
    framed: Option<Framed>,
    orient: f64,
}

fn check(data: &UnfoldInput) -> Result<usize, Malformed> {
    let count = data.r.len();
    if data.positions.len() != 3 * count || data.chart.len() != 2 * count || data.s.len() != count || data.source.len() != count {
        return Err(Malformed("the arrays of the unknowns disagree in length".into()));
    }
    if data.corner_station.len() != data.corners.len() || data.ring_starts.first().copied().unwrap_or(0) != 0 {
        return Err(Malformed("the corners and their stations disagree".into()));
    }
    if data.ring_starts.windows(2).any(|pair| pair[1] < pair[0]) || data.ring_starts.last().copied().unwrap_or(0) != data.corners.len() {
        return Err(Malformed("the ring starts are not a partition of the corners".into()));
    }
    if data.corners.iter().any(|node| *node >= count) {
        return Err(Malformed("a corner names no unknown".into()));
    }
    Ok(count)
}

fn root(parent: &mut [usize], mut node: usize) -> usize {
    while parent[node] != node {
        parent[node] = parent[parent[node]];
        node = parent[node];
    }
    node
}

/// `uv_unfold.unfold`.
pub fn unfold(data: &UnfoldInput) -> Result<UnfoldResult, Malformed> {
    let count = check(data)?;
    let mut numbers = [0i64; 13];
    let chart: Vec<(f64, f64)> = (0..count).map(|node| (data.chart[2 * node], data.chart[2 * node + 1])).collect();
    let mut triangles: Vec<Triangle> = Vec::new();
    for ring in 0..data.ring_starts.len().saturating_sub(1) {
        let (low, high) = (data.ring_starts[ring], data.ring_starts[ring + 1]);
        let ids = &data.corners[low..high];
        let points: Vec<(f64, f64)> = ids.iter().map(|node| chart[*node]).collect();
        let orient = if doubled_area(&points) >= 0.0 { 1.0 } else { -1.0 };
        for [a, b, c] in polyfill(&points) {
            let nodes = [ids[a], ids[b], ids[c]];
            let corner = |node: usize| [data.positions[3 * node], data.positions[3 * node + 1], data.positions[3 * node + 2]];
            let (area, framed, kind) = triangle(
                [corner(nodes[0]), corner(nodes[1]), corner(nodes[2])],
                [data.r[nodes[0]], data.r[nodes[1]], data.r[nodes[2]]],
                [data.corner_station[low + a], data.corner_station[low + b], data.corner_station[low + c]],
            );
            match kind {
                1 => numbers[DEGENERATE] += 1,
                2 => numbers[FLAT] += 1,
                3 => numbers[UNORIENTED] += 1,
                _ => {}
            }
            if framed.is_some() {
                numbers[TRIANGLES] += 1;
            }
            triangles.push(Triangle { nodes, area, framed, orient });
        }
    }
    let mut parent: Vec<usize> = (0..count).collect();
    let mut touched = vec![false; count];
    for item in &triangles {
        if item.framed.is_none() {
            continue;
        }
        for node in item.nodes {
            touched[node] = true;
        }
        for other in [item.nodes[1], item.nodes[2]] {
            let (a, b) = (root(&mut parent, item.nodes[0]), root(&mut parent, other));
            if a != b {
                parent[a.max(b)] = a.min(b);
            }
        }
    }
    let mut islands: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for node in 0..count {
        if touched[node] {
            let top = root(&mut parent, node);
            islands.entry(top).or_default().push(node);
        }
    }
    let mut members: BTreeMap<usize, Vec<(usize, f64, Framed)>> = BTreeMap::new();
    for (index, item) in triangles.iter().enumerate() {
        if let Some(framed) = item.framed {
            let top = root(&mut parent, item.nodes[0]);
            members.entry(top).or_default().push((index, item.area, framed));
        }
    }
    let mut u: Vec<f64> = data.s.clone();
    let mut held = vec![true; count];
    numbers[ISOLATED] = touched.iter().filter(|value| !**value).count() as i64;
    for (island, nodes) in &islands {
        numbers[ISLANDS] += 1;
        let sources: Vec<usize> = nodes.iter().copied().filter(|node| data.source[*node]).collect();
        if sources.is_empty() {
            numbers[WITHOUT_SOURCE] += 1;
        }
        let anchored = data.hold_source && !sources.is_empty();
        let fixed: BTreeSet<usize> = if anchored { sources.iter().copied().collect() } else { [nodes[0]].into_iter().collect() };
        let values: BTreeMap<usize, f64> = fixed.iter().map(|node| (*node, if anchored { data.s[*node] } else { 0.0 })).collect();
        let found = island_solution(data, &triangles, nodes, &members[island], &fixed, &values, &mut numbers);
        let solution = match found {
            Some(solution) => solution,
            None => {
                numbers[SINGULAR] += 1;
                continue;
            }
        };
        let mut shift = 0.0;
        if !anchored {
            let reference: &[usize] = if sources.is_empty() { nodes } else { &sources };
            let mut total = 0.0;
            for node in reference {
                total += data.s[*node] - solution[node];
            }
            shift = total / reference.len() as f64;
        }
        for node in nodes {
            if anchored && fixed.contains(node) {
                continue;
            }
            u[*node] = solution[node] + shift;
            held[*node] = false;
        }
    }
    let (slope_x, slope_y) = slopes(count, &chart, &triangles, &u, &held, &mut numbers);
    flips(&triangles, &u, &data.r, &mut parent, &mut numbers);
    let counters = COUNTER_NAMES.iter().zip(numbers.iter()).filter(|(_, value)| **value != 0).map(|(name, value)| (*name, *value)).collect();
    Ok(UnfoldResult { u, slope_x, slope_y, held, counters })
}

fn along(solution: &BTreeMap<usize, f64>, nodes: [usize; 3], framed: &Framed, axis: usize) -> f64 {
    let pick = |pair: (f64, f64)| if axis == 0 { pair.0 } else { pair.1 };
    solution[&nodes[0]] * pick(framed[0]) + solution[&nodes[1]] * pick(framed[1]) + solution[&nodes[2]] * pick(framed[2])
}

/// `uv_unfold._island_solution`.
fn island_solution(
    data: &UnfoldInput,
    triangles: &[Triangle],
    nodes: &[usize],
    members: &[(usize, f64, Framed)],
    fixed: &BTreeSet<usize>,
    values: &BTreeMap<usize, f64>,
    numbers: &mut [i64; 13],
) -> Option<BTreeMap<usize, f64>> {
    let mut links: BTreeMap<usize, BTreeSet<usize>> = nodes.iter().map(|node| (*node, BTreeSet::new())).collect();
    for (index, _area, _framed) in members {
        let tri = triangles[*index].nodes;
        for i in 0..3 {
            for j in 0..3 {
                links.get_mut(&tri[i]).expect("a node of the island").insert(tri[j]);
            }
        }
    }
    let neighbours: BTreeMap<usize, Vec<usize>> = links.into_iter().map(|(node, items)| (node, items.into_iter().collect())).collect();
    let free: Vec<usize> = nodes.iter().copied().filter(|node| !fixed.contains(node)).collect();
    let order = order_of(&free, &neighbours);
    let place: BTreeMap<usize, usize> = order.iter().enumerate().map(|(at, node)| (*node, at)).collect();
    let mut weights: Vec<f64> = members.iter().map(|(_index, area, _framed)| *area).collect();
    let mut solution: BTreeMap<usize, f64> = values.clone();
    for guard in 0..=data.guard_rounds {
        let mut matrix: BTreeMap<usize, BTreeMap<usize, f64>> = nodes.iter().map(|node| (*node, BTreeMap::new())).collect();
        for ((index, _area, framed), weight) in members.iter().zip(weights.iter()) {
            let tri = triangles[*index].nodes;
            for i in 0..3 {
                let gi = framed[i];
                let row = matrix.get_mut(&tri[i]).expect("a node of the island");
                for j in 0..3 {
                    let gj = framed[j];
                    let entry = row.entry(tri[j]).or_insert(0.0);
                    *entry += weight * (gi.0 * gj.0 + gi.1 * gj.1);
                }
            }
        }
        let mut first = Vec::with_capacity(order.len());
        let mut entries: Vec<BTreeMap<usize, f64>> = Vec::with_capacity(order.len());
        for node in &order {
            let at = place[node];
            let mut row = BTreeMap::new();
            let mut low = at;
            for other in &neighbours[node] {
                if fixed.contains(other) {
                    continue;
                }
                let column = place[other];
                if column <= at {
                    row.insert(column, matrix[node][other]);
                    if column < low {
                        low = column;
                    }
                }
            }
            first.push(low);
            entries.push(row);
        }
        let factor = Profile::factor(first, &entries)?;
        let mut targets: Vec<(f64, f64)> = vec![(1.0, 0.0); members.len()];
        for step in 0..=data.iterations {
            let mut right: BTreeMap<usize, f64> = nodes.iter().map(|node| (*node, 0.0)).collect();
            for (((index, _area, framed), weight), (cx, cy)) in members.iter().zip(weights.iter()).zip(targets.iter()) {
                let tri = triangles[*index].nodes;
                for i in 0..3 {
                    *right.get_mut(&tri[i]).expect("a node of the island") += weight * (framed[i].0 * cx + framed[i].1 * cy);
                }
            }
            let mut constant = Vec::with_capacity(order.len());
            for node in &order {
                let mut total = right[node];
                for other in &neighbours[node] {
                    if fixed.contains(other) {
                        total -= matrix[node][other] * values[other];
                    }
                }
                constant.push(total);
            }
            let answer = factor.solve(&constant);
            for node in &order {
                solution.insert(*node, answer[place[node]]);
            }
            if step == data.iterations {
                break;
            }
            targets.clear();
            for (index, _area, framed) in members {
                let tri = triangles[*index].nodes;
                let gx = along(&solution, tri, framed, 0);
                let gy = along(&solution, tri, framed, 1);
                let rx = data.r[tri[0]] * framed[0].0 + data.r[tri[1]] * framed[1].0 + data.r[tri[2]] * framed[2].0;
                let ry = data.r[tri[0]] * framed[0].1 + data.r[tri[1]] * framed[1].1 + data.r[tri[2]] * framed[2].1;
                let (px, py) = (gx + ry, gy - rx);
                let length = (px * px + py * py).sqrt();
                targets.push(if length > 0.0 { (px / length, py / length) } else { (1.0, 0.0) });
            }
        }
        if guard == data.guard_rounds {
            break;
        }
        let mut squeezed = 0i64;
        for (at, (index, _area, framed)) in members.iter().enumerate() {
            let value = along(&solution, triangles[*index].nodes, framed, 0);
            if value < GUARD_FLOOR || value > GUARD_CEILING {
                weights[at] *= GUARD_FACTOR;
                squeezed += 1;
            }
        }
        if squeezed == 0 {
            break;
        }
        numbers[GUARDED] += squeezed;
    }
    Some(solution)
}

/// `uv_unfold._slopes`.
fn slopes(count: usize, chart: &[(f64, f64)], triangles: &[Triangle], u: &[f64], held: &[bool], numbers: &mut [i64; 13]) -> (Vec<f64>, Vec<f64>) {
    let mut sum_x = vec![0.0f64; count];
    let mut sum_y = vec![0.0f64; count];
    let mut weight = vec![0.0f64; count];
    for item in triangles {
        if item.framed.is_none() {
            continue;
        }
        let [a, b, c] = item.nodes;
        let ((ax, ay), (bx, by), (cx, cy)) = (chart[a], chart[b], chart[c]);
        let doubled = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
        let edges = (bx - ax) * (bx - ax) + (by - ay) * (by - ay) + (cx - bx) * (cx - bx) + (cy - by) * (cy - by) + (ax - cx) * (ax - cx) + (ay - cy) * (ay - cy);
        if !(doubled.abs() > DEGENERATE_SINE * edges) {
            numbers[SLOPE_SKIPPED] += 1;
            continue;
        }
        let (ua, ub, uc) = (u[a], u[b], u[c]);
        let gx = ((ub - ua) * (cy - ay) - (uc - ua) * (by - ay)) / doubled;
        let gy = ((uc - ua) * (bx - ax) - (ub - ua) * (cx - ax)) / doubled;
        let area = doubled.abs();
        for node in item.nodes {
            sum_x[node] += area * gx;
            sum_y[node] += area * gy;
            weight[node] += area;
        }
    }
    let pick = |sums: &[f64]| -> Vec<f64> { (0..count).map(|node| if held[node] || !(weight[node] > 0.0) { 0.0 } else { sums[node] / weight[node] }).collect() };
    (pick(&sum_x), pick(&sum_y))
}

/// `uv_unfold._flips`.
fn flips(triangles: &[Triangle], u: &[f64], values_r: &[f64], parent: &mut [usize], numbers: &mut [i64; 13]) {
    let mut balance: BTreeMap<usize, f64> = BTreeMap::new();
    let mut signed: Vec<(usize, f64)> = Vec::with_capacity(triangles.len());
    for item in triangles {
        let [a, b, c] = item.nodes;
        let area = item.orient * ((u[b] - u[a]) * (values_r[c] - values_r[a]) - (u[c] - u[a]) * (values_r[b] - values_r[a]));
        let island = root(parent, a);
        signed.push((island, area));
        *balance.entry(island).or_insert(0.0) += area;
    }
    let mut flipped_area = 0.0f64;
    for (island, area) in signed {
        if area == 0.0 {
            numbers[ZERO_AREA] += 1;
        } else if (area > 0.0) != (balance[&island] >= 0.0) {
            numbers[FLIPPED] += 1;
            flipped_area += area.abs() * 0.5;
        }
    }
    numbers[FLIPPED_AREA] = (flipped_area * 1e6).ceil() as i64;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_convex_pentagon_is_cut_like_blender_and_a_quad_on_its_first_diagonal() {
        let pentagon = [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (1.5, 2.5), (-0.5, 1.0)];
        let triangles = polyfill(&pentagon);
        assert_eq!(triangles.len(), 3);
        assert_eq!(triangles[0], [4, 0, 1]);
        assert_eq!(polyfill(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]), vec![[0, 1, 2], [0, 2, 3]]);
        // reflex at `2`: the diagonal `0-2` is inside; reflex at `3` (`1` and `3` on one side of `0-2`): Blender takes `1-3`.
        assert_eq!(polyfill(&[(0.0, 0.0), (1.0, 0.0), (0.2, 0.2), (0.0, 1.0)]), vec![[0, 1, 2], [0, 2, 3]]);
        assert_eq!(polyfill(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.8, 0.2)]), vec![[0, 1, 3], [1, 2, 3]]);
    }

    #[test]
    fn the_ears_of_a_concave_ring_cover_its_area() {
        let ring = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (2.0, 1.0), (0.0, 4.0)];
        let mut twice = 0.0;
        for [a, b, c] in polyfill(&ring) {
            let area = cross(ring[b].0 - ring[a].0, ring[b].1 - ring[a].1, ring[c].0 - ring[a].0, ring[c].1 - ring[a].1);
            assert!(area > 0.0);
            twice += area;
        }
        assert_eq!(twice, 2.0 * 10.0);
    }

    /// A strip `0 <= x <= 3`, `0 <= r <= 1` in the plane `z = 0`: `u = x` everywhere with the source held, to rounding.
    fn strip(hold_source: bool) -> UnfoldInput {
        let mut data = UnfoldInput { hold_source, iterations: 2, guard_rounds: 2, ..UnfoldInput::default() };
        for row in 0..2 {
            for column in 0..4 {
                let (x, y) = (column as f64, row as f64);
                data.positions.extend([x, y, 0.0]);
                data.chart.extend([x, y]);
                data.r.push(y);
                data.s.push(x);
                data.source.push(row == 0);
            }
        }
        data.ring_starts.push(0);
        for column in 0..3 {
            for node in [column, column + 1, column + 5, column + 4] {
                data.corners.push(node);
                data.corner_station.push(data.chart[2 * node]);
            }
            data.ring_starts.push(data.corners.len());
        }
        data
    }

    #[test]
    fn a_straight_strip_unfolds_to_its_arclength_held_or_free() {
        for hold in [true, false] {
            let result = unfold(&strip(hold)).expect("a well formed strip");
            for node in 0..8 {
                assert!((result.u[node] - (node % 4) as f64).abs() < 1e-12, "{hold} {node} {}", result.u[node]);
            }
            assert!(!result.counters.iter().any(|(name, _)| name.contains("FLIPPED_TRIANGLES")));
            assert_eq!(result.held[0], hold);
        }
    }

    #[test]
    fn a_malformed_input_is_named() {
        let mut data = strip(true);
        data.corners[0] = 99;
        assert!(unfold(&data).is_err());
    }
}
