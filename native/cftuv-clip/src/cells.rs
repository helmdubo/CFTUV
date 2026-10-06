//! `materialize/clip_cells.py`: the cut cells of the `SOURCE_FACES_CLIPPED_V1` law (the convex face of the source
//! instead of its triangles), the diagonal depth estimates (`hinge_depth_square`, `chord_of`) and the pure-`Fraction`
//! setup (`build_cells` with its loop, hinge and flatness estimates and the memo shared by the two stages).
//!
//! Insertion order is semantic here: `_loop_of` walks a dictionary with deletions (`successor` start = the first
//! surviving half-edge), `build_cells` walks the faces in first-appearance order, the memo is an ordered dictionary
//! that the caller compares. [`OrderedMap`] has Python-dict order semantics.

use std::collections::{HashMap, HashSet};

use cftuv_canon::ordered::OrderedMap;
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::{ClipError, ClipResult};
use crate::numeric::ENCLOSURE_BITS;
use crate::plane::{ChartPoint, Triangle};

pub const NO_HINGE: &str = "NO_HINGE";
pub const NOT_CONVEX: &str = "NOT_CONVEX";
pub const NOT_ONE_LOOP: &str = "NOT_ONE_LOOP";
pub const MIXED_WINDING: &str = "MIXED_WINDING";

/// `ClipCellV1.key` / `.group`: `("t", index)`, `("f", face, first index)`, `("g", face, first index)` or the group of the
/// chain station plan, `("p", smallest face name)`.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum CellKey {
    Triangle(usize),
    Face(String, usize),
    Group(String, usize),
    Plan(String),
}

/// `HingeV1`: the diagonal `edge` of triangle `triangle` and `|k|^2`.
#[derive(Clone, Debug, PartialEq)]
pub struct Hinge {
    pub triangle: usize,
    pub edge: usize,
    pub jump_square: Rat,
}

/// `ClipCellV1`.
#[derive(Clone, Debug, PartialEq)]
pub struct ClipCell {
    pub key: CellKey,
    pub name: String,
    pub chart: Vec<ChartPoint>,
    pub bbox: [f64; 4],
    pub twice_area: Rat,
    pub members: Vec<usize>,
    pub diagonals: Vec<(usize, usize)>,
    pub hinge: Option<Hinge>,
    pub flat_square: Option<Rat>,
    pub straight: Vec<(usize, ChartPoint)>,
    pub group: Option<CellKey>,
}

/// `CellPlanV1`.
#[derive(Clone, Debug, PartialEq)]
pub struct CellPlan {
    pub cells: Vec<ClipCell>,
    /// `(face, reason)` of the faces that merged neither into a cell nor into a group.
    pub unmergeable: Vec<(String, &'static str)>,
    /// `plan_pairs`: the pairs of faces of the chain station plan (`CHAIN_STATION_PLAN_V1`) glued into groups; zero: no plan.
    pub plan_pairs: u64,
}

/// What `_merged_cell` answers: a cell or the reason it is none.
#[derive(Clone, Debug, PartialEq)]
pub enum Built {
    Cell(Box<ClipCell>),
    Reason(&'static str),
}

/// The keys of the caller's `memo` dictionary: `("cell", face)` and `("flat", face)`.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum MemoKey {
    Cell(String),
    Flat(String),
}

#[derive(Clone, Debug, PartialEq)]
pub enum MemoValue {
    Built(Built),
    Flat(Rat),
}

/// The memo of the two stages of one clip: a dictionary of the oracle's insertion order.
pub type CellMemo = OrderedMap<MemoKey, MemoValue>;

fn py_min(current: f64, candidate: f64) -> f64 {
    if candidate < current {
        candidate
    } else {
        current
    }
}

fn py_max(current: f64, candidate: f64) -> f64 {
    if candidate > current {
        candidate
    } else {
        current
    }
}

/// `clip_cells._edge_value(start, end, point)` on fractions: `(end - start) x (point - start)`.
fn fraction_edge_value(start: &ChartPoint, end: &ChartPoint, point: &ChartPoint) -> Rat {
    end.0.sub(&start.0).mul(&point.1.sub(&start.1)).sub(&end.1.sub(&start.1).mul(&point.0.sub(&start.0)))
}

/// `_affine(triangle, point)`: the affine lift of the triangle at a chart point, `(e1*A + e2*B + e0*C) / D` per axis.
fn affine(triangle: &Triangle, point: &ChartPoint) -> ClipResult<[Rat; 3]> {
    let values: Vec<Rat> = (0..3).map(|index| fraction_edge_value(&triangle.chart[index], &triangle.chart[(index + 1) % 3], point)).collect();
    let weights = [&values[1], &values[2], &values[0]];
    let mut out = Vec::with_capacity(3);
    for axis in 0..3 {
        let mut total = Rat::zero();
        for (weight, corner) in weights.iter().zip(triangle.corners.iter()) {
            total = total.add(&weight.mul(&corner[axis]));
        }
        out.push(total.div(&triangle.twice_area).map_err(|_| ClipError::Value("a degenerate source triangle"))?);
    }
    let [x, y, z]: [Rat; 3] = out.try_into().expect("three axes");
    Ok([x, y, z])
}

pub(crate) fn single(triangles: &[Triangle], index: usize, group: Option<CellKey>, flat: Option<Rat>) -> ClipCell {
    let item = &triangles[index];
    ClipCell {
        key: CellKey::Triangle(index),
        name: item.name.clone(),
        chart: item.chart.to_vec(),
        bbox: item.bbox,
        twice_area: item.twice_area.clone(),
        members: vec![index],
        diagonals: Vec::new(),
        hinge: None,
        flat_square: flat,
        straight: Vec::new(),
        group,
    }
}

type Vertex = (ChartPoint, [Rat; 3]);
/// The loop of vertices and the diagonals (`(triangle, position)` of the first owner of each cancelled edge).
type Loop = (Vec<Vertex>, Vec<(usize, usize)>);

/// `_loop_of(triangles, members)`: the one simple loop after the shared edges cancel as opposite half-edges, with the
/// diagonals (the cancelled edges, `(triangle, position)` of the first owner), or `None`.
fn loop_of(triangles: &[Triangle], members: &[usize]) -> Option<Loop> {
    let mut numbers: HashMap<Vertex, usize> = HashMap::new();
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut number = |item: &Triangle, position: usize| -> usize {
        let vertex: Vertex = (item.chart[position].clone(), item.corners[position].clone());
        if let Some(found) = numbers.get(&vertex) {
            return *found;
        }
        let found = vertices.len();
        numbers.insert(vertex.clone(), found);
        vertices.push(vertex);
        found
    };
    let mut directed: OrderedMap<(usize, usize), (usize, usize)> = OrderedMap::new();
    let mut diagonals = Vec::new();
    for &index in members {
        let item = &triangles[index];
        for position in 0..3 {
            let first = number(item, position);
            let second = number(item, (position + 1) % 3);
            if let Some(owner) = directed.remove(&(second, first)) {
                diagonals.push(owner);
            } else if directed.contains_key(&(first, second)) {
                return None;
            } else {
                directed.set((first, second), (index, position));
            }
        }
    }
    let mut successor: OrderedMap<usize, usize> = OrderedMap::new();
    for ((first, second), _owner) in directed.iter() {
        if successor.contains_key(first) {
            return None;
        }
        successor.set(*first, *second);
    }
    if successor.len() == 0 {
        return None;
    }
    let start = *successor.keys().next().expect("a non-empty successor map");
    let mut walk = vec![start];
    let mut current = *successor.get(&start).expect("start is a key");
    while current != start {
        if walk.contains(&current) {
            return None;
        }
        let next = successor.get(&current)?;
        walk.push(current);
        current = *next;
    }
    if walk.len() == successor.len() {
        Some((walk.into_iter().map(|found| vertices[found].clone()).collect(), diagonals))
    } else {
        None
    }
}

/// `_hinge(triangles, diagonal, members)`: the crease of a two-triangle cell, or `None` when the other triangle's apex
/// is not strictly behind the diagonal.
fn hinge(triangles: &[Triangle], diagonal: (usize, usize), members: &[usize]) -> ClipResult<Option<Hinge>> {
    let (owner, edge) = diagonal;
    let first = &triangles[owner];
    let other_index = members.iter().copied().find(|index| *index != owner).ok_or_else(|| ClipError::Unsupported("a hinge cell without a second triangle".into()))?;
    let other = &triangles[other_index];
    let (start, end) = (&first.chart[edge], &first.chart[(edge + 1) % 3]);
    let apex = (0..3)
        .find(|index| &other.chart[*index] != start && &other.chart[*index] != end)
        .ok_or_else(|| ClipError::Unsupported("a hinge cell whose second triangle has no apex".into()))?;
    let (point, corner) = (&other.chart[apex], &other.corners[apex]);
    let value = fraction_edge_value(start, end, point);
    let reach = if first.twice_area.signum() > 0 { value } else { value.neg() };
    if reach.signum() >= 0 {
        return Ok(None);
    }
    let base = affine(first, point)?;
    let mut jump_square = Rat::zero();
    for axis in 0..3 {
        let component = corner[axis].sub(&base[axis]).div(&reach).map_err(|_| ClipError::Value("a degenerate hinge"))?;
        jump_square = jump_square.add(&component.mul(&component));
    }
    Ok(Some(Hinge { triangle: owner, edge, jump_square }))
}

/// `_flat_square(triangles, members)`: `(2 rho)^2`, `rho` the largest deviation of a cell corner from the affine lift of
/// the largest triangle of the cell (the first of equal areas).
fn flat_square(triangles: &[Triangle], members: &[usize]) -> ClipResult<Rat> {
    let mut big = members[0];
    let mut biggest = triangles[big].twice_area.clone();
    if biggest.signum() < 0 {
        biggest = biggest.neg();
    }
    for &index in &members[1..] {
        let area = triangles[index].twice_area.clone();
        let area = if area.signum() < 0 { area.neg() } else { area };
        if area > biggest {
            big = index;
            biggest = area;
        }
    }
    let mut worst = Rat::zero();
    for &index in members {
        for (point, corner) in triangles[index].chart.iter().zip(triangles[index].corners.iter()) {
            let base = affine(&triangles[big], point)?;
            let mut total = Rat::zero();
            for axis in 0..3 {
                let difference = corner[axis].sub(&base[axis]);
                total = total.add(&difference.mul(&difference));
            }
            if total > worst {
                worst = total;
            }
        }
    }
    Ok(worst.mul(&Rat::from_i64(4)))
}

/// `_merged_cell(triangles, face, members)`: the cell of the triangles of one face, or the reason it is none.
fn merged_cell(triangles: &[Triangle], face: &str, members: &[usize]) -> ClipResult<Built> {
    let positive = |index: usize| triangles[index].twice_area.signum() > 0;
    let sign: i8 = if positive(members[0]) { 1 } else { -1 };
    if members.iter().any(|index| positive(*index) != (sign > 0)) {
        return Ok(Built::Reason(MIXED_WINDING));
    }
    let Some((loop_vertices, diagonals)) = loop_of(triangles, members) else {
        return Ok(Built::Reason(NOT_ONE_LOOP));
    };
    let chart: Vec<ChartPoint> = loop_vertices.into_iter().map(|vertex| vertex.0).collect();
    let size = chart.len();
    let mut straight = Vec::new();
    for position in 0..size {
        let (a, b, c) = (&chart[position], &chart[(position + 1) % size], &chart[(position + 2) % size]);
        let turn = b.0.sub(&a.0).mul(&c.1.sub(&b.1)).sub(&b.1.sub(&a.1).mul(&c.0.sub(&b.0)));
        if turn.signum() * sign < 0 {
            return Ok(Built::Reason(NOT_CONVEX));
        }
        if turn.is_zero() {
            straight.push(((position + 1) % size, b.clone()));
        }
    }
    let boxes: Vec<[f64; 4]> = members.iter().map(|index| triangles[*index].bbox).collect();
    let found = if members.len() == 2 && diagonals.len() == 1 { hinge(triangles, diagonals[0], members)? } else { None };
    if members.len() == 2 && found.is_none() {
        return Ok(Built::Reason(NO_HINGE));
    }
    let mut bbox = boxes[0];
    for item in &boxes {
        bbox = [py_min(bbox[0], item[0]), py_max(bbox[1], item[1]), py_min(bbox[2], item[2]), py_max(bbox[3], item[3])];
    }
    let twice_area = members.iter().fold(Rat::zero(), |total, index| total.add(&triangles[*index].twice_area));
    let flat = if found.is_some() { None } else { Some(flat_square(triangles, members)?) };
    Ok(Built::Cell(Box::new(ClipCell {
        key: CellKey::Face(face.to_string(), members[0]),
        name: face.to_string(),
        chart,
        bbox,
        twice_area,
        members: members.to_vec(),
        diagonals,
        hinge: found,
        flat_square: flat,
        straight,
        group: None,
    })))
}

/// The pairs of faces of the chain station plan, in the iteration order of the oracle's frozenset (`inert`): only the order of the
/// memo writes the oracle makes while it asks `usable` depends on it, the groups and the count do not.
pub type InertPairs = [(String, String)];

/// `_merged_cell(...)` kept in the caller's memo under `("cell", face)`: the cell or the reason, computed once and shared by both stages.
fn memoized<'m>(triangles: &[Triangle], face: &str, members: &[usize], memo: &'m mut CellMemo) -> ClipResult<&'m Built> {
    let key = MemoKey::Cell(face.to_string());
    if !matches!(memo.get(&key), Some(MemoValue::Built(_))) {
        let made = merged_cell(triangles, face, members)?;
        memo.set(key.clone(), MemoValue::Built(made));
    }
    match memo.get(&key) {
        Some(MemoValue::Built(found)) => Ok(found),
        _ => unreachable!("the entry was just stored"),
    }
}

/// `_non_convex_flat(triangles, face, members, memo)`: `(2 rho)^2` of a non-convex face (kept in `memo`), zero for a convex face.
fn non_convex_flat(triangles: &[Triangle], face: &str, members: &[usize], memo: &mut CellMemo) -> ClipResult<Rat> {
    if !matches!(memoized(triangles, face, members, memo)?, Built::Reason(reason) if *reason == NOT_CONVEX) {
        return Ok(Rat::zero());
    }
    let key = MemoKey::Flat(face.to_string());
    if let Some(MemoValue::Flat(found)) = memo.get(&key) {
        return Ok(found.clone());
    }
    let made = flat_square(triangles, members)?;
    memo.set(key, MemoValue::Flat(made.clone()));
    Ok(made)
}

/// The number of a face name in the union-find of `plan_groups` (`parent.setdefault(name, name)`).
fn slot_of<'a>(known: &mut HashMap<&'a str, usize>, labels: &mut Vec<&'a str>, parent: &mut Vec<usize>, name: &'a str) -> usize {
    *known.entry(name).or_insert_with(|| {
        parent.push(parent.len());
        labels.push(name);
        parent.len() - 1
    })
}

fn root_of(parent: &mut [usize], mut item: usize) -> usize {
    while parent[item] != item {
        parent[item] = parent[parent[item]];
        item = parent[item];
    }
    item
}

/// `_plan_groups(names, inert, usable)`: `(({face: group key} in the oracle's dictionary order), glued pairs)`. A pair counts when both faces
/// are the domain's and `usable` (the oracle's short-circuit order is kept: the first face is asked before the second); a connected
/// component of two or more faces is a group keyed by its smallest name.
fn plan_groups(names: &HashMap<String, usize>, inert: &InertPairs, usable: &mut dyn FnMut(&str) -> ClipResult<bool>) -> ClipResult<(Vec<(String, String)>, u64)> {
    let mut known: HashMap<&str, usize> = HashMap::new();
    let mut labels: Vec<&str> = Vec::new();
    let mut parent: Vec<usize> = Vec::new();
    let mut pairs = 0u64;
    for (left, right) in inert {
        let (first, second) = if left <= right { (left.as_str(), right.as_str()) } else { (right.as_str(), left.as_str()) };
        if names.contains_key(first) && names.contains_key(second) && usable(first)? && usable(second)? {
            let (first_slot, second_slot) = (slot_of(&mut known, &mut labels, &mut parent, first), slot_of(&mut known, &mut labels, &mut parent, second));
            let (first_root, second_root) = (root_of(&mut parent, first_slot), root_of(&mut parent, second_slot));
            parent[first_root] = second_root;
            pairs += 1;
        }
    }
    let mut order: Vec<usize> = (0..labels.len()).collect();
    order.sort_by(|a, b| labels[*a].cmp(labels[*b]));
    let mut found: Vec<(usize, Vec<usize>)> = Vec::new();
    for slot in order {
        let head = root_of(&mut parent, slot);
        match found.iter_mut().find(|(known, _)| *known == head) {
            Some((_, members)) => members.push(slot),
            None => found.push((head, vec![slot])),
        }
    }
    let mut keys = Vec::new();
    for (_, members) in found.iter().filter(|(_, members)| members.len() >= 2) {
        // the members are in sorted order: the first one is the smallest name
        let key = labels[members[0]];
        keys.extend(members.iter().map(|slot| (labels[*slot].to_string(), key.to_string())));
    }
    let glued = if keys.is_empty() { 0 } else { pairs };
    Ok((keys, glued))
}

/// `build_cells(triangles, split, memo, inert)`: the cut cells of the lifted triangles; the cells and groups whose keys are in
/// `split` stay triangles. The faces are walked in first-appearance order (an empty face is its own face). `inert` are the pairs of faces of
/// the chain station plan: the faces of a group stay triangles of ONE group, the edges between them do not cut, and the estimate of the group
/// is the largest `(2 rho)^2` of its non-convex faces (zero otherwise).
pub fn build_cells(triangles: &[Triangle], split: &HashSet<CellKey>, memo: &mut CellMemo, inert: &InertPairs) -> ClipResult<CellPlan> {
    let mut groups: Vec<(String, Vec<usize>)> = Vec::new();
    let mut lookup: HashMap<String, usize> = HashMap::new();
    for (index, item) in triangles.iter().enumerate() {
        let face = if item.face.is_empty() { format!("\0{index}") } else { item.face.clone() };
        match lookup.get(&face) {
            Some(slot) => groups[*slot].1.push(index),
            None => {
                lookup.insert(face.clone(), groups.len());
                groups.push((face, vec![index]));
            }
        }
    }
    let (planned, plan_pairs) = if inert.is_empty() {
        (Vec::new(), 0)
    } else {
        // `usable(name)`: a lone triangle, a merged convex cell or a non-convex face
        let mut usable = |name: &str| -> ClipResult<bool> {
            let members = &groups[lookup[name]].1;
            if members.len() < 2 {
                return Ok(true);
            }
            Ok(match memoized(triangles, name, members, memo)? {
                Built::Cell(_) => true,
                Built::Reason(reason) => *reason == NOT_CONVEX,
            })
        };
        plan_groups(&lookup, inert, &mut usable)?
    };
    let mut group_flat: HashMap<&str, Rat> = HashMap::new();
    for (name, key) in &planned {
        let members = &groups[lookup[name.as_str()]].1;
        let candidate = if members.len() > 1 { non_convex_flat(triangles, name, members, memo)? } else { Rat::zero() };
        let current = group_flat.get(key.as_str()).cloned().unwrap_or_else(Rat::zero);
        group_flat.insert(key.as_str(), if candidate > current { candidate } else { current });
    }
    let planned_key: HashMap<&str, &str> = planned.iter().map(|(name, key)| (name.as_str(), key.as_str())).collect();
    let mut cells: Vec<(usize, ClipCell)> = Vec::new();
    let mut unmergeable = Vec::new();
    for (face, members) in &groups {
        if let Some(key) = planned_key.get(face.as_str()) {
            let group = CellKey::Plan((*key).to_string());
            if !split.contains(&group) {
                let flat = group_flat[key].clone();
                cells.extend(members.iter().map(|index| (*index, single(triangles, *index, Some(group.clone()), Some(flat.clone())))));
                continue;
            }
        }
        let built = if members.len() == 1 {
            None
        } else {
            let key = MemoKey::Cell(face.clone());
            match memo.get(&key) {
                Some(MemoValue::Built(found)) => Some(found.clone()),
                _ => {
                    let made = merged_cell(triangles, face, members)?;
                    memo.set(key, MemoValue::Built(made.clone()));
                    Some(made)
                }
            }
        };
        match built {
            None => cells.push((members[0], single(triangles, members[0], None, None))),
            Some(Built::Cell(cell)) => {
                if split.contains(&cell.key) {
                    cells.extend(members.iter().map(|index| (*index, single(triangles, *index, None, None))));
                } else {
                    cells.push((members[0], *cell));
                }
            }
            Some(Built::Reason(reason)) if reason == NOT_CONVEX && !split.contains(&CellKey::Group(face.clone(), members[0])) => {
                let flat_key = MemoKey::Flat(face.clone());
                let flat = match memo.get(&flat_key) {
                    Some(MemoValue::Flat(found)) => found.clone(),
                    _ => {
                        let made = flat_square(triangles, members)?;
                        memo.set(flat_key, MemoValue::Flat(made.clone()));
                        made
                    }
                };
                let group = CellKey::Group(face.clone(), members[0]);
                cells.extend(members.iter().map(|index| (*index, single(triangles, *index, Some(group.clone()), Some(flat.clone())))));
            }
            Some(Built::Reason(reason)) => {
                if reason != NOT_CONVEX {
                    unmergeable.push((face.clone(), reason));
                }
                cells.extend(members.iter().map(|index| (*index, single(triangles, *index, None, None))));
            }
        }
    }
    cells.sort_by_key(|entry| entry.0);
    Ok(CellPlan { cells: cells.into_iter().map(|entry| entry.1).collect(), unmergeable, plan_pairs })
}

/// `hinge_depth_square(jump_square, column)`: `|k|^2 (a b / (a + b))^2` from the enclosures of the orientation values
/// of a piece's vertices: an upper bound, zero when every vertex is on one side.
pub fn hinge_depth_square<V: std::borrow::Borrow<SqrtSum>>(jump_square: &Rat, column: &[V]) -> ClipResult<Rat> {
    let mut behind: Option<Rat> = None;
    let mut ahead: Option<Rat> = None;
    for item in column {
        let (low, high) = item.borrow().enclosure(ENCLOSURE_BITS);
        let negated = low.neg();
        behind = Some(match behind {
            Some(current) if !(negated > current) => current,
            _ => negated,
        });
        ahead = Some(match ahead {
            Some(current) if !(high > current) => current,
            _ => high,
        });
    }
    let (Some(behind), Some(ahead)) = (behind, ahead) else {
        return Err(ClipError::Value("a hinge column without a vertex"));
    };
    if behind.signum() <= 0 || ahead.signum() <= 0 {
        return Ok(Rat::zero());
    }
    let gap = behind.mul(&ahead).div(&behind.add(&ahead)).expect("a positive sum");
    Ok(jump_square.mul(&gap).mul(&gap))
}

/// `chord_of(cell, values, budget)`: `(chord depth square, diagonals the piece crosses)`. Every item of every column is
/// signed (a list comprehension, no short circuit): the signs are the cost. A crossing is a column whose items lie
/// strictly on both sides. `flat_square` is `None` for a cell without an estimate (a lone triangle).
pub fn chord_of<V: std::borrow::Borrow<SqrtSum>>(ctx: &mut ExactCtx<'_>, hinge_jump: Option<&Rat>, flat: Option<&Rat>, values: &[Vec<V>]) -> ClipResult<(Option<Rat>, i64)> {
    let mut crossings = 0i64;
    for column in values {
        let mut signs = Vec::with_capacity(column.len());
        for item in column {
            signs.push(exact::sign(ctx, item.borrow(), SIGN_FILTER_BITS)?);
        }
        let (Some(least), Some(greatest)) = (signs.iter().min(), signs.iter().max()) else {
            return Err(ClipError::Value("min() arg is an empty sequence"));
        };
        crossings += i64::from(*least < 0 && 0 < *greatest);
    }
    if let Some(jump) = hinge_jump {
        let first = values.first().ok_or(ClipError::Value("a hinge cell without a diagonal column"))?;
        return Ok((Some(hinge_depth_square(jump, first)?), crossings));
    }
    Ok((flat.cloned(), crossings))
}
