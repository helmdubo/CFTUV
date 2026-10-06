//! `ClipStageV1` (`materialize/clip.py`), first half: the constructor, the node arena, the signs of points against
//! the edges of the cut regions (`_slot` and its filters), the crossings of segments with those lines, and the
//! subdivision of edges (`edge_points`). The cut itself is `cut.rs`, the emission `emit.rs`.
//!
//! IDENTITY. The oracle's `_Node` has no `__eq__` and is interned by `point_key`: one object per point, compared with
//! `is` and used as a dictionary key. Here a node is a [`NodeId`] into an arena that the two stages of a clip share
//! (stage 2 takes the `by_point`, `crossings`, `chords`, `snap` and the gap maxima of stage 1 and COPIES its tally), and an
//! unordered pair of nodes (a `frozenset` of two) is the normalised pair `(min, max)`.
//!
//! COST. Every exact question is asked where and in the order the oracle asks it. The one place whose question sequence
//! depends on the interpreter is `_ordered` (the sort), through `order::ordered` and the version in `PyVersion`.

use std::rc::Rc;
use std::sync::Arc;

use cftuv_core::exact::{self, ExactCtx, Quotient};
use cftuv_core::fused::{product_added, product_added_form};
use cftuv_core::num::UBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::cells::CellKey;
use crate::edge::cheap_sign_with;
use crate::error::{ClipError, ClipResult};
use crate::fxhash::{FxMap, FxSet};
use crate::lift::Lifted;
use crate::numeric::{self, nanometres};
use crate::order;
use crate::plane::{ChartPoint, Plane};
use crate::point::{point_hash, rational_pair, same_point, Point, RationalPair};
use crate::profile::{scope, Phase};
use crate::regions::RegionSet;
use crate::warm::{crossing_key, value_key, CrossingEntry, ValueEntry, Warm};
use crate::regions::EdgeLine;
use crate::pyemu::PyVersion;
use crate::snap::{self, CornerSnap};

pub type NodeId = u32;
/// An unordered pair of nodes: `frozenset((a, b))`.
pub type Pair = (NodeId, NodeId);

pub fn pair(first: NodeId, second: NodeId) -> Pair {
    if first <= second {
        (first, second)
    } else {
        (second, first)
    }
}

/// `(sign inward, zeroed by the tolerance)` of a node at an edge of a region.
#[derive(Clone, Copy, Debug)]
pub struct Slot {
    pub sign: i8,
    pub zeroed: bool,
}

/// `_Node`: the point (the first object interned for its `point_key`), its key, and the per-region caches.
pub struct Node {
    pub point: Arc<Point>,
    pub key: Option<Rc<str>>,
    /// `key.startswith("node:")`
    pub watched: bool,
    /// `key.startswith("src:")`
    pub source: bool,
    pub window: Option<[f64; 4]>,
    /// The triangle on whose edge the node was born at the subdivision of an edge (the lift happens there).
    pub home: Option<usize>,
    /// The index in `points` (the input dictionary) of the Python object this node's point IS, when the representative
    /// came from the input unchanged: the host hands that very object back instead of building an equal one.
    pub origin: Option<u32>,
    /// The hash of the identity of the point (`point::point_hash`).
    pub hash: u64,
    pub rational: Option<RationalPair>,
    /// `{(region key, edge index): slot}`
    pub cache: FxMap<u64, Slot>,
    /// `{(region key, edge index): exact orientation value}`
    pub values: FxMap<u64, Arc<SqrtSum>>,
}

/// The counters the stage keeps in `self.tally` (a `Counter` in the oracle).
#[derive(Clone, Debug, Default)]
pub struct Tally {
    pub vertices_at_source_vertex: u64,
    pub edges_refined: u64,
    pub faces_in_one_triangle: u64,
    pub faces_cut: u64,
    pub faces_cut_by_ears: u64,
    pub pieces_emitted: u64,
    pub pieces_merged: u64,
    pub pieces_kept_separate: u64,
    pub faces_overhang: u64,
    pub faces_boundary_mismatch: u64,
    pub faces_seam_suppressed: u64,
    pub faces_off_corner_suppressed: u64,
    pub flow_free_cut_edges: u64,
    pub predicates: u64,
    pub divisions: u64,
    pub node_signs_zeroed: u64,
}

/// What stage 2 of a clip inherits from stage 1 (`shared=`): the nodes with their caches and everything keyed by them.
pub struct Shared {
    pub(crate) point_ids: FxMap<ChartPoint, u32>,
    pub(crate) line_of: FxMap<Pair, u32>,
    pub(crate) crossings: FxMap<(NodeId, NodeId, u32), NodeId>,
    pub(crate) by_point: FxMap<u64, Vec<NodeId>>,
    pub(crate) nodes: Vec<Node>,
    pub(crate) chords: FxMap<(u32, Vec<NodeId>), (Rat, i64)>,
    pub(crate) snap: CornerSnap,
    pub(crate) node_gap: UBig,
    pub(crate) node_gap_square: Rat,
    pub(crate) tally: Tally,
    pub(crate) region_key_ids: FxMap<CellKey, u32>,
}

/// `DiagonalVerdictV1`.
#[derive(Clone, Debug, Default)]
pub struct Verdict {
    /// `{cell or group key: the largest squared chord depth}` of the cells split into triangles by the tolerance.
    pub over: Vec<(CellKey, Rat)>,
    pub unmergeable: Vec<(String, &'static str)>,
}

/// One write of `plane._normal_by_position[position] = normal`, in the order of the calls.
#[derive(Clone, Debug, PartialEq)]
pub struct NormalWrite {
    pub position: [f64; 3],
    pub normal: [f64; 3],
}

pub struct Stage<'a, 'c> {
    pub(crate) plane: &'a Plane,
    pub(crate) ctx: &'a mut ExactCtx<'c>,
    pub(crate) warm: &'a mut Warm,
    pub(crate) version: PyVersion,
    pub flows: Option<Vec<bool>>,
    pub(crate) faces_mode: bool,
    pub(crate) regions: Arc<RegionSet>,
    pub(crate) region_keys: Vec<u32>,
    pub(crate) interior: Vec<Vec<bool>>,
    pub(crate) inert: Vec<Vec<bool>>,
    pub(crate) line_ids: Vec<Vec<u32>>,
    pub(crate) corners: FxSet<ChartPoint>,
    pub(crate) straights: Vec<Vec<(usize, NodeId)>>,
    // the part stage 2 takes over
    pub(crate) point_ids: FxMap<ChartPoint, u32>,
    pub(crate) line_of: FxMap<Pair, u32>,
    pub(crate) crossings: FxMap<(NodeId, NodeId, u32), NodeId>,
    pub(crate) by_point: FxMap<u64, Vec<NodeId>>,
    pub(crate) nodes: Vec<Node>,
    pub(crate) chords: FxMap<(u32, Vec<NodeId>), (Rat, i64)>,
    pub(crate) snap: CornerSnap,
    pub(crate) node_gap: UBig,
    pub(crate) node_gap_square: Rat,
    pub(crate) tally: Tally,
    pub(crate) region_key_ids: FxMap<CellKey, u32>,
    // the stage's own
    pub(crate) node_of_key: std::collections::HashMap<String, NodeId>,
    pub(crate) edge_cache: FxMap<Pair2, Rc<Vec<NodeId>>>,
    pub(crate) needed: FxSet<Pair>,
    pub(crate) new_points: Vec<(Rc<str>, NodeId)>,
    pub(crate) lifted: Vec<(Rc<str>, Lifted)>,
    pub(crate) count: usize,
    pub(crate) inert_nodes: FxSet<NodeId>,
    pub(crate) where_: FxMap<NodeId, usize>,
    pub(crate) whole: std::collections::HashSet<String>,
    pub(crate) across: i64,
    pub(crate) avoided: i64,
    pub(crate) kept_depth: Rat,
    /// `plan_pairs` / `plan_glued`: the pairs of faces of the chain station plan the cells glued, and the cuts across them the clip did not make.
    pub plan_pairs: u64,
    pub(crate) plan_glued: i64,
    pub verdict: Option<Verdict>,
    /// `{node: {line}}` of the polygon being cut (`self.on`), `None` outside `_cut`.
    pub(crate) on: Option<FxMap<NodeId, FxSet<Pair>>>,
    pub writes: Vec<NormalWrite>,
}

/// An ordered pair of nodes: the key of `edge_cache` and `crossings`.
pub type Pair2 = (NodeId, NodeId);

fn cache_key(region_key: u32, index: usize) -> u64 {
    (u64::from(region_key) << 32) | index as u64
}

fn unsupported(text: &str) -> ClipError {
    ClipError::Unsupported(text.to_string())
}

impl<'a, 'c> Stage<'a, 'c> {
    /// `ClipStageV1(plane, budget, points, cells, shared)`. `regions` are the cut regions: the cells of the law by faces
    /// or the triangles of the lift (`cells is None` in the oracle, `faces_mode` false). The snap of the `src:` vertices
    /// runs here (the only cost of the constructor) unless a first stage already did it.
    pub fn new(
        plane: &'a Plane,
        ctx: &'a mut ExactCtx<'c>,
        warm: &'a mut Warm,
        version: PyVersion,
        points: &[(String, Point)],
        regions: Arc<RegionSet>,
        faces_mode: bool,
        shared: Option<Shared>,
    ) -> ClipResult<Stage<'a, 'c>> {
        let fresh = shared.is_none();
        let mut shared = match shared {
            Some(found) => found,
            None => Shared {
                point_ids: FxMap::default(),
                line_of: FxMap::default(),
                crossings: FxMap::default(),
                by_point: FxMap::default(),
                nodes: Vec::new(),
                chords: FxMap::default(),
                snap: CornerSnap { points: Vec::new(), moved: Vec::new(), counters: std::array::from_fn(|_| UBig::ZERO) },
                node_gap: UBig::ZERO,
                node_gap_square: Rat::zero(),
                tally: Tally::default(),
                region_key_ids: FxMap::default(),
            },
        };
        let mut edges: Vec<Vec<Pair>> = Vec::with_capacity(regions.len());
        for item in regions.iter() {
            let size = item.chart.len();
            let mut row = Vec::with_capacity(size);
            for index in 0..size {
                let first = point_id(&mut shared.point_ids, &item.chart[index]);
                let second = point_id(&mut shared.point_ids, &item.chart[(index + 1) % size]);
                row.push(pair(first, second));
            }
            edges.push(row);
        }
        let mut owners: FxMap<Pair, Vec<usize>> = FxMap::default();
        for (ti, row) in edges.iter().enumerate() {
            for edge in row {
                owners.entry(*edge).or_default().push(ti);
            }
        }
        let groups: Vec<&Option<CellKey>> = regions.iter().map(|item| &item.group).collect();
        let inert: Vec<Vec<bool>> = edges
            .iter()
            .enumerate()
            .map(|(ti, row)| {
                row.iter()
                    .map(|edge| {
                        let both = &owners[edge];
                        both.len() == 2 && groups[ti].is_some() && groups[both[0]] == groups[both[1]]
                    })
                    .collect()
            })
            .collect();
        let interior: Vec<Vec<bool>> = edges
            .iter()
            .enumerate()
            .map(|(ti, row)| row.iter().enumerate().map(|(index, edge)| owners[edge].len() > 1 && !inert[ti][index]).collect())
            .collect();
        let corners: FxSet<ChartPoint> = regions.iter().flat_map(|item| item.chart.iter().cloned()).collect();
        let line_ids: Vec<Vec<u32>> = edges
            .iter()
            .map(|row| {
                row.iter()
                    .map(|edge| {
                        let next = shared.line_of.len() as u32;
                        *shared.line_of.entry(*edge).or_insert(next)
                    })
                    .collect()
            })
            .collect();
        if fresh {
            let _p = scope(Phase::Snap);
            shared.snap = snap::snap_source_vertices(ctx, plane, points)?;
        }
        let region_keys: Vec<u32> = regions
            .iter()
            .map(|item| {
                let next = shared.region_key_ids.len() as u32;
                *shared.region_key_ids.entry(item.key.clone()).or_insert(next)
            })
            .collect();
        let Shared { point_ids, line_of, crossings, by_point, nodes, chords, snap, node_gap, node_gap_square, tally, region_key_ids } = shared;
        let mut stage = Stage {
            plane,
            ctx,
            warm,
            version,
            flows: None,
            faces_mode,
            regions,
            region_keys,
            interior,
            inert,
            line_ids,
            corners,
            straights: Vec::new(),
            point_ids,
            line_of,
            crossings,
            by_point,
            nodes,
            chords,
            snap,
            node_gap,
            node_gap_square,
            tally,
            region_key_ids,
            node_of_key: std::collections::HashMap::new(),
            edge_cache: FxMap::default(),
            needed: FxSet::default(),
            new_points: Vec::new(),
            lifted: Vec::new(),
            count: 0,
            inert_nodes: FxSet::default(),
            where_: FxMap::default(),
            whole: std::collections::HashSet::new(),
            across: 0,
            avoided: 0,
            kept_depth: Rat::zero(),
            plan_pairs: 0,
            plan_glued: 0,
            verdict: None,
            on: None,
            writes: Vec::new(),
        };
        let snapped: Vec<(String, Point)> = stage.snap.points.clone();
        let moved: std::collections::HashSet<String> = stage.snap.moved.iter().map(|(key, _)| key.clone()).collect();
        for (index, (key, point)) in snapped.into_iter().enumerate() {
            let known = stage.nodes.len();
            let node = stage.intern(point);
            if stage.nodes.len() > known && !moved.contains(&key) {
                stage.nodes[node as usize].origin = Some(index as u32);
            }
            stage.set_key(node, Rc::from(key.as_str()));
            stage.node_of_key.insert(key, node);
        }
        let mut straights = Vec::with_capacity(stage.regions.len());
        for ti in 0..stage.regions.len() {
            let mut row = Vec::with_capacity(stage.regions[ti].straight.len());
            for position in 0..stage.regions[ti].straight.len() {
                let (edge, corner) = stage.regions[ti].straight[position].clone();
                let node = stage.intern((SqrtSum::rational(&corner.0), SqrtSum::rational(&corner.1)));
                row.push((edge, node));
            }
            straights.push(row);
        }
        stage.straights = straights;
        Ok(stage)
    }

    /// What the successor stage takes over (`shared=stage`): consumes the stage, which has nothing else to say.
    pub fn into_shared(self) -> Shared {
        Shared {
            point_ids: self.point_ids,
            line_of: self.line_of,
            crossings: self.crossings,
            by_point: self.by_point,
            nodes: self.nodes,
            chords: self.chords,
            snap: self.snap,
            node_gap: self.node_gap,
            node_gap_square: self.node_gap_square,
            tally: self.tally,
            region_key_ids: self.region_key_ids,
        }
    }

    // ---- points, signs, crossings ------------------------------------------------------------------------------------

    /// `_node(point)`: the node of the point's identity, created on first sight with this very point as representative.
    pub(crate) fn intern(&mut self, point: Point) -> NodeId {
        let _p = scope(Phase::Intern);
        let identity = point_hash(&point);
        if let Some(bucket) = self.by_point.get(&identity) {
            if let Some(found) = bucket.iter().find(|id| same_point(&self.nodes[**id as usize].point, &point)) {
                return *found;
            }
        }
        let id = self.nodes.len() as NodeId;
        let rational = rational_pair(&point);
        self.nodes.push(Node {
            point: Arc::new(point),
            key: None,
            watched: false,
            source: false,
            window: None,
            home: None,
            origin: None,
            hash: identity,
            rational,
            cache: FxMap::default(),
            values: FxMap::default(),
        });
        self.by_point.entry(identity).or_default().push(id);
        id
    }

    /// [`Stage::intern`] for a point whose identity hash is known (a warm-cache hit hands over the point it computed before).
    pub(crate) fn intern_known(&mut self, point: Arc<Point>, hash: u64) -> NodeId {
        let _p = scope(Phase::Intern);
        if let Some(bucket) = self.by_point.get(&hash) {
            if let Some(found) = bucket.iter().find(|id| same_point(&self.nodes[**id as usize].point, &point)) {
                return *found;
            }
        }
        let id = self.nodes.len() as NodeId;
        let rational = rational_pair(&point);
        self.nodes.push(Node {
            point,
            key: None,
            watched: false,
            source: false,
            window: None,
            home: None,
            origin: None,
            hash,
            rational,
            cache: FxMap::default(),
            values: FxMap::default(),
        });
        self.by_point.entry(hash).or_default().push(id);
        id
    }

    pub(crate) fn set_key(&mut self, node: NodeId, key: Rc<str>) {
        let item = &mut self.nodes[node as usize];
        item.watched = key.starts_with("node:");
        item.source = key.starts_with("src:");
        item.key = Some(key);
    }

    pub(crate) fn point_of(&self, node: NodeId) -> Arc<Point> {
        self.nodes[node as usize].point.clone()
    }

    pub(crate) fn points_of(&self, nodes: &[NodeId]) -> Vec<Arc<Point>> {
        nodes.iter().map(|node| self.point_of(*node)).collect()
    }

    pub(crate) fn node_for(&self, key: &str) -> ClipResult<NodeId> {
        self.node_of_key.get(key).copied().ok_or_else(|| ClipError::MissingKey(key.to_string()))
    }

    fn slot_key(&self, ti: usize, index: usize) -> u64 {
        cache_key(self.region_keys[ti], index)
    }

    /// `_value(node, ti, index)`: the exact orientation value of a point at an edge of a region (built on demand, cached).
    pub(crate) fn value(&mut self, node: NodeId, ti: usize, index: usize) -> Arc<SqrtSum> {
        let key = self.slot_key(ti, index);
        if let Some(found) = self.nodes[node as usize].values.get(&key) {
            return found.clone();
        }
        let (point, hash) = (self.nodes[node as usize].point.clone(), self.nodes[node as usize].hash);
        let regions = self.regions.clone();
        let value = self.exact_value(&point, hash, &regions.lines[ti][index], regions.line_hashes[ti][index]);
        self.nodes[node as usize].values.insert(key, value.clone());
        value
    }

    /// The exact orientation value of a point at the line of an edge: from the session's warm cache (a function of the two, with no cost)
    /// or computed and remembered.
    pub(crate) fn exact_value(&mut self, point: &Arc<Point>, hash: u64, line: &EdgeLine, line_hash: u64) -> Arc<SqrtSum> {
        let _p = scope(Phase::LineValue);
        let key = value_key(hash, line_hash);
        if self.warm.enabled {
            if let Some(found) = self.warm.find_value(key, point, line) {
                return found;
            }
        }
        let value = Arc::new(line.value(&point.0, &point.1));
        if self.warm.enabled {
            self.warm.store_value(key, ValueEntry { point: point.clone(), line: line.clone(), value: value.clone() });
        }
        value
    }

    /// `_slot`: the sign of a node at an edge, inward `>= 0`; counted once per `(node, region, edge)` (`PREDICATES`).
    pub(crate) fn slot(&mut self, node: NodeId, ti: usize, index: usize) -> ClipResult<Slot> {
        let key = self.slot_key(ti, index);
        if let Some(found) = self.nodes[node as usize].cache.get(&key) {
            return Ok(*found);
        }
        let _p = scope(Phase::Slot);
        self.tally.predicates += 1;
        let watch = self.interior[ti][index] && self.nodes[node as usize].watched;
        let (cheap, far) = match &self.regions.constants[ti][index] {
            None => (None, false),
            Some(constants) => {
                let _c = scope(Phase::CheapSign);
                let item = &self.nodes[node as usize];
                cheap_sign_with(&item.point, item.rational.as_ref(), constants, watch)
            }
        };
        let mut value: Option<Arc<SqrtSum>> = None;
        let mut sign = match cheap {
            Some(decided) => decided,
            None => {
                let found = self.value(node, ti, index);
                let decided = {
                    let _e = scope(Phase::ExactSign);
                    exact::sign(self.ctx, &found, SIGN_FILTER_BITS)?
                };
                value = Some(found);
                decided
            }
        };
        let mut zeroed = false;
        if sign != 0 && watch && !far {
            let found = match value {
                Some(found) => found,
                None => self.value(node, ti, index),
            };
            sign = self.zeroed_by_gap(&found, ti, index, sign)?;
            zeroed = sign == 0;
        }
        let slot = Slot { sign: sign * self.regions.directions[ti], zeroed };
        self.nodes[node as usize].cache.insert(key, slot);
        Ok(slot)
    }

    /// `_zeroed_by_gap`: the sign of a `node:` vertex at an INTERIOR edge is zero when it is within the tolerance of its line.
    fn zeroed_by_gap(&mut self, value: &SqrtSum, ti: usize, index: usize, sign: i8) -> ClipResult<i8> {
        let (within, gap) = {
            let _w = scope(Phase::WithinEdgeGap);
            snap::within_edge_gap(self.ctx, value, &self.regions.edge_squares[ti][index])?
        };
        if !within {
            return Ok(sign);
        }
        self.tally.node_signs_zeroed += 1;
        let mut stretch: Option<Rat> = None;
        for member in &self.regions[ti].members {
            let candidate = self.plane.stretch_square(*member)?;
            stretch = Some(match stretch {
                Some(current) if !(candidate > current) => current,
                _ => candidate,
            });
        }
        let stretch = stretch.ok_or_else(|| unsupported("a region without members"))?;
        let reach = nanometres(&gap.mul(&stretch))?;
        if reach > self.node_gap {
            self.node_gap = reach;
        }
        if gap > self.node_gap_square {
            self.node_gap_square = gap;
        }
        Ok(0)
    }

    /// `_sign`.
    pub(crate) fn sign(&mut self, node: NodeId, ti: usize, index: usize) -> ClipResult<i8> {
        let key = self.slot_key(ti, index);
        if let Some(found) = self.nodes[node as usize].cache.get(&key) {
            return Ok(found.sign);
        }
        Ok(self.slot(node, ti, index)?.sign)
    }

    /// `_zeroed_by_law`: the sign of a `node:` vertex at an interior edge is zeroed by the tolerance of law 2 (a QUESTION: it
    /// writes nothing into the sign cache and counts nothing).
    fn zeroed_by_law(&mut self, node: NodeId, ti: usize, index: usize) -> ClipResult<bool> {
        if !self.nodes[node as usize].watched || !self.interior[ti][index] {
            return Ok(false);
        }
        let key = self.slot_key(ti, index);
        if let Some(found) = self.nodes[node as usize].cache.get(&key) {
            return Ok(found.zeroed);
        }
        if let Some(constants) = &self.regions.constants[ti][index] {
            let item = &self.nodes[node as usize];
            let (sign, far) = cheap_sign_with(&item.point, item.rational.as_ref(), constants, true);
            if far || sign == Some(0) {
                return Ok(false);
            }
        }
        let value = self.value(node, ti, index);
        if value.is_zero() {
            return Ok(false);
        }
        Ok(snap::within_edge_gap(self.ctx, &value, &self.regions.edge_squares[ti][index])?.0)
    }

    /// `_corner_of_gap`: the corner of the region a crossing is moved to when an end of its segment is zeroed by the tolerance
    /// at the neighbouring edge.
    fn corner_of_gap(&mut self, node: NodeId, first: NodeId, second: NodeId, ti: usize, index: usize) -> ClipResult<Option<NodeId>> {
        if self.inert[ti][index] {
            return Ok(None);
        }
        let _p = scope(Phase::CornerOfGap);
        let size = self.regions[ti].chart.len();
        for step in [-1i64, 1] {
            let neighbour = ((index as i64 + step).rem_euclid(size as i64)) as usize;
            if !self.interior[ti][neighbour] {
                continue;
            }
            if !(self.zeroed_by_law(first, ti, neighbour)? || self.zeroed_by_law(second, ti, neighbour)?) {
                continue;
            }
            let value = self.value(node, ti, neighbour);
            if value.is_zero() || !snap::within_edge_gap(self.ctx, &value, &self.regions.edge_squares[ti][neighbour])?.0 {
                continue;
            }
            self.tally.node_signs_zeroed += 1;
            let chart = &self.regions[ti].chart;
            let corner = if step == -1 { chart[index].clone() } else { chart[(index + 1) % size].clone() };
            return Ok(Some(self.intern((SqrtSum::rational(&corner.0), SqrtSum::rational(&corner.1)))));
        }
        Ok(None)
    }

    /// `_crossing`: the point of a segment on the line of an edge, `first + (second - first) * v0 / (v0 - v1)`, exact.
    pub(crate) fn crossing(&mut self, first: NodeId, second: NodeId, ti: usize, index: usize) -> ClipResult<NodeId> {
        let _p = scope(Phase::Crossing);
        self.slot(first, ti, index)?;
        self.slot(second, ti, index)?;
        self.tally.divisions += 1;
        let line = self.line_ids[ti][index];
        let remembered = (first, second, line);
        let node = match self.crossings.get(&remembered) {
            Some(found) => *found,
            None => {
                let made = self.crossing_point(first, second, ti, index)?;
                self.crossings.insert(remembered, made);
                self.crossings.insert((second, first, line), made);
                made
            }
        };
        if let Some(corner) = self.corner_of_gap(node, first, second, ti, index)? {
            return Ok(corner);
        }
        if let Some(on) = &mut self.on {
            let common: Vec<Pair> = match (on.get(&first), on.get(&second)) {
                (Some(left), Some(right)) => left.iter().filter(|line| right.contains(*line)).copied().collect(),
                _ => Vec::new(),
            };
            if !common.is_empty() {
                on.entry(node).or_default().extend(common);
            }
        }
        Ok(node)
    }

    /// The point of the crossing of the segment `first`-`second` with the line of edge `index` of region `ti`, as a node: from the
    /// session's warm cache (the questions of the original computation asked again of the memory) or computed and remembered.
    fn crossing_point(&mut self, first: NodeId, second: NodeId, ti: usize, index: usize) -> ClipResult<NodeId> {
        let (start, end) = (self.point_of(first), self.point_of(second));
        let line = &self.regions.lines[ti][index];
        let key = crossing_key(self.nodes[first as usize].hash, self.nodes[second as usize].hash, self.regions.line_hashes[ti][index]);
        if self.warm.enabled {
            if let Some(entry) = self.warm.find(key, &start, &end, line) {
                let (point, hash) = (entry.point, entry.hash);
                for request in &entry.requests {
                    self.ctx.memory.replay_request(request, self.ctx.budget).map_err(|error| ClipError::Exact(error.into()))?;
                }
                self.warm.hits += 1;
                return Ok(self.intern_known(point, hash));
            }
            self.ctx.memory.start_requests();
        }
        let low = self.value(first, ti, index);
        let high = self.value(second, ti, index);
        let denominator = low.sub(&high);
        // `low / (low - high)` stays an integer form (no canonical terms) because both coordinates multiply it on at once:
        // every result term is normalised there, and the canonical value is the oracle's `product_added(x0, x1 - x0, share)`
        let share = {
            let _d = scope(Phase::DividedBy);
            exact::divided_by_form(self.ctx, &low, &denominator)
        };
        let requests = if self.warm.enabled { self.ctx.memory.take_requests() } else { Vec::new() };
        let share = share?;
        let (x, y) = {
            let _a = scope(Phase::ProductAdded);
            let (dx, dy) = (end.0.sub(&start.0), end.1.sub(&start.1));
            match &share {
                Quotient::Form(form) => (product_added_form(&start.0, &dx, form, self.ctx.products), product_added_form(&start.1, &dy, form, self.ctx.products)),
                Quotient::Sum(sum) => (product_added(&start.0, &dx, sum, self.ctx.products), product_added(&start.1, &dy, sum, self.ctx.products)),
            }
        };
        let made = self.intern((x, y));
        if self.warm.enabled && matches!(share, Quotient::Form(_)) {
            let node = &self.nodes[made as usize];
            let entry = CrossingEntry { first: start, second: end, line: self.regions.lines[ti][index].clone(), point: node.point.clone(), hash: node.hash, requests };
            self.warm.store(key, entry);
        }
        Ok(made)
    }

    /// `_window`.
    fn window(&mut self, node: NodeId) -> ClipResult<[f64; 4]> {
        if let Some(found) = self.nodes[node as usize].window {
            return Ok(found);
        }
        let point = self.point_of(node);
        let found = numeric::window(&point.0, &point.1)?;
        self.nodes[node as usize].window = Some(found);
        Ok(found)
    }

    /// `_candidates`: the regions whose boxes meet the box of the nodes (a filter, never an answer).
    pub(crate) fn candidates(&mut self, nodes: &[NodeId]) -> ClipResult<Vec<usize>> {
        let _p = scope(Phase::Candidates);
        let mut windows = Vec::with_capacity(nodes.len());
        for node in nodes {
            windows.push(self.window(*node)?);
        }
        let first = windows.first().ok_or_else(|| unsupported("a polygon without vertices"))?;
        let (mut x_low, mut x_high, mut y_low, mut y_high) = (first[0], first[1], first[2], first[3]);
        for window in &windows[1..] {
            if window[0] < x_low {
                x_low = window[0];
            }
            if window[1] > x_high {
                x_high = window[1];
            }
            if window[2] < y_low {
                y_low = window[2];
            }
            if window[3] > y_high {
                y_high = window[3];
            }
        }
        Ok(self
            .regions
            .iter()
            .enumerate()
            .filter(|(_, item)| !(x_high < item.bbox[0] || x_low > item.bbox[1] || y_high < item.bbox[2] || y_low > item.bbox[3]))
            .map(|(ti, _)| ti)
            .collect())
    }

    // ---- edges --------------------------------------------------------------------------------------------------------

    /// `_segment_in`: the ends of the part of a segment in the CLOSED region `ti`, or `None`.
    fn segment_in(&mut self, first: NodeId, second: NodeId, ti: usize) -> ClipResult<Option<(NodeId, NodeId)>> {
        let _p = scope(Phase::SegmentIn);
        let (mut low, mut high) = (first, second);
        for index in 0..self.regions[ti].chart.len() {
            let low_sign = self.sign(low, ti, index)?;
            let high_sign = self.sign(high, ti, index)?;
            if low_sign >= 0 && high_sign >= 0 {
                continue;
            }
            if low_sign < 0 && high_sign < 0 {
                return Ok(None);
            }
            if low_sign == 0 {
                high = low;
            } else if high_sign == 0 {
                low = high;
            } else if low_sign < 0 {
                low = self.crossing(low, high, ti, index)?;
            } else {
                high = self.crossing(low, high, ti, index)?;
            }
        }
        Ok(Some((low, high)))
    }

    /// `_on_interior_edge`: `any(interior and sign == 0)`, stopping at the first.
    fn on_interior_edge(&mut self, node: NodeId, ti: usize) -> ClipResult<bool> {
        for index in 0..self.regions[ti].chart.len() {
            if self.interior[ti][index] && self.sign(node, ti, index)? == 0 {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// `_ordered(first, second, nodes)`: the nodes from `first` towards `second` by the exact sort of the oracle.
    pub(crate) fn ordered(&mut self, first: NodeId, second: NodeId, nodes: &[NodeId]) -> ClipResult<Vec<NodeId>> {
        let _p = scope(Phase::Ordered);
        let (start, end) = (self.point_of(first), self.point_of(second));
        let owned = self.points_of(nodes);
        let refs: Vec<&Point> = owned.iter().map(|point| &**point).collect();
        let permutation = order::ordered(self.ctx, self.version, &start, &end, &refs)?;
        Ok(permutation.into_iter().map(|index| nodes[index]).collect())
    }

    /// `edge_points(first, second)`: the new vertices of an edge in order; one computation per pair, the reverse for free.
    pub fn edge_points(&mut self, first: NodeId, second: NodeId) -> ClipResult<Rc<Vec<NodeId>>> {
        if let Some(found) = self.edge_cache.get(&(first, second)) {
            return Ok(found.clone());
        }
        let _p = scope(Phase::EdgePoints);
        let mut found: Vec<NodeId> = Vec::new();
        for ti in self.candidates(&[first, second])? {
            let Some((low, high)) = self.segment_in(first, second, ti)? else {
                continue;
            };
            for end in [low, high] {
                if end == first || end == second || found.contains(&end) {
                    continue;
                }
                if self.on_interior_edge(end, ti)? {
                    found.push(end);
                    if self.nodes[end as usize].home.is_none() {
                        self.nodes[end as usize].home = Some(ti);
                    }
                }
            }
        }
        let points = if found.is_empty() { Vec::new() } else { self.ordered(first, second, &found)? };
        let reversed: Vec<NodeId> = points.iter().rev().copied().collect();
        self.tally.edges_refined += u64::from(!points.is_empty());
        let points = Rc::new(points);
        self.edge_cache.insert((first, second), points.clone());
        self.edge_cache.insert((second, first), Rc::new(reversed));
        Ok(points)
    }

    /// `_refined`: the contour with the vertices of the NEEDED edges (an edge of a cut polygon).
    pub(crate) fn refined(&mut self, nodes: &[NodeId]) -> ClipResult<Vec<NodeId>> {
        let _p = scope(Phase::Refined);
        let mut out = Vec::with_capacity(nodes.len());
        for (index, node) in nodes.iter().enumerate() {
            let following = nodes[(index + 1) % nodes.len()];
            out.push(*node);
            if self.needed.contains(&pair(*node, following)) {
                out.extend(self.edge_points(*node, following)?.iter().copied());
            }
        }
        Ok(out)
    }

    // ---- the cut of one polygon by one region -------------------------------------------------------------------------------

    /// `_clip`: Sutherland-Hodgman of a counter-clockwise contour by the half-planes of the edges of region `ti` (sign `>= 0` is inside).
    pub(crate) fn clip(&mut self, nodes: Vec<NodeId>, ti: usize) -> ClipResult<Vec<NodeId>> {
        let _p = scope(Phase::Clip);
        let mut nodes = nodes;
        for index in 0..self.regions[ti].chart.len() {
            let mut signs = Vec::with_capacity(nodes.len());
            for node in &nodes {
                signs.push(self.sign(*node, ti, index)?);
            }
            if signs.iter().all(|sign| *sign >= 0) {
                continue;
            }
            if signs.iter().all(|sign| *sign <= 0) {
                return Ok(Vec::new());
            }
            let mut result = Vec::with_capacity(nodes.len() + 1);
            let size = nodes.len();
            for position in 0..size {
                let (current, following) = (nodes[position], nodes[(position + 1) % size]);
                let (here, there) = (signs[position], signs[(position + 1) % size]);
                if here >= 0 {
                    result.push(current);
                }
                if here == 0 || there == 0 || (here > 0) == (there > 0) {
                    continue;
                }
                let crossing = self.crossing(current, following, ti, index)?;
                if self.inert[ti][index] {
                    self.inert_nodes.insert(crossing);
                }
                result.push(crossing);
            }
            nodes = result;
        }
        self.with_straight_vertices(nodes, ti)
    }

    /// `_strictly_between`: `middle` is strictly between `first` and `last` on their line (exact, along the axis where the ends differ).
    fn strictly_between(&mut self, first: NodeId, middle: NodeId, last: NodeId) -> ClipResult<bool> {
        let (a, b, c) = (self.point_of(first), self.point_of(middle), self.point_of(last));
        let axis_of = |point: &Point, axis: usize| if axis == 0 { point.0.clone() } else { point.1.clone() };
        let axis = if !c.0.sub(&a.0).is_zero() { 0 } else { 1 };
        let before = exact::sign(self.ctx, &axis_of(&b, axis).sub(&axis_of(&a, axis)), SIGN_FILTER_BITS)?;
        let after = exact::sign(self.ctx, &axis_of(&c, axis).sub(&axis_of(&b, axis)), SIGN_FILTER_BITS)?;
        Ok(i32::from(before) * i32::from(after) > 0)
    }

    /// `_with_straight_vertices`: a straight vertex of a cell inside an edge of the piece that lies on its line joins the piece.
    fn with_straight_vertices(&mut self, nodes: Vec<NodeId>, ti: usize) -> ClipResult<Vec<NodeId>> {
        if self.straights[ti].is_empty() || nodes.len() < 3 {
            return Ok(nodes);
        }
        let found = self.straights[ti].clone();
        let mut out = Vec::with_capacity(nodes.len());
        for (index, node) in nodes.iter().enumerate() {
            let following = nodes[(index + 1) % nodes.len()];
            out.push(*node);
            let mut inside = Vec::new();
            for (edge, vertex) in &found {
                if *vertex != *node
                    && *vertex != following
                    && self.sign(*node, ti, *edge)? == 0
                    && self.sign(following, ti, *edge)? == 0
                    && self.strictly_between(*node, *vertex, following)?
                {
                    inside.push(*vertex);
                }
            }
            if !inside.is_empty() {
                out.extend(self.ordered(*node, following, &inside)?);
            }
        }
        Ok(out)
    }
}

/// `point_ids.setdefault(point, len(point_ids))`.
fn point_id(ids: &mut FxMap<ChartPoint, u32>, point: &ChartPoint) -> u32 {
    if let Some(found) = ids.get(point) {
        return *found;
    }
    let next = ids.len() as u32;
    ids.insert(point.clone(), next);
    next
}
