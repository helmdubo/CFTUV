//! `ClipStageV1`, third part: phases 2 and 3 of the stage and the result. The noise suppression, the subdivision of the
//! needed edges, the emission of the faces with the `clip:<k>` keys and the lift of the new vertices, the glue of the pieces
//! of non-convex faces, the contours, the counters and the note (`run`).
//!
//! The ORDER of every dictionary the oracle builds is the order of the output (`seen`, `lifted`, `points`), and the first
//! writer wins where the oracle says `setdefault` (`where`), so those are ordered containers and first-insert-only maps.

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use cftuv_canon::ordered::OrderedMap;
use cftuv_core::exact;
use cftuv_core::num::UBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::cells::CellKey;
use crate::cut::{chord_budget_square, pts, refusal, Cut, Piece, Suppressed, TESSELLATION_DID_NOT_CLOSE};
use crate::error::{ClipError, ClipResult};
use crate::fxhash::FxSet;
use crate::lift::{self, Lifted};
use crate::numeric::{milli_cells, nanometres};
use crate::plane::ChartPoint;
use crate::point::Point;
use crate::profile::{scope, Phase};
use crate::snap::COUNTER_NAMES as SNAP_COUNTER_NAMES;
use crate::stage::{pair, NodeId, NormalWrite, Stage, Verdict};
use crate::warm::{lift_key, LiftEntry};
use crate::tessellate;

pub const CLIP_PIECE_LEFT_ITS_TRIANGLE: &str = "CLIP_PIECE_LEFT_ITS_TRIANGLE";
pub const BATCH_DID_NOT_VALIDATE: &str = "BATCH_DID_NOT_VALIDATE";

/// The topology law of the request, as far as the cut reads it (`_split_for_law` compares by identity): the whole polygon,
/// the strictly convex quadrilateral of a strip or the ears; EVERY other member of the enum asks for the ears.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Law {
    PlanarPolygons,
    QuadStrips,
    Ears,
}

/// The names of the counters, in the order of the oracle (`run`).
const STAGE_COUNTERS: [&str; 16] = [
    "MATERIALIZE_CLIP_VERTICES_INSERTED",
    "MATERIALIZE_CLIP_VERTICES_AT_SOURCE_VERTEX",
    "MATERIALIZE_CLIP_EDGES_REFINED",
    "MATERIALIZE_CLIP_FACES_IN_ONE_TRIANGLE",
    "MATERIALIZE_CLIP_FACES_CUT",
    "MATERIALIZE_CLIP_FACES_CUT_BY_EARS",
    "MATERIALIZE_CLIP_PIECES_EMITTED",
    "MATERIALIZE_CLIP_PIECES_MERGED",
    "MATERIALIZE_CLIP_PIECES_KEPT_SEPARATE",
    "MATERIALIZE_CLIP_FACES_OVERHANG_TRIANGULATED",
    "MATERIALIZE_CLIP_FACES_BOUNDARY_MISMATCH_TRIANGULATED",
    "MATERIALIZE_CLIP_FACES_SEAM_CROSSINGS_SUPPRESSED",
    "MATERIALIZE_CLIP_FACES_SOURCE_VERTEX_OFF_CORNER_SUPPRESSED",
    "MATERIALIZE_CLIP_FLOW_FREE_CUT_EDGES",
    "MATERIALIZE_CLIP_PREDICATES",
    "MATERIALIZE_CLIP_DIVISIONS",
];
const NODE_SIGNS_ZEROED: &str = "MATERIALIZE_CLIP_NODE_SIGNS_ZEROED_BY_EDGE_GAP";
const NODE_EDGE_GAP_MAX: &str = "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_NANOMETRES";
const NODE_EDGE_GAP_MAX_CELLS: &str = "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_MILLICELLS";
const PLAN_COUNTERS: [&str; 2] = ["MATERIALIZE_CLIP_PLAN_INERT_FACE_PAIRS", "MATERIALIZE_CLIP_PLAN_INERT_CUTS_AVOIDED"];
const DIAGONAL_COUNTERS: [&str; 7] = [
    "MATERIALIZE_CLIP_DIAGONAL_FACES_KEPT_WHOLE",
    "MATERIALIZE_CLIP_DIAGONAL_PIECES_ACROSS",
    "MATERIALIZE_CLIP_DIAGONAL_CUTS_AVOIDED",
    "MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_NOT_PLANAR",
    "MATERIALIZE_CLIP_DIAGONAL_KEPT_FACE_UNMERGEABLE",
    "MATERIALIZE_CLIP_DIAGONAL_MAX_CHORD_KEPT_NANOMETRES",
    "MATERIALIZE_CLIP_DIAGONAL_MAX_CHORD_OVER_BUDGET_NANOMETRES",
];

/// A key with the point of its node, in the order of the oracle's dictionary.
pub type KeyedPoint = (Rc<str>, Arc<Point>);

/// `ClippedV1` (the `memo` label is the caller's).
#[derive(Debug)]
pub struct Clipped {
    /// Per merged face: the key tuples of its faces.
    pub polygons: Vec<Vec<Vec<Rc<str>>>>,
    pub cycles: Vec<Vec<KeyedPoint>>,
    pub vertex_lists: Vec<Vec<KeyedPoint>>,
    pub extra_lists: Vec<Vec<KeyedPoint>>,
    pub points: Vec<KeyedPoint>,
    pub snapped: Vec<(String, Point)>,
    pub lifted: Vec<(Rc<str>, Lifted)>,
    pub counters: Vec<(&'static str, UBig)>,
    pub note: String,
    /// The nodes whose representative point IS an input object: `(that point, index in the input `points`)`. A point of
    /// the lists above that is one of these (by `Rc` identity) is the input's own tuple.
    pub origins: Vec<(Arc<Point>, u32)>,
}

fn oriented(keys: Vec<Rc<str>>, flip: bool) -> Vec<Rc<str>> {
    if !flip || keys.len() < 2 {
        return keys;
    }
    let mut out = Vec::with_capacity(keys.len());
    out.push(keys[0].clone());
    out.extend(keys[1..].iter().rev().cloned());
    out
}

impl<'a, 'c> Stage<'a, 'c> {
    // ---- keys and the lift of new vertices ---------------------------------------------------------------------------------

    /// `(x, y)` of a node whose coordinates are both rational, as chart numbers.
    fn rational_point(&self, node: NodeId) -> Option<ChartPoint> {
        let point = &self.nodes[node as usize].point;
        let x = point.0.as_rational()?.into_value();
        let y = point.1.as_rational()?.into_value();
        Some((x, y))
    }

    /// `_home(node, ti)`: the triangle that lifts a vertex born in region `ti` and the three orientation values there.
    fn home_of(&mut self, node: NodeId, ti: usize) -> ClipResult<(usize, [Arc<SqrtSum>; 3])> {
        let _p = scope(Phase::HomeOf);
        let members = self.regions[ti].members.clone();
        if members.len() == 1 {
            let mut values: Vec<Arc<SqrtSum>> = Vec::with_capacity(3);
            for index in 0..3 {
                self.slot(node, ti, index)?;
                values.push(self.value(node, ti, index));
            }
            let values: [Arc<SqrtSum>; 3] = values.try_into().map_err(|_| ClipError::Unsupported("a single-triangle region without three edges".into()))?;
            return Ok((members[0], values));
        }
        let (point, hash) = (self.point_of(node), self.nodes[node as usize].hash);
        let plane = self.plane;
        for member in members {
            let lines = plane.triangle_lines(member);
            let values: [Arc<SqrtSum>; 3] = [0, 1, 2].map(|index| self.exact_value(&point, hash, &lines[index].0, lines[index].1));
            let direction: i32 = if self.plane.triangles[member].twice_area.signum() > 0 { 1 } else { -1 };
            let mut inside = true;
            for value in &values {
                if i32::from(exact::sign(self.ctx, &**value, SIGN_FILTER_BITS)?) * direction < 0 {
                    inside = false;
                    break;
                }
            }
            if inside {
                return Ok((member, values));
            }
        }
        Err(refusal(
            CLIP_PIECE_LEFT_ITS_TRIANGLE,
            format!("a new clip vertex lies in no triangle of the source face {}", self.regions[ti].name),
        ))
    }

    /// `lift_known` of a vertex in its home triangle: from the session's warm cache (a function of the point, the triangle and the interpreter, with
    /// no cost of its own) or computed and remembered. A lift that fails is never kept: it fails again, the same way.
    fn lift(&mut self, node: NodeId, triangle: usize, values: &[Arc<SqrtSum>; 3]) -> ClipResult<Lifted> {
        let (point, hash) = (self.point_of(node), self.nodes[node as usize].hash);
        let plane = self.plane;
        let key = lift_key(hash, plane.hashes[triangle]);
        if self.warm.enabled {
            if let Some(found) = self.warm.find_lift(key, &point, &plane.triangles[triangle], self.version) {
                return Ok(found);
            }
        }
        let lifted = lift::lift_known_with(self.version, &plane.triangles[triangle], plane.lift_factors(triangle)?, [&*values[0], &*values[1], &*values[2]])?;
        if self.warm.enabled {
            self.warm.store_lift(key, LiftEntry { point, triangle: Arc::new(plane.triangles[triangle].clone()), version: self.version, lifted: lifted.clone() });
        }
        Ok(lifted)
    }

    /// `_key(node, ti)`: the key of a vertex; a new vertex gets `clip:<k>` and its lift in the triangle it was born in.
    pub(crate) fn key_of(&mut self, node: NodeId, ti: Option<usize>) -> ClipResult<Rc<str>> {
        if let Some(key) = &self.nodes[node as usize].key {
            return Ok(key.clone());
        }
        let _p = scope(Phase::KeyOf);
        let Some(home) = self.nodes[node as usize].home.or(ti) else {
            return Err(refusal(CLIP_PIECE_LEFT_ITS_TRIANGLE, "a new clip vertex has no source triangle to be lifted in".to_string()));
        };
        let key: Rc<str> = Rc::from(format!("clip:{}", self.count));
        self.count += 1;
        self.set_key(node, key.clone());
        let (triangle, values) = self.home_of(node, home)?;
        let lifted = {
            let _l = scope(Phase::LiftKnown);
            self.lift(node, triangle, &values)?
        };
        if let Some(normal) = lifted.normal {
            self.writes.push(NormalWrite { position: lifted.position, normal });
        }
        self.lifted.push((key.clone(), lifted));
        self.new_points.push((key.clone(), node));
        self.node_of_key.insert(key.to_string(), node);
        if self.rational_point(node).is_some_and(|point| self.corners.contains(&point)) {
            self.tally.vertices_at_source_vertex += 1;
        }
        Ok(key)
    }

    /// `_prove(nodes, ti)`: every vertex of a piece is in the CLOSED region (the signs of its edges are not negative).
    fn prove(&mut self, nodes: &[NodeId], ti: usize) -> ClipResult<()> {
        let _p = scope(Phase::Prove);
        for node in nodes {
            for index in 0..self.regions[ti].chart.len() {
                if self.sign(*node, ti, index)? < 0 {
                    return Err(refusal(
                        CLIP_PIECE_LEFT_ITS_TRIANGLE,
                        format!("a clip piece vertex lies outside the closed source triangle {} (edge {})", self.regions[ti].name, index),
                    ));
                }
            }
        }
        Ok(())
    }

    /// `_split_for_law`: the faces a piece becomes under the topology law.
    fn split_for_law(&mut self, nodes: &[NodeId], law: Law, fan: bool) -> ClipResult<Vec<Vec<NodeId>>> {
        let _p = scope(Phase::SplitForLaw);
        if law == Law::PlanarPolygons || nodes.len() == 3 {
            return Ok(vec![nodes.to_vec()]);
        }
        let owned = self.points_of(nodes);
        let points = pts(&owned);
        if law == Law::QuadStrips && !fan && tessellate::convex_quad_ring(self.ctx, &points)?.is_some() {
            return Ok(vec![nodes.to_vec()]);
        }
        match tessellate::triangulate_exact(self.ctx, &points)? {
            Some(ears) => Ok(ears.iter().map(|ear| ear.iter().map(|index| nodes[*index]).collect()).collect()),
            None => Err(refusal(TESSELLATION_DID_NOT_CLOSE, format!("a clip piece of {} vertices has no triangulation", nodes.len()))),
        }
    }

    // ---- the diagonal counters of the law by faces --------------------------------------------------------------------------

    /// `_record_whole(ti, piece, merged)`: a piece of a merged cell or group that was emitted whole.
    fn record_whole(&mut self, ti: usize, piece: &[NodeId], merged: u32) -> ClipResult<()> {
        if let Some(group) = &self.regions[ti].group {
            if merged != 0 {
                if let CellKey::Plan(_) = group {
                    // a group of the chain station plan is no face: its glued cuts are counted on their own and its estimate is the plan's zero
                    self.plan_glued += i64::from(merged) - 1;
                } else {
                    if let CellKey::Group(face, _) = group {
                        self.whole.insert(face.clone());
                    }
                    self.across += 1;
                    self.avoided += i64::from(merged) - 1;
                    let flat = self.regions[ti].flat_square.clone().ok_or_else(|| ClipError::Unsupported("a group without a flatness estimate".into()))?;
                    if flat > self.kept_depth {
                        self.kept_depth = flat;
                    }
                }
            }
            return Ok(());
        }
        if self.regions[ti].members.len() < 2 {
            return Ok(());
        }
        let (depth, crossed) = self.piece_chord(ti, piece)?;
        self.whole.insert(self.regions[ti].name.clone());
        self.across += i64::from(crossed > 0);
        self.avoided += crossed;
        if depth > self.kept_depth {
            self.kept_depth = depth;
        }
        Ok(())
    }

    fn diagonal_counters(&self) -> ClipResult<Vec<(&'static str, UBig)>> {
        let empty = Verdict::default();
        let verdict = self.verdict.as_ref().unwrap_or(&empty);
        let faces: HashSet<&str> = verdict
            .over
            .iter()
            .filter_map(|(key, _)| match key {
                CellKey::Face(face, _) | CellKey::Group(face, _) | CellKey::Plan(face) => Some(face.as_str()),
                CellKey::Triangle(_) => None,
            })
            .collect();
        let widest = verdict.over.iter().fold(Rat::zero(), |best, (_, depth)| if *depth > best { depth.clone() } else { best });
        let values = [
            UBig::from(self.whole.len()),
            UBig::from(self.across.max(0) as u64),
            UBig::from(self.avoided.max(0) as u64),
            UBig::from(faces.len()),
            UBig::from(verdict.unmergeable.len()),
            nanometres(&self.kept_depth)?,
            nanometres(&widest)?,
        ];
        Ok(DIAGONAL_COUNTERS.iter().copied().zip(values).collect())
    }

    // ---- glue --------------------------------------------------------------------------------------------------------------

    /// `_components(entries)`: the pieces of one group joined by opposite half-edges (a shared diagonal), as components in the
    /// order of their first piece.
    fn components(entries: &[Piece]) -> Vec<Vec<Piece>> {
        let mut owner: OrderedMap<(NodeId, NodeId), usize> = OrderedMap::new();
        for (index, entry) in entries.iter().enumerate() {
            for (position, node) in entry.nodes.iter().enumerate() {
                owner.set((*node, entry.nodes[(position + 1) % entry.nodes.len()]), index);
            }
        }
        let mut parent: Vec<usize> = (0..entries.len()).collect();
        fn find(parent: &mut [usize], mut item: usize) -> usize {
            while parent[item] != item {
                parent[item] = parent[parent[item]];
                item = parent[item];
            }
            item
        }
        for ((first, second), index) in owner.iter() {
            if let Some(other) = owner.get(&(*second, *first)) {
                let root = find(&mut parent, *other);
                let own = find(&mut parent, *index);
                parent[own] = root;
            }
        }
        let mut roots: Vec<usize> = Vec::new();
        let mut found: Vec<Vec<Piece>> = Vec::new();
        for (index, entry) in entries.iter().enumerate() {
            let root = find(&mut parent, index);
            match roots.iter().position(|known| *known == root) {
                Some(slot) => found[slot].push(entry.clone()),
                None => {
                    roots.push(root);
                    found.push(vec![entry.clone()]);
                }
            }
        }
        found
    }

    /// `_is_corner`: both coordinates rational and the point a corner of a region.
    fn is_corner(&self, node: NodeId) -> bool {
        self.rational_point(node).is_some_and(|point| self.corners.contains(&point))
    }

    /// `_collinear`: the three points are on one line (exact, no sign asked).
    fn collinear(&mut self, first: NodeId, middle: NodeId, last: NodeId) -> bool {
        let (a, b, c) = (self.point_of(first), self.point_of(middle), self.point_of(last));
        let products = &mut *self.ctx.products;
        let left = b.0.sub(&a.0).mul(&c.1.sub(&b.1), products);
        let right = b.1.sub(&a.1).mul(&c.0.sub(&b.0), products);
        left.sub(&right).is_zero()
    }

    /// `_without_inert`: the vertices born of a group's diagonal that lie between their neighbours on a line are dropped.
    fn without_inert(&mut self, loop_nodes: &[NodeId], refined: &FxSet<NodeId>) -> Vec<NodeId> {
        let mut out = loop_nodes.to_vec();
        let mut changed = true;
        while changed && out.len() > 3 {
            changed = false;
            for index in 0..out.len() {
                let node = out[index];
                let size = out.len();
                if self.inert_nodes.contains(&node)
                    && !refined.contains(&node)
                    && !self.is_corner(node)
                    && self.collinear(out[(index + size - 1) % size], node, out[(index + 1) % size])
                {
                    out.remove(index);
                    changed = true;
                    break;
                }
            }
        }
        out
    }

    /// `_glued(pieces, refined)`: the pieces of a non-convex face by its triangles, glued along the shared diagonals.
    fn glued(&mut self, pieces: Vec<Piece>, refined: &FxSet<NodeId>) -> ClipResult<Vec<Piece>> {
        let _p = scope(Phase::Glued);
        let mut out: Vec<Piece> = Vec::new();
        let mut by_group: Vec<(CellKey, Vec<Piece>)> = Vec::new();
        for entry in pieces {
            let Some(group) = self.regions[entry.ti].group.clone() else {
                out.push(entry);
                continue;
            };
            self.prove(&entry.nodes, entry.ti)?;
            for node in &entry.nodes {
                self.where_.entry(*node).or_insert(entry.ti);
            }
            match by_group.iter_mut().find(|(known, _)| *known == group) {
                Some((_, list)) => list.push(entry),
                None => by_group.push((group, vec![entry])),
            }
        }
        for (_, entries) in by_group {
            for component in Self::components(&entries) {
                let outline = if component.len() > 1 {
                    let nodes: Vec<Vec<NodeId>> = component.iter().map(|item| item.nodes.clone()).collect();
                    Self::merged(&nodes)
                } else {
                    None
                };
                match outline {
                    None => out.extend(component),
                    Some(outline) => {
                        let nodes = self.without_inert(&outline, refined);
                        out.push(Piece { ti: component[0].ti, nodes, glued: component.len() as u32 });
                    }
                }
            }
        }
        Ok(out)
    }

    // ---- emission ----------------------------------------------------------------------------------------------------------

    /// `_emit(cut, law, fan, flow)`: phase 3 for one polygon, the key tuples of its faces in its own winding.
    fn emit(&mut self, cut: &Cut, law: Law, fan: bool, flow: bool) -> ClipResult<Vec<Vec<Rc<str>>>> {
        let _p = scope(Phase::Emit);
        let nodes = self.refined(&cut.nodes)?;
        let mut pieces = cut.pieces.clone();
        if self.regions.has_groups {
            if let Some(found) = pieces.take() {
                let refined: FxSet<NodeId> = nodes.iter().copied().collect();
                pieces = Some(self.glued(found, &refined)?);
            }
        }
        if let Some(found) = &pieces {
            if Self::boundary_is(found, &nodes) {
                return self.emit_pieces(found, cut, law, fan, flow);
            }
        }
        // an overhang, a seam edge crossing interior edges of the source, or a subdivision that gave no contour: ears with the same
        // vertices on the needed edges, each cause under its own counter
        if cut.suppressed != Suppressed::No {
            if cut.suppressed == Suppressed::Seam {
                self.tally.faces_seam_suppressed += 1;
            } else {
                self.tally.faces_off_corner_suppressed += 1;
            }
        } else if pieces.is_none() {
            self.tally.faces_overhang += 1;
        } else {
            self.tally.faces_boundary_mismatch += 1;
        }
        let ears = self.ears_of(&nodes)?;
        if flow {
            self.tally.flow_free_cut_edges += ears.len() as u64 - 1;
        }
        let mut faces = Vec::with_capacity(ears.len());
        for ear in &ears {
            let mut keys = Vec::with_capacity(3);
            for index in ear {
                keys.push(self.key_of(nodes[*index], None)?);
            }
            faces.push(oriented(keys, cut.flip));
        }
        Ok(faces)
    }

    fn emit_pieces(&mut self, pieces: &[Piece], cut: &Cut, law: Law, fan: bool, flow: bool) -> ClipResult<Vec<Vec<Rc<str>>>> {
        if pieces.len() == 1 {
            self.tally.faces_in_one_triangle += 1;
        } else {
            self.tally.faces_cut += 1;
        }
        self.tally.faces_cut_by_ears += u64::from(cut.by_ears);
        self.tally.pieces_emitted += pieces.len() as u64;
        self.tally.pieces_merged += cut.merged;
        self.tally.pieces_kept_separate += cut.kept;
        let mut faces = Vec::new();
        let distinct: HashSet<usize> = pieces.iter().map(|piece| piece.ti).collect();
        let mut free = (pieces.len() - distinct.len()) as u64;
        for piece in pieces {
            let (ti, merged) = (piece.ti, piece.glued);
            if merged == 0 {
                self.prove(&piece.nodes, ti)?;
            }
            self.record_whole(ti, &piece.nodes, merged)?;
            // the numbers of `clip:` follow the walk of the piece, not the order of the ears of the law: the names do not depend on the law
            for node in &piece.nodes {
                let born = if merged != 0 { self.where_.get(node).copied().unwrap_or(ti) } else { ti };
                self.key_of(*node, Some(born))?;
            }
            let split = self.split_for_law(&piece.nodes, law, fan)?;
            free += split.len() as u64 - 1;
            for face in split {
                let mut keys = Vec::with_capacity(face.len());
                for node in &face {
                    keys.push(self.key_of(*node, Some(ti))?);
                }
                faces.push(oriented(keys, cut.flip));
            }
        }
        if flow {
            self.tally.flow_free_cut_edges += free;
        }
        Ok(faces)
    }

    // ---- the domain --------------------------------------------------------------------------------------------------------

    /// `_off_corner`: a `src:` vertex that does not stand in a corner of the snapped triangulation (a point with an irrational
    /// coordinate is not in a corner either).
    fn off_corner(&self, node: NodeId) -> bool {
        if !self.nodes[node as usize].source {
            return false;
        }
        match self.rational_point(node) {
            Some(point) => !self.corners.contains(&point),
            None => true,
        }
    }

    /// `_suppress_noise(cuts, seam)`: cut polygons in the noise of the snap stay ears (`seam` or `corner`).
    fn suppress_noise(&mut self, cuts: Vec<Vec<Cut>>, seam: &[(String, String)]) -> ClipResult<Vec<Vec<Cut>>> {
        let _p = scope(Phase::Suppress);
        let mut seam_nodes: FxSet<(NodeId, NodeId)> = FxSet::default();
        for (first, second) in seam {
            let (a, b) = (self.node_for(first)?, self.node_for(second)?);
            seam_nodes.insert(pair(a, b));
        }
        let mut result = Vec::with_capacity(cuts.len());
        for face_cuts in cuts {
            let mut kept = Vec::with_capacity(face_cuts.len());
            for mut cut in face_cuts {
                if cut.pieces.is_some() {
                    let size = cut.nodes.len();
                    let mut crossing = false;
                    for index in 0..size {
                        let (first, second) = (cut.nodes[index], cut.nodes[(index + 1) % size]);
                        if seam_nodes.contains(&pair(first, second)) && !self.edge_points(first, second)?.is_empty() {
                            crossing = true;
                            break;
                        }
                    }
                    if crossing {
                        cut.pieces = None;
                        cut.suppressed = Suppressed::Seam;
                    } else if cut.nodes.iter().any(|node| self.off_corner(*node)) {
                        cut.pieces = None;
                        cut.suppressed = Suppressed::Corner;
                    }
                }
                kept.push(cut);
            }
            result.push(kept);
        }
        Ok(result)
    }

    /// `_contour(cycle)`: the contour of a merged face with the vertices of its edges; every vertex must be emitted.
    fn contour(&mut self, cycle: &[String]) -> ClipResult<Vec<(Rc<str>, NodeId)>> {
        let _p = scope(Phase::Contour);
        let nodes = cycle.iter().map(|key| self.node_for(key)).collect::<ClipResult<Vec<NodeId>>>()?;
        let mut out = Vec::with_capacity(nodes.len());
        for node in self.refined(&nodes)? {
            let Some(key) = &self.nodes[node as usize].key else {
                return Err(refusal(
                    BATCH_DID_NOT_VALIDATE,
                    "CLIP_CONTOUR_POINT_NOT_EMITTED: a contour vertex is on no emitted face".to_string(),
                ));
            };
            out.push((key.clone(), node));
        }
        Ok(out)
    }

    /// `run(cycles, polygons, law, seam, fans, cuts)`: the faces of every merged face of the domain, the contours with the
    /// vertices of their edges, and the numbers of the stage. `cuts` is phase 1 when the caller already has it.
    pub fn run(
        &mut self,
        cycles: &[Vec<String>],
        polygons: &[Vec<Vec<String>>],
        law: Law,
        seam: &[(String, String)],
        fans: Option<&[bool]>,
        cuts: Option<Vec<Vec<Cut>>>,
    ) -> ClipResult<Clipped> {
        let cuts = match cuts {
            Some(found) => found,
            None => self.cuts_of(polygons)?,
        };
        let cuts = self.suppress_noise(cuts, seam)?;
        self.needed = cuts
            .iter()
            .flatten()
            .filter(|cut| cut.pieces.is_some())
            .flat_map(|cut| (0..cut.nodes.len()).map(move |index| pair(cut.nodes[index], cut.nodes[(index + 1) % cut.nodes.len()])))
            .collect();
        let mut faces: Vec<Vec<Vec<Rc<str>>>> = Vec::new();
        let flows = self.flows.clone();
        for (position, face_cuts) in cuts.iter().enumerate() {
            let fan = match fans {
                Some(found) => match found.get(position) {
                    Some(flag) => *flag,
                    None => break,
                },
                None => false,
            };
            let flow = match &flows {
                Some(found) => match found.get(position) {
                    Some(flag) => *flag,
                    None => break,
                },
                None => false,
            };
            let mut emitted = Vec::new();
            for cut in face_cuts {
                emitted.extend(self.emit(cut, law, fan, flow)?);
            }
            faces.push(emitted);
        }
        let _finish = scope(Phase::Finish);
        let (mut refined, mut lists, mut extras) = (Vec::new(), Vec::new(), Vec::new());
        for (cycle, emitted) in cycles.iter().zip(faces.iter()) {
            let contour = self.contour(cycle)?;
            let mut seen: OrderedMap<Rc<str>, NodeId> = OrderedMap::new();
            for (key, _) in &contour {
                seen.set(key.clone(), self.node_for(key)?);
            }
            for keys in emitted {
                for key in keys {
                    if !seen.contains_key(key) {
                        let node = self.node_for(key)?;
                        seen.set(key.clone(), node);
                    }
                }
            }
            let original: HashSet<&str> = cycle.iter().map(|key| key.as_str()).collect();
            let listing = |entries: Vec<(&Rc<str>, &NodeId)>, nodes: &[crate::stage::Node]| -> Vec<KeyedPoint> {
                entries.into_iter().map(|(key, node)| (key.clone(), nodes[*node as usize].point.clone())).collect()
            };
            lists.push(listing(seen.iter().collect(), &self.nodes));
            extras.push(listing(seen.iter().filter(|(key, _)| !original.contains(&***key)).collect(), &self.nodes));
            refined.push(contour.iter().map(|(key, node)| (key.clone(), self.nodes[*node as usize].point.clone())).collect());
        }
        let counters = self.counters()?;
        Ok(Clipped {
            polygons: faces,
            cycles: refined,
            vertex_lists: lists,
            extra_lists: extras,
            points: self.new_points.iter().map(|(key, node)| (key.clone(), self.nodes[*node as usize].point.clone())).collect(),
            snapped: self.snap.moved.clone(),
            lifted: self.lifted.clone(),
            counters,
            note: self.note()?,
            origins: self.nodes.iter().filter_map(|node| node.origin.map(|index| (node.point.clone(), index))).collect(),
        })
    }

    /// The counters of `run`, in the order of the oracle: the stage's, the snap's, the node gap's, the diagonal's.
    fn counters(&self) -> ClipResult<Vec<(&'static str, UBig)>> {
        let tally = &self.tally;
        let stage = [
            self.new_points.len() as u64,
            tally.vertices_at_source_vertex,
            tally.edges_refined,
            tally.faces_in_one_triangle,
            tally.faces_cut,
            tally.faces_cut_by_ears,
            tally.pieces_emitted,
            tally.pieces_merged,
            tally.pieces_kept_separate,
            tally.faces_overhang,
            tally.faces_boundary_mismatch,
            tally.faces_seam_suppressed,
            tally.faces_off_corner_suppressed,
            tally.flow_free_cut_edges,
            tally.predicates,
            tally.divisions,
        ];
        let mut counters: Vec<(&'static str, UBig)> = STAGE_COUNTERS.iter().copied().zip(stage.iter().map(|value| UBig::from(*value))).collect();
        counters.extend(SNAP_COUNTER_NAMES.iter().copied().zip(self.snap.counters.iter().cloned()));
        counters.push((NODE_SIGNS_ZEROED, UBig::from(tally.node_signs_zeroed)));
        counters.push((NODE_EDGE_GAP_MAX, self.node_gap.clone()));
        counters.push((NODE_EDGE_GAP_MAX_CELLS, milli_cells(&self.node_gap_square)?));
        if self.faces_mode {
            counters.extend(self.diagonal_counters()?);
        }
        if self.plan_pairs != 0 {
            counters.extend(PLAN_COUNTERS.iter().copied().zip([UBig::from(self.plan_pairs), UBig::from(self.plan_glued.max(0) as u64)]));
        }
        Ok(counters)
    }

    /// `_note()`.
    fn note(&self) -> ClipResult<String> {
        let tally = &self.tally;
        let mut text = format!(
            "clip_vertices={} (at_source_vertices={}) refined_edges={} faces_in_one_triangle={} faces_cut={} (by_ears={}) pieces={} merged_groups={} \
             kept_separate_groups={} overhang_faces={} boundary_mismatch_faces={} seam_crossings_suppressed_faces={} off_corner_source_vertex_faces={} \
             predicates={} divisions={}",
            self.new_points.len(),
            tally.vertices_at_source_vertex,
            tally.edges_refined,
            tally.faces_in_one_triangle,
            tally.faces_cut,
            tally.faces_cut_by_ears,
            tally.pieces_emitted,
            tally.pieces_merged,
            tally.pieces_kept_separate,
            tally.faces_overhang,
            tally.faces_boundary_mismatch,
            tally.faces_seam_suppressed,
            tally.faces_off_corner_suppressed,
            tally.predicates,
            tally.divisions,
        );
        if self.faces_mode {
            text.push_str(&self.diagonal_note()?);
        }
        Ok(text)
    }

    /// `_diagonal_note()`, with the `repr` of `dict(sorted(Counter(reasons).items()))`.
    fn diagonal_note(&self) -> ClipResult<String> {
        let empty = Verdict::default();
        let verdict = self.verdict.as_ref().unwrap_or(&empty);
        let faces: HashSet<&str> = verdict
            .over
            .iter()
            .filter_map(|(key, _)| match key {
                CellKey::Face(face, _) | CellKey::Group(face, _) | CellKey::Plan(face) => Some(face.as_str()),
                CellKey::Triangle(_) => None,
            })
            .collect();
        let mut reasons: Vec<(&str, usize)> = Vec::new();
        for (_, reason) in &verdict.unmergeable {
            match reasons.iter_mut().find(|(known, _)| known == reason) {
                Some(entry) => entry.1 += 1,
                None => reasons.push((reason, 1)),
            }
        }
        reasons.sort();
        let reasons = format!("{{{}}}", reasons.iter().map(|(reason, count)| format!("'{reason}': {count}")).collect::<Vec<_>>().join(", "));
        let widest = verdict.over.iter().fold(Rat::zero(), |best, (_, depth)| if *depth > best { depth.clone() } else { best });
        let budget_square = chord_budget_square();
        Ok(format!(
            " diagonals: faces_whole={} pieces_across={} cuts_avoided={} faces_cut_not_planar={} faces_cut_unmergeable={}{} \
             max_chord_kept_nm={} max_chord_over_budget_nm={} chord_budget_nm={}",
            self.whole.len(),
            self.across,
            self.avoided,
            faces.len(),
            verdict.unmergeable.len(),
            reasons,
            nanometres(&self.kept_depth)?,
            nanometres(&widest)?,
            nanometres(&budget_square)?,
        ))
    }
}

