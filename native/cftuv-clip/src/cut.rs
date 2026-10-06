//! `ClipStageV1`, second part: phase 1 of the stage, the cut of every polygon by the regions its box meets
//! (`_cut`, `_by_ears`, `_merged`, `_positive_pieces`, `_closed`, `_covers_by_construction`) and the depth estimate of
//! the pieces of merged cells (`piece_chord`, `over_budget`) that decides the second stage of the law by faces.
//!
//! The graph helpers keep Python dictionary semantics where the ORDER is observable: `_merged` walks a dictionary with
//! deletions (`directed`) whose first surviving half-edge starts the loop, so it is an `OrderedMap`; the half-edge
//! sets of `_boundary_is` and `_covers_by_construction` are only asked for membership and run on plain hash maps.

use std::rc::Rc;

use cftuv_canon::ordered::OrderedMap;
use cftuv_core::num::UBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::cells::{chord_of, CellKey};
use crate::error::{ClipError, ClipResult};
use crate::faces::{self, Pt};
use crate::fxhash::{FxMap, FxSet};
use crate::point::Point;
use crate::profile::{scope, Phase};
use crate::stage::{pair, NodeId, Pair, Stage};
use crate::tessellate;

pub const TESSELLATION_DID_NOT_CLOSE: &str = "TESSELLATION_DID_NOT_CLOSE";

/// `(triangle or cell, nodes of the piece, glued count)`: an entry of `_Cut.pieces`. `glued` is the oracle's
/// `self.glued[id(piece)]`: how many pieces a glued piece was made of (zero for a piece of the cut itself).
#[derive(Clone, Debug)]
pub struct Piece {
    pub ti: usize,
    pub nodes: Vec<NodeId>,
    pub glued: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Suppressed {
    No,
    Seam,
    Corner,
}

/// `_Cut`: the outcome of phase 1 for one polygon.
#[derive(Clone, Debug)]
pub struct Cut {
    /// Counter-clockwise nodes of the polygon, without edge vertices.
    pub nodes: Vec<NodeId>,
    pub flip: bool,
    /// `None`: the area did not close (an overhang), or the cut was suppressed.
    pub pieces: Option<Vec<Piece>>,
    pub by_ears: bool,
    pub merged: u64,
    pub kept: u64,
    pub suppressed: Suppressed,
}

/// `[(region, [piece nodes, ...])]`: the groups the cut of one polygon found, before the empty ones are dropped.
type Groups = Vec<(usize, Vec<Vec<NodeId>>)>;

/// `CLIP_DIAGONAL_CHORD_BUDGET ** 2`: `(1/200)^2`.
pub(crate) fn chord_budget_square() -> Rat {
    Rat::reduced(1.into(), UBig::from(40_000u32))
}

pub(crate) fn pts(owned: &[Rc<Point>]) -> Vec<Pt<'_>> {
    owned.iter().map(|point| (&point.0, &point.1)).collect()
}

pub(crate) fn refusal(outcome: &'static str, detail: String) -> ClipError {
    ClipError::Refusal { outcome, detail }
}

impl<'a, 'c> Stage<'a, 'c> {
    /// `_pruned`: consecutive repeats dropped (also first against last); fewer than three nodes is no piece.
    fn pruned(nodes: Vec<NodeId>) -> Vec<NodeId> {
        let mut out: Vec<NodeId> = Vec::with_capacity(nodes.len());
        for node in nodes {
            if out.last() != Some(&node) {
                out.push(node);
            }
        }
        if out.len() > 1 && out.first() == out.last() {
            out.pop();
        }
        if out.len() >= 3 {
            out
        } else {
            Vec::new()
        }
    }

    /// `_area`: the doubled area of a node loop (exact value, no sign asked).
    fn area(&mut self, nodes: &[NodeId]) -> SqrtSum {
        let owned = self.points_of(nodes);
        faces::doubled_shoelace(self.ctx.products, &pts(&owned))
    }

    /// `_ears(points)`: the ear triangulation of a node loop, or the named refusal.
    pub(crate) fn ears_of(&mut self, nodes: &[NodeId]) -> ClipResult<Vec<[usize; 3]>> {
        let owned = self.points_of(nodes);
        let _t = scope(Phase::Triangulate);
        match tessellate::triangulate_exact(self.ctx, &pts(&owned))? {
            Some(ears) => Ok(ears),
            None => Err(refusal(TESSELLATION_DID_NOT_CLOSE, format!("a clip polygon of {} vertices has no triangulation", nodes.len()))),
        }
    }

    /// `_positive_pieces`: the pieces of positive area; a piece that turns against its polygon is a refusal.
    fn positive_pieces(&mut self, groups: Groups) -> ClipResult<Vec<Piece>> {
        let mut found = Vec::new();
        for (ti, pieces) in groups {
            for nodes in pieces {
                let owned = self.points_of(&nodes);
                let sign = {
                    let _s = scope(Phase::ShoelaceSign);
                    faces::shoelace_sign(self.ctx, &pts(&owned))?
                };
                if sign > 0 {
                    found.push(Piece { ti, nodes, glued: 0 });
                } else if sign < 0 {
                    return Err(refusal(
                        TESSELLATION_DID_NOT_CLOSE,
                        format!("CLIP_PIECE_REVERSED: a piece in source triangle {} turns against its polygon", self.regions[ti].name),
                    ));
                }
            }
        }
        Ok(found)
    }

    /// `_merged(group)`: one contour without repeated vertices from the pieces of one region (opposite half-edges cancel),
    /// or `None`.
    pub(crate) fn merged(group: &[Vec<NodeId>]) -> Option<Vec<NodeId>> {
        let _p = scope(Phase::Merged);
        let mut directed: OrderedMap<Pair2, ()> = OrderedMap::new();
        for piece in group {
            for (position, node) in piece.iter().enumerate() {
                let edge = (*node, piece[(position + 1) % piece.len()]);
                if directed.contains_key(&(edge.1, edge.0)) {
                    directed.remove(&(edge.1, edge.0));
                } else if directed.contains_key(&edge) {
                    return None;
                } else {
                    directed.set(edge, ());
                }
            }
        }
        let mut successor: OrderedMap<NodeId, NodeId> = OrderedMap::new();
        for ((start, end), _) in directed.iter() {
            if successor.contains_key(start) {
                return None;
            }
            successor.set(*start, *end);
        }
        if successor.len() == 0 {
            return None;
        }
        let first = *successor.keys().next()?;
        let mut walk = vec![first];
        let mut current = *successor.get(&first)?;
        while current != first {
            if walk.contains(&current) || !successor.contains_key(&current) || walk.len() > successor.len() {
                return None;
            }
            walk.push(current);
            current = *successor.get(&current)?;
        }
        if walk.len() == successor.len() {
            Some(walk)
        } else {
            None
        }
    }

    /// `_by_ears`: the ears of a non-convex contour cut by each candidate region; the pieces of one region fold back into one
    /// contour when they can. `(groups, folded, kept apart)`.
    fn by_ears(&mut self, nodes: &[NodeId], ears: &[[usize; 3]], candidates: &[usize]) -> ClipResult<(Groups, u64, u64)> {
        let (mut groups, mut merged_count, mut kept_count) = (Vec::new(), 0u64, 0u64);
        for &ti in candidates {
            let mut pieces: Vec<Vec<NodeId>> = Vec::new();
            for ear in ears {
                let contour: Vec<NodeId> = ear.iter().map(|index| nodes[*index]).collect();
                let clipped = self.clip(contour, ti)?;
                let piece = Self::pruned(clipped);
                if !piece.is_empty() {
                    pieces.push(piece);
                }
            }
            if pieces.is_empty() {
                continue;
            }
            let merged = if pieces.len() > 1 { Self::merged(&pieces) } else { None };
            if pieces.len() > 1 {
                merged_count += u64::from(merged.is_some());
                kept_count += u64::from(merged.is_none());
            }
            groups.push((ti, match merged {
                Some(whole) => vec![whole],
                None => pieces,
            }));
        }
        Ok((groups, merged_count, kept_count))
    }

    /// `_boundary_is`: the boundary of the pieces (inner half-edges cancelled) is exactly the subdivided contour.
    pub(crate) fn boundary_is(pieces: &[Piece], nodes: &[NodeId]) -> bool {
        let _p = scope(Phase::BoundaryIs);
        let mut half: FxSet<Pair2> = FxSet::default();
        for piece in pieces {
            for (position, node) in piece.nodes.iter().enumerate() {
                let edge = (*node, piece.nodes[(position + 1) % piece.nodes.len()]);
                if half.contains(&(edge.1, edge.0)) {
                    half.remove(&(edge.1, edge.0));
                } else if half.contains(&edge) {
                    return false;
                } else {
                    half.insert(edge);
                }
            }
        }
        let size = nodes.len();
        let wanted: FxSet<Pair2> = (0..size).map(|index| (nodes[index], nodes[(index + 1) % size])).collect();
        half == wanted
    }

    /// `_covers_by_construction`: the unpaired edges of the pieces are exactly the edges of the polygon, subdivided by
    /// vertices that lie on their lines EXACTLY (`on`): the area equality is then an identity.
    fn covers_by_construction(&self, nodes: &[NodeId], pieces: &[Piece]) -> bool {
        let _p = scope(Phase::Covers);
        let mut half: FxSet<Pair2> = FxSet::default();
        for piece in pieces {
            let size = piece.nodes.len();
            for (position, node) in piece.nodes.iter().enumerate() {
                let edge = (*node, piece.nodes[(position + 1) % size]);
                if half.contains(&(edge.1, edge.0)) {
                    half.remove(&(edge.1, edge.0));
                } else if half.contains(&edge) {
                    return false;
                } else {
                    half.insert(edge);
                }
            }
        }
        let mut successor: FxMap<NodeId, NodeId> = FxMap::default();
        for (start, end) in &half {
            if successor.contains_key(start) {
                return false;
            }
            successor.insert(*start, *end);
        }
        let Some(on) = &self.on else {
            return false;
        };
        let (size, mut used) = (nodes.len(), 0usize);
        for index in 0..size {
            let (first, last) = (nodes[index], nodes[(index + 1) % size]);
            let line = pair(first, last);
            let mut current = first;
            while current != last {
                let Some(next) = successor.get(&current) else {
                    return false;
                };
                current = *next;
                used += 1;
                if used > half.len() {
                    return false;
                }
                if current != last && !on.get(&current).is_some_and(|lines| lines.contains(&line)) {
                    return false;
                }
            }
        }
        used == half.len()
    }

    /// `_closed`: the doubled areas of the pieces add up to the polygon's EXACTLY (the pieces cover it without overhang).
    fn closed(&mut self, nodes: &[NodeId], pieces: &[Piece], sign: i8) -> ClipResult<bool> {
        let _p = scope(Phase::Closed);
        if pieces.is_empty() {
            return Ok(sign == 0);
        }
        if self.covers_by_construction(nodes, pieces) {
            return Ok(true);
        }
        let area = self.area(nodes);
        let mut total = SqrtSum::zero();
        for piece in pieces {
            let part = self.area(&piece.nodes);
            total = total.add(&part);
        }
        Ok(total.sub(&area).is_zero())
    }

    /// `_cut(keys)`: phase 1 for one polygon.
    pub(crate) fn cut(&mut self, keys: &[String]) -> ClipResult<Cut> {
        let _p = scope(Phase::Cut);
        let mut nodes = keys.iter().map(|key| self.node_for(key)).collect::<ClipResult<Vec<NodeId>>>()?;
        let owned = self.points_of(&nodes);
        let sign = {
            let _s = scope(Phase::ShoelaceSign);
            faces::shoelace_sign(self.ctx, &pts(&owned))?
        };
        let flip = sign < 0;
        if flip {
            nodes.reverse();
        }
        let owned = self.points_of(&nodes);
        let points = pts(&owned);
        let count = nodes.len();
        let convex = if count == 3 {
            true
        } else {
            let ring: Vec<usize> = (0..count).collect();
            let _h = scope(Phase::HasRightTurn);
            !tessellate::has_right_turn(self.ctx, &points, &ring)?
        };
        let candidates = self.candidates(&nodes)?;
        let mut on: FxMap<NodeId, FxSet<Pair>> = FxMap::default();
        for (index, node) in nodes.iter().enumerate() {
            for other in [nodes[(index + count - 1) % count], nodes[(index + 1) % count]] {
                on.entry(*node).or_default().insert(pair(*node, other));
            }
        }
        self.on = Some(on);
        let body = self.cut_body(&nodes, convex, &candidates, sign);
        self.on = None;
        let (pieces, closed, by_ears, merged, kept) = body?;
        Ok(Cut { nodes, flip, pieces: if closed { Some(pieces) } else { None }, by_ears, merged, kept, suppressed: Suppressed::No })
    }

    /// The `try` body of `_cut`: `(pieces, closed, by ears, folded, kept apart)`.
    fn cut_body(&mut self, nodes: &[NodeId], convex: bool, candidates: &[usize], sign: i8) -> ClipResult<(Vec<Piece>, bool, bool, u64, u64)> {
        let (groups, by_ears, merged, kept) = if convex {
            let mut groups: Groups = Vec::new();
            for &ti in candidates {
                let clipped = self.clip(nodes.to_vec(), ti)?;
                let piece = Self::pruned(clipped);
                if !piece.is_empty() {
                    groups.push((ti, vec![piece]));
                }
            }
            (groups, false, 0, 0)
        } else {
            let ears = self.ears_of(nodes)?;
            let (groups, merged, kept) = self.by_ears(nodes, &ears, candidates)?;
            (groups, true, merged, kept)
        };
        let pieces = self.positive_pieces(groups)?;
        let closed = self.closed(nodes, &pieces, sign)?;
        Ok((pieces, closed, by_ears, merged, kept))
    }

    /// `cuts_of(polygons)`: phase 1 for every polygon of every face.
    pub fn cuts_of(&mut self, polygons: &[Vec<Vec<String>>]) -> ClipResult<Vec<Vec<Cut>>> {
        let mut all = Vec::with_capacity(polygons.len());
        for face_polygons in polygons {
            let mut row = Vec::with_capacity(face_polygons.len());
            for keys in face_polygons {
                row.push(self.cut(keys)?);
            }
            all.push(row);
        }
        Ok(all)
    }

    /// `piece_chord(ti, piece)`: `(squared chord depth, diagonals the piece crosses)` of a piece of a merged cell, cached by
    /// the cell and the nodes of the piece (the cache is shared by the two stages).
    pub(crate) fn piece_chord(&mut self, ti: usize, piece: &[NodeId]) -> ClipResult<(Rat, i64)> {
        let _p = scope(Phase::PieceChord);
        let missing = || ClipError::Unsupported("a merged cell without a depth estimate".to_string());
        if self.regions[ti].group.is_some() {
            return Ok((self.regions[ti].flat_square.clone().ok_or_else(missing)?, 0));
        }
        let ident = (self.region_keys[ti], piece.to_vec());
        if let Some(found) = self.chords.get(&ident) {
            return Ok(found.clone());
        }
        let owned = self.points_of(piece);
        let diagonals = self.regions[ti].diagonals.clone();
        let values: Vec<Vec<SqrtSum>> = diagonals
            .iter()
            .map(|(member, index)| owned.iter().map(|point| self.plane.line_value(*member, *index, &point.0, &point.1)).collect())
            .collect();
        let cell = &self.regions[ti];
        let jump = cell.hinge.as_ref().map(|hinge| &hinge.jump_square);
        let (depth, crossed) = {
            let _c = scope(Phase::ChordOf);
            chord_of(self.ctx, jump, cell.flat_square.as_ref(), &values)?
        };
        let found = (depth.ok_or_else(missing)?, crossed);
        self.chords.insert(ident, found.clone());
        Ok(found)
    }

    /// `over_budget(cuts)`: `{cell or group key: the largest squared chord depth}` of the cells with a piece deeper than the budget.
    pub fn over_budget(&mut self, cuts: &[Vec<Cut>]) -> ClipResult<Vec<(CellKey, Rat)>> {
        let mut found: Vec<(CellKey, Rat)> = Vec::new();
        let limit = chord_budget_square();
        for face_cuts in cuts {
            for cut in face_cuts {
                let Some(pieces) = &cut.pieces else {
                    continue;
                };
                for piece in pieces {
                    if self.regions[piece.ti].members.len() < 2 && self.regions[piece.ti].group.is_none() {
                        continue;
                    }
                    let depth = self.piece_chord(piece.ti, &piece.nodes)?.0;
                    if depth > limit {
                        let key = self.regions[piece.ti].group.clone().unwrap_or_else(|| self.regions[piece.ti].key.clone());
                        match found.iter_mut().find(|(known, _)| *known == key) {
                            Some(entry) => {
                                if depth > entry.1 {
                                    entry.1 = depth;
                                }
                            }
                            None => found.push((key, depth)),
                        }
                    }
                }
            }
        }
        Ok(found)
    }
}

type Pair2 = (NodeId, NodeId);
