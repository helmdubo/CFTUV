//! `clip_geometry` and `_cut_by_faces` (`materialize/clip.py`): the whole geometric cut of a domain, both branches.
//!
//! By faces (the law `SOURCE_FACES_CLIPPED_V1`) the cut runs in up to two stages on shared nodes: stage 1 cuts by the
//! merged cells and measures the chord depth of every piece; the cells whose pieces are deeper than the budget are split
//! into triangles and stage 2 cuts again, taking nodes, crossings, chords, the snap, the gap maxima and a COPY of the tally
//! from stage 1. By triangles (`by_faces` false) it is one stage on the triangles of the lift.
//!
//! The only effect of the operation outside its result, besides the cost the context carries (budget, memory, sign
//! counters), is the write of every lifted offset normal into `plane._normal_by_position`; it is returned as an ORDERED list,
//! also when the operation fails (the writes before the failure stay in the oracle).

use std::collections::HashSet;
use std::sync::Arc;

use cftuv_core::exact::ExactCtx;

use crate::cells::{build_cells, CellKey};
use crate::cut::Cut;
use crate::emit::{Clipped, Law};
use crate::error::ClipResult;
use crate::plane::Plane;
use crate::point::Point;
use crate::profile::{scope, Phase};
use crate::pyemu::PyVersion;
use crate::regions::RegionSet;
use crate::warm::Warm;
use crate::stage::{NormalWrite, Stage, Verdict};

/// The keyword arguments of `clip_geometry` (the plane and the budget are the other two).
pub struct ClipInput<'a> {
    pub points: &'a [(String, Point)],
    pub cycles: &'a [Vec<String>],
    pub polygons: &'a [Vec<Vec<String>>],
    pub law: Law,
    /// Pairs of keys: `frozenset[frozenset[str]]`, only asked for membership.
    pub seam: &'a [(String, String)],
    pub fans: Option<&'a [bool]>,
    pub flows: Option<&'a [bool]>,
    pub by_faces: bool,
}

/// The answer of the operation and the normal writes it made before it answered (or failed).
pub struct ClipRun {
    pub result: ClipResult<Clipped>,
    pub writes: Vec<NormalWrite>,
}

/// `clip_geometry(plane, budget, *, points, cycles, polygons, law, seam, fans, flows, by_faces)`.
pub fn clip_geometry(ctx: &mut ExactCtx<'_>, warm: &mut Warm, version: PyVersion, plane: &Plane, input: &ClipInput<'_>) -> ClipRun {
    let mut writes = Vec::new();
    let result = if input.by_faces { cut_by_faces(ctx, warm, version, plane, input, &mut writes) } else { cut_by_triangles(ctx, warm, version, plane, input, &mut writes) };
    ClipRun { result, writes }
}

fn cut_by_triangles(ctx: &mut ExactCtx<'_>, warm: &mut Warm, version: PyVersion, plane: &Plane, input: &ClipInput<'_>, writes: &mut Vec<NormalWrite>) -> ClipResult<Clipped> {
    let stage = Stage::new(plane, ctx, warm, version, input.points, plane.triangle_regions(), false, None)?;
    finish(stage, input, None, writes)
}

/// `stage.flows = flows; return stage.run(...)`: the last stage runs and leaves its normal writes to the caller.
fn finish(mut stage: Stage<'_, '_>, input: &ClipInput<'_>, cuts: Option<Vec<Vec<Cut>>>, writes: &mut Vec<NormalWrite>) -> ClipResult<Clipped> {
    stage.flows = input.flows.map(<[bool]>::to_vec);
    let result = stage.run(input.cycles, input.polygons, input.law, input.seam, input.fans, cuts);
    *writes = std::mem::take(&mut stage.writes);
    result
}

fn cut_by_faces(ctx: &mut ExactCtx<'_>, warm: &mut Warm, version: PyVersion, plane: &Plane, input: &ClipInput<'_>, writes: &mut Vec<NormalWrite>) -> ClipResult<Clipped> {
    // the cells of the first stage are a function of the plane alone: the plane builds them once (`Plane::first_stage`)
    let built = {
        let _b = scope(Phase::BuildCells);
        plane.first_stage()?
    };
    let mut first = Stage::new(plane, &mut *ctx, &mut *warm, version, input.points, built.regions.clone(), true, None)?;
    let cuts = first.cuts_of(input.polygons)?;
    let over = first.over_budget(&cuts)?;
    let unmergeable = built.unmergeable.clone();
    if over.is_empty() {
        first.verdict = Some(Verdict { over, unmergeable });
        return finish(first, input, Some(cuts), writes);
    }
    let shared = first.into_shared();
    let split: HashSet<CellKey> = over.iter().map(|(key, _)| key.clone()).collect();
    let mut memo = built.memo.clone();
    let second_plan = build_cells(&plane.triangles, &split, &mut memo)?;
    let mut second = Stage::new(plane, ctx, warm, version, input.points, Arc::new(RegionSet::new(second_plan.cells)), true, Some(shared))?;
    // the cuts of stage 1 are discarded: stage 2 cuts again on the same nodes (the signs of the cells that were not split are cached)
    let cuts = second.cuts_of(input.polygons)?;
    second.verdict = Some(Verdict { over, unmergeable });
    finish(second, input, Some(cuts), writes)
}
