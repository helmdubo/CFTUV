//! `exact_mesh` and `canonical_triangles` (`mesh.py`) as native entries: the steps in the order of the stages, under one cost context.

use std::rc::Rc;

use cftuv_clip::faces::Pt;
use cftuv_core::exact::ExactCtx;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::data::{Key, MeshInput, MeshOutput};
use crate::error::MeshResult;
use crate::intern::intern_vertices;
use crate::stations::station_values;
use crate::tally::Tally;
use crate::tessellate::{tessellate_faces, tessellate_triangles, Contour, Faces};

/// `exact_mesh(...)`: `intern_vertices`, `station_values`, `tessellate_faces`. `out` is filled as far as the oracle's effects go: the lines of `notes`
/// appended before a refusal stay (the list is the caller's), the rest is meaningful when the call returns `Ok`.
pub fn exact_mesh(ctx: &mut ExactCtx<'_>, input: &MeshInput, out: &mut MeshOutput) -> MeshResult<()> {
    let mut keys: Vec<Key> = Vec::new();
    let mut cycles: Vec<Vec<(u32, u32)>> = Vec::new();
    let interned = intern_vertices(input, &mut keys, &mut cycles, &mut out.notes);
    let interned = match interned {
        Ok(interned) => interned,
        Err(error) => return Err(error),
    };
    out.keys = keys;
    let mut tally = Tally::new();
    let stations = station_values(ctx, input, &cycles, &mut tally)?;
    let polygons = tessellate_faces(ctx, input, &cycles, &interned.interner, &stations, &mut tally)?;
    out.facts = stations.slots.iter().copied().zip(stations.facts.iter().cloned()).collect::<Vec<((u32, u32), Rc<(SqrtSum, SqrtSum)>)>>();
    out.rungs = stations.rungs.clone();
    out.cycles = cycles;
    out.polygons = polygons;
    out.tally = tally;
    Ok(())
}

/// `canonical_triangles(frame_faces, cycles, budget, reverse)`: the ear-clipped triangles of every merged face by keys. `contours[i]` is the cycle of
/// face `i` (keys and points as the caller holds them), `areas[i]` its doubled area.
pub fn canonical_triangles(ctx: &mut ExactCtx<'_>, contours: &[(Vec<u32>, Vec<Pt>)], areas: &[&SqrtSum], reverse: bool) -> MeshResult<Vec<Faces>> {
    let contours: Vec<Contour> = contours.iter().map(|(keys, points)| Contour { keys: keys.clone(), points: points.clone() }).collect();
    tessellate_triangles(ctx, &contours, areas, reverse)
}
