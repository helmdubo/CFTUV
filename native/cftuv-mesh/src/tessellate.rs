//! `tessellate_faces` and `canonical_triangles` (`mesh.py`): the faces of every merged face.
//!
//! Under `TRIANGLES_V1` only triangles (ear clipping, the sum of the areas closes exactly). Under `QUAD_STRIPS_V1` a strictly convex quadrilateral
//! of a strip stays one face. Under `PLANAR_POLYGONS_V1` (the product law) a merged run is cut at its rungs back into the faces of its source
//! edges, a strip on the exact plane stays ONE polygon of any length if its contour is simple and its UV affine in the chart position
//! (`_plane_ring`), a strictly convex strip face with a bilinear UV stays one face (`_bilinear_ring`), a piece with no such face as a whole is
//! cut by diagonals (`convex_partition`), and a fan is a polygon or a fan of triangles from its apex (`_fan_faces`).
//!
//! The order of the exact questions is the cost and it is the oracle's; so are the names and the order of the numbers written to the tally.

use std::collections::HashMap;

use cftuv_canon::fxhash::FxBuild;
use cftuv_clip::faces::{doubled_shoelace, orientation, shoelace_sign, Pt};
use cftuv_clip::point::Point;
use cftuv_clip::tessellate::{convex_quad_ring, has_right_turn, triangulate_exact};
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::affine::{affine_frame, uv_affine_defect_milli, uv_is_affine_in_chart, uv_vertex_on_affine_map, Frame, Uv};
use crate::data::{Face, Law, MeshInput};
use crate::error::{Closure, MeshError, MeshResult, Whose};
use crate::intern::Interner;
use crate::partition::{self, plan_partition, verify_partition};
use crate::predicates::{contour_is_simple, convex_polygon_ring, counter_clockwise_ring, triangulate_from_apex};
use crate::stations::Stations;
use crate::tally::Tally;

/// The names of the numbers of the law `PLANAR_POLYGONS_V1` (they are the keys of the counters of the materializer).
pub const POLYGON_FACES_EMITTED: &str = "MATERIALIZE_POLYGON_FACES_EMITTED";
pub const POLYGON_FACES_CONCAVE_EMITTED: &str = "MATERIALIZE_POLYGON_FACES_CONCAVE_EMITTED";
pub const POLYGON_FACES_TRIANGULATED_NOT_SIMPLE: &str = "MATERIALIZE_POLYGON_FACES_TRIANGULATED_NOT_SIMPLE";
pub const POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE: &str = "MATERIALIZE_POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE";
pub const CURVED_STRIP_FACES_TRIANGULATED: &str = "MATERIALIZE_CURVED_STRIP_FACES_TRIANGULATED";
pub const MERGED_RUNS_SPLIT_AT_RUNGS: &str = "MATERIALIZE_MERGED_RUNS_SPLIT_AT_RUNGS";
pub const MERGED_RUNS_KEPT_WHOLE: &str = "MATERIALIZE_MERGED_RUNS_KEPT_WHOLE";
pub const FAN_FACES_CUT_BY_NEIGHBOUR: &str = "MATERIALIZE_FAN_FACES_CUT_BY_NEIGHBOUR";
pub const FAN_POLYGON_FACES_EMITTED: &str = "MATERIALIZE_FAN_POLYGON_FACES_EMITTED";
pub const FAN_POLYGON_FACES_CONCAVE_EMITTED: &str = "MATERIALIZE_FAN_POLYGON_FACES_CONCAVE_EMITTED";
pub const FAN_FACES_TRIANGULATED_FROM_APEX: &str = "MATERIALIZE_FAN_FACES_TRIANGULATED_FROM_APEX";
pub const FAN_FACES_NOT_STAR_FROM_APEX: &str = "MATERIALIZE_FAN_FACES_NOT_STAR_FROM_APEX";
pub const QUADS_UV_BILINEAR: &str = "MATERIALIZE_QUADS_UV_BILINEAR";
pub const QUADS_UV_BILINEAR_MAX_MILLI_ALPHA: &str = "MATERIALIZE_QUADS_UV_BILINEAR_MAX_MILLI_ALPHA";
pub const POLYGONS_UV_BILINEAR: &str = "MATERIALIZE_POLYGONS_UV_BILINEAR";
pub const POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA: &str = "MATERIALIZE_POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA";
pub const QUADS_REFUSED_NOT_CONVEX: &str = "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX";

/// A contour by keys and points: what the tessellation reads of a merged face besides its area.
pub struct Contour<'a> {
    pub keys: Vec<u32>,
    pub points: Vec<Pt<'a>>,
}

/// The faces of one merged face: each a list of keys.
pub type Faces = Vec<Vec<u32>>;

fn pt(point: &Point) -> Pt<'_> {
    (&point.0, &point.1)
}

/// `_closes_the_area(total, expected, owner, parts)`: the sum of the areas of the parts is the area of the contour, exactly.
fn closes_the_area(total: &SqrtSum, expected: &SqrtSum, whose: Whose, what: Closure) -> MeshResult<()> {
    if !total.difference_is_zero(expected) {
        return Err(MeshError::NotClosed { whose, what });
    }
    Ok(())
}

/// `_ring_polygon(keys, ring, reverse)`: the polygon counter-clockwise; with `reverse`, the order that keeps the first vertex.
fn ring_polygon(keys: &[u32], ring: &[usize], reverse: bool) -> Vec<u32> {
    let mut ordered: Vec<u32> = ring.iter().map(|index| keys[*index]).collect();
    if reverse {
        ordered[1..].reverse();
    }
    ordered
}

fn triple(keys: &[u32], a: usize, b: usize, c: usize, reverse: bool) -> Vec<u32> {
    if reverse {
        vec![keys[a], keys[c], keys[b]]
    } else {
        vec![keys[a], keys[b], keys[c]]
    }
}

/// `_emitted_triangles(owner, expected, points, keys, triangles, reverse)`: triangles by keys; the sum of their areas closes the contour exactly.
fn emitted_triangles(ctx: &mut ExactCtx<'_>, whose: Whose, expected: &SqrtSum, points: &[Pt], keys: &[u32], triangles: &[[usize; 3]], reverse: bool) -> MeshResult<Faces> {
    let mut total = SqrtSum::zero();
    for [first, second, third] in triangles {
        total = total.add(&doubled_shoelace(ctx.products, &[points[*first], points[*second], points[*third]]));
    }
    closes_the_area(&total, expected, whose, Closure::TriangleAreas)?;
    Ok(triangles.iter().map(|[a, b, c]| triple(keys, *a, *b, *c, reverse)).collect())
}

/// `_triangle_polygons(owner, expected, points, keys, budget, reverse)`: the ear-clipped triangles of a contour.
fn triangle_polygons(ctx: &mut ExactCtx<'_>, whose: Whose, expected: &SqrtSum, points: &[Pt], keys: &[u32], reverse: bool) -> MeshResult<Faces> {
    let triangles = if points.len() >= 3 { triangulate_exact(ctx, points)? } else { None };
    let Some(triangles) = triangles else {
        return Err(MeshError::NoTriangulation { whose, points: points.len() });
    };
    emitted_triangles(ctx, whose, expected, points, keys, &triangles, reverse)
}

/// `canonical_triangles(frame_faces, cycles, budget, reverse)`: `tessellate_faces` under `TRIANGLES_V1`.
pub fn tessellate_triangles(ctx: &mut ExactCtx<'_>, contours: &[Contour], areas: &[&SqrtSum], reverse: bool) -> MeshResult<Vec<Faces>> {
    let mut result = Vec::with_capacity(contours.len());
    for (index, (contour, area)) in contours.iter().zip(areas).enumerate() {
        result.push(triangle_polygons(ctx, Whose { face: index, part: None }, area, &contour.points, &contour.keys, reverse)?);
    }
    Ok(result)
}

/// The first branch of `tessellate_faces` under `QUAD_STRIPS_V1`: a strictly convex quadrilateral of a strip (not a fan) stays one face.
fn tessellate_quads(ctx: &mut ExactCtx<'_>, faces: &[Face], contours: &[Contour], reverse: bool) -> MeshResult<Vec<Faces>> {
    let mut result = Vec::with_capacity(faces.len());
    for (index, (face, contour)) in faces.iter().zip(contours).enumerate() {
        let whose = Whose { face: index, part: None };
        let ring = if face.is_fan() { None } else { convex_quad_ring(ctx, &contour.points)? };
        match ring {
            None => result.push(triangle_polygons(ctx, whose, &face.area, &contour.points, &contour.keys, reverse)?),
            Some(ring) => {
                let ring_points: Vec<Pt> = ring.iter().map(|at| contour.points[*at]).collect();
                closes_the_area(&doubled_shoelace(ctx.products, &ring_points), &face.area, whose, Closure::QuadAreas)?;
                result.push(vec![ring_polygon(&contour.keys, &ring, reverse)]);
            }
        }
    }
    Ok(result)
}

// --------------------------------------------------------------------------
// the polygon law
// --------------------------------------------------------------------------

/// The `(s, r)` of the vertices of one region: `uv_values(frame_face, key)`.
struct UvOf<'s, 'a> {
    stations: &'s Stations<'a>,
    region: u32,
}

impl UvOf<'_, '_> {
    fn of(&self, key: u32) -> Uv<'_> {
        let fact = self.stations.fact(self.region, key).expect("every vertex of a contour has a fact");
        (&fact.0, &fact.1)
    }

    fn all(&self, keys: &[u32]) -> Vec<Uv<'_>> {
        keys.iter().map(|key| self.of(*key)).collect()
    }
}

/// `_plane_ring(points, keys, budget, uv_of, affine_known)`: `(ring, name)` of a face on the exact plane. The ring is `None` for a face that goes to
/// triangles; the name is the number the caller adds ONLY when the face is really emitted.
fn plane_ring(
    ctx: &mut ExactCtx<'_>,
    points: &[Pt],
    keys: &[u32],
    uv: &UvOf,
    affine_known: bool,
) -> MeshResult<(Option<Vec<usize>>, Option<&'static str>)> {
    let Some(ring) = counter_clockwise_ring(ctx, points)? else {
        return Ok((None, None));
    };
    let distinct = {
        let mut seen: Vec<u32> = keys.to_vec();
        seen.sort_unstable();
        seen.dedup();
        seen.len() == keys.len()
    };
    if !distinct || !contour_is_simple(ctx, points)? {
        return Ok((None, Some(POLYGON_FACES_TRIANGULATED_NOT_SIMPLE)));
    }
    if !affine_known {
        let values = uv.all(keys);
        if !uv_is_affine_in_chart(ctx, points, &values)? {
            return Ok((None, Some(POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE)));
        }
    }
    let concave = has_right_turn(ctx, points, &ring)?;
    Ok((Some(ring), if concave { Some(POLYGON_FACES_CONCAVE_EMITTED) } else { None }))
}

/// `_strictly_convex_ring(points, budget)`: the ring of a convex contour with at least four strict left turns, or `None`.
fn strictly_convex_ring(ctx: &mut ExactCtx<'_>, points: &[Pt]) -> MeshResult<Option<Vec<usize>>> {
    let ring = if points.len() == 4 { convex_quad_ring(ctx, points)?.map(|ring| ring.to_vec()) } else { convex_polygon_ring(ctx, points)? };
    let Some(ring) = ring else {
        return Ok(None);
    };
    let size = ring.len();
    let mut corners = 0;
    for position in 0..size {
        if orientation(ctx, points[ring[(position + size - 1) % size]], points[ring[position]], points[ring[(position + 1) % size]])? > 0 {
            corners += 1;
        }
    }
    Ok(if corners >= 4 { Some(ring) } else { None })
}

fn small(value: &IBig) -> MeshResult<u64> {
    u64::try_from(value).map_err(|_| MeshError::Unsupported("a UV defect beyond 64 bits".to_string()))
}

/// `_bilinear_ring(points, keys, budget, uv_of, unit, tally)`: the ring of a convex strip face with a bilinear UV, or `None`; writes the largest kink.
fn bilinear_ring(ctx: &mut ExactCtx<'_>, points: &[Pt], keys: &[u32], uv: &UvOf, unit: &Rat, tally: &mut Tally) -> MeshResult<Option<Vec<usize>>> {
    let Some(ring) = strictly_convex_ring(ctx, points)? else {
        return Ok(None);
    };
    let size = ring.len();
    let values = uv.all(keys);
    let defect = uv_affine_defect_milli(ctx, points, &values, unit, 64)?;
    let name = if size == 4 { QUADS_UV_BILINEAR_MAX_MILLI_ALPHA } else { POLYGONS_UV_BILINEAR_MAX_MILLI_ALPHA };
    tally.raise_to(name, small(&defect)?);
    Ok(Some(ring))
}

/// `_merge_scratch(tally, scratch)`: the kink takes the largest, the rest adds.
fn merge_scratch(tally: &mut Tally, scratch: &Tally) {
    for (name, value) in scratch.items() {
        if name.ends_with("_MAX_MILLI_ALPHA") {
            tally.raise_to(name, value);
        } else {
            tally.add(name, value);
        }
    }
}

fn refused_partition(tally: &mut Tally, reason: &str) {
    tally.add(partition::PIECES_REFUSED, 1);
    tally.add(&format!("{}{}", partition::REFUSED_PREFIX, reason), 1);
}

/// What `_polygon_law_faces` reads for ONE contour besides the contour.
struct PieceContext<'s, 'a> {
    uv: UvOf<'s, 'a>,
    unit: &'s Rat,
    reverse: bool,
    exact_plane: bool,
}

/// `_admitted_polygons(faces, coords, node_keys, look, budget, reverse, in_flow, unit, scratch)`: the parts of a partition admitted by the old law of a piece.
fn admitted_polygons(
    ctx: &mut ExactCtx<'_>,
    parts: &[Vec<usize>],
    points: &[Pt],
    keys: &[u32],
    cx: &PieceContext,
    in_flow: bool,
    scratch: &mut Tally,
) -> MeshResult<Option<(Faces, SqrtSum)>> {
    let mut emitted: Faces = Vec::with_capacity(parts.len());
    let mut total = SqrtSum::zero();
    for nodes in parts {
        let face_points: Vec<Pt> = nodes.iter().map(|node| points[*node]).collect();
        let face_keys: Vec<u32> = nodes.iter().map(|node| keys[*node]).collect();
        let mut ring: Vec<usize> = (0..nodes.len()).collect();
        if nodes.len() > 3 {
            let (found, named) = plane_ring(ctx, &face_points, &face_keys, &cx.uv, false)?;
            let mut found = found;
            if found.is_none() && in_flow && named == Some(POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE) {
                found = bilinear_ring(ctx, &face_points, &face_keys, &cx.uv, cx.unit, scratch)?;
                if found.is_some() {
                    scratch.add(if nodes.len() == 4 { QUADS_UV_BILINEAR } else { POLYGONS_UV_BILINEAR }, 1);
                }
            } else if found.is_some() {
                if let Some(name) = named {
                    scratch.add(name, 1);
                }
            }
            let Some(found) = found else {
                return Ok(None);
            };
            ring = found;
            scratch.add(POLYGON_FACES_EMITTED, (ring.len() > 4) as u64);
        }
        let ring_points: Vec<Pt> = ring.iter().map(|index| face_points[*index]).collect();
        total = total.add(&doubled_shoelace(ctx.products, &ring_points));
        emitted.push(ring_polygon(&face_keys, &ring, cx.reverse));
    }
    Ok(Some((emitted, total)))
}

/// `_convex_faces(piece, cycle, budget, reverse, tally, uv_of, unit, in_flow)`: the faces of a piece by the law `CONVEX_PARTITION_BY_DIAGONALS_V1`, or
/// `None` (the piece stays on ears; the refusal is named in the tally).
fn convex_faces(
    ctx: &mut ExactCtx<'_>,
    area: &SqrtSum,
    points: &[Pt],
    keys: &[u32],
    cx: &PieceContext,
    in_flow: bool,
    tally: &mut Tally,
) -> MeshResult<Option<Faces>> {
    let look = cx.uv.all(keys);
    let mut frames: HashMap<[usize; 3], Frame, FxBuild> = HashMap::default();
    let mut on_map: HashMap<([usize; 3], usize), bool, FxBuild> = HashMap::default();
    let plan = {
        let mut admissible = |ctx: &mut ExactCtx<'_>, verts: &[usize]| -> MeshResult<bool> {
            if in_flow {
                let sub: Vec<Pt> = verts.iter().map(|index| points[*index]).collect();
                if strictly_convex_ring(ctx, &sub)?.is_some() {
                    return Ok(true);
                }
            }
            // `affine(verts)`: the canonical base is the three smallest non-collinear indices, the answers are remembered
            let mut ordered = verts.to_vec();
            ordered.sort_unstable();
            let mut base = None;
            'search: for first in 0..ordered.len() {
                for second in first + 1..ordered.len() {
                    for third in second + 1..ordered.len() {
                        if orientation(ctx, points[ordered[first]], points[ordered[second]], points[ordered[third]])? != 0 {
                            base = Some([ordered[first], ordered[second], ordered[third]]);
                            break 'search;
                        }
                    }
                }
            }
            let Some(base) = base else {
                return Ok(false);
            };
            if !frames.contains_key(&base) {
                frames.insert(base, affine_frame(ctx, points, base));
            }
            for index in &ordered {
                if base.contains(index) {
                    continue;
                }
                let found = match on_map.get(&(base, *index)) {
                    Some(found) => *found,
                    None => {
                        let found = uv_vertex_on_affine_map(ctx, points, &look, base, &frames[&base], *index);
                        on_map.insert((base, *index), found);
                        found
                    }
                };
                if !found {
                    return Ok(false);
                }
            }
            Ok(true)
        };
        plan_partition(ctx, points, &mut admissible)?
    };
    let plan = match plan {
        Err(reason) => {
            refused_partition(tally, reason);
            return Ok(None);
        }
        Ok(plan) => plan,
    };
    if plan.pieces.len() + 2 >= points.len() {
        refused_partition(tally, partition::REASON_NO_GAIN);
        return Ok(None);
    }
    if let Some(reason) = verify_partition(ctx, points, &plan)? {
        refused_partition(tally, reason);
        return Ok(None);
    }
    let sign_of = |ctx: &mut ExactCtx<'_>, ring: &[usize]| -> MeshResult<i8> {
        let uv_points: Vec<Pt> = ring.iter().map(|index| (look[*index].0, look[*index].1)).collect();
        Ok(shoelace_sign(ctx, &uv_points)?)
    };
    let piece_sign = sign_of(ctx, &plan.order)?;
    let mut folded = piece_sign == 0;
    if !folded {
        for part in &plan.pieces {
            if sign_of(ctx, part)? != piece_sign {
                folded = true;
                break;
            }
        }
    }
    if folded {
        refused_partition(tally, partition::REASON_UV_FOLD);
        return Ok(None);
    }
    let mut scratch = Tally::new();
    let admitted = admitted_polygons(ctx, &plan.pieces, points, keys, cx, in_flow, &mut scratch)?;
    let Some((emitted, total)) = admitted else {
        refused_partition(tally, partition::REASON_AREA);
        return Ok(None);
    };
    if !total.difference_is_zero(area) {
        refused_partition(tally, partition::REASON_AREA);
        return Ok(None);
    }
    merge_scratch(tally, &scratch);
    tally.add(partition::PIECES_PARTITIONED, 1);
    tally.add(if plan.exact { partition::PIECES_EXACT } else { partition::PIECES_GREEDY }, 1);
    tally.add(partition::FACES_EMITTED, plan.pieces.len() as u64);
    tally.add(partition::DIAGONALS, plan.pieces.len() as u64 - 1);
    Ok(Some(emitted))
}

/// One piece of a face under the polygon law: its contour and the area it must close.
struct Piece<'a> {
    keys: Vec<u32>,
    points: Vec<Pt<'a>>,
    area: &'a SqrtSum,
    whose: Whose,
}

/// `_contour_polygons(piece, cycle, ...)`: the faces of ONE contour under `PLANAR_POLYGONS_V1`: one polygon, or triangles under a name.
fn contour_polygons(ctx: &mut ExactCtx<'_>, piece: &Piece, cx: &PieceContext, tally: &mut Tally, in_flow: bool, affine_known: bool) -> MeshResult<Faces> {
    let points = &piece.points;
    let keys = &piece.keys;
    let mut named: Option<&'static str> = None;
    if points.len() > 3 {
        let mut ring: Option<Vec<usize>>;
        if cx.exact_plane {
            let (found, name) = plane_ring(ctx, points, keys, &cx.uv, affine_known)?;
            ring = found;
            named = name;
            if in_flow && ring.is_none() && named == Some(POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE) {
                ring = bilinear_ring(ctx, points, keys, &cx.uv, cx.unit, tally)?;
                if ring.is_some() {
                    named = Some(if points.len() == 4 { QUADS_UV_BILINEAR } else { POLYGONS_UV_BILINEAR });
                }
            }
            if ring.is_none() && named == Some(POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE) {
                if let Some(parts) = convex_faces(ctx, piece.area, points, keys, cx, in_flow, tally)? {
                    return Ok(parts);
                }
            }
        } else {
            ring = convex_quad_ring(ctx, points)?.map(|ring| ring.to_vec());
            if ring.is_none() {
                named = Some(if points.len() == 4 { QUADS_REFUSED_NOT_CONVEX } else { CURVED_STRIP_FACES_TRIANGULATED });
            }
        }
        if let Some(ring) = ring {
            let ring_points: Vec<Pt> = ring.iter().map(|index| points[*index]).collect();
            closes_the_area(&doubled_shoelace(ctx.products, &ring_points), piece.area, piece.whose, Closure::PolygonAreas)?;
            tally.add(POLYGON_FACES_EMITTED, (ring.len() > 4) as u64);
            if let Some(name) = named {
                tally.add(name, 1);
            }
            return Ok(vec![ring_polygon(keys, &ring, cx.reverse)]);
        }
    }
    let polygons = triangle_polygons(ctx, piece.whose, piece.area, points, keys, cx.reverse)?;
    // the name is about the face the mesh saw: the refusal of `triangle_polygons` leaves none
    if let Some(name) = named {
        tally.add(name, 1);
    }
    Ok(polygons)
}

/// `_fan_apex(owner, points)`: the index of the vertex of a fan in the contour, or `None`: the point of the node `owner[:2]`.
fn fan_apex(face: &Face, points: &[Pt]) -> Option<usize> {
    let (x, y) = face.apex.as_ref()?;
    points.iter().position(|(px, py)| {
        let matches = |sum: &SqrtSum, wanted: &IBig| sum.as_rational().is_some_and(|found| found.value().is_integer() && found.value().numerator() == wanted);
        matches(px, x) && matches(py, y)
    })
}

/// `_fan_faces(frame_face, cycle, ...)`: the faces of ONE fan under `PLANAR_POLYGONS_V1`.
fn fan_faces(ctx: &mut ExactCtx<'_>, face: &Face, whose: Whose, contour: &Contour, cx: &PieceContext, tally: &mut Tally, affine_known: bool) -> MeshResult<Faces> {
    let (points, keys) = (&contour.points, &contour.keys);
    if points.len() <= 3 {
        return triangle_polygons(ctx, whose, &face.area, points, keys, cx.reverse);
    }
    tally.add(FAN_FACES_CUT_BY_NEIGHBOUR, 1);
    let mut named: Option<&'static str> = None;
    if cx.exact_plane {
        let (ring, name) = plane_ring(ctx, points, keys, &cx.uv, affine_known)?;
        named = name;
        if let Some(ring) = ring {
            let ring_points: Vec<Pt> = ring.iter().map(|index| points[*index]).collect();
            closes_the_area(&doubled_shoelace(ctx.products, &ring_points), &face.area, whose, Closure::FanPolygonAreas)?;
            tally.add(FAN_POLYGON_FACES_EMITTED, 1);
            tally.add(FAN_POLYGON_FACES_CONCAVE_EMITTED, named.is_some() as u64);
            return Ok(vec![ring_polygon(keys, &ring, cx.reverse)]);
        }
    }
    let triangles = match fan_apex(face, points) {
        None => None,
        Some(apex) => triangulate_from_apex(ctx, points, apex)?,
    };
    let polygons = match triangles {
        None => {
            let polygons = triangle_polygons(ctx, whose, &face.area, points, keys, cx.reverse)?;
            tally.add(FAN_FACES_NOT_STAR_FROM_APEX, 1);
            polygons
        }
        Some(triangles) => {
            let polygons = emitted_triangles(ctx, whose, &face.area, points, keys, &triangles, cx.reverse)?;
            tally.add(FAN_FACES_TRIANGULATED_FROM_APEX, 1);
            polygons
        }
    };
    // the reason the cell did not become one face is named on both paths, and only when the mesh saw the face
    if let Some(name) = named {
        tally.add(name, 1);
    }
    Ok(polygons)
}

/// `_rung_pieces(frame_face, key_of)`: the parts of a merged run with their contours by keys, or `None` (the run was not merged, or a part has a
/// point the merged contour lost: a vertex inside a rung; no new vertices are made, so such a run stays whole and is named).
fn rung_pieces<'a>(face: &'a Face, faces: &[Face], interner: &Interner, whose_face: usize) -> Option<Vec<Piece<'a>>> {
    if face.parts.len() < 2 {
        return None;
    }
    let mut pieces = Vec::with_capacity(face.parts.len());
    for (at, part) in face.parts.iter().enumerate() {
        let mut pairs: Vec<(u32, &Point)> = Vec::with_capacity(part.points.len());
        for point in &part.points {
            let key = interner.key_of(faces, point)?;
            pairs.push((key, point));
        }
        // `_cycle(pairs)`
        let mut cycle: Vec<(u32, &Point)> = Vec::with_capacity(pairs.len());
        for item in pairs {
            if cycle.last().is_some_and(|last| last.0 == item.0) {
                continue;
            }
            cycle.push(item);
        }
        if cycle.len() > 1 && cycle[0].0 == cycle[cycle.len() - 1].0 {
            cycle.pop();
        }
        if cycle.len() < 3 {
            return None;
        }
        pieces.push(Piece {
            keys: cycle.iter().map(|(key, _)| *key).collect(),
            points: cycle.iter().map(|(_, point)| pt(point)).collect(),
            area: &part.area,
            whose: Whose { face: whose_face, part: Some(at) },
        });
    }
    Some(pieces)
}

/// `_polygon_law_faces(frame_faces, cycles, ...)`: `tessellate_faces` under `PLANAR_POLYGONS_V1`.
fn polygon_law_faces(
    ctx: &mut ExactCtx<'_>,
    input: &MeshInput,
    contours: &[Contour],
    interner: &Interner,
    stations: &Stations,
    tally: &mut Tally,
) -> MeshResult<Vec<Faces>> {
    let mut result = Vec::with_capacity(input.faces.len());
    for (index, (face, contour)) in input.faces.iter().zip(contours).enumerate() {
        let simple = stations.plain[index];
        let cx = PieceContext { uv: UvOf { stations, region: face.region }, unit: input.lattice_alpha, reverse: input.chart_cw, exact_plane: input.exact_plane };
        let whose = Whose { face: index, part: None };
        if face.is_fan() {
            result.push(fan_faces(ctx, face, whose, contour, &cx, tally, simple)?);
            continue;
        }
        let pieces = rung_pieces(face, input.faces, interner, index);
        if face.parts.len() >= 2 {
            tally.add(if pieces.is_some() { MERGED_RUNS_SPLIT_AT_RUNGS } else { MERGED_RUNS_KEPT_WHOLE }, 1);
        }
        let pieces = match pieces {
            None => vec![Piece { keys: contour.keys.clone(), points: contour.points.clone(), area: &face.area, whose }],
            Some(pieces) => {
                let mut total = SqrtSum::zero();
                for piece in &pieces {
                    total = total.add(piece.area);
                }
                closes_the_area(&total, &face.area, whose, Closure::RunPartAreas)?;
                pieces
            }
        };
        let mut polygons: Faces = Vec::new();
        let cycle_keys: &[u32] = if simple { &contour.keys } else { &[] };
        for piece in &pieces {
            let in_flow = face.in_flow && piece.keys.iter().any(|key| stations.is_rung(face.region, *key));
            let affine_known = simple && piece.keys.iter().all(|key| cycle_keys.contains(key));
            polygons.extend(contour_polygons(ctx, piece, &cx, tally, in_flow, affine_known)?);
        }
        result.push(polygons);
    }
    Ok(result)
}

/// `tessellate_faces(...)` of `exact_mesh`: the faces of every merged face under the law of the domain.
pub fn tessellate_faces(
    ctx: &mut ExactCtx<'_>,
    input: &MeshInput,
    cycles: &[Vec<(u32, u32)>],
    interner: &Interner,
    stations: &Stations,
    tally: &mut Tally,
) -> MeshResult<Vec<Faces>> {
    let contours: Vec<Contour> = input
        .faces
        .iter()
        .zip(cycles)
        .map(|(face, cycle)| Contour { keys: cycle.iter().map(|(key, _)| *key).collect(), points: cycle.iter().map(|(_, vertex)| pt(&face.points[*vertex as usize])).collect() })
        .collect();
    match input.law {
        Law::PlanarPolygons => polygon_law_faces(ctx, input, &contours, interner, stations, tally),
        Law::QuadStrips => tessellate_quads(ctx, input.faces, &contours, input.chart_cw),
        Law::Triangles => {
            let areas: Vec<&SqrtSum> = input.faces.iter().map(|face| &face.area).collect();
            tessellate_triangles(ctx, &contours, &areas, input.chart_cw)
        }
    }
}
