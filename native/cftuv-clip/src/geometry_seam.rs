//! The whole-operation seam of the clip port (opcode 124, test-only): `clip_geometry` on the arguments the oracle saw,
//! decoded from the boundary values the other seams use, answered as the boundary value of `ClippedV1` (the Python side
//! rebuilds the oracle's own types from it). The fast boundary that builds Python objects directly is the next step.
//!
//! Arguments: `[python major, minor, triangles, points, cycles, polygons, law, seam, fans, flows, by_faces, inert]`
//! `triangles`  the seam's triangle records (`seam::triangles_of`), in the order of `plane.triangles`
//! `points`     `[[key, point], ...]` in dictionary order      `cycles`   `[[key, ...], ...]` (only the keys are read)
//! `polygons`   `[[[key, ...], ...], ...]` per face            `law`      0 planar polygons, 1 quad strips, 2 anything else
//! `seam`       `[[key, key], ...]`                            `fans`, `flows`  none or `[bool, ...]`
//! `inert`      `[[name, name], ...]` the chain station plan's pairs of faces, in the order of the oracle's frozenset
//!
//! Answer value (outcome code 0): `[polygons, cycles, vertex lists, extra lists, points, snapped, lifted, counters, note]`
//! with `lifted` entries `[key, [x, y, z], triangle name, normal or none]` and `counters` entries `[name, int]`.
//! Whatever the outcome, the extras of the answer are `[normal writes, compute nanoseconds]`: the writes
//! `[[x, y, z], [nx, ny, nz]]` in the order of the calls (also of a failed operation), and the time of the operation itself
//! (arguments decoded, result not yet encoded).

use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};

use cftuv_core::codec::Value;
use cftuv_core::exact::ExactCtx;

use crate::emit::{Clipped, KeyedPoint, Law};
use crate::error::ClipResult;
use crate::geometry::{clip_geometry, ClipInput, ClipRun};
use crate::plane::Plane;
use crate::point::Point;
use crate::warm::Warm;

thread_local! {
    /// The cross-call cache of the seam's thread, used only while [`enable_warm`] is on (the seam is stateless otherwise: one call, one answer).
    static SEAM_WARM: std::cell::RefCell<Warm> = std::cell::RefCell::new(Warm::new());
}

static SEAM_WARM_ON: AtomicBool = AtomicBool::new(false);

/// Lets consecutive seam calls of this process share one warm cache, as the calls of a session do (the profiler measures chains of alphas this way).
pub fn enable_warm() {
    SEAM_WARM_ON.store(true, Ordering::Relaxed);
}
use crate::seam::{bad, flag_of, float_list, inert_pairs_of, int, list, point_of, point_value, str_of, str_value, triangles_of, ubig_value, usize_of, version_of, Wire};

fn keys_of(value: &Value, what: &str) -> Wire<Vec<String>> {
    list(value, what)?.iter().map(|key| str_of(key, what)).collect()
}

fn flags_of(value: &Value, what: &str) -> Wire<Option<Vec<bool>>> {
    match value {
        Value::None => Ok(None),
        other => Ok(Some(list(other, what)?.iter().map(|flag| flag_of(flag, what)).collect::<Wire<_>>()?)),
    }
}

fn law_of(value: &Value) -> Wire<Law> {
    match usize_of(value, "a law")? {
        0 => Ok(Law::PlanarPolygons),
        1 => Ok(Law::QuadStrips),
        2 => Ok(Law::Ears),
        _ => Err(bad("a law code")),
    }
}

fn keyed_points(entries: &[KeyedPoint]) -> Value {
    Value::List(entries.iter().map(|(key, point)| Value::List(vec![str_value(key), point_value(point)])).collect())
}

fn keyed_lists(lists: &[Vec<KeyedPoint>]) -> Value {
    Value::List(lists.iter().map(|entries| keyed_points(entries)).collect())
}

fn result_value(clipped: &Clipped) -> Value {
    let key_tuple = |keys: &Vec<Rc<str>>| Value::List(keys.iter().map(|key| str_value(key)).collect());
    let polygons = Value::List(clipped.polygons.iter().map(|face| Value::List(face.iter().map(key_tuple).collect())).collect());
    let snapped = Value::List(clipped.snapped.iter().map(|(key, point)| Value::List(vec![str_value(key), point_value(point)])).collect());
    let lifted = Value::List(
        clipped
            .lifted
            .iter()
            .map(|(key, lifted)| {
                let normal = lifted.normal.as_ref().map_or(Value::None, |normal| float_list(normal));
                Value::List(vec![str_value(key), float_list(&lifted.position), str_value(&lifted.triangle), normal])
            })
            .collect(),
    );
    let counters = Value::List(clipped.counters.iter().map(|(name, value)| Value::List(vec![str_value(name), ubig_value(value)])).collect());
    Value::List(vec![
        polygons,
        keyed_lists(&clipped.cycles),
        keyed_lists(&clipped.vertex_lists),
        keyed_lists(&clipped.extra_lists),
        keyed_points(&clipped.points),
        snapped,
        lifted,
        counters,
        str_value(&clipped.note),
    ])
}

/// Opcode 124. `extras` gets `[writes, compute nanoseconds]` whatever the outcome.
pub fn clip_geometry_seam(args: &[Value], ctx: &mut ExactCtx<'_>, extras: &mut Vec<Value>) -> Wire<ClipResult<Value>> {
    let version = version_of(&args[0], &args[1]);
    let plane = Plane::new(triangles_of(&args[2])?);
    let mut points: Vec<(String, Point)> = Vec::new();
    for entry in list(&args[3], "named points")? {
        let [key, found] = crate::seam::fixed::<2>(entry, "a named point")?;
        points.push((str_of(key, "a point key")?, point_of(found)?));
    }
    let cycles: Vec<Vec<String>> = list(&args[4], "cycles")?.iter().map(|cycle| keys_of(cycle, "a cycle")).collect::<Wire<_>>()?;
    let polygons: Vec<Vec<Vec<String>>> = list(&args[5], "polygons")?
        .iter()
        .map(|face| list(face, "a face")?.iter().map(|keys| keys_of(keys, "a polygon")).collect::<Wire<_>>())
        .collect::<Wire<_>>()?;
    let law = law_of(&args[6])?;
    let mut seam: Vec<(String, String)> = Vec::new();
    for entry in list(&args[7], "seam pairs")? {
        let [first, second] = crate::seam::fixed::<2>(entry, "a seam pair")?;
        seam.push((str_of(first, "a seam key")?, str_of(second, "a seam key")?));
    }
    let (fans, flows) = (flags_of(&args[8], "fans")?, flags_of(&args[9], "flows")?);
    let by_faces = flag_of(&args[10], "by_faces")?;
    let inert = inert_pairs_of(&args[11])?;
    let version = match version {
        Ok(found) => found,
        Err(error) => {
            extras.extend([Value::List(Vec::new()), int(0u8)]);
            return Ok(Err(error));
        }
    };
    let input = ClipInput { points: &points, cycles: &cycles, polygons: &polygons, law, seam: &seam, fans: fans.as_deref(), flows: flows.as_deref(), by_faces, inert: &inert };
    let started = std::time::Instant::now();
    let ClipRun { result, writes } = if SEAM_WARM_ON.load(Ordering::Relaxed) {
        SEAM_WARM.with(|warm| clip_geometry(ctx, &mut warm.borrow_mut(), version, &plane, &input))
    } else {
        clip_geometry(ctx, &mut Warm::disabled(), version, &plane, &input)
    };
    let elapsed = started.elapsed().as_nanos() as u64;
    let writes = writes.iter().map(|write| Value::List(vec![float_list(&write.position), float_list(&write.normal)])).collect();
    extras.extend([Value::List(writes), int(elapsed)]);
    Ok(result.map(|clipped| result_value(&clipped)))
}
