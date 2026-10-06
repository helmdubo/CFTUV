//! The differential seams of the clip port: every function of the first third behind ONE entry (`run`), so the Python
//! test harness can call the native function on exactly the arguments the oracle saw and compare the answers.
//!
//! Wire: a request is one boundary value `[header, opcode, [argument, ...]]` (`cftuv_core::codec`), the answer one
//! boundary value `[outcome, counts, articles, log, state]` (the cost answer of `cftuv_core::session`), for every seam
//! (a pure one answers zero counts, the articles as they came and an empty log). `header` is the cost header
//! `[options, sync, budget]` or none. Strings, which the codec has no tag for, travel as `[None, int]` where the int
//! holds the UTF-8 bytes followed by one `0x01` sentinel byte (little endian), so no byte of the text is lost.
//!
//! Outcome codes of the cost answer: 0 ok, 1..7 as `cftuv_core::session`, plus 8 `OverflowError` (`[8, kind]`),
//! 9 `ZeroDivisionError` (`[9, text]`), 10 `ValueError` (`[10, text]`), 11 `MaterializationRefusal`
//! (`[11, outcome, detail]`), 12 NativeUnsupported (`[12, text]`), 13 `KeyError` (`[13, key]`).
//!
//! A seam may append extras to the answer (after the nanoseconds of the seam): the whole-operation seam of opcode 124
//! (`geometry_seam`) answers `[normal writes, compute nanoseconds]` there, whatever the outcome.

use std::collections::HashSet;

use cftuv_core::codec::{DecodeError, Reader, Value, Writer};
use cftuv_core::exact::ExactCtx;
use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;
use cftuv_core::session::{outcome_value, CostRun, Session};
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};

use crate::cells::{self, Built, CellKey, CellMemo, CellPlan, ClipCell, MemoKey, MemoValue};
use crate::edge::{self, EdgeConstants};
use crate::error::{ClipError, ClipResult};
use crate::faces::{self, Pt};
use crate::geometry_seam;
use crate::lift;
use crate::numeric;
use crate::order;
use crate::plane::{self, ChartPoint, Plane, Triangle};
use crate::point::{self, Point};
use crate::pyemu::{self, PyVersion};
use crate::snap;
use crate::tessellate;

/// `(opcode, name)` of the seams, in the order the Python harness uses.
pub const SEAMS: &[(u8, &str)] = &[
    (100, "SORT_SEQUENCE"),
    (101, "FLOAT_SUM"),
    (102, "LINE_VALUE"),
    (103, "WINDOW"),
    (104, "VALUES_IN"),
    (105, "STRETCH_SQUARE"),
    (106, "LIFT_KNOWN"),
    (107, "NANOMETRES"),
    (108, "MILLI_CELLS"),
    (109, "ORIENTATION"),
    (110, "SHOELACE_SIGN"),
    (111, "DOUBLED_SHOELACE"),
    (112, "WITHIN_EDGE_GAP"),
    (113, "HINGE_DEPTH_SQUARE"),
    (114, "CHORD_OF"),
    (115, "EDGE_CONSTANTS"),
    (116, "CHEAP_SIGN"),
    (117, "RATIONAL_PAIR"),
    (118, "BUILD_CELLS"),
    (119, "SNAP_SOURCE_VERTICES"),
    (120, "TRIANGULATE_EXACT"),
    (121, "CONVEX_QUAD_RING"),
    (122, "HAS_RIGHT_TURN"),
    (123, "ORDERED"),
    (124, "CLIP_GEOMETRY"),
];

/// A request the seam cannot read (a bug of the caller, never an answer).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeamError(pub String);

impl std::fmt::Display for SeamError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "clip seam: {}", self.0)
    }
}

impl std::error::Error for SeamError {}

impl From<DecodeError> for SeamError {
    fn from(error: DecodeError) -> SeamError {
        SeamError(format!("cannot decode the request ({error})"))
    }
}

pub(crate) fn bad(what: &str) -> SeamError {
    SeamError(format!("bad argument: {what}"))
}

pub(crate) type Wire<T> = Result<T, SeamError>;

// --------------------------------------------------------------------------
// reading arguments
// --------------------------------------------------------------------------

pub(crate) fn list<'a>(value: &'a Value, what: &str) -> Wire<&'a [Value]> {
    match value {
        Value::List(items) => Ok(items),
        _ => Err(bad(what)),
    }
}

pub(crate) fn fixed<'a, const N: usize>(value: &'a Value, what: &str) -> Wire<&'a [Value; N]> {
    list(value, what)?.try_into().map_err(|_| bad(what))
}

pub(crate) fn int_of(value: &Value, what: &str) -> Wire<IBig> {
    match value {
        Value::Int(number) => Ok(number.clone()),
        _ => Err(bad(what)),
    }
}

pub(crate) fn usize_of(value: &Value, what: &str) -> Wire<usize> {
    match value {
        Value::Int(number) => usize::try_from(number).map_err(|_| bad(what)),
        _ => Err(bad(what)),
    }
}

pub(crate) fn flag_of(value: &Value, what: &str) -> Wire<bool> {
    match value {
        Value::Bool(flag) => Ok(*flag),
        _ => Err(bad(what)),
    }
}

pub(crate) fn float_of(value: &Value, what: &str) -> Wire<f64> {
    match value {
        Value::Float(number) => Ok(*number),
        _ => Err(bad(what)),
    }
}

pub(crate) fn rat_of(value: &Value, what: &str) -> Wire<Rat> {
    match value {
        Value::Int(number) => Ok(Rat::from_int(number.clone())),
        Value::Frac(number) => Ok(number.clone()),
        _ => Err(bad(what)),
    }
}

pub(crate) fn sum_of<'a>(value: &'a Value, what: &str) -> Wire<&'a SqrtSum> {
    match value {
        Value::Sum(found) => Ok(found),
        _ => Err(bad(what)),
    }
}

/// A string: `[None, int]`, the int holding the UTF-8 bytes and a final `0x01`.
pub(crate) fn str_of(value: &Value, what: &str) -> Wire<String> {
    let [Value::None, Value::Int(number)] = fixed::<2>(value, what)? else {
        return Err(bad(what));
    };
    if num::is_negative(number) {
        return Err(bad(what));
    }
    let mut bytes = num::magnitude_le_bytes(&num::magnitude(number));
    if bytes.pop() != Some(1) {
        return Err(bad(what));
    }
    String::from_utf8(bytes).map_err(|_| bad(what))
}

pub(crate) fn str_value(text: &str) -> Value {
    let mut bytes = text.as_bytes().to_vec();
    bytes.push(1);
    Value::List(vec![Value::None, Value::Int(IBig::from(UBig::from_le_bytes(&bytes)))])
}

fn chart_point_of(value: &Value) -> Wire<ChartPoint> {
    let [x, y] = fixed::<2>(value, "a chart point")?;
    Ok((rat_of(x, "a chart x")?, rat_of(y, "a chart y")?))
}

fn chart_of(value: &Value) -> Wire<Vec<ChartPoint>> {
    list(value, "a chart")?.iter().map(chart_point_of).collect()
}

pub(crate) fn point_of(value: &Value) -> Wire<Point> {
    let [x, y] = fixed::<2>(value, "a point")?;
    Ok((sum_of(x, "a point x")?.clone(), sum_of(y, "a point y")?.clone()))
}

pub(crate) fn points_of(value: &Value) -> Wire<Vec<Point>> {
    list(value, "points")?.iter().map(point_of).collect()
}

fn triangle_of(value: &Value) -> Wire<Triangle> {
    let [name, chart, corners, area, bbox, normals, face] = fixed::<7>(value, "a triangle")?;
    let chart: [ChartPoint; 3] = chart_of(chart)?.try_into().map_err(|_| bad("a triangle chart of three points"))?;
    let corners: Vec<[Rat; 3]> = list(corners, "triangle corners")?
        .iter()
        .map(|corner| {
            let axes: Vec<Rat> = list(corner, "a corner")?.iter().map(|axis| rat_of(axis, "a corner axis")).collect::<Wire<_>>()?;
            axes.try_into().map_err(|_| bad("a corner of three axes"))
        })
        .collect::<Wire<_>>()?;
    let corners: [[Rat; 3]; 3] = corners.try_into().map_err(|_| bad("three corners"))?;
    let bbox: Vec<f64> = list(bbox, "a box")?.iter().map(|edge| float_of(edge, "a box edge")).collect::<Wire<_>>()?;
    let bbox: [f64; 4] = bbox.try_into().map_err(|_| bad("a box of four floats"))?;
    let normals = match normals {
        Value::None => None,
        other => {
            let rows: Vec<[f64; 3]> = list(other, "normals")?
                .iter()
                .map(|row| {
                    let axes: Vec<f64> = list(row, "a normal")?.iter().map(|axis| float_of(axis, "a normal axis")).collect::<Wire<_>>()?;
                    axes.try_into().map_err(|_| bad("a normal of three floats"))
                })
                .collect::<Wire<_>>()?;
            Some(<[[f64; 3]; 3]>::try_from(rows).map_err(|_| bad("three normals"))?)
        }
    };
    Ok(Triangle {
        name: str_of(name, "a triangle name")?,
        chart,
        corners,
        twice_area: rat_of(area, "a twice area")?,
        bbox,
        normals,
        face: str_of(face, "a triangle face")?,
    })
}

pub(crate) fn triangles_of(value: &Value) -> Wire<Vec<Triangle>> {
    list(value, "triangles")?.iter().map(triangle_of).collect()
}

fn key_of(value: &Value) -> Wire<CellKey> {
    let items = list(value, "a cell key")?;
    let tag = str_of(items.first().ok_or_else(|| bad("a cell key tag"))?, "a cell key tag")?;
    match (tag.as_str(), &items[1..]) {
        ("t", [index]) => Ok(CellKey::Triangle(usize_of(index, "a cell index")?)),
        ("f", [face, index]) => Ok(CellKey::Face(str_of(face, "a cell face")?, usize_of(index, "a cell index")?)),
        ("g", [face, index]) => Ok(CellKey::Group(str_of(face, "a group face")?, usize_of(index, "a group index")?)),
        _ => Err(bad("a cell key")),
    }
}

pub(crate) fn version_of(major: &Value, minor: &Value) -> ClipResult<PyVersion> {
    let number = |value: &Value| match value {
        Value::Int(found) => u32::try_from(found).map_err(|_| ClipError::Unsupported("a python version number".into())),
        _ => Err(ClipError::Unsupported("a python version number".into())),
    };
    PyVersion::from_version(number(major)?, number(minor)?)
}

// --------------------------------------------------------------------------
// writing answers
// --------------------------------------------------------------------------

pub(crate) fn int(number: impl Into<IBig>) -> Value {
    Value::Int(number.into())
}

pub(crate) fn ubig_value(number: &UBig) -> Value {
    Value::Int(IBig::from(number.clone()))
}

fn rat_value(number: &Rat) -> Value {
    Value::Frac(number.clone())
}

fn chart_point_value(point: &ChartPoint) -> Value {
    Value::List(vec![rat_value(&point.0), rat_value(&point.1)])
}

pub(crate) fn point_value(point: &Point) -> Value {
    Value::List(vec![Value::Sum(point.0.clone()), Value::Sum(point.1.clone())])
}

pub(crate) fn float_list(values: &[f64]) -> Value {
    Value::List(values.iter().map(|value| Value::Float(*value)).collect())
}

fn key_value(key: &CellKey) -> Value {
    match key {
        CellKey::Triangle(index) => Value::List(vec![str_value("t"), int(*index as u64)]),
        CellKey::Face(face, index) => Value::List(vec![str_value("f"), str_value(face), int(*index as u64)]),
        CellKey::Group(face, index) => Value::List(vec![str_value("g"), str_value(face), int(*index as u64)]),
    }
}

fn optional(value: Option<Value>) -> Value {
    value.unwrap_or(Value::None)
}

fn cell_value(cell: &ClipCell) -> Value {
    Value::List(vec![
        key_value(&cell.key),
        str_value(&cell.name),
        Value::List(cell.chart.iter().map(chart_point_value).collect()),
        float_list(&cell.bbox),
        rat_value(&cell.twice_area),
        Value::List(cell.members.iter().map(|member| int(*member as u64)).collect()),
        Value::List(cell.diagonals.iter().map(|(owner, edge)| Value::List(vec![int(*owner as u64), int(*edge as u64)])).collect()),
        optional(cell.hinge.as_ref().map(|hinge| Value::List(vec![int(hinge.triangle as u64), int(hinge.edge as u64), rat_value(&hinge.jump_square)]))),
        optional(cell.flat_square.as_ref().map(rat_value)),
        Value::List(cell.straight.iter().map(|(edge, corner)| Value::List(vec![int(*edge as u64), chart_point_value(corner)])).collect()),
        optional(cell.group.as_ref().map(key_value)),
    ])
}

fn memo_value(memo: &CellMemo) -> Value {
    Value::List(
        memo.iter()
            .map(|(key, value)| match (key, value) {
                (MemoKey::Cell(face), MemoValue::Built(Built::Cell(cell))) => Value::List(vec![int(0u8), str_value(face), Value::List(vec![int(0u8), cell_value(cell)])]),
                (MemoKey::Cell(face), MemoValue::Built(Built::Reason(reason))) => Value::List(vec![int(0u8), str_value(face), Value::List(vec![int(1u8), str_value(reason)])]),
                (MemoKey::Flat(face), MemoValue::Flat(flat)) => Value::List(vec![int(1u8), str_value(face), rat_value(flat)]),
                (MemoKey::Cell(face) | MemoKey::Flat(face), _) => Value::List(vec![int(2u8), str_value(face)]),
            })
            .collect(),
    )
}

fn memo_of(value: &Value) -> Wire<CellMemo> {
    let mut memo = CellMemo::new();
    for entry in list(value, "a cell memo")? {
        let items = list(entry, "a memo entry")?;
        match items {
            [kind, face, payload] => {
                let face = str_of(face, "a memo face")?;
                match usize_of(kind, "a memo kind")? {
                    0 => {
                        let [tag, body] = fixed::<2>(payload, "a built cell")?;
                        let built = match usize_of(tag, "a built tag")? {
                            0 => Built::Cell(Box::new(cell_of(body)?)),
                            1 => Built::Reason(reason_of(&str_of(body, "a reason")?)?),
                            _ => return Err(bad("a built tag")),
                        };
                        memo.set(MemoKey::Cell(face), MemoValue::Built(built));
                    }
                    1 => memo.set(MemoKey::Flat(face), MemoValue::Flat(rat_of(payload, "a flat square")?)),
                    _ => return Err(bad("a memo kind")),
                }
            }
            _ => return Err(bad("a memo entry")),
        }
    }
    Ok(memo)
}

fn reason_of(text: &str) -> Wire<&'static str> {
    [cells::NO_HINGE, cells::NOT_CONVEX, cells::NOT_ONE_LOOP, cells::MIXED_WINDING].into_iter().find(|known| *known == text).ok_or_else(|| bad("a cell reason"))
}

fn cell_of(value: &Value) -> Wire<ClipCell> {
    let [key, name, chart, bbox, area, members, diagonals, hinge, flat, straight, group] = fixed::<11>(value, "a cell")?;
    let bbox: Vec<f64> = list(bbox, "a box")?.iter().map(|edge| float_of(edge, "a box edge")).collect::<Wire<_>>()?;
    let pairs = |value: &Value| -> Wire<Vec<(usize, usize)>> {
        list(value, "pairs")?
            .iter()
            .map(|pair| {
                let [first, second] = fixed::<2>(pair, "a pair")?;
                Ok((usize_of(first, "a pair index")?, usize_of(second, "a pair index")?))
            })
            .collect()
    };
    Ok(ClipCell {
        key: key_of(key)?,
        name: str_of(name, "a cell name")?,
        chart: chart_of(chart)?,
        bbox: bbox.try_into().map_err(|_| bad("a box of four floats"))?,
        twice_area: rat_of(area, "a cell area")?,
        members: list(members, "members")?.iter().map(|member| usize_of(member, "a member")).collect::<Wire<_>>()?,
        diagonals: pairs(diagonals)?,
        hinge: match hinge {
            Value::None => None,
            other => {
                let [triangle, edge, jump] = fixed::<3>(other, "a hinge")?;
                Some(cells::Hinge { triangle: usize_of(triangle, "a hinge triangle")?, edge: usize_of(edge, "a hinge edge")?, jump_square: rat_of(jump, "a hinge jump")? })
            }
        },
        flat_square: match flat {
            Value::None => None,
            other => Some(rat_of(other, "a flat square")?),
        },
        straight: list(straight, "straight vertices")?
            .iter()
            .map(|entry| {
                let [edge, corner] = fixed::<2>(entry, "a straight vertex")?;
                Ok((usize_of(edge, "a straight edge")?, chart_point_of(corner)?))
            })
            .collect::<Wire<_>>()?,
        group: match group {
            Value::None => None,
            other => Some(key_of(other)?),
        },
    })
}

fn plan_value(plan: &CellPlan) -> Value {
    Value::List(vec![
        Value::List(plan.cells.iter().map(cell_value).collect()),
        Value::List(plan.unmergeable.iter().map(|(face, reason)| Value::List(vec![str_value(face), str_value(reason)])).collect()),
    ])
}

fn constants_value(constants: &Option<EdgeConstants>) -> Value {
    match constants {
        None => Value::None,
        Some(found) => Value::List(vec![
            int(found.x0),
            int(found.y0),
            int(found.dx),
            int(found.dy),
            Value::Float(found.fx0),
            Value::Float(found.fy0),
            Value::Float(found.fdx),
            Value::Float(found.fdy),
            Value::Float(found.tolerance),
        ]),
    }
}

fn constants_of(value: &Value) -> Wire<EdgeConstants> {
    let [x0, y0, dx, dy, fx0, fy0, fdx, fdy, tolerance] = fixed::<9>(value, "edge constants")?;
    let small = |value: &Value| i64::try_from(&int_of(value, "an edge integer")?).map_err(|_| bad("an edge integer in range"));
    Ok(EdgeConstants {
        x0: small(x0)?,
        y0: small(y0)?,
        dx: small(dx)?,
        dy: small(dy)?,
        fx0: float_of(fx0, "fx0")?,
        fy0: float_of(fy0, "fy0")?,
        fdx: float_of(fdx, "fdx")?,
        fdy: float_of(fdy, "fdy")?,
        tolerance: float_of(tolerance, "a tolerance")?,
    })
}

// --------------------------------------------------------------------------
// outcomes
// --------------------------------------------------------------------------

fn clip_outcome(result: ClipResult<Value>) -> Value {
    let coded = |code: u8, parts: Vec<Value>| {
        let mut entry = vec![int(code)];
        entry.extend(parts);
        Value::List(entry)
    };
    match result {
        Ok(value) => Value::List(vec![int(0u8), value]),
        Err(ClipError::Exact(error)) => outcome_value(Err(error)),
        Err(ClipError::Overflow(kind)) => coded(8, vec![int(kind as u8)]),
        Err(ClipError::ZeroDivision(text)) => coded(9, vec![str_value(text)]),
        Err(ClipError::Value(text)) => coded(10, vec![str_value(text)]),
        Err(ClipError::Refusal { outcome, detail }) => coded(11, vec![str_value(outcome), str_value(&detail)]),
        Err(ClipError::MissingKey(key)) => coded(13, vec![str_value(&key)]),
        Err(ClipError::Unsupported(text)) => coded(12, vec![str_value(&text)]),
    }
}

// --------------------------------------------------------------------------
// the seams
// --------------------------------------------------------------------------

fn pt(point: &Point) -> Pt<'_> {
    (&point.0, &point.1)
}

fn ok(value: Value) -> ClipResult<Value> {
    Ok(value)
}

fn sign_value(sign: i8) -> Value {
    int(i64::from(sign))
}

fn dispatch(code: u8, args: &[Value], ctx: &mut ExactCtx<'_>, extras: &mut Vec<Value>) -> Wire<ClipResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    let expect = |count: usize| if args.len() == count { Ok(()) } else { Err(bad("the argument count")) };
    Ok(match code {
        100 => {
            expect(3)?;
            let version = version_of(at(0)?, at(1)?);
            let keys: Vec<i64> = list(at(2)?, "sort keys")?.iter().map(|key| i64::try_from(&int_of(key, "a sort key")?).map_err(|_| bad("a sort key in range"))).collect::<Wire<_>>()?;
            version.and_then(|version| {
                let mut log: Vec<Value> = Vec::new();
                let order = pyemu::sort_by_less(version, (0..keys.len()).collect::<Vec<usize>>(), |left, right| {
                    log.push(Value::List(vec![int(*left as u64), int(*right as u64)]));
                    Ok(keys[*left] < keys[*right])
                })?;
                Ok(Value::List(vec![Value::List(order.into_iter().map(|index| int(index as u64)).collect()), Value::List(log)]))
            })
        }
        101 => {
            expect(3)?;
            let version = version_of(at(0)?, at(1)?);
            let terms: Vec<f64> = list(at(2)?, "float terms")?.iter().map(|term| float_of(term, "a float term")).collect::<Wire<_>>()?;
            version.map(|version| Value::Float(pyemu::float_sum(version, &terms)))
        }
        102 => {
            expect(4)?;
            let chart = chart_of(at(0)?)?;
            let index = usize_of(at(1)?, "an edge index")?;
            if index >= chart.len() {
                return Err(bad("an edge index in range"));
            }
            ok(Value::Sum(plane::line_value(&chart, index, sum_of(at(2)?, "x")?, sum_of(at(3)?, "y")?)))
        }
        103 => {
            expect(2)?;
            numeric::window(sum_of(at(0)?, "x")?, sum_of(at(1)?, "y")?).map(|window| float_list(&window))
        }
        104 => {
            expect(3)?;
            let triangle = triangle_of(at(0)?)?;
            let found = plane::values_in(&triangle, sum_of(at(1)?, "x")?, sum_of(at(2)?, "y")?);
            ok(Value::List(found.into_iter().map(Value::Sum).collect()))
        }
        105 => {
            expect(1)?;
            let single = Plane::new(vec![triangle_of(at(0)?)?]);
            single.stretch_square(0).map(|square| rat_value(&square))
        }
        106 => {
            expect(4)?;
            let version = version_of(at(0)?, at(1)?);
            let triangle = triangle_of(at(2)?)?;
            let values: Vec<SqrtSum> = list(at(3)?, "orientation values")?.iter().map(|value| sum_of(value, "an orientation value").cloned()).collect::<Wire<_>>()?;
            let values: [SqrtSum; 3] = values.try_into().map_err(|_| bad("three orientation values"))?;
            version.and_then(|version| lift::lift_known(version, &triangle, &values)).map(|lifted| {
                Value::List(vec![float_list(&lifted.position), str_value(&lifted.triangle), optional(lifted.normal.as_ref().map(|normal| float_list(normal)))])
            })
        }
        107 => {
            expect(1)?;
            numeric::nanometres(&rat_of(at(0)?, "a depth square")?).map(|value| ubig_value(&value))
        }
        108 => {
            expect(1)?;
            numeric::milli_cells(&rat_of(at(0)?, "a distance square")?).map(|value| ubig_value(&value))
        }
        109 => {
            expect(3)?;
            let points = [point_of(at(0)?)?, point_of(at(1)?)?, point_of(at(2)?)?];
            faces::orientation(ctx, pt(&points[0]), pt(&points[1]), pt(&points[2])).map(sign_value)
        }
        110 => {
            expect(1)?;
            let points = points_of(at(0)?)?;
            let refs: Vec<Pt> = points.iter().map(pt).collect();
            faces::shoelace_sign(ctx, &refs).map(sign_value)
        }
        111 => {
            expect(1)?;
            let points = points_of(at(0)?)?;
            let refs: Vec<Pt> = points.iter().map(pt).collect();
            ok(Value::Sum(faces::doubled_shoelace(ctx.products, &refs)))
        }
        112 => {
            expect(2)?;
            snap::within_edge_gap(ctx, sum_of(at(0)?, "a value")?, &rat_of(at(1)?, "an edge square")?).map(|(within, gap)| Value::List(vec![Value::Bool(within), rat_value(&gap)]))
        }
        113 => {
            expect(2)?;
            let column: Vec<SqrtSum> = list(at(1)?, "a column")?.iter().map(|value| sum_of(value, "a column value").cloned()).collect::<Wire<_>>()?;
            cells::hinge_depth_square(&rat_of(at(0)?, "a jump square")?, &column).map(|square| rat_value(&square))
        }
        114 => {
            expect(3)?;
            let jump = match at(0)? {
                Value::None => None,
                other => Some(rat_of(other, "a jump square")?),
            };
            let flat = match at(1)? {
                Value::None => None,
                other => Some(rat_of(other, "a flat square")?),
            };
            let columns: Vec<Vec<SqrtSum>> = list(at(2)?, "columns")?
                .iter()
                .map(|column| list(column, "a column")?.iter().map(|value| sum_of(value, "a column value").cloned()).collect::<Wire<_>>())
                .collect::<Wire<_>>()?;
            cells::chord_of(ctx, jump.as_ref(), flat.as_ref(), &columns).map(|(depth, crossings)| Value::List(vec![optional(depth.as_ref().map(rat_value)), int(crossings)]))
        }
        115 => {
            expect(2)?;
            let chart = chart_of(at(0)?)?;
            let index = usize_of(at(1)?, "an edge index")?;
            if index >= chart.len() {
                return Err(bad("an edge index in range"));
            }
            ok(constants_value(&edge::edge_constants(&chart, index)))
        }
        116 => {
            expect(3)?;
            let (sign, far) = edge::cheap_sign(&point_of(at(0)?)?, &constants_of(at(1)?)?, flag_of(at(2)?, "watch")?);
            ok(Value::List(vec![sign.map_or(Value::None, sign_value), Value::Bool(far)]))
        }
        117 => {
            expect(1)?;
            ok(match point::rational_pair(&point_of(at(0)?)?) {
                None => Value::None,
                Some((xn, xd, yn, yd)) => Value::List(vec![Value::Int(xn), ubig_value(&xd), Value::Int(yn), ubig_value(&yd)]),
            })
        }
        118 => {
            expect(3)?;
            let triangles = triangles_of(at(0)?)?;
            let split: HashSet<CellKey> = list(at(1)?, "split keys")?.iter().map(key_of).collect::<Wire<_>>()?;
            let mut memo = memo_of(at(2)?)?;
            cells::build_cells(&triangles, &split, &mut memo).map(|plan| Value::List(vec![plan_value(&plan), memo_value(&memo)]))
        }
        119 => {
            expect(2)?;
            let plane = Plane::new(triangles_of(at(0)?)?);
            let mut named: Vec<(String, Point)> = Vec::new();
            for entry in list(at(1)?, "named points")? {
                let [key, found] = fixed::<2>(entry, "a named point")?;
                named.push((str_of(key, "a point key")?, point_of(found)?));
            }
            let named_value = |entries: &[(String, Point)]| Value::List(entries.iter().map(|(key, found)| Value::List(vec![str_value(key), point_value(found)])).collect());
            snap::snap_source_vertices(ctx, &plane, &named).map(|result| {
                Value::List(vec![named_value(&result.points), named_value(&result.moved), Value::List(result.counters.iter().map(ubig_value).collect())])
            })
        }
        120 => {
            expect(1)?;
            let points = points_of(at(0)?)?;
            let refs: Vec<Pt> = points.iter().map(pt).collect();
            tessellate::triangulate_exact(ctx, &refs).map(|found| match found {
                None => Value::None,
                Some(triangles) => Value::List(triangles.iter().map(|triangle| Value::List(triangle.iter().map(|index| int(*index as u64)).collect())).collect()),
            })
        }
        121 => {
            expect(1)?;
            let points = points_of(at(0)?)?;
            let refs: Vec<Pt> = points.iter().map(pt).collect();
            tessellate::convex_quad_ring(ctx, &refs).map(|found| match found {
                None => Value::None,
                Some(ring) => Value::List(ring.iter().map(|index| int(*index as u64)).collect()),
            })
        }
        122 => {
            expect(2)?;
            let points = points_of(at(0)?)?;
            let refs: Vec<Pt> = points.iter().map(pt).collect();
            let ring: Vec<usize> = list(at(1)?, "a ring")?.iter().map(|index| usize_of(index, "a ring index")).collect::<Wire<_>>()?;
            if ring.iter().any(|index| *index >= refs.len()) {
                return Err(bad("ring indices in range"));
            }
            tessellate::has_right_turn(ctx, &refs, &ring).map(Value::Bool)
        }
        123 => {
            expect(5)?;
            let version = version_of(at(0)?, at(1)?);
            let (first, second) = (point_of(at(2)?)?, point_of(at(3)?)?);
            let nodes = points_of(at(4)?)?;
            let refs: Vec<&Point> = nodes.iter().collect();
            version.and_then(|version| order::ordered(ctx, version, &first, &second, &refs)).map(|permutation| Value::List(permutation.into_iter().map(|index| int(index as u64)).collect()))
        }
        124 => {
            expect(11)?;
            geometry_seam::clip_geometry_seam(args, ctx, extras)?
        }
        other => return Err(SeamError(format!("unknown clip seam opcode {other}"))),
    })
}

fn default_header() -> Value {
    let table = || Value::List(vec![Value::Bool(false), Value::List(Vec::new()), Value::List(Vec::new())]);
    Value::List(vec![int(0u8), Value::List(vec![table(), table(), table(), table()]), Value::None])
}

/// Runs one seam on `session` and answers the cost answer buffer.
pub fn run(session: &mut Session, request: &[u8]) -> Result<Vec<u8>, SeamError> {
    let mut reader = Reader::new(request, true);
    let parsed = reader.get_value()?;
    reader.finish()?;
    let [header, code, args] = fixed::<3>(&parsed, "a request")?;
    let code = u8::try_from(&int_of(code, "an opcode")?).map_err(|_| bad("an opcode"))?;
    let args = list(args, "the arguments")?;
    let header = if matches!(header, Value::None) { default_header() } else { header.clone() };
    let mut cost = CostRun::begin(session, &header).map_err(|error| SeamError(error.to_string()))?;
    let mut counts = SignCounts::default();
    let mut extras: Vec<Value> = Vec::new();
    let started = std::time::Instant::now();
    let result = {
        let mut ctx = ExactCtx { memory: &mut session.memory, budget: cost.budget_mut(), counts: &mut counts, products: &mut session.products };
        dispatch(code, args, &mut ctx, &mut extras)?
    };
    let elapsed = started.elapsed().as_nanos() as u64;
    let mut answer = cost.answer(session, clip_outcome(result), &counts);
    // the compute time of the seam itself (decoding the arguments is inside it, the boundary crossing is not), nanoseconds
    if let Value::List(parts) = &mut answer {
        parts.push(int(elapsed));
        parts.extend(extras);
    }
    let mut writer = Writer::new();
    writer.put_value(&answer);
    Ok(writer.into_bytes())
}
