//! The side cuts of the drag certificate (SIDE_CUT_NATIVE_V1): `parametric._side_inputs` (the points, contours and facts before the cut moved
//! along their slopes by a shift of the lattice width) and `parametric._clip_slope_values` (the slopes of the clip nodes and of the facts the clip
//! added, by the exact second differences of three cuts). No budget and no memory: `SqrtSumV1` sums, differences and rational scalings only.
//!
//! The answers are the oracle's containers in the oracle's order. A point without a slope is the SAME object (`_shifted`); a fact without a slope
//! a new pair of the same two objects. An input the port does not carry (a class, a missing key the oracle would raise `KeyError` on) is declined
//! (`None`): the dispatcher runs the oracle, which answers or raises as it does.

use std::collections::HashMap;

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyTuple};

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::SqrtSum;

use crate::materialize_hot::{carried, pair_of, rat_of, unsupported, Fail, Host, Read};
use crate::pyobj::{sqrt_sum_to_py, Pool};

/// `point if slope is None else (point[0] + slope[0].scaled(shift), point[1] + slope[1].scaled(shift))`.
fn shifted<'py>(py: Python<'py>, pool: &Pool, point: &Bound<'py, PyAny>, slope: &(SqrtSum, SqrtSum), shift: &Rat, what: &str) -> Read<Bound<'py, PyAny>> {
    let (x, y) = pair_of(pool, point, what)?;
    let moved = [sqrt_sum_to_py(py, pool, &x.add(&slope.0.scaled(shift)))?, sqrt_sum_to_py(py, pool, &y.add(&slope.1.scaled(shift)))?];
    Ok(PyTuple::new(py, moved)?.into_any())
}

/// `_second_difference_slope(low, middle, high, delta, name)`: `Some(slope)`, or `None` for the refusal (the second difference is not exactly zero).
fn second_difference(low: &SqrtSum, middle: &SqrtSum, high: &SqrtSum, inverse: &Rat) -> Option<SqrtSum> {
    let up = high.sub(middle);
    let down = middle.sub(low);
    if up.canonical_form() != down.canonical_form() {
        return None;
    }
    Some(up.scaled(inverse))
}

/// `d[key]` of an exact dict, declined when it is missing (the oracle raises `KeyError`).
fn item<'py>(dict: &Bound<'py, PyDict>, key: &Bound<'py, PyAny>, what: &str) -> Read<Bound<'py, PyAny>> {
    match dict.get_item(key)? {
        Some(found) => Ok(found),
        None => unsupported(format!("a key missing from {what} (KeyError in the oracle)")),
    }
}

fn dict_of<'a, 'py>(object: &'a Bound<'py, PyAny>, what: &str) -> Read<&'a Bound<'py, PyDict>> {
    match object.cast_exact::<PyDict>() {
        Ok(dict) => Ok(dict),
        Err(_) => unsupported(format!("{what} is not a dict")),
    }
}

/// An ordered dict of the answer (`point_slopes[key] = axes`: an overwrite keeps the first position).
struct Ordered<'py> {
    keys: Vec<Bound<'py, PyAny>>,
    values: Vec<Bound<'py, PyAny>>,
}

/// The status of `clip_slope_values`: 0 the slopes, 1 `PARAM_CLIP_NODE`, 2 `PARAM_CLIP_FACT` (the refusal of the oracle, by its name in the shim).
pub type Slopes<'py> = (u8, Option<Bound<'py, PyAny>>, Option<Bound<'py, PyAny>>);

impl Host {
    /// `parametric._side_inputs(points_before, cycles_before, facts_before, key_slope, fact_slope, shift)`: `Some((points, cycles, facts))` or `None` (declined).
    #[allow(clippy::too_many_arguments)]
    pub fn side_inputs<'py>(
        &self,
        py: Python<'py>,
        points_before: &Bound<'py, PyAny>,
        cycles_before: &Bound<'py, PyAny>,
        facts_before: &Bound<'py, PyAny>,
        key_slope: &Bound<'py, PyAny>,
        fact_slope: &Bound<'py, PyAny>,
        shift: &Bound<'py, PyAny>,
    ) -> PyResult<Option<Bound<'py, PyTuple>>> {
        let pool = &self.classes()?.pool;
        let found = (|| -> Read<Bound<'py, PyTuple>> {
            let shift = rat_of(pool, shift, "the shift")?;
            let (points_before, facts_before) = (dict_of(points_before, "points_before")?, dict_of(facts_before, "facts_before")?);
            let (key_slope, fact_slope) = (dict_of(key_slope, "key_slope")?, dict_of(fact_slope, "fact_slope")?);
            let points = PyDict::new(py);
            for (key, point) in points_before.iter() {
                let moved = match key_slope.get_item(&key)? {
                    Some(slope) if !slope.is_none() => shifted(py, pool, &point, &pair_of(pool, &slope, "a key slope")?, &shift, "a point before the cut")?,
                    _ => point,
                };
                points.set_item(key, moved)?;
            }
            let Ok(cycles_before) = cycles_before.cast_exact::<PyList>() else {
                return unsupported("cycles_before is not a list");
            };
            let mut cycles = Vec::with_capacity(cycles_before.len());
            for cycle in cycles_before.iter() {
                let mut entries = Vec::new();
                for entry in cycle.try_iter()? {
                    let entry = entry?;
                    let Ok(pair) = entry.cast_exact::<PyTuple>() else {
                        return unsupported("a contour entry is not a (key, point) tuple");
                    };
                    if pair.len() != 2 {
                        return unsupported("a contour entry is not a (key, point) pair");
                    }
                    let key = pair.get_item(0)?;
                    let point = item(&points, &key, "the points")?;
                    entries.push(PyTuple::new(py, [key, point])?);
                }
                cycles.push(PyList::new(py, entries)?);
            }
            let facts = PyDict::new(py);
            for (slot, value) in facts_before.iter() {
                let Ok(fact) = value.cast_exact::<PyTuple>() else {
                    return unsupported("a fact is not an (s, r) tuple");
                };
                if fact.len() != 2 {
                    return unsupported("a fact is not an (s, r) pair");
                }
                // `_nonzero(fact_slope.get(slot))`: no slope, or a zero pair, leaves the fact
                let slope = match fact_slope.get_item(&slot)? {
                    Some(slope) if !slope.is_none() => {
                        let pair = pair_of(pool, &slope, "a fact slope")?;
                        if pair.0.is_zero() && pair.1.is_zero() {
                            None
                        } else {
                            Some(pair)
                        }
                    }
                    _ => None,
                };
                let moved = match slope {
                    None => PyTuple::new(py, [fact.get_item(0)?, fact.get_item(1)?])?,
                    Some(slope) => {
                        let (s, r) = pair_of(pool, fact.as_any(), "a fact")?;
                        PyTuple::new(py, [sqrt_sum_to_py(py, pool, &s.add(&slope.0.scaled(&shift)))?, sqrt_sum_to_py(py, pool, &r.add(&slope.1.scaled(&shift)))?])?
                    }
                };
                facts.set_item(slot, moved)?;
            }
            Ok(PyTuple::new(py, [points.into_any(), PyList::new(py, cycles)?.into_any(), facts.into_any()])?)
        })();
        match found {
            Ok(found) => Ok(Some(found)),
            Err(Fail::Unsupported(_)) => Ok(None),
            Err(Fail::Py(error)) => Err(error),
        }
    }

    /// `parametric._clip_slope_values(cut, up, down, facts_before, facts_after, facts_up, facts_down, delta)` on the `points` and `snapped` of the three cuts:
    /// `Some((status, point_slopes, fact_slopes))` or `None` (declined).
    #[allow(clippy::too_many_arguments)]
    pub fn clip_slope_values<'py>(
        &self,
        py: Python<'py>,
        sources: [&Bound<'py, PyAny>; 6],
        facts_before: &Bound<'py, PyAny>,
        facts_after: &Bound<'py, PyAny>,
        facts_up: &Bound<'py, PyAny>,
        facts_down: &Bound<'py, PyAny>,
        delta: &Bound<'py, PyAny>,
    ) -> PyResult<Option<Slopes<'py>>> {
        let pool = &self.classes()?.pool;
        let found = (|| -> Read<Slopes<'py>> {
            let delta = rat_of(pool, delta, "delta")?;
            let inverse = Rat::one().div(&delta).map_err(|_| Fail::Unsupported("delta is zero (ZeroDivisionError in the oracle)".to_string()))?;
            let mut ordered = Ordered { keys: Vec::new(), values: Vec::new() };
            let mut index: HashMap<String, usize, FxBuild> = HashMap::default();
            let [cut_points, up_points, down_points, cut_snapped, up_snapped, down_snapped] = sources;
            for (source, source_up, source_down) in [(cut_points, up_points, down_points), (cut_snapped, up_snapped, down_snapped)] {
                let (source, source_up, source_down) = (dict_of(source, "the clip points")?, dict_of(source_up, "the clip points")?, dict_of(source_down, "the clip points")?);
                for (key, point) in source.iter() {
                    let low = pair_of(pool, &item(source_down, &key, "a side cut")?, "a clip point")?;
                    let middle = pair_of(pool, &point, "a clip point")?;
                    let high = pair_of(pool, &item(source_up, &key, "a side cut")?, "a clip point")?;
                    let Some(x) = second_difference(&low.0, &middle.0, &high.0, &inverse) else {
                        return Ok((1, None, None));
                    };
                    let Some(y) = second_difference(&low.1, &middle.1, &high.1, &inverse) else {
                        return Ok((1, None, None));
                    };
                    if x.is_zero() && y.is_zero() {
                        continue;
                    }
                    let axes = PyTuple::new(py, [sqrt_sum_to_py(py, pool, &x)?, sqrt_sum_to_py(py, pool, &y)?])?.into_any();
                    let name: String = carried(key.extract::<String>(), "a clip node key")?;
                    match index.get(&name) {
                        Some(at) => ordered.values[*at] = axes,
                        None => {
                            index.insert(name, ordered.keys.len());
                            ordered.keys.push(key);
                            ordered.values.push(axes);
                        }
                    }
                }
            }
            let point_slopes = PyDict::new(py);
            for (key, value) in ordered.keys.iter().zip(&ordered.values) {
                point_slopes.set_item(key, value)?;
            }
            let (facts_before, facts_after) = (dict_of(facts_before, "facts_before")?, dict_of(facts_after, "facts_after")?);
            let (facts_up, facts_down) = (dict_of(facts_up, "facts_up")?, dict_of(facts_down, "facts_down")?);
            let fact_slopes = PyDict::new(py);
            for (slot, value) in facts_after.iter() {
                // `for slot, (s, r) in facts_after.items()` unpacks before it skips: the shape is checked first, the values only when they are read
                if !value.cast_exact::<PyTuple>().is_ok_and(|pair| pair.len() == 2) {
                    return unsupported("a fact is not an (s, r) pair");
                }
                if facts_before.contains(&slot)? {
                    continue;
                }
                let middle = pair_of(pool, &value, "a fact")?;
                let high = pair_of(pool, &item(facts_up, &slot, "the facts of a side cut")?, "a fact")?;
                let low = pair_of(pool, &item(facts_down, &slot, "the facts of a side cut")?, "a fact")?;
                let Some(s) = second_difference(&low.0, &middle.0, &high.0, &inverse) else {
                    return Ok((2, None, None));
                };
                let Some(r) = second_difference(&low.1, &middle.1, &high.1, &inverse) else {
                    return Ok((2, None, None));
                };
                fact_slopes.set_item(slot, PyTuple::new(py, [sqrt_sum_to_py(py, pool, &s)?, sqrt_sum_to_py(py, pool, &r)?])?)?;
            }
            Ok((0, Some(point_slopes.into_any()), Some(fact_slopes.into_any())))
        })();
        match found {
            Ok(found) => Ok(Some(found)),
            Err(Fail::Unsupported(_)) => Ok(None),
            Err(Fail::Py(error)) => Err(error),
        }
    }
}
