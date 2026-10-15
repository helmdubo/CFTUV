//! Memo facts export: allocation слотов без Python geometry/constructor callbacks.
use super::output::Encoder;
use crate::pyobj;
use cftuv_density::density_attempt_memo_v1::*;
use cftuv_density::density_raw_result_v1::{DensityShellEventFactsV1, FanAuthorityFactsV1};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyString};

fn record<'py>(
    e: &Encoder<'py>,
    name: &str,
    fields: Vec<(&str, Bound<'py, PyAny>)>,
) -> PyResult<Bound<'py, PyAny>> {
    let class =
        e.py.import("cftuv_envelope.reference.density_attempt_memo_v1")?
            .getattr(name)?;
    let declared = class.getattr("__dataclass_fields__")?;
    let declared = declared.cast::<PyDict>()?;
    if declared.len() != fields.len()
        || fields
            .iter()
            .any(|(name, _)| !declared.contains(name).unwrap_or(false))
    {
        return Err(pyobj::refuse("density memo output fields changed"));
    }
    let value = pyobj::alloc(e.py, &class.unbind())?;
    for (name, field) in fields {
        pyobj::set_slot(&value, &PyString::intern(e.py, name).unbind(), &field)?;
    }
    Ok(value)
}
fn literal<'py>(e: &Encoder<'py>, value: &DensityLiteralTreeV1) -> PyResult<Bound<'py, PyAny>> {
    use DensityLiteralTreeV1 as T;
    let (tag, args) = match value {
        T::Rational(v) => {
            let (n, d) = v.parts();
            ("Q", vec![e.z(n)?, e.u(d)?])
        }
        T::Pi => ("pi", vec![]),
        T::Add(v) | T::Mul(v) => (
            if matches!(value, T::Add(_)) {
                "Add"
            } else {
                "Mul"
            },
            v.iter()
                .map(|v| literal(e, v))
                .collect::<PyResult<Vec<_>>>()?,
        ),
        T::Pow(a, b) | T::Atan2(a, b) => (
            if matches!(value, T::Pow(_, _)) {
                "Pow"
            } else {
                "atan2"
            },
            vec![literal(e, a)?, literal(e, b)?],
        ),
        T::Sin(v) => ("sin", vec![literal(e, v)?]),
        T::Cos(v) => ("cos", vec![literal(e, v)?]),
        T::Atan(v) => ("atan", vec![literal(e, v)?]),
        T::Tan(v) => ("tan", vec![literal(e, v)?]),
    };
    let mut values = vec![e.text(tag)?];
    values.extend(args);
    e.seq(values)
}
fn key<'py>(e: &Encoder<'py>, value: &DensityAttemptKeyV1) -> PyResult<Bound<'py, PyAny>> {
    // Canon carrier повторяет исходный namespace; здесь нет validation callbacks.
    let class =
        e.py.import("cftuv_envelope.reference.density_oracle_v1")?
            .getattr("DensityOracleCanonV1")?;
    let canon = pyobj::alloc(e.py, &class.unbind())?;
    for (name, value) in [
        ("construction", &value.canon_ids.construction),
        ("shell", &value.canon_ids.shell),
        ("angle", &value.canon_ids.angle),
    ] {
        pyobj::set_slot(
            &canon,
            &PyString::intern(e.py, name).unbind(),
            &e.text(value)?,
        )?;
    }
    record(
        e,
        "DensityAttemptKeyV1",
        vec![
            ("canon_ids", canon),
            (
                "gram",
                e.seq(
                    value
                        .gram
                        .iter()
                        .map(|v| e.q(v))
                        .collect::<PyResult<Vec<_>>>()?,
                )?,
            ),
            (
                "inverse_gram",
                e.seq(
                    value
                        .inverse_gram
                        .iter()
                        .map(|v| e.q(v))
                        .collect::<PyResult<Vec<_>>>()?,
                )?,
            ),
            (
                "owner_orientation_sign",
                e.i(value.owner_orientation_sign as i64)?,
            ),
            (
                "ideal_primal_literals",
                e.seq(
                    value
                        .ideal_primal_literals
                        .iter()
                        .map(|v| e.seq(vec![literal(e, &v[0])?, literal(e, &v[1])?]))
                        .collect::<PyResult<Vec<_>>>()?,
                )?,
            ),
            (
                "orientation",
                e.text(if value.orientation == 1 {
                    "CCW_IN_OWNER_PATCH_ORIENTATION"
                } else {
                    "CW_IN_OWNER_PATCH_ORIENTATION"
                })?,
            ),
            ("q", e.n(value.q)?),
            (
                "binding_reasons",
                e.seq(
                    value
                        .binding_reasons
                        .iter()
                        .map(|v| e.optional_text(v))
                        .collect::<PyResult<Vec<_>>>()?,
                )?,
            ),
            ("adaptive_only", e.flag(value.adaptive_only)?),
            (
                "window_law",
                e.text(
                    if value.window_law == cftuv_density::geometry::WindowLaw::NarrowBand {
                        "NARROW_ROTATION_BAND"
                    } else {
                        "VORONOI_OF_NEIGHBOUR_MIDPOINTS"
                    },
                )?,
            ),
        ],
    )
}
pub fn snapshot<'py>(
    e: &Encoder<'py>,
    value: &DensityAttemptMemoSnapshotV1,
) -> PyResult<Bound<'py, PyAny>> {
    let entries = value
        .entries
        .iter()
        .map(|v| {
            let certificates = v
                .binding_certificates
                .iter()
                .map(|v| match v {
                    Some(v) => e.certificate(
                        &cftuv_density::density_result_v1::certificate_facts(v, None).map_err(
                            |why| pyobj::refuse(format!("memo certificate projection: {why:?}")),
                        )?,
                    ),
                    None => e.none(),
                })
                .collect::<PyResult<Vec<_>>>()?;
            let authority = match &v.authority {
                Some(a) => e.authority(&FanAuthorityFactsV1::Adaptive(
                    cftuv_density::density_result_v1::authority_facts(
                        a,
                        &v.key.binding_reasons,
                        v.key.window_law,
                    )
                    .map_err(|why| pyobj::refuse(format!("memo authority projection: {why:?}")))?,
                ))?,
                None => e.none()?,
            };
            // Closed facts имеют тот же sorted/deduplicated порядок, что raw result.
            let shell_facts = v
                .shell_events
                .iter()
                .map(|(wire, bundle)| (wire, bundle))
                .collect::<std::collections::BTreeMap<_, _>>();
            let shells = shell_facts
                .into_iter()
                .map(|(wire, b)| {
                    e.shell(&DensityShellEventFactsV1 {
                        value_wire: wire.clone(),
                        codes: b.codes.iter().map(|v| v.code().into()).collect(),
                        rational_numerator: b.rational.as_ref().map(|r| r.numerator.clone()),
                        rational_denominator: b.rational.as_ref().map(|r| r.denominator.clone()),
                        rational_grid_units: b.rational.as_ref().and_then(|r| r.grid_units.clone()),
                        legacy_projected_min_width_units: b
                            .legacy_projected_min_width_units
                            .map(cftuv_density::UBig::from),
                    })
                })
                .collect::<PyResult<Vec<_>>>()?;
            record(
                e,
                "DensityAttemptReceiptV1",
                vec![
                    ("key", key(e, &v.key)?),
                    ("binding_certificates", e.seq(certificates)?),
                    ("authority", authority),
                    ("shell_events", e.seq(shells)?),
                ],
            )
        })
        .collect::<PyResult<Vec<_>>>()?;
    record(
        e,
        "DensityAttemptMemoSnapshotV1",
        vec![("entries", e.seq(entries)?)],
    )
}
pub fn report<'py>(
    e: &Encoder<'py>,
    value: &DensityMemoBatchResultV1,
) -> PyResult<Bound<'py, PyAny>> {
    e.seq(vec![e.report(&value.report)?, snapshot(e, &value.delta)?])
}
