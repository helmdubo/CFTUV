//! Закрытый memo transport: literal ключ никогда не импортируется в arithmetic arena.
use super::input::{self, Decoder};
use cftuv_density::atlas_plan::*;
use cftuv_density::density_attempt_memo_v1::*;
use cftuv_density::density_raw_types_v1::DensityOracleCanonV1;
use cftuv_density::density_shell_v2::{OutcomeBundle, OutcomeCode, RationalMetadata};
use cftuv_density::fan_compile::{
    AuthorityWindowBounds, PreparedAuthorityWindow, PreparedFanAuthority,
};
use cftuv_density::geometry::WindowLaw;
use cftuv_density::legacy_binding::{LegacyCertificate, ProvenPredicates};
use cftuv_density::{IBig, Rational, UBig};
use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use std::collections::HashSet;

fn malformed(message: &str) -> PyErr {
    PyValueError::new_err(format!("density_compute_with_memo: {message}"))
}
fn memo_class(value: &Bound<'_, PyAny>, name: &str) -> PyResult<()> {
    let class = value
        .py()
        .import("cftuv_envelope.reference.density_attempt_memo_v1")?
        .getattr(name)?;
    if !value.get_type().is(&class) {
        return Err(PyTypeError::new_err(format!(
            "density_compute_with_memo: exact {name} required"
        )));
    }
    Ok(())
}
fn sign(value: &Bound<'_, PyAny>) -> PyResult<i8> {
    let value: i8 = input::integer(value)?
        .try_into()
        .map_err(|_| malformed("sign range"))?;
    if !matches!(value, -1 | 1) {
        return Err(malformed("orientation must be -1 or 1"));
    }
    Ok(value)
}
fn law(value: &Bound<'_, PyAny>) -> PyResult<WindowLaw> {
    match input::text(value)?.as_str() {
        "NARROW_ROTATION_BAND" => Ok(WindowLaw::NarrowBand),
        "VORONOI_OF_NEIGHBOUR_MIDPOINTS" => Ok(WindowLaw::Voronoi),
        _ => Err(malformed("unknown window law")),
    }
}
fn triple(decoder: &Decoder<'_>, value: &Bound<'_, PyAny>) -> PyResult<[Rational; 3]> {
    input::tuple(value)?
        .iter()
        .map(|v| decoder.rational(&v))
        .collect::<PyResult<Vec<_>>>()?
        .try_into()
        .map_err(|_| malformed("metric requires three exact Fractions"))
}
fn reasons(value: &Bound<'_, PyAny>) -> PyResult<Vec<Option<String>>> {
    input::tuple(value)?
        .iter()
        .map(|v| {
            let value = input::optional_text(&v)?;
            if value.as_deref().is_some_and(|v| {
                !matches!(
                    v,
                    "SOURCE_DIRECTION_IRRATIONAL" | "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL"
                )
            }) {
                return Err(malformed("unknown binding reason"));
            }
            Ok(value)
        })
        .collect()
}
fn literal(value: &Bound<'_, PyAny>, depth: usize) -> PyResult<DensityLiteralTreeV1> {
    if depth > 512 {
        return Err(malformed("literal tree depth limit"));
    }
    let row = input::tuple(value)?;
    if row.is_empty() {
        return Err(malformed("empty literal tag"));
    }
    let tag = input::text(&row.get_item(0)?)?;
    let arity = row.len() - 1;
    let child = |i| literal(&row.get_item(i)?, depth + 1).map(Box::new);
    Ok(match (tag.as_str(), arity) {
        ("Q", 2) => DensityLiteralTreeV1::Rational(
            Rational::new(
                input::integer(&row.get_item(1)?)?,
                input::unsigned(&row.get_item(2)?)?,
            )
            .map_err(|_| malformed("literal rational denominator"))?,
        ),
        ("pi", 0) => DensityLiteralTreeV1::Pi,
        ("Add" | "Mul", n) if n >= 2 => {
            let values = (1..row.len())
                .map(|i| literal(&row.get_item(i)?, depth + 1))
                .collect::<PyResult<Vec<_>>>()?;
            if tag == "Add" {
                DensityLiteralTreeV1::Add(values)
            } else {
                DensityLiteralTreeV1::Mul(values)
            }
        }
        ("Pow", 2) => DensityLiteralTreeV1::Pow(child(1)?, child(2)?),
        ("sin", 1) => DensityLiteralTreeV1::Sin(child(1)?),
        ("cos", 1) => DensityLiteralTreeV1::Cos(child(1)?),
        ("atan", 1) => DensityLiteralTreeV1::Atan(child(1)?),
        ("atan2", 2) => DensityLiteralTreeV1::Atan2(child(1)?, child(2)?),
        ("tan", 1) => DensityLiteralTreeV1::Tan(child(1)?),
        _ => return Err(malformed("unsupported literal tag or arity")),
    })
}
fn key(decoder: &Decoder<'_>, value: &Bound<'_, PyAny>) -> PyResult<DensityAttemptKeyV1> {
    memo_class(value, "DensityAttemptKeyV1")?;
    let canon = value.getattr("canon_ids")?;
    decoder.class(&canon, "DensityOracleCanonV1")?;
    let orientation = match input::text(&value.getattr("orientation")?)?.as_str() {
        "CCW_IN_OWNER_PATCH_ORIENTATION" => 1,
        "CW_IN_OWNER_PATCH_ORIENTATION" => -1,
        _ => return Err(malformed("unknown turn orientation")),
    };
    let forest = input::tuple(&value.getattr("ideal_primal_literals")?)?
        .iter()
        .map(|ray| {
            let pair = input::tuple(&ray)?;
            if pair.len() != 2 {
                return Err(malformed("primal literal ray requires two coordinates"));
            }
            Ok([
                literal(&pair.get_item(0)?, 1)?,
                literal(&pair.get_item(1)?, 1)?,
            ])
        })
        .collect::<PyResult<Vec<_>>>()?;
    let key = DensityAttemptKeyV1 {
        canon_ids: DensityOracleCanonV1 {
            construction: input::text(&canon.getattr("construction")?)?,
            shell: input::text(&canon.getattr("shell")?)?,
            angle: input::text(&canon.getattr("angle")?)?,
        },
        gram: triple(decoder, &value.getattr("gram")?)?,
        inverse_gram: triple(decoder, &value.getattr("inverse_gram")?)?,
        owner_orientation_sign: sign(&value.getattr("owner_orientation_sign")?)?,
        ideal_primal_literals: forest,
        orientation,
        q: input::size(&value.getattr("q")?)?,
        binding_reasons: reasons(&value.getattr("binding_reasons")?)?,
        adaptive_only: input::boolean(&value.getattr("adaptive_only")?)?,
        window_law: law(&value.getattr("window_law")?)?,
    };
    if !(2..=6).contains(&key.q) || key.ideal_primal_literals.len() != key.binding_reasons.len() + 2
    {
        return Err(malformed("attempt cardinality/q mismatch"));
    }
    Ok(key)
}
fn decimal(decoder: &Decoder<'_>, value: &Bound<'_, PyAny>) -> PyResult<(Rational, i64)> {
    decoder.class(value, "DecimalValueFactsV1")?;
    let coefficient = input::integer(&value.getattr("coefficient")?)?;
    let exponent: i64 = input::integer(&value.getattr("exponent")?)?
        .try_into()
        .map_err(|_| malformed("decimal exponent range"))?;
    if exponent.unsigned_abs() > 10000 {
        return Err(malformed("decimal exponent profile"));
    }
    let power: UBig = format!("1{}", "0".repeat(exponent.unsigned_abs() as usize))
        .parse()
        .unwrap();
    let value = if exponent < 0 {
        Rational::new(coefficient, power).unwrap()
    } else {
        Rational::integer(coefficient * IBig::from(power))
    };
    Ok((value, exponent))
}
fn interval(decoder: &Decoder<'_>, value: &Bound<'_, PyAny>) -> PyResult<DecimalEnvelope> {
    decoder.class(value, "DecimalIntervalFactsV1")?;
    let kind = |name| match input::text(&value.getattr(name)?)?.as_str() {
        "OPEN" => Ok(EndpointKind::Open),
        "CLOSED" => Ok(EndpointKind::Closed),
        _ => Err(malformed("endpoint kind")),
    };
    let (lower, le) = decimal(decoder, &value.getattr("lower")?)?;
    let (upper, ue) = decimal(decoder, &value.getattr("upper")?)?;
    let (error, ee) = decimal(decoder, &value.getattr("absolute_error_bound")?)?;
    Ok(DecimalEnvelope {
        lower,
        upper,
        lower_kind: kind("lower_kind")?,
        upper_kind: kind("upper_kind")?,
        absolute_error_bound: error,
        quantum: Some(DecimalQuantum {
            lower: le,
            upper: ue,
            absolute_error_bound: ee,
        }),
    })
}
fn vector(value: &Bound<'_, PyAny>) -> PyResult<cftuv_density::farey::PrimitiveVector> {
    let pair = input::tuple(value)?;
    if pair.len() != 2 {
        return Err(malformed("bound vector arity"));
    }
    Ok(cftuv_density::farey::PrimitiveVector(
        input::integer(&pair.get_item(0)?)?,
        input::integer(&pair.get_item(1)?)?,
    ))
}
fn certificates(
    decoder: &Decoder<'_>,
    value: &Bound<'_, PyAny>,
) -> PyResult<Vec<Option<LegacyCertificate>>> {
    input::tuple(value)?
        .iter()
        .map(|v| {
            if v.is_none() {
                return Ok(None);
            }
            decoder.class(&v, "BindingCertificateFactsV1")?;
            let predicates = input::texts(&v.getattr("proven_predicates")?)?;
            if predicates
                != [
                    "BINDING_INSIDE_OWN_ORDINAL_WINDOW",
                    "BINDING_MONOTONE",
                    "BINDING_SUBTURN_LE_DELTA_MAX",
                ]
                || !v.getattr("binding_reason")?.is_none()
            {
                return Err(malformed("receipt certificate predicates/reason"));
            }
            let width = decimal(decoder, &v.getattr("certified_window_width_lower_bound")?)?;
            let certificate = LegacyCertificate {
                bound_primitive_integer_vector: vector(
                    &v.getattr("bound_primitive_integer_vector")?,
                )?,
                ideal_window_lower_slope_envelope: interval(
                    decoder,
                    &v.getattr("ideal_window_lower_slope_envelope")?,
                )?,
                ideal_window_upper_slope_envelope: interval(
                    decoder,
                    &v.getattr("ideal_window_upper_slope_envelope")?,
                )?,
                certified_window_width_lower_bound: width.0,
                proven_predicates: ProvenPredicates::Canonical,
            };
            let projected = cftuv_density::density_result_v1::certificate_facts(&certificate, None)
                .map_err(|_| malformed("certificate projection mismatch"))?;
            if projected.certified_window_width_lower_bound.exponent != width.1 {
                return Err(malformed("certificate width quantum mismatch"));
            }
            Ok(Some(certificate))
        })
        .collect()
}
fn bounds(decoder: &Decoder<'_>, value: &Bound<'_, PyAny>) -> PyResult<AuthorityWindowBounds> {
    let q = |name| decoder.rational(&value.getattr(name)?);
    Ok(AuthorityWindowBounds {
        ordinal: input::size(&value.getattr("ordinal")?)?,
        termination_lower_slope: q("termination_lower_slope")?,
        termination_upper_slope: q("termination_upper_slope")?,
        certified_termination_width: q("certified_termination_width")?,
        admissible: [
            q("admissible_lower_outward")?,
            q("admissible_lower_inward")?,
            q("admissible_upper_inward")?,
            q("admissible_upper_outward")?,
        ],
    })
}
fn piece(decoder: &Decoder<'_>, value: &Bound<'_, PyAny>) -> PyResult<ChartPiece> {
    decoder.class(value, "ChartPieceFactsV1")?;
    let flag = |name| input::boolean(&value.getattr(name)?).map(BoolField::Bool);
    let pole = match input::text(&value.getattr("pole_ownership")?)?.as_str() {
        "NONE" => PoleOwnership::None,
        "X_ZERO" => PoleOwnership::XZero,
        "Y_ZERO" => PoleOwnership::YZero,
        _ => return Err(malformed("pole ownership")),
    };
    Ok(ChartPiece {
        is_piece_v1: true,
        piece_index: NumericField::Int(IBig::from(input::size(&value.getattr("piece_index")?)?)),
        use_x_denominator: flag("use_x_denominator")?,
        denominator_sign: NumericField::Int(IBig::from(sign(&value.getattr("denominator_sign")?)?)),
        lower_slope_envelope: interval(decoder, &value.getattr("lower_slope_envelope")?)?,
        upper_slope_envelope: interval(decoder, &value.getattr("upper_slope_envelope")?)?,
        lower_endpoint_included: flag("lower_endpoint_included")?,
        upper_endpoint_included: flag("upper_endpoint_included")?,
        slope_increases_in_ordinal_order: flag("slope_increases_in_ordinal_order")?,
        pole_ownership: PoleField::Enum(pole),
    })
}
fn window(decoder: &Decoder<'_>, value: &Bound<'_, PyAny>) -> PyResult<PreparedAuthorityWindow> {
    let local = value
        .py()
        .import("cftuv_envelope.reference.density_oracle_v1")?
        .getattr("LocalWindowFactsV1")?;
    if value.get_type().is(&local) {
        Ok(PreparedAuthorityWindow::SingleChart {
            bounds: bounds(decoder, value)?,
            record: cftuv_density::fan_prepare::PrepareRecord {
                use_x_denominator: input::boolean(&value.getattr("use_x_denominator")?)?,
                denominator_sign: sign(&value.getattr("denominator_sign")?)?,
                lower_slope_envelope: interval(
                    decoder,
                    &value.getattr("full_lower_slope_envelope")?,
                )?,
                upper_slope_envelope: interval(
                    decoder,
                    &value.getattr("full_upper_slope_envelope")?,
                )?,
            },
        })
    } else {
        decoder.class(value, "AtlasWindowFactsV1")?;
        Ok(PreparedAuthorityWindow::Atlas {
            bounds: bounds(decoder, value)?,
            pieces: input::tuple(&value.getattr("pieces")?)?
                .iter()
                .map(|v| piece(decoder, &v))
                .collect::<PyResult<Vec<_>>>()?,
            termination_piece_index: input::size(&value.getattr("termination_piece_index")?)?,
        })
    }
}
fn authority(
    decoder: &Decoder<'_>,
    value: &Bound<'_, PyAny>,
    key: &DensityAttemptKeyV1,
) -> PyResult<Option<PreparedFanAuthority>> {
    if value.is_none() {
        return Ok(None);
    }
    decoder.class(value, "AdaptiveFanAuthorityFactsV1")?;
    if !value.getattr("authority_id")?.is_none() {
        return Err(malformed(
            "receipt authority must have no per-spec identity",
        ));
    }
    if reasons(&value.getattr("binding_reasons")?)? != key.binding_reasons {
        return Err(malformed("authority reasons mismatch"));
    }
    let witness = value.getattr("previous_height_witness")?;
    decoder.class(&witness, "FareyHeightWitnessFactsV1")?;
    let exhaustive = input::unsigned(&value.getattr("exhaustive_previous_height")?)?;
    if input::unsigned(&witness.getattr("first_height")?)? != UBig::ONE
        || input::unsigned(&witness.getattr("last_height")?)? != exhaustive
    {
        return Err(malformed("authority witness mismatch"));
    }
    let authority = PreparedFanAuthority {
        max_subturn_q: input::size(&value.getattr("max_subturn_q")?)?,
        minimal_common_height: input::unsigned(&value.getattr("minimal_common_height")?)?,
        exhaustive_previous_height: exhaustive,
        termination_height_upper_bound: input::unsigned(
            &value.getattr("termination_height_upper_bound")?,
        )?,
        bound_primitive_integer_vectors: input::tuple(
            &value.getattr("bound_primitive_integer_vectors")?,
        )?
        .iter()
        .map(|v| vector(&v))
        .collect::<PyResult<Vec<_>>>()?,
        ordinal_windows: input::tuple(&value.getattr("ordinal_windows")?)?
            .iter()
            .map(|v| window(decoder, &v))
            .collect::<PyResult<Vec<_>>>()?,
        previous_counts: input::tuple(&witness.getattr("primitive_candidate_counts")?)?
            .iter()
            .map(|v| input::size(&v))
            .collect::<PyResult<Vec<_>>>()?,
        // Текущие counts не входят в facts; projection test доказывает их ненаблюдаемость.
        counts: vec![],
    };
    let projected = cftuv_density::density_result_v1::authority_facts(
        &authority,
        &key.binding_reasons,
        key.window_law,
    )
    .map_err(|_| malformed("authority projection mismatch"))?;
    if authority.max_subturn_q != key.q
        || input::texts(&value.getattr("proven_predicates")?)? != projected.proven_predicates
    {
        return Err(malformed("authority predicates/q mismatch"));
    }
    Ok(Some(authority))
}
fn shells(
    decoder: &Decoder<'_>,
    value: &Bound<'_, PyAny>,
) -> PyResult<Vec<(Vec<IBig>, OutcomeBundle)>> {
    input::tuple(value)?
        .iter()
        .map(|v| {
            decoder.class(&v, "DensityShellEventFactsV1")?;
            let wire = input::tuple(&v.getattr("value_wire")?)?
                .iter()
                .map(|v| input::integer(&v))
                .collect::<PyResult<Vec<_>>>()?;
            let codes = input::texts(&v.getattr("codes")?)?
                .iter()
                .map(|code| match code.as_str() {
                    "DENSITY_DECIMAL_SHELL_V2_APPLIED" => Ok(OutcomeCode::Applied),
                    "DENSITY_DECIMAL_GRID_POINT_V2" => Ok(OutcomeCode::GridPoint),
                    "DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2" => {
                        Ok(OutcomeCode::NonDyadicGridStraw)
                    }
                    _ => Err(malformed("shell code")),
                })
                .collect::<PyResult<Vec<_>>>()?;
            let n = v.getattr("rational_numerator")?;
            let d = v.getattr("rational_denominator")?;
            let grid = v.getattr("rational_grid_units")?;
            let rational = if n.is_none() {
                if !d.is_none() || !grid.is_none() {
                    return Err(malformed("shell rational metadata cardinality"));
                }
                None
            } else {
                let denominator = input::unsigned(&d)?;
                if denominator == UBig::ZERO {
                    return Err(malformed("shell denominator"));
                }
                Some(RationalMetadata {
                    numerator: input::integer(&n)?,
                    denominator,
                    grid_units: if grid.is_none() {
                        None
                    } else {
                        Some(input::integer(&grid)?)
                    },
                })
            };
            let straw = v.getattr("legacy_projected_min_width_units")?;
            let straw = if straw.is_none() {
                None
            } else {
                Some(
                    input::unsigned(&straw)?
                        .try_into()
                        .map_err(|_| malformed("shell straw profile"))?,
                )
            };
            Ok((
                wire,
                OutcomeBundle {
                    codes,
                    rational,
                    legacy_projected_min_width_units: straw,
                },
            ))
        })
        .collect()
}
pub fn snapshot(
    decoder: &Decoder<'_>,
    value: &Bound<'_, PyAny>,
    canon: &DensityOracleCanonV1,
) -> PyResult<DensityAttemptMemoSnapshotV1> {
    memo_class(value, "DensityAttemptMemoSnapshotV1")?;
    let mut seen = HashSet::new();
    let entries = input::tuple(&value.getattr("entries")?)?
        .iter()
        .map(|v| {
            memo_class(&v, "DensityAttemptReceiptV1")?;
            let key = key(decoder, &v.getattr("key")?)?;
            if &key.canon_ids != canon {
                return Err(malformed("memo canon IDs mismatch"));
            }
            if !seen.insert(key.clone()) {
                return Err(malformed("duplicate memo attempt keys"));
            }
            let certificates = certificates(decoder, &v.getattr("binding_certificates")?)?;
            if certificates.len() != key.binding_reasons.len() {
                return Err(malformed("certificate cardinality"));
            }
            Ok(DensityAttemptReceiptV1 {
                authority: authority(decoder, &v.getattr("authority")?, &key)?,
                shell_events: shells(decoder, &v.getattr("shell_events")?)?,
                key,
                binding_certificates: certificates,
            })
        })
        .collect::<PyResult<Vec<_>>>()?;
    Ok(DensityAttemptMemoSnapshotV1 { entries })
}
