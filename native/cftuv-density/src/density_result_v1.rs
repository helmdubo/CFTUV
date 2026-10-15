//! Типизированные Density facts: печать не повторяет геометрию/поиск и не угадывает quantum.
use crate::atlas_plan::{DecimalEnvelope, EndpointKind};
use crate::decimal_projection;
use crate::legacy_binding::{LegacyCertificate, ProvenPredicates};
use crate::{IBig, Rational, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResultInvalidV1 {
    MissingQuantum,
    QuantumRange(i64),
    NonIntegralCoefficient,
    EndpointKind,
    Predicates,
    Projection(decimal_projection::ProjectionError),
    WidthMismatch,
    Descriptor,
    Cardinality,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecimalValueFactsV1 {
    pub coefficient: IBig,
    pub exponent: i64,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecimalIntervalFactsV1 {
    pub lower: DecimalValueFactsV1,
    pub upper: DecimalValueFactsV1,
    pub lower_kind: EndpointKind,
    pub upper_kind: EndpointKind,
    pub absolute_error_bound: DecimalValueFactsV1,
}
/// Разряд задан происхождением Decimal; рациональное число проверяется на совместимость.
pub fn decimal_facts(
    value: &Rational,
    exponent: i64,
) -> Result<DecimalValueFactsV1, ResultInvalidV1> {
    if exponent.unsigned_abs() > 10_000 {
        return Err(ResultInvalidV1::QuantumRange(exponent));
    }
    let power: UBig = format!("1{}", "0".repeat(exponent.unsigned_abs() as usize))
        .parse()
        .unwrap();
    let (n, d) = value.parts();
    let (n, d) = if exponent < 0 {
        (n * IBig::from(power), IBig::from(d.clone()))
    } else {
        (n.clone(), IBig::from(d * power))
    };
    if &n % &d != IBig::ZERO {
        return Err(ResultInvalidV1::NonIntegralCoefficient);
    }
    Ok(DecimalValueFactsV1 {
        coefficient: n / d,
        exponent,
    })
}
pub fn interval_facts(value: &DecimalEnvelope) -> Result<DecimalIntervalFactsV1, ResultInvalidV1> {
    let quantum = value.quantum.ok_or(ResultInvalidV1::MissingQuantum)?;
    if matches!(value.lower_kind, EndpointKind::Other)
        || matches!(value.upper_kind, EndpointKind::Other)
    {
        return Err(ResultInvalidV1::EndpointKind);
    }
    Ok(DecimalIntervalFactsV1 {
        lower: decimal_facts(&value.lower, quantum.lower)?,
        upper: decimal_facts(&value.upper, quantum.upper)?,
        lower_kind: value.lower_kind,
        upper_kind: value.upper_kind,
        absolute_error_bound: decimal_facts(
            &value.absolute_error_bound,
            quantum.absolute_error_bound,
        )?,
    })
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BindingCertificateFactsV1 {
    pub bound_primitive_integer_vector: crate::farey::PrimitiveVector,
    pub ideal_window_lower_slope_envelope: DecimalIntervalFactsV1,
    pub ideal_window_upper_slope_envelope: DecimalIntervalFactsV1,
    pub certified_window_width_lower_bound: DecimalValueFactsV1,
    pub proven_predicates: Vec<&'static str>,
    pub binding_reason: Option<String>,
}
pub fn certificate_facts(
    value: &LegacyCertificate,
    binding_reason: Option<String>,
) -> Result<BindingCertificateFactsV1, ResultInvalidV1> {
    if value.proven_predicates != ProvenPredicates::Canonical {
        return Err(ResultInvalidV1::Predicates);
    }
    let lower = interval_facts(&value.ideal_window_lower_slope_envelope)?;
    let upper = interval_facts(&value.ideal_window_upper_slope_envelope)?;
    // Прежний width считается на сетке -27, затем ROUND_HALF_EVEN precision=28.
    let l = decimal_facts(&value.ideal_window_lower_slope_envelope.upper, -27)?.coefficient;
    let u = decimal_facts(&value.ideal_window_upper_slope_envelope.lower, -27)?.coefficient;
    let (width, _) =
        decimal_projection::width_from_units(&l, &u, 28).map_err(ResultInvalidV1::Projection)?;
    let check = decimal_facts(&value.certified_window_width_lower_bound, width.exponent())?;
    if check.coefficient != *width.coefficient() {
        return Err(ResultInvalidV1::WidthMismatch);
    }
    Ok(BindingCertificateFactsV1 {
        bound_primitive_integer_vector: value.bound_primitive_integer_vector.clone(),
        ideal_window_lower_slope_envelope: lower,
        ideal_window_upper_slope_envelope: upper,
        certified_window_width_lower_bound: check,
        proven_predicates: vec![
            "BINDING_INSIDE_OWN_ORDINAL_WINDOW",
            "BINDING_MONOTONE",
            "BINDING_SUBTURN_LE_DELTA_MAX",
        ],
        binding_reason,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChartPieceFactsV1 {
    pub piece_index: usize,
    pub use_x_denominator: bool,
    pub denominator_sign: i8,
    pub lower_slope_envelope: DecimalIntervalFactsV1,
    pub upper_slope_envelope: DecimalIntervalFactsV1,
    pub lower_endpoint_included: bool,
    pub upper_endpoint_included: bool,
    pub slope_increases_in_ordinal_order: bool,
    pub pole_ownership: crate::atlas_plan::PoleOwnership,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OrdinalWindowFactsV1 {
    Local {
        bounds: crate::fan_compile::AuthorityWindowBounds,
        use_x_denominator: bool,
        denominator_sign: i8,
        full_lower_slope_envelope: DecimalIntervalFactsV1,
        full_upper_slope_envelope: DecimalIntervalFactsV1,
    },
    Atlas {
        bounds: crate::fan_compile::AuthorityWindowBounds,
        pieces: Vec<ChartPieceFactsV1>,
        termination_piece_index: usize,
    },
}
fn piece_facts(
    value: &crate::atlas_plan::ChartPiece,
    index: usize,
) -> Result<ChartPieceFactsV1, ResultInvalidV1> {
    use crate::atlas_plan::{BoolField, NumericField, PoleField};
    let (
        NumericField::Int(piece),
        BoolField::Bool(use_x),
        NumericField::Int(sign),
        BoolField::Bool(lower),
        BoolField::Bool(upper),
        BoolField::Bool(increases),
        PoleField::Enum(pole),
    ) = (
        &value.piece_index,
        value.use_x_denominator,
        &value.denominator_sign,
        value.lower_endpoint_included,
        value.upper_endpoint_included,
        value.slope_increases_in_ordinal_order,
        value.pole_ownership,
    )
    else {
        return Err(ResultInvalidV1::Descriptor);
    };
    if !value.is_piece_v1
        || piece != &IBig::from(index)
        || ![IBig::from(-1), IBig::ONE].contains(sign)
    {
        return Err(ResultInvalidV1::Descriptor);
    }
    Ok(ChartPieceFactsV1 {
        piece_index: index,
        use_x_denominator: use_x,
        denominator_sign: if sign == &IBig::ONE { 1 } else { -1 },
        lower_slope_envelope: interval_facts(&value.lower_slope_envelope)?,
        upper_slope_envelope: interval_facts(&value.upper_slope_envelope)?,
        lower_endpoint_included: lower,
        upper_endpoint_included: upper,
        slope_increases_in_ordinal_order: increases,
        pole_ownership: pole,
    })
}
pub fn window_facts(
    value: &crate::fan_compile::PreparedAuthorityWindow,
) -> Result<OrdinalWindowFactsV1, ResultInvalidV1> {
    use crate::fan_compile::PreparedAuthorityWindow;
    Ok(match value {
        PreparedAuthorityWindow::SingleChart { bounds, record } => OrdinalWindowFactsV1::Local {
            bounds: bounds.clone(),
            use_x_denominator: record.use_x_denominator,
            denominator_sign: record.denominator_sign,
            full_lower_slope_envelope: interval_facts(&record.lower_slope_envelope)?,
            full_upper_slope_envelope: interval_facts(&record.upper_slope_envelope)?,
        },
        PreparedAuthorityWindow::Atlas {
            bounds,
            pieces,
            termination_piece_index,
        } => {
            if *termination_piece_index >= pieces.len() {
                return Err(ResultInvalidV1::Descriptor);
            }
            OrdinalWindowFactsV1::Atlas {
                bounds: bounds.clone(),
                pieces: pieces
                    .iter()
                    .enumerate()
                    .map(|(i, piece)| piece_facts(piece, i))
                    .collect::<Result<_, _>>()?,
                termination_piece_index: *termination_piece_index,
            }
        }
    })
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FareyHeightWitnessFactsV1 {
    pub first_height: UBig,
    pub last_height: UBig,
    pub primitive_candidate_counts: Vec<usize>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdaptiveFanAuthorityFactsV1 {
    pub max_subturn_q: usize,
    pub minimal_common_height: UBig,
    pub exhaustive_previous_height: UBig,
    pub termination_height_upper_bound: UBig,
    pub bound_primitive_integer_vectors: Vec<crate::farey::PrimitiveVector>,
    pub binding_reasons: Vec<Option<String>>,
    pub ordinal_windows: Vec<OrdinalWindowFactsV1>,
    pub previous_height_witness: FareyHeightWitnessFactsV1,
    pub proven_predicates: Vec<&'static str>,
    pub authority_id: Option<String>,
}
pub fn authority_facts(
    value: &crate::fan_compile::PreparedFanAuthority,
    binding_reasons: &[Option<String>],
    law: crate::geometry::WindowLaw,
) -> Result<AdaptiveFanAuthorityFactsV1, ResultInvalidV1> {
    let n = value.ordinal_windows.len();
    if binding_reasons.len() != n
        || value.bound_primitive_integer_vectors.len() != n
        || value.previous_counts.len() != n
    {
        return Err(ResultInvalidV1::Cardinality);
    }
    let mut predicates = vec![
        "ADAPTIVE_FAN_FAREY_HEIGHT_RANGE",
        "ADAPTIVE_FAN_GRAM_OBJECTIVE",
        "ADAPTIVE_FAN_INNER_OUTWARD_ENVELOPE",
        "ADAPTIVE_FAN_MINIMAL_COMMON_HEIGHT",
        "ADAPTIVE_FAN_MONOTONE",
        "ADAPTIVE_FAN_ORDINAL_WINDOWS",
        "ADAPTIVE_FAN_POSITIVE_TERMINATION_WIDTH",
        "ADAPTIVE_FAN_SUBTURN",
    ];
    if law == crate::geometry::WindowLaw::NarrowBand {
        predicates.push("ADAPTIVE_FAN_NARROW_ROTATION_BAND");
        predicates.sort_unstable();
    }
    let windows = value
        .ordinal_windows
        .iter()
        .map(window_facts)
        .collect::<Result<Vec<_>, _>>()?;
    for (index, window) in windows.iter().enumerate() {
        let bounds = match window {
            OrdinalWindowFactsV1::Local { bounds, .. }
            | OrdinalWindowFactsV1::Atlas { bounds, .. } => bounds,
        };
        if bounds.ordinal != index + 1 {
            return Err(ResultInvalidV1::Cardinality);
        }
    }
    Ok(AdaptiveFanAuthorityFactsV1 {
        max_subturn_q: value.max_subturn_q,
        minimal_common_height: value.minimal_common_height.clone(),
        exhaustive_previous_height: value.exhaustive_previous_height.clone(),
        termination_height_upper_bound: value.termination_height_upper_bound.clone(),
        bound_primitive_integer_vectors: value.bound_primitive_integer_vectors.clone(),
        binding_reasons: binding_reasons.to_vec(),
        ordinal_windows: windows,
        previous_height_witness: FareyHeightWitnessFactsV1 {
            first_height: UBig::ONE,
            last_height: value.exhaustive_previous_height.clone(),
            primitive_candidate_counts: value.previous_counts.clone(),
        },
        proven_predicates: predicates,
        authority_id: None,
    })
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BindingResultFactsV1 {
    pub certificates: Vec<Option<BindingCertificateFactsV1>>,
    pub authority: Option<AdaptiveFanAuthorityFactsV1>,
}
pub fn binding_result_facts(
    value: &crate::density_bind_v1::OwnedBindingResultV1,
    binding_reasons: &[Option<String>],
    law: crate::geometry::WindowLaw,
) -> Result<BindingResultFactsV1, ResultInvalidV1> {
    if value.certificates.len() != binding_reasons.len() {
        return Err(ResultInvalidV1::Cardinality);
    }
    let certificates = value
        .certificates
        .iter()
        .zip(binding_reasons)
        .map(|(value, reason)| {
            value
                .as_ref()
                .map(|value| certificate_facts(value, reason.clone()))
                .transpose()
        })
        .collect::<Result<_, _>>()?;
    let authority = value
        .authority
        .as_ref()
        .map(|value| authority_facts(value, binding_reasons, law))
        .transpose()?;
    Ok(BindingResultFactsV1 {
        certificates,
        authority,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::atlas_plan::DecimalQuantum;
    fn envelope(exponent: Option<i64>) -> DecimalEnvelope {
        DecimalEnvelope {
            lower: Rational::integer(IBig::ONE),
            upper: Rational::integer(IBig::ONE),
            absolute_error_bound: Rational::integer(IBig::ZERO),
            lower_kind: EndpointKind::Closed,
            upper_kind: EndpointKind::Closed,
            quantum: exponent.map(DecimalQuantum::uniform),
        }
    }
    #[test]
    fn authored_quantum_survives_equal_rational_values() {
        let exact = interval_facts(&envelope(Some(0))).unwrap();
        let shell = interval_facts(&envelope(Some(-27))).unwrap();
        assert_eq!(exact.lower.coefficient, IBig::ONE);
        assert_eq!(exact.lower.exponent, 0);
        assert_eq!(
            shell.lower.coefficient.to_string(),
            "1000000000000000000000000000"
        );
        assert_eq!(shell.absolute_error_bound.exponent, -27);
        assert_ne!(exact, shell);
        assert_eq!(
            interval_facts(&envelope(None)),
            Err(ResultInvalidV1::MissingQuantum)
        );
    }
}
