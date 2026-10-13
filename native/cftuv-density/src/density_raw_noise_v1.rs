//! Сырые декларации → канонический факт и точные рациональные границы шума.
//! Не принимает нормали, ideal, предвычисленные bounds или решения Python.

use crate::interval::{
    rational_add, rational_cmp, rational_div, rational_mul, rational_neg, rational_sign,
    rational_sub,
};
use crate::{IBig, Rational, UBig};
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawNoisePointV1 {
    pub vertex_id: String,
    pub x: Rational,
    pub y: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawNoiseInputV1 {
    pub source_reflex_interval: [Rational; 2],
    pub canonical_excess_over_pi: Option<Rational>,
    pub binding_present: bool,
    pub source_coordinates_available: bool,
    pub incoming_vertex_ids: Vec<String>,
    pub outgoing_vertex_ids: Vec<String>,
    pub anchor_vertex_id: Option<String>,
    pub source_coordinates: Vec<RawNoisePointV1>,
    pub evaluation_coordinates: Vec<RawNoisePointV1>,
    pub source_gram: [Rational; 3],
    pub base_lattice_scale: Option<UBig>,
    pub direction_sine_bound: Option<Rational>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanonicalRelationV1 {
    PiOverTwo,
}
impl CanonicalRelationV1 {
    pub fn as_str(self) -> &'static str {
        "CANONICAL_REFLEX_EXCESS_PI_OVER_2"
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RawNoiseRefusalV1 {
    SourceCoordinatesUnavailable,
    CornerVerticesUnavailable,
    DegenerateCornerEdge,
    LateralShiftExceedsOneLatticeCell,
    EdgeDirectionNoiseExceedsDeclaredBound,
    TurnNoiseExceedsDeclaredBound,
    CanonicalRelationHasNoAngularBound,
}
impl RawNoiseRefusalV1 {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::SourceCoordinatesUnavailable => "SOURCE_COORDINATES_UNAVAILABLE",
            Self::CornerVerticesUnavailable => "CORNER_VERTICES_UNAVAILABLE",
            Self::DegenerateCornerEdge => "DEGENERATE_CORNER_EDGE",
            Self::LateralShiftExceedsOneLatticeCell => "LATERAL_SHIFT_EXCEEDS_ONE_LATTICE_CELL",
            Self::EdgeDirectionNoiseExceedsDeclaredBound => {
                "EDGE_DIRECTION_NOISE_EXCEEDS_DECLARED_BOUND"
            }
            Self::TurnNoiseExceedsDeclaredBound => "TURN_NOISE_EXCEEDS_DECLARED_BOUND",
            Self::CanonicalRelationHasNoAngularBound => "CANONICAL_RELATION_HAS_NO_ANGULAR_BOUND",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RawNoiseInputErrorV1 {
    ReversedInterval,
    CanonicalDeclarationUnproven,
    CanonicalDeclarationMismatch,
    InvalidGram,
    MissingBaseLatticeScale,
    ZeroBaseLatticeScale,
    MissingDirectionSineBound,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawNoiseBoundsV1 {
    pub lateral_squared: Rational,
    pub edge_sine_squared: Rational,
    pub turn_cosine_squared: Rational,
    pub direction_sine_bound: Rational,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawNoiseDecisionV1 {
    pub relation: Option<CanonicalRelationV1>,
    pub canonical: Option<Rational>,
    pub noise: Option<RawNoiseBoundsV1>,
    pub refusal: Option<RawNoiseRefusalV1>,
}
impl RawNoiseDecisionV1 {
    pub fn applicable(&self) -> bool {
        self.canonical.is_some() && self.noise.is_some() && self.refusal.is_none()
    }

    fn refused(mut self, refusal: RawNoiseRefusalV1) -> Self {
        self.refusal = Some(refusal);
        self
    }
}

fn q(numerator: i64, denominator: u64) -> Rational {
    Rational::new(IBig::from(numerator), UBig::from(denominator))
        .expect("positive constant denominator")
}
fn abs(value: &Rational) -> Rational {
    if rational_sign(value) < 0 {
        rational_neg(value)
    } else {
        value.clone()
    }
}
fn max(left: &Rational, right: &Rational) -> Rational {
    if rational_cmp(left, right).is_gt() {
        left.clone()
    } else {
        right.clone()
    }
}

/// Только оплаченный канон π/2; близость покрывает обе границы исходного интервала.
pub fn canonical_selector_fact(
    input: &RawNoiseInputV1,
) -> Result<Option<(CanonicalRelationV1, Rational)>, RawNoiseInputErrorV1> {
    let [lower, upper] = &input.source_reflex_interval;
    if rational_cmp(lower, upper).is_gt() {
        return Err(RawNoiseInputErrorV1::ReversedInterval);
    }
    let canonical = q(1, 2);
    let deviation = max(
        &abs(&rational_sub(lower, &canonical)),
        &abs(&rational_sub(upper, &canonical)),
    );
    let radians = rational_mul(
        &deviation,
        &q(3_141_592_653_589_793_239, 1_000_000_000_000_000_000),
    );
    if rational_cmp(&radians, &q(1745, 1_000_000)).is_gt() {
        return if input.canonical_excess_over_pi.is_some() {
            Err(RawNoiseInputErrorV1::CanonicalDeclarationUnproven)
        } else {
            Ok(None)
        };
    }
    if input
        .canonical_excess_over_pi
        .as_ref()
        .is_some_and(|claim| claim != &canonical)
    {
        return Err(RawNoiseInputErrorV1::CanonicalDeclarationMismatch);
    }
    Ok(Some((CanonicalRelationV1::PiOverTwo, canonical)))
}

fn dot(gram: &[Rational; 3], left: &[Rational; 2], right: &[Rational; 2]) -> Rational {
    let diagonal = rational_add(
        &rational_mul(&gram[0], &rational_mul(&left[0], &right[0])),
        &rational_mul(&gram[2], &rational_mul(&left[1], &right[1])),
    );
    let mixed = rational_add(
        &rational_mul(&left[0], &right[1]),
        &rational_mul(&left[1], &right[0]),
    );
    rational_add(&diagonal, &rational_mul(&gram[1], &mixed))
}

fn coordinates(rows: &[RawNoisePointV1]) -> HashMap<&str, [Rational; 2]> {
    rows.iter()
        .map(|point| (point.vertex_id.as_str(), [point.x.clone(), point.y.clone()]))
        .collect()
}

fn edge_noise(
    gram: &[Rational; 3],
    determinant: &Rational,
    source: &HashMap<&str, [Rational; 2]>,
    evaluation: &HashMap<&str, [Rational; 2]>,
    start: &str,
    end: &str,
) -> Option<(Rational, Rational, [Rational; 2])> {
    let edge = [
        rational_sub(&source[end][0], &source[start][0]),
        rational_sub(&source[end][1], &source[start][1]),
    ];
    let moved = [
        rational_sub(&evaluation[end][0], &evaluation[start][0]),
        rational_sub(&evaluation[end][1], &evaluation[start][1]),
    ];
    let original_squared = dot(gram, &edge, &edge);
    let moved_squared = dot(gram, &moved, &moved);
    if rational_sign(&original_squared) == 0 || rational_sign(&moved_squared) == 0 {
        return None;
    }
    let cross = rational_sub(
        &rational_mul(&edge[0], &moved[1]),
        &rational_mul(&edge[1], &moved[0]),
    );
    let area_squared = rational_mul(determinant, &rational_mul(&cross, &cross));
    Some((
        rational_div(&area_squared, &original_squared).expect("nonzero edge norm"),
        rational_div(
            &area_squared,
            &rational_mul(&original_squared, &moved_squared),
        )
        .expect("nonzero edge norms"),
        moved,
    ))
}

/// Порядок gates совпадает с Python: канон → binding → coordinates → edges → bounds.
pub fn evaluate_raw_noise(
    input: &RawNoiseInputV1,
) -> Result<RawNoiseDecisionV1, RawNoiseInputErrorV1> {
    let fact = canonical_selector_fact(input)?;
    let result = RawNoiseDecisionV1 {
        relation: fact.as_ref().map(|f| f.0),
        canonical: fact.map(|f| f.1),
        noise: None,
        refusal: None,
    };
    if result.canonical.is_none() || !input.binding_present {
        return Ok(result);
    }
    if !input.source_coordinates_available {
        return Ok(result.refused(RawNoiseRefusalV1::SourceCoordinatesUnavailable));
    }
    let anchor = input.anchor_vertex_id.as_deref();
    let incoming = &input.incoming_vertex_ids;
    let outgoing = &input.outgoing_vertex_ids;
    if incoming.len() < 2
        || outgoing.len() < 2
        || incoming.last().map(String::as_str) != anchor
        || outgoing.first().map(String::as_str) != anchor
    {
        return Ok(result.refused(RawNoiseRefusalV1::CornerVerticesUnavailable));
    }
    let ids = [
        incoming[incoming.len() - 2].as_str(),
        anchor.expect("matched string anchor"),
        outgoing[1].as_str(),
    ];
    let source = coordinates(&input.source_coordinates);
    let evaluation = coordinates(&input.evaluation_coordinates);
    if ids
        .iter()
        .any(|id| !source.contains_key(id) || !evaluation.contains_key(id))
    {
        return Ok(result.refused(RawNoiseRefusalV1::CornerVerticesUnavailable));
    }
    let gram = &input.source_gram;
    let determinant = rational_sub(
        &rational_mul(&gram[0], &gram[2]),
        &rational_mul(&gram[1], &gram[1]),
    );
    if rational_sign(&gram[0]) <= 0 || rational_sign(&determinant) <= 0 {
        return Err(RawNoiseInputErrorV1::InvalidGram);
    }
    let first = edge_noise(gram, &determinant, &source, &evaluation, ids[0], ids[1]);
    let second = edge_noise(gram, &determinant, &source, &evaluation, ids[1], ids[2]);
    let (Some(first), Some(second)) = (first, second) else {
        return Ok(result.refused(RawNoiseRefusalV1::DegenerateCornerEdge));
    };
    let cross_dot = dot(gram, &first.2, &second.2);
    let turn_squared = rational_div(
        &rational_mul(&cross_dot, &cross_dot),
        &rational_mul(
            &dot(gram, &first.2, &first.2),
            &dot(gram, &second.2, &second.2),
        ),
    )
    .expect("nonzero edge norms");
    let scale = input
        .base_lattice_scale
        .as_ref()
        .ok_or(RawNoiseInputErrorV1::MissingBaseLatticeScale)?;
    if scale.is_zero() {
        return Err(RawNoiseInputErrorV1::ZeroBaseLatticeScale);
    }
    let numerator = rational_add(
        &rational_add(&gram[0], &rational_mul(&q(2, 1), &abs(&gram[1]))),
        &gram[2],
    );
    let scale_squared = Rational::integer(IBig::from(scale * scale));
    let cell_bound = rational_div(&numerator, &scale_squared).expect("positive base lattice scale");
    let lateral = max(&first.0, &second.0);
    let edge_sine = max(&first.1, &second.1);
    let bound = input
        .direction_sine_bound
        .as_ref()
        .ok_or(RawNoiseInputErrorV1::MissingDirectionSineBound)?;
    let bound_squared = rational_mul(bound, bound);
    if rational_mul(result.canonical.as_ref().expect("canonical gate"), &q(2, 1)) != q(1, 1) {
        return Ok(result.refused(RawNoiseRefusalV1::CanonicalRelationHasNoAngularBound));
    }
    for (value, limit, refusal) in [
        (
            &lateral,
            &cell_bound,
            RawNoiseRefusalV1::LateralShiftExceedsOneLatticeCell,
        ),
        (
            &edge_sine,
            &bound_squared,
            RawNoiseRefusalV1::EdgeDirectionNoiseExceedsDeclaredBound,
        ),
        (
            &turn_squared,
            &bound_squared,
            RawNoiseRefusalV1::TurnNoiseExceedsDeclaredBound,
        ),
    ] {
        if rational_cmp(value, limit).is_gt() {
            return Ok(result.refused(refusal));
        }
    }
    Ok(RawNoiseDecisionV1 {
        noise: Some(RawNoiseBoundsV1 {
            lateral_squared: lateral,
            edge_sine_squared: edge_sine,
            turn_cosine_squared: turn_squared,
            direction_sine_bound: bound.clone(),
        }),
        ..result
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(name: &str, x: Rational, y: Rational) -> RawNoisePointV1 {
        RawNoisePointV1 {
            vertex_id: name.into(),
            x,
            y,
        }
    }
    fn input() -> RawNoiseInputV1 {
        let points = vec![
            point("previous", q(-1, 1), q(0, 1)),
            point("anchor", q(0, 1), q(0, 1)),
            point("next", q(0, 1), q(1, 1)),
        ];
        RawNoiseInputV1 {
            source_reflex_interval: [q(1, 2), q(1, 2)],
            canonical_excess_over_pi: None,
            binding_present: true,
            source_coordinates_available: true,
            incoming_vertex_ids: vec!["previous".into(), "anchor".into()],
            outgoing_vertex_ids: vec!["anchor".into(), "next".into()],
            anchor_vertex_id: Some("anchor".into()),
            source_coordinates: points.clone(),
            evaluation_coordinates: points,
            source_gram: [q(1, 1), q(0, 1), q(1, 1)],
            base_lattice_scale: Some(UBig::from(10u8)),
            direction_sine_bound: Some(q(1, 400)),
        }
    }

    #[test]
    fn exact_closed_canonical_and_whole_interval_tolerance() {
        let mut data = input();
        assert_eq!(
            canonical_selector_fact(&data).unwrap(),
            Some((CanonicalRelationV1::PiOverTwo, q(1, 2)))
        );
        let tolerance = rational_div(
            &q(1745, 1_000_000),
            &q(3_141_592_653_589_793_239, 1_000_000_000_000_000_000),
        )
        .unwrap();
        data.source_reflex_interval = [
            rational_sub(&q(1, 2), &tolerance),
            rational_add(&q(1, 2), &tolerance),
        ];
        assert!(canonical_selector_fact(&data).unwrap().is_some());
        data.source_reflex_interval[1] =
            rational_add(&q(1, 2), &rational_mul(&tolerance, &q(1001, 1000)));
        assert_eq!(canonical_selector_fact(&data), Ok(None));
        data.canonical_excess_over_pi = Some(q(1, 2));
        assert_eq!(
            canonical_selector_fact(&data),
            Err(RawNoiseInputErrorV1::CanonicalDeclarationUnproven)
        );
        data.source_reflex_interval = [q(1, 2), q(1, 2)];
        data.canonical_excess_over_pi = Some(q(1, 4));
        assert_eq!(
            canonical_selector_fact(&data),
            Err(RawNoiseInputErrorV1::CanonicalDeclarationMismatch)
        );
    }

    #[test]
    fn silent_gates_do_not_read_later_declarations() {
        let mut data = input();
        data.binding_present = false;
        data.source_coordinates_available = false;
        data.base_lattice_scale = None;
        data.direction_sine_bound = None;
        assert!(!evaluate_raw_noise(&data).unwrap().applicable());
        data.binding_present = true;
        assert_eq!(
            evaluate_raw_noise(&data).unwrap().refusal,
            Some(RawNoiseRefusalV1::SourceCoordinatesUnavailable)
        );
        data.source_reflex_interval = [q(1, 4), q(1, 4)];
        assert_eq!(evaluate_raw_noise(&data).unwrap().canonical, None);
    }

    #[test]
    fn unavailable_vertices_and_degenerate_edges_precede_bounds() {
        let mut data = input();
        data.base_lattice_scale = None;
        data.direction_sine_bound = None;
        data.anchor_vertex_id = None;
        assert_eq!(
            evaluate_raw_noise(&data).unwrap().refusal,
            Some(RawNoiseRefusalV1::CornerVerticesUnavailable)
        );
        data.anchor_vertex_id = Some("anchor".into());
        data.source_coordinates.remove(2);
        assert_eq!(
            evaluate_raw_noise(&data).unwrap().refusal,
            Some(RawNoiseRefusalV1::CornerVerticesUnavailable)
        );
        data = input();
        data.base_lattice_scale = None;
        data.source_coordinates[0].x = q(0, 1);
        assert_eq!(
            evaluate_raw_noise(&data).unwrap().refusal,
            Some(RawNoiseRefusalV1::DegenerateCornerEdge)
        );
    }

    #[test]
    fn noise_refusal_priority_and_exact_bounds() {
        let mut data = input();
        data.evaluation_coordinates[1].y = q(1, 100);
        data.base_lattice_scale = Some(UBig::from(1000u16));
        assert_eq!(
            evaluate_raw_noise(&data).unwrap().refusal,
            Some(RawNoiseRefusalV1::LateralShiftExceedsOneLatticeCell)
        );
        data.base_lattice_scale = Some(UBig::from(10u8));
        assert_eq!(
            evaluate_raw_noise(&data).unwrap().refusal,
            Some(RawNoiseRefusalV1::EdgeDirectionNoiseExceedsDeclaredBound)
        );
        data = input();
        data.evaluation_coordinates[0].y = q(-1, 600);
        data.evaluation_coordinates[2].x = q(1, 600);
        assert_eq!(
            evaluate_raw_noise(&data).unwrap().refusal,
            Some(RawNoiseRefusalV1::TurnNoiseExceedsDeclaredBound)
        );
        data.evaluation_coordinates[0].y = q(-1, 1000);
        data.evaluation_coordinates[2].x = q(1, 1000);
        let result = evaluate_raw_noise(&data).unwrap();
        assert!(result.applicable());
        let noise = result.noise.unwrap();
        assert_eq!(noise.lateral_squared, q(1, 1_000_000));
        assert_eq!(noise.edge_sine_squared, q(1, 1_000_001));
        assert_eq!(noise.turn_cosine_squared, q(4_000_000, 1_000_002_000_001));
    }

    #[test]
    fn skew_gram_and_invalid_scale_are_exact() {
        let mut data = input();
        data.source_gram = [q(2, 1), q(1, 1), q(3, 1)];
        data.source_coordinates[2].x = q(-1, 2);
        data.evaluation_coordinates = data.source_coordinates.clone();
        assert!(evaluate_raw_noise(&data).unwrap().applicable());
        data.base_lattice_scale = Some(UBig::ZERO);
        assert_eq!(
            evaluate_raw_noise(&data),
            Err(RawNoiseInputErrorV1::ZeroBaseLatticeScale)
        );
    }
}
