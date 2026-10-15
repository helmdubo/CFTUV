//! Закрытые raw declaration facts; ни нормалей, ни seed/ideal, ни derived решения.
use crate::{IBig, Rational, UBig};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RawDensityPointV1 {
    pub x: Rational,
    pub y: Rational,
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RawDensityMetricV1 {
    /// Симметричные записи (00, 01, 11), ещё проверяемые raw geometry gate.
    pub gram: [Rational; 3],
    pub inverse_gram: [Rational; 3],
    pub owner_orientation_sign: i8,
    pub metric_id: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum RawDirectionDeclarationV1 {
    CertifiedPlanar {
        x: Rational,
        y: Rational,
    },
    CertifiedAffine {
        x: Rational,
        y: Rational,
        reference_metric_id: Option<String>,
    },
    /// Остальные declaration kinds source producer не считает доказанной planar direction.
    Other {
        kind: String,
    },
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum ChainUseOrientationV1 {
    AStartToEnd,
    BStartToEnd,
    Other(String),
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RawDensityIncidentV1 {
    pub start: RawDensityPointV1,
    pub end: RawDensityPointV1,
    pub owner_left: Option<bool>,
    pub anchor_vertex_id: Option<String>,
    pub source_vertex_start_id: Option<String>,
    pub source_vertex_end_id: Option<String>,
    pub support_id: Option<String>,
    pub chain_use_orientation: Option<ChainUseOrientationV1>,
    pub source_direction_declarations: Vec<RawDirectionDeclarationV1>,
    pub owner_face_cycles: Vec<Vec<RawDensityPointV1>>,
    pub owner_face_vertex_id_cycles: Vec<Vec<String>>,
    pub owner_face_ids: Vec<String>,
    pub owner_face_edge_ordinals: Vec<usize>,
    pub physical_edge_id: Option<String>,
    pub chain_use_id: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RawDensityGeometryV1 {
    pub metric: RawDensityMetricV1,
    pub incoming: RawDensityIncidentV1,
    pub outgoing: RawDensityIncidentV1,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RawContractFailureV1 {
    InvalidGramFacts,
    InvalidOwnerOrientation,
    InvalidTurnOrientation,
    InvalidCountProfile,
    FaceCycleCardinality,
    FaceIdCardinality,
    FaceVertexIdCardinality,
    FaceEdgeOrdinal,
    SelectionPolicyMismatch,
    SelectionBucketUnproven,
    SelectionDeclarationInvalid,
    DeclarationRowsInvalid,
    CanonMismatch,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RawGeometryRefusalV1 {
    AffineMetricMismatch,
    ConflictingSupportDirections { chain_use_id: Option<String> },
    SupportNeitherTangentNorNormal { physical_edge_id: Option<String> },
    AmbiguousOwnerNormal { physical_edge_id: Option<String> },
    OrderedSupportsDoNotRealizeTurn,
}
impl RawGeometryRefusalV1 {
    pub fn outcome(&self) -> &'static str {
        "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED"
    }
    pub fn message(&self) -> String {
        let id = |value: &Option<String>| value.as_deref().unwrap_or("None").to_owned();
        match self {
            Self::AffineMetricMismatch => {
                "affine support direction references another metric".into()
            }
            Self::ConflictingSupportDirections { chain_use_id } => format!(
                "conflicting certified support directions for {}",
                id(chain_use_id)
            ),
            Self::SupportNeitherTangentNorNormal { physical_edge_id } => format!(
                "certified support direction is neither tangent nor normal for {}",
                id(physical_edge_id)
            ),
            Self::AmbiguousOwnerNormal { physical_edge_id } => format!(
                "owner-interior normal is ambiguous for {}",
                id(physical_edge_id)
            ),
            Self::OrderedSupportsDoNotRealizeTurn => {
                "ordered support normals do not realize the certified owner-sector turn".into()
            }
        }
    }
}

/// Полный закрытый value wire деклараций; не AST и не arbitrary Python object.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum PrimitiveFactV1 {
    Bool(bool),
    Int(IBig),
    Text(String),
    None,
    Tuple(Vec<PrimitiveFactV1>),
}
pub type DeclarationRowsV1 = Vec<(String, PrimitiveFactV1)>;
impl PrimitiveFactV1 {
    pub fn rows(&self) -> Option<DeclarationRowsV1> {
        let Self::Tuple(values) = self else {
            return None;
        };
        values
            .iter()
            .map(|row| {
                let Self::Tuple(row) = row else { return None };
                if row.len() != 2 {
                    return None;
                }
                let Self::Text(name) = &row[0] else {
                    return None;
                };
                Some((name.clone(), row[1].clone()))
            })
            .collect()
    }
    pub fn ratio(&self) -> Option<Rational> {
        let Self::Tuple(values) = self else {
            return None;
        };
        if values.len() != 2 {
            return None;
        }
        let (Self::Int(n), Self::Int(d)) = (&values[0], &values[1]) else {
            return None;
        };
        if d <= &IBig::ZERO {
            return None;
        }
        let d = UBig::try_from(d.clone()).ok()?;
        Rational::new(n.clone(), d).ok()
    }
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RawDensitySelectionV1 {
    pub q: usize,
    pub source_hidden_count: usize,
    pub selection_policy_id: String,
    pub max_subturn_value_id: String,
    pub bucket_c: usize,
    pub selection_certificate_id: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RawDensityCanonicalFactsV1 {
    pub source_reflex_interval: [Rational; 2],
    pub canonical_excess_over_pi: Option<Rational>,
    pub restoration_declaration: DeclarationRowsV1,
    pub noise_policy_declaration: DeclarationRowsV1,
    pub source_reflex_declaration: DeclarationRowsV1,
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RawDensitySpecInputV1 {
    pub source_geometry: RawDensityGeometryV1,
    pub evaluation_geometry: RawDensityGeometryV1,
    pub turn_orientation_sign: i8,
    pub selection: RawDensitySelectionV1,
    pub canonical: RawDensityCanonicalFactsV1,
    pub spec_id: Option<String>,
    pub source_relation_id: Option<String>,
    pub owner_sector_id: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct DensityOracleCanonV1 {
    pub construction: String,
    pub shell: String,
    pub angle: String,
}
impl DensityOracleCanonV1 {
    pub fn candidate() -> Self {
        Self {
            construction: crate::scalar_v2::CANON.into(),
            shell: crate::density_shell_v2::CANON.into(),
            angle: crate::density_angle_value_v1::CANON.into(),
        }
    }
    pub fn validate_candidate(&self) -> Result<(), RawContractFailureV1> {
        if self == &Self::candidate() {
            Ok(())
        } else {
            Err(RawContractFailureV1::CanonMismatch)
        }
    }
}
#[derive(Clone, Debug)]
pub struct DensityOracleLimitsV1 {
    pub preparation_cap: UBig,
    pub search_cap: UBig,
    pub box_cap: usize,
    pub height_cap: UBig,
    pub finite_expand: crate::density_expand_v1::ExpandLimits,
}
impl DensityOracleLimitsV1 {
    pub fn compile_limits(&self) -> crate::fan_compile::CompileLimits {
        crate::fan_compile::CompileLimits {
            preparation_work_cap: self.preparation_cap.clone(),
            search_work_cap: self.search_cap.clone(),
            box_refinement_cap: self.box_cap,
            resource_height_cap: self.height_cap.clone(),
        }
    }
}
#[derive(Clone, Debug)]
pub struct RawDensityBatchInputV1 {
    pub canon_ids: DensityOracleCanonV1,
    pub limits: DensityOracleLimitsV1,
    pub window_law: crate::geometry::WindowLaw,
    pub specs: Vec<RawDensitySpecInputV1>,
}

pub fn declaration_value<'a>(
    rows: &'a DeclarationRowsV1,
    name: &str,
) -> Option<&'a PrimitiveFactV1> {
    rows.iter()
        .rev()
        .find(|(key, _)| key == name)
        .map(|(_, value)| value)
}
pub fn validate_declaration_rows(rows: &DeclarationRowsV1) -> Result<(), RawContractFailureV1> {
    let mut names = std::collections::HashSet::new();
    if rows.iter().any(|(name, _)| !names.insert(name)) {
        Err(RawContractFailureV1::DeclarationRowsInvalid)
    } else {
        Ok(())
    }
}
impl RawDensitySpecInputV1 {
    pub fn validate(&self) -> Result<(), RawContractFailureV1> {
        let selection = &self.selection;
        let h = selection.source_hidden_count;
        let q = selection.q;
        let c = selection.bucket_c;
        if !(2..=6).contains(&q)
            || h >= q
            || c == 0
            || c > q
            || h != if h == 0 && c == 1 { 0 } else { 1.max(c - 1) }
        {
            return Err(RawContractFailureV1::SelectionDeclarationInvalid);
        }
        if !matches!(self.turn_orientation_sign, -1 | 1) {
            return Err(RawContractFailureV1::InvalidTurnOrientation);
        }
        use crate::interval::rational_cmp;
        if rational_cmp(
            &self.canonical.source_reflex_interval[0],
            &self.canonical.source_reflex_interval[1],
        )
        .is_gt()
        {
            return Err(RawContractFailureV1::DeclarationRowsInvalid);
        }
        for rows in [
            &self.canonical.restoration_declaration,
            &self.canonical.noise_policy_declaration,
            &self.canonical.source_reflex_declaration,
        ] {
            validate_declaration_rows(rows)?;
        }
        let forbidden = [
            "evaluation_turn_sign",
            "evaluation_turn_cosine_squared",
            "binding_lateral_offset_gram_squared_bound",
            "edge_direction_sine_squared_bound",
            "noise_applicable",
            "lifted",
            "binding_reasons",
            "ideal",
        ];
        if self
            .canonical
            .noise_policy_declaration
            .iter()
            .any(|(name, _)| forbidden.contains(&name.as_str()))
        {
            return Err(RawContractFailureV1::DeclarationRowsInvalid);
        }
        Ok(())
    }
}
