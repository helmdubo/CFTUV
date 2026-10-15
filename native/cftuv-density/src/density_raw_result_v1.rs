//! Полный closed raw batch result: formatter/IDs за границей; никакого AST/host объекта.
use crate::density_result_v1::{AdaptiveFanAuthorityFactsV1, BindingCertificateFactsV1};
use crate::{IBig, Rational, UBig};
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonicalFanAuthorityFactsV1 {
    pub ray_law: String,
    pub canonical_relation: String,
    pub canonical_reflex_excess_over_pi: Rational,
    pub hidden_edge_count: usize,
    pub max_subturn_q: usize,
    pub ray_rotation_pairs: Vec<[IBig; 2]>,
    pub bound_primitive_integer_vectors: Vec<crate::farey::PrimitiveVector>,
    pub proven_predicates: Vec<String>,
    pub authority_id: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FanAuthorityFactsV1 {
    Adaptive(AdaptiveFanAuthorityFactsV1),
    Canonical(CanonicalFanAuthorityFactsV1),
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CountLiftFactsV1 {
    pub lift_law: String,
    pub source_hidden_edge_count: usize,
    pub effective_hidden_edge_count: usize,
    pub max_subturn_q: usize,
    pub evaluation_turn_sign: String,
    pub evaluation_turn_cosine_squared: Rational,
    pub minimality_predecessor_hidden_edge_count: usize,
    pub proven_predicates: Vec<String>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonUsageFactsV1 {
    pub canon: String,
    pub applied_event: String,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DensityPriceArticleV1 {
    pub phase: String,
    pub cap: UBig,
    pub counters: Vec<(String, UBig)>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NoiseRecordFactsV1 {
    pub noise_law: String,
    pub effect: String,
    pub canonical_relation: String,
    pub canonical_reflex_excess_over_pi: Rational,
    pub source_hidden_edge_count: usize,
    pub effective_hidden_edge_count: usize,
    pub max_subturn_q: usize,
    pub evaluation_turn_sign: String,
    pub evaluation_turn_cosine_squared: Rational,
    pub binding_lateral_offset_gram_squared_bound: Rational,
    pub edge_direction_sine_squared_bound: Rational,
    pub direction_sine_bound: Rational,
    pub proven_predicates: Vec<String>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonicalSubturnAuthorityFactsV1 {
    pub guarantee_law: String,
    pub canonical_relation: String,
    pub canonical_reflex_excess_over_pi: Rational,
    pub hidden_edge_count: usize,
    pub max_subturn_q: usize,
    pub canonical_subturn_over_pi: Rational,
    pub raw_residual_upper_bound_radians: Rational,
    pub proven_predicates: Vec<String>,
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CanonicalLawRefusalFactsV1 {
    pub noise_refusal: Option<String>,
    pub rays_refusal: Option<String>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DensityShellEventFactsV1 {
    pub value_wire: Vec<IBig>,
    pub codes: Vec<String>,
    pub rational_numerator: Option<IBig>,
    pub rational_denominator: Option<UBig>,
    pub rational_grid_units: Option<IBig>,
    pub legacy_projected_min_width_units: Option<UBig>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DensityFailureFactsV1 {
    pub exception_type: String,
    pub outcome: String,
    pub message: String,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawDensitySpecResultV1 {
    pub outcome: String,
    pub effective_hidden_count: Option<usize>,
    pub binding_reasons: Vec<Option<String>>,
    pub binding_certificates: Vec<Option<BindingCertificateFactsV1>>,
    pub authority: Option<FanAuthorityFactsV1>,
    pub count_lift: Option<CountLiftFactsV1>,
    pub canon_usage: Vec<CanonUsageFactsV1>,
    pub ordered_work_articles: Vec<DensityPriceArticleV1>,
    pub named_events: Vec<(String, crate::density_raw_types_v1::PrimitiveFactV1)>,
    pub named_refusals: Vec<(Option<String>, String)>,
    pub rust_work_diagnostic: Vec<(String, UBig)>,
    pub noise_record: Option<NoiseRecordFactsV1>,
    pub canonical_subturn_authority: Option<CanonicalSubturnAuthorityFactsV1>,
    pub law_refusals: CanonicalLawRefusalFactsV1,
    pub shell_events: Vec<DensityShellEventFactsV1>,
    pub failure: Option<DensityFailureFactsV1>,
}
impl RawDensitySpecResultV1 {
    pub fn blank(construction: &str) -> Self {
        Self {
            outcome: "OK".into(),
            effective_hidden_count: None,
            binding_reasons: vec![],
            binding_certificates: vec![],
            authority: None,
            count_lift: None,
            canon_usage: vec![CanonUsageFactsV1 {
                canon: construction.into(),
                applied_event: "DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED".into(),
            }],
            ordered_work_articles: vec![],
            named_events: vec![],
            named_refusals: vec![],
            rust_work_diagnostic: vec![],
            noise_record: None,
            canonical_subturn_authority: None,
            law_refusals: CanonicalLawRefusalFactsV1::default(),
            shell_events: vec![],
            failure: None,
        }
    }
}
#[derive(Clone, Debug)]
pub struct RawBatchInternalFailureV1 {
    pub spec_index: usize,
    pub detail: String,
    pub partial_spec_effects: RawDensitySpecResultV1,
}
#[derive(Clone, Debug)]
pub struct DensityRawBatchReportV1 {
    pub results: Vec<RawDensitySpecResultV1>,
    pub internal_failure: Option<RawBatchInternalFailureV1>,
}
