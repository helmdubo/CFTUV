//! Pure raw batch: одна arena, registry, metric cache и success-only per-window memo.
//! Declaration → geometry/count/noise → coupled bind → closed facts; не replay/AST route.
use crate::density_attempt_memo_v1::*;
use crate::density_bind_v1::{
    self, OwnedBindingFailureV1, OwnedBindingReportV1, OwnedBindingResultV1,
};
use crate::density_construct_v1::{ConstructorV1, VectorExprV1};
use crate::density_expression::Expressions;
use crate::density_laws_v1::{LawsErrorV1, OwnedDensityLawsV1, OwnedFanGeometryV1};
use crate::density_raw_laws_v1::*;
use crate::density_raw_noise_v1::*;
use crate::density_raw_result_v1::*;
use crate::density_raw_types_v1::*;
use crate::density_result_v1::{binding_result_facts, ResultInvalidV1};
use crate::geometry::WindowLaw;
use crate::interval::{rational_cmp, rational_div, rational_mul};
use crate::{IBig, Rational, UBig};
use std::collections::{BTreeMap, HashMap};

#[derive(Debug)]
enum Failure {
    Raw(RawLawsFailureV1),
    Bind(OwnedBindingFailureV1),
    Result(ResultInvalidV1),
    Noise(RawNoiseInputErrorV1),
    Declaration(&'static str),
    Named(DensityFailureFactsV1),
}
impl Failure {
    fn internal_detail(&self) -> String {
        match self {
            Self::Raw(error) => format!("Raw({error:?})"),
            Self::Bind(error) => format!("Bind({error:?})"),
            Self::Result(error) => format!("Result({error:?})"),
            Self::Noise(error) => format!("Noise({error:?})"),
            Self::Declaration(error) => format!("Declaration({error:?})"),
            Self::Named(error) => format!("Named({error:?})"),
        }
    }
}
impl From<RawLawsFailureV1> for Failure {
    fn from(e: RawLawsFailureV1) -> Self {
        Self::Raw(e)
    }
}
impl From<ResultInvalidV1> for Failure {
    fn from(e: ResultInvalidV1) -> Self {
        Self::Result(e)
    }
}
type Result<T> = std::result::Result<T, Failure>;
fn named(exception: &str, outcome: &str, message: impl Into<String>) -> DensityFailureFactsV1 {
    DensityFailureFactsV1 {
        exception_type: exception.into(),
        outcome: outcome.into(),
        message: message.into(),
    }
}
fn geometry(outcome: &str, message: impl Into<String>) -> Failure {
    Failure::Named(named("ReferenceGeometryError", outcome, message))
}
fn binding_unproven(predicate: &str) -> Failure {
    Failure::Named(named("DirectionBindingCertificateUnproven","REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE",format!("direction binding certificate is not proven: привязка: сертификат не доказан ({predicate})")))
}
fn named_failure(error: &Failure) -> Option<DensityFailureFactsV1> {
    match error {
        Failure::Named(e) => Some(e.clone()),
        Failure::Raw(RawLawsFailureV1::Geometry(e)) => {
            Some(named("ReferenceGeometryError", e.outcome(), e.message()))
        }
        Failure::Raw(RawLawsFailureV1::CanonicalRestorationInvalid { message }) => Some(named(
            "ReferenceGeometryError",
            "REFERENCE_CANONICAL_ANGLE_RESTORATION_INVALID",
            message,
        )),
        Failure::Raw(RawLawsFailureV1::Arithmetic(e)) => laws_named_failure(e),
        Failure::Bind(e) => density_bind_v1::binding_failure_text(e).and_then(|text| {
            let (exception, message) = text.split_once(": ")?;
            let (outcome, message) = match exception {
                "ReferenceGeometryError" => (
                    "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE",
                    message.to_owned(),
                ),
                "DirectionBindingCertificateUnproven" => (
                    "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE",
                    format!("direction binding certificate is not proven: {message}"),
                ),
                "DensityRationalAuthorityExhausted" => {
                    ("DENSITY_RATIONAL_AUTHORITY_EXHAUSTED", message.to_owned())
                }
                "DensityWindowChartUnrepresentable" => (
                    "DENSITY_WINDOW_CHART_UNREPRESENTABLE",
                    "DENSITY_WINDOW_CHART_UNREPRESENTABLE".into(),
                ),
                "DensityTerminationBoxesExhausted" => {
                    ("DENSITY_FAN_BOX_REFINEMENT_EXHAUSTED", message.to_owned())
                }
                "AdaptiveDensityFanInvalid" => ("DENSITY_SEALED_FAN_INVALID", message.to_owned()),
                _ => return None,
            };
            Some(named(exception, outcome, message))
        }),
        _ => None,
    }
}
fn laws_named_failure(error: &LawsErrorV1) -> Option<DensityFailureFactsV1> {
    let failure = OwnedBindingFailureV1::Windows(
        crate::window_build::WindowBuildFailure::Callback(error.clone()),
    );
    named_failure(&Failure::Bind(failure))
}
fn sign_name(sign: i8) -> String {
    match sign {
        1 => "POSITIVE",
        -1 => "NEGATIVE",
        _ => "ZERO",
    }
    .into()
}
fn predicates(values: &[&str]) -> Vec<String> {
    let mut values = values.iter().map(|v| v.to_string()).collect::<Vec<_>>();
    values.sort();
    values.dedup();
    values
}

struct Frontend<'c, 'g> {
    laws: RawDensityLawsV1<'c, 'g>,
    canon: DensityOracleCanonV1,
    limits: DensityOracleLimitsV1,
    law: WindowLaw,
    memo: HashMap<DensityAttemptKeyV1, DensityAttemptReceiptV1>,
    delta: DensityAttemptMemoSnapshotV1,
    prices: Vec<DensityPriceArticleV1>,
    shells: Vec<(Vec<IBig>, crate::density_shell_v2::OutcomeBundle)>,
}
impl<'c, 'g> Frontend<'c, 'g> {
    fn evaluate(
        &mut self,
        raw: &RawDensitySpecInputV1,
    ) -> std::result::Result<RawDensitySpecResultV1, (Failure, RawDensitySpecResultV1)> {
        self.prices.clear();
        self.shells.clear();
        let result = self.compute(raw);
        let mut result = match result {
            Ok(v) => v,
            Err(e) => {
                let mut blank = RawDensitySpecResultV1::blank(&self.canon.construction);
                if let Some(failure) = named_failure(&e) {
                    blank.outcome = failure.outcome.clone();
                    blank.failure = Some(failure);
                    blank
                } else {
                    blank.ordered_work_articles = std::mem::take(&mut self.prices);
                    blank.shell_events = self.shell_facts();
                    return Err((e, blank));
                }
            }
        };
        result.ordered_work_articles = std::mem::take(&mut self.prices);
        result.shell_events = self.shell_facts();
        Ok(result)
    }
    fn shell_facts(&mut self) -> Vec<DensityShellEventFactsV1> {
        let mut unique = BTreeMap::new();
        for (wire, bundle) in self.shells.drain(..) {
            unique.insert(wire, bundle);
        }
        unique
            .into_iter()
            .map(|(wire, bundle)| {
                let (n, d, grid) = bundle
                    .rational
                    .map(|r| (Some(r.numerator), Some(r.denominator), r.grid_units))
                    .unwrap_or((None, None, None));
                DensityShellEventFactsV1 {
                    value_wire: wire,
                    codes: bundle.codes.iter().map(|code| code.code().into()).collect(),
                    rational_numerator: n,
                    rational_denominator: d,
                    rational_grid_units: grid,
                    legacy_projected_min_width_units: bundle
                        .legacy_projected_min_width_units
                        .map(UBig::from),
                }
            })
            .collect()
    }
    fn make_ideal(
        &mut self,
        m: &ProducedRawMetricV1,
        incoming: &VectorExprV1,
        outgoing: &VectorExprV1,
        count: usize,
        evaluation: bool,
        raw: &RawDensitySpecInputV1,
        noise: &RawNoiseDecisionV1,
        canonical_used: &mut HashMap<(bool, usize), bool>,
    ) -> Result<Vec<VectorExprV1>> {
        let turn = raw.turn_orientation_sign;
        let q = raw.selection.q;
        let witness = self.laws.turn_witness(m, incoming, outgoing, turn)?;
        let fan = self
            .laws
            .principal_fan(m, incoming, outgoing, turn, count)?
            .ideal;
        if self
            .laws
            .turn_count_feasible(witness.dot_sign, &witness.cosine_squared, count, q)?
        {
            return Ok(fan);
        }
        let mut canonical = raw.canonical.canonical_excess_over_pi.clone();
        if canonical.is_none()
            && evaluation
            && noise.applicable()
            && canonical_steps(noise.canonical.as_ref(), q, count + 1)
        {
            let candidate = self.laws.canonical_fan(
                m,
                incoming,
                outgoing,
                turn,
                count,
                noise.canonical.as_ref().unwrap(),
            )?;
            let mut rational = true;
            for ray in &candidate[1..candidate.len() - 1] {
                if !matches!(
                    self.laws.rational_direction(ray)?,
                    RationalDirectionV1::Rational(_)
                ) {
                    rational = false;
                    break;
                }
            }
            if rational {
                canonical = noise.canonical.clone();
            }
        }
        if let Some(canonical) = canonical {
            if rational_cmp(
                &rational_mul(&canonical, &Rational::integer(IBig::from(q))),
                &Rational::integer(IBig::from(count + 1)),
            )
            .is_gt()
            {
                return Err(geometry(
                    "REFERENCE_CANONICAL_ANGLE_RESTORATION_INVALID",
                    "canonical subturn exceeds the declared maximum subturn",
                ));
            }
            let fan = self
                .laws
                .canonical_fan(m, incoming, outgoing, turn, count, &canonical)?;
            canonical_used.insert((evaluation, count), true);
            Ok(fan)
        } else {
            Ok(fan)
        }
    }
    fn any_irrational(&mut self, fan: &[VectorExprV1]) -> Result<bool> {
        for ray in &fan[1..fan.len() - 1] {
            if self.laws.rational_direction(ray)? == RationalDirectionV1::Irrational {
                return Ok(true);
            }
        }
        Ok(false)
    }
    fn exact_irrational(
        &mut self,
        fan: &[VectorExprV1],
        count: usize,
        q: usize,
        witness: &TurnWitnessV1,
    ) -> Result<bool> {
        Ok(self
            .laws
            .turn_count_exact_limit(witness.dot_sign, &witness.cosine_squared, count, q)?
            && self.any_irrational(fan)?)
    }
    fn feasible(
        &mut self,
        fan: &[VectorExprV1],
        count: usize,
        raw: &RawDensitySpecInputV1,
        m: &ProducedRawMetricV1,
        incoming: &VectorExprV1,
        outgoing: &VectorExprV1,
        noise: &RawNoiseDecisionV1,
        witness: &TurnWitnessV1,
        used: &HashMap<(bool, usize), bool>,
    ) -> Result<bool> {
        let q = raw.selection.q;
        if noise.applicable() && canonical_steps(noise.canonical.as_ref(), q, count + 1) {
            let canonical = self.laws.canonical_fan(
                m,
                incoming,
                outgoing,
                raw.turn_orientation_sign,
                count,
                noise.canonical.as_ref().unwrap(),
            )?;
            let first = self.laws.turn_witness(
                m,
                &canonical[0],
                &canonical[1],
                raw.turn_orientation_sign,
            )?;
            for pair in canonical[..canonical.len() - 1].windows(2) {
                if !self.laws.subturn(m, &pair[0], &pair[1], q)? {
                    return Ok(false);
                }
            }
            return Ok(!(self.laws.turn_count_exact_limit(
                first.dot_sign,
                &first.cosine_squared,
                0,
                q,
            )? && self.any_irrational(&canonical)?));
        }
        if used.get(&(true, count)).copied().unwrap_or(false) {
            let angular = self.laws.subturn(m, &fan[0], &fan[1], q)?;
            let boundary = self.laws.subturn_boundary(m, &fan[0], &fan[1], q)?;
            return Ok(angular && !(boundary && self.any_irrational(fan)?));
        }
        Ok(self
            .laws
            .turn_count_feasible(witness.dot_sign, &witness.cosine_squared, count, q)?
            && !self.exact_irrational(fan, count, q, witness)?)
    }
    fn compute(&mut self, raw: &RawDensitySpecInputV1) -> Result<RawDensitySpecResultV1> {
        raw.validate()
            .map_err(|e| Failure::Raw(RawLawsFailureV1::Contract(e)))?;
        let q = raw.selection.q;
        let source_count = raw.selection.source_hidden_count;
        let turn = raw.turn_orientation_sign;
        let source = self.laws.raw_geometry(&raw.source_geometry, true)?;
        let binding_present = decl_bool(
            &raw.canonical.noise_policy_declaration,
            "binding_present",
            None,
        )?;
        let evaluation = self
            .laws
            .raw_geometry(&raw.evaluation_geometry, !binding_present)?;
        let si = self.laws.covector(&source.metric, &source.incoming)?;
        let so = self.laws.covector(&source.metric, &source.outgoing)?;
        let ei = self
            .laws
            .covector(&evaluation.metric, &evaluation.incoming)?;
        let eo = self
            .laws
            .covector(&evaluation.metric, &evaluation.outgoing)?;
        let witness = self.laws.turn_witness(&evaluation.metric, &ei, &eo, turn)?;
        let noise =
            evaluate_raw_noise(&decode_noise(raw, binding_present)?).map_err(Failure::Noise)?;
        self.laws
            .verify_raw_selection(raw, noise.canonical.as_ref())?;
        let mut used = HashMap::new();
        let mut ideal = self.make_ideal(
            &evaluation.metric,
            &ei,
            &eo,
            source_count,
            true,
            raw,
            &noise,
            &mut used,
        )?;
        let mut predecessor = ideal.clone();
        let mut count = source_count;
        let mut lift = None;
        if !self.feasible(
            &ideal,
            count,
            raw,
            &evaluation.metric,
            &ei,
            &eo,
            &noise,
            &witness,
            &used,
        )? {
            let mut found = false;
            for next in source_count + 1..q {
                count = next;
                ideal = self.make_ideal(
                    &evaluation.metric,
                    &ei,
                    &eo,
                    count,
                    true,
                    raw,
                    &noise,
                    &mut used,
                )?;
                if self.feasible(
                    &ideal,
                    count,
                    raw,
                    &evaluation.metric,
                    &ei,
                    &eo,
                    &noise,
                    &witness,
                    &used,
                )? {
                    found = true;
                    break;
                }
                predecessor = ideal.clone();
            }
            if !found {
                return Err(binding_unproven("BINDING_SUBTURN_LE_DELTA_MAX"));
            }
            let law = if !self.laws.turn_count_feasible(
                witness.dot_sign,
                &witness.cosine_squared,
                source_count,
                q,
            )? && !self.laws.turn_count_feasible(
                witness.dot_sign,
                &witness.cosine_squared,
                count - 1,
                q,
            )? {
                0
            } else if self.exact_irrational(&predecessor, count - 1, q, &witness)? {
                1
            } else if noise.applicable() && canonical_steps(noise.canonical.as_ref(), q, count) {
                let canonical = self.laws.canonical_fan(
                    &evaluation.metric,
                    &ei,
                    &eo,
                    turn,
                    count - 1,
                    noise.canonical.as_ref().unwrap(),
                )?;
                if !self.any_irrational(&canonical)? {
                    return Err(Failure::Declaration(
                        "DENSITY_RAW_CANONICAL_LIFT_NOT_PROVEN",
                    ));
                }
                2
            } else {
                return Err(Failure::Declaration("DENSITY_RAW_LIFT_NOT_PROVEN"));
            };
            lift = Some(CountLiftFactsV1 {
                lift_law: lift_law(law).into(),
                source_hidden_edge_count: source_count,
                effective_hidden_edge_count: count,
                max_subturn_q: q,
                evaluation_turn_sign: sign_name(witness.dot_sign),
                evaluation_turn_cosine_squared: witness.cosine_squared.clone(),
                minimality_predecessor_hidden_edge_count: count - 1,
                proven_predicates: lift_predicates(law),
            });
        }
        let source_ideal = self.make_ideal(
            &source.metric,
            &si,
            &so,
            count,
            false,
            raw,
            &noise,
            &mut used,
        )?;
        self.finish(
            raw,
            &evaluation.metric,
            &ei,
            &eo,
            &source_ideal,
            &noise,
            count,
            lift,
            &ideal,
            used.get(&(true, count)).copied().unwrap_or(false),
            binding_present,
            &witness,
        )
    }
    fn finish(
        &mut self,
        raw: &RawDensitySpecInputV1,
        m: &ProducedRawMetricV1,
        ei: &VectorExprV1,
        eo: &VectorExprV1,
        source_ideal: &[VectorExprV1],
        noise: &RawNoiseDecisionV1,
        count: usize,
        lift: Option<CountLiftFactsV1>,
        ideal: &[VectorExprV1],
        canonical_used: bool,
        binding_present: bool,
        witness: &TurnWitnessV1,
    ) -> Result<RawDensitySpecResultV1> {
        let q = raw.selection.q;
        let turn = raw.turn_orientation_sign;
        let mut needed = vec![];
        let mut reasons = vec![];
        for (source, ray) in source_ideal[1..source_ideal.len() - 1]
            .iter()
            .zip(&ideal[1..ideal.len() - 1])
        {
            let rational = matches!(
                self.laws.rational_direction(source)?,
                RationalDirectionV1::Rational(_)
            );
            let need = !rational
                || !matches!(
                    self.laws.rational_direction(ray)?,
                    RationalDirectionV1::Rational(_)
                );
            needed.push(need);
            reasons.push(if !rational {
                Some("SOURCE_DIRECTION_IRRATIONAL".into())
            } else if need {
                Some("EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL".into())
            } else {
                None
            });
        }
        let mut authority = None;
        let mut certificates = vec![None; count];
        let mut rays_refusal = None;
        if let Some(canonical) = &noise.canonical {
            let rotations = rotation_table(canonical, count + 1, q);
            if rotations.is_none() {
                if lift.is_some() {
                    rays_refusal = Some("NO_CANONICAL_ROTATION_TABLE_ENTRY".into());
                }
            } else if noise.refusal.is_some() {
                rays_refusal = Some("BINDING_NOISE_OUTSIDE_THE_DECLARED_BOUNDS".into());
            } else {
                let rotations = rotations.unwrap();
                let fan = self.laws.rotation_fan(m, ei, eo, turn, &rotations)?;
                let mut vectors = vec![];
                for ray in &fan[1..fan.len() - 1] {
                    match self.laws.rational_direction(ray)? {
                        RationalDirectionV1::Rational(v) => vectors.push(v),
                        _ => {
                            rays_refusal = Some("CANONICAL_RAYS_IRRATIONAL_IN_CHART".into());
                            break;
                        }
                    }
                }
                if rays_refusal.is_none() {
                    let mut valid = true;
                    for pair in fan.windows(2) {
                        if !self.laws.subturn(m, &pair[0], &pair[1], q)? {
                            valid = false;
                            break;
                        }
                        let cross = self.laws.oriented_cross(m, &pair[0], &pair[1])?;
                        if self.laws.sign(&cross)? != turn {
                            valid = false;
                            break;
                        }
                    }
                    if !valid {
                        rays_refusal =
                            Some("CANONICAL_ROTATION_FAN_VIOLATES_SUBTURN_GUARANTEE".into());
                    } else if lift.is_some() || needed.iter().any(|v| *v) {
                        authority = Some(FanAuthorityFactsV1::Canonical(
                            CanonicalFanAuthorityFactsV1 {
                                ray_law: "CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1".into(),
                                canonical_relation: noise.relation.unwrap().as_str().into(),
                                canonical_reflex_excess_over_pi: canonical.clone(),
                                hidden_edge_count: count,
                                max_subturn_q: q,
                                ray_rotation_pairs: rotations
                                    .iter()
                                    .map(|r| [IBig::from(r[0]), IBig::from(r[1])])
                                    .collect(),
                                bound_primitive_integer_vectors: vectors,
                                proven_predicates: canonical_ray_predicates(),
                                authority_id: None,
                            },
                        ));
                    }
                }
            }
        }
        let mut named_refusals = vec![];
        if authority.is_none() && (needed.iter().any(|v| *v) || lift.is_some()) {
            let (bound, law, refusal) = self.bind(
                m,
                ideal,
                turn,
                q,
                &reasons,
                &needed,
                lift.is_some(),
                !binding_present,
            )?;
            if let Some(refusal) = refusal {
                named_refusals.push((raw.spec_id.clone(), refusal));
            }
            let mut printed = binding_result_facts(&bound, &reasons, law)?;
            for (index, certificate) in printed.certificates.iter_mut().enumerate() {
                if !needed[index] {
                    *certificate = None;
                } else if let Some(certificate) = certificate {
                    if !binding_present {
                        certificate.binding_reason = None;
                    }
                }
            }
            certificates = printed.certificates;
            authority = printed.authority.map(FanAuthorityFactsV1::Adaptive);
        }
        if matches!(authority, Some(FanAuthorityFactsV1::Canonical(_)))
            || authority.is_none() && certificates.iter().all(Option::is_none)
        {
            rays_refusal = None;
        }
        let mut result = RawDensitySpecResultV1::blank(&self.canon.construction);
        result.effective_hidden_count = Some(count);
        result.binding_reasons = reasons;
        result.binding_certificates = certificates;
        result.authority = authority;
        result.count_lift = lift;
        result.named_refusals = named_refusals;
        let effect = if result
            .count_lift
            .as_ref()
            .is_some_and(|l| l.lift_law == lift_law(2))
        {
            Some("CANONICAL_COUNT_LIFT")
        } else if canonical_used && raw.canonical.canonical_excess_over_pi.is_none() {
            Some("CANONICAL_ROTATION_FAN")
        } else {
            None
        };
        if let Some(effect) = effect {
            let bounds = noise
                .noise
                .as_ref()
                .ok_or(Failure::Declaration("DENSITY_RAW_NOISE_BOUNDS_MISSING"))?;
            result.noise_record = Some(NoiseRecordFactsV1 {
                noise_law: "EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1".into(),
                effect: effect.into(),
                canonical_relation: noise.relation.unwrap().as_str().into(),
                canonical_reflex_excess_over_pi: noise.canonical.clone().unwrap(),
                source_hidden_edge_count: raw.selection.source_hidden_count,
                effective_hidden_edge_count: count,
                max_subturn_q: q,
                evaluation_turn_sign: sign_name(witness.dot_sign),
                evaluation_turn_cosine_squared: witness.cosine_squared.clone(),
                binding_lateral_offset_gram_squared_bound: bounds.lateral_squared.clone(),
                edge_direction_sine_squared_bound: bounds.edge_sine_squared.clone(),
                direction_sine_bound: bounds.direction_sine_bound.clone(),
                proven_predicates: noise_predicates(effect),
            });
        }
        if canonical_used {
            if let Some(restored) = &raw.canonical.canonical_excess_over_pi {
                let declaration = &raw.canonical.restoration_declaration;
                let relation = decl_text(declaration, "canonical_relation")?;
                let deviation = decl_ratio(declaration, "deviation_upper_bound_radians")?;
                result.canonical_subturn_authority = Some(CanonicalSubturnAuthorityFactsV1 {
                    guarantee_law: "SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1".into(),
                    canonical_relation: relation,
                    canonical_reflex_excess_over_pi: restored.clone(),
                    hidden_edge_count: count,
                    max_subturn_q: q,
                    canonical_subturn_over_pi: rational_div(
                        restored,
                        &Rational::integer(IBig::from(count + 1)),
                    )
                    .unwrap(),
                    raw_residual_upper_bound_radians: deviation,
                    proven_predicates: canonical_subturn_predicates(),
                });
            }
        }
        result.law_refusals = CanonicalLawRefusalFactsV1 {
            noise_refusal: if canonical_steps(
                noise.canonical.as_ref(),
                q,
                raw.selection.source_hidden_count + 1,
            ) {
                noise.refusal.map(|r| r.as_str().into())
            } else {
                None
            },
            rays_refusal,
        };
        Ok(result)
    }
    fn bind(
        &mut self,
        m: &ProducedRawMetricV1,
        ideal: &[VectorExprV1],
        turn: i8,
        q: usize,
        reasons: &[Option<String>],
        needed: &[bool],
        lifted: bool,
        _use_declarations: bool,
    ) -> Result<(OwnedBindingResultV1, WindowLaw, Option<String>)> {
        let mut primal = vec![];
        for ray in ideal {
            primal.push(self.laws.primal(m, ray)?);
        }
        let mut literals = vec![];
        for ray in &primal {
            let c = self.laws.constructor();
            let x = c.raw_id(&ray[0]).map_err(|e| Failure::Raw(e.into()))?;
            let y = c.raw_id(&ray[1]).map_err(|e| Failure::Raw(e.into()))?;
            let tree = |id| {
                DensityLiteralTreeV1::from_expression(c.graph(), id).map_err(|e| {
                    Failure::Raw(crate::density_construct_v1::ConstructErrorV1::from(e).into())
                })
            };
            literals.push([tree(x)?, tree(y)?]);
        }
        let mut covectors = vec![];
        for ray in &primal {
            covectors.push(self.laws.covector(m, ray)?);
        }
        let inverse = &m.declaration.inverse_gram;
        let geometry = OwnedFanGeometryV1 {
            inverse_gram: [
                [inverse[0].clone(), inverse[1].clone()],
                [inverse[1].clone(), inverse[2].clone()],
            ],
            owner_orientation: m.declaration.owner_orientation_sign,
            turn_orientation: turn,
            ideal: covectors,
            q,
            window_law: self.law,
        };
        let registry = self.laws.registry();
        let mut laws = OwnedDensityLawsV1::new(
            self.laws.constructor(),
            geometry,
            registry,
            self.limits.finite_expand,
        )
        .map_err(|e| Failure::Raw(RawLawsFailureV1::Arithmetic(e)))?;
        let caps = self.limits.compile_limits();
        let memo = &mut self.memo;
        let delta = &mut self.delta;
        let shells = &mut self.shells;
        let metric = m.declaration.clone();
        let canon_ids = self.canon.clone();
        let report = density_bind_v1::bind_with_attempts(self.law, lifted, |law, adaptive_only| {
            let key = DensityAttemptKeyV1 {
                canon_ids: canon_ids.clone(),
                gram: metric.gram.clone(),
                inverse_gram: metric.inverse_gram.clone(),
                owner_orientation_sign: metric.owner_orientation_sign,
                ideal_primal_literals: literals.clone(),
                orientation: turn,
                q,
                binding_reasons: reasons.to_vec(),
                adaptive_only,
                window_law: law,
            };
            if let Some(entry) = memo.get(&key) {
                shells.extend(entry.shell_events.iter().cloned());
                return OwnedBindingReportV1 {
                    result: Ok(entry.binding_result()),
                    windows: None,
                    legacy_budgets: vec![],
                    adaptive_budgets: Default::default(),
                };
            }
            laws.set_window_law(law);
            let result = density_bind_v1::bind_owned_fan_once(
                &mut laws,
                turn,
                q,
                needed,
                adaptive_only,
                &caps,
            );
            let bundle = laws.take_shell_outcomes();
            shells.extend(bundle.iter().cloned());
            if let Ok(bound) = &result.result {
                let receipt = DensityAttemptReceiptV1 {
                    key: key.clone(),
                    binding_certificates: bound.certificates.clone(),
                    authority: bound.authority.clone(),
                    shell_events: bundle,
                };
                delta.entries.push(receipt.clone());
                memo.insert(key, receipt);
            }
            result
        });
        let registry = laws.take_values();
        drop(laws);
        self.laws.replace_registry(registry);
        for attempt in &report.attempts {
            for budget in attempt.report.legacy_budgets.iter().flatten() {
                self.prices.push(price("PREPARATION", budget));
            }
            if let Some(budget) = &attempt.report.adaptive_budgets.preparation {
                self.prices.push(price("PREPARATION", budget));
            }
            if let Some(budget) = &attempt.report.adaptive_budgets.search {
                self.prices.push(price("SEARCH", budget));
            }
        }
        let law = report.attempts.last().unwrap().window_law;
        let refusal = report.band_refusal.map(|r| r.text());
        let result = report
            .attempts
            .into_iter()
            .last()
            .unwrap()
            .report
            .result
            .map_err(Failure::Bind)?;
        Ok((result, law, refusal))
    }
}
fn canonical_steps(canonical: Option<&Rational>, q: usize, steps: usize) -> bool {
    canonical.is_some_and(|c| {
        rational_mul(c, &Rational::integer(IBig::from(q))) == Rational::integer(IBig::from(steps))
    })
}
fn price(phase: &str, budget: &crate::farey::WorkBudget) -> DensityPriceArticleV1 {
    DensityPriceArticleV1 {
        phase: phase.into(),
        cap: budget.cap.clone(),
        counters: budget
            .counters()
            .into_iter()
            .map(|(k, v)| (k.into(), v))
            .collect(),
    }
}
fn decl_bool(rows: &DeclarationRowsV1, key: &str, default: Option<bool>) -> Result<bool> {
    match declaration_value(rows, key) {
        Some(PrimitiveFactV1::Bool(v)) => Ok(*v),
        None => default.ok_or(Failure::Declaration(
            "DENSITY_RAW_DECLARATION_FIELD_MISSING",
        )),
        _ => Err(Failure::Declaration(
            "DENSITY_RAW_DECLARATION_BOOL_REQUIRED",
        )),
    }
}
fn decl_text(rows: &DeclarationRowsV1, key: &str) -> Result<String> {
    match declaration_value(rows, key) {
        Some(PrimitiveFactV1::Text(v)) => Ok(v.clone()),
        _ => Err(Failure::Declaration(
            "DENSITY_RAW_DECLARATION_TEXT_REQUIRED",
        )),
    }
}
fn decl_ratio(rows: &DeclarationRowsV1, key: &str) -> Result<Rational> {
    declaration_value(rows, key)
        .and_then(PrimitiveFactV1::ratio)
        .ok_or(Failure::Declaration(
            "DENSITY_RAW_DECLARATION_RATIO_REQUIRED",
        ))
}
fn text_array(rows: &DeclarationRowsV1, key: &str) -> Result<Vec<String>> {
    match declaration_value(rows, key) {
        None => Ok(vec![]),
        Some(PrimitiveFactV1::Tuple(values)) => values
            .iter()
            .map(|v| match v {
                PrimitiveFactV1::Text(v) => Ok(v.clone()),
                _ => Err(Failure::Declaration("DENSITY_RAW_VERTEX_IDS_REQUIRED")),
            })
            .collect(),
        _ => Err(Failure::Declaration("DENSITY_RAW_VERTEX_IDS_REQUIRED")),
    }
}
fn coordinates(rows: &DeclarationRowsV1, key: &str) -> Result<Vec<RawNoisePointV1>> {
    let Some(value) = declaration_value(rows, key) else {
        return Ok(vec![]);
    };
    let PrimitiveFactV1::Tuple(values) = value else {
        return Err(Failure::Declaration("DENSITY_RAW_COORDINATES_REQUIRED"));
    };
    values
        .iter()
        .map(|value| {
            let PrimitiveFactV1::Tuple(row) = value else {
                return Err(Failure::Declaration("DENSITY_RAW_COORDINATE_ROW_REQUIRED"));
            };
            if row.len() != 3 {
                return Err(Failure::Declaration("DENSITY_RAW_COORDINATE_ROW_REQUIRED"));
            }
            let PrimitiveFactV1::Text(id) = &row[0] else {
                return Err(Failure::Declaration("DENSITY_RAW_COORDINATE_ID_REQUIRED"));
            };
            Ok(RawNoisePointV1 {
                vertex_id: id.clone(),
                x: row[1].ratio().ok_or(Failure::Declaration(
                    "DENSITY_RAW_COORDINATE_RATIO_REQUIRED",
                ))?,
                y: row[2].ratio().ok_or(Failure::Declaration(
                    "DENSITY_RAW_COORDINATE_RATIO_REQUIRED",
                ))?,
            })
        })
        .collect()
}
fn decode_noise(raw: &RawDensitySpecInputV1, binding_present: bool) -> Result<RawNoiseInputV1> {
    let rows = &raw.canonical.noise_policy_declaration;
    let scale = match declaration_value(rows, "base_lattice_scale") {
        Some(PrimitiveFactV1::Int(v)) => Some(
            UBig::try_from(v.clone())
                .map_err(|_| Failure::Declaration("DENSITY_RAW_LATTICE_SCALE_REQUIRED"))?,
        ),
        None => None,
        _ => return Err(Failure::Declaration("DENSITY_RAW_LATTICE_SCALE_REQUIRED")),
    };
    let bound = declaration_value(rows, "noise_direction_sine_bound")
        .map(|v| {
            v.ratio()
                .ok_or(Failure::Declaration("DENSITY_RAW_NOISE_BOUND_REQUIRED"))
        })
        .transpose()?;
    Ok(RawNoiseInputV1 {
        source_reflex_interval: raw.canonical.source_reflex_interval.clone(),
        canonical_excess_over_pi: raw.canonical.canonical_excess_over_pi.clone(),
        binding_present,
        source_coordinates_available: decl_bool(rows, "source_coordinates_available", Some(false))?,
        incoming_vertex_ids: text_array(rows, "incoming_directed_vertex_ids")?,
        outgoing_vertex_ids: text_array(rows, "outgoing_directed_vertex_ids")?,
        anchor_vertex_id: raw.source_geometry.incoming.anchor_vertex_id.clone(),
        source_coordinates: coordinates(rows, "source_coordinates")?,
        evaluation_coordinates: coordinates(rows, "evaluation_coordinates")?,
        source_gram: raw.source_geometry.metric.gram.clone(),
        base_lattice_scale: scale,
        direction_sine_bound: bound,
    })
}
fn rotation_table(canonical: &Rational, sectors: usize, q: usize) -> Option<Vec<[i32; 2]>> {
    if canonical != &Rational::new(IBig::ONE, UBig::from(2u8)).unwrap() {
        return None;
    }
    match (sectors, q) {
        (2, 2) | (2, 3) => Some(vec![[1, 1]]),
        (3, 4) | (3, 5) => Some(vec![[7, 4], [4, 7]]),
        (4, 6) => Some(vec![[12, 5], [1, 1], [5, 12]]),
        _ => None,
    }
}
fn lift_law(law: u8) -> &'static str {
    match law {
        0 => "EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1",
        1 => "EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1",
        _ => "EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1",
    }
}
fn lift_predicates(law: u8) -> Vec<String> {
    let mut names = vec![
        "SOURCE_SELECTION_CERTIFICATE_IMMUTABLE",
        "EFFECTIVE_COUNT_EXACTLY_FEASIBLE_IN_EVALUATION_GEOMETRY",
        "EFFECTIVE_COUNT_IS_MINIMAL",
    ];
    names.extend(match law {
        0 => vec!["SOURCE_COUNT_EXACTLY_INFEASIBLE_IN_EVALUATION_GEOMETRY"],
        1 => vec![
            "PREDECESSOR_COUNT_EXACTLY_AT_SUBTURN_LIMIT_IN_EVALUATION_GEOMETRY",
            "PREDECESSOR_FAN_HAS_IRRATIONAL_HIDDEN_DIRECTION",
        ],
        _ => vec![
            "SELECTOR_INTERVAL_IS_EXACTLY_CANONICAL",
            "PREDECESSOR_COUNT_EXACTLY_AT_SUBTURN_LIMIT_ON_CANONICAL_ANGLE",
            "PREDECESSOR_CANONICAL_FAN_HAS_IRRATIONAL_HIDDEN_DIRECTION",
            "EVALUATION_BINDING_NOISE_IS_RECORDED_WITH_ITS_EXACT_LATERAL_OFFSET_BOUND",
        ],
    });
    predicates(&names)
}
fn canonical_ray_predicates() -> Vec<String> {
    predicates(&[
        "SOURCE_SELECTION_CERTIFICATE_IMMUTABLE",
        "SELECTOR_INTERVAL_IS_EXACTLY_CANONICAL",
        "ROTATION_ROW_IS_THE_TABLE_ENTRY_OF_THE_CANONICAL_ANGLE",
        "BOUND_DIRECTIONS_EQUAL_THE_TABLE_ROTATIONS_OF_THE_INCOMING_SUPPORT",
        "EVERY_SECTOR_INCLUDING_THE_LAST_IS_EXACTLY_WITHIN_MAX_SUBTURN",
        "BOUND_DIRECTIONS_ARE_PRIMITIVE_RATIONAL_COVECTORS",
    ])
}
fn canonical_subturn_predicates() -> Vec<String> {
    predicates(&[
        "SOURCE_EQUAL_SUBTURN_FAN_VIOLATES_GUARANTEE_ON_SOURCE_SUPPORTS",
        "CANONICAL_SUBTURN_IS_EXACTLY_WITHIN_MAX_SUBTURN",
        "CANONICAL_RAYS_ARE_EXACT_ORDINAL_ROTATIONS_OF_THE_INCOMING_SUPPORT",
        "RESIDUAL_IS_BOUNDED_BY_THE_AUTHORING_INTENT_TOLERANCE",
    ])
}
fn noise_predicates(effect: &str) -> Vec<String> {
    let mut names = vec![
        "SELECTOR_INTERVAL_IS_EXACTLY_CANONICAL",
        "EVALUATION_TURN_SIGN_AND_COSINE_SQUARED_ARE_EXACT",
        "BINDING_LATERAL_OFFSET_BOUND_IS_EXACT_AND_WITHIN_ONE_LATTICE_CELL",
        "EDGE_DIRECTION_NOISE_IS_EXACT_AND_WITHIN_THE_DECLARED_SINE_BOUND",
        "TURN_NOISE_IS_WITHIN_THE_DECLARED_SINE_BOUND",
    ];
    names.extend(if effect == "CANONICAL_COUNT_LIFT" {
        vec![
            "CANONICAL_PREDECESSOR_COUNT_IS_EXACTLY_AT_SUBTURN_LIMIT",
            "CANONICAL_PREDECESSOR_FAN_HAS_IRRATIONAL_HIDDEN_DIRECTION",
        ]
    } else {
        vec![
            "EVALUATION_EQUAL_SUBTURN_FAN_VIOLATES_GUARANTEE_ON_EVALUATION_SUPPORTS",
            "CANONICAL_SUBTURN_IS_EXACTLY_AT_MAX_SUBTURN_WITH_RATIONAL_RAYS",
            "CANONICAL_RAYS_ARE_EXACT_ORDINAL_ROTATIONS_OF_THE_INCOMING_SUPPORT",
        ]
    });
    predicates(&names)
}

pub fn evaluate_raw_density_batch(input: &RawDensityBatchInputV1) -> DensityRawBatchReportV1 {
    evaluate_raw_density_batch_with_memo(input, &DensityAttemptMemoSnapshotV1::default()).report
}

/// Snapshot и delta принадлежат host; успешные префиксы остаются при любом отказе.
pub fn evaluate_raw_density_batch_with_memo(
    input: &RawDensityBatchInputV1,
    snapshot: &DensityAttemptMemoSnapshotV1,
) -> DensityMemoBatchResultV1 {
    let invalid = |detail: String| DensityMemoBatchResultV1 {
        report: DensityRawBatchReportV1 {
            results: vec![],
            internal_failure: Some(RawBatchInternalFailureV1 {
                spec_index: 0,
                detail,
                partial_spec_effects: RawDensitySpecResultV1::blank(&input.canon_ids.construction),
            }),
        },
        delta: DensityAttemptMemoSnapshotV1::default(),
    };
    if input.specs.iter().any(|spec| spec.spec_id.is_none())
        || input
            .specs
            .windows(2)
            .any(|pair| pair[0].spec_id >= pair[1].spec_id)
    {
        return invalid("raw density batch spec IDs must be sorted and unique".into());
    }
    if let Err(error) = input.canon_ids.validate_candidate() {
        return invalid(format!("{error:?}"));
    }
    let mut memo = HashMap::new();
    for receipt in &snapshot.entries {
        if receipt.key.canon_ids != input.canon_ids {
            return invalid("density attempt memo canon IDs mismatch".into());
        }
        if memo.insert(receipt.key.clone(), receipt.clone()).is_some() {
            return invalid("density attempt memo contains duplicate keys".into());
        }
    }
    let mut graph = Expressions::new();
    let mut constructor = ConstructorV1::new(&mut graph);
    let mut frontend = Frontend {
        laws: RawDensityLawsV1::new(&mut constructor, input.limits.finite_expand),
        canon: input.canon_ids.clone(),
        limits: input.limits.clone(),
        law: input.window_law,
        memo,
        delta: DensityAttemptMemoSnapshotV1::default(),
        prices: vec![],
        shells: vec![],
    };
    let mut report = DensityRawBatchReportV1 {
        results: vec![],
        internal_failure: None,
    };
    for (index, spec) in input.specs.iter().enumerate() {
        match frontend.evaluate(spec) {
            Ok(result) => {
                let failed = result.failure.is_some();
                report.results.push(result);
                if failed {
                    break;
                }
            }
            Err((error, effects)) => {
                report.internal_failure = Some(RawBatchInternalFailureV1 {
                    spec_index: index,
                    detail: error.internal_detail(),
                    partial_spec_effects: effects,
                });
                break;
            }
        }
    }
    DensityMemoBatchResultV1 {
        report,
        delta: frontend.delta,
    }
}

#[cfg(test)]
mod failure_contract_tests {
    use super::*;
    #[test]
    fn raw_arithmetic_maps_only_geometric_undecidable() {
        use crate::density_construct_v1::{ConstructErrorV1, OwnedSignErrorV1};
        use crate::density_expand_v1::{ExpandErrorV1, ExpandProfileV1, LazySignErrorV1};
        let undecidable = LawsErrorV1::Sign(LazySignErrorV1::Sign(OwnedSignErrorV1::Sign(
            crate::density_sign::SignFailure::Undecidable,
        )));
        let failure =
            named_failure(&Failure::Raw(RawLawsFailureV1::Arithmetic(undecidable))).unwrap();
        assert_eq!(failure.exception_type, "ReferenceGeometryError");
        assert_eq!(failure.outcome, "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE");
        for internal in [
            LawsErrorV1::Construction(ConstructErrorV1::Scalar(
                crate::scalar_v2::ScalarError::Profile(crate::scalar_v2::ProfileUnsupported {
                    code: crate::scalar_v2::ProfileCode::InverseClassLimit,
                    detail: "bounded inverse class".into(),
                }),
            )),
            LawsErrorV1::Value(
                crate::density_angle_value_v1::AngleValueErrorV1::UnregisteredOpaque,
            ),
            LawsErrorV1::Sign(LazySignErrorV1::Expand(ExpandErrorV1::Profile(
                ExpandProfileV1::TermLimit {
                    requested: 2,
                    maximum: 1,
                },
            ))),
        ] {
            assert!(named_failure(&Failure::Raw(RawLawsFailureV1::Arithmetic(internal))).is_none());
        }
    }
}
