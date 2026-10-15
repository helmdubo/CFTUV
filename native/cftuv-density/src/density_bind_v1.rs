//! Первый замкнутый consumer: raw rational covectors -> owned principal fan -> общий controller.
//! Не принимает старый ideal AST, Python callback или готовую authority.
use crate::angle::AngleAtom;
use crate::density_angle_value_v1::AngleValueRegistryV1;
use crate::density_construct_v1::{self, ConstructorV1, OwnedExprV1, OwnedFormV1, VectorExprV1};
use crate::density_expand_v1::{self, ExpandLimits};
use crate::density_laws_v1::{LawsErrorV1, LawsResultV1, OwnedDensityLawsV1, OwnedFanGeometryV1};
use crate::density_trig_v1;
use crate::fan_compile::{CompileLimits, CompileReport};
use crate::geometry::WindowLaw;
use crate::window_build::{self, BuiltWindows, WindowBuildFailure};
use crate::{IBig, Rational, UBig};

pub struct RawFanGeometryV1 {
    pub inverse_gram: [[Rational; 2]; 2],
    pub incoming: [Rational; 2],
    pub outgoing: [Rational; 2],
    pub owner_orientation: i8,
    pub turn_orientation: i8,
    pub hidden_count: usize,
    pub q: usize,
    pub window_law: WindowLaw,
}
pub struct ProducedFanV1 {
    pub geometry: OwnedFanGeometryV1,
    pub values: AngleValueRegistryV1,
}
fn whole_rational(c: &ConstructorV1<'_>, v: &OwnedExprV1) -> LawsResultV1<Rational> {
    let (n, d) = c
        .numeric_value(v)?
        .and_then(|v| v.value().as_rational())
        .ok_or(LawsErrorV1::RationalCandidateRequired)?;
    Ok(Rational::new(n, d).expect("canonical Scalar denominator"))
}
pub fn produce_principal_fan(
    c: &mut ConstructorV1<'_>,
    raw: &RawFanGeometryV1,
    limits: ExpandLimits,
) -> LawsResultV1<ProducedFanV1> {
    if raw.hidden_count > 5
        || !(2..=6).contains(&raw.q)
        || !matches!(raw.owner_orientation, -1 | 1)
        || !matches!(raw.turn_orientation, -1 | 1)
    {
        return Err(LawsErrorV1::InvalidInput(
            "principal fan profile H0..5/q2..6/orientations",
        ));
    }
    let g = &raw.inverse_gram;
    let matrix = [
        [c.rational(&g[0][0])?, c.rational(&g[0][1])?],
        [c.rational(&g[1][0])?, c.rational(&g[1][1])?],
    ];
    let incoming = [c.rational(&raw.incoming[0])?, c.rational(&raw.incoming[1])?];
    let outgoing = [c.rational(&raw.outgoing[0])?, c.rational(&raw.outgoing[1])?];
    let ni = density_construct_v1::dot(c, &matrix, &incoming, &incoming)?;
    let no = density_construct_v1::dot(c, &matrix, &outgoing, &outgoing)?;
    let dot = density_construct_v1::dot(c, &matrix, &incoming, &outgoing)?;
    let cross =
        density_construct_v1::oriented_cross(c, &incoming, &outgoing, raw.owner_orientation)?;
    if density_expand_v1::lazy_sign(c, &cross, limits)?.result.sign != raw.turn_orientation {
        return Err(LawsErrorV1::InvalidInput(
            "ordered support normals do not realize owner-sector turn",
        ));
    }
    let incoming_unit = density_construct_v1::rational_norm_unit(c, &matrix, &incoming)?;
    let outgoing_unit = density_construct_v1::rational_norm_unit(c, &matrix, &outgoing)?;
    let mut ideal = vec![incoming_unit.clone()];
    let mut values = AngleValueRegistryV1::new();
    if raw.hidden_count > 0 {
        let dot2 = c.mul(&[dot.clone(), dot.clone()])?;
        let norms = c.mul(&[ni, no])?;
        let cosine_squared = c.div(&dot2, &norms)?;
        let ratio = whole_rational(c, &cosine_squared)?;
        let one = c.rational(&Rational::integer(IBig::ONE))?;
        let excess = c.sub(&cosine_squared, &one)?;
        if density_expand_v1::lazy_sign(c, &cosine_squared, limits)?
            .result
            .sign
            < 0
            || density_expand_v1::lazy_sign(c, &excess, limits)?
                .result
                .sign
                >= 0
        {
            return Err(LawsErrorV1::InvalidInput(
                "Density principal turn must lie strictly between 0 and pi",
            ));
        }
        let sign = density_expand_v1::lazy_sign(c, &dot, limits)?.result.sign;
        let cosine_root = c.sqrt(&cosine_squared)?;
        let cosine_total = c.scaled(&cosine_root, &Rational::integer(IBig::from(sign)))?;
        let sine_squared = c.sub(&one, &cosine_squared)?;
        let sine_total = c.sqrt(&sine_squared)?;
        let principal = c.raw_atan2(&sine_total, &cosine_total)?;
        let basis =
            density_construct_v1::oriented_basis(c, &matrix, &incoming, raw.owner_orientation)?;
        let n = (raw.hidden_count + 1) as u8;
        let atom = AngleAtom::new(n, ratio.clone(), sign)
            .map_err(|_| LawsErrorV1::InvalidInput("principal angle profile"))?;
        for j in 1..n {
            let pair =
                if let Some(pair) = density_trig_v1::owned_principal_ray(c, &ratio, sign, n, j)? {
                    pair
                } else {
                    let angle = c.scaled(
                        &principal,
                        &Rational::new(IBig::from(j), UBig::from(n)).unwrap(),
                    )?;
                    [c.raw_cos(&angle)?, c.raw_sin(&angle)?]
                };
            // Только raw компоненты нуждаются в semantic atom; finite roots композируются сами.
            if matches!(c.form(&pair[0])?, OwnedFormV1::Cos(_))
                || matches!(c.form(&pair[1])?, OwnedFormV1::Sin(_))
            {
                values.register_ray(c, &pair[0], &pair[1], &atom, j)?;
            }
            let sine = c.scaled(
                &pair[1],
                &Rational::integer(IBig::from(raw.turn_orientation)),
            )?;
            let mut coordinates = Vec::new();
            for axis in 0..2 {
                let a = c.mul(&[pair[0].clone(), basis[0][axis].clone()])?;
                let b = c.mul(&[sine.clone(), basis[1][axis].clone()])?;
                coordinates.push(c.add(&[a, b])?);
            }
            ideal.push([coordinates.remove(0), coordinates.remove(0)]);
        }
    }
    ideal.push(outgoing_unit);
    Ok(ProducedFanV1 {
        geometry: OwnedFanGeometryV1 {
            inverse_gram: raw.inverse_gram.clone(),
            owner_orientation: raw.owner_orientation,
            turn_orientation: raw.turn_orientation,
            ideal,
            q: raw.q,
            window_law: raw.window_law,
        },
        values,
    })
}

#[derive(Clone, Debug)]
pub struct OwnedFanCompileReportV1 {
    pub windows: Result<BuiltWindows<VectorExprV1>, WindowBuildFailure<LawsErrorV1>>,
    pub compiled: Option<CompileReport<LawsErrorV1>>,
}
/// Сначала window controller; затем тот же prepared controller и его ordered budgets.
pub fn compile_owned_fan(
    laws: &mut OwnedDensityLawsV1<'_, '_>,
    orientation: i8,
    q: usize,
    limits: &CompileLimits,
) -> OwnedFanCompileReportV1 {
    let ideal = laws.ideal().to_vec();
    let windows = window_build::window_envelopes(&ideal, laws);
    let compiled = windows.as_ref().ok().map(|windows| {
        crate::fan_compile::compile_prepared(&ideal, orientation, q, &windows.records, limits, laws)
    });
    OwnedFanCompileReportV1 { windows, compiled }
}

#[derive(Clone, Debug)]
pub enum OwnedBindingFailureV1 {
    Windows(WindowBuildFailure<LawsErrorV1>),
    Legacy(crate::legacy_binding::LegacyBindingFailure<LawsErrorV1>),
    Adaptive(crate::fan_compile::FanCompileFailure<LawsErrorV1>),
    Input(&'static str),
}
#[derive(Clone, Debug)]
pub struct OwnedBindingResultV1 {
    pub certificates: Vec<Option<crate::legacy_binding::LegacyCertificate>>,
    pub authority: Option<crate::fan_compile::PreparedFanAuthority>,
}
#[derive(Clone, Debug)]
pub struct OwnedBindingReportV1 {
    pub result: Result<OwnedBindingResultV1, OwnedBindingFailureV1>,
    pub windows: Option<BuiltWindows<VectorExprV1>>,
    // Порядок B(w) ordinal budgets не теряется; adaptive budgets отдельные source scopes.
    pub legacy_budgets: Vec<Option<crate::farey::WorkBudget>>,
    pub adaptive_budgets: crate::fan_compile::CompileBudgets,
}

/// Именованные исходы полосы; внутренние ошибки арифметики сюда не допускаются.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BandRefusalV1 {
    Invalid(crate::fan_prepare::PrepareInvalid),
    OrdinalEnvelopesOverlap,
    ChartUnrepresentable,
    LegacyEnvelopeUnproven,
    CertifiedPredicateUndecidable,
    Work(crate::farey::WorkBudgetExhausted),
    RationalAuthorityExhausted,
    TerminationBoxes {
        hidden_rays: usize,
        at_limit: crate::fan_prepare::LimitDiagnostic,
        halvings: usize,
    },
}
impl BandRefusalV1 {
    pub fn text(&self) -> String {
        match self {
            Self::Invalid(reason) => format!("AdaptiveDensityFanInvalid: {}", reason.message()),
            Self::OrdinalEnvelopesOverlap => "AdaptiveDensityFanInvalid: ordinal rational envelopes overlap".into(),
            Self::ChartUnrepresentable => "DensityWindowChartUnrepresentable: DENSITY_WINDOW_CHART_UNREPRESENTABLE".into(),
            Self::LegacyEnvelopeUnproven => "DirectionBindingCertificateUnproven: привязка: сертификат не доказан (BINDING_INSIDE_OWN_ORDINAL_WINDOW)".into(),
            Self::CertifiedPredicateUndecidable => "ReferenceGeometryError: Density A exact sign is not certified without generic factorization".into(),
            Self::Work(reason) => format!("DensityRationalAuthorityExhausted: {reason}"),
            Self::RationalAuthorityExhausted => "DensityRationalAuthorityExhausted: DENSITY_RATIONAL_AUTHORITY_EXHAUSTED".into(),
            Self::TerminationBoxes { hidden_rays, at_limit, halvings } => format!("DensityTerminationBoxesExhausted: termination box refinement exhausted after {halvings} halvings for {hidden_rays} hidden rays: no feasible box was witnessed (ideal fan exactly at the subturn limit: {})", at_limit.text()),
        }
    }
}
fn band_refusal(failure: &OwnedBindingFailureV1) -> Option<BandRefusalV1> {
    use crate::fan_compile::FanCompileFailure as F;
    use crate::fan_prepare::FanPrepareFailure as P;
    use crate::farey::FareyFailure as S;
    match failure {
        OwnedBindingFailureV1::Windows(WindowBuildFailure::OrdinalEnvelopesOverlap) => {
            Some(BandRefusalV1::OrdinalEnvelopesOverlap)
        }
        OwnedBindingFailureV1::Windows(WindowBuildFailure::ChartUnrepresentable) => {
            Some(BandRefusalV1::ChartUnrepresentable)
        }
        OwnedBindingFailureV1::Adaptive(F::Preparation(P::Invalid(reason)) | F::Height(reason)) => {
            Some(BandRefusalV1::Invalid(*reason))
        }
        OwnedBindingFailureV1::Adaptive(
            F::Preparation(P::Work(reason)) | F::Search(S::Work(reason)),
        ) => Some(BandRefusalV1::Work(reason.clone())),
        OwnedBindingFailureV1::Adaptive(F::Preparation(P::TerminationBoxesExhausted {
            hidden_rays,
            at_limit,
            halvings,
        })) => Some(BandRefusalV1::TerminationBoxes {
            hidden_rays: *hidden_rays,
            at_limit: *at_limit,
            halvings: *halvings,
        }),
        OwnedBindingFailureV1::Adaptive(F::RationalAuthorityExhausted) => {
            Some(BandRefusalV1::RationalAuthorityExhausted)
        }
        OwnedBindingFailureV1::Windows(WindowBuildFailure::Callback(error))
        | OwnedBindingFailureV1::Adaptive(
            F::Preparation(P::Callback(error))
            | F::Segments(crate::atlas_plan::AtlasPlanFailure::Callback(error))
            | F::Search(S::Predicate(error)),
        ) => laws_refusal(error),
        _ => None,
    }
}
fn laws_refusal(error: &LawsErrorV1) -> Option<BandRefusalV1> {
    match error {
        LawsErrorV1::ChartUnrepresentable => Some(BandRefusalV1::ChartUnrepresentable),
        LawsErrorV1::LegacyEnvelopeUnproven => Some(BandRefusalV1::LegacyEnvelopeUnproven),
        LawsErrorV1::Sign(crate::density_expand_v1::LazySignErrorV1::Sign(
            crate::density_construct_v1::OwnedSignErrorV1::Sign(
                crate::density_sign::SignFailure::Undecidable,
            ),
        )) => Some(BandRefusalV1::CertifiedPredicateUndecidable),
        _ => None,
    }
}
/// Только известный source отказ; внутренний профиль не превращается в геометрический исход.
pub fn binding_failure_text(failure: &OwnedBindingFailureV1) -> Option<String> {
    match failure {
        OwnedBindingFailureV1::Legacy(crate::legacy_binding::LegacyBindingFailure::Work(
            reason,
        )) => Some(BandRefusalV1::Work(reason.clone()).text()),
        OwnedBindingFailureV1::Legacy(crate::legacy_binding::LegacyBindingFailure::EmptyWindow) => {
            Some("AdaptiveDensityFanInvalid: legacy Density window is empty".into())
        }
        OwnedBindingFailureV1::Legacy(crate::legacy_binding::LegacyBindingFailure::Callback(
            error,
        )) => laws_refusal(error).map(|r| r.text()),
        _ => band_refusal(failure).map(|refusal| refusal.text()),
    }
}
#[derive(Clone, Debug)]
pub struct BindingAttemptV1 {
    pub window_law: WindowLaw,
    pub adaptive_only: bool,
    pub report: OwnedBindingReportV1,
}
#[derive(Clone, Debug)]
pub struct DensityFanBindingReportV1 {
    /// Порядок попыток и цена отказавшей полосы сохраняются.
    pub attempts: Vec<BindingAttemptV1>,
    pub band_refusal: Option<BandRefusalV1>,
}
impl DensityFanBindingReportV1 {
    pub fn result(&self) -> &Result<OwnedBindingResultV1, OwnedBindingFailureV1> {
        &self
            .attempts
            .last()
            .expect("binding always attempts a law")
            .report
            .result
    }
}
/// Внешний controller окна; derived reasons/lifted пока принадлежат raw frontend.
pub fn bind_density_fan(
    laws: &mut OwnedDensityLawsV1<'_, '_>,
    orientation: i8,
    q: usize,
    binding_required: &[bool],
    lifted: bool,
    law: WindowLaw,
    limits: &CompileLimits,
) -> DensityFanBindingReportV1 {
    bind_with_attempts(law, lifted, |window_law, adaptive_only| {
        laws.set_window_law(window_law);
        bind_owned_fan_once(
            laws,
            orientation,
            q,
            binding_required,
            adaptive_only,
            limits,
        )
    })
}
pub(crate) fn bind_with_attempts(
    law: WindowLaw,
    lifted: bool,
    mut build: impl FnMut(WindowLaw, bool) -> OwnedBindingReportV1,
) -> DensityFanBindingReportV1 {
    let adaptive_only = lifted || law != WindowLaw::Voronoi;
    let first = build(law, adaptive_only);
    let refusal = if law == WindowLaw::NarrowBand {
        first.result.as_ref().err().and_then(band_refusal)
    } else {
        None
    };
    let mut attempts = vec![BindingAttemptV1 {
        window_law: law,
        adaptive_only,
        report: first,
    }];
    if refusal.is_some() {
        attempts.push(BindingAttemptV1 {
            window_law: WindowLaw::Voronoi,
            adaptive_only: lifted,
            report: build(WindowLaw::Voronoi, lifted),
        });
    }
    DensityFanBindingReportV1 {
        attempts,
        band_refusal: refusal,
    }
}
/// Prepared consumer, не raw-spec facade: binding_required — уже выбранные controller facts.
/// Внешний raw-spec producer вычислит причины сам; Python ideal сюда не импортируется.
pub fn bind_owned_fan_once(
    laws: &mut OwnedDensityLawsV1<'_, '_>,
    orientation: i8,
    q: usize,
    binding_required: &[bool],
    adaptive_only: bool,
    limits: &CompileLimits,
) -> OwnedBindingReportV1 {
    use crate::legacy_binding::{legacy_density_certificate, legacy_full_fan_valid};
    let mut report = OwnedBindingReportV1 {
        result: Err(OwnedBindingFailureV1::Input("pending")),
        windows: None,
        legacy_budgets: Vec::new(),
        adaptive_budgets: crate::fan_compile::CompileBudgets::default(),
    };
    let ideal = laws.ideal().to_vec();
    let hidden = ideal.len().saturating_sub(2);
    if binding_required.len() != hidden {
        report.result = Err(OwnedBindingFailureV1::Input("binding reason cardinality"));
        return report;
    }
    if !adaptive_only && hidden == 0 {
        report.result = Ok(OwnedBindingResultV1 {
            certificates: vec![],
            authority: None,
        });
        return report;
    }
    let windows = match window_build::window_envelopes(&ideal, laws) {
        Ok(value) => value,
        Err(e) => {
            report.result = Err(OwnedBindingFailureV1::Windows(e));
            return report;
        }
    };
    let mut certificates = vec![None; hidden];
    if !adaptive_only {
        for ordinal in 1..=hidden {
            let legacy = legacy_density_certificate(
                &ideal,
                ordinal,
                q,
                &windows.records[ordinal - 1],
                limits.preparation_work_cap.clone(),
                laws,
            );
            report.legacy_budgets.push(legacy.budget);
            match legacy.result {
                Ok(value) => certificates[ordinal - 1] = value,
                Err(e) => {
                    report.windows = Some(windows);
                    report.result = Err(OwnedBindingFailureV1::Legacy(e));
                    return report;
                }
            }
        }
        let valid = match legacy_full_fan_valid(
            &ideal,
            orientation,
            q,
            &windows.records,
            &certificates,
            laws,
        ) {
            Ok(value) => value,
            Err(e) => {
                report.windows = Some(windows);
                report.result = Err(OwnedBindingFailureV1::Legacy(e));
                return report;
            }
        };
        if valid
            && binding_required
                .iter()
                .zip(&certificates)
                .all(|(required, certificate)| !required || certificate.is_some())
        {
            report.windows = Some(windows);
            report.result = Ok(OwnedBindingResultV1 {
                certificates,
                authority: None,
            });
            return report;
        }
    }
    let adaptive = crate::fan_compile::compile_prepared(
        &ideal,
        orientation,
        q,
        &windows.records,
        limits,
        laws,
    );
    report.adaptive_budgets = adaptive.budgets;
    report.windows = Some(windows);
    report.result = adaptive
        .result
        .map(|authority| OwnedBindingResultV1 {
            certificates,
            authority: Some(authority),
        })
        .map_err(OwnedBindingFailureV1::Adaptive);
    report
}

#[cfg(test)]
mod outer_tests {
    use super::*;
    fn report(
        result: Result<OwnedBindingResultV1, OwnedBindingFailureV1>,
        cap: u8,
    ) -> OwnedBindingReportV1 {
        OwnedBindingReportV1 {
            result,
            windows: None,
            legacy_budgets: vec![Some(crate::farey::WorkBudget::new(UBig::from(cap)))],
            adaptive_budgets: crate::fan_compile::CompileBudgets {
                preparation: Some(crate::farey::WorkBudget::new(UBig::from(cap))),
                search: None,
            },
        }
    }
    fn success() -> Result<OwnedBindingResultV1, OwnedBindingFailureV1> {
        Ok(OwnedBindingResultV1 {
            certificates: vec![],
            authority: None,
        })
    }
    #[test]
    fn band_refusal_retains_attempt_order_and_lift_policy() {
        for lifted in [false, true] {
            for fallback_success in [false, true] {
                let mut calls = vec![];
                let result = bind_with_attempts(WindowLaw::NarrowBand, lifted, |law, adaptive| {
                    calls.push((law, adaptive));
                    if law == WindowLaw::NarrowBand {
                        report(
                            Err(OwnedBindingFailureV1::Windows(
                                WindowBuildFailure::OrdinalEnvelopesOverlap,
                            )),
                            3,
                        )
                    } else if fallback_success {
                        report(success(), 7)
                    } else {
                        report(Err(OwnedBindingFailureV1::Input("fallback failure")), 7)
                    }
                });
                assert_eq!(
                    calls,
                    vec![(WindowLaw::NarrowBand, true), (WindowLaw::Voronoi, lifted)]
                );
                assert_eq!(result.result().is_ok(), fallback_success);
                assert_eq!(
                    result.attempts[0]
                        .report
                        .adaptive_budgets
                        .preparation
                        .as_ref()
                        .unwrap()
                        .cap,
                    UBig::from(3u8)
                );
                assert_eq!(
                    result.attempts[1].report.legacy_budgets[0]
                        .as_ref()
                        .unwrap()
                        .cap,
                    UBig::from(7u8)
                );
                assert_eq!(
                    result.band_refusal.unwrap().text(),
                    "AdaptiveDensityFanInvalid: ordinal rational envelopes overlap"
                );
            }
        }
    }
    #[test]
    fn internal_error_does_not_authorize_fallback() {
        let mut calls = 0;
        let result = bind_with_attempts(WindowLaw::NarrowBand, false, |_, _| {
            calls += 1;
            report(
                Err(OwnedBindingFailureV1::Input("internal profile failure")),
                1,
            )
        });
        assert_eq!(calls, 1);
        assert!(result.band_refusal.is_none());
        assert!(result.result().is_err());
    }
    #[test]
    fn geometric_undecidable_retries_but_opaque_and_profile_errors_propagate() {
        let geometric = LawsErrorV1::Sign(crate::density_expand_v1::LazySignErrorV1::Sign(
            crate::density_construct_v1::OwnedSignErrorV1::Sign(
                crate::density_sign::SignFailure::Undecidable,
            ),
        ));
        let internal = [
            LawsErrorV1::Value(
                crate::density_angle_value_v1::AngleValueErrorV1::UnregisteredOpaque,
            ),
            LawsErrorV1::Construction(crate::density_construct_v1::ConstructErrorV1::Profile(
                crate::density_construct_v1::ProfileCode::PartialDomainDiscard,
            )),
        ];
        for (error, fallback) in
            std::iter::once((geometric, true)).chain(internal.into_iter().map(|e| (e, false)))
        {
            let mut calls = 0;
            let result = bind_with_attempts(WindowLaw::NarrowBand, false, |law, _| {
                calls += 1;
                if law == WindowLaw::Voronoi {
                    report(success(), 7)
                } else {
                    report(
                        Err(OwnedBindingFailureV1::Windows(
                            WindowBuildFailure::Callback(error.clone()),
                        )),
                        3,
                    )
                }
            });
            assert_eq!(calls, if fallback { 2 } else { 1 });
            assert_eq!(result.result().is_ok(), fallback);
            assert_eq!(result.band_refusal.is_some(), fallback);
        }
    }
}
