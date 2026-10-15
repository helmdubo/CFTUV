use cftuv_density::density_angle_value_v1::AngleValueRegistryV1;
use cftuv_density::density_bind_v1::{compile_owned_fan, produce_principal_fan, RawFanGeometryV1};
use cftuv_density::density_construct_v1::{ConstructErrorV1, ConstructorV1, OwnedExprV1};
use cftuv_density::density_expand_v1::ExpandLimits;
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_laws_v1::*;
use cftuv_density::fan_compile::CompileLimits;
use cftuv_density::farey::PrimitiveVector;
use cftuv_density::geometry::WindowLaw;
use cftuv_density::scalar_v2::{ProfileCode as ScalarProfile, ScalarError};
use cftuv_density::{IBig, Rational, UBig};
fn q(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn number(c: &mut ConstructorV1<'_>, n: i32) -> OwnedExprV1 {
    c.rational(&q(n, 1)).unwrap()
}
fn limits() -> ExpandLimits {
    ExpandLimits {
        max_terms: 4096,
        max_polynomial_exponent: 16,
        max_rational_power_bits: 65536,
    }
}
fn raw(qv: usize, owner: i8, turn: i8) -> RawFanGeometryV1 {
    RawFanGeometryV1 {
        inverse_gram: [[q(1, 1), q(0, 1)], [q(0, 1), q(1, 1)]],
        incoming: [q(1, 1), q(0, 1)],
        outgoing: [q(0, 1), q(1, 1)],
        owner_orientation: owner,
        turn_orientation: turn,
        hidden_count: 1,
        q: qv,
        window_law: WindowLaw::Voronoi,
    }
}
fn geometry(c: &mut ConstructorV1<'_>) -> OwnedFanGeometryV1 {
    OwnedFanGeometryV1 {
        inverse_gram: [[q(1, 1), q(0, 1)], [q(0, 1), q(1, 1)]],
        owner_orientation: 1,
        turn_orientation: 1,
        ideal: vec![[number(c, 1), number(c, 0)], [number(c, 0), number(c, 1)]],
        q: 3,
        window_law: WindowLaw::Voronoi,
    }
}
#[test]
fn fresh_raw_q_fan_closes_windows_and_adaptive_controller_with_ordered_budgets() {
    let mut g = Expressions::new();
    let mut c = ConstructorV1::new(&mut g);
    let p = produce_principal_fan(&mut c, &raw(3, 1, 1), limits()).unwrap();
    let mut laws = OwnedDensityLawsV1::new(&mut c, p.geometry, p.values, limits()).unwrap();
    let report = compile_owned_fan(&mut laws, 1, 3, &CompileLimits::default());
    assert_eq!(report.windows.as_ref().unwrap().records.len(), 1);
    let compiled = report.compiled.unwrap();
    let authority = compiled.result.unwrap();
    assert_eq!(authority.minimal_common_height, UBig::ONE);
    assert_eq!(
        authority.bound_primitive_integer_vectors,
        vec![PrimitiveVector(IBig::ONE, IBig::ONE)]
    );
    assert!(compiled.budgets.preparation.is_some());
    assert!(compiled.budgets.search.is_some());
}
#[test]
fn owner_orientation_and_turn_orientation_are_independent_and_legacy_q4_closes() {
    use cftuv_density::{legacy_binding, window_build};
    let mut g = Expressions::new();
    let mut c = ConstructorV1::new(&mut g);
    let p = produce_principal_fan(&mut c, &raw(4, -1, -1), limits()).unwrap();
    let ideal = p.geometry.ideal.clone();
    let mut laws = OwnedDensityLawsV1::new(&mut c, p.geometry, p.values, limits()).unwrap();
    let windows = window_build::window_envelopes(&ideal, &mut laws).unwrap();
    let certificate = legacy_binding::legacy_density_certificate(
        &ideal,
        1,
        4,
        &windows.records[0],
        UBig::from(65536u32),
        &mut laws,
    );
    let certificate = certificate.result.unwrap().unwrap();
    assert_eq!(
        certificate.bound_primitive_integer_vector,
        PrimitiveVector(IBig::ONE, IBig::ONE)
    );
    assert!(legacy_binding::legacy_full_fan_valid(
        &ideal,
        -1,
        4,
        &windows.records,
        &[Some(certificate)],
        &mut laws
    )
    .unwrap());
    let cross = laws.cross(&ideal[0], &ideal[1]).unwrap();
    assert_eq!(laws.source_sign(&cross).unwrap(), -1);
}
#[test]
fn prepared_bind_routes_legacy_before_adaptive_and_keeps_budget_scopes() {
    use cftuv_density::density_bind_v1::bind_owned_fan_once;
    let mut g = Expressions::new();
    let mut c = ConstructorV1::new(&mut g);
    let p = produce_principal_fan(&mut c, &raw(4, 1, 1), limits()).unwrap();
    let mut laws = OwnedDensityLawsV1::new(&mut c, p.geometry, p.values, limits()).unwrap();
    let report = bind_owned_fan_once(&mut laws, 1, 4, &[true], false, &CompileLimits::default());
    let result = report.result.unwrap();
    assert!(result.authority.is_none());
    assert!(result.certificates[0].is_some());
    assert_eq!(report.legacy_budgets.len(), 1);
    assert!(report.legacy_budgets[0].is_some());
    assert!(report.adaptive_budgets.preparation.is_none());
    assert!(report.adaptive_budgets.search.is_none());
}
#[test]
fn actual_q5_boundary_uses_old_admission_and_positive_half_ring_law() {
    let mut g = Expressions::new();
    let mut c = ConstructorV1::new(&mut g);
    let left = [number(&mut c, 1), number(&mut c, 0)];
    let right = cftuv_density::density_trig_v1::owned_exact_q_trig(&mut c, 5).unwrap();
    let input = OwnedFanGeometryV1 {
        inverse_gram: [[q(1, 1), q(0, 1)], [q(0, 1), q(1, 1)]],
        owner_orientation: 1,
        turn_orientation: 1,
        ideal: vec![left.clone(), right.clone()],
        q: 5,
        window_law: WindowLaw::Voronoi,
    };
    let mut laws =
        OwnedDensityLawsV1::new(&mut c, input, AngleValueRegistryV1::new(), limits()).unwrap();
    assert!(laws.subturn_boundary(&left, &right, 5).unwrap());
    assert!(laws.subturn(&left, &right, 5).unwrap());
}
#[test]
fn old_interval_admission_precedes_successful_a2_composition() {
    let mut g = Expressions::new();
    let mut c = ConstructorV1::new(&mut g);
    let ratio = c.rational(&q(1, 64)).unwrap();
    let cosine_total = c.sqrt(&ratio).unwrap();
    let one = number(&mut c, 1);
    let sine_squared = c.sub(&one, &ratio).unwrap();
    let sine_total = c.sqrt(&sine_squared).unwrap();
    let total = c.raw_atan2(&sine_total, &cosine_total).unwrap();
    let angle = c.scaled(&total, &q(1, 2)).unwrap();
    let cosine = c.raw_cos(&angle).unwrap();
    let sine = c.raw_sin(&angle).unwrap();
    let mut values = AngleValueRegistryV1::new();
    let atom = cftuv_density::angle::AngleAtom::new(2, q(1, 64), 1).unwrap();
    values.register_ray(&c, &cosine, &sine, &atom, 1).unwrap();
    let exact_one = c.scaled(&cosine, &q(4, 3)).unwrap();
    assert!(c.numeric_value(&exact_one).unwrap().is_none());
    let zero = c.sub(&exact_one, &one).unwrap();
    let power = c.sqrt(&zero).unwrap();
    assert!(values.enclosure(&c, &power).is_ok());
    let input = geometry(&mut c);
    let mut laws = OwnedDensityLawsV1::new(&mut c, input, values, limits()).unwrap();
    let grid = laws.decimal_envelope(&exact_one).unwrap();
    let exact = cftuv_density::window_build::exact_decimal_interval(IBig::ONE);
    assert_eq!(grid.lower, exact.lower);
    assert_eq!(grid.upper, exact.upper);
    assert_eq!(
        grid.quantum,
        Some(cftuv_density::atlas_plan::DecimalQuantum::uniform(-27))
    );
    assert_eq!(
        exact.quantum,
        Some(cftuv_density::atlas_plan::DecimalQuantum::uniform(0))
    );
    assert_eq!(
        laws.decimal_envelope(&exact_one).unwrap().quantum,
        grid.quantum
    );
    for _ in 0..2 {
        assert!(matches!(
            laws.decimal_envelope(&power),
            Err(LawsErrorV1::LegacyEnvelopeUnproven)
        ));
    }
}
#[test]
fn sealed_nonzero_allows_zero_over_safe_opaque_but_does_not_bypass_numeric_inverse() {
    let mut g = Expressions::new();
    let mut c = ConstructorV1::new(&mut g);
    let zero = number(&mut c, 0);
    let cosine = c.raw_cos(&zero).unwrap();
    let proof = prove_nonzero(&mut c, &cosine, limits()).unwrap();
    let quotient = c.divide_with_nonzero_witness(&zero, &proof).unwrap();
    assert!(c
        .numeric_value(&quotient)
        .unwrap()
        .unwrap()
        .value()
        .terms()
        .is_empty());
    assert!(matches!(
        prove_nonzero(&mut c, &zero, limits()),
        Err(LawsErrorV1::ZeroDenominator)
    ));
    let mut roots = Vec::new();
    for n in [2, 3, 5] {
        let n = number(&mut c, n);
        roots.push(c.sqrt(&n).unwrap());
    }
    let three = c.add(&roots).unwrap();
    let four = c.integer_power(&three, 2).unwrap();
    let proof = prove_nonzero(&mut c, &four, limits()).unwrap();
    assert!(
        matches!(c.divide_with_nonzero_witness(&zero,&proof),Err(ConstructErrorV1::Scalar(ScalarError::Profile(p))) if p.code==ScalarProfile::InverseClassLimit)
    );
    let pi = c.pi().unwrap();
    let partial = c.formal_reciprocal(&pi).unwrap();
    let proof = prove_nonzero(&mut c, &partial, limits()).unwrap();
    assert!(matches!(
        c.divide_with_nonzero_witness(&zero, &proof),
        Err(ConstructErrorV1::Profile(
            cftuv_density::density_construct_v1::ProfileCode::PartialDomainDiscard
        ))
    ));
}
#[test]
fn witness_keeps_arena_token_alive_and_cannot_cross_sequential_constructor() {
    let mut g = Expressions::new();
    let proof = {
        let mut c = ConstructorV1::new(&mut g);
        let one = number(&mut c, 1);
        prove_nonzero(&mut c, &one, limits()).unwrap()
    };
    let mut c = ConstructorV1::new(&mut g);
    let zero = number(&mut c, 0);
    assert!(matches!(
        c.divide_with_nonzero_witness(&zero, &proof),
        Err(ConstructErrorV1::ForeignArena)
    ));
}
