use cftuv_density::decimal_projection::{project, width_from_units, ProjectionError};
use cftuv_density::legacy_interval::LegacyInterval;
use cftuv_density::legacy_mpf::{Mpf, Precision, Rounding};
use cftuv_density::IBig;
fn point(n: i64) -> LegacyInterval {
    let v = Mpf::from_man_exp(IBig::from(n), 0, None, Rounding::TowardZero).unwrap();
    LegacyInterval::new(v.clone(), v).unwrap()
}
#[test]
fn source_grid_straw_and_zero_quantum_are_preserved() {
    let p = Precision::new(160).unwrap();
    let value = point(1657).div(&point(5120), p).unwrap();
    let out = project(&value, 28).unwrap();
    assert_eq!(out.lower.text(), "0.323632812499999999999999999");
    assert_eq!(out.upper.text(), "0.323632812500000000000000001");
    assert_eq!(out.width.text(), "2E-27");
    assert!(!out.width_rounded);
    let zero = project(&LegacyInterval::zero(), 28).unwrap();
    assert_eq!(zero.lower.text(), "0E-27");
    assert_eq!(zero.width.text(), "0E-27");
}
#[test]
fn explicit_width_precision_handles_half_even_ties_and_carry() {
    for (upper, expected, rounded) in [
        (125, "1.2E+2", true),
        (135, "1.4E+2", true),
        (999, "1.0E+3", true),
        (120, "1.2E+2", false),
    ] {
        let value = LegacyInterval::new(Mpf::zero(), point(upper).upper().clone()).unwrap();
        let result = project(&value, 2).unwrap();
        assert_eq!(result.width.text(), expected);
        assert_eq!(result.width_rounded, rounded);
    }
    assert_eq!(
        project(&LegacyInterval::entire(), 28),
        Err(ProjectionError::NonFinite)
    );
    assert_eq!(
        project(&LegacyInterval::zero(), 0),
        Err(ProjectionError::Precision(0))
    );
}

#[test]
fn integer_grid_width_retains_sign_quantum_and_half_even_rounding() {
    for (upper, expected, rounded) in [
        (125, "1.2E-25", true),
        (-125, "-1.2E-25", true),
        (-135, "-1.4E-25", true),
        (-999, "-1.0E-24", true),
        (-120, "-1.2E-25", false),
        (0, "0E-27", false),
    ] {
        let (width, did_round) = width_from_units(&IBig::ZERO, &IBig::from(upper), 2).unwrap();
        assert_eq!(width.text(), expected);
        assert_eq!(did_round, rounded);
    }
    assert_eq!(
        width_from_units(&IBig::from(77), &IBig::from(212), 2),
        width_from_units(&IBig::ZERO, &IBig::from(135), 2),
    );
    for invalid in [0, 1_000_001] {
        assert_eq!(
            width_from_units(&IBig::ZERO, &IBig::ZERO, invalid),
            Err(ProjectionError::Precision(invalid)),
        );
    }
}

#[test]
fn equal_endpoint_values_retain_different_source_decimal_quantum() {
    use cftuv_density::atlas_plan::DecimalQuantum;
    use cftuv_density::density_angle_value_v1::AngleValueRegistryV1;
    use cftuv_density::density_construct_v1::ConstructorV1;
    use cftuv_density::density_expand_v1::ExpandLimits;
    use cftuv_density::density_expression::Expressions;
    use cftuv_density::density_laws_v1::{OwnedDensityLawsV1, OwnedFanGeometryV1};
    use cftuv_density::geometry::WindowLaw;
    use cftuv_density::Rational;
    let mut expressions = Expressions::new();
    let mut constructor = ConstructorV1::new(&mut expressions);
    let one = constructor.rational(&Rational::integer(IBig::ONE)).unwrap();
    let zero = constructor.rational(&Rational::integer(IBig::ZERO)).unwrap();
    let q = |value| Rational::integer(IBig::from(value));
    let input = OwnedFanGeometryV1 {
        inverse_gram: [[q(1), q(0)], [q(0), q(1)]],
        owner_orientation: 1,
        turn_orientation: 1,
        ideal: vec![[one.clone(), zero.clone()], [zero, one.clone()]],
        q: 3,
        window_law: WindowLaw::Voronoi,
    };
    let limits = ExpandLimits { max_terms: 256, max_polynomial_exponent: 16, max_rational_power_bits: 4096 };
    let mut laws = OwnedDensityLawsV1::new(&mut constructor, input, AngleValueRegistryV1::new(), limits).unwrap();
    let shell = laws.decimal_envelope(&one).unwrap();
    let exact_atlas_endpoint = cftuv_density::window_build::exact_decimal_interval(IBig::ONE);
    assert_eq!(shell.lower, exact_atlas_endpoint.lower);
    assert_eq!(shell.upper, exact_atlas_endpoint.upper);
    assert_eq!(shell.absolute_error_bound, exact_atlas_endpoint.absolute_error_bound);
    assert_eq!(shell.quantum, Some(DecimalQuantum::uniform(-27)));
    assert_eq!(exact_atlas_endpoint.quantum, Some(DecimalQuantum::uniform(0)));
    assert_ne!(shell, exact_atlas_endpoint);
    // Hit памяти также возвращает исходную запись, не восстанавливает разряды по значению.
    assert_eq!(laws.decimal_envelope(&one).unwrap(), shell);
}
