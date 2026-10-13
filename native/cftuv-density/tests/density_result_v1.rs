use cftuv_density::density_bind_v1::{bind_density_fan, produce_principal_fan, RawFanGeometryV1};
use cftuv_density::density_construct_v1::ConstructorV1;
use cftuv_density::density_expand_v1::ExpandLimits;
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_laws_v1::OwnedDensityLawsV1;
use cftuv_density::density_result_v1::{authority_facts, certificate_facts, OrdinalWindowFactsV1};
use cftuv_density::fan_compile::CompileLimits;
use cftuv_density::geometry::WindowLaw;
use cftuv_density::{IBig, Rational, UBig};
fn q(n: i32) -> Rational {
    Rational::integer(IBig::from(n))
}
#[test]
fn freshly_constructed_raw_principal_bind_exports_decimal_facts_in_ordinal_order() {
    for law in [WindowLaw::Voronoi, WindowLaw::NarrowBand] {
        for lifted in [false, true] {
            let mut arena = Expressions::new();
            let mut c = ConstructorV1::new(&mut arena);
            let expand = ExpandLimits {
                max_terms: 4096,
                max_polynomial_exponent: 16,
                max_rational_power_bits: 65536,
            };
            let raw = RawFanGeometryV1 {
                inverse_gram: [[q(1), q(0)], [q(0), q(1)]],
                incoming: [q(1), q(0)],
                outgoing: [q(0), q(1)],
                owner_orientation: 1,
                turn_orientation: 1,
                hidden_count: 1,
                q: 3,
                window_law: law,
            };
            let produced = produce_principal_fan(&mut c, &raw, expand).unwrap();
            let mut laws =
                OwnedDensityLawsV1::new(&mut c, produced.geometry, produced.values, expand)
                    .unwrap();
            let report = bind_density_fan(
                &mut laws,
                1,
                3,
                &[false],
                lifted,
                law,
                &CompileLimits::default(),
            );
            let bound = report.result().as_ref().unwrap();
            assert_eq!(
                report.attempts[0].adaptive_only,
                lifted || law == WindowLaw::NarrowBand
            );
            if let Some(authority) = &bound.authority {
                let final_law = report.attempts.last().unwrap().window_law;
                let facts = authority_facts(authority, &[None], final_law).unwrap();
                assert_eq!(facts.bound_primitive_integer_vectors[0].0, IBig::ONE);
                assert_eq!(facts.minimal_common_height, UBig::ONE);
                assert_eq!(facts.previous_height_witness.first_height, UBig::ONE);
                assert!(facts.authority_id.is_none());
                let OrdinalWindowFactsV1::Local {
                    bounds,
                    full_lower_slope_envelope,
                    ..
                } = &facts.ordinal_windows[0]
                else {
                    panic!("expected local window");
                };
                assert_eq!(bounds.ordinal, 1);
                assert_eq!(full_lower_slope_envelope.lower.exponent, -27);
            } else {
                let certificate = bound.certificates[0].as_ref().unwrap();
                let facts = certificate_facts(certificate, None).unwrap();
                assert_eq!(facts.bound_primitive_integer_vector.0, IBig::ONE);
                assert_eq!(facts.ideal_window_lower_slope_envelope.lower.exponent, -27);
                assert!(facts.proven_predicates.windows(2).all(|p| p[0] < p[1]));
            }
        }
    }
}
#[test]
fn exact_subturn_limit_band_refusal_then_voronoi_preserves_failure_and_both_prices() {
    let mut arena = Expressions::new();
    let mut c = ConstructorV1::new(&mut arena);
    let expand = ExpandLimits {
        max_terms: 4096,
        max_polynomial_exponent: 16,
        max_rational_power_bits: 65536,
    };
    let raw = RawFanGeometryV1 {
        inverse_gram: [[q(1), q(0)], [q(0), q(1)]],
        incoming: [q(1), q(0)],
        outgoing: [q(0), q(1)],
        owner_orientation: 1,
        turn_orientation: 1,
        hidden_count: 1,
        q: 4,
        window_law: WindowLaw::NarrowBand,
    };
    let produced = produce_principal_fan(&mut c, &raw, expand).unwrap();
    let mut laws =
        OwnedDensityLawsV1::new(&mut c, produced.geometry, produced.values, expand).unwrap();
    let report = bind_density_fan(
        &mut laws,
        1,
        4,
        &[false],
        true,
        WindowLaw::NarrowBand,
        &CompileLimits::default(),
    );
    assert_eq!(report.attempts.len(), 2);
    assert!(report.attempts.iter().all(|attempt| attempt.adaptive_only));
    assert!(report.attempts.iter().all(|attempt| attempt
        .report
        .adaptive_budgets
        .preparation
        .is_some()));
    assert!(report.result().is_err());
    assert_eq!(
        report.band_refusal.unwrap().text(),
        "DensityTerminationBoxesExhausted: termination box refinement exhausted after 96 halvings for 1 hidden rays: no feasible box was witnessed (ideal fan exactly at the subturn limit: True)"
    );
}

#[test]
fn legacy_width_keeps_source_precision_quantum_after_rounding_to_integer() {
    use cftuv_density::atlas_plan::{DecimalEnvelope, DecimalQuantum, EndpointKind};
    use cftuv_density::farey::PrimitiveVector;
    use cftuv_density::legacy_binding::{LegacyCertificate, ProvenPredicates};
    let lower = DecimalEnvelope {
        lower: q(0),
        upper: q(0),
        lower_kind: EndpointKind::Closed,
        upper_kind: EndpointKind::Closed,
        absolute_error_bound: q(0),
        quantum: Some(DecimalQuantum::uniform(-27)),
    };
    let units: IBig = "10000000000000000000000000000001".parse().unwrap();
    let scale: UBig = "1000000000000000000000000000".parse().unwrap();
    let endpoint = Rational::new(units, scale).unwrap();
    let upper = DecimalEnvelope {
        lower: endpoint.clone(),
        upper: endpoint,
        ..lower.clone()
    };
    let value = LegacyCertificate {
        bound_primitive_integer_vector: PrimitiveVector(IBig::ONE, IBig::ONE),
        ideal_window_lower_slope_envelope: lower,
        ideal_window_upper_slope_envelope: upper,
        certified_window_width_lower_bound: q(10000),
        proven_predicates: ProvenPredicates::Canonical,
    };
    let facts = certificate_facts(&value, None).unwrap();
    assert_eq!(
        facts
            .certified_window_width_lower_bound
            .coefficient
            .to_string(),
        "1000000000000000000000000000"
    );
    assert_eq!(facts.certified_window_width_lower_bound.exponent, -23);
}
