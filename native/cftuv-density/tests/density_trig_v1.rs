use cftuv_density::density_construct_v1::ConstructorV1;
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_sign::SourceSignProbe;
use cftuv_density::density_trig_v1::*;
use cftuv_density::{IBig, Rational, UBig};

fn q(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}

#[test]
fn special_principal_angles_and_inclusive_ray_endpoints_are_known() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    for (ratio, sign, m) in [
        (q(0, 1), 0, 6),
        (q(1, 4), 1, 4),
        (q(1, 4), -1, 8),
        (q(1, 2), 1, 3),
        (q(1, 2), -1, 9),
        (q(3, 4), 1, 2),
        (q(3, 4), -1, 10),
        (q(1, 1), 1, 0),
        (q(1, 1), -1, 12),
    ] {
        assert_eq!(principal_pi_multiple(&ratio, sign).unwrap(), Some(q(m, 12)));
        for n in 2..=6 {
            for j in 0..=n {
                assert!(owned_principal_ray(&mut c, &ratio, sign, n, j)
                    .unwrap()
                    .is_some());
            }
        }
    }
    assert_eq!(principal_pi_multiple(&q(2, 3), 1).unwrap(), None);
}
#[test]
fn full_circle_twelfths_are_numeric_and_unit_norm_is_exact_zero() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    for numerator in -48..=48 {
        let [a, b] = owned_rational_pi_trig(&mut c, &q(numerator, 12)).unwrap();
        let cosine = c.numeric_value(&a).unwrap().unwrap();
        let sine = c.numeric_value(&b).unwrap().unwrap();
        let unit = cosine.mul(cosine).add(&sine.mul(sine));
        assert_eq!(unit.value().as_rational(), Some((IBig::ONE, UBig::ONE)));
        let unit = c.numeric(unit).unwrap();
        let one = c.rational(&q(1, 1)).unwrap();
        let zero = c.sub(&unit, &one).unwrap();
        assert!(matches!(c.probe_sign(&zero).unwrap(),SourceSignProbe::Decided(v) if v.sign==0));
    }
}
#[test]
fn fifth_cosines_numeric_and_exact_q_five_sine_keeps_opaque_root() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    for n in [1, 2] {
        let [cosine, sine] = owned_rational_pi_trig(&mut c, &q(n, 5)).unwrap();
        assert!(c.numeric_value(&cosine).unwrap().is_some());
        assert!(c.numeric_value(&sine).unwrap().is_none());
    }
    let [cosine, sine] = owned_exact_q_trig(&mut c, 5).unwrap();
    assert!(c.numeric_value(&cosine).unwrap().is_some());
    assert!(c.numeric_value(&sine).unwrap().is_none());
    let literal = c.export_raw(&sine).unwrap();
    assert!(literal.nodes().iter().any(|v| matches!(
        v,
        cftuv_density::density_construct_v1::DecisionNodeV1::Pow(..)
    )));
    assert!(matches!(c.probe_sign(&sine).unwrap(),SourceSignProbe::Decided(v) if v.sign==1));
}
#[test]
fn unknown_components_retain_original_raw_trig_and_guard_order() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    for value in [q(1, 16), q(3, 16), q(1, 7), q(-31, 16)] {
        let [cosine, sine] = owned_rational_pi_trig(&mut c, &value).unwrap();
        assert!(c.numeric_value(&cosine).unwrap().is_none());
        assert!(c.numeric_value(&sine).unwrap().is_none());
        let pi = c.pi().unwrap();
        let angle = c.scaled(&pi, &value).unwrap();
        let raw_c = c.raw_cos(&angle).unwrap();
        let raw_s = c.raw_sin(&angle).unwrap();
        assert_eq!(
            c.export_raw(&cosine).unwrap(),
            c.export_raw(&raw_c).unwrap()
        );
        assert_eq!(c.export_raw(&sine).unwrap(), c.export_raw(&raw_s).unwrap());
    }
    assert_eq!(
        owned_principal_ray(&mut c, &q(-1, 1), 0, 7, 8).unwrap_err(),
        TrigErrorV1::InvalidSubdivision
    );
    assert_eq!(
        owned_principal_ray(&mut c, &q(-1, 1), 0, 6, 7).unwrap_err(),
        TrigErrorV1::InvalidOrdinal
    );
    assert_eq!(
        principal_pi_multiple(&q(0, 1), 1).unwrap_err(),
        TrigErrorV1::InconsistentDotSign
    );
    assert_eq!(
        principal_pi_multiple(&q(2, 1), 1).unwrap_err(),
        TrigErrorV1::RatioOutsideClosedUnitInterval
    );
}

#[test]
fn finite_special_norms_resolve_zero_while_old_opaque_profile_stays_undecidable() {
    use cftuv_density::density_construct_v1::OwnedSignErrorV1;
    use cftuv_density::density_expand_v1::{lazy_sign, ExpandLimits, LazySignErrorV1};
    use cftuv_density::density_sign::SignFailure;
    let angles = [
        (7, 30, true),
        (7, 24, true),
        (7, 20, true),
        (11, 30, true),
        (13, 30, true),
        (11, 24, true),
        (1, 36, false),
        (1, 30, true),
        (1, 24, true),
        (1, 20, true),
        (1, 18, false),
        (1, 16, false),
        (1, 15, true),
        (1, 12, true),
        (1, 10, true),
        (1, 9, false),
        (1, 8, true),
        (2, 15, true),
        (5, 36, false),
        (3, 20, true),
        (1, 6, true),
        (3, 16, false),
        (1, 5, true),
        (5, 24, true),
        (2, 9, false),
        (1, 4, true),
        (4, 15, true),
        (5, 18, false),
        (3, 10, true),
        (1, 3, true),
        (3, 8, true),
        (2, 5, true),
        (5, 12, true),
        (4, 9, false),
        (9, 20, true),
        (1, 2, true),
        (8, 15, true),
        (5, 9, false),
        (9, 16, false),
        (3, 5, true),
        (5, 8, true),
        (2, 3, true),
        (25, 36, false),
    ];
    for (n, d, known) in angles {
        let mut graph = Expressions::new();
        let mut c = ConstructorV1::new(&mut graph);
        let [a, b] = owned_rational_pi_trig(&mut c, &q(n, d)).unwrap();
        let aa = c.integer_power(&a, 2).unwrap();
        let bb = c.integer_power(&b, 2).unwrap();
        let sum = c.add(&[aa, bb]).unwrap();
        let one = c.rational(&q(1, 1)).unwrap();
        let residual = c.sub(&sum, &one).unwrap();
        let result = lazy_sign(
            &mut c,
            &residual,
            ExpandLimits {
                max_terms: 4096,
                max_polynomial_exponent: 4,
                max_rational_power_bits: 65536,
            },
        );
        if known {
            assert_eq!(result.unwrap().result.sign, 0, "{n}/{d}");
        } else {
            assert!(
                matches!(
                    result,
                    Err(LazySignErrorV1::Sign(OwnedSignErrorV1::Sign(
                        SignFailure::Undecidable
                    )))
                ),
                "{n}/{d}"
            );
        }
    }
}
