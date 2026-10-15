use cftuv_density::angle::AngleAtom;
use cftuv_density::density_angle_value_v1::*;
use cftuv_density::density_construct_v1::{ConstructorV1, OwnedExprV1};
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_sign::SourceSignProbe;
use cftuv_density::density_trig_v1::principal_pi_multiple;
use cftuv_density::interval::IntervalError;
use cftuv_density::{IBig, Rational, UBig};
fn q(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn pair(c: &mut ConstructorV1<'_>, atom: &AngleAtom, j: u8, canonical: bool) -> [OwnedExprV1; 2] {
    let factor = q(i32::from(j), u32::from(atom.subdivision()));
    let angle = if canonical {
        let pi = c.pi().unwrap();
        let multiple = principal_pi_multiple(atom.ratio(), atom.dot_sign())
            .unwrap()
            .unwrap();
        let (n, d) = multiple.parts();
        let multiple =
            Rational::new(n * IBig::from(j), d * UBig::from(atom.subdivision())).unwrap();
        c.scaled(&pi, &multiple).unwrap()
    } else {
        let r = c.rational(atom.ratio()).unwrap();
        let root = c.sqrt(&r).unwrap();
        let cosine = c.scaled(&root, &q(i32::from(atom.dot_sign()), 1)).unwrap();
        let one = c.rational(&q(1, 1)).unwrap();
        let residual = c.sub(&one, &r).unwrap();
        let sine = c.sqrt(&residual).unwrap();
        let angle = c.raw_atan2(&sine, &cosine).unwrap();
        c.scaled(&angle, &factor).unwrap()
    };
    [c.raw_cos(&angle).unwrap(), c.raw_sin(&angle).unwrap()]
}

#[test]
fn exact_a2_value_keeps_non_numeric_literal_provenance() {
    use cftuv_density::density_construct_v1::OwnedFormV1;
    use cftuv_density::interval::RationalInterval;
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let atom = AngleAtom::new(2, q(1, 64), 1).unwrap();
    let [cosine, sine] = pair(&mut c, &atom, 1, false);
    let mut values = AngleValueRegistryV1::new();
    values.register_ray(&c, &cosine, &sine, &atom, 1).unwrap();
    let scaled = c.scaled(&cosine, &q(4, 3)).unwrap();
    assert!(matches!(c.form(&cosine).unwrap(), OwnedFormV1::Cos(_)));
    assert!(matches!(c.form(&scaled).unwrap(), OwnedFormV1::Mul(_)));
    assert!(c.numeric_value(&scaled).unwrap().is_none());
    assert_eq!(
        values.enclosure(&c, &cosine).unwrap(),
        RationalInterval::point(q(3, 4))
    );
    assert_eq!(
        values.enclosure(&c, &scaled).unwrap(),
        RationalInterval::point(q(1, 1))
    );
}

#[test]
fn observed_minus_quarter_components_preserve_signed_bounds_and_registration_guards() {
    use cftuv_density::density_construct_v1::OwnedFormV1;
    use cftuv_density::interval::RationalInterval;
    for (n, j) in [(3, 2), (4, 1)] {
        let mut graph = Expressions::new();
        let mut c = ConstructorV1::new(&mut graph);
        let atom = AngleAtom::new(n, q(81, 100), -1).unwrap();
        let [cosine, _] = pair(&mut c, &atom, j, false);
        let OwnedFormV1::Cos(angle) = c.form(&cosine).unwrap() else {
            panic!("raw cosine");
        };
        let pi = c.pi().unwrap();
        let half_pi = c.scaled(&pi, &q(1, 2)).unwrap();
        let shifted = c.sub(&angle, &half_pi).unwrap();
        let cosine = c.raw_cos(&shifted).unwrap();
        let sine = c.raw_sin(&shifted).unwrap();
        let literal = [c.export_raw(&cosine).unwrap(), c.export_raw(&sine).unwrap()];
        let mut values = AngleValueRegistryV1::new();
        values
            .register_ray_with_quarter_turns(&c, &cosine, &sine, &atom, j, -1)
            .unwrap();
        let ray = cftuv_density::angle_profile::ray(&atom, j).unwrap();
        let expected = RationalInterval::point(q(0, 1)).sub(&ray.cosine);
        assert_eq!(values.enclosure(&c, &cosine).unwrap(), ray.sine);
        assert_eq!(values.enclosure(&c, &sine).unwrap(), expected);
        if j == 2 {
            assert!(expected.lower().parts().0 > &IBig::ZERO);
        } else {
            assert!(expected.upper().parts().0 < &IBig::ZERO);
        }
        assert_eq!(
            [c.export_raw(&cosine).unwrap(), c.export_raw(&sine).unwrap()],
            literal
        );
        assert_eq!(
            values
                .register_ray(&c, &cosine, &sine, &atom, j)
                .unwrap_err(),
            AngleValueErrorV1::ConflictingRegistration
        );
        for invalid in [1, -2] {
            assert_eq!(
                values
                    .register_ray_with_quarter_turns(&c, &cosine, &sine, &atom, j, invalid)
                    .unwrap_err(),
                AngleValueErrorV1::InvalidRegistration
            );
        }
        let mut foreign_graph = Expressions::new();
        let mut foreign = ConstructorV1::new(&mut foreign_graph);
        let other = foreign.rational(&q(0, 1)).unwrap();
        assert!(matches!(
            values.register_ray_with_quarter_turns(&c, &other, &other, &atom, j, 1),
            Err(AngleValueErrorV1::Construction(_))
        ));
        assert_eq!(values.enclosure(&c, &sine).unwrap(), expected);
    }
}

#[test]
#[ignore = "explicit actual OLD prepared-angle matrix from Python registration audit"]
fn live_actual_prepared_registration_bounds_match_python_for_all_missing_components() {
    use cftuv_density::density_construct_v1::OwnedFormV1;
    use cftuv_density::interval::RationalInterval;
    let path =
        std::env::var("CFTUV_DENSITY_PREPARED_ANGLE_MATRIX").expect("explicit actual fixture path");
    let text = std::fs::read_to_string(path).unwrap();
    let fraction = |text: &str| {
        let (n, d) = text.split_once('/').unwrap();
        Rational::new(n.parse::<IBig>().unwrap(), d.parse::<UBig>().unwrap()).unwrap()
    };
    let mut count = 0;
    for line in text.lines() {
        let row: Vec<_> = line.split('\t').collect();
        assert_eq!(row.len(), 9);
        let atom = AngleAtom::new(
            row[0].parse().unwrap(),
            fraction(row[1]),
            row[2].parse().unwrap(),
        )
        .unwrap();
        let j = row[3].parse().unwrap();
        let shift: i8 = row[4].parse().unwrap();
        let mut graph = Expressions::new();
        let mut c = ConstructorV1::new(&mut graph);
        let [cosine, _] = pair(&mut c, &atom, j, false);
        let OwnedFormV1::Cos(mut angle) = c.form(&cosine).unwrap() else {
            panic!("raw cosine");
        };
        if shift == -1 {
            let pi = c.pi().unwrap();
            let half_pi = c.scaled(&pi, &q(1, 2)).unwrap();
            angle = c.sub(&angle, &half_pi).unwrap();
        }
        let cosine = c.raw_cos(&angle).unwrap();
        let sine = c.raw_sin(&angle).unwrap();
        let mut values = AngleValueRegistryV1::new();
        values
            .register_ray_with_quarter_turns(&c, &cosine, &sine, &atom, j, shift)
            .unwrap();
        assert_eq!(
            values.enclosure(&c, &cosine).unwrap(),
            RationalInterval::new(fraction(row[5]), fraction(row[6])).unwrap()
        );
        assert_eq!(
            values.enclosure(&c, &sine).unwrap(),
            RationalInterval::new(fraction(row[7]), fraction(row[8])).unwrap()
        );
        count += 1;
    }
    assert_eq!(count, 154);
    println!("actual_prepared_registration_rows={count}");
}
#[test]
fn registered_value_keeps_literal_and_metadata_is_idempotent_or_conflicting() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let atom = AngleAtom::new(4, q(0, 1), 0).unwrap();
    let [cosine, sine] = pair(&mut c, &atom, 1, false);
    let old = c.export_raw(&cosine).unwrap();
    let mut values = AngleValueRegistryV1::new();
    assert_eq!(
        values.enclosure(&c, &cosine).unwrap_err(),
        AngleValueErrorV1::UnregisteredOpaque
    );
    values.register_ray(&c, &cosine, &sine, &atom, 1).unwrap();
    let saved = values.enclosure(&c, &cosine).unwrap();
    values.register_ray(&c, &cosine, &sine, &atom, 1).unwrap();
    assert_eq!(values.enclosure(&c, &cosine).unwrap(), saved);
    let other = AngleAtom::new(2, q(1, 2), 1).unwrap();
    assert_eq!(
        values
            .register_ray(&c, &cosine, &sine, &other, 1)
            .unwrap_err(),
        AngleValueErrorV1::ConflictingRegistration
    );
    assert_eq!(values.enclosure(&c, &cosine).unwrap(), saved);
    assert_eq!(c.export_raw(&cosine).unwrap(), old);
    let mut g = Expressions::new();
    let mut foreign = ConstructorV1::new(&mut g);
    let mut expr = foreign.rational(&q(100, 1)).unwrap();
    for number in 1..=c.raw_id(&cosine).unwrap() {
        expr = foreign.rational(&q(100 + number as i32, 1)).unwrap();
    }
    assert_eq!(foreign.raw_id(&expr).unwrap(), c.raw_id(&cosine).unwrap());
    assert!(matches!(
        values.enclosure(&foreign, &expr),
        Err(AngleValueErrorV1::Construction(_))
    ));
}
#[test]
fn canonical_aliases_choose_fixed_recipe_and_non_numeric_five_pi_over_thirty_six_is_covered() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let atom = AngleAtom::new(4, q(0, 1), 0).unwrap();
    let [a, b] = pair(&mut c, &atom, 1, true);
    let mut values = AngleValueRegistryV1::new();
    values.register_ray(&c, &a, &b, &atom, 1).unwrap();
    let (fixed, j) = canonical_pi_recipe(&q(1, 8)).unwrap();
    assert_eq!(fixed.subdivision(), 2);
    assert_eq!(fixed.ratio(), &q(1, 2));
    assert_eq!(
        values.enclosure(&c, &a).unwrap(),
        cftuv_density::angle_profile::ray(&fixed, j).unwrap().cosine
    );
    values.register_ray(&c, &a, &b, &fixed, j).unwrap();
    let (atom, j) = canonical_pi_recipe(&q(5, 36)).unwrap();
    let [a, b] = pair(&mut c, &atom, j, true);
    values
        .register_canonical_ray(&c, &a, &b, &q(5, 36))
        .unwrap();
    assert_eq!(
        values.enclosure(&c, &b).unwrap(),
        cftuv_density::angle_profile::ray(&atom, j).unwrap().sine
    );
    assert_eq!(
        values
            .register_canonical_ray(&c, &a, &b, &q(1, 7))
            .unwrap_err(),
        AngleValueErrorV1::NoCanonicalRecipe
    );
}
#[test]
fn tiny_positive_sine_old_sign_and_fixed_value_reciprocal_are_distinct() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let denominator = UBig::from(10u8).pow(200);
    let ratio = Rational::new(IBig::from(denominator.clone()) - IBig::ONE, denominator).unwrap();
    let atom = AngleAtom::new(2, ratio, 1).unwrap();
    let [cosine, sine] = pair(&mut c, &atom, 1, false);
    assert!(matches!(c.probe_sign(&sine).unwrap(),SourceSignProbe::Decided(v) if v.sign==1));
    let mut values = AngleValueRegistryV1::new();
    values.register_ray(&c, &cosine, &sine, &atom, 1).unwrap();
    assert_eq!(values.enclosure(&c, &sine).unwrap().lower(), &q(0, 1));
    let reciprocal = c.formal_reciprocal(&sine).unwrap();
    assert_eq!(
        values.enclosure(&c, &reciprocal).unwrap_err(),
        AngleValueErrorV1::Interval(IntervalError::DenominatorContainsZero)
    );
}
