use cftuv_density::density_construct_v1::*;
use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::scalar_v2::{ProfileCode as ScalarProfile, ScalarError, ScalarV2};
use cftuv_density::{IBig, Rational, UBig};
use cftuv_radical::Failure;

fn q(c: &mut ConstructorV1<'_>, n: i64, d: u64) -> OwnedExprV1 {
    c.rational(&Rational::new(IBig::from(n), UBig::from(d)).unwrap())
        .unwrap()
}
fn literal(c: &ConstructorV1<'_>, value: &OwnedExprV1) -> DecisionLiteralV1 {
    c.export_raw(value).unwrap()
}
#[test]
fn scalar_island_is_atomic_and_literal_sharing_is_exact() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let two = q(&mut c, 2, 1);
    let three = q(&mut c, 3, 1);
    let sqrt2 = c.sqrt(&two).unwrap();
    let sqrt3 = c.sqrt(&three).unwrap();
    let number = c.add(&[sqrt2.clone(), sqrt3.clone()]).unwrap();
    let pi = c.pi().unwrap();
    let sin = c.raw_sin(&pi).unwrap();
    let mixed = c.mul(&[number.clone(), sin.clone()]).unwrap();
    let Node::Mul(args) = c.graph().node(c.raw_id(&mixed).unwrap()).unwrap() else {
        panic!("mixed Mul");
    };
    assert_eq!(args.len(), 2);
    assert!(args.contains(&c.raw_id(&number).unwrap()));
    let split_a = c.mul(&[sqrt2, sin.clone()]).unwrap();
    let split_b = c.mul(&[sqrt3, sin]).unwrap();
    let split = c.add(&[split_a, split_b]).unwrap();
    assert_eq!(literal(&c, &mixed), literal(&c, &split));
    let half = q(&mut c, 1, 2);
    let scalar = c.add(&[half, number]).unwrap();
    let raw = literal(&c, &scalar);
    assert_eq!(
        raw.nodes(),
        &[
            DecisionNodeV1::Q(IBig::ONE, UBig::from(2u8)),
            DecisionNodeV1::Q(IBig::from(2), UBig::ONE),
            DecisionNodeV1::Pow(1, 0),
            DecisionNodeV1::Q(IBig::from(3), UBig::ONE),
            DecisionNodeV1::Pow(3, 0),
            DecisionNodeV1::Add(vec![0, 2, 4])
        ]
    );
}
#[test]
fn finite_rational_add_branches_like_terms_and_integer_powers() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let pi = c.pi().unwrap();
    let sin = c.raw_sin(&pi).unwrap();
    let cos = c.raw_cos(&pi).unwrap();
    let sum = c.add(&[sin.clone(), cos.clone()]).unwrap();
    let twice = c.scaled(&sum, &Rational::integer(IBig::from(2))).unwrap();
    assert!(matches!(
        c.graph().node(c.raw_id(&twice).unwrap()).unwrap(),
        Node::Add(_)
    ));
    let two = q(&mut c, 2, 1);
    let root = c.sqrt(&two).unwrap();
    let inner = c.mul(&[root.clone(), sum.clone()]).unwrap();
    let kept = c.mul(&[root, inner]).unwrap(); // Numeric factors ->2, raw coefficient*Add.
    assert!(matches!(
        c.graph().node(c.raw_id(&kept).unwrap()).unwrap(),
        Node::Mul(_)
    ));
    let kept_three = c.scaled(&kept, &Rational::integer(IBig::from(3))).unwrap();
    let Node::Mul(args) = c.graph().node(c.raw_id(&kept_three).unwrap()).unwrap() else {
        panic!();
    };
    assert_eq!(args[1], c.raw_id(&sum).unwrap());
    let recovered = c
        .scaled(&kept, &Rational::new(IBig::ONE, UBig::from(2u8)).unwrap())
        .unwrap();
    assert_eq!(literal(&c, &recovered), literal(&c, &sum));
    let pow = c.integer_power(&sum, 2).unwrap();
    let times = c.mul(&[pow.clone(), pow.clone()]).unwrap();
    let expected = c.integer_power(&sum, 4).unwrap();
    assert_eq!(literal(&c, &times), literal(&c, &expected));
    let nested = c.integer_power(&pow, 3).unwrap();
    let flattened = c.integer_power(&sum, 6).unwrap();
    assert_ne!(literal(&c, &nested), literal(&c, &flattened));
    let huge = c.integer_power(&sin, 1 << 20).unwrap();
    assert!(matches!(
        c.mul(&[huge.clone(), huge]),
        Err(ConstructErrorV1::ExponentProfile)
    ));
    let repeat = c.add(&[sin.clone(), sin.clone()]).unwrap();
    let expected = c.scaled(&sin, &Rational::integer(IBig::from(2))).unwrap();
    assert_eq!(literal(&c, &repeat), literal(&c, &expected));
    let difference = c.sub(&repeat, &expected).unwrap();
    assert!(c
        .numeric_value(&difference)
        .unwrap()
        .unwrap()
        .value()
        .terms()
        .is_empty());
    let numeric = q(&mut c, 2, 1);
    let root = c.sqrt(&numeric).unwrap();
    let term = c.mul(&[root, sin]).unwrap();
    let twice = c.add(&[term.clone(), term]).unwrap();
    let Node::Mul(args) = c.graph().node(c.raw_id(&twice).unwrap()).unwrap() else {
        panic!();
    };
    assert_eq!(args.len(), 2); // Scalar 2sqrt2 целиком, не literal factor fragments.
}
#[test]
fn positive_half_pair_and_even_power_fold_and_numeric_negative_base_first() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let pi = c.pi().unwrap();
    let root = c.sqrt(&pi).unwrap();
    let repeat = c.mul(&[root.clone(), root.clone()]).unwrap();
    assert_eq!(literal(&c, &repeat), literal(&c, &pi));
    let explicit = c.integer_power(&root, 2).unwrap();
    assert_eq!(literal(&c, &repeat), literal(&c, &explicit));
    let sine = c.raw_sin(&pi).unwrap();
    let root_sine = c.sqrt(&sine).unwrap();
    let repeat_sine = c.mul(&[root_sine.clone(), root_sine.clone()]).unwrap();
    assert_eq!(literal(&c, &repeat_sine), literal(&c, &sine));
    let two = q(&mut c, 2, 1);
    let numeric_root = c.sqrt(&two).unwrap();
    let nested = c.sqrt(&numeric_root).unwrap();
    let repeat_nested = c.mul(&[nested.clone(), nested.clone()]).unwrap();
    assert_eq!(literal(&c, &repeat_nested), literal(&c, &numeric_root));
    assert!(matches!(
        c.integer_power(&pi, -1),
        Err(ConstructErrorV1::Profile(
            ProfileCode::FreshNegativeOpaquePower
        ))
    ));
    assert!(matches!(
        c.raw_opaque_pow(&pi, &Rational::new(IBig::ONE, UBig::from(3u8)).unwrap()),
        Err(ConstructErrorV1::Profile(
            ProfileCode::HalfIntegerExponentRequired
        ))
    ));
    let minus = q(&mut c, -1, 1);
    assert!(matches!(
        c.sqrt(&minus),
        Err(ConstructErrorV1::Profile(ProfileCode::NegativeRationalRoot))
    ));
    let mut roots = Vec::new();
    for n in [2, 3, 5] {
        let value = q(&mut c, n, 1);
        roots.push(c.sqrt(&value).unwrap());
    }
    let three = c.add(&roots).unwrap();
    let invsquare = c.integer_power(&three, -2).unwrap();
    let expected = c
        .numeric_value(&three)
        .unwrap()
        .unwrap()
        .integer_power(-2)
        .unwrap();
    assert_eq!(c.numeric_value(&invsquare).unwrap(), Some(&expected));
    let square = c.integer_power(&three, 2).unwrap();
    assert!(
        matches!(c.integer_power(&square,-1),Err(ConstructErrorV1::Scalar(ScalarError::Profile(p))) if p.code==ScalarProfile::InverseClassLimit)
    );
    let zero = q(&mut c, 0, 1);
    assert!(matches!(
        c.integer_power(&zero, -1),
        Err(ConstructErrorV1::Scalar(ScalarError::Algebra(
            Failure::ZeroDivision
        )))
    ));
    assert!(matches!(
        c.integer_power(&zero, i64::MIN),
        Err(ConstructErrorV1::ExponentProfile)
    ));
}
#[test]
fn positive_odd_half_law_preserves_odd_outer_profile_caps_and_partial_domain() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let pi = c.pi().unwrap();
    let half3 = c
        .raw_opaque_pow(&pi, &Rational::new(IBig::from(3), UBig::from(2u8)).unwrap())
        .unwrap();
    let paired = c.mul(&[half3.clone(), half3.clone()]).unwrap();
    let pi3 = c.integer_power(&pi, 3).unwrap();
    assert!(c.same_literal(&paired, &pi3).unwrap());
    let fourth = c.integer_power(&half3, 4).unwrap();
    let pi6 = c.integer_power(&pi, 6).unwrap();
    assert!(c.same_literal(&fourth, &pi6).unwrap());
    let odd = c.integer_power(&half3, 3).unwrap();
    assert!(matches!(
        c.form(&odd).unwrap(),
        OwnedFormV1::IntegerPow { exponent: 3, .. }
    ));
    let oddpair = c.mul(&[odd.clone(), odd]).unwrap();
    let pi9 = c.integer_power(&pi, 9).unwrap();
    assert!(c.same_literal(&oddpair, &pi9).unwrap());
    let odd5 = c.integer_power(&half3, 5).unwrap();
    let odd7 = c.integer_power(&half3, 7).unwrap();
    let odd3 = c.integer_power(&half3, 3).unwrap();
    let handles = [odd3, odd5, odd7];
    let canonical = c.integer_power(&half3, 15).unwrap();
    for permutation in [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ] {
        let result = c.mul(&permutation.map(|i| handles[i].clone())).unwrap();
        assert!(c.same_literal(&result, &canonical).unwrap());
    }
    let triple = c
        .mul(&[half3.clone(), half3.clone(), half3.clone()])
        .unwrap();
    let expected = c.mul(&[pi3, half3]).unwrap();
    assert!(c.same_literal(&triple, &expected).unwrap());

    let high = c
        .raw_opaque_pow(
            &pi,
            &Rational::new(IBig::from((1i64 << 20) - 1), UBig::from(2u8)).unwrap(),
        )
        .unwrap();
    assert!(matches!(
        c.integer_power(&high, 4),
        Err(ConstructErrorV1::ExponentProfile)
    ));
    assert!(matches!(
        c.raw_opaque_pow(&pi, &Rational::new(-IBig::ONE, UBig::from(2u8)).unwrap()),
        Err(ConstructErrorV1::Profile(
            ProfileCode::FreshNegativeOpaquePower
        ))
    ));

    let reciprocal = c.formal_reciprocal(&pi).unwrap();
    let root = c.sqrt(&reciprocal).unwrap();
    let folded = c.mul(&[root.clone(), root]).unwrap();
    assert!(c.same_literal(&folded, &reciprocal).unwrap());
    assert!(c.contains_partial_reciprocal(&folded).unwrap());
    let zero = q(&mut c, 0, 1);
    assert!(matches!(
        c.mul(&[zero, folded.clone()]),
        Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard))
    ));
    assert!(matches!(
        c.sub(&folded, &folded),
        Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard))
    ));
    assert!(matches!(
        c.integer_power(&folded, 0),
        Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard))
    ));
    let half3_partial = c
        .raw_opaque_pow(
            &reciprocal,
            &Rational::new(IBig::from(3), UBig::from(2u8)).unwrap(),
        )
        .unwrap();
    let folded4 = c.integer_power(&half3_partial, 4).unwrap();
    assert!(c.contains_partial_reciprocal(&folded4).unwrap());
}

#[test]
fn actual_numeric_coefficients_group_on_opaque_monomial_and_preserve_partial() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let one = q(&mut c, 1, 1);
    let sine = c.raw_sin(&one).unwrap();
    let two = q(&mut c, 2, 1);
    let three = q(&mut c, 3, 1);
    let root2 = c.sqrt(&two).unwrap();
    let root3 = c.sqrt(&three).unwrap();
    let term = c.mul(&[root2.clone(), sine.clone()]).unwrap();
    let zero = c.sub(&term, &term).unwrap();
    assert!(c
        .numeric_value(&zero)
        .unwrap()
        .unwrap()
        .value()
        .terms()
        .is_empty());
    let twice = c.scaled(&term, &Rational::integer(IBig::from(2))).unwrap();
    let negative = c.neg(&term).unwrap();
    let zero = c.add(&[twice, negative.clone(), negative]).unwrap();
    assert!(c
        .numeric_value(&zero)
        .unwrap()
        .unwrap()
        .value()
        .terms()
        .is_empty());
    let a = c.mul(&[root2.clone(), sine.clone()]).unwrap();
    let b = c.mul(&[root3.clone(), sine.clone()]).unwrap();
    let combined = c.add(&[a, b]).unwrap();
    let coefficient = c.add(&[root2.clone(), root3]).unwrap();
    let expected = c.mul(&[coefficient, sine]).unwrap();
    assert!(c.same_literal(&combined, &expected).unwrap());
    let pi = c.pi().unwrap();
    let inverse = c.formal_reciprocal(&pi).unwrap();
    let partial = c.mul(&[root2, inverse]).unwrap();
    assert!(matches!(
        c.sub(&partial, &partial),
        Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard))
    ));
}

#[test]
fn retained_token_rejects_foreign_and_same_graph_sequential_constructor_before_zero() {
    let mut graph = Expressions::new();
    let stale = {
        let mut c = ConstructorV1::new(&mut graph);
        q(&mut c, 9, 1)
    };
    let mut c = ConstructorV1::new(&mut graph);
    let zero = q(&mut c, 0, 1);
    assert!(matches!(
        c.mul(&[zero, stale.clone()]),
        Err(ConstructErrorV1::ForeignArena)
    ));
    assert!(matches!(
        c.integer_power(&stale, 0),
        Err(ConstructErrorV1::ForeignArena)
    ));
    assert!(matches!(
        c.export_raw(&stale),
        Err(ConstructErrorV1::ForeignArena)
    ));
    assert!(matches!(
        c.probe_sign(&stale),
        Err(OwnedSignErrorV1::Construction(
            ConstructErrorV1::ForeignArena
        ))
    ));
    let mut othergraph = Expressions::new();
    let mut other = ConstructorV1::new(&mut othergraph);
    let one = q(&mut other, 1, 1);
    assert!(matches!(c.add(&[one]), Err(ConstructErrorV1::ForeignArena)));
}
#[test]
fn closed_gram_unit_basis_principal_ray_consumer_and_old_reader() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let matrix = [
        [q(&mut c, 1, 1), q(&mut c, 0, 1)],
        [q(&mut c, 0, 1), q(&mut c, 3, 1)],
    ];
    let incoming = [q(&mut c, 1, 1), q(&mut c, 1, 1)];
    let outgoing = [q(&mut c, -3, 1), q(&mut c, 1, 1)];
    let rawdot = dot(&mut c, &matrix, &incoming, &outgoing).unwrap();
    assert!(c
        .numeric_value(&rawdot)
        .unwrap()
        .unwrap()
        .value()
        .terms()
        .is_empty());
    let cross = oriented_cross(&mut c, &incoming, &outgoing, 1).unwrap();
    assert_eq!(
        c.numeric_value(&cross).unwrap().unwrap(),
        &ScalarV2::rational(&Rational::integer(IBig::from(4)))
    );
    let basis = oriented_basis(&mut c, &matrix, &incoming, 1).unwrap();
    for vector in &basis {
        let norm = dot(&mut c, &matrix, vector, vector).unwrap();
        assert_eq!(
            c.numeric_value(&norm)
                .unwrap()
                .unwrap()
                .value()
                .as_rational(),
            Some((IBig::ONE, UBig::ONE))
        );
    }
    let perpendicular = dot(&mut c, &matrix, &basis[0], &basis[1]).unwrap();
    assert!(c
        .numeric_value(&perpendicular)
        .unwrap()
        .unwrap()
        .value()
        .terms()
        .is_empty());
    let angle = c.raw_atan2(&cross, &rawdot).unwrap();
    let angle = c
        .scaled(&angle, &Rational::new(IBig::ONE, UBig::from(3u8)).unwrap())
        .unwrap();
    let ray = principal_linear_combination(&mut c, &basis, &angle).unwrap();
    for axis in &ray {
        assert!(c.numeric_value(axis).unwrap().is_none());
        c.export_raw(axis).unwrap();
        c.interval(
            axis,
            cftuv_density::legacy_mpf::Precision::new(160).unwrap(),
        )
        .unwrap();
    }
    // Reader принимает только exported view; он не получает новые numeric/sign правила.
    use cftuv_density::density_sign::SourceSignProbe;
    assert!(matches!(
        c.probe_sign(&cross).unwrap(),
        SourceSignProbe::Decided(_)
    ));
    let zero = [q(&mut c, 0, 1), q(&mut c, 0, 1)];
    assert!(matches!(
        rational_norm_unit(&mut c, &matrix, &zero),
        Err(ConstructErrorV1::NonPositiveNorm)
    ));
    assert!(matches!(
        oriented_cross(&mut c, &incoming, &outgoing, 0),
        Err(ConstructErrorV1::InvalidOrientation)
    ));
}
