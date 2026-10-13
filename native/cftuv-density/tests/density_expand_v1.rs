use cftuv_density::density_construct_v1::*;
use cftuv_density::density_expand_v1::*;
use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_sign::{SignFailure, SignPath};
use cftuv_density::scalar_v2::{ProfileCode as ScalarProfile, ScalarError};
use cftuv_density::{IBig, Rational, UBig};
use cftuv_radical::Failure;
fn q(c: &mut ConstructorV1<'_>, n: i64, d: u64) -> OwnedExprV1 {
    c.rational(&Rational::new(IBig::from(n), UBig::from(d)).unwrap())
        .unwrap()
}
fn limits() -> ExpandLimits {
    ExpandLimits {
        max_terms: 256,
        max_polynomial_exponent: 16,
        max_rational_power_bits: 4096,
    }
}
#[test]
fn formal_reciprocal_domain_is_not_cancelled_or_hidden_in_descendants() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let zero = q(&mut c, 0, 1);
    let opaquezero = c.raw_sin(&zero).unwrap();
    let reciprocal = c.formal_reciprocal(&opaquezero).unwrap();
    assert!(c.contains_partial_reciprocal(&reciprocal).unwrap());
    let ratio = c.partial_div(&opaquezero, &opaquezero).unwrap();
    assert!(matches!(
        c.graph().node(c.raw_id(&ratio).unwrap()).unwrap(),
        Node::Mul(_)
    ));
    let one = q(&mut c, 1, 1);
    let added = c.add(&[reciprocal.clone(), one]).unwrap();
    for expression in [
        reciprocal.clone(),
        c.raw_sin(&reciprocal).unwrap(),
        c.sqrt(&reciprocal).unwrap(),
        added,
    ] {
        assert!(matches!(
            c.mul(&[zero.clone(), expression.clone()]),
            Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard))
        ));
        assert!(matches!(
            c.sub(&expression, &expression),
            Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard))
        ));
        assert!(matches!(
            c.integer_power(&expression, 0),
            Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard))
        ));
    }
    let twice = c.mul(&[reciprocal.clone(), reciprocal.clone()]).unwrap();
    assert!(
        matches!(c.graph().node(c.raw_id(&twice).unwrap()).unwrap(),Node::Mul(args) if args==&vec![c.raw_id(&reciprocal).unwrap();2])
    );
    assert!(matches!(
        c.formal_reciprocal(&zero),
        Err(ConstructErrorV1::Scalar(ScalarError::Algebra(
            Failure::ZeroDivision
        )))
    ));
    let mut roots = Vec::new();
    for n in [2, 3, 5] {
        let value = q(&mut c, n, 1);
        roots.push(c.sqrt(&value).unwrap());
    }
    let three = c.add(&roots).unwrap();
    let square = c.integer_power(&three, 2).unwrap();
    assert!(
        matches!(c.formal_reciprocal(&square),Err(ConstructErrorV1::Scalar(ScalarError::Profile(p))) if p.code==ScalarProfile::InverseClassLimit)
    );
    assert!(matches!(
        c.raw_opaque_pow(
            &opaquezero,
            &Rational::new(-IBig::ONE, UBig::from(2u8)).unwrap()
        ),
        Err(ConstructErrorV1::Profile(
            ProfileCode::FreshNegativeOpaquePower
        ))
    ));
}
#[test]
fn polynomial_cancellation_only_after_probe_needs_expanded() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let pi = c.pi().unwrap();
    let angle = c
        .scaled(&pi, &Rational::new(IBig::ONE, UBig::from(7u8)).unwrap())
        .unwrap();
    let a = c.raw_sin(&angle).unwrap();
    let one = q(&mut c, 1, 1);
    let plus = c.add(&[a.clone(), one.clone()]).unwrap();
    let minus = c.sub(&a, &one).unwrap();
    let product = c.mul(&[plus, minus]).unwrap();
    let square = c.integer_power(&a, 2).unwrap();
    let negative = c.neg(&square).unwrap();
    let source = c.add(&[product, negative, one]).unwrap();
    assert!(matches!(
        c.probe_sign(&source).unwrap(),
        cftuv_density::density_sign::SourceSignProbe::NeedsExpanded
    ));
    let result = lazy_sign(&mut c, &source, limits()).unwrap();
    assert_eq!(
        (result.result.sign, result.result.path),
        (0, SignPath::Quadratic)
    );
    let expanded = result.expanded.unwrap();
    assert!(expanded.stats.generated_products > 0);
    assert!(c
        .numeric_value(&expanded.expression)
        .unwrap()
        .unwrap()
        .value()
        .is_zero());
    // Direct raw sin(0) не становится Numeric0 по новой trig identity.
    let zero = q(&mut c, 0, 1);
    let sinezero = c.raw_sin(&zero).unwrap();
    assert!(matches!(
        lazy_sign(&mut c, &sinezero, limits()),
        Err(LazySignErrorV1::Sign(OwnedSignErrorV1::Sign(
            SignFailure::Undecidable
        )))
    ));
}
#[test]
fn successful_interval_probe_never_calls_unsupported_expand_even_with_zero_caps() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let pi = c.pi().unwrap();
    let small = c
        .scaled(&pi, &Rational::new(IBig::ONE, UBig::from(16u8)).unwrap())
        .unwrap();
    let epsilon = q(&mut c, 1, 100);
    let plus = c.add(&[small.clone(), epsilon.clone()]).unwrap();
    let minus = c.sub(&small, &epsilon).unwrap();
    let argument = c.mul(&[plus, minus]).unwrap();
    let source = c.raw_sin(&argument).unwrap();
    assert!(matches!(
        expand(&mut c, &source, limits()),
        Err(ExpandErrorV1::Profile(ExpandProfileV1::OpaqueChildChanges))
    ));
    let result = lazy_sign(
        &mut c,
        &source,
        ExpandLimits {
            max_terms: 0,
            max_polynomial_exponent: 0,
            max_rational_power_bits: 0,
        },
    )
    .unwrap();
    assert_eq!(
        (result.result.sign, result.result.path),
        (1, SignPath::Interval)
    );
    assert!(result.expanded.is_none());
    let repeat = lazy_sign(
        &mut c,
        &source,
        ExpandLimits {
            max_terms: 0,
            max_polynomial_exponent: 0,
            max_rational_power_bits: 0,
        },
    )
    .unwrap();
    assert_eq!(
        (repeat.result.sign, repeat.result.path),
        (1, SignPath::Memo)
    );
    assert!(repeat.expanded.is_none());
    let one = q(&mut c, 1, 1);
    for _ in 0..2 {
        assert_eq!(
            lazy_sign(
                &mut c,
                &one,
                ExpandLimits {
                    max_terms: 0,
                    max_polynomial_exponent: 0,
                    max_rational_power_bits: 0
                }
            )
            .unwrap()
            .result
            .path,
            SignPath::Rational
        );
    }
}
#[test]
fn numeric_sum_exposed_only_at_multiplication_boundary_and_caps_bound_uncoalesced_choices() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let two = q(&mut c, 2, 1);
    let three = q(&mut c, 3, 1);
    let r2 = c.sqrt(&two).unwrap();
    let r3 = c.sqrt(&three).unwrap();
    let numeric = c.add(&[r2.clone(), r3]).unwrap();
    let expanded = expand(&mut c, &numeric, limits()).unwrap();
    assert!(c.same_literal(&numeric, &expanded.expression).unwrap());
    let pi = c.pi().unwrap();
    let a = c.raw_sin(&pi).unwrap();
    let b = c.raw_cos(&pi).unwrap();
    let product = c.mul(&[numeric, a.clone()]).unwrap();
    let expanded = expand(&mut c, &product, limits()).unwrap();
    assert!(c.same_literal(&product, &expanded.expression).unwrap());
    assert!(expanded.stats.generated_products >= 2);
    // Временные choices остаются двумя, хотя Scalar coefficients собираются обратно.
    assert!(matches!(
        expand(
            &mut c,
            &product,
            ExpandLimits {
                max_terms: 1,
                ..limits()
            }
        ),
        Err(ExpandErrorV1::Profile(ExpandProfileV1::TermLimit { .. }))
    ));
    let sum = c.add(&[a.clone(), b.clone()]).unwrap();
    let left = c.mul(&[sum, pi.clone()]).unwrap();
    let right = c.mul(&[a, pi.clone()]).unwrap();
    let reducible = c.sub(&left, &right).unwrap();
    let outside = c.mul(&[reducible.clone(), r2]).unwrap();
    for cap in [0, 1, 2] {
        assert!(matches!(
            expand(
                &mut c,
                &outside,
                ExpandLimits {
                    max_terms: cap,
                    ..limits()
                }
            ),
            Err(ExpandErrorV1::Profile(ExpandProfileV1::TermLimit { .. }))
        ));
    }
    let expanded = expand(
        &mut c,
        &outside,
        ExpandLimits {
            max_terms: 3,
            ..limits()
        },
    )
    .unwrap();
    assert!(c.numeric_value(&expanded.expression).unwrap().is_none());
    let power = c.integer_power(&reducible, 2).unwrap();
    assert!(matches!(
        expand(
            &mut c,
            &power,
            ExpandLimits {
                max_terms: 8,
                ..limits()
            }
        ),
        Err(ExpandErrorV1::Profile(ExpandProfileV1::TermLimit {
            requested: 9,
            ..
        }))
    ));
    expand(
        &mut c,
        &power,
        ExpandLimits {
            max_terms: 9,
            ..limits()
        },
    )
    .unwrap();
    let huge = c.integer_power(&b, 1 << 20).unwrap();
    assert!(matches!(
        expand(&mut c, &huge, limits()),
        Err(ExpandErrorV1::Profile(
            ExpandProfileV1::PolynomialExponentLimit { .. }
        ))
    ));
}
#[test]
fn half_powers_and_partial_reciprocal_children_pass_only_when_unchanged() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let pi = c.pi().unwrap();
    let a = c.raw_sin(&pi).unwrap();
    let b = c.raw_cos(&pi).unwrap();
    let sum = c.add(&[a.clone(), b]).unwrap();
    let reciprocal = c.formal_reciprocal(&sum).unwrap();
    let expanded = expand(&mut c, &reciprocal, limits()).unwrap();
    assert!(c.same_literal(&reciprocal, &expanded.expression).unwrap());
    let half = c.sqrt(&a).unwrap();
    let explicit = c.integer_power(&half, 2).unwrap();
    let expanded = expand(&mut c, &explicit, limits()).unwrap();
    let expected = c.mul(&[half.clone(), half]).unwrap();
    assert!(c.same_literal(&expanded.expression, &expected).unwrap());
    let denom = c.mul(&[sum, pi]).unwrap();
    let inverse = c.formal_reciprocal(&denom).unwrap();
    assert!(matches!(
        expand(&mut c, &inverse, limits()),
        Err(ExpandErrorV1::Profile(ExpandProfileV1::OpaqueChildChanges))
    ));
    let two = q(&mut c, 2, 1);
    let rationalproduct = c.mul(&[two.clone(), two]).unwrap();
    let pi = c.pi().unwrap();
    let mixed = c.mul(&[rationalproduct, pi]).unwrap();
    assert!(matches!(
        expand(
            &mut c,
            &mixed,
            ExpandLimits {
                max_rational_power_bits: 0,
                ..limits()
            }
        ),
        Err(ExpandErrorV1::Profile(
            ExpandProfileV1::RationalPowerBits { .. }
        ))
    ));
}

#[test]
fn old_q5_and_nested_pi_over_eight_root_producer_norms_close_without_trig_identity() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let [cosine, sine] = cftuv_density::density_trig_v1::owned_exact_q_trig(&mut c, 5).unwrap();
    let cos2 = c.mul(&[cosine.clone(), cosine.clone()]).unwrap();
    let sin2 = c.mul(&[sine.clone(), sine]).unwrap();
    let norm = c.add(&[cos2.clone(), sin2]).unwrap();
    assert_eq!(
        c.numeric_value(&norm)
            .unwrap()
            .and_then(|v| v.value().as_rational()),
        Some((IBig::ONE, UBig::ONE))
    );
    let five = q(&mut c, 5, 1);
    let root5 = c.sqrt(&five).unwrap();
    let three = q(&mut c, 3, 1);
    let scale = c.add(&[three, root5]).unwrap();
    let first = c.scaled(&cos2, &Rational::integer(IBig::from(8))).unwrap();
    let second = c.mul(&[scale, norm]).unwrap();
    let boundary = c.sub(&first, &second).unwrap();
    assert_eq!(
        lazy_sign(&mut c, &boundary, limits()).unwrap().result.sign,
        0
    );
    // Actual SymPy pi/8 producer formula: sqrt(2 +/- sqrt2)/2, не generic cos²+sin².
    let two = q(&mut c, 2, 1);
    let root2 = c.sqrt(&two).unwrap();
    let plus = c.add(&[two.clone(), root2.clone()]).unwrap();
    let minus = c.sub(&two, &root2).unwrap();
    let nested_cos = c.sqrt(&plus).unwrap();
    let nested_sin = c.sqrt(&minus).unwrap();
    let nested_cos = c
        .scaled(
            &nested_cos,
            &Rational::new(IBig::ONE, UBig::from(2u8)).unwrap(),
        )
        .unwrap();
    let nested_sin = c
        .scaled(
            &nested_sin,
            &Rational::new(IBig::ONE, UBig::from(2u8)).unwrap(),
        )
        .unwrap();
    let a = c.mul(&[nested_cos.clone(), nested_cos]).unwrap();
    let b = c.mul(&[nested_sin.clone(), nested_sin]).unwrap();
    let norm = c.add(&[a, b]).unwrap();
    assert_eq!(
        c.numeric_value(&norm)
            .unwrap()
            .and_then(|v| v.value().as_rational()),
        Some((IBig::ONE, UBig::ONE))
    );
    // Raw trig view сохраняет старую grammar boundary, даже на этом special angle.
    let pi = c.pi().unwrap();
    let angle = c
        .scaled(&pi, &Rational::new(IBig::ONE, UBig::from(8u8)).unwrap())
        .unwrap();
    let cosine = c.raw_cos(&angle).unwrap();
    let sine = c.raw_sin(&angle).unwrap();
    let a = c.mul(&[cosine.clone(), cosine]).unwrap();
    let b = c.mul(&[sine.clone(), sine]).unwrap();
    let raw_norm = c.add(&[a, b]).unwrap();
    let one = q(&mut c, 1, 1);
    let raw_residual = c.sub(&raw_norm, &one).unwrap();
    assert!(matches!(
        lazy_sign(&mut c, &raw_residual, limits()),
        Err(LazySignErrorV1::Sign(OwnedSignErrorV1::Sign(
            SignFailure::Undecidable
        )))
    ));
}
