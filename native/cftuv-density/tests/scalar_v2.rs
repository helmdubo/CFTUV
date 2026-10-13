use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_quadratic::QuadraticReader;
use cftuv_density::scalar_v2::{
    LiteralNode as N, ProfileCode, ScalarError, ScalarLiteralAstV1, ScalarV2,
};
use cftuv_density::{IBig, Rational, UBig};
use cftuv_radical::Failure;
fn q(n: i64, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn value(n: i64) -> ScalarV2 {
    ScalarV2::rational(&q(n, 1))
}
fn root(n: i64) -> ScalarV2 {
    ScalarV2::sqrt_rational(&q(n, 1)).unwrap()
}
fn check(s: &ScalarV2) -> ScalarLiteralAstV1 {
    let ast = s.literal_ast();
    let mut graph = Expressions::new();
    for row in ast.nodes() {
        let node = match row {
            N::Q(n, d) => Node::Rational(Rational::new(n.clone(), d.clone()).unwrap()),
            N::Pow { base, exponent } => Node::Pow(*base, *exponent),
            N::Mul { coefficient, root } => Node::Mul(vec![*coefficient, *root]),
            N::Add(args) => Node::Add(args.clone()),
        };
        graph.push(node).unwrap();
    }
    assert_eq!(
        &QuadraticReader::new(&graph).read(ast.root()).unwrap().0,
        s.value()
    );
    ast
}
#[test]
fn exact_literal_zero_identity_negative_coefficient_and_first_encounter_order() {
    assert_eq!(
        value(0).literal_ast().nodes(),
        &[N::Q(IBig::ZERO, UBig::ONE)]
    );
    assert_eq!(
        root(2).literal_ast().nodes(),
        &[
            N::Q(IBig::from(2), UBig::ONE),
            N::Q(IBig::ONE, UBig::from(2u8)),
            N::Pow {
                base: 0,
                exponent: 1
            }
        ]
    );
    let negative = check(&root(2).neg());
    assert_eq!(
        negative.nodes(),
        &[
            N::Q(IBig::NEG_ONE, UBig::ONE),
            N::Q(IBig::from(2), UBig::ONE),
            N::Q(IBig::ONE, UBig::from(2u8)),
            N::Pow {
                base: 1,
                exponent: 2
            },
            N::Mul {
                coefficient: 0,
                root: 3
            }
        ]
    );
    let half = ScalarV2::rational(&q(1, 2));
    let s = root(3).add(&half).add(&root(2));
    let ast = check(&s);
    assert_eq!(
        ast.nodes(),
        &[
            N::Q(IBig::ONE, UBig::from(2u8)),
            N::Q(IBig::from(2), UBig::ONE),
            N::Pow {
                base: 1,
                exponent: 0
            },
            N::Q(IBig::from(3), UBig::ONE),
            N::Pow {
                base: 3,
                exponent: 0
            },
            N::Add(vec![0, 2, 4])
        ]
    );
    assert_eq!(ast, root(2).add(&root(3)).add(&half).literal_ast());
}
#[test]
fn canonical_value_handles_large_squarefree_products_and_hidden_square_factors() {
    let p = (UBig::ONE << 89usize) - UBig::ONE;
    let big_q = (UBig::ONE << 61usize) - UBig::ONE;
    let r = (UBig::ONE << 31usize) - UBig::ONE;
    let sqrt = |n: UBig| ScalarV2::sqrt_rational(&Rational::integer(IBig::from(n))).unwrap();
    let a = sqrt(&big_q * &r).mul(&sqrt(&p * &big_q));
    let b = sqrt(&p * &r).scaled(&Rational::integer(IBig::from(big_q.clone())));
    let c = sqrt(&p * &big_q * &big_q * &r);
    assert_eq!(a, b);
    assert_eq!(b, c);
    assert_eq!(check(&a), check(&b));
    assert_eq!(check(&b), check(&c));
    let hidden = sqrt(&p * &p * &big_q);
    let explicit = sqrt(big_q).scaled(&Rational::integer(IBig::from(p)));
    assert_eq!(check(&hidden), check(&explicit)); // Никакого source-AST parity утверждения.
}
#[test]
fn whole_value_root_cancellation_and_negative_power_profiles_are_separate() {
    let zero = root(18).sub(&root(2).scaled(&q(3, 1)));
    assert_eq!(zero, value(0));
    assert_eq!(zero.sqrt().unwrap(), zero);
    assert_eq!(root(2).integer_power(2).unwrap().sqrt().unwrap(), root(2));
    for outside in [value(-1), value(2).add(&root(2))] {
        assert!(
            matches!(outside.sqrt(),Err(ScalarError::Profile(ref v)) if v.code==ProfileCode::RootRequiresNonnegativeRational)
        );
    }
    assert_eq!(
        zero.inverse(),
        Err(ScalarError::Algebra(Failure::ZeroDivision))
    );
    let three = root(2).add(&root(3)).add(&root(5));
    check(&three.integer_power(-2).unwrap());
    let four = three.add(&root(7));
    assert!(
        matches!(four.integer_power(-2),Err(ScalarError::Profile(ref v)) if v.code==ProfileCode::InverseClassLimit)
    );
    assert!(matches!(
        value(0).integer_power(1048577),
        Err(ScalarError::ExponentProfile { .. })
    ));
    assert!(matches!(
        value(0).integer_power(i64::MIN),
        Err(ScalarError::ExponentProfile { .. })
    ));
}
#[test]
fn generated_gram_unit_and_principal_squared_root_recipe_is_exact() {
    let g = [[value(2), value(1)], [value(1), value(3)]];
    let left = [value(1), value(2)];
    let right = [value(-2), value(1)];
    let dot = |a: &[ScalarV2; 2], b: &[ScalarV2; 2]| {
        a[0].mul(&g[0][0].mul(&b[0]).add(&g[0][1].mul(&b[1])))
            .add(&a[1].mul(&g[1][0].mul(&b[0]).add(&g[1][1].mul(&b[1]))))
    };
    let ll = dot(&left, &left);
    let rr = dot(&right, &right);
    assert_eq!(ll, value(18));
    assert_eq!(rr, value(7));
    let length = ll.sqrt().unwrap();
    let unit = [left[0].div(&length).unwrap(), left[1].div(&length).unwrap()];
    assert_eq!(dot(&unit, &unit), value(1));
    let cosine2 = dot(&left, &right)
        .integer_power(2)
        .unwrap()
        .div(&ll.mul(&rr))
        .unwrap();
    let cosine = cosine2.sqrt().unwrap();
    let sine = value(1).sub(&cosine2).sqrt().unwrap();
    assert_eq!(
        cosine
            .integer_power(2)
            .unwrap()
            .add(&sine.integer_power(2).unwrap()),
        value(1)
    );
    check(&cosine);
    check(&sine);
    check(&unit[0]);
    check(&unit[1]);
}
