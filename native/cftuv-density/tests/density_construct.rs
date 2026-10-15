use cftuv_density::density_construct::{add, mul, ConstructError, UnsupportedProfile};
use cftuv_density::density_construct::{Constructor, SourceSqrtContract};
use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_expression_order::compare;
use cftuv_density::{IBig, Rational, UBig};
fn q(g: &mut Expressions, n: i64, d: u64) -> usize {
    g.push(Node::Rational(
        Rational::new(IBig::from(n), UBig::from(d)).unwrap(),
    ))
    .unwrap()
}
fn eq(g: &Expressions, a: usize, b: usize) {
    assert!(compare(g, a, b).unwrap().is_eq());
}

#[test]
fn rational_and_nested_add_coefficients_cancel_structurally() {
    let mut g = Expressions::new();
    let pi = g.push(Node::Pi).unwrap();
    let half = q(&mut g, 1, 2);
    let third = q(&mut g, 1, 3);
    let minus = q(&mut g, -1, 1);
    let p = g.push(Node::Mul(vec![half, pi])).unwrap();
    let m = g.push(Node::Mul(vec![minus, pi])).unwrap();
    let nested = g.push(Node::Add(vec![pi, p])).unwrap();
    let built = add(&mut g, &[nested, m, third]).unwrap();
    let expected = g.push(Node::Add(vec![third, p])).unwrap();
    eq(&g, built, expected);
    let zero = q(&mut g, 0, 1);
    let cancel = add(&mut g, &[pi, m]).unwrap();
    eq(&g, cancel, zero);
}
#[test]
fn repeated_opaque_bases_collect_integer_powers_and_cancel() {
    let mut g = Expressions::new();
    let pi = g.push(Node::Pi).unwrap();
    let a = g.push(Node::Sin(pi)).unwrap();
    let two = q(&mut g, 2, 1);
    let squared = g.push(Node::Pow(a, two)).unwrap();
    let actual = mul(&mut g, &[a, a]).unwrap();
    eq(&g, actual, squared);
    let minus_two = q(&mut g, -2, 1);
    let inverse = g.push(Node::Pow(a, minus_two)).unwrap();
    let cancel = mul(&mut g, &[squared, inverse]).unwrap();
    let one = q(&mut g, 1, 1);
    eq(&g, cancel, one);
}
#[test]
fn rational_add_distribution_and_two_arg_hack_are_distinct() {
    let mut g = Expressions::new();
    let pi = g.push(Node::Pi).unwrap();
    let a = g.push(Node::Sin(pi)).unwrap();
    let sum = g.push(Node::Add(vec![pi, a])).unwrap();
    let two = q(&mut g, 2, 1);
    let three = q(&mut g, 3, 1);
    let actual = mul(&mut g, &[two, sum]).unwrap();
    let left = g.push(Node::Mul(vec![two, pi])).unwrap();
    let right = g.push(Node::Mul(vec![two, a])).unwrap();
    let expected = g.push(Node::Add(vec![left, right])).unwrap();
    eq(&g, actual, expected);
    let raw = g.push(Node::Mul(vec![three, sum])).unwrap();
    let preserved = mul(&mut g, &[two, raw]).unwrap();
    let six = q(&mut g, 6, 1);
    let expected = g.push(Node::Mul(vec![six, sum])).unwrap();
    eq(&g, preserved, expected);
}
#[test]
fn positive_numeric_power_is_opaque_in_add_but_scaling_and_mul_are_explicitly_unsupported() {
    let mut g = Expressions::new();
    let two = q(&mut g, 2, 1);
    let half = q(&mut g, 1, 2);
    let root = g.push(Node::Pow(two, half)).unwrap();
    let pi = g.push(Node::Pi).unwrap();
    assert!(add(&mut g, &[root, pi]).is_ok());
    assert_eq!(
        mul(&mut g, &[two, root]),
        Err(ConstructError::UnsupportedProfile(
            UnsupportedProfile::NumericPowerInMul(root)
        ))
    );
    assert_eq!(
        add(&mut g, &[root, root]),
        Err(ConstructError::UnsupportedProfile(
            UnsupportedProfile::NumericPowerCoefficientInAdd(root)
        ))
    );
    let minus = q(&mut g, -1, 1);
    let neg = g.push(Node::Mul(vec![minus, root])).unwrap();
    let canceled = add(&mut g, &[root, neg]).unwrap();
    let zero = q(&mut g, 0, 1);
    eq(&g, canceled, zero);
}
#[test]
fn arena_refs_and_unproved_power_profiles_do_not_become_product_outcomes() {
    let mut g = Expressions::new();
    assert!(matches!(add(&mut g, &[0]), Err(ConstructError::Arena(_))));
    let pi = g.push(Node::Pi).unwrap();
    let half = q(&mut g, 1, 2);
    let root = g.push(Node::Pow(pi, half)).unwrap();
    assert_eq!(
        mul(&mut g, &[root, pi]),
        Err(ConstructError::UnsupportedProfile(
            UnsupportedProfile::NonIntegerPowerInMul(root)
        ))
    );
    let forbidden = g.push(Node::Tan(pi)).unwrap();
    let nested = g.push(Node::Add(vec![pi, forbidden])).unwrap();
    assert_eq!(
        add(&mut g, &[nested, pi]),
        Err(ConstructError::UnsupportedProfile(
            UnsupportedProfile::NodeOutsideProfile(forbidden)
        ))
    );
}

#[test]
fn fixed_root_provenance_closes_scaling_and_gcd_products_without_factoring() {
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let r2 = c.fixed_sqrt(2).unwrap();
    let r3 = c.fixed_sqrt(3).unwrap();
    let r6 = c.fixed_sqrt(6).unwrap();
    let two = c
        .push(Node::Rational(Rational::integer(IBig::from(2))))
        .unwrap();
    let doubled = c.add(&[r2, r2]).unwrap();
    let expected = c.push(Node::Mul(vec![two, r2])).unwrap();
    eq(c.graph(), doubled, expected);
    let product = c.mul(&[r2, r3]).unwrap();
    eq(c.graph(), product, r6);
    let product = c.mul(&[r6, r3]).unwrap();
    let three = c
        .push(Node::Rational(Rational::integer(IBig::from(3))))
        .unwrap();
    let expected = c.push(Node::Mul(vec![three, r2])).unwrap();
    eq(c.graph(), product, expected);
    let product = c.mul(&[r2, r3, r6]).unwrap();
    let six = c
        .push(Node::Rational(Rational::integer(IBig::from(6))))
        .unwrap();
    eq(c.graph(), product, six);
    assert_eq!(
        c.fixed_sqrt(8),
        Err(ConstructError::UnsupportedProfile(
            UnsupportedProfile::FixedRootOutsideProfile(8)
        ))
    );
}

#[test]
fn declared_source_contract_checks_shape_and_does_not_certify_raw_or_nested_roots() {
    let mut g = Expressions::new();
    let thirteen = q(&mut g, 13, 1);
    let half = q(&mut g, 1, 2);
    let root = g.push(Node::Pow(thirteen, half)).unwrap();
    let two = q(&mut g, 2, 1);
    assert!(mul(&mut g, &[two, root]).is_err());
    {
        let mut c = Constructor::new(&mut g);
        c.declare_source_sqrt(
            root,
            UBig::from(13u8),
            SourceSqrtContract::Sympy114CanonicalPositiveIntegerSqrt,
        )
        .unwrap();
        let actual = c.mul(&[root, root]).unwrap();
        eq(c.graph(), actual, thirteen);
        assert!(c
            .declare_source_sqrt(
                root,
                UBig::from(17u8),
                SourceSqrtContract::Sympy114CanonicalPositiveIntegerSqrt
            )
            .is_err());
        let eight = c
            .push(Node::Rational(Rational::integer(IBig::from(8))))
            .unwrap();
        let raw8 = c.push(Node::Pow(eight, half)).unwrap();
        let twelve = c
            .push(Node::Rational(Rational::integer(IBig::from(12))))
            .unwrap();
        let raw12 = c.push(Node::Pow(twelve, half)).unwrap();
        for raw in [raw8, raw12] {
            assert!(c.mul(&[two, raw]).is_err());
        }
        let nested = c.push(Node::Pow(root, half)).unwrap();
        assert!(c.mul(&[two, nested]).is_err());
    }
    // Registry не переносится в новый context или free raw API.
    assert!(Constructor::new(&mut g).mul(&[two, root]).is_err());
    assert!(mul(&mut g, &[two, root]).is_err());
}

#[test]
fn normalized_source_sqrt_shape_is_not_a_squarefree_certificate() {
    // Actual SymPy1.14 оставляет sqrt(N) в этой форме после limited factor search.
    // При этом p² делит N; ни primality, ни factorization для опровержения не нужны.
    let p = (UBig::ONE << 89usize) - UBig::ONE;
    let q = (UBig::ONE << 61usize) - UBig::ONE;
    let square = &p * &p;
    let n = &square * &q;
    assert!(p > UBig::ONE);
    assert!((&n % &square).is_zero());
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let base = c
        .push(Node::Rational(Rational::integer(IBig::from(n))))
        .unwrap();
    let half = c
        .push(Node::Rational(
            Rational::new(IBig::ONE, UBig::from(2u8)).unwrap(),
        ))
        .unwrap();
    let raw = c.push(Node::Pow(base, half)).unwrap();
    let two = c
        .push(Node::Rational(Rational::integer(IBig::from(2))))
        .unwrap();
    assert_eq!(c.registered_sqrt_radicand(raw).unwrap(), None);
    assert_eq!(
        c.mul(&[two, raw]),
        Err(ConstructError::UnsupportedProfile(
            UnsupportedProfile::NumericPowerInMul(raw)
        ))
    );
}

#[test]
fn squarefree_gcd_value_closure_is_not_generic_sympy_mul_ast_closure() {
    // Review source SymPy1.14: true squarefree a=q*r,b=p*q сохраняют
    // bare sqrt(p*q²*r) из-за limited factorizer; gcd-value даёт q*sqrt(p*r).
    let p = (UBig::ONE << 89usize) - UBig::ONE;
    let q = (UBig::ONE << 61usize) - UBig::ONE;
    let r = (UBig::ONE << 31usize) - UBig::ONE;
    let a = &q * &r;
    let b = &p * &q;
    let source_radicand = &a * &b;
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let half = c
        .push(Node::Rational(
            Rational::new(IBig::ONE, UBig::from(2u8)).unwrap(),
        ))
        .unwrap();
    let qa = c
        .push(Node::Rational(Rational::integer(IBig::from(a.clone()))))
        .unwrap();
    let qb = c
        .push(Node::Rational(Rational::integer(IBig::from(b.clone()))))
        .unwrap();
    let ra = c.push(Node::Pow(qa, half)).unwrap();
    let rb = c.push(Node::Pow(qb, half)).unwrap();
    for (id, n) in [(ra, a), (rb, b)] {
        c.declare_source_sqrt(
            id,
            n,
            SourceSqrtContract::Sympy114CanonicalPositiveIntegerSqrt,
        )
        .unwrap();
    }
    let native = c.mul(&[ra, rb]).unwrap();
    let qs = c
        .push(Node::Rational(Rational::integer(IBig::from(
            source_radicand,
        ))))
        .unwrap();
    let actual_source = c.push(Node::Pow(qs, half)).unwrap();
    assert!(!compare(c.graph(), native, actual_source).unwrap().is_eq());
    // Этот тест фиксирует staging gap, НЕ продуктовый допуск такого generic input.
}
