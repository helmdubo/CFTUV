use cftuv_density::density_construct::{Constructor, SourceSqrtContract};
use cftuv_density::density_expand::{expand, ExpandError, ExpandLimits, UnsupportedProfile};
use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_expression_order::compare;
use cftuv_density::density_sign::{SignFailure, SourceSignProbe};
use cftuv_density::{IBig, Rational, UBig};
fn limits() -> ExpandLimits {
    ExpandLimits {
        max_terms: 65536,
        max_rational_power_bits: 1 << 20,
        max_polynomial_exponent: 64,
    }
}
fn q(c: &mut Constructor<'_>, n: i64) -> usize {
    c.push(Node::Rational(Rational::integer(IBig::from(n))))
        .unwrap()
}
fn eq(c: &Constructor<'_>, a: usize, b: usize) {
    assert!(compare(c.graph(), a, b).unwrap().is_eq());
}

#[test]
fn polynomial_distribution_preserves_raw_atom_distinctions() {
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let one = q(&mut c, 1);
    let two = q(&mut c, 2);
    let pi = c.push(Node::Pi).unwrap();
    let raw_add = c.push(Node::Add(vec![one, one])).unwrap();
    let result = expand(&mut c, raw_add, limits()).unwrap();
    eq(&c, result, raw_add);
    let raw_power = c.push(Node::Pow(two, two)).unwrap();
    let result = expand(&mut c, raw_power, limits()).unwrap();
    eq(&c, result, raw_power);
    let sum = c.push(Node::Add(vec![one, pi])).unwrap();
    let square = c.push(Node::Pow(sum, two)).unwrap();
    let result = expand(&mut c, square, limits()).unwrap();
    let term = c.push(Node::Mul(vec![two, pi])).unwrap();
    let p2 = c.push(Node::Pow(pi, two)).unwrap();
    let expected = c.add(&[one, term, p2]).unwrap();
    eq(&c, result, expected);
}

#[test]
fn certified_fixed_roots_and_exact_fallback_cancel() {
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let one = q(&mut c, 1);
    let two = q(&mut c, 2);
    let minus = q(&mut c, -1);
    let three = q(&mut c, 3);
    let r2 = c.fixed_sqrt(2).unwrap();
    let sum = c.push(Node::Add(vec![one, r2])).unwrap();
    let square = c.push(Node::Pow(sum, two)).unwrap();
    let n3 = c.push(Node::Mul(vec![minus, three])).unwrap();
    let n2 = q(&mut c, -2);
    let nr = c.push(Node::Mul(vec![n2, r2])).unwrap();
    let raw = c.push(Node::Add(vec![square, n3, nr])).unwrap();
    let expanded = expand(&mut c, raw, limits()).unwrap();
    let zero = q(&mut c, 0);
    eq(&c, expanded, zero);
    assert_eq!(
        c.sign_with_expanded_fallback(raw, expanded).unwrap().sign,
        0
    );
}

#[test]
fn opaque_tan_polynomial_expands_to_zero_without_interval_admission() {
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let one = q(&mut c, 1);
    let minus = q(&mut c, -1);
    let two = q(&mut c, 2);
    let t = c.push(Node::Tan(one)).unwrap();
    assert_eq!(c.probe_sign(t).unwrap(), SourceSignProbe::NeedsExpanded);
    let expanded_t = expand(&mut c, t, limits()).unwrap();
    assert!(matches!(
        c.sign_with_expanded_fallback(t, expanded_t),
        Err(SignFailure::Undecidable)
    ));
    let a = c.push(Node::Add(vec![t, one])).unwrap();
    let b = c.push(Node::Add(vec![t, minus])).unwrap();
    let product = c.push(Node::Mul(vec![a, b])).unwrap();
    let t2 = c.push(Node::Pow(t, two)).unwrap();
    let nt2 = c.push(Node::Mul(vec![minus, t2])).unwrap();
    let raw = c.push(Node::Add(vec![product, nt2, one])).unwrap();
    assert_eq!(c.probe_sign(raw).unwrap(), SourceSignProbe::NeedsExpanded);
    let result = expand(&mut c, raw, limits()).unwrap();
    let zero = q(&mut c, 0);
    eq(&c, result, zero);
    assert_eq!(c.sign_with_expanded_fallback(raw, result).unwrap().sign, 0);
    assert!(c.mul(&[two, t]).is_err()); // Обычный constructor снова strict.
}

#[test]
fn unsupported_expansion_is_independent_of_successful_source_interval() {
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let one = q(&mut c, 1);
    let minus = q(&mut c, -1);
    let pi = c.push(Node::Pi).unwrap();
    let sum = c.push(Node::Add(vec![one, pi])).unwrap();
    let reciprocal = c.push(Node::Pow(sum, minus)).unwrap();
    let SourceSignProbe::Decided(result) = c.probe_sign(reciprocal).unwrap() else {
        panic!("успешный source interval не требует expand");
    };
    assert_eq!(result.sign, 1);
    assert!(matches!(
        expand(&mut c, reciprocal, limits()),
        Err(ExpandError::UnsupportedProfile(
            UnsupportedProfile::NonPolynomialPower { .. }
        ))
    ));
}

#[test]
fn opaque_child_changes_and_huge_polynomial_exponents_refuse_promptly() {
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let one = q(&mut c, 1);
    let two = q(&mut c, 2);
    let minus = q(&mut c, -1);
    let pi = c.push(Node::Pi).unwrap();
    let sum = c.push(Node::Add(vec![one, pi])).unwrap();
    let product = c.push(Node::Mul(vec![two, sum])).unwrap();
    let sin = c.push(Node::Sin(product)).unwrap();
    assert!(matches!(
        expand(&mut c, sin, limits()),
        Err(ExpandError::UnsupportedProfile(
            UnsupportedProfile::OpaqueChildChanges { .. }
        ))
    ));
    let neg = c.push(Node::Mul(vec![minus, pi])).unwrap();
    let cancellation = c.push(Node::Add(vec![pi, neg])).unwrap();
    let huge = q(&mut c, u32::MAX as i64);
    let power = c.push(Node::Pow(cancellation, huge)).unwrap();
    assert!(matches!(
        expand(&mut c, power, limits()),
        Err(ExpandError::UnsupportedProfile(
            UnsupportedProfile::PolynomialExponentLimit {
                requested: u32::MAX,
                ..
            }
        ))
    ));
}

#[test]
fn arbitrary_squarefree_root_product_refuses_instead_of_claiming_source_ast_parity() {
    let p = (UBig::ONE << 89usize) - UBig::ONE;
    let big_q = (UBig::ONE << 61usize) - UBig::ONE;
    let r = (UBig::ONE << 31usize) - UBig::ONE;
    let a = &big_q * &r;
    let b = &p * &big_q;
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let half = c
        .push(Node::Rational(
            Rational::new(IBig::ONE, UBig::from(2u8)).unwrap(),
        ))
        .unwrap();
    let mut roots = Vec::new();
    for n in [a, b] {
        let base = c
            .push(Node::Rational(Rational::integer(IBig::from(n.clone()))))
            .unwrap();
        let root = c.push(Node::Pow(base, half)).unwrap();
        c.declare_source_sqrt(
            root,
            n,
            SourceSqrtContract::Sympy114CanonicalPositiveIntegerSqrt,
        )
        .unwrap();
        assert_eq!(expand(&mut c, root, limits()).unwrap(), root); // Только bare leaf passthrough.
        roots.push(root);
    }
    let product = c.push(Node::Mul(roots)).unwrap();
    assert!(matches!(
        expand(&mut c, product, limits()),
        Err(ExpandError::UnsupportedProfile(
            UnsupportedProfile::ArbitraryRootAlgebra { .. }
        ))
    ));
}

#[test]
fn fixed_root_family_products_remain_in_the_small_closed_family() {
    let mut g = Expressions::new();
    let mut c = Constructor::new(&mut g);
    let r2 = c.fixed_sqrt(2).unwrap();
    let r3 = c.fixed_sqrt(3).unwrap();
    let r5 = c.fixed_sqrt(5).unwrap();
    let r6 = c.fixed_sqrt(6).unwrap();
    let product = c.push(Node::Mul(vec![r2, r3, r5])).unwrap();
    let r30 = expand(&mut c, product, limits()).unwrap();
    assert_eq!(
        c.registered_sqrt_radicand(r30).unwrap(),
        Some(UBig::from(30u8))
    );
    let second = c.push(Node::Mul(vec![r30, r6])).unwrap();
    let actual = expand(&mut c, second, limits()).unwrap();
    let six = q(&mut c, 6);
    let expected = c.mul(&[six, r5]).unwrap();
    eq(&c, actual, expected);
}
