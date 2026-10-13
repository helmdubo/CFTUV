use cftuv_density::density_expression::{ExpressionError, Expressions, Node};
use cftuv_density::density_residual::filtered_sign;
use cftuv_density::legacy_mpf::Precision;
use cftuv_density::{IBig, Rational, UBig};

fn number(g: &mut Expressions, n: i32, d: UBig) -> usize {
    g.push(Node::Rational(Rational::new(IBig::from(n), d).unwrap()))
        .unwrap()
}

#[test]
fn fixed_margin_yields_at_boundary_and_does_not_admit_non_add() {
    let mut g = Expressions::new();
    let zero = number(&mut g, 0, UBig::ONE);
    let one = number(&mut g, 1, UBig::ONE);
    let half = number(&mut g, 1, UBig::from(2u8));
    let tiny = number(&mut g, 1, UBig::ONE << 110usize);
    let large = number(&mut g, 1, UBig::ONE << 80usize);
    let boundary = g.push(Node::Add(vec![half, zero])).unwrap();
    let near = g.push(Node::Add(vec![half, tiny])).unwrap();
    let positive = g.push(Node::Add(vec![half, large])).unwrap();
    let negative = g.push(Node::Add(vec![zero, zero])).unwrap();
    assert_eq!(filtered_sign(&mut g, boundary, one, one, 3).unwrap(), None);
    assert_eq!(filtered_sign(&mut g, near, one, one, 3).unwrap(), None);
    assert_eq!(
        filtered_sign(&mut g, positive, one, one, 3).unwrap(),
        Some(1)
    );
    assert_eq!(
        filtered_sign(&mut g, negative, one, one, 3).unwrap(),
        Some(-1)
    );
    assert_eq!(
        filtered_sign(&mut g, one, usize::MAX, usize::MAX, 3).unwrap(),
        None
    );
    assert_eq!(
        filtered_sign(&mut g, usize::MAX, usize::MAX, usize::MAX, 2).unwrap(),
        None
    );
}

#[test]
fn unsupported_source_yields_but_invalid_arena_is_internal_and_order_is_preserved() {
    let mut g = Expressions::new();
    let one = number(&mut g, 1, UBig::ONE);
    let opaque = g.push(Node::Tan(one)).unwrap();
    let dot = g.push(Node::Add(vec![one, opaque])).unwrap();
    assert_eq!(
        filtered_sign(&mut g, dot, usize::MAX, usize::MAX, 5).unwrap(),
        None
    );
    assert!(g.interval_cached(one, Precision::new(160).unwrap()));
    let good = g.push(Node::Add(vec![one, one])).unwrap();
    assert_eq!(
        filtered_sign(&mut g, good, usize::MAX, one, 5),
        Err(ExpressionError::InvalidReference(usize::MAX))
    );
}
