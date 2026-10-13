use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_sign::{SignEvaluation, SignFailure, SignPath};
use cftuv_density::{IBig, Rational, UBig};
fn q(g: &mut Expressions, n: i32, d: u32) -> usize {
    g.push(Node::Rational(
        Rational::new(IBig::from(n), UBig::from(d)).unwrap(),
    ))
    .unwrap()
}
#[test]
fn interval_short_circuit_and_exact_zero_are_separate_and_transaction_local() {
    let mut g = Expressions::new();
    let one = q(&mut g, 1, 1);
    let two = q(&mut g, 2, 1);
    let half = q(&mut g, 1, 2);
    let minus = q(&mut g, -1, 1);
    let root = g.push(Node::Pow(two, half)).unwrap();
    let negative = g.push(Node::Mul(vec![minus, root])).unwrap();
    let zero = g.push(Node::Add(vec![root, negative])).unwrap();
    let sine = g.push(Node::Sin(one)).unwrap();
    let mut eval = SignEvaluation::new(&g);
    let positive = eval.sign_with_expanded_fallback(sine, 9999).unwrap();
    assert_eq!((positive.sign, positive.path), (1, SignPath::Interval));
    let result = eval.sign_with_expanded_fallback(zero, zero).unwrap();
    assert_eq!((result.sign, result.path), (0, SignPath::Quadratic));
    assert_eq!(
        eval.sign_with_expanded_fallback(zero, zero).unwrap().path,
        SignPath::Memo
    );
    assert_eq!(
        SignEvaluation::new(&g)
            .sign_with_expanded_fallback(zero, zero)
            .unwrap()
            .path,
        SignPath::Quadratic
    );
}
#[test]
fn source_caught_failure_may_use_expansion_but_profile_limit_never_becomes_undecidable() {
    let mut g = Expressions::new();
    let minus = q(&mut g, -1, 1);
    let zero = q(&mut g, 0, 1);
    let one = q(&mut g, 1, 1);
    let two = q(&mut g, 2, 1);
    let nine = q(&mut g, 9, 1);
    let tan = g.push(Node::Tan(one)).unwrap();
    let plus = g.push(Node::Add(vec![tan, one])).unwrap();
    let less = g.push(Node::Add(vec![tan, minus])).unwrap();
    let product = g.push(Node::Mul(vec![plus, less])).unwrap();
    let square = g.push(Node::Pow(tan, two)).unwrap();
    let negative = g.push(Node::Mul(vec![minus, square])).unwrap();
    let polynomial = g.push(Node::Add(vec![product, negative, one])).unwrap();
    let cosine = g.push(Node::Cos(nine)).unwrap();
    let mut eval = SignEvaluation::new(&g);
    let expanded = eval.sign_with_expanded_fallback(polynomial, zero).unwrap();
    assert_eq!((expanded.sign, expanded.path), (0, SignPath::Quadratic));
    assert_eq!(
        eval.sign_with_expanded_fallback(tan, tan),
        Err(SignFailure::Undecidable)
    );
    assert!(matches!(
        eval.sign_with_expanded_fallback(cosine, cosine),
        Err(SignFailure::IntervalInternal(_))
    ));
}

#[test]
fn growing_arena_keeps_value_and_admission_memos_separate() {
    let mut g = Expressions::new();
    let two = q(&mut g, 2, 1);
    let half = q(&mut g, 1, 2);
    let minus = q(&mut g, -1, 1);
    let root = g.push(Node::Pow(two, half)).unwrap();
    let negative = g.push(Node::Mul(vec![minus, root])).unwrap();
    let zero = g.push(Node::Add(vec![root, negative])).unwrap();
    assert_eq!(
        g.sign_with_expanded_fallback(zero, zero).unwrap().path,
        SignPath::Quadratic
    );
    let one = q(&mut g, 1, 1);
    let plus = g.push(Node::Add(vec![root, one])).unwrap();
    assert_eq!(g.sign_with_expanded_fallback(plus, plus).unwrap().sign, 1);
    assert_eq!(
        g.sign_with_expanded_fallback(zero, zero).unwrap().path,
        SignPath::Memo
    );
    assert_eq!(g.quadratic(root).unwrap().1, cftuv_density::Work::NONE);
    // Равное root значение не даёт raw reader права принять другой grammar.
    let sum = g.push(Node::Add(vec![one, one])).unwrap();
    let nested = g.push(Node::Pow(sum, half)).unwrap();
    assert!(matches!(
        g.quadratic(nested).unwrap_err().kind,
        cftuv_density::density_quadratic::QuadraticFailureKind::UnsupportedSyntax(_)
    ));
    let opaque = g.push(Node::Tan(one)).unwrap();
    assert_eq!(
        g.sign_with_expanded_fallback(opaque, opaque),
        Err(SignFailure::Undecidable)
    );
    assert_eq!(
        g.sign_with_expanded_fallback(opaque, opaque),
        Err(SignFailure::Undecidable)
    );
    // Внешний cold reader и новая arena не получают cache предыдущей транзакции.
    assert_eq!(
        SignEvaluation::new(&g)
            .sign_with_expanded_fallback(zero, zero)
            .unwrap()
            .path,
        SignPath::Quadratic
    );
    g = Expressions::new();
    let value = q(&mut g, -1, 1);
    assert_eq!(
        g.sign_with_expanded_fallback(value, value).unwrap().sign,
        -1
    );
}
