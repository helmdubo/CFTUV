use cftuv_density::density_expression::{ExpressionError, Expressions, Node};
use cftuv_density::legacy_mpf::{Mpf, Precision, Rounding};
use cftuv_density::{IBig, Rational, UBig};

fn integer(g: &mut Expressions, n: i64) -> usize {
    g.push(Node::Rational(Rational::integer(IBig::from(n))))
        .unwrap()
}
fn value(n: i64) -> Mpf {
    Mpf::from_man_exp(IBig::from(n), 0, None, Rounding::TowardZero).unwrap()
}

#[test]
fn equal_value_different_order_retains_different_legacy_intervals() {
    let mut g = Expressions::new();
    let four = integer(&mut g, 4);
    let one = integer(&mut g, 1);
    let minus_four = integer(&mut g, -4);
    let a = g.push(Node::Add(vec![four, one, minus_four])).unwrap();
    let b = g.push(Node::Add(vec![four, minus_four, one])).unwrap();
    let mut evaluation = g.evaluation(Precision::new(2).unwrap());
    let left = evaluation.interval(a).unwrap();
    let right = evaluation.interval(b).unwrap();
    assert_eq!(left.lower(), &value(0));
    assert_eq!(left.upper(), &value(2));
    assert_eq!(right.lower(), &value(1));
    assert_eq!(right.upper(), &value(1));
    assert!(evaluation.cached(a));
    assert!(!g.evaluation(Precision::new(2).unwrap()).cached(a));
}

#[test]
fn unsupported_pow_evaluates_base_first_but_tan_does_not() {
    let mut g = Expressions::new();
    let four = integer(&mut g, 4);
    let third = g
        .push(Node::Rational(
            Rational::new(IBig::ONE, UBig::from(3u8)).unwrap(),
        ))
        .unwrap();
    let power = g.push(Node::Pow(four, third)).unwrap();
    let tan = g.push(Node::Tan(four)).unwrap();
    let p = Precision::new(160).unwrap();
    let mut evaluation = g.evaluation(p);
    assert_eq!(
        evaluation.interval(power),
        Err(ExpressionError::UnsupportedExpression(power))
    );
    assert!(evaluation.cached(four));
    assert!(!evaluation.cached(third));
    assert!(!evaluation.cached(power));
    let mut fresh = g.evaluation(p);
    assert_eq!(
        fresh.interval(tan),
        Err(ExpressionError::UnsupportedExpression(tan))
    );
    assert!(!fresh.cached(four));
}

#[test]
fn graph_rejects_forward_references_and_bounds_stack_depth() {
    let mut g = Expressions::new();
    assert_eq!(
        g.push(Node::Sin(0)),
        Err(ExpressionError::InvalidReference(0))
    );
    let mut last = integer(&mut g, 1);
    for _ in 1..512 {
        last = g.push(Node::Sin(last)).unwrap();
    }
    assert_eq!(g.push(Node::Sin(last)), Err(ExpressionError::DepthLimit));
    assert_eq!(
        g.push(Node::Add(vec![last])),
        Err(ExpressionError::InvalidArity)
    );
    assert_eq!(
        g.evaluation(Precision::new(160).unwrap()).interval(9999),
        Err(ExpressionError::InvalidReference(9999))
    );
}

#[test]
fn append_transaction_retains_children_after_failure_and_separates_precision() {
    let mut graph = Expressions::new();
    let one = integer(&mut graph, 1);
    let three = integer(&mut graph, 3);
    let third = graph
        .push(Node::Rational(
            Rational::new(IBig::ONE, UBig::from(3u8)).unwrap(),
        ))
        .unwrap();
    let p = Precision::new(160).unwrap();
    let before = graph.interval(third, p).unwrap();
    assert!(graph.interval_cached(third, p));
    let bad = graph.push(Node::Pow(three, third)).unwrap();
    assert_eq!(
        graph.interval(bad, p),
        Err(ExpressionError::UnsupportedExpression(bad))
    );
    assert!(graph.interval_cached(three, p));
    assert!(graph.interval_cached(third, p));
    assert!(!graph.interval_cached(bad, p));
    let sum = graph.push(Node::Add(vec![one, third])).unwrap();
    assert_eq!(
        graph.interval(sum, p).unwrap(),
        graph.evaluation(p).interval(sum).unwrap()
    );
    assert_eq!(graph.interval(third, p).unwrap(), before);
    let low = Precision::new(2).unwrap();
    assert!(!graph.interval_cached(third, low));
    assert_ne!(graph.interval(third, low).unwrap(), before);
    assert!(!graph.interval_cached(sum, p));
    assert_eq!(graph.interval(third, p).unwrap(), before);
    // Полная замена arena заменяет её память; прежний NodeId не переносит результат.
    graph = Expressions::new();
    let new = integer(&mut graph, -7);
    assert_eq!(new, one);
    assert!(!graph.interval_cached(new, p));
    assert_eq!(graph.interval(new, p).unwrap().lower(), &value(-7));
}

#[test]
fn structural_keys_match_duplicate_trees_without_merging_equal_values_or_order() {
    let mut graph = Expressions::new();
    let one = integer(&mut graph, 1);
    let again = integer(&mut graph, 1);
    assert_ne!(one, again);
    assert_eq!(graph.canonical_id(again).unwrap(), one);
    let two = integer(&mut graph, 2);
    let a = graph.push(Node::Add(vec![one, two])).unwrap();
    let b = graph.push(Node::Add(vec![again, two])).unwrap();
    assert_eq!(graph.canonical_id(b).unwrap(), a);
    assert_eq!(graph.intern(Node::Add(vec![again, two])).unwrap(), a);
    let order = graph.intern(Node::Add(vec![two, again])).unwrap();
    assert_ne!(order, a);
    let three = integer(&mut graph, 3);
    assert_ne!(
        graph.canonical_id(three).unwrap(),
        graph.canonical_id(a).unwrap()
    );
    let duplicate_child = graph.push(Node::Sin(b)).unwrap();
    let shared_child = graph.intern(Node::Sin(a)).unwrap();
    assert_eq!(shared_child, duplicate_child);
    assert_eq!(
        graph.intern(Node::Add(vec![one])),
        Err(ExpressionError::InvalidArity)
    );
    assert_eq!(
        graph.intern(Node::Sin(99999)),
        Err(ExpressionError::InvalidReference(99999))
    );
}
