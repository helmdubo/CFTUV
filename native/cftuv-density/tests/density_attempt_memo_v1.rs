use cftuv_density::density_attempt_memo_v1::DensityLiteralTreeV1;
use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::{IBig, Rational, UBig};

fn rational(n: i8) -> Node {
    Node::Rational(Rational::new(IBig::from(n), UBig::ONE).unwrap())
}

#[test]
fn literal_identity_survives_arena_numbering_and_preserves_order_and_syntax() {
    let mut first = Expressions::new();
    let a = first.push(rational(1)).unwrap();
    let b = first.push(Node::Pi).unwrap();
    let add = first.push(Node::Add(vec![a, b])).unwrap();
    let swapped = first.push(Node::Add(vec![b, a])).unwrap();
    let mul = first.push(Node::Mul(vec![a, b])).unwrap();
    let mut second = Expressions::new();
    second.push(rational(42)).unwrap();
    let b2 = second.push(Node::Pi).unwrap();
    let a2 = second.push(rational(1)).unwrap();
    let add2 = second.push(Node::Add(vec![a2, b2])).unwrap();
    let tree = DensityLiteralTreeV1::from_expression(&first, add).unwrap();
    assert_eq!(
        tree,
        DensityLiteralTreeV1::from_expression(&second, add2).unwrap()
    );
    assert_ne!(
        tree,
        DensityLiteralTreeV1::from_expression(&first, swapped).unwrap()
    );
    assert_ne!(
        tree,
        DensityLiteralTreeV1::from_expression(&first, mul).unwrap()
    );
    // Значение 1+1 равно 2, но literal identity их не объединяет.
    let two = first.push(rational(2)).unwrap();
    let sum = first.push(Node::Add(vec![a, a])).unwrap();
    assert_ne!(
        DensityLiteralTreeV1::from_expression(&first, sum).unwrap(),
        DensityLiteralTreeV1::from_expression(&first, two).unwrap()
    );
}

#[test]
fn closed_literal_export_covers_every_raw_tag_and_rejects_invalid_reference() {
    let mut graph = Expressions::new();
    let a = graph.push(rational(1)).unwrap();
    let b = graph.push(Node::Pi).unwrap();
    for node in [
        Node::Sin(a),
        Node::Cos(a),
        Node::Atan(a),
        Node::Atan2(a, b),
        Node::Pow(a, b),
        Node::Tan(a),
    ] {
        let id = graph.push(node).unwrap();
        assert!(DensityLiteralTreeV1::from_expression(&graph, id).is_ok());
    }
    assert!(DensityLiteralTreeV1::from_expression(&graph, usize::MAX).is_err());
}
