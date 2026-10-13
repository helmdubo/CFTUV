use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_expression_order::compare;
use cftuv_density::{IBig, Rational, UBig};
use std::cmp::Ordering;
fn q(g: &mut Expressions, n: i32, d: u32) -> usize {
    g.push(Node::Rational(
        Rational::new(IBig::from(n), UBig::from(d)).unwrap(),
    ))
    .unwrap()
}
#[test]
fn class_order_is_not_numeric_order_or_node_insertion_order() {
    let mut g = Expressions::new();
    let hundred = q(&mut g, 100, 1);
    let two = q(&mut g, 2, 1);
    let negative = q(&mut g, -100, 1);
    let half = q(&mut g, 1, 2);
    let third = q(&mut g, 1, 3);
    let fifth = q(&mut g, 1, 5);
    assert_eq!(compare(&g, hundred, two).unwrap(), Ordering::Greater);
    assert_eq!(compare(&g, half, negative).unwrap(), Ordering::Less);
    assert_eq!(compare(&g, third, fifth).unwrap(), Ordering::Less);
    let cosine = g.push(Node::Cos(two)).unwrap();
    let atan = g.push(Node::Atan(hundred)).unwrap();
    assert_eq!(compare(&g, atan, cosine).unwrap(), Ordering::Less);
}
#[test]
fn equal_classes_compare_arity_then_recursive_source_arguments() {
    let mut g = Expressions::new();
    let one = q(&mut g, 1, 1);
    let two = q(&mut g, 2, 1);
    let a = g.push(Node::Add(vec![two, two])).unwrap();
    let b = g.push(Node::Add(vec![one, one, one])).unwrap();
    let c = g.push(Node::Add(vec![one, two])).unwrap();
    let duplicate = g.push(Node::Add(vec![one, two])).unwrap();
    assert_eq!(compare(&g, a, b).unwrap(), Ordering::Less);
    assert_eq!(compare(&g, a, c).unwrap(), Ordering::Greater);
    assert_eq!(compare(&g, c, duplicate).unwrap(), Ordering::Equal);
}
