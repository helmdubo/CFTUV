//! Закрытый порядок Basic.compare SymPy 1.14 для Density node whitelist.
//! Числа сравниваются по class/hashable content, не по числовому значению.
//! Этот порядок нужен исходным Add/Mul; он не заменяет value equality Radical.
use crate::density_expression::{ExpressionError, Expressions, Node, NodeId};
use crate::{IBig, UBig};
use std::cmp::Ordering;

fn class(node: &Node) -> (u8, &'static str) {
    match node {
        Node::Rational(value) => {
            let (n, d) = value.parts();
            if n == &IBig::ZERO {
                (0, "")
            } else if n == &IBig::ONE && d == &UBig::ONE {
                (1, "")
            } else if n == &IBig::ONE && d == &UBig::from(2u8) {
                (2, "")
            } else if n == &-IBig::ONE && d == &UBig::ONE {
                (5, "")
            } else if d == &UBig::ONE {
                (7, "")
            } else {
                (8, "")
            }
        }
        Node::Pi => (11, ""),
        Node::Pow(_, _) => (15, ""),
        Node::Mul(_) => (16, ""),
        Node::Add(_) => (17, ""),
        // Function class names нижнего регистра не входят в historical ordered list.
        Node::Atan(_) => (255, "atan"),
        Node::Atan2(_, _) => (255, "atan2"),
        Node::Cos(_) => (255, "cos"),
        Node::Sin(_) => (255, "sin"),
        Node::Tan(_) => (255, "tan"),
    }
}
fn arity(node: &Node) -> usize {
    match node {
        Node::Add(args) | Node::Mul(args) => args.len(),
        Node::Pow(_, _) | Node::Atan2(_, _) => 2,
        Node::Sin(_) | Node::Cos(_) | Node::Tan(_) | Node::Atan(_) => 1,
        _ => 0,
    }
}
fn child(node: &Node, index: usize) -> NodeId {
    match node {
        Node::Add(args) | Node::Mul(args) => args[index],
        Node::Pow(a, b) | Node::Atan2(a, b) => {
            if index == 0 {
                *a
            } else {
                *b
            }
        }
        Node::Sin(a) | Node::Cos(a) | Node::Tan(a) | Node::Atan(a) => *a,
        _ => unreachable!("arity zero excludes child"),
    }
}
pub fn compare(
    graph: &Expressions,
    left: NodeId,
    right: NodeId,
) -> Result<Ordering, ExpressionError> {
    let a = graph.node(left)?;
    let b = graph.node(right)?;
    if left == right {
        return Ok(Ordering::Equal);
    }
    let order = class(a).cmp(&class(b));
    if !order.is_eq() {
        return Ok(order);
    }
    if let (Node::Rational(a), Node::Rational(b)) = (a, b) {
        let (an, ad) = a.parts();
        let (bn, bd) = b.parts();
        return Ok(an.cmp(bn).then_with(|| ad.cmp(bd)));
    }
    let count = arity(a);
    let order = count.cmp(&arity(b));
    if !order.is_eq() {
        return Ok(order);
    }
    for index in 0..count {
        let order = compare(graph, child(a, index), child(b, index))?;
        if !order.is_eq() {
            return Ok(order);
        }
    }
    Ok(Ordering::Equal)
}
