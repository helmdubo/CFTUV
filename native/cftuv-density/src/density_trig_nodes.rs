//! Закрытая таблица `_exact_q_trig`: точные source-деревья для q=2..6.
//! sin(pi/5) остаётся вложенным корнем; Radical-reader не расширяет его допуск.
use crate::density_expression::{ExpressionError, Expressions, Node, NodeId};
use crate::{IBig, Rational, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TrigNodeFailure {
    UnsupportedDensityQ(usize),
    Expression(ExpressionError),
}
impl From<ExpressionError> for TrigNodeFailure {
    fn from(value: ExpressionError) -> Self {
        Self::Expression(value)
    }
}
fn rational(graph: &mut Expressions, n: i32, d: u32) -> Result<NodeId, TrigNodeFailure> {
    Ok(graph.push(Node::Rational(
        Rational::new(IBig::from(n), UBig::from(d)).expect("ненулевой denominator таблицы"),
    ))?)
}
pub fn exact_q_trig(graph: &mut Expressions, q: usize) -> Result<[NodeId; 2], TrigNodeFailure> {
    if !(2..=6).contains(&q) {
        return Err(TrigNodeFailure::UnsupportedDensityQ(q));
    }
    if q == 2 {
        return Ok([rational(graph, 0, 1)?, rational(graph, 1, 1)?]);
    }
    let half = rational(graph, 1, 2)?;
    if q == 5 {
        let quarter = rational(graph, 1, 4)?;
        let five = rational(graph, 5, 1)?;
        let root_five = graph.push(Node::Pow(five, half))?;
        let quarter_root = graph.push(Node::Mul(vec![quarter, root_five]))?;
        // Source распределяет (1 + sqrt(5))/4 при создании Mul.
        let cosine = graph.push(Node::Add(vec![quarter, quarter_root]))?;
        let ten = rational(graph, 10, 1)?;
        let minus_two = rational(graph, -2, 1)?;
        let subtract = graph.push(Node::Mul(vec![minus_two, root_five]))?;
        let radicand = graph.push(Node::Add(vec![ten, subtract]))?;
        let root = graph.push(Node::Pow(radicand, half))?;
        return Ok([cosine, graph.push(Node::Mul(vec![quarter, root]))?]);
    }
    let radicand = rational(graph, if q == 4 { 2 } else { 3 }, 1)?;
    let root = graph.push(Node::Pow(radicand, half))?;
    let value = graph.push(Node::Mul(vec![half, root]))?;
    Ok(match q {
        3 => [half, value],
        4 => [value, value],
        6 => [value, half],
        _ => unreachable!(),
    })
}
