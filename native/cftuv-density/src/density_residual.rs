//! Source-фильтр подшага: прежние iv160 операции и запас 2^-100.
//! None означает уступить построению source residual, а не продуктовый отказ.
use crate::density_expression::{ExpressionError, ExpressionResult, Expressions, Node, NodeId};
use crate::legacy_interval::LegacyInterval;
use crate::legacy_mpf::{LegacyError, Mpf, Precision, Rounding};
use crate::IBig;

fn point(value: Mpf) -> ExpressionResult<LegacyInterval> {
    Ok(LegacyInterval::new(value.clone(), value)?)
}
fn integer(value: i32) -> ExpressionResult<LegacyInterval> {
    point(Mpf::from_man_exp(
        IBig::from(value),
        0,
        None,
        Rounding::TowardZero,
    )?)
}

/// Вызов сохраняет порядок dot -> left -> right и успехи interval memo при отказе.
pub fn filtered_sign(
    graph: &mut Expressions,
    dot: NodeId,
    norm_left: NodeId,
    norm_right: NodeId,
    q: usize,
) -> ExpressionResult<Option<i8>> {
    if !matches!(q, 3..=6) {
        return Ok(None);
    }
    if !matches!(graph.node(dot)?, Node::Add(_)) {
        return Ok(None);
    }
    match calculate(graph, dot, norm_left, norm_right, q) {
        Err(
            ExpressionError::UnsupportedExpression(_)
            | ExpressionError::Legacy(LegacyError::ZeroDivision | LegacyError::ComplexResult),
        ) => Ok(None),
        result => result,
    }
}

fn calculate(
    graph: &mut Expressions,
    dot: NodeId,
    norm_left: NodeId,
    norm_right: NodeId,
    q: usize,
) -> ExpressionResult<Option<i8>> {
    let p = Precision::new(160).expect("фиксированная source precision");
    let dot_box = graph.interval(dot, p)?;
    let left = graph.interval(norm_left, p)?;
    let right = graph.interval(norm_right, p)?;
    let norms = left.mul(&right, p)?;
    let (first, second) = if q == 5 {
        (
            integer(8)?.mul(&dot_box.pow_int(2, p)?, p)?,
            integer(3)?.add(&integer(5)?.sqrt(p)?, p)?.mul(&norms, p)?,
        )
    } else {
        (
            integer(if q == 4 { 2 } else { 4 })?.mul(&dot_box.pow_int(2, p)?, p)?,
            integer(if q == 6 { 3 } else { 1 })?.mul(&norms, p)?,
        )
    };
    let residual = first.sub(&second, p)?;
    let scale =
        point(first.abs(p)?.upper().clone())?.add(&point(second.abs(p)?.upper().clone())?, p)?;
    let margin = point(Mpf::from_man_exp(
        IBig::ONE,
        -100,
        None,
        Rounding::TowardZero,
    )?)?;
    let limit = scale.mul(&margin, p)?;
    if residual.lower().compare(limit.upper()).is_gt() {
        return Ok(Some(1));
    }
    if residual.upper().neg_exact().compare(limit.upper()).is_gt() {
        return Ok(Some(-1));
    }
    Ok(None)
}
