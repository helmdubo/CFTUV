//! Source-формулы в ordered AST на cache miss. Симметричная память dot/cross
//! принадлежит geometry adapter; здесь нет переупорядочивания матричной формулы.
use crate::density_construct::{ConstructError, ConstructResult, Constructor};
use crate::density_expression::{Node, NodeId};
use crate::{IBig, Rational};

pub type VectorNodes = [NodeId; 2];
pub type MatrixNodes = [[NodeId; 2]; 2];

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VectorNodeFailure {
    Construction(ConstructError),
    /// Нарушение admitted integer candidate / positive Gram; не product refusal.
    NonPositiveRationalCandidateNorm,
}
impl From<ConstructError> for VectorNodeFailure {
    fn from(error: ConstructError) -> Self {
        Self::Construction(error)
    }
}

pub fn matrix_nodes(
    c: &mut Constructor<'_>,
    values: &[[Rational; 2]; 2],
) -> ConstructResult<MatrixNodes> {
    Ok([
        [
            c.push(Node::Rational(values[0][0].clone()))?,
            c.push(Node::Rational(values[0][1].clone()))?,
        ],
        [
            c.push(Node::Rational(values[1][0].clone()))?,
            c.push(Node::Rational(values[1][1].clone()))?,
        ],
    ])
}

pub fn negate(c: &mut Constructor<'_>, value: NodeId) -> ConstructResult<NodeId> {
    let minus = c.push(Node::Rational(Rational::integer(-IBig::ONE)))?;
    c.mul(&[minus, value])
}

pub fn subtract(c: &mut Constructor<'_>, left: NodeId, right: NodeId) -> ConstructResult<NodeId> {
    let right = negate(c, right)?;
    c.add(&[left, right])
}

pub fn difference(
    c: &mut Constructor<'_>,
    left: VectorNodes,
    right: VectorNodes,
) -> ConstructResult<VectorNodes> {
    Ok([
        subtract(c, left[0], right[0])?,
        subtract(c, left[1], right[1])?,
    ])
}

pub fn matrix_vector(
    c: &mut Constructor<'_>,
    matrix: MatrixNodes,
    vector: VectorNodes,
) -> ConstructResult<VectorNodes> {
    let mut row = |index: usize| {
        let x = c.mul(&[matrix[index][0], vector[0]])?;
        let y = c.mul(&[matrix[index][1], vector[1]])?;
        c.add(&[x, y])
    };
    Ok([row(0)?, row(1)?])
}

/// Одинаковая формула для Gram и dual-Gram; scalar calls идут как Python expression.
pub fn dot(
    c: &mut Constructor<'_>,
    matrix: MatrixNodes,
    left: VectorNodes,
    right: VectorNodes,
) -> ConstructResult<NodeId> {
    // Не вызываем matrix_vector целиком: source умножает lx до построения второй строки.
    let first_x = c.mul(&[matrix[0][0], right[0]])?;
    let first_y = c.mul(&[matrix[0][1], right[1]])?;
    let first_sum = c.add(&[first_x, first_y])?;
    let first = c.mul(&[left[0], first_sum])?;
    let second_x = c.mul(&[matrix[1][0], right[0]])?;
    let second_y = c.mul(&[matrix[1][1], right[1]])?;
    let second_sum = c.add(&[second_x, second_y])?;
    let second = c.mul(&[left[1], second_sum])?;
    c.add(&[first, second])
}

pub fn oriented_cross(
    c: &mut Constructor<'_>,
    left: VectorNodes,
    right: VectorNodes,
    orientation: i8,
) -> ConstructResult<NodeId> {
    let first = c.mul(&[left[0], right[1]])?;
    let second = c.mul(&[left[1], right[0]])?;
    let difference = subtract(c, first, second)?;
    let orientation = c.push(Node::Rational(Rational::integer(IBig::from(orientation))))?;
    c.mul(&[orientation, difference])
}

/// Source `_objective`: integer covector и rational positive inverse-Gram.
pub fn objective(
    c: &mut Constructor<'_>,
    inverse: MatrixNodes,
    ideal: VectorNodes,
    candidate: &(IBig, IBig),
) -> Result<NodeId, VectorNodeFailure> {
    let candidate = [
        c.push(Node::Rational(Rational::integer(candidate.0.clone())))?,
        c.push(Node::Rational(Rational::integer(candidate.1.clone())))?,
    ];
    let value = dot(c, inverse, candidate, ideal)?;
    let norm = dot(c, inverse, candidate, candidate)?;
    let Node::Rational(norm) = c.node(norm).map_err(ConstructError::from)? else {
        return Err(VectorNodeFailure::NonPositiveRationalCandidateNorm);
    };
    let (n, d) = norm.parts();
    if n <= &IBig::ZERO {
        return Err(VectorNodeFailure::NonPositiveRationalCandidateNorm);
    }
    let reciprocal = Rational::new(IBig::from(d.clone()), cftuv_radical::arith::magnitude(n))
        .expect("положительная candidate norm");
    let square = c.mul(&[value, value])?;
    let inverse = c.push(Node::Rational(reciprocal))?;
    Ok(c.mul(&[square, inverse])?)
}
