//! Тот же закрытый grammar, что exact_quadratic_value, на значениях Radical V2.
//! Вызывающий код отвечает за source expand перед этим читателем. Никаких
//! тригонометрических тождеств или расширения допуска вложенных корней здесь нет.
use crate::density_expression::{Expressions, Node, NodeId};
use crate::{IBig, UBig};
use cftuv_radical::{Failure, Priced, Radical, Work};
use std::collections::HashMap;
/// Внутренний конечный exponent profile читателя, НЕ product refusal.
pub const MAX_EXPONENT_MAGNITUDE: u64 = 1 << 20;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum QuadraticFailureKind {
    UnsupportedSyntax(NodeId),
    InvalidReference(NodeId),
    ExponentProfile(NodeId),
    /// В частности InverseClassLimit не становится продуктовым UNDECIDABLE.
    InternalAlgebra(Failure),
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QuadraticFailure {
    pub kind: QuadraticFailureKind,
    pub work: Work,
}
impl QuadraticFailure {
    fn new(kind: QuadraticFailureKind) -> Self {
        Self {
            kind,
            work: Work::NONE,
        }
    }
    fn after(mut self, work: Work) -> Self {
        self.work = work + self.work;
        self
    }
    fn algebra(error: Failure, work: Work) -> Self {
        let own = match &error {
            Failure::Refused(refusal) => refusal.work,
            _ => Work::NONE,
        };
        Self {
            kind: QuadraticFailureKind::InternalAlgebra(error),
            work: work + own,
        }
    }
}
pub struct QuadraticReader<'a> {
    expressions: &'a Expressions,
    memo: HashMap<NodeId, Radical>,
}
impl Expressions {
    /// Значения допустимого grammar, отдельно от interval и sign admission memo.
    pub fn quadratic(&mut self, id: NodeId) -> Result<Priced<Radical>, QuadraticFailure> {
        let memo = std::mem::take(&mut self.quadratic_memo);
        let mut reader = QuadraticReader {
            expressions: self,
            memo,
        };
        let result = reader.read(id);
        self.quadratic_memo = reader.memo;
        result
    }
}
impl<'a> QuadraticReader<'a> {
    pub fn new(expressions: &'a Expressions) -> Self {
        Self {
            expressions,
            memo: HashMap::new(),
        }
    }
    pub fn read(&mut self, id: NodeId) -> Result<Priced<Radical>, QuadraticFailure> {
        if let Some(value) = self.memo.get(&id) {
            return Ok((value.clone(), Work::NONE));
        }
        let node = self
            .expressions
            .node(id)
            .map_err(|_| QuadraticFailure::new(QuadraticFailureKind::InvalidReference(id)))?;
        let mut work = Work::NONE;
        let value = match node {
            Node::Rational(value) => value.radical(),
            Node::Add(children) | Node::Mul(children) => {
                let multiply = matches!(node, Node::Mul(_));
                let mut result = if multiply {
                    Radical::one()
                } else {
                    Radical::zero()
                };
                for &child in children {
                    let (value, price) = self.read(child).map_err(|e| e.after(work))?;
                    work = work + price;
                    let (value, price) = if multiply {
                        result.mul(&value)
                    } else {
                        result.add(&value)
                    };
                    result = value;
                    work = work + price;
                }
                result
            }
            Node::Pow(base, exponent) => {
                let Ok(Node::Rational(exponent)) = self.expressions.node(*exponent) else {
                    return Err(QuadraticFailure::new(
                        QuadraticFailureKind::UnsupportedSyntax(id),
                    ));
                };
                let (n, d) = exponent.parts();
                let power = i64::try_from(n.clone()).map_err(|_| {
                    QuadraticFailure::new(QuadraticFailureKind::ExponentProfile(id))
                })?;
                if power.unsigned_abs() > MAX_EXPONENT_MAGNITUDE {
                    return Err(QuadraticFailure::new(
                        QuadraticFailureKind::ExponentProfile(id),
                    ));
                }
                let factor = if d == &UBig::ONE {
                    let (value, price) = self.read(*base)?;
                    work = work + price;
                    value
                } else if d == &UBig::from(2u8) {
                    let Ok(Node::Rational(base)) = self.expressions.node(*base) else {
                        return Err(QuadraticFailure::new(
                            QuadraticFailureKind::UnsupportedSyntax(id),
                        ));
                    };
                    let (n, d) = base.parts();
                    if n < &IBig::ZERO {
                        return Err(QuadraticFailure::new(
                            QuadraticFailureKind::UnsupportedSyntax(id),
                        ));
                    }
                    let (value, price) = Radical::sqrt_of_rational(n.clone(), d.clone())
                        .map_err(|e| QuadraticFailure::algebra(Failure::Refused(e), work))?;
                    work = work + price;
                    value
                } else {
                    return Err(QuadraticFailure::new(
                        QuadraticFailureKind::UnsupportedSyntax(id),
                    ));
                };
                // V2 Work здесь только диагностика; source Density бюджет это не тарифицирует.
                let mut remaining = power.unsigned_abs();
                let mut result = Radical::one();
                let mut factor = factor;
                // Source __pow__ и A1 V2 reader сначала инвертируют BASE.
                // Степень может увеличить число классов: inverse(result) имеет другой допуск.
                if power < 0 {
                    let (value, price) = factor
                        .inverse()
                        .map_err(|e| QuadraticFailure::algebra(e, work))?;
                    factor = value;
                    work = work + price;
                }
                while remaining != 0 {
                    if remaining & 1 != 0 {
                        let (value, price) = result.mul(&factor);
                        result = value;
                        work = work + price;
                    }
                    remaining >>= 1;
                    if remaining != 0 {
                        let (value, price) = factor.mul(&factor);
                        factor = value;
                        work = work + price;
                    }
                }
                result
            }
            _ => {
                return Err(QuadraticFailure::new(
                    QuadraticFailureKind::UnsupportedSyntax(id),
                ))
            }
        };
        self.memo.insert(id, value.clone());
        Ok((value, work))
    }
}
