//! Контроллер source sign: исходный ordered interval, затем отдельно expanded
//! quadratic fallback. Это не нормализатор: equivalence двух узлов — контракт
//! native construction layer. Нельзя подставлять произвольное значение Radical.
use crate::density_expression::{Evaluation, ExpressionError, Expressions, Node, NodeId};
use crate::density_quadratic::{QuadraticFailure, QuadraticFailureKind, QuadraticReader};
use crate::legacy_interval::LegacyInterval;
use crate::legacy_mpf::{LegacyError, Precision};
use crate::{IBig, Priced, Radical, Work};
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SignFailure {
    Undecidable,
    IntervalInternal(ExpressionError),
    QuadraticInternal(QuadraticFailure),
    AlgebraInternal(cftuv_radical::Failure),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SignPath {
    Rational,
    Interval,
    Quadratic,
    Memo,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignResult {
    pub sign: i8,
    pub path: SignPath,
    pub work: Work,
}

/// Резервный AST нужен только после неразрешившегося исходного интервала.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SourceSignProbe {
    Decided(SignResult),
    NeedsExpanded,
}

pub struct SignEvaluation<'a> {
    expressions: &'a Expressions,
    intervals: Evaluation<'a>,
    quadratic: QuadraticReader<'a>,
    // Контракт пары виден в ключе; разные fallback certificates не смешиваются.
    memo: HashMap<(NodeId, NodeId), i8>,
}
impl<'a> SignEvaluation<'a> {
    pub fn new(expressions: &'a Expressions) -> Self {
        Self {
            expressions,
            intervals: expressions.evaluation(Precision::new(160).unwrap()),
            quadratic: QuadraticReader::new(expressions),
            memo: HashMap::new(),
        }
    }
    pub fn sign_with_expanded_fallback(
        &mut self,
        source: NodeId,
        expanded: NodeId,
    ) -> Result<SignResult, SignFailure> {
        decide(self, source, expanded)
    }

    pub fn probe_sign(&mut self, source: NodeId) -> Result<SourceSignProbe, SignFailure> {
        probe(self, source)
    }
}

trait SignContext {
    fn node(&self, id: NodeId) -> Result<&Node, ExpressionError>;
    fn remembered(&self, source: NodeId, expanded: NodeId) -> Option<i8>;
    fn remember(&mut self, source: NodeId, expanded: NodeId, sign: i8);
    fn interval(&mut self, id: NodeId) -> Result<LegacyInterval, ExpressionError>;
    fn quadratic(&mut self, id: NodeId) -> Result<Priced<Radical>, QuadraticFailure>;
}
impl SignContext for SignEvaluation<'_> {
    fn node(&self, id: NodeId) -> Result<&Node, ExpressionError> {
        self.expressions.node(id)
    }
    fn remembered(&self, source: NodeId, expanded: NodeId) -> Option<i8> {
        self.memo.get(&(source, expanded)).copied()
    }
    fn remember(&mut self, source: NodeId, expanded: NodeId, sign: i8) {
        self.memo.insert((source, expanded), sign);
    }
    fn interval(&mut self, id: NodeId) -> Result<LegacyInterval, ExpressionError> {
        self.intervals.interval(id)
    }
    fn quadratic(&mut self, id: NodeId) -> Result<Priced<Radical>, QuadraticFailure> {
        self.quadratic.read(id)
    }
}
impl SignContext for Expressions {
    fn node(&self, id: NodeId) -> Result<&Node, ExpressionError> {
        Expressions::node(self, id)
    }
    fn remembered(&self, source: NodeId, expanded: NodeId) -> Option<i8> {
        self.sign_memo.get(&(source, expanded)).copied()
    }
    fn remember(&mut self, source: NodeId, expanded: NodeId, sign: i8) {
        self.sign_memo.insert((source, expanded), sign);
    }
    fn interval(&mut self, id: NodeId) -> Result<LegacyInterval, ExpressionError> {
        Expressions::interval(self, id, Precision::new(160).unwrap())
    }
    fn quadratic(&mut self, id: NodeId) -> Result<Priced<Radical>, QuadraticFailure> {
        Expressions::quadratic(self, id)
    }
}
impl Expressions {
    /// Для растущего графа: все три memo принадлежат этой compile-транзакции.
    pub fn sign_with_expanded_fallback(
        &mut self,
        source: NodeId,
        expanded: NodeId,
    ) -> Result<SignResult, SignFailure> {
        decide(self, source, expanded)
    }

    pub fn probe_sign(&mut self, source: NodeId) -> Result<SourceSignProbe, SignFailure> {
        probe(self, source)
    }
}

fn decide(
    context: &mut impl SignContext,
    source: NodeId,
    expanded: NodeId,
) -> Result<SignResult, SignFailure> {
    if let Some(sign) = context.remembered(source, expanded) {
        return Ok(SignResult {
            sign,
            path: SignPath::Memo,
            work: Work::NONE,
        });
    }
    if let SourceSignProbe::Decided(result) = probe(context, source)? {
        if result.path != SignPath::Rational {
            context.remember(source, expanded, result.sign);
        }
        return Ok(result);
    }
    let (value, work) = context
        .quadratic(expanded)
        .map_err(|error| match error.kind {
            QuadraticFailureKind::UnsupportedSyntax(_) => SignFailure::Undecidable,
            _ => SignFailure::QuadraticInternal(error),
        })?;
    let (sign, sign_work) = value.signum(None).map_err(SignFailure::AlgebraInternal)?;
    context.remember(source, expanded, sign);
    Ok(SignResult {
        sign,
        path: SignPath::Quadratic,
        work: work + sign_work,
    })
}

fn probe(context: &mut impl SignContext, source: NodeId) -> Result<SourceSignProbe, SignFailure> {
    if let Node::Rational(value) = context
        .node(source)
        .map_err(SignFailure::IntervalInternal)?
    {
        let (n, _) = value.parts();
        return Ok(SourceSignProbe::Decided(SignResult {
            sign: if n > &IBig::ZERO {
                1
            } else if n < &IBig::ZERO {
                -1
            } else {
                0
            },
            path: SignPath::Rational,
            work: Work::NONE,
        }));
    }
    match context.interval(source) {
        Ok(interval) => {
            let sign = if interval.lower().sign() > 0 {
                Some(1)
            } else if interval.upper().sign() < 0 {
                Some(-1)
            } else {
                None
            };
            if let Some(sign) = sign {
                return Ok(SourceSignProbe::Decided(SignResult {
                    sign,
                    path: SignPath::Interval,
                    work: Work::NONE,
                }));
            }
        }
        // Только source-caught исключения допускают резервный exact путь.
        Err(
            ExpressionError::UnsupportedExpression(_)
            | ExpressionError::Legacy(LegacyError::ZeroDivision | LegacyError::ComplexResult),
        ) => {}
        Err(error) => return Err(SignFailure::IntervalInternal(error)),
    }
    Ok(SourceSignProbe::NeedsExpanded)
}
