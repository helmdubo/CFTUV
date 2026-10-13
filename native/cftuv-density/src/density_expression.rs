//! Закрытое ordered-выражение Density: только whitelist `_density_policy`.
//! Это interval/admission identity, не канонический ключ Radical по значению.
//! Память принадлежит одному Evaluation, внешняя глобальная память отсутствует.
use crate::legacy_atan::{mpi_atan2, AtanError};
use crate::legacy_interval::LegacyInterval;
use crate::legacy_mpf::{LegacyError, Mpf, Precision, Rounding};
use crate::legacy_trig::{mpf_pi, TrigError};
use crate::legacy_trig_eval::{mpi_cos, mpi_sin, EvalError};
use crate::{IBig, Radical, Rational, UBig};
use std::collections::HashMap;

pub type NodeId = usize;
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Node {
    Rational(Rational),
    Pi,
    Add(Vec<NodeId>),
    Mul(Vec<NodeId>),
    Pow(NodeId, NodeId),
    Sin(NodeId),
    Cos(NodeId),
    Atan(NodeId),
    Atan2(NodeId, NodeId),
    /// Закрытый отрицательный пример; исходный whitelist не поддерживает tan.
    Tan(NodeId),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExpressionError {
    InvalidReference(NodeId),
    InvalidArity,
    DepthLimit,
    UnsupportedExpression(NodeId),
    IntegerExponentRange(NodeId),
    Legacy(LegacyError),
    Trig(TrigError),
    TrigEval(EvalError),
    Atan(AtanError),
}
impl From<LegacyError> for ExpressionError {
    fn from(e: LegacyError) -> Self {
        Self::Legacy(e)
    }
}
impl From<TrigError> for ExpressionError {
    fn from(e: TrigError) -> Self {
        Self::Trig(e)
    }
}
impl From<EvalError> for ExpressionError {
    fn from(e: EvalError) -> Self {
        Self::TrigEval(e)
    }
}
impl From<AtanError> for ExpressionError {
    fn from(e: AtanError) -> Self {
        Self::Atan(e)
    }
}
pub type ExpressionResult<T> = Result<T, ExpressionError>;

#[derive(Default)]
pub struct Expressions {
    nodes: Vec<Node>,
    depths: Vec<usize>,
    canonical_ids: Vec<NodeId>,
    structural_nodes: HashMap<Node, NodeId>,
    // Одна точность на транзакцию. Смена точности сбрасывает только interval memo.
    interval_memo: Option<(Precision, Vec<Option<LegacyInterval>>)>,
    pub(crate) quadratic_memo: HashMap<NodeId, Radical>,
    pub(crate) sign_memo: HashMap<(NodeId, NodeId), i8>,
}
impl Expressions {
    pub fn new() -> Self {
        Self::default()
    }
    /// Только обратные ссылки; дерево не сортируется и не упрощается.
    pub fn push(&mut self, node: Node) -> ExpressionResult<NodeId> {
        let children: &[NodeId] = match &node {
            Node::Rational(_) | Node::Pi => &[],
            Node::Add(children) | Node::Mul(children) => {
                if children.len() < 2 {
                    return Err(ExpressionError::InvalidArity);
                }
                children
            }
            Node::Sin(id) | Node::Cos(id) | Node::Atan(id) | Node::Tan(id) => {
                std::slice::from_ref(id)
            }
            Node::Pow(_, _) | Node::Atan2(_, _) => &[],
        };
        let mut depth = 1;
        for &child in children {
            depth = depth.max(
                1 + *self
                    .depths
                    .get(child)
                    .ok_or(ExpressionError::InvalidReference(child))?,
            );
        }
        if let Node::Pow(a, b) | Node::Atan2(a, b) = &node {
            for &child in [a, b] {
                depth = depth.max(
                    1 + *self
                        .depths
                        .get(child)
                        .ok_or(ExpressionError::InvalidReference(child))?,
                );
            }
        }
        if depth > 512 {
            return Err(ExpressionError::DepthLimit);
        }
        let id = self.nodes.len();
        let key = self.canonical_form(&node)?;
        let canonical = *self.structural_nodes.entry(key).or_insert(id);
        self.nodes.push(node);
        self.depths.push(depth);
        self.canonical_ids.push(canonical);
        Ok(id)
    }
    /// Структурный Expr-key: порядок args остаётся частью identity, значения Radical нет.
    pub fn canonical_id(&self, id: NodeId) -> ExpressionResult<NodeId> {
        self.canonical_ids
            .get(id)
            .copied()
            .ok_or(ExpressionError::InvalidReference(id))
    }
    fn canonical_form(&self, node: &Node) -> ExpressionResult<Node> {
        let id = |value: NodeId| self.canonical_id(value);
        Ok(match node {
            Node::Rational(_) | Node::Pi => node.clone(),
            Node::Add(args) | Node::Mul(args) => {
                if args.len() < 2 {
                    return Err(ExpressionError::InvalidArity);
                }
                let args = args
                    .iter()
                    .map(|&value| id(value))
                    .collect::<ExpressionResult<Vec<_>>>()?;
                if matches!(node, Node::Add(_)) {
                    Node::Add(args)
                } else {
                    Node::Mul(args)
                }
            }
            Node::Pow(a, b) => Node::Pow(id(*a)?, id(*b)?),
            Node::Atan2(a, b) => Node::Atan2(id(*a)?, id(*b)?),
            Node::Sin(a) => Node::Sin(id(*a)?),
            Node::Cos(a) => Node::Cos(id(*a)?),
            Node::Atan(a) => Node::Atan(id(*a)?),
            Node::Tan(a) => Node::Tan(id(*a)?),
        })
    }
    /// Native construction переиспользует первый структурно равный узел этой arena.
    /// Raw replay сохраняет append numbering через push; эти два API не смешивают value admission.
    pub fn intern(&mut self, node: Node) -> ExpressionResult<NodeId> {
        let key = self.canonical_form(&node)?;
        match self.structural_nodes.get(&key) {
            Some(&id) => Ok(id),
            None => self.push(node),
        }
    }
    pub fn node(&self, id: NodeId) -> ExpressionResult<&Node> {
        self.nodes
            .get(id)
            .ok_or(ExpressionError::InvalidReference(id))
    }
    pub fn evaluation(&self, precision: Precision) -> Evaluation<'_> {
        Evaluation {
            expressions: self,
            precision,
            memo: vec![None; self.nodes.len()],
        }
    }

    /// Накопительный расчёт при построении DAG: append не обесценивает старые узлы.
    /// Успешные дочерние значения сохраняются и после отказа родителя, как в source.
    /// Мемо неотделимо от arena, поэтому совпадение NodeId другой транзакции безопасно.
    pub fn interval(
        &mut self,
        id: NodeId,
        precision: Precision,
    ) -> ExpressionResult<LegacyInterval> {
        let mut memo = match self.interval_memo.take() {
            Some((old_precision, memo)) if old_precision == precision => memo,
            _ => Vec::new(),
        };
        memo.resize(self.nodes.len(), None);
        let mut evaluation = Evaluation {
            expressions: self,
            precision,
            memo,
        };
        let result = evaluation.interval(id);
        self.interval_memo = Some((precision, evaluation.memo));
        result
    }

    pub fn interval_cached(&self, id: NodeId, precision: Precision) -> bool {
        self.interval_memo
            .as_ref()
            .is_some_and(|(old_precision, memo)| {
                *old_precision == precision && memo.get(id).is_some_and(Option::is_some)
            })
    }
}

pub struct Evaluation<'a> {
    expressions: &'a Expressions,
    precision: Precision,
    memo: Vec<Option<LegacyInterval>>,
}
impl Evaluation<'_> {
    pub fn cached(&self, id: NodeId) -> bool {
        self.memo.get(id).is_some_and(Option::is_some)
    }
    fn integer(&self, n: IBig) -> ExpressionResult<LegacyInterval> {
        Ok(LegacyInterval::new(
            Mpf::from_man_exp(n.clone(), 0, Some(self.precision), Rounding::Floor)?,
            Mpf::from_man_exp(n, 0, Some(self.precision), Rounding::Ceiling)?,
        )?)
    }
    pub fn interval(&mut self, id: NodeId) -> ExpressionResult<LegacyInterval> {
        if let Some(Some(value)) = self.memo.get(id) {
            return Ok(value.clone());
        }
        let p = self.precision;
        let value = match self.expressions.node(id)? {
            Node::Rational(value) => {
                let (n, d) = value.parts();
                let numerator = self.integer(n.clone())?;
                if d == &UBig::ONE {
                    numerator
                } else {
                    numerator.div(&self.integer(IBig::from(d.clone()))?, p)?
                }
            }
            Node::Pi => {
                LegacyInterval::new(mpf_pi(p, Rounding::Floor)?, mpf_pi(p, Rounding::Ceiling)?)?
            }
            Node::Add(children) => {
                let mut value = LegacyInterval::zero();
                for &child in children {
                    value = value.add(&self.interval(child)?, p)?;
                }
                value
            }
            Node::Mul(children) => {
                let mut value = LegacyInterval::one();
                for &child in children {
                    value = value.mul(&self.interval(child)?, p)?;
                }
                value
            }
            Node::Pow(base, exponent) => {
                // Source вычисляет base до проверки допустимости exponent.
                let base = self.interval(*base)?;
                let Node::Rational(value) = self.expressions.node(*exponent)? else {
                    return Err(ExpressionError::UnsupportedExpression(id));
                };
                let (n, d) = value.parts();
                if d != &UBig::ONE && d != &UBig::from(2u8) {
                    return Err(ExpressionError::UnsupportedExpression(id));
                }
                let power = i64::try_from(n.clone())
                    .map_err(|_| ExpressionError::IntegerExponentRange(id))?;
                if d == &UBig::ONE {
                    base.pow_int(power, p)?
                } else {
                    let root = base.sqrt(p)?;
                    if power == 1 {
                        root
                    } else {
                        root.pow_int(power, p)?
                    }
                }
            }
            Node::Sin(child) => mpi_sin(&self.interval(*child)?, p)?,
            Node::Cos(child) => mpi_cos(&self.interval(*child)?, p)?,
            Node::Atan(child) => mpi_atan2(&self.interval(*child)?, &LegacyInterval::one(), p)?,
            Node::Atan2(y, x) => {
                let y = self.interval(*y)?;
                let x = self.interval(*x)?;
                mpi_atan2(&y, &x, p)?
            }
            Node::Tan(_) => return Err(ExpressionError::UnsupportedExpression(id)),
        };
        self.memo[id] = Some(value.clone());
        Ok(value)
    }
}
