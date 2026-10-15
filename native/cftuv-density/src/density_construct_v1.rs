//! Owned candidate DENSITY_SOURCE_CONSTRUCTION_V1: ScalarV2 остаётся целым атомом.
//! Один Expressions хранит decision tree; локальные tags задают границу Numeric/Formal.
//! Legacy Constructor, RootRegistry, sign/admission и expand здесь не используются.
use crate::density_expression::{ExpressionError, Expressions, Node, NodeId};
use crate::density_expression_order::compare;
use crate::interval::rational_mul;
use crate::scalar_v2::{LiteralNode, ScalarError, ScalarV2, CANON, MAX_EXPONENT_MAGNITUDE};
use crate::{IBig, Rational, UBig};
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProfileCode {
    NegativeRationalRoot,
    FreshNegativeOpaquePower,
    HalfIntegerExponentRequired,
    RationalNormRequired,
    PartialDomainDiscard,
}
impl ProfileCode {
    pub fn code(&self) -> &'static str {
        match self {
            Self::NegativeRationalRoot => "DENSITY_CONSTRUCT_V1_NEGATIVE_RATIONAL_ROOT",
            Self::FreshNegativeOpaquePower => "DENSITY_CONSTRUCT_V1_NEGATIVE_OPAQUE_POWER",
            Self::HalfIntegerExponentRequired => "DENSITY_CONSTRUCT_V1_POWER_PROFILE",
            Self::RationalNormRequired => "DENSITY_CONSTRUCT_V1_RATIONAL_NORM_REQUIRED",
            Self::PartialDomainDiscard => "DENSITY_CONSTRUCT_V1_PARTIAL_DOMAIN_DISCARD",
        }
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConstructErrorV1 {
    ForeignArena,
    Arena(ExpressionError),
    Scalar(ScalarError),
    Profile(ProfileCode),
    ExponentProfile,
    NonPositiveNorm,
    InvalidOrientation,
}
impl From<ExpressionError> for ConstructErrorV1 {
    fn from(value: ExpressionError) -> Self {
        Self::Arena(value)
    }
}
impl From<ScalarError> for ConstructErrorV1 {
    fn from(value: ScalarError) -> Self {
        Self::Scalar(value)
    }
}
pub type ConstructResultV1<T> = Result<T, ConstructErrorV1>;

/// Живой handle удерживает token после drop constructor: адрес нельзя переиспользовать.
/// Raw NodeId не является входом arithmetic API, чужая arena проверяется до identities.
#[derive(Clone, Debug)]
pub struct OwnedExprV1 {
    node: NodeId,
    arena: Arc<()>,
}
#[derive(Clone, Debug)]
enum Kind {
    Numeric(ScalarV2),
    Pi,
    Sin,
    Cos,
    Atan2,
    OpaquePow,
    IntegerPow { base: NodeId, exponent: i64 },
    Add(Vec<NodeId>),
    Mul(Vec<NodeId>),
}
/// Readonly typed view: Numeric boundary сохраняется, child handles не forgeable.
#[derive(Clone, Debug)]
pub enum OwnedFormV1 {
    Numeric(ScalarV2),
    Pi,
    Sin(OwnedExprV1),
    Cos(OwnedExprV1),
    Atan2(OwnedExprV1, OwnedExprV1),
    OpaquePow {
        base: OwnedExprV1,
        exponent: Rational,
    },
    IntegerPow {
        base: OwnedExprV1,
        exponent: i64,
    },
    Add(Vec<OwnedExprV1>),
    Mul(Vec<OwnedExprV1>),
}
/// Private exclusive graph borrow исключает reset и отделяет tags от другой транзакции.
/// Метаданные не содержат второй AST или root provenance; raw tree уже в Expressions.
pub struct ConstructorV1<'a> {
    graph: &'a mut Expressions,
    kinds: HashMap<NodeId, Kind>,
    arena: Arc<()>,
    partial: HashSet<NodeId>,
    source_sign_memo: HashMap<NodeId, i8>,
}
impl<'a> ConstructorV1<'a> {
    pub fn new(graph: &'a mut Expressions) -> Self {
        Self {
            graph,
            kinds: HashMap::new(),
            arena: Arc::new(()),
            partial: HashSet::new(),
            source_sign_memo: HashMap::new(),
        }
    }
    pub fn canon(&self) -> &'static str {
        CANON
    }
    pub fn graph(&self) -> &Expressions {
        self.graph
    }
    pub fn interval(
        &mut self,
        value: &OwnedExprV1,
        precision: crate::legacy_mpf::Precision,
    ) -> ConstructResultV1<crate::legacy_interval::LegacyInterval> {
        let id = self.raw_id(value)?;
        Ok(self.graph.interval(id, precision)?)
    }
    pub fn filtered_residual_sign(
        &mut self,
        dot: &OwnedExprV1,
        norm_left: &OwnedExprV1,
        norm_right: &OwnedExprV1,
        q: usize,
    ) -> ConstructResultV1<Option<i8>> {
        let dot = self.raw_id(dot)?;
        let left = self.raw_id(norm_left)?;
        let right = self.raw_id(norm_right)?;
        Ok(crate::density_residual::filtered_sign(
            self.graph, dot, left, right, q,
        )?)
    }
    /// Свежая opaque reciprocal создаётся только после прежнего nonzero proof.
    /// Уже существующие partial descendants не стираются; Numeric inverse profile тот же.
    pub fn divide_with_nonzero_witness(
        &mut self,
        numerator: &OwnedExprV1,
        proof: &crate::density_laws_v1::NonzeroWitnessV1,
    ) -> ConstructResultV1<OwnedExprV1> {
        let numerator_id = self.raw_id(numerator)?;
        let denominator = proof.denominator();
        let denominator_id = self.raw_id(denominator)?;
        if self.number(denominator_id).is_some() {
            return self.div(numerator, denominator);
        }
        let inherited_partial = self.partial.contains(&denominator_id);
        if !inherited_partial
            && self
                .number(numerator_id)
                .is_some_and(|v| v.value().terms().is_empty())
        {
            return self.integer_power(numerator, 1);
        }
        let reciprocal = self.formal_reciprocal(denominator)?;
        if !inherited_partial {
            let id = self.raw_id(&reciprocal)?;
            self.partial.remove(&id);
        }
        self.mul(&[numerator.clone(), reciprocal])
    }
    pub fn probe_sign(
        &mut self,
        value: &OwnedExprV1,
    ) -> Result<crate::density_sign::SourceSignProbe, OwnedSignErrorV1> {
        let id = self.raw_id(value).map_err(OwnedSignErrorV1::Construction)?;
        self.graph.probe_sign(id).map_err(OwnedSignErrorV1::Sign)
    }
    pub fn sign_with_expanded_fallback(
        &mut self,
        source: &OwnedExprV1,
        expanded: &OwnedExprV1,
    ) -> Result<crate::density_sign::SignResult, OwnedSignErrorV1> {
        let source = self
            .raw_id(source)
            .map_err(OwnedSignErrorV1::Construction)?;
        let expanded = self
            .raw_id(expanded)
            .map_err(OwnedSignErrorV1::Construction)?;
        self.graph
            .sign_with_expanded_fallback(source, expanded)
            .map_err(OwnedSignErrorV1::Sign)
    }
    fn handle(&self, node: NodeId) -> OwnedExprV1 {
        OwnedExprV1 {
            node,
            arena: self.arena.clone(),
        }
    }
    pub fn raw_id(&self, value: &OwnedExprV1) -> ConstructResultV1<NodeId> {
        if !Arc::ptr_eq(&self.arena, &value.arena) {
            return Err(ConstructErrorV1::ForeignArena);
        }
        self.graph.node(value.node)?;
        debug_assert!(self.kinds.contains_key(&value.node));
        Ok(value.node)
    }
    fn ids(&self, args: &[OwnedExprV1]) -> ConstructResultV1<Vec<NodeId>> {
        args.iter().map(|v| self.raw_id(v)).collect()
    }
    fn kind(&self, id: NodeId) -> &Kind {
        &self.kinds[&id]
    }
    fn put(&mut self, node: Node, kind: Kind) -> ConstructResultV1<NodeId> {
        let partial = matches!(kind,Kind::IntegerPow{exponent,..} if exponent<0)
            || match &node {
                Node::Add(args) | Node::Mul(args) => {
                    args.iter().any(|id| self.partial.contains(id))
                }
                Node::Pow(a, b) | Node::Atan2(a, b) => {
                    self.partial.contains(a) || self.partial.contains(b)
                }
                Node::Sin(id) | Node::Cos(id) | Node::Atan(id) | Node::Tan(id) => {
                    self.partial.contains(id)
                }
                _ => false,
            };
        let id = self.graph.intern(node)?;
        self.kinds.insert(id, kind);
        if partial {
            self.partial.insert(id);
        }
        Ok(id)
    }
    pub fn contains_partial_reciprocal(&self, value: &OwnedExprV1) -> ConstructResultV1<bool> {
        Ok(self.partial.contains(&self.raw_id(value)?))
    }
    pub fn same_literal(&self, left: &OwnedExprV1, right: &OwnedExprV1) -> ConstructResultV1<bool> {
        let left = self.raw_id(left)?;
        let right = self.raw_id(right)?;
        Ok(self.graph.canonical_id(left)? == self.graph.canonical_id(right)?)
    }
    pub(crate) fn remembered_source_sign(
        &self,
        value: &OwnedExprV1,
    ) -> ConstructResultV1<Option<i8>> {
        Ok(self.source_sign_memo.get(&self.raw_id(value)?).copied())
    }
    pub(crate) fn remember_source_sign(
        &mut self,
        value: &OwnedExprV1,
        result: crate::density_sign::SignResult,
    ) -> ConstructResultV1<()> {
        let id = self.raw_id(value)?;
        if result.path != crate::density_sign::SignPath::Rational {
            self.source_sign_memo.insert(id, result.sign);
        }
        Ok(())
    }
    fn number(&self, id: NodeId) -> Option<&ScalarV2> {
        match self.kind(id) {
            Kind::Numeric(value) => Some(value),
            _ => None,
        }
    }
    fn rational_at(&self, id: NodeId) -> Option<Rational> {
        let (n, d) = self.number(id)?.value().as_rational()?;
        Some(Rational::new(n, d).expect("canonical V2 rational denominator"))
    }
    pub fn numeric_value(&self, value: &OwnedExprV1) -> ConstructResultV1<Option<&ScalarV2>> {
        Ok(self.number(self.raw_id(value)?))
    }
    pub fn form(&self, value: &OwnedExprV1) -> ConstructResultV1<OwnedFormV1> {
        let id = self.raw_id(value)?;
        let child = |id| -> ConstructResultV1<OwnedExprV1> {
            let id = self.graph.canonical_id(id)?;
            if !self.kinds.contains_key(&id) {
                return Err(ConstructErrorV1::Arena(ExpressionError::InvalidReference(
                    id,
                )));
            }
            Ok(self.handle(id))
        };
        Ok(match self.kind(id) {
            Kind::Numeric(v) => OwnedFormV1::Numeric(v.clone()),
            Kind::Pi => OwnedFormV1::Pi,
            Kind::Add(children) => {
                OwnedFormV1::Add(children.iter().map(|&id| self.handle(id)).collect())
            }
            Kind::Mul(children) => {
                OwnedFormV1::Mul(children.iter().map(|&id| self.handle(id)).collect())
            }
            Kind::IntegerPow { base, exponent } => OwnedFormV1::IntegerPow {
                base: self.handle(*base),
                exponent: *exponent,
            },
            Kind::Sin | Kind::Cos | Kind::Atan2 | Kind::OpaquePow => match self.graph.node(id)? {
                Node::Sin(a) => OwnedFormV1::Sin(child(*a)?),
                Node::Cos(a) => OwnedFormV1::Cos(child(*a)?),
                Node::Atan2(a, b) => OwnedFormV1::Atan2(child(*a)?, child(*b)?),
                Node::Pow(a, b) => {
                    let Node::Rational(exponent) = self.graph.node(*b)? else {
                        unreachable!("owned rational power")
                    };
                    OwnedFormV1::OpaquePow {
                        base: child(*a)?,
                        exponent: exponent.clone(),
                    }
                }
                _ => unreachable!("owned tag and raw decision shape"),
            },
        })
    }
    fn numeric_id(&mut self, value: ScalarV2) -> ConstructResultV1<NodeId> {
        let literal = value.literal_ast();
        let mut ids = Vec::with_capacity(literal.nodes().len());
        for node in literal.nodes() {
            let node = match node {
                LiteralNode::Q(n, d) => {
                    Node::Rational(Rational::new(n.clone(), d.clone()).expect("literal Q"))
                }
                LiteralNode::Pow { base, exponent } => Node::Pow(ids[*base], ids[*exponent]),
                LiteralNode::Mul { coefficient, root } => {
                    Node::Mul(vec![ids[*coefficient], ids[*root]])
                }
                LiteralNode::Add(args) => Node::Add(args.iter().map(|&id| ids[id]).collect()),
            };
            ids.push(self.graph.intern(node)?);
        }
        let id = ids[literal.root()];
        self.kinds.insert(id, Kind::Numeric(value));
        Ok(id)
    }
    pub fn numeric(&mut self, value: ScalarV2) -> ConstructResultV1<OwnedExprV1> {
        let id = self.numeric_id(value)?;
        Ok(self.handle(id))
    }
    pub fn rational(&mut self, value: &Rational) -> ConstructResultV1<OwnedExprV1> {
        self.numeric(ScalarV2::rational(value))
    }
    fn integer_id(&mut self, value: i64) -> ConstructResultV1<NodeId> {
        self.numeric_id(ScalarV2::rational(&Rational::integer(IBig::from(value))))
    }
    pub fn pi(&mut self) -> ConstructResultV1<OwnedExprV1> {
        let id = self.put(Node::Pi, Kind::Pi)?;
        Ok(self.handle(id))
    }
    pub fn raw_sin(&mut self, value: &OwnedExprV1) -> ConstructResultV1<OwnedExprV1> {
        let value = self.raw_id(value)?;
        let id = self.put(Node::Sin(value), Kind::Sin)?;
        Ok(self.handle(id))
    }
    pub fn raw_cos(&mut self, value: &OwnedExprV1) -> ConstructResultV1<OwnedExprV1> {
        let value = self.raw_id(value)?;
        let id = self.put(Node::Cos(value), Kind::Cos)?;
        Ok(self.handle(id))
    }
    pub fn raw_atan2(
        &mut self,
        y: &OwnedExprV1,
        x: &OwnedExprV1,
    ) -> ConstructResultV1<OwnedExprV1> {
        let y = self.raw_id(y)?;
        let x = self.raw_id(x)?;
        let id = self.put(Node::Atan2(y, x), Kind::Atan2)?;
        Ok(self.handle(id))
    }
    fn exponent(exponent: i64) -> ConstructResultV1<()> {
        if exponent.unsigned_abs() > MAX_EXPONENT_MAGNITUDE {
            Err(ConstructErrorV1::ExponentProfile)
        } else {
            Ok(())
        }
    }
    fn positive_half_power(&self, id: NodeId) -> Option<(NodeId, i64)> {
        if !matches!(self.kind(id), Kind::OpaquePow) {
            return None;
        }
        let Node::Pow(base, exponent) = self.graph.node(id).expect("owned node") else {
            return None;
        };
        let q = self.rational_at(*exponent)?;
        let (n, d) = q.parts();
        let p = i64::try_from(n.clone()).ok()?;
        (p > 0 && p % 2 == 1 && d == &UBig::from(2u8)).then_some((*base, p))
    }
    fn even_half_power(
        &mut self,
        base: NodeId,
        exponent: i64,
    ) -> ConstructResultV1<Option<NodeId>> {
        if exponent <= 0 || exponent % 2 != 0 {
            return Ok(None);
        }
        let Some((inner, p)) = self.positive_half_power(base) else {
            return Ok(None);
        };
        let combined = p
            .checked_mul(exponent / 2)
            .ok_or(ConstructErrorV1::ExponentProfile)?;
        Self::exponent(combined)?;
        // Только положительная половинная степень и чётная внешняя: domain base сохраняется.
        Ok(Some(self.power_id(inner, combined)?))
    }
    fn power_id(&mut self, base: NodeId, exponent: i64) -> ConstructResultV1<NodeId> {
        Self::exponent(exponent)?;
        if let Some(value) = self.number(base) {
            return self.numeric_id(value.integer_power(exponent)?);
        }
        if exponent < 0 {
            return Err(ConstructErrorV1::Profile(
                ProfileCode::FreshNegativeOpaquePower,
            ));
        }
        if exponent == 0 {
            if self.partial.contains(&base) {
                return Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard));
            }
            return self.integer_id(1);
        }
        if exponent == 1 {
            return Ok(base);
        }
        if let Some(result) = self.even_half_power(base, exponent)? {
            return Ok(result);
        }
        // Явная nested integer power сохраняет outer tree; только Mul собирает один layer.
        let power = self.integer_id(exponent)?;
        self.put(Node::Pow(base, power), Kind::IntegerPow { base, exponent })
    }
    pub fn integer_power(
        &mut self,
        value: &OwnedExprV1,
        exponent: i64,
    ) -> ConstructResultV1<OwnedExprV1> {
        let value = self.raw_id(value)?;
        let id = self.power_id(value, exponent)?;
        Ok(self.handle(id))
    }
    pub fn sqrt(&mut self, value: &OwnedExprV1) -> ConstructResultV1<OwnedExprV1> {
        let base = self.raw_id(value)?;
        if let Some(q) = self.rational_at(base) {
            if q.parts().0 < &IBig::ZERO {
                return Err(ConstructErrorV1::Profile(ProfileCode::NegativeRationalRoot));
            }
            let value = self.number(base).expect("Numeric rational").sqrt()?;
            return self.numeric(value);
        }
        let half = self.numeric_id(ScalarV2::rational(
            &Rational::new(IBig::ONE, UBig::from(2u8)).expect("half"),
        ))?;
        let id = self.put(Node::Pow(base, half), Kind::OpaquePow)?;
        Ok(self.handle(id))
    }
    pub fn raw_opaque_pow(
        &mut self,
        value: &OwnedExprV1,
        exponent: &Rational,
    ) -> ConstructResultV1<OwnedExprV1> {
        let base = self.raw_id(value)?;
        let (n, d) = exponent.parts();
        let n = i64::try_from(n.clone()).map_err(|_| ConstructErrorV1::ExponentProfile)?;
        Self::exponent(n)?;
        if d == &UBig::ONE {
            return self.integer_power(value, n);
        }
        if d != &UBig::from(2u8) {
            return Err(ConstructErrorV1::Profile(
                ProfileCode::HalfIntegerExponentRequired,
            ));
        }
        if self.rational_at(base).is_some() {
            let root = self.sqrt(value)?;
            return self.integer_power(&root, n);
        }
        if n < 0 {
            return Err(ConstructErrorV1::Profile(
                ProfileCode::FreshNegativeOpaquePower,
            ));
        }
        let power = self.numeric_id(ScalarV2::rational(exponent))?;
        let id = self.put(Node::Pow(base, power), Kind::OpaquePow)?;
        Ok(self.handle(id))
    }
    /// Raw partial reciprocal НЕ утверждает nonzero/domain. Numeric обхода class limit нет.
    pub fn formal_reciprocal(&mut self, value: &OwnedExprV1) -> ConstructResultV1<OwnedExprV1> {
        let base = self.raw_id(value)?;
        if self.number(base).is_some() {
            return self.integer_power(value, -1);
        }
        let exponent = self.integer_id(-1)?;
        let id = self.put(
            Node::Pow(base, exponent),
            Kind::IntegerPow { base, exponent: -1 },
        )?;
        Ok(self.handle(id))
    }
    pub fn partial_div(
        &mut self,
        left: &OwnedExprV1,
        right: &OwnedExprV1,
    ) -> ConstructResultV1<OwnedExprV1> {
        self.raw_id(left)?;
        self.raw_id(right)?;
        let reciprocal = self.formal_reciprocal(right)?;
        self.mul(&[left.clone(), reciprocal])
    }
    fn sorted(&self, args: &mut [NodeId]) -> ConstructResultV1<()> {
        // Все ids уже проверены: comparator error здесь означал бы нарушение arena invariant.
        args.sort_by(|a, b| compare(self.graph, *a, *b).expect("owned decision-tree ids"));
        Ok(())
    }
    fn coefficient_term(
        &mut self,
        coefficient: Rational,
        term: NodeId,
    ) -> ConstructResultV1<NodeId> {
        if coefficient.parts() == (&IBig::ONE, &UBig::ONE) {
            return Ok(term);
        }
        let q = self.numeric_id(ScalarV2::rational(&coefficient))?;
        // Add reassembly не повторяет rational-times-Add branch, но собирает Numeric factors.
        self.mul_flat_ids(&[q, term])
    }
    fn scalar_coefficient_term(
        &mut self,
        coefficient: ScalarV2,
        term: NodeId,
    ) -> ConstructResultV1<NodeId> {
        if coefficient.value().as_rational() == Some((IBig::ONE, UBig::ONE)) {
            return Ok(term);
        }
        let scalar = self.numeric_id(coefficient)?;
        self.mul_flat_ids(&[scalar, term])
    }
    fn add_ids(&mut self, args: &[NodeId]) -> ConstructResultV1<NodeId> {
        let mut flat = Vec::new();
        for &id in args {
            match self.kind(id) {
                Kind::Add(children) => flat.extend(children),
                _ => flat.push(id),
            }
        }
        let mut numeric = ScalarV2::rational(&Rational::integer(IBig::ZERO));
        let mut terms: HashMap<NodeId, ScalarV2> = HashMap::new();
        for id in flat {
            if let Some(value) = self.number(id) {
                numeric = numeric.add(value);
                continue;
            }
            let (coefficient, remainder) = match self.kind(id).clone() {
                Kind::Mul(mut children) => {
                    if let Some(position) =
                        children.iter().position(|&id| self.number(id).is_some())
                    {
                        // Numeric — целый Scalar, независимо от позиции в literal ordering.
                        let coefficient = self
                            .number(children.remove(position))
                            .expect("guard")
                            .clone();
                        let remainder = if children.len() == 1 {
                            children[0]
                        } else {
                            self.put(Node::Mul(children.clone()), Kind::Mul(children))?
                        };
                        (coefficient, remainder)
                    } else {
                        (ScalarV2::rational(&Rational::integer(IBig::ONE)), id)
                    }
                }
                _ => (ScalarV2::rational(&Rational::integer(IBig::ONE)), id),
            };
            terms
                .entry(remainder)
                .and_modify(|c| *c = c.add(&coefficient))
                .or_insert(coefficient);
        }
        let mut result = Vec::new();
        if !numeric.value().terms().is_empty() {
            result.push(self.numeric_id(numeric)?);
        }
        // Итог сортируется по AST; порядок обхода HashMap не влияет на literal export.
        for (term, coefficient) in terms {
            if coefficient.value().terms().is_empty() && self.partial.contains(&term) {
                return Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard));
            }
            if !coefficient.value().terms().is_empty() {
                result.push(self.scalar_coefficient_term(coefficient, term)?);
            }
        }
        self.sorted(&mut result)?;
        match result.as_slice() {
            [] => self.integer_id(0),
            [only] => Ok(*only),
            _ => self.put(Node::Add(result.clone()), Kind::Add(result)),
        }
    }
    pub fn add(&mut self, args: &[OwnedExprV1]) -> ConstructResultV1<OwnedExprV1> {
        let ids = self.ids(args)?;
        let id = self.add_ids(&ids)?;
        Ok(self.handle(id))
    }
    fn rational_times_add(&mut self, args: &[NodeId]) -> ConstructResultV1<Option<NodeId>> {
        if args.len() != 2 {
            return Ok(None);
        }
        let pair = if let Some(q) = self.rational_at(args[0]) {
            Some((q, args[1]))
        } else {
            self.rational_at(args[1]).map(|q| (q, args[0]))
        };
        let Some((factor, other)) = pair else {
            return Ok(None);
        };
        match self.kind(other).clone() {
            Kind::Add(children) => {
                let q = self.numeric_id(ScalarV2::rational(&factor))?;
                let terms = children
                    .iter()
                    .map(|&id| self.mul_flat_ids(&[q, id]))
                    .collect::<ConstructResultV1<Vec<_>>>()?;
                Ok(Some(self.add_ids(&terms)?))
            }
            Kind::Mul(children)
                if children.len() == 2
                    && self.rational_at(children[0]).is_some()
                    && matches!(self.kind(children[1]), Kind::Add(_)) =>
            {
                let coefficient =
                    rational_mul(&factor, &self.rational_at(children[0]).expect("guard"));
                if coefficient.parts().0 == &IBig::ZERO {
                    if self.partial.contains(&children[1]) {
                        return Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard));
                    }
                    return Ok(Some(self.integer_id(0)?));
                }
                Ok(Some(self.coefficient_term(coefficient, children[1])?))
            }
            _ => Ok(None),
        }
    }
    fn mul_ids(&mut self, args: &[NodeId]) -> ConstructResultV1<NodeId> {
        if let Some(result) = self.rational_times_add(args)? {
            return Ok(result);
        }
        self.mul_flat_ids(args)
    }
    fn mul_flat_ids(&mut self, args: &[NodeId]) -> ConstructResultV1<NodeId> {
        let mut queue: VecDeque<_> = args.iter().copied().collect();
        let mut numeric = ScalarV2::rational(&Rational::integer(IBig::ONE));
        let mut powers: HashMap<NodeId, i64> = HashMap::new();
        let mut opaque_powers = Vec::new();
        let mut half_pending = HashSet::new();
        loop {
            while let Some(id) = queue.pop_front() {
                if let Kind::Mul(children) = self.kind(id) {
                    for &child in children.iter().rev() {
                        queue.push_front(child);
                    }
                    continue;
                }
                if let Some(value) = self.number(id) {
                    numeric = numeric.mul(value);
                    continue;
                }
                if self.positive_half_power(id).is_some() {
                    if half_pending.remove(&id) {
                        // После пары снова обычный factor loop: Numeric и FormalMul не теряют границу.
                        let folded = self.power_id(id, 2)?;
                        queue.push_front(folded);
                    } else {
                        half_pending.insert(id);
                    }
                    continue;
                }
                if matches!(self.kind(id), Kind::OpaquePow)
                    || matches!(self.kind(id),Kind::IntegerPow{exponent,..} if *exponent<0)
                {
                    opaque_powers.push(id);
                    continue;
                }
                let (base, exponent) = match self.kind(id) {
                    Kind::IntegerPow { base, exponent } => (*base, *exponent),
                    _ => (id, 1),
                };
                let old = powers.get(&base).copied().unwrap_or(0);
                let exponent = old
                    .checked_add(exponent)
                    .ok_or(ConstructErrorV1::ExponentProfile)?;
                Self::exponent(exponent)?;
                powers.insert(base, exponent);
            }
            // Сначала весь один layer integer exponents: fold промежуточной суммы зависел бы от args order.
            let mut foldable: Vec<_> = powers
                .iter()
                .filter_map(|(&base, &exponent)| {
                    (exponent > 0 && exponent % 2 == 0 && self.positive_half_power(base).is_some())
                        .then_some(base)
                })
                .collect();
            if foldable.is_empty() {
                break;
            }
            self.sorted(&mut foldable)?;
            for base in foldable {
                let exponent = powers.remove(&base).expect("collected power");
                let folded = self
                    .even_half_power(base, exponent)?
                    .expect("positive even half power");
                queue.push_back(folded);
            }
        }
        if numeric.value().terms().is_empty() {
            if args.iter().any(|id| self.partial.contains(id)) {
                return Err(ConstructErrorV1::Profile(ProfileCode::PartialDomainDiscard));
            }
            return self.numeric_id(numeric);
        }
        let mut factors = opaque_powers;
        factors.extend(half_pending);
        for (base, exponent) in powers {
            factors.push(self.power_id(base, exponent)?);
        }
        let whole_rational = numeric.value().as_rational().is_some();
        let one = numeric.value().as_rational() == Some((IBig::ONE, UBig::ONE));
        if !one || factors.is_empty() {
            let number = self.numeric_id(numeric)?;
            if whole_rational && !factors.is_empty() {
                self.sorted(&mut factors)?;
                factors.insert(0, number);
            } else {
                factors.push(number);
                self.sorted(&mut factors)?;
            }
        } else {
            self.sorted(&mut factors)?;
        }
        match factors.as_slice() {
            [] => self.integer_id(1),
            [only] => Ok(*only),
            _ => self.put(Node::Mul(factors.clone()), Kind::Mul(factors)),
        }
    }
    pub fn mul(&mut self, args: &[OwnedExprV1]) -> ConstructResultV1<OwnedExprV1> {
        let ids = self.ids(args)?;
        let id = self.mul_ids(&ids)?;
        Ok(self.handle(id))
    }
    pub fn scaled(
        &mut self,
        value: &OwnedExprV1,
        factor: &Rational,
    ) -> ConstructResultV1<OwnedExprV1> {
        self.raw_id(value)?;
        let q = self.rational(factor)?;
        self.mul(&[q, value.clone()])
    }
    pub fn neg(&mut self, value: &OwnedExprV1) -> ConstructResultV1<OwnedExprV1> {
        self.scaled(value, &Rational::integer(-IBig::ONE))
    }
    pub fn sub(
        &mut self,
        left: &OwnedExprV1,
        right: &OwnedExprV1,
    ) -> ConstructResultV1<OwnedExprV1> {
        self.raw_id(left)?;
        self.raw_id(right)?;
        let negative = self.neg(right)?;
        self.add(&[left.clone(), negative])
    }
    pub fn div(
        &mut self,
        left: &OwnedExprV1,
        right: &OwnedExprV1,
    ) -> ConstructResultV1<OwnedExprV1> {
        self.raw_id(left)?;
        self.raw_id(right)?;
        let reciprocal = self.integer_power(right, -1)?;
        self.mul(&[left.clone(), reciprocal])
    }
    pub fn export_raw(&self, value: &OwnedExprV1) -> ConstructResultV1<DecisionLiteralV1> {
        let root = self.raw_id(value)?;
        let mut builder = LiteralBuilder::default();
        let root = builder.visit(self.graph, root)?;
        Ok(DecisionLiteralV1 {
            nodes: builder.nodes,
            root,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OwnedSignErrorV1 {
    Construction(ConstructErrorV1),
    Sign(crate::density_sign::SignFailure),
}

/// Immutable standalone raw view; интернер работает по первым встреченным exact rows.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum DecisionNodeV1 {
    Q(IBig, UBig),
    Pi,
    Add(Vec<usize>),
    Mul(Vec<usize>),
    Pow(usize, usize),
    Sin(usize),
    Cos(usize),
    Atan2(usize, usize),
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionLiteralV1 {
    nodes: Vec<DecisionNodeV1>,
    root: usize,
}
impl DecisionLiteralV1 {
    pub fn canon(&self) -> &'static str {
        CANON
    }
    pub fn nodes(&self) -> &[DecisionNodeV1] {
        &self.nodes
    }
    pub fn root(&self) -> usize {
        self.root
    }
}
#[derive(Default)]
struct LiteralBuilder {
    nodes: Vec<DecisionNodeV1>,
    index: HashMap<DecisionNodeV1, usize>,
    memo: HashMap<NodeId, usize>,
}
impl LiteralBuilder {
    fn visit(&mut self, graph: &Expressions, id: NodeId) -> ConstructResultV1<usize> {
        if let Some(&id) = self.memo.get(&id) {
            return Ok(id);
        }
        let node = match graph.node(id)? {
            Node::Rational(q) => DecisionNodeV1::Q(q.parts().0.clone(), q.parts().1.clone()),
            Node::Pi => DecisionNodeV1::Pi,
            Node::Add(args) | Node::Mul(args) => {
                let children = args
                    .iter()
                    .map(|&id| self.visit(graph, id))
                    .collect::<ConstructResultV1<Vec<_>>>()?;
                if matches!(graph.node(id)?, Node::Add(_)) {
                    DecisionNodeV1::Add(children)
                } else {
                    DecisionNodeV1::Mul(children)
                }
            }
            Node::Pow(a, b) => DecisionNodeV1::Pow(self.visit(graph, *a)?, self.visit(graph, *b)?),
            Node::Sin(a) => DecisionNodeV1::Sin(self.visit(graph, *a)?),
            Node::Cos(a) => DecisionNodeV1::Cos(self.visit(graph, *a)?),
            Node::Atan2(a, b) => {
                DecisionNodeV1::Atan2(self.visit(graph, *a)?, self.visit(graph, *b)?)
            }
            _ => unreachable!("owned whitelist never creates Atan/Tan"),
        };
        let local = if let Some(&id) = self.index.get(&node) {
            id
        } else {
            let id = self.nodes.len();
            self.nodes.push(node.clone());
            self.index.insert(node, id);
            id
        };
        self.memo.insert(id, local);
        Ok(local)
    }
}

pub type VectorExprV1 = [OwnedExprV1; 2];
pub type MatrixExprV1 = [[OwnedExprV1; 2]; 2];
fn validate_geometry(
    c: &ConstructorV1<'_>,
    matrix: &MatrixExprV1,
    vectors: &[&VectorExprV1],
) -> ConstructResultV1<()> {
    for row in matrix {
        for value in row {
            c.raw_id(value)?;
        }
    }
    for vector in vectors {
        for value in *vector {
            c.raw_id(value)?;
        }
    }
    Ok(())
}
/// Та же короткая scalar formula, но все промежуточные значения остаются owned.
pub fn dot(
    c: &mut ConstructorV1<'_>,
    matrix: &MatrixExprV1,
    left: &VectorExprV1,
    right: &VectorExprV1,
) -> ConstructResultV1<OwnedExprV1> {
    validate_geometry(c, matrix, &[left, right])?;
    let first_x = c.mul(&[matrix[0][0].clone(), right[0].clone()])?;
    let first_y = c.mul(&[matrix[0][1].clone(), right[1].clone()])?;
    let first_sum = c.add(&[first_x, first_y])?;
    let first = c.mul(&[left[0].clone(), first_sum])?;
    let second_x = c.mul(&[matrix[1][0].clone(), right[0].clone()])?;
    let second_y = c.mul(&[matrix[1][1].clone(), right[1].clone()])?;
    let second_sum = c.add(&[second_x, second_y])?;
    let second = c.mul(&[left[1].clone(), second_sum])?;
    c.add(&[first, second])
}
pub fn oriented_cross(
    c: &mut ConstructorV1<'_>,
    left: &VectorExprV1,
    right: &VectorExprV1,
    owner: i8,
) -> ConstructResultV1<OwnedExprV1> {
    for value in left.iter().chain(right) {
        c.raw_id(value)?;
    }
    if !matches!(owner, -1 | 1) {
        return Err(ConstructErrorV1::InvalidOrientation);
    }
    let a = c.mul(&[left[0].clone(), right[1].clone()])?;
    let b = c.mul(&[left[1].clone(), right[0].clone()])?;
    let cross = c.sub(&a, &b)?;
    c.scaled(&cross, &Rational::integer(IBig::from(owner)))
}
/// Первый consumer допускает только положительную WHOLE-rational squared norm.
/// Никакой cos²+sin² identity или normalization opaque ray здесь нет.
pub fn rational_norm_unit(
    c: &mut ConstructorV1<'_>,
    matrix: &MatrixExprV1,
    vector: &VectorExprV1,
) -> ConstructResultV1<VectorExprV1> {
    let norm = dot(c, matrix, vector, vector)?;
    let value = c
        .numeric_value(&norm)?
        .and_then(|v| v.value().as_rational());
    let Some((n, _)) = value else {
        return Err(ConstructErrorV1::Profile(ProfileCode::RationalNormRequired));
    };
    if n <= IBig::ZERO {
        return Err(ConstructErrorV1::NonPositiveNorm);
    }
    let length = c.sqrt(&norm)?;
    Ok([c.div(&vector[0], &length)?, c.div(&vector[1], &length)?])
}
pub fn oriented_basis(
    c: &mut ConstructorV1<'_>,
    matrix: &MatrixExprV1,
    vector: &VectorExprV1,
    owner: i8,
) -> ConstructResultV1<[VectorExprV1; 2]> {
    validate_geometry(c, matrix, &[vector])?;
    if !matches!(owner, -1 | 1) {
        return Err(ConstructErrorV1::InvalidOrientation);
    }
    let left = rational_norm_unit(c, matrix, vector)?;
    let gx0 = c.mul(&[matrix[0][0].clone(), left[0].clone()])?;
    let gx1 = c.mul(&[matrix[0][1].clone(), left[1].clone()])?;
    let gx = c.add(&[gx0, gx1])?;
    let gy0 = c.mul(&[matrix[1][0].clone(), left[0].clone()])?;
    let gy1 = c.mul(&[matrix[1][1].clone(), left[1].clone()])?;
    let gy = c.add(&[gy0, gy1])?;
    let normal = [
        c.scaled(&gy, &Rational::integer(IBig::from(-owner)))?,
        c.scaled(&gx, &Rational::integer(IBig::from(owner)))?,
    ];
    let normal = rational_norm_unit(c, matrix, &normal)?;
    Ok([left, normal])
}
pub fn principal_linear_combination(
    c: &mut ConstructorV1<'_>,
    basis: &[VectorExprV1; 2],
    angle: &OwnedExprV1,
) -> ConstructResultV1<VectorExprV1> {
    c.raw_id(angle)?;
    for vector in basis {
        for value in vector {
            c.raw_id(value)?;
        }
    }
    let cosine = c.raw_cos(angle)?;
    let sine = c.raw_sin(angle)?;
    let mut axis = |index: usize| {
        let a = c.mul(&[cosine.clone(), basis[0][index].clone()])?;
        let b = c.mul(&[sine.clone(), basis[1][index].clone()])?;
        c.add(&[a, b])
    };
    Ok([axis(0)?, axis(1)?])
}
