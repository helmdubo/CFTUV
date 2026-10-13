//! Numeric scalar island candidate DENSITY_SOURCE_CONSTRUCTION_V1, отдельно от legacy AST.
//! Свежая арифметика — actual Radical; literal encoder не читает SymPy/cache/registry.
//! Origin: reference/density_scalar_v2.py; Work здесь не агрегируется и не является Density budget.
use crate::{IBig, Radical, Rational, UBig};
use cftuv_radical::{Failure, RefusalCode};
use std::collections::HashMap;

pub const CANON: &str = "DENSITY_SOURCE_CONSTRUCTION_V1";
/// Явный internal prototype profile; это не ограничение Python и не product refusal.
pub const MAX_EXPONENT_MAGNITUDE: u64 = 1 << 20;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileCode {
    RootRequiresNonnegativeRational,
    InverseClassLimit,
}
impl ProfileCode {
    pub fn code(self) -> &'static str {
        match self {
            Self::RootRequiresNonnegativeRational => {
                "DENSITY_SCALAR_V2_ROOT_REQUIRES_NONNEGATIVE_RATIONAL"
            }
            Self::InverseClassLimit => "DENSITY_SCALAR_V2_INVERSE_CLASS_LIMIT",
        }
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProfileUnsupported {
    pub code: ProfileCode,
    pub detail: String,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ScalarError {
    Profile(ProfileUnsupported),
    Algebra(Failure),
    ExponentProfile { requested: i64, maximum: u64 },
}
pub type ScalarResult<T> = Result<T, ScalarError>;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct ScalarV2 {
    value: Radical,
}
impl ScalarV2 {
    /// Принимается actual Radical со стандартным canonical-value контрактом этого типа.
    pub fn from_value(value: Radical) -> Self {
        Self { value }
    }
    pub fn value(&self) -> &Radical {
        &self.value
    }
    pub fn rational(value: &Rational) -> Self {
        Self::from_value(value.radical())
    }
    pub fn sqrt_rational(value: &Rational) -> ScalarResult<Self> {
        Self::rational(value).sqrt()
    }
    pub fn add(&self, other: &Self) -> Self {
        Self::from_value(self.value.add(&other.value).0)
    }
    pub fn sub(&self, other: &Self) -> Self {
        Self::from_value(self.value.sub(&other.value).0)
    }
    pub fn mul(&self, other: &Self) -> Self {
        Self::from_value(self.value.mul(&other.value).0)
    }
    pub fn neg(&self) -> Self {
        Self::from_value(self.value.neg())
    }
    pub fn scaled(&self, factor: &Rational) -> Self {
        let (n, d) = factor.parts();
        Self::from_value(self.value.scaled(n, d))
    }
    pub fn inverse(&self) -> ScalarResult<Self> {
        match self.value.inverse() {
            Ok((value, _)) => Ok(Self::from_value(value)),
            Err(Failure::Refused(v)) if v.code == RefusalCode::InverseClassLimit => {
                Err(ScalarError::Profile(ProfileUnsupported {
                    code: ProfileCode::InverseClassLimit,
                    detail: v.detail,
                }))
            }
            Err(error) => Err(ScalarError::Algebra(error)),
        }
    }
    pub fn div(&self, other: &Self) -> ScalarResult<Self> {
        Ok(self.mul(&other.inverse()?))
    }
    pub fn integer_power(&self, exponent: i64) -> ScalarResult<Self> {
        let mut remaining = exponent.unsigned_abs();
        if remaining > MAX_EXPONENT_MAGNITUDE {
            return Err(ScalarError::ExponentProfile {
                requested: exponent,
                maximum: MAX_EXPONENT_MAGNITUDE,
            });
        }
        // Source __pow__: inverse(BASE) до любых степеней, без обхода class limit.
        let mut base = if exponent < 0 {
            self.inverse()?
        } else {
            self.clone()
        };
        let mut result = Self::from_value(Radical::one());
        while remaining != 0 {
            if remaining & 1 != 0 {
                result = result.mul(&base);
            }
            remaining >>= 1;
            if remaining != 0 {
                base = base.mul(&base);
            }
        }
        Ok(result)
    }
    pub fn sqrt(&self) -> ScalarResult<Self> {
        let rational = self.value.as_rational().filter(|(n, _)| n >= &IBig::ZERO);
        let Some((n, d)) = rational else {
            return Err(ScalarError::Profile(ProfileUnsupported {
                code:ProfileCode::RootRequiresNonnegativeRational,
                detail:"sqrt requires the whole scalar value to be a nonnegative rational; opaque roots remain outside this layer".into(),
            }));
        };
        let (value, _) = Radical::sqrt_of_rational(n, d)
            .map_err(|v| ScalarError::Algebra(Failure::Refused(v)))?;
        Ok(Self::from_value(value))
    }
    pub fn literal_ast(&self) -> ScalarLiteralAstV1 {
        scalar_literal_ast(&self.value)
    }
}

/// Только raw literal rows: это НОВЫЙ canon, не shape ambient SymPy constructors.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum LiteralNode {
    Q(IBig, UBig),
    Pow { base: usize, exponent: usize },
    Mul { coefficient: usize, root: usize },
    Add(Vec<usize>),
}
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct ScalarLiteralAstV1 {
    nodes: Vec<LiteralNode>,
    root: usize,
}
impl ScalarLiteralAstV1 {
    pub fn canon(&self) -> &'static str {
        CANON
    }
    pub fn nodes(&self) -> &[LiteralNode] {
        &self.nodes
    }
    pub fn root(&self) -> usize {
        self.root
    }
}
#[derive(Default)]
struct Builder {
    nodes: Vec<LiteralNode>,
    index: HashMap<LiteralNode, usize>,
}
impl Builder {
    fn node(&mut self, node: LiteralNode) -> usize {
        if let Some(&id) = self.index.get(&node) {
            return id;
        }
        let id = self.nodes.len();
        self.nodes.push(node.clone());
        self.index.insert(node, id);
        id
    }
    fn term(&mut self, term: &cftuv_radical::Term) -> usize {
        if term.radicand == UBig::ONE {
            return self.node(LiteralNode::Q(
                term.numerator.clone(),
                term.denominator.clone(),
            ));
        }
        let coefficient = if term.numerator == IBig::ONE && term.denominator == UBig::ONE {
            None
        } else {
            Some(self.node(LiteralNode::Q(
                term.numerator.clone(),
                term.denominator.clone(),
            )))
        };
        let base = self.node(LiteralNode::Q(IBig::from(term.radicand.clone()), UBig::ONE));
        let half = self.node(LiteralNode::Q(IBig::ONE, UBig::from(2u8)));
        let root = self.node(LiteralNode::Pow {
            base,
            exponent: half,
        });
        coefficient.map_or(root, |coefficient| {
            self.node(LiteralNode::Mul { coefficient, root })
        })
    }
}
/// Child-first, exact-row interning по первому появлению; HashMap никогда не итерируется.
/// Term order задаётся actual V2 canonical mprime; rational class имеет mprime1.
pub fn scalar_literal_ast(value: &Radical) -> ScalarLiteralAstV1 {
    let mut builder = Builder::default();
    let children: Vec<_> = value.terms().iter().map(|v| builder.term(v)).collect();
    let root = match children.as_slice() {
        [] => builder.node(LiteralNode::Q(IBig::ZERO, UBig::ONE)),
        [only] => *only,
        _ => builder.node(LiteralNode::Add(children)),
    };
    ScalarLiteralAstV1 {
        nodes: builder.nodes,
        root,
    }
}
