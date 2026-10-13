//! Partial source sp.expand, SymPy1.14: polynomial distribution с raw AST distinctions.
//! Запускается caller ТОЛЬКО после NeedsExpanded; controller/sign здесь не вызывается.
//! Opaque functions не перестраиваются; изменившиеся children дают internalUnsupported.
//! Root algebra ограничена finite squarefree divisors 30, порождёнными 2/3/5/6.
//! Arbitrary declared root leaf сохраняется raw; его arithmetic rewrite не допускается.
use crate::density_construct::{ConstructError, Constructor};
use crate::density_expression::{ExpressionError, Node, NodeId};
use crate::interval::rational_mul;
use crate::{IBig, Rational, UBig};
use cftuv_radical::arith::magnitude;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
pub struct ExpandLimits {
    pub max_terms: usize,
    pub max_rational_power_bits: usize,
    pub max_polynomial_exponent: u32,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UnsupportedProfile {
    NonPolynomialPower {
        node: NodeId,
    },
    ArbitraryRootAlgebra {
        node: NodeId,
        radicand: UBig,
    },
    IntegerExponentRange {
        node: NodeId,
        requested: IBig,
    },
    OpaqueChildChanges {
        node: NodeId,
        child: NodeId,
        expanded: NodeId,
    },
    TermLimit {
        requested: usize,
        maximum: usize,
    },
    TermCountOverflow,
    PolynomialExponentLimit {
        node: NodeId,
        requested: u32,
        maximum: u32,
    },
    RationalPowerBits {
        requested: u64,
        maximum: usize,
    },
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExpandError {
    UnsupportedProfile(UnsupportedProfile),
    Constructor(ConstructError),
    Arena(ExpressionError),
}
impl From<ConstructError> for ExpandError {
    fn from(v: ConstructError) -> Self {
        Self::Constructor(v)
    }
}
impl From<ExpressionError> for ExpandError {
    fn from(v: ExpressionError) -> Self {
        Self::Arena(v)
    }
}
pub type ExpandResult<T> = Result<T, ExpandError>;
fn refuse(v: UnsupportedProfile) -> ExpandError {
    ExpandError::UnsupportedProfile(v)
}

pub fn expand(
    context: &mut Constructor<'_>,
    source: NodeId,
    limits: ExpandLimits,
) -> ExpandResult<NodeId> {
    Worker {
        context,
        limits,
        memo: HashMap::new(),
    }
    .node(source)
}
struct Worker<'a, 'g> {
    context: &'a mut Constructor<'g>,
    limits: ExpandLimits,
    memo: HashMap<NodeId, NodeId>,
}
impl Worker<'_, '_> {
    fn equal(&self, a: NodeId, b: NodeId) -> bool {
        self.context
            .graph()
            .canonical_id(a)
            .expect("validated arena")
            == self
                .context
                .graph()
                .canonical_id(b)
                .expect("validated arena")
    }
    fn q(&mut self, value: Rational) -> ExpandResult<NodeId> {
        Ok(self.context.push(Node::Rational(value))?)
    }
    fn integer(&mut self, value: i64) -> ExpandResult<NodeId> {
        self.q(Rational::integer(IBig::from(value)))
    }
    fn node(&mut self, id: NodeId) -> ExpandResult<NodeId> {
        if let Some(&result) = self.memo.get(&id) {
            return Ok(result);
        }
        let result = match self.context.node(id)?.clone() {
            Node::Rational(_) | Node::Pi => id,
            Node::Add(args) => {
                let mut children = Vec::new();
                for &child in &args {
                    children.push(self.node(child)?);
                }
                // Source unevaluated Add(1,1) остаётся raw, если children не изменились.
                if args.iter().zip(&children).all(|(&a, &b)| self.equal(a, b)) {
                    id
                } else {
                    for &child in &children {
                        self.algebra_supported(child)?;
                    }
                    self.context.add_for_expansion(&children)?
                }
            }
            Node::Mul(args) => {
                let mut children = Vec::new();
                for child in args {
                    children.push(self.node(child)?);
                }
                self.product(&children)?
            }
            Node::Pow(base, exponent) => {
                if self.context.registered_sqrt_radicand(id)?.is_some() {
                    id
                } else {
                    let Node::Rational(e) = self.context.node(exponent)?.clone() else {
                        return Err(refuse(UnsupportedProfile::NonPolynomialPower { node: id }));
                    };
                    if e.parts().1 != &UBig::ONE || e.parts().0 < &IBig::ZERO {
                        return Err(refuse(UnsupportedProfile::NonPolynomialPower { node: id }));
                    }
                    let n = u32::try_from(e.parts().0).map_err(|_| {
                        refuse(UnsupportedProfile::IntegerExponentRange {
                            node: id,
                            requested: e.parts().0.clone(),
                        })
                    })?;
                    let expanded = self.node(base)?;
                    match self.context.node(expanded)? {
                        Node::Add(_) | Node::Mul(_) => self.new_power(expanded, n)?,
                        _ if self.equal(base, expanded) => id, // bare raw Pow(2,2) source не пересоздаёт.
                        _ => self.new_power(expanded, n)?,
                    }
                }
            }
            Node::Sin(a) | Node::Cos(a) | Node::Atan(a) | Node::Tan(a) => {
                let expanded = self.node(a)?;
                if !self.equal(a, expanded) {
                    return Err(refuse(UnsupportedProfile::OpaqueChildChanges {
                        node: id,
                        child: a,
                        expanded,
                    }));
                }
                id
            }
            Node::Atan2(a, b) => {
                for child in [a, b] {
                    let expanded = self.node(child)?;
                    if !self.equal(child, expanded) {
                        return Err(refuse(UnsupportedProfile::OpaqueChildChanges {
                            node: id,
                            child,
                            expanded,
                        }));
                    }
                }
                id
            }
        };
        self.memo.insert(id, result);
        Ok(result)
    }
    fn fixed_root_family(n: &UBig) -> bool {
        [2u8, 3, 5, 6, 10, 15, 30]
            .into_iter()
            .any(|v| n == &UBig::from(v))
    }
    fn algebra_supported(&self, id: NodeId) -> ExpandResult<()> {
        if let Some(radicand) = self.context.registered_sqrt_radicand(id)? {
            if !Self::fixed_root_family(&radicand) {
                return Err(refuse(UnsupportedProfile::ArbitraryRootAlgebra {
                    node: id,
                    radicand,
                }));
            }
            return Ok(());
        }
        // Opaque functions — формальные атомы; node() уже проверил unchanged children.
        match self.context.node(id)? {
            Node::Add(args) | Node::Mul(args) => {
                for &child in args {
                    self.algebra_supported(child)?;
                }
            }
            Node::Pow(base, _) => self.algebra_supported(*base)?,
            _ => {}
        }
        Ok(())
    }
    fn product(&mut self, factors: &[NodeId]) -> ExpandResult<NodeId> {
        let mut terms: Vec<Vec<NodeId>> = vec![Vec::new()];
        for &factor in factors {
            self.algebra_supported(factor)?;
            let choices = match self.context.node(factor)?.clone() {
                Node::Add(children) => children,
                _ => vec![factor],
            };
            let count = terms
                .len()
                .checked_mul(choices.len())
                .ok_or_else(|| refuse(UnsupportedProfile::TermCountOverflow))?;
            if count > self.limits.max_terms {
                return Err(refuse(UnsupportedProfile::TermLimit {
                    requested: count,
                    maximum: self.limits.max_terms,
                }));
            }
            let mut next = Vec::with_capacity(count);
            for term in terms {
                for &choice in &choices {
                    let mut value = term.clone();
                    value.push(choice);
                    next.push(value);
                }
            }
            terms = next;
        }
        let mut products = Vec::new();
        for term in terms {
            products.push(self.context.mul_for_expansion(&term)?);
        }
        Ok(self.context.add_for_expansion(&products)?)
    }
    fn rational_power(&mut self, value: Rational, n: u32) -> ExpandResult<NodeId> {
        let bits = |v: &UBig| {
            let words = v.as_words();
            words.len().checked_sub(1).map_or(0, |i| {
                i * std::mem::size_of_val(&words[i]) * 8 + std::mem::size_of_val(&words[i]) * 8
                    - words[i].leading_zeros() as usize
            })
        };
        let bound = (bits(&magnitude(value.parts().0)).max(bits(value.parts().1)) as u64)
            .checked_mul(u64::from(n))
            .unwrap_or(u64::MAX);
        if bound > self.limits.max_rational_power_bits as u64 {
            return Err(refuse(UnsupportedProfile::RationalPowerBits {
                requested: bound,
                maximum: self.limits.max_rational_power_bits,
            }));
        }
        let mut result = Rational::integer(IBig::ONE);
        let mut value = value;
        let mut n = n;
        while n != 0 {
            if n & 1 != 0 {
                result = rational_mul(&result, &value);
            }
            n >>= 1;
            if n != 0 {
                value = rational_mul(&value, &value);
            }
        }
        self.q(result)
    }
    fn new_power(&mut self, base: NodeId, n: u32) -> ExpandResult<NodeId> {
        if n == 0 {
            return self.integer(1);
        }
        if n == 1 {
            return Ok(base);
        }
        if let Some(radicand) = self.context.registered_sqrt_radicand(base)? {
            self.algebra_supported(base)?;
            let power = self.rational_power(Rational::integer(IBig::from(radicand)), n / 2)?;
            return if n % 2 == 0 {
                Ok(power)
            } else {
                Ok(self.context.mul_for_expansion(&[power, base])?)
            };
        }
        match self.context.node(base)?.clone() {
            Node::Rational(q) => self.rational_power(q, n),
            Node::Add(_) => {
                if n > self.limits.max_polynomial_exponent {
                    return Err(refuse(UnsupportedProfile::PolynomialExponentLimit {
                        node: base,
                        requested: n,
                        maximum: self.limits.max_polynomial_exponent,
                    }));
                }
                let mut result = self.integer(1)?;
                for _ in 0..n {
                    result = self.product(&[result, base])?;
                }
                Ok(result)
            }
            Node::Mul(args) => {
                let mut powers = Vec::new();
                for child in args {
                    powers.push(self.new_power(child, n)?);
                }
                self.product(&powers)
            }
            Node::Pow(inner, e) => {
                let Node::Rational(e) = self.context.node(e)?.clone() else {
                    return Err(refuse(UnsupportedProfile::NonPolynomialPower {
                        node: base,
                    }));
                };
                if e.parts().1 != &UBig::ONE || e.parts().0 < &IBig::ZERO {
                    return Err(refuse(UnsupportedProfile::NonPolynomialPower {
                        node: base,
                    }));
                }
                let product = e.parts().0 * IBig::from(n);
                let n = u32::try_from(&product).map_err(|_| {
                    refuse(UnsupportedProfile::IntegerExponentRange {
                        node: base,
                        requested: product,
                    })
                })?;
                self.new_power(inner, n)
            }
            _ => {
                let exponent = self.integer(i64::from(n))?;
                Ok(self.context.push(Node::Pow(base, exponent))?)
            }
        }
    }
}
