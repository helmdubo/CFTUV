//! Конечный source Add/Mul: commutative Density AST, SymPy 1.14 construction order.
//! Числовые radical/Pow simplifications требуют отдельного admission-профиля.
//! Declared root gcd closure доказывает значение, НЕ общий source MulAST.
//! Синтаксис не заменяется выражением произвольного Radical равного значения.
//! Источник core/add.py SHA256 4dd770b22719151e3212d5539199a4cc5b67f677409617da77bbc98840abea8e;
//! core/mul.py SHA256 ae30a2c8eab1320513591f2c01c9a29517d02eea7dc2ff088fbb3b2487d49153.
/*
Copyright (c) 2006-2023 SymPy Development Team
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
a. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
b. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
c. Neither the name of SymPy nor the names of its contributors may be used
   to endorse or promote products derived from this software without
   specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
use cftuv_radical::arith::gcd;
use std::collections::VecDeque;

use crate::density_expression::{ExpressionError, Expressions, Node, NodeId};
use crate::density_expression_order::compare;
use crate::interval::{rational_add, rational_mul};
use crate::{IBig, Rational, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UnsupportedProfile {
    NumericPowerInMul(NodeId),
    NumericPowerReevaluationInAdd(NodeId),
    NumericPowerCoefficientInAdd(NodeId),
    NonIntegerPowerInMul(NodeId),
    CompoundPowerBase(NodeId),
    NonCanonicalCoefficientPlacement(NodeId),
    NodeOutsideProfile(NodeId),
    FixedRootOutsideProfile(u32),
    InvalidSourceRootDeclaration { node: NodeId, radicand: UBig },
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConstructError {
    UnsupportedProfile(UnsupportedProfile),
    Arena(ExpressionError),
}
impl From<ExpressionError> for ConstructError {
    fn from(error: ExpressionError) -> Self {
        Self::Arena(error)
    }
}
pub type ConstructResult<T> = Result<T, ConstructError>;

/// Явное заявление caller о canonical source boundary, НЕ native squarefree proof.
/// Production importer не может выводить этот контракт только из формы raw Pow.
/// Даже true squarefree не доказывает общий source MulAST: SymPy factorization ограничена.
/// Gcd даёт canonical VALUE closure; source AST parity здесь только эмпирический staging.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SourceSqrtContract {
    Sympy114CanonicalPositiveIntegerSqrt,
}

#[derive(Default)]
struct RootRegistry {
    roots: Vec<(NodeId, UBig)>,
    formal_tan: bool,
}
impl std::ops::Deref for RootRegistry {
    type Target = Vec<(NodeId, UBig)>;
    fn deref(&self) -> &Self::Target {
        &self.roots
    }
}
impl std::ops::DerefMut for RootRegistry {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.roots
    }
}

/// Registry живёт только при exclusive borrow того же append-only arena.
/// Drop теряет declarations; внешний reset невозможен при живом Constructor.
/// Declared arbitrary roots не имеют общего source AST closure proof.
/// Fixed2/3/5/6 small generated closure и recorded поле проверены отдельно.
pub struct Constructor<'a> {
    graph: &'a mut Expressions,
    roots: RootRegistry,
}
impl<'a> Constructor<'a> {
    pub fn new(graph: &'a mut Expressions) -> Self {
        Self {
            graph,
            roots: RootRegistry::default(),
        }
    }
    pub fn graph(&self) -> &Expressions {
        self.graph
    }
    pub fn node(&self, id: NodeId) -> Result<&Node, ExpressionError> {
        self.graph.node(id)
    }
    /// Readonly registry query: caller declaration, НЕ проверка squarefree по форме.
    pub fn registered_sqrt_radicand(&self, id: NodeId) -> ConstructResult<Option<UBig>> {
        self.graph.node(id)?;
        Ok(registered_radicand(self.graph, &self.roots, id))
    }
    pub fn interval(
        &mut self,
        id: NodeId,
        precision: crate::legacy_mpf::Precision,
    ) -> crate::density_expression::ExpressionResult<crate::legacy_interval::LegacyInterval> {
        self.graph.interval(id, precision)
    }
    pub fn sign_with_expanded_fallback(
        &mut self,
        source: NodeId,
        expanded: NodeId,
    ) -> Result<crate::density_sign::SignResult, crate::density_sign::SignFailure> {
        self.graph.sign_with_expanded_fallback(source, expanded)
    }
    pub fn probe_sign(
        &mut self,
        source: NodeId,
    ) -> Result<crate::density_sign::SourceSignProbe, crate::density_sign::SignFailure> {
        self.graph.probe_sign(source)
    }
    pub fn push(&mut self, node: Node) -> ConstructResult<NodeId> {
        Ok(self.graph.intern(node)?)
    }
    /// Import/replay сохраняет raw append numbering; это не выдаёт root provenance.
    pub fn push_raw(&mut self, node: Node) -> ConstructResult<NodeId> {
        Ok(self.graph.push(node)?)
    }
    pub fn add(&mut self, args: &[NodeId]) -> ConstructResult<NodeId> {
        add_inner(self.graph, args, &mut self.roots)
    }
    pub fn mul(&mut self, args: &[NodeId]) -> ConstructResult<NodeId> {
        mul_inner(self.graph, args, &mut self.roots)
    }
    /// Formal expand разрешает opaque Tan; interval/Quadratic grammar НЕ меняется.
    pub fn add_for_expansion(&mut self, args: &[NodeId]) -> ConstructResult<NodeId> {
        let previous = self.roots.formal_tan;
        self.roots.formal_tan = true;
        let result = add_inner(self.graph, args, &mut self.roots);
        self.roots.formal_tan = previous;
        result
    }
    pub fn mul_for_expansion(&mut self, args: &[NodeId]) -> ConstructResult<NodeId> {
        let previous = self.roots.formal_tan;
        self.roots.formal_tan = true;
        let result = mul_inner(self.graph, args, &mut self.roots);
        self.roots.formal_tan = previous;
        result
    }
    pub fn fixed_sqrt(&mut self, radicand: u32) -> ConstructResult<NodeId> {
        if !matches!(radicand, 2 | 3 | 5 | 6) {
            return Err(refuse(UnsupportedProfile::FixedRootOutsideProfile(
                radicand,
            )));
        }
        registered_root(self.graph, UBig::from(radicand), &mut self.roots)
    }
    /// Проверяет только AST/positive integer. Squarefree поступает от caller contract.
    /// Это доверенная граница; метод не доказывает честность declaration факторизацией.
    pub fn declare_source_sqrt(
        &mut self,
        id: NodeId,
        radicand: UBig,
        _contract: SourceSqrtContract,
    ) -> ConstructResult<()> {
        let valid = if let Node::Pow(base, exponent) = self.graph.node(id)? {
            match (self.graph.node(*base)?, self.graph.node(*exponent)?) {
                (Node::Rational(n), Node::Rational(e)) => {
                    n.parts() == (&IBig::from(radicand.clone()), &UBig::ONE)
                        && e.parts() == (&IBig::ONE, &UBig::from(2u8))
                        && radicand > UBig::ONE
                }
                _ => false,
            }
        } else {
            false
        };
        if !valid {
            return Err(refuse(UnsupportedProfile::InvalidSourceRootDeclaration {
                node: id,
                radicand,
            }));
        }
        let canonical = self.graph.canonical_id(id)?;
        if self.roots.iter().all(|(old, _)| *old != canonical) {
            self.roots.push((canonical, radicand));
        }
        Ok(())
    }
}

fn registered_root(
    graph: &mut Expressions,
    radicand: UBig,
    roots: &mut RootRegistry,
) -> ConstructResult<NodeId> {
    debug_assert!(radicand > UBig::ONE);
    let base = push_q(graph, Rational::integer(IBig::from(radicand.clone())))?;
    let half = push_q(
        graph,
        Rational::new(IBig::ONE, UBig::from(2u8)).expect("positive denominator"),
    )?;
    let id = graph.intern(Node::Pow(base, half))?;
    // Повтор возможен только по тому же AST внутри registry, не по Radical value.
    if let Some((existing, _)) = roots.iter().find(|(old, _)| equal(graph, *old, id)) {
        return Ok(*existing);
    }
    roots.push((id, radicand));
    Ok(id)
}
fn registered_radicand(graph: &Expressions, roots: &RootRegistry, id: NodeId) -> Option<UBig> {
    let id = graph.canonical_id(id).expect("validated append-only arena");
    roots
        .iter()
        .find(|(old, _)| *old == id)
        .map(|(_, n)| n.clone())
}

fn refuse(reason: UnsupportedProfile) -> ConstructError {
    ConstructError::UnsupportedProfile(reason)
}
fn integer(value: i64) -> Rational {
    Rational::integer(IBig::from(value))
}
fn is(value: &Rational, n: i64) -> bool {
    value.parts() == (&IBig::from(n), &UBig::ONE)
}
fn push_q(graph: &mut Expressions, value: Rational) -> ConstructResult<NodeId> {
    Ok(graph.intern(Node::Rational(value))?)
}
fn sort(graph: &Expressions, args: &mut [NodeId]) {
    // Все references уже проверены node()/push(); compare не может отказать здесь.
    args.sort_by(|a, b| compare(graph, *a, *b).expect("validated append-only arena"));
}
fn equal(graph: &Expressions, a: NodeId, b: NodeId) -> bool {
    compare(graph, a, b)
        .expect("validated append-only arena")
        .is_eq()
}
fn nonidentity(
    graph: &Expressions,
    args: &[NodeId],
    identity: i64,
    formal_tan: bool,
) -> ConstructResult<Vec<NodeId>> {
    let mut result = Vec::new();
    for &id in args {
        let node = graph.node(id)?;
        if !formal_tan && matches!(node, Node::Tan(_)) {
            return Err(refuse(UnsupportedProfile::NodeOutsideProfile(id)));
        }
        if !matches!(node,Node::Rational(q) if is(q,identity)) {
            result.push(id);
        }
    }
    Ok(result)
}

fn split_coefficient(graph: &mut Expressions, id: NodeId) -> ConstructResult<(Rational, NodeId)> {
    let Node::Mul(args) = graph.node(id)?.clone() else {
        return Ok((integer(1), id));
    };
    let Node::Rational(coefficient) = graph.node(args[0])?.clone() else {
        return Ok((integer(1), id));
    };
    let remaining = &args[1..];
    if remaining
        .iter()
        .any(|&child| matches!(graph.node(child), Ok(Node::Rational(_))))
    {
        return Err(refuse(
            UnsupportedProfile::NonCanonicalCoefficientPlacement(id),
        ));
    }
    let term = if remaining.len() == 1 {
        remaining[0]
    } else {
        graph.intern(Node::Mul(remaining.to_vec()))?
    };
    Ok((coefficient, term))
}

fn coefficient_term(
    graph: &mut Expressions,
    coefficient: Rational,
    term: NodeId,
    roots: &mut RootRegistry,
) -> ConstructResult<NodeId> {
    if is(&coefficient, 1) {
        return Ok(term);
    }
    let q = push_q(graph, coefficient)?;
    match graph.node(term)?.clone() {
        Node::Mul(args) => {
            let mut args = args;
            args.insert(0, q);
            Ok(graph.intern(Node::Mul(args))?)
        }
        Node::Add(_) => Ok(graph.intern(Node::Mul(vec![q, term]))?),
        Node::Pow(base, _) if matches!(graph.node(base)?, Node::Rational(_)) => {
            if registered_radicand(graph, roots, term).is_some() {
                Ok(graph.intern(Node::Mul(vec![q, term]))?)
            } else {
                Err(refuse(UnsupportedProfile::NumericPowerCoefficientInAdd(
                    term,
                )))
            }
        }
        _ => mul_inner(graph, &[q, term], roots),
    }
}

/// Source Add: flatten, coefficient collection, structural like-terms и Basic.compare order.
/// Canonical positive numeric Pow можно оставить opaque; их новая scaling не доказана.
pub fn add(graph: &mut Expressions, args: &[NodeId]) -> ConstructResult<NodeId> {
    add_inner(graph, args, &mut RootRegistry::default())
}
fn add_inner(
    graph: &mut Expressions,
    args: &[NodeId],
    roots: &mut RootRegistry,
) -> ConstructResult<NodeId> {
    let args = nonidentity(graph, args, 0, roots.formal_tan)?;
    if args.is_empty() {
        return push_q(graph, integer(0));
    }
    if args.len() == 1 {
        return Ok(args[0]);
    }
    let mut queue: VecDeque<_> = args.into();
    let mut constant = integer(0);
    let mut terms: Vec<(NodeId, Rational)> = Vec::new();
    while let Some(id) = queue.pop_front() {
        match graph.node(id)?.clone() {
            Node::Add(children) => queue.extend(children),
            Node::Rational(q) => constant = rational_add(&constant, &q),
            Node::Tan(_) if !roots.formal_tan => {
                return Err(refuse(UnsupportedProfile::NodeOutsideProfile(id)))
            }
            node => {
                if let Node::Pow(base, exponent) = node {
                    if let (Node::Rational(_), Node::Rational(e)) =
                        (graph.node(base)?, graph.node(exponent)?)
                    {
                        if e.parts().1 == &UBig::ONE || e.parts().0 < &IBig::ZERO {
                            return Err(refuse(UnsupportedProfile::NumericPowerReevaluationInAdd(
                                id,
                            )));
                        }
                    }
                }
                let (coefficient, term) = split_coefficient(graph, id)?;
                if let Some((_, old)) = terms.iter_mut().find(|(t, _)| equal(graph, *t, term)) {
                    *old = rational_add(old, &coefficient);
                } else {
                    terms.push((term, coefficient));
                }
            }
        }
    }
    let mut result = Vec::new();
    for (term, coefficient) in terms {
        if !is(&coefficient, 0) {
            result.push(coefficient_term(graph, coefficient, term, roots)?);
        }
    }
    if !is(&constant, 0) {
        result.push(push_q(graph, constant)?);
    }
    sort(graph, &mut result);
    match result.len() {
        0 => push_q(graph, integer(0)),
        1 => Ok(result[0]),
        _ => Ok(graph.intern(Node::Add(result))?),
    }
}

// Source _keep_coeff для two-arg rational×Add: Mul-term fast path остаётся raw.
fn keep_coefficient(
    graph: &mut Expressions,
    coefficient: &Rational,
    term: NodeId,
    roots: &mut RootRegistry,
) -> ConstructResult<NodeId> {
    match graph.node(term)?.clone() {
        Node::Rational(q) => push_q(graph, rational_mul(coefficient, &q)),
        Node::Mul(_) => {
            let (q, rest) = split_coefficient(graph, term)?;
            coefficient_term(graph, rational_mul(coefficient, &q), rest, roots)
        }
        _ => coefficient_term(graph, coefficient.clone(), term, roots),
    }
}
fn distribute(
    graph: &mut Expressions,
    coefficient: &Rational,
    sum: NodeId,
    roots: &mut RootRegistry,
) -> ConstructResult<NodeId> {
    let Node::Add(terms) = graph.node(sum)?.clone() else {
        unreachable!("validated Add")
    };
    let mut scaled = Vec::new();
    for term in terms {
        scaled.push(keep_coefficient(graph, coefficient, term, roots)?);
    }
    add_inner(graph, &scaled, roots)
}

/// Source Mul: flatten, Rational product, integer exponents над opaque/Add bases.
/// Numeric Pow/radical gathering и нецелые exponent transformations — internalUnsupported.
pub fn mul(graph: &mut Expressions, args: &[NodeId]) -> ConstructResult<NodeId> {
    mul_inner(graph, args, &mut RootRegistry::default())
}
fn mul_inner(
    graph: &mut Expressions,
    args: &[NodeId],
    roots: &mut RootRegistry,
) -> ConstructResult<NodeId> {
    let args = nonidentity(graph, args, 1, roots.formal_tan)?;
    if args.is_empty() {
        return push_q(graph, integer(1));
    }
    if args.len() == 1 {
        return Ok(args[0]);
    }
    // SymPy two-arg hack: r*(c*Add) сохраняет raw Mul, r*Add распределяет.
    if args.len() == 2 {
        let pair = match graph.node(args[0])?.clone() {
            Node::Rational(q) => Some((q, args[1])),
            _ => match graph.node(args[1])?.clone() {
                Node::Rational(q) => Some((q, args[0])),
                _ => None,
            },
        };
        if let Some((q, other)) = pair {
            if !is(&q, 0) {
                let (r, term) = split_coefficient(graph, other)?;
                if matches!(graph.node(term)?, Node::Add(_)) {
                    let coefficient = rational_mul(&q, &r);
                    if !is(&r, 1) {
                        return if is(&coefficient, 1) {
                            Ok(term)
                        } else {
                            let q = push_q(graph, coefficient)?;
                            Ok(graph.intern(Node::Mul(vec![q, term]))?)
                        };
                    }
                    return distribute(graph, &q, term, roots);
                }
            }
        }
    }
    let mut queue: VecDeque<_> = args.into();
    let mut coefficient = integer(1);
    let mut powers: Vec<(NodeId, IBig)> = Vec::new();
    let mut root_radicand = UBig::ONE;
    while let Some(id) = queue.pop_front() {
        let (base, exponent) = match graph.node(id)?.clone() {
            Node::Mul(children) => {
                queue.extend(children);
                continue;
            }
            Node::Rational(q) => {
                coefficient = rational_mul(&coefficient, &q);
                continue;
            }
            Node::Pow(base, exponent) => {
                if let Some(n) = registered_radicand(graph, roots, id) {
                    // a,b squarefree по provenance. a/g и b/g coprime squarefree,
                    // поэтому sqrt(a)*sqrt(b)=g*sqrt((a/g)*(b/g)) без factorization.
                    let common = gcd(&root_radicand, &n);
                    root_radicand = (&root_radicand / &common) * (&n / &common);
                    coefficient =
                        rational_mul(&coefficient, &Rational::integer(IBig::from(common)));
                    continue;
                }
                if matches!(graph.node(base)?, Node::Rational(_)) {
                    return Err(refuse(UnsupportedProfile::NumericPowerInMul(id)));
                }
                let Node::Rational(e) = graph.node(exponent)? else {
                    return Err(refuse(UnsupportedProfile::NonIntegerPowerInMul(id)));
                };
                if e.parts().1 != &UBig::ONE {
                    return Err(refuse(UnsupportedProfile::NonIntegerPowerInMul(id)));
                }
                if matches!(graph.node(base)?, Node::Pow(_, _) | Node::Mul(_)) {
                    return Err(refuse(UnsupportedProfile::CompoundPowerBase(id)));
                }
                (base, e.parts().0.clone())
            }
            Node::Tan(_) if !roots.formal_tan => {
                return Err(refuse(UnsupportedProfile::NodeOutsideProfile(id)))
            }
            _ => (id, IBig::ONE),
        };
        if let Some((_, old)) = powers.iter_mut().find(|(b, _)| equal(graph, *b, base)) {
            *old += exponent;
        } else {
            powers.push((base, exponent));
        }
    }
    if is(&coefficient, 0) {
        return push_q(graph, integer(0));
    }
    let mut result = Vec::new();
    if root_radicand != UBig::ONE {
        result.push(registered_root(graph, root_radicand, roots)?);
    }
    for (base, exponent) in powers {
        if exponent.is_zero() {
            continue;
        }
        if exponent == IBig::ONE {
            result.push(base);
        } else {
            let exponent = push_q(graph, Rational::integer(exponent))?;
            result.push(graph.intern(Node::Pow(base, exponent))?);
        }
    }
    sort(graph, &mut result);
    if !is(&coefficient, 1) {
        if result.len() == 1 && matches!(graph.node(result[0])?, Node::Add(_)) {
            return distribute(graph, &coefficient, result[0], roots);
        }
        result.insert(0, push_q(graph, coefficient)?);
    }
    match result.len() {
        0 => push_q(graph, integer(1)),
        1 => Ok(result[0]),
        _ => Ok(graph.intern(Node::Mul(result))?),
    }
}
