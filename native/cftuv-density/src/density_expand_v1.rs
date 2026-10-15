//! Конечный owned expand: Numeric SUM terms открываются только при distribution.
//! Negative opaque power остаётся partial атомом; positive half имеет только pair/even law.
//! Lazy sign использует прежний probe/controller; новые profile gaps не product outcomes.
use crate::density_construct_v1::{
    ConstructErrorV1, ConstructorV1, OwnedExprV1, OwnedFormV1, OwnedSignErrorV1,
};
pub use crate::density_expand::ExpandLimits;
use crate::density_expression::NodeId;
use crate::density_sign::{SignResult, SourceSignProbe};
use crate::scalar_v2::ScalarV2;
use crate::{IBig, Radical, Rational, UBig};
use cftuv_radical::arith::magnitude;
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExpandProfileV1 {
    TermLimit { requested: usize, maximum: usize },
    CountOverflow,
    PolynomialExponentLimit { requested: i64, maximum: u32 },
    RationalPowerBits { requested: usize, maximum: usize },
    OpaqueChildChanges,
}
impl ExpandProfileV1 {
    pub fn code(&self) -> &'static str {
        match self {
            Self::TermLimit { .. }
            | Self::CountOverflow
            | Self::PolynomialExponentLimit { .. }
            | Self::RationalPowerBits { .. } => "DENSITY_CONSTRUCT_V1_EXPANSION_LIMIT",
            Self::OpaqueChildChanges => "DENSITY_CONSTRUCT_V1_OPAQUE_CHILD_CHANGES",
        }
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExpandErrorV1 {
    Profile(ExpandProfileV1),
    Construction(ConstructErrorV1),
}
impl From<ConstructErrorV1> for ExpandErrorV1 {
    fn from(value: ConstructErrorV1) -> Self {
        Self::Construction(value)
    }
}
pub type ExpandResultV1<T> = Result<T, ExpandErrorV1>;
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ExpandStatsV1 {
    pub visited: usize,
    pub generated_products: usize,
}
#[derive(Clone, Debug)]
pub struct ExpandedV1 {
    pub expression: OwnedExprV1,
    pub stats: ExpandStatsV1,
}
pub fn expand(
    c: &mut ConstructorV1<'_>,
    source: &OwnedExprV1,
    limits: ExpandLimits,
) -> ExpandResultV1<ExpandedV1> {
    c.raw_id(source)?;
    let mut worker = Worker {
        c,
        limits,
        memo: HashMap::new(),
        stats: ExpandStatsV1::default(),
    };
    let expression = worker.expanded(source)?;
    Ok(ExpandedV1 {
        expression,
        stats: worker.stats,
    })
}
struct Worker<'a, 'g> {
    c: &'a mut ConstructorV1<'g>,
    limits: ExpandLimits,
    memo: HashMap<(NodeId, bool), Vec<OwnedExprV1>>,
    stats: ExpandStatsV1,
}
impl Worker<'_, '_> {
    fn count(&self, requested: usize) -> ExpandResultV1<()> {
        if requested > self.limits.max_terms {
            Err(ExpandErrorV1::Profile(ExpandProfileV1::TermLimit {
                requested,
                maximum: self.limits.max_terms,
            }))
        } else {
            Ok(())
        }
    }
    fn child_unchanged(&mut self, child: &OwnedExprV1) -> ExpandResultV1<()> {
        let expanded = self.expanded(child)?;
        if !self.c.same_literal(child, &expanded)? {
            return Err(ExpandErrorV1::Profile(ExpandProfileV1::OpaqueChildChanges));
        }
        Ok(())
    }
    fn expanded(&mut self, value: &OwnedExprV1) -> ExpandResultV1<OwnedExprV1> {
        let terms = self.choices(value, false)?;
        Ok(self.c.add(&terms)?)
    }
    fn choices(
        &mut self,
        value: &OwnedExprV1,
        multiplicative: bool,
    ) -> ExpandResultV1<Vec<OwnedExprV1>> {
        let key = (self.c.raw_id(value)?, multiplicative);
        if let Some(result) = self.memo.get(&key) {
            return Ok(result.clone());
        }
        self.stats.visited = self
            .stats
            .visited
            .checked_add(1)
            .ok_or(ExpandErrorV1::Profile(ExpandProfileV1::CountOverflow))?;
        let choices = match self.c.form(value)? {
            OwnedFormV1::Numeric(scalar) if multiplicative && scalar.value().terms().len() > 1 => {
                self.count(scalar.value().terms().len())?;
                let mut values = Vec::new();
                for term in scalar.value().terms() {
                    // Подмножество actual canonical terms остаётся canonical; не source sqrt registry.
                    values.push(self.c.numeric(ScalarV2::from_value(
                        Radical::from_canonical_terms(vec![term.clone()]),
                    ))?);
                }
                values
            }
            OwnedFormV1::Numeric(_) => vec![value.clone()],
            OwnedFormV1::Add(children) => {
                let mut terms = Vec::new();
                for child in children {
                    let children = self.choices(&child, false)?;
                    let requested = terms
                        .len()
                        .checked_add(children.len())
                        .ok_or(ExpandErrorV1::Profile(ExpandProfileV1::CountOverflow))?;
                    self.count(requested)?;
                    terms.extend(children);
                }
                terms
            }
            OwnedFormV1::Mul(children) => {
                let mut terms = vec![self.c.rational(&Rational::integer(IBig::ONE))?];
                for child in children {
                    let choices = self.choices(&child, true)?;
                    terms = self.product(&terms, &choices)?;
                }
                terms
            }
            OwnedFormV1::IntegerPow { base, exponent } if exponent >= 0 => {
                if exponent == 0 {
                    vec![self.c.integer_power(&base, 0)?]
                } else if exponent == 1 {
                    self.choices(&base, false)?
                } else {
                    if exponent > i64::from(self.limits.max_polynomial_exponent) {
                        return Err(ExpandErrorV1::Profile(
                            ExpandProfileV1::PolynomialExponentLimit {
                                requested: exponent,
                                maximum: self.limits.max_polynomial_exponent,
                            },
                        ));
                    }
                    let choices = self.choices(&base, true)?;
                    let mut terms = vec![self.c.rational(&Rational::integer(IBig::ONE))?];
                    for _ in 0..exponent {
                        terms = self.product(&terms, &choices)?;
                    }
                    terms
                }
            }
            OwnedFormV1::IntegerPow { base, .. }
            | OwnedFormV1::OpaquePow { base, .. }
            | OwnedFormV1::Sin(base)
            | OwnedFormV1::Cos(base) => {
                self.child_unchanged(&base)?;
                vec![value.clone()]
            }
            OwnedFormV1::Atan2(y, x) => {
                self.child_unchanged(&y)?;
                self.child_unchanged(&x)?;
                vec![value.clone()]
            }
            OwnedFormV1::Pi => vec![value.clone()],
        };
        self.count(choices.len())?;
        self.memo.insert(key, choices.clone());
        Ok(choices)
    }
    fn bits_guard(&self, value: &OwnedExprV1) -> ExpandResultV1<()> {
        if let Some((n, d)) = self
            .c
            .numeric_value(value)?
            .and_then(|v| v.value().as_rational())
        {
            let bits = |value: &UBig| -> ExpandResultV1<usize> {
                let words = value.as_words();
                let Some(last) = words.last() else {
                    return Ok(0);
                };
                let size = std::mem::size_of_val(last) * 8;
                (words.len() - 1)
                    .checked_mul(size)
                    .and_then(|v| v.checked_add(size - last.leading_zeros() as usize))
                    .ok_or(ExpandErrorV1::Profile(ExpandProfileV1::CountOverflow))
            };
            let requested = bits(&magnitude(&n))?.max(bits(&d)?);
            if requested > self.limits.max_rational_power_bits {
                return Err(ExpandErrorV1::Profile(ExpandProfileV1::RationalPowerBits {
                    requested,
                    maximum: self.limits.max_rational_power_bits,
                }));
            }
        }
        Ok(())
    }
    fn product(
        &mut self,
        left: &[OwnedExprV1],
        right: &[OwnedExprV1],
    ) -> ExpandResultV1<Vec<OwnedExprV1>> {
        let requested = left
            .len()
            .checked_mul(right.len())
            .ok_or(ExpandErrorV1::Profile(ExpandProfileV1::CountOverflow))?;
        self.count(requested)?;
        let mut products = Vec::with_capacity(requested);
        for a in left {
            for b in right {
                self.stats.generated_products = self
                    .stats
                    .generated_products
                    .checked_add(1)
                    .ok_or(ExpandErrorV1::Profile(ExpandProfileV1::CountOverflow))?;
                let product = self.c.mul(&[a.clone(), b.clone()])?;
                self.bits_guard(&product)?;
                products.push(product);
            }
        }
        Ok(products)
    }
}

#[derive(Clone, Debug)]
pub struct LazySignV1 {
    pub result: SignResult,
    pub expanded: Option<ExpandedV1>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LazySignErrorV1 {
    Sign(OwnedSignErrorV1),
    Expand(ExpandErrorV1),
}
/// Никакой eager expand: successful legacy probe не читает ни limits, ни fallback AST.
pub fn lazy_sign(
    c: &mut ConstructorV1<'_>,
    source: &OwnedExprV1,
    limits: ExpandLimits,
) -> Result<LazySignV1, LazySignErrorV1> {
    let id = c
        .raw_id(source)
        .map_err(|e| LazySignErrorV1::Sign(OwnedSignErrorV1::Construction(e)))?;
    let rational = matches!(
        c.graph().node(id).expect("validated owned id"),
        crate::density_expression::Node::Rational(_)
    );
    if !rational {
        if let Some(sign) = c
            .remembered_source_sign(source)
            .map_err(|e| LazySignErrorV1::Sign(OwnedSignErrorV1::Construction(e)))?
        {
            return Ok(LazySignV1 {
                result: SignResult {
                    sign,
                    path: crate::density_sign::SignPath::Memo,
                    work: crate::Work::NONE,
                },
                expanded: None,
            });
        }
    }
    match c.probe_sign(source).map_err(LazySignErrorV1::Sign)? {
        SourceSignProbe::Decided(result) => {
            c.remember_source_sign(source, result)
                .map_err(|e| LazySignErrorV1::Sign(OwnedSignErrorV1::Construction(e)))?;
            Ok(LazySignV1 {
                result,
                expanded: None,
            })
        }
        SourceSignProbe::NeedsExpanded => {
            let expanded = expand(c, source, limits).map_err(LazySignErrorV1::Expand)?;
            let result = c
                .sign_with_expanded_fallback(source, &expanded.expression)
                .map_err(LazySignErrorV1::Sign)?;
            c.remember_source_sign(source, result)
                .map_err(|e| LazySignErrorV1::Sign(OwnedSignErrorV1::Construction(e)))?;
            Ok(LazySignV1 {
                result,
                expanded: Some(expanded),
            })
        }
    }
}
