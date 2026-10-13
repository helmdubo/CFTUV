//! Host-owned successful-attempt memo: ключ только для equality, не источник геометрии.
//! Закрытые receipts переживают arena; цены не сохраняются и не воспроизводятся.
use crate::density_bind_v1::OwnedBindingResultV1;
use crate::density_expression::{ExpressionResult, Expressions, Node, NodeId};
use crate::density_raw_result_v1::DensityRawBatchReportV1;
use crate::density_raw_types_v1::DensityOracleCanonV1;
use crate::density_shell_v2::OutcomeBundle;
use crate::geometry::WindowLaw;
use crate::{IBig, Rational};
use std::hash::{Hash, Hasher};

/// Literal ordered forest без arena handles и без simplification по значению.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum DensityLiteralTreeV1 {
    Rational(Rational),
    Pi,
    Add(Vec<Self>),
    Mul(Vec<Self>),
    Pow(Box<Self>, Box<Self>),
    Sin(Box<Self>),
    Cos(Box<Self>),
    Atan(Box<Self>),
    Atan2(Box<Self>, Box<Self>),
    Tan(Box<Self>),
}
impl DensityLiteralTreeV1 {
    /// Только export; обратного import в arithmetic arena у memo нет.
    pub fn from_expression(graph: &Expressions, id: NodeId) -> ExpressionResult<Self> {
        let child = |id| Self::from_expression(graph, id).map(Box::new);
        Ok(match graph.node(id)? {
            Node::Rational(r) => Self::Rational(r.clone()),
            Node::Pi => Self::Pi,
            Node::Add(args) | Node::Mul(args) => {
                let trees = args
                    .iter()
                    .map(|&id| Self::from_expression(graph, id))
                    .collect::<ExpressionResult<Vec<_>>>()?;
                if matches!(graph.node(id)?, Node::Add(_)) {
                    Self::Add(trees)
                } else {
                    Self::Mul(trees)
                }
            }
            Node::Pow(a, b) => Self::Pow(child(*a)?, child(*b)?),
            Node::Sin(a) => Self::Sin(child(*a)?),
            Node::Cos(a) => Self::Cos(child(*a)?),
            Node::Atan(a) => Self::Atan(child(*a)?),
            Node::Atan2(a, b) => Self::Atan2(child(*a)?, child(*b)?),
            Node::Tan(a) => Self::Tan(child(*a)?),
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DensityAttemptKeyV1 {
    pub canon_ids: DensityOracleCanonV1,
    pub gram: [Rational; 3],
    pub inverse_gram: [Rational; 3],
    pub owner_orientation_sign: i8,
    pub ideal_primal_literals: Vec<[DensityLiteralTreeV1; 2]>,
    pub orientation: i8,
    pub q: usize,
    pub binding_reasons: Vec<Option<String>>,
    pub adaptive_only: bool,
    pub window_law: WindowLaw,
}
impl Hash for DensityAttemptKeyV1 {
    fn hash<H: Hasher>(&self, h: &mut H) {
        self.canon_ids.hash(h);
        self.gram.hash(h);
        self.inverse_gram.hash(h);
        self.owner_orientation_sign.hash(h);
        self.ideal_primal_literals.hash(h);
        self.orientation.hash(h);
        self.q.hash(h);
        self.binding_reasons.hash(h);
        self.adaptive_only.hash(h);
        std::mem::discriminant(&self.window_law).hash(h);
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DensityAttemptReceiptV1 {
    pub key: DensityAttemptKeyV1,
    pub binding_certificates: Vec<Option<crate::legacy_binding::LegacyCertificate>>,
    pub authority: Option<crate::fan_compile::PreparedFanAuthority>,
    pub shell_events: Vec<(Vec<IBig>, OutcomeBundle)>,
}
impl DensityAttemptReceiptV1 {
    pub fn binding_result(&self) -> OwnedBindingResultV1 {
        OwnedBindingResultV1 {
            certificates: self.binding_certificates.clone(),
            authority: self.authority.clone(),
        }
    }
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DensityAttemptMemoSnapshotV1 {
    pub entries: Vec<DensityAttemptReceiptV1>,
}
#[derive(Clone, Debug)]
pub struct DensityMemoBatchResultV1 {
    pub report: DensityRawBatchReportV1,
    pub delta: DensityAttemptMemoSnapshotV1,
}
