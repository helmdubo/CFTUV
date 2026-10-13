//! Отдельный A1 numerical shell candidate; продуктовый evaluator/admission не меняется.
//! Whole AST -> invocation-local QuadraticReader -> V2 enclosure160 -> grid10^-27.
//! Origin: reference/density_shell_v2.py; Work — только V2 диагностика.
//! Source SHA256 74063bff8e7d9db2860c22f4032706c40685c375417c11e4c35c150c51e9a0d0.
use crate::density_expression::{Expressions, Node, NodeId};
use crate::density_quadratic::{QuadraticFailure, QuadraticFailureKind, QuadraticReader};
use crate::interval::RationalInterval;
use crate::{IBig, Radical, Rational, UBig, Work};
use cftuv_radical::{Failure, RefusalCode};

pub const CANON: &str = "DENSITY_SHELL_CANON_V2_CANDIDATE";
pub const SHELL_BITS: usize = 160;
pub const SHELL_QUANTUM: i32 = -27;
/// Нативный reader profile, НЕ продуктовый допуск и НЕ Python candidate cap.
pub const MAX_EXPONENT_MAGNITUDE: u64 = crate::density_quadratic::MAX_EXPONENT_MAGNITUDE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NotApplicableCode {
    UnsupportedSyntax,
    InverseClassLimit,
}
impl NotApplicableCode {
    pub fn code(self) -> &'static str {
        match self {
            Self::UnsupportedSyntax => "DENSITY_SHELL_V2_UNSUPPORTED_SYNTAX",
            Self::InverseClassLimit => "DENSITY_SHELL_V2_INVERSE_CLASS_LIMIT",
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutcomeCode {
    Applied,
    GridPoint,
    NonDyadicGridStraw,
}
impl OutcomeCode {
    pub fn code(self) -> &'static str {
        match self {
            Self::Applied => "DENSITY_DECIMAL_SHELL_V2_APPLIED",
            Self::GridPoint => "DENSITY_DECIMAL_GRID_POINT_V2",
            Self::NonDyadicGridStraw => "DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2",
        }
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RationalMetadata {
    pub numerator: IBig,
    pub denominator: UBig,
    pub grid_units: Option<IBig>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OutcomeBundle {
    pub codes: Vec<OutcomeCode>,
    pub rational: Option<RationalMetadata>,
    /// Mathematical finite-binary straw minimum, НЕ измеренная legacy width.
    pub legacy_projected_min_width_units: Option<u8>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IntegerShell {
    pub lower_units: IBig,
    pub upper_units: IBig,
    pub width_units: IBig,
    pub quantum: i32,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Candidate {
    pub canon: &'static str,
    pub value: Radical,
    pub enclosure160: RationalInterval,
    pub shell: IntegerShell,
    pub work: Work,
    pub outcomes: OutcomeBundle,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NotApplicable {
    pub canon: &'static str,
    pub code: NotApplicableCode,
    pub detail: String,
    pub work: Work,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ShellResult {
    Applied(Candidate),
    NotApplicable(NotApplicable),
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ShellError {
    Reader(QuadraticFailure),
}

fn scale() -> UBig {
    "1000000000000000000000000000".parse().unwrap()
}
/// Целочисленные units и width точны при любом размере; Decimal context отсутствует.
pub fn integer_shell(enclosure: &RationalInterval) -> IntegerShell {
    let (lower_units, upper_units) = enclosure.scale10_27_envelope();
    IntegerShell {
        width_units: &upper_units - &lower_units,
        lower_units,
        upper_units,
        quantum: SHELL_QUANTUM,
    }
}
pub(crate) fn outcome_bundle(value: &Radical) -> OutcomeBundle {
    let mut result = OutcomeBundle {
        codes: vec![OutcomeCode::Applied],
        rational: None,
        legacy_projected_min_width_units: None,
    };
    let Some((numerator, denominator)) = value.as_rational() else {
        return result;
    };
    let scale = scale();
    let grid_units = if &scale % &denominator == UBig::ZERO {
        result.codes.push(OutcomeCode::GridPoint);
        if (&denominator & (&denominator - UBig::ONE)) != UBig::ZERO {
            result.codes.push(OutcomeCode::NonDyadicGridStraw);
            result.legacy_projected_min_width_units = Some(2);
        }
        Some(&numerator * IBig::from(&scale / &denominator))
    } else {
        None
    };
    result.rational = Some(RationalMetadata {
        numerator,
        denominator,
        grid_units,
    });
    result
}
fn unsupported_detail(graph: &Expressions, id: NodeId) -> String {
    let name = match graph.node(id) {
        Ok(Node::Pow(_, _)) => {
            return "Pow requires an integer exponent or nonnegative rational base to p/2".into()
        }
        Ok(Node::Pi) => "Pi",
        Ok(Node::Sin(_)) => "sin",
        Ok(Node::Cos(_)) => "cos",
        Ok(Node::Tan(_)) => "tan",
        Ok(Node::Atan(_)) => "atan",
        Ok(Node::Atan2(_, _)) => "atan2",
        _ => "unknown",
    };
    format!("unsupported AST node: {name}")
}
/// Memo принадлежит ровно одному вызову; Expressions cached quadratic/sign не используется.
/// Exact zero, invalid reference и exponent/profile errors остаются internal Err.
pub fn density_shell_v2(graph: &Expressions, source: NodeId) -> Result<ShellResult, ShellError> {
    let (value, work) = match QuadraticReader::new(graph).read(source) {
        Ok(value) => value,
        Err(error) => {
            let (code, detail) = match &error.kind {
                QuadraticFailureKind::UnsupportedSyntax(id) => (
                    NotApplicableCode::UnsupportedSyntax,
                    unsupported_detail(graph, *id),
                ),
                QuadraticFailureKind::InternalAlgebra(Failure::Refused(refusal))
                    if refusal.code == RefusalCode::InverseClassLimit =>
                {
                    (NotApplicableCode::InverseClassLimit, refusal.detail.clone())
                }
                _ => return Err(ShellError::Reader(error)),
            };
            return Ok(ShellResult::NotApplicable(NotApplicable {
                canon: CANON,
                code,
                detail,
                work: error.work,
            }));
        }
    };
    let (((lo_n, lo_d), (hi_n, hi_d)), enclosure_work) = value.enclosure(SHELL_BITS);
    let enclosure160 = RationalInterval::new(
        Rational::new(lo_n, lo_d).expect("Radical enclosure positive denominator"),
        Rational::new(hi_n, hi_d).expect("Radical enclosure positive denominator"),
    )
    .expect("Radical enclosure ordered");
    let shell = integer_shell(&enclosure160);
    let outcomes = outcome_bundle(&value);
    Ok(ShellResult::Applied(Candidate {
        canon: CANON,
        value,
        enclosure160,
        shell,
        work: work + enclosure_work,
        outcomes,
    }))
}
