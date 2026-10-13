//! Controller подготовленной adaptive Density authority без authority ID/PyO3.
//! Oracle: adaptive_density_fan.py::_certify_adaptive_density_fan_prepared.
//! Подготовка/Atlas/Farey/order не копируются; scalar/geometry admission у adapter.

use crate::atlas_plan::{
    self, AtlasPlanFailure, BoolField, ChartPiece, LocalRecord, NumericField, SegmentPlanLaws,
    TerminationBox,
};
use crate::atlas_search;
use crate::fan_prepare::{
    self, FanPrepareFailure, FanPrepareLaws, FanTerminationBox, PreparationLimits, PrepareInvalid,
};
use crate::farey::{
    self, FareyFailure, FareyPredicates, PREPARATION_WORK_CAP, PrimitiveVector, SEARCH_WORK_CAP,
    SearchMode, SingleChartWindow, WorkBudget,
};
use crate::interval::rational_sub;
use crate::order::CanonicalOrdering;
use crate::window_build::PreparedWindowRecord;
use crate::{IBig, Rational, UBig};

pub trait FanCompileLaws:
    FanPrepareLaws
    + SegmentPlanLaws<
        Vector = <Self as FanPrepareLaws>::Vector,
        Scalar = <Self as FanPrepareLaws>::Scalar,
        Error = <Self as FanPrepareLaws>::Error,
    > + FareyPredicates<Error = <Self as FanPrepareLaws>::Error>
{
}
impl<T> FanCompileLaws for T where
    T: FanPrepareLaws
        + SegmentPlanLaws<
            Vector = <T as FanPrepareLaws>::Vector,
            Scalar = <T as FanPrepareLaws>::Scalar,
            Error = <T as FanPrepareLaws>::Error,
        > + FareyPredicates<Error = <T as FanPrepareLaws>::Error>
{
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompileLimits {
    pub preparation_work_cap: UBig,
    pub search_work_cap: UBig,
    pub box_refinement_cap: usize,
    pub resource_height_cap: UBig,
}
impl Default for CompileLimits {
    fn default() -> Self {
        Self {
            preparation_work_cap: UBig::from(PREPARATION_WORK_CAP),
            search_work_cap: UBig::from(SEARCH_WORK_CAP),
            box_refinement_cap: fan_prepare::BOX_REFINEMENT_CAP,
            resource_height_cap: UBig::from(1_000_000u32),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuthorityWindowBounds {
    pub ordinal: usize,
    pub termination_lower_slope: Rational,
    pub termination_upper_slope: Rational,
    pub certified_termination_width: Rational,
    pub admissible: [Rational; 4],
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PreparedAuthorityWindow {
    SingleChart {
        bounds: AuthorityWindowBounds,
        record: fan_prepare::PrepareRecord,
    },
    Atlas {
        bounds: AuthorityWindowBounds,
        pieces: Vec<ChartPiece>,
        termination_piece_index: usize,
    },
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreparedFanAuthority {
    pub max_subturn_q: usize,
    pub minimal_common_height: UBig,
    pub exhaustive_previous_height: UBig,
    pub termination_height_upper_bound: UBig,
    pub bound_primitive_integer_vectors: Vec<PrimitiveVector>,
    pub ordinal_windows: Vec<PreparedAuthorityWindow>,
    pub previous_counts: Vec<usize>,
    pub counts: Vec<usize>,
}
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CompileBudgets {
    pub preparation: Option<WorkBudget>,
    pub search: Option<WorkBudget>,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FanCompileFailure<E> {
    Preparation(FanPrepareFailure<E>),
    Height(PrepareInvalid),
    Segments(AtlasPlanFailure<E>),
    Search(FareyFailure<E>),
    /// Source fan is None: то же имя отказа, без budget suffix.
    RationalAuthorityExhausted,
    /// Не product malformed/UNDECIDABLE: source returns height вместе с fan.
    MissingSearchHeight,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CompileReport<E> {
    pub result: Result<PreparedFanAuthority, FanCompileFailure<E>>,
    pub budgets: CompileBudgets,
}

fn plan_record(record: &fan_prepare::PrepareRecord) -> LocalRecord {
    LocalRecord {
        use_x_denominator: BoolField::Bool(record.use_x_denominator),
        denominator_sign: NumericField::Int(IBig::from(record.denominator_sign)),
        lower_slope_envelope: record.lower_slope_envelope.clone(),
        upper_slope_envelope: record.upper_slope_envelope.clone(),
    }
}
fn global_height(boxes: &[FanTerminationBox]) -> Result<UBig, PrepareInvalid> {
    let plan: Vec<_> = boxes
        .iter()
        .map(|b| TerminationBox {
            lower: b.lower.clone(),
            upper: b.upper.clone(),
            use_x_denominator: BoolField::Bool(b.use_x_denominator),
            denominator_sign: NumericField::Int(IBig::from(b.denominator_sign)),
        })
        .collect();
    let value = atlas_plan::global_termination_height(&plan, |b| {
        fan_prepare::termination_height(&[FanTerminationBox {
            lower: b.lower.clone(),
            upper: b.upper.clone(),
            use_x_denominator: true,
            denominator_sign: 1,
        }])
        .map(|h| cftuv_radical::arith::signed(&h))
    })?;
    Ok(cftuv_radical::arith::magnitude(&value))
}
fn authority_windows(
    records: &[PreparedWindowRecord],
    boxes: &[FanTerminationBox],
    sealed: &[[Rational; 4]],
) -> Vec<PreparedAuthorityWindow> {
    // Все три длины получены из records теми же операциями, не внешним zip input.
    records
        .iter()
        .zip(boxes)
        .zip(sealed)
        .enumerate()
        .map(|(i, ((record, b), s))| {
            let bounds = AuthorityWindowBounds {
                ordinal: i + 1,
                termination_lower_slope: b.lower.clone(),
                termination_upper_slope: b.upper.clone(),
                certified_termination_width: rational_sub(&b.upper, &b.lower),
                admissible: s.clone(),
            };
            match record {
                PreparedWindowRecord::SingleChart(record) => PreparedAuthorityWindow::SingleChart {
                    bounds,
                    record: record.clone(),
                },
                PreparedWindowRecord::Atlas(a) => PreparedAuthorityWindow::Atlas {
                    bounds,
                    pieces: a.pieces.clone(),
                    termination_piece_index: a.termination_piece_index,
                },
            }
        })
        .collect()
}

pub fn compile_prepared<L: FanCompileLaws>(
    ideal: &[<L as FanPrepareLaws>::Vector],
    orientation: i8,
    q: usize,
    records: &[PreparedWindowRecord],
    limits: &CompileLimits,
    laws: &mut L,
) -> CompileReport<<L as FanPrepareLaws>::Error> {
    let mut budgets = CompileBudgets::default();
    let result = compile_inner(ideal, orientation, q, records, limits, laws, &mut budgets);
    CompileReport { result, budgets }
}

fn compile_inner<L: FanCompileLaws>(
    ideal: &[<L as FanPrepareLaws>::Vector],
    orientation: i8,
    q: usize,
    records: &[PreparedWindowRecord],
    limits: &CompileLimits,
    laws: &mut L,
    budgets: &mut CompileBudgets,
) -> Result<PreparedFanAuthority, FanCompileFailure<<L as FanPrepareLaws>::Error>> {
    let local: Vec<_> = records.iter().map(|r| r.local_record().clone()).collect();
    let uses_atlas = records
        .iter()
        .any(|r| matches!(r, PreparedWindowRecord::Atlas(_)));
    let mut prep = WorkBudget::new(limits.preparation_work_cap.clone());
    let mut prep_created = false;
    let boxes = fan_prepare::termination_boxes_with_limits_observed(
        ideal,
        orientation,
        q,
        &local,
        &mut prep,
        laws,
        PreparationLimits {
            box_refinement_cap: limits.box_refinement_cap,
        },
        || prep_created = true,
    );
    budgets.preparation = prep_created.then_some(prep);
    let boxes = boxes.map_err(FanCompileFailure::Preparation)?;
    let sealed = local
        .iter()
        .enumerate()
        .map(|(i, r)| fan_prepare::sealed_local_interval(ideal, i + 1, orientation, q, r, laws))
        .collect::<Result<Vec<_>, _>>()
        .map_err(FanCompileFailure::Preparation)?;
    let termination_height = if uses_atlas {
        global_height(&boxes)
    } else {
        fan_prepare::termination_height(&boxes)
    }
    .map_err(FanCompileFailure::Height)?;
    let stop_height = termination_height
        .clone()
        .min(limits.resource_height_cap.clone());
    let result = if uses_atlas {
        let windows: Vec<_> = records
            .iter()
            .enumerate()
            .map(|(i, r)| r.plan_window(i + 1))
            .collect();
        let plan_records: Vec<_> = local.iter().map(plan_record).collect();
        // Source keyword segments=... вычисляется ДО keyword budget=_work_budget().
        let segments =
            atlas_plan::search_segments(orientation, q, &windows, &plan_records, &sealed, laws)
                .map_err(FanCompileFailure::Segments)?;
        let search = budgets
            .search
            .insert(WorkBudget::new(limits.search_work_cap.clone()));
        atlas_search::search_global_height(
            &segments,
            &stop_height,
            search,
            laws,
            &mut CanonicalOrdering,
        )
    } else {
        let windows: Vec<_> = local
            .iter()
            .zip(&sealed)
            .map(|(r, s)| SingleChartWindow {
                use_x: r.use_x_denominator,
                denominator_sign: r.denominator_sign,
                sealed: s.clone(),
            })
            .collect();
        let search = budgets
            .search
            .insert(WorkBudget::new(limits.search_work_cap.clone()));
        farey::search(
            SearchMode::SingleChart,
            &windows,
            &stop_height,
            search,
            laws,
            &mut CanonicalOrdering,
        )
    }
    .map_err(FanCompileFailure::Search)?;
    let fan = result
        .fan
        .ok_or(FanCompileFailure::RationalAuthorityExhausted)?;
    let height = result
        .height
        .ok_or(FanCompileFailure::MissingSearchHeight)?;
    let windows = authority_windows(records, &boxes, &sealed);
    Ok(PreparedFanAuthority {
        max_subturn_q: q,
        exhaustive_previous_height: &height - UBig::ONE,
        minimal_common_height: height,
        termination_height_upper_bound: termination_height,
        bound_primitive_integer_vectors: fan,
        ordinal_windows: windows,
        previous_counts: result.previous_counts,
        counts: result.counts,
    })
}
