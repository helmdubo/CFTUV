//! Подготовка sealed/termination рациональных свидетелей адаптивного Density.
//! Oracle: adaptive_density_fan.py::_sealed_local_interval, _box_is_feasible,
//! _termination_boxes, _termination_height. Scalar/Expr admission — у callbacks.

use crate::atlas_plan::DecimalEnvelope;
use crate::farey::{WorkBudget, WorkBudgetExhausted};
use crate::interval::{rational_add, rational_cmp, rational_mul, rational_sub};
use crate::{IBig, Rational, UBig};

pub const BOX_REFINEMENT_CAP: usize = 96;
pub const BOX_INITIAL_DIVISOR_BITS: usize = 24;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PreparationLimits {
    pub box_refinement_cap: usize,
}
impl Default for PreparationLimits {
    fn default() -> Self {
        Self {
            box_refinement_cap: BOX_REFINEMENT_CAP,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PrepareRecord {
    pub use_x_denominator: bool,
    pub denominator_sign: i8,
    pub lower_slope_envelope: DecimalEnvelope,
    pub upper_slope_envelope: DecimalEnvelope,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FanTerminationBox {
    pub lower: Rational,
    pub upper: Rational,
    pub use_x_denominator: bool,
    pub denominator_sign: i8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrepareInvalid {
    BoundaryOverlapsIdeal,
    SealedIntervalEmpty,
    InnerEnvelopeNotAdmissible,
    IdealOutsideWindow,
    NonPositiveTerminationWidth,
}

impl PrepareInvalid {
    pub fn message(self) -> &'static str {
        match self {
            Self::BoundaryOverlapsIdeal => "subturn boundary overlaps the ideal direction",
            Self::SealedIntervalEmpty => "sealed local admissible interval is empty",
            Self::InnerEnvelopeNotAdmissible => "inner rational envelope is not admissible",
            Self::IdealOutsideWindow => "ideal is not strictly inside its ordinal window",
            Self::NonPositiveTerminationWidth => "fan termination width is not positive",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LimitDiagnostic {
    True,
    False,
    Undecided,
}

impl LimitDiagnostic {
    pub fn text(self) -> &'static str {
        match self {
            Self::True => "True",
            Self::False => "False",
            Self::Undecided => "undecided",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FanPrepareFailure<E> {
    Invalid(PrepareInvalid),
    Callback(E),
    Work(WorkBudgetExhausted),
    TerminationBoxesExhausted {
        hidden_rays: usize,
        at_limit: LimitDiagnostic,
        halvings: usize,
    },
    /// Внутренний malformed prototype input, не продуктовый UNDECIDABLE.
    MissingIdealVector {
        index: usize,
    },
}

/// Контекст metric/ideal/memo остаётся у adapter; module не создаёт Scalar дерева.
pub trait FanPrepareLaws {
    type Vector: Clone;
    type Scalar;
    type Error;
    fn subturn_boundary_vectors(
        &mut self,
        ordinal: usize,
        orientation: i8,
        q: usize,
    ) -> Result<Vec<Self::Vector>, Self::Error>;
    fn denominator(
        &mut self,
        vector: &Self::Vector,
        use_x: bool,
    ) -> Result<Self::Scalar, Self::Error>;
    fn sign(&mut self, scalar: &Self::Scalar) -> Result<i8, Self::Error>;
    fn slope(&mut self, vector: &Self::Vector, use_x: bool) -> Result<Self::Scalar, Self::Error>;
    fn decimal_envelope(&mut self, scalar: &Self::Scalar) -> Result<DecimalEnvelope, Self::Error>;
    fn vector_from_slope(
        &mut self,
        value: &Rational,
        use_x: bool,
        sign: i8,
    ) -> Result<Self::Vector, Self::Error>;
    fn candidate_vector(
        &mut self,
        value: &Rational,
        use_x: bool,
        sign: i8,
    ) -> Result<Self::Vector, Self::Error>;
    fn local_candidate(
        &mut self,
        ordinal: usize,
        vector: &Self::Vector,
        q: usize,
    ) -> Result<bool, Self::Error>;
    fn window_neighbours(
        &mut self,
        ordinal: usize,
    ) -> Result<(Self::Vector, Self::Vector), Self::Error>;
    fn inside_ordinal(
        &mut self,
        vector: &Self::Vector,
        previous: &Self::Vector,
        center: &Self::Vector,
        following: &Self::Vector,
    ) -> Result<bool, Self::Error>;
    fn oriented_cross_sign(
        &mut self,
        left: &Self::Vector,
        right: &Self::Vector,
    ) -> Result<i8, Self::Error>;
    fn subturn(
        &mut self,
        left: &Self::Vector,
        right: &Self::Vector,
        q: usize,
    ) -> Result<bool, Self::Error>;
    fn subturn_boundary(
        &mut self,
        left: &Self::Vector,
        right: &Self::Vector,
        q: usize,
    ) -> Result<bool, Self::Error>;
    /// Только исходный ReferenceGeometryError превращается в "undecided" final diagnostic.
    fn is_reference_geometry_error(error: &Self::Error) -> bool;
}

fn ideal_vector<V, E>(ideal: &[V], index: usize) -> Result<&V, FanPrepareFailure<E>> {
    ideal
        .get(index)
        .ok_or(FanPrepareFailure::MissingIdealVector { index })
}

fn half(value: &Rational) -> Rational {
    rational_mul(
        value,
        &Rational::new(IBig::ONE, UBig::from(2u8)).expect("positive denominator"),
    )
}

pub fn sealed_local_interval<L: FanPrepareLaws>(
    ideal: &[L::Vector],
    ordinal: usize,
    orientation: i8,
    q: usize,
    record: &PrepareRecord,
    laws: &mut L,
) -> Result<[Rational; 4], FanPrepareFailure<L::Error>> {
    let use_x = record.use_x_denominator;
    let boundary_vectors = laws
        .subturn_boundary_vectors(ordinal, orientation, q)
        .map_err(FanPrepareFailure::Callback)?;
    let mut boundaries = Vec::new();
    for vector in boundary_vectors {
        let denominator = laws
            .denominator(&vector, use_x)
            .map_err(FanPrepareFailure::Callback)?;
        // Исходный sealed закон пропускает только ZERO; opposite-sign boundary применяется.
        if laws
            .sign(&denominator)
            .map_err(FanPrepareFailure::Callback)?
            == 0
        {
            continue;
        }
        let slope = laws
            .slope(&vector, use_x)
            .map_err(FanPrepareFailure::Callback)?;
        boundaries.push(
            laws.decimal_envelope(&slope)
                .map_err(FanPrepareFailure::Callback)?,
        );
    }
    let slope = laws
        .slope(ideal_vector(ideal, ordinal)?, use_x)
        .map_err(FanPrepareFailure::Callback)?;
    let center_envelope = laws
        .decimal_envelope(&slope)
        .map_err(FanPrepareFailure::Callback)?;
    let center = half(&rational_add(
        &center_envelope.lower,
        &center_envelope.upper,
    ));
    let mut lower = record.lower_slope_envelope.clone();
    let mut upper = record.upper_slope_envelope.clone();
    for boundary in boundaries {
        if rational_cmp(&boundary.upper, &center).is_lt() {
            if rational_cmp(&boundary.upper, &lower.upper).is_gt() {
                lower = boundary;
            }
        } else if rational_cmp(&boundary.lower, &center).is_gt() {
            if rational_cmp(&boundary.lower, &upper.lower).is_lt() {
                upper = boundary;
            }
        } else {
            return Err(FanPrepareFailure::Invalid(
                PrepareInvalid::BoundaryOverlapsIdeal,
            ));
        }
    }
    let lower_inward = if lower.lower == lower.upper {
        half(&rational_add(&lower.upper, &center))
    } else {
        lower.upper
    };
    let upper_inward = if upper.lower == upper.upper {
        half(&rational_add(&upper.lower, &center))
    } else {
        upper.lower
    };
    let sealed = [lower.lower, lower_inward, upper_inward, upper.upper];
    if rational_cmp(&sealed[0], &sealed[1]).is_gt()
        || rational_cmp(&sealed[1], &sealed[2]).is_ge()
        || rational_cmp(&sealed[2], &sealed[3]).is_gt()
    {
        return Err(FanPrepareFailure::Invalid(
            PrepareInvalid::SealedIntervalEmpty,
        ));
    }
    for value in [&sealed[1], &sealed[2]] {
        let vector = laws
            .vector_from_slope(value, use_x, record.denominator_sign)
            .map_err(FanPrepareFailure::Callback)?;
        if !laws
            .local_candidate(ordinal, &vector, q)
            .map_err(FanPrepareFailure::Callback)?
        {
            return Err(FanPrepareFailure::Invalid(
                PrepareInvalid::InnerEnvelopeNotAdmissible,
            ));
        }
    }
    Ok(sealed)
}

pub fn box_is_feasible<L: FanPrepareLaws>(
    ideal: &[L::Vector],
    orientation: i8,
    q: usize,
    boxes: &[FanTerminationBox],
    laws: &mut L,
) -> Result<bool, FanPrepareFailure<L::Error>> {
    let mut endpoints = Vec::new();
    for (index, item) in boxes.iter().enumerate() {
        let ordinal = index + 1;
        let lower = laws
            .candidate_vector(&item.lower, item.use_x_denominator, item.denominator_sign)
            .map_err(FanPrepareFailure::Callback)?;
        let upper = laws
            .candidate_vector(&item.upper, item.use_x_denominator, item.denominator_sign)
            .map_err(FanPrepareFailure::Callback)?;
        let (previous, following) = laws
            .window_neighbours(ordinal)
            .map_err(FanPrepareFailure::Callback)?;
        let center = ideal_vector(ideal, ordinal)?;
        for vector in [&lower, &upper] {
            if !laws
                .inside_ordinal(vector, &previous, center, &following)
                .map_err(FanPrepareFailure::Callback)?
            {
                return Ok(false);
            }
        }
        endpoints.push([lower, upper]);
    }
    let mut sequences = vec![Vec::new()];
    for pair in endpoints {
        let mut following = Vec::new();
        for prefix in sequences {
            for item in &pair {
                let mut next = prefix.clone();
                next.push(item.clone());
                following.push(next);
            }
        }
        sequences = following;
    }
    for hidden in sequences {
        let mut sequence = vec![ideal_vector(ideal, 0)?.clone()];
        sequence.extend(hidden);
        sequence.push(ideal_vector(ideal, ideal.len().saturating_sub(1))?.clone());
        // В каждой sequence ВСЕ cross идут прежде первого subturn — не edge interleave.
        for pair in sequence.windows(2) {
            if laws
                .oriented_cross_sign(&pair[0], &pair[1])
                .map_err(FanPrepareFailure::Callback)?
                != orientation
            {
                return Ok(false);
            }
        }
        for pair in sequence.windows(2) {
            if !laws
                .subturn(&pair[0], &pair[1], q)
                .map_err(FanPrepareFailure::Callback)?
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Canonical caller создаёт свежий WorkBudget::preparation(); счётчики доступны после отказа.
pub fn termination_boxes<L: FanPrepareLaws>(
    ideal: &[L::Vector],
    orientation: i8,
    q: usize,
    records: &[PrepareRecord],
    budget: &mut WorkBudget,
    laws: &mut L,
) -> Result<Vec<FanTerminationBox>, FanPrepareFailure<L::Error>> {
    termination_boxes_with_limits(
        ideal,
        orientation,
        q,
        records,
        budget,
        laws,
        PreparationLimits::default(),
    )
}

pub fn termination_boxes_with_limits<L: FanPrepareLaws>(
    ideal: &[L::Vector],
    orientation: i8,
    q: usize,
    records: &[PrepareRecord],
    budget: &mut WorkBudget,
    laws: &mut L,
    limits: PreparationLimits,
) -> Result<Vec<FanTerminationBox>, FanPrepareFailure<L::Error>> {
    termination_boxes_with_limits_observed(
        ideal,
        orientation,
        q,
        records,
        budget,
        laws,
        limits,
        || {},
    )
}

/// Наблюдаемая граница source _preparation_budget(): после всех centers/gaps,
/// до divisor/loop, в том числе при нулевом box cap. Алгоритм остаётся один.
pub fn termination_boxes_with_limits_observed<L: FanPrepareLaws>(
    ideal: &[L::Vector],
    orientation: i8,
    q: usize,
    records: &[PrepareRecord],
    budget: &mut WorkBudget,
    laws: &mut L,
    limits: PreparationLimits,
    mut on_budget_created: impl FnMut(),
) -> Result<Vec<FanTerminationBox>, FanPrepareFailure<L::Error>> {
    let mut centers = Vec::new();
    let mut gaps = Vec::new();
    for (index, record) in records.iter().enumerate() {
        let slope = laws
            .slope(ideal_vector(ideal, index + 1)?, record.use_x_denominator)
            .map_err(FanPrepareFailure::Callback)?;
        let envelope = laws
            .decimal_envelope(&slope)
            .map_err(FanPrepareFailure::Callback)?;
        let center = half(&rational_add(&envelope.lower, &envelope.upper));
        let left_gap = rational_sub(&center, &record.lower_slope_envelope.upper);
        let right_gap = rational_sub(&record.upper_slope_envelope.lower, &center);
        let gap = if rational_cmp(&left_gap, &right_gap).is_le() {
            left_gap
        } else {
            right_gap
        };
        if rational_cmp(&gap, &Rational::integer(IBig::ZERO)).is_le() {
            return Err(FanPrepareFailure::Invalid(
                PrepareInvalid::IdealOutsideWindow,
            ));
        }
        centers.push(center);
        gaps.push(gap);
    }
    on_budget_created();
    let mut divisor = UBig::ONE << BOX_INITIAL_DIVISOR_BITS;
    for _ in 0..limits.box_refinement_cap {
        budget
            .spend_order_steps(UBig::ONE << centers.len())
            .map_err(FanPrepareFailure::Work)?;
        let reciprocal = Rational::new(IBig::ONE, divisor.clone()).expect("positive divisor");
        let boxes: Vec<_> = centers
            .iter()
            .zip(&gaps)
            .zip(records)
            .map(|((center, gap), record)| {
                let radius = rational_mul(gap, &reciprocal);
                FanTerminationBox {
                    lower: rational_sub(center, &radius),
                    upper: rational_add(center, &radius),
                    use_x_denominator: record.use_x_denominator,
                    denominator_sign: record.denominator_sign,
                }
            })
            .collect();
        if box_is_feasible(ideal, orientation, q, &boxes, laws)? {
            return Ok(boxes);
        }
        divisor <<= 1usize;
    }
    let at_limit = match laws.subturn_boundary(ideal_vector(ideal, 0)?, ideal_vector(ideal, 1)?, q)
    {
        Ok(true) => LimitDiagnostic::True,
        Ok(false) => LimitDiagnostic::False,
        Err(error) if L::is_reference_geometry_error(&error) => LimitDiagnostic::Undecided,
        Err(error) => return Err(FanPrepareFailure::Callback(error)),
    };
    Err(FanPrepareFailure::TerminationBoxesExhausted {
        hidden_rays: centers.len(),
        at_limit,
        halvings: limits.box_refinement_cap,
    })
}

pub fn termination_height(boxes: &[FanTerminationBox]) -> Result<UBig, PrepareInvalid> {
    let mut maximum = UBig::ZERO;
    for item in boxes {
        let width = rational_sub(&item.upper, &item.lower);
        if width.parts().0 <= &IBig::ZERO {
            return Err(PrepareInvalid::NonPositiveTerminationWidth);
        }
        let (n, d) = width.parts();
        let n = cftuv_radical::arith::magnitude(n);
        let bound = d / n + UBig::from(2u8);
        maximum = maximum.max(bound);
    }
    Ok(maximum)
}
