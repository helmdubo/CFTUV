//! Подготовка Atlas segment plan, без построения окон/sealed/termination boxes.
//! Oracle: adaptive_density_atlas.py::termination_piece .. search_segments.
//! Все геометрические решения принадлежат callbacks; work budget здесь отсутствует.

use crate::atlas_search::{GlobalHeightSegment, cell_segments};
use crate::farey::PrimitiveVector;
use crate::interval::{rational_add, rational_cmp, rational_mul, rational_neg, rational_sign};
use crate::{IBig, Rational, UBig};

/// Точный числовой descriptor. OtherNumber сохраняет равенство чисел, но не type(int).
/// NonNumeric означает только отрицательный результат numeric equality/membership.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NumericField {
    Int(IBig),
    OtherNumber(Rational),
    NonNumeric,
}

impl NumericField {
    pub fn equals_integer(&self, value: &IBig) -> bool {
        match self {
            Self::Int(number) => number == value,
            Self::OtherNumber(number) => number == &Rational::integer(value.clone()),
            Self::NonNumeric => false,
        }
    }

    fn unit_sign(&self) -> Option<i8> {
        [-1, 1]
            .into_iter()
            .find(|&value| self.equals_integer(&IBig::from(value)))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoolField {
    Bool(bool),
    /// Готовый результат truthiness для прямых helpers, без интерпретатора объектов.
    OtherTruthiness(bool),
}

impl BoolField {
    pub fn truth(&self) -> bool {
        match self {
            Self::Bool(value) | Self::OtherTruthiness(value) => *value,
        }
    }

    fn is_bool(&self) -> bool {
        matches!(self, Self::Bool(_))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EndpointKind {
    Open,
    Closed,
    Other,
}

/// Печатные разряды сохраняются отдельно от рационального значения.
/// Это происхождение Decimal, а не вывод из совпадения его значения с целым.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecimalQuantum {
    pub lower: i64,
    pub upper: i64,
    pub absolute_error_bound: i64,
}
impl DecimalQuantum {
    pub const fn uniform(exponent: i64) -> Self {
        Self { lower: exponent, upper: exponent, absolute_error_bound: exponent }
    }
}

/// Decimal endpoints представлены точными rational. Их создание/finite guards — вне
/// этого модуля; atlas validator не проверяет kind/error и не вводит новые guards.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecimalEnvelope {
    pub lower: Rational,
    pub upper: Rational,
    pub lower_kind: EndpointKind,
    pub upper_kind: EndpointKind,
    pub absolute_error_bound: Rational,
    /// None допустимо в старых тестовых проводах, где quantum не записывался.
    /// Продуктовый formatter не вправе угадывать его из rational endpoints.
    pub quantum: Option<DecimalQuantum>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoleOwnership {
    None,
    XZero,
    YZero,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoleField {
    Enum(PoleOwnership),
    Other,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChartPiece {
    pub is_piece_v1: bool,
    pub piece_index: NumericField,
    pub use_x_denominator: BoolField,
    pub denominator_sign: NumericField,
    pub lower_slope_envelope: DecimalEnvelope,
    pub upper_slope_envelope: DecimalEnvelope,
    pub lower_endpoint_included: BoolField,
    pub upper_endpoint_included: BoolField,
    pub slope_increases_in_ordinal_order: BoolField,
    pub pole_ownership: PoleField,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindowRecordKind {
    AtlasV1,
    Prepared,
    Other,
}

/// Метаданные ratio не читаются переносимыми операциями; тип/содержимое сохранены.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RatioField {
    Pair(NumericField, NumericField),
    Other,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AtlasWindow {
    pub kind: WindowRecordKind,
    pub ordinal: NumericField,
    pub pieces: Vec<ChartPiece>,
    pub termination_piece_index: NumericField,
    pub termination_lower_slope: RatioField,
    pub termination_upper_slope: RatioField,
    pub certified_termination_width: RatioField,
    pub admissible_lower_outward: RatioField,
    pub admissible_lower_inward: RatioField,
    pub admissible_upper_inward: RatioField,
    pub admissible_upper_outward: RatioField,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Window {
    /// getattr(window, "pieces", None) is None: этот путь не читает atlas fields.
    SingleChart,
    Atlas(AtlasWindow),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LocalRecord {
    pub use_x_denominator: BoolField,
    pub denominator_sign: NumericField,
    pub lower_slope_envelope: DecimalEnvelope,
    pub upper_slope_envelope: DecimalEnvelope,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TerminationBox {
    pub lower: Rational,
    pub upper: Rational,
    pub use_x_denominator: BoolField,
    pub denominator_sign: NumericField,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AtlasInvalid {
    Identity,
    Cardinality,
    Piece,
    PoleOwnership,
    Order,
    OuterBoundaryClosed,
    GapOrOverlap,
    PoleBoundaryOwnership,
    ChartUnrepresentable,
}

impl AtlasInvalid {
    pub fn message(&self) -> &'static str {
        match self {
            Self::Identity => "adaptive fan atlas identity invalid",
            Self::Cardinality => "adaptive fan atlas cardinality invalid",
            Self::Piece => "adaptive fan atlas piece invalid",
            Self::PoleOwnership => "adaptive fan atlas pole ownership invalid",
            Self::Order => "adaptive fan atlas order invalid",
            Self::OuterBoundaryClosed => "adaptive fan atlas outer boundary closed",
            Self::GapOrOverlap => "adaptive fan atlas gap or overlap",
            Self::PoleBoundaryOwnership => "adaptive fan atlas pole boundary ownership invalid",
            Self::ChartUnrepresentable => "DENSITY_WINDOW_CHART_UNREPRESENTABLE",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZipRelation {
    Shorter,
    Longer,
}

/// Не продуктовый отказ: ошибки внутреннего prototype domain/строгого zip отделены.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AtlasPlanFailure<E> {
    Invalid(AtlasInvalid),
    Callback(E),
    /// cell_segments сохраняет сырой truthy объект в chart; bool segment domain
    /// не представляет такую запись. Это internal prototype error, не validator.
    UnsupportedChartDescriptor,
    UnsupportedDenominatorDescriptor,
    StrictZip {
        argument: usize,
        relation: ZipRelation,
    },
}

fn termination_index(item: &AtlasWindow) -> Result<usize, AtlasInvalid> {
    let NumericField::Int(index) = &item.termination_piece_index else {
        return Err(AtlasInvalid::Identity);
    };
    if index < &IBig::ZERO || index >= &IBig::from(item.pieces.len()) {
        return Err(AtlasInvalid::Identity);
    }
    usize::try_from(index.clone()).map_err(|_| AtlasInvalid::Identity)
}

pub fn termination_piece(item: &AtlasWindow) -> Result<&ChartPiece, AtlasInvalid> {
    Ok(&item.pieces[termination_index(item)?])
}

pub fn piece_ordered_endpoints(piece: &ChartPiece) -> (&DecimalEnvelope, &DecimalEnvelope) {
    if piece.slope_increases_in_ordinal_order.truth() {
        (&piece.lower_slope_envelope, &piece.upper_slope_envelope)
    } else {
        (&piece.upper_slope_envelope, &piece.lower_slope_envelope)
    }
}

pub fn piece_cell_index(piece: &ChartPiece) -> usize {
    let positive = piece.denominator_sign.equals_integer(&IBig::ONE);
    match (piece.use_x_denominator.truth(), positive) {
        (true, true) => 0,
        (true, false) => 2,
        (false, true) => 1,
        (false, false) => 3,
    }
}

pub fn validate_atlas_structure(item: &AtlasWindow) -> Result<(), AtlasInvalid> {
    if item.kind != WindowRecordKind::AtlasV1 || !(2..=4).contains(&item.pieces.len()) {
        return Err(AtlasInvalid::Cardinality);
    }
    let mut cells = Vec::new();
    for (index, piece) in item.pieces.iter().enumerate() {
        if !piece.is_piece_v1
            || !piece.piece_index.equals_integer(&IBig::from(index))
            || !piece.use_x_denominator.is_bool()
            || piece.denominator_sign.unit_sign().is_none()
            || !piece.lower_endpoint_included.is_bool()
            || !piece.upper_endpoint_included.is_bool()
            || !piece.slope_increases_in_ordinal_order.is_bool()
            || !matches!(piece.pole_ownership, PoleField::Enum(_))
            || rational_cmp(
                &piece.lower_slope_envelope.upper,
                &piece.upper_slope_envelope.lower,
            )
            .is_ge()
        {
            return Err(AtlasInvalid::Piece);
        }
        let expected = if rational_sign(&piece.lower_slope_envelope.upper) < 0
            && rational_sign(&piece.upper_slope_envelope.lower) > 0
        {
            if piece.use_x_denominator.truth() {
                PoleOwnership::YZero
            } else {
                PoleOwnership::XZero
            }
        } else {
            PoleOwnership::None
        };
        if piece.pole_ownership != PoleField::Enum(expected) {
            return Err(AtlasInvalid::PoleOwnership);
        }
        cells.push(piece_cell_index(piece));
    }
    let steps: Vec<_> = cells
        .windows(2)
        .map(|pair| (pair[1] + 4 - pair[0]) % 4)
        .collect();
    if steps.is_empty()
        || steps
            .iter()
            .any(|&step| step != steps[0] || !matches!(step, 1 | 3))
    {
        return Err(AtlasInvalid::Order);
    }
    let first = &item.pieces[0];
    let last = item.pieces.last().expect("cardinality checked");
    let first_included = if first.slope_increases_in_ordinal_order.truth() {
        first.lower_endpoint_included.truth()
    } else {
        first.upper_endpoint_included.truth()
    };
    let last_included = if last.slope_increases_in_ordinal_order.truth() {
        last.upper_endpoint_included.truth()
    } else {
        last.lower_endpoint_included.truth()
    };
    if first_included || last_included {
        return Err(AtlasInvalid::OuterBoundaryClosed);
    }
    for pair in item.pieces.windows(2) {
        let (left, right) = (&pair[0], &pair[1]);
        let (_, end) = piece_ordered_endpoints(left);
        let (start, _) = piece_ordered_endpoints(right);
        if end.lower != end.upper
            || start.lower != start.upper
            || end.lower != start.lower
            || (end.lower != Rational::integer(IBig::from(-1))
                && end.lower != Rational::integer(IBig::ONE))
        {
            return Err(AtlasInvalid::GapOrOverlap);
        }
        let left_included = if left.slope_increases_in_ordinal_order.truth() {
            left.upper_endpoint_included.truth()
        } else {
            left.lower_endpoint_included.truth()
        };
        let right_included = if right.slope_increases_in_ordinal_order.truth() {
            right.lower_endpoint_included.truth()
        } else {
            right.upper_endpoint_included.truth()
        };
        if left_included == right_included {
            return Err(AtlasInvalid::PoleBoundaryOwnership);
        }
    }
    termination_piece(item)?;
    Ok(())
}

pub fn piece_contains_vector(piece: &ChartPiece, vector: &PrimitiveVector) -> bool {
    let (denominator, numerator) = if piece.use_x_denominator.truth() {
        (&vector.0, &vector.1)
    } else {
        (&vector.1, &vector.0)
    };
    if denominator.is_zero() {
        return false;
    }
    let sign = if denominator > &IBig::ZERO { 1 } else { -1 };
    if !piece.denominator_sign.equals_integer(&IBig::from(sign)) {
        return false;
    }
    let magnitude = cftuv_radical::arith::magnitude(denominator);
    let slope = Rational::new(
        if sign > 0 {
            numerator.clone()
        } else {
            -numerator
        },
        magnitude,
    )
    .expect("nonzero denominator");
    let lower = &piece.lower_slope_envelope;
    let upper = &piece.upper_slope_envelope;
    (rational_cmp(&lower.upper, &slope).is_lt() && rational_cmp(&slope, &upper.lower).is_lt())
        || (piece.lower_endpoint_included.truth()
            && lower.lower == lower.upper
            && slope == lower.upper)
        || (piece.upper_endpoint_included.truth()
            && upper.lower == upper.upper
            && slope == upper.lower)
}

/// local_height получает один (lower,upper,True,1), независимо от исходной карты.
pub fn global_termination_height<E, F>(
    boxes: &[TerminationBox],
    mut local_height: F,
) -> Result<IBig, E>
where
    F: FnMut(&TerminationBox) -> Result<IBig, E>,
{
    let mut maximum: Option<IBig> = None;
    for item in boxes {
        let canonical = TerminationBox {
            lower: item.lower.clone(),
            upper: item.upper.clone(),
            use_x_denominator: BoolField::Bool(true),
            denominator_sign: NumericField::Int(IBig::ONE),
        };
        let local = local_height(&canonical)?;
        let absolute = |r: &Rational| {
            if rational_sign(r) < 0 {
                rational_neg(r)
            } else {
                r.clone()
            }
        };
        let mut magnitude = absolute(&item.lower);
        for value in [absolute(&item.upper), Rational::integer(IBig::ONE)] {
            if rational_cmp(&value, &magnitude).is_gt() {
                magnitude = value;
            }
        }
        let scaled = rational_mul(&magnitude, &Rational::integer(local.clone()));
        let (n, d) = scaled.parts();
        let mut ceiling = n / d;
        if n > &IBig::ZERO && !(n % d).is_zero() {
            ceiling += IBig::ONE;
        }
        let bound = local.max(ceiling);
        maximum = Some(maximum.map_or_else(|| bound.clone(), |old| old.max(bound.clone())));
    }
    Ok(maximum.unwrap_or(IBig::ZERO))
}

pub(crate) fn transition_vector(left: usize, right: usize) -> Result<PrimitiveVector, AtlasInvalid> {
    let pair = (left.min(right), left.max(right));
    let (x, y) = match pair {
        (0, 1) => (1, 1),
        (1, 2) => (-1, 1),
        (2, 3) => (-1, -1),
        (0, 3) => (1, -1),
        _ => return Err(AtlasInvalid::ChartUnrepresentable),
    };
    Ok(PrimitiveVector(IBig::from(x), IBig::from(y)))
}

pub fn reachable_pieces<E, F>(
    item: &AtlasWindow,
    mut admissible: F,
) -> Result<Vec<usize>, AtlasPlanFailure<E>>
where
    F: FnMut(&PrimitiveVector) -> Result<bool, E>,
{
    let start = termination_index(item).map_err(AtlasPlanFailure::Invalid)?;
    let mut reached = vec![false; item.pieces.len()];
    reached[start] = true;
    for step in [-1isize, 1] {
        let mut index = start;
        while let Some(next) = index
            .checked_add_signed(step)
            .filter(|&next| next < item.pieces.len())
        {
            let left = index.min(next);
            let vector = transition_vector(
                piece_cell_index(&item.pieces[left]),
                piece_cell_index(&item.pieces[left + 1]),
            )
            .map_err(AtlasPlanFailure::Invalid)?;
            if !admissible(&vector).map_err(AtlasPlanFailure::Callback)? {
                break;
            }
            index = next;
            reached[index] = true;
        }
    }
    Ok(reached
        .iter()
        .enumerate()
        .filter_map(|(index, &yes)| yes.then_some(index))
        .collect())
}

/// Контекст metric/ideal владеет adapter; ordinal/orientation/q переданы явно.
pub trait SegmentPlanLaws {
    type Vector;
    type Scalar;
    type Error;
    fn denominator(
        &mut self,
        vector: &Self::Vector,
        use_x: bool,
    ) -> Result<Self::Scalar, Self::Error>;
    fn sign(&mut self, value: &Self::Scalar) -> Result<i8, Self::Error>;
    fn slope(&mut self, vector: &Self::Vector, use_x: bool) -> Result<Self::Scalar, Self::Error>;
    fn decimal_envelope(&mut self, value: &Self::Scalar) -> Result<DecimalEnvelope, Self::Error>;
    fn local_candidate(
        &mut self,
        ordinal: usize,
        vector: &Self::Vector,
        q: usize,
    ) -> Result<bool, Self::Error>;
    fn vector_from_slope(
        &mut self,
        value: &Rational,
        use_x: bool,
        sign: i8,
    ) -> Result<Self::Vector, Self::Error>;
    /// Python reachable_pieces передаёт integer pair непосредственно local_candidate.
    fn local_transition(
        &mut self,
        ordinal: usize,
        vector: &PrimitiveVector,
        q: usize,
    ) -> Result<bool, Self::Error>;
    fn subturn_boundary_vectors(
        &mut self,
        ordinal: usize,
        orientation: i8,
        q: usize,
    ) -> Result<Vec<Self::Vector>, Self::Error>;
}

pub fn piece_bracket<L: SegmentPlanLaws>(
    boundaries: &[L::Vector],
    piece: &ChartPiece,
    laws: &mut L,
) -> Result<[Rational; 4], AtlasPlanFailure<L::Error>> {
    let use_x = piece.use_x_denominator.truth();
    let sum = rational_add(
        &piece.lower_slope_envelope.upper,
        &piece.upper_slope_envelope.lower,
    );
    let middle = rational_mul(
        &sum,
        &Rational::new(IBig::ONE, UBig::from(2u8)).expect("positive denominator"),
    );
    let mut lower = piece.lower_slope_envelope.clone();
    let mut upper = piece.upper_slope_envelope.clone();
    for boundary in boundaries {
        let denominator = laws
            .denominator(boundary, use_x)
            .map_err(AtlasPlanFailure::Callback)?;
        let sign = laws
            .sign(&denominator)
            .map_err(AtlasPlanFailure::Callback)?;
        if !piece.denominator_sign.equals_integer(&IBig::from(sign)) {
            continue;
        }
        let slope = laws
            .slope(boundary, use_x)
            .map_err(AtlasPlanFailure::Callback)?;
        let envelope = laws
            .decimal_envelope(&slope)
            .map_err(AtlasPlanFailure::Callback)?;
        if rational_cmp(&envelope.upper, &middle).is_lt() {
            if rational_cmp(&envelope.upper, &lower.upper).is_gt() {
                lower = envelope;
            }
        } else if rational_cmp(&envelope.lower, &middle).is_gt()
            && rational_cmp(&envelope.lower, &upper.lower).is_lt()
        {
            upper = envelope;
        }
    }
    Ok([lower.lower, lower.upper, upper.lower, upper.upper])
}

pub fn proven_segment<L: SegmentPlanLaws>(
    ordinal: usize,
    q: usize,
    segment: &GlobalHeightSegment,
    laws: &mut L,
) -> Result<GlobalHeightSegment, AtlasPlanFailure<L::Error>> {
    if rational_cmp(&segment.inner_lower, &segment.inner_upper).is_ge() {
        return Ok(segment.clone());
    }
    for value in [&segment.inner_lower, &segment.inner_upper] {
        let vector = laws
            .vector_from_slope(value, segment.use_x_denominator, segment.denominator_sign)
            .map_err(AtlasPlanFailure::Callback)?;
        if !laws
            .local_candidate(ordinal, &vector, q)
            .map_err(AtlasPlanFailure::Callback)?
        {
            let mut unproven = segment.clone();
            unproven.inner_lower = Rational::integer(IBig::ZERO);
            unproven.inner_upper = Rational::integer(IBig::ZERO);
            return Ok(unproven);
        }
    }
    Ok(segment.clone())
}

pub fn ordinal_segments<L: SegmentPlanLaws>(
    ordinal: usize,
    orientation: i8,
    q: usize,
    window: &Window,
    record: &LocalRecord,
    sealed: &[Rational; 4],
    laws: &mut L,
) -> Result<Vec<GlobalHeightSegment>, AtlasPlanFailure<L::Error>> {
    let mut raw = Vec::new();
    match window {
        Window::SingleChart => {
            if !record.use_x_denominator.is_bool() {
                return Err(AtlasPlanFailure::UnsupportedChartDescriptor);
            }
            let sign = record
                .denominator_sign
                .unit_sign()
                .ok_or(AtlasPlanFailure::UnsupportedDenominatorDescriptor)?;
            raw = cell_segments(record.use_x_denominator.truth(), sign, sealed);
        }
        Window::Atlas(item) => {
            // Boundary callback идёт ДО termination identity и reachable callbacks.
            let boundaries = laws
                .subturn_boundary_vectors(ordinal, orientation, q)
                .map_err(AtlasPlanFailure::Callback)?;
            let indices =
                reachable_pieces(item, |vector| laws.local_transition(ordinal, vector, q))?;
            for index in indices {
                let piece = &item.pieces[index];
                let bracket = if item
                    .termination_piece_index
                    .equals_integer(&IBig::from(index))
                {
                    sealed.clone()
                } else {
                    piece_bracket(&boundaries, piece, laws)?
                };
                let sign = piece
                    .denominator_sign
                    .unit_sign()
                    .ok_or(AtlasPlanFailure::UnsupportedDenominatorDescriptor)?;
                if !piece.use_x_denominator.is_bool() {
                    return Err(AtlasPlanFailure::UnsupportedChartDescriptor);
                }
                raw.extend(cell_segments(
                    piece.use_x_denominator.truth(),
                    sign,
                    &bracket,
                ));
            }
        }
    }
    // Все brackets строятся до первого inner proof; late bracket failure не вызывает proof.
    raw.iter()
        .map(|item| proven_segment(ordinal, q, item, laws))
        .collect()
}

pub fn search_segments<L: SegmentPlanLaws>(
    orientation: i8,
    q: usize,
    windows: &[Window],
    records: &[LocalRecord],
    sealed: &[[Rational; 4]],
    laws: &mut L,
) -> Result<Vec<Vec<GlobalHeightSegment>>, AtlasPlanFailure<L::Error>> {
    let mut result = Vec::new();
    for index in 0.. {
        // CPython zip(strict=True) отказывает при следующем чтении, после ранних ordinals.
        let Some(window) = windows.get(index) else {
            if records.get(index).is_some() {
                return Err(AtlasPlanFailure::StrictZip {
                    argument: 2,
                    relation: ZipRelation::Longer,
                });
            }
            if sealed.get(index).is_some() {
                return Err(AtlasPlanFailure::StrictZip {
                    argument: 3,
                    relation: ZipRelation::Longer,
                });
            }
            return Ok(result);
        };
        let Some(record) = records.get(index) else {
            return Err(AtlasPlanFailure::StrictZip {
                argument: 2,
                relation: ZipRelation::Shorter,
            });
        };
        let Some(bracket) = sealed.get(index) else {
            return Err(AtlasPlanFailure::StrictZip {
                argument: 3,
                relation: ZipRelation::Shorter,
            });
        };
        result.push(ordinal_segments(
            index + 1,
            orientation,
            q,
            window,
            record,
            bracket,
            laws,
        )?);
    }
    unreachable!()
}
