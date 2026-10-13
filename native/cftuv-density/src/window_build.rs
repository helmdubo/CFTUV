//! Построение ordinal windows/Atlas prepared records, без sealed/termination/wire authority.
//! Oracle: adaptive_density_fan.py::_window_envelopes; adaptive_density_atlas.py
//! ::_exact_decimal_interval/_cell/_cell_path/_piece_vectors/build_atlas_window_record;
//! density_projective.py::_chart_for_window. Scalar/legacy admission — у callbacks.

use crate::atlas_plan::{
    AtlasWindow, BoolField, ChartPiece, DecimalEnvelope, EndpointKind, NumericField, PoleField,
    PoleOwnership, RatioField, Window, WindowRecordKind, transition_vector,
};
use crate::fan_prepare::PrepareRecord;
use crate::{IBig, Rational};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PreparedAtlasWindow {
    pub pieces: Vec<ChartPiece>,
    pub termination_piece_index: usize,
    pub local_record: PrepareRecord,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PreparedWindowRecord {
    SingleChart(PrepareRecord),
    Atlas(PreparedAtlasWindow),
}

impl PreparedWindowRecord {
    pub fn local_record(&self) -> &PrepareRecord {
        match self {
            Self::SingleChart(record) => record,
            Self::Atlas(record) => &record.local_record,
        }
    }

    /// Prepared не содержит семи ratio wire-полей; Other явно обозначает их отсутствие.
    /// Проекция не выдаётся за AtlasV1 authority и не проходит premature wire validation.
    pub fn plan_window(&self, ordinal: usize) -> Window {
        match self {
            Self::SingleChart(_) => Window::SingleChart,
            Self::Atlas(record) => Window::Atlas(AtlasWindow {
                kind: WindowRecordKind::Prepared,
                ordinal: NumericField::Int(IBig::from(ordinal)),
                pieces: record.pieces.clone(),
                termination_piece_index: NumericField::Int(IBig::from(
                    record.termination_piece_index,
                )),
                termination_lower_slope: RatioField::Other,
                termination_upper_slope: RatioField::Other,
                certified_termination_width: RatioField::Other,
                admissible_lower_outward: RatioField::Other,
                admissible_lower_inward: RatioField::Other,
                admissible_upper_inward: RatioField::Other,
                admissible_upper_outward: RatioField::Other,
            }),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BuiltWindows<V> {
    pub windows: Vec<(V, V)>,
    pub records: Vec<PreparedWindowRecord>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum WindowBuildFailure<E> {
    ChartUnrepresentable,
    OrdinalEnvelopesOverlap,
    Callback(E),
    /// Неконсистентные callback snapshots: source path.index даёт поздний ValueError.
    /// Внутренний prototype input failure; не product ChartUnrepresentable/UNDECIDABLE.
    MissingIdealCell(usize),
}

pub trait WindowBuildLaws {
    type Vector: Clone;
    type Scalar: Clone;
    type Error;
    fn window_neighbours(
        &mut self,
        ordinal: usize,
    ) -> Result<(Self::Vector, Self::Vector), Self::Error>;
    fn coordinates(
        &mut self,
        vector: &Self::Vector,
    ) -> Result<(Self::Scalar, Self::Scalar), Self::Error>;
    fn add(
        &mut self,
        left: &Self::Scalar,
        right: &Self::Scalar,
    ) -> Result<Self::Scalar, Self::Error>;
    fn sub(
        &mut self,
        left: &Self::Scalar,
        right: &Self::Scalar,
    ) -> Result<Self::Scalar, Self::Error>;
    fn mul(
        &mut self,
        left: &Self::Scalar,
        right: &Self::Scalar,
    ) -> Result<Self::Scalar, Self::Error>;
    fn sign(&mut self, scalar: &Self::Scalar) -> Result<i8, Self::Error>;
    fn vector(&mut self, x: Self::Scalar, y: Self::Scalar) -> Result<Self::Vector, Self::Error>;
    fn integer_vector(&mut self, x: &IBig, y: &IBig) -> Result<Self::Vector, Self::Error>;
    fn slope(&mut self, vector: &Self::Vector, use_x: bool) -> Result<Self::Scalar, Self::Error>;
    fn decimal_envelope(&mut self, scalar: &Self::Scalar) -> Result<DecimalEnvelope, Self::Error>;
    /// Именно source equality/membership, не новый exact-sign proof scalar-integer.
    fn equals_integer(&mut self, scalar: &Self::Scalar, integer: i8) -> Result<bool, Self::Error>;
    fn integer_value(&mut self, scalar: &Self::Scalar) -> Result<IBig, Self::Error>;
    fn rational_lt_scalar(
        &mut self,
        rational: &Rational,
        scalar: &Self::Scalar,
    ) -> Result<bool, Self::Error>;
    fn scalar_lt_rational(
        &mut self,
        scalar: &Self::Scalar,
        rational: &Rational,
    ) -> Result<bool, Self::Error>;
    /// Catch только исходного DensityWindowChartUnrepresentable, только внутри chart try.
    fn is_chart_unrepresentable(error: &Self::Error) -> bool;
}

pub fn exact_decimal_interval(value: IBig) -> DecimalEnvelope {
    let value = Rational::integer(value);
    DecimalEnvelope {
        lower: value.clone(),
        upper: value,
        lower_kind: EndpointKind::Closed,
        upper_kind: EndpointKind::Closed,
        absolute_error_bound: Rational::integer(IBig::ZERO),
        quantum: Some(crate::atlas_plan::DecimalQuantum::uniform(0)),
    }
}

fn square_balance<L: WindowBuildLaws>(
    x: &L::Scalar,
    y: &L::Scalar,
    laws: &mut L,
) -> Result<L::Scalar, WindowBuildFailure<L::Error>> {
    let xx = laws.mul(x, x).map_err(WindowBuildFailure::Callback)?;
    let yy = laws.mul(y, y).map_err(WindowBuildFailure::Callback)?;
    laws.sub(&xx, &yy).map_err(WindowBuildFailure::Callback)
}
fn cross<L: WindowBuildLaws>(
    left: &(L::Scalar, L::Scalar),
    right: &(L::Scalar, L::Scalar),
    laws: &mut L,
) -> Result<L::Scalar, WindowBuildFailure<L::Error>> {
    let a = laws
        .mul(&left.0, &right.1)
        .map_err(WindowBuildFailure::Callback)?;
    let b = laws
        .mul(&left.1, &right.0)
        .map_err(WindowBuildFailure::Callback)?;
    laws.sub(&a, &b).map_err(WindowBuildFailure::Callback)
}

pub fn cell<L: WindowBuildLaws>(
    vector: &L::Vector,
    laws: &mut L,
) -> Result<(usize, bool, i8), WindowBuildFailure<L::Error>> {
    let (x, y) = laws
        .coordinates(vector)
        .map_err(WindowBuildFailure::Callback)?;
    let balance = square_balance::<L>(&x, &y, laws)?;
    let use_x = laws.sign(&balance).map_err(WindowBuildFailure::Callback)? >= 0;
    let sign = laws
        .sign(if use_x { &x } else { &y })
        .map_err(WindowBuildFailure::Callback)?;
    if sign == 0 {
        return Err(WindowBuildFailure::ChartUnrepresentable);
    }
    let index = if use_x {
        if sign > 0 { 0 } else { 2 }
    } else if sign > 0 {
        1
    } else {
        3
    };
    Ok((index, use_x, sign))
}

pub fn chart_for_window<L: WindowBuildLaws>(
    ideal: &L::Vector,
    left: &L::Vector,
    right: &L::Vector,
    laws: &mut L,
) -> Result<(bool, i8), WindowBuildFailure<L::Error>> {
    let (ix, iy) = laws
        .coordinates(ideal)
        .map_err(WindowBuildFailure::Callback)?;
    let balance = square_balance::<L>(&ix, &iy, laws)?;
    let preferred = laws.sign(&balance).map_err(WindowBuildFailure::Callback)? >= 0;
    for use_x in [preferred, !preferred] {
        let mut values = Vec::new();
        for vector in [left, ideal, right] {
            let (x, y) = laws
                .coordinates(vector)
                .map_err(WindowBuildFailure::Callback)?;
            values.push(if use_x { x } else { y });
        }
        let mut signs = Vec::new();
        for value in &values {
            signs.push(laws.sign(value).map_err(WindowBuildFailure::Callback)?);
        }
        if signs.contains(&0) || signs.iter().any(|s| *s != signs[0]) {
            continue;
        }
        return Ok((use_x, signs[0]));
    }
    Err(WindowBuildFailure::ChartUnrepresentable)
}

pub fn cell_path<L: WindowBuildLaws>(
    left: &L::Vector,
    ideal: &L::Vector,
    right: &L::Vector,
    laws: &mut L,
) -> Result<Vec<usize>, WindowBuildFailure<L::Error>> {
    let mut values = Vec::new();
    for vector in [left, ideal, right] {
        values.push(
            laws.coordinates(vector)
                .map_err(WindowBuildFailure::Callback)?,
        );
    }
    let first = cross::<L>(&values[0], &values[1], laws)?;
    let first = laws.sign(&first).map_err(WindowBuildFailure::Callback)?;
    let second = cross::<L>(&values[1], &values[2], laws)?;
    let second = laws.sign(&second).map_err(WindowBuildFailure::Callback)?;
    if first == 0 || second == 0 || first != second {
        return Err(WindowBuildFailure::ChartUnrepresentable);
    }
    let mut cells = Vec::new();
    // Source перечитывает coordinates в каждом _cell; snapshots не заменяют callbacks.
    for vector in [left, ideal, right] {
        cells.push(cell(vector, laws)?.0);
    }
    let mut path = vec![cells[0]];
    for _ in 0..4 {
        if *path.last().unwrap() == cells[2] {
            break;
        }
        let current = *path.last().unwrap();
        path.push((current + if first > 0 { 1 } else { 3 }) % 4);
    }
    if *path.last().unwrap() != cells[2] || !path.contains(&cells[1]) {
        return Err(WindowBuildFailure::ChartUnrepresentable);
    }
    Ok(path)
}

fn transition<L: WindowBuildLaws>(
    left: usize,
    right: usize,
    laws: &mut L,
) -> Result<L::Vector, WindowBuildFailure<L::Error>> {
    let vector =
        transition_vector(left, right).map_err(|_| WindowBuildFailure::ChartUnrepresentable)?;
    laws.integer_vector(&vector.0, &vector.1)
        .map_err(WindowBuildFailure::Callback)
}

fn piece_vectors<L: WindowBuildLaws>(
    path: &[usize],
    index: usize,
    left: &L::Vector,
    right: &L::Vector,
    laws: &mut L,
) -> Result<(L::Vector, L::Vector), WindowBuildFailure<L::Error>> {
    let start = if index == 0 {
        left.clone()
    } else {
        transition(path[index - 1], path[index], laws)?
    };
    let end = if index == path.len() - 1 {
        right.clone()
    } else {
        transition(path[index], path[index + 1], laws)?
    };
    Ok((start, end))
}

fn endpoint_envelope<L: WindowBuildLaws>(
    value: &L::Scalar,
    laws: &mut L,
) -> Result<DecimalEnvelope, WindowBuildFailure<L::Error>> {
    if laws
        .equals_integer(value, -1)
        .map_err(WindowBuildFailure::Callback)?
        || laws
            .equals_integer(value, 1)
            .map_err(WindowBuildFailure::Callback)?
    {
        let integer = laws
            .integer_value(value)
            .map_err(WindowBuildFailure::Callback)?;
        Ok(exact_decimal_interval(integer))
    } else {
        laws.decimal_envelope(value)
            .map_err(WindowBuildFailure::Callback)
    }
}

pub fn build_atlas_window_record<L: WindowBuildLaws>(
    ideal: &L::Vector,
    left: &L::Vector,
    right: &L::Vector,
    laws: &mut L,
) -> Result<PreparedAtlasWindow, WindowBuildFailure<L::Error>> {
    let path = cell_path(left, ideal, right, laws)?;
    let ideal_cell = cell(ideal, laws)?.0;
    let mut pieces = Vec::new();
    for (index, &cell) in path.iter().enumerate() {
        let use_x = matches!(cell, 0 | 2);
        let denominator_sign = if matches!(cell, 0 | 1) { 1 } else { -1 };
        let (start_vector, end_vector) = piece_vectors(&path, index, left, right, laws)?;
        let start_exact = laws
            .slope(&start_vector, use_x)
            .map_err(WindowBuildFailure::Callback)?;
        let end_exact = laws
            .slope(&end_vector, use_x)
            .map_err(WindowBuildFailure::Callback)?;
        let difference = laws
            .sub(&end_exact, &start_exact)
            .map_err(WindowBuildFailure::Callback)?;
        let order = laws
            .sign(&difference)
            .map_err(WindowBuildFailure::Callback)?;
        if order == 0 {
            return Err(WindowBuildFailure::ChartUnrepresentable);
        }
        let increases = order > 0;
        let start = endpoint_envelope::<L>(&start_exact, laws)?;
        let end = endpoint_envelope::<L>(&end_exact, laws)?;
        let (lower, upper, lower_exact, upper_exact) = if increases {
            (start, end, &start_exact, &end_exact)
        } else {
            (end, start, &end_exact, &start_exact)
        };
        if crate::interval::rational_cmp(&lower.upper, &upper.lower).is_ge() {
            return Err(WindowBuildFailure::ChartUnrepresentable);
        }
        let pole = if laws
            .sign(lower_exact)
            .map_err(WindowBuildFailure::Callback)?
            < 0
            && laws
                .sign(upper_exact)
                .map_err(WindowBuildFailure::Callback)?
                > 0
        {
            if use_x {
                PoleOwnership::YZero
            } else {
                PoleOwnership::XZero
            }
        } else {
            PoleOwnership::None
        };
        let start_included = index != 0 && use_x;
        let end_included = index != path.len() - 1 && use_x;
        pieces.push(ChartPiece {
            is_piece_v1: true,
            piece_index: NumericField::Int(IBig::from(index)),
            use_x_denominator: BoolField::Bool(use_x),
            denominator_sign: NumericField::Int(IBig::from(denominator_sign)),
            lower_slope_envelope: lower,
            upper_slope_envelope: upper,
            lower_endpoint_included: BoolField::Bool(if increases {
                start_included
            } else {
                end_included
            }),
            upper_endpoint_included: BoolField::Bool(if increases {
                end_included
            } else {
                start_included
            }),
            slope_increases_in_ordinal_order: BoolField::Bool(increases),
            pole_ownership: PoleField::Enum(pole),
        });
    }
    let index = path
        .iter()
        .position(|&c| c == ideal_cell)
        .ok_or(WindowBuildFailure::MissingIdealCell(ideal_cell))?;
    let piece = &pieces[index];
    let use_x = piece.use_x_denominator.truth();
    let slope = laws
        .slope(ideal, use_x)
        .map_err(WindowBuildFailure::Callback)?;
    if !laws
        .rational_lt_scalar(&piece.lower_slope_envelope.upper, &slope)
        .map_err(WindowBuildFailure::Callback)?
        || !laws
            .scalar_lt_rational(&slope, &piece.upper_slope_envelope.lower)
            .map_err(WindowBuildFailure::Callback)?
    {
        return Err(WindowBuildFailure::ChartUnrepresentable);
    }
    Ok(PreparedAtlasWindow {
        local_record: PrepareRecord {
            use_x_denominator: use_x,
            denominator_sign: if matches!(ideal_cell, 0 | 1) { 1 } else { -1 },
            lower_slope_envelope: piece.lower_slope_envelope.clone(),
            upper_slope_envelope: piece.upper_slope_envelope.clone(),
        },
        pieces,
        termination_piece_index: index,
    })
}

pub fn window_envelopes<L: WindowBuildLaws>(
    ideal: &[L::Vector],
    laws: &mut L,
) -> Result<BuiltWindows<L::Vector>, WindowBuildFailure<L::Error>> {
    let mut windows = Vec::new();
    let mut records = Vec::new();
    for ordinal in 1..ideal.len().saturating_sub(1) {
        let (previous, following) = laws
            .window_neighbours(ordinal)
            .map_err(WindowBuildFailure::Callback)?;
        let (px, py) = laws
            .coordinates(&previous)
            .map_err(WindowBuildFailure::Callback)?;
        let (ix, iy) = laws
            .coordinates(&ideal[ordinal])
            .map_err(WindowBuildFailure::Callback)?;
        let (fx, fy) = laws
            .coordinates(&following)
            .map_err(WindowBuildFailure::Callback)?;
        let lx = laws.add(&px, &ix).map_err(WindowBuildFailure::Callback)?;
        let ly = laws.add(&py, &iy).map_err(WindowBuildFailure::Callback)?;
        let left = laws.vector(lx, ly).map_err(WindowBuildFailure::Callback)?;
        let rx = laws.add(&ix, &fx).map_err(WindowBuildFailure::Callback)?;
        let ry = laws.add(&iy, &fy).map_err(WindowBuildFailure::Callback)?;
        let right = laws.vector(rx, ry).map_err(WindowBuildFailure::Callback)?;
        let chart = chart_for_window(&ideal[ordinal], &left, &right, laws);
        let (use_x, sign) = match chart {
            Ok(chart) => chart,
            Err(WindowBuildFailure::ChartUnrepresentable) => {
                let record = build_atlas_window_record(&ideal[ordinal], &left, &right, laws)?;
                windows.push((left, right));
                records.push(PreparedWindowRecord::Atlas(record));
                continue;
            }
            Err(WindowBuildFailure::Callback(error)) if L::is_chart_unrepresentable(&error) => {
                let record = build_atlas_window_record(&ideal[ordinal], &left, &right, laws)?;
                windows.push((left, right));
                records.push(PreparedWindowRecord::Atlas(record));
                continue;
            }
            Err(error) => return Err(error),
        };
        // Следующие slope/envelope failures уже ВНЕ chart try; atlas fallback здесь запрещён.
        let a = laws
            .slope(&left, use_x)
            .map_err(WindowBuildFailure::Callback)?;
        let b = laws
            .slope(&right, use_x)
            .map_err(WindowBuildFailure::Callback)?;
        let difference = laws.sub(&b, &a).map_err(WindowBuildFailure::Callback)?;
        let increasing = laws
            .sign(&difference)
            .map_err(WindowBuildFailure::Callback)?
            > 0;
        let (lower_exact, upper_exact) = if increasing { (a, b) } else { (b, a) };
        let lower = laws
            .decimal_envelope(&lower_exact)
            .map_err(WindowBuildFailure::Callback)?;
        let upper = laws
            .decimal_envelope(&upper_exact)
            .map_err(WindowBuildFailure::Callback)?;
        if crate::interval::rational_cmp(&lower.upper, &upper.lower).is_ge() {
            return Err(WindowBuildFailure::OrdinalEnvelopesOverlap);
        }
        windows.push((left, right));
        records.push(PreparedWindowRecord::SingleChart(PrepareRecord {
            use_x_denominator: use_x,
            denominator_sign: sign,
            lower_slope_envelope: lower,
            upper_slope_envelope: upper,
        }));
    }
    Ok(BuiltWindows { windows, records })
}
