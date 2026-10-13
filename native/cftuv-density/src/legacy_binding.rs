//! B(w) сертификат и проверка legacy Density fan над prepared records.
//! Oracle: adaptive_density_fan.py::_legacy_density_certificate/_legacy_full_fan_valid.
//! Scalar/legacy admission — у callbacks; authority hash/Decimal repr здесь отсутствуют.

use cftuv_radical::Term;
use cftuv_radical::arith::{gcd_signed, magnitude};

use crate::atlas_plan::DecimalEnvelope;
use crate::farey::{PrimitiveVector, SingleChartWindow, WorkBudget, WorkBudgetExhausted};
use crate::interval::{rational_cmp, rational_sub};
use crate::window_build::PreparedWindowRecord;
use crate::{IBig, Radical, Rational, UBig};

/// Результат исходного equality с _PROVEN_PREDICATES, без интерпретатора set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProvenPredicates {
    Canonical,
    NonCanonical,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyCertificate {
    pub bound_primitive_integer_vector: PrimitiveVector,
    pub ideal_window_lower_slope_envelope: DecimalEnvelope,
    pub ideal_window_upper_slope_envelope: DecimalEnvelope,
    pub certified_window_width_lower_bound: Rational,
    pub proven_predicates: ProvenPredicates,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LegacyBindingFailure<E> {
    EmptyWindow,
    Work(WorkBudgetExhausted),
    Callback(E),
    /// Внутренний недостижимый product input; не продуктовый UNDECIDABLE.
    MissingIdealVector {
        index: usize,
    },
    /// Ordinal product API — строго 1-based; malformed ordinal не интерпретируется.
    InvalidOrdinal,
    /// Source zip(strict=True) проверяет длину только при достижении этой позиции.
    RecordsShorter,
    RecordsLonger,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyCertificateReport<E> {
    pub result: Result<Option<LegacyCertificate>, LegacyBindingFailure<E>>,
    pub budget: Option<WorkBudget>,
}

pub trait LegacyBindingLaws {
    type Vector: Clone;
    type Score;
    type Error;
    fn vector(&mut self, value: &PrimitiveVector) -> Result<Self::Vector, Self::Error>;
    fn subturn(
        &mut self,
        left: &Self::Vector,
        right: &Self::Vector,
        q: usize,
    ) -> Result<bool, Self::Error>;
    fn objective(
        &mut self,
        ordinal: usize,
        value: &PrimitiveVector,
    ) -> Result<Self::Score, Self::Error>;
    /// Оба вызова source _sign(score-winner) наблюдаемы; кеш здесь запрещён.
    fn score_difference_sign(
        &mut self,
        left: &Self::Score,
        right: &Self::Score,
    ) -> Result<i8, Self::Error>;
    fn oriented_cross_sign(
        &mut self,
        left: &Self::Vector,
        right: &Self::Vector,
    ) -> Result<i8, Self::Error>;
    fn window_triple(
        &mut self,
        ordinal: usize,
    ) -> Result<(Self::Vector, Self::Vector, Self::Vector), Self::Error>;
    fn inside_ordinal(
        &mut self,
        candidate: &Self::Vector,
        triple: &(Self::Vector, Self::Vector, Self::Vector),
    ) -> Result<bool, Self::Error>;
    /// Сохраняет source _slope и последующее преобразование в Fraction.
    fn slope(&mut self, value: &Self::Vector, use_x: bool) -> Result<Rational, Self::Error>;
    /// Source поле — Decimal subtraction, а не exact Fraction width для B(w).
    /// Context/округление обязаны прийти от adapter, например decimal_projection glue.
    fn certified_width(
        &mut self,
        lower: &DecimalEnvelope,
        upper: &DecimalEnvelope,
    ) -> Result<Rational, Self::Error>;
}

pub fn legacy_authority_budget<E>(width: &Rational) -> Result<UBig, LegacyBindingFailure<E>> {
    let (numerator, denominator) = width.parts();
    if numerator <= &IBig::ZERO {
        return Err(LegacyBindingFailure::EmptyWindow);
    }
    let numerator = magnitude(numerator);
    let quotient_ceiling = (denominator + &numerator - UBig::ONE) / &numerator;
    let mut budget = Radical::floor_root(&Term {
        radicand: quotient_ceiling.clone(),
        numerator: IBig::ONE,
        denominator: UBig::ONE,
    });
    while &budget * &budget < quotient_ceiling {
        budget += UBig::ONE;
    }
    while budget > UBig::ONE {
        let previous = &budget - UBig::ONE;
        if &previous * &previous * &numerator < *denominator {
            break;
        }
        budget -= UBig::ONE;
    }
    Ok(budget)
}

fn ideal_vector<V, E>(ideal: &[V], index: usize) -> Result<&V, LegacyBindingFailure<E>> {
    ideal
        .get(index)
        .ok_or(LegacyBindingFailure::MissingIdealVector { index })
}

pub fn legacy_density_certificate<L: LegacyBindingLaws>(
    ideal: &[L::Vector],
    ordinal: usize,
    q: usize,
    record: &PreparedWindowRecord,
    preparation_cap: UBig,
    laws: &mut L,
) -> LegacyCertificateReport<L::Error> {
    let mut budget = None;
    let result = certificate_inner(
        ideal,
        ordinal,
        q,
        record,
        preparation_cap,
        laws,
        &mut budget,
    );
    LegacyCertificateReport { result, budget }
}

fn certificate_inner<L: LegacyBindingLaws>(
    ideal: &[L::Vector],
    ordinal: usize,
    q: usize,
    record: &PreparedWindowRecord,
    cap: UBig,
    laws: &mut L,
    observed: &mut Option<WorkBudget>,
) -> Result<Option<LegacyCertificate>, LegacyBindingFailure<L::Error>> {
    let PreparedWindowRecord::SingleChart(record) = record else {
        return Ok(None);
    };
    let lower = &record.lower_slope_envelope.upper;
    let upper = &record.upper_slope_envelope.lower;
    let width = rational_sub(upper, lower);
    let bound = legacy_authority_budget(&width)?;
    let budget = observed.insert(WorkBudget::new(cap));
    budget
        .spend_shell_probes(bound.clone())
        .map_err(LegacyBindingFailure::Work)?;
    let chart = SingleChartWindow {
        use_x: record.use_x_denominator,
        denominator_sign: record.denominator_sign,
        sealed: [lower.clone(), lower.clone(), upper.clone(), upper.clone()],
    };
    let mut winner: Option<(L::Score, Rational, PrimitiveVector)> = None;
    let mut denominator = UBig::ONE;
    while denominator <= bound {
        let (mut numerator, last) =
            crate::farey::strict_numerator_bounds(lower, upper, &denominator);
        while numerator <= last {
            if gcd_signed(&numerator, &denominator) == UBig::ONE {
                let slope = Rational::new(numerator.clone(), denominator.clone())
                    .expect("positive denominator");
                let vector = crate::farey::vector_from_slope(&numerator, &denominator, &chart);
                let candidate = laws
                    .vector(&vector)
                    .map_err(LegacyBindingFailure::Callback)?;
                let previous_index = ordinal
                    .checked_sub(1)
                    .ok_or(LegacyBindingFailure::InvalidOrdinal)?;
                let previous = ideal_vector(ideal, previous_index)?;
                if laws
                    .subturn(previous, &candidate, q)
                    .map_err(LegacyBindingFailure::Callback)?
                {
                    let following = ideal_vector(ideal, ordinal + 1)?;
                    if laws
                        .subturn(&candidate, following, q)
                        .map_err(LegacyBindingFailure::Callback)?
                    {
                        let score = laws
                            .objective(ordinal, &vector)
                            .map_err(LegacyBindingFailure::Callback)?;
                        let replace = if let Some((best, best_slope, _)) = &winner {
                            laws.score_difference_sign(&score, best)
                                .map_err(LegacyBindingFailure::Callback)?
                                > 0
                                || (laws
                                    .score_difference_sign(&score, best)
                                    .map_err(LegacyBindingFailure::Callback)?
                                    == 0
                                    && rational_cmp(&slope, best_slope).is_gt())
                        } else {
                            true
                        };
                        if replace {
                            winner = Some((score, slope, vector));
                        }
                    }
                }
            }
            numerator += IBig::ONE;
        }
        denominator += UBig::ONE;
    }
    let Some((_, _, vector)) = winner else {
        return Ok(None);
    };
    let certified_width = laws
        .certified_width(&record.lower_slope_envelope, &record.upper_slope_envelope)
        .map_err(LegacyBindingFailure::Callback)?;
    Ok(Some(LegacyCertificate {
        bound_primitive_integer_vector: vector,
        ideal_window_lower_slope_envelope: record.lower_slope_envelope.clone(),
        ideal_window_upper_slope_envelope: record.upper_slope_envelope.clone(),
        certified_window_width_lower_bound: certified_width,
        proven_predicates: ProvenPredicates::Canonical,
    }))
}

pub fn legacy_full_fan_valid<L: LegacyBindingLaws>(
    ideal: &[L::Vector],
    orientation: i8,
    q: usize,
    records: &[PreparedWindowRecord],
    certificates: &[Option<LegacyCertificate>],
    laws: &mut L,
) -> Result<bool, LegacyBindingFailure<L::Error>> {
    // Source сначала resolve ВСЕ vectors, включая будущий noncanonical certificate.
    let mut resolved = Vec::new();
    for (index, certificate) in certificates.iter().enumerate() {
        resolved.push(match certificate {
            Some(certificate) => laws
                .vector(&certificate.bound_primitive_integer_vector)
                .map_err(LegacyBindingFailure::Callback)?,
            None => ideal_vector(ideal, index + 1)?.clone(),
        });
    }
    let mut sequence = vec![ideal_vector(ideal, 0)?.clone()];
    sequence.extend(resolved.iter().cloned());
    sequence.push(
        ideal
            .last()
            .ok_or(LegacyBindingFailure::MissingIdealVector { index: 0 })?
            .clone(),
    );
    if certificates
        .iter()
        .flatten()
        .any(|c| c.proven_predicates != ProvenPredicates::Canonical)
    {
        return Ok(false);
    }
    for pair in sequence.windows(2) {
        if laws
            .oriented_cross_sign(&pair[0], &pair[1])
            .map_err(LegacyBindingFailure::Callback)?
            != orientation
            || !laws
                .subturn(&pair[0], &pair[1], q)
                .map_err(LegacyBindingFailure::Callback)?
        {
            return Ok(false);
        }
    }
    for (index, (certificate, candidate)) in certificates.iter().zip(&resolved).enumerate() {
        let record = records
            .get(index)
            .ok_or(LegacyBindingFailure::RecordsShorter)?;
        if certificate.is_none() {
            continue;
        }
        let PreparedWindowRecord::SingleChart(record) = record else {
            return Ok(false);
        };
        let triple = laws
            .window_triple(index + 1)
            .map_err(LegacyBindingFailure::Callback)?;
        if !laws
            .inside_ordinal(candidate, &triple)
            .map_err(LegacyBindingFailure::Callback)?
        {
            return Ok(false);
        }
        let slope = laws
            .slope(candidate, record.use_x_denominator)
            .map_err(LegacyBindingFailure::Callback)?;
        if rational_cmp(&record.lower_slope_envelope.upper, &slope).is_ge()
            || rational_cmp(&slope, &record.upper_slope_envelope.lower).is_ge()
        {
            return Ok(false);
        }
    }
    if records.len() > certificates.len() {
        return Err(LegacyBindingFailure::RecordsLonger);
    }
    Ok(true)
}
