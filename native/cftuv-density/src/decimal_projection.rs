//! Integer projection legacy MPI на decimal grid 10^-27 с сохранением quantum.
//! Старые endpoints не заменяются более узкой exact-оболочкой. Округление width
//! соответствует явной точности ROUND_HALF_EVEN; контекста процесса здесь нет.
use crate::legacy_fixed::floor_div;
use crate::legacy_interval::LegacyInterval;
use crate::legacy_mpf::{Mpf, MAX_SHIFT_BITS};
use crate::{IBig, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectionError {
    NonFinite,
    Shift(u64),
    Precision(u32),
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecimalText {
    coefficient: IBig,
    exponent: i64,
}
impl DecimalText {
    pub fn coefficient(&self) -> &IBig { &self.coefficient }
    pub fn exponent(&self) -> i64 { self.exponent }
    pub fn text(&self) -> String {
        let raw = self.coefficient.to_string();
        let (sign, digits) = if let Some(digits) = raw.strip_prefix('-') {
            ("-", digits)
        } else {
            ("", raw.as_str())
        };
        let adjusted = digits.len() as i64 - 1 + self.exponent;
        if self.exponent <= 0 && adjusted >= -6 {
            if self.exponent == 0 {
                return format!("{sign}{digits}");
            }
            let point = digits.len() as i64 + self.exponent;
            if point > 0 {
                return format!(
                    "{sign}{}.{}",
                    &digits[..point as usize],
                    &digits[point as usize..]
                );
            }
            return format!("{sign}0.{}{digits}", "0".repeat((-point) as usize));
        }
        let mantissa = if digits.len() == 1 {
            digits.to_owned()
        } else {
            format!("{}.{}", &digits[..1], &digits[1..])
        };
        format!(
            "{sign}{mantissa}E{}{adjusted}",
            if adjusted >= 0 { "+" } else { "" }
        )
    }
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecimalProjection {
    pub lower: DecimalText,
    pub upper: DecimalText,
    pub width: DecimalText,
    pub width_rounded: bool,
}

fn units(value: &Mpf, ceiling: bool) -> Result<IBig, ProjectionError> {
    if value.is_nan() || value.is_infinite() {
        return Err(ProjectionError::NonFinite);
    }
    let (sign, mantissa, exponent, _) = value.raw_tuple();
    if exponent.unsigned_abs() > MAX_SHIFT_BITS as u64 {
        return Err(ProjectionError::Shift(exponent.unsigned_abs()));
    }
    let mut numerator =
        IBig::from(mantissa) * "1000000000000000000000000000".parse::<IBig>().unwrap();
    if sign != 0 {
        numerator = -numerator;
    }
    if exponent >= 0 {
        return Ok(numerator << exponent as usize);
    }
    let denominator = IBig::from(UBig::ONE << (-exponent) as usize);
    Ok(if ceiling {
        -floor_div(&(-numerator), &denominator)
    } else {
        floor_div(&numerator, &denominator)
    })
}

fn rounded_width(coefficient: IBig, precision: u32) -> (DecimalText, bool) {
    let digits = coefficient.to_string();
    if digits.len() <= precision as usize {
        return (
            DecimalText {
                coefficient,
                exponent: -27,
            },
            false,
        );
    }
    let discarded = digits.len() - precision as usize;
    let denominator = format!("1{}", "0".repeat(discarded))
        .parse::<IBig>()
        .unwrap();
    let quotient = &coefficient / &denominator;
    let remainder = &coefficient - &quotient * &denominator;
    let twice = &remainder * IBig::from(2);
    let odd = &quotient % IBig::from(2) != IBig::ZERO;
    let mut rounded = if twice > denominator || (twice == denominator && odd) {
        quotient + IBig::ONE
    } else {
        quotient
    };
    let mut exponent = -27 + discarded as i64;
    if rounded.to_string().len() > precision as usize {
        rounded /= IBig::from(10);
        exponent += 1;
    }
    (
        DecimalText {
            coefficient: rounded,
            exponent,
        },
        remainder != IBig::ZERO,
    )
}

/// Разность целых узлов сетки 10^-27 с явным ROUND_HALF_EVEN.
/// Отрицательная ширина сохраняется для именованного отказа пустого окна.
pub fn width_from_units(
    lower: &IBig,
    upper: &IBig,
    precision: u32,
) -> Result<(DecimalText, bool), ProjectionError> {
    if !(1..=1_000_000).contains(&precision) {
        return Err(ProjectionError::Precision(precision));
    }
    let difference = upper - lower;
    let negative = difference < IBig::ZERO;
    let magnitude = if negative { -difference } else { difference };
    let (mut width, rounded) = rounded_width(magnitude, precision);
    if negative {
        width.coefficient = -width.coefficient;
    }
    Ok((width, rounded))
}

/// Профиль Decimal arithmetic context передаётся явно; текущий host default — 28.
pub fn project(
    value: &LegacyInterval,
    width_precision: u32,
) -> Result<DecimalProjection, ProjectionError> {
    if !(1..=1_000_000).contains(&width_precision) {
        return Err(ProjectionError::Precision(width_precision));
    }
    let lower = units(value.lower(), false)?;
    let upper = units(value.upper(), true)?;
    let (width, width_rounded) = width_from_units(&lower, &upper, width_precision)?;
    Ok(DecimalProjection {
        lower: DecimalText {
            coefficient: lower,
            exponent: -27,
        },
        upper: DecimalText {
            coefficient: upper,
            exponent: -27,
        },
        width,
        width_rounded,
    })
}
