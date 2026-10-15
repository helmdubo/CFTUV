//! Рациональный/двоичный legacy-слой: mpmath 1.3.0 `libmp/libmpf.py`, Python backend.
//! SHA256 источника: be93f490d56449c6c2568668809e166ad97823b09ed1de1dcc75637d57b9eeca.
//! Перенесены _normalize, from_rational, mpf_add/sub/mul/div/pow_int/sqrt.
//! Профиль прототипа ограничен; его отказы не являются продуктовыми исходами.
/*
Copyright (c) 2005-2021 Fredrik Johansson and mpmath contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
a. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
b. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
c. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
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

use std::cmp::Ordering;

use cftuv_radical::arith::{magnitude, signed};
use cftuv_radical::{Radical, Term};

use crate::{IBig, UBig};

// BitTest trait не реэкспортируется leaf-крейтом; длина берётся из его публичных integer words.
trait LegacyBitLen {
    fn bit_len(&self) -> usize;
}

impl LegacyBitLen for UBig {
    fn bit_len(&self) -> usize {
        let words = self.as_words();
        let Some(index) = words.iter().rposition(|word| *word != 0) else {
            return 0;
        };
        let width = std::mem::size_of_val(&words[index]) * 8;
        index * width + width - words[index].leading_zeros() as usize
    }
}

pub const MIN_PRECISION_BITS: u32 = 2;
pub const MAX_PRECISION_BITS: u32 = 1 << 20;
pub const MAX_INTERNAL_PRECISION_BITS: u32 = MAX_PRECISION_BITS + 256;
pub const MAX_MANTISSA_BITS: usize = 1 << 22;
pub const MAX_SHIFT_BITS: usize = 1 << 23;
pub const MAX_WORK_BITS: usize = 2 * MAX_MANTISSA_BITS + MAX_INTERNAL_PRECISION_BITS as usize;
pub const MAX_ABS_EXPONENT: i64 = 1 << 40;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UnsupportedProfile {
    Precision {
        requested: u32,
        minimum: u32,
        maximum: u32,
    },
    Exponent {
        requested: i64,
        maximum_absolute: i64,
    },
    ExponentArithmeticOverflow,
    Shift {
        requested: u64,
        maximum: usize,
    },
    MantissaBits {
        requested: usize,
        maximum: usize,
    },
    PowerExponent {
        requested: i64,
        minimum: i64,
        maximum: i64,
    },
    RawTuple {
        sign: u8,
        exponent: i64,
        bit_count: i64,
    },
    NaNIntervalEndpoint,
    ReversedInterval,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LegacyError {
    UnsupportedProfile(UnsupportedProfile),
    ZeroDivision,
    ComplexResult,
}

pub type LegacyResult<T> = Result<T, LegacyError>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rounding {
    Floor,
    Ceiling,
    TowardZero,
    AwayFromZero,
    Nearest,
}

impl Rounding {
    fn magnitude_down(self, negative: bool) -> bool {
        match self {
            Self::Floor => !negative,
            Self::Ceiling => negative,
            Self::TowardZero => true,
            Self::AwayFromZero => false,
            Self::Nearest => true,
        }
    }

    fn reciprocal(self) -> Self {
        match self {
            Self::Floor => Self::Ceiling,
            Self::Ceiling => Self::Floor,
            Self::TowardZero => Self::AwayFromZero,
            Self::AwayFromZero => Self::TowardZero,
            Self::Nearest => Self::Nearest,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Precision(u32);

impl Precision {
    pub fn new(bits: u32) -> LegacyResult<Self> {
        checked_precision(bits, MAX_PRECISION_BITS)
    }
    pub fn bits(self) -> u32 {
        self.0
    }
    pub(crate) fn extra(self, extra: u32) -> LegacyResult<Self> {
        let bits = self.0.checked_add(extra).ok_or_else(|| {
            profile(UnsupportedProfile::Precision {
                requested: u32::MAX,
                minimum: MIN_PRECISION_BITS,
                maximum: MAX_INTERNAL_PRECISION_BITS,
            })
        })?;
        checked_precision(bits, MAX_INTERNAL_PRECISION_BITS)
    }
}

fn checked_precision(bits: u32, maximum: u32) -> LegacyResult<Precision> {
    if !(MIN_PRECISION_BITS..=maximum).contains(&bits) {
        Err(profile(UnsupportedProfile::Precision {
            requested: bits,
            minimum: MIN_PRECISION_BITS,
            maximum,
        }))
    } else {
        Ok(Precision(bits))
    }
}

pub(crate) fn profile(reason: UnsupportedProfile) -> LegacyError {
    LegacyError::UnsupportedProfile(reason)
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Kind {
    Finite {
        negative: bool,
        mantissa: UBig,
        exponent: i64,
    },
    Infinity(bool),
    NaN,
}

/// Конечная каноническая мантисса нечётна или равна нулю; signed zero канонизируется в +zero.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Mpf {
    kind: Kind,
}

impl Mpf {
    pub fn zero() -> Self {
        Self {
            kind: Kind::Finite {
                negative: false,
                mantissa: UBig::ZERO,
                exponent: 0,
            },
        }
    }
    pub fn one() -> Self {
        Self {
            kind: Kind::Finite {
                negative: false,
                mantissa: UBig::ONE,
                exponent: 0,
            },
        }
    }
    pub fn positive_infinity() -> Self {
        Self {
            kind: Kind::Infinity(false),
        }
    }
    pub fn negative_infinity() -> Self {
        Self {
            kind: Kind::Infinity(true),
        }
    }
    pub fn nan() -> Self {
        Self { kind: Kind::NaN }
    }

    pub fn is_nan(&self) -> bool {
        matches!(self.kind, Kind::NaN)
    }
    pub fn is_zero(&self) -> bool {
        matches!(&self.kind, Kind::Finite { mantissa, .. } if mantissa.is_zero())
    }
    pub fn is_positive_infinity(&self) -> bool {
        matches!(self.kind, Kind::Infinity(false))
    }
    pub fn is_negative_infinity(&self) -> bool {
        matches!(self.kind, Kind::Infinity(true))
    }
    pub fn is_infinite(&self) -> bool {
        matches!(self.kind, Kind::Infinity(_))
    }

    pub fn sign(&self) -> i8 {
        match &self.kind {
            Kind::NaN => 0,
            Kind::Finite { mantissa, .. } if mantissa.is_zero() => 0,
            Kind::Finite { negative, .. } | Kind::Infinity(negative) => {
                if *negative {
                    -1
                } else {
                    1
                }
            }
        }
    }

    pub fn raw_tuple(&self) -> (u8, UBig, i64, i64) {
        match &self.kind {
            Kind::Finite {
                negative,
                mantissa,
                exponent,
            } => (
                u8::from(*negative),
                mantissa.clone(),
                *exponent,
                mantissa.bit_len() as i64,
            ),
            Kind::Infinity(false) => (0, UBig::ZERO, -456, -2),
            Kind::Infinity(true) => (1, UBig::ZERO, -789, -3),
            Kind::NaN => (0, UBig::ZERO, -123, -1),
        }
    }

    pub fn from_raw_tuple(
        sign: u8,
        mantissa: UBig,
        exponent: i64,
        bit_count: i64,
    ) -> LegacyResult<Self> {
        let invalid = || {
            profile(UnsupportedProfile::RawTuple {
                sign,
                exponent,
                bit_count,
            })
        };
        if sign > 1 {
            return Err(invalid());
        }
        if mantissa.is_zero() {
            return match (sign, exponent, bit_count) {
                (_, 0, 0) => Ok(Self::zero()),
                (0, -123, -1) => Ok(Self::nan()),
                (0, -456, -2) => Ok(Self::positive_infinity()),
                (1, -789, -3) => Ok(Self::negative_infinity()),
                _ => Err(invalid()),
            };
        }
        check_bits(mantissa.bit_len(), MAX_MANTISSA_BITS)?;
        check_exponent(exponent)?;
        if bit_count != mantissa.bit_len() as i64 || (&mantissa & UBig::ONE).is_zero() {
            return Err(invalid());
        }
        Ok(Self {
            kind: Kind::Finite {
                negative: sign != 0,
                mantissa,
                exponent,
            },
        })
    }

    pub fn from_man_exp(
        mantissa: IBig,
        exponent: i64,
        precision: Option<Precision>,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        let man = magnitude(&mantissa);
        normalize(
            mantissa < IBig::ZERO,
            man.clone(),
            exponent,
            man.bit_len(),
            precision,
            rounding,
        )
    }

    pub fn from_rational(
        numerator: IBig,
        denominator: IBig,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        let left = Self::from_man_exp(numerator, 0, None, Rounding::TowardZero)?;
        let right = Self::from_man_exp(denominator, 0, None, Rounding::TowardZero)?;
        left.div(&right, precision, rounding)
    }

    pub fn pos(&self, precision: Precision, rounding: Rounding) -> LegacyResult<Self> {
        match &self.kind {
            Kind::Finite {
                negative,
                mantissa,
                exponent,
            } => normalize(
                *negative,
                mantissa.clone(),
                *exponent,
                mantissa.bit_len(),
                Some(precision),
                rounding,
            ),
            _ => Ok(self.clone()),
        }
    }

    pub fn neg_exact(&self) -> Self {
        match &self.kind {
            Kind::Finite { mantissa, .. } if mantissa.is_zero() => self.clone(),
            Kind::Finite {
                negative,
                mantissa,
                exponent,
            } => Self {
                kind: Kind::Finite {
                    negative: !negative,
                    mantissa: mantissa.clone(),
                    exponent: *exponent,
                },
            },
            Kind::Infinity(negative) => Self {
                kind: Kind::Infinity(!negative),
            },
            Kind::NaN => self.clone(),
        }
    }

    pub fn neg(&self, precision: Precision, rounding: Rounding) -> LegacyResult<Self> {
        self.neg_exact().pos(precision, rounding)
    }

    /// Конвенция libmpf cmp, включая NaN; конструктор интервала отвергает NaN.
    pub fn compare(&self, other: &Self) -> Ordering {
        if self.is_zero() {
            return 0i8.cmp(&other.sign());
        }
        if other.is_zero() {
            return self.sign().cmp(&0);
        }
        if self == other {
            return Ordering::Equal;
        }
        if self.is_nan() || other.is_nan() || self.is_infinite() || other.is_infinite() {
            return if other.is_nan() || self.is_positive_infinity() || other.is_negative_infinity()
            {
                Ordering::Greater
            } else {
                Ordering::Less
            };
        }
        let (s, sm, se, sbc) = self.raw_tuple();
        let (t, tm, te, tbc) = other.raw_tuple();
        if s != t {
            return if s == 0 {
                Ordering::Greater
            } else {
                Ordering::Less
            };
        }
        let high = (se + sbc).cmp(&(te + tbc));
        let magnitude_order = if high != Ordering::Equal {
            high
        } else if se >= te {
            (sm << (se - te) as usize).cmp(&tm)
        } else {
            sm.cmp(&(tm << (te - se) as usize))
        };
        if s == 1 {
            magnitude_order.reverse()
        } else {
            magnitude_order
        }
    }

    pub fn add(
        &self,
        other: &Self,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        self.add_internal(other, Some(precision), rounding)
    }
    pub fn sub(
        &self,
        other: &Self,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        self.add_internal(&other.neg_exact(), Some(precision), rounding)
    }

    fn add_internal(
        &self,
        other: &Self,
        precision: Option<Precision>,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        if self.is_nan() || other.is_nan() {
            return Ok(Self::nan());
        }
        if self.is_infinite() || other.is_infinite() {
            if self.is_infinite() && other.is_infinite() && self != other {
                return Ok(Self::nan());
            }
            return Ok(if self.is_infinite() {
                self.clone()
            } else {
                other.clone()
            });
        }
        if self.is_zero() {
            return match precision {
                Some(p) => other.pos(p, rounding),
                None => Ok(other.clone()),
            };
        }
        if other.is_zero() {
            return match precision {
                Some(p) => self.pos(p, rounding),
                None => Ok(self.clone()),
            };
        }
        let (s, sm, se, sbc) = self.raw_tuple();
        let (t, tm, te, tbc) = other.raw_tuple();
        let offset = se - te;
        if let Some(p) = precision {
            // Большой разрыв экспонент в libmpf mpf_add: возмущение на precision+4 заменяет огромный exact shift.
            let gap = (se + sbc) - (te + tbc);
            if offset.abs() > 100
                && gap.abs() > i64::from(p.bits()) + 4
                && gap.signum() == offset.signum()
            {
                let (negative, man, exp) = if offset > 0 {
                    (s != 0, sm, se)
                } else {
                    (t != 0, tm, te)
                };
                let shift = p.bits() as usize + 4;
                let enlarged = checked_shift(&man, shift)?;
                let perturbed = if s == t {
                    enlarged + UBig::ONE
                } else {
                    enlarged - UBig::ONE
                };
                return normalize(
                    negative,
                    perturbed.clone(),
                    exponent_add(exp, -(shift as i64))?,
                    perturbed.bit_len(),
                    Some(p),
                    rounding,
                );
            }
        }
        let base = se.min(te);
        let left = checked_shift(&sm, (se - base) as usize)?;
        let right = checked_shift(&tm, (te - base) as usize)?;
        let total = if s == 1 {
            -signed(&left)
        } else {
            signed(&left)
        } + if t == 1 {
            -signed(&right)
        } else {
            signed(&right)
        };
        Self::from_man_exp(total, base, precision, rounding)
    }

    pub fn mul(
        &self,
        other: &Self,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        self.mul_internal(other, Some(precision), rounding)
    }
    pub(crate) fn mul_exact(&self, other: &Self) -> LegacyResult<Self> {
        self.mul_internal(other, None, Rounding::TowardZero)
    }
    fn mul_internal(
        &self,
        other: &Self,
        precision: Option<Precision>,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        if self.is_nan() || other.is_nan() {
            return Ok(Self::nan());
        }
        if self.is_infinite() || other.is_infinite() {
            if self.is_zero() || other.is_zero() {
                return Ok(Self::nan());
            }
            return Ok(if self.sign() * other.sign() < 0 {
                Self::negative_infinity()
            } else {
                Self::positive_infinity()
            });
        }
        if self.is_zero() || other.is_zero() {
            return Ok(Self::zero());
        }
        let (s, sm, se, _) = self.raw_tuple();
        let (t, tm, te, _) = other.raw_tuple();
        check_bits(sm.bit_len() + tm.bit_len(), MAX_WORK_BITS)?;
        let man = sm * tm;
        normalize(
            s != t,
            man.clone(),
            exponent_add(se, te)?,
            man.bit_len(),
            precision,
            rounding,
        )
    }

    pub fn square(&self, precision: Precision, rounding: Rounding) -> LegacyResult<Self> {
        self.mul(self, precision, rounding)
    }

    pub fn div(
        &self,
        other: &Self,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        if other.is_zero() {
            return Err(LegacyError::ZeroDivision);
        }
        if self.is_zero() {
            return Ok(if other.is_nan() {
                Self::nan()
            } else {
                Self::zero()
            });
        }
        if self.is_nan() || other.is_nan() || (self.is_infinite() && other.is_infinite()) {
            return Ok(Self::nan());
        }
        if self.is_infinite() {
            return Ok(if self.sign() * other.sign() < 0 {
                Self::negative_infinity()
            } else {
                Self::positive_infinity()
            });
        }
        if other.is_infinite() {
            return Ok(Self::zero());
        }
        let (s, sm, se, sbc) = self.raw_tuple();
        let (t, tm, te, tbc) = other.raw_tuple();
        let exp = exponent_add(se, -te)?;
        if tm.is_one() {
            return normalize(s != t, sm, exp, sbc as usize, Some(precision), rounding);
        }
        let mut extra = (i64::from(precision.bits()) - sbc + tbc + 5).max(5) as usize;
        let numerator = checked_shift(&sm, extra)?;
        let mut quotient = &numerator / &tm;
        if &quotient * &tm != numerator {
            quotient = (quotient << 1usize) + UBig::ONE;
            extra += 1;
        }
        normalize(
            s != t,
            quotient.clone(),
            exponent_add(exp, -(extra as i64))?,
            quotient.bit_len(),
            Some(precision),
            rounding,
        )
    }

    pub fn sqrt(&self, precision: Precision, rounding: Rounding) -> LegacyResult<Self> {
        if self.sign() < 0 {
            return Err(LegacyError::ComplexResult);
        }
        if self.is_zero() || self.is_nan() || self.is_positive_infinity() {
            return Ok(self.clone());
        }
        let (_, mut man, mut exp, mut bc) = self.raw_tuple();
        if exp & 1 != 0 {
            exp -= 1;
            man <<= 1usize;
            bc += 1;
        } else if man.is_one() {
            return normalize(false, man, exp / 2, bc as usize, Some(precision), rounding);
        }
        let mut shift = (2 * i64::from(precision.bits()) - bc + 4).max(4) as usize;
        shift += shift & 1;
        let radicand = checked_shift(&man, shift)?;
        let mut root = integer_sqrt(&radicand);
        if !matches!(rounding, Rounding::Floor | Rounding::TowardZero) && &root * &root != radicand
        {
            root = (root << 1usize) + UBig::ONE;
            shift += 2;
        }
        normalize(
            false,
            root.clone(),
            exponent_add(exp, -(shift as i64))? / 2,
            root.bit_len(),
            Some(precision),
            rounding,
        )
    }

    pub fn pow_int(
        &self,
        exponent: i64,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        let n = i32::try_from(exponent).map_err(|_| {
            profile(UnsupportedProfile::PowerExponent {
                requested: exponent,
                minimum: i64::from(i32::MIN),
                maximum: i64::from(i32::MAX),
            })
        })?;
        if self.is_nan() {
            return Ok(Self::nan());
        }
        if self.is_infinite() {
            return Ok(if n == 0 {
                Self::nan()
            } else if n < 0 {
                Self::zero()
            } else if self.sign() < 0 && n & 1 != 0 {
                Self::negative_infinity()
            } else {
                Self::positive_infinity()
            });
        }
        if n == 0 {
            return Ok(Self::one());
        }
        if n == 1 {
            return self.pos(precision, rounding);
        }
        if n == -1 {
            return Self::one().div(self, precision, rounding);
        }
        if n < 0 {
            // Отрицательная степень libmpf: prec+5 и reciprocal_rnd; отдельный mpi-путь использует prec+20.
            let inverse =
                self.pow_positive(n.unsigned_abs(), precision.extra(5)?, rounding.reciprocal())?;
            return Self::one().div(&inverse, precision, rounding);
        }
        self.pow_positive(n as u32, precision, rounding)
    }

    pub(crate) fn pow_nonnegative(
        &self,
        n: u32,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        if self.is_nan() {
            return Ok(Self::nan());
        }
        if self.is_infinite() {
            return Ok(if n == 0 {
                Self::nan()
            } else if self.sign() < 0 && n & 1 != 0 {
                Self::negative_infinity()
            } else {
                Self::positive_infinity()
            });
        }
        if n == 0 {
            return Ok(Self::one());
        }
        if n == 1 {
            return self.pos(precision, rounding);
        }
        self.pow_positive(n, precision, rounding)
    }

    fn pow_positive(
        &self,
        mut n: u32,
        precision: Precision,
        rounding: Rounding,
    ) -> LegacyResult<Self> {
        if self.is_zero() {
            return Ok(Self::zero());
        }
        if n == 2 {
            return self.square(precision, rounding);
        }
        let (s, mut man, mut exp, mut bc_raw) = self.raw_tuple();
        let negative = s == 1 && n & 1 != 0;
        let final_exp = exp
            .checked_mul(i64::from(n))
            .ok_or_else(|| profile(UnsupportedProfile::ExponentArithmeticOverflow))?;
        if man.is_one() {
            return normalize(negative, man, final_exp, 1, None, rounding);
        }
        if (bc_raw as u64) * u64::from(n) < 1000 {
            let man = integer_power(&man, n);
            return normalize(
                negative,
                man.clone(),
                final_exp,
                man.bit_len(),
                Some(precision),
                rounding,
            );
        }
        // Двоичная степень libmpf: сохраняются порядок операций и временный учёт bc после округления.
        let n_bits = u32::BITS - n.leading_zeros();
        let work_precision = precision.extra(4 * n_bits + 4)?.bits() as usize;
        let rounds_down = rounding == Rounding::Nearest || rounding.magnitude_down(negative);
        let mut result_man = UBig::ONE;
        let mut result_exp = 0i64;
        let mut result_bc = 1usize;
        let mut bc = bc_raw as usize;
        loop {
            if n & 1 != 0 {
                result_man *= &man;
                result_exp = exponent_add(result_exp, exp)?;
                let estimate = result_bc + bc - 2;
                result_bc = estimate + (&result_man >> estimate).bit_len();
                if result_bc > work_precision {
                    let cut = result_bc - work_precision;
                    result_man = cut_magnitude(&result_man, cut, rounds_down);
                    result_exp = exponent_add(result_exp, cut as i64)?;
                    result_bc = work_precision;
                }
                n -= 1;
                if n == 0 {
                    break;
                }
            }
            man = &man * &man;
            exp = exponent_add(exp, exp)?;
            let estimate = 2 * bc - 2;
            bc = estimate + (&man >> estimate).bit_len();
            if bc > work_precision {
                let cut = bc - work_precision;
                man = cut_magnitude(&man, cut, rounds_down);
                exp = exponent_add(exp, cut as i64)?;
                bc = work_precision;
            }
            n /= 2;
        }
        bc_raw = result_bc as i64;
        normalize(
            negative,
            result_man,
            result_exp,
            bc_raw as usize,
            Some(precision),
            rounding,
        )
    }
}

fn normalize(
    negative: bool,
    mut man: UBig,
    mut exp: i64,
    bc: usize,
    precision: Option<Precision>,
    rounding: Rounding,
) -> LegacyResult<Mpf> {
    if man.is_zero() {
        return Ok(Mpf::zero());
    }
    check_bits(man.bit_len(), MAX_WORK_BITS)?;
    if let Some(p) = precision {
        if bc > p.bits() as usize {
            let cut = bc - p.bits() as usize;
            man = if rounding == Rounding::Nearest {
                nearest(&man, cut)
            } else {
                cut_magnitude(&man, cut, rounding.magnitude_down(negative))
            };
            exp = exponent_add(exp, cut as i64)?;
        }
    }
    let trailing = trailing_bits(&man);
    man >>= trailing;
    exp = exponent_add(exp, trailing as i64)?;
    check_exponent(exp)?;
    check_bits(man.bit_len(), MAX_MANTISSA_BITS)?;
    Ok(Mpf {
        kind: Kind::Finite {
            negative,
            mantissa: man,
            exponent: exp,
        },
    })
}

fn nearest(man: &UBig, cut: usize) -> UBig {
    let quotient = man >> cut;
    let remainder = man - (&quotient << cut);
    let halfway = UBig::ONE << (cut - 1);
    if remainder > halfway || (remainder == halfway && !(&quotient & UBig::ONE).is_zero()) {
        quotient + UBig::ONE
    } else {
        quotient
    }
}

fn cut_magnitude(man: &UBig, cut: usize, down: bool) -> UBig {
    let quotient = man >> cut;
    if !down && (&quotient << cut) != *man {
        quotient + UBig::ONE
    } else {
        quotient
    }
}

fn trailing_bits(man: &UBig) -> usize {
    let mut bits = 0;
    for word in man.as_words() {
        if *word == 0 {
            bits += std::mem::size_of_val(word) * 8;
        } else {
            return bits + word.trailing_zeros() as usize;
        }
    }
    bits
}

fn checked_shift(man: &UBig, shift: usize) -> LegacyResult<UBig> {
    if shift > MAX_SHIFT_BITS {
        return Err(profile(UnsupportedProfile::Shift {
            requested: shift as u64,
            maximum: MAX_SHIFT_BITS,
        }));
    }
    check_bits(man.bit_len().saturating_add(shift), MAX_WORK_BITS)?;
    Ok(man << shift)
}

fn check_bits(bits: usize, maximum: usize) -> LegacyResult<()> {
    if bits > maximum {
        Err(profile(UnsupportedProfile::MantissaBits {
            requested: bits,
            maximum,
        }))
    } else {
        Ok(())
    }
}

fn check_exponent(exp: i64) -> LegacyResult<()> {
    if exp.unsigned_abs() > MAX_ABS_EXPONENT as u64 {
        Err(profile(UnsupportedProfile::Exponent {
            requested: exp,
            maximum_absolute: MAX_ABS_EXPONENT,
        }))
    } else {
        Ok(())
    }
}

fn exponent_add(left: i64, right: i64) -> LegacyResult<i64> {
    left.checked_add(right)
        .ok_or_else(|| profile(UnsupportedProfile::ExponentArithmeticOverflow))
}

fn integer_power(value: &UBig, mut n: u32) -> UBig {
    let mut base = value.clone();
    let mut result = UBig::ONE;
    while n > 0 {
        if n & 1 != 0 {
            result *= &base;
        }
        n >>= 1;
        if n > 0 {
            base = &base * &base;
        }
    }
    result
}

fn integer_sqrt(value: &UBig) -> UBig {
    Radical::floor_root(&Term {
        radicand: value.clone(),
        numerator: IBig::ONE,
        denominator: UBig::ONE,
    })
}
