//! Ограниченный source-port mpmath 1.3.0 Python backend, без product admission.
//! libelefun.py SHA256: 8e80593f814e7713df89e5aca352cfb52afa747c9da46fcb42217f6d841858c8.
//! libmpi.py SHA256: bb4239122c24a9afb8f9d5c44e2e64eccb9ac417996ef0803c5b65f7753d605d.
//! mod_pi2, mpf_cos_sin (обычный argument), cos_sin_quadrant, mpi_cos_sin.
//! Используются только ранее проверенные finite π/basecase; atan отсутствует.
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

use crate::legacy_interval::LegacyInterval;
use crate::legacy_mpf::{LegacyError, Mpf, Precision, Rounding, MAX_SHIFT_BITS};
use crate::legacy_trig::{pi_fixed, TrigError};
use crate::legacy_trig_basecase::{cos_sin_basecase, BasecaseError};
use crate::{IBig, UBig};

pub const MIN_PRECISION_BITS: u32 = 2;
pub const MAX_PRECISION_BITS: u32 = 512;
pub const MAX_ABSOLUTE_ARGUMENT: u32 = 8;
pub const MIN_REDUCTION_WORK_BITS: u32 = 10;
pub const MAX_REDUCTION_WORK_BITS: u32 = 400;

/// Это внутренний профиль прототипа; ограничения не заменяют old UNDECIDABLE.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UnsupportedProfile {
    Precision {
        requested: u32,
        minimum: u32,
        maximum: u32,
    },
    Argument {
        sign: u8,
        mantissa: UBig,
        exponent: i64,
        maximum_integer: u32,
    },
    ReductionWorkBits {
        requested: u64,
        minimum: u32,
        maximum: u32,
    },
    NonFiniteReduction {
        sign: u8,
        exponent: i64,
        bit_count: i64,
    },
    NaNIntervalEndpoint,
    Shift {
        requested: u64,
        maximum: usize,
    },
    ArithmeticOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvalError {
    UnsupportedProfile(UnsupportedProfile),
    LegacyArithmetic(LegacyError),
    Pi(TrigError),
    Basecase(BasecaseError),
}
pub type EvalResult<T> = Result<T, EvalError>;

impl From<LegacyError> for EvalError {
    fn from(value: LegacyError) -> Self {
        Self::LegacyArithmetic(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Reduction {
    pub fixed_argument: UBig,
    pub quadrant: i64,
    pub work_bits: u32,
    // Диагностика количества source cancellation-расширений, не Work-бюджет.
    pub cancellation_steps: u32,
}

fn unsupported(reason: UnsupportedProfile) -> EvalError {
    EvalError::UnsupportedProfile(reason)
}

fn check_precision(precision: Precision) -> EvalResult<()> {
    let bits = precision.bits();
    if !(MIN_PRECISION_BITS..=MAX_PRECISION_BITS).contains(&bits) {
        Err(unsupported(UnsupportedProfile::Precision {
            requested: bits,
            minimum: MIN_PRECISION_BITS,
            maximum: MAX_PRECISION_BITS,
        }))
    } else {
        Ok(())
    }
}

fn checked_work(bits: u64) -> EvalResult<u32> {
    if !(u64::from(MIN_REDUCTION_WORK_BITS)..=u64::from(MAX_REDUCTION_WORK_BITS)).contains(&bits) {
        Err(unsupported(UnsupportedProfile::ReductionWorkBits {
            requested: bits,
            minimum: MIN_REDUCTION_WORK_BITS,
            maximum: MAX_REDUCTION_WORK_BITS,
        }))
    } else {
        Ok(bits as u32)
    }
}

fn check_argument(value: &Mpf) -> EvalResult<()> {
    let absolute = if value.sign() < 0 {
        value.neg_exact()
    } else {
        value.clone()
    };
    let eight = Mpf::from_man_exp(
        IBig::from(MAX_ABSOLUTE_ARGUMENT),
        0,
        None,
        Rounding::TowardZero,
    )?;
    if absolute.compare(&eight).is_gt() {
        let (sign, mantissa, exponent, _) = value.raw_tuple();
        Err(unsupported(UnsupportedProfile::Argument {
            sign,
            mantissa,
            exponent,
            maximum_integer: MAX_ABSOLUTE_ARGUMENT,
        }))
    } else {
        Ok(())
    }
}

fn shifted(value: &UBig, offset: i64) -> EvalResult<UBig> {
    let count = offset.unsigned_abs();
    if count > MAX_SHIFT_BITS as u64 {
        return Err(unsupported(UnsupportedProfile::Shift {
            requested: count,
            maximum: MAX_SHIFT_BITS,
        }));
    }
    Ok(if offset >= 0 {
        value << count as usize
    } else {
        value >> count as usize
    })
}

/// Source mod_pi2 принимает absolute mantissa даже для negative input.
/// Caller получает finite work-отказ отдельно от продуктового исхода.
pub fn mod_pi2(value: &Mpf, work_bits: u32) -> EvalResult<Reduction> {
    checked_work(u64::from(work_bits))?;
    let (sign, man, exp, bc) = value.raw_tuple();
    if value.is_nan() || value.is_infinite() {
        return Err(unsupported(UnsupportedProfile::NonFiniteReduction {
            sign,
            exponent: exp,
            bit_count: bc,
        }));
    }
    check_argument(value)?;
    let mag = exp + bc;
    if mag > 0 {
        let mut i = 0u32;
        loop {
            let cancellation = 20u64
                .checked_shl(i)
                .ok_or_else(|| unsupported(UnsupportedProfile::ArithmeticOverflow))?;
            let wp = checked_work(u64::from(work_bits) + cancellation)?;
            let wpmod = u64::from(wp) + mag as u64;
            let pi2 = pi_fixed((wpmod - 1) as u32).map_err(EvalError::Pi)?;
            let pi4 = &pi2 >> 1usize;
            let t = shifted(&man, exp + wpmod as i64)?;
            let n = &t / &pi2;
            let y = &t % &pi2;
            let small = if y > pi4 { &pi2 - &y } else { y.clone() };
            if !(&small >> (i64::from(work_bits) + mag - 10) as usize).is_zero() {
                return Ok(Reduction {
                    fixed_argument: y >> mag as usize,
                    quadrant: i64::try_from(n)
                        .map_err(|_| unsupported(UnsupportedProfile::ArithmeticOverflow))?,
                    work_bits: wp,
                    cancellation_steps: i,
                });
            }
            i += 1;
        }
    } else {
        let wp = checked_work(u64::from(work_bits) + mag.unsigned_abs())?;
        Ok(Reduction {
            fixed_argument: shifted(&man, exp + i64::from(wp))?,
            quadrant: 0,
            work_bits: wp,
            cancellation_steps: 0,
        })
    }
}

use crate::legacy_fixed::perturb;

fn scalar_core(value: &Mpf, precision: Precision, rounding: Rounding) -> EvalResult<(Mpf, Mpf)> {
    if value.is_zero() {
        return Ok((Mpf::one(), Mpf::zero()));
    }
    if value.is_nan() || value.is_infinite() {
        return Ok((Mpf::nan(), Mpf::nan()));
    }
    check_argument(value)?;
    let (sign, _, exp, bc) = value.raw_tuple();
    let mag = exp + bc;
    let wp = precision.bits() + 10;
    if mag < -i64::from(wp) {
        return Ok((
            perturb(&Mpf::one(), true, precision, rounding)?,
            perturb(value, sign == 0, precision, rounding)?,
        ));
    }
    let reduced = mod_pi2(value, wp)?;
    let (mut cos, mut sin) =
        cos_sin_basecase(&IBig::from(reduced.fixed_argument), reduced.work_bits)
            .map_err(EvalError::Basecase)?;
    (cos, sin) = quadrant_pair(cos, sin, reduced.quadrant);
    if sign != 0 {
        sin = -sin;
    }
    let exp = -i64::from(reduced.work_bits);
    Ok((
        Mpf::from_man_exp(cos, exp, Some(precision), rounding)?,
        Mpf::from_man_exp(sin, exp, Some(precision), rounding)?,
    ))
}

fn quadrant_pair(cos: IBig, sin: IBig, n: i64) -> (IBig, IBig) {
    match n.rem_euclid(4) {
        0 => (cos, sin),
        1 => (-sin, cos),
        2 => (-cos, -sin),
        3 => (sin, -cos),
        _ => unreachable!(),
    }
}

pub fn mpf_cos_sin(
    value: &Mpf,
    precision: Precision,
    rounding: Rounding,
) -> EvalResult<(Mpf, Mpf)> {
    check_precision(precision)?;
    scalar_core(value, precision, rounding)
}
pub fn mpf_cos(value: &Mpf, precision: Precision, rounding: Rounding) -> EvalResult<Mpf> {
    Ok(mpf_cos_sin(value, precision, rounding)?.0)
}
pub fn mpf_sin(value: &Mpf, precision: Precision, rounding: Rounding) -> EvalResult<Mpf> {
    Ok(mpf_cos_sin(value, precision, rounding)?.1)
}

fn interval_quadrant(value: &Mpf, precision: Precision) -> EvalResult<(Mpf, Mpf, i64)> {
    if value.is_zero() {
        return Ok((Mpf::one(), Mpf::zero(), 0));
    }
    let (cos, sin) = scalar_core(value, precision, Rounding::TowardZero)?;
    let mut n = mod_pi2(value, 15)?.quadrant;
    if value.sign() < 0 {
        n = -1 - n;
    }
    Ok((cos, sin, n))
}

fn min_max(left: Mpf, right: Mpf) -> (Mpf, Mpf) {
    if left.compare(&right).is_le() {
        (left, right)
    } else {
        (right, left)
    }
}

fn finalize(
    value: Mpf,
    more: &Mpf,
    less: &Mpf,
    precision: Precision,
    rounding: Rounding,
) -> EvalResult<Mpf> {
    let negative = value.sign() < 0;
    let multiplier = if negative == (rounding == Rounding::Floor) {
        more
    } else {
        less
    };
    let answer = value.mul(multiplier, precision, rounding)?;
    let (sign, _, exp, bc) = answer.raw_tuple();
    if exp + bc >= 1 {
        Ok(if sign != 0 {
            Mpf::one().neg_exact()
        } else {
            Mpf::one()
        })
    } else {
        Ok(answer)
    }
}

pub fn mpi_cos_sin(
    value: &LegacyInterval,
    precision: Precision,
) -> EvalResult<(LegacyInterval, LegacyInterval)> {
    check_precision(precision)?;
    let a = value.lower();
    let b = value.upper();
    if a.is_nan() || b.is_nan() {
        return Err(unsupported(UnsupportedProfile::NaNIntervalEndpoint));
    }
    if a.is_zero() && b.is_zero() {
        return Ok((LegacyInterval::one(), LegacyInterval::zero()));
    }
    if a.is_infinite() || b.is_infinite() {
        let full = LegacyInterval::new(Mpf::one().neg_exact(), Mpf::one())?;
        return Ok((full.clone(), full));
    }
    let wp = precision.bits() + 20;
    let internal_precision = Precision::new(wp)?;
    let (ca, sa, na) = interval_quadrant(a, internal_precision)?;
    let (cb, sb, nb) = interval_quadrant(b, internal_precision)?;
    let (mut ca, mut cb) = min_max(ca, cb);
    let (mut sa, mut sb) = min_max(sa, sb);
    if na != nb {
        if nb - na >= 4 {
            let full = LegacyInterval::new(Mpf::one().neg_exact(), Mpf::one())?;
            return Ok((full.clone(), full));
        }
        if na.div_euclid(4) != nb.div_euclid(4) {
            cb = Mpf::one();
        }
        if (na - 2).div_euclid(4) != (nb - 2).div_euclid(4) {
            ca = Mpf::one().neg_exact();
        }
        if (na - 1).div_euclid(4) != (nb - 1).div_euclid(4) {
            sb = Mpf::one();
        }
        if (na - 3).div_euclid(4) != (nb - 3).div_euclid(4) {
            sa = Mpf::one().neg_exact();
        }
    }
    let one = IBig::ONE << wp as usize;
    let step = IBig::ONE << 10usize;
    let more = Mpf::from_man_exp(&one + &step, -i64::from(wp), None, Rounding::TowardZero)?;
    let less = Mpf::from_man_exp(&one - &step, -i64::from(wp), None, Rounding::TowardZero)?;
    let ca = finalize(ca, &more, &less, precision, Rounding::Floor)?;
    let cb = finalize(cb, &more, &less, precision, Rounding::Ceiling)?;
    let sa = finalize(sa, &more, &less, precision, Rounding::Floor)?;
    let sb = finalize(sb, &more, &less, precision, Rounding::Ceiling)?;
    Ok((LegacyInterval::new(ca, cb)?, LegacyInterval::new(sa, sb)?))
}
pub fn mpi_cos(value: &LegacyInterval, precision: Precision) -> EvalResult<LegacyInterval> {
    Ok(mpi_cos_sin(value, precision)?.0)
}
pub fn mpi_sin(value: &LegacyInterval, precision: Precision) -> EvalResult<LegacyInterval> {
    Ok(mpi_cos_sin(value, precision)?.1)
}
