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
//! Ограниченный integer-only source port atan/atan2 из libelefun/libmpi 1.3.0.
//! Immutable atan(n/128) floor at532 заменяет конечную таблицу Newton.
//! Допуск Density и произвольная история внешнего legacy cache здесь не моделируются.
use crate::legacy_atan_table::ATAN_FLOOR_532;
use crate::legacy_fixed::{floor_div, floor_shift, perturb};
use crate::legacy_interval::LegacyInterval;
use crate::legacy_mpf::{LegacyError, Mpf, Precision, Rounding, MAX_SHIFT_BITS};
use crate::legacy_trig::{mpf_pi, pi_fixed, TrigError};
use crate::{IBig, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AtanError {
    Precision(u32),
    WorkPrecision(u64),
    AnchorIndex(IBig),
    Shift(u64),
    Legacy(LegacyError),
    Pi(TrigError),
}
impl From<LegacyError> for AtanError {
    fn from(e: LegacyError) -> Self {
        Self::Legacy(e)
    }
}
impl From<TrigError> for AtanError {
    fn from(e: TrigError) -> Self {
        Self::Pi(e)
    }
}
pub type AtanResult<T> = Result<T, AtanError>;

fn check(p: Precision) -> AtanResult<()> {
    if p.bits() <= 512 {
        Ok(())
    } else {
        Err(AtanError::Precision(p.bits()))
    }
}
fn negative_rounding(r: Rounding) -> Rounding {
    match r {
        Rounding::Floor => Rounding::Ceiling,
        Rounding::Ceiling => Rounding::Floor,
        _ => r,
    }
}
fn shifted_mpf(x: &Mpf, shift: i64) -> AtanResult<Mpf> {
    let (s, m, e, b) = x.raw_tuple();
    if x.is_zero() || x.is_infinite() || x.is_nan() {
        return Ok(x.clone());
    }
    Ok(Mpf::from_raw_tuple(s, m, e + shift, b)?)
}
fn fixed(x: &Mpf, p: u32) -> AtanResult<IBig> {
    let (s, m, e, _) = x.raw_tuple();
    let offset = e + i64::from(p);
    if offset.unsigned_abs() > MAX_SHIFT_BITS as u64 {
        return Err(AtanError::Shift(offset.unsigned_abs()));
    }
    let v = if s == 0 {
        IBig::from(m)
    } else {
        -IBig::from(m)
    };
    Ok(if offset >= 0 {
        v << offset as usize
    } else {
        floor_shift(&v, (-offset) as usize)
    })
}

/// Source prec2=(next_power_of_two(prec))+20; lower exact floors наследуются shift.
pub fn atan_cached(n: usize, p: u32) -> AtanResult<(IBig, IBig)> {
    if !(8..=512).contains(&p) {
        return Err(AtanError::WorkPrecision(u64::from(p)));
    }
    if n >= 256 {
        return Err(AtanError::AnchorIndex(IBig::from(n)));
    }
    let p2 = p.next_power_of_two() + 20;
    let a = IBig::from(n) << ((p2 - 7) as usize);
    let prefix = UBig::from_str_radix(ATAN_FLOOR_532[n], 16).expect("проверенная таблица atan");
    let cached = IBig::from(prefix >> ((532 - p2) as usize));
    Ok((a >> ((p2 - p) as usize), cached >> ((p2 - p) as usize)))
}

pub fn atan_taylor(x: &IBig, p: u32) -> AtanResult<IBig> {
    if !(8..=512).contains(&p) {
        return Err(AtanError::WorkPrecision(u64::from(p)));
    }
    let n = floor_shift(x, (p - 7) as usize);
    let index = usize::try_from(n.clone()).map_err(|_| AtanError::AnchorIndex(n.clone()))?;
    let (a, atan_a) = atan_cached(index, p)?;
    let d = x - &a;
    let denominator = floor_shift(&(&a * &a), p as usize)
        + floor_shift(&(&a * &d), p as usize)
        + (IBig::ONE << p as usize);
    let mut v = floor_div(&(&d << p as usize), &denominator);
    let mut s0 = v.clone();
    let v2 = floor_shift(&(&v * &v), p as usize);
    let v4 = floor_shift(&(&v2 * &v2), p as usize);
    let mut s1 = floor_div(&v, &IBig::from(3));
    v = floor_shift(&(&v * &v4), p as usize);
    let mut k = 5usize;
    while v != IBig::ZERO {
        s0 += floor_div(&v, &IBig::from(k));
        k += 2;
        s1 += floor_div(&v, &IBig::from(k));
        v = floor_shift(&(&v * &v4), p as usize);
        k += 2;
    }
    s1 = floor_shift(&(&s1 * &v2), p as usize);
    Ok(atan_a + s0 - s1)
}

fn infinity(negative: bool, p: Precision, r: Rounding) -> AtanResult<Mpf> {
    let result = shifted_mpf(
        &mpf_pi(p, if negative { negative_rounding(r) } else { r })?,
        -1,
    )?;
    Ok(if negative { result.neg_exact() } else { result })
}
pub fn mpf_atan(x: &Mpf, p: Precision, r: Rounding) -> AtanResult<Mpf> {
    check(p)?;
    if x.is_zero() {
        return Ok(Mpf::zero());
    }
    if x.is_infinite() {
        return infinity(x.sign() < 0, p, r);
    }
    if x.is_nan() {
        return Ok(Mpf::nan());
    }
    let (s, _, e, b) = x.raw_tuple();
    let mag = e + b;
    if mag > i64::from(p.bits()) + 20 {
        return infinity(s != 0, p, r);
    }
    if -mag > i64::from(p.bits()) + 20 {
        return Ok(perturb(x, s == 0, p, r)?);
    }
    let wp = u64::from(p.bits()) + 30 + mag.unsigned_abs();
    if wp > 512 {
        return Err(AtanError::WorkPrecision(wp));
    }
    let wp = wp as u32;
    let reciprocal = mag >= 2;
    let value = if reciprocal {
        Mpf::one().div(x, Precision::new(wp)?, Rounding::TowardZero)?
    } else {
        x.clone()
    };
    let mut t = fixed(&value, wp)?;
    if s != 0 {
        t = -t;
    }
    let mut a = atan_taylor(&t, wp)?;
    if reciprocal {
        a = IBig::from(pi_fixed(wp)? >> 1usize) + IBig::ONE - a;
    }
    if s != 0 {
        a = -a;
    }
    Ok(Mpf::from_man_exp(a, -i64::from(wp), Some(p), r)?)
}
pub fn mpf_atan2(y: &Mpf, x: &Mpf, p: Precision, r: Rounding) -> AtanResult<Mpf> {
    check(p)?;
    if y.is_zero() || y.is_infinite() || y.is_nan() {
        if y.is_zero() && !x.is_nan() {
            return if x.sign() >= 0 {
                Ok(Mpf::zero())
            } else {
                Ok(mpf_pi(p, r)?)
            };
        }
        if y.is_infinite() {
            return if x.is_infinite() {
                Ok(Mpf::nan())
            } else {
                infinity(y.sign() < 0, p, r)
            };
        }
        return Ok(Mpf::nan());
    }
    if y.sign() < 0 {
        return Ok(mpf_atan2(&y.neg_exact(), x, p, negative_rounding(r))?.neg_exact());
    }
    if x.is_zero() || x.is_infinite() || x.is_nan() {
        if x.is_nan() {
            return Ok(Mpf::nan());
        }
        if x.is_positive_infinity() {
            return Ok(Mpf::zero());
        }
        if x.is_negative_infinity() {
            return Ok(mpf_pi(p, r)?);
        }
        return infinity(false, p, r);
    }
    let extra = Precision::new(p.bits() + 4)?;
    let quotient = y.div(x, extra, Rounding::TowardZero)?;
    let angle = mpf_atan(&quotient, extra, Rounding::TowardZero)?;
    if x.sign() < 0 {
        Ok(mpf_pi(extra, Rounding::TowardZero)?.add(&angle, p, r)?)
    } else {
        Ok(angle.pos(p, r)?)
    }
}
pub fn mpi_atan(x: &LegacyInterval, p: Precision) -> AtanResult<LegacyInterval> {
    Ok(LegacyInterval::new(
        mpf_atan(x.lower(), p, Rounding::Floor)?,
        mpf_atan(x.upper(), p, Rounding::Ceiling)?,
    )?)
}
pub fn mpi_atan2(
    y: &LegacyInterval,
    x: &LegacyInterval,
    p: Precision,
) -> AtanResult<LegacyInterval> {
    let (a, b) = mpi_atan2_raw(y, x, p)?;
    Ok(LegacyInterval::new(a, b)?)
}
/// Сырой legacy-ответ: на отрицательной полуоси source может вернуть a > b.
/// Не сортировать: это изменит прежний результат. Проверенный interval — отдельный API.
pub fn mpi_atan2_raw(
    y: &LegacyInterval,
    x: &LegacyInterval,
    p: Precision,
) -> AtanResult<(Mpf, Mpf)> {
    check(p)?;
    let (ya, yb, xa, xb) = (y.lower(), y.upper(), x.lower(), x.upper());
    let zero = Mpf::zero();
    if ya.is_zero() && yb.is_zero() {
        return if xa.compare(&zero).is_ge() {
            Ok((Mpf::zero(), Mpf::zero()))
        } else {
            Ok((mpf_pi(p, Rounding::Floor)?, mpf_pi(p, Rounding::Ceiling)?))
        };
    }
    let (a, b) = if xa.compare(&zero).is_ge() {
        (
            mpf_atan2(
                ya,
                if ya.compare(&zero).is_ge() { xb } else { xa },
                p,
                Rounding::Floor,
            )?,
            mpf_atan2(
                yb,
                if yb.compare(&zero).is_ge() { xa } else { xb },
                p,
                Rounding::Ceiling,
            )?,
        )
    } else if ya.compare(&zero).is_ge() {
        // Source вычисляет b прежде a; порядок отказов наблюдаем.
        let b = mpf_atan2(ya, xa, p, Rounding::Ceiling)?;
        let a = if xb.compare(&zero).is_le() {
            mpf_atan2(yb, xb, p, Rounding::Floor)?
        } else {
            mpf_atan2(ya, xb, p, Rounding::Floor)?
        };
        (a, b)
    } else if yb.compare(&zero).is_le() {
        let a = mpf_atan2(yb, xa, p, Rounding::Floor)?;
        let b = if xb.compare(&zero).is_le() {
            mpf_atan2(ya, xb, p, Rounding::Ceiling)?
        } else {
            mpf_atan2(yb, xb, p, Rounding::Ceiling)?
        };
        (a, b)
    } else {
        let b = mpf_pi(p, Rounding::Ceiling)?;
        (b.neg_exact(), b)
    };
    Ok((a, b))
}
