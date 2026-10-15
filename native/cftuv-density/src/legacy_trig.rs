//! Ограниченный legacy-слой константы π; sin/cos пока не перенесены.
//! Источник: mpmath 1.3.0, Python backend, libmp/libelefun.py::def_mpf_constant.
//! SHA256: 8e80593f814e7713df89e5aca352cfb52afa747c9da46fcb42217f6d841858c8.
//! Таблица π математически сертифицируется независимым Machin-тестом.
//! Проверенный legacy-профиль не доказывает совместимость произвольного memo history.
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

use crate::legacy_mpf::{LegacyError, Mpf, Precision, Rounding};
use crate::{IBig, UBig};

pub const PI_PREFIX_BITS: u32 = 2048;
pub const MIN_MPF_PI_PRECISION_BITS: u32 = 2;
pub const MAX_MPF_PI_PRECISION_BITS: u32 = 512;
pub const MAX_PI_FIXED_BITS: u32 = 800;
// Проверены все memo-вычисления от requested=0 до этой границы включительно.
// Битовая совместимость после более высоких legacy-запросов ещё не доказана.
pub const CALIBRATED_LEGACY_PI_MEMO_REQUEST_MAX: u32 = 1536;

/// floor(π * 2^2048); данные неизменяемы, mutable cache отсутствует.
pub const PI_PREFIX_HEX: &str = concat!(
    "3243f6a8885a308d313198a2e03707344a4093822299f31d0082efa98ec4e6c894",
    "52821e638d01377be5466cf34e90c6cc0ac29b7c97c50dd3f84d5b5b547091792",
    "16d5d98979fb1bd1310ba698dfb5ac2ffd72dbd01adfb7b8e1afed6a267e96ba7",
    "c9045f12c7f9924a19947b3916cf70801f2e2858efc16636920d871574e69a458f",
    "ea3f4933d7e0d95748f728eb658718bcd5882154aee7b54a41dc25a59b59c30d53",
    "92af26013c5d1b023286085f0ca417918b8db38ef8e79dcb0603a180e6c9e0e8b",
    "b01e8a3ed71577c1bd314b2778af2fda55605c60e65525f3aa55ab945748986263",
    "e8144055ca396a2aab10b6b4cc5c341141e8cea15486af7c72e993",
);

/// Это внутренние границы прототипа, не продуктовые исходы Density.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum UnsupportedProfile {
    PiFixedBits {
        requested: u32,
        maximum: u32,
    },
    MpfPiPrecision {
        requested: u32,
        minimum: u32,
        maximum: u32,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TrigError {
    UnsupportedProfile(UnsupportedProfile),
    LegacyArithmetic(LegacyError),
}

pub type TrigResult<T> = Result<T, TrigError>;

/// Точный математический floor в явно ограниченном legacy-профиле.
/// Совместимость memo проверена до CALIBRATED_LEGACY_PI_MEMO_REQUEST_MAX.
pub fn pi_fixed(bits: u32) -> TrigResult<UBig> {
    if bits > MAX_PI_FIXED_BITS {
        return Err(TrigError::UnsupportedProfile(
            UnsupportedProfile::PiFixedBits {
                requested: bits,
                maximum: MAX_PI_FIXED_BITS,
            },
        ));
    }
    let prefix = UBig::from_str_radix(PI_PREFIX_HEX, 16)
        .expect("сертифицированная неизменяемая hexadecimal-константа π");
    Ok(prefix >> (PI_PREFIX_BITS - bits) as usize)
}

/// Порядок def_mpf_constant сохранён: wp=p+20, затем +1 для Ceiling/AwayFromZero.
/// Это leaf-прототип p=2..512; замены продуктового admission здесь нет.
pub fn mpf_pi(precision: Precision, rounding: Rounding) -> TrigResult<Mpf> {
    let bits = precision.bits();
    if !(MIN_MPF_PI_PRECISION_BITS..=MAX_MPF_PI_PRECISION_BITS).contains(&bits) {
        return Err(TrigError::UnsupportedProfile(
            UnsupportedProfile::MpfPiPrecision {
                requested: bits,
                minimum: MIN_MPF_PI_PRECISION_BITS,
                maximum: MAX_MPF_PI_PRECISION_BITS,
            },
        ));
    }
    let wp = bits + 20;
    let mut mantissa = pi_fixed(wp)?;
    if matches!(rounding, Rounding::Ceiling | Rounding::AwayFromZero) {
        mantissa += UBig::ONE;
    }
    Mpf::from_man_exp(
        IBig::from(mantissa),
        -i64::from(wp),
        Some(precision),
        rounding,
    )
    .map_err(TrigError::LegacyArithmetic)
}
