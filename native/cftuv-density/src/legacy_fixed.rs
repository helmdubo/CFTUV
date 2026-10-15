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
//! Общие целочисленные правила floor и directed perturb исходного Python backend.
use crate::legacy_mpf::{LegacyResult, Mpf, Precision, Rounding};
use crate::{IBig, UBig};

pub(crate) fn floor_div(value: &IBig, denominator: &IBig) -> IBig {
    debug_assert!(denominator > &IBig::ZERO);
    let quotient = value / denominator;
    if value < &IBig::ZERO && &quotient * denominator != *value {
        quotient - IBig::ONE
    } else {
        quotient
    }
}
pub(crate) fn floor_shift(value: &IBig, bits: usize) -> IBig {
    floor_div(value, &IBig::from(UBig::ONE << bits))
}
pub(crate) fn perturb(
    value: &Mpf,
    epsilon_negative: bool,
    precision: Precision,
    rounding: Rounding,
) -> LegacyResult<Mpf> {
    if rounding == Rounding::Nearest {
        return value.pos(precision, rounding);
    }
    let (sign, _, exp, bc) = value.raw_tuple();
    let epsilon_exp = exp + bc - i64::from(precision.bits()) - 1;
    let epsilon = Mpf::from_man_exp(
        if epsilon_negative {
            -IBig::ONE
        } else {
            IBig::ONE
        },
        epsilon_exp,
        None,
        Rounding::TowardZero,
    )?;
    let away = if sign != 0 {
        matches!(rounding, Rounding::TowardZero | Rounding::Ceiling) ^ epsilon_negative
    } else {
        matches!(rounding, Rounding::AwayFromZero | Rounding::Ceiling) ^ epsilon_negative
    };
    if away {
        value.add(&epsilon, precision, rounding)
    } else {
        value.pos(precision, rounding)
    }
}
