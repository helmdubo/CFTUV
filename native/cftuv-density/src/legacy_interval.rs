//! Рациональный/двоичный интервальный legacy-слой: mpmath 1.3.0 `libmp/libmpi.py`.
//! SHA256 источника: bb4239122c24a9afb8f9d5c44e2e64eccb9ac417996ef0803c5b65f7753d605d.
//! Перенесены ветви по знакам и порядок точности из mpi_* строк 64..341.
//! Тригонометрия, трансцендентные константы, DAG-кэш и продуктовый диспетчер сюда не входят.
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

use crate::legacy_mpf::{profile, LegacyResult, Mpf, Precision, Rounding, UnsupportedProfile};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LegacyInterval {
    lower: Mpf,
    upper: Mpf,
}

impl LegacyInterval {
    pub fn new(lower: Mpf, upper: Mpf) -> LegacyResult<Self> {
        if lower.is_nan() || upper.is_nan() {
            return Err(profile(UnsupportedProfile::NaNIntervalEndpoint));
        }
        if lower.compare(&upper).is_gt() {
            return Err(profile(UnsupportedProfile::ReversedInterval));
        }
        Ok(Self { lower, upper })
    }
    pub fn lower(&self) -> &Mpf {
        &self.lower
    }
    pub fn upper(&self) -> &Mpf {
        &self.upper
    }
    pub fn zero() -> Self {
        Self {
            lower: Mpf::zero(),
            upper: Mpf::zero(),
        }
    }
    pub fn one() -> Self {
        Self {
            lower: Mpf::one(),
            upper: Mpf::one(),
        }
    }
    pub fn entire() -> Self {
        Self {
            lower: Mpf::negative_infinity(),
            upper: Mpf::positive_infinity(),
        }
    }

    fn check_input(&self) -> LegacyResult<()> {
        if self.lower.is_nan() || self.upper.is_nan() {
            Err(profile(UnsupportedProfile::NaNIntervalEndpoint))
        } else {
            Ok(())
        }
    }

    pub fn pos(&self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        Ok(Self {
            lower: self.lower.pos(precision, Rounding::Floor)?,
            upper: self.upper.pos(precision, Rounding::Ceiling)?,
        })
    }
    pub fn neg(&self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        Ok(Self {
            lower: self.upper.neg(precision, Rounding::Floor)?,
            upper: self.lower.neg(precision, Rounding::Ceiling)?,
        })
    }
    fn neg_exact(&self) -> Self {
        Self {
            lower: self.upper.neg_exact(),
            upper: self.lower.neg_exact(),
        }
    }

    pub fn abs(&self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        if self.lower.sign() >= 0 {
            return self.pos(precision);
        }
        if self.upper.sign() <= 0 {
            return self.neg(precision);
        }
        let neg_lower = self.lower.neg_exact();
        let upper = if neg_lower.compare(&self.upper).is_lt() {
            self.upper.pos(precision, Rounding::Ceiling)?
        } else {
            neg_lower.pos(precision, Rounding::Ceiling)?
        };
        Ok(Self {
            lower: Mpf::zero(),
            upper,
        })
    }

    pub fn add(&self, other: &Self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        other.check_input()?;
        let lower = self.lower.add(&other.lower, precision, Rounding::Floor)?;
        let upper = self.upper.add(&other.upper, precision, Rounding::Ceiling)?;
        Ok(Self {
            lower: replace_nan(lower, Mpf::negative_infinity()),
            upper: replace_nan(upper, Mpf::positive_infinity()),
        })
    }
    pub fn sub(&self, other: &Self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        other.check_input()?;
        let lower = self.lower.sub(&other.upper, precision, Rounding::Floor)?;
        let upper = self.upper.sub(&other.lower, precision, Rounding::Ceiling)?;
        Ok(Self {
            lower: replace_nan(lower, Mpf::negative_infinity()),
            upper: replace_nan(upper, Mpf::positive_infinity()),
        })
    }

    pub fn mul(&self, other: &Self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        other.check_input()?;
        let (sa, sb, ta, tb) = (&self.lower, &self.upper, &other.lower, &other.upper);
        let (sas, sbs, tas, tbs) = (sa.sign(), sb.sign(), ta.sign(), tb.sign());
        if sas == 0 && sbs == 0 {
            return Ok(if ta.is_negative_infinity() || tb.is_positive_infinity() {
                Self::entire()
            } else {
                Self::zero()
            });
        }
        if tas == 0 && tbs == 0 {
            return Ok(if sa.is_negative_infinity() || sb.is_positive_infinity() {
                Self::entire()
            } else {
                Self::zero()
            });
        }
        let floor = |left: &Mpf, right: &Mpf| left.mul(right, precision, Rounding::Floor);
        let ceil = |left: &Mpf, right: &Mpf| left.mul(right, precision, Rounding::Ceiling);
        let (lower, upper, lower_nan, upper_nan) = if sas >= 0 {
            if tas >= 0 {
                (
                    floor(sa, ta)?,
                    ceil(sb, tb)?,
                    Mpf::zero(),
                    Mpf::positive_infinity(),
                )
            } else if tbs <= 0 {
                (
                    floor(sb, ta)?,
                    ceil(sa, tb)?,
                    Mpf::negative_infinity(),
                    Mpf::zero(),
                )
            } else {
                (
                    floor(sb, ta)?,
                    ceil(sb, tb)?,
                    Mpf::negative_infinity(),
                    Mpf::positive_infinity(),
                )
            }
        } else if sbs <= 0 {
            if tas >= 0 {
                (
                    floor(sa, tb)?,
                    ceil(sb, ta)?,
                    Mpf::negative_infinity(),
                    Mpf::zero(),
                )
            } else if tbs <= 0 {
                (
                    floor(sb, tb)?,
                    ceil(sa, ta)?,
                    Mpf::zero(),
                    Mpf::positive_infinity(),
                )
            } else {
                (
                    floor(sa, tb)?,
                    ceil(sa, ta)?,
                    Mpf::negative_infinity(),
                    Mpf::positive_infinity(),
                )
            }
        } else {
            // В источнике при обоих знаках s выполняются общие exact cross-products.
            return self.mixed_products(other, precision);
        };
        Ok(Self {
            lower: replace_nan(lower, lower_nan),
            upper: replace_nan(upper, upper_nan),
        })
    }

    fn mixed_products(&self, other: &Self, precision: Precision) -> LegacyResult<Self> {
        let cases = [
            self.lower.mul_exact(&other.lower)?,
            self.lower.mul_exact(&other.upper)?,
            self.upper.mul_exact(&other.lower)?,
            self.upper.mul_exact(&other.upper)?,
        ];
        if cases.iter().any(Mpf::is_nan) {
            return Ok(Self::entire());
        }
        let lower = cases
            .iter()
            .min_by(|left, right| left.compare(right))
            .unwrap()
            .pos(precision, Rounding::Floor)?;
        let upper = cases
            .iter()
            .max_by(|left, right| left.compare(right))
            .unwrap()
            .pos(precision, Rounding::Ceiling)?;
        Ok(Self { lower, upper })
    }

    pub fn square(&self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        let (low, high) = if self.lower.sign() >= 0 {
            (&self.lower, &self.upper)
        } else if self.upper.sign() <= 0 {
            (&self.upper, &self.lower)
        } else {
            let neg_lower = self.lower.neg_exact();
            let high = if neg_lower.compare(&self.upper).is_ge() {
                neg_lower
            } else {
                self.upper.clone()
            };
            return Ok(Self {
                lower: Mpf::zero(),
                upper: high.square(precision, Rounding::Ceiling)?,
            });
        };
        Ok(Self {
            lower: low.square(precision, Rounding::Floor)?,
            upper: high.square(precision, Rounding::Ceiling)?,
        })
    }

    pub fn div(&self, other: &Self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        other.check_input()?;
        let (sa, sb, ta, tb) = (&self.lower, &self.upper, &other.lower, &other.upper);
        let (sas, sbs, tas, tbs) = (sa.sign(), sb.sign(), ta.sign(), tb.sign());
        if sas == 0 && sbs == 0 {
            return Ok(if (tas < 0 && tbs > 0) || tas == 0 || tbs == 0 {
                Self::entire()
            } else {
                Self::zero()
            });
        }
        if tas < 0 && tbs > 0 {
            return Ok(Self::entire());
        }
        if tas < 0 {
            return self.neg_exact().div(&other.neg_exact(), precision);
        }
        let floor = |left: &Mpf, right: &Mpf| left.div(right, precision, Rounding::Floor);
        let ceil = |left: &Mpf, right: &Mpf| left.div(right, precision, Rounding::Ceiling);
        if tas == 0 {
            if (sas < 0 && sbs > 0) || tas == tbs {
                return Ok(Self::entire());
            }
            // libmpi сохраняет возможные NaN-концы в этой ветке: raw-ответ остаётся тем же.
            // Следующая операция получает явный UnsupportedProfile вместо подмены значения.
            if sas >= 0 {
                return Ok(Self {
                    lower: floor(sa, tb)?,
                    upper: Mpf::positive_infinity(),
                });
            }
            return Ok(Self {
                lower: Mpf::negative_infinity(),
                upper: ceil(sb, tb)?,
            });
        }
        let (lower, upper, lower_nan, upper_nan) = if sas >= 0 {
            (
                floor(sa, tb)?,
                ceil(sb, ta)?,
                Mpf::zero(),
                Mpf::positive_infinity(),
            )
        } else if sbs <= 0 {
            (
                floor(sa, ta)?,
                ceil(sb, tb)?,
                Mpf::negative_infinity(),
                Mpf::zero(),
            )
        } else {
            (
                floor(sa, ta)?,
                ceil(sb, ta)?,
                Mpf::negative_infinity(),
                Mpf::positive_infinity(),
            )
        };
        Ok(Self {
            lower: replace_nan(lower, lower_nan),
            upper: replace_nan(upper, upper_nan),
        })
    }

    pub fn sqrt(&self, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        Ok(Self {
            lower: self.lower.sqrt(precision, Rounding::Floor)?,
            upper: self.upper.sqrt(precision, Rounding::Ceiling)?,
        })
    }

    pub fn pow_int(&self, exponent: i64, precision: Precision) -> LegacyResult<Self> {
        self.check_input()?;
        let n = i32::try_from(exponent).map_err(|_| {
            profile(UnsupportedProfile::PowerExponent {
                requested: exponent,
                minimum: i64::from(i32::MIN),
                maximum: i64::from(i32::MAX),
            })
        })?;
        if n < 0 {
            // Отрицательная целая степень libmpi: положительная при prec+20, затем интервальное деление при prec.
            let denominator = self.pow_nonnegative(n.unsigned_abs(), precision.extra(20)?)?;
            return Self::one().div(&denominator, precision);
        }
        self.pow_nonnegative(n as u32, precision)
    }

    fn pow_nonnegative(&self, n: u32, precision: Precision) -> LegacyResult<Self> {
        if n == 0 {
            return Ok(Self::one());
        }
        if n == 1 {
            return Ok(self.clone());
        }
        if n == 2 {
            return self.square(precision);
        }
        let floor = |value: &Mpf| value.pow_nonnegative(n, precision, Rounding::Floor);
        let ceil = |value: &Mpf| value.pow_nonnegative(n, precision, Rounding::Ceiling);
        if n & 1 != 0 || self.lower.sign() >= 0 {
            return Ok(Self {
                lower: floor(&self.lower)?,
                upper: ceil(&self.upper)?,
            });
        }
        if self.upper.sign() <= 0 {
            return Ok(Self {
                lower: floor(&self.upper)?,
                upper: ceil(&self.lower)?,
            });
        }
        let neg_lower = self.lower.neg_exact();
        let high = if neg_lower.compare(&self.upper).is_ge() {
            neg_lower
        } else {
            self.upper.clone()
        };
        Ok(Self {
            lower: Mpf::zero(),
            upper: ceil(&high)?,
        })
    }
}

fn replace_nan(value: Mpf, substitute: Mpf) -> Mpf {
    if value.is_nan() {
        substitute
    } else {
        value
    }
}
