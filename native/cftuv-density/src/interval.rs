//! Closed rational intervals for A2/A1 prototypes; no product canon/admission.
//! Exact endpoint arithmetic, positive sqrt by integer outward dyadic bounds.

use std::cmp::Ordering;

use cftuv_radical::arith::{fraction, magnitude, signed};
use cftuv_radical::{Radical, Term};

use crate::{IBig, Rational, UBig};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IntervalError {
    ReversedBounds,
    EmptyIntersection,
    /// Внутренний unsupported interval quotient; не oracle UNDECIDABLE.
    DenominatorContainsZero,
    NegativeRadicand,
    BitCountOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RationalInterval {
    lower: Rational,
    upper: Rational,
}

impl RationalInterval {
    pub fn new(lower: Rational, upper: Rational) -> Result<Self, IntervalError> {
        if rational_cmp(&lower, &upper).is_gt() {
            return Err(IntervalError::ReversedBounds);
        }
        Ok(Self { lower, upper })
    }

    pub fn point(value: Rational) -> Self {
        Self {
            lower: value.clone(),
            upper: value,
        }
    }

    pub fn lower(&self) -> &Rational {
        &self.lower
    }
    pub fn upper(&self) -> &Rational {
        &self.upper
    }

    pub fn contains(&self, value: &Rational) -> bool {
        rational_cmp(&self.lower, value).is_le() && rational_cmp(value, &self.upper).is_le()
    }

    pub fn add(&self, other: &Self) -> Self {
        Self {
            lower: rational_add(&self.lower, &other.lower),
            upper: rational_add(&self.upper, &other.upper),
        }
    }

    pub fn sub(&self, other: &Self) -> Self {
        Self {
            lower: rational_sub(&self.lower, &other.upper),
            upper: rational_sub(&self.upper, &other.lower),
        }
    }

    pub fn mul(&self, other: &Self) -> Self {
        let candidates = [
            rational_mul(&self.lower, &other.lower),
            rational_mul(&self.lower, &other.upper),
            rational_mul(&self.upper, &other.lower),
            rational_mul(&self.upper, &other.upper),
        ];
        let lower = candidates
            .iter()
            .min_by(|left, right| rational_cmp(left, right))
            .unwrap()
            .clone();
        let upper = candidates
            .iter()
            .max_by(|left, right| rational_cmp(left, right))
            .unwrap()
            .clone();
        Self { lower, upper }
    }

    pub fn div(&self, other: &Self) -> Result<Self, IntervalError> {
        if rational_sign(&other.lower) <= 0 && rational_sign(&other.upper) >= 0 {
            return Err(IntervalError::DenominatorContainsZero);
        }
        let one = Rational::integer(IBig::ONE);
        let reciprocal = Self {
            lower: rational_div(&one, &other.upper).expect("denominator excludes zero"),
            upper: rational_div(&one, &other.lower).expect("denominator excludes zero"),
        };
        Ok(self.mul(&reciprocal))
    }

    /// Учитывает повторную переменную: square([-1,1])=[0,1], не [-1,1].
    pub fn square(&self) -> Self {
        let lower_square = rational_mul(&self.lower, &self.lower);
        let upper_square = rational_mul(&self.upper, &self.upper);
        if rational_sign(&self.lower) >= 0 {
            Self {
                lower: lower_square,
                upper: upper_square,
            }
        } else if rational_sign(&self.upper) <= 0 {
            Self {
                lower: upper_square,
                upper: lower_square,
            }
        } else {
            Self {
                lower: Rational::integer(IBig::ZERO),
                upper: if rational_cmp(&lower_square, &upper_square).is_gt() {
                    lower_square
                } else {
                    upper_square
                },
            }
        }
    }

    pub fn intersect(&self, other: &Self) -> Result<Self, IntervalError> {
        let lower = if rational_cmp(&self.lower, &other.lower).is_gt() {
            self.lower.clone()
        } else {
            other.lower.clone()
        };
        let upper = if rational_cmp(&self.upper, &other.upper).is_lt() {
            self.upper.clone()
        } else {
            other.upper.clone()
        };
        Self::new(lower, upper).map_err(|_| IntervalError::EmptyIntersection)
    }

    /// Для каждого endpoint — floor/ceil(sqrt(x)*2^bits), целочисленно.
    /// Ширина исходного interval сохраняется; обещания абсолютной ширины нет.
    pub fn positive_sqrt(&self, bits: u32) -> Result<Self, IntervalError> {
        if rational_sign(&self.lower) < 0 {
            return Err(IntervalError::NegativeRadicand);
        }
        let double_bits = bits.checked_mul(2).ok_or(IntervalError::BitCountOverflow)?;
        let shift = usize::try_from(bits).map_err(|_| IntervalError::BitCountOverflow)?;
        let double_shift =
            usize::try_from(double_bits).map_err(|_| IntervalError::BitCountOverflow)?;
        let (lower, _) = dyadic_sqrt(&self.lower, shift, double_shift);
        let (_, upper) = dyadic_sqrt(&self.upper, shift, double_shift);
        Ok(Self { lower, upper })
    }

    /// A1 primitive: floor(lower*10^27), ceil(upper*10^27), без Decimal/canonflip.
    pub fn scale10_27_envelope(&self) -> (IBig, IBig) {
        (
            floor_scaled_10_27(&self.lower),
            ceil_scaled_10_27(&self.upper),
        )
    }
}

pub fn floor_scaled_10_27(value: &Rational) -> IBig {
    scaled_integer(value, false)
}

pub fn ceil_scaled_10_27(value: &Rational) -> IBig {
    scaled_integer(value, true)
}

fn scaled_integer(value: &Rational, ceiling: bool) -> IBig {
    let factor = (0..27).fold(UBig::ONE, |current, _| current * UBig::from(10u8));
    let (n, d) = value.parts();
    let scaled = magnitude(n) * factor;
    let quotient = &scaled / d;
    let has_remainder = &quotient * d != scaled;
    if *n < IBig::ZERO {
        -signed(
            &(quotient
                + if !ceiling && has_remainder {
                    UBig::ONE
                } else {
                    UBig::ZERO
                }),
        )
    } else {
        signed(
            &(quotient
                + if ceiling && has_remainder {
                    UBig::ONE
                } else {
                    UBig::ZERO
                }),
        )
    }
}

fn dyadic_sqrt(value: &Rational, bits: usize, double_bits: usize) -> (Rational, Rational) {
    let (n, d) = value.parts();
    let scaled = magnitude(n) << double_bits;
    let quotient = &scaled / d;
    // Public integer isqrt helper Radical::floor_root читает только radicand.
    // Algebraic value/выражение не создаётся; цена Radical здесь не расходуется.
    let root = Radical::floor_root(&Term {
        radicand: quotient,
        numerator: IBig::ONE,
        denominator: UBig::ONE,
    });
    let exact = &root * &root * d == scaled;
    let denominator = UBig::ONE << bits;
    let lower =
        Rational::new(signed(&root), denominator.clone()).expect("dyadic denominator is positive");
    let upper = Rational::new(
        signed(&(root + if exact { UBig::ZERO } else { UBig::ONE })),
        denominator,
    )
    .expect("dyadic denominator is positive");
    (lower, upper)
}

// Одна rational-арифметика для interval и polynomial, без копий в AngleAtom.
pub(crate) fn rational_sign(value: &Rational) -> i8 {
    cftuv_radical::arith::signum(value.parts().0)
}

pub(crate) fn rational_cmp(left: &Rational, right: &Rational) -> Ordering {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    (ln * signed(rd)).cmp(&(rn * signed(ld)))
}

pub(crate) fn rational_add(left: &Rational, right: &Rational) -> Rational {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    Rational::new(ln * signed(rd) + rn * signed(ld), ld * rd)
        .expect("positive rational denominators")
}

pub(crate) fn rational_neg(value: &Rational) -> Rational {
    let (n, d) = value.parts();
    Rational::new(-n, d.clone()).expect("positive rational denominator")
}

pub(crate) fn rational_sub(left: &Rational, right: &Rational) -> Rational {
    rational_add(left, &rational_neg(right))
}

pub(crate) fn rational_mul(left: &Rational, right: &Rational) -> Rational {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    Rational::new(ln * rn, ld * rd).expect("positive rational denominators")
}

pub(crate) fn rational_div(left: &Rational, right: &Rational) -> Option<Rational> {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    let (n, d) = fraction(ln * signed(rd), signed(ld) * rn)?;
    Some(Rational::new(n, d).expect("positive rational denominator"))
}
