//! A2 prototype: exact rank enclosure for `cos(acos(sign*sqrt(r))/n)`.
//!
//! Это оболочка корня по рангу, НЕ изоляция единственного корня. Близкие rank1/2
//! могут иметь одну оболочку. Здесь нет product admission, old UNDECIDABLE,
//! principal-angle expression tree или выбора производственной точности.

use std::cmp::Ordering;

use crate::interval::{IntervalError, RationalInterval};
use crate::{Rational, UBig};

#[path = "angle_poly.rs"]
mod polynomial;
use polynomial::{
    angle_polynomial, chebyshev, compare, integer, midpoint, sign, sturm, variations, Polynomial,
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AngleError {
    UnsupportedSubdivision(u8),
    RatioOutsideHalfOpenUnitInterval,
    InconsistentDotSign(i8),
    BitCountOverflow,
    SturmInvariant,
}

/// Наблюдения прототипа; это не цена или лимит Python DensityExactWorkBudget.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AngleStats {
    pub polynomial_degree: usize,
    pub sturm_polynomials: usize,
    pub bisections: u32,
    pub variation_evaluations: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RankEnclosure {
    pub lower: Rational,
    pub upper: Rational,
    pub stats: AngleStats,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RayEnclosureError {
    InvalidOrdinal { ordinal: u8, subdivision: u8 },
    Angle(AngleError),
    InternalInterval(IntervalError),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RayEnclosure {
    pub cosine: RationalInterval,
    pub sine: RationalInterval,
    pub root: RankEnclosure,
}

/// Immutable exact polynomial plus rational-coefficient Sturm sequence.
#[derive(Clone, Debug)]
pub struct AngleAtom {
    subdivision: u8,
    ratio: Rational,
    dot_sign: i8,
    rank: usize,
    polynomial: Polynomial,
    sturm: Vec<Polynomial>,
    variations_at_one: usize,
    total_roots: usize,
}

impl AngleAtom {
    pub fn new(subdivision: u8, ratio: Rational, dot_sign: i8) -> Result<Self, AngleError> {
        if !matches!(subdivision, 1..=6) {
            return Err(AngleError::UnsupportedSubdivision(subdivision));
        }
        if sign(&ratio) < 0 || compare(&ratio, &integer(1)) != Ordering::Less {
            return Err(AngleError::RatioOutsideHalfOpenUnitInterval);
        }
        if !((sign(&ratio) == 0 && dot_sign == 0)
            || (sign(&ratio) > 0 && matches!(dot_sign, -1 | 1)))
        {
            return Err(AngleError::InconsistentDotSign(dot_sign));
        }
        let polynomial = angle_polynomial(subdivision, &ratio);
        let sequence = sturm(&polynomial);
        let (left_variations, left_root) = variations(&sequence, &integer(-1));
        let (right_variations, right_root) = variations(&sequence, &integer(1));
        let total_roots = left_variations
            .checked_sub(right_variations)
            .ok_or(AngleError::SturmInvariant)?;
        let expected = usize::from(subdivision) * if sign(&ratio) == 0 { 1 } else { 2 };
        if left_root || right_root || total_roots != expected || total_roots != polynomial.degree()
        {
            return Err(AngleError::SturmInvariant);
        }
        Ok(Self {
            subdivision,
            ratio,
            dot_sign,
            rank: if dot_sign < 0 { 2 } else { 1 },
            polynomial,
            sturm: sequence,
            variations_at_one: right_variations,
            total_roots,
        })
    }

    pub fn subdivision(&self) -> u8 {
        self.subdivision
    }
    pub fn ratio(&self) -> &Rational {
        &self.ratio
    }
    pub fn dot_sign(&self) -> i8 {
        self.dot_sign
    }
    pub fn rank(&self) -> usize {
        self.rank
    }
    pub fn total_roots(&self) -> usize {
        self.total_roots
    }
    pub fn polynomial_coefficients(&self) -> &[Rational] {
        &self.polynomial.0
    }

    pub fn is_root(&self, value: &Rational) -> bool {
        sign(&self.polynomial.evaluate(value)) == 0
    }

    /// G(x)=V(x)-V(1): число корней СТРОГО ВЫШЕ x (при x в [-1,1]).
    pub fn roots_strictly_above(&self, value: &Rational) -> usize {
        variations(&self.sturm, value).0 - self.variations_at_one
    }

    /// j/n principal rays: T_j(root) и положительный sqrt(1-cos²).
    /// Интервальная зависимость консервативна; ширина композиции не обещана.
    pub fn ray_enclosure(&self, ordinal: u8, bits: u32) -> Result<RayEnclosure, RayEnclosureError> {
        if ordinal == 0 || ordinal >= self.subdivision {
            return Err(RayEnclosureError::InvalidOrdinal {
                ordinal,
                subdivision: self.subdivision,
            });
        }
        bits.checked_mul(2)
            .ok_or(RayEnclosureError::InternalInterval(
                IntervalError::BitCountOverflow,
            ))?;
        let root = crate::angle_monotone::enclose_principal(
            self.subdivision,
            &self.ratio,
            self.dot_sign,
            bits,
        )
        .map_err(RayEnclosureError::Angle)?;
        let root_interval = RationalInterval::new(root.lower.clone(), root.upper.clone())
            .map_err(RayEnclosureError::InternalInterval)?;
        let mut cosine = RationalInterval::point(integer(0));
        for coefficient in chebyshev(ordinal).0.iter().rev() {
            cosine = cosine
                .mul(&root_interval)
                .add(&RationalInterval::point(coefficient.clone()));
        }
        let unit = RationalInterval::new(integer(-1), integer(1))
            .map_err(RayEnclosureError::InternalInterval)?;
        let nonnegative_unit = RationalInterval::new(integer(0), integer(1))
            .map_err(RayEnclosureError::InternalInterval)?;
        let cosine = cosine
            .intersect(&unit)
            .map_err(RayEnclosureError::InternalInterval)?;
        let sine_squared = RationalInterval::point(integer(1))
            .sub(&cosine.square())
            .intersect(&nonnegative_unit)
            .map_err(RayEnclosureError::InternalInterval)?;
        let sine = sine_squared
            .positive_sqrt(bits)
            .map_err(RayEnclosureError::InternalInterval)?
            .intersect(&nonnegative_unit)
            .map_err(RayEnclosureError::InternalInterval)?;
        Ok(RayEnclosure { cosine, sine, root })
    }

    /// После B+1 делений ширина не более 2^-B; exact target даёт точку.
    /// Оболочка вправе содержать другие roots: distinct-root isolation не обещана.
    pub fn enclose(&self, bits: u32) -> Result<RankEnclosure, AngleError> {
        let steps = bits.checked_add(1).ok_or(AngleError::BitCountOverflow)?;
        let mut lower = integer(-1);
        let mut upper = integer(1);
        let mut stats = AngleStats {
            polynomial_degree: self.polynomial.degree(),
            sturm_polynomials: self.sturm.len(),
            bisections: 0,
            variation_evaluations: 0,
        };
        for _ in 0..steps {
            let middle = midpoint(&lower, &upper);
            let (middle_variations, at_root) = variations(&self.sturm, &middle);
            let above = middle_variations
                .checked_sub(self.variations_at_one)
                .ok_or(AngleError::SturmInvariant)?;
            stats.bisections += 1;
            stats.variation_evaluations += 1;
            if at_root && above == self.rank - 1 {
                return Ok(RankEnclosure {
                    lower: middle.clone(),
                    upper: middle,
                    stats,
                });
            }
            if above >= self.rank {
                lower = middle;
            } else {
                upper = middle;
            }
        }
        // Проверка ширины целыми числами также охраняет B+1 от случайного B.
        let width = polynomial::sub(&upper, &lower);
        let (n, d) = width.parts();
        if sign(&width) < 0
            || cftuv_radical::arith::magnitude(n) * (UBig::ONE << bits as usize) > *d
        {
            return Err(AngleError::SturmInvariant);
        }
        Ok(RankEnclosure {
            lower,
            upper,
            stats,
        })
    }
}
