//! Быстрая A2-оболочка на главной монотонной ветви T_n.
//!
//! Guard x<=cos(pi/n) проверяется до polynomial equality: корни других ветвей
//! не могут завершить поиск. Вся проверка T_n(x) против s*sqrt(r) целочисленна.
//! Это математический прототип; старый допуск/UNDECIDABLE не изменяется.

use std::cmp::Ordering;

use cftuv_radical::arith::{magnitude, signed, signum};

use crate::angle::{AngleError, AngleStats, RankEnclosure};
use crate::interval::{rational_add, rational_mul, rational_sub};
use crate::{IBig, Rational, UBig};

/// Сравнить x с alpha=cos(acos(s*sqrt(r))/n), не вычисляя угол или корень.
pub fn compare_to_principal(
    subdivision: u8,
    ratio: &Rational,
    dot_sign: i8,
    coordinate: &Rational,
) -> Result<Ordering, AngleError> {
    validate(subdivision, ratio, dot_sign)?;
    Ok(compare_validated(subdivision, ratio, dot_sign, coordinate))
}

/// Концы и число делений совпадают с rank-оболочкой прежнего AngleAtom::enclose.
/// Диагностика явно отличает fast-путь: variation_evaluations=sturm_polynomials=0.
pub fn enclose_principal(
    subdivision: u8,
    ratio: &Rational,
    dot_sign: i8,
    bits: u32,
) -> Result<RankEnclosure, AngleError> {
    validate(subdivision, ratio, dot_sign)?;
    let steps = bits.checked_add(1).ok_or(AngleError::BitCountOverflow)?;
    let mut lower = Rational::integer(IBig::NEG_ONE);
    let mut upper = Rational::integer(IBig::ONE);
    let half = Rational::new(IBig::ONE, UBig::from(2u8)).expect("positive denominator");
    let mut stats = AngleStats {
        polynomial_degree: usize::from(subdivision) * if ratio.parts().0.is_zero() { 1 } else { 2 },
        sturm_polynomials: 0,
        bisections: 0,
        variation_evaluations: 0,
    };
    for _ in 0..steps {
        let middle = rational_mul(&rational_add(&lower, &upper), &half);
        stats.bisections += 1;
        match compare_validated(subdivision, ratio, dot_sign, &middle) {
            Ordering::Equal => {
                return Ok(RankEnclosure {
                    lower: middle.clone(),
                    upper: middle,
                    stats,
                })
            }
            Ordering::Less => lower = middle,
            Ordering::Greater => upper = middle,
        }
    }
    let width = rational_sub(&upper, &lower);
    let (n, d) = width.parts();
    if *n < IBig::ZERO || magnitude(n) * (UBig::ONE << bits as usize) > *d {
        return Err(AngleError::SturmInvariant);
    }
    Ok(RankEnclosure {
        lower,
        upper,
        stats,
    })
}

fn validate(subdivision: u8, ratio: &Rational, dot_sign: i8) -> Result<(), AngleError> {
    if !(1..=6).contains(&subdivision) {
        return Err(AngleError::UnsupportedSubdivision(subdivision));
    }
    let (p, q) = ratio.parts();
    if *p < IBig::ZERO || *p >= signed(q) {
        return Err(AngleError::RatioOutsideHalfOpenUnitInterval);
    }
    if !((p.is_zero() && dot_sign == 0) || (*p > IBig::ZERO && matches!(dot_sign, -1 | 1))) {
        return Err(AngleError::InconsistentDotSign(dot_sign));
    }
    Ok(())
}

fn compare_validated(
    subdivision: u8,
    ratio: &Rational,
    dot_sign: i8,
    coordinate: &Rational,
) -> Ordering {
    let (a, d) = coordinate.parts();
    let (p, q) = ratio.parts();
    if subdivision == 1 {
        return compare_with_signed_root(a, d, p, q, dot_sign);
    }
    // r<1 доказывает alpha>cos(pi/n). Даже точный polynomial root здесь НЕ target.
    if at_or_below_principal_floor(subdivision, a, d) {
        return Ordering::Less;
    }
    let d_squared = signed(&(d * d));
    let two_a = a * IBig::from(2u8);
    let mut previous = IBig::ONE;
    let mut current = a.clone();
    for _ in 1..subdivision {
        let next = &two_a * &current - &d_squared * &previous;
        previous = current;
        current = next;
    }
    let mut denominator = UBig::ONE;
    for _ in 0..subdivision {
        denominator *= d;
    }
    compare_with_signed_root(&current, &denominator, p, q, dot_sign)
}

fn at_or_below_principal_floor(subdivision: u8, a: &IBig, d: &UBig) -> bool {
    let d_squared = signed(&(d * d));
    match subdivision {
        2 => *a <= IBig::ZERO,
        3 => a * IBig::from(2u8) <= signed(d),
        4 => *a <= IBig::ZERO || a * a * IBig::from(2u8) <= d_squared,
        5 => {
            let y = a * IBig::from(4u8) - signed(d);
            y <= IBig::ZERO || &y * &y <= d_squared * IBig::from(5u8)
        }
        6 => *a <= IBig::ZERO || a * a * IBig::from(4u8) <= d_squared * IBig::from(3u8),
        _ => unreachable!("validated subdivision 2..6"),
    }
}

/// P/D против s*sqrt(p/q). При разных знаках квадратирование не требуется.
fn compare_with_signed_root(
    p_value: &IBig,
    denominator: &UBig,
    p: &IBig,
    q: &UBig,
    dot_sign: i8,
) -> Ordering {
    let value_sign = signum(p_value);
    if p.is_zero() {
        return value_sign.cmp(&0);
    }
    if value_sign != dot_sign {
        return value_sign.cmp(&dot_sign);
    }
    let residual = signed(q) * p_value * p_value - p * signed(&(denominator * denominator));
    let comparison = residual.cmp(&IBig::ZERO);
    if dot_sign > 0 {
        comparison
    } else {
        comparison.reverse()
    }
}
