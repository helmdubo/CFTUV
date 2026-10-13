//! Фиксированная A2-точность для компонент главного луча, без legacy admission.
//!
//! Для T_j на [-1,1] Horner-оболочка ширины δ имеет ширину <=L_j δ,
//! L_j=sum(k*|a_k|)=(1,4,15,48,145), j=1..5. После square ширина <=2L_j δ.
//! Для неотрицательных a<=b: sqrt(b)-sqrt(a)<=sqrt(b-a). Dyadic sqrt
//! расширяет каждый конец меньше чем на 2^-B. Поэтому при
//! B=320+ceil(log2(2L_j))+4 обе компоненты уже имеют ширину <2^-160,
//! включая сколь угодно близкий к нулю синус. B зависит только от n.
//! Эта граница не ограничивает ошибку произвольной композиции или нормы луча.

use crate::angle::{AngleAtom, RayEnclosure, RayEnclosureError};
use crate::interval::RationalInterval;
use crate::{IBig, UBig};

pub const COMPONENT_BITS: u32 = 160;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FixedAngleError {
    NoHiddenOrdinal,
    Ray(RayEnclosureError),
    WidthProofViolated,
}

/// Максимальный j=n-1 определяет фиксированную точность всех лучей одного n.
pub fn root_bits(subdivision: u8) -> Result<u32, FixedAngleError> {
    match subdivision {
        2 => Ok(325),
        3 => Ok(327),
        4 => Ok(329),
        5 => Ok(331),
        6 => Ok(333),
        _ => Err(FixedAngleError::NoHiddenOrdinal),
    }
}

fn narrow_enough(interval: &RationalInterval) -> bool {
    let (ln, ld) = interval.lower().parts();
    let (un, ud) = interval.upper().parts();
    let numerator = un * ld - ln * ud;
    numerator >= IBig::ZERO
        && (numerator << COMPONENT_BITS as usize) < IBig::from(ld * ud)
}

pub fn ray(atom: &AngleAtom, ordinal: u8) -> Result<RayEnclosure, FixedAngleError> {
    let result = atom.ray_enclosure(ordinal, root_bits(atom.subdivision())?)
        .map_err(FixedAngleError::Ray)?;
    // Это проверка доказательства, никогда не именованный отказ старого оракула.
    if !narrow_enough(&result.cosine) || !narrow_enough(&result.sine) {
        return Err(FixedAngleError::WidthProofViolated);
    }
    Ok(result)
}

/// Целочисленная проверка достаточности таблицы; полезна при смене точности.
pub fn profile_bound_is_valid(subdivision: u8) -> bool {
    let Ok(bits) = root_bits(subdivision) else { return false };
    let lipschitz = [0u32, 1, 4, 15, 48, 145][usize::from(subdivision - 1)];
    // sqrt(2L*2^-B)<=2^-(P+2), а 2*2^-B<2^-(P+2).
    UBig::from(2 * lipschitz) <= (UBig::ONE << (bits - 2 * COMPONENT_BITS - 4) as usize)
        && bits > COMPONENT_BITS + 3
}
