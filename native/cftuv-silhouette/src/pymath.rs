//! The CPython 3.11 float semantics the silhouette law and the source lift read: `math.hypot` (the 3.11 `vector_norm`, not the C
//! library's, for a largest component that is normal; two subnormals are DECLINED by name, see [`hypot`]), `math.atan2` (CPython's special
//! cases, then the C runtime's `atan2`), `float ** 2` (`float_pow`: the C runtime's `pow` of the magnitude, never folded to a product), the
//! builtin `min`/`max` of two floats (the first wins unless the second is strictly better), the left fold of `sum`, and the exact comparisons of
//! a float with a `Fraction` (`Fraction(x) <= f`, `math.ceil(Fraction(x) * n)`).
//!
//! Proof: the unit vectors of this module were computed by the product interpreter (CPython 3.11.11 of Blender 4.5, MSVC, the UCRT `atan2`/`pow`
//! this module links as well); the field parity of the whole law (`tests/test_native_materialize_hot.py`, the shadow runs) covers the rest.

use cftuv_core::num::{IBig, UBig};
use cftuv_core::rat::Rat;

extern "C" {
    #[link_name = "atan2"]
    fn c_atan2(y: f64, x: f64) -> f64;
    #[link_name = "pow"]
    fn c_pow(x: f64, y: f64) -> f64;
}

const T27: f64 = 134217729.0;

/// The binary exponent `e` of `frexp(x)` (`x = m * 2**e`, `0.5 <= |m| < 1`) of a finite non-zero float.
fn frexp_exponent(x: f64) -> i32 {
    let bits = x.to_bits();
    let biased = ((bits >> 52) & 0x7ff) as i32;
    if biased == 0 {
        return frexp_exponent(x * 18446744073709551616.0) - 64;
    }
    biased - 1022
}

/// `ldexp(1.0, e)`, exact (normal or subnormal).
fn power_of_two(e: i32) -> f64 {
    if e >= -1022 {
        f64::from_bits(((e + 1023) as u64) << 52)
    } else {
        f64::from_bits(1u64 << (e + 1074))
    }
}

/// `vector_norm` of CPython 3.11 (`Modules/mathmodule.c`): the Veltkamp split, three compensation sums, one correction step.
fn vector_norm(vec: &mut [f64], max: f64, found_nan: bool) -> f64 {
    if max.is_infinite() {
        return max;
    }
    if found_nan {
        return f64::NAN;
    }
    if max == 0.0 || vec.len() <= 1 {
        return max;
    }
    let max_e = frexp_exponent(max);
    debug_assert!(max_e >= -1021, "hypot declines a subnormal largest component before vector_norm");
    let scale = power_of_two(-max_e);
    let mut csum = 1.0f64;
    let (mut frac1, mut frac2, mut frac3) = (0.0f64, 0.0f64, 0.0f64);
    for item in vec.iter() {
        let mut x = *item * scale;
        let t = x * T27;
        let hi = t - (t - x);
        let lo = x - hi;
        x = hi * hi;
        let mut old = csum;
        csum += x;
        frac1 += (old - csum) + x;
        x = 2.0 * hi * lo;
        old = csum;
        csum += x;
        frac2 += (old - csum) + x;
        frac3 += lo * lo;
    }
    let h = (csum - 1.0 + (frac1 + frac2 + frac3)).sqrt();
    let t = h * T27;
    let hi = t - (t - h);
    let lo = h - hi;
    let mut x = -hi * hi;
    let mut old = csum;
    csum += x;
    frac1 += (old - csum) + x;
    x = -2.0 * hi * lo;
    old = csum;
    csum += x;
    frac2 += (old - csum) + x;
    x = -lo * lo;
    old = csum;
    csum += x;
    frac3 += (old - csum) + x;
    x = csum - 1.0 + (frac1 + frac2 + frac3);
    (h + x / (2.0 * h)) / scale
}

/// `math.hypot(a, b)` of CPython 3.11, or `None` when the largest component is a non-zero SUBNORMAL: there CPython 3.11.11 rounds by its own
/// subnormal path, which differs by one ulp from both the DBL_MIN rescaling of 3.12+ and the division by the maximum (77 and 27 of 20000
/// random subnormal pairs). The callers decline such an input by name; a mesh never carries such a UV residual.
pub fn hypot(a: f64, b: f64) -> Option<f64> {
    let mut coordinates = [a.abs(), b.abs()];
    let mut max = 0.0f64;
    let mut found_nan = false;
    for x in coordinates {
        found_nan |= x.is_nan();
        if x > max {
            max = x;
        }
    }
    if !found_nan && max.is_finite() && max != 0.0 && max < f64::MIN_POSITIVE {
        return None;
    }
    Some(vector_norm(&mut coordinates, max, found_nan))
}

/// `math.atan2(y, x)` of CPython 3.11 (`m_atan2`): the special cases, then the C runtime's `atan2`.
pub fn atan2(y: f64, x: f64) -> f64 {
    if x.is_nan() || y.is_nan() {
        return f64::NAN;
    }
    let pi = std::f64::consts::PI;
    if y.is_infinite() {
        if x.is_infinite() {
            return if x.is_sign_positive() { (0.25 * pi).copysign(y) } else { (0.75 * pi).copysign(y) };
        }
        return (0.5 * pi).copysign(y);
    }
    if x.is_infinite() || y == 0.0 {
        if 1.0f64.copysign(x) == 1.0 {
            return 0.0f64.copysign(y);
        }
        return pi.copysign(y);
    }
    // SAFETY: a pure C runtime function of two doubles.
    unsafe { c_atan2(y, x) }
}

/// `x ** 2` for a float `x`: CPython's `float_pow` takes the magnitude of a negative base (`iv = -iv`; the exponent 2 is even, so the sign is
/// not restored) and calls the C runtime's `pow` (finite `x`; an overflow is an `OverflowError` in Python, `None` here).
pub fn square(x: f64) -> Option<f64> {
    // SAFETY: a pure C runtime function of two doubles; the exponent goes through `black_box` so the call is never folded into `x * x`.
    let value = unsafe { c_pow(x.abs(), std::hint::black_box(2.0)) };
    if value.is_infinite() && x.is_finite() {
        return None;
    }
    Some(value)
}

/// `max(a, b)` of two floats: `b` only when it is strictly greater.
pub fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// `min(a, b)` of two floats: `b` only when it is strictly smaller.
pub fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `left_fold_sum` (`_cpython311.py`): `0 + t0 + t1 + ...` without compensation.
pub fn left_fold_sum(terms: impl IntoIterator<Item = f64>) -> f64 {
    let mut total = 0.0f64;
    for term in terms {
        total += term;
    }
    total
}

/// `Fraction(x)` of a finite float, exactly.
pub fn rat_of_float(x: f64) -> Option<Rat> {
    if !x.is_finite() {
        return None;
    }
    if x == 0.0 {
        return Some(Rat::zero());
    }
    let bits = x.to_bits();
    let negative = bits >> 63 == 1;
    let biased = ((bits >> 52) & 0x7ff) as i64;
    let fraction = bits & ((1u64 << 52) - 1);
    let (mantissa, exponent) = if biased == 0 { (fraction, -1074i64) } else { (fraction | (1u64 << 52), biased - 1075) };
    let mut numerator = IBig::from(mantissa);
    if negative {
        numerator = -numerator;
    }
    if exponent >= 0 {
        Some(Rat::from_int(numerator << (exponent as usize)))
    } else {
        Some(Rat::reduced(numerator, UBig::ONE << ((-exponent) as usize)))
    }
}

/// `math.isfinite(x) and Fraction(x) <= budget` (`silhouette._within`).
pub fn within(x: f64, budget: &Rat) -> bool {
    match rat_of_float(x) {
        Some(value) => value <= *budget,
        None => false,
    }
}

/// `math.ceil(Fraction(x) * factor)` of a finite float as an unsigned integer (`None`: not finite, or negative).
pub fn ceil_scaled(x: f64, factor: u64) -> Option<u64> {
    let value = rat_of_float(x)?.mul(&Rat::from_int(IBig::from(factor)));
    let denominator = IBig::from(value.denominator().clone());
    let (quotient, remainder) = dashu_int::ops::DivRemEuclid::div_rem_euclid(value.numerator().clone(), denominator);
    let ceiling = if remainder.is_zero() { quotient } else { quotient + IBig::ONE };
    u64::try_from(&ceiling).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hypot_matches_known_values() {
        assert_eq!(hypot(3.0, 4.0), Some(5.0));
        assert_eq!(hypot(0.0, 1e308), Some(1e308));
        assert_eq!(hypot(-0.0, 0.0), Some(0.0));
        assert!(hypot(f64::NAN, 1.0).unwrap().is_nan());
        assert_eq!(hypot(f64::INFINITY, f64::NAN), Some(f64::INFINITY));
        assert_eq!(hypot(f64::MIN_POSITIVE, 0.0), Some(f64::MIN_POSITIVE));
        // two subnormals: CPython 3.11.11 gives (bits) 0x416bb9934e58b for the first pair, the 3.12+ rescaling ...58a; declined, never guessed
        for (a, b) in [(0x37a8d0cb996d3u64, 0x2260ff290ae97u64), (0x37b0bb716dac0, 0x1a79044c46a4e), (0x356589a63021b, 0x35a74b1e1f9d3)] {
            assert_eq!(hypot(f64::from_bits(a), f64::from_bits(b)), None);
            assert_eq!(hypot(-f64::from_bits(a), f64::from_bits(b)), None);
        }
        // `math.hypot` of CPython 3.11 where `sqrt(a*a + b*b)` rounds otherwise (taken from the interpreter).
        for (a, b, wanted) in [
            (0x4010fb13954ba954u64, 0x4020897b9de9e3efu64, 0x402296d965ca76ecu64),
            (0x40212b5bc7553b5c, 0x40072b32bc30fef6, 0x40221ebc65333b0f),
            (0x3fe3128720c2a87c, 0x40007a08d0f6d232, 0x40012719886d45d7),
            (0x400f46a6612b0cae, 0x40216dae35bdc3a8, 0x40231a1dfcd5ecbe),
        ] {
            assert_eq!(hypot(f64::from_bits(a), f64::from_bits(b)).unwrap().to_bits(), wanted);
        }
    }

    #[test]
    fn the_square_of_a_negative_base_is_the_square_of_its_magnitude() {
        // `x ** 2` of CPython 3.11.11 (Blender 4.5), bits
        for (x, wanted) in [(0xc06dbee477bd3073u64, 0x40eba670d873fa78u64), (0xc08101d66306f0cf, 0x411213e7c873bbf5), (0xc0771f4af15e4303, 0x4100b51a54daaa67), (0x9eb67e9c127b6e74, 0x00000000000007e8)] {
            assert_eq!(square(f64::from_bits(x)).unwrap().to_bits(), wanted);
            assert_eq!(square(-f64::from_bits(x)).unwrap().to_bits(), wanted);
        }
        assert_eq!(square(-1e200), None);
    }

    #[test]
    fn exact_float_comparisons() {
        assert!(within(0.005, &Rat::new(IBig::from(1), IBig::from(200)).unwrap()) == (rat_of_float(0.005).unwrap() <= Rat::new(IBig::from(1), IBig::from(200)).unwrap()));
        assert_eq!(ceil_scaled(0.5, 1000), Some(500));
        assert_eq!(ceil_scaled(1e-9, 1_000_000_000), Some(2), "1e-9 is a little above 10**-9");
        assert_eq!(ceil_scaled(0.0, 7), Some(0));
        assert_eq!(py_max(0.0, -0.0).to_bits(), 0.0f64.to_bits());
        assert_eq!(py_min(1.0, -0.0).to_bits(), (-0.0f64).to_bits());
    }
}
