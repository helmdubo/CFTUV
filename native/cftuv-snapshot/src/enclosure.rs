//! Rigorous rational enclosures of `sqrt(x)` and `cos(pi * r)`, as `reference/angle_certificate_sign.py` takes them.
//!
//! The Python path decides the sign of `dot - c * sqrt(N)` on a BOX: `c` from a 256-bit `mpmath.iv` enclosure of `cos(pi * r)`, `sqrt(N)` from an integer enclosure. It declares a sign only when the
//! whole box is on one side. The port decides on the same kind of box, and the one rule that makes its verdicts agree with Python's is: THE PORT'S BOX IS NEVER NARROWER THAN PYTHON'S. Then a sign the
//! port proves is a sign Python proves too (its box is inside the port's), and an angle on which Python falls back to the exact path is one the port declines or decides by exact arithmetic - never
//! one it decides by an enclosure Python could not have used (that would leave different counters behind).
//!
//! * `sqrt_enclosure` is the integer formula of Python, copied: the two sides are the same numbers.
//! * `cos_pi_enclosure` computes `cos` in fixed point (400 fractional bits) with an error bound that is far below what the result is then widened to: [`COS_HALF_WIDTH`], `2^-240` on each side, which is
//!   wider than the `2^-256`-relative interval `mpmath` returns. The widening is the guarantee; the test `cosine_box_contains_the_mpmath_box` (in the Python differential tests) compares both on
//!   thousands of `r`.

use std::sync::OnceLock;

use cftuv_core::num::{self, IBig, UBig};
use cftuv_core::rat::Rat;
use dashu_int::ops::{BitTest, SquareRoot};

/// Bits of the `sqrt` enclosure (Python `_ENCLOSURE_BITS`).
pub const ENCLOSURE_BITS: usize = 256;

/// Fractional bits of the fixed-point computation of `cos`.
const FIXED: usize = 400;
// The fixed-point error of `pi` (Machin's series: about a hundred truncations of one unit of `2^-FIXED`, sixteen times over) and of the `cos` series built on it (a few hundred operations on numbers
// below 2, one unit each) stays below `2^30` units, i.e. `2^-370`: ten orders of magnitude of 2 under the `2^-240` the result is widened to. The Python differential test checks it against `mpmath` at 600 bits.
/// The half width of the box around `cos(pi r)`: `2^-240`.
const COS_HALF_WIDTH_BITS: usize = 240;

/// `[lo, hi]` with `lo <= sqrt(value) <= hi`, relative width about `2^-ENCLOSURE_BITS`; `value > 0`. The formula of `angle_certificate_sign._sqrt_enclosure`.
pub fn sqrt_enclosure(value: &Rat) -> (Rat, Rat) {
    let radicand = num::magnitude(value.numerator()) * value.denominator();
    let half_bits = radicand.bit_len() / 2;
    let shift = ENCLOSURE_BITS.saturating_sub(half_bits);
    let root = (&radicand << (2 * shift)).sqrt();
    let scale = value.denominator() << shift;
    let low = Rat::reduced(IBig::from(root.clone()), scale.clone());
    let high = Rat::reduced(IBig::from(root + UBig::ONE), scale);
    (low, high)
}

fn pi_fixed() -> &'static IBig {
    static PI: OnceLock<IBig> = OnceLock::new();
    PI.get_or_init(|| {
        // pi = 16 atan(1/5) - 4 atan(1/239), atan(1/m) = sum (-1)^k / ((2k + 1) m^(2k+1)), in units of 2^-FIXED
        let one = IBig::ONE << FIXED;
        let atan_inverse = |m: u32| -> IBig {
            let m_squared = IBig::from(m) * IBig::from(m);
            let mut power = &one / IBig::from(m);
            let mut sum = IBig::ZERO;
            let mut k = 0u32;
            while power != IBig::ZERO {
                let term = &power / IBig::from(2 * k + 1);
                if k % 2 == 0 {
                    sum += term;
                } else {
                    sum -= term;
                }
                power = &power / &m_squared;
                k += 1;
            }
            sum
        };
        atan_inverse(5) * IBig::from(16) - atan_inverse(239) * IBig::from(4)
    })
}

/// `[lo, hi]` with `lo <= cos(pi * numerator / denominator) <= hi` and a half width of `2^-240` around the value.
pub fn cos_pi_enclosure(numerator: &IBig, denominator: &UBig) -> (Rat, Rat) {
    // cos(pi t) has period 2 in t and cos(pi (2 - t)) = cos(pi t); reduce to t in [0, 1], then cos(pi t) = -cos(pi (1 - t)) for t > 1/2
    let double = IBig::from(denominator.clone()) * IBig::from(2);
    let mut numerator_reduced = numerator.clone() % &double;
    if num::is_negative(&numerator_reduced) {
        numerator_reduced += &double;
    }
    let denominator_signed = IBig::from(denominator.clone());
    if numerator_reduced > denominator_signed {
        numerator_reduced = &double - numerator_reduced;
    }
    let mut negate = false;
    if &numerator_reduced * IBig::from(2) > denominator_signed {
        negate = true;
        numerator_reduced = &denominator_signed - numerator_reduced;
    }
    // the argument is exactly pi/2: cos is exactly 0 (the common case: every exact 270 degree corner has the interval [0.5, 0.5]), no series
    if &numerator_reduced * IBig::from(2) == denominator_signed {
        let half = Rat::reduced(IBig::ONE, UBig::ONE << COS_HALF_WIDTH_BITS);
        return (half.neg(), half);
    }
    // x = pi * s with s = numerator_reduced / denominator in [0, 1/2]
    let x = (pi_fixed() * &numerator_reduced) / &denominator_signed;
    let x_squared = (&x * &x) >> FIXED;
    let one = IBig::ONE << FIXED;
    let mut term = one.clone();
    let mut sum = one;
    let mut k = 0u32;
    loop {
        // term_{k+1} = -term_k * x^2 / ((2k+1)(2k+2))
        let divisor = IBig::from((2 * k + 1) as u64 * (2 * k + 2) as u64);
        term = -((&term * &x_squared) >> FIXED) / divisor;
        if term == IBig::ZERO {
            break;
        }
        sum += &term;
        k += 1;
        if k > 400 {
            break;
        }
    }
    let center = if negate { -sum } else { sum };
    // the value is `center / 2^FIXED` within about `2^-370`; the box is `center / 2^FIXED +- 2^-240`, which is wider by far
    let center_rat = Rat::reduced(center, UBig::ONE << FIXED);
    let half = Rat::reduced(IBig::ONE, UBig::ONE << COS_HALF_WIDTH_BITS);
    (center_rat.sub(&half), center_rat.add(&half))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rat(n: i64, d: u64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    fn to_f64(value: &Rat) -> f64 {
        cftuv_core::pyfloat::rat_to_f64(value).unwrap()
    }

    #[test]
    fn the_sqrt_box_contains_the_root_and_is_tight() {
        for (n, d) in [(2u64, 1u64), (1, 4), (9, 16), (1, 3), (1_000_000_000_000_000_000, 7), (1, 1)] {
            let value = rat(n as i64, d);
            let (low, high) = sqrt_enclosure(&value);
            assert!(low.mul(&low) <= value && value <= high.mul(&high), "{n}/{d}");
            let width = high.sub(&low);
            assert!(width < rat(1, 1u64 << 40), "{n}/{d}");
        }
    }

    #[test]
    fn pi_has_the_right_digits() {
        let pi = pi_fixed();
        let scaled = (pi * IBig::from(10u8).pow(30)) >> FIXED;
        assert_eq!(scaled.to_string(), "3141592653589793238462643383279");
    }

    #[test]
    fn cosines_of_known_angles() {
        let cases: [(i64, u64, f64); 7] = [(0, 1, 1.0), (1, 3, 0.5), (1, 2, 0.0), (2, 3, -0.5), (1, 1, -1.0), (5, 4, -(0.5f64).sqrt()), (-1, 3, 0.5)];
        for (n, d, expected) in cases {
            let (low, high) = cos_pi_enclosure(&IBig::from(n), &UBig::from(d));
            assert!(low <= high);
            let middle = to_f64(&low.add(&high).mul(&rat(1, 2)));
            assert!((middle - expected).abs() < 1e-15, "{n}/{d}: {middle} vs {expected}");
            let width = high.sub(&low);
            assert!(width < rat(1, 1u64 << 60), "{n}/{d}: the box is narrow");
        }
        // cos(pi/2) = 0 is inside its box: the box straddles zero, which is why exact zero needs its own rule
        let (low, high) = cos_pi_enclosure(&IBig::ONE, &UBig::from(2u8));
        assert!(low.signum() <= 0 && high.signum() >= 0);
    }

    #[test]
    fn a_28_digit_argument_is_reduced_and_enclosed() {
        // 0.110405986615677725844280220 pi
        let numerator: IBig = "110405986615677725844280220".parse().unwrap();
        let denominator = UBig::from(10u8).pow(27);
        let (low, high) = cos_pi_enclosure(&numerator, &denominator);
        let value = to_f64(&low.add(&high).mul(&rat(1, 2)));
        let expected = (std::f64::consts::PI * 0.110405986615677725844280220).cos();
        assert!((value - expected).abs() < 1e-15, "{value} {expected}");
        // the same argument shifted by 2 is the same cosine, shifted by 1 the negative
        let shifted = &numerator + IBig::from(2) * IBig::from(denominator.clone());
        let (low2, high2) = cos_pi_enclosure(&shifted, &denominator);
        assert_eq!((low.clone(), high.clone()), (low2, high2));
        let flipped = &numerator + IBig::from(denominator.clone());
        let (low3, high3) = cos_pi_enclosure(&flipped, &denominator);
        assert_eq!((low3, high3), (high.neg(), low.neg()));
    }
}
