//! A binary64 coordinate read exactly (`float.as_integer_ratio`) and the integers it becomes.
//!
//! A finite `f64` is `m * 2^e` with `m` odd (or zero). Everything the oracle does with the coordinates is an exact computation on those values, and an exact computation on dyadic rationals
//! is a computation on integers over one power of two: [`Scaled`] holds a whole map of coordinates as integers over the common `2^-e_min`, so no division and no reduction happens
//! before the answer is written out.

use cftuv_core::num::IBig;

/// `m * 2^e` with `m` odd, or `m = 0` and `e = 0`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dyadic {
    pub m: i64,
    pub e: i32,
}

impl Dyadic {
    pub const ZERO: Dyadic = Dyadic { m: 0, e: 0 };

    /// `None` for a NaN or an infinity (a `LocalPoint3V1` is finite by construction; the reader declines otherwise).
    pub fn of(x: f64) -> Option<Dyadic> {
        if !x.is_finite() {
            return None;
        }
        let bits = x.to_bits();
        let negative = bits >> 63 != 0;
        let exponent = ((bits >> 52) & 0x7ff) as i32;
        let fraction = (bits & ((1u64 << 52) - 1)) as i64;
        let (mut m, mut e) = if exponent == 0 { (fraction, -1074) } else { (fraction | (1i64 << 52), exponent - 1075) };
        if m == 0 {
            return Some(Dyadic::ZERO);
        }
        let zeros = m.trailing_zeros();
        m >>= zeros;
        e += zeros as i32;
        Some(Dyadic { m: if negative { -m } else { m }, e })
    }

    pub fn is_zero(self) -> bool {
        self.m == 0
    }

    /// `floor(self * 2^exponent + 1/2)` (`robust.grid.snap_value`: `(2 * numerator + denominator) // (2 * denominator)` of `value * scale`), or `None` when the result might not fit the
    /// `i128` fast path (the caller then runs [`Dyadic::snap_big`]).
    pub fn snap_fixed(self, exponent: u32) -> Option<i128> {
        if self.m == 0 {
            return Some(0);
        }
        let shift = self.e + exponent as i32;
        if shift >= 0 {
            // `m` has at most 53 bits: stay far from the edge of the i128
            return if (shift as u32) + 53 > 100 { None } else { Some((self.m as i128) << shift) };
        }
        let down = (-shift) as u32;
        if down >= 120 {
            // |x * scale| < 2^53 * 2^-120: the sum with one half floors to zero
            return Some(0);
        }
        Some(((2 * (self.m as i128)) + (1i128 << down)) >> (down + 1))
    }

    /// The same, in big integers (no fast-path limit).
    pub fn snap_big(self, exponent: u32) -> IBig {
        if self.m == 0 {
            return IBig::ZERO;
        }
        let shift = self.e + exponent as i32;
        if shift >= 0 {
            return IBig::from(self.m) << (shift as usize);
        }
        let down = (-shift) as usize;
        let numerator = IBig::from(2 * (self.m as i128)) + (IBig::ONE << down);
        numerator >> (down + 1)
    }

    /// `self * 2^-e_min` as an integer (`e_min <= self.e` for a non-zero value).
    pub fn lifted(self, e_min: i32) -> IBig {
        if self.m == 0 {
            return IBig::ZERO;
        }
        IBig::from(self.m) << ((self.e - e_min) as usize)
    }
}

/// The smallest exponent of the non-zero values (`None` when all are zero).
pub fn minimum_exponent(values: &[[Dyadic; 3]]) -> Option<i32> {
    values.iter().flat_map(|point| point.iter()).filter(|item| !item.is_zero()).map(|item| item.e).min()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn value_of(d: Dyadic) -> f64 {
        d.m as f64 * 2f64.powi(d.e)
    }

    #[test]
    fn decomposition_is_exact_for_ordinary_subnormal_and_negative_numbers() {
        for x in [0.0, -0.0, 1.0, -1.0, 0.1, 0.5, 3.0, 1.0e-3, 123456.789, f64::MIN_POSITIVE, 5e-324, -2.5e-310, 1.0e300, 0.30000000000000004] {
            let d = Dyadic::of(x).unwrap();
            if d.m != 0 {
                assert!(d.m % 2 != 0, "m is odd for {x}");
            }
            if x.abs() > 1e-300 && x.abs() < 1e290 {
                assert_eq!(value_of(d), x, "{x}");
            }
        }
        assert_eq!(Dyadic::of(f64::NAN), None);
        assert_eq!(Dyadic::of(f64::INFINITY), None);
        assert_eq!(Dyadic::of(0.5), Some(Dyadic { m: 1, e: -1 }));
        assert_eq!(Dyadic::of(-0.75), Some(Dyadic { m: -3, e: -2 }));
    }

    #[test]
    fn snapping_is_floor_of_the_scaled_value_plus_one_half() {
        // value * 2^k + 1/2, floored, by exact rational arithmetic on i128
        let reference = |x: f64, k: u32| -> i128 {
            let d = Dyadic::of(x).unwrap();
            if d.m == 0 {
                return 0;
            }
            let shift = d.e + k as i32;
            if shift >= 0 {
                return (d.m as i128) << shift;
            }
            let down = (-shift) as u32;
            let numerator = 2 * (d.m as i128) + (1i128 << down);
            numerator.div_euclid(1i128 << (down + 1))
        };
        for x in [0.0, 0.5, -0.5, 1.5, -1.5, 0.25, 0.125, 0.3, -0.3, 7.0, 1000.001, -1000.001, 1e-9, -1e-9, 2.5e-7] {
            for k in [0, 1, 2, 5, 7, 8, 13, 20, 30] {
                let d = Dyadic::of(x).unwrap();
                assert_eq!(d.snap_fixed(k), Some(reference(x, k)), "{x} {k}");
                assert_eq!(d.snap_big(k), IBig::from(reference(x, k)), "{x} {k}");
            }
        }
        // the half is rounded up, also for negative values
        assert_eq!(Dyadic::of(0.5).unwrap().snap_fixed(0), Some(1));
        assert_eq!(Dyadic::of(-0.5).unwrap().snap_fixed(0), Some(0));
        assert_eq!(Dyadic::of(-1.5).unwrap().snap_fixed(0), Some(-1));
        assert_eq!(Dyadic::of(2.5).unwrap().snap_fixed(0), Some(3));
    }

    #[test]
    fn lifting_matches_the_value() {
        let a = Dyadic::of(0.75).unwrap();
        let b = Dyadic::of(3.0).unwrap();
        let e_min = minimum_exponent(&[[a, b, Dyadic::ZERO]]).unwrap();
        assert_eq!(e_min, -2);
        assert_eq!(a.lifted(e_min), IBig::from(3));
        assert_eq!(b.lifted(e_min), IBig::from(12));
    }
}
