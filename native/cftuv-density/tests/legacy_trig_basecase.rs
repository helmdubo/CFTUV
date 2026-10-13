use cftuv_density::legacy_trig::pi_fixed;
use cftuv_density::legacy_trig_basecase::{
    anchor, cos_sin_basecase, BasecaseError, UnsupportedProfile, ANCHOR_COUNT,
    MAX_FIXED_PRECISION_BITS, MIN_FIXED_PRECISION_BITS,
};
use cftuv_density::{IBig, Radical, UBig};
use cftuv_radical::arith::magnitude;
use cftuv_radical::Term;

fn hex(value: &str) -> IBig {
    IBig::from_str_radix(value, 16).unwrap()
}

// Независимая finite-модель exponential_series(...,410,2) с exact integer sqrt.
// Фиксированное source int(0.5*sqrt(410)) равно 10; универсального seed proof нет.
fn exact_isqrt_anchor_model(index: u32) -> (IBig, IBig) {
    let precision = 410i32;
    let xmag = if index == 0 {
        -precision
    } else {
        32 - index.leading_zeros() as i32 + 402 - precision
    };
    let r = (xmag + 10).max(0);
    let extra = 10 + 2 * r.max(-xmag);
    let wp = precision + extra;
    let x = (IBig::from(index) << 402usize) << (extra - r) as usize;
    let one = IBig::ONE << wp as usize;
    let x2 = (&x * &x) >> wp as usize;
    let x4 = (&x2 * &x2) >> wp as usize;
    let mut a = x2.clone();
    let mut s0 = IBig::ZERO;
    let mut s1 = IBig::ZERO;
    let mut k = 2u32;
    while !a.is_zero() {
        a /= IBig::from((k - 1) * k);
        s0 += &a;
        k += 2;
        a /= IBig::from((k - 1) * k);
        s1 += &a;
        k += 2;
        a = (&a * &x4) >> wp as usize;
    }
    s1 = (&x2 * &s1) >> wp as usize;
    let mut cos = s1 - s0 + &one;
    for _ in 0..r {
        cos = (&cos * &cos >> (wp - 1) as usize) - &one;
    }
    let radicand = magnitude(&((&one << wp as usize) - &cos * &cos));
    let sin = Radical::floor_root(&Term {
        radicand,
        numerator: IBig::ONE,
        denominator: UBig::ONE,
    });
    // Source raw410 сравнен с actual отдельно; таблица хранит raw400 после >>10.
    (
        cos >> (extra + 10) as usize,
        IBig::from(sin >> (extra + 10) as usize),
    )
}

#[test]
fn all_403_actual_anchors_match_finite_integer_isqrt_model() {
    assert_eq!(ANCHOR_COUNT, 403);
    for index in 0..ANCHOR_COUNT {
        assert_eq!(
            anchor(index).unwrap(),
            exact_isqrt_anchor_model(index),
            "anchor {index}"
        );
    }
}

#[test]
fn anchor_boundaries_and_zero_keep_source_cache_tail_precision() {
    for bits in [8, 9, 16, 17, 53, 160, 180, 256, 399, 400] {
        assert_eq!(
            cos_sin_basecase(&IBig::ZERO, bits).unwrap(),
            (IBig::ONE << bits as usize, IBig::ZERO)
        );
        for index in 0..ANCHOR_COUNT {
            let argument = IBig::from(index) << (bits - 8) as usize;
            let (cos, sin) = anchor(index).unwrap();
            assert_eq!(
                cos_sin_basecase(&argument, bits).unwrap(),
                (cos >> (400 - bits) as usize, sin >> (400 - bits) as usize)
            );
        }
    }
}

#[test]
fn recorded_source_cases_cover_negative_tail_small_inputs_and_last_cell() {
    let cases = [
        (17, "afff", "1e20c", "ac8b"),
        (32, "57000001", "f15bc8ad", "5555c188"),
        (32, "57ffffff", "f105fa4d", "5646f27c"),
        (53, "32400000000001", "3f6a885c208", "1fffffc12991f8"),
        (160, "1", "10000000000000000000000000000000000000000", "1"),
        (400,
         "19200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001",
         "1fb5442e104a9130f3d0cc2aee5a74f1e3df2ddc363c684d695f80d4b81bfd05359baff851ab5d12f3b13f343dab312b19",
         "fffffe094c8fc0cf163aafac3f94ac8bd3d11a89f4ac941279a1101abf8dbe590c210a6607c373cd862f7185bbd36b5a9aa5"),
    ];
    for (bits, input, cos, sin) in cases {
        assert_eq!(
            cos_sin_basecase(&hex(input), bits).unwrap(),
            (hex(cos), hex(sin))
        );
    }
}

#[test]
fn cache_tail_is_not_replaced_by_a_fresh_once_computed_series() {
    // Actual mpmath1.3: exponential_series(45055,17,2) даёт sin=44172,
    // но canonical cached basecase даёт 44171. Оба значения — legacy-данные.
    let cached = cos_sin_basecase(&IBig::from(45055), 17).unwrap();
    assert_eq!(cached, (IBig::from(123404), IBig::from(44171)));
    assert_ne!(cached, (IBig::from(123404), IBig::from(44172)));
}

#[test]
fn returned_anchor_mutation_cannot_poison_future_evaluations() {
    let original = anchor(87).unwrap();
    let mut caller_copy = original.clone();
    caller_copy.0 = IBig::ZERO;
    caller_copy.1 = IBig::ZERO;
    assert_ne!(caller_copy, original);
    assert_eq!(anchor(87).unwrap(), original);
    assert_eq!(
        cos_sin_basecase(&hex("57000001"), 32).unwrap(),
        (hex("f15bc8ad"), hex("5555c188"))
    );
}

#[test]
fn finite_profile_and_exact_dyadic_domain_are_explicit() {
    for bits in [0, 7, 401, u32::MAX] {
        assert_eq!(
            cos_sin_basecase(&IBig::ZERO, bits),
            Err(BasecaseError::UnsupportedProfile(
                UnsupportedProfile::Precision {
                    requested: bits,
                    minimum: MIN_FIXED_PRECISION_BITS,
                    maximum: MAX_FIXED_PRECISION_BITS
                }
            ))
        );
    }
    assert_eq!(
        anchor(403),
        Err(BasecaseError::UnsupportedProfile(
            UnsupportedProfile::AnchorIndex {
                requested: 403,
                count: 403
            }
        ))
    );
    for bits in [8, 53, 160, 400] {
        let maximum = IBig::from(pi_fixed(bits - 1).unwrap());
        assert!(cos_sin_basecase(&maximum, bits).is_ok());
        for argument in [IBig::from(-1), &maximum + IBig::ONE] {
            assert_eq!(
                cos_sin_basecase(&argument, bits),
                Err(BasecaseError::UnsupportedProfile(
                    UnsupportedProfile::ReducedArgument {
                        requested: argument,
                        minimum: IBig::ZERO,
                        maximum: maximum.clone(),
                        bits
                    }
                ))
            );
        }
    }
}
