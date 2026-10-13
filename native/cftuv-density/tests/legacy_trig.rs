use cftuv_density::legacy_mpf::{Precision, Rounding};
use cftuv_density::legacy_trig::{
    mpf_pi, pi_fixed, TrigError, UnsupportedProfile, CALIBRATED_LEGACY_PI_MEMO_REQUEST_MAX,
    MAX_MPF_PI_PRECISION_BITS, MAX_PI_FIXED_BITS, PI_PREFIX_BITS, PI_PREFIX_HEX,
};
use cftuv_density::{IBig, Rational, UBig};

fn add(a: &Rational, b: &Rational) -> Rational {
    let (an, ad) = a.parts();
    let (bn, bd) = b.parts();
    Rational::new(
        an * IBig::from(bd.clone()) + bn * IBig::from(ad.clone()),
        ad * bd,
    )
    .unwrap()
}

fn scaled(value: &Rational, multiplier: i64) -> Rational {
    let (n, d) = value.parts();
    Rational::new(n * IBig::from(multiplier), d.clone()).unwrap()
}

// 500 членов чередующегося ряда: чётная частичная сумма — lower,
// она плюс следующий положительный член — upper. Только целые/Fraction.
fn atan_reciprocal_enclosure(q: u32) -> (Rational, Rational) {
    let q = UBig::from(q);
    let q2 = &q * &q;
    let mut power = q;
    let mut sum = Rational::integer(IBig::ZERO);
    for k in 0..500u32 {
        let term = Rational::new(
            IBig::from(if k % 2 == 0 { 1 } else { -1 }),
            &power * UBig::from(2 * k + 1),
        )
        .unwrap();
        sum = add(&sum, &term);
        power *= &q2;
    }
    let next = Rational::new(IBig::ONE, power * UBig::from(1001u32)).unwrap();
    let upper = add(&sum, &next);
    (sum, upper)
}

fn positive_floor_dyadic(value: &Rational, bits: u32) -> UBig {
    let (n, d) = value.parts();
    assert!(n > &IBig::ZERO);
    UBig::try_from((n << bits as usize) / IBig::from(d.clone())).unwrap()
}

#[test]
fn independent_machin_enclosure_certifies_every_bit_of_pi_prefix() {
    let (a_lo, a_hi) = atan_reciprocal_enclosure(5);
    let (b_lo, b_hi) = atan_reciprocal_enclosure(239);
    // Machin: π = 16 atan(1/5) - 4 atan(1/239).
    let lo = add(&scaled(&a_lo, 16), &scaled(&b_hi, -4));
    let hi = add(&scaled(&a_hi, 16), &scaled(&b_lo, -4));
    let prefix = UBig::from_str_radix(PI_PREFIX_HEX, 16).unwrap();
    assert_eq!(positive_floor_dyadic(&lo, PI_PREFIX_BITS), prefix);
    assert_eq!(positive_floor_dyadic(&hi, PI_PREFIX_BITS), prefix);
    for bits in 0..=MAX_PI_FIXED_BITS {
        assert_eq!(
            pi_fixed(bits).unwrap(),
            &prefix >> (PI_PREFIX_BITS - bits) as usize
        );
    }
}

#[test]
fn constant_rounding_keeps_legacy_order_and_raw_tuple_normalization() {
    for rounding in [Rounding::Floor, Rounding::TowardZero, Rounding::Nearest] {
        assert_eq!(
            mpf_pi(Precision::new(2).unwrap(), rounding)
                .unwrap()
                .raw_tuple(),
            (0, UBig::from(3u32), 0, 2)
        );
    }
    for rounding in [Rounding::Ceiling, Rounding::AwayFromZero] {
        assert_eq!(
            mpf_pi(Precision::new(2).unwrap(), rounding)
                .unwrap()
                .raw_tuple(),
            (0, UBig::ONE, 2, 1)
        );
    }
    assert_eq!(
        mpf_pi(Precision::new(53).unwrap(), Rounding::Nearest)
            .unwrap()
            .raw_tuple(),
        (0, UBig::from(884279719003555u64), -48, 50)
    );
    assert_eq!(pi_fixed(0).unwrap(), UBig::from(3u32));
    assert_eq!(pi_fixed(8).unwrap(), UBig::from(804u32));
    assert!(mpf_pi(Precision::new(160).unwrap(), Rounding::Nearest).is_ok());
    assert!(mpf_pi(Precision::new(512).unwrap(), Rounding::Nearest).is_ok());
}

#[test]
fn prototype_limits_are_explicit_and_not_density_outcomes() {
    assert_eq!(
        pi_fixed(801),
        Err(TrigError::UnsupportedProfile(
            UnsupportedProfile::PiFixedBits {
                requested: 801,
                maximum: MAX_PI_FIXED_BITS
            }
        ))
    );
    assert_eq!(
        mpf_pi(Precision::new(513).unwrap(), Rounding::Nearest),
        Err(TrigError::UnsupportedProfile(
            UnsupportedProfile::MpfPiPrecision {
                requested: 513,
                minimum: 2,
                maximum: MAX_MPF_PI_PRECISION_BITS
            }
        ))
    );
    assert_eq!(CALIBRATED_LEGACY_PI_MEMO_REQUEST_MAX, 1536);
}

#[test]
fn three_observed_history_sensitive_pi_reads_are_named_counterexamples() {
    let prefix = UBig::from_str_radix(PI_PREFIX_HEX, 16).unwrap();
    // actual mpmath 1.3.0 Python backend: requested → memo_precision → read_precision.
    // Эти raw-чтения намеренно вне API: exact prefix не исправляет legacy oracle.
    let legacy901 = concat!(
        "6487ed5110b4611a62633145c06e0e68948127044533e63a0105df531d89cd9128a5",
        "043cc71a026ef7ca8cd9e69d218d98158536f92f8a1ba7f09ab6b6a8e122f242dabb",
        "312f3f637a262174d31bf6b585ffae5b7a035bf6f71c35fdad44cfd2d74f9208be258",
        "ff324943328f6722d9ee0",
    );
    let legacy902 = concat!(
        "c90fdaa22168c234c4c6628b80dc1cd129024e088a67cc74020bbea63b139b22514a0",
        "8798e3404ddef9519b3cd3a431b302b0a6df25f14374fe1356d6d51c245e485b5766",
        "25e7ec6f44c42e9a637ed6b0bff5cb6f406b7edee386bfb5a899fa5ae9f24117c4b1",
        "fe649286651ece45b3dc1",
    );
    let fixtures = [
        (849u32, 901u32, 901u32, legacy901),
        (850, 902, 901, legacy901),
        (850, 902, 902, legacy902),
    ];
    for (requested, memo_bits, read_bits, actual_hex) in fixtures {
        assert_eq!(memo_bits, requested * 105 / 100 + 10);
        let exact_read = &prefix >> (PI_PREFIX_BITS - read_bits) as usize;
        let observed_legacy_read = UBig::from_str_radix(actual_hex, 16).unwrap();
        assert_ne!(observed_legacy_read, exact_read);
        assert_eq!(exact_read - observed_legacy_read, UBig::ONE);
        assert!(matches!(
            pi_fixed(read_bits),
            Err(TrigError::UnsupportedProfile(_))
        ));
    }
}
