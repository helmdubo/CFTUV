use cftuv_density::legacy_interval::LegacyInterval;
use cftuv_density::legacy_mpf::{
    LegacyError, Mpf, Precision, Rounding, UnsupportedProfile, MAX_ABS_EXPONENT,
};
use cftuv_density::{IBig, UBig};

fn p(bits: u32) -> Precision {
    Precision::new(bits).unwrap()
}
fn exact(man: i64, exp: i64) -> Mpf {
    Mpf::from_man_exp(IBig::from(man), exp, None, Rounding::TowardZero).unwrap()
}
fn raw(sign: u8, man: &str, exp: i64, bc: i64) -> (u8, UBig, i64, i64) {
    (sign, UBig::from_str_radix(man, 10).unwrap(), exp, bc)
}

#[test]
fn normalization_is_canonical_directed_and_nearest_ties_even() {
    assert_eq!(exact(12, -2).raw_tuple(), raw(0, "3", 0, 2));
    assert_eq!(
        Mpf::from_man_exp(IBig::ZERO, i64::MAX, None, Rounding::Floor).unwrap(),
        Mpf::zero()
    );
    assert_eq!(
        Mpf::from_raw_tuple(1, UBig::ZERO, 0, 0).unwrap(),
        Mpf::zero()
    );
    for (man, round, expected) in [
        (40, Rounding::Nearest, raw(0, "1", 5, 1)),
        (56, Rounding::Nearest, raw(0, "1", 6, 1)),
        (-40, Rounding::Nearest, raw(1, "1", 5, 1)),
        (33, Rounding::Floor, raw(0, "1", 5, 1)),
        (33, Rounding::Ceiling, raw(0, "3", 4, 2)),
        (-33, Rounding::Floor, raw(1, "3", 4, 2)),
        (-33, Rounding::Ceiling, raw(1, "1", 5, 1)),
        (-33, Rounding::TowardZero, raw(1, "1", 5, 1)),
        (-33, Rounding::AwayFromZero, raw(1, "3", 4, 2)),
    ] {
        assert_eq!(
            Mpf::from_man_exp(IBig::from(man), 0, Some(p(2)), round)
                .unwrap()
                .raw_tuple(),
            expected
        );
    }
}

#[test]
fn rational_conversion_matches_recorded_mpmath_1_3_0_tuples_at_required_precisions() {
    // Фактический libmpf.from_rational, Python backend: raw-кортежи направленного/nearest округления.
    let cases = [
        (1657, 5120, 8, Rounding::Floor, raw(0, "165", -9, 8)),
        (1657, 5120, 8, Rounding::Ceiling, raw(0, "83", -8, 7)),
        (
            3647,
            5120,
            53,
            Rounding::Nearest,
            raw(0, "6415870250398515", -53, 53),
        ),
        (
            -1657,
            5120,
            160,
            Rounding::Floor,
            raw(
                1,
                "236494942681377552291357985137781344098621115597",
                -159,
                158,
            ),
        ),
        (
            1657,
            5120,
            180,
            Rounding::Ceiling,
            raw(
                0,
                "247982921017068148271462990623834210669555734908030157",
                -179,
                178,
            ),
        ),
        (
            3647,
            5120,
            256,
            Rounding::Nearest,
            raw(
                0,
                "41239623969579312959937830305340312494226136140194446977724786023130779667661",
                -255,
                255,
            ),
        ),
    ];
    for (n, d, bits, round, expected) in cases {
        assert_eq!(
            Mpf::from_rational(IBig::from(n), IBig::from(d), p(bits), round)
                .unwrap()
                .raw_tuple(),
            expected
        );
    }
}

#[test]
fn enormous_addition_gaps_perturb_without_materializing_the_gap() {
    let large = exact(1, MAX_ABS_EXPONENT);
    let tiny = exact(1, -MAX_ABS_EXPONENT);
    let floor = large.add(&tiny, p(160), Rounding::Floor).unwrap();
    let ceil = large.add(&tiny, p(160), Rounding::Ceiling).unwrap();
    assert_eq!(floor, large);
    assert_eq!(
        ceil.raw_tuple(),
        (
            0,
            (UBig::ONE << 159usize) + UBig::ONE,
            MAX_ABS_EXPONENT - 159,
            160
        )
    );
    let just_below = large.sub(&tiny, p(160), Rounding::Floor).unwrap();
    assert_eq!(
        just_below.raw_tuple(),
        (
            0,
            (UBig::ONE << 160usize) - UBig::ONE,
            MAX_ABS_EXPONENT - 160,
            160
        )
    );
    assert_eq!(large.sub(&tiny, p(160), Rounding::Ceiling).unwrap(), large);
    assert_eq!(
        large
            .neg_exact()
            .sub(&tiny, p(160), Rounding::Ceiling)
            .unwrap(),
        large.neg_exact()
    );
}

#[test]
fn scalar_special_numbers_keep_the_libmpf_results_and_error_order() {
    let zero = Mpf::zero();
    let inf = Mpf::positive_infinity();
    let ninf = Mpf::negative_infinity();
    let nan = Mpf::nan();
    assert!(inf.mul(&zero, p(160), Rounding::Floor).unwrap().is_nan());
    assert!(inf.add(&ninf, p(160), Rounding::Ceiling).unwrap().is_nan());
    assert!(zero.div(&nan, p(160), Rounding::Floor).unwrap().is_nan());
    assert!(inf.div(&inf, p(160), Rounding::Floor).unwrap().is_nan());
    assert_eq!(
        inf.div(&zero, p(160), Rounding::Floor),
        Err(LegacyError::ZeroDivision)
    );
    assert_eq!(
        nan.div(&zero, p(160), Rounding::Floor),
        Err(LegacyError::ZeroDivision)
    );
    assert_eq!(
        Mpf::one().div(&inf, p(160), Rounding::Ceiling).unwrap(),
        zero
    );
    assert_eq!(
        ninf.sqrt(p(160), Rounding::Floor),
        Err(LegacyError::ComplexResult)
    );
    assert!(nan.sqrt(p(160), Rounding::Ceiling).unwrap().is_nan());
    assert!(inf.pow_int(0, p(160), Rounding::Floor).unwrap().is_nan());
    assert_eq!(
        LegacyInterval::new(inf.clone(), inf)
            .unwrap()
            .pow_int(0, p(160))
            .unwrap(),
        LegacyInterval::one()
    );
}

#[test]
fn rounded_large_powers_and_direct_square_match_known_raw_results() {
    let man = IBig::from((UBig::ONE << 513usize) - UBig::ONE);
    let value = Mpf::from_man_exp(man, -200, None, Rounding::TowardZero).unwrap();
    assert_eq!(
        value
            .pow_int(17, p(8), Rounding::Floor)
            .unwrap()
            .raw_tuple(),
        raw(0, "255", 5313, 8)
    );
    assert_eq!(
        value
            .pow_int(17, p(8), Rounding::Ceiling)
            .unwrap()
            .raw_tuple(),
        raw(0, "1", 5321, 1)
    );
    assert_eq!(
        value
            .pow_int(17, p(53), Rounding::Nearest)
            .unwrap()
            .raw_tuple(),
        raw(0, "1", 5321, 1)
    );
    for round in [Rounding::Floor, Rounding::Ceiling, Rounding::Nearest] {
        // В источнике n=2 непосредственно возводит мантиссу в квадрат до единственного округления.
        assert_eq!(
            value.pow_int(2, p(180), round).unwrap(),
            value.square(p(180), round).unwrap()
        );
    }
}

#[test]
fn negative_scalar_and_interval_powers_preserve_the_different_work_precisions() {
    // Фактический контрпример libmpf/libmpi: MPF использует p+5, положительная степень MPI — p+20.
    let value = exact(7, -7);
    assert_eq!(
        value
            .pow_int(-3, p(2), Rounding::Floor)
            .unwrap()
            .raw_tuple(),
        raw(0, "1", 12, 1)
    );
    assert_eq!(
        value
            .pow_int(-3, p(2), Rounding::Ceiling)
            .unwrap()
            .raw_tuple(),
        raw(0, "1", 13, 1)
    );
    let interval = LegacyInterval::new(value.clone(), value)
        .unwrap()
        .pow_int(-3, p(2))
        .unwrap();
    assert_eq!(interval.lower().raw_tuple(), raw(0, "1", 12, 1));
    assert_eq!(interval.upper().raw_tuple(), raw(0, "3", 11, 2));
    assert_eq!(
        exact(33, -7)
            .pow_int(-17, p(8), Rounding::Floor)
            .unwrap()
            .raw_tuple(),
        raw(0, "151", 26, 8)
    );
    assert_eq!(
        exact(33, -7)
            .pow_int(-17, p(8), Rounding::Ceiling)
            .unwrap()
            .raw_tuple(),
        raw(0, "19", 29, 5)
    );
}

#[test]
fn interval_sign_cases_preserve_extended_bounds_and_raw_nan_outputs() {
    let cross = LegacyInterval::new(exact(-1, 0), exact(1, 0)).unwrap();
    assert_eq!(
        cross.square(p(160)).unwrap(),
        LegacyInterval::new(Mpf::zero(), Mpf::one()).unwrap()
    );
    assert_eq!(cross.div(&cross, p(160)).unwrap(), LegacyInterval::entire());
    assert_eq!(
        LegacyInterval::zero()
            .mul(&LegacyInterval::entire(), p(160))
            .unwrap(),
        LegacyInterval::entire()
    );
    assert_eq!(
        LegacyInterval::zero().div(&cross, p(160)).unwrap(),
        LegacyInterval::entire()
    );
    let nonpositive = LegacyInterval::new(exact(-2, 0), exact(-1, 0)).unwrap();
    assert_eq!(
        nonpositive.mul(&nonpositive, p(160)).unwrap(),
        LegacyInterval::new(exact(1, 0), exact(4, 0)).unwrap()
    );
    let inf = LegacyInterval::new(Mpf::positive_infinity(), Mpf::positive_infinity()).unwrap();
    let zero_inf = LegacyInterval::new(Mpf::zero(), Mpf::positive_infinity()).unwrap();
    let raw_nan_output = inf.div(&zero_inf, p(160)).unwrap();
    assert!(raw_nan_output.lower().is_nan());
    assert!(raw_nan_output.upper().is_positive_infinity());
    assert_eq!(
        raw_nan_output.pos(p(160)),
        Err(LegacyError::UnsupportedProfile(
            UnsupportedProfile::NaNIntervalEndpoint
        ))
    );
    // mpi_pow_int при n=1 возвращает исходный exact-вход, даже когда мантисса длиннее p.
    let unchanged = LegacyInterval::new(exact(33, 0), exact(35, 0)).unwrap();
    assert_eq!(unchanged.pow_int(1, p(2)).unwrap(), unchanged);
}

#[test]
fn sqrt_is_directed_on_tiny_values_and_known_exact_roots() {
    let tiny = exact(1, -512);
    assert_eq!(tiny.sqrt(p(160), Rounding::Floor).unwrap(), exact(1, -256));
    assert_eq!(
        tiny.sqrt(p(180), Rounding::Ceiling).unwrap(),
        exact(1, -256)
    );
    assert_eq!(
        exact(9, -4).sqrt(p(53), Rounding::Nearest).unwrap(),
        exact(3, -2)
    );
    let square_root = exact(2, 0).sqrt(p(8), Rounding::Floor).unwrap();
    assert_eq!(square_root.raw_tuple(), raw(0, "181", -7, 8));
    assert_eq!(
        exact(2, 0)
            .sqrt(p(8), Rounding::Ceiling)
            .unwrap()
            .raw_tuple(),
        raw(0, "91", -6, 7)
    );
}

#[test]
fn unsupported_profiles_are_internal_named_errors_not_oracle_admission() {
    assert!(matches!(
        Precision::new(0),
        Err(LegacyError::UnsupportedProfile(
            UnsupportedProfile::Precision { requested: 0, .. }
        ))
    ));
    assert!(matches!(
        Mpf::from_raw_tuple(0, UBig::from(2u8), 0, 2),
        Err(LegacyError::UnsupportedProfile(
            UnsupportedProfile::RawTuple { .. }
        ))
    ));
    assert!(matches!(
        Mpf::one().pow_int(i64::MAX, p(160), Rounding::Floor),
        Err(LegacyError::UnsupportedProfile(
            UnsupportedProfile::PowerExponent {
                requested: i64::MAX,
                ..
            }
        ))
    ));
    assert!(matches!(
        Mpf::from_man_exp(IBig::ONE, MAX_ABS_EXPONENT + 1, None, Rounding::Floor),
        Err(LegacyError::UnsupportedProfile(
            UnsupportedProfile::Exponent { .. }
        ))
    ));
    assert_eq!(
        LegacyInterval::new(Mpf::nan(), Mpf::one()),
        Err(LegacyError::UnsupportedProfile(
            UnsupportedProfile::NaNIntervalEndpoint
        ))
    );
}
