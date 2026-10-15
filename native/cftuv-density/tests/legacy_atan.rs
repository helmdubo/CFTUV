use cftuv_density::legacy_atan::{
    atan_cached, mpf_atan, mpf_atan2, mpi_atan2, mpi_atan2_raw, AtanError,
};
use cftuv_density::legacy_interval::LegacyInterval;
use cftuv_density::legacy_mpf::{LegacyError, Mpf, Precision, Rounding, UnsupportedProfile};
use cftuv_density::legacy_trig::mpf_pi;
use cftuv_density::IBig;

fn value(n: i64, exp: i64) -> Mpf {
    Mpf::from_man_exp(IBig::from(n), exp, None, Rounding::TowardZero).unwrap()
}

#[test]
fn source_reversed_atan2_result_is_preserved_without_relaxing_interval_invariant() {
    let y = LegacyInterval::new(Mpf::negative_infinity(), Mpf::zero()).unwrap();
    let x = LegacyInterval::new(value(-1, 0), value(1, 0)).unwrap();
    let p = Precision::new(160).unwrap();
    let (a, b) = mpi_atan2_raw(&y, &x, p).unwrap();
    assert_eq!(a, mpf_pi(p, Rounding::Floor).unwrap());
    assert_eq!(b, Mpf::zero());
    assert!(matches!(
        mpi_atan2(&y, &x, p),
        Err(AtanError::Legacy(LegacyError::UnsupportedProfile(
            UnsupportedProfile::ReversedInterval
        )))
    ));
}

#[test]
fn scalar_source_quadrants_and_nonfinite_paths_are_explicit() {
    let p = Precision::new(160).unwrap();
    assert_eq!(
        mpf_atan2(&Mpf::zero(), &value(-1, 0), p, Rounding::Ceiling).unwrap(),
        mpf_pi(p, Rounding::Ceiling).unwrap()
    );
    assert_eq!(
        mpf_atan2(&value(-1, 0), &Mpf::positive_infinity(), p, Rounding::Floor).unwrap(),
        Mpf::zero()
    );
    assert!(mpf_atan2(
        &Mpf::positive_infinity(),
        &Mpf::negative_infinity(),
        p,
        Rounding::Floor
    )
    .unwrap()
    .is_nan());
    for r in [
        Rounding::Floor,
        Rounding::Ceiling,
        Rounding::TowardZero,
        Rounding::AwayFromZero,
        Rounding::Nearest,
    ] {
        assert!(mpf_atan(&value(1, -1000), p, r).is_ok());
        assert!(mpf_atan(&value(-1, 1000), p, r).is_ok());
    }
}

#[test]
fn immutable_cached_floor_prefixes_and_profile_limits_are_distinct() {
    for n in 0..256 {
        let (_, full) = atan_cached(n, 512).unwrap();
        for bits in [8, 16, 32, 64, 128, 256] {
            let (a, b) = atan_cached(n, bits).unwrap();
            assert_eq!(a, IBig::from(n) << ((bits - 7) as usize));
            assert_eq!(b, &full >> ((512 - bits) as usize));
        }
    }
    assert!(matches!(
        atan_cached(256, 160),
        Err(AtanError::AnchorIndex(_))
    ));
    assert!(matches!(
        mpf_atan(&value(1, 0), Precision::new(500).unwrap(), Rounding::Floor),
        Err(AtanError::WorkPrecision(_))
    ));
}
