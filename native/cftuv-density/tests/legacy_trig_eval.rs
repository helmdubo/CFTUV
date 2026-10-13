use cftuv_density::legacy_interval::LegacyInterval;
use cftuv_density::legacy_mpf::{Mpf, Precision, Rounding};
use cftuv_density::legacy_trig_eval::{mpf_cos_sin, mpi_cos_sin, EvalError, UnsupportedProfile};
use cftuv_density::IBig;

fn value(n: i64, exp: i64) -> Mpf {
    Mpf::from_man_exp(IBig::from(n), exp, None, Rounding::TowardZero).unwrap()
}

#[test]
fn legacy_sine_extra_ulp_is_preserved_at_the_sign_admission_counterexample() {
    let x = value(365, -13);
    let point = LegacyInterval::new(x.clone(), x).unwrap();
    let (_, sine) = mpi_cos_sin(&point, Precision::new(160).unwrap()).unwrap();
    let a: IBig = "1041546121700086938441153703724603043422243247291"
        .parse()
        .unwrap();
    let low = Mpf::from_man_exp(a.clone(), -164, None, Rounding::TowardZero).unwrap();
    let high = Mpf::from_man_exp(a + IBig::from(2), -164, None, Rounding::TowardZero).unwrap();
    assert_eq!(sine.lower(), &low);
    assert_eq!(sine.upper(), &high);
}

#[test]
fn zero_full_period_and_tiny_scalar_keep_distinct_source_paths() {
    let p = Precision::new(160).unwrap();
    let (cos, sin) = mpi_cos_sin(&LegacyInterval::zero(), p).unwrap();
    assert_eq!(cos, LegacyInterval::one());
    assert_eq!(sin, LegacyInterval::zero());
    let whole = LegacyInterval::new(value(-8, 0), value(8, 0)).unwrap();
    let (cos, sin) = mpi_cos_sin(&whole, p).unwrap();
    let full = LegacyInterval::new(value(-1, 0), value(1, 0)).unwrap();
    assert_eq!(cos, full);
    assert_eq!(sin, full);
    let tiny = value(1, -1000);
    assert!(mpf_cos_sin(&tiny, p, Rounding::Nearest).is_ok());
    // Дополнительный modulo MPI требует больше 400 бит; это внутренний profile guard.
    assert!(matches!(
        mpi_cos_sin(&LegacyInterval::new(tiny.clone(), tiny).unwrap(), p),
        Err(EvalError::UnsupportedProfile(
            UnsupportedProfile::ReductionWorkBits { .. }
        ))
    ));
}

#[test]
fn finite_profile_guards_are_not_density_product_refusals() {
    let p = Precision::new(160).unwrap();
    assert!(matches!(
        mpf_cos_sin(&value(9, 0), p, Rounding::Nearest),
        Err(EvalError::UnsupportedProfile(
            UnsupportedProfile::Argument { .. }
        ))
    ));
    assert!(matches!(
        mpf_cos_sin(
            &value(1, 0),
            Precision::new(513).unwrap(),
            Rounding::Nearest
        ),
        Err(EvalError::UnsupportedProfile(
            UnsupportedProfile::Precision { .. }
        ))
    ));
    assert!(matches!(
        mpf_cos_sin(
            &value(1, 0),
            Precision::new(400).unwrap(),
            Rounding::Nearest
        ),
        Err(EvalError::UnsupportedProfile(
            UnsupportedProfile::ReductionWorkBits { .. }
        ))
    ));
}
