use cftuv_density::angle::AngleAtom;
use cftuv_density::angle_profile::{profile_bound_is_valid, ray, root_bits, FixedAngleError};
use cftuv_density::{IBig, Rational, UBig};

#[test]
fn fixed_subdivision_table_satisfies_the_integer_width_proof() {
    for n in 2..=6 {
        assert!(profile_bound_is_valid(n));
    }
    for n in [0, 1, 7, 255] {
        assert_eq!(root_bits(n), Err(FixedAngleError::NoHiddenOrdinal));
        assert!(!profile_bound_is_valid(n));
    }
}

#[test]
fn near_endpoint_and_near_zero_rays_keep_the_fixed_component_width() {
    let denominator = UBig::ONE << 400usize;
    let values = [
        (Rational::integer(IBig::ZERO), 0),
        (Rational::new(IBig::ONE, denominator.clone()).unwrap(), 1),
        (Rational::new(IBig::ONE, denominator.clone()).unwrap(), -1),
        (Rational::new(IBig::from(&denominator - UBig::ONE), denominator.clone()).unwrap(), 1),
        (Rational::new(IBig::from(&denominator - UBig::ONE), denominator).unwrap(), -1),
        (Rational::new(IBig::from(81), UBig::from(256u32)).unwrap(), -1),
    ];
    for n in 2..=6 {
        for (ratio, sign) in &values {
            let atom = AngleAtom::new(n, ratio.clone(), *sign).unwrap();
            for ordinal in 1..n {
                let result = ray(&atom, ordinal).unwrap();
                assert!(result.root.stats.bisections <= root_bits(n).unwrap() + 1);
            }
        }
    }
    // Точный cos(phi)=3/4 из regression старого UNDECIDABLE: новый proof не снимает отказ.
    let atom = AngleAtom::new(3, Rational::new(IBig::from(81), UBig::from(256u32)).unwrap(), -1).unwrap();
    let value = Rational::new(IBig::from(3), UBig::from(4u32)).unwrap();
    assert!(ray(&atom, 1).unwrap().cosine.contains(&value));
}
