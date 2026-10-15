use std::cmp::Ordering;

use cftuv_density::angle::{AngleAtom, AngleError};
use cftuv_density::angle_monotone::{compare_to_principal, enclose_principal};
use cftuv_density::{IBig, Rational, UBig};

fn ratio(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}

fn assert_same_as_sturm(n: u8, r: &Rational, sign: i8, bits: u32) {
    let old = AngleAtom::new(n, r.clone(), sign)
        .unwrap()
        .enclose(bits)
        .unwrap();
    let fast = enclose_principal(n, r, sign, bits).unwrap();
    assert_eq!(
        fast.lower, old.lower,
        "n={n} r={r:?} sign={sign} bits={bits}"
    );
    assert_eq!(
        fast.upper, old.upper,
        "n={n} r={r:?} sign={sign} bits={bits}"
    );
    assert_eq!(fast.stats.bisections, old.stats.bisections);
    assert_eq!(fast.stats.polynomial_degree, old.stats.polynomial_degree);
    assert_eq!(fast.stats.variation_evaluations, 0);
    assert_eq!(fast.stats.sturm_polynomials, 0);
}

#[test]
fn exact_midpoints_stop_only_on_the_selected_principal_branch() {
    let wrong_n2 = ratio(3, 4);
    assert_eq!(
        compare_to_principal(2, &ratio(1, 64), 1, &wrong_n2).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        compare_to_principal(2, &ratio(1, 64), -1, &wrong_n2).unwrap(),
        Ordering::Greater
    );
    let midpoint_n3 = ratio(3, 4);
    assert_eq!(
        compare_to_principal(3, &ratio(81, 256), -1, &midpoint_n3).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        compare_to_principal(3, &ratio(81, 256), 1, &midpoint_n3).unwrap(),
        Ordering::Less
    );
    let selected = enclose_principal(3, &ratio(81, 256), -1, 32).unwrap();
    assert_eq!(selected.lower, midpoint_n3);
    assert_eq!(selected.upper, selected.lower);
    assert_eq!(selected.stats.bisections, 3);
    // T4(1/2)=-1/2 — тот же знак и polynomial root, но вне principal branch.
    assert_eq!(
        compare_to_principal(4, &ratio(1, 4), -1, &ratio(1, 2)).unwrap(),
        Ordering::Less
    );
    let selected = enclose_principal(4, &ratio(1, 4), -1, 32).unwrap();
    assert_ne!(selected.lower, ratio(1, 2));
    assert_ne!(selected.upper, ratio(1, 2));
    assert_ne!(selected.lower, selected.upper);
}

#[test]
fn approved_edge_families_have_identical_endpoints_and_bisection_counts() {
    let denominator = UBig::ONE << 400usize;
    let near_zero = Rational::new(IBig::ONE, denominator.clone()).unwrap();
    let near_one = Rational::new(IBig::from(&denominator - UBig::ONE), denominator).unwrap();
    let mut cases = Vec::new();
    for n in 1..=6 {
        cases.push((n, ratio(0, 1), 0));
        for sign in [-1, 1] {
            cases.push((n, near_zero.clone(), sign));
            cases.push((n, near_one.clone(), sign));
        }
    }
    for sign in [-1, 1] {
        cases.push((1, ratio(9, 16), sign));
        cases.push((2, ratio(1, 64), sign));
        cases.push((3, ratio(81, 256), sign));
    }
    cases.push((4, ratio(1, 4), -1));
    for (n, r, sign) in cases {
        for bits in [0, 1, 2, 7, 32] {
            assert_same_as_sturm(n, &r, sign, bits);
        }
    }
}

#[test]
fn all_small_rational_input_ratios_agree_with_the_actual_sturm_engine() {
    for n in 1..=6 {
        for numerator in 1..11 {
            let r = ratio(numerator, 11);
            for sign in [-1, 1] {
                for bits in [2, 7, 32] {
                    assert_same_as_sturm(n, &r, sign, bits);
                }
            }
        }
    }
}

#[test]
fn profile_validation_matches_angle_atom_and_overflow_is_explicit() {
    for (n, r, sign, expected) in [
        (0, ratio(0, 1), 0, AngleError::UnsupportedSubdivision(0)),
        (7, ratio(0, 1), 0, AngleError::UnsupportedSubdivision(7)),
        (
            3,
            ratio(-1, 2),
            1,
            AngleError::RatioOutsideHalfOpenUnitInterval,
        ),
        (
            3,
            ratio(1, 1),
            1,
            AngleError::RatioOutsideHalfOpenUnitInterval,
        ),
        (3, ratio(0, 1), 1, AngleError::InconsistentDotSign(1)),
        (3, ratio(1, 2), 0, AngleError::InconsistentDotSign(0)),
    ] {
        assert_eq!(enclose_principal(n, &r, sign, 32), Err(expected.clone()));
        assert_eq!(
            compare_to_principal(n, &r, sign, &ratio(0, 1)),
            Err(expected)
        );
    }
    assert_eq!(
        enclose_principal(3, &ratio(1, 2), 1, u32::MAX),
        Err(AngleError::BitCountOverflow)
    );
}
