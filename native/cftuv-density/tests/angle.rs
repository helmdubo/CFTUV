use cftuv_density::angle::{AngleAtom, AngleError, RankEnclosure, RayEnclosureError};
use cftuv_density::{IBig, Rational, UBig};

fn ratio(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}

fn compare(left: &Rational, right: &Rational) -> std::cmp::Ordering {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    (ln * IBig::from(rd.clone())).cmp(&(rn * IBig::from(ld.clone())))
}

fn assert_rank_and_width(atom: &AngleAtom, enclosure: &RankEnclosure, bits: u32) {
    assert!(compare(&enclosure.lower, &enclosure.upper).is_le());
    assert!(compare(&ratio(-1, 1), &enclosure.lower).is_le());
    assert!(compare(&enclosure.upper, &ratio(1, 1)).is_le());
    let lower_above = atom.roots_strictly_above(&enclosure.lower);
    let upper_above = atom.roots_strictly_above(&enclosure.upper);
    assert!(
        lower_above >= atom.rank()
            || (atom.is_root(&enclosure.lower) && lower_above == atom.rank() - 1)
    );
    assert!(upper_above < atom.rank());
    let (ln, ld) = enclosure.lower.parts();
    let (un, ud) = enclosure.upper.parts();
    let width_n = un * IBig::from(ld.clone()) - ln * IBig::from(ud.clone());
    assert!(width_n >= IBig::ZERO);
    assert!(width_n * IBig::from(UBig::ONE << bits as usize) <= IBig::from(ld * ud));
    assert!(enclosure.stats.bisections <= bits + 1);
    assert_eq!(enclosure.stats.polynomial_degree, atom.total_roots());
}

fn assert_sturm_stats(enclosure: &RankEnclosure) {
    assert_eq!(
        enclosure.stats.variation_evaluations,
        u64::from(enclosure.stats.bisections)
    );
    assert!(enclosure.stats.sturm_polynomials >= 2);
}

fn assert_fast_stats(enclosure: &RankEnclosure) {
    assert_eq!(enclosure.stats.variation_evaluations, 0);
    assert_eq!(enclosure.stats.sturm_polynomials, 0);
}

#[test]
fn angle_domain_rejects_one_and_preserves_the_ratio_sign_contract() {
    for n in [0, 7, 255] {
        assert_eq!(
            AngleAtom::new(n, ratio(1, 2), 1).unwrap_err(),
            AngleError::UnsupportedSubdivision(n)
        );
    }
    for r in [ratio(-1, 3), ratio(1, 1), ratio(2, 1)] {
        assert_eq!(
            AngleAtom::new(3, r, 1).unwrap_err(),
            AngleError::RatioOutsideHalfOpenUnitInterval
        );
    }
    for dot_sign in [-2, -1, 1, 2] {
        assert_eq!(
            AngleAtom::new(3, ratio(0, 1), dot_sign).unwrap_err(),
            AngleError::InconsistentDotSign(dot_sign)
        );
    }
    for dot_sign in [-2, 0, 2] {
        assert_eq!(
            AngleAtom::new(3, ratio(1, 3), dot_sign).unwrap_err(),
            AngleError::InconsistentDotSign(dot_sign)
        );
    }
    assert_eq!(
        AngleAtom::new(3, ratio(1, 2), 1)
            .unwrap()
            .enclose(u32::MAX)
            .unwrap_err(),
        AngleError::BitCountOverflow
    );
}

#[test]
fn zero_ratio_uses_t_n_and_has_exactly_n_roots_between_nonroot_endpoints() {
    let coefficients: [&[i64]; 6] = [
        &[0, 1],
        &[-1, 0, 2],
        &[0, -3, 0, 4],
        &[1, 0, -8, 0, 8],
        &[0, 5, 0, -20, 0, 16],
        &[-1, 0, 18, 0, -48, 0, 32],
    ];
    for n in 1..=6 {
        let atom = AngleAtom::new(n, ratio(0, 1), 0).unwrap();
        let expected: Vec<_> = coefficients[usize::from(n - 1)]
            .iter()
            .map(|&coefficient| ratio(coefficient, 1))
            .collect();
        assert_eq!(atom.polynomial_coefficients(), expected.as_slice());
        assert_eq!(atom.rank(), 1);
        assert_eq!(atom.total_roots(), usize::from(n));
        assert_eq!(atom.roots_strictly_above(&ratio(-1, 1)), usize::from(n));
        assert_eq!(atom.roots_strictly_above(&ratio(1, 1)), 0);
        assert!(!atom.is_root(&ratio(-1, 1)));
        assert!(!atom.is_root(&ratio(1, 1)));
        assert_rank_and_width(&atom, &atom.enclose(32).unwrap(), 32);
    }
}

#[test]
fn n1_returns_the_signed_root_and_zero_without_spurious_sign() {
    for dot_sign in [-1, 1] {
        let atom = AngleAtom::new(1, ratio(9, 16), dot_sign).unwrap();
        let bound = atom.enclose(32).unwrap();
        assert_eq!(bound.lower, ratio(i64::from(dot_sign) * 3, 4));
        assert_eq!(bound.upper, bound.lower);
        assert_rank_and_width(&atom, &bound, 32);
    }
    let zero = AngleAtom::new(1, ratio(0, 1), 0).unwrap();
    let bound = zero.enclose(0).unwrap();
    assert_eq!(bound.lower, ratio(0, 1));
    assert_eq!(bound.upper, ratio(0, 1));
    assert_eq!(bound.stats.bisections, 1);
    assert_rank_and_width(&zero, &bound, 0);
}

#[test]
fn an_exact_midpoint_of_the_other_rank_cannot_be_returned_as_the_target() {
    // T3(3/4)=-9/16; при r=81/256 это rank2, НЕ rank1.
    let midpoint = ratio(3, 4);
    let positive = AngleAtom::new(3, ratio(81, 256), 1).unwrap();
    let negative = AngleAtom::new(3, ratio(81, 256), -1).unwrap();
    assert!(positive.is_root(&midpoint));
    assert_eq!(positive.roots_strictly_above(&midpoint), 1);
    assert_eq!(negative.roots_strictly_above(&midpoint), 1);
    let rank1 = positive.enclose(32).unwrap();
    let rank2 = negative.enclose(32).unwrap();
    assert_ne!(rank1.lower, midpoint);
    assert!(compare(&rank1.lower, &midpoint).is_gt());
    assert_eq!(rank2.lower, midpoint);
    assert_eq!(rank2.upper, midpoint);
    assert_eq!(rank2.stats.bisections, 3);
    assert_rank_and_width(&positive, &rank1, 32);
    assert_rank_and_width(&negative, &rank2, 32);
}

#[test]
fn close_rank1_and_rank2_roots_may_have_the_same_32_bit_enclosure() {
    let near_zero = Rational::new(IBig::ONE, UBig::ONE << 512usize).unwrap();
    for n in [3, 5, 6] {
        let rank1 = AngleAtom::new(n, near_zero.clone(), 1).unwrap();
        let rank2 = AngleAtom::new(n, near_zero.clone(), -1).unwrap();
        let first = rank1.enclose(32).unwrap();
        let second = rank2.enclose(32).unwrap();
        assert_eq!(first.lower, second.lower, "n={n}");
        assert_eq!(first.upper, second.upper, "n={n}");
        assert_ne!(first.lower, first.upper, "rank enclosure need not isolate");
        assert!(rank1.roots_strictly_above(&first.lower) >= 2);
        assert_eq!(rank1.roots_strictly_above(&first.upper), 0);
        assert!(compare(&first.lower, &ratio(0, 1)).is_gt());
        assert_rank_and_width(&rank1, &first, 32);
        assert_rank_and_width(&rank2, &second, 32);
    }
}

#[test]
fn ratios_approaching_one_do_not_turn_into_endpoint_roots() {
    let denominator = UBig::ONE << 256usize;
    let near_one = Rational::new(IBig::from(&denominator - UBig::ONE), denominator).unwrap();
    for n in [1, 3, 6] {
        for dot_sign in [-1, 1] {
            let atom = AngleAtom::new(n, near_one.clone(), dot_sign).unwrap();
            assert!(!atom.is_root(&ratio(-1, 1)));
            assert!(!atom.is_root(&ratio(1, 1)));
            assert_eq!(atom.total_roots(), usize::from(n) * 2);
            let bound = atom.enclose(32).unwrap();
            assert_ne!(bound.lower, bound.upper);
            if dot_sign > 0 {
                assert_eq!(bound.upper, ratio(1, 1));
                assert!(compare(&bound.lower, &ratio(0, 1)).is_gt());
            } else if n == 1 {
                assert_eq!(bound.lower, ratio(-1, 1));
                assert!(compare(&bound.upper, &ratio(0, 1)).is_lt());
            } else {
                assert!(compare(&bound.lower, &ratio(0, 1)).is_gt());
            }
            assert_rank_and_width(&atom, &bound, 32);
        }
    }
}

#[test]
fn sturm_counts_certify_every_returned_rank_and_b_plus_one_width() {
    for n in 1..=6 {
        for numerator in 0..13 {
            for dot_sign in if numerator == 0 { vec![0] } else { vec![-1, 1] } {
                let atom = AngleAtom::new(n, ratio(numerator, 13), dot_sign).unwrap();
                assert_eq!(atom.roots_strictly_above(&ratio(-1, 1)), atom.total_roots());
                assert_eq!(atom.roots_strictly_above(&ratio(1, 1)), 0);
                assert!(!atom.is_root(&ratio(-1, 1)));
                assert!(!atom.is_root(&ratio(1, 1)));
                for bits in [0, 1, 7, 32] {
                    let bound = atom.enclose(bits).unwrap();
                    assert_rank_and_width(&atom, &bound, bits);
                    assert_sturm_stats(&bound);
                    if bound.lower == bound.upper {
                        assert!(atom.is_root(&bound.lower));
                        assert_eq!(atom.roots_strictly_above(&bound.lower), atom.rank() - 1);
                    } else {
                        assert_eq!(bound.stats.bisections, bits + 1);
                    }
                }
            }
        }
    }
}

fn assert_positive_sqrt_contains(
    bound: &cftuv_density::interval::RationalInterval,
    squared: &Rational,
) {
    let square = |x: &Rational| {
        let (n, d) = x.parts();
        Rational::new(n * n, d * d).unwrap()
    };
    assert!(compare(bound.lower(), &ratio(0, 1)).is_ge());
    assert!(compare(&square(bound.lower()), squared).is_le());
    assert!(compare(squared, &square(bound.upper())).is_le());
}

#[test]
fn rays_cover_selected_exact_algebraic_angles_and_a_quarter_turn() {
    let atom = AngleAtom::new(3, ratio(81, 256), -1).unwrap();
    let ray = atom.ray_enclosure(1, 32).unwrap();
    assert_fast_stats(&ray.root);
    assert_eq!(ray.cosine.lower(), &ratio(3, 4));
    assert_eq!(ray.cosine.upper(), &ratio(3, 4));
    assert_positive_sqrt_contains(&ray.sine, &ratio(7, 16));
    let ray = atom.ray_enclosure(2, 32).unwrap();
    assert_eq!(ray.cosine.lower(), &ratio(1, 8));
    assert_eq!(ray.cosine.upper(), &ratio(1, 8));
    assert_positive_sqrt_contains(&ray.sine, &ratio(63, 64));
    let zero_dot = AngleAtom::new(3, ratio(0, 1), 0).unwrap();
    let ray = zero_dot.ray_enclosure(2, 32).unwrap();
    assert!(ray.cosine.contains(&ratio(1, 2)));
    assert_positive_sqrt_contains(&ray.sine, &ratio(3, 4));
    // acos(-sqrt(1/2))/3=pi/4, j=2 даёт pi/2.
    let quarter = AngleAtom::new(3, ratio(1, 2), -1)
        .unwrap()
        .ray_enclosure(2, 32)
        .unwrap();
    assert!(quarter.cosine.contains(&ratio(0, 1)));
    assert!(quarter.sine.contains(&ratio(1, 1)));
    assert_eq!(quarter.sine.upper(), &ratio(1, 1));
}

#[test]
fn coarse_ray_dependency_is_conservative_and_invalid_ordinals_are_explicit() {
    let atom = AngleAtom::new(6, ratio(1, 3), -1).unwrap();
    assert_eq!(
        atom.ray_enclosure(1, u32::MAX).unwrap_err(),
        RayEnclosureError::InternalInterval(
            cftuv_density::interval::IntervalError::BitCountOverflow
        )
    );
    for bits in [0, 1, 32] {
        for ordinal in 1..6 {
            let ray = atom.ray_enclosure(ordinal, bits).unwrap();
            assert_fast_stats(&ray.root);
            assert!(compare(ray.cosine.lower(), &ratio(-1, 1)).is_ge());
            assert!(compare(ray.cosine.upper(), &ratio(1, 1)).is_le());
            assert!(compare(ray.sine.lower(), &ratio(0, 1)).is_ge());
            assert!(compare(ray.sine.upper(), &ratio(1, 1)).is_le());
            assert_rank_and_width(&atom, &ray.root, bits);
        }
    }
    for ordinal in [0, 6, 7] {
        assert_eq!(
            atom.ray_enclosure(ordinal, 32).unwrap_err(),
            RayEnclosureError::InvalidOrdinal {
                ordinal,
                subdivision: 6
            }
        );
    }
}
