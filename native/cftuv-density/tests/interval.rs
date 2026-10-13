use cftuv_density::interval::{
    ceil_scaled_10_27, floor_scaled_10_27, IntervalError, RationalInterval,
};
use cftuv_density::{IBig, Rational, UBig};

fn ratio(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}

fn interval(lo: i64, hi: i64) -> RationalInterval {
    RationalInterval::new(ratio(lo, 1), ratio(hi, 1)).unwrap()
}

fn compare(left: &Rational, right: &Rational) -> std::cmp::Ordering {
    let (ln, ld) = left.parts();
    let (rn, rd) = right.parts();
    (ln * IBig::from(rd.clone())).cmp(&(rn * IBig::from(ld.clone())))
}

fn assert_sqrt_encloses_point(bound: &RationalInterval, value: &Rational) {
    let square = |x: &Rational| {
        let (n, d) = x.parts();
        Rational::new(n * n, d * d).unwrap()
    };
    assert!(compare(bound.lower(), &ratio(0, 1)).is_ge());
    assert!(compare(&square(bound.lower()), value).is_le());
    assert!(compare(value, &square(bound.upper())).is_le());
}

#[test]
fn closed_bounds_are_validated_and_intersection_keeps_shared_endpoints() {
    assert_eq!(
        RationalInterval::new(ratio(2, 1), ratio(1, 1)),
        Err(IntervalError::ReversedBounds)
    );
    assert_eq!(
        interval(-1, 2).intersect(&interval(2, 3)).unwrap(),
        RationalInterval::point(ratio(2, 1))
    );
    assert_eq!(
        interval(-1, 2).intersect(&interval(3, 4)),
        Err(IntervalError::EmptyIntersection)
    );
    assert!(interval(-1, 2).contains(&ratio(-1, 1)));
    assert!(interval(-1, 2).contains(&ratio(2, 1)));
}

#[test]
fn endpoint_arithmetic_is_exact_and_general_dependency_is_conservative() {
    assert_eq!(interval(-2, 3).add(&interval(4, 5)), interval(2, 8));
    assert_eq!(interval(-2, 3).sub(&interval(4, 5)), interval(-7, -1));
    assert_eq!(interval(-2, 3).mul(&interval(-4, 5)), interval(-12, 15));
    let x = interval(-1, 1);
    assert_eq!(x.mul(&x), interval(-1, 1));
    assert_eq!(x.sub(&x), interval(-2, 2));
    assert!(x.sub(&x).contains(&ratio(0, 1)));
}

#[test]
fn square_knows_zero_crossings_and_negative_endpoint_order() {
    for (value, expected) in [
        (interval(-3, -2), interval(4, 9)),
        (interval(-3, 2), interval(0, 9)),
        (interval(-1, 4), interval(0, 16)),
        (interval(2, 3), interval(4, 9)),
        (interval(0, 0), interval(0, 0)),
    ] {
        assert_eq!(value.square(), expected);
    }
}

#[test]
fn division_requires_zero_outside_the_closed_denominator_interval() {
    for denominator in [
        interval(-1, 1),
        interval(0, 1),
        interval(-1, 0),
        interval(0, 0),
    ] {
        assert_eq!(
            interval(-2, 3).div(&denominator),
            Err(IntervalError::DenominatorContainsZero)
        );
        assert_eq!(
            interval(0, 0).div(&denominator),
            Err(IntervalError::DenominatorContainsZero)
        );
    }
    assert_eq!(
        interval(-2, 3).div(&interval(2, 4)).unwrap(),
        RationalInterval::new(ratio(-1, 1), ratio(3, 2)).unwrap()
    );
    assert_eq!(
        interval(-2, 3).div(&interval(-4, -2)).unwrap(),
        RationalInterval::new(ratio(-3, 2), ratio(1, 1)).unwrap()
    );
}

#[test]
fn positive_sqrt_rounds_outward_and_exact_dyadic_roots_stay_points() {
    for value in [ratio(0, 1), ratio(2, 1), ratio(7, 16), ratio(17, 3)] {
        for bits in [0, 1, 8, 32] {
            let bound = RationalInterval::point(value.clone())
                .positive_sqrt(bits)
                .unwrap();
            assert_sqrt_encloses_point(&bound, &value);
        }
    }
    assert_eq!(interval(4, 4).positive_sqrt(32).unwrap(), interval(2, 2));
    assert_eq!(
        RationalInterval::new(ratio(1, 4), ratio(4, 1))
            .unwrap()
            .positive_sqrt(32)
            .unwrap(),
        RationalInterval::new(ratio(1, 2), ratio(2, 1)).unwrap()
    );
    assert_eq!(
        interval(-1, 0).positive_sqrt(32),
        Err(IntervalError::NegativeRadicand)
    );
    assert_eq!(
        interval(1, 1).positive_sqrt(u32::MAX),
        Err(IntervalError::BitCountOverflow)
    );
}

#[test]
fn near_zero_roots_have_safe_coarse_bounds_and_exact_fine_bounds() {
    let value = Rational::new(IBig::ONE, UBig::ONE << 512usize).unwrap();
    let coarse = RationalInterval::point(value.clone())
        .positive_sqrt(32)
        .unwrap();
    assert_eq!(coarse.lower(), &ratio(0, 1));
    assert_eq!(
        coarse.upper(),
        &Rational::new(IBig::ONE, UBig::ONE << 32usize).unwrap()
    );
    assert_sqrt_encloses_point(&coarse, &value);
    let exact = Rational::new(IBig::ONE, UBig::ONE << 256usize).unwrap();
    assert_eq!(
        RationalInterval::point(value).positive_sqrt(256).unwrap(),
        RationalInterval::point(exact)
    );
}

#[test]
fn integer_10_27_scaling_has_correct_negative_floor_and_exact_point_units() {
    let floor = IBig::from_str_radix("-333333333333333333333333334", 10).unwrap();
    let ceiling = IBig::from_str_radix("-333333333333333333333333333", 10).unwrap();
    assert_eq!(floor_scaled_10_27(&ratio(-1, 3)), floor);
    assert_eq!(ceil_scaled_10_27(&ratio(-1, 3)), ceiling);
    assert_eq!(
        RationalInterval::point(ratio(0, 1)).scale10_27_envelope(),
        (IBig::ZERO, IBig::ZERO)
    );
    for (value, units) in [
        (ratio(1657, 5120), "323632812500000000000000000"),
        (ratio(3647, 5120), "712304687500000000000000000"),
    ] {
        let expected = IBig::from_str_radix(units, 10).unwrap();
        assert_eq!(
            RationalInterval::point(value.clone()).scale10_27_envelope(),
            (expected.clone(), expected.clone())
        );
        let (n, d) = value.parts();
        let negative = Rational::new(-n, d.clone()).unwrap();
        assert_eq!(
            RationalInterval::point(negative).scale10_27_envelope(),
            (-&expected, -&expected)
        );
    }
}

#[test]
fn the_q5_nested_sine_can_be_enclosed_without_admitting_a_product_ray() {
    let one = interval(1, 1);
    let root5 = interval(5, 5).positive_sqrt(64).unwrap();
    let cosine = one.add(&root5).div(&interval(4, 4)).unwrap();
    let sine = one.sub(&cosine.square()).positive_sqrt(64).unwrap();
    assert!(compare(sine.lower(), &ratio(587, 1000)).is_gt());
    assert!(compare(sine.upper(), &ratio(589, 1000)).is_lt());
    assert!(compare(cosine.lower(), &ratio(808, 1000)).is_gt());
    assert!(compare(cosine.upper(), &ratio(810, 1000)).is_lt());
}
