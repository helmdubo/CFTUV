use cftuv_density::{
    DensityFailure, DensityFailureKind, DensityTransaction, IBig, InvalidInput, Metric2, Radical,
    Rational, UBig, Vector2, Work,
};
use cftuv_radical::{Failure, RefusalCode};

fn rational(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}

fn scalar(n: i64, d: u64) -> Radical {
    rational(n, d).radical()
}

fn root(n: i64, d: u64) -> Radical {
    Radical::sqrt_of_rational(IBig::from(n), UBig::from(d))
        .unwrap()
        .0
}

fn vector(x: i64, y: i64) -> Vector2 {
    Vector2::new(scalar(x, 1), scalar(y, 1))
}

fn metric(a: i64, b: i64, c: i64, owner: i8) -> Metric2 {
    Metric2::from_gram(
        [
            [rational(a, 1), rational(b, 1)],
            [rational(b, 1), rational(c, 1)],
        ],
        owner,
    )
    .unwrap()
    .0
}

fn identity() -> DensityTransaction {
    DensityTransaction::new(metric(1, 0, 1, 1))
}

fn scale(vector: &Vector2, n: i64, d: u64) -> Vector2 {
    Vector2::new(
        vector.x.scaled(&IBig::from(n), &UBig::from(d)),
        vector.y.scaled(&IBig::from(n), &UBig::from(d)),
    )
}

#[test]
fn rational_gram_is_validated_and_its_inverse_is_derived_exactly() {
    assert_eq!(rational(8, 12), rational(2, 3));
    let (g, work) = Metric2::from_gram(
        [
            [rational(2, 1), rational(1, 1)],
            [rational(1, 1), rational(3, 1)],
        ],
        -1,
    )
    .unwrap();
    assert_eq!(work, Work::NONE);
    assert_eq!(g.determinant(), &rational(5, 1));
    assert_eq!(
        g.inverse_gram(),
        &[
            [rational(3, 5), rational(-1, 5)],
            [rational(-1, 5), rational(2, 5)]
        ]
    );
    assert_eq!(g.owner_orientation_sign(), -1);
    for (gram, owner, expected) in [
        (
            [
                [rational(1, 1), rational(0, 1)],
                [rational(1, 1), rational(1, 1)],
            ],
            1,
            InvalidInput::AsymmetricGram,
        ),
        (
            [
                [rational(1, 1), rational(1, 1)],
                [rational(1, 1), rational(1, 1)],
            ],
            1,
            InvalidInput::NonPositiveDefiniteGram,
        ),
        (
            [
                [rational(-1, 1), rational(0, 1)],
                [rational(0, 1), rational(-1, 1)],
            ],
            1,
            InvalidInput::NonPositiveDefiniteGram,
        ),
        (
            [
                [rational(1, 1), rational(0, 1)],
                [rational(0, 1), rational(1, 1)],
            ],
            0,
            InvalidInput::InvalidOwnerOrientation(0),
        ),
    ] {
        assert_eq!(
            Metric2::from_gram(gram, owner).unwrap_err().kind,
            DensityFailureKind::InvalidInput(expected)
        );
    }
    assert_eq!(
        Rational::new(IBig::ONE, UBig::ZERO).unwrap_err().kind,
        DensityFailureKind::InvalidInput(InvalidInput::ZeroDenominator)
    );
}

#[test]
fn gram_dual_dot_and_owner_cross_have_known_values_on_a_skew_chart() {
    let mut transaction = DensityTransaction::new(metric(2, 1, 3, -1));
    let left = vector(1, 2);
    let right = vector(3, -1);
    assert_eq!(transaction.gram_dot(&left, &right).0, scalar(5, 1));
    assert_eq!(transaction.dual_dot(&left, &right).0, scalar(0, 1));
    assert_eq!(transaction.oriented_cross(&left, &right).0, scalar(7, 1));
    assert_eq!(
        transaction.gram_dot(&right, &left),
        (scalar(5, 1), Work::NONE)
    );
    assert_eq!(
        transaction.dual_dot(&right, &left),
        (scalar(0, 1), Work::NONE)
    );
    assert_eq!(
        transaction.oriented_cross(&right, &left),
        (scalar(-7, 1), Work::NONE)
    );
}

#[test]
fn quarter_turn_is_dual_orthogonal_norm_preserving_and_has_the_requested_orientation() {
    let left = Vector2::new(root(2, 1).add(&scalar(1, 3)).0, root(3, 2));
    for owner in [-1, 1] {
        for orientation in [-1, 1] {
            let mut transaction = DensityTransaction::new(metric(2, 1, 3, owner));
            let (turned, cold_work) = transaction.quarter_turn(&left, orientation).unwrap();
            assert!(cold_work.class_merge_checks > 0);
            assert!(transaction.dual_dot(&left, &turned).0.is_zero());
            assert_eq!(
                transaction.dual_dot(&turned, &turned).0,
                transaction.dual_dot(&left, &left).0
            );
            let cross = transaction.oriented_cross(&left, &turned).0;
            assert_eq!(transaction.sign(&cross).unwrap().0, orientation);
            assert_eq!(
                transaction.quarter_turn(&turned, orientation).unwrap().0,
                scale(&left, -1, 1)
            );
            assert_eq!(
                transaction.quarter_turn(&left, -orientation).unwrap().0,
                scale(&turned, -1, 1)
            );
            assert_eq!(
                transaction.quarter_turn(&left, orientation).unwrap(),
                (turned, Work::NONE)
            );
        }
    }
    let mut transaction = DensityTransaction::new(metric(2, 1, 3, 1));
    let turned = transaction.quarter_turn(&vector(1, 2), 1).unwrap().0;
    assert_eq!(
        turned,
        Vector2::new(
            root(5, 1).scaled(&IBig::from(-3), &UBig::from(5u8)),
            root(5, 1).scaled(&IBig::ONE, &UBig::from(5u8))
        )
    );
    assert_eq!(
        transaction.quarter_turn(&vector(1, 2), 0).unwrap_err().kind,
        DensityFailureKind::InvalidInput(InvalidInput::InvalidTurnOrientation(0))
    );
}

#[test]
fn the_representable_subturn_boundaries_are_closed_and_positive_dot_is_required() {
    let mut transaction = identity();
    let left = vector(1, 0);
    let boundaries = [
        (2, vector(0, 1)),
        (
            3,
            Vector2::new(
                scalar(1, 2),
                root(3, 1).scaled(&IBig::ONE, &UBig::from(2u8)),
            ),
        ),
        (4, Vector2::new(root(1, 2), root(1, 2))),
        (
            6,
            Vector2::new(
                root(3, 1).scaled(&IBig::ONE, &UBig::from(2u8)),
                scalar(1, 2),
            ),
        ),
    ];
    for (q, boundary) in boundaries {
        assert!(transaction.subturn(&left, &boundary, q).unwrap().0, "q={q}");
        assert!(
            transaction.subturn_boundary(&left, &boundary, q).unwrap().0,
            "q={q}"
        );
        assert!(transaction.subturn(&boundary, &left, q).unwrap().0);
        assert_eq!(
            transaction
                .subturn(&left, &scale(&boundary, -1, 1), q)
                .unwrap()
                .0,
            q == 2
        );
        assert_eq!(
            transaction
                .subturn_boundary(&left, &scale(&boundary, -1, 1), q)
                .unwrap()
                .0,
            q == 2
        );
    }
    for q in 2..=6 {
        assert!(transaction.subturn(&left, &left, q).unwrap().0);
        assert!(!transaction.subturn_boundary(&left, &left, q).unwrap().0);
        assert!(!transaction.subturn(&left, &vector(-1, 0), q).unwrap().0);
        assert!(
            !transaction
                .subturn_boundary(&left, &vector(-1, 0), q)
                .unwrap()
                .0
        );
    }
}

#[test]
fn q5_uses_its_cosine_squared_without_constructing_the_nested_sine_root() {
    let mut transaction = identity();
    let left = vector(1, 0);
    // Exact residuals: 8*18²-(3+sqrt5)*(18²+13²)>0;
    // 8*11²-(3+sqrt5)*(11²+8²)<0. Это строгие стороны 36°.
    assert!(transaction.subturn(&left, &vector(18, 13), 5).unwrap().0);
    assert!(!transaction.subturn(&left, &vector(11, 8), 5).unwrap().0);
    assert!(
        transaction
            .subturn(&left, &Vector2::new(root(3, 1), scalar(1, 1)), 5)
            .unwrap()
            .0
    );
    assert!(!transaction.subturn(&left, &vector(1, 1), 5).unwrap().0);
    assert!(
        !transaction
            .subturn_boundary(&left, &vector(18, 13), 5)
            .unwrap()
            .0
    );
}

#[test]
fn zero_vectors_keep_the_oracle_predicate_order_without_new_refusals() {
    let left = vector(0, 0);
    let mut transaction = identity();
    for right in [vector(0, 0), vector(1, 2)] {
        for q in 2..=6 {
            assert_eq!(transaction.subturn(&left, &right, q).unwrap().0, q == 2);
            assert!(transaction.subturn_boundary(&left, &right, q).unwrap().0);
        }
    }
    assert_eq!(transaction.quarter_turn(&left, 1).unwrap().0, left);
    for q in [0, 1, 7, 12, 255] {
        assert_eq!(
            transaction
                .subturn(&left, &vector(1, 0), q)
                .unwrap_err()
                .kind,
            DensityFailureKind::InvalidInput(InvalidInput::UnsupportedDensityQ(q))
        );
        assert_eq!(
            transaction
                .subturn_boundary(&left, &vector(1, 0), q)
                .unwrap_err()
                .kind,
            DensityFailureKind::InvalidInput(InvalidInput::UnsupportedDensityQ(q))
        );
    }
}

#[test]
fn rational_chart_changes_and_positive_vector_scaling_preserve_dual_predicates() {
    let left = Vector2::new(root(2, 1), scalar(-1, 3));
    let right = Vector2::new(root(3, 1), scalar(2, 5));
    let mut euclidean = identity();
    let expected_dot = euclidean.dual_dot(&left, &right).0;
    for a in -3i64..=3 {
        for b in -2i64..=2 {
            let c = 2;
            let d = 3;
            let determinant = a * d - b * c;
            if determinant == 0 {
                continue;
            }
            let map = |value: &Vector2| {
                let x = value
                    .x
                    .scaled(&IBig::from(a), &UBig::ONE)
                    .add(&value.y.scaled(&IBig::from(c), &UBig::ONE))
                    .0;
                let y = value
                    .x
                    .scaled(&IBig::from(b), &UBig::ONE)
                    .add(&value.y.scaled(&IBig::from(d), &UBig::ONE))
                    .0;
                Vector2::new(x, y)
            };
            let mapped_left = map(&left);
            let mapped_right = map(&right);
            let mut chart = DensityTransaction::new(metric(
                a * a + c * c,
                a * b + c * d,
                b * b + d * d,
                if determinant > 0 { 1 } else { -1 },
            ));
            assert_eq!(chart.dual_dot(&mapped_left, &mapped_right).0, expected_dot);
            for q in 2..=6 {
                let within = euclidean.subturn(&left, &right, q).unwrap().0;
                let boundary = euclidean.subturn_boundary(&left, &right, q).unwrap().0;
                assert_eq!(
                    chart.subturn(&mapped_left, &mapped_right, q).unwrap().0,
                    within
                );
                assert_eq!(
                    chart
                        .subturn_boundary(&mapped_left, &mapped_right, q)
                        .unwrap()
                        .0,
                    boundary
                );
                assert_eq!(
                    chart
                        .subturn(&scale(&mapped_left, 2, 3), &scale(&mapped_right, 5, 2), q)
                        .unwrap()
                        .0,
                    within
                );
                assert_eq!(
                    chart
                        .subturn_boundary(
                            &scale(&mapped_left, 2, 3),
                            &scale(&mapped_right, 5, 2),
                            q
                        )
                        .unwrap()
                        .0,
                    boundary
                );
            }
        }
    }
}

#[test]
fn arbitrary_small_rational_residuals_are_decided_exactly() {
    let denominator = UBig::ONE << 220usize;
    let epsilon = Radical::rational(IBig::ONE, denominator);
    let left = vector(1, 0);
    let mut transaction = identity();
    let inside = Vector2::new(Radical::one(), Radical::one().sub(&epsilon).0);
    let outside = Vector2::new(Radical::one(), Radical::one().add(&epsilon).0);
    assert!(transaction.subturn(&left, &inside, 4).unwrap().0);
    assert!(!transaction.subturn(&left, &outside, 4).unwrap().0);
    assert!(!transaction.subturn_boundary(&left, &inside, 4).unwrap().0);
    assert!(!transaction.subturn_boundary(&left, &outside, 4).unwrap().0);
}

#[test]
fn memo_is_local_to_a_transaction_and_only_reuses_success() {
    let left = Vector2::new(root(2, 1).add(&root(3, 1)).0, root(5, 1));
    let right = Vector2::new(root(7, 1), root(11, 1));
    let mut first = identity();
    let cold = first.subturn(&left, &right, 5).unwrap();
    assert!(cold.1.class_merge_checks > 0);
    assert_eq!(
        first.subturn(&left, &right, 5).unwrap(),
        (cold.0, Work::NONE)
    );
    assert_eq!(
        first.subturn(&right, &left, 5).unwrap(),
        (cold.0, Work::NONE)
    );
    assert_eq!(identity().subturn(&left, &right, 5).unwrap(), cold);
}

#[test]
fn inverse_class_limit_is_an_internal_algebra_error_with_its_original_work() {
    let value = Radical::from_terms(&[
        (IBig::from(2), IBig::ONE, UBig::ONE),
        (IBig::from(3), IBig::ONE, UBig::ONE),
        (IBig::from(5), IBig::ONE, UBig::ONE),
        (IBig::from(7), IBig::ONE, UBig::ONE),
    ])
    .unwrap()
    .0;
    let failure = value.inverse().unwrap_err();
    let before = Work::checks(17) + Work::rounds(2);
    let preserved = DensityFailure::from_algebra(failure.clone(), before);
    assert_eq!(preserved.kind, DensityFailureKind::InternalAlgebra(failure));
    assert_eq!(preserved.work, before);
    match preserved.kind {
        DensityFailureKind::InternalAlgebra(Failure::Refused(refusal)) => {
            assert_eq!(refusal.code, RefusalCode::InverseClassLimit)
        }
        other => panic!("lost the algebra failure category: {other:?}"),
    }
}
