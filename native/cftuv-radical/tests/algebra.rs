//! The laws the type stands on, checked in Rust alone (no oracle, no Python): the value-canonical form is a function of the value, the field axioms hold on random sums of square roots, the
//! sign agrees with a binary64 evaluation where that is decisive, the near-zero families get their sign inside the separation bound, and the price caps are named refusals that carry their work.
//! The differential tests against the oracle are `tests/vectors.rs` (and `tests/test_native_radical_v2.py` with the extension).

use cftuv_radical::{Failure, IBig, Radical, RefusalCode, Term, UBig, Work};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }

    fn range(&mut self, low: i64, high: i64) -> i64 {
        low + self.below((high - low + 1) as u64) as i64
    }
}

const SQUAREFREE: [u64; 12] = [2, 3, 5, 6, 7, 10, 11, 13, 14, 15, 17, 19];

fn ib(value: i64) -> IBig {
    IBig::from(value)
}

fn ub(value: u64) -> UBig {
    UBig::from(value)
}

/// `sum c * sqrt(r)` from `(radicand, numerator, denominator)`.
fn value(pairs: &[(i64, i64, u64)]) -> Radical {
    let pairs: Vec<(IBig, IBig, UBig)> = pairs.iter().map(|&(r, n, d)| (ib(r), ib(n), ub(d))).collect();
    Radical::from_terms(&pairs).unwrap().0
}

/// A random value over `low..=high` square classes (squares hidden in the radicands, classes split in parts), plus a rational part with probability 1/2.
fn random_between(rng: &mut Rng, low: i64, high: i64) -> Radical {
    let classes = rng.range(low, high);
    let rational = rng.below(2) == 0;
    let mut pairs: Vec<(i64, i64, u64)> = Vec::new();
    let mut pool = SQUAREFREE.to_vec();
    for _ in 0..classes {
        let radicand = pool.remove(rng.below(pool.len() as u64) as usize) as i64;
        for _ in 0..rng.range(1, 2) {
            let square = [1, 1, 2, 3][rng.below(4) as usize];
            pairs.push((radicand * square * square, rng.range(-9, 9), rng.range(1, 9) as u64 * square as u64));
        }
    }
    if rational {
        pairs.push((1, rng.range(-9, 9), rng.range(1, 9) as u64));
    }
    value(&pairs)
}

fn random_value(rng: &mut Rng, high: i64) -> Radical {
    random_between(rng, 0, high)
}

fn rational(numerator: i64, denominator: u64) -> Radical {
    Radical::rational(ib(numerator), ub(denominator))
}

/// A binary64 evaluation of the value (the terms are `n/d sqrt(m')`).
fn float_of(value: &Radical) -> f64 {
    value
        .terms()
        .iter()
        .map(|term| {
            let n = f64::from(i32::try_from(&term.numerator).unwrap());
            let d = u64::try_from(&term.denominator).unwrap() as f64;
            n / d * (u64::try_from(&term.radicand).unwrap() as f64).sqrt()
        })
        .sum()
}

fn assert_canonical(value: &Radical, what: &str) {
    let terms = value.terms();
    for (index, term) in terms.iter().enumerate() {
        assert!(!term.numerator.is_zero() && term.denominator > UBig::ZERO, "{what}: a zero term");
        if index > 0 {
            assert!(terms[index - 1].radicand < term.radicand, "{what}: terms in m' order");
        }
        if term.radicand.is_one() {
            assert_eq!(index, 0, "{what}: the rational term comes first");
        } else {
            assert!(term.numerator == IBig::ONE || term.numerator == IBig::NEG_ONE, "{what}: a sign");
            assert!(&(&term.radicand / &term.denominator) * &term.denominator == term.radicand, "{what}: Q divides m'");
        }
    }
    // no two terms of one class: the product of their radicands is never a perfect square
    for left in 0..terms.len() {
        for right in left + 1..terms.len() {
            let product = &terms[left].radicand * &terms[right].radicand;
            let root = dashu_int_sqrt(&product);
            assert!(&root * &root != product, "{what}: two terms of one class");
        }
    }
}

fn dashu_int_sqrt(n: &UBig) -> UBig {
    dashu_int::ops::SquareRoot::sqrt(n)
}

#[test]
fn the_field_axioms_hold_on_random_sums_of_square_roots() {
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let one = Radical::one();
    for round in 0..400 {
        let x = random_value(&mut rng, 3);
        let y = random_value(&mut rng, 2);
        let z = random_value(&mut rng, 2);
        for (name, v) in [("x", &x), ("y", &y), ("z", &z)] {
            assert_canonical(v, &format!("round {round} {name}"));
        }
        let (xy, _) = x.add(&y);
        assert_eq!(xy, y.add(&x).0, "round {round}: + commutes");
        assert_eq!(xy.sub(&y).0, x, "round {round}: (x + y) - y = x");
        assert_eq!(x.mul(&y).0, y.mul(&x).0, "round {round}: * commutes");
        assert_eq!(x.mul(&y).0.mul(&z).0, x.mul(&y.mul(&z).0).0, "round {round}: * associates");
        let (sum, _) = y.add(&z);
        assert_eq!(x.mul(&sum).0, x.mul(&y).0.add(&x.mul(&z).0).0, "round {round}: distributes");
        assert!(x.add(&x.neg()).0.is_zero(), "round {round}: x + (-x) = 0");
        assert_eq!(x.sub(&y).0, x.add(&y.neg()).0, "round {round}: x - y = x + (-y)");
        assert_canonical(&x.mul(&y).0, &format!("round {round} x*y"));
        assert_canonical(&xy, &format!("round {round} x+y"));
        if !x.is_zero() && x.terms().len() <= 3 {
            let (inverse, work) = x.inverse().unwrap();
            assert_eq!(x.mul(&inverse).0, one, "round {round}: x * x^-1 = 1");
            assert_canonical(&inverse, &format!("round {round} x^-1"));
            assert_eq!(work.conjugation_products % 2, 0, "conjugations come in pairs");
            if inverse.terms().len() <= 3 {
                assert_eq!(inverse.inverse().unwrap().0, x, "round {round}: x^-1^-1 = x");
            }
            assert_eq!(y.div(&x).unwrap().0, y.mul(&inverse).0, "round {round}: y / x = y * x^-1");
        }
        // scaled is a free product by a rational
        let factor = (rng.range(-7, 7), rng.range(1, 7) as u64);
        assert_eq!(x.scaled(&ib(factor.0), &ub(factor.1)), x.mul(&rational(factor.0, factor.1)).0, "round {round}: scaled");
    }
}

#[test]
fn text_and_wire_read_back_to_the_same_value() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    for _ in 0..300 {
        let x = random_value(&mut rng, 4);
        let text = x.text();
        assert_eq!(Radical::from_text(&text).unwrap().0, x, "{text}");
        assert_eq!(Radical::from_wire(&x.wire()).unwrap(), x);
        assert_eq!(Radical::from_text(&text).unwrap().0.text(), text);
    }
    // one value in several representations is one canonical form: terms, text, wire
    let forms: [&[(i64, i64, u64)]; 6] = [&[(18, 1, 1)], &[(2, 3, 1)], &[(8, 3, 2)], &[(2, 1, 1), (8, 1, 1)], &[(50, 3, 5)], &[(72, 1, 2)]];
    let first = value(forms[0]);
    for form in forms {
        let v = value(form);
        assert_eq!((v.terms(), v.text(), v.wire()), (first.terms(), first.text(), first.wire()));
    }
    assert_eq!(first.text(), "Sqrt(Rational(18, 1))");
}

#[test]
fn the_sign_agrees_with_binary64_where_that_is_decisive_and_the_enclosure_holds_the_value() {
    let mut rng = Rng(0x1234_5678_9abc_def1);
    let mut decided_by_ladder = 0;
    for _ in 0..600 {
        let x = random_between(&mut rng, 1, 5);
        let float = float_of(&x);
        let (sign, work) = x.signum(None).unwrap();
        if float.abs() > 1e-6 {
            assert_eq!(sign, if float > 0.0 { 1 } else { -1 }, "{}: float {float}", x.text());
        }
        if x.terms().len() >= 3 {
            assert!(work.enclosure_rounds >= 1);
            decided_by_ladder += 1;
        } else {
            assert_eq!(work, Work::NONE, "a sign of at most two classes costs no enclosure");
        }
        let ((low_n, low_d), (high_n, high_d)) = x.enclosure(60).0;
        let (low, high) = (f64_of(&low_n, &low_d), f64_of(&high_n, &high_d));
        assert!(low <= float + 1e-9 && float - 1e-9 <= high, "{}: [{low}, {high}] holds {float}", x.text());
    }
    assert!(decided_by_ladder > 100);
}

fn f64_of(numerator: &IBig, denominator: &UBig) -> f64 {
    // a quotient of two integers that fit in f64 range here (60-bit enclosures of small sums)
    let text_n: f64 = numerator.to_string().parse().unwrap();
    let text_d: f64 = denominator.to_string().parse().unwrap();
    text_n / text_d
}

/// `(sqrt2 - 1)^n = a + b sqrt2` and `(2 - sqrt3)^m = c + d sqrt3`.
fn pell(n: usize, m: usize) -> ((IBig, IBig), (IBig, IBig)) {
    let (mut a, mut b) = (IBig::ONE, IBig::ZERO);
    for _ in 0..n {
        (a, b) = (-&a + &b * ib(2), &a - &b);
    }
    let (mut c, mut d) = (IBig::ONE, IBig::ZERO);
    for _ in 0..m {
        (c, d) = (&c * ib(2) - &d * ib(3), &d * ib(2) - &c);
    }
    ((a, b), (c, d))
}

fn sum3(a: &IBig, b: &IBig, c: &IBig, d: &IBig, sign: i64) -> Radical {
    let pairs = vec![(IBig::ONE, a + c * ib(sign), UBig::ONE), (ib(2), b.clone(), UBig::ONE), (ib(3), d * ib(sign), UBig::ONE)];
    Radical::from_terms(&pairs).unwrap().0
}

#[test]
fn near_zero_sums_get_their_sign_and_the_caps_are_named_prices() {
    // (sqrt2 - 1)^n + (2 - sqrt3)^m > 0 near 2^-3800 and 2^-5100: the first is decided inside the seven rungs V1 had (64 .. 4096), the second needs the eighth (8192 bits)
    for ((n, m), beyond) in [((3000, 2000), false), ((4000, 2700), true)] {
        let ((a, b), (c, d)) = pell(n, m);
        let x = sum3(&a, &b, &c, &d, 1);
        let (sign, work) = x.signum(None).unwrap();
        assert_eq!(sign, 1, "({n}, {m})");
        assert_eq!(work.enclosure_rounds > 7, beyond, "({n}, {m}): rounds {}", work.enclosure_rounds);
        let required = cftuv_radical::ladder::separation_bits(&x.integer_form().items);
        assert!(UBig::from(64u64 << (work.enclosure_rounds - 1)) < required, "the ladder stops short of the bound");
        if beyond {
            match x.signum(Some(4096)) {
                Err(Failure::Refused(refusal)) => {
                    assert_eq!(refusal.code, RefusalCode::SignUndecided);
                    assert_eq!(refusal.code.code(), "EXACT_NATIVE_SIGN_UNDECIDED");
                    assert_eq!(refusal.work, Work::rounds(7), "the work done before the refusal");
                    assert!(refusal.detail.starts_with("3 classes: the separation bound needs "), "{}", refusal.detail);
                    assert!(refusal.detail.ends_with(" bits, the cap is 4096"), "{}", refusal.detail);
                    assert_eq!(refusal.message(), format!("EXACT_NATIVE_SIGN_UNDECIDED: {}", refusal.detail));
                }
                other => panic!("({n}, {m}) under the cap of 4096: {other:?}"),
            }
            assert_eq!(x.signum(Some(8192)).unwrap(), (1, Work::rounds(8)));
        } else {
            assert_eq!(x.signum(Some(4096)).unwrap().0, 1);
        }
    }
    // u - v with u = (sqrt2-1)^n, v = (2-sqrt3)^m: the sign is that of m ln(2+sqrt3) - n ln(sqrt2+1); n = 300, m = 201: 201 * 1.3169579 = 264.7 < 300 * 0.8813736 = 264.4?
    let ((a, b), (c, d)) = pell(300, 201);
    let difference = sum3(&a, &b, &c, &d, -1);
    let (sign, _) = difference.signum(None).unwrap();
    let (u, v) = (300.0 * (2f64.sqrt() + 1.0).ln(), 201.0 * (2.0 + 3f64.sqrt()).ln());
    // u - v > 0 exactly when the (sqrt2-1)^n is the smaller one, i.e. when n ln(sqrt2+1) > m ln(2+sqrt3)
    assert_eq!(sign, if u > v { -1 } else { 1 }, "u = {u}, v = {v}");
}

#[test]
fn a_value_cancels_across_representations_and_the_empty_sum_is_zero() {
    for pairs in [
        vec![(2, 1, 1), (8, 1, 1), (18, -1, 1)],
        vec![(12, 1, 1), (3, -2, 1)],
        vec![(24, 1, 6), (6, -1, 3)],
        vec![(50, 3, 5), (2, -3, 1)],
        vec![(1, 5, 1), (4, -5, 2)],
        vec![(72, 1, 1), (8, -3, 1)],
    ] {
        let x = value(&pairs);
        assert!(x.is_zero() && x.terms().is_empty() && x.text() == "Integer(0)", "{pairs:?}");
        assert_eq!(x.signum(None).unwrap(), (0, Work::NONE));
    }
    let (square, _) = value(&[(2, 1, 1), (3, 1, 1)]).mul(&value(&[(2, 1, 1), (3, 1, 1)]));
    let (rest, _) = square.sub(&rational(5, 1));
    let (six, _) = value(&[(2, 1, 1)]).mul(&value(&[(3, 1, 1)]));
    assert!(rest.sub(&six.scaled(&ib(2), &ub(1))).0.is_zero(), "(sqrt2 + sqrt3)^2 = 5 + 2 sqrt6");
}

#[test]
fn the_named_refusals_carry_their_codes() {
    let four = value(&[(1, 1, 1), (2, 1, 1), (3, 1, 1), (5, 1, 1)]);
    match four.inverse() {
        Err(Failure::Refused(refusal)) => {
            assert_eq!(refusal.code.code(), "EXACT_NATIVE_INVERSE_CLASS_LIMIT");
            assert_eq!(refusal.message(), "EXACT_NATIVE_INVERSE_CLASS_LIMIT: 4 classes, limit 3");
            assert_eq!(refusal.work, Work::NONE);
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(Radical::zero().inverse().unwrap_err(), Failure::ZeroDivision);
    let refusal = Radical::sqrt_of_rational(ib(-1), ub(2)).unwrap_err();
    assert_eq!((refusal.code.code(), refusal.detail.as_str()), ("EXACT_NATIVE_OUTSIDE_FIELD", "root of a negative number"));
    // a refusal in the middle of a reading carries the checks done before it
    let refusal = Radical::from_terms(&[(ib(2), ib(1), ub(1)), (ib(3), ib(1), ub(1)), (ib(-5), ib(1), ub(1))]).unwrap_err();
    assert_eq!(refusal.work, Work::checks(3), "two radicands asked if they are squares (2), the second also asked the first class (1)");
    // a zero cap is below the first rung
    let three = value(&[(1, 1, 1), (2, 1, 1), (3, 1, 1)]);
    match three.signum(Some(63)) {
        Err(Failure::Refused(refusal)) => {
            assert_eq!(refusal.work, Work::NONE);
            assert_eq!(refusal.detail, "3 classes: the separation bound needs more than 63 bits, the cap is 63");
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(three.signum(Some(64)).unwrap(), (1, Work::rounds(1)));
}

#[test]
fn the_terms_of_a_canonical_value_are_triples_in_m_order() {
    // sqrt3 / 2 = sqrt(3/4) = sqrt(12) / 4: (12, 1, 4); -sqrt2 is (2, -1, 1)
    let x = value(&[(1, -3, 4), (2, -1, 1), (3, 1, 2)]);
    let shown: Vec<(u64, i64, u64)> = x.terms().iter().map(|Term { radicand, numerator, denominator }| (u64::try_from(radicand).unwrap(), i64::try_from(numerator).unwrap(), u64::try_from(denominator).unwrap())).collect();
    assert_eq!(shown, vec![(1, -3, 4), (2, -1, 1), (12, 1, 4)]);
    assert_eq!(x.text(), "Sum(Rational(-3, 4), -Sqrt(Rational(2, 1)), Sqrt(Rational(3, 4)))");
}
