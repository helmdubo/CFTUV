use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_quadratic::{QuadraticFailureKind, QuadraticReader};
use cftuv_density::{IBig, Radical, Rational, UBig, Work};
use cftuv_radical::{Failure, RefusalCode};
fn q(g: &mut Expressions, n: i32, d: u32) -> usize {
    g.push(Node::Rational(
        Rational::new(IBig::from(n), UBig::from(d)).unwrap(),
    ))
    .unwrap()
}
#[test]
fn nested_root_is_not_admitted_by_value_and_zero_power_reads_base_first() {
    let mut g = Expressions::new();
    let one = q(&mut g, 1, 1);
    let two = q(&mut g, 2, 1);
    let half = q(&mut g, 1, 2);
    let zero = q(&mut g, 0, 1);
    let root = g.push(Node::Pow(two, half)).unwrap();
    let sum = g.push(Node::Add(vec![one, root])).unwrap();
    let nested = g.push(Node::Pow(sum, half)).unwrap();
    let no_escape = g.push(Node::Pow(nested, zero)).unwrap();
    let mut reader = QuadraticReader::new(&g);
    assert!(matches!(
        reader.read(nested).unwrap_err().kind,
        QuadraticFailureKind::UnsupportedSyntax(_)
    ));
    assert!(matches!(
        reader.read(no_escape).unwrap_err().kind,
        QuadraticFailureKind::UnsupportedSyntax(_)
    ));
    let (value, _) = reader.read(sum).unwrap();
    assert_eq!(
        value,
        Radical::one()
            .add(
                &Radical::sqrt_of_rational(IBig::from(2), UBig::ONE)
                    .unwrap()
                    .0
            )
            .0
    );
    assert_eq!(reader.read(sum).unwrap().1, Work::NONE);
}
#[test]
fn inverse_class_limit_and_zero_division_stay_internal_algebra_outcomes() {
    let mut g = Expressions::new();
    let half = q(&mut g, 1, 2);
    let inverse = q(&mut g, -1, 1);
    let mut roots = vec![];
    for n in [2, 3, 5, 7] {
        let n = q(&mut g, n, 1);
        roots.push(g.push(Node::Pow(n, half)).unwrap());
    }
    let sum = g.push(Node::Add(roots)).unwrap();
    let inv = g.push(Node::Pow(sum, inverse)).unwrap();
    let zero = q(&mut g, 0, 1);
    let inv_zero = g.push(Node::Pow(zero, inverse)).unwrap();
    let mut reader = QuadraticReader::new(&g);
    let error = reader.read(inv).unwrap_err();
    assert!(
        matches!(error.kind, QuadraticFailureKind::InternalAlgebra(Failure::Refused(ref r)) if r.code == RefusalCode::InverseClassLimit)
    );
    assert_eq!(
        reader.read(inv_zero).unwrap_err().kind,
        QuadraticFailureKind::InternalAlgebra(Failure::ZeroDivision)
    );
}

#[test]
fn negative_power_inverts_three_class_base_before_four_class_square() {
    let mut g = Expressions::new();
    let half = q(&mut g, 1, 2);
    let minus_two = q(&mut g, -2, 1);
    let mut roots = Vec::new();
    for n in [2, 3, 5] {
        let n = q(&mut g, n, 1);
        roots.push(g.push(Node::Pow(n, half)).unwrap());
    }
    let base = g.push(Node::Add(roots)).unwrap();
    let negative_square = g.push(Node::Pow(base, minus_two)).unwrap();
    let (base_value, base_work) = QuadraticReader::new(&g).read(base).unwrap();
    let (old_square, _) = base_value.mul(&base_value);
    assert_eq!(old_square.terms().len(), 4);
    assert!(matches!(old_square.inverse(),
        Err(Failure::Refused(ref r)) if r.code == RefusalCode::InverseClassLimit));
    let (inverse, inverse_work) = base_value.inverse().unwrap();
    let (square, square_work) = inverse.mul(&inverse);
    let (expected, one_work) = Radical::one().mul(&square);
    let mut reader = QuadraticReader::new(&g);
    let (actual, work) = reader.read(negative_square).unwrap();
    assert_eq!(actual, expected);
    assert_eq!(work, base_work + inverse_work + square_work + one_work);
    assert_eq!(reader.read(negative_square).unwrap().1, Work::NONE);
}
