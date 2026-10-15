use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_quadratic::QuadraticFailureKind;
use cftuv_density::density_shell_v2::{
    density_shell_v2, integer_shell, Candidate, NotApplicableCode, OutcomeCode, ShellError,
    ShellResult, SHELL_QUANTUM,
};
use cftuv_density::interval::RationalInterval;
use cftuv_density::{IBig, Rational, UBig};
use cftuv_radical::Failure;
fn q(g: &mut Expressions, n: i64, d: u32) -> usize {
    g.push(Node::Rational(
        Rational::new(IBig::from(n), UBig::from(d)).unwrap(),
    ))
    .unwrap()
}
fn root(g: &mut Expressions, n: i64) -> usize {
    let n = q(g, n, 1);
    let half = q(g, 1, 2);
    g.push(Node::Pow(n, half)).unwrap()
}
fn accepted(g: &Expressions, id: usize) -> Candidate {
    let ShellResult::Applied(v) = density_shell_v2(g, id).unwrap() else {
        panic!("expected candidate");
    };
    v
}
#[test]
fn rational_grid_quantum_metadata_and_non_dyadic_straw_are_exact() {
    let mut g = Expressions::new();
    for (n, d, grid, straw) in [
        (0, 1, true, false),
        (1, 8, true, false),
        (-1657, 5120, true, true),
        (3647, 5120, true, true),
        (1, 3, false, false),
    ] {
        let id = q(&mut g, n, d);
        let v = accepted(&g, id);
        assert_eq!(v.shell.quantum, SHELL_QUANTUM);
        assert_eq!(v.outcomes.codes.contains(&OutcomeCode::GridPoint), grid);
        assert_eq!(
            v.outcomes.codes.contains(&OutcomeCode::NonDyadicGridStraw),
            straw
        );
        assert_eq!(
            v.outcomes.legacy_projected_min_width_units,
            if straw { Some(2) } else { None }
        );
        let metadata = v.outcomes.rational.unwrap();
        assert_eq!(metadata.numerator, IBig::from(n));
        assert_eq!(metadata.denominator, UBig::from(d));
        if grid {
            assert_eq!(v.shell.width_units, IBig::ZERO);
            assert_eq!(metadata.grid_units, Some(v.shell.lower_units.clone()));
        } else {
            assert_eq!(metadata.grid_units, None);
            assert_eq!(v.shell.width_units, IBig::ONE);
        }
    }
}
#[test]
fn equivalent_raw_root_forms_have_the_same_value_enclosure_and_shell() {
    let mut g = Expressions::new();
    let r2 = root(&mut g, 2);
    let r8 = root(&mut g, 8);
    let half = q(&mut g, 1, 2);
    let equivalent = g.push(Node::Mul(vec![half, r8])).unwrap();
    let a = accepted(&g, r2);
    let b = accepted(&g, equivalent);
    assert_eq!(a.value, b.value);
    assert_eq!(a.enclosure160, b.enclosure160);
    assert_eq!(a.shell, b.shell);
    assert_eq!(a.outcomes.codes, vec![OutcomeCode::Applied]);
    assert_eq!(a.outcomes.rational, None);
    assert_eq!(density_shell_v2(&g, r2), density_shell_v2(&g, r2)); // Invocation memo не протекает.
}
#[test]
fn integer_width_never_uses_decimal_context_rounding() {
    let size = format!("1{}", "0".repeat(100)).parse::<IBig>().unwrap();
    let interval =
        RationalInterval::new(Rational::integer(-&size), Rational::integer(size)).unwrap();
    let shell = integer_shell(&interval);
    let expected = format!("2{}", "0".repeat(127)).parse::<IBig>().unwrap();
    assert_eq!(shell.width_units, expected);
    assert_eq!(shell.quantum, -27);
    let interval = RationalInterval::point(Rational::new(IBig::from(-1), UBig::from(3u8)).unwrap());
    let shell = integer_shell(&interval);
    assert_eq!(
        shell.lower_units,
        "-333333333333333333333333334".parse::<IBig>().unwrap()
    );
    assert_eq!(
        shell.upper_units,
        "-333333333333333333333333333".parse::<IBig>().unwrap()
    );
}
#[test]
fn four_class_value_is_allowed_but_inverse_is_not_applicable() {
    let mut g = Expressions::new();
    let mut roots = Vec::new();
    for n in [2, 3, 5, 7] {
        roots.push(root(&mut g, n));
    }
    let sum = g.push(Node::Add(roots)).unwrap();
    assert_eq!(accepted(&g, sum).value.terms().len(), 4);
    let minus = q(&mut g, -1, 1);
    let inverse = g.push(Node::Pow(sum, minus)).unwrap();
    let ShellResult::NotApplicable(v) = density_shell_v2(&g, inverse).unwrap() else {
        panic!("expected inverse class limit");
    };
    assert_eq!(v.code, NotApplicableCode::InverseClassLimit);
}
#[test]
fn unsupported_children_are_read_even_after_zero_and_zero_division_stays_internal() {
    let mut g = Expressions::new();
    let zero = q(&mut g, 0, 1);
    let pi = g.push(Node::Pi).unwrap();
    let product = g.push(Node::Mul(vec![zero, pi])).unwrap();
    let ShellResult::NotApplicable(v) = density_shell_v2(&g, product).unwrap() else {
        panic!("expected unsupported");
    };
    assert_eq!(v.code, NotApplicableCode::UnsupportedSyntax);
    let minus = q(&mut g, -1, 1);
    let inverse = g.push(Node::Pow(zero, minus)).unwrap();
    assert!(
        matches!(density_shell_v2(&g,inverse),Err(ShellError::Reader(ref e))
        if e.kind==QuadraticFailureKind::InternalAlgebra(Failure::ZeroDivision))
    );
    assert!(
        matches!(density_shell_v2(&g,usize::MAX),Err(ShellError::Reader(ref e))
        if matches!(e.kind,QuadraticFailureKind::InvalidReference(_)))
    );
}
#[test]
fn declared_exponent_profile_is_internal_instead_of_not_applicable() {
    let mut g = Expressions::new();
    let zero = q(&mut g, 0, 1);
    let huge = q(&mut g, 1048577, 1);
    let pow = g.push(Node::Pow(zero, huge)).unwrap();
    assert!(
        matches!(density_shell_v2(&g,pow),Err(ShellError::Reader(ref e))
        if matches!(e.kind,QuadraticFailureKind::ExponentProfile(_)))
    );
}
