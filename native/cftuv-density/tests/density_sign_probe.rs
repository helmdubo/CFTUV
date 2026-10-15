use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_sign::{SignEvaluation, SignFailure, SignPath, SourceSignProbe};
use cftuv_density::{IBig, Rational, UBig};

fn q(g: &mut Expressions, n: i32, d: u32) -> usize {
    g.push(Node::Rational(
        Rational::new(IBig::from(n), UBig::from(d)).unwrap(),
    ))
    .unwrap()
}

#[test]
fn interval_success_never_requires_a_quadratic_or_expanded_form() {
    let mut g = Expressions::new();
    let one = q(&mut g, 1, 1);
    let sine = g.push(Node::Sin(one)).unwrap();
    let SourceSignProbe::Decided(result) = g.probe_sign(sine).unwrap() else {
        panic!("положительный интервал не требует expand")
    };
    assert_eq!((result.sign, result.path), (1, SignPath::Interval));
    assert_eq!(
        SignEvaluation::new(&g).probe_sign(sine).unwrap(),
        SourceSignProbe::Decided(result)
    );
    assert_eq!(
        g.sign_with_expanded_fallback(sine, usize::MAX).unwrap(),
        result
    );
    assert_eq!(
        g.sign_with_expanded_fallback(sine, usize::MAX)
            .unwrap()
            .path,
        SignPath::Memo
    );
}

#[test]
fn unresolved_interval_does_not_admit_raw_syntax_or_hide_internal_failure() {
    let mut g = Expressions::new();
    let one = q(&mut g, 1, 1);
    let zero = q(&mut g, 0, 1);
    let tan = g.push(Node::Tan(one)).unwrap();
    assert_eq!(g.probe_sign(tan).unwrap(), SourceSignProbe::NeedsExpanded);
    assert_eq!(
        g.sign_with_expanded_fallback(tan, tan),
        Err(SignFailure::Undecidable)
    );
    // У составного выражения expand вправе уничтожить непрочитываемые атомы.
    let minus = q(&mut g, -1, 1);
    let negative = g.push(Node::Mul(vec![minus, tan])).unwrap();
    let cancellation = g.push(Node::Add(vec![tan, negative])).unwrap();
    assert_eq!(
        g.probe_sign(cancellation).unwrap(),
        SourceSignProbe::NeedsExpanded
    );
    let expanded = g.sign_with_expanded_fallback(cancellation, zero).unwrap();
    assert_eq!((expanded.sign, expanded.path), (0, SignPath::Quadratic));
    assert!(matches!(
        g.probe_sign(usize::MAX),
        Err(SignFailure::IntervalInternal(_))
    ));
    assert_eq!(
        SignEvaluation::new(&g).probe_sign(tan).unwrap(),
        SourceSignProbe::NeedsExpanded
    );
}
