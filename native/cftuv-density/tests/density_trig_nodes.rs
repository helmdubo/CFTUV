use cftuv_density::density_expression::{Expressions, Node};
use cftuv_density::density_quadratic::{QuadraticFailureKind, QuadraticReader};
use cftuv_density::density_trig_nodes::{exact_q_trig, TrigNodeFailure};
use cftuv_density::legacy_mpf::Precision;
use cftuv_density::{IBig, Radical, UBig};

#[test]
fn fifth_sine_stays_interval_only_and_other_pairs_are_quadratic() {
    let mut graph = Expressions::new();
    for q in 2..=6 {
        let [c, s] = exact_q_trig(&mut graph, q).unwrap();
        let mut reader = QuadraticReader::new(&graph);
        let cosine = reader.read(c).unwrap().0;
        if q == 5 {
            assert_eq!(
                cosine,
                Radical::one()
                    .add(
                        &Radical::sqrt_of_rational(IBig::from(5), UBig::ONE)
                            .unwrap()
                            .0
                    )
                    .0
                    .scaled(&IBig::ONE, &UBig::from(4u8))
            );
            assert!(matches!(
                reader.read(s).unwrap_err().kind,
                QuadraticFailureKind::UnsupportedSyntax(_)
            ));
            assert!(
                graph
                    .interval(s, Precision::new(160).unwrap())
                    .unwrap()
                    .lower()
                    .sign()
                    > 0
            );
        } else {
            let sine = reader.read(s).unwrap().0;
            assert_eq!(
                cosine.mul(&cosine).0.add(&sine.mul(&sine).0).0,
                Radical::one()
            );
        }
    }
}

#[test]
fn unsupported_q_does_not_mutate_arena() {
    let mut graph = Expressions::new();
    assert_eq!(
        exact_q_trig(&mut graph, 12),
        Err(TrigNodeFailure::UnsupportedDensityQ(12))
    );
    let [zero, _] = exact_q_trig(&mut graph, 2).unwrap();
    assert_eq!(zero, 0);
    assert!(matches!(graph.node(zero).unwrap(), Node::Rational(_)));
}
