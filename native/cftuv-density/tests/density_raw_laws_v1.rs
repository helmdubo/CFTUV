use cftuv_density::density_construct_v1::ConstructorV1;
use cftuv_density::density_expand_v1::ExpandLimits;
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_raw_laws_v1::*;
use cftuv_density::density_raw_types_v1::*;
use cftuv_density::farey::PrimitiveVector;
use cftuv_density::{IBig, Rational, UBig};
fn q(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn p(x: i32, y: i32) -> RawDensityPointV1 {
    RawDensityPointV1 {
        x: q(x, 1),
        y: q(y, 1),
    }
}
fn limits() -> ExpandLimits {
    ExpandLimits {
        max_terms: 4096,
        max_polynomial_exponent: 16,
        max_rational_power_bits: 65536,
    }
}
fn incident() -> RawDensityIncidentV1 {
    RawDensityIncidentV1 {
        start: p(0, 0),
        end: p(1, 0),
        owner_left: Some(true),
        anchor_vertex_id: None,
        source_vertex_start_id: None,
        source_vertex_end_id: None,
        support_id: None,
        chain_use_orientation: None,
        source_direction_declarations: vec![],
        owner_face_cycles: vec![],
        owner_face_vertex_id_cycles: vec![],
        owner_face_ids: vec![],
        owner_face_edge_ordinals: vec![],
        physical_edge_id: Some("edge".into()),
        chain_use_id: Some("use".into()),
    }
}
fn raw(skew: bool, owner: i8) -> RawDensityGeometryV1 {
    let mut outgoing = incident();
    outgoing.end = p(0, 1);
    RawDensityGeometryV1 {
        metric: RawDensityMetricV1 {
            gram: if skew {
                [q(2, 1), q(1, 2), q(1, 1)]
            } else {
                [q(1, 1), q(0, 1), q(1, 1)]
            },
            inverse_gram: if skew {
                [q(4, 7), q(-2, 7), q(8, 7)]
            } else {
                [q(1, 1), q(0, 1), q(1, 1)]
            },
            owner_orientation_sign: owner,
            metric_id: Some("metric".into()),
        },
        incoming: incident(),
        outgoing,
    }
}
#[test]
fn raw_rational_incidents_produce_gram_unit_normals_and_invocation_local_metrics() {
    for skew in [false, true] {
        for owner in [-1, 1] {
            let mut graph = Expressions::new();
            let mut c = ConstructorV1::new(&mut graph);
            let mut laws = RawDensityLawsV1::new(&mut c, limits());
            let input = raw(skew, owner);
            let geometry = laws.raw_geometry(&input, true).unwrap();
            let norm = laws
                .dot_g(&geometry.metric, &geometry.incoming, &geometry.incoming)
                .unwrap();
            assert_eq!(laws.fraction(&norm).unwrap(), q(1, 1));
            let expected = if skew {
                PrimitiveVector(IBig::from(-owner), IBig::from(4 * owner))
            } else {
                PrimitiveVector(IBig::ZERO, IBig::from(owner))
            };
            assert_eq!(
                laws.rational_direction(&geometry.incoming).unwrap(),
                RationalDirectionV1::Rational(expected)
            );
            let covector = laws.covector(&geometry.metric, &geometry.incoming).unwrap();
            let recovered = laws.primal(&geometry.metric, &covector).unwrap();
            assert_eq!(
                laws.rational_direction(&recovered).unwrap(),
                laws.rational_direction(&geometry.incoming).unwrap()
            );
            laws.raw_geometry(&input, true).unwrap();
            assert_eq!(laws.metric_cache_len(), 1);
            laws.raw_geometry(&input, false).unwrap();
            assert_eq!(laws.metric_cache_len(), 2);
        }
    }
}
#[test]
fn owner_face_cycles_override_fallback_side_and_declared_normals_keep_authority() {
    for skew in [false, true] {
        for owner in [-1, 1] {
            let mut graph = Expressions::new();
            let mut c = ConstructorV1::new(&mut graph);
            let mut laws = RawDensityLawsV1::new(&mut c, limits());
            let mut input = raw(skew, owner);
            input.incoming.owner_left = None;
            input.incoming.owner_face_cycles = vec![vec![p(0, 0), p(1, 0), p(1, 1), p(0, 1)]];
            input.incoming.owner_face_edge_ordinals = vec![0];
            let produced = laws.raw_geometry(&input, true).unwrap();
            let expected = if skew {
                PrimitiveVector(IBig::from(-1), IBig::from(4))
            } else {
                PrimitiveVector(IBig::ZERO, IBig::ONE)
            };
            assert_eq!(
                laws.rational_direction(&produced.incoming).unwrap(),
                RationalDirectionV1::Rational(expected)
            );
            input.incoming.source_direction_declarations =
                vec![RawDirectionDeclarationV1::CertifiedPlanar {
                    x: q(1, 1),
                    y: q(0, 1),
                }];
            let tangent_declaration = laws.raw_geometry(&input, true).unwrap();
            assert_eq!(
                laws.rational_direction(&produced.incoming).unwrap(),
                laws.rational_direction(&tangent_declaration.incoming)
                    .unwrap()
            );
            input.incoming.source_direction_declarations =
                vec![RawDirectionDeclarationV1::CertifiedPlanar {
                    x: q(if skew { -1 } else { 0 }, 1),
                    y: q(if skew { 4 } else { 1 }, 1),
                }];
            input.incoming.owner_face_cycles.clear();
            input.incoming.owner_face_edge_ordinals.clear();
            let authoritative = laws.raw_geometry(&input, true).unwrap();
            assert_eq!(
                laws.rational_direction(&produced.incoming).unwrap(),
                laws.rational_direction(&authoritative.incoming).unwrap()
            );
        }
    }
}
#[test]
fn raw_declared_direction_failures_remain_named_and_disabled_declarations_are_not_read() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let mut laws = RawDensityLawsV1::new(&mut c, limits());
    let mut input = raw(false, 1);
    input.incoming.source_direction_declarations =
        vec![RawDirectionDeclarationV1::CertifiedAffine {
            x: q(0, 1),
            y: q(1, 1),
            reference_metric_id: Some("other".into()),
        }];
    assert!(matches!(
        laws.raw_geometry(&input, true),
        Err(RawLawsFailureV1::Geometry(
            RawGeometryRefusalV1::AffineMetricMismatch
        ))
    ));
    assert!(laws.raw_geometry(&input, false).is_ok());
    input.incoming.source_direction_declarations = vec![
        RawDirectionDeclarationV1::CertifiedPlanar {
            x: q(0, 1),
            y: q(1, 1),
        },
        RawDirectionDeclarationV1::CertifiedPlanar {
            x: q(0, 1),
            y: q(-1, 1),
        },
    ];
    assert!(matches!(
        laws.raw_geometry(&input, true),
        Err(RawLawsFailureV1::Geometry(
            RawGeometryRefusalV1::ConflictingSupportDirections { .. }
        ))
    ));
    input.incoming.source_direction_declarations =
        vec![RawDirectionDeclarationV1::CertifiedPlanar {
            x: q(1, 1),
            y: q(1, 1),
        }];
    assert!(matches!(
        laws.raw_geometry(&input, true),
        Err(RawLawsFailureV1::Geometry(
            RawGeometryRefusalV1::SupportNeitherTangentNorNormal { .. }
        ))
    ));
    input.incoming.source_direction_declarations.clear();
    input.incoming.owner_left = None;
    assert!(matches!(
        laws.raw_geometry(&input, true),
        Err(RawLawsFailureV1::Geometry(
            RawGeometryRefusalV1::AmbiguousOwnerNormal { .. }
        ))
    ));
    input.incoming.chain_use_orientation = Some(ChainUseOrientationV1::BStartToEnd);
    assert!(laws.raw_geometry(&input, true).is_ok());
    input.metric.inverse_gram[0] = q(2, 1);
    assert!(matches!(
        laws.raw_geometry(&input, true),
        Err(RawLawsFailureV1::Contract(
            RawContractFailureV1::InvalidGramFacts
        ))
    ));
}
#[test]
fn count_gates_use_exact_whole_turn_thresholds_q2_to_q6() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let mut laws = RawDensityLawsV1::new(&mut c, limits());
    for qv in 2..=6 {
        for count in 0..qv {
            for sign in [-1, 0, 1] {
                let squared = q(0, 1);
                let steps = count + 1;
                let expected = if steps >= qv {
                    true
                } else if 2 * steps == qv {
                    sign >= 0
                } else if 2 * steps < qv {
                    false
                } else {
                    true
                };
                assert_eq!(
                    laws.turn_count_feasible(sign, &squared, count, qv).unwrap(),
                    expected,
                    "q={qv},count={count},sign={sign}"
                );
                let exact = steps < qv && 2 * steps == qv && sign == 0;
                assert_eq!(
                    laws.turn_count_exact_limit(sign, &squared, count, qv)
                        .unwrap(),
                    exact
                );
            }
        }
    }
    assert!(laws.turn_count_exact_limit(1, &q(1, 4), 0, 3).unwrap());
    assert!(laws.turn_count_exact_limit(-1, &q(1, 4), 1, 3).unwrap());
    assert!(laws.turn_count_exact_limit(1, &q(3, 4), 0, 6).unwrap());
    assert!(laws.turn_count_exact_limit(-1, &q(3, 4), 4, 6).unwrap());
    assert!(matches!(
        laws.turn_count_feasible(1, &q(1, 1), 6, 6),
        Err(RawLawsFailureV1::Contract(
            RawContractFailureV1::InvalidCountProfile
        ))
    ));
}
#[test]
fn numeric_direction_has_three_states_and_does_not_infer_opaque_values() {
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let one = c.rational(&q(1, 1)).unwrap();
    let two = c.rational(&q(2, 1)).unwrap();
    let root = c.sqrt(&two).unwrap();
    let doubled = c.scaled(&root, &q(-2, 1)).unwrap();
    let opaque = c.raw_cos(&one).unwrap();
    let mut laws = RawDensityLawsV1::new(&mut c, limits());
    assert_eq!(
        laws.rational_direction(&[root.clone(), doubled]).unwrap(),
        RationalDirectionV1::Rational(PrimitiveVector(IBig::ONE, IBig::from(-2)))
    );
    assert_eq!(
        laws.rational_direction(&[one.clone(), root]).unwrap(),
        RationalDirectionV1::Irrational
    );
    assert_eq!(
        laws.rational_direction(&[one, opaque]).unwrap(),
        RationalDirectionV1::Undetermined
    );
}
#[test]
fn raw_turn_witness_keeps_owner_orientation_independent_of_dual_dot_sign() {
    for skew in [false, true] {
        for owner in [-1, 1] {
            let mut graph = Expressions::new();
            let mut c = ConstructorV1::new(&mut graph);
            let mut laws = RawDensityLawsV1::new(&mut c, limits());
            let produced = laws.raw_geometry(&raw(skew, owner), true).unwrap();
            let incoming = laws.covector(&produced.metric, &produced.incoming).unwrap();
            let outgoing = laws.covector(&produced.metric, &produced.outgoing).unwrap();
            let witness = laws
                .turn_witness(&produced.metric, &incoming, &outgoing, owner)
                .unwrap();
            assert_eq!(witness.dot_sign, if skew { 1 } else { 0 });
            assert_eq!(witness.cosine_squared, if skew { q(1, 8) } else { q(0, 1) });
            assert!(matches!(
                laws.turn_witness(&produced.metric, &incoming, &outgoing, -owner),
                Err(RawLawsFailureV1::Geometry(
                    RawGeometryRefusalV1::OrderedSupportsDoNotRealizeTurn
                ))
            ));
        }
    }
}
#[test]
#[ignore = "Requires CFTUV_RAW_LAWS_ORACLE_PYTHON, CFTUV_RAW_LAWS_REPLAY, CFTUV_RAW_LAWS_RECEIPT; optional CFTUV_RAW_LAWS_CORPUS"]
fn actual_python_fresh_raw_geometry_value_wires_turn_count_and_named_failures() {
    let python = std::env::var("CFTUV_RAW_LAWS_ORACLE_PYTHON").expect("Python oracle executable");
    let replay = std::env::var("CFTUV_RAW_LAWS_REPLAY").expect("Raw leaf replay executable");
    let out = std::env::var("CFTUV_RAW_LAWS_RECEIPT").expect("External receipt directory");
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let mut command = std::process::Command::new(python);
    command
        .arg(root.join("native/cftuv-density/tests/density_raw_laws_oracle.py"))
        .arg("--root")
        .arg(root)
        .arg("--replay")
        .arg(replay)
        .arg("--out")
        .arg(out);
    if let Ok(corpus) = std::env::var("CFTUV_RAW_LAWS_CORPUS") {
        command.arg("--corpus").arg(corpus);
    }
    if let Ok(oracle) = std::env::var("CFTUV_RAW_LAWS_ORACLE_ROOT") {
        command.arg("--oracle-root").arg(oracle);
    }
    if let Ok(tools) = std::env::var("CFTUV_RAW_LAWS_TOOLS_ROOT") {
        command.arg("--tools-root").arg(tools);
    }
    let result = command.output().expect("test-only Python bridge");
    assert!(
        result.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
}
