use cftuv_density::density_attempt_memo_v1::DensityAttemptMemoSnapshotV1;
use cftuv_density::density_raw_frontend_v1::evaluate_raw_density_batch;
use cftuv_density::density_raw_frontend_v1::evaluate_raw_density_batch_with_memo;
use cftuv_density::density_raw_types_v1::*;
use cftuv_density::{IBig, Rational, UBig};
fn r(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn incident(x: i32, y: i32) -> RawDensityIncidentV1 {
    RawDensityIncidentV1 {
        start: RawDensityPointV1 {
            x: r(0, 1),
            y: r(0, 1),
        },
        end: RawDensityPointV1 {
            x: r(x, 1),
            y: r(y, 1),
        },
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
        physical_edge_id: None,
        chain_use_id: None,
    }
}
fn spec(id: &str) -> RawDensitySpecInputV1 {
    let geometry = RawDensityGeometryV1 {
        metric: RawDensityMetricV1 {
            gram: [r(1, 1), r(0, 1), r(1, 1)],
            inverse_gram: [r(1, 1), r(0, 1), r(1, 1)],
            owner_orientation_sign: 1,
            metric_id: Some("metric".into()),
        },
        incoming: incident(1, 0),
        outgoing: incident(0, 1),
    };
    RawDensitySpecInputV1 {
        source_geometry: geometry.clone(),
        evaluation_geometry: geometry,
        turn_orientation_sign: 1,
        selection: RawDensitySelectionV1 {
            q: 2,
            source_hidden_count: 0,
            selection_policy_id: "HUBER_EMANATED_COUNT_DENSITY_A_V1".into(),
            max_subturn_value_id: "LINEAR_REFLEX_DENSITY_0_V1".into(),
            bucket_c: 1,
            selection_certificate_id: None,
        },
        canonical: RawDensityCanonicalFactsV1 {
            source_reflex_interval: [r(1, 2), r(1, 2)],
            canonical_excess_over_pi: None,
            restoration_declaration: vec![],
            noise_policy_declaration: vec![(
                "binding_present".into(),
                PrimitiveFactV1::Bool(false),
            )],
            source_reflex_declaration: vec![],
        },
        spec_id: Some(id.into()),
        source_relation_id: None,
        owner_sector_id: None,
    }
}
fn batch(specs: Vec<RawDensitySpecInputV1>) -> RawDensityBatchInputV1 {
    RawDensityBatchInputV1 {
        specs,
        canon_ids: DensityOracleCanonV1::candidate(),
        window_law: cftuv_density::geometry::WindowLaw::Voronoi,
        limits: DensityOracleLimitsV1 {
            preparation_cap: UBig::from(4096u32),
            search_cap: UBig::from(4096u32),
            box_cap: 4096,
            height_cap: UBig::from(1000000u32),
            finite_expand: cftuv_density::density_expand_v1::ExpandLimits {
                max_terms: 4096,
                max_polynomial_exponent: 16,
                max_rational_power_bits: 65536,
            },
        },
    }
}
#[test]
fn batch_rejects_missing_duplicate_and_unsorted_ids_before_any_work() {
    let mut missing = spec("a");
    missing.spec_id = None;
    for specs in [
        vec![missing],
        vec![spec("a"), spec("a")],
        vec![spec("b"), spec("a")],
    ] {
        let report = evaluate_raw_density_batch(&batch(specs));
        assert!(report.results.is_empty());
        let failure = report.internal_failure.unwrap();
        assert!(failure.detail.contains("sorted and unique"));
        assert!(failure
            .partial_spec_effects
            .ordered_work_articles
            .is_empty());
    }
}
#[test]
fn raw_rational_batch_accepts_empty_id_and_is_pure_between_invocations() {
    let input = batch(vec![spec("")]);
    let first = evaluate_raw_density_batch(&input);
    let second = evaluate_raw_density_batch(&input);
    assert!(
        first.internal_failure.is_none(),
        "{:?}",
        first.internal_failure
    );
    assert_eq!(first.results.len(), 1);
    assert!(
        first.results[0].failure.is_none(),
        "{:?}",
        first.results[0].failure
    );
    assert_eq!(first.results, second.results);
}
#[test]
fn batch_named_geometry_failure_stops_in_order_and_keeps_prior_result() {
    let mut bad = spec("b");
    bad.source_geometry.incoming.owner_left = None;
    let report = evaluate_raw_density_batch(&batch(vec![spec("a"), bad, spec("c")]));
    assert!(
        report.internal_failure.is_none(),
        "{:?}",
        report.internal_failure
    );
    assert_eq!(report.results.len(), 2);
    let failure = report.results[1].failure.as_ref().unwrap();
    assert_eq!(failure.exception_type, "ReferenceGeometryError");
    assert_eq!(failure.outcome, "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED");
    assert!(report.results[1].named_refusals.is_empty());
}
#[test]
fn successful_window_memo_is_batch_local_and_replays_shells_with_zero_warm_prices() {
    let mut a = spec("a");
    a.selection.q = 5;
    a.selection.source_hidden_count = 1;
    a.selection.bucket_c = 2;
    a.source_geometry.outgoing = incident(1, 2);
    a.evaluation_geometry = a.source_geometry.clone();
    a.canonical.source_reflex_interval = [r(2, 5), r(2, 5)];
    a.selection.max_subturn_value_id = "LINEAR_REFLEX_DENSITY_3_V1".into();
    let mut b = a.clone();
    b.spec_id = Some("b".into());
    let input = batch(vec![a, b]);
    let report = evaluate_raw_density_batch(&input);
    assert!(
        report.internal_failure.is_none(),
        "{:?}",
        report.internal_failure
    );
    assert_eq!(report.results.len(), 2);
    for result in &report.results {
        assert!(result.failure.is_none(), "{:?}", result.failure);
    }
    let cold = &report.results[0];
    let warm = &report.results[1];
    assert!(!cold.ordered_work_articles.is_empty());
    assert_eq!(cold.shell_events, warm.shell_events);
    assert!(
        warm.ordered_work_articles.is_empty(),
        "{:?}",
        warm.ordered_work_articles
    );
    assert_eq!(report.results, evaluate_raw_density_batch(&input).results);
}
#[test]
fn narrow_failure_is_not_memoized_while_successful_voronoi_is() {
    let mut a = spec("a");
    a.selection.q = 5;
    a.selection.source_hidden_count = 1;
    a.selection.bucket_c = 2;
    a.selection.max_subturn_value_id = "LINEAR_REFLEX_DENSITY_3_V1".into();
    a.source_geometry.outgoing = incident(1, 2);
    a.evaluation_geometry = a.source_geometry.clone();
    a.canonical.source_reflex_interval = [r(2, 5), r(2, 5)];
    let mut b = a.clone();
    b.spec_id = Some("b".into());
    let mut input = batch(vec![a, b]);
    input.window_law = cftuv_density::geometry::WindowLaw::NarrowBand;
    input.limits.search_cap = UBig::from(6u8);
    let report = evaluate_raw_density_batch(&input);
    assert!(
        report.internal_failure.is_none(),
        "{:?}",
        report.internal_failure
    );
    assert_eq!(report.results.len(), 2);
    for result in &report.results {
        assert!(result.failure.is_none(), "{:?}", result.failure);
    }
    let cold = &report.results[0];
    let warm = &report.results[1];
    assert!(!cold.named_refusals.is_empty(), "{:?}", cold);
    assert_eq!(cold.named_refusals[0].1, warm.named_refusals[0].1);
    assert!(warm.ordered_work_articles.len() < cold.ordered_work_articles.len());
    assert!(
        warm.ordered_work_articles
            .iter()
            .any(|article| article.counters.iter().any(|(_, v)| v != &UBig::ZERO)),
        "{:?}",
        warm.ordered_work_articles
    );
    assert_eq!(
        warm.ordered_work_articles,
        cold.ordered_work_articles[..warm.ordered_work_articles.len()]
    );
}
#[test]
fn raw_skew_principal_cancellation_is_a_named_certified_predicate_failure() {
    let mut a = spec("a");
    a.selection.q = 5;
    a.selection.source_hidden_count = 3;
    a.selection.bucket_c = 4;
    a.selection.max_subturn_value_id = "LINEAR_REFLEX_DENSITY_3_V1".into();
    a.source_geometry.metric.gram = [r(2, 1), r(1, 2), r(1, 1)];
    a.source_geometry.metric.inverse_gram = [r(4, 7), r(-2, 7), r(8, 7)];
    a.evaluation_geometry = a.source_geometry.clone();
    a.canonical.source_reflex_interval = [r(7, 10), r(7, 10)];
    let report = evaluate_raw_density_batch(&batch(vec![a]));
    assert!(
        report.internal_failure.is_none(),
        "{:?}",
        report.internal_failure
    );
    assert_eq!(report.results.len(), 1);
    let failure = report.results[0].failure.as_ref().unwrap();
    assert_eq!(failure.exception_type, "ReferenceGeometryError");
    assert_eq!(failure.outcome, "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE");
    assert_eq!(
        failure.message,
        "Density A exact sign is not certified without generic factorization"
    );
    assert!(report.results[0].named_refusals.is_empty());
    let prices = &report.results[0].ordered_work_articles;
    assert_eq!(prices.len(), 2);
    for price in prices {
        assert_eq!(price.phase, "PREPARATION");
        assert_eq!(price.cap, UBig::from(4096u32));
        assert_eq!(
            price.counters,
            vec![
                ("DENSITY_FAN_SHELL_PROBES".into(), UBig::from(2u8)),
                ("DENSITY_FAN_ORDER_STEPS".into(), UBig::ZERO)
            ]
        );
    }
    assert!(report.results[0].shell_events.is_empty());
    assert_eq!(report.results[0].canon_usage.len(), 1);
}

#[test]
fn final_named_failure_keeps_attempt_prices_but_drops_band_refusal() {
    let mut a = spec("a");
    a.selection.q = 5;
    a.selection.source_hidden_count = 1;
    a.selection.bucket_c = 2;
    a.selection.max_subturn_value_id = "LINEAR_REFLEX_DENSITY_3_V1".into();
    a.source_geometry.outgoing = incident(1, 2);
    a.evaluation_geometry = a.source_geometry.clone();
    a.canonical.source_reflex_interval = [r(2, 5), r(2, 5)];
    let mut input = batch(vec![a]);
    input.window_law = cftuv_density::geometry::WindowLaw::NarrowBand;
    input.limits.preparation_cap = UBig::ZERO;
    input.limits.search_cap = UBig::ZERO;
    let report = evaluate_raw_density_batch(&input);
    assert!(
        report.internal_failure.is_none(),
        "{:?}",
        report.internal_failure
    );
    let result = &report.results[0];
    assert!(result.failure.is_some());
    assert!(result.named_refusals.is_empty());
    assert!(result.ordered_work_articles.len() >= 2);
    assert!(result
        .ordered_work_articles
        .iter()
        .all(|article| article.cap == UBig::ZERO));
    assert_eq!(
        result.canon_usage[0].applied_event,
        "DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED"
    );
}

#[test]
fn malformed_metric_stays_internal_instead_of_becoming_geometric_undecidable() {
    let mut a = spec("a");
    a.source_geometry.metric.gram = [r(0, 1), r(0, 1), r(0, 1)];
    let report = evaluate_raw_density_batch(&batch(vec![a]));
    assert!(report.results.is_empty());
    assert!(report
        .internal_failure
        .unwrap()
        .detail
        .contains("InvalidGramFacts"));
}

fn memo_spec(id: &str) -> RawDensitySpecInputV1 {
    let mut a = spec(id);
    a.selection.q = 5;
    a.selection.source_hidden_count = 1;
    a.selection.bucket_c = 2;
    a.selection.max_subturn_value_id = "LINEAR_REFLEX_DENSITY_3_V1".into();
    a.source_geometry.outgoing = incident(1, 2);
    a.evaluation_geometry = a.source_geometry.clone();
    a.canonical.source_reflex_interval = [r(2, 5), r(2, 5)];
    a
}

#[test]
fn transaction_memo_survives_fresh_arena_metric_alias_declarations_and_lower_caps() {
    let cold_input = batch(vec![memo_spec("a")]);
    let cold = evaluate_raw_density_batch_with_memo(&cold_input, &Default::default());
    assert!(cold.report.internal_failure.is_none());
    assert!(!cold.delta.entries.is_empty());
    let mut warm_spec = memo_spec("new-spec-id");
    for geometry in [
        &mut warm_spec.source_geometry,
        &mut warm_spec.evaluation_geometry,
    ] {
        geometry.metric.metric_id = Some("equivalent metric alias".into());
    }
    warm_spec.canonical.noise_policy_declaration[0].1 = PrimitiveFactV1::Bool(true);
    let mut warm_input = batch(vec![warm_spec]);
    warm_input.limits.preparation_cap = UBig::ZERO;
    warm_input.limits.search_cap = UBig::ZERO;
    warm_input.limits.height_cap = UBig::ZERO;
    warm_input.limits.box_cap = 0;
    let warm = evaluate_raw_density_batch_with_memo(&warm_input, &cold.delta);
    assert!(warm.report.internal_failure.is_none(), "{:?}", warm.report);
    assert!(
        warm.report.results[0].failure.is_none(),
        "{:?}",
        warm.report
    );
    assert!(warm.report.results[0].ordered_work_articles.is_empty());
    assert_eq!(
        warm.report.results[0].shell_events,
        cold.report.results[0].shell_events
    );
    assert_eq!(
        warm.report.results[0].canon_usage,
        cold.report.results[0].canon_usage
    );
    assert!(warm.delta.entries.is_empty());
    let mut expected = cold.report.results[0].clone();
    expected.ordered_work_articles.clear();
    // binding_present влияет на formatter reason, но не на identity successful attempt.
    for (certificate, reason) in expected
        .binding_certificates
        .iter_mut()
        .zip(&expected.binding_reasons)
    {
        if let Some(certificate) = certificate {
            certificate.binding_reason = reason.clone();
        }
    }
    assert_eq!(warm.report.results[0], expected);
}

#[test]
fn failed_narrow_attempt_is_repriced_across_batches_and_never_added_to_delta() {
    let mut input = batch(vec![memo_spec("a")]);
    input.window_law = cftuv_density::geometry::WindowLaw::NarrowBand;
    input.limits.search_cap = UBig::from(6u8);
    let cold = evaluate_raw_density_batch_with_memo(&input, &Default::default());
    assert!(cold.report.internal_failure.is_none());
    assert!(cold
        .delta
        .entries
        .iter()
        .all(|r| r.key.window_law == cftuv_density::geometry::WindowLaw::Voronoi));
    assert!(!cold.delta.entries.is_empty());
    let warm = evaluate_raw_density_batch_with_memo(&input, &cold.delta);
    let cold = &cold.report.results[0];
    let warm_result = &warm.report.results[0];
    assert!(!warm_result.ordered_work_articles.is_empty());
    assert!(warm_result.ordered_work_articles.len() < cold.ordered_work_articles.len());
    assert_eq!(
        warm_result.ordered_work_articles,
        cold.ordered_work_articles[..warm_result.ordered_work_articles.len()]
    );
    assert_eq!(warm_result.named_refusals, cold.named_refusals);
    assert_eq!(warm_result.shell_events, cold.shell_events);
    assert!(warm.delta.entries.is_empty());
}

#[test]
fn success_delta_survives_later_named_and_internal_failure_in_order() {
    let successful =
        evaluate_raw_density_batch_with_memo(&batch(vec![memo_spec("a")]), &Default::default());
    for internal in [false, true] {
        let mut bad = spec("b");
        if internal {
            bad.source_geometry.metric.gram = [r(0, 1), r(0, 1), r(0, 1)];
        } else {
            bad.source_geometry.incoming.owner_left = None;
        }
        let result = evaluate_raw_density_batch_with_memo(
            &batch(vec![memo_spec("a"), bad, memo_spec("c")]),
            &Default::default(),
        );
        assert_eq!(result.delta, successful.delta);
        assert_eq!(result.report.results[0], successful.report.results[0]);
        assert_eq!(result.report.internal_failure.is_some(), internal);
        assert_eq!(result.report.results.len(), if internal { 1 } else { 2 });
    }
}

#[test]
fn snapshots_reject_duplicate_keys_and_canon_mismatch_before_work_and_empty_has_no_delta() {
    let input = batch(vec![memo_spec("a")]);
    let success = evaluate_raw_density_batch_with_memo(&input, &Default::default());
    let receipt = success.delta.entries[0].clone();
    let mut conflicting = receipt.clone();
    conflicting.binding_certificates.clear();
    let mut wrong_canon = receipt.clone();
    wrong_canon.key.canon_ids.angle.push_str("_other");
    for entries in [
        vec![receipt.clone(), receipt],
        vec![conflicting.clone(), success.delta.entries[0].clone()],
        vec![wrong_canon],
    ] {
        let result =
            evaluate_raw_density_batch_with_memo(&input, &DensityAttemptMemoSnapshotV1 { entries });
        assert!(result.report.results.is_empty());
        assert!(result.report.internal_failure.is_some());
        assert!(result.delta.entries.is_empty());
        assert!(result
            .report
            .internal_failure
            .unwrap()
            .partial_spec_effects
            .ordered_work_articles
            .is_empty());
    }
    let result = evaluate_raw_density_batch_with_memo(&batch(vec![]), &success.delta);
    assert!(result.report.internal_failure.is_none());
    assert!(result.report.results.is_empty());
    assert!(result.delta.entries.is_empty());
}

#[test]
fn current_height_counts_are_not_observable_in_memo_projection() {
    let mut input = batch(vec![memo_spec("a")]);
    // Малый B(w) cap отдаёт решение adaptive authority.
    input.limits.preparation_cap = UBig::from(4u8);
    let mut result = evaluate_raw_density_batch_with_memo(&input, &Default::default());
    // Выбираем настоящий adaptive receipt; никакой authority из тестовых данных.
    let receipt = result
        .delta
        .entries
        .iter_mut()
        .find(|r| r.authority.is_some())
        .expect("adaptive receipt");
    let before = cftuv_density::density_result_v1::binding_result_facts(
        &receipt.binding_result(),
        &receipt.key.binding_reasons,
        receipt.key.window_law,
    )
    .unwrap();
    receipt.authority.as_mut().unwrap().counts.clear();
    let after = cftuv_density::density_result_v1::binding_result_facts(
        &receipt.binding_result(),
        &receipt.key.binding_reasons,
        receipt.key.window_law,
    )
    .unwrap();
    assert_eq!(before, after);
}

#[test]
fn forty_five_degree_alias_matches_legacy_cold_three_warm_two_articles() {
    let mut a = memo_spec("a");
    a.selection.q = 4;
    a.selection.bucket_c = 1;
    a.selection.max_subturn_value_id = "LINEAR_REFLEX_DENSITY_2_V1".into();
    a.source_geometry.incoming = incident(0, 0);
    a.source_geometry.incoming.start.y = r(1, 1);
    a.source_geometry.outgoing = incident(1, -1);
    a.evaluation_geometry = a.source_geometry.clone();
    a.canonical.source_reflex_interval = [r(1, 4), r(1, 4)];
    let mut input = batch(vec![a.clone()]);
    input.window_law = cftuv_density::geometry::WindowLaw::NarrowBand;
    input.limits.preparation_cap = UBig::from(65536u32);
    input.limits.search_cap = UBig::ONE;
    input.limits.box_cap = 96;
    let cold = evaluate_raw_density_batch_with_memo(&input, &Default::default());
    assert!(cold.report.internal_failure.is_none(), "{:?}", cold.report);
    assert!(
        cold.report.results[0].failure.is_none(),
        "{:?}",
        cold.report
    );
    assert_eq!(cold.report.results[0].ordered_work_articles.len(), 3);
    assert_eq!(cold.delta.entries.len(), 1);
    let mut alias = a.clone();
    alias.spec_id = Some("b".into());
    alias.source_geometry.metric.metric_id = Some("different label".into());
    alias.evaluation_geometry.metric.metric_id = Some("different label".into());
    let mut aliased = input.clone();
    aliased.specs = vec![alias.clone()];
    let warm = evaluate_raw_density_batch_with_memo(&aliased, &cold.delta);
    assert!(warm.report.internal_failure.is_none());
    assert_eq!(
        warm.report.results[0].ordered_work_articles,
        cold.report.results[0].ordered_work_articles[..2]
    );
    assert_eq!(
        warm.report.results[0].shell_events,
        cold.report.results[0].shell_events
    );
    assert_eq!(
        warm.report.results[0].binding_certificates,
        cold.report.results[0].binding_certificates
    );
    assert!(warm.delta.entries.is_empty());
    // И прежний stateless API должен объединять equivalent metrics внутри batch.
    aliased.specs = vec![a, alias];
    let stateless = evaluate_raw_density_batch(&aliased);
    assert_eq!(
        stateless.results[1].ordered_work_articles,
        warm.report.results[0].ordered_work_articles
    );
    assert_eq!(
        stateless.results[1].binding_certificates,
        warm.report.results[0].binding_certificates
    );
}

#[test]
fn delta_order_is_first_success_order_and_never_hashmap_iteration() {
    let first = memo_spec("a");
    let mut second = memo_spec("b");
    second.source_geometry.outgoing = incident(1, 3);
    second.evaluation_geometry = second.source_geometry.clone();
    let a = evaluate_raw_density_batch_with_memo(&batch(vec![first.clone()]), &Default::default());
    let b = evaluate_raw_density_batch_with_memo(&batch(vec![second.clone()]), &Default::default());
    assert!(!a.delta.entries.is_empty());
    assert!(!b.delta.entries.is_empty());
    let combined =
        evaluate_raw_density_batch_with_memo(&batch(vec![first, second]), &Default::default());
    let mut expected = a.delta;
    expected.entries.extend(b.delta.entries);
    assert_eq!(combined.delta, expected);
}
