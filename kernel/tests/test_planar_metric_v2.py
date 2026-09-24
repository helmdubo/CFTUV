from __future__ import annotations

from dataclasses import replace

import pytest

from cftuv_envelope import (
    AffineChartOrientationV1,
    LineageId,
    LocalPoint3V1,
    NamedOutcome,
    PhysicalEdgeId,
    PlanarMetricAdmissionError,
    RationalAffinePlanarMetricCodecV2,
    RuntimePlanarMetricCodecV1,
    RuntimePredicateResultV1,
    RuntimePredicateTelemetryV1,
    SurfaceTriangleId,
    SurfaceTriangleV1,
    SourceRevision,
    SourceVertexId,
    SourceVertexV1,
    build_rational_affine_planar_metric,
    build_runtime_planar_metric,
    canonical_json_bytes,
    compile_reference_envelopes,
    evaluate_filtered_runtime_raw_coverage,
    evaluate_reference_raw_coverage,
    resolve_orient2d,
    validate_rational_affine_planar_metric,
    validate_runtime_planar_metric,
)

from factories import full_host_snapshot
from reference_factories import straight_snapshot


def _metric(snapshot=None):
    snapshot = snapshot or full_host_snapshot()
    domain = next(iter(snapshot.patch_domains))
    return build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
        source_lineage=frozenset({LineageId("metric-source")}),
    )


def _coordinate_projection(metric):
    return tuple(
        sorted(
            (
                item.source_vertex_id.value,
                item.domain_coordinate.x.numerator,
                item.domain_coordinate.x.denominator,
                item.domain_coordinate.y.numerator,
                item.domain_coordinate.y.denominator,
            )
            for item in metric.exact_source_vertex_coordinates
        )
    )


def test_public_metric_contracts_codec_and_validation_round_trip():
    reference = _metric()
    runtime = build_runtime_planar_metric(reference)

    assert validate_rational_affine_planar_metric(reference) == ()
    assert validate_runtime_planar_metric(runtime) == ()
    assert (
        RationalAffinePlanarMetricCodecV2.loads(
            RationalAffinePlanarMetricCodecV2.dumps(reference)
        )
        == reference
    )
    assert (
        RuntimePlanarMetricCodecV1.loads(
            RuntimePlanarMetricCodecV1.dumps(runtime)
        )
        == runtime
    )
    assert runtime.reference_metric_id == reference.reference_metric_id
    assert (
        runtime.fallback_contract.authoritative_reference_metric_id
        == reference.reference_metric_id
    )


def test_affine_frame_is_deterministic_under_record_order_and_loop_shift():
    snapshot = full_host_snapshot()
    first = _metric(snapshot)
    face = next(iter(snapshot.surface_ir.source_faces))
    shifted_face = replace(
        face,
        vertex_cycle=face.vertex_cycle[1:] + face.vertex_cycle[:1],
        edge_cycle=face.edge_cycle[1:] + face.edge_cycle[:1],
    )
    shifted = replace(
        snapshot,
        source_vertices=frozenset(
            reversed(
                sorted(
                    snapshot.source_vertices,
                    key=lambda item: item.vertex_id.value,
                )
            )
        ),
        surface_ir=replace(
            snapshot.surface_ir,
            source_faces=frozenset({shifted_face}),
        ),
    )
    second = _metric(shifted)

    assert first == second
    assert canonical_json_bytes(first) == canonical_json_bytes(second)


def test_rigid_rotation_preserves_affine_coordinates_and_gram():
    snapshot = full_host_snapshot()
    reference = _metric(snapshot)
    rotated_vertices = frozenset(
        SourceVertexV1(
            item.vertex_id,
            LocalPoint3V1(
                -item.position.y,
                item.position.x,
                item.position.z,
            ),
        )
        for item in snapshot.source_vertices
    )
    rotated = _metric(
        replace(
            snapshot,
            source_revision=SourceRevision("rotated"),
            source_vertices=rotated_vertices,
            surface_ir=replace(
                snapshot.surface_ir,
                source_revision=SourceRevision("rotated"),
            ),
        )
    )

    assert _coordinate_projection(reference) == _coordinate_projection(rotated)
    assert reference.exact_gram_matrix == rotated.exact_gram_matrix
    assert reference.chart_orientation == rotated.chart_orientation


def test_uniform_scale_preserves_coordinates_and_scales_gram():
    snapshot = full_host_snapshot()
    reference = _metric(snapshot)
    scaled = _metric(
        replace(
            snapshot,
            source_revision=SourceRevision("scaled"),
            source_vertices=frozenset(
                SourceVertexV1(
                    item.vertex_id,
                    LocalPoint3V1(
                        item.position.x * 8.0,
                        item.position.y * 8.0,
                        item.position.z * 8.0,
                    ),
                )
                for item in snapshot.source_vertices
            ),
            surface_ir=replace(
                snapshot.surface_ir,
                source_revision=SourceRevision("scaled"),
            ),
        )
    )

    assert _coordinate_projection(reference) == _coordinate_projection(scaled)
    assert (
        scaled.exact_gram_matrix.m00.numerator
        == 64 * reference.exact_gram_matrix.m00.numerator
    )
    assert (
        scaled.exact_gram_matrix.m11.numerator
        == 64 * reference.exact_gram_matrix.m11.numerator
    )


def test_rigid_rotation_preserves_full_exact_semantic_result():
    snapshot, request = straight_snapshot(
        faces=(((0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)),),
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (6.0, 0.0)),
            },
        ),
        alpha="0.25",
        revision_name="rigid-semantic",
    )
    domain = next(iter(snapshot.patch_domains))
    baseline_metric = build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
    )
    baseline_snapshot = replace(
        snapshot,
        surface_metric_descriptors=frozenset({baseline_metric}),
    )
    rotated_vertices = frozenset(
        SourceVertexV1(
            item.vertex_id,
            LocalPoint3V1(
                -item.position.y,
                item.position.x,
                item.position.z,
            ),
        )
        for item in snapshot.source_vertices
    )
    rotated_metric = build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=rotated_vertices,
        source_faces=snapshot.surface_ir.source_faces,
    )
    rotated_snapshot = replace(
        snapshot,
        source_vertices=rotated_vertices,
        surface_metric_descriptors=frozenset({rotated_metric}),
    )

    baseline = evaluate_reference_raw_coverage(
        compile_reference_envelopes(
            baseline_snapshot, request
        ).compilation,
        request.requested_alpha,
    )
    rotated = evaluate_reference_raw_coverage(
        compile_reference_envelopes(
            rotated_snapshot, request
        ).compilation,
        request.requested_alpha,
    )

    assert baseline_metric.exact_gram_matrix == rotated_metric.exact_gram_matrix
    assert _coordinate_projection(baseline_metric) == _coordinate_projection(
        rotated_metric
    )
    assert rotated == baseline


def test_surface_retriangulation_preserves_v2_metric_and_raw_digest():
    snapshot, request = straight_snapshot(
        faces=(((0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)),),
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (6.0, 0.0)),
            },
        ),
        alpha="0.25",
        revision_name="retriangulated-semantic",
    )
    face = next(iter(snapshot.surface_ir.source_faces))
    v0, v1, v2, v3 = face.vertex_cycle
    e0, e1, e2, e3 = face.edge_cycle
    alternate = frozenset(
        {
            SurfaceTriangleV1(
                SurfaceTriangleId("metric-alternate-a"),
                face.face_id,
                (v0, v1, v3),
                (e0, None, e3),
                face.polygon_normal,
            ),
            SurfaceTriangleV1(
                SurfaceTriangleId("metric-alternate-b"),
                face.face_id,
                (v1, v2, v3),
                (e1, e2, None),
                face.polygon_normal,
            ),
        }
    )
    changed_face = replace(
        face, triangle_ids=tuple(item.triangle_id for item in alternate)
    )
    changed_snapshot = replace(
        snapshot,
        surface_ir=replace(
            snapshot.surface_ir,
            source_faces=frozenset({changed_face}),
            surface_triangles=alternate,
        ),
    )
    domain = next(iter(snapshot.patch_domains))
    baseline_metric = build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
    )
    changed_metric = build_rational_affine_planar_metric(
        source_revision=changed_snapshot.source_revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=changed_snapshot.source_vertices,
        source_faces=changed_snapshot.surface_ir.source_faces,
    )
    baseline_snapshot = replace(
        snapshot,
        surface_metric_descriptors=frozenset({baseline_metric}),
    )
    changed_snapshot = replace(
        changed_snapshot,
        surface_metric_descriptors=frozenset({changed_metric}),
    )
    baseline = evaluate_reference_raw_coverage(
        compile_reference_envelopes(
            baseline_snapshot, request
        ).compilation,
        request.requested_alpha,
    )
    changed = evaluate_reference_raw_coverage(
        compile_reference_envelopes(
            changed_snapshot, request
        ).compilation,
        request.requested_alpha,
    )

    assert changed_metric == baseline_metric
    assert changed.raw_coverage.semantic_digest == (
        baseline.raw_coverage.semantic_digest
    )


def test_exact_source_plane_rejects_near_planar_without_budget():
    snapshot = full_host_snapshot()
    source_vertices = set(snapshot.source_vertices)
    extra_id = SourceVertexId("v3")
    source_vertices.add(
        SourceVertexV1(extra_id, LocalPoint3V1(1.0, 1.0, 0.001))
    )
    face = next(iter(snapshot.surface_ir.source_faces))
    extended_face = replace(
        face,
        vertex_cycle=(*face.vertex_cycle, extra_id),
        edge_cycle=(*face.edge_cycle, PhysicalEdgeId("e3")),
    )

    with pytest.raises(PlanarMetricAdmissionError) as caught:
        _metric(
            replace(
                snapshot,
                source_vertices=frozenset(source_vertices),
                surface_ir=replace(
                    snapshot.surface_ir,
                    source_faces=frozenset({extended_face}),
                ),
            )
        )

    assert (
        caught.value.outcome
        is NamedOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED
    )


def test_runtime_filter_falls_back_for_zero_and_keeps_exact_identity():
    reference = _metric()
    runtime = build_runtime_planar_metric(reference)
    ids = tuple(
        sorted(
            (
                item.source_vertex_id
                for item in reference.exact_source_vertex_coordinates
            ),
            key=lambda item: item.value,
        )
    )
    telemetry = RuntimePredicateTelemetryV1()
    positive = resolve_orient2d(
        reference_metric=reference,
        runtime_metric=runtime,
        source_vertex_ids=ids,
        telemetry=telemetry,
    )
    zero = resolve_orient2d(
        reference_metric=reference,
        runtime_metric=runtime,
        source_vertex_ids=(ids[0], ids[1], ids[1]),
        telemetry=telemetry,
    )

    assert positive.result is RuntimePredicateResultV1.CERTIFIED_POSITIVE
    assert positive.semantic_sign == 1
    assert zero.result is RuntimePredicateResultV1.CERTIFIED_ZERO
    assert zero.semantic_sign == 0
    assert zero.exact_fallback_used is True
    assert telemetry.filtered_predicate_count == 2
    assert telemetry.fast_path_certified_count == 1
    assert telemetry.exact_fallback_count == 1
    assert telemetry.fallback_fraction == "1/2"


def test_chart_orientation_records_semantic_face_reverse():
    snapshot = full_host_snapshot()
    face = next(iter(snapshot.surface_ir.source_faces))
    reversed_snapshot = replace(
        snapshot,
        surface_ir=replace(
            snapshot.surface_ir,
            source_faces=frozenset(
                {
                    replace(
                        face,
                        vertex_cycle=tuple(reversed(face.vertex_cycle)),
                        edge_cycle=tuple(reversed(face.edge_cycle)),
                    )
                }
            ),
        ),
    )
    metric = _metric(reversed_snapshot)

    assert (
        metric.chart_orientation
        is AffineChartOrientationV1.COORDINATE_CW_MATCHES_OWNER_PATCH
    )


def test_filtered_runtime_emits_the_authoritative_exact_raw_coverage():
    snapshot, request = straight_snapshot(
        faces=(((0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)),),
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (6.0, 0.0)),
            },
        ),
        alpha="1",
        revision_name="runtime-differential",
    )
    domain = next(iter(snapshot.patch_domains))
    reference_metric = build_rational_affine_planar_metric(
        source_revision=snapshot.source_revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=snapshot.source_vertices,
        source_faces=snapshot.surface_ir.source_faces,
    )
    snapshot = replace(
        snapshot,
        surface_metric_descriptors=frozenset({reference_metric}),
    )
    compilation = compile_reference_envelopes(
        snapshot, request
    ).compilation
    exact = evaluate_reference_raw_coverage(
        compilation, request.requested_alpha
    )
    runtime = evaluate_filtered_runtime_raw_coverage(
        compilation, request.requested_alpha
    )

    assert runtime.raw_result == exact
    assert runtime.raw_result.raw_coverage.semantic_digest == (
        exact.raw_coverage.semantic_digest
    )
    assert runtime.performance.filtered_predicate_count == 2
    assert (
        runtime.performance.fast_path_certified_count
        + runtime.performance.exact_fallback_count
        == runtime.performance.filtered_predicate_count
    )
    # По умолчанию память не мерится: tracemalloc стоит 3.75x на этом коде.
    # Ноль обязан означать «не мерили», а не «нисколько» — поэтому проверяются
    # оба режима, иначе выключение замера прошло бы незамеченным.
    assert runtime.performance.peak_memory_bytes == 0
    measured = evaluate_filtered_runtime_raw_coverage(
        compilation, request.requested_alpha, measure_peak_memory=True
    )
    assert measured.performance.peak_memory_bytes > 0


@pytest.mark.parametrize(
    ("fixture_name", "transform"),
    (
        ("axis_aligned", lambda x, y, z: (x, y, z)),
        (
            "arbitrary_rotated_plane",
            lambda x, y, z: (x, y, x + 2.0 * y),
        ),
        (
            "forty_five_degrees",
            lambda x, y, z: (
                (2.0**-0.5) * (x - y),
                (2.0**-0.5) * (x + y),
                z,
            ),
        ),
        (
            "irrational_unit_normal",
            lambda x, y, z: (
                x,
                (2.0**-0.5) * y,
                (2.0**-0.5) * y,
            ),
        ),
        (
            "micro_scale",
            lambda x, y, z: (
                x * 2.0**-30,
                y * 2.0**-30,
                z * 2.0**-30,
            ),
        ),
        (
            "huge_coordinates",
            lambda x, y, z: (
                2.0**40 + x * 2.0**20,
                -(2.0**40) + y * 2.0**20,
                2.0**39 + z,
            ),
        ),
        (
            "same_plane_after_rotation",
            lambda x, y, z: (-y, z, x),
        ),
    ),
)
def test_blocking_metric_fixtures_runtime_equal_reference(
    fixture_name, transform
):
    snapshot, request = straight_snapshot(
        faces=(((0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)),),
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (6.0, 0.0)),
            },
        ),
        alpha="0.25",
        revision_name=fixture_name,
    )
    revision = SourceRevision(f"runtime-corpus:{fixture_name}")
    transformed = frozenset(
        SourceVertexV1(
            item.vertex_id,
            LocalPoint3V1(
                *transform(
                    item.position.x,
                    item.position.y,
                    item.position.z,
                )
            ),
        )
        for item in snapshot.source_vertices
    )
    snapshot = replace(
        snapshot,
        source_revision=revision,
        source_vertices=transformed,
        surface_ir=replace(snapshot.surface_ir, source_revision=revision),
    )
    domain = next(iter(snapshot.patch_domains))
    metric = build_rational_affine_planar_metric(
        source_revision=revision,
        patch_domain_id=domain.patch_domain_id,
        owner_patch_id=domain.owner_patch_id,
        source_vertices=transformed,
        source_faces=snapshot.surface_ir.source_faces,
    )
    snapshot = replace(
        snapshot,
        surface_metric_descriptors=frozenset({metric}),
    )
    compilation = compile_reference_envelopes(
        snapshot, request
    ).compilation
    reference = evaluate_reference_raw_coverage(
        compilation, request.requested_alpha
    )
    runtime = evaluate_filtered_runtime_raw_coverage(
        compilation, request.requested_alpha
    )

    assert runtime.raw_result == reference
    assert runtime.raw_result.raw_coverage.semantic_digest == (
        reference.raw_coverage.semantic_digest
    )
