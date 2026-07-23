from __future__ import annotations

import pytest

from cftuv_envelope import (
    DebugDiagnosticId,
    DebugPrimitiveId,
    EnvelopeDebugSceneCodecV1,
    build_envelope_debug_scene,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
    resolve_coverage_interactions,
    validate_envelope_debug_scene,
)

from reference_factories import angular_snapshot, straight_snapshot


def _scene():
    snapshot, request = straight_snapshot(
        faces=(((0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)),),
        source_routes=(
            {
                "name": "bottom",
                "points": ((0.0, 0.0), (6.0, 0.0)),
            },
        ),
        alpha="1",
        revision_name="debug-scene",
    )
    compile_result = compile_reference_envelopes(snapshot, request)
    assert compile_result.compilation is not None
    compilation = compile_result.compilation
    raw_result = evaluate_reference_raw_coverage(
        compilation, request.requested_alpha
    )
    assert raw_result.raw_coverage is not None
    raw = raw_result.raw_coverage
    interaction = resolve_coverage_interactions(
        compilation, raw.boundary_resolved_envelopes, raw
    )
    assert interaction.resolved_coverage is not None
    return build_envelope_debug_scene(
        snapshot,
        request,
        (compilation,),
        (raw,),
        (interaction,),
    )


def test_debug_scene_projects_ready_exact_results_and_roundtrips():
    scene = _scene()
    assert validate_envelope_debug_scene(scene) == ()
    assert scene.raw_coverage_digests
    assert scene.resolved_coverage_digests
    assert any(item.style_key == "ENV_20_SOURCE_SUPPORTS" for item in scene.paths)
    assert any(item.style_key == "ENV_30_ENVELOPE_INSTANCES" for item in scene.regions)
    assert any(item.style_key == "ENV_40_RAW_COVERAGE" for item in scene.loops)
    assert any(item.style_key == "ENV_60_RESOLVED_COVERAGE" for item in scene.loops)
    assert all(
        point.x_expression and point.y_expression
        for path in scene.paths
        for point in path.exact_points
    )
    payload = EnvelopeDebugSceneCodecV1.dumps(scene)
    assert EnvelopeDebugSceneCodecV1.loads(payload) == scene


def test_debug_identity_types_are_nominally_distinct():
    assert DebugPrimitiveId("shared") != DebugDiagnosticId("shared")


@pytest.mark.parametrize("hidden_count", (0, 1, 2))
def test_debug_scene_exposes_k0_k1_k2_angular_profiles(hidden_count):
    snapshot, request = angular_snapshot(hidden_count)
    compile_result = compile_reference_envelopes(snapshot, request)
    assert compile_result.compilation is not None
    compilation = compile_result.compilation
    angular_spec = next(
        item
        for item in compilation.envelope_specs
        if type(item).__name__ == "AngularEnvelopeSpec"
    )
    assert angular_spec.resolved_hidden_edge_count == hidden_count
    raw_result = evaluate_reference_raw_coverage(
        compilation, request.requested_alpha
    )
    assert raw_result.raw_coverage is not None
    raw = raw_result.raw_coverage
    interaction = resolve_coverage_interactions(
        compilation, raw.boundary_resolved_envelopes, raw
    )
    assert interaction.resolved_coverage is not None
    scene = build_envelope_debug_scene(
        snapshot, request, (compilation,), (raw,), (interaction,)
    )
    hidden_paths = [
        item for item in scene.paths if item.style_key == "ENV_22_HIDDEN_SUPPORTS"
    ]
    assert bool(hidden_paths) is (hidden_count > 0)
    assert any(
        item.style_key == "ENV_30_ENVELOPE_INSTANCES" for item in scene.regions
    )
