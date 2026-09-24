from __future__ import annotations

import json
from dataclasses import replace

import cftuv_envelope as kernel


def _rational(numerator: int, denominator: int = 1):
    return kernel.ExactRationalV1(numerator, denominator)


def _binding() -> kernel.EvaluationGeometryBindingV1:
    return kernel.EvaluationGeometryBindingV1(
        schema_version=kernel.EVALUATION_GEOMETRY_BINDING_SCHEMA_V1,
        source_revision=kernel.SourceRevision("test:revision"),
        patch_domain_id=kernel.PatchDomainId("test:domain"),
        reference_metric_id=kernel.ReferenceMetricId("test:metric"),
        binding_law=(
            kernel.EvaluationGeometryBindingLawV1.EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1
        ),
        lattice_scale=2,
        source_vertex_coordinates=frozenset(
            {
                kernel.EvaluationGeometrySourceVertexV1(
                    kernel.SourceVertexId("test:v0"),
                    kernel.ExactPoint2V1(_rational(0), _rational(0)),
                ),
                kernel.EvaluationGeometrySourceVertexV1(
                    kernel.SourceVertexId("test:v1"),
                    kernel.ExactPoint2V1(_rational(1, 2), _rational(-3, 2)),
                ),
            }
        ),
        bound_hidden_support_ids=frozenset(),
    )


def _chain_straight_binding() -> kernel.ChainStraightEvaluationGeometryBindingV2:
    chain_id = kernel.PhysicalChainId("test:straight-chain")
    vertex_ids = tuple(kernel.SourceVertexId(f"test:v{index}") for index in range(3))
    source_points = (
        kernel.ExactPoint2V1(_rational(0), _rational(0)),
        kernel.ExactPoint2V1(_rational(1, 2), _rational(0)),
        kernel.ExactPoint2V1(_rational(1), _rational(0)),
    )
    nodes = ((0, 0), (1, 0), (2, 0))
    authorities = (
        kernel.ChainStraightVertexAuthorityV2.BASE_BOUND_ENDPOINT_V1,
        kernel.ChainStraightVertexAuthorityV2.CHAIN_STRAIGHT_INTERNAL_REFINED_V2,
        kernel.ChainStraightVertexAuthorityV2.BASE_BOUND_ENDPOINT_V1,
    )
    zero_vector = kernel.ExactVector2V1(_rational(0), _rational(0))
    assignment = kernel.ChainStraightInternalAssignmentV2(
        physical_chain_id=chain_id,
        ordinal=1,
        source_vertex_id=vertex_ids[1],
        lower_k=1,
        upper_k=1,
        projection_k_gram=_rational(1),
        unconstrained_canonical_k=1,
        selected_k=1,
        clamped=False,
        disposition=(
            kernel.ChainStraightAssignmentDispositionV2.UNCLAMPED_WITHIN_HALF_STEP
        ),
        assigned_refined_node=nodes[1],
        exact_offset_from_source=zero_vector,
        exact_gram_displacement_squared=_rational(0),
        half_step_gram_squared_bound=_rational(1, 16),
        exact_longitudinal_displacement_k=_rational(0),
        longitudinal_half_step_bound_k=_rational(1, 2),
    )
    return kernel.ChainStraightEvaluationGeometryBindingV2(
        schema_version=(
            kernel.CHAIN_STRAIGHT_EVALUATION_GEOMETRY_BINDING_SCHEMA_V2
        ),
        source_revision=kernel.SourceRevision("test:revision"),
        patch_domain_id=kernel.PatchDomainId("test:domain"),
        reference_metric_id=kernel.ReferenceMetricId("test:metric"),
        binding_law=(
            kernel.ChainStraightEvaluationGeometryBindingLawV2.EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_CHAIN_STRAIGHT_V2
        ),
        base_lattice_scale=2,
        refinement_power=0,
        lattice_scale=2,
        source_vertex_coordinates=frozenset(
            kernel.EvaluationGeometrySourceVertexV1(vertex_id, point)
            for vertex_id, point in zip(vertex_ids, source_points, strict=True)
        ),
        vertex_authorities=frozenset(
            kernel.ChainStraightVertexAuthorityRecordV2(
                source_vertex_id=vertex_id,
                authority=authority,
                physical_chain_ids=frozenset({chain_id}),
                source_domain_coordinate=point,
                base_bound_node=node,
                base_bound_coordinate=point,
                assigned_refined_node=node,
                assigned_domain_coordinate=point,
                exact_offset_from_source=zero_vector,
            )
            for vertex_id, authority, point, node in zip(
                vertex_ids, authorities, source_points, nodes, strict=True
            )
        ),
        straight_chain_bindings=frozenset(
            {
                kernel.ChainStraightPhysicalChainBindingV2(
                    physical_chain_id=chain_id,
                    ordered_source_vertex_ids=vertex_ids,
                    primitive_direction=(1, 0),
                    base_start_node=nodes[0],
                    base_end_node=nodes[-1],
                    base_endpoint_span_k=2,
                    refined_endpoint_span_k=2,
                    internal_assignments=(assignment,),
                )
            }
        ),
        previous_refinement_capacity_deficits=frozenset(),
        bound_hidden_support_ids=frozenset(),
    )


def test_binding_codec_schema_and_validator_are_symmetric():
    binding = _binding()
    payload = kernel.EvaluationGeometryBindingCodecV1.dumps(binding)

    assert kernel.EvaluationGeometryBindingCodecV1.loads(payload) == binding
    assert kernel.validate_evaluation_geometry_binding(binding) == ()
    assert json.loads(payload)["binding_law"] == (
        "EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1"
    )


def test_chain_straight_v2_codec_schema_and_validator_are_symmetric():
    binding = _chain_straight_binding()
    payload = kernel.ChainStraightEvaluationGeometryBindingCodecV2.dumps(binding)

    assert (
        kernel.ChainStraightEvaluationGeometryBindingCodecV2.loads(payload)
        == binding
    )
    assert kernel.validate_chain_straight_evaluation_geometry_binding(binding) == ()
    assert json.loads(payload)["binding_law"] == (
        "EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_CHAIN_STRAIGHT_V2"
    )


def test_chain_straight_v2_codec_rejects_forged_and_unknown_root_tags():
    v1_payload = kernel.EvaluationGeometryBindingCodecV1.dumps(_binding())
    try:
        kernel.ChainStraightEvaluationGeometryBindingCodecV2.loads(v1_payload)
    except kernel.ContractCodecError as exc:
        assert "expected record ChainStraightEvaluationGeometryBindingV2" in str(exc)
    else:  # pragma: no cover - failure is the contract.
        raise AssertionError("forged V1 root tag was accepted as V2")

    data = json.loads(
        kernel.ChainStraightEvaluationGeometryBindingCodecV2.dumps(
            _chain_straight_binding()
        )
    )
    data["$type"] = "FutureChainStraightEvaluationGeometryBindingV9"
    try:
        kernel.ChainStraightEvaluationGeometryBindingCodecV2.loads(json.dumps(data))
    except kernel.ContractCodecError as exc:
        assert "FutureChainStraightEvaluationGeometryBindingV9" in str(exc)
    else:  # pragma: no cover - failure is the contract.
        raise AssertionError("unknown V2 root tag was accepted")


def test_chain_straight_v2_validator_rejects_inconsistent_clamp_disposition():
    binding = _chain_straight_binding()
    (chain,) = binding.straight_chain_bindings
    (assignment,) = chain.internal_assignments
    malformed_assignment = replace(
        assignment,
        disposition=(
            kernel.ChainStraightAssignmentDispositionV2.CLAMPED_CONSTRAINT_EXCESS_ALLOWED
        ),
    )
    malformed = replace(
        binding,
        straight_chain_bindings=frozenset(
            {
                replace(
                    chain,
                    internal_assignments=(malformed_assignment,),
                )
            }
        ),
    )

    assert {
        issue.code
        for issue in kernel.validate_chain_straight_evaluation_geometry_binding(
            malformed
        )
    } == {kernel.ValidationCode.EVALUATION_GEOMETRY}


def test_chain_straight_v2_validator_recomputes_v1_half_up_base_binding():
    binding = _chain_straight_binding()
    authority = next(
        item
        for item in binding.vertex_authorities
        if item.source_vertex_id == kernel.SourceVertexId("test:v1")
    )
    malformed = replace(
        binding,
        vertex_authorities=(
            binding.vertex_authorities
            - frozenset({authority})
            | frozenset(
                {
                    replace(
                        authority,
                        base_bound_node=(0, 0),
                        base_bound_coordinate=kernel.ExactPoint2V1(
                            _rational(0),
                            _rational(0),
                        ),
                    )
                }
            )
        ),
    )

    assert {
        issue.code
        for issue in kernel.validate_chain_straight_evaluation_geometry_binding(
            malformed
        )
    } == {kernel.ValidationCode.EVALUATION_GEOMETRY}


def test_binding_codec_rejects_unknown_law_and_extra_fields():
    data = json.loads(kernel.EvaluationGeometryBindingCodecV1.dumps(_binding()))
    data["binding_law"] = "SILENT_REPAIR"
    try:
        kernel.EvaluationGeometryBindingCodecV1.loads(json.dumps(data))
    except kernel.ContractCodecError:
        pass
    else:  # pragma: no cover - failure is the contract.
        raise AssertionError("unknown binding law was accepted")

    data = json.loads(kernel.EvaluationGeometryBindingCodecV1.dumps(_binding()))
    data["fallback"] = True
    try:
        kernel.EvaluationGeometryBindingCodecV1.loads(json.dumps(data))
    except kernel.ContractCodecError:
        pass
    else:  # pragma: no cover - failure is the contract.
        raise AssertionError("fallback field was accepted")


def test_binding_validator_rejects_off_lattice_and_duplicate_vertices():
    binding = _binding()
    off_lattice = replace(
        binding,
        source_vertex_coordinates=frozenset(
            {
                kernel.EvaluationGeometrySourceVertexV1(
                    kernel.SourceVertexId("test:v0"),
                    kernel.ExactPoint2V1(_rational(1, 3), _rational(0)),
                )
            }
        ),
    )
    assert {
        issue.code for issue in kernel.validate_evaluation_geometry_binding(off_lattice)
    } == {kernel.ValidationCode.EVALUATION_GEOMETRY}

    duplicate = replace(
        binding,
        source_vertex_coordinates=frozenset(
            {
                kernel.EvaluationGeometrySourceVertexV1(
                    kernel.SourceVertexId("test:v0"),
                    kernel.ExactPoint2V1(_rational(0), _rational(0)),
                ),
                kernel.EvaluationGeometrySourceVertexV1(
                    kernel.SourceVertexId("test:v0"),
                    kernel.ExactPoint2V1(_rational(1), _rational(1)),
                ),
            }
        ),
    )
    assert {
        issue.code for issue in kernel.validate_evaluation_geometry_binding(duplicate)
    } == {kernel.ValidationCode.DUPLICATE_ID}


def test_existing_plan_and_request_bytes_do_not_gain_binding_fields():
    from pathlib import Path

    from ec0_adapter import load_projection

    case_path = (
        Path(__file__).parents[1]
        / "fixtures"
        / "session_a_v5"
        / "cases"
        / "ec0-c01-straight-owner-wing.json"
    )
    projection = load_projection(case_path)
    (plan,) = projection.plans

    assert b"evaluation_geometry" not in kernel.DecalRequestCodecV1.dumps(
        projection.request
    )
    assert b"evaluation_geometry" not in kernel.CompiledPlanCodecV1.dumps(plan)
