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
    )


def test_binding_codec_schema_and_validator_are_symmetric():
    binding = _binding()
    payload = kernel.EvaluationGeometryBindingCodecV1.dumps(binding)

    assert kernel.EvaluationGeometryBindingCodecV1.loads(payload) == binding
    assert kernel.validate_evaluation_geometry_binding(binding) == ()
    assert json.loads(payload)["binding_law"] == (
        "EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1"
    )


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
