from __future__ import annotations

import json
from pathlib import Path

from cftuv_envelope import (
    AnalysisSnapshotV1,
    CompiledPatchEvaluationPlanV1,
    DecalRequestV1,
    EnvelopeDebugSceneV1,
    GeometryBatchV1,
    RationalAffinePlanarMetricV2,
    RuntimePlanarMetricV1,
    json_schema_for,
)


SCHEMA_ROOT = Path(__file__).resolve().parents[1] / "schema"


def test_checked_in_schemas_are_generated_from_public_types():
    expected = {
        "analysis_snapshot_v1.schema.json": (
            AnalysisSnapshotV1,
            "cftuv.envelope.analysis_snapshot.v1",
        ),
        "decal_request_v1.schema.json": (
            DecalRequestV1,
            "cftuv.envelope.decal_request.v1",
        ),
        "compiled_patch_evaluation_plan_v1.schema.json": (
            CompiledPatchEvaluationPlanV1,
            "cftuv.envelope.compiled_patch_evaluation_plan.v1",
        ),
        "geometry_batch_v1.schema.json": (
            GeometryBatchV1,
            "cftuv.envelope.geometry_batch.v1",
        ),
        "envelope_debug_scene_v1.schema.json": (
            EnvelopeDebugSceneV1,
            "cftuv.envelope.debug_scene.v1",
        ),
        "rational_affine_planar_metric_v2.schema.json": (
            RationalAffinePlanarMetricV2,
            "cftuv.envelope.rational_affine_planar_metric.v2",
        ),
        "runtime_planar_metric_v1.schema.json": (
            RuntimePlanarMetricV1,
            "cftuv.envelope.runtime_planar_metric.v1",
        ),
    }
    for filename, (record_type, schema_id) in expected.items():
        checked_in = json.loads((SCHEMA_ROOT / filename).read_text(encoding="utf-8"))
        assert checked_in == json_schema_for(record_type, schema_id)

