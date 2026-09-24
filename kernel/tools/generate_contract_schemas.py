"""Generate checked-in schemas from the authoritative public record types."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from cftuv_envelope import (
    AnalysisSnapshotV1,
    ChainStraightEvaluationGeometryBindingV2,
    CompiledPatchEvaluationPlanV1,
    DecalRequestV1,
    EnvelopeDebugSceneV1,
    EvaluationGeometryBindingV1,
    EmbeddingCertifiedRationalAffinePlanarMetricV1,
    GeometryBatchV1,
    RationalAffinePlanarMetricV2,
    RuntimePlanarMetricV1,
    json_schema_for,
)

# Реестр допусков намеренно НЕ выведен в корневой фасад: счёт и дайджест
# публичного `__all__` — замороженная расписка P0-4, а реестровая карточка не
# двигает поведение и не имеет права двигать эту расписку. Схема при этом
# нужна: без неё запись реестра — питоновский литерал, который никто извне не
# может ни прочитать, ни проверить.
from cftuv_envelope.contracts.tolerance_policy import (  # noqa: E402
    TOLERANCE_POLICY_REGISTRY_SCHEMA_V1,
    TolerancePolicyRegistryV1,
)


SCHEMAS = (
    (AnalysisSnapshotV1, "cftuv.envelope.analysis_snapshot.v1", "analysis_snapshot_v1.schema.json"),
    (DecalRequestV1, "cftuv.envelope.decal_request.v1", "decal_request_v1.schema.json"),
    (
        CompiledPatchEvaluationPlanV1,
        "cftuv.envelope.compiled_patch_evaluation_plan.v1",
        "compiled_patch_evaluation_plan_v1.schema.json",
    ),
    (
        EvaluationGeometryBindingV1,
        "cftuv.envelope.evaluation_geometry_binding.v1",
        "evaluation_geometry_binding_v1.schema.json",
    ),
    (
        ChainStraightEvaluationGeometryBindingV2,
        "cftuv.envelope.chain_straight_evaluation_geometry_binding.v2",
        "evaluation_geometry_binding_v2.schema.json",
    ),
    (GeometryBatchV1, "cftuv.envelope.geometry_batch.v1", "geometry_batch_v1.schema.json"),
    (
        EnvelopeDebugSceneV1,
        "cftuv.envelope.debug_scene.v1",
        "envelope_debug_scene_v1.schema.json",
    ),
    (
        RationalAffinePlanarMetricV2,
        "cftuv.envelope.rational_affine_planar_metric.v2",
        "rational_affine_planar_metric_v2.schema.json",
    ),
    (
        EmbeddingCertifiedRationalAffinePlanarMetricV1,
        "cftuv.envelope.embedding_certified_rational_affine_planar_metric.v1",
        "embedding_certified_rational_affine_planar_metric_v1.schema.json",
    ),
    (
        RuntimePlanarMetricV1,
        "cftuv.envelope.runtime_planar_metric.v1",
        "runtime_planar_metric_v1.schema.json",
    ),
    (
        TolerancePolicyRegistryV1,
        TOLERANCE_POLICY_REGISTRY_SCHEMA_V1,
        "tolerance_policy_registry_v1.schema.json",
    ),
)


def rendered_schemas() -> dict[str, str]:
    return {
        filename: json.dumps(
            json_schema_for(record_type, schema_id),
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
        )
        + "\n"
        for record_type, schema_id, filename in SCHEMAS
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    expected = rendered_schemas()
    if args.check:
        errors = [
            filename
            for filename, content in expected.items()
            if not (args.output / filename).is_file()
            or (args.output / filename).read_text(encoding="utf-8") != content
        ]
        if errors:
            print("generated schemas differ: " + ", ".join(errors))
            return 1
        print("generated-contract-schemas: OK")
        return 0
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, content in expected.items():
        (args.output / filename).write_text(content, encoding="utf-8", newline="\n")
    print(f"generated {len(expected)} schemas")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

