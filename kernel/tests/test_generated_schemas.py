from __future__ import annotations

import json
from pathlib import Path

from cftuv_envelope import (
    AnalysisSnapshotV1,
    CertifiedBoundHiddenSupportDirectionLawV1,
    ChainStraightEvaluationGeometryBindingV2,
    CompiledPatchEvaluationPlanV1,
    DecalRequestV1,
    EnvelopeDebugSceneV1,
    EmbeddingCertifiedRationalAffinePlanarMetricV1,
    EvaluationGeometryBindingV1,
    GeometryBatchV1,
    HiddenSupportDirectionLaw,
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
        "evaluation_geometry_binding_v1.schema.json": (
            EvaluationGeometryBindingV1,
            "cftuv.envelope.evaluation_geometry_binding.v1",
        ),
        "evaluation_geometry_binding_v2.schema.json": (
            ChainStraightEvaluationGeometryBindingV2,
            "cftuv.envelope.chain_straight_evaluation_geometry_binding.v2",
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
        "embedding_certified_rational_affine_planar_metric_v1.schema.json": (
            EmbeddingCertifiedRationalAffinePlanarMetricV1,
            "cftuv.envelope.embedding_certified_rational_affine_planar_metric.v1",
        ),
        "runtime_planar_metric_v1.schema.json": (
            RuntimePlanarMetricV1,
            "cftuv.envelope.runtime_planar_metric.v1",
        ),
    }
    for filename, (record_type, schema_id) in expected.items():
        checked_in = json.loads((SCHEMA_ROOT / filename).read_text(encoding="utf-8"))
        assert checked_in == json_schema_for(record_type, schema_id)


def test_hidden_support_tag_schemas_reject_the_other_tags_direction_law():
    schema = json_schema_for(
        CompiledPatchEvaluationPlanV1,
        "cftuv.envelope.compiled_patch_evaluation_plan.v1",
    )
    definitions = schema["$defs"]
    legacy_values = definitions["HiddenSupportSpecV1"]["properties"][
        "direction_law"
    ]["enum"]
    bound_values = definitions["CertifiedBoundHiddenSupportSpecV1"][
        "properties"
    ]["direction_law"]["enum"]
    legacy_law = (
        HiddenSupportDirectionLaw.ORIENTED_OWNER_SECTOR_ORDINAL_SUBTURN.value
    )
    bound_law = (
        CertifiedBoundHiddenSupportDirectionLawV1.CERTIFIED_RATIONAL_BINDING_IN_ORDINAL_SUBTURN_V1.value
    )

    assert legacy_values == [legacy_law]
    assert bound_values == [bound_law]
    assert bound_law not in legacy_values
    assert legacy_law not in bound_values



def test_checked_in_contract_schemas_match_their_generator():
    """`generate_contract_schemas.py --check` — часть прогона, а не памятка.

    Тест выше знает свои десять файлов поимённо и потому не увидит
    ОДИННАДЦАТОГО: схема, добавленная в генератор и забытая в репозитории,
    прошла бы мимо. Здесь сверяется весь набор генератора целиком.
    """

    import sys

    sys.path.insert(0, str(SCHEMA_ROOT.parent / "tools"))
    try:
        from generate_contract_schemas import rendered_schemas
    finally:
        sys.path.pop(0)

    for filename, content in rendered_schemas().items():
        path = SCHEMA_ROOT / filename
        assert path.is_file(), filename
        assert path.read_text(encoding="utf-8") == content, filename


def test_checked_in_surface_schemas_match_their_generator():
    """Схемы поверхности — не обещание генератора, а сверенные байты.

    Генератор `kernel/tools/generate_surface_contract_schemas.py` умеет
    `--check` с самого S0, но его никто не звал: проверка, которую надо не
    забыть запустить, — это слух. Здесь она становится частью прогона, и
    расхождение файла с типом падает сразу, а не на приёмке.
    """

    import sys

    sys.path.insert(0, str(SCHEMA_ROOT.parent / "tools"))
    try:
        from generate_surface_contract_schemas import rendered_schemas
    finally:
        sys.path.pop(0)

    for filename, content in rendered_schemas().items():
        path = SCHEMA_ROOT / filename
        assert path.is_file(), filename
        assert path.read_text(encoding="utf-8") == content, filename
