"""Artifact gate for the M-R0 planar metric decision.

The benchmark itself intentionally lives outside production packages.  This
test protects the recorded semantic conclusions without asserting timings.
"""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS_PATH = ROOT / "artifacts" / "envelope_metric_spike" / "results.json"
BUILDING_PATH = (
    ROOT
    / "artifacts"
    / "envelope_metric_spike"
    / "building_002_patch3.json"
)

MODEL_A = "ALGEBRAIC_ORTHONORMAL_FRAME"
MODEL_B = "RATIONAL_AFFINE_GRAM"
MODEL_C = "CERTIFIED_FLOAT_FILTERED_CANDIDATE"

FIXTURES = {
    "axis_aligned_plane",
    "arbitrary_rotated_plane",
    "forty_five_degrees",
    "irrational_normal",
    "micro_scale",
    "huge_coordinates",
    "nearly_planar_outside_budget",
    "same_plane_different_triangulation",
    "same_plane_after_object_rotation",
    "building_002_patch_3",
}


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_metric_spike_compares_all_models_on_all_required_fixtures() -> None:
    results = _load(RESULTS_PATH)
    fixtures = {item["fixture"]: item for item in results["fixtures"]}

    assert results["schema"] == "cftuv.envelope.planar_metric_spike_results.v1"
    assert set(fixtures) == FIXTURES
    assert results["iterations_per_admitted_model"] == 5

    for fixture in fixtures.values():
        assert set(fixture["models"]) == {MODEL_A, MODEL_B, MODEL_C}


def test_exact_models_have_one_semantic_result_and_stable_digests() -> None:
    results = _load(RESULTS_PATH)

    assert results["gate"]["exact_a_b_semantic_equivalence"] is True
    assert results["gate"]["all_digests_stable"] is True
    assert results["gate"]["production_contracts_changed"] is False

    for fixture in results["fixtures"]:
        models = fixture["models"]
        if models[MODEL_A]["admission_success"]:
            assert models[MODEL_B]["admission_success"] is True
            assert (
                fixture["comparisons"]["exact_model_semantic_result_equal"]
                is True
            )
            assert (
                fixture["comparisons"]["all_admitted_digests_stable"] is True
            )


def test_nearly_planar_fixture_is_rejected_without_hidden_tolerance() -> None:
    results = _load(RESULTS_PATH)
    fixture = next(
        item
        for item in results["fixtures"]
        if item["fixture"] == "nearly_planar_outside_budget"
    )

    assert all(
        not model["admission_success"]
        for model in fixture["models"].values()
    )
    assert (
        fixture["models"][MODEL_C]["admission_outcome"].split(":", 1)[0]
        == "RUNTIME_PLANAR_RESIDUAL_BUDGET_EXCEEDED"
    )
    assert (
        results["runtime_residual_policy_candidate"]["status"]
        == "RESEARCH_CANDIDATE_NOT_PRODUCT_POLICY"
    )
    assert (
        results["filtered_predicate_policy_candidate"]["status"]
        == "RESEARCH_CANDIDATE_NOT_PRODUCT_POLICY"
    )


def test_runtime_candidate_is_stopped_on_semantic_rotation_mismatches() -> None:
    results = _load(RESULTS_PATH)

    assert (
        results["gate"]["runtime_candidate_status"]
        == "STOPPED_NOT_READY_FOR_PRODUCTION"
    )
    assert set(results["gate"]["runtime_semantic_mismatch_fixtures"]) == {
        "arbitrary_rotated_plane",
        "same_plane_after_object_rotation",
    }

    fixtures = {item["fixture"]: item for item in results["fixtures"]}
    for name in results["gate"]["runtime_semantic_mismatch_fixtures"]:
        comparison = fixtures[name]["comparisons"]["runtime_vs_reference"]
        assert comparison["combinatorial_topology_equal"] is True
        assert comparison["semantic_topology_and_provenance_equal"] is False


def test_building_patch3_receipt_is_read_only_and_resolves_v1_rejection() -> None:
    receipt = _load(BUILDING_PATH)
    source = receipt["source"]
    fixture = receipt["fixture_result"]

    assert receipt["schema"] == "cftuv.envelope.planar_metric_building_patch.v1"
    assert source["source_object"] == "building.002"
    assert source["patch_id"] == 3
    assert source["source_mesh_unchanged"] is True
    assert (
        source["mesh_fingerprint_before"]
        == source["mesh_fingerprint_after"]
    )
    assert source["current_v1_admission"] == {
        "message": "host Patch 3 exact plane has no rational unit normal",
        "outcome": "ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE",
        "status": "REJECTED",
    }

    for model in fixture["models"].values():
        assert model["admission_success"] is True
    assert (
        fixture["models"][MODEL_B]["chart"][
            "domain_coordinates_all_rational"
        ]
        is True
    )
    assert fixture["models"][MODEL_C]["chart"]["exact"] is False
    assert fixture["comparisons"]["exact_model_semantic_result_equal"] is True
    assert (
        fixture["comparisons"]["runtime_vs_reference"][
            "semantic_topology_and_provenance_equal"
        ]
        is True
    )
