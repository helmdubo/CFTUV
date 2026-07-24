from __future__ import annotations

import json
from pathlib import Path


REPORT = (
    Path(__file__).resolve().parents[1]
    / "artifacts"
    / "envelope_r2c"
    / "building_002_point_touch_report.json"
)
OPERATOR_REPORT = REPORT.with_name("blender_debug_operator_report.json")


def test_building_patch_zero_reaches_raw_ready():
    report = json.loads(REPORT.read_text(encoding="utf-8"))
    patch_zero = next(
        item for item in report["domains"] if item["patch_id"] == 0
    )

    assert report["gate_passed"] is True
    assert report["source_mesh_unchanged"] is True
    assert report["selected_physical_edge_ids"] == [2, 3, 7]
    assert patch_zero["raw_ready"] is True
    assert patch_zero["raw_schema_version"].endswith(".v2")
    assert patch_zero["outcome"] == "MULTIWAY_INTERACTION_POLICY_UNPROVEN"
    assert patch_zero["point_contact_count"] == 2
    assert patch_zero["shared_arrangement_point_count"] == 2
    assert patch_zero["boundary_occurrence_count"] == 12
    assert patch_zero["region_count"] == 3
    assert patch_zero["halfedges_consumed_exactly_once"] is True
    assert patch_zero["provenance_complete"] is True
    assert patch_zero["exhaustive_differential_equal"] is True


def test_blender_operator_projects_raw_point_contact_records():
    report = json.loads(OPERATOR_REPORT.read_text(encoding="utf-8"))

    assert report["operator_result"] == ["FINISHED"]
    assert report["source_mesh_unchanged"] is True
    assert report["selected_physical_edge_ids"] == [2, 3, 7]
    assert report["stage_summary"] == (
        "Topology: 4/4 | Metric: 4/4 | Raw: 4/4 | Resolved: 3/4"
    )
    assert report["domain_status"] == "Patch 0: INTERACTION_REJECTED"
    assert report["outcome"] == "MULTIWAY_INTERACTION_POLICY_UNPROVEN"
    assert report["gp_point_contact_strokes"] > 0
    assert report["gp_boundary_occurrence_strokes"] > 0
