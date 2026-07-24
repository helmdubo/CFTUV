"""Проверяет L0 evidence corpus без импорта Blender или legacy CFTUV."""

from __future__ import annotations

import json
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_DIR = ROOT / "artifacts" / "envelope_legacy_evidence"


def _load(name: str) -> dict:
    path = EVIDENCE_DIR / name
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _assert_close(actual: float, expected: float, label: str) -> None:
    tolerance = max(1e-12, abs(expected) * 1e-12)
    assert abs(actual - expected) <= tolerance, (
        f"{label}: expected {expected!r}, got {actual!r}"
    )


def _validate_manifest(manifest: dict) -> None:
    assert manifest["schema"] == "CFTUV_ENVELOPE_LEGACY_EVIDENCE_MANIFEST_V1"
    assert manifest["session"] == "SESSION_L0_LEGACY_PLANARITY_CHAINUSE_EVIDENCE"
    assert manifest["base"]["commit"] == (
        "e95d9039c35d420b68d42b61509fc51190b01b66"
    )

    trace = manifest["field_trace"]
    assert trace["pre_state"] == trace["post_state"]
    assert trace["mutation_policy"] == {
        "analysis_input": "bmesh.from_edit_mesh(object.data).copy()",
        "geometry_materialized": False,
        "selection_changed": False,
        "scene_saved": False,
    }

    for relative_path in manifest["evidence_files"].values():
        assert (ROOT / relative_path).is_file(), f"missing evidence file: {relative_path}"


def _validate_catalog(cases_doc: dict) -> set[str]:
    catalog = cases_doc["physical_chain_catalog"]
    face_normals = cases_doc["source"]["owner_face_normal_catalog"]
    observed_refs: set[str] = set()

    assert len(catalog) == cases_doc["source"]["analysis_counts"][
        "physical_chain_edge_set_groups"
    ]

    for edge_key, physical_chain in catalog.items():
        edge_id = int(edge_key)
        kind = physical_chain["neighbor_kind"]
        uses = physical_chain["uses"]
        expected_count = 1 if kind == "MESH_BORDER" else 2
        assert len(uses) == expected_count, (
            f"edge {edge_id}: {kind} must have {expected_count} uses"
        )

        for use in uses:
            assert use["ref"].endswith(f":E{edge_id}")
            assert len(use["oriented_vertex_ids"]) == 2
            owner_normal = face_normals[str(use["owner_face_id"])]
            assert len(owner_normal) == 3
            assert use["ref"] not in observed_refs
            observed_refs.add(use["ref"])

        if kind == "PATCH":
            first, second = uses
            assert first["neighbor_patch_id"] == second["patch_id"]
            assert second["neighbor_patch_id"] == first["patch_id"]
            assert first["oriented_vertex_ids"] == list(
                reversed(second["oriented_vertex_ids"])
            )
        elif kind == "MESH_BORDER":
            assert uses[0]["neighbor_patch_id"] is None
        elif kind == "SEAM_SELF":
            assert uses[0]["patch_id"] == uses[1]["patch_id"]
        else:
            raise AssertionError(f"unknown neighbor kind: {kind}")

    return observed_refs


def _validate_planarity(cases_doc: dict) -> None:
    predicate = cases_doc["legacy_planarity_predicate"]
    assert predicate["scope"] == "all serialized vertices of the owner Patch"
    assert predicate["angle_test"] is None
    assert predicate["chart_test"] is None

    patches = cases_doc["patch_planarity"]
    assert len(patches) == cases_doc["source"]["analysis_counts"]["patches"]
    assert {patch["patch_id"] for patch in patches} == set(range(13))
    for patch in patches:
        assert patch["result"] == "PLANAR"
        assert patch["max_residual"] <= patch["tolerance"]
        assert patch["source_face_ids"]
        assert patch["source_vertex_ids"]


def _validate_cases(cases_doc: dict, all_refs: set[str]) -> None:
    catalog = cases_doc["physical_chain_catalog"]
    cases = cases_doc["cases"]
    assert [case["id"] for case in cases] == [
        "building_002_one_chain",
        "building_002_three_chains",
        "building_002_ten_chains",
        "building_002_all_seam_chains",
    ]

    seen_refs: set[str] = set()
    for case in cases:
        selected = case["selected_edge_ids"]
        assert selected == case["physical_chain_ids"]
        assert len(case["source_face_ids"]) == case["source_face_count"]
        assert len(case["source_edge_ids"]) == case["source_edge_count"]

        expected_refs = {
            use["ref"]
            for edge_id in selected
            for use in catalog[str(edge_id)]["uses"]
        }
        assert set(case["chain_use_refs"]) == expected_refs
        assert case["use_count"] == len(expected_refs)
        assert set(case["patch_ids"]) == {
            use["patch_id"]
            for edge_id in selected
            for use in catalog[str(edge_id)]["uses"]
        }
        assert case["planarity"]["patch_ids"] == case["patch_ids"]
        assert case["planarity"]["result"] == "ALL_TOUCHED_PATCHES_PLANAR"
        seen_refs.update(expected_refs)

        expected_kind_counts = {"PATCH": 0, "SEAM_SELF": 0, "MESH_BORDER": 0}
        for edge_id in selected:
            expected_kind_counts[catalog[str(edge_id)]["neighbor_kind"]] += 1
        assert case["neighbor_kind_counts"] == expected_kind_counts

    assert seen_refs == all_refs
    assert cases[-1]["selected_edge_ids"] == cases_doc["source"][
        "seam_marked_edge_ids"
    ]
    assert cases[2]["result"] == "EVALUATION_FAILURE"
    assert cases[2]["failure"]["code"] == "TERMINAL_ROUTE_RECOMPILE_REQUIRED"


def _validate_timings(cases_doc: dict, timings: dict) -> None:
    assert timings["schema"] == "CFTUV_ENVELOPE_LEGACY_BUILDING_002_TIMINGS_V1"
    assert timings["repetitions_per_case"] == 5

    case_by_id = {case["id"]: case for case in cases_doc["cases"]}
    timing_by_id = {case["id"]: case for case in timings["cases"]}
    assert timing_by_id.keys() == case_by_id.keys()

    fields = (
        "analysis_ms",
        "compile_ms",
        "evaluation_wall_ms",
        "evaluation_reported_ms",
    )
    for case_id, timing in timing_by_id.items():
        fixture_case = case_by_id[case_id]
        assert timing["selected_edge_ids"] == fixture_case["selected_edge_ids"]
        assert len(timing["samples"]) == timings["repetitions_per_case"]
        assert timing["result"]["status"] == fixture_case["result"]

        for field in fields:
            values = [
                sample[field]
                for sample in timing["samples"]
                if sample[field] is not None
            ]
            expected = timing["median"][field]
            if not values:
                assert expected is None
            else:
                assert expected is not None
                _assert_close(statistics.median(values), expected, f"{case_id}.{field}")


def main() -> None:
    manifest = _load("manifest.json")
    cases_doc = _load("building_002_cases.json")
    timings = _load("timings.json")

    _validate_manifest(manifest)
    all_refs = _validate_catalog(cases_doc)
    _validate_planarity(cases_doc)
    _validate_cases(cases_doc, all_refs)
    _validate_timings(cases_doc, timings)

    print(
        "L0 legacy evidence OK: "
        f"{len(cases_doc['cases'])} cases, "
        f"{len(cases_doc['physical_chain_catalog'])} physical chains, "
        f"{len(all_refs)} chain uses"
    )


if __name__ == "__main__":
    main()
