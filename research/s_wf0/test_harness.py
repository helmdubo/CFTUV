"""Tier-0 structural tests for the S-WF0 research harness."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from research.s_wf0.fixtures import all_fixtures, connected_components, mesh_edges


def test_fixture_manifest_is_complete():
    fixtures = all_fixtures()
    assert {fixture.name for fixture in fixtures} == {
        "planar_l",
        "planar_tx",
        "concave_owner",
        "cylinder_one_seam",
        "half_sphere",
        "close_parallel_sheets",
        "retriangulated_surface",
        "fold_near_smooth",
    }
    assert len(fixtures) == 9


def test_every_source_segment_is_a_mesh_edge():
    for fixture in all_fixtures():
        edges = set(mesh_edges(fixture))
        for branch in fixture.branches:
            pairs = list(zip(branch.vertex_ids, branch.vertex_ids[1:]))
            if branch.closed:
                pairs.append((branch.vertex_ids[-1], branch.vertex_ids[0]))
            assert all(tuple(sorted(pair)) in edges for pair in pairs), fixture.key


def test_parallel_sheets_are_disconnected():
    fixture = next(
        fixture for fixture in all_fixtures() if fixture.name == "close_parallel_sheets"
    )
    assert len(connected_components(fixture)) == 2


def test_retriangulation_pair_preserves_embedded_quad_surface():
    variants = {
        fixture.variant: fixture
        for fixture in all_fixtures()
        if fixture.name == "retriangulated_surface"
    }
    assert np.array_equal(variants["a"].vertices, variants["b"].vertices)
    assert not np.array_equal(variants["a"].faces, variants["b"].faces)
    assert variants["a"].common_keys == variants["b"].common_keys


def _committed_receipt():
    path = Path(__file__).resolve().parents[2] / "artifacts" / "s_wf0" / "results.json"
    return json.loads(path.read_text(encoding="utf-8"))


def test_committed_receipt_has_all_runs_and_no_field_boundary_leak():
    receipt = _committed_receipt()
    assert receipt["schema"] == "cftuv.decal_s_wf0_results.v1"
    assert len(receipt["fixtures"]) == 9
    assert all(
        record["metrics"]["boundary_leak_count"] == 0
        for fixture in receipt["fixtures"]
        for record in fixture["methods"].values()
    )


def test_committed_retriangulation_distinguishes_heat_from_fmm():
    comparison = _committed_receipt()["retriangulation"]["methods"]
    assert comparison["FMM"]["width_delta_max"] == 0.0
    assert comparison["FMM"]["locus_f1"] == 1.0
    assert comparison["HEAT"]["width_delta_p95"] > 0.02
    assert comparison["HEAT"]["locus_f1"] < 0.7
