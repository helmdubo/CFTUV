"""Tier-0 structural tests for the S-WF0 research harness."""

from __future__ import annotations

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
