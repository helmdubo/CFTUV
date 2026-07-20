from __future__ import annotations

import pytest
from mathutils import Vector

from cftuv import analysis_topology
from cftuv.model import (
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
    SeamEdge,
)


def _two_patch_graph(chain_a, chain_b):
    graph = PatchGraph()
    graph.add_node(
        PatchNode(
            patch_id=1,
            face_indices=[101],
            boundary_loops=[BoundaryLoop(chains=[chain_a])],
        )
    )
    graph.add_node(
        PatchNode(
            patch_id=2,
            face_indices=[102],
            boundary_loops=[BoundaryLoop(chains=[chain_b])],
        )
    )
    graph.add_edge(
        SeamEdge(
            patch_a_id=1,
            patch_b_id=2,
            shared_vert_indices=[10, 11, 12],
        )
    )
    return graph


def _x6_reports(monkeypatch, graph):
    reports = []
    monkeypatch.setattr(
        analysis_topology,
        "_report_graph_topology_invariant_violation",
        lambda rule_code, detail: reports.append((rule_code, detail)),
    )
    analysis_topology._validate_patch_graph_seam_consistency(graph)
    return [report for report in reports if report[0] == "X6"]


def test_x6_closed_seam_ignores_different_loop_anchors(monkeypatch):
    graph = _two_patch_graph(
        BoundaryChain(
            vert_indices=[10, 11, 12, 10],
            neighbor_patch_id=2,
            is_closed=True,
        ),
        BoundaryChain(
            vert_indices=[12, 11, 10, 12],
            neighbor_patch_id=1,
            is_closed=True,
        ),
    )

    assert not _x6_reports(monkeypatch, graph)


def test_x6_open_endpoint_mismatch_remains_visible(monkeypatch):
    graph = _two_patch_graph(
        BoundaryChain(
            vert_indices=[10, 11],
            neighbor_patch_id=2,
        ),
        BoundaryChain(
            vert_indices=[10, 12],
            neighbor_patch_id=1,
        ),
    )

    reports = _x6_reports(monkeypatch, graph)
    assert len(reports) == 1
    assert "endpoint_pair_mismatch" in reports[0][1]


def test_dihedral_pairs_preserve_both_local_normals_per_segment():
    chain_a = BoundaryChain(
        vert_indices=[10, 11, 12],
        vert_cos=[
            Vector((0.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((2.0, 0.0, 0.0)),
        ],
        edge_indices=[20, 21],
        side_face_indices=[101, 102],
        side_face_normals=[
            Vector((0.0, 0.0, 1.0)),
            Vector((0.0, 1.0, 0.0)),
        ],
        neighbor_patch_id=2,
    )
    chain_b = BoundaryChain(
        vert_indices=[12, 11, 10],
        vert_cos=[
            Vector((2.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((0.0, 0.0, 0.0)),
        ],
        edge_indices=[21, 20],
        side_face_indices=[202, 201],
        side_face_normals=[
            Vector((0.0, 0.0, 1.0)),
            Vector((0.0, 0.001, 0.9999995)),
        ],
        neighbor_patch_id=1,
    )
    graph = _two_patch_graph(chain_a, chain_b)

    analysis_topology._assign_chain_dihedral_convexity(graph)

    assert tuple(pair.edge_index for pair in chain_a.dihedral_segment_pairs) == (20, 21)
    first, second = chain_a.dihedral_segment_pairs
    assert first.owner_face_index == 101
    assert first.neighbor_face_index == 201
    assert first.owner_normal == (0.0, 0.0, 1.0)
    assert first.neighbor_normal == pytest.approx((0.0, 0.001, 0.9999995))
    assert first.convexity != 0.0
    assert second.owner_face_index == 102
    assert second.neighbor_face_index == 202
    assert chain_a.dihedral_convexity == pytest.approx(
        sum(pair.convexity for pair in chain_a.dihedral_segment_pairs) / 2.0
    )
