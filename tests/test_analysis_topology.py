from __future__ import annotations

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
