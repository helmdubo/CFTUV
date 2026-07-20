from __future__ import annotations

from cftuv import analysis_boundary_loops
from cftuv.model import BoundaryChain, BoundaryLoop


def test_closed_single_chain_spans_follow_anchor_and_closing_repeat():
    assert analysis_boundary_loops._loop_vertex_span(
        [10, 11, 12, 13], 2, 2
    ) == [12, 13, 10, 11, 12]
    assert analysis_boundary_loops._loop_edge_span(
        [100, 101, 102, 103], 2, 2
    ) == [102, 103, 100, 101]


def test_closed_single_chain_does_not_emit_false_x7_x8(monkeypatch):
    reports = []
    monkeypatch.setattr(
        analysis_boundary_loops,
        "_report_boundary_loop_invariant_violation",
        lambda patch_id, loop_index, rule_code, detail: reports.append(
            (rule_code, detail)
        ),
    )
    boundary_loop = BoundaryLoop(
        vert_indices=[10, 11, 12, 13],
        edge_indices=[100, 101, 102, 103],
        chains=[
            BoundaryChain(
                vert_indices=[12, 13, 10, 11, 12],
                edge_indices=[102, 103, 100, 101],
                start_loop_index=2,
                end_loop_index=2,
                is_closed=True,
            )
        ],
    )

    analysis_boundary_loops._validate_boundary_loop_topology(
        boundary_loop,
        patch_id=7,
        loop_index=0,
    )

    assert not [report for report in reports if report[0] in {"X7", "X8"}]


def test_open_chain_span_semantics_are_unchanged():
    assert analysis_boundary_loops._loop_vertex_span(
        [10, 11, 12, 13], 2, 0
    ) == [12, 13, 10]
    assert analysis_boundary_loops._loop_edge_span(
        [100, 101, 102, 103], 2, 0
    ) == [102, 103]
