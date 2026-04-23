from __future__ import annotations

from dataclasses import FrozenInstanceError, replace

import pytest
from mathutils import Vector

import cftuv.solve_skeleton as solve_skeleton_module
from cftuv.analysis_boundary_loops import _build_boundary_loop_chain_uses
from cftuv.analysis_junctions import _build_patch_graph_junctions, _materialize_junction
from cftuv.analysis_records import _JunctionBuildEntry, _JunctionCornerRef, _PatchGraphDerivedTopology
from cftuv.model import (
    BoundaryChain,
    BoundaryCorner,
    BoundaryLoop,
    ChainUse,
    CornerKind,
    FrameRole,
    LoopKind,
    PatchGraph,
    PatchNode,
    PatchType,
    ScaffoldMap,
    ScaffoldChainPlacement,
    ScaffoldPatchPlacement,
    ScaffoldPointKey,
    ScaffoldQuiltPlacement,
    SkeletonFlags,
    WorldFacing,
)
from cftuv.solve_records import SolveView
from cftuv.solve_records import QuiltPlan, QuiltStep, SeamRelationProfile, SolvePlan
from cftuv.solve_reporting import (
    format_regression_snapshot_report,
    format_root_scaffold_report,
)
from cftuv.shape_types import PatchShapeClass
from cftuv.solve_skeleton import (
    SkeletonSolveInput,
    apply_skeleton_solve,
    apply_skeleton_solve_to_scaffold_map,
    build_sibling_groups,
    build_skeleton_graphs,
)


def _make_chain(vert_indices, edge_indices, vert_cos, role):
    return BoundaryChain(
        vert_indices=list(vert_indices),
        edge_indices=list(edge_indices),
        vert_cos=[Vector(co) for co in vert_cos],
        frame_role=role,
    )


def _make_corner(loop_vert_index, vert_index, vert_co, prev_chain_index, next_chain_index):
    return BoundaryCorner(
        loop_vert_index=loop_vert_index,
        vert_index=vert_index,
        vert_co=Vector(vert_co),
        prev_chain_index=prev_chain_index,
        next_chain_index=next_chain_index,
        corner_kind=CornerKind.JUNCTION,
    )


def _make_loop(kind, vert_specs, edge_specs):
    chain_count = len(edge_specs)
    return BoundaryLoop(
        kind=kind,
        vert_indices=[vert_index for vert_index, _ in vert_specs],
        vert_cos=[Vector(vert_co) for _, vert_co in vert_specs],
        chains=[
            _make_chain(
                edge_spec["vert_indices"],
                edge_spec["edge_indices"],
                edge_spec["vert_cos"],
                edge_spec["role"],
            )
            for edge_spec in edge_specs
        ],
        corners=[
            _make_corner(
                loop_vert_index,
                vert_specs[loop_vert_index][0],
                vert_specs[loop_vert_index][1],
                (loop_vert_index - 1) % chain_count,
                loop_vert_index,
            )
            for loop_vert_index in range(chain_count)
        ],
    )


def _make_rectangle_patch(
    patch_id,
    vert_specs,
    edge_specs,
    *,
    patch_type=PatchType.WALL,
    world_facing=WorldFacing.SIDE,
):
    loop = _make_loop(LoopKind.OUTER, vert_specs, edge_specs)
    return PatchNode(
        patch_id=patch_id,
        face_indices=[],
        boundary_loops=[loop],
        patch_type=patch_type,
        world_facing=world_facing,
    )


def _build_derived_topology_for_graph(graph):
    graph.rebuild_chain_use_index()
    junctions = tuple(_build_patch_graph_junctions(graph))
    return _PatchGraphDerivedTopology(
        junctions=junctions,
        junctions_by_vert_index={junction.vert_index: junction for junction in junctions},
    )


def _make_chain_placement(patch_id, loop_index, chain_index, frame_role, uv_points):
    return ScaffoldChainPlacement(
        patch_id=patch_id,
        loop_index=loop_index,
        chain_index=chain_index,
        frame_role=frame_role,
        points=tuple(
            (
                ScaffoldPointKey(
                    patch_id=patch_id,
                    loop_index=loop_index,
                    chain_index=chain_index,
                    source_point_index=point_index,
                ),
                Vector(uv_point),
            )
            for point_index, uv_point in enumerate(uv_points)
        ),
    )


def _make_patch_placement(patch_id, loop_index, chain_placements, root_chain_index=0):
    return ScaffoldPatchPlacement(
        patch_id=patch_id,
        loop_index=loop_index,
        root_chain_index=root_chain_index,
        chain_placements=list(chain_placements),
    )


def _assert_uv(point, expected_x, expected_y):
    assert point.x == pytest.approx(expected_x)
    assert point.y == pytest.approx(expected_y)


def test_build_boundary_loop_chain_uses_assigns_shared_chain_id_and_opposite_axis_sign():
    loop_a = BoundaryLoop(
        kind=LoopKind.OUTER,
        chains=[
            _make_chain((10, 11, 12), (20, 21), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), FrameRole.H_FRAME),
        ],
    )
    loop_b = BoundaryLoop(
        kind=LoopKind.OUTER,
        chains=[
            _make_chain((12, 11, 10), (21, 20), ((2.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)), FrameRole.H_FRAME),
        ],
    )

    chain_use_a = _build_boundary_loop_chain_uses(loop_a, patch_id=1, loop_index=0)[0]
    chain_use_b = _build_boundary_loop_chain_uses(loop_b, patch_id=2, loop_index=0)[0]

    assert chain_use_a.chain_id == chain_use_b.chain_id
    assert chain_use_a.axis_sign == -chain_use_b.axis_sign
    assert chain_use_a.position_in_loop == 0
    assert chain_use_b.position_in_loop == 0


def test_patch_iter_boundary_loop_oriented_returns_chain_uses_in_canonical_order():
    boundary_loop = BoundaryLoop(
        kind=LoopKind.OUTER,
        vert_indices=[0, 1, 2],
        chains=[
            _make_chain((0, 1), (10,), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), FrameRole.H_FRAME),
            _make_chain((1, 2), (11,), ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), FrameRole.V_FRAME),
        ],
        chain_uses=[
            ChainUse(chain_id=(-1, 10), patch_id=7, loop_index=0, chain_index=0, position_in_loop=1, axis_sign=1, role_in_loop=FrameRole.H_FRAME),
            ChainUse(chain_id=(-1, 11), patch_id=7, loop_index=0, chain_index=1, position_in_loop=0, axis_sign=1, role_in_loop=FrameRole.V_FRAME),
        ],
    )
    boundary_loop.chains[0].start_loop_index = 0
    boundary_loop.chains[0].end_loop_index = 1
    boundary_loop.chains[1].start_loop_index = 1
    boundary_loop.chains[1].end_loop_index = 2
    patch = PatchNode(patch_id=7, face_indices=[], boundary_loops=[boundary_loop])

    oriented = patch.iter_boundary_loop_oriented()

    assert [chain_use.chain_index for chain_use in oriented] == [1, 0]
    assert [chain_use.position_in_loop for chain_use in oriented] == [0, 1]
    assert [chain_use.role_in_loop for chain_use in oriented] == [FrameRole.V_FRAME, FrameRole.H_FRAME]

    resolved_source_point = boundary_loop.resolve_chain_use_source_point(oriented[0], 1)
    assert resolved_source_point == (2, 2)


def test_boundary_loop_oriented_neighbor_indices_follow_position_in_loop_not_storage_order():
    boundary_loop = BoundaryLoop(
        kind=LoopKind.OUTER,
        chains=[
            _make_chain((0, 1), (10,), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), FrameRole.H_FRAME),
            _make_chain((1, 2), (11,), ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), FrameRole.V_FRAME),
            _make_chain((2, 3), (12,), ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), FrameRole.H_FRAME),
        ],
        chain_uses=[
            ChainUse(chain_id=(-1, 10), patch_id=5, loop_index=0, chain_index=0, position_in_loop=2, axis_sign=1, role_in_loop=FrameRole.H_FRAME),
            ChainUse(chain_id=(-1, 11), patch_id=5, loop_index=0, chain_index=1, position_in_loop=0, axis_sign=1, role_in_loop=FrameRole.V_FRAME),
            ChainUse(chain_id=(-1, 12), patch_id=5, loop_index=0, chain_index=2, position_in_loop=1, axis_sign=1, role_in_loop=FrameRole.H_FRAME),
        ],
    )

    assert [chain_use.chain_index for chain_use in boundary_loop.iter_chain_uses_oriented()] == [1, 2, 0]
    assert boundary_loop.oriented_neighbor_chain_indices(2) == (1, 0)


def test_patch_graph_rebuild_chain_use_index_tracks_shared_chain_uses():
    loop_a = BoundaryLoop(
        kind=LoopKind.OUTER,
        chains=[
            _make_chain((10, 11), (20,), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), FrameRole.H_FRAME),
        ],
    )
    loop_b = BoundaryLoop(
        kind=LoopKind.OUTER,
        chains=[
            _make_chain((11, 10), (20,), ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)), FrameRole.H_FRAME),
        ],
    )
    graph = PatchGraph(
        nodes={
            1: PatchNode(patch_id=1, face_indices=[], boundary_loops=[loop_a]),
            2: PatchNode(patch_id=2, face_indices=[], boundary_loops=[loop_b]),
        }
    )

    graph.rebuild_chain_use_index()

    chain_use_a = graph.get_chain_use(1, 0, 0)
    assert chain_use_a is not None
    chain_uses = graph.iter_chain_uses_for_chain_id(chain_use_a.chain_id)
    assert {(chain_use.patch_id, chain_use.loop_index, chain_use.chain_index) for chain_use in chain_uses} == {
        (1, 0, 0),
        (2, 0, 0),
    }


def test_chain_use_invariants_cover_mesh_border_seam_self_hole_and_immutability():
    mesh_border_chain = _make_chain(
        (0, 1),
        (10,),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        FrameRole.H_FRAME,
    )
    mesh_border_chain.neighbor_patch_id = -1

    seam_self_outer = _make_chain(
        (1, 2),
        (20,),
        ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)),
        FrameRole.V_FRAME,
    )
    seam_self_outer.neighbor_patch_id = -2

    seam_self_hole = _make_chain(
        (2, 1),
        (20,),
        ((1.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
        FrameRole.V_FRAME,
    )
    seam_self_hole.neighbor_patch_id = -2

    graph = PatchGraph(
        nodes={
            7: PatchNode(
                patch_id=7,
                face_indices=[],
                boundary_loops=[
                    BoundaryLoop(kind=LoopKind.OUTER, chains=[mesh_border_chain, seam_self_outer]),
                    BoundaryLoop(kind=LoopKind.HOLE, chains=[seam_self_hole]),
                ],
            )
        }
    )

    graph.rebuild_chain_use_index()

    assert len(graph.chain_use_by_ref) == 3

    mesh_border_use = graph.get_chain_use(7, 0, 0)
    assert mesh_border_use is not None
    assert len(graph.iter_chain_uses_for_chain_id(mesh_border_use.chain_id)) == 1

    seam_self_use_outer = graph.get_chain_use(7, 0, 1)
    seam_self_use_hole = graph.get_chain_use(7, 1, 0)
    assert seam_self_use_outer is not None
    assert seam_self_use_hole is not None
    seam_self_uses = graph.iter_chain_uses_for_chain_id(seam_self_use_outer.chain_id)
    assert {(chain_use.patch_id, chain_use.loop_index) for chain_use in seam_self_uses} == {
        (7, 0),
        (7, 1),
    }
    assert seam_self_use_outer.axis_sign == -seam_self_use_hole.axis_sign

    with pytest.raises(FrozenInstanceError):
        seam_self_use_outer.axis_sign = 99


def test_solve_view_iter_visible_chain_records_uses_chain_use_order_and_skips_holes():
    outer_loop = BoundaryLoop(
        kind=LoopKind.OUTER,
        chains=[
            _make_chain((0, 1), (10,), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), FrameRole.H_FRAME),
            _make_chain((1, 2), (11,), ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), FrameRole.V_FRAME),
        ],
        chain_uses=[
            ChainUse(chain_id=(-1, 10), patch_id=1, loop_index=0, chain_index=0, position_in_loop=1, axis_sign=1, role_in_loop=FrameRole.H_FRAME),
            ChainUse(chain_id=(-1, 11), patch_id=1, loop_index=0, chain_index=1, position_in_loop=0, axis_sign=1, role_in_loop=FrameRole.V_FRAME),
        ],
    )
    outer_loop.chains[1].neighbor_patch_id = 2

    hole_loop = BoundaryLoop(
        kind=LoopKind.HOLE,
        chains=[
            _make_chain((3, 4), (12,), ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), FrameRole.FREE),
        ],
    )

    target_loop = BoundaryLoop(
        kind=LoopKind.OUTER,
        chains=[
            _make_chain((2, 1), (11,), ((1.0, 1.0, 0.0), (1.0, 0.0, 0.0)), FrameRole.V_FRAME),
        ],
    )
    target_loop.chains[0].neighbor_patch_id = 1

    graph = PatchGraph(
        nodes={
            1: PatchNode(patch_id=1, face_indices=[], boundary_loops=[outer_loop, hole_loop]),
            2: PatchNode(patch_id=2, face_indices=[], boundary_loops=[target_loop]),
        }
    )
    graph.rebuild_chain_use_index()
    solve_view = SolveView(
        graph=graph,
        visible_loop_indices_by_patch={1: (0,), 2: (0,)},
        primary_loop_index_by_patch={1: 0, 2: 0},
        locally_solvable_patch_ids=frozenset({1, 2}),
    )

    records = list(solve_view.iter_visible_chain_records(1))
    assert [(chain_use.loop_index, chain_use.chain_index) for chain_use, _, _ in records] == [
        (0, 1),
        (0, 0),
    ]

    attachment_refs = solve_view.iter_attachment_neighbor_chains(1, 2)
    assert len(attachment_refs) == 1
    assert attachment_refs[0].chain_use.loop_index == 0
    assert attachment_refs[0].chain_use.chain_index == 1


def test_materialized_junction_orders_cardinal_chain_incidences_by_angle():
    graph = PatchGraph()
    patch_specs = [
        (1, (0, 1), (10,), ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))),
        (2, (0, 2), (11,), ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))),
        (3, (3, 0), (12,), ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0))),
        (4, (4, 0), (13,), ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0))),
    ]

    entry = _JunctionBuildEntry(vert_index=0, vert_co=Vector((0.0, 0.0, 0.0)))
    for patch_id, vert_indices, edge_indices, vert_cos in patch_specs:
        boundary_loop = BoundaryLoop(
            kind=LoopKind.OUTER,
            chains=[_make_chain(vert_indices, edge_indices, vert_cos, FrameRole.H_FRAME)],
            corners=[
                BoundaryCorner(
                    vert_index=0,
                    vert_co=Vector((0.0, 0.0, 0.0)),
                    prev_chain_index=0,
                    next_chain_index=0,
                    corner_kind=CornerKind.JUNCTION,
                )
            ],
        )
        graph.add_node(PatchNode(patch_id=patch_id, face_indices=[], boundary_loops=[boundary_loop]))
        entry.patch_ids.add(patch_id)
        entry.corner_refs.append(
            _JunctionCornerRef(
                patch_id=patch_id,
                loop_index=0,
                corner_index=0,
                prev_chain_index=0,
                next_chain_index=0,
            )
        )

    graph.rebuild_chain_use_index()
    junction = _materialize_junction(graph, entry, {})

    ordered_refs = [
        (incidence.chain_use.patch_id, incidence.side)
        for incidence in junction.ordered_disk_cycle()
    ]
    assert ordered_refs == [
        (4, "end"),
        (1, "start"),
        (2, "start"),
        (3, "end"),
    ]

    chain_ref_map = {
        (chain_ref.patch_id, chain_ref.loop_index, chain_ref.chain_index): chain_ref.chain_use
        for chain_ref in junction.chain_refs
    }
    assert chain_ref_map[(1, 0, 0)] is not None
    assert junction.row_component_id is None
    assert junction.col_component_id is None
    assert junction.canonical_u is None
    assert junction.canonical_v is None
    assert junction.skeleton_flags == SkeletonFlags(0)


def test_apply_skeleton_solve_is_noop_in_s1_checkpoint():
    report = apply_skeleton_solve(quilt=None, diagnostics=None)

    assert report.applied is False
    assert report.row_component_count == 0
    assert report.col_component_count == 0
    assert report.notes == ("skeleton_solve_not_integrated",)


def test_build_skeleton_graphs_builds_row_and_column_components_for_rectangle():
    graph = PatchGraph(
        nodes={
            10: _make_rectangle_patch(
                10,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (100,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (101,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (102,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (103,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            )
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)

    skeleton_graphs = build_skeleton_graphs(graph, derived_topology)

    assert len(skeleton_graphs.row_component_members) == 2
    assert len(skeleton_graphs.col_component_members) == 2
    assert skeleton_graphs.junctions_by_vert_index[0].row_component_id == skeleton_graphs.junctions_by_vert_index[1].row_component_id
    assert skeleton_graphs.junctions_by_vert_index[2].row_component_id == skeleton_graphs.junctions_by_vert_index[3].row_component_id
    assert skeleton_graphs.junctions_by_vert_index[0].col_component_id == skeleton_graphs.junctions_by_vert_index[3].col_component_id
    assert skeleton_graphs.junctions_by_vert_index[1].col_component_id == skeleton_graphs.junctions_by_vert_index[2].col_component_id


def test_build_skeleton_graphs_keeps_shared_vertical_seam_in_one_column_component():
    graph = PatchGraph(
        nodes={
            20: _make_rectangle_patch(
                20,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (200,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (201,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (202,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (203,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
            21: _make_rectangle_patch(
                21,
                [
                    (1, (1.0, 0.0, 0.0)),
                    (4, (2.0, 0.0, 0.0)),
                    (5, (2.0, 1.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (1, 4), "edge_indices": (210,), "vert_cos": ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (4, 5), "edge_indices": (211,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (5, 2), "edge_indices": (212,), "vert_cos": ((2.0, 1.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (2, 1), "edge_indices": (201,), "vert_cos": ((1.0, 1.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)

    skeleton_graphs = build_skeleton_graphs(graph, derived_topology)

    seam_bottom = skeleton_graphs.junctions_by_vert_index[1]
    seam_top = skeleton_graphs.junctions_by_vert_index[2]
    assert seam_bottom.col_component_id == seam_top.col_component_id
    assert len(skeleton_graphs.row_component_members) == 2


def test_build_skeleton_graphs_splits_valence_five_row_outlier():
    nodes = {}
    outer_specs = [
        (1, (1.0, 0.0, 0.0)),
        (2, (2.0, 0.0, 0.0)),
        (3, (3.0, 0.0, 0.0)),
        (4, (4.0, 0.0, 0.0)),
        (5, (5.0, 0.0, 1.0)),
    ]
    for patch_id, (outer_vert, outer_co) in enumerate(outer_specs, start=30):
        loop = BoundaryLoop(
            kind=LoopKind.OUTER,
            vert_indices=[0, outer_vert],
            vert_cos=[Vector((0.0, 0.0, 0.0)), Vector(outer_co)],
            chains=[
                _make_chain(
                    (0, outer_vert),
                    (300 + patch_id,),
                    ((0.0, 0.0, 0.0), outer_co),
                    FrameRole.H_FRAME,
                )
            ],
            corners=[
                _make_corner(0, 0, (0.0, 0.0, 0.0), 0, 0),
                _make_corner(1, outer_vert, outer_co, 0, 0),
            ],
        )
        nodes[patch_id] = PatchNode(patch_id=patch_id, face_indices=[], boundary_loops=[loop])

    graph = PatchGraph(nodes=nodes)
    derived_topology = _build_derived_topology_for_graph(graph)

    skeleton_graphs = build_skeleton_graphs(graph, derived_topology)

    assert any(report.resolved for report in skeleton_graphs.singular_unions)
    assert skeleton_graphs.junctions_by_vert_index[0].skeleton_flags & SkeletonFlags.SINGULAR_SPLIT
    assert len(skeleton_graphs.row_component_members) == 2
    assert skeleton_graphs.junctions_by_vert_index[5].row_component_id != skeleton_graphs.junctions_by_vert_index[1].row_component_id


def test_build_sibling_groups_detects_repeated_doorway_tops_and_sides():
    graph = PatchGraph(
        nodes={
            60: PatchNode(
                patch_id=60,
                face_indices=[],
                boundary_loops=[
                    _make_loop(
                        LoopKind.OUTER,
                        [
                            (0, (0.0, 0.0, 0.0)),
                            (1, (8.0, 0.0, 0.0)),
                            (2, (8.0, 4.0, 0.0)),
                            (3, (0.0, 4.0, 0.0)),
                        ],
                        [
                            {"vert_indices": (0, 1), "edge_indices": (600,), "vert_cos": ((0.0, 0.0, 0.0), (8.0, 0.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (1, 2), "edge_indices": (601,), "vert_cos": ((8.0, 0.0, 0.0), (8.0, 4.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (2, 3), "edge_indices": (602,), "vert_cos": ((8.0, 4.0, 0.0), (0.0, 4.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (3, 0), "edge_indices": (603,), "vert_cos": ((0.0, 4.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.FREE},
                        ],
                    ),
                    _make_loop(
                        LoopKind.HOLE,
                        [
                            (10, (1.0, 0.0, 0.0)),
                            (11, (2.0, 0.0, 0.0)),
                            (12, (2.0, 2.0, 0.0)),
                            (13, (1.0, 2.0, 0.0)),
                        ],
                        [
                            {"vert_indices": (10, 11), "edge_indices": (610,), "vert_cos": ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (11, 12), "edge_indices": (611,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 2.0, 0.0)), "role": FrameRole.V_FRAME},
                            {"vert_indices": (12, 13), "edge_indices": (612,), "vert_cos": ((2.0, 2.0, 0.0), (1.0, 2.0, 0.0)), "role": FrameRole.H_FRAME},
                            {"vert_indices": (13, 10), "edge_indices": (613,), "vert_cos": ((1.0, 2.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                        ],
                    ),
                    _make_loop(
                        LoopKind.HOLE,
                        [
                            (20, (4.0, 0.0, 0.0)),
                            (21, (5.0, 0.0, 0.0)),
                            (22, (5.0, 2.0, 0.0)),
                            (23, (4.0, 2.0, 0.0)),
                        ],
                        [
                            {"vert_indices": (20, 21), "edge_indices": (620,), "vert_cos": ((4.0, 0.0, 0.0), (5.0, 0.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (21, 22), "edge_indices": (621,), "vert_cos": ((5.0, 0.0, 0.0), (5.0, 2.0, 0.0)), "role": FrameRole.V_FRAME},
                            {"vert_indices": (22, 23), "edge_indices": (622,), "vert_cos": ((5.0, 2.0, 0.0), (4.0, 2.0, 0.0)), "role": FrameRole.H_FRAME},
                            {"vert_indices": (23, 20), "edge_indices": (623,), "vert_cos": ((4.0, 2.0, 0.0), (4.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                        ],
                    ),
                ],
            )
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    skeleton_graphs = build_skeleton_graphs(graph, derived_topology)

    sibling_groups = build_sibling_groups(graph, skeleton_graphs)

    assert len(sibling_groups) == 2
    h_group = next(group for group in sibling_groups if group.role == FrameRole.H_FRAME)
    v_group = next(group for group in sibling_groups if group.role == FrameRole.V_FRAME)
    assert h_group.patch_id == 60
    assert len(h_group.members) == 2
    assert h_group.target_length == pytest.approx(1.0)
    assert len(v_group.members) == 4
    assert v_group.target_length == pytest.approx(2.0)


def test_build_sibling_groups_skips_non_matching_openings():
    graph = PatchGraph(
        nodes={
            61: PatchNode(
                patch_id=61,
                face_indices=[],
                boundary_loops=[
                    _make_loop(
                        LoopKind.OUTER,
                        [
                            (0, (0.0, 0.0, 0.0)),
                            (1, (8.0, 0.0, 0.0)),
                            (2, (8.0, 5.0, 0.0)),
                            (3, (0.0, 5.0, 0.0)),
                        ],
                        [
                            {"vert_indices": (0, 1), "edge_indices": (700,), "vert_cos": ((0.0, 0.0, 0.0), (8.0, 0.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (1, 2), "edge_indices": (701,), "vert_cos": ((8.0, 0.0, 0.0), (8.0, 5.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (2, 3), "edge_indices": (702,), "vert_cos": ((8.0, 5.0, 0.0), (0.0, 5.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (3, 0), "edge_indices": (703,), "vert_cos": ((0.0, 5.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.FREE},
                        ],
                    ),
                    _make_loop(
                        LoopKind.HOLE,
                        [
                            (10, (1.0, 0.0, 0.0)),
                            (11, (2.0, 0.0, 0.0)),
                            (12, (2.0, 2.0, 0.0)),
                            (13, (1.0, 1.7, 0.0)),
                        ],
                        [
                            {"vert_indices": (10, 11), "edge_indices": (710,), "vert_cos": ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (11, 12), "edge_indices": (711,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 2.0, 0.0)), "role": FrameRole.V_FRAME},
                            {"vert_indices": (12, 13), "edge_indices": (712,), "vert_cos": ((2.0, 2.0, 0.0), (1.0, 1.7, 0.0)), "role": FrameRole.H_FRAME},
                            {"vert_indices": (13, 10), "edge_indices": (713,), "vert_cos": ((1.0, 1.7, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                        ],
                    ),
                    _make_loop(
                        LoopKind.HOLE,
                        [
                            (20, (4.0, 1.0, 0.0)),
                            (21, (5.7, 1.0, 0.0)),
                            (22, (5.7, 2.3, 0.0)),
                            (23, (4.0, 2.1, 0.0)),
                        ],
                        [
                            {"vert_indices": (20, 21), "edge_indices": (720,), "vert_cos": ((4.0, 1.0, 0.0), (5.7, 1.0, 0.0)), "role": FrameRole.FREE},
                            {"vert_indices": (21, 22), "edge_indices": (721,), "vert_cos": ((5.7, 1.0, 0.0), (5.7, 2.3, 0.0)), "role": FrameRole.V_FRAME},
                            {"vert_indices": (22, 23), "edge_indices": (722,), "vert_cos": ((5.7, 2.3, 0.0), (4.0, 2.1, 0.0)), "role": FrameRole.H_FRAME},
                            {"vert_indices": (23, 20), "edge_indices": (723,), "vert_cos": ((4.0, 2.1, 0.0), (4.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                        ],
                    ),
                ],
            )
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    skeleton_graphs = build_skeleton_graphs(graph, derived_topology)

    assert build_sibling_groups(graph, skeleton_graphs) == ()


def test_apply_skeleton_solve_preserves_aligned_rectangle():
    graph = PatchGraph(
        nodes={
            70: _make_rectangle_patch(
                70,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (800,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (801,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (802,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (803,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            )
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=0,
        root_patch_id=70,
        patches={
            70: _make_patch_placement(
                70,
                0,
                [
                    _make_chain_placement(70, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (2.0, 0.0)]),
                    _make_chain_placement(70, 0, 1, FrameRole.V_FRAME, [(2.0, 0.0), (2.0, 1.0)]),
                    _make_chain_placement(70, 0, 2, FrameRole.H_FRAME, [(2.0, 1.0), (0.0, 1.0)]),
                    _make_chain_placement(70, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                ],
            )
        },
    )

    report = apply_skeleton_solve(
        SkeletonSolveInput(
            graph=graph,
            derived_topology=derived_topology,
            quilt_scaffold=quilt_scaffold,
            final_scale=1.0,
        )
    )

    assert report.applied is True
    assert report.residual_u == pytest.approx(0.0)
    assert report.residual_v == pytest.approx(0.0)
    rebuilt_patch = report.quilt_scaffold.patches[70]
    assert rebuilt_patch.closure_valid is True
    assert rebuilt_patch.closure_error == pytest.approx(0.0)
    _assert_uv(rebuilt_patch.chain_placements[0].points[0][1], 0.0, 0.0)
    _assert_uv(rebuilt_patch.chain_placements[0].points[-1][1], 2.0, 0.0)
    _assert_uv(rebuilt_patch.chain_placements[1].points[-1][1], 2.0, 1.0)
    _assert_uv(rebuilt_patch.chain_placements[2].points[-1][1], 0.0, 1.0)


def test_apply_skeleton_solve_corrects_drifted_rectangle():
    graph = PatchGraph(
        nodes={
            71: _make_rectangle_patch(
                71,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (810,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (811,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (812,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (813,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            )
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=0,
        root_patch_id=71,
        patches={
            71: _make_patch_placement(
                71,
                0,
                [
                    _make_chain_placement(71, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (2.0, 0.0)]),
                    _make_chain_placement(71, 0, 1, FrameRole.V_FRAME, [(2.2, 0.0), (2.2, 1.15)]),
                    _make_chain_placement(71, 0, 2, FrameRole.H_FRAME, [(2.2, 1.15), (0.0, 1.15)]),
                    _make_chain_placement(71, 0, 3, FrameRole.V_FRAME, [(0.0, 1.15), (0.0, 0.0)]),
                ],
            )
        },
    )

    report = apply_skeleton_solve(
        SkeletonSolveInput(
            graph=graph,
            derived_topology=derived_topology,
            quilt_scaffold=quilt_scaffold,
            final_scale=1.0,
        )
    )

    rebuilt_patch = report.quilt_scaffold.patches[71]
    assert report.applied is True
    assert report.residual_u == pytest.approx(0.0)
    assert report.residual_v == pytest.approx(0.0)
    _assert_uv(rebuilt_patch.chain_placements[1].points[0][1], 2.0, 0.0)
    _assert_uv(rebuilt_patch.chain_placements[1].points[-1][1], 2.0, 1.0)
    _assert_uv(rebuilt_patch.chain_placements[2].points[0][1], 2.0, 1.0)
    _assert_uv(rebuilt_patch.chain_placements[2].points[-1][1], 0.0, 1.0)
    assert rebuilt_patch.closure_error == pytest.approx(0.0)


def test_apply_skeleton_solve_aligns_shared_vertical_seam_across_two_patches():
    graph = PatchGraph(
        nodes={
            72: _make_rectangle_patch(
                72,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (820,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (821,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (822,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (823,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
            73: _make_rectangle_patch(
                73,
                [
                    (1, (1.0, 0.0, 0.0)),
                    (4, (2.0, 0.0, 0.0)),
                    (5, (2.0, 1.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (1, 4), "edge_indices": (830,), "vert_cos": ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (4, 5), "edge_indices": (831,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (5, 2), "edge_indices": (832,), "vert_cos": ((2.0, 1.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (2, 1), "edge_indices": (821,), "vert_cos": ((1.0, 1.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=0,
        root_patch_id=72,
        patches={
            72: _make_patch_placement(
                72,
                0,
                [
                    _make_chain_placement(72, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (1.0, 0.0)]),
                    _make_chain_placement(72, 0, 1, FrameRole.V_FRAME, [(1.0, 0.0), (1.0, 1.0)]),
                    _make_chain_placement(72, 0, 2, FrameRole.H_FRAME, [(1.0, 1.0), (0.0, 1.0)]),
                    _make_chain_placement(72, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                ],
            ),
            73: _make_patch_placement(
                73,
                0,
                [
                    _make_chain_placement(73, 0, 0, FrameRole.H_FRAME, [(1.12, 0.0), (2.12, 0.0)]),
                    _make_chain_placement(73, 0, 1, FrameRole.V_FRAME, [(2.12, 0.0), (2.12, 1.05)]),
                    _make_chain_placement(73, 0, 2, FrameRole.H_FRAME, [(2.12, 1.05), (1.12, 1.05)]),
                    _make_chain_placement(73, 0, 3, FrameRole.V_FRAME, [(1.12, 1.05), (1.12, 0.0)]),
                ],
            ),
        },
    )

    report = apply_skeleton_solve(
        SkeletonSolveInput(
            graph=graph,
            derived_topology=derived_topology,
            quilt_scaffold=quilt_scaffold,
            final_scale=1.0,
        )
    )

    patch_left = report.quilt_scaffold.patches[72]
    patch_right = report.quilt_scaffold.patches[73]
    assert report.applied is True
    assert report.residual_u == pytest.approx(0.0)
    assert report.residual_v == pytest.approx(0.0)
    _assert_uv(patch_left.chain_placements[1].points[0][1], 1.0, 0.0)
    _assert_uv(patch_left.chain_placements[1].points[-1][1], 1.0, 1.0)
    _assert_uv(patch_right.chain_placements[3].points[0][1], 1.0, 1.0)
    _assert_uv(patch_right.chain_placements[3].points[-1][1], 1.0, 0.0)
    assert report.junctions_by_vert_index[1].canonical_u == pytest.approx(1.0)
    assert report.junctions_by_vert_index[2].canonical_u == pytest.approx(1.0)


def test_apply_skeleton_solve_to_scaffold_map_attaches_report_and_updates_quilt():
    graph = PatchGraph(
        nodes={
            80: _make_rectangle_patch(
                80,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (900,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (901,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (902,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (903,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    scaffold_map = ScaffoldMap(
        quilts=[
            ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=80,
                patches={
                    80: _make_patch_placement(
                        80,
                        0,
                        [
                            _make_chain_placement(80, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (2.0, 0.0)]),
                            _make_chain_placement(80, 0, 1, FrameRole.V_FRAME, [(2.2, 0.0), (2.2, 1.15)]),
                            _make_chain_placement(80, 0, 2, FrameRole.H_FRAME, [(2.2, 1.15), (0.0, 1.15)]),
                            _make_chain_placement(80, 0, 3, FrameRole.V_FRAME, [(0.0, 1.15), (0.0, 0.0)]),
                        ],
                    ),
                },
            )
        ]
    )

    updated_scaffold_map, reports = apply_skeleton_solve_to_scaffold_map(
        graph,
        derived_topology,
        scaffold_map,
        final_scale=1.0,
    )

    assert len(reports) == 1
    updated_quilt = updated_scaffold_map.quilts[0]
    assert updated_quilt.skeleton_solve_report is reports[0]
    assert reports[0].applied is True
    rebuilt_patch = updated_quilt.patches[80]
    _assert_uv(rebuilt_patch.chain_placements[1].points[0][1], 2.0, 0.0)
    _assert_uv(rebuilt_patch.chain_placements[1].points[-1][1], 2.0, 1.0)


def test_solve_reporting_surfaces_skeleton_summary_lines():
    graph = PatchGraph(
        nodes={
            81: _make_rectangle_patch(
                81,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (910,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (911,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (912,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (913,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    scaffold_map, reports = apply_skeleton_solve_to_scaffold_map(
        graph,
        derived_topology,
        ScaffoldMap(
            quilts=[
                ScaffoldQuiltPlacement(
                    quilt_index=0,
                    root_patch_id=81,
                    patches={
                        81: _make_patch_placement(
                            81,
                            0,
                            [
                                _make_chain_placement(81, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (2.0, 0.0)]),
                                _make_chain_placement(81, 0, 1, FrameRole.V_FRAME, [(2.2, 0.0), (2.2, 1.15)]),
                                _make_chain_placement(81, 0, 2, FrameRole.H_FRAME, [(2.2, 1.15), (0.0, 1.15)]),
                                _make_chain_placement(81, 0, 3, FrameRole.V_FRAME, [(0.0, 1.15), (0.0, 0.0)]),
                            ],
                        ),
                    },
                )
            ]
        ),
        final_scale=1.0,
    )

    assert reports[0].notes == ("skeleton_solve_applied",)

    root_report = format_root_scaffold_report(graph, scaffold_map)
    regression_report = format_regression_snapshot_report(None, graph, None, scaffold_map)

    assert any("Skeleton:" in line and "rows:2" in line and "cols:2" in line for line in root_report.lines)
    assert any(line.startswith("skeleton: applied:Y") for line in regression_report.lines)


def test_apply_skeleton_solve_respects_final_scale_in_uv_lengths():
    graph = PatchGraph(
        nodes={
            82: _make_rectangle_patch(
                82,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (920,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (921,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (922,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (923,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    report = apply_skeleton_solve(
        SkeletonSolveInput(
            graph=graph,
            derived_topology=derived_topology,
            quilt_scaffold=ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=82,
                patches={
                    82: _make_patch_placement(
                        82,
                        0,
                        [
                            _make_chain_placement(82, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (0.5, 0.0)]),
                            _make_chain_placement(82, 0, 1, FrameRole.V_FRAME, [(0.55, 0.0), (0.55, 0.30)]),
                            _make_chain_placement(82, 0, 2, FrameRole.H_FRAME, [(0.55, 0.30), (0.0, 0.30)]),
                            _make_chain_placement(82, 0, 3, FrameRole.V_FRAME, [(0.0, 0.30), (0.0, 0.0)]),
                        ],
                    ),
                },
            ),
            final_scale=0.25,
        )
    )

    rebuilt_patch = report.quilt_scaffold.patches[82]
    assert report.applied is True
    _assert_uv(rebuilt_patch.chain_placements[0].points[-1][1], 0.5, 0.0)
    _assert_uv(rebuilt_patch.chain_placements[1].points[0][1], 0.5, 0.0)
    _assert_uv(rebuilt_patch.chain_placements[1].points[-1][1], 0.5, 0.25)
    _assert_uv(rebuilt_patch.chain_placements[2].points[0][1], 0.5, 0.25)


def test_apply_skeleton_solve_to_scaffold_map_skips_non_tree_closure_quilt():
    graph = PatchGraph(
        nodes={
            83: _make_rectangle_patch(
                83,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (930,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (931,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (932,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (933,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
                patch_type=PatchType.FLOOR,
                world_facing=WorldFacing.UP,
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    original_scaffold_map = ScaffoldMap(
        quilts=[
            ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=83,
                patches={
                    83: _make_patch_placement(
                        83,
                        0,
                        [
                            _make_chain_placement(83, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (1.0, 0.0)]),
                            _make_chain_placement(83, 0, 1, FrameRole.V_FRAME, [(1.15, 0.0), (1.15, 1.0)]),
                            _make_chain_placement(83, 0, 2, FrameRole.H_FRAME, [(1.15, 1.0), (0.0, 1.0)]),
                            _make_chain_placement(83, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                        ],
                    ),
                },
            )
        ]
    )

    solve_plan = SolvePlan(
        quilts=[
            QuiltPlan(
                quilt_index=0,
                component_index=0,
                root_patch_id=83,
                root_score=1.0,
                steps=[QuiltStep(step_index=0, patch_id=83, is_root=True)],
                seam_relation_by_edge={
                    (83, 84): SeamRelationProfile(edge_key=(83, 84)),
                },
            )
        ]
    )

    updated_scaffold_map, reports = apply_skeleton_solve_to_scaffold_map(
        graph,
        derived_topology,
        original_scaffold_map,
        solve_plan=solve_plan,
        final_scale=1.0,
    )

    assert len(reports) == 1
    assert reports[0].applied is False
    assert reports[0].notes == ("skeleton_skip_non_tree_closure",)
    updated_patch = updated_scaffold_map.quilts[0].patches[83]
    _assert_uv(updated_patch.chain_placements[1].points[0][1], 1.15, 0.0)
    _assert_uv(updated_patch.chain_placements[1].points[-1][1], 1.15, 1.0)


def test_apply_skeleton_solve_to_scaffold_map_allows_non_tree_closure_for_wall_quilt():
    graph = PatchGraph(
        nodes={
            88: _make_rectangle_patch(
                88,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (980,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (981,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (982,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (983,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    original_scaffold_map = ScaffoldMap(
        quilts=[
            ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=88,
                patches={
                    88: _make_patch_placement(
                        88,
                        0,
                        [
                            _make_chain_placement(88, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (1.0, 0.0)]),
                            _make_chain_placement(88, 0, 1, FrameRole.V_FRAME, [(1.0, 0.0), (1.0, 1.0)]),
                            _make_chain_placement(88, 0, 2, FrameRole.H_FRAME, [(1.0, 1.0), (0.0, 1.0)]),
                            _make_chain_placement(88, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                        ],
                    ),
                },
            )
        ]
    )
    solve_plan = SolvePlan(
        quilts=[
            QuiltPlan(
                quilt_index=0,
                component_index=0,
                root_patch_id=88,
                root_score=1.0,
                steps=[QuiltStep(step_index=0, patch_id=88, is_root=True)],
                seam_relation_by_edge={
                    (88, 89): SeamRelationProfile(edge_key=(88, 89)),
                },
            )
        ]
    )

    updated_scaffold_map, reports = apply_skeleton_solve_to_scaffold_map(
        graph,
        derived_topology,
        original_scaffold_map,
        solve_plan=solve_plan,
        final_scale=1.0,
    )

    assert len(reports) == 1
    assert reports[0].applied is True
    assert reports[0].notes == ("skeleton_solve_applied",)
    updated_patch = updated_scaffold_map.quilts[0].patches[88]
    _assert_uv(updated_patch.chain_placements[1].points[0][1], 1.0, 0.0)
    _assert_uv(updated_patch.chain_placements[1].points[-1][1], 1.0, 1.0)


def test_apply_skeleton_solve_to_scaffold_map_skips_runtime_shape_quilt():
    graph = PatchGraph(
        nodes={
            90: _make_rectangle_patch(
                90,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (994,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (995,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (996,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (997,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = replace(
        _build_derived_topology_for_graph(graph),
        patch_shape_classes={90: PatchShapeClass.BAND},
    )
    original_scaffold_map = ScaffoldMap(
        quilts=[
            ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=90,
                patches={
                    90: _make_patch_placement(
                        90,
                        0,
                        [
                            _make_chain_placement(90, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (1.0, 0.0)]),
                            _make_chain_placement(90, 0, 1, FrameRole.V_FRAME, [(1.15, 0.0), (1.15, 1.0)]),
                            _make_chain_placement(90, 0, 2, FrameRole.H_FRAME, [(1.15, 1.0), (0.0, 1.0)]),
                            _make_chain_placement(90, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                        ],
                    ),
                },
            )
        ]
    )

    updated_scaffold_map, reports = apply_skeleton_solve_to_scaffold_map(
        graph,
        derived_topology,
        original_scaffold_map,
        final_scale=1.0,
    )

    assert len(reports) == 1
    assert reports[0].applied is False
    assert reports[0].notes == ("skeleton_skip_runtime_shape_quilt",)
    updated_patch = updated_scaffold_map.quilts[0].patches[90]
    _assert_uv(updated_patch.chain_placements[1].points[0][1], 1.15, 0.0)
    _assert_uv(updated_patch.chain_placements[1].points[-1][1], 1.15, 1.0)


def test_apply_skeleton_solve_to_scaffold_map_passthrough_runtime_shape_patch():
    graph = PatchGraph(
        nodes={
            100: _make_rectangle_patch(
                100,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (1100,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (1101,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (1102,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (1103,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
            101: _make_rectangle_patch(
                101,
                [
                    (1, (1.0, 0.0, 0.0)),
                    (4, (2.0, 0.0, 0.0)),
                    (5, (2.0, 1.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (1, 4), "edge_indices": (1110,), "vert_cos": ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (4, 5), "edge_indices": (1111,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (5, 2), "edge_indices": (1112,), "vert_cos": ((2.0, 1.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (2, 1), "edge_indices": (1101,), "vert_cos": ((1.0, 1.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = replace(
        _build_derived_topology_for_graph(graph),
        patch_shape_classes={100: PatchShapeClass.MIX, 101: PatchShapeClass.BAND},
    )
    original_scaffold_map = ScaffoldMap(
        quilts=[
            ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=100,
                patches={
                    100: _make_patch_placement(
                        100,
                        0,
                        [
                            _make_chain_placement(100, 0, 0, FrameRole.H_FRAME, [(0.2, 0.0), (1.2, 0.0)]),
                            _make_chain_placement(100, 0, 1, FrameRole.V_FRAME, [(1.2, 0.0), (1.2, 1.0)]),
                            _make_chain_placement(100, 0, 2, FrameRole.H_FRAME, [(1.2, 1.0), (0.2, 1.0)]),
                            _make_chain_placement(100, 0, 3, FrameRole.V_FRAME, [(0.2, 1.0), (0.2, 0.0)]),
                        ],
                    ),
                    101: _make_patch_placement(
                        101,
                        0,
                        [
                            _make_chain_placement(101, 0, 0, FrameRole.H_FRAME, [(1.0, 0.0), (2.0, 0.0)]),
                            _make_chain_placement(101, 0, 1, FrameRole.V_FRAME, [(2.0, 0.0), (2.0, 1.0)]),
                            _make_chain_placement(101, 0, 2, FrameRole.H_FRAME, [(2.0, 1.0), (1.0, 1.0)]),
                            _make_chain_placement(101, 0, 3, FrameRole.V_FRAME, [(1.0, 1.0), (1.0, 0.0)]),
                        ],
                    ),
                },
            )
        ]
    )

    updated_scaffold_map, reports = apply_skeleton_solve_to_scaffold_map(
        graph,
        derived_topology,
        original_scaffold_map,
        final_scale=1.0,
    )

    assert len(reports) == 1
    assert reports[0].applied is True
    assert "skeleton_runtime_shape_passthrough" in reports[0].notes

    moved_patch = updated_scaffold_map.quilts[0].patches[100]
    passthrough_patch = updated_scaffold_map.quilts[0].patches[101]
    _assert_uv(moved_patch.chain_placements[1].points[0][1], 1.0, 0.0)
    _assert_uv(moved_patch.chain_placements[1].points[-1][1], 1.0, 1.0)
    _assert_uv(moved_patch.chain_placements[3].points[0][1], 0.0, 1.0)
    _assert_uv(moved_patch.chain_placements[3].points[-1][1], 0.0, 0.0)
    _assert_uv(passthrough_patch.chain_placements[3].points[0][1], 1.0, 1.0)
    _assert_uv(passthrough_patch.chain_placements[3].points[-1][1], 1.0, 0.0)


def test_build_skeleton_graphs_uses_runtime_chain_roles_for_band_passthrough():
    graph = PatchGraph(
        nodes={
            102: _make_rectangle_patch(
                102,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (1120,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (1121,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.FREE},
                    {"vert_indices": (2, 3), "edge_indices": (1122,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (1123,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.FREE},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=0,
        root_patch_id=102,
        patches={
            102: _make_patch_placement(
                102,
                0,
                [
                    _make_chain_placement(102, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (1.0, 0.0)]),
                    _make_chain_placement(102, 0, 1, FrameRole.V_FRAME, [(1.0, 0.0), (1.0, 1.0)]),
                    _make_chain_placement(102, 0, 2, FrameRole.H_FRAME, [(1.0, 1.0), (0.0, 1.0)]),
                    _make_chain_placement(102, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                ],
            ),
        },
    )
    chain_role_by_ref = solve_skeleton_module._collect_chain_role_overrides(
        quilt_scaffold,
        allowed_patch_ids={102},
    )

    skeleton_graphs = build_skeleton_graphs(
        graph,
        derived_topology,
        allowed_patch_ids={102},
        chain_role_by_ref=chain_role_by_ref,
    )
    sibling_groups = build_sibling_groups(
        graph,
        skeleton_graphs,
        allowed_patch_ids={102},
        chain_role_by_ref=chain_role_by_ref,
    )

    assert len(skeleton_graphs.col_component_members) == 2
    assert any(
        group.patch_id == 102 and group.role == FrameRole.V_FRAME
        for group in sibling_groups
    )


def test_apply_skeleton_solve_to_scaffold_map_reverts_high_residual_wall_cycle(monkeypatch):
    graph = PatchGraph(
        nodes={
            89: _make_rectangle_patch(
                89,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (1.0, 0.0, 0.0)),
                    (2, (1.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (990,), "vert_cos": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (991,), "vert_cos": ((1.0, 0.0, 0.0), (1.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (992,), "vert_cos": ((1.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (993,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    original_scaffold_map = ScaffoldMap(
        quilts=[
            ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=89,
                patches={
                    89: _make_patch_placement(
                        89,
                        0,
                        [
                            _make_chain_placement(89, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (1.0, 0.0)]),
                            _make_chain_placement(89, 0, 1, FrameRole.V_FRAME, [(1.0, 0.0), (1.0, 1.0)]),
                            _make_chain_placement(89, 0, 2, FrameRole.H_FRAME, [(1.0, 1.0), (0.0, 1.0)]),
                            _make_chain_placement(89, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                        ],
                    ),
                },
            )
        ]
    )
    solve_plan = SolvePlan(
        quilts=[
            QuiltPlan(
                quilt_index=0,
                component_index=0,
                root_patch_id=89,
                root_score=1.0,
                steps=[QuiltStep(step_index=0, patch_id=89, is_root=True)],
                seam_relation_by_edge={
                    (89, 90): SeamRelationProfile(edge_key=(89, 90)),
                },
            )
        ]
    )

    def _fake_apply_skeleton_solve(_solve_input, diagnostics=None):
        _ = diagnostics
        return solve_skeleton_module.SkeletonSolveReport(
            applied=True,
            residual_u=1.0,
            residual_v=0.02,
            notes=("skeleton_residual_u_warn", "skeleton_residual_v_warn"),
            quilt_scaffold=replace(original_scaffold_map.quilts[0]),
        )

    monkeypatch.setattr(solve_skeleton_module, "apply_skeleton_solve", _fake_apply_skeleton_solve)

    updated_scaffold_map, reports = solve_skeleton_module.apply_skeleton_solve_to_scaffold_map(
        graph,
        derived_topology,
        original_scaffold_map,
        solve_plan=solve_plan,
        final_scale=1.0,
    )

    assert len(reports) == 1
    assert reports[0].applied is False
    assert reports[0].notes == ("skeleton_skip_cycle_high_residual",)
    updated_patch = updated_scaffold_map.quilts[0].patches[89]
    _assert_uv(updated_patch.chain_placements[1].points[0][1], 1.0, 0.0)
    _assert_uv(updated_patch.chain_placements[1].points[-1][1], 1.0, 1.0)


def test_apply_skeleton_solve_is_quilt_local_over_multi_quilt_graph():
    graph = PatchGraph(
        nodes={
            84: _make_rectangle_patch(
                84,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (940,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (941,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (942,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (943,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
            85: _make_rectangle_patch(
                85,
                [
                    (10, (4.0, 0.0, 0.0)),
                    (11, (6.0, 0.0, 0.0)),
                    (12, (6.0, 1.0, 0.0)),
                    (13, (4.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (10, 11), "edge_indices": (950,), "vert_cos": ((4.0, 0.0, 0.0), (6.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (11, 12), "edge_indices": (951,), "vert_cos": ((6.0, 0.0, 0.0), (6.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (12, 13), "edge_indices": (952,), "vert_cos": ((6.0, 1.0, 0.0), (4.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (13, 10), "edge_indices": (953,), "vert_cos": ((4.0, 1.0, 0.0), (4.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)

    report = apply_skeleton_solve(
        SkeletonSolveInput(
            graph=graph,
            derived_topology=derived_topology,
            quilt_scaffold=ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=84,
                patches={
                    84: _make_patch_placement(
                        84,
                        0,
                        [
                            _make_chain_placement(84, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (2.0, 0.0)]),
                            _make_chain_placement(84, 0, 1, FrameRole.V_FRAME, [(2.1, 0.0), (2.1, 1.0)]),
                            _make_chain_placement(84, 0, 2, FrameRole.H_FRAME, [(2.1, 1.0), (0.0, 1.0)]),
                            _make_chain_placement(84, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                        ],
                    ),
                },
            ),
            final_scale=1.0,
        )
    )

    assert report.applied is True
    assert report.row_component_count == 2
    assert report.col_component_count == 2
    assert report.sibling_group_count == 2


def test_apply_skeleton_solve_skips_non_wall_quilt():
    graph = PatchGraph(
        nodes={
            86: _make_rectangle_patch(
                86,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (2.0, 0.0, 0.0)),
                    (2, (2.0, 1.0, 0.0)),
                    (3, (0.0, 1.0, 0.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (960,), "vert_cos": ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (961,), "vert_cos": ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (962,), "vert_cos": ((2.0, 1.0, 0.0), (0.0, 1.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (963,), "vert_cos": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
                patch_type=PatchType.FLOOR,
                world_facing=WorldFacing.UP,
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)
    quilt_scaffold = ScaffoldQuiltPlacement(
        quilt_index=0,
        root_patch_id=86,
        patches={
            86: _make_patch_placement(
                86,
                0,
                [
                    _make_chain_placement(86, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (2.0, 0.0)]),
                    _make_chain_placement(86, 0, 1, FrameRole.V_FRAME, [(2.0, 0.0), (2.0, 1.0)]),
                    _make_chain_placement(86, 0, 2, FrameRole.H_FRAME, [(2.0, 1.0), (0.0, 1.0)]),
                    _make_chain_placement(86, 0, 3, FrameRole.V_FRAME, [(0.0, 1.0), (0.0, 0.0)]),
                ],
            ),
        },
    )

    report = apply_skeleton_solve(
        SkeletonSolveInput(
            graph=graph,
            derived_topology=derived_topology,
            quilt_scaffold=quilt_scaffold,
            final_scale=1.0,
        )
    )

    assert report.applied is False
    assert report.notes == ("skeleton_skip_non_wall_quilt",)
    assert report.quilt_scaffold is quilt_scaffold


def test_apply_skeleton_solve_wall_row_safety_uses_world_z_not_y():
    graph = PatchGraph(
        nodes={
            87: _make_rectangle_patch(
                87,
                [
                    (0, (0.0, 0.0, 0.0)),
                    (1, (0.0, 2.0, 0.0)),
                    (2, (0.0, 2.0, 2.0)),
                    (3, (0.0, 0.0, 2.0)),
                ],
                [
                    {"vert_indices": (0, 1), "edge_indices": (970,), "vert_cos": ((0.0, 0.0, 0.0), (0.0, 2.0, 0.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (1, 2), "edge_indices": (971,), "vert_cos": ((0.0, 2.0, 0.0), (0.0, 2.0, 2.0)), "role": FrameRole.V_FRAME},
                    {"vert_indices": (2, 3), "edge_indices": (972,), "vert_cos": ((0.0, 2.0, 2.0), (0.0, 0.0, 2.0)), "role": FrameRole.H_FRAME},
                    {"vert_indices": (3, 0), "edge_indices": (973,), "vert_cos": ((0.0, 0.0, 2.0), (0.0, 0.0, 0.0)), "role": FrameRole.V_FRAME},
                ],
            ),
        }
    )
    derived_topology = _build_derived_topology_for_graph(graph)

    report = apply_skeleton_solve(
        SkeletonSolveInput(
            graph=graph,
            derived_topology=derived_topology,
            quilt_scaffold=ScaffoldQuiltPlacement(
                quilt_index=0,
                root_patch_id=87,
                patches={
                    87: _make_patch_placement(
                        87,
                        0,
                        [
                            _make_chain_placement(87, 0, 0, FrameRole.H_FRAME, [(0.0, 0.0), (2.0, 0.0)]),
                            _make_chain_placement(87, 0, 1, FrameRole.V_FRAME, [(2.1, 0.0), (2.1, 2.0)]),
                            _make_chain_placement(87, 0, 2, FrameRole.H_FRAME, [(2.1, 2.0), (0.0, 2.0)]),
                            _make_chain_placement(87, 0, 3, FrameRole.V_FRAME, [(0.0, 2.0), (0.0, 0.0)]),
                        ],
                    ),
                },
            ),
            final_scale=1.0,
        )
    )

    assert report.applied is True
    assert report.unconstrained_row_junction_count == 0
    assert all(
        not (junction.skeleton_flags & SkeletonFlags.UNCONSTRAINED_ROW)
        for junction in report.junctions
    )
