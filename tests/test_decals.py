from __future__ import annotations

from mathutils import Vector

from cftuv.decals import (
    _OrientedCornerRun,
    _OrientedRibbonRun,
    _boundary_wing_direction,
    _collect_manual_chain_decals,
    _collect_manual_edge_decals,
    _collect_trim_ribbon_runs,
    _collect_wall_pair_chains,
    _corner_offset_join,
    _corner_wing_directions,
    _dedupe_polyline,
    _junction_miter_position,
    _offset_plane_junction_center,
    _polyline_tangents,
    _prepare_seam_junctions,
    _ribbon_vertex_frames,
    _stitch_corner_runs,
    _stitch_ribbon_runs,
    _trim_run_for_junctions,
    _trim_quad_layout,
    _trim_quad_requires_flip,
    chain_refs_for_edge_indices,
)
from cftuv.model import (
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
    PatchType,
    DecalSettings,
)


def _make_wall_node(patch_id, normal, basis_v, chains):
    node = PatchNode(patch_id=patch_id, face_indices=[patch_id * 100])
    node.patch_type = PatchType.WALL
    node.normal = Vector(normal)
    node.basis_v = Vector(basis_v)
    node.boundary_loops = [BoundaryLoop(chains=chains)]
    return node


def _make_chain(
    vert_indices,
    vert_cos,
    neighbor_patch_id,
    edge_indices=None,
    dihedral_convexity=0.0,
    side_face_normals=None,
):
    chain = BoundaryChain(
        vert_indices=list(vert_indices),
        vert_cos=[Vector(co) for co in vert_cos],
        edge_indices=(
            list(edge_indices)
            if edge_indices is not None
            else list(range(len(vert_indices) - 1))
        ),
        neighbor_patch_id=neighbor_patch_id,
        side_face_normals=[Vector(normal) for normal in (side_face_normals or ())],
    )
    chain.dihedral_convexity = dihedral_convexity
    return chain


def _make_graph(*nodes):
    graph = PatchGraph()
    for node in nodes:
        graph.add_node(node)
    return graph


def _make_ribbon_run(start, end, point_a, point_b, normal=(0, 1, 0)):
    return _OrientedRibbonRun(
        vert_indices=[start, end],
        points=[Vector(point_a), Vector(point_b)],
        segment_normals=[Vector(normal)],
        segment_ups=[Vector((0, 0, 1))],
        segment_chain_refs=[(start, 0, 0)],
    )


def _make_corner_run(
    start,
    end,
    point_a,
    point_b,
    edge_index,
    normal_a=(0, 1, 0),
    normal_b=(0, 0, 1),
):
    return _OrientedCornerRun(
        vert_indices=[start, end],
        points=[Vector(point_a), Vector(point_b)],
        segment_normals_a=[Vector(normal_a)],
        segment_normals_b=[Vector(normal_b)],
        segment_convexities=[1.0],
        segment_edge_indices=[edge_index],
    )


def _ribbon_edges(runs):
    return [
        (run.vert_indices[index], run.vert_indices[index + 1])
        for run in runs
        for index in range(len(run.vert_indices) - 1)
    ]


class TestCollectTrimSegments:
    def test_uses_local_owner_face_normal_instead_of_patch_average(self):
        # Один wrapped WALL patch может содержать стены с разными нормалями.
        # Средняя +Y ошибочно объявила бы нижнее +X ребро верхним; локальная
        # owner-face normal -Y правильно оставляет его в BOTTOM.
        chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (1, 0, 0)],
            -1,
            side_face_normals=[(0, -1, 0)],
        )
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain])
        graph = _make_graph(wall)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == []
        assert _ribbon_edges(bottom_runs) == [(0, 1)]
        assert bottom_runs[0].segment_normals[0].y == -1.0

    def test_top_bottom_classification(self):
        # Стена с нормалью +Y, вертикаль +Z. Верхняя кромка идёт по +X
        # (outward = d x n = +Z), нижняя — по -X (outward = -Z).
        top_chain = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        bottom_chain = _make_chain([2, 3], [(1, 0, 0), (0, 0, 0)], -1)
        side_chain = _make_chain([1, 2], [(1, 0, 1), (1, 0, 0)], -1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [top_chain, bottom_chain, side_chain])
        graph = _make_graph(wall)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert _ribbon_edges(top_runs) == [(0, 1)]
        assert _ribbon_edges(bottom_runs) == [(2, 3)]
        normal = top_runs[0].segment_normals[0]
        up = top_runs[0].segment_ups[0]
        assert abs(normal.y - 1.0) < 1e-6
        assert abs(up.z - 1.0) < 1e-6

    def test_wall_wall_chains_excluded(self):
        # Кромка к WALL соседу — под corner/seam декали, в тримы не идёт.
        # Кромка к FLOOR соседу — идёт.
        to_wall = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], 1)
        to_floor = _make_chain([2, 3], [(1, 0, 0), (0, 0, 0)], 2)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [to_wall, to_floor])

        other_wall = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [])
        floor = PatchNode(patch_id=2, face_indices=[200])
        floor.patch_type = PatchType.FLOOR
        graph = _make_graph(wall, other_wall, floor)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == []
        assert _ribbon_edges(bottom_runs) == [(2, 3)]

    def test_non_wall_patches_ignored(self):
        chain = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        floor = PatchNode(patch_id=0, face_indices=[0])
        floor.patch_type = PatchType.FLOOR
        floor.boundary_loops = [BoundaryLoop(chains=[chain])]
        graph = _make_graph(floor)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == [] and bottom_runs == []

    def test_short_edges_skipped(self):
        chain = _make_chain([0, 1], [(0, 0, 1), (0.01, 0, 1)], -1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain])
        graph = _make_graph(wall)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        assert top_runs == [] and bottom_runs == []

    def test_selected_edge_enables_its_complete_chain(self):
        top_chain = _make_chain(
            [0, 1, 2],
            [(0, 0, 1), (1, 0, 1), (2, 0, 1)],
            -1,
            edge_indices=[10, 11],
        )
        other_top_chain = _make_chain(
            [3, 4],
            [(3, 0, 1), (4, 0, 1)],
            -1,
            edge_indices=[12],
        )
        wall = _make_wall_node(
            0,
            (0, 1, 0),
            (0, 0, 1),
            [top_chain, other_top_chain],
        )
        graph = _make_graph(wall)

        chain_refs = chain_refs_for_edge_indices(graph, [11])
        top_runs, bottom_runs = _collect_trim_ribbon_runs(
            graph, chain_refs=chain_refs
        )

        assert chain_refs == {(0, 0, 0)}
        assert _ribbon_edges(top_runs) == [(0, 1), (1, 2)]
        assert bottom_runs == []


class TestOrientedRibbonRuns:
    def test_mixed_chain_directions_form_one_closed_run(self):
        runs = [
            _make_ribbon_run(0, 1, (0, 0, 0), (1, 0, 0), (0, 1, 0)),
            _make_ribbon_run(2, 1, (1, 1, 0), (1, 0, 0), (1, 0, 0)),
            _make_ribbon_run(2, 3, (1, 1, 0), (0, 1, 0), (0, -1, 0)),
            _make_ribbon_run(0, 3, (0, 0, 0), (0, 1, 0), (-1, 0, 0)),
        ]

        stitched = _stitch_ribbon_runs(runs)

        assert len(stitched) == 1
        ring = stitched[0]
        assert ring.is_closed
        assert ring.vert_indices == [0, 1, 2, 3, 0]
        assert len(ring.segment_normals) == 4
        frames = _ribbon_vertex_frames(ring)
        assert (frames[0][0] - frames[-1][0]).length < 1e-9

    def test_point_contact_branch_is_not_joined_arbitrarily(self):
        runs = [
            _make_ribbon_run(0, 1, (0, 0, 0), (1, 0, 0)),
            _make_ribbon_run(1, 2, (1, 0, 0), (2, 0, 0)),
            _make_ribbon_run(1, 3, (1, 0, 0), (1, 1, 0)),
        ]

        stitched = _stitch_ribbon_runs(runs)

        assert len(stitched) == 3
        assert all(len(run.vert_indices) == 2 for run in stitched)

    def test_reversed_segment_requests_winding_flip(self):
        down = Vector((0, 0, -1))
        desired_normal = Vector((0, 1, 0))

        assert not _trim_quad_requires_flip(
            Vector((0, 0, 0)), Vector((1, 0, 0)), down, down, desired_normal
        )
        assert _trim_quad_requires_flip(
            Vector((1, 0, 0)), Vector((0, 0, 0)), down, down, desired_normal
        )

    def test_winding_flip_does_not_swap_base_tip_uv(self):
        verts, uvs = _trim_quad_layout(
            "base_a", "base_b", "tip_a", "tip_b", 2.0, 3.0, 0.8, 1.0, True
        )

        uv_by_vert = dict(zip(verts, uvs))
        assert uv_by_vert["base_a"][1] == 0.8
        assert uv_by_vert["base_b"][1] == 0.8
        assert uv_by_vert["tip_a"][1] == 1.0
        assert uv_by_vert["tip_b"][1] == 1.0


class TestCollectWallPairChains:
    def test_paired_self_seam_on_wrapped_wall_becomes_corner(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[42],
            dihedral_convexity=0.5,
            side_face_normals=[(0, 1, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[42],
            dihedral_convexity=0.5,
            side_face_normals=[(1, 0, 0)],
        )
        wrapped_wall = _make_wall_node(
            0, (0.707, 0.707, 0), (0, 0, 1), [side_a, side_b]
        )
        graph = _make_graph(wrapped_wall)

        corner_chains, seam_chains = _collect_wall_pair_chains(graph)

        assert len(corner_chains) == 1
        assert seam_chains == []
        points, normal_a, normal_b, _closed, convexity = corner_chains[0]
        assert points == side_a.vert_cos
        assert normal_a.dot(Vector((0, 1, 0))) > 0.999
        assert normal_b.dot(Vector((1, 0, 0))) > 0.999
        assert convexity == 0.5

    def test_nearly_coplanar_self_seam_remains_corner(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(0, 1, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(0.05, 0.99875, 0)],
        )
        wrapped_wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [side_a, side_b])

        corner_chains, seam_chains = _collect_wall_pair_chains(
            _make_graph(wrapped_wall)
        )

        assert len(corner_chains) == 1
        assert seam_chains == []

    def test_corner_vs_seam_split_and_dedupe(self):
        # patch 0 | patch 1 — перпендикулярны (угол), patch 0 | patch 2 —
        # копланарны (шов). Обратные цепочки от соседей должны быть
        # отброшены дедупликацией (owner id < neighbor id).
        corner_chain = _make_chain([0, 1], [(0, 0, 0), (0, 0, 1)], 1)
        seam_chain = _make_chain([2, 3], [(1, 0, 0), (1, 0, 1)], 2)
        wall_0 = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [corner_chain, seam_chain])

        back_chain = _make_chain([1, 0], [(0, 0, 1), (0, 0, 0)], 0)
        wall_1 = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [back_chain])

        back_seam = _make_chain([3, 2], [(1, 0, 1), (1, 0, 0)], 0)
        wall_2 = _make_wall_node(2, (0, 1, 0), (0, 0, 1), [back_seam])

        graph = _make_graph(wall_0, wall_1, wall_2)
        corner_chains, seam_chains = _collect_wall_pair_chains(graph)

        assert len(corner_chains) == 1
        assert len(seam_chains) == 1
        corner_points, normal_a, normal_b, _closed, _convexity = corner_chains[0]
        assert len(corner_points) == 2
        assert abs(normal_a.y - 1.0) < 1e-6
        assert abs(normal_b.x - 1.0) < 1e-6

    def test_wall_floor_pairs_ignored(self):
        chain = _make_chain([0, 1], [(0, 0, 0), (0, 0, 1)], 1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain])
        floor = PatchNode(patch_id=1, face_indices=[100])
        floor.patch_type = PatchType.FLOOR
        graph = _make_graph(wall, floor)

        corner_chains, seam_chains = _collect_wall_pair_chains(graph)

        assert corner_chains == [] and seam_chains == []

    def test_selected_shared_edge_captures_both_chain_sides(self):
        owner_chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (0, 0, 1)],
            1,
            edge_indices=[42],
        )
        neighbor_chain = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, 0)],
            0,
            edge_indices=[42],
        )
        wall_0 = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [owner_chain])
        wall_1 = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [neighbor_chain])
        graph = _make_graph(wall_0, wall_1)

        chain_refs = chain_refs_for_edge_indices(graph, [42])
        corner_chains, seam_chains = _collect_wall_pair_chains(
            graph, chain_refs=chain_refs
        )

        assert chain_refs == {(0, 0, 0), (1, 0, 0)}
        assert len(corner_chains) == 1
        assert seam_chains == []

    def test_unselected_wall_pair_is_filtered_out(self):
        first_chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (0, 0, 1)],
            1,
            edge_indices=[20],
        )
        selected_chain = _make_chain(
            [2, 3],
            [(1, 0, 0), (1, 0, 1)],
            2,
            edge_indices=[30],
        )
        wall_0 = _make_wall_node(
            0,
            (0, 1, 0),
            (0, 0, 1),
            [first_chain, selected_chain],
        )
        wall_1 = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [])
        wall_2 = _make_wall_node(2, (0, 1, 0), (0, 0, 1), [])
        graph = _make_graph(wall_0, wall_1, wall_2)

        chain_refs = chain_refs_for_edge_indices(graph, [30])
        corner_chains, seam_chains = _collect_wall_pair_chains(
            graph, chain_refs=chain_refs
        )

        assert corner_chains == []
        assert len(seam_chains) == 1


class TestManualChainDecals:
    def test_selected_edges_are_atomic_across_asymmetric_chain_splits(self):
        wall_chain = _make_chain(
            [0, 1, 2, 3],
            [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)],
            1,
            edge_indices=[5, 14, 20],
            side_face_normals=[(0, 1, 0), (0, 1, 0), (0, 1, 0)],
        )
        top_edge_a = _make_chain(
            [1, 0],
            [(1, 0, 0), (0, 0, 0)],
            0,
            edge_indices=[5],
            side_face_normals=[(0, 0, 1)],
        )
        top_edge_b = _make_chain(
            [2, 1],
            [(2, 0, 0), (1, 0, 0)],
            0,
            edge_indices=[14],
            side_face_normals=[(0, 0, 1)],
        )
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [wall_chain])
        top = _make_wall_node(
            1, (0, 0, 1), (0, 1, 0), [top_edge_a, top_edge_b]
        )
        top.patch_type = PatchType.FLOOR

        paired_runs, boundary_edges = _collect_manual_edge_decals(
            _make_graph(wall, top), [5, 14]
        )

        assert len(paired_runs) == 1
        assert boundary_edges == []
        run = paired_runs[0]
        assert [tuple(point) for point in run.points] == [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
        ]
        assert run.segment_edge_indices == [5, 14]
        assert all(
            normal.dot(Vector((0, 1, 0))) > 0.999
            for normal in run.segment_normals_a
        )
        assert all(
            normal.dot(Vector((0, 0, 1))) > 0.999
            for normal in run.segment_normals_b
        )

    def test_selected_self_seam_pairs_two_uses_of_one_edge(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[35],
            side_face_normals=[(1, 0, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[35],
            side_face_normals=[(0, -1, 0)],
        )
        wall = _make_wall_node(0, (1, -1, 0), (0, 0, 1), [side_a, side_b])

        paired_runs, boundary_edges = _collect_manual_edge_decals(
            _make_graph(wall), [35]
        )

        assert len(paired_runs) == 1
        assert abs(paired_runs[0].segment_convexities[0]) > 0.99
        assert boundary_edges == []

    def test_corner_runs_preserve_surface_sides_across_chain_splits(self):
        first = _make_corner_run(0, 1, (0, 0, 0), (1, 0, 0), 5)
        second = _make_corner_run(
            1,
            2,
            (1, 0, 0),
            (2, 0.5, 0),
            14,
            normal_a=(0, 0, 1),
            normal_b=(0, 1, 0),
        )

        stitched = _stitch_corner_runs([first, second])

        assert len(stitched) == 1
        run = stitched[0]
        assert run.segment_edge_indices == [5, 14]
        assert all(normal.y > 0.99 for normal in run.segment_normals_a)
        assert all(normal.z > 0.99 for normal in run.segment_normals_b)

    def test_corner_runs_stop_at_valence_three_junction(self):
        runs = [
            _make_corner_run(0, 1, (0, 0, 0), (1, 0, 0), 5),
            _make_corner_run(1, 2, (1, 0, 0), (2, 0, 0), 14),
            _make_corner_run(1, 3, (1, 0, 0), (1, 1, 0), 35),
        ]

        stitched = _stitch_corner_runs(runs)

        assert len(stitched) == 3
        assert all(len(run.segment_edge_indices) == 1 for run in stitched)

    def test_t_junction_prepares_equal_branch_cuts_for_seam_patch(self):
        runs = _stitch_corner_runs(
            [
                _make_corner_run(
                    1,
                    2,
                    (0, 0, 0),
                    (2, 0, 0),
                    5,
                    normal_a=(0, 0, 1),
                    normal_b=(0, 0, 1),
                ),
                _make_corner_run(
                    1,
                    3,
                    (0, 0, 0),
                    (-2, 0, 0),
                    14,
                    normal_a=(0, 0, 1),
                    normal_b=(0, 0, 1),
                ),
                _make_corner_run(
                    1,
                    4,
                    (0, 0, 0),
                    (0, 2, 0),
                    35,
                    normal_a=(0, 0, 1),
                    normal_b=(0, 0, 1),
                ),
            ]
        )
        settings = DecalSettings(width_seam=0.2, offset=0.02)

        specs, cuts = _prepare_seam_junctions(runs, settings, 0.1)

        assert set(specs) == {1}
        assert len(cuts) == 3
        assert all(abs(cut - 0.1) < 1e-6 for cut in cuts.values())
        assert (specs[1].core_pos - Vector((0, 0, 0.02))).length < 1e-6

        trimmed = _trim_run_for_junctions(runs[0], start_cut=cuts[(0, True)])
        assert (trimmed.points[0] - Vector((0.1, 0, 0))).length < 1e-6
        assert trimmed.vert_indices == runs[0].vert_indices

    def test_trihedral_offset_planes_share_one_junction_core(self):
        center = _offset_plane_junction_center(
            Vector((0, 0, 0)),
            [
                Vector((1, 0, 0)),
                Vector((0, 1, 0)),
                Vector((0, 0, 1)),
            ],
            0.02,
        )

        assert (center - Vector((0.02, 0.02, 0.02))).length < 1e-6

    def test_junction_miter_intersects_outer_branch_contours(self):
        miter = _junction_miter_position(
            Vector((0.02, 0.02, 0.02)),
            Vector((0.12, -0.08, 0.02)),
            Vector((1, 0, 0)),
            Vector((-0.08, 0.12, 0.02)),
            Vector((0, 1, 0)),
            0.1,
        )

        assert (miter - Vector((-0.08, -0.08, 0.02))).length < 1e-6

    def test_junction_miter_averages_parallel_outer_contours(self):
        miter = _junction_miter_position(
            Vector((0, 0, 0)),
            Vector((-0.1, -0.1, 0)),
            Vector((-1, 0, 0)),
            Vector((0.1, -0.1, 0)),
            Vector((1, 0, 0)),
            0.1,
        )

        assert (miter - Vector((0, -0.1, 0))).length < 1e-6

    def test_selected_self_seam_uses_both_owner_sides(self):
        side_a = _make_chain(
            [0, 1],
            [(0, 0, -1), (0, 0, 1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(0, 1, 0)],
        )
        side_b = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, -1)],
            -2,
            edge_indices=[42],
            side_face_normals=[(1, 0, 0)],
        )
        wrapped_wall = _make_wall_node(
            0, (0.707, 0.707, 0), (0, 0, 1), [side_a, side_b]
        )
        graph = _make_graph(wrapped_wall)

        chain_refs = chain_refs_for_edge_indices(graph, [42])
        corner_chains, boundary_chains = _collect_manual_chain_decals(
            graph, chain_refs
        )

        assert chain_refs == {(0, 0, 0), (0, 0, 1)}
        assert len(corner_chains) == 1
        assert boundary_chains == []

    def test_patch_pair_ignores_semantic_patch_types(self):
        owner_chain = _make_chain(
            [0, 1],
            [(0, 0, 0), (0, 0, 1)],
            1,
            edge_indices=[42],
            dihedral_convexity=-1.0,
        )
        neighbor_chain = _make_chain(
            [1, 0],
            [(0, 0, 1), (0, 0, 0)],
            0,
            edge_indices=[42],
            dihedral_convexity=-1.0,
        )
        floor = _make_wall_node(0, (0, 0, 1), (0, 1, 0), [owner_chain])
        slope = _make_wall_node(1, (1, 0, 0), (0, 0, 1), [neighbor_chain])
        floor.patch_type = PatchType.FLOOR
        slope.patch_type = PatchType.SLOPE
        graph = _make_graph(floor, slope)

        chain_refs = chain_refs_for_edge_indices(graph, [42])
        corner_chains, boundary_chains = _collect_manual_chain_decals(
            graph, chain_refs
        )

        assert len(corner_chains) == 1
        assert corner_chains[0][4] == -1.0
        assert boundary_chains == []

    def test_mesh_boundary_becomes_flat_corner_width_decal(self):
        border_chain = _make_chain(
            [0, 1, 2],
            [(0, 0, 0), (1, 0, 0), (2, 0, 0)],
            -1,
            edge_indices=[7, 8],
        )
        floor = _make_wall_node(0, (0, 0, 1), (0, 1, 0), [border_chain])
        floor.patch_type = PatchType.FLOOR
        graph = _make_graph(floor)

        chain_refs = chain_refs_for_edge_indices(graph, [8])
        corner_chains, boundary_chains = _collect_manual_chain_decals(
            graph, chain_refs
        )

        assert corner_chains == []
        assert len(boundary_chains) == 1
        assert len(boundary_chains[0][0]) == 3

    def test_boundary_wing_points_inside_owner_patch(self):
        wing_dir = _boundary_wing_direction(
            Vector((1, 0, 0)),
            Vector((0, 0, 1)),
        )

        assert wing_dir is not None
        assert wing_dir.y > 0.0
        assert abs(wing_dir.x) < 1e-9


class TestCornerWingDirections:
    def test_planar_wings_follow_non_right_dihedral(self):
        # Две плоскости под 60/120 градусов вокруг вертикального seam.
        # Направления крыльев должны сохранить этот угол, а не стать 90°.
        wings = _corner_wing_directions(
            Vector((0, 0, 1)),
            Vector((0, -1, 0)),
            Vector((0.8660254038, -0.5, 0)),
            dihedral_convexity=1.0,
        )

        assert wings is not None
        wing_a, wing_b = wings
        assert abs(abs(wing_a.dot(wing_b)) - 0.5) < 1e-6
        assert abs(wing_a.dot(wing_b)) > 0.1  # явно не 90°

    def test_convex_wings_follow_patch_interiors(self):
        wings = _corner_wing_directions(
            Vector((0, 0, 1)),
            Vector((0, -1, 0)),
            Vector((1, 0, 0)),
            dihedral_convexity=1.0,
        )

        assert wings is not None
        wing_a, wing_b = wings
        assert wing_a.x < 0.0
        assert wing_b.y > 0.0

    def test_concave_wings_are_not_inverted(self):
        wings = _corner_wing_directions(
            Vector((0, 0, -1)),
            Vector((0, 1, 0)),
            Vector((-1, 0, 0)),
            dihedral_convexity=-1.0,
        )

        assert wings is not None
        wing_a, wing_b = wings
        assert wing_a.x < 0.0
        assert wing_b.y > 0.0


class TestCornerOffsetJoin:
    def test_right_angle_miter_preserves_width_to_both_segments(self):
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            Vector((0, 1, 0)),
            Vector((-1, 0, 0)),
            1.0,
        )

        assert not join.is_bevel
        assert (join.incoming - join.outgoing).length < 1e-9
        assert abs(join.incoming.x + 1.0) < 1e-6
        assert abs(join.incoming.y - 1.0) < 1e-6

    def test_straight_offset_remains_one_shared_point(self):
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            0.5,
        )

        assert not join.is_bevel
        assert (join.incoming - Vector((0, 0.5, 0))).length < 1e-9

    def test_acute_turn_uses_bevel_instead_of_long_spike(self):
        next_tangent = Vector((-0.984807753, 0.173648178, 0))
        next_wing = Vector((-0.173648178, -0.984807753, 0))
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            next_tangent,
            next_wing,
            1.0,
        )

        assert join.is_bevel
        assert join.incoming.length <= 1.000001
        assert join.outgoing.length <= 1.000001

    def test_skew_surface_offsets_use_bevel(self):
        join = _corner_offset_join(
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((0, 1, 0)),
            Vector((0, 1, 0)),
            Vector((-1, 0, 1)).normalized(),
            1.0,
        )

        assert join.is_bevel


class TestPolylineTangents:
    def test_straight_line(self):
        points = [Vector((0, 0, 0)), Vector((1, 0, 0)), Vector((2, 0, 0))]
        tangents = _polyline_tangents(points)
        for tangent in tangents:
            assert abs(tangent.x - 1.0) < 1e-6

    def test_corner_bisector(self):
        points = [Vector((0, 0, 0)), Vector((1, 0, 0)), Vector((1, 1, 0))]
        tangents = _polyline_tangents(points)
        mid = tangents[1]
        assert abs(mid.x - mid.y) < 1e-6

    def test_closed_wraparound_endpoints_match(self):
        # Квадрат, замкнутый дублированием первой точки: касательные на
        # обоих концах — одинаковая биссектриса последнего и первого
        # сегментов (иначе крылья ленты дают щель на стыке кольца).
        square = [
            Vector((0, 0, 0)),
            Vector((1, 0, 0)),
            Vector((1, 1, 0)),
            Vector((0, 1, 0)),
            Vector((0, 0, 0)),
        ]
        tangents = _polyline_tangents(square, closed=True)
        first, last = tangents[0], tangents[-1]
        assert (first - last).length < 1e-9
        # биссектриса сегментов (0,-1,0) и (1,0,0)
        assert abs(first.x + first.y) < 1e-6 and first.x > 0


class TestDedupePolyline:
    def test_collapses_near_duplicates(self):
        points = [
            Vector((0, 0, 0)),
            Vector((0.005, 0, 0)),
            Vector((1, 0, 0)),
        ]
        result = _dedupe_polyline(points)
        assert len(result) == 2
        assert result[1].x == 1.0

    def test_empty(self):
        assert _dedupe_polyline([]) == []


class TestTrimEdgeDedup:
    def test_duplicate_edge_registered_once(self):
        # Non-manifold: одно и то же ребро в двух WALL patches — в сборку
        # путей должно попасть один раз (иначе путь дублируется обратно).
        chain_a = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        wall_a = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain_a])
        chain_b = _make_chain([1, 0], [(1, 0, 1), (0, 0, 1)], -1)
        wall_b = _make_wall_node(1, (0, -1, 0), (0, 0, 1), [chain_b])
        graph = _make_graph(wall_a, wall_b)

        top_runs, bottom_runs = _collect_trim_ribbon_runs(graph)

        # ребро (0,1) — верхняя кромка обеих стен, но ключ регистрируется
        # только первым patch; дубликат отброшен (first-wins)
        assert _ribbon_edges(top_runs) == [(0, 1)]
        assert bottom_runs == []
