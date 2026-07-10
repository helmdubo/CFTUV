from __future__ import annotations

from mathutils import Vector

from cftuv.decals import (
    _boundary_wing_direction,
    _chain_edge_paths,
    _collect_manual_chain_decals,
    _collect_trim_segments,
    _collect_wall_pair_chains,
    _corner_wing_directions,
    _dedupe_polyline,
    _polyline_tangents,
    _trim_vertex_frames,
    chain_refs_for_edge_indices,
)
from cftuv.model import (
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
    PatchType,
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
    )
    chain.dihedral_convexity = dihedral_convexity
    return chain


def _make_graph(*nodes):
    graph = PatchGraph()
    for node in nodes:
        graph.add_node(node)
    return graph


class TestCollectTrimSegments:
    def test_top_bottom_classification(self):
        # Стена с нормалью +Y, вертикаль +Z. Верхняя кромка идёт по +X
        # (outward = d x n = +Z), нижняя — по -X (outward = -Z).
        top_chain = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        bottom_chain = _make_chain([2, 3], [(1, 0, 0), (0, 0, 0)], -1)
        side_chain = _make_chain([1, 2], [(1, 0, 1), (1, 0, 0)], -1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [top_chain, bottom_chain, side_chain])
        graph = _make_graph(wall)

        top_edges, bottom_edges, edge_frames, vert_cos = _collect_trim_segments(graph)

        assert top_edges == [(0, 1)]
        assert bottom_edges == [(2, 3)]
        assert (0, 1) in edge_frames and (2, 3) in edge_frames
        normal, up = edge_frames[(0, 1)]
        assert abs(normal.y - 1.0) < 1e-6
        assert abs(up.z - 1.0) < 1e-6
        assert set(vert_cos.keys()) == {0, 1, 2, 3}

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

        top_edges, bottom_edges, _frames, _cos = _collect_trim_segments(graph)

        assert top_edges == []
        assert bottom_edges == [(2, 3)]

    def test_non_wall_patches_ignored(self):
        chain = _make_chain([0, 1], [(0, 0, 1), (1, 0, 1)], -1)
        floor = PatchNode(patch_id=0, face_indices=[0])
        floor.patch_type = PatchType.FLOOR
        floor.boundary_loops = [BoundaryLoop(chains=[chain])]
        graph = _make_graph(floor)

        top_edges, bottom_edges, _frames, _cos = _collect_trim_segments(graph)

        assert top_edges == [] and bottom_edges == []

    def test_short_edges_skipped(self):
        chain = _make_chain([0, 1], [(0, 0, 1), (0.01, 0, 1)], -1)
        wall = _make_wall_node(0, (0, 1, 0), (0, 0, 1), [chain])
        graph = _make_graph(wall)

        top_edges, bottom_edges, _frames, _cos = _collect_trim_segments(graph)

        assert top_edges == [] and bottom_edges == []

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
        top_edges, bottom_edges, _frames, _cos = _collect_trim_segments(
            graph, chain_refs=chain_refs
        )

        assert chain_refs == {(0, 0, 0)}
        assert top_edges == [(0, 1), (1, 2)]
        assert bottom_edges == []


class TestChainEdgePaths:
    def test_merges_shared_vertices(self):
        paths = _chain_edge_paths([(0, 1), (1, 2), (5, 6)])
        assert sorted(len(p) for p in paths) == [2, 3]
        merged = next(p for p in paths if len(p) == 3)
        assert merged in ([0, 1, 2], [2, 1, 0])

    def test_grows_both_directions(self):
        paths = _chain_edge_paths([(1, 2), (0, 1), (2, 3)])
        assert len(paths) == 1
        assert paths[0] in ([0, 1, 2, 3], [3, 2, 1, 0])

    def test_empty(self):
        assert _chain_edge_paths([]) == []


class TestTrimVertexFrames:
    def test_bisector_at_patch_junction(self):
        frames_by_edge = {
            (0, 1): (Vector((0, 1, 0)), Vector((0, 0, 1))),
            (1, 2): (Vector((1, 0, 0)), Vector((0, 0, 1))),
        }
        frames = _trim_vertex_frames([0, 1, 2], frames_by_edge)

        assert abs(frames[0][0].y - 1.0) < 1e-6  # конец — фрейм своего ребра
        mid_normal = frames[1][0]
        assert abs(mid_normal.x - mid_normal.y) < 1e-6  # биссектриса
        assert mid_normal.length - 1.0 < 1e-6
        assert abs(frames[2][0].x - 1.0) < 1e-6


class TestCollectWallPairChains:
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


class TestTrimRingClosure:
    def test_ring_endpoint_frames_bisected(self):
        # Кольцо из рёбер двух перпендикулярных стен: точка замыкания
        # должна получить биссектрису, идентичную на обоих концах пути.
        frames_by_edge = {
            (0, 3): (Vector((1, 0, 0)), Vector((0, 0, 1))),  # последнее ребро
            (0, 1): (Vector((0, 1, 0)), Vector((0, 0, 1))),  # первое ребро
            (1, 2): (Vector((0, 1, 0)), Vector((0, 0, 1))),
            (2, 3): (Vector((1, 0, 0)), Vector((0, 0, 1))),
        }
        ring_path = [0, 1, 2, 3, 0]
        frames = _trim_vertex_frames(ring_path, frames_by_edge)
        first_normal = frames[0][0]
        last_normal = frames[-1][0]
        assert (first_normal - last_normal).length < 1e-9
        assert abs(first_normal.x - first_normal.y) < 1e-6  # биссектриса


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

        top_edges, bottom_edges, _frames, _cos = _collect_trim_segments(graph)

        # ребро (0,1) — верхняя кромка обеих стен, но ключ регистрируется
        # только первым patch; дубликат отброшен (first-wins)
        assert top_edges == [(0, 1)]
        assert bottom_edges == []
