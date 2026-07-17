from math import cos, pi, sin

import pytest
from mathutils import Vector

pytest.importorskip("pyvoronoi")

from cftuv.decal_chart_admission import admit_intrinsic_strip_chart
from cftuv.decal_charts import (
    ChartBuildFailure,
    ChartSiteSeed,
    build_intrinsic_strip_charts,
)
from cftuv.decal_voronoi import (
    DomainBudgetExceeded,
    PatchVoronoiPlan,
    PatchVoronoiDiagnostics,
    _build_decal_arrangement,
    _compile_intrinsic_surface,
    _evaluate_surface_crops,
    _normalized_corner_runtime_settings,
    compile_patch_voronoi_attempt,
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
    serialize_network_faces,
)
from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode


def _closed_tube_graph(
    segment_count=8,
    *,
    top_radius=1.0,
    reversed_winding=False,
    ring_count=2,
):
    positions = []
    for row in range(ring_count):
        factor = row / (ring_count - 1)
        height = 4.0 * factor
        radius = 1.0 + (top_radius - 1.0) * factor
        positions.extend(
            Vector(
                (
                    radius * cos(index * 2.0 * pi / segment_count),
                    radius * sin(index * 2.0 * pi / segment_count),
                    height,
                )
            )
            for index in range(segment_count)
        )
    triangles = []
    face_ids = []
    face_normals = []
    physical_edges = set()
    for row in range(ring_count - 1):
        for index in range(segment_count):
            next_index = (index + 1) % segment_count
            bottom = row * segment_count + index
            bottom_next = row * segment_count + next_index
            top = (row + 1) * segment_count + index
            top_next = (row + 1) * segment_count + next_index
            physical_edges.update(
                (
                    tuple(sorted((bottom, bottom_next))),
                    tuple(sorted((top, top_next))),
                    tuple(sorted((bottom, top))),
                    tuple(sorted((bottom_next, top_next))),
                )
            )
            facet_triangles = (
                (bottom, bottom_next, top_next),
                (bottom, top_next, top),
            )
            if reversed_winding:
                facet_triangles = tuple(
                    tuple(reversed(triangle)) for triangle in facet_triangles
                )
            triangles.extend(facet_triangles)
            face_id = 100 + row * segment_count + index
            face_ids.extend((face_id, face_id))
            middle = (index + 0.5) * 2.0 * pi / segment_count
            normal = Vector((cos(middle), sin(middle), 0.0))
            if reversed_winding:
                normal = normal * -1.0
            face_normals.extend((normal.copy(), normal.copy()))
    edge_index_by_pair = {
        pair: 8000 + index for index, pair in enumerate(sorted(physical_edges))
    }
    triangle_edge_indices = []
    for first, second, third in triangles:
        triangle_edge_indices.append(
            (
                edge_index_by_pair.get(tuple(sorted((second, third))), -1),
                edge_index_by_pair.get(tuple(sorted((third, first))), -1),
                edge_index_by_pair.get(tuple(sorted((first, second))), -1),
            )
        )
    node = PatchNode(
        patch_id=81,
        face_indices=sorted(set(face_ids)),
        centroid=sum(positions, Vector()) / len(positions),
        normal=Vector((-1.0 if reversed_winding else 1.0, 0.0, 0.0)),
        basis_u=Vector((0.0, 1.0, 0.0)),
        basis_v=Vector((0.0, 0.0, 1.0)),
        mesh_verts=positions,
        mesh_vert_indices=list(range(len(positions))),
        mesh_tris=triangles,
        mesh_tri_face_indices=face_ids,
        mesh_tri_face_normals=face_normals,
        mesh_tri_edge_indices=triangle_edge_indices,
    )

    def ring_chain(row):
        row_start = row * segment_count
        vertices = [row_start + index for index in range(segment_count)]
        pairs = [
            tuple(sorted((vertices[index], vertices[(index + 1) % segment_count])))
            for index in range(segment_count)
        ]
        owner_row = 0 if row == 0 else ring_count - 2
        owner_offset = 0 if row == 0 else -1
        return BoundaryChain(
            vert_indices=vertices,
            vert_cos=[positions[index] for index in vertices],
            edge_indices=[edge_index_by_pair[pair] for pair in pairs],
            side_face_indices=[
                100
                + owner_row * segment_count
                + ((index + owner_offset) % segment_count)
                for index in range(segment_count)
            ],
            side_face_normals=[
                face_normals[
                    (
                        owner_row * segment_count
                        + ((index + owner_offset) % segment_count)
                    )
                    * 2
                ]
                for index in range(segment_count)
            ],
            is_closed=True,
        )

    bottom_chain = ring_chain(0)
    top_chain = ring_chain(ring_count - 1)
    node.boundary_loops = [
        BoundaryLoop(chains=[bottom_chain]),
        BoundaryLoop(chains=[top_chain]),
    ]
    graph = PatchGraph()
    graph.add_node(node)
    vertical_edges = tuple(
        edge_index_by_pair[
            (row * segment_count + index, (row + 1) * segment_count + index)
        ]
        for row in range(ring_count - 1)
        for index in range(segment_count)
    )
    return graph, node, tuple(bottom_chain.edge_indices), vertical_edges


def _seed(node, edge_index, vertices, face_id):
    return ChartSiteSeed(
        edge_index=edge_index,
        source_vertex_ids=tuple(sorted(vertices)),
        source_face_id=face_id,
        chain_ref=(node.patch_id, 0, edge_index),
    )


def _assert_connected_manifold(faces):
    assert faces
    edge_owners = {}
    for face_index, face in enumerate(faces):
        assert len(face.positions) >= 3
        root = face.positions[0]
        area = sum(
            (face.positions[index] - root)
            .cross(face.positions[index + 1] - root)
            .length
            * 0.5
            for index in range(1, len(face.positions) - 1)
        )
        assert area > 1e-10
        for index, first in enumerate(face.vert_keys):
            second = face.vert_keys[(index + 1) % len(face.vert_keys)]
            edge = tuple(sorted((repr(first), repr(second))))
            edge_owners.setdefault(edge, []).append(face_index)
    assert max(map(len, edge_owners.values())) <= 2
    neighbours = [set() for _face in faces]
    for owners in edge_owners.values():
        if len(owners) == 2:
            first, second = owners
            neighbours[first].add(second)
            neighbours[second].add(first)
    unseen = set(range(len(faces)))
    frontier = [unseen.pop()]
    while frontier:
        for neighbour in neighbours[frontier.pop()]:
            if neighbour in unseen:
                unseen.remove(neighbour)
                frontier.append(neighbour)
    assert not unseen


def _supporting_line_key(first, second):
    direction = second - first
    if direction.length <= 1e-10:
        return None
    direction.normalize()
    values = tuple(float(value) for value in direction)
    if next((value for value in values if abs(value) > 1e-9), 1.0) < 0.0:
        direction = direction * -1.0
    moment = first.cross(direction)
    return tuple(
        round(float(value), 5) for value in (*direction, *moment)
    )


def _interior_supporting_lines(faces):
    edge_uses = {}
    for face in faces:
        for index, first in enumerate(face.positions):
            second = face.positions[(index + 1) % len(face.positions)]
            key = tuple(
                sorted(
                    (
                        tuple(round(float(value), 6) for value in first),
                        tuple(round(float(value), 6) for value in second),
                    )
                )
            )
            edge_uses.setdefault(key, (0, first, second))
            count, stored_first, stored_second = edge_uses[key]
            edge_uses[key] = (count + 1, stored_first, stored_second)
    return {
        line
        for count, first, second in edge_uses.values()
        if count >= 2
        for line in (_supporting_line_key(first, second),)
        if line is not None
    }


def _arrangement_area(plan, width):
    pending = []
    alpha = width * 0.5
    for surface in plan.surfaces:
        _evaluate_surface_crops(
            surface,
            alpha,
            pending,
            _normalized_corner_runtime_settings(None),
        )
    arrangement = _build_decal_arrangement(pending, tolerance=5e-6)
    return sum(
        abs(
            sum(
                face.points[index][0]
                * face.points[(index + 1) % len(face.points)][1]
                - face.points[(index + 1) % len(face.points)][0]
                * face.points[index][1]
                for index in range(len(face.points))
            )
            * 0.5
        )
        for face in arrangement.faces
    )


def _direct_plan(node, chart, raw_sites, diagnostics=None):
    surface = _compile_intrinsic_surface(
        node, chart, raw_sites, diagnostics
    )
    return PatchVoronoiPlan(
        offset=0.01,
        surfaces=(surface,),
        lifted_vertices={
            index: position.copy()
            for index, position in enumerate(node.mesh_verts)
        },
        max_lateral_lift_ratio=0.0,
        alpha_budget=chart.alpha_budget,
        budget_source=chart.budget_source,
        requested_alpha_budget=100.0,
    )


@pytest.mark.parametrize("segment_count", (8, 6))
def test_d4_1_2_closed_round_and_polygonal_tubes(segment_count):
    graph, _node, ring_edges, _vertical_edges = _closed_tube_graph(segment_count)
    diagnostics = PatchVoronoiDiagnostics()

    plan = compile_patch_voronoi_plan(
        graph,
        ring_edges,
        offset=0.01,
        alpha_budget=100.0,
        diagnostics=diagnostics,
    )

    assert plan.backend_kind == "INTRINSIC_DEVELOPABLE"
    assert len(plan.surfaces) == 1
    surface = plan.surfaces[0]
    assert surface.domain.periodic_axis == "U"
    assert diagnostics.periodic_copy_count > 0
    faces = evaluate_patch_voronoi_plan(
        plan, width=0.5, preview=True, diagnostics=diagnostics
    )
    _assert_connected_manifold(faces)
    assert diagnostics.periodic_weld_count > 0
    assert diagnostics.runtime_policy_counts == {"MITER": segment_count}
    assert serialize_network_faces(faces) == serialize_network_faces(
        evaluate_patch_voronoi_plan(plan, width=0.5, preview=False)
    )


def test_d5_2_multiring_tube_uses_one_periodic_cut_path():
    _graph, node, _ring_edges, _vertical_edges = _closed_tube_graph(
        8, ring_count=3
    )
    middle_row = 8
    seeds = tuple(
        _seed(
            node,
            9100 + index,
            (middle_row + index, middle_row + (index + 1) % 8),
            100 + index,
        )
        for index in range(8)
    )
    chart = admit_intrinsic_strip_chart(
        build_intrinsic_strip_charts(
            node, seeds, alpha_budget=100.0
        )[0]
    )

    assert chart.periodic_axis == "U"
    assert len(chart.cuts) == 1
    assert len(chart.periodic_cut.source_edges) == 2
    raw_sites = tuple(
        {
            "patch_id": node.patch_id,
            "edge_index": seed.edge_index,
            "vert_a": seed.source_vertex_ids[0],
            "vert_b": seed.source_vertex_ids[1],
            "source_a": node.mesh_verts[seed.source_vertex_ids[0]].copy(),
            "source_b": node.mesh_verts[seed.source_vertex_ids[1]].copy(),
            "arc_start": float(index),
            "segment_length": (
                node.mesh_verts[seed.source_vertex_ids[1]]
                - node.mesh_verts[seed.source_vertex_ids[0]]
            ).length,
            "side_normal": node.mesh_tri_face_normals[index * 2].copy(),
            "owner_face_index": seed.source_face_id,
            "uv_sign": -1.0,
            "two_sided": True,
        }
        for index, seed in enumerate(seeds)
    )
    diagnostics = PatchVoronoiDiagnostics()
    plan = _direct_plan(node, chart, raw_sites, diagnostics)
    faces = evaluate_patch_voronoi_plan(
        plan, width=0.5, preview=True, diagnostics=diagnostics
    )

    _assert_connected_manifold(faces)
    assert diagnostics.periodic_weld_count > 0
    assert serialize_network_faces(faces) == serialize_network_faces(
        evaluate_patch_voronoi_plan(plan, width=0.5, preview=False)
    )


def test_d5_3_periodic_supporting_lines_never_disappear():
    graph, _node, ring_edges, _vertical_edges = _closed_tube_graph(8)
    plan = compile_patch_voronoi_plan(
        graph, (ring_edges[0],), offset=0.01, alpha_budget=100.0
    )
    period = plan.surfaces[0].domain.period
    previous_lines = set()

    for fraction in (0.15, 0.2, 0.3, 0.5, 0.7, 0.8, 0.9, 1.0):
        faces = evaluate_patch_voronoi_plan(
            plan, width=period * fraction, preview=True
        )
        lines = _interior_supporting_lines(faces)
        assert previous_lines <= lines, (
            f"width={period * fraction:.6g}: periodic S1 lines vanished "
            f"{sorted(previous_lines - lines)!r}"
        )
        previous_lines = lines


def test_d5_3_closed_ring_keeps_resolved_partition_during_drag():
    graph, _node, ring_edges, _vertical_edges = _closed_tube_graph(8)
    plan = compile_patch_voronoi_plan(
        graph, ring_edges, offset=0.01, alpha_budget=100.0
    )
    period = plan.surfaces[0].domain.period
    reference_lines = None

    for fraction in (0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0):
        faces = evaluate_patch_voronoi_plan(
            plan, width=period * fraction, preview=True
        )
        assert len(faces) == 8
        lines = _interior_supporting_lines(faces)
        if reference_lines is None:
            reference_lines = lines
        else:
            assert lines == reference_lines


def test_d4_3_4_sites_near_cut_collide_through_periodic_boundary():
    _graph, node, ring_edges, vertical_edges = _closed_tube_graph(12)
    ring_seed = _seed(node, ring_edges[0], (0, 1), 100)
    chart = admit_intrinsic_strip_chart(
        build_intrinsic_strip_charts(
            node, (ring_seed,), alpha_budget=100.0
        )[0]
    )
    cut_index = chart.periodic_cut.source_edge[0]
    site_indices = ((cut_index - 1) % 12, (cut_index + 1) % 12)
    raw_sites = tuple(
        {
            "patch_id": node.patch_id,
            "edge_index": vertical_edges[index],
            "vert_a": index,
            "vert_b": 12 + index,
            "source_a": node.mesh_verts[index].copy(),
            "source_b": node.mesh_verts[12 + index].copy(),
            "arc_start": float(order * 4),
            "segment_length": 4.0,
            "side_normal": node.mesh_tri_face_normals[index * 2].copy(),
            "owner_face_index": 100 + index,
            "uv_sign": -1.0,
            "two_sided": True,
        }
        for order, index in enumerate(site_indices)
    )
    diagnostics = PatchVoronoiDiagnostics()
    plan = _direct_plan(node, chart, raw_sites, diagnostics)

    narrow = evaluate_patch_voronoi_plan(plan, width=0.2, preview=True)
    collided = evaluate_patch_voronoi_plan(
        plan,
        width=min(chart.period, 1.5),
        preview=True,
        diagnostics=diagnostics,
    )

    assert diagnostics.periodic_copy_count > 0
    assert diagnostics.periodic_weld_count > 0
    assert len(narrow) > 0 and len(collided) > 0
    assert len({frozenset(face.vert_keys) for face in collided}) == len(collided)


def test_d4_5_reversed_winding_keeps_period_and_side_parity():
    regular = _closed_tube_graph(8)
    reversed_fixture = _closed_tube_graph(8, reversed_winding=True)
    results = []
    for graph, _node, ring_edges, _vertical_edges in (
        regular,
        reversed_fixture,
    ):
        plan = compile_patch_voronoi_plan(
            graph, ring_edges, offset=0.01, alpha_budget=100.0
        )
        faces = evaluate_patch_voronoi_plan(plan, width=0.5, preview=True)
        results.append((plan.surfaces[0].domain.period, faces))
    assert results[0][0] == pytest.approx(results[1][0])
    assert sorted(
        round(abs(value), 6)
        for face in results[0][1]
        for value in face.u_fracs
    ) == sorted(
        round(abs(value), 6)
        for face in results[1][1]
        for value in face.u_fracs
    )


def test_d4_6_half_period_saturates_without_gap_or_double_cover():
    _graph, node, _ring_edges, vertical_edges = _closed_tube_graph(8)
    diagnostics = PatchVoronoiDiagnostics()
    selected_index = 0
    seed = _seed(
        node,
        vertical_edges[selected_index],
        (selected_index, 8 + selected_index),
        100 + selected_index,
    )
    chart = admit_intrinsic_strip_chart(
        build_intrinsic_strip_charts(
            node, (seed,), alpha_budget=100.0
        )[0]
    )
    raw_site = {
        "patch_id": node.patch_id,
        "edge_index": vertical_edges[selected_index],
        "vert_a": selected_index,
        "vert_b": 8 + selected_index,
        "source_a": node.mesh_verts[selected_index].copy(),
        "source_b": node.mesh_verts[8 + selected_index].copy(),
        "arc_start": 0.0,
        "segment_length": 4.0,
        "side_normal": node.mesh_tri_face_normals[0].copy(),
        "owner_face_index": 100,
        "uv_sign": -1.0,
        "two_sided": True,
    }
    plan = _direct_plan(node, chart, (raw_site,), diagnostics)
    compile_construct_calls = diagnostics.construct_calls
    period = plan.surfaces[0].domain.period
    width = period
    first = evaluate_patch_voronoi_plan(
        plan, width, preview=True, diagnostics=diagnostics
    )
    repeated = evaluate_patch_voronoi_plan(plan, width, preview=True)
    confirmed = evaluate_patch_voronoi_plan(plan, width, preview=False)
    domain_area = sum(
        abs(
            sum(
                triangle[index][0] * triangle[(index + 1) % 3][1]
                - triangle[(index + 1) % 3][0] * triangle[index][1]
                for index in range(3)
            )
            * 0.5
        )
        for triangle in plan.surfaces[0].domain.boundary_triangles
    )

    _assert_connected_manifold(first)
    assert diagnostics.periodic_weld_count > 0
    assert diagnostics.construct_calls == compile_construct_calls
    assert _arrangement_area(plan, width) == pytest.approx(domain_area, rel=1e-6)
    assert serialize_network_faces(first) == serialize_network_faces(repeated)
    assert serialize_network_faces(first) == serialize_network_faces(confirmed)
    assert len({frozenset(face.vert_keys) for face in first}) == len(first)


def test_d4_6_public_compile_enforces_periodic_half_period_budget():
    graph, _node, ring_edges, _vertical_edges = _closed_tube_graph(8)
    plan = compile_patch_voronoi_plan(
        graph,
        ring_edges,
        offset=0.01,
        alpha_budget=100.0,
    )
    period = plan.surfaces[0].domain.period

    assert plan.alpha_budget == pytest.approx(period * 0.5)
    evaluate_patch_voronoi_plan(plan, width=period, preview=True)
    with pytest.raises(DomainBudgetExceeded) as captured:
        evaluate_patch_voronoi_plan(
            plan,
            width=period * 1.01,
            preview=True,
        )
    assert captured.value.code == "DOMAIN_BUDGET_EXCEEDED"


def test_d4_n1_frustum_rejects_rotational_holonomy():
    graph, node, ring_edges, _vertical_edges = _closed_tube_graph(
        8, top_radius=0.7
    )
    seed = _seed(node, ring_edges[0], (0, 1), 100)
    with pytest.raises(ChartBuildFailure) as captured:
        admit_intrinsic_strip_chart(
            build_intrinsic_strip_charts(
                node, (seed,), alpha_budget=100.0
            )[0]
        )
    assert captured.value.code == "PERIODIC_HOLONOMY_UNSUPPORTED"
    attempt = compile_patch_voronoi_attempt(
        graph,
        ring_edges,
        offset=0.01,
        alpha_budget=100.0,
        allow_partial=True,
    )
    assert attempt.plan is None
    assert {failure.reason for failure in attempt.failures} == {
        "PERIODIC_HOLONOMY_UNSUPPORTED"
    }
    assert attempt.rejected_edge_indices == tuple(sorted(ring_edges))


def test_d4_n2_selected_generators_leave_no_legal_cut():
    _graph, node, _ring_edges, vertical_edges = _closed_tube_graph(8)
    seeds = tuple(
        _seed(node, edge_index, (index, 8 + index), 100 + index)
        for index, edge_index in enumerate(vertical_edges)
    )
    with pytest.raises(ChartBuildFailure) as captured:
        admit_intrinsic_strip_chart(
            build_intrinsic_strip_charts(
                node, seeds, alpha_budget=100.0
            )[0]
        )
    assert captured.value.code == "CUT_CROSSES_SELECTED_CHAIN"
