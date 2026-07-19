from __future__ import annotations

from dataclasses import replace
from math import cos, isclose, pi, sqrt

import pytest

from mathutils import Vector

from cftuv import decal_rails
from cftuv.decal_rail_geometry import (
    RailGeometryEvaluationError,
    compile_planar_rail_geometry_attempt,
    evaluate_planar_rail_geometry_plan,
)
from cftuv.model import BoundaryChain

from decal_rail_fixtures import (
    mesh_graph,
    planar_acute_join,
    planar_concave_corner_join,
    planar_dihedral_strip,
    planar_quad_strip,
    planar_rf1_ring,
    planar_rf10_quarter_join,
    planar_rf10_with_disconnected_concave_face,
    planar_rf11_boundary_join,
    planar_rf13_collinear_fan_face,
    planar_rf13_degenerate_footprint_face,
    planar_rf13_fold_boundary_join,
    planar_rf13_shared_fold_boundary,
    planar_rf15_structural_cap_fan,
    planar_rf13_with_remote_degenerate_face,
    planar_shallow_dihedral_strip,
    rr9_rf16_terminal_fold_caps,
)


def _compile_geometry(fixture, *, alpha_budget=1.5, **compile_kwargs):
    graph, _edge_ids, selected, _vertex_at = fixture
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=alpha_budget,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
        **compile_kwargs,
    )
    assert attempt.failures == ()
    assert attempt.plan is not None
    return rail_plan, attempt.plan


def _face_signature(faces):
    return tuple(
        (
            face.surface_id,
            tuple(face.vert_keys),
            tuple(tuple(float(value) for value in position) for position in face.positions),
            tuple(face.u_fracs),
            tuple(face.v_lengths),
            face.component_kind,
            face.component_side,
            face.rail_provenance,
        )
        for face in faces
    )


def _polygon_area(face):
    origin = face.positions[0]
    area = Vector((0.0, 0.0, 0.0))
    for index in range(1, len(face.positions) - 1):
        area += (face.positions[index] - origin).cross(
            face.positions[index + 1] - origin
        )
    return area.length * 0.5


def _edge_incidence(faces):
    incidence = {}
    for face_index, face in enumerate(faces):
        for vertex_a, vertex_b in zip(
            face.vert_keys,
            face.vert_keys[1:] + face.vert_keys[:1],
        ):
            edge_key = frozenset((vertex_a, vertex_b))
            incidence.setdefault(edge_key, []).append(face_index)
    return incidence


def _face_component_count(faces):
    adjacency = {index: set() for index in range(len(faces))}
    for owners in _edge_incidence(faces).values():
        if len(owners) == 2:
            adjacency[owners[0]].add(owners[1])
            adjacency[owners[1]].add(owners[0])
    unseen = set(adjacency)
    component_count = 0
    while unseen:
        component_count += 1
        pending = [unseen.pop()]
        while pending:
            current = pending.pop()
            neighbors = adjacency[current].intersection(unseen)
            unseen.difference_update(neighbors)
            pending.extend(neighbors)
    return component_count


def _assert_unique_key_positions(faces):
    positions_by_key = {}
    for face in faces:
        for key, position in zip(face.vert_keys, face.positions):
            exact_position = tuple(float(value) for value in position)
            if key in positions_by_key:
                assert positions_by_key[key] == exact_position
            else:
                positions_by_key[key] = exact_position


def _boundary_loop_count(faces):
    boundary_edges = [
        tuple(edge_key)
        for edge_key, owners in _edge_incidence(faces).items()
        if len(owners) == 1
    ]
    adjacency = {}
    for vertex_a, vertex_b in boundary_edges:
        adjacency.setdefault(vertex_a, set()).add(vertex_b)
        adjacency.setdefault(vertex_b, set()).add(vertex_a)
    assert adjacency
    assert {len(neighbors) for neighbors in adjacency.values()} == {2}
    unseen = set(adjacency)
    loop_count = 0
    while unseen:
        loop_count += 1
        pending = [unseen.pop()]
        while pending:
            current = pending.pop()
            neighbors = adjacency[current].intersection(unseen)
            unseen.difference_update(neighbors)
            pending.extend(neighbors)
    return loop_count


def _point_line_distance(point, point_a, point_b):
    tangent = point_b - point_a
    return tangent.cross(point - point_a).length / tangent.length


def _point_in_source_face(point, source_face, source_positions):
    normal = Vector(source_face.normal)
    polygon = [
        Vector(source_positions[vertex_id])
        for vertex_id in source_face.vertex_ids
    ]
    return all(
        (polygon[(index + 1) % len(polygon)] - polygon[index])
        .cross(point - polygon[index])
        .dot(normal)
        >= -1.0e-12
        for index in range(len(polygon))
    )


def test_r1_regular_planar_channels_are_compile_static_and_fully_provenanced():
    _rail_plan, plan = _compile_geometry(
        planar_quad_strip(),
        alpha_budget=1.5,
    )

    assert plan.component.is_closed is False
    assert len(plan.channels) == 4
    assert {len(channel.cells) for channel in plan.channels} == {2}
    assert plan.rail_plan.routes == ()
    assert plan.cap_traces == ()
    assert {path.kind for path in plan.boundary_paths} <= {
        "IN_PLANE",
        "IN_PLANE_JOIN",
        "CORNER",
    }
    assert all(
        cell.source_edge_ids
        and cell.boundary_path_ids
        and all(
                vertex.source_face_id == cell.owner_face_id
                and vertex.source_feature
                and vertex.source_feature_id is not None
                and vertex.boundary_path_id
                and all(route_id >= 0 for route_id in vertex.route_ids)
            for vertex in cell.vertices
        )
        for channel in plan.channels
        for cell in channel.cells
    )
    assert all(
        face.normal == (0.0, 0.0, 1.0)
        and face.planarity_min_dot == 1.0
        for face in plan.rail_plan.faces
    )


def test_rp_planar_endpoint_uses_full_width_analytic_path_without_taper():
    _rail_plan, plan = _compile_geometry(
        planar_quad_strip(),
        alpha_budget=1.5,
    )
    faces = evaluate_planar_rail_geometry_plan(plan, width=1.0)

    start_station_faces = [
        face
        for face in faces
        if min(face.v_lengths) == 0.0
    ]
    assert len(start_station_faces) == 2
    for face in start_station_faces:
        at_start = [
            abs(u_frac)
            for u_frac, s in zip(face.u_fracs, face.v_lengths)
            if s == 0.0
        ]
        assert sorted(at_start) == [0.0, 1.0]
    assert plan.rail_plan.events == ()
    assert plan.cap_traces == ()
    assert all(
        max(piece.end.r for piece in path.pieces) == plan.alpha_budget
        for path in plan.boundary_paths
        if path.kind == "IN_PLANE"
    )
    assert all(
        use.kind == decal_rails.RailTerminalKind.IN_PLANE_EMPTY
        for use in plan.rail_plan.terminal_uses
    )


def test_r12_rr9_rf16_materialization_reads_diagonal_terminal_routes():
    graph, _edge_ids, selected, vertex_at = rr9_rf16_terminal_fold_caps()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )

    assert attempt.failures == ()
    assert attempt.plan is not None
    terminal = tuple(
        use
        for use in rail_plan.terminal_uses
        if use.spine_vertex_id == vertex_at[1]
    )
    path_by_route = {
        path.route_id: path
        for path in attempt.plan.boundary_paths
        if path.kind == "ROUTE"
    }
    endpoint = Vector((0.0, 1.0, 0.0))
    positions = {
        vertex.vertex_id: Vector(vertex.position) for vertex in rail_plan.vertices
    }
    edges = {edge.edge_id: edge for edge in rail_plan.edges}
    for use in terminal:
        path = path_by_route[use.route_id]
        target_id = next(
            vertex_id
            for vertex_id in edges[use.route_edge_id].vertex_ids
            if vertex_id != vertex_at[1]
        )
        expected = (positions[target_id] - endpoint).normalized()
        assert path.pieces
        for piece in path.pieces:
            assert tuple(piece.start.position) == tuple(endpoint)
            delta = Vector(piece.end.position) - endpoint
            assert isclose(delta.length, 0.75, abs_tol=1e-9)
            assert isclose(delta.normalized().dot(expected), 1.0, abs_tol=1e-9)

    faces = evaluate_planar_rail_geometry_plan(attempt.plan, width=1.5)
    used_routes = {
        route_id
        for face in faces
        for route_id in face.rail_provenance.route_ids
    }
    assert {use.route_id for use in terminal}.issubset(used_routes)


def test_r11_rf11_boundary_pchain_preempts_cap_and_uses_plain_channel():
    graph, edge_ids, selected, vertex_at = planar_rf11_boundary_join()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.5,
    )
    endpoint_id = vertex_at[(0, 1)]
    boundary_edge_ids = {
        edge_ids[tuple(sorted((vertex_at[(0, 1)], vertex_at[(-1, 1)])))],
        edge_ids[tuple(sorted((vertex_at[(0, 1)], vertex_at[(1, 1)])))],
    }
    boundary_routes = tuple(
        route
        for route in rail_plan.routes
        if route.key.side.spine_vertex_id == endpoint_id
        and route.key.side.start_edge_id in boundary_edge_ids
    )

    assert len(boundary_routes) == 2
    assert {
        route.key.side.start_edge_id for route in boundary_routes
    } == boundary_edge_ids
    assert all(
        route.termination == decal_rails.RailTermination.ALPHA
        for route in boundary_routes
    )
    assert all(len(route.segments) == 2 for route in boundary_routes)
    assert all(route.stations[-1].distance == 1.5 for route in boundary_routes)
    assert not any(
        route.key.side.spine_vertex_id == endpoint_id
        and route.key.side.start_edge_id < 0
        for route in rail_plan.routes
    )
    boundary_chain = graph.nodes[0].boundary_loops[0].chains[1]
    boundary_chain.vert_indices.reverse()
    boundary_chain.edge_indices.reverse()
    reversed_plan = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=1.5,
    )
    assert reversed_plan == rail_plan

    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )
    assert attempt.failures == ()
    assert attempt.plan is not None
    assert attempt.plan.cap_traces == ()
    assert attempt.plan.corner_partitions == ()
    assert attempt.plan.corner_cells == ()

    faces = evaluate_planar_rail_geometry_plan(attempt.plan, width=3.0)
    endpoint_vertices = [
        position
        for face in faces
        for position, station in zip(face.positions, face.v_lengths)
        if station == 1.0
    ]
    assert endpoint_vertices
    assert all(position.y == 1.0 for position in endpoint_vertices)
    assert {face.component_kind for face in faces} == {"RAIL_SEGMENT"}


def test_r11_rf13_remote_degenerate_face_is_outside_validation_footprint():
    graph, _edge_ids, selected, _vertex_at = (
        planar_rf13_with_remote_degenerate_face()
    )
    rail_attempt = decal_rails.compile_decal_rail_attempt(
        graph,
        selected,
        alpha_budget=1.5,
    )

    assert rail_attempt.failures == ()
    assert rail_attempt.plan is not None
    geometry_attempt = compile_planar_rail_geometry_attempt(
        rail_attempt.plan,
        edge_indices=selected,
    )
    assert geometry_attempt.failures == ()
    assert geometry_attempt.plan is not None
    faces = evaluate_planar_rail_geometry_plan(
        geometry_attempt.plan,
        width=1.0,
    )
    assert faces
    assert max(position.x for face in faces for position in face.positions) < 10.0


def test_r11_rf13_collinear_fan_triangle_materializes_valid_face():
    graph, _edge_ids, selected, _vertices = planar_rf13_collinear_fan_face()
    node = graph.nodes[0]
    first_triangle = tuple(
        node.mesh_verts[index] for index in node.mesh_tris[0]
    )
    assert (first_triangle[1] - first_triangle[0]).cross(
        first_triangle[2] - first_triangle[0]
    ).length == 0.0

    rail_attempt = decal_rails.compile_decal_rail_attempt(
        graph,
        selected,
        alpha_budget=1.5,
    )
    assert rail_attempt.failures == ()
    assert rail_attempt.plan is not None
    geometry_attempt = compile_planar_rail_geometry_attempt(
        rail_attempt.plan,
        edge_indices=selected,
    )
    assert geometry_attempt.failures == ()
    assert geometry_attempt.plan is not None
    faces = evaluate_planar_rail_geometry_plan(
        geometry_attempt.plan,
        width=1.0,
    )
    assert faces
    assert all(_polygon_area(face) > 0.0 for face in faces)


def test_r11_rf13_true_degenerate_footprint_face_is_named_failure():
    graph, _edge_ids, selected, _vertices = (
        planar_rf13_degenerate_footprint_face()
    )
    attempt = decal_rails.compile_decal_rail_attempt(
        graph,
        selected,
        alpha_budget=1.5,
    )

    assert attempt.plan is None
    assert len(attempt.failures) == 1
    failure = attempt.failures[0]
    assert failure.reason == "SOURCE_TRIANGLE_DEGENERATE"
    assert failure.vertex_indices == (0, 1, 2)
    assert dict(failure.details)["face_id"] == 1000


def test_r11_rf13_fold_boundary_ignores_non_materialized_far_side():
    graph, _edge_ids, selected, _vertex_at = planar_rf13_fold_boundary_join()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.5,
    )
    edges = {edge.edge_id: edge for edge in rail_plan.edges}
    faces = {face.face_id: face for face in rail_plan.faces}
    boundary_routes = tuple(
        route
        for route in rail_plan.routes
        if route.key.side.start_edge_id >= 0
        and edges[route.key.side.start_edge_id].is_pchain
    )
    assert len(boundary_routes) == 2
    assert all(
        any(
            sum(a * b for a, b in zip(faces[first].normal, faces[second].normal))
            == 0.0
            for segment in route.segments
            for first in segment.source_face_ids
            for second in segment.source_face_ids
            if first < second
        )
        for route in boundary_routes
    )

    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )
    assert attempt.failures == ()
    assert attempt.plan is not None
    materialized = evaluate_planar_rail_geometry_plan(
        attempt.plan,
        width=1.0,
    )
    assert materialized


def test_r11_rf13_one_fold_route_serves_two_face_sectors_with_shared_stations():
    graph, edge_ids, selected, _vertices = planar_rf13_shared_fold_boundary()
    boundary_edge_id = edge_ids[(1, 2)]
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    boundary_routes = tuple(
        route
        for route in rail_plan.routes
        if route.key.side.start_edge_id == boundary_edge_id
    )
    assert len(boundary_routes) == 1
    boundary_route = boundary_routes[0]
    assert len(boundary_route.key.side.source_face_ids) == 2

    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )
    assert attempt.failures == ()
    assert attempt.plan is not None
    route_paths = tuple(
        path
        for path in attempt.plan.boundary_paths
        if path.route_id == boundary_route.route_id
    )
    assert len(route_paths) == 1
    assert {
        piece.owner_face_id for piece in route_paths[0].pieces
    } == set(boundary_route.key.side.source_face_ids)
    materialized = evaluate_planar_rail_geometry_plan(
        attempt.plan,
        width=0.75,
    )
    owner_faces = {
        face.rail_provenance.source_face_id
        for face in materialized
        if boundary_route.route_id in face.rail_provenance.route_ids
    }
    assert owner_faces == set(boundary_route.key.side.source_face_ids)


@pytest.mark.skip(reason="R3-scope: RM5a structural cap is non-planar only")
def test_r3_rf15_rm5a_empty_perpendicular_degrades_to_structural_cap():
    graph, edge_ids, selected, _vertices = planar_rf15_structural_cap_fan()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.5,
    )
    dam_sector = next(
        sector
        for sector in rail_plan.start_sectors
        if sector.spine_vertex_id == 0
        and sector.kind == decal_rails.RailStartSectorKind.DAM
    )
    structural_edge = edge_ids[(0, 2)]
    assert dam_sector.internal_edge_ids == (
        structural_edge,
        edge_ids[(0, 3)],
    )

    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )
    assert attempt.failures == ()
    structural_trace = next(
        trace
        for trace in attempt.plan.cap_traces
        if trace.spine_vertex_id == 0 and trace.initial_face_id == 1000
    )
    assert structural_trace.termination == "STRUCTURAL"
    assert len(structural_trace.pieces) == 1
    assert structural_trace.pieces[0].source_edge_ids == (structural_edge,)
    assert structural_trace.pieces[0].end.r > 0.0
    assert evaluate_planar_rail_geometry_plan(
        attempt.plan,
        width=0.5,
    )


def test_r1_width_drag_only_clips_compiled_cells_and_preview_equals_confirm():
    _rail_plan, plan = _compile_geometry(
        planar_quad_strip(),
        alpha_budget=1.5,
    )
    channel_snapshot = plan.channels

    narrow = evaluate_planar_rail_geometry_plan(
        plan,
        width=0.5,
        offset=0.02,
        preview=True,
    )
    wide = evaluate_planar_rail_geometry_plan(
        plan,
        width=3.0,
        offset=0.02,
        preview=True,
    )
    confirmed = evaluate_planar_rail_geometry_plan(
        plan,
        width=3.0,
        offset=0.02,
        preview=False,
    )

    assert plan.channels is channel_snapshot
    assert len(narrow) == 4
    assert len(wide) == 8
    assert _face_signature(confirmed) == _face_signature(wide)
    assert all(position.z == 0.02 for face in wide for position in face.positions)


def test_r1_dihedral_shared_keys_receive_one_canonical_offset_lift():
    graph, _edge_ids, selected, _vertex_at = planar_dihedral_strip()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )
    assert attempt.failures == ()

    preview = evaluate_planar_rail_geometry_plan(
        attempt.plan,
        width=1.0,
        offset=0.02,
        preview=True,
    )
    confirmed = evaluate_planar_rail_geometry_plan(
        attempt.plan,
        width=1.0,
        offset=0.02,
        preview=False,
    )
    assert _face_signature(preview) == _face_signature(confirmed)
    _assert_unique_key_positions(preview)

    positions_by_key = {
        key: tuple(float(value) for value in position)
        for face in preview
        for key, position in zip(face.vert_keys, face.positions)
    }
    assert positions_by_key[("rail-source-vertex", 0)] == (
        -0.02,
        0.0,
        0.02,
    )
    assert positions_by_key[("rail-source-vertex", 1)] == (
        -0.02,
        1.0,
        0.02,
    )

    source_positions = {
        vertex.vertex_id: Vector(vertex.position)
        for vertex in rail_plan.vertices
    }
    source_faces = {face.face_id: face for face in rail_plan.faces}
    for face in preview:
        source_face = source_faces[face.surface_id]
        normal = Vector(source_face.normal)
        origin = source_positions[source_face.vertex_ids[0]]
        assert all(
            isclose(normal.dot(position - origin), 0.02, abs_tol=1e-12)
            for position in face.positions
        )

    node = graph.nodes[0]
    node.mesh_tris.reverse()
    node.mesh_tri_face_indices.reverse()
    node.mesh_tri_edge_indices.reverse()
    reversed_rail = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=1.0,
    )
    reversed_plan = compile_planar_rail_geometry_attempt(
        reversed_rail,
        edge_indices=tuple(reversed(selected)),
    ).plan
    reversed_faces = evaluate_planar_rail_geometry_plan(
        reversed_plan,
        width=1.0,
        offset=0.02,
    )
    assert reversed_rail == rail_plan
    assert reversed_plan == attempt.plan
    assert _face_signature(reversed_faces) == _face_signature(preview)


def test_r1_canonical_offset_lift_rejects_any_raw_key_position_desync():
    _rail_plan, plan = _compile_geometry(
        planar_dihedral_strip(),
        alpha_budget=1.0,
    )
    channels = list(plan.channels)
    changed = False
    for channel_index, channel in enumerate(channels):
        cells = list(channel.cells)
        for cell_index, cell in enumerate(cells):
            vertices = list(cell.vertices)
            for vertex_index, vertex in enumerate(vertices):
                if (
                    not changed
                    and cell.owner_face_id == 1001
                    and vertex.key == ("rail-source-vertex", 0)
                ):
                    vertices[vertex_index] = replace(
                        vertex,
                        position=(0.0, 0.0, 1e-15),
                    )
                    cells[cell_index] = replace(
                        cell,
                        vertices=tuple(vertices),
                    )
                    changed = True
                    break
        channels[channel_index] = replace(channel, cells=tuple(cells))
    assert changed
    broken_plan = replace(plan, channels=tuple(channels))

    with pytest.raises(RailGeometryEvaluationError) as exc_info:
        evaluate_planar_rail_geometry_plan(
            broken_plan,
            width=1.0,
            offset=0.02,
        )
    assert (
        exc_info.value.failure.reason
        == "RAIL_GEOMETRY_SHARED_KEY_POSITION_DESYNC"
    )
    assert exc_info.value.failure.face_indices == (1000, 1001)


def test_r1_shallow_dihedral_keeps_exact_offset_plane_incidence():
    graph, _edge_ids, selected, _vertex_at = planar_shallow_dihedral_strip()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )
    assert attempt.failures == ()

    preview = evaluate_planar_rail_geometry_plan(
        attempt.plan,
        width=1.0,
        offset=0.02,
        preview=True,
    )
    confirmed = evaluate_planar_rail_geometry_plan(
        attempt.plan,
        width=1.0,
        offset=0.02,
        preview=False,
    )
    assert _face_signature(preview) == _face_signature(confirmed)
    _assert_unique_key_positions(preview)

    source_positions = {
        vertex.vertex_id: Vector(vertex.position)
        for vertex in rail_plan.vertices
    }
    source_faces = {face.face_id: face for face in rail_plan.faces}
    for face in preview:
        source_face = source_faces[face.surface_id]
        normal = Vector(source_face.normal)
        origin = source_positions[source_face.vertex_ids[0]]
        assert all(
            isclose(normal.dot(position - origin), 0.02, abs_tol=1e-12)
            for position in face.positions
        )

    node = graph.nodes[0]
    node.mesh_tris.reverse()
    node.mesh_tri_face_indices.reverse()
    node.mesh_tri_edge_indices.reverse()
    reversed_rail = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=1.0,
    )
    reversed_plan = compile_planar_rail_geometry_attempt(
        reversed_rail,
        edge_indices=tuple(reversed(selected)),
    ).plan
    reversed_faces = evaluate_planar_rail_geometry_plan(
        reversed_plan,
        width=1.0,
        offset=0.02,
    )
    assert reversed_rail == rail_plan
    assert reversed_plan == attempt.plan
    assert _face_signature(reversed_faces) == _face_signature(preview)


def test_r1_exact_source_station_never_emits_zero_area_cell():
    _rail_plan, plan = _compile_geometry(
        planar_quad_strip(),
        alpha_budget=1.5,
    )

    faces = evaluate_planar_rail_geometry_plan(plan, width=2.0)

    assert len(faces) == 4
    assert all(_polygon_area(face) > 0.0 for face in faces)
    assert all(len(set(face.vert_keys)) == len(face.vert_keys) for face in faces)


def test_r1_every_emitted_face_carries_complete_immutable_provenance():
    _rail_plan, plan = _compile_geometry(
        planar_rf10_quarter_join(),
        alpha_budget=2.0,
    )
    faces = evaluate_planar_rail_geometry_plan(plan, width=3.0)

    assert faces
    assert all(
        face.rail_provenance is not None
        and face.rail_provenance.component_id == plan.component.component_id
        and face.rail_provenance.source_face_id == face.surface_id
        and face.rail_provenance.source_edge_ids
        and face.rail_provenance.boundary_path_ids
        and all(
            path_id[:1]
            in {
                ("ROUTE",),
                ("CORNER_PATH",),
                ("IN_PLANE_PATH",),
                ("IN_PLANE_JOIN_PATH",),
            }
            for path_id in face.rail_provenance.boundary_path_ids
        )
        and face.rail_provenance.station_keys
        for face in faces
    )


def test_r1_dynamic_bands_and_stable_acute_policy_fail_structurally():
    graph, _edge_ids, selected, _vertex_at = planar_quad_strip()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    dynamic = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
        dynamic_corner_bands=True,
    )
    assert dynamic.plan is None
    assert dynamic.failures[0].reason == "RAIL_GEOMETRY_DYNAMIC_BANDS_UNSUPPORTED"

    graph, _edge_ids, selected, _vertex_at = planar_acute_join()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.25,
    )
    acute = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
        split_angle=pi / 3.0,
    )
    assert acute.plan is None
    assert acute.failures[0].reason == "RAIL_GEOMETRY_A10_POLICY_UNSUPPORTED"
    assert dict(acute.failures[0].details)["extrusion_angle"] < pi / 3.0


def test_r1_foreign_pchain_is_a_materialized_boundary_not_a_crossing():
    graph, edge_ids, selected, vertex_at = planar_quad_strip()
    foreign_pair = (vertex_at[(1, 1)], vertex_at[(1, 2)])
    foreign_edge = edge_ids[tuple(sorted(foreign_pair))]
    graph.nodes[0].boundary_loops[0].chains.append(
        BoundaryChain(
            vert_indices=list(foreign_pair),
            edge_indices=[foreign_edge],
        )
    )
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.5,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )

    assert attempt.failures == ()
    assert rail_plan.routes == ()
    faces = evaluate_planar_rail_geometry_plan(attempt.plan, width=3.0)
    assert faces
    assert max(position.x for face in faces for position in face.positions) == 1.5
    limited_channels = [
        channel
        for channel in attempt.plan.channels
        if any(foreign_edge in cell.source_edge_ids for cell in channel.cells)
    ]
    assert len(limited_channels) == 1
    limited_faces = [
        face
        for face in faces
        if face.rail_provenance.channel_id == limited_channels[0].channel_id
    ]
    assert max(
        position.x for face in limited_faces for position in face.positions
    ) == 1.0


def test_rp_analytic_path_preserves_canonical_vertex_at_large_coordinates():
    graph, _edge_ids, selected, _vertex_at = planar_quad_strip()
    node = graph.nodes[0]
    node.mesh_verts = [
        Vector((point.x + 1.0e9, point.y - 1.0e9, point.z))
        for point in node.mesh_verts
    ]
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )

    assert attempt.failures == ()
    assert attempt.plan.cap_traces == ()
    faces = evaluate_planar_rail_geometry_plan(attempt.plan, width=1.0)
    _assert_unique_key_positions(faces)
    assert all(position.x >= 1.0e9 - 2.0 for face in faces for position in face.positions)


@pytest.mark.skip(reason="R3-scope: virtual caps and dam barriers are non-planar only")
def test_r3_virtual_cap_stops_on_existing_rail_and_dam_vertex():
    graph, _edge_ids, selected, _vertex_at = planar_quad_strip()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.5,
    )
    baseline = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    ).plan
    first_hit = baseline.cap_traces[0].pieces[0].end
    template = rail_plan.routes[0]
    outside_vertex = next(
        vertex.vertex_id
        for vertex in rail_plan.vertices
        if vertex.vertex_id not in baseline.component.vertex_ids
    )
    synthetic = replace(
        template,
        route_id=999,
        key=replace(
            template.key,
            side=replace(
                template.key.side,
                spine_vertex_id=outside_vertex,
            ),
        ),
        segments=(
            replace(template.segments[0], edge_id=first_hit.source_edge_id),
        ),
    )
    with_rail_barrier = replace(
        rail_plan,
        routes=rail_plan.routes + (synthetic,),
    )
    rail_attempt = compile_planar_rail_geometry_attempt(
        with_rail_barrier,
        edge_indices=selected,
    )
    assert rail_attempt.failures == ()
    assert rail_attempt.plan.cap_traces[0].termination == "DAM"
    assert len(rail_attempt.plan.cap_traces[0].pieces) == 1

    dam_station = replace(
        template.stations[0],
        source_vertex_id=first_hit.source_feature_id,
    )
    dam_route = replace(
        synthetic,
        segments=(),
        stations=(dam_station,),
        termination=decal_rails.RailTermination.DAM,
    )
    dam_event = decal_rails.RailEvent(
        decal_rails.RailEventKind.DAM,
        dam_route.route_id,
        dam_station.station_index,
    )
    with_dam_barrier = replace(
        rail_plan,
        routes=rail_plan.routes + (dam_route,),
        events=rail_plan.events + (dam_event,),
    )
    dam_attempt = compile_planar_rail_geometry_attempt(
        with_dam_barrier,
        edge_indices=selected,
    )
    assert dam_attempt.failures == ()
    assert dam_attempt.plan.cap_traces[0].termination == "DAM"
    assert len(dam_attempt.plan.cap_traces[0].pieces) == 1


def test_rp_closed_rf1_ring_uses_seven_analytic_channels_and_a10_corners():
    fixture = planar_rf1_ring()
    rail_plan, plan = _compile_geometry(fixture, alpha_budget=2.0)

    assert plan.component.is_closed is True
    assert plan.component.vertex_ids[0] == plan.component.vertex_ids[-1]
    assert rail_plan.routes == ()
    assert rail_plan.events == ()
    assert len(plan.channels) == 7
    assert {len(channel.cells) for channel in plan.channels} == {2}
    assert len(plan.corners) == 7
    assert {corner.policy for corner in plan.corners} == {
        "STABLE_A10_IN_PLANE"
    }
    assert len(plan.corner_partitions) == 7
    assert {partition.mode for partition in plan.corner_partitions} == {
        "OUTER_FILL"
    }
    expected_ratio = 1.0 / cos(pi / 7.0)
    assert all(
        isclose(partition.miter_ratio, expected_ratio, rel_tol=1e-12)
        for partition in plan.corner_partitions
    )
    assert plan.alpha_budget == rail_plan.alpha_budget

    faces = evaluate_planar_rail_geometry_plan(plan, width=2.0)
    assert len(faces) == 21
    source_vertices = {
        vertex.vertex_id: Vector(vertex.position)
        for vertex in rail_plan.vertices
    }
    source_edges = {edge.edge_id: edge for edge in rail_plan.edges}
    channels = {channel.channel_id: channel for channel in plan.channels}
    measured = []
    for face in faces:
        channel_id = face.rail_provenance.channel_id
        if channel_id is None:
            continue
        channel = channels[channel_id]
        spine = source_edges[channel.spine_edge_id]
        point_a, point_b = (
            source_vertices[vertex_id] for vertex_id in spine.vertex_ids
        )
        measured.extend(
            (
                _point_line_distance(position, point_a, point_b),
                channel.alpha_limit,
            )
            for position, u_frac in zip(face.positions, face.u_fracs)
            if isclose(
                abs(u_frac),
                channel.alpha_limit,
                abs_tol=1e-12,
            )
        )
    assert measured
    assert all(
        isclose(value, expected, abs_tol=1e-12)
        for value, expected in measured
    )
    assert _face_component_count(faces) == 1
    assert _boundary_loop_count(faces) == 2
    assert max(map(len, _edge_incidence(faces).values())) <= 2
    _assert_unique_key_positions(faces)

    with pytest.raises(RailGeometryEvaluationError) as exc_info:
        evaluate_planar_rail_geometry_plan(
            plan,
            width=2.0 * plan.alpha_budget + 0.01,
        )
    assert exc_info.value.failure.reason == "RAIL_GEOMETRY_DOMAIN_BUDGET_EXCEEDED"
    assert "DOMAIN_BUDGET_EXCEEDED" in exc_info.value.failure.reason


def test_r1_rf10_planar_join_shares_the_compiled_corner_rail():
    rail_plan, plan = _compile_geometry(
        planar_rf10_quarter_join(),
        alpha_budget=2.0,
    )
    assert len(plan.corners) == 1
    assert len(plan.channels) == 4
    assert {partition.mode for partition in plan.corner_partitions} == {
        "INNER_DIVIDER",
        "OUTER_FILL",
    }
    assert all(
        isclose(partition.miter_ratio, sqrt(2.0), rel_tol=1e-12)
        for partition in plan.corner_partitions
    )

    for width in (0.2, 0.5, 1.0, 2.0, 2.5, 3.0, 4.0):
        preview = evaluate_planar_rail_geometry_plan(
            plan,
            width=width,
            preview=True,
        )
        confirmed = evaluate_planar_rail_geometry_plan(
            plan,
            width=width,
            preview=False,
        )
        assert _face_signature(preview) == _face_signature(confirmed)
        assert _face_component_count(preview) == 1
        assert _boundary_loop_count(preview) == 1
        assert max(map(len, _edge_incidence(preview).values())) <= 2
        assert all(_polygon_area(face) > 0.0 for face in preview)
        _assert_unique_key_positions(preview)

    narrow = evaluate_planar_rail_geometry_plan(plan, width=0.5)
    assert len(narrow) == 6
    inner_partition = next(
        partition
        for partition in plan.corner_partitions
        if partition.mode == "INNER_DIVIDER"
    )
    inner_faces = [
        face
        for face in narrow
        if face.surface_id == inner_partition.owner_face_id
    ]
    assert len(inner_faces) == 2
    assert isclose(sum(map(_polygon_area, inner_faces)), 0.4375)
    outer_faces = [
        face for face in narrow if face.component_kind == "RAIL_CORNER"
    ]
    assert len(outer_faces) == 2
    assert isclose(sum(map(_polygon_area, outer_faces)), 0.0625)
    miter_position = next(
        position
        for outer_face in outer_faces
        for key, position in zip(outer_face.vert_keys, outer_face.positions)
        if key[0] == "rail-corner-frontier"
    )
    corner_position = next(
        position
        for outer_face in outer_faces
        for key, position in zip(outer_face.vert_keys, outer_face.positions)
        if key == ("rail-source-vertex", inner_partition.source_vertex_id)
    )
    assert isclose((miter_position - corner_position).length, 0.25 * sqrt(2.0))

    at_station = evaluate_planar_rail_geometry_plan(plan, width=2.0)
    station_corners = [
        face for face in at_station if face.component_kind == "RAIL_CORNER"
    ]
    assert sum(
        key[0] == "rail-source-vertex"
        for face in station_corners
        for key in face.vert_keys
    ) >= 4
    beyond_station = evaluate_planar_rail_geometry_plan(plan, width=3.0)
    extended_corners = [
        face for face in beyond_station if face.component_kind == "RAIL_CORNER"
    ]
    assert sum(
        key[0] == "rail-source-vertex"
        for face in extended_corners
        for key in face.vert_keys
    ) >= 4
    assert sum(
        key[0] == "rail-frontier"
        for face in extended_corners
        for key in face.vert_keys
    ) >= 2

    wide = evaluate_planar_rail_geometry_plan(plan, width=4.0)
    wide_corners = [
        face for face in wide if face.component_kind == "RAIL_CORNER"
    ]
    outer_partition = next(
        partition
        for partition in plan.corner_partitions
        if partition.mode == "OUTER_FILL"
    )
    channels = {channel.channel_id: channel for channel in plan.channels}
    segment_faces = {
        channels[outer_partition.previous_channel_id].initial_face_id,
        channels[outer_partition.next_channel_id].initial_face_id,
    }
    materialized_corner_faces = {
        face.surface_id for face in wide_corners
    }
    assert materialized_corner_faces
    assert materialized_corner_faces <= (
        set(outer_partition.owner_face_ids).difference(segment_faces)
    )
    source_faces = {face.face_id: face for face in rail_plan.faces}
    source_positions = {
        vertex.vertex_id: vertex.position for vertex in rail_plan.vertices
    }
    assert all(
        _point_in_source_face(
            position,
            source_faces[face.surface_id],
            source_positions,
        )
        for face in wide_corners
        for position in face.positions
    )


def test_r1_reversed_enumeration_is_bit_identical():
    graph, _edge_ids, selected, _vertex_at = planar_rf1_ring()
    forward_rail = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    forward = compile_planar_rail_geometry_attempt(
        forward_rail,
        edge_indices=selected,
    ).plan
    node = graph.nodes[0]
    node.mesh_tris.reverse()
    node.mesh_tri_face_indices.reverse()
    node.mesh_tri_edge_indices.reverse()
    reversed_rail = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=2.0,
    )
    reversed_plan = compile_planar_rail_geometry_attempt(
        reversed_rail,
        edge_indices=tuple(reversed(selected)),
    ).plan

    assert reversed_rail == forward_rail
    assert reversed_plan == forward
    reversed_faces = evaluate_planar_rail_geometry_plan(
        reversed_plan,
        width=3.0,
    )
    forward_faces = evaluate_planar_rail_geometry_plan(
        forward,
        width=3.0,
    )
    _assert_unique_key_positions(reversed_faces)
    _assert_unique_key_positions(forward_faces)
    assert _face_signature(reversed_faces) == _face_signature(forward_faces)


def test_r1_closed_station_origin_comes_from_chain_provenance_not_vertex_id():
    graph, _edge_ids, selected, _vertex_at = planar_rf1_ring()
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.5,
    )
    original = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    ).plan
    remap = {
        vertex.vertex_id: 1000 - vertex.vertex_id * 7
        for vertex in rail_plan.vertices
    }
    remapped_vertices = {
        remap[vertex.vertex_id]: vertex.position
        for vertex in rail_plan.vertices
    }
    remapped_faces = [
        tuple(remap[vertex_id] for vertex_id in face.vertex_ids)
        for face in rail_plan.faces
    ]
    source_chain = graph.nodes[0].boundary_loops[0].chains[0]
    remapped_path = tuple(remap[vertex_id] for vertex_id in source_chain.vert_indices)
    remapped_graph, _ids, remapped_selected = mesh_graph(
        remapped_vertices,
        remapped_faces,
        ((remapped_path, True),),
    )
    remapped_rail = decal_rails.compile_decal_rail_plan(
        remapped_graph,
        remapped_selected,
        alpha_budget=1.5,
    )
    remapped = compile_planar_rail_geometry_attempt(
        remapped_rail,
        edge_indices=remapped_selected,
    ).plan
    original_positions = {
        vertex.vertex_id: vertex.position for vertex in rail_plan.vertices
    }
    remapped_positions = {
        vertex.vertex_id: vertex.position for vertex in remapped_rail.vertices
    }

    assert tuple(
        original_positions[station.source_vertex_id]
        for station in original.component.stations
    ) == tuple(
        remapped_positions[station.source_vertex_id]
        for station in remapped.component.stations
    )
    assert tuple(station.s for station in original.component.stations) == tuple(
        station.s for station in remapped.component.stations
    )


def test_r1_rf10_corner_cells_are_reversed_enumeration_stable():
    graph, _edge_ids, selected, _vertex_at = planar_rf10_quarter_join()
    forward_rail = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    forward = compile_planar_rail_geometry_attempt(
        forward_rail,
        edge_indices=selected,
    ).plan
    node = graph.nodes[0]
    node.mesh_tris.reverse()
    node.mesh_tri_face_indices.reverse()
    node.mesh_tri_edge_indices.reverse()
    reversed_rail = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=2.0,
    )
    reversed_plan = compile_planar_rail_geometry_attempt(
        reversed_rail,
        edge_indices=tuple(reversed(selected)),
    ).plan

    assert reversed_rail == forward_rail
    assert reversed_plan == forward
    for width in (0.2, 0.5, 1.0, 2.0, 2.5, 3.0, 4.0):
        reversed_faces = evaluate_planar_rail_geometry_plan(
            reversed_plan,
            width=width,
        )
        forward_faces = evaluate_planar_rail_geometry_plan(
            forward,
            width=width,
        )
        _assert_unique_key_positions(reversed_faces)
        _assert_unique_key_positions(forward_faces)
        assert _face_signature(reversed_faces) == _face_signature(forward_faces)


def test_rp_concave_corner_source_face_is_triangulated_and_stable():
    graph, _edge_ids, selected, _vertex_at = planar_concave_corner_join()
    forward_rail = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    forward = compile_planar_rail_geometry_attempt(
        forward_rail,
        edge_indices=selected,
    )

    assert forward.failures == ()
    assert forward.plan is not None
    forward_faces = evaluate_planar_rail_geometry_plan(
        forward.plan,
        width=2.0,
    )
    assert forward_faces
    assert all(_polygon_area(face) > 0.0 for face in forward_faces)

    node = graph.nodes[0]
    node.mesh_tris.reverse()
    node.mesh_tri_face_indices.reverse()
    node.mesh_tri_edge_indices.reverse()
    reversed_rail = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=2.0,
    )
    reversed_attempt = compile_planar_rail_geometry_attempt(
        reversed_rail,
        edge_indices=tuple(reversed(selected)),
    )

    assert reversed_rail == forward_rail
    assert reversed_attempt.failures == ()
    assert reversed_attempt.plan == forward.plan
    reversed_faces = evaluate_planar_rail_geometry_plan(
        reversed_attempt.plan,
        width=2.0,
    )
    assert _face_signature(reversed_faces) == _face_signature(forward_faces)


def test_r1_disconnected_concave_face_does_not_reject_corner_compile():
    graph, _edge_ids, selected, _vertex_at = (
        planar_rf10_with_disconnected_concave_face()
    )
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )

    assert attempt.failures == ()
    assert attempt.plan is not None
    assert all(cell.owner_face_id != 1016 for cell in attempt.plan.corner_cells)


def test_r1_rf10_corner_fill_cannot_cross_foreign_closed_pchain():
    graph, _edge_ids, selected, _vertex_at = planar_rf10_quarter_join()
    baseline = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    blocked_face = next(face for face in baseline.faces if face.face_id == 1012)
    graph.nodes[0].boundary_loops[0].chains.append(
        BoundaryChain(
            vert_indices=list(blocked_face.vertex_ids),
            edge_indices=list(blocked_face.edge_ids),
            is_closed=True,
        )
    )
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )

    assert attempt.failures == ()
    assert all(
        cell.owner_face_id != blocked_face.face_id
        for cell in attempt.plan.corner_cells
    )
    wide = evaluate_planar_rail_geometry_plan(attempt.plan, width=4.0)
    assert all(
        not (
            face.component_kind == "RAIL_CORNER"
            and face.surface_id == blocked_face.face_id
        )
        for face in wide
    )


def test_r1_non_planar_source_face_is_structured_compile_failure():
    graph, _edge_ids, selected, vertex_at = planar_quad_strip()
    node = graph.nodes[0]
    local_index = node.mesh_vert_indices.index(vertex_at[(1, 1)])
    point = node.mesh_verts[local_index]
    node.mesh_verts[local_index] = Vector((point.x, point.y, 2.0))
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    attempt = compile_planar_rail_geometry_attempt(
        rail_plan,
        edge_indices=selected,
    )

    assert attempt.plan is None
    assert attempt.failures[0].reason == "RAIL_GEOMETRY_SOURCE_FACE_NON_PLANAR"
    assert attempt.failures[0].face_indices


def test_rp_curved_owner_is_deferred_to_r3_and_planar_fan_has_no_pole():
    graph, _edge_ids, selected, vertex_at = planar_quad_strip()
    node = graph.nodes[0]
    for coordinate in ((2, 0), (2, 1), (2, 2)):
        local_index = node.mesh_vert_indices.index(vertex_at[coordinate])
        point = node.mesh_verts[local_index]
        node.mesh_verts[local_index] = Vector((point.x, point.y, 2.0))
    curved_rail = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    curved = compile_planar_rail_geometry_attempt(
        curved_rail,
        edge_indices=selected,
    )
    assert curved.plan is None
    assert curved.failures[0].reason == "RAIL_GEOMETRY_CURVED_PENDING_R3"

    center = 0
    ring = (1, 2, 3, 4)
    vertices = {
        center: (0.0, 0.0, 0.0),
        1: (1.0, 0.0, 0.0),
        2: (0.0, 1.0, 0.0),
        3: (-1.0, 0.0, 0.0),
        4: (0.0, -1.0, 0.0),
    }
    faces = [
        (center, ring[index], ring[(index + 1) % len(ring)])
        for index in range(len(ring))
    ]
    pole_graph, _ids, pole_selected = mesh_graph(
        vertices,
        faces,
        (((1, 2, 3, 4), True),),
    )
    pole_rail = decal_rails.compile_decal_rail_plan(
        pole_graph,
        pole_selected,
        alpha_budget=3.0,
    )
    pole = compile_planar_rail_geometry_attempt(
        pole_rail,
        edge_indices=pole_selected,
    )
    assert pole.failures == ()
    assert pole.plan is not None
    assert pole_rail.routes == ()


def test_r1_merge_and_true_spine_branch_are_structured_failures():
    fold_graph, _fold_edges, fold_selected, _fold_vertices = (
        planar_rf13_shared_fold_boundary()
    )
    fold_plan = decal_rails.compile_decal_rail_plan(
        fold_graph,
        fold_selected,
        alpha_budget=1.0,
    )
    merge_route = replace(
        fold_plan.routes[0],
        termination=decal_rails.RailTermination.MERGE,
    )
    merged = compile_planar_rail_geometry_attempt(
        replace(fold_plan, routes=(merge_route,)),
        edge_indices=fold_selected,
    )
    assert merged.plan is None
    assert merged.failures[0].reason == "RAIL_GEOMETRY_MERGE_UNSUPPORTED"

    vertices = {
        0: (-1.0, 0.0, 0.0),
        1: (-2.0, -1.0, 0.0),
        2: (-2.0, 1.0, 0.0),
        3: (1.0, 0.0, 0.0),
        4: (2.0, -1.0, 0.0),
        5: (2.0, 1.0, 0.0),
        6: (0.0, 0.0, 0.0),
    }
    faces = [
        (0, 1, 6),
        (0, 6, 2),
        (3, 4, 6),
        (3, 6, 5),
    ]
    graph, _edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (
            ((1, 0, 2), False),
            ((4, 3, 5), False),
        ),
    )
    rail_plan = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(
            edge_id
            for chain in graph.nodes[0].boundary_loops[0].chains
            for edge_id in chain.edge_indices
        ),
        alpha_budget=4.0,
    )
    branch_use = replace(
        rail_plan.spine_uses[0],
        edge_id=999,
        vertex_ids=(rail_plan.spine_uses[0].vertex_ids[0], 999),
    )
    branched_plan = replace(
        rail_plan,
        spine_uses=rail_plan.spine_uses + (branch_use,),
        edges=rail_plan.edges
        + (
            replace(
                rail_plan.edges[0],
                edge_id=999,
                vertex_ids=branch_use.vertex_ids,
                is_spine=True,
            ),
        ),
    )
    branch_edges = tuple(use.edge_id for use in branched_plan.spine_uses[:2]) + (999,)
    branch = compile_planar_rail_geometry_attempt(
        branched_plan,
        edge_indices=branch_edges,
    )
    assert branch.plan is None
    assert branch.failures[0].reason in {
        "RAIL_GEOMETRY_BRANCH_UNSUPPORTED",
        "RAIL_GEOMETRY_COMPONENT_PARTIAL",
    }
