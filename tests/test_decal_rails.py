from __future__ import annotations

from mathutils import Vector

from cftuv import decal_rails
from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode


def _mesh_graph(vertices, faces, spine_pairs, *, extra_pchain_pairs=()):
    """Сериализует polygon faces тем же opposite-slot контрактом, что analysis."""

    canonical_pairs = {
        tuple(sorted((int(vertex_a), int(vertex_b))))
        for face in faces
        for vertex_a, vertex_b in zip(face, face[1:] + face[:1])
    }
    edge_ids = {
        pair: 100 + index for index, pair in enumerate(sorted(canonical_pairs))
    }
    local_vertex_ids = tuple(sorted(vertices))
    local_by_global = {
        vertex_id: local_index
        for local_index, vertex_id in enumerate(local_vertex_ids)
    }
    triangles = []
    triangle_faces = []
    triangle_edges = []
    for face_id, face in enumerate(faces, 1000):
        boundary_pairs = {
            tuple(sorted((vertex_a, vertex_b)))
            for vertex_a, vertex_b in zip(face, face[1:] + face[:1])
        }
        for index in range(1, len(face) - 1):
            global_triangle = (face[0], face[index], face[index + 1])
            triangles.append(
                tuple(local_by_global[vertex_id] for vertex_id in global_triangle)
            )
            triangle_faces.append(face_id)
            opposite_edges = []
            for opposite_slot in range(3):
                pair = tuple(
                    sorted(
                        (
                            global_triangle[(opposite_slot + 1) % 3],
                            global_triangle[(opposite_slot + 2) % 3],
                        )
                    )
                )
                opposite_edges.append(
                    edge_ids[pair] if pair in boundary_pairs else -1
                )
            triangle_edges.append(tuple(opposite_edges))

    pchain_pairs = tuple(spine_pairs) + tuple(extra_pchain_pairs)
    chain = BoundaryChain(
        vert_indices=[vertex_id for pair in pchain_pairs for vertex_id in pair],
        edge_indices=[edge_ids[tuple(sorted(pair))] for pair in pchain_pairs],
    )
    node = PatchNode(patch_id=0, face_indices=list(range(1000, 1000 + len(faces))))
    node.mesh_vert_indices = list(local_vertex_ids)
    node.mesh_verts = [Vector(vertices[vertex_id]) for vertex_id in local_vertex_ids]
    node.mesh_tris = triangles
    node.mesh_tri_face_indices = triangle_faces
    node.mesh_tri_edge_indices = triangle_edges
    node.boundary_loops = [BoundaryLoop(chains=[chain])]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, edge_ids


def _quad_strip(*, with_ambiguity=False):
    vertices = {}
    vertex_at = {}
    next_id = 0
    for y in range(3):
        for x in range(-2, 3):
            vertex_at[(x, y)] = next_id
            vertices[next_id] = (float(x), float(y), 0.0)
            next_id += 1
    faces = []
    for y in range(2):
        for x in range(-2, 2):
            faces.append(
                (
                    vertex_at[(x, y)],
                    vertex_at[(x + 1, y)],
                    vertex_at[(x + 1, y + 1)],
                    vertex_at[(x, y + 1)],
                )
            )
    if with_ambiguity:
        branch = next_id
        vertices[branch] = (1.5, 2.0, 1.0)
        faces.append(
            (
                vertex_at[(1, 1)],
                vertex_at[(1, 2)],
                branch,
            )
        )
    spine_pairs = (
        (vertex_at[(0, 0)], vertex_at[(0, 1)]),
        (vertex_at[(0, 1)], vertex_at[(0, 2)]),
    )
    graph, edge_ids = _mesh_graph(vertices, faces, spine_pairs)
    return graph, edge_ids, vertex_at


def _flow_routes(plan):
    return tuple(route for route in plan.routes if route.key.side.start_edge_id >= 0)


def test_r0_regular_quad_spine_traces_two_static_alpha_routes():
    graph, edge_ids, vertex_at = _quad_strip()
    selected = tuple(
        edge_ids[tuple(sorted(pair))]
        for pair in (
            (vertex_at[(0, 0)], vertex_at[(0, 1)]),
            (vertex_at[(0, 1)], vertex_at[(0, 2)]),
        )
    )

    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.5,
    )
    routes = _flow_routes(plan)

    assert len(routes) == 2
    assert {route.termination for route in routes} == {
        decal_rails.RailTermination.ALPHA
    }
    assert all(route.stations[-1].kind == decal_rails.RailStationKind.EDGE for route in routes)
    assert all(route.stations[-1].distance == 1.5 for route in routes)
    assert all(len(route.segments) == len(route.stations) - 1 for route in routes)
    assert all(route.key.side.source_face_ids for route in routes)
    assert all(
        segment.source_face_ids
        for route in routes
        for segment in route.segments
    )
    assert dict(plan.event_counts)["ALPHA"] == 2
    assert len([route for route in plan.routes if route.key.side.start_edge_id < 0]) == 2


def test_r0_reversed_selected_enumeration_is_bit_identical():
    graph, edge_ids, vertex_at = _quad_strip()
    selected = tuple(
        edge_ids[tuple(sorted(pair))]
        for pair in (
            (vertex_at[(0, 0)], vertex_at[(0, 1)]),
            (vertex_at[(0, 1)], vertex_at[(0, 2)]),
        )
    )

    forward = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    reversed_plan = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=2.0,
    )

    assert reversed_plan == forward

    node = next(iter(graph.nodes.values()))
    node.mesh_tris.reverse()
    node.mesh_tri_face_indices.reverse()
    node.mesh_tri_edge_indices.reverse()
    node.boundary_loops[0].chains[0].edge_indices.reverse()
    reordered_topology = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=2.0,
    )
    assert reordered_topology == forward


def test_r0_shallow_nonplanar_ngon_normal_is_permutation_stable():
    graph, edge_ids, vertex_at = _quad_strip()
    selected = tuple(
        edge_ids[tuple(sorted(pair))]
        for pair in (
            (vertex_at[(0, 0)], vertex_at[(0, 1)]),
            (vertex_at[(0, 1)], vertex_at[(0, 2)]),
        )
    )
    node = graph.nodes[0]
    lifted_vertex = vertex_at[(1, 1)]
    local_index = node.mesh_vert_indices.index(lifted_vertex)
    point = node.mesh_verts[local_index]
    node.mesh_verts[local_index] = Vector((point.x, point.y, 1.0e-6))

    forward = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    node.mesh_tris.reverse()
    node.mesh_tri_face_indices.reverse()
    node.mesh_tri_edge_indices.reverse()
    reversed_plan = decal_rails.compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=1.0,
    )

    assert reversed_plan == forward
    assert any(face.planarity_min_dot < 1.0 for face in forward.faces)


def test_r0_non_quad_ambiguity_is_dam_until_artist_mark_bridges_it():
    graph, edge_ids, vertex_at = _quad_strip(with_ambiguity=True)
    selected = tuple(
        edge_ids[tuple(sorted(pair))]
        for pair in (
            (vertex_at[(0, 0)], vertex_at[(0, 1)]),
            (vertex_at[(0, 1)], vertex_at[(0, 2)]),
        )
    )
    right_edge = edge_ids[
        tuple(sorted((vertex_at[(1, 1)], vertex_at[(2, 1)])))
    ]

    blocked = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.75,
    )
    bridged = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.75,
        rail_mark_edge_indices=(right_edge,),
    )
    blocked_right = next(
        route
        for route in _flow_routes(blocked)
        if route.stations[1].source_vertex_id == vertex_at[(1, 1)]
    )
    bridged_right = next(
        route
        for route in _flow_routes(bridged)
        if route.stations[1].source_vertex_id == vertex_at[(1, 1)]
    )

    assert blocked_right.termination == decal_rails.RailTermination.DAM
    assert blocked_right.stations[-1].source_vertex_id == vertex_at[(1, 1)]
    assert bridged_right.termination == decal_rails.RailTermination.ALPHA
    assert bridged_right.stations[-1].kind == decal_rails.RailStationKind.EDGE
    assert dict(bridged.event_counts)["MARK"] == 1


def test_r0_conflicting_artist_marks_are_a_visible_dam():
    graph, edge_ids, vertex_at = _quad_strip(with_ambiguity=True)
    selected = tuple(
        edge_ids[tuple(sorted(pair))]
        for pair in (
            (vertex_at[(0, 0)], vertex_at[(0, 1)]),
            (vertex_at[(0, 1)], vertex_at[(0, 2)]),
        )
    )
    marked = (
        edge_ids[tuple(sorted((vertex_at[(1, 1)], vertex_at[(2, 1)])))],
        edge_ids[tuple(sorted((vertex_at[(1, 1)], vertex_at[(1, 2)])))],
    )

    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.75,
        rail_mark_edge_indices=marked,
    )
    right = next(
        route
        for route in _flow_routes(plan)
        if route.stations[1].source_vertex_id == vertex_at[(1, 1)]
    )

    assert sum(
        vertex_at[(1, 1)] in edge.vertex_ids for edge in plan.edges
    ) == 5
    assert right.termination == decal_rails.RailTermination.DAM
    assert right.stations[-1].source_vertex_id == vertex_at[(1, 1)]


def test_r0_artist_mark_can_bridge_missing_start_junction():
    graph, edge_ids, vertex_at = _quad_strip()
    selected_pairs = (
        (vertex_at[(0, 0)], vertex_at[(0, 1)]),
        (vertex_at[(0, 1)], vertex_at[(0, 2)]),
    )
    selected = tuple(
        edge_ids[tuple(sorted(pair))] for pair in selected_pairs
    )
    endpoint_mark = edge_ids[
        tuple(sorted((vertex_at[(0, 0)], vertex_at[(1, 0)])))
    ]

    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.5,
        rail_mark_edge_indices=(endpoint_mark,),
    )
    endpoint_route = next(
        route
        for route in _flow_routes(plan)
        if route.key.side.spine_vertex_id == vertex_at[(0, 0)]
    )

    assert endpoint_route.key.side.start_edge_id == endpoint_mark
    assert endpoint_route.termination == decal_rails.RailTermination.ALPHA
    assert endpoint_route.segments[0].edge_id == endpoint_mark
    assert endpoint_route.segments[0].source_face_ids


def test_r0_pchain_boundary_beats_exact_alpha_equality():
    graph, edge_ids, vertex_at = _quad_strip()
    selected = tuple(
        edge_ids[tuple(sorted(pair))]
        for pair in (
            (vertex_at[(0, 0)], vertex_at[(0, 1)]),
            (vertex_at[(0, 1)], vertex_at[(0, 2)]),
        )
    )
    foreign_edge = edge_ids[
        tuple(sorted((vertex_at[(1, 1)], vertex_at[(1, 2)])))
    ]
    graph.nodes[0].boundary_loops[0].chains.append(
        BoundaryChain(
            vert_indices=(vertex_at[(1, 1)], vertex_at[(1, 2)]),
            edge_indices=(foreign_edge,),
        )
    )

    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=1.0,
    )
    right = next(
        route
        for route in _flow_routes(plan)
        if route.stations[-1].source_vertex_id == vertex_at[(1, 1)]
    )

    assert right.stations[-1].distance == 1.0
    assert right.termination == decal_rails.RailTermination.PCHAIN
    assert dict(plan.event_counts)["PCHAIN"] >= 1


def test_r0_closed_fan_has_topological_pole_instead_of_merge():
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
    spine_pairs = tuple(
        (ring[index], ring[(index + 1) % len(ring)])
        for index in range(len(ring))
    )
    graph, edge_ids = _mesh_graph(vertices, faces, spine_pairs)
    selected = tuple(edge_ids[tuple(sorted(pair))] for pair in spine_pairs)

    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=3.0,
    )

    assert len(plan.routes) == 4
    assert all(route.termination == decal_rails.RailTermination.POLE for route in plan.routes)
    assert all(route.stations[-1].source_vertex_id == center for route in plan.routes)
    assert dict(plan.event_counts)["POLE"] == 4
    assert "MERGE" not in dict(plan.event_counts)


def test_r0_merge_trims_only_later_route_and_keeps_canonical_owner():
    key_a = decal_rails.RailRouteKey(decal_rails.RailSideKey(1, 10))
    key_b = decal_rails.RailRouteKey(decal_rails.RailSideKey(2, 20))
    first = decal_rails.RailRoute(
        0,
        key_a,
        (
            decal_rails.RailStation(0, 0.0, decal_rails.RailStationKind.VERTEX, source_vertex_id=1),
            decal_rails.RailStation(1, 1.0, decal_rails.RailStationKind.VERTEX, source_vertex_id=3),
            decal_rails.RailStation(2, 2.0, decal_rails.RailStationKind.VERTEX, source_vertex_id=4),
        ),
        (
            decal_rails.RailRouteSegment(10, 0, 1, (100,)),
            decal_rails.RailRouteSegment(11, 1, 2, (101,)),
        ),
        decal_rails.RailTermination.DAM,
    )
    second = decal_rails.RailRoute(
        1,
        key_b,
        (
            decal_rails.RailStation(0, 0.0, decal_rails.RailStationKind.VERTEX, source_vertex_id=2),
            decal_rails.RailStation(1, 1.0, decal_rails.RailStationKind.VERTEX, source_vertex_id=3),
            decal_rails.RailStation(2, 2.0, decal_rails.RailStationKind.VERTEX, source_vertex_id=4),
        ),
        (
            decal_rails.RailRouteSegment(20, 0, 1, (200,)),
            decal_rails.RailRouteSegment(11, 1, 2, (101,)),
        ),
        decal_rails.RailTermination.DAM,
    )

    routes, events = decal_rails._apply_poles_and_merges((first, second), ())

    assert routes[0].stations == first.stations
    assert routes[1].termination == decal_rails.RailTermination.MERGE
    assert len(routes[1].stations) == 2
    assert routes[1].segments == second.segments[:1]
    assert events == (
        decal_rails.RailEvent(decal_rails.RailEventKind.MERGE, 1, 1),
    )


def test_r0_pchain_boundary_cannot_be_relabelled_as_merge():
    owner = decal_rails.RailRoute(
        0,
        decal_rails.RailRouteKey(decal_rails.RailSideKey(3, 10)),
        (
            decal_rails.RailStation(
                0,
                0.0,
                decal_rails.RailStationKind.VERTEX,
                source_vertex_id=3,
            ),
        ),
        (),
        decal_rails.RailTermination.DAM,
    )
    boundary = decal_rails.RailRoute(
        1,
        decal_rails.RailRouteKey(decal_rails.RailSideKey(2, 20)),
        (
            decal_rails.RailStation(
                0,
                0.0,
                decal_rails.RailStationKind.VERTEX,
                source_vertex_id=2,
            ),
            decal_rails.RailStation(
                1,
                1.0,
                decal_rails.RailStationKind.VERTEX,
                source_vertex_id=3,
            ),
        ),
        (decal_rails.RailRouteSegment(20, 0, 1, (200,)),),
        decal_rails.RailTermination.PCHAIN,
    )
    pchain_event = decal_rails.RailEvent(
        decal_rails.RailEventKind.PCHAIN,
        1,
        1,
    )

    routes, events = decal_rails._apply_poles_and_merges(
        (owner, boundary),
        (pchain_event,),
    )

    assert routes[1].termination == decal_rails.RailTermination.PCHAIN
    assert events == (pchain_event,)


def test_r0_public_compile_detects_two_rail_merge():
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
    spine_pairs = ((0, 1), (0, 2), (3, 4), (3, 5))
    graph, edge_ids = _mesh_graph(vertices, faces, spine_pairs)
    selected = tuple(edge_ids[tuple(sorted(pair))] for pair in spine_pairs)

    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=4.0,
    )
    flow_routes = _flow_routes(plan)

    assert len(flow_routes) == 2
    assert {route.termination for route in flow_routes} == {
        decal_rails.RailTermination.DAM,
        decal_rails.RailTermination.MERGE,
    }
    merged = next(
        route
        for route in flow_routes
        if route.termination == decal_rails.RailTermination.MERGE
    )
    assert merged.stations[-1].source_vertex_id == 6
    assert dict(plan.event_counts)["MERGE"] == 1


def test_r0_missing_selected_edge_is_structured_compile_failure():
    graph, _edge_ids, _vertex_at = _quad_strip()

    attempt = decal_rails.compile_decal_rail_attempt(
        graph,
        (999999,),
        alpha_budget=1.0,
    )

    assert attempt.plan is None
    assert [failure.reason for failure in attempt.failures] == [
        "SELECTED_SPINE_EDGE_MISSING"
    ]
    assert attempt.failures[0].edge_indices == (999999,)
