from __future__ import annotations

from types import SimpleNamespace

from mathutils import Vector

from cftuv import decal_rails
from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode
from decal_rail_fixtures import (
    planar_shallow_dihedral_strip,
    rr9_rf16_ambiguous_terminal_fan,
    rr9_rf16_terminal_fold_caps,
    rr9a_rf18_terminal_seam_snap,
    rr9b_rf20_l_turn_continuation,
    rr9b_rf20_terminal_continuation,
    rr9b_rf21_spine_reaches_mesh_border,
    rr9b_rf22_one_sided_boundary_continuation,
    rr8c_rf27_segmented_contour,
)


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
    for y in range(-1, 4):
        for x in range(-2, 3):
            vertex_at[(x, y)] = next_id
            vertices[next_id] = (float(x), float(y), 0.0)
            next_id += 1
    faces = []
    for y in range(-1, 3):
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


def test_rp_planar_quad_strip_has_no_routes_dams_or_caps():
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
    assert plan.routes == ()
    assert plan.events == ()
    assert plan.start_sectors == ()
    assert all(
        not edge.is_fold
        for edge in plan.edges
        if edge.edge_id not in selected
    )


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


def test_rp_planar_topology_noise_and_artist_mark_do_not_create_rails():
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
    assert blocked.routes == ()
    assert blocked.events == ()
    assert bridged.routes == ()
    assert bridged.events == ()
    assert bridged.start_sectors == ()


def test_rp_conflicting_marks_inside_planar_region_are_inert():
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
    assert sum(
        vertex_at[(1, 1)] in edge.vertex_ids for edge in plan.edges
    ) == 5
    assert plan.routes == ()
    assert plan.events == ()


def test_rp_artist_mark_cannot_create_planar_start_route():
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
    assert plan.routes == ()
    assert plan.events == ()
    assert plan.start_sectors == ()


def test_rp_foreign_planar_pchain_is_clip_topology_not_a_route():
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
    assert plan.routes == ()
    assert plan.events == ()
    assert next(edge for edge in plan.edges if edge.edge_id == foreign_edge).is_pchain


def test_rf14_rp_sector_fan_keeps_only_the_fold_boundary_route():
    center = 0
    spine_a = 1
    sector_a = 2
    boundary = 3
    sector_b = 4
    spine_b = 5
    vertices = {
        center: (0.0, 0.0, 0.0),
        spine_a: (-1.0, 0.0, 0.0),
        sector_a: (-1.0, 1.0, 0.0),
        boundary: (0.0, 1.0, 0.0),
        sector_b: (0.0, 1.0, 1.0),
        spine_b: (0.0, 0.0, 1.0),
    }
    faces = [
        (center, spine_a, sector_a),
        (center, sector_a, boundary),
        (center, boundary, sector_b),
        (center, sector_b, spine_b),
    ]
    spine_pairs = ((center, spine_a), (center, spine_b))
    boundary_pair = (center, boundary)
    graph, edge_ids = _mesh_graph(
        vertices,
        faces,
        spine_pairs,
        extra_pchain_pairs=(boundary_pair,),
    )
    selected = tuple(
        edge_ids[tuple(sorted(pair))] for pair in spine_pairs
    )
    boundary_edge = edge_ids[tuple(sorted(boundary_pair))]
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    center_routes = tuple(
        route
        for route in plan.routes
        if route.key.side.spine_vertex_id == center
    )

    assert len(center_routes) == 1
    assert center_routes[0].key.side.start_edge_id == boundary_edge
    assert plan.start_sectors == ()
    # RR8c: seam route приходит в физическую развилку двух border-рёбер.
    assert sum(
        event.kind == decal_rails.RailEventKind.DAM
        for event in plan.events
    ) == 1


def test_rp_closed_planar_fan_has_no_routes_or_poles():
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

    assert plan.routes == ()
    assert plan.events == ()
    assert plan.start_sectors == ()


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


def test_rr9_planar_border_routes_do_not_invent_merge_or_pole():
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
    assert len(plan.routes) == 4
    assert all(
        route.termination not in {
            decal_rails.RailTermination.MERGE,
            decal_rails.RailTermination.POLE,
        }
        for route in plan.routes
    )
    assert all(
        use.kind == decal_rails.RailTerminalKind.ROUTE
        for use in plan.terminal_uses
    )
    assert plan.start_sectors == ()


def test_user_four_degree_fold_threshold_is_canonical_for_rails():
    classifications = []
    for angle_degrees in (3.0, 5.0):
        graph, _edge_ids, selected, _vertex_at = planar_shallow_dihedral_strip(
            angle_degrees
        )
        plan = decal_rails.compile_decal_rail_plan(
            graph,
            selected,
            alpha_budget=0.5,
        )
        selected_edge = next(
            edge for edge in plan.edges if edge.edge_id == selected[0]
        )
        classifications.append(selected_edge.is_fold)

    assert classifications == [False, True]


def test_rr9_rf16_terminal_fold_routes_are_published_once_per_side():
    graph, edge_ids, selected, _vertex_at = rr9_rf16_terminal_fold_caps()
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
    )
    expected_edges = {
        edge_ids[(1, 3)],
        edge_ids[(1, 5)],
    }
    terminal = tuple(
        use for use in plan.terminal_uses if use.spine_vertex_id == 1
    )

    assert len(terminal) == 2
    assert {use.route_edge_id for use in terminal} == expected_edges
    assert all(use.kind == decal_rails.RailTerminalKind.ROUTE for use in terminal)
    assert all(use.route_id is not None for use in terminal)
    assert len({use.route_id for use in terminal}) == 2
    assert plan.terminal_fallback_counts == (("IN_PLANE_EMPTY", 1),)


def test_rr9_rf16_ambiguous_terminal_falls_back_counted_and_mark_bridges():
    graph, edge_ids, selected, _vertex_at = rr9_rf16_ambiguous_terminal_fan()
    ambiguous = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.5,
    )
    endpoint = next(
        use for use in ambiguous.terminal_uses if use.spine_vertex_id == 1
    )
    expected_candidates = {
        edge_ids[(1, 3)],
        edge_ids[(1, 4)],
    }

    assert endpoint.kind == decal_rails.RailTerminalKind.IN_PLANE_AMBIGUOUS
    assert set(endpoint.candidate_edge_ids) == expected_candidates
    assert endpoint.route_id is None
    assert ambiguous.terminal_fallback_counts == (
        ("IN_PLANE_AMBIGUOUS", 1),
    )

    marked_edge = min(expected_candidates)
    marked = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.5,
        rail_mark_edge_indices=(marked_edge,),
    )
    bridged = next(
        use for use in marked.terminal_uses if use.spine_vertex_id == 1
    )
    assert bridged.kind == decal_rails.RailTerminalKind.ROUTE
    assert bridged.route_edge_id == marked_edge
    assert bridged.route_id is not None


def test_rr9a_rf18_snap_selects_unique_wavefront_nearest_pchain():
    graph, edge_ids, selected, _vertex_at = rr9a_rf18_terminal_seam_snap()
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
    )
    near_edge = edge_ids[(1, 2)]
    oblique_edge = edge_ids[(1, 4)]
    patch_side = next(
        use
        for use in plan.terminal_uses
        if use.spine_vertex_id == 1 and 1000 in use.source_face_ids
    )

    assert patch_side.kind == decal_rails.RailTerminalKind.ROUTE
    assert patch_side.candidate_edge_ids == tuple(sorted((near_edge, oblique_edge)))
    assert patch_side.route_edge_id == near_edge
    assert patch_side.route_id is not None
    # RR8 не проигрывает RR9a: обе pChain опубликованы одной route каждая.
    routes_by_edge = {
        route.key.side.start_edge_id: route
        for route in plan.routes
        if route.key.side.spine_vertex_id == 1
    }
    assert near_edge in routes_by_edge
    assert oblique_edge in routes_by_edge
    assert routes_by_edge[near_edge].route_id == patch_side.route_id


def test_rr9a_rf18_exact_tie_is_counted_and_mark_beats_snap():
    tie_graph, tie_edges, tie_selected, _vertex_at = (
        rr9a_rf18_terminal_seam_snap(exact_tie=True)
    )
    tied = decal_rails.compile_decal_rail_plan(
        tie_graph,
        tie_selected,
        alpha_budget=0.75,
    )
    tied_side = next(
        use
        for use in tied.terminal_uses
        if use.spine_vertex_id == 1 and 1000 in use.source_face_ids
    )
    assert tied_side.kind == decal_rails.RailTerminalKind.SNAP_TIE_DAM
    assert tied_side.route_id is None
    assert dict(tied.terminal_fallback_counts)["SNAP_TIE_DAM"] == 1

    graph, edge_ids, selected, _vertex_at = rr9a_rf18_terminal_seam_snap()
    oblique_edge = edge_ids[(1, 4)]
    marked = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
        rail_mark_edge_indices=(oblique_edge,),
    )
    marked_side = next(
        use
        for use in marked.terminal_uses
        if use.spine_vertex_id == 1 and 1000 in use.source_face_ids
    )
    assert marked_side.kind == decal_rails.RailTerminalKind.ROUTE
    assert marked_side.route_edge_id == oblique_edge
    assert marked_side.route_id is not None


def test_rr9b_rf20_own_pchain_continuation_is_delimiter_not_guide():
    graph, edge_ids, selected, _vertex_at = rr9b_rf20_terminal_continuation()
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
    )
    continuation_edge = edge_ids[(1, 5)]
    terminal = tuple(
        use for use in plan.terminal_uses if use.spine_vertex_id == 1
    )

    assert len(terminal) == 2
    assert all(
        use.kind == decal_rails.RailTerminalKind.IN_PLANE_EMPTY
        for use in terminal
    )
    assert all(
        continuation_edge not in use.candidate_edge_ids for use in terminal
    )
    assert not any(
        route.key.side.spine_vertex_id == 1
        and route.key.side.start_edge_id == continuation_edge
        for route in plan.routes
    )


def test_rr9b_rf20_l_turn_of_own_pchain_remains_side_guide():
    graph, edge_ids, selected, _vertex_at = rr9b_rf20_l_turn_continuation()
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
    )
    turn_edge = edge_ids[(1, 2)]
    terminal = tuple(
        use for use in plan.terminal_uses if use.spine_vertex_id == 1
    )
    guided = tuple(use for use in terminal if use.route_edge_id == turn_edge)

    assert len(terminal) == 2
    assert len(guided) == 1
    assert guided[0].kind == decal_rails.RailTerminalKind.ROUTE
    assert guided[0].route_id is not None
    assert sum(
        use.kind == decal_rails.RailTerminalKind.IN_PLANE_EMPTY
        for use in terminal
    ) == 1


def test_rr9b_rf21_mesh_border_owns_both_terminal_guides():
    graph, edge_ids, selected, _vertex_at = rr9b_rf21_spine_reaches_mesh_border()
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
    )
    border_edges = {edge_ids[(1, 2)], edge_ids[(1, 4)]}
    terminal = tuple(
        use for use in plan.terminal_uses if use.spine_vertex_id == 1
    )

    assert len(terminal) == 2
    assert {use.route_edge_id for use in terminal} == border_edges
    assert all(use.kind == decal_rails.RailTerminalKind.ROUTE for use in terminal)
    assert all(use.route_id is not None for use in terminal)


def test_rr9b_rf22_one_sided_boundary_continuations_are_delimiters():
    graph, edge_ids, selected, _vertex_at = (
        rr9b_rf22_one_sided_boundary_continuation()
    )
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=0.75,
    )
    continuation_edges = {edge_ids[(0, 1)], edge_ids[(2, 3)]}
    terminal = tuple(
        use
        for use in plan.terminal_uses
        if use.spine_vertex_id in {1, 2}
    )

    assert len(terminal) == 2
    assert all(
        use.kind == decal_rails.RailTerminalKind.IN_PLANE_EMPTY
        for use in terminal
    )
    assert all(use.route_edge_id is None for use in terminal)
    assert all(
        continuation_edges.isdisjoint(use.candidate_edge_ids)
        for use in terminal
    )
    assert not any(
        route.key.side.spine_vertex_id in {1, 2}
        and route.key.side.start_edge_id in continuation_edges
        for route in plan.routes
    )


def test_rr8c_rf27_boundary_route_crosses_derived_chain_boundary():
    graph, edge_ids, selected, _vertex_at = rr8c_rf27_segmented_contour()
    plan = decal_rails.compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=2.0,
    )
    guide_edge = edge_ids[(0, 2)]
    continuation_edge = edge_ids[(2, 3)]
    route = next(
        route
        for route in plan.routes
        if route.key.side.start_edge_id == guide_edge
    )

    assert tuple(segment.edge_id for segment in route.segments) == (
        guide_edge,
        continuation_edge,
    )
    assert route.stations[-1].distance == 2.0
    assert route.stations[-1].source_edge_id == continuation_edge
    assert route.termination == decal_rails.RailTermination.ALPHA


def test_rr8c_rf27_contour_fork_is_counted_dam_without_hidden_choice():
    incoming = decal_rails.RailSourceEdge(
        100, (0, 1), (1000,), 1.0, is_pchain=True
    )
    branch_a = decal_rails.RailSourceEdge(
        101, (1, 2), (1000,), 1.0, is_pchain=True
    )
    branch_b = decal_rails.RailSourceEdge(
        102, (1, 3), (1001,), 1.0, is_pchain=True
    )
    topology = SimpleNamespace(
        vertex_edges={1: (100, 101, 102)},
        edge_by_id={
            edge.edge_id: edge
            for edge in (incoming, branch_a, branch_b)
        },
    )

    continuation, stop_reason, event_kind = (
        decal_rails._boundary_contour_continuation(
            topology,
            {},
            1,
            incoming.edge_id,
            (incoming.edge_id,),
        )
    )
    assert continuation is None
    assert stop_reason == decal_rails.RailTermination.DAM
    assert event_kind is None

    physical_end = SimpleNamespace(
        vertex_edges={1: (100,)},
        edge_by_id={100: incoming},
    )
    continuation, stop_reason, event_kind = (
        decal_rails._boundary_contour_continuation(
            physical_end,
            {},
            1,
            incoming.edge_id,
            (incoming.edge_id,),
        )
    )
    assert continuation is None
    assert stop_reason == decal_rails.RailTermination.PCHAIN
    assert event_kind is None


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
