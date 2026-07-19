"""Reusable topology fixtures for TRANCHE R acceptance."""

from __future__ import annotations

from math import cos, pi, sin

from mathutils import Vector

from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode


def mesh_graph(vertices, faces, spine_paths, *, extra_pchain_paths=()):
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

    chains = []
    for path, is_closed in tuple(spine_paths) + tuple(extra_pchain_paths):
        pairs = list(zip(path, path[1:]))
        if is_closed:
            pairs.append((path[-1], path[0]))
        chains.append(
            BoundaryChain(
                vert_indices=list(path),
                edge_indices=[
                    edge_ids[tuple(sorted(pair))] for pair in pairs
                ],
                is_closed=is_closed,
            )
        )
    node = PatchNode(
        patch_id=0,
        face_indices=list(range(1000, 1000 + len(faces))),
    )
    node.mesh_vert_indices = list(local_vertex_ids)
    node.mesh_verts = [Vector(vertices[index]) for index in local_vertex_ids]
    node.mesh_tris = triangles
    node.mesh_tri_face_indices = triangle_faces
    node.mesh_tri_edge_indices = triangle_edges
    node.boundary_loops = [BoundaryLoop(chains=chains)]
    graph = PatchGraph()
    graph.add_node(node)
    selected = tuple(chains[0].edge_indices) if chains else ()
    return graph, edge_ids, selected


def planar_quad_strip():
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
    spine_path = (
        vertex_at[(0, 0)],
        vertex_at[(0, 1)],
        vertex_at[(0, 2)],
    )
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        ((spine_path, False),),
    )
    return graph, edge_ids, selected, vertex_at


def planar_rf1_ring(station_count=7):
    """Closed all-quad RF1: seven radial rails, two intervals each."""

    radii = (1.0, 2.0, 3.0)
    vertices = {}
    vertex_at = {}
    next_id = 0
    for ring_index, radius in enumerate(radii):
        for station in range(station_count):
            angle = 2.0 * pi * station / station_count
            vertex_at[(ring_index, station)] = next_id
            vertices[next_id] = (
                radius * cos(angle),
                radius * sin(angle),
                0.0,
            )
            next_id += 1
    faces = []
    for ring_index in range(len(radii) - 1):
        for station in range(station_count):
            next_station = (station + 1) % station_count
            faces.append(
                (
                    vertex_at[(ring_index, station)],
                    vertex_at[(ring_index + 1, station)],
                    vertex_at[(ring_index + 1, next_station)],
                    vertex_at[(ring_index, next_station)],
                )
            )
    spine_path = tuple(vertex_at[(0, station)] for station in range(station_count))
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        ((spine_path, True),),
    )
    return graph, edge_ids, selected, vertex_at


def planar_rf11_boundary_join():
    """RF11: открытый spine упирается в поперечный незаделенный pChain."""

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
    spine_path = (vertex_at[(0, 0)], vertex_at[(0, 1)])
    boundary_path = tuple(vertex_at[(x, 1)] for x in range(-2, 3))
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        ((spine_path, False),),
        extra_pchain_paths=((boundary_path, False),),
    )
    return graph, edge_ids, selected, vertex_at


def planar_rf10_quarter_join():
    """Настоящий grid-L: (0,-1) -> (0,0) -> (1,0)."""

    vertices = {}
    vertex_at = {}
    next_id = 0
    for y in range(-2, 3):
        for x in range(-2, 3):
            vertex_at[(x, y)] = next_id
            vertices[next_id] = (float(x), float(y), 0.0)
            next_id += 1
    faces = []
    for y in range(-2, 2):
        for x in range(-2, 2):
            faces.append(
                (
                    vertex_at[(x, y)],
                    vertex_at[(x + 1, y)],
                    vertex_at[(x + 1, y + 1)],
                    vertex_at[(x, y + 1)],
                )
            )
    spine_path = (
        vertex_at[(0, -1)],
        vertex_at[(0, 0)],
        vertex_at[(1, 0)],
    )
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        ((spine_path, False),),
    )
    return graph, edge_ids, selected, vertex_at


def planar_acute_join():
    """Две quad-грани вокруг 45-degree extrusion corner."""

    vertices = {
        0: (-2.0, 0.0, 0.0),
        1: (0.0, 0.0, 0.0),
        2: (-2.0, 2.0, 0.0),
        3: (0.0, 1.0, 0.0),
        4: (-2.0, 1.0, 0.0),
        5: (-0.5, -0.5, 0.0),
        6: (-2.5, 1.5, 0.0),
    }
    faces = [
        (0, 1, 3, 4),
        (1, 2, 6, 5),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1, 2), False),),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def planar_concave_corner_join():
    """OUTER_FILL пересекает concave source n-gon рядом с L-spine."""

    vertices = {
        0: (0.0, -1.0, 0.0),
        1: (0.0, 0.0, 0.0),
        2: (1.0, 0.0, 0.0),
        3: (0.0, 2.0, 0.0),
        4: (-2.0, 2.0, 0.0),
        5: (-2.0, 0.0, 0.0),
        6: (-1.25, 0.0, 0.0),
        7: (-1.25, 1.5, 0.0),
        8: (-0.75, 1.5, 0.0),
        9: (-0.75, 0.0, 0.0),
        10: (1.0, -1.0, 0.0),
        11: (1.0, 2.0, 0.0),
        12: (-2.0, -1.0, 0.0),
        13: (-1.25, -1.0, 0.0),
        14: (-0.75, -1.0, 0.0),
    }
    faces = [
        (4, 5, 6, 7, 8, 9, 1, 3),
        (6, 9, 8, 7),
        (12, 13, 6, 5),
        (13, 14, 9, 6),
        (14, 0, 1, 9),
        (0, 10, 2, 1),
        (1, 2, 11, 3),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1, 2), False),),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def planar_rf10_with_disconnected_concave_face():
    """RF10 плюс далёкая topology-disconnected concave face."""

    vertices = {}
    vertex_at = {}
    next_id = 0
    for y in range(-2, 3):
        for x in range(-2, 3):
            vertex_at[(x, y)] = next_id
            vertices[next_id] = (float(x), float(y), 0.0)
            next_id += 1
    faces = []
    for y in range(-2, 2):
        for x in range(-2, 2):
            faces.append(
                (
                    vertex_at[(x, y)],
                    vertex_at[(x + 1, y)],
                    vertex_at[(x + 1, y + 1)],
                    vertex_at[(x, y + 1)],
                )
            )
    concave_points = (
        (10.0, 0.0, 0.0),
        (13.0, 0.0, 0.0),
        (13.0, 3.0, 0.0),
        (12.0, 3.0, 0.0),
        (12.0, 1.0, 0.0),
        (11.0, 1.0, 0.0),
        (11.0, 3.0, 0.0),
        (10.0, 3.0, 0.0),
    )
    concave_ids = []
    for point in concave_points:
        concave_ids.append(next_id)
        vertices[next_id] = point
        next_id += 1
    faces.append(tuple(concave_ids))
    spine_path = (
        vertex_at[(0, -1)],
        vertex_at[(0, 0)],
        vertex_at[(1, 0)],
    )
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        ((spine_path, False),),
    )
    return graph, edge_ids, selected, vertex_at


def planar_dihedral_strip():
    """Две quad-плоскости сходятся на одном selected spine edge."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (0.0, 1.0, 0.0),
        2: (-1.0, 0.0, 0.0),
        3: (-1.0, 1.0, 0.0),
        4: (0.0, 0.0, 1.0),
        5: (0.0, 1.0, 1.0),
    }
    faces = [
        (2, 0, 1, 3),
        (0, 4, 5, 1),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def planar_shallow_dihedral_strip():
    """Пятиградусный fold, который нельзя группировать как coplanar rail."""

    angle = 5.0 * pi / 180.0
    outer = (cos(angle), 0.0, sin(angle))
    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (0.0, 1.0, 0.0),
        2: (-1.0, 0.0, 0.0),
        3: (-1.0, 1.0, 0.0),
        4: outer,
        5: (outer[0], 1.0, outer[2]),
    }
    faces = [
        (2, 0, 1, 3),
        (0, 4, 5, 1),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}
