"""Reusable topology fixtures for TRANCHE R acceptance."""

from __future__ import annotations

from math import cos, pi, sin

from mathutils import Vector

from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode


def mesh_graph(
    vertices,
    faces,
    spine_paths,
    *,
    extra_pchain_paths=(),
    face_patch_ids=None,
):
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
    graph = PatchGraph()
    if face_patch_ids is None:
        face_patch_ids = (0,) * len(faces)
    if len(face_patch_ids) != len(faces):
        raise ValueError("face_patch_ids must match faces")
    patch_by_face_id = {
        1000 + face_index: int(face_patch_ids[face_index])
        for face_index in range(len(faces))
    }
    for patch_id in sorted(set(patch_by_face_id.values())):
        triangle_indices = tuple(
            index
            for index, face_id in enumerate(triangle_faces)
            if patch_by_face_id[face_id] == patch_id
        )
        patch_vertex_ids = tuple(
            sorted(
                {
                    local_vertex_ids[local_index]
                    for triangle_index in triangle_indices
                    for local_index in triangles[triangle_index]
                }
            )
        )
        patch_local_by_global = {
            vertex_id: local_index
            for local_index, vertex_id in enumerate(patch_vertex_ids)
        }
        node = PatchNode(
            patch_id=patch_id,
            face_indices=sorted(
                face_id
                for face_id, owner_patch_id in patch_by_face_id.items()
                if owner_patch_id == patch_id
            ),
        )
        node.mesh_vert_indices = list(patch_vertex_ids)
        node.mesh_verts = [Vector(vertices[index]) for index in patch_vertex_ids]
        node.mesh_tris = [
            tuple(
                patch_local_by_global[local_vertex_ids[local_index]]
                for local_index in triangles[triangle_index]
            )
            for triangle_index in triangle_indices
        ]
        node.mesh_tri_face_indices = [
            triangle_faces[index] for index in triangle_indices
        ]
        node.mesh_tri_edge_indices = [
            triangle_edges[index] for index in triangle_indices
        ]
        # Для rail fixtures достаточно канонической pChain-провенанс на
        # каждом owner-patch; _source_chain_records дедуплицирует продолжения.
        node.boundary_loops = [BoundaryLoop(chains=list(chains))]
        graph.add_node(node)
    selected = tuple(chains[0].edge_indices) if chains else ()
    return graph, edge_ids, selected


def planar_quad_strip():
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


def planar_rf13_collinear_fan_face():
    """RF13/RV2: валидный face с нулевой первой fan-тройкой."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (1.0, 0.0, 0.0),
        2: (2.0, 0.0, 0.0),
        3: (2.0, 2.0, 0.0),
        4: (0.0, 2.0, 0.0),
    }
    graph, edge_ids, selected = mesh_graph(
        vertices,
        ((0, 1, 2, 3, 4),),
        (((4, 0), False),),
    )
    return graph, edge_ids, selected, vertices


def planar_rf13_with_remote_degenerate_face():
    """RF13/RV1: истинно плохая грань лежит вне rail-footprint."""

    graph, _edge_ids, _selected, vertex_at = planar_rf11_boundary_join()
    vertices = {
        vertex_id: tuple(graph.nodes[0].mesh_verts[local_index])
        for local_index, vertex_id in enumerate(graph.nodes[0].mesh_vert_indices)
    }
    vertices.update(
        {
            100: (20.0, 0.0, 0.0),
            101: (21.0, 0.0, 0.0),
            102: (22.0, 0.0, 0.0),
        }
    )
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
    faces.append((100, 101, 102))
    spine_path = (vertex_at[(0, 0)], vertex_at[(0, 1)])
    boundary_path = tuple(vertex_at[(x, 1)] for x in range(-2, 3))
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        ((spine_path, False),),
        extra_pchain_paths=((boundary_path, False),),
    )
    return graph, edge_ids, selected, vertex_at


def planar_rf13_degenerate_footprint_face():
    """RF13 negative: полностью коллинеарная грань входит в footprint."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (1.0, 0.0, 0.0),
        2: (2.0, 0.0, 0.0),
    }
    graph, edge_ids, selected = mesh_graph(
        vertices,
        ((0, 1, 2),),
        (((0, 1), False),),
    )
    return graph, edge_ids, selected, vertices


def planar_rf13_fold_boundary_join():
    """RR8a: pChain на 90-degree fold ограничивает один face-sector."""

    vertices = {}
    vertex_at = {}
    next_id = 0
    for row in range(3):
        for x in range(-2, 3):
            vertex_at[(x, row)] = next_id
            vertices[next_id] = (
                float(x),
                0.0 if row == 0 else 1.0,
                0.0 if row < 2 else 1.0,
            )
            next_id += 1
    faces = []
    for row in range(2):
        for x in range(-2, 2):
            faces.append(
                (
                    vertex_at[(x, row)],
                    vertex_at[(x + 1, row)],
                    vertex_at[(x + 1, row + 1)],
                    vertex_at[(x, row + 1)],
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


def planar_rf13_shared_fold_boundary():
    """RR8a: один fold-route служит двум perpendicular face-sector'ам."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (0.0, 1.0, 0.0),
        2: (0.0, 2.0, 0.0),
        3: (-1.0, 0.0, 0.0),
        4: (-1.0, 2.0, 0.0),
        5: (0.0, 0.0, 1.0),
        6: (0.0, 2.0, 1.0),
    }
    faces = (
        (3, 0, 1, 2, 4),
        (0, 5, 6, 2, 1),
    )
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
        extra_pchain_paths=(((0, 1, 2), False),),
    )
    return graph, edge_ids, selected, vertices


def planar_rf15_structural_cap_fan():
    """RM5a: perpendicular вне spine-face, cap идёт по fan-ребру."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (1.0, 0.0, 0.0),
        2: (1.0, 1.0, 0.0),
        3: (0.5, 1.0, 0.0),
        4: (0.0, 1.0, 0.0),
    }
    faces = (
        (0, 1, 2),
        (0, 2, 3),
        (0, 3, 4),
    )
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
        extra_pchain_paths=(
            ((0, 4), False),
            ((1, 2), False),
        ),
    )
    return graph, edge_ids, selected, vertices


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


def planar_shallow_dihedral_strip(angle_degrees=5.0):
    """Мелкий fold для проверки канонического angular threshold."""

    angle = float(angle_degrees) * pi / 180.0
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


def rr9_rf16_terminal_fold_caps():
    """RF16: endpoint rails диагональны spine-перпендикуляру на обеих сторонах."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (0.0, 1.0, 0.0),
        2: (-1.0, -0.5, 0.0),
        3: (-1.0, 1.5, 0.0),
        4: (1.0, -0.5, 0.0),
        5: (1.0, 1.5, 0.0),
        6: (-2.0, -1.0, 0.0),
        7: (0.0, -1.0, 0.0),
        8: (2.0, -1.0, 0.0),
        9: (-1.5, 2.0, 1.0),
        10: (1.5, 2.0, 1.0),
    }
    faces = [
        (2, 0, 1, 3),
        (0, 4, 5, 1),
        (6, 7, 0, 2),
        (7, 8, 4, 0),
        (3, 1, 9),
        (1, 5, 10),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr9_rf16_ambiguous_terminal_fan():
    """RF16-negative: один planar sector выходит к двум border-кандидатам."""

    vertices = {
        0: (0.0, -1.0, 0.0),
        1: (0.0, 0.0, 0.0),
        2: (1.0, 0.0, 0.0),
        3: (1.0, 1.0, 0.0),
        4: (1.0, -1.0, 0.0),
    }
    faces = [
        (0, 1, 2),
        (1, 3, 2),
        (1, 2, 4),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr9a_rf18_terminal_seam_snap(*, exact_tie=False):
    """RF18: два внешних pChain в одном owner-patch стороны endpoint."""

    vertices = {
        0: (0.0, -2.0, 0.0),
        1: (0.0, 0.0, 0.0),
        2: ((1.0, 1.0, 0.0) if exact_tie else (1.0, 0.1, 0.0)),
        3: (1.4, 1.0, 0.0),
        4: (-1.0, 1.0, 0.0) if exact_tie else (0.45, 1.0, 0.0),
        5: (-1.4, 0.0, 0.0),
    }
    faces = [
        (0, 1, 2),
        (2, 1, 3),
        (3, 1, 4),
        (4, 1, 5),
        (5, 1, 0),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
        extra_pchain_paths=(
            ((1, 2), False),
            ((1, 4), False),
        ),
        # Пара spine faces принадлежит разным patch. Оба внешних pChain
        # входят в patch 0; второй также ограничивает patch 1.
        face_patch_ids=(0, 0, 0, 0, 1),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr9b_rf20_terminal_continuation():
    """RF20-negative: unselected продолжение своей pChain делит стороны."""

    vertices = {
        0: (0.0, -1.0, 0.0),
        1: (0.0, 0.0, 0.0),
        2: (1.0, 0.0, 0.0),
        4: (-1.0, 0.0, 0.0),
        5: (0.0, 1.0, 0.0),
    }
    faces = [
        (0, 1, 2),
        (2, 1, 5),
        (5, 1, 4),
        (4, 1, 0),
    ]
    graph, edge_ids, _selected = mesh_graph(
        vertices,
        faces,
        (((0, 1, 5), False),),
        face_patch_ids=(0, 0, 1, 1),
    )
    selected = (edge_ids[(0, 1)],)
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr9b_rf20_l_turn_continuation():
    """RF20-positive: L-поворот своей pChain целиком лежит на стороне 0."""

    vertices = {
        0: (0.0, -1.0, 0.0),
        1: (0.0, 0.0, 0.0),
        2: (1.0, 0.0, 0.0),
        3: (1.0, 1.0, 0.0),
        5: (-1.0, 0.0, 0.0),
    }
    faces = [
        (0, 1, 2),
        (2, 1, 3),
        (3, 1, 5),
        (5, 1, 0),
    ]
    graph, edge_ids, _selected = mesh_graph(
        vertices,
        faces,
        (((0, 1, 2), False),),
        face_patch_ids=(0, 0, 0, 1),
    )
    selected = (edge_ids[(0, 1)],)
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr9b_rf21_spine_reaches_mesh_border():
    """RF21: конец seam внутри двух border-рёбер разных сторон."""

    vertices = {
        0: (0.0, -1.0, 0.0),
        1: (0.0, 0.0, 0.0),
        2: (-1.0, 0.0, 0.0),
        3: (-1.0, -1.0, 0.0),
        4: (1.0, 0.0, 0.0),
        5: (1.0, -1.0, 0.0),
    }
    faces = [
        (0, 1, 2, 3),
        (0, 5, 4, 1),
    ]
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
        face_patch_ids=(0, 1),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr9b_rf22_one_sided_boundary_continuation():
    """RF22: выбран середний отрезок односторонней boundary-pChain."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (1.0, 0.0, 0.0),
        2: (2.0, 0.0, 0.0),
        3: (3.0, 0.0, 0.0),
        4: (0.0, 1.0, 0.0),
        5: (1.0, 1.0, 0.0),
        6: (2.0, 1.0, 0.0),
        7: (3.0, 1.0, 0.0),
    }
    faces = [
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
    ]
    graph, edge_ids, _selected = mesh_graph(
        vertices,
        faces,
        (((0, 1, 2, 3), False),),
        face_patch_ids=(0, 0, 0),
    )
    selected = (edge_ids[(1, 2)],)
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr8b_rf26_unequal_boundary_reach():
    """RF26 geometry seed: два terminal route разной длины."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (1.0, 0.0, 0.0),
        2: (0.0, 1.0, 0.0),
        3: (1.0, 3.0, 0.0),
    }
    faces = ((0, 1, 3, 2),)
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
        extra_pchain_paths=(((0, 2), False),),
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr8c_rf27_segmented_contour():
    """RF27: guide-chain заканчивается, но физический contour идёт за угол."""

    vertices = {
        0: (0.0, 0.0, 0.0),
        1: (0.0, -1.0, 0.0),
        2: (1.0, 0.0, 0.0),
        3: (1.0, 2.0, 0.0),
        4: (0.0, 2.0, 0.0),
    }
    faces = ((1, 0, 2, 3, 4),)
    # Два BoundaryChain намеренно описывают один физический L-контур.
    extra_pchain_paths = (
        ((0, 2), False),
        ((2, 3), False),
    )
    graph, edge_ids, selected = mesh_graph(
        vertices,
        faces,
        (((0, 1), False),),
        extra_pchain_paths=extra_pchain_paths,
    )
    return graph, edge_ids, selected, {vertex_id: vertex_id for vertex_id in vertices}


def rr10_rf7_opposing_cylinder(station_count=8):
    """RF7/RF17: два seam-кольца связаны многорёберными нитями."""

    heights = (0.0, 1.0, 2.0)
    vertices = {}
    vertex_at = {}
    next_id = 0
    for height_index, height in enumerate(heights):
        for station in range(station_count):
            angle = 2.0 * pi * station / station_count
            vertex_at[(height_index, station)] = next_id
            vertices[next_id] = (
                cos(angle),
                sin(angle),
                height,
            )
            next_id += 1
    faces = []
    for height_index in range(len(heights) - 1):
        for station in range(station_count):
            next_station = (station + 1) % station_count
            faces.append(
                (
                    vertex_at[(height_index, station)],
                    vertex_at[(height_index, next_station)],
                    vertex_at[(height_index + 1, next_station)],
                    vertex_at[(height_index + 1, station)],
                )
            )
    rings = tuple(
        (
            tuple(vertex_at[(height_index, station)] for station in range(station_count)),
            True,
        )
        for height_index in (0, 2)
    )
    graph, edge_ids, _selected = mesh_graph(
        vertices,
        tuple(faces),
        rings,
    )
    selected = tuple(
        edge_ids[
            tuple(
                sorted(
                    (
                        vertex_at[(height_index, station)],
                        vertex_at[(height_index, (station + 1) % station_count)],
                    )
                )
            )
        ]
        for height_index in (0, 2)
        for station in range(station_count)
    )
    return graph, edge_ids, selected, vertex_at
