"""Unit-test builders for immutable PatchSurfaceIR snapshots."""

from __future__ import annotations

from mathutils import Vector

from cftuv.surface_ir import (
    AnalysisBundle,
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision,
    SourceVertex,
    SurfaceTriangle,
)


def attach_patch_surface(
    node,
    *,
    mesh_verts,
    mesh_vert_indices,
    mesh_tris,
    mesh_tri_face_indices,
    mesh_tri_face_normals=(),
    mesh_tri_edge_indices=(),
):
    """Attach a separate SurfaceIR to a topology-only PatchNode fixture."""

    positions = tuple(Vector(value) for value in mesh_verts)
    source_vertex_ids = tuple(int(value) for value in mesh_vert_indices)
    triangles = tuple(tuple(int(value) for value in tri) for tri in mesh_tris)
    face_ids = tuple(int(value) for value in mesh_tri_face_indices)
    supplied_face_normals = tuple(Vector(value) for value in mesh_tri_face_normals)
    physical_edges = tuple(mesh_tri_edge_indices)
    revision = SourceRevision("unit-fixture", f"patch:{node.patch_id}:{len(triangles)}")

    surface_triangles = []
    triangle_ids_by_face = {}
    vertices_by_face = {}
    edges_by_face = {}
    edge_vertices = {}
    edge_faces = {}
    for triangle_id, local_triangle in enumerate(triangles):
        vertex_ids = tuple(source_vertex_ids[index] for index in local_triangle)
        points = tuple(positions[index] for index in local_triangle)
        normal = (points[1] - points[0]).cross(points[2] - points[0])
        if normal.length_squared:
            normal.normalize()
        face_id = face_ids[triangle_id]
        edge_ids = (
            tuple(physical_edges[triangle_id])
            if len(physical_edges) == len(triangles)
            else (-1, -1, -1)
        )
        normalized_edge_ids = tuple(
            None if int(edge_id) < 0 else int(edge_id) for edge_id in edge_ids
        )
        surface_triangles.append(
            SurfaceTriangle(
                triangle_id=triangle_id,
                source_face_id=face_id,
                vertex_ids=vertex_ids,
                physical_edge_ids=normalized_edge_ids,
                triangle_normal=tuple(normal),
            )
        )
        triangle_ids_by_face.setdefault(face_id, []).append(triangle_id)
        vertices_by_face.setdefault(face_id, []).extend(vertex_ids)
        for opposite, edge_id in enumerate(normalized_edge_ids):
            if edge_id is None:
                continue
            endpoints = tuple(
                sorted(
                    (
                        vertex_ids[(opposite + 1) % 3],
                        vertex_ids[(opposite + 2) % 3],
                    )
                )
            )
            edge_vertices.setdefault(edge_id, endpoints)
            edge_faces.setdefault(edge_id, set()).add(face_id)
            edges_by_face.setdefault(face_id, []).append(edge_id)

    source_faces = []
    for face_offset, face_id in enumerate(sorted(triangle_ids_by_face)):
        vertex_cycle = tuple(dict.fromkeys(vertices_by_face[face_id]))
        if len(vertex_cycle) < 3:
            vertex_cycle = surface_triangles[triangle_ids_by_face[face_id][0]].vertex_ids
        normal = (
            supplied_face_normals[triangle_ids_by_face[face_id][0]]
            if len(supplied_face_normals) == len(triangles)
            else Vector(surface_triangles[triangle_ids_by_face[face_id][0]].triangle_normal)
        )
        source_faces.append(
            SourceFace(
                face_id=face_id,
                patch_id=int(node.patch_id),
                vertex_cycle=vertex_cycle,
                edge_cycle=tuple(dict.fromkeys(edges_by_face.get(face_id, ()))),
                polygon_normal=tuple(normal),
                triangle_ids=tuple(triangle_ids_by_face[face_id]),
            )
        )

    surface = PatchSurfaceIR(
        source_revision=revision,
        vertices=tuple(
            SourceVertex(vertex_id, tuple(positions[index]))
            for index, vertex_id in enumerate(source_vertex_ids)
        ),
        edges=tuple(
            SourceEdge(edge_id, edge_vertices[edge_id], tuple(sorted(edge_faces[edge_id])))
            for edge_id in sorted(edge_vertices)
        ),
        faces=tuple(source_faces),
        triangles=tuple(surface_triangles),
    )
    node._test_patch_surface = surface
    # A few algorithm-level tests still inspect/mutate their synthetic input.
    # These are fixture-only attributes, not PatchNode dataclass fields and
    # never cross a production boundary.
    node.mesh_verts = list(positions)
    node.mesh_vert_indices = list(source_vertex_ids)
    node.mesh_tris = list(triangles)
    node.mesh_tri_face_indices = list(face_ids)
    node.mesh_tri_face_normals = list(supplied_face_normals)
    node.mesh_tri_edge_indices = list(physical_edges)
    return node


def patch_surface(node):
    return node._test_patch_surface


def analysis_bundle_from_graph(graph):
    """Combine per-node fixture surfaces into one revision-consistent bundle."""

    if isinstance(graph, AnalysisBundle):
        return graph
    revision = SourceRevision("unit-fixture", f"graph:{sorted(graph.nodes)}")
    graph.source_revision = revision
    vertices = {}
    edges = {}
    edge_faces = {}
    faces = []
    triangles = []
    for patch_id in sorted(graph.nodes):
        node = graph.nodes[patch_id]
        if hasattr(node, "mesh_verts") and hasattr(node, "mesh_tris"):
            triangle_count = len(node.mesh_tris)
            face_ids = list(getattr(node, "mesh_tri_face_indices", ()))
            if len(face_ids) != triangle_count:
                available = list(node.face_indices) or [patch_id]
                face_ids = [
                    available[min(index, len(available) - 1)]
                    for index in range(triangle_count)
                ]
            source_ids = list(getattr(node, "mesh_vert_indices", ()))
            if len(source_ids) != len(node.mesh_verts):
                source_ids = [
                    (int(patch_id) + 1) * 1_000_000 + index
                    for index in range(len(node.mesh_verts))
                ]
                for boundary_loop in node.boundary_loops:
                    for vertex_id, coordinate in zip(
                        boundary_loop.vert_indices,
                        boundary_loop.vert_cos,
                    ):
                        matches = [
                            index
                            for index, position in enumerate(node.mesh_verts)
                            if tuple(position) == tuple(coordinate)
                        ]
                        if len(matches) == 1:
                            source_ids[matches[0]] = int(vertex_id)
                    for chain in boundary_loop.chains:
                        for vertex_id, coordinate in zip(
                            chain.vert_indices,
                            chain.vert_cos,
                        ):
                            matches = [
                                index
                                for index, position in enumerate(node.mesh_verts)
                                if tuple(position) == tuple(coordinate)
                            ]
                            if len(matches) == 1:
                                source_ids[matches[0]] = int(vertex_id)
            attach_patch_surface(
                node,
                mesh_verts=node.mesh_verts,
                mesh_vert_indices=source_ids,
                mesh_tris=node.mesh_tris,
                mesh_tri_face_indices=face_ids,
                mesh_tri_face_normals=getattr(node, "mesh_tri_face_normals", ()),
                mesh_tri_edge_indices=getattr(node, "mesh_tri_edge_indices", ()),
            )
        surface = getattr(node, "_test_patch_surface", None)
        if surface is None:
            continue
        for vertex in surface.vertices:
            vertices.setdefault(vertex.vertex_id, vertex.position)
        for edge in surface.edges:
            edges.setdefault(edge.edge_id, edge.vertex_ids)
            edge_faces.setdefault(edge.edge_id, set()).update(edge.source_face_ids)
        triangle_id_map = {}
        for triangle in surface.triangles:
            new_id = len(triangles)
            triangle_id_map[triangle.triangle_id] = new_id
            triangles.append(
                SurfaceTriangle(
                    triangle_id=new_id,
                    source_face_id=triangle.source_face_id,
                    vertex_ids=triangle.vertex_ids,
                    physical_edge_ids=triangle.physical_edge_ids,
                    triangle_normal=triangle.triangle_normal,
                )
            )
        faces.extend(
            SourceFace(
                face_id=face.face_id,
                patch_id=patch_id,
                vertex_cycle=face.vertex_cycle,
                edge_cycle=face.edge_cycle,
                polygon_normal=face.polygon_normal,
                triangle_ids=tuple(triangle_id_map[value] for value in face.triangle_ids),
            )
            for face in surface.faces
        )
    combined = PatchSurfaceIR(
        source_revision=revision,
        vertices=tuple(SourceVertex(key, vertices[key]) for key in sorted(vertices)),
        edges=tuple(
            SourceEdge(key, edges[key], tuple(sorted(edge_faces[key])))
            for key in sorted(edges)
        ),
        faces=tuple(sorted(faces, key=lambda face: face.face_id)),
        triangles=tuple(triangles),
    )
    for node in graph.nodes.values():
        for boundary_loop in node.boundary_loops:
            for chain in boundary_loop.chains:
                if len(chain.side_face_normals) < len(chain.edge_indices):
                    chain.side_face_normals = [
                        Vector(node.normal) for _edge_id in chain.edge_indices
                    ]
                if len(chain.side_face_indices) < len(chain.edge_indices):
                    default_face_id = int(node.face_indices[0]) if node.face_indices else -1
                    chain.side_face_indices = [
                        default_face_id for _edge_id in chain.edge_indices
                    ]
    return AnalysisBundle(revision, graph, combined)
