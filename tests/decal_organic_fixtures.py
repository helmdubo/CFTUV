"""Детерминированные parametric fixtures для Tranche E."""

from __future__ import annotations

from math import cos, pi, sin, sqrt

from mathutils import Vector

from cftuv.decal_charts import ChartSiteSeed
from cftuv.model import BoundaryChain, BoundaryLoop, PatchNode


def _surface_fixture(
    height,
    *,
    alpha=1.0,
    width=2.4,
    length=4.0,
    width_segments=12,
    length_segments=20,
    patch_id=90,
):
    row_size = length_segments + 1
    positions = []
    for width_index in range(width_segments + 1):
        x = width * width_index / width_segments
        for length_index in range(length_segments + 1):
            y = length * (length_index / length_segments - 0.5)
            positions.append(Vector((x, y, height(x, y))))
    triangles = []
    face_ids = []
    face_normals = []
    physical_edges = set()
    for width_index in range(width_segments):
        for length_index in range(length_segments):
            lower = width_index * row_size + length_index
            upper = (width_index + 1) * row_size + length_index
            cell = (lower, lower + 1, upper + 1, upper)
            physical_edges.update(
                tuple(sorted((cell[index], cell[(index + 1) % 4])))
                for index in range(4)
            )
            cell_triangles = (
                (cell[0], cell[1], cell[2]),
                (cell[0], cell[2], cell[3]),
            )
            face_id = 1000 + width_index * length_segments + length_index
            for triangle in cell_triangles:
                triangles.append(triangle)
                face_ids.append(face_id)
                first, second, third = (
                    positions[index] for index in triangle
                )
                normal = (second - first).cross(third - first).normalized()
                face_normals.append(normal)
    edge_index_by_pair = {
        pair: 12000 + index
        for index, pair in enumerate(sorted(physical_edges))
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
        patch_id=patch_id,
        face_indices=sorted(set(face_ids)),
        centroid=sum(positions, Vector()) / len(positions),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        mesh_verts=positions,
        mesh_vert_indices=list(range(len(positions))),
        mesh_tris=triangles,
        mesh_tri_face_indices=face_ids,
        mesh_tri_face_normals=face_normals,
        mesh_tri_edge_indices=triangle_edge_indices,
    )
    edge_indices = tuple(
        edge_index_by_pair[(index, index + 1)]
        for index in range(length_segments)
    )
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=list(range(row_size)),
                    vert_cos=positions[:row_size],
                    edge_indices=list(edge_indices),
                    side_face_indices=[
                        1000 + index for index in range(length_segments)
                    ],
                    side_face_normals=[
                        face_normals[index * 2]
                        for index in range(length_segments)
                    ],
                )
            ]
        )
    ]
    seeds = tuple(
        ChartSiteSeed(
            edge_index=edge_indices[index],
            source_vertex_ids=(index, index + 1),
            source_face_id=1000 + index,
            chain_ref=(patch_id, 0, index),
        )
        for index in range(length_segments)
    )
    return node, seeds, alpha


def sphere_cap_fixture(radius=12.0, *, alpha=1.0, patch_id=90):
    def height(x, y):
        radial2 = x * x + y * y
        return radius - sqrt(max(radius * radius - radial2, 1e-9))

    scale = min(1.0, radius / 4.0)
    return _surface_fixture(
        height,
        alpha=alpha,
        width=2.4 * scale,
        length=4.0 * scale,
        patch_id=patch_id,
    )


def intermediate_dome_fixture(*, alpha=1.0, patch_id=94):
    radius = 3.0

    def height(x, y):
        return radius - sqrt(max(radius * radius - x * x - y * y, 1e-9))

    return _surface_fixture(
        height,
        alpha=alpha,
        width=1.8,
        length=3.2,
        patch_id=patch_id,
    )


def tight_sphere_fixture(*, alpha=1.0, patch_id=95):
    radius = 2.0

    def height(x, y):
        return radius - sqrt(max(radius * radius - x * x - y * y, 1e-9))

    return _surface_fixture(
        height,
        alpha=alpha,
        width=1.0,
        length=3.2,
        patch_id=patch_id,
    )


def cliff_fixture(*, alpha=1.0, patch_id=91):
    return _surface_fixture(
        lambda x, y: 0.16 * sin(x * 0.8) * cos(y * 0.55),
        alpha=alpha,
        patch_id=patch_id,
    )


def saddle_fixture(radius=12.0, *, alpha=1.0, patch_id=92):
    return _surface_fixture(
        lambda x, y: (x * x - y * y) / (2.0 * radius),
        alpha=alpha,
        patch_id=patch_id,
    )


def crumple_fixture(*, alpha=1.0, patch_id=93):
    return _surface_fixture(
        lambda x, y: 0.45 * sin(4.0 * pi * x) * sin(2.0 * pi * y),
        alpha=alpha,
        width_segments=20,
        length_segments=28,
        patch_id=patch_id,
    )


__all__ = [
    "cliff_fixture",
    "crumple_fixture",
    "intermediate_dome_fixture",
    "saddle_fixture",
    "sphere_cap_fixture",
    "tight_sphere_fixture",
]
