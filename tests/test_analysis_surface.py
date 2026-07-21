from __future__ import annotations

from types import SimpleNamespace

import pytest
from mathutils import Vector

from cftuv.analysis_surface import (
    build_patch_surface_ir,
    source_revision_from_bmesh,
    validate_analysis_bundle,
)
from cftuv.model import BoundaryChain, BoundaryLoop, PatchGraph, PatchNode
from cftuv.surface_ir import (
    AnalysisBundle,
    AnalysisCapabilities,
    AnalysisCrossIrError,
    AnalysisSchemaError,
    SourceRevision,
)


class _Sequence(list):
    def ensure_lookup_table(self):
        pass


def _fake_bmesh(points, face_cycle, triangle_cycles):
    vertices = _Sequence(
        SimpleNamespace(index=index, co=Vector(point), link_faces=[])
        for index, point in enumerate(points)
    )
    edge_by_pair = {}
    edges = _Sequence()
    for first, second in zip(face_cycle, face_cycle[1:] + face_cycle[:1]):
        edge = SimpleNamespace(
            index=len(edges),
            verts=(vertices[first], vertices[second]),
        )
        edges.append(edge)
        edge_by_pair[frozenset((first, second))] = edge
    face = SimpleNamespace(index=0, edges=tuple(edges))
    face.loops = tuple(
        SimpleNamespace(vert=vertices[vertex_id], edge=edges[index], face=face)
        for index, vertex_id in enumerate(face_cycle)
    )
    face.verts = tuple(vertices[vertex_id] for vertex_id in face_cycle)
    for vertex in face.verts:
        vertex.link_faces.append(face)
    # Polygon normal intentionally differs from non-planar triangle normals.
    face.normal = Vector((0.0, 0.0, 1.0))
    faces = _Sequence([face])

    def calc_loop_triangles():
        loop_by_vertex = {loop.vert.index: loop for loop in face.loops}
        return tuple(
            tuple(loop_by_vertex[vertex_id] for vertex_id in triangle)
            for triangle in triangle_cycles
        )

    return SimpleNamespace(
        verts=vertices,
        edges=edges,
        faces=faces,
        calc_loop_triangles=calc_loop_triangles,
    )


def _analysis_bundle(points, face_cycle, triangle_cycles):
    revision = SourceRevision("fixture", "rev-1")
    graph = PatchGraph(source_revision=revision)
    node = PatchNode(patch_id=0, face_indices=[0])
    node.boundary_loops = [
        BoundaryLoop(
            vert_indices=list(face_cycle),
            vert_cos=[Vector(points[index]) for index in face_cycle],
            edge_indices=list(range(len(face_cycle))),
            chains=[
                BoundaryChain(
                    vert_indices=list(face_cycle) + [face_cycle[0]],
                    vert_cos=[Vector(points[index]) for index in face_cycle]
                    + [Vector(points[face_cycle[0]])],
                    edge_indices=list(range(len(face_cycle))),
                    side_face_indices=[0] * len(face_cycle),
                    side_face_normals=[Vector((0.0, 0.0, 1.0))]
                    * len(face_cycle),
                )
            ],
        )
    ]
    graph.add_node(node)
    bm = _fake_bmesh(points, list(face_cycle), triangle_cycles)
    surface = build_patch_surface_ir(bm, graph, revision)
    bundle = AnalysisBundle(revision, graph, surface)
    validate_analysis_bundle(bundle)
    return bundle


def _triangle_area_xy(points, triangle):
    first, second, third = (points[index] for index in triangle)
    return abs(
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    ) * 0.5


def _point_in_polygon_xy(point, polygon):
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        if (first[1] > point[1]) == (second[1] > point[1]):
            continue
        crossing_x = first[0] + (
            (point[1] - first[1])
            * (second[0] - first[0])
            / (second[1] - first[1])
        )
        if crossing_x >= point[0]:
            inside = not inside
    return inside


def test_concave_u_ngon_preserves_source_cycle_and_blender_tessellation_area():
    points = (
        (0.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
        (3.0, 3.0, 0.0),
        (2.0, 3.0, 0.0),
        (2.0, 1.0, 0.0),
        (1.0, 1.0, 0.0),
        (1.0, 3.0, 0.0),
        (0.0, 3.0, 0.0),
    )
    # A valid triangulation of the U polygon; this is the fake Blender output,
    # not a fan reconstructed by production code.
    triangles = (
        (0, 1, 4),
        (0, 4, 5),
        (0, 5, 7),
        (5, 6, 7),
        (1, 2, 3),
        (1, 3, 4),
    )
    bundle = _analysis_bundle(points, tuple(range(8)), triangles)

    source_face = bundle.patch_surface.faces[0]
    emitted = tuple(
        triangle.vertex_ids for triangle in bundle.patch_surface.triangles
    )
    assert source_face.vertex_cycle == tuple(range(8))
    assert emitted == triangles
    assert sum(_triangle_area_xy(points, tri) for tri in emitted) == pytest.approx(7.0)
    polygon = tuple(points[index] for index in source_face.vertex_cycle)
    assert all(
        _point_in_polygon_xy(
            tuple(
                sum(points[vertex_id][axis] for vertex_id in triangle) / 3.0
                for axis in range(2)
            ),
            polygon,
        )
        for triangle in emitted
    )
    assert all(
        edge_id is None
        for triangle in bundle.patch_surface.triangles
        for edge_id in triangle.physical_edge_ids
        if edge_id not in source_face.edge_cycle
    )


def test_non_planar_quad_keeps_real_per_triangle_normals():
    points = (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.4),
        (0.0, 1.0, 0.0),
    )
    bundle = _analysis_bundle(points, (0, 1, 2, 3), ((0, 1, 2), (0, 2, 3)))
    normals = tuple(
        triangle.triangle_normal for triangle in bundle.patch_surface.triangles
    )
    assert normals[0] != normals[1]
    assert all(normal != (0.0, 0.0, 1.0) for normal in normals)


def test_reversed_winding_preserves_area_and_flips_triangle_normals():
    points = (
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    forward = _analysis_bundle(points, (0, 1, 2, 3), ((0, 1, 2), (0, 2, 3)))
    reverse = _analysis_bundle(points, (3, 2, 1, 0), ((3, 2, 1), (3, 1, 0)))
    assert sum(
        _triangle_area_xy(points, triangle.vertex_ids)
        for triangle in forward.patch_surface.triangles
    ) == pytest.approx(2.0)
    assert sum(
        _triangle_area_xy(points, triangle.vertex_ids)
        for triangle in reverse.patch_surface.triangles
    ) == pytest.approx(2.0)
    assert forward.patch_surface.triangles[0].triangle_normal[2] == pytest.approx(1.0)
    assert reverse.patch_surface.triangles[0].triangle_normal[2] == pytest.approx(-1.0)


def test_analysis_bundle_rejects_revision_and_schema_mismatch():
    bundle = _analysis_bundle(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        (0, 1, 2),
        ((0, 1, 2),),
    )
    with pytest.raises(AnalysisCrossIrError, match="REVISION_IDENTITY"):
        AnalysisBundle(
            SourceRevision("fixture", "different-object"),
            bundle.patch_graph,
            bundle.patch_surface,
        )
    with pytest.raises(AnalysisSchemaError, match="DECAL_ANALYSIS_SCHEMA_UNSUPPORTED"):
        AnalysisBundle(
            bundle.source_revision,
            bundle.patch_graph,
            bundle.patch_surface,
            AnalysisCapabilities(patch_surface_schema=2),
        )


def test_source_revision_covers_scope_coordinates_topology_and_seams():
    bm = _fake_bmesh(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        [0, 1, 2],
        ((0, 1, 2),),
    )
    for edge in bm.edges:
        edge.seam = False
    baseline = source_revision_from_bmesh(bm, face_indices=(0,))
    assert source_revision_from_bmesh(bm, face_indices=()).digest != baseline.digest

    bm.edges[0].seam = True
    seam_changed = source_revision_from_bmesh(bm, face_indices=(0,))
    assert seam_changed.digest != baseline.digest

    bm.edges[0].seam = False
    bm.verts[0].co = Vector((0.25, 0.0, 0.0))
    coordinate_changed = source_revision_from_bmesh(bm, face_indices=(0,))
    assert coordinate_changed.digest != baseline.digest


def test_analysis_bundle_rejects_missing_per_segment_owner_normals():
    bundle = _analysis_bundle(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        (0, 1, 2),
        ((0, 1, 2),),
    )
    bundle.patch_graph.nodes[0].boundary_loops[0].chains[0].side_face_normals = []

    with pytest.raises(
        AnalysisSchemaError,
        match="DECAL_ANALYSIS_SCHEMA_UNSUPPORTED",
    ):
        validate_analysis_bundle(bundle)


def test_analysis_bundle_accepts_open_chain_endpoint_side_face_id():
    bundle = _analysis_bundle(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        (0, 1, 2),
        ((0, 1, 2),),
    )
    chain = bundle.patch_graph.nodes[0].boundary_loops[0].chains[0]
    chain.side_face_indices.append(chain.side_face_indices[-1])

    validate_analysis_bundle(bundle)
