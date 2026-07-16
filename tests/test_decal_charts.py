from types import SimpleNamespace

import pytest
from mathutils import Vector

from cftuv import decal_charts
from cftuv.analysis_topology import _serialize_patch_geometry
from cftuv.decal_charts import (
    ChartAdjacency,
    ChartBuildFailure,
    ChartBuildMetrics,
    ChartCut,
    IntrinsicStripChart,
    chart_site_seeds_from_chain,
    chart_triangles_from_patch,
)
from cftuv.model import BoundaryChain, PatchNode


def _serialized_patch():
    return PatchNode(
        patch_id=7,
        face_indices=[70, 71],
        mesh_verts=[
            Vector((0.0, 0.0, 0.0)),
            Vector((1.0, 0.0, 0.0)),
            Vector((1.0, 1.0, 0.0)),
            Vector((0.0, 1.0, 0.0)),
        ],
        mesh_vert_indices=[10, 20, 30, 40],
        mesh_tris=[(0, 1, 2), (0, 2, 3)],
        mesh_tri_face_indices=[70, 71],
        mesh_tri_face_normals=[
            Vector((0.0, 0.0, 1.0)),
            Vector((0.0, 0.0, 1.0)),
        ],
    )


def test_c0_serialized_patch_builds_immutable_triangle_ir():
    triangles = chart_triangles_from_patch(_serialized_patch())

    assert tuple(triangle.triangle_id for triangle in triangles) == (0, 1)
    assert triangles[0].source_face_id == 70
    assert triangles[0].source_vertex_ids == (10, 20, 30)
    assert triangles[0].positions == (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
    )
    assert triangles[0].source_edge_ids == (
        (20, 30),
        (10, 30),
        (10, 20),
    )
    assert not triangles[0].is_placed
    assert "bpy" not in decal_charts.__dict__
    assert "bmesh" not in decal_charts.__dict__


def test_c0_analysis_serializes_global_vertex_provenance():
    vertices = [
        SimpleNamespace(index=101, co=Vector((0.0, 0.0, 0.0))),
        SimpleNamespace(index=205, co=Vector((1.0, 0.0, 0.0))),
        SimpleNamespace(index=309, co=Vector((1.0, 1.0, 0.0))),
        SimpleNamespace(index=412, co=Vector((0.0, 1.0, 0.0))),
    ]
    face = SimpleNamespace(
        index=17,
        verts=vertices,
        normal=Vector((0.0, 0.0, 1.0)),
    )
    serialized = _serialize_patch_geometry(
        SimpleNamespace(faces=[face]), [0]
    )

    mesh_verts, source_ids, triangles, face_ids, normals = serialized
    assert len(mesh_verts) == 4
    assert source_ids == [101, 205, 309, 412]
    assert triangles == [(0, 1, 2), (0, 2, 3)]
    assert face_ids == [17, 17]
    assert len(normals) == 2


def test_c0_boundary_chain_seeds_keep_edge_face_and_chain_provenance():
    chain = BoundaryChain(
        vert_indices=[10, 20, 30],
        edge_indices=[501, 502],
        side_face_indices=[70, 71, 71],
    )

    seeds = chart_site_seeds_from_chain(chain, (7, 0, 3))

    assert tuple(seed.edge_index for seed in seeds) == (501, 502)
    assert tuple(seed.source_vertex_ids for seed in seeds) == (
        (10, 20),
        (20, 30),
    )
    assert tuple(seed.source_face_id for seed in seeds) == (70, 71)
    assert all(seed.chain_ref == (7, 0, 3) for seed in seeds)


def test_c0_chart_ir_validates_relations_cuts_and_metrics():
    triangles = chart_triangles_from_patch(_serialized_patch())
    adjacency = ChartAdjacency(
        triangle_a=0,
        triangle_b=1,
        source_edge=(10, 30),
        local_edge_a=1,
        local_edge_b=2,
    )
    cut = ChartCut(
        source_edge=(10, 30),
        triangle_ids=(0, 1),
        reason="PERIODIC_TEST_CUT",
        transition_key=(7, 10, 30),
    )
    metrics = ChartBuildMetrics(
        support_triangle_count=2,
        adjacency_count=1,
        boundary_edge_count=4,
        cut_count=1,
    )

    chart = IntrinsicStripChart(
        chart_id=2,
        patch_id=7,
        triangles=triangles,
        adjacency=(adjacency,),
        cuts=(cut,),
        alpha_budget=0.5,
        budget_source="STRIP_BUDGET",
        metrics=metrics,
    )

    assert chart.support_triangle_ids == (0, 1)
    assert chart.placed_triangle_ids == ()
    assert chart.metrics.support_triangle_count == 2

    with pytest.raises(ValueError, match="unknown triangle"):
        IntrinsicStripChart(
            chart_id=2,
            patch_id=7,
            triangles=triangles,
            support_triangle_ids=(0, 9),
        )
    with pytest.raises(ValueError, match="disagree with provenance"):
        IntrinsicStripChart(
            chart_id=2,
            patch_id=7,
            triangles=triangles,
            adjacency=(
                ChartAdjacency(
                    triangle_a=0,
                    triangle_b=1,
                    source_edge=(10, 30),
                    local_edge_a=0,
                    local_edge_b=2,
                ),
            ),
        )


def test_c0_missing_patch_provenance_is_explicit_failure():
    node = _serialized_patch()
    node.mesh_vert_indices = []

    with pytest.raises(ChartBuildFailure) as captured:
        chart_triangles_from_patch(node)

    assert captured.value.code == "MISSING_SOURCE_VERTEX_PROVENANCE"
    assert captured.value.patch_id == 7
    assert "verts=4 ids=0" in captured.value.details
