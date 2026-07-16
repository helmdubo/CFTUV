"""Blender smoke для A10 live Acute Split Angle.

Run:
    blender --background --factory-startup --python \
        artifacts/verify_decal_live_acute.py
"""

from __future__ import annotations

from math import cos, pi, sin
from pathlib import Path
import sys

import bpy
from mathutils import Vector


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

from cftuv import decals as decals_module  # noqa: E402
from cftuv.decal_voronoi import (  # noqa: E402
    PatchVoronoiDiagnostics,
    compile_patch_voronoi_attempt,
)
from cftuv.decals import (  # noqa: E402
    DecalPreviewState,
    ManualSeamDecalPlan,
    PreviewStatus,
    _ManualSeamBackendPartition,
    remove_decal_preview_object,
)
from cftuv.model import (  # noqa: E402
    BoundaryChain,
    BoundaryLoop,
    DecalSettings,
    PatchGraph,
    PatchNode,
)


def _acute_graph():
    angle = pi / 12.0
    points = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0 * cos(angle), -2.0 * sin(angle), 0.0)),
        Vector((2.0 * cos(angle), 2.0 * sin(angle), 0.0)),
    ]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / 3.0,
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [(0, 1, 2)]
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=[0, 1],
                    vert_cos=[points[0], points[1]],
                    edge_indices=[40],
                ),
                BoundaryChain(
                    vert_indices=[2, 0],
                    vert_cos=[points[2], points[0]],
                    edge_indices=[41],
                ),
            ]
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, points


def _source(points):
    mesh = bpy.data.meshes.new("A10_Source_Geo")
    mesh.from_pydata([tuple(point) for point in points], (), ((0, 1, 2),))
    obj = bpy.data.objects.new("A10_Source", mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _remove_object(obj):
    if obj is None:
        return
    mesh = obj.data if obj.type == "MESH" else None
    bpy.data.objects.remove(obj, do_unlink=True)
    if mesh is not None and mesh.users == 0:
        bpy.data.meshes.remove(mesh)


def _mesh_signature(obj):
    return (
        len(obj.data.vertices),
        len(obj.data.edges),
        len(obj.data.polygons),
    )


def main():
    graph, points = _acute_graph()
    source = _source(points)
    preview_state = DecalPreviewState()
    preview_obj = None
    diagnostics = PatchVoronoiDiagnostics()
    compile_attempt = compile_patch_voronoi_attempt(
        graph,
        (40, 41),
        offset=0.01,
        allow_partial=False,
        diagnostics=diagnostics,
    )
    assert compile_attempt.plan is not None, compile_attempt.failures
    compiled = compile_attempt.plan
    construct_calls = diagnostics.construct_calls
    partition = _ManualSeamBackendPartition(
        backend="PATCH_VORONOI",
        edge_indices=(40, 41),
        topology_component_count=1,
        corner_runs=(object(),),
        boundary_runs=(),
        compiled_plan=compiled,
    )
    plan = ManualSeamDecalPlan(
        corner_runs=(object(),),
        boundary_runs=(),
        patch_voronoi_plan=compiled,
        backend_partitions=(partition,),
        selected_edge_indices=(40, 41),
    )
    assert plan.supports_live_corner_controls

    def generate(settings):
        return decals_module.generate_decal_result(
            graph,
            source,
            settings,
            "SEAMS",
            scene=bpy.context.scene,
            chain_refs=(),
            selected_edge_indices=(40, 41),
            preview=True,
            decal_plan=plan,
            preview_state=preview_state,
        )

    try:
        split = generate(
            DecalSettings(
                width_seam=1.0,
                offset=0.01,
                corner_acute_split_angle=pi / 3.0,
            )
        )
        assert split.status == PreviewStatus.UPDATED
        preview_obj = bpy.data.objects[split.object_name]
        object_pointer = preview_obj.as_pointer()
        mesh_pointer = preview_obj.data.as_pointer()
        split_signature = _mesh_signature(preview_obj)
        assert dict(split.policy_counts).get("ACUTE_SPLIT") == 1

        miter = generate(
            DecalSettings(
                width_seam=1.0,
                offset=0.01,
                corner_acute_split_angle=pi / 12.0,
            )
        )
        assert miter.status == PreviewStatus.UPDATED
        assert preview_obj.as_pointer() == object_pointer
        assert preview_obj.data.as_pointer() == mesh_pointer
        miter_signature = _mesh_signature(preview_obj)
        assert miter_signature != split_signature
        assert dict(miter.policy_counts).get("ACUTE_SPLIT", 0) == 0
        assert dict(miter.policy_counts).get("MITER", 0) >= 1

        split_again = generate(
            DecalSettings(
                width_seam=1.0,
                offset=0.01,
                corner_acute_split_angle=pi / 3.0,
            )
        )
        assert split_again.status == PreviewStatus.UPDATED
        assert _mesh_signature(preview_obj) == split_signature
        assert preview_obj.as_pointer() == object_pointer
        assert preview_obj.data.as_pointer() == mesh_pointer
        assert diagnostics.construct_calls == construct_calls

        original_evaluate = decals_module.evaluate_patch_voronoi_plan
        decals_module.evaluate_patch_voronoi_plan = (
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                RuntimeError("injected invalid threshold frame")
            )
        )
        try:
            invalid = generate(
                DecalSettings(
                    width_seam=1.0,
                    offset=0.01,
                    corner_acute_split_angle=pi / 2.0,
                )
            )
        finally:
            decals_module.evaluate_patch_voronoi_plan = original_evaluate
        assert invalid.status == PreviewStatus.RETAINED_LAST_VALID
        assert preview_obj.as_pointer() == object_pointer
        assert preview_obj.data.as_pointer() == mesh_pointer
        assert _mesh_signature(preview_obj) == split_signature
        print(
            "[CFTUV][A10] PASS",
            f"split={split_signature}",
            f"miter={miter_signature}",
            f"eval_ms={split_again.evaluation_ms:.3f}",
            f"rebuilds={preview_state.topology_rebuilds}",
            "construct_drag=0",
        )
    finally:
        remove_decal_preview_object("SEAMS", source, preview_state)
        _remove_object(source)


main()
