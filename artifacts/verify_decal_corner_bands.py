"""Blender smoke для approved A11 corner-band oracle."""

from __future__ import annotations

from dataclasses import replace
from math import cos, pi, sin
from pathlib import Path
import sys

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

from cftuv import decal_voronoi  # noqa: E402
from cftuv.model import (  # noqa: E402
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
)


def _corner_graph(theta):
    half_angle = theta * 0.5
    points = [
        Vector((0.0, 0.0, 0.0)),
        Vector((2.0 * cos(half_angle), -2.0 * sin(half_angle), 0.0)),
        Vector((2.0 * cos(half_angle), 2.0 * sin(half_angle), 0.0)),
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
    return graph


def main():
    diagnostics = decal_voronoi.PatchVoronoiDiagnostics()
    plan = decal_voronoi.compile_patch_voronoi_plan(
        _corner_graph(75.0 * pi / 180.0),
        (40, 41),
        offset=0.01,
        diagnostics=diagnostics,
    )
    assert plan is not None
    construct_calls = diagnostics.construct_calls
    corner = next(
        item for item in plan.surfaces[0].corners if item.vert_index == 0
    )

    expected = {
        170: "MITER",
        130: "MITER",
        100: "KITE",
        75: "FAN",
        45: "ACUTE_SPLIT",
        20: "HAIRPIN",
    }
    actual = {}
    for degrees, policy_name in expected.items():
        probe = replace(
            corner,
            interior_angle=degrees * pi / 180.0,
            extrusion_angle=degrees * pi / 180.0,
        )
        actual[degrees] = decal_voronoi.classify_corner_runtime(probe).value
        assert actual[degrees] == policy_name

    split_settings = decal_voronoi.CornerRuntimeSettings(
        split_angle=80.0 * pi / 180.0
    )
    fan_settings = decal_voronoi.CornerRuntimeSettings(
        split_angle=50.0 * pi / 180.0
    )
    split_faces = decal_voronoi.evaluate_patch_voronoi_plan(
        plan,
        width=1.0,
        preview=True,
        corner_settings=split_settings,
        diagnostics=diagnostics,
    )
    split_counts = dict(diagnostics.runtime_policy_counts)
    fan_faces = decal_voronoi.evaluate_patch_voronoi_plan(
        plan,
        width=1.0,
        preview=True,
        corner_settings=fan_settings,
        diagnostics=diagnostics,
    )
    fan_counts = dict(diagnostics.runtime_policy_counts)
    confirmed = decal_voronoi.evaluate_patch_voronoi_plan(
        plan,
        width=1.0,
        preview=False,
        corner_settings=fan_settings,
    )

    assert split_counts.get("ACUTE_SPLIT") == 1
    assert fan_counts.get("FAN") == 1
    assert sum(face.component_kind == "ACUTE_SPLIT" for face in split_faces) == 2
    assert sum(face.component_kind == "FAN" for face in fan_faces) == 2
    assert decal_voronoi.serialize_network_faces(fan_faces) == (
        decal_voronoi.serialize_network_faces(confirmed)
    )
    assert decal_voronoi.serialize_network_faces(split_faces) != (
        decal_voronoi.serialize_network_faces(fan_faces)
    )
    assert diagnostics.construct_calls == construct_calls
    print(
        "[CFTUV][A11] PASS",
        f"bands={actual}",
        f"split_faces={len(split_faces)}",
        f"fan_faces={len(fan_faces)}",
        "construct_drag=0",
    )


main()
