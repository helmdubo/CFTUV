"""R1.7: post-clip V/P provenance native BEVEL point-cell на sagging_wall."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
for path in (REPO_ROOT, SCRIPT_ROOT / "artifacts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

import verify_decal_r13_rr9a as field  # noqa: E402
from cftuv import decal_voronoi as voronoi  # noqa: E402
from cftuv.decals import compile_manual_seam_decal_plan  # noqa: E402
from cftuv.model import DecalSettings  # noqa: E402


OBJECT_NAME = "sagging_wall"
SELECTED = (6, 7, 8)
WIDTH = float(os.environ.get("CFTUV_R17_WIDTH", "3.2"))
OUTPUT_JSON = SCRIPT_ROOT / "artifacts" / "decal_r17_bevel_pointcell.json"


def _key(value):
    if isinstance(value, tuple):
        return [_key(item) for item in value]
    return value


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    obj = bpy.data.objects[OBJECT_NAME]
    graph, selected, _all_seams = field._source_graph(obj, SELECTED)
    settings = DecalSettings(
        width_seam=WIDTH,
        offset=0.01,
        corner_join_mode="BEVEL",
    )
    plan = compile_manual_seam_decal_plan(
        graph, settings, selected, alpha_budget=4.0
    )
    corner_settings = voronoi.corner_runtime_settings_from_decal_settings(
        settings
    )
    alpha = WIDTH * 0.5
    payload = {"width": WIDTH, "corners": []}
    for backend in plan.backend_partitions:
        if backend.backend != "PATCH_VORONOI":
            continue
        geometry = backend.compiled_plan
        pending = []
        for surface in geometry.surfaces:
            terminal_points = voronoi._surface_terminal_bridge_points(
                surface,
                plan.terminal_routing,
                plan.rail_plan,
                alpha,
            )
            voronoi._evaluate_surface_crops(
                surface,
                alpha,
                pending,
                corner_settings,
                terminal_bridge_points=terminal_points,
            )
        arrangement = voronoi._build_decal_arrangement(
            pending,
            tolerance=max(1e-8, voronoi.DECAL_WELD_DISTANCE * 0.5),
            alpha=alpha,
        )
        for surface in geometry.surfaces:
            for corner in surface.corners:
                policy = voronoi._classify_surface_corner_runtime(
                    surface, corner, corner_settings
                )
                if policy != voronoi._CornerPolicy.BEVEL:
                    continue
                record = {
                    "patch_id": surface.patch_id,
                    "vertex_id": corner.vert_index,
                    "relation": voronoi._corner_offset_edge_relation(
                        surface, corner
                    ),
                    "incident_edges": [
                        surface.sites[index].edge_index
                        for index in corner.ordered_sites
                    ],
                    "site_faces": [],
                }
                for site_index in corner.ordered_sites:
                    site = surface.sites[site_index]
                    for face in arrangement.faces:
                        if (
                            face.surface is not surface
                            or face.crop.kind != "SEGMENT"
                            or face.site.edge_index != site.edge_index
                        ):
                            continue
                        record["site_faces"].append(
                            {
                                "edge_id": site.edge_index,
                                "side": face.crop.side,
                                "points": [list(point) for point in face.points],
                                "keys": [_key(key) for key in face.point_keys],
                                "distances": [
                                    voronoi._segment_point_distance2(
                                        site.point_a, site.point_b, point
                                    )[0]
                                    for point in face.points
                                ],
                            }
                        )
                payload["corners"].append(record)
    OUTPUT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("CFTUV_R17_BEVEL_POINTCELL=" + json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":
    main()
