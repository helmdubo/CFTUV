"""R1.9/RF29: read-only dump of extreme terminal-route saturation.

The script opens the user's current blend in background mode and never saves
it.  Both preview and confirm evaluate the same compile-static plan so UI
last-valid retention cannot masquerade as evaluator parity.
"""

from __future__ import annotations

import json
import os
import sys
from hashlib import sha256
from dataclasses import replace
from pathlib import Path

import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
OUTPUT_JSON = Path(
    os.environ.get(
        "CFTUV_R19_REPORT",
        str(SCRIPT_ROOT / "artifacts" / "decal_r19_saturation_before.json"),
    )
)
OBJECT_NAME = os.environ.get("CFTUV_R19_OBJECT", "sagging_wall")
WIDTH = float(os.environ.get("CFTUV_R19_WIDTH", "35.98"))
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

from cftuv import decal_voronoi  # noqa: E402
from cftuv.decal_voronoi import serialize_network_faces  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402

import verify_decal_r13_rr9a as field  # noqa: E402


def _selected_seams(obj):
    selected = tuple(
        int(edge.index)
        for edge in obj.data.edges
        if edge.select and edge.use_seam
    )
    return selected or (6, 7, 8)


def _point(point):
    return [float(point[0]), float(point[1])]


def _guide_record(guide):
    return {
        "point": _point(guide.point),
        "contour_points": [_point(point) for point in guide.contour_points],
        "source_vertex_ids": list(guide.source_vertex_ids),
    }


def _snapshot_digests(faces):
    snapshot = serialize_network_faces(faces)
    payload = json.dumps(
        snapshot, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    geometry = {
        "topology_signature": snapshot["topology_signature"],
        "faces": [
            {
                "surface_id": face["surface_id"],
                "component_kind": face["component_kind"],
                "component_side": face["component_side"],
                "vert_keys": face["vert_keys"],
                "positions": face["positions"],
            }
            for face in snapshot["faces"]
        ],
    }
    geometry_payload = json.dumps(
        geometry, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return sha256(payload).hexdigest(), sha256(geometry_payload).hexdigest()


def _terminal_dump(plan, width):
    alpha = width * 0.5
    rail_plan = plan.rail_plan
    route_by_id = {
        route.route_id: route for route in rail_plan.routes
    } if rail_plan is not None else {}
    records = []
    for terminal in plan.terminal_routing:
        if terminal.spine_vertex_id != 10 or terminal.spine_edge_id != 8:
            continue
        route = route_by_id.get(terminal.route_id)
        record = {
            "terminal": {
                "patch_id": terminal.patch_id,
                "spine_vertex_id": terminal.spine_vertex_id,
                "spine_edge_id": terminal.spine_edge_id,
                "choice": terminal.choice,
                "route_id": terminal.route_id,
                "edge_ids": list(terminal.edge_ids),
            },
            "alpha": alpha,
        }
        if route is None:
            record["route_missing"] = True
            records.append(record)
            continue
        segment_edge_ids = [segment.edge_id for segment in route.segments]
        record["route"] = {
            "termination": route.termination.value,
            "segment_edge_ids": segment_edge_ids,
            "revisited_edge_ids": sorted(
                {
                    edge_id
                    for edge_id in segment_edge_ids
                    if segment_edge_ids.count(edge_id) > 1
                }
            ),
            "reach": float(route.stations[-1].distance),
            "saturated": alpha >= float(route.stations[-1].distance),
            "stations": [
                {
                    "index": station.station_index,
                    "distance": float(station.distance),
                    "kind": station.kind.value,
                    "source_vertex_id": station.source_vertex_id,
                    "source_edge_id": station.source_edge_id,
                    "edge_parameter": station.edge_parameter,
                }
                for station in route.stations
            ],
        }
        for partition in plan.backend_partitions:
            if partition.backend != "PATCH_VORONOI":
                continue
            for surface in partition.compiled_plan.surfaces:
                if surface.patch_id != terminal.patch_id:
                    continue
                corner = next(
                    (
                        candidate
                        for candidate in surface.corners
                        if candidate.vert_index == terminal.spine_vertex_id
                        and len(candidate.incident_sites) == 1
                        and surface.sites[candidate.incident_sites[0]].edge_index
                        == terminal.spine_edge_id
                    ),
                    None,
                )
                if corner is None:
                    continue
                guide = decal_voronoi._terminal_route_chart_guide(
                    surface, terminal, rail_plan, alpha, corner.point
                )
                record["guide"] = _guide_record(guide)
                site = surface.sites[corner.incident_sites[0]]
                record["site"] = {
                    "vert_a": site.vert_a,
                    "vert_b": site.vert_b,
                    "point_a": _point(site.point_a),
                    "point_b": _point(site.point_b),
                    "length": float(site.segment_length),
                    "inward_normal": _point(site.inward_normal),
                }
                kwargs = (
                    {"start_guide": guide}
                    if site.vert_a == terminal.spine_vertex_id
                    else {"end_guide": guide}
                )
                try:
                    components = decal_voronoi._terminal_segment_crop_components(
                        site, alpha, **kwargs
                    )
                    record["terminal_components"] = [
                        {
                            "side": component.side,
                            "points": [_point(point) for point in component.points],
                        }
                        for component, _vertices in components
                    ]
                except Exception as exc:
                    record["terminal_components_error"] = repr(exc)
                if (
                    len(guide.contour_points) >= 2
                    and decal_voronoi._dist2(
                        guide.contour_points[-2], guide.contour_points[-1]
                    ) <= 1e-20
                ):
                    canonical_guide = decal_voronoi._TerminalBridgeGuide(
                        point=guide.contour_points[-2],
                        contour_points=guide.contour_points[:-1],
                        source_vertex_ids=guide.source_vertex_ids,
                    )
                    try:
                        canonical_components = (
                            decal_voronoi._terminal_segment_crop_components(
                                site,
                                alpha,
                                **(
                                    {"start_guide": canonical_guide}
                                    if site.vert_a == terminal.spine_vertex_id
                                    else {"end_guide": canonical_guide}
                                ),
                            )
                        )
                        record["canonical_endpoint_status"] = "OK"
                        record["canonical_endpoint_component_count"] = len(
                            canonical_components
                        )
                    except Exception as exc:
                        record["canonical_endpoint_status"] = "ERROR"
                        record["canonical_endpoint_error"] = repr(exc)
                record["station_cut_probe"] = []
                probe_extents = sorted(
                    {
                        float(station.distance)
                        for station in route.stations[1:]
                    }
                )
                for extent in probe_extents:
                    probe = {"extent": extent}
                    try:
                        probe_guide = decal_voronoi._terminal_route_chart_guide(
                            surface,
                            terminal,
                            rail_plan,
                            extent,
                            corner.point,
                        )
                        probe["guide"] = _guide_record(probe_guide)
                        components = (
                            decal_voronoi._terminal_segment_crop_components(
                                site,
                                extent,
                                **(
                                    {"start_guide": probe_guide}
                                    if site.vert_a == terminal.spine_vertex_id
                                    else {"end_guide": probe_guide}
                                ),
                            )
                        )
                        probe["status"] = "OK"
                        probe["component_count"] = len(components)
                    except Exception as exc:
                        probe["status"] = "ERROR"
                        probe["error"] = repr(exc)
                    try:
                        wide_components = (
                            decal_voronoi._terminal_segment_crop_components(
                                site,
                                alpha,
                                **(
                                    {"start_guide": probe_guide}
                                    if site.vert_a == terminal.spine_vertex_id
                                    else {"end_guide": probe_guide}
                                ),
                            )
                        )
                        probe["wide_status"] = "OK"
                        probe["wide_component_count"] = len(wide_components)
                    except Exception as exc:
                        probe["wide_status"] = "ERROR"
                        probe["wide_error"] = repr(exc)
                    record["station_cut_probe"].append(probe)
        records.append(record)
    return records


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    obj = bpy.data.objects[OBJECT_NAME]
    selected = _selected_seams(obj)
    graph, selected, _all_seams = field._source_graph(obj, selected)
    settings = DecalSettings(width_seam=WIDTH, offset=0.01)
    plan = compile_manual_seam_decal_plan(graph, settings, selected)
    coordinates = [vertex.co.copy() for vertex in obj.data.vertices]
    minimum = coordinates[0].copy()
    maximum = coordinates[0].copy()
    for coordinate in coordinates[1:]:
        for axis in range(3):
            minimum[axis] = min(minimum[axis], coordinate[axis])
            maximum[axis] = max(maximum[axis], coordinate[axis])
    mesh_diagonal = float((maximum - minimum).length)
    payload = {
        "schema": "cftuv.decal_r19.saturation.v1",
        "blend": str(bpy.data.filepath),
        "object": obj.name,
        "selected": list(selected),
        "width": WIDTH,
        "alpha": WIDTH * 0.5,
        "mesh_diagonal": mesh_diagonal,
        "rail_alpha_budget": (
            float(plan.rail_plan.alpha_budget) if plan.rail_plan else None
        ),
        "evaluations": {},
        "terminal_v10_e8": _terminal_dump(plan, WIDTH),
        "blend_saved": False,
    }
    for preview in (True, False):
        key = "preview" if preview else "confirm"
        try:
            result = evaluate_manual_seam_faces(
                obj, replace(settings, width_seam=WIDTH), plan, preview=preview
            )
            snapshot_digest, geometry_digest = _snapshot_digests(result.faces)
            payload["evaluations"][key] = {
                "status": "OK",
                "face_count": len(result.faces),
                "policy_counts": [list(item) for item in result.policy_counts],
                "snapshot_digest": snapshot_digest,
                "geometry_digest": geometry_digest,
            }
        except Exception as exc:
            payload["evaluations"][key] = {
                "status": "ERROR",
                "error": repr(exc),
            }
    payload["preview_confirm_equal"] = (
        payload["evaluations"]["preview"].get("snapshot_digest")
        == payload["evaluations"]["confirm"].get("snapshot_digest")
    )
    rf29_width = max(WIDTH, mesh_diagonal * 10.0)
    rf29_settings = replace(settings, width_seam=rf29_width)
    rf29_plan = compile_manual_seam_decal_plan(
        graph, rf29_settings, selected
    )
    rf29_evaluations = {}
    for preview in (True, False):
        key = "preview" if preview else "confirm"
        try:
            result = evaluate_manual_seam_faces(
                obj, rf29_settings, rf29_plan, preview=preview
            )
            snapshot_digest, geometry_digest = _snapshot_digests(result.faces)
            rf29_evaluations[key] = {
                "status": "OK",
                "face_count": len(result.faces),
                "policy_counts": [list(item) for item in result.policy_counts],
                "snapshot_digest": snapshot_digest,
                "geometry_digest": geometry_digest,
            }
        except Exception as exc:
            rf29_evaluations[key] = {
                "status": "ERROR",
                "error": repr(exc),
            }
    payload["rf29_extreme"] = {
        "width": rf29_width,
        "mesh_diagonal_multiple": rf29_width / mesh_diagonal,
        "evaluations": rf29_evaluations,
        "preview_confirm_equal": (
            rf29_evaluations["preview"].get("snapshot_digest")
            == rf29_evaluations["confirm"].get("snapshot_digest")
        ),
        "geometry_saturated_equal_to_width_35_98": (
            payload["evaluations"]["preview"].get("geometry_digest")
            == rf29_evaluations["preview"].get("geometry_digest")
        ),
    }
    OUTPUT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("CFTUV_R19_SATURATION=" + json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":
    main()
