"""R2.1 drag scan for pChain collision, fan and CAP transitions.

Read-only field harness for ``sagging_wall``.  A join plan is compiled once
at the maximum width and evaluated through a dense width sequence so any
runtime-only topology or semantic-kind transition is visible.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import replace
from hashlib import sha256
import json
import os
from pathlib import Path
import sys

import bmesh
import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
OUTPUT_JSON = Path(
    os.environ.get(
        "CFTUV_R21_DRAG_REPORT",
        SCRIPT_ROOT / "artifacts" / "decal_r21_drag_scan.json",
    )
)
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]

from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.decal_voronoi import serialize_network_faces  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import CornerJoinMode, DecalSettings  # noqa: E402


WIDTHS = (
    0.2,
    0.4,
    0.6,
    0.8,
    1.0,
    1.2,
    1.6,
    2.0,
    2.4,
    2.8,
    3.2,
    4.0,
    5.0,
    6.0,
    7.0,
    7.8,
    9.0,
    12.0,
    18.0,
    24.0,
    30.0,
    36.0,
)


def _scan_widths():
    value = os.environ.get("CFTUV_R21_WIDTHS", "").strip()
    if not value:
        return WIDTHS
    return tuple(float(item) for item in value.split(","))


def _source_bundle(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    try:
        return build_analysis_bundle(
            bm, [face.index for face in bm.faces], obj
        )
    finally:
        bm.free()


def _digest(faces):
    payload = json.dumps(
        serialize_network_faces(faces),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return sha256(payload).hexdigest()


def _fan_witnesses(faces):
    uses = defaultdict(list)
    for face_index, face in enumerate(faces):
        owner = repr(face.provenance.semantic_owner_id)
        for loop_index, key in enumerate(face.vert_keys):
            uses[(owner, repr(key))].append(
                (face_index, str(face.component_kind), loop_index)
            )
    records = []
    for (owner, key), vertex_uses in sorted(uses.items()):
        unique_faces = {record[0] for record in vertex_uses}
        if len(unique_faces) < 3:
            continue
        records.append(
            {
                "owner": owner,
                "key": key,
                "face_count": len(unique_faces),
                "kinds": sorted({record[1] for record in vertex_uses}),
            }
        )
    return records


def _cap_face_details(faces):
    edge_uses = defaultdict(list)
    vertex_uses = defaultdict(list)
    for face_index, face in enumerate(faces):
        count = len(face.vert_keys)
        for loop_index, key in enumerate(face.vert_keys):
            following = face.vert_keys[(loop_index + 1) % count]
            edge_uses[frozenset((key, following))].append(
                (face_index, loop_index)
            )
            vertex_uses[key].append((face_index, loop_index))
    records = []
    for cap_index, face in enumerate(faces):
        if str(face.component_kind) != "CAP":
            continue
        shared_edges = []
        vertex_neighbors = set()
        count = len(face.vert_keys)
        for loop_index, key in enumerate(face.vert_keys):
            following = face.vert_keys[(loop_index + 1) % count]
            for other_index, other_loop in edge_uses[
                frozenset((key, following))
            ]:
                if other_index == cap_index:
                    continue
                other = faces[other_index]
                shared_edges.append(
                    {
                        "cap_loop": loop_index,
                        "other_face": other_index,
                        "other_loop": other_loop,
                        "other_kind": str(other.component_kind),
                        "other_side": str(other.component_side),
                        "other_owner": repr(
                            other.provenance.semantic_owner_id
                        ),
                        "keys": [repr(key), repr(following)],
                        "cap_facts": [
                            [float(face.u_fracs[loop_index]), float(face.v_lengths[loop_index])],
                            [
                                float(face.u_fracs[(loop_index + 1) % count]),
                                float(face.v_lengths[(loop_index + 1) % count]),
                            ],
                        ],
                        "other_facts": [
                            [
                                float(other.u_fracs[other_loop]),
                                float(other.v_lengths[other_loop]),
                            ],
                            [
                                float(other.u_fracs[(other_loop + 1) % len(other.vert_keys)]),
                                float(other.v_lengths[(other_loop + 1) % len(other.vert_keys)]),
                            ],
                        ],
                    }
                )
            vertex_neighbors.update(
                other_index
                for other_index, _other_loop in vertex_uses[key]
                if other_index != cap_index
            )
        records.append(
            {
                "face": cap_index,
                "surface": int(face.surface_id),
                "side": str(face.component_side),
                "source_edges": list(face.provenance.source_edge_ids),
                "owner": repr(face.provenance.semantic_owner_id),
                "keys": [repr(key) for key in face.vert_keys],
                "u_fracs": [float(value) for value in face.u_fracs],
                "v_lengths": [float(value) for value in face.v_lengths],
                "shared_edges": shared_edges,
                "vertex_neighbors": [
                    {
                        "face": other_index,
                        "kind": str(faces[other_index].component_kind),
                        "side": str(faces[other_index].component_side),
                        "owner": repr(
                            faces[other_index].provenance.semantic_owner_id
                        ),
                    }
                    for other_index in sorted(vertex_neighbors)
                ],
            }
        )
    return records


def _target_face_details(faces):
    records = []
    for face_index, face in enumerate(faces):
        owner = repr(face.provenance.semantic_owner_id)
        if (
            not ({5, 7, 11, 26} & set(face.provenance.source_edge_ids))
            and "('corner', 6)" not in owner
            and not any(repr(key) == "('pv-sv', 5)" for key in face.vert_keys)
        ):
            continue
        records.append(
            {
                "face": face_index,
                "kind": str(face.component_kind),
                "side": str(face.component_side),
                "owner": owner,
                "source_edges": list(face.provenance.source_edge_ids),
                "keys": [repr(key) for key in face.vert_keys],
                "u_fracs": [float(value) for value in face.u_fracs],
                "v_lengths": [float(value) for value in face.v_lengths],
                "planarity": _face_planarity(face.positions),
            }
        )
    return records


def _face_planarity(positions):
    if len(positions) < 4:
        return {"max_residual": 0.0, "extent": 0.0}
    from mathutils import Vector

    points = tuple(Vector(point) for point in positions)
    origin = points[0]
    candidate = max(
        (
            (points[first] - origin).cross(points[second] - origin)
            for first in range(1, len(points) - 1)
            for second in range(first + 1, len(points))
        ),
        key=lambda value: value.length_squared,
        default=Vector(),
    )
    if candidate.length_squared <= 1e-20:
        return {"max_residual": 0.0, "extent": 0.0}
    normal = candidate.normalized()
    return {
        "max_residual": max(
            abs((point - origin).dot(normal)) for point in points
        ),
        "extent": max((point - origin).length for point in points),
    }


def _compiled_counts(plan):
    surfaces = tuple(
        surface
        for partition in plan.backend_partitions
        for surface in getattr(partition.compiled_plan, "surfaces", ())
    )
    rail_plan = getattr(plan, "rail_plan", None)
    return {
        "surface_count": len(surfaces),
        "domain_kinds": dict(
            sorted(Counter(surface.domain.kind for surface in surfaces).items())
        ),
        "atom_count": sum(len(surface.atoms) for surface in surfaces),
        "mutual_arrival_atom_count": sum(
            len(records)
            for surface in surfaces
            for records in getattr(
                surface, "mutual_arrival_atoms", {}
            ).values()
        ),
        "site11_atoms": [
            {
                "patch": int(surface.patch_id),
                "site_index": atom.site_index,
                "fragment_triangle_ids": list(atom.fragment_triangle_ids),
                "fragment_groups": [
                    surface.domain.triangle_merge_groups[triangle_id]
                    for triangle_id in atom.fragment_triangle_ids
                ],
                "triangle_source_faces": [
                    surface.domain.intrinsic_triangles[
                        triangle_id
                    ].source_face_id
                    for triangle_id in atom.fragment_triangle_ids
                ],
            }
            for surface in surfaces
            for atom in surface.atoms
            if surface.sites[atom.site_index].edge_index == 11
        ],
        "routes": []
        if rail_plan is None
        else [
            {
                "route": int(route.route_id),
                "termination": str(route.termination),
                "spine_vertex": int(route.key.side.spine_vertex_id),
                "start_edge": int(route.key.side.start_edge_id),
                "edges": [int(segment.edge_id) for segment in route.segments],
                "reach": float(route.stations[-1].distance),
            }
            for route in rail_plan.routes
        ],
        "freeze_loci": []
        if rail_plan is None
        else [
            {
                "route_ids": list(locus.route_ids),
                "arrivals": list(locus.arrival_distances),
                "source_edge": locus.source_edge_id,
                "source_vertex": locus.source_vertex_id,
                "edge_parameter": locus.edge_parameter,
                "competition_kind": str(locus.competition_kind),
                "owner_chain": repr(locus.owner_chain_ref),
            }
            for locus in rail_plan.freeze_loci
        ],
        "terminals": [
            {
                "patch": int(terminal.patch_id),
                "vertex": int(terminal.spine_vertex_id),
                "edge": int(terminal.spine_edge_id),
                "route": terminal.route_id,
                "choice": str(terminal.choice),
            }
            for terminal in plan.terminal_routing
        ],
    }


def _scan_join(obj, bundle, selected, join_mode):
    widths = _scan_widths()
    settings = DecalSettings(
        width_seam=widths[0],
        offset=0.01,
        corner_join_mode=CornerJoinMode(join_mode),
    )
    plan = compile_manual_seam_decal_plan(
        bundle, settings, selected, alpha_budget=max(widths)
    )
    records = []
    for width in widths:
        runtime = replace(settings, width_seam=float(width))
        evaluations = {}
        for preview in (True, False):
            key = "preview" if preview else "confirm"
            try:
                result = evaluate_manual_seam_faces(
                    obj, runtime, plan, preview=preview
                )
                faces = tuple(result.faces)
                evaluations[key] = {
                    "status": "OK",
                    "digest": _digest(faces),
                    "face_count": len(faces),
                    "component_counts": dict(
                        sorted(
                            Counter(
                                str(face.component_kind) for face in faces
                            ).items()
                        )
                    ),
                    "policy_counts": dict(result.policy_counts),
                    "fan_witnesses": _fan_witnesses(faces),
                    "cap_faces": _cap_face_details(faces),
                    "target_faces": _target_face_details(faces),
                }
            except Exception as exc:
                evaluations[key] = {
                    "status": "ERROR",
                    "error": repr(exc),
                }
        records.append(
            {
                "width": float(width),
                "evaluations": evaluations,
                "preview_confirm_equal": (
                    evaluations["preview"].get("digest")
                    == evaluations["confirm"].get("digest")
                ),
            }
        )
    return {
        "join": join_mode,
        "compiled": _compiled_counts(plan),
        "widths": records,
    }


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    obj = bpy.data.objects["sagging_wall"]
    selected_spec = os.environ.get("CFTUV_R21_EDGES", "6,7,8")
    selected = (
        tuple(
            sorted(edge.index for edge in obj.data.edges if edge.use_seam)
        )
        if selected_spec == "ALL_SEAMS"
        else tuple(
            sorted(
                int(value)
                for value in selected_spec.split(",")
                if value.strip()
            )
        )
    )
    bundle = _source_bundle(obj)
    blend_path = Path(bpy.data.filepath)
    report = {
        "schema": "cftuv.decal_r21_drag_scan.v1",
        "source_root": str(REPO_ROOT),
        "source_commit": os.environ.get("CFTUV_SOURCE_COMMIT", "WORKTREE"),
        "blend_file": str(blend_path),
        "blend_sha256": sha256(blend_path.read_bytes()).hexdigest(),
        "blend_saved": False,
        "object": obj.name,
        "selected": list(selected),
        "joins": {
            join: _scan_join(obj, bundle, selected, join)
            for join in ("MITER", "BEVEL")
        },
    }
    OUTPUT_JSON.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("CFTUV_R21_DRAG=" + json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
