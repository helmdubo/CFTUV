"""R2.1/RF29b/RF30/RF31: read-only Blender 4.3 field gate.

The source blend is never saved.  Every case compiles one immutable plan at
its maximum width and compares preview/confirm by full GeometryBatch digest.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
import os
from pathlib import Path
import sys

import bmesh
import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(
    os.environ.get("CFTUV_SOURCE_ROOT", str(SCRIPT_ROOT))
).resolve()
OUTPUT_JSON = Path(
    os.environ.get(
        "CFTUV_R21_REPORT",
        str(SCRIPT_ROOT / "artifacts" / "decal_r21_hotfix_acceptance.json"),
    )
)
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

from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.decal_voronoi import serialize_network_faces  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import CornerJoinMode, DecalSettings  # noqa: E402


DENSE_WIDTHS = (
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


def _digest(faces):
    payload = json.dumps(
        serialize_network_faces(faces),
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return sha256(payload).hexdigest()


def _source_bundle(obj, requested_edges):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    selected = tuple(sorted(int(edge_id) for edge_id in requested_edges))
    try:
        return (
            build_analysis_bundle(
                bm, [face.index for face in bm.faces], obj
            ),
            selected,
        )
    finally:
        bm.free()


def _compiled_counts(plan):
    surfaces = tuple(
        surface
        for partition in plan.backend_partitions
        for surface in getattr(partition.compiled_plan, "surfaces", ())
    )
    rail_plan = getattr(plan, "rail_plan", None)
    site_pair_loci = tuple(
        locus
        for locus in (() if rail_plan is None else rail_plan.freeze_loci)
        if str(getattr(locus, "competition_kind", "")).endswith(
            "TERMINAL_SITE_PAIR"
        )
    )
    return {
        "surface_count": len(surfaces),
        "domain_kinds": dict(
            sorted(Counter(surface.domain.kind for surface in surfaces).items())
        ),
        "atom_count": sum(len(surface.atoms) for surface in surfaces),
        "point_atom_count": sum(
            atom.cell_kind == "POINT"
            for surface in surfaces
            for atom in surface.atoms
        ),
        "corner_release_atom_count": sum(
            len(records)
            for surface in surfaces
            for records in surface.corner_release_atoms.values()
        ),
        "mutual_arrival_atom_count": sum(
            len(records)
            for surface in surfaces
            for records in getattr(
                surface, "mutual_arrival_atoms", {}
            ).values()
        ),
        "terminal_site_pair_loci": [
            {
                "route_ids": list(locus.route_ids),
                "arrivals": list(locus.arrival_distances),
                "source_edge": locus.source_edge_id,
                "edge_parameter": locus.edge_parameter,
                "owner_chain": repr(locus.owner_chain_ref),
            }
            for locus in site_pair_loci
        ],
    }


def _evaluate_case(object_name, selected, widths, join_mode):
    obj = bpy.data.objects[object_name]
    bundle, selected = _source_bundle(obj, selected)
    settings = DecalSettings(
        width_seam=float(widths[0]),
        offset=0.01,
        corner_join_mode=CornerJoinMode(join_mode),
    )
    plan = compile_manual_seam_decal_plan(
        bundle,
        settings,
        selected,
        alpha_budget=max(float(width) for width in widths),
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
                    "face_count": len(faces),
                    "digest": _digest(faces),
                    "component_counts": dict(
                        sorted(
                            Counter(
                                str(face.component_kind) for face in faces
                            ).items()
                        )
                    ),
                    "policy_counts": dict(result.policy_counts),
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
        "object": object_name,
        "selected": list(selected),
        "join": join_mode,
        "compiled": _compiled_counts(plan),
        "widths": records,
    }


def _evaluations_green(case):
    return all(
        record["preview_confirm_equal"]
        and all(
            evaluation.get("status") == "OK"
            for evaluation in record["evaluations"].values()
        )
        for record in case["widths"]
    )


def _no_cap_outputs(case):
    return all(
        record["evaluations"]["preview"].get(
            "component_counts", {}
        ).get("CAP", 0)
        == 0
        for record in case["widths"]
    )


def _policy_total(case, policy):
    return sum(
        record["evaluations"]["preview"].get(
            "policy_counts", {}
        ).get(policy, 0)
        for record in case["widths"]
    )


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    blend_path = Path(bpy.data.filepath)
    report = {
        "schema": "cftuv.decal_r21_hotfix.v2",
        "source_commit": os.environ.get("CFTUV_SOURCE_COMMIT", "WORKTREE"),
        "source_root": str(REPO_ROOT),
        "blender_version": bpy.app.version_string,
        "blend_file": str(blend_path),
        "blend_sha256": sha256(blend_path.read_bytes()).hexdigest(),
        "blend_saved": False,
        # Corrected field fixture: variable sagging_wall, never walls.001.
        "rf30_mutual_arrival": _evaluate_case(
            "sagging_wall",
            (3, 4, 5, 6, 7, 8, 9, 11),
            DENSE_WIDTHS,
            "BEVEL",
        ),
        # Exact live Blender selection from the user's second screenshot.
        "rf31_pchain_drag_miter": _evaluate_case(
            "sagging_wall", (7, 26), DENSE_WIDTHS, "MITER"
        ),
        "rf31_pchain_drag_bevel": _evaluate_case(
            "sagging_wall", (7, 26), DENSE_WIDTHS, "BEVEL"
        ),
        "rf29b_terminal_capacity": _evaluate_case(
            "sagging_wall",
            (6, 7, 8),
            (7.8, 12.0, 18.0, 24.0, 30.0, 36.0),
            "MITER",
        ),
    }
    rf30 = report["rf30_mutual_arrival"]
    rf31_cases = (
        report["rf31_pchain_drag_miter"],
        report["rf31_pchain_drag_bevel"],
    )
    rf29b = report["rf29b_terminal_capacity"]
    report["oracles"] = {
        "rf30_intrinsic_mutual_atoms": (
            rf30["compiled"]["mutual_arrival_atom_count"] > 0
        ),
        "rf30_runtime_release_observed": (
            _policy_total(rf30, "MUTUAL_ARRIVAL_RELEASE") > 0
        ),
        "rf31_site_pair_locus": all(
            len(case["compiled"]["terminal_site_pair_loci"]) == 1
            and case["compiled"]["terminal_site_pair_loci"][0][
                "source_edge"
            ]
            == 7
            and case["compiled"]["terminal_site_pair_loci"][0][
                "edge_parameter"
            ]
            == 0.5
            for case in rf31_cases
        ),
        "rf31_no_cap_outputs": all(
            _no_cap_outputs(case) for case in rf31_cases
        ),
        "rf31_cap_pairs_aligned": all(
            _policy_total(case, "CAP_PAIR_ALIGNED") > 0
            for case in rf31_cases
        ),
    }
    all_cases = (rf30, *rf31_cases, rf29b)
    report["green"] = all(
        _evaluations_green(case) for case in all_cases
    ) and all(report["oracles"].values())
    OUTPUT_JSON.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("CFTUV_R21_HOTFIX=" + json.dumps(report, ensure_ascii=False))
    require_green = os.environ.get("CFTUV_REQUIRE_GREEN", "1") != "0"
    if require_green and not report["green"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
