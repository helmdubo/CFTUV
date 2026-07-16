"""Deterministic Patch Voronoi runtime benchmark for Blender fixtures.

Example:
    blender -b scene.blend --python artifacts/verify_decal_runtime.py -- \
        --objects walls.003 walls.001 --output artifacts/decal_runtime.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from statistics import median
import sys
from time import perf_counter

import bmesh
import bpy


_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
_loaded_cftuv = sys.modules.get("cftuv")
if _loaded_cftuv is not None:
    loaded_path = Path(getattr(_loaded_cftuv, "__file__", "")).resolve()
    if _REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]


def _arguments():
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--objects", nargs="+", default=["walls.003", "walls.001"])
    parser.add_argument(
        "--widths", nargs="+", type=float, default=[2.0, 3.0, 3.7076, 4.5]
    )
    parser.add_argument("--offset", type=float, default=0.02)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument(
        "--output", default="artifacts/decal_runtime_benchmark.json"
    )
    return parser.parse_args(raw)


def _percentile(values, fraction):
    values = sorted(float(value) for value in values)
    if not values:
        return 0.0
    index = min(len(values) - 1, int(round((len(values) - 1) * fraction)))
    return values[index]


def _timing_summary(values):
    return {
        "median_ms": round(median(values), 6) if values else 0.0,
        "p95_ms": round(_percentile(values, 0.95), 6),
        "samples_ms": tuple(round(float(value), 6) for value in values),
    }


def _geometry_digest(snapshot):
    payload = json.dumps(
        snapshot, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _source_graph(obj):
    from cftuv.analysis import build_patch_graph

    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    selected_edges = tuple(sorted(edge.index for edge in bm.edges if edge.seam))
    graph = build_patch_graph(bm, [face.index for face in bm.faces], obj)
    bm.free()
    return graph, selected_edges


def _materialization_metrics(faces, settings):
    from cftuv.constants import DECAL_UV_RECT_SEAM
    from cftuv.decals import _materialize_network_faces

    bm = bmesh.new()
    started = perf_counter()
    result = _materialize_network_faces(
        bm,
        faces,
        settings,
        DECAL_UV_RECT_SEAM,
        backend="PATCH_VORONOI",
    )
    elapsed_ms = (perf_counter() - started) * 1000.0
    counts = {
        "vertices": len(bm.verts),
        "edges": len(bm.edges),
        "faces": len(bm.faces),
        "created_faces": result.created_face_count,
        "dropped_faces": 0,
    }
    bm.free()
    return elapsed_ms, counts


def _benchmark_object(obj, args):
    from cftuv.decal_voronoi import (
        CornerRuntimeSettings,
        PatchVoronoiDiagnostics,
        compile_patch_voronoi_attempt,
        evaluate_patch_voronoi_plan,
        serialize_network_faces,
    )
    from cftuv.model import DecalSettings

    prepare_started = perf_counter()
    graph, selected_edges = _source_graph(obj)
    prepare_ms = (perf_counter() - prepare_started) * 1000.0
    diagnostics = PatchVoronoiDiagnostics()
    compile_started = perf_counter()
    attempt = compile_patch_voronoi_attempt(
        graph,
        selected_edges,
        args.offset,
        allow_partial=False,
        diagnostics=diagnostics,
    )
    compile_ms = (perf_counter() - compile_started) * 1000.0
    result = {
        "object": obj.name,
        "selected_edge_count": len(selected_edges),
        "prepare_patch_graph_ms": round(prepare_ms, 6),
        "compile_ms": round(compile_ms, 6),
        "backend": "PATCH_VORONOI" if attempt.plan is not None else "REJECTED",
        "compile_failures": tuple(
            {
                "patch_id": failure.patch_id,
                "reason": failure.reason,
                "edge_indices": failure.edge_indices,
                "details": failure.details,
            }
            for failure in attempt.failures
        ),
        "diagnostics_after_compile": diagnostics.as_dict(),
        "widths": [],
    }
    if attempt.plan is None:
        return result

    settings = DecalSettings(offset=args.offset)
    corner_settings = CornerRuntimeSettings()
    construct_calls_after_compile = diagnostics.construct_calls
    for width in args.widths:
        preview_timings = []
        materialization_timings = []
        preview_snapshots = []
        counts = None
        policy_counts = {}
        for _repeat in range(max(2, int(args.repeats))):
            started = perf_counter()
            faces = evaluate_patch_voronoi_plan(
                attempt.plan,
                width,
                preview=True,
                corner_settings=corner_settings,
                diagnostics=diagnostics,
            )
            preview_timings.append((perf_counter() - started) * 1000.0)
            snapshot = serialize_network_faces(faces)
            preview_snapshots.append(snapshot)
            materialize_ms, counts = _materialization_metrics(faces, settings)
            materialization_timings.append(materialize_ms)
            policy_counts = {}
            for face in faces:
                policy_counts[face.component_kind] = (
                    policy_counts.get(face.component_kind, 0) + 1
                )

        confirm_started = perf_counter()
        confirmed_faces = evaluate_patch_voronoi_plan(
            attempt.plan,
            width,
            preview=False,
            corner_settings=corner_settings,
            diagnostics=diagnostics,
        )
        confirm_ms = (perf_counter() - confirm_started) * 1000.0
        confirmed_snapshot = serialize_network_faces(confirmed_faces)
        reference_snapshot = preview_snapshots[0]
        result["widths"].append(
            {
                "width": float(width),
                "evaluate_preview": _timing_summary(preview_timings),
                "evaluate_confirm_ms": round(confirm_ms, 6),
                "materialization": _timing_summary(materialization_timings),
                "mesh_counts": counts,
                "policy_counts": dict(sorted(policy_counts.items())),
                "topology_signature": reference_snapshot["topology_signature"],
                "geometry_digest": _geometry_digest(reference_snapshot),
                "repeated_preview_equal": all(
                    snapshot == reference_snapshot
                    for snapshot in preview_snapshots[1:]
                ),
                "preview_confirm_equal": confirmed_snapshot == reference_snapshot,
            }
        )
    result["construct_calls_during_drag"] = (
        diagnostics.construct_calls - construct_calls_after_compile
    )
    result["diagnostics_final"] = diagnostics.as_dict()
    return result


def main():
    args = _arguments()
    report = {
        "schema": "cftuv.decal_runtime_benchmark.v1",
        "blender_version": bpy.app.version_string,
        "python_version": sys.version.split()[0],
        "pyvoronoi_version": "unknown",
        "blend_file": Path(bpy.data.filepath).name,
        "requested_objects": tuple(args.objects),
        "widths": tuple(float(value) for value in args.widths),
        "objects": [],
        "missing_objects": [],
    }
    try:
        import pyvoronoi

        report["pyvoronoi_version"] = getattr(
            pyvoronoi, "__version__", "unknown"
        )
    except ImportError:
        report["pyvoronoi_version"] = "unavailable"

    for object_name in args.objects:
        obj = bpy.data.objects.get(object_name)
        if obj is None or obj.type != "MESH":
            report["missing_objects"].append(object_name)
            continue
        report["objects"].append(_benchmark_object(obj, args))

    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = Path.cwd() / output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"[CFTUV] decal runtime benchmark: {output_path}")
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))


main()
