"""Blender/headless отчёт E0 по intrinsic chart measurements.

Example:
    blender -b scene.blend --python artifacts/verify_decal_charts.py -- \
        --objects rounded_wall --output artifacts/decal_charts.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import bmesh
import bpy


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _arguments():
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--objects", nargs="+", required=True)
    parser.add_argument("--offset", type=float, default=0.02)
    parser.add_argument("--alpha-budget", type=float, default=1.0)
    parser.add_argument("--output", default="artifacts/decal_charts.json")
    return parser.parse_args(raw)


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


def _measure_object(obj, args):
    from cftuv.decal_voronoi import (
        PatchVoronoiDiagnostics,
        compile_patch_voronoi_attempt,
    )

    graph, selected_edges = _source_graph(obj)
    diagnostics = PatchVoronoiDiagnostics()
    attempt = compile_patch_voronoi_attempt(
        graph,
        selected_edges,
        args.offset,
        alpha_budget=args.alpha_budget,
        diagnostics=diagnostics,
        allow_partial=True,
    )
    return {
        "object": obj.name,
        "selected_edge_count": len(selected_edges),
        "admitted": attempt.plan is not None,
        "reasons": tuple(failure.reason for failure in attempt.failures),
        "metrics": diagnostics.as_dict(),
    }


def main():
    args = _arguments()
    report = {
        "alpha_budget": args.alpha_budget,
        "objects": [
            _measure_object(bpy.data.objects[name], args)
            for name in args.objects
        ],
    }
    output = Path(args.output)
    if not output.is_absolute():
        output = REPO_ROOT / output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
