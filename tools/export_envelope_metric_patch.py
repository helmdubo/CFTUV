"""Export current AnalysisBundle facts for the M-R0 metric spike.

Run under Blender. The exporter reads only current host analysis records,
records the existing V1 admission outcome, and never saves or mutates the
source mesh.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
for import_root in (
    REPOSITORY_ROOT,
    REPOSITORY_ROOT / "kernel" / "src",
):
    root_text = str(import_root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)


def _mesh_fingerprint(obj) -> str:
    mesh = obj.data
    payload = {
        "vertices": [
            (item.index, tuple(float(value) for value in item.co))
            for item in mesh.vertices
        ],
        "edges": [
            (
                item.index,
                tuple(int(value) for value in item.vertices),
                bool(item.use_seam),
            )
            for item in mesh.edges
        ],
        "polygons": [
            (item.index, tuple(int(value) for value in item.vertices))
            for item in mesh.polygons
        ],
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode(
            "utf-8"
        )
    ).hexdigest()


def _run(args) -> int:
    import bmesh
    import bpy

    from cftuv.analysis import build_analysis_bundle
    from cftuv.envelope_request_export import (
        build_envelope_analysis_snapshot,
    )

    obj = bpy.data.objects.get(args.object)
    if obj is None or obj.type != "MESH":
        raise RuntimeError(f"mesh object {args.object!r} is unavailable")
    before = _mesh_fingerprint(obj)
    bm = bmesh.new()
    try:
        bm.from_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        bundle = build_analysis_bundle(
            bm,
            frozenset(face.index for face in bm.faces),
            obj,
        )
    finally:
        bm.free()

    patch_id = int(args.patch_id)
    patch = bundle.patch_graph.nodes.get(patch_id)
    if patch is None:
        raise RuntimeError(
            f"Patch {patch_id} is unavailable; "
            f"known={sorted(bundle.patch_graph.nodes)}"
        )
    patch_faces = bundle.patch_surface.patch_faces(patch_id)
    patch_vertex_ids = sorted(
        {
            int(vertex_id)
            for face in patch_faces
            for vertex_id in face.vertex_cycle
        }
    )
    vertex_by_id = bundle.patch_surface.vertex_by_id

    admission = {
        "status": "ADMITTED",
        "outcome": "PLANAR_PATCH_FRAME_V1",
        "message": "",
    }
    try:
        snapshot = build_envelope_analysis_snapshot(
            bundle,
            included_patch_ids=frozenset({patch_id}),
        )
        frame = next(iter(snapshot.surface_metric_descriptors))
        admission["patch_domain_id"] = frame.patch_domain_id.value
    except Exception as exc:
        outcome = getattr(getattr(exc, "outcome", None), "value", None)
        admission = {
            "status": "REJECTED",
            "outcome": outcome or type(exc).__name__,
            "message": str(exc),
        }

    loops = []
    for loop_index, loop in enumerate(patch.boundary_loops):
        loops.append(
            {
                "loop_index": loop_index,
                "kind": loop.kind.value,
                "vertex_ids": [int(item) for item in loop.vert_indices],
                "edge_ids": [int(item) for item in loop.edge_indices],
                "chains": [
                    {
                        "chain_index": chain_index,
                        "vertex_ids": [
                            int(item) for item in chain.vert_indices
                        ],
                        "edge_ids": [
                            int(item) for item in chain.edge_indices
                        ],
                        "neighbor_kind": chain.neighbor_kind.value,
                        "neighbor_patch_id": int(chain.neighbor_patch_id),
                    }
                    for chain_index, chain in enumerate(loop.chains)
                ],
            }
        )

    after = _mesh_fingerprint(obj)
    payload = {
        "schema": "cftuv.envelope.metric_spike_host_patch.v1",
        "source_file": bpy.data.filepath,
        "source_object": obj.name,
        "source_revision": {
            "source_name": bundle.source_revision.source_name,
            "digest": bundle.source_revision.digest,
        },
        "patch_id": patch_id,
        "patch_ids": sorted(int(item) for item in bundle.patch_graph.nodes),
        "mesh_vertices": len(obj.data.vertices),
        "mesh_edges": len(obj.data.edges),
        "mesh_faces": len(obj.data.polygons),
        "mesh_fingerprint_before": before,
        "mesh_fingerprint_after": after,
        "source_mesh_unchanged": before == after,
        "host_patch": {
            "centroid": [float(value) for value in patch.centroid],
            "normal": [float(value) for value in patch.normal],
            "basis_u": [float(value) for value in patch.basis_u],
            "basis_v": [float(value) for value in patch.basis_v],
            "face_ids": [int(item) for item in patch.face_indices],
            "vertex_ids": patch_vertex_ids,
        },
        "vertices": [
            {
                "vertex_id": vertex_id,
                "position": [
                    float(value)
                    for value in vertex_by_id[vertex_id].position
                ],
            }
            for vertex_id in patch_vertex_ids
        ],
        "faces": [
            {
                "face_id": int(face.face_id),
                "vertex_cycle": [
                    int(item) for item in face.vertex_cycle
                ],
                "edge_cycle": [int(item) for item in face.edge_cycle],
                "triangle_ids": [
                    int(item) for item in face.triangle_ids
                ],
            }
            for face in patch_faces
        ],
        "triangles": [
            {
                "triangle_id": int(triangle.triangle_id),
                "source_face_id": int(triangle.source_face_id),
                "vertex_ids": [
                    int(item) for item in triangle.vertex_ids
                ],
            }
            for triangle in bundle.patch_surface.patch_triangles(patch_id)
        ],
        "boundary_loops": loops,
        "current_v1_admission": admission,
    }
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True),
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["source_mesh_unchanged"] else 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", required=True)
    parser.add_argument("--patch-id", required=True, type=int)
    parser.add_argument("--output", required=True)
    return parser


def main() -> int:
    argv = (
        sys.argv[sys.argv.index("--") + 1 :]
        if "--" in sys.argv
        else None
    )
    return _run(_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
