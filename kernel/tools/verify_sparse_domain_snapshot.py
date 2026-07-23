"""C-R2A offline verifier for a Blender-exported AnalysisSnapshotV1.

``export-blender`` runs in Blender and writes only host snapshots plus a mesh
fingerprint. ``verify`` runs in the Blender-free kernel environment and invokes
the independent source-face oracle. The oracle is therefore never imported by
the add-on runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace


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


def _export_blender(args) -> int:
    import bmesh
    import bpy

    from cftuv.analysis import build_analysis_bundle
    from cftuv.envelope_host_adapter import build_envelope_analysis_snapshot
    from cftuv.envelope_request_export import _selected_patch_scope
    from cftuv_envelope import AnalysisSnapshotCodecV1

    obj = bpy.data.objects.get(args.object)
    if obj is None or obj.type != "MESH":
        raise RuntimeError(f"mesh object {args.object!r} is unavailable")
    selected_edges = frozenset(int(item) for item in args.selected_edge)
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
    selected_patch_ids = _selected_patch_scope(bundle, selected_edges)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    domain_records = []
    for patch_id in sorted(selected_patch_ids):
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle,
                included_patch_ids=frozenset({patch_id}),
            )
        except Exception as exc:
            outcome = getattr(getattr(exc, "outcome", None), "value", None)
            domain_records.append(
                {
                    "patch_id": patch_id,
                    "stage": "METRIC_REJECTED",
                    "outcome": outcome or type(exc).__name__,
                    "message": str(exc),
                }
            )
            continue
        snapshot_path = output_dir / f"patch_{patch_id}_snapshot.json"
        snapshot_path.write_bytes(AnalysisSnapshotCodecV1.dumps(snapshot))
        domain = next(iter(snapshot.patch_domains))
        domain_records.append(
            {
                "patch_id": patch_id,
                "patch_domain_id": domain.patch_domain_id.value,
                "stage": "SNAPSHOT_EXPORTED",
                "snapshot_path": str(snapshot_path),
            }
        )
    after = _mesh_fingerprint(obj)
    manifest = {
        "schema": "cftuv.envelope.sparse_domain_blender_export.v1",
        "source_file": bpy.data.filepath,
        "source_object": obj.name,
        "mesh_vertices": len(obj.data.vertices),
        "mesh_edges": len(obj.data.edges),
        "mesh_faces": len(obj.data.polygons),
        "selected_physical_edge_ids": sorted(selected_edges),
        "selected_patch_ids": sorted(selected_patch_ids),
        "mesh_fingerprint_before": before,
        "mesh_fingerprint_after": after,
        "source_mesh_unchanged": before == after,
        "domains": domain_records,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
        newline="\n",
    )
    print(f"CFTUV_R2A_MANIFEST={manifest_path}")
    return 0


def _verify(args) -> int:
    from cftuv_envelope import AnalysisSnapshotCodecV1
    from cftuv_envelope.reference.common import GeometryContext
    from cftuv_envelope.reference.domain_geometry import (
        build_sparse_patch_domain_geometry,
    )
    from cftuv_envelope.reference.domain_oracle import (
        build_source_face_union_oracle,
        compare_sparse_domain_to_face_union,
    )

    manifest_path = Path(args.manifest)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    verified_domains = []
    for record in manifest["domains"]:
        if record["stage"] != "SNAPSHOT_EXPORTED":
            verified_domains.append(record)
            continue
        snapshot = AnalysisSnapshotCodecV1.loads(
            Path(record["snapshot_path"]).read_text(encoding="utf-8")
        )
        domain = next(iter(snapshot.patch_domains))
        frame = next(
            item
            for item in snapshot.surface_metric_descriptors
            if item.patch_domain_id == domain.patch_domain_id
        )
        compilation = SimpleNamespace(
            analysis_snapshot=snapshot,
            owner_patch_id=domain.owner_patch_id,
            plan_key=SimpleNamespace(
                patch_domain_id=domain.patch_domain_id
            ),
            source_provenance=(),
        )
        context = GeometryContext.build(compilation, frame)
        sparse = build_sparse_patch_domain_geometry(context)
        oracle = build_source_face_union_oracle(context)
        equivalence = compare_sparse_domain_to_face_union(context, sparse)
        public_record = {
            key: value
            for key, value in record.items()
            if key != "snapshot_path"
        }
        verified_domains.append(
            {
                **public_record,
                "stage": "EXACT_EQUIVALENT"
                if equivalence.exact_equivalent
                else "EQUIVALENCE_REJECTED",
                "source_face_count": equivalence.source_face_count,
                "face_boundary_segment_count": (
                    equivalence.face_boundary_segment_count
                ),
                "sparse_domain_segment_count": (
                    equivalence.sparse_domain_segment_count
                ),
                "outer_component_count": oracle.outer_component_count,
                "hole_count": oracle.hole_count,
                "exact_area_expression": oracle.exact_area_expression,
                "internal_source_edge_count": len(
                    equivalence.internal_source_edge_ids
                ),
                "internal_source_edges_absent": (
                    equivalence.internal_source_edges_absent
                ),
                "boundary_physical_edge_lineage_equal": (
                    equivalence.boundary_physical_edge_lineage_equal
                ),
                "boundary_geometry_equal": (
                    equivalence.boundary_geometry_equal
                ),
                "exact_equivalent": equivalence.exact_equivalent,
            }
        )
    report = {
        "schema": "cftuv.envelope.sparse_domain_report.v1",
        "source_file": manifest["source_file"],
        "source_object": manifest["source_object"],
        "mesh_vertices": manifest["mesh_vertices"],
        "mesh_edges": manifest["mesh_edges"],
        "mesh_faces": manifest["mesh_faces"],
        "selected_physical_edge_ids": manifest[
            "selected_physical_edge_ids"
        ],
        "selected_patch_ids": manifest["selected_patch_ids"],
        "mesh_fingerprint_before": manifest["mesh_fingerprint_before"],
        "mesh_fingerprint_after": manifest["mesh_fingerprint_after"],
        "source_mesh_unchanged": manifest["source_mesh_unchanged"],
        "domains": verified_domains,
    }
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if all(
        item["stage"] in {"EXACT_EQUIVALENT", "METRIC_REJECTED"}
        for item in verified_domains
    ) else 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    export = subparsers.add_parser("export-blender")
    export.add_argument("--object", required=True)
    export.add_argument(
        "--selected-edge", action="append", required=True, type=int
    )
    export.add_argument("--output-dir", required=True)
    export.set_defaults(handler=_export_blender)
    verify = subparsers.add_parser("verify")
    verify.add_argument("--manifest", required=True)
    verify.add_argument("--output", required=True)
    verify.set_defaults(handler=_verify)
    return parser


def main() -> int:
    argv = (
        sys.argv[sys.argv.index("--") + 1 :]
        if "--" in sys.argv
        else None
    )
    args = _parser().parse_args(argv)
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
