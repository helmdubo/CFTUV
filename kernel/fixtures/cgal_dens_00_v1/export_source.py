"""Export the central building.002 density domain without saving the .blend.

This exporter deliberately stops before reference compilation or skeleton
construction.  Its only purpose is to freeze the real host input and prove how
many zero-length moving fan sources the accepted density selector requires.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


OBJECT_NAME = "building.002"
PATCH_ID = 0
ALPHA = 0.3
DENSITIES = (0, 1, 4)
SCHEMA = "cftuv.proof.cgal_dens_00.source_snapshot.v1"


def _arguments() -> argparse.Namespace:
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    return parser.parse_args(raw)


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_json(payload: Any) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def _git_sha(source_root: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source_root.resolve()), "rev-parse", "HEAD"],
        text=True,
    ).strip()


def _activate_source(source_root: Path):
    source_root = source_root.resolve()
    for module_name in tuple(sys.modules):
        if (
            module_name == "cftuv"
            or module_name.startswith("cftuv.")
            or module_name == "cftuv_envelope"
            or module_name.startswith("cftuv_envelope.")
        ):
            del sys.modules[module_name]
    for entry in (source_root / "kernel" / "src", source_root):
        value = str(entry)
        if value in sys.path:
            sys.path.remove(value)
        sys.path.insert(0, value)
    import cftuv_envelope as kernel

    return kernel


def _bundle_for_object(obj):
    import bmesh
    import bpy

    from cftuv.analysis import build_analysis_bundle

    bpy.context.view_layer.objects.active = obj
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    try:
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        return build_analysis_bundle(
            bm,
            tuple(face.index for face in bm.faces),
            obj,
        )
    finally:
        bpy.ops.object.mode_set(mode="OBJECT")


def _mesh_payload(obj) -> dict[str, Any]:
    mesh = obj.data
    return {
        "vertices": [
            (
                int(vertex.index),
                tuple(float(vertex.co[axis]).hex() for axis in range(3)),
            )
            for vertex in mesh.vertices
        ],
        "edges": [
            (
                int(edge.index),
                tuple(int(value) for value in edge.vertices),
                bool(edge.use_seam),
            )
            for edge in mesh.edges
        ],
        "faces": [
            (
                int(polygon.index),
                tuple(int(value) for value in polygon.vertices),
            )
            for polygon in mesh.polygons
        ],
    }


def _mesh_fingerprint(obj) -> str:
    return _sha256(_canonical_json(_mesh_payload(obj)))


def _value(item) -> str:
    return str(item.value) if hasattr(item, "value") else str(item)


def _density_rows(kernel, snapshot, request, patch_domain_id) -> list[dict[str, Any]]:
    from cftuv_envelope.reference.compile import (
        _resolve_angular_profile_selection,
    )

    local_use_ids = frozenset(request.selected_chain_use_ids)
    owner_sectors = {
        item.owner_sector_id: item for item in snapshot.angular_owner_sectors
    }
    angle_certificates = {
        item.certificate_id: item for item in snapshot.reflex_angle_certificates
    }
    rows = []
    for relation in sorted(
        snapshot.corner_relations,
        key=lambda item: item.corner_relation_id.value,
    ):
        sector = owner_sectors.get(relation.owner_sector_id)
        if (
            sector is None
            or _value(sector.patch_domain_id) != _value(patch_domain_id)
        ):
            continue
        if not frozenset(sector.ordered_incident_chain_use_ids).issubset(
            local_use_ids
        ):
            continue
        certificate = angle_certificates.get(
            relation.reflex_angle_certificate_id
        )
        if (
            certificate is None
            or not isinstance(
                certificate.measure_payload,
                kernel.CertifiedReflexAngleMeasureV1,
            )
        ):
            raise RuntimeError(
                f"{relation.corner_relation_id.value}: no certified reflex angle"
            )
        resolved = _resolve_angular_profile_selection(
            request,
            certificate.measure_payload,
        )
        if resolved is None:
            raise RuntimeError(
                f"{relation.corner_relation_id.value}: density is not exact"
            )
        rows.append(
            {
                "corner_relation_id": relation.corner_relation_id.value,
                "source_vertex_id": relation.source_vertex_id.value,
                "owner_sector_id": relation.owner_sector_id.value,
                "hidden_support_count": int(resolved.hidden_count),
                "zero_length_at_alpha_zero": True,
                "separate_initial_front_channel": True,
            }
        )
    return rows


def main() -> None:
    import bpy

    arguments = _arguments()
    source_root = arguments.source_root.resolve()
    output_root = arguments.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    kernel = _activate_source(source_root)

    from cftuv.envelope_request_export import (
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import (
        build_envelope_topology_export,
        stage_domain_inputs,
    )

    obj = bpy.data.objects.get(OBJECT_NAME)
    if obj is None or obj.type != "MESH":
        raise RuntimeError(f"{OBJECT_NAME!r} mesh object is unavailable")
    before = _mesh_fingerprint(obj)
    bundle = _bundle_for_object(obj)
    topology_export = build_envelope_topology_export(bundle)
    selected_edges = frozenset(
        int(edge.index) for edge in obj.data.edges if edge.use_seam
    )
    if not selected_edges:
        raise RuntimeError(f"{OBJECT_NAME!r} has no seam edges")
    (
        _,
        _revision,
        patch_ids,
        request_id,
        selected_edges_by_domain,
    ) = stage_domain_inputs(
        bundle,
        selected_edges,
        topology_export=topology_export,
    )
    if PATCH_ID not in frozenset(patch_ids):
        raise RuntimeError(f"central patch {PATCH_ID} is absent")
    patch_domain_id = topology_export.patch_domain_id_by_patch[PATCH_ID]
    snapshot = build_envelope_analysis_snapshot(
        bundle,
        included_patch_ids=frozenset({PATCH_ID}),
        topology_export=topology_export,
    )
    snapshot_issues = kernel.validate_analysis_snapshot(snapshot)
    if snapshot_issues:
        raise RuntimeError(f"analysis snapshot invalid: {snapshot_issues!r}")
    snapshot_bytes = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    (output_root / "analysis_snapshot.json").write_bytes(snapshot_bytes)

    density_rows = []
    request_hashes = {}
    for density in DENSITIES:
        request = build_envelope_decal_request(
            snapshot,
            frozenset(selected_edges_by_domain[patch_domain_id]),
            ALPHA,
            decal_request_id_value=request_id,
            density=density,
        )
        request_issues = kernel.validate_decal_request(request)
        reference_issues = kernel.validate_snapshot_request_references(
            snapshot,
            request,
        )
        if request_issues or reference_issues:
            raise RuntimeError(
                f"density {density}: request={request_issues!r}, "
                f"references={reference_issues!r}"
            )
        request_bytes = kernel.DecalRequestCodecV1.dumps(request)
        request_name = f"decal_request_d{density}.json"
        (output_root / request_name).write_bytes(request_bytes)
        request_hashes[request_name] = _sha256(request_bytes)
        rows = _density_rows(kernel, snapshot, request, patch_domain_id)
        density_rows.append(
            {
                "density": density,
                "max_subturn_value_id": _value(
                    request.max_subturn_value_id
                ),
                "reflex_vertex_count": len(rows),
                "fan_source_count": sum(
                    item["hidden_support_count"] for item in rows
                ),
                "relations": rows,
            }
        )

    after = _mesh_fingerprint(obj)
    if before != after:
        raise RuntimeError(
            f"source mesh changed during export: {before} != {after}"
        )
    source_blend_path = Path(bpy.data.filepath)
    files = {
        "analysis_snapshot.json": _sha256(snapshot_bytes),
        **request_hashes,
    }
    receipt = {
        "schema": SCHEMA,
        "base_revision": _git_sha(source_root),
        "source": {
            "kind": "DEVELOPER_LOCAL_TESTSCENE_BLEND",
            "blend_path": str(source_blend_path),
            "blend_sha256": _sha256(source_blend_path.read_bytes()),
            "object_name": obj.name,
            "object_data_name": obj.data.name,
            "patch_id": PATCH_ID,
            "patch_domain_id": _value(patch_domain_id),
            "selected_seam_edge_ids": sorted(selected_edges),
            "effective_domain_edge_ids": sorted(
                selected_edges_by_domain[patch_domain_id]
            ),
            "mesh_counts": {
                "vertices": len(obj.data.vertices),
                "edges": len(obj.data.edges),
                "faces": len(obj.data.polygons),
            },
            "mesh_fingerprint_before": before,
            "mesh_fingerprint_after": after,
            "mesh_unchanged": before == after,
        },
        "density_profiles": density_rows,
        "files": files,
    }
    _write_json(output_root / "source_snapshot.json", receipt)
    print(json.dumps(receipt, ensure_ascii=False, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
