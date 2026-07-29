"""Blender exporter for the seven SEM-CLB-02 domain fixtures.

The exporter reuses the portable FIX-00 mesh/canonicalization helpers but keeps
the SEM-CLB-02 identities explicit.  It never saves the source scene.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


CASES = (
    {
        "fixture_id": "building_all_seams_patch_001_lost_resolved_v1",
        "object_name": "building",
        "scenario": "all_seams",
        "patch_id": 1,
    },
    {
        "fixture_id": "building_all_seams_patch_006_lost_resolved_v1",
        "object_name": "building",
        "scenario": "all_seams",
        "patch_id": 6,
    },
    {
        "fixture_id": "building_all_seams_patch_011_lost_resolved_v1",
        "object_name": "building",
        "scenario": "all_seams",
        "patch_id": 11,
    },
    {
        "fixture_id": "building_all_seams_patch_105_lost_resolved_v1",
        "object_name": "building",
        "scenario": "all_seams",
        "patch_id": 105,
    },
    {
        "fixture_id": "building_001_single_edge_patch_000_named_outcome_v1",
        "object_name": "building.001",
        "scenario": "single_edge",
        "patch_id": 0,
    },
    {
        "fixture_id": "building_002_single_edge_patch_000_named_outcome_v1",
        "object_name": "building.002",
        "scenario": "single_edge",
        "patch_id": 0,
    },
    {
        "fixture_id": "building_003_single_edge_patch_000_named_outcome_v1",
        "object_name": "building.003",
        "scenario": "single_edge",
        "patch_id": 0,
    },
)
ALPHA = 0.3
SCHEMA = "cftuv.envelope.sem_clb_02_domain_fixture.v1"


def _arguments() -> argparse.Namespace:
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(__file__).resolve().parent / "cases",
    )
    return parser.parse_args(raw)


def _git_sha(source_root: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source_root.resolve()), "rev-parse", "HEAD"],
        text=True,
    ).strip()


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


def _source_selection(obj, scenario: str) -> frozenset[int]:
    seam_edges = tuple(
        int(edge.index) for edge in obj.data.edges if edge.use_seam
    )
    if not seam_edges:
        raise RuntimeError(f"{obj.name!r} has no seam edges")
    if scenario == "all_seams":
        return frozenset(seam_edges)
    if scenario == "single_edge":
        return frozenset({seam_edges[0]})
    raise ValueError(f"unknown scenario: {scenario}")


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


def _host_mesh_payload(obj) -> dict[str, Any]:
    mesh = obj.data
    return {
        "schema": "cftuv.envelope.host_mesh_fixture.v1",
        "object_name": obj.name,
        "vertices": [
            {
                "index": int(vertex.index),
                "coordinate_binary64_hex": [
                    float(vertex.co[axis]).hex() for axis in range(3)
                ],
            }
            for vertex in mesh.vertices
        ],
        "edges": [
            {
                "index": int(edge.index),
                "vertex_indices": [int(value) for value in edge.vertices],
                "use_seam": bool(edge.use_seam),
            }
            for edge in mesh.edges
        ],
        "faces": [
            {
                "index": int(polygon.index),
                "vertex_cycle": [int(value) for value in polygon.vertices],
            }
            for polygon in mesh.polygons
        ],
    }


def _mesh_fingerprint_from_payload(payload: dict[str, Any]) -> str:
    normalized = {
        "vertices": [
            (
                item["index"],
                tuple(float.fromhex(value) for value in item["coordinate_binary64_hex"]),
            )
            for item in payload["vertices"]
        ],
        "edges": [
            (
                item["index"],
                tuple(item["vertex_indices"]),
                item["use_seam"],
            )
            for item in payload["edges"]
        ],
        "faces": [
            (item["index"], tuple(item["vertex_cycle"]))
            for item in payload["faces"]
        ],
    }
    return _sha256(_canonical_json(normalized))


def _mesh_fingerprint(obj) -> str:
    return _mesh_fingerprint_from_payload(_host_mesh_payload(obj))


def _export_object_cases(
    kernel,
    source_root: Path,
    output_root: Path,
    object_cases: tuple[dict[str, Any], ...],
) -> list[dict[str, Any]]:
    import bpy

    from cftuv.envelope_request_export import (
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )
    from cftuv.envelope_topology_export import (
        build_envelope_topology_export,
        stage_domain_inputs,
    )

    object_name = object_cases[0]["object_name"]
    scenario = object_cases[0]["scenario"]
    obj = bpy.data.objects.get(object_name)
    if obj is None or obj.type != "MESH":
        raise RuntimeError(f"{object_name!r} mesh object is unavailable")
    before = _mesh_fingerprint(obj)
    mesh_payload = _host_mesh_payload(obj)
    mesh_bytes = _canonical_json(mesh_payload)
    bundle = _bundle_for_object(obj)
    topology_export = build_envelope_topology_export(bundle)
    source_selection = _source_selection(obj, scenario)
    (
        _,
        revision,
        patch_ids,
        request_id,
        selected_edges_by_domain,
    ) = stage_domain_inputs(
        bundle,
        source_selection,
        topology_export=topology_export,
    )
    known_patch_ids = frozenset(patch_ids)
    reports = []
    for case in object_cases:
        fixture_id = case["fixture_id"]
        patch_id = int(case["patch_id"])
        if patch_id not in known_patch_ids:
            raise RuntimeError(
                f"{fixture_id}: patch {patch_id} is absent from selection scope"
            )
        domain_id = topology_export.patch_domain_id_by_patch[patch_id]
        snapshot = build_envelope_analysis_snapshot(
            bundle,
            included_patch_ids=frozenset({patch_id}),
            topology_export=topology_export,
        )
        request = build_envelope_decal_request(
            snapshot,
            frozenset(selected_edges_by_domain[domain_id]),
            ALPHA,
            decal_request_id_value=request_id,
        )
        snapshot_issues = kernel.validate_analysis_snapshot(snapshot)
        request_issues = kernel.validate_decal_request(request)
        reference_issues = kernel.validate_snapshot_request_references(
            snapshot, request
        )
        if snapshot_issues or request_issues or reference_issues:
            raise RuntimeError(
                f"{fixture_id}: contract validation failed: "
                f"snapshot={snapshot_issues!r} request={request_issues!r} "
                f"references={reference_issues!r}"
            )
        snapshot_bytes = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
        request_bytes = kernel.DecalRequestCodecV1.dumps(request)
        fixture_dir = output_root / fixture_id
        fixture_dir.mkdir(parents=True, exist_ok=True)
        (fixture_dir / "analysis_snapshot.json").write_bytes(snapshot_bytes)
        (fixture_dir / "decal_request.json").write_bytes(request_bytes)
        (fixture_dir / "host_mesh.json").write_bytes(mesh_bytes)
        file_hashes = {
            "analysis_snapshot.json": _sha256(snapshot_bytes),
            "decal_request.json": _sha256(request_bytes),
            "host_mesh.json": _sha256(mesh_bytes),
        }
        fixture_hash = _sha256(
            _canonical_json(
                [[name, file_hashes[name]] for name in sorted(file_hashes)]
            )
        )
        selected_uses = frozenset(request.selected_chain_use_ids)
        manifest = {
            "schema": SCHEMA,
            "fixture_id": fixture_id,
            "description": (
                f"SEM-CLB-02 {object_name}/{scenario} patch {patch_id} "
                "portable domain snapshot; no .blend content is included."
            ),
            "provenance": {
                "source_kind": "DEVELOPER_LOCAL_TESTSCENE_BLEND",
                "source_blend_path": bpy.data.filepath,
                "source_blend_sha256": _sha256(
                    Path(bpy.data.filepath).read_bytes()
                ),
                "source_path_included": True,
                "accepted_exporter_sha": _git_sha(source_root),
                "exporter": str(Path(__file__).resolve().relative_to(source_root)),
            },
            "object_name": obj.name,
            "object_data_name": obj.data.name,
            "selection_scenario": scenario,
            "selected_physical_edge_ids": sorted(source_selection),
            "effective_domain_physical_edge_ids": sorted(
                selected_edges_by_domain[domain_id]
            ),
            "patch_id": patch_id,
            "patch_domain_id": domain_id,
            "patch_domain_ids": sorted(
                item.patch_domain_id.value for item in snapshot.patch_domains
            ),
            "decal_request_id": request.decal_request_id.value,
            "selected_chain_use_ids": sorted(
                item.value for item in request.selected_chain_use_ids
            ),
            "selected_chain_use_to_physical_chain": sorted(
                (
                    {
                        "chain_use_id": item.chain_use_id.value,
                        "physical_chain_id": item.physical_chain_id.value,
                    }
                    for item in snapshot.chain_uses
                    if item.chain_use_id in selected_uses
                ),
                key=lambda item: item["chain_use_id"],
            ),
            "physical_chain_ids": sorted(
                item.physical_chain_id.value for item in snapshot.physical_chains
            ),
            "requested_alpha": str(ALPHA),
            "source_revision": snapshot.source_revision.value,
            "mesh_counts": {
                "vertices": len(mesh_payload["vertices"]),
                "edges": len(mesh_payload["edges"]),
                "faces": len(mesh_payload["faces"]),
            },
            "source_mesh_fingerprint": before,
            "source_mesh_unchanged": None,
            "contract_validation": {
                "analysis_snapshot_issue_count": 0,
                "decal_request_issue_count": 0,
                "cross_reference_issue_count": 0,
            },
            "expected_kernel_results": {},
            "files": file_hashes,
            "fixture_hash": fixture_hash,
        }
        after = _mesh_fingerprint(obj)
        manifest["source_mesh_unchanged"] = before == after
        manifest["source_mesh_fingerprint_after"] = after
        manifest_path = fixture_dir / "manifest.json"
        if manifest_path.exists():
            previous = json.loads(
                manifest_path.read_text(encoding="utf-8")
            )
            for field in (
                "expected_kernel_results",
                "semantic_change_receipt",
            ):
                if field in previous:
                    manifest[field] = previous[field]
        _write_json(manifest_path, manifest)
        reports.append(
            {
                "fixture_id": fixture_id,
                "patch_domain_id": domain_id,
                "fixture_hash": fixture_hash,
                "source_mesh_fingerprint_before": before,
                "source_mesh_fingerprint_after": after,
                "source_mesh_unchanged": before == after,
            }
        )
    after = _mesh_fingerprint(obj)
    if before != after:
        raise RuntimeError(
            f"{object_name}: source mesh changed during export: {before} != {after}"
        )
    return reports


def main() -> None:
    arguments = _arguments()
    kernel = _activate_source(arguments.source_root)
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for case in CASES:
        grouped.setdefault(
            (case["object_name"], case["scenario"]), []
        ).append(case)
    reports = []
    for object_cases in grouped.values():
        reports.extend(
            _export_object_cases(
                kernel,
                arguments.source_root,
                arguments.output_root.resolve(),
                tuple(object_cases),
            )
        )
    print(
        json.dumps(
            {
                "mode": "export",
                "kernel_revision": _git_sha(arguments.source_root),
                "fixture_count": len(reports),
                "fixtures": reports,
            },
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
