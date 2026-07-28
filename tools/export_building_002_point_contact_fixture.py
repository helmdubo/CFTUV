"""Portable FIX-00 fixture exporter and cross-revision kernel reproducer.

The ``export`` mode runs inside Blender and serializes only the immutable
AnalysisSnapshotV1 / DecalRequestV1 contracts plus the minimum source mesh
needed to rebuild the same host analysis input.  The ``verify`` mode is
Blender-free and evaluates those exact bytes with the kernel selected by
``--source-root``.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


SCHEMA = "cftuv.envelope.building_002_point_contact_fixture.v1"
HOST_MESH_SCHEMA = "cftuv.envelope.host_mesh_fixture.v1"
OBJECT_NAME = "building.002"
PATCH_ID = 0
SELECTED_EDGES = (2, 3, 7)
ALPHA = 0.25
HISTORICAL_BASE_SHA = "df587ed166cfb0e0b615148f08c583b4477c5ac4"
SELECTED_IMPLEMENTATION_SHA = "c2622d07020338e5231b81f41655fe6c74cdca72"
# Ревизия хоста, чей экспортёр признан извлекающей властью для отгруженных
# байтов. Константу читает ТОЛЬКО `_export`, и только чтобы записать её в
# `manifest.provenance.accepted_exporter_sha`; `_verify` её не читает вовсе,
# поэтому verify-режим не ломается ни на старом значении, ни на новом.
#
# Было `43e69d3` — экспортёр C-R2C, признанный властью потому, что host того
# времени отвергал точный сертификат угла до сериализации
# (`artifacts/envelope_fix_00/session_fix_00_handoff.json`). Тот блокер снят
# слиянием `cb444d5`, и полевая фикстура пересобрана уже починенным хостом.
#
# Что 43e69d3 НЕ МОГ записать эти байты — доказано, а не предположено:
# оба отгруженных `grid_certificate` объявляют `SOURCE_ONLY_GRID_SNAP_V1`,
# а этот закон появился в `4f596a9` (R1b, 25 июля), который НЕ является
# предком `43e69d3` (24 июля). Сверх того дескриптор отгруженной фикстуры
# побитово равен `build_rational_affine_planar_metric` под ровно теми
# политиками, что хост передаёт в `envelope_request_export.py:1836-1837`
# (`HOST_GRID_POLICY` / `HOST_PLANARITY_POLICY` из `cftuv/surface_ir.py`).
ACCEPTED_EXPORTER_SHA = "cb444d5a4f0f08a92f3fa70e39ee86e70e84e984"


def _arguments() -> argparse.Namespace:
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("export", "verify"))
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--fixture-dir", type=Path, required=True)
    parser.add_argument("--output-report", type=Path)
    parser.add_argument("--object-name", default=OBJECT_NAME)
    return parser.parse_args(raw)


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


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _git_sha(source_root: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source_root.resolve()), "rev-parse", "HEAD"],
        text=True,
    ).strip()


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
    )


def _host_mesh_payload(obj) -> dict[str, Any]:
    mesh = obj.data
    return {
        "schema": HOST_MESH_SCHEMA,
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


def _value(value: Any) -> Any:
    return value.value if hasattr(value, "value") else value


def _diagnostic_payload(kernel, diagnostic) -> dict[str, Any]:
    construction = (
        json.loads(kernel.canonical_json_bytes(diagnostic.construction))
        if diagnostic.construction is not None
        else None
    )
    return {
        "outcome": _value(diagnostic.outcome),
        "severity": _value(diagnostic.severity),
        "message": diagnostic.message,
        "envelope_spec_id": diagnostic.envelope_spec_id,
        "front_component_id": diagnostic.front_component_id,
        "construction": construction,
    }


def _canonical_record(kernel, value: Any) -> Any:
    return json.loads(kernel.canonical_json_bytes(value))


def _non_manifold_trace(kernel):
    """Capture the exact rejected vertex without changing arrangement behavior."""

    capture: dict[str, Any] = {}

    def trace(frame, event, arg):
        if event != "exception" or capture:
            return trace
        exception_type, _, _ = arg
        if exception_type.__name__ != "ExactArrangementNonManifold":
            return trace
        outgoing = frame.f_locals.get("outgoing")
        output_by_edge_id = frame.f_locals.get("output_by_edge_id")
        if not isinstance(outgoing, dict) or not isinstance(output_by_edge_id, dict):
            return trace
        rejected = []
        for vertex_id, edges in sorted(outgoing.items()):
            if len(edges) == 1:
                continue
            edge_payloads = []
            point = None
            for edge in sorted(edges, key=lambda item: item.edge_id):
                output = output_by_edge_id[edge.edge_id]
                point = output.start
                edge_payloads.append(
                    {
                        "edge_id": edge.edge_id,
                        "start_vertex_id": edge.start_vertex_id,
                        "end_vertex_id": edge.end_vertex_id,
                        "construction_certificates": sorted(
                            (
                                _canonical_record(kernel, item)
                                for item in edge.construction_certificates
                            ),
                            key=lambda item: _canonical_json(item),
                        ),
                        "provenance": _canonical_record(kernel, edge.provenance),
                    }
                )
            rejected.append(
                {
                    "arrangement_point_id": vertex_id,
                    "exact_point": (
                        _canonical_record(kernel, point) if point is not None else None
                    ),
                    "outgoing_edge_count": len(edges),
                    "outgoing_edges": edge_payloads,
                }
            )
        capture["rejected_vertices"] = rejected
        return trace

    return trace, capture


def _evaluation_payload(kernel, compile_result, request) -> dict[str, Any]:
    if compile_result.compilation is None:
        return {
            "stage": "COMPILE_REJECTED",
            "outcome": _value(compile_result.outcome),
            "diagnostics": [
                _diagnostic_payload(kernel, item)
                for item in compile_result.diagnostics
            ],
            "raw_coverage": None,
        }
    trace, trace_capture = _non_manifold_trace(kernel)
    previous_trace = sys.gettrace()
    sys.settrace(trace)
    try:
        result = kernel.evaluate_reference_raw_coverage(
            compile_result.compilation,
            request.requested_alpha,
        )
    finally:
        sys.settrace(previous_trace)
    raw = result.raw_coverage
    payload = {
        "stage": "RAW_READY" if raw is not None else "RAW_REJECTED",
        "outcome": _value(result.outcome),
        "diagnostics": [
            _diagnostic_payload(kernel, item) for item in result.diagnostics
        ],
        "raw_coverage": None,
    }
    if raw is None:
        if trace_capture:
            payload["rejected_topology"] = trace_capture
        return payload
    degrees: Counter[str] = Counter()
    for edge in raw.edges:
        degrees[edge.start_vertex_id] += 1
        degrees[edge.end_vertex_id] += 1
    occurrences = getattr(raw, "boundary_vertex_occurrences", ())
    point_contacts = getattr(raw, "point_contacts", ())
    vertex_by_id = {item.vertex_id: item for item in raw.vertices}
    occurrence_counts = Counter(
        item.arrangement_point_id for item in occurrences
    )
    payload["raw_coverage"] = {
        "schema_version": raw.schema_version,
        "semantic_digest": raw.semantic_digest,
        "exact_area_expression": raw.exact_area_expression,
        "vertex_count": len(raw.vertices),
        "edge_count": len(raw.edges),
        "loop_count": len(raw.loops),
        "region_count": len(raw.regions),
        "boundary_occurrence_count": len(occurrences),
        "point_contact_count": len(point_contacts),
        "shared_arrangement_point_count": sum(
            1 for count in occurrence_counts.values() if count > 1
        ),
        "max_incident_degree": max(degrees.values(), default=0),
        "point_contacts": sorted(
            (
                {
                    **_canonical_record(kernel, item),
                    "exact_point": _canonical_record(
                        kernel,
                        vertex_by_id[item.arrangement_point_id].point,
                    ),
                }
                for item in point_contacts
            ),
            key=lambda item: item["point_contact_id"],
        ),
        "shared_boundary_occurrences": sorted(
            (
                _canonical_record(kernel, item)
                for item in occurrences
                if occurrence_counts[item.arrangement_point_id] > 1
            ),
            key=lambda item: item["boundary_occurrence_id"],
        ),
    }
    return payload


def _export(arguments: argparse.Namespace) -> dict[str, Any]:
    import bmesh
    import bpy

    kernel = _activate_source(arguments.source_root)
    from cftuv.analysis import build_analysis_bundle
    from cftuv.envelope_request_export import (
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )

    obj = bpy.data.objects.get(arguments.object_name)
    if obj is None or obj.type != "MESH":
        raise RuntimeError(f"{arguments.object_name!r} mesh object is unavailable")
    before = _mesh_fingerprint(obj)
    bm = bmesh.new()
    try:
        bm.from_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        bundle = build_analysis_bundle(
            bm,
            tuple(face.index for face in bm.faces),
            obj,
        )
    finally:
        bm.free()
    snapshot = build_envelope_analysis_snapshot(
        bundle,
        included_patch_ids=frozenset({PATCH_ID}),
    )
    request = build_envelope_decal_request(
        snapshot,
        frozenset(SELECTED_EDGES),
        ALPHA,
    )
    snapshot_issues = kernel.validate_analysis_snapshot(snapshot)
    request_issues = kernel.validate_decal_request(request)
    reference_issues = kernel.validate_snapshot_request_references(
        snapshot, request
    )
    if snapshot_issues or request_issues or reference_issues:
        raise RuntimeError(
            "fixture contracts failed validation: "
            f"snapshot={snapshot_issues!r} request={request_issues!r} "
            f"references={reference_issues!r}"
        )

    fixture_dir = arguments.fixture_dir.resolve()
    fixture_dir.mkdir(parents=True, exist_ok=True)
    snapshot_bytes = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    request_bytes = kernel.DecalRequestCodecV1.dumps(request)
    mesh_payload = _host_mesh_payload(obj)
    mesh_bytes = _canonical_json(mesh_payload)
    (fixture_dir / "analysis_snapshot.json").write_bytes(snapshot_bytes)
    (fixture_dir / "decal_request.json").write_bytes(request_bytes)
    (fixture_dir / "host_mesh.json").write_bytes(mesh_bytes)

    file_hashes = {
        "analysis_snapshot.json": _sha256(snapshot_bytes),
        "decal_request.json": _sha256(request_bytes),
        "host_mesh.json": _sha256(mesh_bytes),
    }
    fixture_hash = _sha256(
        _canonical_json([[name, file_hashes[name]] for name in sorted(file_hashes)])
    )
    manifest = {
        "schema": SCHEMA,
        "fixture_id": "building_002_point_contact_v1",
        "description": (
            "Minimal portable building.002 patch-0 point-contact reproducer; "
            "no proprietary .blend content is included."
        ),
        "provenance": {
            "source_kind": "DEVELOPER_LOCAL_BUILDING_002",
            "source_path_included": False,
            "accepted_exporter_sha": ACCEPTED_EXPORTER_SHA,
        },
        "object_name": arguments.object_name,
        "patch_id": PATCH_ID,
        "patch_domain_ids": sorted(
            item.patch_domain_id.value for item in snapshot.patch_domains
        ),
        "selected_physical_edge_ids": list(SELECTED_EDGES),
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
                if item.chain_use_id in request.selected_chain_use_ids
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
        "source_mesh_unchanged": before == _mesh_fingerprint(obj),
        "contract_validation": {
            "analysis_snapshot_issue_count": 0,
            "decal_request_issue_count": 0,
            "cross_reference_issue_count": 0,
        },
        "expected_kernel_results": {
            HISTORICAL_BASE_SHA: {
                "outcome": "REFERENCE_ARRANGEMENT_NON_MANIFOLD",
                "stage": "RAW_REJECTED",
            },
            SELECTED_IMPLEMENTATION_SHA: {
                "outcome": "EXACT",
                "stage": "RAW_READY",
                "raw_schema_version": "cftuv.envelope.raw_coverage_result.v2",
                "semantic_digest": (
                    "622e1f6eec09e64bc1294c37643af19f630086005f9af21f10ac4cd6ed0e987a"
                ),
                "exact_area_expression": (
                    "Rational(122766786560, 373821260323)"
                ),
                "boundary_occurrence_count": 12,
                "point_contact_count": 2,
                "loop_count": 3,
                "region_count": 3,
            },
        },
        "files": file_hashes,
        "fixture_hash": fixture_hash,
    }
    _write_json(fixture_dir / "manifest.json", manifest)
    result = {
        "mode": "export",
        "fixture_id": manifest["fixture_id"],
        "accepted_exporter_sha": _git_sha(arguments.source_root),
        "fixture_hash": fixture_hash,
        "source_mesh_fingerprint_before": before,
        "source_mesh_fingerprint_after": _mesh_fingerprint(obj),
        "source_mesh_unchanged": before == _mesh_fingerprint(obj),
        "source_revision": snapshot.source_revision.value,
        "selected_chain_use_ids": manifest["selected_chain_use_ids"],
    }
    if arguments.output_report:
        _write_json(arguments.output_report, result)
    return result


def _verify(arguments: argparse.Namespace) -> dict[str, Any]:
    kernel = _activate_source(arguments.source_root)
    fixture_dir = arguments.fixture_dir.resolve()
    snapshot_bytes = (fixture_dir / "analysis_snapshot.json").read_bytes()
    request_bytes = (fixture_dir / "decal_request.json").read_bytes()
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    request = kernel.DecalRequestCodecV1.loads(request_bytes)
    issues = {
        "analysis_snapshot": [
            str(item) for item in kernel.validate_analysis_snapshot(snapshot)
        ],
        "decal_request": [
            str(item) for item in kernel.validate_decal_request(request)
        ],
        "cross_references": [
            str(item)
            for item in kernel.validate_snapshot_request_references(
                snapshot, request
            )
        ],
    }
    compile_result = kernel.compile_reference_envelopes(snapshot, request)
    result = {
        "schema": "cftuv.envelope.building_002_kernel_reproduction.v1",
        "mode": "verify",
        "kernel_revision": _git_sha(arguments.source_root),
        "kernel_version": kernel.__version__,
        "fixture_hash": json.loads(
            (fixture_dir / "manifest.json").read_text(encoding="utf-8")
        )["fixture_hash"],
        "validation_issues": issues,
        "evaluation": _evaluation_payload(kernel, compile_result, request),
    }
    if any(issues.values()):
        raise RuntimeError(f"fixture contract validation failed: {issues!r}")
    if arguments.output_report:
        _write_json(arguments.output_report, result)
    return result


def main() -> None:
    arguments = _arguments()
    result = _export(arguments) if arguments.mode == "export" else _verify(arguments)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
