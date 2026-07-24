"""C-R2C field verifier for exact boundary occurrences and point contacts."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
for import_root in (REPOSITORY_ROOT, REPOSITORY_ROOT / "kernel" / "src"):
    root_text = str(import_root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)


def _mesh_fingerprint(obj) -> str:
    payload = {
        "vertices": [
            (item.index, tuple(float(value) for value in item.co))
            for item in obj.data.vertices
        ],
        "edges": [
            (
                item.index,
                tuple(int(value) for value in item.vertices),
                bool(item.use_seam),
            )
            for item in obj.data.edges
        ],
        "polygons": [
            (item.index, tuple(int(value) for value in item.vertices))
            for item in obj.data.polygons
        ],
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode(
            "utf-8"
        )
    ).hexdigest()


def _counter_map(profile):
    return {
        (item.name, item.patch_domain_id): item.value
        for item in profile.counters
    }


def _counter(counters, name: str, domain_id: str) -> int:
    value = counters.get((name, domain_id), 0)
    return int(value)


def _raw_topology_record(domain_evaluation, counters) -> dict:
    domain_id = domain_evaluation.patch_domain_id
    raw = domain_evaluation.raw_result
    record = {
        "patch_id": domain_evaluation.patch_id,
        "patch_domain_id": domain_id,
        "stage": domain_evaluation.receipt.stage.value,
        "outcome": domain_evaluation.receipt.outcome,
        "raw_ready": raw is not None,
    }
    if raw is None:
        record["message"] = domain_evaluation.receipt.message
        return record

    edge_ids = {item.edge_id for item in raw.edges}
    incoming_occurrence_edges = {
        item.incoming_boundary_edge_id
        for item in raw.boundary_vertex_occurrences
    }
    outgoing_occurrence_edges = {
        item.outgoing_boundary_edge_id
        for item in raw.boundary_vertex_occurrences
    }
    contact_point_ids = {
        item.arrangement_point_id for item in raw.point_contacts
    }
    record.update(
        {
            "raw_schema_version": raw.schema_version,
            "raw_semantic_digest": raw.semantic_digest,
            "shared_arrangement_point_count": len(contact_point_ids),
            "boundary_occurrence_count": len(
                raw.boundary_vertex_occurrences
            ),
            "point_contact_count": len(raw.point_contacts),
            "loop_count": len(raw.loops),
            "region_count": len(raw.regions),
            "exact_area": raw.exact_area_expression,
            "halfedges_consumed_exactly_once": (
                incoming_occurrence_edges == edge_ids
                and outgoing_occurrence_edges == edge_ids
                and len(raw.boundary_vertex_occurrences) == len(raw.edges)
            ),
            "provenance_complete": all(
                item.provenance.envelope_instance_ids
                and item.provenance.envelope_spec_ids
                for item in raw.boundary_vertex_occurrences
            )
            and all(
                item.provenance.envelope_instance_ids
                and item.participating_loop_ids
                for item in raw.point_contacts
            ),
            "branch_point_count": _counter(
                counters, "ARRANGEMENT_BRANCH_POINTS", domain_id
            ),
            "max_incident_degree": _counter(
                counters, "ARRANGEMENT_MAX_INCIDENT_DEGREE", domain_id
            ),
            "rotation_comparisons": _counter(
                counters, "ARRANGEMENT_ROTATION_COMPARISONS", domain_id
            ),
            "face_walk_count": _counter(
                counters, "ARRANGEMENT_FACE_WALKS", domain_id
            ),
        }
    )
    return record


def _run(args) -> int:
    import bmesh
    import bpy

    from cftuv.analysis import build_analysis_bundle
    from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1
    from cftuv.envelope_host_adapter import evaluate_envelope_debug_staged
    from cftuv_envelope.reference.arrangement import (
        ExhaustiveExactArrangementBackend,
    )

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

    profile_builder = EnvelopeDebugProfileBuilderV1(
        obj.name,
        "C_R2C_BOUNDARY_ROTATION_FIELD",
    )
    evaluation = evaluate_envelope_debug_staged(
        bundle,
        selected_edges,
        args.alpha,
        profile=profile_builder,
    )
    profile = profile_builder.snapshot()
    counters = _counter_map(profile)
    records = [
        _raw_topology_record(item, counters)
        for item in evaluation.domains
    ]

    raw_module = importlib.import_module(
        "cftuv_envelope.reference.raw_coverage"
    )
    production_backend = raw_module.REFERENCE_ARRANGEMENT_BACKEND
    try:
        raw_module.REFERENCE_ARRANGEMENT_BACKEND = (
            ExhaustiveExactArrangementBackend()
        )
        for domain_evaluation, record in zip(evaluation.domains, records):
            if (
                domain_evaluation.compilation is None
                or domain_evaluation.request is None
                or domain_evaluation.raw_result is None
            ):
                record["exhaustive_differential_equal"] = None
                continue
            oracle = raw_module.evaluate_reference_raw_coverage(
                domain_evaluation.compilation,
                domain_evaluation.request.requested_alpha,
            )
            record["exhaustive_outcome"] = oracle.outcome.value
            record["exhaustive_differential_equal"] = (
                oracle.raw_coverage == domain_evaluation.raw_result
            )
    finally:
        raw_module.REFERENCE_ARRANGEMENT_BACKEND = production_backend

    after = _mesh_fingerprint(obj)
    patch_zero = next(
        (item for item in records if item["patch_id"] == 0),
        None,
    )
    report = {
        "schema": "cftuv.envelope.boundary_rotation_asset_report.v1",
        "session": "SESSION_C_R2C_REGULARIZED_BOUNDARY_ROTATION_SYSTEM",
        "source_file": bpy.data.filepath,
        "source_object": obj.name,
        "requested_alpha": str(args.alpha),
        "selected_physical_edge_ids": sorted(selected_edges),
        "mesh_vertices": len(obj.data.vertices),
        "mesh_edges": len(obj.data.edges),
        "mesh_faces": len(obj.data.polygons),
        "mesh_fingerprint_before": before,
        "mesh_fingerprint_after": after,
        "source_mesh_unchanged": before == after,
        "domains": records,
        "patch_0_raw_ready": bool(
            patch_zero and patch_zero["raw_ready"]
        ),
        "patch_0_old_non_manifold_absent": bool(
            patch_zero
            and patch_zero["outcome"]
            != "REFERENCE_ARRANGEMENT_NON_MANIFOLD"
        ),
    }
    report["gate_passed"] = bool(
        report["source_mesh_unchanged"]
        and report["patch_0_raw_ready"]
        and report["patch_0_old_non_manifold_absent"]
        and all(
            item.get("halfedges_consumed_exactly_once", True)
            for item in records
        )
        and all(
            item.get("provenance_complete", True)
            for item in records
        )
        and all(
            item.get("exhaustive_differential_equal") in (None, True)
            for item in records
        )
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["gate_passed"] else 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", required=True)
    parser.add_argument(
        "--selected-edge", action="append", required=True, type=int
    )
    parser.add_argument("--alpha", required=True, type=float)
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
