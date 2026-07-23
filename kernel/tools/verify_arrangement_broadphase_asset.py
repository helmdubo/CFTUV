"""C-R2B Blender asset verifier for exact broadphase telemetry.

The script evaluates the current staged Envelope path, records production
broadphase counters, then reruns the same compiled plans with the permanent
exhaustive oracle. It never saves or mutates the source mesh.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import sys
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
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


def _counter_map(profile) -> dict[tuple[str, str | None], int | float]:
    return {
        (item.name, item.patch_domain_id): item.value
        for item in profile.counters
    }


def _counter(
    counters: dict[tuple[str, str | None], int | float],
    name: str,
    patch_domain_id: str,
) -> int:
    value = counters.get((name, patch_domain_id), 0)
    if isinstance(value, float) and not value.is_integer():
        raise RuntimeError(f"counter {name} is not integral: {value}")
    return int(value)


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
        "EXACT_REFERENCE_BROADPHASE",
    )
    evaluation = evaluate_envelope_debug_staged(
        bundle,
        selected_edges,
        args.alpha,
        profile=profile_builder,
    )
    profile = profile_builder.snapshot()
    counters = _counter_map(profile)

    raw_module = importlib.import_module(
        "cftuv_envelope.reference.raw_coverage"
    )
    production_backend = raw_module.REFERENCE_ARRANGEMENT_BACKEND
    exhaustive_backend = ExhaustiveExactArrangementBackend()
    domain_records = []
    try:
        raw_module.REFERENCE_ARRANGEMENT_BACKEND = exhaustive_backend
        for domain_evaluation in evaluation.domains:
            domain_id = domain_evaluation.patch_domain_id
            record = {
                "patch_id": domain_evaluation.patch_id,
                "patch_domain_id": domain_id,
                "stage": domain_evaluation.receipt.stage.value,
                "outcome": domain_evaluation.receipt.outcome,
            }
            if (
                domain_evaluation.compilation is None
                or domain_evaluation.request is None
                or domain_evaluation.raw_result is None
            ):
                domain_records.append(record)
                continue
            oracle_evaluation = raw_module.evaluate_reference_raw_coverage(
                domain_evaluation.compilation,
                domain_evaluation.request.requested_alpha,
            )
            oracle_result = oracle_evaluation.raw_coverage
            if oracle_result is None:
                raise RuntimeError(
                    "exhaustive oracle rejected a production-resolved domain: "
                    + domain_id
                )
            all_pairs = _counter(
                counters,
                "ARRANGEMENT_ALL_POSSIBLE_PAIRS",
                domain_id,
            )
            candidate_pairs = _counter(
                counters,
                "ARRANGEMENT_BROADPHASE_CANDIDATE_PAIRS",
                domain_id,
            )
            narrowphase_tests = _counter(
                counters,
                "ARRANGEMENT_NARROWPHASE_TESTS",
                domain_id,
            )
            record.update(
                {
                    "input_segments": _counter(
                        counters,
                        "ARRANGEMENT_INPUT_SEGMENTS",
                        domain_id,
                    ),
                    "all_possible_pairs": all_pairs,
                    "broadphase_candidate_pairs": candidate_pairs,
                    "narrowphase_tests": narrowphase_tests,
                    "actual_intersections": _counter(
                        counters,
                        "ARRANGEMENT_INTERSECTIONS",
                        domain_id,
                    ),
                    "atomic_edges": _counter(
                        counters,
                        "ARRANGEMENT_ATOMIC_SEGMENTS",
                        domain_id,
                    ),
                    "candidate_pair_reduction": (
                        all_pairs - candidate_pairs
                    ),
                    "candidate_pair_reduction_fraction": (
                        f"{all_pairs - candidate_pairs}/{all_pairs}"
                        if all_pairs
                        else "0/0"
                    ),
                    "broadphase_semantic_digest": (
                        domain_evaluation.raw_result.semantic_digest
                    ),
                    "exhaustive_semantic_digest": (
                        oracle_result.semantic_digest
                    ),
                    "exact_differential_equal": (
                        domain_evaluation.raw_result == oracle_result
                    ),
                }
            )
            domain_records.append(record)
    finally:
        raw_module.REFERENCE_ARRANGEMENT_BACKEND = production_backend

    after = _mesh_fingerprint(obj)
    exact_records = [
        item for item in domain_records if "all_possible_pairs" in item
    ]
    totals = {
        key: sum(item[key] for item in exact_records)
        for key in (
            "input_segments",
            "all_possible_pairs",
            "broadphase_candidate_pairs",
            "narrowphase_tests",
            "actual_intersections",
            "atomic_edges",
            "candidate_pair_reduction",
        )
    }
    report = {
        "schema": "cftuv.envelope.arrangement_broadphase_asset_report.v1",
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
        "backend_identity": production_backend.backend_identity,
        "backend_version": production_backend.backend_version,
        "oracle_identity": exhaustive_backend.backend_identity,
        "domains": domain_records,
        "totals": totals,
        "exact_differential_domain_count": len(exact_records),
        "non_resolved_domain_count": (
            len(domain_records) - len(exact_records)
        ),
        "all_resolved_domain_differentials_equal": all(
            item.get("exact_differential_equal", True)
            for item in exact_records
        ),
    }
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if (
        report["source_mesh_unchanged"]
        and report["all_resolved_domain_differentials_equal"]
        and all(
            item["narrowphase_tests"] <= item["all_possible_pairs"]
            for item in exact_records
        )
    ) else 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", required=True)
    parser.add_argument(
        "--selected-edge",
        action="append",
        required=True,
        type=int,
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
