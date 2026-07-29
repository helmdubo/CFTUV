"""Blender-free SEM-CLB-02 fixture verifier."""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument(
        "--fixture-root",
        type=Path,
        default=Path(__file__).resolve().parent / "cases",
    )
    parser.add_argument("--output-report", type=Path)
    return parser.parse_args()


def _git_sha(source_root: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source_root.resolve()), "rev-parse", "HEAD"],
        text=True,
    ).strip()


def _activate_source(source_root: Path):
    source_root = source_root.resolve()
    for module_name in tuple(sys.modules):
        if (
            module_name == "cftuv_envelope"
            or module_name.startswith("cftuv_envelope.")
        ):
            del sys.modules[module_name]
    value = str(source_root / "kernel" / "src")
    if value in sys.path:
        sys.path.remove(value)
    sys.path.insert(0, value)
    import cftuv_envelope as kernel

    return kernel


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_json(payload: Any) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _value(value: Any) -> Any:
    return value.value if hasattr(value, "value") else value


def _fraction(value: Fraction | None) -> str | None:
    return None if value is None else str(value)


def _diagnostic(diagnostic) -> dict[str, Any]:
    return {
        "outcome": _value(diagnostic.outcome),
        "severity": _value(diagnostic.severity),
        "message": diagnostic.message,
        "envelope_spec_id": diagnostic.envelope_spec_id,
        "front_component_id": diagnostic.front_component_id,
    }


def _point(point) -> list[str]:
    return [str(point.x), str(point.y)]


def _segment(segment) -> dict[str, Any]:
    return {
        "chain_use_id": segment.chain_use_id.value,
        "physical_edge_id": segment.physical_edge_id.value,
        "source_vertex_start_id": segment.source_vertex_start_id.value,
        "source_vertex_end_id": segment.source_vertex_end_id.value,
        "start": _point(segment.start),
        "end": _point(segment.end),
        "tangent": _point(segment.tangent),
    }


def _geometry_failure_trace():
    capture: dict[str, Any] = {}

    def trace(frame, event, arg):
        if event == "call":
            return (
                trace
                if frame.f_code.co_name == "_validate_segment_corners"
                else None
            )
        if event != "exception" or capture:
            return trace
        exception_type, exception, _ = arg
        outcome = getattr(getattr(exception, "outcome", None), "value", None)
        if (
            exception_type.__name__ != "ReferenceGeometryError"
            or outcome != "PLANAR_CHAIN_SUPPORT_NOT_LINEAR"
        ):
            return trace
        payload: dict[str, Any] = {
            "function": frame.f_code.co_name,
            "outcome": outcome,
            "message": str(exception),
        }
        if frame.f_code.co_name == "_validate_segment_corners":
            incoming = frame.f_locals.get("incoming")
            outgoing = frame.f_locals.get("outgoing")
            context = frame.f_locals.get("self")
            chain_use_id = frame.f_locals.get("chain_use_id")
            declared = frame.f_locals.get("declared_corner_vertices", ())
            if incoming is not None and outgoing is not None and context is not None:
                payload.update(
                    {
                        "chain_use_id": chain_use_id.value,
                        "incoming": _segment(incoming),
                        "outgoing": _segment(outgoing),
                        "shared_source_vertex_id": (
                            incoming.source_vertex_end_id.value
                        ),
                        "oriented_cross": str(
                            context.metric.oriented_cross(
                                incoming.tangent, outgoing.tangent
                            )
                        ),
                        "declared_corner_vertex_ids": sorted(
                            item.value for item in declared
                        ),
                    }
                )
        capture.update(payload)
        return trace

    return trace, capture


def _binding_trace(kernel, compilation) -> dict[str, Any]:
    binding = getattr(compilation, "evaluation_geometry_binding", None)
    domain_id = compilation.plan_key.patch_domain_id
    frame = next(
        item
        for item in compilation.analysis_snapshot.surface_metric_descriptors
        if item.patch_domain_id == domain_id
    )
    if binding is None:
        return {
            "binding_present": False,
            "lattice_scale": None,
            "validation_issues": [],
            "bound_coordinate_collisions": [],
            "zero_length_bound_chain_edges": [],
            "source_collinear_bound_non_linear_triples": [],
        }
    from cftuv_envelope.validation import validate_evaluation_geometry_binding

    source_by_id = {
        item.source_vertex_id: [
            str(
                Fraction(
                    item.domain_coordinate.x.numerator,
                    item.domain_coordinate.x.denominator,
                )
            ),
            str(
                Fraction(
                    item.domain_coordinate.y.numerator,
                    item.domain_coordinate.y.denominator,
                )
            ),
        ]
        for item in frame.exact_source_vertex_coordinates
    }
    bound_groups: dict[tuple[Fraction, Fraction], list] = {}
    for item in binding.source_vertex_coordinates:
        coordinate = (
            Fraction(
                item.domain_coordinate.x.numerator,
                item.domain_coordinate.x.denominator,
            ),
            Fraction(
                item.domain_coordinate.y.numerator,
                item.domain_coordinate.y.denominator,
            ),
        )
        bound_groups.setdefault(coordinate, []).append(item.source_vertex_id)
    collisions = []
    for coordinate, vertex_ids in sorted(
        bound_groups.items(),
        key=lambda item: (item[0][0], item[0][1]),
    ):
        if len(vertex_ids) < 2:
            continue
        ordered_ids = sorted(vertex_ids, key=lambda item: item.value)
        collisions.append(
            {
                "bound_coordinate": [str(coordinate[0]), str(coordinate[1])],
                "source_vertices": [
                    {
                        "source_vertex_id": item.value,
                        "exact_source_coordinate": source_by_id[item],
                    }
                    for item in ordered_ids
                ],
            }
        )
    bound_by_id = {
        item.source_vertex_id: coordinate
        for coordinate, vertex_ids in bound_groups.items()
        for item in binding.source_vertex_coordinates
        if item.source_vertex_id in vertex_ids
    }

    def cross(a, b, c):
        incoming = (b[0] - a[0], b[1] - a[1])
        outgoing = (c[0] - b[0], c[1] - b[1])
        return incoming[0] * outgoing[1] - incoming[1] * outgoing[0]

    zero_length_edges = []
    bent_triples = []
    for chain in sorted(
        compilation.analysis_snapshot.physical_chains,
        key=lambda item: item.physical_chain_id.value,
    ):
        vertices = chain.ordered_source_vertex_ids
        pairs = tuple(zip(vertices, vertices[1:]))
        triples = tuple(zip(vertices, vertices[1:], vertices[2:]))
        if chain.is_closed and len(vertices) > 1:
            pairs += ((vertices[-1], vertices[0]),)
        if chain.is_closed and len(vertices) > 2:
            triples += (
                (vertices[-2], vertices[-1], vertices[0]),
                (vertices[-1], vertices[0], vertices[1]),
            )
        for start, end in pairs:
            if bound_by_id[start] == bound_by_id[end]:
                zero_length_edges.append(
                    {
                        "physical_chain_id": chain.physical_chain_id.value,
                        "source_vertex_ids": [start.value, end.value],
                        "bound_coordinate": [
                            str(value) for value in bound_by_id[start]
                        ],
                    }
                )
        for start, middle, end in triples:
            source = tuple(
                tuple(
                    Fraction(value)
                    for value in source_by_id[vertex_id]
                )
                for vertex_id in (start, middle, end)
            )
            bound = tuple(
                bound_by_id[vertex_id]
                for vertex_id in (start, middle, end)
            )
            source_cross = cross(*source)
            bound_cross = cross(*bound)
            if source_cross == 0 and bound_cross != 0:
                bent_triples.append(
                    {
                        "physical_chain_id": chain.physical_chain_id.value,
                        "source_vertex_ids": [
                            start.value,
                            middle.value,
                            end.value,
                        ],
                        "source_oriented_cross": str(source_cross),
                        "bound_oriented_cross": str(bound_cross),
                        "source_coordinates": [
                            [str(value) for value in item] for item in source
                        ],
                        "bound_coordinates": [
                            [str(value) for value in item] for item in bound
                        ],
                    }
                )
    return {
        "binding_present": True,
        "lattice_scale": binding.lattice_scale,
        "validation_issues": [
            str(item) for item in validate_evaluation_geometry_binding(binding)
        ],
        "bound_coordinate_collisions": collisions,
        "zero_length_bound_chain_edges": zero_length_edges,
        "source_collinear_bound_non_linear_triples": bent_triples,
    }


def _add_source_geometry_cross(compilation, failure: dict[str, Any]) -> None:
    if (
        not failure
        or failure.get("function") != "_validate_segment_corners"
    ):
        return
    from cftuv_envelope.reference.metric import ExactPlanarMetric
    from cftuv_envelope.reference.planar_types import (
        ExactPlanarPoint,
        point_sub,
    )

    domain_id = compilation.plan_key.patch_domain_id
    frame = next(
        item
        for item in compilation.analysis_snapshot.surface_metric_descriptors
        if item.patch_domain_id == domain_id
    )
    source_points = {
        item.source_vertex_id.value: ExactPlanarPoint.from_values(
            Fraction(
                item.domain_coordinate.x.numerator,
                item.domain_coordinate.x.denominator,
            ),
            Fraction(
                item.domain_coordinate.y.numerator,
                item.domain_coordinate.y.denominator,
            ),
        )
        for item in frame.exact_source_vertex_coordinates
    }
    start_id = failure["incoming"]["source_vertex_start_id"]
    shared_id = failure["shared_source_vertex_id"]
    end_id = failure["outgoing"]["source_vertex_end_id"]
    incoming = point_sub(source_points[shared_id], source_points[start_id])
    outgoing = point_sub(source_points[end_id], source_points[shared_id])
    failure["source_geometry_oriented_cross"] = str(
        ExactPlanarMetric.from_descriptor(frame).oriented_cross(
            incoming, outgoing
        )
    )
    failure["source_geometry_vertex_coordinates"] = {
        vertex_id: _point(source_points[vertex_id])
        for vertex_id in (start_id, shared_id, end_id)
    }


def _region_trace(region) -> dict[str, Any]:
    bridge = region.bridge
    skeleton = region.skeleton
    partition = region.partition
    return {
        "region_id": region.region_id,
        "bridge": {
            "outcome": _value(region.bridge_outcome),
            "findings": [_value(item) for item in bridge.findings],
            "edge_count": bridge.edge_count,
            "law_count": bridge.law_count,
            "matched_edge_count": bridge.matched_edge_count,
            "off_lattice_points": [
                [str(x), str(y)] for x, y in bridge.off_lattice_points
            ],
            "non_unit_speed_laws": [
                [name, str(value)] for name, value in bridge.non_unit_speed_laws
            ],
            "unmatched_laws": list(bridge.unmatched_laws),
            "surplus_laws": list(bridge.surplus_laws),
            "owner_by_edge_count": len(bridge.owner_by_edge),
            "ambiguous_owner_spans": [
                list(item) for item in bridge.ambiguous_owner_spans
            ],
            "wall_spans": [list(item) for item in bridge.wall_spans],
            "undetermined_source_count": bridge.undetermined_source_count,
            "lattice_scale": bridge.lattice_scale,
            "snap_residual": _fraction(bridge.snap_residual),
            "weighted_edge_count": bridge.weighted_edge_count,
            "fan_edge_count": bridge.fan_edge_count,
        },
        "skeleton": (
            None
            if skeleton is None
            else {
                "outcome": _value(skeleton.outcome),
                "node_count": len(skeleton.nodes),
                "levels": skeleton.levels,
                "counters": [[name, value] for name, value in skeleton.counters],
            }
        ),
        "faces": (
            None
            if partition is None
            else {
                "outcome": _value(partition.outcome),
                "face_count": len(partition.faces),
            }
        ),
    }


def _evaluation(kernel, snapshot, request) -> dict[str, Any]:
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id
    compiled = kernel.compile_reference_envelopes(
        snapshot, request, domain_id
    )
    from cftuv_envelope.wavefront import prepare_conveyor

    trace_function, failure_capture = _geometry_failure_trace()
    previous_trace = sys.gettrace()
    sys.settrace(trace_function)
    try:
        prepared = prepare_conveyor(
            snapshot, request, patch_domain_id=domain_id
        )
    finally:
        sys.settrace(previous_trace)
    binding = (
        None
        if compiled.compilation is None
        else _binding_trace(kernel, compiled.compilation)
    )
    if compiled.compilation is not None:
        _add_source_geometry_cross(
            compiled.compilation, failure_capture
        )
    trace = {
        "compile": {
            "outcome": _value(compiled.outcome),
            "compilation_available": compiled.compilation is not None,
            "diagnostics": [_diagnostic(item) for item in compiled.diagnostics],
            "envelope_spec_count": (
                None
                if compiled.compilation is None
                else len(compiled.compilation.envelope_specs)
            ),
            "evaluation_geometry_binding": binding,
        },
        "prepare": {
            "outcome": _value(prepared.outcome),
            "detail": prepared.detail,
            "lattice_scale": (
                None if prepared.lattice is None else prepared.lattice.scale
            ),
            "law_names": list(prepared.law_names),
            "counters": [[name, value] for name, value in prepared.counters],
            "regions": [_region_trace(item) for item in prepared.regions],
            "first_geometry_failure": failure_capture or None,
        },
    }
    trace["trace_sha256"] = _sha256(_canonical_json(trace))
    return trace


def _verify_fixture(kernel, revision: str, fixture_dir: Path) -> dict[str, Any]:
    manifest_path = fixture_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest["fixture_id"] != fixture_dir.name:
        raise RuntimeError(
            f"{fixture_dir}: fixture_id does not match directory name"
        )
    actual_hashes = {
        name: _sha256((fixture_dir / name).read_bytes())
        for name in sorted(manifest["files"])
    }
    if actual_hashes != manifest["files"]:
        raise RuntimeError(
            f"{fixture_dir}: file hashes differ: {actual_hashes!r}"
        )
    fixture_hash = _sha256(
        _canonical_json(
            [[name, actual_hashes[name]] for name in sorted(actual_hashes)]
        )
    )
    if fixture_hash != manifest["fixture_hash"]:
        raise RuntimeError(
            f"{fixture_dir}: fixture hash differs: {fixture_hash}"
        )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (fixture_dir / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (fixture_dir / "decal_request.json").read_bytes()
    )
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
    if any(issues.values()):
        raise RuntimeError(f"{fixture_dir}: contract validation failed: {issues}")
    evaluation = _evaluation(kernel, snapshot, request)
    expected = manifest.get("expected_kernel_results", {}).get(revision)
    return {
        "fixture_id": manifest["fixture_id"],
        "fixture_hash": fixture_hash,
        "validation_issues": issues,
        "evaluation": evaluation,
        "expected_result_recorded": expected is not None,
        "expected_result_matches": expected == evaluation if expected else None,
    }


def main() -> None:
    arguments = _arguments()
    kernel = _activate_source(arguments.source_root)
    revision = _git_sha(arguments.source_root)
    reports = [
        _verify_fixture(kernel, revision, fixture_dir)
        for fixture_dir in sorted(arguments.fixture_root.iterdir())
        if fixture_dir.is_dir()
    ]
    payload = {
        "schema": "cftuv.envelope.sem_clb_02_verification.v1",
        "kernel_revision": revision,
        "fixture_count": len(reports),
        "fixtures": reports,
    }
    if arguments.output_report:
        arguments.output_report.parent.mkdir(parents=True, exist_ok=True)
        arguments.output_report.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
            newline="\n",
        )
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), flush=True)
    if any(
        item["expected_result_recorded"]
        and not item["expected_result_matches"]
        for item in reports
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
