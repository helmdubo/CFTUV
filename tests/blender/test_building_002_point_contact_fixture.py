"""Blender host round-trip equality gate for the portable FIX-00 fixture."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def _arguments() -> argparse.Namespace:
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-dir", type=Path, required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-report", type=Path)
    return parser.parse_args(raw)


def _activate_source(source_root: Path):
    for module_name in tuple(sys.modules):
        if (
            module_name == "cftuv"
            or module_name.startswith("cftuv.")
            or module_name == "cftuv_envelope"
            or module_name.startswith("cftuv_envelope.")
        ):
            del sys.modules[module_name]
    for entry in (source_root / "kernel" / "src", source_root):
        value = str(entry.resolve())
        if value in sys.path:
            sys.path.remove(value)
        sys.path.insert(0, value)
    import cftuv_envelope as kernel

    return kernel


def _mesh_fingerprint(obj) -> str:
    payload = {
        "vertices": [
            (vertex.index, tuple(float(value) for value in vertex.co))
            for vertex in obj.data.vertices
        ],
        "edges": [
            (
                edge.index,
                tuple(int(value) for value in edge.vertices),
                bool(edge.use_seam),
            )
            for edge in obj.data.edges
        ],
        "faces": [
            (polygon.index, tuple(int(value) for value in polygon.vertices))
            for polygon in obj.data.polygons
        ],
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def main() -> None:
    import bmesh

    arguments = _arguments()
    fixture_dir = arguments.fixture_dir.resolve()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from generate_building_002_point_contact_fixture import generate

    obj, _, generated_fingerprint = generate(fixture_dir / "host_mesh.json")
    kernel = _activate_source(arguments.source_root)
    from cftuv.analysis import build_analysis_bundle
    from cftuv.envelope_request_export import (
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )

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
        included_patch_ids=frozenset({0}),
    )
    request = build_envelope_decal_request(
        snapshot,
        frozenset({2, 3, 7}),
        0.25,
    )
    actual_snapshot = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    actual_request = kernel.DecalRequestCodecV1.dumps(request)
    expected_snapshot = (fixture_dir / "analysis_snapshot.json").read_bytes()
    expected_request = (fixture_dir / "decal_request.json").read_bytes()
    after = _mesh_fingerprint(obj)
    report = {
        "schema": "cftuv.envelope.host_export_equality.v1",
        "accepted_exporter_revision": subprocess.check_output(
            [
                "git",
                "-C",
                str(arguments.source_root.resolve()),
                "rev-parse",
                "HEAD",
            ],
            text=True,
        ).strip(),
        "generated_mesh_fingerprint": generated_fingerprint,
        "analysis_mesh_fingerprint_before": before,
        "analysis_mesh_fingerprint_after": after,
        "source_mesh_unchanged": before == after,
        "analysis_snapshot_byte_equal": actual_snapshot == expected_snapshot,
        "decal_request_byte_equal": actual_request == expected_request,
        "analysis_snapshot_sha256": hashlib.sha256(actual_snapshot).hexdigest(),
        "decal_request_sha256": hashlib.sha256(actual_request).hexdigest(),
    }
    if not all(
        (
            report["source_mesh_unchanged"],
            report["analysis_snapshot_byte_equal"],
            report["decal_request_byte_equal"],
        )
    ):
        raise RuntimeError(f"host fixture round-trip mismatch: {report!r}")
    if arguments.output_report:
        arguments.output_report.parent.mkdir(parents=True, exist_ok=True)
        arguments.output_report.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
