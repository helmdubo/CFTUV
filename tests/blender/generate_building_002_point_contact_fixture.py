"""Deterministically rebuild the minimal FIX-00 host mesh in a blank scene."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


def _arguments() -> argparse.Namespace:
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-dir", type=Path, required=True)
    parser.add_argument("--save-blend", type=Path)
    return parser.parse_args(raw)


def _fingerprint(payload: dict) -> str:
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
    encoded = json.dumps(
        normalized, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def generate(host_mesh_path: Path):
    import bpy

    payload = json.loads(host_mesh_path.read_text(encoding="utf-8"))
    if payload["schema"] != "cftuv.envelope.host_mesh_fixture.v1":
        raise ValueError("unsupported host mesh fixture schema")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    vertices = [
        tuple(float.fromhex(value) for value in item["coordinate_binary64_hex"])
        for item in payload["vertices"]
    ]
    edges = [tuple(item["vertex_indices"]) for item in payload["edges"]]
    faces = [tuple(item["vertex_cycle"]) for item in payload["faces"]]
    mesh = bpy.data.meshes.new(f"{payload['object_name']}_fixture_mesh")
    mesh.from_pydata(vertices, edges, faces)
    mesh.update(calc_edges=True)
    obj = bpy.data.objects.new(payload["object_name"], mesh)
    bpy.context.collection.objects.link(obj)
    for expected, edge in zip(payload["edges"], mesh.edges, strict=True):
        if int(edge.index) != int(expected["index"]):
            raise RuntimeError("deterministic edge indexing was not preserved")
        if tuple(sorted(edge.vertices)) != tuple(
            sorted(expected["vertex_indices"])
        ):
            raise RuntimeError("deterministic edge endpoints were not preserved")
        edge.use_seam = bool(expected["use_seam"])
    if [int(item.index) for item in mesh.vertices] != [
        int(item["index"]) for item in payload["vertices"]
    ]:
        raise RuntimeError("deterministic vertex indexing was not preserved")
    if [int(item.index) for item in mesh.polygons] != [
        int(item["index"]) for item in payload["faces"]
    ]:
        raise RuntimeError("deterministic face indexing was not preserved")
    return obj, payload, _fingerprint(payload)


def main() -> None:
    import bpy

    arguments = _arguments()
    fixture_dir = arguments.fixture_dir.resolve()
    obj, payload, fingerprint = generate(fixture_dir / "host_mesh.json")
    manifest = json.loads(
        (fixture_dir / "manifest.json").read_text(encoding="utf-8")
    )
    if fingerprint != manifest["source_mesh_fingerprint"]:
        raise RuntimeError("generated host mesh fingerprint differs from manifest")
    if arguments.save_blend:
        arguments.save_blend.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(arguments.save_blend.resolve()))
    print(
        json.dumps(
            {
                "schema": "cftuv.envelope.host_mesh_generation_report.v1",
                "object_name": obj.name,
                "mesh_counts": {
                    "vertices": len(payload["vertices"]),
                    "edges": len(payload["edges"]),
                    "faces": len(payload["faces"]),
                },
                "source_mesh_fingerprint": fingerprint,
                "saved_blend": (
                    str(arguments.save_blend.resolve())
                    if arguments.save_blend
                    else None
                ),
            },
            sort_keys=True,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
