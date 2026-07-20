"""S0b strict-SEAMS semantic receipt for one unsaved Blender scene.

Run the same command before and after removing archived direct builders.  The
paired reports are compared by ``compare_decal_s0b_differential.py``.
"""

from __future__ import annotations

import argparse
from dataclasses import fields, is_dataclass, replace
from enum import Enum
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import bpy


REPO_ROOT = Path(__file__).resolve().parents[1]
for path in (REPO_ROOT, REPO_ROOT / "artifacts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

import verify_decal_r13_rr9a as field  # noqa: E402
from cftuv.decal_voronoi import serialize_network_faces  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import DecalSettings  # noqa: E402


def _arguments():
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", required=True, choices=("before", "after"))
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--suite", choices=("field", "verification"), default="field"
    )
    return parser.parse_args(raw)


def _json_bytes(value):
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _sha256(value):
    return hashlib.sha256(_json_bytes(value)).hexdigest()


def _canonical(value, *, digits=10):
    """Convert immutable compile IR into stable, JSON-ready semantics."""

    if is_dataclass(value) and not isinstance(value, type):
        return {
            "type": type(value).__name__,
            "fields": {
                item.name: _canonical(getattr(value, item.name), digits=digits)
                for item in fields(value)
            },
        }
    if isinstance(value, Enum):
        return _canonical(value.value, digits=digits)
    if isinstance(value, dict):
        items = [
            (
                _canonical(key, digits=digits),
                _canonical(item, digits=digits),
            )
            for key, item in value.items()
        ]
        items.sort(key=lambda pair: _json_bytes(pair[0]))
        return {"map": items}
    if isinstance(value, (tuple, list)):
        return [_canonical(item, digits=digits) for item in value]
    if isinstance(value, (set, frozenset)):
        items = [_canonical(item, digits=digits) for item in value]
        items.sort(key=_json_bytes)
        return {"set": items}
    if isinstance(value, float):
        if value == float("inf"):
            return "Infinity"
        if value == float("-inf"):
            return "-Infinity"
        if value != value:
            return "NaN"
        return round(value, digits)
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    try:
        return {
            "vector": [round(float(component), digits) for component in value]
        }
    except (TypeError, ValueError):
        raise TypeError(
            "Unsupported compiled-plan value: "
            f"{type(value).__module__}.{type(value).__qualname__}"
        ) from None


def _failure_counts(plan):
    counts = {}
    failures = (
        tuple(plan.compile_failures)
        + tuple(plan.rail_compile_failures)
        + tuple(plan.rail_geometry_failures)
        + tuple(plan.rejected_edges)
    )
    for failure in failures:
        reason = str(getattr(failure, "reason", "UNKNOWN"))
        counts[reason] = counts.get(reason, 0) + 1
    return dict(sorted(counts.items()))


def _plan_receipt(plan):
    canonical = _canonical(plan)
    return {
        "digest": _sha256(canonical),
        "backend_summary": plan.backend_summary,
        "selected_edge_indices": list(plan.selected_edge_indices),
        "accepted_rail_planar": list(plan.accepted_rail_planar_edge_indices),
        "accepted_patch_voronoi": list(
            plan.accepted_patch_voronoi_edge_indices
        ),
        "rejected_edge_indices": list(plan.rejected_edge_indices),
        "accounting_is_exact": bool(plan.accounting_is_exact),
        "fallback_counters": _failure_counts(plan),
    }


def _evaluation_receipt(result):
    serialization = serialize_network_faces(result.faces)
    return {
        "geometry_batch_serialization_digest": _sha256(serialization),
        "topology_signature": serialization["topology_signature"],
        "face_count": len(result.faces),
        "policy_counts": [list(item) for item in result.policy_counts],
    }, serialization


def _object_receipt(obj, requested_edges):
    graph, selected, _all_seam_edges = field._source_graph(
        obj, requested_edges
    )
    settings = DecalSettings(width_seam=0.8, offset=0.01)
    plan = compile_manual_seam_decal_plan(graph, settings, selected)
    record = {
        "object": obj.name,
        "plan": _plan_receipt(plan),
        "widths": [],
    }
    for width in (0.4, 0.8):
        width_settings = replace(settings, width_seam=width)
        preview = evaluate_manual_seam_faces(
            obj, width_settings, plan, preview=True
        )
        confirm = evaluate_manual_seam_faces(
            obj, width_settings, plan, preview=False
        )
        preview_receipt, preview_serialization = _evaluation_receipt(preview)
        confirm_receipt, confirm_serialization = _evaluation_receipt(confirm)
        record["widths"].append(
            {
                "width": width,
                "preview": preview_receipt,
                "confirm": confirm_receipt,
                "preview_confirm_equal": (
                    preview_serialization == confirm_serialization
                    and preview.policy_counts == confirm.policy_counts
                ),
            }
        )
    return record


def _suite_objects(suite):
    if suite == "verification":
        return {"VERIFY_SRC_MANUAL_SEAMS": "ALL_SEAMS"}
    return field.FIELD_OBJECTS


def _git_head():
    return subprocess.check_output(
        ("git", "rev-parse", "HEAD"), cwd=REPO_ROOT, text=True
    ).strip()


def main():
    args = _arguments()
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    blend_path = Path(bpy.data.filepath)
    report = {
        "schema": "cftuv.decal_s0b_differential.v1",
        "phase": args.phase,
        "suite": args.suite,
        "source_commit": _git_head(),
        "blender_version": bpy.app.version_string,
        "python_version": sys.version.split()[0],
        "blend_file": str(blend_path),
        "blend_sha256": hashlib.sha256(blend_path.read_bytes()).hexdigest(),
        "objects": [],
        "missing_objects": [],
        "blend_saved": False,
    }
    for object_name, requested_edges in _suite_objects(args.suite).items():
        obj = bpy.data.objects.get(object_name)
        if obj is None or obj.type != "MESH":
            report["missing_objects"].append(object_name)
            continue
        report["objects"].append(_object_receipt(obj, requested_edges))

    output = Path(args.output)
    if not output.is_absolute():
        output = REPO_ROOT / output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("CFTUV_S0B_RECEIPT=" + json.dumps(report, ensure_ascii=False))
    if report["missing_objects"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
