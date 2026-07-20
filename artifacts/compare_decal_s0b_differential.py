"""Compare S0b before/after receipts while ignoring provenance fields."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def _semantic_payload(report):
    return {
        "schema": report["schema"],
        "suite": report["suite"],
        "blender_version": report["blender_version"],
        "python_version": report["python_version"],
        "blend_sha256": report["blend_sha256"],
        "objects": report["objects"],
        "missing_objects": report["missing_objects"],
        "blend_saved": report["blend_saved"],
    }


def main():
    args = _arguments()
    before = json.loads(args.before.read_text(encoding="utf-8"))
    after = json.loads(args.after.read_text(encoding="utf-8"))
    before_payload = _semantic_payload(before)
    after_payload = _semantic_payload(after)
    equal = before_payload == after_payload
    report = {
        "schema": "cftuv.decal_s0b_comparison.v1",
        "suite": before["suite"],
        "before_commit": before["source_commit"],
        "after_commit": after["source_commit"],
        "semantic_equal": equal,
        "object_count": len(before_payload["objects"]),
        "all_preview_confirm_equal": all(
            width["preview_confirm_equal"]
            for record in after_payload["objects"]
            for width in record["widths"]
        ),
    }
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    if not equal:
        raise SystemExit("S0b strict-SEAMS differential mismatch")


if __name__ == "__main__":
    main()
