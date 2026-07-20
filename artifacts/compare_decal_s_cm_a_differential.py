"""Compare the production MITER semantics around S-CM.a.

The I5 micro-slice intentionally enriches compiled plan provenance, so the raw
plan dataclass digest is reported separately and excluded from the behavioral
comparison. Material identity is expected to change from legacy ``KITE`` to
``MITER`` where CornerModel takes ownership; topology, policy counters,
routing/accounting and preview/confirm parity remain strict.
"""

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


def _plan_semantics(plan):
    return {key: value for key, value in plan.items() if key != "digest"}


def _normalized_topology(value):
    topology = value["topology_signature"]
    return {
        "vertex_count": topology["vertex_count"],
        "edge_count": topology["edge_count"],
        "face_count": topology["face_count"],
        "face_loops": sorted(topology["face_loops"]),
    }


def _width_semantics_equal(before_width, after_width):
    if before_width["width"] != after_width["width"]:
        return False
    for mode in ("preview", "confirm"):
        before_value = before_width[mode]
        after_value = after_width[mode]
        if _normalized_topology(before_value) != _normalized_topology(after_value):
            return False
        if before_value["policy_counts"] != after_value["policy_counts"]:
            return False
        if before_value["face_count"] != after_value["face_count"]:
            return False
    return True


def main():
    args = _arguments()
    before = json.loads(args.before.read_text(encoding="utf-8"))
    after = json.loads(args.after.read_text(encoding="utf-8"))
    before_objects = {item["object"]: item for item in before["objects"]}
    after_objects = {item["object"]: item for item in after["objects"]}
    object_names = sorted(set(before_objects) | set(after_objects))
    objects = []
    for name in object_names:
        before_item = before_objects.get(name)
        after_item = after_objects.get(name)
        if before_item is None or after_item is None:
            objects.append(
                {
                    "object": name,
                    "present_before": before_item is not None,
                    "present_after": after_item is not None,
                    "plan_digest_equal": False,
                    "plan_semantics_equal": False,
                    "topology_and_policy_equal": False,
                    "geometry_identity_digest_equal": False,
                    "preview_confirm_equal": False,
                }
            )
            continue
        objects.append(
            {
                "object": name,
                "present_before": True,
                "present_after": True,
                "plan_digest_equal": (
                    before_item["plan"]["digest"]
                    == after_item["plan"]["digest"]
                ),
                "plan_semantics_equal": (
                    _plan_semantics(before_item["plan"])
                    == _plan_semantics(after_item["plan"])
                ),
                "topology_and_policy_equal": (
                    len(before_item["widths"]) == len(after_item["widths"])
                    and all(
                        _width_semantics_equal(before_width, after_width)
                        for before_width, after_width in zip(
                            before_item["widths"], after_item["widths"]
                        )
                    )
                ),
                "geometry_identity_digest_equal": all(
                    before_width["preview"][
                        "geometry_batch_serialization_digest"
                    ]
                    == after_width["preview"][
                        "geometry_batch_serialization_digest"
                    ]
                    for before_width, after_width in zip(
                        before_item["widths"], after_item["widths"]
                    )
                ),
                "preview_confirm_equal": all(
                    width["preview_confirm_equal"]
                    for width in after_item["widths"]
                ),
            }
        )
    report = {
        "schema": "cftuv.decal_s_cm_a_comparison.v1",
        "before_commit": before["source_commit"],
        "after_commit": after["source_commit"],
        "blend_sha256_equal": before["blend_sha256"] == after["blend_sha256"],
        "plan_digest_changes_are_i5_provenance": True,
        "objects": objects,
        "plan_semantics_equal": all(
            item["plan_semantics_equal"] for item in objects
        ),
        "topology_and_policy_equal": all(
            item["topology_and_policy_equal"] for item in objects
        ),
        "geometry_identity_changed_objects": [
            item["object"]
            for item in objects
            if not item["geometry_identity_digest_equal"]
        ],
        "all_preview_confirm_equal": all(
            item["preview_confirm_equal"] for item in objects
        ),
    }
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    if not (
        report["blend_sha256_equal"]
        and report["plan_semantics_equal"]
        and report["topology_and_policy_equal"]
        and report["all_preview_confirm_equal"]
    ):
        raise SystemExit("S-CM.a MITER semantic differential mismatch")


if __name__ == "__main__":
    main()
