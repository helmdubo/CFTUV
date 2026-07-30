"""Produce the deterministic fail-closed CGAL-DENS-00 mapping receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


BASE = "c1d6cd5a92d607bd438ccc25561f0bfa6ab5150e"
PRIMARY_STOP = (
    "CGAL_INPUT_CANNOT_REPRESENT_ZERO_LENGTH_MOVING_VERTEX_FAN_SOURCES"
)
SCHEMA = "cftuv.proof.cgal_dens_00.mapping_receipt.v1"


def _canonical(payload: Any) -> bytes:
    return (
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_receipt(source_path: Path, reference_path: Path) -> dict[str, Any]:
    source = json.loads(source_path.read_text(encoding="utf-8"))
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    if source.get("base_revision") != BASE:
        raise RuntimeError("SOURCE_BASE_REVISION_MISMATCH")
    profiles = source.get("density_profiles")
    if type(profiles) is not list:
        raise RuntimeError("SOURCE_DENSITY_PROFILES_INVALID")
    expected_densities = [0, 1, 4]
    if [item.get("density") for item in profiles] != expected_densities:
        raise RuntimeError("SOURCE_DENSITY_SET_MISMATCH")
    if any(
        type(item.get("fan_source_count")) is not int
        or item["fan_source_count"] <= 0
        for item in profiles
    ):
        raise RuntimeError("SOURCE_HAS_NO_REQUIRED_FAN_SOURCES")

    density_results = []
    for item in profiles:
        density_results.append(
            {
                "density": item["density"],
                "fan_source_count": item["fan_source_count"],
                "reflex_vertex_count": item["reflex_vertex_count"],
                "relation_hidden_support_counts": [
                    {
                        "corner_relation_id": relation["corner_relation_id"],
                        "hidden_support_count": relation[
                            "hidden_support_count"
                        ],
                        "owner_sector_id": relation["owner_sector_id"],
                        "source_vertex_id": relation["source_vertex_id"],
                    }
                    for relation in item["relations"]
                ],
                "stage": "ENCODING_REFUSED",
            }
        )

    return {
        "schema": SCHEMA,
        "verdict": "STOP",
        "primary_stop": {
            "code": PRIMARY_STOP,
            "first_incompatible_invariant": (
                "CFTUV VertexFanV1 carries k consecutive zero-length moving "
                "initial-front edges at one contour vertex in a separate "
                "fan-source channel. CGAL Straight_skeleton_2 accepts contour "
                "cycles of a polygon-with-holes and has no general planar "
                "directed graph input. Turning a fan support into a contour "
                "edge would repeat the anchor point, create a zero-length "
                "contour edge, and change source/owner topology."
            ),
            "forbidden_transform_not_attempted": True,
        },
        "secondary_stop": {
            "code": "PRODUCTION_LICENSE_CLEARANCE_NOT_ESTABLISHED",
            "cgal_package_license": reference["official_cgal"]["license"][
                "package_license"
            ],
            "svcgal_repository_license": reference["svcgal"][
                "license_evidence"
            ]["ctSVCGAL_or_repository_license_file"],
            "svcgal_python_license_content": reference["svcgal"][
                "license_evidence"
            ]["pySVCGAL_license_content"],
        },
        "source_identity": {
            "base_revision": source["base_revision"],
            "blend_sha256": source["source"]["blend_sha256"],
            "mesh_fingerprint": source["source"][
                "mesh_fingerprint_before"
            ],
            "mesh_unchanged": source["source"]["mesh_unchanged"],
            "object_name": source["source"]["object_name"],
            "patch_domain_id": source["source"]["patch_domain_id"],
            "source_snapshot_sha256": _sha256(source_path),
        },
        "reference_identity": {
            "reference_contract_sha256": _sha256(reference_path),
            "svcgal_commit": reference["svcgal"]["commit"],
            "svcgal_commit_tree": reference["svcgal"]["commit_tree"],
        },
        "semantic_mapping": [
            {
                "cftuv_feature": "outer_contour",
                "cgal_status": "ENCODABLE_IF_WEAKLY_SIMPLE",
            },
            {
                "cftuv_feature": "holes",
                "cgal_status": "ENCODABLE_IF_DISJOINT_AND_ORIENTED",
            },
            {
                "cftuv_feature": "positive_weighted_contour_sources",
                "cgal_status": "ENCODABLE_BY_OFFICIAL_CGAL",
            },
            {
                "cftuv_feature": "stationary_wall_edges",
                "cgal_status": "NOT_ENCODABLE_BY_POSITIVE_WEIGHT_API",
            },
            {
                "cftuv_feature": "zero_length_moving_vertex_fan_sources",
                "cgal_status": "NOT_ENCODABLE_PRIMARY_STOP",
            },
            {
                "cftuv_feature": "fan_source_owner_identity",
                "cgal_status": "NOT_PROJECTABLE_AFTER_PRIMARY_STOP",
            },
        ],
        "density_results": density_results,
        "stages": {
            "dependency_checkout": "REFERENCE_COMMIT_IDENTIFIED",
            "dependency_build": "NOT_RUN_AFTER_SEMANTIC_STOP",
            "engine_invocation_count": 0,
            "conversion_runtime_ms": None,
            "skeleton_runtime_ms": None,
            "topology_projection": "NOT_RUN_AFTER_SEMANTIC_STOP",
            "event_projection": "NOT_RUN_AFTER_SEMANTIC_STOP",
            "owner_projection": "NOT_RUN_AFTER_SEMANTIC_STOP",
            "miter_assertion": "NOT_CLAIMED",
        },
        "role_estimate": {
            "production_backend": "NO",
            "preview_backend": "NO_FOR_ACCEPTED_DENSITY_SEMANTICS",
            "contour_only_diagnostic_oracle": (
                "POSSIBLE_ONLY_IF_FAN_SOURCES_AND_WALLS_ARE_EXPLICITLY "
                "OUT_OF_SCOPE; NOT AN ORACLE FOR CFTUV DENSITY"
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    payload = build_receipt(
        arguments.source.resolve(),
        arguments.reference.resolve(),
    )
    arguments.output.resolve().write_bytes(_canonical(payload))
    print(
        json.dumps(
            {
                "verdict": payload["verdict"],
                "primary_stop": payload["primary_stop"]["code"],
                "engine_invocation_count": payload["stages"][
                    "engine_invocation_count"
                ],
                "output_sha256": _sha256(arguments.output.resolve()),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
