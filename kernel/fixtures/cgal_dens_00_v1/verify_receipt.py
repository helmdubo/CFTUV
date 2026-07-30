"""Fail-closed verifier for the CGAL-DENS-00 semantic STOP receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any


BASE = "c1d6cd5a92d607bd438ccc25561f0bfa6ab5150e"
PRIMARY_STOP = (
    "CGAL_INPUT_CANNOT_REPRESENT_ZERO_LENGTH_MOVING_VERTEX_FAN_SOURCES"
)
EXPECTED_TREES = {
    "kernel/src": "9750a6a0b07691016639db53aa3a0ce869e0d7f5",
    "cftuv": "154eb59e341285d64d37db5cd4dbc5d7868d5c2a",
    "tools": "ec5439a5fc8ea46784d4621ac763f941b4833c36",
}
EXPECTED_PRODUCT_FILES = {
    "kernel/src/cftuv_envelope/wavefront/polygon.py": (
        "9fc7f6c631442a528faf8b465ddb96e1a79dc4c43922cf46da7d2ee36ca76d69"
    ),
    "kernel/src/cftuv_envelope/wavefront/bridge.py": (
        "9cba14947aa23e4285111e512586392abd6314c239d46aad5efd0bcc337006d5"
    ),
    "kernel/src/cftuv_envelope/reference/compile.py": (
        "af010d4d8387e7ed360c7c22327ce0ed67c4abb15351d40ff25beee2bc3dc4c0"
    ),
}
EXPECTED_FANS = {
    0: (4, (1, 1, 1, 1)),
    1: (5, (2, 1, 1, 1)),
    4: (11, (4, 3, 2, 2)),
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_exact_dict(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if type(payload) is not dict:
        raise AssertionError(f"{path.name}: top level must be an exact dict")
    return payload


def _expect_keys(payload: dict[str, Any], expected: set[str], label: str) -> None:
    actual = set(payload)
    if actual != expected:
        raise AssertionError(
            f"{label}: keyset mismatch missing={sorted(expected - actual)} "
            f"extra={sorted(actual - expected)}"
        )


def _git(repo: Path, *arguments: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo), *arguments],
        text=True,
    ).strip()


def _verify_product_identity(repo: Path) -> None:
    if _git(repo, "merge-base", "--is-ancestor", BASE, "HEAD") != "":
        raise AssertionError("unexpected merge-base output")
    for path, expected in EXPECTED_TREES.items():
        actual = _git(repo, "rev-parse", f"HEAD:{path}")
        if actual != expected:
            raise AssertionError(
                f"unknown product tree {path}: {actual} != {expected}"
            )
    for relative, expected in EXPECTED_PRODUCT_FILES.items():
        path = repo / relative
        actual = _sha256(path)
        if actual != expected:
            raise AssertionError(
                f"unknown product file {relative}: {actual} != {expected}"
            )
    polygon = (
        repo / "kernel/src/cftuv_envelope/wavefront/polygon.py"
    ).read_text(encoding="utf-8")
    bridge = (
        repo / "kernel/src/cftuv_envelope/wavefront/bridge.py"
    ).read_text(encoding="utf-8")
    for fragment in (
        "k` рёбер нулевой длины подряд",
        "Рёбра `edges()` веера НЕ содержат",
        "def fan_edges(",
    ):
        if fragment not in polygon:
            raise AssertionError(f"missing product fan invariant: {fragment}")
    if "идущие СВОИМ каналом" not in bridge:
        raise AssertionError("bridge no longer declares a separate fan channel")


def _verify_source(root: Path, source: dict[str, Any]) -> None:
    _expect_keys(
        source,
        {"schema", "base_revision", "source", "density_profiles", "files"},
        "source snapshot",
    )
    if source["base_revision"] != BASE:
        raise AssertionError("source snapshot has the wrong base revision")
    identity = source["source"]
    if (
        identity["object_name"] != "building.002"
        or identity["patch_id"] != 0
        or identity["patch_domain_id"]
        != "host-v0:patch-domain:7c6ca95fa5927015493b646a"
        or identity["blend_sha256"]
        != "65948c37cc25cc1bf934be779bfa02ca48f2ddcdf43aa3b5575cea30077c0697"
        or identity["mesh_unchanged"] is not True
        or identity["mesh_fingerprint_before"]
        != identity["mesh_fingerprint_after"]
    ):
        raise AssertionError("source identity or unchanged-mesh proof drifted")
    source_dir = root / "source"
    for name, expected in source["files"].items():
        if type(name) is not str or type(expected) is not str:
            raise AssertionError("source file digest record is not exact")
        actual = _sha256(source_dir / name)
        if actual != expected:
            raise AssertionError(f"source file hash mismatch: {name}")

    profiles = source["density_profiles"]
    if type(profiles) is not list or len(profiles) != 3:
        raise AssertionError("density profile count mismatch")
    identities = None
    for profile in profiles:
        density = profile["density"]
        if density not in EXPECTED_FANS:
            raise AssertionError(f"unexpected density: {density!r}")
        expected_total, expected_per_relation = EXPECTED_FANS[density]
        rows = profile["relations"]
        actual_per_relation = tuple(
            row["hidden_support_count"] for row in rows
        )
        if (
            profile["fan_source_count"] != expected_total
            or profile["reflex_vertex_count"] != 4
            or actual_per_relation != expected_per_relation
        ):
            raise AssertionError(
                f"density d{density} fan counts drifted: "
                f"{profile['fan_source_count']}, {actual_per_relation}"
            )
        current_identities = tuple(
            (
                row["corner_relation_id"],
                row["owner_sector_id"],
                row["source_vertex_id"],
            )
            for row in rows
        )
        if identities is None:
            identities = current_identities
        elif current_identities != identities:
            raise AssertionError("fan relation identity changed across density")
        if any(
            row["zero_length_at_alpha_zero"] is not True
            or row["separate_initial_front_channel"] is not True
            for row in rows
        ):
            raise AssertionError("fan source semantics were weakened")


def _verify_reference(reference: dict[str, Any]) -> None:
    if (
        reference["svcgal"]["commit"]
        != "6e57d533790693e90062d121e4a2c586fb48becb"
        or reference["svcgal"]["commit_tree"]
        != "6aab87a62532c361f81d8fa5e3df95f07de767f6"
    ):
        raise AssertionError("SVCGAL reference identity drifted")
    license_evidence = reference["svcgal"]["license_evidence"]
    if (
        license_evidence["ctSVCGAL_or_repository_license_file"] != "ABSENT"
        or license_evidence["github_detected_license"] is not None
        or license_evidence["pySVCGAL_license_content"] != "GPL3"
        or license_evidence["production_clearance"] != "NOT_ESTABLISHED"
    ):
        raise AssertionError("SVCGAL license evidence was weakened")
    contract = reference["official_cgal"]["input_contract"]
    if (
        contract["general_planar_directed_graph_supported"] is not False
        or contract["polygon_with_holes_required"] is not True
        or contract["weight_domain"]
        != "POSITIVE_MULTIPLICATIVE_CONTOUR_EDGE_WEIGHT"
    ):
        raise AssertionError("official CGAL input contract drifted")
    if (
        reference["official_cgal"]["license"]["package_license"] != "GPL"
        or reference["official_cgal"]["license"]["production_clearance"]
        != "NOT_ESTABLISHED"
    ):
        raise AssertionError("CGAL production license gate was weakened")


def _verify_manifest(root: Path, manifest: dict[str, Any]) -> None:
    if (
        manifest["base_revision"] != BASE
        or manifest["expected_result"]["verdict"] != "STOP"
        or manifest["expected_result"]["primary_stop"] != PRIMARY_STOP
        or manifest["dependency_reachability"]["engine_invocation_count"] != 0
        or manifest["dependency_reachability"]["build_status"]
        != "NOT_RUN_AFTER_SEMANTIC_STOP"
    ):
        raise AssertionError("manifest gate was weakened")
    for relative, expected in manifest["files"].items():
        path = root / relative
        if path.suffix == ".py":
            actual = hashlib.sha256(
                path.read_bytes().replace(b"\r\n", b"\n")
            ).hexdigest()
        else:
            actual = _sha256(path)
        if actual != expected:
            raise AssertionError(
                f"manifest file hash mismatch {relative}: {actual} != {expected}"
            )


def _verify_receipt(
    receipt: dict[str, Any],
    source_path: Path,
    reference_path: Path,
) -> None:
    _expect_keys(
        receipt,
        {
            "schema",
            "verdict",
            "primary_stop",
            "secondary_stop",
            "source_identity",
            "reference_identity",
            "semantic_mapping",
            "density_results",
            "stages",
            "role_estimate",
        },
        "mapping receipt",
    )
    if receipt["verdict"] != "STOP":
        raise AssertionError("semantic incompatibility must be STOP")
    if receipt["primary_stop"]["code"] != PRIMARY_STOP:
        raise AssertionError("wrong primary STOP")
    if receipt["primary_stop"]["forbidden_transform_not_attempted"] is not True:
        raise AssertionError("forbidden topology transform was not refused")
    if (
        receipt["source_identity"]["source_snapshot_sha256"]
        != _sha256(source_path)
        or receipt["reference_identity"]["reference_contract_sha256"]
        != _sha256(reference_path)
    ):
        raise AssertionError("receipt is not bound to its inputs")
    stages = receipt["stages"]
    expected_stages = {
        "dependency_checkout": "REFERENCE_COMMIT_IDENTIFIED",
        "dependency_build": "NOT_RUN_AFTER_SEMANTIC_STOP",
        "engine_invocation_count": 0,
        "conversion_runtime_ms": None,
        "skeleton_runtime_ms": None,
        "topology_projection": "NOT_RUN_AFTER_SEMANTIC_STOP",
        "event_projection": "NOT_RUN_AFTER_SEMANTIC_STOP",
        "owner_projection": "NOT_RUN_AFTER_SEMANTIC_STOP",
        "miter_assertion": "NOT_CLAIMED",
    }
    if stages != expected_stages:
        raise AssertionError(
            "receipt claims an engine/runtime/projection result after STOP"
        )
    if [row["fan_source_count"] for row in receipt["density_results"]] != [
        4,
        5,
        11,
    ]:
        raise AssertionError("receipt fan counts drifted")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent,
    )
    arguments = parser.parse_args()
    root = arguments.root.resolve()
    repo = root.parents[2]
    source_path = root / "source/source_snapshot.json"
    reference_path = root / "reference_contract.json"
    run1_path = root / "receipt_run_1.json"
    run2_path = root / "receipt_run_2.json"

    _verify_product_identity(repo)
    manifest = _load_exact_dict(root / "manifest.json")
    _verify_manifest(root, manifest)
    source = _load_exact_dict(source_path)
    reference = _load_exact_dict(reference_path)
    _verify_source(root, source)
    _verify_reference(reference)
    run1 = _load_exact_dict(run1_path)
    run2 = _load_exact_dict(run2_path)
    _verify_receipt(run1, source_path, reference_path)
    _verify_receipt(run2, source_path, reference_path)
    if run1_path.read_bytes() != run2_path.read_bytes():
        raise AssertionError("tracked fresh-run receipts are not bit-identical")

    analyzer = root / "analyze_mapping.py"
    with tempfile.TemporaryDirectory(prefix="cgal-dens-00-") as temporary:
        generated_paths = [
            Path(temporary) / "fresh_1.json",
            Path(temporary) / "fresh_2.json",
        ]
        for output in generated_paths:
            subprocess.run(
                [
                    sys.executable,
                    str(analyzer),
                    "--source",
                    str(source_path),
                    "--reference",
                    str(reference_path),
                    "--output",
                    str(output),
                ],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        if generated_paths[0].read_bytes() != generated_paths[1].read_bytes():
            raise AssertionError("two fresh analyzer runs are not deterministic")
        if generated_paths[0].read_bytes() != run1_path.read_bytes():
            raise AssertionError("fresh analyzer output differs from tracked receipt")

    print(
        "CGAL_DENS_00_VERIFY PASS "
        "verdict=STOP primary="
        f"{PRIMARY_STOP} fan_counts=d0:4,d1:5,d4:11 "
        "engine_invocations=0 deterministic_runs=2 "
        "license_clearance=NOT_ESTABLISHED"
    )


if __name__ == "__main__":
    main()
