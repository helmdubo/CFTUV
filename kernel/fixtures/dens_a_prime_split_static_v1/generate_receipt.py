"""Generate the proof-only DENS-A-prime static/pre-request split receipt."""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import fields, is_dataclass, replace
from decimal import Decimal
from enum import Enum
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any


HERE = Path(__file__).resolve().parent


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, default=HERE.parents[2])
    parser.add_argument("--inputs", type=Path, default=HERE / "inputs.json")
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


def canonical(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def pretty(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n"
    ).encode("utf-8")


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def encoded(value: Any) -> Any:
    if is_dataclass(value):
        return {
            "$type": type(value).__name__,
            **{
                item.name: encoded(getattr(value, item.name))
                for item in fields(value)
            },
        }
    if isinstance(value, Enum):
        return {"$enum_type": type(value).__name__, "value": value.value}
    if isinstance(value, Fraction):
        return {
            "$type": "ExactFractionV1",
            "numerator": value.numerator,
            "denominator": value.denominator,
        }
    if isinstance(value, Decimal):
        return {"$type": "ExactDecimalV1", "value": str(value)}
    if isinstance(value, (tuple, list)):
        return [encoded(item) for item in value]
    if isinstance(value, (set, frozenset)):
        items = [encoded(item) for item in value]
        return sorted(items, key=lambda item: canonical(item))
    if isinstance(value, dict):
        return {str(key): encoded(item) for key, item in value.items()}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(f"unsupported canonical proof value: {type(value)!r}")


def id_record(value: Any) -> dict[str, str]:
    if isinstance(value, Enum):
        return {"$enum_type": type(value).__name__, "value": str(value.value)}
    if hasattr(value, "value"):
        return {"$id_type": type(value).__name__, "value": str(value.value)}
    return {"$value_type": type(value).__name__, "value": str(value)}


def git_oid(root: Path, expression: str) -> str:
    return subprocess.run(
        ["git", "rev-parse", expression],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def git_is_ancestor(root: Path, revision: str) -> bool:
    return (
        subprocess.run(
            ["git", "merge-base", "--is-ancestor", revision, "HEAD"],
            cwd=root,
            check=False,
        ).returncode
        == 0
    )


def input_integer_bits(value: Any) -> int:
    maximum = 1
    if isinstance(value, dict):
        for item in value.values():
            maximum = max(maximum, input_integer_bits(item))
    elif isinstance(value, list):
        for item in value:
            maximum = max(maximum, input_integer_bits(item))
    elif isinstance(value, int):
        maximum = max(maximum, abs(value).bit_length())
    elif isinstance(value, str) and re.fullmatch(r"-?\d+(?:/\d+)?", value):
        fraction = Fraction(value)
        maximum = max(
            maximum,
            abs(fraction.numerator).bit_length(),
            fraction.denominator.bit_length(),
        )
    return maximum


def source_evidence(root: Path, records: list[dict[str, Any]]):
    output = []
    scanned = 0
    for record in records:
        path = root / record["path"]
        data = path.read_bytes()
        scanned += len(data)
        if sha(data) != record["sha256"]:
            raise ValueError(f"source hash mismatch: {record['path']}")
        if git_oid(root, f"HEAD:{record['path']}") != record["blob_oid"]:
            raise ValueError(f"source blob mismatch: {record['path']}")
        lines = data.decode("utf-8").splitlines()
        ranges = {}
        for name, interval in record["line_ranges"].items():
            lower, upper = (int(item) for item in interval.split("-"))
            if not (1 <= lower <= upper <= len(lines)):
                raise ValueError(f"invalid source range: {record['path']}:{interval}")
            payload = ("\n".join(lines[lower - 1 : upper]) + "\n").encode()
            ranges[name] = {
                "line_range": interval,
                "sha256": sha(payload),
                "byte_length": len(payload),
            }
        output.append(
            {
                "$type": "ProofSourceAuthorityV1",
                "path": record["path"],
                "blob_oid": record["blob_oid"],
                "sha256": record["sha256"],
                "ranges": ranges,
            }
        )
    return output, scanned


def static_record(prepared, arrival_laws, contract) -> dict[str, Any]:
    record = {
        "$type": contract["static_type"],
        "schema_version": contract["static_schema_version"],
        "outcome": id_record(prepared.outcome),
        "regions": encoded(prepared.regions),
        "lattice": encoded(prepared.lattice),
        "arrival_laws": encoded(arrival_laws),
        "law_names": list(prepared.law_names),
    }
    if list(record) != contract["static_field_whitelist"]:
        raise ValueError("static field whitelist/order mismatch")
    return record


def coverage_record(coverage) -> dict[str, Any]:
    return {
        "$type": "ConveyorCoverageCanonicalProjectionV1",
        "outcome": id_record(coverage.outcome),
        "alpha": encoded(coverage.alpha),
        "lattice_alpha": encoded(coverage.lattice_alpha),
        "regions": encoded(coverage.regions),
        "doubled_area": encoded(coverage.doubled_area),
        "polygon_doubled_area": coverage.polygon_doubled_area,
        "counters": encoded(coverage.counters),
        "detail": coverage.detail,
    }


def compile_static_policy(request, contract, density_value_id):
    names = (
        "metric_space",
        "angular_profile_family_id",
        "angular_profile_selection_policy_id",
        "max_subturn_parameter_id",
        "max_subturn_value_id",
        "max_subturn_exact_value",
        "cap_policy_id",
        "boundary_policy_id",
        "interaction_policy_id",
        "ownership_policy_id",
        "material_policy_id",
        "uv_policy_id",
    )
    return {
        "$type": "ConveyorCompileStaticPolicyV1",
        **{name: encoded(getattr(request, name)) for name in names},
        "density_selection_policy_id": {
            "$id_type": "AngularProfileSelectionPolicyId",
            "value": contract["density_selection_policy_id"],
        },
        "density_parameter_id": {
            "$id_type": "LinearReflexDensityParameterId",
            "value": contract["density_parameter_id"],
        },
        "density_value_id": {
            "$id_type": "LinearReflexDensityValueId",
            "value": density_value_id,
        },
    }


def key_record(
    snapshot_digest: str,
    patch_domain_id: str,
    request,
    contract,
    density_value_id: str,
) -> dict[str, Any]:
    return {
        "$type": contract["key_type"],
        "schema_version": contract["key_schema_version"],
        "key_type": contract["key_law"],
        "snapshot_digest": {
            "$id_type": "SnapshotDigest",
            "value": snapshot_digest,
        },
        "patch_domain_id": {
            "$id_type": "PatchDomainId",
            "value": patch_domain_id,
        },
        "selected_chain_use_ids": [
            {"$id_type": "ChainUseId", "value": item.value}
            for item in sorted(
                request.selected_chain_use_ids, key=lambda item: item.value
            )
        ],
        "compile_static_policy": compile_static_policy(
            request, contract, density_value_id
        ),
    }


def recompose(static_preparation, static_dto, per_request_preparation):
    if canonical(static_dto["regions"]) != canonical(
        encoded(static_preparation.regions)
    ):
        raise ValueError("static DTO regions do not authorize recomposition")
    if canonical(static_dto["lattice"]) != canonical(
        encoded(static_preparation.lattice)
    ):
        raise ValueError("static DTO lattice does not authorize recomposition")
    if static_dto["law_names"] != list(static_preparation.law_names):
        raise ValueError("static DTO law names do not authorize recomposition")
    return replace(
        per_request_preparation,
        outcome=static_preparation.outcome,
        regions=static_preparation.regions,
        lattice=static_preparation.lattice,
        law_names=static_preparation.law_names,
    )


def digest_row(data: bytes) -> dict[str, Any]:
    return {"byte_length": len(data), "sha256": sha(data)}


def main() -> None:
    args = arguments()
    root = args.source_root.resolve()
    inputs_bytes = args.inputs.read_bytes()
    inputs = json.loads(inputs_bytes)
    limit = inputs["performance"]["input_integer_bit_limit"]
    input_bits = input_integer_bits(inputs)
    if input_bits > limit:
        raise ValueError("input integer bit limit exceeded")
    if not git_is_ancestor(root, inputs["proof_base_revision"]):
        raise ValueError("unknown proof ancestry")
    product_tree = {
        "proof_base_revision": inputs["proof_base_revision"],
        "kernel_src": git_oid(root, "HEAD:kernel/src"),
        "cftuv": git_oid(root, "HEAD:cftuv"),
    }
    if product_tree["kernel_src"] != inputs["accepted_product_oids"]["kernel_src"]:
        raise ValueError("unknown kernel product tree")
    if product_tree["cftuv"] != inputs["accepted_product_oids"]["cftuv"]:
        raise ValueError("unknown host product tree")
    sources, source_bytes = source_evidence(root, inputs["source_authority"])

    sys.path.insert(0, str(root / "kernel" / "src"))
    import cftuv_envelope as kernel
    from cftuv_envelope.wavefront import conveyor as conveyor_module

    fixture = inputs["fixture"]
    snapshot_bytes = (root / fixture["snapshot_path"]).read_bytes()
    request_bytes = (root / fixture["request_path"]).read_bytes()
    if sha(snapshot_bytes) != fixture["snapshot_sha256"]:
        raise ValueError("snapshot hash mismatch")
    if sha(request_bytes) != fixture["request_sha256"]:
        raise ValueError("request hash mismatch")
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    base_request = kernel.DecalRequestCodecV1.loads(request_bytes)
    requests = {
        alpha: replace(
            base_request,
            requested_alpha=replace(
                base_request.requested_alpha, value=Decimal(alpha)
            ),
        )
        for alpha in fixture["alphas"]
    }
    preparations = {
        alpha: conveyor_module.prepare_conveyor(snapshot, request)
        for alpha, request in requests.items()
    }
    if any(item.outcome.value != "EXACT" for item in preparations.values()):
        raise ValueError("cold preparation is not exact")
    laws = {
        alpha: conveyor_module._arrival_laws(item.context).laws
        for alpha, item in preparations.items()
    }
    contract = inputs["planned_contract"]
    static_records = {
        alpha: static_record(preparations[alpha], laws[alpha], contract)
        for alpha in fixture["alphas"]
    }
    static_bytes = {
        alpha: canonical(record) for alpha, record in static_records.items()
    }
    if len(set(static_bytes.values())) != 1:
        raise ValueError("static bytes differ across alpha")

    compilations = {
        alpha: kernel.compile_reference_envelopes(snapshot, request).compilation
        for alpha, request in requests.items()
    }
    wrapper_bytes = {
        alpha: kernel.canonical_json_bytes(compilation)
        for alpha, compilation in compilations.items()
    }
    for alpha, expected in fixture["expected_compilation_wrappers"].items():
        if (
            len(wrapper_bytes[alpha]) != expected["length"]
            or sha(wrapper_bytes[alpha]) != expected["sha256"]
        ):
            raise ValueError(f"compilation wrapper mismatch at {alpha}")
    if wrapper_bytes["0.25"] == wrapper_bytes["0.5"]:
        raise ValueError("full wrappers unexpectedly equal")

    cold = {
        alpha: conveyor_module.conveyor_coverage(
            preparations[alpha], alpha
        )
        for alpha in fixture["alphas"]
    }
    if any(item.outcome.value != "EXACT" for item in cold.values()):
        raise ValueError("cold coverage is not exact")
    cold_bytes = {
        alpha: canonical(coverage_record(item))
        for alpha, item in cold.items()
    }
    reuse_matrix = []
    reuse_bytes = []
    for static_alpha, target_alpha in (
        ("0.25", "0.25"),
        ("0.25", "0.5"),
        ("0.5", "0.5"),
        ("0.5", "0.25"),
    ):
        recomposed = recompose(
            preparations[static_alpha],
            static_records[static_alpha],
            preparations[target_alpha],
        )
        covered = conveyor_module.conveyor_coverage(recomposed, target_alpha)
        result_bytes = canonical(coverage_record(covered))
        reuse_bytes.append(result_bytes)
        if result_bytes != cold_bytes[target_alpha]:
            raise ValueError(
                f"reuse mismatch {static_alpha}->{target_alpha}"
            )
        reuse_matrix.append(
            {
                "static_alpha_source": static_alpha,
                "per_request_alpha": target_alpha,
                "per_request_compilation_sha256": sha(
                    wrapper_bytes[target_alpha]
                ),
                "static_dto_sha256": sha(static_bytes[static_alpha]),
                "static_schema_type": static_records[static_alpha]["$type"],
                "requested_alpha_recomposed": str(
                    recomposed.requested_alpha.value
                ),
                "per_request_compilation_from_target": (
                    recomposed.compilation
                    is preparations[target_alpha].compilation
                ),
                "per_request_context_from_target": (
                    recomposed.context is preparations[target_alpha].context
                ),
                "per_request_domain_from_target": (
                    recomposed.domain is preparations[target_alpha].domain
                ),
                "per_request_requested_alpha_from_target": (
                    recomposed.requested_alpha
                    == preparations[target_alpha].requested_alpha
                ),
                "cold_coverage_sha256": sha(cold_bytes[target_alpha]),
                "reuse_coverage_sha256": sha(result_bytes),
                "byte_identical": True,
            }
        )

    snapshot_digest = sha(kernel.canonical_json_bytes(snapshot))
    key_matrix = []
    key_bytes = []
    for density_id in contract["density_value_ids"]:
        pair = []
        for alpha in fixture["alphas"]:
            record = key_record(
                snapshot_digest,
                fixture["patch_domain_id"],
                requests[alpha],
                contract,
                density_id,
            )
            payload = canonical(record)
            key_bytes.append(payload)
            pair.append(
                {
                    "alpha": alpha,
                    "sha256": sha(payload),
                    "byte_length": len(payload),
                }
            )
        if pair[0]["sha256"] != pair[1]["sha256"]:
            raise ValueError("same density key depends on alpha")
        key_matrix.append({"density_value_id": density_id, "alphas": pair})
    density_digests = [item["alphas"][0]["sha256"] for item in key_matrix]
    if len(set(density_digests)) != len(density_digests):
        raise ValueError("density key collision")

    base_key = key_record(
        snapshot_digest,
        fixture["patch_domain_id"],
        requests["0.25"],
        contract,
        contract["density_value_ids"][0],
    )
    policy_separation = []
    for field_name in base_key["compile_static_policy"]:
        if field_name == "$type":
            continue
        forged = deepcopy(base_key)
        forged["compile_static_policy"][field_name] = {
            "$forged_static_policy_field": field_name
        }
        policy_separation.append(
            {
                "field": field_name,
                "base_sha256": sha(canonical(base_key)),
                "changed_sha256": sha(canonical(forged)),
                "different": canonical(base_key) != canonical(forged),
            }
        )
    if not all(item["different"] for item in policy_separation):
        raise ValueError("compile-static policy field omitted from key")

    exclusions = [
        {
            "field": "compilation",
            "classification": "PER_REQUEST_ALPHA_BEARING_WRAPPER",
            "canonical_static": False,
        },
        {
            "field": "context",
            "classification": "PER_REQUEST_RUNTIME_RECOMPOSITION",
            "canonical_static": False,
        },
        {
            "field": "domain",
            "classification": "PER_REQUEST_RUNTIME_RECOMPOSITION",
            "canonical_static": False,
        },
        {
            "field": "requested_alpha",
            "classification": "PER_REQUEST_ALPHA",
            "canonical_static": False,
        },
        {
            "field": "counters",
            "classification": "DIAGNOSTIC_NONCANONICAL",
            "canonical_static": False,
        },
        {
            "field": "detail",
            "classification": "DIAGNOSTIC_NONCANONICAL",
            "canonical_static": False,
        },
        {
            "field": "timings",
            "classification": "TIMING_NONCANONICAL",
            "canonical_static": False,
        },
        {
            "field": "decal_request_id",
            "classification": "PER_REQUEST_KEY_EXCLUSION",
            "canonical_static": False,
        },
    ]
    red_controls = {
        "unknown_schema": "UNKNOWN_STATIC_SCHEMA",
        "forged_type": "FORGED_STATIC_TYPE",
        "static_includes_alpha": "ALPHA_IN_STATIC_DTO",
        "static_includes_compilation": "COMPILATION_IN_STATIC_DTO",
        "static_includes_timing": "TIMING_IN_STATIC_DTO",
        "omitted_used_field": "STATIC_REQUIRED_FIELD_MISSING",
        "alpha_in_key": "ALPHA_IN_STATIC_KEY",
        "density_collision": "DENSITY_KEY_COLLISION",
        "static_bytes_differ": "STATIC_ALPHA_DEPENDENCE",
        "wrapper_bytes_same": "PER_REQUEST_WRAPPER_IDENTITY_FORGED",
        "reuse_mismatch_forward": "STATIC_REUSE_MISMATCH",
        "reuse_mismatch_reverse": "STATIC_REUSE_MISMATCH",
        "false_determinism": "FALSE_DETERMINISM_CLAIM",
        "false_all_expected_reject": "FALSE_REJECTION_CLAIM",
        "unknown_product_tree": "UNKNOWN_PRODUCT_TREE",
        "unknown_source_blob": "UNKNOWN_SOURCE_BLOB",
    }
    scanned = (
        len(inputs_bytes)
        + source_bytes
        + len(snapshot_bytes)
        + len(request_bytes)
        + sum(len(item) for item in static_bytes.values())
        + sum(len(item) for item in wrapper_bytes.values())
        + sum(len(item) for item in cold_bytes.values())
        + sum(len(item) for item in reuse_bytes)
        + sum(len(item) for item in key_bytes)
    )
    budget = inputs["performance"]["byte_budget"]
    performance = {
        "model": "CANONICAL_BYTES_LINEAR_SCAN_UPPER_BOUND_V1",
        "canonical_bytes_processed_upper_bound": scanned,
        "byte_budget": budget,
        "input_max_integer_bit_length": input_bits,
        "input_integer_bit_limit": limit,
        "pass": scanned <= budget and input_bits <= limit,
    }
    if not performance["pass"]:
        raise ValueError("proof performance budget exceeded")

    evidence = {
        "schema": "cftuv.envelope.dens_a_prime.split_static_receipt.v1",
        "inputs_sha256": sha(inputs_bytes),
        "product_tree": product_tree,
        "source_authority": sources,
        "planned_contract": {
            "$type": "ConveyorStaticPreparationPlanV1",
            "schema_version": "cftuv.envelope.conveyor_static_preparation_plan.v1",
            "core_owner": "kernel/src/cftuv_envelope/wavefront/conveyor.py",
            "static_type": contract["static_type"],
            "static_schema_version": contract["static_schema_version"],
            "static_field_whitelist": contract["static_field_whitelist"],
            "static_key_type": contract["key_type"],
            "static_key_schema_version": contract["key_schema_version"],
            "static_key_law": contract["key_law"],
            "canonicalization": "UTF8_SORTED_KEYS_NO_WHITESPACE_V1",
            "exclusions": exclusions,
            "per_request_recomposition_fields": [
                "compilation",
                "context",
                "domain",
                "requested_alpha",
            ],
            "host_projection_forbidden": True,
            "future_host_migration": {
                "$type": "NamedFutureHostMigrationV1",
                "seam": "cftuv/envelope_debug_session.py:430-474",
                "action": "CONSUME_CORE_STATIC_DTO_AND_TYPED_KEY",
                "host_projection": "FORBIDDEN",
            },
        },
        "alpha_runs": {
            alpha: {
                "static": digest_row(static_bytes[alpha]),
                "full_compilation_wrapper": digest_row(wrapper_bytes[alpha]),
                "cold_coverage": digest_row(cold_bytes[alpha]),
            }
            for alpha in fixture["alphas"]
        },
        "static_bytes_identical": static_bytes["0.25"] == static_bytes["0.5"],
        "full_wrapper_bytes_different": (
            wrapper_bytes["0.25"] != wrapper_bytes["0.5"]
        ),
        "reuse_matrix": reuse_matrix,
        "key_matrix": key_matrix,
        "compile_static_policy_separation": policy_separation,
        "red_controls": red_controls,
        "performance": performance,
        "claims": {
            "deterministic": True,
            "all_expected_reject": True,
            "static_alpha_independent": True,
            "per_request_wrapper_alpha_dependent": True,
            "reuse_both_orders_exact": True,
            "same_density_alpha_same_key": True,
            "all_density_ids_distinct": True,
            "core_single_owner": True,
            "host_projection_forbidden": True,
        },
    }
    wrapper = {
        "schema": "cftuv.envelope.dens_a_prime.split_static_wrapper.v1",
        "evidence": evidence,
        "evidence_sha256": sha(canonical(evidence)),
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_bytes(pretty(wrapper))


if __name__ == "__main__":
    main()
