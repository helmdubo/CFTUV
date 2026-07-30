"""Independent verifier for the DENS-A-prime static split receipt.

The verifier does not import the generator.  It rebuilds every canonical
artifact from the frozen fixture and checks the complete evidence mapping.
"""

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


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, default=HERE.parents[2])
    parser.add_argument("--inputs", type=Path, default=HERE / "inputs.json")
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


def canonical_bytes(value):
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def pretty_bytes(value):
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n"
    ).encode("utf-8")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def serialize(value):
    if is_dataclass(value):
        result = {"$type": type(value).__name__}
        for field in fields(value):
            result[field.name] = serialize(getattr(value, field.name))
        return result
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
        return [serialize(item) for item in value]
    if isinstance(value, (set, frozenset)):
        result = [serialize(item) for item in value]
        return sorted(result, key=canonical_bytes)
    if isinstance(value, dict):
        return {str(key): serialize(item) for key, item in value.items()}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(type(value))


def typed_value(value):
    if isinstance(value, Enum):
        return {"$enum_type": type(value).__name__, "value": str(value.value)}
    if hasattr(value, "value"):
        return {"$id_type": type(value).__name__, "value": str(value.value)}
    return {"$value_type": type(value).__name__, "value": str(value)}


def revision(root, expression):
    return subprocess.run(
        ["git", "rev-parse", expression],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def ancestor(root, value):
    return (
        subprocess.run(
            ["git", "merge-base", "--is-ancestor", value, "HEAD"],
            cwd=root,
            check=False,
        ).returncode
        == 0
    )


def maximum_input_bits(value):
    result = 1
    if isinstance(value, dict):
        for item in value.values():
            result = max(result, maximum_input_bits(item))
    elif isinstance(value, list):
        for item in value:
            result = max(result, maximum_input_bits(item))
    elif isinstance(value, int):
        result = max(result, abs(value).bit_length())
    elif isinstance(value, str) and re.fullmatch(r"-?\d+(?:/\d+)?", value):
        number = Fraction(value)
        result = max(
            result,
            abs(number.numerator).bit_length(),
            number.denominator.bit_length(),
        )
    return result


def prove_sources(root, declarations):
    result = []
    byte_count = 0
    for declaration in declarations:
        data = (root / declaration["path"]).read_bytes()
        byte_count += len(data)
        assert digest(data) == declaration["sha256"]
        assert revision(root, f"HEAD:{declaration['path']}") == declaration[
            "blob_oid"
        ]
        text_lines = data.decode("utf-8").splitlines()
        ranges = {}
        for name, span in declaration["line_ranges"].items():
            begin, end = [int(item) for item in span.split("-")]
            assert 1 <= begin <= end <= len(text_lines)
            selected = (
                "\n".join(text_lines[begin - 1 : end]) + "\n"
            ).encode()
            ranges[name] = {
                "line_range": span,
                "sha256": digest(selected),
                "byte_length": len(selected),
            }
        result.append(
            {
                "$type": "ProofSourceAuthorityV1",
                "path": declaration["path"],
                "blob_oid": declaration["blob_oid"],
                "sha256": declaration["sha256"],
                "ranges": ranges,
            }
        )
    return result, byte_count


def project_static(prepared, laws, contract):
    result = {
        "$type": contract["static_type"],
        "schema_version": contract["static_schema_version"],
        "outcome": typed_value(prepared.outcome),
        "regions": serialize(prepared.regions),
        "lattice": serialize(prepared.lattice),
        "arrival_laws": serialize(laws),
        "law_names": list(prepared.law_names),
    }
    assert list(result) == contract["static_field_whitelist"]
    assert set(result) == set(contract["static_field_whitelist"])
    return result


def project_coverage(value):
    return {
        "$type": "ConveyorCoverageCanonicalProjectionV1",
        "outcome": typed_value(value.outcome),
        "alpha": serialize(value.alpha),
        "lattice_alpha": serialize(value.lattice_alpha),
        "regions": serialize(value.regions),
        "doubled_area": serialize(value.doubled_area),
        "polygon_doubled_area": value.polygon_doubled_area,
        "counters": serialize(value.counters),
        "detail": value.detail,
    }


def static_policy(request, contract, density_id):
    result = {"$type": "ConveyorCompileStaticPolicyV1"}
    for field_name in (
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
    ):
        result[field_name] = serialize(getattr(request, field_name))
    result.update(
        {
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
                "value": density_id,
            },
        }
    )
    return result


def static_key(snapshot_hash, domain_id, request, contract, density_id):
    return {
        "$type": contract["key_type"],
        "schema_version": contract["key_schema_version"],
        "key_type": contract["key_law"],
        "snapshot_digest": {
            "$id_type": "SnapshotDigest",
            "value": snapshot_hash,
        },
        "patch_domain_id": {
            "$id_type": "PatchDomainId",
            "value": domain_id,
        },
        "selected_chain_use_ids": [
            {"$id_type": "ChainUseId", "value": item.value}
            for item in sorted(
                request.selected_chain_use_ids, key=lambda item: item.value
            )
        ],
        "compile_static_policy": static_policy(
            request, contract, density_id
        ),
    }


def recomposed(static_source, static_dto, per_request):
    assert canonical_bytes(static_dto["regions"]) == canonical_bytes(
        serialize(static_source.regions)
    )
    assert canonical_bytes(static_dto["lattice"]) == canonical_bytes(
        serialize(static_source.lattice)
    )
    assert static_dto["law_names"] == list(static_source.law_names)
    return replace(
        per_request,
        outcome=static_source.outcome,
        regions=static_source.regions,
        lattice=static_source.lattice,
        law_names=static_source.law_names,
    )


def bytes_fact(data):
    return {"byte_length": len(data), "sha256": digest(data)}


def build_expected(root, inputs, inputs_bytes):
    limit = inputs["performance"]["input_integer_bit_limit"]
    bits = maximum_input_bits(inputs)
    assert bits <= limit
    assert ancestor(root, inputs["proof_base_revision"])
    tree = {
        "proof_base_revision": inputs["proof_base_revision"],
        "kernel_src": revision(root, "HEAD:kernel/src"),
        "cftuv": revision(root, "HEAD:cftuv"),
    }
    assert tree["kernel_src"] == inputs["accepted_product_oids"]["kernel_src"]
    assert tree["cftuv"] == inputs["accepted_product_oids"]["cftuv"]
    sources, source_byte_count = prove_sources(
        root, inputs["source_authority"]
    )

    sys.path.insert(0, str(root / "kernel" / "src"))
    import cftuv_envelope as kernel
    from cftuv_envelope.wavefront import conveyor as conveyor_module

    fixture = inputs["fixture"]
    snapshot_bytes = (root / fixture["snapshot_path"]).read_bytes()
    request_bytes = (root / fixture["request_path"]).read_bytes()
    assert digest(snapshot_bytes) == fixture["snapshot_sha256"]
    assert digest(request_bytes) == fixture["request_sha256"]
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    original_request = kernel.DecalRequestCodecV1.loads(request_bytes)
    requests = {}
    for alpha in fixture["alphas"]:
        requests[alpha] = replace(
            original_request,
            requested_alpha=replace(
                original_request.requested_alpha, value=Decimal(alpha)
            ),
        )
    preparations = {
        alpha: conveyor_module.prepare_conveyor(snapshot, request)
        for alpha, request in requests.items()
    }
    assert all(item.outcome.value == "EXACT" for item in preparations.values())
    arrival_laws = {
        alpha: conveyor_module._arrival_laws(item.context).laws
        for alpha, item in preparations.items()
    }
    contract = inputs["planned_contract"]
    static_records = {
        alpha: project_static(
            preparations[alpha], arrival_laws[alpha], contract
        )
        for alpha in fixture["alphas"]
    }
    static_payloads = {
        alpha: canonical_bytes(record)
        for alpha, record in static_records.items()
    }
    assert static_payloads["0.25"] == static_payloads["0.5"]

    compiled = {
        alpha: kernel.compile_reference_envelopes(snapshot, request).compilation
        for alpha, request in requests.items()
    }
    wrappers = {
        alpha: kernel.canonical_json_bytes(value)
        for alpha, value in compiled.items()
    }
    for alpha, expected in fixture["expected_compilation_wrappers"].items():
        assert len(wrappers[alpha]) == expected["length"]
        assert digest(wrappers[alpha]) == expected["sha256"]
    assert wrappers["0.25"] != wrappers["0.5"]

    cold_results = {
        alpha: conveyor_module.conveyor_coverage(
            preparations[alpha], alpha
        )
        for alpha in fixture["alphas"]
    }
    assert all(item.outcome.value == "EXACT" for item in cold_results.values())
    cold_payloads = {
        alpha: canonical_bytes(project_coverage(value))
        for alpha, value in cold_results.items()
    }
    reuse_rows = []
    reuse_payloads = []
    for static_alpha, request_alpha in (
        ("0.25", "0.25"),
        ("0.25", "0.5"),
        ("0.5", "0.5"),
        ("0.5", "0.25"),
    ):
        joined = recomposed(
            preparations[static_alpha],
            static_records[static_alpha],
            preparations[request_alpha],
        )
        coverage = conveyor_module.conveyor_coverage(joined, request_alpha)
        payload = canonical_bytes(project_coverage(coverage))
        reuse_payloads.append(payload)
        assert payload == cold_payloads[request_alpha]
        reuse_rows.append(
            {
                "static_alpha_source": static_alpha,
                "per_request_alpha": request_alpha,
                "per_request_compilation_sha256": digest(
                    wrappers[request_alpha]
                ),
                "static_dto_sha256": digest(static_payloads[static_alpha]),
                "static_schema_type": static_records[static_alpha]["$type"],
                "requested_alpha_recomposed": str(
                    joined.requested_alpha.value
                ),
                "per_request_compilation_from_target": (
                    joined.compilation
                    is preparations[request_alpha].compilation
                ),
                "per_request_context_from_target": (
                    joined.context is preparations[request_alpha].context
                ),
                "per_request_domain_from_target": (
                    joined.domain is preparations[request_alpha].domain
                ),
                "per_request_requested_alpha_from_target": (
                    joined.requested_alpha
                    == preparations[request_alpha].requested_alpha
                ),
                "cold_coverage_sha256": digest(
                    cold_payloads[request_alpha]
                ),
                "reuse_coverage_sha256": digest(payload),
                "byte_identical": True,
            }
        )

    snapshot_hash = digest(kernel.canonical_json_bytes(snapshot))
    key_rows = []
    key_payloads = []
    for density_id in contract["density_value_ids"]:
        alpha_rows = []
        for alpha in fixture["alphas"]:
            payload = canonical_bytes(
                static_key(
                    snapshot_hash,
                    fixture["patch_domain_id"],
                    requests[alpha],
                    contract,
                    density_id,
                )
            )
            key_payloads.append(payload)
            alpha_rows.append(
                {
                    "alpha": alpha,
                    "sha256": digest(payload),
                    "byte_length": len(payload),
                }
            )
        assert alpha_rows[0]["sha256"] == alpha_rows[1]["sha256"]
        key_rows.append(
            {"density_value_id": density_id, "alphas": alpha_rows}
        )
    density_hashes = [item["alphas"][0]["sha256"] for item in key_rows]
    assert len(set(density_hashes)) == 5

    first_key = static_key(
        snapshot_hash,
        fixture["patch_domain_id"],
        requests["0.25"],
        contract,
        contract["density_value_ids"][0],
    )
    policy_rows = []
    for field_name in first_key["compile_static_policy"]:
        if field_name == "$type":
            continue
        altered = deepcopy(first_key)
        altered["compile_static_policy"][field_name] = {
            "$forged_static_policy_field": field_name
        }
        policy_rows.append(
            {
                "field": field_name,
                "base_sha256": digest(canonical_bytes(first_key)),
                "changed_sha256": digest(canonical_bytes(altered)),
                "different": canonical_bytes(first_key)
                != canonical_bytes(altered),
            }
        )
    assert all(item["different"] for item in policy_rows)

    exclusions = [
        {"field": "compilation", "classification": "PER_REQUEST_ALPHA_BEARING_WRAPPER", "canonical_static": False},
        {"field": "context", "classification": "PER_REQUEST_RUNTIME_RECOMPOSITION", "canonical_static": False},
        {"field": "domain", "classification": "PER_REQUEST_RUNTIME_RECOMPOSITION", "canonical_static": False},
        {"field": "requested_alpha", "classification": "PER_REQUEST_ALPHA", "canonical_static": False},
        {"field": "counters", "classification": "DIAGNOSTIC_NONCANONICAL", "canonical_static": False},
        {"field": "detail", "classification": "DIAGNOSTIC_NONCANONICAL", "canonical_static": False},
        {"field": "timings", "classification": "TIMING_NONCANONICAL", "canonical_static": False},
        {"field": "decal_request_id", "classification": "PER_REQUEST_KEY_EXCLUSION", "canonical_static": False},
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
    byte_count = (
        len(inputs_bytes)
        + source_byte_count
        + len(snapshot_bytes)
        + len(request_bytes)
        + sum(len(item) for item in static_payloads.values())
        + sum(len(item) for item in wrappers.values())
        + sum(len(item) for item in cold_payloads.values())
        + sum(len(item) for item in reuse_payloads)
        + sum(len(item) for item in key_payloads)
    )
    budget = inputs["performance"]["byte_budget"]
    performance = {
        "model": "CANONICAL_BYTES_LINEAR_SCAN_UPPER_BOUND_V1",
        "canonical_bytes_processed_upper_bound": byte_count,
        "byte_budget": budget,
        "input_max_integer_bit_length": bits,
        "input_integer_bit_limit": limit,
        "pass": byte_count <= budget and bits <= limit,
    }
    assert performance["pass"]
    return {
        "schema": "cftuv.envelope.dens_a_prime.split_static_receipt.v1",
        "inputs_sha256": digest(inputs_bytes),
        "product_tree": tree,
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
                "static": bytes_fact(static_payloads[alpha]),
                "full_compilation_wrapper": bytes_fact(wrappers[alpha]),
                "cold_coverage": bytes_fact(cold_payloads[alpha]),
            }
            for alpha in fixture["alphas"]
        },
        "static_bytes_identical": True,
        "full_wrapper_bytes_different": True,
        "reuse_matrix": reuse_rows,
        "key_matrix": key_rows,
        "compile_static_policy_separation": policy_rows,
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


def refresh(wrapper):
    wrapper["evidence_sha256"] = digest(
        canonical_bytes(wrapper["evidence"])
    )


def rejected(wrapper, expected):
    try:
        assert wrapper["schema"] == (
            "cftuv.envelope.dens_a_prime.split_static_wrapper.v1"
        )
        assert wrapper["evidence_sha256"] == digest(
            canonical_bytes(wrapper["evidence"])
        )
        assert wrapper["evidence"] == expected
    except (AssertionError, KeyError):
        return True
    return False


def main():
    args = parse_arguments()
    root = args.source_root.resolve()
    inputs_bytes = args.inputs.read_bytes()
    inputs = json.loads(inputs_bytes)
    receipt_bytes = args.receipt.read_bytes()
    wrapper = json.loads(receipt_bytes)
    expected = build_expected(root, inputs, inputs_bytes)
    assert wrapper["schema"] == (
        "cftuv.envelope.dens_a_prime.split_static_wrapper.v1"
    )
    assert wrapper["evidence_sha256"] == digest(
        canonical_bytes(wrapper["evidence"])
    )
    assert wrapper["evidence"] == expected

    mutations = {}
    forged = deepcopy(wrapper)
    forged["evidence"]["planned_contract"]["schema_version"] = "unknown"
    mutations["unknown_schema"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["planned_contract"]["$type"] = "FORGED"
    mutations["forged_type"] = forged
    for name, field_name in (
        ("static_includes_alpha", "requested_alpha"),
        ("static_includes_compilation", "compilation"),
        ("static_includes_timing", "timings"),
    ):
        forged = deepcopy(wrapper)
        forged["evidence"]["planned_contract"][
            "static_field_whitelist"
        ].append(field_name)
        mutations[name] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["planned_contract"]["static_field_whitelist"].remove(
        "regions"
    )
    mutations["omitted_used_field"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["key_matrix"][0]["alphas"][1]["sha256"] = (
        "alpha-dependent-key"
    )
    mutations["alpha_in_key"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["key_matrix"][1]["alphas"] = deepcopy(
        forged["evidence"]["key_matrix"][0]["alphas"]
    )
    mutations["density_collision"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["alpha_runs"]["0.5"]["static"]["sha256"] = "different"
    mutations["static_bytes_differ"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["alpha_runs"]["0.5"][
        "full_compilation_wrapper"
    ] = deepcopy(
        forged["evidence"]["alpha_runs"]["0.25"][
            "full_compilation_wrapper"
        ]
    )
    mutations["wrapper_bytes_same"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["reuse_matrix"][1]["reuse_coverage_sha256"] = "wrong"
    mutations["reuse_mismatch_forward"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["reuse_matrix"][3]["reuse_coverage_sha256"] = "wrong"
    mutations["reuse_mismatch_reverse"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["claims"]["deterministic"] = False
    mutations["false_determinism"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["claims"]["all_expected_reject"] = False
    mutations["false_all_expected_reject"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["product_tree"]["kernel_src"] = "0" * 40
    mutations["unknown_product_tree"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["source_authority"][0]["blob_oid"] = "0" * 40
    mutations["unknown_source_blob"] = forged

    matrix = {}
    for name, forged in mutations.items():
        refresh(forged)
        assert rejected(forged, expected)
        matrix[name] = expected["red_controls"][name]
    assert set(matrix) == set(expected["red_controls"])

    output = {
        "schema": "cftuv.envelope.dens_a_prime.split_static_verification.v1",
        "verified_receipt_sha256": digest(receipt_bytes),
        "verified_evidence_sha256": wrapper["evidence_sha256"],
        "static_sha256": expected["alpha_runs"]["0.25"]["static"]["sha256"],
        "alpha_runs": expected["alpha_runs"],
        "reuse_matrix": expected["reuse_matrix"],
        "key_matrix": expected["key_matrix"],
        "exploit_regression_matrix": matrix,
        "all_expected_reject": set(matrix) == set(expected["red_controls"]),
        "performance": expected["performance"],
        "verdict": "VERIFIED",
        "scope_note": "proof-only split; no general DENS-A-prime GO",
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_bytes(pretty_bytes(output))


if __name__ == "__main__":
    main()
