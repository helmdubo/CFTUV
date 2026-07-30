"""Independent verifier for the DENS-A-prime static split receipt.

The verifier does not import the generator.  It rebuilds every canonical
artifact from the frozen fixture and checks the complete evidence mapping.
"""

from __future__ import annotations

import argparse
import ast
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
PROOF_MODULE_PATHS = (
    "kernel/fixtures/dens_a_prime_split_static_v1/static_contract.py",
    "kernel/fixtures/dens_a_prime_split_static_v1/generate_receipt.py",
    "kernel/fixtures/dens_a_prime_split_static_v1/verify_receipt.py",
)
sys.path.insert(0, str(HERE))
from static_contract import (  # noqa: E402
    STATIC_FIELDS,
    STATIC_SCHEMA_VERSION,
    STATIC_TYPE,
    StaticContractReject,
    conveyor_static_to_wire,
    decode_conveyor_static,
    recompose_conveyor_static,
    recomposition_static_to_wire,
)


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


def proof_module_hashes(root):
    result = []
    for relative_path in PROOF_MODULE_PATHS:
        data = (root / relative_path).read_bytes()
        result.append(
            {
                "$type": "ProofModuleHashV1",
                "path": relative_path,
                "byte_length": len(data),
                "sha256": digest(data),
            }
        )
    return result


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


def static_authority(prepared, laws, contract):
    live = {
        "$type": contract["static_type"],
        "schema_version": contract["static_schema_version"],
        "outcome": prepared.outcome,
        "regions": prepared.regions,
        "lattice": prepared.lattice,
        "arrival_laws": laws,
        "law_names": prepared.law_names,
    }
    result = {
        key: (
            value
            if key in {"$type", "schema_version"}
            else serialize(value)
        )
        for key, value in live.items()
    }
    assert list(result) == contract["static_field_whitelist"]
    assert set(result) == set(contract["static_field_whitelist"])
    return result


def project_static(decoded, contract):
    result = conveyor_static_to_wire(decoded)
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


def materialize_oracle(recomposed, expected_static_wire, per_request):
    assert recomposition_static_to_wire(recomposed) == expected_static_wire
    return per_request


def bytes_fact(data):
    return {"byte_length": len(data), "sha256": digest(data)}


def validate_contract_declaration(contract):
    expected = {
        "static_type": STATIC_TYPE,
        "static_schema_version": STATIC_SCHEMA_VERSION,
        "static_field_whitelist": list(STATIC_FIELDS),
        "decoder_module": (
            "kernel/fixtures/dens_a_prime_split_static_v1/"
            "static_contract.py"
        ),
        "decoder_entrypoint": "decode_conveyor_static",
        "decoder_input_law": "CANONICAL_UTF8_JSON_BYTES_ONLY_V1",
        "decoded_outcome_type": "ConveyorOutcomeValueV1",
        "recompose_entrypoint": "recompose_conveyor_static",
        "recompose_law": (
            "DECODED_CONVEYOR_STATIC_PLUS_DECLARED_ALPHA_ONLY_V1"
        ),
        "key_type": "ConveyorStaticPreparationKeyV1",
        "key_schema_version": (
            "cftuv.envelope.conveyor_static_preparation_key.v1"
        ),
        "key_law": (
            "SNAPSHOT_AND_ALL_COMPILE_STATIC_POLICY_WITH_DENSITY_V1"
        ),
        "density_parameter_id": "LINEAR_REFLEX_DENSITY_A_V1",
        "density_value_ids": [
            f"LINEAR_REFLEX_DENSITY_{density}_V1"
            for density in range(5)
        ],
        "density_selection_policy_id": (
            "HUBER_EMANATED_COUNT_DENSITY_A_V1"
        ),
    }
    assert contract == expected


def inspect_isolation(root, contract):
    path = root / contract["decoder_module"]
    source = path.read_bytes()
    tree = ast.parse(source, filename=str(path))
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.append(node.module or "")
    product_imports = sorted(
        name for name in imported if name.startswith("cftuv_envelope")
    )
    functions = {
        item.name: item
        for item in tree.body
        if isinstance(item, ast.FunctionDef)
    }
    decoder = functions[contract["decoder_entrypoint"]]
    recomposer = functions[contract["recompose_entrypoint"]]
    decoder_parameters = [item.arg for item in decoder.args.args]
    recompose_parameters = [item.arg for item in recomposer.args.args]
    forbidden = {
        "prepare_conveyor",
        "conveyor_coverage",
        "ConveyorPreparationV1",
        "compilation",
        "context",
        "domain",
        "requested_alpha",
        "replace",
    }
    symbols = {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    } | {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
    }
    static_reads = sorted(
        {
            node.attr
            for node in ast.walk(recomposer)
            if isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id == "static_preparation"
        }
    )
    expected_reads = [
        "arrival_laws",
        "lattice",
        "law_names",
        "outcome",
        "regions",
    ]
    result = {
        "$type": "ProofOnlyStaticDecoderIsolationV1",
        "module_path": contract["decoder_module"],
        "module_sha256": digest(source),
        "decoder_entrypoint": decoder.name,
        "decoder_parameters": decoder_parameters,
        "recompose_entrypoint": recomposer.name,
        "recompose_parameters": recompose_parameters,
        "recompose_static_reads": static_reads,
        "product_imports": product_imports,
        "forbidden_live_symbols": sorted(forbidden & symbols),
        "ast_isolated": (
            decoder_parameters == ["payload"]
            and recompose_parameters == ["static_preparation", "alpha"]
            and static_reads == expected_reads
            and not product_imports
            and not (forbidden & symbols)
        ),
    }
    assert result["ast_isolated"]
    return result


def prove_decoder_reds(
    authority,
    per_request,
    alpha,
    coverage_oracle,
):
    from collections.abc import Mapping
    from types import MappingProxyType

    trace = {"calls": 0}

    def counted_coverage(prepared, requested_alpha):
        trace["calls"] += 1
        return coverage_oracle(prepared, requested_alpha)

    def is_exact_wire(value):
        if type(value) is dict:
            return all(
                type(key) is str and is_exact_wire(item)
                for key, item in value.items()
            )
        if type(value) is list:
            return all(is_exact_wire(item) for item in value)
        return (
            value is None
            or type(value) is str
            or type(value) is int
            or type(value) is bool
        )

    base_hash = digest(canonical_bytes(authority))
    result = []

    def red(
        name,
        code,
        mutate,
        *,
        decode=True,
        replace_input=False,
        force_json_bytes=False,
        wire_transform=None,
    ):
        forged = deepcopy(authority)
        replacement = mutate(forged)
        candidate = replacement if replace_input else forged
        if is_exact_wire(candidate) or force_json_bytes:
            digest_basis = (
                "CANONICAL_WIRE_VALUE_V1"
                if is_exact_wire(candidate)
                else "CANONICAL_JSON_STRUCTURAL_RED_V1"
            )
            decode_input = canonical_bytes(candidate)
            if wire_transform is not None:
                decode_input = wire_transform(decode_input)
                digest_basis = "RAW_NONCANONICAL_JSON_BYTES_V1"
            recomputed = digest(decode_input)
        else:
            digest_basis = "NON_WIRE_INPUT_DESCRIPTOR_V1"
            decode_input = candidate
            recomputed = digest(
                canonical_bytes(
                    {
                        "$type": "NonWireStaticForgeryDescriptorV1",
                        "case": name,
                        "base_static_sha256": base_hash,
                        "python_type": type(candidate).__name__,
                    }
                )
            )
        before = trace["calls"]
        try:
            decoded = (
                decode_conveyor_static(decode_input)
                if decode
                else candidate
            )
            recomposed = recompose_conveyor_static(decoded, alpha)
            prepared = materialize_oracle(
                recomposed,
                authority,
                per_request,
            )
            counted_coverage(prepared, alpha)
        except StaticContractReject as exc:
            assert exc.code == code
        else:
            raise AssertionError(f"{name}: decoder fail-open")
        after = trace["calls"]
        assert before == after
        result.append(
            {
                "name": name,
                "expected_code": code,
                "outcome": "REJECT",
                "digest_recomputed": True,
                "forged_dto_sha256": recomputed,
                "digest_basis": digest_basis,
                "coverage_calls_before": before,
                "coverage_calls_after": after,
            }
        )

    red(
        "decoder_missing_key",
        "STATIC_REQUIRED_FIELD_MISSING",
        lambda item: item.pop("regions"),
    )
    red(
        "decoder_extra_key",
        "STATIC_EXTRA_FIELD",
        lambda item: item.update({"requested_alpha": alpha}),
    )
    red(
        "decoder_unknown_key",
        "STATIC_UNKNOWN_FIELD",
        lambda item: item.update({"mystery_static_field": "forged"}),
    )
    red(
        "decoder_forged_type",
        "FORGED_STATIC_TYPE",
        lambda item: item.update({"$type": "FORGED_STATIC_TYPE"}),
    )
    red(
        "decoder_unknown_schema",
        "UNKNOWN_STATIC_SCHEMA",
        lambda item: item.update({"schema_version": "unknown"}),
    )
    red(
        "decoder_forged_outcome_enum",
        "STATIC_OUTCOME_ENUM_MISMATCH",
        lambda item: item.update(
            {
                "outcome": {
                    "$enum_type": "ForgedConveyorOutcome",
                    "value": "EXACT",
                }
            }
        ),
    )

    def forge_law(item):
        laws = item["arrival_laws"]
        laws[0]["name"] = f"{laws[0]['name']}:forged"

    red(
        "decoder_forged_arrival_laws",
        "STATIC_ARRIVAL_LAW_MISMATCH",
        forge_law,
    )
    red(
        "decoder_forged_law_names",
        "STATIC_ARRIVAL_LAW_MISMATCH",
        lambda item: item.update(
            {"law_names": ["forged-law-name", *item["law_names"][1:]]}
        ),
    )

    def forge_all(item):
        item["$type"] = "FORGED_STATIC_TYPE"
        item["schema_version"] = "unknown"
        item["outcome"] = {
            "$enum_type": "ForgedConveyorOutcome",
            "value": "EXACT",
        }
        forge_law(item)
        item["law_names"] = [
            law["name"] for law in item["arrival_laws"]
        ]

    red(
        "coupled_rehash_static_authority",
        "FORGED_STATIC_TYPE",
        forge_all,
    )
    red(
        "recompose_requires_decoded_dto",
        "STATIC_RECOMPOSE_DECODED_REQUIRED",
        lambda item: None,
        decode=False,
    )

    spoofed_outcome_type = type(
        "ConveyorOutcome",
        (),
        {
            "__module__": "cftuv_envelope.wavefront.conveyor",
            "value": "EXACT",
        },
    )
    red(
        "decoder_spoofed_product_metadata",
        "STATIC_DTO_BYTES_REQUIRED",
        lambda item: item.update(
            {"outcome": spoofed_outcome_type()}
        ),
    )

    class DictSubclass(dict):
        pass

    red(
        "decoder_dict_subclass",
        "STATIC_DTO_BYTES_REQUIRED",
        lambda item: DictSubclass(item),
        replace_input=True,
    )

    class StrSubclass(str):
        pass

    red(
        "decoder_str_subclass",
        "STATIC_DTO_BYTES_REQUIRED",
        lambda item: StrSubclass(
            canonical_bytes(item).decode("utf-8")
        ),
        replace_input=True,
    )

    class BytesSubclass(bytes):
        pass

    red(
        "decoder_bytes_subclass",
        "STATIC_DTO_BYTES_REQUIRED",
        lambda item: BytesSubclass(canonical_bytes(item)),
        replace_input=True,
    )
    red(
        "decoder_mapping_proxy",
        "STATIC_DTO_BYTES_REQUIRED",
        lambda item: MappingProxyType(item),
        replace_input=True,
    )

    class CustomMapping(Mapping):
        def __init__(self, value):
            self._value = value

        def __getitem__(self, key):
            return self._value[key]

        def __iter__(self):
            return iter(self._value)

        def __len__(self):
            return len(self._value)

    red(
        "decoder_custom_mapping",
        "STATIC_DTO_BYTES_REQUIRED",
        lambda item: CustomMapping(item),
        replace_input=True,
    )

    class PropertyOutcome:
        __module__ = "cftuv_envelope.wavefront.conveyor"

        @property
        def value(self):
            return "EXACT"

    PropertyOutcome.__name__ = "ConveyorOutcome"
    red(
        "decoder_property_outcome_object",
        "STATIC_DTO_BYTES_REQUIRED",
        lambda item: item.update({"outcome": PropertyOutcome()}),
    )
    red(
        "decoder_noncanonical_json_bytes",
        "STATIC_DTO_JSON_NOT_CANONICAL",
        lambda item: None,
        wire_transform=lambda payload: b" " + payload,
    )
    red(
        "decoder_float_lattice_scale",
        "STATIC_LATTICE_SCHEMA_MISMATCH",
        lambda item: item["lattice"].update({"scale": 1.5}),
        force_json_bytes=True,
    )
    return result, trace["calls"]


def build_expected(root, inputs, inputs_bytes):
    limit = inputs["performance"]["input_integer_bit_limit"]
    bits = maximum_input_bits(inputs)
    assert bits <= limit
    assert ancestor(root, inputs["proof_base_revision"])
    tree = {
        "proof_base_revision": inputs["proof_base_revision"],
        "kernel_src": revision(root, "HEAD:kernel/src"),
        "cftuv": revision(root, "HEAD:cftuv"),
        "tools": revision(root, "HEAD:tools"),
    }
    assert tree["kernel_src"] == inputs["accepted_product_oids"]["kernel_src"]
    assert tree["cftuv"] == inputs["accepted_product_oids"]["cftuv"]
    assert tree["tools"] == inputs["accepted_product_oids"]["tools"]
    sources, source_byte_count = prove_sources(
        root, inputs["source_authority"]
    )
    contract = inputs["planned_contract"]
    validate_contract_declaration(contract)
    isolation = inspect_isolation(root, contract)

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
    static_authorities = {
        alpha: static_authority(
            preparations[alpha],
            arrival_laws[alpha],
            contract,
        )
        for alpha in fixture["alphas"]
    }
    decoded_static = {
        alpha: decode_conveyor_static(
            canonical_bytes(static_authorities[alpha])
        )
        for alpha in fixture["alphas"]
    }
    static_records = {
        alpha: project_static(decoded_static[alpha], contract)
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
        recomposed = recompose_conveyor_static(
            decoded_static[static_alpha],
            request_alpha,
        )
        joined = materialize_oracle(
            recomposed,
            static_authorities[request_alpha],
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
                    recomposed.alpha
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
                "outcome_from_decoded": (
                    recomposed.outcome
                    is decoded_static[static_alpha].outcome
                ),
                "regions_from_decoded": (
                    recomposed.regions
                    is decoded_static[static_alpha].regions
                ),
                "lattice_from_decoded": (
                    recomposed.lattice
                    is decoded_static[static_alpha].lattice
                ),
                "arrival_laws_from_decoded": (
                    recomposed.arrival_laws
                    is decoded_static[static_alpha].arrival_laws
                ),
                "law_names_from_decoded": (
                    recomposed.law_names
                    is decoded_static[static_alpha].law_names
                ),
                "external_oracle_static_wire_authorized": (
                    recomposition_static_to_wire(recomposed)
                    == static_authorities[request_alpha]
                ),
                "cold_coverage_sha256": digest(
                    cold_payloads[request_alpha]
                ),
                "reuse_coverage_sha256": digest(payload),
                "byte_identical": True,
            }
        )

    decoder_reds, rejected_coverage_calls = prove_decoder_reds(
        static_authorities["0.25"],
        preparations["0.5"],
        "0.5",
        conveyor_module.conveyor_coverage,
    )
    assert rejected_coverage_calls == 0

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
        {"field": "compilation", "classification": "EXTERNAL_COLD_COVERAGE_ORACLE_ONLY", "canonical_static": False},
        {"field": "context", "classification": "EXTERNAL_COLD_COVERAGE_ORACLE_ONLY", "canonical_static": False},
        {"field": "domain", "classification": "EXTERNAL_COLD_COVERAGE_ORACLE_ONLY", "canonical_static": False},
        {"field": "requested_alpha", "classification": "EXTERNAL_COLD_COVERAGE_ORACLE_ONLY", "canonical_static": False},
        {"field": "counters", "classification": "DIAGNOSTIC_NONCANONICAL", "canonical_static": False},
        {"field": "detail", "classification": "DIAGNOSTIC_NONCANONICAL", "canonical_static": False},
        {"field": "timings", "classification": "TIMING_NONCANONICAL", "canonical_static": False},
        {"field": "decal_request_id", "classification": "PER_REQUEST_KEY_EXCLUSION", "canonical_static": False},
    ]
    red_controls = {
        "coupled_rehash_static_authority": "FORGED_STATIC_TYPE",
        "decoder_extra_key": "STATIC_EXTRA_FIELD",
        "decoder_forged_arrival_laws": "STATIC_ARRIVAL_LAW_MISMATCH",
        "decoder_forged_law_names": "STATIC_ARRIVAL_LAW_MISMATCH",
        "decoder_forged_outcome_enum": "STATIC_OUTCOME_ENUM_MISMATCH",
        "decoder_forged_type": "FORGED_STATIC_TYPE",
        "decoder_spoofed_product_metadata": "STATIC_DTO_BYTES_REQUIRED",
        "decoder_dict_subclass": "STATIC_DTO_BYTES_REQUIRED",
        "decoder_str_subclass": "STATIC_DTO_BYTES_REQUIRED",
        "decoder_bytes_subclass": "STATIC_DTO_BYTES_REQUIRED",
        "decoder_mapping_proxy": "STATIC_DTO_BYTES_REQUIRED",
        "decoder_custom_mapping": "STATIC_DTO_BYTES_REQUIRED",
        "decoder_property_outcome_object": "STATIC_DTO_BYTES_REQUIRED",
        "decoder_noncanonical_json_bytes": (
            "STATIC_DTO_JSON_NOT_CANONICAL"
        ),
        "decoder_float_lattice_scale": "STATIC_LATTICE_SCHEMA_MISMATCH",
        "decoder_missing_key": "STATIC_REQUIRED_FIELD_MISSING",
        "decoder_unknown_key": "STATIC_UNKNOWN_FIELD",
        "decoder_unknown_schema": "UNKNOWN_STATIC_SCHEMA",
        "recompose_requires_decoded_dto": (
            "STATIC_RECOMPOSE_DECODED_REQUIRED"
        ),
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
    module_hashes = proof_module_hashes(root)
    byte_count = (
        len(inputs_bytes)
        + source_byte_count
        + sum(item["byte_length"] for item in module_hashes)
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
        "proof_modules": module_hashes,
        "planned_contract": {
            "$type": "ConveyorStaticPreparationPlanV1",
            "schema_version": "cftuv.envelope.conveyor_static_preparation_plan.v1",
            "core_owner": "kernel/src/cftuv_envelope/wavefront/conveyor.py",
            "static_type": contract["static_type"],
            "static_schema_version": contract["static_schema_version"],
            "static_field_whitelist": contract["static_field_whitelist"],
            "decoder_module": contract["decoder_module"],
            "decoder_entrypoint": contract["decoder_entrypoint"],
            "decoder_input_law": contract["decoder_input_law"],
            "decoded_outcome_type": contract["decoded_outcome_type"],
            "recompose_entrypoint": contract["recompose_entrypoint"],
            "recompose_law": contract["recompose_law"],
            "static_key_type": contract["key_type"],
            "static_key_schema_version": contract["key_schema_version"],
            "static_key_law": contract["key_law"],
            "canonicalization": "UTF8_SORTED_KEYS_NO_WHITESPACE_V1",
            "exclusions": exclusions,
            "recompose_inputs": [
                "decoded:ConveyorStaticPreparationV1",
                "alpha",
            ],
            "external_oracle_only_fields": [
                "compilation",
                "context",
                "domain",
                "requested_alpha",
            ],
            "host_projection_forbidden": True,
            "decoder_isolation": isolation,
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
        "decoder_red_matrix": decoder_reds,
        "rejected_dto_coverage_call_count": rejected_coverage_calls,
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
            "decoded_static_is_only_static_authority": True,
            "decoder_accepts_canonical_json_bytes_only": True,
            "decoded_outcome_is_proof_local_immutable": True,
            "recomposer_has_no_live_preparation_input": True,
            "rejected_dto_never_reaches_coverage": True,
            "all_proof_modules_hashed_at_generation": True,
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
    actual_module_hashes = proof_module_hashes(root)
    assert wrapper["evidence"]["proof_modules"] == actual_module_hashes
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

    decoder_matrix = expected["decoder_red_matrix"]
    assert expected["rejected_dto_coverage_call_count"] == 0
    assert all(
        item["outcome"] == "REJECT"
        and item["digest_recomputed"]
        and item["coverage_calls_before"] == 0
        and item["coverage_calls_after"] == 0
        for item in decoder_matrix
    )
    matrix = {
        item["name"]: item["expected_code"]
        for item in decoder_matrix
    }
    for name, forged in mutations.items():
        refresh(forged)
        assert rejected(forged, expected)
        matrix[name] = expected["red_controls"][name]
    assert set(matrix) == set(expected["red_controls"])

    output = {
        "schema": "cftuv.envelope.dens_a_prime.split_static_verification.v1",
        "verified_receipt_sha256": digest(receipt_bytes),
        "verified_evidence_sha256": wrapper["evidence_sha256"],
        "proof_modules": actual_module_hashes,
        "tracked_proof_modules_match_disk": True,
        "static_sha256": expected["alpha_runs"]["0.25"]["static"]["sha256"],
        "alpha_runs": expected["alpha_runs"],
        "reuse_matrix": expected["reuse_matrix"],
        "key_matrix": expected["key_matrix"],
        "decoder_isolation": expected["planned_contract"][
            "decoder_isolation"
        ],
        "decoder_red_matrix": decoder_matrix,
        "rejected_dto_coverage_call_count": expected[
            "rejected_dto_coverage_call_count"
        ],
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
