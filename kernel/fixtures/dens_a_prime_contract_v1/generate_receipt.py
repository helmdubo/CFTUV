"""Генератор proof-only расписки DENS-A′/CONTRACT.

Файл намеренно не импортируется независимым верификатором. Он строит
канонические байты будущего typed preparation key из реальных V1-входов,
проверяет production OID до любых утверждений и фиксирует seam/OID/V1 факты.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from hashlib import sha256
import importlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Callable


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASE_COMMIT = "b5b97e43ee7ca4d30a540f42ff36cf01852c2911"
FIXTURE = ROOT / "kernel" / "fixtures" / "building_002_full_selection_v1"
RECEIPT = HERE / "dens_a_prime_contract_receipt.json"

EXPECTED_PRODUCT_OIDS = {
    "cftuv": "01f510088b50c425570cc2e42c13e400ca4875f3",
    "kernel/src": "ed33d3f43635a5acbe5b8695dfed9b2e12396c6b",
    "tools": "ec5439a5fc8ea46784d4621ac763f941b4833c36",
}
EXPECTED_BLOBS = {
    "cftuv/envelope_debug_session.py": (
        "3e69d6c2e201ea8f5cbe5508486b68681b2128ee"
    ),
    "cftuv/envelope_queue_export.py": (
        "92690a1f25076b40cd77082da9c746cf466a2d83"
    ),
    "kernel/src/cftuv_envelope/wavefront/conveyor.py": (
        "eac26a92a16f676375d75edbda7c62cb3219d1c0"
    ),
    "kernel/src/cftuv_envelope/contracts/request.py": (
        "88e09b704fafd5133e42d81b5ffe07071cfec206"
    ),
    "kernel/src/cftuv_envelope/reference/compile.py": (
        "51f46766e21a6e5aa47b766bc06b33ce26810271"
    ),
    "kernel/src/cftuv_envelope/codec.py": (
        "a921638fee6dda0e74dbe3181a22241db5850b86"
    ),
}
ANCHORS = (
    ("cftuv/envelope_debug_session.py", 32, 67, "compile-key evidence"),
    ("cftuv/envelope_debug_session.py", 125, 130, "current preparation key"),
    ("cftuv/envelope_debug_session.py", 430, 474, "cache construction"),
    ("cftuv/envelope_debug_session.py", 541, 555, "request-consuming seam"),
    ("cftuv/envelope_queue_export.py", 700, 736, "queue provider seam"),
    (
        "kernel/src/cftuv_envelope/wavefront/conveyor.py",
        995,
        1021,
        "kernel preparation entry",
    ),
)

SNAPSHOT_SHA256 = (
    "5c759cfc80548072eb3918f5fdc114d6f93ad077b70a5f0c663f10ede2f41d16"
)
REQUEST_SHA256 = (
    "40a039e7f239d88283317a461be0d7cd3e2f3679f018ab5223f2c3d50d6bfe7d"
)
COMPILATION_LENGTH = 262580
COMPILATION_SHA256 = (
    "db385afb41940d0eca68a16e54e12bd93f44d9517fd7cc2c24b259e46f3c4c84"
)

DENSITY_POLICY = "HUBER_EMANATED_VERTEX_FLOOR_V1"
DENSITY_PARAMETER = "LINEAR_REFLEX_MAX_SUBTURN_V1"
DENSITY_ROWS = (
    {
        "density": 0,
        "max_subturn_value_id": "LINEAR_REFLEX_MAX_SUBTURN_90_DEGREES_V1",
        "max_subturn_exact_value_symbol": "PI_OVER_2",
        "max_subturn_exact_turn_fraction": {
            "numerator": 1,
            "denominator": 2,
        },
    },
    {
        "density": 1,
        "max_subturn_value_id": (
            "LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1"
        ),
        "max_subturn_exact_value_symbol": "PI_OVER_3",
        "max_subturn_exact_turn_fraction": {
            "numerator": 1,
            "denominator": 3,
        },
    },
    {
        "density": 2,
        "max_subturn_value_id": "LINEAR_REFLEX_MAX_SUBTURN_45_DEGREES_V1",
        "max_subturn_exact_value_symbol": "PI_OVER_4",
        "max_subturn_exact_turn_fraction": {
            "numerator": 1,
            "denominator": 4,
        },
    },
    {
        "density": 3,
        "max_subturn_value_id": "LINEAR_REFLEX_MAX_SUBTURN_36_DEGREES_V1",
        "max_subturn_exact_value_symbol": "PI_OVER_5",
        "max_subturn_exact_turn_fraction": {
            "numerator": 1,
            "denominator": 5,
        },
    },
    {
        "density": 4,
        "max_subturn_value_id": "LINEAR_REFLEX_MAX_SUBTURN_30_DEGREES_V1",
        "max_subturn_exact_value_symbol": "PI_OVER_6",
        "max_subturn_exact_turn_fraction": {
            "numerator": 1,
            "denominator": 6,
        },
    },
)

KEY_FIELDS = (
    "$type",
    "schema_version",
    "source_revision_value",
    "patch_domain_id",
    "selected_chain_use_ids",
    "selected_physical_edge_ids",
    "compile_static_request.schema_version",
    "compile_static_request.decal_request_id",
    "compile_static_request.metric_space",
    "compile_static_request.angular_profile_family_id",
    "compile_static_request.angular_profile_selection_policy_id",
    "compile_static_request.max_subturn_parameter_id",
    "compile_static_request.max_subturn_value_id",
    "compile_static_request.max_subturn_exact_value.$type",
    "compile_static_request.max_subturn_exact_value.symbol",
    (
        "compile_static_request.max_subturn_exact_value."
        "turn_fraction.$type"
    ),
    (
        "compile_static_request.max_subturn_exact_value."
        "turn_fraction.numerator"
    ),
    (
        "compile_static_request.max_subturn_exact_value."
        "turn_fraction.denominator"
    ),
    "compile_static_request.cap_policy_id",
    "compile_static_request.boundary_policy_id",
    "compile_static_request.interaction_policy_id",
    "compile_static_request.ownership_policy_id",
    "compile_static_request.material_policy_id",
    "compile_static_request.uv_policy_id",
)
REQUEST_SCALAR_FIELDS = (
    "schema_version",
    "decal_request_id",
    "metric_space",
    "angular_profile_family_id",
    "angular_profile_selection_policy_id",
    "max_subturn_parameter_id",
    "max_subturn_value_id",
    "cap_policy_id",
    "boundary_policy_id",
    "interaction_policy_id",
    "ownership_policy_id",
    "material_policy_id",
    "uv_policy_id",
)
FORBIDDEN_COMPILE_STATIC_FIELDS = (
    "requested_alpha",
    "ui_density_integer",
)


class ProofReject(RuntimeError):
    """Именованный fail-closed исход proof-контракта."""

    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


def _run_git(*args: str) -> str:
    completed = subprocess.run(
        ("git", *args),
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def _git_oid(path: str) -> str:
    return _run_git("rev-parse", f"HEAD:{path}")


def _sha(payload: bytes) -> str:
    return sha256(payload).hexdigest()


def _canonical_key(value: dict) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _canonical_receipt(value: dict) -> bytes:
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            indent=2,
        )
        + "\n"
    ).encode("utf-8")


def _assert_product_tree(actual: dict[str, str]) -> None:
    if actual != EXPECTED_PRODUCT_OIDS:
        raise ProofReject("UNKNOWN_PRODUCT_TREE")


def _assert_blob_tree(actual: dict[str, str]) -> None:
    if actual != EXPECTED_BLOBS:
        raise ProofReject("UNKNOWN_SEAM_BLOB")


def _assert_sha(payload: bytes, expected: str, code: str) -> None:
    if _sha(payload) != expected:
        raise ProofReject(code)


def _assert_compilation(payload: bytes, length: int, digest: str) -> None:
    if len(payload) != length or _sha(payload) != digest:
        raise ProofReject("LEGACY_COMPILE_ANCHOR_CHANGED")


def _density_table(rows: tuple[dict, ...] = DENSITY_ROWS) -> dict[int, dict]:
    if tuple(item["density"] for item in rows) != (0, 1, 2, 3, 4):
        raise ProofReject("DENSITY_LEVEL_SET_INVALID")
    value_ids = [item["max_subturn_value_id"] for item in rows]
    symbols = [item["max_subturn_exact_value_symbol"] for item in rows]
    ratios = [
        (
            item["max_subturn_exact_turn_fraction"]["numerator"],
            item["max_subturn_exact_turn_fraction"]["denominator"],
        )
        for item in rows
    ]
    if len(set(value_ids)) != len(value_ids):
        raise ProofReject("DENSITY_VALUE_ID_COLLISION")
    if len(set(zip(value_ids, symbols, ratios, strict=True))) != len(value_ids):
        raise ProofReject("DENSITY_EXACT_AUTHORITY_COLLISION")
    if any(
        numerator != 1 or denominator != density + 2
        for density, (numerator, denominator) in enumerate(ratios)
    ):
        raise ProofReject("DENSITY_EXACT_VALUE_INVALID")
    return {item["density"]: dict(item) for item in rows}


def _require_nonempty(record: dict, field: str) -> str:
    if field not in record:
        raise ProofReject("COMPILE_STATIC_FIELD_MISSING")
    value = record[field]
    if not isinstance(value, str) or not value:
        raise ProofReject("COMPILE_STATIC_FIELD_INVALID")
    return value


def _build_key(
    authority: dict,
    density: int,
    *,
    density_override: dict | None = None,
    forbidden_extra: dict | None = None,
    ui_density_integer: int | None = None,
    requested_alpha: str | None = None,
) -> tuple[dict, bytes]:
    """Строит typed proof DTO; UI/alpha принимаются только как не-власть."""

    del ui_density_integer, requested_alpha
    extra = {} if forbidden_extra is None else dict(forbidden_extra)
    forbidden = sorted(set(extra) & set(FORBIDDEN_COMPILE_STATIC_FIELDS))
    if forbidden:
        if "requested_alpha" in forbidden:
            raise ProofReject("ALPHA_IS_NOT_COMPILE_STATIC")
        raise ProofReject("UI_INTEGER_IS_NOT_COMPILE_STATIC")
    if extra:
        raise ProofReject("UNDECLARED_COMPILE_STATIC_FIELD")

    density_rows = _density_table()
    if density not in density_rows:
        raise ProofReject("DENSITY_LEVEL_UNKNOWN")
    canonical_density = density_rows[density]
    density_authority = dict(canonical_density)
    if density_override is not None:
        for field, value in density_override.items():
            if value is None:
                density_authority.pop(field, None)
            else:
                density_authority[field] = value
    required_density = (
        "max_subturn_value_id",
        "max_subturn_exact_value_symbol",
        "max_subturn_exact_turn_fraction",
    )
    if any(field not in density_authority for field in required_density):
        raise ProofReject("DENSITY_AUTHORITY_FIELD_MISSING")
    if (
        authority.get("angular_profile_selection_policy_id")
        != DENSITY_POLICY
    ):
        raise ProofReject("DENSITY_SELECTION_POLICY_ID_UNKNOWN")
    if authority.get("max_subturn_parameter_id") != DENSITY_PARAMETER:
        raise ProofReject("DENSITY_PARAMETER_ID_UNKNOWN")
    if (
        density_authority["max_subturn_value_id"]
        != canonical_density["max_subturn_value_id"]
    ):
        raise ProofReject("DENSITY_VALUE_ID_UNKNOWN")
    if (
        density_authority["max_subturn_exact_value_symbol"]
        != canonical_density["max_subturn_exact_value_symbol"]
    ):
        raise ProofReject("DENSITY_EXACT_VALUE_MISMATCH")
    if (
        density_authority["max_subturn_exact_turn_fraction"]
        != canonical_density["max_subturn_exact_turn_fraction"]
    ):
        raise ProofReject("DENSITY_EXACT_VALUE_MISMATCH")

    request_authority = {}
    for field in REQUEST_SCALAR_FIELDS:
        if field == "max_subturn_value_id":
            value = density_authority[field]
        else:
            value = _require_nonempty(authority, field)
        request_authority[field] = value
    request_authority["max_subturn_exact_value"] = {
        "$type": "ExactAngleV2Proof",
        "symbol": density_authority["max_subturn_exact_value_symbol"],
        "turn_fraction": {
            "$type": "ExactRatioV1",
            **density_authority["max_subturn_exact_turn_fraction"],
        },
    }

    selected_uses = authority.get("selected_chain_use_ids")
    selected_edges = authority.get("selected_physical_edge_ids")
    if not isinstance(selected_uses, list) or not selected_uses:
        raise ProofReject("SELECTED_CHAIN_AUTHORITY_MISSING")
    if not isinstance(selected_edges, list) or not selected_edges:
        raise ProofReject("SELECTED_EDGE_AUTHORITY_MISSING")
    if len(set(selected_uses)) != len(selected_uses):
        raise ProofReject("SELECTED_CHAIN_AUTHORITY_DUPLICATE")
    if len(set(selected_edges)) != len(selected_edges):
        raise ProofReject("SELECTED_EDGE_AUTHORITY_DUPLICATE")

    key = {
        "$type": "ConveyorPreparationCacheKeyV2Proof",
        "schema_version": "cftuv.envelope.conveyor_preparation_cache_key.proof.v2",
        "source_revision_value": _require_nonempty(
            authority, "source_revision_value"
        ),
        "patch_domain_id": _require_nonempty(authority, "patch_domain_id"),
        "selected_chain_use_ids": sorted(selected_uses),
        "selected_physical_edge_ids": sorted(int(item) for item in selected_edges),
        "compile_static_request": request_authority,
    }
    payload = _canonical_key(key)
    if any(
        field.encode("utf-8") in payload
        for field in FORBIDDEN_COMPILE_STATIC_FIELDS
    ):
        raise ProofReject("FORBIDDEN_FIELD_LEAKED_TO_KEY")
    return key, payload


def _expect_reject(code: str, action: Callable[[], object]) -> dict:
    try:
        action()
    except ProofReject as exc:
        if exc.code != code:
            raise AssertionError(f"expected {code}, got {exc.code}") from exc
        return {"expected": code, "observed": exc.code, "status": "PASS"}
    raise AssertionError(f"expected named reject {code}")


def _line_anchor(path: str, start: int, end: int, purpose: str) -> dict:
    lines = (ROOT / path).read_text(encoding="utf-8").splitlines()
    if start <= 0 or end < start or end > len(lines):
        raise ProofReject("SEAM_LINE_RANGE_INVALID")
    text = "\n".join(lines[start - 1 : end]) + "\n"
    return {
        "path": path,
        "blob_oid": EXPECTED_BLOBS[path],
        "line_start": start,
        "line_end": end,
        "purpose": purpose,
        "normalized_lf_sha256": _sha(text.encode("utf-8")),
        "normalized_lf_text": text,
    }


def _kernel():
    sys.path.insert(0, str(ROOT / "kernel" / "src"))
    return importlib.import_module("cftuv_envelope")


def _legacy_facts() -> tuple[dict, dict, object, object, bytes, bytes]:
    kernel = _kernel()
    snapshot_bytes = (FIXTURE / "analysis_snapshot.json").read_bytes()
    request_bytes = (FIXTURE / "decal_request.json").read_bytes()
    _assert_sha(
        snapshot_bytes,
        SNAPSHOT_SHA256,
        "FROZEN_SNAPSHOT_SHA256_MISMATCH",
    )
    _assert_sha(
        request_bytes,
        REQUEST_SHA256,
        "FROZEN_REQUEST_SHA256_MISMATCH",
    )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    request = kernel.DecalRequestCodecV1.loads(request_bytes)
    if kernel.AnalysisSnapshotCodecV1.dumps(snapshot) != snapshot_bytes:
        raise ProofReject("SNAPSHOT_CODEC_NOT_BYTE_FIXED")
    if kernel.DecalRequestCodecV1.dumps(request) != request_bytes:
        raise ProofReject("REQUEST_CODEC_NOT_BYTE_FIXED")
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    if compiled.outcome.value != "EXACT" or compiled.compilation is None:
        raise ProofReject("LEGACY_COMPILE_NOT_EXACT")
    compilation_bytes = importlib.import_module(
        "cftuv_envelope.codec"
    ).canonical_json_bytes(compiled.compilation)
    _assert_compilation(
        compilation_bytes,
        COMPILATION_LENGTH,
        COMPILATION_SHA256,
    )
    angular = sorted(
        (
            item
            for item in compiled.compilation.envelope_specs
            if type(item).__name__ == "AngularEnvelopeSpec"
        ),
        key=lambda item: item.owner_sector_id.value,
    )
    selections = []
    for item in angular:
        selections.append(
            {
                "owner_sector_id": item.owner_sector_id.value,
                "selection_certificate_id": item.selection_certificate_id.value,
                "hidden_support_count": len(item.hidden_supports),
                "subturns": [
                    {
                        "ordinal": support.ordinal,
                        "numerator": support.turn_fraction.numerator,
                        "denominator": support.turn_fraction.denominator,
                    }
                    for support in item.hidden_supports
                ],
            }
        )
    if len(selections) != 4 or any(
        row["hidden_support_count"] != 1
        or row["subturns"]
        != [{"ordinal": 1, "numerator": 1, "denominator": 2}]
        for row in selections
    ):
        raise ProofReject("LEGACY_SELECTION_ANCHOR_CHANGED")
    return (
        json.loads(snapshot_bytes),
        json.loads(request_bytes),
        snapshot,
        request,
        snapshot_bytes,
        request_bytes,
    )


def _base_authority(snapshot_json: dict, request_json: dict) -> dict:
    manifest = json.loads((FIXTURE / "manifest.json").read_text("utf-8"))
    return {
        "source_revision_value": manifest["source_revision"],
        "patch_domain_id": manifest["patch_domain_ids"][0],
        "selected_chain_use_ids": [
            item["value"] for item in request_json["selected_chain_use_ids"]
        ],
        "selected_physical_edge_ids": manifest["selected_physical_edge_ids"],
        "schema_version": request_json["schema_version"],
        "decal_request_id": request_json["decal_request_id"]["value"],
        "metric_space": request_json["metric_space"],
        "angular_profile_family_id": request_json[
            "angular_profile_family_id"
        ],
        "angular_profile_selection_policy_id": DENSITY_POLICY,
        "max_subturn_parameter_id": DENSITY_PARAMETER,
        "cap_policy_id": request_json["cap_policy_id"],
        "boundary_policy_id": request_json["boundary_policy_id"],
        "interaction_policy_id": request_json["interaction_policy_id"],
        "ownership_policy_id": request_json["ownership_policy_id"],
        "material_policy_id": request_json["material_policy_id"]["value"],
        "uv_policy_id": request_json["uv_policy_id"]["value"],
    }


def _key_contract(authority: dict) -> dict:
    rows = []
    payload_by_density = {}
    for density in range(5):
        _, payload = _build_key(authority, density)
        payload_by_density[density] = payload
        rows.append(
            {
                "density": density,
                "canonical_utf8": payload.decode("utf-8"),
                "byte_length": len(payload),
                "sha256": _sha(payload),
            }
        )
    digests = [item["sha256"] for item in rows]
    if len(set(digests)) != 5:
        raise ProofReject("DENSITY_KEY_COLLISION")

    _, alpha_a = _build_key(authority, 1, requested_alpha="0.25")
    _, alpha_b = _build_key(authority, 1, requested_alpha="0.5")
    _, ui_a = _build_key(authority, 1, ui_density_integer=1)
    _, ui_b = _build_key(authority, 1, ui_density_integer=4)
    if alpha_a != alpha_b:
        raise ProofReject("ALPHA_CHANGED_COMPILE_STATIC_KEY")
    if ui_a != ui_b:
        raise ProofReject("UI_INTEGER_CHANGED_EXACT_AUTHORITY_KEY")

    mutation_specs = (
        ("source_revision_value", "proof-source-revision:alternate"),
        ("patch_domain_id", "proof:patch-domain:alternate"),
        (
            "selected_chain_use_ids",
            [*authority["selected_chain_use_ids"], "proof:chain-use:alternate"],
        ),
        (
            "selected_physical_edge_ids",
            [*authority["selected_physical_edge_ids"], 999],
        ),
        ("schema_version", "cftuv.envelope.decal_request.proof.v2"),
        ("decal_request_id", "proof:decal-request:alternate"),
        ("metric_space", "PROOF_ALTERNATE_METRIC_SPACE"),
        ("angular_profile_family_id", "PROOF_ALTERNATE_PROFILE_FAMILY"),
        ("cap_policy_id", "PROOF_ALTERNATE_CAP_POLICY"),
        ("boundary_policy_id", "PROOF_ALTERNATE_BOUNDARY_POLICY"),
        ("interaction_policy_id", "PROOF_ALTERNATE_INTERACTION_POLICY"),
        ("ownership_policy_id", "PROOF_ALTERNATE_OWNERSHIP_POLICY"),
        ("material_policy_id", "PROOF_ALTERNATE_MATERIAL_POLICY"),
        ("uv_policy_id", "PROOF_ALTERNATE_UV_POLICY"),
    )
    baseline_digest = _sha(payload_by_density[1])
    mutations = []
    for field, value in mutation_specs:
        changed = deepcopy(authority)
        changed[field] = value
        _, changed_payload = _build_key(changed, 1)
        changed_digest = _sha(changed_payload)
        if changed_digest == baseline_digest:
            raise ProofReject("COMPILE_STATIC_KEY_COLLISION")
        mutations.append(
            {
                "field": field,
                "baseline_sha256": baseline_digest,
                "mutated_sha256": changed_digest,
                "status": "DIFFERENT",
            }
        )

    missing = deepcopy(authority)
    del missing["cap_policy_id"]
    missing_density = {"max_subturn_exact_value_symbol": None}
    collided = list(deepcopy(DENSITY_ROWS))
    collided[4]["max_subturn_value_id"] = collided[3][
        "max_subturn_value_id"
    ]
    red_controls = {
        "unknown_selection_policy": _expect_reject(
            "DENSITY_SELECTION_POLICY_ID_UNKNOWN",
            lambda: _build_key(
                dict(
                    authority,
                    angular_profile_selection_policy_id="UNKNOWN_POLICY",
                ),
                1,
            ),
        ),
        "unknown_parameter_id": _expect_reject(
            "DENSITY_PARAMETER_ID_UNKNOWN",
            lambda: _build_key(
                dict(authority, max_subturn_parameter_id="UNKNOWN_PARAMETER"),
                1,
            ),
        ),
        "unknown_value_id": _expect_reject(
            "DENSITY_VALUE_ID_UNKNOWN",
            lambda: _build_key(
                authority,
                1,
                density_override={"max_subturn_value_id": "UNKNOWN_VALUE"},
            ),
        ),
        "mismatched_exact_symbol": _expect_reject(
            "DENSITY_EXACT_VALUE_MISMATCH",
            lambda: _build_key(
                authority,
                1,
                density_override={
                    "max_subturn_exact_value_symbol": "PI_OVER_4"
                },
            ),
        ),
        "mismatched_exact_fraction": _expect_reject(
            "DENSITY_EXACT_VALUE_MISMATCH",
            lambda: _build_key(
                authority,
                1,
                density_override={
                    "max_subturn_exact_turn_fraction": {
                        "numerator": 1,
                        "denominator": 4,
                    }
                },
            ),
        ),
        "missing_density_field": _expect_reject(
            "DENSITY_AUTHORITY_FIELD_MISSING",
            lambda: _build_key(
                authority,
                1,
                density_override=missing_density,
            ),
        ),
        "missing_compile_static_field": _expect_reject(
            "COMPILE_STATIC_FIELD_MISSING",
            lambda: _build_key(missing, 1),
        ),
        "alpha_included": _expect_reject(
            "ALPHA_IS_NOT_COMPILE_STATIC",
            lambda: _build_key(
                authority,
                1,
                forbidden_extra={"requested_alpha": "0.25"},
            ),
        ),
        "ui_integer_included": _expect_reject(
            "UI_INTEGER_IS_NOT_COMPILE_STATIC",
            lambda: _build_key(
                authority,
                1,
                forbidden_extra={"ui_density_integer": 1},
            ),
        ),
        "density_collision": _expect_reject(
            "DENSITY_VALUE_ID_COLLISION",
            lambda: _density_table(tuple(collided)),
        ),
    }
    return {
        "$type": "ConveyorPreparationCacheKeyContractProofV1",
        "key_type": "ConveyorPreparationCacheKeyV2Proof",
        "field_order": list(KEY_FIELDS),
        "canonicalization": (
            "UTF8_JSON_SORT_KEYS_COMPACT_NO_TRAILING_LF_V1"
        ),
        "excluded_non_authority": list(FORBIDDEN_COMPILE_STATIC_FIELDS),
        "density_authorities": [dict(item) for item in DENSITY_ROWS],
        "density_key_matrix": rows,
        "same_authority_matrix": [
            {
                "case": "alpha_0.25_to_0.5",
                "sha256_a": _sha(alpha_a),
                "sha256_b": _sha(alpha_b),
                "status": "SAME",
            },
            {
                "case": "ui_integer_1_to_4_exact_authority_unchanged",
                "sha256_a": _sha(ui_a),
                "sha256_b": _sha(ui_b),
                "status": "SAME",
            },
        ],
        "compile_static_mutation_matrix": mutations,
        "named_red_controls": red_controls,
        "decal_request_id_is_not_sufficient": {
            "shared_decal_request_id": authority["decal_request_id"],
            "distinct_density_key_count": len(set(digests)),
            "status": "PASS",
        },
    }


def _build_receipt() -> dict:
    product_oids = {path: _git_oid(path) for path in EXPECTED_PRODUCT_OIDS}
    _assert_product_tree(product_oids)
    blob_oids = {path: _git_oid(path) for path in EXPECTED_BLOBS}
    _assert_blob_tree(blob_oids)

    (
        snapshot_json,
        request_json,
        snapshot,
        request,
        snapshot_bytes,
        request_bytes,
    ) = _legacy_facts()
    authority = _base_authority(snapshot_json, request_json)
    key_contract = _key_contract(authority)

    kernel = _kernel()
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    compilation_bytes = importlib.import_module(
        "cftuv_envelope.codec"
    ).canonical_json_bytes(compiled.compilation)
    angular = sorted(
        (
            item
            for item in compiled.compilation.envelope_specs
            if type(item).__name__ == "AngularEnvelopeSpec"
        ),
        key=lambda item: item.owner_sector_id.value,
    )
    selection_rows = [
        {
            "owner_sector_id": item.owner_sector_id.value,
            "selection_certificate_id": item.selection_certificate_id.value,
            "hidden_support_count": len(item.hidden_supports),
            "subturn_count": len(item.hidden_supports) + 1,
            "hidden_turn_fractions": [
                (
                    f"{support.turn_fraction.numerator}/"
                    f"{support.turn_fraction.denominator}"
                )
                for support in item.hidden_supports
            ],
        }
        for item in angular
    ]

    simulated_unknown = dict(EXPECTED_PRODUCT_OIDS)
    simulated_unknown["cftuv"] = "0" * 40
    red_preclaim = {
        "unknown_product_tree": _expect_reject(
            "UNKNOWN_PRODUCT_TREE",
            lambda: _assert_product_tree(simulated_unknown),
        ),
        "corrupted_receipt_sha": _expect_reject(
            "RECEIPT_SHA256_MISMATCH",
            lambda: _assert_sha(
                b"receipt-corrupted",
                _sha(b"receipt-canonical"),
                "RECEIPT_SHA256_MISMATCH",
            ),
        ),
        "altered_snapshot_bytes": _expect_reject(
            "FROZEN_SNAPSHOT_SHA256_MISMATCH",
            lambda: _assert_sha(
                snapshot_bytes + b"\x00",
                SNAPSHOT_SHA256,
                "FROZEN_SNAPSHOT_SHA256_MISMATCH",
            ),
        ),
        "altered_request_bytes": _expect_reject(
            "FROZEN_REQUEST_SHA256_MISMATCH",
            lambda: _assert_sha(
                request_bytes + b"\x00",
                REQUEST_SHA256,
                "FROZEN_REQUEST_SHA256_MISMATCH",
            ),
        ),
        "changed_legacy_compile_anchor": _expect_reject(
            "LEGACY_COMPILE_ANCHOR_CHANGED",
            lambda: _assert_compilation(
                compilation_bytes,
                COMPILATION_LENGTH,
                "0" * 64,
            ),
        ),
    }
    max_key_bytes = max(
        item["byte_length"]
        for item in key_contract["density_key_matrix"]
    )
    max_integer_bits = max(
        int(item).bit_length()
        for item in authority["selected_physical_edge_ids"]
    )
    return {
        "$type": "DensAPrimeContractProofReceiptV1",
        "schema_version": "cftuv.envelope.dens_a_prime_contract_proof.v1",
        "scope": "PROOF_ONLY_NO_PRODUCT_AUTHORITY",
        "base_commit": BASE_COMMIT,
        "preclaim_gate": {
            "rule": "PRODUCT_TREE_AND_SEAM_BLOBS_MATCH_BEFORE_CLAIMS",
            "product_tree_oids": product_oids,
            "seam_blob_oids": blob_oids,
            "status": "PASS",
        },
        "real_seams": [
            _line_anchor(path, start, end, purpose)
            for path, start, end, purpose in ANCHORS
        ],
        "planned_key_contract": key_contract,
        "legacy_v1_60": {
            "$type": "LegacyV160ByteStabilityProofV1",
            "fixture": "building_002_full_selection_v1",
            "snapshot": {
                "sha256": _sha(snapshot_bytes),
                "byte_length": len(snapshot_bytes),
                "codec": "AnalysisSnapshotCodecV1",
                "decode_reencode_byte_fixed": True,
            },
            "request": {
                "sha256": _sha(request_bytes),
                "byte_length": len(request_bytes),
                "codec": "DecalRequestCodecV1",
                "decode_reencode_byte_fixed": True,
                "selection_policy": request_json[
                    "angular_profile_selection_policy_id"
                ],
                "max_subturn_value_id": request_json[
                    "max_subturn_value_id"
                ],
                "max_subturn_exact_value_symbol": request_json[
                    "max_subturn_exact_value"
                ]["symbol"],
            },
            "compile": {
                "entrypoint": "compile_reference_envelopes",
                "outcome": compiled.outcome.value,
                "canonical_byte_length": len(compilation_bytes),
                "canonical_sha256": _sha(compilation_bytes),
                "angular_selection_count": len(selection_rows),
                "selections": selection_rows,
            },
            "commands": [
                (
                    "PYTHONSAFEPATH=1 python "
                    "kernel/fixtures/dens_a_prime_contract_v1/"
                    "verify_contract.py"
                ),
                (
                    "PYTHONSAFEPATH=1 python "
                    "kernel/fixtures/dens_a_prime_contract_v1/"
                    "generate_receipt.py --check"
                ),
            ],
        },
        "falsifiability": {
            "preclaim_named_red_controls": red_preclaim,
            "key_named_red_controls": key_contract["named_red_controls"],
            "all_expected_reject": True,
        },
        "determinism": {
            "receipt_encoding": "UTF8_JSON_SORT_KEYS_INDENT_2_TRAILING_LF_V1",
            "wall_clock_excluded_from_canonical_bytes": True,
            "independent_generator_and_verifier_modules": True,
        },
        "deterministic_budgets": {
            "max_key_bytes": max_key_bytes,
            "max_integer_bits": max_integer_bits,
            "legacy_compilation_bytes": len(compilation_bytes),
            "source_blob_count": len(blob_oids),
            "seam_anchor_count": len(ANCHORS),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--digest", action="store_true")
    args = parser.parse_args()
    if sum((args.write, args.check, args.digest)) != 1:
        parser.error("choose exactly one of --write, --check, --digest")
    payload = _canonical_receipt(_build_receipt())
    if args.write:
        RECEIPT.write_bytes(payload)
        print(f"WROTE {RECEIPT} {len(payload)} {_sha(payload)}")
    elif args.check:
        actual = RECEIPT.read_bytes()
        if actual != payload:
            raise ProofReject("RECEIPT_REGENERATION_MISMATCH")
        print(f"REGEN_MATCH {len(payload)} {_sha(payload)}")
    else:
        print(f"{len(payload)} {_sha(payload)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
