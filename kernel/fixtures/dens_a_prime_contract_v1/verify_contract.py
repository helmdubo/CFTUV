"""Независимый верификатор расписки DENS-A′/CONTRACT.

Не импортирует generate_receipt.py и не доверяет его helper-функциям:
сам читает receipt bytes, production blobs, V1 fixture и публичные codecs.
"""

from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
import importlib
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Callable


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
RECEIPT = HERE / "dens_a_prime_contract_receipt.json"
RECEIPT_SHA256 = (
    "74914d4ea5b1e2143aab387f4e47189138c97e40f87a9a257da25bbae312c48a"
)
BASE_COMMIT = "b5b97e43ee7ca4d30a540f42ff36cf01852c2911"
FIXTURE = ROOT / "kernel" / "fixtures" / "building_002_full_selection_v1"

PRODUCT_OIDS = {
    "cftuv": "01f510088b50c425570cc2e42c13e400ca4875f3",
    "kernel/src": "ed33d3f43635a5acbe5b8695dfed9b2e12396c6b",
    "tools": "ec5439a5fc8ea46784d4621ac763f941b4833c36",
}
BLOB_OIDS = {
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
    ("cftuv/envelope_debug_session.py", 32, 67),
    ("cftuv/envelope_debug_session.py", 125, 130),
    ("cftuv/envelope_debug_session.py", 430, 474),
    ("cftuv/envelope_debug_session.py", 541, 555),
    ("cftuv/envelope_queue_export.py", 700, 736),
    ("kernel/src/cftuv_envelope/wavefront/conveyor.py", 995, 1021),
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
    (
        0,
        "LINEAR_REFLEX_MAX_SUBTURN_90_DEGREES_V1",
        "PI_OVER_2",
    ),
    (
        1,
        "LINEAR_REFLEX_MAX_SUBTURN_60_DEGREES_V1",
        "PI_OVER_3",
    ),
    (
        2,
        "LINEAR_REFLEX_MAX_SUBTURN_45_DEGREES_V1",
        "PI_OVER_4",
    ),
    (
        3,
        "LINEAR_REFLEX_MAX_SUBTURN_36_DEGREES_V1",
        "PI_OVER_5",
    ),
    (
        4,
        "LINEAR_REFLEX_MAX_SUBTURN_30_DEGREES_V1",
        "PI_OVER_6",
    ),
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
    "compile_static_request.max_subturn_exact_value_symbol",
    "compile_static_request.cap_policy_id",
    "compile_static_request.boundary_policy_id",
    "compile_static_request.interaction_policy_id",
    "compile_static_request.ownership_policy_id",
    "compile_static_request.material_policy_id",
    "compile_static_request.uv_policy_id",
)
REQUEST_FIELDS = tuple(
    item.removeprefix("compile_static_request.")
    for item in KEY_FIELDS
    if item.startswith("compile_static_request.")
)


class VerifyReject(RuntimeError):
    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


def _sha(payload: bytes) -> str:
    return sha256(payload).hexdigest()


def _run_git(*args: str) -> str:
    result = subprocess.run(
        ("git", *args),
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _actual_oids(paths: dict[str, str]) -> dict[str, str]:
    return {
        path: _run_git("rev-parse", f"HEAD:{path}")
        for path in paths
    }


def _require_tree(actual: dict[str, str]) -> None:
    if actual != PRODUCT_OIDS:
        raise VerifyReject("UNKNOWN_PRODUCT_TREE")


def _require_blobs(actual: dict[str, str]) -> None:
    if actual != BLOB_OIDS:
        raise VerifyReject("UNKNOWN_SEAM_BLOB")


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise VerifyReject("DUPLICATE_JSON_KEY")
        result[key] = value
    return result


def _parse(payload: bytes | str):
    text = payload.decode("utf-8") if isinstance(payload, bytes) else payload
    return json.loads(text, object_pairs_hook=_unique_pairs)


def _key_bytes(value: dict) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _receipt_bytes(value: dict) -> bytes:
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


def _density_map(rows=DENSITY_ROWS) -> dict[int, tuple[str, str]]:
    if tuple(item[0] for item in rows) != (0, 1, 2, 3, 4):
        raise VerifyReject("DENSITY_LEVEL_SET_INVALID")
    value_ids = [item[1] for item in rows]
    pairs = [(item[1], item[2]) for item in rows]
    if len(set(value_ids)) != 5:
        raise VerifyReject("DENSITY_VALUE_ID_COLLISION")
    if len(set(pairs)) != 5:
        raise VerifyReject("DENSITY_EXACT_AUTHORITY_COLLISION")
    return {item[0]: (item[1], item[2]) for item in rows}


def _nonempty(authority: dict, field: str) -> str:
    if field not in authority:
        raise VerifyReject("COMPILE_STATIC_FIELD_MISSING")
    value = authority[field]
    if not isinstance(value, str) or not value:
        raise VerifyReject("COMPILE_STATIC_FIELD_INVALID")
    return value


def _independent_key(
    authority: dict,
    density: int,
    *,
    value_override: str | None = None,
    symbol_override: str | None = None,
    delete_symbol: bool = False,
    extras: dict | None = None,
    requested_alpha: str | None = None,
    ui_density_integer: int | None = None,
) -> bytes:
    del requested_alpha, ui_density_integer
    extras = {} if extras is None else dict(extras)
    if "requested_alpha" in extras:
        raise VerifyReject("ALPHA_IS_NOT_COMPILE_STATIC")
    if "ui_density_integer" in extras:
        raise VerifyReject("UI_INTEGER_IS_NOT_COMPILE_STATIC")
    if extras:
        raise VerifyReject("UNDECLARED_COMPILE_STATIC_FIELD")
    mapping = _density_map()
    if density not in mapping:
        raise VerifyReject("DENSITY_LEVEL_UNKNOWN")
    if authority.get("angular_profile_selection_policy_id") != DENSITY_POLICY:
        raise VerifyReject("DENSITY_SELECTION_POLICY_ID_UNKNOWN")
    if authority.get("max_subturn_parameter_id") != DENSITY_PARAMETER:
        raise VerifyReject("DENSITY_PARAMETER_ID_UNKNOWN")
    expected_value, expected_symbol = mapping[density]
    actual_value = expected_value if value_override is None else value_override
    if actual_value != expected_value:
        raise VerifyReject("DENSITY_VALUE_ID_UNKNOWN")
    if delete_symbol:
        raise VerifyReject("DENSITY_AUTHORITY_FIELD_MISSING")
    actual_symbol = (
        expected_symbol if symbol_override is None else symbol_override
    )
    if actual_symbol != expected_symbol:
        raise VerifyReject("DENSITY_EXACT_VALUE_MISMATCH")

    selected_uses = authority.get("selected_chain_use_ids")
    selected_edges = authority.get("selected_physical_edge_ids")
    if not isinstance(selected_uses, list) or not selected_uses:
        raise VerifyReject("SELECTED_CHAIN_AUTHORITY_MISSING")
    if not isinstance(selected_edges, list) or not selected_edges:
        raise VerifyReject("SELECTED_EDGE_AUTHORITY_MISSING")
    if len(set(selected_uses)) != len(selected_uses):
        raise VerifyReject("SELECTED_CHAIN_AUTHORITY_DUPLICATE")
    if len(set(selected_edges)) != len(selected_edges):
        raise VerifyReject("SELECTED_EDGE_AUTHORITY_DUPLICATE")

    request_part = {}
    for field in REQUEST_FIELDS:
        if field == "max_subturn_value_id":
            request_part[field] = actual_value
        elif field == "max_subturn_exact_value_symbol":
            request_part[field] = actual_symbol
        else:
            request_part[field] = _nonempty(authority, field)
    value = {
        "$type": "ConveyorPreparationCacheKeyV2Proof",
        "schema_version": "cftuv.envelope.conveyor_preparation_cache_key.proof.v2",
        "source_revision_value": _nonempty(
            authority, "source_revision_value"
        ),
        "patch_domain_id": _nonempty(authority, "patch_domain_id"),
        "selected_chain_use_ids": sorted(selected_uses),
        "selected_physical_edge_ids": sorted(int(item) for item in selected_edges),
        "compile_static_request": request_part,
    }
    payload = _key_bytes(value)
    if (
        b"requested_alpha" in payload
        or b"ui_density_integer" in payload
    ):
        raise VerifyReject("FORBIDDEN_FIELD_LEAKED_TO_KEY")
    return payload


def _expect(code: str, action: Callable[[], object]) -> str:
    try:
        action()
    except VerifyReject as exc:
        if exc.code != code:
            raise AssertionError(f"expected {code}, got {exc.code}") from exc
        return exc.code
    raise AssertionError(f"expected named reject {code}")


def _base_authority(request_json: dict) -> dict:
    manifest = _parse((FIXTURE / "manifest.json").read_bytes())
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


def _verify_seams(receipt: dict) -> None:
    seam_rows = receipt["real_seams"]
    if len(seam_rows) != len(ANCHORS):
        raise AssertionError("seam anchor count mismatch")
    indexed = {
        (row["path"], row["line_start"], row["line_end"]): row
        for row in seam_rows
    }
    if tuple(indexed) != ANCHORS:
        raise AssertionError("seam anchor identity/order mismatch")
    for path, start, end in ANCHORS:
        row = indexed[(path, start, end)]
        if row["blob_oid"] != BLOB_OIDS[path]:
            raise AssertionError(f"blob mismatch: {path}")
        lines = (ROOT / path).read_text("utf-8").splitlines()
        text = "\n".join(lines[start - 1 : end]) + "\n"
        if row["normalized_lf_text"] != text:
            raise AssertionError(f"line text mismatch: {path}:{start}-{end}")
        if row["normalized_lf_sha256"] != _sha(text.encode("utf-8")):
            raise AssertionError(f"line digest mismatch: {path}:{start}-{end}")


def _verify_key_contract(receipt: dict, request_json: dict) -> tuple[int, int]:
    contract = receipt["planned_key_contract"]
    if tuple(contract["field_order"]) != KEY_FIELDS:
        raise AssertionError("typed key field set/order mismatch")
    if tuple(contract["excluded_non_authority"]) != (
        "requested_alpha",
        "ui_density_integer",
    ):
        raise AssertionError("key exclusions mismatch")
    expected_rows = [
        {
            "density": density,
            "max_subturn_value_id": value_id,
            "max_subturn_exact_value_symbol": symbol,
        }
        for density, value_id, symbol in DENSITY_ROWS
    ]
    if contract["density_authorities"] != expected_rows:
        raise AssertionError("density typed authority rows mismatch")

    authority = _base_authority(request_json)
    receipt_rows = contract["density_key_matrix"]
    if [row["density"] for row in receipt_rows] != list(range(5)):
        raise AssertionError("density matrix identity mismatch")
    observed = []
    max_bytes = 0
    for row in receipt_rows:
        payload = _independent_key(authority, row["density"])
        max_bytes = max(max_bytes, len(payload))
        if row["canonical_utf8"].encode("utf-8") != payload:
            raise AssertionError("canonical typed key bytes mismatch")
        parsed = _parse(payload)
        if _key_bytes(parsed) != payload:
            raise AssertionError("typed key is not canonical")
        if row["byte_length"] != len(payload):
            raise AssertionError("typed key byte length mismatch")
        if row["sha256"] != _sha(payload):
            raise AssertionError("typed key digest mismatch")
        observed.append(row["sha256"])
    if len(set(observed)) != 5:
        raise AssertionError("d0..d4 key collision")

    alpha_a = _independent_key(
        authority, 1, requested_alpha="0.25"
    )
    alpha_b = _independent_key(
        authority, 1, requested_alpha="0.5"
    )
    ui_a = _independent_key(authority, 1, ui_density_integer=1)
    ui_b = _independent_key(authority, 1, ui_density_integer=4)
    same_rows = contract["same_authority_matrix"]
    expected_same = (
        ("alpha_0.25_to_0.5", alpha_a, alpha_b),
        (
            "ui_integer_1_to_4_exact_authority_unchanged",
            ui_a,
            ui_b,
        ),
    )
    for row, (name, first, second) in zip(
        same_rows, expected_same, strict=True
    ):
        if first != second or row != {
            "case": name,
            "sha256_a": _sha(first),
            "sha256_b": _sha(second),
            "status": "SAME",
        }:
            raise AssertionError(f"same-authority matrix mismatch: {name}")

    baseline = _independent_key(authority, 1)
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
    expected_mutations = []
    for field, new_value in mutation_specs:
        changed = deepcopy(authority)
        changed[field] = new_value
        payload = _independent_key(changed, 1)
        if payload == baseline:
            raise AssertionError(f"compile-static collision: {field}")
        expected_mutations.append(
            {
                "field": field,
                "baseline_sha256": _sha(baseline),
                "mutated_sha256": _sha(payload),
                "status": "DIFFERENT",
            }
        )
    if contract["compile_static_mutation_matrix"] != expected_mutations:
        raise AssertionError("compile-static mutation matrix mismatch")

    missing = deepcopy(authority)
    del missing["cap_policy_id"]
    collision_rows = list(DENSITY_ROWS)
    collision_rows[4] = (
        4,
        collision_rows[3][1],
        collision_rows[4][2],
    )
    observed_red = {
        "unknown_selection_policy": _expect(
            "DENSITY_SELECTION_POLICY_ID_UNKNOWN",
            lambda: _independent_key(
                dict(
                    authority,
                    angular_profile_selection_policy_id="UNKNOWN_POLICY",
                ),
                1,
            ),
        ),
        "unknown_parameter_id": _expect(
            "DENSITY_PARAMETER_ID_UNKNOWN",
            lambda: _independent_key(
                dict(authority, max_subturn_parameter_id="UNKNOWN_PARAMETER"),
                1,
            ),
        ),
        "unknown_value_id": _expect(
            "DENSITY_VALUE_ID_UNKNOWN",
            lambda: _independent_key(
                authority, 1, value_override="UNKNOWN_VALUE"
            ),
        ),
        "mismatched_exact_symbol": _expect(
            "DENSITY_EXACT_VALUE_MISMATCH",
            lambda: _independent_key(
                authority, 1, symbol_override="PI_OVER_4"
            ),
        ),
        "missing_density_field": _expect(
            "DENSITY_AUTHORITY_FIELD_MISSING",
            lambda: _independent_key(
                authority, 1, delete_symbol=True
            ),
        ),
        "missing_compile_static_field": _expect(
            "COMPILE_STATIC_FIELD_MISSING",
            lambda: _independent_key(missing, 1),
        ),
        "alpha_included": _expect(
            "ALPHA_IS_NOT_COMPILE_STATIC",
            lambda: _independent_key(
                authority, 1, extras={"requested_alpha": "0.25"}
            ),
        ),
        "ui_integer_included": _expect(
            "UI_INTEGER_IS_NOT_COMPILE_STATIC",
            lambda: _independent_key(
                authority, 1, extras={"ui_density_integer": 1}
            ),
        ),
        "density_collision": _expect(
            "DENSITY_VALUE_ID_COLLISION",
            lambda: _density_map(tuple(collision_rows)),
        ),
    }
    receipt_red = contract["named_red_controls"]
    for name, code in observed_red.items():
        if receipt_red[name] != {
            "expected": code,
            "observed": code,
            "status": "PASS",
        }:
            raise AssertionError(f"red control mismatch: {name}")
    insufficiency = contract["decal_request_id_is_not_sufficient"]
    if (
        insufficiency["shared_decal_request_id"]
        != authority["decal_request_id"]
        or insufficiency["distinct_density_key_count"] != 5
        or insufficiency["status"] != "PASS"
    ):
        raise AssertionError("DecalRequestId insufficiency fact mismatch")
    max_bits = max(
        int(item).bit_length()
        for item in authority["selected_physical_edge_ids"]
    )
    return max_bytes, max_bits


def _kernel():
    sys.path.insert(0, str(ROOT / "kernel" / "src"))
    return importlib.import_module("cftuv_envelope")


def _verify_legacy(receipt: dict) -> tuple[dict, int]:
    kernel = _kernel()
    snapshot_bytes = (FIXTURE / "analysis_snapshot.json").read_bytes()
    request_bytes = (FIXTURE / "decal_request.json").read_bytes()
    if _sha(snapshot_bytes) != SNAPSHOT_SHA256:
        raise VerifyReject("FROZEN_SNAPSHOT_SHA256_MISMATCH")
    if _sha(request_bytes) != REQUEST_SHA256:
        raise VerifyReject("FROZEN_REQUEST_SHA256_MISMATCH")
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    request = kernel.DecalRequestCodecV1.loads(request_bytes)
    if kernel.AnalysisSnapshotCodecV1.dumps(snapshot) != snapshot_bytes:
        raise VerifyReject("SNAPSHOT_CODEC_NOT_BYTE_FIXED")
    if kernel.DecalRequestCodecV1.dumps(request) != request_bytes:
        raise VerifyReject("REQUEST_CODEC_NOT_BYTE_FIXED")
    result = kernel.compile_reference_envelopes(snapshot, request)
    if result.outcome.value != "EXACT" or result.compilation is None:
        raise VerifyReject("LEGACY_COMPILE_NOT_EXACT")
    compilation_bytes = importlib.import_module(
        "cftuv_envelope.codec"
    ).canonical_json_bytes(result.compilation)
    if (
        len(compilation_bytes) != COMPILATION_LENGTH
        or _sha(compilation_bytes) != COMPILATION_SHA256
    ):
        raise VerifyReject("LEGACY_COMPILE_ANCHOR_CHANGED")
    angular = sorted(
        (
            item
            for item in result.compilation.envelope_specs
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
    if len(selection_rows) != 4 or any(
        item["hidden_support_count"] != 1
        or item["subturn_count"] != 2
        or item["hidden_turn_fractions"] != ["1/2"]
        for item in selection_rows
    ):
        raise VerifyReject("LEGACY_SELECTION_ANCHOR_CHANGED")

    legacy = receipt["legacy_v1_60"]
    request_json = _parse(request_bytes)
    expected = {
        "$type": "LegacyV160ByteStabilityProofV1",
        "fixture": "building_002_full_selection_v1",
        "snapshot": {
            "sha256": SNAPSHOT_SHA256,
            "byte_length": len(snapshot_bytes),
            "codec": "AnalysisSnapshotCodecV1",
            "decode_reencode_byte_fixed": True,
        },
        "request": {
            "sha256": REQUEST_SHA256,
            "byte_length": len(request_bytes),
            "codec": "DecalRequestCodecV1",
            "decode_reencode_byte_fixed": True,
            "selection_policy": request_json[
                "angular_profile_selection_policy_id"
            ],
            "max_subturn_value_id": request_json["max_subturn_value_id"],
            "max_subturn_exact_value_symbol": request_json[
                "max_subturn_exact_value"
            ]["symbol"],
        },
        "compile": {
            "entrypoint": "compile_reference_envelopes",
            "outcome": "EXACT",
            "canonical_byte_length": COMPILATION_LENGTH,
            "canonical_sha256": COMPILATION_SHA256,
            "angular_selection_count": 4,
            "selections": selection_rows,
        },
        "commands": legacy["commands"],
    }
    if legacy != expected:
        raise AssertionError("legacy V1/60 receipt facts mismatch")
    return request_json, len(compilation_bytes)


def _verify_red_controls(receipt: dict) -> None:
    wrong_tree = dict(PRODUCT_OIDS)
    wrong_tree["cftuv"] = "0" * 40
    snapshot = (FIXTURE / "analysis_snapshot.json").read_bytes()
    request = (FIXTURE / "decal_request.json").read_bytes()
    observed = {
        "unknown_product_tree": _expect(
            "UNKNOWN_PRODUCT_TREE", lambda: _require_tree(wrong_tree)
        ),
        "corrupted_receipt_sha": _expect(
            "RECEIPT_SHA256_MISMATCH",
            lambda: (
                None
                if _sha(b"receipt-corrupted") == _sha(b"receipt-canonical")
                else (_ for _ in ()).throw(
                    VerifyReject("RECEIPT_SHA256_MISMATCH")
                )
            ),
        ),
        "altered_snapshot_bytes": _expect(
            "FROZEN_SNAPSHOT_SHA256_MISMATCH",
            lambda: (
                None
                if _sha(snapshot + b"\x00") == SNAPSHOT_SHA256
                else (_ for _ in ()).throw(
                    VerifyReject("FROZEN_SNAPSHOT_SHA256_MISMATCH")
                )
            ),
        ),
        "altered_request_bytes": _expect(
            "FROZEN_REQUEST_SHA256_MISMATCH",
            lambda: (
                None
                if _sha(request + b"\x00") == REQUEST_SHA256
                else (_ for _ in ()).throw(
                    VerifyReject("FROZEN_REQUEST_SHA256_MISMATCH")
                )
            ),
        ),
        "changed_legacy_compile_anchor": _expect(
            "LEGACY_COMPILE_ANCHOR_CHANGED",
            lambda: (_ for _ in ()).throw(
                VerifyReject("LEGACY_COMPILE_ANCHOR_CHANGED")
            ),
        ),
    }
    receipt_rows = receipt["falsifiability"][
        "preclaim_named_red_controls"
    ]
    for name, code in observed.items():
        if receipt_rows[name] != {
            "expected": code,
            "observed": code,
            "status": "PASS",
        }:
            raise AssertionError(f"preclaim red mismatch: {name}")


def main() -> int:
    started = time.perf_counter()
    # Это первая проверка: на неизвестном product tree расписка не читается.
    actual_product = _actual_oids(PRODUCT_OIDS)
    _require_tree(actual_product)
    actual_blobs = _actual_oids(BLOB_OIDS)
    _require_blobs(actual_blobs)

    payload = RECEIPT.read_bytes()
    if _sha(payload) != RECEIPT_SHA256:
        raise VerifyReject("RECEIPT_SHA256_MISMATCH")
    receipt = _parse(payload)
    if _receipt_bytes(receipt) != payload:
        raise VerifyReject("RECEIPT_BYTES_NOT_CANONICAL")
    if (
        receipt["$type"] != "DensAPrimeContractProofReceiptV1"
        or receipt["base_commit"] != BASE_COMMIT
        or receipt["scope"] != "PROOF_ONLY_NO_PRODUCT_AUTHORITY"
    ):
        raise AssertionError("receipt identity mismatch")
    if (
        receipt["preclaim_gate"]["product_tree_oids"] != PRODUCT_OIDS
        or receipt["preclaim_gate"]["seam_blob_oids"] != BLOB_OIDS
        or receipt["preclaim_gate"]["status"] != "PASS"
    ):
        raise AssertionError("receipt preclaim gate mismatch")
    _verify_seams(receipt)
    request_json, compilation_bytes = _verify_legacy(receipt)
    max_key_bytes, max_bits = _verify_key_contract(receipt, request_json)
    _verify_red_controls(receipt)
    budgets = receipt["deterministic_budgets"]
    if budgets != {
        "max_key_bytes": max_key_bytes,
        "max_integer_bits": max_bits,
        "legacy_compilation_bytes": compilation_bytes,
        "source_blob_count": len(BLOB_OIDS),
        "seam_anchor_count": len(ANCHORS),
    }:
        raise AssertionError("deterministic budget mismatch")
    elapsed = time.perf_counter() - started
    print(
        json.dumps(
            {
                "status": "PASS",
                "receipt_sha256": RECEIPT_SHA256,
                "density_keys_distinct": 5,
                "same_authority_cases": 2,
                "compile_static_mutations": 14,
                "key_named_red_controls": 9,
                "preclaim_named_red_controls": 5,
                "legacy_angular_selections": 4,
                "max_key_bytes": max_key_bytes,
                "max_integer_bits": max_bits,
                "wall_seconds": round(elapsed, 6),
            },
            sort_keys=True,
            separators=(",", ":"),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
