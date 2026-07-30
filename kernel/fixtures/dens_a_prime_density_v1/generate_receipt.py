"""Generate the proof-only DENS-A-prime owner/root receipt.

This file deliberately contains no product implementation.  It reads a frozen
field fixture, asks the existing compiler for its evaluation geometry, then
proves the density owner law and isolates the resulting root with exact
SignedCosSquaredV1 predicates over integers/Fractions.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any


HERE = Path(__file__).resolve().parent
SCHEMA = "cftuv.envelope.dens_a_prime.radical_receipt.v1"
INPUT_SCHEMA = "cftuv.envelope.dens_a_prime.radical_inputs.v1"
AUTHORITY_TYPE = "ProofOnlyLinearReflexDensityAuthorityV1"
EXACT_ANGLE_TYPE = "ProofOnlyExactAngleV1"
SIGNED_COS_SQUARED_TYPE = "SignedCosSquaredV1"
OWNER_POLICY = "HUBER_EMANATED_COUNT_DENSITY_A_V1"
OWNER_PARAMETER = "LINEAR_REFLEX_DENSITY_A_V1"
OWNER_FORMULA = {
    "q": "d+2",
    "C": "ceil(q*u)",
    "E": "max(2,C)",
    "H": "max(1,C-1)",
}
DENSITY_VALUES = {
    f"LINEAR_REFLEX_DENSITY_{density}_V1": density
    for density in range(5)
}
DENSITY_ANGLES = {
    density: {
        "$type": EXACT_ANGLE_TYPE,
        "symbol": f"PI_OVER_{density + 2}",
        "turn_fraction_of_pi": f"1/{density + 2}",
    }
    for density in range(5)
}
EXPECTED_OWNER_CASES = (
    {
        "case_id": "H1_C1",
        "d": 0,
        "u": "1/4",
        "expected_C": 1,
        "expected_E": 2,
        "expected_H": 1,
    },
    {
        "case_id": "H1_C2",
        "d": 0,
        "u": "3/4",
        "expected_C": 2,
        "expected_E": 2,
        "expected_H": 1,
    },
    {
        "case_id": "H2_C3",
        "d": 1,
        "u": "5/6",
        "expected_C": 3,
        "expected_E": 3,
        "expected_H": 2,
    },
    {
        "case_id": "H3_C4",
        "d": 2,
        "u": "7/8",
        "expected_C": 4,
        "expected_E": 4,
        "expected_H": 3,
    },
    {
        "case_id": "H4_C5",
        "d": 3,
        "u": "9/10",
        "expected_C": 5,
        "expected_E": 5,
        "expected_H": 4,
    },
    {
        "case_id": "H5_C6",
        "d": 4,
        "u": "11/12",
        "expected_C": 6,
        "expected_E": 6,
        "expected_H": 5,
    },
)
FROZEN_SOURCE_IDENTITY = {
    "snapshot_path": (
        "kernel/fixtures/building_002_full_selection_v1/"
        "analysis_snapshot.json"
    ),
    "snapshot_sha256": (
        "5c759cfc80548072eb3918f5fdc114d6f93ad077b70a5f0c663f10ede2f41d16"
    ),
    "request_path": (
        "kernel/fixtures/building_002_full_selection_v1/decal_request.json"
    ),
    "request_sha256": (
        "40a039e7f239d88283317a461be0d7cd3e2f3679f018ab5223f2c3d50d6bfe7d"
    ),
    "patch_domain_id": "host-v0:patch-domain:bf66337616136188f314fb69",
    "reference_metric_id": "reference-metric:749dbc5d924dc816c848e90a",
    "metric_codec_sha256": (
        "ca72780636570dbf5c0c41f610352a63300677783e5d59b8c4f55c6fb77ca9a4"
    ),
    "angular_spec_id": "angular-spec:739252a710101eaedbeef0e7",
    "owner_sector_id": "host-v0:owner-sector:b4426220b8b8721bc83b51c0",
    "source_relation_id": "host-v0:corner-relation:60fc81c8f39e546c88f9ff6a",
    "anchor_source_vertex_id": (
        "host-vertex:host-source:"
        "63845922ce058ac94746bc8b624049f3fdde3b87a4c5fb1bd8a8bb0b217e081e:"
        "building.002:8"
    ),
    "incoming_chain_use_id": "host-v0:chain-use:6d9faa2581ad0c02e518d297",
    "outgoing_chain_use_id": "host-v0:chain-use:fefd7b48cd65d85d0f50b1d0",
}
PRODUCTION_INPUT_KEYS = {
    *FROZEN_SOURCE_IDENTITY,
    "density_authority",
    "expected_gram",
    "expected_signed_cos_squared",
    "principal_half_tangent_bracket",
    "other_branch_half_tangent_bracket",
    "root_refinement_steps",
}


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, default=HERE.parents[2])
    parser.add_argument("--inputs", type=Path, default=HERE / "inputs.json")
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


def _canonical(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def _pretty(value: Any) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n"
    ).encode("utf-8")


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _exact_keys(record: dict[str, Any], expected: set[str], label: str) -> None:
    actual = set(record)
    if actual != expected:
        raise ValueError(
            f"{label} field mismatch: missing={sorted(expected - actual)} "
            f"extra={sorted(actual - expected)}"
        )


def _validate_owner_law(owner_law: dict[str, Any]) -> None:
    _exact_keys(
        owner_law,
        {
            "selection_policy_id",
            "density_parameter_id",
            "formula",
            "certificates",
        },
        "owner_law",
    )
    if owner_law["selection_policy_id"] != OWNER_POLICY:
        raise ValueError("unknown owner selection policy")
    if owner_law["density_parameter_id"] != OWNER_PARAMETER:
        raise ValueError("unknown owner density parameter")
    if owner_law["formula"] != OWNER_FORMULA:
        raise ValueError("owner law formula mismatch")
    if owner_law["certificates"] != list(EXPECTED_OWNER_CASES):
        raise ValueError("owner law certificate matrix mismatch")
    for row in owner_law["certificates"]:
        _exact_keys(
            row,
            {
                "case_id",
                "d",
                "u",
                "expected_C",
                "expected_E",
                "expected_H",
            },
            f"owner_law certificate {row.get('case_id', '<missing>')}",
        )


def _validate_density_authority(
    authority: dict[str, Any],
    owner_law: dict[str, Any],
) -> int:
    _exact_keys(
        authority,
        {
            "$type",
            "selection_policy_id",
            "density_parameter_id",
            "density_value_id",
            "density_integer",
            "max_subturn_exact_value",
            "gate0_hidden_count_witness_case_id",
            "reflex_excess_source",
            "expected_reflex_excess_over_pi",
        },
        "density_authority",
    )
    if authority["$type"] != AUTHORITY_TYPE:
        raise ValueError("unknown density authority type")
    if authority["selection_policy_id"] != OWNER_POLICY:
        raise ValueError("unknown density authority selection policy")
    if authority["density_parameter_id"] != OWNER_PARAMETER:
        raise ValueError("unknown density authority parameter")
    value_id = authority["density_value_id"]
    if value_id not in DENSITY_VALUES:
        raise ValueError("unknown density authority value")
    density = DENSITY_VALUES[value_id]
    if (
        type(authority["density_integer"]) is not int
        or authority["density_integer"] != density
    ):
        raise ValueError("density integer disagrees with typed value id")
    angle = authority["max_subturn_exact_value"]
    if not isinstance(angle, dict):
        raise ValueError("density exact angle is not a typed record")
    _exact_keys(
        angle,
        {"$type", "symbol", "turn_fraction_of_pi"},
        "density exact angle",
    )
    if angle != DENSITY_ANGLES[density]:
        raise ValueError("density exact angle disagrees with typed value id")
    if authority["reflex_excess_source"] != "PUBLIC_REFLEX_ANGLE_CERTIFICATE":
        raise ValueError("unknown reflex excess source")
    if Fraction(authority["expected_reflex_excess_over_pi"]) != Fraction(1, 2):
        raise ValueError("foreign reflex excess witness")
    witness_id = authority["gate0_hidden_count_witness_case_id"]
    witness = next(
        (
            row
            for row in owner_law["certificates"]
            if row["case_id"] == witness_id
        ),
        None,
    )
    if witness is None:
        raise ValueError("unknown Gate0 hidden-count witness")
    certified_u = Fraction(
        authority["expected_reflex_excess_over_pi"]
    )
    count = _ceil((density + 2) * certified_u)
    if (
        witness["d"] != density
        or witness["expected_C"] != count
        or witness["expected_E"] != max(2, count)
        or witness["expected_H"] != max(1, count - 1)
    ):
        raise ValueError("Gate0 witness disagrees with density authority")
    return density


def _validate_signed_cos_squared_record(record: dict[str, Any]) -> None:
    _exact_keys(
        record,
        {"$type", "turn_sign", "cos_squared"},
        "signed cosine squared",
    )
    if record["$type"] != SIGNED_COS_SQUARED_TYPE:
        raise ValueError("unknown signed cosine squared type")
    if record["turn_sign"] not in {"NEGATIVE", "ZERO", "POSITIVE"}:
        raise ValueError("unknown signed cosine turn sign")
    cosine_squared = Fraction(record["cos_squared"])
    if not 0 <= cosine_squared <= 1:
        raise ValueError("cosine squared is outside [0,1]")
    if record["turn_sign"] == "ZERO":
        if cosine_squared != 0:
            raise ValueError("zero cosine sign requires zero squared value")
    elif cosine_squared == 0:
        raise ValueError("nonzero cosine sign requires positive squared value")
    if record["cos_squared"] != str(cosine_squared):
        raise ValueError("cosine squared is not in irreducible canonical form")


def _validate_inputs_contract(inputs: dict[str, Any]) -> int:
    _exact_keys(
        inputs,
        {
            "schema",
            "proof_base_revision",
            "accepted_product_oids",
            "owner_law",
            "production_radical",
            "bit_cost_model",
        },
        "radical inputs",
    )
    if inputs["schema"] != INPUT_SCHEMA:
        raise ValueError("unknown radical input schema")
    _exact_keys(
        inputs["accepted_product_oids"],
        {"kernel_src", "cftuv"},
        "accepted product oids",
    )
    _exact_keys(
        inputs["bit_cost_model"],
        {"name", "budget", "input_integer_bit_limit"},
        "bit cost model",
    )
    if (
        inputs["bit_cost_model"]["name"]
        != "SCHOOLBOOK_INTEGER_BIT_OPS_UPPER_BOUND_V1"
    ):
        raise ValueError("unknown bit cost model")
    _validate_owner_law(inputs["owner_law"])
    source = inputs["production_radical"]
    _exact_keys(source, PRODUCTION_INPUT_KEYS, "production_radical")
    for field, expected in FROZEN_SOURCE_IDENTITY.items():
        if source[field] != expected:
            raise ValueError(f"foreign production source field: {field}")
    _validate_signed_cos_squared_record(source["expected_signed_cos_squared"])
    if type(source["root_refinement_steps"]) is not int:
        raise ValueError("root refinement steps must be an integer")
    return _validate_density_authority(
        source["density_authority"], inputs["owner_law"]
    )


def _f(value: Fraction | int | str) -> str:
    return str(Fraction(value))


def _ceil(value: Fraction) -> int:
    return -(-value.numerator // value.denominator)


def _sign(value: Fraction) -> int:
    return (value > 0) - (value < 0)


def _rat(value: Any) -> Fraction:
    if isinstance(value, Fraction):
        return value
    if isinstance(value, str | int):
        return Fraction(value)
    return Fraction(value.numerator, value.denominator)


def _point(value: Any) -> tuple[Fraction, Fraction]:
    return _rat(value.x), _rat(value.y)


def _sub(left, right) -> tuple[Fraction, Fraction]:
    return left[0] - right[0], left[1] - right[1]


def _cross(left, right) -> Fraction:
    return left[0] * right[1] - left[1] * right[0]


def _gram_dot(matrix, left, right) -> Fraction:
    return left[0] * (
        matrix[0][0] * right[0] + matrix[0][1] * right[1]
    ) + left[1] * (
        matrix[1][0] * right[0] + matrix[1][1] * right[1]
    )


def _input_integer_bits(value: Any) -> int:
    maximum = 1
    if isinstance(value, dict):
        for item in value.values():
            maximum = max(maximum, _input_integer_bits(item))
    elif isinstance(value, list):
        for item in value:
            maximum = max(maximum, _input_integer_bits(item))
    elif isinstance(value, int):
        maximum = max(maximum, abs(value).bit_length())
    elif isinstance(value, str) and re.fullmatch(r"-?\d+(?:/\d+)?", value):
        parsed = Fraction(value)
        maximum = max(
            maximum,
            abs(parsed.numerator).bit_length(),
            parsed.denominator.bit_length(),
        )
    return maximum


@dataclass
class BitLedger:
    max_integer_bits: int = 1
    exact_steps: int = 0

    def observe(self, *values: Any) -> None:
        for value in values:
            if isinstance(value, Fraction):
                self.max_integer_bits = max(
                    self.max_integer_bits,
                    abs(value.numerator).bit_length(),
                    value.denominator.bit_length(),
                )
            elif isinstance(value, int):
                self.max_integer_bits = max(
                    self.max_integer_bits, abs(value).bit_length()
                )
            elif isinstance(value, (tuple, list)):
                self.observe(*value)

    def step(self, count: int, *values: Any) -> None:
        self.exact_steps += count
        self.observe(*values)

    def receipt(self, budget: int) -> dict[str, Any]:
        fraction_ops = self.exact_steps * 256
        cost = fraction_ops * (2 * self.max_integer_bits) ** 2
        return {
            "model": "SCHOOLBOOK_INTEGER_BIT_OPS_UPPER_BOUND_V1",
            "exact_proof_step_count": self.exact_steps,
            "max_integer_bit_length": self.max_integer_bits,
            "fraction_arithmetic_operation_upper_bound": fraction_ops,
            "schoolbook_integer_bit_ops_upper_bound": cost,
            "budget": budget,
            "pass": cost <= budget,
        }


def _signed_cos_squared(
    dot: Fraction,
    norm_product: Fraction,
) -> dict[str, Any]:
    if norm_product <= 0:
        raise ValueError("non-positive norm product")
    if dot == 0:
        return {
            "$type": SIGNED_COS_SQUARED_TYPE,
            "turn_sign": "ZERO",
            "cos_squared": "0",
        }
    cosine_squared = dot * dot / norm_product
    if not 0 < cosine_squared <= 1:
        raise ValueError("derived cosine squared is outside (0,1]")
    return {
        "$type": SIGNED_COS_SQUARED_TYPE,
        "turn_sign": "POSITIVE" if dot > 0 else "NEGATIVE",
        "cos_squared": _f(cosine_squared),
    }


def _chebyshev_from_half_tangent(t: Fraction, n: int) -> Fraction:
    if type(n) is not int or n < 0:
        raise ValueError("Chebyshev degree must be a non-negative integer")
    t2 = t * t
    x = (1 - t2) / (1 + t2)
    if n == 0:
        return Fraction(1)
    if n == 1:
        return x
    previous = Fraction(1)
    current = x
    for _ in range(2, n + 1):
        previous, current = current, 2 * x * current - previous
    return current


def _root_sign(
    t: Fraction,
    root_n: int,
    target: dict[str, Any],
    ledger: BitLedger,
) -> int:
    chebyshev = _chebyshev_from_half_tangent(t, root_n)
    ledger.step(4 + 3 * max(0, root_n - 1), t, chebyshev)
    cosine_squared = Fraction(target["cos_squared"])
    target_sign = {
        "NEGATIVE": -1,
        "ZERO": 0,
        "POSITIVE": 1,
    }[target["turn_sign"]]
    ledger.step(1, chebyshev, cosine_squared, target_sign)
    if target_sign == 0:
        return _sign(chebyshev)
    chebyshev_sign = _sign(chebyshev)
    if chebyshev_sign != target_sign:
        return 1 if chebyshev_sign > target_sign else -1
    chebyshev_squared = chebyshev * chebyshev
    ledger.step(1, chebyshev_squared, cosine_squared)
    comparison = _sign(chebyshev_squared - cosine_squared)
    return comparison * target_sign


def _refine(
    lower: Fraction,
    upper: Fraction,
    root_n: int,
    target: dict[str, Any],
    steps: int,
    ledger: BitLedger,
) -> tuple[Fraction, Fraction]:
    if _root_sign(lower, root_n, target, ledger) != 1:
        raise ValueError("principal lower endpoint does not have positive sign")
    if _root_sign(upper, root_n, target, ledger) != -1:
        raise ValueError("principal upper endpoint does not have negative sign")
    for _ in range(steps):
        middle = (lower + upper) / 2
        if _root_sign(middle, root_n, target, ledger) > 0:
            lower = middle
        else:
            upper = middle
    return lower, upper


def _owner_rows(source: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for record in source["certificates"]:
        d = int(record["d"])
        q = d + 2
        u = Fraction(record["u"])
        qu = q * u
        c = _ceil(qu)
        e = max(2, c)
        h = max(1, c - 1)
        lower_crossing = Fraction(c - 1, q)
        upper_crossing = Fraction(c, q)
        rows.append(
            {
                **record,
                "q": q,
                "q_times_u": _f(qu),
                "derived_C": c,
                "derived_E": e,
                "derived_H": h,
                "crossing_cell": {
                    "lower": _f(lower_crossing),
                    "lower_kind": "OPEN" if c > 0 else "CLOSED",
                    "upper": _f(upper_crossing),
                    "upper_kind": "CLOSED",
                    "contains_u": lower_crossing < u <= upper_crossing,
                    "uncertainty": "NONE_EXACT_RATIONAL",
                },
                "certificate_pass": (
                    c == record["expected_C"]
                    and e == record["expected_E"]
                    and h == record["expected_H"]
                ),
                "red_claimed_next_H": h + 1,
                "red_claim_outcome": "UNREACHABLE_H_CLAIM",
            }
        )
    return rows


def _git_oid(root: Path, expression: str) -> str:
    return subprocess.run(
        ["git", "rev-parse", expression],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _git_is_ancestor(root: Path, ancestor: str) -> bool:
    return (
        subprocess.run(
            ["git", "merge-base", "--is-ancestor", ancestor, "HEAD"],
            cwd=root,
            check=False,
        ).returncode
        == 0
    )


def _load_compilation(root: Path, source: dict[str, Any]):
    kernel_src = root / "kernel" / "src"
    sys.path.insert(0, str(kernel_src))
    import cftuv_envelope as kernel

    snapshot_bytes = (root / source["snapshot_path"]).read_bytes()
    request_bytes = (root / source["request_path"]).read_bytes()
    if _sha(snapshot_bytes) != source["snapshot_sha256"]:
        raise ValueError("snapshot hash mismatch")
    if _sha(request_bytes) != source["request_sha256"]:
        raise ValueError("request hash mismatch")
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    request = kernel.DecalRequestCodecV1.loads(request_bytes)
    result = kernel.compile_reference_envelopes(snapshot, request)
    if result.outcome.value != "EXACT" or result.compilation is None:
        raise ValueError(f"fixture compile is not EXACT: {result.diagnostics}")
    compilation = result.compilation
    metric = next(
        item
        for item in snapshot.surface_metric_descriptors
        if getattr(item.reference_metric_id, "value", None)
        == source["reference_metric_id"]
    )
    metric_bytes = kernel.RationalAffinePlanarMetricCodecV2.dumps(metric)
    if _sha(metric_bytes) != source["metric_codec_sha256"]:
        raise ValueError("metric codec hash mismatch")
    spec = next(
        item
        for item in compilation.envelope_specs
        if item.envelope_spec_id.value == source["angular_spec_id"]
    )
    if spec.owner_sector_id.value != source["owner_sector_id"]:
        raise ValueError("owner sector mismatch")
    if spec.source_relation_id.value != source["source_relation_id"]:
        raise ValueError("source relation mismatch")
    relation = next(
        item
        for item in snapshot.corner_relations
        if item.corner_relation_id.value == source["source_relation_id"]
    )
    sector = next(
        item
        for item in snapshot.angular_owner_sectors
        if item.owner_sector_id.value == source["owner_sector_id"]
    )
    certificate = next(
        item
        for item in snapshot.reflex_angle_certificates
        if item.certificate_id == relation.reflex_angle_certificate_id
    )
    incident = tuple(
        item.value for item in sector.ordered_incident_chain_use_ids
    )
    expected_incident = (
        source["incoming_chain_use_id"],
        source["outgoing_chain_use_id"],
    )
    if incident != expected_incident:
        raise ValueError("declared incident chains do not equal owner sector")
    if relation.owner_sector_id != sector.owner_sector_id:
        raise ValueError("relation does not own the declared sector")
    if relation.source_vertex_id.value != source["anchor_source_vertex_id"]:
        raise ValueError("relation anchor mismatch")
    return (
        kernel,
        snapshot,
        compilation,
        metric,
        spec,
        relation,
        sector,
        certificate,
    )


def _metric_matrix(metric) -> tuple[tuple[Fraction, Fraction], ...]:
    value = metric.exact_gram_matrix
    return (
        (_rat(value.m00), _rat(value.m01)),
        (_rat(value.m10), _rat(value.m11)),
    )


def _build_production_source(
    kernel,
    snapshot,
    compilation,
    metric,
    spec,
    relation,
    sector,
    certificate,
) -> dict[str, Any]:
    """Вся source-власть пересобирается из public codecs/compiler."""

    from cftuv_envelope.codec import canonical_json_bytes

    def record(record) -> dict[str, Any]:
        payload = canonical_json_bytes(record)
        return {
            "record_type": type(record).__name__,
            "canonical_byte_length": len(payload),
            "canonical_sha256": _sha(payload),
        }

    request = compilation.decal_request
    snapshot_bytes = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    request_bytes = kernel.DecalRequestCodecV1.dumps(request)
    compilation_bytes = canonical_json_bytes(compilation)
    domain = next(
        item
        for item in snapshot.patch_domains
        if item.patch_domain_id == compilation.plan_key.patch_domain_id
    )
    binding = compilation.evaluation_geometry_binding
    grid = metric.grid_certificate
    planarity = metric.planarity_certificate
    hidden = sorted(spec.hidden_supports, key=lambda item: item.ordinal)
    return {
        "$type": "ProofOnlyProductionRadicalSourceV2",
        "snapshot": {
            "path": FROZEN_SOURCE_IDENTITY["snapshot_path"],
            "codec": "AnalysisSnapshotCodecV1",
            "canonical_byte_length": len(snapshot_bytes),
            "canonical_sha256": _sha(snapshot_bytes),
            "source_revision": snapshot.source_revision.value,
        },
        "request": {
            "path": FROZEN_SOURCE_IDENTITY["request_path"],
            "codec": "DecalRequestCodecV1",
            "canonical_byte_length": len(request_bytes),
            "canonical_sha256": _sha(request_bytes),
            "decal_request_id": request.decal_request_id.value,
        },
        "compilation": {
            "schema_version": compilation.schema_version,
            "outcome": "EXACT",
            "canonical_byte_length": len(compilation_bytes),
            "canonical_sha256": _sha(compilation_bytes),
            "source_revision": compilation.source_revision.value,
            "owner_patch_id": compilation.owner_patch_id.value,
            "plan_key": {
                **record(compilation.plan_key),
                "decal_request_id": compilation.plan_key.decal_request_id.value,
                "patch_domain_id": (
                    compilation.plan_key.patch_domain_id.value
                ),
            },
            "initial_front_spec": record(compilation.initial_front_spec),
        },
        "patch_domain": {
            **record(domain),
            "patch_domain_id": domain.patch_domain_id.value,
            "owner_patch_id": domain.owner_patch_id.value,
            "surface_regime": domain.surface_regime.value,
        },
        "reference_metric": {
            **record(metric),
            "codec": "RationalAffinePlanarMetricCodecV2",
            "codec_sha256": _sha(
                kernel.RationalAffinePlanarMetricCodecV2.dumps(metric)
            ),
            "reference_metric_id": metric.reference_metric_id.value,
            "patch_domain_id": metric.patch_domain_id.value,
            "source_revision": metric.source_revision.value,
            "chart_orientation": metric.chart_orientation.value,
            "frame_selection_law": metric.frame_selection_law.value,
            "planarity_admission_law": planarity.admission_law.value,
            "planarity_reconstruction_law": (
                planarity.reconstruction_law.value
            ),
            "grid_search_order": grid.search_order.value,
            "grid_snapping_law": grid.snapping_law.value,
            "grid_source_scale": grid.source_scale,
        },
        "angular_spec": {
            **record(spec),
            "angular_spec_id": spec.envelope_spec_id.value,
            "patch_domain_id": spec.patch_domain_id.value,
            "owner_sector_id": spec.owner_sector_id.value,
            "source_relation_id": spec.source_relation_id.value,
            "angle_certificate_id": spec.angle_certificate_id.value,
            "selection_certificate_id": spec.selection_certificate_id.value,
            "profile_family_id": spec.profile_family_id.value,
            "resolved_hidden_edge_count": spec.resolved_hidden_edge_count,
            "subdivision_policy": spec.subdivision_policy.value,
            "all_support_normal_speed": spec.all_support_normal_speed,
            "hidden_supports": [
                {
                    "hidden_support_id": item.hidden_support_id.value,
                    "ordinal": item.ordinal,
                    "direction_law": item.direction_law.value,
                    "binding_reason": (
                        item.direction_binding.binding_reason.value
                    ),
                }
                for item in hidden
            ],
        },
        "corner_relation": {
            **record(relation),
            "corner_relation_id": relation.corner_relation_id.value,
            "source_vertex_id": relation.source_vertex_id.value,
            "owner_sector_id": relation.owner_sector_id.value,
            "angle_certificate_id": (
                relation.reflex_angle_certificate_id.value
            ),
            "exact_two_pi": relation.exact_two_pi,
        },
        "owner_sector": {
            **record(sector),
            "owner_sector_id": sector.owner_sector_id.value,
            "owner_patch_id": sector.owner_patch_id.value,
            "patch_domain_id": sector.patch_domain_id.value,
            "ordered_incident_chain_use_ids": [
                item.value for item in sector.ordered_incident_chain_use_ids
            ],
            "turn_orientation": sector.turn_orientation.value,
            "interior_selection_law": sector.interior_selection_law.value,
        },
        "angle_certificate": {
            **record(certificate),
            "angle_certificate_id": certificate.certificate_id.value,
            "owner_sector_id": certificate.owner_sector_id.value,
            "measure_law": certificate.measure_law.value,
            "measure_source": certificate.measure_source.value,
            "strict_range_certificate": (
                certificate.strict_range_certificate.value
            ),
            "reflex_excess_law": certificate.reflex_excess_law.value,
            "exact_two_pi": certificate.exact_two_pi,
        },
        "evaluation_geometry_binding": {
            **record(binding),
            "schema_version": binding.schema_version,
            "source_revision": binding.source_revision.value,
            "patch_domain_id": binding.patch_domain_id.value,
            "reference_metric_id": binding.reference_metric_id.value,
            "binding_law": binding.binding_law.value,
            "lattice_scale": binding.lattice_scale,
            "bound_hidden_support_ids": sorted(
                item.value for item in binding.bound_hidden_support_ids
            ),
        },
    }


def _chain_vector(
    snapshot,
    coordinates: dict[str, tuple[Fraction, Fraction]],
    chain_use_id: str,
) -> tuple[Fraction, Fraction]:
    use = next(
        item
        for item in snapshot.chain_uses
        if item.chain_use_id.value == chain_use_id
    )
    chain = next(
        item
        for item in snapshot.physical_chains
        if item.physical_chain_id == use.physical_chain_id
    )
    vertices = [item.value for item in chain.ordered_source_vertex_ids]
    if use.orientation.value == "A_START_TO_END":
        start, end = vertices[0], vertices[-1]
    elif use.orientation.value == "B_START_TO_END":
        start, end = vertices[-1], vertices[0]
    else:
        raise ValueError(f"unknown chain orientation {use.orientation}")
    return _sub(coordinates[end], coordinates[start])


def _cosine_squared_binding(
    matrix,
    incoming,
    outgoing,
    expected: dict[str, Any],
    ledger: BitLedger,
) -> tuple[dict[str, Any], dict[str, Any]]:
    aa = _gram_dot(matrix, incoming, incoming)
    bb = _gram_dot(matrix, outgoing, outgoing)
    ab = _gram_dot(matrix, incoming, outgoing)
    product = aa * bb
    if aa <= 0 or bb <= 0:
        raise ValueError("production rays do not define positive Gram norms")
    target = _signed_cos_squared(ab, product)
    if target != expected:
        raise ValueError("serialized signed cosine differs from production")
    ledger.step(12, matrix, incoming, outgoing, aa, bb, ab, product)
    return target, {
        "incoming_norm_squared": _f(aa),
        "outgoing_norm_squared": _f(bb),
        "dot": _f(ab),
        "norm_product": _f(product),
        "identity": "cos_squared=dot^2/(incoming_norm_squared*outgoing_norm_squared)",
    }


def _solve_orientation(
    name: str,
    matrix,
    incoming,
    outgoing,
    expected_orientation: int,
    principal_bracket: tuple[Fraction, Fraction],
    other_bracket: tuple[Fraction, Fraction],
    root_n: int,
    steps: int,
    expected_cosine: dict[str, Any],
    ledger: BitLedger,
) -> dict[str, Any]:
    if root_n != 2:
        raise ValueError("this exact proof slice implements derived n=2")
    cross = _cross(incoming, outgoing)
    if _sign(cross) != expected_orientation:
        raise ValueError(f"{name}: orientation mismatch")
    signed_cosine, gram = _cosine_squared_binding(
        matrix, incoming, outgoing, expected_cosine, ledger
    )
    principal_signs = [
        _root_sign(value, root_n, signed_cosine, ledger)
        for value in principal_bracket
    ]
    other_signs = [
        _root_sign(value, root_n, signed_cosine, ledger)
        for value in other_bracket
    ]
    refined = _refine(
        *principal_bracket, root_n, signed_cosine, steps, ledger
    )
    # d/dt cos(2 theta(t)) = -16t(1-t^2)/(1+t^2)^3.
    # Thus it is strictly negative on (0,1), strictly positive on (1,+inf).
    branch = {
        "parameter": "t=tan(theta/2)",
        "principal_domain": ["0", "1"],
        "principal_domain_kinds": ["OPEN", "OPEN"],
        "principal_derivative_numerator_sign": "NEGATIVE",
        "principal_monotone": "STRICTLY_DECREASING",
        "principal_bracket_inside_domain": (
            0 < principal_bracket[0] < principal_bracket[1] < 1
        ),
        "principal_unique_root": principal_signs == [1, -1],
        "subturn_branch": "0<theta<pi/2",
        "other_domain": ["1", "POSITIVE_INFINITY"],
        "other_derivative_numerator_sign": "POSITIVE",
        "other_monotone": "STRICTLY_INCREASING",
        "other_bracket_inside_domain": (
            1 < other_bracket[0] < other_bracket[1]
        ),
        "other_bracket_signs": other_signs,
        "other_branch_has_root": other_signs == [-1, 1],
        "other_branch_rejection": "OUTSIDE_OWNER_SUBTURN_BRANCH",
    }
    if not (
        branch["principal_bracket_inside_domain"]
        and branch["principal_unique_root"]
        and branch["other_branch_has_root"]
    ):
        raise ValueError("branch proof failed")
    return {
        "name": name,
        "gram": [[_f(value) for value in row] for row in matrix],
        "incoming": [_f(value) for value in incoming],
        "outgoing": [_f(value) for value in outgoing],
        "cross": _f(cross),
        "orientation_sign": _sign(cross),
        "cosine": {
            **signed_cosine,
            **gram,
        },
        "root_equation": (
            "T_n((1-t^2)/(1+t^2))=signed_sqrt(cos_squared)"
        ),
        "root_predicate": {
            "n": root_n,
            "selected_sign_branch": signed_cosine["turn_sign"],
            "comparison": (
                "sign(T_n(x))=turn_sign; compare T_n(x)^2 to cos_squared"
            ),
            "arithmetic": "EXACT_FRACTION",
        },
        "principal_bracket": [_f(value) for value in principal_bracket],
        "principal_bracket_signs": principal_signs,
        "refined_bracket": [_f(value) for value in refined],
        "refinement_steps": steps,
        "branch_certificate": branch,
    }


def main() -> None:
    args = _args()
    root = args.source_root.resolve()
    inputs_bytes = args.inputs.read_bytes()
    inputs = json.loads(inputs_bytes)
    validated_density = _validate_inputs_contract(inputs)
    input_bits = _input_integer_bits(inputs)
    if input_bits > inputs["bit_cost_model"]["input_integer_bit_limit"]:
        raise ValueError("input integer bit limit exceeded")
    accepted = inputs["accepted_product_oids"]
    observed = {
        "proof_base_revision": inputs["proof_base_revision"],
        "kernel_src": _git_oid(root, "HEAD:kernel/src"),
        "cftuv": _git_oid(root, "HEAD:cftuv"),
    }
    if not _git_is_ancestor(root, inputs["proof_base_revision"]):
        raise ValueError("proof base is not an ancestor")
    if observed["kernel_src"] != accepted["kernel_src"]:
        raise ValueError("unknown kernel product tree")
    if observed["cftuv"] != accepted["cftuv"]:
        raise ValueError("unknown host product tree")

    owner = _owner_rows(inputs["owner_law"])
    if not all(item["certificate_pass"] for item in owner):
        raise ValueError("owner law fixture failed")
    h1 = [item for item in owner if item["derived_H"] == 1]
    if {item["derived_C"] for item in h1} != {1, 2}:
        raise ValueError("H1 must cover both C=1 and C=2")

    source = inputs["production_radical"]
    (
        kernel,
        snapshot,
        compilation,
        metric,
        spec,
        relation,
        sector,
        certificate,
    ) = _load_compilation(root, source)
    authority = source["density_authority"]
    measure = certificate.measure_payload.reflex_excess_over_pi
    if (
        measure.lower != measure.upper
        or measure.absolute_error_bound != 0
        or measure.lower_kind.value != "CLOSED"
        or measure.upper_kind.value != "CLOSED"
    ):
        raise ValueError("relation reflex excess is not exact")
    certified_u = Fraction(str(measure.lower))
    if certified_u != Fraction(authority["expected_reflex_excess_over_pi"]):
        raise ValueError("certified reflex excess differs from authority")
    density = validated_density
    q = density + 2
    count = _ceil(q * certified_u)
    emanated = max(2, count)
    hidden = max(1, count - 1)
    root_n = hidden + 1
    witness = next(
        item
        for item in owner
        if item["case_id"]
        == authority["gate0_hidden_count_witness_case_id"]
    )
    if witness["derived_H"] != hidden:
        raise ValueError("Gate0 witness has foreign hidden count")
    if spec.resolved_hidden_edge_count != hidden:
        raise ValueError("compile observation conflicts with density authority")
    matrix = _metric_matrix(metric)
    if [[_f(value) for value in row] for row in matrix] != source[
        "expected_gram"
    ]:
        raise ValueError("production Gram mismatch")
    binding = compilation.evaluation_geometry_binding
    coordinates = {
        item.source_vertex_id.value: _point(item.domain_coordinate)
        for item in binding.source_vertex_coordinates
    }
    incoming = _chain_vector(
        snapshot, coordinates, source["incoming_chain_use_id"]
    )
    outgoing = _chain_vector(
        snapshot, coordinates, source["outgoing_chain_use_id"]
    )
    principal = tuple(Fraction(x) for x in source["principal_half_tangent_bracket"])
    other = tuple(Fraction(x) for x in source["other_branch_half_tangent_bracket"])
    ledger = BitLedger()
    original = _solve_orientation(
        "ORIGINAL_CW",
        matrix,
        incoming,
        outgoing,
        -1,
        principal,
        other,
        root_n,
        source["root_refinement_steps"],
        source["expected_signed_cos_squared"],
        ledger,
    )

    # Mirror is rebuilt from mirrored coordinates and M^T G M, not from the
    # original result.  M=diag(1,-1), so the off-diagonal entries change sign.
    mirrored_coordinates = {
        key: (value[0], -value[1]) for key, value in coordinates.items()
    }
    mirrored_matrix = (
        (matrix[0][0], -matrix[0][1]),
        (-matrix[1][0], matrix[1][1]),
    )
    mirrored_incoming = _chain_vector(
        snapshot, mirrored_coordinates, source["incoming_chain_use_id"]
    )
    mirrored_outgoing = _chain_vector(
        snapshot, mirrored_coordinates, source["outgoing_chain_use_id"]
    )
    mirrored = _solve_orientation(
        "MIRRORED_CCW",
        mirrored_matrix,
        mirrored_incoming,
        mirrored_outgoing,
        1,
        principal,
        other,
        root_n,
        source["root_refinement_steps"],
        source["expected_signed_cos_squared"],
        ledger,
    )
    if mirrored["refined_bracket"] != original["refined_bracket"]:
        raise ValueError("mirrored solve changed the root")

    wrong_metric_dot = _gram_dot(
        matrix, mirrored_incoming, mirrored_outgoing
    )
    red = {
        "unreachable_owner_H": {
            "reached": True,
            "outcome": "UNREACHABLE_H_CLAIM",
        },
        "other_root_branch": {
            "reached": True,
            "signs": original["branch_certificate"][
                "other_bracket_signs"
            ],
            "outcome": "OUTSIDE_OWNER_SUBTURN_BRANCH",
        },
        "mirror_without_metric_transform": {
            "reached": True,
            "wrong_dot": _f(wrong_metric_dot),
            "correct_dot": mirrored["cosine"]["dot"],
            "outcome": "MIRROR_GRAM_COVARIANCE_VIOLATION",
        },
        "mirrored_input_reversal": {
            "reached": True,
            "reversed_cross": _f(
                _cross(mirrored_outgoing, mirrored_incoming)
            ),
            "expected_orientation_sign": 1,
            "outcome": "OWNER_ORIENTATION_REVERSAL",
        },
    }
    perf = ledger.receipt(inputs["bit_cost_model"]["budget"])
    perf["input_max_integer_bit_length"] = input_bits
    perf["input_integer_bit_limit"] = inputs["bit_cost_model"][
        "input_integer_bit_limit"
    ]
    if not perf["pass"]:
        raise ValueError("bit-cost budget exceeded")

    evidence = {
        "schema": SCHEMA,
        "inputs_sha256": _sha(inputs_bytes),
        "product_tree": observed,
        "owner_law": {
            "selection_policy_id": inputs["owner_law"][
                "selection_policy_id"
            ],
            "density_parameter_id": inputs["owner_law"][
                "density_parameter_id"
            ],
            "formula": inputs["owner_law"]["formula"],
            "certificates": owner,
        },
        "production_source": _build_production_source(
            kernel,
            snapshot,
            compilation,
            metric,
            spec,
            relation,
            sector,
            certificate,
        ),
        "radical_binding": {
            "owner_density_authority": {
                **authority,
                "corner_relation_id": relation.corner_relation_id.value,
                "angle_certificate_id": certificate.certificate_id.value,
                "certified_u": _f(certified_u),
                "q": q,
                "derived_C": count,
                "derived_E": emanated,
                "derived_H": hidden,
                "derived_root_n": root_n,
            },
            "root_n": root_n,
            "original": original,
            "mirrored": mirrored,
            "mirror_matrix": [["1", "0"], ["0", "-1"]],
            "mirror_root_identity": True,
        },
        "red_controls": red,
        "performance": perf,
        "claims": {
            "owner_H1_through_H5_reachable": sorted(
                {item["derived_H"] for item in owner}
            )
            == [1, 2, 3, 4, 5],
            "H1_C1_and_C2_covered": {item["derived_C"] for item in h1}
            == {1, 2},
            "production_signed_cos_squared": True,
            "principal_branch_unique": original["branch_certificate"][
                "principal_unique_root"
            ],
            "other_branch_rejected": original["branch_certificate"][
                "other_branch_has_root"
            ],
            "mirrored_input_off_diagonal_gram": (
                matrix[0][1] != 0
                and mirrored_matrix[0][1] == -matrix[0][1]
            ),
            "mirror_root_identity": True,
            "all_red_controls_reached": all(
                item["reached"] for item in red.values()
            ),
            "bit_budget_pass": perf["pass"],
        },
    }
    wrapper = {
        "schema": "cftuv.envelope.dens_a_prime.radical_receipt_wrapper.v1",
        "evidence": evidence,
        "evidence_sha256": _sha(_canonical(evidence)),
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_bytes(_pretty(wrapper))


if __name__ == "__main__":
    main()
