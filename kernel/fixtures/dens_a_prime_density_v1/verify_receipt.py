"""Independent verifier for the DENS-A-prime owner/root receipt.

It intentionally does not import generate_receipt.py.  All exact predicates,
fixture extraction, signed-cos-squared derivation, mirroring, root isolation,
and red controls are repeated here from the receipt inputs.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any


HERE = Path(__file__).resolve().parent
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
COSINE_RECEIPT_KEYS = {
    "$type",
    "turn_sign",
    "cos_squared",
    "incoming_norm_squared",
    "outgoing_norm_squared",
    "dot",
    "norm_product",
    "identity",
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, default=HERE.parents[2])
    parser.add_argument("--inputs", type=Path, default=HERE / "inputs.json")
    parser.add_argument("--receipt", type=Path, required=True)
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


def exact_keys(record: dict[str, Any], expected: set[str], label: str) -> None:
    actual = set(record)
    assert actual == expected, (
        f"{label} field mismatch: missing={sorted(expected - actual)} "
        f"extra={sorted(actual - expected)}"
    )


def validate_owner_law(owner_law: dict[str, Any]) -> None:
    exact_keys(
        owner_law,
        {
            "selection_policy_id",
            "density_parameter_id",
            "formula",
            "certificates",
        },
        "owner_law",
    )
    assert owner_law["selection_policy_id"] == OWNER_POLICY
    assert owner_law["density_parameter_id"] == OWNER_PARAMETER
    assert owner_law["formula"] == OWNER_FORMULA
    assert owner_law["certificates"] == list(EXPECTED_OWNER_CASES)
    for row in owner_law["certificates"]:
        exact_keys(
            row,
            {
                "case_id",
                "d",
                "u",
                "expected_C",
                "expected_E",
                "expected_H",
            },
            f"owner certificate {row.get('case_id', '<missing>')}",
        )


def validate_density_authority(
    authority: dict[str, Any],
    owner_law: dict[str, Any],
) -> int:
    exact_keys(
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
    assert authority["$type"] == AUTHORITY_TYPE
    assert authority["selection_policy_id"] == OWNER_POLICY
    assert authority["density_parameter_id"] == OWNER_PARAMETER
    value_id = authority["density_value_id"]
    assert value_id in DENSITY_VALUES
    density = DENSITY_VALUES[value_id]
    assert type(authority["density_integer"]) is int
    assert authority["density_integer"] == density
    angle = authority["max_subturn_exact_value"]
    assert isinstance(angle, dict)
    exact_keys(
        angle,
        {"$type", "symbol", "turn_fraction_of_pi"},
        "density exact angle",
    )
    assert angle == DENSITY_ANGLES[density]
    assert authority["reflex_excess_source"] == (
        "PUBLIC_REFLEX_ANGLE_CERTIFICATE"
    )
    assert Fraction(authority["expected_reflex_excess_over_pi"]) == Fraction(
        1, 2
    )
    witness_id = authority["gate0_hidden_count_witness_case_id"]
    witness = next(
        (
            row
            for row in owner_law["certificates"]
            if row["case_id"] == witness_id
        ),
        None,
    )
    assert witness is not None
    certified_u = Fraction(authority["expected_reflex_excess_over_pi"])
    count = ceil((density + 2) * certified_u)
    assert witness["d"] == density
    assert witness["expected_C"] == count
    assert witness["expected_E"] == max(2, count)
    assert witness["expected_H"] == max(1, count - 1)
    return density


def validate_signed_cos_squared_record(record: dict[str, Any]) -> None:
    exact_keys(
        record,
        {"$type", "turn_sign", "cos_squared"},
        "signed cosine squared",
    )
    assert record["$type"] == SIGNED_COS_SQUARED_TYPE
    assert record["turn_sign"] in {"NEGATIVE", "ZERO", "POSITIVE"}
    cosine_squared = Fraction(record["cos_squared"])
    assert 0 <= cosine_squared <= 1
    if record["turn_sign"] == "ZERO":
        assert cosine_squared == 0
    else:
        assert cosine_squared > 0
    assert record["cos_squared"] == str(cosine_squared)


def validate_inputs_contract(inputs: dict[str, Any]) -> int:
    exact_keys(
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
    assert inputs["schema"] == INPUT_SCHEMA
    exact_keys(
        inputs["accepted_product_oids"],
        {"kernel_src", "cftuv"},
        "accepted product oids",
    )
    exact_keys(
        inputs["bit_cost_model"],
        {"name", "budget", "input_integer_bit_limit"},
        "bit cost model",
    )
    assert inputs["bit_cost_model"]["name"] == (
        "SCHOOLBOOK_INTEGER_BIT_OPS_UPPER_BOUND_V1"
    )
    validate_owner_law(inputs["owner_law"])
    source = inputs["production_radical"]
    exact_keys(source, PRODUCTION_INPUT_KEYS, "production radical")
    for field, expected in FROZEN_SOURCE_IDENTITY.items():
        assert source[field] == expected, field
    validate_signed_cos_squared_record(source["expected_signed_cos_squared"])
    assert type(source["root_refinement_steps"]) is int
    return validate_density_authority(
        source["density_authority"], inputs["owner_law"]
    )


def rat(value: Any) -> Fraction:
    if isinstance(value, str | int):
        return Fraction(value)
    return Fraction(value.numerator, value.denominator)


def point(value: Any) -> tuple[Fraction, Fraction]:
    return rat(value.x), rat(value.y)


def f(value: Fraction | int | str) -> str:
    return str(Fraction(value))


def sign(value: Fraction) -> int:
    return (value > 0) - (value < 0)


def ceil(value: Fraction) -> int:
    return -(-value.numerator // value.denominator)


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
        parsed = Fraction(value)
        maximum = max(
            maximum,
            abs(parsed.numerator).bit_length(),
            parsed.denominator.bit_length(),
        )
    return maximum


def cross(left, right) -> Fraction:
    return left[0] * right[1] - left[1] * right[0]


def subtract(left, right) -> tuple[Fraction, Fraction]:
    return left[0] - right[0], left[1] - right[1]


def dot(matrix, left, right) -> Fraction:
    return left[0] * (
        matrix[0][0] * right[0] + matrix[0][1] * right[1]
    ) + left[1] * (
        matrix[1][0] * right[0] + matrix[1][1] * right[1]
    )


class Audit:
    def __init__(self) -> None:
        self.steps = 0
        self.max_bits = 1

    def record(self, count: int, *values: Any) -> None:
        self.steps += count
        for value in values:
            if isinstance(value, Fraction):
                self.max_bits = max(
                    self.max_bits,
                    abs(value.numerator).bit_length(),
                    value.denominator.bit_length(),
                )
            elif isinstance(value, int):
                self.max_bits = max(self.max_bits, abs(value).bit_length())
            elif isinstance(value, (tuple, list)):
                self.record(0, *value)

    def cost(self) -> int:
        return self.steps * 256 * (2 * self.max_bits) ** 2


class RootSolveTrace:
    def __init__(self) -> None:
        self.endpoint_predicate_calls = 0


def chebyshev_from_half_tangent(t: Fraction, n: int) -> Fraction:
    assert type(n) is int and n >= 0
    square = t * t
    x = (1 - square) / (1 + square)
    if n == 0:
        return Fraction(1)
    if n == 1:
        return x
    before = Fraction(1)
    value = x
    for _ in range(2, n + 1):
        before, value = value, 2 * x * value - before
    return value


def root_sign(
    t: Fraction,
    root_n: int,
    target: dict[str, Any],
    audit: Audit,
    trace: RootSolveTrace,
) -> int:
    trace.endpoint_predicate_calls += 1
    value = chebyshev_from_half_tangent(t, root_n)
    audit.record(4 + 3 * max(0, root_n - 1), t, value)
    cosine_squared = Fraction(target["cos_squared"])
    target_sign = {
        "NEGATIVE": -1,
        "ZERO": 0,
        "POSITIVE": 1,
    }[target["turn_sign"]]
    audit.record(1, value, cosine_squared, target_sign)
    if target_sign == 0:
        return sign(value)
    value_sign = sign(value)
    if value_sign != target_sign:
        return 1 if value_sign > target_sign else -1
    value_squared = value * value
    audit.record(1, value_squared, cosine_squared)
    return sign(value_squared - cosine_squared) * target_sign


def refine(
    lower: Fraction,
    upper: Fraction,
    root_n: int,
    target: dict[str, Any],
    steps: int,
    audit: Audit,
    trace: RootSolveTrace,
) -> tuple[Fraction, Fraction]:
    assert root_sign(lower, root_n, target, audit, trace) == 1
    assert root_sign(upper, root_n, target, audit, trace) == -1
    for _ in range(steps):
        middle = (lower + upper) / 2
        if root_sign(middle, root_n, target, audit, trace) > 0:
            lower = middle
        else:
            upper = middle
    return lower, upper


def git_oid(root: Path, expression: str) -> str:
    return subprocess.run(
        ["git", "rev-parse", expression],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def git_is_ancestor(root: Path, ancestor: str) -> bool:
    return (
        subprocess.run(
            ["git", "merge-base", "--is-ancestor", ancestor, "HEAD"],
            cwd=root,
            check=False,
        ).returncode
        == 0
    )


def load_product(root: Path, source: dict[str, Any]):
    sys.path.insert(0, str(root / "kernel" / "src"))
    import cftuv_envelope as kernel

    snapshot_bytes = (root / source["snapshot_path"]).read_bytes()
    request_bytes = (root / source["request_path"]).read_bytes()
    assert sha(snapshot_bytes) == source["snapshot_sha256"]
    assert sha(request_bytes) == source["request_sha256"]
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_bytes)
    request = kernel.DecalRequestCodecV1.loads(request_bytes)
    result = kernel.compile_reference_envelopes(snapshot, request)
    assert result.outcome.value == "EXACT"
    compilation = result.compilation
    assert compilation is not None
    metric = next(
        item
        for item in snapshot.surface_metric_descriptors
        if item.reference_metric_id.value == source["reference_metric_id"]
    )
    assert sha(kernel.RationalAffinePlanarMetricCodecV2.dumps(metric)) == source[
        "metric_codec_sha256"
    ]
    spec = next(
        item
        for item in compilation.envelope_specs
        if item.envelope_spec_id.value == source["angular_spec_id"]
    )
    assert spec.owner_sector_id.value == source["owner_sector_id"]
    assert spec.source_relation_id.value == source["source_relation_id"]
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
    expected_incident = (
        source["incoming_chain_use_id"],
        source["outgoing_chain_use_id"],
    )
    actual_incident = tuple(
        item.value for item in sector.ordered_incident_chain_use_ids
    )
    assert actual_incident == expected_incident
    assert relation.owner_sector_id == sector.owner_sector_id
    assert relation.source_vertex_id.value == source["anchor_source_vertex_id"]
    return snapshot, compilation, metric, spec, relation, sector, certificate


def matrix_from(metric):
    matrix = metric.exact_gram_matrix
    return (
        ((rat(matrix.m00)), rat(matrix.m01)),
        (rat(matrix.m10), rat(matrix.m11)),
    )


def rebuild_production_source(
    snapshot,
    compilation,
    metric,
    spec,
    relation,
    sector,
    certificate,
) -> dict[str, Any]:
    """Независимо пересобирает весь receipt source-record."""

    import cftuv_envelope as kernel
    from cftuv_envelope.codec import canonical_json_bytes

    def record(value) -> dict[str, Any]:
        payload = canonical_json_bytes(value)
        return {
            "record_type": type(value).__name__,
            "canonical_byte_length": len(payload),
            "canonical_sha256": sha(payload),
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
            "canonical_sha256": sha(snapshot_bytes),
            "source_revision": snapshot.source_revision.value,
        },
        "request": {
            "path": FROZEN_SOURCE_IDENTITY["request_path"],
            "codec": "DecalRequestCodecV1",
            "canonical_byte_length": len(request_bytes),
            "canonical_sha256": sha(request_bytes),
            "decal_request_id": request.decal_request_id.value,
        },
        "compilation": {
            "schema_version": compilation.schema_version,
            "outcome": "EXACT",
            "canonical_byte_length": len(compilation_bytes),
            "canonical_sha256": sha(compilation_bytes),
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
            "codec_sha256": sha(
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


def chain_vector(snapshot, coordinates, chain_use_id):
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
    ids = [item.value for item in chain.ordered_source_vertex_ids]
    if use.orientation.value == "A_START_TO_END":
        start, end = ids[0], ids[-1]
    else:
        assert use.orientation.value == "B_START_TO_END"
        start, end = ids[-1], ids[0]
    return subtract(coordinates[end], coordinates[start])


def derive_signed_cos_squared(
    matrix,
    incoming,
    outgoing,
    expected: dict[str, Any],
    audit: Audit,
):
    aa = dot(matrix, incoming, incoming)
    bb = dot(matrix, outgoing, outgoing)
    ab = dot(matrix, incoming, outgoing)
    product = aa * bb
    audit.record(12, matrix, incoming, outgoing, aa, bb, ab, product)
    assert aa > 0 and bb > 0
    if ab == 0:
        derived = {
            "$type": SIGNED_COS_SQUARED_TYPE,
            "turn_sign": "ZERO",
            "cos_squared": "0",
        }
    else:
        cosine_squared = ab * ab / product
        assert 0 < cosine_squared <= 1
        derived = {
            "$type": SIGNED_COS_SQUARED_TYPE,
            "turn_sign": "POSITIVE" if ab > 0 else "NEGATIVE",
            "cos_squared": f(cosine_squared),
        }
    assert expected == derived
    return derived, aa, bb, ab, product


def verify_owner(inputs, receipt) -> dict[str, Any]:
    owner_receipt = receipt["evidence"]["owner_law"]
    exact_keys(
        owner_receipt,
        {
            "selection_policy_id",
            "density_parameter_id",
            "formula",
            "certificates",
        },
        "owner_law receipt",
    )
    assert owner_receipt["selection_policy_id"] == OWNER_POLICY
    assert owner_receipt["density_parameter_id"] == OWNER_PARAMETER
    assert owner_receipt["formula"] == OWNER_FORMULA
    observed = owner_receipt["certificates"]
    assert len(observed) == len(inputs["owner_law"]["certificates"])
    h1_c = set()
    hidden = set()
    for source, row in zip(inputs["owner_law"]["certificates"], observed):
        assert row["case_id"] == source["case_id"]
        q = source["d"] + 2
        u = Fraction(source["u"])
        c = ceil(q * u)
        e = max(2, c)
        h = max(1, c - 1)
        assert (c, e, h) == (
            source["expected_C"],
            source["expected_E"],
            source["expected_H"],
        )
        assert (row["derived_C"], row["derived_E"], row["derived_H"]) == (
            c,
            e,
            h,
        )
        assert row["crossing_cell"]["lower"] == f(Fraction(c - 1, q))
        assert row["crossing_cell"]["upper"] == f(Fraction(c, q))
        assert Fraction(c - 1, q) < u <= Fraction(c, q)
        expected_row = {
            **source,
            "q": q,
            "q_times_u": f(q * u),
            "derived_C": c,
            "derived_E": e,
            "derived_H": h,
            "crossing_cell": {
                "lower": f(Fraction(c - 1, q)),
                "lower_kind": "OPEN" if c > 0 else "CLOSED",
                "upper": f(Fraction(c, q)),
                "upper_kind": "CLOSED",
                "contains_u": True,
                "uncertainty": "NONE_EXACT_RATIONAL",
            },
            "certificate_pass": True,
            "red_claimed_next_H": h + 1,
            "red_claim_outcome": "UNREACHABLE_H_CLAIM",
        }
        assert row == expected_row
        hidden.add(h)
        if h == 1:
            h1_c.add(c)
    assert hidden == {1, 2, 3, 4, 5}
    assert h1_c == {1, 2}
    return {"hidden_counts": sorted(hidden), "H1_C_cells": sorted(h1_c)}


def verify_radical(
    root,
    inputs,
    receipt,
    audit: Audit,
    trace: RootSolveTrace,
    owner_summary,
) -> dict[str, Any]:
    source = inputs["production_radical"]
    (
        snapshot,
        compilation,
        metric,
        spec,
        relation,
        sector,
        certificate,
    ) = load_product(root, source)
    authority_input = source["density_authority"]
    measure = certificate.measure_payload.reflex_excess_over_pi
    assert measure.lower == measure.upper
    assert measure.absolute_error_bound == 0
    assert measure.lower_kind.value == "CLOSED"
    assert measure.upper_kind.value == "CLOSED"
    certified_u = Fraction(str(measure.lower))
    assert certified_u == Fraction(
        authority_input["expected_reflex_excess_over_pi"]
    )
    density = validate_density_authority(
        authority_input, inputs["owner_law"]
    )
    q = density + 2
    count = ceil(q * certified_u)
    emanated = max(2, count)
    hidden = max(1, count - 1)
    root_n = hidden + 1
    assert root_n == 2
    witness_id = authority_input["gate0_hidden_count_witness_case_id"]
    witness = next(
        item
        for item in receipt["evidence"]["owner_law"]["certificates"]
        if item["case_id"] == witness_id
    )
    assert witness["derived_H"] == hidden
    assert spec.resolved_hidden_edge_count == hidden

    radical_receipt = receipt["evidence"]["radical_binding"]
    authority_receipt = radical_receipt["owner_density_authority"]
    expected_authority = {
        **authority_input,
        "corner_relation_id": relation.corner_relation_id.value,
        "angle_certificate_id": certificate.certificate_id.value,
        "certified_u": f(certified_u),
        "q": q,
        "derived_C": count,
        "derived_E": emanated,
        "derived_H": hidden,
        "derived_root_n": root_n,
    }
    assert authority_receipt == expected_authority
    assert radical_receipt["root_n"] == root_n

    source_receipt = receipt["evidence"]["production_source"]
    expected_source = rebuild_production_source(
        snapshot,
        compilation,
        metric,
        spec,
        relation,
        sector,
        certificate,
    )
    assert set(source_receipt) == set(expected_source)
    assert source_receipt == expected_source
    matrix = matrix_from(metric)
    expected_matrix = tuple(
        tuple(Fraction(value) for value in row)
        for row in source["expected_gram"]
    )
    assert matrix == expected_matrix
    binding = compilation.evaluation_geometry_binding
    coordinates = {
        item.source_vertex_id.value: point(item.domain_coordinate)
        for item in binding.source_vertex_coordinates
    }
    incoming = chain_vector(
        snapshot, coordinates, source["incoming_chain_use_id"]
    )
    outgoing = chain_vector(
        snapshot, coordinates, source["outgoing_chain_use_id"]
    )
    assert sign(cross(incoming, outgoing)) == -1
    signed_cosine, aa, bb, ab, product = derive_signed_cos_squared(
        matrix,
        incoming,
        outgoing,
        source["expected_signed_cos_squared"],
        audit,
    )
    generated = radical_receipt["original"]
    assert generated["gram"] == [[f(x) for x in row] for row in matrix]
    assert generated["incoming"] == [f(x) for x in incoming]
    assert generated["outgoing"] == [f(x) for x in outgoing]
    assert {
        key: generated["cosine"][key]
        for key in ("$type", "turn_sign", "cos_squared")
    } == signed_cosine
    assert generated["cosine"]["dot"] == f(ab)
    assert generated["cosine"]["norm_product"] == f(product)

    mirrored_coordinates = {
        key: (value[0], -value[1]) for key, value in coordinates.items()
    }
    mirrored_matrix = (
        (matrix[0][0], -matrix[0][1]),
        (-matrix[1][0], matrix[1][1]),
    )
    mirrored_incoming = chain_vector(
        snapshot, mirrored_coordinates, source["incoming_chain_use_id"]
    )
    mirrored_outgoing = chain_vector(
        snapshot, mirrored_coordinates, source["outgoing_chain_use_id"]
    )
    assert sign(cross(mirrored_incoming, mirrored_outgoing)) == 1
    mirrored_signed_cosine, _, _, m_dot, m_product = derive_signed_cos_squared(
        mirrored_matrix,
        mirrored_incoming,
        mirrored_outgoing,
        source["expected_signed_cos_squared"],
        audit,
    )
    assert mirrored_signed_cosine == signed_cosine
    generated_mirror = radical_receipt["mirrored"]
    assert generated_mirror["gram"] == [
        [f(x) for x in row] for row in mirrored_matrix
    ]
    assert generated_mirror["incoming"] == [
        f(x) for x in mirrored_incoming
    ]
    assert generated_mirror["outgoing"] == [
        f(x) for x in mirrored_outgoing
    ]
    assert {
        key: generated_mirror["cosine"][key]
        for key in ("$type", "turn_sign", "cos_squared")
    } == mirrored_signed_cosine
    assert generated_mirror["cosine"]["dot"] == f(m_dot)
    assert generated_mirror["cosine"]["norm_product"] == f(m_product)

    lower, upper = (
        Fraction(value) for value in source["principal_half_tangent_bracket"]
    )
    other_lower, other_upper = (
        Fraction(value)
        for value in source["other_branch_half_tangent_bracket"]
    )
    principal_signs = [
        root_sign(lower, root_n, signed_cosine, audit, trace),
        root_sign(upper, root_n, signed_cosine, audit, trace),
    ]
    other_signs = [
        root_sign(other_lower, root_n, signed_cosine, audit, trace),
        root_sign(other_upper, root_n, signed_cosine, audit, trace),
    ]
    assert 0 < lower < upper < 1
    assert principal_signs == [1, -1]
    # Independent uniqueness argument: derivative numerator
    # -16*t*(1-t^2) is strictly negative on this whole domain.
    assert lower > 0 and upper < 1
    assert 1 < other_lower < other_upper
    assert other_signs == [-1, 1]
    refined = refine(
        lower,
        upper,
        root_n,
        signed_cosine,
        source["root_refinement_steps"],
        audit,
        trace,
    )

    assert generated["root_equation"] == (
        "T_n((1-t^2)/(1+t^2))=signed_sqrt(cos_squared)"
    )
    assert generated["root_predicate"] == {
        "n": root_n,
        "selected_sign_branch": signed_cosine["turn_sign"],
        "comparison": (
            "sign(T_n(x))=turn_sign; compare T_n(x)^2 to cos_squared"
        ),
        "arithmetic": "EXACT_FRACTION",
    }
    assert generated["principal_bracket_signs"] == principal_signs
    assert generated["refined_bracket"] == [f(x) for x in refined]
    assert generated["branch_certificate"]["principal_monotone"] == (
        "STRICTLY_DECREASING"
    )
    assert generated["branch_certificate"]["principal_unique_root"]
    assert generated["branch_certificate"]["other_branch_has_root"]
    assert generated["branch_certificate"]["other_branch_rejection"] == (
        "OUTSIDE_OWNER_SUBTURN_BRANCH"
    )

    mirrored_refined = refine(
        lower,
        upper,
        root_n,
        mirrored_signed_cosine,
        source["root_refinement_steps"],
        audit,
        trace,
    )
    assert mirrored_refined == refined
    # Repeat the same sign/bracket work for the independently mirrored input.
    assert (
        root_sign(lower, root_n, mirrored_signed_cosine, audit, trace) == 1
    )
    assert (
        root_sign(upper, root_n, mirrored_signed_cosine, audit, trace) == -1
    )
    assert (
        root_sign(
            other_lower, root_n, mirrored_signed_cosine, audit, trace
        )
        == -1
    )
    assert (
        root_sign(
            other_upper, root_n, mirrored_signed_cosine, audit, trace
        )
        == 1
    )
    assert generated_mirror["root_equation"] == generated["root_equation"]
    assert (
        generated_mirror["root_predicate"] == generated["root_predicate"]
    )
    assert generated_mirror["refined_bracket"] == [f(x) for x in refined]

    # Reach the two mirror red controls independently.
    wrong_metric_dot = dot(matrix, mirrored_incoming, mirrored_outgoing)
    assert wrong_metric_dot != m_dot
    reversal_cross = cross(mirrored_outgoing, mirrored_incoming)
    assert sign(reversal_cross) == -1
    return {
        "principal_signs": principal_signs,
        "principal_unique_by_strict_derivative": True,
        "other_branch_signs": other_signs,
        "other_branch_rejected": True,
        "refined_bracket": [f(x) for x in refined],
        "production_signed_cos_squared": signed_cosine,
        "derived_owner_density": {
            "certified_u": f(certified_u),
            "density": density,
            "C": count,
            "E": emanated,
            "H": hidden,
            "root_n": root_n,
        },
        "off_diagonal_gram": matrix[0][1] != 0,
        "mirror_root_identity": mirrored_refined == refined,
        "mirror_wrong_metric_red_reached": True,
        "reversal_red_reached": True,
        "_wrong_metric_dot": f(wrong_metric_dot),
        "_correct_mirror_dot": f(m_dot),
        "_reversal_cross": f(reversal_cross),
    }


def semantic_verify(
    root: Path,
    inputs: dict[str, Any],
    expected_inputs_sha: str,
    wrapper: dict[str, Any],
    trace: RootSolveTrace | None = None,
) -> dict[str, Any]:
    if trace is None:
        trace = RootSolveTrace()
    validate_inputs_contract(inputs)
    exact_keys(
        wrapper,
        {"schema", "evidence_sha256", "evidence"},
        "radical receipt wrapper",
    )
    assert wrapper["schema"] == (
        "cftuv.envelope.dens_a_prime.radical_receipt_wrapper.v1"
    )
    evidence = wrapper["evidence"]
    exact_keys(
        evidence,
        {
            "schema",
            "inputs_sha256",
            "product_tree",
            "owner_law",
            "production_source",
            "radical_binding",
            "red_controls",
            "performance",
            "claims",
        },
        "radical receipt evidence",
    )
    radical_binding = evidence["radical_binding"]
    for orientation in ("original", "mirrored"):
        cosine = radical_binding[orientation]["cosine"]
        exact_keys(
            cosine,
            COSINE_RECEIPT_KEYS,
            f"{orientation} signed cosine receipt",
        )
        validate_signed_cos_squared_record(
            {
                key: cosine[key]
                for key in ("$type", "turn_sign", "cos_squared")
            }
        )
    assert sha(canonical(evidence)) == wrapper["evidence_sha256"]
    assert evidence["inputs_sha256"] == expected_inputs_sha
    input_bits = input_integer_bits(inputs)
    input_limit = inputs["bit_cost_model"]["input_integer_bit_limit"]
    assert input_bits <= input_limit

    observed_tree = {
        "proof_base_revision": inputs["proof_base_revision"],
        "kernel_src": git_oid(root, "HEAD:kernel/src"),
        "cftuv": git_oid(root, "HEAD:cftuv"),
    }
    assert git_is_ancestor(root, inputs["proof_base_revision"])
    assert observed_tree["kernel_src"] == inputs["accepted_product_oids"][
        "kernel_src"
    ]
    assert observed_tree["cftuv"] == inputs["accepted_product_oids"]["cftuv"]
    assert evidence["product_tree"] == observed_tree

    audit = Audit()
    owner = verify_owner(inputs, wrapper)
    radical = verify_radical(root, inputs, wrapper, audit, trace, owner)
    budget = inputs["bit_cost_model"]["budget"]
    assert audit.cost() <= budget
    primary_perf = evidence["performance"]
    expected_perf = {
        "model": "SCHOOLBOOK_INTEGER_BIT_OPS_UPPER_BOUND_V1",
        "exact_proof_step_count": audit.steps,
        "max_integer_bit_length": audit.max_bits,
        "fraction_arithmetic_operation_upper_bound": audit.steps * 256,
        "schoolbook_integer_bit_ops_upper_bound": audit.cost(),
        "budget": budget,
        "pass": audit.cost() <= budget,
        "input_max_integer_bit_length": input_bits,
        "input_integer_bit_limit": input_limit,
    }
    assert primary_perf == expected_perf

    expected_red = {
        "unreachable_owner_H": {
            "reached": True,
            "outcome": "UNREACHABLE_H_CLAIM",
        },
        "other_root_branch": {
            "reached": True,
            "signs": radical["other_branch_signs"],
            "outcome": "OUTSIDE_OWNER_SUBTURN_BRANCH",
        },
        "mirror_without_metric_transform": {
            "reached": True,
            "wrong_dot": radical["_wrong_metric_dot"],
            "correct_dot": radical["_correct_mirror_dot"],
            "outcome": "MIRROR_GRAM_COVARIANCE_VIOLATION",
        },
        "mirrored_input_reversal": {
            "reached": True,
            "reversed_cross": radical["_reversal_cross"],
            "expected_orientation_sign": 1,
            "outcome": "OWNER_ORIENTATION_REVERSAL",
        },
    }
    assert evidence["red_controls"] == expected_red
    expected_claims = {
        "owner_H1_through_H5_reachable": owner["hidden_counts"]
        == [1, 2, 3, 4, 5],
        "H1_C1_and_C2_covered": owner["H1_C_cells"] == [1, 2],
        "production_signed_cos_squared": (
            radical["production_signed_cos_squared"]
            == inputs["production_radical"]["expected_signed_cos_squared"]
        ),
        "principal_branch_unique": radical[
            "principal_unique_by_strict_derivative"
        ],
        "other_branch_rejected": radical["other_branch_rejected"],
        "mirrored_input_off_diagonal_gram": radical[
            "off_diagonal_gram"
        ],
        "mirror_root_identity": radical["mirror_root_identity"],
        "all_red_controls_reached": all(
            item["reached"] for item in expected_red.values()
        ),
        "bit_budget_pass": expected_perf["pass"],
    }
    assert evidence["claims"] == expected_claims
    public_radical = {
        key: value
        for key, value in radical.items()
        if not key.startswith("_")
    }
    return {
        "product_tree": observed_tree,
        "owner_law": owner,
        "radical_binding": public_radical,
        "red_controls": expected_red,
        "performance": expected_perf,
        "root_solve_trace": {
            "endpoint_predicate_calls": trace.endpoint_predicate_calls,
        },
    }


def refresh_digest(wrapper: dict[str, Any]) -> None:
    wrapper["evidence_sha256"] = sha(canonical(wrapper["evidence"]))


def expect_reject(
    root: Path,
    inputs: dict[str, Any],
    inputs_sha: str,
    wrapper: dict[str, Any],
) -> tuple[str, int]:
    trace = RootSolveTrace()
    try:
        semantic_verify(root, inputs, inputs_sha, wrapper, trace)
    except (AssertionError, KeyError, StopIteration, ValueError) as exc:
        return type(exc).__name__, trace.endpoint_predicate_calls
    raise AssertionError("forged receipt was accepted")


def main() -> None:
    args = arguments()
    root = args.source_root.resolve()
    inputs_bytes = args.inputs.read_bytes()
    inputs = json.loads(inputs_bytes)
    receipt_bytes = args.receipt.read_bytes()
    wrapper = json.loads(receipt_bytes)
    verified = semantic_verify(root, inputs, sha(inputs_bytes), wrapper)

    mutation_results: dict[str, str] = {}
    mutation_root_calls: dict[str, int] = {}

    def record_rejection(
        name: str,
        changed_inputs: dict[str, Any],
        changed_inputs_sha: str,
        changed_wrapper: dict[str, Any],
        *,
        require_zero_root_calls: bool = False,
    ) -> None:
        outcome, root_calls = expect_reject(
            root,
            changed_inputs,
            changed_inputs_sha,
            changed_wrapper,
        )
        if require_zero_root_calls:
            assert root_calls == 0
        mutation_results[name] = outcome
        mutation_root_calls[name] = root_calls

    stale = deepcopy(wrapper)
    stale["evidence"]["claims"]["principal_branch_unique"] = False
    record_rejection(
        "stale_digest",
        inputs,
        sha(inputs_bytes),
        stale,
    )

    mutations: dict[str, dict[str, Any]] = {}
    forged = deepcopy(wrapper)
    forged["evidence"]["radical_binding"]["root_n"] = 999
    mutations["root_n_999"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["radical_binding"]["owner_density_authority"][
        "gate0_hidden_count_witness_case_id"
    ] = "H5_C6"
    mutations["foreign_H_case"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["red_controls"] = {}
    mutations["empty_red_controls"] = forged
    for claim in (
        "all_red_controls_reached",
        "other_branch_rejected",
        "production_signed_cos_squared",
        "bit_budget_pass",
    ):
        forged = deepcopy(wrapper)
        forged["evidence"]["claims"][claim] = False
        mutations[f"false_claim_{claim}"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["owner_sector"][
        "ordered_incident_chain_use_ids"
    ][0] = (
        "host-v0:chain-use:foreign"
    )
    mutations["wrong_incident_chain"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["corner_relation"][
        "source_vertex_id"
    ] = (
        "host-vertex:foreign"
    )
    mutations["wrong_relation_anchor"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["angular_spec"][
        "angular_spec_id"
    ] = "angular-spec:foreign"
    mutations["foreign_angular_spec"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["patch_domain"][
        "patch_domain_id"
    ] = "host-v0:patch-domain:foreign"
    mutations["foreign_patch_domain"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["reference_metric"][
        "reference_metric_id"
    ] = "reference-metric:foreign"
    mutations["foreign_reference_metric"] = forged
    for name, section in (
        ("zero_snapshot_hash", "snapshot"),
        ("zero_request_hash", "request"),
        ("zero_metric_hash", "reference_metric"),
    ):
        forged = deepcopy(wrapper)
        key = (
            "codec_sha256"
            if section == "reference_metric"
            else "canonical_sha256"
        )
        forged["evidence"]["production_source"][section][key] = "0" * 64
        mutations[name] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["reference_metric"][
        "frame_selection_law"
    ] = "FOREIGN_FRAME_LAW"
    mutations["altered_reference_metric_law"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["reference_metric"][
        "grid_snapping_law"
    ] = "FOREIGN_LATTICE_SNAP_LAW"
    mutations["altered_lattice_law"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["evaluation_geometry_binding"][
        "binding_law"
    ] = "FOREIGN_EVALUATION_BINDING_LAW"
    mutations["altered_evaluation_binding_law"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["evaluation_geometry_binding"][
        "lattice_scale"
    ] += 1
    mutations["altered_lattice_scale"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["performance"][
        "schoolbook_integer_bit_ops_upper_bound"
    ] = 0
    mutations["perf_upper_bound_zero"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["performance"]["max_integer_bit_length"] = 10**9
    mutations["huge_bit_receipt"] = forged
    for name, mutated in mutations.items():
        refresh_digest(mutated)
        record_rejection(
            name,
            inputs,
            sha(inputs_bytes),
            mutated,
        )

    # Receipt fields are witnesses only: even a structurally valid opposite
    # sign must lose to aa/bb/ab derivation before the first root predicate.
    for orientation in ("original", "mirrored"):
        forged = deepcopy(wrapper)
        forged["evidence"]["radical_binding"][orientation]["cosine"][
            "turn_sign"
        ] = "NEGATIVE"
        refresh_digest(forged)
        record_rejection(
            f"receipt_only_opposite_sign_{orientation}",
            inputs,
            sha(inputs_bytes),
            forged,
            require_zero_root_calls=True,
        )

    def rehashed_input_red(
        name: str,
        path: tuple,
        value: Any,
        *,
        require_zero_root_calls: bool = False,
    ) -> None:
        changed_inputs = deepcopy(inputs)
        cursor = changed_inputs
        for part in path[:-1]:
            cursor = cursor[part]
        cursor[path[-1]] = value
        changed_sha = sha(canonical(changed_inputs))
        changed_wrapper = deepcopy(wrapper)
        changed_wrapper["evidence"]["inputs_sha256"] = changed_sha
        refresh_digest(changed_wrapper)
        record_rejection(
            name,
            changed_inputs,
            changed_sha,
            changed_wrapper,
            require_zero_root_calls=require_zero_root_calls,
        )

    authority_path = ("production_radical", "density_authority")
    rehashed_input_red(
        "rehashed_bogus_authority_type",
        (*authority_path, "$type"),
        "BOGUS_AUTHORITY",
    )
    rehashed_input_red(
        "rehashed_bogus_density_parameter",
        (*authority_path, "density_parameter_id"),
        "BOGUS_PARAMETER",
    )
    rehashed_input_red(
        "rehashed_density_4_value_with_integer_0",
        (*authority_path, "density_value_id"),
        "LINEAR_REFLEX_DENSITY_4_V1",
    )
    rehashed_input_red(
        "rehashed_density_exact_angle_mismatch",
        (*authority_path, "max_subturn_exact_value", "symbol"),
        "PI_OVER_6",
    )
    rehashed_input_red(
        "rehashed_foreign_reflex_excess_source",
        (*authority_path, "reflex_excess_source"),
        "FOREIGN_REFLEX_EXCESS_SOURCE",
    )
    rehashed_input_red(
        "rehashed_foreign_gate0_witness",
        (*authority_path, "gate0_hidden_count_witness_case_id"),
        "FOREIGN_GATE0_WITNESS",
    )
    rehashed_input_red(
        "rehashed_same_H_foreign_gate0_witness",
        (*authority_path, "gate0_hidden_count_witness_case_id"),
        "H1_C2",
    )
    rehashed_input_red(
        "rehashed_owner_policy",
        ("owner_law", "selection_policy_id"),
        "FOREIGN_OWNER_POLICY",
    )
    rehashed_input_red(
        "rehashed_owner_parameter",
        ("owner_law", "density_parameter_id"),
        "FOREIGN_OWNER_PARAMETER",
    )
    rehashed_input_red(
        "rehashed_owner_formula",
        ("owner_law", "formula", "H"),
        "max(0,C-1)",
    )
    rehashed_input_red(
        "rehashed_owner_row",
        ("owner_law", "certificates", 5, "expected_H"),
        4,
    )
    rehashed_input_red(
        "rehashed_foreign_angular_spec",
        ("production_radical", "angular_spec_id"),
        "angular-spec:foreign",
    )
    rehashed_input_red(
        "rehashed_foreign_patch_domain",
        ("production_radical", "patch_domain_id"),
        "host-v0:patch-domain:foreign",
    )
    rehashed_input_red(
        "rehashed_foreign_reference_metric",
        ("production_radical", "reference_metric_id"),
        "reference-metric:foreign",
    )
    for name, field in (
        ("rehashed_zero_snapshot_hash", "snapshot_sha256"),
        ("rehashed_zero_request_hash", "request_sha256"),
        ("rehashed_zero_metric_hash", "metric_codec_sha256"),
    ):
        rehashed_input_red(
            name,
            ("production_radical", field),
            "0" * 64,
        )

    def coupled_cosine_red(
        name: str,
        mutated_record: dict[str, Any],
    ) -> None:
        changed_inputs = deepcopy(inputs)
        changed_inputs["production_radical"][
            "expected_signed_cos_squared"
        ] = deepcopy(mutated_record)
        changed_sha = sha(canonical(changed_inputs))
        changed_wrapper = deepcopy(wrapper)
        changed_wrapper["evidence"]["inputs_sha256"] = changed_sha
        for orientation in ("original", "mirrored"):
            cosine = changed_wrapper["evidence"]["radical_binding"][
                orientation
            ]["cosine"]
            for key, value in mutated_record.items():
                cosine[key] = value
        refresh_digest(changed_wrapper)
        record_rejection(
            name,
            changed_inputs,
            changed_sha,
            changed_wrapper,
            require_zero_root_calls=True,
        )

    signed_cosine = inputs["production_radical"][
        "expected_signed_cos_squared"
    ]
    opposite_sign = deepcopy(signed_cosine)
    opposite_sign["turn_sign"] = "NEGATIVE"
    coupled_cosine_red(
        "rehashed_signed_cosine_opposite_sign",
        opposite_sign,
    )

    unknown_type = deepcopy(signed_cosine)
    unknown_type["$type"] = "ProofOnlySignedNormalizedRadicalV1"
    coupled_cosine_red(
        "rehashed_unknown_signed_cosine_type",
        unknown_type,
    )

    unknown_form = deepcopy(signed_cosine)
    unknown_form["form"] = "SIGNED_SQUAREFREE_RADICAL"
    coupled_cosine_red(
        "rehashed_unknown_legacy_radical_form",
        unknown_form,
    )

    unknown_sign = deepcopy(signed_cosine)
    unknown_sign["turn_sign"] = "CLOCKWISE"
    coupled_cosine_red(
        "rehashed_unknown_turn_sign_enum",
        unknown_sign,
    )

    zero_sign_with_nonzero_square = deepcopy(signed_cosine)
    zero_sign_with_nonzero_square["turn_sign"] = "ZERO"
    coupled_cosine_red(
        "rehashed_zero_sign_with_nonzero_square",
        zero_sign_with_nonzero_square,
    )

    current_squared = Fraction(signed_cosine["cos_squared"])
    out_of_range_squared = deepcopy(signed_cosine)
    out_of_range_squared["cos_squared"] = "2"
    coupled_cosine_red(
        "rehashed_cos_squared_out_of_range",
        out_of_range_squared,
    )

    unreduced_squared = deepcopy(signed_cosine)
    unreduced_squared["cos_squared"] = (
        f"{2 * current_squared.numerator}/"
        f"{2 * current_squared.denominator}"
    )
    coupled_cosine_red(
        "rehashed_unreduced_cos_squared",
        unreduced_squared,
    )

    # Старый scale-equivalent обход не представим в SignedCosSquaredV1:
    # сама obsolete-запись отвергается exact-keyset до root predicate.
    coupled_cosine_red(
        "rehashed_legacy_scaled_radical_pair",
        {
            "$type": "ProofOnlyScaledQuadraticRadicalV1",
            "coefficient": (
                "1002251807/10034548322036468254965133066"
            ),
            "radicand": "20069096644072936509930266132",
        },
    )

    # Неанонсированный deterministic-random RED: один бит числителя.
    random_bitflip = deepcopy(signed_cosine)
    random_bitflip["cos_squared"] = str(
        Fraction(
            current_squared.numerator ^ 1,
            current_squared.denominator,
        )
    )
    coupled_cosine_red(
        "rehashed_random_cos_squared_numerator_bitflip",
        random_bitflip,
    )

    unknown_inputs = deepcopy(inputs)
    unknown_inputs["accepted_product_oids"]["kernel_src"] = "0" * 40
    unknown_wrapper = deepcopy(wrapper)
    unknown_sha = sha(canonical(unknown_inputs))
    unknown_wrapper["evidence"]["inputs_sha256"] = unknown_sha
    refresh_digest(unknown_wrapper)
    record_rejection(
        "unknown_product_tree",
        unknown_inputs,
        unknown_sha,
        unknown_wrapper,
    )

    huge_inputs = deepcopy(inputs)
    huge_inputs["production_radical"]["expected_signed_cos_squared"][
        "cos_squared"
    ] = (
        "1/" + "1" + "0" * 1600
    )
    huge_wrapper = deepcopy(wrapper)
    huge_sha = sha(canonical(huge_inputs))
    huge_wrapper["evidence"]["inputs_sha256"] = huge_sha
    refresh_digest(huge_wrapper)
    record_rejection(
        "huge_bit_input",
        huge_inputs,
        huge_sha,
        huge_wrapper,
    )

    invalid_signed_cosine_cases = (
        "receipt_only_opposite_sign_original",
        "receipt_only_opposite_sign_mirrored",
        "rehashed_signed_cosine_opposite_sign",
        "rehashed_unknown_signed_cosine_type",
        "rehashed_unknown_legacy_radical_form",
        "rehashed_unknown_turn_sign_enum",
        "rehashed_zero_sign_with_nonzero_square",
        "rehashed_cos_squared_out_of_range",
        "rehashed_unreduced_cos_squared",
        "rehashed_legacy_scaled_radical_pair",
        "rehashed_random_cos_squared_numerator_bitflip",
    )
    invalid_signed_cosine_trace = {
        name: mutation_root_calls[name]
        for name in invalid_signed_cosine_cases
    }
    assert all(value == 0 for value in invalid_signed_cosine_trace.values())

    late_post_root_classes = {
        "empty_red_controls": "RED_CONTROL_RECEIPT",
        "false_claim_all_red_controls_reached": "CLAIM_RECEIPT",
        "false_claim_bit_budget_pass": "CLAIM_RECEIPT",
        "false_claim_other_branch_rejected": "CLAIM_RECEIPT",
        "false_claim_production_signed_cos_squared": "CLAIM_RECEIPT",
        "huge_bit_receipt": "PERFORMANCE_RECEIPT",
        "perf_upper_bound_zero": "PERFORMANCE_RECEIPT",
    }
    nonzero_call_cases = {
        name
        for name, calls in mutation_root_calls.items()
        if calls != 0
    }
    assert nonzero_call_cases == set(late_post_root_classes)
    valid_root_calls = verified["root_solve_trace"][
        "endpoint_predicate_calls"
    ]
    late_post_root_trace = {
        name: {
            "classification": classification,
            "endpoint_predicate_calls": mutation_root_calls[name],
        }
        for name, classification in late_post_root_classes.items()
    }
    assert all(
        row["endpoint_predicate_calls"] == valid_root_calls
        for row in late_post_root_trace.values()
    )

    output = {
        "schema": "cftuv.envelope.dens_a_prime.radical_verification.v1",
        "verified_receipt_sha256": sha(receipt_bytes),
        "verified_evidence_sha256": wrapper["evidence_sha256"],
        **verified,
        "exploit_regression_matrix": mutation_results,
        "exploit_root_solve_call_counts": mutation_root_calls,
        "invalid_signed_cos_squared_zero_call_trace": {
            "endpoint_predicate_calls": invalid_signed_cosine_trace,
            "all_zero": True,
        },
        "late_post_root_receipt_red_trace": {
            "expected_valid_endpoint_predicate_calls": valid_root_calls,
            "cases": late_post_root_trace,
        },
        "all_exploit_regressions_rejected": all(
            value in {"AssertionError", "KeyError", "StopIteration", "ValueError"}
            for value in mutation_results.values()
        ),
        "verdict": "VERIFIED",
        "scope_note": "proof receipt only; verifier does not authorize product GO",
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_bytes(pretty(output))


if __name__ == "__main__":
    main()
