"""Independent verifier for the DENS-A-prime owner/radical receipt.

It intentionally does not import generate_receipt.py.  All exact predicates,
fixture extraction, mirroring, root isolation, and red controls are repeated
here from the receipt inputs.
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


def sqrt_sum_sign(
    rational: Fraction,
    coefficient: Fraction,
    radicand: int,
    audit: Audit,
) -> int:
    if radicand <= 0:
        raise AssertionError("radicand must be positive")
    audit.record(1, rational, coefficient, radicand)
    if coefficient == 0:
        return sign(rational)
    if rational == 0:
        return sign(coefficient)
    if sign(rational) == sign(coefficient):
        return sign(rational)
    left = rational * rational
    right = coefficient * coefficient * radicand
    audit.record(1, left, right)
    comparison = sign(left - right)
    return comparison if rational > 0 else -comparison


def cos_two(t: Fraction) -> Fraction:
    square = t * t
    return (1 - 6 * square + square * square) / (
        1 + 2 * square + square * square
    )


def root_sign(
    t: Fraction, coefficient: Fraction, radicand: int, audit: Audit
) -> int:
    value = cos_two(t)
    audit.record(4, t, value)
    return sqrt_sum_sign(value, -coefficient, radicand, audit)


def refine(
    lower: Fraction,
    upper: Fraction,
    coefficient: Fraction,
    radicand: int,
    steps: int,
    audit: Audit,
) -> tuple[Fraction, Fraction]:
    assert root_sign(lower, coefficient, radicand, audit) == 1
    assert root_sign(upper, coefficient, radicand, audit) == -1
    for _ in range(steps):
        middle = (lower + upper) / 2
        if root_sign(middle, coefficient, radicand, audit) > 0:
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


def derive_cosine(matrix, incoming, outgoing, expected, audit: Audit):
    aa = dot(matrix, incoming, incoming)
    bb = dot(matrix, outgoing, outgoing)
    ab = dot(matrix, incoming, outgoing)
    product = aa * bb
    coefficient = Fraction(expected["expected_cosine_coefficient"])
    radicand = int(expected["expected_cosine_radicand"])
    audit.record(12, matrix, incoming, outgoing, aa, bb, ab, product)
    assert aa > 0 and bb > 0 and ab > 0
    assert coefficient * coefficient * radicand == ab * ab / product
    return coefficient, radicand, aa, bb, ab, product


def verify_owner(inputs, receipt) -> dict[str, Any]:
    observed = receipt["evidence"]["owner_law"]["certificates"]
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
        assert row["crossing_cell"]["uncertainty"] == "NONE_EXACT_RATIONAL"
        hidden.add(h)
        if h == 1:
            h1_c.add(c)
    assert hidden == {1, 2, 3, 4, 5}
    assert h1_c == {1, 2}
    return {"hidden_counts": sorted(hidden), "H1_C_cells": sorted(h1_c)}


def verify_radical(
    root, inputs, receipt, audit: Audit, owner_summary
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
    density = int(authority_input["density_integer"])
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
    assert source_receipt["anchor_source_vertex_id"] == (
        relation.source_vertex_id.value
    )
    assert source_receipt["relation_source_vertex_id"] == (
        relation.source_vertex_id.value
    )
    actual_incident = [
        item.value for item in sector.ordered_incident_chain_use_ids
    ]
    assert source_receipt["ordered_incident_chain_use_ids"] == actual_incident
    assert source_receipt["incoming_chain_use_id"] == actual_incident[0]
    assert source_receipt["outgoing_chain_use_id"] == actual_incident[1]
    assert source_receipt["angle_certificate_id"] == (
        certificate.certificate_id.value
    )
    legacy = source_receipt["legacy_compile_observation"]
    assert legacy == {
        "selection_certificate_id": spec.selection_certificate_id.value,
        "resolved_hidden_edge_count": spec.resolved_hidden_edge_count,
        "separate_from_proof_density_authority": True,
        "compatible_with_proof_density_hidden_count": True,
    }
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
    coefficient, radicand, aa, bb, ab, product = derive_cosine(
        matrix, incoming, outgoing, source, audit
    )
    lower, upper = (
        Fraction(value) for value in source["principal_half_tangent_bracket"]
    )
    other_lower, other_upper = (
        Fraction(value)
        for value in source["other_branch_half_tangent_bracket"]
    )
    principal_signs = [
        root_sign(lower, coefficient, radicand, audit),
        root_sign(upper, coefficient, radicand, audit),
    ]
    other_signs = [
        root_sign(other_lower, coefficient, radicand, audit),
        root_sign(other_upper, coefficient, radicand, audit),
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
        coefficient,
        radicand,
        source["root_refinement_steps"],
        audit,
    )

    generated = radical_receipt["original"]
    assert generated["gram"] == [[f(x) for x in row] for row in matrix]
    assert generated["incoming"] == [f(x) for x in incoming]
    assert generated["outgoing"] == [f(x) for x in outgoing]
    assert generated["cosine"]["coefficient"] == f(coefficient)
    assert generated["cosine"]["radicand"] == radicand
    assert generated["cosine"]["dot"] == f(ab)
    assert generated["cosine"]["norm_product"] == f(product)
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
    m_coefficient, m_radicand, _, _, m_dot, m_product = derive_cosine(
        mirrored_matrix,
        mirrored_incoming,
        mirrored_outgoing,
        source,
        audit,
    )
    assert (m_coefficient, m_radicand) == (coefficient, radicand)
    mirrored_refined = refine(
        lower,
        upper,
        m_coefficient,
        m_radicand,
        source["root_refinement_steps"],
        audit,
    )
    assert mirrored_refined == refined
    # Repeat the same sign/bracket work for the independently mirrored input.
    assert root_sign(lower, m_coefficient, m_radicand, audit) == 1
    assert root_sign(upper, m_coefficient, m_radicand, audit) == -1
    assert root_sign(other_lower, m_coefficient, m_radicand, audit) == -1
    assert root_sign(other_upper, m_coefficient, m_radicand, audit) == 1
    generated_mirror = radical_receipt["mirrored"]
    assert generated_mirror["gram"] == [
        [f(x) for x in row] for row in mirrored_matrix
    ]
    assert generated_mirror["incoming"] == [f(x) for x in mirrored_incoming]
    assert generated_mirror["outgoing"] == [f(x) for x in mirrored_outgoing]
    assert generated_mirror["cosine"]["dot"] == f(m_dot)
    assert generated_mirror["cosine"]["norm_product"] == f(m_product)
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
        "production_radicand": radicand,
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
) -> dict[str, Any]:
    assert wrapper["schema"] == (
        "cftuv.envelope.dens_a_prime.radical_receipt_wrapper.v1"
    )
    evidence = wrapper["evidence"]
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
    radical = verify_radical(root, inputs, wrapper, audit, owner)
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
        "production_quadratic_radical": (
            radical["production_radicand"]
            == int(
                inputs["production_radical"][
                    "expected_cosine_radicand"
                ]
            )
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
    }


def refresh_digest(wrapper: dict[str, Any]) -> None:
    wrapper["evidence_sha256"] = sha(canonical(wrapper["evidence"]))


def expect_reject(
    root: Path,
    inputs: dict[str, Any],
    inputs_sha: str,
    wrapper: dict[str, Any],
) -> str:
    try:
        semantic_verify(root, inputs, inputs_sha, wrapper)
    except (AssertionError, KeyError, StopIteration, ValueError) as exc:
        return type(exc).__name__
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

    stale = deepcopy(wrapper)
    stale["evidence"]["claims"]["principal_branch_unique"] = False
    mutation_results["stale_digest"] = expect_reject(
        root, inputs, sha(inputs_bytes), stale
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
        "production_quadratic_radical",
        "bit_budget_pass",
    ):
        forged = deepcopy(wrapper)
        forged["evidence"]["claims"][claim] = False
        mutations[f"false_claim_{claim}"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["incoming_chain_use_id"] = (
        "host-v0:chain-use:foreign"
    )
    mutations["wrong_incident_chain"] = forged
    forged = deepcopy(wrapper)
    forged["evidence"]["production_source"]["anchor_source_vertex_id"] = (
        "host-vertex:foreign"
    )
    mutations["wrong_relation_anchor"] = forged
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
        mutation_results[name] = expect_reject(
            root, inputs, sha(inputs_bytes), mutated
        )

    unknown_inputs = deepcopy(inputs)
    unknown_inputs["accepted_product_oids"]["kernel_src"] = "0" * 40
    unknown_wrapper = deepcopy(wrapper)
    unknown_sha = sha(canonical(unknown_inputs))
    unknown_wrapper["evidence"]["inputs_sha256"] = unknown_sha
    refresh_digest(unknown_wrapper)
    mutation_results["unknown_product_tree"] = expect_reject(
        root, unknown_inputs, unknown_sha, unknown_wrapper
    )

    huge_inputs = deepcopy(inputs)
    huge_inputs["production_radical"]["expected_cosine_coefficient"] = (
        "1" + "0" * 1600
    )
    huge_wrapper = deepcopy(wrapper)
    huge_sha = sha(canonical(huge_inputs))
    huge_wrapper["evidence"]["inputs_sha256"] = huge_sha
    refresh_digest(huge_wrapper)
    mutation_results["huge_bit_input"] = expect_reject(
        root, huge_inputs, huge_sha, huge_wrapper
    )

    output = {
        "schema": "cftuv.envelope.dens_a_prime.radical_verification.v1",
        "verified_receipt_sha256": sha(receipt_bytes),
        "verified_evidence_sha256": wrapper["evidence_sha256"],
        **verified,
        "exploit_regression_matrix": mutation_results,
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
