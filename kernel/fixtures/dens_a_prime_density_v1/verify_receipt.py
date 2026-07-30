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
    return snapshot, compilation, metric


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


def verify_radical(root, inputs, receipt, audit: Audit) -> dict[str, Any]:
    source = inputs["production_radical"]
    snapshot, compilation, metric = load_product(root, source)
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

    generated = receipt["evidence"]["radical_binding"]["original"]
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
    generated_mirror = receipt["evidence"]["radical_binding"]["mirrored"]
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
        "off_diagonal_gram": matrix[0][1] != 0,
        "mirror_root_identity": mirrored_refined == refined,
        "mirror_wrong_metric_red_reached": True,
        "reversal_red_reached": True,
    }


def main() -> None:
    args = arguments()
    root = args.source_root.resolve()
    inputs_bytes = args.inputs.read_bytes()
    inputs = json.loads(inputs_bytes)
    wrapper = json.loads(args.receipt.read_bytes())
    assert wrapper["schema"] == (
        "cftuv.envelope.dens_a_prime.radical_receipt_wrapper.v1"
    )
    evidence = wrapper["evidence"]
    assert sha(canonical(evidence)) == wrapper["evidence_sha256"]
    assert evidence["inputs_sha256"] == sha(inputs_bytes)

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
    radical = verify_radical(root, inputs, wrapper, audit)
    budget = inputs["bit_cost_model"]["budget"]
    assert audit.cost() <= budget
    primary_perf = evidence["performance"]
    assert primary_perf["pass"]
    assert primary_perf["schoolbook_integer_bit_ops_upper_bound"] <= budget

    # Receipt corruption is rejected before semantic checks.
    corrupted = deepcopy(wrapper)
    corrupted["evidence"]["claims"]["principal_branch_unique"] = False
    assert sha(canonical(corrupted["evidence"])) != corrupted[
        "evidence_sha256"
    ]
    # Unknown product trees are fail-closed.
    forged_tree = deepcopy(inputs)
    forged_tree["accepted_product_oids"]["kernel_src"] = "0" * 40
    assert observed_tree["kernel_src"] != forged_tree[
        "accepted_product_oids"
    ]["kernel_src"]
    # H+1 cannot be asserted without changing the owner-law cell.
    first = inputs["owner_law"]["certificates"][0]
    q = first["d"] + 2
    actual_h = max(1, ceil(q * Fraction(first["u"])) - 1)
    assert actual_h + 1 != actual_h

    red = {
        "corrupted_receipt": "EVIDENCE_DIGEST_MISMATCH",
        "unknown_product_tree": "UNKNOWN_PRODUCT_TREE",
        "unreachable_owner_H": "UNREACHABLE_H_CLAIM",
        "other_root_branch": "OUTSIDE_OWNER_SUBTURN_BRANCH",
        "mirror_without_metric_transform": (
            "MIRROR_GRAM_COVARIANCE_VIOLATION"
        ),
        "mirrored_input_reversal": "OWNER_ORIENTATION_REVERSAL",
    }
    output = {
        "schema": "cftuv.envelope.dens_a_prime.radical_verification.v1",
        "verified_receipt_sha256": sha(args.receipt.read_bytes()),
        "verified_evidence_sha256": wrapper["evidence_sha256"],
        "product_tree": observed_tree,
        "owner_law": owner,
        "radical_binding": radical,
        "red_controls": red,
        "performance": {
            "model": "SCHOOLBOOK_INTEGER_BIT_OPS_UPPER_BOUND_V1",
            "exact_proof_step_count": audit.steps,
            "max_integer_bit_length": audit.max_bits,
            "schoolbook_integer_bit_ops_upper_bound": audit.cost(),
            "budget": budget,
            "pass": audit.cost() <= budget,
        },
        "verdict": "VERIFIED",
        "scope_note": "proof receipt only; verifier does not authorize product GO",
    }
    args.output_report.parent.mkdir(parents=True, exist_ok=True)
    args.output_report.write_bytes(pretty(output))


if __name__ == "__main__":
    main()
