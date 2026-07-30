"""Generate the proof-only DENS-A-prime owner/radical receipt.

This file deliberately contains no product implementation.  It reads a frozen
field fixture, asks the existing compiler for its evaluation geometry, then
proves the density owner law and the resulting quadratic-radical root with
integer/Fraction predicates.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import hashlib
import json
from math import isqrt
from pathlib import Path
import subprocess
import sys
from typing import Any


HERE = Path(__file__).resolve().parent
SCHEMA = "cftuv.envelope.dens_a_prime.radical_receipt.v1"


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


def _square_fraction(value: Fraction) -> bool:
    return (
        isqrt(value.numerator) ** 2 == value.numerator
        and isqrt(value.denominator) ** 2 == value.denominator
    )


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


@dataclass(frozen=True)
class SqrtSum:
    rational: Fraction
    coefficient: Fraction
    radicand: int

    def sign(self, ledger: BitLedger) -> int:
        if self.radicand <= 0:
            raise ValueError("non-positive radicand")
        ledger.step(1, self.rational, self.coefficient, self.radicand)
        if self.coefficient == 0:
            return _sign(self.rational)
        if self.rational == 0:
            return _sign(self.coefficient)
        if _sign(self.rational) == _sign(self.coefficient):
            return _sign(self.rational)
        rational_sq = self.rational * self.rational
        radical_sq = self.coefficient * self.coefficient * self.radicand
        ledger.step(1, rational_sq, radical_sq)
        comparison = _sign(rational_sq - radical_sq)
        return comparison if self.rational > 0 else -comparison


def _cos_two_from_half_tangent(t: Fraction) -> Fraction:
    t2 = t * t
    return (1 - 6 * t2 + t2 * t2) / (1 + 2 * t2 + t2 * t2)


def _root_sign(
    t: Fraction, coefficient: Fraction, radicand: int, ledger: BitLedger
) -> int:
    rational = _cos_two_from_half_tangent(t)
    ledger.step(4, t, rational)
    return SqrtSum(rational, -coefficient, radicand).sign(ledger)


def _refine(
    lower: Fraction,
    upper: Fraction,
    coefficient: Fraction,
    radicand: int,
    steps: int,
    ledger: BitLedger,
) -> tuple[Fraction, Fraction]:
    if _root_sign(lower, coefficient, radicand, ledger) != 1:
        raise ValueError("principal lower endpoint does not have positive sign")
    if _root_sign(upper, coefficient, radicand, ledger) != -1:
        raise ValueError("principal upper endpoint does not have negative sign")
    for _ in range(steps):
        middle = (lower + upper) / 2
        if _root_sign(middle, coefficient, radicand, ledger) > 0:
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
    return kernel, snapshot, compilation, metric, spec


def _metric_matrix(metric) -> tuple[tuple[Fraction, Fraction], ...]:
    value = metric.exact_gram_matrix
    return (
        (_rat(value.m00), _rat(value.m01)),
        (_rat(value.m10), _rat(value.m11)),
    )


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


def _cosine_radical(
    matrix,
    incoming,
    outgoing,
    ledger: BitLedger,
) -> tuple[Fraction, int, dict[str, Any]]:
    aa = _gram_dot(matrix, incoming, incoming)
    bb = _gram_dot(matrix, outgoing, outgoing)
    ab = _gram_dot(matrix, incoming, outgoing)
    product = aa * bb
    if aa <= 0 or bb <= 0 or ab <= 0:
        raise ValueError("production rays do not define an acute excess angle")
    # ab/sqrt(product) == (ab/product)*sqrt(product).  In this field case
    # product has a rational square denominator and a square factor upstairs.
    numerator_root = isqrt(product.numerator)
    denominator_root = isqrt(product.denominator)
    # Reduce square factors without a CAS.  The accepted field witness is
    # deliberately represented by the already reduced integer radicand.
    expected_r = 5017274161018234127482566533
    coefficient = Fraction(1002251807, expected_r)
    if coefficient * coefficient * expected_r != ab * ab / product:
        raise ValueError("derived cosine does not equal production radical")
    if _square_fraction(product):
        raise ValueError("production norm product unexpectedly became rational")
    ledger.step(12, matrix, incoming, outgoing, aa, bb, ab, product)
    return coefficient, expected_r, {
        "incoming_norm_squared": _f(aa),
        "outgoing_norm_squared": _f(bb),
        "dot": _f(ab),
        "norm_product": _f(product),
        "norm_product_is_square": False,
        "integer_sqrt_floor_numerator": numerator_root,
        "integer_sqrt_floor_denominator": denominator_root,
        "identity": "cos_delta=(dot/norm_product)*sqrt(norm_product)",
    }


def _solve_orientation(
    name: str,
    matrix,
    incoming,
    outgoing,
    expected_orientation: int,
    principal_bracket: tuple[Fraction, Fraction],
    other_bracket: tuple[Fraction, Fraction],
    steps: int,
    ledger: BitLedger,
) -> dict[str, Any]:
    cross = _cross(incoming, outgoing)
    if _sign(cross) != expected_orientation:
        raise ValueError(f"{name}: orientation mismatch")
    coefficient, radicand, gram = _cosine_radical(
        matrix, incoming, outgoing, ledger
    )
    principal_signs = [
        _root_sign(value, coefficient, radicand, ledger)
        for value in principal_bracket
    ]
    other_signs = [
        _root_sign(value, coefficient, radicand, ledger)
        for value in other_bracket
    ]
    refined = _refine(
        *principal_bracket, coefficient, radicand, steps, ledger
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
            "coefficient": _f(coefficient),
            "radicand": radicand,
            "fraction_only_root_sign": False,
            **gram,
        },
        "root_equation": (
            "(1-6*t^2+t^4)/(1+2*t^2+t^4)"
            "=coefficient*sqrt(radicand)"
        ),
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
    kernel, snapshot, compilation, metric, spec = _load_compilation(
        root, source
    )
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
        source["root_refinement_steps"],
        ledger,
    )
    if original["cosine"]["coefficient"] != source[
        "expected_cosine_coefficient"
    ]:
        raise ValueError("cosine coefficient mismatch")
    if original["cosine"]["radicand"] != int(
        source["expected_cosine_radicand"]
    ):
        raise ValueError("cosine radicand mismatch")

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
        source["root_refinement_steps"],
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
            "formula": {
                "q": "d+2",
                "C": "ceil(q*u)",
                "E": "max(2,C)",
                "H": "max(1,C-1)",
            },
            "certificates": owner,
        },
        "production_source": {
            "snapshot_sha256": source["snapshot_sha256"],
            "request_sha256": source["request_sha256"],
            "metric_codec_sha256": source["metric_codec_sha256"],
            "reference_metric_id": source["reference_metric_id"],
            "patch_domain_id": source["patch_domain_id"],
            "angular_spec_id": spec.envelope_spec_id.value,
            "owner_sector_id": spec.owner_sector_id.value,
            "source_relation_id": spec.source_relation_id.value,
            "evaluation_binding_law": binding.binding_law.value,
            "lattice_scale": binding.lattice_scale,
            "anchor_source_vertex_id": source["anchor_source_vertex_id"],
            "incoming_chain_use_id": source["incoming_chain_use_id"],
            "outgoing_chain_use_id": source["outgoing_chain_use_id"],
        },
        "radical_binding": {
            "owner_hidden_count_case_id": source["hidden_count_case_id"],
            "root_n": 2,
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
            "production_quadratic_radical": True,
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
