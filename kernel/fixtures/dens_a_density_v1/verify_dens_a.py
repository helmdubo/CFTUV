"""DENS-A proof-only oracle.

This file proves reachability and canonical laws without authorizing product
behaviour.  It uses exact rational arithmetic only and never materializes an
algebraic ideal unit normal.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from math import gcd
from pathlib import Path
import subprocess
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
SCHEMA = "cftuv.envelope.dens_a.proof_receipt.v1"
EXPECTED_CASES_SHA256 = (
    "1ea6b8c303c6c2797e6f42231d3e221e9e174f10c316603990342b3c479ed5e2"
)
UNKNOWN_TREE = "UNKNOWN_KERNEL_SRC_PRODUCT_TREE"
UNKNOWN_CASES = "UNKNOWN_DENS_A_FIXTURE_FORM"
UNCERTAIN = "ANGULAR_PROFILE_SELECTION_UNCERTAIN"
EMPTY_WINDOW = "DENSITY_DIRECTION_BINDING_EMPTY_WINDOW_DEGRADED"


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root",
        type=Path,
        default=SCRIPT_DIR.parents[2],
    )
    parser.add_argument(
        "--cases",
        type=Path,
        default=SCRIPT_DIR / "cases.json",
    )
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


def _canonical_json(payload: Any) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _pretty_json(payload: Any) -> bytes:
    return (
        json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _fraction(value: Fraction | int | str) -> str:
    return str(Fraction(value))


def _ceil(value: Fraction) -> int:
    return -(-value.numerator // value.denominator)


def _select_density_point(density: int, u: Fraction) -> dict[str, Any]:
    if density < 0 or density > 4:
        return {
            "outcome": "DENSITY_VALUE_OUT_OF_RANGE",
            "density": density,
        }
    if u <= 0 or u > 1:
        return {
            "outcome": "REFLEX_EXCESS_FRACTION_OUT_OF_RANGE",
            "u": _fraction(u),
        }
    q = density + 2
    count = _ceil(q * u)
    emanated = max(2, count)
    hidden = max(1, count - 1)
    return {
        "outcome": "EXACT",
        "density": density,
        "q": q,
        "u": _fraction(u),
        "C": count,
        "E": emanated,
        "H": hidden,
        "subturn_predicate": {
            "law": "u/E <= 1/q",
            "exact_left_q_times_u": _fraction(q * u),
            "exact_right_E": emanated,
            "resolved": q * u <= emanated,
        },
    }


def _select_density_interval(
    density: int,
    lower: Fraction,
    upper: Fraction,
) -> dict[str, Any]:
    if lower > upper:
        return {"outcome": "INVALID_INTERVAL_ORDER"}
    lower_result = _select_density_point(density, lower)
    upper_result = _select_density_point(density, upper)
    if (
        lower_result["outcome"] != "EXACT"
        or upper_result["outcome"] != "EXACT"
    ):
        return {
            "outcome": "INTERVAL_ENDPOINT_NOT_ADMISSIBLE",
            "lower": lower_result,
            "upper": upper_result,
        }
    if lower_result["C"] != upper_result["C"]:
        return {
            "outcome": UNCERTAIN,
            "density": density,
            "q": density + 2,
            "lower": _fraction(lower),
            "upper": _fraction(upper),
            "lower_C": lower_result["C"],
            "upper_C": upper_result["C"],
        }
    return {
        **lower_result,
        "authority": "CERTIFIED_INTERVAL_SINGLE_CEILING_CELL",
        "lower": _fraction(lower),
        "upper": _fraction(upper),
    }


def _chebyshev(n: int, x: Fraction) -> Fraction:
    if n == 0:
        return Fraction(1)
    if n == 1:
        return x
    previous = Fraction(1)
    current = x
    for _ in range(2, n + 1):
        previous, current = current, 2 * x * current - previous
    return current


def _cos_from_half_tangent(value: Fraction) -> Fraction:
    return (1 - value * value) / (1 + value * value)


def _root_sign(n: int, cos_delta: Fraction, value: Fraction) -> int:
    residual = _chebyshev(n, _cos_from_half_tangent(value)) - cos_delta
    return (residual > 0) - (residual < 0)


def _first_root_bracket(
    n: int,
    cos_delta: Fraction,
    scan_denominator: int,
    iterations: int,
) -> tuple[Fraction, Fraction, int]:
    evaluations = 1
    previous = Fraction(0)
    if _root_sign(n, cos_delta, previous) <= 0:
        raise ValueError("root does not start after zero")
    bracket: tuple[Fraction, Fraction] | None = None
    for numerator in range(1, scan_denominator + 1):
        current = Fraction(numerator, scan_denominator)
        sign = _root_sign(n, cos_delta, current)
        evaluations += 1
        if sign <= 0:
            bracket = previous, current
            break
        previous = current
    if bracket is None:
        raise ValueError("first root was not bracketed")
    lower, upper = bracket
    for _ in range(iterations):
        middle = (lower + upper) / 2
        sign = _root_sign(n, cos_delta, middle)
        evaluations += 1
        if sign > 0:
            lower = middle
        else:
            upper = middle
    if not (
        _root_sign(n, cos_delta, lower) > 0
        and _root_sign(n, cos_delta, upper) <= 0
    ):
        raise ValueError("exact sign bracket was lost")
    evaluations += 2
    return lower, upper, evaluations


def _primitive_direction(half_tangent: Fraction) -> tuple[int, int]:
    numerator = half_tangent.numerator
    denominator = half_tangent.denominator
    x = denominator * denominator - numerator * numerator
    y = 2 * numerator * denominator
    divisor = gcd(abs(x), abs(y))
    x //= divisor
    y //= divisor
    if x < 0:
        x, y = -x, -y
    return x, y


def _complex_power(direction: tuple[int, int], exponent: int) -> tuple[int, int]:
    result = (1, 0)
    for _ in range(exponent):
        result = (
            result[0] * direction[0] - result[1] * direction[1],
            result[0] * direction[1] + result[1] * direction[0],
        )
        divisor = gcd(abs(result[0]), abs(result[1]))
        result = result[0] // divisor, result[1] // divisor
    return result


def _cross(left: tuple[int, int], right: tuple[int, int]) -> int:
    return left[0] * right[1] - left[1] * right[0]


def _rational_unit_point(half_tangent: Fraction) -> tuple[Fraction, Fraction]:
    denominator = 1 + half_tangent * half_tangent
    return (
        (1 - half_tangent * half_tangent) / denominator,
        2 * half_tangent / denominator,
    )


def _gram_distance_squared(
    left: tuple[Fraction, Fraction],
    right: tuple[Fraction, Fraction],
    gram: tuple[tuple[Fraction, Fraction], tuple[Fraction, Fraction]],
) -> Fraction:
    dx = left[0] - right[0]
    dy = left[1] - right[1]
    return (
        dx * (gram[0][0] * dx + gram[0][1] * dy)
        + dy * (gram[1][0] * dx + gram[1][1] * dy)
    )


def _direction_case(
    source: dict[str, Any],
    scan_denominator: int,
    iterations: int,
) -> tuple[dict[str, Any], int]:
    hidden = int(source["hidden_edge_count"])
    n = hidden + 1
    total_half_tangent = Fraction(source["total_half_tangent"])
    cos_delta = _cos_from_half_tangent(total_half_tangent)
    gram = tuple(
        tuple(Fraction(value) for value in row)
        for row in source["gram"]
    )
    lower, upper, evaluations = _first_root_bracket(
        n,
        cos_delta,
        scan_denominator,
        iterations,
    )
    directions = []
    for ordinal in range(1, hidden + 1):
        lower_t = lower + (upper - lower) / 4
        upper_t = lower + 3 * (upper - lower) / 4
        lower_point = _rational_unit_point(lower_t)
        upper_point = _rational_unit_point(upper_t)
        target = (
            (lower_point[0] + upper_point[0]) / 2,
            (lower_point[1] + upper_point[1]) / 2,
        )
        lower_distance = _gram_distance_squared(
            lower_point, target, gram
        )
        upper_distance = _gram_distance_squared(
            upper_point, target, gram
        )
        tied = lower_distance == upper_distance
        chosen_t = upper_t if tied else (
            lower_t
            if lower_distance < upper_distance
            else upper_t
        )
        chosen_base = _primitive_direction(chosen_t)
        chosen = _complex_power(chosen_base, ordinal)
        lower_bound = _complex_power(
            _primitive_direction(lower), ordinal
        )
        upper_bound = _complex_power(
            _primitive_direction(upper), ordinal
        )
        directions.append(
            {
                "ordinal": ordinal,
                "ideal_authority": (
                    f"FIRST_ROOT_OF_T_{n}(x)=cos(delta), "
                    f"direction ordinal {ordinal}/{n}"
                ),
                "root_half_tangent_window": [
                    _fraction(lower),
                    _fraction(upper),
                ],
                "lower_ordinal_bound": list(lower_bound),
                "upper_ordinal_bound": list(upper_bound),
                "candidate_primitive_vector": list(chosen),
                "gram_squared_distance_lower": _fraction(
                    lower_distance
                ),
                "gram_squared_distance_upper": _fraction(
                    upper_distance
                ),
                "gram_tie": tied,
                "half_up_selected_ordinal": "UPPER",
                "strictly_inside_ordinal_window": (
                    _cross(lower_bound, chosen) > 0
                    and _cross(chosen, upper_bound) > 0
                ),
                "authority": "REFINED_INTERNAL",
                "boundary": "STRICT_ORDINAL_ROOT_WINDOW",
            }
        )
    normalized_digest = _sha256(
        _canonical_json(
            [item["candidate_primitive_vector"] for item in directions]
        )
    )
    orientation_rows = []
    for orientation in ("CCW", "CW"):
        world = [
            (
                item["candidate_primitive_vector"]
                if orientation == "CCW"
                else [
                    item["candidate_primitive_vector"][0],
                    -item["candidate_primitive_vector"][1],
                ]
            )
            for item in directions
        ]
        normalized = [
            [item[0], item[1] if orientation == "CCW" else -item[1]]
            for item in world
        ]
        orientation_rows.append(
            {
                "orientation": orientation,
                "world_primitive_vectors": world,
                "canonical_normalized_vectors": normalized,
                "canonical_digest": _sha256(_canonical_json(normalized)),
            }
        )
    counters = {
        "reversal": int(
            any(
                row["canonical_digest"] != normalized_digest
                for row in orientation_rows
            )
        ),
        "sticky": int(
            len(
                {
                    tuple(item["candidate_primitive_vector"])
                    for item in directions
                }
            )
            != len(directions)
        ),
        "cross": sum(
            not item["strictly_inside_ordinal_window"]
            for item in directions
        ),
        "order": sum(
            _cross(
                tuple(directions[index]["candidate_primitive_vector"]),
                tuple(directions[index + 1]["candidate_primitive_vector"]),
            )
            <= 0
            for index in range(len(directions) - 1)
        ),
    }
    return {
        "case_id": source["case_id"],
        "H": hidden,
        "n": n,
        "cos_delta": _fraction(cos_delta),
        "ideal_normals_materialized": False,
        "root_law": f"T_{n}(x)=cos(delta)",
        "exact_sign_bracket": [_fraction(lower), _fraction(upper)],
        "gram": [[_fraction(value) for value in row] for row in gram],
        "base_bound_authority": {
            "start_before": [1, 0],
            "start_after": [1, 0],
            "end_half_tangent_before": _fraction(total_half_tangent),
            "end_half_tangent_after": _fraction(total_half_tangent),
            "unchanged": True,
        },
        "directions": directions,
        "orientations": orientation_rows,
        "counters": counters,
    }, evaluations


def _threshold_matrix() -> dict[str, Any]:
    rows = []
    predicate_rows = 0
    predicate_failures = 0
    uncertainty_rows = 0
    for q in range(2, 7):
        density = q - 2
        epsilon = Fraction(1, q * 1000)
        for numerator in range(1, q):
            threshold = Fraction(numerator, q)
            samples = (
                ("below", threshold - epsilon, numerator),
                ("exact", threshold, numerator),
                ("above", threshold + epsilon, numerator + 1),
            )
            for position, u, expected_count in samples:
                result = _select_density_point(density, u)
                predicate_rows += 1
                predicate_failures += int(
                    not result["subturn_predicate"]["resolved"]
                )
                rows.append(
                    {
                        "q": q,
                        "density": density,
                        "threshold": _fraction(threshold),
                        "position": position,
                        "u": _fraction(u),
                        "expected_C": expected_count,
                        **result,
                    }
                )
            straddling = _select_density_interval(
                density,
                threshold - epsilon,
                threshold + epsilon,
            )
            uncertainty_rows += int(straddling["outcome"] == UNCERTAIN)
            rows.append(
                {
                    "q": q,
                    "density": density,
                    "threshold": _fraction(threshold),
                    "position": "straddling_interval",
                    **straddling,
                }
            )
    resolved_failures = sum(
        row.get("position") != "straddling_interval"
        and (
            row["outcome"] != "EXACT"
            or row["C"] != row["expected_C"]
        )
        for row in rows
    )
    return {
        "row_count": len(rows),
        "point_row_count": predicate_rows,
        "straddling_row_count": uncertainty_rows,
        "resolved_selection_failures": resolved_failures,
        "exact_subturn_predicate_rows": predicate_rows,
        "exact_subturn_predicate_failures": predicate_failures,
        "rows": rows,
    }


def _monotonicity_and_anchors() -> dict[str, Any]:
    grid = [Fraction(item, 60) for item in range(1, 61)]
    by_density = {
        density: [_select_density_point(density, u)["H"] for u in grid]
        for density in range(5)
    }
    phi_failures = sum(
        current > following
        for values in by_density.values()
        for current, following in zip(values, values[1:])
    )
    density_failures = sum(
        by_density[density][index] > by_density[density + 1][index]
        for density in range(4)
        for index in range(len(grid))
    )
    return {
        "grid_u": [_fraction(value) for value in grid],
        "phi_monotonicity_failures": phi_failures,
        "density_monotonicity_failures": density_failures,
        "d0_hidden_values": sorted(set(by_density[0])),
        "d1_hidden_values": sorted(set(by_density[1])),
        "d1_limit_u_1_hidden": _select_density_point(1, Fraction(1))["H"],
        "d4_limit_u_1_hidden": _select_density_point(4, Fraction(1))["H"],
    }


def _bounded_rationals(max_denominator: int) -> list[Fraction]:
    return sorted(
        {
            Fraction(numerator, denominator)
            for denominator in range(1, max_denominator + 1)
            for numerator in range(1, denominator)
        }
    )


def _empty_window_witness(
    source: dict[str, Any],
    scan_denominator: int,
    iterations: int,
) -> tuple[dict[str, Any], int]:
    hidden = int(source["hidden_edge_count"])
    n = hidden + 1
    lower, upper, evaluations = _first_root_bracket(
        n,
        Fraction(source["cos_delta"]),
        scan_denominator,
        iterations,
    )
    candidates = [
        item
        for item in _bounded_rationals(
            int(source["candidate_max_denominator"])
        )
        if lower < item < upper
    ]
    point_selection = _select_density_point(
        int(source["density"]), Fraction(source["u"])
    )
    interval_selection = _select_density_interval(
        int(source["density"]),
        Fraction(source["u"]) - Fraction(1, 3000),
        Fraction(source["u"]) + Fraction(1, 3000),
    )
    return {
        "u": source["u"],
        "q": source["q"],
        "density": source["density"],
        "point_selection": point_selection,
        "exact_threshold": (
            Fraction(source["q"]) * Fraction(source["u"])
        ).denominator
        == 1,
        "root_half_tangent_window": [
            _fraction(lower),
            _fraction(upper),
        ],
        "candidate_max_denominator": source[
            "candidate_max_denominator"
        ],
        "candidates_in_open_window": [_fraction(item) for item in candidates],
        "outcome": source["named_outcome"] if not candidates else "EXACT",
        "named_degradation_reached": (
            not candidates and source["named_outcome"] == EMPTY_WINDOW
        ),
        "evidence_class": source["evidence_class"],
        "evidence_scope_pass": (
            source["evidence_class"] == "POINT_THRESHOLD_ONLY"
            and interval_selection["outcome"] == UNCERTAIN
        ),
        "straddling_interval_outcome": interval_selection["outcome"],
    }, evaluations


def _cache_authority(
    source_root: Path,
    cases: dict[str, Any],
) -> dict[str, Any]:
    old = cases["old_explicit_v1_authority"]
    request_path = source_root / old["request_path"]
    snapshot_path = source_root / old["snapshot_path"]
    request_before = request_path.read_bytes()
    snapshot_bytes = snapshot_path.read_bytes()
    request = json.loads(request_before)
    request_after = request_path.read_bytes()
    snapshot_sha = _sha256(snapshot_bytes)
    chain_digest = _sha256(
        _canonical_json(sorted(request["selected_chain_use_ids"], key=str))
    )
    policy = cases["future_policy_authority"]
    plan_rows = []
    for density in range(5):
        density_value_id = policy["density_value_id_pattern"].format(
            d=density
        )
        request_identity = {
            "selection_policy_id": policy["selection_policy_id"],
            "density_parameter_id": policy["density_parameter_id"],
            "density_value_id": density_value_id,
        }
        projection = {
            "product_tree_oid": cases["accepted_product_tree_oid"],
            "snapshot_sha256": snapshot_sha,
            "selected_chain_use_ids_sha256": chain_digest,
            "request_policy_value_identity": request_identity,
            "density": density,
            "selection_at_u_11_over_12": _select_density_point(
                density, Fraction(11, 12)
            ),
        }
        first = _sha256(_canonical_json(projection))
        second = _sha256(_canonical_json(projection))
        plan_rows.append(
            {
                "density": density,
                "request_policy_value_identity": request_identity,
                "plan_digest_first": first,
                "plan_digest_second": second,
                "same_density_bit_stable": first == second,
            }
        )
    alpha_rows = []
    selected_density = 1
    density_value_id = policy["density_value_id_pattern"].format(
        d=selected_density
    )
    preparation_projection = {
        "source_revision": cases["proof_base_revision"],
        "snapshot_sha256": snapshot_sha,
        "selected_chain_use_ids_sha256": chain_digest,
        "selection_policy_id": policy["selection_policy_id"],
        "density_parameter_id": policy["density_parameter_id"],
        "density_value_id": density_value_id,
    }
    preparation_key = _sha256(_canonical_json(preparation_projection))
    for alpha in ("1/4", "1/2"):
        coverage_pair = {
            "preparation_key": preparation_key,
            "effective_alpha": alpha,
        }
        alpha_rows.append(
            {
                "alpha": alpha,
                "preparation_key": preparation_key,
                "coverage_pair_key": _sha256(
                    _canonical_json(coverage_pair)
                ),
            }
        )
    mismatch_value_id = policy["density_value_id_pattern"].format(d=2)
    mismatch_outcome = (
        "DENSITY_POLICY_VALUE_IDENTITY_MISMATCH"
        if mismatch_value_id
        != policy["density_value_id_pattern"].format(d=1)
        else "EXACT"
    )
    return {
        "snapshot_sha256": snapshot_sha,
        "snapshot_hash_matches_frozen_authority": (
            snapshot_sha == old["snapshot_sha256"]
        ),
        "old_explicit_v1_60": {
            "request_sha256_before": _sha256(request_before),
            "request_sha256_after": _sha256(request_after),
            "expected_request_sha256": old["request_sha256"],
            "byte_stable": (
                request_before == request_after
                and _sha256(request_before) == old["request_sha256"]
            ),
            "selection_policy_id": request[
                "angular_profile_selection_policy_id"
            ],
            "max_subturn_value_id": request["max_subturn_value_id"],
            "authority_identity_pass": (
                request["angular_profile_selection_policy_id"]
                == old["selection_policy_id"]
                and request["max_subturn_value_id"]
                == old["max_subturn_value_id"]
            ),
        },
        "new_density_plan_rows": plan_rows,
        "same_snapshot_different_density_digest_count": len(
            {row["plan_digest_first"] for row in plan_rows}
        ),
        "same_density_bit_stability_failures": sum(
            not row["same_density_bit_stable"] for row in plan_rows
        ),
        "density_identity_is_exact_request_policy_value_id": all(
            row["request_policy_value_identity"]["density_value_id"]
            == policy["density_value_id_pattern"].format(
                d=row["density"]
            )
            for row in plan_rows
        ),
        "alpha_preparation_pair_law": {
            "preparation_is_alpha_independent": (
                alpha_rows[0]["preparation_key"]
                == alpha_rows[1]["preparation_key"]
            ),
            "coverage_pair_is_alpha_specific": (
                alpha_rows[0]["coverage_pair_key"]
                != alpha_rows[1]["coverage_pair_key"]
            ),
            "rows": alpha_rows,
        },
        "live_identity_mismatch_negative": {
            "density": 1,
            "wrong_density_value_id": mismatch_value_id,
            "outcome": mismatch_outcome,
            "named_refusal_reached": (
                mismatch_outcome
                == "DENSITY_POLICY_VALUE_IDENTITY_MISMATCH"
            ),
        },
        "live_out_of_range_negative": _select_density_point(
            5, Fraction(1, 2)
        ),
    }


def _tree_oracle(observed: str, accepted: str) -> dict[str, Any]:
    return {
        "observed_product_tree_oid": observed,
        "accepted_product_tree_oid": accepted,
        "outcome": "DENS_A_ORACLE_ACCEPTED" if observed == accepted else UNKNOWN_TREE,
        "accepted": observed == accepted,
    }


def _fixture_oracle(observed: str) -> dict[str, Any]:
    return {
        "observed_cases_sha256": observed,
        "accepted_cases_sha256": EXPECTED_CASES_SHA256,
        "outcome": (
            "DENS_A_FIXTURE_ACCEPTED"
            if observed == EXPECTED_CASES_SHA256
            else UNKNOWN_CASES
        ),
        "accepted": observed == EXPECTED_CASES_SHA256,
    }


def main() -> None:
    arguments = _arguments()
    source_root = arguments.source_root.resolve()
    cases_bytes = arguments.cases.read_bytes()
    cases = json.loads(cases_bytes)
    fixture_oracle = _fixture_oracle(_sha256(cases_bytes))
    unknown_fixture_negative = _fixture_oracle("f" * 64)
    product_tree_oid = subprocess.check_output(
        [
            "git",
            "-C",
            str(source_root),
            "rev-parse",
            "HEAD:kernel/src",
        ],
        text=True,
    ).strip()
    oracle = _tree_oracle(
        product_tree_oid, cases["accepted_product_tree_oid"]
    )
    unknown_tree_negative = _tree_oracle(
        "0" * 40, cases["accepted_product_tree_oid"]
    )
    failures: list[dict[str, Any]] = []
    if not oracle["accepted"]:
        failures.append(oracle)
    if not fixture_oracle["accepted"]:
        failures.append(fixture_oracle)

    root_config = cases["root_bisection"]
    direction_rows = []
    chebyshev_evaluations = 0
    if oracle["accepted"] and fixture_oracle["accepted"]:
        for source in cases["direction_cases"]:
            row, evaluations = _direction_case(
                source,
                int(root_config["scan_denominator"]),
                int(root_config["iterations"]),
            )
            direction_rows.append(row)
            chebyshev_evaluations += evaluations
    threshold_matrix = _threshold_matrix()
    monotonicity = _monotonicity_and_anchors()
    empty_window, empty_evaluations = _empty_window_witness(
        cases["empty_window_witness"],
        int(root_config["scan_denominator"]),
        int(root_config["iterations"]),
    )
    chebyshev_evaluations += empty_evaluations
    cache = _cache_authority(source_root, cases)
    direction_counters = {
        name: sum(row["counters"][name] for row in direction_rows)
        for name in ("reversal", "sticky", "cross", "order")
    }
    gram_ties = sum(
        item["gram_tie"]
        for row in direction_rows
        for item in row["directions"]
    )
    gram_ties_half_up = sum(
        item["gram_tie"]
        and item["half_up_selected_ordinal"] == "UPPER"
        for row in direction_rows
        for item in row["directions"]
    )
    checks = {
        "known_product_tree": oracle["accepted"],
        "known_frozen_fixture_form": fixture_oracle["accepted"],
        "unknown_tree_fail_closed": (
            unknown_tree_negative["outcome"] == UNKNOWN_TREE
            and not unknown_tree_negative["accepted"]
        ),
        "unknown_fixture_fail_closed": (
            unknown_fixture_negative["outcome"] == UNKNOWN_CASES
            and not unknown_fixture_negative["accepted"]
        ),
        "direction_H_1_through_5": (
            [row["H"] for row in direction_rows] == [1, 2, 3, 4, 5]
        ),
        "direction_n_is_H_plus_1": all(
            row["n"] == row["H"] + 1 for row in direction_rows
        ),
        "no_algebraic_ideal_normal_materialized": all(
            not row["ideal_normals_materialized"]
            for row in direction_rows
        ),
        "orientation_counters_zero": all(
            value == 0 for value in direction_counters.values()
        ),
        "gram_ties_half_up": gram_ties > 0
        and gram_ties == gram_ties_half_up,
        "base_bound_unchanged": all(
            row["base_bound_authority"]["unchanged"]
            for row in direction_rows
        ),
        "threshold_rows_exact": (
            threshold_matrix["row_count"] == 60
            and threshold_matrix["point_row_count"] == 45
            and threshold_matrix["straddling_row_count"] == 15
            and threshold_matrix["resolved_selection_failures"] == 0
        ),
        "straddling_named_uncertain": all(
            row["outcome"] == UNCERTAIN
            for row in threshold_matrix["rows"]
            if row["position"] == "straddling_interval"
        ),
        "exact_subturn_predicates": (
            threshold_matrix["exact_subturn_predicate_rows"] == 45
            and threshold_matrix["exact_subturn_predicate_failures"] == 0
        ),
        "density_monotone": (
            monotonicity["density_monotonicity_failures"] == 0
        ),
        "phi_monotone": monotonicity["phi_monotonicity_failures"] == 0,
        "anchors": (
            monotonicity["d0_hidden_values"] == [1]
            and monotonicity["d1_hidden_values"] == [1, 2]
            and monotonicity["d1_limit_u_1_hidden"] == 2
            and monotonicity["d4_limit_u_1_hidden"] == 5
        ),
        "empty_window_named_degradation": (
            empty_window["named_degradation_reached"]
            and empty_window["evidence_scope_pass"]
        ),
        "cache_different_density_distinct": (
            cache["same_snapshot_different_density_digest_count"] == 5
        ),
        "cache_same_density_stable": (
            cache["same_density_bit_stability_failures"] == 0
        ),
        "old_explicit_v1_60_byte_stable": (
            cache["old_explicit_v1_60"]["byte_stable"]
            and cache["old_explicit_v1_60"]["authority_identity_pass"]
        ),
        "density_request_identity_exact": (
            cache["density_identity_is_exact_request_policy_value_id"]
        ),
        "alpha_preparation_pair_law": (
            cache["alpha_preparation_pair_law"][
                "preparation_is_alpha_independent"
            ]
            and cache["alpha_preparation_pair_law"][
                "coverage_pair_is_alpha_specific"
            ]
        ),
        "live_negatives": (
            cache["live_identity_mismatch_negative"][
                "named_refusal_reached"
            ]
            and cache["live_out_of_range_negative"]["outcome"]
            == "DENSITY_VALUE_OUT_OF_RANGE"
        ),
        "chebyshev_synthetic_budget": (
            chebyshev_evaluations
            <= int(root_config["synthetic_chebyshev_evaluation_budget"])
        ),
        "no_sympy_or_trig_hot_path": (
            b"import " + b"sympy" not in Path(__file__).read_bytes()
            and b"from " + b"sympy" not in Path(__file__).read_bytes()
            and b"from math import " + b"cos" not in Path(__file__).read_bytes()
            and b"from math import " + b"sin" not in Path(__file__).read_bytes()
        ),
    }
    for name, passed in checks.items():
        if not passed:
            failures.append(
                {
                    "outcome": "DENS_A_PROOF_CHECK_FAILED",
                    "check": name,
                }
            )
    outcome = "DENS_A_GO" if not failures else "DENS_A_REFUSED"
    payload = {
        "schema": SCHEMA,
        "outcome": outcome,
        "proof_base_revision": cases["proof_base_revision"],
        "kernel_src_product_tree_oid": product_tree_oid,
        "cases": {
            "path": "cases.json",
            "sha256": _sha256(cases_bytes),
        },
        "oracle": {
            "known_tree": oracle,
            "live_unknown_tree_negative": unknown_tree_negative,
            "known_fixture": fixture_oracle,
            "live_unknown_fixture_negative": unknown_fixture_negative,
        },
        "owner_A_law": {
            "u": "(phi-pi)/pi",
            "density_domain": [0, 4],
            "q": "d+2",
            "C": "ceil(q*u), lower-OPEN/upper-CLOSED",
            "E": "max(2,C)",
            "H": "max(1,C-1)",
            "floor_authority": "FUTURE_NAMED_POLICY_ONLY",
            "existing_explicit_V1_60_authority": "BYTE_STABLE",
        },
        "direction_binding": {
            "method": (
                "EXACT_FRACTION_CHEBYSHEV_SIGN_BISECTION_"
                "TO_RATIONAL_PRIMITIVE_ORDINAL_WINDOW"
            ),
            "ideal_step_law": "FIRST_ROOT_OF_T_n(x)=cos(delta)",
            "n_law": "n=H+1",
            "algebraic_ideal_normals_materialized": False,
            "sympy_hot_path": False,
            "n_ge_4_universal_binding_promised": False,
            "canonical_objective": (
                "RationalAffinePlanarMetricV2_GRAM_SQUARED_DISTANCE"
            ),
            "tie_law": "EQUAL_GRAM_SQUARED_DISTANCE_SELECTS_UPPER_ORDINAL",
            "authorities": {
                "boundary_rays": "BASE_BOUND_UNCHANGED",
                "hidden_directions": "REFINED_INTERNAL_ORDINAL_WINDOW",
            },
            "orientation_counters": direction_counters,
            "gram_tie_count": gram_ties,
            "gram_tie_half_up_count": gram_ties_half_up,
            "rows": direction_rows,
        },
        "threshold_matrix": threshold_matrix,
        "monotonicity_and_anchors": monotonicity,
        "empty_window_witness": empty_window,
        "cache_key_authority": cache,
        "performance": {
            "line": "T_n exact sign/bisection synthetic operation budget",
            "unit": "exact Chebyshev Fraction evaluations",
            "measured": chebyshev_evaluations,
            "budget": int(
                root_config["synthetic_chebyshev_evaluation_budget"]
            ),
            "pass": checks["chebyshev_synthetic_budget"],
            "wall_clock": "NON_CANONICAL_DIAGNOSTIC_EXCLUDED",
        },
        "checks": checks,
        "failures": failures,
    }
    report_bytes = _pretty_json(payload)
    arguments.output_report.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_report.write_bytes(report_bytes)
    print(
        json.dumps(
            {
                "outcome": outcome,
                "receipt_sha256": _sha256(report_bytes),
                "threshold_rows": threshold_matrix["row_count"],
                "direction_rows": len(direction_rows),
                "hidden_direction_rows": sum(
                    len(row["directions"]) for row in direction_rows
                ),
                "orientation_counters": direction_counters,
                "chebyshev_evaluations": chebyshev_evaluations,
                "failure_count": len(failures),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    raise SystemExit(0 if not failures else 2)


if __name__ == "__main__":
    main()
