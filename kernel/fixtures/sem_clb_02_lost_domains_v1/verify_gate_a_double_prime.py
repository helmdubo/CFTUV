"""Exact refinement reachability proof for SEM-CLB-02-R Gate A-double-prime.

Fixture evidence only: this script never constructs or mutates product state.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from math import gcd
from pathlib import Path
import sys
from typing import Any


_SCRIPT_DIR = str(Path(__file__).resolve().parent)
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
import verify_gate_a_prime as gate_a


SCHEMA = "cftuv.envelope.sem_clb_02_gate_a_double_prime.v1"
PROOF_BASE_REVISION = "34d6f963a29648f7370f582cf603e5b80e59fadd"
EXPECTED_PRODUCT_TREE_OID = gate_a.EXPECTED_PRODUCT_TREE_OID
FIXTURE_IDS = gate_a.FIXTURE_IDS
R_MAX = 8
FAILURE_COUNTER_NAMES = (
    "norm_k_disagreements",
    "canonicality_failures",
    "strict_order_failures",
    "sticky_k",
    "reversals",
    "nonzero_cross",
    "endpoint_mutations",
    "base_node_mutations",
    "full_boundary_chart_euclidean_failures",
    "full_boundary_rational_affine_metric_failures",
    "longitudinal_chart_euclidean_failures",
    "longitudinal_rational_affine_metric_failures",
    "refinement_budget_refusal_self_check_failures",
    "no_admissible_refinement_domains",
    "unknown_fixture_form",
    "unknown_product_tree",
)
OBSERVATION_COUNTER_NAMES = (
    "capacity_infeasible_chain_attempts",
    "half_ties_chart_euclidean_resolved_up",
    "half_ties_rational_affine_metric_resolved_up",
    "selected_internal_vertices",
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    parser.add_argument(
        "--fixture-root",
        type=Path,
        default=Path(__file__).resolve().parent / "cases",
    )
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


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


def _fraction(value: Fraction | int) -> str:
    return str(Fraction(value))


def _failure(
    counters: dict[str, int],
    failures: list[dict[str, Any]],
    counter: str,
    outcome: str,
    detail: str,
    **identity: Any,
) -> None:
    counters[counter] += 1
    failures.append(
        {
            "outcome": outcome,
            "detail": detail,
            **{name: value for name, value in identity.items() if value is not None},
        }
    )


def _half_up(value: Fraction) -> tuple[int, bool]:
    lower = value.numerator // value.denominator
    residual = value - lower
    tie = residual == Fraction(1, 2)
    return (lower if residual < Fraction(1, 2) else lower + 1), tie


def _gram_dot(first, second, gram) -> Fraction:
    return gate_a._gram_dot(first, second, gram)


def _dot(first, second) -> Fraction:
    return gate_a._dot(first, second)


def _subtract(first, second) -> tuple[Fraction, Fraction]:
    return gate_a._subtract(first, second)


def _projection(
    source_scaled: tuple[Fraction, Fraction],
    anchor: tuple[int, int],
    direction: tuple[int, int],
    gram,
) -> Fraction:
    offset = _subtract(source_scaled, anchor)
    if gram is None:
        numerator = _dot(direction, offset)
        denominator = _dot(direction, direction)
    else:
        numerator = _gram_dot(direction, offset, gram)
        denominator = _gram_dot(direction, direction, gram)
    if denominator <= 0:
        raise ValueError("projection has non-positive line period")
    return numerator / denominator


def _quadratic_objective(
    k: int,
    projection: Fraction,
    period_squared_scaled: Fraction,
) -> Fraction:
    residual = Fraction(k) - projection
    return period_squared_scaled * residual * residual


def _assign_norm(
    source: tuple[tuple[Fraction, Fraction], ...],
    base_start_node: tuple[int, int],
    direction: tuple[int, int],
    endpoint_span: int,
    base_scale: int,
    r: int,
    gram,
    norm_name: str,
) -> dict[str, Any]:
    factor = 1 << r
    scale = base_scale * factor
    anchor = (
        base_start_node[0] * factor,
        base_start_node[1] * factor,
    )
    endpoint_k = endpoint_span * factor
    vertex_count = len(source)
    capacity_required = vertex_count - 1
    if endpoint_k < capacity_required:
        return {
            "norm": norm_name,
            "capacity_feasible": False,
            "capacity_required": capacity_required,
            "endpoint_k": endpoint_k,
            "outcome": "CHAIN_REFINEMENT_CAPACITY_INSUFFICIENT",
            "steps": [],
            "k_sequence": None,
            "canonical": False,
            "strict_order": False,
        }

    direction_period_scaled = (
        _dot(direction, direction)
        if gram is None
        else _gram_dot(direction, direction, gram)
    )
    if direction_period_scaled <= 0:
        raise ValueError("direction has non-positive norm")
    steps: list[dict[str, Any]] = []
    k_sequence = [0]
    previous_k = 0
    all_canonical = True
    for ordinal, source_point in enumerate(source[1:-1], start=1):
        remaining_to_end = vertex_count - 1 - ordinal
        lower = previous_k + 1
        upper = endpoint_k - remaining_to_end
        if lower > upper:
            return {
                "norm": norm_name,
                "capacity_feasible": True,
                "capacity_required": capacity_required,
                "endpoint_k": endpoint_k,
                "outcome": "CHAIN_SEQUENTIAL_INTERVAL_EMPTY",
                "steps": steps,
                "k_sequence": None,
                "canonical": False,
                "strict_order": False,
            }
        source_scaled = source_point[0] * scale, source_point[1] * scale
        projection = _projection(source_scaled, anchor, direction, gram)
        unconstrained, tie = _half_up(projection)
        assigned = min(max(unconstrained, lower), upper)
        expected_clamp = min(max(unconstrained, lower), upper)
        assigned_objective = _quadratic_objective(
            assigned, projection, direction_period_scaled
        )
        lower_neighbor = assigned - 1 if assigned - 1 >= lower else None
        upper_neighbor = assigned + 1 if assigned + 1 <= upper else None
        lower_objective = (
            None
            if lower_neighbor is None
            else _quadratic_objective(
                lower_neighbor, projection, direction_period_scaled
            )
        )
        upper_objective = (
            None
            if upper_neighbor is None
            else _quadratic_objective(
                upper_neighbor, projection, direction_period_scaled
            )
        )
        local_minimum = (
            (lower_objective is None or assigned_objective <= lower_objective)
            and (
                upper_objective is None
                or assigned_objective <= upper_objective
            )
        )
        tie_up_respected = (
            not tie
            or unconstrained != projection.numerator // projection.denominator
        )
        canonical = (
            assigned == expected_clamp
            and lower <= assigned <= upper
            and local_minimum
            and tie_up_respected
        )
        all_canonical = all_canonical and canonical
        steps.append(
            {
                "ordinal": ordinal,
                "lower": lower,
                "upper": upper,
                "remaining_vertices_to_end_endpoint": remaining_to_end,
                "projection_k": _fraction(projection),
                "unconstrained_nearest_half_up_k": unconstrained,
                "exact_half_tie": tie,
                "assigned_k": assigned,
                "clamp_equivalent": assigned == expected_clamp,
                "assigned_longitudinal_objective_scaled": _fraction(
                    assigned_objective
                ),
                "lower_neighbor_k": lower_neighbor,
                "lower_neighbor_objective_scaled": (
                    None
                    if lower_objective is None
                    else _fraction(lower_objective)
                ),
                "upper_neighbor_k": upper_neighbor,
                "upper_neighbor_objective_scaled": (
                    None
                    if upper_objective is None
                    else _fraction(upper_objective)
                ),
                "canonicality_verified": canonical,
            }
        )
        k_sequence.append(assigned)
        previous_k = assigned
    k_sequence.append(endpoint_k)
    gaps = [
        current - previous
        for previous, current in zip(k_sequence, k_sequence[1:])
    ]
    return {
        "norm": norm_name,
        "capacity_feasible": True,
        "capacity_required": capacity_required,
        "endpoint_k": endpoint_k,
        "outcome": (
            "EXACT_ASSIGNMENT"
            if all_canonical and all(gap > 0 for gap in gaps)
            else "ASSIGNMENT_CERTIFICATE_INVALID"
        ),
        "steps": steps,
        "k_sequence": k_sequence,
        "consecutive_k_gaps": gaps,
        "canonical": all_canonical,
        "strict_order": all(gap > 0 for gap in gaps),
        "min_k_gap": min(gaps),
    }


def _boundary_for_assignment(
    source: tuple[tuple[Fraction, Fraction], ...],
    source_vertex_ids: tuple,
    base_start_node: tuple[int, int],
    direction: tuple[int, int],
    base_scale: int,
    r: int,
    assignment: dict[str, Any],
    gram,
) -> list[dict[str, Any]]:
    factor = 1 << r
    scale = base_scale * factor
    anchor = (
        base_start_node[0] * factor,
        base_start_node[1] * factor,
    )
    euclidean_period_squared = Fraction(_dot(direction, direction), scale * scale)
    gram_period_squared = _gram_dot(direction, direction, gram) / (
        scale * scale
    )
    half_euclidean_squared = euclidean_period_squared / 4
    half_gram_squared = gram_period_squared / 4
    if assignment["k_sequence"] is None:
        selected_vertices = (
            (0, source_vertex_ids[0], source[0], 0),
            (
                len(source) - 1,
                source_vertex_ids[-1],
                source[-1],
                assignment["endpoint_k"],
            ),
        )
    else:
        selected_vertices = tuple(
            (ordinal, source_vertex_id, source_point, k)
            for ordinal, (source_vertex_id, source_point, k) in enumerate(
                zip(source_vertex_ids, source, assignment["k_sequence"])
            )
        )
    result = []
    for ordinal, source_vertex_id, source_point, k in selected_vertices:
        assigned_node = (
            anchor[0] + k * direction[0],
            anchor[1] + k * direction[1],
        )
        assigned_coordinate = (
            Fraction(assigned_node[0], scale),
            Fraction(assigned_node[1], scale),
        )
        displacement = _subtract(assigned_coordinate, source_point)
        euclidean_squared = _dot(displacement, displacement)
        gram_squared = _gram_dot(displacement, displacement, gram)
        source_scaled = source_point[0] * scale, source_point[1] * scale
        projection_euclidean = _projection(
            source_scaled, anchor, direction, None
        )
        projection_gram = _projection(
            source_scaled, anchor, direction, gram
        )
        residual_euclidean = abs(Fraction(k) - projection_euclidean)
        residual_gram = abs(Fraction(k) - projection_gram)
        cross = gate_a._cross(
            (
                assigned_node[0] - anchor[0],
                assigned_node[1] - anchor[1],
            ),
            direction,
        )
        internal = ordinal not in (0, len(source) - 1)
        result.append(
            {
                "ordinal": ordinal,
                "source_vertex_id": source_vertex_id.value,
                "is_endpoint": not internal,
                "assigned_k": k,
                "assigned_refined_node": list(assigned_node),
                "assigned_coordinate": [
                    _fraction(item) for item in assigned_coordinate
                ],
                "source_coordinate": [_fraction(item) for item in source_point],
                "exact_cross_with_bound_line": _fraction(cross),
                "displacement": [_fraction(item) for item in displacement],
                "displacement_squared_chart_euclidean": _fraction(
                    euclidean_squared
                ),
                "displacement_squared_rational_affine_metric": _fraction(
                    gram_squared
                ),
                "half_line_period_squared_chart_euclidean": _fraction(
                    half_euclidean_squared
                ),
                "half_line_period_squared_rational_affine_metric": _fraction(
                    half_gram_squared
                ),
                "full_vector_boundary_chart_euclidean_pass": (
                    euclidean_squared <= half_euclidean_squared
                ),
                "full_vector_boundary_rational_affine_metric_pass": (
                    gram_squared <= half_gram_squared
                ),
                "projection_k_chart_euclidean": _fraction(
                    projection_euclidean
                ),
                "projection_k_rational_affine_metric": _fraction(
                    projection_gram
                ),
                "longitudinal_residual_k_chart_euclidean": _fraction(
                    residual_euclidean
                ),
                "longitudinal_residual_k_rational_affine_metric": _fraction(
                    residual_gram
                ),
                "longitudinal_boundary_chart_euclidean_pass": (
                    residual_euclidean <= Fraction(1, 2)
                ),
                "longitudinal_boundary_rational_affine_metric_pass": (
                    residual_gram <= Fraction(1, 2)
                ),
                "boundary_scope": (
                    "BASE_BOUND_ENDPOINT_AUTHORITY_FIXED_NOT_REFINED_BOUNDARY_REQUIRED"
                    if not internal
                    else "A_DOUBLE_PRIME_INTERNAL_ASSIGNMENT"
                ),
            }
        )
    return result


def _chain_info(snapshot, chain, source_by_id, base_node_by_id) -> dict[str, Any]:
    vertex_ids = chain.ordered_source_vertex_ids
    source = tuple(source_by_id[item] for item in vertex_ids)
    source_direction = _subtract(source[-1], source[0])
    if source_direction == (0, 0):
        raise ValueError("declared-straight chain endpoints coincide")
    if any(
        gate_a._cross(source_direction, _subtract(point, source[0])) != 0
        for point in source
    ):
        raise ValueError("declared-straight chain is not source-collinear")
    start_node = base_node_by_id[vertex_ids[0]]
    end_node = base_node_by_id[vertex_ids[-1]]
    delta = end_node[0] - start_node[0], end_node[1] - start_node[1]
    endpoint_span = gcd(abs(delta[0]), abs(delta[1]))
    if endpoint_span == 0:
        raise ValueError("bound endpoints coincide")
    direction = delta[0] // endpoint_span, delta[1] // endpoint_span
    return {
        "chain": chain,
        "physical_chain_id": chain.physical_chain_id.value,
        "source_vertex_ids": vertex_ids,
        "source": source,
        "base_start_node": start_node,
        "base_end_node": end_node,
        "primitive_direction": direction,
        "base_endpoint_span": endpoint_span,
        "vertex_count": len(vertex_ids),
        "internal_vertex_count": len(vertex_ids) - 2,
    }


def _attempt_chain(
    info: dict[str, Any],
    base_scale: int,
    r: int,
    gram,
    observations: dict[str, int],
) -> dict[str, Any]:
    factor = 1 << r
    refined_scale = base_scale * factor
    base_start_coordinate = tuple(
        Fraction(item, base_scale) for item in info["base_start_node"]
    )
    base_end_coordinate = tuple(
        Fraction(item, base_scale) for item in info["base_end_node"]
    )
    refined_start_coordinate = tuple(
        Fraction(item * factor, refined_scale)
        for item in info["base_start_node"]
    )
    refined_end_coordinate = tuple(
        Fraction(item * factor, refined_scale)
        for item in info["base_end_node"]
    )
    endpoints_fixed_physically = (
        base_start_coordinate == refined_start_coordinate
        and base_end_coordinate == refined_end_coordinate
    )
    euclidean = _assign_norm(
        info["source"],
        info["base_start_node"],
        info["primitive_direction"],
        info["base_endpoint_span"],
        base_scale,
        r,
        None,
        "CHART_EUCLIDEAN",
    )
    metric = _assign_norm(
        info["source"],
        info["base_start_node"],
        info["primitive_direction"],
        info["base_endpoint_span"],
        base_scale,
        r,
        gram,
        "EXACT_RATIONAL_AFFINE_PLANAR_METRIC_GRAM",
    )
    if not euclidean["capacity_feasible"]:
        observations["capacity_infeasible_chain_attempts"] += 1
    observations["half_ties_chart_euclidean_resolved_up"] += sum(
        step["exact_half_tie"] for step in euclidean["steps"]
    )
    observations["half_ties_rational_affine_metric_resolved_up"] += sum(
        step["exact_half_tie"] for step in metric["steps"]
    )
    euclidean_boundary = _boundary_for_assignment(
        info["source"],
        info["source_vertex_ids"],
        info["base_start_node"],
        info["primitive_direction"],
        base_scale,
        r,
        euclidean,
        gram,
    )
    metric_boundary = _boundary_for_assignment(
        info["source"],
        info["source_vertex_ids"],
        info["base_start_node"],
        info["primitive_direction"],
        base_scale,
        r,
        metric,
        gram,
    )
    norm_agreement = (
        euclidean["k_sequence"] is not None
        and metric["k_sequence"] is not None
        and euclidean["k_sequence"] == metric["k_sequence"]
    )
    feasible = (
        euclidean["capacity_feasible"]
        and metric["capacity_feasible"]
        and euclidean["canonical"]
        and metric["canonical"]
        and euclidean["strict_order"]
        and metric["strict_order"]
        and all(
            item["exact_cross_with_bound_line"] == "0"
            for item in euclidean_boundary + metric_boundary
        )
    )
    boundary_variants = (
        (euclidean_boundary,)
        if norm_agreement
        else (euclidean_boundary, metric_boundary)
    )
    reviewer_boundary_records = [
        item for item in euclidean_boundary if item["is_endpoint"]
    ] + [
        item
        for boundary in boundary_variants
        for item in boundary
        if not item["is_endpoint"]
    ]
    matrix = {
        "capacity_failures": int(
            not euclidean["capacity_feasible"]
            or not metric["capacity_feasible"]
        ),
        "canonicality_failures": int(
            euclidean["capacity_feasible"] and not euclidean["canonical"]
        )
        + int(metric["capacity_feasible"] and not metric["canonical"]),
        "norm_k_disagreements": int(
            euclidean["capacity_feasible"]
            and metric["capacity_feasible"]
            and not norm_agreement
        ),
        "strict_order_failures": int(
            euclidean["capacity_feasible"] and not euclidean["strict_order"]
        )
        + int(metric["capacity_feasible"] and not metric["strict_order"]),
        "sticky_k": sum(
            (
                len(assignment["k_sequence"])
                - len(set(assignment["k_sequence"]))
            )
            if assignment["k_sequence"] is not None
            else 0
            for assignment in (euclidean, metric)
        ),
        "reversals": sum(
            sum(
                current < previous
                for previous, current in zip(sequence, sequence[1:])
            )
            for sequence in (
                euclidean["k_sequence"],
                metric["k_sequence"],
            )
            if sequence is not None
        ),
        "nonzero_cross": sum(
            item["exact_cross_with_bound_line"] != "0"
            for item in euclidean_boundary + metric_boundary
        ),
        "endpoint_mutations": sum(
            int(
                assignment["k_sequence"] is not None
                and (
                    assignment["k_sequence"][0] != 0
                    or assignment["k_sequence"][-1]
                    != info["base_endpoint_span"] * (1 << r)
                )
            )
            for assignment in (euclidean, metric)
        )
        + int(not endpoints_fixed_physically),
        "full_boundary_chart_euclidean_failures": sum(
            not item["full_vector_boundary_chart_euclidean_pass"]
            for item in reviewer_boundary_records
        ),
        "full_boundary_rational_affine_metric_failures": sum(
            not item["full_vector_boundary_rational_affine_metric_pass"]
            for item in reviewer_boundary_records
        ),
        "longitudinal_chart_euclidean_failures": sum(
            not item["longitudinal_boundary_chart_euclidean_pass"]
            for item in reviewer_boundary_records
        ),
        "longitudinal_rational_affine_metric_failures": sum(
            not item["longitudinal_boundary_rational_affine_metric_pass"]
            for item in reviewer_boundary_records
        ),
    }
    reviewer_admissible = all(value == 0 for value in matrix.values())
    return {
        "physical_chain_id": info["physical_chain_id"],
        "vertex_count": info["vertex_count"],
        "internal_vertex_count": info["internal_vertex_count"],
        "base_endpoint_span_K": info["base_endpoint_span"],
        "refined_endpoint_span_K_prime": info["base_endpoint_span"] * (1 << r),
        "capacity_required": info["vertex_count"] - 1,
        "primitive_direction": list(info["primitive_direction"]),
        "base_bound_start_node": list(info["base_start_node"]),
        "base_bound_end_node": list(info["base_end_node"]),
        "refined_bound_start_node": [
            item * factor for item in info["base_start_node"]
        ],
        "refined_bound_end_node": [
            item * factor for item in info["base_end_node"]
        ],
        "base_bound_start_coordinate": [
            _fraction(item) for item in base_start_coordinate
        ],
        "base_bound_end_coordinate": [
            _fraction(item) for item in base_end_coordinate
        ],
        "refined_bound_start_coordinate": [
            _fraction(item) for item in refined_start_coordinate
        ],
        "refined_bound_end_coordinate": [
            _fraction(item) for item in refined_end_coordinate
        ],
        "bound_endpoints_fixed_physically": endpoints_fixed_physically,
        "euclidean_assignment": euclidean,
        "metric_assignment": metric,
        "norm_k_agreement": norm_agreement,
        "assignment_feasible": feasible,
        "reviewer_failure_matrix": matrix,
        "reviewer_admissible": reviewer_admissible,
        "euclidean_boundary": euclidean_boundary,
        "metric_boundary": metric_boundary,
    }


def _base_node_hash(base_node_by_id: dict) -> str:
    payload = [
        [item.value, list(base_node_by_id[item])]
        for item in sorted(base_node_by_id, key=lambda value: value.value)
    ]
    return _sha256(gate_a._canonical_json(payload))


def _analyze_fixture(
    kernel,
    fixture_dir: Path,
    counters: dict[str, int],
    observations: dict[str, int],
    failures: list[dict[str, Any]],
) -> dict[str, Any]:
    manifest = json.loads(
        (fixture_dir / "manifest.json").read_text(encoding="utf-8")
    )
    gate_a._validate_fixture_files(fixture_dir, manifest)
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (fixture_dir / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (fixture_dir / "decal_request.json").read_bytes()
    )
    issues = (
        tuple(kernel.validate_analysis_snapshot(snapshot))
        + tuple(kernel.validate_decal_request(request))
        + tuple(kernel.validate_snapshot_request_references(snapshot, request))
    )
    if issues:
        raise ValueError("fixture contracts are invalid")
    domains = tuple(snapshot.patch_domains)
    if len(domains) != 1:
        raise ValueError("fixture must contain exactly one PatchDomain")
    compiled = kernel.compile_reference_envelopes(
        snapshot, request, domains[0].patch_domain_id
    )
    if getattr(compiled.outcome, "value", compiled.outcome) != "EXACT":
        raise ValueError("fixture does not compile EXACT")
    compilation = compiled.compilation
    if compilation is None or compilation.evaluation_geometry_binding is None:
        raise ValueError("fixture has no base evaluation geometry binding")
    binding = compilation.evaluation_geometry_binding
    frame = next(
        item
        for item in snapshot.surface_metric_descriptors
        if item.patch_domain_id == domains[0].patch_domain_id
    )
    if type(frame).__name__ != "RationalAffinePlanarMetricV2":
        raise ValueError("fixture metric is not RationalAffinePlanarMetricV2")
    gram = gate_a._gram_matrix(frame)
    base_scale = binding.lattice_scale
    source_by_id = {
        item.source_vertex_id: gate_a._point(item.domain_coordinate)
        for item in frame.exact_source_vertex_coordinates
    }
    base_node_by_id = {}
    for item in binding.source_vertex_coordinates:
        coordinate = gate_a._point(item.domain_coordinate)
        scaled = coordinate[0] * base_scale, coordinate[1] * base_scale
        if any(value.denominator != 1 for value in scaled):
            raise ValueError("base binding coordinate is not a lattice node")
        base_node_by_id[item.source_vertex_id] = (
            int(scaled[0]),
            int(scaled[1]),
        )
    reproduced = {
        vertex_id: (
            gate_a._snap_half_up(point[0], base_scale),
            gate_a._snap_half_up(point[1], base_scale),
        )
        for vertex_id, point in source_by_id.items()
    }
    if reproduced != base_node_by_id:
        raise ValueError("base binding law was not reproduced")
    base_hash_before = _base_node_hash(base_node_by_id)

    declared_chains = []
    for chain in sorted(
        snapshot.physical_chains,
        key=lambda item: item.physical_chain_id.value,
    ):
        if chain.is_closed or len(chain.ordered_source_vertex_ids) < 3:
            continue
        if gate_a._declared_internal_corner_ids(snapshot, chain):
            continue
        declared_chains.append(
            _chain_info(snapshot, chain, source_by_id, base_node_by_id)
        )
    if not declared_chains:
        raise ValueError("fixture has no declared-straight multi-vertex chains")

    attempts = []
    for r in range(R_MAX + 1):
        chain_attempts = [
            _attempt_chain(info, base_scale, r, gram, observations)
            for info in declared_chains
        ]
        matrix_names = tuple(
            chain_attempts[0]["reviewer_failure_matrix"]
        )
        domain_matrix = {
            name: sum(
                item["reviewer_failure_matrix"][name]
                for item in chain_attempts
            )
            for name in matrix_names
        }
        attempts.append(
            {
                "r": r,
                "base_scale_S": base_scale,
                "refined_scale_S_prime": base_scale * (1 << r),
                "chain_attempts": chain_attempts,
                "domain_capacity_and_assignment_feasible": all(
                    item["assignment_feasible"] for item in chain_attempts
                ),
                "all_norm_k_assignments_agree": all(
                    item["norm_k_agreement"] for item in chain_attempts
                    if item["euclidean_assignment"]["capacity_feasible"]
                ),
                "reviewer_failure_matrix": domain_matrix,
                "reviewer_admissible": all(
                    value == 0 for value in domain_matrix.values()
                ),
            }
        )
    capacity_feasible_attempts = [
        item
        for item in attempts
        if item["domain_capacity_and_assignment_feasible"]
    ]
    capacity_min_r = (
        capacity_feasible_attempts[0]["r"]
        if capacity_feasible_attempts
        else None
    )
    if capacity_min_r is None:
        capacity_minimality_witness = {
            "outcome": "REFINEMENT_BUDGET_EXHAUSTED",
            "r_max": R_MAX,
        }
    elif capacity_min_r == 0:
        capacity_minimality_witness = {
            "outcome": "BASE_SCALE_ALREADY_FEASIBLE",
            "previous_r": None,
        }
    else:
        previous = attempts[capacity_min_r - 1]
        reasons = [
            {
                "physical_chain_id": item["physical_chain_id"],
                "outcome": item["euclidean_assignment"]["outcome"],
                "K_prime": item["refined_endpoint_span_K_prime"],
                "capacity_required": item["capacity_required"],
                "exact_deficit": (
                    item["capacity_required"]
                    - item["refined_endpoint_span_K_prime"]
                ),
            }
            for item in previous["chain_attempts"]
            if not item["assignment_feasible"]
        ]
        capacity_minimality_witness = {
            "outcome": "PREVIOUS_REFINEMENT_INFEASIBLE",
            "previous_r": capacity_min_r - 1,
            "reasons": reasons,
        }

    reviewer_admissible_attempts = [
        item for item in attempts if item["reviewer_admissible"]
    ]
    r_min = (
        reviewer_admissible_attempts[0]["r"]
        if reviewer_admissible_attempts
        else None
    )
    if r_min is None:
        reviewer_minimality_witness = {
            "outcome": "NO_R_LE_8",
            "r_max": R_MAX,
            "per_r_failure_matrix": [
                {
                    "r": item["r"],
                    "failure_matrix": item["reviewer_failure_matrix"],
                }
                for item in attempts
            ],
        }
        _failure(
            counters,
            failures,
            "no_admissible_refinement_domains",
            "NO_ADMISSIBLE_REFINEMENT_R_LE_8",
            "no r in 0..8 passes every reviewer criterion",
            fixture_id=fixture_dir.name,
        )
    elif r_min == 0:
        reviewer_minimality_witness = {
            "outcome": "BASE_SCALE_FULLY_ADMISSIBLE",
            "previous_r": None,
        }
    else:
        previous = attempts[r_min - 1]
        reviewer_minimality_witness = {
            "outcome": "PREVIOUS_R_NOT_FULLY_ADMISSIBLE",
            "previous_r": r_min - 1,
            "failure_matrix": previous["reviewer_failure_matrix"],
        }

    # Counters aggregate every exact r=0..8 attempt.  Expected failed search
    # candidates remain diagnostics; terminal refusal is recorded separately.
    for attempt in attempts:
        for chain_attempt in attempt["chain_attempts"]:
            matrix = chain_attempt["reviewer_failure_matrix"]
            counters["norm_k_disagreements"] += matrix[
                "norm_k_disagreements"
            ]
            counters["canonicality_failures"] += matrix[
                "canonicality_failures"
            ]
            counters["strict_order_failures"] += matrix[
                "strict_order_failures"
            ]
            counters["sticky_k"] += matrix["sticky_k"]
            counters["reversals"] += matrix["reversals"]
            counters["nonzero_cross"] += matrix["nonzero_cross"]
            counters["endpoint_mutations"] += matrix["endpoint_mutations"]
            counters[
                "full_boundary_chart_euclidean_failures"
            ] += matrix["full_boundary_chart_euclidean_failures"]
            counters[
                "full_boundary_rational_affine_metric_failures"
            ] += matrix[
                "full_boundary_rational_affine_metric_failures"
            ]
            counters[
                "longitudinal_chart_euclidean_failures"
            ] += matrix["longitudinal_chart_euclidean_failures"]
            counters[
                "longitudinal_rational_affine_metric_failures"
            ] += matrix[
                "longitudinal_rational_affine_metric_failures"
            ]

    if r_min is not None:
        observations["selected_internal_vertices"] += sum(
            item["internal_vertex_count"] for item in declared_chains
        )

    base_hash_after = _base_node_hash(base_node_by_id)
    if base_hash_before != base_hash_after:
        _failure(
            counters,
            failures,
            "base_node_mutations",
            "BASE_BINDING_NODE_MUTATED",
            "proof changed a base binding node",
            fixture_id=fixture_dir.name,
        )
    return {
        "fixture_id": fixture_dir.name,
        "patch_domain_id": domains[0].patch_domain_id.value,
        "fixture_hash": manifest["fixture_hash"],
        "base_scale_S": base_scale,
        "exact_gram_matrix": [
            [_fraction(item) for item in row] for row in gram
        ],
        "base_binding_node_count": len(base_node_by_id),
        "base_binding_node_sha256_before": base_hash_before,
        "base_binding_node_sha256_after": base_hash_after,
        "base_binding_nodes_unchanged": base_hash_before == base_hash_after,
        "non_chain_and_base_nodes_mutated_by_proof": False,
        "declared_straight_chain_count": len(declared_chains),
        "internal_vertex_count": sum(
            item["internal_vertex_count"] for item in declared_chains
        ),
        "chain_base_facts": [
            {
                "physical_chain_id": item["physical_chain_id"],
                "vertex_count_n": item["vertex_count"],
                "internal_vertex_count": item["internal_vertex_count"],
                "base_endpoint_span_K": item["base_endpoint_span"],
                "primitive_direction": list(item["primitive_direction"]),
                "base_bound_start_node": list(item["base_start_node"]),
                "base_bound_end_node": list(item["base_end_node"]),
                "source_vertex_ids": [
                    value.value for value in item["source_vertex_ids"]
                ],
            }
            for item in declared_chains
        ],
        "capacity_min_r": capacity_min_r,
        "capacity_minimality_witness": capacity_minimality_witness,
        "r_min": r_min,
        "reviewer_minimality_witness": reviewer_minimality_witness,
        "attempts": attempts,
    }


def _synthetic_budget_refusal_self_check() -> dict[str, Any]:
    base_endpoint_span = 1
    vertex_count = (1 << R_MAX) + 2
    attempts = [
        {
            "r": r,
            "K_prime": base_endpoint_span * (1 << r),
            "capacity_required": vertex_count - 1,
            "capacity_feasible": (
                base_endpoint_span * (1 << r) >= vertex_count - 1
            ),
        }
        for r in range(R_MAX + 1)
    ]
    refused = not any(item["capacity_feasible"] for item in attempts)
    return {
        "schema": "cftuv.envelope.sem_clb_02_gate_a_double_prime_budget_self_check.v1",
        "synthetic_base_endpoint_span_K": base_endpoint_span,
        "synthetic_vertex_count_n": vertex_count,
        "r_max": R_MAX,
        "attempts": attempts,
        "outcome": (
            "REFINEMENT_BUDGET_EXHAUSTED"
            if refused
            else "SYNTHETIC_CAPACITY_UNEXPECTEDLY_REACHABLE"
        ),
        "named_refusal_reached": refused,
    }


def _aggregate(
    fixtures: list[dict[str, Any]],
    counters: dict[str, int],
    observations: dict[str, int],
) -> dict[str, Any]:
    capacity_min_records = []
    for fixture in fixtures:
        if fixture["capacity_min_r"] is None:
            continue
        attempt = next(
            item
            for item in fixture["attempts"]
            if item["r"] == fixture["capacity_min_r"]
        )
        for chain in attempt["chain_attempts"]:
            for vertex in chain["euclidean_boundary"]:
                if not vertex["is_endpoint"]:
                    continue
                capacity_min_records.append(
                    {
                        "fixture_id": fixture["fixture_id"],
                        "physical_chain_id": chain["physical_chain_id"],
                        "source_vertex_id": vertex["source_vertex_id"],
                        "r": fixture["capacity_min_r"],
                        "assignment_norm": "COMMON_FIXED_ENDPOINT",
                        "vertex": vertex,
                    }
                )
            for norm, boundary_name in (
                ("CHART_EUCLIDEAN", "euclidean_boundary"),
                (
                    "EXACT_RATIONAL_AFFINE_PLANAR_METRIC_GRAM",
                    "metric_boundary",
                ),
            ):
                for vertex in chain[boundary_name]:
                    if vertex["is_endpoint"]:
                        continue
                    capacity_min_records.append(
                        {
                            "fixture_id": fixture["fixture_id"],
                            "physical_chain_id": chain["physical_chain_id"],
                            "source_vertex_id": vertex["source_vertex_id"],
                            "r": fixture["capacity_min_r"],
                            "assignment_norm": norm,
                            "vertex": vertex,
                        }
                    )

    def maximum_ratio(value_name: str, boundary_name: str):
        candidates = []
        for record in capacity_min_records:
            vertex = record["vertex"]
            ratio = Fraction(vertex[value_name]) / Fraction(
                vertex[boundary_name]
            )
            candidates.append((ratio, record))
        if not candidates:
            return None
        ratio, record = max(candidates, key=lambda item: item[0])
        vertex = record["vertex"]
        return {
            "ratio": _fraction(ratio),
            "value": vertex[value_name],
            "boundary": vertex[boundary_name],
            "fixture_id": record["fixture_id"],
            "physical_chain_id": record["physical_chain_id"],
            "source_vertex_id": record["source_vertex_id"],
            "r": record["r"],
            "assignment_norm": record["assignment_norm"],
        }

    def maximum_longitudinal(field: str):
        candidates = []
        for record in capacity_min_records:
            residual = Fraction(record["vertex"][field])
            candidates.append((residual * 2, record))
        if not candidates:
            return None
        ratio, record = max(candidates, key=lambda item: item[0])
        return {
            "ratio_to_half_period": _fraction(ratio),
            "residual_k": _fraction(ratio / 2),
            "fixture_id": record["fixture_id"],
            "physical_chain_id": record["physical_chain_id"],
            "source_vertex_id": record["source_vertex_id"],
            "r": record["r"],
            "assignment_norm": record["assignment_norm"],
        }

    return {
        "fixture_count": len(fixtures),
        "declared_straight_chain_count": sum(
            item["declared_straight_chain_count"] for item in fixtures
        ),
        "internal_vertex_count": sum(
            item["internal_vertex_count"] for item in fixtures
        ),
        "domain_r_min": {
            item["fixture_id"]: item["r_min"] for item in fixtures
        },
        "domain_capacity_min_r": {
            item["fixture_id"]: item["capacity_min_r"] for item in fixtures
        },
        "ratio_scope": "CAPACITY_MIN_R_ALL_CHAIN_VERTICES_DIAGNOSTIC",
        "failure_counters": counters,
        "observation_counters": observations,
        "max_full_vector_ratio_chart_euclidean_at_capacity_min": maximum_ratio(
            "displacement_squared_chart_euclidean",
            "half_line_period_squared_chart_euclidean",
        ),
        "max_full_vector_ratio_rational_affine_metric_at_capacity_min": maximum_ratio(
            "displacement_squared_rational_affine_metric",
            "half_line_period_squared_rational_affine_metric",
        ),
        "max_longitudinal_ratio_chart_euclidean_at_capacity_min": maximum_longitudinal(
            "longitudinal_residual_k_chart_euclidean",
        ),
        "max_longitudinal_ratio_rational_affine_metric_at_capacity_min": maximum_longitudinal(
            "longitudinal_residual_k_rational_affine_metric",
        ),
    }


def main() -> None:
    arguments = _arguments()
    counters = {name: 0 for name in FAILURE_COUNTER_NAMES}
    observations = {name: 0 for name in OBSERVATION_COUNTER_NAMES}
    failures: list[dict[str, Any]] = []
    fixtures: list[dict[str, Any]] = []
    product_tree_oid: str | None = None
    budget_self_check = _synthetic_budget_refusal_self_check()
    if not budget_self_check["named_refusal_reached"]:
        _failure(
            counters,
            failures,
            "refinement_budget_refusal_self_check_failures",
            "REFINEMENT_BUDGET_REFUSAL_NOT_REACHABLE",
            "synthetic r_max=8 capacity case did not refuse",
        )
    try:
        product_tree_oid = gate_a._git_value(
            arguments.source_root, "HEAD:kernel/src"
        )
        if product_tree_oid != EXPECTED_PRODUCT_TREE_OID:
            _failure(
                counters,
                failures,
                "unknown_product_tree",
                "UNKNOWN_KERNEL_SRC_PRODUCT_TREE",
                "Gate A-double-prime has no oracle for this product tree",
                kernel_src_product_tree_oid=product_tree_oid,
            )
        else:
            kernel = gate_a._activate_kernel(arguments.source_root)
            present = {
                item.name
                for item in arguments.fixture_root.iterdir()
                if item.is_dir() and item.name in FIXTURE_IDS
            }
            if present != set(FIXTURE_IDS):
                _failure(
                    counters,
                    failures,
                    "unknown_fixture_form",
                    "UNKNOWN_FIXTURE_SET",
                    "one or more required lost-domain fixtures are absent",
                )
            else:
                for fixture_id in FIXTURE_IDS:
                    try:
                        fixtures.append(
                            _analyze_fixture(
                                kernel,
                                arguments.fixture_root / fixture_id,
                                counters,
                                observations,
                                failures,
                            )
                        )
                    except Exception as error:
                        _failure(
                            counters,
                            failures,
                            "unknown_fixture_form",
                            "UNKNOWN_FIXTURE_FORM",
                            f"{type(error).__name__}: {error}",
                            fixture_id=fixture_id,
                        )
    except Exception as error:
        _failure(
            counters,
            failures,
            "unknown_fixture_form",
            "GATE_A_DOUBLE_PRIME_EXECUTION_FORM_UNKNOWN",
            f"{type(error).__name__}: {error}",
        )

    aggregate = _aggregate(fixtures, counters, observations)
    outcome = (
        "GATE_A_DOUBLE_PRIME_EXACT_REACHABLE"
        if not failures
        else "GATE_A_DOUBLE_PRIME_REFUSED"
    )
    payload = {
        "schema": SCHEMA,
        "outcome": outcome,
        "proof_base_revision": PROOF_BASE_REVISION,
        "kernel_src_product_tree_oid": product_tree_oid,
        "accepted_kernel_src_product_tree_oid": EXPECTED_PRODUCT_TREE_OID,
        "r_domain": {"minimum": 0, "maximum": R_MAX},
        "refinement_law": (
            "BASE_ENDPOINTS_FIXED_INTERNAL_BOUND_LINE_NODES_SCALE_BY_POWER_OF_TWO_V1"
        ),
        "assignment_law": (
            "DECLARED_SOURCE_ORDER_EXACT_HALF_UP_NEAREST_CLAMPED_TO_STRICT_CAPACITY_INTERVAL_V1"
        ),
        "canonicality_law": (
            "CLAMP_OF_EXACT_HALF_UP_PROJECTION_ON_POSITIVE_QUADRATIC_INTEGER_INTERVAL_V1"
        ),
        "boundary_law": {
            "literal_full_vector": (
                "SOURCE_TO_ASSIGNED_NODE_SQUARED_NORM_LE_HALF_LINE_PERIOD_SQUARED"
            ),
            "longitudinal": "ABS_ASSIGNED_K_MINUS_PROJECTION_K_LE_ONE_HALF",
            "endpoint_scope": (
                "BASE_BOUND_ENDPOINT_AUTHORITY_FIXED_NOT_REFINED_BOUNDARY_REQUIRED"
            ),
            "internal_scope": "A_DOUBLE_PRIME_REFINEMENT",
        },
        "synthetic_budget_refusal_self_check": budget_self_check,
        "aggregate": aggregate,
        "failures": failures,
        "fixtures": fixtures,
    }
    report_bytes = _pretty_json(payload)
    arguments.output_report.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_report.write_bytes(report_bytes)
    print(
        json.dumps(
            {
                "outcome": outcome,
                "fixture_count": aggregate["fixture_count"],
                "declared_straight_chain_count": aggregate[
                    "declared_straight_chain_count"
                ],
                "internal_vertex_count": aggregate["internal_vertex_count"],
                "domain_r_min": aggregate["domain_r_min"],
                "failure_counters": counters,
                "report_sha256": _sha256(report_bytes),
            },
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )
    raise SystemExit(0 if not failures else 2)


if __name__ == "__main__":
    main()
