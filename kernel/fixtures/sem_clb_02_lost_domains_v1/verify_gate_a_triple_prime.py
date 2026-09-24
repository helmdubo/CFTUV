"""Exact canonical-Gram refinement proof for SEM-CLB-02-R Gate A-triple-prime.

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


_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))
import verify_gate_a_double_prime as gate_a2


gate_a = gate_a2.gate_a
SCHEMA = "cftuv.envelope.sem_clb_02_gate_a_triple_prime.v1"
PROOF_BASE_REVISION = "a331fe49ce99925219caa5829ca211c857c2c8d0"
EXPECTED_PRODUCT_TREE_OID = gate_a2.EXPECTED_PRODUCT_TREE_OID
EXPECTED_A2_RECEIPT_SHA256 = (
    "891d008ffc27a03a1b819ed93facbb86819830b95a5a32f40c28223681e68a4e"
)
FIXTURE_IDS = gate_a2.FIXTURE_IDS
R_MAX = 8
EXPECTED_COVERAGE = {
    "fixture_count": 4,
    "declared_straight_chain_count": 8,
    "internal_vertex_count": 13,
}
R_MIN_HYPOTHESIS = {
    "building_all_seams_patch_001_lost_resolved_v1": 2,
    "building_all_seams_patch_006_lost_resolved_v1": 3,
    "building_all_seams_patch_011_lost_resolved_v1": 2,
    "building_all_seams_patch_105_lost_resolved_v1": 1,
}
MATRIX_NAMES = (
    "capacity_failures",
    "assignment_recompute_failures",
    "canonicality_failures",
    "strict_order_failures",
    "unclamped_full_gram_boundary_failures",
    "unclamped_longitudinal_failures",
    "clamped_excess_unnamed_failures",
    "base_endpoint_authority_failures",
    "base_non_chain_authority_failures",
    "prior_v1_binding_failures",
    "source_order_failures",
    "nonzero_cross",
)
TERMINAL_COUNTER_NAMES = (
    "coverage_failures",
    "no_admissible_refinement_domains",
    "refinement_budget_refusal_self_check_failures",
    "unknown_fixture_form",
    "unknown_product_tree",
    "unknown_prior_a2_evidence",
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
        default=_SCRIPT_DIR / "cases",
    )
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


def _fraction(value: Fraction | int) -> str:
    return gate_a2._fraction(value)


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


def _terminal_failure(
    counters: dict[str, int],
    failures: list[dict[str, Any]],
    counter: str,
    code: str,
    detail: str,
    **context: Any,
) -> None:
    counters[counter] += 1
    failures.append({"code": code, "detail": detail, **context})


def _gram_dot(left, right, gram) -> Fraction:
    return gate_a2._gram_dot(left, right, gram)


def _projection(source_scaled, anchor, direction, gram) -> Fraction:
    return gate_a2._projection(source_scaled, anchor, direction, gram)


def _half_up(value: Fraction) -> tuple[int, bool]:
    return gate_a2._half_up(value)


def _review_half_up(value: Fraction) -> tuple[int, bool]:
    lower = value.numerator // value.denominator
    twice_remainder = 2 * (
        value.numerator - lower * value.denominator
    )
    tie = twice_remainder == value.denominator
    return lower + int(twice_remainder >= value.denominator), tie


def _canonical_gram_assignment(
    info: dict[str, Any],
    base_scale: int,
    r: int,
    gram,
) -> dict[str, Any]:
    factor = 1 << r
    scale = base_scale * factor
    direction = info["primitive_direction"]
    anchor = (
        info["base_start_node"][0] * factor,
        info["base_start_node"][1] * factor,
    )
    endpoint_k = info["base_endpoint_span"] * factor
    required = info["vertex_count"] - 1
    if endpoint_k < required:
        return {
            "outcome": "CHAIN_REFINEMENT_CAPACITY_INSUFFICIENT",
            "capacity_feasible": False,
            "capacity_required": required,
            "K_prime": endpoint_k,
            "steps": [],
            "k_sequence": None,
        }

    period_squared = _gram_dot(direction, direction, gram) / (scale * scale)
    half_period_squared = period_squared / 4
    previous_k = 0
    k_sequence = [0]
    steps = []
    for ordinal, (vertex_id, source_point) in enumerate(
        zip(info["source_vertex_ids"][1:-1], info["source"][1:-1]),
        start=1,
    ):
        remaining = info["vertex_count"] - 1 - ordinal
        lower = previous_k + 1
        upper = endpoint_k - remaining
        source_scaled = source_point[0] * scale, source_point[1] * scale
        projection = _projection(source_scaled, anchor, direction, gram)
        unconstrained, exact_half_tie = _half_up(projection)
        selected = min(max(unconstrained, lower), upper)
        clamped = selected != unconstrained
        assigned_node = (
            anchor[0] + selected * direction[0],
            anchor[1] + selected * direction[1],
        )
        assigned_coordinate = (
            Fraction(assigned_node[0], scale),
            Fraction(assigned_node[1], scale),
        )
        displacement = (
            assigned_coordinate[0] - source_point[0],
            assigned_coordinate[1] - source_point[1],
        )
        gram_displacement_squared = _gram_dot(displacement, displacement, gram)
        longitudinal_residual = abs(Fraction(selected) - projection)
        full_boundary_pass = gram_displacement_squared <= half_period_squared
        longitudinal_boundary_pass = longitudinal_residual <= Fraction(1, 2)
        if not clamped:
            disposition = "UNCLAMPED_WITHIN_HALF_STEP"
        elif full_boundary_pass and longitudinal_boundary_pass:
            disposition = "CLAMPED_WITHIN_HALF_STEP"
        else:
            disposition = "CLAMPED_CONSTRAINT_EXCESS_ALLOWED"
        steps.append(
            {
                "ordinal": ordinal,
                "source_vertex_id": vertex_id.value,
                "source_coordinate": [_fraction(item) for item in source_point],
                "lower": lower,
                "upper": upper,
                "remaining_vertices_to_end_endpoint": remaining,
                "projection_k_gram": _fraction(projection),
                "unconstrained_canonical_k": unconstrained,
                "exact_half_tie": exact_half_tie,
                "selected_k": selected,
                "CLAMPED": clamped,
                "assigned_refined_node": list(assigned_node),
                "assigned_coordinate": [
                    _fraction(item) for item in assigned_coordinate
                ],
                "exact_displacement": [
                    _fraction(item) for item in displacement
                ],
                "exact_gram_displacement_squared": _fraction(
                    gram_displacement_squared
                ),
                "half_S_prime_step_gram_squared_bound": _fraction(
                    half_period_squared
                ),
                "full_gram_boundary_pass": full_boundary_pass,
                "exact_longitudinal_displacement_k": _fraction(
                    longitudinal_residual
                ),
                "longitudinal_half_step_bound_k": "1/2",
                "longitudinal_boundary_pass": longitudinal_boundary_pass,
                "boundary_disposition": disposition,
                "exact_cross_with_bound_line": _fraction(
                    gate_a._cross(
                        (
                            assigned_node[0] - anchor[0],
                            assigned_node[1] - anchor[1],
                        ),
                        direction,
                    )
                ),
            }
        )
        k_sequence.append(selected)
        previous_k = selected
    k_sequence.append(endpoint_k)
    return {
        "outcome": "EXACT_CANONICAL_GRAM_ORDERED_ASSIGNMENT",
        "capacity_feasible": True,
        "capacity_required": required,
        "K_prime": endpoint_k,
        "gram_line_period_squared": _fraction(period_squared),
        "half_S_prime_step_gram_squared_bound": _fraction(
            half_period_squared
        ),
        "steps": steps,
        "k_sequence": k_sequence,
    }


def _review_gram_assignment(
    candidate: dict[str, Any],
    info: dict[str, Any],
    base_scale: int,
    r: int,
    gram,
) -> dict[str, int]:
    matrix = {name: 0 for name in MATRIX_NAMES}
    factor = 1 << r
    scale = base_scale * factor
    direction = info["primitive_direction"]
    anchor = (
        info["base_start_node"][0] * factor,
        info["base_start_node"][1] * factor,
    )
    endpoint_k = info["base_endpoint_span"] * factor
    required = info["vertex_count"] - 1
    if endpoint_k < required:
        matrix["capacity_failures"] = 1
        expected = {
            "outcome": "CHAIN_REFINEMENT_CAPACITY_INSUFFICIENT",
            "capacity_feasible": False,
            "capacity_required": required,
            "K_prime": endpoint_k,
            "steps": [],
            "k_sequence": None,
        }
        matrix["assignment_recompute_failures"] += int(candidate != expected)
        return matrix

    expected_ids = [
        item.value for item in info["source_vertex_ids"][1:-1]
    ]
    recorded_ids = [item["source_vertex_id"] for item in candidate["steps"]]
    matrix["source_order_failures"] += int(recorded_ids != expected_ids)
    previous_k = 0
    recomputed_sequence = [0]
    for ordinal, (vertex_id, source_point, step) in enumerate(
        zip(
            info["source_vertex_ids"][1:-1],
            info["source"][1:-1],
            candidate["steps"],
        ),
        start=1,
    ):
        remaining = info["vertex_count"] - 1 - ordinal
        lower = previous_k + 1
        upper = endpoint_k - remaining
        projection = _projection(
            (source_point[0] * scale, source_point[1] * scale),
            anchor,
            direction,
            gram,
        )
        unconstrained, tie = _review_half_up(projection)
        selected = min(max(unconstrained, lower), upper)
        clamped = selected != unconstrained
        assigned_node = (
            anchor[0] + selected * direction[0],
            anchor[1] + selected * direction[1],
        )
        assigned_coordinate = (
            Fraction(assigned_node[0], scale),
            Fraction(assigned_node[1], scale),
        )
        displacement = (
            assigned_coordinate[0] - source_point[0],
            assigned_coordinate[1] - source_point[1],
        )
        gram_squared = _gram_dot(displacement, displacement, gram)
        half_squared = (
            _gram_dot(direction, direction, gram) / (scale * scale) / 4
        )
        residual = abs(Fraction(selected) - projection)
        full_pass = gram_squared <= half_squared
        longitudinal_pass = residual <= Fraction(1, 2)
        if not clamped:
            disposition = "UNCLAMPED_WITHIN_HALF_STEP"
        elif full_pass and longitudinal_pass:
            disposition = "CLAMPED_WITHIN_HALF_STEP"
        else:
            disposition = "CLAMPED_CONSTRAINT_EXCESS_ALLOWED"
        expected_step = {
            "ordinal": ordinal,
            "source_vertex_id": vertex_id.value,
            "source_coordinate": [_fraction(item) for item in source_point],
            "lower": lower,
            "upper": upper,
            "remaining_vertices_to_end_endpoint": remaining,
            "projection_k_gram": _fraction(projection),
            "unconstrained_canonical_k": unconstrained,
            "exact_half_tie": tie,
            "selected_k": selected,
            "CLAMPED": clamped,
            "assigned_refined_node": list(assigned_node),
            "assigned_coordinate": [
                _fraction(item) for item in assigned_coordinate
            ],
            "exact_displacement": [_fraction(item) for item in displacement],
            "exact_gram_displacement_squared": _fraction(gram_squared),
            "half_S_prime_step_gram_squared_bound": _fraction(half_squared),
            "full_gram_boundary_pass": full_pass,
            "exact_longitudinal_displacement_k": _fraction(residual),
            "longitudinal_half_step_bound_k": "1/2",
            "longitudinal_boundary_pass": longitudinal_pass,
            "boundary_disposition": disposition,
            "exact_cross_with_bound_line": _fraction(
                gate_a._cross(
                    (
                        assigned_node[0] - anchor[0],
                        assigned_node[1] - anchor[1],
                    ),
                    direction,
                )
            ),
        }
        matrix["assignment_recompute_failures"] += int(step != expected_step)
        matrix["canonicality_failures"] += int(
            step.get("unconstrained_canonical_k") != unconstrained
            or step.get("selected_k") != selected
            or step.get("CLAMPED") != clamped
        )
        matrix["unclamped_full_gram_boundary_failures"] += int(
            not clamped and not full_pass
        )
        matrix["unclamped_longitudinal_failures"] += int(
            not clamped and not longitudinal_pass
        )
        matrix["clamped_excess_unnamed_failures"] += int(
            clamped
            and (not full_pass or not longitudinal_pass)
            and disposition != "CLAMPED_CONSTRAINT_EXCESS_ALLOWED"
        )
        matrix["nonzero_cross"] += int(
            expected_step["exact_cross_with_bound_line"] != "0"
        )
        previous_k = selected
        recomputed_sequence.append(selected)
    recomputed_sequence.append(endpoint_k)
    matrix["assignment_recompute_failures"] += int(
        candidate.get("k_sequence") != recomputed_sequence
    )
    matrix["strict_order_failures"] += sum(
        current <= previous
        for previous, current in zip(
            recomputed_sequence, recomputed_sequence[1:]
        )
    )
    return matrix


def _endpoint_authority(
    info: dict[str, Any],
    base_scale: int,
    r: int,
    gram,
    reproduced_nodes: dict,
) -> list[dict[str, Any]]:
    factor = 1 << r
    direction = info["primitive_direction"]
    records = []
    for ordinal, vertex_id, source_point, base_node, base_k in (
        (
            0,
            info["source_vertex_ids"][0],
            info["source"][0],
            info["base_start_node"],
            0,
        ),
        (
            info["vertex_count"] - 1,
            info["source_vertex_ids"][-1],
            info["source"][-1],
            info["base_end_node"],
            info["base_endpoint_span"],
        ),
    ):
        base_coordinate = (
            Fraction(base_node[0], base_scale),
            Fraction(base_node[1], base_scale),
        )
        displacement = (
            base_coordinate[0] - source_point[0],
            base_coordinate[1] - source_point[1],
        )
        base_projection = _projection(
            (
                source_point[0] * base_scale,
                source_point[1] * base_scale,
            ),
            info["base_start_node"],
            direction,
            gram,
        )
        base_gram_squared = _gram_dot(displacement, displacement, gram)
        base_component_bound = Fraction(1, 2 * base_scale)
        base_component_budget_pass = all(
            abs(item) <= base_component_bound for item in displacement
        )
        base_half_squared = (
            _gram_dot(direction, direction, gram)
            / (base_scale * base_scale)
            / 4
        )
        base_longitudinal = abs(Fraction(base_k) - base_projection)
        refined_scale = base_scale * factor
        refined_anchor = (
            info["base_start_node"][0] * factor,
            info["base_start_node"][1] * factor,
        )
        refined_k = base_k * factor
        refined_node = base_node[0] * factor, base_node[1] * factor
        refined_coordinate = (
            Fraction(refined_node[0], refined_scale),
            Fraction(refined_node[1], refined_scale),
        )
        refined_projection = _projection(
            (
                source_point[0] * refined_scale,
                source_point[1] * refined_scale,
            ),
            refined_anchor,
            direction,
            gram,
        )
        refined_half_squared = (
            _gram_dot(direction, direction, gram)
            / (refined_scale * refined_scale)
            / 4
        )
        refined_longitudinal = abs(
            Fraction(refined_k) - refined_projection
        )
        refined_displacement = (
            refined_coordinate[0] - source_point[0],
            refined_coordinate[1] - source_point[1],
        )
        displacement_unchanged = displacement == refined_displacement
        cross = gate_a._cross(
            (
                refined_node[0] - refined_anchor[0],
                refined_node[1] - refined_anchor[1],
            ),
            direction,
        )
        authority_pass = (
            base_node == reproduced_nodes[vertex_id]
            and base_coordinate == refined_coordinate
            and displacement_unchanged
            and base_component_budget_pass
            and cross == 0
            and info["source_vertex_ids"][ordinal] == vertex_id
        )
        records.append(
            {
                "ordinal": ordinal,
                "source_vertex_id": vertex_id.value,
                "authority": "BASE_BOUND_V1_FIXED_NOT_REFINED",
                "base_V1_node": list(base_node),
                "base_V1_coordinate": [
                    _fraction(item) for item in base_coordinate
                ],
                "source_coordinate": [
                    _fraction(item) for item in source_point
                ],
                "exact_source_to_base_displacement": [
                    _fraction(item) for item in displacement
                ],
                "prior_V1_snap_node_identity_pass": (
                    base_node == reproduced_nodes[vertex_id]
                ),
                "exact_unchanged_coordinate_pass": (
                    base_coordinate == refined_coordinate
                ),
                "exact_unchanged_displacement_pass": (
                    displacement_unchanged
                ),
                "source_order_identity_pass": (
                    info["source_vertex_ids"][ordinal] == vertex_id
                ),
                "exact_cross_with_bound_line": _fraction(cross),
                "base_V1_component_half_cell_bound": _fraction(
                    base_component_bound
                ),
                "base_V1_coordinate_half_cell_budget_pass": (
                    base_component_budget_pass
                ),
                "base_V1_line_period_diagnostic_only": {
                    "gram_displacement_squared": _fraction(
                        base_gram_squared
                    ),
                    "half_step_gram_squared_bound": _fraction(
                        base_half_squared
                    ),
                    "full_gram_boundary_pass_non_gating": (
                        base_gram_squared <= base_half_squared
                    ),
                    "longitudinal_displacement_k": _fraction(
                        base_longitudinal
                    ),
                    "longitudinal_boundary_pass_non_gating": (
                        base_longitudinal <= Fraction(1, 2)
                    ),
                },
                "S_prime_diagnostic_only": {
                    "r": r,
                    "refined_representation_node": list(refined_node),
                    "exact_same_physical_coordinate": [
                        _fraction(item) for item in refined_coordinate
                    ],
                    "gram_half_step_squared_bound": _fraction(
                        refined_half_squared
                    ),
                    "full_gram_boundary_pass_non_gating": (
                        base_gram_squared <= refined_half_squared
                    ),
                    "longitudinal_displacement_k": _fraction(
                        refined_longitudinal
                    ),
                    "longitudinal_boundary_pass_non_gating": (
                        refined_longitudinal <= Fraction(1, 2)
                    ),
                },
                "authority_pass": authority_pass,
            }
        )
    return records


def _non_chain_authority(
    source_by_id: dict,
    base_node_by_id: dict,
    reproduced_nodes: dict,
    chain_vertex_ids: set,
    base_scale: int,
) -> list[dict[str, Any]]:
    records = []
    component_bound = Fraction(1, 2 * base_scale)
    for vertex_id in sorted(
        set(base_node_by_id) - chain_vertex_ids,
        key=lambda item: item.value,
    ):
        source = source_by_id[vertex_id]
        node = base_node_by_id[vertex_id]
        coordinate = (
            Fraction(node[0], base_scale),
            Fraction(node[1], base_scale),
        )
        prior_node = reproduced_nodes[vertex_id]
        prior_coordinate = (
            Fraction(prior_node[0], base_scale),
            Fraction(prior_node[1], base_scale),
        )
        displacement = (
            coordinate[0] - source[0],
            coordinate[1] - source[1],
        )
        prior_displacement = (
            prior_coordinate[0] - source[0],
            prior_coordinate[1] - source[1],
        )
        records.append(
            {
                "source_vertex_id": vertex_id.value,
                "authority": "BASE_BOUND_V1_NON_CHAIN_FIXED_NOT_REFINED",
                "base_V1_node": list(node),
                "base_V1_coordinate": [
                    _fraction(item) for item in coordinate
                ],
                "source_coordinate": [_fraction(item) for item in source],
                "exact_source_to_base_displacement": [
                    _fraction(item) for item in displacement
                ],
                "component_half_step_bound": _fraction(component_bound),
                "component_snap_budget_pass": all(
                    abs(item) <= component_bound for item in displacement
                ),
                "prior_V1_snap_node_identity_pass": (
                    node == prior_node
                ),
                "exact_unchanged_coordinate_pass": (
                    coordinate == prior_coordinate
                ),
                "exact_unchanged_displacement_pass": (
                    displacement == prior_displacement
                ),
                "authority_pass": (
                    node == prior_node
                    and coordinate == prior_coordinate
                    and displacement == prior_displacement
                    and all(
                        abs(item) <= component_bound for item in displacement
                    )
                ),
            }
        )
    return records


def _euclidean_diagnostic(
    info: dict[str, Any],
    base_scale: int,
    r: int,
    gram_candidate: dict[str, Any],
) -> dict[str, Any]:
    candidate = gate_a2._assign_norm(
        info["source"],
        info["base_start_node"],
        info["primitive_direction"],
        info["base_endpoint_span"],
        base_scale,
        r,
        None,
        "CHART_EUCLIDEAN_DIAGNOSTIC_NON_GATING",
    )
    gram_sequence = gram_candidate["k_sequence"]
    euclidean_sequence = candidate["k_sequence"]
    return {
        "scope": "DIAGNOSTIC_ONLY_NON_GATING",
        "capacity_feasible": candidate["capacity_feasible"],
        "unconstrained_internal_k": [
            item["unconstrained_nearest_half_up_k"]
            for item in candidate["steps"]
        ],
        "selected_k_sequence": euclidean_sequence,
        "canonical_gram_selected_k_sequence": gram_sequence,
        "selected_k_disagreement": (
            gram_sequence is not None
            and euclidean_sequence is not None
            and gram_sequence != euclidean_sequence
        ),
        "disagreement_ordinals": (
            []
            if gram_sequence is None or euclidean_sequence is None
            else [
                ordinal
                for ordinal, (gram_k, euclidean_k) in enumerate(
                    zip(gram_sequence, euclidean_sequence)
                )
                if gram_k != euclidean_k
            ]
        ),
    }


def _analyze_fixture(
    kernel,
    fixture_dir: Path,
    prior_fixture: dict[str, Any],
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
    reproduced_nodes = {
        vertex_id: (
            gate_a._snap_half_up(point[0], base_scale),
            gate_a._snap_half_up(point[1], base_scale),
        )
        for vertex_id, point in source_by_id.items()
    }
    if reproduced_nodes != base_node_by_id:
        raise ValueError("base V1 binding law was not reproduced")
    base_hash = gate_a2._base_node_hash(base_node_by_id)
    prior_binding_pass = (
        prior_fixture["base_binding_node_sha256_before"] == base_hash
        and prior_fixture["base_binding_node_sha256_after"] == base_hash
        and prior_fixture["base_binding_nodes_unchanged"]
    )

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
            gate_a2._chain_info(
                snapshot, chain, source_by_id, base_node_by_id
            )
        )
    if not declared_chains:
        raise ValueError("fixture has no declared-straight multi-vertex chains")
    chain_vertex_ids = {
        vertex_id
        for info in declared_chains
        for vertex_id in info["source_vertex_ids"]
    }
    non_chain = _non_chain_authority(
        source_by_id,
        base_node_by_id,
        reproduced_nodes,
        chain_vertex_ids,
        base_scale,
    )
    non_chain_failures = sum(not item["authority_pass"] for item in non_chain)

    attempts = []
    for r in range(R_MAX + 1):
        chain_attempts = []
        for info in declared_chains:
            candidate = _canonical_gram_assignment(
                info, base_scale, r, gram
            )
            matrix = _review_gram_assignment(
                candidate, info, base_scale, r, gram
            )
            endpoints = _endpoint_authority(
                info, base_scale, r, gram, reproduced_nodes
            )
            matrix["base_endpoint_authority_failures"] = sum(
                not item["authority_pass"] for item in endpoints
            )
            matrix["source_order_failures"] += sum(
                not item["source_order_identity_pass"] for item in endpoints
            )
            matrix["nonzero_cross"] += sum(
                item["exact_cross_with_bound_line"] != "0"
                for item in endpoints
            )
            matrix["base_non_chain_authority_failures"] = non_chain_failures
            matrix["prior_v1_binding_failures"] = int(
                not prior_binding_pass
            )
            chain_attempts.append(
                {
                    "physical_chain_id": info["physical_chain_id"],
                    "ordered_source_vertex_ids": [
                        item.value for item in info["source_vertex_ids"]
                    ],
                    "vertex_count": info["vertex_count"],
                    "internal_vertex_count": info["internal_vertex_count"],
                    "base_endpoint_span_K": info["base_endpoint_span"],
                    "K_prime": (
                        info["base_endpoint_span"] * (1 << r)
                    ),
                    "primitive_direction": list(
                        info["primitive_direction"]
                    ),
                    "canonical_gram_assignment": candidate,
                    "stepwise_reviewer_failure_matrix": matrix,
                    "endpoint_BASE_BOUND_authority": endpoints,
                    "euclidean_diagnostic_non_gating": (
                        _euclidean_diagnostic(
                            info, base_scale, r, candidate
                        )
                    ),
                }
            )
        domain_matrix = {
            name: sum(
                item["stepwise_reviewer_failure_matrix"][name]
                for item in chain_attempts
            )
            for name in MATRIX_NAMES
        }
        attempts.append(
            {
                "r": r,
                "base_scale_S": base_scale,
                "refined_scale_S_prime": base_scale * (1 << r),
                "reviewer_failure_matrix": domain_matrix,
                "reviewer_admissible": all(
                    value == 0 for value in domain_matrix.values()
                ),
                "chain_attempts": chain_attempts,
            }
        )
    admissible = [item for item in attempts if item["reviewer_admissible"]]
    r_min = admissible[0]["r"] if admissible else None
    if r_min is None:
        minimality = {
            "outcome": "NO_R_LE_8",
            "per_r_failure_matrix": [
                {
                    "r": item["r"],
                    "failure_matrix": item["reviewer_failure_matrix"],
                }
                for item in attempts
            ],
        }
    elif r_min == 0:
        minimality = {
            "outcome": "BASE_SCALE_FULLY_ADMISSIBLE",
            "previous_r": None,
        }
    else:
        minimality = {
            "outcome": "PREVIOUS_R_NOT_ADMISSIBLE",
            "previous_r": r_min - 1,
            "failure_matrix": attempts[r_min - 1][
                "reviewer_failure_matrix"
            ],
        }
    return {
        "fixture_id": fixture_dir.name,
        "fixture_hash": manifest["fixture_hash"],
        "patch_domain_id": domains[0].patch_domain_id.value,
        "base_scale_S": base_scale,
        "exact_gram_matrix": [
            [_fraction(item) for item in row] for row in gram
        ],
        "canonical_norm": "RationalAffinePlanarMetricV2_GRAM",
        "base_binding_node_sha256": base_hash,
        "prior_A2_base_binding_identity_pass": prior_binding_pass,
        "declared_straight_chain_count": len(declared_chains),
        "internal_vertex_count": sum(
            item["internal_vertex_count"] for item in declared_chains
        ),
        "non_chain_BASE_BOUND_authority": non_chain,
        "non_chain_vertex_count": len(non_chain),
        "attempts": attempts,
        "r_min": r_min,
        "minimality_witness": minimality,
        "r_min_hypothesis_non_gating": R_MIN_HYPOTHESIS[fixture_dir.name],
        "hypothesis_matches_observation": (
            r_min == R_MIN_HYPOTHESIS[fixture_dir.name]
        ),
    }


def _synthetic_budget_refusal_self_check() -> dict[str, Any]:
    vertex_count = 258
    base_k = 1
    attempts = [
        {
            "r": r,
            "K_prime": base_k * (1 << r),
            "capacity_required": vertex_count - 1,
            "capacity_feasible": (
                base_k * (1 << r) >= vertex_count - 1
            ),
        }
        for r in range(R_MAX + 1)
    ]
    refused = not any(item["capacity_feasible"] for item in attempts)
    return {
        "synthetic_vertex_count": vertex_count,
        "synthetic_base_endpoint_span_K": base_k,
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
    terminal_counters: dict[str, int],
) -> dict[str, Any]:
    diagnostic_counters = {name: 0 for name in MATRIX_NAMES}
    euclidean_disagreements = 0
    clamped_steps_at_r_min = 0
    unclamped_steps_at_r_min = 0
    for fixture in fixtures:
        for attempt in fixture["attempts"]:
            for name in MATRIX_NAMES:
                diagnostic_counters[name] += attempt[
                    "reviewer_failure_matrix"
                ][name]
            euclidean_disagreements += sum(
                item["euclidean_diagnostic_non_gating"][
                    "selected_k_disagreement"
                ]
                for item in attempt["chain_attempts"]
            )
        if fixture["r_min"] is not None:
            selected = fixture["attempts"][fixture["r_min"]]
            for chain in selected["chain_attempts"]:
                for step in chain["canonical_gram_assignment"]["steps"]:
                    if step["CLAMPED"]:
                        clamped_steps_at_r_min += 1
                    else:
                        unclamped_steps_at_r_min += 1
    return {
        "fixture_count": len(fixtures),
        "declared_straight_chain_count": sum(
            item["declared_straight_chain_count"] for item in fixtures
        ),
        "internal_vertex_count": sum(
            item["internal_vertex_count"] for item in fixtures
        ),
        "non_chain_vertex_count": sum(
            item["non_chain_vertex_count"] for item in fixtures
        ),
        "domain_r_min": {
            item["fixture_id"]: item["r_min"] for item in fixtures
        },
        "r_min_hypothesis_non_gating": R_MIN_HYPOTHESIS,
        "all_hypotheses_match": all(
            item["hypothesis_matches_observation"] for item in fixtures
        ),
        "canonical_gram_search_diagnostic_counters": diagnostic_counters,
        "euclidean_selected_k_disagreement_chain_attempts_non_gating": (
            euclidean_disagreements
        ),
        "clamped_internal_steps_at_r_min": clamped_steps_at_r_min,
        "unclamped_internal_steps_at_r_min": unclamped_steps_at_r_min,
        "terminal_failure_counters": terminal_counters,
    }


def main() -> None:
    arguments = _arguments()
    terminal_counters = {
        name: 0 for name in TERMINAL_COUNTER_NAMES
    }
    failures: list[dict[str, Any]] = []
    fixtures: list[dict[str, Any]] = []
    product_tree_oid: str | None = None
    prior_receipt_path = _SCRIPT_DIR / "gate_a_double_prime_receipt.json"
    prior_receipt_sha256: str | None = None
    prior_by_fixture: dict[str, Any] = {}
    budget_self_check = _synthetic_budget_refusal_self_check()
    if not budget_self_check["named_refusal_reached"]:
        _terminal_failure(
            terminal_counters,
            failures,
            "refinement_budget_refusal_self_check_failures",
            "REFINEMENT_BUDGET_REFUSAL_NOT_REACHABLE",
            "synthetic r_max=8 capacity case did not refuse",
        )
    try:
        prior_bytes = prior_receipt_path.read_bytes()
        prior_receipt_sha256 = _sha256(prior_bytes)
        if prior_receipt_sha256 != EXPECTED_A2_RECEIPT_SHA256:
            _terminal_failure(
                terminal_counters,
                failures,
                "unknown_prior_a2_evidence",
                "UNKNOWN_PRIOR_A2_RECEIPT",
                "Gate A-triple-prime requires the accepted A-double-prime receipt",
                observed_sha256=prior_receipt_sha256,
            )
        else:
            prior_payload = json.loads(prior_bytes)
            prior_by_fixture = {
                item["fixture_id"]: item
                for item in prior_payload["fixtures"]
            }
    except Exception as error:
        _terminal_failure(
            terminal_counters,
            failures,
            "unknown_prior_a2_evidence",
            "PRIOR_A2_EVIDENCE_FORM_UNKNOWN",
            f"{type(error).__name__}: {error}",
        )
    try:
        product_tree_oid = gate_a._git_value(
            arguments.source_root, "HEAD:kernel/src"
        )
        if product_tree_oid != EXPECTED_PRODUCT_TREE_OID:
            _terminal_failure(
                terminal_counters,
                failures,
                "unknown_product_tree",
                "UNKNOWN_KERNEL_SRC_PRODUCT_TREE",
                "Gate A-triple-prime has no oracle for this product tree",
                kernel_src_product_tree_oid=product_tree_oid,
            )
        elif len(prior_by_fixture) == len(FIXTURE_IDS):
            kernel = gate_a._activate_kernel(arguments.source_root)
            present = {
                item.name
                for item in arguments.fixture_root.iterdir()
                if item.is_dir() and item.name in FIXTURE_IDS
            }
            if present != set(FIXTURE_IDS):
                _terminal_failure(
                    terminal_counters,
                    failures,
                    "unknown_fixture_form",
                    "UNKNOWN_FIXTURE_SET",
                    "one or more required lost-domain fixtures are absent",
                )
            else:
                for fixture_id in FIXTURE_IDS:
                    try:
                        fixture = _analyze_fixture(
                            kernel,
                            arguments.fixture_root / fixture_id,
                            prior_by_fixture[fixture_id],
                        )
                        fixtures.append(fixture)
                        if fixture["r_min"] is None:
                            _terminal_failure(
                                terminal_counters,
                                failures,
                                "no_admissible_refinement_domains",
                                "NO_ADMISSIBLE_REFINEMENT_R_LE_8",
                                "no r in 0..8 passes the canonical Gram law",
                                fixture_id=fixture_id,
                            )
                    except Exception as error:
                        _terminal_failure(
                            terminal_counters,
                            failures,
                            "unknown_fixture_form",
                            "UNKNOWN_FIXTURE_FORM",
                            f"{type(error).__name__}: {error}",
                            fixture_id=fixture_id,
                        )
    except Exception as error:
        _terminal_failure(
            terminal_counters,
            failures,
            "unknown_fixture_form",
            "GATE_A_TRIPLE_PRIME_EXECUTION_FORM_UNKNOWN",
            f"{type(error).__name__}: {error}",
        )

    aggregate = _aggregate(fixtures, terminal_counters)
    observed_coverage = {
        name: aggregate[name] for name in EXPECTED_COVERAGE
    }
    if observed_coverage != EXPECTED_COVERAGE:
        _terminal_failure(
            terminal_counters,
            failures,
            "coverage_failures",
            "A_TRIPLE_PRIME_COVERAGE_MISMATCH",
            "fixture/chain/internal-vertex coverage is not exact",
            expected=EXPECTED_COVERAGE,
            observed=observed_coverage,
        )
        aggregate["terminal_failure_counters"] = terminal_counters
    outcome = (
        "GATE_A_TRIPLE_PRIME_GO"
        if not failures
        else "GATE_A_TRIPLE_PRIME_REFUSED"
    )
    payload = {
        "schema": SCHEMA,
        "outcome": outcome,
        "proof_base_revision": PROOF_BASE_REVISION,
        "kernel_src_product_tree_oid": product_tree_oid,
        "accepted_kernel_src_product_tree_oid": EXPECTED_PRODUCT_TREE_OID,
        "prior_A2_receipt": {
            "path": "gate_a_double_prime_receipt.json",
            "accepted_sha256": EXPECTED_A2_RECEIPT_SHA256,
            "observed_sha256": prior_receipt_sha256,
            "identity_pass": (
                prior_receipt_sha256 == EXPECTED_A2_RECEIPT_SHA256
            ),
        },
        "canonical_law": {
            "sole_canonical_squared_norm": (
                "RationalAffinePlanarMetricV2_GRAM"
            ),
            "euclidean_scope": "DIAGNOSTIC_ONLY_NON_GATING",
            "refined_capacity_scope": (
                "DECLARED_CHAIN_INTERNAL_VERTICES_ONLY"
            ),
            "S_prime_boundary_scope": (
                "DECLARED_CHAIN_INTERNAL_VERTICES_ONLY"
            ),
            "BASE_BOUND_S_prime_diagnostics_gate": False,
            "ordered_assignment": (
                "EXACT_GRAM_HALF_UP_THEN_CLAMP_PREVIOUS_PLUS_ONE_TO_K_PRIME_MINUS_REMAINING"
            ),
            "unclamped_boundary": (
                "EXACT_GRAM_SOURCE_TO_NODE_LE_HALF_S_PRIME_LINE_STEP"
            ),
            "clamped_excess": "CLAMPED_CONSTRAINT_EXCESS_ALLOWED",
            "endpoint_authority": (
                "BASE_BOUND_V1_FIXED_NOT_REFINED_BASE_SCALE_GATING_S_PRIME_DIAGNOSTIC"
            ),
            "non_chain_authority": "BASE_BOUND_V1_FIXED_NOT_REFINED",
        },
        "r_domain": {"minimum": 0, "maximum": R_MAX},
        "synthetic_budget_refusal_self_check": budget_self_check,
        "coverage_expected": EXPECTED_COVERAGE,
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
                "internal_vertex_count": aggregate[
                    "internal_vertex_count"
                ],
                "non_chain_vertex_count": aggregate[
                    "non_chain_vertex_count"
                ],
                "domain_r_min": aggregate["domain_r_min"],
                "all_hypotheses_match": aggregate[
                    "all_hypotheses_match"
                ],
                "canonical_gram_search_diagnostic_counters": aggregate[
                    "canonical_gram_search_diagnostic_counters"
                ],
                "terminal_failure_counters": terminal_counters,
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
