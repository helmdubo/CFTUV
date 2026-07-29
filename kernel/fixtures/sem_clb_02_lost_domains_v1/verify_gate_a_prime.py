"""Exact reachability proof for SEM-CLB-02-R Gate A-prime.

This is fixture evidence only.  It does not construct a product binding and
does not authorize any kernel behavior.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from math import gcd
from pathlib import Path
import subprocess
import sys
from typing import Any


SCHEMA = "cftuv.envelope.sem_clb_02_gate_a_prime.v1"
PROOF_BASE_REVISION = "42b70ac4e3fe2f1ebefe22228c20da68644aeeac"
EXPECTED_PRODUCT_TREE_OID = "000fe75684dbd7dbc57dc782b52b1b569dd1d215"
FIXTURE_IDS = (
    "building_all_seams_patch_001_lost_resolved_v1",
    "building_all_seams_patch_006_lost_resolved_v1",
    "building_all_seams_patch_011_lost_resolved_v1",
    "building_all_seams_patch_105_lost_resolved_v1",
)
FAILURE_COUNTER_NAMES = (
    "ties_chart_euclidean",
    "ties_rational_affine_metric",
    "norm_k_disagreements",
    "sticky_k",
    "reversals",
    "order_failures",
    "nonzero_cross",
    "source_declaration_contradictions",
    "unknown_fixture_form",
    "unknown_product_tree",
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


def _fraction(value: Fraction | int) -> str:
    return str(Fraction(value))


def _point(value) -> tuple[Fraction, Fraction]:
    return (
        Fraction(value.x.numerator, value.x.denominator),
        Fraction(value.y.numerator, value.y.denominator),
    )


def _git_value(source_root: Path, expression: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source_root.resolve()), "rev-parse", expression],
        text=True,
    ).strip()


def _activate_kernel(source_root: Path):
    source_root = source_root.resolve()
    for module_name in tuple(sys.modules):
        if (
            module_name == "cftuv_envelope"
            or module_name.startswith("cftuv_envelope.")
        ):
            del sys.modules[module_name]
    value = str(source_root / "kernel" / "src")
    if value in sys.path:
        sys.path.remove(value)
    sys.path.insert(0, value)
    import cftuv_envelope as kernel

    return kernel


def _snap_half_up(value: Fraction, scale: int) -> int:
    scaled = value * scale
    return (2 * scaled.numerator + scaled.denominator) // (
        2 * scaled.denominator
    )


def _nearest_integer(value: Fraction) -> tuple[int | None, bool]:
    lower = value.numerator // value.denominator
    residual = value - lower
    if residual == Fraction(1, 2):
        return None, True
    return (lower if residual < Fraction(1, 2) else lower + 1), False


def _cross(
    first: tuple[Fraction | int, Fraction | int],
    second: tuple[Fraction | int, Fraction | int],
) -> Fraction:
    return Fraction(first[0]) * Fraction(second[1]) - Fraction(
        first[1]
    ) * Fraction(second[0])


def _dot(
    first: tuple[Fraction | int, Fraction | int],
    second: tuple[Fraction | int, Fraction | int],
) -> Fraction:
    return Fraction(first[0]) * Fraction(second[0]) + Fraction(
        first[1]
    ) * Fraction(second[1])


def _gram_dot(
    first: tuple[Fraction | int, Fraction | int],
    second: tuple[Fraction | int, Fraction | int],
    gram: tuple[tuple[Fraction, Fraction], tuple[Fraction, Fraction]],
) -> Fraction:
    return (
        Fraction(first[0])
        * (gram[0][0] * Fraction(second[0]) + gram[0][1] * Fraction(second[1]))
        + Fraction(first[1])
        * (gram[1][0] * Fraction(second[0]) + gram[1][1] * Fraction(second[1]))
    )


def _subtract(first, second) -> tuple[Fraction, Fraction]:
    return Fraction(first[0]) - Fraction(second[0]), Fraction(
        first[1]
    ) - Fraction(second[1])


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


def _validate_fixture_files(
    fixture_dir: Path,
    manifest: dict[str, Any],
) -> None:
    if manifest.get("schema") != "cftuv.envelope.sem_clb_02_domain_fixture.v2":
        raise ValueError("fixture schema is not SEM-CLB-02 v2")
    if manifest.get("fixture_id") != fixture_dir.name:
        raise ValueError("fixture_id does not match directory")
    if manifest.get("selection_scenario") != "all_seams":
        raise ValueError("fixture is not an all_seams case")
    if manifest.get("object_name") != "building":
        raise ValueError("fixture object is not building")
    if not manifest.get("source_mesh_unchanged"):
        raise ValueError("fixture does not certify an unchanged source mesh")
    registry = manifest.get(
        "expected_kernel_results_by_kernel_src_tree"
    )
    if (
        not isinstance(registry, dict)
        or EXPECTED_PRODUCT_TREE_OID not in registry
    ):
        raise ValueError("fixture has no oracle for the accepted product tree")
    files = manifest.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("fixture file hash registry is missing")
    actual = {
        name: _sha256((fixture_dir / name).read_bytes())
        for name in sorted(files)
    }
    if actual != files:
        raise ValueError("fixture artifact hashes do not match manifest")
    fixture_hash = _sha256(
        _canonical_json([[name, actual[name]] for name in sorted(actual)])
    )
    if fixture_hash != manifest.get("fixture_hash"):
        raise ValueError("fixture_hash does not match artifact hashes")


def _gram_matrix(frame) -> tuple[
    tuple[Fraction, Fraction],
    tuple[Fraction, Fraction],
]:
    matrix = frame.exact_gram_matrix

    def value(item) -> Fraction:
        return Fraction(item.numerator, item.denominator)

    gram = (
        (value(matrix.m00), value(matrix.m01)),
        (value(matrix.m10), value(matrix.m11)),
    )
    if gram[0][1] != gram[1][0]:
        raise ValueError("exact Gram matrix is not symmetric")
    return gram


def _declared_internal_corner_ids(snapshot, chain) -> set:
    internal_ids = set(chain.ordered_source_vertex_ids[1:-1])
    uses = tuple(
        item
        for item in snapshot.chain_uses
        if item.physical_chain_id == chain.physical_chain_id
    )
    if not uses:
        raise ValueError(
            f"{chain.physical_chain_id.value} has no ChainUse declaration"
        )
    sector_ids = {
        sector.owner_sector_id
        for sector in snapshot.angular_owner_sectors
        if any(
            use.chain_use_id in sector.ordered_incident_chain_use_ids
            for use in uses
        )
    }
    return {
        relation.source_vertex_id
        for relation in snapshot.corner_relations
        if relation.owner_sector_id in sector_ids
        and relation.source_vertex_id in internal_ids
    }


def _projection_parameter(
    source_scaled: tuple[Fraction, Fraction],
    anchor: tuple[int, int],
    direction: tuple[int, int],
    gram: tuple[tuple[Fraction, Fraction], tuple[Fraction, Fraction]] | None,
) -> Fraction:
    offset = _subtract(source_scaled, anchor)
    if gram is None:
        numerator = _dot(direction, offset)
        denominator = _dot(direction, direction)
    else:
        numerator = _gram_dot(direction, offset, gram)
        denominator = _gram_dot(direction, direction, gram)
    if denominator <= 0:
        raise ValueError("line direction has non-positive squared period")
    return numerator / denominator


def _analyze_chain(
    chain,
    source_by_id: dict,
    bound_node_by_id: dict,
    scale: int,
    gram: tuple[tuple[Fraction, Fraction], tuple[Fraction, Fraction]],
    counters: dict[str, int],
    failures: list[dict[str, Any]],
    fixture_id: str,
) -> dict[str, Any] | None:
    chain_id = chain.physical_chain_id.value
    vertex_ids = chain.ordered_source_vertex_ids
    source = tuple(source_by_id[item] for item in vertex_ids)
    source_direction = _subtract(source[-1], source[0])
    if source_direction == (0, 0):
        _failure(
            counters,
            failures,
            "source_declaration_contradictions",
            "SOURCE_DECLARED_STRAIGHT_ENDPOINTS_COINCIDE",
            "source endpoints of a declared-straight chain coincide",
            fixture_id=fixture_id,
            physical_chain_id=chain_id,
        )
        return None
    source_crosses = tuple(
        _cross(source_direction, _subtract(point, source[0]))
        for point in source
    )
    if any(value != 0 for value in source_crosses):
        _failure(
            counters,
            failures,
            "source_declaration_contradictions",
            "SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR",
            "a declared-straight PhysicalChain is not exactly source-collinear",
            fixture_id=fixture_id,
            physical_chain_id=chain_id,
        )
        return None

    start_node = bound_node_by_id[vertex_ids[0]]
    end_node = bound_node_by_id[vertex_ids[-1]]
    delta = end_node[0] - start_node[0], end_node[1] - start_node[1]
    divisor = gcd(abs(delta[0]), abs(delta[1]))
    if divisor == 0:
        _failure(
            counters,
            failures,
            "unknown_fixture_form",
            "BOUND_ENDPOINTS_COINCIDE",
            "base-law bound endpoints do not define a line",
            fixture_id=fixture_id,
            physical_chain_id=chain_id,
        )
        return None
    direction = delta[0] // divisor, delta[1] // divisor
    euclidean_period_squared = Fraction(
        direction[0] * direction[0] + direction[1] * direction[1],
        scale * scale,
    )
    metric_period_squared = _gram_dot(direction, direction, gram) / (
        scale * scale
    )
    if metric_period_squared <= 0:
        raise ValueError(f"{chain_id} has non-positive metric line period")

    vertices: list[dict[str, Any]] = []
    assigned_k: list[int | None] = []
    for ordinal, (vertex_id, source_point) in enumerate(zip(vertex_ids, source)):
        source_scaled = source_point[0] * scale, source_point[1] * scale
        endpoint = ordinal in (0, len(vertex_ids) - 1)
        if endpoint:
            k_euclidean = 0 if ordinal == 0 else divisor
            k_metric = k_euclidean
            tie_euclidean = False
            tie_metric = False
            parameter_euclidean = _projection_parameter(
                source_scaled, start_node, direction, None
            )
            parameter_metric = _projection_parameter(
                source_scaled, start_node, direction, gram
            )
        else:
            parameter_euclidean = _projection_parameter(
                source_scaled, start_node, direction, None
            )
            parameter_metric = _projection_parameter(
                source_scaled, start_node, direction, gram
            )
            k_euclidean, tie_euclidean = _nearest_integer(
                parameter_euclidean
            )
            k_metric, tie_metric = _nearest_integer(parameter_metric)
            if tie_euclidean:
                _failure(
                    counters,
                    failures,
                    "ties_chart_euclidean",
                    "NEAREST_K_TIE_CHART_EUCLIDEAN",
                    "chart Euclidean projection is exactly at half line-period",
                    fixture_id=fixture_id,
                    physical_chain_id=chain_id,
                    source_vertex_id=vertex_id.value,
                )
            if tie_metric:
                _failure(
                    counters,
                    failures,
                    "ties_rational_affine_metric",
                    "NEAREST_K_TIE_RATIONAL_AFFINE_METRIC",
                    "RationalAffinePlanarMetric projection is exactly at half line-period",
                    fixture_id=fixture_id,
                    physical_chain_id=chain_id,
                    source_vertex_id=vertex_id.value,
                )
            if (
                not tie_euclidean
                and not tie_metric
                and k_euclidean != k_metric
            ):
                _failure(
                    counters,
                    failures,
                    "norm_k_disagreements",
                    "NEAREST_K_NORM_DISAGREEMENT",
                    "chart Euclidean and RationalAffinePlanarMetric choose different k",
                    fixture_id=fixture_id,
                    physical_chain_id=chain_id,
                    source_vertex_id=vertex_id.value,
                )

        assigned = (
            k_euclidean
            if (
                k_euclidean is not None
                and k_metric is not None
                and k_euclidean == k_metric
            )
            else None
        )
        assigned_k.append(assigned)
        assigned_node = (
            None
            if assigned is None
            else (
                start_node[0] + assigned * direction[0],
                start_node[1] + assigned * direction[1],
            )
        )
        line_cross = (
            None
            if assigned_node is None
            else _cross(
                (
                    assigned_node[0] - start_node[0],
                    assigned_node[1] - start_node[1],
                ),
                direction,
            )
        )
        if line_cross not in (None, 0):
            _failure(
                counters,
                failures,
                "nonzero_cross",
                "ASSIGNED_NODE_NOT_ON_BOUND_LINE",
                "assigned lattice node has nonzero exact cross with bound line",
                fixture_id=fixture_id,
                physical_chain_id=chain_id,
                source_vertex_id=vertex_id.value,
            )
        displacement = (
            None
            if assigned_node is None
            else (
                Fraction(assigned_node[0], scale) - source_point[0],
                Fraction(assigned_node[1], scale) - source_point[1],
            )
        )
        euclidean_squared = (
            None if displacement is None else _dot(displacement, displacement)
        )
        metric_squared = (
            None
            if displacement is None
            else _gram_dot(displacement, displacement, gram)
        )
        residual_euclidean = (
            None
            if assigned is None
            else Fraction(assigned) - parameter_euclidean
        )
        residual_metric = (
            None if assigned is None else Fraction(assigned) - parameter_metric
        )
        vertices.append(
            {
                "ordinal": ordinal,
                "source_vertex_id": vertex_id.value,
                "is_endpoint": endpoint,
                "source_coordinate": [_fraction(item) for item in source_point],
                "base_bound_node": list(bound_node_by_id[vertex_id]),
                "projection_k_chart_euclidean": _fraction(parameter_euclidean),
                "projection_k_rational_affine_metric": _fraction(parameter_metric),
                "nearest_k_chart_euclidean": k_euclidean,
                "nearest_k_rational_affine_metric": k_metric,
                "nearest_k_norms_agree": (
                    k_euclidean is not None
                    and k_metric is not None
                    and k_euclidean == k_metric
                ),
                "assigned_k": assigned,
                "assigned_bound_line_node": (
                    None if assigned_node is None else list(assigned_node)
                ),
                "assigned_coordinate": (
                    None
                    if assigned_node is None
                    else [
                        _fraction(Fraction(item, scale))
                        for item in assigned_node
                    ]
                ),
                "exact_cross_with_bound_line": (
                    None if line_cross is None else _fraction(line_cross)
                ),
                "displacement": (
                    None
                    if displacement is None
                    else [_fraction(item) for item in displacement]
                ),
                "displacement_squared_chart_euclidean": (
                    None
                    if euclidean_squared is None
                    else _fraction(euclidean_squared)
                ),
                "displacement_squared_rational_affine_metric": (
                    None if metric_squared is None else _fraction(metric_squared)
                ),
                "signed_k_residual_chart_euclidean": (
                    None
                    if residual_euclidean is None
                    else _fraction(residual_euclidean)
                ),
                "signed_k_residual_rational_affine_metric": (
                    None
                    if residual_metric is None
                    else _fraction(residual_metric)
                ),
                "nearest_boundary_margin_k_chart_euclidean": (
                    None
                    if residual_euclidean is None
                    else _fraction(
                        Fraction(1, 2) - abs(residual_euclidean)
                    )
                ),
                "nearest_boundary_margin_k_rational_affine_metric": (
                    None
                    if residual_metric is None
                    else _fraction(Fraction(1, 2) - abs(residual_metric))
                ),
            }
        )

    if all(value is not None for value in assigned_k):
        concrete_k = [int(value) for value in assigned_k if value is not None]
        duplicate_count = len(concrete_k) - len(set(concrete_k))
        if duplicate_count:
            counters["sticky_k"] += duplicate_count
            failures.append(
                {
                    "outcome": "STICKY_K",
                    "detail": "two or more chain vertices were assigned the same k",
                    "fixture_id": fixture_id,
                    "physical_chain_id": chain_id,
                    "duplicate_assignment_count": duplicate_count,
                }
            )
        gaps = [
            current - previous
            for previous, current in zip(concrete_k, concrete_k[1:])
        ]
        reversal_count = sum(value < 0 for value in gaps)
        if reversal_count:
            counters["reversals"] += reversal_count
            failures.append(
                {
                    "outcome": "K_ORDER_REVERSAL",
                    "detail": "k reverses against bound start-to-end direction",
                    "fixture_id": fixture_id,
                    "physical_chain_id": chain_id,
                    "reversal_count": reversal_count,
                }
            )
        strict_order = all(value > 0 for value in gaps)
        if not strict_order:
            _failure(
                counters,
                failures,
                "order_failures",
                "K_NOT_STRICTLY_ORDERED",
                "k is not strictly increasing in source chain order",
                fixture_id=fixture_id,
                physical_chain_id=chain_id,
            )
        min_gap = min(gaps) if gaps else None
    else:
        strict_order = False
        gaps = []
        min_gap = None
        _failure(
            counters,
            failures,
            "order_failures",
            "K_ORDER_UNAVAILABLE",
            "tie or norm disagreement prevented a complete k order",
            fixture_id=fixture_id,
            physical_chain_id=chain_id,
        )

    return {
        "physical_chain_id": chain_id,
        "source_vertex_count": len(vertex_ids),
        "internal_vertex_count": len(vertex_ids) - 2,
        "source_vertex_ids": [item.value for item in vertex_ids],
        "source_collinear_crosses": [_fraction(item) for item in source_crosses],
        "lattice_scale": scale,
        "bound_start_node": list(start_node),
        "bound_end_node": list(end_node),
        "primitive_direction": list(direction),
        "endpoint_k_span": divisor,
        "order_direction": "INCREASING_BOUND_START_TO_END",
        "strict_k_order": strict_order,
        "consecutive_k_gaps": gaps,
        "min_k_gap": min_gap,
        "line_period_vector": [
            _fraction(Fraction(item, scale)) for item in direction
        ],
        "half_line_period_parameter": "1/2",
        "line_period_squared_chart_euclidean": _fraction(
            euclidean_period_squared
        ),
        "half_line_period_squared_chart_euclidean": _fraction(
            euclidean_period_squared / 4
        ),
        "line_period_squared_rational_affine_metric": _fraction(
            metric_period_squared
        ),
        "half_line_period_squared_rational_affine_metric": _fraction(
            metric_period_squared / 4
        ),
        "vertices": vertices,
    }


def _analyze_fixture(
    kernel,
    fixture_dir: Path,
    counters: dict[str, int],
    failures: list[dict[str, Any]],
) -> dict[str, Any]:
    manifest = json.loads(
        (fixture_dir / "manifest.json").read_text(encoding="utf-8")
    )
    _validate_fixture_files(fixture_dir, manifest)
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
        raise ValueError(
            "fixture contracts are invalid: "
            + "; ".join(str(item) for item in issues)
        )
    domains = tuple(snapshot.patch_domains)
    if len(domains) != 1:
        raise ValueError("fixture must contain exactly one PatchDomain")
    compiled = kernel.compile_reference_envelopes(
        snapshot, request, domains[0].patch_domain_id
    )
    if getattr(compiled.outcome, "value", compiled.outcome) != "EXACT":
        raise ValueError(f"compile outcome is {compiled.outcome}")
    compilation = compiled.compilation
    if compilation is None or compilation.evaluation_geometry_binding is None:
        raise ValueError("compile did not produce an evaluation geometry binding")
    binding = compilation.evaluation_geometry_binding
    if (
        getattr(binding.binding_law, "value", binding.binding_law)
        != "EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1"
    ):
        raise ValueError("binding law is not the accepted base law")
    frame = next(
        item
        for item in snapshot.surface_metric_descriptors
        if item.patch_domain_id == domains[0].patch_domain_id
    )
    if type(frame).__name__ != "RationalAffinePlanarMetricV2":
        raise ValueError("fixture metric is not RationalAffinePlanarMetricV2")
    scale = binding.lattice_scale
    source_by_id = {
        item.source_vertex_id: _point(item.domain_coordinate)
        for item in frame.exact_source_vertex_coordinates
    }
    bound_node_by_id = {}
    for item in binding.source_vertex_coordinates:
        coordinate = _point(item.domain_coordinate)
        scaled = coordinate[0] * scale, coordinate[1] * scale
        if any(value.denominator != 1 for value in scaled):
            raise ValueError("base binding coordinate is not a lattice node")
        bound_node_by_id[item.source_vertex_id] = (
            int(scaled[0]),
            int(scaled[1]),
        )
    if source_by_id.keys() != bound_node_by_id.keys():
        raise ValueError("source and binding vertex identities differ")
    reproduced = {
        vertex_id: (
            _snap_half_up(point[0], scale),
            _snap_half_up(point[1], scale),
        )
        for vertex_id, point in source_by_id.items()
    }
    if reproduced != bound_node_by_id:
        raise ValueError("base binding does not match exact componentwise half-up")

    gram = _gram_matrix(frame)
    open_multi_vertex = tuple(
        chain
        for chain in snapshot.physical_chains
        if not chain.is_closed and len(chain.ordered_source_vertex_ids) >= 3
    )
    declared_straight = []
    excluded = []
    for chain in sorted(
        open_multi_vertex, key=lambda item: item.physical_chain_id.value
    ):
        corner_ids = _declared_internal_corner_ids(snapshot, chain)
        if corner_ids:
            excluded.append(
                {
                    "physical_chain_id": chain.physical_chain_id.value,
                    "declared_internal_corner_vertex_ids": sorted(
                        item.value for item in corner_ids
                    ),
                }
            )
        else:
            declared_straight.append(chain)
    if not declared_straight:
        raise ValueError("fixture has no declared-straight multi-vertex chains")

    chains = []
    for chain in declared_straight:
        report = _analyze_chain(
            chain,
            source_by_id,
            bound_node_by_id,
            scale,
            gram,
            counters,
            failures,
            fixture_dir.name,
        )
        if report is not None:
            chains.append(report)
    return {
        "fixture_id": fixture_dir.name,
        "fixture_hash": manifest["fixture_hash"],
        "patch_domain_id": domains[0].patch_domain_id.value,
        "lattice_scale": scale,
        "base_binding_law": "EVALUATION_GEOMETRY_CHART_LATTICE_BOUND_V1",
        "base_binding_reproduced_for_all_vertices": True,
        "exact_gram_matrix": [
            [_fraction(item) for item in row] for row in gram
        ],
        "open_multi_vertex_chain_count": len(open_multi_vertex),
        "excluded_declared_internal_corner_chain_count": len(excluded),
        "excluded_declared_internal_corner_chains": excluded,
        "declared_straight_chain_count": len(declared_straight),
        "chains": chains,
    }


def _summarize(
    fixtures: list[dict[str, Any]],
    counters: dict[str, int],
) -> dict[str, Any]:
    chains = [
        chain for fixture in fixtures for chain in fixture.get("chains", ())
    ]
    vertices = [
        vertex for chain in chains for vertex in chain.get("vertices", ())
    ]
    internal_vertices = [
        vertex for vertex in vertices if not vertex["is_endpoint"]
    ]
    gaps = [
        gap
        for chain in chains
        for gap in chain["consecutive_k_gaps"]
    ]
    positive_gaps = [gap for gap in gaps if gap > 0]

    def maximum(field: str) -> dict[str, Any] | None:
        candidates = [
            (Fraction(vertex[field]), chain, vertex)
            for fixture in fixtures
            for chain in fixture.get("chains", ())
            for vertex in chain.get("vertices", ())
            if vertex[field] is not None
        ]
        if not candidates:
            return None
        value, chain, vertex = max(candidates, key=lambda item: item[0])
        return {
            "value": _fraction(value),
            "physical_chain_id": chain["physical_chain_id"],
            "source_vertex_id": vertex["source_vertex_id"],
            "half_line_period_squared_chart_euclidean": chain[
                "half_line_period_squared_chart_euclidean"
            ],
            "half_line_period_squared_rational_affine_metric": chain[
                "half_line_period_squared_rational_affine_metric"
            ],
        }

    return {
        "fixture_count": len(fixtures),
        "declared_straight_chain_count": len(chains),
        "vertex_count": len(vertices),
        "internal_vertex_count": len(internal_vertices),
        "min_k_gap": min(gaps) if gaps else None,
        "min_positive_k_gap": (
            min(positive_gaps) if positive_gaps else None
        ),
        "all_internal_nearest_k_norms_agree": all(
            vertex["nearest_k_norms_agree"] for vertex in internal_vertices
        ),
        "max_displacement_squared_chart_euclidean": maximum(
            "displacement_squared_chart_euclidean"
        ),
        "max_displacement_squared_rational_affine_metric": maximum(
            "displacement_squared_rational_affine_metric"
        ),
        "failure_counters": counters,
    }


def main() -> None:
    arguments = _arguments()
    counters = {name: 0 for name in FAILURE_COUNTER_NAMES}
    failures: list[dict[str, Any]] = []
    fixtures: list[dict[str, Any]] = []
    product_tree_oid: str | None = None
    try:
        product_tree_oid = _git_value(
            arguments.source_root, "HEAD:kernel/src"
        )
        if product_tree_oid != EXPECTED_PRODUCT_TREE_OID:
            _failure(
                counters,
                failures,
                "unknown_product_tree",
                "UNKNOWN_KERNEL_SRC_PRODUCT_TREE",
                "Gate A-prime has no oracle for this kernel/src product tree",
                kernel_src_product_tree_oid=product_tree_oid,
            )
        else:
            kernel = _activate_kernel(arguments.source_root)
            expected_dirs = {
                item.name
                for item in arguments.fixture_root.iterdir()
                if item.is_dir() and item.name in FIXTURE_IDS
            }
            if expected_dirs != set(FIXTURE_IDS):
                _failure(
                    counters,
                    failures,
                    "unknown_fixture_form",
                    "UNKNOWN_FIXTURE_SET",
                    "one or more required lost-domain fixtures are absent",
                )
            else:
                for fixture_id in FIXTURE_IDS:
                    fixture_dir = arguments.fixture_root / fixture_id
                    try:
                        fixtures.append(
                            _analyze_fixture(
                                kernel, fixture_dir, counters, failures
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
            "GATE_A_PRIME_EXECUTION_FORM_UNKNOWN",
            f"{type(error).__name__}: {error}",
        )

    summary = _summarize(fixtures, counters)
    outcome = "EXACT_REACHABLE" if not failures else "GATE_A_PRIME_REFUSED"
    payload = {
        "schema": SCHEMA,
        "outcome": outcome,
        "proof_base_revision": PROOF_BASE_REVISION,
        "kernel_src_product_tree_oid": product_tree_oid,
        "accepted_kernel_src_product_tree_oid": EXPECTED_PRODUCT_TREE_OID,
        "nearest_k_norms": [
            "CHART_EUCLIDEAN",
            "EXACT_RATIONAL_AFFINE_PLANAR_METRIC_GRAM",
        ],
        "nearest_k_tie_policy": "FAIL_CLOSED_AT_EXACT_HALF_LINE_PERIOD",
        "declared_straight_selection_law": (
            "OPEN_MULTI_VERTEX_PHYSICAL_CHAIN_WITHOUT_DECLARED_INTERNAL_CORNER_V1"
        ),
        "summary": summary,
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
                "fixture_count": summary["fixture_count"],
                "declared_straight_chain_count": summary[
                    "declared_straight_chain_count"
                ],
                "internal_vertex_count": summary["internal_vertex_count"],
                "min_k_gap": summary["min_k_gap"],
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
