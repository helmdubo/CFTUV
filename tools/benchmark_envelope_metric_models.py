"""M-R0 research harness for planar metric models A/B/C.

This file is intentionally outside production packages. It compares three
numeric representations on one sparse-domain + strip + exact-arrangement
evaluator and emits machine-readable research evidence only.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import platform
import statistics
import time
import tracemalloc
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import sympy as sp

from cftuv_envelope.reference.arrangement import (
    ExactSegmentArrangementBackend,
)
from cftuv_envelope.reference.common import make_region, make_segment
from cftuv_envelope.reference.contracts import ReachabilityCertificateV1
from cftuv_envelope.reference.planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactScalar,
    PlanarRegion,
    exact_sign,
)
from cftuv_envelope.reference.provenance import make_reference_provenance


MODEL_A = "ALGEBRAIC_ORTHONORMAL_FRAME"
MODEL_B = "RATIONAL_AFFINE_GRAM"
MODEL_C = "CERTIFIED_FLOAT_FILTERED_CANDIDATE"

RUNTIME_RESIDUAL_POLICY = {
    "policy_id": "RUNTIME_PLANAR_RESIDUAL_BUDGET_CANDIDATE_V1",
    "status": "RESEARCH_CANDIDATE_NOT_PRODUCT_POLICY",
    "relative_extent_factor": "1e-7",
    "minimum_extent": "1e-3",
    "coordinate_ulp_multiplier": 64,
    "law": (
        "max(relative_extent_factor * max(planar_extent, minimum_extent), "
        "coordinate_ulp_multiplier * max_coordinate_ulp)"
    ),
}

FILTER_POLICY = {
    "policy_id": "RUNTIME_FILTERED_PREDICATE_BOUND_CANDIDATE_V1",
    "status": "RESEARCH_CANDIDATE_NOT_PRODUCT_POLICY",
    "orientation_error_multiplier": 64,
    "law": (
        "abs(orient2d) > 64 * machine_epsilon * "
        "max_coordinate_delta_squared; otherwise exact fallback"
    ),
}


class AdmissionRejected(ValueError):
    pass


Vec2 = tuple[sp.Expr, sp.Expr]
Vec3 = tuple[sp.Expr, sp.Expr, sp.Expr]


@dataclass(frozen=True)
class Fixture:
    name: str
    semantic_family: str
    points: tuple[Vec3, ...]
    outer: tuple[int, ...]
    triangles: tuple[tuple[int, int, int], ...]
    alpha: sp.Expr
    characteristic_scale: sp.Expr
    expected_admission: bool
    relation: str | None = None
    source_payload: dict | None = None


@dataclass(frozen=True)
class Chart:
    model_id: str
    origin: Vec3
    basis_a: Vec3
    basis_b: Vec3
    gram: tuple[tuple[sp.Expr, sp.Expr], tuple[sp.Expr, sp.Expr]]
    coordinates: tuple[Vec2, ...]
    admission_payload: dict

    def reconstruct(self, point: Vec2) -> Vec3:
        return tuple(
            sp.factor(
                self.origin[index]
                + point[0] * self.basis_a[index]
                + point[1] * self.basis_b[index]
            )
            for index in range(3)
        )


@dataclass(frozen=True)
class CompiledInputs:
    domain: PlanarRegion
    contribution: PlanarRegion
    chart_normal: Vec2
    expressions: tuple[sp.Expr, ...]
    input_segments: tuple


def _q(value: object) -> sp.Expr:
    if isinstance(value, sp.Expr):
        return value
    if isinstance(value, float):
        return sp.Rational(str(value))
    return sp.Rational(value)


def _v3(values) -> Vec3:
    return tuple(_q(item) for item in values)


def _add3(left: Vec3, right: Vec3) -> Vec3:
    return tuple(sp.factor(a + b) for a, b in zip(left, right, strict=True))


def _sub3(left: Vec3, right: Vec3) -> Vec3:
    return tuple(sp.factor(a - b) for a, b in zip(left, right, strict=True))


def _scale3(vector: Vec3, scalar: sp.Expr) -> Vec3:
    return tuple(sp.factor(item * scalar) for item in vector)


def _dot3(left: Vec3, right: Vec3) -> sp.Expr:
    return sp.factor(sum(a * b for a, b in zip(left, right, strict=True)))


def _cross3(left: Vec3, right: Vec3) -> Vec3:
    return (
        sp.factor(left[1] * right[2] - left[2] * right[1]),
        sp.factor(left[2] * right[0] - left[0] * right[2]),
        sp.factor(left[0] * right[1] - left[1] * right[0]),
    )


def _dot2_metric(
    left: Vec2,
    right: Vec2,
    gram: tuple[tuple[sp.Expr, sp.Expr], tuple[sp.Expr, sp.Expr]],
) -> sp.Expr:
    return sp.factor(
        left[0] * (gram[0][0] * right[0] + gram[0][1] * right[1])
        + left[1] * (gram[1][0] * right[0] + gram[1][1] * right[1])
    )


def _sub2(left: Vec2, right: Vec2) -> Vec2:
    return sp.factor(left[0] - right[0]), sp.factor(left[1] - right[1])


def _add2(left: Vec2, right: Vec2) -> Vec2:
    return sp.factor(left[0] + right[0]), sp.factor(left[1] + right[1])


def _scale2(vector: Vec2, scalar: sp.Expr) -> Vec2:
    return sp.factor(vector[0] * scalar), sp.factor(vector[1] * scalar)


def _raw_plane_basis(fixture: Fixture) -> tuple[Vec3, Vec3, Vec3]:
    origin = fixture.points[fixture.outer[0]]
    first = None
    second = None
    for index in fixture.outer[1:]:
        candidate = _sub3(fixture.points[index], origin)
        if _dot3(candidate, candidate) == 0:
            continue
        if first is None:
            first = candidate
            continue
        normal = _cross3(first, candidate)
        if _dot3(normal, normal) != 0:
            second = candidate
            return first, second, normal
    raise AdmissionRejected("source vertices do not define a plane")


def _require_exact_coplanarity(
    fixture: Fixture,
    origin: Vec3,
    raw_normal: Vec3,
) -> None:
    for point in fixture.points:
        if sp.factor(_dot3(_sub3(point, origin), raw_normal)) != 0:
            raise AdmissionRejected(
                "ALL_SOURCE_VERTICES_ON_EXACT_PLANE_FAILED"
            )


def _build_algebraic_orthonormal(fixture: Fixture) -> Chart:
    origin = fixture.points[fixture.outer[0]]
    raw_u, _, raw_normal = _raw_plane_basis(fixture)
    _require_exact_coplanarity(fixture, origin, raw_normal)
    u = _scale3(raw_u, 1 / sp.sqrt(_dot3(raw_u, raw_u)))
    normal = _scale3(
        raw_normal,
        1 / sp.sqrt(_dot3(raw_normal, raw_normal)),
    )
    v = _cross3(normal, u)
    coordinates = tuple(
        (
            sp.factor(_dot3(_sub3(point, origin), u)),
            sp.factor(_dot3(_sub3(point, origin), v)),
        )
        for point in fixture.points
    )
    return Chart(
        MODEL_A,
        origin,
        u,
        v,
        ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1))),
        coordinates,
        {
            "exact": True,
            "normal_length_expression": sp.srepr(
                sp.sqrt(_dot3(raw_normal, raw_normal))
            ),
            "domain_coordinates_all_rational": all(
                value.is_Rational is True
                for coordinate in coordinates
                for value in coordinate
            ),
        },
    )


def _build_rational_affine_gram(fixture: Fixture) -> Chart:
    origin = fixture.points[fixture.outer[0]]
    basis_a, basis_b, raw_normal = _raw_plane_basis(fixture)
    _require_exact_coplanarity(fixture, origin, raw_normal)
    g00 = _dot3(basis_a, basis_a)
    g01 = _dot3(basis_a, basis_b)
    g11 = _dot3(basis_b, basis_b)
    determinant = sp.factor(g00 * g11 - g01 * g01)
    if exact_sign(determinant) <= 0:
        raise AdmissionRejected("GRAM_MATRIX_NOT_POSITIVE_DEFINITE")
    coordinates = []
    for point in fixture.points:
        relative = _sub3(point, origin)
        rhs_a = _dot3(basis_a, relative)
        rhs_b = _dot3(basis_b, relative)
        coordinate = (
            sp.factor((g11 * rhs_a - g01 * rhs_b) / determinant),
            sp.factor((-g01 * rhs_a + g00 * rhs_b) / determinant),
        )
        if _sub3(
            _add3(
                origin,
                _add3(
                    _scale3(basis_a, coordinate[0]),
                    _scale3(basis_b, coordinate[1]),
                ),
            ),
            point,
        ) != (0, 0, 0):
            raise AdmissionRejected("AFFINE_RECONSTRUCTION_FAILED")
        coordinates.append(coordinate)
    return Chart(
        MODEL_B,
        origin,
        basis_a,
        basis_b,
        ((g00, g01), (g01, g11)),
        tuple(coordinates),
        {
            "exact": True,
            "gram_determinant": sp.srepr(determinant),
            "domain_coordinates_all_rational": all(
                value.is_Rational is True
                for coordinate in coordinates
                for value in coordinate
            ),
        },
    )


def _runtime_budget(extent: float, points: tuple[Vec3, ...]) -> tuple[float, float]:
    maximum = max(
        (abs(float(value)) for point in points for value in point),
        default=0.0,
    )
    coordinate_ulp = math.ulp(maximum) if maximum else math.ulp(1.0)
    relative = float(RUNTIME_RESIDUAL_POLICY["relative_extent_factor"])
    minimum_extent = float(RUNTIME_RESIDUAL_POLICY["minimum_extent"])
    budget = max(
        relative * max(extent, minimum_extent),
        int(RUNTIME_RESIDUAL_POLICY["coordinate_ulp_multiplier"])
        * coordinate_ulp,
    )
    return budget, coordinate_ulp


def _build_certified_float(fixture: Fixture) -> Chart:
    exact_origin = fixture.points[fixture.outer[0]]
    raw_u, _, raw_normal = _raw_plane_basis(fixture)
    origin = tuple(float(item) for item in exact_origin)
    raw_u_float = tuple(float(item) for item in raw_u)
    raw_normal_float = tuple(float(item) for item in raw_normal)

    def normalize(vector):
        length = math.sqrt(sum(item * item for item in vector))
        if not math.isfinite(length) or length == 0:
            raise AdmissionRejected("FLOAT_FRAME_DEGENERATE")
        return tuple(item / length for item in vector)

    u = normalize(raw_u_float)
    normal = normalize(raw_normal_float)
    v = (
        normal[1] * u[2] - normal[2] * u[1],
        normal[2] * u[0] - normal[0] * u[2],
        normal[0] * u[1] - normal[1] * u[0],
    )
    float_coordinates = []
    residuals = []
    for point in fixture.points:
        relative = tuple(
            float(value) - origin[index]
            for index, value in enumerate(point)
        )
        float_coordinates.append(
            (
                sum(relative[index] * u[index] for index in range(3)),
                sum(relative[index] * v[index] for index in range(3)),
            )
        )
        residuals.append(
            abs(sum(relative[index] * normal[index] for index in range(3)))
        )
    extent = max(
        max(item[0] for item in float_coordinates)
        - min(item[0] for item in float_coordinates),
        max(item[1] for item in float_coordinates)
        - min(item[1] for item in float_coordinates),
    )
    budget, coordinate_ulp = _runtime_budget(extent, fixture.points)
    max_residual = max(residuals)
    if max_residual > budget:
        raise AdmissionRejected(
            "RUNTIME_PLANAR_RESIDUAL_BUDGET_EXCEEDED:"
            f"residual={max_residual!r}:budget={budget!r}"
        )
    rational_origin = tuple(_q(item) for item in origin)
    rational_u = tuple(_q(item) for item in u)
    rational_v = tuple(_q(item) for item in v)
    coordinates = tuple(
        (_q(item[0]), _q(item[1])) for item in float_coordinates
    )
    return Chart(
        MODEL_C,
        rational_origin,
        rational_u,
        rational_v,
        ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1))),
        coordinates,
        {
            "exact": False,
            "residual_policy_id": RUNTIME_RESIDUAL_POLICY["policy_id"],
            "max_absolute_plane_residual": repr(max_residual),
            "admission_budget": repr(budget),
            "max_coordinate_ulp": repr(coordinate_ulp),
            "domain_coordinates_all_rational": True,
            "input_rounding": "BINARY64_VALUES_RATIONALIZED_FOR_EXACT_FALLBACK",
        },
    )


MODEL_BUILDERS: dict[str, Callable[[Fixture], Chart]] = {
    MODEL_A: _build_algebraic_orthonormal,
    MODEL_B: _build_rational_affine_gram,
    MODEL_C: _build_certified_float,
}


def _certificate(family: str, ordinal: str) -> ConstructionCertificate:
    return ConstructionCertificate(
        ConstructionKind.SOURCE_VERTEX,
        source_vertex_ids=frozenset({f"vertex:{family}:{ordinal}"}),
    )


def _make_loop_region(
    family: str,
    role: str,
    points: tuple[Vec2, ...],
    *,
    contribution: bool,
) -> PlanarRegion:
    planar_points = tuple(
        ExactPlanarPoint.from_values(*point) for point in points
    )
    provenance = make_reference_provenance(
        envelope_spec_ids=(
            frozenset({f"spec:{family}"}) if contribution else frozenset()
        ),
        envelope_instance_ids=(
            frozenset({f"instance:{family}"})
            if contribution
            else frozenset()
        ),
        support_ids=frozenset({f"support:{family}:{role}"}),
        physical_edge_ids=frozenset(
            {
                f"edge:{family}:{role}:{index}"
                for index in range(len(points))
            }
        ),
        chain_use_ids=frozenset({f"use:{family}:source"}),
        patch_domain_ids=frozenset({f"domain:{family}"}),
        boundary_constraint_ids=(
            frozenset({f"constraint:{family}:outer"})
            if not contribution
            else frozenset()
        ),
    )
    certificates = tuple(
        _certificate(family, f"{role}:{index}")
        for index in range(len(points))
    )
    segments = tuple(
        make_segment(
            f"segment:{family}:{role}:{index}",
            planar_points[index],
            planar_points[(index + 1) % len(points)],
            support_ids=provenance.support_ids,
            provenance=provenance,
            start_certificates=frozenset({certificates[index]}),
            end_certificates=frozenset(
                {certificates[(index + 1) % len(points)]}
            ),
            boundary_constraint_ids=provenance.boundary_constraint_ids,
        )
        for index in range(len(points))
    )
    region = make_region(
        f"region:{family}:{role}",
        segments,
        spec_id=f"spec:{family}" if contribution else "",
        instance_id=f"instance:{family}" if contribution else "",
    )
    if region is None:
        raise AdmissionRejected(f"{role} has zero chart area")
    if not contribution:
        region = PlanarRegion(region.region_id, region.outer)
    return region


def _compile_inputs(fixture: Fixture, chart: Chart) -> CompiledInputs:
    outer = tuple(chart.coordinates[index] for index in fixture.outer)
    domain = _make_loop_region(
        fixture.semantic_family,
        "domain",
        outer,
        contribution=False,
    )
    source_start = outer[0]
    source_end = outer[1]
    tangent = _sub2(source_end, source_start)
    gram = chart.gram
    row_x = sp.factor(tangent[0] * gram[0][0] + tangent[1] * gram[1][0])
    row_y = sp.factor(tangent[0] * gram[0][1] + tangent[1] * gram[1][1])
    raw_normal = (-row_y, row_x)
    normal_length_squared = _dot2_metric(raw_normal, raw_normal, gram)
    if exact_sign(normal_length_squared) <= 0:
        raise AdmissionRejected("SOURCE_SUPPORT_METRIC_NORMAL_DEGENERATE")
    normal = _scale2(raw_normal, 1 / sp.sqrt(normal_length_squared))
    moved_start = _add2(source_start, _scale2(normal, fixture.alpha))
    moved_end = _add2(source_end, _scale2(normal, fixture.alpha))
    contribution = _make_loop_region(
        fixture.semantic_family,
        "strip",
        (source_start, source_end, moved_end, moved_start),
        contribution=True,
    )
    expressions = (
        *(value for point in chart.coordinates for value in point),
        *(value for row in gram for value in row),
        *normal,
        *moved_start,
        *moved_end,
    )
    return CompiledInputs(
        domain,
        contribution,
        normal,
        tuple(expressions),
        tuple(
            segment
            for region in (domain, contribution)
            for segment in region.outer.segments
        ),
    )


def _evaluate_raw(compiled: CompiledInputs):
    reachability = {
        next(iter(compiled.contribution.contributor_instance_ids)): (
            ReachabilityCertificateV1(
                frozenset({"front:metric-spike"}),
                True,
                1,
                1,
                (),
                False,
            )
        )
    }
    return ExactSegmentArrangementBackend().exact_union(
        (compiled.contribution,),
        (compiled.domain,),
        reachability,
    )


def _canonical_expr(value: sp.Expr) -> str:
    return sp.srepr(sp.factor(sp.cancel(value)))


def _expression_metrics(
    chart: Chart,
    compiled: CompiledInputs,
    raw,
) -> dict:
    expressions = [
        *chart.origin,
        *chart.basis_a,
        *chart.basis_b,
        *(value for row in chart.gram for value in row),
        *compiled.expressions,
        *(
            coordinate
            for vertex in raw.vertices
            for coordinate in vertex.point.expressions()
        ),
        sp.sympify(raw.exact_area_expression),
        *(sp.sympify(loop.signed_area) for loop in raw.loops),
    ]
    nodes = [
        sum(1 for _ in sp.preorder_traversal(expression))
        for expression in expressions
    ]
    lengths = [len(sp.srepr(expression)) for expression in expressions]
    return {
        "expression_count": len(expressions),
        "expression_node_count": sum(nodes),
        "peak_expression_node_count": max(nodes, default=0),
        "peak_expression_length": max(lengths, default=0),
        "total_expression_length": sum(lengths),
    }


def _angle_certificate_metrics(chart: Chart, fixture: Fixture) -> dict:
    outer = tuple(chart.coordinates[index] for index in fixture.outer)
    incoming = _sub2(outer[1], outer[0])
    outgoing = _sub2(outer[2], outer[1])
    dot_value = _dot2_metric(incoming, outgoing, chart.gram)
    incoming_length = sp.sqrt(
        _dot2_metric(incoming, incoming, chart.gram)
    )
    outgoing_length = sp.sqrt(
        _dot2_metric(outgoing, outgoing, chart.gram)
    )
    cosine = sp.factor(dot_value / (incoming_length * outgoing_length))
    determinant = sp.factor(
        incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    )
    gram_determinant = sp.factor(
        chart.gram[0][0] * chart.gram[1][1]
        - chart.gram[0][1] * chart.gram[1][0]
    )
    oriented_area = sp.factor(determinant * sp.sqrt(gram_determinant))
    expressions = (dot_value, cosine, oriented_area)
    return {
        "expression_node_count": sum(
            sum(1 for _ in sp.preorder_traversal(item))
            for item in expressions
        ),
        "peak_expression_length": max(
            len(sp.srepr(item)) for item in expressions
        ),
        "cosine_expression": sp.srepr(cosine),
        "orientation_sign": exact_sign(oriented_area),
    }


def _topology_projection(raw) -> dict:
    return {
        "edges": sorted(
            (
                edge.edge_id,
                edge.start_vertex_id,
                edge.end_vertex_id,
                sorted(edge.provenance.support_ids),
                sorted(edge.provenance.physical_edge_ids),
            )
            for edge in raw.edges
        ),
        "loops": sorted(
            (
                loop.loop_id,
                loop.kind.value,
                list(loop.ordered_edge_ids),
            )
            for loop in raw.loops
        ),
        "regions": sorted(
            (
                region.region_id,
                region.outer_loop_id,
                list(region.hole_loop_ids),
                sorted(region.contributor_envelope_spec_ids),
            )
            for region in raw.regions
        ),
    }


def _physical_projection(
    chart: Chart,
    raw,
    characteristic_scale: sp.Expr,
) -> tuple[dict, dict]:
    physical_vertices = {
        vertex.vertex_id: chart.reconstruct(vertex.point.expressions())
        for vertex in raw.vertices
    }
    gram_determinant = sp.factor(
        chart.gram[0][0] * chart.gram[1][1]
        - chart.gram[0][1] * chart.gram[1][0]
    )
    physical_area = sp.factor(
        sp.sympify(raw.exact_area_expression) * sp.sqrt(gram_determinant)
    )
    exact_projection = {
        "topology": _topology_projection(raw),
        "vertices": {
            vertex_id: [_canonical_expr(value) for value in point]
            for vertex_id, point in sorted(physical_vertices.items())
        },
        "physical_area": _canonical_expr(physical_area),
    }
    pairwise = []
    ordered = sorted(physical_vertices.items())
    for left_index, (_, left) in enumerate(ordered):
        for _, right in ordered[left_index + 1 :]:
            pairwise.append(
                _canonical_expr(
                    sp.factor(
                        _dot3(_sub3(left, right), _sub3(left, right))
                        / (characteristic_scale * characteristic_scale)
                    )
                )
            )
    invariant_projection = {
        "topology": _topology_projection(raw),
        "normalized_pairwise_squared_distances": sorted(pairwise),
        "normalized_physical_area": _canonical_expr(
            physical_area / (characteristic_scale * characteristic_scale)
        ),
    }
    return exact_projection, invariant_projection


def _digest(payload: dict) -> str:
    return hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()


def _exact_sign_cost(expressions: tuple[sp.Expr, ...]) -> float:
    samples = []
    candidates = tuple(
        expression
        for expression in expressions
        if expression != 0
    )[:32]
    if not candidates:
        return 0.0
    for _ in range(5):
        started = time.perf_counter()
        for expression in candidates:
            exact_sign(expression)
        samples.append(time.perf_counter() - started)
    return statistics.median(samples)


def _filtered_predicate_receipt(compiled: CompiledInputs) -> dict:
    segments = compiled.input_segments
    epsilon = math.ulp(1.0)
    total = 0
    fallbacks = 0
    candidate_pairs = 0
    for left_index, left in enumerate(segments):
        lx0, ly0 = (float(item) for item in left.start.expressions())
        lx1, ly1 = (float(item) for item in left.end.expressions())
        for right in segments[left_index + 1 :]:
            rx0, ry0 = (float(item) for item in right.start.expressions())
            rx1, ry1 = (float(item) for item in right.end.expressions())
            if (
                max(lx0, lx1) < min(rx0, rx1)
                or max(rx0, rx1) < min(lx0, lx1)
                or max(ly0, ly1) < min(ry0, ry1)
                or max(ry0, ry1) < min(ly0, ly1)
            ):
                continue
            candidate_pairs += 1
            for px, py in ((rx0, ry0), (rx1, ry1)):
                total += 1
                ax = lx1 - lx0
                ay = ly1 - ly0
                bx = px - lx0
                by = py - ly0
                determinant = ax * by - ay * bx
                scale = max(
                    abs(ax),
                    abs(ay),
                    abs(bx),
                    abs(by),
                    1.0,
                )
                bound = (
                    int(FILTER_POLICY["orientation_error_multiplier"])
                    * epsilon
                    * scale
                    * scale
                )
                if abs(determinant) <= bound:
                    fallbacks += 1
    return {
        "policy_id": FILTER_POLICY["policy_id"],
        "candidate_pairs": candidate_pairs,
        "orientation_predicates": total,
        "exact_fallbacks": fallbacks,
        "exact_fallback_fraction": (
            f"{fallbacks}/{total}" if total else "0/0"
        ),
    }


def _evaluate_once(fixture: Fixture, model_id: str):
    frame_started = time.perf_counter()
    chart = MODEL_BUILDERS[model_id](fixture)
    frame_time = time.perf_counter() - frame_started

    compile_started = time.perf_counter()
    compiled = _compile_inputs(fixture, chart)
    compile_time = time.perf_counter() - compile_started

    raw_started = time.perf_counter()
    raw = _evaluate_raw(compiled)
    raw_time = time.perf_counter() - raw_started
    return chart, compiled, raw, frame_time, compile_time, raw_time


def _run_model(
    fixture: Fixture,
    model_id: str,
    iterations: int,
) -> dict:
    try:
        first = _evaluate_once(fixture, model_id)
    except AdmissionRejected as exc:
        return {
            "model": model_id,
            "admission_success": False,
            "admission_outcome": str(exc),
            "expected_admission": fixture.expected_admission,
        }
    chart, compiled, raw, first_frame, first_compile, first_raw = first
    timing_samples = [(first_frame, first_compile, first_raw)]
    semantic_projection, invariant_projection = _physical_projection(
        chart,
        raw,
        fixture.characteristic_scale,
    )
    semantic_digest = _digest(semantic_projection)
    invariant_digest = _digest(invariant_projection)
    digest_stable = True
    for _ in range(max(1, iterations - 1)):
        repeated = _evaluate_once(fixture, model_id)
        timing_samples.append(repeated[3:])
        repeated_projection, repeated_invariant = _physical_projection(
            repeated[0],
            repeated[2],
            fixture.characteristic_scale,
        )
        digest_stable = digest_stable and (
            _digest(repeated_projection) == semantic_digest
            and _digest(repeated_invariant) == invariant_digest
        )

    tracemalloc.start()
    _evaluate_once(fixture, model_id)
    _, peak_memory = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    expression_metrics = _expression_metrics(chart, compiled, raw)
    angle_metrics = _angle_certificate_metrics(chart, fixture)
    sign_expressions = tuple(
        (
            left.end.x.as_expr() - left.start.x.as_expr()
        )
        * (
            right.end.y.as_expr() - right.start.y.as_expr()
        )
        - (
            left.end.y.as_expr() - left.start.y.as_expr()
        )
        * (
            right.end.x.as_expr() - right.start.x.as_expr()
        )
        for left_index, left in enumerate(compiled.input_segments)
        for right in compiled.input_segments[left_index + 1 :]
    )
    return {
        "model": model_id,
        "admission_success": True,
        "admission_outcome": "ADMITTED",
        "expected_admission": fixture.expected_admission,
        "frame_build_seconds_median": statistics.median(
            item[0] for item in timing_samples
        ),
        "compile_seconds_median": statistics.median(
            item[1] for item in timing_samples
        ),
        "raw_coverage_seconds_median": statistics.median(
            item[2] for item in timing_samples
        ),
        "peak_tracemalloc_bytes": peak_memory,
        "exact_sign_batch_seconds_median": _exact_sign_cost(
            sign_expressions
        ),
        "expression_metrics": expression_metrics,
        "angle_certificate_complexity": angle_metrics,
        "chart": chart.admission_payload,
        "arrangement_counters": {
            "input_segments": raw.input_segment_count,
            "all_possible_pairs": raw.all_possible_pair_count,
            "broadphase_candidate_pairs": (
                raw.broadphase_candidate_pair_count
            ),
            "narrowphase_tests": raw.narrowphase_test_count,
            "actual_intersections": raw.intersection_count,
            "atomic_edges": raw.atomic_edge_count,
        },
        "semantic_digest": semantic_digest,
        "invariant_digest": invariant_digest,
        "digest_stable_across_repeats": digest_stable,
        "topology_projection": _topology_projection(raw),
        "physical_vertex_expressions": semantic_projection["vertices"],
        "physical_area_expression": semantic_projection["physical_area"],
        "filtered_predicates": (
            _filtered_predicate_receipt(compiled)
            if model_id == MODEL_C
            else None
        ),
    }


def _float_deviation(reference: dict, runtime: dict) -> dict:
    semantic_topology_equal = (
        reference.get("topology_projection")
        == runtime.get("topology_projection")
    )
    reference_vertices = reference.get("physical_vertex_expressions", {})
    runtime_vertices = runtime.get("physical_vertex_expressions", {})
    reference_points = [
        tuple(float(sp.sympify(item)) for item in point)
        for point in reference_vertices.values()
    ]
    runtime_points = [
        tuple(float(sp.sympify(item)) for item in point)
        for point in runtime_vertices.values()
    ]
    unlabeled_deviation = None
    if len(reference_points) == len(runtime_points) and len(reference_points) <= 8:
        unlabeled_deviation = min(
            max(
                math.sqrt(
                    sum(
                        (left[axis] - right[axis]) ** 2
                        for axis in range(3)
                    )
                )
                for left, right in zip(
                    reference_points,
                    permutation,
                    strict=True,
                )
            )
            for permutation in itertools.permutations(runtime_points)
        )
    combinatorial_topology_equal = (
        len(reference.get("topology_projection", {}).get("edges", ()))
        == len(runtime.get("topology_projection", {}).get("edges", ()))
        and len(reference.get("topology_projection", {}).get("loops", ()))
        == len(runtime.get("topology_projection", {}).get("loops", ()))
        and len(reference.get("topology_projection", {}).get("regions", ()))
        == len(runtime.get("topology_projection", {}).get("regions", ()))
    )
    if set(reference_vertices) != set(runtime_vertices):
        return {
            "semantic_topology_and_provenance_equal": (
                semantic_topology_equal
            ),
            "combinatorial_topology_equal": combinatorial_topology_equal,
            "vertex_identity_equal": False,
            "max_vertex_deviation": None,
            "max_unlabeled_vertex_deviation": unlabeled_deviation,
        }
    deviations = []
    for vertex_id in reference_vertices:
        left = [sp.sympify(item) for item in reference_vertices[vertex_id]]
        right = [sp.sympify(item) for item in runtime_vertices[vertex_id]]
        deviations.append(
            math.sqrt(
                sum(
                    (float(a) - float(b)) ** 2
                    for a, b in zip(left, right, strict=True)
                )
            )
        )
    reference_area = reference.get("physical_area_expression")
    runtime_area = runtime.get("physical_area_expression")
    return {
        "semantic_topology_and_provenance_equal": semantic_topology_equal,
        "combinatorial_topology_equal": combinatorial_topology_equal,
        "vertex_identity_equal": True,
        "max_vertex_deviation": max(deviations, default=0.0),
        "max_unlabeled_vertex_deviation": unlabeled_deviation,
        "absolute_area_deviation": abs(
            float(sp.sympify(reference_area))
            - float(sp.sympify(runtime_area))
        ),
    }


def _exact_result_equivalent(left: dict, right: dict) -> bool:
    if left.get("topology_projection") != right.get("topology_projection"):
        return False
    left_vertices = left.get("physical_vertex_expressions", {})
    right_vertices = right.get("physical_vertex_expressions", {})
    if set(left_vertices) != set(right_vertices):
        return False
    for vertex_id in left_vertices:
        for left_value, right_value in zip(
            left_vertices[vertex_id],
            right_vertices[vertex_id],
            strict=True,
        ):
            if sp.simplify(
                sp.sympify(left_value) - sp.sympify(right_value)
            ) != 0:
                return False
    return (
        sp.simplify(
            sp.sympify(left["physical_area_expression"])
            - sp.sympify(right["physical_area_expression"])
        )
        == 0
    )


def _fixture_comparisons(model_results: dict[str, dict]) -> dict:
    a = model_results[MODEL_A]
    b = model_results[MODEL_B]
    c = model_results[MODEL_C]
    exact_models_admitted = (
        a["admission_success"] and b["admission_success"]
    )
    exact_semantic_equal = (
        exact_models_admitted
        and _exact_result_equivalent(a, b)
    )
    runtime_deviation = (
        _float_deviation(b, c)
        if b["admission_success"] and c["admission_success"]
        else None
    )
    return {
        "exact_model_semantic_result_equal": exact_semantic_equal,
        "representation_digest_equal": (
            exact_models_admitted
            and a["semantic_digest"] == b["semantic_digest"]
        ),
        "runtime_vs_reference": runtime_deviation,
        "all_admitted_digests_stable": all(
            result.get("digest_stable_across_repeats", True)
            for result in model_results.values()
        ),
    }


def _synthetic_fixtures() -> tuple[Fixture, ...]:
    base = (
        _v3((0, 0, 0)),
        _v3((4, 0, 0)),
        _v3((4, 3, 0)),
        _v3((0, 3, 0)),
    )
    base_triangles = ((0, 1, 2), (0, 2, 3))

    def affine(
        origin: Vec3,
        axis_a: Vec3,
        axis_b: Vec3,
        width: sp.Expr = sp.Integer(4),
        height: sp.Expr = sp.Integer(3),
    ) -> tuple[Vec3, ...]:
        return (
            origin,
            _add3(origin, _scale3(axis_a, width)),
            _add3(
                origin,
                _add3(
                    _scale3(axis_a, width),
                    _scale3(axis_b, height),
                ),
            ),
            _add3(origin, _scale3(axis_b, height)),
        )

    arbitrary = affine(
        _v3((7, -2, 5)),
        _v3((sp.Rational(1, 3), sp.Rational(2, 3), sp.Rational(2, 3))),
        _v3((sp.Rational(2, 3), sp.Rational(-2, 3), sp.Rational(1, 3))),
    )
    forty_five = affine(
        _v3((0, 0, 0)),
        _v3((1, 0, 0)),
        _v3((0, 1, 1)),
    )
    irrational_normal = affine(
        _v3((2, 3, 5)),
        _v3((1, -1, 0)),
        _v3((1, 0, -1)),
    )
    micro_scale = sp.Rational(1, 10**9)
    micro = tuple(_scale3(point, micro_scale) for point in base)
    huge_origin = _v3((10**12, -10**12, 10**12))
    huge = tuple(_add3(huge_origin, point) for point in base)
    nearly = list(base)
    nearly[2] = (
        nearly[2][0],
        nearly[2][1],
        sp.Rational(1, 1000),
    )
    rotation = (
        (sp.Rational(3, 5), sp.Rational(-4, 5), sp.Integer(0)),
        (sp.Rational(4, 5), sp.Rational(3, 5), sp.Integer(0)),
        (sp.Integer(0), sp.Integer(0), sp.Integer(1)),
    )

    def rotate(point: Vec3) -> Vec3:
        return tuple(
            sp.factor(sum(row[index] * point[index] for index in range(3)))
            for row in rotation
        )

    rotated = tuple(rotate(point) for point in base)
    return (
        Fixture(
            "axis_aligned_plane",
            "base-plane",
            base,
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2),
            sp.Integer(1),
            True,
        ),
        Fixture(
            "arbitrary_rotated_plane",
            "arbitrary-plane",
            arbitrary,
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2),
            sp.Integer(1),
            True,
        ),
        Fixture(
            "forty_five_degrees",
            "forty-five-plane",
            forty_five,
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2),
            sp.Integer(1),
            True,
        ),
        Fixture(
            "irrational_normal",
            "irrational-normal-plane",
            irrational_normal,
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2),
            sp.Integer(1),
            True,
        ),
        Fixture(
            "micro_scale",
            "base-plane",
            micro,
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2) * micro_scale,
            micro_scale,
            True,
            relation="SCALE_OF_axis_aligned_plane",
        ),
        Fixture(
            "huge_coordinates",
            "huge-coordinate-plane",
            huge,
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2),
            sp.Integer(1),
            True,
        ),
        Fixture(
            "nearly_planar_outside_budget",
            "nearly-planar-plane",
            tuple(nearly),
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2),
            sp.Integer(1),
            False,
        ),
        Fixture(
            "same_plane_different_triangulation",
            "base-plane",
            base,
            (0, 1, 2, 3),
            ((0, 1, 3), (1, 2, 3)),
            sp.Rational(1, 2),
            sp.Integer(1),
            True,
            relation="RETRIANGULATION_OF_axis_aligned_plane",
        ),
        Fixture(
            "same_plane_after_object_rotation",
            "base-plane",
            rotated,
            (0, 1, 2, 3),
            base_triangles,
            sp.Rational(1, 2),
            sp.Integer(1),
            True,
            relation="RIGID_ROTATION_OF_axis_aligned_plane",
        ),
    )


def _building_fixture(source_path: Path) -> Fixture:
    payload = json.loads(source_path.read_text(encoding="utf-8"))
    # Позволяет повторить spike прямо из committed building receipt, не
    # сохраняя отдельный временный Blender export рядом с артефактами.
    if payload.get("schema") == (
        "cftuv.envelope.planar_metric_building_patch.v1"
    ):
        payload = payload["source"]
    positions = {
        int(item["vertex_id"]): _v3(item["position"])
        for item in payload["vertices"]
    }
    outer_record = next(
        item
        for item in payload["boundary_loops"]
        if item["kind"] == "OUTER"
    )
    ordered_ids = tuple(int(item) for item in outer_record["vertex_ids"])
    points = tuple(positions[item] for item in ordered_ids)
    index_by_id = {
        vertex_id: index for index, vertex_id in enumerate(ordered_ids)
    }
    triangles = tuple(
        tuple(index_by_id[int(item)] for item in triangle["vertex_ids"])
        for triangle in payload["triangles"]
    )
    edge_length_squared = _dot3(
        _sub3(points[1], points[0]),
        _sub3(points[1], points[0]),
    )
    characteristic = sp.sqrt(edge_length_squared)
    return Fixture(
        "building_002_patch_3",
        "building-002-patch-3",
        points,
        tuple(range(len(points))),
        triangles,
        sp.Rational(3, 10),
        characteristic,
        True,
        source_payload=payload,
    )


def _relation_receipts(fixtures: list[dict]) -> dict:
    by_name = {item["fixture"]: item for item in fixtures}
    base = by_name["axis_aligned_plane"]["models"]
    receipts = {}
    for fixture_name, relation_kind in (
        ("micro_scale", "scale"),
        ("same_plane_different_triangulation", "retriangulation"),
        ("same_plane_after_object_rotation", "rotation"),
    ):
        candidate = by_name[fixture_name]["models"]
        receipts[relation_kind] = {
            model_id: (
                base[model_id].get("invariant_digest")
                == candidate[model_id].get("invariant_digest")
            )
            for model_id in MODEL_BUILDERS
            if base[model_id]["admission_success"]
            and candidate[model_id]["admission_success"]
        }
    return receipts


def _aggregate(fixtures: list[dict]) -> dict:
    aggregate = {}
    for model_id in MODEL_BUILDERS:
        admitted = [
            item["models"][model_id]
            for item in fixtures
            if item["models"][model_id]["admission_success"]
        ]
        aggregate[model_id] = {
            "admitted_fixture_count": len(admitted),
            "rejected_fixture_count": len(fixtures) - len(admitted),
            "frame_build_seconds_median": statistics.median(
                item["frame_build_seconds_median"] for item in admitted
            ),
            "compile_seconds_median": statistics.median(
                item["compile_seconds_median"] for item in admitted
            ),
            "raw_coverage_seconds_median": statistics.median(
                item["raw_coverage_seconds_median"] for item in admitted
            ),
            "expression_node_count_median": statistics.median(
                item["expression_metrics"]["expression_node_count"]
                for item in admitted
            ),
            "peak_expression_length_max": max(
                item["expression_metrics"]["peak_expression_length"]
                for item in admitted
            ),
            "peak_tracemalloc_bytes_max": max(
                item["peak_tracemalloc_bytes"] for item in admitted
            ),
        }
    return aggregate


def _run(args) -> int:
    building_source_path = Path(args.building_source)
    fixtures = [
        *_synthetic_fixtures(),
        _building_fixture(building_source_path),
    ]
    fixture_records = []
    for fixture in fixtures:
        models = {
            model_id: _run_model(fixture, model_id, args.iterations)
            for model_id in MODEL_BUILDERS
        }
        fixture_records.append(
            {
                "fixture": fixture.name,
                "semantic_family": fixture.semantic_family,
                "relation": fixture.relation,
                "expected_admission": fixture.expected_admission,
                "point_count": len(fixture.points),
                "triangle_count": len(fixture.triangles),
                "models": models,
                "comparisons": _fixture_comparisons(models),
            }
        )
    relations = _relation_receipts(fixture_records)
    exact_equal = all(
        item["comparisons"]["exact_model_semantic_result_equal"]
        for item in fixture_records
        if item["models"][MODEL_A]["admission_success"]
        and item["models"][MODEL_B]["admission_success"]
    )
    runtime_semantic_equal = all(
        comparison["semantic_topology_and_provenance_equal"]
        for item in fixture_records
        if (
            comparison := item["comparisons"]["runtime_vs_reference"]
        )
        is not None
    )
    runtime_mismatches = [
        item["fixture"]
        for item in fixture_records
        if (
            comparison := item["comparisons"]["runtime_vs_reference"]
        )
        is not None
        and not comparison["semantic_topology_and_provenance_equal"]
    ]
    results = {
        "schema": "cftuv.envelope.planar_metric_spike_results.v1",
        "session": "SESSION_M_R0_PLANAR_METRIC_CONTRACT_DECISION",
        "status": "RESEARCH_ONLY_NO_PRODUCTION_INTEGRATION",
        "iterations_per_admitted_model": args.iterations,
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "sympy": sp.__version__,
        },
        "runtime_residual_policy_candidate": RUNTIME_RESIDUAL_POLICY,
        "filtered_predicate_policy_candidate": FILTER_POLICY,
        "fixtures": fixture_records,
        "aggregate": _aggregate(fixture_records),
        "metamorphic_relations": relations,
        "gate": {
            "exact_a_b_semantic_equivalence": exact_equal,
            "runtime_semantic_match_fixture_count": (
                sum(
                    1
                    for item in fixture_records
                    if (
                        comparison := item["comparisons"][
                            "runtime_vs_reference"
                        ]
                    )
                    is not None
                    and comparison[
                        "semantic_topology_and_provenance_equal"
                    ]
                )
            ),
            "runtime_semantic_mismatch_fixtures": runtime_mismatches,
            "runtime_candidate_status": (
                "STOPPED_NOT_READY_FOR_PRODUCTION"
                if runtime_mismatches
                else "ELIGIBLE_FOR_PRODUCT_POLICY_REVIEW"
            ),
            "all_digests_stable": all(
                item["comparisons"]["all_admitted_digests_stable"]
                for item in fixture_records
            ),
            "production_contracts_changed": False,
        },
    }
    results_path = Path(args.results)
    results_path.parent.mkdir(parents=True, exist_ok=True)
    results_path.write_text(
        json.dumps(results, indent=2, sort_keys=True),
        encoding="utf-8",
        newline="\n",
    )
    building_record = next(
        item
        for item in fixture_records
        if item["fixture"] == "building_002_patch_3"
    )
    building_payload = {
        "schema": "cftuv.envelope.planar_metric_building_patch.v1",
        "session": results["session"],
        "status": results["status"],
        "source": fixtures[-1].source_payload,
        "fixture_result": building_record,
        "runtime_residual_policy_candidate": RUNTIME_RESIDUAL_POLICY,
        "filtered_predicate_policy_candidate": FILTER_POLICY,
    }
    building_path = Path(args.building_result)
    building_path.parent.mkdir(parents=True, exist_ok=True)
    building_path.write_text(
        json.dumps(building_payload, indent=2, sort_keys=True),
        encoding="utf-8",
        newline="\n",
    )
    print(
        json.dumps(
            {
                "gate": results["gate"],
                "aggregate": results["aggregate"],
                "metamorphic_relations": relations,
                "building_patch_3": building_record,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if (
        exact_equal
        and results["gate"]["all_digests_stable"]
        and len(fixture_records) == 10
    ) else 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--building-source", required=True)
    parser.add_argument("--results", required=True)
    parser.add_argument("--building-result", required=True)
    parser.add_argument("--iterations", type=int, default=5)
    return parser


def main() -> int:
    return _run(_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
