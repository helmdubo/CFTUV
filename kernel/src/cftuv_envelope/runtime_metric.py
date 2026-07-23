"""Certified binary64 preflight with authoritative exact construction."""

from __future__ import annotations

from dataclasses import dataclass
import time
import tracemalloc

from .contracts.metric import (
    RationalAffinePlanarMetricV2,
    RuntimePlanarMetricV1,
)
from .numeric import LocalLengthV1
from .planar_metric import (
    RuntimePredicateTelemetryV1,
    build_runtime_planar_metric,
    resolve_orient2d,
)
from .reference.contracts import (
    RawCoverageResultV1,
    ReferenceEnvelopeCompilationV1,
)
from .reference.raw_coverage import evaluate_reference_raw_coverage


@dataclass(frozen=True, slots=True)
class RuntimeMetricPerformanceReportV1:
    filtered_predicate_count: int
    fast_path_certified_count: int
    exact_fallback_count: int
    fallback_fraction: str
    reference_metric_build_seconds: float
    runtime_view_build_seconds: float
    raw_coverage_seconds: float
    expression_size: int
    peak_memory_bytes: int


@dataclass(frozen=True, slots=True)
class RuntimeRawCoverageEvaluationV1:
    runtime_metric: RuntimePlanarMetricV1
    raw_result: RawCoverageResultV1
    performance: RuntimeMetricPerformanceReportV1


def _reference_metric(
    compilation: ReferenceEnvelopeCompilationV1,
) -> RationalAffinePlanarMetricV2:
    matches = tuple(
        item
        for item in compilation.analysis_snapshot.surface_metric_descriptors
        if isinstance(item, RationalAffinePlanarMetricV2)
        and item.patch_domain_id == compilation.plan_key.patch_domain_id
    )
    if len(matches) != 1:
        raise ValueError(
            "filtered runtime requires one authoritative "
            "RationalAffinePlanarMetricV2"
        )
    return matches[0]


def _run_source_predicates(
    compilation: ReferenceEnvelopeCompilationV1,
    reference_metric: RationalAffinePlanarMetricV2,
    runtime_metric: RuntimePlanarMetricV1,
    telemetry: RuntimePredicateTelemetryV1,
) -> None:
    owner_patch_id = compilation.owner_patch_id
    for face in sorted(
        (
            item
            for item in compilation.analysis_snapshot.surface_ir.source_faces
            if item.patch_id == owner_patch_id
        ),
        key=lambda item: item.face_id.value,
    ):
        if len(face.vertex_cycle) < 3:
            continue
        anchor = face.vertex_cycle[0]
        for ordinal in range(1, len(face.vertex_cycle) - 1):
            resolve_orient2d(
                reference_metric=reference_metric,
                runtime_metric=runtime_metric,
                source_vertex_ids=(
                    anchor,
                    face.vertex_cycle[ordinal],
                    face.vertex_cycle[ordinal + 1],
                ),
                telemetry=telemetry,
            )


def evaluate_filtered_runtime_raw_coverage(
    compilation: ReferenceEnvelopeCompilationV1,
    requested_alpha: LocalLengthV1,
    *,
    runtime_metric: RuntimePlanarMetricV1 | None = None,
    reference_metric_build_seconds: float = 0.0,
) -> RuntimeRawCoverageEvaluationV1:
    """Evaluate with certified candidates and exact semantic construction.

    Binary64 predicates are only a filter.  RawCoverage records, construction
    certificates, IDs and coordinates are emitted by the authoritative exact
    evaluator over the referenced V2 metric.
    """

    reference_metric = _reference_metric(compilation)
    started = time.perf_counter()
    runtime_metric = runtime_metric or build_runtime_planar_metric(
        reference_metric
    )
    runtime_view_seconds = time.perf_counter() - started
    if runtime_metric.reference_metric_id != reference_metric.reference_metric_id:
        raise ValueError("runtime view references another exact metric")

    telemetry = RuntimePredicateTelemetryV1()
    _run_source_predicates(
        compilation,
        reference_metric,
        runtime_metric,
        telemetry,
    )

    tracemalloc.start()
    started = time.perf_counter()
    raw_result = evaluate_reference_raw_coverage(
        compilation, requested_alpha
    )
    raw_seconds = time.perf_counter() - started
    _, peak_memory = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    expressions = []
    if raw_result.raw_coverage is not None:
        expressions.extend(
            (
                vertex.point.x.expression
                for vertex in raw_result.raw_coverage.vertices
            )
        )
        expressions.extend(
            (
                vertex.point.y.expression
                for vertex in raw_result.raw_coverage.vertices
            )
        )
    expression_size = max((len(item) for item in expressions), default=0)
    return RuntimeRawCoverageEvaluationV1(
        runtime_metric=runtime_metric,
        raw_result=raw_result,
        performance=RuntimeMetricPerformanceReportV1(
            filtered_predicate_count=telemetry.filtered_predicate_count,
            fast_path_certified_count=telemetry.fast_path_certified_count,
            exact_fallback_count=telemetry.exact_fallback_count,
            fallback_fraction=telemetry.fallback_fraction,
            reference_metric_build_seconds=max(
                0.0, float(reference_metric_build_seconds)
            ),
            runtime_view_build_seconds=runtime_view_seconds,
            raw_coverage_seconds=raw_seconds,
            expression_size=expression_size,
            peak_memory_bytes=peak_memory,
        ),
    )
