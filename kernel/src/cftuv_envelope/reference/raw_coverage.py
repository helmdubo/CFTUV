"""Full-recompute EnvelopeInstances -> boundary resolution -> exact RawCoverage."""

from __future__ import annotations

import time
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
from typing import Callable

import sympy as sp

from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    CapEnvelopeSpec,
    JunctionEnvelopeSpec,
    StripEnvelopeSpec,
)
from ..numeric import LocalLengthV1, MetricSpace
from .angular import evaluate_angular_envelope
from .arrangement import (
    ExactArrangementCollinearBranchUnproven,
    ExactArrangementRotationSystemUnproven,
    ExactSegmentArrangementBackend,
    ExactTouchingHoleTopologyUnproven,
    bound_evaluation_arrangement,
    segment_intersections,
)
from .boundary import (
    BoundaryRole,
    build_domain_geometry,
    resolve_component_alphas,
)
from .cap import evaluate_cap_envelope
from .common import GeometryContext, ReferenceGeometryError, make_segment
from .contracts import (
    BoundaryResolvedEnvelopeV1,
    RAW_COVERAGE_RESULT_SCHEMA_V2,
    RawCoverageResultV2,
    ReachabilityCertificateV1,
    ReferenceDiagnosticSeverity,
    ReferenceEnvelopeCompilationV1,
    ReferenceEnvelopeInstanceV1,
    ReferenceEvaluationDiagnosticV1,
    ReferenceEvaluationResultV1,
    ReferenceOutcome,
)
from .digest import raw_coverage_semantic_digest
from .junction import evaluate_junction_envelope
from .planar_types import (
    CertifiedPredicateUndecidable,
    ExactScalar,
    PlanarLoop,
    PlanarRegion,
    exact_sign,
    polygon_signed_area,
)
from .strip import evaluate_strip_envelope
from .validation import validate_compilation_geometry_payload


REFERENCE_ARRANGEMENT_BACKEND = ExactSegmentArrangementBackend()

ReferenceTelemetryCallback = Callable[
    [str, float, dict[str, int | float] | None],
    None,
]

# This is an explicit capability boundary, not a promise that every variant
# shares the Strip event law.
REFERENCE_BOUNDARY_CAPABILITIES_V1 = (
    ("StripEnvelope", "EXACT_COMPONENT_CONTACT_AND_NAMED_CAPACITY"),
    ("AngularEnvelope", "EXACT_CLIP_IF_NO_CONTACT_ELSE_NAMED_UNPROVEN"),
    (
        "CapEnvelope",
        "DERIVED_PHYSICAL_TERMINAL_WITH_INCIDENT_STRIP_CAPACITY",
    ),
    ("JunctionEnvelope", "GEOMETRY_LAW_UNPROVEN_FAIL_CLOSED"),
)


def _failure(
    outcome: ReferenceOutcome,
    message: str,
    *,
    existing: tuple[ReferenceEvaluationDiagnosticV1, ...] = (),
) -> ReferenceEvaluationResultV1:
    diagnostic = ReferenceEvaluationDiagnosticV1(
        outcome=outcome,
        severity=ReferenceDiagnosticSeverity.UNSUPPORTED,
        message=message,
    )
    return ReferenceEvaluationResultV1(outcome, None, (*existing, diagnostic))


def normalize_requested_alpha(
    alpha: LocalLengthV1 | Decimal | int | str,
) -> LocalLengthV1:
    if isinstance(alpha, LocalLengthV1):
        return alpha
    return LocalLengthV1(Decimal(str(alpha)), MetricSpace.SOURCE_LOCAL_INTRINSIC)


def _component_ids_for_spec(compilation, spec) -> frozenset[str]:
    if isinstance(spec, StripEnvelopeSpec):
        return frozenset(item.value for item in spec.front_component_ids)
    if isinstance(spec, AngularEnvelopeSpec):
        return frozenset(item.value for item in spec.incident_front_component_ids)
    if isinstance(spec, JunctionEnvelopeSpec):
        return frozenset(item.value for item in spec.incident_front_component_ids)
    if isinstance(spec, CapEnvelopeSpec):
        strip = next(
            item
            for item in compilation.envelope_specs
            if isinstance(item, StripEnvelopeSpec)
            and item.envelope_spec_id == spec.incident_strip_spec_id
        )
        return frozenset(item.value for item in strip.front_component_ids)
    return frozenset()


def spec_effective_alpha(compilation, spec, resolutions, alpha_value):
    """Эффективная alpha спеки — одна на все её инцидентные компоненты.

    `None` означает НЕОДНОРОДНЫЙ вектор: у инцидентных компонентов эффективные
    alpha разные, а одна огибающая с двумя alpha в v1 не доказана. Возвращать
    здесь «какую-нибудь» из них значило бы выбрать за вызывающего, поэтому
    отсутствие ответа отличимо от ответа, а имя исхода даёт вызывающий: у
    полного пути это `SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN`, у входа очереди —
    свой одноимённый по смыслу исход.

    Пустое множество компонентов — не ошибка: тогда эффективная alpha равна
    ЗАПРОШЕННОЙ, потому что резолвер к этой спеке не прикасался.

    Вынесено ради второго вызывающего: вход очереди (`wavefront/conveyor.py`)
    юбок не строит, но имя экземпляра юбки (`strip_envelope_instance_id`) стоит
    именно на эффективной alpha, и вывести её вторым способом значило бы
    получить второе имя для того же экземпляра.
    """

    effective_values = {
        resolutions[item].effective_alpha.expression
        for item in _component_ids_for_spec(compilation, spec)
        if item in resolutions
    }
    if len(effective_values) > 1:
        return None
    if effective_values:
        return ExactScalar(next(iter(effective_values))).as_expr()
    return sp.Rational(str(alpha_value.value))


def _arrangement_regions(arrangement) -> tuple[PlanarRegion, ...]:
    vertices = {item.vertex_id: item for item in arrangement.vertices}
    edges = {item.edge_id: item for item in arrangement.edges}
    loops = {item.loop_id: item for item in arrangement.loops}

    def convert_loop(loop_id: str) -> PlanarLoop:
        loop = loops[loop_id]
        segments = []
        for edge_id in loop.ordered_edge_ids:
            edge = edges[edge_id]
            start = vertices[edge.start_vertex_id]
            end = vertices[edge.end_vertex_id]
            segments.append(
                make_segment(
                    edge.edge_id,
                    start.point,
                    end.point,
                    support_ids=edge.provenance.support_ids,
                    provenance=edge.provenance,
                    start_certificates=start.construction_certificates,
                    end_certificates=end.construction_certificates,
                    boundary_constraint_ids=edge.provenance.boundary_constraint_ids,
                )
            )
        return PlanarLoop(loop_id, tuple(segments))

    return tuple(
        PlanarRegion(
            region_id=item.region_id,
            outer=convert_loop(item.outer_loop_id),
            holes=tuple(convert_loop(loop_id) for loop_id in item.hole_loop_ids),
            contributor_instance_ids=item.contributor_envelope_instance_ids,
            contributor_spec_ids=item.contributor_envelope_spec_ids,
        )
        for item in sorted(arrangement.regions, key=lambda record: record.region_id)
    )


def _emit_telemetry(
    telemetry: ReferenceTelemetryCallback | None,
    stage: str,
    started: float,
    counters: dict[str, int | float] | None = None,
) -> None:
    """Diagnostic observation must never affect exact evaluation."""

    if telemetry is None:
        return
    try:
        telemetry(stage, time.perf_counter() - started, counters)
    except Exception:
        return


def _union_counters(union, input_segments: int) -> dict[str, int | float]:
    """Наблюдение за arrangement: что пришло, что отсеялось, что получилось.

    `union` может быть None: телеметрия эмитится из `finally`, и стадия,
    упавшая на точном предикате, обязана оставить хотя бы вход. Стадия без
    единого числа — это стадия, про которую нечего сказать.
    """

    counters: dict[str, int | float] = {
        "ARRANGEMENT_INPUT_SEGMENTS": input_segments,
        "ARRANGEMENT_ALL_POSSIBLE_PAIRS": (
            input_segments * (input_segments - 1) // 2
        ),
    }
    if union is None:
        return counters
    counters.update(
        {
            "ARRANGEMENT_BROADPHASE_CANDIDATE_PAIRS": (
                union.broadphase_candidate_pair_count
            ),
            "ARRANGEMENT_NARROWPHASE_TESTS": union.narrowphase_test_count,
            # Совместимый alias: теперь это фактические exact tests,
            # а не теоретическое число всех пар.
            "ARRANGEMENT_PAIR_TESTS": union.narrowphase_test_count,
            "ARRANGEMENT_INTERSECTIONS": union.intersection_count,
            "ARRANGEMENT_ATOMIC_SEGMENTS": union.atomic_edge_count,
            "ARRANGEMENT_BRANCH_POINTS": union.branch_point_count,
            "ARRANGEMENT_MAX_INCIDENT_DEGREE": union.max_incident_degree,
            "ARRANGEMENT_BOUNDARY_OCCURRENCES": len(
                union.boundary_vertex_occurrences
            ),
            "ARRANGEMENT_POINT_CONTACTS": len(union.point_contacts),
            "ARRANGEMENT_ROTATION_COMPARISONS": (
                union.rotation_comparison_count
            ),
            "ARRANGEMENT_FACE_WALKS": union.face_walk_count,
            # Выход arrangement. Вход уже есть выше; без выхода по паре чисел
            # не отличить «пришло много» от «расплодилось на пересечениях»,
            # а это разные болезни с разным лечением.
            "ARRANGEMENT_OUTPUT_VERTICES": len(union.vertices),
            "ARRANGEMENT_OUTPUT_EDGES": len(union.edges),
            "ARRANGEMENT_OUTPUT_LOOPS": len(union.loops),
            "ARRANGEMENT_OUTPUT_REGIONS": len(union.regions),
            # Две величины, которых не хватило, чтобы объяснить полевые времена
            # по счётчикам: работа локализации точки и длина точных координат.
            "ARRANGEMENT_POINT_LOCATION_SCANS": (
                union.point_location_segment_scan_count
            ),
            "ARRANGEMENT_MAX_COORDINATE_CHARS": union.max_coordinate_chars,
        }
    )
    return counters


def _region_segment_count(regions: tuple[PlanarRegion, ...]) -> int:
    return sum(
        len(region.outer.segments)
        + sum(len(hole.segments) for hole in region.holes)
        for region in regions
    )


def _clip_instance_to_domain(
    instance: ReferenceEnvelopeInstanceV1,
    domain_regions: tuple[PlanarRegion, ...],
    reachability: ReachabilityCertificateV1,
) -> tuple[ReferenceEnvelopeInstanceV1 | None, ReferenceOutcome | None]:
    if not instance.regions:
        return instance, None
    arrangement = REFERENCE_ARRANGEMENT_BACKEND.exact_union(
        instance.regions,
        domain_regions,
        {instance.envelope_instance_id: reachability},
    )
    regions = _arrangement_regions(arrangement)
    if instance.envelope_variant == "AngularEnvelope":
        original_area = sum(
            (
                polygon_signed_area(segment.start for segment in region.outer.segments)
                + sum(
                    polygon_signed_area(segment.start for segment in hole.segments)
                    for hole in region.holes
                )
            )
            for region in instance.regions
        )
        if exact_sign(
            original_area - ExactScalar(arrangement.exact_area_expression).as_expr()
        ) != 0:
            return (
                None,
                ReferenceOutcome.REFERENCE_ANGULAR_BOUNDARY_CONTACT_UNPROVEN,
            )
    if len(regions) > 1:
        if instance.envelope_variant == "AngularEnvelope":
            return (
                None,
                ReferenceOutcome.REFERENCE_ANGULAR_BOUNDARY_CONTACT_UNPROVEN,
            )
        return None, ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED
    exposed = tuple(
        segment for region in regions for segment in region.outer.segments
    )
    exposed += tuple(
        segment for region in regions for hole in region.holes for segment in hole.segments
    )
    return replace(instance, regions=regions, exposed_segments=exposed), None


# Полоса огибающей обязана быть хотя бы в СТОЛЬКО ячеек решётки.
#
# Три ячейки — это уже фигура, различимая на решётке лишь в одном положении;
# четыре — первое число, при котором полоса имеет внутренность при любом
# сдвиге относительно сетки. Проверка нужна потому, что `decal_detail`
# описывает НАМЕРЕНИЕ (самая тонкая деталь, которую рисует декаль), а alpha —
# то, что запрошено на самом деле, и расходиться они могут: полоса в 2 мм на
# стометровом патче при шаге 1.95 мм — одна ячейка, то есть уничтожена, а
# объявленный сантиметр при этом скажет `WINDOW_AVAILABLE`.
BAND_MINIMUM_CELLS = 4

_NEGATIVE_ALPHA = "reference alpha must be non-negative"


def _require_band_against_grid(metric, alpha_value) -> None:
    """`alpha >= 4 x шаг` либо именованный отказ."""

    if metric.source_step is None:
        return
    requested = Fraction(str(alpha_value.value))
    minimum = BAND_MINIMUM_CELLS * metric.source_step
    if requested >= minimum:
        return
    raise ReferenceGeometryError(
        ReferenceOutcome.ENVELOPE_BAND_BELOW_GRID_RESOLUTION,
        "полоса огибающей тоньше четырёх ячеек решётки: "
        f"alpha={float(requested):.6g}, шаг={float(metric.source_step):.6g}, "
        f"минимум={float(minimum):.6g}",
    )


def evaluate_reference_raw_coverage(
    compilation: ReferenceEnvelopeCompilationV1,
    alpha: LocalLengthV1 | Decimal | int | str,
    *,
    telemetry: ReferenceTelemetryCallback | None = None,
) -> ReferenceEvaluationResultV1:
    """Выполнить evaluator в режиме, объявленном compilation binding."""

    with bound_evaluation_arrangement(
        compilation.evaluation_geometry_binding is not None
    ):
        return _evaluate_reference_raw_coverage(
            compilation,
            alpha,
            telemetry=telemetry,
        )


def _evaluate_reference_raw_coverage(
    compilation: ReferenceEnvelopeCompilationV1,
    alpha: LocalLengthV1 | Decimal | int | str,
    *,
    telemetry: ReferenceTelemetryCallback | None = None,
) -> ReferenceEvaluationResultV1:
    """Rebuild one exact request/domain RawCoverage state from scratch."""

    try:
        alpha_value = normalize_requested_alpha(alpha)
    except (ValueError, ArithmeticError) as exc:
        return _failure(ReferenceOutcome.REFERENCE_INVALID_ALPHA, str(exc))
    if alpha_value.value < 0:
        return _failure(ReferenceOutcome.REFERENCE_INVALID_ALPHA, _NEGATIVE_ALPHA)
    frame, payload_diagnostics = validate_compilation_geometry_payload(compilation)
    if frame is None:
        return ReferenceEvaluationResultV1(
            payload_diagnostics[0].outcome, None, payload_diagnostics
        )
    try:
        context = GeometryContext.build(compilation, frame)
        _require_band_against_grid(context.metric, alpha_value)
        stage_started = time.perf_counter()
        domain = build_domain_geometry(context)
        _emit_telemetry(
            telemetry,
            "DOMAIN_BUILD",
            stage_started,
            {
                "DOMAIN_REGIONS": len(domain.domain_regions),
                "DOMAIN_OUTER_LOOPS": len(domain.outer_loops),
                "DOMAIN_HOLE_LOOPS": len(domain.hole_loops),
                "DOMAIN_EXPLICIT_BARRIERS": len(domain.explicit_barriers),
                "DOMAIN_SOURCE_FACE_CONTRIBUTORS": len(
                    domain.source_face_contributor_ids
                ),
                "DOMAIN_FACE_BOUNDARY_SEGMENTS_ORACLE": sum(
                    len(face.edge_cycle)
                    for face in context.snapshot.surface_ir.source_faces
                    if face.patch_id == context.compilation.owner_patch_id
                ),
                "DOMAIN_INPUT_SEGMENTS": domain.sparse_segment_count,
                "DOMAIN_SPARSE_SEGMENTS": domain.sparse_segment_count,
                "DOMAIN_BOUNDARY_SEGMENTS": domain.boundary_segment_count,
                "ARRANGEMENT_DOMAIN_SEGMENTS": domain.boundary_segment_count,
                "BOUNDARY_RESOLVER_BARRIER_SEGMENTS": len(
                    domain.explicit_barriers
                ),
            },
        )
        clip_elapsed = 0.0
        stage_started = time.perf_counter()
        resolutions, boundary_diagnostics = resolve_component_alphas(
            context, alpha_value, domain
        )
        clip_elapsed += time.perf_counter() - stage_started
        instances = []
        boundary_resolved = []
        instance_elapsed = 0.0
        # Сегменты до и после клипа. Клип — вторая по стоимости стадия в поле,
        # и без этих двух чисел непонятно, растёт ли она от размера огибающих
        # или от размера домена, о который их режут.
        clip_segments_in = 0
        clip_segments_out = 0
        try:
            for spec in sorted(
                compilation.envelope_specs,
                key=lambda item: item.envelope_spec_id.value,
            ):
                component_ids = _component_ids_for_spec(compilation, spec)
                effective = spec_effective_alpha(
                    compilation, spec, resolutions, alpha_value
                )
                if effective is None:
                    return _failure(
                        ReferenceOutcome.SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN,
                        f"{spec.envelope_spec_id} has a non-uniform incident effective-alpha vector",
                        existing=boundary_diagnostics,
                    )
                stage_started = time.perf_counter()
                if isinstance(spec, StripEnvelopeSpec):
                    instance = evaluate_strip_envelope(
                        context, spec, alpha_value, effective
                    )
                elif isinstance(spec, AngularEnvelopeSpec):
                    instance = evaluate_angular_envelope(
                        context, spec, alpha_value, effective
                    )
                elif isinstance(spec, JunctionEnvelopeSpec):
                    instance = evaluate_junction_envelope(
                        context, spec, alpha_value, effective
                    )
                elif isinstance(spec, CapEnvelopeSpec):
                    instance = evaluate_cap_envelope(
                        context, spec, alpha_value, effective
                    )
                else:  # pragma: no cover - EC1 union is closed and validated.
                    raise TypeError(type(spec).__name__)
                instance_elapsed += time.perf_counter() - stage_started
                clip_segments_in += _region_segment_count(instance.regions)
                if isinstance(spec, AngularEnvelopeSpec) and any(
                    segment_intersections(exposed, barrier.segment)
                    for exposed in instance.exposed_segments
                    for barrier in domain.blocking_segments
                    if barrier.role is BoundaryRole.EXPLICIT_BARRIER
                ):
                    return _failure(
                        ReferenceOutcome.REFERENCE_ANGULAR_BOUNDARY_CONTACT_UNPROVEN,
                        f"{spec.envelope_spec_id} contacts an explicit barrier "
                        "without an approved Angular event law",
                        existing=boundary_diagnostics,
                    )
                component_resolutions = [
                    resolutions[item]
                    for item in component_ids
                    if item in resolutions
                ]
                reachability = ReachabilityCertificateV1(
                    front_component_ids=component_ids,
                    source_launch_reachable=True,
                    initial_branch_count=len(component_ids),
                    effective_branch_count=len(component_ids),
                    boundary_event_keys=tuple(
                        sorted(
                            {
                                event
                                for resolution in component_resolutions
                                for event in resolution.event_keys
                            }
                        )
                    ),
                    bypass_used=False,
                )
                stage_started = time.perf_counter()
                clipped, clip_outcome = _clip_instance_to_domain(
                    instance, domain.domain_regions, reachability
                )
                clip_elapsed += time.perf_counter() - stage_started
                if clipped is None:
                    return _failure(
                        clip_outcome
                        or ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED,
                        f"{spec.envelope_spec_id} would require obstacle bypass",
                        existing=boundary_diagnostics,
                    )
                clip_segments_out += _region_segment_count(clipped.regions)
                capacity_outcomes = {
                    item.capacity_outcome
                    for item in component_resolutions
                    if item.capacity_outcome is not None
                }
                capacity_outcome = (
                    next(iter(capacity_outcomes))
                    if capacity_outcomes
                    else None
                )
                diagnostics = tuple(
                    diagnostic
                    for item in component_resolutions
                    for diagnostic in item.diagnostics
                )
                boundary_resolved.append(
                    BoundaryResolvedEnvelopeV1(
                        envelope_instance=clipped,
                        requested_alpha=alpha_value,
                        effective_alpha=ExactScalar.from_value(effective),
                        reachability=reachability,
                        capacity_outcome=capacity_outcome,
                        diagnostics=diagnostics,
                    )
                )
                instances.append(clipped)
        finally:
            if telemetry is not None:
                try:
                    telemetry(
                        "ENVELOPE_INSTANCE_BUILD",
                        instance_elapsed,
                        {
                            "ENVELOPE_INSTANCES": len(instances),
                            "ENVELOPE_INSTANCE_SEGMENTS": clip_segments_in,
                        },
                    )
                    telemetry(
                        "DOMAIN_CLIP",
                        clip_elapsed,
                        {
                            "CLIP_SEGMENTS_IN": clip_segments_in,
                            "CLIP_SEGMENTS_OUT": clip_segments_out,
                        },
                    )
                except Exception:
                    pass

        contribution_regions = tuple(
            region for instance in instances for region in instance.regions
        )
        reachability_by_instance = {
            item.envelope_instance.envelope_instance_id: item.reachability
            for item in boundary_resolved
        }
        arrangement_input_segments = (
            _region_segment_count(contribution_regions)
            + _region_segment_count(domain.domain_regions)
        )
        stage_started = time.perf_counter()
        try:
            union = REFERENCE_ARRANGEMENT_BACKEND.exact_union(
                contribution_regions,
                domain.domain_regions,
                reachability_by_instance,
            )
        finally:
            _emit_telemetry(
                telemetry,
                "RAW_UNION",
                stage_started,
                _union_counters(
                    locals().get("union"),
                    arrangement_input_segments,
                ),
            )
        result = RawCoverageResultV2(
            schema_version=RAW_COVERAGE_RESULT_SCHEMA_V2,
            source_revision=compilation.source_revision,
            plan_key=compilation.plan_key,
            requested_alpha=alpha_value,
            component_effective_alphas=frozenset(
                item.public() for item in resolutions.values()
            ),
            vertices=union.vertices,
            edges=union.edges,
            loops=union.loops,
            regions=union.regions,
            envelope_instances=frozenset(instances),
            boundary_resolved_envelopes=frozenset(boundary_resolved),
            exact_area_expression=union.exact_area_expression,
            semantic_digest="",
            diagnostics=boundary_diagnostics,
            boundary_vertex_occurrences=union.boundary_vertex_occurrences,
            point_contacts=union.point_contacts,
        )
        result = replace(
            result,
            semantic_digest=raw_coverage_semantic_digest(result).sha256_hex,
        )
        capacity = next(
            (
                item.capacity_outcome
                for item in resolutions.values()
                if item.capacity_outcome is not None
            ),
            None,
        )
        return ReferenceEvaluationResultV1(
            capacity or ReferenceOutcome.EXACT,
            result,
            boundary_diagnostics,
        )
    except ReferenceGeometryError as exc:
        return _failure(exc.outcome, str(exc))
    except ExactArrangementCollinearBranchUnproven as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_ARRANGEMENT_COLLINEAR_BRANCH_UNPROVEN,
            str(exc),
        )
    except ExactTouchingHoleTopologyUnproven as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_TOUCHING_HOLE_TOPOLOGY_UNPROVEN,
            str(exc),
        )
    except ExactArrangementRotationSystemUnproven as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_ARRANGEMENT_ROTATION_SYSTEM_UNPROVEN,
            str(exc),
        )
    except CertifiedPredicateUndecidable as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            str(exc),
        )
