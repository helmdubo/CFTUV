"""Domain extraction and boundary-limited FrontComponent saturation."""

from __future__ import annotations

from dataclasses import dataclass, replace
from functools import cmp_to_key

import sympy as sp

from ..contracts.envelopes import StripEnvelopeSpec
from ..numeric import LocalLengthV1
from .common import (
    GeometryContext,
    SourceSupportSegment,
    stable_id,
)
from .contracts import (
    ComponentEffectiveAlphaV1,
    ReferenceDiagnosticSeverity,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)
from .planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactScalar,
    cross,
    dot,
    exact_sign,
    point_add,
    point_key,
    point_sub,
    points_equal,
    vector_scale,
)
from .domain_geometry import (
    BlockingBoundarySegment,
    BoundaryRole,
    SparsePatchDomainGeometryV1,
    build_sparse_patch_domain_geometry,
)
from .provenance import (
    merge_provenance,
)

@dataclass(frozen=True, slots=True)
class ComponentResolution:
    front_component_id: str
    requested_alpha: LocalLengthV1
    effective_alpha: ExactScalar
    capacity_outcome: ReferenceOutcome | None
    event_keys: tuple[str, ...]
    diagnostics: tuple[ReferenceEvaluationDiagnosticV1, ...]

    def public(self) -> ComponentEffectiveAlphaV1:
        return ComponentEffectiveAlphaV1(
            front_component_id=self.front_component_id,
            requested_alpha=self.requested_alpha,
            effective_alpha=self.effective_alpha,
            capacity_outcome=self.capacity_outcome,
        )


def build_domain_geometry(
    context: GeometryContext,
) -> SparsePatchDomainGeometryV1:
    """Compatibility facade for the sparse runtime contract."""

    return build_sparse_patch_domain_geometry(context)


def _contact_candidates(
    source: SourceSupportSegment, boundary: BlockingBoundarySegment
) -> tuple[tuple[sp.Expr, sp.Expr, ExactPlanarPoint], ...]:
    barrier = boundary.segment
    barrier_direction = point_sub(barrier.end, barrier.start)
    source_direction = point_sub(source.end, source.start)
    length = sp.sqrt(dot(source_direction, source_direction))
    s0 = dot(point_sub(barrier.start, source.start), source.tangent)
    ds = dot(barrier_direction, source.tangent)
    a0 = dot(point_sub(barrier.start, source.start), source.owner_normal)
    da = dot(barrier_direction, source.owner_normal)
    candidates = {sp.Integer(0), sp.Integer(1)}
    if exact_sign(ds) != 0:
        candidates.add(sp.factor(-s0 / ds))
        candidates.add(sp.factor((length - s0) / ds))
    if exact_sign(da) != 0:
        candidates.add(sp.factor(-a0 / da))
    result = []
    for parameter in candidates:
        if exact_sign(parameter) < 0 or exact_sign(parameter - 1) > 0:
            continue
        station = sp.factor(s0 + parameter * ds)
        alpha = sp.factor(a0 + parameter * da)
        if exact_sign(station) < 0 or exact_sign(station - length) > 0:
            continue
        if exact_sign(alpha) < 0:
            continue
        point = point_add(barrier.start, vector_scale(barrier_direction, parameter))
        result.append((alpha, station, point))
    result.sort(
        key=cmp_to_key(lambda left, right: exact_sign(left[0] - right[0]))
    )
    return tuple(result)


def _continuous_support_intervals(
    segments: tuple[SourceSupportSegment, ...]
) -> tuple[SourceSupportSegment, ...]:
    if not segments:
        return ()
    groups = [[segments[0]]]
    for segment in segments[1:]:
        previous = groups[-1][-1]
        continuous = points_equal(previous.end, segment.start)
        collinear = exact_sign(cross(previous.tangent, segment.tangent)) == 0
        same_direction = exact_sign(dot(previous.tangent, segment.tangent)) > 0
        same_normal = (
            exact_sign(cross(previous.owner_normal, segment.owner_normal)) == 0
            and exact_sign(dot(previous.owner_normal, segment.owner_normal)) > 0
        )
        if continuous and collinear and same_direction and same_normal:
            groups[-1].append(segment)
        else:
            groups.append([segment])
    result = []
    for group in groups:
        first = group[0]
        last = group[-1]
        result.append(
            replace(
                first,
                source_vertex_end_id=last.source_vertex_end_id,
                end=last.end,
                support_id=stable_id(
                    "continuous-front-support",
                    first.front_component_id,
                    *(item.support_id for item in group),
                ),
                provenance=merge_provenance(*(item.provenance for item in group)),
            )
        )
    return tuple(result)


def resolve_component_alphas(
    context: GeometryContext,
    requested_alpha: LocalLengthV1,
    domain_geometry: SparsePatchDomainGeometryV1,
) -> tuple[dict[str, ComponentResolution], tuple[ReferenceEvaluationDiagnosticV1, ...]]:
    requested = sp.Rational(str(requested_alpha.value))
    resolutions = {
        item.front_component_id.value: ComponentResolution(
            item.front_component_id.value,
            requested_alpha,
            ExactScalar.from_value(requested),
            None,
            (),
            (),
        )
        for item in context.compilation.front_components
    }
    all_diagnostics = []
    for spec in sorted(
        (
            item
            for item in context.compilation.envelope_specs
            if isinstance(item, StripEnvelopeSpec)
        ),
        key=lambda item: item.envelope_spec_id.value,
    ):
        seed = next(
            item
            for item in context.compilation.seeds
            if getattr(item, "seed_id", None) == spec.source_seed_id
        )
        for source in _continuous_support_intervals(
            context.support_segments_for_use(
                seed.chain_use_id, spec.envelope_spec_id.value
            )
        ):
            best_split = None
            best_bypass = None
            endpoint_events = []
            for boundary in domain_geometry.blocking_segments:
                if source.physical_edge_id.value in boundary.segment.provenance.physical_edge_ids:
                    continue
                for alpha, station, point in _contact_candidates(source, boundary):
                    if exact_sign(alpha) == 0:
                        continue
                    if exact_sign(alpha - requested) > 0:
                        continue
                    source_length = sp.sqrt(
                        dot(point_sub(source.end, source.start), point_sub(source.end, source.start))
                    )
                    interior = exact_sign(station) > 0 and exact_sign(station - source_length) < 0
                    split = interior and (
                        boundary.role in (BoundaryRole.HOLE, BoundaryRole.EXPLICIT_BARRIER)
                        or point_key(point) in boundary.concave_vertex_keys
                    )
                    event_key = stable_id(
                        "boundary-contact",
                        source.front_component_id,
                        source.support_id,
                        boundary.segment.segment_id,
                        ExactScalar.from_value(alpha).expression,
                    )
                    construction = ConstructionCertificate(
                        kind=ConstructionKind.EVENT_ANCHOR,
                        support_ids=frozenset({source.support_id}),
                        boundary_constraint_ids=boundary.segment.boundary_constraint_ids,
                        physical_edge_ids=frozenset(
                            boundary.segment.provenance.physical_edge_ids
                        ),
                        event_key=event_key,
                    )
                    if split:
                        candidate = (alpha, event_key, construction)
                        if best_split is None or exact_sign(alpha - best_split[0]) < 0:
                            best_split = candidate
                    elif (
                        boundary.role is BoundaryRole.EXPLICIT_BARRIER
                        and not interior
                        and exact_sign(requested - alpha) > 0
                    ):
                        candidate = (alpha, event_key, construction)
                        if (
                            best_bypass is None
                            or exact_sign(alpha - best_bypass[0]) < 0
                        ):
                            best_bypass = candidate
                    else:
                        endpoint_events.append((alpha, event_key, construction, interior))
            diagnostics = []
            if best_bypass is not None and (
                best_split is None
                or exact_sign(best_bypass[0] - best_split[0]) < 0
            ):
                alpha, event_key, construction = best_bypass
                diagnostic = ReferenceEvaluationDiagnosticV1(
                    outcome=ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED,
                    severity=ReferenceDiagnosticSeverity.CAPACITY,
                    message=(
                        "continuation beyond an explicit barrier endpoint "
                        "would require unsupported obstacle bypass"
                    ),
                    envelope_spec_id=spec.envelope_spec_id.value,
                    front_component_id=source.front_component_id,
                    requested_alpha=requested_alpha,
                    effective_alpha=ExactScalar.from_value(alpha),
                    construction=construction,
                )
                diagnostics.append(diagnostic)
                current = resolutions[source.front_component_id]
                if exact_sign(alpha - current.effective_alpha.as_expr()) <= 0:
                    resolutions[source.front_component_id] = ComponentResolution(
                        source.front_component_id,
                        requested_alpha,
                        ExactScalar.from_value(alpha),
                        ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED,
                        tuple(sorted(set((*current.event_keys, event_key)))),
                        tuple((*current.diagnostics, diagnostic)),
                    )
            elif best_split is not None:
                alpha, event_key, construction = best_split
                diagnostic = ReferenceEvaluationDiagnosticV1(
                    outcome=ReferenceOutcome.BARRIER_SPLIT_REQUIRED,
                    severity=ReferenceDiagnosticSeverity.CAPACITY,
                    message="interior boundary contact would increase FrontComponent branch count",
                    envelope_spec_id=spec.envelope_spec_id.value,
                    front_component_id=source.front_component_id,
                    requested_alpha=requested_alpha,
                    effective_alpha=ExactScalar.from_value(alpha),
                    construction=construction,
                )
                diagnostics.append(diagnostic)
                current = resolutions[source.front_component_id]
                if exact_sign(alpha - current.effective_alpha.as_expr()) <= 0:
                    resolutions[source.front_component_id] = ComponentResolution(
                        source.front_component_id,
                        requested_alpha,
                        ExactScalar.from_value(alpha),
                        ReferenceOutcome.BARRIER_SPLIT_REQUIRED,
                        tuple(sorted(set((*current.event_keys, event_key)))),
                        tuple((*current.diagnostics, diagnostic)),
                    )
            elif endpoint_events:
                ordered_events = sorted(
                    endpoint_events,
                    key=cmp_to_key(
                        lambda left, right: exact_sign(left[0] - right[0])
                    ),
                )
                minimum_alpha = ordered_events[0][0]
                simultaneous = [
                    item
                    for item in ordered_events
                    if exact_sign(item[0] - minimum_alpha) == 0
                ]
                for alpha, event_key, construction, interior in simultaneous:
                    diagnostics.append(
                        ReferenceEvaluationDiagnosticV1(
                            outcome=ReferenceOutcome.EXACT,
                            severity=ReferenceDiagnosticSeverity.INFO,
                            message=(
                                "boundary contact clips or shrinks one active interval"
                                if interior
                                else "endpoint contact may slide on the same boundary component"
                            ),
                            envelope_spec_id=spec.envelope_spec_id.value,
                            front_component_id=source.front_component_id,
                            requested_alpha=requested_alpha,
                            effective_alpha=ExactScalar.from_value(requested),
                            construction=construction,
                        )
                    )
                current = resolutions[source.front_component_id]
                resolutions[source.front_component_id] = ComponentResolution(
                    current.front_component_id,
                    current.requested_alpha,
                    current.effective_alpha,
                    current.capacity_outcome,
                    tuple(
                        sorted(
                            set(
                                (
                                    *current.event_keys,
                                    *(item[1] for item in simultaneous),
                                )
                            )
                        )
                    ),
                    tuple((*current.diagnostics, *diagnostics)),
                )
            all_diagnostics.extend(diagnostics)
    return resolutions, tuple(all_diagnostics)
