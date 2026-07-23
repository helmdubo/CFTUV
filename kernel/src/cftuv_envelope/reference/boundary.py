"""Domain extraction and boundary-limited FrontComponent saturation."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from functools import cmp_to_key

import sympy as sp

from ..contracts.analysis import (
    BoundaryLoopConstraintTargetV1,
    ChainUseConstraintTargetV1,
    PhysicalEdgeSequenceConstraintTargetV1,
)
from ..contracts.envelopes import StripEnvelopeSpec
from ..ids import PhysicalEdgeId
from ..numeric import LocalLengthV1
from .common import (
    GeometryContext,
    SourceSupportSegment,
    make_segment,
    source_vertex_certificate,
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
    PlanarLoop,
    PlanarRegion,
    cross,
    dot,
    exact_sign,
    point_add,
    point_key,
    point_sub,
    points_equal,
    polygon_signed_area,
    vector_scale,
)
from .provenance import ReferenceProvenanceV1
from .provenance import merge_provenance


class BoundaryRole(str, Enum):
    OUTER = "OUTER"
    HOLE = "HOLE"
    EXPLICIT_BARRIER = "EXPLICIT_BARRIER"


@dataclass(frozen=True, slots=True)
class BlockingBoundarySegment:
    segment: object
    role: BoundaryRole
    concave_vertex_keys: frozenset[tuple[str, str]] = frozenset()


@dataclass(frozen=True, slots=True)
class DomainGeometry:
    face_regions: tuple[PlanarRegion, ...]
    blocking_segments: tuple[BlockingBoundarySegment, ...]


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


def _constraint_edges(context: GeometryContext, constraint) -> frozenset[PhysicalEdgeId]:
    target = constraint.target
    if isinstance(target, PhysicalEdgeSequenceConstraintTargetV1):
        return frozenset(target.ordered_physical_edge_ids)
    if isinstance(target, ChainUseConstraintTargetV1):
        chain_use = context.uses_by_id[target.chain_use_id]
        return frozenset(
            context.chains_by_id[chain_use.physical_chain_id].ordered_physical_edge_ids
        )
    if isinstance(target, BoundaryLoopConstraintTargetV1):
        loop = next(
            item
            for item in context.snapshot.boundary_loops
            if item.boundary_loop_id == target.boundary_loop_id
        )
        return frozenset(
            edge_id
            for chain_use_id in loop.ordered_chain_use_ids
            for edge_id in context.chains_by_id[
                context.uses_by_id[chain_use_id].physical_chain_id
            ].ordered_physical_edge_ids
        )
    return frozenset()


def build_domain_geometry(context: GeometryContext) -> DomainGeometry:
    owner_patch_id = context.compilation.owner_patch_id
    constraints_by_edge = {}
    for constraint in context.snapshot.boundary_constraints:
        if constraint.patch_domain_id != context.compilation.plan_key.patch_domain_id:
            continue
        for edge_id in _constraint_edges(context, constraint):
            constraints_by_edge.setdefault(edge_id, set()).add(constraint)
    uses_by_edge = {}
    for chain_use in context.snapshot.chain_uses:
        if chain_use.patch_domain_id != context.compilation.plan_key.patch_domain_id:
            continue
        chain = context.chains_by_id[chain_use.physical_chain_id]
        for edge_id in chain.ordered_physical_edge_ids:
            uses_by_edge.setdefault(edge_id, set()).add(chain_use.chain_use_id.value)

    face_regions = []
    face_segments_by_edge = {}
    for face in sorted(
        (
            item
            for item in context.snapshot.surface_ir.source_faces
            if item.patch_id == owner_patch_id
        ),
        key=lambda item: item.face_id.value,
    ):
        segments = []
        count = len(face.vertex_cycle)
        for ordinal in range(count):
            start_id = face.vertex_cycle[ordinal]
            end_id = face.vertex_cycle[(ordinal + 1) % count]
            edge_id = face.edge_cycle[ordinal]
            start = context.points_by_id[start_id]
            end = context.points_by_id[end_id]
            constraint_ids = frozenset(
                item.boundary_constraint_id.value
                for item in constraints_by_edge.get(edge_id, ())
            )
            provenance = ReferenceProvenanceV1(
                physical_edge_ids=frozenset({edge_id.value}),
                source_face_ids=frozenset({face.face_id.value}),
                chain_use_ids=frozenset(uses_by_edge.get(edge_id, ())),
                patch_domain_ids=frozenset(
                    {context.compilation.plan_key.patch_domain_id.value}
                ),
                boundary_constraint_ids=constraint_ids,
            )
            segment = make_segment(
                stable_id("domain-edge", face.face_id, edge_id),
                start,
                end,
                support_ids=frozenset({stable_id("domain-support", edge_id)}),
                provenance=provenance,
                start_certificates=frozenset({source_vertex_certificate(start_id)}),
                end_certificates=frozenset({source_vertex_certificate(end_id)}),
                boundary_constraint_ids=constraint_ids,
            )
            segments.append(segment)
            face_segments_by_edge.setdefault(edge_id, []).append(segment)
        area = polygon_signed_area(item.start for item in segments)
        if exact_sign(area) == 0:
            raise ValueError(f"source face {face.face_id} has zero planar area")
        if exact_sign(area) < 0:
            segments = [item.reversed() for item in reversed(segments)]
        face_regions.append(
            PlanarRegion(
                region_id=stable_id("domain-face-region", face.face_id),
                outer=PlanarLoop(stable_id("domain-face-loop", face.face_id), tuple(segments)),
            )
        )

    exterior_segments = [
        items[0] for items in face_segments_by_edge.values() if len(items) == 1
    ]
    outgoing = {}
    for segment in exterior_segments:
        outgoing.setdefault(point_key(segment.start), []).append(segment)
    if any(len(items) != 1 for items in outgoing.values()):
        raise ValueError("PatchDomain physical boundary is not a directed 2-manifold")
    unvisited = {item.segment_id for item in exterior_segments}
    boundary_loops = []
    while unvisited:
        first = next(item for item in exterior_segments if item.segment_id == min(unvisited))
        current = first
        loop = []
        while current.segment_id in unvisited:
            unvisited.remove(current.segment_id)
            loop.append(current)
            candidates = outgoing.get(point_key(current.end), ())
            if len(candidates) != 1:
                raise ValueError("PatchDomain boundary loop is open")
            current = candidates[0]
        if current.segment_id != first.segment_id:
            raise ValueError("PatchDomain boundary traversal is non-closing")
        boundary_loops.append(tuple(loop))

    blocking = []
    for loop in boundary_loops:
        area_sign = exact_sign(polygon_signed_area(item.start for item in loop))
        role = BoundaryRole.OUTER if area_sign > 0 else BoundaryRole.HOLE
        concave = set()
        if role is BoundaryRole.OUTER:
            points = tuple(item.start for item in loop)
            for previous, current, following in zip(
                points[-1:] + points[:-1], points, points[1:] + points[:1]
            ):
                if exact_sign(
                    cross(point_sub(current, previous), point_sub(following, current))
                ) < 0:
                    concave.add(point_key(current))
        for segment in loop:
            blocking.append(
                BlockingBoundarySegment(segment, role, frozenset(concave))
            )

    explicit_edge_ids = {
        edge_id
        for constraint in context.snapshot.boundary_constraints
        if constraint.patch_domain_id == context.compilation.plan_key.patch_domain_id
        and constraint.constraint_kind.value == "PHYSICAL_DOMAIN_BARRIER"
        for edge_id in _constraint_edges(context, constraint)
    }
    for edge_id in sorted(explicit_edge_ids, key=lambda item: item.value):
        if any(
            edge_id.value in item.segment.provenance.physical_edge_ids
            and item.role in (BoundaryRole.OUTER, BoundaryRole.HOLE)
            for item in blocking
        ):
            continue
        edge = context.source_edges_by_id[edge_id]
        start = context.points_by_id[edge.vertex_a_id]
        end = context.points_by_id[edge.vertex_b_id]
        constraints = constraints_by_edge.get(edge_id, ())
        constraint_ids = frozenset(item.boundary_constraint_id.value for item in constraints)
        segment = make_segment(
            stable_id("explicit-barrier-edge", edge_id),
            start,
            end,
            support_ids=frozenset({stable_id("barrier-support", edge_id)}),
            provenance=ReferenceProvenanceV1(
                physical_edge_ids=frozenset({edge_id.value}),
                patch_domain_ids=frozenset(
                    {context.compilation.plan_key.patch_domain_id.value}
                ),
                boundary_constraint_ids=constraint_ids,
            ),
            start_certificates=frozenset({source_vertex_certificate(edge.vertex_a_id)}),
            end_certificates=frozenset({source_vertex_certificate(edge.vertex_b_id)}),
            boundary_constraint_ids=constraint_ids,
        )
        blocking.append(BlockingBoundarySegment(segment, BoundaryRole.EXPLICIT_BARRIER))
    return DomainGeometry(tuple(face_regions), tuple(blocking))


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
    domain_geometry: DomainGeometry,
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
                    else:
                        endpoint_events.append((alpha, event_key, construction, interior))
            diagnostics = []
            if best_split is not None:
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
