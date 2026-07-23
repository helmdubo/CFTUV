"""Full-recompute EnvelopeInstances -> boundary resolution -> exact RawCoverage."""

from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from hashlib import sha256

import sympy as sp

from ..codec import canonical_json_bytes
from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    CapEnvelopeSpec,
    JunctionEnvelopeSpec,
    StripEnvelopeSpec,
)
from ..numeric import LocalLengthV1, MetricSpace
from .angular import evaluate_angular_envelope
from .arrangement import ExactSegmentArrangementBackend
from .boundary import (
    build_domain_geometry,
    resolve_component_alphas,
)
from .cap import evaluate_cap_envelope
from .common import GeometryContext, ReferenceGeometryError, make_segment
from .contracts import (
    BoundaryResolvedEnvelopeV1,
    RAW_COVERAGE_RESULT_SCHEMA_V1,
    RawCoverageResultV1,
    ReachabilityCertificateV1,
    ReferenceDiagnosticSeverity,
    ReferenceEnvelopeCompilationV1,
    ReferenceEnvelopeInstanceV1,
    ReferenceEvaluationDiagnosticV1,
    ReferenceEvaluationResultV1,
    ReferenceOutcome,
)
from .junction import evaluate_junction_envelope
from .planar_types import (
    CertifiedPredicateUndecidable,
    ExactScalar,
    PlanarLoop,
    PlanarRegion,
    exact_sign,
)
from .strip import evaluate_strip_envelope
from .validation import validate_reference_geometry_payload


REFERENCE_ARRANGEMENT_BACKEND = ExactSegmentArrangementBackend()


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


def _normalize_alpha(alpha: LocalLengthV1 | Decimal | int | str) -> LocalLengthV1:
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
    if len(regions) > 1:
        return None, ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED
    exposed = tuple(
        segment for region in regions for segment in region.outer.segments
    )
    exposed += tuple(
        segment for region in regions for hole in region.holes for segment in hole.segments
    )
    return replace(instance, regions=regions, exposed_segments=exposed), None


def evaluate_reference_raw_coverage(
    compilation: ReferenceEnvelopeCompilationV1,
    alpha: LocalLengthV1 | Decimal | int | str,
) -> ReferenceEvaluationResultV1:
    """Rebuild one exact request/domain RawCoverage state from scratch."""

    try:
        alpha_value = _normalize_alpha(alpha)
    except (ValueError, ArithmeticError) as exc:
        return _failure(ReferenceOutcome.REFERENCE_INVALID_ALPHA, str(exc))
    if alpha_value.value < 0:
        return _failure(
            ReferenceOutcome.REFERENCE_INVALID_ALPHA,
            "reference alpha must be non-negative",
        )
    frame, payload_diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    if frame is None:
        return ReferenceEvaluationResultV1(
            payload_diagnostics[0].outcome, None, payload_diagnostics
        )
    try:
        context = GeometryContext.build(compilation, frame)
        domain = build_domain_geometry(context)
        resolutions, boundary_diagnostics = resolve_component_alphas(
            context, alpha_value, domain
        )
        instances = []
        boundary_resolved = []
        for spec in sorted(
            compilation.envelope_specs, key=lambda item: item.envelope_spec_id.value
        ):
            component_ids = _component_ids_for_spec(compilation, spec)
            effective_values = {
                resolutions[item].effective_alpha.expression
                for item in component_ids
                if item in resolutions
            }
            if len(effective_values) > 1:
                return _failure(
                    ReferenceOutcome.SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN,
                    f"{spec.envelope_spec_id} has a non-uniform incident effective-alpha vector",
                    existing=boundary_diagnostics,
                )
            effective = (
                ExactScalar(next(iter(effective_values))).as_expr()
                if effective_values
                else sp.Rational(str(alpha_value.value))
            )
            if isinstance(spec, StripEnvelopeSpec):
                instance = evaluate_strip_envelope(context, spec, alpha_value, effective)
            elif isinstance(spec, AngularEnvelopeSpec):
                instance = evaluate_angular_envelope(context, spec, alpha_value, effective)
            elif isinstance(spec, JunctionEnvelopeSpec):
                instance = evaluate_junction_envelope(context, spec, alpha_value, effective)
            elif isinstance(spec, CapEnvelopeSpec):
                instance = evaluate_cap_envelope(context, spec, alpha_value, effective)
            else:  # pragma: no cover - EC1 union is closed and validated.
                raise TypeError(type(spec).__name__)
            component_resolutions = [
                resolutions[item] for item in component_ids if item in resolutions
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
            clipped, clip_outcome = _clip_instance_to_domain(
                instance, domain.face_regions, reachability
            )
            if clipped is None:
                return _failure(
                    clip_outcome or ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED,
                    f"{spec.envelope_spec_id} would require obstacle bypass",
                    existing=boundary_diagnostics,
                )
            capacity_outcomes = {
                item.capacity_outcome
                for item in component_resolutions
                if item.capacity_outcome is not None
            }
            capacity_outcome = (
                next(iter(capacity_outcomes)) if capacity_outcomes else None
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

        contribution_regions = tuple(
            region for instance in instances for region in instance.regions
        )
        reachability_by_instance = {
            item.envelope_instance.envelope_instance_id: item.reachability
            for item in boundary_resolved
        }
        union = REFERENCE_ARRANGEMENT_BACKEND.exact_union(
            contribution_regions,
            domain.face_regions,
            reachability_by_instance,
        )
        result = RawCoverageResultV1(
            schema_version=RAW_COVERAGE_RESULT_SCHEMA_V1,
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
        )
        digest = sha256(canonical_json_bytes(result)).hexdigest()
        result = replace(result, semantic_digest=digest)
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
    except CertifiedPredicateUndecidable as exc:
        return _failure(
            ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
            str(exc),
        )
