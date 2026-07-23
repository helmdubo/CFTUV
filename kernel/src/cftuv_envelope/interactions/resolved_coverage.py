"""Public RawCoverage -> exact ResolvedCoverage reference evaluation."""

from __future__ import annotations

from dataclasses import replace

from ..reference.boundary import build_domain_geometry
from ..reference.common import GeometryContext
from ..reference.contracts import (
    BoundaryResolvedEnvelopeV1,
    RawCoverageResultV1,
    ReferenceEnvelopeCompilationV1,
)
from ..reference.validation import validate_reference_geometry_payload
from .arrival import compile_arrival_models
from .candidates import generate_interaction_candidates
from .components import compile_interaction_components
from .contracts import (
    InteractionDiagnosticSeverity,
    InteractionDiagnosticV1,
    InteractionOutcome,
    InteractionResolutionResultV1,
    RESOLVED_COVERAGE_RESULT_SCHEMA_V1,
    ResolvedCoverageResultV1,
)
from .digest import resolved_coverage_semantic_digest
from .mutual_arrival import prove_mutual_arrivals
from .policy_b import apply_policy_b
from .validation import validate_interaction_inputs


_FATAL_OUTCOMES = frozenset(
    {
        InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
        InteractionOutcome.INTERACTION_JUNCTION_ARRIVAL_UNPROVEN,
        InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN,
        InteractionOutcome.SELF_CONTACT_READING_IDENTITY_UNPROVEN,
        InteractionOutcome.MULTIWAY_INTERACTION_POLICY_UNPROVEN,
        InteractionOutcome.INTERACTION_COINCIDENT_ARRIVAL_LAWS_UNPROVEN,
        InteractionOutcome.INTERACTION_MISSING_FRONT_READING,
        InteractionOutcome.INTERACTION_CAP_READING_AMBIGUOUS,
        InteractionOutcome.INTERACTION_EQUIVALENT_LOCUS_PROVENANCE_UNPROVEN,
        InteractionOutcome.INTERACTION_EXACT_PREDICATE_UNDECIDABLE,
    }
)


def _failure(
    outcome: InteractionOutcome,
    components=(),
    models=(),
    candidates=(),
    diagnostics=(),
) -> InteractionResolutionResultV1:
    return InteractionResolutionResultV1(
        outcome=outcome,
        resolved_coverage=None,
        interaction_components=tuple(components),
        arrival_models=tuple(models),
        candidates=tuple(candidates),
        diagnostics=tuple(diagnostics),
    )


def _first_fatal(
    diagnostics: tuple[InteractionDiagnosticV1, ...],
) -> InteractionOutcome | None:
    return next(
        (
            item.outcome
            for item in diagnostics
            if item.severity is InteractionDiagnosticSeverity.UNSUPPORTED
            and item.outcome in _FATAL_OUTCOMES
        ),
        None,
    )


def resolve_coverage_interactions(
    compilation: ReferenceEnvelopeCompilationV1,
    boundary_resolved_envelopes: (
        tuple[BoundaryResolvedEnvelopeV1, ...]
        | frozenset[BoundaryResolvedEnvelopeV1]
    ),
    raw_coverage: RawCoverageResultV1,
) -> InteractionResolutionResultV1:
    """Full-recompute exact EC2.5 resolver for one request/PatchDomain."""

    boundary_resolved = tuple(
        sorted(
            boundary_resolved_envelopes,
            key=lambda item: item.envelope_instance.envelope_instance_id,
        )
    )
    input_diagnostics = validate_interaction_inputs(
        compilation, boundary_resolved, raw_coverage
    )
    if input_diagnostics:
        return _failure(
            InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
            diagnostics=input_diagnostics,
        )
    components = compile_interaction_components(
        compilation, boundary_resolved
    )
    models, arrival_diagnostics = compile_arrival_models(
        compilation, components, boundary_resolved
    )
    fatal = _first_fatal(arrival_diagnostics)
    if fatal is not None:
        return _failure(
            fatal,
            components=components,
            models=models,
            diagnostics=arrival_diagnostics,
        )
    candidates = generate_interaction_candidates(
        components, models, compilation
    )
    proofs, mutual_diagnostics = prove_mutual_arrivals(candidates, models)
    fatal = _first_fatal(mutual_diagnostics)
    if fatal is not None:
        return _failure(
            fatal,
            components=components,
            models=models,
            candidates=candidates,
            diagnostics=(*arrival_diagnostics, *mutual_diagnostics),
        )
    frame, payload_diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    if frame is None:
        diagnostic = InteractionDiagnosticV1(
            InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
            InteractionDiagnosticSeverity.UNSUPPORTED,
            payload_diagnostics[0].message,
        )
        return _failure(
            diagnostic.outcome,
            components,
            models,
            candidates,
            (diagnostic,),
        )
    domain = build_domain_geometry(GeometryContext.build(compilation, frame))
    policy = apply_policy_b(
        proofs,
        components,
        boundary_resolved,
        raw_coverage,
        domain.face_regions,
    )
    diagnostics = (
        *arrival_diagnostics,
        *mutual_diagnostics,
        *policy.diagnostics,
    )
    fatal = _first_fatal(tuple(diagnostics))
    if fatal is not None:
        return _failure(
            fatal,
            components,
            models,
            candidates,
            diagnostics,
        )
    resolved = ResolvedCoverageResultV1(
        schema_version=RESOLVED_COVERAGE_RESULT_SCHEMA_V1,
        source_revision=raw_coverage.source_revision,
        plan_key=raw_coverage.plan_key,
        requested_alpha=raw_coverage.requested_alpha,
        raw_coverage_digest=raw_coverage.semantic_digest,
        resolved_contributions=policy.resolved_contributions,
        vertices=policy.resolved_union.vertices,
        edges=policy.resolved_union.edges,
        loops=policy.resolved_union.loops,
        regions=policy.resolved_union.regions,
        exact_area_expression=policy.resolved_union.exact_area_expression,
        interaction_applications=policy.applications,
        mutual_arrival_certificates=tuple(
            proof.mutual_arrival_certificate for proof in proofs
        ),
        equality_loci=tuple(
            proof.equality_locus for proof in proofs
        ),
        diagnostics=tuple(diagnostics),
        semantic_digest="",
    )
    resolved = replace(
        resolved,
        semantic_digest=resolved_coverage_semantic_digest(resolved).sha256_hex,
    )
    return InteractionResolutionResultV1(
        outcome=InteractionOutcome.EXACT,
        resolved_coverage=resolved,
        interaction_components=components,
        arrival_models=models,
        candidates=candidates,
        diagnostics=tuple(diagnostics),
    )
