"""Cross-stage input checks for EC2.5."""

from __future__ import annotations

from ..contracts.request import InteractionPolicyId
from ..reference.contracts import (
    BoundaryResolvedEnvelopeV1,
    RawCoverageResultV1,
    ReferenceEnvelopeCompilationV1,
)
from ..reference.digest import validate_raw_coverage_digest
from .contracts import (
    InteractionDiagnosticSeverity,
    InteractionDiagnosticV1,
    InteractionOutcome,
)


def validate_interaction_inputs(
    compilation: ReferenceEnvelopeCompilationV1,
    boundary_resolved_envelopes: tuple[BoundaryResolvedEnvelopeV1, ...],
    raw_coverage: RawCoverageResultV1,
) -> tuple[InteractionDiagnosticV1, ...]:
    issues = []
    if raw_coverage.source_revision != compilation.source_revision:
        issues.append("RawCoverage source revision differs from compilation")
    if raw_coverage.plan_key != compilation.plan_key:
        issues.append("RawCoverage plan key differs from compilation")
    if (
        compilation.decal_request.interaction_policy_id
        is not InteractionPolicyId.INTRAPATCH_POLICY_B_V1
    ):
        issues.append("request does not select INTRAPATCH_POLICY_B_V1")
    if not validate_raw_coverage_digest(raw_coverage):
        issues.append("RawCoverage semantic digest is invalid")
    supplied = frozenset(boundary_resolved_envelopes)
    if supplied != raw_coverage.boundary_resolved_envelopes:
        issues.append(
            "BoundaryResolvedEnvelope input differs from the immutable RawCoverage input"
        )
    instance_ids = {
        item.envelope_instance.envelope_instance_id
        for item in boundary_resolved_envelopes
    }
    raw_instance_ids = {
        item.envelope_instance_id for item in raw_coverage.envelope_instances
    }
    if instance_ids != raw_instance_ids:
        issues.append(
            "BoundaryResolvedEnvelope instances do not exactly match RawCoverage"
        )
    request_id = compilation.plan_key.decal_request_id.value
    domain_id = compilation.plan_key.patch_domain_id.value
    for spec in compilation.envelope_specs:
        if (
            spec.decal_request_id.value != request_id
            or spec.patch_domain_id.value != domain_id
        ):
            issues.append(
                f"EnvelopeSpec {spec.envelope_spec_id} escapes the plan request/domain"
            )
    return tuple(
        InteractionDiagnosticV1(
            InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
            InteractionDiagnosticSeverity.UNSUPPORTED,
            message,
        )
        for message in issues
    )
