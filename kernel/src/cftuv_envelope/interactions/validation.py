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
    front_components = {
        item.front_component_id: item
        for item in compilation.front_components
    }
    declarations_by_reading = {}
    declarations_by_support = {}
    for declaration in compilation.front_reading_declarations:
        declarations_by_reading.setdefault(
            declaration.front_reading_id, []
        ).append(declaration)
        declarations_by_support.setdefault(
            (
                declaration.front_component_id,
                declaration.source_support_id,
            ),
            [],
        ).append(declaration)
        component = front_components.get(declaration.front_component_id)
        if component is None:
            issues.append(
                f"FrontReading {declaration.front_reading_id} references "
                "an absent FrontComponent"
            )
            continue
        if (
            declaration.chain_use_id != component.chain_use_id
            or declaration.owner_sector_id != component.sector_id
        ):
            issues.append(
                f"FrontReading {declaration.front_reading_id} escapes its "
                "FrontComponent ChainUse/OwnerSector"
            )
    if any(len(items) != 1 for items in declarations_by_reading.values()):
        issues.append("FrontReading identity is not unique")
    if any(len(items) != 1 for items in declarations_by_support.values()):
        issues.append("FrontComponent source support has ambiguous readings")
    for component in compilation.front_components:
        compiled_readings = frozenset(
            declaration.front_reading_id
            for declaration in compilation.front_reading_declarations
            if declaration.front_component_id
            == component.front_component_id
        )
        if component.front_reading_ids != compiled_readings:
            issues.append(
                f"FrontComponent {component.front_component_id} reading IDs "
                "differ from FrontReadingDeclarationV1"
            )
    seen_self_pairs = set()
    uses = {
        item.chain_use_id: item
        for item in compilation.analysis_snapshot.chain_uses
    }
    chains = {
        item.physical_chain_id: item
        for item in compilation.analysis_snapshot.physical_chains
    }
    for declaration in compilation.self_contact_pair_declarations:
        pair = frozenset(
            {
                declaration.left_front_reading_id,
                declaration.right_front_reading_id,
            }
        )
        if len(pair) != 2:
            issues.append(
                f"SelfContactPair {declaration.declaration_id} does not "
                "contain two distinct readings"
            )
            continue
        readings = tuple(
            declarations_by_reading.get(reading_id, ())
            for reading_id in pair
        )
        if any(len(items) != 1 for items in readings):
            issues.append(
                f"SelfContactPair {declaration.declaration_id} references "
                "an absent or ambiguous reading"
            )
            continue
        left_reading = declarations_by_reading[
            declaration.left_front_reading_id
        ][0]
        right_reading = declarations_by_reading[
            declaration.right_front_reading_id
        ][0]
        if (
            left_reading.front_component_id
            != declaration.left_front_component_id
            or right_reading.front_component_id
            != declaration.right_front_component_id
        ):
            issues.append(
                f"SelfContactPair {declaration.declaration_id} component "
                "identities differ from its FrontReadings"
            )
        left_lineage = chains[
            uses[left_reading.chain_use_id].physical_chain_id
        ].source_lineage
        right_lineage = chains[
            uses[right_reading.chain_use_id].physical_chain_id
        ].source_lineage
        if (
            not declaration.source_lineage_ids
            or declaration.source_lineage_ids != left_lineage
            or declaration.source_lineage_ids != right_lineage
        ):
            issues.append(
                f"SelfContactPair {declaration.declaration_id} lacks one "
                "identical source lineage"
            )
        canonical_pair = (
            pair,
        )
        if canonical_pair in seen_self_pairs:
            issues.append("duplicate SelfContactPair declaration")
        seen_self_pairs.add(canonical_pair)
    return tuple(
        InteractionDiagnosticV1(
            InteractionOutcome.INTERACTION_INPUT_CONTRACT_INVALID,
            InteractionDiagnosticSeverity.UNSUPPORTED,
            message,
        )
        for message in issues
    )
