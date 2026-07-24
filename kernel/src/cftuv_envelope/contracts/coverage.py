"""Декларативные coverage и interaction records без Boolean implementation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..ids import (
    CoverageId,
    DecalRequestId,
    EnvelopeInstanceId,
    EventPredicateId,
    InteractionId,
    LineageId,
    PatchDomainId,
)


class RawCoverageStage(str, Enum):
    AFTER_BOUNDARY_RESOLVED_ENVELOPE_INSTANCES = (
        "AFTER_BOUNDARY_RESOLVED_ENVELOPE_INSTANCES"
    )


class ResolvedCoverageStage(str, Enum):
    AFTER_APPROVED_INTERACTIONS = "AFTER_APPROVED_INTERACTIONS"


@dataclass(frozen=True, slots=True)
class RawCoverageRef:
    coverage_id: CoverageId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    stage: RawCoverageStage
    envelope_instance_ids: frozenset[EnvelopeInstanceId]


@dataclass(frozen=True, slots=True)
class ResolvedCoverageRef:
    coverage_id: CoverageId
    raw_coverage_id: CoverageId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    stage: ResolvedCoverageStage
    interaction_ids: frozenset[InteractionId]


class InteractionKind(str, Enum):
    INTRAPATCH_POLICY_B = "INTRAPATCH_POLICY_B"
    ANGULAR_PROFILE_FRONT_CONTACT = "ANGULAR_PROFILE_FRONT_CONTACT"


class CoverageEffect(str, Enum):
    NONE = "NONE"
    CLIP_EXISTING_CONTRIBUTIONS_AT_EQUALITY_LOCUS = (
        "CLIP_EXISTING_CONTRIBUTIONS_AT_EQUALITY_LOCUS"
    )
    CLIP_TO_EXPOSED_ANGULAR_PROFILE_BOUNDARY = (
        "CLIP_TO_EXPOSED_ANGULAR_PROFILE_BOUNDARY"
    )


@dataclass(frozen=True, slots=True)
class InteractionDeclarationV1:
    interaction_id: InteractionId
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    kind: InteractionKind
    participant_envelope_instance_ids: frozenset[EnvelopeInstanceId]
    event_gate_id: EventPredicateId
    coverage_effect: CoverageEffect
    creates_new_matter: bool
    provenance: frozenset[LineageId]

