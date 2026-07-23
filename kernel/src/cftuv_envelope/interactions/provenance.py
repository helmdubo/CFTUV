"""Interaction certificates keep source lineage separate from geometry history."""

from __future__ import annotations

from dataclasses import dataclass

from ..ids import (
    DecalRequestId,
    EnvelopeInstanceId,
    EnvelopeSpecId,
    FrontComponentId,
    FrontReadingId,
    InteractionComponentId,
    LineageId,
    PatchDomainId,
)


@dataclass(frozen=True, slots=True)
class InteractionProvenanceV1:
    decal_request_id: DecalRequestId
    patch_domain_id: PatchDomainId
    interaction_component_ids: frozenset[InteractionComponentId]
    envelope_spec_ids: frozenset[EnvelopeSpecId]
    envelope_instance_ids: frozenset[EnvelopeInstanceId]
    front_component_ids: frozenset[FrontComponentId]
    front_reading_ids: frozenset[FrontReadingId]
    source_lineage_ids: frozenset[LineageId]
