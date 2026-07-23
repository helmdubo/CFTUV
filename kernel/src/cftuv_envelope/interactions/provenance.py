"""Interaction certificates keep source lineage separate from geometry history."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class InteractionProvenanceV1:
    decal_request_id: str
    patch_domain_id: str
    interaction_component_ids: frozenset[str]
    envelope_spec_ids: frozenset[str]
    envelope_instance_ids: frozenset[str]
    front_component_ids: frozenset[str]
    front_reading_ids: frozenset[str]
    source_lineage_ids: frozenset[str]
