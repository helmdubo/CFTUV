"""Provenance values propagated by segment history, never reconstructed."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ReferenceProvenanceV1:
    envelope_spec_ids: frozenset[str] = frozenset()
    envelope_instance_ids: frozenset[str] = frozenset()
    support_ids: frozenset[str] = frozenset()
    physical_edge_ids: frozenset[str] = frozenset()
    source_face_ids: frozenset[str] = frozenset()
    chain_use_ids: frozenset[str] = frozenset()
    patch_domain_ids: frozenset[str] = frozenset()
    boundary_constraint_ids: frozenset[str] = frozenset()
    lineage_ids: frozenset[str] = frozenset()


def merge_provenance(*items: ReferenceProvenanceV1) -> ReferenceProvenanceV1:
    return ReferenceProvenanceV1(
        envelope_spec_ids=frozenset().union(*(item.envelope_spec_ids for item in items)),
        envelope_instance_ids=frozenset().union(
            *(item.envelope_instance_ids for item in items)
        ),
        support_ids=frozenset().union(*(item.support_ids for item in items)),
        physical_edge_ids=frozenset().union(
            *(item.physical_edge_ids for item in items)
        ),
        source_face_ids=frozenset().union(*(item.source_face_ids for item in items)),
        chain_use_ids=frozenset().union(*(item.chain_use_ids for item in items)),
        patch_domain_ids=frozenset().union(
            *(item.patch_domain_ids for item in items)
        ),
        boundary_constraint_ids=frozenset().union(
            *(item.boundary_constraint_ids for item in items)
        ),
        lineage_ids=frozenset().union(*(item.lineage_ids for item in items)),
    )
