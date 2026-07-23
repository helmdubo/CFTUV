"""Topology-first interaction candidate generation."""

from __future__ import annotations

from itertools import combinations

from ..reference.common import stable_id
from ..reference.contracts import ReferenceEnvelopeCompilationV1
from .contracts import (
    ArrivalModelV1,
    InteractionCandidateKind,
    InteractionCandidateV1,
    InteractionComponentV1,
)
from .provenance import InteractionProvenanceV1


def _candidate_provenance(
    left: InteractionComponentV1,
    right: InteractionComponentV1,
    models: tuple[ArrivalModelV1, ...],
) -> InteractionProvenanceV1:
    component_ids = frozenset(
        {left.interaction_component_id, right.interaction_component_id}
    )
    participants = tuple(
        item
        for item in models
        if item.interaction_component_id in component_ids
        and hasattr(item, "front_reading_id")
    )
    return InteractionProvenanceV1(
        decal_request_id=left.decal_request_id,
        patch_domain_id=left.patch_domain_id,
        interaction_component_ids=component_ids,
        envelope_spec_ids=left.envelope_spec_ids | right.envelope_spec_ids,
        envelope_instance_ids=(
            left.envelope_instance_ids | right.envelope_instance_ids
        ),
        front_component_ids=(
            left.front_component_ids | right.front_component_ids
        ),
        front_reading_ids=frozenset(
            item.front_reading_id for item in participants
        ),
        source_lineage_ids=(
            left.source_lineage_ids | right.source_lineage_ids
        ),
    )


def generate_interaction_candidates(
    components: tuple[InteractionComponentV1, ...],
    arrival_models: tuple[ArrivalModelV1, ...],
    compilation: ReferenceEnvelopeCompilationV1 | None = None,
) -> tuple[InteractionCandidateV1, ...]:
    """Create only same-request/same-domain pairs plus declared self-contact."""

    candidates = []
    for left, right in combinations(
        sorted(components, key=lambda item: item.interaction_component_id), 2
    ):
        if left.decal_request_id != right.decal_request_id:
            continue
        if left.patch_domain_id != right.patch_domain_id:
            continue
        candidates.append(
            InteractionCandidateV1(
                candidate_id=stable_id(
                    "interaction-candidate",
                    left.interaction_component_id,
                    right.interaction_component_id,
                ),
                left_component_id=left.interaction_component_id,
                right_component_id=right.interaction_component_id,
                candidate_kind=InteractionCandidateKind.DISTINCT_COMPONENTS,
                decal_request_id=left.decal_request_id,
                patch_domain_id=left.patch_domain_id,
                source_provenance=_candidate_provenance(
                    left, right, arrival_models
                ),
            )
        )

    if compilation is not None:
        declared_self_fronts = {
            item.front_component_id.value
            for item in compilation.front_components
            if len(item.front_reading_ids) >= 2
        }
        for component in components:
            if not (component.front_component_ids & declared_self_fronts):
                continue
            candidates.append(
                InteractionCandidateV1(
                    candidate_id=stable_id(
                        "self-contact-candidate",
                        component.interaction_component_id,
                        *sorted(declared_self_fronts & component.front_component_ids),
                    ),
                    left_component_id=component.interaction_component_id,
                    right_component_id=component.interaction_component_id,
                    candidate_kind=InteractionCandidateKind.EXPLICIT_SELF_CONTACT,
                    decal_request_id=component.decal_request_id,
                    patch_domain_id=component.patch_domain_id,
                    source_provenance=_candidate_provenance(
                        component, component, arrival_models
                    ),
                )
            )
    return tuple(sorted(candidates, key=lambda item: item.candidate_id))
