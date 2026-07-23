"""Topology-first interaction candidate generation."""

from __future__ import annotations

from itertools import combinations

from ..ids import FrontReadingId, InteractionCandidateId
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
    participant_reading_ids: frozenset[FrontReadingId] | None = None,
) -> InteractionProvenanceV1:
    component_ids = frozenset(
        {left.interaction_component_id, right.interaction_component_id}
    )
    participants = tuple(
        item
        for item in models
        if item.interaction_component_id in component_ids
        and hasattr(item, "front_reading_id")
        and (
            participant_reading_ids is None
            or item.front_reading_id in participant_reading_ids
        )
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
    declared_self_components = []
    if compilation is not None:
        component_by_front = {
            front_component_id: component
            for component in components
            for front_component_id in component.front_component_ids
        }
        for declaration in sorted(
            compilation.self_contact_pair_declarations,
            key=lambda item: item.declaration_id.value,
        ):
            left_component = component_by_front.get(
                declaration.left_front_component_id
            )
            right_component = component_by_front.get(
                declaration.right_front_component_id
            )
            if left_component is None or right_component is None:
                continue
            declared_self_components.append(
                (declaration, left_component, right_component)
            )
    declared_component_pairs = {
        frozenset(
            {
                left.interaction_component_id,
                right.interaction_component_id,
            }
        )
        for _, left, right in declared_self_components
        if left.interaction_component_id != right.interaction_component_id
    }
    for left, right in combinations(
        sorted(
            components,
            key=lambda item: item.interaction_component_id.value,
        ),
        2,
    ):
        if left.decal_request_id != right.decal_request_id:
            continue
        if left.patch_domain_id != right.patch_domain_id:
            continue
        if (
            frozenset(
                {
                    left.interaction_component_id,
                    right.interaction_component_id,
                }
            )
            in declared_component_pairs
        ):
            continue
        candidates.append(
            InteractionCandidateV1(
                candidate_id=InteractionCandidateId(stable_id(
                    "interaction-candidate",
                    left.interaction_component_id,
                    right.interaction_component_id,
                )),
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
        for declaration, left_component, right_component in (
            declared_self_components
        ):
            participant_readings = (
                declaration.left_front_reading_id,
                declaration.right_front_reading_id,
            )
            candidates.append(
                InteractionCandidateV1(
                    candidate_id=InteractionCandidateId(stable_id(
                        "self-contact-candidate",
                        left_component.interaction_component_id,
                        right_component.interaction_component_id,
                        declaration.declaration_id,
                    )),
                    left_component_id=left_component.interaction_component_id,
                    right_component_id=right_component.interaction_component_id,
                    candidate_kind=InteractionCandidateKind.EXPLICIT_SELF_CONTACT,
                    decal_request_id=left_component.decal_request_id,
                    patch_domain_id=left_component.patch_domain_id,
                    source_provenance=_candidate_provenance(
                        left_component,
                        right_component,
                        arrival_models,
                        frozenset(participant_readings),
                    ),
                    self_contact_pair_declaration_id=(
                        declaration.declaration_id
                    ),
                    participant_front_reading_ids=participant_readings,
                )
            )
    return tuple(
        sorted(candidates, key=lambda item: item.candidate_id.value)
    )
