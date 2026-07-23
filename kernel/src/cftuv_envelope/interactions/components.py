"""Compile topological interaction components without spatial clustering."""

from __future__ import annotations

from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    CapEnvelopeSpec,
    JunctionEnvelopeSpec,
    StripEnvelopeSpec,
)
from ..reference.common import stable_id
from ..reference.contracts import (
    BoundaryResolvedEnvelopeV1,
    ReferenceEnvelopeCompilationV1,
)
from ..ids import (
    EnvelopeInstanceId,
    EnvelopeSpecId,
    FrontComponentId,
    InteractionComponentId,
    InteractionRelationId,
    LineageId,
)
from .contracts import InteractionComponentV1


def _component_ids_for_spec(spec) -> frozenset[str]:
    if isinstance(spec, StripEnvelopeSpec):
        return frozenset(item.value for item in spec.front_component_ids)
    if isinstance(spec, AngularEnvelopeSpec):
        return frozenset(item.value for item in spec.incident_front_component_ids)
    if isinstance(spec, JunctionEnvelopeSpec):
        return frozenset(item.value for item in spec.incident_front_component_ids)
    return frozenset()


def _relation_ids_for_spec(spec) -> frozenset[str]:
    if isinstance(spec, AngularEnvelopeSpec):
        return frozenset({spec.source_relation_id.value})
    if isinstance(spec, JunctionEnvelopeSpec):
        return frozenset({spec.source_relation_id.value})
    if isinstance(spec, CapEnvelopeSpec) and spec.terminal_relation_id is not None:
        return frozenset({spec.terminal_relation_id.value})
    return frozenset()


def compile_interaction_components(
    compilation: ReferenceEnvelopeCompilationV1,
    boundary_resolved_envelopes: tuple[BoundaryResolvedEnvelopeV1, ...],
) -> tuple[InteractionComponentV1, ...]:
    """Group incident Strip/Angular/Junction/Cap specs by declared topology."""

    specs = {
        item.envelope_spec_id.value: item for item in compilation.envelope_specs
    }
    parent = {spec_id: spec_id for spec_id in specs}

    def find(item: str) -> str:
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    def union(left: str, right: str) -> None:
        if left not in parent or right not in parent:
            return
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        if left_root < right_root:
            parent[right_root] = left_root
        else:
            parent[left_root] = right_root

    front_to_specs: dict[str, list[str]] = {}
    for spec_id, spec in specs.items():
        for front_component_id in _component_ids_for_spec(spec):
            front_to_specs.setdefault(front_component_id, []).append(spec_id)
        if isinstance(spec, StripEnvelopeSpec):
            for terminal_id in spec.terminal_interface_spec_ids:
                union(spec_id, terminal_id.value)
        if isinstance(spec, CapEnvelopeSpec):
            union(spec_id, spec.incident_strip_spec_id.value)

    for linked_specs in front_to_specs.values():
        if not linked_specs:
            continue
        for spec_id in linked_specs[1:]:
            union(linked_specs[0], spec_id)

    groups: dict[str, set[str]] = {}
    for spec_id in specs:
        groups.setdefault(find(spec_id), set()).add(spec_id)

    instances_by_spec: dict[str, set[str]] = {}
    for resolved in boundary_resolved_envelopes:
        instance = resolved.envelope_instance
        instances_by_spec.setdefault(instance.envelope_spec_id, set()).add(
            instance.envelope_instance_id
        )

    request_id = compilation.plan_key.decal_request_id
    domain_id = compilation.plan_key.patch_domain_id
    components = []
    for spec_ids in groups.values():
        sorted_spec_ids = tuple(sorted(spec_ids))
        component_id = InteractionComponentId(stable_id(
            "interaction-component", request_id, domain_id, *sorted_spec_ids
        ))
        components.append(
            InteractionComponentV1(
                interaction_component_id=component_id,
                decal_request_id=request_id,
                patch_domain_id=domain_id,
                envelope_spec_ids=frozenset(
                    EnvelopeSpecId(item) for item in spec_ids
                ),
                envelope_instance_ids=frozenset(
                    EnvelopeInstanceId(instance_id)
                    for spec_id in spec_ids
                    for instance_id in instances_by_spec.get(spec_id, ())
                ),
                front_component_ids=frozenset(
                    FrontComponentId(front_component_id)
                    for spec_id in spec_ids
                    for front_component_id in _component_ids_for_spec(specs[spec_id])
                ),
                relation_ids=frozenset(
                    InteractionRelationId(relation_id)
                    for spec_id in spec_ids
                    for relation_id in _relation_ids_for_spec(specs[spec_id])
                ),
                source_lineage_ids=frozenset(
                    LineageId(lineage_id.value)
                    for spec_id in spec_ids
                    for lineage_id in specs[spec_id].source_lineage_ids
                ),
            )
        )
    return tuple(
        sorted(
            components,
            key=lambda item: item.interaction_component_id.value,
        )
    )
