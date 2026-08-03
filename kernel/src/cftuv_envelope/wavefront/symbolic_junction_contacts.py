"""Stable translation and discovery for mixed junction contacts."""

from __future__ import annotations

from dataclasses import dataclass

from .events import EventKind
from . import superlevel as base
from .superlevel_closure import SpanFamilyRefV1
from .symbolic_edge_closure import (
    SymbolicEdgeContactKeyV1,
    SymbolicEdgeContactV1,
    discover_symbolic_edge_contacts,
)
from .symbolic_overlay import JunctionRefV1
from .symbolic_split_endpoint import (
    EndpointContactKeyV1,
    discover_endpoint_contacts,
)


@dataclass(frozen=True, slots=True)
class SymbolicJunctionContactV1:
    kind: str
    key: object
    dead_refs: tuple[JunctionRefV1, ...]
    families: frozenset
    edge: SymbolicEdgeContactV1 | None = None


def edge_contact(contact):
    key = contact.key
    return SymbolicJunctionContactV1(
        "EDGE",
        key,
        tuple(sorted((key.start, key.end), key=repr)),
        frozenset((key.prev_family, key.shared_family, key.next_family)),
        contact,
    )


def endpoint_contact(overlay, key):
    families = {key.family}
    for ref in (key.emitter, key.endpoint):
        vertex = overlay.vertices.get(ref)
        if vertex is not None:
            families.update((vertex.prev_leaf.family, vertex.next_leaf.family))
    return SymbolicJunctionContactV1(
        "ENDPOINT",
        key,
        tuple(sorted((key.emitter, key.endpoint), key=repr)),
        frozenset(families),
    )


def contact_identity(contact):
    return contact.kind, contact.key


def discover_junction_contacts(builder, overlay):
    endpoints, reason = discover_endpoint_contacts(builder, overlay)
    if reason is not None:
        return (), reason
    contacts = tuple(
        edge_contact(item)
        for item in discover_symbolic_edge_contacts(builder, overlay)
    ) + tuple(endpoint_contact(overlay, item) for item in endpoints)
    unique = {contact_identity(item): item for item in contacts}
    return tuple(sorted(
        unique.values(), key=lambda item: repr(contact_identity(item))
    )), None


def initial_junction_seeds(snapshot, overlay):
    """Translate drained EDGE and endpoint SPLIT incidents onto raw F0."""

    seeds = []
    incidents = tuple(sorted(
        snapshot.incidents,
        key=lambda item: (
            item.event.kind is not EventKind.EDGE,
            base._incident_sort_key(item),
        ),
    ))
    for incident in incidents:
        if incident.event.kind is EventKind.EDGE:
            start = JunctionRefV1("EXISTING", incident.emitter_key)
            end = JunctionRefV1("EXISTING", incident.peer_key)
            first, second = overlay.vertices.get(start), overlay.vertices.get(end)
            if first is None or second is None:
                return (), "SYMBOLIC_INITIAL_EDGE_BINDING_UNRESOLVABLE"
            key = SymbolicEdgeContactKeyV1(
                base._time_key(incident.event.time), incident.point_key,
                start, end, first.prev_leaf.family, first.next_leaf.family,
                second.next_leaf.family,
            )
            seeds.append(edge_contact(SymbolicEdgeContactV1(
                key, first.prev_leaf, first.next_leaf, second.next_leaf,
                incident.event.span_unproven, incident.participants,
            )))
            continue
        occurrence = incident.target_occurrence
        if (incident.event.kind is not EventKind.SPLIT
                or occurrence is None
                or incident.point_key not in occurrence[1:]):
            continue
        family = SpanFamilyRefV1(occurrence, (occurrence[0],))
        emitter = JunctionRefV1("EXISTING", incident.emitter_key)
        endpoints = set()
        for leaf, binding in overlay.spans.items():
            if leaf.family != family:
                continue
            if (incident.point_key == leaf.occurrence[1]
                    and binding.start is not None):
                endpoints.add(binding.start)
            if (incident.point_key == leaf.occurrence[2]
                    and binding.end is not None):
                endpoints.add(binding.end)
        if not endpoints or emitter not in overlay.vertices:
            return (), "SYMBOLIC_INITIAL_ENDPOINT_BINDING_UNRESOLVABLE"
        for endpoint in endpoints:
            key = EndpointContactKeyV1(
                base._time_key(incident.event.time), incident.point_key,
                emitter, endpoint, family, incident.participants,
            )
            seeds.append(endpoint_contact(overlay, key))
    unique = {contact_identity(item): item for item in seeds}
    return tuple(sorted(
        unique.values(), key=lambda item: repr(contact_identity(item))
    )), None
