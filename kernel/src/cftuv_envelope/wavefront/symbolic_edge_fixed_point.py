"""Pure causal EDGE generations rebuilt from one frozen symbolic overlay."""

from __future__ import annotations

from dataclasses import dataclass

from .symbolic_component import (
    SymbolicComponentDeltaV1,
    clone_overlay as _clone,
    delta_resources,
    normalize_dead_component,
    overlay_signature,
    point_from_key as _point,
)
from .symbolic_edge_closure import (
    SymbolicEdgeContactV1,
    discover_symbolic_edge_contacts,
)
from .symbolic_overlay import (
    SymbolicOverlayV1,
    SymbolicVertexV1,
    refreshed_span_bindings,
)


@dataclass(frozen=True, slots=True)
class SymbolicEdgeGenerationV1:
    contacts: tuple[SymbolicEdgeContactV1, ...]
    deltas: tuple[SymbolicComponentDeltaV1, ...]


@dataclass(frozen=True, slots=True)
class SymbolicEdgeFixedPointV1:
    generations: tuple[SymbolicEdgeGenerationV1, ...]
    overlay: SymbolicOverlayV1 | None
    signatures: tuple[tuple, ...]
    unresolved_reason: str | None = None


def _resources(contact):
    return frozenset(
        (("VERTEX", contact.key.start), ("VERTEX", contact.key.end),
         ("LEAF", contact.prev_leaf), ("LEAF", contact.shared_leaf),
         ("LEAF", contact.next_leaf))
    )


def _components(contacts):
    pending = list(sorted(contacts, key=lambda item: repr(item.key)))
    result = []
    while pending:
        component = [pending.pop(0)]
        resources = set(_resources(component[0]))
        time_point = (component[0].key.time_key, component[0].key.point_key)
        changed = True
        while changed:
            changed = False
            rest = []
            for contact in pending:
                same = (contact.key.time_key, contact.key.point_key) == time_point
                if same and resources.intersection(_resources(contact)):
                    component.append(contact)
                    resources.update(_resources(contact))
                    changed = True
                else:
                    rest.append(contact)
            pending = rest
        result.append(tuple(sorted(component, key=lambda item: repr(item.key))))
    return tuple(result)


def _valid_contact(overlay, contact):
    start = overlay.vertices.get(contact.key.start)
    end = overlay.vertices.get(contact.key.end)
    return bool(
        start is not None and end is not None and start.alive and end.alive
        and start.next == end.ref and end.prev == start.ref
        and (start.prev_leaf, start.next_leaf, end.next_leaf)
        == (contact.prev_leaf, contact.shared_leaf, contact.next_leaf)
        and (start.prev_leaf.family, start.next_leaf.family, end.next_leaf.family)
        == (contact.key.prev_family, contact.key.shared_family,
            contact.key.next_family)
    )


def _component_delta(overlay, contacts):
    if not all(_valid_contact(overlay, contact) for contact in contacts):
        return None, "SYMBOLIC_EDGE_CONTACT_STALE"
    dead = {ref for contact in contacts for ref in (contact.key.start, contact.key.end)}
    keys = tuple(sorted((contact.key for contact in contacts), key=repr))
    return normalize_dead_component(
        overlay,
        contact_keys=keys,
        dead_refs=dead,
        point_key=contacts[0].key.point_key,
        birth_kind="EDGE_JUNCTION",
        stale_reason="SYMBOLIC_EDGE_CONTACT_STALE",
        ambiguity_reason="SYMBOLIC_EDGE_PORT_MATCHING_AMBIGUOUS",
    )


def normalize_edge_generation(overlay, contacts):
    ordered = tuple(sorted(contacts, key=lambda item: repr(item.key)))
    if len({contact.key for contact in ordered}) != len(ordered):
        return None, "SYMBOLIC_EDGE_DUPLICATE_CONTACT"
    deltas = []
    occupied = set()
    for component in _components(ordered):
        delta, reason = _component_delta(overlay, component)
        if reason is not None:
            return None, reason
        ports = delta_resources(delta)
        if occupied.intersection(ports):
            return None, "SYMBOLIC_EDGE_COMPONENT_DELTAS_OVERLAP"
        occupied.update(ports)
        deltas.append(delta)
    return SymbolicEdgeGenerationV1(
        ordered, tuple(sorted(deltas, key=lambda item: repr(item.contact_keys)))
    ), None


def apply_edge_generation(overlay, generation):
    result = _clone(overlay)
    if any(delta.birth_ref in result.vertices for delta in generation.deltas
           if delta.birth_ref is not None):
        return None, "SYMBOLIC_EDGE_BIRTH_COLLISION"
    for delta in generation.deltas:
        for ref in delta.dead_refs:
            result.vertices[ref].alive = False
    for delta in generation.deltas:
        if delta.birth_ref is None:
            continue
        predecessor, prev_leaf = delta.incoming
        successor, next_leaf = delta.outgoing
        provenance = frozenset().union(
            *(result.vertices[ref].provenance for ref in delta.dead_refs)
        )
        result.vertices[delta.birth_ref] = SymbolicVertexV1(
            delta.birth_ref, predecessor, successor, prev_leaf, next_leaf,
            result.time, _point(delta.point_key), None, provenance,
        )
        result.vertices[predecessor].next = delta.birth_ref
        result.vertices[successor].prev = delta.birth_ref
        result.changed.update((prev_leaf, next_leaf))
    bindings, reason = refreshed_span_bindings(result)
    if reason is not None:
        return None, reason
    result.spans = bindings
    return result, None


def _rebuild(initial, causal_contacts):
    overlay, generations, signatures = _clone(initial), [], []
    for contacts in causal_contacts:
        generation, reason = normalize_edge_generation(overlay, contacts)
        if reason is not None:
            return None, tuple(generations), tuple(signatures), reason
        overlay, reason = apply_edge_generation(overlay, generation)
        if reason is not None:
            return None, tuple(generations), tuple(signatures), reason
        generations.append(generation)
        signatures.append(overlay_signature(overlay))
    return overlay, tuple(generations), tuple(signatures), None


def plan_symbolic_edge_generations(builder, initial, *, budget):
    causal = []
    for iteration in range(budget + 1):
        overlay, generations, signatures, reason = _rebuild(initial, causal)
        if reason is not None:
            return SymbolicEdgeFixedPointV1(generations, None, signatures, reason)
        contacts = discover_symbolic_edge_contacts(builder, overlay)
        if not contacts:
            return SymbolicEdgeFixedPointV1(generations, overlay, signatures)
        generation, reason = normalize_edge_generation(overlay, contacts)
        if reason is not None:
            return SymbolicEdgeFixedPointV1(generations, overlay, signatures, reason)
        if iteration == budget:
            return SymbolicEdgeFixedPointV1(
                generations, overlay, signatures,
                "SYMBOLIC_EDGE_GENERATION_BUDGET_EXHAUSTED",
            )
        causal.append(generation.contacts)
    raise AssertionError("unreachable symbolic EDGE generation loop")
