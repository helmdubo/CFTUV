"""Pure endpoint-to-junction generations; never compile an interior cut."""

from __future__ import annotations

from dataclasses import dataclass

from .candidate_law import evaluate_split_candidate
from .event_time import compare_times
from . import superlevel as base
from .superlevel_closure import SpanFamilyRefV1
from .symbolic_component import (
    SymbolicComponentDeltaV1,
    clone_overlay as _clone,
    delta_resources,
    normalize_dead_component,
    overlay_signature,
    point_from_key as _point,
)
from .symbolic_overlay import (
    JunctionRefV1,
    SymbolicOverlayV1,
    SymbolicVertexV1,
    exact_overlay_view,
    is_symbolic_split_emitter,
    refreshed_span_bindings,
)


@dataclass(frozen=True, slots=True)
class EndpointContactKeyV1:
    time_key: tuple
    point_key: tuple
    emitter: JunctionRefV1
    endpoint: JunctionRefV1
    family: SpanFamilyRefV1
    participants: tuple[tuple[int, ...], ...]

@dataclass(frozen=True, slots=True)
class EndpointGenerationV1:
    contacts: tuple[EndpointContactKeyV1, ...]
    deltas: tuple[SymbolicComponentDeltaV1, ...]

@dataclass(frozen=True, slots=True)
class EndpointFixedPointV1:
    generations: tuple[EndpointGenerationV1, ...]
    overlay: SymbolicOverlayV1 | None
    signatures: tuple[tuple, ...]
    unresolved_reason: str | None = None

def _participants(*leaves):
    return tuple(sorted({participant for leaf in leaves
                         for participant in leaf.family.participant_keys}))

def discover_endpoint_contacts(builder, overlay):
    view = exact_overlay_view(builder, overlay)
    contacts = {}
    emitters = tuple(
        vertex for vertex in overlay.vertices.values()
        if vertex.alive
        and is_symbolic_split_emitter(builder, overlay, vertex)
    )
    for leaf in sorted(overlay.changed, key=repr):
        binding = overlay.spans[leaf]
        for emitter in sorted(emitters, key=lambda item: repr(item.ref)):
            if leaf in (emitter.prev_leaf, emitter.next_leaf):
                continue
            decision = evaluate_split_candidate(
                view, emitter.ref, leaf, now=overlay.time
            )
            candidate = decision.candidate
            if (candidate is None
                    or compare_times(candidate.time, overlay.time) != 0
                    or not (candidate.at_start or candidate.at_end)):
                continue
            endpoints = {
                endpoint for flag, endpoint in (
                    (candidate.at_start, binding.start),
                    (candidate.at_end, binding.end),
                ) if flag and endpoint is not None
            }
            if not endpoints:
                return (), "SYMBOLIC_ENDPOINT_REFERENCE_AMBIGUOUS"
            for endpoint in endpoints:
                key = EndpointContactKeyV1(
                    base._time_key(candidate.time),
                    (candidate.point.x.terms, candidate.point.y.terms),
                    emitter.ref,
                    endpoint,
                    leaf.family,
                    _participants(emitter.prev_leaf, emitter.next_leaf, leaf),
                )
                contacts[key] = key
    return tuple(sorted(contacts, key=repr)), None

def _resources(contact):
    return {contact.emitter, contact.endpoint, contact.family}


def _components(contacts):
    pending = list(sorted(contacts, key=repr))
    result = []
    while pending:
        component = [pending.pop(0)]
        resources = _resources(component[0])
        time_point = (component[0].time_key, component[0].point_key)
        changed = True
        while changed:
            changed = False
            rest = []
            for contact in pending:
                same = (contact.time_key, contact.point_key) == time_point
                if same and resources.intersection(_resources(contact)):
                    component.append(contact)
                    resources.update(_resources(contact))
                    changed = True
                else:
                    rest.append(contact)
            pending = rest
        result.append(tuple(sorted(component, key=repr)))
    return tuple(result)


def _delta(overlay, contacts):
    dead = {ref for contact in contacts
            for ref in (contact.emitter, contact.endpoint)}
    keys = tuple(sorted(contacts, key=repr))
    return normalize_dead_component(
        overlay,
        contact_keys=keys,
        dead_refs=dead,
        point_key=contacts[0].point_key,
        birth_kind="ENDPOINT_JUNCTION",
        stale_reason="SYMBOLIC_ENDPOINT_CONTACT_STALE",
        ambiguity_reason="SYMBOLIC_ENDPOINT_PORT_MATCHING_AMBIGUOUS",
    )


def normalize_endpoint_generation(overlay, contacts):
    ordered = tuple(sorted(set(contacts), key=repr))
    deltas = []
    occupied = set()
    for component in _components(ordered):
        delta, reason = _delta(overlay, component)
        if reason is not None:
            return None, reason
        resources = delta_resources(delta)
        if occupied.intersection(resources):
            return None, "SYMBOLIC_ENDPOINT_COMPONENT_DELTAS_OVERLAP"
        occupied.update(resources)
        deltas.append(delta)
    return EndpointGenerationV1(
        ordered, tuple(sorted(deltas, key=lambda item: repr(item.contact_keys)))
    ), None


def apply_endpoint_generation(overlay, generation):
    result = _clone(overlay)
    if any(delta.birth_ref in result.vertices for delta in generation.deltas
           if delta.birth_ref is not None):
        return None, "SYMBOLIC_ENDPOINT_BIRTH_COLLISION"
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
        generation, reason = normalize_endpoint_generation(overlay, contacts)
        if reason is not None:
            return None, tuple(generations), tuple(signatures), reason
        overlay, reason = apply_endpoint_generation(overlay, generation)
        if reason is not None:
            return None, tuple(generations), tuple(signatures), reason
        generations.append(generation)
        signatures.append(overlay_signature(overlay))
    return overlay, tuple(generations), tuple(signatures), None


def plan_endpoint_generations(builder, initial, *, budget):
    causal = []
    for iteration in range(budget + 1):
        overlay, generations, signatures, reason = _rebuild(initial, causal)
        if reason is not None:
            return EndpointFixedPointV1(generations, None, signatures, reason)
        contacts, reason = discover_endpoint_contacts(builder, overlay)
        if reason is not None:
            return EndpointFixedPointV1(generations, overlay, signatures, reason)
        if not contacts:
            return EndpointFixedPointV1(generations, overlay, signatures)
        generation, reason = normalize_endpoint_generation(overlay, contacts)
        if reason is not None:
            return EndpointFixedPointV1(generations, overlay, signatures, reason)
        if iteration == budget:
            return EndpointFixedPointV1(
                generations, overlay, signatures,
                "SYMBOLIC_ENDPOINT_GENERATION_BUDGET_EXHAUSTED",
            )
        causal.append(generation.contacts)
    raise AssertionError("unreachable symbolic endpoint generation loop")
