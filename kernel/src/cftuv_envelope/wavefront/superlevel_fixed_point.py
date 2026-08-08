"""Pure stable split-contact growth with root-family rebinding."""

from __future__ import annotations

from dataclasses import dataclass, replace

from .candidate_law import evaluate_split_candidate
from .event_time import compare_times
from .events import CandidateEventV1, EventKind
from .exact_identity import exact_point_key
from . import superlevel as base
from .superlevel_closure import SpanFamilyRefV1, plan_split_materialization
from .symbolic_overlay import (
    JunctionRefV1,
    SymbolicOverlayV1,
    build_symbolic_overlay,
    exact_overlay_view,
    is_symbolic_split_emitter,
)


@dataclass(frozen=True, slots=True)
class SymbolicSplitContactKeyV1:
    time_key: tuple
    point_key: tuple
    emitter: JunctionRefV1
    family: SpanFamilyRefV1
    participants: tuple[tuple[int, ...], ...]


@dataclass(frozen=True, slots=True)
class SymbolicSplitContactV1:
    key: SymbolicSplitContactKeyV1
    time: object
    point: object
    projection: object
    leaf: object | None = None


@dataclass(frozen=True, slots=True)
class SymbolicSplitClosureV1:
    materialization: object | None
    overlay: SymbolicOverlayV1 | None
    added_contacts: tuple[SymbolicSplitContactV1, ...]
    iterations: int
    signatures: tuple[tuple, ...]
    unresolved_reason: str | None = None


def merge_symbolic_split_contacts(*groups):
    """Dedupe equal stable contacts; reject identity-equal payload aliases."""

    unique = {}
    for contact in (item for group in groups for item in group):
        previous = unique.get(contact.key)
        if previous is not None and previous != contact:
            return (), "SYMBOLIC_INTERIOR_SPLIT_CONTACT_METADATA_CONFLICT"
        unique[contact.key] = contact
    return tuple(sorted(unique.values(), key=lambda item: repr(item.key))), None


def _overlay_signature(overlay):
    rows = [
        (vertex.ref, vertex.prev, vertex.next,
         vertex.prev_leaf, vertex.next_leaf)
        for vertex in overlay.vertices.values()
    ]
    return tuple(sorted(rows, key=repr))


def _required_keys(contacts):
    required = set()
    for contact in contacts:
        required.add(contact.key.family.occurrence[0])
        if contact.key.emitter.kind != "EXISTING":
            continue
        for edge_key in contact.key.emitter.key:
            if edge_key is not None:
                required.add(edge_key)
    return required


def _hydrate_contacts(builder, snapshot, contacts):
    point_keys, occurrences, duplicate = base._sparse_occurrences(
        builder, contacts[0].time, _required_keys(contacts)
    )
    if duplicate:
        return None, "SYMBOLIC_SPLIT_ROOT_REBIND_UNRESOLVABLE"
    vertices = tuple(
        replace(
            vertex,
            point_key=point_keys[vertex.ident],
            prev_occurrence=occurrences.get(vertex.prev_edge),
            next_occurrence=occurrences.get(vertex.next_edge),
        )
        for vertex in snapshot.vertices
    )
    return vertices, None


def _unique_map(pairs):
    grouped = {}
    for key, value in pairs:
        grouped.setdefault(key, set()).add(value)
    if any(len(values) != 1 for values in grouped.values()):
        return None
    return {key: next(iter(values)) for key, values in grouped.items()}


def _compile_contacts(builder, snapshot, contacts):
    if not contacts:
        return (), None
    vertices, reason = _hydrate_contacts(builder, snapshot, contacts)
    if reason is not None:
        return (), reason
    wanted_emitters = {contact.key.emitter for contact in contacts}
    emitters = _unique_map(
        (ref, vertex.ident)
        for vertex in vertices
        if vertex.point_key is not None
        for ref in (JunctionRefV1("EXISTING", base._port_identity(vertex)),)
        if ref in wanted_emitters
    )
    roots = _unique_map(
        (occurrence, edge_id)
        for vertex in vertices
        for edge_id, occurrence in (
            (vertex.prev_edge, vertex.prev_occurrence),
            (vertex.next_edge, vertex.next_occurrence),
        )
        if occurrence is not None
    )
    if emitters is None or roots is None:
        return (), "SYMBOLIC_SPLIT_ROOT_REBIND_UNRESOLVABLE"
    compiled = []
    for contact in sorted(contacts, key=lambda item: repr(item.key)):
        emitter_id = emitters.get(contact.key.emitter)
        root_edge_id = roots.get(contact.key.family.occurrence)
        if emitter_id is None or root_edge_id is None:
            return (), "SYMBOLIC_SPLIT_ROOT_REBIND_UNRESOLVABLE"
        event = CandidateEventV1(
            EventKind.SPLIT, contact.time, contact.point,
            emitter_id, -1, root_edge_id,
        )
        incident = base._incident(builder, event, vertices)
        compiled.append(
            replace(
                incident,
                participants=contact.key.participants,
                target_participants=contact.key.family.participant_keys,
                target_projection=contact.projection,
                target_occurrence=contact.key.family.occurrence,
                emitter_key=contact.key.emitter.key,
            )
        )
    return tuple(compiled), None


def _participants(*leaves):
    return tuple(sorted({participant for leaf in leaves
                         for participant in leaf.family.participant_keys}))


def _latent_contact(builder, overlay, emitter, leaf):
    if leaf in (emitter.prev_leaf, emitter.next_leaf):
        return None, None
    if not is_symbolic_split_emitter(builder, overlay, emitter):
        return None, None
    decision = evaluate_split_candidate(
        exact_overlay_view(builder, overlay), emitter.ref, leaf,
        now=overlay.time,
    )
    candidate = decision.candidate
    budget = getattr(builder, "work_budget", None)
    if candidate is None or compare_times(
        candidate.time, overlay.time, budget
    ) != 0:
        return None, None
    if candidate.at_start or candidate.at_end:
        return None, "SYMBOLIC_SPLIT_ENDPOINT_JUNCTION_REQUIRED"
    binding = overlay.spans[leaf]
    projection = (
        candidate.point.x.scaled(builder.edges[binding.physical_edge_id].line.b)
        - candidate.point.y.scaled(builder.edges[binding.physical_edge_id].line.a)
    )
    key = SymbolicSplitContactKeyV1(
        base._time_key(candidate.time),
        exact_point_key(candidate.point),
        emitter.ref,
        leaf.family,
        _participants(emitter.prev_leaf, emitter.next_leaf, leaf),
    )
    return SymbolicSplitContactV1(
        key, candidate.time, candidate.point, projection, leaf
    ), None


def _latent_contacts(builder, overlay):
    contacts = []
    emitters = tuple(
        vertex for vertex in overlay.vertices.values()
        if vertex.alive
        and is_symbolic_split_emitter(builder, overlay, vertex)
    )
    for leaf in sorted(overlay.changed, key=repr):
        for emitter in sorted(emitters, key=lambda item: repr(item.ref)):
            contact, reason = _latent_contact(builder, overlay, emitter, leaf)
            if reason is not None:
                return (), reason
            if contact is not None:
                contacts.append(contact)
    return tuple(sorted(contacts, key=lambda item: repr(item.key))), None


def plan_symbolic_split_fixed_point(builder, snapshot, *, budget: int):
    """Grow stable contacts and recompile every round against their root family."""

    contacts = []
    identities = set()
    signatures = []
    for iteration in range(budget + 1):
        compiled, reason = _compile_contacts(builder, snapshot, contacts)
        if reason is not None:
            return SymbolicSplitClosureV1(
                None, None, tuple(contacts), iteration,
                tuple(signatures), reason,
            )
        working = replace(
            snapshot, incidents=(*snapshot.incidents, *compiled)
        )
        materialization = plan_split_materialization(
            working, getattr(builder, "work_budget", None)
        )
        if materialization.unresolved_reason is not None:
            return SymbolicSplitClosureV1(
                materialization, None, tuple(contacts), iteration,
                tuple(signatures), materialization.unresolved_reason,
            )
        overlay = build_symbolic_overlay(builder, working, materialization)
        if overlay is None:
            return SymbolicSplitClosureV1(
                materialization, None, tuple(contacts), iteration,
                tuple(signatures), "SYMBOLIC_SPLIT_OVERLAY_UNRESOLVABLE",
            )
        signatures.append(_overlay_signature(overlay))
        latent, reason = _latent_contacts(builder, overlay)
        if reason is not None:
            return SymbolicSplitClosureV1(
                materialization, overlay, tuple(contacts), iteration,
                tuple(signatures), reason,
            )
        new = tuple({item.key: item for item in latent
                     if item.key not in identities}.values())
        if not new:
            return SymbolicSplitClosureV1(
                materialization, overlay, tuple(contacts), iteration,
                tuple(signatures),
            )
        if iteration == budget:
            return SymbolicSplitClosureV1(
                materialization, overlay, tuple(contacts), iteration,
                tuple(signatures), "SYMBOLIC_SPLIT_BUDGET_EXHAUSTED",
            )
        for item in new:
            identities.add(item.key)
            contacts.append(item)
    raise AssertionError("unreachable symbolic split fixed point")
