"""Per-generation EDGE/endpoint/interior-SPLIT symbolic normal form."""

from __future__ import annotations

from dataclasses import dataclass, replace
from functools import cmp_to_key

from .symbolic_component import (
    SymbolicComponentDeltaV1,
    apply_component_deltas,
    clone_overlay,
    overlay_signature,
)
from .symbolic_junction_contacts import discover_junction_contacts
from .symbolic_junction_fixed_point import (
    SymbolicJunctionFixedPointV1,
    SymbolicJunctionGenerationV1,
)
from .symbolic_junction_normalize import normalize_junction_generation
from .symbolic_overlay import (
    JunctionRefV1,
    SymbolicSpanBindingV1,
    refreshed_span_bindings,
)
from .superlevel_closure import SegmentRefV1
from .superlevel_fixed_point import merge_symbolic_split_contacts


@dataclass(frozen=True, slots=True)
class SymbolicMixedGenerationV1:
    junction_contacts: tuple
    interior_contacts: tuple
    deltas: tuple[SymbolicComponentDeltaV1, ...]


def _contact_compare(first, second, budget=None):
    sign = (first.projection - second.projection).sign(budget=budget)
    if sign:
        return sign
    if first.key == second.key:
        return 0
    return -1 if repr(first.key) < repr(second.key) else 1


def _expand_target_leaves(overlay, contacts, budget=None):
    result = clone_overlay(overlay)
    children_by_contact = {}
    grouped = {}
    for contact in contacts:
        if contact.leaf is None:
            return None, None, "SYMBOLIC_INTERIOR_SPLIT_LEAF_UNAVAILABLE"
        grouped.setdefault(contact.leaf, []).append(contact)
    for leaf, group in sorted(grouped.items(), key=lambda item: repr(item[0])):
        binding = result.spans.get(leaf)
        if binding is None or binding.start is None or binding.end is None:
            return None, None, "SYMBOLIC_INTERIOR_SPLIT_TARGET_STALE"
        ordered = tuple(sorted(
            group,
            key=cmp_to_key(
                lambda first, second: _contact_compare(first, second, budget)
            ),
        ))
        if len({item.key for item in ordered}) != len(ordered):
            return None, None, "SYMBOLIC_INTERIOR_SPLIT_CONTACT_DUPLICATE"
        for first, second in zip(ordered, ordered[1:]):
            if first.key.point_key == second.key.point_key:
                return (
                    None,
                    None,
                    "SYMBOLIC_INTERIOR_SPLIT_POINT_MULTIPLICITY_UNRESOLVABLE",
                )
        occurrence = leaf.occurrence
        points = tuple(item.key.point_key for item in ordered)
        boundaries = (occurrence[1], *points, occurrence[2])
        keys = (leaf.start, *(item.key for item in ordered), leaf.end)
        children = tuple(
            SegmentRefV1(
                leaf.family,
                keys[index],
                keys[index + 1],
                (occurrence[0], boundaries[index], boundaries[index + 1]),
            )
            for index in range(len(ordered) + 1)
        )
        if len(set(children)) != len(children):
            return None, None, "SYMBOLIC_INTERIOR_SPLIT_SEGMENT_COLLISION"
        del result.spans[leaf]
        for child in children:
            result.spans[child] = SymbolicSpanBindingV1(
                child, binding.physical_edge_id, None, None
            )
        start = result.vertices.get(binding.start)
        end = result.vertices.get(binding.end)
        if start is None or end is None or not start.alive or not end.alive:
            return None, None, "SYMBOLIC_INTERIOR_SPLIT_TARGET_STALE"
        start.next_leaf = children[0]
        end.prev_leaf = children[-1]
        for index, contact in enumerate(ordered):
            children_by_contact[contact.key] = (
                children[index], children[index + 1]
            )
        result.changed.discard(leaf)
        result.changed.update(children)
    return result, children_by_contact, None


def _interior_deltas(overlay, contacts, children):
    deltas = []
    for contact in sorted(contacts, key=lambda item: repr(item.key)):
        emitter = overlay.vertices.get(contact.key.emitter)
        if emitter is None or not emitter.alive:
            return None, "SYMBOLIC_INTERIOR_SPLIT_EMITTER_STALE"
        before, after = children[contact.key]
        first = JunctionRefV1(
            "INTERIOR_SPLIT",
            (contact.key, "EMITTER_TO_TARGET", emitter.prev_leaf, after),
        )
        second = JunctionRefV1(
            "INTERIOR_SPLIT",
            (contact.key, "TARGET_TO_EMITTER", before, emitter.next_leaf),
        )
        deltas.append(SymbolicComponentDeltaV1(
            (contact.key,),
            (emitter.ref,),
            None,
            None,
            None,
            contact.key.point_key,
            frozenset((
                emitter.prev_leaf, emitter.next_leaf, before, after,
            )),
            (
                ((emitter.prev, emitter.prev_leaf), (None, after), first),
                ((None, before), (emitter.next, emitter.next_leaf), second),
            ),
        ))
    return tuple(deltas), None


def _translated_junction_contacts(overlay, contacts):
    translated = []
    for contact in contacts:
        if contact.kind != "EDGE":
            translated.append(contact)
            continue
        start = overlay.vertices.get(contact.key.start)
        end = overlay.vertices.get(contact.key.end)
        if start is None or end is None:
            return None, "SYMBOLIC_JUNCTION_CONTACT_STALE"
        translated.append(replace(
            contact,
            edge=replace(
                contact.edge,
                prev_leaf=start.prev_leaf,
                shared_leaf=start.next_leaf,
                next_leaf=end.next_leaf,
            ),
        ))
    return tuple(translated), None


def normalize_mixed_generation(builder, overlay, junction, interior):
    expanded, children, reason = _expand_target_leaves(
        overlay, interior, getattr(builder, "work_budget", None)
    )
    if reason is not None:
        return None, None, reason
    interior_deltas, reason = _interior_deltas(expanded, interior, children)
    if reason is not None:
        return None, None, reason
    translated, reason = _translated_junction_contacts(expanded, junction)
    if reason is not None:
        return None, None, reason
    junction_deltas = ()
    if translated:
        normalized, reason = normalize_junction_generation(
            builder, expanded, translated, SymbolicJunctionGenerationV1
        )
        if reason is not None:
            return None, None, reason
        junction_deltas = normalized.deltas
    deltas = (*junction_deltas, *interior_deltas)
    generation = SymbolicMixedGenerationV1(
        tuple(translated),
        tuple(sorted(interior, key=lambda item: repr(item.key))),
        tuple(sorted(deltas, key=lambda item: repr(item.contact_keys))),
    )
    return expanded, generation, None


def apply_mixed_generation(expanded, generation):
    return apply_component_deltas(
        expanded,
        generation.deltas,
        collision_reason="SYMBOLIC_MIXED_COMPONENT_DELTAS_OVERLAP",
    )


def plan_mixed_generations(builder, initial, discover_interior, *, budget):
    causal = []
    for iteration in range(budget + 1):
        overlay = clone_overlay(initial)
        generations, signatures = [], []
        for junction, interior in causal:
            expanded, generation, reason = normalize_mixed_generation(
                builder, overlay, junction, interior
            )
            if reason is not None:
                return SymbolicJunctionFixedPointV1(
                    tuple(generations), None, tuple(signatures), reason
                ), ()
            overlay, reason = apply_mixed_generation(expanded, generation)
            if reason is not None:
                return SymbolicJunctionFixedPointV1(
                    tuple(generations), None, tuple(signatures), reason
                ), ()
            generations.append(generation)
            signatures.append(overlay_signature(overlay))
        junction, reason = discover_junction_contacts(builder, overlay)
        if reason is not None:
            return SymbolicJunctionFixedPointV1(
                tuple(generations), overlay, tuple(signatures), reason
            ), ()
        interior, reason = discover_interior(builder, overlay)
        if reason is not None:
            return SymbolicJunctionFixedPointV1(
                tuple(generations), overlay, tuple(signatures), reason
            ), ()
        if not junction and not interior:
            return SymbolicJunctionFixedPointV1(
                tuple(generations), overlay, tuple(signatures)
            ), tuple(
                item for _, batch in causal for item in batch
            )
        interior, reason = merge_symbolic_split_contacts(interior)
        if reason is not None:
            return SymbolicJunctionFixedPointV1(
                tuple(generations), overlay, tuple(signatures), reason
            ), ()
        if iteration == budget:
            return SymbolicJunctionFixedPointV1(
                tuple(generations), overlay, tuple(signatures),
                "SYMBOLIC_MIXED_GENERATION_BUDGET_EXHAUSTED",
            ), ()
        causal.append((junction, interior))
    raise AssertionError("unreachable mixed symbolic generation loop")
