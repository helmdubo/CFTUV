"""Fail-closed composition of frozen initial mixed contact plans."""

from __future__ import annotations

from dataclasses import replace

from .events import EventKind


def compose_edge_split_overlap(
    contacts,
    split_cuts,
    vertices,
    *,
    birth_factory,
):
    """Скомпозировать единственную доказанную EDGE+interior-SPLIT биекцию."""

    contact_dead = {
        ident for contact in contacts for ident in contact.dead_vertex_ids
    }
    cut_dead = {
        event.vertex for cut in split_cuts for event in cut.events
    }
    overlap = frozenset(contact_dead & cut_dead)
    if not overlap:
        return contacts, split_cuts, frozenset(), True
    if len(contacts) != 1 or len(split_cuts) != 1 or len(overlap) != 1:
        return contacts, split_cuts, frozenset(), False
    contact, cut = contacts[0], split_cuts[0]
    if (
        EventKind.EDGE not in contact.kinds
        or len(contact.chains) != 1
        or len(contact.births) != 1
        or len(cut.events) != 1
        or len(cut.segment_occurrences) != 2
        or len(cut.births) != 2
        or len(cut.final_birth_ports) != 2
    ):
        return contacts, split_cuts, frozenset(), False
    (emitter_id,) = overlap
    chain = contact.chains[0]
    if emitter_id not in chain or cut.events[0].vertex != emitter_id:
        return contacts, split_cuts, frozenset(), False
    head, tail = vertices[chain[0]], vertices[chain[-1]]
    emitter = vertices[emitter_id]
    contact_birth = contact.births[0]
    if (
        contact_birth.prev_occurrence != head.prev_occurrence
        or contact_birth.next_occurrence != tail.next_occurrence
        or contact_birth.point_key != contact.point_key
    ):
        return contacts, split_cuts, frozenset(), False
    prev_side = tuple(
        birth
        for birth in cut.births
        if birth.prev_occurrence == emitter.prev_occurrence
        and birth.replaces == (emitter_id,)
    )
    next_side = tuple(
        birth
        for birth in cut.births
        if birth.next_occurrence == emitter.next_occurrence
        and birth.replaces == (emitter_id,)
    )
    if (
        len(prev_side) != 1
        or len(next_side) != 1
        or prev_side[0] is next_side[0]
        or prev_side[0].next_occurrence != cut.segment_occurrences[1]
        or next_side[0].prev_occurrence != cut.segment_occurrences[0]
        or any(birth.point_key != contact.point_key for birth in cut.births)
    ):
        return contacts, split_cuts, frozenset(), False
    composed_prev = birth_factory(
        contact.time,
        contact.point_key,
        head.prev_occurrence,
        prev_side[0].next_occurrence,
        replaces=(emitter_id,),
    )
    composed_next = birth_factory(
        contact.time,
        contact.point_key,
        next_side[0].prev_occurrence,
        tail.next_occurrence,
        replaces=(emitter_id,),
    )
    if composed_prev is None or composed_next is None:
        return contacts, split_cuts, frozenset(), False
    composed_births = (composed_prev, composed_next)
    if (
        len({birth.key for birth in composed_births}) != 2
        or len({birth.prev_occurrence for birth in composed_births}) != 2
        or len({birth.next_occurrence for birth in composed_births}) != 2
    ):
        return contacts, split_cuts, frozenset(), False
    flags = {
        key: (keep_prev, keep_next)
        for key, keep_prev, keep_next in cut.final_birth_ports
    }
    if set(flags) != {birth.key for birth in cut.births}:
        return contacts, split_cuts, frozenset(), False
    remapped = {
        prev_side[0].key: composed_prev.key,
        next_side[0].key: composed_next.key,
    }
    final_birth_ports = tuple(
        sorted(
            (remapped[key], keep_prev, keep_next)
            for key, (keep_prev, keep_next) in flags.items()
        )
    )
    return (
        (replace(contact, births=()),),
        (
            replace(
                cut,
                births=tuple(sorted(composed_births, key=lambda item: item.key)),
                final_birth_ports=final_birth_ports,
            ),
        ),
        overlap,
        True,
    )
