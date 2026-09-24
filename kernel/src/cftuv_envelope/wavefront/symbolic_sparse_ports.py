"""Sparse one-hop symbolic ports without unrelated exact hydration."""

from __future__ import annotations

from dataclasses import replace

from . import superlevel as base


def with_line_ports(builder, snapshot, time):
    """Attach stable line-only occurrences without evaluating endpoints.

    Every live LAV vertex is admitted so a remote reflex/sliding emitter and
    the topology around a newly discovered contact are both available.  A
    missing endpoint stays ``None``; exact point hydration remains on demand.
    Owner aliases are checked later while physical occurrences are bound.
    """

    vertices = []
    for vertex in snapshot.vertices:
        if not vertex.alive:
            vertices.append(vertex)
            continue
        prev_occurrence = vertex.prev_occurrence or (
            builder.edges[vertex.prev_edge].key, None, None
        )
        next_occurrence = vertex.next_occurrence or (
            builder.edges[vertex.next_edge].key, None, None
        )
        vertices.append(replace(
            vertex,
            prev_occurrence=prev_occurrence,
            next_occurrence=next_occurrence,
        ))
    line_only = replace(snapshot, vertices=tuple(vertices))
    owners = {}
    for vertex in line_only.vertices:
        if not vertex.alive or vertex.next_occurrence is None:
            continue
        owners.setdefault(vertex.next_occurrence, set()).add(vertex.next_edge)
    ambiguous_keys = {
        occurrence[0]
        for occurrence, edge_ids in owners.items()
        if len(edge_ids) > 1 and occurrence[1:] == (None, None)
    }
    if not ambiguous_keys:
        return line_only

    # Twin edges share one physical key. Hydrate only those ambiguous line
    # families; unrelated sparse ports remain line-only.
    point_keys, occurrences, duplicate = base._sparse_occurrences(
        builder, time, ambiguous_keys
    )
    if duplicate:
        return None
    hydrated = tuple(
        replace(
            vertex,
            point_key=(
                point_keys[vertex.ident]
                if point_keys[vertex.ident] is not None
                else vertex.point_key
            ),
            prev_occurrence=occurrences.get(
                vertex.prev_edge, vertex.prev_occurrence
            ),
            next_occurrence=occurrences.get(
                vertex.next_edge, vertex.next_occurrence
            ),
        )
        for vertex in line_only.vertices
    )
    return replace(line_only, vertices=hydrated)
