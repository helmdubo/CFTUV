"""Frozen prestate одного exact-time packet: снимок и его тождества.

Слой выемки из `superlevel.py` (правило потолков): здесь живёт ВСЁ, что
читает `builder` до первой мутации, и ничего, что мутирует. Планы, рождения
и коммит остались в `superlevel.py`; сюда они ходят только за снимком.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import gcd

from .event_time import EventPointV1, EventTimeV1, ZERO_TIME
from .events import CandidateEventV1, EventKind


@dataclass(frozen=True, slots=True)
class _VertexSnapshot:
    ident: int
    prev: int
    next: int
    prev_edge: int
    next_edge: int
    alive: bool
    incoming_ray: tuple[int, int]
    outgoing_ray: tuple[int, int]
    point_key: tuple | None = None
    prev_occurrence: tuple | None = None
    next_occurrence: tuple | None = None


@dataclass(frozen=True, slots=True)
class SuperlevelIncidentV1:
    """Один live candidate с геометрическим identity и runtime payload."""

    event: CandidateEventV1
    vertex_ids: tuple[int, ...]
    edge_occurrences: tuple[int, ...]
    participants: tuple[tuple[int, ...], ...]
    target_participants: tuple[tuple[int, ...], ...]
    point_key: tuple
    met_vertex_id: int | None = None
    met_adjacent: bool = False
    target_projection: object | None = None
    target_start_id: int | None = None
    target_end_id: int | None = None
    emitter_key: tuple = ()
    peer_key: tuple = ()
    target_occurrence: tuple | None = None
    #: Примитивный целочисленный луч рассекаемого вхождения. Он берётся из
    #: несущей ПРЯМОЙ, а не из ключа: у скрытой опоры веера ключ вырожден
    #: (`x, y, x, y, ordinal`) и направления не несёт.
    target_ray: tuple[int, int] | None = None


@dataclass(frozen=True, slots=True)
class SuperlevelSnapshotV1:
    """Неизменяемый prestate всего снятого exact-time packet."""

    incidents: tuple[SuperlevelIncidentV1, ...]
    vertices: tuple[_VertexSnapshot, ...]
    unsupported: tuple[CandidateEventV1, ...]
    stale_candidates: int
    duplicate_live_owner_edge_ids: tuple[int, ...] = ()

def _event_point_key(event: CandidateEventV1) -> tuple:
    return (event.point.x.terms, event.point.y.terms)


def _time_key(time: EventTimeV1) -> tuple:
    canonical = time.canonical()
    return (canonical.dividend, canonical.divisor.terms)


def _direction(line) -> tuple[int, int]:
    dx, dy = line.b, -line.a
    common = gcd(abs(dx), abs(dy))
    return (dx // common, dy // common)


def _vertex_snapshot(
    builder, vertex, point_key, occurrences
) -> _VertexSnapshot:
    next_line = getattr(builder.edges[vertex.next_edge], "line", None)
    prev_line = getattr(builder.edges[vertex.prev_edge], "line", None)
    outgoing = (1, 0) if next_line is None else _direction(next_line)
    incoming = (1, 0) if prev_line is None else _direction(prev_line)
    return _VertexSnapshot(
        ident=vertex.ident,
        prev=vertex.prev,
        next=vertex.next,
        prev_edge=vertex.prev_edge,
        next_edge=vertex.next_edge,
        alive=vertex.alive,
        incoming_ray=(-incoming[0], -incoming[1]),
        outgoing_ray=outgoing,
        point_key=point_key,
        prev_occurrence=occurrences.get(vertex.prev_edge),
        next_occurrence=occurrences.get(vertex.next_edge),
    )


def _port_identity(vertex: _VertexSnapshot) -> tuple:
    """Hydration-invariant identity of an existing moving-line junction.

    Exact endpoint coordinates are deliberately excluded: a sparse line port
    may acquire either endpoint later when an interior contact grows K1/K2.
    The two owner-distinct source edge keys stay invariant across that
    hydration.  Multiple live aliases of the same ordered pair are rejected by
    the symbolic overlay's unique-ref admission instead of being distinguished
    with a runtime id.
    """

    return (
        None if vertex.prev_occurrence is None else vertex.prev_occurrence[0],
        None if vertex.next_occurrence is None else vertex.next_occurrence[0],
    )

def _incident(
    builder,
    event: CandidateEventV1,
    vertices: tuple[_VertexSnapshot, ...],
) -> SuperlevelIncidentV1:
    vertex = builder.vertices[event.vertex]
    met_vertex_id = None
    met_adjacent = False
    target_projection = None
    target_ray = None
    target_start_id = None
    target_end_id = None
    if event.kind is EventKind.EDGE:
        peer = builder.vertices[event.peer]
        core_vertex_ids = (vertex.ident, peer.ident)
        participant_edge_ids = (
            vertex.prev_edge,
            vertex.next_edge,
            peer.prev_edge,
            peer.next_edge,
        )
        occurrence_ids = participant_edge_ids
        target_occurrence = None
    else:
        endpoint_ids = builder._proof_edge_endpoint_ids(event.edge)
        target_start_id = getattr(builder, "edge_start", {}).get(event.edge)
        target_end_id = getattr(builder, "edge_end", {}).get(event.edge)
        met_by = getattr(builder, "_front_vertex_met_by", None)
        met, met_adjacent = (None, False) if met_by is None else met_by(event)
        met_vertex_id = None if met is None else met.ident
        core_vertex_ids = (
            vertex.ident,
            *endpoint_ids,
            *((met_vertex_id,) if met_vertex_id is not None else ()),
        )
        participant_edge_ids = (
            vertex.prev_edge,
            vertex.next_edge,
            event.edge,
        )
        occurrence_ids = participant_edge_ids
        target_occurrence = vertices[event.vertex].next_occurrence
        if 0 <= event.edge < len(builder.edges):
            target_occurrence = next(
                (
                    item.next_occurrence
                    for item in vertices
                    if item.alive and item.next_edge == event.edge
                ),
                None,
            )
        edge = builder.edges[event.edge]
        if hasattr(edge, "line"):
            target_ray = _direction(edge.line)
            target_projection = (
                event.point.x.scaled(edge.line.b)
                - event.point.y.scaled(edge.line.a)
            )
    resource_vertices = set(core_vertex_ids)
    resource_vertices.update(
        neighbor
        for ident in tuple(resource_vertices)
        for neighbor in (
            builder.vertices[ident].prev,
            builder.vertices[ident].next,
        )
    )
    return SuperlevelIncidentV1(
        event=event,
        vertex_ids=tuple(sorted(resource_vertices)),
        # Runtime id здесь только owner-distinct occurrence token. Он не входит
        # ни в сортировку плана, ни в узел/дайджест.
        edge_occurrences=tuple(sorted(set(occurrence_ids))),
        participants=tuple(
            sorted(set(builder._edge_keys(*participant_edge_ids)))
        ),
        target_participants=tuple(
            sorted(set(builder._edge_keys(*occurrence_ids)))
        ),
        point_key=_event_point_key(event),
        met_vertex_id=met_vertex_id,
        met_adjacent=met_adjacent,
        target_projection=target_projection,
        target_start_id=target_start_id,
        target_end_id=target_end_id,
        emitter_key=_port_identity(vertices[event.vertex]),
        peer_key=(
            _port_identity(vertices[event.peer])
            if event.kind is EventKind.EDGE
            else ()
        ),
        target_occurrence=target_occurrence,
        target_ray=target_ray,
    )

def _live_level(builder, level):
    live = []
    unsupported = []
    stale = 0
    for event in level:
        if event.kind not in {EventKind.SPLIT, EventKind.EDGE}:
            unsupported.append(event)
            continue
        is_live = (
            builder._edge_event_is_live(event)
            if event.kind is EventKind.EDGE
            else builder._split_is_live(event)
        )
        if is_live:
            live.append(event)
        else:
            stale += 1
    return tuple(live), tuple(unsupported), stale


def _required_physical_edge_keys(builder, events) -> set[tuple]:
    required = set()
    split_target_keys = set()
    for event in events:
        vertex = builder.vertices[event.vertex]
        edge_ids = [vertex.prev_edge, vertex.next_edge]
        if event.kind is EventKind.EDGE:
            peer = builder.vertices[event.peer]
            edge_ids.extend((peer.prev_edge, peer.next_edge))
        else:
            edge_ids.append(event.edge)
            split_target_keys.add(builder.edges[event.edge].key)
            for ident in builder._proof_edge_endpoint_ids(event.edge):
                endpoint = builder.vertices[ident]
                edge_ids.extend((endpoint.prev_edge, endpoint.next_edge))
        required.update(builder.edges[ident].key for ident in edge_ids)
    # K1 is exactly one hop around every live alias of a split-target span.
    # It hydrates the opposite endpoint ports needed by a rewrite, but does not
    # walk the LAV recursively.
    for start in builder.vertices:
        if not start.alive:
            continue
        end = builder.vertices[start.next]
        if (
            end.alive
            and builder.edges[start.next_edge].key in split_target_keys
        ):
            required.update(
                (
                    builder.edges[start.prev_edge].key,
                    builder.edges[end.next_edge].key,
                )
            )
    return required


def _sparse_occurrences(builder, time, required_keys):
    span_starts = []
    point_vertex_ids = set()
    live_owner_counts: dict[int, int] = {}
    for vertex in builder.vertices:
        if not vertex.alive:
            continue
        end = builder.vertices[vertex.next]
        if not end.alive:
            continue
        live_owner_counts[vertex.next_edge] = (
            live_owner_counts.get(vertex.next_edge, 0) + 1
        )
        if builder.edges[vertex.next_edge].key in required_keys:
            span_starts.append((vertex, end))
            point_vertex_ids.update((vertex.ident, end.ident))
    point_keys = [None] * len(builder.vertices)
    for ident in sorted(point_vertex_ids):
        point = builder._position(builder.vertices[ident], time)
        point_keys[ident] = (
            None if point is None else (point.x.terms, point.y.terms)
        )
    occurrences = {
        start.next_edge: (
            builder.edges[start.next_edge].key,
            point_keys[start.ident],
            point_keys[end.ident],
        )
        for start, end in span_starts
    }
    duplicate_owner_ids = tuple(
        sorted(
            edge_id
            for edge_id, count in live_owner_counts.items()
            if count != 1
        )
    )
    return point_keys, occurrences, duplicate_owner_ids


def collect_superlevel_snapshot(
    builder, level: tuple[CandidateEventV1, ...]
) -> SuperlevelSnapshotV1:
    """Снять только queried physical-edge incidences из frozen prestate."""

    time = level[0].time if level else ZERO_TIME
    live, unsupported, stale = _live_level(builder, level)
    required_keys = _required_physical_edge_keys(builder, live)
    point_keys, occurrences, duplicate_owner_ids = _sparse_occurrences(
        builder, time, required_keys
    )
    vertices = tuple(
        _vertex_snapshot(builder, vertex, point_keys[vertex.ident], occurrences)
        for vertex in builder.vertices
    )
    return SuperlevelSnapshotV1(
        incidents=tuple(_incident(builder, event, vertices) for event in live),
        vertices=vertices,
        unsupported=unsupported,
        stale_candidates=stale,
        duplicate_live_owner_edge_ids=duplicate_owner_ids,
    )

def _incident_sort_key(incident: SuperlevelIncidentV1) -> tuple:
    """Только геометрия; runtime ids не разрешают ничьих."""

    return (
        incident.event.kind.value,
        repr(incident.point_key),
        repr(incident.emitter_key),
        repr(incident.peer_key),
        repr(incident.target_occurrence),
        incident.participants,
        incident.target_participants,
        repr(incident.target_projection),
        incident.event.span_unproven,
    )
