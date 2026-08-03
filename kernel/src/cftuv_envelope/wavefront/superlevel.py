"""Exact-time utilities shared by node accumulation and level draining."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from functools import cmp_to_key

from .event_time import EventPointV1, EventTimeV1, compare_times
from .events import CandidateEventV1, EventKind, EventQueueV1
from .proof import (
    ProofObligationBranch,
    ProofObligationDisposition,
    ProofObligationV1,
    ProofStatus,
)
# Слой frozen prestate вынесен целиком; имена остаются на этом модуле, потому
# что его читают и `base.X`, и тесты — фасад дешевле правки тридцати мест.
from .superlevel_snapshot import (  # noqa: F401
    SuperlevelIncidentV1,
    SuperlevelSnapshotV1,
    _VertexSnapshot,
    _direction,
    _event_point_key,
    _incident,
    _incident_sort_key,
    _live_level,
    _port_identity,
    _required_physical_edge_keys,
    _sparse_occurrences,
    _time_key,
    _vertex_snapshot,
    collect_superlevel_snapshot,
)


class SkeletonOutcome(str, Enum):
    """Чем кончилось построение. Тихого выхода нет ни одного."""

    EXACT = "EXACT"
    LEVEL_BUDGET_EXHAUSTED = "LEVEL_BUDGET_EXHAUSTED"
    MULTIWAY_SPLIT_UNPROVEN = "MULTIWAY_SPLIT_UNPROVEN"
    WAVEFRONT_LEFT_UNRESOLVED = "WAVEFRONT_LEFT_UNRESOLVED"
    SUPERLEVEL_COMPONENT_UNRESOLVABLE = (
        "SUPERLEVEL_COMPONENT_UNRESOLVABLE"
    )


@dataclass(frozen=True, slots=True)
class SkeletonNodeV1:
    """Узел скелета и canonical incidence component в exact `(time, point)`.

    `participants` — отсортированные ключи вхождений исходных рёбер. У
    `MULTIWAY` поле `kinds` хранит unique sorted исходные виды, а `incidences`
    сохраняет их participant sets для byte-identical face materialization.
    """

    kind: EventKind
    time: EventTimeV1
    point: EventPointV1
    participants: tuple[tuple[int, ...], ...]
    converging_vertices: int
    kinds: tuple[EventKind, ...] = ()
    incidences: tuple[tuple[tuple[int, ...], ...], ...] = ()


@dataclass(frozen=True, slots=True)
class SkeletonV1:
    outcome: SkeletonOutcome
    nodes: tuple[SkeletonNodeV1, ...]
    levels: int
    counters: tuple[tuple[str, int], ...]
    proof_status: ProofStatus = ProofStatus.COMPLETE
    proof_obligations: tuple[ProofObligationV1, ...] = ()

    def counter(self, name: str) -> int:
        return dict(self.counters).get(name, 0)


class SuperlevelResolution(str, Enum):
    """Как frozen component меняет LAV; это не порядок событий."""

    EDGE = "EDGE"
    SPLIT = "SPLIT"
    BOUNDARY_PORT_PAIRING = "BOUNDARY_PORT_PAIRING"
    UNRESOLVABLE = "UNRESOLVABLE"


@dataclass(frozen=True, slots=True)
class BoundaryBirthV1:
    """ContactJunction между двумя geometry-derived edge occurrences."""

    point_key: tuple
    prev_occurrence: tuple
    next_occurrence: tuple
    key: tuple
    replaces: tuple[int, ...] = ()


@dataclass(frozen=True, slots=True)
class VertexReferenceV1:
    """Existing vertex либо symbolic birth frozen delta."""

    existing: int | None = None
    birth_key: tuple | None = None


@dataclass(frozen=True, slots=True)
class EdgeContactPlanV1:
    events: tuple[CandidateEventV1, ...]
    time: EventTimeV1
    point: EventPointV1
    point_key: tuple
    participants: tuple[tuple[int, ...], ...]
    dead_vertex_ids: tuple[int, ...]
    chains: tuple[tuple[int, ...], ...]
    births: tuple[BoundaryBirthV1, ...]
    kinds: tuple[EventKind, ...] = (EventKind.EDGE,)


@dataclass(frozen=True, slots=True)
class SplitCutPlanV1:
    edge_id: int
    target_occurrence: tuple
    events: tuple[CandidateEventV1, ...]
    segment_occurrences: tuple[tuple, ...]
    births: tuple[BoundaryBirthV1, ...]
    final_birth_ports: tuple[tuple[tuple, bool, bool], ...]


@dataclass(frozen=True, slots=True)
class VertexMeetingPlanV1:
    events: tuple[CandidateEventV1, ...]
    time: EventTimeV1
    point: EventPointV1
    meeting_vertex_ids: tuple[int, ...]
    pairs: tuple[tuple[int, int], ...]
    participants: tuple[tuple[int, ...], ...]
    births: tuple[BoundaryBirthV1, ...]


@dataclass(frozen=True, slots=True)
class SuperlevelComponentPlanV1:
    """Чистая delta одного resource-connected contact component."""

    event_kinds: tuple[EventKind, ...]
    time: EventTimeV1
    point_keys: tuple[tuple, ...]
    point: EventPointV1
    resolution: SuperlevelResolution
    events: tuple[CandidateEventV1, ...]
    participants: tuple[tuple[int, ...], ...]
    target_participants: tuple[tuple[int, ...], ...]
    dead_vertex_ids: tuple[int, ...] = ()
    chains: tuple[tuple[int, ...], ...] = ()
    births: tuple[BoundaryBirthV1, ...] = ()
    queue_seed_vertex_ids: tuple[int, ...] = ()
    enqueue_born_vertices: bool = False
    edge_contacts: tuple[EdgeContactPlanV1, ...] = ()
    split_cuts: tuple[SplitCutPlanV1, ...] = ()
    vertex_meetings: tuple[VertexMeetingPlanV1, ...] = ()
    proof_fallbacks: tuple[CandidateEventV1, ...] = ()
    coincident_split_targets: int = 0
    closed_chain_count: int = 0
    suppressed_candidates: int = 0
    birth_wiring: tuple[
        tuple[tuple, VertexReferenceV1, VertexReferenceV1], ...
    ] = ()
    existing_port_rewrites: tuple[tuple[int, tuple, tuple], ...] = ()
    birth_components: tuple[tuple[tuple, ...], ...] = ()
    terminal_birth_cycles: tuple[tuple[tuple, ...], ...] = ()


def _connected_components(
    incidents: tuple[SuperlevelIncidentV1, ...],
) -> tuple[tuple[SuperlevelIncidentV1, ...], ...]:
    """Транзитивное замыкание по точке, вершине и edge occurrence."""

    pending = set(range(len(incidents)))
    components = []
    while pending:
        seed = min(pending)
        component = {seed}
        vertices = set(incidents[seed].vertex_ids)
        edges = set(incidents[seed].edge_occurrences)
        changed = True
        while changed:
            joined = {
                index
                for index in pending - component
                if (
                    vertices.intersection(incidents[index].vertex_ids)
                    or edges.intersection(incidents[index].edge_occurrences)
                )
            }
            changed = bool(joined)
            component.update(joined)
            vertices.update(
                ident
                for index in joined
                for ident in incidents[index].vertex_ids
            )
            edges.update(
                ident
                for index in joined
                for ident in incidents[index].edge_occurrences
            )
        pending.difference_update(component)
        members = tuple(incidents[index] for index in component)
        components.append(tuple(sorted(members, key=_incident_sort_key)))
    return tuple(components)


def _snapshot_chains(
    idents: set[int], vertices: tuple[_VertexSnapshot, ...]
) -> tuple[tuple[int, ...], ...]:
    """Связные dead-runs frozen LAV; runtime ids наружу не канонизируют."""

    live = {ident for ident in idents if vertices[ident].alive}
    chains = []
    seen: set[int] = set()
    for ident in sorted(live):
        if ident in seen:
            continue
        start = ident
        guard = 0
        while (
            vertices[start].prev in live
            and vertices[start].prev != ident
            and guard <= len(live)
        ):
            start = vertices[start].prev
            guard += 1
        chain = [start]
        seen.add(start)
        cursor = vertices[start].next
        while cursor in live and cursor not in seen:
            chain.append(cursor)
            seen.add(cursor)
            cursor = vertices[cursor].next
        chains.append(tuple(chain))
    return tuple(chains)


def _component_fields(
    component: tuple[SuperlevelIncidentV1, ...],
) -> dict:
    """Canonical geometric fields common to every resolution."""

    kinds = tuple(
        sorted({incident.event.kind for incident in component}, key=lambda k: k.value)
    )
    point_keys = tuple(sorted({incident.point_key for incident in component}, key=repr))
    participants = tuple(
        sorted(
            {
                participant
                for incident in component
                for participant in incident.participants
            }
        )
    )
    targets = tuple(
        sorted(
            {
                participant
                for incident in component
                for participant in incident.target_participants
            }
        )
    )
    events = tuple(incident.event for incident in component)
    sample = min(component, key=lambda incident: repr(incident.point_key)).event
    return dict(
        event_kinds=kinds,
        time=sample.time,
        point_keys=point_keys,
        point=sample.point,
        events=events,
        participants=participants,
        target_participants=targets,
    )


def _birth(
    time: EventTimeV1,
    point_key: tuple,
    prev_occurrence: tuple | None,
    next_occurrence: tuple | None,
    *,
    replaces: tuple[int, ...] = (),
) -> BoundaryBirthV1 | None:
    """Построить canonical ContactJunction; runtime id в ключ не входит."""

    if prev_occurrence is None or next_occurrence is None:
        return None
    key = (
        _time_key(time),
        point_key,
        prev_occurrence,
        next_occurrence,
    )
    return BoundaryBirthV1(
        point_key=point_key,
        prev_occurrence=prev_occurrence,
        next_occurrence=next_occurrence,
        key=key,
        replaces=replaces,
    )


def _edge_contact_plans(
    edges: tuple[SuperlevelIncidentV1, ...],
    splits: tuple[SuperlevelIncidentV1, ...],
    vertices: tuple[_VertexSnapshot, ...],
) -> tuple[
    tuple[EdgeContactPlanV1, ...],
    tuple[SuperlevelIncidentV1, ...],
    bool,
]:
    """Plan edge deaths/births and absorb only same-contact incidences."""

    grouped: dict[tuple, list[SuperlevelIncidentV1]] = {}
    for incident in edges:
        grouped.setdefault(incident.point_key, []).append(incident)
    contacts = []
    absorbed: set[CandidateEventV1] = set()
    valid = True
    for point_key in sorted(grouped, key=repr):
        local_edges = tuple(sorted(grouped[point_key], key=_incident_sort_key))
        dead = {
            ident
            for incident in local_edges
            for ident in (incident.event.vertex, incident.event.peer)
        }
        chains = _snapshot_chains(dead, vertices)
        incident_edges = {
            edge_id
            for ident in dead
            for edge_id in (vertices[ident].prev_edge, vertices[ident].next_edge)
        }
        boundary_endpoints = set()
        for chain in chains:
            head, tail = vertices[chain[0]], vertices[chain[-1]]
            boundary_endpoints.update(
                (
                    (head.prev_occurrence, head.ident, "END"),
                    (tail.next_occurrence, tail.ident, "START"),
                )
            )
        local_splits = tuple(
            dict.fromkeys(
                incident
                for incident in splits
                if incident.point_key == point_key
                and incident.event.vertex in dead
                and incident.event.edge in incident_edges
                and incident.met_adjacent
                and incident.met_vertex_id is not None
                and (
                    (
                        incident.target_occurrence,
                        incident.met_vertex_id,
                        "END",
                    )
                    in boundary_endpoints
                    and incident.target_end_id == incident.met_vertex_id
                    or (
                        incident.target_occurrence,
                        incident.met_vertex_id,
                        "START",
                    )
                    in boundary_endpoints
                    and incident.target_start_id == incident.met_vertex_id
                )
            )
        )
        absorbed.update(incident.event for incident in local_splits)
        if local_splits and len(chains) != 1:
            valid = False
        births = []
        for chain in chains:
            head, tail = vertices[chain[0]], vertices[chain[-1]]
            if vertices[tail.next].ident == head.ident:
                continue
            birth = _birth(
                local_edges[0].event.time,
                point_key,
                head.prev_occurrence,
                tail.next_occurrence,
                replaces=chain,
            )
            if birth is None:
                valid = False
                continue
            births.append(birth)
        participants = tuple(
            sorted(
                {
                    participant
                    for incident in (*local_edges, *local_splits)
                    for participant in incident.participants
                }
            )
        )
        kinds = tuple(
            sorted(
                {incident.event.kind for incident in (*local_edges, *local_splits)},
                key=lambda kind: kind.value,
            )
        )
        contacts.append(
            EdgeContactPlanV1(
                events=tuple(
                    incident.event for incident in (*local_edges, *local_splits)
                ),
                time=local_edges[0].event.time,
                point=local_edges[0].event.point,
                point_key=point_key,
                participants=participants,
                dead_vertex_ids=tuple(sorted(dead)),
                chains=chains,
                births=tuple(sorted(births, key=lambda item: item.key)),
                kinds=kinds,
            )
        )
    remaining = tuple(
        incident for incident in splits if incident.event not in absorbed
    )
    return tuple(contacts), remaining, valid


def _reconnect_snapshot(
    meeting: tuple[int, ...], vertices: tuple[_VertexSnapshot, ...]
) -> tuple[tuple[int, int], ...] | None:
    if len(meeting) == 2:
        # Два несоседних exact-time порта имеют единственную cross-пару.
        # Совпавшие входящие/исходящие лучи не создают выбора: выбор лучей
        # нужен только multi-port компоненту с тремя и более вершинами.
        first, second = meeting
        return ((first, second), (second, first))
    incoming: dict[tuple[int, int], list[int]] = {}
    outgoing: dict[tuple[int, int], list[int]] = {}
    for ident in meeting:
        vertex = vertices[ident]
        incoming.setdefault(vertex.incoming_ray, []).append(ident)
        outgoing.setdefault(vertex.outgoing_ray, []).append(ident)
    if len(incoming) != len(meeting) or len(outgoing) != len(meeting):
        return None
    pairs = []
    for ray in sorted(set(incoming) & set(outgoing)):
        pairs.append((incoming.pop(ray)[0], outgoing.pop(ray)[0]))
    rest_in = [ident for group in incoming.values() for ident in group]
    rest_out = [ident for group in outgoing.values() for ident in group]
    if len(rest_in) != len(rest_out) or len(rest_in) > 1:
        return None
    if rest_in:
        pairs.append((rest_in[0], rest_out[0]))
    return tuple(pairs)


def _meeting_plans(
    splits: tuple[SuperlevelIncidentV1, ...],
    vertices: tuple[_VertexSnapshot, ...],
) -> tuple[
    tuple[VertexMeetingPlanV1, ...],
    tuple[SuperlevelIncidentV1, ...],
    tuple[CandidateEventV1, ...],
]:
    grouped: dict[tuple, list[SuperlevelIncidentV1]] = {}
    cuts = []
    fallbacks = []
    for incident in splits:
        if incident.met_vertex_id is None:
            cuts.append(incident)
        elif incident.met_adjacent:
            cuts.append(incident)
            fallbacks.append(incident.event)
        else:
            grouped.setdefault(incident.point_key, []).append(incident)
    meetings = []
    for point_key in sorted(grouped, key=repr):
        incidents = tuple(sorted(grouped[point_key], key=_incident_sort_key))
        meeting = tuple(
            sorted(
                {incident.event.vertex for incident in incidents}
                | {incident.met_vertex_id for incident in incidents}
            )
        )
        touches_self = any(
            vertices[ident].prev in meeting
            or vertices[ident].next in meeting
            for ident in meeting
        )
        pairs = (
            None if touches_self else _reconnect_snapshot(meeting, vertices)
        )
        if pairs is None:
            cuts.extend(incidents)
            fallbacks.extend(incident.event for incident in incidents)
            continue
        planned_births = tuple(
            _birth(
                incidents[0].event.time,
                point_key,
                vertices[incoming].prev_occurrence,
                vertices[outgoing].next_occurrence,
            )
            for incoming, outgoing in pairs
        )
        if any(birth is None for birth in planned_births):
            cuts.extend(incidents)
            fallbacks.extend(incident.event for incident in incidents)
            continue
        births = tuple(
            sorted(planned_births, key=lambda item: item.key)
        )
        participants = tuple(
            sorted(
                {
                    participant
                    for incident in incidents
                    for participant in incident.participants
                }
            )
        )
        meetings.append(
            VertexMeetingPlanV1(
                events=tuple(incident.event for incident in incidents),
                time=incidents[0].event.time,
                point=incidents[0].event.point,
                meeting_vertex_ids=meeting,
                pairs=pairs,
                participants=participants,
                births=births,
            )
        )
    return tuple(meetings), tuple(cuts), tuple(fallbacks)


def _projection_order(
    left: SuperlevelIncidentV1, right: SuperlevelIncidentV1
) -> int:
    difference = left.target_projection - right.target_projection
    order = difference.sign()
    if order:
        return order
    left_key = (
        left.participants,
        left.target_participants,
        repr(left.target_occurrence),
        repr(left.emitter_key),
    )
    right_key = (
        right.participants,
        right.target_participants,
        repr(right.target_occurrence),
        repr(right.emitter_key),
    )
    return (left_key > right_key) - (left_key < right_key)


def _dedupe_split_incidents(
    splits: tuple[SuperlevelIncidentV1, ...],
) -> tuple[tuple[SuperlevelIncidentV1, ...], int, bool]:
    grouped: dict[int, list[SuperlevelIncidentV1]] = {}
    for incident in splits:
        grouped.setdefault(incident.event.vertex, []).append(incident)
    chosen = []
    dropped = 0
    for candidates in grouped.values():
        unique_events = tuple(dict.fromkeys(item.event for item in candidates))
        dropped += len(candidates) - len(unique_events)
        candidates = [
            next(item for item in candidates if item.event == event)
            for event in unique_events
        ]
        if len({candidate.point_key for candidate in candidates}) != 1:
            return (), 0, False
        ordered = sorted(
            candidates,
            key=lambda candidate: (
                candidate.target_participants,
                repr(candidate.target_projection),
                repr(candidate.target_occurrence),
            ),
        )
        if len(ordered) > 1:
            first = (
                ordered[0].target_participants,
                repr(ordered[0].target_projection),
                repr(ordered[0].target_occurrence),
            )
            second = (
                ordered[1].target_participants,
                repr(ordered[1].target_projection),
                repr(ordered[1].target_occurrence),
            )
            if first == second:
                return (), 0, False
        chosen.append(ordered[0])
        dropped += len(ordered) - 1
    return tuple(sorted(chosen, key=_incident_sort_key)), dropped, True


def _split_cut_plans(
    splits: tuple[SuperlevelIncidentV1, ...],
    vertices: tuple[_VertexSnapshot, ...],
) -> tuple[tuple[SplitCutPlanV1, ...], int, bool]:
    chosen, dropped, valid = _dedupe_split_incidents(splits)
    if not valid:
        return (), 0, False
    grouped: dict[tuple, list[SuperlevelIncidentV1]] = {}
    for incident in chosen:
        if incident.target_occurrence is None:
            return (), 0, False
        grouped.setdefault(incident.target_occurrence, []).append(incident)
    plans = []
    for target_occurrence in sorted(grouped, key=repr):
        incidents = grouped[target_occurrence]
        ordered = tuple(sorted(incidents, key=cmp_to_key(_projection_order)))
        first = ordered[0]
        if any(
            incident.event.edge != first.event.edge
            or incident.target_occurrence != target_occurrence
            for incident in ordered
        ):
            return (), 0, False
        try:
            owner_key, span_start, span_end = target_occurrence
        except ValueError:
            return (), 0, False
        points = tuple(item.point_key for item in ordered)
        segment_occurrences = tuple(
            (owner_key, left, right)
            for left, right in zip(
                (span_start, *points), (*points, span_end)
            )
        )
        # Эта транзакция принимает только настоящий interior cut. Endpoint,
        # нулевой, повторный или неизменённый prestate occurrence остаётся
        # fail-closed: отдельного доказанного правила для него здесь нет.
        if (
            any(left == right for _, left, right in segment_occurrences)
            or len(set(segment_occurrences)) != len(segment_occurrences)
            or target_occurrence in segment_occurrences
        ):
            return (), 0, False
        births = []
        final_birth_ports = []
        for index, item in enumerate(ordered):
            emitter = vertices[item.event.vertex]
            left = _birth(
                item.event.time,
                item.point_key,
                emitter.prev_occurrence,
                segment_occurrences[index + 1],
                replaces=(emitter.ident,),
            )
            right = _birth(
                item.event.time,
                item.point_key,
                segment_occurrences[index],
                emitter.next_occurrence,
                replaces=(emitter.ident,),
            )
            if left is None or right is None:
                return (), 0, False
            births.extend((left, right))
            # Interior segment ports уже принадлежат poststate разреза, поэтому
            # повторно переписывать их через target occurrence нельзя.
            final_birth_ports.extend(
                ((left.key, False, True), (right.key, True, False))
            )
        plans.append(
            SplitCutPlanV1(
                edge_id=first.event.edge,
                target_occurrence=target_occurrence,
                events=tuple(item.event for item in ordered),
                segment_occurrences=segment_occurrences,
                births=tuple(sorted(births, key=lambda item: item.key)),
                final_birth_ports=tuple(sorted(final_birth_ports, key=repr)),
            )
        )
    plans.sort(
        key=lambda plan: (
            next(
                (
                    item.target_participants,
                    repr(item.target_occurrence),
                )
                for item in chosen
                if item.target_occurrence == plan.target_occurrence
            ),
            tuple(repr(_event_point_key(event)) for event in plan.events),
        )
    )
    return tuple(plans), dropped, True


def _rewritten_occurrence(
    occurrence: tuple,
    rewrites: dict[tuple, tuple],
) -> tuple:
    return rewrites.get(occurrence, occurrence)


def _rewrite_birth(
    birth: BoundaryBirthV1,
    prev_rewrites: dict[tuple, tuple],
    next_rewrites: dict[tuple, tuple],
    *,
    keep_prev: bool = False,
    keep_next: bool = False,
) -> BoundaryBirthV1:
    prev_occurrence = (
        birth.prev_occurrence
        if keep_prev
        else _rewritten_occurrence(birth.prev_occurrence, prev_rewrites)
    )
    next_occurrence = (
        birth.next_occurrence
        if keep_next
        else _rewritten_occurrence(birth.next_occurrence, next_rewrites)
    )
    return replace(
        birth,
        prev_occurrence=prev_occurrence,
        next_occurrence=next_occurrence,
        key=(
            birth.key[0],
            birth.point_key,
            prev_occurrence,
            next_occurrence,
        ),
    )


def _decompose_birth_function(
    births: tuple[BoundaryBirthV1, ...],
) -> tuple[tuple[tuple[tuple, ...], ...], bool]:
    """Разложить unique partial bijection на anchored chains и cycles."""

    by_prev: dict[tuple, list[BoundaryBirthV1]] = {}
    by_next: dict[tuple, list[BoundaryBirthV1]] = {}
    by_key: dict[tuple, list[BoundaryBirthV1]] = {}
    for birth in births:
        by_prev.setdefault(birth.prev_occurrence, []).append(birth)
        by_next.setdefault(birth.next_occurrence, []).append(birth)
        by_key.setdefault(birth.key, []).append(birth)
    if any(
        len(group) != 1
        for groups in (by_prev, by_next, by_key)
        for group in groups.values()
    ):
        return (), False
    successor = {
        birth.key: by_prev[birth.next_occurrence][0].key
        for birth in births
        if birth.next_occurrence in by_prev
    }
    incoming = set(successor.values())
    seen = set()
    components = []
    for root in sorted(set(by_key) - incoming, key=repr):
        chain = []
        cursor = root
        while cursor not in seen:
            seen.add(cursor)
            chain.append(cursor)
            if cursor not in successor:
                break
            cursor = successor[cursor]
        components.append(tuple(chain))
    for root in sorted(set(by_key) - seen, key=repr):
        if root in seen:
            continue
        cycle = []
        cursor = root
        while cursor not in seen:
            seen.add(cursor)
            cycle.append(cursor)
            cursor = successor.get(cursor)
            if cursor is None:
                return (), False
        if cursor not in cycle:
            return (), False
        offset = min(range(len(cycle)), key=lambda index: repr(cycle[index]))
        components.append(tuple(cycle[offset:] + cycle[:offset]))
    return tuple(sorted(components, key=repr)), len(seen) == len(births)


def _wire_births(
    births: tuple[BoundaryBirthV1, ...],
    dead: set[int],
    vertices: tuple[_VertexSnapshot, ...],
    split_cuts: tuple[SplitCutPlanV1, ...],
) -> tuple[
    tuple[BoundaryBirthV1, ...],
    tuple[tuple[tuple, VertexReferenceV1, VertexReferenceV1], ...],
    tuple[tuple[int, tuple, tuple], ...],
    tuple[tuple[tuple, ...], ...],
    bool,
]:
    """Доказать port matching целиком на frozen prestate до мутации."""

    prev_rewrites = {
        cut.target_occurrence: cut.segment_occurrences[-1]
        for cut in split_cuts
    }
    next_rewrites = {
        cut.target_occurrence: cut.segment_occurrences[0]
        for cut in split_cuts
    }
    final_ports = {
        key: (keep_prev, keep_next)
        for cut in split_cuts
        for key, keep_prev, keep_next in cut.final_birth_ports
    }
    rewritten = tuple(
        sorted(
            (
                _rewrite_birth(
                    birth,
                    prev_rewrites,
                    next_rewrites,
                    keep_prev=final_ports.get(birth.key, (False, False))[0],
                    keep_next=final_ports.get(birth.key, (False, False))[1],
                )
                for birth in births
            ),
            key=lambda item: item.key,
        )
    )
    components, decomposable = _decompose_birth_function(rewritten)
    if not decomposable:
        return rewritten, (), (), (), False

    starts: dict[tuple, list[VertexReferenceV1]] = {}
    ends: dict[tuple, list[VertexReferenceV1]] = {}
    rewrites = []
    for vertex in vertices:
        if not vertex.alive or vertex.ident in dead:
            continue
        prev_occurrence = (
            None
            if vertex.prev_occurrence is None
            else _rewritten_occurrence(vertex.prev_occurrence, prev_rewrites)
        )
        next_occurrence = (
            None
            if vertex.next_occurrence is None
            else _rewritten_occurrence(vertex.next_occurrence, next_rewrites)
        )
        reference = VertexReferenceV1(existing=vertex.ident)
        if next_occurrence is not None:
            starts.setdefault(next_occurrence, []).append(reference)
        if prev_occurrence is not None:
            ends.setdefault(prev_occurrence, []).append(reference)
        if (
            prev_occurrence != vertex.prev_occurrence
            or next_occurrence != vertex.next_occurrence
        ):
            if prev_occurrence is None or next_occurrence is None:
                return rewritten, (), (), (), False
            rewrites.append((vertex.ident, prev_occurrence, next_occurrence))
    for birth in rewritten:
        reference = VertexReferenceV1(birth_key=birth.key)
        starts.setdefault(birth.next_occurrence, []).append(reference)
        ends.setdefault(birth.prev_occurrence, []).append(reference)

    wiring = []
    for birth in rewritten:
        predecessors = starts.get(birth.prev_occurrence, [])
        successors = ends.get(birth.next_occurrence, [])
        if len(predecessors) != 1 or len(successors) != 1:
            return rewritten, (), (), (), False
        predecessor, successor = predecessors[0], successors[0]
        wiring.append((birth.key, predecessor, successor))
    return (
        rewritten,
        tuple(sorted(wiring, key=lambda item: repr(item[0]))),
        tuple(sorted(rewrites)),
        components,
        True,
    )


def _terminal_two_birth_cycles(
    births: tuple[BoundaryBirthV1, ...],
    components: tuple[tuple[tuple, ...], ...],
    wiring: tuple[
        tuple[tuple, VertexReferenceV1, VertexReferenceV1], ...
    ],
) -> tuple[tuple[tuple, ...], ...]:
    """Доказать fully-born reciprocal two-cycle без existing anchors."""

    births_by_key = {birth.key: birth for birth in births}
    wiring_by_key = {
        key: (predecessor, successor)
        for key, predecessor, successor in wiring
    }
    terminal = []
    for component in components:
        if len(component) != 2 or len(set(component)) != 2:
            continue
        first, second = component
        if (
            births_by_key[second].next_occurrence
            != births_by_key[first].prev_occurrence
        ):
            continue
        reciprocal = True
        for key, other in ((first, second), (second, first)):
            predecessor, successor = wiring_by_key[key]
            if (
                predecessor.existing is not None
                or successor.existing is not None
                or predecessor.birth_key != other
                or successor.birth_key != other
            ):
                reciprocal = False
                break
        if reciprocal:
            terminal.append(component)
    return tuple(terminal)


def _composed_initial_plans(contacts, split_cuts, vertices):
    from .symbolic_initial_composition import compose_edge_split_overlap

    return compose_edge_split_overlap(
        contacts, split_cuts, vertices, birth_factory=_birth
    )


def _planned_component(
    component: tuple[SuperlevelIncidentV1, ...],
    vertices: tuple[_VertexSnapshot, ...],
    base: dict,
) -> SuperlevelComponentPlanV1:
    kinds = base["event_kinds"]
    geometric_events: dict[tuple, set[CandidateEventV1]] = {}
    for incident in component:
        geometric_events.setdefault(_incident_sort_key(incident), set()).add(
            incident.event
        )
    unique_incidents = all(
        len(events) == 1 for events in geometric_events.values()
    )
    edges = tuple(
        incident
        for incident in component
        if incident.event.kind is EventKind.EDGE
    )
    splits = tuple(
        incident
        for incident in component
        if incident.event.kind is EventKind.SPLIT
    )
    contacts, remaining, valid = _edge_contact_plans(
        edges, splits, vertices
    )
    meetings, cuts, fallbacks = _meeting_plans(remaining, vertices)
    split_cuts, dropped, cuts_valid = _split_cut_plans(
        cuts, vertices
    )
    (
        contacts,
        split_cuts,
        composed_contact_cut_overlap,
        composition_valid,
    ) = _composed_initial_plans(contacts, split_cuts, vertices)
    contact_dead = {
        ident for contact in contacts for ident in contact.dead_vertex_ids
    }
    meeting_dead = {
        ident for meeting in meetings for ident in meeting.meeting_vertex_ids
    }
    cut_dead = {
        event.vertex for cut in split_cuts for event in cut.events
    }
    valid = (
        valid
        and unique_incidents
        and cuts_valid
        and composition_valid
        and not contact_dead.intersection(meeting_dead)
        and not meeting_dead.intersection(cut_dead)
        and contact_dead.intersection(cut_dead)
        == composed_contact_cut_overlap
        and (not composed_contact_cut_overlap or not meetings)
    )
    dead = contact_dead | meeting_dead | cut_dead
    raw_births = tuple(
        birth for contact in contacts for birth in contact.births
    ) + tuple(
        birth for meeting in meetings for birth in meeting.births
    ) + tuple(
        birth for cut in split_cuts for birth in cut.births
    )
    (
        births,
        birth_wiring,
        existing_port_rewrites,
        birth_components,
        ports_valid,
    ) = _wire_births(
        raw_births,
        dead,
        vertices,
        split_cuts,
    )
    terminal_birth_cycles = _terminal_two_birth_cycles(
        births,
        birth_components,
        birth_wiring,
    )
    # Диагностический witness, не authority на retirement: reciprocal ports
    # не исключают внешний same-time incident на новом occurrence в другой
    # точке. Такой cycle обязан остаться live до полного exact-time fixed point.
    valid = valid and ports_valid
    chains = _snapshot_chains(dead, vertices)
    if contacts:
        base["point"] = contacts[0].point
    if not valid:
        resolution = SuperlevelResolution.UNRESOLVABLE
    elif kinds == (EventKind.EDGE,):
        resolution = SuperlevelResolution.EDGE
    elif kinds == (EventKind.SPLIT,):
        resolution = SuperlevelResolution.SPLIT
    else:
        resolution = SuperlevelResolution.BOUNDARY_PORT_PAIRING
    return SuperlevelComponentPlanV1(
        resolution=resolution,
        dead_vertex_ids=tuple(sorted(dead)),
        chains=chains,
        births=births,
        queue_seed_vertex_ids=(),
        enqueue_born_vertices=bool(births),
        edge_contacts=contacts,
        split_cuts=split_cuts,
        vertex_meetings=meetings,
        proof_fallbacks=fallbacks,
        coincident_split_targets=dropped,
        closed_chain_count=sum(
            vertices[chain[-1]].next == chain[0] for chain in chains
        )
        if not births
        else 0,
        birth_wiring=birth_wiring,
        existing_port_rewrites=existing_port_rewrites,
        birth_components=birth_components,
        terminal_birth_cycles=terminal_birth_cycles,
        suppressed_candidates=0,
        **base,
    )


def _component_plan(
    component: tuple[SuperlevelIncidentV1, ...],
    vertices: tuple[_VertexSnapshot, ...],
) -> SuperlevelComponentPlanV1:
    base = _component_fields(component)
    return _planned_component(component, vertices, base)


def plan_superlevel_components(
    snapshot: SuperlevelSnapshotV1,
) -> tuple[SuperlevelComponentPlanV1, ...]:
    """Построить все component deltas, не меняя ни одного runtime объекта."""

    plans = tuple(
        _component_plan(component, snapshot.vertices)
        for component in _connected_components(snapshot.incidents)
    )
    return tuple(
        sorted(
            plans,
            key=lambda plan: (
                tuple(map(repr, plan.point_keys)),
                tuple(kind.value for kind in plan.event_kinds),
                plan.participants,
            ),
        )
    )


def _record_unresolvable(builder, plan: SuperlevelComponentPlanV1) -> None:
    builder.counters["superlevel_unresolvable_components"] += 1
    builder.refusal = SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    builder._record_obligation(
        cause=ProofObligationBranch.SUPERLEVEL_COMPONENT_UNRESOLVABLE,
        disposition=(
            ProofObligationDisposition.SUPERLEVEL_COMPONENT_UNRESOLVABLE
        ),
        vertex_ids=plan.dead_vertex_ids,
        participant_edge_keys=plan.participants,
        target_edge_keys=plan.target_participants,
        level=plan.time,
        event_kind=EventKind.MULTIWAY,
    )


def _record_duplicate_live_owner(builder, snapshot, level) -> None:
    edge_ids = snapshot.duplicate_live_owner_edge_ids
    vertex_ids = tuple(
        sorted(
            vertex.ident
            for vertex in snapshot.vertices
            if vertex.alive and vertex.next_edge in edge_ids
        )
    )
    participants = builder._edge_keys(*edge_ids)
    builder.counters["superlevel_unresolvable_components"] += 1
    builder.refusal = SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    builder._record_obligation(
        cause=ProofObligationBranch.SUPERLEVEL_COMPONENT_UNRESOLVABLE,
        disposition=(
            ProofObligationDisposition.SUPERLEVEL_COMPONENT_UNRESOLVABLE
        ),
        vertex_ids=vertex_ids,
        participant_edge_keys=participants,
        target_edge_keys=participants,
        level=level[0].time if level else ZERO_TIME,
        event_kind=EventKind.MULTIWAY,
    )


def _record_symbolic_unresolvable(builder, snapshot, reason: str) -> None:
    """Record a coordinator refusal with the complete frozen packet identity."""

    vertex_ids = tuple(sorted(
        vertex.ident for vertex in snapshot.vertices if vertex.alive
    ))
    participants = tuple(sorted({
        participant
        for incident in snapshot.incidents
        for participant in incident.participants
    }))
    targets = tuple(sorted({
        participant
        for incident in snapshot.incidents
        for participant in incident.target_participants
    }))
    builder.counters["superlevel_unresolvable_components"] += 1
    reason_counter = f"superlevel_unresolvable_reason::{reason}"
    builder.counters[reason_counter] = builder.counters.get(reason_counter, 0) + 1
    builder.refusal = SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    builder._record_obligation(
        cause=ProofObligationBranch.SUPERLEVEL_COMPONENT_UNRESOLVABLE,
        disposition=(
            ProofObligationDisposition.SUPERLEVEL_COMPONENT_UNRESOLVABLE
        ),
        vertex_ids=vertex_ids,
        participant_edge_keys=participants,
        target_edge_keys=targets,
        level=(
            snapshot.incidents[0].event.time
            if snapshot.incidents else ZERO_TIME
        ),
        event_kind=EventKind.MULTIWAY,
    )


def _record_unsupported(builder, event: CandidateEventV1) -> None:
    builder.counters["unsupported_event_kind_dropped"] += 1
    valid_vertices = tuple(
        ident
        for ident in (event.vertex, event.peer)
        if 0 <= ident < len(builder.vertices)
    )
    carried_edge = builder._edge_keys(event.edge)
    builder._record_obligation(
        cause=ProofObligationBranch.UNSUPPORTED_EVENT_KIND,
        disposition=ProofObligationDisposition.UNSUPPORTED_EVENT_KIND_DROPPED,
        vertex_ids=valid_vertices,
        participant_edge_keys=carried_edge,
        target_edge_keys=carried_edge,
        level=event.time,
        event_kind=event.kind,
    )


def _record_edge_span_debts(builder, events) -> None:
    for event in events:
        if not event.span_unproven:
            continue
        vertex = builder.vertices[event.vertex]
        peer = builder.vertices[event.peer]
        identity = builder._edge_obligation_identity(vertex, peer)
        builder.counters["edge_collapse_span_unproven_but_accepted"] += 1
        builder._record_obligation(
            cause=ProofObligationBranch.EDGE_COLLAPSE_SPAN_UNPROVEN,
            disposition=(
                ProofObligationDisposition.EVENT_ACCEPTED_WITH_UNPROVEN_SPAN
            ),
            vertex_ids=identity[0],
            participant_edge_keys=identity[1],
            target_edge_keys=identity[2],
            level=event.time,
        )


def _emit_edge_contact(builder, contact: EdgeContactPlanV1) -> None:
    edge_events = tuple(
        event for event in contact.events if event.kind is EventKind.EDGE
    )
    _record_edge_span_debts(builder, edge_events)
    if contact.kinds == (EventKind.EDGE,):
        builder._emit(
            EventKind.EDGE,
            edge_events[0],
            contact.participants,
            contact.dead_vertex_ids,
        )
    else:
        # Один resource-connected contact component даёт одну union incidence;
        # исторические split-first группы сюда не переносятся.
        builder.nodes.append(
            SkeletonNodeV1(
                kind=EventKind.MULTIWAY,
                time=contact.time,
                point=contact.point,
                participants=contact.participants,
                converging_vertices=len(contact.dead_vertex_ids),
                kinds=contact.kinds,
                incidences=(contact.participants,),
            )
        )
        builder._node_vertex_ids.append(contact.dead_vertex_ids)
        builder.counters["split_events"] += 1
    builder.counters["edge_events"] += 1
    if len(contact.participants) > 3:
        builder.counters["multi_participant_nodes"] += 1


def _resolve_vertex(builder, reference, born_by_key):
    if reference.existing is not None:
        return builder.vertices[reference.existing]
    return born_by_key[reference.birth_key]


def _emit_meeting(builder, meeting) -> None:
    builder._emit(
        EventKind.SPLIT,
        meeting.events[0],
        meeting.participants,
        meeting.meeting_vertex_ids,
    )
    builder.counters["vertex_meeting_events"] += 1
    if len(meeting.participants) > 3:
        builder.counters["multi_participant_nodes"] += 1


def _emit_component_nodes(builder, plan) -> None:
    for contact in plan.edge_contacts:
        _emit_edge_contact(builder, contact)
    for meeting in plan.vertex_meetings:
        _emit_meeting(builder, meeting)
    for cut in plan.split_cuts:
        edge = builder.edges[cut.edge_id]
        for event in cut.events:
            builder._emit_split_node(builder.vertices[event.vertex], edge, event)


def _record_fallbacks(builder, plans) -> None:
    from .candidate_refusal import CandidateRefusal

    for plan in plans:
        for event in plan.proof_fallbacks:
            vertex = builder.vertices[event.vertex]
            endpoints = builder._proof_edge_endpoint_ids(event.edge)
            builder._refuse(
                CandidateRefusal.NO_RULE_MEETING_NOT_RECONNECTABLE,
                vertex_ids=(vertex.ident, *endpoints),
                participant_edge_keys=builder._edge_keys(
                    vertex.prev_edge, vertex.next_edge, event.edge
                ),
                target_edge_keys=builder._edge_keys(event.edge),
            )


def _seed_key(builder, vertex) -> tuple:
    return (
        builder.edges[vertex.prev_edge].key,
        builder.edges[vertex.next_edge].key,
        vertex.point.x.terms,
        vertex.point.y.terms,
    )


def _unique_vertices(vertices) -> tuple:
    return tuple({vertex.ident: vertex for vertex in vertices}.values())


def _existing_edges_by_occurrence(snapshot, plans):
    new_occurrences = {
        occurrence
        for plan in plans
        for cut in plan.split_cuts
        for occurrence in cut.segment_occurrences
    }
    required = {
        occurrence
        for plan in plans
        for birth in plan.births
        for occurrence in (birth.prev_occurrence, birth.next_occurrence)
        if occurrence not in new_occurrences
    } | {
        cut.target_occurrence for plan in plans for cut in plan.split_cuts
    } | {
        occurrence
        for plan in plans
        for _, prev_occurrence, next_occurrence in plan.existing_port_rewrites
        for occurrence in (prev_occurrence, next_occurrence)
        if occurrence not in new_occurrences
    }
    grouped: dict[tuple, set[int]] = {}
    for vertex in snapshot.vertices:
        if vertex.alive and vertex.next_occurrence is not None:
            grouped.setdefault(vertex.next_occurrence, set()).add(vertex.next_edge)
    resolved = {}
    for occurrence in required:
        owners = grouped.get(occurrence, set())
        if len(owners) != 1:
            return None
        resolved[occurrence] = next(iter(owners))
    return resolved


def _commit_prestate(snapshot, plans):
    edge_by_occurrence = _existing_edges_by_occurrence(snapshot, plans)
    if edge_by_occurrence is None:
        return None
    new_occurrences = [
        occurrence
        for plan in plans
        for cut in plan.split_cuts
        for occurrence in cut.segment_occurrences
    ]
    birth_keys = [birth.key for plan in plans for birth in plan.births]
    if (
        len(set(new_occurrences)) != len(new_occurrences)
        or any(occurrence in edge_by_occurrence for occurrence in new_occurrences)
        or len(set(birth_keys)) != len(birth_keys)
    ):
        return None
    available = set(edge_by_occurrence) | set(new_occurrences)
    if any(
        occurrence not in available
        for plan in plans
        for birth in plan.births
        for occurrence in (birth.prev_occurrence, birth.next_occurrence)
    ):
        return None
    return edge_by_occurrence


def _birth_origins(plan) -> dict[tuple, tuple[EventTimeV1, EventPointV1]]:
    points = {_event_point_key(event): event.point for event in plan.events}
    return {
        birth.key: (plan.time, points[birth.point_key]) for birth in plan.births
    }


def _commit_plans(builder, plans, edge_by_occurrence) -> None:
    born_by_key = {}
    born = []
    edge_seeds = []
    affected_edges = []
    _record_fallbacks(builder, plans)

    for plan in plans:
        _emit_component_nodes(builder, plan)
        builder.counters["coincident_split_targets"] += (
            plan.coincident_split_targets
        )
        builder.counters["discarded_stale_candidates"] += (
            plan.suppressed_candidates
        )
        builder.counters["peaks"] += plan.closed_chain_count
        for cut in plan.split_cuts:
            edge = builder.edges[cut.edge_id]
            segments = [builder._twin(edge) for _ in cut.events] + [edge]
            for occurrence, segment in zip(cut.segment_occurrences, segments):
                assert occurrence not in edge_by_occurrence
                edge_by_occurrence[occurrence] = segment.ident
                affected_edges.append((occurrence, segment))

    origins = {}
    for plan in plans:
        for key, origin in _birth_origins(plan).items():
            assert key not in origins
            origins[key] = origin
    for plan in plans:
        for birth in plan.births:
            prev_edge = edge_by_occurrence.get(birth.prev_occurrence)
            next_edge = edge_by_occurrence.get(birth.next_occurrence)
            assert prev_edge is not None and next_edge is not None
            affected_edges.extend(
                (
                    (birth.prev_occurrence, builder.edges[prev_edge]),
                    (birth.next_occurrence, builder.edges[next_edge]),
                )
            )
            time, point = origins[birth.key]
            placeholder = len(builder.vertices)
            vertex = builder._new_vertex(
                prev_edge=prev_edge,
                next_edge=next_edge,
                prev=placeholder,
                next=placeholder,
                birth=time,
                point=point,
            )
            born_by_key[birth.key] = vertex
            born.append(vertex)

    for plan in plans:
        for ident in plan.dead_vertex_ids:
            builder.vertices[ident].alive = False
        for ident, prev_occurrence, next_occurrence in plan.existing_port_rewrites:
            vertex = builder.vertices[ident]
            vertex.prev_edge = edge_by_occurrence[prev_occurrence]
            vertex.next_edge = edge_by_occurrence[next_occurrence]
            builder._register(vertex)
        for key, predecessor_ref, successor_ref in plan.birth_wiring:
            vertex = born_by_key[key]
            predecessor = _resolve_vertex(builder, predecessor_ref, born_by_key)
            successor = _resolve_vertex(builder, successor_ref, born_by_key)
            vertex.prev = predecessor.ident
            vertex.next = successor.ident
            predecessor.next = vertex.ident
            successor.prev = vertex.ident
            builder._register(vertex)
            if predecessor_ref.existing is not None:
                edge_seeds.append(predecessor)

    for vertex in sorted(
        (vertex for vertex in _unique_vertices(born) if vertex.alive),
        key=lambda item: _seed_key(builder, item),
    ):
        builder._enqueue_for(vertex)
    born_vertex_ids = frozenset(vertex.ident for vertex in born)
    for vertex in sorted(
        _unique_vertices(edge_seeds), key=lambda item: _seed_key(builder, item)
    ):
        builder._enqueue_edge_event(vertex)
    # Любой edge, у которого изменился хотя бы один live span endpoint, обязан
    # заново породить split candidates. Иначе одинаковый poststate может иметь
    # разную будущую queue completeness в зависимости от типа t-contact.
    unique_affected = {}
    for occurrence, edge in affected_edges:
        prior = unique_affected.get(edge.ident)
        if prior is None or repr(occurrence) < repr(prior[0]):
            unique_affected[edge.ident] = (occurrence, edge)
    for _, edge in sorted(
        unique_affected.values(), key=lambda item: repr(item[0])
    ):
        # Born vertices уже полностью посеяны через `_enqueue_for`; повторный
        # affected-edge scan обязан искать только внешние reflex vertices.
        builder._enqueue_splits_against(
            edge,
            excluded_vertex_ids=born_vertex_ids,
        )


def apply_superlevel_transaction(
    builder, level: tuple[CandidateEventV1, ...]
) -> None:
    """Plan/validate всё на frozen prestate, затем atomically commit deltas."""

    snapshot = collect_superlevel_snapshot(builder, level)
    builder.counters["discarded_stale_candidates"] += (
        snapshot.stale_candidates
    )
    for event in snapshot.unsupported:
        _record_unsupported(builder, event)
    if snapshot.duplicate_live_owner_edge_ids:
        _record_duplicate_live_owner(builder, snapshot, level)
        return
    if not snapshot.incidents:
        return

    from .symbolic_runtime_commit import (
        materialize_symbolic_runtime_commit,
        plan_symbolic_runtime_commit,
    )
    from .symbolic_superlevel_coordinator import (
        plan_symbolic_superlevel_closure,
    )

    # Число живых frozen ports ограничивает число причинных поколений; запас
    # покрывает interior births, не превращая исчерпание в последовательный
    # fallback. Любое исчерпание остаётся именованным отказом coordinator.
    budget = max(8, 2 * len(snapshot.vertices) + len(snapshot.incidents))
    closure = plan_symbolic_superlevel_closure(
        builder,
        snapshot,
        outer_budget=budget,
        junction_budget=budget,
    )
    plans = (
        () if closure.materialization is None
        else closure.materialization.plans
    )
    if closure.unresolved_reason is not None or not plans:
        _record_symbolic_unresolvable(
            builder,
            snapshot,
            closure.unresolved_reason
            or "SYMBOLIC_SUPERLEVEL_MATERIALIZATION_UNAVAILABLE",
        )
        return
    commit, reason = plan_symbolic_runtime_commit(builder, snapshot, closure)
    if reason is not None or commit is None:
        _record_symbolic_unresolvable(
            builder,
            snapshot,
            reason or "SYMBOLIC_RUNTIME_COMMIT_PLAN_UNAVAILABLE",
        )
        return
    commit_reason = materialize_symbolic_runtime_commit(
        builder, snapshot, commit
    )
    if commit_reason is not None:
        _record_symbolic_unresolvable(
            builder, snapshot, commit_reason
        )


def has_same_time_residual(queue: EventQueueV1, now: EventTimeV1) -> bool:
    """Есть ли exact-time packet, который обязан войти в текущий superlevel."""

    upcoming = queue.peek_time()
    return upcoming is not None and compare_times(upcoming, now) == 0


def _place_key(node) -> tuple:
    time = node.time.canonical()
    return (
        time.dividend,
        time.divisor.terms,
        node.point.x.terms,
        node.point.y.terms,
    )


def _incidence_components(nodes: tuple) -> tuple[tuple[int, ...], ...]:
    """Связные компоненты по общему participant key, включая транзитивность."""

    pending = set(range(len(nodes)))
    components = []
    while pending:
        component = {min(pending)}
        participants = set(nodes[min(component)].participants)
        changed = True
        while changed:
            joined = {
                index
                for index in pending - component
                if participants.intersection(nodes[index].participants)
            }
            changed = bool(joined)
            component.update(joined)
            participants.update(
                participant
                for index in joined
                for participant in nodes[index].participants
            )
        pending.difference_update(component)
        components.append(tuple(sorted(component)))
    return tuple(components)


def validate_multiway_node(node) -> tuple:
    """Canonical original kinds/incidences либо именованная schema error."""

    canonical = tuple(sorted(set(node.kinds), key=lambda kind: kind.value))
    if not canonical:
        raise ValueError("MULTIWAY_NODE_KINDS_UNAVAILABLE")
    if EventKind.MULTIWAY in canonical or node.kinds != canonical:
        raise ValueError("MULTIWAY_NODE_KINDS_NOT_CANONICAL")
    if not node.incidences:
        raise ValueError("MULTIWAY_NODE_INCIDENCE_UNAVAILABLE")
    if any(not incidence for incidence in node.incidences):
        raise ValueError("MULTIWAY_NODE_INCIDENCE_EMPTY")
    incidences = tuple(
        sorted(tuple(sorted(set(incidence))) for incidence in node.incidences)
    )
    if node.incidences != incidences:
        raise ValueError("MULTIWAY_NODE_INCIDENCES_NOT_CANONICAL")
    incidence_union = tuple(
        sorted(
            {
                participant
                for incidence in incidences
                for participant in incidence
            }
        )
    )
    if incidence_union != node.participants:
        raise ValueError("MULTIWAY_NODE_INCIDENCE_UNION_MISMATCH")
    return canonical, incidences


def _original_kinds(node) -> tuple[EventKind, ...]:
    return (
        (node.kind,)
        if node.kind is not EventKind.MULTIWAY
        else validate_multiway_node(node)[0]
    )


def _original_incidences(node) -> tuple[tuple[tuple[int, ...], ...], ...]:
    return (
        (node.participants,)
        if node.kind is not EventKind.MULTIWAY
        else validate_multiway_node(node)[1]
    )


def accumulate_nodes(
    nodes: tuple, converged_vertex_ids: tuple[tuple[int, ...], ...]
) -> tuple:
    """Один node на incidence component одного exact `(time, point)`."""

    if len(nodes) != len(converged_vertex_ids):
        raise ValueError("NODE_ACCUMULATOR_IDENTITY_LENGTH_MISMATCH")
    by_place: dict[tuple, list[int]] = {}
    for index, node in enumerate(nodes):
        by_place.setdefault(_place_key(node), []).append(index)
    accumulated = []
    for indices in by_place.values():
        placed = tuple(nodes[index] for index in indices)
        for local_component in _incidence_components(placed):
            component = tuple(indices[index] for index in local_component)
            first = component[0]
            if len(component) == 1:
                accumulated.append((first, nodes[first]))
                continue
            kinds = tuple(
                sorted(
                    {
                        original
                        for index in component
                        for original in _original_kinds(nodes[index])
                    },
                    key=lambda kind: kind.value,
                )
            )
            participants = tuple(
                sorted(
                    {
                        participant
                        for index in component
                        for participant in nodes[index].participants
                    }
                )
            )
            vertices = {
                ident
                for index in component
                for ident in converged_vertex_ids[index]
            }
            accumulated.append(
                (
                    first,
                    replace(
                        nodes[first],
                        kind=EventKind.MULTIWAY,
                        participants=participants,
                        converging_vertices=len(vertices),
                        kinds=kinds,
                        incidences=tuple(
                            sorted(
                                incidence
                                for index in component
                                for incidence in _original_incidences(nodes[index])
                            )
                        ),
                    ),
                )
            )
    return tuple(node for _, node in sorted(accumulated, key=lambda item: item[0]))
