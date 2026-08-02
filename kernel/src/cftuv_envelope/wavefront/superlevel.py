"""Exact-time utilities shared by node accumulation and level draining."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum

from .event_time import EventPointV1, EventTimeV1
from .events import EventKind
from .proof import ProofObligationV1, ProofStatus


class SkeletonOutcome(str, Enum):
    """Чем кончилось построение. Тихого выхода нет ни одного."""

    EXACT = "EXACT"
    LEVEL_BUDGET_EXHAUSTED = "LEVEL_BUDGET_EXHAUSTED"
    MULTIWAY_SPLIT_UNPROVEN = "MULTIWAY_SPLIT_UNPROVEN"
    WAVEFRONT_LEFT_UNRESOLVED = "WAVEFRONT_LEFT_UNRESOLVED"


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
                        for original in (
                            nodes[index].kinds
                            if nodes[index].kind is EventKind.MULTIWAY
                            else (nodes[index].kind,)
                        )
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
                            nodes[index].participants for index in component
                        ),
                    ),
                )
            )
    return tuple(node for _, node in sorted(accumulated, key=lambda item: item[0]))
