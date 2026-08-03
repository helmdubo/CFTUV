"""Pure immediate EDGE contact discovery over a frozen symbolic overlay."""

from __future__ import annotations

from dataclasses import dataclass

from .candidate_law import evaluate_edge_candidate
from .event_time import compare_times
from . import superlevel as base
from .superlevel_closure import SegmentRefV1, SpanFamilyRefV1
from .symbolic_overlay import (
    JunctionRefV1,
    SymbolicOverlayV1,
    exact_overlay_view,
)


@dataclass(frozen=True, slots=True)
class SymbolicEdgeContactKeyV1:
    time_key: tuple
    point_key: tuple
    start: JunctionRefV1
    end: JunctionRefV1
    prev_family: SpanFamilyRefV1
    shared_family: SpanFamilyRefV1
    next_family: SpanFamilyRefV1


@dataclass(frozen=True, slots=True)
class SymbolicEdgeContactV1:
    key: SymbolicEdgeContactKeyV1
    prev_leaf: SegmentRefV1
    shared_leaf: SegmentRefV1
    next_leaf: SegmentRefV1
    span_unproven: bool
    participant_keys: tuple[tuple[int, ...], ...]


def _point_key(point):
    return (point.x.terms, point.y.terms)


def _participants(*leaves):
    return tuple(
        sorted(
            {
                participant
                for leaf in leaves
                for participant in leaf.family.participant_keys
            }
        )
    )


def discover_symbolic_edge_contacts(
    builder, overlay: SymbolicOverlayV1
) -> tuple[SymbolicEdgeContactV1, ...]:
    """Return every immediate same-time contact without changing ``overlay``."""

    view = exact_overlay_view(builder, overlay)
    contacts = []
    for ref in sorted(overlay.vertices, key=repr):
        vertex = overlay.vertices[ref]
        if not vertex.alive or vertex.next is None:
            continue
        peer = overlay.vertices.get(vertex.next)
        if peer is None or not peer.alive or peer.prev != ref:
            continue
        if (
            vertex.next_leaf not in overlay.changed
            and vertex.ref.kind == "EXISTING"
            and peer.ref.kind == "EXISTING"
        ):
            continue
        decision = evaluate_edge_candidate(
            view,
            vertex.ref,
            peer.ref,
            now=overlay.time,
            same_vertex=vertex.ref == peer.ref,
        )
        candidate = decision.candidate
        if (
            candidate is None
            or compare_times(candidate.time, overlay.time) != 0
        ):
            continue
        key = SymbolicEdgeContactKeyV1(
            base._time_key(candidate.time),
            _point_key(candidate.point),
            vertex.ref,
            peer.ref,
            vertex.prev_leaf.family,
            vertex.next_leaf.family,
            peer.next_leaf.family,
        )
        contacts.append(
            SymbolicEdgeContactV1(
                key,
                vertex.prev_leaf,
                vertex.next_leaf,
                peer.next_leaf,
                candidate.span_unproven,
                _participants(
                    vertex.prev_leaf,
                    vertex.next_leaf,
                    peer.next_leaf,
                ),
            )
        )
    return tuple(sorted(contacts, key=lambda item: repr(item.key)))
