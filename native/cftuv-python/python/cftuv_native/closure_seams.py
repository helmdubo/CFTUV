"""Python side of the symbolic-closure seams (test-only, WP-S5): the wire of `native/cftuv-skeleton/src/seam_closure.rs`.

Eight seams, opcodes 310..317: `PLAN_SYMBOLIC_CLOSURE` (`plan_symbolic_superlevel_closure`: the whole closure of a packet), `BUILD_SYMBOLIC_OVERLAY`, `DISCOVER_INTERIOR_CONTACTS`
(`discover_interior_split_contacts`), `PLAN_MIXED_GENERATIONS`, `DISCOVER_JUNCTION_CONTACTS`, `APPLY_COMPONENT_DELTAS` and `CLOSURE_PART` (`with_line_ports`, `build_f0_overlay`,
`initial_interior_contacts`, `overlay_signature`) and `NORMALIZE_MIXED_GENERATION` (`normalize_mixed_generation` and `apply_mixed_generation` on contacts the caller gives). This module encodes oracle objects (the front with its traces, an overlay, a materialization, a delta) into the wire and brings
what the oracle returned to the CANONICAL TEXT the native answer is, so that a comparison cannot differ in the normalisation, only in the content.

THE CANONICAL TEXT is `repr` with three differences, all of which are places where `repr` is not a function of the value: a `set` or a `frozenset` is printed in the order of the
`repr` of its members (CPython prints it in the order of its hash table, which depends on the hash seed and, for a `None` inside a tuple, on an address); the `spans` of an
overlay are printed in the order of the `repr` of their keys (the oracle builds them from a set); the `trace` of a vertex is printed as a marker (the whole `TraceV1` is the
builder's, not the overlay's). Dictionaries keep their insertion order: the vertices of an overlay are compared in the order the oracle made them.
"""

from __future__ import annotations

import dataclasses
from enum import Enum

from . import builder_seams as bseams
from . import skeleton_seams as wire
from .clip_seams import SeamUnsupported

__all__ = ("bare_state", "canon_text", "enc_closure_state", "enc_delta", "enc_interior_contact", "enc_junction_contact", "enc_materialization", "enc_overlay", "enc_vertices")

TRACE_MARKER = "<trace>"


def _val(value) -> list:
    return wire.enc_repr(value)


def _opt(value):
    return None if value is None else _val(value)


# --------------------------------------------------------------------------
# the canonical text
# --------------------------------------------------------------------------


def _items(values) -> list:
    return [canon_text(item) for item in values]


def _dataclass_text(value) -> str:
    cls = type(value).__qualname__
    parts = []
    for item in dataclasses.fields(value):
        if not item.repr:
            continue
        found = getattr(value, item.name)
        if cls == "SymbolicVertexV1" and item.name == "trace":
            text = "None" if found is None else repr(TRACE_MARKER)
        elif cls == "SymbolicOverlayV1" and item.name == "spans":
            pairs = sorted(((canon_text(key), canon_text(entry)) for key, entry in found.items()), key=lambda pair: pair[0])
            text = "{" + ", ".join(f"{key}: {entry}" for key, entry in pairs) + "}"
        else:
            text = canon_text(found)
        parts.append(f"{item.name}={text}")
    return f"{cls}({', '.join(parts)})"


def canon_text(value) -> str:
    """The canonical text of an oracle value (see the module note)."""

    if isinstance(value, (set, frozenset)):
        frozen = isinstance(value, frozenset)
        if not value:
            return "frozenset()" if frozen else "set()"
        body = "{" + ", ".join(sorted(_items(value))) + "}"
        return f"frozenset({body})" if frozen else body
    if isinstance(value, tuple):
        inner = ", ".join(_items(value))
        return f"({inner},)" if len(value) == 1 else f"({inner})"
    if type(value) is list:
        return "[" + ", ".join(_items(value)) + "]"
    if isinstance(value, dict):
        return "{" + ", ".join(f"{canon_text(key)}: {canon_text(entry)}" for key, entry in value.items()) + "}"
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _dataclass_text(value)
    if isinstance(value, Enum):
        return repr(value)
    return repr(value)


# --------------------------------------------------------------------------
# the state
# --------------------------------------------------------------------------


def enc_closure_state(builder, memo=None) -> list:
    """The state of `builder_seams.enc_builder_state` (the front, the memory of places, the prime universe) with the traces of the builder."""

    state = bseams.enc_builder_state(builder, light=True, memo=memo)
    state[12] = [[vertex, wire.enc_trace(trace)] for vertex, trace in builder.traces.items()]
    return state


def bare_state(time) -> list:
    """A state with no front (the seams that read no builder: `APPLY_COMPONENT_DELTAS`)."""

    return [[], [], [], [], [], *([None] * 11), wire.enc_time(time), [False, []], None, bseams.NEXT_IDENT]


def enc_vertices(vertices) -> list:
    """The vertices of a frozen snapshot (`_VertexSnapshot`)."""

    return [bseams.enc_vertex_snapshot(vertex) for vertex in vertices]


# --------------------------------------------------------------------------
# an overlay
# --------------------------------------------------------------------------


def _authority(vertex, builder):
    trace = vertex.trace
    if builder is not None:
        expected = None if vertex.runtime_id is None else builder.traces.get(vertex.runtime_id)
        if trace is not expected:
            raise SeamUnsupported("a vertex of the overlay holds a trace that is not the trace of the builder of its runtime id")
    if trace is None:
        return None
    return [] if trace.crash_time is None else [wire.enc_time(trace.crash_time)]


def enc_overlay(overlay, builder=None) -> list:
    """`[vertices, spans, changed, time]` of a `SymbolicOverlayV1` (the vertices in the order of the oracle's dictionary)."""

    vertices = []
    for vertex in overlay.vertices.values():
        sliding = None if vertex.sliding is None else [vertex.sliding, id(vertex.sliding)]
        vertices.append(
            [
                _val(vertex.ref),
                _opt(vertex.prev),
                _opt(vertex.next),
                _val(vertex.prev_leaf),
                _val(vertex.next_leaf),
                wire.enc_time(vertex.birth),
                None if vertex.point is None else wire.enc_point(vertex.point),
                sliding,
                [_val(key) for key in sorted(vertex.provenance, key=repr)],
                vertex.runtime_id,
                _authority(vertex, builder),
                vertex.alive,
            ]
        )
    spans = [[_val(leaf), binding.physical_edge_id, _opt(binding.start), _opt(binding.end)] for leaf, binding in overlay.spans.items()]
    return [vertices, spans, [_val(leaf) for leaf in sorted(overlay.changed, key=repr)], wire.enc_time(overlay.time)]


# --------------------------------------------------------------------------
# a materialization and a delta
# --------------------------------------------------------------------------


def _keys(keys) -> list:
    return [list(key) for key in keys]


def _enc_birth(birth) -> list:
    return [_val(birth.point_key), _val(birth.prev_occurrence), _val(birth.next_occurrence), _val(birth.key), list(birth.replaces)]


def _enc_reference(reference) -> list:
    return [reference.existing, _opt(reference.birth_key)]


def _enc_plan(plan) -> list:
    return [
        wire.enc_time(plan.time),
        [bseams.enc_event(event) for event in plan.events],
        list(plan.dead_vertex_ids),
        [_enc_birth(birth) for birth in plan.births],
        [[_val(key), _enc_reference(predecessor), _enc_reference(successor)] for key, predecessor, successor in plan.birth_wiring],
        [[ident, _val(prev), _val(next_)] for ident, prev, next_ in plan.existing_port_rewrites],
        [[cut.edge_id, _val(cut.target_occurrence), [_val(item) for item in cut.segment_occurrences]] for cut in plan.split_cuts],
        [[[_enc_birth(birth) for birth in contact.births], _keys(contact.participants), [wire.enc_str(kind.value) for kind in contact.kinds]] for contact in plan.edge_contacts],
    ]


def enc_materialization(materialization) -> list:
    """`[plans, families]` of a `SymbolicMaterializationPlanV1`: only what `build_symbolic_overlay` reads."""

    return [
        [_enc_plan(plan) for plan in materialization.plans],
        [[[_val(segment) for segment in family.segments], [_val(birth) for birth in family.births]] for family in materialization.families],
    ]


def _enc_port(port) -> list:
    return [_opt(port[0]), _val(port[1])]


def enc_delta(delta) -> list:
    """`[contact keys, dead refs, incoming, outgoing, birth ref, point key, leaf resources, rewires]` of a `SymbolicComponentDeltaV1`."""

    return [
        [_val(key) for key in delta.contact_keys],
        [_val(ref) for ref in delta.dead_refs],
        None if delta.incoming is None else _enc_port(delta.incoming),
        None if delta.outgoing is None else _enc_port(delta.outgoing),
        _opt(delta.birth_ref),
        _val(delta.point_key),
        [_val(leaf) for leaf in sorted(delta.leaf_resources, key=repr)],
        [[_enc_port(incoming), _enc_port(outgoing), _val(birth)] for incoming, outgoing, birth in delta.rewires],
    ]


# --------------------------------------------------------------------------
# contacts
# --------------------------------------------------------------------------


def enc_interior_contact(contact) -> list:
    """`[key, time, point, projection, leaf | none]` of a `SymbolicSplitContactV1`."""

    return [_val(contact.key), wire.enc_time(contact.time), wire.enc_point(contact.point), contact.projection, _opt(contact.leaf)]


def enc_junction_contact(contact) -> list:
    """`[kind, key, dead refs, families, edge | none]` of a `SymbolicJunctionContactV1`."""

    edge = contact.edge
    encoded = None
    if edge is not None:
        encoded = [_val(edge.key), _val(edge.prev_leaf), _val(edge.shared_leaf), _val(edge.next_leaf), bool(edge.span_unproven), _keys(edge.participant_keys)]
    return [wire.enc_str(contact.kind), _val(contact.key), [_val(ref) for ref in contact.dead_refs], [_val(family) for family in sorted(contact.families, key=repr)], encoded]
