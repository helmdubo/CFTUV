"""Python side of the builder seams (test-only, WP-S3 and WP-S4): the wire of `native/cftuv-skeleton/src/seam_builder.rs`.

Five seams, opcodes 251..255: `BUILDER_INIT` (the state of `_Builder` after `__init__`), `BUILDER_RUN` (the event loop from the start or from a recorded state up to the next
call of the superlevel transaction, or to the result), `COLLECT_SNAPSHOT` (`collect_superlevel_snapshot` on a recorded front), `PLAN_COMPONENTS` (`plan_superlevel_components`) and
`PLAN_SPLIT_MATERIALIZATION` (`superlevel_closure.plan_split_materialization`). This module encodes oracle objects into the wire, turns an oracle builder and a native state into
the SAME canonical shape (so a comparison cannot differ in the normalisation, only in the content), and decodes the native answers into the oracle's own types.

THE STATE. `enc_builder_state(builder)` is the whole `_Builder` (see the Rust module note for the positions); a seam that needs less is given `light=True` (the front and the memory
only: the other positions are `None`). The identity of a Python object (`id(line)`, `id(vertex.sliding)`) travels as it is, because which lookups of the superlevel memory hit
decides the sign counters; the comparison renumbers identities by first appearance, so that the oracle's addresses and the native numbers are comparable (two edges share a line
in one exactly when they do in the other).

THE PLAN SEAMS answer the `repr` TEXT of what the oracle's function returned: a dataclass prints every field and an `int` differs from a `Fraction` in a `repr`, so the comparison
is byte for byte (`repr(oracle result) == native text`).
"""

from __future__ import annotations

from types import SimpleNamespace

from . import skeleton_seams as wire
from .clip_seams import SeamUnsupported, dec_str, enc_str

__all__ = (
    "canon_state",
    "dec_skeleton",
    "enc_builder_state",
    "enc_event",
    "enc_level",
    "enc_options",
    "enc_snapshot",
    "ORACLE_UNSUPPORTED_MARK",
    "skeleton_wire",
)

#: The text a refusal of the port begins with where the oracle raises `TypeError` from a sort (an internal error of the oracle that the port answers by name, never by guess). The births and
#: their ports are ordered by `identity_order_key` now (a `None` end of an occurrence sorts last), so a real run no longer reaches such a refusal there; the mark stays for the inputs
#: the harness makes up and for the sorts that remain plain.
ORACLE_UNSUPPORTED_MARK = "TypeError in the oracle"

#: The next identity the native builder gives to a line or a sliding projection it makes: far above any address of the oracle's objects (a `repr` of an `id` is below 2**48).
NEXT_IDENT = 1 << 62


def replay_check_now() -> bool:
    """The oracle's `replay_check_enabled()` now (the environment variable `CFTUV_SYMBOLIC_REPLAY_CHECK`): whether its closure replays a packet a second time."""

    from cftuv_envelope.wavefront.symbolic_superlevel_coordinator import replay_check_enabled

    return bool(replay_check_enabled())


def enc_options(dense: bool, march_steps, budgeted: bool, replay=None) -> list:
    """`[dense hydration, march steps | None, budgeted, replay check]`; `replay=None` is what the oracle does NOW (`replay_check_now`), so a call recorded under the variable replays."""

    return [bool(dense), march_steps, bool(budgeted), replay_check_now() if replay is None else bool(replay)]


def enc_event(event) -> list:
    return [enc_str(event.kind.value), wire.enc_time(event.time), wire.enc_point(event.point), event.vertex, event.peer, event.edge, bool(event.span_unproven)]


def enc_level(level) -> list:
    return [enc_event(event) for event in level]


def _keys(keys) -> list:
    return [list(key) for key in keys]


def enc_node(node) -> list:
    return [
        enc_str(node.kind.value),
        wire.enc_time(node.time),
        wire.enc_point(node.point),
        _keys(node.participants),
        node.converging_vertices,
        [enc_str(kind.value) for kind in node.kinds],
        [_keys(incidence) for incidence in node.incidences],
    ]


def enc_obligation(obligation) -> list:
    from cftuv_envelope.wavefront import proof

    kind = 1 if isinstance(obligation.cause, proof.ProofObligationBranch) else 0
    return [
        kind,
        enc_str(obligation.cause.value),
        enc_str(obligation.disposition.value),
        list(obligation.vertex_ids),
        _keys(obligation.participant_edge_keys),
        _keys(obligation.target_edge_keys),
        wire.enc_time(obligation.level),
        None if obligation.event_kind is None else enc_str(obligation.event_kind.value),
    ]


def _cells(cells) -> list:
    return [[column, row] for column, row in cells]


def _buckets(index) -> list:
    return [[cell[0], cell[1], list(idents)] for cell, idents in index.buckets.items()]


def enc_index(index) -> list:
    return [
        wire.enc_grid(index.grid),
        index.speed_bound,
        [[key, _cells(cells)] for key, cells in index.line_cells.items()],
        [[vertex, _cells(cells)] for vertex, cells in index.vertex_cells.items()],
        _buckets(index.lines),
        _buckets(index.vertices),
    ]


def memo_all(memo) -> list:
    """`[active, entries]` of the WHOLE memory of places (every entry as a hit): what a run resumed from a recorded state needs."""

    log = SimpleNamespace(hits=list(memo.entries.items())) if memo is not None else None
    return wire.memo_wire(memo, log, memo is not None)


def enc_builder_state(builder, *, light: bool = False, memo=None) -> list:
    """The whole `_Builder` as the wire of the builder seams. `light`: only the front, the prime universe and the memory (`memo` is then the `[active, entries]` the call hit)."""

    edges = [[edge.ident, wire.enc_line(edge.line), list(edge.span)] for edge in builder.edges]
    vertices = [
        [
            vertex.ident,
            vertex.prev_edge,
            vertex.next_edge,
            vertex.prev,
            vertex.next,
            wire.enc_time(vertex.birth),
            wire.enc_point(vertex.point),
            vertex.reflex,
            vertex.alive,
            None if vertex.sliding is None else [vertex.sliding, id(vertex.sliding)],
        ]
        for vertex in builder.vertices
    ]
    head = [[int(prime) for prime in builder._prime_universe], edges, vertices, sorted([key, value] for key, value in builder.edge_start.items()), sorted([key, value] for key, value in builder.edge_end.items())]
    memo = memo if memo is not None else memo_all(builder._position_memo)
    if light:
        return [*head, *([None] * 11), wire.enc_time(builder.now), memo, None, NEXT_IDENT]
    queue = getattr(builder.queue, "_queue", builder.queue)
    graph, index = builder.graph, builder.index
    return [
        *head,
        [queue.pushed, queue.popped, wire.count_value(queue._counter), [[entry.sequence, enc_event(entry.event)] for entry in queue._heap]],
        [enc_node(node) for node in builder.nodes],
        [list(ids) for ids in builder._node_vertex_ids],
        [enc_obligation(item) for item in builder._proof.obligations],
        None if builder.refusal is None else enc_str(builder.refusal.value),
        None if graph is None else wire.enc_graph(graph),
        None if index is None else enc_index(index),
        [[vertex, wire.enc_trace(trace)] for vertex, trace in builder.traces.items()],
        [[a, b, c, q] for (a, b, c, q) in builder.line_id],
        [list(edges_of) for edges_of in builder.edges_by_line.values()],
        [
            sorted(builder.unindexed_reflex),
            sorted(builder.sliding_vertices),
            sorted(builder.fan_vertices),
            sorted([origin, vertex] for origin, vertex in builder.origin_vertex.items()),
            builder.origin_count,
        ],
        wire.enc_time(builder.now),
        memo,
        [[enc_str(name), value] for name, value in sorted(builder.counters.items())],
        NEXT_IDENT,
    ]


# --------------------------------------------------------------------------
# the canonical shape of a state
# --------------------------------------------------------------------------


def _dense(table: dict, key) -> int:
    return table.setdefault(key, len(table))


def _decode_names(items) -> list:
    return [(dec_str(name) if isinstance(name, list) else name, value) for name, value in items]


def _decode_event(event) -> tuple:
    kind, time, point, vertex, peer, edge, unproven = event
    return (dec_str(kind), tuple(time), tuple(point), vertex, peer, edge, unproven)


def canon_state(state: list, *, memo: bool = True, queue: bool = True) -> dict:
    """A state (the wire the native seam answers, or what `enc_builder_state` made of an oracle builder) in one comparable shape.

    Identities are renumbered by first appearance; the dictionaries of the oracle (`edge_start`, `edge_end`, the sets) are compared by content (the native answer sorts them);
    the queue is the heap ARRAY (an entry is its sequence number and its event). `memo=False` leaves the memory of places out (a run resumed from a state keeps the entries it
    was given; only the sizes are an answer of `BUILDER_INIT`)."""

    (universe, edges, vertices, edge_start, edge_end, queue_wire, nodes, node_ids, proof, refusal, graph, index, traces, line_id, edges_by_line, sets, now, memo_wire_, counters, _next) = state
    lines, slides = {}, {}
    out: dict = {"universe": tuple(universe)}
    out["edges"] = [(ident, tuple(line[:4]), tuple(span), _dense(lines, line[4])) for ident, line, span in edges]
    out["vertices"] = [
        (ident, prev_edge, next_edge, prev, next, tuple(birth), tuple(point), reflex, alive, None if sliding is None else (sliding[0], _dense(slides, sliding[1])))
        for ident, prev_edge, next_edge, prev, next, birth, point, reflex, alive, sliding in vertices
    ]
    out["edge_start"] = sorted(tuple(item) for item in edge_start)
    out["edge_end"] = sorted(tuple(item) for item in edge_end)
    if queue and queue_wire is not None:
        pushed, popped, counter, entries = queue_wire
        out["queue"] = (pushed, popped, counter, [(sequence, _decode_event(event)) for sequence, event in entries])
    if nodes is not None:
        out["nodes"] = [(dec_str(node[0]), tuple(node[1]), tuple(node[2]), [tuple(key) for key in node[3]], node[4], [dec_str(kind) for kind in node[5]], [[tuple(key) for key in incidence] for incidence in node[6]]) for node in nodes]
        out["node_ids"] = [tuple(ids) for ids in node_ids]
        out["proof"] = [(item[0], dec_str(item[1]), dec_str(item[2]), tuple(item[3]), [tuple(key) for key in item[4]], [tuple(key) for key in item[5]], tuple(item[6]), None if item[7] is None else dec_str(item[7])) for item in proof]
        out["refusal"] = None if refusal is None else dec_str(refusal)
        out["graph"] = graph
        out["index"] = index
        out["traces"] = sorted(((vertex, tuple(trace)) for vertex, trace in traces), key=lambda item: item[0])
        out["line_id"] = [tuple(key) for key in line_id]
        out["edges_by_line"] = [tuple(items) for items in edges_by_line]
        out["sets"] = (tuple(sets[0]), tuple(sets[1]), tuple(sets[2]), tuple(tuple(pair) for pair in sets[3]), sets[4])
        out["counters"] = _decode_names(counters)
    out["now"] = tuple(now)
    if memo:
        out["memo"] = tuple(memo_wire_) if len(memo_wire_) == 3 else (memo_wire_[0], *memo_sizes(memo_wire_))
    return out


def memo_sizes(memo_wire_: list) -> tuple:
    """`(places, times)` of the entries of a `[active, entries]` memory wire."""

    entries = memo_wire_[1]
    times = sum(1 for entry in entries if entry[0] in (1, 2))
    return (len(entries) - times, times)


# --------------------------------------------------------------------------
# the snapshot
# --------------------------------------------------------------------------


def _val(value) -> list:
    return wire.enc_repr(value)


def _opt_val(value):
    return None if value is None else _val(value)


def enc_incident(incident) -> list:
    return [
        enc_event(incident.event),
        list(incident.vertex_ids),
        list(incident.edge_occurrences),
        _keys(incident.participants),
        _keys(incident.target_participants),
        _val(incident.point_key),
        incident.met_vertex_id,
        incident.met_adjacent,
        incident.target_projection,
        incident.target_start_id,
        incident.target_end_id,
        _val(incident.emitter_key),
        _val(incident.peer_key),
        _opt_val(incident.target_occurrence),
        None if incident.target_ray is None else list(incident.target_ray),
    ]


def enc_vertex_snapshot(vertex) -> list:
    return [
        vertex.ident,
        vertex.prev,
        vertex.next,
        vertex.prev_edge,
        vertex.next_edge,
        vertex.alive,
        list(vertex.incoming_ray),
        list(vertex.outgoing_ray),
        _opt_val(vertex.point_key),
        _opt_val(vertex.prev_occurrence),
        _opt_val(vertex.next_occurrence),
    ]


def enc_snapshot(snapshot) -> list:
    return [
        [enc_incident(incident) for incident in snapshot.incidents],
        [enc_vertex_snapshot(vertex) for vertex in snapshot.vertices],
        enc_level(snapshot.unsupported),
        snapshot.stale_candidates,
        list(snapshot.duplicate_live_owner_edge_ids),
    ]


# --------------------------------------------------------------------------
# the primitives of the builder
# --------------------------------------------------------------------------

#: `{operation name: (code of BUILDER_PRIMITIVE, encoder of the arguments, decoder of the oracle's result)}`; see `native/cftuv-skeleton/src/seam_primitive.rs`.
PRIMITIVES: dict = {}


def _primitive(name: str, code: int, encode, result=lambda found: None):
    PRIMITIVES[name] = (code, encode, result)


def _ident(found):
    return found.ident


def _contact_args(contact) -> list:
    return [enc_level(contact.events), wire.enc_time(contact.time), wire.enc_point(contact.point), _keys(contact.participants), list(contact.dead_vertex_ids), [enc_str(kind.value) for kind in contact.kinds]]


def _meeting_args(meeting) -> list:
    return [enc_level(meeting.events), _keys(meeting.participants), list(meeting.meeting_vertex_ids)]


def _cut_args(cut) -> list:
    return [cut.edge_id, enc_level(cut.events)]


def _obligation_args(*, cause, disposition, vertex_ids=(), participant_edge_keys=(), target_edge_keys=(), level, event_kind=None) -> list:
    from cftuv_envelope.wavefront import proof

    kind = 1 if isinstance(cause, proof.ProofObligationBranch) else 0
    return [kind, enc_str(cause.value), enc_str(disposition.value), list(vertex_ids), _keys(participant_edge_keys), _keys(target_edge_keys), wire.enc_time(level), None if event_kind is None else enc_str(event_kind.value)]


_primitive("_twin", 0, lambda edge: [edge.ident], _ident)
_primitive(
    "_new_vertex",
    1,
    lambda *, prev_edge, next_edge, prev, next, birth, point: [prev_edge, next_edge, prev, next, wire.enc_time(birth), wire.enc_point(point)],
    _ident,
)
_primitive("_emit", 2, lambda kind, event, participants, converged: [enc_str(kind.value), enc_event(event), _keys(participants), list(converged)])
_primitive("_emit_split_node", 3, lambda vertex, edge, event: [vertex.ident, edge.ident, enc_event(event)])
_primitive("_refuse", 4, lambda reason, *, vertex_ids=(), participant_edge_keys=(), target_edge_keys=(): [enc_str(reason.value), list(vertex_ids), _keys(participant_edge_keys), _keys(target_edge_keys)])
_primitive("_enqueue_for", 5, lambda vertex: [vertex.ident])
_primitive("_enqueue_edge_event", 6, lambda vertex: [vertex.ident])
_primitive("_enqueue_splits_against", 7, lambda edge, *, excluded_vertex_ids=frozenset(): [edge.ident, sorted(excluded_vertex_ids)])
_primitive("_register", 8, lambda vertex: [vertex.ident])
_primitive("_record_obligation", 9, _obligation_args)
_primitive("_front_vertex_met_by", 10, lambda event: [enc_event(event)], lambda found: [None if found[0] is None else found[0].ident, found[1]])
_primitive("_edge_event_is_live", 11, lambda event: [enc_event(event)], lambda found: found)
_primitive("_split_is_live", 12, lambda event: [enc_event(event)], lambda found: found)
_primitive("_position", 13, lambda vertex, time: [vertex.ident, wire.enc_time(time)], lambda found: None if found is None else wire.enc_point(found))
_primitive("_record_unsupported", 14, lambda event: [enc_event(event)])
_primitive("_record_duplicate_live_owner", 15, lambda snapshot, level: [enc_snapshot(snapshot), enc_level(level)])
_primitive("_record_symbolic_unresolvable", 16, lambda snapshot, reason: [enc_snapshot(snapshot), enc_str(reason)])
_primitive("_record_edge_span_debts", 17, lambda events: [enc_level(events)])
_primitive("_emit_edge_contact", 18, lambda contact: [_contact_args(contact)])
_primitive("_emit_meeting", 19, lambda meeting: [_meeting_args(meeting)])
_primitive(
    "_emit_component_nodes",
    20,
    lambda plan: [[_contact_args(contact) for contact in plan.edge_contacts], [_meeting_args(meeting) for meeting in plan.vertex_meetings], [_cut_args(cut) for cut in plan.split_cuts]],
)


def canon_primitive(result, state) -> tuple:
    return (result, canon_state(state))


# --------------------------------------------------------------------------
# the result
# --------------------------------------------------------------------------


def skeleton_wire(skeleton) -> list:
    """An oracle `SkeletonV1` in the shape of the native `skeleton_value`."""

    return [
        enc_str(skeleton.outcome.value),
        [enc_node(node) for node in skeleton.nodes],
        skeleton.levels,
        [[enc_str(name), value] for name, value in skeleton.counters],
        enc_str(skeleton.proof_status.value),
        [enc_obligation(item) for item in skeleton.proof_obligations],
    ]


def canon_skeleton(wire_: list) -> tuple:
    outcome, nodes, levels, counters, status, obligations = wire_
    return (
        dec_str(outcome),
        [(dec_str(node[0]), tuple(node[1]), tuple(node[2]), [tuple(key) for key in node[3]], node[4], [dec_str(kind) for kind in node[5]], [[tuple(key) for key in incidence] for incidence in node[6]]) for node in nodes],
        levels,
        _decode_names(counters),
        dec_str(status),
        [(item[0], dec_str(item[1]), dec_str(item[2]), tuple(item[3]), [tuple(key) for key in item[4]], [tuple(key) for key in item[5]], tuple(item[6]), None if item[7] is None else dec_str(item[7])) for item in obligations],
    )


def dec_skeleton(answer_value: list) -> dict:
    """The answer of `BUILDER_RUN`: `{"stopped": bool, "levels", "level", "state"}` or `{"stopped": False, "skeleton": canonical skeleton}`."""

    if answer_value[0] == 0:
        _, levels, level, state = answer_value
        return {"stopped": True, "levels": levels, "level": [_decode_event(event) for event in level], "state": state}
    return {"stopped": False, "skeleton": canon_skeleton(answer_value[1])}


def decode_text(value) -> str:
    return dec_str(value)


def checked_unsupported(error) -> bool:
    return isinstance(error, SeamUnsupported)
