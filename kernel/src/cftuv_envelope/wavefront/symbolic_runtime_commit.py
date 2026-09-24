"""Single-commit materialization of a closed symbolic superlevel overlay."""

from __future__ import annotations

from dataclasses import dataclass

from .event_time import EventTimeOutcome, compare_times
from .events import CandidateEventV1, EventKind
from .poststate_span import (
    PoststateSpanDisposition,
    classify_poststate_span,
)
from .proof import ProofObligationBranch, ProofObligationDisposition
from .symbolic_component import point_from_key


@dataclass(frozen=True, slots=True)
class SymbolicRuntimeCommitPlanV1:
    """Полностью проверенная проекция final overlay на runtime-объекты."""

    closure: object
    active_leaves_by_edge: tuple[tuple[int, tuple], ...]
    alive_existing: tuple
    dead_existing: tuple
    births: tuple
    affected_edges: tuple[int, ...]
    later_nodes: tuple
    unresolved_fallbacks: tuple
    edge_fingerprint: tuple


@dataclass(frozen=True, slots=True)
class SymbolicNodePayloadV1:
    kind: EventKind
    time: object
    point: object
    participants: tuple
    dead_refs: tuple
    span_unproven: bool = False
    target_participants: tuple = ()
    vertex_meeting: bool = False


@dataclass(frozen=True, slots=True)
class SymbolicFallbackPayloadV1:
    vertex_ids: tuple[int, ...]
    participant_edge_keys: tuple
    target_edge_keys: tuple


@dataclass(frozen=True, slots=True)
class SymbolicPastEdgeEventV1:
    """Активный poststate adjacency с отдельным exact событием в прошлом."""

    vertex_ref: object
    peer_ref: object
    event_time: object
    event_point: object
    now: object
    vertex_birth: object
    peer_birth: object
    participant_edge_keys: tuple
    participant_families: tuple


@dataclass(frozen=True, slots=True)
class SymbolicPoststateSpanV1:
    """Affine birth classification for one changed runtime adjacency."""

    vertex_ref: object
    peer_ref: object
    classification: object
    participant_edge_keys: tuple
    participant_families: tuple


def _later_node_payloads(closure):
    payloads = []
    overlay = closure.overlay
    for generation in closure.junction.generations:
        for contact in generation.junction_contacts:
            if contact.kind == "EDGE":
                payloads.append(SymbolicNodePayloadV1(
                    EventKind.EDGE,
                    overlay.time,
                    point_from_key(contact.key.point_key),
                    contact.edge.participant_keys,
                    contact.dead_refs,
                    contact.edge.span_unproven,
                    contact.edge.shared_leaf.family.participant_keys,
                ))
            else:
                payloads.append(SymbolicNodePayloadV1(
                    EventKind.SPLIT,
                    overlay.time,
                    point_from_key(contact.key.point_key),
                    contact.key.participants,
                    contact.dead_refs,
                    vertex_meeting=True,
                ))
        for contact in generation.interior_contacts:
            payloads.append(SymbolicNodePayloadV1(
                EventKind.SPLIT,
                contact.time,
                contact.point,
                contact.key.participants,
                (contact.key.emitter,),
                False,
                contact.key.family.participant_keys,
            ))
    return tuple(payloads)


def _unresolved_fallbacks(snapshot, closure):
    """Keep only fallbacks not explicitly consumed by a causal endpoint."""

    endpoints = tuple(
        contact.key
        for generation in closure.junction.generations
        for contact in generation.junction_contacts
        if contact.kind == "ENDPOINT"
    )
    incident_by_event = {incident.event: incident for incident in snapshot.incidents}
    unresolved = []
    for component in closure.materialization.plans:
        for event in component.proof_fallbacks:
            incident = incident_by_event.get(event)
            resolved = False
            if incident is not None:
                resolved = any(
                    key.time_key
                    == (
                        incident.event.time.canonical().dividend,
                        incident.event.time.canonical().divisor.terms,
                    )
                    and key.point_key == incident.point_key
                    and key.emitter.key == incident.emitter_key
                    and key.family.occurrence == incident.target_occurrence
                    and key.participants == incident.participants
                    for key in endpoints
                )
            if not resolved:
                target = (
                    ()
                    if incident is None or incident.target_occurrence is None
                    else (incident.target_occurrence[0],)
                )
                unresolved.append(SymbolicFallbackPayloadV1(
                    (
                        (event.vertex,)
                        if incident is None
                        else tuple(sorted({
                            event.vertex,
                            *(
                                ident for ident in (
                                    incident.target_start_id,
                                    incident.target_end_id,
                                )
                                if ident is not None
                            ),
                        }))
                    ),
                    () if incident is None else incident.participants,
                    target,
                ))
    return tuple(unresolved)


def _ordered_lineage(leaves):
    """Order one physical root family by exact segment occurrence.

    An EDGE contact may remove a middle interval and leave two disconnected
    siblings of the same physical edge alive.  Requiring their endpoints to
    form one contiguous chain rejects that valid poststate.  Runtime twins
    need only a stable allocation order; the exact occurrence is the
    geometric authority and never depends on heap or runtime identity.
    """

    leaves = tuple(leaves)
    if not leaves or len({leaf.family for leaf in leaves}) != 1:
        return None
    occurrences = [leaf.occurrence for leaf in leaves]
    if len(set(occurrences)) != len(occurrences):
        return None
    return tuple(sorted(leaves, key=lambda leaf: repr(leaf.occurrence)))


def _edge_fingerprint(builder, edge_ids):
    return tuple(
        (
            edge_id,
            builder.edges[edge_id].key,
            builder.edges[edge_id].line,
            builder.edges[edge_id].span,
        )
        for edge_id in sorted(edge_ids)
    )


def changed_poststate_edge_events(builder, snapshot, overlay):
    """Наблюдать exact EDGE transitions только у changed adjacencies."""

    from .exact_candidate_view import edge_event_time, position
    from .symbolic_overlay import exact_overlay_view

    alive = {
        ref: vertex
        for ref, vertex in overlay.vertices.items()
        if vertex.alive
    }
    view = exact_overlay_view(builder, overlay)
    witnesses = []

    for ref in sorted(alive, key=repr):
        vertex = alive[ref]
        peer = alive.get(vertex.next)
        if peer is None:
            continue
        leaves = (vertex.prev_leaf, vertex.next_leaf, peer.next_leaf)
        changed_adjacency = (
            ref.kind != "EXISTING"
            or peer.ref.kind != "EXISTING"
            or snapshot.vertices[vertex.runtime_id].next != peer.runtime_id
        )
        if not changed_adjacency:
            continue
        event_time, outcome = edge_event_time(
            view, ref, peer.ref, overlay.time
        )
        if outcome is not EventTimeOutcome.EXACT or event_time is None:
            continue
        witnesses.append(SymbolicPastEdgeEventV1(
            ref,
            peer.ref,
            event_time,
            position(view, ref, event_time),
            overlay.time,
            vertex.birth,
            peer.birth,
            tuple(
                builder.edges[overlay.spans[leaf].physical_edge_id].key
                for leaf in leaves
            ),
            tuple(leaf.family for leaf in leaves),
        ))
    return tuple(witnesses)


def poststate_span_classifications(builder, snapshot, overlay):
    """Classify every changed adjacency without consulting event history."""

    from .symbolic_overlay import exact_overlay_view

    alive = {
        ref: vertex
        for ref, vertex in overlay.vertices.items()
        if vertex.alive
    }
    view = exact_overlay_view(builder, overlay)
    witnesses = []
    for ref in sorted(alive, key=repr):
        vertex = alive[ref]
        peer = alive.get(vertex.next)
        if peer is None:
            continue
        # A two-vertex terminal cycle has two shared leaves and no unique
        # oriented newborn span.  Its closure is owned by the terminal
        # certificate, not by the one-span P0-2c law.
        if peer.next == ref:
            continue
        changed_adjacency = (
            ref.kind != "EXISTING"
            or peer.ref.kind != "EXISTING"
            or snapshot.vertices[vertex.runtime_id].next != peer.runtime_id
        )
        if not changed_adjacency:
            continue
        leaves = (vertex.prev_leaf, vertex.next_leaf, peer.next_leaf)
        witnesses.append(SymbolicPoststateSpanV1(
            ref,
            peer.ref,
            classify_poststate_span(view, ref, peer.ref, overlay.time),
            tuple(
                builder.edges[overlay.spans[leaf].physical_edge_id].key
                for leaf in leaves
            ),
            tuple(leaf.family for leaf in leaves),
        ))
    return tuple(witnesses)


def changed_poststate_past_edge_events(builder, snapshot, overlay):
    """Q-08 conjunction без решения о границе event-time domain."""

    budget = getattr(builder, "work_budget", None)
    return tuple(
        witness
        for witness in changed_poststate_edge_events(
            builder, snapshot, overlay
        )
        if compare_times(witness.event_time, witness.now, budget) < 0
        and compare_times(
            witness.event_time, witness.vertex_birth, budget
        ) < 0
        and compare_times(witness.event_time, witness.peer_birth, budget) < 0
    )


def _poststate_span_refusal(builder, snapshot, overlay):
    """Q-08 last guard after the affine P0-2c law has classified spans."""

    budget = getattr(builder, "work_budget", None)
    spans = poststate_span_classifications(builder, snapshot, overlay)
    dispositions = {
        witness.classification.disposition for witness in spans
    }
    if PoststateSpanDisposition.INVERTED in dispositions:
        return "SYMBOLIC_POSTSTATE_SPAN_INVERTED"
    if (
        PoststateSpanDisposition.AFFINE_CLASSIFICATION_UNPROVEN
        in dispositions
    ):
        return "SYMBOLIC_POSTSTATE_SPAN_AFFINE_CLASSIFICATION_UNPROVEN"
    exact_events = {
        (witness.vertex_ref, witness.peer_ref): witness
        for witness in changed_poststate_edge_events(
            builder, snapshot, overlay
        )
    }
    for span in spans:
        if span.classification.disposition is not (
            PoststateSpanDisposition.CLOSING_WITH_FUTURE_EVENT
        ):
            continue
        event = exact_events.get((span.vertex_ref, span.peer_ref))
        if (
            event is None
            or compare_times(event.event_time, event.now, budget) <= 0
            or compare_times(
                event.event_time, event.vertex_birth, budget
            ) <= 0
            or compare_times(event.event_time, event.peer_birth, budget) <= 0
        ):
            return "SYMBOLIC_POSTSTATE_CLOSING_EVENT_UNPROVEN"
    return None


def _validated_existing_vertices(builder, snapshot, overlay, materialization):
    existing = {
        vertex.ref: vertex
        for vertex in overlay.vertices.values()
        if vertex.ref.kind == "EXISTING"
    }
    runtime_ids = [vertex.runtime_id for vertex in existing.values()]
    frozen_live = {vertex.ident for vertex in snapshot.vertices if vertex.alive}
    initially_dead = {
        ident
        for component in materialization.plans
        for ident in component.dead_vertex_ids
    }
    if (
        any(ident is None for ident in runtime_ids)
        or len(set(runtime_ids)) != len(runtime_ids)
        or set(runtime_ids).intersection(initially_dead)
        or set(runtime_ids).union(initially_dead) != frozen_live
        or any(
            ident < 0
            or ident >= len(builder.vertices)
            or not builder.vertices[ident].alive
            for ident in runtime_ids
        )
    ):
        return None, "SYMBOLIC_RUNTIME_EXISTING_REF_UNRESOLVABLE"
    if any(
        builder.vertices[item.ident].prev != item.prev
        or builder.vertices[item.ident].next != item.next
        or builder.vertices[item.ident].prev_edge != item.prev_edge
        or builder.vertices[item.ident].next_edge != item.next_edge
        or builder.vertices[item.ident].alive != item.alive
        for item in snapshot.vertices
    ):
        return None, "SYMBOLIC_RUNTIME_FROZEN_PRESTATE_CHANGED"
    return (existing, initially_dead), None


def _validated_active_leaf_groups(builder, overlay):
    alive = {
        ref: vertex
        for ref, vertex in overlay.vertices.items()
        if vertex.alive
    }
    if any(
        vertex.prev not in alive
        or vertex.next not in alive
        or alive[vertex.prev].next != ref
        or alive[vertex.next].prev != ref
        for ref, vertex in alive.items()
    ):
        return None, "SYMBOLIC_RUNTIME_RECIPROCITY_UNRESOLVABLE"
    starts, ends = {}, {}
    for ref, vertex in alive.items():
        starts.setdefault(vertex.next_leaf, []).append(ref)
        ends.setdefault(vertex.prev_leaf, []).append(ref)
    active_leaves = set(starts) | set(ends)
    if (
        set(starts) != set(ends)
        or any(len(refs) != 1 for refs in (*starts.values(), *ends.values()))
    ):
        return None, "SYMBOLIC_RUNTIME_SPAN_OWNER_AMBIGUOUS"
    grouped = {}
    for leaf in active_leaves:
        binding = overlay.spans.get(leaf)
        if (
            binding is None
            or binding.start != starts[leaf][0]
            or binding.end != ends[leaf][0]
            or binding.physical_edge_id < 0
            or binding.physical_edge_id >= len(builder.edges)
        ):
            return None, "SYMBOLIC_RUNTIME_SPAN_BINDING_UNRESOLVABLE"
        grouped.setdefault(binding.physical_edge_id, []).append(leaf)
    return (active_leaves, grouped), None


def _validated_birth_context(builder, overlay, active_leaves):
    births = tuple(sorted(
        (
            vertex for vertex in overlay.vertices.values()
            if vertex.ref.kind != "EXISTING"
        ),
        key=lambda item: repr(item.ref),
    ))
    if any(vertex.ref.kind == "VIRTUAL_BOUNDARY" for vertex in births):
        return None, "SYMBOLIC_RUNTIME_VIRTUAL_VERTEX_UNRESOLVABLE"
    if any(
        leaf not in overlay.spans
        for vertex in births
        for leaf in (vertex.prev_leaf, vertex.next_leaf)
    ):
        return None, "SYMBOLIC_RUNTIME_BIRTH_SPAN_UNRESOLVABLE"
    referenced_leaves = active_leaves | {
        leaf
        for vertex in births
        for leaf in (vertex.prev_leaf, vertex.next_leaf)
    }
    for leaf in referenced_leaves:
        binding = overlay.spans[leaf]
        if not 0 <= binding.physical_edge_id < len(builder.edges):
            return None, "SYMBOLIC_RUNTIME_PHYSICAL_EDGE_UNRESOLVABLE"
        source_key = builder.edges[binding.physical_edge_id].key
        if (
            leaf.occurrence[0] != source_key
            or leaf.family.occurrence[0] != source_key
            or source_key not in leaf.family.participant_keys
        ):
            return None, "SYMBOLIC_RUNTIME_EDGE_AUTHORITY_MISMATCH"
    return (births, referenced_leaves), None


def plan_symbolic_runtime_commit(builder, snapshot, closure):
    """Validate every reference before the first runtime allocation/write."""

    overlay = closure.overlay
    materialization = closure.materialization
    if overlay is None or materialization is None:
        return None, "SYMBOLIC_RUNTIME_FINAL_OVERLAY_UNAVAILABLE"
    if materialization.unresolved_reason is not None:
        return None, materialization.unresolved_reason

    existing_result, reason = _validated_existing_vertices(
        builder, snapshot, overlay, materialization
    )
    if reason is not None:
        return None, reason
    existing, initially_dead = existing_result
    active_result, reason = _validated_active_leaf_groups(builder, overlay)
    if reason is not None:
        return None, reason
    active_leaves, grouped = active_result
    birth_result, reason = _validated_birth_context(
        builder, overlay, active_leaves
    )
    if reason is not None:
        return None, reason
    births, referenced_leaves = birth_result
    alive_existing = tuple(sorted(
        (vertex for vertex in existing.values() if vertex.alive),
        key=lambda item: item.runtime_id,
    ))
    dead_existing = tuple(sorted(
        initially_dead.union(
            vertex.runtime_id
            for vertex in existing.values()
            if not vertex.alive
        )
    ))
    affected = {
        overlay.spans[leaf].physical_edge_id
        for leaf in overlay.changed
        if leaf in active_leaves
    }
    known_refs = set(existing) | {vertex.ref for vertex in births}
    later_nodes = _later_node_payloads(closure)
    if any(
        ref not in known_refs
        for payload in later_nodes
        for ref in payload.dead_refs
    ):
        return None, "SYMBOLIC_RUNTIME_NODE_REF_UNRESOLVABLE"
    lineages = []
    for edge_id, leaves in grouped.items():
        ordered_lineage = _ordered_lineage(leaves)
        if ordered_lineage is None:
            return None, "SYMBOLIC_RUNTIME_EDGE_LINEAGE_UNRESOLVABLE"
        lineages.append((edge_id, ordered_lineage))
    span_refusal = _poststate_span_refusal(builder, snapshot, overlay)
    if span_refusal is not None:
        return None, span_refusal
    return SymbolicRuntimeCommitPlanV1(
        closure,
        tuple(sorted(lineages)),
        alive_existing,
        dead_existing,
        births,
        tuple(sorted(affected)),
        later_nodes,
        _unresolved_fallbacks(snapshot, closure),
        _edge_fingerprint(
            builder,
            {
                overlay.spans[leaf].physical_edge_id
                for leaf in referenced_leaves
            },
        ),
    ), None


def _runtime_ids(refs, runtime_by_ref):
    return tuple(sorted({runtime_by_ref[ref].ident for ref in refs}))


def _emit_later_nodes(builder, plan, runtime_by_ref):
    """Emit causal symbolic contacts; final accumulator owns point unions."""

    for payload in plan.later_nodes:
        ids = _runtime_ids(payload.dead_refs, runtime_by_ref)
        event = CandidateEventV1(
            payload.kind,
            payload.time,
            payload.point,
            ids[0] if ids else -1,
            ids[1] if len(ids) > 1 else -1,
            -1,
        )
        builder._emit(payload.kind, event, payload.participants, ids)
        if payload.kind is EventKind.EDGE:
            builder.counters["edge_events"] += 1
            if payload.span_unproven:
                builder.counters[
                    "edge_collapse_span_unproven_but_accepted"
                ] += 1
                builder._record_obligation(
                    cause=ProofObligationBranch.EDGE_COLLAPSE_SPAN_UNPROVEN,
                    disposition=(
                        ProofObligationDisposition
                        .EVENT_ACCEPTED_WITH_UNPROVEN_SPAN
                    ),
                    vertex_ids=ids,
                    participant_edge_keys=payload.participants,
                    target_edge_keys=payload.target_participants,
                    level=payload.time,
                )
        else:
            builder.counters["split_events"] += 1
            if payload.vertex_meeting:
                builder.counters["vertex_meeting_events"] += 1
        if len(payload.participants) > 3:
            builder.counters["multi_participant_nodes"] += 1


class _FutureQueueV1:
    """Commit-local queue writer: final closure proved no event at ``now``."""

    def __init__(self, queue, now):
        self._queue = queue
        self._now = now

    def push(self, event):
        # Бюджет берётся у обёрнутой очереди: она принадлежит той же
        # транзакции, и второго счёта у коммита нет.
        budget = getattr(self._queue, "work_budget", None)
        if compare_times(event.time, self._now, budget) > 0:
            self._queue.push(event)


def materialize_symbolic_runtime_commit(builder, snapshot, plan):
    """Commit the already validated final topology without a fallback path."""

    from . import superlevel as base

    closure = plan.closure
    overlay = closure.overlay
    plans = closure.materialization.plans
    if _edge_fingerprint(
        builder, (row[0] for row in plan.edge_fingerprint)
    ) != plan.edge_fingerprint:
        return "SYMBOLIC_RUNTIME_EDGE_FINGERPRINT_CHANGED"
    edge_by_leaf = {}
    for edge_id, leaves in plan.active_leaves_by_edge:
        source = builder.edges[edge_id]
        runtime_edges = [source]
        runtime_edges.extend(builder._twin(source) for _ in leaves[1:])
        edge_by_leaf.update(
            (leaf, edge.ident) for leaf, edge in zip(leaves, runtime_edges)
        )
    for symbolic in plan.births:
        for leaf in (symbolic.prev_leaf, symbolic.next_leaf):
            edge_by_leaf.setdefault(
                leaf, overlay.spans[leaf].physical_edge_id
            )

    runtime_by_ref = {
        vertex.ref: builder.vertices[vertex.runtime_id]
        for vertex in overlay.vertices.values()
        if vertex.ref.kind == "EXISTING"
    }
    for symbolic in plan.births:
        placeholder = len(builder.vertices)
        runtime_by_ref[symbolic.ref] = builder._new_vertex(
            prev_edge=edge_by_leaf[symbolic.prev_leaf],
            next_edge=edge_by_leaf[symbolic.next_leaf],
            prev=placeholder,
            next=placeholder,
            birth=symbolic.birth,
            point=symbolic.point,
        )
        runtime_by_ref[symbolic.ref].alive = symbolic.alive

    from .candidate_refusal import CandidateRefusal

    for fallback in plan.unresolved_fallbacks:
        builder._refuse(
            CandidateRefusal.NO_RULE_MEETING_NOT_RECONNECTABLE,
            vertex_ids=fallback.vertex_ids,
            participant_edge_keys=fallback.participant_edge_keys,
            target_edge_keys=fallback.target_edge_keys,
        )
    for component in plans:
        base._emit_component_nodes(builder, component)
        builder.counters["coincident_split_targets"] += (
            component.coincident_split_targets
        )
        builder.counters["discarded_stale_candidates"] += (
            component.suppressed_candidates
        )
        builder.counters["peaks"] += component.closed_chain_count
    _emit_later_nodes(builder, plan, runtime_by_ref)

    for ident in plan.dead_existing:
        builder.vertices[ident].alive = False
    alive_births = tuple(vertex for vertex in plan.births if vertex.alive)
    for symbolic in (*plan.alive_existing, *alive_births):
        runtime = runtime_by_ref[symbolic.ref]
        runtime.prev_edge = edge_by_leaf[symbolic.prev_leaf]
        runtime.next_edge = edge_by_leaf[symbolic.next_leaf]
    for symbolic in (*plan.alive_existing, *alive_births):
        runtime = runtime_by_ref[symbolic.ref]
        runtime.prev = runtime_by_ref[symbolic.prev].ident
        runtime.next = runtime_by_ref[symbolic.next].ident
        builder._register(runtime)

    born_ids = frozenset(
        runtime_by_ref[vertex.ref].ident for vertex in plan.births
    )
    original_queue = builder.queue
    builder.queue = _FutureQueueV1(original_queue, overlay.time)
    for symbolic in alive_births:
        builder._enqueue_for(runtime_by_ref[symbolic.ref])
    for symbolic in plan.alive_existing:
        frozen = snapshot.vertices[symbolic.runtime_id]
        runtime = runtime_by_ref[symbolic.ref]
        if (
            runtime.prev != frozen.prev
            or runtime.next != frozen.next
            or runtime.prev_edge != frozen.prev_edge
            or runtime.next_edge != frozen.next_edge
        ):
            builder._enqueue_edge_event(runtime)
    active_edges = {
        edge_by_leaf[leaf]
        for physical, leaves in plan.active_leaves_by_edge
        if physical in plan.affected_edges
        for leaf in leaves
    }
    for edge_id in sorted(active_edges):
        builder._enqueue_splits_against(
            builder.edges[edge_id], excluded_vertex_ids=born_ids
        )
    builder.queue = original_queue

    later_births = sum(
        len(delta.rewires)
        + (not delta.rewires and delta.birth_ref is not None)
        for generation in closure.junction.generations
        for delta in generation.deltas
    )
    builder.counters["superlevel_contact_junction_resolutions"] += (
        sum(len(component.births) for component in plans) + later_births
    )
    return None
