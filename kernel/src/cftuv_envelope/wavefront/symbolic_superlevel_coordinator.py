"""Pure outer fixed point: root splits, then unified junction closure."""

from __future__ import annotations

from dataclasses import dataclass, replace

from .candidate_law import evaluate_split_candidate
from .event_time import compare_times
from .events import EventKind
from . import superlevel as base
from .superlevel_closure import SpanFamilyRefV1, plan_split_materialization
from .superlevel_fixed_point import (
    SymbolicSplitContactKeyV1,
    SymbolicSplitContactV1,
    _compile_contacts,
    merge_symbolic_split_contacts,
)
from .symbolic_component import overlay_signature
from .symbolic_f0_overlay import build_f0_overlay
from .symbolic_junction_fixed_point import (
    SymbolicJunctionFixedPointV1,
)
from .symbolic_mixed_generation import plan_mixed_generations
from .symbolic_overlay import (
    JunctionRefV1,
    SymbolicOverlayV1,
    build_symbolic_overlay,
    exact_overlay_view,
    is_symbolic_split_emitter,
)
from .symbolic_sparse_ports import with_line_ports


@dataclass(frozen=True, slots=True)
class SymbolicSuperlevelClosureV1:
    materialization: object | None
    overlay: SymbolicOverlayV1 | None
    split_contacts: tuple[SymbolicSplitContactV1, ...]
    junction: SymbolicJunctionFixedPointV1 | None
    outer_iterations: int
    signatures: tuple[tuple, ...]
    canonical_batch_count: int
    unresolved_reason: str | None = None


def _participants(*leaves):
    return tuple(sorted({participant for leaf in leaves
                         for participant in leaf.family.participant_keys}))


def initial_interior_contacts(snapshot):
    contacts = []
    for incident in snapshot.incidents:
        occurrence = incident.target_occurrence
        if (incident.event.kind is not EventKind.SPLIT
                or occurrence is None
                or incident.point_key in occurrence[1:]):
            continue
        family = SpanFamilyRefV1(occurrence, (occurrence[0],))
        key = SymbolicSplitContactKeyV1(
            base._time_key(incident.event.time), incident.point_key,
            JunctionRefV1("EXISTING", incident.emitter_key), family,
            incident.participants,
        )
        contacts.append(SymbolicSplitContactV1(
            key, incident.event.time, incident.event.point,
            incident.target_projection,
        ))
    return merge_symbolic_split_contacts(contacts)


def _rebuild_splits(builder, snapshot, contacts, time, *, f0=None):
    """Rebuild one mixed frozen-prestate plan from F0 contact authority."""

    port_snapshot = with_line_ports(builder, snapshot, time)
    if port_snapshot is None:
        return None, None, "SYMBOLIC_SPLIT_LINE_PORT_HYDRATION_UNRESOLVABLE"
    compiled, reason = _compile_contacts(builder, port_snapshot, contacts)
    if reason is not None:
        return None, None, reason
    retained = tuple(
        incident
        for incident in snapshot.incidents
        if not (
            incident.event.kind is EventKind.SPLIT
            and incident.target_occurrence is not None
            and incident.point_key not in incident.target_occurrence[1:]
        )
    )
    incidents = tuple(sorted(
        (*retained, *compiled), key=base._incident_sort_key
    ))
    working = replace(port_snapshot, incidents=incidents)
    materialization = plan_split_materialization(
        working, getattr(builder, "work_budget", None)
    )
    if materialization.unresolved_reason is not None:
        return materialization, None, materialization.unresolved_reason
    if any(
        plan.resolution is base.SuperlevelResolution.UNRESOLVABLE
        for plan in materialization.plans
    ):
        return (
            materialization,
            None,
            "SYMBOLIC_UNIFIED_CONTACT_COMPONENT_UNRESOLVABLE",
        )
    overlay = build_symbolic_overlay(
        builder, working, materialization, include_line_ports=True
    )
    reason = None if overlay is not None else "SYMBOLIC_SPLIT_OVERLAY_UNRESOLVABLE"
    return materialization, overlay, reason


def _initial_interior_closure(builder, snapshot, f0, contacts, *, budget):
    ordered, reason = merge_symbolic_split_contacts(contacts)
    if reason is not None:
        return (), 0, None, None, reason
    for iteration in range(budget + 1):
        materialization, mixed_overlay, reason = _rebuild_splits(
            builder, snapshot, ordered, f0.time, f0=f0
        )
        if reason is not None:
            return (), iteration, materialization, None, reason
        latent, reason = discover_interior_split_contacts(
            builder, mixed_overlay
        )
        if reason is not None:
            return (), iteration, materialization, mixed_overlay, reason
        merged, reason = merge_symbolic_split_contacts(ordered, latent)
        if reason is not None:
            return (), iteration, materialization, mixed_overlay, reason
        if len(merged) == len(ordered):
            return (
                ordered,
                iteration,
                materialization,
                mixed_overlay,
                None,
            )
        if iteration == budget:
            return (
                (),
                iteration,
                materialization,
                mixed_overlay,
                "SYMBOLIC_SUPERLEVEL_OUTER_BUDGET_EXHAUSTED",
            )
        ordered = merged
    raise AssertionError("unreachable initial interior closure")


def discover_interior_split_contacts(builder, overlay):
    contacts = []
    view = exact_overlay_view(builder, overlay)
    emitters = tuple(
        vertex for vertex in overlay.vertices.values()
        if vertex.alive
        and is_symbolic_split_emitter(builder, overlay, vertex)
    )
    for leaf in sorted(overlay.changed, key=repr):
        for emitter in sorted(emitters, key=lambda item: repr(item.ref)):
            if leaf in (emitter.prev_leaf, emitter.next_leaf):
                continue
            decision = evaluate_split_candidate(
                view, emitter.ref, leaf, now=overlay.time
            )
            candidate = decision.candidate
            if (candidate is None
                    or compare_times(
                        candidate.time, overlay.time, view.budget
                    ) != 0
                    or candidate.at_start or candidate.at_end):
                continue
            binding = overlay.spans[leaf]
            projection = (
                candidate.point.x.scaled(
                    builder.edges[binding.physical_edge_id].line.b
                ) - candidate.point.y.scaled(
                    builder.edges[binding.physical_edge_id].line.a
                )
            )
            key = SymbolicSplitContactKeyV1(
                base._time_key(candidate.time),
                (candidate.point.x.terms, candidate.point.y.terms),
                emitter.ref, leaf.family,
                _participants(emitter.prev_leaf, emitter.next_leaf, leaf),
            )
            contacts.append(SymbolicSplitContactV1(
                key, candidate.time, candidate.point, projection, leaf
            ))
    return merge_symbolic_split_contacts(contacts)


def _refusal(materialization, contacts, junction, iterations, signatures, reason):
    return SymbolicSuperlevelClosureV1(
        materialization, None, tuple(contacts), junction, iterations,
        tuple(signatures), 0, reason,
    )


def plan_symbolic_superlevel_closure(
    builder, snapshot, *, outer_budget, junction_budget
):
    """Rebuild from F0 whenever the stable interior contact set grows."""

    if not snapshot.incidents:
        return _refusal(
            None, (), None, 0, (), "SYMBOLIC_SUPERLEVEL_EMPTY_PACKET"
        )
    time = snapshot.incidents[0].event.time
    f0 = build_f0_overlay(builder, snapshot, time)
    if f0 is None:
        return _refusal(
            None, (), None, 0, (), "SYMBOLIC_F0_OVERLAY_UNRESOLVABLE"
        )
    initial, reason = initial_interior_contacts(snapshot)
    if reason is not None:
        return _refusal(None, (), None, 0, (), reason)
    (
        contacts,
        discovery_batches,
        materialization,
        split_overlay,
        reason,
    ) = _initial_interior_closure(
        builder,
        snapshot,
        f0,
        initial,
        budget=outer_budget,
    )
    if reason is not None:
        return _refusal(
            materialization,
            contacts,
            None,
            discovery_batches,
            (),
            reason,
        )
    if materialization is None or split_overlay is None:
        return _refusal(
            materialization,
            contacts,
            None,
            0,
            (),
            reason or "SYMBOLIC_SPLIT_OVERLAY_UNRESOLVABLE",
        )
    budget = outer_budget + junction_budget
    junction, later = plan_mixed_generations(
        builder,
        split_overlay,
        discover_interior_split_contacts,
        budget=budget,
    )
    if junction.unresolved_reason is not None or junction.overlay is None:
        return _refusal(
            materialization,
            contacts,
            junction,
            len(later),
            junction.signatures,
            junction.unresolved_reason
            or "SYMBOLIC_JUNCTION_OVERLAY_UNRESOLVABLE",
        )
    replay, replay_later = plan_mixed_generations(
        builder,
        split_overlay,
        discover_interior_split_contacts,
        budget=budget,
    )
    if (
        replay.unresolved_reason is not None
        or replay.overlay is None
        or replay_later != later
        or overlay_signature(replay.overlay)
        != overlay_signature(junction.overlay)
    ):
        return _refusal(
            materialization,
            (*contacts, *later),
            replay,
            len(later),
            junction.signatures,
            "SYMBOLIC_SUPERLEVEL_REPEATED_CONTACT_SET_CHANGED_SIGNATURE",
        )
    all_contacts = {
        item.key: item for item in (*contacts, *later)
    }
    ordered = tuple(sorted(
        all_contacts.values(), key=lambda item: repr(item.key)
    ))
    signatures = (
        overlay_signature(split_overlay),
        *junction.signatures,
        overlay_signature(junction.overlay),
    )
    return SymbolicSuperlevelClosureV1(
        materialization,
        junction.overlay,
        ordered,
        junction,
        len(later),
        signatures,
        1 + discovery_batches + len(junction.generations),
    )
