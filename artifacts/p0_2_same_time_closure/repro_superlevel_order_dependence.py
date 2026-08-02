"""Frozen P0-2 D repro: split-first versus edge-first packet application.

Run from the repository root with the same Python 3.10 environment as tests:

    PYTHONPATH=kernel/src;kernel/tests python \
        artifacts/p0_2_same_time_closure/repro_superlevel_order_dependence.py

The edge-first function is a test-only counterfactual schedule. It changes no
candidate generation, filtering, or production code. A difference is the
mandatory `SUPERLEVEL_ORDER_DEPENDENCE_FOUND` stop, not an invitation to fit
either result.
"""

from __future__ import annotations

from fractions import Fraction
import hashlib
import json
import sys

from wavefront_cases import named_corpus, partial_source_corpus
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.events import EventKind, cluster_by_point
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.proof import (
    ProofObligationBranch,
    ProofObligationDisposition,
)
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.superlevel import has_same_time_residual
from test_wavefront_same_time_closure import (
    _coverage_record,
    _partition_record,
)


CASES = (
    "cross",
    "ell_12_source_edges_0_1",
    "ell_12_source_edges_1_2",
    "ell_12_source_edges_3_4",
    "ell_12_source_edges_4_5",
    "staircase_source_edges_0_1",
    "staircase_source_edges_1_2",
    "staircase_source_edges_2_3",
    "staircase_source_edges_3_4",
    "staircase_source_edges_5_6",
    "staircase_source_edges_6_7",
    "ell_12_source_without_the_reflex_pair",
)


def _sha256(payload) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _fraction(value: Fraction) -> list[int]:
    return [value.numerator, value.denominator]


def _number(value):
    return _fraction(value) if isinstance(value, Fraction) else value


def _sqrt_sum(value) -> list:
    return [
        [radicand, _fraction(coefficient)]
        for radicand, coefficient in value.terms
    ]


def _time(value) -> list:
    canonical = value.canonical()
    return [_fraction(canonical.dividend), _sqrt_sum(canonical.divisor)]


def _point(value) -> list:
    return [_sqrt_sum(value.x), _sqrt_sum(value.y)]


def _participant(key) -> list:
    return [_number(item) for item in key]


def _vertex_token(builder, vertex) -> dict:
    position = builder._position(vertex, builder.now)
    return {
        "prev_edge": _participant(builder.edges[vertex.prev_edge].key),
        "next_edge": _participant(builder.edges[vertex.next_edge].key),
        "birth": _time(vertex.birth),
        "position": None if position is None else _point(position),
        "reflex": vertex.reflex,
        "sliding": None if vertex.sliding is None else _sqrt_sum(vertex.sliding),
    }


def _canonical_cycle(tokens: list[dict]) -> list[dict]:
    rotations = [tokens[index:] + tokens[:index] for index in range(len(tokens))]
    return min(
        rotations,
        key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":")),
    )


def _topology_record(builder) -> dict:
    live = {vertex.ident: vertex for vertex in builder.vertices if vertex.alive}
    unseen = set(live)
    cycles = []
    while unseen:
        start = min(unseen)
        cursor = start
        order = []
        local = set()
        while cursor in live and cursor not in local:
            local.add(cursor)
            unseen.discard(cursor)
            order.append(_vertex_token(builder, live[cursor]))
            cursor = live[cursor].next
        cycles.append(
            {
                "closed": cursor == start,
                "vertices": _canonical_cycle(order),
            }
        )
    cycles.sort(
        key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":"))
    )
    return {
        "time": _time(builder.now),
        "cycles": cycles,
    }


def _proof_projection(obligations) -> list:
    records = [
        {
            "cause": obligation.cause.value,
            "disposition": obligation.disposition.value,
            "level": _time(obligation.level),
            "participants": [
                _participant(key) for key in obligation.participant_edge_keys
            ],
            "targets": [_participant(key) for key in obligation.target_edge_keys],
            "event_kind": None
            if obligation.event_kind is None
            else obligation.event_kind.value,
        }
        for obligation in obligations
    ]
    records.sort(
        key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":"))
    )
    return records


def _superlevel_state(builder) -> dict:
    nodes = [node_record(node) for node in builder.nodes]
    nodes.sort(
        key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":"))
    )
    return {
        "time": _time(builder.now),
        "live_topology_sha256": _sha256(_topology_record(builder)),
        "emitted_nodes_sha256": _sha256(nodes),
        "proof_obligations_sha256": _sha256(
            _proof_projection(builder.proof_obligations)
        ),
    }


def _final_record(polygon, skeleton, superlevels) -> dict:
    partition = build_faces(polygon, skeleton)
    partition_record = _partition_record(partition)
    ownership = [
        {
            "owner": _participant(face.owner),
            "area": _sqrt_sum(face.doubled_area),
            "points": [[_sqrt_sum(x), _sqrt_sum(y)] for x, y in face.points],
        }
        for face in partition.faces
    ]
    ownership.sort(
        key=lambda item: json.dumps(item, sort_keys=True, separators=(",", ":"))
    )
    coverage = [
        _coverage_record(coverage_at(partition, alpha))
        for alpha in (Fraction(1, 4), Fraction(1), Fraction(3))
    ]
    coverage_by_alpha = [
        {
            "alpha": record["alpha"],
            "outcome": record["outcome"],
            "sha256": _sha256(record),
        }
        for record in coverage
    ]
    proof = {
        "status": skeleton.proof_status.value,
        "obligations": _proof_projection(skeleton.proof_obligations),
    }
    return {
        "outcome": skeleton.outcome.value,
        "semantic_digest": semantic_digest(skeleton),
        "topology_after_each_superlevel": [
            {
                "time": state["time"],
                "sha256": state["live_topology_sha256"],
            }
            for state in superlevels
        ],
        "state_after_each_superlevel": superlevels,
        "partition_sha256": _sha256(partition_record),
        "partition_outcome": partition.outcome.value,
        "ownership_sha256": _sha256(ownership),
        "coverage_by_alpha": coverage_by_alpha,
        "coverage_sha256": _sha256(coverage),
        "proof_verdict_sha256": _sha256(proof),
        "proof_status": skeleton.proof_status.value,
    }


def _apply_level_edge_first(self, level) -> None:
    """Counterfactual order only: same branches as production, reversed halves."""

    supported = {EventKind.SPLIT, EventKind.EDGE}
    for event in level:
        if event.kind in supported:
            continue
        self.counters["unsupported_event_kind_dropped"] += 1
        valid_vertices = tuple(
            ident
            for ident in (event.vertex, event.peer)
            if 0 <= ident < len(self.vertices)
        )
        carried_edge = self._edge_keys(event.edge)
        self._record_obligation(
            cause=ProofObligationBranch.UNSUPPORTED_EVENT_KIND,
            disposition=ProofObligationDisposition.UNSUPPORTED_EVENT_KIND_DROPPED,
            vertex_ids=valid_vertices,
            participant_edge_keys=carried_edge,
            target_edge_keys=carried_edge,
            level=event.time,
            event_kind=event.kind,
        )

    collapses = [event for event in level if event.kind is EventKind.EDGE]
    live_collapses = [
        event for event in collapses if self._edge_event_is_live(event)
    ]
    self.counters["discarded_stale_candidates"] += len(collapses) - len(
        live_collapses
    )
    for cluster in cluster_by_point(tuple(live_collapses)):
        self._apply_multi_edge(list(cluster))

    splits = [event for event in level if event.kind is EventKind.SPLIT]
    live_splits = [event for event in splits if self._split_is_live(event)]
    self.counters["discarded_stale_candidates"] += len(splits) - len(
        live_splits
    )
    meetings, cuts = self._separate_vertex_meetings(live_splits)
    cuts += self._apply_vertex_meetings(meetings)
    cuts = [event for event in cuts if self._split_is_live(event)]
    separated = self._dedupe_by_vertex(cuts)
    if separated is None:
        return
    for edge_id, group in self._group_splits(separated):
        self._apply_multi_split(edge_id, group)


def _run(polygon, *, edge_first: bool) -> dict:
    original_apply = skeleton_module._Builder._apply_level
    original_discharge = skeleton_module._Builder._discharge_observed_obligations
    superlevels = []

    def observed_discharge(builder):
        original_discharge(builder)
        if not has_same_time_residual(builder.queue, builder.now):
            superlevels.append(_superlevel_state(builder))

    skeleton_module._Builder._apply_level = (
        _apply_level_edge_first if edge_first else original_apply
    )
    skeleton_module._Builder._discharge_observed_obligations = observed_discharge
    try:
        skeleton = skeleton_module.build_skeleton(polygon)
    finally:
        skeleton_module._Builder._apply_level = original_apply
        skeleton_module._Builder._discharge_observed_obligations = original_discharge
    return _final_record(polygon, skeleton, superlevels)


def main() -> None:
    corpus = dict(named_corpus()) | dict(partial_source_corpus())
    results = {}
    for name in CASES:
        polygon = corpus[name]
        split_first = _run(polygon, edge_first=False)
        edge_first = _run(polygon, edge_first=True)
        split_states = split_first.pop("state_after_each_superlevel")
        edge_states = edge_first.pop("state_after_each_superlevel")
        axes = {
            key: split_first[key] == edge_first[key]
            for key in (
                "outcome",
                "semantic_digest",
                "topology_after_each_superlevel",
                "partition_sha256",
                "partition_outcome",
                "ownership_sha256",
                "coverage_by_alpha",
                "coverage_sha256",
                "proof_verdict_sha256",
                "proof_status",
            )
        }
        split_topology = split_states
        edge_topology = edge_states
        first_differing_superlevel = None
        for index in range(max(len(split_topology), len(edge_topology))):
            left = split_topology[index] if index < len(split_topology) else None
            right = edge_topology[index] if index < len(edge_topology) else None
            if left != right:
                first_differing_superlevel = {
                    "index": index,
                    "different_state_axes": []
                    if left is None or right is None
                    else sorted(
                        key
                        for key in left
                        if key != "time" and left[key] != right[key]
                    ),
                    "split_first": left,
                    "edge_first": right,
                }
                break
        axis_order = (
            "topology_after_each_superlevel",
            "outcome",
            "partition_outcome",
            "partition_sha256",
            "ownership_sha256",
            "coverage_by_alpha",
            "coverage_sha256",
            "proof_status",
            "proof_verdict_sha256",
            "semantic_digest",
        )
        results[name] = {
            "equal_axes": axes,
            "different_axes": sorted(key for key, equal in axes.items() if not equal),
            "first_differing_axis": next(
                (key for key in axis_order if not axes[key]), None
            ),
            "first_differing_superlevel": first_differing_superlevel,
            "split_first": split_first,
            "edge_first": edge_first,
        }
    stopped = [name for name, result in results.items() if result["different_axes"]]
    json.dump(
        {
            "outcome": "SUPERLEVEL_ORDER_DEPENDENCE_FOUND"
            if stopped
            else "NO_ORDER_DEPENDENCE_FOUND",
            "cases_compared": len(results),
            "cases_with_difference": len(stopped),
            "stopped_cases": stopped,
            "remaining_heap_and_input_permutations": (
                "NOT_RUN_AFTER_MANDATORY_STOP" if stopped else "REQUIRED"
            ),
            "results": results,
        },
        sys.stdout,
        ensure_ascii=False,
        indent=2,
        sort_keys=True,
    )
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
