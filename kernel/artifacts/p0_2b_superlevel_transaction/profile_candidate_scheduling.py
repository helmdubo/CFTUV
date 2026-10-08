"""Factual P0-2b candidate scheduling profiler for SEM-CLB patch006.

Run from the selected checkout root:

    PYTHONPATH=kernel/src;kernel/tests python \
        artifacts/p0_2b_superlevel_transaction/profile_candidate_scheduling.py

The script monkeypatches counters only. Candidate predicates, filters, queue
ordering, and product data are not changed.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import cftuv_envelope as kernel
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.event_time import compare_times
from cftuv_envelope.wavefront.events import EventQueueV1


CASE = "building_all_seams_patch_006_lost_resolved_v1"


def _json_number(value):
    if isinstance(value, Fraction):
        return [value.numerator, value.denominator]
    return value


def _sqrt_sum(value):
    return [
        [radicand, _json_number(coefficient)]
        for radicand, coefficient in value.terms
    ]


def _time(value):
    canonical = value.canonical()
    return [_json_number(canonical.dividend), _sqrt_sum(canonical.divisor)]


def _candidate_key(event):
    return (
        event.kind.value,
        event.vertex,
        event.edge,
        repr(_time(event.time)),
        repr((_sqrt_sum(event.point.x), _sqrt_sum(event.point.y))),
    )


def _state_key(builder, vertex, edge):
    return repr(
        (
            vertex.ident,
            vertex.prev,
            vertex.next,
            vertex.prev_edge,
            vertex.next_edge,
            vertex.alive,
            _time(vertex.birth),
            edge.ident,
            edge.key,
            edge.span,
        )
    )


def _sha256(payload) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_inputs():
    root = (
        Path.cwd()
        / "kernel"
        / "fixtures"
        / "sem_clb_02_lost_domains_v1"
        / "cases"
        / CASE
    )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    return snapshot, request, domain.patch_domain_id


def main() -> None:
    method_calls = Counter()
    predicate_calls = Counter()
    exact_point_calls = Counter()
    pushes = Counter()
    timing = Counter()
    pushed_keys = defaultdict(Counter)
    split_states = defaultdict(Counter)
    per_level = defaultdict(Counter)
    stack = []

    originals = {
        "enqueue_for": skeleton_module._Builder._enqueue_for,
        "enqueue_splits_against": (
            skeleton_module._Builder._enqueue_splits_against
        ),
        "enqueue_edge_event": skeleton_module._Builder._enqueue_edge_event,
        "split_candidate": skeleton_module._Builder._split_candidate,
        "event_point": skeleton_module._event_point_with_prime_universe,
        "queue_push": EventQueueV1.push,
    }

    def source():
        return stack[-1][0] if stack else "unscoped"

    def level_key(builder):
        return repr(_time(builder.now))

    def wrap_method(name, label):
        original = originals[name]

        def wrapped(builder, *args, **kwargs):
            method_calls[label] += 1
            per_level[level_key(builder)][f"method:{label}"] += 1
            stack.append((label, builder))
            try:
                return original(builder, *args, **kwargs)
            finally:
                stack.pop()

        return wrapped

    def split_candidate(builder, vertex, edge):
        label = source()
        predicate_calls[label] += 1
        per_level[level_key(builder)][f"predicate:{label}"] += 1
        split_states[label][
            (vertex.ident, edge.ident, _state_key(builder, vertex, edge))
        ] += 1
        return originals["split_candidate"](builder, vertex, edge)

    def event_point(first, second, time, prime_universe):
        label = source()
        exact_point_calls[label] += 1
        if stack:
            per_level[level_key(stack[-1][1])][f"exact_point:{label}"] += 1
        return originals["event_point"](first, second, time, prime_universe)

    def queue_push(queue, event):
        label = source()
        builder = stack[-1][1] if stack else None
        pushes[label] += 1
        pushed_keys[label][_candidate_key(event)] += 1
        if builder is not None:
            relation = compare_times(event.time, builder.now)
            timing[(label, "same_time" if relation == 0 else "future")] += 1
            per_level[level_key(builder)][f"push:{label}"] += 1
        return originals["queue_push"](queue, event)

    skeleton_module._Builder._enqueue_for = wrap_method(
        "enqueue_for", "enqueue_for"
    )
    skeleton_module._Builder._enqueue_splits_against = wrap_method(
        "enqueue_splits_against", "enqueue_splits_against"
    )
    skeleton_module._Builder._enqueue_edge_event = wrap_method(
        "enqueue_edge_event", "edge"
    )
    skeleton_module._Builder._split_candidate = split_candidate
    skeleton_module._event_point_with_prime_universe = event_point
    EventQueueV1.push = queue_push
    try:
        snapshot, request, patch_domain_id = _load_inputs()
        prepared = prepare_conveyor(
            snapshot,
            request,
            patch_domain_id=patch_domain_id,
        )
    finally:
        EventQueueV1.push = originals["queue_push"]
        skeleton_module._event_point_with_prime_universe = originals[
            "event_point"
        ]
        skeleton_module._Builder._split_candidate = originals[
            "split_candidate"
        ]
        skeleton_module._Builder._enqueue_edge_event = originals[
            "enqueue_edge_event"
        ]
        skeleton_module._Builder._enqueue_splits_against = originals[
            "enqueue_splits_against"
        ]
        skeleton_module._Builder._enqueue_for = originals["enqueue_for"]

    (region,) = prepared.regions
    skeleton = region.skeleton
    assert skeleton is not None
    all_pushes = Counter()
    for counter in pushed_keys.values():
        all_pushes.update(counter)

    source_records = {}
    for label in sorted(
        set(method_calls) | set(predicate_calls) | set(exact_point_calls) | set(pushes)
    ):
        exact = pushed_keys[label]
        states = split_states[label]
        source_records[label] = {
            "method_calls": method_calls[label],
            "split_predicate_calls": predicate_calls[label],
            "exact_event_point_calls": exact_point_calls[label],
            "candidate_pushes": pushes[label],
            "unique_candidate_keys": len(exact),
            "duplicate_exact_candidate_pushes": sum(
                count - 1 for count in exact.values()
            ),
            "same_time_pushes": timing[(label, "same_time")],
            "future_pushes": timing[(label, "future")],
            "unique_vertex_edge_state_keys": len(states),
            "rechecked_unchanged_vertex_edge_states": sum(
                count - 1 for count in states.values()
            ),
        }

    maxima = {}
    for metric in sorted(
        {metric for values in per_level.values() for metric in values}
    ):
        value, level = max(
            (values[metric], level)
            for level, values in per_level.items()
        )
        maxima[metric] = {"count": value, "level": level}

    result = {
        "schema": "P0_2B_CANDIDATE_SCHEDULING_PROFILE_V1",
        "case": CASE,
        "commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "dirty": bool(
            subprocess.check_output(
                ["git", "status", "--short"], text=True
            ).strip()
        ),
        "outcome": skeleton.outcome.value,
        "levels": skeleton.levels,
        "node_count": len(skeleton.nodes),
        "semantic_digest": semantic_digest(skeleton),
        "sources": source_records,
        "global": {
            "candidate_pushes": sum(all_pushes.values()),
            "unique_candidate_keys": len(all_pushes),
            "duplicate_exact_candidate_pushes": sum(
                count - 1 for count in all_pushes.values()
            ),
            "split_predicate_calls": sum(predicate_calls.values()),
            "exact_event_point_calls": sum(exact_point_calls.values()),
            "unique_vertex_edge_state_keys": len(
                set().union(*(set(counter) for counter in split_states.values()))
            ),
            "rechecked_unchanged_vertex_edge_states_within_source": sum(
                record["rechecked_unchanged_vertex_edge_states"]
                for record in source_records.values()
            ),
        },
        "per_superlevel_maxima": maxima,
        "per_superlevel_sha256": _sha256(
            {
                level: dict(sorted(values.items()))
                for level, values in sorted(per_level.items())
            }
        ),
    }
    json.dump(result, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
