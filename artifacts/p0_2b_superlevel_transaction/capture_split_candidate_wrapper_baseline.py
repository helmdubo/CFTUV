"""Заморозить wrapper semantics до extraction pure split query."""

from __future__ import annotations

from collections import Counter
import hashlib
import json

from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton
from wavefront_cases import named_corpus, partial_source_corpus
from weighted_wall_differential_cases import weighted_wall_differential_corpus


def _time(time):
    canonical = time.canonical()
    return (
        (canonical.dividend.numerator, canonical.dividend.denominator),
        tuple(
            (
                radicand,
                coefficient.numerator,
                coefficient.denominator,
            )
            for radicand, coefficient in canonical.divisor.terms
        ),
    )


def _point(point):
    return tuple(
        tuple(
            (radicand, coefficient.numerator, coefficient.denominator)
            for radicand, coefficient in coordinate.terms
        )
        for coordinate in (point.x, point.y)
    )


def _obligation(obligation):
    return (
        obligation.cause.value,
        obligation.disposition.value,
        obligation.vertex_ids,
        obligation.participant_edge_keys,
        obligation.target_edge_keys,
        _time(obligation.level),
        None if obligation.event_kind is None else obligation.event_kind.value,
    )


def _sha256(value) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def main() -> None:
    original = skeleton_module._Builder._split_candidate
    active = []

    def measured(builder, vertex, edge):
        counters_before = dict(builder.counters)
        proof_before = len(builder._proof.obligations)
        candidate = original(builder, vertex, edge)
        counter_delta = tuple(
            sorted(
                (name, value - counters_before.get(name, 0))
                for name, value in builder.counters.items()
                if value != counters_before.get(name, 0)
            )
        )
        proof_delta = tuple(
            _obligation(item)
            for item in builder._proof.obligations[proof_before:]
        )
        active.append(
            (
                vertex.ident,
                builder.edges[vertex.prev_edge].key,
                builder.edges[vertex.next_edge].key,
                edge.ident,
                edge.key,
                None
                if candidate is None
                else (
                    candidate.kind.value,
                    _time(candidate.time),
                    _point(candidate.point),
                    candidate.vertex,
                    candidate.peer,
                    candidate.edge,
                ),
                counter_delta,
                proof_delta,
            )
        )
        return candidate

    skeleton_module._Builder._split_candidate = measured
    groups = {
        "corpus_63": tuple(named_corpus()) + tuple(partial_source_corpus()),
        "p0_3_23": tuple(
            (case.name, case.polygon)
            for case in weighted_wall_differential_corpus()
        ),
    }
    receipt = {}
    try:
        for group_name, cases in groups.items():
            for search in SplitSearch:
                rows = []
                for case_name, polygon in cases:
                    active.clear()
                    skeleton = build_skeleton(polygon, split_search=search)
                    rows.append(
                        (
                            case_name,
                            tuple(active),
                            skeleton.outcome.value,
                            skeleton.levels,
                            tuple(skeleton.counters),
                            skeleton.proof_status.value,
                            tuple(
                                _obligation(item)
                                for item in skeleton.proof_obligations
                            ),
                        )
                    )
                key = f"{group_name}::{search.value}"
                receipt[key] = {
                    "case_count": len(rows),
                    "candidate_query_count": sum(
                        len(row[1]) for row in rows
                    ),
                    "sha256": _sha256(rows),
                }
    finally:
        skeleton_module._Builder._split_candidate = original
    print(
        json.dumps(
            {
                "schema": "P0_2B_SPLIT_WRAPPER_BASELINE_V1",
                "groups": receipt,
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
