"""Собрать диагностический P0-2 -> P0-2b differential receipt."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
import json
from pathlib import Path

from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.events import EventKind, cluster_by_point
from cftuv_envelope.wavefront.proof import (
    ProofObligationBranch,
    ProofObligationDisposition,
)
from cftuv_envelope.wavefront.skeleton import build_skeleton
import test_wavefront_superlevel_transaction as transaction_test
from wavefront_cases import named_corpus, partial_source_corpus


OUTPUT = Path(__file__).with_name("delta_receipt_v2.json")
PRODUCT_AXES = (
    "partial_source::ell_12_source_edges_0_1",
    "partial_source::ell_12_source_edges_4_5",
    "partial_source::ell_12_source_without_the_reflex_pair",
    "partial_source::staircase_source_edges_0_1",
    "partial_source::staircase_source_edges_6_7",
)
_CASES = (
    *((f"named::{name}", polygon) for name, polygon in named_corpus()),
    *((f"partial_source::{name}", polygon) for name, polygon in partial_source_corpus()),
)
_LEGACY_COUNTER_NAMES = (
    "coincident_split_targets",
    "discarded_stale_candidates",
    "edge_events",
    "motorcycle_march_steps",
    "motorcycle_trace_crashes",
    "motorcycle_trace_pairs",
    "motorcycle_traces",
    "motorcycle_unbounded_traces",
    "motorcycle_wall_crashes",
    "motorcycle_wall_tests",
    "multi_participant_nodes",
    "peaks",
    "refused_filter_beyond_trace",
    "refused_filter_edge_is_own",
    "refused_filter_event_in_the_past",
    "refused_filter_point_outside_front",
    "refused_filter_solo_vertex",
    "refused_filter_span_does_not_collapse",
    "refused_filter_span_is_born_zero",
    "refused_filter_triple_never_concurrent",
    "refused_no_rule_joint_is_antiparallel",
    "refused_no_rule_joint_is_codirectional",
    "refused_no_rule_joint_is_codirectional_at_different_speeds",
    "refused_no_rule_meeting_not_reconnectable",
    "refused_no_rule_span_vanished",
    "refused_no_rule_triple_always_concurrent",
    "resting_steiner_vertices",
    "ridges",
    "split_candidates_beyond_trace",
    "split_candidates_examined",
    "split_events",
    "split_search_exhaustive_segments",
    "split_search_exhaustive_vertices",
    "start_events",
    "switch_events",
    "vertex_meeting_events",
)
_FOCUSED_COLLECTION_BASELINE = (
    "cf09697eca2de348a2e06fe1c2de84f74f4cb2d1",
    177,
)
_FOCUSED_COLLECTION_ADDED = (
    "tests/test_wavefront_proof_obligations.py::test_cross_has_no_unproven_span_candidate_or_obligation",
    "tests/test_wavefront_proof_obligations.py::test_observed_debt_has_discharge_and_survival_lifecycles",
    "tests/test_wavefront_proof_obligations.py::test_star_seed_4_resolves_endpoint_contacts_atomically",
    "tests/test_wavefront_proof_obligations.py::test_transaction_preserves_an_injected_live_unproven_span",
    "tests/test_wavefront_same_time_closure.py::test_cross_is_consumed_without_a_dynamic_same_time_residual",
    "tests/test_wavefront_same_time_closure.py::test_transaction_drains_every_superlevel_without_moving_accepted_digests",
    "tests/test_wavefront_same_time_closure.py::test_transaction_emits_only_canonical_multiway_components",
    "tests/test_wavefront_same_time_closure.py::test_transaction_preserves_the_accepted_63_case_oracle",
    "tests/test_wavefront_superlevel_transaction.py::test_close_and_proof_wait_for_dynamic_same_time_fixed_point",
    "tests/test_wavefront_superlevel_transaction.py::test_cross_node_projection_is_byte_identical_to_pretransaction_anchor",
    "tests/test_wavefront_superlevel_transaction.py::test_independent_same_point_incidents_remain_separate_and_canonical",
    "tests/test_wavefront_superlevel_transaction.py::test_indistinguishable_occurrence_twins_refuse_before_runtime_tie",
    "tests/test_wavefront_superlevel_transaction.py::test_main_product_has_one_union_incidence_component_node[ell_12_source_edges_0_1]",
    "tests/test_wavefront_superlevel_transaction.py::test_main_product_has_one_union_incidence_component_node[ell_12_source_edges_4_5]",
    "tests/test_wavefront_superlevel_transaction.py::test_main_product_has_one_union_incidence_component_node[ell_12_source_without_the_reflex_pair]",
    "tests/test_wavefront_superlevel_transaction.py::test_main_product_has_one_union_incidence_component_node[staircase_source_edges_0_1]",
    "tests/test_wavefront_superlevel_transaction.py::test_main_product_has_one_union_incidence_component_node[staircase_source_edges_6_7]",
    "tests/test_wavefront_superlevel_transaction.py::test_staircase_3_4_names_the_t4_mixed_component",
    "tests/test_wavefront_superlevel_transaction.py::test_staircase_edge_6_regenerates_live_split_after_t4_rewire",
    "tests/test_wavefront_superlevel_transaction.py::test_star_16_seed_11_conditional_auth_oracle",
)
_FOCUSED_COLLECTION_REMOVED_OR_RENAMED = (
    "tests/test_wavefront_proof_obligations.py::test_cross_queues_four_unproven_spans_but_applies_none",
    "tests/test_wavefront_proof_obligations.py::test_observed_debt_has_discharge_fallback_and_survival_lifecycles",
    "tests/test_wavefront_proof_obligations.py::test_star_seed_4_names_three_unproven_meeting_fallbacks",
    "tests/test_wavefront_same_time_closure.py::test_phase_b_merges_exactly_11_duplicate_cases_and_preserves_invariants",
    "tests/test_wavefront_same_time_closure.py::test_phase_b_preserves_nonduplicate_digests_and_all_legacy_axes",
    "tests/test_wavefront_same_time_closure.py::test_phase_c_drains_every_superlevel_without_changing_b_digests",
    "tests/test_wavefront_same_time_closure.py::test_phase_zero_cross_same_time_residual_semantics",
    "tests/test_wavefront_superlevel_transaction.py::test_staircase_3_4_names_the_midrun_split_split_component",
)


def _apply_p0_2_split_first(self, level) -> None:
    """Точная P0-2 ветка из родителя a091028 для differential only."""

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
            disposition=(
                ProofObligationDisposition.UNSUPPORTED_EVENT_KIND_DROPPED
            ),
            vertex_ids=valid_vertices,
            participant_edge_keys=carried_edge,
            target_edge_keys=carried_edge,
            level=event.time,
            event_kind=event.kind,
        )

    splits = [event for event in level if event.kind is EventKind.SPLIT]
    collapses = [event for event in level if event.kind is EventKind.EDGE]
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

    live_collapses = [
        event for event in collapses if self._edge_event_is_live(event)
    ]
    self.counters["discarded_stale_candidates"] += len(collapses) - len(
        live_collapses
    )
    for cluster in cluster_by_point(tuple(live_collapses)):
        self._apply_multi_edge(list(cluster))


@contextmanager
def _application(apply):
    original = skeleton_module._Builder._apply_level
    skeleton_module._Builder._apply_level = apply
    try:
        yield
    finally:
        skeleton_module._Builder._apply_level = original


def _skeleton_projection(polygon, apply=None) -> dict:
    if apply is None:
        skeleton = build_skeleton(polygon)
    else:
        with _application(apply):
            skeleton = build_skeleton(polygon)
    return {
        "outcome": skeleton.outcome.value,
        "semantic_digest": semantic_digest(skeleton),
        "node_records": [node_record(node) for node in skeleton.nodes],
        "proof_status": skeleton.proof_status.value,
        "canonical_proof_projection": transaction_test._REPRO._proof_projection(
            skeleton.proof_obligations
        ),
        "counters": dict(skeleton.counters),
    }


def _run_projection(polygon, apply=None) -> dict:
    record = transaction_test._run(polygon, apply=apply)
    return {
        "topology_trace": record["topology_after_each_superlevel"],
        "state_trace": record["state_after_each_superlevel"],
        "partition_sha256": record["partition_sha256"],
        "partition_outcome": record["partition_outcome"],
        "partition_geometric_projection": record[
            "partition_geometric_projection"
        ],
        "ownership_sha256": record["ownership_sha256"],
        "coverage_by_alpha": record["coverage_by_alpha"],
        "coverage_sha256": record["coverage_sha256"],
        "coverage_geometric_projection": record[
            "coverage_geometric_projection"
        ],
    }


def _first_sequence_difference(old, new):
    for index in range(max(len(old), len(new))):
        left = old[index] if index < len(old) else None
        right = new[index] if index < len(new) else None
        if left != right:
            return {"index": index, "old": left, "new": right}
    return None


def _first_difference(old: dict, new: dict) -> dict | None:
    for field in ("outcome", "semantic_digest"):
        if old[field] != new[field]:
            return {"record": field, "old": old[field], "new": new[field]}
    for field in ("node_records", "topology_trace", "state_trace"):
        difference = _first_sequence_difference(old[field], new[field])
        if difference is not None:
            return {"record": field, **difference}
    for field in (
        "partition_sha256",
        "ownership_sha256",
        "coverage_sha256",
        "proof_status",
        "canonical_proof_projection",
        "counters",
    ):
        if old[field] != new[field]:
            return {"record": field, "old": old[field], "new": new[field]}
    return None


def _case_record(case_id, polygon) -> dict:
    old = _skeleton_projection(polygon, _apply_p0_2_split_first)
    old.update(_run_projection(polygon, _apply_p0_2_split_first))
    new = _skeleton_projection(polygon)
    new.update(_run_projection(polygon))
    return {
        "case_id": case_id,
        "old": old,
        "new": new,
        "first_differing_component_or_record": _first_difference(old, new),
    }


def _proof_counter(records) -> dict[str, int]:
    counts = {}
    for record in records:
        key = f"{record['cause']}|{record['disposition']}"
        counts[key] = counts.get(key, 0) + 1
    return dict(sorted(counts.items()))


def _record_counter(records):
    counts = {}
    values = {}
    for record in records:
        key = json.dumps(record, sort_keys=True, separators=(",", ":"))
        counts[key] = counts.get(key, 0) + 1
        values[key] = record
    return counts, values


def _proof_mapping(cases) -> dict:
    mapped_cases = []
    old_total = {}
    new_total = {}
    for case in cases:
        old_records = case["old"]["canonical_proof_projection"]
        new_records = case["new"]["canonical_proof_projection"]
        old_counts = _proof_counter(old_records)
        new_counts = _proof_counter(new_records)
        for key, count in old_counts.items():
            old_total[key] = old_total.get(key, 0) + count
        for key, count in new_counts.items():
            new_total[key] = new_total.get(key, 0) + count
        old_exact, old_values = _record_counter(old_records)
        new_exact, new_values = _record_counter(new_records)
        vanished = [
            {"count": count - new_exact.get(key, 0), "record": old_values[key]}
            for key, count in sorted(old_exact.items())
            if count > new_exact.get(key, 0)
        ]
        added = [
            {"count": count - old_exact.get(key, 0), "record": new_values[key]}
            for key, count in sorted(new_exact.items())
            if count > old_exact.get(key, 0)
        ]
        mapped_cases.append(
            {
                "case_id": case["case_id"],
                "old_counts": old_counts,
                "new_counts": new_counts,
                "vanished_records": vanished,
                "added_records": added,
            }
        )
    keys = sorted(set(old_total) | set(new_total))
    return {
        "schema": "P0_2B_PROOF_CAUSE_CASE_MAPPING_V1",
        "diagnostic_only": True,
        "old_total": sum(old_total.values()),
        "new_total": sum(new_total.values()),
        "by_cause_disposition": {
            key: {
                "old": old_total.get(key, 0),
                "new": new_total.get(key, 0),
                "delta": new_total.get(key, 0) - old_total.get(key, 0),
            }
            for key in keys
        },
        "cases": mapped_cases,
    }


def _sha256_json(value) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _node_locus(record) -> dict:
    return {
        key: record[key]
        for key in (
            "time_dividend",
            "time_divisor",
            "point_x",
            "point_y",
        )
    }


def _changed_node_loci(old_records, new_records) -> list[dict]:
    grouped = ({}, {})
    for records, buckets in zip((old_records, new_records), grouped):
        for record in records:
            locus = _node_locus(record)
            buckets.setdefault(_sha256_json(locus), []).append(record)
    changes = []
    for locus_sha256 in sorted(set(grouped[0]) | set(grouped[1])):
        old = grouped[0].get(locus_sha256, [])
        new = grouped[1].get(locus_sha256, [])
        if old == new:
            continue
        new_kinds = {
            kind
            for record in new
            for kind in record.get("kinds", (record["kind"],))
        }
        old_kinds = {
            kind
            for record in old
            for kind in record.get("kinds", (record["kind"],))
        }
        union = (
            len(new) == 1
            and new[0]["kind"] == "MULTIWAY"
            and old_kinds <= new_kinds
        )
        changes.append(
            {
                "locus_sha256": locus_sha256,
                "old_records_sha256": _sha256_json(old),
                "new_records_sha256": _sha256_json(new),
                "old_kinds": sorted(old_kinds),
                "new_kinds": sorted(new_kinds),
                "old_participants": sorted(
                    {
                        tuple(participant)
                        for record in old
                        for participant in record["participants"]
                    }
                ),
                "new_participants": sorted(
                    {
                        tuple(participant)
                        for record in new
                        for participant in record["participants"]
                    }
                ),
                "justification": (
                    "ATOMIC_RESOURCE_CONNECTED_UNION"
                    if union
                    else "ATOMIC_TRANSACTION_CANONICALIZATION"
                ),
            }
        )
    return changes


def _compact_side(side: dict) -> dict:
    return {
        "outcome": side["outcome"],
        "semantic_digest": side["semantic_digest"],
        "node_records_sha256": _sha256_json(side["node_records"]),
        "topology_trace_sha256": _sha256_json(side["topology_trace"]),
        "state_trace_sha256": _sha256_json(side["state_trace"]),
        "partition_outcome": side["partition_outcome"],
        "partition_sha256": side["partition_sha256"],
        "partition_geometric_projection_sha256": _sha256_json(
            side["partition_geometric_projection"]
        ),
        "ownership_sha256": side["ownership_sha256"],
        "coverage_sha256": [
            record["sha256"] for record in side["coverage_by_alpha"]
        ],
        "coverage_geometric_projection_sha256": _sha256_json(
            side["coverage_geometric_projection"]
        ),
        "proof_status": side["proof_status"],
        "proof_projection_sha256": _sha256_json(
            side["canonical_proof_projection"]
        ),
        "proof_counts": _proof_counter(side["canonical_proof_projection"]),
        "legacy_counters": {
            name: side["counters"][name] for name in _LEGACY_COUNTER_NAMES
        },
    }


def main() -> None:
    cases = [_case_record(case_id, polygon) for case_id, polygon in _CASES]
    axes = (
        "outcome",
        "semantic_digest",
        "node_records",
        "topology_trace",
        "state_trace",
        "partition_sha256",
        "partition_outcome",
        "partition_geometric_projection",
        "ownership_sha256",
        "coverage_by_alpha",
        "coverage_sha256",
        "coverage_geometric_projection",
        "proof_status",
        "canonical_proof_projection",
        "counters",
    )
    summary = {
        axis: sum(case["old"][axis] != case["new"][axis] for case in cases)
        for axis in axes
    }
    compact_cases = []
    for case in cases:
        changed_axes = {
            axis: case["old"][axis] != case["new"][axis] for axis in axes
        }
        compact_cases.append(
            {
                "case_id": case["case_id"],
                "changed_axes": changed_axes,
                "old": _compact_side(case["old"]),
                "new": _compact_side(case["new"]),
                "changed_node_loci": _changed_node_loci(
                    case["old"]["node_records"],
                    case["new"]["node_records"],
                ),
            }
        )
    proof = _proof_mapping(cases)
    product_faces_and_coverage = {
        case["case_id"]: {
            side: {
                "partition_sha256": case[side]["partition_sha256"],
                "ownership_sha256": case[side]["ownership_sha256"],
                "coverage_sha256": [
                    record["sha256"]
                    for record in case[side]["coverage_by_alpha"]
                ],
            }
            for side in ("old", "new")
        }
        for case in cases
        if case["case_id"] in PRODUCT_AXES
    }
    focused_collection = {
        "baseline_commit": _FOCUSED_COLLECTION_BASELINE[0],
        "baseline_unique_nodeids": _FOCUSED_COLLECTION_BASELINE[1],
        "current_unique_nodeids": 189,
        "added": list(_FOCUSED_COLLECTION_ADDED),
        "removed_or_renamed": list(_FOCUSED_COLLECTION_REMOVED_OR_RENAMED),
        "duplicate_nodeids": 0,
        "skipped": 0,
        "xfailed": 0,
        "xpassed": 0,
    }
    focused_collection["canonical_sha256"] = _sha256_json(
        focused_collection
    )
    receipt = {
        "schema": "P0_2B_SUPERLEVEL_TRANSACTION_DELTA_RECEIPT_V2",
        "old_parent": "a0910285e624fa053903187820b4d4c8635b5a34",
        "authority_docs": "2851e85053fc1047fc938a51dcc3b5d8dceed88b",
        "case_count": len(cases),
        "reproduce": (
            "$env:PYTHONPATH='kernel/src;kernel/tests'; python "
            "artifacts/p0_2b_superlevel_transaction/"
            "generate_delta_receipt.py"
        ),
        "summary": summary,
        "proof_aggregate": {
            "old_total": proof["old_total"],
            "new_total": proof["new_total"],
            "by_cause_disposition": proof["by_cause_disposition"],
        },
        "focused_collection_attribution": focused_collection,
        "product_faces_and_coverage": product_faces_and_coverage,
        "cases": compact_cases,
    }
    OUTPUT.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
