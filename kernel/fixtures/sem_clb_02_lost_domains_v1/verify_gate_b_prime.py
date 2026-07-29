"""Proof-local Phase 1 candidate for SEM-CLB-02-R Gate B-prime.

The frozen product is never edited.  During verifier execution only, a scoped
adapter applies the existing exact event-born straight/sliding predicate to
seed vertices.  The adapter records the seed before and after classification
and restores every patched callable on both normal and exceptional exits.
Runtime timings are printed but deliberately excluded from the receipt.
"""

from __future__ import annotations

import argparse
import ast
import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


SCHEMA = "cftuv.envelope.sem_clb_02_gate_b_prime_phase1.v1"
PROOF_BASE_REVISION = "95c619e36ac0f23dc5cb4f8767f2bc143b78706f"
PHASE0_RECEIPT_SHA256 = (
    "16ed724fa6f7161d07efbd9f397458446aa1a63aca3dffe736e055b00c92969c"
)
PHASE0_RECEIPT_PATH = (
    "kernel/fixtures/sem_clb_02_lost_domains_v1/gate_b_prime_receipt.json"
)
EXPECTED_KERNEL_SRC_TREE_OID = "66620708899272b0f249b583a40cd83bb1f70495"
HISTORICAL_KERNEL_SRC_TREE_OID = "d9574f7b7c193ed48fb3dc71aa58d28d62be3bcb"
FIXTURE_IDS = (
    "building_all_seams_patch_001_lost_resolved_v1",
    "building_all_seams_patch_006_lost_resolved_v1",
    "building_all_seams_patch_011_lost_resolved_v1",
    "building_all_seams_patch_105_lost_resolved_v1",
)
ACCEPTED_RECEIPT_SHA256 = {
    "gate_a_prime_receipt.json": (
        "b25a44c23dbe46a29806384ceb906210c505aa3aa929ee00d5515a6926bc28fe"
    ),
    "gate_a_double_prime_receipt.json": (
        "891d008ffc27a03a1b819ed93facbb86819830b95a5a32f40c28223681e68a4e"
    ),
    "gate_a_triple_prime_receipt.json": (
        "bf1864fe80ec66ecc926ec02b82a6204a25e3b7f63032adff3fb769227ebb3b4"
    ),
}
H1_POINTS = ((0, 0), (3, 0), (6, 0), (6, 4), (0, 4))
H2_POINTS = ((0, 0), (2, 0), (6, 0), (6, 4), (0, 4))
SMALL_H1_POINTS = ((0, 0), (2, 0), (4, 0), (1, 1), (0, 4))
EXPECTED_NEW_VERTEX_PREDICATE = (
    "first.a * second.b - second.a * first.b == 0 and "
    "first.a * second.a + first.b * second.b > 0 and "
    "(first.q * second.normal_squared == "
    "second.q * first.normal_squared)"
)
EXPECTED_CLASSIFIED_COUNTS = {
    "building_all_seams_patch_001_lost_resolved_v1": 4,
    "building_all_seams_patch_006_lost_resolved_v1": 5,
    "building_all_seams_patch_011_lost_resolved_v1": 3,
    "building_all_seams_patch_105_lost_resolved_v1": 1,
}
EXPECTED_SYNTHETIC = {
    "H1_SMALLER_CONCAVE": {
        "polygon_doubled_area": 8,
        "face_count": 5,
        "levels": 3,
        "node_count": 2,
    },
    "H1_REQUIRED": {
        "polygon_doubled_area": 48,
        "face_count": 5,
        "levels": 3,
        "node_count": 3,
    },
    "H2_SCALE_CONTROL": {
        "polygon_doubled_area": 48,
        "face_count": 5,
        "levels": 4,
        "node_count": 2,
    },
}


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    parser.add_argument(
        "--fixture-root",
        type=Path,
        default=Path(__file__).resolve().parent / "cases",
    )
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


def _pretty_json(payload: Any) -> bytes:
    return (
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_lf(payload: bytes) -> bytes:
    return payload.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def _git_blob_oid(payload: bytes) -> str:
    header = f"blob {len(payload)}\0".encode("ascii")
    return hashlib.sha1(header + payload).hexdigest()


def _ratio(value: Fraction | int) -> str:
    value = Fraction(value)
    if value.denominator == 1:
        return str(value.numerator)
    return f"{value.numerator}/{value.denominator}"


def _sqrt_sum(value) -> list[dict[str, Any]]:
    return [
        {"radicand": radicand, "coefficient": _ratio(coefficient)}
        for radicand, coefficient in value.terms
    ]


def _event_point(value) -> dict[str, Any]:
    return {"x": _sqrt_sum(value.x), "y": _sqrt_sum(value.y)}


def _event_time(value) -> dict[str, Any]:
    return {
        "dividend": _ratio(value.dividend),
        "divisor": _sqrt_sum(value.divisor),
    }


def _git_value(root: Path, expression: str) -> str:
    result = subprocess.run(
        ["git", "rev-parse", expression],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _git_bytes(root: Path, expression: str) -> bytes:
    result = subprocess.run(
        ["git", "show", expression],
        cwd=root,
        check=True,
        capture_output=True,
    )
    return result.stdout


def _tracked_utf8_seal(
    root: Path,
    path: Path,
    git_expression: str,
    binding: str,
) -> tuple[bytes, dict[str, Any]]:
    git_blob = _git_bytes(root, git_expression)
    canonical_lf = _canonical_lf(git_blob)
    canonical_lf.decode("utf-8")
    workspace_lf = _canonical_lf(path.read_bytes())
    observed_blob_oid = _git_value(root, git_expression)
    computed_blob_oid = _git_blob_oid(git_blob)
    return canonical_lf, {
        "path": path.resolve().relative_to(root.resolve()).as_posix(),
        "git_expression": git_expression,
        "binding": binding,
        "byte_domain": "UTF8_GIT_BLOB_CANONICAL_LF_V1",
        "git_blob_oid": observed_blob_oid,
        "git_blob_oid_recomputed": computed_blob_oid,
        "git_blob_identity_exact": observed_blob_oid == computed_blob_oid,
        "git_blob_is_canonical_lf": git_blob == canonical_lf,
        "canonical_lf_bytes": len(canonical_lf),
        "canonical_lf_sha256": _sha256(canonical_lf),
        "canonical_lf_utf8": canonical_lf.decode("utf-8"),
        "workspace_matches_after_lf_normalization": (
            workspace_lf == canonical_lf
        ),
    }


def _compact_tracked_utf8_seal(
    record: dict[str, Any],
) -> dict[str, Any]:
    return {
        key: value
        for key, value in record.items()
        if key != "canonical_lf_utf8"
    }


def _tracked_utf8_seal_exact(
    record: dict[str, Any],
    canonical: bytes | None = None,
) -> bool:
    if canonical is None:
        canonical = record["canonical_lf_utf8"].encode("utf-8")
    return (
        record["byte_domain"] == "UTF8_GIT_BLOB_CANONICAL_LF_V1"
        and b"\r" not in canonical
        and record["canonical_lf_bytes"] == len(canonical)
        and record["canonical_lf_sha256"] == _sha256(canonical)
        and record["git_blob_oid"]
        == record["git_blob_oid_recomputed"]
        == _git_blob_oid(canonical)
        and record["git_blob_identity_exact"]
        and record["git_blob_is_canonical_lf"]
        and record["workspace_matches_after_lf_normalization"]
    )


def _activate_kernel(source_root: Path):
    kernel_src = source_root / "kernel" / "src"
    sys.path.insert(0, str(kernel_src))
    import cftuv_envelope as kernel
    import cftuv_envelope.wavefront.conveyor as conveyor
    import cftuv_envelope.wavefront.skeleton as skeleton_module
    from cftuv_envelope.wavefront.bridge import line_class
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
    from cftuv_envelope.wavefront.faces import (
        build_faces,
        contour_crossings,
        doubled_shoelace,
    )
    from cftuv_envelope.wavefront.polygon import (
        LoopV1,
        PolygonV1,
        signed_double_area,
    )

    return {
        "kernel": kernel,
        "conveyor": conveyor,
        "skeleton": skeleton_module,
        "line_class": line_class,
        "build_faces": build_faces,
        "contour_crossings": contour_crossings,
        "doubled_shoelace": doubled_shoelace,
        "SqrtSumV1": SqrtSumV1,
        "LoopV1": LoopV1,
        "PolygonV1": PolygonV1,
        "signed_double_area": signed_double_area,
    }


def _exact_straight_classifier(first, second) -> bool:
    """Тот же точный предикат, что у event-born `_new_vertex`."""

    return (
        first.a * second.b - second.a * first.b == 0
        and first.a * second.a + first.b * second.b > 0
        and first.q * second.normal_squared
        == second.q * first.normal_squared
    )


def _adapter_vertex_state(builder, vertex, line_class) -> dict[str, Any]:
    """Состояние seed-вершины без вывода новых геометрических фактов."""

    first_edge = builder.edges[vertex.prev_edge]
    second_edge = builder.edges[vertex.next_edge]
    first = first_edge.line
    second = second_edge.line
    determinant = first.a * second.b - second.a * first.b
    dot = first.a * second.a + first.b * second.b
    q1_N2 = first.q * second.normal_squared
    q2_N1 = second.q * first.normal_squared
    frozen_projection = (
        vertex.point.x.scaled(first.b) - vertex.point.y.scaled(first.a)
    )
    return {
        "vertex_id": vertex.ident,
        "birth": _event_time(vertex.birth),
        "point": _event_point(vertex.point),
        "prev_edge_id": vertex.prev_edge,
        "next_edge_id": vertex.next_edge,
        "prev_span": list(first_edge.span),
        "next_span": list(second_edge.span),
        "prev_raw_line_key": [
            *first_edge.line_key[:3],
            _ratio(first_edge.line_key[3]),
        ],
        "next_raw_line_key": [
            *second_edge.line_key[:3],
            _ratio(second_edge.line_key[3]),
        ],
        "prev_support_line_class": (
            None
            if line_class is None
            else _line_class_record(first, line_class)
        ),
        "next_support_line_class": (
            None
            if line_class is None
            else _line_class_record(second, line_class)
        ),
        "prev_q_over_N": _ratio(
            Fraction(first.q, first.normal_squared)
        ),
        "next_q_over_N": _ratio(
            Fraction(second.q, second.normal_squared)
        ),
        "determinant": determinant,
        "dot": dot,
        "q1_N2": _ratio(q1_N2),
        "q2_N1": _ratio(q2_N1),
        "exact_classifier_result": (
            determinant == 0 and dot > 0 and q1_N2 == q2_N1
        ),
        "frozen_projection": _sqrt_sum(frozen_projection),
        "prev_vertex_id": vertex.prev,
        "next_vertex_id": vertex.next,
        "reflex": vertex.reflex,
        "sliding": (
            None if vertex.sliding is None else _sqrt_sum(vertex.sliding)
        ),
    }


def _state_without_sliding(records: list[dict[str, Any]]) -> bytes:
    return _pretty_json(
        [
            {key: value for key, value in item.items() if key != "sliding"}
            for item in records
        ]
    )


def _callable_identity(function) -> dict[str, Any]:
    code = function.__code__
    code_payload = (
        code.co_code
        + repr(code.co_consts).encode("utf-8")
        + repr(code.co_names).encode("utf-8")
    )
    return {
        "module": function.__module__,
        "qualname": function.__qualname__,
        "code_sha256": _sha256(code_payload),
    }


class _SeedClassifierAdapter:
    """Временная proof-local классификация seed с обязательным restore."""

    def __init__(self, skeleton_module, line_class=None) -> None:
        self.skeleton_module = skeleton_module
        self.line_class = line_class
        self.builder_type = skeleton_module._Builder
        self.original_seed_one_loop = self.builder_type._seed_one_loop
        self.original_new_vertex = self.builder_type._new_vertex
        self.original_seed_identity = _callable_identity(
            self.original_seed_one_loop
        )
        self.original_new_vertex_identity = _callable_identity(
            self.original_new_vertex
        )
        self.restored_seed_identity = None
        self.restored_new_vertex_identity = None
        self.records: list[dict[str, Any]] = []
        self.entered = False
        self.restored = False

    def __enter__(self):
        if self.entered:
            raise RuntimeError("SEED_CLASSIFIER_ADAPTER_REENTERED")
        self.entered = True
        adapter = self
        original = self.original_seed_one_loop

        def seed_one_loop_with_exact_classifier(builder, loop):
            first_vertex = len(builder.vertices)
            original(builder, loop)
            seeded = builder.vertices[first_vertex:]
            before = [
                _adapter_vertex_state(builder, vertex, adapter.line_class)
                for vertex in seeded
            ]
            classified = []
            scope_excess = 0
            for vertex in seeded:
                first = builder.edges[vertex.prev_edge].line
                second = builder.edges[vertex.next_edge].line
                if not _exact_straight_classifier(first, second):
                    continue
                if vertex.ident < first_vertex:
                    scope_excess += 1
                    continue
                vertex.sliding = adapter.skeleton_module._project(
                    first, vertex.point
                )
                builder.sliding_vertices.add(vertex.ident)
                classified.append(vertex.ident)
            after = [
                _adapter_vertex_state(builder, vertex, adapter.line_class)
                for vertex in seeded
            ]
            before_immutable = _state_without_sliding(before)
            after_immutable = _state_without_sliding(after)
            adapter.records.append(
                {
                    "builder_identity": id(builder),
                    "split_search": builder.split_search.value,
                    "seed_vertex_ids": [
                        vertex.ident for vertex in seeded
                    ],
                    "classified_vertex_ids": classified,
                    "classified_count": len(classified),
                    "seed_scope_excess_count": scope_excess,
                    "immutable_seed_state_before_sha256": _sha256(
                        before_immutable
                    ),
                    "immutable_seed_state_after_sha256": _sha256(
                        after_immutable
                    ),
                    "immutable_seed_state_unchanged": (
                        before_immutable == after_immutable
                    ),
                    "before": before,
                    "after": after,
                }
            )

        self.builder_type._seed_one_loop = seed_one_loop_with_exact_classifier
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        self.builder_type._seed_one_loop = self.original_seed_one_loop
        self.restored_seed_identity = _callable_identity(
            self.builder_type._seed_one_loop
        )
        self.restored_new_vertex_identity = _callable_identity(
            self.builder_type._new_vertex
        )
        self.restored = (
            self.restored_seed_identity == self.original_seed_identity
            and self.restored_new_vertex_identity
            == self.original_new_vertex_identity
        )
        return False


def _forced_exception_restoration_audit(skeleton_module) -> dict[str, Any]:
    original_seed = skeleton_module._Builder._seed_one_loop
    original_new = skeleton_module._Builder._new_vertex
    observed = None
    adapter = _SeedClassifierAdapter(skeleton_module)
    try:
        with adapter:
            raise RuntimeError("FORCED_ADAPTER_EXCEPTION")
    except RuntimeError as error:
        observed = str(error)
    return {
        "forced_exception": observed,
        "seed_hook_restored": (
            skeleton_module._Builder._seed_one_loop is original_seed
        ),
        "event_born_hook_unchanged": (
            skeleton_module._Builder._new_vertex is original_new
        ),
        "adapter_reported_restored": adapter.restored,
        "seed_hook_before": adapter.original_seed_identity,
        "seed_hook_after": adapter.restored_seed_identity,
        "event_born_hook_before": (
            adapter.original_new_vertex_identity
        ),
        "event_born_hook_after": adapter.restored_new_vertex_identity,
        "passed": (
            observed == "FORCED_ADAPTER_EXCEPTION"
            and adapter.restored
            and skeleton_module._Builder._seed_one_loop is original_seed
            and skeleton_module._Builder._new_vertex is original_new
        ),
    }


def _skeleton_node_record(node) -> dict[str, Any]:
    return {
        "kind": node.kind.value,
        "time": _event_time(node.time),
        "point": _event_point(node.point),
        "participants": [list(item) for item in node.participants],
        "converging_vertices": node.converging_vertices,
    }


def _skeleton_semantic_record(skeleton) -> dict[str, Any]:
    return {
        "outcome": skeleton.outcome.value,
        "levels": skeleton.levels,
        "nodes": [_skeleton_node_record(node) for node in skeleton.nodes],
        "non_search_counters": {
            key: value
            for key, value in skeleton.counters
            if not key.startswith("motorcycle_")
            and not key.startswith("split_search_")
        },
    }


def _binding_codec(kernel, binding):
    if type(binding).__name__ == "ChainStraightEvaluationGeometryBindingV2":
        return kernel.ChainStraightEvaluationGeometryBindingCodecV2
    return kernel.EvaluationGeometryBindingCodecV1


def _binding_fact_record(payload: bytes) -> dict[str, Any]:
    decoded = json.loads(payload)
    rational_count = 0
    node_records = []

    def visit(value, path: tuple[str, ...] = ()) -> None:
        nonlocal rational_count
        if isinstance(value, dict):
            if value.get("$type") == "ExactRationalV1":
                rational_count += 1
            for key, child in sorted(value.items()):
                visit(child, (*path, key))
            return
        if isinstance(value, list):
            if (
                path
                and "node" in path[-1].lower()
                and len(value) == 2
                and all(isinstance(item, int) for item in value)
            ):
                node_records.append(
                    {"path": list(path), "node": list(value)}
                )
            for index, child in enumerate(value):
                visit(child, (*path, str(index)))

    visit(decoded)
    node_bytes = _pretty_json(node_records)
    return {
        "binding_type": decoded.get("$type"),
        "lattice_scale_S_prime": decoded.get("lattice_scale"),
        "source_vertex_coordinate_count": len(
            decoded.get("source_vertex_coordinates", [])
        ),
        "exact_rational_record_count": rational_count,
        "exact_node_record_count": len(node_records),
        "exact_nodes_sha256": _sha256(node_bytes),
        "canonical_bytes": len(payload),
        "canonical_sha256": _sha256(payload),
    }


def _line_class_record(line, line_class) -> list[str] | None:
    key = line_class((line.a, line.b, line.c))
    if key is None:
        return None
    return [_ratio(item) for item in key]


def _line_record(edge, line_class, event_line_id: int | None) -> dict[str, Any]:
    line = edge.line
    return {
        "edge_id": edge.ident,
        "span": list(edge.span),
        "span_is_zero_length": (
            edge.span[0] == edge.span[2] and edge.span[1] == edge.span[3]
        ),
        "raw_line_key": [
            line.a,
            line.b,
            line.c,
            _ratio(line.q),
        ],
        "support_line_class": _line_class_record(line, line_class),
        "event_line_id": event_line_id,
        "normal_squared_N": line.normal_squared,
        "canonical_q": _ratio(line.q),
        "normalized_speed_squared_q_over_N": _ratio(
            Fraction(line.q, line.normal_squared)
        ),
    }


def _joint_record(builder, vertex, line_class) -> dict[str, Any]:
    first_edge = builder.edges[vertex.prev_edge]
    second_edge = builder.edges[vertex.next_edge]
    first = first_edge.line
    second = second_edge.line
    determinant = first.a * second.b - second.a * first.b
    dot = first.a * second.a + first.b * second.b
    speed_left = first.q * second.normal_squared
    speed_right = second.q * first.normal_squared
    support_class = _line_class_record(first, line_class)
    class_members = [
        edge
        for edge in builder.edges
        if _line_class_record(edge.line, line_class) == support_class
    ]
    previous = builder.vertices[vertex.prev]
    following = builder.vertices[vertex.next]
    previous_outcome = builder._edge_event_time(previous, vertex)[1]
    following_outcome = builder._edge_event_time(vertex, following)[1]
    return {
        "vertex_id": vertex.ident,
        "point": _event_point(vertex.point),
        "prev_vertex_id": vertex.prev,
        "next_vertex_id": vertex.next,
        "prev_edge_id": vertex.prev_edge,
        "next_edge_id": vertex.next_edge,
        "prev_span": list(first_edge.span),
        "next_span": list(second_edge.span),
        "prev_support": _line_record(
            first_edge,
            line_class,
            builder.line_id.get(first_edge.line_key),
        ),
        "next_support": _line_record(
            second_edge,
            line_class,
            builder.line_id.get(second_edge.line_key),
        ),
        "determinant": determinant,
        "dot": dot,
        "speed_equality_left_q1_N2": _ratio(speed_left),
        "speed_equality_right_q2_N1": _ratio(speed_right),
        "normalized_speeds_equal": speed_left == speed_right,
        "same_support_line_class": (
            support_class
            == _line_class_record(second, line_class)
        ),
        "support_line_class_member_count": len(class_members),
        "support_line_class_distinct_raw_key_count": len(
            {edge.line_key for edge in class_members}
        ),
        "raw_line_keys_equal": first_edge.line_key == second_edge.line_key,
        "event_born_new_vertex_predicate": (
            determinant == 0 and dot > 0 and speed_left == speed_right
        ),
        "reflex": vertex.reflex,
        "sliding_initialized": vertex.sliding is not None,
        "previous_edge_event_outcome": previous_outcome.value,
        "following_edge_event_outcome": following_outcome.value,
    }


def _seed_snapshot(builder, line_class) -> dict[str, Any]:
    edges = [
        _line_record(
            edge,
            line_class,
            builder.line_id.get(edge.line_key),
        )
        for edge in builder.edges
    ]
    classes: dict[tuple[str, ...], list[dict[str, Any]]] = {}
    for record in edges:
        class_record = record["support_line_class"]
        if class_record is None:
            continue
        classes.setdefault(tuple(class_record), []).append(record)
    class_memberships = [
        {
            "support_line_class": list(key),
            "edge_ids": [item["edge_id"] for item in members],
            "spans": [item["span"] for item in members],
            "raw_line_keys": [item["raw_line_key"] for item in members],
            "normalized_speed_squared_q_over_N": [
                item["normalized_speed_squared_q_over_N"]
                for item in members
            ],
        }
        for key, members in sorted(classes.items())
    ]
    vertices = [
        _joint_record(builder, vertex, line_class)
        for vertex in builder.vertices
    ]
    straight = [
        item for item in vertices if item["event_born_new_vertex_predicate"]
    ]
    return {
        "queue_length_after_seed": len(builder.queue),
        "queue_pushed_after_seed": builder.queue.pushed,
        "queue_popped_after_seed": builder.queue.popped,
        "edge_count": len(edges),
        "vertex_count": len(vertices),
        "edges": edges,
        "line_class_memberships": class_memberships,
        "seed_vertices": vertices,
        "certified_straight_seed_vertices": straight,
        "certified_straight_seed_vertex_count": len(straight),
        "certified_straight_seed_sliding_count": sum(
            item["sliding_initialized"] for item in straight
        ),
        "certified_straight_seed_equal_speed_count": sum(
            item["normalized_speeds_equal"] for item in straight
        ),
        "certified_straight_seed_raw_key_rescaling_count": sum(
            not item["raw_line_keys_equal"] for item in straight
        ),
    }


def _live_cycles(builder) -> list[list[int]]:
    live = {vertex.ident for vertex in builder.vertices if vertex.alive}
    cycles: list[list[int]] = []
    seen: set[int] = set()
    for start in sorted(live):
        if start in seen:
            continue
        cycle = []
        cursor = start
        while cursor in live and cursor not in seen:
            seen.add(cursor)
            cycle.append(cursor)
            cursor = builder.vertices[cursor].next
        cycles.append(cycle)
    return cycles


def _terminal_snapshot(builder, skeleton, line_class) -> dict[str, Any]:
    live = []
    for vertex in builder.vertices:
        if not vertex.alive:
            continue
        position = builder._position(vertex, builder.now)
        live.append(
            {
                "vertex_id": vertex.ident,
                "prev_vertex_id": vertex.prev,
                "next_vertex_id": vertex.next,
                "birth": _event_time(vertex.birth),
                "seed_or_birth_point": _event_point(vertex.point),
                "position_at_terminal_time": (
                    None if position is None else _event_point(position)
                ),
                "reflex": vertex.reflex,
                "sliding_initialized": vertex.sliding is not None,
                "prev_incident_span": list(
                    builder.edges[vertex.prev_edge].span
                ),
                "next_incident_span": list(
                    builder.edges[vertex.next_edge].span
                ),
                "prev_incident_support": _line_record(
                    builder.edges[vertex.prev_edge],
                    line_class,
                    builder.line_id.get(
                        builder.edges[vertex.prev_edge].line_key
                    ),
                ),
                "next_incident_support": _line_record(
                    builder.edges[vertex.next_edge],
                    line_class,
                    builder.line_id.get(
                        builder.edges[vertex.next_edge].line_key
                    ),
                ),
            }
        )
    counters = dict(skeleton.counters)
    return {
        "outcome": skeleton.outcome.value,
        "levels": skeleton.levels,
        "node_count": len(skeleton.nodes),
        "terminal_time": _event_time(builder.now),
        "queue_length": len(builder.queue),
        "queue_pushed": builder.queue.pushed,
        "queue_popped": builder.queue.popped,
        "live_vertex_count": len(live),
        "live_lav_cycles": _live_cycles(builder),
        "live_vertices": live,
        "counters": counters,
        "named_candidate_rejection_counters": {
            key: value
            for key, value in sorted(counters.items())
            if key.startswith("refused_")
        },
    }


def _polygon_record(polygon) -> dict[str, Any]:
    def loop_record(loop):
        return {
            "points": [list(point) for point in loop.points],
            "speeds_squared": (
                None
                if loop.speeds_squared is None
                else [_ratio(item) for item in loop.speeds_squared]
            ),
            "effective_edge_speeds_squared": [
                _ratio(item) for item in loop.edge_speeds_squared
            ],
        }

    return {
        "outer": loop_record(polygon.outer),
        "holes": [loop_record(loop) for loop in polygon.holes],
        "vertex_fans": [
            {
                "point": list(fan.point),
                "supports": [
                    {
                        "normal": [support.normal_x, support.normal_y],
                        "q": _ratio(support.speed_squared),
                    }
                    for support in fan.supports
                ],
            }
            for fan in polygon.vertex_fans
        ],
        "fan_edge_count": polygon.fan_edge_count,
        "vertex_count": polygon.vertex_count,
        "reflex_count": polygon.reflex_count,
    }


def _partition_record(partition) -> dict[str, Any]:
    return {
        "outcome": partition.outcome.value,
        "doubled_area": _sqrt_sum(partition.doubled_area),
        "polygon_doubled_area": partition.polygon_doubled_area,
        "area_defect": _sqrt_sum(partition.area_defect),
        "faces": [
            {
                "owner": list(face.owner),
                "source_start": list(face.source_start),
                "source_end": list(face.source_end),
                "doubled_area": _sqrt_sum(face.doubled_area),
                "points": [
                    {"x": _sqrt_sum(point[0]), "y": _sqrt_sum(point[1])}
                    for point in face.points
                ],
                "line": (
                    None
                    if face.line is None
                    else [
                        face.line.a,
                        face.line.b,
                        face.line.c,
                        _ratio(face.line.q),
                    ]
                ),
            }
            for face in partition.faces
        ],
    }


def _assembler_borders(region, coverage_region) -> dict[str, Any]:
    partition = region.partition
    coverage_sum = coverage_region.doubled_area - coverage_region.doubled_area
    for face in coverage_region.faces:
        coverage_sum = coverage_sum + face.doubled_area
    coverage_defect = coverage_sum - coverage_region.doubled_area
    owner_by_edge = [tuple(item[0]) for item in region.owner_by_edge]
    wall_spans = [tuple(item) for item in region.wall_spans]
    ambiguous = [tuple(item) for item in region.ambiguous_owner_spans]
    source_spans = owner_by_edge + wall_spans + ambiguous
    partition_owners = [tuple(face.owner) for face in partition.faces]
    coverage_owners = [tuple(face.owner) for face in coverage_region.faces]
    ownership_exact = (
        len(source_spans) == len(set(source_spans))
        and len(partition_owners) == len(set(partition_owners))
        and len(coverage_owners) == len(set(coverage_owners))
        and set(source_spans) == set(partition_owners)
        and set(partition_owners) == set(coverage_owners)
    )
    return {
        "repository_border_names": [
            "POLYGON_FACE_DOUBLED_AREA_EQUALITY",
            "PARTITION_COVERAGE_DEFECT_ZERO",
            "OWNERSHIP_SOURCE_SPAN_ACCOUNTING_EXACT",
        ],
        "POLYGON_FACE_DOUBLED_AREA_EQUALITY": {
            "polygon_doubled_area": partition.polygon_doubled_area,
            "face_doubled_area": _sqrt_sum(partition.doubled_area),
            "partition_area_defect": _sqrt_sum(partition.area_defect),
            "exact": partition.area_reproduces_polygon,
        },
        "every_contour_is_simple": partition.every_contour_is_simple,
        "every_face_is_positive": partition.every_face_is_positive,
        "PARTITION_COVERAGE_DEFECT_ZERO": {
            "partition_area_defect": _sqrt_sum(partition.area_defect),
            "coverage_sum_defect": _sqrt_sum(coverage_defect),
            "coverage_polygon_doubled_area": (
                coverage_region.polygon_doubled_area
            ),
            "coverage_doubled_area": _sqrt_sum(
                coverage_region.doubled_area
            ),
            "exact": (
                partition.area_defect.is_zero
                and coverage_defect.is_zero
                and coverage_region.polygon_doubled_area
                == partition.polygon_doubled_area
            ),
        },
        "OWNERSHIP_SOURCE_SPAN_ACCOUNTING_EXACT": {
            "owner_by_edge_count": len(owner_by_edge),
            "wall_span_count": len(wall_spans),
            "ambiguous_owner_span_count": len(ambiguous),
            "partition_owner_count": len(partition_owners),
            "coverage_owner_count": len(coverage_owners),
            "source_spans": [list(item) for item in sorted(source_spans)],
            "partition_owners": [
                list(item) for item in sorted(partition_owners)
            ],
            "coverage_owners": [
                list(item) for item in sorted(coverage_owners)
            ],
            "exact": ownership_exact,
        },
        "leaf_evidence": {
            "owner_by_edge": [
                {
                    "span": list(span),
                    "envelope_spec_id": envelope_spec_id,
                }
                for span, envelope_spec_id in region.owner_by_edge
            ],
            "wall_spans": [list(item) for item in region.wall_spans],
            "ambiguous_owner_spans": [
                list(item) for item in region.ambiguous_owner_spans
            ],
            "coverage_faces": [
                {
                    "owner": list(face.owner),
                    "doubled_area": _sqrt_sum(face.doubled_area),
                }
                for face in coverage_region.faces
            ],
            "coverage_doubled_area": _sqrt_sum(
                coverage_region.doubled_area
            ),
            "coverage_polygon_doubled_area": (
                coverage_region.polygon_doubled_area
            ),
        },
        "all_exact": (
            partition.area_reproduces_polygon
            and partition.every_contour_is_simple
            and partition.every_face_is_positive
            and partition.area_defect.is_zero
            and coverage_defect.is_zero
            and coverage_region.polygon_doubled_area
            == partition.polygon_doubled_area
            and ownership_exact
        ),
    }


def _fraction_from_ratio(value: str | int) -> Fraction:
    if isinstance(value, int):
        return Fraction(value)
    return Fraction(value)


def _sqrt_sum_from_record(
    record: list[dict[str, Any]], modules: dict[str, Any]
):
    value = modules["SqrtSumV1"].zero()
    for term in record:
        value = value + modules["SqrtSumV1"].radical(
            _fraction_from_ratio(term["coefficient"]),
            term["radicand"],
        )
    return value


def _point_from_record(
    record: dict[str, Any], modules: dict[str, Any]
):
    return (
        _sqrt_sum_from_record(record["x"], modules),
        _sqrt_sum_from_record(record["y"], modules),
    )


def _recompute_adapter_record(record: dict[str, Any]) -> dict[str, Any]:
    before = record["before"]
    after = record["after"]
    before_by_id = {item["vertex_id"]: item for item in before}
    after_by_id = {item["vertex_id"]: item for item in after}
    before_ids = sorted(before_by_id)
    after_ids = sorted(after_by_id)
    invariant_fields = (
        "vertex_id",
        "birth",
        "point",
        "prev_edge_id",
        "next_edge_id",
        "prev_span",
        "next_span",
        "prev_raw_line_key",
        "next_raw_line_key",
        "prev_support_line_class",
        "next_support_line_class",
        "prev_q_over_N",
        "next_q_over_N",
        "determinant",
        "dot",
        "q1_N2",
        "q2_N1",
        "exact_classifier_result",
        "frozen_projection",
        "prev_vertex_id",
        "next_vertex_id",
        "reflex",
    )
    field_mismatches = {
        field: 0 for field in invariant_fields
    }
    projection_failures = 0
    recomputed_classified = []
    for ident in sorted(set(before_by_id) | set(after_by_id)):
        left = before_by_id.get(ident)
        right = after_by_id.get(ident)
        if left is None or right is None:
            for field in invariant_fields:
                field_mismatches[field] += 1
            continue
        for field in invariant_fields:
            field_mismatches[field] += int(
                left[field] != right[field]
            )
        if left["exact_classifier_result"]:
            recomputed_classified.append(ident)
            projection_failures += int(left["sliding"] is not None)
            projection_failures += int(
                right["sliding"] != left["frozen_projection"]
            )
        else:
            projection_failures += int(
                right["sliding"] != left["sliding"]
            )
    before_immutable = _state_without_sliding(before)
    after_immutable = _state_without_sliding(after)
    recomputed_immutable = (
        before_ids == after_ids
        and not any(field_mismatches.values())
    )
    summary_inconsistencies = sum(
        (
            record["seed_vertex_ids"] != before_ids,
            sorted(record["classified_vertex_ids"])
            != sorted(recomputed_classified),
            record["classified_count"] != len(recomputed_classified),
            record["immutable_seed_state_before_sha256"]
            != _sha256(before_immutable),
            record["immutable_seed_state_after_sha256"]
            != _sha256(after_immutable),
            record["immutable_seed_state_unchanged"]
            != recomputed_immutable,
        )
    )
    return {
        "before_after_vertex_id_sets_equal": before_ids == after_ids,
        "recomputed_classified_vertex_ids": recomputed_classified,
        "field_mismatch_counts": field_mismatches,
        "projection_failures": projection_failures,
        "summary_inconsistencies": summary_inconsistencies,
        "recomputed_immutable_seed_state_unchanged": (
            recomputed_immutable
        ),
        "passed": (
            recomputed_immutable
            and projection_failures == 0
            and summary_inconsistencies == 0
            and record["seed_scope_excess_count"] == 0
        ),
    }


def _recompute_immutable_input(
    immutable: dict[str, Any],
) -> dict[str, Any]:
    tracked_seals = immutable["tracked_input_byte_seals"]
    snapshot_before = immutable[
        "analysis_snapshot_input_utf8"
    ].encode("utf-8")
    snapshot_after = immutable[
        "analysis_snapshot_after_utf8"
    ].encode("utf-8")
    request_before = immutable["decal_request_input_utf8"].encode(
        "utf-8"
    )
    request_after = immutable["decal_request_after_utf8"].encode(
        "utf-8"
    )
    binding_before = immutable["v2_binding_before_utf8"].encode("utf-8")
    binding_after = immutable["v2_binding_after_utf8"].encode("utf-8")
    binding_facts_before = _binding_fact_record(binding_before)
    binding_facts_after = _binding_fact_record(binding_after)
    polygon_before = _pretty_json(immutable["bridge_polygon_before"])
    polygon_after = _pretty_json(immutable["bridge_polygon_after"])
    leaf_equalities = {
        "analysis_snapshot_bytes_equal": (
            snapshot_before == snapshot_after
        ),
        "decal_request_bytes_equal": request_before == request_after,
        "tracked_snapshot_matches_canonical_leaf": (
            _tracked_utf8_seal_exact(
                tracked_seals["analysis_snapshot.json"],
                snapshot_before,
            )
        ),
        "tracked_request_matches_canonical_leaf": (
            _tracked_utf8_seal_exact(
                tracked_seals["decal_request.json"],
                request_before,
            )
        ),
        "v2_binding_bytes_equal": binding_before == binding_after,
        "v2_binding_nodes_rationals_equal": (
            binding_facts_before == binding_facts_after
        ),
        "bridge_polygon_equal": polygon_before == polygon_after,
        "lattice_S_prime_equal": (
            immutable["lattice_S_prime_before"]
            == immutable["lattice_S_prime_after"]
            == immutable["prepared_lattice_S_prime"]
            == binding_facts_before["lattice_scale_S_prime"]
            == binding_facts_after["lattice_scale_S_prime"]
        ),
    }
    summary_inconsistencies = sum(
        (
            immutable["analysis_snapshot_input_sha256"]
            != _sha256(snapshot_before),
            immutable["analysis_snapshot_after_sha256"]
            != _sha256(snapshot_after),
            immutable["analysis_snapshot_byte_identical"]
            != leaf_equalities["analysis_snapshot_bytes_equal"],
            immutable["decal_request_input_sha256"]
            != _sha256(request_before),
            immutable["decal_request_after_sha256"]
            != _sha256(request_after),
            immutable["decal_request_byte_identical"]
            != leaf_equalities["decal_request_bytes_equal"],
            immutable["v2_binding_before"] != binding_facts_before,
            immutable["v2_binding_after"] != binding_facts_after,
            immutable["v2_binding_byte_identical"]
            != leaf_equalities["v2_binding_bytes_equal"],
            immutable["bridge_polygon_byte_identical"]
            != leaf_equalities["bridge_polygon_equal"],
            immutable["lattice_S_prime_unchanged"]
            != leaf_equalities["lattice_S_prime_equal"],
            immutable["all_exact"] != all(leaf_equalities.values()),
        )
    )
    return {
        "leaf_equalities": leaf_equalities,
        "binding_facts_before": binding_facts_before,
        "binding_facts_after": binding_facts_after,
        "summary_inconsistencies": summary_inconsistencies,
        "passed": (
            all(leaf_equalities.values())
            and summary_inconsistencies == 0
            and binding_facts_before["binding_type"]
            == "ChainStraightEvaluationGeometryBindingV2"
        ),
    }


def _recompute_mode_agreement(
    fixture: dict[str, Any],
) -> dict[str, Any]:
    canonical_skeleton = fixture["region"]["skeleton_semantics"]
    exhaustive = fixture["region"]["exhaustive"]
    exhaustive_skeleton = exhaustive["skeleton_semantics"]
    canonical_partition = fixture["region"]["partition"]
    exhaustive_partition = exhaustive["partition"]
    recomputed = {
        "outcome_equal": (
            canonical_skeleton["outcome"]
            == exhaustive_skeleton["outcome"]
        ),
        "levels_equal": (
            canonical_skeleton["levels"]
            == exhaustive_skeleton["levels"]
        ),
        "nodes_byte_identical": (
            _pretty_json(canonical_skeleton["nodes"])
            == _pretty_json(exhaustive_skeleton["nodes"])
        ),
        "partition_byte_identical": (
            _pretty_json(canonical_partition)
            == _pretty_json(exhaustive_partition)
        ),
    }
    recomputed["all_exact_and_agree"] = (
        canonical_skeleton["outcome"] == "EXACT"
        and exhaustive_skeleton["outcome"] == "EXACT"
        and canonical_partition["outcome"] == "EXACT"
        and exhaustive_partition["outcome"] == "EXACT"
        and recomputed["outcome_equal"]
        and recomputed["nodes_byte_identical"]
        and recomputed["partition_byte_identical"]
    )
    stored = fixture["public_split_search_contract"]["mode_agreement"]
    return {
        "recomputed": recomputed,
        "summary_inconsistencies": sum(
            stored[key] != value for key, value in recomputed.items()
        ),
        "passed": (
            recomputed["all_exact_and_agree"]
            and all(stored[key] == value for key, value in recomputed.items())
        ),
    }


def _recompute_assembler_borders(
    fixture: dict[str, Any],
    modules: dict[str, Any],
) -> dict[str, Any]:
    partition = fixture["region"]["partition"]
    claims = fixture["region"]["assembler_borders"]
    leaves = claims["leaf_evidence"]
    zero = modules["SqrtSumV1"].zero()
    face_total = zero
    face_area_mutations = 0
    simple_contours = True
    positive_faces = True
    partition_owners = []
    for face in partition["faces"]:
        points = tuple(
            _point_from_record(point, modules)
            for point in face["points"]
        )
        recomputed_area = modules["doubled_shoelace"](points)
        stored_area = _sqrt_sum_from_record(
            face["doubled_area"], modules
        )
        face_area_mutations += int(recomputed_area != stored_area)
        face_total = face_total + recomputed_area
        simple_contours = (
            simple_contours
            and not modules["contour_crossings"](points)
        )
        positive_faces = positive_faces and recomputed_area.sign() > 0
        partition_owners.append(tuple(face["owner"]))
    stored_partition_area = _sqrt_sum_from_record(
        partition["doubled_area"], modules
    )
    polygon_area = modules["SqrtSumV1"].rational(
        partition["polygon_doubled_area"]
    )
    recomputed_partition_defect = face_total - polygon_area
    stored_partition_defect = _sqrt_sum_from_record(
        partition["area_defect"], modules
    )
    partition_area_exact = (
        face_area_mutations == 0
        and face_total == stored_partition_area
        and recomputed_partition_defect.is_zero
        and stored_partition_defect == recomputed_partition_defect
    )

    coverage_total = zero
    coverage_owners = []
    for face in leaves["coverage_faces"]:
        coverage_total = coverage_total + _sqrt_sum_from_record(
            face["doubled_area"], modules
        )
        coverage_owners.append(tuple(face["owner"]))
    stored_coverage_area = _sqrt_sum_from_record(
        leaves["coverage_doubled_area"], modules
    )
    coverage_defect_zero = coverage_total == stored_coverage_area
    owner_by_edge = [
        tuple(item["span"]) for item in leaves["owner_by_edge"]
    ]
    wall_spans = [tuple(item) for item in leaves["wall_spans"]]
    ambiguous = [
        tuple(item) for item in leaves["ambiguous_owner_spans"]
    ]
    source_spans = owner_by_edge + wall_spans + ambiguous
    ownership_exact = (
        len(source_spans) == len(set(source_spans))
        and len(partition_owners) == len(set(partition_owners))
        and len(coverage_owners) == len(set(coverage_owners))
        and set(source_spans) == set(partition_owners)
        and set(partition_owners) == set(coverage_owners)
    )
    border_values = {
        "POLYGON_FACE_DOUBLED_AREA_EQUALITY": partition_area_exact,
        "PARTITION_COVERAGE_DEFECT_ZERO": (
            recomputed_partition_defect.is_zero
            and coverage_defect_zero
            and leaves["coverage_polygon_doubled_area"]
            == partition["polygon_doubled_area"]
        ),
        "OWNERSHIP_SOURCE_SPAN_ACCOUNTING_EXACT": ownership_exact,
    }
    recomputed_all = (
        all(border_values.values())
        and simple_contours
        and positive_faces
    )
    summary_inconsistencies = sum(
        (
            claims["every_contour_is_simple"] != simple_contours,
            claims["every_face_is_positive"] != positive_faces,
            claims["all_exact"] != recomputed_all,
            claims["POLYGON_FACE_DOUBLED_AREA_EQUALITY"]["exact"]
            != border_values["POLYGON_FACE_DOUBLED_AREA_EQUALITY"],
            claims["PARTITION_COVERAGE_DEFECT_ZERO"]["exact"]
            != border_values["PARTITION_COVERAGE_DEFECT_ZERO"],
            claims["OWNERSHIP_SOURCE_SPAN_ACCOUNTING_EXACT"]["exact"]
            != border_values[
                "OWNERSHIP_SOURCE_SPAN_ACCOUNTING_EXACT"
            ],
        )
    )
    return {
        "border_values": border_values,
        "simple_contours": simple_contours,
        "positive_faces": positive_faces,
        "face_area_mutations": face_area_mutations,
        "ownership_exact": ownership_exact,
        "summary_inconsistencies": summary_inconsistencies,
        "passed": recomputed_all and summary_inconsistencies == 0,
    }


def _recompute_synthetic_assembler_borders(
    result: dict[str, Any],
    modules: dict[str, Any],
) -> dict[str, Any]:
    partition = result["partition"]
    claims = result["assembler_borders"]
    zero = modules["SqrtSumV1"].zero()
    face_total = zero
    face_area_mutations = 0
    simple_contours = True
    positive_faces = True
    for face in partition["faces"]:
        points = tuple(
            _point_from_record(point, modules)
            for point in face["points"]
        )
        recomputed_area = modules["doubled_shoelace"](points)
        stored_area = _sqrt_sum_from_record(
            face["doubled_area"], modules
        )
        face_area_mutations += int(recomputed_area != stored_area)
        face_total = face_total + recomputed_area
        simple_contours = (
            simple_contours
            and not modules["contour_crossings"](points)
        )
        positive_faces = positive_faces and recomputed_area.sign() > 0
    stored_partition_area = _sqrt_sum_from_record(
        partition["doubled_area"], modules
    )
    polygon_area = modules["SqrtSumV1"].rational(
        partition["polygon_doubled_area"]
    )
    recomputed_defect = face_total - polygon_area
    stored_partition_defect = _sqrt_sum_from_record(
        partition["area_defect"], modules
    )
    stored_claim_defect = _sqrt_sum_from_record(
        claims["area_defect"], modules
    )
    area_exact = (
        face_area_mutations == 0
        and face_total == stored_partition_area
        and recomputed_defect.is_zero
        and stored_partition_defect == recomputed_defect
        and stored_claim_defect == recomputed_defect
    )
    summary_inconsistencies = sum(
        (
            claims["every_contour_is_simple"] != simple_contours,
            claims["every_face_is_positive"] != positive_faces,
            claims["area_reproduces_polygon"] != area_exact,
            stored_claim_defect != recomputed_defect,
        )
    )
    return {
        "area_reproduces_polygon": area_exact,
        "simple_contours": simple_contours,
        "positive_faces": positive_faces,
        "face_area_mutations": face_area_mutations,
        "summary_inconsistencies": summary_inconsistencies,
        "passed": (
            area_exact
            and simple_contours
            and positive_faces
            and summary_inconsistencies == 0
        ),
    }


def _counter_delta(
    current: dict[str, int], historical: dict[str, int]
) -> dict[str, int]:
    return {
        key: current.get(key, 0) - historical.get(key, 0)
        for key in sorted(set(current) | set(historical))
    }


def _historical_summary(manifest: dict[str, Any]) -> dict[str, Any]:
    registry = manifest["expected_kernel_results_by_kernel_src_tree"]
    record = registry[HISTORICAL_KERNEL_SRC_TREE_OID]
    prepare = record["prepare"]
    region = prepare["regions"][0]
    skeleton = region["skeleton"]
    return {
        "evidence_source": "FROZEN_CASE_MANIFEST",
        "kernel_src_tree_oid": HISTORICAL_KERNEL_SRC_TREE_OID,
        "trace_sha256": record["trace_sha256"],
        "compile_outcome": record["compile"]["outcome"],
        "prepare_outcome": prepare["outcome"],
        "coverage_obligation": "PREPARE_EXACT_IMPLIES_PUBLIC_COVERAGE_REACHABLE",
        "lattice_scale": prepare["lattice_scale"],
        "prepare_counters": dict(prepare["counters"]),
        "region_id": region["region_id"],
        "bridge_outcome": region["bridge"]["outcome"],
        "skeleton_outcome": skeleton["outcome"],
        "skeleton_levels": skeleton["levels"],
        "skeleton_node_count": skeleton["node_count"],
        "skeleton_counters": dict(skeleton["counters"]),
        "face_outcome": region["faces"]["outcome"],
        "face_count": region["faces"]["face_count"],
    }


def _fixture_integrity(
    source_root: Path,
    fixture_dir: Path,
    accepted_fixture_hashes: dict[str, str],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    failures = []
    manifest_path = fixture_dir / "manifest.json"
    relative_manifest = manifest_path.relative_to(source_root).as_posix()
    manifest_bytes, manifest_seal = _tracked_utf8_seal(
        source_root,
        manifest_path,
        f"HEAD:{relative_manifest}",
        "VALIDATED_HEAD_TRACKED_FIXTURE_INPUT",
    )
    manifest = json.loads(manifest_bytes)
    fixture_id = fixture_dir.name
    if not _tracked_utf8_seal_exact(manifest_seal):
        failures.append(
            {
                "code": "FIXTURE_TRACKED_BYTE_SEAL_MISMATCH",
                "fixture_id": fixture_id,
                "file": "manifest.json",
            }
        )
    if manifest["fixture_id"] != fixture_id:
        failures.append(
            {
                "code": "UNKNOWN_FIXTURE_FORM",
                "fixture_id": fixture_id,
                "detail": "manifest fixture_id differs from directory name",
            }
        )
    if manifest["fixture_hash"] != accepted_fixture_hashes.get(fixture_id):
        failures.append(
            {
                "code": "UNKNOWN_ACCEPTED_FIXTURE_HASH",
                "fixture_id": fixture_id,
                "detail": manifest["fixture_hash"],
            }
        )
    observed_files = {}
    tracked_file_seals = {
        "manifest.json": _compact_tracked_utf8_seal(manifest_seal)
    }
    for name, expected in sorted(manifest["files"].items()):
        path = fixture_dir / name
        relative_path = path.relative_to(source_root).as_posix()
        canonical_bytes, seal = _tracked_utf8_seal(
            source_root,
            path,
            f"HEAD:{relative_path}",
            "VALIDATED_HEAD_TRACKED_FIXTURE_INPUT",
        )
        observed = _sha256(canonical_bytes)
        observed_files[name] = observed
        tracked_file_seals[name] = _compact_tracked_utf8_seal(seal)
        if not _tracked_utf8_seal_exact(seal):
            failures.append(
                {
                    "code": "FIXTURE_TRACKED_BYTE_SEAL_MISMATCH",
                    "fixture_id": fixture_id,
                    "file": name,
                }
            )
        if observed != expected:
            failures.append(
                {
                    "code": "FIXTURE_FILE_HASH_MISMATCH",
                    "fixture_id": fixture_id,
                    "file": name,
                    "expected": expected,
                    "observed": observed,
                }
            )
    return {
        "fixture_id": fixture_id,
        "fixture_hash": manifest["fixture_hash"],
        "manifest_file_sha256": observed_files,
        "tracked_utf8_seals": tracked_file_seals,
    }, failures


def _public_fixture_record_phase1(
    modules: dict[str, Any],
    source_root: Path,
    fixture_dir: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Public canonical run plus proof-only exhaustive agreement witness."""

    kernel = modules["kernel"]
    conveyor = modules["conveyor"]
    skeleton_module = modules["skeleton"]
    line_class = modules["line_class"]
    snapshot_path = fixture_dir / "analysis_snapshot.json"
    request_path = fixture_dir / "decal_request.json"
    snapshot_relative = snapshot_path.relative_to(source_root).as_posix()
    request_relative = request_path.relative_to(source_root).as_posix()
    snapshot_input, snapshot_input_seal = _tracked_utf8_seal(
        source_root,
        snapshot_path,
        f"HEAD:{snapshot_relative}",
        "VALIDATED_HEAD_TRACKED_FIXTURE_INPUT",
    )
    request_input, request_input_seal = _tracked_utf8_seal(
        source_root,
        request_path,
        f"HEAD:{request_relative}",
        "VALIDATED_HEAD_TRACKED_FIXTURE_INPUT",
    )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(snapshot_input)
    request = kernel.DecalRequestCodecV1.loads(request_input)
    (domain,) = snapshot.patch_domains

    compile_started = time.perf_counter()
    compile_result = kernel.compile_reference_envelopes(
        snapshot, request, domain.patch_domain_id
    )
    compile_seconds = time.perf_counter() - compile_started
    if compile_result.compilation is None:
        raise ValueError("public compile did not return a compilation")
    binding = compile_result.compilation.evaluation_geometry_binding
    binding_codec = _binding_codec(kernel, binding)
    binding_before = binding_codec.dumps(binding)
    binding_facts_before = _binding_fact_record(binding_before)

    captures: list[dict[str, Any]] = []
    original_build_skeleton = conveyor.build_skeleton
    original_new_vertex = skeleton_module._Builder._new_vertex

    def capturing_build_skeleton(polygon):
        builder = skeleton_module._Builder(
            polygon, skeleton_module.SplitSearch.MOTORCYCLE
        )
        seed = _seed_snapshot(builder, line_class)
        result = builder.run()
        captures.append(
            {
                "polygon": _polygon_record(polygon),
                "seed": seed,
                "terminal": _terminal_snapshot(
                    builder, result, line_class
                ),
                "skeleton_semantics": _skeleton_semantic_record(result),
                "result": result,
            }
        )
        return result

    adapter = _SeedClassifierAdapter(skeleton_module, line_class)
    prepare_started = time.perf_counter()
    conveyor.build_skeleton = capturing_build_skeleton
    try:
        with adapter:
            prepared = conveyor.prepare_conveyor(
                snapshot, request, patch_domain_id=domain.patch_domain_id
            )
            coverage = conveyor.conveyor_coverage(prepared)
            if len(captures) != 1 or len(prepared.regions) != 1:
                raise ValueError(
                    "expected exactly one captured sparse domain region"
                )
            canonical_capture = captures[0]
            region = prepared.regions[0]
            if region.bridge.polygon is None:
                raise ValueError("public bridge did not produce a polygon")
            exhaustive_builder = skeleton_module._Builder(
                region.bridge.polygon,
                skeleton_module.SplitSearch.EXHAUSTIVE,
            )
            exhaustive_seed = _seed_snapshot(
                exhaustive_builder, line_class
            )
            exhaustive_skeleton = exhaustive_builder.run()
            exhaustive_partition = modules["build_faces"](
                region.bridge.polygon, exhaustive_skeleton
            )
            exhaustive = {
                "seed": exhaustive_seed,
                "terminal": _terminal_snapshot(
                    exhaustive_builder,
                    exhaustive_skeleton,
                    line_class,
                ),
                "skeleton_semantics": _skeleton_semantic_record(
                    exhaustive_skeleton
                ),
                "partition": _partition_record(exhaustive_partition),
            }
    finally:
        conveyor.build_skeleton = original_build_skeleton
    prepare_seconds = time.perf_counter() - prepare_started

    if not adapter.restored:
        raise ValueError("proof-local seed adapter was not restored")
    if skeleton_module._Builder._new_vertex is not original_new_vertex:
        raise ValueError("event-born _new_vertex callable changed")
    if region.skeleton != canonical_capture["result"]:
        raise ValueError("capture adapter changed the public skeleton result")
    polygon_after = _polygon_record(region.bridge.polygon)
    if polygon_after != canonical_capture["polygon"]:
        raise ValueError("captured polygon differs from public bridge polygon")
    if len(coverage.regions) != 1:
        raise ValueError("expected one public coverage region")

    binding_after = binding_codec.dumps(
        compile_result.compilation.evaluation_geometry_binding
    )
    binding_facts_after = _binding_fact_record(binding_after)
    snapshot_after = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    request_after = kernel.DecalRequestCodecV1.dumps(request)
    canonical_partition = _partition_record(region.partition)
    canonical_skeleton = canonical_capture["skeleton_semantics"]
    exhaustive_skeleton_record = exhaustive["skeleton_semantics"]
    mode_agreement = {
        "outcome_equal": (
            canonical_skeleton["outcome"]
            == exhaustive_skeleton_record["outcome"]
        ),
        "levels_equal": (
            canonical_skeleton["levels"]
            == exhaustive_skeleton_record["levels"]
        ),
        "nodes_byte_identical": (
            _pretty_json(canonical_skeleton["nodes"])
            == _pretty_json(exhaustive_skeleton_record["nodes"])
        ),
        "partition_byte_identical": (
            _pretty_json(canonical_partition)
            == _pretty_json(exhaustive["partition"])
        ),
    }
    mode_agreement["all_exact_and_agree"] = (
        canonical_skeleton["outcome"] == "EXACT"
        and exhaustive_skeleton_record["outcome"] == "EXACT"
        and region.partition.outcome.value == "EXACT"
        and exhaustive_partition.outcome.value == "EXACT"
        and mode_agreement["outcome_equal"]
        and mode_agreement["nodes_byte_identical"]
        and mode_agreement["partition_byte_identical"]
    )
    adapter_records = [
        {
            key: value
            for key, value in record.items()
            if key != "builder_identity"
        }
        for record in adapter.records
    ]
    assembler = _assembler_borders(region, coverage.regions[0])
    immutable_input = {
        "tracked_input_byte_seals": {
            "analysis_snapshot.json": _compact_tracked_utf8_seal(
                snapshot_input_seal
            ),
            "decal_request.json": _compact_tracked_utf8_seal(
                request_input_seal
            ),
        },
        "analysis_snapshot_input_utf8": snapshot_input.decode("utf-8"),
        "analysis_snapshot_after_utf8": snapshot_after.decode("utf-8"),
        "analysis_snapshot_input_sha256": _sha256(snapshot_input),
        "analysis_snapshot_after_sha256": _sha256(snapshot_after),
        "analysis_snapshot_byte_identical": (
            snapshot_input == snapshot_after
        ),
        "decal_request_input_utf8": request_input.decode("utf-8"),
        "decal_request_after_utf8": request_after.decode("utf-8"),
        "decal_request_input_sha256": _sha256(request_input),
        "decal_request_after_sha256": _sha256(request_after),
        "decal_request_byte_identical": request_input == request_after,
        "v2_binding_before_utf8": binding_before.decode("utf-8"),
        "v2_binding_after_utf8": binding_after.decode("utf-8"),
        "v2_binding_before": binding_facts_before,
        "v2_binding_after": binding_facts_after,
        "v2_binding_byte_identical": binding_before == binding_after,
        "bridge_polygon_before": canonical_capture["polygon"],
        "bridge_polygon_after": polygon_after,
        "bridge_polygon_byte_identical": (
            _pretty_json(canonical_capture["polygon"])
            == _pretty_json(polygon_after)
        ),
        "lattice_S_prime_before": (
            binding_facts_before["lattice_scale_S_prime"]
        ),
        "lattice_S_prime_after": (
            binding_facts_after["lattice_scale_S_prime"]
        ),
        "prepared_lattice_S_prime": prepared.lattice.scale,
        "lattice_S_prime_unchanged": (
            binding_facts_before["lattice_scale_S_prime"]
            == binding_facts_after["lattice_scale_S_prime"]
            == prepared.lattice.scale
        ),
    }
    immutable_input["all_exact"] = all(
        (
            immutable_input["analysis_snapshot_byte_identical"],
            immutable_input["decal_request_byte_identical"],
            _tracked_utf8_seal_exact(snapshot_input_seal),
            _tracked_utf8_seal_exact(request_input_seal),
            immutable_input["v2_binding_byte_identical"],
            immutable_input["bridge_polygon_byte_identical"],
            immutable_input["lattice_S_prime_unchanged"],
            binding_facts_before["binding_type"]
            == "ChainStraightEvaluationGeometryBindingV2",
            binding_facts_before == binding_facts_after,
        )
    )

    current = {
        "fixture_id": fixture_dir.name,
        "patch_domain_id": domain.patch_domain_id.value,
        "compile": {
            "outcome": compile_result.outcome.value,
            "binding_type": type(binding).__name__,
            "domain_lattice_scale_S_prime": binding.lattice_scale,
        },
        "prepare": {
            "outcome": prepared.outcome.value,
            "lattice_scale": prepared.lattice.scale,
            "counters": dict(prepared.counters),
        },
        "coverage": {
            "outcome": coverage.outcome.value,
            "region_outcome": coverage.regions[0].outcome.value,
        },
        "public_split_search_contract": {
            "canonical_mode": "MOTORCYCLE",
            "public_request_exposes_split_search": False,
            "reason": (
                "prepare_conveyor_has_no_split_search_parameter;_"
                "MOTORCYCLE_is_the_product_default"
            ),
            "proof_only_secondary_mode": "EXHAUSTIVE",
            "agreement_law": (
                "EXACT_OUTCOME_PLUS_BYTE_IDENTICAL_NODES_AND_PARTITION;_"
                "LEVEL_AND_SEARCH_COUNTER_TRACES_ARE_FROZEN_PER_MODE"
            ),
            "mode_agreement": mode_agreement,
        },
        "immutable_input": immutable_input,
        "adapter": {
            "scope": "SEED_VERTICES_CREATED_BY__seed_one_loop_ONLY",
            "normal_exit_restored": adapter.restored,
            "seed_hook_before": adapter.original_seed_identity,
            "seed_hook_after": adapter.restored_seed_identity,
            "event_born_hook_before": (
                adapter.original_new_vertex_identity
            ),
            "event_born_hook_after": (
                adapter.restored_new_vertex_identity
            ),
            "event_born_new_vertex_callable_unchanged": (
                skeleton_module._Builder._new_vertex
                is original_new_vertex
            ),
            "records": adapter_records,
        },
        "region": {
            "region_id": region.region_id,
            "bridge_outcome": region.bridge_outcome.value,
            "bridge_polygon": canonical_capture["polygon"],
            "seed": canonical_capture["seed"],
            "terminal": canonical_capture["terminal"],
            "skeleton_semantics": canonical_skeleton,
            "partition": canonical_partition,
            "assembler_borders": assembler,
            "exhaustive": exhaustive,
        },
    }
    timing = {
        "fixture_id": fixture_dir.name,
        "compile_seconds": compile_seconds,
        "prepare_and_secondary_mode_seconds": prepare_seconds,
        "prepare_reported_seconds": dict(prepared.timings),
    }
    return current, timing


def _synthetic_case(
    modules: dict[str, Any],
    name: str,
    points: tuple[tuple[int, int], ...],
) -> dict[str, Any]:
    skeleton_module = modules["skeleton"]
    line_class = modules["line_class"]
    polygon = modules["PolygonV1"](modules["LoopV1"](points))
    modes = {}
    for mode in skeleton_module.SplitSearch:
        builder = skeleton_module._Builder(polygon, mode)
        seed = _seed_snapshot(builder, line_class)
        skeleton = builder.run()
        partition = modules["build_faces"](polygon, skeleton)
        modes[mode.value] = {
            "seed": seed,
            "terminal": _terminal_snapshot(
                builder, skeleton, line_class
            ),
            "face_outcome": partition.outcome.value,
            "face_count": len(partition.faces),
            "doubled_face_area": _sqrt_sum(partition.doubled_area),
            "polygon_doubled_area": partition.polygon_doubled_area,
        }
    return {
        "case": name,
        "points": [list(point) for point in points],
        "signed_doubled_area": modules["signed_double_area"](points),
        "modes": modes,
    }


def _synthetic_case_phase1(
    modules: dict[str, Any],
    name: str,
    points: tuple[tuple[int, int], ...],
) -> dict[str, Any]:
    skeleton_module = modules["skeleton"]
    line_class = modules["line_class"]
    polygon = modules["PolygonV1"](modules["LoopV1"](points))
    polygon_before_record = _polygon_record(polygon)
    polygon_before = _pretty_json(polygon_before_record)
    modes = {}
    original_new_vertex = skeleton_module._Builder._new_vertex
    adapter = _SeedClassifierAdapter(skeleton_module, line_class)
    with adapter:
        for mode in skeleton_module.SplitSearch:
            builder = skeleton_module._Builder(polygon, mode)
            seed = _seed_snapshot(builder, line_class)
            skeleton = builder.run()
            partition = modules["build_faces"](polygon, skeleton)
            modes[mode.value] = {
                "seed": seed,
                "terminal": _terminal_snapshot(
                    builder, skeleton, line_class
                ),
                "skeleton_semantics": _skeleton_semantic_record(skeleton),
                "partition": _partition_record(partition),
                "face_outcome": partition.outcome.value,
                "face_count": len(partition.faces),
                "doubled_face_area": _sqrt_sum(partition.doubled_area),
                "polygon_doubled_area": partition.polygon_doubled_area,
                "assembler_borders": {
                    "every_contour_is_simple": (
                        partition.every_contour_is_simple
                    ),
                    "every_face_is_positive": (
                        partition.every_face_is_positive
                    ),
                    "area_reproduces_polygon": (
                        partition.area_reproduces_polygon
                    ),
                    "area_defect": _sqrt_sum(partition.area_defect),
                },
            }
    polygon_after_record = _polygon_record(polygon)
    polygon_after = _pretty_json(polygon_after_record)
    mode_values = list(modes.values())
    agreement = {
        "outcome_equal": (
            mode_values[0]["terminal"]["outcome"]
            == mode_values[1]["terminal"]["outcome"]
        ),
        "levels_equal": (
            mode_values[0]["terminal"]["levels"]
            == mode_values[1]["terminal"]["levels"]
        ),
        "nodes_byte_identical": (
            _pretty_json(mode_values[0]["skeleton_semantics"]["nodes"])
            == _pretty_json(mode_values[1]["skeleton_semantics"]["nodes"])
        ),
        "partition_byte_identical": (
            _pretty_json(mode_values[0]["partition"])
            == _pretty_json(mode_values[1]["partition"])
        ),
    }
    agreement["all_exact_and_agree"] = (
        all(
            mode["terminal"]["outcome"] == "EXACT"
            and mode["face_outcome"] == "EXACT"
            for mode in mode_values
        )
        and all(agreement.values())
    )
    return {
        "case": name,
        "points": [list(point) for point in points],
        "signed_doubled_area": modules["signed_double_area"](points),
        "polygon_before": polygon_before_record,
        "polygon_after": polygon_after_record,
        "polygon_byte_identical": polygon_before == polygon_after,
        "adapter_normal_exit_restored": adapter.restored,
        "seed_hook_before": adapter.original_seed_identity,
        "seed_hook_after": adapter.restored_seed_identity,
        "event_born_hook_before": (
            adapter.original_new_vertex_identity
        ),
        "event_born_hook_after": adapter.restored_new_vertex_identity,
        "event_born_new_vertex_callable_unchanged": (
            skeleton_module._Builder._new_vertex is original_new_vertex
        ),
        "adapter_records": [
            {
                key: value
                for key, value in record.items()
                if key != "builder_identity"
            }
            for record in adapter.records
        ],
        "mode_agreement": agreement,
        "modes": modes,
    }


def _different_speed_local_control(modules: dict[str, Any]) -> dict[str, Any]:
    skeleton_module = modules["skeleton"]
    first = skeleton_module.SupportLineV1(0, 1, 0, 1)
    second = skeleton_module.SupportLineV1(0, 2, 0, 8)
    determinant = first.a * second.b - second.a * first.b
    dot = first.a * second.a + first.b * second.b
    speed_left = first.q * second.normal_squared
    speed_right = second.q * first.normal_squared
    outcome = skeleton_module._joint_kind(first, second)
    return {
        "control": "CODIRECTIONAL_SAME_SUPPORT_DIFFERENT_SPEED",
        "determinant": determinant,
        "dot": dot,
        "q1_N2": _ratio(speed_left),
        "q2_N1": _ratio(speed_right),
        "normalized_speeds_unequal": speed_left != speed_right,
        "classifier_rejects": not _exact_straight_classifier(first, second),
        "named_local_outcome": (
            None if outcome is None else outcome.value
        ),
        "expected_named_local_outcome": (
            "NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS"
        ),
        "passed": (
            determinant == 0
            and dot > 0
            and speed_left != speed_right
            and not _exact_straight_classifier(first, second)
            and outcome is not None
            and outcome.value
            == "NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS"
        ),
    }


def _four_vertex_family(modules: dict[str, Any]) -> dict[str, Any]:
    skeleton_module = modules["skeleton"]
    total = 0
    failures = []
    for a in range(1, 7):
        for b in range(a + 1, 8):
            for x in range(-2, 11):
                for y in range(1, 5):
                    total += 1
                    points = ((0, 0), (a, 0), (b, 0), (x, y))
                    polygon = modules["PolygonV1"](
                        modules["LoopV1"](points)
                    )
                    outcomes = {
                        mode.value: skeleton_module.build_skeleton(
                            polygon, split_search=mode
                        ).outcome.value
                        for mode in skeleton_module.SplitSearch
                    }
                    if set(outcomes.values()) != {"EXACT"}:
                        failures.append(
                            {"points": points, "outcomes": outcomes}
                        )
    return {
        "scope": (
            "POLYGONS_(0,0),(a,0),(b,0),(x,y)_WITH_"
            "1_LE_A_LT_B_LE_7,_MINUS2_LE_X_LE_10,_1_LE_Y_LE_4"
        ),
        "minimality_claim": (
            "TESTED_FAMILY_ONLY_NOT_A_UNIVERSAL_MINIMALITY_PROOF"
        ),
        "polygon_count": total,
        "split_modes_per_polygon": [
            mode.value for mode in skeleton_module.SplitSearch
        ],
        "execution_count": total * len(skeleton_module.SplitSearch),
        "non_exact_count": len(failures),
        "first_non_exact_cases": failures[:10],
    }


def _source_predicate_audit(source_root: Path) -> dict[str, Any]:
    skeleton_path = (
        source_root
        / "kernel"
        / "src"
        / "cftuv_envelope"
        / "wavefront"
        / "skeleton.py"
    )
    verifier_path = (
        source_root
        / "kernel"
        / "fixtures"
        / "sem_clb_02_lost_domains_v1"
        / "verify_gate_b_prime.py"
    )
    skeleton_relative = skeleton_path.relative_to(source_root).as_posix()
    verifier_relative = verifier_path.relative_to(source_root).as_posix()
    skeleton_bytes, skeleton_seal = _tracked_utf8_seal(
        source_root,
        skeleton_path,
        f"HEAD:{skeleton_relative}",
        "VALIDATED_HEAD_PRODUCT_TREE_BLOB",
    )
    verifier_bytes, verifier_seal = _tracked_utf8_seal(
        source_root,
        verifier_path,
        f":{verifier_relative}",
        "STAGED_PROOF_VERIFIER_BLOB_FOR_COMMIT",
    )
    tree = ast.parse(skeleton_bytes.decode("utf-8"))
    functions = {
        node.name: node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
    }
    new_vertex = functions["_new_vertex"]
    new_vertex_if = next(
        node for node in new_vertex.body if isinstance(node, ast.If)
    )
    predicate = ast.unparse(new_vertex_if.test)
    seed = functions["_seed_one_loop"]
    seed_vertex_calls = [
        node
        for node in ast.walk(seed)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_Vertex"
    ]
    seed_sets_sliding = any(
        keyword.arg == "sliding"
        for call in seed_vertex_calls
        for keyword in call.keywords
    )
    event_sets_sliding = any(
        isinstance(node, (ast.Assign, ast.AnnAssign))
        and any(
            isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == "vertex"
            and target.attr == "sliding"
            for target in (
                node.targets if isinstance(node, ast.Assign) else [node.target]
            )
        )
        for node in ast.walk(new_vertex)
    )
    enqueue = functions["_enqueue_edge_event"]
    always_concurrent_index = next(
        (
            index
            for index, node in enumerate(enqueue.body)
            if isinstance(node, ast.If)
            and "outcome is not EventTimeOutcome.EXACT"
            in ast.unparse(node.test)
            and "NO_RULE_TRIPLE_ALWAYS_CONCURRENT"
            in ast.unparse(node)
        ),
        None,
    )
    collapsing_span_index = next(
        (
            index
            for index, node in enumerate(enqueue.body)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "span"
                for target in node.targets
            )
            and "_collapsing_span" in ast.unparse(node.value)
        ),
        None,
    )
    born_zero_filter_index = next(
        (
            index
            for index, node in enumerate(enqueue.body)
            if isinstance(node, ast.If)
            and "compare_times(time, vertex.birth) == 0"
            in ast.unparse(node.test)
            and "FILTER_SPAN_IS_BORN_ZERO" in ast.unparse(node)
        ),
        None,
    )
    observed_order = (
        always_concurrent_index is not None
        and collapsing_span_index is not None
        and born_zero_filter_index is not None
        and always_concurrent_index
        < collapsing_span_index
        < born_zero_filter_index
    )
    verifier_tree = ast.parse(verifier_bytes.decode("utf-8"))
    classifier = next(
        node
        for node in ast.walk(verifier_tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "_exact_straight_classifier"
    )
    classifier_return = next(
        node for node in classifier.body if isinstance(node, ast.Return)
    )
    classifier_predicate = ast.unparse(classifier_return.value)
    return {
        "event_born_new_vertex_predicate": predicate,
        "expected_predicate": EXPECTED_NEW_VERTEX_PREDICATE,
        "predicate_exact_match": predicate == EXPECTED_NEW_VERTEX_PREDICATE,
        "proof_local_seed_classifier_predicate": classifier_predicate,
        "proof_local_classifier_exact_match": (
            classifier_predicate == EXPECTED_NEW_VERTEX_PREDICATE
        ),
        "product_predicate_and_proof_classifier_identical": (
            classifier_predicate == predicate
            == EXPECTED_NEW_VERTEX_PREDICATE
        ),
        "tracked_source_byte_seals": {
            "product_skeleton.py": skeleton_seal,
            "phase1_verifier.py": verifier_seal,
        },
        "tracked_source_byte_domain_law": (
            "UTF8_GIT_BLOB_CANONICAL_LF_V1;_PRODUCT_FROM_VALIDATED_HEAD_"
            "TREE;_VERIFIER_FROM_STAGED_INDEX_BLOB;_WORKSPACE_MUST_MATCH_"
            "AFTER_CRLF_CR_TO_LF_NORMALIZATION;_RAW_CHECKOUT_BYTES_NEVER_"
            "AUTHORIZE_A_SEAL"
        ),
        "product_skeleton_source_sha256": skeleton_seal[
            "canonical_lf_sha256"
        ],
        "phase1_verifier_sha256": verifier_seal[
            "canonical_lf_sha256"
        ],
        "seed_constructor_count": len(seed_vertex_calls),
        "seed_constructor_sets_sliding": seed_sets_sliding,
        "event_born_constructor_sets_sliding": event_sets_sliding,
        "enqueue_edge_event_observed_order": {
            "always_concurrent_refusal_body_index": (
                always_concurrent_index
            ),
            "collapsing_span_body_index": collapsing_span_index,
            "born_zero_filter_body_index": born_zero_filter_index,
            "always_concurrent_precedes_span_and_filter": observed_order,
        },
        "first_exact_structural_difference": (
            "SEED_VERTEX_CONSTRUCTION_OMITS_EVENT_BORN_SLIDING_CLASSIFICATION"
            if (
                predicate == EXPECTED_NEW_VERTEX_PREDICATE
                and not seed_sets_sliding
                and event_sets_sliding
            )
            else "EVENT_BORN_AND_SEED_CLASSIFICATION_RELATION_UNKNOWN"
        ),
    }


def _cheap_checkout_byte_seal_audit(
    source_root: Path,
) -> dict[str, Any]:
    fixture_base = (
        source_root
        / "kernel"
        / "fixtures"
        / "sem_clb_02_lost_domains_v1"
    )
    accepted_payloads = {}
    accepted_records = {}
    accepted_exact = True
    for name, expected_sha256 in ACCEPTED_RECEIPT_SHA256.items():
        path = fixture_base / name
        relative = path.relative_to(source_root).as_posix()
        canonical, seal = _tracked_utf8_seal(
            source_root,
            path,
            f"HEAD:{relative}",
            "VALIDATED_HEAD_ACCEPTED_PROOF_BLOB",
        )
        accepted_payloads[name] = json.loads(canonical)
        exact = (
            _sha256(canonical) == expected_sha256
            and _tracked_utf8_seal_exact(seal)
        )
        accepted_exact = accepted_exact and exact
        accepted_records[name] = {
            "expected_sha256": expected_sha256,
            "canonical_lf_sha256": _sha256(canonical),
            "exact": exact,
            "seal": _compact_tracked_utf8_seal(seal),
        }
    accepted_fixture_hashes = {
        item["fixture_id"]: item["fixture_hash"]
        for item in accepted_payloads[
            "gate_a_triple_prime_receipt.json"
        ]["fixtures"]
    }
    fixture_records = []
    fixture_failures = []
    for fixture_id in FIXTURE_IDS:
        record, failures = _fixture_integrity(
            source_root,
            fixture_base / "cases" / fixture_id,
            accepted_fixture_hashes,
        )
        fixture_records.append(record)
        fixture_failures.extend(failures)
    source_audit = _source_predicate_audit(source_root)
    source_recomputed = _recompute_source_byte_seals(source_audit)
    phase0_bytes = _git_bytes(
        source_root,
        f"{PROOF_BASE_REVISION}:{PHASE0_RECEIPT_PATH}",
    )
    phase0_exact = _sha256(phase0_bytes) == PHASE0_RECEIPT_SHA256
    product_tree_oid = _git_value(source_root, "HEAD:kernel/src")
    all_exact = (
        accepted_exact
        and not fixture_failures
        and source_recomputed["passed"]
        and phase0_exact
        and product_tree_oid == EXPECTED_KERNEL_SRC_TREE_OID
    )
    return {
        "byte_domain_law": (
            "UTF8_GIT_BLOB_CANONICAL_LF_V1;_SEAL_SHA256_AND_BLOB_OID_"
            "DERIVE_ONLY_FROM_TRACKED_GIT_BLOB_BYTES_NORMALIZED_TO_LF;_"
            "WORKSPACE_BYTES_ARE_ADMISSION_ONLY_AND_MUST_MATCH_AFTER_"
            "CRLF_CR_TO_LF_NORMALIZATION"
        ),
        "outcome": (
            "BYTE_SEALS_EXACT"
            if all_exact
            else "BYTE_SEAL_AUDIT_REFUSED"
        ),
        "product_tree_oid": product_tree_oid,
        "source_seals": source_recomputed,
        "accepted_receipts": accepted_records,
        "fixture_inputs": fixture_records,
        "fixture_failures": fixture_failures,
        "phase0_history": {
            "git_expression": (
                f"{PROOF_BASE_REVISION}:{PHASE0_RECEIPT_PATH}"
            ),
            "canonical_git_blob_sha256": _sha256(phase0_bytes),
            "expected_sha256": PHASE0_RECEIPT_SHA256,
            "exact": phase0_exact,
        },
    }


def _filter_span_audit(
    fixtures: list[dict[str, Any]],
    synthetics: dict[str, dict[str, Any]],
    source_audit: dict[str, Any],
) -> dict[str, Any]:
    counters = {
        "field_born_zero_fan_count_mismatches": 0,
        "required_control_fan_edges": 0,
        "required_control_born_zero_filters": 0,
        "code_path_order_unproven": 0,
    }
    classified_seed_adjacency_diagnostic = 0
    field = []
    for fixture in fixtures:
        terminal = fixture["region"]["terminal"]
        born_zero = terminal["named_candidate_rejection_counters"][
            "refused_filter_span_is_born_zero"
        ]
        fan_edges = fixture["region"]["bridge_polygon"]["fan_edge_count"]
        field.append(
            {
                "fixture_id": fixture["fixture_id"],
                "filter_span_is_born_zero": born_zero,
                "fan_edge_count": fan_edges,
                "count_equals_fan_edge_count": born_zero == fan_edges,
                "certified_straight_seed_vertex_count": fixture["region"][
                    "seed"
                ]["certified_straight_seed_vertex_count"],
            }
        )
        counters["field_born_zero_fan_count_mismatches"] += int(
            born_zero != fan_edges
        )
        for vertex in fixture["region"]["seed"][
            "certified_straight_seed_vertices"
        ]:
            classified_seed_adjacency_diagnostic += sum(
                outcome != "WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT"
                for outcome in (
                    vertex["previous_edge_event_outcome"],
                    vertex["following_edge_event_outcome"],
                )
            )
    synthetic = []
    for name, case in synthetics.items():
        for mode, result in case["modes"].items():
            filter_count = result["terminal"][
                "named_candidate_rejection_counters"
            ]["refused_filter_span_is_born_zero"]
            fan_count = sum(
                item["span_is_zero_length"]
                for item in result["seed"]["edges"]
            )
            synthetic.append(
                {
                    "case": name,
                    "mode": mode,
                    "filter_span_is_born_zero": filter_count,
                    "fan_edge_count": fan_count,
                }
            )
            if name in ("H1_REQUIRED", "H2_SCALE_CONTROL"):
                counters["required_control_fan_edges"] += fan_count
                counters[
                    "required_control_born_zero_filters"
                ] += filter_count
    observed_order = source_audit["enqueue_edge_event_observed_order"][
        "always_concurrent_precedes_span_and_filter"
    ]
    counters["code_path_order_unproven"] = int(not observed_order)
    outcome = (
        "FILTER_ORTHOGONALITY_CONFIRMED"
        if not any(counters.values())
        else "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
    )
    return {
        "outcome": outcome,
        "existing_rule": "FILTER_SPAN_IS_BORN_ZERO",
        "failure_counters": counters,
        "field_counts": field,
        "synthetic_counts": synthetic,
        "classified_seed_non_always_concurrent_adjacencies_diagnostic": (
            classified_seed_adjacency_diagnostic
        ),
        "classified_seed_adjacency_is_non_gating_reason": (
            "PHASE1_CLASSIFICATION_SUPPLIES_THE_EXACT_SLIDING_POSITION;_"
            "THE_PHASE0_MISSING_POSITION_REFUSAL_IS_NO_LONGER_THE_RUNTIME_"
            "OBLIGATION"
        ),
        "observed_control_flow": source_audit[
            "enqueue_edge_event_observed_order"
        ],
        "applicability_conclusion": (
            "NOT_THE_SEED_STRAIGHT_JOINT_RULE;_IT_FILTERS_A_BORN_ZERO_"
            "COLLAPSING_SPAN_AFTER_AN_EXACT_EVENT_TIME_EXISTS"
            if outcome == "FILTER_ORTHOGONALITY_CONFIRMED"
            else "NOT_PROVEN"
        ),
    }


def _terminal_signature(result: dict[str, Any]) -> dict[str, Any]:
    terminal = result["terminal"]
    return {
        "outcome": terminal["outcome"],
        "levels": terminal["levels"],
        "node_count": terminal["node_count"],
        "live_vertex_count": terminal["live_vertex_count"],
        "named_candidate_rejection_counters": terminal[
            "named_candidate_rejection_counters"
        ],
        "face_outcome": result["face_outcome"],
        "face_count": result["face_count"],
        "polygon_doubled_area": result["polygon_doubled_area"],
    }


def _h2_evidence_audit(
    fixtures: list[dict[str, Any]],
    synthetics: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    counters = {
        "certified_rescaling_cases_missing": 0,
        "certified_rescaling_class_multiplicity_failures": 0,
        "certified_rescaling_distinct_raw_key_failures": 0,
        "certified_rescaling_support_class_failures": 0,
        "certified_rescaling_q_over_N_failures": 0,
        "unequal_speed_certified_straight_vertices": 0,
        "different_speed_named_refusals": 0,
        "h1_control_same_representation_failures": 0,
        "h2_control_rescaling_failures": 0,
        "h2_control_terminal_signature_differences": 0,
    }
    certified = [
        {
            "fixture_id": fixture["fixture_id"],
            **vertex,
        }
        for fixture in fixtures
        for vertex in fixture["region"]["seed"][
            "certified_straight_seed_vertices"
        ]
    ]
    rescaling = [
        item for item in certified if not item["raw_line_keys_equal"]
    ]
    counters["certified_rescaling_cases_missing"] = int(not rescaling)
    for item in certified:
        counters["unequal_speed_certified_straight_vertices"] += int(
            not item["normalized_speeds_equal"]
        )
    for item in rescaling:
        counters[
            "certified_rescaling_class_multiplicity_failures"
        ] += int(item["support_line_class_member_count"] < 2)
        counters[
            "certified_rescaling_distinct_raw_key_failures"
        ] += int(item["support_line_class_distinct_raw_key_count"] < 2)
        counters["certified_rescaling_support_class_failures"] += int(
            not item["same_support_line_class"]
        )
        counters["certified_rescaling_q_over_N_failures"] += int(
            item["prev_support"][
                "normalized_speed_squared_q_over_N"
            ]
            != item["next_support"][
                "normalized_speed_squared_q_over_N"
            ]
        )
    for fixture in fixtures:
        counters["different_speed_named_refusals"] += fixture["region"][
            "terminal"
        ]["named_candidate_rejection_counters"][
            "refused_no_rule_joint_is_codirectional_at_different_speeds"
        ]

    control_facts = []
    for mode in ("MOTORCYCLE", "EXHAUSTIVE"):
        h1 = synthetics["H1_REQUIRED"]["modes"][mode]
        h2 = synthetics["H2_SCALE_CONTROL"]["modes"][mode]
        h1_straight = h1["seed"]["certified_straight_seed_vertices"]
        h2_straight = h2["seed"]["certified_straight_seed_vertices"]
        h1_same = (
            len(h1_straight) == 1
            and h1_straight[0]["raw_line_keys_equal"]
            and h1_straight[0]["normalized_speeds_equal"]
        )
        h2_rescaled = (
            len(h2_straight) == 1
            and not h2_straight[0]["raw_line_keys_equal"]
            and h2_straight[0]["same_support_line_class"]
            and h2_straight[0]["support_line_class_member_count"] >= 2
            and h2_straight[0][
                "support_line_class_distinct_raw_key_count"
            ]
            >= 2
            and h2_straight[0]["normalized_speeds_equal"]
            and h2_straight[0]["prev_support"][
                "normalized_speed_squared_q_over_N"
            ]
            == h2_straight[0]["next_support"][
                "normalized_speed_squared_q_over_N"
            ]
        )
        h1_diff_speed = h1["terminal"][
            "named_candidate_rejection_counters"
        ]["refused_no_rule_joint_is_codirectional_at_different_speeds"]
        h2_diff_speed = h2["terminal"][
            "named_candidate_rejection_counters"
        ]["refused_no_rule_joint_is_codirectional_at_different_speeds"]
        signatures_equal = _terminal_signature(h1) == _terminal_signature(h2)
        counters["h1_control_same_representation_failures"] += int(
            not h1_same
        )
        counters["h2_control_rescaling_failures"] += int(not h2_rescaled)
        counters["different_speed_named_refusals"] += (
            h1_diff_speed + h2_diff_speed
        )
        counters[
            "h2_control_terminal_signature_differences"
        ] += int(not signatures_equal)
        control_facts.append(
            {
                "mode": mode,
                "h1_same_raw_representation": h1_same,
                "h2_integer_normal_rescaling": h2_rescaled,
                "h1_different_speed_named_refusals": h1_diff_speed,
                "h2_different_speed_named_refusals": h2_diff_speed,
                "terminal_signatures_equal": signatures_equal,
            }
        )
    outcome = (
        "H2_REPRESENTATION_ONLY"
        if not any(counters.values())
        else "INSUFFICIENT_H2_EVIDENCE"
    )
    return {
        "outcome": outcome,
        "failure_counters": counters,
        "certified_straight_vertex_count": len(certified),
        "certified_integer_normal_rescaling_case_count": len(rescaling),
        "certified_integer_normal_rescaling_cases": rescaling,
        "controls": control_facts,
    }


def _negative_self_checks(
    fixtures: list[dict[str, Any]],
    synthetics: dict[str, dict[str, Any]],
    source_audit: dict[str, Any],
    family: dict[str, Any],
) -> dict[str, Any]:
    checks = []
    self_checks_pass = {"all_passed": True, "checks": []}

    def decision_for(
        candidate_fixtures,
        candidate_synthetics,
        h2_audit,
        filter_audit,
    ):
        return _decide(
            candidate_fixtures,
            candidate_synthetics,
            family,
            source_audit,
            h2_audit,
            filter_audit,
            self_checks_pass,
            [],
        )

    no_h2_fixtures = copy.deepcopy(fixtures)
    no_h2_synthetics = copy.deepcopy(synthetics)
    for fixture in no_h2_fixtures:
        seed = fixture["region"]["seed"]
        seed["certified_straight_seed_vertices"] = []
        seed["certified_straight_seed_vertex_count"] = 0
    for case in no_h2_synthetics.values():
        for result in case["modes"].values():
            seed = result["seed"]
            seed["certified_straight_seed_vertices"] = []
            seed["certified_straight_seed_vertex_count"] = 0
    no_h2 = _h2_evidence_audit(no_h2_fixtures, no_h2_synthetics)
    no_h2_decision = decision_for(
        no_h2_fixtures,
        no_h2_synthetics,
        no_h2,
        _filter_span_audit(
            no_h2_fixtures, no_h2_synthetics, source_audit
        ),
    )
    checks.append(
        {
            "name": "REMOVE_ALL_H2_FACTS",
            "expected_outcome": "INSUFFICIENT_H2_EVIDENCE",
            "expected_named_counter": "certified_rescaling_cases_missing",
            "observed_outcome": no_h2["outcome"],
            "observed_decision_outcome": no_h2_decision[0],
            "observed_decision_fact": no_h2_decision[1][
                "named_first_exact_fact"
            ],
            "observed_failure_counters": no_h2["failure_counters"],
            "passed": (
                no_h2["outcome"] == "INSUFFICIENT_H2_EVIDENCE"
                and no_h2_decision[0] == "REFUSED"
                and no_h2_decision[1]["named_first_exact_fact"]
                == "INSUFFICIENT_H2_EVIDENCE"
                and no_h2["failure_counters"][
                    "certified_rescaling_cases_missing"
                ]
                > 0
            ),
        }
    )

    altered_q = copy.deepcopy(fixtures)
    altered = next(
        vertex
        for fixture in altered_q
        for vertex in fixture["region"]["seed"][
            "certified_straight_seed_vertices"
        ]
        if not vertex["raw_line_keys_equal"]
    )
    altered["next_support"][
        "normalized_speed_squared_q_over_N"
    ] = "SELF_CHECK_ALTERED_Q_OVER_N"
    altered_q_audit = _h2_evidence_audit(altered_q, synthetics)
    altered_q_decision = decision_for(
        altered_q,
        synthetics,
        altered_q_audit,
        _filter_span_audit(altered_q, synthetics, source_audit),
    )
    checks.append(
        {
            "name": "ALTER_ONE_Q_OVER_N",
            "expected_outcome": "INSUFFICIENT_H2_EVIDENCE",
            "expected_named_counter": (
                "certified_rescaling_q_over_N_failures"
            ),
            "observed_outcome": altered_q_audit["outcome"],
            "observed_decision_outcome": altered_q_decision[0],
            "observed_decision_fact": altered_q_decision[1][
                "named_first_exact_fact"
            ],
            "observed_failure_counters": altered_q_audit[
                "failure_counters"
            ],
            "passed": (
                altered_q_audit["outcome"] == "INSUFFICIENT_H2_EVIDENCE"
                and altered_q_decision[0] == "REFUSED"
                and altered_q_decision[1]["named_first_exact_fact"]
                == "INSUFFICIENT_H2_EVIDENCE"
                and altered_q_audit["failure_counters"][
                    "certified_rescaling_q_over_N_failures"
                ]
                > 0
            ),
        }
    )

    diff_speed = copy.deepcopy(fixtures)
    diff_speed[0]["region"]["terminal"][
        "named_candidate_rejection_counters"
    ][
        "refused_no_rule_joint_is_codirectional_at_different_speeds"
    ] = 1
    diff_speed_audit = _h2_evidence_audit(diff_speed, synthetics)
    diff_speed_decision = decision_for(
        diff_speed,
        synthetics,
        diff_speed_audit,
        _filter_span_audit(diff_speed, synthetics, source_audit),
    )
    checks.append(
        {
            "name": "SET_DIFFERENT_SPEED_NAMED_REFUSAL",
            "expected_outcome": "INSUFFICIENT_H2_EVIDENCE",
            "expected_named_counter": "different_speed_named_refusals",
            "observed_outcome": diff_speed_audit["outcome"],
            "observed_decision_outcome": diff_speed_decision[0],
            "observed_decision_fact": diff_speed_decision[1][
                "named_first_exact_fact"
            ],
            "observed_failure_counters": diff_speed_audit[
                "failure_counters"
            ],
            "passed": (
                diff_speed_audit["outcome"]
                == "INSUFFICIENT_H2_EVIDENCE"
                and diff_speed_decision[0] == "REFUSED"
                and diff_speed_decision[1]["named_first_exact_fact"]
                == "INSUFFICIENT_H2_EVIDENCE"
                and diff_speed_audit["failure_counters"][
                    "different_speed_named_refusals"
                ]
                > 0
            ),
        }
    )

    altered_fan = copy.deepcopy(fixtures)
    altered_fan[0]["region"]["bridge_polygon"]["fan_edge_count"] += 1
    altered_fan_audit = _filter_span_audit(
        altered_fan, synthetics, source_audit
    )
    altered_fan_decision = decision_for(
        altered_fan,
        synthetics,
        _h2_evidence_audit(altered_fan, synthetics),
        altered_fan_audit,
    )
    checks.append(
        {
            "name": "PERTURB_ONE_BORN_ZERO_FAN_COUNT",
            "expected_outcome": (
                "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
            ),
            "expected_named_counter": (
                "field_born_zero_fan_count_mismatches"
            ),
            "observed_outcome": altered_fan_audit["outcome"],
            "observed_decision_outcome": altered_fan_decision[0],
            "observed_decision_fact": altered_fan_decision[1][
                "named_first_exact_fact"
            ],
            "observed_failure_counters": altered_fan_audit[
                "failure_counters"
            ],
            "passed": (
                altered_fan_audit["outcome"]
                == "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
                and altered_fan_decision[0] == "REFUSED"
                and altered_fan_decision[1]["named_first_exact_fact"]
                == "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
                and altered_fan_audit["failure_counters"][
                    "field_born_zero_fan_count_mismatches"
                ]
                > 0
            ),
        }
    )

    control_filter = copy.deepcopy(synthetics)
    for name in ("H1_REQUIRED", "H2_SCALE_CONTROL"):
        control_filter[name]["modes"]["MOTORCYCLE"]["terminal"][
            "named_candidate_rejection_counters"
        ]["refused_filter_span_is_born_zero"] = 1
    control_filter_audit = _filter_span_audit(
        fixtures, control_filter, source_audit
    )
    control_filter_decision = decision_for(
        fixtures,
        control_filter,
        _h2_evidence_audit(fixtures, control_filter),
        control_filter_audit,
    )
    checks.append(
        {
            "name": "SET_REQUIRED_CONTROL_FILTER_NONZERO",
            "expected_outcome": (
                "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
            ),
            "expected_named_counter": (
                "required_control_born_zero_filters"
            ),
            "observed_outcome": control_filter_audit["outcome"],
            "observed_decision_outcome": control_filter_decision[0],
            "observed_decision_fact": control_filter_decision[1][
                "named_first_exact_fact"
            ],
            "observed_failure_counters": control_filter_audit[
                "failure_counters"
            ],
            "passed": (
                control_filter_audit["outcome"]
                == "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
                and control_filter_decision[0] == "REFUSED"
                and control_filter_decision[1]["named_first_exact_fact"]
                == "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
                and control_filter_audit["failure_counters"][
                    "required_control_born_zero_filters"
                ]
                > 0
            ),
        }
    )
    return {
        "all_passed": all(item["passed"] for item in checks),
        "checks": checks,
    }


def _decide(
    fixtures: list[dict[str, Any]],
    synthetics: dict[str, dict[str, Any]],
    family: dict[str, Any],
    source_audit: dict[str, Any],
    h2_audit: dict[str, Any],
    filter_audit: dict[str, Any],
    negative_self_checks: dict[str, Any],
    failures: list[dict[str, Any]],
) -> tuple[str, dict[str, Any]]:
    if failures:
        return "REFUSED", {
            "named_first_exact_fact": failures[0]["code"],
            "detail": failures[0]["detail"],
        }
    straight = [
        vertex
        for fixture in fixtures
        for vertex in fixture["region"]["seed"][
            "certified_straight_seed_vertices"
        ]
    ]
    if h2_audit["outcome"] != "H2_REPRESENTATION_ONLY":
        return "REFUSED", {
            "named_first_exact_fact": "INSUFFICIENT_H2_EVIDENCE",
            "failure_counters": h2_audit["failure_counters"],
        }
    if filter_audit["outcome"] != "FILTER_ORTHOGONALITY_CONFIRMED":
        return "REFUSED", {
            "named_first_exact_fact": (
                "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE"
            ),
            "failure_counters": filter_audit["failure_counters"],
        }
    if not negative_self_checks["all_passed"]:
        return "REFUSED", {
            "named_first_exact_fact": "NEGATIVE_SELF_CHECK_FAILED",
            "failed_checks": [
                item["name"]
                for item in negative_self_checks["checks"]
                if not item["passed"]
            ],
        }
    public_first_failure_is_skeleton = all(
        fixture["compile"]["outcome"] == "EXACT"
        and fixture["region"]["bridge_outcome"] == "EXACT"
        and fixture["prepare"]["outcome"] == "SKELETON_DID_NOT_CLOSE"
        and fixture["coverage"]["outcome"] == "PREPARATION_IS_NOT_EXACT"
        for fixture in fixtures
    )
    missing_seed_classification = (
        bool(straight)
        and all(not item["sliding_initialized"] for item in straight)
        and source_audit["first_exact_structural_difference"]
        == "SEED_VERTEX_CONSTRUCTION_OMITS_EVENT_BORN_SLIDING_CLASSIFICATION"
    )
    required_controls_unresolved = all(
        result["terminal"]["outcome"] == "WAVEFRONT_LEFT_UNRESOLVED"
        for name in ("H1_REQUIRED", "H2_SCALE_CONTROL")
        for result in synthetics[name]["modes"].values()
    )
    if (
        public_first_failure_is_skeleton
        and missing_seed_classification
        and required_controls_unresolved
        and family["non_exact_count"] == 0
    ):
        return "H1_CONFIRMED", {
            "named_first_exact_fact": (
                "CERTIFIED_STRAIGHT_SEED_VERTEX_IS_NOT_INITIALIZED_AS_SLIDING"
            ),
            "certified_seed_straight_count": len(straight),
            "certified_seed_straight_sliding_count": 0,
            "h2_classification": h2_audit["outcome"],
            "structural_resolution_obligation": (
                "CLASSIFY_A_CERTIFIED_STRAIGHT_SEED_JOINT_AS_ONE_SLIDING_"
                "MOVING_LINE_WITHOUT_CHANGING_THE_GLOBAL_S_PRIME_LATTICE_"
                "OR_ANY_INPUT_COORDINATE"
            ),
        }
    return "THIRD_ROOT", {
        "named_first_exact_fact": "H1_AND_H2_PREDECLARED_FACTS_NOT_OBSERVED",
        "detail": "a new root must be named before another phase",
    }


def _recompute_source_byte_seals(
    source_audit: dict[str, Any],
) -> dict[str, Any]:
    seals = source_audit["tracked_source_byte_seals"]
    skeleton = seals["product_skeleton.py"]
    verifier = seals["phase1_verifier.py"]
    skeleton_bytes = skeleton["canonical_lf_utf8"].encode("utf-8")
    verifier_bytes = verifier["canonical_lf_utf8"].encode("utf-8")
    skeleton_tree = ast.parse(skeleton_bytes.decode("utf-8"))
    verifier_tree = ast.parse(verifier_bytes.decode("utf-8"))
    skeleton_functions = {
        node.name: node
        for node in ast.walk(skeleton_tree)
        if isinstance(node, ast.FunctionDef)
    }
    verifier_functions = {
        node.name: node
        for node in ast.walk(verifier_tree)
        if isinstance(node, ast.FunctionDef)
    }
    new_vertex = skeleton_functions["_new_vertex"]
    new_vertex_if = next(
        node for node in new_vertex.body if isinstance(node, ast.If)
    )
    product_predicate = ast.unparse(new_vertex_if.test)
    classifier = verifier_functions["_exact_straight_classifier"]
    classifier_return = next(
        node for node in classifier.body if isinstance(node, ast.Return)
    )
    classifier_predicate = ast.unparse(classifier_return.value)
    expected_records = {
        "product_skeleton.py": {
            "path": (
                "kernel/src/cftuv_envelope/wavefront/skeleton.py"
            ),
            "git_expression": (
                "HEAD:kernel/src/cftuv_envelope/wavefront/skeleton.py"
            ),
            "binding": "VALIDATED_HEAD_PRODUCT_TREE_BLOB",
        },
        "phase1_verifier.py": {
            "path": (
                "kernel/fixtures/sem_clb_02_lost_domains_v1/"
                "verify_gate_b_prime.py"
            ),
            "git_expression": (
                ":kernel/fixtures/sem_clb_02_lost_domains_v1/"
                "verify_gate_b_prime.py"
            ),
            "binding": "STAGED_PROOF_VERIFIER_BLOB_FOR_COMMIT",
        },
    }
    failures = 0
    for name, expected in expected_records.items():
        record = seals[name]
        failures += int(not _tracked_utf8_seal_exact(record))
        failures += sum(
            record[field] != value
            for field, value in expected.items()
        )
    failures += sum(
        (
            source_audit["product_skeleton_source_sha256"]
            != skeleton["canonical_lf_sha256"],
            source_audit["phase1_verifier_sha256"]
            != verifier["canonical_lf_sha256"],
            source_audit["event_born_new_vertex_predicate"]
            != product_predicate,
            source_audit["proof_local_seed_classifier_predicate"]
            != classifier_predicate,
            product_predicate != EXPECTED_NEW_VERTEX_PREDICATE,
            classifier_predicate != EXPECTED_NEW_VERTEX_PREDICATE,
            source_audit["tracked_source_byte_domain_law"]
            != (
                "UTF8_GIT_BLOB_CANONICAL_LF_V1;_PRODUCT_FROM_VALIDATED_"
                "HEAD_TREE;_VERIFIER_FROM_STAGED_INDEX_BLOB;_WORKSPACE_"
                "MUST_MATCH_AFTER_CRLF_CR_TO_LF_NORMALIZATION;_RAW_"
                "CHECKOUT_BYTES_NEVER_AUTHORIZE_A_SEAL"
            ),
        )
    )
    return {
        "failure_count": failures,
        "product_predicate_recomputed": product_predicate,
        "classifier_predicate_recomputed": classifier_predicate,
        "seals": {
            name: {
                "canonical_lf_sha256": record[
                    "canonical_lf_sha256"
                ],
                "git_blob_oid": record["git_blob_oid"],
                "exact": _tracked_utf8_seal_exact(record),
            }
            for name, record in seals.items()
        },
        "passed": failures == 0,
    }


PHASE1_FAILURE_ORDER = (
    (
        "unknown_kernel_src_product_tree",
        "UNKNOWN_KERNEL_SRC_PRODUCT_TREE",
    ),
    ("unknown_fixture_form", "UNKNOWN_FIXTURE_FORM"),
    ("unknown_verifier_schema", "UNKNOWN_VERIFIER_SCHEMA"),
    ("phase0_history_not_sealed", "PHASE0_HISTORY_NOT_SEALED"),
    ("accepted_a_proof_seal_failures", "BF6_V1_G1_SEAL_FAILURE"),
    (
        "source_byte_seal_failures",
        "SOURCE_BYTE_SEAL_FORGED_OR_STALE",
    ),
    (
        "classifier_determinant_predicate_failures",
        "SEED_CLASSIFIER_DETERMINANT_PREDICATE_MISMATCH",
    ),
    (
        "classifier_dot_predicate_failures",
        "SEED_CLASSIFIER_DOT_PREDICATE_MISMATCH",
    ),
    (
        "classifier_qN_predicate_failures",
        "SEED_CLASSIFIER_QN_PREDICATE_MISMATCH",
    ),
    ("seed_scope_excess", "SEED_CLASSIFIER_SCOPE_EXCESS"),
    (
        "input_coordinate_or_node_mutations",
        "INPUT_COORDINATE_OR_NODE_MUTATION",
    ),
    (
        "immutability_summary_inconsistencies",
        "IMMUTABILITY_SUMMARY_INCONSISTENT",
    ),
    ("adapter_point_mutations", "ADAPTER_POINT_MUTATION"),
    (
        "adapter_participant_mutations",
        "ADAPTER_PARTICIPANT_MUTATION",
    ),
    ("adapter_span_mutations", "ADAPTER_SPAN_MUTATION"),
    (
        "adapter_support_qN_mutations",
        "ADAPTER_SUPPORT_QN_MUTATION",
    ),
    (
        "adapter_projection_failures",
        "ADAPTER_FROZEN_PROJECTION_MISMATCH",
    ),
    (
        "adapter_summary_inconsistencies",
        "ADAPTER_SUMMARY_INCONSISTENT",
    ),
    (
        "participant_span_raw_key_merges",
        "PARTICIPANT_SPAN_RAW_KEY_MERGE",
    ),
    ("adapter_restoration_failures", "ADAPTER_NOT_RESTORED"),
    ("event_born_behavior_changes", "EVENT_BORN_BEHAVIOR_CHANGED"),
    ("public_exactness_failures", "PUBLIC_EXACTNESS_FAILURE"),
    ("public_mode_agreement_failures", "PUBLIC_MODE_AGREEMENT_FAILURE"),
    ("synthetic_exactness_failures", "SYNTHETIC_EXACTNESS_FAILURE"),
    ("assembler_contour_failures", "ASSEMBLER_CONTOUR_NOT_SIMPLE"),
    ("assembler_face_area_mutations", "ASSEMBLER_FACE_AREA_MUTATION"),
    (
        "assembler_ownership_mutations",
        "ASSEMBLER_OWNERSHIP_SOURCE_SPAN_MUTATION",
    ),
    (
        "assembler_summary_inconsistencies",
        "ASSEMBLER_SUMMARY_INCONSISTENT",
    ),
    ("assembler_border_failures", "ASSEMBLER_BORDER_FAILURE"),
    ("h2_evidence_failures", "INSUFFICIENT_H2_EVIDENCE"),
    (
        "filter_orthogonality_failures",
        "INSUFFICIENT_FILTER_ORTHOGONALITY_EVIDENCE",
    ),
    ("family_regression_failures", "BF6_FAMILY_REGRESSION"),
    ("negative_self_check_failures", "NEGATIVE_SELF_CHECK_FAILED"),
)


def _phase1_h2_audit(
    fixtures: list[dict[str, Any]],
    different_speed_control: dict[str, Any],
) -> dict[str, Any]:
    counters = {
        "certified_straight_count_mismatch": 0,
        "classified_seed_count_mismatch": 0,
        "unequal_q_over_N": 0,
        "support_class_identity_failures": 0,
        "rescaling_evidence_count_mismatch": 0,
        "different_speed_control_failures": 0,
    }
    rescaling = []
    total = 0
    for fixture in fixtures:
        seed = fixture["region"]["seed"]
        vertices = seed["certified_straight_seed_vertices"]
        total += len(vertices)
        expected = EXPECTED_CLASSIFIED_COUNTS[fixture["fixture_id"]]
        counters["certified_straight_count_mismatch"] += int(
            len(vertices) != expected
        )
        counters["classified_seed_count_mismatch"] += int(
            seed["certified_straight_seed_sliding_count"] != expected
        )
        for vertex in vertices:
            left = vertex["prev_support"]
            right = vertex["next_support"]
            counters["unequal_q_over_N"] += int(
                left["normalized_speed_squared_q_over_N"]
                != right["normalized_speed_squared_q_over_N"]
                or not vertex["normalized_speeds_equal"]
            )
            counters["support_class_identity_failures"] += int(
                not vertex["same_support_line_class"]
                or vertex["support_line_class_member_count"] < 2
            )
            if not vertex["raw_line_keys_equal"]:
                rescaling.append(
                    {
                        "fixture_id": fixture["fixture_id"],
                        "vertex_id": vertex["vertex_id"],
                        "prev_span": vertex["prev_span"],
                        "next_span": vertex["next_span"],
                        "prev_raw_line_key": left["raw_line_key"],
                        "next_raw_line_key": right["raw_line_key"],
                        "support_line_class": left["support_line_class"],
                        "q_over_N": (
                            left[
                                "normalized_speed_squared_q_over_N"
                            ]
                        ),
                    }
                )
    counters["rescaling_evidence_count_mismatch"] = int(
        len(rescaling) != 10
    )
    counters["different_speed_control_failures"] = int(
        not different_speed_control["passed"]
    )
    return {
        "outcome": (
            "H2_REPRESENTATION_ONLY_RETAINED"
            if not any(counters.values())
            else "INSUFFICIENT_H2_EVIDENCE"
        ),
        "certified_straight_vertex_count": total,
        "integer_normal_rescaling_case_count": len(rescaling),
        "integer_normal_rescaling_cases": rescaling,
        "different_speed_local_control": different_speed_control,
        "failure_counters": counters,
    }


def _phase1_gate_audit(
    fixtures: list[dict[str, Any]],
    synthetics: dict[str, dict[str, Any]],
    family: dict[str, Any],
    source_audit: dict[str, Any],
    h2_audit: dict[str, Any],
    filter_audit: dict[str, Any],
    forced_restoration: dict[str, Any],
    modules: dict[str, Any],
    *,
    product_tree_ok: bool,
    fixture_form_ok: bool,
    verifier_schema_ok: bool,
    phase0_history_ok: bool,
    accepted_a_proofs_ok: bool,
) -> dict[str, Any]:
    counters = {name: 0 for name, _ in PHASE1_FAILURE_ORDER}
    counters["unknown_kernel_src_product_tree"] = int(
        not product_tree_ok
    )
    counters["unknown_fixture_form"] = int(not fixture_form_ok)
    counters["unknown_verifier_schema"] = int(not verifier_schema_ok)
    counters["phase0_history_not_sealed"] = int(not phase0_history_ok)
    counters["accepted_a_proof_seal_failures"] = int(
        not accepted_a_proofs_ok
    )
    source_seal_recomputed = _recompute_source_byte_seals(
        source_audit
    )
    counters["source_byte_seal_failures"] = source_seal_recomputed[
        "failure_count"
    ]

    classifier = source_audit[
        "proof_local_seed_classifier_predicate"
    ]
    counters["classifier_determinant_predicate_failures"] = int(
        "first.a * second.b - second.a * first.b == 0" not in classifier
    )
    counters["classifier_dot_predicate_failures"] = int(
        "first.a * second.a + first.b * second.b > 0" not in classifier
    )
    counters["classifier_qN_predicate_failures"] = int(
        "first.q * second.normal_squared == second.q * first.normal_squared"
        not in classifier
    )
    recomputed_source_predicate_identity = (
        source_audit["event_born_new_vertex_predicate"]
        == source_audit["proof_local_seed_classifier_predicate"]
        == source_audit["expected_predicate"]
        == EXPECTED_NEW_VERTEX_PREDICATE
    )
    counters["event_born_behavior_changes"] = int(
        not recomputed_source_predicate_identity
        or source_audit[
            "product_predicate_and_proof_classifier_identical"
        ]
        != recomputed_source_predicate_identity
        or source_audit["predicate_exact_match"]
        != recomputed_source_predicate_identity
        or source_audit["proof_local_classifier_exact_match"]
        != recomputed_source_predicate_identity
        or not source_audit["event_born_constructor_sets_sliding"]
    )

    field_gate_records = []
    for fixture in fixtures:
        expected_count = EXPECTED_CLASSIFIED_COUNTS[
            fixture["fixture_id"]
        ]
        seed = fixture["region"]["seed"]
        terminal = fixture["region"]["terminal"]
        adapter_recomputations = [
            {
                "split_search": record["split_search"],
                **_recompute_adapter_record(record),
            }
            for record in fixture["adapter"]["records"]
        ]
        canonical_ids = sorted(
            ident
            for result in adapter_recomputations
            if result["split_search"] == "MOTORCYCLE"
            for ident in result["recomputed_classified_vertex_ids"]
        )
        certified_ids = sorted(
            vertex["vertex_id"]
            for vertex in seed["certified_straight_seed_vertices"]
        )
        scope_excess = sum(
            record["seed_scope_excess_count"]
            for record in fixture["adapter"]["records"]
        )
        immutable_recomputed = _recompute_immutable_input(
            fixture["immutable_input"]
        )
        mode_recomputed = _recompute_mode_agreement(fixture)
        assembler_recomputed = _recompute_assembler_borders(
            fixture, modules
        )
        counters["seed_scope_excess"] += scope_excess
        counters["adapter_point_mutations"] += sum(
            result["field_mismatch_counts"]["point"]
            for result in adapter_recomputations
        )
        counters["adapter_participant_mutations"] += sum(
            result["field_mismatch_counts"][field]
            for result in adapter_recomputations
            for field in (
                "vertex_id",
                "prev_edge_id",
                "next_edge_id",
                "prev_vertex_id",
                "next_vertex_id",
            )
        )
        counters["adapter_span_mutations"] += sum(
            result["field_mismatch_counts"][field]
            for result in adapter_recomputations
            for field in ("prev_span", "next_span")
        )
        counters["adapter_support_qN_mutations"] += sum(
            result["field_mismatch_counts"][field]
            for result in adapter_recomputations
            for field in (
                "prev_raw_line_key",
                "next_raw_line_key",
                "prev_support_line_class",
                "next_support_line_class",
                "prev_q_over_N",
                "next_q_over_N",
                "determinant",
                "dot",
                "q1_N2",
                "q2_N1",
                "exact_classifier_result",
            )
        )
        counters["adapter_projection_failures"] += sum(
            result["projection_failures"]
            + result["field_mismatch_counts"]["frozen_projection"]
            for result in adapter_recomputations
        )
        counters["adapter_summary_inconsistencies"] += sum(
            result["summary_inconsistencies"]
            for result in adapter_recomputations
        )
        counters["participant_span_raw_key_merges"] += int(
            canonical_ids != certified_ids
        )
        counters["input_coordinate_or_node_mutations"] += int(
            not all(
                immutable_recomputed["leaf_equalities"].values()
            )
        )
        counters["immutability_summary_inconsistencies"] += (
            immutable_recomputed["summary_inconsistencies"]
        )
        normal_restore_recomputed = (
            fixture["adapter"]["seed_hook_before"]
            == fixture["adapter"]["seed_hook_after"]
            and fixture["adapter"]["event_born_hook_before"]
            == fixture["adapter"]["event_born_hook_after"]
        )
        counters["adapter_restoration_failures"] += int(
            not normal_restore_recomputed
            or fixture["adapter"]["normal_exit_restored"]
            != normal_restore_recomputed
            or fixture["adapter"][
                "event_born_new_vertex_callable_unchanged"
            ]
            != (
                fixture["adapter"]["event_born_hook_before"]
                == fixture["adapter"]["event_born_hook_after"]
            )
        )
        public_exact = (
            fixture["compile"]["outcome"] == "EXACT"
            and fixture["prepare"]["outcome"] == "EXACT"
            and fixture["coverage"]["outcome"] == "EXACT"
            and fixture["coverage"]["region_outcome"] == "EXACT"
            and fixture["region"]["terminal"]["outcome"] == "EXACT"
            and fixture["region"]["partition"]["outcome"] == "EXACT"
            and seed["certified_straight_seed_vertex_count"]
            == expected_count
            and seed["certified_straight_seed_sliding_count"]
            == expected_count
            and terminal["live_vertex_count"] == 0
            and terminal["queue_length"] == 0
        )
        mode_agreement = mode_recomputed["passed"]
        assembler_exact = assembler_recomputed["passed"]
        counters["public_exactness_failures"] += int(not public_exact)
        counters["public_mode_agreement_failures"] += int(
            not mode_agreement
        )
        counters["assembler_border_failures"] += int(
            not all(assembler_recomputed["border_values"].values())
        )
        counters["assembler_summary_inconsistencies"] += (
            assembler_recomputed["summary_inconsistencies"]
        )
        counters["assembler_contour_failures"] += int(
            not assembler_recomputed["simple_contours"]
        )
        counters["assembler_face_area_mutations"] += (
            assembler_recomputed["face_area_mutations"]
        )
        counters["assembler_ownership_mutations"] += int(
            not assembler_recomputed["ownership_exact"]
        )
        field_gate_records.append(
            {
                "fixture_id": fixture["fixture_id"],
                "expected_classified_count": expected_count,
                "observed_classified_count": (
                    seed["certified_straight_seed_sliding_count"]
                ),
                "classified_ids_equal_certified_ids": (
                    canonical_ids == certified_ids
                ),
                "public_compile_prepare_coverage_exact": public_exact,
                "canonical_exhaustive_agreement": mode_agreement,
                "mode_agreement_recomputation": mode_recomputed,
                "immutable_input_exact": immutable_recomputed["passed"],
                "immutable_input_recomputation": (
                    immutable_recomputed
                ),
                "assembler_borders_exact": assembler_exact,
                "assembler_recomputation": assembler_recomputed,
                "adapter_recomputations": adapter_recomputations,
                "terminal_live_frontier": terminal["live_vertex_count"],
                "terminal_queue": terminal["queue_length"],
                "frozen_levels": terminal["levels"],
                "frozen_node_count": terminal["node_count"],
                "frozen_face_count": len(
                    fixture["region"]["partition"]["faces"]
                ),
                "frozen_event_count": len(
                    fixture["region"]["skeleton_semantics"]["nodes"]
                ),
                "frozen_counter_count": len(terminal["counters"]),
            }
        )

    synthetic_gate_records = []
    for name, expected in EXPECTED_SYNTHETIC.items():
        case = synthetics[name]
        polygon_equal = (
            _pretty_json(case["polygon_before"])
            == _pretty_json(case["polygon_after"])
        )
        mode_values = list(case["modes"].values())
        recomputed_mode = {
            "outcome_equal": (
                mode_values[0]["terminal"]["outcome"]
                == mode_values[1]["terminal"]["outcome"]
            ),
            "levels_equal": (
                mode_values[0]["terminal"]["levels"]
                == mode_values[1]["terminal"]["levels"]
            ),
            "nodes_byte_identical": (
                _pretty_json(
                    mode_values[0]["skeleton_semantics"]["nodes"]
                )
                == _pretty_json(
                    mode_values[1]["skeleton_semantics"]["nodes"]
                )
            ),
            "partition_byte_identical": (
                _pretty_json(mode_values[0]["partition"])
                == _pretty_json(mode_values[1]["partition"])
            ),
        }
        recomputed_mode["all_exact_and_agree"] = (
            all(
                mode["terminal"]["outcome"] == "EXACT"
                and mode["partition"]["outcome"] == "EXACT"
                for mode in mode_values
            )
            and all(
                recomputed_mode[key]
                for key in (
                    "outcome_equal",
                    "levels_equal",
                    "nodes_byte_identical",
                    "partition_byte_identical",
                )
            )
        )
        mode_summary_inconsistencies = sum(
            case["mode_agreement"][key] != value
            for key, value in recomputed_mode.items()
        )
        synthetic_adapter_recomputations = [
            _recompute_adapter_record(record)
            for record in case["adapter_records"]
        ]
        for result in synthetic_adapter_recomputations:
            counters["adapter_point_mutations"] += result[
                "field_mismatch_counts"
            ]["point"]
            counters["adapter_participant_mutations"] += sum(
                result["field_mismatch_counts"][field]
                for field in (
                    "vertex_id",
                    "prev_edge_id",
                    "next_edge_id",
                    "prev_vertex_id",
                    "next_vertex_id",
                )
            )
            counters["adapter_span_mutations"] += sum(
                result["field_mismatch_counts"][field]
                for field in ("prev_span", "next_span")
            )
            counters["adapter_support_qN_mutations"] += sum(
                result["field_mismatch_counts"][field]
                for field in (
                    "prev_raw_line_key",
                    "next_raw_line_key",
                    "prev_support_line_class",
                    "next_support_line_class",
                    "prev_q_over_N",
                    "next_q_over_N",
                    "determinant",
                    "dot",
                    "q1_N2",
                    "q2_N1",
                    "exact_classifier_result",
                )
            )
            counters["adapter_projection_failures"] += (
                result["projection_failures"]
                + result["field_mismatch_counts"]["frozen_projection"]
            )
            counters["adapter_summary_inconsistencies"] += result[
                "summary_inconsistencies"
            ]
        synthetic_restore_recomputed = (
            case["seed_hook_before"] == case["seed_hook_after"]
            and case["event_born_hook_before"]
            == case["event_born_hook_after"]
        )
        case_exact = (
            polygon_equal
            and case["polygon_byte_identical"] == polygon_equal
            and synthetic_restore_recomputed
            and case["adapter_normal_exit_restored"]
            == synthetic_restore_recomputed
            and case["event_born_new_vertex_callable_unchanged"]
            == (
                case["event_born_hook_before"]
                == case["event_born_hook_after"]
            )
            and recomputed_mode["all_exact_and_agree"]
            and mode_summary_inconsistencies == 0
            and all(
                result["passed"]
                for result in synthetic_adapter_recomputations
            )
        )
        synthetic_assembler_recomputations = []
        for result in case["modes"].values():
            assembler_recomputed = (
                _recompute_synthetic_assembler_borders(
                    result, modules
                )
            )
            synthetic_assembler_recomputations.append(
                assembler_recomputed
            )
            case_exact = (
                case_exact
                and result["terminal"]["outcome"] == "EXACT"
                and result["terminal"]["levels"] == expected["levels"]
                and result["terminal"]["node_count"]
                == expected["node_count"]
                and result["terminal"]["live_vertex_count"] == 0
                and result["terminal"]["queue_length"] == 0
                and result["face_outcome"] == "EXACT"
                and result["face_count"] == expected["face_count"]
                and result["polygon_doubled_area"]
                == expected["polygon_doubled_area"]
            )
            counters["assembler_border_failures"] += int(
                not assembler_recomputed["area_reproduces_polygon"]
            )
            counters["assembler_summary_inconsistencies"] += (
                assembler_recomputed["summary_inconsistencies"]
            )
            counters["assembler_contour_failures"] += int(
                not assembler_recomputed["simple_contours"]
            )
            counters["assembler_face_area_mutations"] += (
                assembler_recomputed["face_area_mutations"]
            )
        counters["synthetic_exactness_failures"] += int(not case_exact)
        counters["adapter_restoration_failures"] += int(
            not synthetic_restore_recomputed
            or case["adapter_normal_exit_restored"]
            != synthetic_restore_recomputed
        )
        synthetic_gate_records.append(
            {
                "case": name,
                "exact": case_exact,
                "expected": expected,
                "mode_agreement": case["mode_agreement"],
                "mode_agreement_recomputed": recomputed_mode,
                "mode_summary_inconsistencies": (
                    mode_summary_inconsistencies
                ),
                "adapter_recomputations": (
                    synthetic_adapter_recomputations
                ),
                "assembler_recomputations": (
                    synthetic_assembler_recomputations
                ),
            }
        )

    forced_restoration_recomputed = (
        forced_restoration["forced_exception"]
        == "FORCED_ADAPTER_EXCEPTION"
        and forced_restoration["seed_hook_before"]
        == forced_restoration["seed_hook_after"]
        and forced_restoration["event_born_hook_before"]
        == forced_restoration["event_born_hook_after"]
    )
    counters["adapter_restoration_failures"] += int(
        not forced_restoration_recomputed
        or forced_restoration["seed_hook_restored"]
        != (
            forced_restoration["seed_hook_before"]
            == forced_restoration["seed_hook_after"]
        )
        or forced_restoration["event_born_hook_unchanged"]
        != (
            forced_restoration["event_born_hook_before"]
            == forced_restoration["event_born_hook_after"]
        )
        or forced_restoration["adapter_reported_restored"]
        != forced_restoration_recomputed
        or forced_restoration["passed"]
        != forced_restoration_recomputed
    )
    counters["h2_evidence_failures"] = sum(
        h2_audit["failure_counters"].values()
    )
    counters["filter_orthogonality_failures"] = sum(
        filter_audit["failure_counters"].values()
    )
    counters["family_regression_failures"] = family["non_exact_count"]
    return {
        "outcome": (
            "ALL_PHASE1_GATES_SATISFIED"
            if not any(counters.values())
            else "PHASE1_GATE_REFUSED"
        ),
        "failure_counters": counters,
        "field_gate_records": field_gate_records,
        "synthetic_gate_records": synthetic_gate_records,
        "forced_exception_restoration": {
            "captured": forced_restoration,
            "recomputed": forced_restoration_recomputed,
        },
        "source_byte_seal_recomputation": source_seal_recomputed,
        "sealed_obligations": {
            "product_tree": product_tree_ok,
            "fixture_form": fixture_form_ok,
            "verifier_schema": verifier_schema_ok,
            "phase0_history": phase0_history_ok,
            "accepted_A_proofs_and_bf6_V1_G1_behavior": (
                accepted_a_proofs_ok
            ),
        },
    }


def _phase1_decide(
    gate_audit: dict[str, Any],
) -> tuple[str, dict[str, Any]]:
    counters = gate_audit["failure_counters"]
    for counter_name, named_fact in PHASE1_FAILURE_ORDER:
        if counters[counter_name]:
            return "REFUSED", {
                "named_first_exact_fact": named_fact,
                "named_counter": counter_name,
                "counter_value": counters[counter_name],
            }
    return "GATE_B_PRIME_GO", {
        "named_first_exact_fact": "ALL_PHASE1_GATES_SATISFIED",
        "candidate_scope": (
            "PROOF_LOCAL_SEED_CLASSIFICATION_OF_EXISTING_EVENT_SEMANTICS"
        ),
        "product_rule_implemented": False,
    }


def _phase1_negative_self_checks(
    gate_audit: dict[str, Any],
    fixtures: list[dict[str, Any]],
    synthetics: dict[str, dict[str, Any]],
    family: dict[str, Any],
    source_audit: dict[str, Any],
    different_speed_control: dict[str, Any],
    forced_restoration: dict[str, Any],
    modules: dict[str, Any],
) -> dict[str, Any]:
    mutations = (
        (
            "MUTATE_DETERMINANT_PREDICATE",
            "classifier_determinant_predicate_failures",
            "SEED_CLASSIFIER_DETERMINANT_PREDICATE_MISMATCH",
        ),
        (
            "MUTATE_DOT_PREDICATE",
            "classifier_dot_predicate_failures",
            "SEED_CLASSIFIER_DOT_PREDICATE_MISMATCH",
        ),
        (
            "MUTATE_QN_PREDICATE",
            "classifier_qN_predicate_failures",
            "SEED_CLASSIFIER_QN_PREDICATE_MISMATCH",
        ),
        (
            "CLASSIFY_OUTSIDE_SEED_SCOPE",
            "seed_scope_excess",
            "SEED_CLASSIFIER_SCOPE_EXCESS",
        ),
        (
            "MUTATE_INPUT_COORDINATE_OR_NODE",
            "input_coordinate_or_node_mutations",
            "INPUT_COORDINATE_OR_NODE_MUTATION",
        ),
        (
            "MERGE_PARTICIPANT_OR_SPAN",
            "participant_span_raw_key_merges",
            "PARTICIPANT_SPAN_RAW_KEY_MERGE",
        ),
        (
            "DO_NOT_RESTORE_ADAPTER",
            "adapter_restoration_failures",
            "ADAPTER_NOT_RESTORED",
        ),
        (
            "UNKNOWN_PRODUCT_TREE",
            "unknown_kernel_src_product_tree",
            "UNKNOWN_KERNEL_SRC_PRODUCT_TREE",
        ),
        (
            "UNKNOWN_FIXTURE_FORM",
            "unknown_fixture_form",
            "UNKNOWN_FIXTURE_FORM",
        ),
    )
    checks = []
    for name, counter_name, expected_fact in mutations:
        mutated = copy.deepcopy(gate_audit)
        mutated["failure_counters"][counter_name] += 1
        outcome, decision = _phase1_decide(mutated)
        checks.append(
            {
                "name": name,
                "mutated_counter": counter_name,
                "observed_counter_value": mutated["failure_counters"][
                    counter_name
                ],
                "expected_decision_outcome": "REFUSED",
                "expected_named_fact": expected_fact,
                "observed_decision_outcome": outcome,
                "observed_named_fact": decision[
                    "named_first_exact_fact"
                ],
                "passed": (
                    outcome == "REFUSED"
                    and decision["named_first_exact_fact"] == expected_fact
                    and decision["named_counter"] == counter_name
                ),
                "kind": "DECISION_MATRIX_MUTATION",
            }
        )

    forged_source_audit = copy.deepcopy(source_audit)
    forged_source_audit["product_skeleton_source_sha256"] = "0" * 64
    forged_h2_audit = _phase1_h2_audit(
        fixtures, different_speed_control
    )
    forged_filter_audit = _filter_span_audit(
        fixtures, synthetics, forged_source_audit
    )
    forged_source_gate = _phase1_gate_audit(
        fixtures,
        synthetics,
        family,
        forged_source_audit,
        forged_h2_audit,
        forged_filter_audit,
        forced_restoration,
        modules,
        product_tree_ok=True,
        fixture_form_ok=True,
        verifier_schema_ok=True,
        phase0_history_ok=True,
        accepted_a_proofs_ok=True,
    )
    forged_outcome, forged_decision = _phase1_decide(
        forged_source_gate
    )
    forged_counter = "source_byte_seal_failures"
    forged_fact = "SOURCE_BYTE_SEAL_FORGED_OR_STALE"
    checks.append(
        {
            "name": "FORGE_PRODUCT_SKELETON_SOURCE_SEAL",
            "kind": "SOURCE_BYTE_SEAL_MUTATION_WITH_CANONICAL_LEAVES",
            "mutated_counter": forged_counter,
            "observed_counter_value": forged_source_gate[
                "failure_counters"
            ][forged_counter],
            "expected_decision_outcome": "REFUSED",
            "expected_named_fact": forged_fact,
            "observed_decision_outcome": forged_outcome,
            "observed_named_fact": forged_decision[
                "named_first_exact_fact"
            ],
            "passed": (
                forged_outcome == "REFUSED"
                and forged_source_gate["failure_counters"][
                    forged_counter
                ]
                > 0
                and forged_decision["named_counter"]
                == forged_counter
                and forged_decision["named_first_exact_fact"]
                == forged_fact
            ),
        }
    )

    def decide_evidence(mutated_fixtures):
        mutated_h2 = _phase1_h2_audit(
            mutated_fixtures, different_speed_control
        )
        mutated_filter = _filter_span_audit(
            mutated_fixtures, synthetics, source_audit
        )
        mutated_gate = _phase1_gate_audit(
            mutated_fixtures,
            synthetics,
            family,
            source_audit,
            mutated_h2,
            mutated_filter,
            forced_restoration,
            modules,
            product_tree_ok=True,
            fixture_form_ok=True,
            verifier_schema_ok=True,
            phase0_history_ok=True,
            accepted_a_proofs_ok=True,
        )
        outcome, decision = _phase1_decide(mutated_gate)
        return mutated_gate, outcome, decision

    def append_evidence_check(
        name: str,
        mutate,
        expected_counter: str,
        expected_fact: str,
    ) -> None:
        mutated_fixtures = copy.deepcopy(fixtures)
        mutate(mutated_fixtures)
        mutated_gate, outcome, decision = decide_evidence(
            mutated_fixtures
        )
        counter_value = mutated_gate["failure_counters"][
            expected_counter
        ]
        checks.append(
            {
                "name": name,
                "kind": "EVIDENCE_LEAF_MUTATION_WITH_STALE_SUMMARIES",
                "mutated_counter": expected_counter,
                "observed_counter_value": counter_value,
                "expected_decision_outcome": "REFUSED",
                "expected_named_fact": expected_fact,
                "observed_decision_outcome": outcome,
                "observed_named_fact": decision[
                    "named_first_exact_fact"
                ],
                "passed": (
                    outcome == "REFUSED"
                    and counter_value > 0
                    and decision["named_counter"] == expected_counter
                    and decision["named_first_exact_fact"]
                    == expected_fact
                ),
            }
        )

    append_evidence_check(
        "STALE_ALL_EXACT_WITH_SNAPSHOT_BYTE_FLAG_FALSE",
        lambda items: items[0]["immutable_input"].__setitem__(
            "analysis_snapshot_byte_identical", False
        ),
        "immutability_summary_inconsistencies",
        "IMMUTABILITY_SUMMARY_INCONSISTENT",
    )
    append_evidence_check(
        "MUTATE_ADAPTER_AFTER_POINT",
        lambda items: items[0]["adapter"]["records"][0]["after"][0][
            "point"
        ].__setitem__("x", []),
        "adapter_point_mutations",
        "ADAPTER_POINT_MUTATION",
    )
    append_evidence_check(
        "MERGE_ADAPTER_AFTER_PREV_NEXT_PARTICIPANT",
        lambda items: items[0]["adapter"]["records"][0]["after"][0].__setitem__(
            "prev_vertex_id",
            items[0]["adapter"]["records"][0]["after"][0][
                "next_vertex_id"
            ],
        ),
        "adapter_participant_mutations",
        "ADAPTER_PARTICIPANT_MUTATION",
    )
    append_evidence_check(
        "MUTATE_ADAPTER_AFTER_SPAN",
        lambda items: items[0]["adapter"]["records"][0]["after"][0].__setitem__(
            "prev_span",
            [
                *items[0]["adapter"]["records"][0]["after"][0][
                    "prev_span"
                ],
                999999,
            ],
        ),
        "adapter_span_mutations",
        "ADAPTER_SPAN_MUTATION",
    )
    append_evidence_check(
        "MUTATE_ADAPTER_AFTER_RAW_KEY",
        lambda items: items[0]["adapter"]["records"][0]["after"][0].__setitem__(
            "prev_raw_line_key",
            [0, 0, 0, "MUTATED_RAW_KEY"],
        ),
        "adapter_support_qN_mutations",
        "ADAPTER_SUPPORT_QN_MUTATION",
    )
    append_evidence_check(
        "MUTATE_ADAPTER_AFTER_SUPPORT_CLASS",
        lambda items: items[0]["adapter"]["records"][0]["after"][0].__setitem__(
            "prev_support_line_class",
            ["MUTATED_SUPPORT_CLASS"],
        ),
        "adapter_support_qN_mutations",
        "ADAPTER_SUPPORT_QN_MUTATION",
    )
    append_evidence_check(
        "MUTATE_ADAPTER_AFTER_Q_OVER_N",
        lambda items: items[0]["adapter"]["records"][0]["after"][0].__setitem__(
            "prev_q_over_N", "MUTATED_Q_OVER_N"
        ),
        "adapter_support_qN_mutations",
        "ADAPTER_SUPPORT_QN_MUTATION",
    )
    append_evidence_check(
        "STALE_CONTOUR_SIMPLE_SUMMARY_FALSE",
        lambda items: items[0]["region"][
            "assembler_borders"
        ].__setitem__("every_contour_is_simple", False),
        "assembler_summary_inconsistencies",
        "ASSEMBLER_SUMMARY_INCONSISTENT",
    )
    def mutate_matching_mode_face_area(items) -> None:
        items[0]["region"]["partition"]["faces"][0][
            "doubled_area"
        ] = []
        items[0]["region"]["exhaustive"]["partition"]["faces"][0][
            "doubled_area"
        ] = []

    append_evidence_check(
        "MUTATE_PARTITION_FACE_AREA",
        mutate_matching_mode_face_area,
        "assembler_face_area_mutations",
        "ASSEMBLER_FACE_AREA_MUTATION",
    )
    append_evidence_check(
        "MUTATE_OWNERSHIP_SOURCE_SPAN",
        lambda items: items[0]["region"]["assembler_borders"][
            "leaf_evidence"
        ]["owner_by_edge"][0].__setitem__(
            "span",
            [
                *items[0]["region"]["assembler_borders"][
                    "leaf_evidence"
                ]["owner_by_edge"][0]["span"],
                999999,
            ],
        ),
        "assembler_ownership_mutations",
        "ASSEMBLER_OWNERSHIP_SOURCE_SPAN_MUTATION",
    )

    def append_synthetic_assembler_check(
        name: str,
        field: str,
        mode_names: tuple[str, ...],
    ) -> None:
        mutated_synthetics = copy.deepcopy(synthetics)
        for mode_name in mode_names:
            mutated_synthetics["H1_REQUIRED"]["modes"][mode_name][
                "assembler_borders"
            ][field] = False
        mutated_h2 = _phase1_h2_audit(
            fixtures, different_speed_control
        )
        mutated_filter = _filter_span_audit(
            fixtures, mutated_synthetics, source_audit
        )
        mutated_gate = _phase1_gate_audit(
            fixtures,
            mutated_synthetics,
            family,
            source_audit,
            mutated_h2,
            mutated_filter,
            forced_restoration,
            modules,
            product_tree_ok=True,
            fixture_form_ok=True,
            verifier_schema_ok=True,
            phase0_history_ok=True,
            accepted_a_proofs_ok=True,
        )
        outcome, decision = _phase1_decide(mutated_gate)
        counter_name = "assembler_summary_inconsistencies"
        counter_value = mutated_gate["failure_counters"][
            counter_name
        ]
        case_record = next(
            item
            for item in mutated_gate["synthetic_gate_records"]
            if item["case"] == "H1_REQUIRED"
        )
        mode_agreement_preserved = (
            case_record["mode_agreement_recomputed"][
                "all_exact_and_agree"
            ]
            and case_record["mode_summary_inconsistencies"] == 0
        )
        expected_count = len(mode_names)
        expected_fact = "ASSEMBLER_SUMMARY_INCONSISTENT"
        checks.append(
            {
                "name": name,
                "kind": (
                    "SYNTHETIC_ASSEMBLER_SUMMARY_MUTATION_"
                    "WITH_EXACT_LEAF_RECOMPUTATION"
                ),
                "mutated_field": field,
                "mutated_modes": list(mode_names),
                "mutated_counter": counter_name,
                "observed_counter_value": counter_value,
                "expected_counter_value": expected_count,
                "expected_decision_outcome": "REFUSED",
                "expected_named_fact": expected_fact,
                "observed_decision_outcome": outcome,
                "observed_named_fact": decision[
                    "named_first_exact_fact"
                ],
                "mode_agreement_preserved": (
                    mode_agreement_preserved
                ),
                "passed": (
                    outcome == "REFUSED"
                    and counter_value == expected_count
                    and decision["named_counter"] == counter_name
                    and decision["named_first_exact_fact"]
                    == expected_fact
                    and mode_agreement_preserved
                ),
            }
        )

    append_synthetic_assembler_check(
        "STALE_SYNTHETIC_CONTOUR_SIMPLE_ONE_MODE",
        "every_contour_is_simple",
        ("MOTORCYCLE",),
    )
    append_synthetic_assembler_check(
        "STALE_SYNTHETIC_CONTOUR_SIMPLE_BOTH_MODES",
        "every_contour_is_simple",
        ("MOTORCYCLE", "EXHAUSTIVE"),
    )
    append_synthetic_assembler_check(
        "STALE_SYNTHETIC_FACE_POSITIVE_ONE_MODE",
        "every_face_is_positive",
        ("MOTORCYCLE",),
    )
    append_synthetic_assembler_check(
        "STALE_SYNTHETIC_FACE_POSITIVE_BOTH_MODES",
        "every_face_is_positive",
        ("MOTORCYCLE", "EXHAUSTIVE"),
    )
    return {
        "all_passed": all(item["passed"] for item in checks),
        "checks": checks,
    }


def main() -> None:
    arguments = _arguments()
    fixture_base = Path(__file__).resolve().parent
    failures: list[dict[str, Any]] = []
    product_tree_oid = None
    receipt_hashes = {}
    accepted_a_payloads = {}
    accepted_receipt_seals = {}
    for name in ACCEPTED_RECEIPT_SHA256:
        path = fixture_base / name
        relative_path = path.relative_to(
            arguments.source_root
        ).as_posix()
        canonical_bytes, seal = _tracked_utf8_seal(
            arguments.source_root,
            path,
            f"HEAD:{relative_path}",
            "VALIDATED_HEAD_ACCEPTED_PROOF_BLOB",
        )
        receipt_hashes[name] = _sha256(canonical_bytes)
        accepted_a_payloads[name] = json.loads(canonical_bytes)
        accepted_receipt_seals[name] = seal
    accepted_a_proofs_ok = (
        all(
            receipt_hashes[name] == expected
            and _tracked_utf8_seal_exact(
                accepted_receipt_seals[name]
            )
            for name, expected in ACCEPTED_RECEIPT_SHA256.items()
        )
        and accepted_a_payloads[
            "gate_a_triple_prime_receipt.json"
        ].get("outcome")
        == "GATE_A_TRIPLE_PRIME_GO"
        and accepted_a_payloads[
            "gate_a_triple_prime_receipt.json"
        ].get("failures")
        == []
    )
    if not accepted_a_proofs_ok:
        failures.append(
            {
                "code": "UNKNOWN_ACCEPTED_A_PROOF",
                "detail": receipt_hashes,
            }
        )
    phase0_bytes = b""
    phase0_payload = {}
    phase0_history_ok = False
    try:
        phase0_bytes = _git_bytes(
            arguments.source_root,
            f"{PROOF_BASE_REVISION}:{PHASE0_RECEIPT_PATH}",
        )
        phase0_payload = json.loads(phase0_bytes)
        phase0_history_ok = (
            _sha256(phase0_bytes) == PHASE0_RECEIPT_SHA256
            and phase0_payload.get("outcome") == "H1_CONFIRMED"
            and phase0_payload.get("failures") == []
        )
    except Exception as error:
        failures.append(
            {
                "code": "PHASE0_HISTORY_NOT_SEALED",
                "detail": f"{type(error).__name__}: {error}",
            }
        )
    try:
        product_tree_oid = _git_value(arguments.source_root, "HEAD:kernel/src")
    except Exception as error:
        failures.append(
            {
                "code": "UNKNOWN_GIT_FORM",
                "detail": f"{type(error).__name__}: {error}",
            }
        )
    product_tree_ok = product_tree_oid == EXPECTED_KERNEL_SRC_TREE_OID
    if not product_tree_ok:
        failures.append(
            {
                "code": "UNKNOWN_KERNEL_SRC_PRODUCT_TREE",
                "detail": product_tree_oid,
                "expected": EXPECTED_KERNEL_SRC_TREE_OID,
            }
        )

    fixtures = []
    timings = []
    integrity = []
    synthetics = {}
    family = {
        "scope": "NOT_RUN",
        "polygon_count": 0,
        "execution_count": 0,
        "non_exact_count": 0,
        "first_non_exact_cases": [],
    }
    source_audit = {}
    accepted_a3 = accepted_a_payloads["gate_a_triple_prime_receipt.json"]
    accepted_fixture_hashes = {
        item["fixture_id"]: item["fixture_hash"]
        for item in accepted_a3["fixtures"]
    }
    present = {
        item.name
        for item in arguments.fixture_root.iterdir()
        if item.is_dir() and item.name in FIXTURE_IDS
    }
    fixture_form_ok = present == set(FIXTURE_IDS)
    if not fixture_form_ok:
        failures.append(
            {
                "code": "UNKNOWN_FIXTURE_SET",
                "detail": sorted(present),
                "expected": list(FIXTURE_IDS),
            }
        )

    forced_restoration = {"passed": False}
    different_speed_control = {"passed": False}
    h2_audit = {}
    filter_audit = {}
    gate_audit = {
        "outcome": "PHASE1_GATE_REFUSED",
        "failure_counters": {
            name: 0 for name, _ in PHASE1_FAILURE_ORDER
        },
    }
    negative_self_checks = {"all_passed": False, "checks": []}
    modules = None
    if not failures:
        modules = _activate_kernel(arguments.source_root)
        for fixture_id in FIXTURE_IDS:
            fixture_dir = arguments.fixture_root / fixture_id
            integrity_record, integrity_failures = _fixture_integrity(
                arguments.source_root,
                fixture_dir, accepted_fixture_hashes
            )
            integrity.append(integrity_record)
            failures.extend(integrity_failures)
        if not failures:
            for fixture_id in FIXTURE_IDS:
                try:
                    record, timing = _public_fixture_record_phase1(
                        modules,
                        arguments.source_root,
                        arguments.fixture_root / fixture_id,
                    )
                    fixtures.append(record)
                    timings.append(timing)
                except Exception as error:
                    failures.append(
                        {
                            "code": "UNKNOWN_FIXTURE_RUNTIME_FORM",
                            "detail": f"{type(error).__name__}: {error}",
                            "fixture_id": fixture_id,
                        }
                    )
                    break
        if not failures:
            synthetics = {
                "H1_REQUIRED": _synthetic_case_phase1(
                    modules, "H1_REQUIRED", H1_POINTS
                ),
                "H2_SCALE_CONTROL": _synthetic_case_phase1(
                    modules, "H2_SCALE_CONTROL", H2_POINTS
                ),
                "H1_SMALLER_CONCAVE": _synthetic_case_phase1(
                    modules, "H1_SMALLER_CONCAVE", SMALL_H1_POINTS
                ),
            }
            forced_restoration = _forced_exception_restoration_audit(
                modules["skeleton"]
            )
            different_speed_control = _different_speed_local_control(
                modules
            )
            family = _four_vertex_family(modules)
            source_audit = _source_predicate_audit(arguments.source_root)
            h2_audit = _phase1_h2_audit(
                fixtures, different_speed_control
            )
            filter_audit = _filter_span_audit(
                fixtures, synthetics, source_audit
            )
            gate_audit = _phase1_gate_audit(
                fixtures,
                synthetics,
                family,
                source_audit,
                h2_audit,
                filter_audit,
                forced_restoration,
                modules,
                product_tree_ok=product_tree_ok,
                fixture_form_ok=fixture_form_ok,
                verifier_schema_ok=(
                    SCHEMA
                    == "cftuv.envelope.sem_clb_02_gate_b_prime_phase1.v1"
                ),
                phase0_history_ok=phase0_history_ok,
                accepted_a_proofs_ok=accepted_a_proofs_ok,
            )
            negative_self_checks = _phase1_negative_self_checks(
                gate_audit,
                fixtures,
                synthetics,
                family,
                source_audit,
                different_speed_control,
                forced_restoration,
                modules,
            )
            gate_audit["failure_counters"][
                "negative_self_check_failures"
            ] = int(not negative_self_checks["all_passed"])
            gate_audit["outcome"] = (
                "ALL_PHASE1_GATES_SATISFIED"
                if not any(gate_audit["failure_counters"].values())
                else "PHASE1_GATE_REFUSED"
            )

    if failures:
        first = failures[0]["code"]
        mapping = {
            "UNKNOWN_KERNEL_SRC_PRODUCT_TREE": (
                "unknown_kernel_src_product_tree"
            ),
            "UNKNOWN_FIXTURE_SET": "unknown_fixture_form",
            "UNKNOWN_ACCEPTED_A_PROOF": "accepted_a_proof_seal_failures",
            "FIXTURE_TRACKED_BYTE_SEAL_MISMATCH": (
                "source_byte_seal_failures"
            ),
            "PHASE0_HISTORY_NOT_SEALED": "phase0_history_not_sealed",
        }
        gate_audit["failure_counters"][
            mapping.get(first, "public_exactness_failures")
        ] = 1

    outcome, decision = _phase1_decide(gate_audit)
    phase0_observation_history = {
        "accepted_commit": PROOF_BASE_REVISION,
        "receipt_path": PHASE0_RECEIPT_PATH,
        "expected_sha256": PHASE0_RECEIPT_SHA256,
        "observed_sha256": _sha256(phase0_bytes),
        "unchanged": phase0_history_ok,
        "outcome": phase0_payload.get("outcome"),
        "decision": phase0_payload.get("decision"),
        "h2_evidence_outcome": phase0_payload.get(
            "h2_evidence_audit", {}
        ).get("outcome"),
        "filter_orthogonality_outcome": phase0_payload.get(
            "filter_span_is_born_zero_audit", {}
        ).get("outcome"),
        "negative_self_checks_all_passed": phase0_payload.get(
            "negative_self_checks", {}
        ).get("all_passed"),
    }
    legacy_behavior_seal = {
        "scope": "UNCHANGED_BY_CONSTRUCTION_NO_PRODUCT_BYTES_EDITED",
        "bf6_point_contact": {
            "basis": "ACCEPTED_A_PROOFS_AND_IDENTICAL_KERNEL_SRC_TREE",
            "sealed": accepted_a_proofs_ok and product_tree_ok,
        },
        "V1_binding": {
            "prior_v1_binding_failures": accepted_a3["aggregate"][
                "canonical_gram_search_diagnostic_counters"
            ]["prior_v1_binding_failures"],
            "sealed": (
                accepted_a3["aggregate"][
                    "canonical_gram_search_diagnostic_counters"
                ]["prior_v1_binding_failures"]
                == 0
                and accepted_a_proofs_ok
                and product_tree_ok
            ),
        },
        "G1_full_selection": {
            "accepted_A3_all_hypotheses_match": accepted_a3["aggregate"][
                "all_hypotheses_match"
            ],
            "sealed": (
                accepted_a3["aggregate"]["all_hypotheses_match"]
                and accepted_a_proofs_ok
                and product_tree_ok
            ),
        },
    }
    payload = {
        "schema": SCHEMA,
        "phase": "GATE_B_PRIME_PHASE_1_PROOF_LOCAL_CANDIDATE",
        "outcome": outcome,
        "proof_base_revision": PROOF_BASE_REVISION,
        "phase0_observation_history": phase0_observation_history,
        "kernel_src_product_tree_oid": product_tree_oid,
        "accepted_kernel_src_product_tree_oid": (
            EXPECTED_KERNEL_SRC_TREE_OID
        ),
        "accepted_receipts": {
            name: {
                "expected_sha256": expected,
                "observed_sha256": receipt_hashes[name],
                "unchanged": (
                    receipt_hashes[name] == expected
                    and _tracked_utf8_seal_exact(
                        accepted_receipt_seals[name]
                    )
                ),
                "tracked_byte_seal": _compact_tracked_utf8_seal(
                    accepted_receipt_seals[name]
                ),
            }
            for name, expected in ACCEPTED_RECEIPT_SHA256.items()
        },
        "legacy_behavior_seal": legacy_behavior_seal,
        "verifier_schema_seal": {
            "schema": SCHEMA,
            "verifier_sha256": source_audit.get(
                "phase1_verifier_sha256"
            ),
            "sealed": (
                SCHEMA
                == "cftuv.envelope.sem_clb_02_gate_b_prime_phase1.v1"
            ),
        },
        "timing_capture": {
            "captured_to_process_stdout": True,
            "excluded_from_canonical_receipt": True,
            "reason": "WALL_CLOCK_TIMINGS_ARE_NONDETERMINISTIC",
        },
        "capture_adapter": {
            "scope": "PROOF_LOCAL_SEED_VERTICES_ONLY",
            "operation": (
                "APPLY_EXISTING_EVENT_BORN_EXACT_STRAIGHT_CLASSIFIER_"
                "TO_SEED_VERTICES_WITHOUT_MOVEMENT_OR_RESNAPPING"
            ),
            "candidate_rule_applied": True,
            "new_event_rule_applied": False,
            "filter_reused": False,
            "product_or_fixture_state_mutated": False,
            "forced_exception_restoration": forced_restoration,
        },
        "hypotheses": {
            "H1": (
                "A_CERTIFIED_STRAIGHT_SEED_VERTEX_BETWEEN_COLLINEAR_"
                "CODIRECTIONAL_EQUAL_SPEED_SUPPORTS_IS_ONE_SLIDING_"
                "MOVING_LINE_AND_HAS_NO_TWO_LINE_BISECTOR"
            ),
            "H2": (
                "ONE_SUPPORT_LINE_CLASS_MAY_HAVE_MULTIPLE_INTEGER_NORMAL_"
                "REPRESENTATIONS_BUT_EQUAL_Q_OVER_N"
            ),
        },
        "decision": decision,
        "phase1_gate_audit": gate_audit,
        "source_predicate_audit": source_audit,
        "h2_evidence_audit": h2_audit,
        "filter_span_is_born_zero_audit": filter_audit,
        "negative_self_checks": negative_self_checks,
        "four_vertex_family_boundary": family,
        "synthetic_cases": synthetics,
        "fixture_integrity": integrity,
        "fixtures": fixtures,
        "failures": failures,
    }
    report_bytes = _pretty_json(payload)
    arguments.output_report.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_report.write_bytes(report_bytes)
    print(
        json.dumps(
            {
                "outcome": outcome,
                "report_sha256": _sha256(report_bytes),
                "report_bytes": len(report_bytes),
                "fixture_count": len(fixtures),
                "phase1_failure_counters": gate_audit[
                    "failure_counters"
                ],
                "h2_evidence_outcome": h2_audit.get("outcome"),
                "filter_orthogonality_outcome": filter_audit.get(
                    "outcome"
                ),
                "negative_self_checks": {
                    item["name"]: item["passed"]
                    for item in negative_self_checks["checks"]
                },
                "certified_straight_seed_counts": {
                    item["fixture_id"]: item["region"]["seed"][
                        "certified_straight_seed_vertex_count"
                    ]
                    for item in fixtures
                },
                "filter_span_is_born_zero_counts": {
                    item["fixture_id"]: item["region"]["terminal"][
                        "named_candidate_rejection_counters"
                    ]["refused_filter_span_is_born_zero"]
                    for item in fixtures
                },
                "synthetic_outcomes": {
                    name: {
                        mode: result["terminal"]["outcome"]
                        for mode, result in case["modes"].items()
                    }
                    for name, case in synthetics.items()
                },
                "timings_noncanonical": timings,
            },
            ensure_ascii=False,
            sort_keys=True,
        ),
        flush=True,
    )
    raise SystemExit(0 if outcome == "GATE_B_PRIME_GO" else 2)


if __name__ == "__main__":
    main()
