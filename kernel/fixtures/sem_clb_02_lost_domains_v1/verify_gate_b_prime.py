"""Proof-only exact seed-joint audit for SEM-CLB-02-R Gate B-prime.

The verifier observes the frozen product without changing an event rule.  It
captures the public Reference/QUEUE path, exact seed and terminal LAV state,
and bounded synthetic controls.  Runtime timings are printed but deliberately
excluded from the canonical receipt.
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


SCHEMA = "cftuv.envelope.sem_clb_02_gate_b_prime_phase0.v1"
PROOF_BASE_REVISION = "e04760106a310cfedd2794888f7e41d368ef59e9"
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


def _file_sha256(path: Path) -> str:
    return _sha256(path.read_bytes())


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


def _activate_kernel(source_root: Path):
    kernel_src = source_root / "kernel" / "src"
    sys.path.insert(0, str(kernel_src))
    import cftuv_envelope as kernel
    import cftuv_envelope.wavefront.conveyor as conveyor
    import cftuv_envelope.wavefront.skeleton as skeleton_module
    from cftuv_envelope.wavefront.bridge import line_class
    from cftuv_envelope.wavefront.faces import build_faces
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
        "LoopV1": LoopV1,
        "PolygonV1": PolygonV1,
        "signed_double_area": signed_double_area,
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
            "speeds_squared": [_ratio(item) for item in loop.speeds_squared],
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
    fixture_dir: Path,
    accepted_fixture_hashes: dict[str, str],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    failures = []
    manifest_path = fixture_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    fixture_id = fixture_dir.name
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
    for name, expected in sorted(manifest["files"].items()):
        observed = _file_sha256(fixture_dir / name)
        observed_files[name] = observed
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
    }, failures


def _public_fixture_record(
    modules: dict[str, Any],
    fixture_dir: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    kernel = modules["kernel"]
    conveyor = modules["conveyor"]
    skeleton_module = modules["skeleton"]
    line_class = modules["line_class"]
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (fixture_dir / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (fixture_dir / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains

    compile_started = time.perf_counter()
    compile_result = kernel.compile_reference_envelopes(
        snapshot, request, domain.patch_domain_id
    )
    compile_seconds = time.perf_counter() - compile_started

    captures: list[dict[str, Any]] = []
    original_build_skeleton = conveyor.build_skeleton

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
                "result": result,
            }
        )
        return result

    prepare_started = time.perf_counter()
    conveyor.build_skeleton = capturing_build_skeleton
    try:
        prepared = conveyor.prepare_conveyor(
            snapshot, request, patch_domain_id=domain.patch_domain_id
        )
    finally:
        conveyor.build_skeleton = original_build_skeleton
    prepare_seconds = time.perf_counter() - prepare_started

    coverage_started = time.perf_counter()
    coverage = conveyor.conveyor_coverage(prepared)
    coverage_seconds = time.perf_counter() - coverage_started

    if len(captures) != 1 or len(prepared.regions) != 1:
        raise ValueError("expected exactly one captured sparse domain region")
    capture = captures[0]
    region = prepared.regions[0]
    if region.skeleton != capture["result"]:
        raise ValueError("capture adapter changed the public skeleton result")
    if region.bridge.polygon is None:
        raise ValueError("public bridge did not produce a polygon")
    if _polygon_record(region.bridge.polygon) != capture["polygon"]:
        raise ValueError("captured polygon differs from public bridge polygon")

    manifest = json.loads(
        (fixture_dir / "manifest.json").read_text(encoding="utf-8")
    )
    historical = _historical_summary(manifest)
    current_skeleton_counters = dict(region.skeleton.counters)
    current_prepare_counters = dict(prepared.counters)
    binding = compile_result.compilation.evaluation_geometry_binding
    current = {
        "fixture_id": fixture_dir.name,
        "patch_domain_id": domain.patch_domain_id.value,
        "compile": {
            "outcome": compile_result.outcome.value,
            "compilation_available": compile_result.compilation is not None,
            "diagnostics": [
                item.outcome.value for item in compile_result.diagnostics
            ],
            "binding_type": type(binding).__name__,
            "base_lattice_scale_S": binding.base_lattice_scale,
            "refinement_power_r": binding.refinement_power,
            "domain_lattice_scale_S_prime": binding.lattice_scale,
        },
        "prepare": {
            "outcome": prepared.outcome.value,
            "detail": prepared.detail,
            "lattice_scale": prepared.lattice.scale,
            "counters": current_prepare_counters,
            "reported_timing_stage_names": [
                name for name, _ in prepared.timings
            ],
        },
        "coverage": {"outcome": coverage.outcome.value},
        "region": {
            "region_id": region.region_id,
            "bridge_outcome": region.bridge_outcome.value,
            "bridge_polygon": capture["polygon"],
            "seed": capture["seed"],
            "terminal": capture["terminal"],
            "face_outcome": (
                None
                if region.face_outcome is None
                else region.face_outcome.value
            ),
            "face_count": (
                0 if region.partition is None else len(region.partition.faces)
            ),
        },
        "historical_28f6_product_tree_summary": historical,
        "deltas_from_historical_28f6_product_tree": {
            "lattice_scale": (
                prepared.lattice.scale - historical["lattice_scale"]
            ),
            "skeleton_levels": (
                region.skeleton.levels - historical["skeleton_levels"]
            ),
            "skeleton_node_count": (
                len(region.skeleton.nodes)
                - historical["skeleton_node_count"]
            ),
            "face_count": (
                (0 if region.partition is None else len(region.partition.faces))
                - historical["face_count"]
            ),
            "prepare_counters": _counter_delta(
                current_prepare_counters,
                historical["prepare_counters"],
            ),
            "skeleton_counters": _counter_delta(
                current_skeleton_counters,
                historical["skeleton_counters"],
            ),
        },
    }
    timing = {
        "fixture_id": fixture_dir.name,
        "compile_seconds": compile_seconds,
        "prepare_seconds": prepare_seconds,
        "coverage_seconds": coverage_seconds,
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
    path = (
        source_root
        / "kernel"
        / "src"
        / "cftuv_envelope"
        / "wavefront"
        / "skeleton.py"
    )
    tree = ast.parse(path.read_text(encoding="utf-8"))
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
    return {
        "event_born_new_vertex_predicate": predicate,
        "expected_predicate": EXPECTED_NEW_VERTEX_PREDICATE,
        "predicate_exact_match": predicate == EXPECTED_NEW_VERTEX_PREDICATE,
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


def _filter_span_audit(
    fixtures: list[dict[str, Any]],
    synthetics: dict[str, dict[str, Any]],
    source_audit: dict[str, Any],
) -> dict[str, Any]:
    counters = {
        "field_born_zero_fan_count_mismatches": 0,
        "required_control_fan_edges": 0,
        "required_control_born_zero_filters": 0,
        "certified_seed_non_always_concurrent_adjacencies": 0,
        "code_path_order_unproven": 0,
    }
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
            counters[
                "certified_seed_non_always_concurrent_adjacencies"
            ] += sum(
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


def main() -> None:
    arguments = _arguments()
    fixture_base = Path(__file__).resolve().parent
    failures: list[dict[str, Any]] = []
    product_tree_oid = None
    receipt_hashes = {
        name: _file_sha256(fixture_base / name)
        for name in ACCEPTED_RECEIPT_SHA256
    }
    for name, expected in ACCEPTED_RECEIPT_SHA256.items():
        if receipt_hashes[name] != expected:
            failures.append(
                {
                    "code": "UNKNOWN_ACCEPTED_RECEIPT",
                    "detail": name,
                    "expected": expected,
                    "observed": receipt_hashes[name],
                }
            )
    try:
        product_tree_oid = _git_value(arguments.source_root, "HEAD:kernel/src")
        if product_tree_oid != EXPECTED_KERNEL_SRC_TREE_OID:
            failures.append(
                {
                    "code": "UNKNOWN_KERNEL_SRC_PRODUCT_TREE",
                    "detail": product_tree_oid,
                    "expected": EXPECTED_KERNEL_SRC_TREE_OID,
                }
            )
    except Exception as error:
        failures.append(
            {
                "code": "UNKNOWN_GIT_FORM",
                "detail": f"{type(error).__name__}: {error}",
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
    accepted_a3 = json.loads(
        (fixture_base / "gate_a_triple_prime_receipt.json").read_text(
            encoding="utf-8"
        )
    )
    accepted_fixture_hashes = {
        item["fixture_id"]: item["fixture_hash"]
        for item in accepted_a3["fixtures"]
    }
    present = {
        item.name
        for item in arguments.fixture_root.iterdir()
        if item.is_dir() and item.name in FIXTURE_IDS
    }
    if present != set(FIXTURE_IDS):
        failures.append(
            {
                "code": "UNKNOWN_FIXTURE_SET",
                "detail": sorted(present),
                "expected": list(FIXTURE_IDS),
            }
        )

    if not failures:
        modules = _activate_kernel(arguments.source_root)
        for fixture_id in FIXTURE_IDS:
            fixture_dir = arguments.fixture_root / fixture_id
            integrity_record, integrity_failures = _fixture_integrity(
                fixture_dir, accepted_fixture_hashes
            )
            integrity.append(integrity_record)
            failures.extend(integrity_failures)
        if not failures:
            for fixture_id in FIXTURE_IDS:
                try:
                    record, timing = _public_fixture_record(
                        modules, arguments.fixture_root / fixture_id
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
                "H1_REQUIRED": _synthetic_case(
                    modules, "H1_REQUIRED", H1_POINTS
                ),
                "H2_SCALE_CONTROL": _synthetic_case(
                    modules, "H2_SCALE_CONTROL", H2_POINTS
                ),
                "H1_SMALLER_CONCAVE": _synthetic_case(
                    modules, "H1_SMALLER_CONCAVE", SMALL_H1_POINTS
                ),
            }
            family = _four_vertex_family(modules)
            source_audit = _source_predicate_audit(arguments.source_root)

    h2_audit = (
        {}
        if failures
        else _h2_evidence_audit(fixtures, synthetics)
    )
    filter_audit = (
        {}
        if failures
        else _filter_span_audit(fixtures, synthetics, source_audit)
    )
    negative_self_checks = (
        {"all_passed": False, "checks": []}
        if failures
        else _negative_self_checks(
            fixtures, synthetics, source_audit, family
        )
    )
    outcome, decision = _decide(
        fixtures,
        synthetics,
        family,
        source_audit,
        h2_audit,
        filter_audit,
        negative_self_checks,
        failures,
    )
    payload = {
        "schema": SCHEMA,
        "phase": "GATE_B_PRIME_PHASE_0_PROOF_ONLY",
        "outcome": outcome,
        "proof_base_revision": PROOF_BASE_REVISION,
        "kernel_src_product_tree_oid": product_tree_oid,
        "accepted_kernel_src_product_tree_oid": (
            EXPECTED_KERNEL_SRC_TREE_OID
        ),
        "accepted_receipts": {
            name: {
                "expected_sha256": expected,
                "observed_sha256": receipt_hashes[name],
                "unchanged": receipt_hashes[name] == expected,
            }
            for name, expected in ACCEPTED_RECEIPT_SHA256.items()
        },
        "timing_capture": {
            "captured_to_process_stdout": True,
            "excluded_from_canonical_receipt": True,
            "reason": "WALL_CLOCK_TIMINGS_ARE_NONDETERMINISTIC",
        },
        "capture_adapter": {
            "scope": "OBSERVATION_ONLY",
            "operation": (
                "PUBLIC_PREPARE_USES_THE_UNMODIFIED__Builder.run_RESULT_"
                "WHILE_THE_PROOF_RETAINS_ITS_SEED_AND_TERMINAL_STATE"
            ),
            "candidate_rule_applied": False,
            "product_or_fixture_state_mutated": False,
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
                "fixture_count": len(fixtures),
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
    raise SystemExit(0 if outcome != "REFUSED" else 2)


if __name__ == "__main__":
    main()
