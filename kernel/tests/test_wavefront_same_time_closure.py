from __future__ import annotations

from collections import defaultdict
from dataclasses import replace
from fractions import Fraction
import hashlib
import json

import cftuv_envelope.wavefront.skeleton as skeleton_module
import pytest
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.event_time import compare_times
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.events import EventKind
from cftuv_envelope.wavefront.skeleton import build_skeleton
from cftuv_envelope.wavefront.superlevel import accumulate_nodes
from test_wavefront_proof_obligations import (
    _CASES,
    _GEOMETRY_ORACLE,
    _LEGACY_COUNTER_NAMES,
    _P0_2_MERGED_DIGESTS,
    _PROOF_STATUS_ORACLE,
)


# Literal snapshot с точного base 8d93e79. Каждая запись хранит:
# sha256 канонического `(time, point)`, исходные kinds, объединение участников
# до P0-2 и исходные значения `converging_vertices` двух записей.
_DUPLICATE_ORACLE = {
    "partial_source::ell_12_source_edges_0_1": (
        "ebe2413cad452739f49ed77e09f32f4815cc28368a3ef6cc05691a575e558ee1",
        ("EDGE", "SPLIT"),
        ((0, 0, 12, 0), (6, 6, 6, 12), (12, 0, 12, 6), (12, 6, 6, 6)),
        (1, 2),
    ),
    "partial_source::ell_12_source_edges_1_2": (
        "0b44430b6636261e6a84ef4998d56026c6a6d75cb66842cff75843196b24913f",
        ("EDGE", "SPLIT"),
        ((0, 0, 12, 0), (6, 6, 6, 12), (12, 0, 12, 6), (12, 6, 6, 6)),
        (1, 2),
    ),
    "partial_source::ell_12_source_edges_3_4": (
        "8f1b33597c3cbac5dc482e09bb2dc699eb9ad110fddf94cae58e62fc963046da",
        ("EDGE", "SPLIT"),
        ((0, 12, 0, 0), (6, 6, 6, 12), (6, 12, 0, 12), (12, 6, 6, 6)),
        (1, 2),
    ),
    "partial_source::ell_12_source_edges_4_5": (
        "ebe2413cad452739f49ed77e09f32f4815cc28368a3ef6cc05691a575e558ee1",
        ("EDGE", "SPLIT"),
        ((0, 12, 0, 0), (6, 6, 6, 12), (6, 12, 0, 12), (12, 6, 6, 6)),
        (1, 2),
    ),
    "partial_source::staircase_source_edges_0_1": (
        "958962addde340488ba70caeeba1fcb261b1f1fa8ce8ae95f79029f4cf827554",
        ("EDGE", "SPLIT"),
        ((0, 0, 12, 0), (8, 4, 8, 8), (12, 0, 12, 4), (12, 4, 8, 4)),
        (1, 2),
    ),
    "partial_source::staircase_source_edges_1_2": (
        "1e88b5ff20ad84fcae7222aac35f68083ffabb4797cc9ab80566eaac4e2b7607",
        ("EDGE", "SPLIT"),
        ((0, 0, 12, 0), (8, 4, 8, 8), (12, 0, 12, 4), (12, 4, 8, 4)),
        (1, 2),
    ),
    "partial_source::staircase_source_edges_2_3": (
        "6176adfec0d06b4bd17b65c453c879673889d4954c8410b316be54296d913ec6",
        ("EDGE", "SPLIT"),
        ((0, 0, 12, 0), (4, 8, 4, 12), (8, 4, 8, 8), (12, 4, 8, 4)),
        (1, 2),
    ),
    "partial_source::staircase_source_edges_3_4": (
        "0f772552877b39bac371c34fca3dc663c6fa3c9156c6c33734b30a7e957962cc",
        ("SPLIT", "SPLIT"),
        ((4, 8, 4, 12), (8, 4, 8, 8), (8, 8, 4, 8), (12, 4, 8, 4)),
        (1, 1),
    ),
    "partial_source::staircase_source_edges_5_6": (
        "98a7a18ad3edd996edc0016db5b4c0f7be4409dcc10ef943154fa2cdbee93638",
        ("EDGE", "SPLIT"),
        ((0, 12, 0, 0), (4, 8, 4, 12), (4, 12, 0, 12), (8, 8, 4, 8)),
        (1, 2),
    ),
    "partial_source::staircase_source_edges_6_7": (
        "52864ee7309f7d3d7c3b52055b725c87bc34528ce8b036a45dd02407acefa0ec",
        ("EDGE", "SPLIT"),
        ((0, 12, 0, 0), (4, 8, 4, 12), (4, 12, 0, 12), (8, 8, 4, 8)),
        (1, 2),
    ),
    "partial_source::ell_12_source_without_the_reflex_pair": (
        "ebe2413cad452739f49ed77e09f32f4815cc28368a3ef6cc05691a575e558ee1",
        ("EDGE", "SPLIT"),
        (
            (0, 0, 12, 0),
            (0, 12, 0, 0),
            (6, 6, 6, 12),
            (6, 12, 0, 12),
            (12, 0, 12, 6),
            (12, 6, 6, 6),
        ),
        (1, 5),
    ),
}


# Полный projection: proof obligations, face partition и coverage в трёх alpha.
# Он обязан остаться тем же после слияния node records в фазе B.
_INVARIANT_SHA256 = {
    "partial_source::ell_12_source_edges_0_1": "a2c262df5f504573a70a5680397dc8d24a25adf6c9a85ee730107901e422c5de",
    "partial_source::ell_12_source_edges_1_2": "90b0a734954f6d2424b27270f7adec32bd419f58988f7ef3edcd59b0404d24c3",
    "partial_source::ell_12_source_edges_3_4": "3c2c57fdca5ef1cbb771d4acea449cf0280bf2e0a730c82924e89b91f3149e91",
    "partial_source::ell_12_source_edges_4_5": "1640877015e81c138109c44f71e87ab8c1786cdd1d5f6d2258353f2fc34773a0",
    "partial_source::ell_12_source_without_the_reflex_pair": "695e07fb270dfb513759d1524015d3ac87c1d5f14489137808074673604155eb",
    "partial_source::staircase_source_edges_0_1": "fcc283386b16124c9f37d7342cedb8de4b3b2bab3720e48b0a686784ad094438",
    "partial_source::staircase_source_edges_1_2": "f179169b7db4d0f7df04ddb326168b8f058b8930b7727d6dc291f1e6c193737b",
    "partial_source::staircase_source_edges_2_3": "b6ba4ff176903c0bda6a23a88fd675e38ff891f6695dcfc027ba1d11c7691242",
    "partial_source::staircase_source_edges_3_4": "5151ba03d02dad0f76864e21d838bd8b2bd7db9b52173e76e4093a63f36ba2e1",
    "partial_source::staircase_source_edges_5_6": "1720f84aaf26b782f676d91a61178658ffa12a9236535dd1f21c2a1ae723b2e5",
    "partial_source::staircase_source_edges_6_7": "79f771ff87e44d7d427498b4df75fe00bba8069e19358440eeb9cfd9bd6e46c5",
}


def _sqrt_sum_record(value):
    return [
        [radicand, [coefficient.numerator, coefficient.denominator]]
        for radicand, coefficient in value.terms
    ]


def _fraction_record(value):
    return [value.numerator, value.denominator]


def _number_record(value):
    return _fraction_record(value) if isinstance(value, Fraction) else value


def _point_record(point):
    return [_sqrt_sum_record(point[0]), _sqrt_sum_record(point[1])]


def _time_record(value):
    canonical = value.canonical()
    return [
        _fraction_record(canonical.dividend),
        _sqrt_sum_record(canonical.divisor),
    ]


def _partition_record(partition):
    return {
        "outcome": partition.outcome.value,
        "detail": partition.detail,
        "area": _sqrt_sum_record(partition.doubled_area),
        "polygon_area": partition.polygon_doubled_area,
        "faces": [
            {
                "owner": [_number_record(item) for item in face.owner],
                "start": face.source_start,
                "end": face.source_end,
                "points": [_point_record(point) for point in face.points],
                "area": _sqrt_sum_record(face.doubled_area),
                "line": None
                if face.line is None
                else [
                    face.line.a,
                    face.line.b,
                    face.line.c,
                    _number_record(face.line.q),
                ],
            }
            for face in partition.faces
        ],
    }


def _coverage_record(coverage):
    return {
        "outcome": coverage.outcome.value,
        "alpha": _fraction_record(coverage.alpha),
        "detail": coverage.detail,
        "area": _sqrt_sum_record(coverage.doubled_area),
        "polygon_area": coverage.polygon_doubled_area,
        "faces": [
            {
                "owner": [_number_record(item) for item in face.owner],
                "points": [_point_record(point) for point in face.points],
                "area": _sqrt_sum_record(face.doubled_area),
            }
            for face in coverage.faces
        ],
    }


def _obligation_record(obligation):
    return {
        "cause": obligation.cause.value,
        "disposition": obligation.disposition.value,
        "vertices": obligation.vertex_ids,
        "participants": obligation.participant_edge_keys,
        "targets": obligation.target_edge_keys,
        "level": _time_record(obligation.level),
        "kind": None
        if obligation.event_kind is None
        else obligation.event_kind.value,
    }


def _invariant_digest(polygon, skeleton):
    partition = build_faces(polygon, skeleton)
    payload = {
        "outcome": skeleton.outcome.value,
        "proof_status": skeleton.proof_status.value,
        "proof": [
            _obligation_record(obligation)
            for obligation in skeleton.proof_obligations
        ],
        "partition": _partition_record(partition),
        "coverage": [
            _coverage_record(coverage_at(partition, alpha))
            for alpha in (Fraction(1, 4), Fraction(1), Fraction(3))
        ],
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _duplicate_projection(skeleton):
    groups = defaultdict(list)
    for node in skeleton.nodes:
        record = node_record(node)
        key = json.dumps(
            [
                record["time_dividend"],
                record["time_divisor"],
                record["point_x"],
                record["point_y"],
            ],
            sort_keys=True,
            separators=(",", ":"),
        )
        groups[key].append(record)
    duplicated = []
    for key, records in groups.items():
        if len(records) <= 1:
            continue
        participants = tuple(
            sorted(
                {
                    tuple(participant)
                    for record in records
                    for participant in record["participants"]
                }
            )
        )
        duplicated.append(
            (
                hashlib.sha256(key.encode("utf-8")).hexdigest(),
                tuple(sorted(record["kind"] for record in records)),
                participants,
                tuple(
                    sorted(record["converging_vertices"] for record in records)
                ),
            )
        )
    return tuple(sorted(duplicated))


def test_phase_b_preserves_nonduplicate_digests_and_all_legacy_axes():

    assert len(_CASES) == 63
    for case_id, polygon in _CASES:
        expected_outcome, expected_digest, expected_counters = _GEOMETRY_ORACLE[
            case_id
        ]
        skeleton = build_skeleton(polygon)
        counters = dict(skeleton.counters)
        assert skeleton.outcome.value == expected_outcome
        if case_id not in _DUPLICATE_ORACLE:
            assert semantic_digest(skeleton) == expected_digest
        assert tuple(counters[name] for name in _LEGACY_COUNTER_NAMES) == (
            expected_counters
        )
        assert skeleton.proof_status.value == _PROOF_STATUS_ORACLE[case_id]


def test_phase_b_merges_exactly_11_duplicate_cases_and_preserves_invariants():
    merged = {}
    for case_id, polygon in _CASES:
        skeleton = build_skeleton(polygon)
        assert _duplicate_projection(skeleton) == ()
        multiway = [node for node in skeleton.nodes if node.kind is EventKind.MULTIWAY]
        if multiway:
            assert len(multiway) == 1
            node = multiway[0]
            record = node_record(node)
            key = json.dumps(
                [
                    record["time_dividend"],
                    record["time_divisor"],
                    record["point_x"],
                    record["point_y"],
                ],
                sort_keys=True,
                separators=(",", ":"),
            )
            merged[case_id] = (
                hashlib.sha256(key.encode("utf-8")).hexdigest(),
                tuple(record["kinds"]),
                tuple(tuple(item) for item in record["participants"]),
                record["converging_vertices"],
            )
            assert _invariant_digest(polygon, skeleton) == _INVARIANT_SHA256[case_id]
    assert set(merged) == set(_DUPLICATE_ORACLE)
    for case_id, (key, kinds, participants, converging) in merged.items():
        before = _DUPLICATE_ORACLE[case_id]
        assert key == before[0]
        assert kinds == tuple(sorted(set(before[1])))
        assert participants == before[2]
        assert converging == sum(before[3])


def test_phase_b_accumulator_keeps_independent_coincident_components():
    base = build_skeleton(dict(_CASES)["named::axis_square"]).nodes[0]
    shared = base.participants[0]
    second = replace(
        base,
        kind=EventKind.SPLIT,
        participants=(shared, (100, 0, 101, 0)),
        converging_vertices=2,
    )
    independent = replace(
        base,
        participants=((200, 0, 201, 0),),
        converging_vertices=1,
    )
    nodes = accumulate_nodes(
        (base, second, independent),
        ((1, 2), (2, 3), (4,)),
    )

    assert len(nodes) == 2
    merged = next(node for node in nodes if node.kind is EventKind.MULTIWAY)
    assert merged.kinds == (EventKind.EDGE, EventKind.SPLIT)
    assert merged.converging_vertices == 3
    assert set(merged.participants) == set(base.participants) | set(
        second.participants
    )
    assert independent in nodes


def test_phase_b_accumulator_flattens_two_pass_original_incidences():
    skeleton = build_skeleton(dict(_CASES)["named::axis_square"])
    base = skeleton.nodes[0]
    a = (10, 0, 11, 0)
    b = (20, 0, 21, 0)
    c = (30, 0, 31, 0)
    d = (40, 0, 41, 0)
    ab = replace(base, participants=(a, b), converging_vertices=2)
    bc = replace(
        base,
        kind=EventKind.SPLIT,
        participants=(b, c),
        converging_vertices=2,
    )
    cd = replace(base, participants=(c, d), converging_vertices=2)

    first = accumulate_nodes((ab, bc), ((1, 2), (2, 3)))[0]
    merged = accumulate_nodes((first, cd), ((1, 2, 3), (3, 4)))[0]
    assert merged.participants == (a, b, c, d)
    assert merged.incidences == ((a, b), (b, c), (c, d))
    assert merged.kinds == (EventKind.EDGE, EventKind.SPLIT)
    assert merged.converging_vertices == 4

    mutated = replace(merged, incidences=((a, b), (a, c), (c, d)))
    assert node_record(mutated) != node_record(merged)
    assert semantic_digest(replace(skeleton, nodes=(mutated,))) != (
        semantic_digest(replace(skeleton, nodes=(merged,)))
    )
    with pytest.raises(
        ValueError, match="MULTIWAY_NODE_INCIDENCE_UNION_MISMATCH"
    ):
        node_record(replace(merged, incidences=((a, b),)))
    with pytest.raises(ValueError, match="MULTIWAY_NODE_KINDS_NOT_CANONICAL"):
        node_record(
            replace(merged, kinds=(EventKind.SPLIT, EventKind.EDGE))
        )


def test_phase_b_consumers_refuse_unannotated_multiway_by_name():
    polygon = dict(_CASES)["named::axis_square"]
    skeleton = build_skeleton(polygon)
    invalid = replace(
        skeleton.nodes[0],
        kind=EventKind.MULTIWAY,
        kinds=(),
        incidences=(),
    )

    with pytest.raises(ValueError, match="MULTIWAY_NODE_KINDS_UNAVAILABLE"):
        node_record(invalid)
    partition = build_faces(polygon, replace(skeleton, nodes=(invalid,)))
    assert partition.outcome is FaceOutcome.MULTIWAY_NODE_INCIDENCE_UNAVAILABLE


def test_phase_b_duplicate_counters_are_zero_after_accumulation():
    duplicate_cases = []
    mixed_cases = []
    for case_id, polygon in _CASES:
        skeleton = build_skeleton(polygon)
        duplicate_count = skeleton.counter("duplicate_exact_time_point_nodes")
        mixed_count = skeleton.counter("mixed_kind_exact_time_point_nodes")
        assert duplicate_count == 0
        assert mixed_count == 0
        if duplicate_count:
            duplicate_cases.append(case_id)
        if mixed_count:
            mixed_cases.append(case_id)
    assert duplicate_cases == []
    assert mixed_cases == []


def test_phase_c_drains_every_superlevel_without_changing_b_digests():
    for case_id, polygon in _CASES:
        skeleton = build_skeleton(polygon)
        assert skeleton.counter("same_time_residual_after_level") == 0
        assert semantic_digest(skeleton) == _P0_2_MERGED_DIGESTS.get(
            case_id, _GEOMETRY_ORACLE[case_id][1]
        )


def test_phase_zero_cross_same_time_residual_semantics(monkeypatch):
    """Counters имеют разные знаменатели и поэтому не дублируют друг друга.

    `same_time_events_enqueued_during_level` считает СОБЫТИЯ, добавленные во
    время `_apply_level` с тем же exact time. На base cross это 4.
    `same_time_residual_after_level` считает ЗАВЕРШЁННЫЕ processing units, после
    которых в очереди остался хотя бы один exact-time event. На base это 2.
    Размеры остатка по шести пакетам заморожены отдельно: `[2, 0, 2, 0, 0, 0]`.
    После fixed-point drain фаза C обязана свести второй counter к нулю, не
    переписывая историческое число найденных same-time events.
    """

    original = skeleton_module._Builder._apply_level
    measurements = []

    def measured(builder, level):
        pushed_before = builder.queue.pushed
        original(builder, level)
        residual = tuple(
            entry
            for entry in builder.queue._heap
            if compare_times(entry.event.time, level[0].time) == 0
        )
        newly_enqueued = tuple(
            entry for entry in residual if entry.sequence >= pushed_before
        )
        measurements.append((len(newly_enqueued), len(residual)))

    monkeypatch.setattr(skeleton_module._Builder, "_apply_level", measured)
    cross = dict(_CASES)["named::cross"]
    skeleton = build_skeleton(cross)

    assert [residual for _, residual in measurements] == [2, 0, 2, 0, 0, 0]
    assert sum(enqueued for enqueued, _ in measurements) == 4
    assert sum(residual > 0 for _, residual in measurements) == 2
    assert skeleton.counter("same_time_events_enqueued_during_level") == 4
    assert skeleton.counter("same_time_residual_after_level") == 0
