"""P0-3: точный дифференциал MOTORCYCLE↔EXHAUSTIVE на весах и стенах."""

from __future__ import annotations

from collections import Counter
from fractions import Fraction
from functools import cache
import json
from math import gcd

import pytest

from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import node_record
from cftuv_envelope.wavefront.event_time import SupportLineV1
from cftuv_envelope.wavefront.faces import build_faces, fan_edge_key
from cftuv_envelope.wavefront.polygon import PolygonOutcome, PolygonRejected
from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton

from weighted_wall_differential_cases import (
    all_stationary,
    weighted_wall_differential_corpus,
)
from wavefront_cases import cross


ALPHAS = (Fraction(1, 4), Fraction(1), Fraction(4))
CORPUS = weighted_wall_differential_corpus()
BY_NAME = {case.name: case for case in CORPUS}
AXES = (
    "outcome",
    "node_record",
    "times_points",
    "participants",
    "face_owners",
    "face_areas",
    "coverage",
    "proof_status",
    "proof_obligations",
)


def _point(point):
    return (point[0].terms, point[1].terms)


def _coverage_record(partition):
    records = []
    for alpha in ALPHAS:
        covered = coverage_at(partition, alpha)
        faces = Counter(
            (
                face.owner,
                tuple(_point(point) for point in face.points),
                face.doubled_area.terms,
            )
            for face in covered.faces
        )
        records.append(
            (
                alpha,
                covered.outcome.value,
                faces,
                covered.doubled_area.terms,
                covered.polygon_doubled_area,
            )
        )
    return tuple(records)


def _proof_projection(skeleton):
    """Стабильная obligation-запись: runtime vertex ids намеренно исключены."""

    return Counter(
        (
            obligation.cause.value,
            obligation.disposition.value,
            obligation.level.canonical(),
            obligation.participant_edge_keys,
            obligation.target_edge_keys,
            (
                None
                if obligation.event_kind is None
                else obligation.event_kind.value
            ),
        )
        for obligation in skeleton.proof_obligations
    )


def _reading(polygon, skeleton):
    partition = build_faces(polygon, skeleton)
    times_points = Counter(
        (
            node.time.canonical().dividend,
            node.time.canonical().divisor.terms,
            node.point.x.terms,
            node.point.y.terms,
        )
        for node in skeleton.nodes
    )
    face_areas = Counter(
        (face.owner, face.doubled_area.terms) for face in partition.faces
    )
    return {
        "outcome": skeleton.outcome.value,
        "node_record": Counter(
            json.dumps(node_record(node), sort_keys=True)
            for node in skeleton.nodes
        ),
        "times_points": times_points,
        "participants": Counter(node.participants for node in skeleton.nodes),
        "face_owners": (
            partition.outcome.value,
            Counter(face.owner for face in partition.faces),
        ),
        "face_areas": (
            partition.outcome.value,
            face_areas,
            partition.doubled_area.terms,
            partition.polygon_doubled_area,
        ),
        "coverage": _coverage_record(partition),
        "proof_status": skeleton.proof_status.value,
        "proof_obligations": _proof_projection(skeleton),
    }


@cache
def _results(name):
    polygon = BY_NAME[name].polygon
    return {
        search: build_skeleton(polygon, split_search=search)
        for search in SplitSearch
    }


@cache
def _readings(name):
    polygon = BY_NAME[name].polygon
    return {
        search: _reading(polygon, skeleton)
        for search, skeleton in _results(name).items()
    }


@pytest.mark.parametrize("name", tuple(BY_NAME))
def test_motorcycle_equals_exhaustive_on_all_nine_axes(name):
    readings = _readings(name)
    motorcycle = readings[SplitSearch.MOTORCYCLE]
    exhaustive = readings[SplitSearch.EXHAUSTIVE]
    assert tuple(motorcycle) == AXES
    for axis in AXES:
        if motorcycle[axis] != exhaustive[axis]:
            pytest.fail(
                f"{name}: WF_WEIGHTED_WALL_DIFFERENTIAL_MISMATCH axis={axis}\n"
                f"MOTORCYCLE={motorcycle[axis]!r}\n"
                f"EXHAUSTIVE={exhaustive[axis]!r}"
            )


def test_at_least_one_third_of_the_corpus_exercises_real_pruning():
    pruned = []
    for case in CORPUS:
        results = _results(case.name)
        motorcycle = results[SplitSearch.MOTORCYCLE]
        exhaustive = results[SplitSearch.EXHAUSTIVE]
        if (
            motorcycle.counter("split_candidates_beyond_trace") > 0
            or motorcycle.counter("split_candidates_examined")
            < exhaustive.counter("split_candidates_examined")
        ):
            pruned.append(case.name)
    assert len(pruned) * 3 >= len(CORPUS), (len(pruned), len(CORPUS), pruned)


def test_q_zero_full_source_is_rejected_before_split_mode():
    with pytest.raises(PolygonRejected) as refusal:
        all_stationary(cross(wide=6, tall=4))
    assert refusal.value.outcome is PolygonOutcome.POLYGON_HAS_NO_SOURCE_EDGE


def test_strict_pruning_witness_executes_two_different_searches():
    results = _results("search_pruning_witness")
    motorcycle = results[SplitSearch.MOTORCYCLE]
    exhaustive = results[SplitSearch.EXHAUSTIVE]
    assert motorcycle.counter("split_candidates_examined") < exhaustive.counter(
        "split_candidates_examined"
    )


def _primitive_line_key(start, end):
    line = SupportLineV1.through(start, end)
    common = gcd(gcd(abs(line.a), abs(line.b)), abs(line.c))
    row = (line.a // common, line.b // common, line.c // common)
    if next(item for item in row if item) < 0:
        row = tuple(-item for item in row)
    return row


def test_q_grid_and_special_families_are_facts_of_the_inputs():
    for q in (Fraction(1, 4), Fraction(1), Fraction(4), Fraction(65, 64)):
        full = BY_NAME[f"cross_full_q_{q}"].polygon
        partial = BY_NAME[f"cross_source_and_walls_q_{q}"].polygon
        assert {speed for _, _, speed in full.edges()} == {q}
        assert {speed for _, _, speed in partial.edges()} == {Fraction(0), q}

    fan = BY_NAME["weighted_vertex_fan"].polygon
    fan_keys = tuple(
        fan_edge_key(point, ordinal)
        for point, ordinal, _ in fan.fan_edges()
    )
    assert fan.fan_edge_count > 0
    assert all(key[:2] == key[2:4] for key in fan_keys)
    assert all(
        BY_NAME[name].polygon.holes
        for name in ("weighted_holes_1", "weighted_holes_2")
    )

    direct = BY_NAME["mirror_direct"].polygon
    reflected = BY_NAME["mirror_reflected"].polygon
    reflected_edges = {
        (frozenset(((-start[0], start[1]), (-end[0], end[1]))), speed)
        for start, end, speed in direct.edges()
    }
    assert reflected_edges == {
        (frozenset((start, end)), speed) for start, end, speed in reflected.edges()
    }

    collinear = BY_NAME["collinear_moving_source_and_wall"].polygon
    by_line = {}
    for start, end, speed in collinear.edges():
        by_line.setdefault(_primitive_line_key(start, end), set()).add(speed)
    assert any(
        speeds == {Fraction(0), Fraction(65, 64)}
        for speeds in by_line.values()
    )


def test_the_corpus_covers_every_required_family_without_absolute_digests():
    features = Counter(feature for case in CORPUS for feature in case.features)
    assert len(CORPUS) == len(BY_NAME) == 23
    assert {"q_grid", "full_source", "source_and_walls"} <= set(features)
    assert {"fan", "zero_edge", "holes", "mirror", "collinear", "same_time"} <= set(
        features
    )
    assert set(AXES) == set(_readings(CORPUS[0].name)[SplitSearch.MOTORCYCLE])


def test_the_constructed_weighted_case_really_has_a_multi_vertex_collapse():
    skeleton = _results("same_time_weighted_collapse")[SplitSearch.MOTORCYCLE]
    polygon = BY_NAME["same_time_weighted_collapse"].polygon
    assert {speed for _, _, speed in polygon.edges()} == {Fraction(4)}
    assert max(node.converging_vertices for node in skeleton.nodes) >= 4
