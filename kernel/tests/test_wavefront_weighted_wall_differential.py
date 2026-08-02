"""P0-3: точный дифференциал MOTORCYCLE↔EXHAUSTIVE на весах и стенах."""

from __future__ import annotations

from collections import Counter
from fractions import Fraction
from functools import cache
import json

import pytest

from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import node_record
from cftuv_envelope.wavefront.faces import build_faces
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
    "proof_dispositions",
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
        "proof_dispositions": Counter(
            obligation.disposition.value
            for obligation in skeleton.proof_obligations
        ),
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
    mismatches = tuple(axis for axis in AXES if motorcycle[axis] != exhaustive[axis])
    assert mismatches == (), f"{name}: WF_WEIGHTED_WALL_DIFFERENTIAL_MISMATCH {mismatches}"


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


def test_the_literal_q_zero_full_source_cell_is_rejected_before_both_modes():
    with pytest.raises(PolygonRejected) as refusal:
        all_stationary(cross(wide=6, tall=4))
    assert refusal.value.outcome is PolygonOutcome.POLYGON_HAS_NO_SOURCE_EDGE


def test_the_corpus_covers_every_required_family_without_absolute_digests():
    features = Counter(feature for case in CORPUS for feature in case.features)
    assert len(CORPUS) == len(BY_NAME) == 22
    assert {"q_grid", "full_source", "source_and_walls"} <= set(features)
    assert {"fan", "zero_edge", "holes", "mirror", "collinear", "same_time"} <= set(
        features
    )
    assert set(AXES) == set(_readings(CORPUS[0].name)[SplitSearch.MOTORCYCLE])


def test_the_constructed_weighted_case_really_has_a_multi_vertex_collapse():
    skeleton = _results("same_time_weighted_collapse")[SplitSearch.MOTORCYCLE]
    assert max(node.converging_vertices for node in skeleton.nodes) >= 4
