"""P0-3: точный дифференциал MOTORCYCLE↔EXHAUSTIVE на весах и стенах."""

from __future__ import annotations

from collections import Counter
from fractions import Fraction
from functools import cache
import hashlib
import json
from math import gcd
from pathlib import Path

import pytest

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import node_record
from cftuv_envelope.wavefront.event_time import EventTimeV1, SupportLineV1
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
ABSOLUTE_DIGEST_RECEIPT = (
    Path(__file__).parents[2]
    / "artifacts"
    / "kernel_audit_exact_proof"
    / "p0_3_post_p0_2b_absolute_digests.json"
)


def _canonical_json(value) -> str:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )


def _typed_canonical(value):
    """Типизированный JSON без repr/runtime identity и неоднозначных ключей."""

    if isinstance(value, Counter):
        entries = [
            {
                "key": _typed_canonical(key),
                "count": _typed_canonical(count),
            }
            for key, count in value.items()
        ]
        entries.sort(key=lambda entry: _canonical_json(entry["key"]))
        return {"type": "counter", "entries": entries}
    if isinstance(value, Fraction):
        return {
            "type": "fraction",
            "numerator": value.numerator,
            "denominator": value.denominator,
        }
    if isinstance(value, EventTimeV1):
        canonical = value.canonical()
        return {
            "type": "event_time_v1",
            "dividend": _typed_canonical(canonical.dividend),
            "divisor_terms": _typed_canonical(canonical.divisor.terms),
        }
    if isinstance(value, tuple):
        return {
            "type": "tuple",
            "items": [_typed_canonical(item) for item in value],
        }
    if isinstance(value, list):
        return {
            "type": "list",
            "items": [_typed_canonical(item) for item in value],
        }
    if isinstance(value, dict):
        entries = [
            {
                "key": _typed_canonical(key),
                "value": _typed_canonical(item),
            }
            for key, item in value.items()
        ]
        entries.sort(key=lambda entry: _canonical_json(entry["key"]))
        return {"type": "dict", "entries": entries}
    if value is None:
        return {"type": "none"}
    if isinstance(value, bool):
        return {"type": "bool", "value": value}
    if isinstance(value, int):
        return {"type": "int", "value": value}
    if isinstance(value, str):
        return {"type": "str", "value": value}
    raise TypeError(f"P0_3_ABSOLUTE_DIGEST_TYPE_UNSUPPORTED: {type(value)!r}")


def _projection_sha256(reading) -> str:
    encoded = _canonical_json(_typed_canonical(reading)).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _digest_map_sha256(digests) -> str:
    return hashlib.sha256(
        _canonical_json(digests).encode("utf-8")
    ).hexdigest()


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


@pytest.mark.parametrize(
    "grid_cell", ("full_source_q_0", "source_and_walls_q_0")
)
def test_q_zero_grid_cells_are_rejected_before_split_mode(grid_cell):
    with pytest.raises(PolygonRejected) as refusal:
        all_stationary(cross(wide=6, tall=4))
    assert (
        grid_cell,
        refusal.value.outcome,
    ) == (grid_cell, PolygonOutcome.POLYGON_HAS_NO_SOURCE_EDGE)


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

    incidences = BY_NAME["several_collinear_incidences"].polygon
    line_multiplicity = Counter(
        _primitive_line_key(start, end) for start, end, _ in incidences.edges()
    )
    assert sum(count == 2 for count in line_multiplicity.values()) == 4

    collinear = BY_NAME["collinear_moving_source_and_wall"].polygon
    by_line = {}
    for start, end, speed in collinear.edges():
        by_line.setdefault(_primitive_line_key(start, end), set()).add(speed)
    assert any(
        speeds == {Fraction(0), Fraction(65, 64)}
        for speeds in by_line.values()
    )


def test_the_corpus_covers_every_required_family():
    features = Counter(feature for case in CORPUS for feature in case.features)
    assert len(CORPUS) == len(BY_NAME) == 23
    assert {"q_grid", "full_source", "source_and_walls"} <= set(features)
    assert {"fan", "zero_edge", "holes", "mirror", "collinear", "same_time"} <= set(
        features
    )
    assert set(AXES) == set(_readings(CORPUS[0].name)[SplitSearch.MOTORCYCLE])


#: Одиннадцать фигур, чья абсолютная проекция переснята 2026-08-05, и ровно
#: они. Причина одна на всех — закон места рождённого порта
#: (`symbolic_overlay._born_place`): пролёт, чей конец РОДИЛСЯ локусом на двух
#: совпавших антипараллельных прямых, перестал быть неограниченным, и вместе с
#: ним исчезли контакты ВНЕ пролёта. Шесть из одиннадцати — те, где ремонт
#: виден снаружи и модельно-независимо (см.
#: `test_born_place_repair_closes_the_cross_family_faces`).
#: Ключи — имена, значения — АБСОЛЮТНЫЕ проекции ДО ремонта (вершина 0bbf6e4).
#: Они здесь не для истории: возвращение любого из этих чисел означает, что
#: закон места рождённого порта отменён, и это утверждение фальсифицируемо.
_BORN_PLACE_PRE_REPAIR_SHA256 = {
    'collinear_moving_source_and_wall':
        'ae586eedec7e41786c4e1d9869faec088bce6513940b945bf3f570a53e2ec1a3',
    'cross_full_q_1':
        '302903ccebe8671bd99967490bf4b35e19a79aaf9adb2ddcf089e04753a0b9bb',
    'cross_full_q_1/4':
        '1c867b49e3754d241cd42d4dd4ef85a067639c6ee52576fe9f6e5fc4cf91fa46',
    'cross_full_q_4':
        '0bec2cd5df7940b01d6d71cf4dcd5c863c5e3eb9c768a51c2f9ae55fd7bd8130',
    'cross_full_q_65/64':
        '4b7a18d78d13c968c69aa200436077a88af6d9558e76e81fda588bbd58491850',
    'cross_source_and_walls_q_1':
        '6472add9a33a117a084a59300afbfbe537a986371358b5bdc3c04739cecbd19a',
    'cross_source_and_walls_q_1/4':
        '8614e1a5400a5c9e5c7132ed9b0049c6c68ac26b9c379a01c3ba0ed30fa9d781',
    'cross_source_and_walls_q_4':
        '428db43f4ae6d0aea5e23ca461ea5d44a16c1f8ab165a53d9db8049563c4fcde',
    'cross_source_and_walls_q_65/64':
        'c02b46f7e222ded0c60e8b67fc62e74bad01f6736c7cd2ea1ab61014cec4479b',
    'search_pruning_witness':
        'f3da2a67968ae2e3a426ec2712418e5ebef1ac7335cbc6340af64a81c05d577d',
    'several_collinear_incidences':
        '5f2c0d7825f10f5ae7cfaac83f0572a7097b549423a71195612649ea80dd2fb0',
}
_BORN_PLACE_RESHOT_CASES = frozenset(_BORN_PLACE_PRE_REPAIR_SHA256)
#: Шесть фигур, у которых сборка граней ДО правки отказывала
#: `FACE_CHAIN_DOES_NOT_CLOSE` с полным дефектом площади, а после правки
#: закрывается с НУЛЕВЫМ дефектом. Это внешний сертификат: он не спрашивает
#: ядро, какая траектория правильная, он проверяет площадь.
_BORN_PLACE_FACE_REPAIR = {
    "cross_full_q_1": 12,
    "cross_full_q_1/4": 12,
    "cross_full_q_4": 12,
    "cross_full_q_65/64": 12,
    "search_pruning_witness": 12,
    "several_collinear_incidences": 12,
}


def test_born_place_repair_closes_the_cross_family_faces():
    """Расписка «не этим числом» к пересъёмке 2026-08-05, исполняемая.

    До закона места рождённого порта эти шесть фигур давали EXACT-скелет, у
    которого НЕ СОБИРАЛИСЬ грани: `FACE_CHAIN_DOES_NOT_CLOSE`, ноль граней и
    дефект площади во весь многоугольник (кресты -336, `search_pruning_witness`
    -272, `several_collinear_incidences` -262). После правки каждая собирает
    двенадцать граней с дефектом РОВНО ноль и `proof_status` COMPLETE.

    Поэтому сдвиг абсолютных проекций — не «другое число», а починка: судья
    здесь не ядро, а площадь.
    """

    for name, expected_faces in _BORN_PLACE_FACE_REPAIR.items():
        polygon = BY_NAME[name].polygon
        for search, skeleton in _results(name).items():
            partition = build_faces(polygon, skeleton)
            defect = (
                partition.doubled_area
                - SqrtSumV1.rational(partition.polygon_doubled_area)
            )
            assert (
                name,
                search,
                partition.outcome.value,
                len(partition.faces),
                defect.terms,
                skeleton.proof_status.value,
            ) == (
                name,
                search,
                "EXACT",
                expected_faces,
                (),
                "COMPLETE",
            )


def test_post_p0_2b_absolute_projection_digests_are_frozen():
    """ПЕРЕЗАМОРОЗКА 2026-08-05, и она сделана ПОСЛЕДНЕЙ.

    Сдвинулись ровно одиннадцать проекций (`_BORN_PLACE_RESHOT_CASES`) из
    двадцати трёх; остальные двенадцать — байт в байт. Ось `outcome` не
    сдвинулась НИ У ОДНОЙ фигуры: терминал каждой остался прежним, поехали
    узлы (`node_record`, `times_points`, `participants`,
    `proof_obligations`), а у шести — ещё грани, покрытие и `proof_status`
    INCOMPLETE -> COMPLETE. Причина одна и названа выше; внешнее
    подтверждение, что это ремонт, — в
    `test_born_place_repair_closes_the_cross_family_faces`.

    Корпус имён и частичных источников (63 фигуры) при этом НЕ ДВИНУЛСЯ:
    outcome, semantic_digest и node_records там побитово прежние — это
    отдельно держит оракул `test_p0_geometry_and_existing_counter_oracle`.
    """

    receipt = json.loads(ABSOLUTE_DIGEST_RECEIPT.read_text(encoding="utf-8"))
    assert receipt["schema"] == (
        "cftuv.p0_3.post_p0_2b_absolute_digests.v1"
    )
    assert receipt["projection_schema"] == "TYPED_CANONICAL_JSON_V1"
    expected = receipt["case_projection_sha256"]
    assert set(expected) == set(BY_NAME)
    computed = {}
    for name in BY_NAME:
        readings = _readings(name)
        by_search = {
            search.value: _projection_sha256(reading)
            for search, reading in readings.items()
        }
        assert set(by_search.values()) == {expected[name]}, (name, by_search)
        computed[name] = expected[name]
    assert _digest_map_sha256(computed) == receipt["corpus_sha256"]
    # Расписка пересъёмки, исполняемая: имена сдвинувшихся — факты корпуса,
    # шесть починенных граней — их подмножество, а прежние числа обязаны
    # остаться прежними. Возврат любого из них означает отмену закона места
    # рождённого порта, и тогда эти ворота падают именованно.
    assert _BORN_PLACE_RESHOT_CASES <= set(BY_NAME)
    assert set(_BORN_PLACE_FACE_REPAIR) <= _BORN_PLACE_RESHOT_CASES
    assert not (
        set(_BORN_PLACE_PRE_REPAIR_SHA256.values())
        & set(expected.values())
    )


def test_the_constructed_weighted_case_really_has_a_multi_vertex_collapse():
    skeleton = _results("same_time_weighted_collapse")[SplitSearch.MOTORCYCLE]
    polygon = BY_NAME["same_time_weighted_collapse"].polygon
    assert {speed for _, _, speed in polygon.edges()} == {Fraction(4)}
    assert max(node.converging_vertices for node in skeleton.nodes) >= 4
