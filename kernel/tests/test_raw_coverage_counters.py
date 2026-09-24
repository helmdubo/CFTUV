"""Счётчики arrangement: наблюдение обязано пережить падение стадии.

Счётчики заведены, чтобы по полевому прогону было видно, какая величина растёт
нелинейно у большого патча против мелкого. Стадия, упавшая на точном предикате,
— как раз самый интересный случай, и оставить её без единого числа значит
потерять ровно то измерение, ради которого всё это делается.
"""

from __future__ import annotations

from dataclasses import dataclass

from cftuv_envelope.reference.raw_coverage import _union_counters


@dataclass(frozen=True)
class _StubUnion:
    """Только те поля, которые читает `_union_counters`."""

    broadphase_candidate_pair_count: int = 26
    narrowphase_test_count: int = 26
    intersection_count: int = 7
    atomic_edge_count: int = 31
    branch_point_count: int = 2
    max_incident_degree: int = 4
    rotation_comparison_count: int = 11
    face_walk_count: int = 3
    boundary_vertex_occurrences: tuple = (1, 2)
    point_contacts: tuple = ()
    vertices: tuple = (1, 2, 3)
    edges: tuple = (1, 2, 3, 4)
    loops: tuple = (1, 2)
    regions: tuple = (1,)
    point_location_segment_scan_count: int = 1904
    max_coordinate_chars: int = 143


def test_failed_stage_still_reports_its_input():
    counters = _union_counters(None, 24)

    assert counters == {
        "ARRANGEMENT_INPUT_SEGMENTS": 24,
        "ARRANGEMENT_ALL_POSSIBLE_PAIRS": 276,
    }


def test_successful_stage_reports_both_sides_of_the_arrangement():
    counters = _union_counters(_StubUnion(), 24)

    # Вход: сколько сегментов пришло и сколько пар было бы при полном переборе.
    assert counters["ARRANGEMENT_INPUT_SEGMENTS"] == 24
    assert counters["ARRANGEMENT_ALL_POSSIBLE_PAIRS"] == 276
    # Отсев: broadphase обязан быть виден отдельно от полного перебора, иначе
    # его нельзя ни подтвердить, ни опровергнуть замером.
    assert counters["ARRANGEMENT_BROADPHASE_CANDIDATE_PAIRS"] == 26
    assert counters["ARRANGEMENT_NARROWPHASE_TESTS"] == 26
    assert counters["ARRANGEMENT_PAIR_TESTS"] == 26
    assert counters["ARRANGEMENT_INTERSECTIONS"] == 7
    # Выход: без него «пришло много» и «расплодилось на пересечениях»
    # выглядят одинаково, а лечатся по-разному.
    assert counters["ARRANGEMENT_OUTPUT_VERTICES"] == 3
    assert counters["ARRANGEMENT_OUTPUT_EDGES"] == 4
    assert counters["ARRANGEMENT_OUTPUT_LOOPS"] == 2
    assert counters["ARRANGEMENT_OUTPUT_REGIONS"] == 1
    # Работа локализации точки и длина координат: ни одна считаемая величина
    # не объяснила полевые времена, и это две, которых там не хватало.
    assert counters["ARRANGEMENT_POINT_LOCATION_SCANS"] == 1904
    assert counters["ARRANGEMENT_MAX_COORDINATE_CHARS"] == 143


def test_every_counter_is_a_plain_number():
    """Счётчик уходит в JSON-sidecar, поэтому он число, а не объект ядра."""

    for value in _union_counters(_StubUnion(), 24).values():
        assert isinstance(value, (int, float)) and not isinstance(value, bool)
