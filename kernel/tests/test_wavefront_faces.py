"""Грани скелета: обе объявленные границы проверяются на СОБСТВЕННОМ результате.

Правило проекта, поймавшее настоящий дефект пять раз: если функция объявляет
границу — тест обязан проверить, что её собственный результат этой границе
удовлетворяет. `faces.py` объявляет ровно две границы, и здесь по столбцу на
каждую:

| граница | что проверяется |
|---|---|
| 1. сумма площадей граней = площадь многоугольника | `area_reproduces_polygon` |
| 2. площадь каждой грани строго положительна | `every_face_is_positive` |
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.faces import (
    FaceOutcome,
    build_faces,
    doubled_shoelace,
    line_key,
    polygon_edges,
)
from cftuv_envelope.wavefront.polygon import PolygonV1, signed_double_area
from cftuv_envelope.wavefront.skeleton import SplitSearch
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

import wavefront_cases
from wavefront_cases import named_corpus


# Корпус, у которого несущие прямые рёбер различны. `holes_2` вынесен отдельным
# тестом: он и есть свидетельство исхода SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES.
def _distinct_line_corpus():
    for name, polygon in named_corpus():
        keys = [line_key(line) for _, _, line in polygon_edges(polygon)]
        if len(set(keys)) == len(keys):
            yield name, polygon


CORPUS = tuple(_distinct_line_corpus())
CORPUS_IDS = tuple(name for name, _ in CORPUS)


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_the_face_partition_reproduces_the_polygon_area_exactly(name, polygon):
    """Граница 1 на собственном результате: площадь сходится ТОЧНО."""

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert partition.area_reproduces_polygon, (
        f"{name}: дефект площади {partition.area_defect.terms}"
    )
    # Не «почти ноль», а пустой канонический набор коэффициентов.
    assert partition.area_defect.terms == ()


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_every_face_of_the_partition_has_strictly_positive_area(name, polygon):
    """Граница 2 на собственном результате: щепок и вывернутых граней нет."""

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    assert partition.every_face_is_positive
    for face in partition.faces:
        assert face.doubled_area.sign() > 0, f"{name}: грань {face.owner}"


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_there_is_exactly_one_face_per_input_edge_and_owner_is_its_line(
    name, polygon
):
    """Owner выпадает из структуры: одна грань на ребро, ключ — его прямая."""

    partition = build_faces(polygon, build_skeleton(polygon))
    edges = polygon_edges(polygon)
    assert len(partition.faces) == len(edges)
    assert {face.owner for face in partition.faces} == {
        line_key(line) for _, _, line in edges
    }


def test_a_shared_support_line_is_a_named_outcome_not_a_wrong_area():
    """`holes_2`: коллинеарные рёбра дают ИСХОД, а не подогнанную площадь.

    Числа, ради которых тест существует: наивная сборка выдаёт удвоенную
    площадь 1584 при истинной 1344. Молча её принять означало бы поверить
    разбиению, которого нет.
    """

    polygon = next(
        item for name, item in named_corpus() if name == "holes_2"
    )
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES
    assert partition.faces == ()
    assert partition.polygon_doubled_area == 1344


def test_the_two_split_searches_give_the_same_faces():
    """Оптимизация законна ровно настолько, насколько даёт тот же ответ."""

    for name, polygon in CORPUS:
        motorcycle = build_faces(
            polygon, build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
        )
        exhaustive = build_faces(
            polygon, build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
        )
        assert motorcycle.outcome is exhaustive.outcome, name
        assert motorcycle.doubled_area == exhaustive.doubled_area, name
        assert {
            face.owner: face.doubled_area for face in motorcycle.faces
        } == {face.owner: face.doubled_area for face in exhaustive.faces}, name


def test_the_faces_of_an_unproven_skeleton_are_refused_not_invented():
    """Скелет не доказан — граней нет. Тихого нуля здесь быть не может."""

    from dataclasses import replace

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    skeleton = build_skeleton(polygon)
    broken = replace(
        skeleton, outcome=type(skeleton.outcome).WAVEFRONT_LEFT_UNRESOLVED
    )
    partition = build_faces(polygon, broken)
    assert partition.outcome is FaceOutcome.SKELETON_IS_NOT_EXACT
    assert partition.faces == ()
    assert partition.detail == "WAVEFRONT_LEFT_UNRESOLVED"


def test_the_shoelace_over_sqrt_sums_agrees_with_the_integer_one():
    """Площадь над `SqrtSumV1` — та же формула, а не вторая арифметика."""

    for points in (
        ((0, 0), (4, 0), (4, 3), (0, 3)),
        ((0, 0), (7, 0), (3, 5)),
        ((-2, -2), (5, -1), (4, 6), (-3, 3)),
    ):
        lifted = tuple(
            (SqrtSumV1.rational(x), SqrtSumV1.rational(y)) for x, y in points
        )
        assert doubled_shoelace(lifted).as_rational() == Fraction(
            signed_double_area(points)
        )


def test_the_face_area_of_the_axis_square_is_the_quarter_it_must_be():
    """Известный ответ руками: квадрат 8x8 даёт четыре равные грани по 16."""

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    assert [
        face.doubled_area.as_rational() for face in partition.faces
    ] == [Fraction(32)] * 4
    assert partition.doubled_area.as_rational() == Fraction(128)
