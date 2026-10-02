"""Отсечение ушей: точное, детерминированное, все вершины контура в деле."""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize.tessellate import triangulate_exact, triangulate_from_apex
from cftuv_envelope.wavefront.faces import doubled_shoelace, orientation

import materialize_factories as factories
from wavefront_cases import named_corpus


def _points(raw):
    return tuple(
        (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))
        for x, y in raw
    )


def _check(points, triangles):
    """Свойства ЛЮБОЙ верной тесселяции контура — все точные."""

    budget = factories.budget()
    assert len(triangles) == len(points) - 2
    total = SqrtSumV1.zero()
    for first, second, third in triangles:
        triangle = (points[first], points[second], points[third])
        assert orientation(*triangle, budget) > 0
        total = total + doubled_shoelace(triangle)
    assert (total - doubled_shoelace(points)).is_zero or (
        total + doubled_shoelace(points)
    ).is_zero
    used = {index for triangle in triangles for index in triangle}
    assert used == set(range(len(points)))


def test_a_convex_quad_gives_two_triangles():
    points = _points(((0, 0), (4, 0), (4, 3), (0, 3)))
    triangles = triangulate_exact(points, factories.budget())
    assert triangles is not None
    _check(points, triangles)


def test_a_non_convex_polygon_is_closed_without_holes():
    # Г-образный шестиугольник: одна вогнутая вершина.
    points = _points(((0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)))
    triangles = triangulate_exact(points, factories.budget())
    assert triangles is not None
    _check(points, triangles)


def test_a_collinear_boundary_vertex_stays_a_vertex_of_some_triangle():
    """Середина слитой цепи не кончик уха, но и не пропадает: иначе T-стык."""

    points = _points(((0, 0), (2, 0), (4, 0), (4, 1), (0, 1)))
    triangles = triangulate_exact(points, factories.budget())
    assert triangles is not None
    _check(points, triangles)
    assert any(1 in triangle for triangle in triangles)


def test_clockwise_input_gives_the_same_counter_clockwise_triangles():
    ccw = _points(((0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)))
    cw = tuple(reversed(ccw))
    triangles = triangulate_exact(cw, factories.budget())
    assert triangles is not None
    _check(cw, triangles)


def test_the_answer_is_deterministic():
    points = _points(((0, 0), (6, 0), (6, 1), (3, 5), (0, 1)))
    first = triangulate_exact(points, factories.budget())
    second = triangulate_exact(points, factories.budget())
    assert first == second


def test_a_degenerate_contour_does_not_close_and_says_none():
    flat = _points(((0, 0), (2, 0), (4, 0)))
    assert triangulate_exact(flat, factories.budget()) is None
    assert triangulate_exact(_points(((0, 0), (1, 1))), factories.budget()) is None


def test_a_bowtie_does_not_close():
    """Самопересечение — не простой контур, и ушей у него нет: именованный None."""

    bowtie = _points(((0, 0), (4, 4), (4, 0), (0, 4)))
    triangles = triangulate_exact(bowtie, factories.budget())
    assert triangles is None


@pytest.mark.parametrize("name,polygon", named_corpus())
def test_every_outer_loop_of_the_corpus_closes_exactly(name, polygon):
    """Внешние контуры корпуса (в том числе звёзды) тесселируются точно."""

    points = _points(polygon.outer.points)
    triangles = triangulate_exact(points, factories.budget())
    assert triangles is not None, name
    _check(points, triangles)


# --------------------------------------------------------------------------
# Веер от вершины: клетка веера режется от вершины веера, а не по порядку контура
# --------------------------------------------------------------------------

CONVEX_CELL = ((0, 0), (4, 0), (3, 3), (0, 4))
PENTAGON_CELL = ((0, 0), (4, 0), (5, 3), (2, 5), (0, 4))
#: Правый поворот в `(1, 1)`, но вершина `(0, 0)` видит остальные: звезда относительно неё.
DART_CELL = ((0, 0), (4, 0), (1, 1), (0, 4))
#: Вершина `(2, 1)` вершине `(0, 0)` не видна (диагональ `(0, 0)-(4, 4)` вне контура).
HIDDEN_CELL = ((0, 0), (4, 0), (4, 4), (2, 1))


@pytest.mark.parametrize("raw", (CONVEX_CELL, PENTAGON_CELL, DART_CELL))
@pytest.mark.parametrize("start", range(4))
@pytest.mark.parametrize("clockwise", (False, True))
def test_the_fan_cut_from_the_apex_is_the_same_for_every_contour_start_and_direction(raw, start, clockwise):
    """Один и тот же набор треугольников (по точкам) при любом начале и обходе контура."""

    ring = raw[start:] + raw[:start]
    ring = ring[::-1] if clockwise else ring
    points = _points(ring)
    apex = ring.index((0, 0))
    triangles = triangulate_from_apex(points, apex, factories.budget())
    assert triangles is not None
    _check(points, triangles)
    assert all(triangle[0] == apex for triangle in triangles)
    found = {frozenset(ring[index] for index in triangle) for triangle in triangles}
    expected = {
        frozenset((raw[0], raw[index], raw[index + 1])) for index in range(1, len(raw) - 1)
    }
    assert found == expected


def test_the_fan_from_the_apex_differs_from_the_ear_cut_it_replaces():
    """Контур начат с угла, и первое ухо уходит мимо вершины: `(3, 3)` с соседями, а не `(0, 0)`."""

    points = _points(CONVEX_CELL[2:] + CONVEX_CELL[:2])
    ears = triangulate_exact(points, factories.budget())
    fan = triangulate_from_apex(points, 2, factories.budget())
    assert ears is not None and fan is not None
    assert any(2 not in triangle for triangle in ears)
    assert all(2 in triangle for triangle in fan)


@pytest.mark.parametrize("raw", (HIDDEN_CELL, ((0, 0), (2, 0), (4, 0), (4, 3)), ((0, 0), (1, 0), (2, 0))))
def test_a_cell_the_apex_does_not_fully_see_is_none(raw):
    """Невидимая вершина, вершина на одной прямой с вершиной веера, нулевая площадь — `None`, не разрез."""

    assert triangulate_from_apex(_points(raw), 0, factories.budget()) is None


def test_the_apex_must_be_a_vertex_of_the_contour():
    points = _points(CONVEX_CELL)
    assert triangulate_from_apex(points, 4, factories.budget()) is None
    assert triangulate_from_apex(points, -1, factories.budget()) is None
    assert triangulate_from_apex(points[:2], 0, factories.budget()) is None


def test_a_vertex_on_the_rim_is_a_vertex_of_a_triangle_of_the_fan():
    """Вершина на прямой стороне (не вершина веера) несёт T-стык и остаётся вершиной."""

    points = _points(((0, 0), (2, 0), (4, 0), (4, 3), (0, 3)))
    triangles = triangulate_from_apex(points, 3, factories.budget())
    assert triangles is not None
    _check(points, triangles)
    assert any(1 in triangle for triangle in triangles)
