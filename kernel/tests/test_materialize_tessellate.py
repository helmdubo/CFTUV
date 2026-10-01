"""Отсечение ушей: точное, детерминированное, все вершины контура в деле."""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize.tessellate import triangulate_exact
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
