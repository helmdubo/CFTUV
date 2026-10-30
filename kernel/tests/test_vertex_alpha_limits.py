"""Пределы ширины вершин сертификата (VERTEX_ALPHA_LIMITS_V1): где аффинный ход вершины кончается.

Держится: (1) по одной ширине на вершину в каждом поле (`vertex_floor_widths`, `vertex_limit_widths`) в порядке позиций, и ширина сертификата лежит между ними;
(2) предел движущейся вершины - событие прихода, поэтому он не внутри заверенного интервала домена (интервал - окно без событий); неподвижная - без предела;
(3) при пределе и при полу точка фронта движущейся вершины стоит в вершине грани скелета (конец своего ребра); (4) пределы - запись: ответ домена тот же
(`test_parametric_certificate`), а NATIVE и PYTHON дают одни числа (`tests/test_native_key_slopes.py`).
"""

from __future__ import annotations

import math
from fractions import Fraction

import pytest

from test_parametric_certificate import CASES, IDS, alpha_of, step

from cftuv_envelope.materialize.parametric import _float

#: Округление пары float-подъёмов (точка карты против вершины грани), единицы решётки.
NODE_TOLERANCE = 1e-7


def _certificate(build, laws, given):
    prepared = build()
    text = alpha_of(prepared, given)
    base = step(prepared, text, laws)
    return prepared, Fraction(text), base.result.interval, base.result.parametric


def _still(slope) -> bool:
    return slope[0].is_zero and slope[1].is_zero


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES, ids=IDS)
def test_every_vertex_has_a_floor_and_a_limit_around_the_width_and_none_inside_the_interval(name, build, laws, given, clipped):
    _prepared, alpha0, interval, certificate = _certificate(build, laws, given)
    floors, limits = certificate.vertex_floor_widths, certificate.vertex_limit_widths

    assert len(floors) == len(limits) == certificate.vertex_count
    high_of_window = math.inf if interval.high is None else interval.high
    for floor, limit, slope in zip(floors, limits, certificate.exact.point_slopes):
        assert floor <= float(alpha0) <= limit, (name, floor, limit)
        assert floor <= interval.low + 1e-12 and limit >= high_of_window - 1e-12, (name, floor, limit, interval.low, interval.high)
        if _still(slope):
            assert (floor, limit) == (-math.inf, math.inf)  # неподвижная вершина: предела нет


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES, ids=IDS)
def test_at_its_floor_and_its_limit_a_moving_vertex_stands_on_a_node_of_the_skeleton_face(name, build, laws, given, clipped):
    prepared, alpha0, interval, certificate = _certificate(build, laws, given)
    exact = certificate.exact
    window = (interval.low, math.inf if interval.high is None else interval.high)
    nodes = [(_float(x), _float(y)) for region in prepared.regions for face in region.partition.faces for x, y in face.points]
    scale, lattice0 = certificate.scale, alpha0 * certificate.scale
    judged = moving = 0
    for index, (point, slope) in enumerate(zip(exact.points0, exact.point_slopes)):
        if _still(slope):
            continue
        moving += 1
        pair = (certificate.vertex_floor_widths[index], certificate.vertex_limit_widths[index])
        if pair == window:
            continue  # узел резки: окно домена
        for width in pair:
            shift = width * scale - float(lattice0)
            at = (_float(point[0]) + _float(slope[0]) * shift, _float(point[1]) + _float(slope[1]) * shift)
            gap = min(math.hypot(at[0] - x, at[1] - y) for x, y in nodes)
            assert gap <= NODE_TOLERANCE * max(1.0, abs(at[0]), abs(at[1])), (name, index, width, gap)
        judged += 1
    assert judged >= 1 or not moving, f"{name}: {moving} moving vertices, none with limits of its own"
