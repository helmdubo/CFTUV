"""Пределы ширины вершин сертификата (VERTEX_ALPHA_LIMITS_V1): где аффинный ход вершины кончается.

Держится: (1) по две ширины на вершину в порядке позиций, и ширина сертификата лежит между ними; (2) предел вершины - событие прихода, поэтому он не внутри
заверенного интервала домена (интервал - окно без событий); (3) при верхнем и нижнем пределе точка фронта движущейся вершины стоит в вершине грани скелета
(конец своего ребра); (4) пределы - запись: ответ домена с ними и без них тот же (`test_parametric_certificate`), а NATIVE и PYTHON дают одни числа
(`tests/test_native_key_slopes.py`).
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


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES, ids=IDS)
def test_every_vertex_has_its_two_limits_around_the_width_and_none_inside_the_interval(name, build, laws, given, clipped):
    _prepared, alpha0, interval, certificate = _certificate(build, laws, given)
    limits = certificate.alpha_limits

    assert len(limits) == 2 * certificate.vertex_count
    high_of_window = math.inf if interval.high is None else interval.high
    for low, high in zip(limits[0::2], limits[1::2]):
        assert low <= float(alpha0) <= high, (name, low, high)
        assert low <= interval.low + 1e-12 and high >= high_of_window - 1e-12, (name, low, high, interval.low, interval.high)


@pytest.mark.parametrize("name,build,laws,given,clipped", CASES, ids=IDS)
def test_at_its_limit_a_moving_vertex_stands_on_a_node_of_the_skeleton_face(name, build, laws, given, clipped):
    prepared, alpha0, interval, certificate = _certificate(build, laws, given)
    exact = certificate.exact
    window = (interval.low, math.inf if interval.high is None else interval.high)
    nodes = [
        (_float(x), _float(y))
        for region in prepared.regions
        for face in region.partition.faces
        for x, y in face.points
    ]
    scale, lattice0 = certificate.scale, alpha0 * certificate.scale
    judged = moving = 0
    for index, (point, slope) in enumerate(zip(exact.points0, exact.point_slopes)):
        limit_pair = tuple(certificate.alpha_limits[2 * index: 2 * index + 2])
        if slope[0].is_zero and slope[1].is_zero:
            continue  # неподвижная вершина: окно домена
        moving += 1
        if limit_pair == window:
            continue  # узел резки: окно домена
        for limit in limit_pair:
            shift = limit * scale - float(lattice0)
            at = (_float(point[0]) + _float(slope[0]) * shift, _float(point[1]) + _float(slope[1]) * shift)
            gap = min(math.hypot(at[0] - x, at[1] - y) for x, y in nodes)
            assert gap <= NODE_TOLERANCE * max(1.0, abs(at[0]), abs(at[1])), (name, index, limit, gap)
        judged += 1
    assert judged >= 1 or not moving, f"{name}: {moving} moving vertices, none with limits of its own"
