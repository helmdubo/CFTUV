"""Закон `UNFOLD_ALONG_U_V1` (Straight strips): `v = r / alpha` не тронута, `u` - развёртка острова по треугольникам Blender, источник держится, всё названо числом.

Числа посчитаны из входа (`UnfoldInputV1` вручную либо фикстура `test_corner_join`), не записаны с прогона.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from cftuv_envelope.materialize import uv_unfold
from cftuv_envelope.materialize.uv_unfold import NATIVE_EQUALITY, UnfoldInputV1, polyfill, results_agree, unfold
from test_corner_join import _materialized, _prepared, _snapshot


def _strip(xs, top_xs=None, *, source_s=None, top_z=0.0, hold=True, guard_rounds=uv_unfold.GUARD_ROUNDS):
    """Полоса четырёхгранников: низ `(x, 0, 0)` на `r = 0`, верх `(x', 1, top_z)` на `r = 1`; кольца против часовой в карте `(x, y)`; `s_run = x`."""

    top_xs = xs if top_xs is None else top_xs
    source_s = xs if source_s is None else source_s
    count = len(xs)
    positions, chart, r, s, source = [], [], [], [], []
    for x, station in zip(xs, source_s):
        positions += [x, 0.0, 0.0]
        chart += [x, 0.0]
        r.append(0.0)
        s.append(station)
        source.append(True)
    for x in top_xs:
        positions += [x, 1.0, top_z]
        chart += [x, 1.0]
        r.append(1.0)
        s.append(x)
        source.append(False)
    starts, corners, stations = [0], [], []
    for at in range(count - 1):
        ring = (at, at + 1, count + at + 1, count + at)
        corners += ring
        stations += [chart[2 * node] for node in ring]
        starts.append(len(corners))
    return UnfoldInputV1(tuple(positions), tuple(chart), tuple(r), tuple(s), tuple(source), tuple(starts), tuple(corners), tuple(stations), hold, uv_unfold.ARAP_ITERATIONS, guard_rounds)


def _counters(result):
    return dict(result.counters)


# --------------------------------------------------------------------------
# Треугольники Blender
# --------------------------------------------------------------------------


def test_a_quad_is_cut_as_blender_cuts_it():
    """`is_quad_flip_v3_first_third_fast`: диагональ `0-2`, если `1` и `3` по разные стороны от неё, иначе `1-3`."""

    assert polyfill([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]) == [(0, 1, 2), (0, 2, 3)]
    assert polyfill([(0.0, 0.0), (2.0, 0.0), (0.5, 0.5), (0.0, 2.0)]) == [(0, 1, 2), (0, 2, 3)]
    assert polyfill([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (1.5, 0.5)]) == [(0, 1, 3), (1, 2, 3)]


def test_the_ears_of_a_concave_ring_cover_it_without_overlap():
    points = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (2.0, 1.0), (0.0, 4.0)]
    triangles = polyfill(points)
    assert len(triangles) == 3

    def doubled(a, b, c):
        (ax, ay), (bx, by), (cx, cy) = points[a], points[b], points[c]
        return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)

    ring = sum(points[at][0] * points[(at + 1) % 5][1] - points[(at + 1) % 5][0] * points[at][1] for at in range(5))
    assert all(doubled(*triangle) > 0.0 for triangle in triangles)
    assert sum(doubled(*triangle) for triangle in triangles) == pytest.approx(ring, abs=1e-12)


# --------------------------------------------------------------------------
# Решение
# --------------------------------------------------------------------------


@pytest.mark.parametrize("hold", (True, False))
def test_a_straight_strip_unfolds_to_its_length(hold):
    xs = (0.0, 1.0, 2.5, 4.0)
    result = unfold(_strip(xs, hold=hold))
    for value, expected in zip(result.u, xs + xs):
        assert value == pytest.approx(expected, abs=1e-12)
    numbers = _counters(result)
    assert numbers[uv_unfold.ISLANDS] == 1 and numbers[uv_unfold.TRIANGLES] == 6
    assert uv_unfold.FLIPPED not in numbers and uv_unfold.GUARDED not in numbers
    # Держится только источник; свободный ряд не держит ничего.
    assert result.held == ((True,) * 4 + (False,) * 4 if hold else (False,) * 8)


def test_the_held_source_keeps_the_station_and_the_rest_follows_the_geometry():
    """Источник на `u = s` ТОЧНО, даже где станция не равна длине; верх полосы, наклонённой на 45 градусов, - по своей 3D-длине (поперёк - `r`)."""

    xs = (0.0, 1.0, 2.0)
    result = unfold(_strip(xs, source_s=(0.0, 1.0, 2.0), top_z=1.0))
    assert result.u[:3] == (0.0, 1.0, 2.0)
    assert result.slope_x[:3] == (0.0, 0.0, 0.0)
    for value, expected in zip(result.u[3:], xs):
        assert value == pytest.approx(expected, abs=1e-12)


def test_an_island_without_source_takes_the_least_squares_shift():
    xs = (0.0, 1.0, 2.0)
    data = _strip(xs)
    data = dataclasses.replace(data, s=tuple(value + 5.0 for value in data.s), source=(False,) * 6)
    result = unfold(data)
    numbers = _counters(result)
    assert numbers[uv_unfold.WITHOUT_SOURCE] == 1
    # Сдвиг по всем вершинам: среднее `u - s` нуль.
    assert sum(u - s for u, s in zip(result.u, data.s)) == pytest.approx(0.0, abs=1e-12)
    assert result.u[1] - result.u[0] == pytest.approx(1.0, abs=1e-12)


# --------------------------------------------------------------------------
# UNFOLD_DEGENERATE_TRIANGLE_V1
# --------------------------------------------------------------------------


def _with_sliver(height):
    """Полоса из одного квадрата и треугольник `(1, 0) (2, 0) (1.5, height)` в 3D, кольцо - отдельной гранью над источником."""

    base = _strip((0.0, 1.0))
    positions = base.positions + (2.0, 0.0, 0.0, 1.5, height, 0.0)
    chart = base.chart + (2.0, 0.0, 1.5, 0.5)
    corners = base.corners + (1, 4, 5)
    stations = base.corner_station + (1.0, 2.0, 1.5)
    return dataclasses.replace(
        base, positions=positions, chart=chart, r=base.r + (0.0, 0.5), s=base.s + (2.0, 1.5), source=base.source + (True, False),
        ring_starts=base.ring_starts + (len(corners),), corners=corners, corner_station=stations,
    )


def test_a_triangle_without_area_weighs_nothing_by_name():
    result = unfold(_with_sliver(0.0))
    numbers = _counters(result)
    assert numbers[uv_unfold.DEGENERATE] == 1
    # Вершины без треугольника с весом - не неизвестные острова: станция как есть, названы.
    assert numbers[uv_unfold.ISOLATED] == 2 and result.held[4:] == (True, True) and result.u[4:] == (2.0, 1.5)


def test_a_thin_triangle_above_the_bound_is_solved():
    result = unfold(_with_sliver(1e-6))
    numbers = _counters(result)
    assert uv_unfold.DEGENERATE not in numbers and uv_unfold.ISOLATED not in numbers
    assert numbers[uv_unfold.TRIANGLES] == 3 and not result.held[5]
    assert uv_unfold.SLOPE_SKIPPED not in numbers and result.slope_x[5] != 0.0


def test_a_triangle_flat_in_the_chart_gives_no_slope_by_name():
    """Треугольник с площадью в 3D, но на одной прямой в карте: решается, а в наклон живой ширины не идёт (деление на ноль), и это названо."""

    data = _with_sliver(1e-6)
    data = dataclasses.replace(data, chart=data.chart[:-1] + (0.0,))
    result = unfold(data)
    numbers = _counters(result)
    assert numbers[uv_unfold.SLOPE_SKIPPED] == 1 and not result.held[5]
    assert (result.slope_x[5], result.slope_y[5]) == (0.0, 0.0)
    assert all(math.isfinite(value) and abs(value) < 10.0 for value in result.slope_x + result.slope_y)


# --------------------------------------------------------------------------
# UNFOLD_STRETCH_GUARD_V1
# --------------------------------------------------------------------------


def test_a_squeezed_wedge_is_reweighted_by_name():
    """Источник длиной 3 м со станциями через 1: держится, поэтому вдоль цепи треугольники у него сжаты втрое - вне `[GUARD_FLOOR, GUARD_CEILING]`."""

    result = unfold(_strip((0.0, 3.0, 6.0), source_s=(0.0, 1.0, 2.0)))
    assert _counters(result)[uv_unfold.GUARDED] > 0
    assert unfold(_strip((0.0, 3.0, 6.0), source_s=(0.0, 1.0, 2.0), guard_rounds=0)).counters != result.counters


def test_a_strip_within_the_guard_is_not_reweighted():
    result = unfold(_strip((0.0, 1.0, 2.0), source_s=(0.0, 1.1, 2.2)))
    assert uv_unfold.GUARDED not in _counters(result)


# --------------------------------------------------------------------------
# Выворот
# --------------------------------------------------------------------------


def test_a_flipped_triangle_is_counted_with_its_area():
    """Станции источника против хода цепи: держатся, верх идёт по геометрии, треугольники у источника вывернуты и названы, не исправлены молча."""

    result = unfold(_strip((0.0, 1.0, 2.0), source_s=(0.0, 1.0, 0.2)))
    numbers = _counters(result)
    assert numbers[uv_unfold.FLIPPED] >= 1 and numbers[uv_unfold.FLIPPED_AREA] > 0


# --------------------------------------------------------------------------
# UNFOLD_NATIVE_EQUALITY_V1
# --------------------------------------------------------------------------


def test_answers_within_the_native_equality_agree():
    result = unfold(_strip((0.0, 1.0, 2.0)))
    near = dataclasses.replace(result, u=tuple(value + NATIVE_EQUALITY / 2 for value in result.u))
    assert results_agree(result, result) and results_agree(result, near)


def test_answers_beyond_the_native_equality_or_with_other_numbers_disagree():
    result = unfold(_strip((0.0, 1.0, 2.0)))
    far = dataclasses.replace(result, slope_x=tuple(value + 4 * NATIVE_EQUALITY for value in result.slope_x))
    other = dataclasses.replace(result, counters=result.counters + ((uv_unfold.FLIPPED, 1),))
    held = dataclasses.replace(result, held=(not result.held[0],) + result.held[1:])
    assert not results_agree(result, far) and not results_agree(result, other) and not results_agree(result, held)


# --------------------------------------------------------------------------
# Закон в материализации
# --------------------------------------------------------------------------


def test_the_source_is_the_chain_length_and_v_is_r_over_alpha():
    snapshot, request = _snapshot()
    prepared = _prepared(snapshot, request)
    result, counters = _materialized(prepared, request)
    facts = [fact for face in result.batch.faces for fact in face.uv_facts]
    length = 10.0 + math.sqrt(104.0)
    base = sorted(float(fact.uv.u) for fact in facts if float(fact.uv.v) == 0.0)
    assert base[0] == pytest.approx(0.0, abs=1e-12) and base[-1] == pytest.approx(length, abs=1e-9)
    assert max(float(fact.uv.v) for fact in facts) == pytest.approx(1.0, abs=1e-12)
    assert counters[uv_unfold.ISLANDS] >= 1 and counters.get(uv_unfold.FLIPPED, 0) == 0
