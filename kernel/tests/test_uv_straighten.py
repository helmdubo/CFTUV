"""Закон `STRAIGHTEN_RUNGS_V1`: перекладина от вершины цепи и торец острова вертикальны, `v` не трогается, выворот назван (числа посчитаны из входа фикстуры `test_corner_join`)."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize.lift import sqrt_sum_binary64
from cftuv_envelope.materialize.uv_straighten import FLIP_LEFT, FLIP_MOVED, UV_ZERO_AREA, _ears, _repair_flips, _Side, _Tally
from test_corner_join import _materialized, _prepared, _snapshot


def _exact(number: float) -> SqrtSumV1:
    from fractions import Fraction

    return SqrtSumV1.rational(Fraction(number))


def test_the_rung_and_both_island_ends_are_vertical_and_v_is_r_over_alpha():
    snapshot, request = _snapshot()
    prepared = _prepared(snapshot, request)
    result, _counters = _materialized(prepared, request)
    values = [(fact.uv.u, fact.uv.v) for face in result.batch.faces for fact in face.uv_facts]
    length = 10.0 + math.sqrt(104.0)
    # База: u - накопленная длина цепи; источник на v = 0.
    assert sorted(item[0] for item in values if item[1] == pytest.approx(0.0, abs=1e-12))[:1] == [pytest.approx(0.0, abs=1e-12)]
    top = [item for item in values if item[1] == pytest.approx(1.0, abs=1e-12)]
    # Торцы и перекладина над изломом: на верхнем фронте те же u, что у концов и у вершины цепи (в пределах 1e-9).
    for station in (0.0, 10.0, length):
        assert any(abs(item[0] - station) < 1e-9 for item in top), (station, top)
    assert min(item[0] for item in values) == pytest.approx(0.0, abs=1e-9)
    assert max(item[0] for item in values) == pytest.approx(length, abs=1e-9)


def test_a_side_above_its_end_continues_vertically():
    side = _Side([(0.0, 0.0, 5.0, None), (1.0, 0.5, 5.0, None)])
    assert side.at(0.5) == (0.25, 5.0, 0.5, 0.0)
    assert side.at(2.0) == (0.5, 5.0, 0.0, 0.0)


def test_the_ears_of_a_concave_ring_lie_inside_it():
    points = {"a": (0.0, 0.0), "b": (4.0, 0.0), "c": (4.0, 4.0), "d": (2.0, 1.0), "e": (0.0, 4.0)}
    ring = list(points)
    ears, sign = _ears(ring, points.__getitem__)
    assert sign == 1.0 and len(ears) == 3
    twice = 0.0
    for a, b, c in ears:
        pa, pb, pc = (points[ring[item]] for item in (a, b, c))
        area = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])
        assert area > 0.0
        twice += area
    assert twice == pytest.approx(2.0 * 10.0)


class _Layout:
    @staticmethod
    def region_of(_face) -> int:
        return 0


def _scene(values_uv: dict, rings: list) -> tuple:
    values = {(0, key): (_exact(u), _exact(r)) for key, (u, r) in values_uv.items()}
    forms = {slot: ("kept",) for slot in values}
    return values, forms, [SimpleNamespace()] * len(rings), rings


def test_a_flipped_triangle_moves_the_vertex_above_the_base_and_is_named():
    chart = {"a": (0.0, 0.0), "b": (2.0, 0.0), "t": (1.0, 1.0), "q": (3.0, 0.5)}
    # `t` ушла за `q`: вывернут `b q t`, основание `a b` на месте.
    values, forms, frames, rings = _scene({"a": (0.0, 0.0), "b": (2.0, 0.0), "t": (4.5, 1.0), "q": (3.0, 0.5)}, [[["a", "b", "t"]], [["b", "q", "t"]]])
    tally = _Tally()
    _repair_flips(frames, _Layout, rings, chart.__getitem__, values, forms, tally)
    numbers = dict(tally.counters())
    assert numbers[FLIP_MOVED] == 1 and FLIP_LEFT not in numbers
    assert forms[(0, "t")] != ("kept",) and forms[(0, "a")] == forms[(0, "b")] == forms[(0, "q")] == ("kept",)
    # Наименьший сдвиг: до границы допустимого `u <= 4` (прямая `b q` при v = 1) с запасом, `v` не тронут.
    assert values[(0, "t")][1] == _exact(1.0)
    assert float(sqrt_sum_binary64(values[(0, "t")][0])) < 4.0


def test_a_flip_with_only_base_vertices_is_left_by_name():
    chart = {"a": (0.0, 0.0), "b": (2.0, 0.0), "w": (1.0, 1.0), "c": (1.0, -1.0)}
    # Основание не двигается никогда: вывернутое кольцо `a b c` на v = 0 и 0.5 у `c`; у неё другая сторона, выше основания только она, и выворот ей исправим - поэтому здесь `c` в основании.
    values, forms, frames, rings = _scene({"a": (0.0, 0.0), "b": (2.0, 0.0), "w": (1.0, 1.0), "c": (9.0, 0.0)}, [[["a", "b", "w"]], [["w", "b", "c"]]])
    values[(0, "w")] = (_exact(1.0), _exact(1.0))
    tally = _Tally()
    _repair_flips(frames, _Layout, rings, {**chart, "c": (3.0, 1.0)}.__getitem__, values, forms, tally)
    assert FLIP_LEFT not in dict(tally.counters())
    assert forms[(0, "a")] == forms[(0, "b")] == forms[(0, "c")] == ("kept",)


def test_three_vertices_of_one_column_are_zero_area_by_name_and_stay():
    chart = {"a": (0.0, 0.0), "b": (1.0, 0.5), "c": (0.0, 1.0), "w": (-3.0, 0.5)}
    values, forms, frames, rings = _scene({"a": (1.0, 0.0), "b": (1.0, 0.5), "c": (1.0, 1.0), "w": (0.0, 0.5)}, [[["a", "b", "c"]], [["a", "c", "w"]]])
    tally = _Tally()
    _repair_flips(frames, _Layout, rings, chart.__getitem__, values, forms, tally)
    numbers = dict(tally.counters())
    assert numbers[UV_ZERO_AREA] == 1 and FLIP_MOVED not in numbers
