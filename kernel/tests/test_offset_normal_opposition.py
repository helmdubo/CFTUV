"""Допуск глубины противостояния нормали смещения (`SURFACE_OFFSET_OPPOSITION_DEPTH_V1`).

Закон `SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1` отказывал `SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE`, как только нормаль
вершины переставала быть строго «над» нормалью своего треугольника. У треугольника-иголки (патч 89 `building`: угол
при вершине `building:34` 0.17°) вес нормали нуль, косинус -3.08e-4, и смещённая декаль уходит под его плоскость на
`offset * 3.08e-4` — шесть микрон при смещении 2 см. Закон допускает противостояние, если ГРАНИЦА ГЛУБИНЫ
`offset * D / mu` не больше допуска (0.1 мм) на опорном смещении, а допущенное пишет.

Проверяется: ступенька допущена с записью, строгий режим отказывает как прежде; граница действительно покрывает
смесь нормалей по ВСЕМУ треугольнику (выборка барицентрических весов); глубже допуска и без границы отказ остаётся;
счётчики и строка называют худший треугольник и пусты, когда допускать нечего.
"""

from __future__ import annotations

import math
from fractions import Fraction

import pytest

import cftuv_envelope.materialize.offset_normal as offset_normal
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1
from cftuv_envelope.materialize.offset_normal import (
    OFFSET_OPPOSITION_DEPTH_TOLERANCE,
    OFFSET_REFERENCE_METRES,
    OPPOSITION_TOLERATED,
    OPPOSITION_WORST_DEPTH,
    OppositionV1,
    blend,
    opposition_bound,
    opposition_note,
    opposition_totals,
    source_vertex_normals,
)

import developable_factories as factories


def _step():
    vertices, _faces, triangles = factories.step_patch()
    positions = {
        item.vertex_id: tuple(Fraction(a) for a in (item.position.x, item.position.y, item.position.z))
        for item in vertices
    }
    return triangles, positions


def test_the_tolerance_and_the_reference_offset_are_the_declared_ones():
    assert OFFSET_OPPOSITION_DEPTH_TOLERANCE == Fraction(1, 10_000)
    assert OFFSET_REFERENCE_METRES == Fraction(1, 50)


def test_the_strict_law_still_refuses_the_step_patch_needle_by_name():
    triangles, positions = _step()
    with pytest.raises(MaterializationRefusal) as failure:
        source_vertex_normals(triangles, positions)
    assert failure.value.outcome is MaterializationOutcome.SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE
    assert "v:34" in str(failure.value) and "depth bound" in str(failure.value)


def test_the_step_patch_needle_is_tolerated_with_its_depth_recorded():
    triangles, positions = _step()
    tolerated: list = []

    normals = source_vertex_normals(triangles, positions, tolerated)

    assert normals
    (record,) = tolerated
    assert record.vertex == "v:34" and record.triangle.endswith(":t01")
    assert record.cosine == pytest.approx(-3.08e-4, rel=2e-2)
    # Угловая глубина 0.02 * 3.08e-4 = 6.2 мкм; граница делит её на mu (< 1) и остаётся на порядок меньше 0.1 мм.
    assert 0.02 * 3.08e-4 * 0.98 <= record.depth <= 2 * 0.02 * 3.08e-4
    assert record.depth < float(OFFSET_OPPOSITION_DEPTH_TOLERANCE) / 10
    assert dict(opposition_totals(tolerated)) == {
        OPPOSITION_TOLERATED: 1,
        OPPOSITION_WORST_DEPTH: math.ceil(record.depth * 10**9),
    }
    note = opposition_note(tolerated)
    assert record.triangle in note and "v:34" in note and "linearly with the host offset" in note


def test_an_opposition_deeper_than_the_tolerance_keeps_the_refusal(monkeypatch):
    """Красный контроль: тот же угол при допуске в 1 мкм (глубина 6.2 мкм) отказывает, и отказ несёт границу."""

    triangles, positions = _step()
    monkeypatch.setattr(offset_normal, "OFFSET_OPPOSITION_DEPTH_TOLERANCE", Fraction(1, 1_000_000))
    tolerated: list = []

    with pytest.raises(MaterializationRefusal) as failure:
        source_vertex_normals(triangles, positions, tolerated)

    assert failure.value.outcome is MaterializationOutcome.SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE
    assert "depth bound" in str(failure.value)
    assert tolerated == []


def test_a_larger_reference_offset_deepens_the_bound_and_refuses(monkeypatch):
    triangles, positions = _step()
    monkeypatch.setattr(offset_normal, "OFFSET_REFERENCE_METRES", Fraction(1, 2))
    with pytest.raises(MaterializationRefusal):
        source_vertex_normals(triangles, positions, [])


def test_the_bound_is_offset_times_the_worst_cosine_over_the_mean_direction_floor():
    up = ((0.0, 0.0, 1.0),) * 3

    def facing(cosine):
        return (math.sqrt(1.0 - cosine * cosine), 0.0, cosine)

    # Нормаль треугольника лежит почти поперёк нормалей вершин: глубина 0.02 * |cos|.
    shallow = opposition_bound(up, facing(-0.004))
    deep = opposition_bound(up, facing(-0.01))
    assert shallow[0] == pytest.approx(-0.004) and shallow[1] == pytest.approx(0.02 * 0.004)
    assert shallow[2] is True
    assert deep[1] == pytest.approx(0.02 * 0.01) and deep[2] is False
    # Строго над треугольником глубины нет.
    assert opposition_bound(up, (0.0, 0.0, 1.0))[1] == 0.0


def test_normals_without_a_mean_direction_have_no_bound():
    assert opposition_bound(((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0)) is None
    assert opposition_bound(((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0)) is None


def test_the_bound_covers_the_blended_normal_over_the_whole_triangle():
    """Доказательство границы выборкой: глубина смешанной нормали в каждой точке треугольника не больше границы."""

    triangles, positions = _step()
    tolerated: list = []
    normals = source_vertex_normals(triangles, positions, tolerated)
    (record,) = tolerated
    triangle = next(item for item in triangles if item.triangle_id.value == record.triangle)
    corners = [tuple(float(a) for a in positions[v]) for v in triangle.vertex_ids]
    ux = tuple(corners[1][i] - corners[0][i] for i in range(3))
    vx = tuple(corners[2][i] - corners[0][i] for i in range(3))
    cross = (
        ux[1] * vx[2] - ux[2] * vx[1],
        ux[2] * vx[0] - ux[0] * vx[2],
        ux[0] * vx[1] - ux[1] * vx[0],
    )
    length = math.sqrt(sum(a * a for a in cross))
    unit = tuple(a / length for a in cross)
    vertex_normals = [normals[v] for v in triangle.vertex_ids]
    deepest = 0.0
    steps = 80
    for i in range(steps + 1):
        for j in range(steps + 1 - i):
            weights = (i / steps, j / steps, (steps - i - j) / steps)
            mixed = blend(weights, vertex_normals)
            height = float(OFFSET_REFERENCE_METRES) * sum(a * b for a, b in zip(mixed, unit))
            deepest = min(deepest, height)
    assert -deepest <= record.depth * (1.0 + 1e-9)
    assert -deepest > 0.0


def test_a_domain_without_oppositions_records_nothing():
    assert opposition_totals([]) == ()
    assert opposition_note([]) == ""
    triangles, positions = _step()
    tolerated: list = []
    assert source_vertex_normals(triangles[:2], positions, tolerated)
    assert tolerated == []


def test_the_lift_counts_the_tolerated_triangles_only_when_there_are_any():
    plain = SurfaceLiftV1.from_triangles([], 1).bind(None)
    assert not any(name == OPPOSITION_TOLERATED for name, _value in plain.counters())
    assert plain.opposition_note() == ""
    record = OppositionV1("t:1", "v:1", -3e-4, 6.0e-6)
    noted = SurfaceLiftV1.from_triangles([], 1, opposition=(record,)).bind(None)
    assert dict(noted.counters())[OPPOSITION_TOLERATED] == 1
    assert dict(noted.counters())[OPPOSITION_WORST_DEPTH] == 6000
    assert "t:1" in noted.opposition_note()
