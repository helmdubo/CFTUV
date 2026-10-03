"""Допуски шума решётки перед резкой (`materialize/clip_snap.py`): вершина `src:` в угол, знак вершины `node:` у ребра.

Фикстура — квадрат 400x400 ячеек решётки, разрезанный диагональю `(0,0)-(400,400)` на `t0` (под диагональю) и `t1`
(над ней); 3D — плоскость `z = 0`, сотая доля метра на ячейку, поэтому один квадрат расстояния в ячейках — `1e-4`
квадратных метров. Числа, на которых стоят утверждения, посчитаны НЕ проверяемым кодом: `gap^2 = 3^2 + 1^2 = 10`
ячеек, `sqrt(10) / 100 м = 31 622 777 нм` (с округлением вверх), перекладина с отступом 0.3 ячейки от диагонали даёт иглу
высотой `0.3 / sqrt(2)` ячейки.

| что проверяется                                                          | тест |
|--------------------------------------------------------------------------|------|
| `src:` в трёх ячейках от угла встаёт в угол, число и нанометры записаны    | `..._within_the_gap_of_a_corner_snaps_to_it` |
| дальше допуска вершина не трогается, прежний путь назван                   | `..._beyond_the_gap_keeps_its_point` |
| угол занят либо два `src:` метят в один — привязки нет, названо            | `..._a_taken_corner_...` |
| два угла в допуске — вершина не знает, чья она, привязки нет, названо      | `..._two_corners_in_the_gap_...` |
| многоугольник, висевший за карту на ячейки, после привязки замкнут точно   | `..._an_overhanging_source_vertex_...` |
| `node:` в долях ячейки от внутреннего ребра — игла не рождается            | `..._within_the_gap_of_an_interior_edge_...` |
| дальше допуска — игла как была; `src:` допуска ребра не имеет              | `..._beyond_the_gap_...`, `..._a_source_vertex_...` |
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1, exact_work_budget
from cftuv_envelope.materialize import clip, clip_snap
from cftuv_envelope.materialize.clip import ClipStageV1
from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1

POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
SIDE = 400


def budget():
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="clip-snap")


def point(x, y):
    return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))


def square_lift():
    """Квадрат 400x400 ячеек с диагональю `(0,0)-(400,400)`, 3D `z = 0`, 1 ячейка = 0.01 м."""

    def top(x, y):
        return (Fraction(x, 100), Fraction(y, 100), Fraction(0))

    return SurfaceLiftV1.from_triangles(
        [
            ("t0", ((0, 0), (SIDE, 0), (SIDE, SIDE)), (top(0, 0), top(SIDE, 0), top(SIDE, SIDE))),
            ("t1", ((0, 0), (SIDE, SIDE), (0, SIDE)), (top(0, 0), top(SIDE, SIDE), top(0, SIDE))),
        ],
        scale=100,
    ).bind(budget())


def stage_and_result(vertices, ring):
    """`(стадия, итог)` резки одного многоугольника `ring` (ключи по порядку) на квадрате."""

    points = {key: point(*xy) for key, xy in vertices.items()}
    cycles = [[(key, points[key]) for key in ring]]
    polygons = [(tuple(ring),)]
    stage = ClipStageV1(square_lift(), budget(), points)
    return stage, stage.run(cycles, polygons, POLYGONS)


def counters_of(item):
    return dict(item.counters)


# --------------------------------------------------------------------------
# Закон 1: вершина `src:` в угол карты
# --------------------------------------------------------------------------


def test_a_source_vertex_within_the_gap_of_a_corner_snaps_to_it():
    points = {"src:a": point(SIDE - 3, 1), "node:b": point(100, 300)}
    snap = clip_snap.snap_source_vertices(square_lift(), budget(), points)
    assert list(snap.moved) == ["src:a"]
    assert tuple(axis.as_rational() for axis in snap.moved["src:a"]) == (SIDE, 0)
    assert snap.points["node:b"] is points["node:b"]
    found = dict(snap.counters)
    assert found[clip_snap.SOURCE_VERTICES_SNAPPED] == 1
    # `gap^2 = 3^2 + 1^2 = 10` ячеек, ячейка 0.01 м: `sqrt(10) / 100 = 0.0316227766 м`, нанометры вверх.
    assert found[clip_snap.SOURCE_VERTEX_SNAP_MAX_GAP] == 31_622_777
    assert found[clip_snap.SOURCE_VERTEX_SNAP_MAX_GAP_CELLS] == 3163  # sqrt(10) = 3.1623 ячейки, тысячные вверх
    assert found[clip_snap.SOURCE_VERTEX_SNAP_REFUSED_TAKEN] == found[clip_snap.SOURCE_VERTEX_SNAP_REFUSED_AMBIGUOUS] == 0


@pytest.mark.parametrize("offset", ((SIDE - 5, 0), (SIDE - 4, 1), (SIDE + 3, -3)), ids=("5-cells", "4.12-cells", "4.24-cells"))
def test_a_source_vertex_beyond_the_gap_keeps_its_point(offset):
    points = {"src:a": point(*offset)}
    snap = clip_snap.snap_source_vertices(square_lift(), budget(), points)
    assert not snap.moved and snap.points["src:a"] is points["src:a"]
    assert dict(snap.counters)[clip_snap.SOURCE_VERTICES_SNAPPED] == 0
    assert dict(snap.counters)[clip_snap.SOURCE_VERTEX_SNAP_MAX_GAP] == 0


def test_a_source_vertex_exactly_at_the_gap_snaps_and_a_vertex_on_a_corner_is_left_alone():
    # `(SIDE - 4, 0)` отстоит от угла ровно на четыре ячейки: допуск включает границу; вершина в углу не двигается.
    points = {"src:edge": point(SIDE - 4, 0), "src:corner": point(0, SIDE)}
    snap = clip_snap.snap_source_vertices(square_lift(), budget(), points)
    assert list(snap.moved) == ["src:edge"]
    assert dict(snap.counters)[clip_snap.SOURCE_VERTEX_SNAP_REFUSED_AMBIGUOUS] == 0


@pytest.mark.parametrize(
    "vertices",
    (
        {"src:a": point(SIDE - 3, 1), "src:b": point(SIDE - 1, 2)},
        {"src:a": point(SIDE - 3, 1), "node:b": point(SIDE, 0)},
    ),
    ids=("two-source-vertices-aim-at-one-corner", "the-corner-holds-another-vertex"),
)
def test_a_taken_corner_is_never_merged_into_a_vertex(vertices):
    snap = clip_snap.snap_source_vertices(square_lift(), budget(), vertices)
    assert not [key for key in snap.moved if key.startswith("src:a")]
    assert dict(snap.counters)[clip_snap.SOURCE_VERTEX_SNAP_REFUSED_TAKEN] >= 1
    assert all(snap.points[key] is vertices[key] for key in vertices if key not in snap.moved)


def test_two_corners_in_the_gap_leave_the_vertex_unsnapped_and_named():
    narrow = SurfaceLiftV1.from_triangles(
        [
            ("t0", ((0, 0), (6, 0), (0, SIDE)), ((0, 0, 0), (Fraction(6, 100), 0, 0), (0, 4, 0))),
            ("t1", ((6, 0), (6, SIDE), (0, SIDE)), ((Fraction(6, 100), 0, 0), (Fraction(6, 100), 4, 0), (0, 4, 0))),
        ],
        scale=100,
    ).bind(budget())
    snap = clip_snap.snap_source_vertices(narrow, budget(), {"src:a": point(3, 0)})
    assert not snap.moved
    assert dict(snap.counters)[clip_snap.SOURCE_VERTEX_SNAP_REFUSED_AMBIGUOUS] == 1


def test_an_overhanging_source_vertex_no_longer_leaves_the_polygon_as_ears(monkeypatch):
    """Вершина `src:` в трёх ячейках ЗА углом карты: без привязки площади не сходятся (свес), с ней — режется."""

    vertices = {
        "src:o": (0, 0),
        "src:a": (SIDE + 3, -1),
        "src:c": (SIDE, SIDE),
        "src:d": (0, SIDE),
    }
    ring = ["src:o", "src:a", "src:c", "src:d"]
    stage, snapped = stage_and_result(vertices, ring)
    found = counters_of(snapped)
    assert found[clip.FACES_OVERHANG] == found[clip.FACES_OFF_CORNER_SUPPRESSED] == found[clip.FACES_SEAM_SUPPRESSED] == 0
    assert found[clip_snap.SOURCE_VERTICES_SNAPPED] == 1
    assert sorted(len(item) for item in snapped.polygons[0]) == [3, 3]
    assert list(snapped.snapped) == ["src:a"]
    # Контроль: тот же многоугольник без привязки остаётся ушами и назван (прежний путь).
    monkeypatch.setattr(
        clip, "snap_source_vertices", lambda plane, spend, points: clip_snap.CornerSnapV1(dict(points), {}, ())
    )
    _stage, control = stage_and_result(vertices, ring)
    assert counters_of(control)[clip.FACES_OFF_CORNER_SUPPRESSED] + counters_of(control)[clip.FACES_OVERHANG] == 1
    assert not control.snapped


# --------------------------------------------------------------------------
# Закон 2: знак вершины `node:` у прямой внутреннего ребра
# --------------------------------------------------------------------------

#: Прямоугольник вдоль диагонали, чья перекладина `a -> b` идёт на 0.3 по y ниже диагонали: игла в `t0`.
NEEDLE_RING = ["node:a", "node:b", "node:c", "node:d"]


def needle_vertices(prefix: str, offset: Fraction):
    return {
        f"{prefix}:a": (100, 100),
        f"{prefix}:b": (200, 200 - offset),
        f"{prefix}:c": (200, 300),
        f"{prefix}:d": (100, 300),
    }


def test_a_node_within_the_gap_of_an_interior_edge_makes_no_needle():
    vertices = needle_vertices("node", Fraction(3, 10))
    _stage, result = stage_and_result(vertices, NEEDLE_RING)
    found = counters_of(result)
    assert [len(item) for item in result.polygons[0]] == [4]
    assert not result.points
    assert found[clip.FACES_IN_ONE_TRIANGLE] == 1 and found[clip.FACES_CUT] == 0
    assert found[clip.FACES_OVERHANG] == found[clip.FACES_BOUNDARY_MISMATCH] == 0
    assert found[clip_snap.NODE_SIGNS_ZEROED] >= 1
    # Расстояние `0.3 / sqrt(2) = 0.2121` ячейки: нанометры вверх, ячейка 0.01 м -> 2.12 мм = 2 121 321 нм.
    assert 2_000_000 < found[clip_snap.NODE_EDGE_GAP_MAX] < 2_200_000
    assert 212 <= found[clip_snap.NODE_EDGE_GAP_MAX_CELLS] <= 213  # 0.2121 ячейки, тысячные вверх


def test_a_node_beyond_the_gap_of_an_interior_edge_still_cuts_a_needle():
    # Отступ 1.5 по y: расстояние от диагонали `1.5 / sqrt(2) = 1.06 > 1` ячейки.
    vertices = needle_vertices("node", Fraction(3, 2))
    _stage, result = stage_and_result(vertices, NEEDLE_RING)
    found = counters_of(result)
    assert sorted(len(item) for item in result.polygons[0]) == [3, 4]
    assert list(result.points) == ["clip:0"]
    assert found[clip_snap.NODE_SIGNS_ZEROED] == 0 and found[clip_snap.NODE_EDGE_GAP_MAX] == 0


def test_a_source_vertex_has_no_edge_gap_it_stays_exact():
    """Допуск знака — закон вершин `node:`: у `src:` и `clip:` знаки точные."""

    vertices = needle_vertices("src", Fraction(3, 10))
    _stage, result = stage_and_result(vertices, ["src:a", "src:b", "src:c", "src:d"])
    found = counters_of(result)
    assert found[clip_snap.NODE_SIGNS_ZEROED] == 0
    # Вершины `src:` вне углов карты: прежний путь (уши под счётчиком), допуск ребра им не положен.
    assert found[clip.FACES_OFF_CORNER_SUPPRESSED] == 1 and not result.points
    assert [len(item) for item in result.polygons[0]] == [3, 3]


def test_the_edge_gap_never_moves_a_vertex_and_the_pieces_cover_the_polygon_exactly():
    vertices = needle_vertices("node", Fraction(3, 10))
    stage, result = stage_and_result(vertices, NEEDLE_RING)
    assert not result.snapped
    cycle_points = {key: point for key, point in result.cycles[0]}
    for key, xy in vertices.items():
        assert tuple(axis.as_rational() for axis in cycle_points[key]) == (Fraction(xy[0]), Fraction(xy[1]))
    # Площади сошлись ТОЧНО (иначе грань осталась бы ушами под счётчиком свеса).
    assert counters_of(result)[clip.FACES_OVERHANG] == 0


def test_the_cell_estimate_is_an_upper_bound_of_the_local_stretch_of_the_lift_not_the_shortest_edge():
    """Склон растягивает ячейку вдоль него: `J = [[1, 0], [0, 1], [1, 0]]`, `J^T J = diag(2, 1)`, `sigma^2 = 2`."""

    sloped = SurfaceLiftV1.from_triangles(
        [("t0", ((0, 0), (10, 0), (0, 10)), ((0, 0, 0), (10, 0, 10), (0, 10, 0)))], scale=1
    ).bind(budget())
    assert sloped.stretch_square(sloped.triangles[0]) == 2
    # Плоский квадрат с ячейкой 0.01 м: ровно `1e-4` в обоих треугольниках, корень точный.
    flat = square_lift()
    assert [flat.stretch_square(item) for item in flat.triangles] == [Fraction(1, 10_000)] * 2
    # Косая карта: `J = [[1, -1], [0, 1], [0, 0]]`, `J^T J = [[1, -1], [-1, 2]]`, `sigma^2 = (3 + sqrt 5) / 2 = 2.618...`.
    skew = SurfaceLiftV1.from_triangles(
        [("t0", ((0, 0), (10, 0), (10, 10)), ((0, 0, 0), (10, 0, 0), (0, 10, 0)))], scale=1
    ).bind(budget())
    assert Fraction(26180, 10_000) < skew.stretch_square(skew.triangles[0]) < Fraction(26190, 10_000)
