"""Закон подъёма `SOURCE_FACES_CLIPPED_V1`: режет только по настоящим рёбрам меша, диагональ грани не режет.

Диагональ четырёхгранья — ребро триангуляции хоста (`calc_loop_triangles`), а не меша источника: у неё нет
`physical_edge_id`. Закон `SOURCE_TRIANGLES_CLIPPED_V1` резал по ней и рождал рёбра декали, которых в источнике
нет (замер `sagging_wall`: 28 вершин резки на диагоналях). Здесь областью резки служит ЯЧЕЙКА — замкнутая выпуклая
грань источника; кусок над диагональю непланарной грани имеет глубину хорды (`clip_cells.chord_of`), и допуск
`CLIP_DIAGONAL_CHORD_BUDGET` решает, остаётся ли грань целой.

Стадия (`materialize.clip`) проверяется на гранях-четырёхугольниках с настраиваемой высотой углов: планарная грань
не режется никогда, грань в допуске остаётся одним куском поперёк диагонали, глубже допуска режется по своим
треугольникам ровно как прежний закон и названа счётчиком, оценка глубины звучная (не меньше выборки и почти
равна ей), настоящее ребро между гранями режет по-прежнему, прямая вершина грани и невыпуклая грань не рвут сетку.
Домен целиком — на тех же складках, что у `test_clip_law`: ни одной вершины резки вне настоящего ребра меша.
"""

from __future__ import annotations

import math
from fractions import Fraction

import pytest

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1, exact_work_budget
from cftuv_envelope.materialize import clip, clip_cells
from cftuv_envelope.materialize.clip import ClipStageV1, _cut_by_faces
from cftuv_envelope.materialize.clip_cells import (
    CLIP_DIAGONAL_CHORD_BUDGET,
    build_cells,
    nanometres,
)
from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1

from developable_route import materialize_developable
from test_clip_law import (
    DOMAINS,
    centroid,
    closest_distance,
    directed_edges,
    face_points,
    triangles_3d,
)
import developable_factories as df

POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
BY_TRIANGLES = NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1
BY_FACES = NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1


def budget():
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="clip-faces")


def point(x, y):
    return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))


def lift_of(faces):
    """Подъём по граням `[(имя, [(x, y), ...], [(X, Y, Z), ...], ((i, j, k), ...)), ...]`; треугольники — по индексам."""

    items = []
    for name, chart, corners, triangles in faces:
        for number, (i, j, k) in enumerate(triangles):
            items.append(
                (
                    f"{name}.t{number}",
                    (chart[i], chart[j], chart[k]),
                    tuple(tuple(Fraction(axis) for axis in corners[index]) for index in (i, j, k)),
                    (),
                    name,
                )
            )
    return SurfaceLiftV1.from_triangles(items, scale=4)


FAN4 = ((0, 1, 2), (0, 2, 3))


def square(name, x0, y0, heights, *, size=4):
    """Квадрат `[x0, x0 + size] x [y0, y0 + size]` с высотами углов по обходу, диагональ 0-2."""

    chart = [(x0, y0), (x0 + size, y0), (x0 + size, y0 + size), (x0, y0 + size)]
    corners = [(x, y, Fraction(h)) for (x, y), h in zip(chart, heights)]
    return (name, chart, corners, FAN4)


def run(lift, polygons_xy, *, by_faces=True, law=POLYGONS):
    """`(результат, ключи)`: стадия закона по граням (либо по треугольникам) на многоугольниках `[[(x, y), ...], ...]`."""

    spend = budget()
    keys, points, cycles, polygons = {}, {}, [], []
    for polygon in polygons_xy:
        cycle = []
        for xy in polygon:
            if xy not in keys:
                keys[xy] = f"p{len(keys)}"
                points[keys[xy]] = point(*xy)
            cycle.append((keys[xy], points[keys[xy]]))
        cycles.append(cycle)
        polygons.append((tuple(key for key, _point in cycle),))
    plane = lift.bind(spend)
    fans = [False] * len(polygons_xy)
    if by_faces:
        result = _cut_by_faces(plane, spend, points, cycles, polygons, law, frozenset(), fans)
    else:
        result = ClipStageV1(plane, spend, points).run(cycles, polygons, law, frozenset(), fans)
    return result, keys


def counters(result):
    return dict(result.counters)


RECTANGLE = [[(1, 1), (3, 1), (3, 2), (1, 2)]]


# --------------------------------------------------------------------------
# Диагональ и допуск хорды
# --------------------------------------------------------------------------


def test_a_planar_face_is_never_cut_by_its_diagonal():
    """Точно планарная грань (в том числе наклонная): прямоугольник поперёк диагонали — одна грань, вершин нет."""

    for heights in ((0, 0, 0, 0), (0, 1, 2, 1), (3, 3, 3, 3)):
        result, _keys = run(lift_of([square("f0", 0, 0, heights)]), RECTANGLE)
        assert [len(item) for item in result.polygons[0]] == [4], heights
        assert not result.points and not result.lifted
        found = counters(result)
        assert found[clip_cells.DIAGONAL_FACES_WHOLE] == 1
        assert found[clip_cells.DIAGONAL_PIECES_ACROSS] == 1 and found[clip_cells.DIAGONAL_CUTS_AVOIDED] == 1
        assert found[clip_cells.DIAGONAL_KEPT_NOT_PLANAR] == found[clip_cells.DIAGONAL_MAX_CHORD_KEPT] == 0
    # Прежний закон резал бы: на карте две области.
    old, _keys = run(lift_of([square("f0", 0, 0, (0, 0, 0, 0))]), RECTANGLE, by_faces=False)
    assert sorted(len(item) for item in old.polygons[0]) == [3, 4] and "clip:0" in old.points


def test_a_piece_within_the_chord_budget_stays_one_face_across_the_diagonal():
    """Допуск — положительный случай: 1 мм излома, кусок целиком, вершины резки нет, глубина записана и в допуске."""

    lift = lift_of([square("f0", 0, 0, (0, 0, Fraction(1, 1000), 0))])
    result, _keys = run(lift, RECTANGLE)
    assert [len(item) for item in result.polygons[0]] == [4]
    assert not result.points and not result.lifted
    found = counters(result)
    assert found[clip_cells.DIAGONAL_FACES_WHOLE] == 1
    assert found[clip_cells.DIAGONAL_KEPT_NOT_PLANAR] == 0 and found[clip_cells.DIAGONAL_KEPT_NOT_CONVEX] == 0
    depth = found[clip_cells.DIAGONAL_MAX_CHORD_KEPT]
    assert 0 < depth <= nanometres(CLIP_DIAGONAL_CHORD_BUDGET**2)
    assert found[clip.FACES_IN_ONE_TRIANGLE] == 1 and found[clip.FACES_CUT] == 0
    assert f"max_chord_kept_nm={depth}" in result.note and "chord_budget_nm=5000000" in result.note


def test_a_piece_beyond_the_chord_budget_cuts_the_face_by_its_triangles_and_names_it():
    """Допуск — граница: 0.1 м излома, грань режется по диагонали ровно как прежний закон, счётчик назван, глубина записана."""

    lift = lift_of([square("f0", 0, 0, (0, 0, Fraction(1, 10), 0))])
    result, _keys = run(lift, RECTANGLE)
    old, _old_keys = run(lift, RECTANGLE, by_faces=False)
    assert result.polygons == old.polygons and list(result.points) == list(old.points) == ["clip:0"]
    assert result.lifted == old.lifted
    found = counters(result)
    assert found[clip_cells.DIAGONAL_KEPT_NOT_PLANAR] == 1
    assert found[clip_cells.DIAGONAL_FACES_WHOLE] == found[clip_cells.DIAGONAL_CUTS_AVOIDED] == 0
    assert found[clip_cells.DIAGONAL_MAX_CHORD_OVER] > nanometres(CLIP_DIAGONAL_CHORD_BUDGET**2)
    assert "faces_cut_not_planar=1" in result.note


def test_the_budget_is_a_quarter_of_the_host_offset():
    host_offset = Fraction(2, 100)  # `cftuv.envelope_production_mesh.DEFAULT_DECAL_OFFSET`, метры
    assert CLIP_DIAGONAL_CHORD_BUDGET == host_offset / 4 == Fraction(1, 200)
    assert nanometres(CLIP_DIAGONAL_CHORD_BUDGET**2) == 5_000_000


def test_splitting_every_cell_reproduces_the_triangle_law_bitwise():
    """Расщеплённые все ячейки — это треугольники один в один: резка побитово та же, что у прежнего закона."""

    lift = lift_of([square("f0", 0, 0, (0, 0, Fraction(1, 10), 0)), square("f1", 4, 0, (0, Fraction(1, 7), 0, 0))])
    plan = build_cells(lift.triangles)
    assert len(plan.cells) == 2 and all(len(cell.members) == 2 for cell in plan.cells)
    split = frozenset(cell.key for cell in plan.cells)
    assert [cell.name for cell in build_cells(lift.triangles, split).cells] == [item.name for item in lift.triangles]
    faces = [[(1, 1), (3, 1), (3, 2), (1, 2)], [(3, 1), (5, 1), (5, 3), (3, 3)], [(1, 3), (3, 3), (3, 4), (1, 4)]]
    new, _keys = run(lift, faces)
    old, _old_keys = run(lift, faces, by_faces=False)
    assert new.polygons == old.polygons and new.cycles == old.cycles
    assert list(new.points) == list(old.points) and new.lifted == old.lifted


# --------------------------------------------------------------------------
# Оценка глубины хорды звучная и почти точная
# --------------------------------------------------------------------------


def surface_point(bound, xy):
    return bound.lift_exact(point(*xy))


def sampled_depth(lift, vertices):
    """Наибольшее отклонение выпуклой комбинации ДВУХ вершин куска от поверхности над той же точкой карты, метры."""

    bound = lift.bind(budget())
    lifted = {xy: tuple(float(axis.as_rational()) for axis in surface_point(bound, xy)) for xy in vertices}
    worst = 0.0
    for first in vertices:
        for second in vertices:
            for step in range(0, 2001):
                t = Fraction(step, 2000)
                xy = tuple((1 - t) * a + t * b for a, b in zip(first, second))
                on_surface = tuple(float(axis.as_rational()) for axis in surface_point(bound, xy))
                chord = tuple((1 - float(t)) * a + float(t) * b for a, b in zip(lifted[first], lifted[second]))
                worst = max(worst, math.dist(on_surface, chord))
    return worst


@pytest.mark.parametrize(
    "heights, piece",
    [
        ((0, 0, Fraction(1, 1000), 0), [(1, 1), (3, 1), (3, 2), (1, 2)]),
        ((0, Fraction(1, 500), 0, 0), [(1, 2), (3, 2), (3, 3), (1, 3)]),
        ((Fraction(1, 300), 0, Fraction(1, 200), 0), [(0, 0), (4, 0), (4, 4), (0, 4)]),
    ],
)
def test_the_chord_depth_is_a_sound_and_tight_bound_of_the_sampled_deviation(heights, piece):
    lift = lift_of([square("f0", 0, 0, heights)])
    result, _keys = run(lift, [piece])
    found = counters(result)
    assert found[clip_cells.DIAGONAL_FACES_WHOLE] == 1
    bound_meters = found[clip_cells.DIAGONAL_MAX_CHORD_KEPT] / 1e9
    sampled = sampled_depth(lift, piece)
    assert sampled <= bound_meters + 1e-12, (sampled, bound_meters)
    # Оценка не раздувает: у четырёхгранья она достигается на паре вершин куска.
    assert sampled >= 0.99 * bound_meters, (sampled, bound_meters)


def test_a_convex_face_of_many_triangles_is_one_cell_with_the_flat_bound():
    """Выпуклый пятиугольник (3 треугольника): одна ячейка, оценка `2ρ` не зависит от размера куска."""

    chart = [(0, 0), (4, 0), (5, 3), (2, 5), (-1, 3)]
    corners = [(x, y, Fraction(h)) for (x, y), h in zip(chart, (0, 0, Fraction(1, 2000), 0, 0))]
    lift = lift_of([("p", chart, corners, ((0, 1, 2), (0, 2, 3), (0, 3, 4)))])
    plan = build_cells(lift.triangles)
    assert len(plan.cells) == 1 and plan.not_convex == ()
    cell = plan.cells[0]
    assert len(cell.members) == 3 and cell.hinge is None and cell.flat_square is not None
    result, _keys = run(lift, [[(1, 1), (3, 1), (3, 2), (1, 2)]])
    assert [len(item) for item in result.polygons[0]] == [4]
    found = counters(result)
    assert found[clip_cells.DIAGONAL_FACES_WHOLE] == 1
    # `(2ρ)^2` — точный квадрат: 4 * (угол над аффинным подъёмом самого большого треугольника)^2.
    assert found[clip_cells.DIAGONAL_MAX_CHORD_KEPT] == nanometres(cell.flat_square)


# --------------------------------------------------------------------------
# Настоящие рёбра меша режутся по-прежнему
# --------------------------------------------------------------------------


def test_the_real_edge_between_two_faces_still_cuts_and_closes_the_mesh():
    lift = lift_of([square("f0", 0, 0, (0, 0, 0, 0)), square("f1", 4, 0, (0, 0, 0, 0))])
    faces = [[(3, 1), (5, 1), (5, 2), (3, 2)], [(3, 2), (5, 2), (5, 3), (3, 3)]]
    result, _keys = run(lift, faces)
    # Ребро `x = 4` общее у граней: вершины резки только на нём; диагоналей (0,0)-(4,4) и (4,0)-(8,4) нет.
    assert {tuple(axis.as_rational() for axis in value) for value in result.points.values()} == {
        (4, 1),
        (4, 2),
        (4, 3),
    }
    assert [len(item) for polygon in result.polygons for item in polygon] == [4, 4, 4, 4]
    edges = directed_edges(result.polygons)
    assert max(edges.values()) == 1
    found = counters(result)
    assert found[clip.FACES_CUT] == 2 and found[clip.FACES_BOUNDARY_MISMATCH] == found[clip.FACES_OVERHANG] == 0


def test_a_straight_vertex_of_a_face_does_not_open_the_boundary_with_its_neighbours():
    """Пятиугольник с прямой вершиной `(2, 0)`: соседи с поворотом в ней, куски сходятся без T-стыка."""

    chart = [(0, 0), (2, 0), (4, 0), (4, 4), (0, 4)]
    top = ("top", chart, [(x, y, Fraction(0)) for x, y in chart], ((1, 2, 3), (0, 1, 3), (0, 3, 4)))
    left_chart = [(0, -4), (2, -4), (2, 0), (0, 0)]
    right_chart = [(2, -4), (4, -4), (4, 0), (2, 0)]
    left = ("left", left_chart, [(x, y, Fraction(0)) for x, y in left_chart], FAN4)
    right = ("right", right_chart, [(x, y, Fraction(0)) for x, y in right_chart], FAN4)
    lift = lift_of([top, left, right])
    plan = build_cells(lift.triangles)
    top_cell = next(cell for cell in plan.cells if cell.name == "top")
    assert len(top_cell.members) == 3 and [point for _edge, point in top_cell.straight] == [(Fraction(2), Fraction(0))]
    result, _keys = run(lift, [[(1, -1), (3, -1), (3, 1), (1, 1)]])
    found = counters(result)
    assert found[clip.FACES_CUT] == 1 and found[clip.FACES_BOUNDARY_MISMATCH] == found[clip.FACES_OVERHANG] == 0
    # Три куска (по одному на грань); вершина `(2, 0)` — у каждого, граница кусков — контур с вершинами рёбер.
    sizes = sorted(len(item) for item in result.polygons[0])
    assert len(sizes) == 3
    edges = directed_edges(result.polygons)
    assert max(edges.values()) == 1
    shared = [key for key, value in result.points.items() if tuple(float(axis.as_rational()) for axis in value) == (2.0, 0.0)]
    assert len(shared) == 1
    assert sum(shared[0] in item for item in result.polygons[0]) == 3


def test_a_non_convex_face_keeps_its_cuts_between_convex_parts_and_is_named():
    """Г-образная грань: две выпуклые части, диагональ между ними режет и названа; внутри части диагонали нет."""

    chart = [(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)]
    lift = lift_of([("L", chart, [(x, y, Fraction(0)) for x, y in chart], ((0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 5)))])
    plan = build_cells(lift.triangles)
    assert [len(cell.members) for cell in plan.cells] == [2, 2] and plan.not_convex == (("L", "NOT_CONVEX"),)
    result, _keys = run(lift, [[(0.5, 0.5), (3, 0.5), (3, 1.5), (0.5, 1.5)]])
    assert counters(result)[clip_cells.DIAGONAL_KEPT_NOT_CONVEX] == 1
    assert "faces_cut_not_convex=1{'NOT_CONVEX': 1}" in result.note


# --------------------------------------------------------------------------
# Закон как значение
# --------------------------------------------------------------------------


def test_the_law_is_a_clipping_law_judged_as_the_surface_law():
    assert BY_FACES.onto_surface and BY_FACES.clips and BY_FACES.clips_by_faces
    assert BY_TRIANGLES.clips and not BY_TRIANGLES.clips_by_faces
    assert BY_FACES.judged_as is BY_TRIANGLES.judged_as is NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1
    assert not NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1.clips and not NearPlanarLiftLawV1.CERTIFIED_PLANE_V1.clips


def test_every_triangle_of_the_snapshot_lift_carries_its_source_face():
    lift = lift_of([square("f0", 0, 0, (0, 0, 0, 0)), square("f1", 4, 0, (0, 0, 0, 0))])
    assert [item.face for item in lift.triangles] == ["f0", "f0", "f1", "f1"]
    # Без грани треугольник сам себе грань: ячейки не склеиваются.
    bare = SurfaceLiftV1.from_triangles(
        [("a", ((0, 0), (4, 0), (4, 4)), ((0, 0, 0), (4, 0, 0), (4, 4, 0))), ("b", ((0, 0), (4, 4), (0, 4)), ((0, 0, 0), (4, 4, 0), (0, 4, 0)))],
        scale=4,
    )
    assert [len(cell.members) for cell in build_cells(bare.triangles).cells] == [1, 1]


# --------------------------------------------------------------------------
# Домен целиком: складки, фаска, цилиндр
# --------------------------------------------------------------------------


def real_edges_3d(parts):
    """Рёбра меша источника (по циклам граней) как отрезки в 3D: диагоналей триангуляции среди них нет."""

    position = {item.vertex_id: item.position for item in parts[0]}
    found = []
    for face in parts[1]:
        cycle = face.vertex_cycle
        for index, vertex in enumerate(cycle):
            a, b = position[vertex], position[cycle[(index + 1) % len(cycle)]]
            found.append(((a.x, a.y, a.z), (b.x, b.y, b.z)))
    return found


def distance_to_segment(p, segment):
    a, b = segment
    ab = tuple(y - x for x, y in zip(a, b))
    ap = tuple(y - x for x, y in zip(a, p))
    length = sum(x * x for x in ab)
    t = 0.0 if not length else max(0.0, min(1.0, sum(x * y for x, y in zip(ab, ap)) / length))
    return math.dist(p, tuple(x + t * y for x, y in zip(a, ab)))


@pytest.fixture(scope="module")
def faces_pair():
    cache = {}

    def get(name):
        if name not in cache:
            make, route, alpha = DOMAINS[name]
            parts = make()
            by_faces, _ = materialize_developable(
                parts, route, alpha=alpha, decal_topology_law=POLYGONS, near_planar_lift_law=BY_FACES
            )
            by_triangles, _ = materialize_developable(
                parts, route, alpha=alpha, decal_topology_law=POLYGONS, near_planar_lift_law=BY_TRIANGLES
            )
            cache[name] = (by_faces, by_triangles, parts)
        return cache[name]

    return get


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_every_clip_vertex_of_the_domain_lies_on_a_real_source_edge(faces_pair, name):
    by_faces, _by_triangles, parts = faces_pair(name)
    assert by_faces.is_materialized, by_faces.detail
    position = {item.vert_key.value: (item.position.x, item.position.y, item.position.z) for item in by_faces.batch.vertices}
    edges = real_edges_3d(parts)
    for key, where in position.items():
        if key.startswith("clip:"):
            assert min(distance_to_segment(where, edge) for edge in edges) < 1e-9, key


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_the_domain_by_faces_has_no_more_faces_than_by_triangles_and_stays_on_the_surface(faces_pair, name):
    by_faces, by_triangles, parts = faces_pair(name)
    assert len(by_faces.batch.faces) <= len(by_triangles.batch.faces)
    keys = lambda batch: {item.vert_key.value for item in batch.vertices}
    assert keys(by_faces.batch) - {k for k in keys(by_faces.batch) if k.startswith("clip:")} == keys(by_triangles.batch) - {
        k for k in keys(by_triangles.batch) if k.startswith("clip:")
    }
    assert len(keys(by_faces.batch)) <= len(keys(by_triangles.batch))
    triangles = triangles_3d(parts)
    budget_meters = float(CLIP_DIAGONAL_CHORD_BUDGET) + 2e-4
    depth = max(
        min(closest_distance(point_, triangle) for triangle in triangles)
        for points in face_points(by_faces.batch)
        for point_ in (*points, centroid(points))
    )
    assert depth <= budget_meters, depth


def test_the_planar_quads_of_a_fold_strip_lose_every_diagonal_cut(faces_pair):
    """Планарные грани складки (диагонали нет в источнике): вершин резки нет вовсе, кроме настоящего ребра складки."""

    by_faces, by_triangles, _parts = faces_pair("slant")
    clip_keys = lambda result: [item.vert_key.value for item in result.batch.vertices if item.vert_key.value.startswith("clip:")]
    assert 0 < len(clip_keys(by_faces)) < len(clip_keys(by_triangles))
