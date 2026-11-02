"""WALL_CREEP_V1: частичное выделение швов — торец заподлицо с невыделенным соседом.

Проверяется ФОРМА ответа, а не числа очереди: каждое ребро покрытого контура грани-источника
обязано лежать на одной из названных прямых — своей прямой источника, фронте на глубине alpha,
прямой невыделенного ребра (заподлицо, в том числе продолжение на вогнутом конце), биссектрисе
стыка двух источников или поперечном резе T-надреза. Митр между источником и стеной (ребро
«ниоткуда») — провал. Допуск — на величину ползучести `eps * alpha`, а не на форму.
"""

from __future__ import annotations

import math
from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope.wavefront.bridge import (
    BridgeOutcome,
    PlainArrivalLawV1,
    bridge_arrival_laws,
    unit_speed_laws_of,
)
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton
from cftuv_envelope.wavefront.wall_creep import (
    WALL_CREEP_EPSILON,
    creep_walls,
    creeping_speed,
    drop_wall_faces,
)

from wavefront_cases import named_corpus, partial_source_corpus

PARTIAL = partial_source_corpus()
CORPUS = named_corpus()

#: Сколько фигур корпуса частичного источника ползущие стены доводят до точного разбиения: все
#: 38 фигур, где стены есть (у трёх оставшихся из 41 источник — вся граница). С неподвижными
#: стенами (`q = 0`) 17 из них отказывали `WAVEFRONT_LEFT_UNRESOLVED`
#: (`test_wavefront_partial_source.py`). Ратчет: сдвиг вниз — регресс закона.
FIGURES_CLOSED_BY_CREEPING_WALLS = 38


def _float(value) -> float:
    return float(sum(float(coefficient) * math.sqrt(radicand) for radicand, coefficient in value.terms))


def _distance(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    return abs((point[0] - start[0]) * dy - (point[1] - start[1]) * dx) / math.hypot(dx, dy)


def _laws_with_motion(polygon: PolygonV1):
    """Законы источников единичной карты: направление движения — нормаль ребра влево."""

    return tuple(
        replace(law, motion=(float(law.normal_x), float(law.normal_y)))
        for law in unit_speed_laws_of(polygon)
    )


def _loops_of(polygon: PolygonV1):
    return tuple(tuple((Fraction(x), Fraction(y)) for x, y in loop.points) for loop in polygon.loops)


def _crept(polygon: PolygonV1):
    loops = _loops_of(polygon)
    lattice = tuple(loop.points for loop in polygon.loops)
    return creep_walls(polygon, loops, lattice, _laws_with_motion(polygon))


def _classify(original: PolygonV1, creep, contour_face, alpha: Fraction, perps=()) -> list[str]:
    """Классы рёбер покрытого контура одной грани-источника (см. шапку модуля)."""

    sources, walls = [], []
    for start, end, speed in original.edges():
        (sources if speed else walls).append((start, end, speed))
    start = (contour_face.owner[0], contour_face.owner[1])
    end = (contour_face.owner[2], contour_face.owner[3])
    speed = next(q for s, e, q in sources if (s, e) == (start, end))
    depth = float(alpha) * math.sqrt(float(speed)) / math.hypot(end[0] - start[0], end[1] - start[1])
    tolerance = 1e-4 * max(1.0, depth)
    shared = {v for v in (start, end) if any(v in (s, e) for s, e, _ in sources if (s, e) != (start, end))}
    points = [(_float(x), _float(y)) for x, y in contour_face.points]
    classes = []
    for index, p in enumerate(points):
        r = points[(index + 1) % len(points)]
        if math.hypot(r[0] - p[0], r[1] - p[1]) < tolerance:
            classes.append("TINY")
        elif _distance(p, start, end) < tolerance and _distance(r, start, end) < tolerance:
            classes.append("SRC")
        elif abs(_distance(p, start, end) - depth) < tolerance and abs(_distance(r, start, end) - depth) < tolerance:
            classes.append("FRONT")
        elif any(_distance(p, s, e) < tolerance and _distance(r, s, e) < tolerance for s, e, _ in walls):
            classes.append("WALL")
        elif any(math.hypot(p[0] - v[0], p[1] - v[1]) < tolerance or math.hypot(r[0] - v[0], r[1] - v[1]) < tolerance for v in shared):
            classes.append("JOINT")
        elif any(_distance(p, a, b) < tolerance and _distance(r, a, b) < tolerance for a, b in perps):
            classes.append("PERP")
        else:
            classes.append("OTHER")
    return classes


def _solve(polygon: PolygonV1):
    skeleton = build_skeleton(polygon)
    if skeleton.outcome is not SkeletonOutcome.EXACT:
        return skeleton, None
    return skeleton, build_faces(polygon, skeleton)


def test_partial_source_corpus_closes_and_every_selected_end_is_flush():
    closed = 0
    for name, polygon in PARTIAL:
        creep = _crept(polygon)
        if creep is None:
            assert polygon.wall_edge_count == 0, name
            continue
        _, partition = _solve(creep.polygon)
        if partition is None or partition.outcome is not FaceOutcome.EXACT:
            continue
        closed += 1
        assert partition.area_reproduces_polygon, name

        class _Report:
            creeping_wall_count = creep.creeping_wall_count
            wall_spans = creep.wall_keys

        kept = drop_wall_faces(partition, _Report)
        for alpha in (Fraction(1), Fraction(3)):
            coverage = coverage_at(kept, alpha)
            for face in coverage.faces:
                if len(face.points) < 3:
                    continue
                classes = _classify(polygon, creep, face, alpha)
                assert "OTHER" not in classes, (name, alpha, face.owner, classes)
    assert closed == FIGURES_CLOSED_BY_CREEPING_WALLS


def test_wall_faces_are_dropped_and_only_sources_own_the_partition():
    for name, polygon in PARTIAL:
        creep = _crept(polygon)
        if creep is None:
            continue
        _, partition = _solve(creep.polygon)
        if partition is None or partition.outcome is not FaceOutcome.EXACT:
            continue

        class _Report:
            creeping_wall_count = creep.creeping_wall_count
            wall_spans = creep.wall_keys

        kept = drop_wall_faces(partition, _Report)
        walls = set(creep.wall_keys)
        sources = {(s[0], s[1], e[0], e[1]) for s, e, q in polygon.edges() if q}
        assert all(tuple(face.owner) not in walls for face in kept.faces), name
        assert {tuple(face.owner) for face in kept.faces} <= sources, name
        assert len(partition.faces) - len(kept.faces) == creep.creeping_wall_count, name
        dropped = partition.doubled_area - kept.doubled_area
        assert dropped.sign() > 0, name


@pytest.mark.parametrize("name,polygon", CORPUS, ids=[name for name, _ in CORPUS])
def test_all_seams_selected_is_unchanged(name, polygon):
    """Нет стен — нет ветки: мост с `creeping_walls=True` отвечает тем же отчётом побитово."""

    loops = _loops_of(polygon)
    laws = unit_speed_laws_of(polygon)
    before = bridge_arrival_laws(loops, laws)
    after = bridge_arrival_laws(loops, laws, creeping_walls=True)
    assert after == before
    assert after.creeping_wall_count == 0 and after.t_notch_count == 0
    assert creep_walls(polygon, loops, tuple(loop.points for loop in polygon.loops), laws) is None


def _t_rectangle(shear: Fraction):
    """Прямоугольник 4000 x 2000 под сдвигом `x' = x + shear*y`; низ разбит в x=1000, источник — левая часть."""

    points = [(0, 0), (1000, 0), (4000, 0), (4000 + int(shear * 2000), 2000), (int(shear * 2000), 2000)]
    loops = (tuple((Fraction(x), Fraction(y)) for x, y in points),)
    law = PlainArrivalLawV1(
        "source", Fraction(0), Fraction(1), Fraction(0), Fraction(1),
        span=((Fraction(0), Fraction(0)), (Fraction(1000), Fraction(0))),
        motion=(float(shear), 1.0),
    )
    return loops, law


@pytest.mark.parametrize("shear", (Fraction(0), Fraction(1, 2), Fraction(-3, 4)))
def test_t_junction_cut_is_perpendicular_in_the_metric_of_a_sheared_chart(shear):
    """Рез T-стыка идёт вдоль движения фронта (`G^-1 n`), а не вдоль евклидова перпендикуляра карты.

    Карта — сдвиг `x' = x + shear*y` прямоугольника; в исходной метрике рез обязан быть
    вертикалью `x = 1000`. Прежний (евклидов в карте) надрез давал на `shear = 1/2` наклон 27°.
    """

    loops, law = _t_rectangle(shear)
    report = bridge_arrival_laws(loops, (law,), weighted_fronts=True, creeping_walls=True)
    assert report.outcome is BridgeOutcome.EXACT
    assert report.t_notch_count == 1 and report.t_notch_refused_count == 0
    assert report.span_resolved_edge_count == 1
    _, partition = _solve(report.polygon)
    assert partition is not None and partition.outcome is FaceOutcome.EXACT
    coverage = coverage_at(drop_wall_faces(partition, report), Fraction(500))
    (face,) = [face for face in coverage.faces if len(face.points) >= 3]
    assert tuple(face.owner) == (0, 0, 1000, 0)
    original = [(_float(x) - float(shear) * _float(y), _float(y)) for x, y in face.points]
    xs = sorted(x for x, _ in original)
    # Левый торец — заподлицо со стеной x=0, правый — поперечный рез x=1000; глубина 500.
    tolerance = 500 * 1e-4
    assert abs(xs[0]) < tolerance and abs(xs[1]) < tolerance
    assert abs(xs[-1] - 1000) < tolerance and abs(xs[-2] - 1000) < tolerance
    assert max(y for _, y in original) == pytest.approx(500, abs=tolerance)


def test_t_junction_without_a_motion_direction_is_refused_by_name():
    loops, law = _t_rectangle(Fraction(0))
    report = bridge_arrival_laws(loops, (replace(law, motion=None),), weighted_fronts=True, creeping_walls=True)
    assert report.outcome is BridgeOutcome.EXACT
    assert report.t_notch_count == 0 and report.t_notch_refused_count == 1


def test_bridge_resolves_a_partially_covered_line_class_by_the_law_span():
    loops, law = _t_rectangle(Fraction(0))
    resolved = bridge_arrival_laws(loops, (law,), weighted_fronts=True)
    assert resolved.outcome is BridgeOutcome.EXACT
    assert resolved.span_resolved_edge_count == 1
    assert dict(resolved.owner_by_edge) == {(0, 0, 1000, 0): "source"}
    assert (1000, 0, 4000, 0) in resolved.wall_spans

    without = bridge_arrival_laws(loops, (replace(law, span=None),), weighted_fronts=True)
    assert without.outcome is BridgeOutcome.SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED

    stray = replace(law, span=((Fraction(0), Fraction(0)), (Fraction(999), Fraction(0))))
    unnamed = bridge_arrival_laws(loops, (stray,), weighted_fronts=True)
    assert unnamed.outcome is BridgeOutcome.SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED


def test_creeping_wall_speed_is_eps_of_the_slowest_source():
    polygon = PolygonV1(LoopV1(((0, 0), (12, 0), (12, 8), (0, 8)), (144, 0, 0, 0)))
    creep = _crept(polygon)
    speeds = creep.polygon.outer.edge_speeds_squared
    assert speeds[0] == 144
    for (start, end, _), speed in zip(creep.polygon.edges()[1:], speeds[1:]):
        length_squared = (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
        assert speed == WALL_CREEP_EPSILON ** 2 * length_squared
    assert creep.creeping_wall_count == 3


@pytest.mark.parametrize("base", [Fraction(1), Fraction(4, 9), Fraction(7, 3), Fraction(2**61 + 15, 2**59 - 1)])
def test_creeping_wall_speed_is_rational_one_per_domain_and_never_faster_than_the_law(base):
    """Почти плоская карта: `base` — длинная дробь. Нормальная скорость стены `sqrt(q / |d|^2)` рациональна (новых
    радикандов в скелете нет; иначе разложение Полларда исчерпывало бюджет канонизации), одна для стен любой длины
    (коллинеарные стены разной длины не расходятся в параллельные прямые) и не больше закона `eps * sqrt(base)`."""

    speeds = set()
    for length_squared in (1, 2, 5, 9, 2**40 + 3):
        normal = Fraction(creeping_speed(base, length_squared)) / length_squared
        root_n, root_d = math.isqrt(normal.numerator), math.isqrt(normal.denominator)
        assert root_n**2 == normal.numerator and root_d**2 == normal.denominator
        law = base * WALL_CREEP_EPSILON**2
        assert law * (1 - Fraction(1, 1 << 20)) < normal <= law
        speeds.add(normal)
    assert len(speeds) == 1
    if base in (1, Fraction(4, 9)):
        assert speeds == {base * WALL_CREEP_EPSILON**2}


def test_a_soft_bend_into_an_unselected_continuation_is_cut_across():
    """Выделена часть цепочки с изломом 14 градусов (тангенс 1/4): стык — T, торец режется поперёк."""

    polygon = PolygonV1(LoopV1(((0, 0), (8, 0), (16, 2), (16, 10), (0, 10)), (64, 0, 0, 0, 0)))
    assert _crept(polygon).t_notch_count == 1


def test_a_sharp_bend_into_an_unselected_neighbour_stays_flush():
    """Излом 63 градуса (тангенс 2): это угол, торец заподлицо с соседом, надреза нет."""

    polygon = PolygonV1(LoopV1(((0, 0), (8, 0), (16, 16), (0, 16)), (64, 0, 0, 0)))
    assert _crept(polygon).t_notch_count == 0
