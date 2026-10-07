"""Сгенерированные вызовы `build_skeleton` для веток, которых нет ни в полевом корпусе, ни в тестах ядра, и поиск зёрен, которые до них доходят.

Поле (163 вызова) и тесты ядра доводят скелет до `EXACT` и `WAVEFRONT_LEFT_UNRESOLVED`; а `SUPERLEVEL_COMPONENT_UNRESOLVABLE`, знаки сопряжением, исчерпание
границы уровней, плотная гидратация, вызов без бюджета, тёплая память, режим полного перебора, поколения замыкания (`generations > 0`, `normalize_junction_generation`)
без нарочно собранных входов не достигаются. Генератор детерминирован: семейство полигонов и ЗЕРНО (`random.Random(seed)`) полностью определяют полигон. Список достигающих
пар `(семейство, зерно)` — `CASES` — ЗАМОРОЖЕН в этом файле: он найден поиском (`hunt`: направляемый покрытием строк эталона, `native_skeleton_coverage.Tracer`) и повторяется
дёшево; поиск запускают, когда нужно найти новые. Исход (отказ, исчерпание, исключение) решает эталон, а не генератор: запись описывает ВЫЗОВ.

Семейства: `histogram` (ортогональный профиль), `polyomino` (контур случайной клеточной фигуры с дырами), `polyomino_weighted` (то же со стенами и весами: `q = 0`, `4|d|^2`, `9|d|^2`,
`|d|^2/4`), `polyomino_sources` (то же, источник — подмножество рёбер), `star` (звезда общего положения, дроби `q`, веера вогнутых вершин), `skew` (почти равные рёбра при координатах
2^33..2^40: знаки, которые фильтр оболочки не решает), `cross` (крест из полос, многосторонние слияния), `diagonal` (рёбра под 45 градусов), `collinear` (прямые вершины: скольжение),
`scaled` (малая фигура на решётке 2^k). Варианты вызова одного полигона: свежий бюджет, без бюджета, плотная гидратация, тёплая память, граница уровней ниже штатной, полный перебор.

    python tools/native_skeleton_generated.py generate [--corpus synthetic|<каталог>]       # дописать сгенерированные записи к синтетическому корпусу
    python tools/native_skeleton_generated.py hunt --family polyomino_weighted --seeds 0:5000 [--known FILE] [--out FILE]   # поиск зёрен, добавляющих строки
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from collections import Counter
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (str(ROOT / "tools"), str(ROOT / "kernel" / "src"), str(ROOT / "kernel" / "tests")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
import wavefront_cases as cases  # noqa: E402
from cftuv_envelope.wavefront.polygon import (  # noqa: E402
    FanSupportV1,
    PolygonRejected,
    PolygonV1,
    VertexFanV1,
    _edge_normal,
    unit_speed_squared,
    with_edge_speeds,
    with_source_spans,
    with_vertex_fans,
)
from cftuv_envelope.wavefront.skeleton import SplitSearch  # noqa: E402

INDEX_LABEL = "generated"


# --------------------------------------------------------------------------
# Семейства полигонов
# --------------------------------------------------------------------------


def _simplify(loop: list) -> list:
    """Без прямых вершин (угол ровно развёрнутый)."""

    size = len(loop)
    out = []
    for index in range(size):
        a, b, c = loop[index - 1], loop[index], loop[(index + 1) % size]
        if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) != 0:
            out.append(b)
    return out


def histogram(rng: random.Random) -> PolygonV1:
    """Ортогональный профиль: столбцы случайной ширины и высоты над общим основанием (много одновременных событий)."""

    columns = rng.randint(3, 8)
    scale = rng.choice((1, 1, 3, 7))
    widths = [rng.randint(1, 4) * scale for _ in range(columns)]
    heights = [rng.randint(1, 5) * scale for _ in range(columns)]
    edges = [0]
    for width in widths:
        edges.append(edges[-1] + width)
    points = [(0, 0), (edges[-1], 0)]
    for index in range(columns - 1, -1, -1):
        points.extend(((edges[index + 1], heights[index]), (edges[index], heights[index])))
    unique = [point for number, point in enumerate(points) if number == 0 or points[number - 1] != point]
    return PolygonV1.build(tuple(_simplify(unique)))


def _component(cells: set, start) -> set:
    seen, stack = {start}, [start]
    while stack:
        x, y = stack.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            near = (x + dx, y + dy)
            if near in cells and near not in seen:
                seen.add(near)
                stack.append(near)
    return seen


def _boundary_loops(cells: set):
    """Контуры клеточной фигуры (материал слева): внешний против часовой, дыры по часовой; `None` — есть точка касания углами."""

    edges: dict = {}
    for x, y in cells:
        for needed, start, end in (
            ((x, y - 1), (x, y), (x + 1, y)), ((x + 1, y), (x + 1, y), (x + 1, y + 1)),
            ((x, y + 1), (x + 1, y + 1), (x, y + 1)), ((x - 1, y), (x, y + 1), (x, y)),
        ):
            if needed not in cells:
                edges.setdefault(start, []).append(end)
    if any(len(value) > 1 for value in edges.values()):
        return None
    following = {start: ends[0] for start, ends in edges.items()}
    loops, seen = [], set()
    for start in list(following):
        if start in seen:
            continue
        loop, current = [], start
        while current not in seen:
            seen.add(current)
            loop.append(current)
            current = following[current]
        loops.append(loop)
    return loops


def _double_area(loop: list) -> int:
    return sum(loop[i][0] * loop[(i + 1) % len(loop)][1] - loop[(i + 1) % len(loop)][0] * loop[i][1] for i in range(len(loop)))


def polyomino(rng: random.Random) -> PolygonV1 | None:
    """Контур случайной клеточной фигуры (до 7x7) с дырами; единичные скорости."""

    width, height = rng.randint(2, 7), rng.randint(2, 7)
    fill = rng.choice((0.5, 0.65, 0.8))
    cells = {(x, y) for x in range(width) for y in range(height) if rng.random() < fill}
    if not cells:
        return None
    best = max((_component(cells, cell) for cell in list(cells)[:6]), key=len)
    loops = _boundary_loops(best)
    if loops is None:
        return None
    loops = sorted((_simplify(loop) for loop in loops), key=lambda loop: -abs(_double_area(loop)) if len(loop) >= 3 else 0)
    loops = [loop for loop in loops if len(loop) >= 3]
    return PolygonV1.build(tuple(loops[0]), tuple(tuple(loop) for loop in loops[1:]))


def _weights(rng: random.Random, polygon: PolygonV1) -> PolygonV1:
    row = []
    for start, end, _speed in polygon.edges():
        pick = rng.random()
        unit = unit_speed_squared(start, end)
        value = 0 if pick < 0.35 else unit if pick < 0.7 else 4 * unit if pick < 0.8 else 9 * unit if pick < 0.9 else Fraction(unit, 4)
        row.append((start, end, value))
    if all(value == 0 for _s, _e, value in row):
        row[0] = (row[0][0], row[0][1], unit_speed_squared(row[0][0], row[0][1]))
    return with_edge_speeds(polygon, tuple(row))


def polyomino_weighted(rng: random.Random) -> PolygonV1 | None:
    polygon = polyomino(rng)
    return None if polygon is None else _weights(rng, polygon)


def polyomino_sources(rng: random.Random) -> PolygonV1 | None:
    return _sources(rng, polyomino(rng))


def _fans(rng: random.Random, polygon: PolygonV1) -> PolygonV1:
    """Веера вогнутых вершин: одна или две скрытые опоры из сумм нормалей соседних рёбер (строго внутри вогнутого сектора)."""

    fans = []
    for loop in polygon.loops:
        points, flags = loop.points, loop.reflex_flags()
        for index, point in enumerate(points):
            if not flags[index] or rng.random() < 0.4:
                continue
            before = _edge_normal(points[index - 1], point)
            after = _edge_normal(point, points[(index + 1) % len(points)])
            combos = ((1, 1),) if rng.random() < 0.5 else ((2, 1), (1, 2))
            supports = []
            for left, right in combos:
                normal = (left * before[0] + right * after[0], left * before[1] + right * after[1])
                unit = normal[0] * normal[0] + normal[1] * normal[1]
                supports.append(FanSupportV1(normal[0], normal[1], rng.choice((unit, unit, 4 * unit, Fraction(unit, 2)))))
            fans.append(VertexFanV1(point, tuple(supports)))
    return with_vertex_fans(polygon, tuple(fans)) if fans else polygon


def star(rng: random.Random) -> PolygonV1 | None:
    """Звезда общего положения (радикалы общего вида), с весами и веерами в половине случаев."""

    polygon = cases.star(rng.choice((5, 6, 7, 9, 11)), rng.randint(0, 10**6), radius=rng.choice((1 << 12, 1 << 16, 1 << 20)))
    if polygon is None:
        return None
    if rng.random() < 0.5:
        polygon = _weights(rng, polygon)
    return _fans(rng, polygon) if rng.random() < 0.5 else polygon


def skew(rng: random.Random) -> PolygonV1:
    """Почти равные рёбра при координатах 2^33..2^40: разности корней с относительной малостью меньше 2^-64 (знак решает сопряжение)."""

    bits = rng.choice((33, 36, 40))
    side = (1 << bits) + rng.randint(0, 3)
    corners = [(0, 0), (side, rng.randint(0, 2)), (side + rng.randint(0, 2), side + rng.randint(0, 3)), (rng.randint(-2, 2), side)]
    if rng.random() < 0.4:
        corners.insert(2, (side + rng.randint(1, 5), side // 2 + rng.randint(0, 3)))
    return PolygonV1.build(tuple(corners))


def cross(rng: random.Random) -> PolygonV1:
    """Крест из двух полос с произвольными плечами: у вогнутых углов центрального блока события сходятся одновременно."""

    wide, tall = rng.randint(1, 6), rng.randint(1, 6)
    right, top = wide + rng.randint(1, 8), tall + rng.randint(1, 8)
    left, bottom = rng.randint(1, 8), rng.randint(1, 8)
    points = [(0, -bottom), (wide, -bottom), (wide, 0), (right, 0), (right, tall), (wide, tall), (wide, top), (0, top), (0, tall), (-left, tall), (-left, 0), (0, 0)]
    return PolygonV1.build(tuple(points))


def diagonal(rng: random.Random) -> PolygonV1:
    """Рёбра под 45 градусов: ромбы, зубчатые ромбы, ступени по диагонали (один радикал, много совпадений)."""

    radius = rng.randint(2, 12)
    teeth = rng.randint(0, 3)
    points = [(0, -radius), (radius, 0), (0, radius), (-radius, 0)]
    for _ in range(teeth):
        index = rng.randrange(len(points))
        a, b = points[index], points[(index + 1) % len(points)]
        mid = ((a[0] + b[0]) // 2 + rng.choice((-1, 1)), (a[1] + b[1]) // 2 + rng.choice((-1, 1)))
        if mid not in points:
            points.insert(index + 1, mid)
    return PolygonV1.build(tuple(points))


def collinear(rng: random.Random) -> PolygonV1 | None:
    """Фигура с прямыми вершинами (середины рёбер): угол развёрнутый, вершина скользит вдоль прямой."""

    base = rng.choice((polyomino, histogram, cross))(rng)
    if base is None:
        return None
    loops = []
    for loop in (base.outer, *base.holes):
        points = []
        for index, point in enumerate(loop.points):
            points.append(point)
            nxt = loop.points[(index + 1) % len(loop.points)]
            if (point[0] + nxt[0]) % 2 == 0 and (point[1] + nxt[1]) % 2 == 0 and rng.random() < 0.5:
                points.append(((point[0] + nxt[0]) // 2, (point[1] + nxt[1]) // 2))
        loops.append(tuple(points))
    scaled = [tuple((2 * x, 2 * y) for x, y in loop) for loop in loops]
    return PolygonV1.build(scaled[0], tuple(scaled[1:]))


def scaled(rng: random.Random) -> PolygonV1 | None:
    """Малая фигура на решётке 2^k (до 2^20): те же события, большие числа."""

    base = rng.choice((polyomino, histogram, cross, diagonal))(rng)
    if base is None:
        return None
    factor = 1 << rng.randint(3, 20)
    loops = [tuple((x * factor, y * factor) for x, y in loop.points) for loop in (base.outer, *base.holes)]
    return PolygonV1.build(loops[0], tuple(loops[1:]))


def _sources(rng: random.Random, polygon: PolygonV1 | None) -> PolygonV1 | None:
    """Источник — часть рёбер (одно, пара соседних, половина), прочие — стены: стены неподвижны, и вогнутые вершины встречают их углы."""

    if polygon is None:
        return None
    spans = tuple((start, end) for start, end, _ in polygon.edges())
    mode = rng.choice(("one", "two", "half"))
    if mode == "one":
        chosen = (rng.choice(spans),)
    elif mode == "two":
        index = rng.randrange(len(spans))
        chosen = (spans[index], spans[(index + 1) % len(spans)])
    else:
        chosen = tuple(rng.sample(spans, max(1, len(spans) // 2)))
    return with_source_spans(polygon, chosen)


def histogram_sources(rng: random.Random) -> PolygonV1:
    return _sources(rng, histogram(rng))


def histogram_weighted(rng: random.Random) -> PolygonV1:
    return _weights(rng, histogram(rng))


def cross_sources(rng: random.Random) -> PolygonV1:
    return _sources(rng, cross(rng))


def cross_weighted(rng: random.Random) -> PolygonV1:
    return _weights(rng, cross(rng))


def diagonal_sources(rng: random.Random) -> PolygonV1:
    return _sources(rng, diagonal(rng))


def classic(rng: random.Random) -> PolygonV1:
    """Именованные фигуры корпуса ядра (ступени, Г-образная, П-образная, двойная выемка) со случайными источниками и весами."""

    shape = rng.choice((cases.staircase, cases.u_shape, cases.double_notch, lambda: cases.ell(rng.choice((8, 12, 16)))))()
    shape = _sources(rng, shape) if rng.random() < 0.6 else shape
    return _weights(rng, shape) if rng.random() < 0.5 else shape


FAMILIES = {
    "histogram": histogram, "polyomino": polyomino, "polyomino_weighted": polyomino_weighted, "polyomino_sources": polyomino_sources, "star": star,
    "skew": skew, "cross": cross, "diagonal": diagonal, "collinear": collinear, "scaled": scaled, "histogram_sources": histogram_sources,
    "histogram_weighted": histogram_weighted, "cross_sources": cross_sources, "cross_weighted": cross_weighted, "diagonal_sources": diagonal_sources,
    "classic": classic,
}


def make(family: str, seed: int) -> PolygonV1 | None:
    """Полигон семейства по зерну; `None` — зерно не дало допустимого полигона (отказ входа считается пропуском, а не записью)."""

    try:
        return FAMILIES[family](random.Random(seed))
    except PolygonRejected:
        return None
    except (ValueError, ZeroDivisionError):
        return None


#: Достигающие пары `(семейство, зерно)`: заморожены, найдены `hunt` (покрытие строк эталона, исходы, сопряжение). Пополняется поиском, повторяется дёшево.
CASES: tuple = (
    # histogram: 24
    ("histogram", 0), ("histogram", 1), ("histogram", 2), ("histogram", 3), ("histogram", 4), ("histogram", 5),
    ("histogram", 6), ("histogram", 7), ("histogram", 8), ("histogram", 9), ("histogram", 10), ("histogram", 11),
    ("histogram", 12), ("histogram", 13), ("histogram", 14), ("histogram", 15), ("histogram", 16), ("histogram", 17),
    ("histogram", 18), ("histogram", 19), ("histogram", 38), ("histogram", 45), ("histogram", 53), ("histogram", 59),
    # polyomino: 22
    ("polyomino", 0), ("polyomino", 1), ("polyomino", 2), ("polyomino", 3), ("polyomino", 4), ("polyomino", 5),
    ("polyomino", 6), ("polyomino", 7), ("polyomino", 8), ("polyomino", 9), ("polyomino", 10), ("polyomino", 11),
    ("polyomino", 14), ("polyomino", 29), ("polyomino", 37), ("polyomino", 38), ("polyomino", 45), ("polyomino", 46),
    ("polyomino", 1046), ("polyomino", 4000), ("polyomino", 4001), ("polyomino", 4230),
    # polyomino_weighted: 63
    ("polyomino_weighted", 0), ("polyomino_weighted", 1), ("polyomino_weighted", 2), ("polyomino_weighted", 3), ("polyomino_weighted", 4), ("polyomino_weighted", 5),
    ("polyomino_weighted", 6), ("polyomino_weighted", 7), ("polyomino_weighted", 8), ("polyomino_weighted", 9), ("polyomino_weighted", 10), ("polyomino_weighted", 11),
    ("polyomino_weighted", 12), ("polyomino_weighted", 13), ("polyomino_weighted", 14), ("polyomino_weighted", 15), ("polyomino_weighted", 16), ("polyomino_weighted", 17),
    ("polyomino_weighted", 18), ("polyomino_weighted", 20), ("polyomino_weighted", 23), ("polyomino_weighted", 37), ("polyomino_weighted", 45), ("polyomino_weighted", 46),
    ("polyomino_weighted", 51), ("polyomino_weighted", 62), ("polyomino_weighted", 106), ("polyomino_weighted", 110), ("polyomino_weighted", 119), ("polyomino_weighted", 131),
    ("polyomino_weighted", 312), ("polyomino_weighted", 376), ("polyomino_weighted", 413), ("polyomino_weighted", 490), ("polyomino_weighted", 587), ("polyomino_weighted", 743),
    ("polyomino_weighted", 746), ("polyomino_weighted", 887), ("polyomino_weighted", 1018), ("polyomino_weighted", 1071), ("polyomino_weighted", 1323), ("polyomino_weighted", 1604),
    ("polyomino_weighted", 1983), ("polyomino_weighted", 2389), ("polyomino_weighted", 4000), ("polyomino_weighted", 4001), ("polyomino_weighted", 4004), ("polyomino_weighted", 4005),
    ("polyomino_weighted", 4010), ("polyomino_weighted", 4031), ("polyomino_weighted", 4065), ("polyomino_weighted", 4373), ("polyomino_weighted", 4403), ("polyomino_weighted", 4518),
    ("polyomino_weighted", 4531), ("polyomino_weighted", 4571), ("polyomino_weighted", 4721), ("polyomino_weighted", 4762), ("polyomino_weighted", 5057), ("polyomino_weighted", 6021),
    ("polyomino_weighted", 6157), ("polyomino_weighted", 7753), ("polyomino_weighted", 9212),
    # polyomino_sources: 61
    ("polyomino_sources", 0), ("polyomino_sources", 1), ("polyomino_sources", 2), ("polyomino_sources", 3), ("polyomino_sources", 4), ("polyomino_sources", 5),
    ("polyomino_sources", 6), ("polyomino_sources", 7), ("polyomino_sources", 8), ("polyomino_sources", 9), ("polyomino_sources", 10), ("polyomino_sources", 11),
    ("polyomino_sources", 12), ("polyomino_sources", 13), ("polyomino_sources", 14), ("polyomino_sources", 16), ("polyomino_sources", 17), ("polyomino_sources", 18),
    ("polyomino_sources", 20), ("polyomino_sources", 21), ("polyomino_sources", 23), ("polyomino_sources", 29), ("polyomino_sources", 38), ("polyomino_sources", 50),
    ("polyomino_sources", 75), ("polyomino_sources", 95), ("polyomino_sources", 105), ("polyomino_sources", 129), ("polyomino_sources", 145), ("polyomino_sources", 152),
    ("polyomino_sources", 209), ("polyomino_sources", 328), ("polyomino_sources", 414), ("polyomino_sources", 446), ("polyomino_sources", 573), ("polyomino_sources", 747),
    ("polyomino_sources", 1306), ("polyomino_sources", 1400), ("polyomino_sources", 1525), ("polyomino_sources", 1860), ("polyomino_sources", 2014), ("polyomino_sources", 4000),
    ("polyomino_sources", 4001), ("polyomino_sources", 4003), ("polyomino_sources", 4016), ("polyomino_sources", 4029), ("polyomino_sources", 4031), ("polyomino_sources", 4044),
    ("polyomino_sources", 4050), ("polyomino_sources", 4106), ("polyomino_sources", 4170), ("polyomino_sources", 4259), ("polyomino_sources", 4401), ("polyomino_sources", 4639),
    ("polyomino_sources", 4735), ("polyomino_sources", 4913), ("polyomino_sources", 4996), ("polyomino_sources", 5163), ("polyomino_sources", 5305), ("polyomino_sources", 7207),
    ("polyomino_sources", 7210),
    # star: 36
    ("star", 0), ("star", 1), ("star", 2), ("star", 3), ("star", 4), ("star", 5),
    ("star", 6), ("star", 7), ("star", 8), ("star", 9), ("star", 10), ("star", 11),
    ("star", 12), ("star", 13), ("star", 14), ("star", 15), ("star", 16), ("star", 17),
    ("star", 61), ("star", 62), ("star", 94), ("star", 110), ("star", 132), ("star", 136),
    ("star", 155), ("star", 182), ("star", 189), ("star", 209), ("star", 268), ("star", 279),
    ("star", 315), ("star", 328), ("star", 437), ("star", 491), ("star", 509), ("star", 510),
    # skew: 19
    ("skew", 0), ("skew", 1), ("skew", 2), ("skew", 3), ("skew", 4), ("skew", 5),
    ("skew", 6), ("skew", 7), ("skew", 8), ("skew", 9), ("skew", 10), ("skew", 11),
    ("skew", 12), ("skew", 13), ("skew", 14), ("skew", 15), ("skew", 16), ("skew", 17),
    ("skew", 99),
    # cross: 12
    ("cross", 0), ("cross", 1), ("cross", 2), ("cross", 3), ("cross", 4), ("cross", 5),
    ("cross", 6), ("cross", 7), ("cross", 8), ("cross", 9), ("cross", 10), ("cross", 11),
    # diagonal: 19
    ("diagonal", 0), ("diagonal", 1), ("diagonal", 2), ("diagonal", 3), ("diagonal", 4), ("diagonal", 5),
    ("diagonal", 6), ("diagonal", 7), ("diagonal", 8), ("diagonal", 9), ("diagonal", 10), ("diagonal", 11),
    ("diagonal", 82), ("diagonal", 367), ("diagonal", 442), ("diagonal", 455), ("diagonal", 512), ("diagonal", 592),
    ("diagonal", 615),
    # collinear: 23
    ("collinear", 0), ("collinear", 1), ("collinear", 2), ("collinear", 3), ("collinear", 4), ("collinear", 5),
    ("collinear", 6), ("collinear", 7), ("collinear", 8), ("collinear", 9), ("collinear", 10), ("collinear", 11),
    ("collinear", 14), ("collinear", 18), ("collinear", 28), ("collinear", 39), ("collinear", 46), ("collinear", 54),
    ("collinear", 4000), ("collinear", 4002), ("collinear", 4011), ("collinear", 4040), ("collinear", 4158),
    # scaled: 23
    ("scaled", 0), ("scaled", 1), ("scaled", 2), ("scaled", 3), ("scaled", 4), ("scaled", 5),
    ("scaled", 6), ("scaled", 7), ("scaled", 8), ("scaled", 9), ("scaled", 10), ("scaled", 11),
    ("scaled", 14), ("scaled", 28), ("scaled", 46), ("scaled", 53), ("scaled", 64), ("scaled", 66),
    ("scaled", 86), ("scaled", 142), ("scaled", 144), ("scaled", 432), ("scaled", 556),
    # histogram_sources: 9
    ("histogram_sources", 0), ("histogram_sources", 1), ("histogram_sources", 7), ("histogram_sources", 10), ("histogram_sources", 11), ("histogram_sources", 12),
    ("histogram_sources", 23), ("histogram_sources", 654), ("histogram_sources", 1943),
    # histogram_weighted: 10
    ("histogram_weighted", 0), ("histogram_weighted", 1), ("histogram_weighted", 3), ("histogram_weighted", 6), ("histogram_weighted", 11), ("histogram_weighted", 12),
    ("histogram_weighted", 14), ("histogram_weighted", 392), ("histogram_weighted", 652), ("histogram_weighted", 2939),
    # cross_sources: 9
    ("cross_sources", 0), ("cross_sources", 2), ("cross_sources", 5), ("cross_sources", 9), ("cross_sources", 20), ("cross_sources", 25),
    ("cross_sources", 2360), ("cross_sources", 3015), ("cross_sources", 3401),
    # cross_weighted: 10
    ("cross_weighted", 0), ("cross_weighted", 1), ("cross_weighted", 4), ("cross_weighted", 9), ("cross_weighted", 37), ("cross_weighted", 292),
    ("cross_weighted", 390), ("cross_weighted", 1079), ("cross_weighted", 2214), ("cross_weighted", 4558),
    # diagonal_sources: 10
    ("diagonal_sources", 0), ("diagonal_sources", 1), ("diagonal_sources", 9), ("diagonal_sources", 14), ("diagonal_sources", 82), ("diagonal_sources", 297),
    ("diagonal_sources", 1237), ("diagonal_sources", 3010), ("diagonal_sources", 4104), ("diagonal_sources", 4505),
    # classic: 6
    ("classic", 0), ("classic", 2), ("classic", 4), ("classic", 13), ("classic", 52), ("classic", 1640),
)


# --------------------------------------------------------------------------
# Варианты вызова и запись
# --------------------------------------------------------------------------


def variants_for(seed: int) -> tuple:
    """Какие варианты вызова писать для зерна (детерминированно по зерну): свежий бюджет — всегда."""

    found = ["fresh"]
    for name, period in (("none", 3), ("dense", 4), ("warm", 5), ("level", 7), ("exhaustive", 11), ("saturated", 97)):
        if seed % period == 0:
            found.append(name)
    return tuple(found)


def _segment_primes(low: int, count: int) -> list:
    """Первые `count` простых не меньше `low` (решето отрезка; `low` до 2^40)."""

    width = max(1 << 16, count * 40)
    small = [n for n in range(2, math.isqrt(low + width) + 1) if all(n % d for d in range(2, math.isqrt(n) + 1))]
    flags = [True] * width
    for prime in small:
        for multiple in range(max(prime * prime, -(-low // prime) * prime), low + width, prime):
            flags[multiple - low] = False
    found = [low + offset for offset, flag in enumerate(flags) if flag]
    return found[:count]


def _saturate() -> None:
    """Память у границы вытеснения: реестр простых и словарь разложений на одну запись короче ёмкости (8192), так что ближайшие промахи вытесняют и стирают.

    Записи — настоящие факты (простые и их произведения), поэтому состояние допустимо: ответ от него не меняется, цена и порядок таблиц — меняются."""

    registry = exact._KNOWN_PRIME_REGISTRY_ENTRIES - 1
    primes = _segment_primes(2, registry)
    exact._KNOWN_PRIMES.extend(primes)
    exact._KNOWN_PRIME_SET.update(primes)
    big = _segment_primes(1 << 24, 2 * (exact._FACTORIZATION_MEMO_ENTRIES - 1))
    for index in range(exact._FACTORIZATION_MEMO_ENTRIES - 1):
        left, right = big[2 * index], big[2 * index + 1]
        exact._FACTORIZATION_MEMO[left * right] = ((left, 1), (right, 1))


def _cold() -> None:
    exact.reset_factorization_memory()
    exact.reset_sign_counts()
    exact.reset_unbudgeted_work()
    exact.set_canonical_audit(False)


def _budget(label: str):
    return exact.exact_work_budget(stage="PREPARE", domain_id=label)


def _call(wrapped, polygon, variant: str, label: str, levels: int):
    """Один вызов варианта через записывающую обёртку; исход (результат либо исключение) возвращается."""

    options: dict = {"work_budget": _budget(label)}
    if variant == "none":
        options["work_budget"] = None
    elif variant == "dense":
        options["dense_hydration"] = True
    elif variant == "exhaustive":
        options["split_search"] = SplitSearch.EXHAUSTIVE
    pin = max(1, levels // 2) if variant == "level" else None
    with nc.pinned_level_budget(pin):
        try:
            return wrapped(polygon, **options), None
        except Exception as exc:  # noqa: BLE001 - исключение эталона — исход записи
            return None, exc


def _levels_of(polygon) -> int:
    """Число уровней свежего прогона (для границы ниже штатной); прогон не пишется."""

    _cold()
    try:
        return nc.ORACLE[nc.OP_SKELETON](polygon, work_budget=_budget("levels")).levels
    except Exception:  # noqa: BLE001
        return 2


def _record_case(recorder, wrapped, family: str, seed: int, outcomes: Counter) -> None:
    polygon = make(family, seed)
    if polygon is None:
        outcomes["skipped"] += 1
        return
    levels = _levels_of(polygon)
    for variant in variants_for(seed):
        if variant == "exhaustive" and polygon.vertex_count + polygon.fan_edge_count > 24:
            continue
        _cold()
        if variant == "warm":
            sibling = make(family, seed + 1)
            if sibling is not None:
                try:
                    nc.ORACLE[nc.OP_SKELETON](sibling, work_budget=_budget("warm-up"))
                except Exception:  # noqa: BLE001 - тёплая память — то, что осталось после любого прогона
                    pass
        if variant == "saturated":
            _saturate()
        recorder.context.update(mesh=f"generated_{family}", mesh_digest=f"seed={seed}", alpha=None, patch_id=None, domain_id=None)
        result, error = _call(wrapped, polygon, variant, f"{family}:{seed}", levels)
        row = recorder.rows[-1]
        row.update(label=INDEX_LABEL, family=family, seed=seed, variant=variant)
        outcomes[row["outcome"]] += 1


def generate(recorder, cases_list: tuple | None = None) -> Counter:
    """Записывает сгенерированные вызовы `CASES` (либо `cases_list`) рекордером; возвращает число записей по исходам."""

    wrapped = recorder.wrap(nc.OP_SKELETON, nc.ORACLE[nc.OP_SKELETON])
    outcomes: Counter = Counter()
    for family, seed in CASES if cases_list is None else cases_list:
        _record_case(recorder, wrapped, family, seed, outcomes)
    return outcomes


def generate_corpus(root: Path, cases_list: tuple | None = None) -> dict:
    """Дописывает сгенерированные записи к корпусу `root` (индекс переписывается; прежние сгенерированные записи заменяются)."""

    index = sc.load_index(root)
    kept = [row for row in index["records"] if row.get("label") != INDEX_LABEL and row.get("derived") is None]
    for row in index["records"]:
        if row.get("label") == INDEX_LABEL and row.get("derived") is None:
            (root / row["path"]).unlink(missing_ok=True)
    index["records"] = kept
    (root / "index.json").write_text(json.dumps(index, ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
    recorder = nc.Recorder(root, nc.run_description({"corpus": "synthetic_skeleton"}), preset=3, max_bytes=nc.DEFAULT_MAX_BYTES, operations=nc.SKELETON_OPERATIONS)
    recorder.resume()
    outcomes = generate(recorder, cases_list)
    index.update(
        records=recorder.rows, records_count=len(recorder.rows), total_bytes=recorder.bytes, generated_outcomes=dict(outcomes),
        outcomes=dict(Counter(row["outcome"] for row in recorder.rows)),
    )
    index["labels"] = {**index.get("labels", {}), "generated": sum(outcomes.values())}
    (root / "index.json").write_text(json.dumps(index, ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
    return dict(outcomes)


# --------------------------------------------------------------------------
# Поиск зёрен, направляемый покрытием
# --------------------------------------------------------------------------


def _novelty(tracer, before: int, result, error, delta: dict, seen: set) -> list:
    """Что нового дала запись: строки эталона, исход, причина отказа, знак сопряжением, тип исключения."""

    found = []
    now = sum(len(lines) for lines in tracer.hits.values())
    if now > before:
        found.append(f"lines+{now - before}")
    keys = []
    if result is not None:
        keys.append(("outcome", result.outcome.value))
        keys.extend(("reason", name) for name, value in result.counters if name.startswith("superlevel_unresolvable_reason::") and value)
        keys.extend(("counter", name) for name, value in result.counters if value and name.startswith(("refused_no_rule", "vertex_meeting", "unsupported")))
    if error is not None:
        keys.append(("exception", type(error).__qualname__))
    if delta.get("closed_by_conjugation"):
        keys.append(("conjugation", True))
    for key in keys:
        if key not in seen:
            seen.add(key)
            found.append(f"{key[0]}:{key[1]}")
    return found


def hunt(family: str, seeds: range, *, tracer=None, seen: set | None = None, limit_seconds: float = 0.0) -> list:
    """`[(семейство, зерно, что_нового)]`: зёрна, чей прогон добавил строку эталона либо новый исход, причину, класс знака, исключение."""

    import native_skeleton_coverage as coverage

    tracer = tracer or coverage.Tracer()
    seen = set() if seen is None else seen
    started, kept = time.perf_counter(), []
    tracer.start()
    try:
        for seed in seeds:
            polygon = make(family, seed)
            if polygon is None:
                continue
            _cold()
            before = sum(len(lines) for lines in tracer.hits.values())
            signs = dict(exact.SIGN_COUNTS)
            try:
                result, error = nc.ORACLE[nc.OP_SKELETON](polygon, work_budget=_budget("hunt")), None
            except Exception as exc:  # noqa: BLE001
                result, error = None, exc
            news = _novelty(tracer, before, result, error, {key: exact.SIGN_COUNTS[key] - signs[key] for key in signs}, seen)
            if news:
                kept.append((family, seed, news))
            if limit_seconds and time.perf_counter() - started > limit_seconds:
                break
    finally:
        tracer.stop()
    return kept


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    gen = sub.add_parser("generate")
    gen.add_argument("--corpus", default="synthetic")
    find = sub.add_parser("hunt")
    find.add_argument("--family", default="")
    find.add_argument("--seeds", default="0:2000")
    find.add_argument("--seconds", type=float, default=0.0)
    find.add_argument("--known", default="", help="файлы покрытия (`native_skeleton_coverage corpus --dump`): уже достигнутые строки не считаются новыми")
    find.add_argument("--out", type=Path, default=None)
    arguments = parser.parse_args(argv)
    if arguments.command == "generate":
        root = sc.matching(arguments.corpus) if arguments.corpus in sc.KINDS else Path(arguments.corpus)
        if root is None:
            raise SystemExit(f"NATIVE_SKELETON_GENERATED_FAILED {sc.describe_missing(arguments.corpus)}")
        outcomes = generate_corpus(root)
        print(json.dumps(outcomes))
        print("NATIVE_SKELETON_GENERATED_OK", sum(outcomes.values()))
        return 0
    first, _, last = arguments.seeds.partition(":")
    kept: list = []
    seen: set = set()
    import native_skeleton_coverage as coverage

    tracer = coverage.Tracer()
    for name, lines in coverage.merge([item for item in arguments.known.split(",") if item]).items():
        tracer.hits[name].update(lines)
    for family in [item for item in arguments.family.split(",") if item] or list(FAMILIES):
        found = hunt(family, range(int(first), int(last)), tracer=tracer, seen=seen, limit_seconds=arguments.seconds)
        kept.extend(found)
        print(f"{family}: {len(found)} seeds kept", flush=True)
    text = json.dumps([[family, seed, news] for family, seed, news in kept])
    if arguments.out:
        arguments.out.write_text(text, encoding="utf-8")
    print(text)
    print(f"covered statements: {sum(len(lines) for lines in tracer.hits.values())}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
