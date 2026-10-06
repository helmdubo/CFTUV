"""Сгенерированные вызовы `clip_geometry` для веток, которых нет ни в полевом корпусе, ни в тестах ядра.

Тесты ядра строят аккуратные планы и многоугольники; ветки отказов и шума (нецелая карта, коэффициенты `int`, переполнение окна,
исчерпание бюджета на сопряжении, самопересекающийся многоугольник, вершина вне карты, свес, шов, веера, все законы топологии) без
нарочно собранных входов не достигаются. Генератор детерминирован (зерно), строит планы подъёма разного вида и многоугольники разного
рода и отдаёт каждый вызов эталону через записывающую обёртку `native_corpus.Recorder` — запись той же формы, что в полевом корпусе.
Исход (резка, названный отказ, исчерпание, `OverflowError`) решает эталон, а не генератор: запись описывает ВЫЗОВ.

    from native_clip_generated import generate
    generate(recorder, count=220, seed=20261006)
"""

from __future__ import annotations

import contextlib
import math
import random
from collections import Counter
from fractions import Fraction

import native_corpus as nc

import cftuv_envelope.exact_sqrt_sum as exact
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1

LAWS = (
    DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    DecalTopologyLawV1.TRIANGLES_V1,
    DecalTopologyLawV1.QUAD_STRIPS_V1,
    DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1,
)


def _corner(x, y, z, scale: int) -> tuple:
    return (Fraction(x, scale), Fraction(y, scale), Fraction(z))


def _triangle(name, chart, scale, height, face="", normals=()):
    corners = tuple(_corner(x, y, height(x, y), scale) for x, y in chart)
    return (name, tuple(chart), corners, normals, face)


def lift_square(side: int = 8, height=lambda x, y: 0, normals: bool = False) -> SurfaceLiftV1:
    """Квадрат `side` x `side`, диагональ `(0,0)-(side,side)`: два треугольника без граней (закон по треугольникам)."""

    unit = (0.0, 0.0, 1.0)
    extra = (unit, unit, unit) if normals else ()
    return SurfaceLiftV1.from_triangles(
        [
            _triangle("t0", ((0, 0), (side, 0), (side, side)), 4, height, normals=extra),
            _triangle("t1", ((0, 0), (side, side), (0, side)), 4, height, normals=extra),
        ],
        scale=4,
    )


def lift_grid(count: int = 3, cell: int = 6, height=lambda x, y: 0, merged: bool = True) -> SurfaceLiftV1:
    """Решётка `count x count` четырёхгранников (каждый — два треугольника одной грани): ячейки закона по граням."""

    items = []
    for i in range(count):
        for j in range(count):
            x0, y0 = i * cell, j * cell
            face = f"face{i}.{j}" if merged else ""
            items.append(_triangle(f"g{i}{j}a", ((x0, y0), (x0 + cell, y0), (x0 + cell, y0 + cell)), 4, height, face))
            items.append(_triangle(f"g{i}{j}b", ((x0, y0), (x0 + cell, y0 + cell), (x0, y0 + cell)), 4, height, face))
    return SurfaceLiftV1.from_triangles(items, scale=4)


def lift_ell() -> SurfaceLiftV1:
    """Одна Г-образная грань (невыпуклая: группа) рядом с квадратом (ячейка) и прямоугольником с T-вершиной (три треугольника)."""

    fan = [((0, 0), (8, 0), (8, 4)), ((0, 0), (8, 4), (4, 4)), ((0, 0), (4, 4), (4, 8)), ((0, 0), (4, 8), (0, 8))]
    items = [_triangle(f"e{k}", [(x, y) for x, y in chart], 4, lambda x, y: Fraction(x * y, 900), "ell") for k, chart in enumerate(fan)]
    for k, chart in enumerate([((12, 0), (16, 0), (16, 4)), ((12, 0), (16, 4), (12, 4))]):
        items.append(_triangle(f"q{k}", chart, 4, lambda x, y: Fraction(x, 700), "quad"))
    straight = [((20, 0), (24, 0), (20, 6)), ((24, 0), (28, 0), (28, 6)), ((24, 0), (28, 6), (20, 6))]
    for k, chart in enumerate(straight):
        items.append(_triangle(f"s{k}", chart, 4, lambda x, y: Fraction(x + y, 1100), "straight"))
    return SurfaceLiftV1.from_triangles(items, scale=4)


def lift_fraction_chart() -> SurfaceLiftV1:
    """Карта с дробными, отрицательными и крупными координатами: вторая ветка `_edge_value` и отказ `_edge_constants`."""

    charts = [
        ((Fraction(1, 2), Fraction(0)), (Fraction(15, 2), Fraction(1, 3)), (Fraction(0), Fraction(7, 2))),
        ((Fraction(15, 2), Fraction(1, 3)), (Fraction(8), Fraction(8)), (Fraction(0), Fraction(7, 2))),
        ((-4, -4), (0, -4), (0, 0)),
    ]
    items = []
    for number, chart in enumerate(charts):
        corners = tuple((Fraction(x) / 4, Fraction(y) / 4, Fraction(number, 50)) for x, y in chart)
        items.append((f"f{number}", chart, corners, (), ""))
    return SurfaceLiftV1.from_triangles(items, scale=4)


PLANES = {
    "square": lambda: lift_square(),
    "square-folded": lambda: lift_square(height=lambda x, y: Fraction(x * y, 40)),
    "square-normals": lambda: lift_square(height=lambda x, y: Fraction(x + y, 80), normals=True),
    "grid-flat": lambda: lift_grid(3),
    "grid-curved": lambda: lift_grid(3, height=lambda x, y: Fraction(x * x + y, 300)),
    "grid-triangles": lambda: lift_grid(3, merged=False),
    "ell": lift_ell,
    "fraction": lift_fraction_chart,
}


# --------------------------------------------------------------------------
# точки и многоугольники
# --------------------------------------------------------------------------


def _rational(value) -> SqrtSumV1:
    return SqrtSumV1.rational(Fraction(value))


def _coordinate(rng: random.Random, value: Fraction, noise: float, int_coefficients: bool) -> SqrtSumV1:
    """Координата: рациональная (иногда `int`-коэффициентом) либо со сдвигом `q * sqrt(m)`; изредка микроскопический сдвиг на сопряжении."""

    roll = rng.random()
    if roll < noise * 0.5:
        return _rational(value) + SqrtSumV1(((rng.choice((2, 3, 5, 6, 7)), Fraction(rng.randrange(1, 9), rng.choice((7, 11, 13)))),))
    if roll < noise * 0.7:
        return _rational(value) + _hard()
    if int_coefficients and Fraction(value).denominator == 1 and value:
        return SqrtSumV1(((1, int(value)),))
    return _rational(value)


def _hard() -> SqrtSumV1:
    shift = 90

    def scaled(radicand: int) -> int:
        return math.isqrt(radicand << (2 * shift))

    approximation = Fraction(scaled(2) + scaled(3) + 1, 1 << shift)
    return SqrtSumV1(((1, -approximation), (2, Fraction(1)), (3, Fraction(1))))


def _bbox(plane) -> tuple:
    xs = [float(x) for triangle in plane.triangles for x, _y in triangle.chart]
    ys = [float(y) for triangle in plane.triangles for _x, y in triangle.chart]
    return min(xs), max(xs), min(ys), max(ys)


def random_polygon(rng: random.Random, box: tuple, noise: float, int_coefficients: bool, outside: float) -> list:
    """Звёздный (простой) многоугольник вокруг случайного центра; вершины на решётке, на полуцелых либо вне карты (свес)."""

    x0, x1, y0, y1 = box
    centre = (rng.uniform(x0, x1), rng.uniform(y0, y1))
    radius = rng.uniform(0.15, 0.55) * min(x1 - x0, y1 - y0)
    count = rng.choice((3, 3, 4, 4, 4, 5, 5, 6, 7))
    angles = sorted(rng.uniform(0, 2 * math.pi) for _ in range(count))
    points = []
    for angle in angles:
        reach = radius * rng.uniform(0.5, 1.0) * (1 + (outside if rng.random() < 0.2 else 0.0))
        x = Fraction(round((centre[0] + reach * math.cos(angle)) * 2), 2)
        y = Fraction(round((centre[1] + reach * math.sin(angle)) * 2), 2)
        points.append((x, y))
    if len(set(points)) < 3:
        return []
    out = [(_coordinate(rng, x, noise, int_coefficients), _coordinate(rng, y, noise, int_coefficients)) for x, y in points]
    if rng.random() < 0.3:
        out.reverse()
    return out


def random_call(rng: random.Random, plane, index: int, *, noise: float, int_coefficients: bool, outside: float) -> dict | None:
    box = _bbox(plane)
    faces, points, cycles, polygons, seam_pairs = [], {}, [], [], []
    for _face in range(rng.choice((1, 1, 2, 3))):
        polygon = random_polygon(rng, box, noise, int_coefficients, outside)
        if not polygon:
            continue
        keys = []
        for vertex in polygon:
            kind = "src" if rng.random() < 0.35 else "node"
            key = f"{kind}:{len(points)}"
            points[key] = vertex
            keys.append(key)
        cycles.append([(key, points[key]) for key in keys])
        polygons.append((tuple(keys),))
        faces.append(keys)
        for number, key in enumerate(keys):
            if rng.random() < 0.25:
                seam_pairs.append(frozenset((key, keys[(number + 1) % len(keys)])))
    if not faces:
        return None
    fans = None if rng.random() < 0.4 else [rng.random() < 0.3 for _ in faces]
    flows = None if rng.random() < 0.5 else [rng.random() < 0.4 for _ in faces]
    return {
        "points": points,
        "cycles": cycles,
        "polygons": polygons,
        "law": rng.choice(LAWS),
        "seam": frozenset(seam_pairs),
        "fans": fans,
        "flows": flows,
        "by_faces": rng.random() < 0.6,
    }


def special_calls(rng: random.Random) -> list:
    """Нарочно собранные вызовы: самопересечение, дубликаты, огромные координаты, вершина вне карты, коэффициенты `int`."""

    def pt(x, y):
        return _rational(x), _rational(y)

    def call(vertices, **extra):
        keys = [f"node:{k}" for k in range(len(vertices))]
        points = dict(zip(keys, vertices))
        base = {
            "points": points,
            "cycles": [[(key, points[key]) for key in keys]],
            "polygons": [(tuple(keys),)],
            "law": LAWS[0],
            "seam": frozenset(),
            "fans": None,
            "flows": None,
            "by_faces": False,
        }
        base.update(extra)
        return base

    bow_tie = [pt(1, 1), pt(7, 7), pt(7, 1), pt(1, 7)]
    huge = SqrtSumV1(((1, Fraction(10**400)),))
    calls = [
        ("bow-tie", "square", call(bow_tie)),
        ("bow-tie-faces", "grid-flat", call(bow_tie, by_faces=True)),
        ("duplicate-vertex", "square", call([pt(1, 1), pt(5, 1), pt(5, 1), pt(5, 5), pt(1, 5)])),
        ("outside-the-map", "square", call([pt(-30, -30), pt(-20, -30), pt(-25, -20)])),
        ("overhang", "square", call([pt(2, 2), pt(14, 2), pt(14, 6), pt(2, 6)])),
        ("huge-coordinate", "square", call([(huge, _rational(1)), pt(3, 1), pt(3, 3)])),
        ("int-coefficients", "square", call([(SqrtSumV1(((1, 1),)), SqrtSumV1(((1, 1),))), (SqrtSumV1(((1, 7),)), SqrtSumV1(((1, 1),))), (SqrtSumV1(((1, 7),)), SqrtSumV1(((1, 5),))), (SqrtSumV1(((1, 1),)), SqrtSumV1(((1, 5),)))])),
        ("collinear", "square", call([pt(1, 1), pt(3, 3), pt(5, 5)])),
        ("on-the-diagonal", "square", call([pt(1, 1), pt(5, 5), pt(5, 2), pt(2, 5)])),
        ("hard-near-the-diagonal", "square", call([pt(1, 3), (_rational(4) + _hard(), _rational(4)), pt(6, 1), pt(1, 1)])),
        ("sqrt-across-the-diagonal", "square", call([(SqrtSumV1(((2, 1),)), _rational(5)), pt(7, 2), (_rational(2), SqrtSumV1(((3, 1),)))])),
    ]
    return calls


# --------------------------------------------------------------------------
# запись
# --------------------------------------------------------------------------


def _budget(rng: random.Random, number: int, starve: bool):
    caps = (0, 1, 2, 3, 5, 8, 13, 21) if starve else (None,)
    return exact.exact_work_budget(stage="MATERIALIZE", domain_id=f"generated-{number}", superlevel="", cap=rng.choice(caps))


def _run(recorder, label: str, lift, kwargs: dict, budget, *, warm: bool) -> str:
    """Один вызов через записывающую обёртку; исход (`CLIPPED` либо класс исключения) возвращается для инвентаризации."""

    recorder.context.update(mesh=f"generated/{label}", mesh_digest="", alpha=None, patch_id=None, domain_id=None)
    plane = lift.bind(budget)
    wrapped = recorder.wrap(nc.OP_CLIP, nc.ORACLE[nc.OP_CLIP])
    manager = contextlib.nullcontext() if warm else exact.isolated_factorization_memory()
    with manager:
        try:
            wrapped(plane, budget, **kwargs)
        except Exception as error:  # noqa: BLE001 - исход вызова — исключение эталона
            return type(error).__name__
    return "CLIPPED"


def generate(recorder, count: int = 220, seed: int = 20261006) -> Counter:
    """Пишет `count` случайных и все специальные вызовы; возвращает `Counter` исходов."""

    rng = random.Random(seed)
    outcomes: Counter = Counter()
    names = sorted(PLANES)
    for label, plane_name, kwargs in special_calls(rng):
        outcomes[_run(recorder, f"special-{label}", PLANES[plane_name](), kwargs, _budget(rng, 0, False), warm=False)] += 1
    for number in range(count):
        plane_name = names[number % len(names)]
        lift = PLANES[plane_name]()
        kwargs = random_call(
            rng, lift, number, noise=rng.choice((0.0, 0.0, 0.3, 0.8)), int_coefficients=rng.random() < 0.15, outside=rng.choice((0.0, 0.0, 0.5, 2.0))
        )
        if kwargs is None:
            continue
        starve = rng.random() < 0.25
        outcomes[_run(recorder, f"{plane_name}-{number:03d}", lift, kwargs, _budget(rng, number, starve), warm=rng.random() < 0.3)] += 1
    return outcomes
