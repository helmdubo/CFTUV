"""Фазз-генератор вызовов `clip_geometry` для сверки нативной резки с эталоном (тест `tests/test_native_clip_geometry.py`).

Генераторы корпуса (`native_clip_generated`) режут независимые звёздные многоугольники. Здесь вызовы похожи на настоящие домены: ЛЕНТА граней со
ВСЕМИ общими вершинами (две направляющие, четырёхгранья между ними, общие рёбра), вершины на линиях решётки, на диагоналях ячеек и на углах (точные
нули знака, вершины `on`), рядом с ними (допуск закона 2 у `node:`, привязка `src:` к углу), с иррациональным шумом, швы на направляющих, веера,
регионы потока и все законы топологии. Планы — решётки из ячеек разного вида (два треугольника в обе стороны диагонали, веер из четырёх, одиночные
треугольники, Г-образная невыпуклая грань, нецелая карта), с разной высотой (плоско, изгиб, зубья) и нормалями смещения (в том числе противоположными).

Исход решает эталон: вход может быть нелепым (самопересечение, вершина вне карты), и тогда обе стороны обязаны дать один и тот же названный отказ.
Генератор детерминирован: `fuzz_cases(seed, count)` отдаёт `(метка, подъём, kwargs, потолок бюджета)`.
"""

from __future__ import annotations

import math
import random
from fractions import Fraction

import native_clip_generated as generated

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1

LAWS = (
    DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    DecalTopologyLawV1.PLANAR_POLYGONS_V1,
    DecalTopologyLawV1.QUAD_STRIPS_V1,
    DecalTopologyLawV1.TRIANGLES_V1,
    DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1,
)

HEIGHTS = {
    "flat": lambda x, y: 0,
    "slope": lambda x, y: Fraction(x + 2 * y, 200),
    "bend": lambda x, y: Fraction(x * x + y * 3, 400),
    "teeth": lambda x, y: Fraction((x * 7 + y * 13) % 9, 60),
    "ridge": lambda x, y: Fraction(abs(x - 12), 30),
}


def _unit_normals(rng: random.Random, mode: str) -> tuple:
    """Нормали смещения трёх углов: нет, общие, разные, противоположные у двух углов (смесь в середине ребра даёт нуль)."""

    up = (0.0, 0.0, 1.0)
    if mode == "none":
        return ()
    if mode == "same":
        return (up, up, up)
    if mode == "tilted":
        tilt = (0.6, 0.0, 0.8)
        return (up, tilt, (0.0, 0.6, 0.8))
    if mode == "opposed":
        return (up, (0.0, 0.0, -1.0), up)
    return (up, (0.28, 0.0, 0.96), (0.0, 0.28, 0.96)) if rng.random() < 0.5 else (up, up, (0.0, 0.0, 1.0))


def _item(name, chart, scale, height, face="", normals=()):
    corners = tuple((Fraction(x, scale), Fraction(y, scale), Fraction(height(x, y))) for x, y in chart)
    return (name, tuple(chart), corners, normals, face)


def grid_plane(rng: random.Random) -> SurfaceLiftV1:
    """Решётка `n x m` ячеек: каждая — две диагонали на выбор, веер из четырёх, одиночные треугольники либо пара граней; плюс Г-образная грань."""

    columns, rows = rng.choice((2, 2, 3, 3)), rng.choice((1, 2, 3))
    cell = rng.choice((4, 6, 8, 10, 16))
    height = HEIGHTS[rng.choice(sorted(HEIGHTS))]
    normals_mode = rng.choice(("none", "none", "same", "tilted", "opposed", "mixed"))
    items = []
    for column in range(columns):
        for row in range(rows):
            x0, y0 = column * cell, row * cell
            x1, y1 = x0 + cell, y0 + cell
            face = f"f{column}.{row}"
            kind = rng.choice(("diag-a", "diag-a", "diag-b", "fan", "split", "ell"))
            normals = _unit_normals(rng, normals_mode)
            if kind == "diag-a":
                charts = [((x0, y0), (x1, y0), (x1, y1)), ((x0, y0), (x1, y1), (x0, y1))]
            elif kind == "diag-b":
                charts = [((x0, y0), (x1, y0), (x0, y1)), ((x1, y0), (x1, y1), (x0, y1))]
            elif kind == "fan":
                mx, my = x0 + cell // 2, y0 + cell // 2
                charts = [((x0, y0), (x1, y0), (mx, my)), ((x1, y0), (x1, y1), (mx, my)), ((x1, y1), (x0, y1), (mx, my)), ((x0, y1), (x0, y0), (mx, my))]
            elif kind == "ell" and cell >= 8:
                half = cell // 2
                charts = [
                    ((x0, y0), (x1, y0), (x1, y0 + half)),
                    ((x0, y0), (x1, y0 + half), (x0 + half, y0 + half)),
                    ((x0, y0), (x0 + half, y0 + half), (x0 + half, y1)),
                    ((x0, y0), (x0 + half, y1), (x0, y1)),
                ]
            else:
                charts = [((x0, y0), (x1, y0), (x1, y1)), ((x0, y0), (x1, y1), (x0, y1))]
                face = ""
            for number, chart in enumerate(charts):
                items.append(_item(f"{face or 't'}{column}{row}.{number}", chart, 4, height, face, normals))
    return SurfaceLiftV1.from_triangles(items, scale=4)


def _noise(rng: random.Random) -> SqrtSumV1:
    radicand = rng.choice((2, 3, 5, 6, 7, 10, 11, 13))
    return SqrtSumV1(((radicand, Fraction(rng.randrange(1, 7), rng.choice((9, 11, 17, 23, 31)))),))


def _coordinate(rng: random.Random, value: Fraction, mode: str, int_coefficients: bool) -> SqrtSumV1:
    base = SqrtSumV1.rational(value)
    if mode == "exact":
        if int_coefficients and value.denominator == 1 and value:
            return SqrtSumV1(((1, int(value)),))
        return base
    if mode == "near":
        return SqrtSumV1.rational(value + Fraction(rng.choice((-1, 1)) * rng.randrange(1, 8), rng.choice((5, 9, 13, 17))))
    if mode == "noise":
        return base + _noise(rng)
    if mode == "hard":
        return base + generated._hard()
    return base


def _snap(rng: random.Random, value: float, cell: int) -> Fraction:
    """Значение на решётке: целое, полу- и четвертьцелое, либо кратное ячейке (линия или угол)."""

    roll = rng.random()
    if roll < 0.08:
        return Fraction(round(value / cell) * cell)
    if roll < 0.3:
        return Fraction(round(value))
    return Fraction(round(value * 4), 4)


def strip_call(rng: random.Random, lift: SurfaceLiftV1, *, hard: float) -> dict | None:
    """Лента граней вдоль случайной линии: общие вершины, швы на направляющих, шум и допуски."""

    xs = [float(x) for triangle in lift.triangles for x, _y in triangle.chart]
    ys = [float(y) for triangle in lift.triangles for _x, y in triangle.chart]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    cell = max(1, round(min(x1 - x0, y1 - y0) / 2))
    corners = sorted({corner for triangle in lift.triangles for corner in triangle.chart})
    inside = rng.random() < 0.85
    count = rng.choice((1, 2, 3, 4))
    angle = rng.uniform(0, math.pi)
    length = rng.uniform(0.5, 1.1) * math.hypot(x1 - x0, y1 - y0)
    width = rng.uniform(0.08, 0.45) * min(x1 - x0, y1 - y0)
    centre = (rng.uniform(x0 + (x1 - x0) * 0.2, x1 - (x1 - x0) * 0.2), rng.uniform(y0 + (y1 - y0) * 0.2, y1 - (y1 - y0) * 0.2))
    direction, across = (math.cos(angle), math.sin(angle)), (-math.sin(angle), math.cos(angle))
    mode_weights = ("exact",) * 6 + ("near", "noise") + (("hard",) if hard else ())
    int_coefficients = rng.random() < 0.15
    points: dict = {}
    rails = []
    for side in (0, 1):
        rail = []
        for number in range(count + 1):
            t = (number / count - 0.5) * length
            px = centre[0] + direction[0] * t + across[0] * width * side
            py = centre[1] + direction[1] * t + across[1] * width * side
            if inside:
                px, py = min(max(px, x0 + 0.3), x1 - 0.3), min(max(py, y0 + 0.3), y1 - 0.3)
            vx, vy = _snap(rng, px, cell), _snap(rng, py, cell)
            kind = "src" if rng.random() < 0.12 else "node"
            if kind == "src":
                # a mesh vertex stands in a corner of the map, or a little off it (the snap of law 1 moves it back)
                corner = min(corners, key=lambda item: (float(item[0]) - px) ** 2 + (float(item[1]) - py) ** 2)
                vx, vy = corner[0] + rng.choice((0, 0, 0, 1, -1)), corner[1] + rng.choice((0, 0, 0, 1, 2))
            key = f"{kind}:{len(points)}"
            points[key] = (_coordinate(rng, vx, rng.choice(mode_weights), int_coefficients), _coordinate(rng, vy, rng.choice(mode_weights), int_coefficients))
            rail.append(key)
        rails.append(rail)
    cycles, polygons, boundary = [], [], []
    for number in range(count):
        quad = [rails[0][number], rails[0][number + 1], rails[1][number + 1], rails[1][number]]
        if rng.random() < 0.15:
            quad.reverse()
        cycles.append([(key, points[key]) for key in quad])
        shape = rng.random()
        if shape < 0.2:
            polygons.append(((quad[0], quad[1], quad[2]), (quad[0], quad[2], quad[3])))
        else:
            polygons.append((tuple(quad),))
        boundary.append((quad[0], quad[1]))
        boundary.append((quad[2], quad[3]))
    if len(set(cycle_key for cycle in cycles for cycle_key, _ in cycle)) < 3:
        return None
    seam = frozenset(frozenset(pair) for pair in boundary if rng.random() < 0.3)
    fans = None if rng.random() < 0.4 else [rng.random() < 0.3 for _ in cycles]
    flows = None if rng.random() < 0.5 else [rng.random() < 0.4 for _ in cycles]
    return {
        "points": points,
        "cycles": cycles,
        "polygons": polygons,
        "law": rng.choice(LAWS),
        "seam": seam,
        "fans": fans,
        "flows": flows,
        "by_faces": rng.random() < 0.65,
    }


def fuzz_cases(seed: int, count: int):
    """`(метка, подъём, kwargs, потолок бюджета)`: ленты по решётчатым планам и звёздные многоугольники корпуса на тех же планах."""

    rng = random.Random(seed)
    for number in range(count):
        lift = grid_plane(rng)
        if rng.random() < 0.2:
            kwargs = generated.random_call(
                rng, lift, number, noise=rng.choice((0.0, 0.3, 0.8)), int_coefficients=rng.random() < 0.15, outside=rng.choice((0.0, 0.0, 0.5, 2.0))
            )
        else:
            kwargs = strip_call(rng, lift, hard=rng.random() < 0.1)
        if kwargs is None:
            continue
        cap = rng.choice((0, 1, 2, 3, 5, 8, 13, 21, 55, 144)) if rng.random() < 0.2 else None
        kwargs = generated.with_plan(f"fuzz-{seed}-{number}", kwargs, lift)
        yield f"fuzz-{seed}-{number:04d}", lift, kwargs, cap


# --------------------------------------------------------------------------
# нарочно собранные вызовы: ветки, до которых случайный вход не доходит
# --------------------------------------------------------------------------


def _pt(x, y):
    return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))


def _call(vertices, *, prefix="node", **extra) -> dict:
    keys = [f"{prefix}:{index}" for index in range(len(vertices))]
    points = dict(zip(keys, vertices))
    base = {
        "points": points,
        "cycles": [[(key, points[key]) for key in keys]],
        "polygons": [(tuple(keys),)],
        "law": DecalTopologyLawV1.PLANAR_POLYGONS_V1,
        "seam": frozenset(),
        "fans": None,
        "flows": None,
        "by_faces": False,
    }
    base.update(extra)
    return base


def opposed_normals_plane() -> SurfaceLiftV1:
    """Квадрат, у которого углы диагонали несут противоположные нормали: вершина в середине диагонали смешивает нуль (`blend` отказывает)."""

    up, down = (0.0, 0.0, 1.0), (0.0, 0.0, -1.0)
    return SurfaceLiftV1.from_triangles(
        [
            _item("t0", ((0, 0), (8, 0), (8, 8)), 4, HEIGHTS["flat"], "", (up, up, down)),
            _item("t1", ((0, 0), (8, 8), (0, 8)), 4, HEIGHTS["flat"], "", (up, down, up)),
        ],
        scale=4,
    )


def special_cases():
    """`(метка, подъём, kwargs, потолок)`: переполнение рационального знака, слишком малая иррациональная координата, нуль нормали, лишние ключи, обрезка вееров."""

    square = generated.PLANES["square"]
    tiny = SqrtSumV1(((2, Fraction(1, 10**320)),))
    far = SqrtSumV1.rational(Fraction(10**308))
    yield "overflowing-rational-sign", square(), _call([_pt(1, 1), _pt(3, 1), (far, SqrtSumV1.rational(Fraction(3)))]), None
    yield "overflowing-rational-sign-faces", generated.PLANES["grid-flat"](), _call([_pt(1, 1), _pt(3, 1), (far, SqrtSumV1.rational(Fraction(3)))], by_faces=True), None
    yield "tiny-irrational-coordinate", square(), _call([_pt(1, 1), _pt(5, 1), (SqrtSumV1.rational(Fraction(3)) + tiny, SqrtSumV1.rational(Fraction(4)))]), None
    yield "tiny-irrational-coordinate-src", square(), _call([_pt(1, 1), _pt(5, 1), (SqrtSumV1.rational(Fraction(3)) + tiny, SqrtSumV1.rational(Fraction(4)))], prefix="src"), None
    yield "opposed-normals-at-the-midpoint", opposed_normals_plane(), _call([_pt(2, 6), _pt(6, 2), _pt(7, 7), _pt(3, 7)], by_faces=False), None
    yield "opposed-normals-at-the-midpoint-faces", opposed_normals_plane(), _call([_pt(2, 6), _pt(6, 2), _pt(7, 7), _pt(3, 7)], by_faces=True), None
    vertices = [_pt(1, 1), _pt(5, 1), _pt(5, 5), _pt(1, 5)]
    missing_polygon = _call(vertices)
    missing_polygon["polygons"] = [(("node:0", "node:1", "node:9"),)]
    yield "a-polygon-key-no-vertex-has", square(), missing_polygon, None
    missing_cycle = _call(vertices)
    missing_cycle["cycles"] = [[("node:0", vertices[0]), ("node:8", vertices[1])]]
    yield "a-cycle-key-no-vertex-has", square(), missing_cycle, None
    missing_seam = _call(vertices, seam=frozenset({frozenset(("node:0", "node:7"))}))
    yield "a-seam-key-no-vertex-has", square(), missing_seam, None
    yield "fans-and-flows-shorter-than-the-faces", generated.PLANES["grid-flat"](), _call(vertices, fans=[], flows=[True], by_faces=True), None
    two_faces = _call(vertices)
    second = [_pt(2, 2), _pt(4, 2), _pt(4, 4)]
    for index, vertex in enumerate(second):
        two_faces["points"][f"node:{10 + index}"] = vertex
    two_faces["cycles"].append([(f"node:{10 + index}", second[index]) for index in range(3)])
    two_faces["polygons"].append((("node:10", "node:11", "node:12"),))
    two_faces["fans"], two_faces["flows"] = [False], [True, True]
    yield "the-first-list-wins-the-zip", square(), two_faces, None
    yield "no-faces-at-all", square(), _call(vertices, cycles=[], polygons=[]), None
    yield "a-face-without-polygons", square(), _call(vertices, polygons=[()]), None
    yield "no-points-of-the-domain", square(), {**_call(vertices), "points": {}, "cycles": [], "polygons": []}, None
