"""Корпус многоугольников для стендов волнового фронта. Общий для обоих.

Вход волнового фронта обязан лежать на целочисленной решётке, поэтому корпус
собран так, чтобы это было выполнено по построению, а не проверкой:

- `axis_*` — осевые фигуры: все `q = |d|^2` полные квадраты, время рационально;
- `diamond`, `comb` — 45-градусные рёбра: `q = 2k^2`, радикал один и тот же;
- `star` — псевдослучайные звёздчатые контуры: `q` в общем положении;
- `field` — полевой контур `building.002`, приведённый к решётке через
  `robust.grid.snap_value`. Это единственный НЕ синтетический вход, который в
  репозитории есть, и он же единственный со «злыми» радикалами.

Своих порогов здесь нет: привязка полевого контура идёт объявленным законом
решётки, а не округлением «на глаз».
"""

from __future__ import annotations

from fractions import Fraction
import json
import math
from pathlib import Path

from cftuv_envelope.robust.grid import GridSpecV1, snap_value
from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1


REPO_ROOT = Path(__file__).resolve().parents[1]
FIELD_FIXTURE = (
    REPO_ROOT
    / "kernel"
    / "fixtures"
    / "building_002_point_contact_v1"
    / "analysis_snapshot.json"
)

# Полевой контур `building.002`, снятый с фикстуры путём
# codec -> compile_reference_envelopes -> GeometryContext.build ->
# build_sparse_patch_domain_geometry -> domain_regions[0].outer.
# Здесь он записан точными дробями с общим знаменателем, чтобы стенд не тянул
# за собой весь конвейер `reference/` (срез его не трогает).
FIELD_DENOMINATOR = 373821260323
FIELD_NUMERATORS = (
    (5351874934866, -1159816442698),
    (5351874934866, -1159816442698),
    (232403250106, 240403797009),
    (232403250106, 240403797009),
    (0, 373821260323),
    (0, 373821260323),
    (0, 0),
    (0, 0),
    (373821260323, 0),
    (373821260323, 0),
    (6652047654912, -1906216271872),
    (6652047654912, -1906216271872),
)


def axis_square(side: int) -> PolygonV1:
    return PolygonV1.build(((0, 0), (side, 0), (side, side), (0, side)))


def axis_rectangle(width: int, height: int) -> PolygonV1:
    return PolygonV1.build(((0, 0), (width, 0), (width, height), (0, height)))


def right_triangle(leg: int) -> PolygonV1:
    return PolygonV1.build(((0, 0), (leg, 0), (0, leg)))


def diamond(radius: int) -> PolygonV1:
    return PolygonV1.build(
        ((0, -radius), (radius, 0), (0, radius), (-radius, 0))
    )


def ell(size: int) -> PolygonV1:
    half = size // 2
    return PolygonV1.build(
        (
            (0, 0),
            (size, 0),
            (size, half),
            (half, half),
            (half, size),
            (0, size),
        )
    )


def comb(teeth: int, *, pitch: int = 12, depth: int = 6) -> PolygonV1:
    """Гребёнка с `teeth` вогнутыми зубьями. Та же форма, что мерилась у Трики."""

    width = pitch * (teeth + 1)
    points = [(0, 0), (width, 0), (width, depth + 4)]
    for index in range(teeth, 0, -1):
        points.append((pitch * index + pitch // 2, 4))
        points.append((pitch * index, depth + 4))
    points.append((0, depth + 4))
    return PolygonV1.build(tuple(points))


def holes_grid(rows: int, columns: int, *, pitch: int = 20) -> PolygonV1:
    """Прямоугольник с решёткой квадратных дыр. Полем не пройден никогда."""

    outer = (
        (0, 0),
        (pitch * columns, 0),
        (pitch * columns, pitch * rows),
        (0, pitch * rows),
    )
    holes = []
    for row in range(rows):
        for column in range(columns):
            x0, y0 = pitch * column + 6, pitch * row + 6
            holes.append(((x0, y0), (x0 + 8, y0), (x0 + 8, y0 + 8), (x0, y0 + 8)))
    return PolygonV1.build(outer, tuple(holes))


def star(vertices: int, seed: int, *, radius: int = 4096) -> PolygonV1 | None:
    """Звёздчатый контур на решётке. `None`, если вырождение не удалось снять.

    Псевдослучайность детерминированная и своя: `random` в стенде означал бы,
    что повтор замера даёт другой корпус.
    """

    state = seed * 6364136223846793005 + 1442695040888963407
    points: list[tuple[int, int]] = []
    for index in range(vertices):
        state = (state * 6364136223846793005 + 1442695040888963407) % (1 << 64)
        angle = 2 * math.pi * index / vertices
        scale = Fraction(1, 2) + Fraction(state % 1000, 2000)
        points.append(
            (
                int(round(radius * float(scale) * math.cos(angle))),
                int(round(radius * float(scale) * math.sin(angle))),
            )
        )
    if len({point for point in points}) != vertices:
        return None
    try:
        return PolygonV1.build(tuple(points))
    except Exception:
        return None


def field_polygon(scale: int) -> PolygonV1 | None:
    """Полевой контур `building.002`, привязанный к решётке `scale`."""

    grid = GridSpecV1(scale=scale)
    points: list[tuple[int, int]] = []
    for numerator_x, numerator_y in FIELD_NUMERATORS:
        node = (
            snap_value(Fraction(numerator_x, FIELD_DENOMINATOR), grid),
            snap_value(Fraction(numerator_y, FIELD_DENOMINATOR), grid),
        )
        if not points or node != points[-1]:
            points.append(node)
    if len(points) > 2 and points[0] == points[-1]:
        points.pop()
    if len(set(points)) != len(points) or len(points) < 3:
        return None
    try:
        return PolygonV1.build(tuple(points))
    except Exception:
        return None


def named_corpus() -> tuple[tuple[str, PolygonV1], ...]:
    """Именованный корпус стенда. Порядок фиксирован, значит воспроизводим."""

    entries: list[tuple[str, PolygonV1]] = [
        ("axis_square", axis_square(8)),
        ("axis_rectangle", axis_rectangle(20, 8)),
        ("right_triangle", right_triangle(12)),
        ("diamond", diamond(8)),
        ("ell", ell(12)),
        ("comb_2", comb(2)),
        ("comb_4", comb(4)),
        ("hole_1", holes_grid(1, 1)),
        ("holes_2", holes_grid(1, 2)),
    ]
    for scale in (64, 256, 1024):
        polygon = field_polygon(scale)
        if polygon is not None:
            entries.append((f"field_building_002_scale_{scale}", polygon))
    for seed in range(6):
        polygon = star(9, seed)
        if polygon is not None:
            entries.append((f"star_9_seed_{seed}", polygon))
    return tuple(entries)


def loop_points(polygon: PolygonV1) -> tuple[LoopV1, ...]:
    return polygon.loops


def field_fixture_exists() -> bool:
    return FIELD_FIXTURE.exists()


def field_fixture_digest() -> str:
    """Отпечаток фикстуры: стенд обязан падать, если контур в ней сменился."""

    import hashlib

    payload = json.loads(FIELD_FIXTURE.read_text(encoding="utf-8"))
    return hashlib.sha256(
        json.dumps(payload["source_revision"], sort_keys=True).encode("utf-8")
    ).hexdigest()[:16]
