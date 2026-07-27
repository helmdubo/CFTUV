"""Корпус многоугольников для стендов волнового фронта. Общий для обоих.

Вход волнового фронта обязан лежать на целочисленной решётке, поэтому корпус
собран так, чтобы это было выполнено по построению, а не проверкой:

- `axis_*`, `ell`, `cross`, `staircase`, `u_shape`, `double_notch`, `holes_*` —
  осевые фигуры: все `q = |d|^2` полные квадраты, время рационально. У этих же
  фигур митрованный отступ считается в рациональных числах, поэтому только на
  них применим независимый эталон `mitered_standard.py`;
- `diamond`, `comb` — 45-градусные рёбра: `q = 2k^2`, радикал один и тот же;
- `star` — псевдослучайные звёздчатые контуры: `q` в общем положении;
- `field` — полевой контур `building.002`, приведённый к решётке через
  `robust.grid.snap_value`. Это единственный НЕ синтетический вход, который в
  репозитории есть, и он же единственный со «злыми» радикалами.

Своих порогов здесь нет: привязка полевого контура идёт объявленным законом
решётки, а не округлением «на глаз».

Модуль живёт в `kernel/tests/`, а не в `tools/`, и это не вкусовщина: job
`kernel-extraction-readiness` копирует ОДИН каталог `kernel/` в пустой
репозиторий и запускает там тесты. Корпус, лежащий в `tools/`, сделал бы
`test_wavefront_faces.py` неимпортируемым после извлечения. `tools/wavefront_corpus.py`
остался тонким реэкспортом отсюда, чтобы у корпуса был один источник истины.
"""

from __future__ import annotations

from fractions import Fraction
import json
import math
from pathlib import Path

from cftuv_envelope.robust.grid import GridSpecV1, snap_value
from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1


KERNEL_ROOT = Path(__file__).resolve().parents[1]
FIELD_FIXTURE = (
    KERNEL_ROOT
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


def cross(
    *,
    wide: int = 4,
    tall: int = 4,
    right: int = 18,
    top: int = 20,
    arm: int = 4,
) -> PolygonV1:
    """Крест: вертикальная полоса ширины `wide`, горизонтальная высоты `tall`.

    Четыре вогнутые вершины сидят в углах центрального блока `wide x tall`,
    поэтому при `2*alpha > wide` их квадраты `alpha^2` обязаны пересечься. Это и
    есть тот отрицательный контроль, ради которого крест нужен: разность с
    полосами перестаёт быть суммой `alpha^2`, и перестаёт она не из-за дефекта.

    `arm` — толщина левого и нижнего рукавов, и это НЕ декорация. Она задаёт
    границу области, где держался прежний закон дефекта `-(wide - tall)^2`:
    закон был верен, пока `|wide - tall|` не превосходит `arm`. Записан этот
    закон был как безусловный ровно потому, что мерился на самодельной копии
    креста с `arm = 6`, где граница лежит за краем сетки 3..9. Параметр
    заведён, чтобы толщину можно было менять, НЕ заводя второй копии фигуры.

    История поломок на этой фигуре (обе ПОЧИНЕНЫ, запись оставлена, потому что
    отмена должна быть видимой):

    - `wide != tall`: `build_faces` отказывал
      `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON`, потому что среди узлов скелета не
      было пересечения гребней `(arm + wide/2, arm + tall/2)`;
    - `wide == tall` (квадратный блок): очередь отказывала
      `WAVEFRONT_LEFT_UNRESOLVED`, потому что блок схлопывался разом.

    Корень у обеих один: вогнутая вершина, пришедшая не в ребро, а в ВЕРШИНУ
    фронта, разбиралась как разрез. Теперь это встреча вершин с пересоединением
    по лучам (`skeleton.py`), и дефект нулевой на всей сетке.

    В `named_corpus()` крест по-прежнему НЕ входит, но причина сменилась и она
    измерена: `bridge_arrival_laws` отказывает на нём
    `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY`. У креста две пары коллинеарных
    сонаправленных рёбер, мост сопоставляет законы прихода рёбрам по НЕСУЩЕЙ
    ПРЯМОЙ, и часть границы остаётся без своего закона. Это та же болезнь
    «одна прямая на нескольких рёбрах», что и `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES`
    у `holes_2`, и к вырожденному событию она отношения не имеет.

    Свидетельство и числа — `test_wavefront_degenerate_event.py` (сетка 3..9 на
    трёх толщинах рукава), `test_wavefront_faces.py`,
    `test_wavefront_mitered_standard.py`.
    """

    left = bottom = arm
    return PolygonV1.build(
        (
            (left, 0),
            (left + wide, 0),
            (left + wide, bottom),
            (right, bottom),
            (right, bottom + tall),
            (left + wide, bottom + tall),
            (left + wide, top),
            (left, top),
            (left, bottom + tall),
            (0, bottom + tall),
            (0, bottom),
            (left, bottom),
        )
    )


def staircase() -> PolygonV1:
    """Лестница: две вогнутые вершины на диагонали, все рёбра осевые."""

    return PolygonV1.build(
        ((0, 0), (12, 0), (12, 4), (8, 4), (8, 8), (4, 8), (4, 12), (0, 12))
    )


def u_shape() -> PolygonV1:
    """П-образная фигура с зубьями РАЗНОЙ высоты.

    Разная высота не прихоть: у симметричной П два верхних ребра лежат на одной
    несущей прямой с одной ориентацией, и сборщик граней такую фигуру отвергает
    исходом `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES`. Здесь высоты 12 и 9, поэтому
    очередь до сравнения доходит.
    """

    return PolygonV1.build(
        ((0, 0), (16, 0), (16, 12), (11, 12), (11, 5), (5, 5), (5, 9), (0, 9))
    )


def double_notch() -> PolygonV1:
    """Два выреза РАЗНОЙ глубины: четыре вогнутые вершины, все прямые различны.

    Глубины ступенчатые (10/9/8 сверху, 4/3 снизу) ровно затем, чтобы ни одна
    пара рёбер не делила несущую прямую. Взаимные расстояния вогнутых вершин
    разные — 3 внутри выреза и 6 между вырезами, — поэтому квадраты `alpha^2`
    пересекаются не все разом, а по мере роста alpha.
    """

    return PolygonV1.build(
        (
            (0, 0),
            (20, 0),
            (20, 10),
            (14, 10),
            (14, 4),
            (11, 4),
            (11, 9),
            (5, 9),
            (5, 3),
            (2, 3),
            (2, 8),
            (0, 8),
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
        ("staircase", staircase()),
        ("u_shape", u_shape()),
        ("double_notch", double_notch()),
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
