"""Нормаль смещения над развёрнутой поверхностью: закон `SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`.

У развёрнутого домена нет плоскости источника, а значит и нормали плоскости, вдоль
которой хост сдвигает декаль против z-fighting. Решение ВЛАДЕЛЬЦА (2026-10-03): нормаль
ВЕРШИНЫ источника — сумма единичных нормалей инцидентных треугольников владельца с
весами углов при этой вершине, нормированная. Вершины домена общие, поэтому смещение
по нормали вершины не открывает щель на складке (нормаль треугольника открыла бы щель
на сгибе 90° при смещении 0.02), а у точки меша внутри треугольника нормаль — барицентрическое
смешение нормалей трёх его вершин, нормированное: непрерывно через ребро источника.

Арифметика — binary64 с фиксированным порядком операций над позициями, один раз
округлёнными из точных дробей: это направление смещения (политика отображения), а не
решение о геометрии, и закон именован, а не выведен из кода. Ни одна нормаль не
пропадает молча: нулевая нормаль вершины либо нормаль, смотрящая против нормали одного из
её треугольников (`n_v · n_T <= 0`), — именованный отказ `SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE`.
Знак нормали треугольника — обход вершин владельца (`SurfaceTriangleV1.vertex_ids`); карта
развёртки против часовой стрелки согласована с владельцем по построению, поэтому лицевая
сторона батча — именно эта.

ДОПУСК ГЛУБИНЫ ПРОТИВОСТОЯНИЯ (`SURFACE_OFFSET_OPPOSITION_TOLERATED`). Нормаль смещения над
точкой треугольника `T` — нормированное барицентрическое смешение нормалей трёх его вершин `n_i`,
и смещённая поверхность лежит над плоскостью `T` на `offset · (смесь · n_T) / |смесь|`. Знак «против»
(`n_v · n_T <= 0`) поэтому значит не «смещение внутрь поверхности вообще», а «в углу `v` смещённая
поверхность уходит ПОД плоскость `T` на `offset · |cos|`». У треугольника-иголки (`building` патч 89:
угол при вершине `building:34` — 0.17°, вес нормали нуль, косинус −3.08e-4) это микроны, и отказ целого
домена за них — не защита, а потеря патча. Закон: противостояние допускается, если ГРАНИЦА ГЛУБИНЫ не
больше `OFFSET_OPPOSITION_DEPTH_TOLERANCE` на опорном смещении `OFFSET_REFERENCE_METRES` (умолчание хоста
`DEFAULT_DECAL_OFFSET`, 0.02 м; настоящее смещение хозяин задаёт при записи, ядро его не знает, и
глубина растёт с ним линейно — это сказано в диагностике). Граница СТРОГАЯ: числитель смеси не меньше
`−D`, где `D = max(0, −min_i n_i · n_T)` (смесь — выпуклая комбинация), а длина смеси не меньше `μ`,
наименьшего `n_i · u` по единичной средней `u` трёх нормалей (`|смесь| >= смесь · u >= μ`); значит
глубина не больше `offset · D / μ`. `μ <= 0` — границы нет, отказ остаётся. Сравнение — точные дроби
двух binary64 (`D`, `μ`). Допущенные треугольники и худшая глубина ЗАПИСАНЫ (счётчики материализатора,
диагностика с именем треугольника и вершины); глубже допуска — прежний именованный отказ.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction
from hashlib import sha256

from .._cpython311 import left_fold_sum
from .admit import MaterializationOutcome
from .frames import MaterializationRefusal

OFFSET_NORMAL_LAW = "SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1"

#: Наибольшая глубина (метры) смещённой поверхности под плоскостью треугольника источника, при которой
#: противостояние нормали вершины его нормали допускается. Запись реестра допусков
#: `SURFACE_OFFSET_OPPOSITION_DEPTH_V1`: 0.1 мм — доля толщины листа декали и далеко ниже видимого.
OFFSET_OPPOSITION_DEPTH_TOLERANCE = Fraction(1, 10_000)

#: Смещение (метры), при котором глубина судится: умолчание хоста (`DEFAULT_DECAL_OFFSET`, сверено тестом
#: хоста). Той же записи реестра: допуск глубины без смещения ничего не значит.
OFFSET_REFERENCE_METRES = Fraction(1, 50)

OPPOSITION_TOLERATED = "MATERIALIZE_OFFSET_OPPOSITIONS_TOLERATED"
OPPOSITION_WORST_DEPTH = "MATERIALIZE_OFFSET_OPPOSITION_WORST_DEPTH_NANOMETRES"
NANOMETRES_PER_METRE = 10**9


@dataclass(frozen=True, slots=True)
class OppositionV1:
    """Допущенное противостояние: треугольник, вершина с худшим косинусом, косинус и граница глубины (метры)."""

    triangle: str
    vertex: str
    cosine: float
    depth: float


def _vector(first, second):
    return (second[0] - first[0], second[1] - first[1], second[2] - first[2])


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _length(vector) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector):
    length = _length(vector)
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _refusal(detail: str) -> MaterializationRefusal:
    return MaterializationRefusal(
        MaterializationOutcome.SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE, detail
    )


def opposition_bound(normals, triangle_normal):
    """`(худший косинус, граница глубины в метрах, в допуске ли)` либо `None`, если границы нет (`μ <= 0`).

    `normals` — три единичные нормали вершин треугольника, `triangle_normal` — его единичная нормаль.
    Граница `offset · D / μ` на опорном смещении (см. докстринг модуля); допуск судит ТОЧНОЕ
    сравнение дробей двух binary64 `D` и `μ`: `offset · D <= tolerance · μ`, без деления.
    """

    worst = min(_dot(normal, triangle_normal) for normal in normals)
    total = tuple(normals[0][axis] + normals[1][axis] + normals[2][axis] for axis in range(3))
    if not _length(total):
        return None
    mean = _unit(total)
    floor = min(_dot(normal, mean) for normal in normals)
    if not floor > 0.0:
        return None
    reach = max(0.0, -worst)
    within = (
        OFFSET_REFERENCE_METRES * Fraction(reach)
        <= OFFSET_OPPOSITION_DEPTH_TOLERANCE * Fraction(floor)
    )
    return worst, float(OFFSET_REFERENCE_METRES) * reach / floor, within


def source_vertex_normals(triangles, position, tolerated=None) -> dict:
    """`vertex_id -> единичная нормаль смещения` по треугольникам владельца.

    `triangles` — треугольники владельца (по имени), `position` — точные привязанные
    3D-позиции вершин. Отказывает, если нормаль вершины нулевая либо смотрит против
    нормали своего треугольника. `tolerated` — список для допущенных противостояний
    (`OppositionV1`): с ним противостояние в допуске глубины не отказ, а запись; без него
    (`None`) закон строгий, как прежде.
    """

    corners = {
        item.triangle_id: tuple(
            tuple(float(axis) for axis in position[vertex]) for vertex in item.vertex_ids
        )
        for item in triangles
    }
    unit = {}
    total: dict = {}
    for item in sorted(triangles, key=lambda entry: entry.triangle_id.value):
        points = corners[item.triangle_id]
        normal = _cross(_vector(points[0], points[1]), _vector(points[0], points[2]))
        if not _length(normal):
            raise _refusal(f"triangle {item.triangle_id.value} has no normal")
        unit[item.triangle_id] = _unit(normal)
        for index, vertex in enumerate(item.vertex_ids):
            first = _vector(points[index], points[(index + 1) % 3])
            second = _vector(points[index], points[(index + 2) % 3])
            angle = math.atan2(_length(_cross(first, second)), _dot(first, second))
            weighted = tuple(angle * axis for axis in unit[item.triangle_id])
            previous = total.get(vertex, (0.0, 0.0, 0.0))
            total[vertex] = tuple(a + b for a, b in zip(previous, weighted))
    result = {}
    for vertex, vector in sorted(total.items(), key=lambda entry: entry[0].value):
        if not _length(vector):
            raise _refusal(
                f"the angle-weighted normal of vertex {vertex.value} is zero"
            )
        result[vertex] = _unit(vector)
    for item in triangles:
        _judge_opposition(item, result, unit[item.triangle_id], tolerated)
    return result


def _judge_opposition(item, result, triangle_normal, tolerated) -> None:
    """Противостояние нормалей вершин треугольника его нормали: отказ либо допущенная запись."""

    normals = tuple(result[vertex] for vertex in item.vertex_ids)
    opposing = [
        vertex for vertex, normal in zip(item.vertex_ids, normals) if not _dot(normal, triangle_normal) > 0.0
    ]
    if not opposing:
        return
    bound = opposition_bound(normals, triangle_normal)
    if tolerated is not None and bound is not None and bound[2]:
        tolerated.append(OppositionV1(item.triangle_id.value, opposing[0].value, bound[0], bound[1]))
        return
    depth = "" if bound is None else (
        f" (depth bound {bound[1]:.6g} m at the reference offset {float(OFFSET_REFERENCE_METRES):g} m "
        f"against the tolerance {float(OFFSET_OPPOSITION_DEPTH_TOLERANCE):g} m)"
    )
    raise _refusal(
        f"the offset normal of vertex {opposing[0].value} opposes the normal "
        f"of its triangle {item.triangle_id.value}{depth}"
    )


def opposition_totals(tolerated) -> tuple[tuple[str, int], ...]:
    """Счётчики допущенных противостояний; пусто, если допускать было нечего (счётчики прежних доменов те же)."""

    if not tolerated:
        return ()
    worst = max(item.depth for item in tolerated)
    return (
        (OPPOSITION_TOLERATED, len(tolerated)),
        (OPPOSITION_WORST_DEPTH, math.ceil(worst * NANOMETRES_PER_METRE)),
    )


def opposition_note(tolerated) -> str:
    """Строка диагностики допущенных противостояний: худший треугольник, вершина, косинус, глубина."""

    if not tolerated:
        return ""
    worst = max(tolerated, key=lambda item: (item.depth, item.triangle))
    return (
        f"{len(tolerated)} source triangles tolerated: the vertex offset normal opposes the triangle at a "
        f"corner, so the offset surface dips below its plane by offset * |cosine|; worst depth "
        f"{worst.depth * NANOMETRES_PER_METRE:.0f} nm at the reference offset "
        f"{float(OFFSET_REFERENCE_METRES):g} m (triangle {worst.triangle}, vertex {worst.vertex}, "
        f"cosine {worst.cosine:.9g}; depth scales linearly with the host offset), within the tolerance "
        f"{float(OFFSET_OPPOSITION_DEPTH_TOLERANCE):g} m"
    )


def min_gap_cosine(triangles):
    """Наименьший `n_v . n_T` по углам треугольников подъёма: `(косинус, треугольник, угол)` либо `None`.

    Смещение декали на `d` вдоль нормали вершины поднимает её над плоскостью треугольника ровно
    на `d * (n_v . n_T)`: на складке внутри патча с двугранным углом `phi` это `d * cos(phi / 2)`,
    и на острой складке зазор стремится к нулю, пока косинус положителен (неположительный косинус
    — отказ `SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE`). Порога здесь нет, и быть без записи допуска
    не может: число ЗАПИСЫВАЕТСЯ диагностикой `DEVELOPABLE_OFFSET_MIN_GAP_COSINE`, а решает по нему
    владелец. Треугольники идут по имени, поэтому первый минимум определён однозначно.
    """

    best = None
    for item in triangles:
        if not item.normals:
            continue
        points = tuple(tuple(float(axis) for axis in corner) for corner in item.corners)
        normal = _unit(_cross(_vector(points[0], points[1]), _vector(points[0], points[2])))
        for index, vertex_normal in enumerate(item.normals):
            value = _dot(vertex_normal, normal)
            if best is None or value < best[0]:
                best = (value, item.name, index)
    return best


def offset_normals_digest(normals) -> str:
    """sha256 нормалей смещения вершин батча `((vert_key, (x, y, z)), ...)`; пусто, если нормалей нет.

    Нормали сдвигают вершины меша писателем хоста, но в дайджест батча не входят (смещение —
    политика отображения), поэтому без собственного дайджеста ни один ворота их не видели бы.
    Двоичные64 пишутся шестнадцатеричной формой `float.hex`: побитово, без десятичного округления.
    """

    if not normals:
        return ""
    lines = (
        "\x1f".join((key, *(float(axis).hex() for axis in vector)))
        for key, vector in sorted(normals, key=lambda entry: entry[0])
    )
    return sha256("\n".join(lines).encode("utf-8")).hexdigest()


def blend(weights, normals):
    """Нормаль точки внутри треугольника: барицентрическое смешение, нормированное."""

    mixed = tuple(
        left_fold_sum(weight * normal[axis] for weight, normal in zip(weights, normals))
        for axis in range(3)
    )
    if not _length(mixed):
        raise _refusal("the blended offset normal of a mesh vertex is zero")
    return _unit(mixed)
