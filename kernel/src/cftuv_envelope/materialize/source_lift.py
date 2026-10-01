"""Закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1`: вершина `src:` стоит там, где стоит вершина хоста.

ЗАЧЕМ. Ключ `src:<SourceVertexId>` называет ОДНУ вершину исходника, а лежала она у каждого
домена в СВОЁМ месте: подъём ставит узел решётки домена, и узлы соседних доменов расходятся
на шаг решётки (замер `2`: точное совпадение у 12 из 73 общих вершин, прочие — на 0.5…1 шаг
источника; у вершин прямых цепей — на метры, см. `evaluation_geometry`). Меш без общей вершины
нельзя сшить: хост сваривает вершины соседних доменов по `semantic_location_ref` ТОЛЬКО при
побитовом равенстве позиций, и равенство обязано обеспечить ядро.

ЗАКОН. Вершина `src:` кладётся в ТОЧНУЮ позицию вершины исходника из снапшота (один binary64
на вершину, одинаковый во ВСЕХ доменах), если её смещение от подъёма узла не больше бюджета
`SOURCE_VERTEX_LIFT_BUDGET_CELLS` ячеек источника. Ячейка — шаг решётки источника домена
(`grid_certificate.window_step`, метры): ею источник привязан, а решётка карты не крупнее её
(`chart_grid_for`). Сравнение ТОЧНОЕ: квадрат расстояния двух binary64-точек — рациональное
число (`Fraction` от float точен), бюджет — рациональный квадрат.

ЧТО ЗАКОН НЕ ДЕЛАЕТ МОЛЧА.
* Смещение больше бюджета — вершина остаётся на подъёме узла, и это называется:
  `SOURCE_VERTEX_DISPLACED_BY_LATTICE` (счёт и худшая вершина с числами). Так остаются
  вершины внутренностей объявленных прямых цепей (`evaluation_geometry`: сдвиг вдоль хорды
  до полушага ШАГА ХОРДЫ, метры на хорде с `gcd = 1`): в соседних доменах они не сойдутся.
* Позиции исходника либо ячейки нет (координатно-свободный вход, нет решётки) — вершина
  остаётся, счёт `..._HOST_POSITION_UNAVAILABLE`.
* Нет закона «грань остаётся гранью» без проверки: подъём двигает вершину не больше чем на
  бюджет, но тонкая грань могла бы от этого перевернуться (замер `building`: сливер высотой в
  доли ячейки из двух узлов на ребре и вершины источника — ровно такая грань). Поэтому
  ориентация КАЖДОГО канонического треугольника с подвинутой вершиной сверяется с ориентацией
  до подъёма (знак скалярного произведения векторов площади); перевернувшийся треугольник
  возвращает свои вершины на узлы (`SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION`, счёт), и
  проверка идёт до неподвижной точки (откат вершины меняет соседние треугольники).
  КАНОНИЧЕСКИЙ треугольник — треугольник закона `TRIANGLES_V1` (`tessellate.fan_out`: грань
  закона четырёхгранья даёт ровно прежние два): позиции вершин не вправе зависеть от закона
  топологии (семантический дайджест у законов один), поэтому проверка идёт по тому разбиению,
  которое у всех законов общее, а не по граням, которые закон выпустил.

ПЛОСКОСТЬ. Вершина исходника лежит на плоскости домена не точно: плоскость проходит через
позиции, привязанные к решётке источника домена, а хостовая позиция — не привязана, то есть
лежит в стороне до половины ячейки по каждой оси. Решение: вершина вправе сойти с носителя
подъёма на величину бюджета, но четырёхгранье перестаёт быть ТОЧНО плоским — оно плоское с
точностью до одной ячейки источника. Это записано, а не спрятано: наибольшее отклонение
четырёхгранников с подвинутой вершиной от плоскости идёт счётчиком в нанометрах
(`MATERIALIZE_QUADS_MAX_OFF_PLANE_NANOMETRES`; он считает ГРАНИ, поэтому, как все счётчики
граней, зависит от закона топологии и у `TRIANGLES_V1` нуль). Хост ставит смещение декали
вдоль нормали вершины (митра общих вершин), и отклонение от плоскости остаётся порядка той же
ячейки.

Без хостовой позиции, отличной от подъёма (синтетика с точными координатами), закон — нуль
действий: ни позиция, ни батч, ни дайджест не меняются, и диагностика не пишется.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

from ..numeric import LocalPoint3V1
from .tessellate import fan_out

SOURCE_VERTEX_LIFT_LAW = "SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1"

#: Бюджет смещения вершины `src:` от подъёма узла, в ячейках источника домена.
#: Запись реестра допусков `SOURCE_VERTEX_LIFT_BUDGET_V1`. Одна ячейка: источник привязан
#: к решётке ячейки `h`, и привязка двигает точку не больше чем на `h/2` по каждой оси, то
#: есть на `h·√3/2 < h`; решётка карты не крупнее (`chart_grid_for`), поэтому дальше ячейки
#: расхождение уже не шум привязки, а другое положение вершины (внутренность прямой цепи).
SOURCE_VERTEX_LIFT_BUDGET_CELLS = 1

LIFTED = "MATERIALIZE_SOURCE_VERTICES_LIFTED_AT_HOST"
DISPLACED = "MATERIALIZE_SOURCE_VERTICES_DISPLACED_BY_LATTICE"
UNAVAILABLE = "MATERIALIZE_SOURCE_VERTICES_HOST_POSITION_UNAVAILABLE"
ORIENTATION_KEPT = "MATERIALIZE_SOURCE_VERTICES_LIFT_REFUSED_BY_FACE_ORIENTATION"

QUAD_OFF_PLANE = "MATERIALIZE_QUADS_MAX_OFF_PLANE_NANOMETRES"
NANOMETRES_PER_METRE = 10**9

_PREFIX = "src:"


@dataclass(frozen=True, slots=True)
class SourceLiftV1:
    """Итог закона: позиции вершин и всё, что в них не сошлось, с числами."""

    positions: dict
    #: Вершины `src:` в итоговых позициях исходника (`positions[key]` — позиция хоста).
    lifted: int
    #: Из них действительно сдвинутые (позиция хоста не равна подъёму узла).
    moved: int
    #: Вершины `src:`, оставленные на подъёме: смещение больше бюджета.
    displaced: int
    #: Вершины, чьё положение хоста не известно (нет позиции либо нет ячейки).
    unavailable: int
    #: Вершины, возвращённые на узлы: контур с ними перевернулся бы.
    kept_for_orientation: int
    total: int
    #: Худшая из оставленных: `(ключ, расстояние в метрах)` либо `None`.
    worst_displaced: tuple | None
    #: Наибольшее смещение среди подвинутых, метры.
    max_lifted_displacement: float
    #: Бюджет в метрах (ячейка источника × число ячеек), либо `None`.
    budget: float | None
    #: Наибольшее отклонение от плоскости у четырёхгранников с подвинутой вершиной, метры.
    max_quad_deviation: float

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            (LIFTED, self.lifted),
            (DISPLACED, self.displaced),
            (UNAVAILABLE, self.unavailable),
            (ORIENTATION_KEPT, self.kept_for_orientation),
            (QUAD_OFF_PLANE, round(self.max_quad_deviation * NANOMETRES_PER_METRE)),
        )

    def lifted_note(self) -> str:
        return (
            f"{self.moved} of {self.total} source vertices lifted at the exact host position "
            f"(law {SOURCE_VERTEX_LIFT_LAW}); largest move {self.max_lifted_displacement:.6g} m "
            f"against the budget {self.budget:.6g} m "
            f"({SOURCE_VERTEX_LIFT_BUDGET_CELLS} source cell)"
        )

    def displaced_note(self) -> str:
        key, distance = self.worst_displaced
        return (
            f"{self.displaced} of {self.total} source vertices stay at their lattice lift, "
            f"farther than the budget {self.budget:.6g} m from the host position; worst "
            f"{key} is {distance:.6g} m away (declared straight chains slide their internal "
            "vertices along the chord)"
        )

    def orientation_note(self) -> str:
        return (
            f"{self.kept_for_orientation} source vertices stay at their lattice lift: the "
            "host position would turn a face contour over"
        )


def _distance_squared(first, second) -> Fraction:
    return sum(
        (
            (Fraction(a) - Fraction(b)) ** 2
            for a, b in zip(
                (first.x, first.y, first.z), (second.x, second.y, second.z), strict=True
            )
        ),
        Fraction(0),
    )


def _fan_area(points):
    """Вектор удвоенной площади контура: веер из первой вершины (как в аудите сетки)."""

    first = points[0]
    total = (0.0, 0.0, 0.0)
    for index in range(1, len(points) - 1):
        second, third = points[index], points[index + 1]
        ux, uy, uz = second.x - first.x, second.y - first.y, second.z - first.z
        vx, vy, vz = third.x - first.x, third.y - first.y, third.z - first.z
        total = (
            total[0] + (uy * vz - uz * vy),
            total[1] + (uz * vx - ux * vz),
            total[2] + (ux * vy - uy * vx),
        )
    return total


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _deviation(points) -> float:
    """Наибольшее расстояние вершины контура от плоскости, заданной его вектором площади."""

    normal = _fan_area(points)
    length = math.sqrt(_dot(normal, normal))
    if not length:
        return 0.0
    first = points[0]
    return max(
        abs(_dot((p.x - first.x, p.y - first.y, p.z - first.z), normal)) / length
        for p in points
    )


def _flipped_contours(contours, before, proposed, moved):
    """Индексы контуров с подвинутой вершиной, чья ориентация после подъёма не сохранилась."""

    found = []
    for index, keys in contours:
        if not any(key in moved for key in keys):
            continue
        after = _fan_area(tuple(proposed[key] for key in keys))
        if not _dot(before[index], after) > 0.0:
            found.append(index)
    return found


def lift_source_vertices(positions, polygons, host_positions, step) -> SourceLiftV1:
    """Закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` над подъёмом домена.

    `positions` — `{ключ: LocalPoint3V1}` подъёма узлов (`lift_vertices`); `polygons` — грани
    сетки домена по ключам вершин (после тесселяции и закона топологии: треугольники,
    четырёхгранья, любой длины от трёх); `host_positions` — `{SourceVertexId.value: позиция}`
    вершин исходника из снапшота; `step` — ячейка источника домена (метры, `Fraction`) либо
    `None`, если решётки нет.
    """

    names = tuple(key for key in positions if key.startswith(_PREFIX))
    budget = None if step is None else Fraction(step) * SOURCE_VERTEX_LIFT_BUDGET_CELLS
    budget_squared = None if budget is None else budget * budget
    allowed: dict = {}
    displaced = unavailable = 0
    worst = None
    for key in names:
        host = host_positions.get(key[len(_PREFIX):])
        if budget_squared is None or not isinstance(host, LocalPoint3V1):
            unavailable += 1
            continue
        squared = _distance_squared(positions[key], host)
        if squared <= budget_squared:
            allowed[key] = host
        else:
            displaced += 1
            if worst is None or squared > worst[1]:
                worst = (key, squared)
    contours = tuple(
        enumerate(
            triangle for polygon in polygons for triangle in fan_out(tuple(polygon))
        )
    )
    before = {
        index: _fan_area(tuple(positions[key] for key in keys)) for index, keys in contours
    }
    kept = 0
    while allowed:
        proposed = {**positions, **allowed}
        flipped = _flipped_contours(contours, before, proposed, allowed)
        if not flipped:
            break
        by_index = dict(contours)
        reverted = {
            key for index in flipped for key in by_index[index] if key in allowed
        }
        kept += len(reverted)
        for key in reverted:
            del allowed[key]
    final = {**positions, **allowed}
    changed = {key for key, host in allowed.items() if positions[key] != host}
    deviation = max(
        (
            _deviation(tuple(final[key] for key in polygon))
            for polygon in polygons
            if len(polygon) == 4 and any(key in changed for key in polygon)
        ),
        default=0.0,
    )
    biggest = max(
        (_distance_squared(positions[key], allowed[key]) for key in changed),
        default=Fraction(0),
    )
    return SourceLiftV1(
        positions=final,
        lifted=len(allowed),
        moved=len(changed),
        displaced=displaced,
        unavailable=unavailable,
        kept_for_orientation=kept,
        total=len(names),
        worst_displaced=(
            None if worst is None else (worst[0], math.sqrt(float(worst[1])))
        ),
        max_lifted_displacement=math.sqrt(float(biggest)),
        budget=None if budget is None else float(budget),
        max_quad_deviation=deviation,
    )


def host_positions_of(snapshot) -> dict:
    """`{SourceVertexId.value: позиция}` вершин исходника снапшота (координатно-свободные — как есть)."""

    return {item.vertex_id.value: item.position for item in snapshot.source_vertices}


def source_step_of(frame) -> Fraction | None:
    """Ячейка источника домена (метры, `Fraction`) либо `None`, если решётки источника нет."""

    certificate = getattr(frame, "grid_certificate", None)
    step = None if certificate is None else certificate.window_step
    return None if step is None else Fraction(step.numerator, step.denominator)


def rebind_offset_normals(plane, before, sourced) -> None:
    """Нормали смещения подъёма (у развёртки) следуют за вершинами, которые закон переставил."""

    rebind = getattr(plane, "rebind_position", None)
    if rebind is None:
        return
    for key, position in sourced.positions.items():
        if position != before[key]:
            rebind(before[key], position)
