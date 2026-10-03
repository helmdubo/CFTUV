"""Привязка вершин домена к карте подъёма ПЕРЕД резкой: шум решётки назван допуском, а не молчанием.

Два закона одного рода: (1) вершина `src:` встаёт в угол привязанной триангуляции; (2) вершина `node:` в долях
ячейки от ВНУТРЕННЕГО ребра источника встаёт на это ребро. Оба двигают точку карты на доли и единицы ячеек
решётки, оба названы счётчиками (правило 4 `AGENTS.md`: допуск, меняющий ответ, даёт названный записанный исход).

ЗАКОН (1): ВЕРШИНА `src:` В УГОЛ. Вершина `src:` многоугольника домена — образ вершины источника, и карта подъёма
(`lift_surface`) привязывает тот же образ к решётке ещё раз, независимо. Два округления одной точки (а у вершины
объявленной прямой цепи ещё и сдвиг вдоль хорды) расходятся: замер на `sagging_wall` — до 2.8 ячейки решётки.
Резка (`ClipStageV1._cut`) принимает многоугольник, только если площади кусков сходятся ТОЧНО, а вершина `src:`,
стоящая в нескольких ячейках от угла карты, площадь не сводит: 47 из 49 не-секторных треугольников домена 0
`sagging_wall` — уши свеса, шовные уши и уши «вершина не в углу»
(`MATERIALIZE_CLIP_FACES_OVERHANG_TRIANGULATED`, `..._SEAM_CROSSINGS_SUPPRESSED`,
`..._SOURCE_VERTEX_OFF_CORNER_SUPPRESSED`), то есть длинные диагональные треугольники на изогнутой стене вместо
четырёхгранников.

Вершина `src:`, не стоящая в углу карты ТОЧНО, но отстоящая от ближайшего угла не дальше
`SOURCE_VERTEX_CORNER_SNAP_CELLS` ячеек (точно: квадрат расстояния на `SqrtSumV1` под бюджетом), встаёт в этот
угол — ДО резки, во всех её стадиях одинаково. Исходы:

* `..._SNAPPED_TO_CORNER` — вершина привязана; `..._SNAP_MAX_GAP_MILLICELLS` — наибольший сдвиг в тысячных ячейки
  (единица допуска), `..._SNAP_MAX_GAP_NANOMETRES` — он же верхней оценкой в нанометрах (растяжение подъёма в
  треугольниках угла, `BoundSurfaceLiftV1.stretch_square`: у крутого треугольника ячейка длиннее, и оценка это показывает);
* `..._SNAP_REFUSED_CORNER_TAKEN` — в угол уже стоит другая вершина домена (точно) либо на него метят две вершины
  `src:`: привязка слила бы две вершины в одну, и она не делается;
* `..._SNAP_REFUSED_CORNERS_AMBIGUOUS` — в допуск попали два угла и больше: вершина не знает, чей она образ, и
  остаётся как есть (прежний путь, названный счётчиками резки).

Дальше допуска вершина не трогается: настоящий сдвиг (вершина прямой цепи, отстоящая от угла на глубину изгиба
цепи) не шум привязки, и его прежний путь — уши под счётчиком — остаётся.

ЗАКОН (2): ВЕРШИНА `node:` НА ВНУТРЕННЕМ РЕБРЕ. Перекладина полосы вдоль сетки меша лежит на ребре источника ТОЧНО
по построению (`rounded_wall.001`: рамка окна, перекладины идут по горизонтальным рёбрам сетки стены), а на карте
конец перекладины отстоит от прямой ребра на доли ячейки (замер: 0.011–0.29). Резка тогда отрезает от грани иглу
между перекладиной и ребром: площадь нулевая по смыслу и положительная точно (52 на `rounded_wall.001`, 12 на
`sagging_wall`: треугольники длиной в полосу и высотой в микроны). Здесь допуск стоит в ЗНАКЕ, а не в точке: знак
вершины `node:` относительно прямой ВНУТРЕННЕГО ребра области резки нулевой, если вершина отстоит от прямой не дальше
`NODE_EDGE_SNAP_CELLS` ячеек (точно: квадрат значения ориентации против квадрата допуска на квадрат длины ребра).
Вершина при этом не двигается: её факты `(s, r)` и классы рёбер (источник, фронт, стена) остаются точными, а сдвиг
вершины по ребру перевёл бы фронт в стену там, где резка ставит на нём вершину (`CLIP_VERTEX_ON_SEAM_CHAIN`).
Нулевой знак в обоих треугольниках ребра даёт тот же итог, что вершина на ребре: игла не рождается, куски
по-прежнему покрывают многоугольник ТОЧНО (знак у вершины один для обеих сторон ребра, и Сазерленд—Ходжмен делит
многоугольник по одной и той же линии для обоих треугольников), а доказательство «кусок в замкнутом треугольнике»
(`ClipStageV1._prove`) читает тот же знак. Следствие, названное цифрой: вершина `node:` куска лежит в его
треугольнике с точностью до `NODE_EDGE_SNAP_CELLS` ячеек по нормали к ребру, а подъём вершины идёт в треугольнике,
который содержит её точно. Исходы: `..._NODE_SIGNS_ZEROED_BY_EDGE_GAP` (пары «вершина, ребро области», чей точный
знак не нуль), `..._NODE_EDGE_GAP_MAX_MILLICELLS` и `..._NODE_EDGE_GAP_MAX_NANOMETRES` (наибольшее расстояние).

Привязанные к углам точки возвращаются стадией (`ClippedV1.snapped`), и домен кладёт их во ВСЕ последующие шаги
(подъём вершины, уши выпущенных кусков, положение вершин `src:`): у вершины в домене одна точка карты, а сосед,
у которого вершина стоит в углу точно, поднимает её в ту же позицию, и сварка по `location:src:` не расходится.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

from ..exact_sqrt_sum import SqrtSumV1
from .clip_cells import nanometres
from .coalesce import point_key
from .lift import ENCLOSURE_BITS

#: Допуск привязки вершины `src:` к углу привязанной триангуляции, ячейки решётки карты. ИЗМЕРЕНО: наибольший
#: сдвиг на `sagging_wall` 2.83 ячейки (по оси не более двух — два независимых округления плюс шум цепи); четыре —
#: запас, за которым это уже не шум привязки, а другая геометрия. Запись реестра допусков
#: `CLIP_SOURCE_VERTEX_CORNER_SNAP_CELLS_V1`.
SOURCE_VERTEX_CORNER_SNAP_CELLS = Fraction(4)

#: Допуск знака вершины `node:` у прямой внутреннего ребра источника, ячейки решётки карты. ИЗМЕРЕНО: перекладины
#: `rounded_wall.001` отстоят от прямой ребра на 0.011–0.29 ячейки; одна ячейка — запас на шум двух округлений по
#: оси ребра. Запись реестра допусков `CLIP_NODE_SOURCE_EDGE_GAP_CELLS_V1`.
NODE_EDGE_SNAP_CELLS = Fraction(1)

SOURCE_VERTICES_SNAPPED = "MATERIALIZE_CLIP_SOURCE_VERTICES_SNAPPED_TO_CORNER"
SOURCE_VERTEX_SNAP_MAX_GAP = "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_MAX_GAP_NANOMETRES"
SOURCE_VERTEX_SNAP_MAX_GAP_CELLS = "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_MAX_GAP_MILLICELLS"
SOURCE_VERTEX_SNAP_REFUSED_TAKEN = "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_REFUSED_CORNER_TAKEN"
SOURCE_VERTEX_SNAP_REFUSED_AMBIGUOUS = "MATERIALIZE_CLIP_SOURCE_VERTEX_SNAP_REFUSED_CORNERS_AMBIGUOUS"
NODE_SIGNS_ZEROED = "MATERIALIZE_CLIP_NODE_SIGNS_ZEROED_BY_EDGE_GAP"
NODE_EDGE_GAP_MAX = "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_NANOMETRES"
NODE_EDGE_GAP_MAX_CELLS = "MATERIALIZE_CLIP_NODE_EDGE_GAP_MAX_MILLICELLS"

COUNTER_NAMES = (
    SOURCE_VERTICES_SNAPPED,
    SOURCE_VERTEX_SNAP_MAX_GAP,
    SOURCE_VERTEX_SNAP_MAX_GAP_CELLS,
    SOURCE_VERTEX_SNAP_REFUSED_TAKEN,
    SOURCE_VERTEX_SNAP_REFUSED_AMBIGUOUS,
)


@dataclass(frozen=True, slots=True)
class CornerSnapV1:
    """Итог привязки вершин `src:` к углам: точки всех вершин после неё, сдвинутые вершины и числа."""

    #: `{ключ: точка}` ВСЕХ вершин домена: сдвинутые `src:` стоят в углах, остальные — те же объекты.
    points: dict
    #: `{ключ: угол}` только сдвинутых вершин `src:`.
    moved: dict
    #: `((имя, число), ...)` по `COUNTER_NAMES`; у домена без вершин `src:` все числа нулевые.
    counters: tuple


def _corner_grid(plane):
    """`({(ячейка сетки): [(угол, float x, float y)]}, {угол: наибольшее растяжение его треугольников})`."""

    step = float(SOURCE_VERTEX_CORNER_SNAP_CELLS)
    stretch: dict = {}
    for triangle in plane.triangles:
        square = plane.stretch_square(triangle)
        for corner in triangle.chart:
            stretch[corner] = max(stretch.get(corner, Fraction(0)), square)
    grid: dict = {}
    for corner in sorted(stretch):
        x, y = float(corner[0]), float(corner[1])
        grid.setdefault((math.floor(x / step), math.floor(y / step)), []).append((corner, x, y))
    return grid, stretch


def _near_corners(grid, point, plane):
    """Углы, чья float-оценка расстояния не больше допуска (с запасом на округление): фильтр, не ответ."""

    x_low, x_high, y_low, y_high = plane.window(point)
    step = float(SOURCE_VERTEX_CORNER_SNAP_CELLS)
    reach = step + 1.0
    found = []
    for cell_x in range(math.floor((x_low - reach) / step), math.floor((x_high + reach) / step) + 1):
        for cell_y in range(math.floor((y_low - reach) / step), math.floor((y_high + reach) / step) + 1):
            for corner, cx, cy in grid.get((cell_x, cell_y), ()):
                if x_low - reach <= cx <= x_high + reach and y_low - reach <= cy <= y_high + reach:
                    found.append(corner)
    return found


def _gap_square(point, corner) -> SqrtSumV1:
    dx = point[0] - SqrtSumV1.rational(corner[0])
    dy = point[1] - SqrtSumV1.rational(corner[1])
    return dx * dx + dy * dy


def milli_cells(square: Fraction) -> int:
    """Целая верхняя граница расстояния в тысячных ячейки по квадрату в ячейках: `ceil(sqrt(square · 10^6))`, точно."""

    scaled = square * 1_000_000
    whole = -(-scaled.numerator // scaled.denominator)
    root = math.isqrt(whole)
    return root if root * root == whole else root + 1


def _snap_to_corners(plane, budget, points, tally) -> tuple:
    """`({ключ: угол}, наибольший сдвиг в нанометрах, его наибольший квадрат в ячейках)` для вершин `src:` (закон 1)."""

    candidates = [key for key in points if key.startswith("src:")]
    if not candidates:
        return {}, 0, Fraction(0)
    limit = SqrtSumV1.rational(SOURCE_VERTEX_CORNER_SNAP_CELLS * SOURCE_VERTEX_CORNER_SNAP_CELLS)
    grid, stretch = _corner_grid(plane)
    taken = {point_key(point) for point in points.values()}
    proposals: dict = {}
    for key in candidates:
        point = points[key]
        within = []
        for corner in _near_corners(grid, point, plane):
            gap = _gap_square(point, corner)
            if (limit - gap).sign(budget=budget) >= 0:
                within.append((corner, gap))
        if not within or any(gap.is_zero for _corner, gap in within):
            continue
        if len(within) > 1:
            tally[SOURCE_VERTEX_SNAP_REFUSED_AMBIGUOUS] += 1
            continue
        proposals[key] = within[0]
    aimed: dict = {}
    for key, (corner, _gap) in proposals.items():
        aimed.setdefault(corner, []).append(key)
    moved: dict = {}
    widest, widest_square = 0, Fraction(0)
    for key, (corner, gap) in sorted(proposals.items()):
        target = (SqrtSumV1.rational(corner[0]), SqrtSumV1.rational(corner[1]))
        if len(aimed[corner]) > 1 or point_key(target) in taken:
            tally[SOURCE_VERTEX_SNAP_REFUSED_TAKEN] += 1
            continue
        moved[key] = target
        tally[SOURCE_VERTICES_SNAPPED] += 1
        upper = gap.enclosure(ENCLOSURE_BITS)[1]
        widest = max(widest, nanometres(upper * stretch[corner]))
        widest_square = max(widest_square, upper)
    return moved, widest, widest_square


def snap_source_vertices(plane, budget, points) -> CornerSnapV1:
    """Вершины `src:` в нескольких ячейках от угла карты встают в угол (закон 1); остальное не тронуто."""

    tally = dict.fromkeys(COUNTER_NAMES, 0)
    moved, widest, widest_square = _snap_to_corners(plane, budget, points, tally)
    tally[SOURCE_VERTEX_SNAP_MAX_GAP] = widest
    tally[SOURCE_VERTEX_SNAP_MAX_GAP_CELLS] = milli_cells(widest_square)
    return CornerSnapV1({**points, **moved}, moved, tuple((name, tally[name]) for name in COUNTER_NAMES))


def within_edge_gap(value, edge_square, budget):
    """`(в допуске, верхняя граница квадрата расстояния в ячейках)` вершины от прямой ребра (закон 2).

    `value` — значение ориентации вершины у ребра (`|ребро| · расстояние`, `SqrtSumV1`), `edge_square` — точный
    квадрат длины ребра в ячейках. Оболочка `value` — строгий фильтр: вершина, чей наименьший по модулю конец оболочки
    уже дальше допуска, отброшена без точного сравнения; остальные решает точный знак `допуск² · |ребро|² − value²`.
    """

    low, high = value.enclosure(ENCLOSURE_BITS)
    limit = NODE_EDGE_SNAP_CELLS * NODE_EDGE_SNAP_CELLS * edge_square
    nearest = Fraction(0) if low <= 0 <= high else min(low * low, high * high)
    if nearest > limit:
        return False, Fraction(0)
    if (SqrtSumV1.rational(limit) - value * value).sign(budget=budget) < 0:
        return False, Fraction(0)
    return True, max(low * low, high * high) / edge_square
