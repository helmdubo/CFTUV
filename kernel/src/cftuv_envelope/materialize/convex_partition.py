"""Закон `CONVEX_PARTITION_BY_DIAGONALS_V1`: кусок, которому не хватило закона станций, режется на НАИМЕНЬШЕЕ число допустимых граней диагоналями между его же вершинами.

ЗАЧЕМ. Закон станций (`slabs`) ставит новую вершину на ребро куска, а ребро источника и ребро, общее у двух
кусков, новых вершин не терпят (шов с соседним доменом, T-стык соседа): у куска полосы потока, у которого источник
внизу, а общий с соседом фронт вверху, отказывают ВСЕ разрезы (`REFUSED_ENDPOINT_ON_SEAM_EDGE`). Остаются уши, а
они — самое дорогое разбиение: диагональ на каждую вершину сверх третьей. Но допустимая грань не обязана быть
треугольником: строго выпуклый многоугольник потока с билинейной UV уже допускается (`QUAD_UV_BILINEAR_V1`,
`_bilinear_ring`), аффинный многоугольник — тем более, а ушам эта допустимость не нужна. Диагональ между двумя
СУЩЕСТВУЮЩИМИ вершинами не рождает вершин и не касается шва.

ЗАКОН. Грань куска — кольцо его вершин (кольцо против часовой на карте). Закон ищет разбиение кольца
непересекающимися внутренними диагоналями, в котором КАЖДАЯ часть допустима тем же законом, что и целый кусок
(`admissible`: треугольник; аффинная UV; строго выпуклый многоугольник потока с билинейной UV), а число частей
наименьшее. Поиск — точное запоминающее разбиение по частям-подкольцам (подкольцо — циклический отрезок исходного
порядка вершин): целиком допустимое подкольцо — одна часть; иначе перебор диагоналей по возрастанию пары индексов,
первая из равных по числу частей побеждает (детерминизм). Диагональ допустима, когда ни одно ребро подкольца её не
пересекает трансверсально, ни одна другая вершина не лежит на ней, и она входит в угол каждого своего конца внутрь
(знак `orientation` каждого шага точный). Допусков нет.

ДОКАЗАТЕЛЬСТВО. Тот же довод, что у закона станций: каждая часть — простой многоугольник положительной ориентации;
полурёбра частей сокращаются парами (диагонали по одной в каждую сторону), а остаток — ровно контур куска. Тогда части
покрывают кусок один раз без щелей и наложений. Дополнительно (вызывающий): ориентация образа каждой части в UV та
же, что у образа куска (складка UV не вызывается разбиением), сумма площадей частей равна площади куска точно.
Отказ любого пункта — кусок остаётся на ушах (прежний путь, побитово) под ИМЕНЕМ причины. Разбиение не короче ушей
(`n - 2` частей: у куска вне потока допустимы одни треугольники) выигрыша не даёт — `NO_GAIN_OVER_EARS`, уши прежние
побитово, а не другая триангуляция того же числа граней.

ГРАНИЦА РАБОТЫ — ИМЕНОВАННАЯ. Поиск стоит порядка `n^4` знаков на кусок (измерено на случайных, почти сплошь
невыпуклых контурах: 10 вершин — до 0.12 с, 12 — до 0.9 с, 14 — до 7 с, 16 — до 30 с); куски потока поля имеют до 9
вершин. Кусок длиннее `MAX_VERTICES` не ищется и отказан `TOO_MANY_VERTICES`: это граница работы, а не геометрии, и
она записана числом отказов, ответ ушей остаётся прежним.

ИЗМЕРЕНО НА ПОЛЕ (headless, кнопка, `E:/testScene.blend`, Fan Density 2; грани / рёбра / грани с углом меньше пяти градусов в
меше). `sagging_wall`, alpha 0.987, Max stretch 42 %: 224 / 416 / 43 -> 153 / 302 / 26, ушей 57 граней (15 кусков) -> 4 (1 кусок
со складкой UV); `rounded_wall_noise_top`, alpha 0.5, 42 %: 557 / 1051 / 33 -> 472 / 910 / 19, ушей 78 -> 0; `building`,
alpha 0.25: 1012 / 2248 / 24 -> 998 / 2235 / 16, ушей 25 -> 0; меш `2` и домены без ушей побитово те же;
`ADAPTER_SEAM_T_JUNCTIONS` = 0 везде (вершин закон не рождает).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from functools import lru_cache

from ..wavefront.faces import orientation, segments_cross, shoelace_sign
from .tessellate import contour_is_simple, counter_clockwise_ring

#: Наибольшее число вершин куска, для которого ищется разбиение (граница работы: число отказов записано).
MAX_VERTICES = 12

#: Имена чисел закона (они же ключи счётчиков материализатора). Нулевые числа в счётчики не пишутся.
PIECES_PARTITIONED = "MATERIALIZE_CONVEX_PARTITION_PIECES"
FACES_EMITTED = "MATERIALIZE_CONVEX_PARTITION_FACES_EMITTED"
DIAGONALS = "MATERIALIZE_CONVEX_PARTITION_DIAGONALS"
PIECES_REFUSED = "MATERIALIZE_CONVEX_PARTITION_PIECES_REFUSED"
REFUSED_PREFIX = "MATERIALIZE_CONVEX_PARTITION_REFUSED_"

REASON_TOO_MANY_VERTICES = "TOO_MANY_VERTICES"
REASON_NO_PARTITION = "NO_ADMISSIBLE_PARTITION"
REASON_NO_GAIN = "NO_GAIN_OVER_EARS"
REASON_NOT_A_SUBDIVISION = "NOT_A_SUBDIVISION"
REASON_UV_FOLD = "UV_FOLD"
REASON_AREA = "AREA_DOES_NOT_CLOSE"
REASONS = (
    REASON_TOO_MANY_VERTICES,
    REASON_NO_PARTITION,
    REASON_NO_GAIN,
    REASON_NOT_A_SUBDIVISION,
    REASON_UV_FOLD,
    REASON_AREA,
)


@dataclass(frozen=True, slots=True)
class PartitionPlanV1:
    """Части куска: кольца индексов входного кольца против часовой на карте; диагоналей на единицу меньше частей."""

    pieces: tuple
    order: tuple


@dataclass(frozen=True, slots=True)
class PartitionRefusalV1:
    """Именованный отказ закона: куска не разбили, причина названа (`REASON_*`)."""

    reason: str
    detail: str = ""


def _on_open_segment(first, second, other, budget) -> bool:
    """Точка `other` лежит на ОТКРЫТОМ отрезке `first - second` (коллинеарна и строго между концами). Точно."""

    if orientation(first, second, other, budget):
        return False
    along = (
        (second[0] - first[0]) * (other[0] - first[0]) + (second[1] - first[1]) * (other[1] - first[1])
    ).sign(budget=budget)
    beyond = (
        (first[0] - second[0]) * (other[0] - second[0]) + (first[1] - second[1]) * (other[1] - second[1])
    ).sign(budget=budget)
    return along > 0 and beyond > 0


def _enters_interior(ring, index: int, target, budget) -> bool:
    """Направление из вершины `ring[index]` на точку `target` входит в угол кольца (против часовой) строго внутрь."""

    before, here, after = ring[index - 1], ring[index], ring[(index + 1) % len(ring)]
    toward_after = orientation(here, after, target, budget)
    from_before = orientation(before, here, target, budget)
    turn = orientation(before, here, after, budget)
    if turn > 0:
        return from_before > 0 and toward_after > 0
    if turn < 0:
        return from_before > 0 or toward_after > 0
    return from_before > 0


def _valid_diagonal(ring, first: int, second: int, budget) -> bool:
    """Диагональ `ring[first] - ring[second]` лежит внутри кольца: нет пересечений, нет вершин на ней, оба конца входят внутрь."""

    size = len(ring)
    start, end = ring[first], ring[second]
    for index in range(size):
        if index in (first, second):
            continue
        if _on_open_segment(start, end, ring[index], budget):
            return False
        if (index + 1) % size in (first, second):
            continue
        if segments_cross(start, end, ring[index], ring[(index + 1) % size], budget):
            return False
    return _enters_interior(ring, first, end, budget) and _enters_interior(ring, second, start, budget)


def plan_partition(points, budget, admissible):
    """`PartitionPlanV1` с наименьшим числом допустимых частей либо `PartitionRefusalV1`.

    `points[i]` — точка карты вершины простого кольца; `admissible(кольцо индексов против часовой)` — допустима ли
    часть (вызывающий отвечает прежним законом куска, ответ запоминается здесь). Кусок целиком допустим — одна часть.
    """

    count = len(points)
    if count > MAX_VERTICES:
        return PartitionRefusalV1(REASON_TOO_MANY_VERTICES, f"{count} vertices")
    ring = counter_clockwise_ring(tuple(points), budget)
    if ring is None:
        return PartitionRefusalV1(REASON_NOT_A_SUBDIVISION, "zero area")
    verdicts: dict = {}

    def allowed(verts) -> bool:
        found = verdicts.get(verts)
        if found is None:
            found = verdicts[verts] = len(verts) == 3 or bool(admissible(verts))
        return found

    @lru_cache(maxsize=None)
    def solve(verts):
        """`(число частей, части)` наименьшего разбиения подкольца `verts` либо `(None, ())`."""

        if allowed(verts):
            return 1, (verts,)
        subring = tuple(points[index] for index in verts)
        size = len(verts)
        best = (None, ())
        for first in range(size):
            for second in range(first + 2, size):
                if first == 0 and second == size - 1:
                    continue
                if not _valid_diagonal(subring, first, second, budget):
                    continue
                left_count, left = solve(verts[first : second + 1])
                if left_count is None:
                    continue
                right_count, right = solve(verts[second:] + verts[: first + 1])
                if right_count is None:
                    continue
                if best[0] is None or left_count + right_count < best[0]:
                    best = (left_count + right_count, left + right)
                    if best[0] == 2:
                        return best
        return best

    total, pieces = solve(tuple(ring))
    if total is None:
        return PartitionRefusalV1(REASON_NO_PARTITION, f"{count} vertices")
    return PartitionPlanV1(pieces, tuple(ring))


def verify_partition(points, plan: PartitionPlanV1, budget):
    """Доказательство разбиения: `None` либо `PartitionRefusalV1`. Части простые, одной ориентации, полурёбра сокращаются."""

    ring = plan.order
    used: Counter = Counter()
    for piece in plan.pieces:
        ring_points = tuple(points[index] for index in piece)
        if len(set(piece)) != len(piece) or shoelace_sign(ring_points, budget) <= 0:
            return PartitionRefusalV1(REASON_NOT_A_SUBDIVISION, "a part is degenerate or reversed")
        if not contour_is_simple(ring_points, budget):
            return PartitionRefusalV1(REASON_NOT_A_SUBDIVISION, "a part is not a simple polygon")
        used.update(zip(piece, piece[1:] + piece[:1]))
    boundary = set(zip(ring, ring[1:] + ring[:1]))
    for (first, second), times in used.items():
        if times != 1:
            return PartitionRefusalV1(REASON_NOT_A_SUBDIVISION, "an edge is used twice in one direction")
        paired = (second, first) in used
        if paired == ((first, second) in boundary):
            return PartitionRefusalV1(REASON_NOT_A_SUBDIVISION, "a part edge is neither a cut nor a contour edge")
    if any(edge not in used for edge in boundary):
        return PartitionRefusalV1(REASON_NOT_A_SUBDIVISION, "a contour edge is on no part")
    return None


def partition_counters(tally) -> tuple:
    """Числа закона из `tally`, только ненулевые: домен, где закон не сработал, не получает новых строк."""

    names = (
        PIECES_PARTITIONED,
        FACES_EMITTED,
        DIAGONALS,
        PIECES_REFUSED,
        *(REFUSED_PREFIX + reason for reason in REASONS),
    )
    return tuple((name, tally[name]) for name in names if tally[name])
