"""Закон `CONVEX_PARTITION_BY_DIAGONALS_V1`: кусок, у которого нет аффинной и билинейной грани целиком, режется на допустимые грани диагоналями между его же вершинами вместо ушей.

ЗАЧЕМ. Кусок полосы потока с неаффинной UV и невыпуклым контуром (`_bilinear_ring` требует строгой выпуклости) шёл в
уши (`UV_NOT_AFFINE`): диагональ на каждую вершину сверх третьей, тонкие треугольники, а резка по граням источника
множит каждое ухо (`sagging_wall`, alpha 0.987: 57 граней-ушей дают 224 грани меша на 29 граней источника). Допустимая
грань, однако, не обязана быть треугольником: строго выпуклый многоугольник потока с билинейной UV уже допускается
(`QUAD_UV_BILINEAR_V1`, `_bilinear_ring`), аффинный — тем более, а ушам эта допустимость не нужна.

ПОЧЕМУ ДИАГОНАЛИ, А НЕ РАЗРЕЗЫ ПО СТАНЦИЯМ. Разрез по `s = const` из вершины фронта кончается на ребре источника: новая
вершина на цепи, общей с соседним доменом, — T-стык шва (хост сваривает только `src:`; `clip.seam_edges`,
`ADAPTER_SEAM_T_JUNCTIONS`, `clip_gate`: шовные цепи побитово те же). У кусков потока источник внизу, а всё остальное —
общие рёбра соседей, поэтому ни один такой разрез не законен: ядро не ставит новых вершин на цепь источника или стены
(политику шва не открывает). Диагональ между двумя СУЩЕСТВУЮЩИМИ вершинами не рождает вершин и шва не касается.

ЗАКОН. Грань куска — кольцо его вершин (против часовой на карте). Часть допустима, когда она (`admissible`) треугольник,
либо строго выпуклый многоугольник потока (билинейная UV, UV не нужна), либо имеет аффинную UV. Закон ищет разбиение
кольца непересекающимися внутренними диагоналями на допустимые части.

* Кусок до `EXACT_VERTICES` вершин — ТОЧНОЕ наименьшее число частей: запоминающее разбиение по подкольцам (подкольцо —
  циклический отрезок исходного порядка вершин; диагонали по возрастанию пары индексов, первая из равных по числу частей
  побеждает). Диагональ допустима, когда ни одно ребро подкольца её не пересекает трансверсально, ни одна другая вершина
  не лежит на ней и она входит в угол каждого конца внутрь (знак `orientation` каждого шага точный).
* Длиннее — жадное слияние Hertel - Mehlhorn без потолка по вершинам: старт — те же уши (`triangulate_exact`), диагонали
  в порядке номеров концов по возрастанию, диагональ снимается, когда объединение двух её частей допустимо, и обход
  повторяется, пока снимать нечего (`O(n)` проверок допустимости на проход). Ни одна диагональ результата уже не
  снимается, а число частей не больше четырёх оптимальных (граница Hertel - Mehlhorn для любой триангуляции).

Почему точный поиск остался для малых кусков. На поле жадный даёт столько же частей (142 против 141 на 70 кусках), но ДРУГИЕ
диагонали из равных по числу: у типичного пятиугольника ленты `A B C D E` с невыпуклой `D` точный берёт `A - D`, жадный
`B - D`, и после резки по граням источника это 165 против 153 граней меша на `sagging_wall` (+7.8 %, предел 5 % нарушен).
Точный поиск на малых кусках стоит до 0.03 с, а жадный нужен там, где точный дорог: `n^4` знаков на 12 вершинах это до
0.9 с, на 14 — до 7 с.

ДОКАЗАТЕЛЬСТВО (точно, допусков нет). Каждая часть — простой многоугольник положительной ориентации; полурёбра частей
сокращаются парами (диагонали по одной в каждую сторону), а остаток — ровно контур куска. Тогда части покрывают кусок
один раз без щелей и наложений. Дополнительно (вызывающий): ориентация образа каждой части в UV та же, что у образа
куска (складка UV не вызывается разбиением), сумма площадей частей равна площади куска точно. Отказ любого пункта —
кусок остаётся на ушах (прежний путь, побитово) под ИМЕНЕМ причины. Разбиение не короче ушей (`n - 2` частей: у куска
вне потока допустимы одни треугольники) выигрыша не даёт — `NO_GAIN_OVER_EARS`, уши прежние побитово, а не другая
триангуляция того же числа граней. Куска, оставленного на ушах из-за длины, нет: потолка по вершинам нет.

ИЗМЕРЕНО НА ПОЛЕ (headless, кнопка, `E:/testScene.blend`, Fan Density 2; грани / рёбра / грани с углом меньше пяти градусов в
меше; «до» — закон выключен на том же дереве). См. DECISIONS 2026-10-05: `sagging_wall` alpha 0.987 (Max stretch 42 %),
`rounded_wall_noise_top` alpha 0.5 (42 %), `building` d2, меш `2`; `ADAPTER_SEAM_T_JUNCTIONS` = 0 везде (вершин закон не рождает).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from functools import lru_cache

from ..wavefront.faces import orientation, segments_cross, shoelace_sign
from .tessellate import contour_is_simple, counter_clockwise_ring, triangulate_exact

#: Кусок до стольких вершин ищется точно (наименьшее число частей), длиннее — жадным слиянием: граница метода, не отказ.
EXACT_VERTICES = 8

#: Имена чисел закона (они же ключи счётчиков материализатора). Нулевые числа в счётчики не пишутся.
PIECES_PARTITIONED = "MATERIALIZE_CONVEX_PARTITION_PIECES"
PIECES_EXACT = "MATERIALIZE_CONVEX_PARTITION_PIECES_EXACT"
PIECES_GREEDY = "MATERIALIZE_CONVEX_PARTITION_PIECES_GREEDY"
FACES_EMITTED = "MATERIALIZE_CONVEX_PARTITION_FACES_EMITTED"
DIAGONALS = "MATERIALIZE_CONVEX_PARTITION_DIAGONALS"
PIECES_REFUSED = "MATERIALIZE_CONVEX_PARTITION_PIECES_REFUSED"
REFUSED_PREFIX = "MATERIALIZE_CONVEX_PARTITION_REFUSED_"

REASON_NO_TRIANGULATION = "NO_TRIANGULATION"
REASON_NO_PARTITION = "NO_ADMISSIBLE_PARTITION"
REASON_NO_GAIN = "NO_GAIN_OVER_EARS"
REASON_NOT_A_SUBDIVISION = "NOT_A_SUBDIVISION"
REASON_UV_FOLD = "UV_FOLD"
REASON_AREA = "AREA_DOES_NOT_CLOSE"
REASONS = (
    REASON_NO_TRIANGULATION,
    REASON_NO_PARTITION,
    REASON_NO_GAIN,
    REASON_NOT_A_SUBDIVISION,
    REASON_UV_FOLD,
    REASON_AREA,
)


@dataclass(frozen=True, slots=True)
class PartitionPlanV1:
    """Части куска: кольца индексов входного кольца против часовой на карте; диагоналей на единицу меньше частей.

    `exact` — части найдены точным поиском (кусок до `EXACT_VERTICES` вершин), иначе жадным слиянием.
    """

    pieces: tuple
    order: tuple
    exact: bool = True


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


def _plan_exact(points, ring, budget, allowed):
    """Части с наименьшим числом допустимых частей (запоминающее разбиение по подкольцам) либо `None`."""

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
    return None if total is None else pieces


def _merged_ring(first, second, start, end):
    """Кольцо объединения двух частей по общей диагонали: у `first` ребро `start -> end`, у `second` — `end -> start`."""

    at = first.index(start)
    rotated_first = first[at:] + first[:at]
    at = second.index(end)
    rotated_second = second[at:] + second[:at]
    return rotated_first[1:] + (start,) + rotated_second[2:]


def _plan_greedy(triangles, allowed):
    """Части жадным слиянием Hertel - Mehlhorn от триангуляции `triangles` (индексы входного кольца против часовой)."""

    faces = dict(enumerate(triangles))
    owner = {edge: number for number, face in faces.items() for edge in zip(face, face[1:] + face[:1])}
    diagonals = sorted({(min(a, b), max(a, b)) for a, b in owner if (b, a) in owner})
    created = len(faces)
    merged = True
    while merged:
        merged = False
        for low, high in diagonals:
            if (low, high) not in owner or (high, low) not in owner:
                continue
            first, second = owner[(low, high)], owner[(high, low)]
            union = _merged_ring(faces[first], faces[second], low, high)
            if not allowed(union):
                continue
            for number in (first, second):
                face = faces.pop(number)
                for edge in zip(face, face[1:] + face[:1]):
                    del owner[edge]
            faces[created] = union
            owner.update((edge, created) for edge in zip(union, union[1:] + union[:1]))
            created += 1
            merged = True
    return tuple(sorted(face[face.index(min(face)) :] + face[: face.index(min(face))] for face in faces.values()))


def plan_partition(points, budget, admissible):
    """`PartitionPlanV1` либо `PartitionRefusalV1`: часть куска допустима, когда её признал `admissible(кольцо индексов)`.

    Кусок до `EXACT_VERTICES` вершин — наименьшее число частей, длиннее — жадное слияние (см. докстринг модуля). Кусок
    целиком допустим — одна часть. Ответ `admissible` запоминается здесь.
    """

    ring = counter_clockwise_ring(tuple(points), budget)
    triangles = None if ring is None else triangulate_exact(points, budget)
    if triangles is None:
        return PartitionRefusalV1(REASON_NO_TRIANGULATION, f"{len(points)} vertices")
    verdicts: dict = {}

    def allowed(verts) -> bool:
        found = verdicts.get(verts)
        if found is None:
            found = verdicts[verts] = len(verts) == 3 or bool(admissible(verts))
        return found

    exact = len(points) <= EXACT_VERTICES
    if allowed(tuple(ring)):
        return PartitionPlanV1((tuple(ring),), tuple(ring), exact)
    pieces = _plan_exact(points, ring, budget, allowed) if exact else _plan_greedy(triangles, allowed)
    if pieces is None:
        return PartitionRefusalV1(REASON_NO_PARTITION, f"{len(points)} vertices")
    return PartitionPlanV1(pieces, tuple(ring), exact)


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
        PIECES_EXACT,
        PIECES_GREEDY,
        FACES_EMITTED,
        DIAGONALS,
        PIECES_REFUSED,
        *(REFUSED_PREFIX + reason for reason in REASONS),
    )
    return tuple((name, tally[name]) for name in names if tally[name])
