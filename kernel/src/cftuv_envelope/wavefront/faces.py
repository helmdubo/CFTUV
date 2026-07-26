"""Грани скелета из узлов очереди: owner каждого куска и его ТОЧНАЯ площадь.

Зачем это здесь. `skeleton.py` выдаёт события — момент, место и участников. Это
достаточно, чтобы доказать порядок и одновременность, но недостаточно, чтобы
сравниться с попарным кроем: тот проверяет себя ПЛОЩАДЬЮ
(`interactions/policy_b.py:976`, `INTERACTION_POLICY_B_PARTITION_UNPROVEN` —
«Policy B clipping did not reproduce the exact RawCoverage set»). Чтобы задать
очереди тот же вопрос, из узлов надо собрать грани и сложить их площади.

Правило сборки — не эвристика, а свойство straight skeleton: **грань опорного
ребра МОНОТОННА по направлению этого ребра.** Отсюда грань ребра `e` есть

    e.start, e.end, затем все узлы, среди участников которых есть `e`,
    упорядоченные по УБЫВАНИЮ проекции на направление `e`.

Порядок берётся точным знаком `SqrtSumV1`, не сравнением чисел: координаты узлов
иррациональны, и «больше» здесь решается тем же предикатом, что и всё остальное
в модуле. Порогов нет ни одного, площадь считается по формуле трапеций над
`SqrtSumV1` (у неё есть произведение), и сравнение с целой площадью
многоугольника — это `is_zero` разности, то есть чисто рациональная проверка.

ОБЪЯВЛЕННЫЕ ГРАНИЦЫ (границ две, и обе проверяются тестом на собственном
результате этой функции — `test_wavefront_faces.py`):

1. **Сумма площадей граней РАВНА площади многоугольника.** Точно, не «почти».
   Это и есть ровно то утверждение, которое попарный крой о себе не доказывает.
2. **Площадь каждой грани строго положительна.** Грань — двумерный кусок, а не
   вырожденная щепка и не кусок с вывернутой ориентацией.

Чего здесь нет и почему это названо, а не замолчано: ключ участника —
геометрический ключ НЕСУЩЕЙ ПРЯМОЙ `(a, b, c, q)`, а не тождество ребра. Два
различных ребра входа могут лежать на одной прямой (в корпусе это `holes_2`:
рёбра соседних дыр коллинеарны), и тогда узлы двух разных граней попадают в один
список, монотонность ломается, и площадь не сходится: 1584 вместо 1344. Это
НЕ починено подгонкой — это отдельный исход
`SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES`, и он возвращается до всякого счёта.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from functools import cmp_to_key

from .event_time import EventPointV1, SupportLineV1
from .polygon import PolygonV1, signed_double_area
from .skeleton import SkeletonNodeV1, SkeletonOutcome, SkeletonV1
from .sqrt_sum import SqrtSumV1


LineKey = tuple[int, int, int, int]


class FaceOutcome(str, Enum):
    """Чем кончилась сборка граней. Тихого возврата нет ни одного."""

    EXACT = "EXACT"
    # Скелет не доказан — граней у него нет, и придумывать их нельзя.
    SKELETON_IS_NOT_EXACT = "SKELETON_IS_NOT_EXACT"
    # Два ребра входа делят одну несущую прямую: ключ участника не различает
    # их грани. См. докстроку: `holes_2` в корпусе.
    SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES = (
        "SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES"
    )
    # У ребра нет ни одного узла. Грань не замкнуть двумя точками.
    FACE_HAS_NO_SKELETON_NODE = "FACE_HAS_NO_SKELETON_NODE"


@dataclass(frozen=True, slots=True)
class FaceV1:
    """Грань скелета: её owner, её контур и её точная удвоенная площадь.

    `owner` — геометрический ключ опорного ребра. Это и есть тот самый owner
    покрытия, который в попарном крое приходится вычислять отдельно: здесь он
    выпадает из структуры, потому что грань определена ребром.
    """

    owner: LineKey
    source_start: tuple[int, int]
    source_end: tuple[int, int]
    points: tuple[tuple[SqrtSumV1, SqrtSumV1], ...]
    doubled_area: SqrtSumV1

    @property
    def node_count(self) -> int:
        return len(self.points) - 2


@dataclass(frozen=True, slots=True)
class FacePartitionV1:
    """Разбиение многоугольника гранями скелета и обе объявленные границы."""

    outcome: FaceOutcome
    faces: tuple[FaceV1, ...]
    doubled_area: SqrtSumV1
    polygon_doubled_area: int
    detail: str = ""

    @property
    def area_reproduces_polygon(self) -> bool:
        """Граница 1: сумма площадей граней равна площади многоугольника.

        Проверка ТОЧНАЯ: разность двух величин `SqrtSumV1` равна нулю тогда и
        только тогда, когда пуст её канонический набор коэффициентов.
        """

        difference = self.doubled_area - SqrtSumV1.rational(
            self.polygon_doubled_area
        )
        return difference.is_zero

    @property
    def every_face_is_positive(self) -> bool:
        """Граница 2: площадь каждой грани строго положительна."""

        return all(face.doubled_area.sign() > 0 for face in self.faces)

    @property
    def area_defect(self) -> SqrtSumV1:
        """Расхождение площадей. Ноль означает совпадение, и это доказано."""

        return self.doubled_area - SqrtSumV1.rational(self.polygon_doubled_area)


def polygon_edges(
    polygon: PolygonV1,
) -> tuple[tuple[tuple[int, int], tuple[int, int], SupportLineV1], ...]:
    """Рёбра всех петель области в том же порядке, в каком их видит скелет."""

    edges = []
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        for index in range(size):
            start = points[index]
            end = points[(index + 1) % size]
            edges.append((start, end, SupportLineV1.through(start, end)))
    return tuple(edges)


def line_key(line: SupportLineV1) -> LineKey:
    return (line.a, line.b, line.c, line.q)


def _projection(point: EventPointV1, dx: int, dy: int) -> SqrtSumV1:
    """Проекция точки на направление `(dx, dy)`. Без нормировки: знак важен."""

    return point.x.scaled(dx) + point.y.scaled(dy)


def doubled_shoelace(
    points: tuple[tuple[SqrtSumV1, SqrtSumV1], ...],
) -> SqrtSumV1:
    """Удвоенная ориентированная площадь по точкам-суммам корней.

    Произведение `SqrtSumV1` замкнуто (`sqrt(a)*sqrt(b) = g*sqrt(ab/g^2)`),
    поэтому площадь остаётся канонической величиной и сравнима побитово.
    """

    total = SqrtSumV1.zero()
    size = len(points)
    for index in range(size):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % size]
        total = total + (x0 * y1) - (x1 * y0)
    return total


def build_faces(polygon: PolygonV1, skeleton: SkeletonV1) -> FacePartitionV1:
    """Грани скелета по правилу монотонности, с обеими границами в результате."""

    empty = FacePartitionV1(
        FaceOutcome.EXACT,
        (),
        SqrtSumV1.zero(),
        sum(signed_double_area(loop.points) for loop in polygon.loops),
    )
    if skeleton.outcome is not SkeletonOutcome.EXACT:
        return FacePartitionV1(
            FaceOutcome.SKELETON_IS_NOT_EXACT,
            (),
            SqrtSumV1.zero(),
            empty.polygon_doubled_area,
            skeleton.outcome.value,
        )

    edges = polygon_edges(polygon)
    keys = [line_key(line) for _, _, line in edges]
    if len(set(keys)) != len(keys):
        shared = sorted({key for key in keys if keys.count(key) > 1})
        return FacePartitionV1(
            FaceOutcome.SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES,
            (),
            SqrtSumV1.zero(),
            empty.polygon_doubled_area,
            f"{len(shared)} прямых на нескольких рёбрах: {shared[:3]}",
        )

    nodes_by_key: dict[LineKey, list[SkeletonNodeV1]] = {}
    for node in skeleton.nodes:
        for key in node.participants:
            nodes_by_key.setdefault(key, []).append(node)

    faces: list[FaceV1] = []
    total = SqrtSumV1.zero()
    for start, end, line in edges:
        key = line_key(line)
        candidates = nodes_by_key.get(key, ())
        if not candidates:
            return FacePartitionV1(
                FaceOutcome.FACE_HAS_NO_SKELETON_NODE,
                (),
                SqrtSumV1.zero(),
                empty.polygon_doubled_area,
                f"ребро {start} -> {end} без узлов",
            )
        dx, dy = end[0] - start[0], end[1] - start[1]

        def compare(left: SkeletonNodeV1, right: SkeletonNodeV1) -> int:
            """По убыванию проекции на ребро; при равенстве — на нормаль.

            Второй ключ не украшение: два узла грани могут лежать на одном
            перпендикуляре к ребру, и без него порядок зависел бы от того, в
            каком порядке события легли в список.
            """

            along = _projection(right.point, dx, dy) - _projection(
                left.point, dx, dy
            )
            sign = along.sign()
            if sign:
                return sign
            across = _projection(right.point, -dy, dx) - _projection(
                left.point, -dy, dx
            )
            return across.sign()

        ordered = sorted(candidates, key=cmp_to_key(compare))
        points = (
            (SqrtSumV1.rational(start[0]), SqrtSumV1.rational(start[1])),
            (SqrtSumV1.rational(end[0]), SqrtSumV1.rational(end[1])),
        ) + tuple((node.point.x, node.point.y) for node in ordered)
        doubled = doubled_shoelace(points)
        faces.append(FaceV1(key, start, end, points, doubled))
        total = total + doubled

    return FacePartitionV1(
        FaceOutcome.EXACT,
        tuple(faces),
        total,
        empty.polygon_doubled_area,
    )
