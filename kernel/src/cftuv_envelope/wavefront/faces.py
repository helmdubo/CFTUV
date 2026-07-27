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

ОБЪЯВЛЕННЫЕ ГРАНИЦЫ (границ две, и обе проверяются САМОЙ `build_faces` перед
тем, как она вернёт `EXACT`, — у каждой свой член `FaceOutcome` и число потери
в `detail`; тест `test_wavefront_faces.py` проверяет их же на собственном
результате функции):

1. **Сумма площадей граней РАВНА площади многоугольника.** Точно, не «почти».
   Это и есть ровно то утверждение, которое попарный крой о себе не доказывает.
2. **Площадь каждой грани строго положительна.** Грань — двумерный кусок, а не
   вырожденная щепка и не кусок с вывернутой ориентацией.

ТОЖДЕСТВО УЧАСТНИКА — ВХОЖДЕНИЕ РЕБРА `(x0, y0, x1, y1)`, А НЕ НЕСУЩАЯ ПРЯМАЯ,
и это измеренный ответ, а не вкус. Ключом был `(a, b, c, q)`, то есть прямая с
ненормированной нормалью; он совпадал у двух коллинеарных сонаправленных рёбер
ОДНОЙ длины, узлы двух РАЗНЫХ граней сливались в один список, монотонность
ломалась, и площадь не сходилась: 1584 вместо 1344 на `holes_2`. Отказ
`SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES` ловил это громко, но ловил уже, чем
обещал: у креста рёбра тоже коллинеарны, а ключи различны, потому что `q`
вносит ДЛИНУ.

Довод в пользу прямой был назван и он не отменён: биссектриса straight skeleton
равноудалена от несущих ПРЯМЫХ, а не от отрезков, и этим straight skeleton
отличается от медиальной оси. Но этот довод — про ГЕОМЕТРИЮ, и геометрия
по-прежнему идёт через `SupportLineV1`: время события, положение вершины,
полуплоскость усечения. Ключ же решает другой вопрос — ЧЬЯ это грань, — и у
двух рёбер на одной прямой граней две. Одна сущность исполняла две роли, и
починка в том, чтобы их разделить, а не выбрать между ними.

Что проверено этим измерением, а не принято на слово: сумма площадей граней
`holes_2` сходится точно (1344 из 1344), все двенадцать граней положительны,
владельцы различны, а покрытие совпадает с НЕЗАВИСИМЫМ митрованным эталоном —
188 / 384 / 588 при alpha = 1, 2, 3.

Исход `TWO_EDGES_SHARE_ONE_SPAN` остался на месте: два вхождения с одинаковыми
концами означали бы, что вход задан дважды, и грань у них была бы одна на двоих.
На корпусе он не срабатывает ни разу, но убирать проверку нельзя — `LoopV1`
непересекаемости петель не требует.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from functools import cmp_to_key

from .event_time import SupportLineV1
from .polygon import PolygonV1, signed_double_area
from .skeleton import SkeletonNodeV1, SkeletonOutcome, SkeletonV1
from .sqrt_sum import SqrtSumV1


#: Ключ УЧАСТНИКА события и владельца грани: вхождение ребра `(x0, y0, x1, y1)`.
#: Имя `LineKey` не оставлено даже синонимом: тип у него тот же, и молчаливый
#: синоним означал бы, что смысл ключа выясняется по значению, а не по имени.
EdgeKey = tuple[int, int, int, int]


class FaceOutcome(str, Enum):
    """Чем кончилась сборка граней. Тихого возврата нет ни одного."""

    EXACT = "EXACT"
    # Скелет не доказан — граней у него нет, и придумывать их нельзя.
    SKELETON_IS_NOT_EXACT = "SKELETON_IS_NOT_EXACT"
    # Два вхождения с одинаковыми концами: вход задал одно ребро дважды, и грань
    # у них была бы одна на двоих. Заменил `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES`,
    # который отказывал на коллинеарных рёбрах РАЗНЫХ граней, — это оказалось
    # свойством ключа, а не входа, и вылечено сменой ключа.
    TWO_EDGES_SHARE_ONE_SPAN = "TWO_EDGES_SHARE_ONE_SPAN"
    # У ребра нет ни одного узла. Грань не замкнуть двумя точками.
    FACE_HAS_NO_SKELETON_NODE = "FACE_HAS_NO_SKELETON_NODE"
    # Граница 1 нарушена: сумма площадей граней НЕ равна площади многоугольника.
    # Член отдельный, а не общий с границей 2, потому что это другой дефект:
    # здесь разбиение потеряло (или удвоило) кусок ПЛОЩАДИ, и величина потери
    # лежит числом в `detail`. Слить их в один исход означало бы в диагностике
    # не отличить «клин потерян» от «грань вывернута».
    FACE_AREA_DOES_NOT_REPRODUCE_POLYGON = "FACE_AREA_DOES_NOT_REPRODUCE_POLYGON"
    # Граница 2 нарушена: у какой-то грани площадь не строго положительна.
    # Это дефект СБОРКИ одной грани (порядок обхода вывернул её либо схлопнул),
    # а не арифметики суммы, поэтому он назван своим именем.
    FACE_IS_NOT_POSITIVE = "FACE_IS_NOT_POSITIVE"


@dataclass(frozen=True, slots=True)
class FaceV1:
    """Грань скелета: её owner, её контур и её точная удвоенная площадь.

    `owner` — ключ ВХОЖДЕНИЯ опорного ребра. Это и есть тот самый owner
    покрытия, который в попарном крое приходится вычислять отдельно: здесь он
    выпадает из структуры, потому что грань определена ребром.

    Несущая прямая по owner'у больше не восстанавливается — она берётся из
    `source_start`/`source_end` (`SupportLineV1.through`). Восстановление из
    ключа было возможно, пока ключ был прямой, и ровно этим совмещением ролей
    два коллинеарных ребра и сливались в одну грань.
    """

    owner: EdgeKey
    source_start: tuple[int, int]
    source_end: tuple[int, int]
    points: tuple[tuple[SqrtSumV1, SqrtSumV1], ...]
    doubled_area: SqrtSumV1
    #: `q` опорного ребра. Хранится, а не пересчитывается по концам: у
    #: взвешенного ребра из концов его не вывести, а усечение по времени
    #: (`coverage.py`) спрашивает именно скорость.
    speed_squared: int = 0

    @property
    def node_count(self) -> int:
        return len(self.points) - 2


@dataclass(frozen=True, slots=True)
class FacePartitionV1:
    """Разбиение многоугольника гранями скелета и обе объявленные границы."""

    # При отказе по границе 1 или 2 грани НЕ обнуляются, в отличие от отказов,
    # случившихся до счёта: там граней просто нет, а тут они посчитаны, и без
    # них дефект нечем измерить. Смотреть на них можно только после проверки
    # `outcome`, и потребитель это делает (`coverage.py:160`).
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


def as_number(value: SqrtSumV1) -> str:
    """Величина ЧИСЛОМ для `detail`, а не словом «нарушено».

    Отказ, у которого в причине стоит слово, нельзя ни сравнить с прошлым
    прогоном, ни отличить потерю в 1/2 от потери в 18. Рациональная величина
    печатается дробью, иррациональная — своим каноническим набором
    коэффициентов: он тоже число, просто записанное в базисе корней.
    """

    rational = value.as_rational()
    if rational is not None:
        return str(rational)
    return " + ".join(
        f"{coefficient}*sqrt({radicand})" for radicand, coefficient in value.terms
    )


def polygon_edges(
    polygon: PolygonV1,
) -> tuple[tuple[tuple[int, int], tuple[int, int], SupportLineV1], ...]:
    """Рёбра всех петель области в том же порядке, в каком их видит скелет.

    Скорость берётся из `polygon.edges()`, а не восстанавливается через
    `through`: у стены `q = 0`, и `through` вернул бы ей `|d|^2`, то есть
    сборщик увидел бы источник там, где вход задал стену.
    """

    return tuple(
        (start, end, SupportLineV1.with_speed(start, end, speed_squared))
        for start, end, speed_squared in polygon.edges()
    )


def edge_key(start: tuple[int, int], end: tuple[int, int]) -> EdgeKey:
    """Ключ вхождения ребра. Тот же, что `_Edge.span` в `skeleton.py`.

    Функция одна на модуль и одна на репозиторий именно затем, чтобы сборщик и
    очередь не могли разойтись в тождестве участника правкой одного из двух.
    """

    return (start[0], start[1], end[0], end[1])


def line_key(line: SupportLineV1) -> tuple[int, int, int, int]:
    """Ключ несущей ПРЯМОЙ. Участником он больше не служит.

    Оставлен для тех, кто спрашивает про прямую именно как про прямую: например
    чтобы посчитать, сколько рёбер её делят. Смешивать его с `edge_key` нельзя,
    и типы у них поэтому не одноимённые.
    """

    return (line.a, line.b, line.c, line.q)


Point = tuple[SqrtSumV1, SqrtSumV1]


def projection(point: Point, dx: int, dy: int) -> SqrtSumV1:
    """Проекция точки на направление `(dx, dy)`. Без нормировки: знак важен."""

    return point[0].scaled(dx) + point[1].scaled(dy)


def order_along_edge(
    points: tuple[Point, ...], dx: int, dy: int
) -> tuple[Point, ...]:
    """Порядок узлов грани: по УБЫВАНИЮ проекции на ребро; ничьи — по ГЛУБИНЕ.

    Вынесено из `build_faces` отдельной функцией не для красоты: стенд, который
    проверяет ГИПОТЕЗУ о причине дефекта, обязан сортировать тем же самым
    правилом, а не своей копией. Копия доказывала бы про копию.

    Ничья по проекции — не редкость и не шум: она означает, что кусок дальней
    границы грани ПЕРПЕНДИКУЛЯРЕН опорному ребру. Такой кусок рождается там, где
    вершина фронта с развёрнутым углом скользит по своей прямой, — то есть ровно
    в вырожденной точке, ради которой сделан срез.

    Разрешить ничью одним знаком нельзя, и это ИЗМЕРЕНО, а не выведено: на
    кресте один и тот же перпендикулярный кусок `(8,6)-(7,6)` входит в грань
    ребра 1 и в грань ребра 5, и обходится он в них В РАЗНОМ ПОРЯДКЕ. Любое
    фиксированное «сначала глубже» или «сначала мельче» ошибается ровно на
    половине граней: 18 вместо 23 и 58 вместо 71.

    Правило, которое верно для обеих: **глубина вдоль обхода УНИМОДАЛЬНА.**
    Дальняя граница грани — это следы двух концов движущегося отрезка, у каждого
    глубина равна времени и потому монотонна. Обход идёт от `end` вверх по следу
    одного конца до самой дальней точки и обратно вниз по следу другого. Значит
    группы до самой глубокой обходятся ПО ВОЗРАСТАНИЮ глубины, после неё — по
    убыванию. Когда ничьих нет, каждая группа состоит из одного узла и правило
    возвращает прежний порядок в точности.

    ГРАНИЦА правила названа, а не замолчана: оно опирается на то, что самая
    глубокая точка обхода лежит в КОНЦЕ своей группы. Если вся дальняя граница
    грани перпендикулярна ребру (группа одна, и самая глубокая точка стоит в её
    начале), правило её перевернёт, и грань выйдет вырожденной либо вывернутой.
    Такое встречается на кресте при `|wide - tall| >= 2 * arm`, и там оно ловится
    громко — `FACE_IS_NOT_POSITIVE`, а не тихой потерей площади.
    """

    def depth(point: Point) -> SqrtSumV1:
        """Удаление от несущей прямой ребра. Оно же — время прихода фронта."""

        return projection(point, -dy, dx)

    buckets: dict[tuple, list[Point]] = {}
    for point in points:
        buckets.setdefault(projection(point, dx, dy).terms, []).append(point)
    lanes = sorted(
        buckets,
        key=cmp_to_key(
            lambda left, right: (SqrtSumV1(right) - SqrtSumV1(left)).sign()
        ),
    )

    deepest = 0
    best: SqrtSumV1 | None = None
    for index, lane in enumerate(lanes):
        for point in buckets[lane]:
            if best is None or (depth(point) - best).sign() > 0:
                deepest, best = index, depth(point)

    ordered: list[Point] = []
    for index, lane in enumerate(lanes):
        rising = index <= deepest
        ordered.extend(
            sorted(
                buckets[lane],
                key=cmp_to_key(
                    lambda left, right, rising=rising: (
                        (depth(left) - depth(right)).sign()
                        if rising
                        else (depth(right) - depth(left)).sign()
                    )
                ),
            )
        )
    return tuple(ordered)


def face_contour(
    start: tuple[int, int], end: tuple[int, int], nodes: tuple[Point, ...]
) -> tuple[Point, ...]:
    """Контур грани: опорное ребро, затем его узлы в порядке монотонности."""

    dx, dy = end[0] - start[0], end[1] - start[1]
    return (
        (SqrtSumV1.rational(start[0]), SqrtSumV1.rational(start[1])),
        (SqrtSumV1.rational(end[0]), SqrtSumV1.rational(end[1])),
    ) + order_along_edge(nodes, dx, dy)


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
    keys = [edge_key(start, end) for start, end, _ in edges]
    if len(set(keys)) != len(keys):
        shared = sorted({key for key in keys if keys.count(key) > 1})
        return FacePartitionV1(
            FaceOutcome.TWO_EDGES_SHARE_ONE_SPAN,
            (),
            SqrtSumV1.zero(),
            empty.polygon_doubled_area,
            f"{len(shared)} вхождений на нескольких рёбрах: {shared[:3]}",
        )

    nodes_by_key: dict[EdgeKey, list[SkeletonNodeV1]] = {}
    for node in skeleton.nodes:
        for key in node.participants:
            nodes_by_key.setdefault(key, []).append(node)

    faces: list[FaceV1] = []
    total = SqrtSumV1.zero()
    for start, end, line in edges:
        if line.is_stationary:
            # Стена не заметает НИЧЕГО: её отрезок фронта остаётся на своей
            # прямой, только укорачивается. Грань нулевой площади — не грань, и
            # граница «каждая грань строго положительна» отвергла бы её. Узлы
            # стены при этом никуда не деваются: они входят в грани соседей как
            # участники, и сумма площадей по-прежнему обязана дать всю область.
            continue
        key = edge_key(start, end)
        candidates = nodes_by_key.get(key, ())
        if not candidates:
            return FacePartitionV1(
                FaceOutcome.FACE_HAS_NO_SKELETON_NODE,
                (),
                SqrtSumV1.zero(),
                empty.polygon_doubled_area,
                f"ребро {start} -> {end} без узлов",
            )
        points = face_contour(
            start, end, tuple((n.point.x, n.point.y) for n in candidates)
        )
        doubled = doubled_shoelace(points)
        faces.append(FaceV1(key, start, end, points, doubled, line.q))
        total = total + doubled

    assembled = FacePartitionV1(
        FaceOutcome.EXACT,
        tuple(faces),
        total,
        empty.polygon_doubled_area,
    )

    # Обе объявленные границы проверяются ЗДЕСЬ, до возврата `EXACT`, а не
    # только тестом. Причина измерена, а не гигиеническая: на кресте сборка
    # теряла клин площади, отдавала `EXACT`, и потеря протекала до самого
    # ответа — граница 2 при этом ДЕРЖАЛАСЬ (все грани положительны), а
    # `coverage_at.does_not_exceed_the_polygon` проходила, потому что потеря
    # делает покрытие МЕНЬШЕ, а не больше. Все прочие защиты односторонние,
    # ловит только двусторонняя проверка на равенство, и она стоит ниже.
    #
    # Порядок проверок не произволен: неположительная грань — дефект ОДНОЙ
    # грани, и он же поднимает (или гасит) сумму, поэтому он называется первым.
    # Иначе корень был бы спрятан за его следствием.
    #
    # Проверки точные и бесплатные: обе величины уже посчитаны, сравнение с
    # нулём — `is_zero` разности и `sign()`, ни одного порога.
    if not assembled.every_face_is_positive:
        guilty = [
            face for face in faces if face.doubled_area.sign() <= 0
        ]
        areas = ", ".join(as_number(face.doubled_area) for face in guilty[:3])
        return FacePartitionV1(
            FaceOutcome.FACE_IS_NOT_POSITIVE,
            assembled.faces,
            total,
            empty.polygon_doubled_area,
            f"{len(guilty)} из {len(faces)}, удвоенные площади: {areas}",
        )
    if not assembled.area_reproduces_polygon:
        return FacePartitionV1(
            FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON,
            assembled.faces,
            total,
            empty.polygon_doubled_area,
            as_number(assembled.area_defect),
        )
    return assembled
