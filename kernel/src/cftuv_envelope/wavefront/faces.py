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

ОБЪЯВЛЕННЫЕ ГРАНИЦЫ (границ ТРИ, и все три проверяются САМОЙ `build_faces`
перед тем, как она вернёт `EXACT`, — у каждой свой член `FaceOutcome` и число
дефекта в `detail`; тест `test_wavefront_faces.py` проверяет их же на
собственном результате функции). Порядок здесь — это порядок ОПРОСА, и он от
причины к следствию:

1. **Контур каждой грани ПРОСТОЙ**: никакие два его сегмента не пересекаются
   ТРАНСВЕРСАЛЬНО. Перекрученный контур — дефект СБОРКИ, то есть порядка обхода;
   он же портит обе оставшиеся границы, поэтому спрашивается первым.
2. **Площадь каждой грани строго положительна.** Грань — двумерный кусок, а не
   вырожденная щепка и не кусок с вывернутой ориентацией.
3. **Сумма площадей граней РАВНА площади многоугольника.** Точно, не «почти».
   Это и есть ровно то утверждение, которое попарный крой о себе не доказывает.

ПОЧЕМУ ТРЕТЬЯ ГРАНИЦА ПОНАДОБИЛАСЬ, и это измерено, а не выведено. Границы 2 и
3 обе смотрят на ПЛОЩАДЬ, а формула трапеций на перекрученном контуре считает
кусок дважды либо с обратным знаком. На полевом патче `bf6` это дало ИЗБЫТОК
+2.90% (112 055 108 527 против 108 901 947 644), которого не было ни в одном из
случаев корпуса, — и по площади причина не читалась вовсе: грань 1 из трёх
самопересекалась ТРИЖДЫ, а границу 2 проходила, потому что оставалась
положительной. То есть перекрученный контур проходил молча, и класс «тихого
исчезновения» здесь был не в площади, а в порядке.

ПОЧЕМУ ТРАНСВЕРСАЛЬНОСТЬ, А НЕ ПОЛНАЯ ПРОСТОТА, и это тоже измерено. Более
строгий вариант («никакие два несоседних сегмента не имеют ОБЩЕЙ ТОЧКИ»)
срабатывает на пяти фигурах корпуса частичного источника, у которых сейчас
`EXACT` и площадь сходится ТОЧНО (`ell_12_source_edges_0_1`,
`ell_12_source_edges_4_5`, `ell_12_source_without_the_reflex_pair`,
`staircase_source_edges_0_1`, `staircase_source_edges_6_7`). Дефект у них один и
тот же и площади не двигает: два узла скелета стоят в ОДНОЙ точке, отчего в
контуре появляется сегмент нулевой длины. Граница, отвергающая верный ответ, —
не граница, поэтому объявлено ровно то, что портит площадь: трансверсальное
пересечение. Касание записано как известный пробел, а не замолчано:
`staircase_source_edge_6` имеет защемление контура без трансверсального
пересечения, и ловится оно границей 3.

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

from dataclasses import dataclass, replace
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
    # Граница 1 нарушена: контур грани самопересекается, то есть порядок обхода
    # перекрутил его. Член отдельный, потому что это дефект СБОРКИ, а не
    # площади: площадь на таком контуре считается по формуле трапеций и выходит
    # каким угодно — и больше истинной, и меньше, и даже равной ей. В `detail`
    # лежат ЧИСЛО пересечений и сами пары сегментов, потому что «контур не
    # простой» без пар не отличает один перекрут от трёх и не даёт места, где
    # чинить.
    FACE_CONTOUR_IS_NOT_SIMPLE = "FACE_CONTOUR_IS_NOT_SIMPLE"
    # Граница 3 нарушена: сумма площадей граней НЕ равна площади многоугольника.
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
    """Разбиение многоугольника гранями скелета и все три объявленные границы."""

    # При отказе по любой из трёх границ грани НЕ обнуляются, в отличие от
    # отказов, случившихся до счёта: там граней просто нет, а тут они
    # посчитаны, и без них дефект нечем измерить. Смотреть на них можно только
    # после проверки `outcome`, и потребитель это делает (`coverage.py:160`).
    outcome: FaceOutcome
    faces: tuple[FaceV1, ...]
    doubled_area: SqrtSumV1
    polygon_doubled_area: int
    detail: str = ""

    @property
    def every_contour_is_simple(self) -> bool:
        """Граница 1: ни один контур не пересекает сам себя трансверсально."""

        return all(not contour_crossings(face.points) for face in self.faces)

    @property
    def area_reproduces_polygon(self) -> bool:
        """Граница 3: сумма площадей граней равна площади многоугольника.

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


def orientation(first: Point, second: Point, third: Point) -> int:
    """Знак удвоенной ориентированной площади треугольника. ТОЧНО, без порогов.

    Тот же самый предикат, что и `doubled_shoelace` на трёх точках, только
    спрошенный про ЗНАК: `(b - a) x (c - a)`. Величина здесь не нужна и не
    возвращается — читать `SqrtSumV1` по частям нельзя, а `.sign()` отвечает
    целиком (сначала целочисленная оболочка, при неудаче сопряжение).
    """

    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    ).sign()


def segments_cross(
    first_start: Point, first_end: Point, second_start: Point, second_end: Point
) -> bool:
    """Два отрезка пересекаются ТРАНСВЕРСАЛЬНО: каждый строго разделяет другой.

    Именно трансверсальность, а не «имеют общую точку», и это объявлено, а не
    упрощено. Касание концом и наложение коллинеарных кусков площадь формулы
    трапеций НЕ меняют, а в верных гранях корпуса частичного источника
    встречаются (два узла скелета в одной точке дают сегмент нулевой длины).
    Граница, отвергающая верный ответ, — не граница; поэтому объявлено ровно
    то, что портит площадь.

    Условие Cormen: концы одного отрезка лежат по РАЗНЫЕ стороны от прямой
    другого, и наоборот. Строгое произведение знаков `< 0` с обеих сторон само
    исключает вырожденные случаи — при нулевом ориентире произведение равно
    нулю, и трансверсальности нет.
    """

    return (
        orientation(second_start, second_end, first_start)
        * orientation(second_start, second_end, first_end)
        < 0
        and orientation(first_start, first_end, second_start)
        * orientation(first_start, first_end, second_end)
        < 0
    )


def contour_crossings(points: tuple[Point, ...]) -> tuple[tuple[int, int], ...]:
    """Пары индексов сегментов замкнутого контура, пересекающихся трансверсально.

    Сегмент `k` — это `points[k] -> points[(k + 1) % n]`. Соседние по контуру
    пары пропускаются: у них общий конец по построению, и трансверсальным их
    пересечение всё равно быть не может, но проверять их означало бы тратить
    точный предикат на заведомый ответ.

    Возвращаются ПАРЫ, а не признак: «контур не простой» без пар не отличает
    один перекрут от трёх и не показывает, какой кусок цепочки поставлен не
    туда. На `bf6` это ровно те три пары, по которым и найден хвост цепочки.
    """

    size = len(points)
    crossings: list[tuple[int, int]] = []
    for first in range(size):
        for second in range(first + 1, size):
            if (second + 1) % size == first or (first + 1) % size == second:
                continue
            if segments_cross(
                points[first],
                points[(first + 1) % size],
                points[second],
                points[(second + 1) % size],
            ):
                crossings.append((first, second))
    return tuple(crossings)


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

    return check_declared_boundaries(
        FacePartitionV1(
            FaceOutcome.EXACT,
            tuple(faces),
            total,
            empty.polygon_doubled_area,
        )
    )


def check_declared_boundaries(assembled: FacePartitionV1) -> FacePartitionV1:
    """Опрос всех ТРЁХ объявленных границ, от причины к следствию.

    Вынесено из `build_faces` отдельной функцией и названо, а не спрятано в
    приватное имя: это самостоятельный этап конвейера, и стенд обязан уметь
    задать те же вопросы готовому разбиению, не пересобирая его.

    Границы спрашиваются ЗДЕСЬ, до возврата `EXACT`, а не только тестом.
    Причина измерена, а не гигиеническая: на кресте сборка теряла клин площади,
    отдавала `EXACT`, и потеря протекала до самого ответа — граница 2 при этом
    ДЕРЖАЛАСЬ (все грани положительны), а `coverage_at.does_not_exceed_the_polygon`
    проходила, потому что потеря делает покрытие МЕНЬШЕ, а не больше. Все
    прочие защиты односторонние; ловит только двусторонняя проверка равенства.

    ПОРЯДОК ОПРОСА — от причины к следствию, и первая его ступень доказана
    числом, а не рассуждением.

    Самопересечение спрашивается ПЕРВЫМ, потому что оно единственное из трёх
    есть дефект ПОРЯДКА ОБХОДА, а две другие границы читают ПЛОЩАДЬ, которую
    этот порядок и задаёт. Формула трапеций на перекрученном контуре считает
    кусок дважды либо с обратным знаком, поэтому перекрут проявляется как что
    угодно: на `bf6` он дал положительную грань и ИЗБЫТОК суммы (+2.90%), на
    остатке креста при `|wide - tall| >= 2 * arm` — отрицательную грань. Оба
    раза «площадь не та» было следствием, а корнем — порядок.

    Замер, который это доказал: на сетке `wide, tall` из 3..11 по трём толщинам
    рукава креста (243 строки) ВСЕ 32 строки, отвечавшие `FACE_IS_NOT_POSITIVE`,
    имеют трансверсальное самопересечение, и ни одна из 211 строк с `EXACT` его
    не имеет. То есть новая граница не добавила ни одного отказа и не сняла ни
    одного — она переименовала имевшиеся в их корень.

    Дальше положительность и только потом сумма — по прежнему доводу:
    вывернутая грань сама сдвигает сумму, и назвать следствие раньше корня
    значило бы спрятать корень.

    Все три проверки точные: `segments_cross` — знаком `SqrtSumV1`,
    положительность — `sign()`, равенство сумм — `is_zero` разности. Порога нет
    ни одного. Грани при отказе НЕ обнуляются: без них дефект нечем измерить.
    """

    faces = assembled.faces
    twisted = [
        (face, contour_crossings(face.points))
        for face in faces
        if contour_crossings(face.points)
    ]
    if twisted:
        where = "; ".join(
            f"{face.owner}: {list(pairs)}" for face, pairs in twisted[:3]
        )
        return replace(
            assembled,
            outcome=FaceOutcome.FACE_CONTOUR_IS_NOT_SIMPLE,
            detail=(
                f"{sum(len(pairs) for _, pairs in twisted)} пересечений на "
                f"{len(twisted)} гранях из {len(faces)}, пары сегментов: {where}"
            ),
        )
    if not assembled.every_face_is_positive:
        guilty = [face for face in faces if face.doubled_area.sign() <= 0]
        areas = ", ".join(as_number(face.doubled_area) for face in guilty[:3])
        return replace(
            assembled,
            outcome=FaceOutcome.FACE_IS_NOT_POSITIVE,
            detail=(
                f"{len(guilty)} из {len(faces)}, удвоенные площади: {areas}"
            ),
        )
    if not assembled.area_reproduces_polygon:
        return replace(
            assembled,
            outcome=FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON,
            detail=as_number(assembled.area_defect),
        )
    return assembled
