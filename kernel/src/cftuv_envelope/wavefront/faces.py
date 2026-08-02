"""Грани скелета из узлов очереди: owner каждого куска и его ТОЧНАЯ площадь.

Зачем это здесь. `skeleton.py` выдаёт события — момент, место и участников. Это
достаточно, чтобы доказать порядок и одновременность, но недостаточно, чтобы
сравниться с попарным кроем: тот проверяет себя ПЛОЩАДЬЮ
(`interactions/policy_b.py:976`, `INTERACTION_POLICY_B_PARTITION_UNPROVEN` —
«Policy B clipping did not reproduce the exact RawCoverage set»). Чтобы задать
очереди тот же вопрос, из узлов надо собрать грани и сложить их площади.

ПРАВИЛО СБОРКИ — СМЕЖНОСТЬ. Дальняя граница грани ребра `e` состоит из ДУГ, и
каждая дуга есть след вершины фронта между `e` и каким-то одним другим ребром
`f`. Значит две точки дальней границы соседние тогда и только тогда, когда у них
есть ОБЩИЙ второй участник: именно эта вершина между ними и ехала. Отсюда грань
ребра `e` есть

    e.start, e.end, затем цепочка его узлов, связанная общим вторым участником.

КОНЦЫ ЦЕПОЧКИ ЗАДАНЫ ВХОДОМ, а не найдены перебором и не выбраны по геометрии:
точка с участником `succ(e)` замыкает конец у `e.end`, точка с `pred(e)` — конец
у `e.start`. Соседи ребра по его собственной петле — факт входа, и спрашивать за
них геометрию значило бы гадать там, где вход отвечает точно.

ПОРЯДОК НЕ СПРАШИВАЕТ НИ ОДНОГО ЗНАКА `SqrtSumV1`, и это свойство правила, а не
экономия. Смежность — отношение на МНОЖЕСТВАХ УЧАСТНИКОВ, то есть на целых
ключах рёбер; координаты узлов в него не входят вовсе. Точная арифметика в
модуле остаётся ровно там, где она про площадь и про границы: формула трапеций
над `SqrtSumV1` (у неё есть произведение), `sign()` у положительности,
`is_zero` у равенства сумм и `orientation` у самопересечения. Порогов нет ни
одного.

УЗЛЫ ГРУППИРУЮТСЯ ПО ТОЧКЕ, и это не оптимизация. Несколько событий в одной
точке — обычное дело в вырожденной конфигурации (у креста до пяти записей в
одной точке, у `ell` с двумя стенами шесть участников в одном узле), а вершина
контура там ОДНА. Смежность определена между точками; узлы внутри точки
выдаются подряд, и их взаимный порядок на площадь не влияет, потому что сегмент
между ними нулевой длины.

ЦЕПОЧКА, КОТОРАЯ НЕ СЛОЖИЛАСЬ, — ИМЕНОВАННЫЙ ОТКАЗ `FACE_CHAIN_DOES_NOT_CLOSE`
с причиной-ЧИСЛОМ в `detail`, а не тихий фолбэк на другой порядок. Второго
порядка в модуле нет вовсе: молчаливый запасной путь означал бы, что верным
считается тот ответ, который получился, а не тот, который доказан. На
`named_corpus()`, на сетке крестов 3..11 по трём толщинам рукава и на корпусе
частичного источника отказ не срабатывает ни разу, и этот ноль заморожен
тестом.

ПРЕЖНЕЕ ПРАВИЛО И ПРИЧИНА ЗАМЕНЫ — история, и она записана числами, а не словом
«улучшили». Поставлена была МОНОТОННОСТЬ: «грань опорного ребра монотонна по
направлению этого ребра», узлы сортировались по УБЫВАНИЮ проекции на ребро, а
ничьи (кусок дальней границы, перпендикулярный ребру) разрешались правилом
«глубина вдоль обхода унимодальна». Это теорема straight skeleton, и у неё есть
ГИПОТЕЗА: **каждое ребро движется.** Ровно её нарушает частичный источник —
дальняя граница грани идёт по НЕПОДВИЖНЫМ стенам, то есть буквально по границе
входа, а та немонотонна настолько, насколько немонотонен вход.

| где монотонность ломалась | чем это выглядело |
|---|---|
| полевой патч `bf6` (домен с двумя выступами) | хвост цепочки вставал не туда: три трансверсальных самопересечения на одной грани из трёх, пары сегментов `(5, 9)`, `(6, 9)`, `(7, 9)`, ИЗБЫТОК суммы +2.90 % (112 055 108 527 против 108 901 947 644) |
| крест при `|wide - tall| >= 2 * arm` | вся дальняя граница грани перпендикулярна ребру, обход переворачивался: 32 строки сетки из 243 |
| `staircase_source_edges_7_0` | недостаток площади при НУЛЕВОЙ незаметённой части — чистый дефект порядка |

Замена измерена ДО постановки и на обеих половинах сразу: правило смежности
воспроизводило поставленный порядок ПОТОЧЕЧНО на 185 гранях из 185 корпуса и на
2884 гранях из 2916 сетки крестов, а разошлось ровно на тех 32, где
монотонность и врала. «Новое правило чинит сломанное» без «новое правило не
трогает верное» было бы просто другим правилом.

ПОРЯДОК РАБОТ БЫЛ ОБЯЗАТЕЛЕН, и это тоже часть истории. До разбора коллинеарной
неподвижной стены в `skeleton.py` смежность на трёх фигурах корпуса частичного
источника превращала верный ГРОМКИЙ ОТКАЗ в число, противоречащее независимому
митрованному эталону (`ell_12_source_edge_4` при alpha 8: 120 против 96).
Корень лежал не здесь: фронт гасил стену, с которой его отрезок перекрывался в
единственной точке, и скачком расширялся вдвое. Стена разобрана
(`FILTER_SPAN_DOES_NOT_COLLAPSE`), три фигуры отказывают СКЕЛЕТОМ до всякого
числа, и только после этого смежность поставлена.

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

Тот конкретный перекрут вылечен сменой правила сборки, и граница ОСТАЁТСЯ:
граница снимается тогда, когда доказано, что нарушить её нечем, а не тогда,
когда её перестал нарушать один известный вход. Проверка стоит на том же месте
и на том же входе — `bf6` теперь отвечает `EXACT`, и это утверждение о ней, а не
её отсутствие.

ПОЧЕМУ ТРАНСВЕРСАЛЬНОСТЬ, А НЕ ПОЛНАЯ ПРОСТОТА, и это тоже измерено. Более
строгий вариант («никакие два несоседних сегмента не имеют ОБЩЕЙ ТОЧКИ»)
срабатывает на пяти фигурах корпуса частичного источника, у которых сейчас
`EXACT` и площадь сходится ТОЧНО (`ell_12_source_edges_0_1`,
`ell_12_source_edges_4_5`, `ell_12_source_without_the_reflex_pair`,
`staircase_source_edges_0_1`, `staircase_source_edges_6_7`). Дефект у них один и
тот же и площади не двигает: два узла скелета стоят в ОДНОЙ точке, отчего в
контуре появляется сегмент нулевой длины. Граница, отвергающая верный ответ, —
не граница, поэтому объявлено ровно то, что портит площадь: трансверсальное
пересечение. Все пять фигур сохраняют ровно по одному сегменту нулевой длины и
при поставленном правиле смежности: узлы группируются по точке и выдаются
подряд, поэтому вырожденный сегмент никуда не делся и по-прежнему проходит
границу 1, как и должен.

Касание как ПРОБЕЛ записано и не замолчано, но пример его сменился, и это
следствие починки скелета, а не сборщика: `staircase_source_edge_6` имел
защемление контура без трансверсального пересечения, а теперь отказывает
`WAVEFRONT_LEFT_UNRESOLVED` до всякой сборки — под его дефектом лежала
коллинеарная неподвижная стена. Живого входа с защемлением на корпусе сейчас
нет ни одного; сама конфигурация от этого не перестала быть возможной, и
ловить её по-прежнему пришлось бы границе 3.

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
from fractions import Fraction

from .event_time import SupportLineV1
from .events import EventKind
from .polygon import PolygonV1, signed_double_area
from .skeleton import SkeletonNodeV1, SkeletonOutcome, SkeletonV1
from .sqrt_sum import SqrtSumV1


#: Ключ УЧАСТНИКА события и владельца грани: вхождение ребра `(x0, y0, x1, y1)`,
#: а у скрытой опоры веера — `(x, y, x, y, ordinal)`. Имя `LineKey` не оставлено
#: даже синонимом: тип у него тот же, и молчаливый синоним означал бы, что смысл
#: ключа выясняется по значению, а не по имени.
#:
#: Ординал в ключе веера обязателен, и это не запас на будущее: `k` опор одной
#: вершины стоят в ОДНОЙ точке, вхождения у них вырождены и совпадают все до
#: одного, а граней у них `k`. Без ординала они слились бы в одного участника, и
#: `TWO_EDGES_SHARE_ONE_SPAN` был бы верным ответом на неверно заданный вопрос.
EdgeKey = tuple[int, ...]


class FaceOutcome(str, Enum):
    """Чем кончилась сборка граней. Тихого возврата нет ни одного."""

    EXACT = "EXACT"
    # Скелет не доказан — граней у него нет, и придумывать их нельзя.
    SKELETON_IS_NOT_EXACT = "SKELETON_IS_NOT_EXACT"
    MULTIWAY_NODE_INCIDENCE_UNAVAILABLE = (
        "MULTIWAY_NODE_INCIDENCE_UNAVAILABLE"
    )
    # Два вхождения с одинаковыми концами: вход задал одно ребро дважды, и грань
    # у них была бы одна на двоих. Заменил `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES`,
    # который отказывал на коллинеарных рёбрах РАЗНЫХ граней, — это оказалось
    # свойством ключа, а не входа, и вылечено сменой ключа.
    TWO_EDGES_SHARE_ONE_SPAN = "TWO_EDGES_SHARE_ONE_SPAN"
    # У ребра нет ни одного узла. Грань не замкнуть двумя точками.
    FACE_HAS_NO_SKELETON_NODE = "FACE_HAS_NO_SKELETON_NODE"
    # Цепочка смежности не сложилась: развилка, участник в трёх и более точках,
    # концов не по одному, цепочка не покрыла все точки грани либо кончилась не
    # у предшествующего ребра. В `detail` лежит ЧИСЛО — сколько точек прошли,
    # сколько их всего, сколько нашлось концов, — потому что «не сложилась» без
    # числа не отличает развилку от разрыва и не говорит, где обрыв.
    #
    # Член ОТДЕЛЬНЫЙ и молчаливого запасного порядка рядом с ним нет: правило
    # смежности либо доказало обход, либо не доказало, и второй ответ тут был бы
    # ровно тем тихим неверным числом, против которого заведены три границы
    # ниже. На корпусах и на всей сетке крестов исход не срабатывает ни разу, и
    # ноль этот заморожен тестом, а не подразумевается.
    FACE_CHAIN_DOES_NOT_CLOSE = "FACE_CHAIN_DOES_NOT_CLOSE"
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
    #: Несущая прямая опорного ребра ЦЕЛИКОМ. Хранилось `q`, а прямая
    #: восстанавливалась по концам (`SupportLineV1.with_speed`), — у ребра
    #: НУЛЕВОЙ ДЛИНЫ этот путь падает `DegenerateEdgeError`, потому что нормаль
    #: из совпавших концов не выводится. У скрытой опоры веера нормаль задана
    #: входом и ниоткуда больше не следует, поэтому хранится она, а не её
    #: половина.
    line: SupportLineV1 | None = None

    @property
    def speed_squared(self) -> int | Fraction:
        return 0 if self.line is None else self.line.q

    @property
    def is_fan_support(self) -> bool:
        """Грань скрытой опоры веера: её ребро имеет нулевую длину."""

        return self.source_start == self.source_end

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

    Рёбра ВЕЕРА сюда не входят: у них нулевая длина, а функция отдаёт пару
    концов, из которой вызывающие выводят направление. Полный список фронтов —
    `polygon_fronts`.
    """

    return tuple(
        (start, end, SupportLineV1.with_speed(start, end, speed_squared))
        for start, end, speed_squared in polygon.edges()
    )


def polygon_fronts(
    polygon: PolygonV1,
) -> tuple[tuple[EdgeKey, tuple[int, int], tuple[int, int], SupportLineV1], ...]:
    """ВСЕ фронты области: контурные рёбра, затем скрытые опоры вееров.

    Ключ идёт первым полем, а не выводится из концов, ровно потому, что у
    скрытой опоры концы совпадают и различает такие опоры только ординал.
    Прямая веера берётся из `polygon.fan_edges()` — единственного места, где она
    объявлена; порядок между собой детерминирован сортировкой по точке, чтобы
    отчёт не зависел от порядка объявления вееров.
    """

    records: list[
        tuple[EdgeKey, tuple[int, int], tuple[int, int], SupportLineV1]
    ] = [
        (edge_key(start, end), start, end, line)
        for start, end, line in polygon_edges(polygon)
    ]
    records.extend(
        (fan_edge_key(point, ordinal), point, point, line)
        for point, ordinal, line in polygon.fan_edges()
    )
    return tuple(records)


def edge_key(start: tuple[int, int], end: tuple[int, int]) -> EdgeKey:
    """Ключ вхождения ребра. Тот же, что `_Edge.span` в `skeleton.py`.

    Функция одна на модуль и одна на репозиторий именно затем, чтобы сборщик и
    очередь не могли разойтись в тождестве участника правкой одного из двух.
    """

    return (start[0], start[1], end[0], end[1])


def fan_edge_key(point: tuple[int, int], ordinal: int) -> EdgeKey:
    """Ключ вхождения скрытой опоры веера: вырожденный отрезок плюс ординал.

    Ординал начинается с единицы и совпадает с `HiddenSupportSpecV1.ordinal`
    эталона. Пятый элемент отличает ключ веера от ключа контурного ребра и в
    сортировке ставит его сразу за вырожденным отрезком — то есть порядок
    остаётся полным и без единого исключения по типу.
    """

    return (point[0], point[1], point[0], point[1], ordinal)


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


def edge_neighbours(
    polygon: PolygonV1,
) -> dict[EdgeKey, tuple[EdgeKey, EdgeKey]]:
    """Для каждого ребра — сосед НАЗАД и сосед ВПЕРЁД по его собственной петле.

    Нужны затем, что цепочка грани имеет два объявленных конца, и оба заданы
    входом, а не найдены перебором. Искать концы по геометрии значило бы гадать
    там, где вход отвечает точно.

    Сосед может оказаться стеной, и это законно: у стены нет своей грани, но
    вершина фронта между ребром и стеной существует и оставляет след — именно
    она и замыкает цепочку.

    ВЕЕР ВСТАЁТ В ЭТОТ ЖЕ ЦИКЛ, а не рядом с ним: скрытые опоры вставлены в LAV
    между входящим и исходящим ребром вершины, значит и в кольце соседства они
    стоят там же. Отдельная ветка «а если веер» означала бы второй порядок
    обхода, который мог бы разойтись с первым — и разошёлся бы молча, потому что
    концы цепочки берутся отсюда.
    """

    neighbours: dict[EdgeKey, tuple[EdgeKey, EdgeKey]] = {}
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        ring: list[EdgeKey] = []
        for index in range(size):
            fan = polygon.fan_at(points[index])
            if fan is not None:
                ring.extend(
                    fan_edge_key(points[index], ordinal)
                    for ordinal in range(1, len(fan.supports) + 1)
                )
            ring.append(edge_key(points[index], points[(index + 1) % size]))
        total = len(ring)
        for index, span in enumerate(ring):
            neighbours[span] = (
                ring[(index - 1) % total],
                ring[(index + 1) % total],
            )
    return neighbours


def face_chain(
    owner: EdgeKey,
    nodes: tuple[SkeletonNodeV1, ...],
    previous: EdgeKey,
    following: EdgeKey,
) -> tuple[tuple[SkeletonNodeV1, ...] | None, str]:
    """Узлы грани в порядке смежности либо `None` и ПРИЧИНА, почему не вышло.

    Узлы сначала группируются ПО ТОЧКЕ: несколько записей в одной точке дают
    одну вершину контура, и их взаимный порядок на площадь не влияет — сегмент
    между ними нулевой длины. Дальше каждой точке сопоставляется множество её
    вторых участников, и две точки объявляются соседними ровно тогда, когда у
    них есть общий второй участник.

    Проверок здесь пять, и каждая отвечает своим числом, а не общим «не
    сложилось»: участник больше чем в двух точках (дуг у одной пары рёбер
    больше одной), концов не по одному, развилка на обходе, обход не покрыл все
    точки, обход кончился не у предшествующего ребра. Причина возвращается
    строкой и поднимается наверх исходом `FACE_CHAIN_DOES_NOT_CLOSE`.
    """

    places: dict[tuple, list[SkeletonNodeV1]] = {}
    for node in nodes:
        places.setdefault(
            (node.point.x.terms, node.point.y.terms), []
        ).append(node)
    order_of_place = list(places)
    partners = [
        {
            participant
            for node in places[place]
            for participant in node.participants
            if participant != owner
        }
        for place in order_of_place
    ]

    shared: dict[EdgeKey, list[int]] = {}
    for index, group in enumerate(partners):
        for participant in group:
            shared.setdefault(participant, []).append(index)
    crowded = sorted(f for f, seats in shared.items() if len(seats) > 2)
    if crowded:
        return None, (
            f"{len(crowded)} участников в трёх и более точках из "
            f"{len(places)}: {crowded[:2]}"
        )

    links: dict[int, set[int]] = {index: set() for index in range(len(places))}
    for seats in shared.values():
        if len(seats) == 2:
            first, second = seats
            links[first].add(second)
            links[second].add(first)

    heads = [i for i, group in enumerate(partners) if following in group]
    tails = [i for i, group in enumerate(partners) if previous in group]
    if len(heads) != 1 or len(tails) != 1:
        return None, f"концов цепочки {len(heads)} и {len(tails)}, нужно по 1"

    walk = [heads[0]]
    seen = {heads[0]}
    while True:
        ahead = sorted(links[walk[-1]] - seen)
        if not ahead:
            break
        if len(ahead) > 1:
            return None, f"развилка после {len(walk)} точек из {len(places)}"
        walk.append(ahead[0])
        seen.add(ahead[0])
    if len(walk) != len(places):
        return None, f"цепочка {len(walk)} точек из {len(places)}"
    if walk[-1] != tails[0]:
        return None, (
            f"цепочка из {len(walk)} точек кончилась не у предшествующего ребра"
        )
    return tuple(
        node for index in walk for node in places[order_of_place[index]]
    ), ""


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
    """Грани скелета по правилу СМЕЖНОСТИ, со всеми тремя границами в ответе."""

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

    edges = polygon_fronts(polygon)
    keys = [key for key, _, _, _ in edges]
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
        if node.kind is EventKind.MULTIWAY:
            if not node.incidences:
                return FacePartitionV1(
                    FaceOutcome.MULTIWAY_NODE_INCIDENCE_UNAVAILABLE,
                    (),
                    SqrtSumV1.zero(),
                    empty.polygon_doubled_area,
                    "MULTIWAY node has no original participant incidences",
                )
            incidences = node.incidences
        else:
            incidences = (node.participants,)
        for incidence in incidences:
            for key in incidence:
                nodes_by_key.setdefault(key, []).append(node)
    neighbours = edge_neighbours(polygon)

    faces: list[FaceV1] = []
    total = SqrtSumV1.zero()
    for key, start, end, line in edges:
        if line.is_stationary:
            # Стена не заметает НИЧЕГО: её отрезок фронта остаётся на своей
            # прямой, только укорачивается. Грань нулевой площади — не грань, и
            # граница «каждая грань строго положительна» отвергла бы её. Узлы
            # стены при этом никуда не деваются: они входят в грани соседей как
            # участники, и сумма площадей по-прежнему обязана дать всю область.
            continue
        candidates = nodes_by_key.get(key, ())
        if not candidates:
            return FacePartitionV1(
                FaceOutcome.FACE_HAS_NO_SKELETON_NODE,
                (),
                SqrtSumV1.zero(),
                empty.polygon_doubled_area,
                f"ребро {start} -> {end} без узлов",
            )
        previous, following = neighbours[key]
        chain, why = face_chain(key, tuple(candidates), previous, following)
        if chain is None:
            return FacePartitionV1(
                FaceOutcome.FACE_CHAIN_DOES_NOT_CLOSE,
                (),
                SqrtSumV1.zero(),
                empty.polygon_doubled_area,
                f"ребро {start} -> {end}: {why}",
            )
        points = (
            (SqrtSumV1.rational(start[0]), SqrtSumV1.rational(start[1])),
            (SqrtSumV1.rational(end[0]), SqrtSumV1.rational(end[1])),
        ) + tuple((node.point.x, node.point.y) for node in chain)
        doubled = doubled_shoelace(points)
        faces.append(FaceV1(key, start, end, points, doubled, line))
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
    угодно: у прежнего правила монотонности на `bf6` он дал положительную грань
    и ИЗБЫТОК суммы (+2.90%), на остатке креста при `|wide - tall| >= 2 * arm` —
    отрицательную грань. Оба раза «площадь не та» было следствием, а корнем —
    порядок.

    Замер, который это доказал (и который же назвал корень заменяемым): на
    сетке `wide, tall` из 3..11 по трём толщинам рукава креста (243 строки) ВСЕ
    32 строки, отвечавшие у монотонности `FACE_IS_NOT_POSITIVE`, имели
    трансверсальное самопересечение, и ни одна из 211 строк с `EXACT` его не
    имела. То есть граница 1 не добавила ни одного отказа и не сняла ни одного —
    она переименовала имевшиеся в их корень, а корень оказался правилом порядка.
    С правилом смежности этих 32 отказов нет вовсе, и это проверяется на той же
    сетке тем же числом.

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
