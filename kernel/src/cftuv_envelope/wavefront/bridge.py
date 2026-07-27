"""Мост входа: из того, что есть у попарного кроя, — во вход очереди событий.

Это самая содержательная часть дифференциальной сверки, потому что входы у двух
путей РАЗНЫЕ, и соответствие между ними надо не предположить, а записать.

    попарный крой (`interactions/policy_b.py`)   очередь (`wavefront/skeleton.py`)
    ------------------------------------------   --------------------------------
    домен: PlanarRegion, точные дроби            PolygonV1: узлы ЦЕЛОЙ решётки
    источник: закон прихода n*p = c + a*s        SupportLineV1: a*x + b*y = c + t*sqrt(q)
    компонент: юбка со своим законом             ребро контура, одно из всех
    alpha конечна (в поле 1/4)                   скелет считается до конца

ЧТО ОТОБРАЖАЕТСЯ ТОЧНО. Закон прихода `n*p = c + alpha*s` совпадает с несущей
прямой `a*x + b*y = c' + t*sqrt(q)` тогда и только тогда, когда `(a, b, c')`
пропорционально `(n_x, n_y, c)` с ПОЛОЖИТЕЛЬНЫМ рациональным множителем и
`q = (lambda*s)^2`. Отсюда единственное условие совпадения времён: **фронт
идёт с единичной скоростью по Евклиду**, то есть `s^2 = n_x^2 + n_y^2`. Проверка
чисто рациональная, поэтому порога в ней нет: `s` бывает иррациональной, но `s^2`
рациональна при обеих принятых записях закона (`s = 1` и `s = sqrt(q)`), и
вызывающий передаёт именно `s^2`.

Ориентация отображается тоже, и её нормирует `PolygonV1.build`: внешний контур
CCW, дыры CW, «влево от хода» — в сторону материала. Отдельного параметра
стороны фронта не требуется.

ЧТО НЕ ОТОБРАЖАЕТСЯ. Каждый пункт здесь — ИМЕНОВАННЫЙ исход, а не оговорка:

1. `DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE` — `PolygonV1` требует целых узлов,
   а домен `reference/` живёт в точных дробях. Привязка к решётке существует
   (`robust/grid.py`), но она СДВИГАЕТ вершины, то есть меняет задачу, и делать
   это внутри моста молча нельзя.
2. `ARRIVAL_LAW_IS_NOT_UNIT_SPEED` — у закона `s^2 != |n|^2`. Тогда фронты идут
   с РАЗНЫМИ евклидовыми скоростями, и скелет нужен взвешенный. Арифметика
   `event_time.py` к этому готова (`q` — свободное целое поле), но геометрия нет:
   `SupportLineV1.through` строит только `q = |d|^2`, а теорема 2.11 в
   `motorcycle.py` опирается на `dist(p, line) = t`, что при `s != |n|` неверно, —
   значит индекс кандидатов по ячейкам теряет право фильтровать.
3. `ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE` — закон, чья прямая не является несущей
   прямой ни одного ребра домена. Такому источнику ребра контура не
   соответствует вовсе.
4. `MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE` — на одной несущей прямой законов
   больше, чем рёбер домена. Лишний закон источником быть не может: у очереди
   ребро одно, а претендентов на него два.
5. `SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED` — на одной несущей прямой
   рёбер больше, чем законов, но законов не ноль. Какое именно из рёбер класса
   источник, во входе НЕ ЗАПИСАНО: закон протяжённости не несёт. Выбрать одно
   значило бы придумать пометку.

ЧАСТИЧНЫЙ ИСТОЧНИК БОЛЬШЕ НЕ ОТКАЗ. Исход `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY`
удалён, а не переименован: он говорил «`PolygonV1` неподвижную часть границы не
выражает», и это перестало быть правдой — у ребра появилось своё `q`, и ребро
без закона становится СТЕНОЙ (`q = 0`). Оставить исход членом перечисления
значило бы держать в диагностике причину, которой нет: он не сработал бы больше
никогда, и отличить «не бывает» от «не проверяется» стало бы нечем.

СОПОСТАВЛЕНИЕ ЗАКОНА С РЕБРОМ — БИЕКЦИЯ, и это не украшение. Раньше ребро для
закона искалось `next(...)` по первой попавшейся пропорциональной прямой, и два
коллинеарных сонаправленных ребра давали ОДИН И ТОТ ЖЕ индекс. Счёт
сопоставленных рёбер тогда недосчитывался, и `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY`
срабатывал там, где источником было каждое ребро: на кресте `cross(wide=6,
tall=4)` — 12 законов на 12 рёбер и отказ, на `holes_2` — 12 на 12 и отказ.
Диагноз лгал, а на нём стоит планирование: на полевом патче `bf6` тот же исход
верен по НАСТОЯЩЕЙ причине (3 ребра домена из 12).

Чем различать коллинеарные сонаправленные рёбра — ИЗМЕРЕНО, а не выбрано:
`PlainArrivalLawV1` не несёт протяжённости вовсе (поля — имя, нормаль, константа,
квадрат скорости), поэтому «проекции концов» взять неоткуда. Значит закон
определяет КЛАСС несущей прямой, а не вхождение ребра, и вопрос «сколько рёбер
имеют источник» — это размер наибольшего паросочетания в двудольном графе
«закон — ребро», где ребро графа есть совпадение классов.

Этот граф — несвязное объединение ПОЛНЫХ двудольных блоков по одному на класс
(совпадение классов есть отношение эквивалентности: пропорциональность с
положительным множителем рефлексивна, симметрична и транзитивна). У полного
двудольного блока наибольшее паросочетание равно `min(законов, рёбер)`, поэтому
размер считается сложением, а не поиском. Никакого перебора и ни одного порога.

Внутри класса, где рёбер несколько, ЧЬЁ ребро чей — по входу не определено, и
это названо, а не заполнено произвольно: `owner_by_edge` содержит только
вынужденные назначения (класс из одного закона и одного ребра), а остальные
рёбра перечислены в `ambiguous_owner_spans`. Придуманная пара выглядела бы как
знание, которого во входе нет.

Конечность alpha в исходы не входит: она отображается усечением по времени и
разбирается в стенде, а не здесь.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .event_time import SupportLineV1
from .polygon import LoopV1, PolygonV1, with_source_spans


class BridgeOutcome(str, Enum):
    """Именованные исходы моста. Тихого «не получилось» здесь нет."""

    EXACT = "EXACT"
    NO_ARRIVAL_LAW_GIVEN = "NO_ARRIVAL_LAW_GIVEN"
    DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE = (
        "DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE"
    )
    ARRIVAL_LAW_IS_NOT_UNIT_SPEED = "ARRIVAL_LAW_IS_NOT_UNIT_SPEED"
    ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE = "ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE"
    # На одной несущей прямой законов больше, чем рёбер домена. Член заведён не
    # для полноты: до биекции лишний закон ТИХО затирал предыдущего в
    # `owner_by_index`, потому что `next(...)` возвращал им один и тот же
    # индекс, и в `unmatched_laws` он при этом не попадал — прямая-то домену
    # принадлежит. Исчезновение было безымянным, теперь оно названо.
    MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE = (
        "MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE"
    )
    # Класс несущей прямой покрыт законами ЧАСТИЧНО: рёбер больше, чем законов,
    # и оба числа не ноль. Кто из рёбер класса источник, а кто стена, во входе
    # не записано, и придумать это нельзя — пометка попала бы не на то ребро,
    # а разница между стеной и источником в геометрии полная.
    SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED = (
        "SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED"
    )


# Порядок объявлен, а не выведен из порядка проверок: чем ниже, тем глубже
# причина. `outcome` берёт первый применимый, `findings` перечисляет ВСЕ.
_OUTCOME_ORDER = (
    BridgeOutcome.NO_ARRIVAL_LAW_GIVEN,
    BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE,
    BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,
    BridgeOutcome.ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE,
    BridgeOutcome.MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE,
    BridgeOutcome.SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED,
)


@dataclass(frozen=True, slots=True)
class PlainArrivalLawV1:
    """Закон прихода `n*p = c + alpha*s` в точных дробях, без SymPy.

    `speed_squared` — это `s^2`, и он рационален при обеих принятых записях
    закона: у полевого пути `s = 1`, у синтетического `s = sqrt(q)`. Хранить
    квадрат, а не саму скорость, — не приём: именно квадрат делает проверку
    единичной скорости рациональной, то есть беспороговой.
    """

    name: str
    normal_x: Fraction
    normal_y: Fraction
    constant: Fraction
    speed_squared: Fraction

    @property
    def normal_squared(self) -> Fraction:
        return self.normal_x * self.normal_x + self.normal_y * self.normal_y

    @property
    def is_unit_speed(self) -> bool:
        """Единичная евклидова скорость. Рациональное равенство, не допуск."""

        return self.speed_squared == self.normal_squared

    @property
    def speed_ratio_squared(self) -> Fraction:
        """`(s/|n|)^2`. Единица означает совпадение времён по построению."""

        return self.speed_squared / self.normal_squared


@dataclass(frozen=True, slots=True)
class BridgeReportV1:
    """Что отобразилось, что нет и с какими числами."""

    outcome: BridgeOutcome
    findings: tuple[BridgeOutcome, ...]
    polygon: PolygonV1 | None
    edge_count: int
    law_count: int
    matched_edge_count: int
    off_lattice_points: tuple[tuple[Fraction, Fraction], ...]
    non_unit_speed_laws: tuple[tuple[str, Fraction], ...]
    unmatched_laws: tuple[str, ...]
    # Законы, чья прямая домену принадлежит, но рёбер на ней меньше, чем
    # претендентов. Отдельно от `unmatched_laws`, потому что причина другая:
    # там прямой нет вовсе, здесь она есть и занята.
    surplus_laws: tuple[str, ...]
    # Ключ — ВХОЖДЕНИЕ ребра `(x0, y0, x1, y1)`, а не `(a, b, c, q)` несущей
    # прямой. Прежний ключ у двух коллинеарных рёбер ОДНОЙ длины совпадал
    # (`holes_2`), и карта владельцев молча теряла записи.
    owner_by_edge: tuple[tuple[tuple[int, int, int, int], str], ...]
    # Рёбра, у которых источник есть, а КОТОРЫЙ именно — по входу не определено:
    # на их несущей прямой несколько рёбер и несколько законов, а закон
    # протяжённости не несёт. Перечислены, а не заполнены наугад.
    ambiguous_owner_spans: tuple[tuple[int, int, int, int], ...] = ()
    # Рёбра домена, ставшие СТЕНАМИ: закона у них нет, фронт от них не идёт.
    # Перечислены только на решётке — вне её полигона нет вовсе; ЧИСЛО их
    # доступно всегда через `wall_edge_count`, и это тот самый размер
    # частичного источника, ради которого срез.
    wall_spans: tuple[tuple[int, int, int, int], ...] = ()
    # Сколько рёбер лежат в классах, покрытых законами ЧАСТИЧНО: источник среди
    # них есть, а какое именно — во входе не записано. Ноль означает, что
    # разметка «источник или стена» определена входом однозначно.
    undetermined_source_count: int = 0

    @property
    def wall_edge_count(self) -> int:
        """Рёбра домена БЕЗ закона прихода. Они и есть неподвижная часть границы.

        Считается вычитанием, а не длиной списка стен: список существует только
        на решётке, а число — свойство сопоставления, и оно осмысленно и тогда,
        когда полигон построить нельзя.
        """

        return self.edge_count - self.matched_edge_count

    @property
    def maps(self) -> bool:
        return self.outcome is BridgeOutcome.EXACT

    @property
    def speed_ratio_spread(self) -> tuple[Fraction, Fraction] | None:
        """Наименьшее и наибольшее `(s/|n|)^2` среди законов, если они разные.

        Спред больше единицы по отношению — прямое свидетельство того, что
        скелет здесь ВЗВЕШЕННЫЙ, а не единичный.
        """

        if not self.non_unit_speed_laws:
            return None
        values = [value for _, value in self.non_unit_speed_laws]
        return min(values), max(values)


def _as_lattice(value: Fraction) -> int | None:
    return int(value) if value.denominator == 1 else None


def _lattice_span(edge: "RationalEdge") -> tuple[int, int, int, int]:
    """Вхождение ребра целыми числами: `(x0, y0, x1, y1)`. Только на решётке."""

    (x0, y0), (x1, y1), _ = edge
    return (int(x0), int(y0), int(x1), int(y1))


def _proportional_positive(
    left: tuple[Fraction, Fraction, Fraction],
    right: tuple[Fraction, Fraction, Fraction],
) -> bool:
    """`left = k*right` при `k > 0`. Точно, перекрёстными произведениями."""

    factor: Fraction | None = None
    for a, b in zip(left, right):
        if b == 0:
            if a != 0:
                return False
            continue
        candidate = Fraction(a, 1) / b
        if factor is None:
            factor = candidate
        elif factor != candidate:
            return False
    return factor is not None and factor > 0


RationalPoint = tuple[Fraction, Fraction]
RationalLine = tuple[Fraction, Fraction, Fraction]
#: Ребро домена: два его конца и его несущая прямая.
RationalEdge = tuple[RationalPoint, RationalPoint, RationalLine]


def _rational_edges(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
) -> tuple[RationalEdge, ...]:
    """Рёбра всех петель в точных дробях: концы и несущая прямая `(a, b, c)`.

    Концы хранятся вместе с прямой, а не отбрасываются: два коллинеарных
    сонаправленных ребра различаются ТОЛЬКО ими, и карте владельцев нужен ключ,
    который их различает. Нормаль смотрит влево от хода, как и в
    `SupportLineV1.through`, но решётка здесь не требуется: сопоставление закона
    с ребром — вопрос коллинеарности, а не целочисленности.
    """

    edges: list[RationalEdge] = []
    for loop in loops:
        size = len(loop)
        for index in range(size):
            start = loop[index]
            end = loop[(index + 1) % size]
            dx, dy = end[0] - start[0], end[1] - start[1]
            if dx == 0 and dy == 0:
                continue
            a, b = -dy, dx
            edges.append((start, end, (a, b, a * start[0] + b * start[1])))
    return tuple(edges)


def _rational_edge_lines(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
) -> tuple[RationalLine, ...]:
    """Только несущие прямые. Оставлено для читателей, которым концы не нужны."""

    return tuple(line for _, _, line in _rational_edges(loops))


def line_class(line: RationalLine) -> RationalLine | None:
    """Канонический представитель класса «одна прямая, одна сторона фронта».

    Класс совпадает у `left` и `right` ТОГДА И ТОЛЬКО ТОГДА, когда
    `_proportional_positive(left, right)`: деление на модуль первой ненулевой
    из `(a, b)` снимает положительный множитель и сохраняет знак. Это не
    ускорение и не приближение, а перевод предиката в ключ словаря — только так
    сопоставление становится группировкой, то есть биекцией по классам, а не
    поиском «первого попавшегося».

    `None` — у закона нулевая нормаль: прямой у него нет вовсе, и ни одному
    ребру он соответствовать не может. Молча такой закон отбрасывать нельзя,
    поэтому ответ отличим от класса, а не подменён нулевым.
    """

    a, b, c = line
    scale = abs(a) if a != 0 else abs(b)
    if scale == 0:
        return None
    return (a / scale, b / scale, c / scale)


def bridge_arrival_laws(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
    laws: tuple[PlainArrivalLawV1, ...],
) -> BridgeReportV1:
    """Собрать вход очереди из домена и законов прихода попарного кроя.

    Первая петля — внешний контур, остальные — дыры. Ориентацию нормирует
    `PolygonV1.build`, поэтому от вызывающего она не требуется.
    """

    findings: list[BridgeOutcome] = []
    off_lattice: list[tuple[Fraction, Fraction]] = []
    for loop in loops:
        for x, y in loop:
            if _as_lattice(x) is None or _as_lattice(y) is None:
                off_lattice.append((x, y))
    if off_lattice:
        findings.append(BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE)
    if not laws:
        findings.append(BridgeOutcome.NO_ARRIVAL_LAW_GIVEN)

    non_unit = tuple(
        (law.name, law.speed_ratio_squared)
        for law in laws
        if not law.is_unit_speed
    )
    if non_unit:
        findings.append(BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED)

    # Несущие прямые рёбер считаются в ТОЧНЫХ ДРОБЯХ, до всякой решётки.
    # Иначе «источник не вся граница» пришлось бы объявлять следствием
    # нецелого домена, а это два независимых расхождения, и мерить их надо
    # по отдельности.
    rational_edges = _rational_edges(loops)

    matching = _match_by_line_class(rational_edges, laws)
    if matching.unmatched:
        findings.append(BridgeOutcome.ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE)
    if matching.surplus:
        findings.append(BridgeOutcome.MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE)
    if matching.undetermined:
        findings.append(
            BridgeOutcome.SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED
        )

    polygon: PolygonV1 | None = None
    owner_by_edge: dict[tuple[int, int, int, int], str] = {}
    lattice_ambiguous: list[tuple[int, int, int, int]] = []
    wall_spans: tuple[tuple[int, int, int, int], ...] = ()
    if (
        not off_lattice
        and loops
        and matching.source_edges
        and not matching.undetermined
    ):
        polygon = _polygon_with_walls(loops, matching.source_edges)
        wall_spans = tuple(
            sorted(
                (start[0], start[1], end[0], end[1])
                for start, end, speed in polygon.edges()
                if speed == 0
            )
        )
        # Ключом служит САМО ребро, взятое из тех же `rational_edges`, что и
        # сопоставление. Прежний код нумеровал рёбра заново по `polygon.loops`
        # и надеялся, что порядок совпадёт, — а `PolygonV1.build` нормирует
        # ориентацию и может РАЗВЕРНУТЬ петлю, после чего номер означал другое
        # ребро. Здесь совпадать нечему: список один.
        for edge, name in matching.forced_owner.items():
            owner_by_edge[_lattice_span(edge)] = name
        lattice_ambiguous = [_lattice_span(edge) for edge in matching.ambiguous]

    ordered = tuple(
        outcome for outcome in _OUTCOME_ORDER if outcome in findings
    )
    return BridgeReportV1(
        outcome=ordered[0] if ordered else BridgeOutcome.EXACT,
        findings=ordered,
        polygon=polygon if not ordered else None,
        edge_count=len(rational_edges),
        law_count=len(laws),
        matched_edge_count=matching.matched_edge_count,
        off_lattice_points=tuple(off_lattice),
        non_unit_speed_laws=non_unit,
        unmatched_laws=tuple(matching.unmatched),
        surplus_laws=tuple(matching.surplus),
        owner_by_edge=tuple(sorted(owner_by_edge.items())),
        ambiguous_owner_spans=tuple(sorted(lattice_ambiguous)),
        wall_spans=wall_spans,
        undetermined_source_count=len(matching.undetermined),
    )


@dataclass(frozen=True, slots=True)
class _MatchingV1:
    """Итог сопоставления законов с рёбрами по классам несущих прямых."""

    matched_edge_count: int
    source_edges: tuple[RationalEdge, ...]
    undetermined: tuple[RationalEdge, ...]
    unmatched: tuple[str, ...]
    surplus: tuple[str, ...]
    forced_owner: dict[RationalEdge, str]
    ambiguous: tuple[RationalEdge, ...]


def _match_by_line_class(
    rational_edges: tuple[RationalEdge, ...],
    laws: tuple[PlainArrivalLawV1, ...],
) -> _MatchingV1:
    """Кто источник, кто стена и что осталось неопределённым — по классам прямых.

    Двудольный граф «закон — ребро» сворачивается в классы несущих прямых:
    внутри класса он полный, между классами рёбер нет. Поэтому наибольшее
    паросочетание считается сложением `min` по классам, и счёт точен — ни
    перебора, ни порога.

    Этап вынесен отдельной функцией не для красоты: `bridge_arrival_laws`
    переросла бюджет функции, а разметка «источник или стена» — самостоятельный
    вопрос со своим ответом, и держать его внутри сборки отчёта означало бы
    мерить его вместе с ней.
    """

    edges_by_class: dict[RationalLine, list[RationalEdge]] = {}
    for edge in rational_edges:
        key = line_class(edge[2])
        if key is not None:
            edges_by_class.setdefault(key, []).append(edge)
    laws_by_class: dict[RationalLine, list[str]] = {}
    unmatched: list[str] = []
    for law in laws:
        key = line_class((law.normal_x, law.normal_y, law.constant))
        if key is None or key not in edges_by_class:
            unmatched.append(law.name)
            continue
        laws_by_class.setdefault(key, []).append(law.name)

    matched_edge_count = 0
    surplus: list[str] = []
    forced_owner: dict[RationalEdge, str] = {}
    ambiguous: list[RationalEdge] = []
    source_edges: list[RationalEdge] = []
    undetermined: list[RationalEdge] = []
    for key, names in laws_by_class.items():
        block = edges_by_class[key]
        matched_edge_count += min(len(names), len(block))
        if len(names) > len(block):
            surplus.extend(names[len(block):])
        if len(names) < len(block):
            # Класс покрыт ЧАСТИЧНО: источником в нём является не всякое ребро,
            # а какое именно — во входе не записано. Это не «источник не вся
            # граница» (та задача решена стенами), а неопределённость ВНУТРИ
            # класса, и она называется отдельно.
            undetermined.extend(block)
        else:
            source_edges.extend(block)
        if len(names) == 1 and len(block) == 1:
            # Пара ВЫНУЖДЕНА: класс из одного закона и одного ребра.
            forced_owner[block[0]] = names[0]
        else:
            # Класс, где рёбер несколько: закон протяжённости не несёт, значит
            # чьё ребро чей — во входе не записано. Перечисляем, а не гадаем.
            ambiguous.extend(block[: min(len(names), len(block))])
    return _MatchingV1(
        matched_edge_count=matched_edge_count,
        source_edges=tuple(source_edges),
        undetermined=tuple(undetermined),
        unmatched=tuple(unmatched),
        surplus=tuple(surplus),
        forced_owner=forced_owner,
        ambiguous=tuple(ambiguous),
    )


def _polygon_with_walls(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
    source_edges: tuple[RationalEdge, ...] | list[RationalEdge],
) -> PolygonV1:
    """Полигон, где рёбра с законом — источники, а ОСТАЛЬНЫЕ — стены.

    Пометка ставится по концам ребра, а не по его номеру: `PolygonV1.build`
    нормирует ориентацию и может развернуть петлю, после чего номер означал бы
    другое ребро. Ровно на этом уже обжигались (`DECISIONS.md`, 2026-07-27),
    поэтому здесь номера не участвуют вовсе.
    """

    polygon = PolygonV1.build(
        LoopV1(tuple((int(x), int(y)) for x, y in loops[0])),
        tuple(
            LoopV1(tuple((int(x), int(y)) for x, y in loop))
            for loop in loops[1:]
        ),
    )
    spans = tuple(
        ((int(start[0]), int(start[1])), (int(end[0]), int(end[1])))
        for start, end, _ in source_edges
    )
    return with_source_spans(polygon, spans)


def unit_speed_laws_of(polygon: PolygonV1) -> tuple[PlainArrivalLawV1, ...]:
    """Законы прихода рёбер-ИСТОЧНИКОВ контура, с единичной скоростью.

    Обратное направление моста, и оно нужно стенду: чтобы сравнить два пути на
    одном входе, попарному крою надо выдать ровно те законы, которые очередь
    видит у себя. `s^2 = q = |d|^2` — то есть скорость ровно единичная.

    У стены закона НЕТ, и это не пропуск, а само определение стены: фронт от
    неё не идёт, значит и закона прихода у неё быть не может. Отсюда замыкание
    моста в обе стороны: `bridge_arrival_laws(loops, unit_speed_laws_of(p))`
    возвращает полигон с ТОЙ ЖЕ разметкой стен, что у `p`.
    """

    laws: list[PlainArrivalLawV1] = []
    for loop_index, loop in enumerate(polygon.loops):
        points = loop.points
        speeds = loop.edge_speeds_squared
        for index in range(len(points)):
            if speeds[index] == 0:
                continue
            start = points[index]
            end = points[(index + 1) % len(points)]
            line = SupportLineV1.with_speed(start, end, speeds[index])
            laws.append(
                PlainArrivalLawV1(
                    name=f"loop{loop_index}-edge{index}",
                    normal_x=Fraction(line.a),
                    normal_y=Fraction(line.b),
                    constant=Fraction(line.c),
                    speed_squared=Fraction(line.q),
                )
            )
    return tuple(laws)
