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

from dataclasses import dataclass, field
from enum import Enum
from fractions import Fraction

from ..robust.grid import GridSpecV1, snap_value
from .event_time import SupportLineV1, normalized_speed
from .polygon import (
    LoopV1,
    PolygonRejected,
    PolygonV1,
    with_edge_speeds,
)


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
    # Привязка склеила два конца ребра в один узел. Ребро при этом ИСЧЕЗАЕТ, а
    # вместе с ним и его закон, поэтому исход отдельный: `PolygonV1` сказал бы
    # `LOOP_HAS_REPEATED_POINT` — правду про петлю и не про причину.
    LATTICE_SNAP_COLLAPSES_A_DOMAIN_EDGE = (
        "LATTICE_SNAP_COLLAPSES_A_DOMAIN_EDGE"
    )
    # Привязанный домен `PolygonV1` не принял. Отказ полигона пересказывается
    # своим именем ровно потому, что вершины двигал МОСТ: назвать это отказом
    # входа значило бы свалить свою работу на вызывающего.
    LATTICE_SNAP_BREAKS_THE_DOMAIN_POLYGON = (
        "LATTICE_SNAP_BREAKS_THE_DOMAIN_POLYGON"
    )
    # В одном классе несущей прямой законы РАЗНЫХ скоростей, а рёбер больше
    # одного. Какому ребру какая скорость — во входе не записано, и здесь это
    # тяжелее, чем неопределённость владельца: у владельца хотя бы геометрия
    # одна, а разные скорости дают разную геометрию.
    EDGE_SPEED_INSIDE_A_LINE_CLASS_IS_UNDETERMINED = (
        "EDGE_SPEED_INSIDE_A_LINE_CLASS_IS_UNDETERMINED"
    )


# Порядок объявлен, а не выведен из порядка проверок: чем ниже, тем глубже
# причина. `outcome` берёт первый применимый, `findings` перечисляет ВСЕ.
_OUTCOME_ORDER = (
    BridgeOutcome.NO_ARRIVAL_LAW_GIVEN,
    BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE,
    BridgeOutcome.LATTICE_SNAP_COLLAPSES_A_DOMAIN_EDGE,
    BridgeOutcome.LATTICE_SNAP_BREAKS_THE_DOMAIN_POLYGON,
    BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,
    BridgeOutcome.ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE,
    BridgeOutcome.MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE,
    BridgeOutcome.SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED,
    BridgeOutcome.EDGE_SPEED_INSIDE_A_LINE_CLASS_IS_UNDETERMINED,
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
    # Масштаб решётки, к которой мост ПРИВЯЗАЛ домен. `None` — домен лежал в
    # узлах сам, и мост не двигал ни одной вершины.
    lattice_scale: int | None = None
    # Наибольшее смещение вершины при привязке, точной дробью. Ноль означает
    # «привязка ничего не сдвинула», `None` — «привязки не было вовсе»; путать
    # эти два ответа нельзя, потому что первый — измерение, а второй — его
    # отсутствие.
    snap_residual: Fraction | None = None
    # Рёбра-источники, чья скорость НЕ единичная. Их `q` рационально, а не
    # `|d|^2`, и это ровно та величина, ради которой снималась единичность.
    weighted_edge_count: int = 0

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


LatticeLoops = tuple[tuple[tuple[int, int], ...], ...]


def _lattice_span(
    edge: "RationalEdge", lattice_loops: LatticeLoops
) -> tuple[int, int, int, int]:
    """Вхождение ребра узлами решётки: `(x0, y0, x1, y1)`.

    Берётся не из самого ребра, а из его образа В РЕШЁТКЕ по ПОЛОЖЕНИЮ, потому
    что при привязке образ уже не равен оригиналу. Когда домен и так на решётке,
    образ ей тождественно равен, и число выходит то же самое.
    """

    loop = lattice_loops[edge.loop_index]
    x0, y0 = loop[edge.edge_index]
    x1, y1 = loop[(edge.edge_index + 1) % len(loop)]
    return (x0, y0, x1, y1)


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


@dataclass(frozen=True, slots=True)
class RationalEdge:
    """Ребро домена: где оно в петлях, два его конца и его несущая прямая.

    ПОЛОЖЕНИЕ (`loop_index`, `edge_index`) хранится вместе с геометрией, и это
    не удобство. Привязка домена к решётке НЕ СОХРАНЯЕТ классы несущих прямых —
    на `bf6` совпало 0 классов из 12 на всех проверенных масштабах, — поэтому
    сопоставление закона с ребром обязано идти по дорешёточным дробям, а
    разметка переноситься на привязанный полигон по положению ребра в петле.
    Другого моста между двумя видами одного ребра нет: по прямой их уже не
    узнать, а по концам — тем более.
    """

    loop_index: int
    edge_index: int
    start: RationalPoint
    end: RationalPoint
    line: RationalLine


def _rational_edges(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
) -> tuple[RationalEdge, ...]:
    """Рёбра всех петель в точных дробях: положение, концы и прямая `(a, b, c)`.

    Концы хранятся вместе с прямой, а не отбрасываются: два коллинеарных
    сонаправленных ребра различаются ТОЛЬКО ими, и карте владельцев нужен ключ,
    который их различает. Нормаль смотрит влево от хода, как и в
    `SupportLineV1.through`, но решётка здесь не требуется: сопоставление закона
    с ребром — вопрос коллинеарности, а не целочисленности.
    """

    edges: list[RationalEdge] = []
    for loop_index, loop in enumerate(loops):
        size = len(loop)
        for index in range(size):
            start = loop[index]
            end = loop[(index + 1) % size]
            dx, dy = end[0] - start[0], end[1] - start[1]
            if dx == 0 and dy == 0:
                continue
            a, b = -dy, dx
            edges.append(
                RationalEdge(
                    loop_index,
                    index,
                    start,
                    end,
                    (a, b, a * start[0] + b * start[1]),
                )
            )
    return tuple(edges)


def _rational_edge_lines(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
) -> tuple[RationalLine, ...]:
    """Только несущие прямые. Оставлено для читателей, которым концы не нужны."""

    return tuple(edge.line for edge in _rational_edges(loops))


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
    *,
    lattice: GridSpecV1 | None = None,
    weighted_fronts: bool = False,
) -> BridgeReportV1:
    """Собрать вход очереди из домена и законов прихода попарного кроя.

    Первая петля — внешний контур, остальные — дыры. Ориентацию нормирует
    `PolygonV1.build`, поэтому от вызывающего она не требуется.

    ОБА РАСШИРЕНИЯ ВЫКЛЮЧЕНЫ ПО УМОЛЧАНИЮ, и это не осторожность, а разница в
    том, кто отвечает за ответ.

    `lattice` — решётка, к которой мост ПРИВЯЖЕТ домен. Привязка двигает
    вершины, то есть меняет задачу, и делать это молча нельзя: без параметра
    нецелый домен остаётся находкой `DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE`, ровно
    как раньше. С параметром находки нет, а есть измеренная цена —
    `snap_residual`, наибольшее смещение вершины точной дробью.

    `weighted_fronts` — согласие вызывающего на ВЗВЕШЕННЫЙ скелет. Геометрия
    очереди его держит, арифметика тоже (`q` рационально), но времена очереди
    тогда совпадают с alpha закона лишь потому, что `q = (lambda*s)^2`, а не
    потому, что скорость единичная, — и вызывающему, который сравнивает очередь
    с попарным кроем, разницу надо знать. Без параметра неединичный закон
    остаётся находкой `ARRIVAL_LAW_IS_NOT_UNIT_SPEED`.

    Сопоставление закона с ребром идёт по ДОРЕШЁТОЧНЫМ дробям при любом
    значении `lattice`, и это измерено, а не выбрано: после привязки `bf6`
    совпало 0 классов несущих прямых из 12. Разметка переносится на привязанный
    полигон по ПОЛОЖЕНИЮ ребра в петле — единственному, что привязка сохраняет.
    """

    findings: list[BridgeOutcome] = []
    lattice_loops, off_lattice, residual = _lattice_image(loops, lattice)
    if lattice_loops is None:
        findings.append(BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE)
    elif _collapsed_edges(loops, lattice_loops):
        findings.append(BridgeOutcome.LATTICE_SNAP_COLLAPSES_A_DOMAIN_EDGE)
        lattice_loops = None
    if not laws:
        findings.append(BridgeOutcome.NO_ARRIVAL_LAW_GIVEN)

    non_unit = tuple(
        (law.name, law.speed_ratio_squared)
        for law in laws
        if not law.is_unit_speed
    )
    if non_unit and not weighted_fronts:
        findings.append(BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED)

    # Несущие прямые рёбер считаются в ТОЧНЫХ ДРОБЯХ, до всякой решётки.
    # Иначе «источник не вся граница» пришлось бы объявлять следствием
    # нецелого домена, а это два независимых расхождения, и мерить их надо
    # по отдельности.
    rational_edges = _rational_edges(loops)

    matching = _match_by_line_class(rational_edges, laws)
    for present, outcome in (
        (matching.unmatched, BridgeOutcome.ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE),
        (
            matching.surplus,
            BridgeOutcome.MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE,
        ),
        (
            matching.undetermined,
            BridgeOutcome.SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED,
        ),
        (
            matching.undetermined_speed,
            BridgeOutcome.EDGE_SPEED_INSIDE_A_LINE_CLASS_IS_UNDETERMINED,
        ),
    ):
        if present:
            findings.append(outcome)

    built = _built_polygon(loops, lattice_loops, matching, findings, lattice)
    ordered = tuple(
        outcome for outcome in _OUTCOME_ORDER if outcome in findings
    )
    return BridgeReportV1(
        outcome=ordered[0] if ordered else BridgeOutcome.EXACT,
        findings=ordered,
        polygon=built.polygon if not ordered else None,
        edge_count=len(rational_edges),
        law_count=len(laws),
        matched_edge_count=matching.matched_edge_count,
        off_lattice_points=off_lattice,
        non_unit_speed_laws=non_unit,
        unmatched_laws=tuple(matching.unmatched),
        surplus_laws=tuple(matching.surplus),
        owner_by_edge=tuple(sorted(built.owner_by_edge.items())),
        ambiguous_owner_spans=tuple(sorted(built.ambiguous_spans)),
        wall_spans=built.wall_spans,
        undetermined_source_count=len(matching.undetermined),
        lattice_scale=None if lattice is None else lattice.scale,
        snap_residual=residual,
        weighted_edge_count=built.weighted_edge_count,
    )


@dataclass(frozen=True, slots=True)
class _BuiltPolygonV1:
    """Полигон очереди и всё, что известно про его рёбра. Пустой — тоже ответ."""

    polygon: PolygonV1 | None = None
    owner_by_edge: dict[tuple[int, int, int, int], str] = field(
        default_factory=dict
    )
    ambiguous_spans: tuple[tuple[int, int, int, int], ...] = ()
    wall_spans: tuple[tuple[int, int, int, int], ...] = ()
    weighted_edge_count: int = 0


def _lattice_image(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
    lattice: GridSpecV1 | None,
) -> tuple[
    LatticeLoops | None, tuple[tuple[Fraction, Fraction], ...], Fraction | None
]:
    """Образ домена в узлах решётки, вершины вне узлов и цена привязки.

    Два режима, и они отвечают на разные вопросы. Без решётки спрашивается «а
    домен и так целый?» — и ответ «нет» перечисляет ИМЕННО ТЕ вершины, которые
    целыми не оказались. С решёткой домен привязывается, вершин вне узлов не
    остаётся по построению, а вместо них возвращается наибольшее смещение —
    цена, которую привязка взяла.

    Ноль в `snap_residual` и `None` — разные ответы: первый значит «привязка
    была и ничего не сдвинула», второй — «привязки не было». Свести их к одному
    значило бы потерять единственное свидетельство того, что мост домен не
    трогал.
    """

    if lattice is None:
        off_lattice = tuple(
            (x, y)
            for loop in loops
            for x, y in loop
            if _as_lattice(x) is None or _as_lattice(y) is None
        )
        if off_lattice:
            return None, off_lattice, None
        image = tuple(
            tuple((int(x), int(y)) for x, y in loop) for loop in loops
        )
        return image, (), None
    image = tuple(
        tuple(
            (snap_value(x, lattice), snap_value(y, lattice)) for x, y in loop
        )
        for loop in loops
    )
    residual = max(
        (
            abs(Fraction(node, lattice.scale) - value)
            for loop, snapped in zip(loops, image)
            for point, node_pair in zip(loop, snapped)
            for value, node in zip(point, node_pair)
        ),
        default=Fraction(0),
    )
    return image, (), residual


def _collapsed_edges(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
    lattice_loops: LatticeLoops,
) -> tuple[tuple[int, int], ...]:
    """Рёбра, у которых привязка склеила оба конца в один узел.

    Проверяется до `PolygonV1`, потому что полигон сказал бы про ПЕТЛЮ
    («повторившаяся точка»), а причина — в ребре: закон, привязанный к
    исчезнувшему ребру, исчез бы вместе с ним и молча.
    """

    collapsed: list[tuple[int, int]] = []
    for loop_index, (loop, snapped) in enumerate(zip(loops, lattice_loops)):
        size = len(loop)
        for index in range(size):
            following = (index + 1) % size
            if loop[index] == loop[following]:
                continue
            if snapped[index] == snapped[following]:
                collapsed.append((loop_index, index))
    return tuple(collapsed)


def _built_polygon(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
    lattice_loops: LatticeLoops | None,
    matching: "_MatchingV1",
    findings: list[BridgeOutcome],
    lattice: GridSpecV1 | None,
) -> _BuiltPolygonV1:
    """Полигон очереди из привязанного домена и разметки по классам прямых.

    `q` источника считается ОДНОЙ формулой на все три случая: `q = ratio *
    |d|^2`, где `ratio = (s/|n|)^2` закона. У единичного закона `ratio = 1`, и
    число выходит ровно то же `|d|^2`, которое клал прежний путь, — поэтому
    ветки «взвешенный или нет» здесь нет вовсе и разойтись ей негде.
    """

    if (
        lattice_loops is None
        or not loops
        or not matching.source_edges
        or matching.undetermined
        or matching.undetermined_speed
    ):
        return _BuiltPolygonV1()
    speeds = tuple(
        _weighted_span(edge, lattice_loops, matching.speed_ratio[edge])
        for edge in matching.source_edges
    )
    try:
        polygon = with_edge_speeds(
            PolygonV1.build(
                LoopV1(lattice_loops[0]),
                tuple(LoopV1(loop) for loop in lattice_loops[1:]),
            ),
            speeds,
        )
    except PolygonRejected as refusal:
        if lattice is None:
            # Вершины двигал не мост — отказ полигона принадлежит вызывающему
            # и пересказу своими словами не подлежит.
            raise
        findings.append(BridgeOutcome.LATTICE_SNAP_BREAKS_THE_DOMAIN_POLYGON)
        _ = refusal
        return _BuiltPolygonV1()
    return _BuiltPolygonV1(
        polygon=polygon,
        owner_by_edge={
            _lattice_span(edge, lattice_loops): name
            for edge, name in matching.forced_owner.items()
        },
        ambiguous_spans=tuple(
            _lattice_span(edge, lattice_loops) for edge in matching.ambiguous
        ),
        wall_spans=tuple(
            sorted(
                (start[0], start[1], end[0], end[1])
                for start, end, speed in polygon.edges()
                if speed == 0
            )
        ),
        # Считается по ОТНОШЕНИЮ скоростей, а не по сравнению `q` с `|d|^2`:
        # второе повторило бы формулу, по которой `q` только что посчитано, и
        # проверяло бы арифметику вместо входа.
        weighted_edge_count=sum(
            1 for edge in matching.source_edges if matching.speed_ratio[edge] != 1
        ),
    )


def _weighted_span(
    edge: RationalEdge, lattice_loops: LatticeLoops, ratio: Fraction
) -> tuple[tuple[int, int], tuple[int, int], int | Fraction]:
    """Ребро решётки со своим `q = ratio * |d|^2`, уже нормированным."""

    x0, y0, x1, y1 = _lattice_span(edge, lattice_loops)
    dx, dy = x1 - x0, y1 - y0
    return (x0, y0), (x1, y1), normalized_speed(ratio * (dx * dx + dy * dy))


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
    #: `(s/|n|)^2` каждого ребра-источника. Есть у ВСЕХ `source_edges`, потому
    #: что скорость определена классом, а не владельцем: чьё именно ребро чей
    #: закон, знать не обязательно, пока все законы класса идут одинаково.
    speed_ratio: dict[RationalEdge, Fraction] = field(default_factory=dict)
    #: Рёбра классов, где законы идут РАЗНЫМИ скоростями, а рёбер больше одного.
    #: Тяжелее неопределённости владельца: там геометрия одна на всех кандидатов,
    #: здесь у каждой скорости своя.
    undetermined_speed: tuple[RationalEdge, ...] = ()


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
        key = line_class(edge.line)
        if key is not None:
            edges_by_class.setdefault(key, []).append(edge)
    laws_by_class: dict[RationalLine, list[PlainArrivalLawV1]] = {}
    unmatched: list[str] = []
    for law in laws:
        key = line_class((law.normal_x, law.normal_y, law.constant))
        if key is None or key not in edges_by_class:
            unmatched.append(law.name)
            continue
        laws_by_class.setdefault(key, []).append(law)

    matched_edge_count = 0
    surplus: list[str] = []
    forced_owner: dict[RationalEdge, str] = {}
    ambiguous: list[RationalEdge] = []
    source_edges: list[RationalEdge] = []
    undetermined: list[RationalEdge] = []
    undetermined_speed: list[RationalEdge] = []
    speed_ratio: dict[RationalEdge, Fraction] = {}
    for key, block_laws in laws_by_class.items():
        names = [law.name for law in block_laws]
        block = edges_by_class[key]
        matched_edge_count += min(len(names), len(block))
        if len(names) > len(block):
            surplus.extend(names[len(block):])
        ratios = {law.speed_ratio_squared for law in block_laws}
        if len(names) < len(block):
            # Класс покрыт ЧАСТИЧНО: источником в нём является не всякое ребро,
            # а какое именно — во входе не записано. Это не «источник не вся
            # граница» (та задача решена стенами), а неопределённость ВНУТРИ
            # класса, и она называется отдельно.
            undetermined.extend(block)
        elif len(ratios) > 1 and len(block) > 1:
            # Скорости в классе разные, а рёбер несколько: какому ребру какая —
            # во входе не записано. Владельца тут не хватило бы и подавно, но
            # разница в том, что неизвестное имя оставляет геометрию одной, а
            # неизвестная скорость делает её другой.
            undetermined_speed.extend(block)
        else:
            source_edges.extend(block)
            for edge in block:
                speed_ratio[edge] = next(iter(ratios))
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
        speed_ratio=speed_ratio,
        undetermined_speed=tuple(undetermined_speed),
    )


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
