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
3. `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY` — очередь пускает фронт от КАЖДОГО ребра
   контура, а покрытие растёт только от выбранных цепочек. Неподвижная стена
   представима (`SupportLineV1` с `q = 0`), но `PolygonV1` её не выражает.
4. `ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE` — закон, чья прямая не является несущей
   прямой ни одного ребра домена. Такому источнику ребра контура не
   соответствует вовсе.

Конечность alpha в исходы не входит: она отображается усечением по времени и
разбирается в стенде, а не здесь.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .event_time import SupportLineV1
from .polygon import LoopV1, PolygonV1


class BridgeOutcome(str, Enum):
    """Именованные исходы моста. Тихого «не получилось» здесь нет."""

    EXACT = "EXACT"
    NO_ARRIVAL_LAW_GIVEN = "NO_ARRIVAL_LAW_GIVEN"
    DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE = (
        "DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE"
    )
    ARRIVAL_LAW_IS_NOT_UNIT_SPEED = "ARRIVAL_LAW_IS_NOT_UNIT_SPEED"
    ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE = "ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE"
    SOURCE_IS_NOT_THE_WHOLE_BOUNDARY = "SOURCE_IS_NOT_THE_WHOLE_BOUNDARY"


# Порядок объявлен, а не выведен из порядка проверок: чем ниже, тем глубже
# причина. `outcome` берёт первый применимый, `findings` перечисляет ВСЕ.
_OUTCOME_ORDER = (
    BridgeOutcome.NO_ARRIVAL_LAW_GIVEN,
    BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE,
    BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,
    BridgeOutcome.ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE,
    BridgeOutcome.SOURCE_IS_NOT_THE_WHOLE_BOUNDARY,
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
    owner_by_edge: tuple[tuple[tuple[int, int, int, int], str], ...]

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


def _rational_edge_lines(
    loops: tuple[tuple[tuple[Fraction, Fraction], ...], ...],
) -> tuple[tuple[Fraction, Fraction, Fraction], ...]:
    """Несущие прямые всех рёбер петель в точных дробях: `(a, b, c)`.

    Нормаль смотрит влево от хода, как и в `SupportLineV1.through`, но решётка
    здесь не требуется: сопоставление закона с ребром — вопрос коллинеарности,
    а не целочисленности.
    """

    lines: list[tuple[Fraction, Fraction, Fraction]] = []
    for loop in loops:
        size = len(loop)
        for index in range(size):
            x0, y0 = loop[index]
            x1, y1 = loop[(index + 1) % size]
            dx, dy = x1 - x0, y1 - y0
            if dx == 0 and dy == 0:
                continue
            a, b = -dy, dx
            lines.append((a, b, a * x0 + b * y0))
    return tuple(lines)


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
    rational_edges = _rational_edge_lines(loops)
    owner_by_index: dict[int, str] = {}
    unmatched: list[str] = []
    for law in laws:
        target = (law.normal_x, law.normal_y, law.constant)
        match = next(
            (
                index
                for index, line in enumerate(rational_edges)
                if _proportional_positive(line, target)
            ),
            None,
        )
        if match is None:
            unmatched.append(law.name)
            continue
        owner_by_index[match] = law.name
    if unmatched:
        findings.append(BridgeOutcome.ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE)
    if laws and len(owner_by_index) != len(rational_edges):
        findings.append(BridgeOutcome.SOURCE_IS_NOT_THE_WHOLE_BOUNDARY)

    polygon: PolygonV1 | None = None
    owner_by_edge: dict[tuple[int, int, int, int], str] = {}
    if not off_lattice and loops:
        polygon = PolygonV1.build(
            LoopV1(tuple((int(x), int(y)) for x, y in loops[0])),
            tuple(
                LoopV1(tuple((int(x), int(y)) for x, y in loop))
                for loop in loops[1:]
            ),
        )
        position = 0
        for loop in polygon.loops:
            points = loop.points
            for index in range(len(points)):
                line = SupportLineV1.through(
                    points[index], points[(index + 1) % len(points)]
                )
                name = owner_by_index.get(position)
                if name is not None:
                    owner_by_edge[(line.a, line.b, line.c, line.q)] = name
                position += 1

    ordered = tuple(
        outcome for outcome in _OUTCOME_ORDER if outcome in findings
    )
    return BridgeReportV1(
        outcome=ordered[0] if ordered else BridgeOutcome.EXACT,
        findings=ordered,
        polygon=polygon if not ordered else None,
        edge_count=len(rational_edges),
        law_count=len(laws),
        matched_edge_count=len(owner_by_index),
        off_lattice_points=tuple(off_lattice),
        non_unit_speed_laws=non_unit,
        unmatched_laws=tuple(unmatched),
        owner_by_edge=tuple(sorted(owner_by_edge.items())),
    )


def unit_speed_laws_of(polygon: PolygonV1) -> tuple[PlainArrivalLawV1, ...]:
    """Законы прихода, отвечающие КАЖДОМУ ребру контура, с единичной скоростью.

    Обратное направление моста, и оно нужно стенду: чтобы сравнить два пути на
    одном входе, попарному крою надо выдать ровно те законы, которые очередь
    видит у себя. `s^2 = q = |d|^2` — то есть скорость ровно единичная.
    """

    laws: list[PlainArrivalLawV1] = []
    for loop_index, loop in enumerate(polygon.loops):
        points = loop.points
        for index in range(len(points)):
            start = points[index]
            end = points[(index + 1) % len(points)]
            line = SupportLineV1.through(start, end)
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
