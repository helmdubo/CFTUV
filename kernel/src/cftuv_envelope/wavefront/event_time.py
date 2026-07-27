"""Время и место события волнового фронта — точно и без извлечения корня.

Вывод, на котором держится весь модуль. Ребро `i` целочисленной решётки с
направлением `d_i` задаёт несущую прямую `a_i*x + b_i*y = c_i` с ЦЕЛЫМИ
`a_i = -d_iy`, `b_i = d_ix`, `c_i = a_i*x_i + b_i*y_i`. Фронт идёт с единичной
скоростью по нормали, а нормаль здесь не единичная, поэтому прямая в момент t:

    a_i*x + b_i*y = c_i + t*sqrt(q_i),   q_i = |d_i|^2  (целое)

Событие — это момент, когда ТРИ такие прямые сходятся в одной точке, то есть
обращается в ноль определитель

    | a_i  b_i  c_i + t*sqrt(q_i) |
    | a_j  b_j  c_j + t*sqrt(q_j) | = D0 + t*S
    | a_k  b_k  c_k + t*sqrt(q_k) |

Он АФФИНЕН по t: `D0` целое, `S = C_i*sqrt(q_i) + C_j*sqrt(q_j) + C_k*sqrt(q_k)`
с целыми алгебраическими дополнениями `C`. Отсюда `t = -D0/S`, и это единая
формула и для edge-, и для split-события: разница между ними в том, КАКИЕ три
ребра берутся, а не в арифметике.

Практическое следствие, ради которого всё сделано так: **время не вычисляется,
хранится пара (целое, SqrtSum)**, а сравнение двух времён

    sign(t1 - t2) = sign(d1*S2 - d2*S1),  когда S1 > 0 и S2 > 0

не требует ни деления, ни корня — только линейную комбинацию двух `SqrtSum`.
РАВЕНСТВО времён при этом чисто рационально: разность обращается в ноль тогда и
только тогда, когда пусты её коэффициенты. Это и есть точная одновременность,
которой у Kendzi служит `SplitEpsilon = 1e-10`, а у Трики — решётка 1 мм.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .sqrt_sum import SqrtSumV1


class DegenerateEdgeError(ValueError):
    """Ребро нулевой длины. Прямой у него нет, и придумывать её нельзя."""


class ZeroDivisorTimeError(ZeroDivisionError):
    """Время с нулевым знаменателем. Не бывает после `concurrency_time`."""


class ParallelSupportLinesError(ValueError):
    """Точку пересечения параллельных прямых придумывать нельзя."""


class EventTimeOutcome(str, Enum):
    """Почему тройка рёбер не даёт события. Тихого возврата здесь нет."""

    EXACT = "EXACT"
    # Определитель не зависит от t: три прямые движутся так, что их
    # конфигурация не меняется (параллельные фронты). События нет никогда.
    WAVEFRONT_TRIPLE_NEVER_CONCURRENT = "WAVEFRONT_TRIPLE_NEVER_CONCURRENT"
    # Прямые уже сошлись и остаются сошедшимися: D0 = 0 и S = 0.
    WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT = "WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT"


@dataclass(frozen=True, slots=True)
class SupportLineV1:
    """Несущая прямая ребра фронта: `a*x + b*y = c + t*sqrt(q)`.

    Все четыре числа целые. Это не удобство, а условие задачи: пока они целые,
    каждый предикат ниже — целочисленный определитель.
    """

    a: int
    b: int
    c: int
    q: int

    @staticmethod
    def through(start: tuple[int, int], end: tuple[int, int]) -> "SupportLineV1":
        """Прямая ребра `start -> end` с нормалью, смотрящей ВЛЕВО от хода.

        Для контура, обойдённого против часовой стрелки, «влево» — внутрь,
        то есть в ту сторону, куда идёт фронт.
        """

        dx, dy = end[0] - start[0], end[1] - start[1]
        if dx == 0 and dy == 0:
            raise DegenerateEdgeError(f"ребро нулевой длины в {start}")
        a, b = -dy, dx
        return SupportLineV1(a, b, a * start[0] + b * start[1], dx * dx + dy * dy)


@dataclass(frozen=True, slots=True)
class EventTimeV1:
    """`t = dividend / divisor`, знаменатель нормирован положительным.

    Значение НЕ вычисляется. Нормировка знака знаменателя нужна, чтобы знак
    времени читался прямо из числителя, а сравнение не тащило знак делителя.
    """

    dividend: Fraction
    divisor: SqrtSumV1

    @staticmethod
    def normalized(dividend: Fraction | int, divisor: SqrtSumV1) -> "EventTimeV1":
        dividend = Fraction(dividend)
        sign = divisor.sign()
        if sign == 0:
            raise ZeroDivisorTimeError("знаменатель времени доказанно нулевой")
        if sign < 0:
            dividend, divisor = -dividend, -divisor
        return EventTimeV1(dividend, divisor)

    @property
    def sign(self) -> int:
        return (self.dividend > 0) - (self.dividend < 0)

    def canonical(self) -> "EventTimeV1":
        """Единственное представление данного значения времени.

        Одно и то же время приходит из разных троек рёбер с разными парами
        `(dividend, divisor)`, и они пропорциональны: `D1/S1 = D2/S2` влечёт
        `S2 = (D2/D1)*S1`. Деление обеих частей на модуль коэффициента при
        наименьшем радикале снимает этот произвол, поэтому дайджест перестаёт
        зависеть от того, каким событием время было найдено.
        """

        if self.dividend == 0:
            return EventTimeV1(Fraction(0), SqrtSumV1.rational(1))
        scale = abs(self.divisor.terms[0][1])
        factor = Fraction(1) / scale
        return EventTimeV1(self.dividend * factor, self.divisor.scaled(factor))

    def enclosure(self, bits: int = 64) -> tuple[Fraction, Fraction]:
        """Оболочка значения. Только для отчётов и стендов, не для решений."""

        low, high = self.divisor.enclosure(bits)
        if low <= 0 <= high:
            raise ZeroDivisorTimeError("оболочка знаменателя накрывает ноль")
        candidates = (self.dividend / low, self.dividend / high)
        return min(candidates), max(candidates)


ZERO_TIME = EventTimeV1(Fraction(0), SqrtSumV1.rational(1))


def compare_times(left: EventTimeV1, right: EventTimeV1) -> int:
    """Точный знак `left - right`. Ни деления, ни корня, ни порога.

    Знаменатели положительны по построению, поэтому знак разности равен знаку
    `d1*S2 - d2*S1`. Ноль этой величины — рациональная проверка: у `SqrtSumV1`
    представление каноническое, и пустой набор коэффициентов означает ТОЧНОЕ
    равенство времён, а не «достаточно близко».
    """

    difference = right.divisor.scaled(left.dividend) - left.divisor.scaled(
        right.dividend
    )
    return difference.sign()


def times_are_equal(left: EventTimeV1, right: EventTimeV1) -> bool:
    """Точная одновременность — чисто рациональная проверка."""

    difference = right.divisor.scaled(left.dividend) - left.divisor.scaled(
        right.dividend
    )
    return difference.is_zero


def concurrency_time(
    first: SupportLineV1, second: SupportLineV1, third: SupportLineV1
) -> tuple[EventTimeV1 | None, EventTimeOutcome]:
    """Момент, когда три движущиеся прямые сходятся в одной точке.

    Единственная формула события: edge-событие берёт три подряд идущих ребра
    фронта, split-событие — два ребра при reflex-вершине и одно далёкое. Ответ
    может быть отрицательным: «событие в прошлом» — тоже ответ, и отбрасывает
    его вызывающий, а не эта функция.
    """

    cofactor_first = second.a * third.b - third.a * second.b
    cofactor_second = third.a * first.b - first.a * third.b
    cofactor_third = first.a * second.b - second.a * first.b
    offset = (
        first.c * cofactor_first
        + second.c * cofactor_second
        + third.c * cofactor_third
    )
    speed = (
        SqrtSumV1.radical(cofactor_first, first.q)
        + SqrtSumV1.radical(cofactor_second, second.q)
        + SqrtSumV1.radical(cofactor_third, third.q)
    )
    if speed.is_zero:
        if offset == 0:
            return None, EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT
        return None, EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT
    return EventTimeV1.normalized(-offset, speed), EventTimeOutcome.EXACT


def sliding_point(
    line: SupportLineV1, along: SqrtSumV1, time: EventTimeV1
) -> "EventPointV1":
    """Точка на движущейся прямой с ЗАКРЕПЛЁННОЙ проекцией вдоль неё.

    Зачем это нужно. У вершины фронта, чьи соседние рёбра лежат на одной
    движущейся прямой и смотрят в одну сторону, угол ровно развёрнутый.
    Биссектриса развёрнутого угла перпендикулярна обеим сторонам, поэтому такая
    вершина идёт ПЕРПЕНДИКУЛЯРНО прямой, а её координата ВДОЛЬ прямой не
    меняется. Пересечения двух прямых у неё нет — и придумывать его не надо:
    вершина задана прямой и одним сохраняющимся числом.

    Решается система (обе строки точные, деления на корень нет):

        a*x + b*y = c + t*sqrt(q)      — вершина на прямой
        b*x - a*y = along             — проекция вдоль прямой сохраняется

    Определитель системы равен `a^2 + b^2 = q`, и он не ноль ни для одного
    невырожденного ребра, поэтому решение всегда есть и оно единственно.
    """

    moving = SqrtSumV1.rational(line.c) + (
        SqrtSumV1.radical(time.dividend, line.q) / time.divisor
    )
    scale = Fraction(1, line.q)
    x = (moving.scaled(line.a) + along.scaled(line.b)).scaled(scale)
    y = (moving.scaled(line.b) - along.scaled(line.a)).scaled(scale)
    return EventPointV1(x, y)


def sliding_time(
    line: SupportLineV1, along: SqrtSumV1, other: SupportLineV1
) -> tuple[EventTimeV1 | None, EventTimeOutcome]:
    """Когда скользящая точка `line`/`along` окажется на прямой `other`.

    Подстановка решения из `sliding_point` в `a2*x + b2*y = c2 + t*sqrt(q2)`
    даёт уравнение, АФФИННОЕ по t, ровно как у тройки прямых:

        t * (K*sqrt(q1) - q1*sqrt(q2)) = q1*c2 + D*along - K*c1

    где `K = a1*a2 + b1*b2` и `D = a1*b2 - a2*b1` — целые. Отличие от
    `concurrency_time` только одно, и оно принципиальное: числитель здесь
    ИРРАЦИОНАЛЕН (в нём `along`), поэтому ответ хранится как `1/(S/N)`, а не как
    `-D0/S`. Значение то же самое, форма другая: `EventTimeV1` требует
    рационального числителя, и деление `SqrtSumV1` замкнуто и точно, поэтому
    перенос иррациональности в знаменатель ничего не приближает.
    """

    weight = line.a * other.a + line.b * other.b
    cross = line.a * other.b - other.a * line.b
    numerator = (
        SqrtSumV1.rational(line.q * other.c)
        + along.scaled(cross)
        - SqrtSumV1.rational(weight * line.c)
    )
    speed = SqrtSumV1.radical(weight, line.q) - SqrtSumV1.radical(line.q, other.q)
    if speed.is_zero:
        if numerator.is_zero:
            return None, EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT
        return None, EventTimeOutcome.WAVEFRONT_TRIPLE_NEVER_CONCURRENT
    if numerator.is_zero:
        return ZERO_TIME, EventTimeOutcome.EXACT
    return (
        EventTimeV1.normalized(1, speed / numerator),
        EventTimeOutcome.EXACT,
    )


@dataclass(frozen=True, slots=True)
class EventPointV1:
    """Точка события в канонической форме: две `SqrtSumV1`.

    Каноничность здесь не украшение, а механизм: «та же точка» решается
    сравнением коэффициентов, поэтому кластеризация одновременных событий по
    совпадению точки — словарь, а не перебор с допуском.
    """

    x: SqrtSumV1
    y: SqrtSumV1

    def enclosure(self, bits: int = 64) -> tuple[tuple[Fraction, Fraction], ...]:
        return self.x.enclosure(bits), self.y.enclosure(bits)


def event_point(
    first: SupportLineV1, second: SupportLineV1, time: EventTimeV1
) -> EventPointV1:
    """Пересечение двух движущихся прямых в момент `time`, точно.

    Деление на `SqrtSumV1` идёт домножением на сопряжённые, поэтому результат
    остаётся канонической суммой корней и сравним побитово.
    """

    determinant = first.a * second.b - second.a * first.b
    if determinant == 0:
        raise ParallelSupportLinesError("прямые параллельны, точки пересечения нет")
    right_first = SqrtSumV1.rational(first.c) * time.divisor + SqrtSumV1.radical(
        time.dividend, first.q
    )
    right_second = SqrtSumV1.rational(second.c) * time.divisor + SqrtSumV1.radical(
        time.dividend, second.q
    )
    scale = time.divisor.scaled(determinant)
    x = (right_first.scaled(second.b) - right_second.scaled(first.b)) / scale
    y = (right_second.scaled(first.a) - right_first.scaled(second.a)) / scale
    return EventPointV1(x, y)
