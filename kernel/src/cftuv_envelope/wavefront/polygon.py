"""Вход волнового фронта: контуры на целочисленной решётке, дыры с начала.

Дыра здесь не особый случай и не надстройка. Внутренний контур испускает свой
фронт наружу, встреча его с внешним фронтом — обычное split-событие, и код
события об этом не знает. Единственное, что дыра требует отдельно, — свой LAV
на старте (`InitSlav` у Kendzi) и обратная ориентация.

Ориентация нормируется здесь и только здесь: внешний контур против часовой
стрелки, дыры по часовой. Тогда «влево от хода» у КАЖДОГО ребра означает «в
сторону материала», и `SupportLineV1.through` не нуждается в параметре.

Порогов в проверках нет: замкнутость, простота ориентации и вырожденность
рёбер решаются целочисленными предикатами `robust/`.

ЧАСТИЧНЫЙ ИСТОЧНИК. Контур больше не обязан быть источником целиком. У каждого
ребра есть своё `q` — КВАДРАТ СКОРОСТИ ФРОНТА В ЕДИНИЦАХ НОРМАЛИ, то самое `q`
из `a*x + b*y = c + t*sqrt(q)`. Евклидова скорость ребра равна
`sqrt(q)/|(a, b)|`, поэтому:

| `q` | что это | евклидова скорость |
|---|---|---|
| `0` | СТЕНА: прямая не двигается, фронт от ребра не идёт | 0 |
| `\\|d\\|^2` | источник единичной скорости — УМОЛЧАНИЕ | 1 |
| `s^2 * \\|d\\|^2` | ВЗВЕШЕННОЕ ребро скорости `s` | `s` |

Три строки — одна сущность, и это не удобство записи. Стена и вес — один и тот
же выход за пределы тождества `q = a^2 + b^2`, поэтому обобщение сделано по
`q`, а не отдельным флагом «стена»: флаг закрыл бы первую строку и оставил
третью.

Умолчание `speeds_squared is None` означает «источником является ВСЁ» и
сохраняет прежнее поведение побитово: `q` вычисляется как `|d|^2`, ровно то же
число, которое кладёт `SupportLineV1.through`.

Пометка ПЕРЕЖИВАЕТ нормировку. `oriented()` разворачивает петлю, и вместе с
точками переставляются скорости: у развёрнутой петли ребро `j` — это прежнее
ребро `n - 2 - j`, взятое в обратную сторону. Без этой перестановки пометка
молча уезжала бы на соседнее ребро у каждой петли, ориентацию которой пришлось
править, — то есть у всех дыр.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from ..robust.predicates import orient2d
from .event_time import (
    NegativeSpeedError,
    NonRationalSpeedError,
    normalized_speed,
)


class PolygonOutcome(str, Enum):
    """Именованные отказы входа. Молча ничего не чинится и не отбрасывается."""

    EXACT = "EXACT"
    LOOP_TOO_SHORT = "LOOP_TOO_SHORT"
    LOOP_HAS_REPEATED_POINT = "LOOP_HAS_REPEATED_POINT"
    LOOP_HAS_ZERO_AREA = "LOOP_HAS_ZERO_AREA"
    LOOP_COORDINATE_IS_NOT_INTEGER = "LOOP_COORDINATE_IS_NOT_INTEGER"
    # Скоростей не столько, сколько рёбер. Молча дополнить их умолчанием
    # значило бы решить за вызывающего, какие рёбра он забыл пометить.
    LOOP_SPEED_COUNT_DOES_NOT_MATCH_EDGES = (
        "LOOP_SPEED_COUNT_DOES_NOT_MATCH_EDGES"
    )
    # `q` обязано быть РАЦИОНАЛЬНЫМ. Прежде здесь стояло требование целого
    # (`LOOP_SPEED_IS_NOT_INTEGER`), и оно было границей `SqrtSumV1`, а не
    # свойством задачи: закон прихода со скоростью `s` даёт `q = (s/|n|)^2 *
    # |d|^2`, и у полевого патча знаменатель этой дроби БЕСКВАДРАТЕН, поэтому
    # целым `q` не станет ни при каком допустимом шаге решётки. Требование
    # снято тождеством `sqrt(p/r) = (1/r)*sqrt(p*r)`, а не округлением.
    #
    # `bool` отвергается по-прежнему, хотя формально он `int`: `(True, False,
    # ...)` означало бы «скорости 1 и 0», а написать это хотели как «источник
    # и стена». `float` отвергается тем же членом: приблизительное `q`
    # отравило бы каноническую форму, у которой единственность держится на
    # точных дробях.
    LOOP_SPEED_IS_NOT_RATIONAL = "LOOP_SPEED_IS_NOT_RATIONAL"
    LOOP_SPEED_IS_NEGATIVE = "LOOP_SPEED_IS_NEGATIVE"
    # Ни одного ребра-источника: фронту неоткуда пойти. Отдельный исход, потому
    # что иначе очередь ответила бы `WAVEFRONT_LEFT_UNRESOLVED` — верно по
    # букве и бесполезно по существу, ведь неразрешённым остался бы ВЕСЬ фронт.
    POLYGON_HAS_NO_SOURCE_EDGE = "POLYGON_HAS_NO_SOURCE_EDGE"
    # Помечено ребро, которого в полигоне нет. Пометка, не нашедшая своего
    # ребра, — это потеря, и она названа, а не проигнорирована.
    SOURCE_SPAN_IS_NOT_AN_EDGE = "SOURCE_SPAN_IS_NOT_AN_EDGE"


class PolygonRejected(ValueError):
    """Вход отвергнут с названным исходом."""

    def __init__(self, outcome: PolygonOutcome, detail: str) -> None:
        super().__init__(f"{outcome.value}: {detail}")
        self.outcome = outcome


#: Ребро-СТЕНА. `a*x + b*y = c + t*sqrt(0)` — та же прямая при любом `t`.
WALL_SPEED_SQUARED = 0


def unit_speed_squared(
    start: tuple[int, int], end: tuple[int, int]
) -> int:
    """`q = |d|^2` — ровно то, что кладёт `SupportLineV1.through`.

    Функция одна на репозиторий именно затем, чтобы умолчание корпуса и
    умолчание несущей прямой не могли разойтись правкой одного из двух.
    """

    dx, dy = end[0] - start[0], end[1] - start[1]
    return dx * dx + dy * dy


def signed_double_area(points: tuple[tuple[int, int], ...]) -> int:
    """Удвоенная ориентированная площадь. Целое, поэтому знак точен."""

    total = 0
    for index in range(len(points)):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % len(points)]
        total += x0 * y1 - x1 * y0
    return total


@dataclass(frozen=True, slots=True)
class LoopV1:
    """Замкнутый контур из различных узлов решётки.

    `speeds_squared[i]` — это `q` ребра `points[i] -> points[i+1]`. `None`
    означает умолчание «каждое ребро источник единичной скорости»; хранить его
    отдельным значением, а не заполненным кортежем, нужно затем, чтобы
    «пометки не было» отличалось от «пометка совпала с умолчанием»: первое
    переживает разворот петли тождественно, второе — перестановкой.
    """

    points: tuple[tuple[int, int], ...]
    speeds_squared: tuple[int | Fraction, ...] | None = None

    def __post_init__(self) -> None:
        if len(self.points) < 3:
            raise PolygonRejected(
                PolygonOutcome.LOOP_TOO_SHORT, f"{len(self.points)} вершин"
            )
        for point in self.points:
            if not all(isinstance(value, int) for value in point):
                raise PolygonRejected(
                    PolygonOutcome.LOOP_COORDINATE_IS_NOT_INTEGER, str(point)
                )
        if len(set(self.points)) != len(self.points):
            raise PolygonRejected(
                PolygonOutcome.LOOP_HAS_REPEATED_POINT, str(self.points)
            )
        if signed_double_area(self.points) == 0:
            raise PolygonRejected(
                PolygonOutcome.LOOP_HAS_ZERO_AREA, str(self.points)
            )
        self._check_speeds()

    def _check_speeds(self) -> None:
        """Пометка проверяется целиком и до всякого счёта, а не по месту.

        Проверка стоит отдельным методом, потому что `__post_init__` иначе
        перерос бы бюджет функции, а дробить условия по вызывающим значило бы
        завести второе место, где пометка может пройти незамеченной.

        Пометка ещё и НОРМИРУЕТСЯ здесь: дробь со знаменателем 1 становится
        `int`. Иначе `Fraction(4)` и `4` были бы двумя разными пометками при
        одном числе, и петля с одной и той же геометрией давала бы два разных
        `line_key`, то есть два разных дайджеста. Нормировка одна на репозиторий
        (`event_time.normalized_speed`), потому что второе её место немедленно
        разошлось бы с первым.
        """

        speeds = self.speeds_squared
        if speeds is None:
            return
        if len(speeds) != len(self.points):
            raise PolygonRejected(
                PolygonOutcome.LOOP_SPEED_COUNT_DOES_NOT_MATCH_EDGES,
                f"{len(speeds)} скоростей при {len(self.points)} рёбрах",
            )
        normalized: list[int | Fraction] = []
        for value in speeds:
            try:
                normalized.append(normalized_speed(value))
            except NonRationalSpeedError:
                raise PolygonRejected(
                    PolygonOutcome.LOOP_SPEED_IS_NOT_RATIONAL, repr(value)
                ) from None
            except NegativeSpeedError:
                raise PolygonRejected(
                    PolygonOutcome.LOOP_SPEED_IS_NEGATIVE, str(value)
                ) from None
        object.__setattr__(self, "speeds_squared", tuple(normalized))

    @property
    def edge_speeds_squared(self) -> tuple[int | Fraction, ...]:
        """`q` каждого ребра, с умолчанием, уже раскрытым в числа."""

        if self.speeds_squared is not None:
            return self.speeds_squared
        size = len(self.points)
        return tuple(
            unit_speed_squared(
                self.points[index], self.points[(index + 1) % size]
            )
            for index in range(size)
        )

    @property
    def source_flags(self) -> tuple[bool, ...]:
        """Испускает ли ребро фронт. Ноль скорости — стена, всё прочее — источник."""

        return tuple(value > 0 for value in self.edge_speeds_squared)

    @property
    def is_counter_clockwise(self) -> bool:
        return signed_double_area(self.points) > 0

    def oriented(self, *, counter_clockwise: bool) -> "LoopV1":
        if self.is_counter_clockwise == counter_clockwise:
            return self
        size = len(self.points)
        speeds = self.speeds_squared
        # Ребро `j` развёрнутой петли — это прежнее ребро `n - 2 - j`, пройденное
        # в обратную сторону. Скорость ребра от направления обхода не зависит,
        # поэтому переставляется она, а не пересчитывается.
        return LoopV1(
            tuple(reversed(self.points)),
            None
            if speeds is None
            else tuple(speeds[(size - 2 - index) % size] for index in range(size)),
        )

    def rotated(self, shift: int) -> "LoopV1":
        """Тот же контур с другой стартовой вершины. Для проверки инвариантности."""

        size = len(self.points)
        shift %= size
        speeds = self.speeds_squared
        return LoopV1(
            self.points[shift:] + self.points[:shift],
            None
            if speeds is None
            else speeds[shift:] + speeds[:shift],
        )

    def reflex_flags(self) -> tuple[bool, ...]:
        """Для каждой вершины: вогнута ли она относительно материала.

        Вершина `i` — стык рёбер `i-1` и `i`. Вогнутость решается знаком
        ориентации тройки, то есть целым определителем, а не углом.
        """

        size = len(self.points)
        return tuple(
            orient2d(
                self.points[(index - 1) % size],
                self.points[index],
                self.points[(index + 1) % size],
            )
            < 0
            for index in range(size)
        )


@dataclass(frozen=True, slots=True)
class PolygonV1:
    """Область с дырами. Внешний контур CCW, дыры CW — нормируется здесь."""

    outer: LoopV1
    holes: tuple[LoopV1, ...] = ()

    def __post_init__(self) -> None:
        if not any(any(loop.source_flags) for loop in self.loops):
            raise PolygonRejected(
                PolygonOutcome.POLYGON_HAS_NO_SOURCE_EDGE,
                f"{self.edge_count} рёбер, все стены",
            )

    @staticmethod
    def build(
        outer: LoopV1 | tuple[tuple[int, int], ...],
        holes: tuple = (),
    ) -> "PolygonV1":
        outer_loop = outer if isinstance(outer, LoopV1) else LoopV1(tuple(outer))
        hole_loops = tuple(
            hole if isinstance(hole, LoopV1) else LoopV1(tuple(hole))
            for hole in holes
        )
        return PolygonV1(
            outer_loop.oriented(counter_clockwise=True),
            tuple(
                hole.oriented(counter_clockwise=False) for hole in hole_loops
            ),
        )

    @property
    def loops(self) -> tuple[LoopV1, ...]:
        return (self.outer,) + self.holes

    @property
    def vertex_count(self) -> int:
        return sum(len(loop.points) for loop in self.loops)

    @property
    def edge_count(self) -> int:
        return self.vertex_count

    @property
    def reflex_count(self) -> int:
        return sum(sum(loop.reflex_flags()) for loop in self.loops)

    def edges(
        self,
    ) -> tuple[
        tuple[tuple[int, int], tuple[int, int], int | Fraction], ...
    ]:
        """Рёбра всех петель: концы и `q`, в порядке обхода петель.

        ЕДИНСТВЕННОЕ место, откуда потребители берут скорость ребра. Пока
        каждый строил несущую прямую сам через `SupportLineV1.through`, пометка
        терялась не отказом, а умолчанием: `through` кладёт `|d|^2` всегда, и
        стена молча становилась источником. Один источник пометки делает такую
        потерю невозможной, а не маловероятной.
        """

        records: list[
            tuple[tuple[int, int], tuple[int, int], int | Fraction]
        ] = []
        for loop in self.loops:
            points = loop.points
            speeds = loop.edge_speeds_squared
            size = len(points)
            for index in range(size):
                records.append(
                    (points[index], points[(index + 1) % size], speeds[index])
                )
        return tuple(records)

    @property
    def source_edge_count(self) -> int:
        return sum(1 for _, _, speed in self.edges() if speed > 0)

    @property
    def wall_edge_count(self) -> int:
        return sum(1 for _, _, speed in self.edges() if speed == 0)


def with_source_spans(
    polygon: PolygonV1,
    spans: tuple[tuple[tuple[int, int], tuple[int, int]], ...],
) -> PolygonV1:
    """Тот же полигон, где источники — ТОЛЬКО перечисленные рёбра, прочие стены.

    Ребро задаётся парой концов и опознаётся БЕЗ УЧЁТА направления: `PolygonV1`
    нормирует ориентацию петли и может развернуть её, после чего записанное
    вызывающим направление означало бы уже не то ребро. Пометка, не нашедшая
    своего ребра, — отказ `SOURCE_SPAN_IS_NOT_AN_EDGE`, а не тихо
    проигнорированная строка.
    """

    wanted = {frozenset((start, end)) for start, end in spans}
    seen: set[frozenset] = set()
    loops: list[LoopV1] = []
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        speeds = []
        for index in range(size):
            edge = frozenset((points[index], points[(index + 1) % size]))
            if edge in wanted:
                seen.add(edge)
                speeds.append(
                    unit_speed_squared(points[index], points[(index + 1) % size])
                )
            else:
                speeds.append(WALL_SPEED_SQUARED)
        loops.append(LoopV1(points, tuple(speeds)))
    missing = wanted - seen
    if missing:
        raise PolygonRejected(
            PolygonOutcome.SOURCE_SPAN_IS_NOT_AN_EDGE,
            f"{len(missing)} из {len(wanted)}: {sorted(map(sorted, missing))}",
        )
    return PolygonV1(loops[0], tuple(loops[1:]))


def with_edge_speeds(
    polygon: PolygonV1,
    speeds: tuple[
        tuple[tuple[int, int], tuple[int, int], int | Fraction], ...
    ],
) -> PolygonV1:
    """Тот же полигон с ПРОИЗВОЛЬНЫМИ `q` у перечисленных рёбер, прочие стены.

    Обобщение `with_source_spans` на взвешенный случай: там скорость каждого
    названного ребра единичная, здесь она задаётся числом. Общая функция одна,
    а не две похожих, потому что стена, источник и вес — три значения одного
    поля, и разводить их по разным путям означало бы снова считать стену
    особым случаем.
    """

    wanted = {frozenset((start, end)): value for start, end, value in speeds}
    seen: set[frozenset] = set()
    loops: list[LoopV1] = []
    for loop in polygon.loops:
        points = loop.points
        size = len(points)
        row = []
        for index in range(size):
            edge = frozenset((points[index], points[(index + 1) % size]))
            if edge in wanted:
                seen.add(edge)
                row.append(wanted[edge])
            else:
                row.append(WALL_SPEED_SQUARED)
        loops.append(LoopV1(points, tuple(row)))
    missing = set(wanted) - seen
    if missing:
        raise PolygonRejected(
            PolygonOutcome.SOURCE_SPAN_IS_NOT_AN_EDGE,
            f"{len(missing)} из {len(wanted)}: {sorted(map(sorted, missing))}",
        )
    return PolygonV1(loops[0], tuple(loops[1:]))
