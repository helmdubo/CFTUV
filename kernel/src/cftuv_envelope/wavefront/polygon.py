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

ВЕЕР ВОГНУТОЙ ВЕРШИНЫ. Продукту нужен МЯГКИЙ вогнутый угол, а straight skeleton
по построению даёт только острый митрованный: вершина уходит внутрь по
биссектрисе, и апекс остаётся точкой. Замковый камень — linear axis (Tanase &
Veltkamp): начальный фронт есть ИЗМЕНЁННЫЙ полигон, у которого в вогнутой
вершине вставлены рёбра НУЛЕВОЙ ДЛИНЫ; дальше они живут обычными движущимися
рёбрами, и ни одного нового правила событий им не требуется.

Веер поэтому НЕ является частью `LoopV1` и не может ею быть: петля запрещает
повтор точки, а ребро нулевой длины — это ровно повторившаяся точка. Он
объявлен пер-вершинной разметкой `VertexFanV1`, привязанной к КООРДИНАТАМ
вершины: `PolygonV1.build` нормирует ориентацию и может развернуть петлю, после
чего индекс вершины означал бы уже другую вершину, а координата остаётся собой.

Длина веера здесь НЕ ограничена. Ограничение `k <= 2` живёт в разделяемом
рецепте направлений (`reference/angular.py`), потому что оно вытекает из
продуктового `delta_max = pi/3`, а не из машинерии фронта; плотность сегментов
объявлена будущим пользовательским параметром, и вход, зашивший в себя `k <= 2`,
пришлось бы переписывать вместе с ним.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from ..robust.predicates import orient2d
from .event_time import (
    NegativeSpeedError,
    NonRationalSpeedError,
    SupportLineV1,
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
    # Веер объявлен в точке, которая вершиной контура не является либо является
    # ею дважды (объявлен два раза, либо точка принадлежит двум петлям). Веер
    # вставляется МЕЖДУ входящим и исходящим ребром вершины, и без единственной
    # вершины у него нет места.
    VERTEX_FAN_POINT_IS_NOT_A_UNIQUE_VERTEX = (
        "VERTEX_FAN_POINT_IS_NOT_A_UNIQUE_VERTEX"
    )
    # У скрытой опоры нулевая нормаль (прямой нет вовсе) либо `q = 0` (стена).
    # Ребро веера обязано ДВИГАТЬСЯ: неподвижное ребро нулевой длины не заметает
    # ничего и остаётся точкой, то есть веер молча вырождается в митр.
    VERTEX_FAN_SUPPORT_IS_NOT_A_MOVING_LINE = (
        "VERTEX_FAN_SUPPORT_IS_NOT_A_MOVING_LINE"
    )
    # Нормали веера не поворачивают ВНУТРЬ вогнутого сектора монотонно. Порядок
    # опор — часть входа (от входящей нормали к исходящей), и опора, стоящая не
    # в своём секторе, дала бы фронт, идущий не туда; отсортировать её здесь
    # значило бы придумать порядок, которого во входе нет.
    VERTEX_FAN_SUPPORT_IS_NOT_INSIDE_THE_REFLEX_SECTOR = (
        "VERTEX_FAN_SUPPORT_IS_NOT_INSIDE_THE_REFLEX_SECTOR"
    )


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


def _edge_normal(
    start: tuple[int, int], end: tuple[int, int]
) -> tuple[int, int]:
    """Нормаль ребра ВЛЕВО от хода — та же `(-dy, dx)`, что у `SupportLineV1`."""

    return (-(end[1] - start[1]), end[0] - start[0])


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
class FanSupportV1:
    """Одна скрытая опора веера: несущая прямая ЧЕРЕЗ вершину и её `q`.

    Константа прямой здесь НЕ хранится: она равна `a*x + b*y` в самой вершине по
    определению веера, и второе место её записи немедленно разошлось бы с
    первым. Нормаль целая, `q` рационально — те же требования, что у
    `SupportLineV1`, и по той же причине: все предикаты ниже целочисленные, а
    рациональность входит ровно под корень.
    """

    normal_x: int
    normal_y: int
    speed_squared: int | Fraction

    def __post_init__(self) -> None:
        if self.normal_x == 0 and self.normal_y == 0:
            raise PolygonRejected(
                PolygonOutcome.VERTEX_FAN_SUPPORT_IS_NOT_A_MOVING_LINE,
                "нормаль скрытой опоры нулевая",
            )
        try:
            speed = normalized_speed(self.speed_squared)
        except NonRationalSpeedError:
            raise PolygonRejected(
                PolygonOutcome.LOOP_SPEED_IS_NOT_RATIONAL,
                repr(self.speed_squared),
            ) from None
        except NegativeSpeedError:
            raise PolygonRejected(
                PolygonOutcome.LOOP_SPEED_IS_NEGATIVE, str(self.speed_squared)
            ) from None
        if speed == 0:
            raise PolygonRejected(
                PolygonOutcome.VERTEX_FAN_SUPPORT_IS_NOT_A_MOVING_LINE,
                "q скрытой опоры равно нулю: ребро веера не двигалось бы",
            )
        object.__setattr__(self, "speed_squared", speed)

    @property
    def normal(self) -> tuple[int, int]:
        return (self.normal_x, self.normal_y)

    def constant_at(self, point: tuple[int, int]) -> int:
        return self.normal_x * point[0] + self.normal_y * point[1]


@dataclass(frozen=True, slots=True)
class VertexFanV1:
    """Веер скрытых опор в ОДНОЙ вершине: `k` рёбер нулевой длины подряд.

    `supports` упорядочены от ВХОДЯЩЕГО ребра вершины к ИСХОДЯЩЕМУ, ровно как
    `reference/angular.py` упорядочивает `_interpolated_normals`. Пустой веер
    законен и означает митрованный угол (`k = 0`): вход тогда тождественно
    равен полигону без веера, и это проверяемое свойство, а не соглашение.
    """

    point: tuple[int, int]
    supports: tuple[FanSupportV1, ...]


@dataclass(frozen=True, slots=True)
class PolygonV1:
    """Область с дырами. Внешний контур CCW, дыры CW — нормируется здесь."""

    outer: LoopV1
    holes: tuple[LoopV1, ...] = ()
    #: Вееры вогнутых вершин: рёбра нулевой длины начального фронта. Привязаны
    #: к КООРДИНАТАМ вершины, потому что нормировка ориентации переставляет
    #: индексы, а координаты оставляет собой.
    vertex_fans: tuple[VertexFanV1, ...] = ()

    def __post_init__(self) -> None:
        if not any(any(loop.source_flags) for loop in self.loops):
            raise PolygonRejected(
                PolygonOutcome.POLYGON_HAS_NO_SOURCE_EDGE,
                f"{self.edge_count} рёбер, все стены",
            )
        self._check_vertex_fans()

    def _check_vertex_fans(self) -> None:
        """Веер проверяется ЦЕЛИКОМ и до всякого счёта, как и пометка скоростей.

        Проверок здесь две, и каждая ловит своё. Первая — что точка веера есть
        ЕДИНСТВЕННАЯ вершина контуров: веер вставляется между входящим и
        исходящим ребром, и у точки, принадлежащей двум петлям, таких пар две.
        Вторая — что нормали поворачивают внутрь ВОГНУТОГО сектора монотонно:
        `n_prev x n_1 < 0`, `n_1 x n_2 < 0`, ..., `n_k x n_next < 0`. Знак
        векторного произведения целочислен, поэтому порога тут нет; строгое
        неравенство отвергает и совпавшие направления (ребро веера, слившееся с
        соседом), и развёрнутый на pi поворот.

        Отрицательный знак — это и есть вогнутость: `_is_reflex` в `skeleton.py`
        решает её ровно тем же произведением нормалей, и функция одна на смысл,
        чтобы вход и цикл событий не могли разойтись правкой одного из двух.
        """

        if not self.vertex_fans:
            return
        corners: dict[tuple[int, int], tuple[tuple[int, int], tuple[int, int]]] = {}
        shared: set[tuple[int, int]] = set()
        for loop in self.loops:
            points = loop.points
            size = len(points)
            for index in range(size):
                if points[index] in corners:
                    shared.add(points[index])
                corners[points[index]] = (
                    _edge_normal(points[(index - 1) % size], points[index]),
                    _edge_normal(points[index], points[(index + 1) % size]),
                )
        seen: set[tuple[int, int]] = set()
        for fan in self.vertex_fans:
            corner = corners.get(fan.point)
            if corner is None or fan.point in shared or fan.point in seen:
                raise PolygonRejected(
                    PolygonOutcome.VERTEX_FAN_POINT_IS_NOT_A_UNIQUE_VERTEX,
                    str(fan.point),
                )
            seen.add(fan.point)
            normals = (
                corner[0],
                *(support.normal for support in fan.supports),
                corner[1],
            )
            for left, right in zip(normals, normals[1:]):
                if left[0] * right[1] - left[1] * right[0] >= 0:
                    raise PolygonRejected(
                        PolygonOutcome
                        .VERTEX_FAN_SUPPORT_IS_NOT_INSIDE_THE_REFLEX_SECTOR,
                        f"{fan.point}: {left} -> {right}",
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

    @property
    def fan_edge_count(self) -> int:
        """Сколько рёбер НУЛЕВОЙ ДЛИНЫ добавляют вееры в начальный фронт."""

        return sum(len(fan.supports) for fan in self.vertex_fans)

    def fan_edges(
        self,
    ) -> tuple[tuple[tuple[int, int], int, SupportLineV1], ...]:
        """Рёбра вееров: вершина, порядковый номер опоры и её несущая прямая.

        ЕДИНСТВЕННОЕ место, откуда потребители берут прямую скрытой опоры, —
        по той же причине, по какой `edges()` единственный источник скорости
        ребра: второе место немедленно разошлось бы с первым, и разошлось бы
        молча. Ординал начинается с единицы и совпадает с `ordinal` у
        `HiddenSupportSpecV1`, чтобы веер очереди и веер эталона нумеровались
        одинаково.

        Рёбра `edges()` веера НЕ содержат: у них нулевая длина, а `edges()`
        отдаёт пары концов, из которых потребители выводят и `|d|^2`, и
        направление. Ноль там означал бы деление на ноль в каждом втором
        вызывающем.
        """

        records: list[tuple[tuple[int, int], int, SupportLineV1]] = []
        for fan in sorted(self.vertex_fans, key=lambda item: item.point):
            for ordinal, support in enumerate(fan.supports, start=1):
                records.append(
                    (
                        fan.point,
                        ordinal,
                        SupportLineV1(
                            support.normal_x,
                            support.normal_y,
                            support.constant_at(fan.point),
                            support.speed_squared,
                        ),
                    )
                )
        return tuple(records)

    def fan_at(self, point: tuple[int, int]) -> VertexFanV1 | None:
        for fan in self.vertex_fans:
            if fan.point == point:
                return fan
        return None


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
    return PolygonV1(loops[0], tuple(loops[1:]), polygon.vertex_fans)


def with_vertex_fans(
    polygon: PolygonV1, fans: tuple[VertexFanV1, ...]
) -> PolygonV1:
    """Тот же полигон с ВЕЕРАМИ в перечисленных вершинах.

    Отдельной функцией, а не параметром `build`, ровно по той же причине, что и
    `with_edge_speeds`: `build` НОРМИРУЕТ ориентацию и может развернуть петлю, а
    веер задан относительно входящего и исходящего ребра вершины. Привязка идёт
    по координате, поэтому применять её надо к уже нормированному полигону —
    иначе «входящее ребро» означало бы разное до и после разворота.

    Проверки веера делает `PolygonV1.__post_init__`, и здесь они не дублируются:
    второе место проверки — это второе место, где её можно ослабить.
    """

    return PolygonV1(polygon.outer, polygon.holes, tuple(fans))


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
    return PolygonV1(loops[0], tuple(loops[1:]), polygon.vertex_fans)
