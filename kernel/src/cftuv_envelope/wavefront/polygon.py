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
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..robust.predicates import orient2d


class PolygonOutcome(str, Enum):
    """Именованные отказы входа. Молча ничего не чинится и не отбрасывается."""

    EXACT = "EXACT"
    LOOP_TOO_SHORT = "LOOP_TOO_SHORT"
    LOOP_HAS_REPEATED_POINT = "LOOP_HAS_REPEATED_POINT"
    LOOP_HAS_ZERO_AREA = "LOOP_HAS_ZERO_AREA"
    LOOP_COORDINATE_IS_NOT_INTEGER = "LOOP_COORDINATE_IS_NOT_INTEGER"


class PolygonRejected(ValueError):
    """Вход отвергнут с названным исходом."""

    def __init__(self, outcome: PolygonOutcome, detail: str) -> None:
        super().__init__(f"{outcome.value}: {detail}")
        self.outcome = outcome


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
    """Замкнутый контур из различных узлов решётки."""

    points: tuple[tuple[int, int], ...]

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

    @property
    def is_counter_clockwise(self) -> bool:
        return signed_double_area(self.points) > 0

    def oriented(self, *, counter_clockwise: bool) -> "LoopV1":
        if self.is_counter_clockwise == counter_clockwise:
            return self
        return LoopV1(tuple(reversed(self.points)))

    def rotated(self, shift: int) -> "LoopV1":
        """Тот же контур с другой стартовой вершины. Для проверки инвариантности."""

        size = len(self.points)
        shift %= size
        return LoopV1(self.points[shift:] + self.points[:shift])

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
    def reflex_count(self) -> int:
        return sum(sum(loop.reflex_flags()) for loop in self.loops)
