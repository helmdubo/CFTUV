"""Сетка ячеек над целочисленной решёткой: ФИЛЬТР, не принимающий решений.

Зачем она нужна. Наивный поиск split-кандидатов перебирает `reflex x рёбра` —
1 998 000 пар при 2002 вершинах. Теорема 2.11 Huber'а сводит поиск к трассе
мотоцикла, но вопрос «какие несущие прямые проходят близко к трассе»
пространственный: без индекса он снова стоит O(n) на каждую вершину, и
показатель степени не двигается ни на сколько.

Чем эта сетка НЕ является. Она никогда не отвечает на геометрический вопрос —
только на вопрос «что стоит проверить точно», и ответ обязан быть НАДмножеством
истины. Отсюда три свойства, и они же причина, по которой здесь нет порога:

- разметка консервативна: объект попадает во все ячейки, которые он может
  задевать;
- границы считаются на `Fraction` и округляются НАРУЖУ, а `floor`/`ceil` от
  точной дроби точны — это доказанное включение, а не приближение;
- лишняя ячейка не портит ничего, а пропущенная ловится дифференциальным тестом
  против полного перебора.

Сторона ячейки — степень двойки: деление на неё точно, и выбор стороны выводится
из области и числа объектов, а не подбирается.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from fractions import Fraction
import math


Number = Fraction | int
Cell = tuple[int, int]


class CellGridRejected(ValueError):
    """Область сетки пуста. Именованный отказ, а не сетка нулевого размера."""


@dataclass(frozen=True, slots=True)
class CellGridV1:
    """Прямоугольная область, разбитая на квадратные ячейки со стороной `cell`."""

    x_min: int
    y_min: int
    x_max: int
    y_max: int
    cell: int

    @staticmethod
    def covering(
        box: tuple[int, int, int, int], *, targets: int
    ) -> "CellGridV1":
        """Сетка над `box`, у которой ячеек не больше примерно `targets`.

        Сторона — наименьшая степень двойки, при которой условие выполнено.
        Мельче было бы дороже размечать, крупнее — дороже опрашивать; ни то ни
        другое не влияет на ОТВЕТ, только на цену.
        """

        x_min, y_min, x_max, y_max = box
        if x_max < x_min or y_max < y_min:
            raise CellGridRejected(f"пустая область {box}")
        width = max(1, x_max - x_min)
        height = max(1, y_max - y_min)
        budget = max(1, targets)
        cell = 1
        while (width // cell + 1) * (height // cell + 1) > budget:
            cell *= 2
        return CellGridV1(x_min, y_min, x_max, y_max, cell)

    @property
    def columns(self) -> int:
        return (self.x_max - self.x_min) // self.cell + 1

    @property
    def rows(self) -> int:
        return (self.y_max - self.y_min) // self.cell + 1

    @property
    def x_limit(self) -> int:
        """Правая граница ПОКРЫТОЙ ячейками площади, а не объявленной рамки."""

        return self.x_min + self.columns * self.cell

    @property
    def y_limit(self) -> int:
        return self.y_min + self.rows * self.cell

    def column(self, x: Number) -> int:
        return math.floor(Fraction(x - self.x_min, self.cell))

    def row(self, y: Number) -> int:
        return math.floor(Fraction(y - self.y_min, self.cell))

    # Ячейки ЗАМКНУТЫ, и это не мелочь представления. На целочисленной решётке
    # осевые рёбра ложатся ровно на границы ячеек, а точка пересечения двух
    # объектов может оказаться ровно на границе. При полуоткрытых ячейках такая
    # точка принадлежала бы одной ячейке, и совпадение приходилось бы разбирать
    # отдельно; при замкнутых объект помечает обе соседние, и пересечение
    # заведомо накрыто общей ячейкой. Цена — до четырёх лишних ячеек на объект.
    def _low_index(self, value: Number, origin: int) -> int:
        return math.ceil(Fraction(value - origin, self.cell)) - 1

    def _high_index(self, value: Number, origin: int) -> int:
        return math.floor(Fraction(value - origin, self.cell))

    def contains_box(
        self, x_low: Number, x_high: Number, y_low: Number, y_high: Number
    ) -> bool:
        """Прямоугольник целиком внутри области.

        Если нет — запрос по сетке НЕПОЛОН, и вызывающий обязан уйти на полный
        перебор. Молча отдать усечённый ответ значило бы потерять события.
        """

        return (
            x_low >= self.x_min
            and x_high <= self.x_max
            and y_low >= self.y_min
            and y_high <= self.y_max
        )

    def box_cells(
        self, x_low: Number, x_high: Number, y_low: Number, y_high: Number
    ) -> tuple[Cell, ...]:
        """Все ячейки, задеваемые прямоугольником. Клип к области сетки."""

        first_column = max(0, self._low_index(x_low, self.x_min))
        last_column = min(self.columns - 1, self._high_index(x_high, self.x_min))
        first_row = max(0, self._low_index(y_low, self.y_min))
        last_row = min(self.rows - 1, self._high_index(y_high, self.y_min))
        if last_column < first_column or last_row < first_row:
            return ()
        return tuple(
            (column, row)
            for column in range(first_column, last_column + 1)
            for row in range(first_row, last_row + 1)
        )

    def line_cells(
        self,
        a: int,
        b: int,
        c: int,
        *,
        window: tuple[Number, Number, Number, Number] | None = None,
    ) -> tuple[Cell, ...]:
        """Ячейки, которые может пересекать прямая `a*x + b*y = c`.

        `window` ограничивает объект его собственной рамкой: у отрезка она есть,
        у прямой — нет. Проход идёт по столбцам, и диапазон столбцов заранее
        сужается до участка, где прямая вообще попадает в область: иначе даже
        почти вертикальная прямая стоила бы перебора всех столбцов.
        """

        x_low, x_high, y_low, y_high = self._clipped(window)
        if x_high < x_low or y_high < y_low:
            return ()
        if a == 0 and b == 0:
            return ()
        if b == 0:
            x = Fraction(c, a)
            if not x_low <= x <= x_high:
                return ()
            return self.box_cells(x, x, y_low, y_high)
        if a != 0:
            at_low = Fraction(c - b * y_low, a)
            at_high = Fraction(c - b * y_high, a)
            x_low = max(Fraction(x_low), min(at_low, at_high))
            x_high = min(Fraction(x_high), max(at_low, at_high))
            if x_high < x_low:
                return ()
        return self._columns_of_line(a, b, c, x_low, x_high, y_low, y_high)

    def segment_cells(
        self, start: tuple[int, int], end: tuple[int, int]
    ) -> tuple[Cell, ...]:
        """Ячейки отрезка: та же разметка, но окном служит его рамка."""

        (x0, y0), (x1, y1) = start, end
        a, b = y0 - y1, x1 - x0
        if a == 0 and b == 0:
            return self.box_cells(x0, x0, y0, y0)
        return self.line_cells(
            a,
            b,
            a * x0 + b * y0,
            window=(min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)),
        )

    # ---- внутреннее ------------------------------------------------------

    def _clipped(
        self, window: tuple[Number, Number, Number, Number] | None
    ) -> tuple[Number, Number, Number, Number]:
        if window is None:
            return self.x_min, self.x_limit, self.y_min, self.y_limit
        x_low, x_high, y_low, y_high = window
        return (
            max(x_low, self.x_min),
            min(x_high, self.x_limit),
            max(y_low, self.y_min),
            min(y_high, self.y_limit),
        )

    def _columns_of_line(
        self,
        a: int,
        b: int,
        c: int,
        x_low: Number,
        x_high: Number,
        y_low: Number,
        y_high: Number,
    ) -> tuple[Cell, ...]:
        cells: list[Cell] = []
        first_column = max(0, self._low_index(x_low, self.x_min))
        last_column = min(self.columns - 1, self._high_index(x_high, self.x_min))
        for column in range(first_column, last_column + 1):
            left = max(Fraction(x_low), Fraction(self.x_min + column * self.cell))
            right = min(
                Fraction(x_high),
                Fraction(self.x_min + (column + 1) * self.cell),
            )
            if right < left:
                continue
            at_left = Fraction(c - a * left, b)
            at_right = Fraction(c - a * right, b)
            low = max(Fraction(y_low), min(at_left, at_right))
            high = min(Fraction(y_high), max(at_left, at_right))
            if high < low:
                continue
            first_row = max(0, self._low_index(low, self.y_min))
            last_row = min(self.rows - 1, self._high_index(high, self.y_min))
            for row in range(first_row, last_row + 1):
                cells.append((column, row))
        return tuple(cells)


@dataclass(slots=True)
class CellIndexV1:
    """Ячейка -> идентификаторы. Порядок выдачи отсортирован, значит воспроизводим."""

    buckets: dict[Cell, list[int]] = field(default_factory=dict)

    def add(self, ident: int, cells: tuple[Cell, ...]) -> None:
        for cell in cells:
            self.buckets.setdefault(cell, []).append(ident)

    def lookup(self, cells: tuple[Cell, ...]) -> tuple[int, ...]:
        found: set[int] = set()
        for cell in cells:
            found.update(self.buckets.get(cell, ()))
        return tuple(sorted(found))
