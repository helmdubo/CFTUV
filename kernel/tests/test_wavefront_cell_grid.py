"""Сетка ячеек обязана быть НАДмножеством: доказывается перебором всех ячеек.

Единственное свойство, ради которого сетка существует, — консервативность
разметки. Если объект задевает ячейку, а ячейка не размечена, то фильтр теряет
кандидата молча, и потеря выйдет наружу далеко от места ошибки. Поэтому здесь
не проверяется «сетка работает»: здесь для каждой ячейки области точным
целочисленным предикатом решается, задевает ли её объект, и результат сверяется
с разметкой.

Обратное включение НЕ требуется и не проверяется: лишняя ячейка стоит времени,
но не меняет ответа. Проверяется только то, что лишних не слишком много, —
иначе фильтр перестал бы быть фильтром.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront.cell_grid import (
    CellGridV1,
    CellIndexV1,
    CellGridRejected,
)


def _corners(grid: CellGridV1, cell: tuple[int, int]) -> tuple[tuple[int, int], ...]:
    column, row = cell
    x0 = grid.x_min + column * grid.cell
    y0 = grid.y_min + row * grid.cell
    x1, y1 = x0 + grid.cell, y0 + grid.cell
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def _line_meets_cell(grid, cell, a: int, b: int, c: int) -> bool:
    """Прямая задевает замкнутую ячейку. Точный целочисленный предикат."""

    values = [a * x + b * y - c for x, y in _corners(grid, cell)]
    return min(values) <= 0 <= max(values)


def _boxes_overlap(first, second) -> bool:
    x0, x1, y0, y1 = first
    u0, u1, v0, v1 = second
    return not (x1 < u0 or u1 < x0 or y1 < v0 or v1 < y0)


def _segment_meets_cell(grid, cell, start, end) -> bool:
    """Отрезок против ячейки — разделяющие оси x, y и нормаль отрезка.

    В двух измерениях этих трёх осей достаточно, поэтому предикат полон, а не
    приблизителен: пересечения нет тогда и только тогда, когда разделяет одна
    из них.
    """

    (x0, y0), (x1, y1) = start, end
    column, row = cell
    cell_box = (
        grid.x_min + column * grid.cell,
        grid.x_min + (column + 1) * grid.cell,
        grid.y_min + row * grid.cell,
        grid.y_min + (row + 1) * grid.cell,
    )
    if not _boxes_overlap(
        (min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)), cell_box
    ):
        return False
    a, b = y0 - y1, x1 - x0
    if a == 0 and b == 0:
        return True
    return _line_meets_cell(grid, cell, a, b, a * x0 + b * y0)


def _all_cells(grid: CellGridV1):
    for column in range(grid.columns):
        for row in range(grid.rows):
            yield (column, row)


GRIDS = (
    CellGridV1(0, 0, 64, 64, 8),
    CellGridV1(-30, -7, 130, 41, 16),
    CellGridV1(0, 0, 12000, 10, 16),
    CellGridV1(-5, -5, 5, 5, 1),
)

LINES = (
    (0, 1, 0),
    (1, 0, 7),
    (1, -1, 26),
    (1, 1, 34),
    (3, 7, 121),
    (-11, 2, -45),
    (1, 1000, 4000),
    (1000, 1, 4000),
)


@pytest.mark.parametrize("grid", GRIDS, ids=lambda g: f"cell{g.cell}")
@pytest.mark.parametrize("line", LINES, ids=lambda line: str(line))
def test_every_cell_a_line_really_touches_is_marked(grid, line):
    a, b, c = line
    marked = set(grid.line_cells(a, b, c))
    truth = {cell for cell in _all_cells(grid) if _line_meets_cell(grid, cell, a, b, c)}
    assert truth <= marked, f"пропущены ячейки: {sorted(truth - marked)}"


@pytest.mark.parametrize("grid", GRIDS, ids=lambda g: f"cell{g.cell}")
@pytest.mark.parametrize("line", LINES, ids=lambda line: str(line))
def test_a_line_is_not_marked_far_wider_than_it_needs(grid, line):
    """Отрицательный контроль: фильтр обязан ФИЛЬТРОВАТЬ.

    Без этой границы разметка «все ячейки» тоже была бы консервативной и тоже
    прошла бы предыдущий тест, не сузив ничего.
    """

    a, b, c = line
    marked = grid.line_cells(a, b, c)
    truth = [cell for cell in _all_cells(grid) if _line_meets_cell(grid, cell, a, b, c)]
    assert len(marked) <= 3 * len(truth) + 4


SEGMENTS = (
    ((0, 0), (64, 0)),
    ((0, 0), (0, 64)),
    ((3, 5), (61, 59)),
    ((36, 10), (30, 4)),
    ((30, 4), (24, 10)),
    ((-30, -7), (130, 41)),
    ((5, 5), (5, 5)),
)


@pytest.mark.parametrize("grid", GRIDS, ids=lambda g: f"cell{g.cell}")
@pytest.mark.parametrize("segment", SEGMENTS, ids=lambda s: str(s))
def test_every_cell_a_segment_really_touches_is_marked(grid, segment):
    start, end = segment
    marked = set(grid.segment_cells(start, end))
    truth = {
        cell
        for cell in _all_cells(grid)
        if _segment_meets_cell(grid, cell, start, end)
    }
    assert truth <= marked, f"пропущены ячейки: {sorted(truth - marked)}"


@pytest.mark.parametrize("grid", GRIDS, ids=lambda g: f"cell{g.cell}")
def test_a_box_query_covers_every_cell_it_overlaps(grid):
    marked = set(grid.box_cells(Fraction(-1, 2), Fraction(21, 2), 0, 9))
    truth = {
        cell
        for cell in _all_cells(grid)
        if _boxes_overlap(
            (Fraction(-1, 2), Fraction(21, 2), 0, 9),
            (
                grid.x_min + cell[0] * grid.cell,
                grid.x_min + (cell[0] + 1) * grid.cell,
                grid.y_min + cell[1] * grid.cell,
                grid.y_min + (cell[1] + 1) * grid.cell,
            ),
        )
    }
    assert truth <= marked


def test_fractional_bounds_are_rounded_outward_not_to_the_nearest():
    """Округление наружу — включение, а округление к ближайшему — потеря."""

    grid = CellGridV1(0, 0, 32, 32, 8)
    # Точка 7.9 лежит в нулевом столбце, 8.1 — в первом. Запрос по интервалу
    # обязан отдать оба, а не «в среднем восьмёрку».
    assert set(grid.box_cells(Fraction(79, 10), Fraction(81, 10), 0, 0)) == {
        (0, 0),
        (1, 0),
    }
    assert grid.column(Fraction(-1, 10)) == -1


def test_the_declared_cell_budget_is_respected_by_the_grid_it_builds():
    """Функция объявляет границу — граница проверяется на её же результате."""

    for box in ((0, 0, 12000, 10), (-4096, -4096, 4096, 4096), (0, 0, 1, 1)):
        for targets in (1, 16, 512, 4096):
            grid = CellGridV1.covering(box, targets=targets)
            assert grid.columns * grid.rows <= max(targets, 4)
            assert grid.cell >= 1
            assert grid.cell & (grid.cell - 1) == 0, "сторона не степень двойки"


def test_an_empty_region_is_a_named_refusal_not_a_zero_sized_grid():
    with pytest.raises(CellGridRejected):
        CellGridV1.covering((10, 0, 0, 10), targets=16)


def test_a_query_that_leaves_the_region_is_reported_and_not_silently_clipped():
    """`contains_box` — единственное честное «мой ответ неполон»."""

    grid = CellGridV1(0, 0, 64, 64, 8)
    assert grid.contains_box(0, 64, 0, 64)
    assert not grid.contains_box(-1, 64, 0, 64)
    assert not grid.contains_box(0, 65, 0, 64)
    # Клип при этом всё равно происходит, поэтому вызывающий обязан спросить.
    assert grid.box_cells(-100, -50, 0, 8) == ()


def test_the_index_returns_a_sorted_union_so_the_order_is_reproducible():
    index = CellIndexV1()
    index.add(7, ((0, 0), (1, 0)))
    index.add(3, ((1, 0),))
    index.add(5, ((9, 9),))
    assert index.lookup(((1, 0),)) == (3, 7)
    assert index.lookup(((0, 0), (1, 0))) == (3, 7)
    assert index.lookup(((4, 4),)) == ()
