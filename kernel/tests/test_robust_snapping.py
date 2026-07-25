"""Привязка к решётке: доказать до подключения, а не после.

Функции среза R1a ещё никем не вызываются. Тесты здесь проверяют именно их, а не
конвейер: подключение — срез R1b, и он двинет дайджесты. Если доказательства
отложить до подключения, нельзя будет сказать, что дайджест сдвинула привязка,
а не что-то рядом.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.robust.grid import (
    SNAP_COUNTS,
    SnapOutcome,
    GridSpecV1,
    reset_snap_counts,
    unsnap_value,
)
from cftuv_envelope.robust.snapping import (
    GridSnappingLawV1,
    grid_for_extent,
    law_for,
    snap_intersection,
    snap_offset,
)


@pytest.fixture(autouse=True)
def _clean_counters():
    reset_snap_counts()
    yield
    reset_snap_counts()


def _unit_square():
    return ((0, 0), (1, 0), (1, 1), (0, 1))


# --------------------------------------------------------------------------
# Решётка из габарита
# --------------------------------------------------------------------------


def test_grid_scale_covers_the_requested_node_count():
    grid = grid_for_extent(_unit_square(), nodes_across=1000)

    assert grid.scale >= 1000
    assert grid.scale & (grid.scale - 1) == 0, (
        "масштаб обязан быть степенью двойки: иначе привязка вносит новый "
        "знаменатель и удлиняет числа, которые призвана укоротить"
    )


def test_grid_scale_follows_the_larger_side():
    """Габарит берётся по большей стороне, иначе узкий патч получит грубый шаг."""

    wide = grid_for_extent(((0, 0), (100, 0), (100, 1)), nodes_across=100)
    small = grid_for_extent(_unit_square(), nodes_across=100)

    assert wide.scale < small.scale


def test_degenerate_extent_does_not_invent_a_scale():
    """Нулевой габарит — не повод угадывать. Единичная решётка видна в сертификате."""

    assert grid_for_extent(((3, 4), (3, 4)), nodes_across=1000).scale == 1


def test_empty_extent_and_bad_node_count_are_errors_not_guesses():
    with pytest.raises(ValueError):
        grid_for_extent((), nodes_across=10)
    with pytest.raises(ValueError):
        grid_for_extent(_unit_square(), nodes_across=0)


# --------------------------------------------------------------------------
# Тест 1 карточки — невырожденный вход не искажается сверх половины ячейки
# --------------------------------------------------------------------------


def test_snapping_moves_no_point_further_than_half_a_cell():
    grid = GridSpecV1(scale=1024)
    cell = Fraction(1, grid.scale)

    for index in range(64):
        value = Fraction(index * 7919, 65536)
        snapped = snap_offset(value, -value, grid)
        for coordinate, exact in ((snapped.x, value), (snapped.y, -value)):
            assert abs(unsnap_value(coordinate, grid) - exact) <= cell / 2


def test_distinct_points_stay_distinct_when_the_step_is_finer_than_the_gap():
    """Топология сохраняется: шаг мельче всех расстояний — слияний нет."""

    grid = GridSpecV1(scale=1 << 20)
    points = [(Fraction(index, 1000), Fraction(index * index, 1000)) for index in range(32)]

    nodes = {(snap_offset(x, y, grid).x, snap_offset(x, y, grid).y) for x, y in points}

    assert len(nodes) == len(points)
    assert SNAP_COUNTS["merged_points"] == 0


# --------------------------------------------------------------------------
# Тест 2 карточки — задуманное вырождение восстанавливается
# --------------------------------------------------------------------------


def test_three_fronts_meant_to_meet_collapse_into_one_node():
    """Три точки, разнесённые шумом на 1e-9, ложатся в один узел.

    Это и есть bf6 в миниатюре: контур задуман сходящимся, binary64 разносит
    вершины, точная арифметика добросовестно считает шум. Решётка возвращает
    вырождение структурно, а не допуском в предикате.
    """

    noise = Fraction(1, 10**9)
    meeting = Fraction(1, 2)
    grid = grid_for_extent(_unit_square(), nodes_across=1000)

    nodes = {
        (snap_offset(meeting + offset, meeting - offset, grid).x,
         snap_offset(meeting + offset, meeting - offset, grid).y)
        for offset in (-noise, Fraction(0), noise)
    }

    assert len(nodes) == 1
    assert SNAP_COUNTS["merged_points"] > 0


def test_a_fine_grid_keeps_the_noise_apart():
    """Обратная сторона: слишком мелкий шаг вырождение НЕ восстанавливает.

    Это натяжение и есть предмет выбора масштаба; тест держит его видимым, чтобы
    «решётка всё чинит» не стало молчаливым допущением.
    """

    noise = Fraction(1, 10**9)
    grid = GridSpecV1(scale=1 << 40)
    meeting = Fraction(1, 2)

    nodes = {
        snap_offset(meeting + offset, meeting, grid).x
        for offset in (-noise, Fraction(0), noise)
    }

    assert len(nodes) == 3


# --------------------------------------------------------------------------
# Тест 3 карточки — бюджет срабатывает, а не сглаживает
# --------------------------------------------------------------------------


def test_budget_exceeded_is_a_named_outcome_not_a_silent_move():
    grid = GridSpecV1(scale=2)
    far_from_a_node = Fraction(1, 4)

    snapped = snap_offset(far_from_a_node, far_from_a_node, grid, squared_budget=Fraction(1, 10**6))

    assert snapped.outcome is SnapOutcome.SNAP_DISPLACEMENT_EXCEEDS_BUDGET
    assert snapped.squared_displacement > 0


def test_point_already_on_the_grid_reports_on_grid():
    grid = GridSpecV1(scale=8)

    snapped = snap_intersection(Fraction(3, 8), Fraction(-5, 8), grid)

    assert snapped.outcome is SnapOutcome.ON_GRID
    assert snapped.squared_displacement == 0
    assert SNAP_COUNTS["points_moved"] == 0


# --------------------------------------------------------------------------
# Тест 5 карточки — регион с дырой, до поля
# --------------------------------------------------------------------------


def test_hole_contour_is_snapped_like_the_outer_one():
    """Внутренний контур привязывается наравне с внешним.

    `DOMAIN_HOLE_LOOPS = 0` во всех семи полевых доменах, то есть путь дыр не
    пройден в поле ни разу и поле его не покажет. Проверка обязана быть
    синтетической и стоять ДО подключения: обход только по `region.outer`
    потерял бы внутренний фронт молча.
    """

    grid = GridSpecV1(scale=16)
    outer = ((0, 0), (4, 0), (4, 4), (0, 4))
    hole = (
        (Fraction(1, 32), Fraction(1, 32)),
        (Fraction(63, 32), Fraction(1, 32)),
        (Fraction(63, 32), Fraction(63, 32)),
    )

    outer_nodes = {(snap_offset(x, y, grid).x, snap_offset(x, y, grid).y) for x, y in outer}
    hole_snapped = [snap_offset(x, y, grid) for x, y in hole]

    assert len(outer_nodes) == 4
    assert all(item.outcome is SnapOutcome.SNAPPED_WITHIN_BUDGET for item in hole_snapped)
    assert all(item.squared_displacement > 0 for item in hole_snapped), (
        "точки дыры лежат между узлами — если сдвиг нулевой, дыру не привязали"
    )


# --------------------------------------------------------------------------
# Тест 6 карточки — воспроизводимость и различимость
# --------------------------------------------------------------------------


def test_same_input_and_same_spec_give_the_same_node():
    grid = GridSpecV1(scale=256)
    value = Fraction(12345, 9973)

    first = snap_offset(value, value * 2, grid)
    second = snap_offset(value, value * 2, grid)

    assert (first.x, first.y, first.squared_displacement) == (
        second.x,
        second.y,
        second.squared_displacement,
    )


def test_different_specs_give_different_nodes():
    """Различимость — тоже проверка: иначе спецификацию можно не записывать."""

    value = Fraction(12345, 9973)

    coarse = snap_offset(value, value, GridSpecV1(scale=8))
    fine = snap_offset(value, value, GridSpecV1(scale=4096))

    assert unsnap_value(coarse.x, GridSpecV1(scale=8)) != unsnap_value(
        fine.x, GridSpecV1(scale=4096)
    )


# --------------------------------------------------------------------------
# Закон записывается, а не угадывается
# --------------------------------------------------------------------------


def test_every_snapping_site_declares_its_law():
    assert law_for("offset_support_g") is GridSnappingLawV1.OFFSET_CONSTRUCTION_V1
    assert law_for("segment_intersections") is (
        GridSnappingLawV1.SEGMENT_INTERSECTION_V1
    )


def test_an_undeclared_site_is_an_error_not_a_default():
    with pytest.raises(KeyError):
        law_for("somewhere_else")
