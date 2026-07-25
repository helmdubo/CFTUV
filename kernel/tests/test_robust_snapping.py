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
    GridWindowOutcome,
    grid_window_for_patch,
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


def test_a_grid_that_restores_degeneracy_can_still_destroy_real_detail():
    """Две границы, а не одна — иначе выбор шага делается по половине картины.

    «Вырождение восстановлено» меряет, слились ли точки, которые ОБЯЗАНЫ были
    слиться. Оно ничего не говорит про точки, которые обязаны были остаться
    разными. Стенд подбора масштаба сначала мерил только первое, и группы в
    нём были разнесены на 0.67 м — поэтому даже шаг 6 см проходил как годный,
    хотя на нём две настоящие детали в 1 см становятся одной.
    """

    from fractions import Fraction

    from cftuv_envelope.robust.snapping import grid_for_extent, snap_offset

    contour = [
        (Fraction(0), Fraction(0)),
        (Fraction(4), Fraction(0)),
        (Fraction(4), Fraction(4)),
        (Fraction(0), Fraction(4)),
    ]
    noise = Fraction(1, 10**4)
    detail = Fraction(1, 100)

    def merges(grid, distance):
        left = snap_offset(Fraction(2), Fraction(2), grid)
        right = snap_offset(Fraction(2) + distance, Fraction(2), grid)
        return (left.x, left.y) == (right.x, right.y)

    coarse = grid_for_extent(contour, nodes_across=64)
    assert merges(coarse, noise), "крупный шаг обязан сливать шум"
    assert merges(coarse, detail), (
        "и он же уничтожает настоящую деталь в 1 см — это и есть верхняя граница"
    )

    fine = grid_for_extent(contour, nodes_across=65536)
    assert not merges(fine, noise), "мелкий шаг шум не сливает — нижняя граница"
    assert not merges(fine, detail)

    # Между границами есть шаг, делающий и то и другое правильно. Если его нет,
    # решётка для этих данных не настройка, а именованный отказ.
    inside = grid_for_extent(contour, nodes_across=4096)
    assert merges(inside, noise)
    assert not merges(inside, detail)


def test_snapping_a_computed_point_merges_only_by_chance():
    """Слияние вычисленной точки — вероятность, а не свойство.

    Первая редакция R1b обосновывала привязку смещений и пересечений тем, что
    она восстанавливает задуманное вырождение. Замер это опроверг: доля
    положений, в которых пара на расстоянии d сливается при шаге step, идёт как
    1 − d/step. При step/d = 1.2 сливается 18.5%, при 9.8 — 90%.

    Значит надёжное слияние требует шага в сотню раз крупнее шума, а деталь
    декали требует шага мельче себя. Для шума 1e-4 и детали 1 мм окна нет.
    Восстановление вырождения обязано идти от ИСТОЧНИКА, а привязка
    вычисленных точек оправдывается только устранением радикалов.
    """

    from fractions import Fraction

    from cftuv_envelope.robust.grid import GridSpecV1, snap_point

    separation = Fraction(1, 10**4)

    def merge_fraction(step: Fraction, samples: int = 200) -> float:
        grid = GridSpecV1(scale=int(1 / step))
        merged = 0
        for index in range(samples):
            base = Fraction(2) + step * Fraction(index, samples)
            left = snap_point(base, Fraction(2), grid)
            right = snap_point(base + separation, Fraction(2), grid)
            if (left.x, left.y) == (right.x, right.y):
                merged += 1
        return merged / samples

    # Шаг мельче расстояния — не сливает никогда.
    assert merge_fraction(separation / 2) == 0.0
    # Шаг вдвое крупнее — сливает примерно в половине положений, не всегда.
    half = merge_fraction(separation * 2)
    assert 0.3 < half < 0.8, half
    # Даже вдесятеро крупнее — не гарантия.
    assert merge_fraction(separation * 10) < 1.0


# --------------------------------------------------------------------------
# Окно шага: две границы и именованный отказ при их пересечении
# --------------------------------------------------------------------------


FIELD_ANGLE = Fraction(7, 10**6)
DECAL_DETAIL = Fraction(1, 1000)


def _window(extent):
    return grid_window_for_patch(
        extent=extent,
        author_angular_error=FIELD_ANGLE,
        decal_detail=DECAL_DETAIL,
    )


def test_building_002_scale_has_a_window():
    """Габарит 4 м при полевом угле 7e-6 и детали 1 мм — запас есть."""

    window = _window(4)

    assert window.outcome is GridWindowOutcome.WINDOW_AVAILABLE
    assert window.lower_bound < window.upper_bound
    assert window.grid is not None


def test_a_large_patch_closes_the_window_and_that_is_a_named_refusal():
    """На 100 м окна нет вовсе: шум растёт с габаритом, деталь декали — нет.

    Это отказ, а не повод взять шаг покрупнее: крупный шаг съест деталь, ради
    которой декаль и рисуется.
    """

    window = _window(100)

    assert window.outcome is GridWindowOutcome.GRID_WINDOW_CLOSED
    assert window.lower_bound > window.upper_bound
    assert window.grid is None, "у закрытого окна не может быть решётки"


def test_the_window_closes_monotonically_with_patch_size():
    """Граница одна и она предсказуема, а не подбирается на глаз."""

    open_extents = [
        extent
        for extent in (1, 4, 10, 40, 100, 400)
        if _window(extent).outcome is GridWindowOutcome.WINDOW_AVAILABLE
    ]

    assert open_extents == sorted(open_extents)
    assert 4 in open_extents and 100 not in open_extents


def test_step_sits_at_the_lower_bound_to_spare_the_detail():
    """Шаг берётся у нижней границы: чинит вход и минимально трогает деталь."""

    window = _window(4)
    step = Fraction(1, window.grid.scale)

    assert step <= window.upper_bound


def test_degenerate_inputs_are_errors_not_silent_defaults():
    with pytest.raises(ValueError):
        grid_window_for_patch(
            extent=0, author_angular_error=FIELD_ANGLE, decal_detail=DECAL_DETAIL
        )
    with pytest.raises(ValueError):
        grid_window_for_patch(
            extent=4, author_angular_error=FIELD_ANGLE, decal_detail=Fraction(0)
        )


def test_the_returned_grid_lies_inside_the_window_it_reports():
    """Окно считалось верно и не использовалось — вот проверка на это.

    Первая редакция округляла масштаб вверх, то есть шаг вниз, и при окне
    5.6e-05 … 1.0e-03 возвращала 3.05e-05. Все тесты проходили, потому что ни
    один не сверял возвращённую решётку с собственными границами функции.
    """

    from fractions import Fraction

    from cftuv_envelope.robust.snapping import (
        GridWindowOutcome,
        grid_window_for_patch,
    )

    for extent in (1, 4, 10, 40):
        window = grid_window_for_patch(
            extent=extent,
            author_angular_error=Fraction(7, 10**6),
            decal_detail=Fraction(1, 1000),
        )
        if window.outcome is not GridWindowOutcome.WINDOW_AVAILABLE:
            continue
        step = Fraction(1, window.grid.scale)
        assert window.lower_bound <= step <= window.upper_bound, (
            f"габарит {extent}: шаг {float(step):.3e} вне окна "
            f"{float(window.lower_bound):.3e}…{float(window.upper_bound):.3e}"
        )
        # И то условие, ради которого нижняя граница вообще существует.
        author_error = Fraction(7, 10**6) * extent
        assert author_error < step / 2, (
            f"габарит {extent}: ошибка {float(author_error):.3e} не меньше "
            f"половины ячейки {float(step / 2):.3e} — вход не чинится"
        )


def test_a_window_with_no_power_of_two_inside_is_named_not_forced():
    """Границы разошлись, но степени двойки между ними нет — отдельный исход."""

    from fractions import Fraction

    from cftuv_envelope.robust.snapping import (
        GridWindowOutcome,
        grid_window_for_patch,
    )

    # lower = 2 * 3e-4 * 1 = 6e-4; upper = 9e-4. Между ними степеней двойки нет:
    # 1/2048 = 4.88e-4 ниже, 1/1024 = 9.77e-4 выше.
    window = grid_window_for_patch(
        extent=1,
        author_angular_error=Fraction(3, 10**4),
        decal_detail=Fraction(9, 10**4),
    )
    assert window.lower_bound < window.upper_bound, "вещественное окно открыто"
    assert window.outcome is GridWindowOutcome.NO_POWER_OF_TWO_STEP_IN_WINDOW
    assert window.grid is None
