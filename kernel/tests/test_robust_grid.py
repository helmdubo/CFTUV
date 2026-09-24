"""Доказательства для целочисленного ядра предикатов.

Здесь не «проверяется, что работает» — здесь доказывается три вещи, на которых
стоит решение уйти на решётку:

1. целочисленный предикат даёт ТОТ ЖЕ ответ, что нынешний точный путь;
2. граница, до которой binary64 точен, названа верно и **тесна** — то есть
   отказ от float обоснован замером, а не вкусом;
3. привязка к решётке детерминирована и записывает свой сдвиг точной дробью.
"""

from __future__ import annotations

import ast
import math
import random
from fractions import Fraction
from pathlib import Path

import pytest

from cftuv_envelope.reference.planar_types import (
    ExactPlanarPoint,
    ExactPlanarVector,
    cross,
    dot,
    exact_sign,
    point_sub,
)
from cftuv_envelope.robust import (
    DOUBLE_EXACT_MAGNITUDE_BOUND,
    GridSpecV1,
    SnapOutcome,
    cross_sign,
    dot_sign,
    on_segment,
    orient2d,
    point_in_loop,
    segment_intersection,
    snap_point,
    snap_value,
)


GRID = GridSpecV1(scale=1024)


def _random_points(seed: int, count: int, bound: int) -> list[tuple[int, int]]:
    generator = random.Random(seed)
    return [
        (generator.randint(-bound, bound), generator.randint(-bound, bound))
        for _ in range(count)
    ]


def _float_orient(a, b, c) -> int:
    value = (b[0] - a[0]) * float(c[1] - a[1]) - (b[1] - a[1]) * float(c[0] - a[0])
    return (value > 0.0) - (value < 0.0)


def _unimodular_triples(bound_bits: int, count: int, seed: int):
    """Тройки с определителем РОВНО ±1 при больших координатах.

    Это худший вход для binary64 и единственный, который что-то доказывает.
    Первая версия этого теста брала случайные точки и «почти коллинеарные» —
    у них определитель большой, и float отвечает верно даже при B=40. Тест
    показывал ноль расхождений и не доказывал ничего.
    """

    generator = random.Random(seed)
    triples = []
    while len(triples) < count:
        dx = generator.randrange(1 << (bound_bits - 1), 1 << bound_bits)
        dy = generator.randrange(1 << (bound_bits - 1), 1 << bound_bits)
        if math.gcd(dx, dy) != 1 or dy <= 1:
            continue
        inverse = pow(dx, -1, dy)
        second = ((dx * inverse - 1) // dy, inverse)
        if second[0] <= 0 or second[1] <= 0:
            continue
        triples.append(((0, 0), (dx, dy), second))
    return triples


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_integer_orient2d_matches_the_exact_evaluator(seed: int):
    """Замена законна ровно настолько, насколько совпадает с прежним путём."""

    points = _random_points(seed, 600, 1 << 20)
    for index in range(0, len(points) - 2, 3):
        a, b, c = points[index], points[index + 1], points[index + 2]
        exact = exact_sign(
            cross(
                point_sub(
                    ExactPlanarPoint.from_values(*b), ExactPlanarPoint.from_values(*a)
                ),
                point_sub(
                    ExactPlanarPoint.from_values(*c), ExactPlanarPoint.from_values(*a)
                ),
            )
        )
        assert orient2d(a, b, c) == exact


@pytest.mark.parametrize("seed", [3, 4])
def test_integer_dot_and_cross_match_the_exact_evaluator(seed: int):
    points = _random_points(seed, 400, 1 << 20)
    for index in range(0, len(points) - 1, 2):
        u, v = points[index], points[index + 1]
        left = ExactPlanarVector.from_values(*u)
        right = ExactPlanarVector.from_values(*v)
        assert cross_sign(u, v) == exact_sign(cross(left, right))
        assert dot_sign(u, v) == exact_sign(dot(left, right))


def test_the_double_bound_holds_where_it_is_claimed():
    """При B = 25 binary64 обязан совпасть с целым на худшем входе."""

    triples = _unimodular_triples(DOUBLE_EXACT_MAGNITUDE_BOUND, 2000, seed=7)
    divergent = sum(
        1 for a, b, c in triples if orient2d(a, b, c) != _float_orient(a, b, c)
    )
    assert divergent == 0


@pytest.mark.parametrize("bound_bits", [27, 30, 40])
def test_the_double_bound_is_tight_not_generous(bound_bits: int):
    """За границей binary64 обязан ломаться — иначе граница выбрана наугад.

    Это не проверка float (его здесь не используют), а проверка обоснования:
    если бы за границей расхождений не было, отказ от фильтра был бы
    предпочтением, а не выводом.
    """

    triples = _unimodular_triples(bound_bits, 2000, seed=11)
    divergent = sum(
        1 for a, b, c in triples if orient2d(a, b, c) != _float_orient(a, b, c)
    )
    assert divergent > 0


def test_integer_predicates_are_exact_far_beyond_the_double_bound():
    """У целочисленного пути границы нет вовсе — в этом весь смысл выбора."""

    for a, b, c in _unimodular_triples(120, 200, seed=13):
        assert orient2d(a, b, c) == 1
        assert orient2d(a, c, b) == -1
        assert orient2d(a, b, (2 * b[0], 2 * b[1])) == 0


def test_snapping_records_its_own_displacement():
    grid = GridSpecV1(scale=100)
    on_node = snap_point(Fraction(3, 100), Fraction(-7, 100), grid)
    assert (on_node.x, on_node.y) == (3, -7)
    assert on_node.squared_displacement == 0
    assert on_node.outcome is SnapOutcome.ON_GRID

    moved = snap_point(Fraction(1, 3), Fraction(1, 7), grid)
    assert moved.squared_displacement > 0
    assert moved.outcome is SnapOutcome.SNAPPED_WITHIN_BUDGET
    # Невязка обязана быть точной дробью, а не float: она идёт в дайджест.
    assert isinstance(moved.squared_displacement, Fraction)

    refused = snap_point(
        Fraction(1, 3), Fraction(1, 7), grid, squared_budget=Fraction(1, 10**12)
    )
    assert refused.outcome is SnapOutcome.SNAP_DISPLACEMENT_EXCEEDS_BUDGET


def test_snapping_is_deterministic_at_the_half_step():
    """Банковское округление в дайджесте читается как случайность. Его тут нет."""

    grid = GridSpecV1(scale=2)
    assert snap_value(Fraction(1, 4), grid) == 1
    assert snap_value(Fraction(3, 4), grid) == 2
    assert snap_value(Fraction(5, 4), grid) == 3
    assert snap_value(Fraction(-1, 4), grid) == 0
    assert snap_value(Fraction(-3, 4), grid) == -1


def test_segment_intersection_lands_on_the_grid_and_reports_the_shift():
    crossing = segment_intersection((0, 0), (4, 4), (0, 4), (4, 0), GRID)
    assert crossing is not None
    assert (crossing.x, crossing.y) == (2 * GRID.scale, 2 * GRID.scale)
    assert crossing.outcome is SnapOutcome.ON_GRID

    assert segment_intersection((0, 0), (4, 0), (0, 1), (4, 1), GRID) is None
    assert segment_intersection((0, 0), (1, 1), (5, 5), (6, 6), GRID) is None
    # Касание концом — законное пересечение, а не «почти».
    touching = segment_intersection((0, 0), (4, 4), (4, 4), (8, 0), GRID)
    assert touching is not None


def test_point_in_loop_agrees_with_a_brute_force_ray_cast():
    loop = ((0, 0), (10, 0), (10, 10), (5, 4), (0, 10))
    assert point_in_loop((5, 1), loop) == 1
    assert point_in_loop((5, 9), loop) == -1
    assert point_in_loop((0, 0), loop) == 0
    assert point_in_loop((5, 0), loop) == 0
    assert point_in_loop((-1, 5), loop) == -1
    assert point_in_loop((20, 5), loop) == -1


def test_on_segment_needs_collinearity_first():
    assert on_segment((2, 2), (0, 0), (4, 4))
    assert on_segment((0, 0), (0, 0), (4, 4))
    assert not on_segment((2, 3), (0, 0), (4, 4))
    assert not on_segment((6, 6), (0, 0), (4, 4))


def test_the_robust_core_never_imports_sympy():
    """Обещание пакета исполняемое, а не декларативное.

    Один `import sympy` внутри вернул бы сюда и машину предположений, и
    строковое представление числа — то есть ровно то, ради ухода от чего
    решётка и заводилась.
    """

    root = Path(__file__).resolve().parents[1] / "src" / "cftuv_envelope" / "robust"
    for path in sorted(root.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            for name in names:
                assert not name.startswith(("sympy", "mpmath")), (
                    f"{path.name} импортирует {name}"
                )


def test_merged_points_counts_distinct_sources_not_repeated_snaps():
    """`merged_points` — величина, по которой владелец выбирает шаг решётки.

    Первая редакция считала слиянием любую привязку в занятый узел. Общий конец
    двух соседних сегментов привязывается дважды, поэтому счётчик показывал бы
    примерно число сегментов при ЛЮБОМ масштабе — то есть сигнал, по которому
    надо настраивать, был бы неотличим от шума.
    """

    from cftuv_envelope.robust import grid as grid_module

    fine = GridSpecV1(scale=100)

    grid_module.reset_snap_counts()
    snap_point(Fraction(1, 2), Fraction(1, 2), fine)
    snap_point(Fraction(1, 2), Fraction(1, 2), fine)
    assert grid_module.SNAP_COUNTS["points_total"] == 2
    assert grid_module.SNAP_COUNTS["merged_points"] == 0, (
        "повторная привязка той же точки — не слияние"
    )

    grid_module.reset_snap_counts()
    snap_point(Fraction(1, 2), Fraction(1, 2), fine)
    snap_point(Fraction(1, 2) + Fraction(1, 10**6), Fraction(1, 2), fine)
    assert grid_module.SNAP_COUNTS["merged_points"] == 1, (
        "две различные точки в одном узле — ровно одно слияние"
    )

    # Третья различная точка в том же узле — ещё одно слияние, не два.
    snap_point(Fraction(1, 2) + Fraction(1, 10**7), Fraction(1, 2), fine)
    assert grid_module.SNAP_COUNTS["merged_points"] == 2

    # На мелкой решётке те же точки расходятся по узлам — слияний нет.
    grid_module.reset_snap_counts()
    coarse = GridSpecV1(scale=10**9)
    snap_point(Fraction(1, 2), Fraction(1, 2), coarse)
    snap_point(Fraction(1, 2) + Fraction(1, 10**6), Fraction(1, 2), coarse)
    assert grid_module.SNAP_COUNTS["merged_points"] == 0, (
        "иначе счётчик не реагирует на масштаб, а именно это от него и нужно"
    )


def test_snap_counters_reset_at_the_domain_boundary():
    from cftuv_envelope.robust import grid as grid_module

    grid_module.reset_snap_counts()
    snap_point(Fraction(1, 3), Fraction(1, 7), GRID)
    assert grid_module.SNAP_COUNTS["points_total"] == 1
    assert grid_module.SNAP_DISPLACEMENT_MAX_SQUARED[0] > 0
    grid_module.reset_snap_counts()
    assert grid_module.SNAP_COUNTS["points_total"] == 0
    assert grid_module.SNAP_DISPLACEMENT_MAX_SQUARED[0] == 0
