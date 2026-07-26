"""Покрытие к моменту alpha: обе объявленные границы на СОБСТВЕННОМ результате.

`coverage.py` объявляет ровно две границы, и здесь по столбцу на каждую:

| граница | что проверяется |
|---|---|
| 1. покрытие не убывает по alpha | `test_coverage_never_decreases_as_alpha_grows` |
| 2. грань не превзойдена, сумма не превзошла область | `test_coverage_never_exceeds_...` |

Плюс значения, известные руками: у квадрата 8x8 покрытие при alpha 1, 2, 4 равно
28, 48, 64, а у прямоугольного треугольника с катетом 12 при alpha = 1 оно равно
`21 + 10*sqrt(2)` — ровно то, что независимо выдаёт `RawCoverage` попарного пути.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.coverage import (
    CoverageOutcome,
    clip_to_halfplane,
    coverage_at,
)
from cftuv_envelope.wavefront.event_time import SupportLineV1
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces, line_key, polygon_edges
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from wavefront_cases import named_corpus


def _mapped_corpus():
    for name, polygon in named_corpus():
        keys = [line_key(line) for _, _, line in polygon_edges(polygon)]
        if len(set(keys)) == len(keys):
            yield name, polygon


CORPUS = tuple(_mapped_corpus())
CORPUS_IDS = tuple(name for name, _ in CORPUS)


def _partition(polygon):
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    return partition


def _alpha_ladder(polygon) -> tuple[Fraction, ...]:
    """Лестница alpha от нуля до заведомо полного покрытия.

    Верхняя ступень — габарит области: фронт с единичной скоростью накрывает её
    не позже. Значение выведено, а не подобрано.
    """

    xs = [x for loop in polygon.loops for x, _ in loop.points]
    ys = [y for loop in polygon.loops for _, y in loop.points]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    return tuple(
        Fraction(span * step, 8) for step in (0, 1, 2, 3, 5, 8)
    )


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_coverage_never_decreases_as_alpha_grows(name, polygon):
    """Граница 1 на собственном результате: рост фронта не обратим."""

    partition = _partition(polygon)
    previous = None
    for alpha in _alpha_ladder(polygon):
        coverage = coverage_at(partition, alpha)
        assert coverage.outcome is CoverageOutcome.EXACT
        if previous is not None:
            growth = coverage.doubled_area - previous
            assert growth.sign() >= 0, f"{name}: покрытие убыло при alpha={alpha}"
        previous = coverage.doubled_area


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_coverage_never_exceeds_its_face_nor_the_polygon(name, polygon):
    """Граница 2 на собственном результате: ни грань, ни область не превзойдены."""

    partition = _partition(polygon)
    face_area = {face.owner: face.doubled_area for face in partition.faces}
    for alpha in _alpha_ladder(polygon):
        coverage = coverage_at(partition, alpha)
        assert coverage.does_not_exceed_the_polygon, f"{name}: alpha={alpha}"
        for face in coverage.faces:
            slack = face_area[face.owner] - face.doubled_area
            assert slack.sign() >= 0, f"{name}: грань {face.owner}, alpha={alpha}"
            assert face.doubled_area.sign() >= 0


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_a_large_enough_alpha_covers_the_whole_polygon_exactly(name, polygon):
    """Габарит области покрывается целиком, и это ТОЧНОЕ равенство площадей.

    Это и есть тот же вопрос, который попарному крою задаёт
    `policy_b.py:976`: воспроизводит ли разбиение исходное множество.
    """

    partition = _partition(polygon)
    alpha = _alpha_ladder(polygon)[-1]
    coverage = coverage_at(partition, alpha)
    assert coverage.covers_everything, (
        f"{name}: при alpha={alpha} недобор "
        f"{(SqrtSumV1.rational(coverage.polygon_doubled_area) - coverage.doubled_area).terms}"
    )


def test_the_axis_square_coverage_is_the_hand_computed_ladder():
    """Значения руками: 8x8 при alpha 1, 2, 4 даёт 28, 48, 64."""

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    partition = _partition(polygon)
    got = {
        int(alpha): coverage_at(partition, Fraction(alpha)).doubled_area.as_rational()
        for alpha in (1, 2, 4)
    }
    assert got == {1: Fraction(56), 2: Fraction(96), 4: Fraction(128)}


def test_the_right_triangle_coverage_is_exactly_twenty_one_plus_ten_root_two():
    """Значение руками и независимо: `RawCoverage` даёт ровно `21 + 10*sqrt(2)`.

    Совпадение не «в пределах точности», а побитовое: канонический набор
    коэффициентов совпадает целиком.
    """

    polygon = PolygonV1.build(((0, 0), (12, 0), (0, 12)))
    coverage = coverage_at(_partition(polygon), Fraction(1))
    assert coverage.doubled_area.terms == (
        (1, Fraction(42)),
        (2, Fraction(20)),
    )


def test_a_zero_alpha_covers_exactly_nothing():
    """Фронт ещё не пошёл — покрытия нет, и это ноль доказанный, а не малый."""

    polygon = PolygonV1.build(((0, 0), (12, 0), (0, 12)))
    coverage = coverage_at(_partition(polygon), Fraction(0))
    assert coverage.doubled_area.is_zero
    assert not coverage.covers_everything


def test_a_negative_alpha_is_a_named_outcome_not_a_negative_area():
    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    coverage = coverage_at(_partition(polygon), Fraction(-1))
    assert coverage.outcome is CoverageOutcome.ALPHA_IS_NEGATIVE
    assert coverage.faces == ()


def test_the_coverage_of_an_unproven_partition_is_refused_not_invented():
    from dataclasses import replace

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    partition = replace(
        _partition(polygon),
        outcome=FaceOutcome.SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES,
    )
    coverage = coverage_at(partition, Fraction(1))
    assert coverage.outcome is CoverageOutcome.PARTITION_IS_NOT_EXACT
    assert coverage.doubled_area.is_zero


def test_the_halfplane_clip_keeps_a_polygon_it_does_not_touch():
    """Резак, не задевший контур, обязан вернуть его без изменений."""

    square = tuple(
        (SqrtSumV1.rational(x), SqrtSumV1.rational(y))
        for x, y in ((0, 0), (4, 0), (4, 4), (0, 4))
    )
    line = SupportLineV1.through((0, 0), (4, 0))
    assert clip_to_halfplane(square, line, Fraction(100)) == square
    assert clip_to_halfplane(square, line, Fraction(0)) == ()


def test_the_halfplane_clip_halves_a_square_exactly():
    from cftuv_envelope.wavefront.faces import doubled_shoelace

    square = tuple(
        (SqrtSumV1.rational(x), SqrtSumV1.rational(y))
        for x, y in ((0, 0), (4, 0), (4, 4), (0, 4))
    )
    line = SupportLineV1.through((0, 0), (4, 0))
    clipped = clip_to_halfplane(square, line, Fraction(2))
    assert doubled_shoelace(clipped).as_rational() == Fraction(16)
