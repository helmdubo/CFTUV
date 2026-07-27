"""Грани скелета: обе объявленные границы проверяются на СОБСТВЕННОМ результате.

Правило проекта, поймавшее настоящий дефект пять раз: если функция объявляет
границу — тест обязан проверить, что её собственный результат этой границе
удовлетворяет. `faces.py` объявляет ровно две границы, и здесь по столбцу на
каждую:

| граница | что проверяется |
|---|---|
| 1. сумма площадей граней = площадь многоугольника | `area_reproduces_polygon` |
| 2. площадь каждой грани строго положительна | `every_face_is_positive` |

С этого среза границы проверяет и САМА `build_faces` перед возвратом `EXACT`, у
каждой границы свой член `FaceOutcome`, и в `detail` лежит ЧИСЛО потери. Ниже —
столбец тестов и на это: что отказ громкий, что число то самое, и что цепочка
до `coverage_at` закрыта (она не считает по неточному разбиению).
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.faces import (
    FaceOutcome,
    build_faces,
    doubled_shoelace,
    face_contour,
    line_key,
    polygon_edges,
    projection,
)
from cftuv_envelope.wavefront.polygon import PolygonV1, signed_double_area
from cftuv_envelope.wavefront.skeleton import SplitSearch
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

import wavefront_cases
from wavefront_cases import named_corpus


# Корпус, у которого несущие прямые рёбер различны. `holes_2` вынесен отдельным
# тестом: он и есть свидетельство исхода SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES.
def _distinct_line_corpus():
    for name, polygon in named_corpus():
        keys = [line_key(line) for _, _, line in polygon_edges(polygon)]
        if len(set(keys)) == len(keys):
            yield name, polygon


CORPUS = tuple(_distinct_line_corpus())
CORPUS_IDS = tuple(name for name, _ in CORPUS)


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_the_face_partition_reproduces_the_polygon_area_exactly(name, polygon):
    """Граница 1 на собственном результате: площадь сходится ТОЧНО."""

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert partition.area_reproduces_polygon, (
        f"{name}: дефект площади {partition.area_defect.terms}"
    )
    # Не «почти ноль», а пустой канонический набор коэффициентов.
    assert partition.area_defect.terms == ()


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_every_face_of_the_partition_has_strictly_positive_area(name, polygon):
    """Граница 2 на собственном результате: щепок и вывернутых граней нет."""

    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    assert partition.every_face_is_positive
    for face in partition.faces:
        assert face.doubled_area.sign() > 0, f"{name}: грань {face.owner}"


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_there_is_exactly_one_face_per_input_edge_and_owner_is_its_line(
    name, polygon
):
    """Owner выпадает из структуры: одна грань на ребро, ключ — его прямая."""

    partition = build_faces(polygon, build_skeleton(polygon))
    edges = polygon_edges(polygon)
    assert len(partition.faces) == len(edges)
    assert {face.owner for face in partition.faces} == {
        line_key(line) for _, _, line in edges
    }


def test_a_shared_support_line_is_a_named_outcome_not_a_wrong_area():
    """`holes_2`: коллинеарные рёбра дают ИСХОД, а не подогнанную площадь.

    Числа, ради которых тест существует: наивная сборка выдаёт удвоенную
    площадь 1584 при истинной 1344. Молча её принять означало бы поверить
    разбиению, которого нет.
    """

    polygon = next(
        item for name, item in named_corpus() if name == "holes_2"
    )
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES
    assert partition.faces == ()
    assert partition.polygon_doubled_area == 1344


def test_the_two_split_searches_give_the_same_faces():
    """Оптимизация законна ровно настолько, насколько даёт тот же ответ."""

    for name, polygon in CORPUS:
        motorcycle = build_faces(
            polygon, build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
        )
        exhaustive = build_faces(
            polygon, build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
        )
        assert motorcycle.outcome is exhaustive.outcome, name
        assert motorcycle.doubled_area == exhaustive.doubled_area, name
        assert {
            face.owner: face.doubled_area for face in motorcycle.faces
        } == {face.owner: face.doubled_area for face in exhaustive.faces}, name


def test_the_faces_of_an_unproven_skeleton_are_refused_not_invented():
    """Скелет не доказан — граней нет. Тихого нуля здесь быть не может."""

    from dataclasses import replace

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    skeleton = build_skeleton(polygon)
    broken = replace(
        skeleton, outcome=type(skeleton.outcome).WAVEFRONT_LEFT_UNRESOLVED
    )
    partition = build_faces(polygon, broken)
    assert partition.outcome is FaceOutcome.SKELETON_IS_NOT_EXACT
    assert partition.faces == ()
    assert partition.detail == "WAVEFRONT_LEFT_UNRESOLVED"


def test_a_partition_that_loses_area_refuses_loudly_with_the_defect_as_a_number():
    """Граница 1 нарушена — исход названный, и в нём ЧИСЛО, а не слово.

    Крест `5 x 4` теряет клин удвоенной площади 1. До этого среза `build_faces`
    отдавала на нём `EXACT`, и потеря протекала до самого ответа: граница 2
    держалась, а односторонняя проверка покрытия «не превысил многоугольник»
    проходила, потому что потеря делает покрытие МЕНЬШЕ.
    """

    figure = wavefront_cases.cross(wide=5, tall=4)
    partition = build_faces(figure, build_skeleton(figure))
    assert partition.outcome is (
        FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
    )
    assert partition.detail == "-1"
    assert partition.area_defect.as_rational() == Fraction(-1)
    # Граница 2 при этом ДЕРЖИТСЯ, и это ровно то, почему исходы разные: один
    # общий член слил бы «потерян клин» и «грань вывернута» в одно слово.
    assert partition.every_face_is_positive


def test_the_two_broken_boundaries_are_two_different_outcomes_not_one():
    """Отдельный член на каждую границу — иначе диагностика их путает.

    Вывернутая грань подделывается `replace` над готовой партицией: своей
    фигуры, ломающей ИМЕННО границу 2, в корпусе нет, а проверить надо, что
    членов два и они не подменяют друг друга.
    """

    from dataclasses import replace

    assert (
        FaceOutcome.FACE_IS_NOT_POSITIVE
        is not FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
    )
    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    flipped = replace(
        partition.faces[0], doubled_area=SqrtSumV1.rational(-32)
    )
    broken = replace(partition, faces=(flipped,) + partition.faces[1:])
    assert not broken.every_face_is_positive
    # Свойство границы 1 у подделки тоже сломано, и это как раз показывает,
    # почему `build_faces` спрашивает про положительность ПЕРВОЙ: корень —
    # одна грань, а расхождение суммы — его следствие.
    assert broken.area_reproduces_polygon


def test_an_inexact_partition_never_reaches_the_coverage_answer():
    """Цепочка закрыта ДОКАЗАННО: `coverage_at` отказывает, а не считает.

    Проверяется на фигуре, где разбиение действительно неточно, а не на
    подделке: крест `6 x 4`, удвоенный дефект −4. Раньше на ней покрытие
    считалось и отдавало 166 вместо эталонных 168.
    """

    figure = wavefront_cases.cross(wide=6, tall=4)
    partition = build_faces(figure, build_skeleton(figure))
    assert partition.outcome is (
        FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
    )
    for alpha in (Fraction(0), Fraction(1), Fraction(3)):
        coverage = coverage_at(partition, alpha)
        assert coverage.outcome is CoverageOutcome.PARTITION_IS_NOT_EXACT
        assert coverage.faces == ()
        assert coverage.doubled_area.is_zero
        assert coverage.detail == (
            FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON.value
        )


def test_the_shoelace_over_sqrt_sums_agrees_with_the_integer_one():
    """Площадь над `SqrtSumV1` — та же формула, а не вторая арифметика."""

    for points in (
        ((0, 0), (4, 0), (4, 3), (0, 3)),
        ((0, 0), (7, 0), (3, 5)),
        ((-2, -2), (5, -1), (4, 6), (-3, 3)),
    ):
        lifted = tuple(
            (SqrtSumV1.rational(x), SqrtSumV1.rational(y)) for x, y in points
        )
        assert doubled_shoelace(lifted).as_rational() == Fraction(
            signed_double_area(points)
        )


def test_the_face_area_of_the_axis_square_is_the_quarter_it_must_be():
    """Известный ответ руками: квадрат 8x8 даёт четыре равные грани по 16."""

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    assert [
        face.doubled_area.as_rational() for face in partition.faces
    ] == [Fraction(32)] * 4
    assert partition.doubled_area.as_rational() == Fraction(128)


# --------------------------------------------------------------------------
# Крест: измеренный диагноз дефекта, а не гипотеза о нём
#
# Ниже — стенд, который отвечает на вопрос «что именно теряет сборщик» ЧИСЛАМИ
# по всей сетке `wide, tall` из 3..9, а не на двух фигурах, попавших в тест.
# Три измерения, и каждое опровергает или подтверждает свою версию причины:
#
# | версия причины                     | измерение | вывод |
# |------------------------------------|-----------|-------|
# | «узлы с равной проекцией, порядок» | 0 пар различных точек с равной проекцией | ОПРОВЕРГНУТА |
# | «в списке лишний узел (состав)»    | правка ложится ровно на 4 грани поровну | не подтверждена |
# | «нужной вершины НЕТ среди узлов»   | точка пересечения гребней отсутствует во всех 42 | ПОДТВЕРЖДЕНА |
# --------------------------------------------------------------------------


# Дефект по всей сетке, измеренный, а не выведенный. Первые два числа — `wide`
# и `tall`, третье — удвоенный дефект площади. Диагональ `wide == tall` сюда не
# входит: там скелет отказывает `WAVEFRONT_LEFT_UNRESOLVED` и граней нет.
CROSS_GRID_DEFECT = (
    (3, 4, -1), (3, 5, -4), (3, 6, -9), (3, 7, -16), (3, 8, -20), (3, 9, -24),
    (4, 3, -1), (4, 5, -1), (4, 6, -4), (4, 7, -9), (4, 8, -16), (4, 9, -20),
    (5, 3, -4), (5, 4, -1), (5, 6, -1), (5, 7, -4), (5, 8, -9), (5, 9, -16),
    (6, 3, -9), (6, 4, -4), (6, 5, -1), (6, 7, -1), (6, 8, -4), (6, 9, -9),
    (7, 3, -16), (7, 4, -9), (7, 5, -4), (7, 6, -1), (7, 8, -1), (7, 9, -4),
    (8, 3, -20), (8, 4, -16), (8, 5, -9), (8, 6, -4), (8, 7, -1), (8, 9, -1),
    (9, 3, -24), (9, 4, -20), (9, 5, -16), (9, 6, -9), (9, 7, -4), (9, 8, -1),
)

# Толщина рукавов у `wavefront_cases.cross`: `left == bottom == 4`. Это не
# декорация, а граница области, где закон дефекта имеет силу, — см. тест ниже.
CROSS_ARM = 4

# Четыре стенки центрального блока и четыре его «крышки» в порядке рёбер
# `polygon_edges`. Пересечение двух гребней сидит между ними.
CROSS_WALLS = (1, 5, 7, 11)
CROSS_CAPS = (2, 4, 8, 10)


def _cross_partition(wide: int, tall: int):
    figure = wavefront_cases.cross(wide=wide, tall=tall)
    skeleton = build_skeleton(figure)
    return figure, skeleton, build_faces(figure, skeleton)


def _reassembled_total(figure, skeleton, extra):
    """Пересборка ТЕМ ЖЕ правилом, но с дополнительными точками на ребро.

    Правило берётся из `faces.py` (`face_contour`), а не переписывается здесь:
    стенд, доказывающий про свою копию правила, не доказывает ничего.
    """

    total = SqrtSumV1.zero()
    areas = {}
    for index, (start, end, line) in enumerate(polygon_edges(figure)):
        key = line_key(line)
        nodes = tuple(
            (node.point.x, node.point.y)
            for node in skeleton.nodes
            if key in node.participants
        ) + tuple(extra.get(index, ()))
        doubled = doubled_shoelace(face_contour(start, end, nodes))
        areas[index] = doubled
        total = total + doubled
    return total, areas


def test_the_cross_defect_is_measured_over_the_whole_grid_not_over_two_figures():
    """42 строки сетки, и у каждой дефект — записанное число.

    Смысл таблицы в том, что её нельзя подогнать: она не про две фигуры,
    попавшие в отчёт, а про всю сетку `wide, tall` из 3..9. Семь диагональных
    строк `wide == tall` в неё не входят — там отказывает скелет, а не сборщик.
    """

    measured = []
    for wide in range(3, 10):
        for tall in range(3, 10):
            _, _, partition = _cross_partition(wide, tall)
            if partition.outcome is FaceOutcome.SKELETON_IS_NOT_EXACT:
                assert wide == tall, (wide, tall, partition.detail)
                continue
            assert partition.outcome is (
                FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
            ), (wide, tall)
            measured.append(
                (wide, tall, partition.area_defect.as_rational())
            )
    assert len(measured) == 42
    assert measured == [
        (wide, tall, Fraction(defect))
        for wide, tall, defect in CROSS_GRID_DEFECT
    ]


def test_the_defect_law_holds_only_while_the_gap_fits_inside_the_arm():
    """`-(wide - tall)^2` — закон с ОБЛАСТЬЮ, и область измерена, а не додумана.

    Записанный ранее закон «удвоенный дефект = -(wide - tall)^2 на всех 42
    строках» верен на 36 из 42. Он ломается ровно там, где `|wide - tall|`
    перерастает толщину рукава (у `cross` она 4): 6 строк — (3,8), (3,9),
    (4,9), (8,3), (9,3), (9,4) — дают −20 и −24 вместо −25 и −36. Причина не в
    длине рукавов: те же 6 строк расходятся и при `right, top` = 40, 44 и 60,
    64. Причина в толщине: потерянный клин упирается в рукав и обрезается.
    """

    inside = [
        (wide, tall, defect)
        for wide, tall, defect in CROSS_GRID_DEFECT
        if abs(wide - tall) <= CROSS_ARM
    ]
    outside = [
        (wide, tall, defect)
        for wide, tall, defect in CROSS_GRID_DEFECT
        if abs(wide - tall) > CROSS_ARM
    ]
    assert len(inside) == 36 and len(outside) == 6
    assert all(defect == -((wide - tall) ** 2) for wide, tall, defect in inside)
    # Снаружи области закон не просто «неточен» — он завышает потерю, и на
    # сколько именно, тоже число.
    assert [
        (wide, tall, defect, -((wide - tall) ** 2))
        for wide, tall, defect in outside
    ] == [
        (3, 8, -20, -25),
        (3, 9, -24, -36),
        (4, 9, -20, -25),
        (8, 3, -20, -25),
        (9, 3, -24, -36),
        (9, 4, -20, -25),
    ]


def test_no_two_distinct_skeleton_points_of_a_cross_share_a_projection():
    """ОПРОВЕРЖЕНИЕ гипотезы «порядок ломается на равных проекциях».

    Равные проекции на кресте есть, и их много — до 26 пар на фигуру. Но все
    они между записями узлов, стоящими в ОДНОЙ И ТОЙ ЖЕ точке: из 18 узлов
    скелета креста 10 — повторы по точке. Пар РАЗЛИЧНЫХ точек с равной
    проекцией нет ни одной ни на одной из 42 строк, а порядок совпадающих
    точек на площадь не влияет никак. Значит переставлять нечего, и чинить
    сортировку бессмысленно.
    """

    coincident_ties = 0
    for wide, tall, _ in CROSS_GRID_DEFECT:
        figure, skeleton, _ = _cross_partition(wide, tall)
        for start, end, line in polygon_edges(figure):
            key = line_key(line)
            dx, dy = end[0] - start[0], end[1] - start[1]
            buckets: dict[object, set] = {}
            for node in skeleton.nodes:
                if key not in node.participants:
                    continue
                point = (node.point.x, node.point.y)
                buckets.setdefault(
                    projection(point, dx, dy), set()
                ).add(point)
            for places in buckets.values():
                assert len(places) == 1, (wide, tall, start, end)
            coincident_ties += sum(
                1
                for node in skeleton.nodes
                if key in node.participants
            ) - len(buckets)
    # Совпадения проекций всё-таки есть — и это делает измерение измерением,
    # а не пустой проверкой на пустом множестве.
    assert coincident_ties == 26 * 42


def test_the_defect_is_the_ridge_crossing_that_the_skeleton_never_emits():
    """ПОДТВЕРЖДЕНИЕ настоящей причины: нужной вершины НЕТ среди узлов.

    У креста два гребня: горизонтальный (центральный блок схлопывается по
    высоте) и вертикальный (по ширине). Пересекаются они в центре блока —
    точке `(4 + wide/2, 4 + tall/2)`. Именно там сходятся грани четырёх
    коллинеарных стенок, и именно её среди узлов скелета НЕТ ни на одной из 42
    строк. Добавь её кандидатом к четырём стенкам — и дефект обнуляется ТОЧНО
    на всех 36 строках, где закон имеет силу, а правка ложится ровно на эти
    четыре грани и поровну на каждую.

    Отсюда вывод, меняющий план: дело не в порядке обхода и не в составе
    списка. Сборщику НЕЧЕМ собрать правильную грань — вершины ему не дали.
    Корень в `skeleton.py`, а не в `faces.py`, и `build_faces` тут не чинится.
    """

    zeroed = 0
    for wide, tall, defect in CROSS_GRID_DEFECT:
        figure, skeleton, partition = _cross_partition(wide, tall)
        crossing = (
            SqrtSumV1.rational(Fraction(4) + Fraction(wide, 2)),
            SqrtSumV1.rational(Fraction(4) + Fraction(tall, 2)),
        )
        places = {(node.point.x, node.point.y) for node in skeleton.nodes}
        assert crossing not in places, (wide, tall)

        target = CROSS_WALLS if wide > tall else CROSS_CAPS
        before, before_areas = _reassembled_total(figure, skeleton, {})
        after, after_areas = _reassembled_total(
            figure, skeleton, {index: (crossing,) for index in target}
        )
        assert before == partition.doubled_area
        moved = {
            index
            for index in before_areas
            if not (after_areas[index] - before_areas[index]).is_zero
        }
        if abs(wide - tall) > CROSS_ARM:
            # Снаружи области закона одной вершины мало: остаток остаётся, и
            # он тоже число, а не «почти сошлось».
            residual = after - SqrtSumV1.rational(
                partition.polygon_doubled_area
            )
            assert not residual.is_zero, (wide, tall)
            continue
        assert moved == set(target), (wide, tall)
        gains = {
            (after_areas[index] - before_areas[index]).as_rational()
            for index in target
        }
        assert gains == {Fraction(-defect, 4)}, (wide, tall)
        assert (
            after - SqrtSumV1.rational(partition.polygon_doubled_area)
        ).is_zero, (wide, tall)
        zeroed += 1
    assert zeroed == 36
