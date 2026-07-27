"""Частичный источник: часть границы — НЕПОДВИЖНЫЕ СТЕНЫ, а не рёбра-источники.

Что здесь проверяется и почему именно так.

**Умолчание обязано сохранить поведение.** Полигон без пометок — это прежний
полигон до последнего числа, и проверяется это не словом «по построению», а
сравнением скоростей с `|d|^2` на всём корпусе плюс тем, что `FROZEN_DIGESTS` в
`test_wavefront_motorcycle_graph.py` не сдвинулись ни на одной фигуре.

**Ответ проверяется НЕЗАВИСИМО.** У фигуры с частичным источником верный ответ
ДРУГОЙ, и брать его у очереди значило бы проверять очередь очередью.
`mitered_standard.partial_source_standard` считает его своей формулой — union
митрованных полос по рёбрам-источникам, — целиком в дробях и без единой строки
из `wavefront/`.

**Отрицательный контроль важнее самой проверки.** У фигуры, где источниками
помечены ВСЕ рёбра, расширенный эталон обязан дать ровно прежние числа: `ell`
44 / 63 / 80 / 108 при alpha = 1, 3/2, 2, 3. Это два РАЗНЫХ вычисления
(объединение сжатых вписанных прямоугольников против объединения шаров
Чебышёва), и совпадение их — единственное свидетельство, что расширение не
подменило модель.

**Отказ очереди — тоже ответ, и он тоже сверяется.** Из 38 фигур корпуса
частичного источника на 18 очередь отвечает ИМЕНОВАННЫМ отказом: источник не
заметает фигуру целиком, потому что конец цепочки едет по продолжению прямой
стены внутрь области. Это не дефект и не пропуск: обе объявленные границы
(`WAVEFRONT_LEFT_UNRESOLVED` у скелета, `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON` у
сборщика) срабатывают ГРОМКО, и тихого неверного числа ни на одной из них нет.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.event_time import (
    SupportLineV1,
    ZERO_TIME,
    EventTimeV1,
    sliding_point,
)
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.motorcycle import speed_bound_of
from cftuv_envelope.wavefront.polygon import (
    LoopV1,
    PolygonOutcome,
    PolygonRejected,
    PolygonV1,
    unit_speed_squared,
    with_edge_speeds,
    with_source_spans,
)
from cftuv_envelope.wavefront.skeleton import (
    SkeletonOutcome,
    build_skeleton,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from mitered_standard import (
    StandardOutcome,
    partial_source_standard,
    rectilinear_standard,
)
from wavefront_cases import (
    PARTIAL_SOURCE_ALPHAS,
    axis_rectangle,
    ell,
    named_corpus,
    partial_source_corpus,
)


CORPUS = named_corpus()
CORPUS_IDS = tuple(name for name, _ in CORPUS)
PARTIAL = partial_source_corpus()
PARTIAL_IDS = tuple(name for name, _ in PARTIAL)

#: Исходы, которыми очередь имеет право ответить «фигуру источник не заметает».
#: Перечислены поимённо: «отказала как-нибудь» и «отказала по этой причине» —
#: разные утверждения, и проверять надо второе.
NAMED_PARTIAL_REFUSALS = (
    FaceOutcome.SKELETON_IS_NOT_EXACT,
    FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON,
)

#: Исходы, которыми ЭТАЛОН имеет право сказать «моя модель здесь не описывает
#: фронт». Отказ эталона — такой же законный ответ, как отказ очереди, и
#: перечислен он поимённо ровно по той же причине.
NAMED_STANDARD_REFUSALS = (
    StandardOutcome.SOURCE_SPEEDS_DIFFER_AT_A_REFLEX_VERTEX,
    StandardOutcome.SOURCE_SPEED_IS_NOT_RATIONAL,
)

#: Как корпус частичного источника раскладывается на три названные доли. Числа
#: заморожены не как цель, а как ратчет: их сдвиг означает, что изменилось
#: поведение на частичном источнике, и он обязан быть виден в диффе.
FIGURES_WHERE_THE_QUEUE_MATCHES_THE_STANDARD = 22
FIGURES_WHERE_THE_QUEUE_REFUSES_BY_NAME = 18
FIGURES_WHERE_THE_STANDARD_REFUSES_BY_NAME = 1


# --------------------------------------------------------------------------
# 1. Умолчание сохраняет поведение
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_a_polygon_without_marks_has_every_edge_as_a_unit_speed_source(
    name, polygon
):
    """Умолчание — это `q = |d|^2` у КАЖДОГО ребра, число в число.

    Проверка сравнивает не «источник ли», а само `q` с тем, что кладёт
    `SupportLineV1.through`: совпадение флагов ничего не сказало бы про
    арифметику, а именно она входит во время события.
    """

    for start, end, speed_squared in polygon.edges():
        assert speed_squared == unit_speed_squared(start, end)
        assert speed_squared == SupportLineV1.through(start, end).q
    assert polygon.source_edge_count == polygon.edge_count
    assert polygon.wall_edge_count == 0


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_the_speed_bound_of_a_unit_speed_polygon_is_exactly_one(name, polygon):
    """Обобщённая теорема 2.11 не двигает индекс там, где скорости единичны.

    `speed_bound_of` — единственный множитель, которым обобщение вошло в
    `TraceCandidateIndexV1`. Пока он равен единице, рамка кандидатов та же до
    последней ячейки, и счёт кандидатов не может измениться.
    """

    assert speed_bound_of(polygon) == 1


def test_the_speed_bound_grows_with_the_fastest_edge_and_not_with_walls():
    """Стена границу не поднимает, вес поднимает — и обе проверены числом."""

    rectangle = axis_rectangle(12, 8)
    bottom = rectangle.edges()[0][:2]
    walls = with_source_spans(rectangle, (bottom,))
    assert speed_bound_of(walls) == 1
    fast = with_edge_speeds(
        rectangle, ((bottom[0], bottom[1], 4 * unit_speed_squared(*bottom)),)
    )
    assert speed_bound_of(fast) == 2


# --------------------------------------------------------------------------
# 2. Пометка не теряется молча
# --------------------------------------------------------------------------


def test_reversing_a_loop_carries_the_marks_to_the_same_edges():
    """Разворот петли переставляет скорости вместе с точками.

    Это ровно то место, где пометка терялась бы молча: `PolygonV1.build`
    нормирует ориентацию, у дыры она обратная ВСЕГДА, и без перестановки
    пометка съезжала бы на соседнее ребро у каждой дыры корпуса.
    """

    points = ((0, 0), (4, 0), (4, 3), (0, 3))
    loop = LoopV1(points, (10, 20, 30, 40))
    reversed_loop = loop.oriented(counter_clockwise=not loop.is_counter_clockwise)
    before = {
        frozenset((points[index], points[(index + 1) % 4])): speed
        for index, speed in enumerate(loop.edge_speeds_squared)
    }
    after_points = reversed_loop.points
    after = {
        frozenset((after_points[index], after_points[(index + 1) % 4])): speed
        for index, speed in enumerate(reversed_loop.edge_speeds_squared)
    }
    assert before == after


def test_rotating_a_loop_carries_the_marks_to_the_same_edges():
    """Смена стартовой вершины пометку не двигает — она едет вместе с ребром."""

    points = ((0, 0), (4, 0), (4, 3), (0, 3))
    loop = LoopV1(points, (10, 20, 30, 40))
    for shift in range(4):
        rotated = loop.rotated(shift)
        spun = rotated.points
        assert {
            frozenset((spun[index], spun[(index + 1) % 4])): speed
            for index, speed in enumerate(rotated.edge_speeds_squared)
        } == {
            frozenset((points[index], points[(index + 1) % 4])): speed
            for index, speed in enumerate(loop.edge_speeds_squared)
        }


def test_a_hole_keeps_its_marks_through_the_orientation_normalisation():
    """Дыра — тот случай, где разворот происходит ВСЕГДА, а не иногда."""

    outer = ((0, 0), (20, 0), (20, 20), (0, 20))
    hole_points = ((6, 6), (14, 6), (14, 14), (6, 14))
    polygon = PolygonV1.build(outer, (hole_points,))
    marked = with_source_spans(polygon, (((6, 6), (14, 6)),))
    sources = [
        (start, end)
        for start, end, speed in marked.edges()
        if speed > 0
    ]
    assert len(sources) == 1
    assert frozenset(sources[0]) == frozenset(((6, 6), (14, 6)))


@pytest.mark.parametrize(
    "speeds,outcome",
    (
        ((1, 2, 3), PolygonOutcome.LOOP_SPEED_COUNT_DOES_NOT_MATCH_EDGES),
        ((True, False, True, False), PolygonOutcome.LOOP_SPEED_IS_NOT_INTEGER),
        ((Fraction(1, 2), 1, 1, 1), PolygonOutcome.LOOP_SPEED_IS_NOT_INTEGER),
        ((-1, 1, 1, 1), PolygonOutcome.LOOP_SPEED_IS_NEGATIVE),
    ),
)
def test_a_broken_mark_is_rejected_under_its_own_name(speeds, outcome):
    """Каждый способ испортить пометку назван отдельно, а не свален в один."""

    with pytest.raises(PolygonRejected) as error:
        LoopV1(((0, 0), (4, 0), (4, 3), (0, 3)), speeds)
    assert error.value.outcome is outcome


def test_a_polygon_without_a_single_source_is_rejected_by_name():
    """Все стены — фронту неоткуда пойти, и это вход, а не результат."""

    with pytest.raises(PolygonRejected) as error:
        PolygonV1(LoopV1(((0, 0), (4, 0), (4, 3), (0, 3)), (0, 0, 0, 0)))
    assert error.value.outcome is PolygonOutcome.POLYGON_HAS_NO_SOURCE_EDGE


def test_marking_an_edge_that_does_not_exist_is_refused_and_not_ignored():
    """Пометка, не нашедшая ребра, — потеря. Она названа, а не пропущена."""

    rectangle = axis_rectangle(12, 8)
    with pytest.raises(PolygonRejected) as error:
        with_source_spans(rectangle, (((0, 0), (12, 0)), ((3, 3), (5, 5))))
    assert error.value.outcome is PolygonOutcome.SOURCE_SPAN_IS_NOT_AN_EDGE


# --------------------------------------------------------------------------
# 3. Единичная скорость снята там, где была зашита
# --------------------------------------------------------------------------


@pytest.mark.parametrize("speed_squared", (0, 1, 25, 100, 400))
def test_the_sliding_point_solves_its_own_system_at_any_speed(speed_squared):
    """Определитель системы скольжения — `a^2 + b^2`, и проверяется он ОТВЕТОМ.

    Обе строки системы подставляются обратно: точка обязана лежать на движущейся
    прямой и иметь ровно ту проекцию вдоль неё, которую ей задали. Пока
    определитель брался из `q`, равенство держалось только при `q = a^2 + b^2`;
    при `q = 0` это было делением на ноль, при взвешенном ребре — неверным
    масштабом, и ни то ни другое не ловилось, потому что таких `q` не бывало.
    """

    line = SupportLineV1.with_speed((0, 0), (5, 0), speed_squared)
    along = SqrtSumV1.rational(7)
    time = EventTimeV1.normalized(3, SqrtSumV1.rational(1))
    point = sliding_point(line, along, time)

    on_the_line = (
        point.x.scaled(line.a)
        + point.y.scaled(line.b)
        - SqrtSumV1.rational(line.c)
        - SqrtSumV1.radical(time.dividend, line.q) / time.divisor
    )
    projection = point.x.scaled(line.b) - point.y.scaled(line.a) - along
    assert on_the_line.is_zero
    assert projection.is_zero


def test_a_wall_line_does_not_move_at_all_and_says_so():
    """`q = 0` — та же прямая при любом `t`. Это утверждение, а не запись."""

    wall = SupportLineV1.with_speed((0, 0), (5, 0), 0)
    assert wall.is_stationary
    assert not SupportLineV1.through((0, 0), (5, 0)).is_stationary
    here = sliding_point(wall, SqrtSumV1.rational(7), ZERO_TIME)
    later = sliding_point(
        wall, SqrtSumV1.rational(7), EventTimeV1.normalized(9, SqrtSumV1.rational(1))
    )
    assert (here.x - later.x).is_zero
    assert (here.y - later.y).is_zero


# --------------------------------------------------------------------------
# 4. Ответ на частичном источнике, проверенный НЕЗАВИСИМО
# --------------------------------------------------------------------------


@pytest.mark.parametrize("alpha", (1, 2, 4, 8, 9))
def test_a_rectangle_with_one_source_edge_is_checked_by_hand(alpha):
    """Самый простой частичный источник: полоса `12 x alpha`, и это видно глазом.

    Фигура нужна не для полноты. Её ответ — `12 * min(alpha, 8)` — не зависит ни
    от эталона, ни от очереди, поэтому она проверяет ОБОИХ сразу.
    """

    rectangle = axis_rectangle(12, 8)
    figure = with_source_spans(rectangle, (((0, 0), (12, 0)),))
    partition = build_faces(figure, build_skeleton(figure))
    assert partition.outcome is FaceOutcome.EXACT
    covered = coverage_at(partition, Fraction(alpha))
    assert covered.outcome is CoverageOutcome.EXACT
    assert covered.doubled_area.as_rational() == 2 * 12 * min(alpha, 8)


@pytest.mark.parametrize("alpha", (1, 2, 4, 8))
def test_a_weighted_source_edge_moves_at_its_own_speed(alpha):
    """Взвешенное ребро: `q = 4|d|^2` — скорость 2, покрытие `12 * min(2a, 8)`.

    Оно здесь потому, что стена и вес — ОДНО обобщение. Проверять только нулевую
    его сторону значило бы принять частный случай за общий ровно так же, как
    прежняя единичная скорость.
    """

    rectangle = axis_rectangle(12, 8)
    bottom = rectangle.edges()[0][:2]
    figure = with_edge_speeds(
        rectangle, ((bottom[0], bottom[1], 4 * unit_speed_squared(*bottom)),)
    )
    partition = build_faces(figure, build_skeleton(figure))
    assert partition.outcome is FaceOutcome.EXACT
    covered = coverage_at(partition, Fraction(alpha))
    assert covered.doubled_area.as_rational() == 2 * 12 * min(2 * alpha, 8)


@pytest.mark.parametrize("name,polygon", PARTIAL, ids=PARTIAL_IDS)
def test_the_queue_either_matches_the_standard_or_refuses_by_name(
    name, polygon
):
    """Главная проверка среза: тихого неверного числа нет ни на одной фигуре.

    Исходов ровно три, и все три названы: очередь отказывает своей объявленной
    границей; эталон отказывает своей (модель не применима); либо оба отвечают,
    и тогда числа обязаны совпасть ТОЧНО при каждой alpha. Четвёртого — числа,
    которое никем не проверено, — быть не должно.
    """

    partition = build_faces(polygon, build_skeleton(polygon))
    if partition.outcome is not FaceOutcome.EXACT:
        assert partition.outcome in NAMED_PARTIAL_REFUSALS, partition.detail
        return
    assert len(partition.faces) == polygon.source_edge_count
    for alpha in PARTIAL_SOURCE_ALPHAS:
        covered = coverage_at(partition, alpha)
        standard = partial_source_standard(polygon, alpha)
        assert covered.outcome is CoverageOutcome.EXACT
        if standard.outcome is not StandardOutcome.EXACT:
            assert standard.outcome in NAMED_STANDARD_REFUSALS, standard.detail
            continue
        assert covered.doubled_area.as_rational() == 2 * standard.covered, (
            name,
            alpha,
        )


def test_the_partial_source_corpus_splits_into_three_named_parts():
    """Сколько фигур сверено, сколько отвергла очередь, сколько эталон.

    Ратчет: сдвиг любого из трёх чисел означает смену поведения на частичном
    источнике. Он законен, но обязан быть виден в диффе, а не растворяться в
    «тесты зелёные». Сумма трёх долей обязана дать корпус целиком — иначе
    какая-то фигура не проверяется вовсе, и этого не было бы видно.
    """

    matched = refused_by_queue = refused_by_standard = 0
    for _, polygon in PARTIAL:
        partition = build_faces(polygon, build_skeleton(polygon))
        if partition.outcome is not FaceOutcome.EXACT:
            assert partition.outcome in NAMED_PARTIAL_REFUSALS
            refused_by_queue += 1
            continue
        standard = partial_source_standard(polygon, Fraction(1))
        if standard.outcome is StandardOutcome.EXACT:
            matched += 1
        else:
            assert standard.outcome in NAMED_STANDARD_REFUSALS
            refused_by_standard += 1
    assert matched == FIGURES_WHERE_THE_QUEUE_MATCHES_THE_STANDARD
    assert refused_by_queue == FIGURES_WHERE_THE_QUEUE_REFUSES_BY_NAME
    assert refused_by_standard == FIGURES_WHERE_THE_STANDARD_REFUSES_BY_NAME
    assert (
        matched + refused_by_queue + refused_by_standard == len(PARTIAL) == 41
    )


@pytest.mark.parametrize(
    "span,skeleton_outcome,face_outcome",
    (
        # Источник — верхнее ребро правого рукава, стена при вогнутой вершине —
        # вертикальное ребро левого. Фронт едет вниз, его левый конец ведёт
        # ПРЯМАЯ стены, а ниже вогнутой вершины эта прямая уходит внутрь
        # фигуры. Левый рукав не заметается, и вершины фронта остаются живы:
        # отказывает сам скелет.
        (
            ((12, 6), (6, 6)),
            SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED,
            FaceOutcome.SKELETON_IS_NOT_EXACT,
        ),
        # Источник — верхнее ребро левого рукава. Здесь фронт доходит до конца
        # и скелет ЗАКРЫВАЕТСЯ, а незаметённой остаётся часть правого рукава,
        # поэтому громко говорит уже сборщик граней. Два разных отказа на одной
        # фигуре при разных источниках — и оба названы.
        (
            ((6, 12), (0, 12)),
            SkeletonOutcome.EXACT,
            FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON,
        ),
    ),
)
def test_the_reflex_vertex_with_the_source_on_one_side_is_refused_loudly(
    span, skeleton_outcome, face_outcome
):
    """Вогнутая вершина, у которой источник по одну сторону, — названный отказ.

    Разбор, ради которого фигуры заведены: незаметённая часть фигуры обязана
    называться, а не превращаться в тихо заниженное покрытие. Обе объявленные
    границы ловят её, и какая именно — зависит от того, успевает ли фронт
    умереть; поэтому в проверке стоят обе, поимённо.
    """

    figure = with_source_spans(ell(12), (span,))
    skeleton = build_skeleton(figure)
    partition = build_faces(figure, skeleton)
    assert skeleton.outcome is skeleton_outcome
    assert partition.outcome is face_outcome


def test_the_reflex_vertex_with_the_source_on_both_sides_is_answered(
):
    """Та же вогнутая вершина, но источник по ОБЕ стороны — ответ есть.

    Пара к предыдущему тесту: разницу делает не вогнутость, а то, ведёт ли
    прямая стены фронт внутрь фигуры. Числа сверены с независимым эталоном.
    """

    figure = with_source_spans(
        ell(12), (((12, 6), (6, 6)), ((6, 6), (6, 12)))
    )
    partition = build_faces(figure, build_skeleton(figure))
    assert partition.outcome is FaceOutcome.EXACT
    assert [
        coverage_at(partition, Fraction(alpha)).doubled_area.as_rational()
        for alpha in (1, 2, 3, 6)
    ] == [26, 56, 90, 216]


def test_a_weighted_edge_next_to_a_reflex_vertex_uses_the_generalised_bound():
    """Взвешенное ребро при ВОГНУТОЙ вершине — там, где индекс кандидатов живой.

    Без этой фигуры обобщение теоремы 2.11 осталось бы непроверенным: у
    выпуклого прямоугольника reflex-вершин нет вовсе, трасс нет, и множитель
    `speed_bound` ни разу не участвует. Здесь он равен двум, трасса есть, и
    кандидаты по ней действительно перебираются.
    """

    shape = ell(12)
    figure = with_edge_speeds(
        shape,
        tuple(
            (start, end, (4 if index == 0 else 1) * unit_speed_squared(start, end))
            for index, (start, end, _) in enumerate(shape.edges())
        ),
    )
    assert speed_bound_of(figure) == 2
    assert figure.reflex_count == 1
    skeleton = build_skeleton(figure)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert skeleton.counter("split_candidates_examined") > 0
    partition = build_faces(figure, skeleton)
    assert partition.outcome is FaceOutcome.EXACT
    for alpha in (Fraction(1), Fraction(3, 2), Fraction(2), Fraction(3)):
        standard = partial_source_standard(figure, alpha)
        assert standard.outcome is StandardOutcome.EXACT
        assert (
            coverage_at(partition, alpha).doubled_area.as_rational()
            == 2 * standard.covered
        )


def test_the_standard_refuses_mixed_speeds_at_a_reflex_vertex_by_name():
    """Граница МОДЕЛИ эталона, а не очереди, и она названа своим исходом.

    Митрованная полоса заворачивает вогнутый угол КВАДРАТОМ, и это верно ровно
    при равных скоростях сходящихся там источников. У разных фронт заворачивает
    по взвешенной биссектрисе, угол выходит несимметричным, и эталон обязан
    сказать «не считаю», а не посчитать. Очередь при этом отвечает — отказ здесь
    именно эталона, и различать это важно: иначе граница модели выглядела бы
    дефектом очереди.
    """

    shape = ell(12)
    figure = with_edge_speeds(
        shape,
        tuple(
            (start, end, (4 if index in (0, 3) else 1) * unit_speed_squared(start, end))
            for index, (start, end, _) in enumerate(shape.edges())
        ),
    )
    partition = build_faces(figure, build_skeleton(figure))
    assert partition.outcome is FaceOutcome.EXACT
    standard = partial_source_standard(figure, Fraction(1))
    assert (
        standard.outcome
        is StandardOutcome.SOURCE_SPEEDS_DIFFER_AT_A_REFLEX_VERTEX
    )
    assert "(6, 6)" in standard.detail


# --------------------------------------------------------------------------
# 5. Отрицательный контроль расширенного эталона
# --------------------------------------------------------------------------


def test_the_extended_standard_reproduces_the_frozen_ell_numbers():
    """`ell` со ВСЕМИ рёбрами-источниками: 44 / 63 / 80 / 108, как и было.

    Величина из замкнутой формулы, от кода не зависящая. Если расширение
    эталона сдвинуло бы её, оно подменило бы модель, а не обобщило её.
    """

    figure = ell(12)
    assert [
        partial_source_standard(figure, alpha).covered
        for alpha in (Fraction(1), Fraction(3, 2), Fraction(2), Fraction(3))
    ] == [44, 63, 80, 108]


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_the_extended_standard_agrees_with_the_old_one_where_it_applies(
    name, polygon
):
    """Два РАЗНЫХ вычисления одного множества обязаны дать одно число.

    Прежний эталон строит отступ объединением сжатых вписанных прямоугольников,
    расширенный — объединением шаров Чебышёва вокруг рёбер-источников. Общего
    кода у них нет, кроме сетки; совпадение на всём корпусе и есть проверка
    того, что расширение — обобщение, а не вторая модель.
    """

    for alpha in (Fraction(1), Fraction(3, 2), Fraction(2), Fraction(3)):
        old = rectilinear_standard(polygon, alpha)
        new = partial_source_standard(polygon, alpha)
        assert new.outcome is old.outcome
        if old.outcome is not StandardOutcome.EXACT:
            continue
        assert new.covered == old.mitered_covered
        assert new.source_edge_count == polygon.edge_count
        assert new.wall_edge_count == 0
