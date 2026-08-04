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

**Отказ очереди — тоже ответ, и он тоже сверяется.** Из 41 фигуры корпуса
частичного источника на 17 очередь отвечает ИМЕНОВАННЫМ отказом: источник не
заметает фигуру целиком, потому что конец цепочки едет по продолжению прямой
стены внутрь области. Это не дефект и не пропуск: обе объявленные границы
(`WAVEFRONT_LEFT_UNRESOLVED` у скелета, `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON` у
сборщика) срабатывают ГРОМКО, и тихого неверного числа ни на одной из них нет.
Восемнадцатой фигурой был `staircase_source_edges_7_0`, и её отказ оказался не
свойством входа, а дефектом ПОРЯДКА СБОРКИ: с правилом смежности она отвечает
`EXACT` и совпадает с эталоном на всех восьми alpha.

**КОЛЛИНЕАРНАЯ НЕПОДВИЖНАЯ СТЕНА.** Отдельный раздел ниже держит разбор одной
конкретной болезни и её числа. Движущаяся прямая источника наезжает на прямую
коллинеарной стены; тройка прямых сходится честно, но ОТРЕЗКИ перекрываются в
единственной точке, и стена не сжимается ни на шаг. Скелет тем не менее гасил её
целиком, фронт скачком расширялся вдвое и заметал то, чего достичь не мог. Три
фигуры корпуса, одна болезнь, и её числа заморожены ниже поточечно.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.event_time import (
    SupportLineV1,
    ZERO_TIME,
    EventTimeOutcome,
    EventTimeV1,
    compare_times,
    sliding_point,
)
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.motorcycle import speed_bound_of
from cftuv_envelope.wavefront.polygon import (
    LoopV1,
    PolygonOutcome,
    PolygonRejected,
    PolygonV1,
    signed_double_area,
    unit_speed_squared,
    with_edge_speeds,
    with_source_spans,
)
from cftuv_envelope.wavefront.skeleton import (
    CandidateRefusal,
    SkeletonOutcome,
    build_skeleton,
    refusal_counter,
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
#:
#: Расклад 22 / 18 / 1 держался до постановки правила смежности. С ней
#: `staircase_source_edges_7_0` переехал из «очередь отказала» в «совпал с
#: эталоном»: его отказ был чистым дефектом порядка сборки, а не свойством
#: входа. Остальные 40 строк не двинулись ни одной.
FIGURES_WHERE_THE_QUEUE_MATCHES_THE_STANDARD = 23
FIGURES_WHERE_THE_QUEUE_REFUSES_BY_NAME = 17
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
        ((True, False, True, False), PolygonOutcome.LOOP_SPEED_IS_NOT_RATIONAL),
        ((0.5, 1, 1, 1), PolygonOutcome.LOOP_SPEED_IS_NOT_RATIONAL),
        (("1/2", 1, 1, 1), PolygonOutcome.LOOP_SPEED_IS_NOT_RATIONAL),
        ((-1, 1, 1, 1), PolygonOutcome.LOOP_SPEED_IS_NEGATIVE),
        ((Fraction(-1, 2), 1, 1, 1), PolygonOutcome.LOOP_SPEED_IS_NEGATIVE),
    ),
)
def test_a_broken_mark_is_rejected_under_its_own_name(speeds, outcome):
    """Каждый способ испортить пометку назван отдельно, а не свален в один.

    `Fraction(1, 2)` из этого списка УБРАН и переехал в тест ниже: дробное `q`
    перестало быть поломкой и стало входом. Отвергается по-прежнему всё, что
    не рационально ТОЧНО, — `bool`, `float`, строка, — и это не педантизм:
    `float` прошёл бы каждое сравнение ниже и отравил бы каноническую форму,
    единственность которой держится на точных дробях.
    """

    with pytest.raises(PolygonRejected) as error:
        LoopV1(((0, 0), (4, 0), (4, 3), (0, 3)), speeds)
    assert error.value.outcome is outcome


def test_a_rational_speed_is_accepted_and_an_integer_valued_fraction_is_int():
    """Дробное `q` — вход, а не поломка, и оно НОРМИРУЕТСЯ при знаменателе 1.

    Обе половины важны, и вторая важнее. Приём дроби — это то, ради чего
    снималось требование целого. Нормировка — это то, чем держится дайджест:
    `Fraction(16)` и `16` равны по значению, но как ключи прямой различались бы,
    и один и тот же четырёхугольник давал бы два разных дайджеста в зависимости
    от того, каким типом записана его скорость.
    """

    loop = LoopV1(((0, 0), (4, 0), (4, 3), (0, 3)), (Fraction(1, 2), 16, 0, 9))
    assert loop.speeds_squared == (Fraction(1, 2), 16, 0, 9)
    # Нормировка проверяется ТИПОМ, а не равенством: `Fraction(16) == 16`.
    assert [type(value) for value in loop.speeds_squared] == [
        Fraction,
        int,
        int,
        int,
    ]
    integral = LoopV1(
        ((0, 0), (4, 0), (4, 3), (0, 3)),
        (Fraction(16), Fraction(16), Fraction(0), Fraction(9)),
    )
    assert all(type(value) is int for value in integral.speeds_squared)


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
    # Совпадение суммы не доказывает совпадение множества (`DECISIONS.md`,
    # 2026-07-27), поэтому сверяется не только площадь: владельцы граней обязаны
    # быть РОВНО рёбрами-источниками, поимённо. Верная сумма при владельце-стене
    # означала бы, что грань собрана не из того ребра.
    assert {face.owner for face in partition.faces} == {
        (start[0], start[1], end[0], end[1])
        for start, end, speed in polygon.edges()
        if speed > 0
    }
    assert len(partition.faces) == polygon.source_edge_count
    assert partition.every_face_is_positive
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
        # Источник — верхнее ребро ЛЕВОГО рукава. Раньше эта строка стояла с
        # `EXACT` у скелета и отказом у сборщика, и обе половины были неверны по
        # одной причине: фронт гасил коллинеарную стену `(12,6) -> (6,6)`, с
        # которой его отрезок не перекрывался, скачком расширялся вдвое и
        # доезжал до конца через правый рукав, которого достичь не мог. Теперь
        # фиктивное схлопывание отвергнуто по имени, фронт остаётся шириной 6, и
        # правый рукав честно недостижим — вершины фронта остаются живы.
        (
            ((6, 12), (0, 12)),
            SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED,
            FaceOutcome.SKELETON_IS_NOT_EXACT,
        ),
    ),
)
def test_the_reflex_vertex_with_the_source_on_one_side_is_refused_loudly(
    span, skeleton_outcome, face_outcome
):
    """Вогнутая вершина, у которой источник по одну сторону, — названный отказ.

    Разбор, ради которого фигуры заведены: незаметённая часть фигуры обязана
    называться, а не превращаться в тихо заниженное покрытие.

    Обе строки теперь отказывают на одной ступени, и это не потеря
    различительной силы, а исправленный диагноз: вторая строка отказывала
    ПОЗЖЕ только потому, что скелет её досчитывал по неверному фронту.

    Строки, где отказ приходит от СБОРЩИКА, в корпусе не осталось ни одной.
    Последней была `staircase_source_edges_7_0`, у которой эталон при любой
    alpha заметает фигуру ЦЕЛИКОМ (192 из 192 удвоенных): незаметённой части
    там нет вовсе, и терять площадь было нечему, кроме порядка сборки. Правило
    смежности её закрыло — числа в
    `test_the_last_pure_order_defect_of_the_corpus_is_closed_and_agrees_with_the_standard`.
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


# --------------------------------------------------------------------------
# 6. Коллинеарная неподвижная стена: диагноз числами и его починка
# --------------------------------------------------------------------------

#: Три фигуры корпуса, где коллинеарная стена гасилась фронтом, которого не
#: касалась, и числа их конфигурации. Список фиксирован здесь, а не собирается
#: перебором: перечень входов берётся из корпуса и по ходу не дополняется.
#:
#: | поле | что это |
#: |---|---|
#: | `wall` | вхождение ребра-СТЕНЫ, которое гасилось |
#: | `source` | вхождение ребра-ИСТОЧНИКА, чей фронт на неё наезжал |
#: | `moment` | момент наезда, целое (у всех трёх время рационально) |
#: | `meeting` | ЕДИНСТВЕННАЯ общая точка двух отрезков |
#: | `node` | точка, в которой рождался фиктивный узел — дальний конец стены |
#: | `wall_span` | отрезок фронта СТЕНЫ в проекции на общее направление |
#: | `source_span` | отрезок фронта ИСТОЧНИКА в той же проекции |
COLLINEAR_WALL_CASES = (
    (
        "ell_12_source_edge_4",
        (12, 6, 6, 6),
        (6, 12, 0, 12),
        6,
        (6, 6),
        (12, 6),
        (-72, -36),
        (-36, 0),
    ),
    (
        "staircase_source_edge_6",
        (8, 8, 4, 8),
        (4, 12, 0, 12),
        4,
        (4, 8),
        (8, 8),
        (-32, -16),
        (-16, 0),
    ),
    (
        "staircase_source_edges_4_5",
        (12, 4, 8, 4),
        (8, 8, 4, 8),
        4,
        (8, 4),
        (12, 4),
        (-48, -32),
        (-32, 0),
    ),
)

COLLINEAR_WALL_IDS = tuple(row[0] for row in COLLINEAR_WALL_CASES)


def _codirectional_wall_joint(builder):
    """Живая вершина, у которой соседи КОЛЛИНЕАРНЫ и разной скорости.

    Именно она и есть разрыв фронта: у стены скорость ноль, у источника нет, их
    прямые совпадают на одно мгновение и тут же расходятся, и позиции у такой
    вершины не существует — не «её трудно посчитать», а её нет.
    """

    for vertex in builder.vertices:
        if not vertex.alive:
            continue
        prev_line = builder.edges[vertex.prev_edge].line
        next_line = builder.edges[vertex.next_edge].line
        if prev_line.a * next_line.b - next_line.a * prev_line.b:
            continue
        if skeleton_module._joint_kind(prev_line, next_line) is (
            CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS
        ):
            return vertex
    return None


def _run_until_the_wall_is_reached(name):
    """Цикл событий, доведённый до уровня, где фронт встал на прямую стены.

    Строитель гоняется по тем же правилам, что и `run()`, а не по своим:
    измерять надо то, что делает очередь, а не то, что похоже на неё.
    """

    builder = skeleton_module._Builder(dict(PARTIAL)[name])
    while len(builder.queue):
        level = builder.queue.pop_level()
        builder.now = level[0].time
        builder._apply_level(level)
        builder._close_short_lavs()
        joint = _codirectional_wall_joint(builder)
        if joint is not None:
            return builder, joint
    raise AssertionError(f"{name}: коллинеарный стык не встретился")


@pytest.mark.parametrize(
    "name,wall,source,moment,meeting,node,wall_span,source_span",
    COLLINEAR_WALL_CASES,
    ids=COLLINEAR_WALL_IDS,
)
def test_the_front_and_the_collinear_wall_touch_in_exactly_one_point(
    name, wall, source, moment, meeting, node, wall_span, source_span
):
    """ДИАГНОЗ ЧИСЛАМИ: отрезки касаются в одной точке, а не перекрываются.

    Что измеряется, по шагам и на фигуре ИЗ КОРПУСА:

    1. фронт источника встаёт на прямую стены в момент `moment`, и тройка
       прямых сходится ТОЧНО — `EventTimeOutcome.EXACT`, никакого «почти»;
    2. фиктивный узел рождался бы в точке `node` — это место ПРЕДШЕСТВЕННИКА,
       то есть дальний конец стены, а вовсе не место встречи;
    3. отрезок фронта стены есть `wall_span`, отрезок фронта источника —
       `source_span`, и оба меряются проекцией на ОБЩЕЕ направление, поэтому
       сравнимы напрямую;
    4. пересечение этих отрезков — единственная точка: нижняя граница
       пересечения равна верхней, и это проверяется точным предикатом
       `SqrtSumV1.is_zero`, а не разностью чисел с плавающей точкой.

    Начало отрезка источника берётся из ТОЧКИ РОЖДЕНИЯ стыка, и это законно
    ровно потому, что стык рождён в этот же самый момент: `birth == moment`
    проверяется здесь же. Спрашивать у стыка позицию бессмысленно — её нет.
    """

    builder, joint = _run_until_the_wall_is_reached(name)
    wall_edge = builder.edges[joint.prev_edge]
    source_edge = builder.edges[joint.next_edge]
    assert wall_edge.span == wall
    assert wall_edge.line.is_stationary
    assert source_edge.span == source
    assert not source_edge.line.is_stationary

    before = builder.vertices[joint.prev]
    time, outcome = builder._edge_event_time(before, joint)
    assert outcome is EventTimeOutcome.EXACT
    assert time is not None
    # Момент рационален у всех трёх: сравнение с целым идёт через разность
    # времён, а не через вычисление значения.
    assert compare_times(
        time, EventTimeV1.normalized(moment, SqrtSumV1.rational(1))
    ) == 0
    assert compare_times(joint.birth, time) == 0
    assert (joint.point.x - SqrtSumV1.rational(meeting[0])).is_zero
    assert (joint.point.y - SqrtSumV1.rational(meeting[1])).is_zero
    # Место стыка не существует: его соседи параллельны и разной скорости.
    assert builder._position(joint, time) is None

    place = builder._position(before, time)
    assert (place.x - SqrtSumV1.rational(node[0])).is_zero
    assert (place.y - SqrtSumV1.rational(node[1])).is_zero

    low = builder._span_end(before, wall_edge, time, at_start=True)
    high = builder._span_end(joint, wall_edge, time, at_start=False)
    assert low.as_rational() == wall_span[0]
    assert high.as_rational() == wall_span[1]

    tail = builder._edge_end_vertex(source_edge.ident)
    source_low = skeleton_module._project(source_edge.line, joint.point)
    source_high = builder._span_end(tail, source_edge, time, at_start=False)
    assert source_low.as_rational() == source_span[0]
    assert source_high.as_rational() == source_span[1]

    # ПЕРЕКРЫТИЕ ПО МЕРЕ: нижняя граница пересечения равна верхней, значит
    # длина пересечения ровно ноль. Отрезок стены при этом не пуст.
    assert (source_low - high).is_zero
    assert not (high - low).is_zero


@pytest.mark.parametrize(
    "name,wall,source,moment,meeting,node,wall_span,source_span",
    COLLINEAR_WALL_CASES,
    ids=COLLINEAR_WALL_IDS,
)
def test_the_fictitious_collapse_is_refused_under_its_own_name(
    name, wall, source, moment, meeting, node, wall_span, source_span
):
    """ПОЧИНКА: отрезок положительной длины не схлопывается, и это названо.

    Правило спрашивается ровно там, где рождается кандидат, и отвечает точным
    равенством: длина схлопываемого отрезка в проекции равна `wall_span[1] -
    wall_span[0]`, то есть не ноль. Событие в очередь не попадает, а счётчик
    несёт его имя — тихо не исчезает ничего.
    """

    builder, joint = _run_until_the_wall_is_reached(name)
    before = builder.vertices[joint.prev]
    time, _ = builder._edge_event_time(before, joint)
    span = builder._collapsing_span(before, joint, time)
    assert span is not None
    assert span.as_rational() == wall_span[1] - wall_span[0]
    assert not span.is_zero

    counter = refusal_counter(CandidateRefusal.FILTER_SPAN_DOES_NOT_COLLAPSE)
    was = builder.counters[counter]
    builder._enqueue_edge_event(before)
    assert builder.counters[counter] == was + 1


@pytest.mark.parametrize("name,polygon", PARTIAL, ids=PARTIAL_IDS)
def test_a_stationary_edge_never_reaches_beyond_its_own_span(name, polygon):
    """Отрезок фронта НЕПОДВИЖНОГО ребра — подотрезок самого ребра, всегда.

    Это и есть тот второй ответ `_span_end`, без которого стена с вырожденным
    концом выглядела бы отрезком без границы. Проверяется он не на одной
    фигуре, а на всём корпусе и в момент рождения: концы отрезка обязаны лежать
    между концами ребра по проекции. У ДВИЖУЩЕГОСЯ ребра такого утверждения
    нет и быть не может — его отрезок растёт за свои концы всякий раз, когда
    рядом стоит вогнутая вершина.
    """

    builder = skeleton_module._Builder(polygon)
    for edge in builder.edges:
        if not edge.line.is_stationary:
            continue
        x0, y0, x1, y1 = edge.span
        start = SqrtSumV1.rational(x0 * edge.line.b - y0 * edge.line.a)
        end = SqrtSumV1.rational(x1 * edge.line.b - y1 * edge.line.a)
        assert (end - start).sign() > 0
        for vertex, at_start in (
            (builder._edge_start_vertex(edge.ident), True),
            (builder._edge_end_vertex(edge.ident), False),
        ):
            assert vertex is not None
            here = builder._span_end(vertex, edge, ZERO_TIME, at_start=at_start)
            assert here is not None
            assert (here - start).sign() >= 0
            assert (end - here).sign() >= 0


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_the_new_rule_never_fires_where_every_edge_is_a_source(name, polygon):
    """ОТРИЦАТЕЛЬНЫЙ КОНТРОЛЬ, и он важнее самой починки.

    На корпусе, где источником является ВСЯ граница, нового отказа нет ни
    одного, и второй ответ `_span_end` не спрашивается ни разу: неподвижных
    рёбер там нет вовсе. Отсюда следует, что замороженные ответы этого корпуса
    сдвинуться не могли — не «мы проверили, что они те же», а «их нечем было
    сдвинуть». `FROZEN_DIGESTS` это подтверждает независимо.
    """

    skeleton = build_skeleton(polygon)
    assert skeleton.counter(
        refusal_counter(CandidateRefusal.FILTER_SPAN_DOES_NOT_COLLAPSE)
    ) == 0
    assert all(not line.is_stationary for line in _support_lines(polygon))


def _support_lines(polygon):
    return tuple(
        SupportLineV1.with_speed(start, end, speed)
        for start, end, speed in polygon.edges()
    )


#: Касание в одной точке у ДВИЖУЩИХСЯ рёбер — законное событие, и вот его
#: счётчики на обоих корпусах. Числа заморожены затем, что починка стены
#: обязана была их не тронуть: правило различает неподвижное и движущееся, а не
#: «касание» и «перекрытие».
TOUCHING_STAYS_LEGITIMATE = {
    "ell": {"vertex_meeting_events": 1},
    "hole_1": {"vertex_meeting_events": 4},
    "holes_2": {"vertex_meeting_events": 4},
    "cross": {"vertex_meeting_events": 3},
    "u_shape": {"vertex_meeting_events": 2},
    # Свидетель у `double_notch` СМЕНИЛСЯ, а явление осталось. Прежним был
    # `coincident_split_targets` — счётчик СНЯТЫХ дублей: касание опознавалось
    # по тому, что два split-кандидата одной вершины целились в совпавшие цели
    # и один отбрасывался. После quotient Q-10-ADD дубль поглощается локусом
    # раньше, снимать нечего, и счётчик обнулился — не потому, что касания нет,
    # а потому, что он мерил бухгалтерию снятия. Само касание проверяется ниже
    # ПО КНИГЕ СОБЫТИЙ: germ-леджер обязан материализовать зародыши в точке
    # касания, а скелет — нести там SPLIT-узел.
    "double_notch": {"split_events": 4},
}

#: Локусы касания у `double_notch`: у движущегося ребра отрезок фронта в этот
#: момент сжимается в точку, и локус предъявляется леджеру ДВАЖДЫ — по
#: зародышу на каждую сторону разреза.
TOUCHING_GERM_LOCI = {"double_notch": 4}


@pytest.mark.parametrize("name", sorted(TOUCHING_GERM_LOCI))
def test_touching_materialises_germs_in_the_book_of_events(name, monkeypatch):
    """Касание опознаётся по КНИГЕ СОБЫТИЙ, а не по счётчику снятых дублей.

    Germ-леджер и есть каноническая книга: зародыш — локус контакта, и касание
    обязано его породить. Проверка сильнее прежнего счётчика сразу двумя
    сторонами: локус назван ТОЧКОЙ и ВРЕМЕНЕМ, а не суммой, и рядом стоит узел
    скелета в той же точке — то есть событие не только предъявлено, но и
    материализовано.
    """

    from cftuv_envelope.wavefront import superlevel_germ as germ_module
    from cftuv_envelope.wavefront.events import EventKind

    presented = []
    original = germ_module.SuperlevelGermLedgerV1.materialize

    def observed(self, key, germ, *, fold):
        presented.append((key.time_key[0], key.point_key))
        return original(self, key, germ, fold=fold)

    monkeypatch.setattr(
        germ_module.SuperlevelGermLedgerV1, "materialize", observed
    )
    skeleton = build_skeleton(dict(CORPUS)[name])
    assert skeleton.outcome is SkeletonOutcome.EXACT
    # Свидетель непустой: леджер, который никого не поймал, ничего не проверяет.
    assert presented

    twice = {
        locus for locus in presented if presented.count(locus) > 1
    }
    assert len(twice) == TOUCHING_GERM_LOCI[name]
    split_loci = {
        (node.time.canonical().dividend, (node.point.x.terms, node.point.y.terms))
        for node in skeleton.nodes
        if node.kind is EventKind.SPLIT
    }
    # Каждый локус касания несёт узел скелета в ТОЙ ЖЕ точке и в ТО ЖЕ время.
    assert twice == split_loci


@pytest.mark.parametrize("name", sorted(TOUCHING_STAYS_LEGITIMATE))
def test_touching_in_one_point_is_still_an_event_for_moving_edges(name):
    """Обратная сторона правила: у движущихся рёбер касание событием ОСТАЛОСЬ.

    Вершина, пришедшая в угол фронта, — это `coincident_split_targets`, а
    встреча вершин — `vertex_meeting_events`; обе конфигурации суть касание в
    одной точке, и обе законны. Различие проходит по «неподвижное против
    движущегося», и оно не подгонка: у движущегося ребра отрезок фронта в этот
    момент действительно сжимается в точку, у неподвижной стены — нет, она
    сохраняет всю свою длину. Первое есть событие, второе — его отсутствие.
    """

    polygon = dict(CORPUS)[name]
    skeleton = build_skeleton(polygon)
    for counter, value in TOUCHING_STAYS_LEGITIMATE[name].items():
        assert skeleton.counter(counter) == value


# --------------------------------------------------------------------------
# 5. Дефект порядка сборки, оставшийся на корпусе, и его закрытие смежностью
# --------------------------------------------------------------------------


def test_the_last_pure_order_defect_of_the_corpus_is_closed_and_agrees_with_the_standard():
    """Сборщик на корпусе больше не теряет площадь НИ НА ОДНОЙ фигуре.

    ИСТОРИЯ, и без неё число ничего не значит. Прежнее правило порядка
    (монотонность) отказывало `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON` на ЧЕТЫРЁХ
    фигурах корпуса. Три из них лечились не здесь: под их отказом лежал дефект
    СКЕЛЕТА — фронт гасил коллинеарную неподвижную стену, с которой его отрезок
    не перекрывался, и скачком расширялся вдвое. Стена разобрана, и все три
    теперь отказывают `WAVEFRONT_LEFT_UNRESOLVED` до всякого числа
    (`test_the_three_contradicting_figures_are_now_refused_before_any_number`).

    Четвёртая, `staircase_source_edges_7_0`, была чистым дефектом ПОРЯДКА:
    эталон заметает фигуру целиком (192 из 192 удвоенных), незаметённой части
    нет вовсе, а монотонность всё равно теряла площадь. Правило смежности
    закрывает её, и это проверяется не суммой, а поточечно — сумма совпала бы и
    у неверного разбиения:

    | alpha | 1 | 3/2 | 2 | 3 | 4 | 6 | 8 | 12 |
    |---|---:|---:|---:|---:|---:|---:|---:|---:|
    | покрытие x2 | 46 | 135/2 | 88 | 126 | 160 | 184 | 192 | 192 |

    Числа — у НЕЗАВИСИМОГО митрованного эталона, и сверяются все восемь alpha
    корпуса, а не одна.
    """

    losing_area = []
    for name, polygon in PARTIAL:
        skeleton = build_skeleton(polygon)
        if skeleton.outcome is not SkeletonOutcome.EXACT:
            continue
        partition = build_faces(polygon, skeleton)
        if partition.outcome is not FaceOutcome.EXACT:
            losing_area.append((name, partition.outcome.value, partition.detail))
    assert losing_area == []

    polygon = dict(PARTIAL)["staircase_source_edges_7_0"]
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    assert partition.polygon_doubled_area == 192
    assert partition.area_defect.terms == ()
    # Владельцы — РОВНО рёбра-источники, поимённо: совпадение суммы не
    # доказывает совпадения множества.
    assert {face.owner for face in partition.faces} == {
        (0, 0, 12, 0),
        (0, 12, 0, 0),
    }

    measured = []
    for alpha in PARTIAL_SOURCE_ALPHAS:
        covered = coverage_at(partition, alpha)
        standard = partial_source_standard(polygon, alpha)
        assert covered.outcome is CoverageOutcome.EXACT, alpha
        assert standard.outcome is StandardOutcome.EXACT, alpha
        assert covered.doubled_area.as_rational() == 2 * standard.covered, alpha
        measured.append(covered.doubled_area.as_rational())
    assert measured == [
        Fraction(46),
        Fraction(135, 2),
        Fraction(88),
        Fraction(126),
        Fraction(160),
        Fraction(184),
        Fraction(192),
        Fraction(192),
    ]


@pytest.mark.parametrize(
    "name", ("ell_12_source_edge_4", "staircase_source_edge_6",
             "staircase_source_edges_4_5")
)
def test_the_three_contradicting_figures_are_now_refused_before_any_number(name):
    """Те самые три фигуры: отказ приходит от СКЕЛЕТА, а не от сборщика.

    Проверяется поимённо, а не «отказала как-нибудь»: раньше скелет на них
    отвечал `EXACT`, и именно этот `EXACT` был единственным, что отделяло
    корпус от тихого неверного числа.
    """

    polygon = dict(PARTIAL)[name]
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
    assert (
        build_faces(polygon, skeleton).outcome
        is FaceOutcome.SKELETON_IS_NOT_EXACT
    )
    # Фиктивное схлопывание сосчитано под своим именем, а не пропало.
    assert skeleton.counter(
        refusal_counter(CandidateRefusal.FILTER_SPAN_DOES_NOT_COLLAPSE)
    ) > 0


@pytest.mark.parametrize(
    "name,polygon_doubled_area,swept_doubled_area",
    (
        ("ell_12_source_edge_4", 216, 144),
        ("staircase_source_edge_6", 192, 96),
        ("staircase_source_edges_4_5", 192, 160),
        ("staircase_source_edges_7_0", 192, 192),
    ),
)
def test_what_the_source_can_reach_at_all_is_a_number_from_the_standard(
    name, polygon_doubled_area, swept_doubled_area
):
    """Сколько фигуры источник заметает В ПРЕДЕЛЕ — и почему это решает всё.

    Число берётся у НЕЗАВИСИМОГО эталона при заведомо большой alpha, а не у
    очереди. Оно объясняет, почему `EXACT` у сборщика на первых трёх фигурах
    невозможен НИ ПРИ КАКОЙ починке порядка: граница 1 сборщика требует, чтобы
    сумма граней воспроизвела площадь многоугольника, а источник до части
    фигуры не доходит вовсе — 72, 96 и 32 удвоенных единицы соответственно.
    Четвёртая строка — контроль: там незаметённого НОЛЬ, и потому её отказ
    действительно был чистым дефектом порядка.
    """

    polygon = dict(PARTIAL)[name]
    assert sum(
        signed_double_area(loop.points) for loop in polygon.loops
    ) == polygon_doubled_area
    limit = partial_source_standard(polygon, Fraction(10**6))
    assert limit.outcome is StandardOutcome.EXACT
    assert 2 * limit.covered == swept_doubled_area


