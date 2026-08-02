"""Очередь событий волнового фронта: то, ради чего срез.

Главное здесь — третий и четвёртый тесты. Третий доказывает, что одновременная
встреча k фронтов в одной точке даёт ОДНО событие с k участниками, а не
k(k-1)/2 попарных. Четвёртый доказывает, что результат не зависит от порядка
входа, и доказывает перебором перестановок, а не заявлением.
"""

from __future__ import annotations

from fractions import Fraction
import itertools

import pytest

from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.event_time import compare_times
from cftuv_envelope.wavefront.events import EventKind, MotorcycleSeam
from cftuv_envelope.wavefront.polygon import (
    LoopV1,
    PolygonOutcome,
    PolygonRejected,
    PolygonV1,
)
from cftuv_envelope.wavefront.skeleton import (
    SkeletonOutcome,
    build_skeleton,
    level_budget,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1


def _time_is_exactly(time, expected: SqrtSumV1) -> bool:
    """Время РАВНО заданной алгебраической величине — точно, без float.

    Сверять через `float(...)` нельзя: оболочка тут уже, чем binary64, и
    сравнение с округлённым эталоном ловило бы не арифметику, а округление.
    Здесь проверяется тождество `dividend = divisor * expected`, то есть
    рациональное совпадение коэффициентов.
    """

    return (time.divisor * expected) == SqrtSumV1.rational(time.dividend)


SQUARE = ((0, 0), (8, 0), (8, 8), (0, 8))
RECTANGLE = ((0, 0), (20, 0), (20, 8), (0, 8))
TRIANGLE = ((0, 0), (12, 0), (0, 12))
DIAMOND = ((0, -8), (8, 0), (0, 8), (-8, 0))
ELL = ((0, 0), (12, 0), (12, 6), (6, 6), (6, 12), (0, 12))
COMB = ((0, 0), (24, 0), (24, 10), (18, 4), (12, 10), (6, 4), (0, 10))
HOLE_OUTER = ((0, 0), (20, 0), (20, 20), (0, 20))
HOLE_INNER = ((8, 8), (12, 8), (12, 12), (8, 12))
TWO_HOLES_OUTER = ((0, 0), (40, 0), (40, 20), (0, 20))
TWO_HOLES = (
    ((6, 6), (12, 6), (12, 14), (6, 14)),
    ((26, 6), (32, 6), (32, 14), (26, 14)),
)

CORPUS = {
    "square": (SQUARE, ()),
    "rectangle": (RECTANGLE, ()),
    "triangle": (TRIANGLE, ()),
    "diamond": (DIAMOND, ()),
    "ell": (ELL, ()),
    "comb": (COMB, ()),
    "hole": (HOLE_OUTER, (HOLE_INNER,)),
    "two_holes": (TWO_HOLES_OUTER, TWO_HOLES),
}


def _skeleton(outer, holes=()):
    return build_skeleton(PolygonV1.build(outer, holes))


# --------------------------------------------------------------------------
# 1. Квадрат схлопывается в точку, время равно половине стороны, точно
# --------------------------------------------------------------------------


def test_the_square_collapses_into_one_point_at_half_its_side():
    skeleton = _skeleton(SQUARE)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert len(skeleton.nodes) == 1
    node = skeleton.nodes[0]
    time = node.time.canonical()
    assert time.divisor.is_rational()
    assert time.dividend / time.divisor.as_rational() == Fraction(4)
    assert node.point.x.as_rational() == Fraction(4)
    assert node.point.y.as_rational() == Fraction(4)
    assert skeleton.counter("peaks") == 1


# --------------------------------------------------------------------------
# 2. Reflex-вершина даёт split, и он совпадает с посчитанным вручную
# --------------------------------------------------------------------------


def test_the_reflex_vertex_splits_where_hand_computation_says():
    """Вершина (6,6) идёт по биссектрисе со скоростью sqrt(2) и встречает
    нижнюю сторону. Вручную: положение (6-t, 6-t), нижний фронт y = t,
    значит 6-t = t, значит t = 3 и точка (3, 3)."""

    skeleton = _skeleton(ELL)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    splits = [
        node
        for node in skeleton.nodes
        if node.kind is EventKind.SPLIT
        or (
            node.kind is EventKind.MULTIWAY
            and EventKind.SPLIT in node.kinds
        )
    ]
    assert len(splits) == 1
    split = splits[0]
    time = split.time.canonical()
    assert time.dividend / time.divisor.as_rational() == Fraction(3)
    assert split.point.x.as_rational() == Fraction(3)
    assert split.point.y.as_rational() == Fraction(3)


def test_both_teeth_of_the_comb_split_at_the_same_exact_instant():
    skeleton = _skeleton(COMB)
    splits = [
        node
        for node in skeleton.nodes
        if node.kind is EventKind.SPLIT
        or (
            node.kind is EventKind.MULTIWAY
            and EventKind.SPLIT in node.kinds
        )
    ]
    assert len(splits) == 2
    assert compare_times(splits[0].time, splits[1].time) == 0
    # Время иррационально: 4/(1 + sqrt(2)) = 4*sqrt(2) - 4.
    assert not splits[0].time.divisor.is_rational()
    assert _time_is_exactly(
        splits[0].time, SqrtSumV1.radical(4, 2) - SqrtSumV1.rational(4)
    )


# --------------------------------------------------------------------------
# 3. БЛОКЕР: k фронтов в одной точке — ОДНО событие с k участниками
# --------------------------------------------------------------------------


def test_three_fronts_meeting_at_one_point_are_one_event_with_three_participants():
    """Треугольник: три фронта сходятся в инцентре ОДНОВРЕМЕННО.

    Попарный крой дал бы здесь ТРИ события. Очередь по времени даёт одно, и
    участников у него три. Время при этом иррационально (12 - 6*sqrt(2)) —
    то есть одновременность доказана, а не подогнана к решётке.
    """

    skeleton = _skeleton(TRIANGLE)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert len(skeleton.nodes) == 1
    node = skeleton.nodes[0]
    assert node.converging_vertices == 3
    assert len(node.participants) == 3
    assert not node.time.divisor.is_rational()
    assert _time_is_exactly(
        node.time, SqrtSumV1.rational(12) - SqrtSumV1.radical(6, 2)
    )


def test_four_fronts_meeting_at_one_point_are_one_event_with_four_participants():
    for outer in (SQUARE, DIAMOND):
        skeleton = _skeleton(outer)
        assert len(skeleton.nodes) == 1
        assert skeleton.nodes[0].converging_vertices == 4
        assert len(skeleton.nodes[0].participants) == 4


def test_two_independent_collapses_at_one_instant_stay_two_atomic_events():
    """Отрицательный контроль, без которого предыдущий тест ничего не стоит.

    У прямоугольника 20x8 ДВА события в один и тот же момент t = 4, но в
    РАЗНЫХ точках. Они никогда не были multiway и обязаны остаться двумя
    отдельными событиями по два участника, иначе «починка multiway» свелась бы
    к тому, чтобы слить всё, что совпало по времени.
    """

    skeleton = _skeleton(RECTANGLE)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert len(skeleton.nodes) == 2
    first, second = skeleton.nodes
    assert compare_times(first.time, second.time) == 0
    assert first.point != second.point
    assert first.converging_vertices == 2
    assert second.converging_vertices == 2
    assert skeleton.counter("multi_participant_nodes") == 0


# --------------------------------------------------------------------------
# 4. Перестановочная инвариантность — перебором, а не заявлением
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_any_input_order_gives_one_and_the_same_digest(name):
    outer, holes = CORPUS[name]
    reference = semantic_digest(build_skeleton(PolygonV1.build(outer, holes)))
    seen = {reference}
    for shift in range(len(outer)):
        rotated_outer = LoopV1(tuple(outer)).rotated(shift).points
        for order in itertools.permutations(range(len(holes))):
            for hole_shift in range(len(holes[0]) if holes else 1):
                rotated_holes = tuple(
                    LoopV1(tuple(holes[index])).rotated(hole_shift).points
                    for index in order
                )
                seen.add(
                    semantic_digest(
                        build_skeleton(
                            PolygonV1.build(rotated_outer, rotated_holes)
                        )
                    )
                )
    assert seen == {reference}, f"{name}: порядок входа изменил результат"


def test_reversing_the_declared_winding_changes_nothing():
    """Ориентация нормируется на входе, значит обход задом наперёд — тот же вход."""

    forward = semantic_digest(build_skeleton(PolygonV1.build(ELL)))
    backward = semantic_digest(build_skeleton(PolygonV1.build(tuple(reversed(ELL)))))
    assert forward == backward


# --------------------------------------------------------------------------
# 5. Дыры: внутренний фронт встречается с внешним
# --------------------------------------------------------------------------


def test_the_front_of_a_hole_meets_the_outer_front_as_an_ordinary_event():
    """`DOMAIN_HOLE_LOOPS = 0` во всех семи полевых доменах, поэтому только так.

    Дыра испускает фронт наружу; встреча с внешним фронтом — обычное
    split-событие, сливающее два LAV в один. Проверяется, что событие
    ПОСТРОЕНО, а не что путь не упал.
    """

    skeleton = _skeleton(HOLE_OUTER, (HOLE_INNER,))
    assert skeleton.outcome is SkeletonOutcome.EXACT
    splits = [
        node
        for node in skeleton.nodes
        if node.kind is EventKind.SPLIT
        or (
            node.kind is EventKind.MULTIWAY
            and EventKind.SPLIT in node.kinds
        )
    ]
    assert len(splits) == 4
    for node in splits:
        time = node.time.canonical()
        assert time.dividend / time.divisor.as_rational() == Fraction(4)
    corners = {
        (node.point.x.as_rational(), node.point.y.as_rational())
        for node in splits
    }
    assert corners == {
        (Fraction(4), Fraction(4)),
        (Fraction(4), Fraction(16)),
        (Fraction(16), Fraction(4)),
        (Fraction(16), Fraction(16)),
    }


def test_two_holes_aiming_at_one_edge_are_cut_at_once_not_one_after_another():
    """Четыре угла двух дыр целят в верхнее ребро в один и тот же момент.

    Последовательное применение здесь измеримо ломалось: дайджест зависел от
    порядка входа. Уровень применяется целиком, поэтому не зависит.
    """

    skeleton = _skeleton(TWO_HOLES_OUTER, TWO_HOLES)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    # Считать надо ОБА разбора, а не один: с этого среза часть таких кандидатов
    # приходит не в ребро, а в вершину фронта, и разбирается встречей вершин.
    # Сумма не изменилась — изменилось, каким правилом каждый разобран.
    assert (
        skeleton.counter("split_events")
        + skeleton.counter("vertex_meeting_events") * 2
    ) >= 8


# --------------------------------------------------------------------------
# 6. Именованный отказ там, где ответа нет
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "points,expected",
    [
        (((0, 0), (1, 0)), PolygonOutcome.LOOP_TOO_SHORT),
        (((0, 0), (4, 0), (0, 0), (4, 4)), PolygonOutcome.LOOP_HAS_REPEATED_POINT),
        (((0, 0), (4, 0), (8, 0)), PolygonOutcome.LOOP_HAS_ZERO_AREA),
        (
            ((0, 0), (4, 0), (Fraction(1, 2), 4)),
            PolygonOutcome.LOOP_COORDINATE_IS_NOT_INTEGER,
        ),
    ],
)
def test_a_rejected_input_is_rejected_by_name(points, expected):
    with pytest.raises(PolygonRejected) as failure:
        LoopV1(points)
    assert failure.value.outcome is expected


def test_exhausting_the_declared_budget_is_a_named_outcome_not_a_silent_return():
    """У Трики это `bestt == 1e100 -> return`. Здесь — исход с именем.

    Бюджет подменяется на заведомо малый, потому что честный вход его не
    исчерпывает: иначе тест доказывал бы наличие дефекта, а не наличие имени.
    """

    original = skeleton_module.level_budget
    skeleton_module.level_budget = lambda polygon: 1
    try:
        skeleton = build_skeleton(PolygonV1.build(COMB))
    finally:
        skeleton_module.level_budget = original
    assert skeleton.outcome is SkeletonOutcome.LEVEL_BUDGET_EXHAUSTED
    assert skeleton.levels == 1


def test_the_unbuilt_extended_wavefront_is_a_named_seam_not_a_silence():
    """Типов событий четыре; два из них требуют РАСШИРЕННОГО фронта, которого нет.

    Счётчики `start_events`/`switch_events` существуют и равны нулю, а причина
    названа. Ноль с названной причиной — измерение; отсутствие счётчика —
    умолчание.

    Прежняя причина («граф не построен») отменена срезом Q2 и здесь же
    проверяется отменённой: значение осталось в перечислении, чтобы отмена была
    видимой, но действующий шов — уже другой.
    """

    assert {kind.value for kind in EventKind} == {
        "EDGE",
        "MULTIWAY",
        "SPLIT",
        "START",
        "SWITCH",
    }
    assert (
        MotorcycleSeam.current()
        is MotorcycleSeam.EXTENDED_WAVEFRONT_NOT_BUILT_IN_THIS_SLICE
    )
    assert (
        MotorcycleSeam.current()
        is not MotorcycleSeam.MOTORCYCLE_GRAPH_NOT_BUILT_IN_THIS_SLICE
    )
    for outer, holes in CORPUS.values():
        skeleton = _skeleton(outer, holes)
        assert skeleton.counter("start_events") == 0
        assert skeleton.counter("switch_events") == 0


# --------------------------------------------------------------------------
# 7. Функция объявила две границы — обе проверяются на её же результате
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_emitted_times_never_decrease(name):
    """Граница первая: очередь по определению выдаёт неубывающее время."""

    outer, holes = CORPUS[name]
    skeleton = _skeleton(outer, holes)
    times = [node.time for node in skeleton.nodes]
    for earlier, later in zip(times, times[1:]):
        assert compare_times(earlier, later) <= 0, f"{name}: время пошло назад"


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_result_stays_inside_the_budget_the_function_declares(name):
    """Граница вторая: уровней не больше объявленного бюджета."""

    outer, holes = CORPUS[name]
    polygon = PolygonV1.build(outer, holes)
    skeleton = build_skeleton(polygon)
    assert skeleton.levels <= level_budget(polygon)
    assert skeleton.outcome is not SkeletonOutcome.LEVEL_BUDGET_EXHAUSTED


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_every_event_consumes_at_least_one_wavefront_vertex(name):
    """Третье следствие тех же границ: событие без участников не событие."""

    outer, holes = CORPUS[name]
    skeleton = _skeleton(outer, holes)
    assert skeleton.nodes
    for node in skeleton.nodes:
        assert node.converging_vertices >= 1
        assert len(node.participants) >= 2


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_wavefront_is_fully_consumed(name):
    """Поле закрывается полностью: незакрытый фронт — именованный исход."""

    outer, holes = CORPUS[name]
    skeleton = _skeleton(outer, holes)
    assert skeleton.outcome is SkeletonOutcome.EXACT


# --------------------------------------------------------------------------
# 8. Состязательный корпус: не образцы, а фигуры общего положения
# --------------------------------------------------------------------------


def _star(vertices: int, seed: int, radius: int = 4096):
    """Звёздчатый контур на решётке. Псевдослучайность своя и детерминированная.

    `random` здесь означал бы, что повтор прогона проверяет другой корпус, то
    есть что зелёный тест ничего не удостоверяет.
    """

    import math as _math

    state = seed * 6364136223846793005 + 1442695040888963407
    points = []
    for index in range(vertices):
        state = (state * 6364136223846793005 + 1442695040888963407) % (1 << 64)
        angle = 2 * _math.pi * index / vertices
        scale = 0.5 + (state % 1000) / 2000.0
        points.append(
            (
                int(round(radius * scale * _math.cos(angle))),
                int(round(radius * scale * _math.sin(angle))),
            )
        )
    if len(set(points)) != vertices:
        return None
    try:
        return PolygonV1.build(tuple(points))
    except PolygonRejected:
        return None


@pytest.mark.parametrize("seed", range(8))
def test_a_general_position_polygon_is_resolved_exactly(seed):
    polygon = _star(9, seed)
    assert polygon is not None
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert skeleton.levels <= level_budget(polygon)
    times = [node.time for node in skeleton.nodes]
    for earlier, later in zip(times, times[1:]):
        assert compare_times(earlier, later) <= 0


@pytest.mark.parametrize("seed", range(4))
def test_a_general_position_polygon_is_permutation_invariant(seed):
    polygon = _star(9, seed)
    assert polygon is not None
    points = polygon.outer.points
    reference = semantic_digest(build_skeleton(polygon))
    for shift in range(len(points)):
        rotated = PolygonV1.build(LoopV1(points).rotated(shift).points)
        assert semantic_digest(build_skeleton(rotated)) == reference


def test_general_position_inputs_are_resolved_and_never_go_back_in_time():
    """Сто контуров общего положения: все EXACT, время нигде не убывает.

    Тест заведён как замер НЕПОЛНОТЫ наивного поиска split — и нашёл вместо
    неё другое: 15 контуров из 240 кончались `WAVEFRONT_LEFT_UNRESOLVED`
    потому, что кандидат мог попасть в очередь со временем РАНЬШЕ уже снятого
    уровня. Отсечка по времени текущего уровня закрыла все 240 и заодно
    вернула объявленную границу «время не убывает», которую тот же тест ловил
    нарушенной. Оба факта держатся здесь, а не в прозе.
    """

    total = unresolved = 0
    for vertices in (5, 7, 9, 12, 16):
        for seed in range(20):
            polygon = _star(vertices, seed)
            if polygon is None:
                continue
            total += 1
            skeleton = build_skeleton(polygon)
            unresolved += skeleton.outcome is not SkeletonOutcome.EXACT
            times = [node.time for node in skeleton.nodes]
            for earlier, later in zip(times, times[1:]):
                assert compare_times(earlier, later) <= 0
    assert total == 100
    assert unresolved == 0, f"неразрешённых {unresolved} из {total}"


def test_a_candidate_from_the_past_never_reaches_the_queue():
    """Отсечка по времени уровня — свойство очереди, а не следствие корпуса.

    Проверяется на самом состоянии: после каждого снятого уровня время
    ЛЮБОГО кандидата в куче не меньше времени этого уровня.
    """

    builder = skeleton_module._Builder(PolygonV1.build(COMB))
    levels = 0
    while len(builder.queue):
        level = builder.queue.pop_level()
        builder.now = level[0].time
        levels += 1
        builder._apply_level(level)
        builder._close_short_lavs()
        for entry in builder.queue._heap:
            assert compare_times(entry.event.time, builder.now) >= 0
    assert levels > 0
