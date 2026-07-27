"""Кратное одновременное событие в ВЫРОЖДЕННОЙ точке: измерение и починка.

Стенд отвечает на один вопрос: что происходит там, где у креста терялось
пересечение гребней. Ответ измеряется числом на каждом шаге пути, а не выводится
из чтения кода, — потому что предыдущая записанная причина («ломается порядок
сортировки узлов») была именно выведена, и измерение её опровергло.

ЧТО ИЗМЕРЕНИЕ ПОКАЗАЛО (числа на кресте 6x4, до починки):

| вопрос | число |
|---|---|
| событий в уровне схлопывания рукавов | 6 (4 разреза + 2 схлопывания) в 4 точках |
| уровень того же времени снимался | 4 раза подряд: 6, 8, 2, 2 события |
| «разрезов», пришедших в ВЕРШИНУ, а не в ребро | 4 из 4 |
| отрезков фронта нулевой длины после уровня | 4 |
| пар живых вершин, стоящих в одной точке | 2 |
| стыков сонаправленных, о которых спросили | 0 |
| стыков антипараллельных | 8 |

Последние две строки разрешают противоречие двух прежних замеров. Записано было,
что причина дефекта — вершина между сонаправленными стенками, и одновременно
что таких стыков ноль. Спора нет: сонаправленный стык не СОЗДАВАЛСЯ вовсе.
Вместо одной вершины между стенками стояли ДВЕ, разделённые отрезком нулевой
длины, поэтому вопрос о её позиции никому не задавался. «Не создаётся» и
«создаётся и молчит» — разные болезни, и лечатся они разным.

ЧТО ПОЧИНЕНО. Кандидат, чья точка совпала с концом рассекаемого отрезка, — не
разрез, а встреча ВЕРШИН, и разбирается она пересоединением по лучам. Вершина с
развёрнутым углом получила позицию (скольжение по общей прямой) и собственные
времена событий. Дефект стал нулевым на всей сетке 3..9, включая диагональ.

Фигура берётся ИЗ КОРПУСА (`wavefront_cases.cross`), а не воспроизводится здесь
по памяти. Это правило, купленное ошибкой: своя копия креста отличалась толщиной
рукава (6 вместо 4), и именно эта разница задавала границу измеряемого явления,
из-за чего закон дефекта был записан без своей области. Толщина теперь параметр
самой корпусной фигуры, поэтому второй копии не заводится и здесь.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

import wavefront_cases
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.event_time import (
    EventPointV1,
    EventTimeOutcome,
    EventTimeV1,
    concurrency_time,
    sliding_point,
    sliding_time,
)
from cftuv_envelope.wavefront.skeleton import (
    CandidateRefusal,
    SkeletonOutcome,
    build_skeleton,
    refusal_counter,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1


# Толщина рукава у `wavefront_cases.cross` по умолчанию: `left == bottom == 4`.
# Отсюда время схлопывания горизонтальной полосы равно `tall/2`, вертикальной —
# `wide/2`.
CROSS_ARM = 4

# Рабочая строка сетки: до починки дефект на ней был, и закон `-(w-t)^2` в силе.
WIDE, TALL = 6, 4


def _levels_until(polygon, limit: Fraction):
    """Прогон очереди по уровням до заданного времени включительно.

    Возвращает сам построитель и список описаний уровней. Стенд ведёт цикл сам,
    а не читает `SkeletonV1`, потому что вопрос среза — про УРОВЕНЬ, а готовый
    результат уровней уже не помнит.
    """

    builder = skeleton_module._Builder(polygon)
    described: list[dict] = []
    while len(builder.queue):
        level = builder.queue.pop_level()
        low, _ = level[0].time.enclosure()
        if low > limit:
            break
        builder.now = level[0].time
        kinds: dict[str, int] = {}
        for event in level:
            kinds[event.kind.value] = kinds.get(event.kind.value, 0) + 1
        described.append(
            {
                "time": low,
                "events": len(level),
                "kinds": tuple(sorted(kinds.items())),
                "points": len(
                    {
                        (event.point.x.terms, event.point.y.terms)
                        for event in level
                    }
                ),
            }
        )
        builder._apply_level(level)
        if builder.refusal is not None:
            break
        builder._close_short_lavs()
    return builder, described


def _projection_of(line, x: int, y: int) -> SqrtSumV1:
    """Проекция целой точки на направление прямой — тем же правилом, что в коде."""

    return skeleton_module._project(
        line, EventPointV1(SqrtSumV1.rational(x), SqrtSumV1.rational(y))
    )


def _at(elapsed: int) -> EventTimeV1:
    """Целый момент времени в том же виде, в каком его хранит очередь."""

    return EventTimeV1(Fraction(elapsed), SqrtSumV1.rational(1))


def _live_pairs_on_one_moving_line(builder):
    """Соседние ЖИВЫЕ вершины, чьи внешние рёбра лежат на одной прямой.

    Такая пара — отрезок фронта между двумя вершинами, которые обязаны стоять в
    одной точке во все времена: их внешние прямые совпадают. Ровно из-за таких
    пар сонаправленный стык не рождался; после починки их не должно остаться ни
    одной, и это проверяется.
    """

    found = []
    for vertex in builder.vertices:
        if not vertex.alive:
            continue
        peer = builder.vertices[vertex.next]
        if peer.ident == vertex.ident:
            continue
        outer_left = builder.edges[vertex.prev_edge].line
        outer_right = builder.edges[peer.next_edge].line
        if outer_left.a * outer_right.b - outer_right.a * outer_left.b:
            continue
        here = builder._position(vertex, builder.now)
        there = builder._position(peer, builder.now)
        if here is None or there is None:
            continue
        if not (here.x - there.x).is_zero or not (here.y - there.y).is_zero:
            continue
        found.append((vertex, peer, outer_left, outer_right))
    return found


def test_the_arms_of_a_cross_collapse_in_one_level_of_six_simultaneous_events():
    """Какой уровень срабатывает в вырожденной точке и из чего он состоит.

    Горизонтальная полоса креста схлопывается в момент `tall/2`, и это ОДИН
    уровень очереди: шесть событий одного точного времени в четырёх различных
    точках. Четыре из них приходят от вогнутых вершин, два — схлопывания торцов
    рукавов. Числа записаны, потому что «несколько событий» и «шесть событий в
    четырёх точках» — разные утверждения.

    Уровень того же времени снимается не один раз: применение рождает новых
    кандидатов на ТОМ ЖЕ времени. До починки таких заходов было четыре
    (6, 8, 2, 2 события) — ровно потому, что разрезы плодили отрезки нулевой
    длины и те схлопывались следующими заходами. Теперь их два.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    _, levels = _levels_until(figure, Fraction(TALL, 2))
    first = levels[0]
    assert first["time"] == Fraction(TALL, 2)
    assert first["events"] == 6
    assert first["kinds"] == (("EDGE", 2), ("SPLIT", 4))
    assert first["points"] == 4
    assert [level["time"] for level in levels] == [Fraction(TALL, 2)] * 2
    assert [level["events"] for level in levels] == [6, 2]


def test_every_split_of_that_level_is_a_meeting_of_vertices_not_a_cut():
    """Это не разрезы, а встречи ВЕРШИН, и разбираются они как встречи.

    Все четыре кандидата уровня приходят ровно в конец рассекаемого отрезка
    фронта, то есть вогнутая вершина встречает не ребро, а другую вершину. До
    починки правила для этого не было: разрезом такая встреча обрабатывалась
    неверно и рождала ровно четыре отрезка фронта нулевой длины. Теперь четыре
    кандидата дают ДВЕ встречи вершин — по две вершины в каждой из двух
    вырожденных точек.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    builder, _ = _levels_until(figure, Fraction(TALL, 2))
    assert builder.counters["vertex_meeting_events"] == 2


def test_the_codirectional_joint_is_born_as_ONE_vertex_and_it_slides():
    """Что осталось в LAV после уровня: одна вершина вместо пары.

    До починки после схлопывания рукавов оставались ДВЕ пары живых вершин,
    каждая пара стояла в одной точке с отрезком нулевой длины между вершинами.
    Пара не схлопывалась никогда: тройка прямых её edge-события сходилась
    ТОЖДЕСТВЕННО, и на этот ответ вызывающий молча ничего не делал.

    Теперь на месте каждой пары стоит ОДНА вершина с развёрнутым углом. Её
    соседние рёбра — стенки блока, коллинеарные и сонаправленные (скалярное
    произведение нормалей положительно), и позиция у неё есть: она скользит по
    общей прямой, не сдвигаясь вдоль неё.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    builder, _ = _levels_until(figure, Fraction(TALL, 2))
    assert _live_pairs_on_one_moving_line(builder) == []
    sliding = [
        vertex
        for vertex in builder.vertices
        if vertex.alive and vertex.sliding is not None
    ]
    assert len(sliding) == 2
    for vertex in sliding:
        first = builder.edges[vertex.prev_edge].line
        second = builder.edges[vertex.next_edge].line
        assert first.a * second.b - second.a * first.b == 0
        assert first.a * second.a + first.b * second.b > 0
        assert builder._position(vertex, builder.now) is not None


def test_the_sliding_vertex_keeps_its_place_along_the_line_and_only_that():
    """Скольжение — точное утверждение, и оно проверяется на самих числах.

    Вершина с развёрнутым углом идёт ПЕРПЕНДИКУЛЯРНО общей прямой, потому что
    биссектриса развёрнутого угла перпендикулярна обеим сторонам. Значит её
    проекция вдоль прямой не меняется ни в один момент, а расстояние до
    исходного положения прямой равно прошедшему времени.

    Проверяется на стенке креста `x = 4` (ребро 11, `(4,4) -> (4,0)`): точка,
    закреплённая на высоте 6, обязана идти по `y = 6`, `x = 4 + t`.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    builder = skeleton_module._Builder(figure)
    wall, facing = builder.edges[11].line, builder.edges[1].line
    frozen = _projection_of(wall, 6, 6)

    # Стенка `x = 4` идёт вправо с единичной скоростью, значит закреплённая на
    # высоте 6 точка обязана стоять в `(4 + t, 6)` в любой момент.
    for elapsed in (0, 1, 2, 3):
        where = sliding_point(wall, frozen, _at(elapsed))
        assert (where.x.as_rational(), where.y.as_rational()) == (
            Fraction(4 + elapsed),
            Fraction(6),
        )

    # Встреча со встречной стенкой `x = 10` (ребро 1): стенки сходятся на
    # `x = 7`, то есть каждая проходит по 3.
    time, outcome = sliding_time(wall, frozen, facing)
    assert outcome is EventTimeOutcome.EXACT
    where = sliding_point(wall, frozen, time)
    assert (where.x.as_rational(), where.y.as_rational()) == (
        Fraction(7),
        Fraction(6),
    )


def test_the_ridge_crossing_is_the_meeting_of_the_two_sliding_vertices():
    """Чем кончается второй вырожденный уровень: узел в пересечении гребней.

    Стенки блока встречаются в момент `wide/2`. Уровень снова кратный — шесть
    событий, — и его точки лежат на вертикальном гребне `x = 4 + wide/2`. До
    починки две пары вершин разъезжались по ложным траекториям вдоль рухнувших
    крышек рукавов и приходили на гребень в `y = 4 + tall/2 -+ (wide - tall)/2`:
    узлы стояли в 3, 5, 5, 7, 7, 17, а самого пересечения `(7, 6)` не было.

    Теперь на гребне ровно три узла — 3, 6, 17, — и средний из них есть встреча
    двух скользящих вершин.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    _, levels = _levels_until(figure, Fraction(WIDE, 2))
    walls = [level for level in levels if level["time"] == Fraction(WIDE, 2)]
    assert walls[0]["events"] == 6
    assert walls[0]["kinds"] == (("EDGE", 2), ("SPLIT", 4))

    skeleton = build_skeleton(figure)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    ridge_x = SqrtSumV1.rational(Fraction(4) + Fraction(WIDE, 2))
    on_ridge = sorted(
        node.point.y.as_rational()
        for node in skeleton.nodes
        if (node.point.x - ridge_x).is_zero
    )
    assert on_ridge == [3, 6, 17]
    assert skeleton.counter("vertex_meeting_events") == 3


def test_a_square_block_puts_every_event_of_the_cross_into_one_single_point():
    """Почему `wide == tall` отказывал, и почему теперь не отказывает.

    Отказ и тихая потеря сидели на одной фигуре по разные стороны от `w = t`, и
    разница измерима. При `wide != tall` вырожденных точек ДВЕ и в каждой
    встречаются по две вогнутые вершины. При `wide == tall` она ОДНА, и в ней
    встречаются все четыре сразу: уровень несёт 20 событий вместо шести.

    Правило сшивки по лучам разбирает и такую встречу, потому что оно не про
    «две вершины», а про то, какие концы отрезков фронта совпали направлением.
    Четыре вогнутые вершины дают четыре аннигилирующие пары и ни одного остатка,
    поэтому квадратный блок больше не оставляет неразрешённого фронта.
    """

    figure = wavefront_cases.cross(wide=TALL, tall=TALL)
    builder, levels = _levels_until(figure, Fraction(TALL, 2))
    assert levels[0]["events"] == 20
    assert levels[0]["kinds"] == (("EDGE", 4), ("SPLIT", 16))
    assert builder.counters["vertex_meeting_events"] == 1
    skeleton = build_skeleton(figure)
    assert skeleton.outcome is SkeletonOutcome.EXACT


def test_the_triple_that_is_always_concurrent_is_never_asked_about_a_sliding_vertex():
    """Вырожденной тройке больше не задаётся вопрос, на который она не отвечает.

    У вершины с развёрнутым углом две соседние прямые — ОДНА движущаяся прямая,
    поэтому определитель тройки равен нулю тождественно и `concurrency_time`
    отвечает «сошлись навсегда» вместо времени. Это не «события нет», это
    неверно заданный вопрос. Верный вопрос задаёт `sliding_time`, и здесь
    проверяется, что оба ответа расходятся именно так.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    builder = skeleton_module._Builder(figure)
    wall_low, wall_high, facing = (
        builder.edges[11].line,
        builder.edges[7].line,
        builder.edges[1].line,
    )
    _, outcome = concurrency_time(wall_high, wall_low, facing)
    assert outcome is EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT
    frozen = _projection_of(wall_low, 6, 6)
    time, sliding_outcome = sliding_time(wall_low, frozen, facing)
    assert sliding_outcome is EventTimeOutcome.EXACT
    assert time is not None


@pytest.mark.parametrize(
    "name,polygon", wavefront_cases.named_corpus(), ids=lambda value: value
)
def test_the_named_corpus_reports_every_refusal_under_a_name(name, polygon):
    """Ни один отказ кандидата не исчезает без имени.

    Проверка идёт на самом результате: все члены `CandidateRefusal` обязаны
    иметь свой счётчик в отчёте. Без этого «отказов нет» и «счётчика нет»
    выглядели бы одинаково.
    """

    skeleton = build_skeleton(polygon)
    reported = dict(skeleton.counters)
    for member in CandidateRefusal:
        assert refusal_counter(member) in reported, member


@pytest.mark.parametrize(
    "name,polygon", wavefront_cases.named_corpus(), ids=lambda value: value
)
def test_no_codirectional_joint_is_ever_left_without_a_position(name, polygon):
    """Развёрнутый стык больше не остаётся без правила НИ РАЗУ на корпусе.

    Это и есть проверка починки на собственном результате: счётчик «стык
    сонаправлен, позиции нет» обязан быть нулём везде. Антипараллельный стык
    нулём не обязан — у него позиции нет по существу, фронт там вывернулся, — но
    такая вершина обязана умереть, и это ловит `WAVEFRONT_LEFT_UNRESOLVED`.
    """

    skeleton = build_skeleton(polygon)
    assert skeleton.counter(
        refusal_counter(CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL)
    ) == 0
    assert skeleton.outcome is SkeletonOutcome.EXACT
