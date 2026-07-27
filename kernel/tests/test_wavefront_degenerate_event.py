"""Кратное одновременное событие в ВЫРОЖДЕННОЙ точке: что там происходит.

Стенд отвечает ровно на один вопрос: почему у креста среди узлов скелета нет
пересечения гребней. Ответ измеряется числом на каждом шаге пути, а не выводится
из чтения кода, — потому что предыдущая записанная причина («ломается порядок
сортировки узлов») была именно выведена, и измерение её опровергло.

Фигура берётся ИЗ КОРПУСА (`wavefront_cases.cross`), а не воспроизводится здесь
по памяти. Это правило, купленное ошибкой: своя копия креста отличалась толщиной
рукава (6 вместо 4), и именно эта разница задавала границу измеряемого явления,
из-за чего закон дефекта был записан без своей области.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

import wavefront_cases
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.event_time import (
    EventTimeOutcome,
    concurrency_time,
)
from cftuv_envelope.wavefront.events import EventKind
from cftuv_envelope.wavefront.skeleton import (
    CandidateRefusal,
    SkeletonOutcome,
    build_skeleton,
    refusal_counter,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1


# Толщина рукава у `wavefront_cases.cross`: `left == bottom == 4`. Отсюда время
# схлопывания горизонтальной полосы равно `tall/2`, вертикальной — `wide/2`.
CROSS_ARM = 4

# Рабочая строка сетки: дефект есть, закон `-(w-t)^2` на ней в силе.
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


def _live_pairs_on_one_moving_line(builder):
    """Соседние ЖИВЫЕ вершины, чьи внешние рёбра лежат на одной прямой.

    Такая пара — это отрезок фронта между двумя вершинами, которые обязаны
    стоять в одной точке во все времена: их внешние прямые совпадают, значит
    обе вершины лежат на пересечении одной и той же пары прямых.
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
        here = builder._vertex_position(vertex, builder.now)
        there = builder._vertex_position(peer, builder.now)
        if here is None or there is None:
            continue
        if not (here.x - there.x).is_zero or not (here.y - there.y).is_zero:
            continue
        found.append((vertex, peer, outer_left, outer_right))
    return found


def test_the_arms_of_a_cross_collapse_in_one_level_of_six_simultaneous_events():
    """ШАГ 1, пункт первый: какой уровень срабатывает и из чего он состоит.

    Горизонтальная полоса креста схлопывается в момент `tall/2`, и это ОДИН
    уровень очереди: шесть событий одного точного времени в четырёх различных
    точках. Четыре из них — разрезы (по одному на каждую вогнутую вершину),
    два — схлопывания торцов рукавов. Числа записаны, потому что «несколько
    событий» и «шесть событий в четырёх точках» — разные утверждения.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    _, levels = _levels_until(figure, Fraction(TALL, 2))
    first = levels[0]
    assert first["time"] == Fraction(TALL, 2)
    assert first["events"] == 6
    assert first["kinds"] == (("EDGE", 2), ("SPLIT", 4))
    assert first["points"] == 4
    # Уровень одного и того же времени снимается не один раз: применение
    # событий рождает новых кандидатов на ТОМ ЖЕ времени. Это тоже число.
    assert [level["time"] for level in levels] == [Fraction(TALL, 2)] * 4
    assert [level["events"] for level in levels] == [6, 8, 2, 2]


def test_every_split_of_that_level_lands_on_a_front_vertex_not_inside_an_edge():
    """ШАГ 1, пункт второй: это не разрезы, а встречи ВЕРШИН.

    Все четыре «разреза» уровня приходят ровно в конец рассекаемого отрезка
    фронта, то есть вогнутая вершина встречает не ребро, а другую вершину. Для
    такой встречи у алгоритма правила нет: разрезом она обрабатывается неверно,
    и неверность немедленно видна числом — рождается ровно столько отрезков
    фронта нулевой длины, сколько было таких «разрезов».
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    builder, _ = _levels_until(figure, Fraction(TALL, 2))
    assert builder.counter_of(
        CandidateRefusal.NO_RULE_SPLIT_HITS_FRONT_VERTEX
    ) == 4
    assert builder.counters["zero_length_front_segments"] == 4


def test_the_zero_length_front_edge_survives_because_its_triple_never_resolves():
    """ШАГ 1, пункт третий: что именно осталось в LAV после уровня.

    После схлопывания рукавов в LAV остаются ДВЕ пары живых вершин, каждая пара
    стоит в одной точке, между вершинами пары — отрезок фронта нулевой длины.
    Внешние рёбра пары — это четыре стенки центрального блока, и они попарно
    коллинеарны и СОНАПРАВЛЕНЫ (скалярное произведение нормалей положительно).

    Пара не схлопывается никогда, и причина названа: тройка прямых её
    edge-события сходится ТОЖДЕСТВЕННО (две из трёх — одна и та же движущаяся
    прямая), поэтому формула `t = -D0/S` отвечает «сошлись навсегда», а
    вызывающий на этот ответ молча ничего не делает.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    builder, _ = _levels_until(figure, Fraction(TALL, 2))
    pairs = _live_pairs_on_one_moving_line(builder)
    assert len(pairs) == 2
    for vertex, peer, outer_left, outer_right in pairs:
        assert outer_left.a * outer_right.a + outer_left.b * outer_right.b > 0
        _, outcome = concurrency_time(
            builder.edges[vertex.prev_edge].line,
            builder.edges[vertex.next_edge].line,
            builder.edges[peer.next_edge].line,
        )
        assert outcome is EventTimeOutcome.WAVEFRONT_TRIPLE_ALWAYS_CONCURRENT


def test_the_codirectional_joint_never_forms_so_it_is_never_even_asked_about():
    """ШАГ 1, пункт четвёртый: разрешение противоречия между двумя замерами.

    Записаны были два измерения, и на первый взгляд они спорят. Первое: ветка
    «позиции вершины нет» срабатывает на кресте, и все срабатывания —
    АНТИпараллельные, сонаправленных ноль. Второе: причина дефекта — вершина
    между двумя сонаправленными стенками.

    Спора нет, и разрешается он так: сонаправленный стык в LAV не РОЖДАЕТСЯ
    вовсе. Вместо одной вершины между стенками стоят ДВЕ, разделённые отрезком
    нулевой длины (тест выше), поэтому вопрос о позиции сонаправленного стыка
    никому не задаётся. Это разные болезни: «не создаётся» и «создаётся и
    молчит», и лечатся они разным.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    skeleton = build_skeleton(figure)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    codirectional = skeleton.counter(
        refusal_counter(CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL)
    )
    antiparallel = skeleton.counter(
        refusal_counter(CandidateRefusal.NO_RULE_JOINT_IS_ANTIPARALLEL)
    )
    assert (codirectional, antiparallel) == (0, 8)


def test_the_walls_meet_at_four_points_and_none_of_them_is_the_ridge_crossing():
    """ШАГ 1, пункт пятый: чем кончается второй вырожденный уровень.

    Стенки блока встречаются в момент `wide/2`. Уровень снова кратный — шесть
    событий, — и его точки лежат на вертикальном гребне `x = 4 + wide/2`. Но
    пересечения гребней среди них НЕТ: две пары вершин, оставшиеся от первого
    уровня, разъехались по ложным траекториям вдоль рухнувших крышек рукавов и
    пришли на гребень в точки `y = 4 + tall/2 -+ (wide - tall)/2`, а не в
    `y = 4 + tall/2`.
    """

    figure = wavefront_cases.cross(wide=WIDE, tall=TALL)
    _, levels = _levels_until(figure, Fraction(WIDE, 2))
    walls = [level for level in levels if level["time"] == Fraction(WIDE, 2)]
    assert walls[0]["events"] == 6
    assert walls[0]["kinds"] == (("EDGE", 2), ("SPLIT", 4))

    skeleton = build_skeleton(figure)
    ridge_x = SqrtSumV1.rational(Fraction(4) + Fraction(WIDE, 2))
    ridge_y = SqrtSumV1.rational(Fraction(4) + Fraction(TALL, 2))
    on_ridge = sorted(
        node.point.y.as_rational()
        for node in skeleton.nodes
        if (node.point.x - ridge_x).is_zero
    )
    assert on_ridge == [3, 5, 5, 7, 7, 17]
    assert not any(
        (node.point.x - ridge_x).is_zero and (node.point.y - ridge_y).is_zero
        for node in skeleton.nodes
    )


def test_a_square_block_puts_every_event_of_the_cross_into_one_single_point():
    """ШАГ 1, пункт шестой: почему `wide == tall` ОТКАЗЫВАЕТ, а не теряет тихо.

    Отказ и тихая потеря сидят на одной фигуре по разные стороны от `w = t`, и
    разница измерима. При `wide != tall` вырожденных точек ДВЕ и в каждой
    встречаются по две вогнутые вершины. При `wide == tall` она ОДНА, и в ней
    встречаются все четыре сразу: уровень несёт 20 событий вместо шести, из них
    16 разрезов, и каждый из них приходит в вершину фронта, а не в ребро.

    Правило «встретились две вершины» ещё можно было бы дописать парой; правила
    «встретились четыре» нет и подавно, поэтому квадратный блок распадается на
    куски, которые очередь потом не сводит, и четыре вершины остаются живыми.
    Это и есть `WAVEFRONT_LEFT_UNRESOLVED`: не другая болезнь, а та же, но без
    остатка, за который смогла бы зацепиться неверная сборка.
    """

    figure = wavefront_cases.cross(wide=TALL, tall=TALL)
    builder, levels = _levels_until(figure, Fraction(TALL, 2))
    assert levels[0]["events"] == 20
    assert levels[0]["kinds"] == (("EDGE", 4), ("SPLIT", 16))
    assert builder.counter_of(
        CandidateRefusal.NO_RULE_SPLIT_HITS_FRONT_VERTEX
    ) == 16
    skeleton = build_skeleton(figure)
    assert skeleton.outcome is SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED


@pytest.mark.parametrize(
    "name,polygon", wavefront_cases.named_corpus(), ids=lambda value: value
)
def test_the_named_corpus_reports_every_refusal_under_a_name(name, polygon):
    """ШАГ 2: ни один отказ кандидата не исчезает без имени.

    Проверка идёт на самом результате: сумма именованных отказов не может быть
    меньше числа отказов без правила, а все члены `CandidateRefusal` обязаны
    иметь свой счётчик в отчёте. Без этого «отказов нет» и «счётчика нет»
    выглядели бы одинаково.
    """

    skeleton = build_skeleton(polygon)
    reported = dict(skeleton.counters)
    for member in CandidateRefusal:
        assert refusal_counter(member) in reported, member
