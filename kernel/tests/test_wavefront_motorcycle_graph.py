"""Motorcycle graph: теорема 2.11 в исполняемой форме и сверка с перебором.

Главный тест здесь третий: найденные split-события обязаны СОВПАСТЬ с полным
перебором — ни пропущенного, ни лишнего, на всём корпусе. Оптимизация законна
ровно настолько, насколько даёт тот же ответ, и проверять это нужно значением, а
не рассуждением о том, почему фильтр консервативен.

Второй по важности — первый: теорема 2.11 не должна остаться заявлением.
Проверяется она не на своей же формуле, а на РЕЗУЛЬТАТЕ пути полного перебора:
ни одна reflex-вершина волны не переживает крушение своего мотоцикла. Если бы
крушение считалось слишком рано, этот тест упал бы прежде дифференциального.
"""

from __future__ import annotations

from fractions import Fraction
import itertools

import pytest

from cftuv_envelope.wavefront import motorcycle as motorcycle_module
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.event_time import (
    ZERO_TIME,
    EventTimeV1,
    compare_times,
    event_point,
)
from cftuv_envelope.wavefront.events import EventKind, MotorcycleSeam
from cftuv_envelope.wavefront.motorcycle import (
    CrashKind,
    SupportLineV1,
    TraceCandidateIndexV1,
    TraceOutcome,
    build_motorcycle_graph,
    march_budget,
    walls_of,
)
from cftuv_envelope.wavefront.polygon import (
    FanSupportV1,
    LoopV1,
    PolygonRejected,
    PolygonV1,
    VertexFanV1,
    with_vertex_fans,
)
from cftuv_envelope.wavefront.skeleton import (
    SkeletonOutcome,
    SplitSearch,
    build_skeleton,
)
from cftuv_envelope.wavefront.sqrt_sum import SIGN_COUNTS, SqrtSumV1, reset_sign_counts


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


def test_motorcycle_prime_universe_includes_a_fan_only_factor(monkeypatch):
    polygon = with_vertex_fans(
        PolygonV1.build(ELL),
        (
            VertexFanV1(
                (6, 6),
                (FanSupportV1(-1, -2, 5),),
            ),
        ),
    )
    observed_q = []
    original = motorcycle_module._prime_universe_from_q_values

    def capture(q_values):
        observed_q.append(q_values)
        return original(q_values)

    monkeypatch.setattr(
        motorcycle_module,
        "_prime_universe_from_q_values",
        capture,
    )
    build_motorcycle_graph(polygon)
    assert len(observed_q) == 1
    assert observed_q[0][-1] == 5
    assert original(observed_q[0]) == (5,)


# Дайджесты, снятые с коммита ДО этого среза (`1586f2d`) и обязанные не
# сдвинуться ни на один. Это главные ворота среза: motorcycle graph меняет цену
# поиска и не имеет права менять семантику результата. Значения проверены на
# извлечённой копии базового пакета, а не «по памяти».
# Дайджест скелета на каждой фигуре корпуса. Это ХРАПОВИК модуля, а не эталон
# конвейера: `wavefront/` в конвейер не подключён, поэтому ни одна фикстура и ни
# один артефакт от этих чисел не зависят.
#
# СДВИГ 2026-07-27, три фигуры из восьми, и он объявлен, а не подшит. Срез
# «кратное одновременное событие в вырожденной точке» изменил разбор случая, где
# вогнутая вершина приходит не в ребро, а в ВЕРШИНУ фронта: раньше это
# обрабатывалось разрезом и рождало отрезок-близнец нулевой длины, теперь —
# пересоединением. Сдвинулись ровно те фигуры, где такая встреча случается:
#
# | фигура | встреч вершин | было | стало |
# |---|---:|---|---|
# | `ell`       | 1 | f616d56aa63f478a… | b521cec89869c506… |
# | `hole`      | 4 | f528c104a9f31047… | ea480720c06fa04e… |
# | `two_holes` | 4 | 9c7ff34dcbcce234… | 5d474a7e6be2daca… |
#
# Новый ответ проверен НЕ этим числом: `ell` и `hole` сверены с независимым
# митрованным эталоном (`test_wavefront_mitered_standard.py`), у обоих сумма
# площадей граней точно равна площади фигуры. Остальные пять фигур не
# сдвинулись — встреч вершин на них нет.
#
# СДВИГ 2026-07-27 ВТОРОЙ, ВОСЕМЬ ФИГУР ИЗ ВОСЬМИ, и он ДРУГОГО РОДА: сменился
# не ответ, а ЗАПИСЬ участника. Ключом участника была несущая прямая
# `(a, b, c, q)`, стало вхождение ребра `(x0, y0, x1, y1)`; в дайджест участники
# входят буквально, поэтому сдвинулось всё восемь. Значения ниже — прежние
# рядом с новыми:
#
# | фигура | было (ключ — прямая) | стало (ключ — вхождение) |
# |---|---|---|
# | `comb`      | e2c414a29a0a86b3… | b454deaf48ae7849… |
# | `diamond`   | b5df4dfbe4debfd0… | 9ea7f5e065760f25… |
# | `ell`       | b521cec89869c506… | 1c329d51eff1a96a… |
# | `hole`      | ea480720c06fa04e… | b631437d083548ec… |
# | `rectangle` | c54d463de4123011… | 881d8d89ae7490ba… |
# | `square`    | 7cadb8a12f8edacf… | 05db9afe7cdbb59c… |
# | `triangle`  | f701cba697d9f8e9… | dfdcbeed3283fab0… |
# | `two_holes` | 5d474a7e6be2daca… | b5e81ff224868236… |
#
# Условие «новый ответ проверен НЕ этим числом» выполнено СИЛЬНЕЕ, чем обычно:
# если перевести участников нового скелета обратно в прямые, получается прежнее
# замороженное число ПОБИТОВО, на всех восьми фигурах и обоих поисках. То есть
# доказано не «новый ответ тоже верен», а «ответ ТОТ ЖЕ САМЫЙ». Это проверяет
# `test_the_digest_shift_is_the_key_encoding_and_not_the_answer`, и прежние
# значения для него лежат в `DIGESTS_BEFORE_THE_PARTICIPANT_KEY_CHANGED`.
#
# Сверх того закрыт открытый счёт по `two_holes`: прежний сдвиг был принят на
# доверии, потому что грани этой фигуры отказывали. Теперь они не отказывают —
# сумма площадей 1408 из 1408, а покрытие совпадает с независимым митрованным
# эталоном (180 при alpha = 1, 368 при alpha = 2).
FROZEN_DIGESTS = {
    "comb": "b454deaf48ae78496745f5609623e8cc60231a806eb4932db9b84a73d3263ff1",
    "diamond": "9ea7f5e065760f256c71e23f79b9b9a0eda3c0fc8e341da1c743af7d4712641e",
    "ell": "1c329d51eff1a96a127b7c5ebab0ea035b9be023e8ad1d557812c49c5506573e",
    "hole": "b631437d083548ec634a40e84c91d015800150a43f996d6b34a951d2f79140a1",
    "rectangle": "881d8d89ae7490bad41c6bb012e2a4acf78516a6d97e62830f4efff64e4c0808",
    "square": "05db9afe7cdbb59ca7fd4840410047c80fcd7e4e4a38fb30014517e41dbff9d8",
    "triangle": "dfdcbeed3283fab0acc04169a198b024d4cefe1776c5f03d4e9f5d21d4a6e8a3",
    "two_holes": "b5e81ff224868236ff22a164eb7ff44d0f76bd3d3a47cd12fc1baaeffe721e04",
}

#: Те же дайджесты, но с прежним ключом участника — несущей прямой. Хранятся не
#: как история: по ним проверяется, что сдвиг выше есть смена записи, а не
#: смена ответа. Удалить их значило бы принять сдвиг на слово.
DIGESTS_BEFORE_THE_PARTICIPANT_KEY_CHANGED = {
    "comb": "e2c414a29a0a86b36ce6a1834618dac82d9085434be3576cb7d48ac6de5b0c0a",
    "diamond": "b5df4dfbe4debfd0ed62423de538bbc33f030f54b717f6a299ebae7cb6a803e6",
    "ell": "b521cec89869c506e3f9cad4fe91723a07d98358ec618eb70d6718467b53fb2a",
    "hole": "ea480720c06fa04e641f10c78612c7c79d4dd17d33ae514fd3aedfedb37a7001",
    "rectangle": "c54d463de4123011014b04b5fd257bf71607e7c0fbe56e7b2f149696b702cbd8",
    "square": "7cadb8a12f8edacf162ea3f0407a102bba36a779a632716a087ad12a879613b0",
    "triangle": "f701cba697d9f8e950a099311024a7305b3789c71cd9f32da9754cbfd43e8af4",
    "two_holes": "5d474a7e6be2dacaa367eaf5839284bebcbd84aca51c09ff41046d0ab5dbf27d",
}


def _polygon(name: str) -> PolygonV1:
    outer, holes = CORPUS[name]
    return PolygonV1.build(outer, holes)


def _star(vertices: int, seed: int, radius: int = 4096) -> PolygonV1 | None:
    """Тот же звёздчатый корпус, что у очереди событий: фигуры общего положения."""

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


# --------------------------------------------------------------------------
# 1. Теорема 2.11: мотоцикл не уходит за свою трассу
# --------------------------------------------------------------------------


def _reflex_death_times(polygon: PolygonV1) -> dict[int, EventTimeV1]:
    """Когда умирает каждая ИСХОДНАЯ reflex-вершина на пути полного перебора.

    Путь берётся именно эталонный: граница проверяется против ответа, который
    получен БЕЗ неё, иначе проверка была бы кольцевой.
    """

    builder = skeleton_module._Builder(polygon, SplitSearch.EXHAUSTIVE)
    initial = {vertex.ident for vertex in builder.vertices if vertex.reflex}
    deaths: dict[int, EventTimeV1] = {}
    while len(builder.queue):
        level = builder.queue.pop_level()
        builder.now = level[0].time
        alive_before = {v.ident for v in builder.vertices if v.alive}
        builder._apply_level(level)
        if builder.refusal is not None:
            break
        builder._close_short_lavs()
        alive_after = {v.ident for v in builder.vertices if v.alive}
        for ident in sorted(alive_before - alive_after):
            if ident in initial and ident not in deaths:
                deaths[ident] = level[0].time
    return deaths


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_wavefront_reflex_vertex_never_outlives_its_own_trace(name):
    """ТЕОРЕМА 2.11 в исполняемой форме. Без этого достаточность — заявление."""

    polygon = _polygon(name)
    graph = build_motorcycle_graph(polygon)
    deaths = _reflex_death_times(polygon)
    checked = 0
    for ident, trace in graph.traces.items():
        if trace.crash_time is None or ident not in deaths:
            continue
        checked += 1
        assert trace.bounds_time(deaths[ident]), (
            f"{name}: вершина {ident} пережила крушение своего мотоцикла"
        )
    assert checked == polygon.reflex_count


@pytest.mark.parametrize("seed", range(8))
def test_the_theorem_holds_on_general_position_polygons_too(seed):
    """Корпус общего положения: не образцы, а фигуры со «злыми» радикалами."""

    polygon = _star(9, seed)
    assert polygon is not None
    graph = build_motorcycle_graph(polygon)
    deaths = _reflex_death_times(polygon)
    for ident, trace in graph.traces.items():
        if trace.crash_time is None or ident not in deaths:
            continue
        assert trace.bounds_time(deaths[ident]), f"seed {seed}, вершина {ident}"


# --------------------------------------------------------------------------
# 2. Функция объявила границу — граница проверяется на её же результате
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_trace_satisfies_the_bound_it_declares(name):
    """`bounds_time` обязана пропускать своё крушение и не пускать позже него."""

    graph = build_motorcycle_graph(_polygon(name))
    for trace in graph.traces.values():
        if trace.crash_time is None:
            assert trace.outcome is not TraceOutcome.EXACT
            # Трасса без крушения не ручается НИ ЗА ЧТО, и говорит это прямо.
            assert not trace.bounds_time(ZERO_TIME)
            continue
        assert trace.bounds_time(trace.crash_time)
        later = EventTimeV1.normalized(
            trace.crash_time.dividend * 2 + 1, trace.crash_time.divisor
        )
        assert not trace.bounds_time(later)


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_crash_point_is_exactly_where_the_position_formula_puts_it(name):
    """Точка крушения и параметризация трассы обязаны быть одним объектом."""

    graph = build_motorcycle_graph(_polygon(name))
    for trace in graph.traces.values():
        if trace.crash_point is None:
            continue
        expected = event_point(trace.left_line, trace.right_line, trace.crash_time)
        assert trace.crash_point.x == expected.x
        assert trace.crash_point.y == expected.y


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_a_wall_crash_lands_on_that_wall_and_inside_its_segment(name):
    """Мотоцикл останавливается НА стене, а не за ней. Точно, без допуска."""

    polygon = _polygon(name)
    graph = build_motorcycle_graph(polygon)
    walls = walls_of(polygon)
    seen = 0
    for trace in graph.traces.values():
        if trace.crash_kind is not CrashKind.WALL:
            continue
        seen += 1
        wall = walls[trace.crash_target]
        residual = (
            trace.crash_point.x.scaled(wall.line.a)
            + trace.crash_point.y.scaled(wall.line.b)
            - SqrtSumV1.rational(wall.line.c)
        )
        assert residual.is_zero, "точка крушения не лежит на прямой стены"
        assert motorcycle_module._projection_is_inside(trace.crash_point, wall)
    assert seen or polygon.reflex_count == 0


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_march_stays_inside_the_budget_the_module_declares(name):
    graph = build_motorcycle_graph(_polygon(name))
    if not graph.traces:
        return
    assert graph.counter("motorcycle_march_steps") <= march_budget(graph.grid) * len(
        graph.traces
    )


# --------------------------------------------------------------------------
# 3. ГЛАВНОЕ: совпадение с полным перебором, событие за событием
# --------------------------------------------------------------------------


def _split_records(skeleton) -> list:
    from cftuv_envelope.wavefront.digest import node_record

    return sorted(
        repr(node_record(node))
        for node in skeleton.nodes
        if node.kind is EventKind.SPLIT
    )


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_split_events_match_the_exhaustive_search_exactly(name):
    """Ни пропущенного, ни лишнего. Сверяются УЗЛЫ, а не их число."""

    polygon = _polygon(name)
    motorcycle = build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
    exhaustive = build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
    found, reference = _split_records(motorcycle), _split_records(exhaustive)
    assert found == reference, f"{name}: расхождение со полным перебором"
    assert semantic_digest(motorcycle) == semantic_digest(exhaustive)
    assert motorcycle.outcome is exhaustive.outcome


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_digest_has_not_moved_since_before_the_slice(name):
    """Ворота среза: цена поиска изменилась, семантика результата — нет."""

    polygon = _polygon(name)
    for search in SplitSearch:
        assert (
            semantic_digest(build_skeleton(polygon, split_search=search))
            == FROZEN_DIGESTS[name]
        ), f"{name}/{search.value}: дайджест сдвинулся"


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_digest_shift_is_the_key_encoding_and_not_the_answer(name):
    """Сдвиг храповика на восьми фигурах обязан быть сменой ЗАПИСИ, не ответа.

    Правило проекта разрешает двигать `FROZEN_DIGESTS` при условии, что новый
    ответ проверен НЕ этим числом. Здесь условие выполнено сильнее: участники
    нового скелета переводятся обратно в несущие прямые, и дайджест получается
    ПРЕЖНИЙ побитово. То есть проверено не «новый ответ тоже хорош», а «ответ
    тот же самый, записан иначе».

    Перевод односторонний по построению: у вхождения ребра прямая считается
    однозначно (`SupportLineV1.through`), у прямой вхождение — нет. Именно эта
    односторонность и была болезнью: ключ терял то, чего в нём нет.
    """

    from dataclasses import replace

    from cftuv_envelope.wavefront.event_time import SupportLineV1

    polygon = _polygon(name)
    for search in SplitSearch:
        skeleton = build_skeleton(polygon, split_search=search)
        translated = []
        for node in skeleton.nodes:
            lines = set()
            for x0, y0, x1, y1 in node.participants:
                line = SupportLineV1.through((x0, y0), (x1, y1))
                lines.add((line.a, line.b, line.c, line.q))
            translated.append(replace(node, participants=tuple(sorted(lines))))
        assert (
            semantic_digest(replace(skeleton, nodes=tuple(translated)))
            == DIGESTS_BEFORE_THE_PARTICIPANT_KEY_CHANGED[name]
        ), f"{name}/{search.value}: сдвинулся ОТВЕТ, а не запись ключа"


def test_the_two_holes_digest_shifted_last_slice_is_now_checked_independently():
    """Закрытие ЗАПИСАННОГО ОТКРЫТОГО СЧЁТА, а не новая проверка про запас.

    Прошлый срез сдвинул дайджесты трёх фигур и проверил две из них независимо
    (`ell` — замкнутой формулой митрованного отступа, `hole` — сохранением
    площади). Третью, `two_holes`, проверить было нечем: её грани отказывали
    `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES`, потому что ключом участника служила
    несущая прямая, а у рёбер двух дыр они попарно совпадают. Сдвиг был принят
    на доверии к общей причине, и это записано как открытый счёт.

    Причина отменена сменой ключа на вхождение ребра, и счёт закрывается двумя
    независимыми числами: сумма площадей граней 1408 из 1408 ТОЧНО, а покрытие
    совпадает с митрованным эталоном, посчитанным вне обоих путей.
    """

    from fractions import Fraction

    from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
    from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
    from mitered_standard import StandardOutcome, rectilinear_standard

    polygon = _polygon("two_holes")
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert partition.polygon_doubled_area == 1408
    assert partition.doubled_area.as_rational() == Fraction(1408)
    assert partition.area_defect.terms == ()
    assert len(partition.faces) == 12
    assert len({face.owner for face in partition.faces}) == 12

    measured = []
    for alpha in (Fraction(1), Fraction(2)):
        standard = rectilinear_standard(polygon, alpha)
        assert standard.outcome is StandardOutcome.EXACT, standard.detail
        coverage = coverage_at(partition, alpha)
        assert coverage.outcome is CoverageOutcome.EXACT
        area = coverage.doubled_area.as_rational() / 2
        assert area == standard.mitered_covered, alpha
        measured.append(area)
    assert measured == [Fraction(180), Fraction(368)]


def test_split_events_match_the_exhaustive_search_on_a_hundred_general_polygons():
    """Сто контуров общего положения: один и тот же ответ на обоих путях."""

    total = 0
    for vertices in (5, 7, 9, 12, 16):
        for seed in range(20):
            polygon = _star(vertices, seed)
            if polygon is None:
                continue
            total += 1
            motorcycle = build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
            exhaustive = build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
            assert motorcycle.outcome is SkeletonOutcome.EXACT
            assert _split_records(motorcycle) == _split_records(exhaustive)
            assert semantic_digest(motorcycle) == semantic_digest(exhaustive)
    assert total == 100


def test_the_search_by_trace_really_examines_fewer_candidates():
    """Отрицательный контроль: без сужения предыдущие тесты прошли бы тоже."""

    polygon = _polygon("two_holes")
    motorcycle = build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
    exhaustive = build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
    assert motorcycle.counter("split_candidates_examined") < exhaustive.counter(
        "split_candidates_examined"
    )
    assert _has_no_fallbacks(motorcycle)


def _has_no_fallbacks(skeleton) -> bool:
    return (
        skeleton.counter("split_search_exhaustive_vertices") == 0
        and skeleton.counter("split_search_exhaustive_segments") == 0
    )


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


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_graph_itself_does_not_depend_on_the_input_order(name):
    """Крушения — геометрия, а не порядок обхода контура."""

    outer, holes = CORPUS[name]
    def fingerprint(polygon):
        graph = build_motorcycle_graph(polygon)
        return sorted(
            (
                trace.crash_kind.value,
                repr(trace.crash_point.x.terms if trace.crash_point else None),
                repr(trace.crash_point.y.terms if trace.crash_point else None),
            )
            for trace in graph.traces.values()
        )

    reference = fingerprint(PolygonV1.build(outer, holes))
    for shift in range(len(outer)):
        rotated = LoopV1(tuple(outer)).rotated(shift).points
        assert fingerprint(PolygonV1.build(rotated, holes)) == reference


# --------------------------------------------------------------------------
# 5. Дыра: внутренний фронт и его мотоциклы
# --------------------------------------------------------------------------


def test_every_corner_of_a_hole_emits_its_own_motorcycle():
    """Дыра не особый случай: её углы reflex по отношению к материалу.

    У квадратной дыры четыре угла, каждый выпускает мотоцикл наружу, и каждый
    разбивается о свою сторону внешнего контура. Проверяется, что трассы
    ПОСТРОЕНЫ и что крушения попали на разные стены, а не в одну точку.
    """

    polygon = _polygon("hole")
    graph = build_motorcycle_graph(polygon)
    assert len(graph.traces) == 4
    assert graph.counter("motorcycle_wall_crashes") == 4
    corners = {
        (
            trace.crash_point.x.as_rational(),
            trace.crash_point.y.as_rational(),
        )
        for trace in graph.traces.values()
    }
    assert corners == {
        (Fraction(0), Fraction(0)),
        (Fraction(0), Fraction(20)),
        (Fraction(20), Fraction(0)),
        (Fraction(20), Fraction(20)),
    }


def test_two_holes_aiming_at_one_edge_are_still_cut_at_once():
    polygon = _polygon("two_holes")
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    # Считать надо ОБА разбора: часть кандидатов приходит не в ребро, а в
    # вершину фронта, и разбирается встречей вершин, а не разрезом. Сумма та же.
    assert (
        skeleton.counter("split_events")
        + skeleton.counter("vertex_meeting_events") * 2
    ) >= 8


# --------------------------------------------------------------------------
# 6. Мотоциклы гасят друг друга
# --------------------------------------------------------------------------


def test_motorcycles_extinguish_each_other_and_leave_a_resting_steiner_vertex():
    """Фикстура с крушением о трассу, а не о стену. Это и есть Steiner-вершина.

    Ищется она не подбором: у `star_9_seed_4` три reflex-вершины, их трассы
    сходятся, и крушений о трассу ровно два. Без этой фикстуры граф был бы
    просто стрельбой лучами по стенам, а «мотоциклы гасят друг друга» —
    непроверенным словом.
    """

    polygon = _star(9, 4)
    assert polygon is not None
    graph = build_motorcycle_graph(polygon)
    assert graph.counter("motorcycle_trace_crashes") == 2
    assert graph.counter("resting_steiner_vertices") == 2
    crashed = [
        trace
        for trace in graph.traces.values()
        if trace.crash_kind in (CrashKind.TRACE, CrashKind.SIMULTANEOUS)
    ]
    assert len(crashed) == 2
    for trace in crashed:
        host = graph.traces[trace.crash_target]
        normal_x, normal_y = host.velocity[1], -host.velocity[0]
        residual = (
            normal_x * (trace.crash_point.x - host.origin.x)
            + normal_y * (trace.crash_point.y - host.origin.y)
        )
        assert residual.is_zero, "погашен не на трассе соседа"


def test_a_trace_crash_only_ever_shortens_the_wall_bound():
    """Уточнение обязано быть уточнением: длиннее стенной границы оно не станет."""

    polygon = _star(9, 4)
    graph = build_motorcycle_graph(polygon)
    walls_only = motorcycle_module.MotorcycleGraphV1(
        walls=graph.walls,
        grid=graph.grid,
        wall_index=graph.wall_index,
        traces={},
        counters=dict(graph.counters),
    )
    for ident, trace in graph.traces.items():
        wall_trace = walls_only._march(
            ident,
            trace.left_line,
            trace.right_line,
            trace.start_time,
            trace.origin,
        )
        assert wall_trace.crash_time is not None
        assert compare_times(trace.crash_time, wall_trace.crash_time) <= 0


# --------------------------------------------------------------------------
# 7. Именованный отказ там, где ответа нет
# --------------------------------------------------------------------------


def test_a_motorcycle_without_a_bisector_is_a_named_refusal():
    """Две параллельные прямые биссектрисы не задают. Придумывать её нельзя."""

    graph = build_motorcycle_graph(_polygon("ell"))
    parallel = SupportLineV1(0, 4, 0, 16)
    trace = graph.trace_for(parallel, SupportLineV1(0, 4, 8, 16), ZERO_TIME, graph.traces[3].origin)
    assert trace.outcome is TraceOutcome.MOTORCYCLE_HAS_NO_BISECTOR
    assert trace.crash_time is None
    assert not trace.bounds_time(ZERO_TIME)


def test_a_motorcycle_that_never_meets_a_wall_is_a_named_refusal():
    """Старт позже всех крушений: луч уходит за область, и это исход с именем."""

    graph = build_motorcycle_graph(_polygon("ell"))
    trace = graph.traces[3]
    late = EventTimeV1.normalized(1000, SqrtSumV1.rational(1))
    beyond = graph.trace_for(
        trace.left_line, trace.right_line, late, trace.origin
    )
    assert beyond.outcome is TraceOutcome.MOTORCYCLE_NEVER_MEETS_A_WALL
    assert beyond.crash_time is None


def test_exhausting_the_march_budget_is_a_named_outcome_not_a_silent_return():
    """У Трики это `bestt == 1e100 -> return`. Здесь — исход с именем.

    Бюджет подменяется на заведомо недостижимый: честный вход находит стену за
    один-два шага, и с бюджетом 1 тест доказывал бы наличие дефекта, а не
    наличие имени — проверено, он проходил зелёным и ничего не удостоверял.
    """

    original = motorcycle_module.march_budget
    motorcycle_module.march_budget = lambda grid: 0
    try:
        graph = build_motorcycle_graph(_polygon("comb"))
    finally:
        motorcycle_module.march_budget = original
    assert graph.counter("motorcycle_unbounded_traces") == 2
    for trace in graph.traces.values():
        assert trace.outcome is TraceOutcome.MOTORCYCLE_MARCH_BUDGET_EXHAUSTED
        assert trace.crash_time is None


# --------------------------------------------------------------------------
# 8. Отступление на полный перебор обязано давать тот же ответ
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_when_the_index_refuses_a_vertex_the_answer_does_not_change(name):
    """Отступление — не аварийный путь, а часть контракта фильтра.

    Индекс заставляют отказать по каждой вершине. Тогда весь поиск идёт
    перебором, и дайджест обязан остаться тем же: иначе «фильтр либо доказывает,
    либо уступает точному пути» было бы неправдой.
    """

    original = TraceCandidateIndexV1.register_trace
    TraceCandidateIndexV1.register_trace = lambda self, vertex, trace: False
    try:
        skeleton = build_skeleton(_polygon(name))
    finally:
        TraceCandidateIndexV1.register_trace = original
    assert semantic_digest(skeleton) == FROZEN_DIGESTS[name]
    polygon = _polygon(name)
    if polygon.reflex_count:
        assert skeleton.counter("split_search_exhaustive_vertices") > 0


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_when_the_index_refuses_a_line_the_answer_does_not_change(name):
    original = TraceCandidateIndexV1.knows_line
    TraceCandidateIndexV1.knows_line = lambda self, key: False
    try:
        skeleton = build_skeleton(_polygon(name))
    finally:
        TraceCandidateIndexV1.knows_line = original
    assert semantic_digest(skeleton) == FROZEN_DIGESTS[name]


# --------------------------------------------------------------------------
# 9. Индекс симметричен, иначе перепорождение теряет события
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_two_directions_of_the_index_agree_pair_for_pair(name):
    """«Прямые вокруг вершины» и «вершины вокруг прямой» — одно множество пар."""

    polygon = _polygon(name)
    graph = build_motorcycle_graph(polygon)
    index = TraceCandidateIndexV1.covering(polygon, graph)
    for ident, wall in enumerate(walls_of(polygon)):
        index.register_line(ident, wall.line)
    known = [
        ident
        for ident, trace in graph.traces.items()
        if index.register_trace(ident, trace)
    ]
    forward = {
        (vertex, line)
        for vertex in known
        for line in index.lines_near(vertex)
    }
    backward = {
        (vertex, line)
        for line in range(len(walls_of(polygon)))
        for vertex in index.vertices_near(line)
        if vertex in set(known)
    }
    assert forward == backward


# --------------------------------------------------------------------------
# 10. Цена знака: интервальный фильтр обязан закрывать всё
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_graph_never_falls_through_to_the_expensive_sign_solver(name):
    """Выигрыш в кандидатах не имеет права съесться ценой знака.

    В базе среза дорогой решатель не входил ни разу. Motorcycle graph порождает
    выражения другой формы — произведения `SqrtSum`, а не только суммы, — и это
    ровно то место, где интервальный фильтр мог бы начать уступать. Проверяется
    числом, а не предположением.
    """

    reset_sign_counts()
    build_skeleton(_polygon(name))
    assert SIGN_COUNTS["closed_by_conjugation"] == 0
    assert SIGN_COUNTS["total"] > 0


def test_the_expensive_sign_solver_stays_out_on_general_position_inputs():
    reset_sign_counts()
    for seed in range(8):
        polygon = _star(9, seed)
        if polygon is not None:
            build_skeleton(polygon)
    assert SIGN_COUNTS["closed_by_conjugation"] == 0


# --------------------------------------------------------------------------
# 11. START и SWITCH: шов назван точнее, чем был
# --------------------------------------------------------------------------


def test_start_and_switch_are_a_named_seam_and_the_reason_is_now_precise():
    """Граф построен, значит прежняя причина отменена. Новая названа.

    `START` и `SWITCH` рождаются не из графа, а из РАСШИРЕННОГО фронта: чтобы
    они появились, Steiner-вершины графа обязаны стать вершинами самого фронта,
    а фронт — вестись по ячейкам арранжемента трасс. Граф Steiner-вершины даёт и
    считает; фронт по ячейкам не ведётся. Ноль с названной причиной — измерение.
    """

    assert {kind.value for kind in EventKind} == {
        "EDGE",
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
    for name in sorted(CORPUS):
        skeleton = build_skeleton(_polygon(name))
        assert skeleton.counter("start_events") == 0
        assert skeleton.counter("switch_events") == 0
    # А Steiner-вершины при этом СУЩЕСТВУЮТ, и их число не ноль на фикстуре,
    # где мотоциклы гасят друг друга. Иначе шов был бы отговоркой.
    graph = build_motorcycle_graph(_star(9, 4))
    assert graph.counter("resting_steiner_vertices") == 2
