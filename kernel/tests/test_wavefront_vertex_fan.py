"""Мягкий вогнутый угол очереди против НЕЗАВИСИМОГО эталона цепи.

Митрованный стенд (`test_wavefront_mitered_standard.py`) проверял ОДНУ точку
семейства профилей — острый апекс. Здесь проверяется вторая, продуктовая:
`LINEAR_REFLEX_EQUAL_V1` при `k = 1`, то есть двухзвенная цепь. Эталон
(`chamfered_standard.py`) выведен руками, живёт вне `wavefront/` и не знает ни
одной строки очереди.

| что проверяется                                            | тест |
|------------------------------------------------------------|------|
| веер входит в фронт и скелет с ним сходится                  | `..._queue_digests_the_zero_length_start` |
| без нового правила ребро веера гаснет на нулевом уровне      | `..._without_the_new_rule_the_fan_dies_at_zero` |
| покрытие с веером = эталон цепи ТОЧНО                        | `..._queue_with_a_fan_equals_the_chain_standard` |
| цепь лежит СТРОГО между полосами и митром                    | `..._chain_is_strictly_between_the_strips_and_the_miter` |
| хорда — другая модель, и расхождение названо числом          | `..._bevel_is_a_different_model_and_the_gap_is_a_number` |
| пустой веер тождественно равен отсутствию веера              | `..._an_empty_fan_is_the_miter_bit_for_bit` |
| веер вне вогнутого сектора — именованный отказ               | `..._a_support_outside_the_reflex_sector_is_refused` |
| веер не в вершине — именованный отказ                        | `..._a_fan_off_the_contour_is_refused` |
| `k` опор одной вершины дают `k` РАЗНЫХ граней                | `..._two_supports_of_one_vertex_get_two_faces` |

Ни одна строка сверки не пропущена: `ell`, `staircase`, `u_shape` при каждой
alpha, для которой независимость квадратов доказана самим митрованным эталоном.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces, fan_edge_key
from cftuv_envelope.wavefront.polygon import (
    FanSupportV1,
    PolygonOutcome,
    PolygonRejected,
    PolygonV1,
    VertexFanV1,
    with_vertex_fans,
)
from cftuv_envelope.wavefront.skeleton import (
    CandidateRefusal,
    SkeletonOutcome,
    build_skeleton,
    refusal_counter,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from chamfered_standard import (
    ChamferOutcome,
    chamfered_standard,
    right_angle_fans,
    with_right_angle_fans,
)
from wavefront_cases import ell, named_corpus, staircase, u_shape


# Фигура и те alpha, при которых митрованный эталон САМ доказал независимость
# квадратов вогнутых вершин. Список не подобран: при большей alpha квадраты
# пересекаются, эталон цепи отвечает именованным отказом, и это проверяется
# отдельным тестом, а не обходится молчанием.
FAN_CASES = (
    ("ell", ell(12), (Fraction(1), Fraction(2), Fraction(3))),
    ("staircase", staircase(), (Fraction(1), Fraction(3, 2))),
    ("u_shape", u_shape(), (Fraction(1), Fraction(2))),
)


def _fanned_partition(polygon: PolygonV1):
    fanned = with_right_angle_fans(polygon)
    skeleton = build_skeleton(fanned)
    assert skeleton.outcome is SkeletonOutcome.EXACT, skeleton.outcome
    partition = build_faces(fanned, skeleton)
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    return fanned, skeleton, partition


@pytest.mark.parametrize(
    "name, polygon", [(name, figure) for name, figure, _ in FAN_CASES]
)
def test_the_queue_digests_the_zero_length_start(name, polygon):
    """Гипотеза карточки, измеренная: старт из нулевой длины проходит цикл.

    Проходит он ровно с одной поправкой, и она видна счётчиком: на каждую
    скрытую опору ровно один отказ `FILTER_SPAN_IS_BORN_ZERO`. Остальные правила
    события — встречи вершин, скользящие вершины, взвешенные скорости — веер
    приняли без единой правки, и это утверждение проверяется здесь числом, а не
    отсутствием исключения: скелет обязан выдать `EXACT`, а разбиение —
    воспроизвести площадь фигуры.
    """

    fanned, skeleton, partition = _fanned_partition(polygon)
    assert fanned.fan_edge_count == polygon.reflex_count
    assert skeleton.counter(
        refusal_counter(CandidateRefusal.FILTER_SPAN_IS_BORN_ZERO)
    ) == fanned.fan_edge_count
    assert partition.area_reproduces_polygon
    assert partition.every_face_is_positive
    assert partition.every_contour_is_simple
    # Грань у каждой скрытой опоры СВОЯ и не пустая: веер, чья грань выродилась
    # бы в точку, был бы митром под другим именем.
    fan_faces = [face for face in partition.faces if face.is_fan_support]
    assert len(fan_faces) == fanned.fan_edge_count
    assert all(face.doubled_area.sign() > 0 for face in fan_faces)


def test_without_the_new_rule_the_fan_dies_at_the_zero_level():
    """Отрицательный контроль: правило `FILTER_SPAN_IS_BORN_ZERO` несущее.

    Три прямые веера проходят через саму вершину, поэтому `concurrency_time`
    честно отвечает `t = 0`. Без фильтра ребро веера гасится на нулевом уровне,
    его грань выходит нулевой площади, и сборщик отвечает `FACE_IS_NOT_POSITIVE`.
    Проверяется это ПОДМЕНОЙ правила, а не рассказом: если фильтр когда-нибудь
    станет лишним, тест это заметит.
    """

    from cftuv_envelope.wavefront import skeleton as module

    fanned = with_right_angle_fans(ell(12))
    original = module._Builder._enqueue_edge_event

    def without_the_rule(self, vertex):
        counter = refusal_counter(CandidateRefusal.FILTER_SPAN_IS_BORN_ZERO)
        before = self.counters[counter]
        result = original(self, vertex)
        if self.counters[counter] > before:
            self.counters[counter] = before
            peer = self.vertices[vertex.next]
            time, outcome = self._edge_event_time(vertex, peer)
            point = self._vertex_position(vertex, time)
            if point is not None:
                self.queue.push(
                    module.CandidateEventV1(
                        module.EventKind.EDGE,
                        time,
                        point,
                        vertex.ident,
                        peer.ident,
                        -1,
                    )
                )
        return result

    module._Builder._enqueue_edge_event = without_the_rule
    try:
        broken = build_skeleton(fanned)
        partition = build_faces(fanned, broken)
    finally:
        module._Builder._enqueue_edge_event = original
    assert partition.outcome is FaceOutcome.FACE_IS_NOT_POSITIVE
    assert "0" in partition.detail


@pytest.mark.parametrize(
    "name, polygon, alphas",
    FAN_CASES,
    ids=[name for name, _, _ in FAN_CASES],
)
def test_the_queue_with_a_fan_equals_the_chain_standard_exactly(
    name, polygon, alphas
):
    """Покрытие очереди с веером совпадает с замкнутой формой цепи ТОЧНО.

    Точно — это `is_zero` разности двух `SqrtSumV1`, а не «совпало до знака».
    Величины иррациональны обе (`sqrt(2)` в недоборе угла), поэтому сравнение
    рациональным быть и не могло.
    """

    _, _, partition = _fanned_partition(polygon)
    for alpha in alphas:
        standard = chamfered_standard(polygon, alpha)
        assert standard.outcome is ChamferOutcome.EXACT, standard.detail
        covered = coverage_at(partition, alpha)
        assert covered.outcome is CoverageOutcome.EXACT
        assert (covered.doubled_area - standard.doubled_chained).is_zero, (
            name,
            alpha,
            covered.doubled_area.terms,
            standard.doubled_chained.terms,
        )


def test_the_frozen_numbers_of_the_chain_are_written_down():
    """Замороженные числа среза. Знаменатели перемеряны прогоном, не выведены.

    | фигура      | alpha | покрытие цепи (2S)   | митр (2S) | полосы (2S) |
    |-------------|------:|----------------------|----------:|------------:|
    | `ell`       |     1 | `82 + 4*sqrt(2)`     |        88 |          86 |
    | `ell`       |     2 | `136 + 16*sqrt(2)`   |       160 |         152 |
    | `ell`       |     3 | `162 + 36*sqrt(2)`   |       216 |         198 |
    | `staircase` |     1 | `76 + 8*sqrt(2)`     |        88 |          84 |
    | `staircase` |   3/2 | `99 + 18*sqrt(2)`    |       126 |         117 |
    | `u_shape`   |     1 | `108 + 8*sqrt(2)`    |       120 |         116 |
    | `u_shape`   |     2 | `176 + 32*sqrt(2)`   |       224 |         208 |

    Недобор цепи против митра равен `r*alpha^2*(6 - 4*sqrt(2))` на каждой строке,
    и это тот же вывод, что в докстроке эталона, — но посчитанный очередью.

    Числа ПЕРЕМЕРЯНЫ прогоном, а не выписаны из формулы: первая редакция таблицы
    держала у `u_shape` при alpha = 2 значение `192 + 32*sqrt(2)`, посчитанное
    в уме, и прогон ответил `176 + 32*sqrt(2)` — рациональная часть разошлась на
    16, потому что при alpha = 2 у `u_shape` митрованное покрытие уже 224, а не
    предполагавшиеся 208.
    """

    frozen = {
        ("ell", Fraction(1)): ((1, Fraction(82)), (2, Fraction(4))),
        ("ell", Fraction(2)): ((1, Fraction(136)), (2, Fraction(16))),
        ("ell", Fraction(3)): ((1, Fraction(162)), (2, Fraction(36))),
        ("staircase", Fraction(1)): ((1, Fraction(76)), (2, Fraction(8))),
        ("staircase", Fraction(3, 2)): ((1, Fraction(99)), (2, Fraction(18))),
        ("u_shape", Fraction(1)): ((1, Fraction(108)), (2, Fraction(8))),
        ("u_shape", Fraction(2)): ((1, Fraction(176)), (2, Fraction(32))),
    }
    figures = {name: figure for name, figure, _ in FAN_CASES}
    for (name, alpha), terms in frozen.items():
        _, _, partition = _fanned_partition(figures[name])
        covered = coverage_at(partition, alpha)
        assert covered.doubled_area.terms == terms, (name, alpha)


@pytest.mark.parametrize(
    "name, polygon, alphas",
    FAN_CASES,
    ids=[name for name, _, _ in FAN_CASES],
)
def test_the_chain_is_strictly_between_the_strips_and_the_miter(
    name, polygon, alphas
):
    """Цепь накрывает БОЛЬШЕ полос и МЕНЬШЕ митра, и оба неравенства строгие.

    Это и есть определение точки семейства: не совпасть ни с одним из двух
    прежних ответов. Совпадение с любым из них означало бы, что веер молча
    выродился, а `alpha^2` перестало быть верхней границей семейства.
    """

    for alpha in alphas:
        standard = chamfered_standard(polygon, alpha)
        assert standard.outcome is ChamferOutcome.EXACT
        assert (standard.doubled_mitered - standard.doubled_chained).sign() > 0
        assert (standard.doubled_chained - standard.doubled_strips).sign() > 0


def test_the_bevel_is_a_different_model_and_the_gap_is_a_number():
    """Хорда (BEVEL) — НЕ цепь, и расхождение записано числом.

    Оно почти вдвое больше самого недобора цепи (`0.3284` против `0.1716` на
    `alpha^2`), поэтому перепутать две модели молча нельзя: очередь, случайно
    посчитавшая хорду, разошлась бы с эталоном на величину, которую видно.
    """

    standard = chamfered_standard(ell(12), Fraction(1))
    assert standard.doubled_chain_over_bevel.terms == (
        (1, Fraction(-5)),
        (2, Fraction(4)),
    )
    assert standard.doubled_chain_over_bevel.sign() > 0


def test_an_empty_fan_is_the_miter_bit_for_bit():
    """Веер из нуля опор тождественно равен его отсутствию.

    Проверяется побитово — дайджестом скелета, а не площадью: `k = 0` есть
    вырожденный член того же семейства, и если вход с пустым веером идёт другим
    путём, то «семейство» распалось бы на два построения.
    """

    from cftuv_envelope.wavefront.digest import semantic_digest

    figure = ell(12)
    empty = with_vertex_fans(figure, (VertexFanV1((6, 6), ()),))
    assert empty.fan_edge_count == 0
    assert semantic_digest(build_skeleton(empty)) == semantic_digest(
        build_skeleton(figure)
    )


def test_a_support_outside_the_reflex_sector_is_refused_by_name():
    """Опора, повёрнутая не туда, — именованный отказ, а не пересортировка."""

    figure = ell(12)
    with pytest.raises(PolygonRejected) as refusal:
        with_vertex_fans(
            figure, (VertexFanV1((6, 6), (FanSupportV1(1, 1, 2),)),)
        )
    assert (
        refusal.value.outcome
        is PolygonOutcome.VERTEX_FAN_SUPPORT_IS_NOT_INSIDE_THE_REFLEX_SECTOR
    )


def test_a_fan_at_a_convex_vertex_is_refused_by_name():
    """Веер в ВЫПУКЛОЙ вершине не бывает: сектор там не вогнутый.

    Отказ идёт тем же членом, что и опора не в секторе, и это не небрежность:
    у выпуклой вершины поворот от входящей нормали к исходящей уже
    положительный, поэтому ЛЮБАЯ опора окажется вне вогнутого сектора. Причина
    одна, значит и имя одно.
    """

    with pytest.raises(PolygonRejected) as refusal:
        with_vertex_fans(
            ell(12), (VertexFanV1((0, 0), (FanSupportV1(1, 1, 2),)),)
        )
    assert (
        refusal.value.outcome
        is PolygonOutcome.VERTEX_FAN_SUPPORT_IS_NOT_INSIDE_THE_REFLEX_SECTOR
    )


def test_a_fan_off_the_contour_is_refused_by_name():
    """Веер в точке, которая вершиной не является, — потеря, и она названа."""

    with pytest.raises(PolygonRejected) as refusal:
        with_vertex_fans(
            ell(12), (VertexFanV1((7, 7), (FanSupportV1(-1, -1, 2),)),)
        )
    assert (
        refusal.value.outcome
        is PolygonOutcome.VERTEX_FAN_POINT_IS_NOT_A_UNIQUE_VERTEX
    )


def test_a_stationary_fan_support_is_refused_by_name():
    """`q = 0` у скрытой опоры — это митр под именем веера, и он отвергнут."""

    with pytest.raises(PolygonRejected) as refusal:
        VertexFanV1((6, 6), (FanSupportV1(-1, -1, 0),))
    assert (
        refusal.value.outcome
        is PolygonOutcome.VERTEX_FAN_SUPPORT_IS_NOT_A_MOVING_LINE
    )


def test_two_supports_of_one_vertex_get_two_distinct_faces():
    """`k = 2` в одной вершине даёт ДВЕ грани, а не одну на двоих.

    Ключ участника у скрытой опоры несёт ординал ровно ради этого. Вход здесь
    k-агностичен намеренно: плотность сегментов объявлена будущим
    пользовательским параметром, и машинерия, зашившая `k <= 2`, переписывалась
    бы вместе с ним. Направления взяты РАЦИОНАЛЬНЫЕ (`(-1,-2)` и `(-2,-1)`) —
    они не равноугольны, то есть профилем `LINEAR_REFLEX_EQUAL_V1` не являются;
    здесь проверяется машинерия, а не рецепт направлений.
    """

    figure = ell(12)
    fanned = with_vertex_fans(
        figure,
        (
            VertexFanV1(
                (6, 6),
                (FanSupportV1(-1, -2, 5), FanSupportV1(-2, -1, 5)),
            ),
        ),
    )
    assert fanned.fan_edge_count == 2
    skeleton = build_skeleton(fanned)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    partition = build_faces(fanned, skeleton)
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    owners = {face.owner for face in partition.faces if face.is_fan_support}
    assert owners == {fan_edge_key((6, 6), 1), fan_edge_key((6, 6), 2)}
    assert partition.area_reproduces_polygon


def test_the_chain_standard_refuses_when_the_reflex_squares_meet():
    """Эталон отказывает ИМЕНЕМ там, где квадраты вершин перестали быть независимы.

    У `staircase` при alpha = 2 квадраты соседних вогнутых вершин пересекаются,
    и «вычесть T на вершину» перестаёт быть верным. Эталон это не подправляет и
    не молчит — он отвечает `REFLEX_SQUARES_ARE_NOT_INDEPENDENT` и кладёт в
    `detail` оба числа.
    """

    standard = chamfered_standard(staircase(), Fraction(3))
    assert standard.outcome is ChamferOutcome.REFLEX_SQUARES_ARE_NOT_INDEPENDENT
    assert "против бюджета" in standard.detail


def test_the_corpus_without_fans_never_fires_the_new_filter():
    """Новое правило не трогает ни один прежний вход. Ноль ЗАМОРОЖЕН, а не подразумевается.

    Без этого счёта «фильтр ничего не сломал» держалось бы на зелёном прогоне,
    то есть на отсутствии наблюдения. Здесь наблюдение есть: конфигурация
    «отрезок нулевой в момент рождения обоих концов» на корпусе без вееров не
    встречается ни разу.
    """

    counter = refusal_counter(CandidateRefusal.FILTER_SPAN_IS_BORN_ZERO)
    for name, figure in named_corpus():
        assert build_skeleton(figure).counter(counter) == 0, name
