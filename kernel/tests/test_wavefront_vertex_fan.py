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
| очередь строит веер ТЕМ ЖЕ рецептом, что эталон              | `..._conveyor_builds_the_fan_with_the_reference_recipe` |
| мост проводит веер своим каналом и называет владельца        | `..._bridge_carries_the_fan_through_its_own_channel` |
| нерациональный веер больше не валит домен, исход УДАЛЁН       | `..._an_irrational_fan_no_longer_refuses_the_domain_and_the_outcome_is_gone` |
| деградация всех вееров = митрованный домен ПОБИТОВО           | `..._every_fan_degrading_leaves_the_field_domain_mitered_bit_for_bit` |
| полевые фикстуры объявляют углы: 4, 4 и 0; вееров 2, 4 и 0    | `..._the_field_fixtures_declare_their_corner_relations_by_name` |
| полное выделение: домен `EXACT`, 4 веера, 0 деградаций        | `..._the_full_selection_domain_builds_with_four_rational_fans` |
| chart-lattice binding делает canonical разность точным нулём | `..._bound_full_selection_freezes_the_exact_queue_reference_sliver` |
| зеркальная карта даёт ОТРАЖЁННЫЕ скелет и покрытие            | `..._the_mirrored_chart_gives_the_reflected_skeleton_and_coverage` |
| перевёрнутый руками порядок на прямой карте — отказ по имени   | `..._the_inverted_support_order_on_a_direct_chart_is_still_refused` |
| склеенные привязкой якоря — отказ решётки, ветка достижима     | `..._the_lattice_keeps_the_refusal_it_actually_causes` |

Ни одна строка сверки не пропущена: `ell`, `staircase`, `u_shape` при каждой
alpha, для которой независимость квадратов доказана самим митрованным эталоном.
"""

from __future__ import annotations

from fractions import Fraction

import pytest
import sympy as sp

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


def test_the_conveyor_builds_the_fan_with_the_reference_recipe():
    """Веер очереди строится РЕЦЕПТОМ ЭТАЛОНА, и его числа заморожены.

    Проверяется на единственном угловом входе, который в репозитории есть, —
    `angular_snapshot(k)`. Направления берутся у `_interpolated_normals` через
    `angular_hidden_support_lines`, второй реализации поворота нет, и это видно
    по числам: у `k = 2` вторая опора получается УДВОЕНИЕМ первой ровно так, как
    её строит эталон, а не независимым счётом.

    | `k` | митрованных углов | вееров | опоры `(a, b, q)` |
    |----:|------------------:|-------:|-------------------|
    | 0   | 1                 | 0      | — (митр — законный член семейства) |
    | 1   | 0                 | 1      | `(3, -4, 25)` |
    | 2   | 0                 | 1      | `(3, -4, 25)`, `(-7, -24, 625)` |

    ГИПОТЕЗА КАРТОЧКИ «`k = 2` даёт кубику тройного угла, значит направление
    алгебраично» на этом входе ОПРОВЕРГНУТА: фикстура стоит на утроенном
    пифагоровом угле (`cos 3t = -117/125` при `cos t = 3/5`), и трисекция выходит
    рациональной. Отсюда и вход веера оставлен k-агностичным: ограничение живёт в
    рецепте направлений, а не в машинерии.

    `q = a^2 + b^2` у обеих опор — единичная евклидова скорость, то самое
    `all_support_normal_speed = 1` профиля, прочитанное из записи, а не
    вписанное.
    """

    from cftuv_envelope.reference.common import GeometryContext
    from cftuv_envelope.reference.compile import compile_reference_envelopes
    from cftuv_envelope.reference.validation import (
        validate_reference_geometry_payload,
    )
    from cftuv_envelope.wavefront.conveyor import _angular_fans

    from reference_factories import angular_snapshot

    expected = {
        0: ((), 1),
        1: (((3, -4, Fraction(25)),), 0),
        2: (((3, -4, Fraction(25)), (-7, -24, Fraction(625))), 0),
    }
    for hidden_count, (supports, mitered) in expected.items():
        snapshot, request = angular_snapshot(hidden_count)
        compilation = compile_reference_envelopes(snapshot, request).compilation
        frame, _ = validate_reference_geometry_payload(
            compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
        )
        fans = _angular_fans(GeometryContext.build(compilation, frame))
        assert fans.degraded_corners == (), hidden_count
        assert fans.mitered_corner_count == mitered, hidden_count
        assert len(fans.fans) == (0 if mitered else 1), hidden_count
        if supports:
            assert fans.fans[0].supports == supports, hidden_count
            assert fans.fans[0].point == (Fraction(0), Fraction(0))
        # Единичная скорость проверяется РАВЕНСТВОМ, а не «похоже на единицу».
        for a, b, speed in supports:
            assert speed == a * a + b * b


def test_the_bridge_carries_the_fan_through_its_own_channel():
    """Мост проводит веер СВОИМ каналом: не по рёбрам, а по узлу вершины.

    Прямая скрытой опоры проходит через саму вершину и ребром домена не является
    ни при какой привязке, поэтому в сопоставлении по классам прямых она
    получила бы `ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE` — верный ответ на неверно
    заданный вопрос. Здесь проверяется, что канал у неё отдельный: полигон
    получает ребро нулевой длины, а владелец грани называется именем спеки.
    """

    from cftuv_envelope.wavefront.bridge import (
        BridgeOutcome,
        VertexFanLawV1,
        bridge_arrival_laws,
        unit_speed_laws_of,
    )

    figure = ell(12)
    loops = tuple(
        tuple((Fraction(x), Fraction(y)) for x, y in loop.points)
        for loop in figure.loops
    )
    report = bridge_arrival_laws(
        loops,
        unit_speed_laws_of(figure),
        vertex_fans=(
            VertexFanLawV1(
                "angular-spec:probe",
                (Fraction(6), Fraction(6)),
                ((-1, -1, Fraction(2)),),
            ),
        ),
    )
    assert report.outcome is BridgeOutcome.EXACT, report.findings
    assert report.fan_edge_count == 1
    assert dict(report.owner_by_edge)[fan_edge_key((6, 6), 1)] == (
        "angular-spec:probe"
    )
    partition = build_faces(
        report.polygon, build_skeleton(report.polygon)
    )
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    covered = coverage_at(partition, Fraction(1))
    assert covered.doubled_area.terms == ((1, Fraction(82)), (2, Fraction(4)))


def test_a_fan_anchor_off_the_lattice_domain_is_refused_by_name():
    """Якорь, не севший в узел домена, — именованный отказ моста.

    Сопоставить веер «ближайшей» вершине значило бы придумать вершину, а веер,
    выброшенный молча, превратил бы мягкий угол в митрованный без следа.
    """

    from cftuv_envelope.wavefront.bridge import (
        BridgeOutcome,
        VertexFanLawV1,
        bridge_arrival_laws,
        unit_speed_laws_of,
    )

    figure = ell(12)
    loops = tuple(
        tuple((Fraction(x), Fraction(y)) for x, y in loop.points)
        for loop in figure.loops
    )
    report = bridge_arrival_laws(
        loops,
        unit_speed_laws_of(figure),
        vertex_fans=(
            VertexFanLawV1(
                "angular-spec:probe",
                (Fraction(7), Fraction(7)),
                ((-1, -1, Fraction(2)),),
            ),
        ),
    )
    assert (
        report.outcome is BridgeOutcome.VERTEX_FAN_ANCHOR_IS_NOT_A_LATTICE_VERTEX
    )
    assert report.polygon is None


# Веер `k = 2` В ПОРЯДКЕ ОБХОДА ПАТЧА-ВЛАДЕЛЬЦА, взятый у
# `test_two_supports_of_one_vertex_get_two_distinct_faces`: там же он и
# проверен машинерией граней. Направления рациональны и не равноугольны —
# профилем `LINEAR_REFLEX_EQUAL_V1` они не являются, и здесь это не нужно:
# предмет теста — ПОРЯДОК, а не рецепт.
MIRROR_ANCHOR = (6, 6)
MIRROR_OWNER_SUPPORTS = ((-1, -2, Fraction(5)), (-2, -1, Fraction(5)))


def _reflected(point):
    """Отражение координат карты `(x, y) -> (y, x)`: определитель равен -1."""

    return (point[1], point[0])


def _mirror_pair_column(mirrored: bool):
    """Одна колонка синтетической пары «прямая карта ↔ её зеркало».

    Отражается ВСЁ, что живёт в координатах карты: петля домена, якорь угла и
    ковекторы нормалей опор (`a*x + b*y` при `(x, y) -> (y, x)` переходит в
    `b*x' + a*y'`). ПОРЯДОК опор не отражается — он приходит из обхода
    патча-владельца, а патч отражение карты не трогает. Ровно поэтому колонки
    различаются только знаком `owner_orientation_sign`, который и есть вся
    власть закона.
    """

    from cftuv_envelope.wavefront.bridge import (
        VertexFanLawV1,
        bridge_arrival_laws,
        unit_speed_laws_of,
    )
    from cftuv_envelope.wavefront.conveyor import computation_loop_supports

    points = tuple(loop.points for loop in ell(12).loops)
    supports = MIRROR_OWNER_SUPPORTS
    anchor = MIRROR_ANCHOR
    if mirrored:
        points = tuple(tuple(_reflected(item) for item in loop) for loop in points)
        supports = tuple((b, a, q) for a, b, q in supports)
        anchor = _reflected(MIRROR_ANCHOR)
    figure = PolygonV1.build(points[0], points[1:])
    report = bridge_arrival_laws(
        tuple(
            tuple((Fraction(x), Fraction(y)) for x, y in loop.points)
            for loop in figure.loops
        ),
        unit_speed_laws_of(figure),
        vertex_fans=(
            VertexFanLawV1(
                "angular-spec:mirror",
                (Fraction(anchor[0]), Fraction(anchor[1])),
                computation_loop_supports(supports, -1 if mirrored else 1),
            ),
        ),
    )
    return figure, report


def _mirror_pair_reading(mirrored: bool, alpha: Fraction):
    """Скелет и покрытие колонки, приведённые к ПРЯМЫМ координатам.

    Приведение — то же отражение, что и на входе, поэтому равенство колонок
    после него есть в точности «зеркально равны», а не «похожи по числу».
    """

    from cftuv_envelope.wavefront.bridge import BridgeOutcome

    _, report = _mirror_pair_column(mirrored)
    assert report.outcome is BridgeOutcome.EXACT, report.findings
    skeleton = build_skeleton(report.polygon)
    assert skeleton.outcome is SkeletonOutcome.EXACT, skeleton.outcome
    partition = build_faces(report.polygon, skeleton)
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    covered = coverage_at(partition, alpha)
    assert covered.outcome is CoverageOutcome.EXACT

    def straight(x, y):
        return (y.terms, x.terms) if mirrored else (x.terms, y.terms)

    nodes = sorted(
        straight(node.point.x, node.point.y) for node in skeleton.nodes
    )
    members = sorted(
        (
            tuple(sorted(straight(x, y) for x, y in face.points)),
            face.doubled_area.terms,
        )
        for face in covered.faces
    )
    return nodes, members, covered


@pytest.mark.parametrize("alpha", (Fraction(1), Fraction(2)))
def test_the_mirrored_chart_gives_the_reflected_skeleton_and_coverage(alpha):
    """ЗАКОН ПОРЯДКА ОПОР, проверенный ПОКРЫТИЕМ, а не приёмом входа.

    Пара синтетическая и различается ровно одним: ориентацией карты. Прямая
    колонка — веер `k = 2` из `..._two_supports_of_one_vertex_get_two_faces`,
    зеркальная — та же задача в отражённых координатах карты
    (`COORDINATE_CW_MATCHES_OWNER_PATCH`). Продукт от выбора карты зависеть не
    может, поэтому требуется не «мост принял», а зеркальное равенство ВСЕГО
    ответа: узлы скелета, члены покрытия со своими площадями и сумма площадей.

    До закона зеркальная колонка КРАСНАЯ, и краснеет она не приёмом входа, а
    отказом: `VERTEX_FAN_SUPPORTS_DO_NOT_TURN_WITH_THE_COMPUTATION_LOOP` при
    `lattice=None`, то есть там, где привязка не двигала ни одной вершины.
    """

    direct_nodes, direct_members, direct_covered = _mirror_pair_reading(
        False, alpha
    )
    mirror_nodes, mirror_members, mirror_covered = _mirror_pair_reading(
        True, alpha
    )

    assert mirror_nodes == direct_nodes
    assert mirror_members == direct_members
    # Сумма — ТОЧНОЕ равенство `SqrtSumV1`, а не совпадение до знака: площадь
    # обеих колонок иррациональна, и рациональным это сравнение быть не могло.
    assert (
        mirror_covered.doubled_area - direct_covered.doubled_area
    ).is_zero
    assert direct_covered.polygon_doubled_area == (
        mirror_covered.polygon_doubled_area
    )


def test_the_inverted_support_order_on_a_direct_chart_is_still_refused():
    """ОТРИЦАТЕЛЬНЫЙ КОНТРОЛЬ: закон не ослабил проверку сектора.

    Порядок опор перевёрнут РУКАМИ на ПРЯМОЙ карте, то есть власть ориентации
    сказала «оставить как есть», а вход всё равно пришёл развёрнутым. Мост
    обязан отказать именем, а не молча починить: если бы закон был реализован
    переворотом «до зелёного», этот тест стал бы зелёным вместе с ним.
    """

    from cftuv_envelope.wavefront.bridge import (
        BridgeOutcome,
        VertexFanLawV1,
        bridge_arrival_laws,
        unit_speed_laws_of,
    )
    from cftuv_envelope.wavefront.conveyor import computation_loop_supports

    figure = ell(12)
    inverted = tuple(reversed(MIRROR_OWNER_SUPPORTS))
    assert computation_loop_supports(inverted, 1) == inverted
    report = bridge_arrival_laws(
        tuple(
            tuple((Fraction(x), Fraction(y)) for x, y in loop.points)
            for loop in figure.loops
        ),
        unit_speed_laws_of(figure),
        vertex_fans=(
            VertexFanLawV1(
                "angular-spec:inverted",
                (Fraction(6), Fraction(6)),
                inverted,
            ),
        ),
    )

    assert report.outcome is (
        BridgeOutcome.VERTEX_FAN_SUPPORTS_DO_NOT_TURN_WITH_THE_COMPUTATION_LOOP
    )
    assert report.polygon is None


def test_the_lattice_keeps_the_refusal_it_actually_causes():
    """Разведение имён: у решётки остаётся ровно тот отказ, что её.

    Два якоря веера сидят в РАЗНЫХ точках дроби и садятся в ОДИН узел — это и
    есть работа привязки, поэтому имя `LATTICE_SNAP_BREAKS_A_VERTEX_FAN`
    остаётся правдой и ветка достижима. Отказ вращательного смысла имя сменил
    именно затем, чтобы эти два случая перестали отвечать одинаково: первый
    лечится шагом решётки, второй ориентацией карты.
    """

    from cftuv_envelope.robust.grid import GridSpecV1
    from cftuv_envelope.wavefront.bridge import (
        BridgeOutcome,
        VertexFanLawV1,
        bridge_arrival_laws,
        unit_speed_laws_of,
    )

    figure = ell(12)
    report = bridge_arrival_laws(
        tuple(
            tuple((Fraction(x), Fraction(y)) for x, y in loop.points)
            for loop in figure.loops
        ),
        unit_speed_laws_of(figure),
        lattice=GridSpecV1(1),
        vertex_fans=(
            VertexFanLawV1(
                "angular-spec:first",
                (Fraction(6), Fraction(6)),
                ((-1, -1, Fraction(2)),),
            ),
            VertexFanLawV1(
                "angular-spec:second",
                (Fraction(24, 4) + Fraction(1, 5), Fraction(6)),
                ((-1, -1, Fraction(2)),),
            ),
        ),
    )

    assert report.outcome is BridgeOutcome.LATTICE_SNAP_BREAKS_A_VERTEX_FAN
    assert report.polygon is None


# Сценарий владельца: тот же патч `bf6`, но выбраны ВСЕ 12 цепочек шва. Домен
# объявляет четыре вогнутых угла, и ровно один из них не прямой.
FULL_SELECTION = "building_002_full_selection_v1"
POINT_CONTACT = "building_002_point_contact_v1"
DEGRADED_SPEC = "angular-spec:66cf5e6f75ddafed7cdb3dca"
DEGRADED_RELATION = "host-v0:corner-relation:a2b66a4ed578c532315cd09b"
DEGRADED_SUPPORT = "hidden-support:36a61d30ef0657696ba6f3a0"
# Площадь домена того же патча, замороженная `test_wavefront_conveyor.py` на
# фикстуре `point_contact`. Здесь она обязана СОВПАСТЬ: выделение запроса другое,
# а домен тот же, и это отделяет «переписали фикстуру» от «выбрали больше швов».
BF6_DOUBLED_AREA = 27224141715
FULL_SELECTION_SLIVER_PROJECTION = (
    (
        1,
        -32760370170769515943782596878464889863314396137215851124364992057288761041375893298426919779094145354990494928895896481867486209029817817043,
        16381362189677002231500200280039403729977453338560685993927289108788062591548374450266605252017726090141743409074391949315776949488502911250,
    ),
    (2, -98304, 1426085),
    (165133, -16384, 360799505),
    (266669, -74432, 368215147),
    (44035851977, -16384, 234256993393),
    (355697254093, 1, 8251520),
    (711394508186, 32768, 1701042789875),
    (2300590626205, 11, 20628800),
    (2856933890605, 65536, 6817919287725),
    (5713867781210, 1, 20628800),
    (35567098911805, 1, 82515200),
    (71134197823610, 8192, 4252913666925),
    (934341031240685, 1, 137154880),
    (25438578787060210, 1, 2811675040),
    (170962881381695341, 1, 2249340032),
    (308813560755902945, 1, 5623350080),
    (341925762763390682, 879615279104, 15828538430110778961995),
    (399646465898033210, 1, 11246700160),
    (19497871955518329185, 1, 11246700160),
    (38995743911036658370, 16384, 6297051055227305),
    (297513663411907677816353, -16384, 777846928859140375),
    (864679808037807823744061, -8, 647515592700475),
    (3219742089630608253306605, 16384, 1593174185986412145),
    (8100173827195581712794298, -557056, 137993518029402265175),
    (332342539198733583484373705, -32768, 51995676302758403925),
    (16162756466265633743701502905, 1, 1097159818705024),
    (28418143324800742775510146949, -2, 29346257673784425),
    (36776916529638856010871450722, -32768, 546969297554948422125),
    (439344098428476115533587790629, -8192, 472670056077527729425),
    (
        2669004463543842896998660279211365,
        16384,
        57674213732698744339351,
    ),
    (
        258375540036545773940152217388442350424059002,
        -65536,
        91695830202823208533800348421,
    ),
)
FULL_SELECTION_SLIVER_SHA256 = (
    "9ec373daa7609fefbf0cc0d651354956c2d26fb0c887f49d95a75c8e1c949dd2"
)
FULL_SELECTION_DOMAIN_AREA_SLIVER = Fraction(
    79051943949, 612498843631616
)


def _full_selection_input():
    import cftuv_envelope as kernel

    from wavefront_cases import FIELD_FIXTURE

    folder = FIELD_FIXTURE.parent.parent / FULL_SELECTION
    return (
        kernel.AnalysisSnapshotCodecV1.loads(
            (folder / "analysis_snapshot.json").read_bytes()
        ),
        kernel.DecalRequestCodecV1.loads(
            (folder / "decal_request.json").read_bytes()
        ),
    )


def _without_the_degraded_corner(snapshot):
    """Тот же снапшот, из которого снят РОВНО один угол — деградировавший.

    Снимаются все три его записи (отношение, сектор, сертификат): снапшот с
    отношением без сертификата отверг бы валидатор, и тест мерил бы отказ
    валидации вместо геометрии.
    """

    from dataclasses import replace as dataclass_replace

    (doomed,) = [
        item
        for item in snapshot.corner_relations
        if item.corner_relation_id.value == DEGRADED_RELATION
    ]
    return dataclass_replace(
        snapshot,
        corner_relations=frozenset(
            item for item in snapshot.corner_relations if item is not doomed
        ),
        angular_owner_sectors=frozenset(
            item
            for item in snapshot.angular_owner_sectors
            if item.owner_sector_id != doomed.owner_sector_id
        ),
        reflex_angle_certificates=frozenset(
            item
            for item in snapshot.reflex_angle_certificates
            if item.certificate_id != doomed.reflex_angle_certificate_id
        ),
    )


def _algebraic_fan_directions(module):
    """Подмена рецепта направлений на заведомо алгебраическое `(sqrt(3), 1)`.

    Направление выбрано так, что не спасается и рескейлом: деление на `sqrt(3)`
    рационализирует первую компоненту и портит вторую (`1/sqrt(3)`), то есть
    отказ приходит именно от НАПРАВЛЕНИЯ, а не от записи. Якорь при этом остаётся
    рациональным, поэтому вторая причина деградации в этот тест не подмешивается.
    """

    import sympy as sp

    from cftuv_envelope.reference.planar_types import ExactPlanarVector

    original = module.angular_hidden_support_lines

    def algebraic(context, spec):
        relation, anchor, hidden = original(context, spec)
        return (
            relation,
            anchor,
            tuple(
                (
                    support_id,
                    ExactPlanarVector.from_values(sp.sqrt(3), sp.Integer(1)),
                    constant,
                )
                for support_id, _, constant in hidden
            ),
        )

    return original, algebraic


def test_an_irrational_fan_no_longer_refuses_the_domain_and_the_outcome_is_gone():
    """Нерациональное направление БОЛЬШЕ НЕ ОСТАНАВЛИВАЕТ путь, и исход удалён.

    ЧТО БЫЛО И ПОЧЕМУ ИЗМЕНИЛОСЬ. Прежняя редакция требовала отказа всего домена
    (`ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL`) и была верна как утверждение про
    ТИШИНУ: посчитать острый угол молча — подмена продукта. Она же оказалась
    неверна как утверждение про ЦЕНУ: на полевом входе
    `building_002_full_selection_v1` одна вершина из четырёх уносила с собой три
    рациональных веера и весь домен, то есть владелец не получал ничего вместо
    «почти всего». Решением владельца домен обязан строиться, а деградация —
    оставаться громкой.

    ИСХОД УДАЛЁН, А НЕ ОСТАВЛЕН НА ВСЯКИЙ СЛУЧАЙ, и это здесь проверяется именем:
    после деградации его не эмитит ни одна ветка, то есть по правилу проекта он
    «не сработал бы больше никогда». Атрибута в перечислении нет, и тест это
    сторожит — иначе мёртвое имя вернулось бы в отчёты хоста как живое.

    ЧТО ПУТЬ ИДЁТ ДАЛЬШЕ, показано на этом же входе НЕ равенством `EXACT`:
    `angular_snapshot` строит `PlanarPatchFrameV1`, у которого закона решётки нет
    вовсе, поэтому подготовка обязана споткнуться на СЛЕДУЮЩЕЙ своей ступени —
    `CHART_LATTICE_IS_NOT_DECLARED`. Именно это и доказывает, что веер её больше
    не останавливает; домен, доходящий до `EXACT`, проверяется на полевых байтах
    соседним тестом, где решётка объявлена.

    Счётчик деградации при этом ПЕРЕЖИВАЕТ поздний отказ — на нём и держится
    различение «вееры были рациональны» от «веер деградировал, а споткнулись мы
    дальше». Пер-вершинная запись живёт в регионе, а регионов в этом отказе нет
    по построению, поэтому здесь наблюдаемо только число; вершину поимённо
    называет полевой тест.
    """

    from cftuv_envelope.wavefront import conveyor as module

    from reference_factories import angular_snapshot

    assert not hasattr(
        module.ConveyorOutcome, "ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL"
    )

    snapshot, request = angular_snapshot(1)
    original, algebraic = _algebraic_fan_directions(module)
    module.angular_hidden_support_lines = algebraic
    try:
        prepared = module.prepare_conveyor(snapshot, request)
    finally:
        module.angular_hidden_support_lines = original

    assert (
        prepared.outcome
        is module.ConveyorOutcome.CHART_LATTICE_IS_NOT_DECLARED
    )
    assert prepared.detail == "PlanarPatchFrameV1"
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 1
    assert prepared.counter("CONVEYOR_RATIONAL_VERTEX_FANS") == 0


def test_every_fan_degrading_leaves_the_field_domain_mitered_bit_for_bit():
    """Деградация ВСЕХ четырёх вееров даёт ровно тот домен, у которого углов нет.

    Здесь проверяется, что деградация — это МИТР, а не третий, непредусмотренный
    ответ (веер с пустыми опорами, сдвинутый якорь, потерянное ребро). Проверка
    идёт не осмотром результата, а сравнением двух входов на ОДНИХ байтах:

    - направления всех четырёх вееров подменены на заведомо алгебраические, то
      есть деградируют все четыре;
    - у второго входа сняты сами угловые отношения, то есть веера не требуется
      вовсе — это и есть митрованный домен по определению.

    Скелет обязан совпасть ПОБИТОВО (`semantic_digest`), покрытие — членами.
    Совпадение и есть смысл слова «митрованная» в отчёте очереди; расхождение
    означало бы, что деградация оставляет след в геометрии, а такой след был бы
    той самой тихой подменой, ради запрета которой всё это и стоит.

    Вход выбран полевой, а не синтетический, по измеренной причине: у
    `angular_snapshot` кадр без закона решётки, и подготовка на нём не доходит до
    региона вовсе (соседний тест это и фиксирует). Полевые байты решётку
    объявляют, поэтому только на них видно, что домен ДОСТРОИЛСЯ.
    """

    from dataclasses import replace as dataclass_replace

    from cftuv_envelope.wavefront import conveyor as module
    from cftuv_envelope.wavefront.digest import semantic_digest

    snapshot, request = _full_selection_input()
    original, algebraic = _algebraic_fan_directions(module)
    module.angular_hidden_support_lines = algebraic
    try:
        prepared = module.prepare_conveyor(snapshot, request)
        covered = module.conveyor_coverage(prepared)
    finally:
        module.angular_hidden_support_lines = original

    assert prepared.outcome is module.ConveyorOutcome.EXACT, prepared.detail
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 4
    assert prepared.counter("CONVEYOR_RATIONAL_VERTEX_FANS") == 0
    # Ни одной опоры и ни одного ребра веера в полигоне: сеять оказалось нечего.
    assert prepared.counter("CONVEYOR_FAN_SUPPORTS") == 0
    assert prepared.counter("CONVEYOR_FAN_EDGES") == 0
    # Митр здесь ВЫНУЖДЕННЫЙ у всех четырёх, а не выбранный профилем: счётчик
    # законных `k = 0` остаётся нулём, и одно от другого отличимо.
    assert prepared.counter("CONVEYOR_MITERED_CORNERS") == 0

    (region,) = prepared.regions
    assert len(region.degraded_miter_corners) == 4
    # Каждая вершина названа СВОИМ угловым отношением: четыре записи, четыре
    # различных имени. Одного счёта «4» было бы мало — он держался бы и при
    # четырёх копиях одной вершины.
    assert len(
        {corner.corner_relation_id for corner in region.degraded_miter_corners}
    ) == 4
    assert DEGRADED_RELATION in {
        corner.corner_relation_id for corner in region.degraded_miter_corners
    }
    assert all(
        corner.anchor is not None and corner.reason
        for corner in region.degraded_miter_corners
    )

    mitred = module.prepare_conveyor(
        dataclass_replace(
            snapshot,
            corner_relations=frozenset(),
            angular_owner_sectors=frozenset(),
            reflex_angle_certificates=frozenset(),
        ),
        request,
    )
    assert mitred.outcome is module.ConveyorOutcome.EXACT, mitred.detail
    assert mitred.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 0
    (mitred_region,) = mitred.regions
    assert mitred_region.degraded_miter_corners == ()
    assert semantic_digest(mitred_region.skeleton) == semantic_digest(
        region.skeleton
    )
    assert (
        module.conveyor_coverage(mitred).doubled_area.terms
        == covered.doubled_area.terms
    )


def test_the_field_fixtures_declare_their_corner_relations_by_name():
    """ЧТО ИМЕННО объявляет каждая полевая фикстура, и почему числа у них разные.

    Прежняя редакция этого теста называлась «фикстуры вееров не содержат вовсе»
    и была верна: адаптер хоста углов не объявлял, поэтому прежние митрованные
    числа `bf6` не сдвинулись, а сильнейший критерий среза — равенство путей —
    был достижим, но пуст. Блокер снят на стороне хоста (сертификат угла
    переживает запись, `cb444d5`), фикстура `point_contact` пересобрана, и
    утверждение ИНВЕРТИРОВАЛОСЬ. Оставить прежнее было бы нельзя: оно сторожило
    ровно то состояние, выход из которого и был целью.

    Три фикстуры отвечают РАЗНОЕ, и все цифры содержательны, поэтому тест
    перечисляет их поимённо, а не сводит к «углы есть»:

    | фикстура | углов | вееров на запросе | почему столько |
    |---|---:|---:|---|
    | `building_002_point_contact_v1`   | 4 | 2 | вырезы домена; на выбранные рёбра запроса (2/3/7) попадают два угла |
    | `building_002_full_selection_v1`  | 4 | 4 | ТЕ ЖЕ четыре угла того же патча, но выбраны все 12 цепочек, поэтому в план попадают все |
    | `building_002_weighted_normals_v1`| 0 | 0 | домен ВЫПУКЛЫЙ прямоугольник — вогнутых углов в нём нет по построению |

    Пара «4 угла → 2 спеки» и «4 угла → 4 спеки» на ОДНОМ патче отделяет объявление
    от выбора: углы объявляет топология, а сколько их дойдёт до плана — решает
    выделение запроса. Без второй строки числа первой читались бы как свойство меша.

    Ноль у третьей — не пропуск адаптера, и это здесь проверено, а не заявлено:
    у выпуклого домена вогнутого угла не бывает, поэтому `CornerRelation` ему
    взяться неоткуда. Именно поэтому равенство с эталоном на нём — точный ноль
    (`test_wavefront_conveyor.py`), а на `bf6` со стенами остаётся остаток.

    Сверх счёта проверяется СОГЛАСОВАННОСТЬ объявления: каждому углу отвечает
    свой сектор и свой сертификат, и все три множества равномощны. Один счёт
    углов этого не дал бы — снапшот с четырьмя углами и тремя сертификатами
    прошёл бы проверку «углов 4».
    """

    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.envelopes import (
        AngularEnvelopeSpec,
        CertifiedBoundHiddenSupportSpecV1,
    )
    from cftuv_envelope.reference.compile import compile_reference_envelopes

    from wavefront_cases import FIELD_FIXTURE

    expected = {
        "building_002_point_contact_v1": (4, 2),
        "building_002_full_selection_v1": (4, 4),
        "building_002_weighted_normals_v1": (0, 0),
    }
    root = FIELD_FIXTURE.parent
    for name, (corner_count, fan_count) in expected.items():
        folder = root.parent / name
        snapshot = kernel.AnalysisSnapshotCodecV1.loads(
            (folder / "analysis_snapshot.json").read_bytes()
        )
        request = kernel.DecalRequestCodecV1.loads(
            (folder / "decal_request.json").read_bytes()
        )
        assert len(snapshot.corner_relations) == corner_count, name
        # Объявление согласовано: угол, сектор и сертификат — по одному на угол.
        assert len(snapshot.angular_owner_sectors) == corner_count, name
        assert len(snapshot.reflex_angle_certificates) == corner_count, name
        assert len(
            {item.owner_sector_id for item in snapshot.corner_relations}
        ) == corner_count, name
        assert len(
            {item.reflex_angle_certificate_id for item in snapshot.corner_relations}
        ) == corner_count, name
        compilation = compile_reference_envelopes(snapshot, request).compilation
        assert compilation is not None, name
        angular = [
            spec
            for spec in compilation.envelope_specs
            if isinstance(spec, AngularEnvelopeSpec)
        ]
        assert len(angular) == fan_count, name
        # У каждой угловой спеки профиль выбрал k = 1, то есть цепь, а не митр.
        assert all(spec.resolved_hidden_edge_count == 1 for spec in angular), name


def test_every_named_fixture_manifest_matches_its_directory():
    """Именованный manifest не имеет права называться соседней фикстурой.

    Отсутствие `fixture_id` разрешено и не является отказом. Сейчас такой
    exporter-format debt фактически несут ровно четыре существующих каталога:
    `session_a_v5`, `session_c_planar_v1`,
    `session_c_r2c_boundary_rotation_v1`, `session_d_interactions_v1`.
    Они здесь не переписываются и проходят guard отсутствием поля.
    """

    import json
    from pathlib import Path

    fixture_root = Path(__file__).parents[1] / "fixtures"
    for manifest_path in sorted(fixture_root.rglob("manifest.json")):
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if "fixture_id" not in manifest:
            continue
        assert manifest["fixture_id"] == manifest_path.parent.name, manifest_path


def test_the_full_selection_domain_builds_with_four_rational_fans():
    """Привязанный ordinal снимает единственную деградацию и достраивает веер.

    На тех же байтах было 3 веера + 1 видимый митр, 13 узлов, 15 граней и
    26 членов покрытия. Общая bound-геометрия даёт 4/0, четыре plan-authority
        directions, 14 узлов, 16 граней и 28 членов. Сумма граней по-прежнему
        равна площади домена точно; стен нет, потому что выбраны все 12 источников.
    """

    from cftuv_envelope.wavefront import (
        ConveyorOutcome,
        conveyor_coverage,
        prepare_conveyor,
    )
    from cftuv_envelope.wavefront.bridge import BridgeOutcome
    from cftuv_envelope.wavefront.digest import semantic_digest

    snapshot, request = _full_selection_input()
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome is ConveyorOutcome.EXACT, prepared.detail
    assert prepared.detail == ""
    assert dict(prepared.counters) == {
        "CONVEYOR_DOMAIN_REGIONS": 1,
        "CONVEYOR_ARRIVAL_LAWS": 12,
        "CONVEYOR_RESCALED_ARRIVAL_LAWS": 16,
        "CONVEYOR_RATIONAL_VERTEX_FANS": 4,
        "CONVEYOR_DEGRADED_MITER_CORNERS": 0,
        "CONVEYOR_MITERED_CORNERS": 0,
        "CONVEYOR_BOUND_FAN_DIRECTIONS": 4,
        "CONVEYOR_FAN_SUPPORTS": 4,
        "CONVEYOR_FAN_EDGES": 4,
        "CONVEYOR_LATTICE_SCALE": 32768,
        "CONVEYOR_DOMAIN_EDGES": 12,
        "CONVEYOR_SOURCE_EDGES": 12,
        "CONVEYOR_WALL_EDGES": 0,
        "CONVEYOR_AMBIGUOUS_OWNER_EDGES": 0,
        "CONVEYOR_WEIGHTED_SOURCE_EDGES": 12,
        "CONVEYOR_SKELETON_NODES": 14,
        "CONVEYOR_FACES": 16,
    }

    (region,) = prepared.regions
    assert region.bridge_outcome is BridgeOutcome.EXACT
    assert region.bridge.findings == ()
    assert region.skeleton_outcome is SkeletonOutcome.EXACT
    assert region.face_outcome is FaceOutcome.EXACT
    assert len(region.skeleton.nodes) == 14
    assert sorted(face.node_count for face in region.partition.faces) == [
        1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 5, 6, 7,
    ]
    # Сумма граней есть площадь домена ТОЧНО: дефект — пустой набор членов, а не
    # «маленькое число». И площадь та же, что у `point_contact`: домен один.
    assert region.partition.polygon_doubled_area == BF6_DOUBLED_AREA
    assert region.partition.area_defect.terms == ()
    assert region.wall_spans == ()
    assert region.ambiguous_owner_spans == ()

    assert region.degraded_miter_corners == ()
    _assert_the_bound_corner_is_the_only_one_off_the_right_angle(snapshot)

    covered = conveyor_coverage(prepared)
    assert covered.outcome is ConveyorOutcome.EXACT, covered.detail
    assert covered.alpha == Fraction(1, 4)
    assert covered.lattice_alpha == Fraction(8192)
    assert len(covered.doubled_area.terms) == 28
    low, high = covered.doubled_area.enclosure(64)
    assert 4600769253 < low <= high < 4600769254
    assert covered.polygon_doubled_area == BF6_DOUBLED_AREA
    assert covered.counter("CONVEYOR_COVERED_FACES") == 16
    assert covered.counter("CONVEYOR_COVERAGE_TERMS") == 28
    assert covered.counter("CONVEYOR_NAMED_OWNERS") == 12
    assert (
        sum(1 for face in covered.faces if face.envelope_instance_id is None) == 4
    )
    # При заведомо большой alpha фронт съедает домен целиком, и покрытие сходится
    # с его площадью в один рациональный член — тот же точный ноль, но с другой
    # стороны, чем `area_defect`.
    full = conveyor_coverage(prepared, "1000000")
    assert full.doubled_area.terms == ((1, Fraction(BF6_DOUBLED_AREA)),)

    trimmed = prepare_conveyor(_without_the_degraded_corner(snapshot), request)
    assert trimmed.outcome is ConveyorOutcome.EXACT, trimmed.detail
    assert trimmed.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 0
    assert trimmed.counter("CONVEYOR_RATIONAL_VERTEX_FANS") == 3
    assert trimmed.counter("CONVEYOR_BOUND_FAN_DIRECTIONS") == 3
    (trimmed_region,) = trimmed.regions
    assert semantic_digest(trimmed_region.skeleton) != semantic_digest(
        region.skeleton
    )
    assert (
        conveyor_coverage(trimmed).doubled_area.terms
        != covered.doubled_area.terms
    )


def _point_contact_input():
    import cftuv_envelope as kernel

    from wavefront_cases import FIELD_FIXTURE

    folder = FIELD_FIXTURE.parent.parent / POINT_CONTACT
    return (
        kernel.AnalysisSnapshotCodecV1.loads(
            (folder / "analysis_snapshot.json").read_bytes()
        ),
        kernel.DecalRequestCodecV1.loads(
            (folder / "decal_request.json").read_bytes()
        ),
    )


def _canonical_moving_line(normal_x, normal_y, constant, speed_squared):
    """Положительно-нормированная запись `(a,b,c,q)` без production bridge."""

    values = tuple(map(sp.sympify, (normal_x, normal_y)))
    first = next(value for value in values if value != 0)
    sign = sp.sign(first)
    assert sign in (-1, 1)
    scale = first if sign > 0 else -first

    def rational(value):
        exact = sp.cancel(sp.radsimp(value))
        assert exact.is_Rational, exact
        return Fraction(int(exact.p), int(exact.q))

    return (
        rational(values[0] / scale),
        rational(values[1] / scale),
        rational(sp.sympify(constant) / scale),
        rational(sp.sympify(speed_squared) / (scale * scale)),
    )


def test_chart_lattice_binding_is_pointwise_one_geometry_and_one_line_set():
    """B1/B2: домен и все moving supports тождественны ДО exact union."""

    from dataclasses import replace

    import cftuv_envelope as kernel
    from cftuv_envelope.interactions.arrival import (
        angular_hidden_support_lines,
        strip_front_support_line,
    )
    from cftuv_envelope.reference.boundary import build_domain_geometry
    from cftuv_envelope.reference.common import GeometryContext
    from cftuv_envelope.reference.planar_types import polygon_signed_area
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront.event_time import SupportLineV1

    snapshot, request = _full_selection_input()
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT", prepared.detail
    compilation = prepared.compilation
    context = prepared.context
    binding = compilation.evaluation_geometry_binding
    scale = prepared.lattice.scale
    assert binding.lattice_scale == scale == 32768

    # Независимый half-up рецепт по каждой source vertex: production snap
    # helper для expected не вызывается.
    frame_coordinates = {
        item.source_vertex_id: item.domain_coordinate
        for item in context.frame.exact_source_vertex_coordinates
    }
    bound_coordinates = {
        item.source_vertex_id: item.domain_coordinate
        for item in binding.source_vertex_coordinates
    }
    assert set(bound_coordinates) == set(frame_coordinates)
    moved = 0
    for vertex_id, source in frame_coordinates.items():
        source_xy = tuple(
            Fraction(value.numerator, value.denominator)
            for value in (source.x, source.y)
        )
        expected_node = tuple(
            (value * scale + Fraction(1, 2)).numerator
            // (value * scale + Fraction(1, 2)).denominator
            for value in source_xy
        )
        actual = bound_coordinates[vertex_id]
        assert tuple(
            Fraction(value.numerator, value.denominator)
            for value in (actual.x, actual.y)
        ) == tuple(Fraction(value, scale) for value in expected_node)
        moved += int(
            source_xy != tuple(Fraction(value, scale) for value in expected_node)
        )
    assert len(bound_coordinates) == 12
    assert moved == 9

    (domain_region,) = prepared.domain.domain_regions
    reference_nodes = tuple(
        tuple(
            int(
                Fraction(int(value.p), int(value.q))
                * scale
            )
            for value in point.expressions()
        )
        for point in domain_region.outer.points
    )
    polygon = prepared.regions[0].bridge.polygon
    assert polygon is not None
    assert reference_nodes == polygon.outer.points
    assert reference_nodes == (
        (469074, -101640),
        (212020, -27984),
        (232392, -39678),
        (20371, 21074),
        (0, 32768),
        (-113982, 65428),
        (0, 0),
        (65428, -18748),
        (32768, 0),
        (485615, -129758),
        (518223, -148491),
        (583056, -167068),
    )
    bound_area = polygon_signed_area(domain_region.outer.points)
    queue_area = Fraction(
        prepared.regions[0].partition.polygon_doubled_area,
        2 * scale * scale,
    )
    assert bound_area == queue_area == Fraction(
        27224141715,
        2147483648,
    )

    source_context = GeometryContext.build(
        replace(compilation, evaluation_geometry_binding=None),
        context.frame,
        require_evaluation_binding=False,
    )
    (source_region,) = build_domain_geometry(source_context).domain_regions
    source_area = polygon_signed_area(source_region.outer.points)
    assert source_area == Fraction(3615798, 285217)
    assert source_area - bound_area == Fraction(
        79051943949,
        612498843631616,
    )

    reference_boundary = {}
    for spec in compilation.envelope_specs:
        if not isinstance(spec, kernel.StripEnvelopeSpec):
            continue
        seed = next(
            item
            for item in compilation.seeds
            if getattr(item, "seed_id", None) == spec.source_seed_id
        )
        records = []
        for source in context.support_segments_for_use(
            seed.chain_use_id,
            spec.envelope_spec_id.value,
        ):
            normal, constant = strip_front_support_line(context, source)
            records.append(
                _canonical_moving_line(
                        *normal.expressions(),
                        constant.as_expr() * scale,
                        1,
                )
            )
        reference_boundary[spec.envelope_spec_id.value] = tuple(
            sorted(records)
        )

    owners = dict(prepared.regions[0].owner_by_edge)
    queue_boundary = {}
    for start, end, speed_squared in polygon.edges():
        key = (*start, *end)
        owner = owners[key]
        line = SupportLineV1.with_speed(start, end, speed_squared)
        queue_boundary.setdefault(owner, []).append(
            _canonical_moving_line(line.a, line.b, line.c, line.q)
        )
    queue_boundary = {
        owner: tuple(sorted(records))
        for owner, records in queue_boundary.items()
    }
    assert sum(map(len, reference_boundary.values())) == 12
    assert reference_boundary == queue_boundary

    reference_fans = {}
    relation_by_id = {
        item.corner_relation_id: item for item in snapshot.corner_relations
    }
    node_by_vertex = {
        item.source_vertex_id: (
            int(
                Fraction(
                    item.domain_coordinate.x.numerator,
                    item.domain_coordinate.x.denominator,
                )
                * scale
            ),
            int(
                Fraction(
                    item.domain_coordinate.y.numerator,
                    item.domain_coordinate.y.denominator,
                )
                * scale
            ),
        )
        for item in binding.source_vertex_coordinates
    }
    queue_fan_lines = {
        (point, ordinal): line
        for point, ordinal, line in polygon.fan_edges()
    }
    for spec in compilation.envelope_specs:
        if not isinstance(spec, kernel.AngularEnvelopeSpec):
            continue
        _, _, hidden = angular_hidden_support_lines(context, spec)
        by_ordinal = {item.ordinal: item for item in spec.hidden_supports}
        relation = relation_by_id[spec.source_relation_id]
        point = node_by_vertex[relation.source_vertex_id]
        for ordinal, (support_id, normal, constant) in enumerate(
            hidden,
            start=1,
        ):
            plan_support = by_ordinal[ordinal]
            assert support_id == plan_support.hidden_support_id.value
            reference_fans[support_id] = _canonical_moving_line(
                *normal.expressions(),
                constant.as_expr() * scale,
                1,
            )
            queue_line = queue_fan_lines[(point, ordinal)]
            assert queue_line.c == (
                queue_line.a * point[0] + queue_line.b * point[1]
            )
            assert reference_fans[support_id] == _canonical_moving_line(
                queue_line.a,
                queue_line.b,
                queue_line.c,
                queue_line.q,
            )
    assert len(reference_fans) == len(queue_fan_lines) == 4


def _assert_the_bound_corner_is_the_only_one_off_the_right_angle(snapshot):
    """K остаётся фактом source angle; directions принадлежат bound geometry.

    Проверка стоит отдельной функцией не ради длины: она утверждает про ВХОД, а не
    про ответ очереди, и потому обязана читаться независимо от того, что очередь
    посчитала.
    """

    from decimal import Decimal

    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.envelopes import (
        AngularEnvelopeSpec,
        CertifiedBoundHiddenSupportSpecV1,
    )

    compilation = kernel.compile_reference_envelopes(
        snapshot, _full_selection_input()[1]
    ).compilation
    by_certificate = {
        item.certificate_id: item for item in snapshot.reflex_angle_certificates
    }
    seen = {}
    bound_supports = {}
    for spec in compilation.envelope_specs:
        if not isinstance(spec, AngularEnvelopeSpec):
            continue
        seen[spec.envelope_spec_id.value] = by_certificate[
            spec.angle_certificate_id
        ].measure_payload.phi_over_pi
        bound_supports[spec.envelope_spec_id.value] = tuple(
            item
            for item in spec.hidden_supports
            if isinstance(item, CertifiedBoundHiddenSupportSpecV1)
        )
    assert len(seen) == 4
    bound_interval = seen.pop(DEGRADED_SPEC)
    assert bound_interval.lower > Decimal("1.5")
    assert str(bound_interval.lower) == "1.500123280352543932868625829"
    assert str(bound_interval.upper) == "1.50012328035254393286862583"
    assert all(len(supports) == 1 for supports in bound_supports.values())
    assert (
        bound_supports[DEGRADED_SPEC][0]
        .direction_binding.bound_primitive_integer_vector
        == (1, 3)
    )
    assert {
        support.direction_binding.binding_reason.value
        for supports in bound_supports.values()
        for support in supports
    } == {
        "SOURCE_DIRECTION_IRRATIONAL",
        "EVALUATION_GEOMETRY_UNBINDS_SOURCE_RATIONAL",
    }
    # У трёх рациональных вееров сертификат — ТОЧКА: оба конца равны `1.5`, то
    # есть угол прямой доказанно, а не в пределах округления.
    for name, interval in seen.items():
        assert str(interval.lower) == str(interval.upper) == "1.5", name


def _canonical_sqrt_sum(expression) -> SqrtSumV1:
    result = SqrtSumV1.zero()
    for term in sp.Add.make_args(sp.expand(expression)):
        coefficient, radical = term.as_coeff_Mul(rational=True)
        rational = Fraction(int(coefficient.p), int(coefficient.q))
        if radical == 1:
            result = result + SqrtSumV1.rational(rational)
            continue
        base, exponent = radical.as_base_exp()
        assert exponent == sp.Rational(1, 2), term
        assert base.is_Rational, term
        radicand = Fraction(int(base.p), int(base.q))
        result = result + SqrtSumV1.radical(rational, radicand)
    return result


def test_bound_full_selection_domain_area_is_shared_exactly():
    """Bound-domain и partition читают одну площадь без прежнего sliver.

    До `CHART_LATTICE_BOUND` точная разность была
    `79051943949/612498843631616`. Константа сохранена ниже как исторический
    факт, но новый контракт требует доказанный ноль на общей bound-геометрии.
    """

    import cftuv_envelope as kernel
    from cftuv_envelope.reference.boundary import build_domain_geometry
    from cftuv_envelope.reference.common import GeometryContext
    from cftuv_envelope.reference.planar_types import polygon_signed_area
    from cftuv_envelope.reference.validation import (
        validate_reference_geometry_payload,
    )
    from cftuv_envelope.wavefront import prepare_conveyor

    snapshot, request = _full_selection_input()
    compilation = kernel.compile_reference_envelopes(
        snapshot, request
    ).compilation
    assert compilation is not None
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    assert frame is not None, diagnostics
    context = GeometryContext.build(compilation, frame)
    domain = build_domain_geometry(context)
    reference_area = sum(
        (
            polygon_signed_area(region.outer.points)
            - sum(
                (
                    polygon_signed_area(hole.points)
                    for hole in region.holes
                ),
                sp.Integer(0),
            )
            for region in domain.domain_regions
        ),
        sp.Integer(0),
    )
    prepared = prepare_conveyor(snapshot, request)
    queue_area = sp.Rational(
        prepared.regions[0].partition.polygon_doubled_area,
        2 * prepared.lattice.scale * prepared.lattice.scale,
    )
    sliver = sp.cancel(reference_area - queue_area)
    assert sliver.is_Rational
    assert Fraction(int(sliver.p), int(sliver.q)) == 0
    assert FULL_SELECTION_DOMAIN_AREA_SLIVER == Fraction(
        79051943949, 612498843631616
    )
    assert FULL_SELECTION_DOMAIN_AREA_SLIVER > 0


def test_the_bound_reflex_anchor_is_an_exact_lattice_vertex():
    """Прежний смещённый anchor теперь точно совпадает с bound-узлом.

    До привязки displacement был
    `(561163/1426085, 94954/1426085)` и никакое направление внутри reflex-sector
    не могло одновременно пройти через source-anchor и snapped-node. После B1
    это две записи одной точки `(485615, -129758)` на решётке 32768; искать
    «исправляющий» ковектор больше не требуется.
    """

    import cftuv_envelope as kernel
    from cftuv_envelope.reference.common import GeometryContext
    from cftuv_envelope.reference.validation import (
        validate_reference_geometry_payload,
    )
    from cftuv_envelope.robust.grid import snap_value
    from cftuv_envelope.wavefront import prepare_conveyor

    snapshot, request = _full_selection_input()
    compilation = kernel.compile_reference_envelopes(
        snapshot, request
    ).compilation
    assert compilation is not None
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    assert frame is not None, diagnostics
    context = GeometryContext.build(compilation, frame)
    relation = next(
        item
        for item in snapshot.corner_relations
        if item.corner_relation_id.value == DEGRADED_RELATION
    )
    anchor = context.points_by_id[relation.source_vertex_id]
    x, y = (
        Fraction(int(value.p), int(value.q))
        for value in anchor.expressions()
    )
    lattice = prepare_conveyor(snapshot, request).lattice
    node = (snap_value(x, lattice), snap_value(y, lattice))
    displacement = (
        Fraction(node[0]) - x * lattice.scale,
        Fraction(node[1]) - y * lattice.scale,
    )
    assert (x, y) == (
        Fraction(485615, 32768),
        Fraction(-64879, 16384),
    )
    assert node == (485615, -129758)
    assert displacement == (Fraction(0), Fraction(0))


def test_bound_full_selection_freezes_the_exact_queue_reference_sliver():
    """Оба evaluator'а на одной bound-геометрии дают доказанный exact zero.

    История до `CHART_LATTICE_BOUND`: после первого direction binding остаток
    имел 31 radical-term, digest `9ec373daa7609fefbf0cc0d651354956c2d26fb0c887f49d95a75c8e1c949dd2`
    и положительную оболочку `~3.4065385e-6`. Это примерно в 580 раз меньше
    исходного относительного дефекта `+0.09225%`, но всё ещё не ноль.

    Теперь canonical projection разности пуста. Десятичный tolerance здесь
    запрещён: B1/B2 делают ноль следствием тождества входной геометрии и прямых.
    """

    import cftuv_envelope as kernel
    from cftuv_envelope.wavefront import (
        ConveyorOutcome,
        conveyor_coverage,
        prepare_conveyor,
    )

    snapshot, request = _full_selection_input()
    prepared = prepare_conveyor(snapshot, request)
    covered = conveyor_coverage(prepared)
    assert covered.outcome is ConveyorOutcome.EXACT, covered.detail
    # Стен нет ни одной: объяснение остатка моделью стены исключено ЧИСЛОМ.
    assert prepared.counter("CONVEYOR_WALL_EDGES") == 0
    assert prepared.counter("CONVEYOR_SOURCE_EDGES") == 12
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 0
    assert prepared.counter("CONVEYOR_BOUND_FAN_DIRECTIONS") == 4

    scale = prepared.lattice.scale
    queue = covered.doubled_area.scaled(Fraction(1, 2 * scale * scale))
    evaluated = kernel.evaluate_reference_raw_coverage(
        kernel.compile_reference_envelopes(snapshot, request).compilation,
        request.requested_alpha,
    )
    reference = _canonical_sqrt_sum(
        sp.sympify(evaluated.raw_coverage.exact_area_expression)
    )
    difference = queue - reference
    assert difference.terms == ()
    assert difference.is_zero


def test_bound_point_contact_decomposes_wall_and_lattice_residual_to_zero():
    """B4: девять стен остаются моделью покрытия, sliver исчезает отдельно.

    До CHART_LATTICE_BOUND разность была `+0.00026650827%` и называлась
    `wall+sliver`, не будучи разложенной. Теперь стеновая часть измерена как
    `bound domain − coverage` и положительна, а QUEUE и exact union независимо
    дают одно и то же покрытие. Поэтому lattice-sliver — отдельный точный ноль,
    а не компенсация большой стеновой величиной.
    """

    import cftuv_envelope as kernel
    from cftuv_envelope.reference.evaluation_geometry import (
        evaluation_geometry_binding_residual,
    )
    from cftuv_envelope.wavefront import (
        ConveyorOutcome,
        conveyor_coverage,
        prepare_conveyor,
    )

    snapshot, request = _point_contact_input()
    prepared = prepare_conveyor(snapshot, request)
    covered = conveyor_coverage(prepared)
    assert covered.outcome is ConveyorOutcome.EXACT, covered.detail
    assert prepared.counter("CONVEYOR_SOURCE_EDGES") == 3
    assert prepared.counter("CONVEYOR_WALL_EDGES") == 9
    assert prepared.counter("CONVEYOR_BOUND_FAN_DIRECTIONS") == 2
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 0

    region = prepared.regions[0]
    binding_residual = evaluation_geometry_binding_residual(
        prepared.context.frame,
        prepared.compilation.evaluation_geometry_binding,
    )
    assert binding_residual == Fraction(684097, 46729953280)
    assert region.bridge.snap_residual == binding_residual
    assert region.partition.area_defect.is_zero
    assert len(region.wall_spans) == 9
    assert region.ambiguous_owner_spans == ()
    scale = prepared.lattice.scale
    domain_area = SqrtSumV1.rational(
        Fraction(
            region.partition.polygon_doubled_area,
            2 * scale * scale,
        )
    )
    queue_area = covered.doubled_area.scaled(
        Fraction(1, 2 * scale * scale)
    )
    wall_residual = domain_area - queue_area
    assert wall_residual.terms == (
        (
            1,
            Fraction(
                10683988724414818340594601845691056497874253431297883467,
                839683950514132914842709869716805561674773758660837376,
            ),
        ),
        (27851087972573, Fraction(-1, 73015552)),
        (174086333192405, Fraction(-1, 182538880)),
        (180146433272521, Fraction(-1, 73015552)),
        (28747398459061414865, Fraction(-16384, 5983606356581395)),
        (29748120965591210293, Fraction(-8192, 2421345159454085)),
        (35827778823357770065, Fraction(-4096, 1549612092808361)),
        (231741272493937377005, Fraction(-8192, 7462851791807771)),
    )
    assert wall_residual.sign() > 0

    evaluated = kernel.evaluate_reference_raw_coverage(
        kernel.compile_reference_envelopes(snapshot, request).compilation,
        request.requested_alpha,
    )
    reference_area = _canonical_sqrt_sum(
        sp.sympify(evaluated.raw_coverage.exact_area_expression)
    )
    assert domain_area - reference_area == wall_residual
    assert queue_area - reference_area == SqrtSumV1.zero()


def test_the_closed_form_shortfall_does_not_apply_to_the_field_domain():
    """ГИПОТЕЗА ОПРОВЕРГНУТА: `alpha^2*(3-2*sqrt(2))` на `bf6` не выполняется.

    Ожидание было такое: у двух вееров ПРЯМЫХ углов недобор цепи против митра
    известен замкнутой формой, значит на `bf6` расхождение митра и цепи обязано
    объясняться ровно двумя веерами — `2*alpha^2*(6-4*sqrt(2))` в удвоенной
    площади при `alpha = 8192` на решётке `32768`. Прогон это опровергает:

    | величина | число |
    |---|---:|
        | измеренный недобор (удвоенная площадь) | `8 384 741.4334` |
        | замкнутая форма для двух прямых углов  | `805306368 - 536870912*sqrt(2)` = `46 056 243.0060` |
        | отношение измеренного к предсказанному | `0.1821` |

    ПОСЫЛКА ПРО УГОЛ ПРИ ЭТОМ ВЕРНА, и это проверено здесь же: оба сертификата,
    попавшие в веера, объявляют `phi/pi` РОВНО `1.5`, то есть угол 3pi/2 точно, а
    не приближённо. Ломается не угол — ломаются две другие посылки замкнутой
    формы, и обе названы числом:

    1. **карта не ортонормальна.** Матрица Грама патча далека от единичной
       (`m00 = 2066105/262144`, `m01 = 13024865/524288`), поэтому прямой угол
       ИСТОЧНИКА прямым углом в координатах карты не является, а `alpha` в
       единицах решётки не есть евклидов радиус в карте;
    2. **скорости фронтов не единичные** — три взвешенных ребра-источника и два
       bound-направления; всего переписаны 3 юбки + 2 опоры веера.

    Решающая улика — РАДИКАЛЫ. Замкнутая форма даёт величину вида `a + b*sqrt(2)`;
    измеренный недобор несёт собственные радикалы bound-домена
    (`sqrt(27851087972573)`, `sqrt(174086333192405)` и далее), то есть
    принадлежит другому полю чисел вовсе, и никаким выбором `r` в
    `r*alpha^2*(6-4*sqrt(2))` получен быть не может. Проверено перебором `r`
    от 1 до 4.

    Замкнутая форма от этого не отменяется: она держится на СИНТЕТИЧЕСКОМ корпусе
    (`ell`, `staircase`, `u_shape` — семь строк в
    `test_the_frozen_numbers_of_the_chain_are_written_down`), где все три посылки
    выполнены. Область её применимости теперь названа границей, а не подразумевается.
    """

    from dataclasses import replace as dataclass_replace

    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec
    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

    from wavefront_cases import FIELD_FIXTURE

    folder = FIELD_FIXTURE.parent
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (folder / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (folder / "decal_request.json").read_bytes()
    )

    # Посылка про угол: оба веера стоят на РОВНО 3pi/2.
    compilation = kernel.compile_reference_envelopes(snapshot, request).compilation
    fan_certificate_ids = {
        spec.angle_certificate_id
        for spec in compilation.envelope_specs
        if isinstance(spec, AngularEnvelopeSpec)
    }
    assert len(fan_certificate_ids) == 2
    for certificate in snapshot.reflex_angle_certificates:
        if certificate.certificate_id not in fan_certificate_ids:
            continue
        interval = certificate.measure_payload.phi_over_pi
        assert str(interval.lower) == str(interval.upper) == "1.5"

    # Посылка про карту: матрица Грама НЕ единичная, и это её и ломает.
    gram = next(iter(snapshot.surface_metric_descriptors)).exact_gram_matrix
    as_fraction = lambda value: Fraction(value.numerator, value.denominator)
    assert as_fraction(gram.m00) == Fraction(2066105, 262144)
    assert as_fraction(gram.m01) == Fraction(13024865, 524288)
    assert as_fraction(gram.m00) != 1 or as_fraction(gram.m01) != 0

    prepared = prepare_conveyor(snapshot, request)
    covered = conveyor_coverage(prepared)
    mitred = prepare_conveyor(
        dataclass_replace(
            snapshot,
            corner_relations=frozenset(),
            angular_owner_sectors=frozenset(),
            reflex_angle_certificates=frozenset(),
        ),
        request,
    )
    mitred_covered = conveyor_coverage(mitred)
    assert prepared.counter("CONVEYOR_FAN_SUPPORTS") == 2
    assert mitred.counter("CONVEYOR_FAN_SUPPORTS") == 0
    assert prepared.counter("CONVEYOR_WEIGHTED_SOURCE_EDGES") == 3
    assert prepared.counter("CONVEYOR_RESCALED_ARRIVAL_LAWS") == 5

    def symbolic(value):
        return sum(
            sp.Rational(coefficient.numerator, coefficient.denominator)
            * sp.sqrt(radicand)
            for radicand, coefficient in value.terms
        )

    shortfall = sp.simplify(
        sp.radsimp(
            symbolic(mitred_covered.doubled_area) - symbolic(covered.doubled_area)
        )
    )
    # Недобор положителен: цепь накрывает МЕНЬШЕ митра, как и обязана.
    assert sp.N(shortfall, 20) > 0
    assert abs(
        sp.N(shortfall, 20) - sp.Float("8384741.4333844053")
    ) < sp.Float("1e-4")

    alpha = sp.Rational(
        covered.lattice_alpha.numerator, covered.lattice_alpha.denominator
    )
    assert alpha == 8192
    assert prepared.counter("CONVEYOR_LATTICE_SCALE") == 32768
    # Ни одно целое число вееров замкнутую форму здесь не воспроизводит.
    for count in (1, 2, 3, 4):
        predicted = count * alpha**2 * (6 - 4 * sp.sqrt(2))
        assert sp.simplify(sp.radsimp(shortfall - predicted)).is_zero is not True
    predicted_two = 2 * alpha**2 * (6 - 4 * sp.sqrt(2))
    assert abs(
        sp.N(shortfall / predicted_two, 12) - sp.Float("0.182054394500")
    ) < sp.Float("1e-10")

    # Решающая улика: в недоборе живут радикалы САМОГО домена, а не только sqrt(2).
    radicands = {radicand for radicand, _ in covered.doubled_area.terms} | {
        radicand for radicand, _ in mitred_covered.doubled_area.terms
    }
    assert 27851087972573 in radicands
    assert 174086333192405 in radicands
    assert 2 not in radicands


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
