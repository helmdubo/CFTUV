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
| полное выделение: домен `EXACT` при 3 веерах и 1 деградации   | `..._the_full_selection_domain_builds_with_one_degraded_corner` |
| цена деградации против эталона — `+0.09225 %`, а не ноль      | `..._the_degraded_corner_costs_a_measured_excess_over_the_reference_path` |

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


# Сценарий владельца: тот же патч `bf6`, но выбраны ВСЕ 12 цепочек шва. Домен
# объявляет четыре вогнутых угла, и ровно один из них не прямой.
FULL_SELECTION = "building_002_full_selection_v1"
DEGRADED_SPEC = "angular-spec:66cf5e6f75ddafed7cdb3dca"
DEGRADED_RELATION = "host-v0:corner-relation:a2b66a4ed578c532315cd09b"
DEGRADED_SUPPORT = "hidden-support:36a61d30ef0657696ba6f3a0"
# Площадь домена того же патча, замороженная `test_wavefront_conveyor.py` на
# фикстуре `point_contact`. Здесь она обязана СОВПАСТЬ: выделение запроса другое,
# а домен тот же, и это отделяет «переписали фикстуру» от «выбрали больше швов».
BF6_DOUBLED_AREA = 27224141715


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
    from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec
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


def test_the_full_selection_domain_builds_with_one_degraded_corner():
    """Сценарий владельца: домен СТРОИТСЯ, и одна вершина названа острой.

    ЧТО БЫЛО. На этих же байтах `prepare_conveyor` отвечал
    `ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL` с деталью «1 вогнутых вершин из 4
    с веером»: одна вершина уносила с собой три РАЦИОНАЛЬНЫХ веера и весь домен,
    и владелец не получал ничего. Решение владельца отменило цену, не отменяя
    громкости: вершина остаётся митрованной, но названной.

    ПОЧЕМУ ДЕГРАДИРОВАЛА ИМЕННО ЭТА ВЕРШИНА — проверено НЕ машинерией веера, а
    независимой величиной снапшота, сертификатом угла:

    | угол | `phi/pi` сертификата | судьба веера |
    |---|---|---|
    | `739252a7…`, `7dde3e10…`, `fac6c5eb…` | ровно `1.5`, оба конца | рациональный веер |
    | `66cf5e6f…` | `[1.500123280352543932868625829, 1.50012328035254393286862583]` | ДЕГРАДИРОВАЛ в митр |

    У деградировавшей вершины сертификат — не точка, а оболочка (концы округлены
    НАРУЖУ кодеком хоста), и это не мешает утверждению: вся оболочка лежит СТРОГО
    выше `1.5`, то есть «угол не прямой» доказано, а не следует из ширины записи.
    У трёх остальных оба конца совпадают и равны `1.5` ровно.

    Читается это так: при `phi = 3pi/2` обе скрытые опоры несут ОДИН радикал
    (`sqrt(2)`), и биссектриса после рескейла целочисленна; отклонение угла на
    `1.2e-4` от прямого даёт вложенный радикал
    `sqrt(1333345 - sqrt(266669))`, который записью в точных дробях не выражается
    ни при каком множителе. То есть деградация — свойство УГЛА, а не сбой ступени,
    и предсказывается она полем, которое очередь не считает.

    ЧТО ДЕГРАДАЦИЯ ЕСТЬ РОВНО МИТР, проверено вторым входом на тех же байтах: со
    снятым угловым отношением этой вершины (веер не требуется вовсе) очередь даёт
    ПОБИТОВО тот же скелет и то же покрытие. Совпадение исключает третий ответ —
    например, веер с пустыми опорами или сдвинутый якорь.

    ЧИСЛА ДОМЕНА, измеренные этим прогоном:

    | величина | число |
    |---|---:|
    | законов прихода / спасено масштабом | 12 / 4 |
    | вееров рациональных / деградировавших | **3 / 1** |
    | рёбер домена / источников / стен | 12 / 12 / **0** |
    | неопределённых владельцев | 4 |
    | узлов скелета / граней | 13 / 15 |
    | удвоенная площадь домена | 27 224 141 715 |
    | членов покрытия при `alpha = 1/4` | 26 |

    Ноль стен здесь не совпадение и не свойство меша: выбраны все 12 цепочек, то
    есть источником служит КАЖДОЕ ребро домена. У `point_contact` того же патча
    источников 3 и стен 9 — разница ровно в выделении.
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
        "CONVEYOR_RESCALED_ARRIVAL_LAWS": 4,
        # 3 + 1 + 0 = 4 угловые спеки, поимённо: веер посеян, веер деградировал,
        # профиль выбрал `k = 0`. Суммы «4» было бы мало — она держалась бы и при
        # четырёх деградациях.
        "CONVEYOR_RATIONAL_VERTEX_FANS": 3,
        "CONVEYOR_DEGRADED_MITER_CORNERS": 1,
        "CONVEYOR_MITERED_CORNERS": 0,
        "CONVEYOR_FAN_SUPPORTS": 3,
        "CONVEYOR_FAN_EDGES": 3,
        "CONVEYOR_LATTICE_SCALE": 32768,
        "CONVEYOR_DOMAIN_EDGES": 12,
        "CONVEYOR_SOURCE_EDGES": 12,
        "CONVEYOR_WALL_EDGES": 0,
        "CONVEYOR_AMBIGUOUS_OWNER_EDGES": 4,
        "CONVEYOR_WEIGHTED_SOURCE_EDGES": 12,
        "CONVEYOR_SKELETON_NODES": 13,
        "CONVEYOR_FACES": 15,
    }

    (region,) = prepared.regions
    assert region.bridge_outcome is BridgeOutcome.EXACT
    assert region.bridge.findings == ()
    assert region.skeleton_outcome is SkeletonOutcome.EXACT
    assert region.face_outcome is FaceOutcome.EXACT
    assert len(region.skeleton.nodes) == 13
    assert sorted(face.node_count for face in region.partition.faces) == [
        1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 5, 5, 7,
    ]
    # Сумма граней есть площадь домена ТОЧНО: дефект — пустой набор членов, а не
    # «маленькое число». И площадь та же, что у `point_contact`: домен один.
    assert region.partition.polygon_doubled_area == BF6_DOUBLED_AREA
    assert region.partition.area_defect.terms == ()
    assert region.wall_spans == ()
    assert len(region.ambiguous_owner_spans) == 4

    # Деградация названа НА ВЕРШИНЕ: спека, угловое отношение, якорь и причина.
    (corner,) = region.degraded_miter_corners
    assert corner.envelope_spec_id == DEGRADED_SPEC
    assert corner.corner_relation_id == DEGRADED_RELATION
    assert corner.anchor == (
        Fraction(169074147, 11408680),
        Fraction(-22588627, 5704340),
    )
    assert corner.reason == (
        f"{DEGRADED_SUPPORT}: запись закона не рациональна и после масштаба"
    )

    _assert_the_degraded_corner_is_the_only_one_off_the_right_angle(snapshot)

    covered = conveyor_coverage(prepared)
    assert covered.outcome is ConveyorOutcome.EXACT, covered.detail
    assert covered.alpha == Fraction(1, 4)
    assert covered.lattice_alpha == Fraction(8192)
    assert len(covered.doubled_area.terms) == 26
    low, high = covered.doubled_area.enclosure(64)
    assert 4604827284 < low <= high < 4604827285
    assert covered.polygon_doubled_area == BF6_DOUBLED_AREA
    # 15 граней = 8 названных владельцев + 4 неопределённых + 3 веера. Имя
    # экземпляра у веера не выводится (открытый счёт, `DECISIONS.md`), и здесь
    # это ЧИСЛО, а не умолчание.
    assert covered.counter("CONVEYOR_COVERED_FACES") == 15
    assert covered.counter("CONVEYOR_NAMED_OWNERS") == 8
    assert (
        sum(1 for face in covered.faces if face.envelope_instance_id is None) == 7
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
    (trimmed_region,) = trimmed.regions
    assert semantic_digest(trimmed_region.skeleton) == semantic_digest(
        region.skeleton
    )
    assert (
        conveyor_coverage(trimmed).doubled_area.terms
        == covered.doubled_area.terms
    )


def _assert_the_degraded_corner_is_the_only_one_off_the_right_angle(snapshot):
    """Судьба веера предсказана сертификатом угла, а не самой машинерией веера.

    Проверка стоит отдельной функцией не ради длины: она утверждает про ВХОД, а не
    про ответ очереди, и потому обязана читаться независимо от того, что очередь
    посчитала.
    """

    from decimal import Decimal

    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec

    compilation = kernel.compile_reference_envelopes(
        snapshot, _full_selection_input()[1]
    ).compilation
    by_certificate = {
        item.certificate_id: item for item in snapshot.reflex_angle_certificates
    }
    seen = {}
    for spec in compilation.envelope_specs:
        if not isinstance(spec, AngularEnvelopeSpec):
            continue
        seen[spec.envelope_spec_id.value] = by_certificate[
            spec.angle_certificate_id
        ].measure_payload.phi_over_pi
    assert len(seen) == 4
    degraded = seen.pop(DEGRADED_SPEC)
    # ВСЯ оболочка деградировавшего угла лежит строго выше прямого: утверждение
    # «не прямой» стоит на нижнем конце, а не на середине записи.
    assert degraded.lower > Decimal("1.5")
    assert str(degraded.lower) == "1.500123280352543932868625829"
    assert str(degraded.upper) == "1.50012328035254393286862583"
    # У трёх рациональных вееров сертификат — ТОЧКА: оба конца равны `1.5`, то
    # есть угол прямой доказанно, а не в пределах округления.
    for name, interval in seen.items():
        assert str(interval.lower) == str(interval.upper) == "1.5", name


def test_the_degraded_corner_costs_a_measured_excess_over_the_reference_path():
    """ГИПОТЕЗА КАРТОЧКИ «разность близка к нулю» ОПРОВЕРГНУТА, и числом.

    Ожидание было такое: у `point_contact` веера есть у ОБОИХ путей, и остаток
    там `+0.00026651 %`; на `full_selection` стен нет вовсе (источником служат все
    12 рёбер), значит объявленная разница моделей у стен исчезает и разность
    обязана упасть ещё ниже. Прогон отвечает иначе:

    | вход | стен | остаток очереди над эталоном |
    |---|---:|---:|
    | `point_contact` (2 веера у обоих путей) | 9 | `+0.00026651 %` |
    | `full_selection` (3 веера у очереди, 4 у эталона) | **0** | **`+0.09225 %`** |

    Разность выросла в 346 раз ПРИ ИСЧЕЗНОВЕНИИ стен, то есть объяснить её
    моделью стены нельзя по построению: `CONVEYOR_WALL_EDGES` здесь ноль, и это
    проверено здесь же. Остаётся ровно одна названная причина — деградировавшая
    вершина: очередь считает её МИТРОМ, эталон — цепью `k = 1`, а митр накрывает
    больше цепи. Знак это подтверждает: избыток, а не недобор.

    ЧТО ПРИЧИНА ИМЕННО В ЭТОЙ ВЕРШИНЕ, измерено третьим прогоном (в гейт он не
    включён — эталонный путь на этих байтах стоит 74 с, и второй такой же удвоил
    бы цену ради знака, который уже доказан): со снятым угловым отношением
    деградировавшей вершины эталон теряет свою угловую заплату и опускается до
    `2.1327932877294125741`, тогда как ОЧЕРЕДЬ не двигается ни на бит
    (`2.1442898010911475551`, точная сверка членов — в соседнем тесте). Из трёх
    чисел следует разложение: заплата цепи стоит `+0.0095202`, митр очереди
    `+0.0114965`, разница между ними `+0.0019763` — ровно измеренный здесь
    остаток.

    Число заморожено затем, чтобы следующая ступень — сертифицированная привязка
    направлений — могла показать, что она его УБРАЛА, а не «улучшила». Цена
    заморозки названа, а не спрятана: тест стоит 78 с из 448 с всего гейта ядра
    (было 347 с), и почти всё это — эталонный союз шестнадцати юбок (`74 с`);
    очередь на тех же байтах отвечает за `0.9 с`.
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
    assert prepared.counter("CONVEYOR_DEGRADED_MITER_CORNERS") == 1

    scale = prepared.lattice.scale
    queue = sum(
        (
            sp.Rational(coefficient.numerator, coefficient.denominator)
            * sp.sqrt(radicand)
            for radicand, coefficient in covered.doubled_area.terms
        ),
        sp.Integer(0),
    ) / (2 * scale * scale)
    evaluated = kernel.evaluate_reference_raw_coverage(
        kernel.compile_reference_envelopes(snapshot, request).compilation,
        request.requested_alpha,
    )
    reference = sp.sympify(evaluated.raw_coverage.exact_area_expression)
    excess = sp.simplify(sp.radsimp(queue - reference))
    # Разность НЕ ноль — и это доказанное «не ноль», а не «не сошлось численно».
    assert excess.is_zero is not True
    assert sp.N(excess, 20) > 0
    assert abs(
        sp.N(excess / reference * 100, 8) - sp.Float("0.092249631")
    ) < sp.Float("1e-9")


def test_the_closed_form_shortfall_does_not_apply_to_the_field_domain():
    """ГИПОТЕЗА ОПРОВЕРГНУТА: `alpha^2*(3-2*sqrt(2))` на `bf6` не выполняется.

    Ожидание было такое: у двух вееров ПРЯМЫХ углов недобор цепи против митра
    известен замкнутой формой, значит на `bf6` расхождение митра и цепи обязано
    объясняться ровно двумя веерами — `2*alpha^2*(6-4*sqrt(2))` в удвоенной
    площади при `alpha = 8192` на решётке `32768`. Прогон это опровергает:

    | величина | число |
    |---|---:|
    | измеренный недобор (удвоенная площадь) | `8 465 025.6685` |
    | замкнутая форма для двух прямых углов  | `805306368 - 536870912*sqrt(2)` = `46 056 243.0060` |
    | отношение измеренного к предсказанному | `0.1838` |

    ПОСЫЛКА ПРО УГОЛ ПРИ ЭТОМ ВЕРНА, и это проверено здесь же: оба сертификата,
    попавшие в веера, объявляют `phi/pi` РОВНО `1.5`, то есть угол 3pi/2 точно, а
    не приближённо. Ломается не угол — ломаются две другие посылки замкнутой
    формы, и обе названы числом:

    1. **карта не ортонормальна.** Матрица Грама патча далека от единичной
       (`m00 = 2066105/262144`, `m01 = 13024865/524288`), поэтому прямой угол
       ИСТОЧНИКА прямым углом в координатах карты не является, а `alpha` в
       единицах решётки не есть евклидов радиус в карте;
    2. **скорости фронтов не единичные** — три взвешенных ребра-источника и два
       пере-масштабированных закона.

    Решающая улика — РАДИКАЛЫ. Замкнутая форма даёт величину вида `a + b*sqrt(2)`;
    измеренный недобор несёт собственные радикалы домена (`sqrt(711394508186)`,
    `sqrt(341925762763390682)` и далее), то есть принадлежит другому полю чисел
    вовсе, и никаким выбором `r` в `r*alpha^2*(6-4*sqrt(2))` получен быть не
    может. Проверено перебором `r` от 1 до 4.

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
    assert prepared.counter("CONVEYOR_RESCALED_ARRIVAL_LAWS") == 2

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
    assert abs(sp.N(shortfall, 20) - sp.Float("8465025.6685360800")) < sp.Float("1e-4")

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
        sp.N(shortfall / predicted_two, 12) - sp.Float("0.183797572621")
    ) < sp.Float("1e-10")

    # Решающая улика: в недоборе живут радикалы САМОГО домена, а не только sqrt(2).
    radicands = {radicand for radicand, _ in covered.doubled_area.terms} | {
        radicand for radicand, _ in mitred_covered.doubled_area.terms
    }
    assert 711394508186 in radicands
    assert 341925762763390682 in radicands
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
