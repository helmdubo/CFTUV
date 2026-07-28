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
| нерациональный веер — громкий отказ, а не откат к митру       | `..._an_irrational_fan_refuses_the_whole_domain` |
| полевые фикстуры вееров НЕ содержат, и это измерено           | `..._the_field_fixtures_declare_no_corner_relation_at_all` |

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
        assert fans.irrational_fan_count == 0, hidden_count
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


def test_an_irrational_fan_refuses_the_whole_domain_and_counts_the_vertices():
    """Нерациональное направление — ОТКАЗ уровня домена, а не откат к митру.

    Проверяется подменой рецепта направлений на заведомо алгебраическое: если
    когда-нибудь появится тихая ветка «не вышло — посчитаем острый угол», этот
    тест увидит `EXACT` там, где обязано стоять имя.
    """

    import sympy as sp

    from cftuv_envelope.reference.planar_types import ExactPlanarVector
    from cftuv_envelope.wavefront import conveyor as module

    from reference_factories import angular_snapshot

    snapshot, request = angular_snapshot(1)
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

    module.angular_hidden_support_lines = algebraic
    try:
        prepared = module.prepare_conveyor(snapshot, request)
    finally:
        module.angular_hidden_support_lines = original
    assert (
        prepared.outcome
        is module.ConveyorOutcome.ANGULAR_PROFILE_DIRECTION_IS_NOT_RATIONAL
    )
    assert prepared.detail.startswith("1 вогнутых вершин из 1")
    assert prepared.counter("CONVEYOR_IRRATIONAL_VERTEX_FANS") == 1
    assert prepared.counter("CONVEYOR_RATIONAL_VERTEX_FANS") == 0


def test_the_field_fixtures_declare_no_corner_relation_at_all():
    """ИЗМЕРЕННАЯ находка: полевые фикстуры вееров не содержат вовсе.

    Карточка среза объявляла сильнейшим критерием равенство с эталоном на `bf6`
    и предупреждала, что прежние митрованные числа `bf6` СДВИНУТСЯ. Они не
    сдвинулись, и причина не в очереди: снапшот `building.002` не объявляет НИ
    ОДНОГО `CornerRelation`, поэтому компилятор не строит ни одной
    `AngularEnvelopeSpec`, веера в плане нет, и оба пути — эталон и очередь —
    считают ту же митрованную точку семейства, что и до среза.

    Критерий, стало быть, ДОСТИЖИМ, но пуст: он проверяет равенство двух путей,
    в которое веер не входит. Это не повод его снимать (равенство само по себе
    ценно и проверяется в `test_wavefront_conveyor.py`), но повод записать
    числом, чего именно на полевом входе нет. Мягкий угол на `building.002`
    появится тогда, когда хост начнёт объявлять вогнутые углы домена, — и это
    работа адаптера хоста, а не очереди.
    """

    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.envelopes import AngularEnvelopeSpec
    from cftuv_envelope.reference.compile import compile_reference_envelopes

    from wavefront_cases import FIELD_FIXTURE

    root = FIELD_FIXTURE.parent
    for name in (
        "building_002_point_contact_v1",
        "building_002_weighted_normals_v1",
    ):
        folder = root.parent / name
        snapshot = kernel.AnalysisSnapshotCodecV1.loads(
            (folder / "analysis_snapshot.json").read_bytes()
        )
        request = kernel.DecalRequestCodecV1.loads(
            (folder / "decal_request.json").read_bytes()
        )
        assert snapshot.corner_relations == frozenset(), name
        assert snapshot.angular_owner_sectors == frozenset(), name
        compilation = compile_reference_envelopes(snapshot, request).compilation
        assert not [
            spec
            for spec in compilation.envelope_specs
            if isinstance(spec, AngularEnvelopeSpec)
        ], name


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
