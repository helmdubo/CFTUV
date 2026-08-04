"""Дифференциальная сверка: очередь событий против попарного кроя, один вход.

Срез НИЧЕГО не переключает. Здесь только измерение: где два пути дают один
ответ, где разный, и чем именно.

Главный вопрос среза — про полевой патч `bf6`
(`kernel/fixtures/building_002_point_contact_v1`), на котором попарный крой
падает с `INTERACTION_POLICY_B_PARTITION_UNPROVEN`. Ответ разложен на два теста,
и они отвечают на разное:

- `test_the_field_patch_input_does_not_map_onto_the_queue_and_here_is_why`
  — вход `bf6` в очередь НЕ отображается, и причин ДВЕ, каждая с числом. Третья
  («источник — не вся граница») ушла вместе со срезом частичного источника: она
  была границей очереди, а не свойством входа, и девять рёбер из двенадцати
  теперь размечаются стенами;
- `test_the_field_patch_pairwise_clipping_destroys_all_of_its_own_coverage`
  — что именно там ломается у попарного кроя: он теряет ВСЮ площадь.

Отображаемая часть корпуса проверяется отдельно, и там ответ есть:
`test_the_queue_reproduces_the_raw_coverage_where_the_clipper_cannot`.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
from fractions import Fraction

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope.interactions import policy_b as policy_b_module
from cftuv_envelope.interactions.arrival import compile_arrival_models
from cftuv_envelope.interactions.components import compile_interaction_components
from cftuv_envelope.interactions.contracts import InteractionOutcome
from cftuv_envelope.reference.arrangement import bound_evaluation_arrangement
from cftuv_envelope.reference.boundary import build_domain_geometry
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.planar_types import (
    ExactScalar,
    exact_quadratic_value,
    exact_sign,
)
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)
from cftuv_envelope.robust.grid import GridSpecV1
from cftuv_envelope.wavefront import build_skeleton, chart_lattice_for_frame
from cftuv_envelope.wavefront.bridge import (
    BridgeOutcome,
    PlainArrivalLawV1,
    VertexFanLawV1,
    _proportional_positive,
    _rational_edge_lines,
    bridge_arrival_laws,
    line_class,
    unit_speed_laws_of,
)
from cftuv_envelope.wavefront.conveyor import _arrival_laws
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.events import EventKind
from cftuv_envelope.wavefront.faces import (
    FaceOutcome,
    build_faces,
    contour_crossings,
)
from cftuv_envelope.wavefront.polygon import (
    PolygonV1,
    unit_speed_squared,
    with_edge_speeds,
)
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, SplitSearch
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from reference_factories import straight_snapshot
from wavefront_cases import (
    FIELD_FIXTURE,
    cross,
    field_fixture_digest,
    holes_grid,
    named_corpus,
)


FIXTURE = FIELD_FIXTURE.parent


# --------------------------------------------------------------------------
# 1. Мост входа: обратное направление обязано замыкаться
# --------------------------------------------------------------------------


# Мост берёт корпус ЦЕЛИКОМ, а не только фигуры с различными несущими
# прямыми. Отбор существовал, пока сопоставление закона с ребром шло
# `next(...)` по первой попавшейся прямой: коллинеарные сонаправленные рёбра
# схлопывались в один индекс, и мост отказывал на верном входе. Теперь
# сопоставление — биекция по классам прямых, поэтому `holes_2` и крест
# проходят наравне со всеми, и это ровно та проверка, которую отбор скрывал.
#
# Крест `6 x 4` лежит уже в самом корпусе (`named_corpus()`), поэтому здесь
# добавляется только квадратный блок: у него гребни приходят в точку
# одновременно, и мост обязан принять его так же.
CORPUS = named_corpus() + (("cross_4x4", cross(wide=4, tall=4)),)
CORPUS_IDS = tuple(name for name, _ in CORPUS)


@pytest.mark.parametrize("name,polygon", CORPUS, ids=CORPUS_IDS)
def test_the_bridge_accepts_the_laws_it_itself_produced(name, polygon):
    """Объявленная граница моста на его собственном результате.

    `unit_speed_laws_of` выдаёт законы прихода для КАЖДОГО ребра с единичной
    скоростью, то есть ровно то, что очередь у себя и видит. Если после этого
    мост находит расхождение, расходится он сам с собой.
    """

    loops = tuple(
        tuple((Fraction(x), Fraction(y)) for x, y in loop.points)
        for loop in polygon.loops
    )
    report = bridge_arrival_laws(loops, unit_speed_laws_of(polygon))
    assert report.outcome is BridgeOutcome.EXACT, report.findings
    assert report.maps
    assert report.matched_edge_count == report.edge_count
    assert report.law_count == report.edge_count
    assert report.non_unit_speed_laws == ()
    assert report.off_lattice_points == ()
    assert report.polygon is not None
    assert report.polygon.vertex_count == polygon.vertex_count


def _loops_of(polygon):
    return tuple(
        tuple((Fraction(x), Fraction(y)) for x, y in loop.points)
        for loop in polygon.loops
    )


def _unit_laws_for(loops) -> tuple[PlainArrivalLawV1, ...]:
    """Закон единичной скорости на КАЖДОЕ ребро дробного домена.

    `unit_speed_laws_of` требует уже построенного `PolygonV1`, то есть домена
    на решётке, — а тесты привязки живут именно там, где домена на решётке ещё
    нет. Скорость берётся `s^2 = |n|^2`, то есть ровно единичной, поэтому
    находка про скорость в таких тестах не срабатывает и не мешает читать ту,
    ради которой тест написан.
    """

    return tuple(
        PlainArrivalLawV1(
            name=f"line{index}",
            normal_x=a,
            normal_y=b,
            constant=c,
            speed_squared=a * a + b * b,
        )
        for index, (a, b, c) in enumerate(_rational_edge_lines(loops))
    )


@pytest.mark.parametrize(
    "name,polygon,edges,classes",
    (
        ("cross_6x4", cross(wide=6, tall=4), 12, 8),
        ("cross_4x4", cross(wide=4, tall=4), 12, 8),
        ("holes_2", holes_grid(1, 2), 12, 10),
        ("holes_2x2", holes_grid(2, 2), 20, 12),
    ),
)
def test_two_collinear_edges_are_two_sources_and_not_one(
    name, polygon, edges, classes
):
    """Источником является КАЖДОЕ ребро, и мост обязан это увидеть.

    Прежнее сопоставление искало ребро для закона `next(...)` по первой
    пропорциональной несущей прямой, поэтому два коллинеарных сонаправленных
    ребра получали ОДИН индекс, счёт недосчитывался, и мост отказывал на входе,
    где источником было всё:

    | фигура | рёбер | классов прямых | сопоставлено было | стало |
    |---|---:|---:|---:|---:|
    | `cross(6, 4)` | 12 | 8 | 8 | 12 |
    | `cross(4, 4)` | 12 | 8 | 8 | 12 |
    | `holes_2` | 12 | 10 | 10 | 12 |
    | `holes_2x2` | 20 | 12 | 12 | 20 |

    Число классов проверяется здесь же, а не подразумевается: без него
    «стало 12» ничем не отличалось бы от «фигура и так была чистой».
    """

    loops = _loops_of(polygon)
    laws = unit_speed_laws_of(polygon)
    report = bridge_arrival_laws(loops, laws)
    assert report.edge_count == edges
    assert report.law_count == edges
    assert len({line_class(line) for line in _rational_edge_lines(loops)}) == classes
    assert report.matched_edge_count == edges
    assert report.findings == (), name
    assert report.outcome is BridgeOutcome.EXACT


def test_the_line_class_is_exactly_the_old_proportionality_predicate():
    """Группировка по классу обязана совпасть с прежним предикатом ПОТОЧЕЧНО.

    Замена поиска `next(...)` на словарь законна ровно потому, что равенство
    классов и `_proportional_positive` — одно и то же отношение. Это не
    рассуждение: здесь сверяются ВСЕ пары прямых корпуса, включая пары из
    разных фигур, то есть заведомо непропорциональные.

    Прямых стало 203 против прежних 185, и +18 — это ровно цена ПОЧИНЕННОЙ
    транскрипции полевого контура (`wavefront_cases.FIELD_NUMERATORS`): прежняя
    запись держала 6 различных вершин вместо 12, и три полевые фигуры корпуса
    (`scale_64/256/1024`) давали по 6 рёбер вместо 12. Число проверяется не
    собой: 3 фигуры x (12 - 6) = 18, и площадь починенного контура сходится с
    площадью самого домена — сверка записана в `wavefront_cases.py` и стоит
    исполняемой в `test_the_transcribed_field_contour_is_the_fixtures_own`.
    """

    lines = []
    for _, polygon in CORPUS:
        lines.extend(_rational_edge_lines(_loops_of(polygon)))
    assert len(lines) == 203
    checked = 0
    for left in lines:
        for right in lines:
            checked += 1
            same_class = line_class(left) == line_class(right)
            assert same_class is _proportional_positive(left, right), (
                left,
                right,
            )
    assert checked == len(lines) * len(lines)


def test_ownership_inside_one_support_line_is_named_undetermined_not_guessed():
    """Закон не несёт протяжённости, значит «чьё это ребро» во входе нет.

    Это и есть измеренный ответ на вопрос «чем различать коллинеарные
    сонаправленные рёбра»: проекциями концов — нельзя, потому что у
    `PlainArrivalLawV1` концов НЕТ ни в каком виде. Поля закона — имя, нормаль,
    константа и квадрат скорости, и это проверяется здесь же.

    Поэтому мост считает, СКОЛЬКО рёбер имеют источник (это точный размер
    наибольшего паросочетания), и называет те рёбра, у которых владелец не
    определён, вместо того чтобы поставить им первого попавшегося.
    """

    assert PlainArrivalLawV1.__slots__ == (
        "name",
        "normal_x",
        "normal_y",
        "constant",
        "speed_squared",
    )

    polygon = cross(wide=6, tall=4)
    report = bridge_arrival_laws(
        _loops_of(polygon), unit_speed_laws_of(polygon)
    )
    # Четыре класса из восьми несут по два ребра, значит восемь рёбер
    # неопределённых и четыре с вынужденным владельцем.
    assert len(report.ambiguous_owner_spans) == 8
    assert len(report.owner_by_edge) == 4
    assert len(report.ambiguous_owner_spans) + len(report.owner_by_edge) == 12
    # Ключ владельца — ВХОЖДЕНИЕ ребра, а не `(a, b, c, q)`: у `holes_2` два
    # коллинеарных ребра одной длины давали один ключ, и запись пропадала.
    for span, _ in report.owner_by_edge:
        assert len(span) == 4
    assert len(set(report.ambiguous_owner_spans)) == 8


def test_a_surplus_law_on_one_line_is_named_instead_of_overwriting_silently():
    """Два закона на одну прямую при одном ребре — ИМЕНОВАННЫЙ исход.

    До биекции такой закон исчезал молча: `next(...)` возвращал обоим один
    индекс, второй затирал первого в карте владельцев, а в `unmatched_laws` он
    не попадал — прямая-то домену принадлежит. Отрицательный контроль на
    «тихое исчезновение»: счёт сопоставленных рёбер при этом остаётся 4 из 4,
    то есть по нему подмену было НЕ ВИДНО.
    """

    from dataclasses import replace

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    laws = unit_speed_laws_of(polygon)
    doubled = laws + (replace(laws[0], name="duplicate-of-edge-0"),)
    report = bridge_arrival_laws(_loops_of(polygon), doubled)
    assert report.outcome is BridgeOutcome.MORE_ARRIVAL_LAWS_THAN_EDGES_ON_ONE_LINE
    assert report.surplus_laws == ("duplicate-of-edge-0",)
    assert report.unmatched_laws == ()
    assert (report.matched_edge_count, report.edge_count) == (4, 4)
    assert report.law_count == 5


def test_a_non_unit_speed_law_is_named_and_measured_not_rounded():
    """Скорость чуть-чуть не единичная — всё равно ИМЕНОВАННЫЙ отказ.

    Порогов нет: `s^2 = q + 1` вместо `q` уже расхождение, потому что
    сравнение рациональное, а не «в пределах».
    """

    from dataclasses import replace

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    loops = ((
        (Fraction(0), Fraction(0)),
        (Fraction(8), Fraction(0)),
        (Fraction(8), Fraction(8)),
        (Fraction(0), Fraction(8)),
    ),)
    laws = list(unit_speed_laws_of(polygon))
    laws[0] = replace(laws[0], speed_squared=laws[0].speed_squared + 1)
    report = bridge_arrival_laws(loops, tuple(laws))
    assert report.outcome is BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED
    assert report.polygon is None
    assert len(report.non_unit_speed_laws) == 1
    assert report.non_unit_speed_laws[0][1] == Fraction(65, 64)


def test_a_source_that_is_not_the_whole_boundary_becomes_walls_not_a_refusal():
    """Ребро без закона — СТЕНА, и вход отображается, а не отвергается.

    До среза здесь стоял отказ `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY`: очередь
    пускала фронт от каждого ребра, и часть границы без закона выражать было
    нечем. Теперь у ребра есть своё `q`, ноль означает неподвижную прямую, и
    мост ставит его сам. Проверяется не только исход: разметка обязана лечь на
    ТЕ ЖЕ рёбра, у которых закон, — иначе стены встали бы верным числом, но не
    на своё место.
    """

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    loops = ((
        (Fraction(0), Fraction(0)),
        (Fraction(8), Fraction(0)),
        (Fraction(8), Fraction(8)),
        (Fraction(0), Fraction(8)),
    ),)
    laws = unit_speed_laws_of(polygon)[:2]
    report = bridge_arrival_laws(loops, laws)
    assert report.outcome is BridgeOutcome.EXACT
    assert report.findings == ()
    assert (report.matched_edge_count, report.edge_count) == (2, 4)
    assert report.wall_edge_count == 2
    assert report.undetermined_source_count == 0
    assert report.polygon is not None
    assert report.polygon.source_edge_count == 2
    assert report.wall_spans == ((0, 8, 0, 0), (8, 8, 0, 8))
    sources = {
        frozenset((start, end))
        for start, end, speed in report.polygon.edges()
        if speed > 0
    }
    assert sources == {
        frozenset(((0, 0), (8, 0))),
        frozenset(((8, 0), (8, 8))),
    }


def test_the_bridge_closes_on_a_partially_marked_polygon_in_both_directions():
    """Мост туда-обратно сохраняет РАЗМЕТКУ, а не только форму.

    `unit_speed_laws_of` выдаёт законы только рёбрам-источникам — у стены закона
    нет по определению, — а `bridge_arrival_laws` обратно превращает рёбра без
    закона в стены. Значит замыкание проверяемо, и проверяется оно на
    подмножестве, а не на полной границе, где стен нет вовсе.
    """

    from wavefront_cases import partial_source_corpus

    checked = 0
    for name, polygon in partial_source_corpus():
        loops = _loops_of(polygon)
        report = bridge_arrival_laws(loops, unit_speed_laws_of(polygon))
        if report.findings:
            # Взвешенное ребро мост обязан назвать неединичным — и называет.
            assert report.findings == (
                BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,
            ), name
            continue
        assert report.polygon is not None, name
        assert report.wall_edge_count == polygon.wall_edge_count, name
        assert {
            frozenset((start, end))
            for start, end, speed in report.polygon.edges()
            if speed > 0
        } == {
            frozenset((start, end))
            for start, end, speed in polygon.edges()
            if speed > 0
        }, name
        checked += 1
    assert checked == 37


def test_a_partially_covered_line_class_leaves_the_source_undetermined():
    """Два ребра на одной прямой и ОДИН закон: какое из них источник — неизвестно.

    Придуманная пометка выглядела бы как знание, которого во входе нет, и цена
    ошибки здесь полная: стена и источник — разная геометрия, а не разный вес.
    Поэтому вход отвергается по имени, а не размечается наугад.
    """

    polygon = holes_grid(1, 2)
    loops = _loops_of(polygon)
    laws = unit_speed_laws_of(polygon)
    shared = [
        law
        for law in laws
        if line_class((law.normal_x, law.normal_y, law.constant))
        == line_class(_rational_edge_lines(loops)[4])
    ]
    assert len(shared) == 2, "у holes_2 есть класс ровно из двух рёбер"
    trimmed = tuple(law for law in laws if law is not shared[0])
    report = bridge_arrival_laws(loops, trimmed)
    assert (
        BridgeOutcome.SOURCE_EDGE_INSIDE_A_LINE_CLASS_IS_UNDETERMINED
        in report.findings
    )
    assert report.undetermined_source_count == 2
    assert report.polygon is None


def test_a_lattice_on_a_domain_already_in_the_nodes_changes_nothing():
    """Отрицательный контроль привязки: где двигать нечего, она не двигает.

    Без него «привязка отобразила `bf6`» не отличалось бы от «привязка меняет
    ответ вообще везде». Берётся весь корпус — фигуры и так на решётке, — и
    требуется ПОБИТОВОЕ совпадение полигона и нулевая невязка. Ноль здесь
    отличается от `None`: привязка была и ничего не сдвинула.
    """

    for name, polygon in CORPUS:
        loops = _loops_of(polygon)
        laws = unit_speed_laws_of(polygon)
        without = bridge_arrival_laws(loops, laws)
        snapped = bridge_arrival_laws(loops, laws, lattice=GridSpecV1(scale=1))
        assert snapped.outcome is BridgeOutcome.EXACT, name
        assert snapped.snap_residual == 0, name
        assert without.snap_residual is None, name
        assert snapped.polygon == without.polygon, name
        assert snapped.weighted_edge_count == 0, name


def test_a_weighted_law_needs_the_caller_to_say_so_and_then_maps():
    """Взвешенность включается вызывающим, и обе стороны переключателя видны.

    Без параметра неединичный закон — находка, ровно как был. С параметром
    находки нет, а `q` ребра равно `(s/|n|)^2 * |d|^2`, то есть ЧЕТВЕРТИ
    квадрата длины при скорости вдвое меньшей. Число проверяется целиком, а не
    «отличается от `|d|^2`»: половинная скорость и нулевая тоже отличаются.
    """

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    loops = _loops_of(polygon)
    laws = tuple(
        PlainArrivalLawV1(
            name=law.name,
            normal_x=law.normal_x,
            normal_y=law.normal_y,
            constant=law.constant,
            speed_squared=law.speed_squared / 4,
        )
        for law in unit_speed_laws_of(polygon)
    )
    refused = bridge_arrival_laws(loops, laws)
    assert refused.findings == (BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,)

    mapped = bridge_arrival_laws(loops, laws, weighted_fronts=True)
    assert mapped.maps
    assert mapped.weighted_edge_count == 4
    assert {speed for _, _, speed in mapped.polygon.edges()} == {16}
    assert build_skeleton(mapped.polygon).outcome is SkeletonOutcome.EXACT


def test_a_lattice_that_collapses_an_edge_is_named_and_not_left_to_the_polygon():
    """Привязка склеила ребро — отказ у МОСТА, а не у полигона.

    Разница не в словах. `PolygonV1` сказал бы `LOOP_HAS_REPEATED_POINT` —
    правду про петлю, — и по этому имени нельзя понять, что вершины двигал
    мост и что вместе с ребром исчез его закон.
    """

    # Ребро длиной 1/10 при шаге 1 обязано склеиться: оба конца ближе к
    # одному узлу, чем к разным.
    loops = (
        (
            (Fraction(0), Fraction(0)),
            (Fraction(8), Fraction(0)),
            (Fraction(8), Fraction(1, 10)),
            (Fraction(0), Fraction(8)),
        ),
    )
    report = bridge_arrival_laws(
        loops, _unit_laws_for(loops), lattice=GridSpecV1(scale=1)
    )
    assert report.findings == (
        BridgeOutcome.LATTICE_SNAP_COLLAPSES_A_DOMAIN_EDGE,
    )
    assert report.polygon is None
    # Без решётки тот же вход отвечает про решётку, а не про склейку: обе
    # причины существуют по отдельности и не подменяют друг друга.
    assert bridge_arrival_laws(loops, _unit_laws_for(loops)).findings == (
        BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE,
    )


def test_two_speeds_on_one_support_line_leave_the_speed_undetermined():
    """Разные скорости в одном классе прямых — отказ, а не выбор одной из них.

    Тяжелее неопределённости ВЛАДЕЛЬЦА, и потому названо отдельно: не зная
    имени закона, геометрию всё равно строишь одну, а не зная скорости — разную.
    Проверяется на кресте, где класс несёт два коллинеарных ребра.
    """

    polygon = cross(wide=6, tall=4)
    loops = _loops_of(polygon)
    laws = list(unit_speed_laws_of(polygon))
    classes = [
        line_class((law.normal_x, law.normal_y, law.constant)) for law in laws
    ]
    shared = next(
        key for key in classes if classes.count(key) == 2
    )
    first = classes.index(shared)
    laws[first] = PlainArrivalLawV1(
        name=laws[first].name,
        normal_x=laws[first].normal_x,
        normal_y=laws[first].normal_y,
        constant=laws[first].constant,
        speed_squared=laws[first].speed_squared / 4,
    )
    report = bridge_arrival_laws(loops, tuple(laws), weighted_fronts=True)
    assert (
        BridgeOutcome.EDGE_SPEED_INSIDE_A_LINE_CLASS_IS_UNDETERMINED
        in report.findings
    )
    assert report.polygon is None


# --------------------------------------------------------------------------
# 2. Полевой патч bf6: почему вход не отображается, с числами
# --------------------------------------------------------------------------


def _field_state():
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (FIXTURE / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (FIXTURE / "decal_request.json").read_bytes()
    )
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    assert compiled.compilation is not None
    raw = kernel.evaluate_reference_raw_coverage(
        compiled.compilation, request.requested_alpha
    ).raw_coverage
    assert raw is not None
    boundary_resolved = tuple(
        sorted(
            raw.boundary_resolved_envelopes,
            key=lambda item: item.envelope_instance.envelope_instance_id,
        )
    )
    return compiled.compilation, request, raw, boundary_resolved


def _field_mitred_state():
    """Тот же отгруженный снапшот, но БЕЗ угловых отношений — то есть МИТР.

    Это отрицательный контроль полевых тестов, и он стоит здесь, а не собирается
    в каждом: варианта два, различаются они РОВНО угловыми отношениями, и
    собирать их по-разному значило бы объяснять разницу исходов разницей сборки.

    Снятие угловых отношений есть митрованный член того же семейства, а не
    подделка входа: `test_wavefront_vertex_fan.py::test_an_empty_fan_is_the_miter_bit_for_bit`
    держит это тождеством. Байты при этом ТЕ ЖЕ, включая закон решётки, поэтому
    разница двух половин не может быть отнесена к привязке.
    """

    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (FIXTURE / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (FIXTURE / "decal_request.json").read_bytes()
    )
    mitred = replace(
        snapshot,
        corner_relations=frozenset(),
        angular_owner_sectors=frozenset(),
        reflex_angle_certificates=frozenset(),
    )
    compiled = kernel.compile_reference_envelopes(mitred, request)
    assert compiled.compilation is not None
    raw = kernel.evaluate_reference_raw_coverage(
        compiled.compilation, request.requested_alpha
    ).raw_coverage
    assert raw is not None
    return compiled.compilation, raw


def _as_fraction(expression) -> Fraction:
    exact = sp.cancel(expression)
    assert exact.is_Rational, exact
    return Fraction(int(exact.p), int(exact.q))


def _field_bridge_input():
    """Домен, законы ЮБОК, ВЕЕРА вогнутых вершин и кадр патча — точными дробями.

    Общий этап у всех полевых тестов — того, что вход НЕ отображается
    умолчанием, и того, что он отображается решёткой с весами. Общий он
    намеренно: если бы каждый собирал вход по-своему, «две находки исчезли»
    объяснялось бы разницей сборки, а не изменением моста.

    ЮБКИ И ВЕЕРА РАЗДЕЛЕНЫ, и разделены они не здесь: берётся `_arrival_laws`
    очереди, то есть ТОТ ЖЕ рецепт, которым конвейер читает план. Второй копии
    рецепта в стенде нет намеренно — разойдись она с очередью, тест мерил бы
    свой разбор плана, а не разбор ядра.

    Раньше здесь стояло `compile_arrival_models`, и все его модели уходили в
    один параметр `laws`. После пересборки фикстуры это стало неверно: моделей
    девять (3 юбки + 6 угловых опор, по три на каждый из двух вееров), а прямая
    скрытой опоры ребром домена не является ни при какой привязке — она
    проходит через саму вершину. Мост отвечал на это `ARRIVAL_LAW_IS_NOT_A_DOMAIN_EDGE`,
    и это был верный ответ на неверно заданный вопрос: у веера СВОЙ канал
    (`vertex_fans`), и `bridge_arrival_laws` его объявляет.
    """

    compilation, _, raw, boundary_resolved = _field_state()
    frame, _ = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    context = GeometryContext.build(compilation, frame)
    domain = build_domain_geometry(context)
    region = domain.domain_regions[0]
    loops = (
        tuple(
            (_as_fraction(point.x.as_expr()), _as_fraction(point.y.as_expr()))
            for point in region.outer.points
        ),
    ) + tuple(
        tuple(
            (_as_fraction(point.x.as_expr()), _as_fraction(point.y.as_expr()))
            for point in hole.points
        )
        for hole in region.holes
    )
    reading = _arrival_laws(context)
    assert reading.detail is None, reading.detail
    fans = tuple(
        VertexFanLawV1(fan.name, fan.point, fan.supports) for fan in reading.fans
    )
    return loops, reading.laws, fans, frame


def _field_chart_lattice(frame) -> GridSpecV1:
    """Решётка карты для `bf6` — ПО ОБЪЯВЛЕННОМУ ЗАКОНУ, а не подобранная.

    Два шага, и оба уже объявлены в ядре. Шаг источника берётся из сертификата
    самого патча (`window_step`, он же самый мелкий допустимый шаг окна по
    `FINEST_ADMISSIBLE_FIRST_V1`), а решётка карты выводится из него
    `chart_grid_for`: координаты карты — кратные базисным векторам, а не метры,
    поэтому шаг переносить напрямую нельзя.

    Число здесь не выбирается вовсе, и это главное свойство теста. Подставь
    сюда «удобный» масштаб — и «вход отобразился» стало бы утверждением про
    подобранную решётку, а не про закон, который ядро объявляет для всех.

    Сам ЗАКОН переехал в ядро (`wavefront.conveyor.chart_lattice_for_frame`),
    потому что публичному входу очереди он нужен тоже. Здесь остался вызов, а
    не копия: две копии одного закона разошлись бы при первой же правке, и
    тогда тест проверял бы свою решётку, а не решётку ядра.
    """

    lattice = chart_lattice_for_frame(frame)
    assert lattice is not None
    return lattice


def test_the_transcribed_field_contour_is_the_fixtures_own():
    """Транскрипция корпуса обязана БЫТЬ контуром фикстуры, а не помнить его.

    Тест заведён потому, что молчания тут уже было достаточно. `wavefront_cases`
    держит полевой контур точными дробями, чтобы корпус не тянул за собой весь
    `reference/`, и держал он его НЕВЕРНО: 6 различных вершин вместо 12, каждая
    продублирована. Сторож `field_fixture_digest()` при этом существовал и
    сработать не мог по построению — он хэшировал `source_revision`, поле, от
    контура не зависящее вовсе.

    Здесь сверяются две вещи, и вторая нужна ровно затем, чтобы первая не
    проверяла себя:

    1. транскрипция совпадает с контуром, снятым с байтов ТЕМ ЖЕ путём, что её и
       породил, — поточечно, а не по числу точек;
    2. площадь транскрипции совпадает с `polygon_doubled_area` домена очереди с
       точностью привязки. Прежняя запись эту сверку проваливала на 3.6 %, и
       именно она называет ошибку ошибкой, а не другим соглашением о порядке.
    """

    from wavefront_cases import FIELD_DENOMINATOR, FIELD_NUMERATORS

    loops, _, _, _ = _field_bridge_input()
    measured = loops[0]
    transcribed = tuple(
        (Fraction(x, FIELD_DENOMINATOR), Fraction(y, FIELD_DENOMINATOR))
        for x, y in FIELD_NUMERATORS
    )
    assert len(transcribed) == 12
    assert measured == transcribed

    # Сверка НЕ СОБОЙ: шнуровка транскрипции против площади домена очереди.
    doubled = sum(
        transcribed[index][0] * transcribed[(index + 1) % 12][1]
        - transcribed[(index + 1) % 12][0] * transcribed[index][1]
        for index in range(12)
    )
    assert doubled > 0
    lattice_doubled = Fraction(27224141715, 32768 * 32768)
    assert abs(doubled - lattice_doubled) < Fraction(1, 1000)
    # И то, ради чего сторож вообще есть: отпечаток стоит на полях, от которых
    # контур зависит, поэтому смена закона решётки его двигает.
    #
    # 91d7c72707b64670 -> f2f59e61ea6d95d7 (2026-07-31). Двинула его канонизация
    # `exact_plane_normal` в фикстуре: 1426085/262144 -> 1/1. Отпечаток
    # хэширует весь дескриптор метрики, поэтому смену ЗАПИСИ он видит наравне
    # со сменой геометрии — и это правильно, сторож не обязан их различать.
    # Различают их две проверки ВЫШЕ, и обе прошли неизменными: транскрипция
    # совпала с контуром поточечно, площадь — с `polygon_doubled_area` домена.
    # То есть контур на месте, переехала форма записи нормали.
    assert field_fixture_digest() == "f2f59e61ea6d95d7"


def test_the_field_patch_input_does_not_map_onto_the_queue_and_here_is_why():
    """`bf6` в очередь НЕ отображается, и причин осталось ДВЕ, каждая с числом.

    Это и есть ответ на главный вопрос среза в той его части, которая про
    отображение. Он не «нет», а «вход другой», и разница названа точно:

    | причина | число |
    |---|---|
    | домен не на целой решётке | 9 вершин из 12 |
    | скорости прихода не единичные | три точные bound-величины, заморожены ниже |

    **Третья находка УБРАНА, и вот единственная убранная строка.**
    `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY` — «источник не вся граница, 3 ребра из
    12» — был не свойством входа, а границей очереди: `PolygonV1` знал только
    контур, у которого источником является всё. Теперь у ребра есть своё `q`,
    девять рёбер без закона становятся стенами (`q = 0`), и вход в этой части
    отображается. Числа при этом не исчезли, а переехали в утверждение: 3
    источника и 9 стен из 12 проверяются здесь же, и проверяется ещё одно —
    что разметка ОДНОЗНАЧНА (`undetermined_source_count == 0`), то есть какие
    именно девять рёбер стены, следует из входа, а не выбрано.

    Две оставшиеся находки НЕ ослаблены ни на строку: они те же самые. После
    `CHART_LATTICE_BOUND` три источника имеют три точные записи скорости, потому
    что их bound-отрезки различны. Спред `13.632051778` остаётся близок к
    прежнему `13.629852633`: привязка меняет запись, а не класс задачи.

    ВЕЕРА ЗДЕСЬ НЕ МЕШАЮТ, и это тоже проверено: два веера фикстуры идут своим
    каналом (`vertex_fans`), поэтому в вопрос «отображается ли домен» они не
    входят — находок от них ноль, `fan_edge_count` без привязки ноль тоже.
    """

    loops, laws, fans, _ = _field_bridge_input()

    report = bridge_arrival_laws(loops, laws, vertex_fans=fans)
    assert not report.maps
    assert report.findings == (
        BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE,
        BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,
    )
    # Домен: 9 вершин из 12 не лежат в узлах решётки.
    assert (len(report.off_lattice_points), report.edge_count) == (9, 12)
    # Скорости: три закона, три различные величины, и ни одна не единичная.
    assert len(report.non_unit_speed_laws) == 3
    assert {value for _, value in report.non_unit_speed_laws} == {
        Fraction(36508550182711853056, 224411586007514383265),
        Fraction(1825248901370544128, 11221407898274523445),
        Fraction(47224306603791745024, 3957104371349379650105),
    }
    low, high = report.speed_ratio_spread
    # Евклидовы скорости фронтов различаются в 13.62... раза по квадрату,
    # то есть в 3.69... раза сами. Единичного скелета здесь нет.
    assert high / low == Fraction(
        1354912815551995951514836,
        99391701090710210255317,
    )
    assert float(high / low) == pytest.approx(13.632051778, abs=1e-6)
    # Веер в этот вопрос не входит: ни находки, ни ребра до привязки.
    assert report.fan_edge_count == 0
    assert len(fans) == 2
    # Источник: три ребра домена из двенадцати. Остальные девять — СТЕНЫ, и это
    # теперь утверждение о размётке, а не находка. Однозначность разметки
    # проверяется отдельным числом: ни одно ребро не осталось «источник или
    # стена, неизвестно».
    assert (report.matched_edge_count, report.edge_count) == (3, 12)
    assert report.wall_edge_count == 9
    assert report.undetermined_source_count == 0
    assert report.unmatched_laws == ()
    # Полигона всё ещё нет, и причина ровно одна из двух оставшихся: домен не
    # на решётке. Стены тут ни при чём, и это видно по тому, что список стен
    # пуст, а их ЧИСЛО известно.
    assert report.polygon is None
    assert report.wall_spans == ()


def test_the_field_patch_maps_onto_the_queue_on_the_declared_chart_lattice():
    """`bf6` ОТОБРАЖАЕТСЯ, и обе оставшиеся находки уходят вместе, не по одной.

    Это ответ на главный вопрос среза. Вход отображается ровно тогда, когда
    вызывающий называет две вещи, которые мост не имеет права решить за него:
    решётку, к которой домен привязать, и согласие на ВЗВЕШЕННЫЙ фронт. Обе
    названы законом, а не подобраны: решётка выводится из сертификата самого
    патча (`_field_chart_lattice`), взвешенность следует из измеренных законов.

    | что было находкой | чем стало |
    |---|---|
    | `DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE`, 9 вершин из 12 | bound-привязка, невязка точно 0 |
    | `ARRIVAL_LAW_IS_NOT_UNIT_SPEED`, спред 13.630 | `q` рациональное у 3 рёбер |

    Числа прежнего теста при этом НЕ отменены: без обоих параметров мост
    отвечает ровно то же, что отвечал, и это проверяет тест выше. Здесь
    проверяется другое — что при названных параметрах находок ноль.

    ТРЕТИЙ ПАРАМЕТР — ВЕЕРА, и он не «ещё один флаг»: их два, они входят своим
    каналом, и `fan_edge_count == 2` — единственное число, которым видно, что
    мягкий угол доехал до полигона, а не растворился в митре. Ноль здесь читался
    бы как «веер потерялся», и потерялся бы молча.

    Решётка карты стала `32768` вместо `65536`, и число это НЕ выбрано: шаг
    источника берётся из сертификата решётки самой фикстуры, а он теперь
    `1/2048` (`SOURCE_ONLY_GRID_SNAP_V1`, `source_scale = 2048`) вместо прежнего
    `1/4096`. Сертификат при этом ещё и объясняет, ЗАЧЕМ хост выбрал 2048:
    `scale_trials` = [(4096, 1), (2048, 3)], то есть на 4096 восстанавливался
    один прямой угол из трёх, а на 2048 — все три.
    """

    loops, laws, fans, frame = _field_bridge_input()
    lattice = _field_chart_lattice(frame)
    # Решётка выведена, а не выбрана: шаг источника 1/2048 из сертификата.
    assert lattice.scale == 32768

    report = bridge_arrival_laws(
        loops, laws, lattice=lattice, weighted_fronts=True, vertex_fans=fans
    )
    assert report.maps
    assert report.findings == ()
    assert report.outcome is BridgeOutcome.EXACT
    # Разметка та же самая, что мерил прежний тест: три источника, девять стен.
    assert (report.matched_edge_count, report.edge_count) == (3, 12)
    assert report.wall_edge_count == 9
    assert report.undetermined_source_count == 0
    assert len(report.wall_spans) == 9
    # Все три источника ВЗВЕШЕННЫЕ. Ноль здесь означал бы, что скорость по
    # дороге потерялась и стала единичной, а находка ушла подгонкой.
    assert report.weighted_edge_count == 3
    # Оба веера доехали до полигона своим каналом.
    assert report.fan_edge_count == 2
    assert report.lattice_scale == 32768
    # Вход уже evaluation-bound: второй snap моста — точное тождество.
    assert report.snap_residual == 0
    assert report.snap_residual * lattice.scale < Fraction(1, 2)

    polygon = report.polygon
    # `q` у источников — РАЦИОНАЛЬНОЕ. Два знаменателя принадлежат уже
    # bound-отрезкам, а не прежней source-chart записи.
    weighted = [speed for _, _, speed in polygon.edges() if speed]
    assert len(weighted) == 3
    assert {Fraction(speed).denominator for speed in weighted} == {
        81348737089,
        406743685445,
    }
    assert all(not isinstance(speed, int) for speed in weighted)


def test_the_field_patch_skeleton_is_exact_and_both_search_paths_agree():
    """Очередь считается на ПОЛЕВОЙ геометрии до конца, и ответ у неё один.

    Первое измерение очереди на настоящем патче, поэтому проверяется не только
    исход, но и то, что он не зависит от пути поиска split-кандидатов: сужение
    по трассам motorcycle graph и полный перебор обязаны дать один дайджест.
    Совпадение ЧИСЛА узлов этого не доказало бы — совпадение суммы не
    доказывает совпадения множества, и здесь сверяется именно множество.

    ГРАНИ ТЕПЕРЬ СХОДЯТСЯ, и это сдвиг ровно одной ступени — сборщика, а не
    скелета: числа скелета в этом тесте не двинулись ни одно. История отказа
    сохраняется здесь, потому что ею и найден корень.

    | что было при правиле монотонности | число |
    |---|---|
    | исход сборщика | `FACE_CONTOUR_IS_NOT_SIMPLE` |
    | перекрученных граней | 1 из 3, та, у которой восемь узлов |
    | пары сегментов | `(5, 9)`, `(6, 9)`, `(7, 9)` |
    | сумма граней против площади домена | 112 055 108 527 против 108 901 947 644, ИЗБЫТОК +2.90 % |

    Сегмент 9 — замыкающий, от последнего узла обратно к `source_start`, то есть
    не туда вставал ХВОСТ цепочки. Знак расхождения тогда разошёлся со всем
    корпусом частичного источника, где все случаи
    `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON` были НЕДОСТАТКОМ, и именно
    разошедшийся знак назвал причину: недостаток — незаметённый кусок фигуры,
    избыток — перекрут контура, на котором формула трапеций считает кусок
    дважды. Домен `bf6` имеет два выступа, монотонность на нём не выполняется, и
    правило смежности эту грань собирает верно.

    ЧИСЛА СКЕЛЕТА СДВИНУЛИСЬ после пересборки фикстуры, и сдвиг разложен:
    12 узлов вместо 10, `SPLIT` 6 вместо 4. Прибавка ровно на два расщепления —
    по одному на каждый из двух вееров: скрытая опора вогнутой вершины входит в
    фронт своим ребром, и вершина, которая прежде схлопывалась одним EDGE,
    теперь ещё и расщепляется. `EDGE` при этом не двинулся вовсе (6 и 6), то
    есть привязка к решётке `32768` состав рёберных событий не изменила — а это
    и есть та половина утверждения, которой отличается «веер добавил события» от
    «решётка пересобрала скелет заново».
    """

    loops, laws, fans, frame = _field_bridge_input()
    report = bridge_arrival_laws(
        loops,
        laws,
        lattice=_field_chart_lattice(frame),
        weighted_fronts=True,
        vertex_fans=fans,
    )
    polygon = report.polygon

    by_trace = build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
    exhaustive = build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
    assert by_trace.outcome is SkeletonOutcome.EXACT
    assert exhaustive.outcome is SkeletonOutcome.EXACT
    assert semantic_digest(by_trace) == semantic_digest(exhaustive)
    assert len(by_trace.nodes) == 12
    kinds = Counter(
        kind
        for node in by_trace.nodes
        for kind in (
            node.kinds if node.kind is EventKind.MULTIWAY else (node.kind,)
        )
    )
    # 6/6 -> 10/6 после quotient Q-10-ADD, и единица измерения тут изменилась,
    # а не ответ. Счётчик считает НЕ события, а узлы, у которых данный вид
    # присутствует среди `kinds`; до факторизации все 12 узлов были чистыми
    # (6 EDGE + 6 SPLIT, ни одного MULTIWAY), теперь четыре бывших чистых SPLIT
    # стали MULTIWAY(EDGE, SPLIT) — в них вошёл EDGE-инцидент ТОГО ЖЕ локуса.
    # Это ровно предмет карточки (`test_main_product_has_one_union_incidence_
    # component_node`), а не потеря событий: число узлов не двинулось (12),
    # дайджест обоих путей поиска совпал, грани и площадь ниже не двинулись.
    assert kinds == Counter({EventKind.EDGE: 10, EventKind.SPLIT: 6})

    partition = build_faces(polygon, by_trace)
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert len(partition.faces) == 5
    # Форма разбиения проверяется ПОИМЁННО, а не счётом: три юбки дают
    # `[2, 5, 5]`, два веера — по грани в два узла каждый. Одна сумма «5 граней»
    # держалась бы и при неверном составе.
    assert sorted(face.node_count for face in partition.faces) == [2, 2, 2, 5, 5]
    assert sum(1 for face in partition.faces if len(face.owner) == 5) == 2
    # Ни одного трансверсального самопересечения — по КАЖДОЙ грани, а не «в
    # среднем». Раньше их было три, все на восьмиузловой.
    for face in partition.faces:
        assert contour_crossings(face.points) == (), face.owner
    assert partition.every_contour_is_simple
    assert partition.every_face_is_positive
    # Сумма граней воспроизводит площадь домена ТОЧНО: пустой канонический
    # набор коэффициентов, а не «около нуля». Прежний избыток был +2.90 %.
    assert partition.polygon_doubled_area == 27224141715
    assert partition.area_defect.terms == ()
    assert (
        partition.doubled_area
        - SqrtSumV1.rational(partition.polygon_doubled_area)
    ).is_zero


def test_the_field_patch_needs_the_weights_and_not_only_the_lattice():
    """Отрицательный контроль: на решётке БЕЗ весов очередь не досчитывает.

    Без него «`bf6` отобразился» свелось бы к «домен привязали», и вклад
    рационального `q` остался бы недоказанным. Берётся ТОТ ЖЕ привязанный
    полигон и те же три ребра-источника, но скорость каждого заменяется
    единичной — и фронт остаётся неразрешённым.

    То есть взвешенность на `bf6` покупает не точность и не удобство, а сам
    ответ: `WAVEFRONT_LEFT_UNRESOLVED` против `EXACT` на одной геометрии.

    Контроль ПЕРЕЖИЛ пересборку фикстуры без ослабления: тот же отказ по имени
    на той же привязанной геометрии, теперь ещё и с двумя веерами в полигоне.
    Веер, стало быть, ответа за веса не покупает — и это отдельная строка,
    потому что «стало EXACT само» было бы ровно тем, чего контроль не должен
    допустить незамеченным.
    """

    loops, laws, fans, frame = _field_bridge_input()
    report = bridge_arrival_laws(
        loops,
        laws,
        lattice=_field_chart_lattice(frame),
        weighted_fronts=True,
        vertex_fans=fans,
    )
    assert report.fan_edge_count == 2
    unit = with_edge_speeds(
        report.polygon,
        tuple(
            (start, end, unit_speed_squared(start, end))
            for start, end, speed in report.polygon.edges()
            if speed
        ),
    )
    assert unit.source_edge_count == report.polygon.source_edge_count
    # СТАРЫЙ СВИДЕТЕЛЬ СЪЕДЕН ТРАНЗАКЦИЕЙ, и это усиление, а не потеря.
    # Контроль требовал `WAVEFRONT_LEFT_UNRESOLVED` на единичных скоростях:
    # доказательством служило то, что БЕЗ весов очередь не досчитывает. После
    # quotient Q-10-ADD и плотной гидратации она досчитывает и там — обе
    # геометрии закрываются EXACT.
    #
    # Настоящее утверждение контроля от этого не пострадало, а стало острее:
    # веса покупают не «досчитал / не досчитал», а САМ ОТВЕТ. Проверяется
    # теперь именно расхождение ответов. Счётом его подменить нельзя и это
    # измерено: у обеих геометрий 12 узлов и 5 граней, совпадает даже исход
    # сборщика — различает их ТОЛЬКО дайджест. Равенство дайджестов означало
    # бы, что вес не доехал до геометрии, и было бы дефектом.
    weighted_skeleton = build_skeleton(report.polygon)
    unit_skeleton = build_skeleton(unit)
    assert weighted_skeleton.outcome is SkeletonOutcome.EXACT
    assert unit_skeleton.outcome is SkeletonOutcome.EXACT
    assert len(unit_skeleton.nodes) == len(weighted_skeleton.nodes)
    assert semantic_digest(unit_skeleton) != semantic_digest(weighted_skeleton)


def test_the_field_patch_pairwise_clipping_destroys_all_of_its_own_coverage():
    """Что ломается на `bf6` МИТРОВАННОМ и чего больше не ломается на веерном.

    Числа, ради которых тест написан (митрованный `bf6`, то есть тот же самый
    отгруженный снапшот с СНЯТЫМИ угловыми отношениями):
    - `ResolvedCoverage` = 0;
    - расхождение = вся площадь покрытия, то есть 100%;
    - три юбки ДО кроя попарно не пересекаются, значит верный ответ — не резать
      ничего. Крой вместо этого стирает всё.

    **ОБЪЯВЛЕНИЕ ВОГНУТЫХ УГЛОВ ЭТУ ПАТОЛОГИЮ СНИМАЕТ, и это измерено, а не
    предположено.** На фикстуре КАК ОТГРУЖЕНА (4 `CornerRelation`) крой доходит
    до `EXACT`, кандидатов ноль, а разность с `RawCoverage` — ДОКАЗАННЫЙ ноль.
    Причина названа тем же прогоном: у митрованного входа диагностика говорит
    «multiway meet resolved as one event with 3 participants», то есть крой
    ломался на ТОЧЕЧНЫХ КОНТАКТАХ трёх юбок; с веерами точечных контактов ноль
    (`point_contacts` 0 против 2), сходиться в точку стало нечему, и резать
    нечего по-настоящему, а не по недосмотру.

    Разложение обязательно, потому что причин сдвига было две. Закон решётки
    (`SOURCE_ONLY_GRID_SNAP_V1`) на этот исход НЕ влияет: митрованная половина
    теста считается на ТЕХ ЖЕ привязанных байтах и отказывает ровно как прежде,
    тем же именем и с той же диагностикой. Разницу делают углы, не решётка.
    """

    compilation, _, raw, boundary_resolved = _field_state()

    # Половина первая: фикстура как отгружена — крой СХОДИТСЯ.
    resolution = kernel.resolve_coverage_interactions(
        compilation, raw.boundary_resolved_envelopes, raw
    )
    assert resolution.outcome is InteractionOutcome.EXACT
    assert resolution.resolved_coverage is not None
    assert resolution.candidates == ()
    resolved_expression = resolution.resolved_coverage.exact_area_expression
    difference = sp.simplify(
        sp.radsimp(
            ExactScalar(raw.exact_area_expression).as_expr()
            - ExactScalar(resolved_expression).as_expr()
        )
    )
    assert difference.is_zero is True
    assert len(raw.point_contacts) == 0

    # Половина вторая: тот же снапшот БЕЗ угловых отношений — прежний отказ
    # достижим и назван. Это отрицательный контроль первой половины: без него
    # «крой сошёлся» не отличалось бы от «крой перестали спрашивать».
    mitred_compilation, mitred_raw = _field_mitred_state()
    mitred = kernel.resolve_coverage_interactions(
        mitred_compilation, mitred_raw.boundary_resolved_envelopes, mitred_raw
    )
    assert mitred.outcome is (
        InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN
    )
    assert mitred.resolved_coverage is None
    assert any(
        item.message == (
            "Policy B clipping did not reproduce the exact RawCoverage set"
        )
        for item in mitred.diagnostics
    )
    # Причина отказа названа поимённо: точечные контакты трёх юбок.
    assert len(mitred_raw.point_contacts) == 2
    assert any(
        "multiway meet resolved as one event with 3 participants" in item.message
        for item in mitred.diagnostics
    )

    # Прежние числа кроя — все они про МИТРОВАННЫЙ вход, и все достижимы.
    # Историческое рациональное RawCoverage сменилось exact quadratic
    # выражением: это цена общей bound-геометрии, а не кроя.
    with bound_evaluation_arrangement(
        mitred_compilation.evaluation_geometry_binding is not None
    ):
        mitred_boundary_resolved = tuple(
            sorted(
                mitred_raw.boundary_resolved_envelopes,
                key=lambda item: item.envelope_instance.envelope_instance_id,
            )
        )
        components = compile_interaction_components(
            mitred_compilation, mitred_boundary_resolved
        )
        models, _ = compile_arrival_models(
            mitred_compilation, components, mitred_boundary_resolved
        )
        from cftuv_envelope.interactions.candidates import (
            generate_interaction_candidates,
        )
        from cftuv_envelope.interactions.mutual_arrival import (
            prove_mutual_arrivals,
        )

        candidates = generate_interaction_candidates(
            components, models, mitred_compilation
        )
        proofs, _ = prove_mutual_arrivals(candidates, models)
        frame, _ = validate_reference_geometry_payload(
            mitred_compilation.analysis_snapshot,
            mitred_compilation.plan_key.patch_domain_id,
        )
        domain = build_domain_geometry(
            GeometryContext.build(mitred_compilation, frame)
        )
        policy = policy_b_module.apply_policy_b(
            proofs,
            components,
            mitred_boundary_resolved,
            mitred_raw,
            domain.domain_regions,
        )

        raw_area = ExactScalar(mitred_raw.exact_area_expression).as_expr()
        resolved_area = ExactScalar(
            policy.resolved_union.exact_area_expression
        ).as_expr()
        assert exact_quadratic_value(raw_area).terms == (
            (
                1,
                Fraction(
                    -4415387860811427033088,
                    12774729985721009594933894745,
                ),
            ),
            (27851087972573, Fraction(1, 73015552)),
            (174086333192405, Fraction(1, 182538880)),
            (180146433272521, Fraction(1, 73015552)),
        )
        assert exact_sign(resolved_area) == 0
        assert exact_sign(raw_area - resolved_area) != 0
        assert all(
            not contribution.retained_exact_regions
            for contribution in policy.resolved_contributions
        )

        # Юбки уже попарно не пересекаются: резать было нечего. Компонент три —
        # столько же, сколько было; веера их сливают в одну, и это ровно то число,
        # которым первая половина отличается от второй (1 против 3).
        reachability = {
            str(item.envelope_instance.envelope_instance_id): item.reachability
            for item in mitred_boundary_resolved
        }
        regions_by_component = {}
        for component in components:
            regions = tuple(
                region
                for typed_id in component.envelope_instance_ids
                for item in mitred_boundary_resolved
                if item.envelope_instance.envelope_instance_id == typed_id.value
                for region in item.envelope_instance.regions
            )
            if regions:
                regions_by_component[
                    str(component.interaction_component_id)
                ] = regions
        assert len(regions_by_component) == 3
        names = sorted(regions_by_component)
        for left in range(len(names)):
            for right in range(left + 1, len(names)):
                overlap = policy_b_module._component_intersection_area(
                    regions_by_component[names[left]],
                    regions_by_component[names[right]],
                    reachability,
                )
                assert exact_sign(overlap) == 0
        # И само число, которым две половины отличаются:
        # одна компонента против трёх.
        assert len(
            compile_interaction_components(compilation, boundary_resolved)
        ) == 1


# --------------------------------------------------------------------------
# 3. Отображаемая часть: там у главного вопроса есть ответ
# --------------------------------------------------------------------------


def _pairwise(polygon, alpha: str):
    """Попарный крой на тех же рёбрах и с теми же законами, что видит очередь."""

    face = tuple((float(x), float(y)) for x, y in polygon.outer.points)
    routes = tuple(
        {
            "name": f"e{index}",
            "points": (face[index], face[(index + 1) % len(face)]),
        }
        for index in range(len(face))
    )
    snapshot, request = straight_snapshot(
        faces=(face,),
        source_routes=routes,
        alpha=alpha,
        revision_name=f"q3-diff-{len(face)}-{alpha}",
    )
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    assert compiled.compilation is not None, compiled.diagnostics
    raw = kernel.evaluate_reference_raw_coverage(
        compiled.compilation, request.requested_alpha
    ).raw_coverage
    assert raw is not None
    resolution = kernel.resolve_coverage_interactions(
        compiled.compilation, raw.boundary_resolved_envelopes, raw
    )
    return raw, resolution


def _queue_area(polygon, alpha: Fraction) -> sp.Expr:
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT
    coverage = coverage_at(partition, alpha)
    total = sp.Integer(0)
    for radicand, coefficient in coverage.doubled_area.terms:
        total += sp.Rational(
            coefficient.numerator, coefficient.denominator
        ) * sp.sqrt(sp.Integer(radicand))
    return total / 2


def test_the_queue_reproduces_the_raw_coverage_where_the_clipper_cannot():
    """Отображаемый вход, на котором крой падает, а очередь — нет.

    Прямоугольный треугольник с катетом 12 при alpha = 1: `RawCoverage` даёт
    `21 + 10*sqrt(2)`, попарный крой отказывает
    `INTERACTION_POLICY_B_PARTITION_UNPROVEN`, а очередь выдаёт РОВНО то же
    `21 + 10*sqrt(2)`. Сравнение точное, тем же предикатом `exact_sign`,
    которым проверяет себя сам крой на `policy_b.py:971`.
    """

    polygon = PolygonV1.build(((0, 0), (12, 0), (0, 12)))
    raw, resolution = _pairwise(polygon, "1")
    raw_area = ExactScalar(raw.exact_area_expression).as_expr()
    queue_area = _queue_area(polygon, Fraction(1))

    assert resolution.outcome is (
        InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN
    )
    assert resolution.resolved_coverage is None
    assert exact_sign(raw_area - queue_area) == 0
    assert sp.simplify(queue_area - (21 + 10 * sp.sqrt(2))) == 0


def test_a_convex_case_where_both_paths_agree_is_the_negative_control():
    """Отрицательный контроль: где крой справляется, очередь даёт ТО ЖЕ.

    Без него «очередь лучше» свелось бы к тому, что она просто другая.
    """

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    for alpha in ("1", "2", "4"):
        raw, resolution = _pairwise(polygon, alpha)
        assert resolution.outcome is InteractionOutcome.EXACT
        raw_area = ExactScalar(raw.exact_area_expression).as_expr()
        resolved_area = ExactScalar(
            resolution.resolved_coverage.exact_area_expression
        ).as_expr()
        queue_area = _queue_area(polygon, Fraction(alpha))
        assert exact_sign(raw_area - resolved_area) == 0
        assert exact_sign(raw_area - queue_area) == 0


def test_a_reflex_vertex_makes_the_two_paths_cover_DIFFERENT_SETS():
    """Расхождение, и оно не в разбиении, а в самом покрываемом МНОЖЕСТВЕ.

    Юбка попарного пути — перпендикулярная полоса, обрезанная концами своего
    ребра. Грань скелета обрезана БИССЕКТРИСАМИ и потому выходит за концы
    ребра. У вогнутой вершины на 90 градусов полосы оставляют незакрытый
    квадрат со стороной alpha, а очередь его закрывает:

    | alpha | RawCoverage | очередь | разность |
    |---|---:|---:|---:|
    | 1 | 43 | 44 | 1 |
    | 2 | 76 | 80 | 4 |

    То есть `alpha^2` ровно, и это НЕ дефект очереди: модель владельца требует
    «поле закрывается полностью, дыры быть не должно», и дыру оставляет
    попарный путь. Но и означает, что проверка `policy_b.py:976` в нынешнем
    виде очередь не пропустит: её эталоном служит именно множество полос.
    """

    polygon = PolygonV1.build(
        ((0, 0), (12, 0), (12, 6), (6, 6), (6, 12), (0, 12))
    )
    expected = {Fraction(1): 1, Fraction(2): 4}
    for alpha, gap in expected.items():
        raw, resolution = _pairwise(polygon, str(alpha))
        raw_area = ExactScalar(raw.exact_area_expression).as_expr()
        queue_area = _queue_area(polygon, alpha)
        assert exact_sign(queue_area - raw_area - gap) == 0, (
            f"alpha={alpha}: raw={raw_area}, queue={queue_area}"
        )
        assert resolution.outcome is (
            InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN
        )


def test_the_field_patch_closes_under_the_shipped_order_and_gets_its_coverage():
    """`bf6` сходится ТОЧНО через `build_faces`, и вот его первое покрытие.

    Это ответ полевого входа — того самого, ради которого срез и делался.
    Правило смежности собирает все ПЯТЬ граней просто, положительно и с суммой
    РОВНО в площадь домена `27 224 141 715`. Проверяется здесь именно
    ПОСТАВЛЕННЫЙ путь: `build_faces` и `coverage_at`, без единой подделки
    разбиения; пока правило было кандидатом, партицию приходилось собирать в
    стенде, и это утверждение было про стенд.

    Покрытие очереди на настоящей геометрии, посчитанное здесь же: при alpha из
    запроса (`decal_request.json`, `0.25` в собственной метрике источника, то
    есть `8192` в единицах привязанной решётки масштаба 32768) удвоенная
    площадь покрытия равна примерно `2.7408 %` площади домена. Число
    иррациональное — ВОСЕМЬ членов в каноническом наборе, — поэтому читается оно ЦЕЛИКОМ
    оболочкой, а не по частям, и границы оболочки целые по построению.

    Процент проверяется НЕ СОБОЙ, и это главное здесь число: тот же ответ даёт
    ЭТАЛОННЫЙ путь `evaluate_reference_raw_coverage` на тех же байтах —
    и после `CHART_LATTICE_BOUND` совпадает с QUEUE точно. Две реализации
    сходятся к одному canonical `SqrtSum`, и это сильнее любой замороженной
    десятичной строки. Стеновая часть измеряется отдельно как
    `bound domain - coverage` в `test_wavefront_vertex_fan.py`.

    ПОРЯДОК РАБОТ, которым эта строка оплачена. Смежность нельзя было ставить до
    разбора коллинеарной неподвижной стены в `skeleton.py`: на трёх фигурах
    корпуса частичного источника она превращала верный ГРОМКИЙ ОТКАЗ в число,
    противоречащее независимому митрованному эталону. Стена разобрана, три
    фигуры отказывают СКЕЛЕТОМ до всякого числа, и только после этого порядок
    сборки заменён. Числа перехода — `test_wavefront_partial_source.py`.
    """

    loops, laws, fans, frame = _field_bridge_input()
    report = bridge_arrival_laws(
        loops,
        laws,
        lattice=_field_chart_lattice(frame),
        weighted_fronts=True,
        vertex_fans=fans,
    )
    polygon = report.polygon
    skeleton = build_skeleton(polygon)
    partition = build_faces(polygon, skeleton)
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert len(partition.faces) == 5
    assert sorted(face.node_count for face in partition.faces) == [2, 2, 2, 5, 5]
    for face in partition.faces:
        # Каждая грань по отдельности: простая и строго положительная.
        assert contour_crossings(face.points) == (), face.owner
        assert face.doubled_area.sign() > 0, face.owner
    assert partition.polygon_doubled_area == 27224141715
    assert partition.area_defect.terms == ()
    assert (
        partition.doubled_area
        - SqrtSumV1.rational(partition.polygon_doubled_area)
    ).is_zero

    # alpha запроса 1/4 в метрике источника; решётка карты масштаба 32768,
    # поэтому в её единицах это 8192. Масштаб взят из самого отчёта моста, а
    # не вписан числом, иначе проверялась бы подставленная решётка.
    assert report.lattice_scale == 32768
    covered = coverage_at(partition, Fraction(1, 4) * report.lattice_scale)
    assert covered.outcome is CoverageOutcome.EXACT
    assert len(covered.faces) == 5
    assert len(covered.doubled_area.terms) == 8
    low, high = covered.doubled_area.enclosure(80)
    assert 746152769 < low <= high < 746152770
    # Покрытие строго внутри домена и строго положительно.
    assert covered.doubled_area.sign() > 0
    assert (
        SqrtSumV1.rational(partition.polygon_doubled_area)
        - covered.doubled_area
    ).sign() > 0
