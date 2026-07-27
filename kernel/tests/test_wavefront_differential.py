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
from cftuv_envelope.reference.boundary import build_domain_geometry
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactScalar, exact_sign
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)
from cftuv_envelope.robust.grid import GridSpecV1
from cftuv_envelope.source_grid import chart_grid_for
from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.bridge import (
    BridgeOutcome,
    PlainArrivalLawV1,
    _proportional_positive,
    _rational_edge_lines,
    bridge_arrival_laws,
    line_class,
    unit_speed_laws_of,
)
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.events import EventKind
from cftuv_envelope.wavefront.faces import (
    FaceOutcome,
    build_faces,
    contour_crossings,
    doubled_shoelace,
)
from cftuv_envelope.wavefront.polygon import (
    PolygonV1,
    unit_speed_squared,
    with_edge_speeds,
)
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, SplitSearch
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

from adjacency_chain import chained_contours
from reference_factories import straight_snapshot
from wavefront_cases import FIELD_FIXTURE, cross, holes_grid, named_corpus


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
    """

    lines = []
    for _, polygon in CORPUS:
        lines.extend(_rational_edge_lines(_loops_of(polygon)))
    assert len(lines) == 185
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


def _as_fraction(expression) -> Fraction:
    numerator, denominator = sp.fraction(sp.nsimplify(expression))
    return Fraction(int(numerator), int(denominator))


def _field_bridge_input():
    """Домен и законы `bf6` в точных дробях плюс кадр патча.

    Общий этап у обоих полевых тестов — того, что вход НЕ отображается
    умолчанием, и того, что он отображается решёткой с весами. Общий он
    намеренно: если бы каждый собирал вход по-своему, «две находки исчезли»
    объяснялось бы разницей сборки, а не изменением моста.
    """

    compilation, _, raw, boundary_resolved = _field_state()
    components = compile_interaction_components(compilation, boundary_resolved)
    models, _ = compile_arrival_models(
        compilation, components, boundary_resolved
    )
    frame, _ = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    domain = build_domain_geometry(GeometryContext.build(compilation, frame))
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
    laws = tuple(
        PlainArrivalLawV1(
            name=str(model.envelope_instance_id),
            normal_x=_as_fraction(model.arrival_law.normal.expressions()[0]),
            normal_y=_as_fraction(model.arrival_law.normal.expressions()[1]),
            constant=_as_fraction(model.arrival_law.source_constant.as_expr()),
            speed_squared=_as_fraction(
                model.arrival_law.normal_speed.as_expr()
            )
            ** 2,
        )
        for model in sorted(models, key=lambda item: str(item.arrival_model_id))
    )
    return loops, laws, frame


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
    """

    certificate = frame.grid_certificate
    step = Fraction(
        certificate.window_step.numerator, certificate.window_step.denominator
    )
    return chart_grid_for(ExactPlanarMetric.from_descriptor(frame).gram, step)


def test_the_field_patch_input_does_not_map_onto_the_queue_and_here_is_why():
    """`bf6` в очередь НЕ отображается, и причин осталось ДВЕ, каждая с числом.

    Это и есть ответ на главный вопрос среза в той его части, которая про
    отображение. Он не «нет», а «вход другой», и разница названа точно:

    | причина | число |
    |---|---|
    | домен не на целой решётке | 9 вершин из 12 |
    | скорости прихода не единичные | `(s/|n|)^2` = 137438953472/844687660141 и 17179869184/1439659412197 |

    **Третья находка УБРАНА, и вот единственная убранная строка.**
    `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY` — «источник не вся граница, 3 ребра из
    12» — был не свойством входа, а границей очереди: `PolygonV1` знал только
    контур, у которого источником является всё. Теперь у ребра есть своё `q`,
    девять рёбер без закона становятся стенами (`q = 0`), и вход в этой части
    отображается. Числа при этом не исчезли, а переехали в утверждение: 3
    источника и 9 стен из 12 проверяются здесь же, и проверяется ещё одно —
    что разметка ОДНОЗНАЧНА (`undetermined_source_count == 0`), то есть какие
    именно девять рёбер стены, следует из входа, а не выбрано.

    Две оставшиеся находки НЕ ослаблены ни на строку: числа те же самые,
    включая спред скоростей 13.634951522381032.
    """

    loops, laws, _ = _field_bridge_input()

    report = bridge_arrival_laws(loops, laws)
    assert not report.maps
    assert report.findings == (
        BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE,
        BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,
    )
    # Домен: 9 вершин из 12 не лежат в узлах решётки.
    assert (len(report.off_lattice_points), report.edge_count) == (9, 12)
    # Скорости: три закона, две различные величины, и ни одна не единичная.
    assert len(report.non_unit_speed_laws) == 3
    assert {value for _, value in report.non_unit_speed_laws} == {
        Fraction(137438953472, 844687660141),
        Fraction(17179869184, 1439659412197),
    }
    low, high = report.speed_ratio_spread
    # Евклидовы скорости фронтов различаются в 13.63... раза по квадрату,
    # то есть в 3.69... раза сами. Единичного скелета здесь нет.
    assert high / low == Fraction(
        137438953472 * 1439659412197, 844687660141 * 17179869184
    )
    assert float(high / low) == pytest.approx(13.634951522, abs=1e-6)
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
    | `DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE`, 9 вершин из 12 | привязка, невязка 7.34e-06 ед. карты |
    | `ARRIVAL_LAW_IS_NOT_UNIT_SPEED`, спред 13.635 | `q` рациональное у 3 рёбер |

    Числа прежнего теста при этом НЕ отменены: без обоих параметров мост
    отвечает ровно то же, что отвечал, и это проверяет тест выше. Здесь
    проверяется другое — что при названных параметрах находок ноль.
    """

    loops, laws, frame = _field_bridge_input()
    lattice = _field_chart_lattice(frame)
    # Решётка выведена, а не выбрана: шаг источника 1/4096 из сертификата.
    assert lattice.scale == 65536

    report = bridge_arrival_laws(
        loops, laws, lattice=lattice, weighted_fronts=True
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
    assert report.lattice_scale == 65536
    # Цена привязки названа числом, а не словом «мала».
    assert report.snap_residual == Fraction(179789606827, 24498750116528128)
    assert report.snap_residual * lattice.scale < Fraction(1, 2)

    polygon = report.polygon
    # `q` у источников — РАЦИОНАЛЬНОЕ, и знаменатели те самые, из-за которых
    # маршрут с подъёмом масштаба и оказался невозможен.
    weighted = [speed for _, _, speed in polygon.edges() if speed]
    assert len(weighted) == 3
    assert {Fraction(speed).denominator for speed in weighted} <= {
        844687660141,
        1439659412197,
        64975973857,
    }
    assert all(not isinstance(speed, int) for speed in weighted)


def test_the_field_patch_skeleton_is_exact_and_both_search_paths_agree():
    """Очередь считается на ПОЛЕВОЙ геометрии до конца, и ответ у неё один.

    Первое измерение очереди на настоящем патче, поэтому проверяется не только
    исход, но и то, что он не зависит от пути поиска split-кандидатов: сужение
    по трассам motorcycle graph и полный перебор обязаны дать один дайджест.
    Совпадение ЧИСЛА узлов этого не доказало бы — совпадение суммы не
    доказывает совпадения множества, и здесь сверяется именно множество.

    Грани при этом ОТКАЗЫВАЮТ, и с появлением третьей объявленной границы
    отказ называет не следствие, а корень: `FACE_CONTOUR_IS_NOT_SIMPLE`, три
    трансверсальных самопересечения на одной грани из трёх, пары сегментов
    `(5, 9)`, `(6, 9)`, `(7, 9)`. Сегмент 9 — замыкающий, от последнего узла
    обратно к `source_start`, то есть не туда поставлен ХВОСТ цепочки.

    Прежнее наблюдение не отменяется, а становится следствием и проверяется
    здесь же: сумма граней по-прежнему ПРЕВОСХОДИТ площадь полигона
    (112 055 108 527 против 108 901 947 644, избыток +2.90%), а в корпусе
    частичного источника все четыре случая `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON`
    были НЕДОСТАТКОМ. Знак разошёлся именно потому, что причина другая:
    недостаток — это незаметённый кусок фигуры, избыток — перекрут контура, на
    котором формула трапеций считает кусок дважды.
    """

    loops, laws, frame = _field_bridge_input()
    report = bridge_arrival_laws(
        loops,
        laws,
        lattice=_field_chart_lattice(frame),
        weighted_fronts=True,
    )
    polygon = report.polygon

    by_trace = build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
    exhaustive = build_skeleton(polygon, split_search=SplitSearch.EXHAUSTIVE)
    assert by_trace.outcome is SkeletonOutcome.EXACT
    assert exhaustive.outcome is SkeletonOutcome.EXACT
    assert semantic_digest(by_trace) == semantic_digest(exhaustive)
    assert len(by_trace.nodes) == 10
    kinds = Counter(node.kind for node in by_trace.nodes)
    assert kinds == Counter({EventKind.EDGE: 6, EventKind.SPLIT: 4})

    partition = build_faces(polygon, by_trace)
    assert partition.outcome is FaceOutcome.FACE_CONTOUR_IS_NOT_SIMPLE
    assert len(partition.faces) == 3
    # Перекручена РОВНО ОДНА грань из трёх, и это та, у которой восемь узлов;
    # две другие по два узла и просты. Пары сегментов проверяются буквально:
    # без них «контур не простой» не показывает, какой кусок цепочки не там.
    twisted = [
        face for face in partition.faces if contour_crossings(face.points)
    ]
    assert len(twisted) == 1
    assert twisted[0].node_count == 8
    assert contour_crossings(twisted[0].points) == ((5, 9), (6, 9), (7, 9))
    assert sorted(face.node_count for face in partition.faces) == [2, 2, 8]
    # Граница 2 при этом ДЕРЖИТСЯ: все три грани положительны. Именно поэтому
    # перекрут и проходил молча, пока границ было две.
    assert partition.every_face_is_positive
    # Знак расхождения — ИЗБЫТОК. Проверяется точным предикатом на сумме
    # корней, а не разностью чисел с плавающей точкой.
    deficit = partition.doubled_area - SqrtSumV1.rational(
        partition.polygon_doubled_area
    )
    assert deficit.sign() > 0
    assert partition.polygon_doubled_area == 108901947644


def test_the_field_patch_needs_the_weights_and_not_only_the_lattice():
    """Отрицательный контроль: на решётке БЕЗ весов очередь не досчитывает.

    Без него «`bf6` отобразился» свелось бы к «домен привязали», и вклад
    рационального `q` остался бы недоказанным. Берётся ТОТ ЖЕ привязанный
    полигон и те же три ребра-источника, но скорость каждого заменяется
    единичной — и фронт остаётся неразрешённым.

    То есть взвешенность на `bf6` покупает не точность и не удобство, а сам
    ответ: `WAVEFRONT_LEFT_UNRESOLVED` против `EXACT` на одной геометрии.
    """

    loops, laws, frame = _field_bridge_input()
    report = bridge_arrival_laws(
        loops,
        laws,
        lattice=_field_chart_lattice(frame),
        weighted_fronts=True,
    )
    unit = with_edge_speeds(
        report.polygon,
        tuple(
            (start, end, unit_speed_squared(start, end))
            for start, end, speed in report.polygon.edges()
            if speed
        ),
    )
    assert unit.source_edge_count == report.polygon.source_edge_count
    assert build_skeleton(unit).outcome is (
        SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
    )


def test_the_field_patch_pairwise_clipping_destroys_all_of_its_own_coverage():
    """Что именно ломается на `bf6`: крой теряет ВСЮ площадь, а не часть.

    Числа, ради которых тест написан:
    - `RawCoverage` = 122766786560/373821260323;
    - `ResolvedCoverage` = 0;
    - расхождение = вся площадь покрытия, то есть 100%.

    И главное: три юбки `bf6` ДО кроя попарно не пересекаются, значит верный
    ответ — не резать ничего. Крой вместо этого стирает всё.
    """

    compilation, _, raw, boundary_resolved = _field_state()
    resolution = kernel.resolve_coverage_interactions(
        compilation, raw.boundary_resolved_envelopes, raw
    )
    assert resolution.outcome is (
        InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN
    )
    assert resolution.resolved_coverage is None
    assert any(
        item.message == (
            "Policy B clipping did not reproduce the exact RawCoverage set"
        )
        for item in resolution.diagnostics
    )

    components = compile_interaction_components(compilation, boundary_resolved)
    models, _ = compile_arrival_models(
        compilation, components, boundary_resolved
    )
    from cftuv_envelope.interactions.candidates import (
        generate_interaction_candidates,
    )
    from cftuv_envelope.interactions.mutual_arrival import prove_mutual_arrivals

    candidates = generate_interaction_candidates(components, models, compilation)
    proofs, _ = prove_mutual_arrivals(candidates, models)
    frame, _ = validate_reference_geometry_payload(
        compilation.analysis_snapshot, compilation.plan_key.patch_domain_id
    )
    domain = build_domain_geometry(GeometryContext.build(compilation, frame))
    policy = policy_b_module.apply_policy_b(
        proofs, components, boundary_resolved, raw, domain.domain_regions
    )

    raw_area = ExactScalar(raw.exact_area_expression).as_expr()
    resolved_area = ExactScalar(
        policy.resolved_union.exact_area_expression
    ).as_expr()
    assert raw_area == sp.Rational(122766786560, 373821260323)
    assert exact_sign(resolved_area) == 0
    assert exact_sign(raw_area - resolved_area) != 0
    assert all(
        not contribution.retained_exact_regions
        for contribution in policy.resolved_contributions
    )

    # Юбки уже попарно не пересекаются: резать было нечего.
    reachability = {
        str(item.envelope_instance.envelope_instance_id): item.reachability
        for item in boundary_resolved
    }
    regions_by_component = {}
    for component in components:
        regions = tuple(
            region
            for typed_id in component.envelope_instance_ids
            for item in boundary_resolved
            if item.envelope_instance.envelope_instance_id == typed_id.value
            for region in item.envelope_instance.regions
        )
        if regions:
            regions_by_component[str(component.interaction_component_id)] = regions
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


def test_the_adjacency_rule_would_close_the_field_patch_but_is_not_shipped():
    """`bf6` под правилом смежности сходится ТОЧНО, и вот его первое покрытие.

    Это измерение кандидата на полевом входе — том самом, ради которого срез и
    делался. Поставленное правило перекручивает одну грань из трёх; правило
    смежности собирает все три просто, положительно и с суммой РОВНО в площадь
    домена `108 901 947 644`.

    Первое покрытие очереди на настоящей геометрии, посчитанное здесь же:
    при alpha из запроса (`decal_request.json`, `0.25` в собственной метрике
    источника, то есть `16384` в единицах привязанной решётки масштаба 65536)
    удвоенная площадь покрытия равна `2.7717 %` площади домена. Число
    иррациональное — шесть членов в каноническом наборе, — поэтому проверяется
    оболочкой, а не равенством дроби, и границы оболочки целые по построению.

    ПОЧЕМУ КАНДИДАТ ВСЁ ЖЕ НЕ ПОСТАВЛЕН, записано числами там, где противоречие
    и меряется: `test_wavefront_partial_source.py`
    `test_the_adjacency_rule_would_answer_where_the_standard_says_refuse`. Здесь
    только та половина, которая говорит «правило верно»; вторая говорит «его
    нельзя ставить, пока `skeleton.py` гасит коллинеарную неподвижную стену
    целиком», и обе половины обязаны стоять рядом.
    """

    loops, laws, frame = _field_bridge_input()
    report = bridge_arrival_laws(
        loops,
        laws,
        lattice=_field_chart_lattice(frame),
        weighted_fronts=True,
    )
    polygon = report.polygon
    skeleton = build_skeleton(polygon)
    shipped = build_faces(polygon, skeleton)
    assert shipped.outcome is FaceOutcome.FACE_CONTOUR_IS_NOT_SIMPLE

    contours, why = chained_contours(polygon, skeleton)
    assert not why
    assert len(contours) == 3
    total = SqrtSumV1.zero()
    for points in contours.values():
        area = doubled_shoelace(points)
        # Каждая грань по отдельности: простая и строго положительная.
        assert contour_crossings(points) == ()
        assert area.sign() > 0
        total = total + area
    assert (
        total - SqrtSumV1.rational(shipped.polygon_doubled_area)
    ).is_zero
    assert shipped.polygon_doubled_area == 108901947644

    faces = tuple(
        replace(
            face,
            points=contours[face.owner],
            doubled_area=doubled_shoelace(contours[face.owner]),
        )
        for face in shipped.faces
    )
    partition = replace(
        shipped, outcome=FaceOutcome.EXACT, faces=faces, doubled_area=total
    )
    # alpha запроса 1/4 в метрике источника; решётка карты масштаба 65536,
    # поэтому в её единицах это 16384. Масштаб взят из самого отчёта моста, а
    # не вписан числом, иначе проверялась бы подставленная решётка.
    assert report.lattice_scale == 65536
    covered = coverage_at(partition, Fraction(1, 4) * report.lattice_scale)
    assert covered.outcome is CoverageOutcome.EXACT
    assert len(covered.faces) == 3
    # Величина иррациональна: шесть членов в каноническом наборе. Читается она
    # ЦЕЛИКОМ оболочкой, а не по частям — правило проекта.
    assert len(covered.doubled_area.terms) == 6
    low, high = covered.doubled_area.enclosure(80)
    assert 3018411397 < low <= high < 3018411399
    # Покрытие строго внутри домена и строго положительно.
    assert covered.doubled_area.sign() > 0
    assert (
        SqrtSumV1.rational(shipped.polygon_doubled_area)
        - covered.doubled_area
    ).sign() > 0
