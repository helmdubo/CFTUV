"""Дифференциальная сверка: очередь событий против попарного кроя, один вход.

Срез НИЧЕГО не переключает. Здесь только измерение: где два пути дают один
ответ, где разный, и чем именно.

Главный вопрос среза — про полевой патч `bf6`
(`kernel/fixtures/building_002_point_contact_v1`), на котором попарный крой
падает с `INTERACTION_POLICY_B_PARTITION_UNPROVEN`. Ответ разложен на два теста,
и они отвечают на разное:

- `test_the_field_patch_input_does_not_map_onto_the_queue_and_here_is_why`
  — вход `bf6` в очередь НЕ отображается, и причин три, каждая с числом;
- `test_the_field_patch_pairwise_clipping_destroys_all_of_its_own_coverage`
  — что именно там ломается у попарного кроя: он теряет ВСЮ площадь.

Отображаемая часть корпуса проверяется отдельно, и там ответ есть:
`test_the_queue_reproduces_the_raw_coverage_where_the_clipper_cannot`.
"""

from __future__ import annotations

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
from cftuv_envelope.reference.planar_types import ExactScalar, exact_sign
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)
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
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.polygon import PolygonV1

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
    ребра получали ОДИН индекс, счёт недосчитывался, и срабатывал
    `SOURCE_IS_NOT_THE_WHOLE_BOUNDARY` на входе, где источником было всё:

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


def test_a_source_that_is_not_the_whole_boundary_is_named():
    """Очередь пускает фронт от каждого ребра. Часть рёбер — уже другой вход."""

    polygon = PolygonV1.build(((0, 0), (8, 0), (8, 8), (0, 8)))
    loops = ((
        (Fraction(0), Fraction(0)),
        (Fraction(8), Fraction(0)),
        (Fraction(8), Fraction(8)),
        (Fraction(0), Fraction(8)),
    ),)
    report = bridge_arrival_laws(loops, unit_speed_laws_of(polygon)[:2])
    assert report.outcome is BridgeOutcome.SOURCE_IS_NOT_THE_WHOLE_BOUNDARY
    assert (report.matched_edge_count, report.edge_count) == (2, 4)


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


def test_the_field_patch_input_does_not_map_onto_the_queue_and_here_is_why():
    """`bf6` в очередь НЕ отображается, и причин ровно три, каждая с числом.

    Это и есть ответ на главный вопрос среза в той его части, которая про
    отображение. Он не «нет», а «вход другой», и разница названа точно:

    | причина | число |
    |---|---|
    | домен не на целой решётке | 9 вершин из 12 |
    | скорости прихода не единичные | `(s/|n|)^2` = 137438953472/844687660141 и 17179869184/1439659412197 |
    | источник не вся граница | 3 ребра из 12 |
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

    report = bridge_arrival_laws(loops, laws)
    assert not report.maps
    assert report.findings == (
        BridgeOutcome.DOMAIN_IS_NOT_ON_THE_INTEGER_LATTICE,
        BridgeOutcome.ARRIVAL_LAW_IS_NOT_UNIT_SPEED,
        BridgeOutcome.SOURCE_IS_NOT_THE_WHOLE_BOUNDARY,
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
    # Источник: три ребра домена из двенадцати. Остальные девять — стены.
    assert (report.matched_edge_count, report.edge_count) == (3, 12)
    assert report.unmatched_laws == ()


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
