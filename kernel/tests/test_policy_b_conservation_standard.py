"""Закон сохранения Policy B, развязанный от `RawCoverage`, и что он теперь ловит.

Проверка на `interactions/policy_b.py:969-976` — закон сохранения: разбиение
обязано воспроизвести то же точечное множество, что дал производитель покрытия.
Инвариант верен и здесь не меняется. Менялась только его ЖЁСТКАЯ привязка к
`RawCoverage`, и вот чем эта привязка была плоха, измеренно:

**Привязка круговая, и это видно голыми числами.** Если Policy B не резала ничего
(`proofs = ()` — законный путь: ни одного доказанного взаимного прибытия), союз
вкладов равен `RawCoverage` тождественно. Закон сохранения при этом ВЫПОЛНЕН — и
площадь совпала, и подпись множества петель совпала, — хотя покрытие оставляет у
вогнутой вершины пустой квадрат `alpha^2`. На `ell` при alpha = 1 это ровно 1 из
44, и охранник молчит.

Подставьте вместо `RawCoverage` независимый митрованный эталон
(`mitered_standard.py`) — и тот же самый охранник, той же самой логикой, отдаёт
`INTERACTION_POLICY_B_PARTITION_UNPROVEN` с разностью площадей ровно `alpha^2`.
Параметр не мёртвый: он меняет ответ там, где ответ был неверен.

Отрицательный контроль обязателен и он здесь: на выпуклой фигуре подстановка не
меняет НИЧЕГО — у `axis_square` при alpha = 1 множество петель митрованного
эталона совпадает с множеством петель `RawCoverage` побитово, вплоть до
канонической srepr-строки каждой координаты. Значит эталон не «другой», он тот же
там, где спорить не о чем, и другой ровно у вогнутой вершины.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.interactions import policy_b as policy_b_module
from cftuv_envelope.interactions.components import compile_interaction_components
from cftuv_envelope.interactions.contracts import InteractionOutcome
from cftuv_envelope.reference.boundary import build_domain_geometry
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.contracts import RawCoverageLoopKind
from cftuv_envelope.reference.planar_types import (
    ExactPlanarPoint,
    ExactScalar,
    exact_sign,
)
from cftuv_envelope.reference.validation import (
    validate_reference_geometry_payload,
)

from mitered_standard import (
    StandardOutcome,
    mitered_inset_loop,
    rectilinear_standard,
)
from reference_factories import straight_snapshot
from wavefront_cases import named_corpus


# --------------------------------------------------------------------------
# Эталон в той форме, в которой закон сохранения умеет его читать
# --------------------------------------------------------------------------
#
# От эталона берутся ровно четыре вещи — `vertices`, `edges`, `loops` и
# `exact_area_expression`, — и ничего сверх. Поэтому переходник тонкий: он не
# повторяет `RawCoverageResultV1`, он предъявляет ту же поверхность.
#
# Координаты здесь РАЦИОНАЛЬНЫЕ, и это не мелочь: `point_key` сравнивает
# канонические srepr-строки, а канонизация нерационального значения — та самая
# операция, которая на полевом патче `bf6` дала 21 590 вызовов и 44 секунды. У
# прямоугольного эталона её цена нулевая, потому что канонизировать нечего.


@dataclass(frozen=True, slots=True)
class _StandardVertex:
    vertex_id: str
    point: ExactPlanarPoint


@dataclass(frozen=True, slots=True)
class _StandardEdge:
    edge_id: str
    start_vertex_id: str


@dataclass(frozen=True, slots=True)
class _StandardLoop:
    loop_id: str
    kind: RawCoverageLoopKind
    ordered_edge_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ConservationStandardV1:
    """Множество петель плюс его точная площадь. Больше закону ничего не нужно."""

    vertices: tuple[_StandardVertex, ...]
    edges: tuple[_StandardEdge, ...]
    loops: tuple[_StandardLoop, ...]
    exact_area_expression: str


def _as_standard(loops, area: Fraction) -> ConservationStandardV1:
    vertices: list[_StandardVertex] = []
    edges: list[_StandardEdge] = []
    built: list[_StandardLoop] = []
    for loop_index, (kind, points) in enumerate(loops):
        edge_ids = []
        for point_index, (x, y) in enumerate(points):
            vertex_id = f"v-{loop_index}-{point_index}"
            edge_id = f"e-{loop_index}-{point_index}"
            vertices.append(
                _StandardVertex(
                    vertex_id, ExactPlanarPoint.from_values(x, y)
                )
            )
            edges.append(_StandardEdge(edge_id, vertex_id))
            edge_ids.append(edge_id)
        built.append(
            _StandardLoop(f"l-{loop_index}", kind, tuple(edge_ids))
        )
    return ConservationStandardV1(
        tuple(vertices),
        tuple(edges),
        tuple(built),
        ExactScalar.from_value(area).expression,
    )


def mitered_conservation_standard(polygon, alpha: Fraction):
    """Митрованное покрытие как множество петель: внешний контур плюс отступ дырой.

    Покрытие к моменту alpha — это фигура БЕЗ своего отступа, поэтому петель две:
    внешний контур фигуры и контур отступа, взятый в обратную сторону. Обратная
    сторона не косметика: арранжировка держит дыры по часовой стрелке, а подпись
    множества нормирует только НАЧАЛО обхода, не его направление.
    """

    inset = mitered_inset_loop(polygon, alpha)
    if isinstance(inset, StandardOutcome):
        return inset
    standard = rectilinear_standard(polygon, alpha)
    outer = tuple(
        (Fraction(x), Fraction(y)) for x, y in polygon.outer.points
    )
    return _as_standard(
        (
            (RawCoverageLoopKind.OUTER, outer),
            (RawCoverageLoopKind.HOLE, tuple(reversed(inset))),
        ),
        standard.mitered_covered,
    )


# --------------------------------------------------------------------------
# Прогон Policy B без единого доказанного взаимного прибытия
# --------------------------------------------------------------------------


def _policy_state(polygon, alpha: str):
    """Вход, на котором крой ничего не режет: `proofs = ()`.

    Это не искусственная подстановка, а законная ветка: когда ни одного взаимного
    прибытия не доказано, `apply_policy_b` получает пустой кортеж и оставляет все
    вклады как есть. Именно на ней круговая привязка видна в чистом виде — союз
    вкладов и есть `RawCoverage`, и закон сохранения выполняется тождественно.
    """

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
        revision_name=f"conservation-{len(face)}-{alpha}",
    )
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    assert compiled.compilation is not None, compiled.diagnostics
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
    components = compile_interaction_components(
        compiled.compilation, boundary_resolved
    )
    frame, _ = validate_reference_geometry_payload(
        compiled.compilation.analysis_snapshot,
        compiled.compilation.plan_key.patch_domain_id,
    )
    domain = build_domain_geometry(
        GeometryContext.build(compiled.compilation, frame)
    )
    return raw, components, boundary_resolved, domain.domain_regions


def _apply(state, conservation=None):
    raw, components, boundary_resolved, domain_regions = state
    return policy_b_module.apply_policy_b(
        (),
        components,
        boundary_resolved,
        raw,
        domain_regions,
        conservation=conservation,
    )


def _fired(policy) -> bool:
    return any(
        item.outcome
        is InteractionOutcome.INTERACTION_POLICY_B_PARTITION_UNPROVEN
        for item in policy.diagnostics
    )


# --------------------------------------------------------------------------
# 1. Значение по умолчанию не меняет НИЧЕГО
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name,alpha", (("axis_square", "1"), ("ell", "1")))
def test_the_default_standard_keeps_the_conservation_law_exactly_as_it_was(
    name, alpha
):
    """Без параметра — прежний ответ. Привязка стала выбором, а не переключателем.

    Тест сравнивает не «оба зелёные», а сами объекты: диагностики, площадь союза
    и множество вкладов обязаны совпасть у вызова без параметра и у вызова с явно
    переданным `RawCoverage`.
    """

    polygon = dict(named_corpus())[name]
    state = _policy_state(polygon, alpha)
    default = _apply(state)
    explicit = _apply(state, conservation=state[0])
    assert default.diagnostics == explicit.diagnostics
    assert (
        default.resolved_union.exact_area_expression
        == explicit.resolved_union.exact_area_expression
    )
    assert len(default.resolved_contributions) == len(
        explicit.resolved_contributions
    )


# --------------------------------------------------------------------------
# 2. Круговая привязка: закон выполняется, дыра остаётся
# --------------------------------------------------------------------------


def test_the_raw_coverage_binding_is_silent_about_the_hole_it_produced_itself():
    """`ell`, alpha = 1: охранник молчит, а квадрат `alpha^2` пуст. Это и есть круг.

    Числа: `RawCoverage` = 43, союз вкладов = 43, разность = 0, подпись множества
    петель совпадает, диагностик нет. Митрованный эталон при этом даёт 44, и
    недостающая единица — квадрат `[5,6]x[5,6]` у вогнутой вершины `(6,6)`.
    """

    polygon = dict(named_corpus())["ell"]
    state = _policy_state(polygon, "1")
    policy = _apply(state)
    raw_area = ExactScalar(state[0].exact_area_expression).as_expr()
    union_area = ExactScalar(
        policy.resolved_union.exact_area_expression
    ).as_expr()
    assert exact_sign(raw_area - 43) == 0
    assert exact_sign(raw_area - union_area) == 0
    assert policy_b_module._raw_set_signature(
        state[0]
    ) == policy_b_module._set_signature(policy.resolved_union)
    assert policy.diagnostics == ()
    assert not _fired(policy)

    standard = rectilinear_standard(polygon, Fraction(1))
    assert standard.mitered_covered == 44
    assert standard.strip_covered == 43


# --------------------------------------------------------------------------
# 3. Подстановка эталона ловит дыру, и ровно её
# --------------------------------------------------------------------------


REFLEX_CASES = (
    ("ell", "1", Fraction(1), Fraction(43), Fraction(44)),
    ("ell", "2", Fraction(4), Fraction(76), Fraction(80)),
    ("staircase", "1", Fraction(2), Fraction(42), Fraction(44)),
    ("u_shape", "1", Fraction(2), Fraction(58), Fraction(60)),
    ("u_shape", "2", Fraction(8), Fraction(104), Fraction(112)),
)


@pytest.mark.parametrize(
    "name,alpha,gap,strips,mitered",
    REFLEX_CASES,
    ids=[f"{name}-{alpha}" for name, alpha, _, _, _ in REFLEX_CASES],
)
def test_the_mitered_standard_catches_the_alpha_squared_hole_that_was_silent(
    name, alpha, gap, strips, mitered
):
    """Тот же охранник, та же логика, другой эталон — и дыра НАЗВАНА.

    Разность площадей равна ровно сумме `alpha^2` по вогнутым вершинам фигуры:
    `ell` теряет 1 и 4, `staircase` 2, `u_shape` 2 и 8. Это не «эталон строже»,
    это ровно тот квадрат, который не накрывает ни одна юбка.
    """

    polygon = dict(named_corpus())[name]
    state = _policy_state(polygon, alpha)
    standard = mitered_conservation_standard(polygon, Fraction(alpha))
    assert not isinstance(standard, StandardOutcome), standard

    silent = _apply(state)
    assert not _fired(silent), f"{name}: привязка к RawCoverage не молчала"

    caught = _apply(state, conservation=standard)
    assert _fired(caught), f"{name}: подстановка эталона дыру не поймала"
    assert any(
        item.message
        == "Policy B clipping did not reproduce the exact RawCoverage set"
        for item in caught.diagnostics
    )

    union_area = ExactScalar(
        caught.resolved_union.exact_area_expression
    ).as_expr()
    standard_area = ExactScalar(standard.exact_area_expression).as_expr()
    assert exact_sign(union_area - strips) == 0
    assert exact_sign(standard_area - mitered) == 0
    # Разность — ровно сумма `alpha^2`, а не «какое-то расхождение».
    assert exact_sign(standard_area - union_area - gap) == 0
    assert gap == (
        rectilinear_standard(polygon, Fraction(alpha)).reflex_count
        * Fraction(alpha) ** 2
    )


def test_a_convex_figure_makes_the_substitution_change_absolutely_nothing():
    """Отрицательный контроль, без которого «эталон ловит» значило бы «эталон другой».

    У выпуклой фигуры вогнутых вершин нет, спорить не о чем, и множество петель
    митрованного эталона обязано совпасть с множеством петель `RawCoverage` — не
    по площади, а ПОБИТОВО, включая каноническую строку каждой координаты. Если
    бы совпадало только число, подстановка означала бы замену одного эталона
    другим, а не развязку.
    """

    for name, alpha in (("axis_square", "1"), ("axis_rectangle", "2")):
        polygon = dict(named_corpus())[name]
        state = _policy_state(polygon, alpha)
        standard = mitered_conservation_standard(polygon, Fraction(alpha))
        assert not isinstance(standard, StandardOutcome), standard

        assert policy_b_module._raw_set_signature(
            state[0]
        ) == policy_b_module._raw_set_signature(standard), name
        assert not _fired(_apply(state))
        assert not _fired(_apply(state, conservation=standard))


def test_a_degenerate_kernel_refuses_to_hand_out_a_loop_instead_of_inventing_one():
    """Эталон-множество существует не всегда, и молчать об этом нельзя.

    При alpha, на которой ядро выродилось или распалось, контур отступа не
    строится, и вместо петли возвращается ИСХОД. Подставлять в закон сохранения
    выдуманное множество было бы хуже, чем не подставлять никакого.
    """

    figures = dict(named_corpus())
    # `ell` при alpha = 3: ядро выродилось, покрытие равно всей фигуре.
    assert mitered_conservation_standard(
        figures["ell"], Fraction(3)
    ) is StandardOutcome.INSET_IS_NOT_ONE_LOOP
    # `double_notch` уже при alpha = 1: два ребра отступа слились в одно, и
    # петля с совпавшими соседними вершинами арранжировке не годится, хотя
    # ПЛОЩАДЬ эталона при этой alpha честная — 76 против 72 у полос.
    assert mitered_conservation_standard(
        figures["double_notch"], Fraction(1)
    ) is StandardOutcome.INSET_IS_NOT_ONE_LOOP
    assert rectilinear_standard(
        figures["double_notch"], Fraction(1)
    ).mitered_covered == 76
    assert mitered_conservation_standard(
        figures["hole_1"], Fraction(1)
    ) is StandardOutcome.INSET_LOOP_NEEDS_A_FIGURE_WITHOUT_HOLES
    assert mitered_conservation_standard(
        figures["comb_2"], Fraction(1)
    ) is StandardOutcome.EDGE_IS_NOT_AXIS_PARALLEL
