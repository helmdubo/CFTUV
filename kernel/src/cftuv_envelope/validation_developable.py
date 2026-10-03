"""Проверки метрики с сертификатом развёртки: по проводу и пересчётом из источника.

Отдельный модуль, как `validation_metric` отделён от `validation.py`: тот стоит на
своём потолке длины, а закон развёртки ничего не должен делить с законом плоскости,
кроме общей проводной проверки Грама и покрытия вершин.

Две проверки, и вторая не заменяется первой.

1. ПО ПРОВОДУ (`check_developable_certificate`): запись метрики сама согласована —
   тот же домен и ревизия, объявленные законы, репер развёртки `origin = 0`,
   `A = e_x/S'`, `B = e_y/S'`, целые координаты (кроме внутренностей объявленных
   прямыми цепей: те на хорде своих концов, строго по порядку), бюджет растяжения —
   тот, что объявляет закон, судья растяжения не находит нарушений, кратность
   решётки карты — из объявленного перечня.
2. ПЕРЕСЧЁТ (`validate_developable_recomputation`): карта и сертификат СТРОЯТСЯ
   ЗАНОВО из привязанных позиций источника и треугольников снапшота и сравниваются
   с записью на равенство. Предложение развёртки детерминировано (binary64 с
   фиксированным порядком операций), поэтому равенство точное, а не «в допуске».

   У сертификата второго предложения (`ARAP_LOCAL_GLOBAL_80_BINARY64_V1`, а при самонакрытии
   изометрии с граничной вершиной-избытком — `ARAP_CONE_RELIEF_80_BINARY64_V1`) пересчёт
   тот же и ЗВУЧЕН по той же причине: ARAP — только `+ - * /` и `sqrt` в
   фиксированном порядке, число итераций названо законом, поэтому предложение
   воспроизводится побитово, а карту после него судит тот же ТОЧНЫЙ суд, что перечитывает
   пересчёт (карта ARAP не принимается на слово: растяжение и граница измерены заново).
   Билдер сам решает, нужен ли ARAP (шарнир отказал по названной причине либо принятая карта
   шарнира растянута выше `DEVELOPABLE_ISOMETRIC_ENOUGH`), поэтому подмена закона в записи
   расходится с пересчётом, а не проходит. У закона «лучшее предложение» пересчёт строит ОБА
   предложения заново и сверяет оба числа, победителя и причину, а проводная проверка
   (`_check_proposal_selection`) видит расхождение закона выбора с самими записанными числами.

ДОПУСК — ПОЛИТИКА ЗАПРОСА. Запись сверяется не с константой, а с допуском запроса
(`developable_stretch_budget`): снапшот, проверяемый вместе с запросом, обязан нести
сертификат, записанный под ЭТИМ допуском, и пересчёт строит карту под ним. Снапшот без запроса
(`developable_stretch_budget=None`) проверяется под собственным записанным допуском, законным
по `developable_stretch_budget_is_lawful`; связать его с запросом дело
`validate_snapshot_request_references`.
"""

from __future__ import annotations

from fractions import Fraction

from ._band_chart import build_band_chart
from ._developable import (
    ARAP_TRIGGER_OUTCOMES,
    UNFOLD_CHART_SCALE_FACTORS,
    build_developable_chart,
)
from ._stretch import band_bounds, stretch_violations
from .contracts.metric import (
    DEVELOPABLE_ISOMETRIC_ENOUGH,
    AffineChartOrientationV1,
    AffineFrameSelectionLawV1,
    AffineReconstructionLawV1,
    DevelopableBandChartCertificateV1,
    DevelopableFanClosureLawV1,
    DevelopableLiftLawV1,
    DevelopableProposalLawV1,
    DevelopableProposalSelectionLawV1,
    DevelopableStraightChainLawV1,
    DevelopableStretchLawV1,
    DevelopableUnfoldCertificateV1,
    DevelopableUnfoldTreeLawV1,
    PlanarityAdmissionLawV1,
    VertexDevelopabilityClassV1,
    developable_stretch_budget_is_lawful,
    is_unfolded_certificate,
)
from .numeric import LocalPoint3V1
from .outcomes import NamedOutcome
from .validation_band import check_band_certificate
from .validation_issues import ValidationCode, ValidationIssue, add_issue


#: Законы семейства ARAP (второе предложение): простой ARAP — после именованного отказа шарнира либо соперником принятого
#: шарнира, ARAP с запасом угла — только после отказа шарнира. Какой из случаев, говорит закон выбора, а не закон предложения.
SECOND_PROPOSAL_LAWS = frozenset(
    {
        DevelopableProposalLawV1.ARAP_LOCAL_GLOBAL_80_BINARY64_V1,
        DevelopableProposalLawV1.ARAP_CONE_RELIEF_80_BINARY64_V1,
    }
)

#: Законы предложения, которые ядро объявляет: шарнир и (после его именованного отказа) ARAP либо ARAP с запасом угла.
DECLARED_PROPOSAL_LAWS = SECOND_PROPOSAL_LAWS | {DevelopableProposalLawV1.BINARY64_HINGE_V1}


def _fraction(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _arap_after_refusal(certificate) -> bool:
    """ARAP — единственное предложение, потому что шарнир отказан именем (а не соперник шарнира)."""

    return (
        certificate.proposal_selection_law
        is DevelopableProposalSelectionLawV1.ARAP_AFTER_HINGE_REFUSED_V1
    )


def ladder_trace(certificate) -> tuple[str, ...]:
    """След ступеней НИЖЕ развёртки: у второго предложения после отказа шарнира последняя запись — отказ шарнира."""

    trace = certificate.previous_refusals
    if _arap_after_refusal(certificate):
        return trace[:-1]
    return trace


def _policy_budget(certificate, budget) -> Fraction:
    """Допуск, под которым судится запись: ЗАПРОС; без запроса — собственный записанный допуск."""

    return _fraction(certificate.stretch.stretch_budget) if budget is None else budget


def _point(value) -> tuple[Fraction, Fraction, Fraction]:
    return _fraction(value.x), _fraction(value.y), _fraction(value.z)


def check_developable_certificate(issues, path, metric, developable_stretch_budget=None) -> None:
    """Запись метрики с сертификатом развёртки согласована сама с собой (и с допуском запроса)."""

    certificate = metric.planarity_certificate
    banded = type(certificate) is DevelopableBandChartCertificateV1
    if (
        certificate.patch_domain_id != metric.patch_domain_id
        or certificate.source_revision != metric.source_revision
        or certificate.admission_law
        is not (
            PlanarityAdmissionLawV1.DEVELOPABLE_BAND_CHART_V1
            if banded
            else PlanarityAdmissionLawV1.DEVELOPABLE_UNFOLD_V1
        )
        or certificate.reconstruction_law
        is not AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1
        or certificate.exact
        or certificate.tree_law
        is not DevelopableUnfoldTreeLawV1.CANONICAL_BFS_SMALLEST_TRIANGLE_ID_V1
        or certificate.proposal_law not in DECLARED_PROPOSAL_LAWS
        or certificate.lift_law is not DevelopableLiftLawV1.UNFOLDED_SOURCE_TRIANGLES_V1
        or certificate.straight_chain_law
        is not DevelopableStraightChainLawV1.INTERIOR_NODES_ON_ENDPOINT_SEGMENT_V1
        or certificate.stretch.law
        is not DevelopableStretchLawV1.EXACT_GRAM_SINGULAR_VALUE_BAND_V1
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            "developable certificate must be a non-exact same-domain same-revision "
            "record of the declared tree, proposal, stretch and lift laws",
        )
    if metric.frame_selection_law is not (
        AffineFrameSelectionLawV1.UNFOLDED_DEVELOPMENT_FRAME_V1
    ) or metric.chart_orientation is not (
        AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("frame_selection_law",),
            "an unfolded chart is described by the unfolded development frame, "
            "counter-clockwise against the owner Patch",
        )
    _check_frame(issues, path, metric, certificate)
    _check_judgement(issues, path, metric, certificate, developable_stretch_budget)
    _check_proposal_selection(issues, path, certificate)
    recorded = {item.source_vertex_id for item in certificate.snapped_source_positions}
    if recorded != set(certificate.source_vertex_ids) or len(recorded) != len(
        certificate.snapped_source_positions
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("snapped_source_positions",),
            "snapped positions must cover the certificate source vertices exactly once",
        )
    _check_ladder_trace(issues, path, certificate)
    if banded:
        check_band_certificate(issues, path, metric, certificate)
    for item in certificate.vertex_classes:
        undecided = (
            item.developability_class
            is VertexDevelopabilityClassV1.UNDECIDED_WORK_BUDGET
        )
        if undecided != (
            item.closure_law is DevelopableFanClosureLawV1.FAN_CLOSURE_UNDECIDED_V1
        ):
            add_issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                path + ("vertex_classes",),
                "an undecided vertex and the undecided closure law come together",
            )


def _check_ladder_trace(issues, path, certificate) -> None:
    """След лестницы: домен попал сюда после отказа near-planar по названной причине.

    Пересчёт ступени ниже (весь near-planar построитель) валидатору не по карману, и
    записанный след проверяется не пересчётом, а ДОПУСТИМОСТЬЮ: он непуст и называет
    только триггеры лестницы. Подмена одного допустимого имени другим карту не
    меняет — карту пересчёт проверяет целиком.

    След ARAP-сертификата заканчивается отказом шарнира: именем из `ARAP_TRIGGER_OUTCOMES`
    (ARAP пробуется только после него); шарнирный сертификат такой записи не несёт.
    """

    from .planar_metric import BAND_TRIGGER_OUTCOMES, LADDER_TRIGGER_OUTCOMES, PLANE_NORMAL_UNDEFINED_TRACE

    allowed = {item.value for item in LADDER_TRIGGER_OUTCOMES} | {
        PLANE_NORMAL_UNDEFINED_TRACE
    }
    trace = ladder_trace(certificate)
    below = trace
    if type(certificate) is DevelopableBandChartCertificateV1:
        # Полоса пробуется после отказа развёртки ЦЕЛОГО патча: след - отказ near-planar и именованный отказ целого.
        named = {item.value for item in BAND_TRIGGER_OUTCOMES}
        below = trace[:1] if len(trace) == 2 and trace[1] in named else ()
    if not below or any(item not in allowed for item in below):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("previous_refusals",),
            "an unfolded chart is tried only after a named near-planar refusal: "
            "the ladder trace must name a ladder trigger",
        )
    after = _arap_after_refusal(certificate)
    relief = certificate.proposal_law is (
        DevelopableProposalLawV1.ARAP_CONE_RELIEF_80_BINARY64_V1
    )
    last = {NamedOutcome.DEVELOPABLE_CHART_SELF_OVERLAP.value} if relief else {
        item.value for item in ARAP_TRIGGER_OUTCOMES
    }
    if after and (
        len(certificate.previous_refusals) != len(trace) + 1
        or certificate.previous_refusals[-1] not in last
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("previous_refusals",),
            "the second (ARAP) proposal is tried only after a named refusal of the "
            "hinge proposal (the cone relief only after its self-overlap): the trace "
            "must end with that refusal",
        )


def _check_frame(issues, path, metric, certificate) -> None:
    scale = certificate.chart_scale
    unit = Fraction(1, scale)
    expected = (
        (Fraction(0),) * 3,
        (unit, Fraction(0), Fraction(0)),
        (Fraction(0), unit, Fraction(0)),
    )
    declared = (
        _point(metric.exact_origin),
        _point(metric.exact_basis_a),
        _point(metric.exact_basis_b),
    )
    if declared != expected or _point(certificate.exact_plane_normal) != (
        Fraction(0),
        Fraction(0),
        Fraction(1),
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_origin", "exact_basis_a", "exact_basis_b"),
            "the unfolded development frame is origin 0, A = e_x/S', B = e_y/S' "
            "with the chart-plane normal (0, 0, 1)",
        )
    _check_coordinates(issues, path, metric, certificate)
    source_scale = metric.grid_certificate.source_scale
    factors = {factor * source_scale for factor in UNFOLD_CHART_SCALE_FACTORS} if (
        source_scale is not None and metric.grid_certificate.snapping_law.snaps_source
    ) else set()
    if scale not in factors:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("chart_scale",),
            "the chart scale is not a declared multiple of the source grid scale",
        )
    elif UNFOLD_CHART_SCALE_FACTORS.index(scale // source_scale) + 1 != (
        certificate.chart_scale_trials
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("chart_scale_trials",),
            "the recorded trial count does not lead to the recorded chart scale",
        )


def _check_coordinates(issues, path, metric, certificate) -> None:
    """Узлы карты — целые; внутренности объявленных прямыми цепей — на хорде, строго вперёд."""

    interior = {
        vertex for item in certificate.declared_straight_chains for vertex in item.vertex_ids[1:-1]
    }
    coordinates = {
        item.source_vertex_id: (_fraction(item.domain_coordinate.x), _fraction(item.domain_coordinate.y))
        for item in metric.exact_source_vertex_coordinates
    }
    fractional = sorted(
        vertex.value
        for vertex, point in coordinates.items()
        if vertex not in interior and (point[0].denominator != 1 or point[1].denominator != 1)
    )
    if fractional:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_source_vertex_coordinates",),
            f"unfolded chart coordinates are integer lattice nodes, except interior vertices "
            f"of declared straight chains; not integer: {fractional[:3]}",
        )
    for item in certificate.declared_straight_chains:
        points = [coordinates.get(vertex) for vertex in item.vertex_ids]
        if None in points or not _on_chord_in_order(points):
            add_issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                path + ("declared_straight_chains",),
                f"declared straight chain {item.vertex_ids[0].value}..{item.vertex_ids[-1].value} "
                "is not on one line of the chart in order",
            )


def _on_chord_in_order(points) -> bool:
    span = (points[-1][0] - points[0][0], points[-1][1] - points[0][1])
    reach = span[0] * span[0] + span[1] * span[1]
    previous = Fraction(0)
    for point in points[1:-1]:
        offset = (point[0] - points[0][0], point[1] - points[0][1])
        along = offset[0] * span[0] + offset[1] * span[1]
        if span[0] * offset[1] - span[1] * offset[0] or not previous < along < reach:
            return False
        previous = along
    return bool(reach)


def _ratio_text(value) -> str:
    return "none" if value is None else f"{value.numerator}/{value.denominator}"


def _check_judgement(issues, path, metric, certificate, budget=None) -> None:
    stretch = certificate.stretch
    recorded = _fraction(stretch.stretch_budget)
    lawful = developable_stretch_budget_is_lawful(recorded)
    if recorded != _policy_budget(certificate, budget) or not lawful:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("stretch", "stretch_budget"),
            f"recorded={_ratio_text(recorded)} request={_ratio_text(budget)}: the recorded stretch "
            "budget is not the request's lawful developable_stretch_budget",
        )
    for outcome in stretch_violations(stretch):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("stretch",),
            f"admitted as an unfolded chart, but {outcome.value}",
        )
    if certificate.chart_boundary_overlap_count:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("chart_boundary_overlap_count",),
            "admitted as an unfolded chart, but the chart boundary is not simple",
        )


def _check_proposal_selection(issues, path, certificate) -> None:
    """Закон выбора предложения согласован с записанными числами и победителем.

    Читается только сама запись: победитель (`proposal_law`, семейство ARAP включает запас угла у
    конуса), оба числа и порог изометрии обязаны сходиться. Расхождение с пересчётом ловит `validate_developable_recomputation`.
    """

    law = DevelopableProposalSelectionLawV1
    selection = certificate.proposal_selection_law
    hinge = certificate.hinge_chart_worst_band_squared_upper
    rival = certificate.arap_chart_worst_band_squared_upper
    arap_won = certificate.proposal_law in SECOND_PROPOSAL_LAWS
    shape = {
        law.HINGE_ISOMETRIC_ENOUGH_V1: (False, True, False),
        law.ARAP_AFTER_HINGE_REFUSED_V1: (True, False, True),
        law.BEST_HINGE_WON_V1: (False, True, True),
        law.BEST_ARAP_WON_V1: (True, True, True),
        law.HINGE_KEPT_ARAP_UNAVAILABLE_V1: (False, True, False),
        law.HINGE_KEPT_ARAP_REFUSED_V1: (False, True, False),
    }[selection]
    sound = (arap_won, hinge is not None, rival is not None) == shape
    # Третье предложение (запас угла) бывает только ВТОРЫМ после отказа шарнира, соперником принятого оно не бывает.
    if certificate.proposal_law is DevelopableProposalLawV1.ARAP_CONE_RELIEF_80_BINARY64_V1:
        sound = sound and selection is law.ARAP_AFTER_HINGE_REFUSED_V1
    if sound:
        chosen = _fraction(rival if arap_won else hinge)
        sound = chosen == _fraction(certificate.stretch.worst_band_squared_upper)
        if hinge is not None:
            isometric = _fraction(hinge) <= band_bounds(DEVELOPABLE_ISOMETRIC_ENOUGH)[1]
            sound = sound and isometric == (selection is law.HINGE_ISOMETRIC_ENOUGH_V1)
        if selection is law.BEST_HINGE_WON_V1:
            sound = sound and _fraction(hinge) <= _fraction(rival)
        if selection is law.BEST_ARAP_WON_V1:
            sound = sound and _fraction(rival) < _fraction(hinge)
    named = certificate.arap_refusal in {item.value for item in NamedOutcome}
    sound = sound and named == (selection is law.HINGE_KEPT_ARAP_REFUSED_V1)
    sound = sound and (selection is law.HINGE_KEPT_ARAP_REFUSED_V1 or not certificate.arap_refusal)
    if not sound:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("proposal_selection_law",),
            "the proposal selection law disagrees with the recorded hinge and ARAP chart "
            "stretch numbers, the winning proposal law or the isometric threshold",
        )


def _recomputation_inputs(
    metric, budget, *, source_vertices, source_faces, surface_triangles, owner_patch_id, declared_straight_chains
) -> dict:
    """Входы построителя карты: привязанные позиции источника по ОБЪЯВЛЕННОМУ закону решётки метрики, треугольники патча."""

    from .validation_metric import _source_embedding_inputs, position_under_grid_law

    faces, required_ids, positions = _source_embedding_inputs(
        source_vertices=source_vertices,
        source_faces=source_faces,
        owner_patch_id=owner_patch_id,
    )
    face_ids = {face.face_id for face in faces}
    grid = metric.grid_certificate
    return dict(
        source_revision=metric.source_revision,
        patch_domain_id=metric.patch_domain_id,
        snapped={
            vertex_id: position_under_grid_law(
                LocalPoint3V1(*(float(axis) for axis in position)),
                metric.grid_certificate,
            )
            for vertex_id, position in positions.items()
        },
        owner_triangles=tuple(
            item for item in surface_triangles if item.source_face_id in face_ids
        ),
        required_ids=required_ids,
        source_scale=grid.source_scale if grid.snapping_law.snaps_source else None,
        previous_refusals=ladder_trace(metric.planarity_certificate),
        declared_straight_chains=tuple(declared_straight_chains),
        budget=budget,
    )


def validate_developable_recomputation(
    metric,
    *,
    source_vertices,
    source_faces,
    surface_triangles,
    owner_patch_id,
    declared_straight_chains=(),
    developable_stretch_budget=None,
    chart_band=None,
) -> tuple[ValidationIssue, ...]:
    """Построить карту и сертификат заново и сравнить с записью на равенство.

    `declared_straight_chains` — вершины объявленных прямыми цепей домена ИЗ СНАПШОТА
    (`declared_chains`), а не из записи: запись метрики этого не заявляет, а проверяет. `chart_band`
    (`ChartBandRequestV1` по выбору и досягаемости, ЗАПИСАННЫМ в сертификате полосы, и цепям снапшота) нужен только
    сертификату полосы: пересчёт строит носитель, развёртку, границу и запас тем же построителем.
    """

    from .planar_metric import PlanarMetricAdmissionError

    certificate = metric.planarity_certificate
    if not is_unfolded_certificate(certificate):
        return ()
    banded = type(certificate) is DevelopableBandChartCertificateV1
    issues: list[ValidationIssue] = []
    path = ("RationalAffinePlanarMetricV2", "planarity_certificate")
    if banded and chart_band is None:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            "a band chart cannot be recomputed without the selected chains and the reach cap",
        )
        return tuple(issues)
    budget = _policy_budget(certificate, developable_stretch_budget)
    if not developable_stretch_budget_is_lawful(budget):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            "the unfolded chart cannot be recomputed under a stretch budget outside (0, 1/2]",
        )
        return tuple(issues)
    if any(not isinstance(item.position, LocalPoint3V1) for item in source_vertices):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            "an unfolded chart cannot be recomputed without local coordinates",
        )
        return tuple(issues)
    inputs = _recomputation_inputs(
        metric,
        budget,
        source_vertices=source_vertices,
        source_faces=source_faces,
        surface_triangles=surface_triangles,
        owner_patch_id=owner_patch_id,
        declared_straight_chains=declared_straight_chains,
    )
    try:
        expected = build_band_chart(band=chart_band, **inputs) if banded else build_developable_chart(**inputs)
        if expected is None:
            add_issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                path,
                "the band support is the whole patch: a band chart is not a chart of this domain",
            )
            return tuple(issues)
    except PlanarMetricAdmissionError as error:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            f"the unfolded chart cannot be recomputed: {error.outcome.value}",
        )
        return tuple(issues)
    if expected.certificate != certificate:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            "developable certificate differs from exact recomputation",
        )
    declared = {
        item.source_vertex_id: (
            _fraction(item.domain_coordinate.x),
            _fraction(item.domain_coordinate.y),
        )
        for item in metric.exact_source_vertex_coordinates
    }
    if declared != {
        vertex_id: (Fraction(node[0]), Fraction(node[1]))
        for vertex_id, node in expected.nodes.items()
    }:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            ("RationalAffinePlanarMetricV2", "exact_source_vertex_coordinates"),
            "unfolded chart coordinates differ from exact recomputation",
        )
    return tuple(issues)


__all__ = (
    "check_developable_certificate",
    "validate_developable_recomputation",
)
