"""Проверки метрики с сертификатом развёртки: по проводу и пересчётом из источника.

Отдельный модуль, как `validation_metric` отделён от `validation.py`: тот стоит на
своём потолке длины, а закон развёртки ничего не должен делить с законом плоскости,
кроме общей проводной проверки Грама и покрытия вершин.

Две проверки, и вторая не заменяется первой.

1. ПО ПРОВОДУ (`check_developable_certificate`): запись метрики сама согласована —
   тот же домен и ревизия, объявленные законы, репер развёртки `origin = 0`,
   `A = e_x/S'`, `B = e_y/S'`, целые координаты, бюджет растяжения — тот, что
   объявляет закон, судья растяжения не находит нарушений, кратность решётки
   карты — из объявленного перечня.
2. ПЕРЕСЧЁТ (`validate_developable_recomputation`): карта и сертификат СТРОЯТСЯ
   ЗАНОВО из привязанных позиций источника и треугольников снапшота и сравниваются
   с записью на равенство. Предложение развёртки детерминировано (binary64 с
   фиксированным порядком операций), поэтому равенство точное, а не «в допуске».
"""

from __future__ import annotations

from fractions import Fraction

from ._developable import UNFOLD_CHART_SCALE_FACTORS, build_developable_chart
from ._stretch import stretch_violations
from .contracts.metric import (
    DEVELOPABLE_STRETCH_BUDGET,
    AffineChartOrientationV1,
    AffineFrameSelectionLawV1,
    AffineReconstructionLawV1,
    DevelopableFanClosureLawV1,
    DevelopableLiftLawV1,
    DevelopableProposalLawV1,
    DevelopableStretchLawV1,
    DevelopableUnfoldCertificateV1,
    DevelopableUnfoldTreeLawV1,
    PlanarityAdmissionLawV1,
    VertexDevelopabilityClassV1,
)
from .numeric import LocalPoint3V1
from .validation_issues import ValidationCode, ValidationIssue, add_issue


def _fraction(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _point(value) -> tuple[Fraction, Fraction, Fraction]:
    return _fraction(value.x), _fraction(value.y), _fraction(value.z)


def check_developable_certificate(issues, path, metric) -> None:
    """Запись метрики с сертификатом развёртки согласована сама с собой."""

    certificate = metric.planarity_certificate
    if (
        certificate.patch_domain_id != metric.patch_domain_id
        or certificate.source_revision != metric.source_revision
        or certificate.admission_law is not PlanarityAdmissionLawV1.DEVELOPABLE_UNFOLD_V1
        or certificate.reconstruction_law
        is not AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1
        or certificate.exact
        or certificate.tree_law
        is not DevelopableUnfoldTreeLawV1.CANONICAL_BFS_SMALLEST_TRIANGLE_ID_V1
        or certificate.proposal_law is not DevelopableProposalLawV1.BINARY64_HINGE_V1
        or certificate.lift_law is not DevelopableLiftLawV1.UNFOLDED_SOURCE_TRIANGLES_V1
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
    _check_judgement(issues, path, metric, certificate)
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
    """

    from .planar_metric import LADDER_TRIGGER_OUTCOMES, PLANE_NORMAL_UNDEFINED_TRACE

    allowed = {item.value for item in LADDER_TRIGGER_OUTCOMES} | {
        PLANE_NORMAL_UNDEFINED_TRACE
    }
    trace = certificate.previous_refusals
    if not trace or any(item not in allowed for item in trace):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("previous_refusals",),
            "an unfolded chart is tried only after a named near-planar refusal: "
            "the ladder trace must name a ladder trigger",
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
    if any(
        item.domain_coordinate.x.denominator != 1
        or item.domain_coordinate.y.denominator != 1
        for item in metric.exact_source_vertex_coordinates
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("exact_source_vertex_coordinates",),
            "unfolded chart coordinates are integer lattice nodes",
        )
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


def _check_judgement(issues, path, metric, certificate) -> None:
    stretch = certificate.stretch
    if _fraction(stretch.stretch_budget) != DEVELOPABLE_STRETCH_BUDGET:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("stretch", "stretch_budget"),
            "recorded stretch budget is not the one the law declares",
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


def validate_developable_recomputation(
    metric,
    *,
    source_vertices,
    source_faces,
    surface_triangles,
    owner_patch_id,
) -> tuple[ValidationIssue, ...]:
    """Построить карту и сертификат заново и сравнить с записью на равенство."""

    from .planar_metric import PlanarMetricAdmissionError
    from .validation_metric import _source_embedding_inputs, position_under_grid_law

    certificate = metric.planarity_certificate
    if type(certificate) is not DevelopableUnfoldCertificateV1:
        return ()
    issues: list[ValidationIssue] = []
    path = ("RationalAffinePlanarMetricV2", "planarity_certificate")
    if any(not isinstance(item.position, LocalPoint3V1) for item in source_vertices):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path,
            "an unfolded chart cannot be recomputed without local coordinates",
        )
        return tuple(issues)
    faces, required_ids, positions = _source_embedding_inputs(
        source_vertices=source_vertices,
        source_faces=source_faces,
        owner_patch_id=owner_patch_id,
    )
    snapped = {
        vertex_id: position_under_grid_law(
            LocalPoint3V1(*(float(axis) for axis in position)),
            metric.grid_certificate,
        )
        for vertex_id, position in positions.items()
    }
    face_ids = {face.face_id for face in faces}
    grid = metric.grid_certificate
    try:
        expected = build_developable_chart(
            source_revision=metric.source_revision,
            patch_domain_id=metric.patch_domain_id,
            snapped=snapped,
            owner_triangles=tuple(
                item for item in surface_triangles if item.source_face_id in face_ids
            ),
            required_ids=required_ids,
            source_scale=grid.source_scale if grid.snapping_law.snaps_source else None,
            previous_refusals=certificate.previous_refusals,
        )
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
