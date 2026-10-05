"""Проверки метрики с сертификатом ПОЛОСЫ: по проводу и против политики запроса.

Пересчёт полосы из снапшота (носитель, развёртка, граница, запас) делает `validation_developable.
validate_developable_recomputation` тем же построителем (`_band_chart.build_band_chart`), а здесь две проверки, которым
построитель не нужен.

1. ПО ПРОВОДУ (`check_band_certificate`): запись сама согласована - закон носителя объявленный, досягаемость законна,
   `support_reach = (1 + b) * cap` по ЗАПИСАННОМУ допуску растяжения, граница носителя замкнута и не выходит за вершины
   носителя, а запас до стены досягаемости пересчитан по координатам САМОЙ метрики (точно) и не меньше `cap^2`.
2. ПРОТИВ ЗАПРОСА (`check_band_policy`): запись снапшота обязана быть записана под досягаемостью и выбором ЗАПРОСА, а
   alpha запроса не больше досягаемости. Без запроса (проверка выгрузки хоста) запись судится под собственной досягаемостью
   и собственным выбором; связать их с запросом дело `validate_snapshot_request_references`.
"""

from __future__ import annotations

from fractions import Fraction

from ._annulus_cut import SEAM_RESIDUAL_BOUND
from ._band_chart import cut_measurements, rim_to_wall_gap_squared
from .chart_band import BAND_TIGHTEN_OUTCOMES, request_chart_policy
from .contracts.metric import (
    BandBoundaryRoleV1,
    BandSupportLawV1,
    DevelopableBandChartCertificateV1,
    band_is_reach_limited,
    chart_reach_cap_is_lawful,
    silhouette_uv_slide_is_lawful,
)
from .validation_issues import ValidationCode, add_issue


def _fraction(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def check_band_certificate(issues, path, metric, certificate) -> None:
    """Запись полосы согласована сама с собой и с координатами метрики."""

    cap = _fraction(certificate.reach_cap)
    budget = _fraction(certificate.stretch.stretch_budget)
    if certificate.support_law is not BandSupportLawV1.FACES_WITHIN_EUCLIDEAN_REACH_V1:
        add_issue(issues, ValidationCode.SURFACE_METRIC, path + ("support_law",), "undeclared band support law")
    if not chart_reach_cap_is_lawful(cap):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("reach_cap",),
            "the band reach cap is not a lawful chart_reach_cap",
        )
    if _fraction(certificate.support_reach) != (1 + budget) * cap:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("support_reach",),
            "the support reach is not (1 + stretch budget) * reach cap",
        )
    if certificate.tightened is not None and certificate.tightened.refused_outcome not in {
        item.value for item in BAND_TIGHTEN_OUTCOMES
    }:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("tightened", "refused_outcome"),
            "a band is tightened only after a refusal that a narrower support can cure (seam residual or stretch)",
        )
    sides = certificate.strip_boundary
    vertices = set(certificate.source_vertex_ids)
    closed = all(
        item.end_vertex_id == following.start_vertex_id
        for item, following in zip(sides, sides[1:] + sides[:1])
    )
    if not closed or any(
        item.start_vertex_id not in vertices or item.end_vertex_id not in vertices for item in sides
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("strip_boundary",),
            "the support boundary is one closed loop of sides over the support vertices",
        )
        return
    if not any(item.role is BandBoundaryRoleV1.RIM for item in sides):
        add_issue(issues, ValidationCode.SURFACE_METRIC, path + ("strip_boundary",), "the band has no rim side")
        return
    nodes = {
        item.source_vertex_id: (_fraction(item.domain_coordinate.x), _fraction(item.domain_coordinate.y))
        for item in metric.exact_source_vertex_coordinates
    }
    if any(item.start_vertex_id not in nodes or item.end_vertex_id not in nodes for item in sides):
        return
    margin = rim_to_wall_gap_squared(nodes, sides, certificate.chart_scale)
    limited = band_is_reach_limited(certificate)
    recorded = None if certificate.chart_reach_margin_squared is None else _fraction(certificate.chart_reach_margin_squared)
    if margin != recorded or (limited and (recorded is None or recorded < cap * cap)):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("chart_reach_margin_squared",),
            "the recorded chart distance from the rim to the reach wall differs from the chart coordinates, or "
            "is shorter than the reach cap",
        )
    if certificate.cut is not None:
        check_band_cut(issues, path, certificate, nodes)


def check_band_cut(issues, path, certificate, nodes) -> None:
    """Разрез кольца записан честно: путь, голономия и оба числа пересчитаны по координатам САМОЙ метрики, судья не против."""

    from .contracts.metric import CUT_RIGHT_COPY_MARK
    from .ids import SourceVertexId

    cut = certificate.cut
    where = path + ("cut",)
    copies = [SourceVertexId(item.value + CUT_RIGHT_COPY_MARK) for item in cut.path_vertex_ids]
    if any(item not in nodes for item in (*cut.path_vertex_ids, *copies)):
        add_issue(issues, ValidationCode.SURFACE_METRIC, where, "a cut path vertex has no chart coordinates for both copies")
        return
    if any(item.triangle_id not in certificate.support_triangle_ids for item in cut.right_corners):
        add_issue(issues, ValidationCode.SURFACE_METRIC, where, "a right copy of the cut is a corner of a triangle outside the support")
    wall_edges = {
        role: [item.physical_edge_id for item in certificate.strip_boundary if item.role is role]
        for role in (BandBoundaryRoleV1.CUT_LEFT, BandBoundaryRoleV1.CUT_RIGHT)
    }
    if sorted(item.value for item in wall_edges[BandBoundaryRoleV1.CUT_LEFT]) != sorted(
        item.value for item in wall_edges[BandBoundaryRoleV1.CUT_RIGHT]
    ):
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            where,
            "the left and the right wall of the cut do not run along the same physical edges",
        )
    try:
        cosine, sine, shift_x, shift_y, residual, deviation, passes, counted = cut_measurements(
            cut.path_vertex_ids,
            nodes,
            certificate.chart_scale,
            certificate.strip_boundary,
            _fraction(certificate.reach_cap) if band_is_reach_limited(certificate) else None,
        )
    except Exception as error:  # a refusal of the measurement is a forged record, not a crash of the validator
        add_issue(issues, ValidationCode.SURFACE_METRIC, where, f"the cut cannot be measured on the chart: {error}")
        return
    measured = (cosine, sine, shift_x, shift_y, residual, deviation)
    recorded = tuple(
        _fraction(item)
        for item in (
            cut.rotation_cosine,
            cut.rotation_sine,
            cut.translation_x,
            cut.translation_y,
            cut.seam_residual_squared,
            cut.bisector_deviation_sine_squared,
        )
    )
    if measured != recorded or counted != cut.seam_vertex_count:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            where,
            "the recorded holonomy, seam residual or bisector deviation differs from the chart coordinates",
        )
    if residual > SEAM_RESIDUAL_BOUND * SEAM_RESIDUAL_BOUND or not passes:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            where,
            "PERIODIC_CUT_SEAM_RESIDUAL or PERIODIC_CUT_BISECTOR_DEVIATION is beyond its registered bound",
        )


def check_band_policy(issues, path, certificate, policy, domain_use_ids) -> None:
    """Запись полосы записана под досягаемостью и выбором запроса, а alpha запроса не выше досягаемости."""

    cap = _fraction(certificate.reach_cap)
    # Суженная карта записана под досягаемостью `alpha * (1 + b)`, а запрос несёт запрошенную (`tightened`): сверяется она.
    asked = _fraction(certificate.tightened.requested_reach_cap) if certificate.tightened is not None else cap
    if asked != policy.reach_cap:
        add_issue(
            issues,
            ValidationCode.POLICY_MISMATCH,
            path + ("reach_cap",),
            f"recorded={asked} request={policy.reach_cap}: the band was recorded under another chart_reach_cap",
        )
    chosen = frozenset(policy.selected_chain_use_ids) & frozenset(domain_use_ids)
    if certificate.selected_chain_use_ids != chosen:
        add_issue(
            issues,
            ValidationCode.POLICY_MISMATCH,
            path + ("selected_chain_use_ids",),
            "the band was recorded for other ChainUses than the request selects in this domain",
        )
    if band_is_reach_limited(certificate) and policy.requested_alpha is not None and policy.requested_alpha > cap:
        add_issue(
            issues,
            ValidationCode.POLICY_MISMATCH,
            path + ("requested_alpha",),
            f"REQUEST_ALPHA_EXCEEDS_CHART_REACH: alpha={float(policy.requested_alpha):.6g} m is beyond the "
            f"chart reach cap {float(cap):.6g} m of the band chart",
        )


def chart_reach_cap_issues(request) -> tuple:
    """Досягаемость запроса законна: положительна и не выше `MAX_CHART_REACH_CAP`."""

    if chart_reach_cap_is_lawful(_fraction(request.chart_reach_cap)):
        return ()
    issues: list = []
    add_issue(issues, ValidationCode.POLICY_MISMATCH, ("chart_reach_cap",), "chart reach cap must lie in (0, 100] m")
    return tuple(issues)


def silhouette_uv_slide_issues(request) -> tuple:
    """Сдвиг UV закона `SILHOUETTE_TOPOLOGY_V1` законен: положителен и не выше `MAX_SILHOUETTE_UV_SLIDE` (доля alpha)."""

    if silhouette_uv_slide_is_lawful(_fraction(request.silhouette_uv_slide)):
        return ()
    issues: list = []
    add_issue(issues, ValidationCode.POLICY_MISMATCH, ("silhouette_uv_slide",), "silhouette UV slide must lie in (0, 1/16] of alpha")
    return tuple(issues)


def check_cut_corners(issues, path, certificate, snapshot, domain_id) -> None:
    """Угловое отношение в вершине разреза кольца недопустимо: на карте вершина раздвоена, угол разрезан на два со стеной.

    Хост такие углы не пишет (`ANGULAR_CORNERS_AT_RING_CUT`); снапшот, который их несёт, читал бы одну из двух копий и
    тихо описывал бы не тот угол.
    """

    on_path = frozenset(certificate.cut.path_vertex_ids)
    sectors = {item.owner_sector_id: item for item in snapshot.angular_owner_sectors}
    for relation in snapshot.corner_relations:
        sector = sectors.get(relation.owner_sector_id)
        if relation.source_vertex_id in on_path and sector is not None and sector.patch_domain_id == domain_id:
            add_issue(
                issues,
                ValidationCode.SURFACE_METRIC,
                path + ("cut", "path_vertex_ids"),
                f"ANGULAR_CORNERS_AT_RING_CUT: the corner relation at {relation.source_vertex_id.value} lies on the cut "
                "path of the ring band: the vertex is two vertices on the chart and the corner is cut in two",
            )


def band_policy_issues(snapshot, request) -> tuple:
    """Полосы снапшота против политики запроса. Не память снапшота: alpha меняется с ползунком, снапшот нет."""

    issues: list = []
    policy = request_chart_policy(request)
    for descriptor in snapshot.surface_metric_descriptors:
        certificate = getattr(descriptor, "planarity_certificate", None)
        if type(certificate) is not DevelopableBandChartCertificateV1:
            continue
        domain_uses = {
            item.chain_use_id for item in snapshot.chain_uses if item.patch_domain_id == descriptor.patch_domain_id
        }
        where = ("surface_metric_descriptors", str(descriptor.patch_domain_id), "planarity_certificate")
        check_band_policy(issues, where, certificate, policy, domain_uses)
        if certificate.cut is not None:
            check_cut_corners(issues, where, certificate, snapshot, descriptor.patch_domain_id)
    return tuple(issues)
