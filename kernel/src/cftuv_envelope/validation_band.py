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

from ._band_chart import rim_to_wall_gap_squared
from .chart_band import request_chart_policy
from .contracts.metric import (
    BandBoundaryRoleV1,
    BandSupportLawV1,
    DevelopableBandChartCertificateV1,
    chart_reach_cap_is_lawful,
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
    recorded = _fraction(certificate.chart_reach_margin_squared)
    if margin != recorded or recorded < cap * cap:
        add_issue(
            issues,
            ValidationCode.SURFACE_METRIC,
            path + ("chart_reach_margin_squared",),
            "the recorded chart distance from the rim to the reach wall differs from the chart coordinates, or "
            "is shorter than the reach cap",
        )


def check_band_policy(issues, path, certificate, policy, domain_use_ids) -> None:
    """Запись полосы записана под досягаемостью и выбором запроса, а alpha запроса не выше досягаемости."""

    cap = _fraction(certificate.reach_cap)
    if cap != policy.reach_cap:
        add_issue(
            issues,
            ValidationCode.POLICY_MISMATCH,
            path + ("reach_cap",),
            f"recorded={cap} request={policy.reach_cap}: the band was recorded under another chart_reach_cap",
        )
    chosen = frozenset(policy.selected_chain_use_ids) & frozenset(domain_use_ids)
    if certificate.selected_chain_use_ids != chosen:
        add_issue(
            issues,
            ValidationCode.POLICY_MISMATCH,
            path + ("selected_chain_use_ids",),
            "the band was recorded for other ChainUses than the request selects in this domain",
        )
    if policy.requested_alpha is not None and policy.requested_alpha > cap:
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
        check_band_policy(
            issues,
            ("surface_metric_descriptors", str(descriptor.patch_domain_id), "planarity_certificate"),
            certificate,
            policy,
            domain_uses,
        )
    return tuple(issues)
