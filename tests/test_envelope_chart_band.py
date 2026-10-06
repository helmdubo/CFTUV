"""Полосовая карта на стороне хоста: политика запроса, выделение, кэш метрики по выделению, отказ по alpha.

Само построение полосы - дело ядра (`kernel/tests/test_developable_band.py`). Здесь проверяется то, что решает хост:
какая политика едет в запросе и в ключах кэшей, какие цепи домена попадают в полосу, после каких отказов она
пробуется и что метрика целого патча не пересобирается из-за смены выделения.
"""

from __future__ import annotations

import sys
from decimal import Decimal
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

import cftuv.envelope_chart_band as chart_band  # noqa: E402
from cftuv import envelope_debug_session as session  # noqa: E402
from cftuv.envelope_metric_export import EnvelopePatchMetricExportV1, band_key_of  # noqa: E402
from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeDebugHostOutcome,
    EnvelopeHostAdapterError,
    METRIC_STAGE_OUTCOMES,
)
from cftuv.envelope_request_policy import (  # noqa: E402
    DEFAULT_ENVELOPE_CHART_REACH_CAP,
    build_envelope_request_contract,
    envelope_angular_policy,
    envelope_chart_reach_cap,
    envelope_decal_request_id_value,
    envelope_request_policy_signature,
    topology_chart_reach_cap,
)
from cftuv.envelope_topology_export import ChartBandPolicyV1, EnvelopeTopologyExportV1  # noqa: E402


def _request(cap=None):
    import cftuv_envelope as kernel

    return build_envelope_request_contract(
        kernel,
        kernel.DecalRequestId("request"),
        frozenset(),
        Decimal("0.25"),
        envelope_angular_policy(kernel, None, None, cap),
    )


# --------------------------------------------------------------------------
# Политика запроса
# --------------------------------------------------------------------------


def test_the_default_reach_cap_is_the_kernels_and_a_request_without_it_is_the_legacy_request():
    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.metric import DEFAULT_CHART_REACH_CAP

    assert DEFAULT_ENVELOPE_CHART_REACH_CAP == DEFAULT_CHART_REACH_CAP == Fraction(1, 2)
    request = _request()
    assert request.chart_reach_cap == kernel.ExactRationalV1(1, 2)
    assert b"chart_reach_cap" not in kernel.DecalRequestCodecV1.dumps(request)
    assert _request(DEFAULT_ENVELOPE_CHART_REACH_CAP) == request
    assert envelope_chart_reach_cap(Fraction(1, 2)) is None
    assert envelope_chart_reach_cap(Fraction(3, 4)) == Fraction(3, 4)
    with pytest.raises(ValueError, match="positive length"):
        envelope_chart_reach_cap(Fraction(0))


def test_a_non_default_reach_cap_reaches_the_request_the_policy_signature_and_the_request_id():
    import cftuv_envelope as kernel

    wide_policy = envelope_angular_policy(kernel, None, None, Fraction(3, 4))
    default_policy = envelope_angular_policy(kernel, None)
    wide, default = _request(Fraction(3, 4)), _request()
    assert wide.chart_reach_cap == kernel.ExactRationalV1(3, 4)
    assert envelope_request_policy_signature(default) + ("reach=3/4",) == envelope_request_policy_signature(wide)

    def typed(*parts):
        return "|".join(str(item) for item in parts)

    assert envelope_decal_request_id_value(typed, "rev", (), "base", default_policy) == "base"
    assert envelope_decal_request_id_value(typed, "rev", (), "base", wide_policy) != "base"
    stretch_and_reach = envelope_angular_policy(kernel, None, Fraction(7, 20), Fraction(3, 4))
    assert len(
        {
            envelope_decal_request_id_value(typed, "rev", (), "base", policy)
            for policy in (default_policy, wide_policy, stretch_and_reach)
        }
    ) == 3


def test_the_topology_export_carries_the_band_policy_without_changing_the_whole_patch_identity():
    export = EnvelopeTopologyExportV1("rev", None, (), {0: "domain0"})
    assert export.chart_band is None and topology_chart_reach_cap(export) is None
    banded = export.with_chart_band(None, {3, 4})
    assert banded.chart_band.reach_cap == DEFAULT_ENVELOPE_CHART_REACH_CAP
    assert banded.chart_band.selected_physical_edge_ids == frozenset({3, 4})
    assert topology_chart_reach_cap(banded) is None
    assert topology_chart_reach_cap(export.with_chart_band(Fraction(3, 4), {3})) == Fraction(3, 4)
    assert banded.with_chart_band(None, [4, 3]) is banded
    assert banded.without_chart_band() == export


# --------------------------------------------------------------------------
# Вход ядра и исходы
# --------------------------------------------------------------------------


def test_the_band_trigger_outcomes_of_the_host_are_the_kernels_and_the_band_refusals_are_metric_stage():
    from cftuv_envelope.planar_metric import BAND_TRIGGER_OUTCOMES

    assert chart_band.band_trigger_host_outcomes() == frozenset(
        EnvelopeDebugHostOutcome(item.value) for item in BAND_TRIGGER_OUTCOMES
    )
    for name in (
        "DEVELOPABLE_BAND_SUPPORT_DISCONNECTED",
        "CHART_REACH_SHORT_OF_CAP",
        "DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED",
        # Разрез кольца: три отказа метрики по своим именам, а не одно безымянное `PIPELINE_STAGE_FAILED`.
        "PERIODIC_CUT_PATH_UNAVAILABLE",
        "PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED",
        "PERIODIC_CUT_BISECTOR_DEVIATION_EXCEEDED",
    ):
        assert EnvelopeDebugHostOutcome(name) in METRIC_STAGE_OUTCOMES
    # Отказ по alpha - отказ ЗАПРОСА, а не метрики: метрика полосы одна на любую alpha до досягаемости.
    assert EnvelopeDebugHostOutcome.REQUEST_ALPHA_EXCEEDS_CHART_REACH not in METRIC_STAGE_OUTCOMES


def _chain_objects():
    import cftuv_envelope as kernel
    from cftuv_envelope.contracts.analysis import (
        ChainUseOrientation,
        ChainUseRole,
        LaunchLocusV1,
        OwnerInteriorDirection,
        PhysicalChainKind,
    )
    from cftuv_envelope.ids import BoundaryConstraintId, BoundaryLoopId, PatchId

    vertices = [kernel.SourceVertexId(f"v{index}") for index in range(5)]
    edges = [kernel.PhysicalEdgeId(f"edge{index}") for index in range(4)]
    chains, uses = [], []
    for index in range(2):
        chain_id = kernel.PhysicalChainId(f"chain{index}")
        chains.append(
            kernel.PhysicalChainV1(
                chain_id,
                PhysicalChainKind.PHYSICAL_DECAL_SOURCE,
                False,
                tuple(vertices[2 * index : 2 * index + 3]),
                tuple(edges[2 * index : 2 * index + 2]),
                frozenset(),
                frozenset(),
            )
        )
        uses.append(
            kernel.ChainUseV1(
                kernel.ChainUseId(f"use{index}"),
                chain_id,
                PatchId("patch"),
                kernel.PatchDomainId("domain"),
                BoundaryLoopId("loop"),
                ChainUseOrientation.A_START_TO_END,
                frozenset({ChainUseRole.DOMAIN_BOUNDARY}),
                LaunchLocusV1(BoundaryConstraintId(f"launch{index}"), OwnerInteriorDirection.OWNER_INTERIOR, True),
            )
        )
    return {item: edge for item, edge in enumerate(edges)}, chains, uses


def test_a_chain_is_selected_for_the_band_by_any_of_its_selected_edges_and_only_in_its_domain():
    edge_ids, chains, uses = _chain_objects()
    import cftuv_envelope as kernel

    domain = kernel.PatchDomainId("domain")
    policy = ChartBandPolicyV1(Fraction(1, 2), frozenset({1}))
    band = chart_band.chart_band_request(policy, edge_ids, chains, uses, domain)
    assert band.selected_chain_use_ids == frozenset({kernel.ChainUseId("use0")})
    assert len(band.rim_edges) == 2 and len(band.boundary_uses) == 4
    assert band.reach_cap == Fraction(1, 2)
    nothing = ChartBandPolicyV1(Fraction(1, 2), frozenset({99}))
    assert chart_band.chart_band_request(nothing, edge_ids, chains, uses, domain) is None
    assert chart_band.chart_band_request(None, edge_ids, chains, uses, domain) is None
    other_domain = kernel.PatchDomainId("other")
    assert chart_band.chart_band_request(policy, edge_ids, chains, uses, other_domain) is None


def test_an_alpha_beyond_the_reach_cap_of_a_band_chart_is_a_named_host_refusal(monkeypatch):
    class Certificate:
        reach_cap = SimpleNamespace(numerator=1, denominator=2)
        excluded_triangle_count = 4

    class WholeRing(Certificate):
        excluded_triangle_count = 0

    snapshot = SimpleNamespace(
        surface_metric_descriptors=(
            SimpleNamespace(planarity_certificate=Certificate(), patch_domain_id=SimpleNamespace(value="domain0")),
            SimpleNamespace(planarity_certificate=object(), patch_domain_id=SimpleNamespace(value="domain1")),
        )
    )
    monkeypatch.setattr(chart_band, "band_certificate_type", lambda: Certificate)
    chart_band.refuse_alpha_beyond_reach(snapshot, Decimal("0.5"))
    with pytest.raises(EnvelopeHostAdapterError) as refused:
        chart_band.refuse_alpha_beyond_reach(snapshot, Decimal("0.75"))
    assert refused.value.outcome is EnvelopeDebugHostOutcome.REQUEST_ALPHA_EXCEEDS_CHART_REACH
    assert refused.value.patch_domain_id == "domain0"
    assert "0.75" in str(refused.value) and "0.5" in str(refused.value)
    # Кольцо, целиком лежащее в досягаемости и разрезанное по пути, стены досягаемости не имеет: alpha не ограничена.
    ring = SimpleNamespace(
        surface_metric_descriptors=(
            SimpleNamespace(planarity_certificate=WholeRing(), patch_domain_id=SimpleNamespace(value="ring")),
        )
    )
    monkeypatch.setattr(chart_band, "band_certificate_type", lambda: Certificate)
    chart_band.refuse_alpha_beyond_reach(ring, Decimal("0.75"))


# --------------------------------------------------------------------------
# Кэш метрики: целый патч от выделения не зависит, полоса зависит
# --------------------------------------------------------------------------


def _record(patch_id, edges):
    return SimpleNamespace(patch_id=patch_id, canonical_edge_ids=tuple(edges))


def _export(selected=None, alpha=None):
    export = EnvelopeTopologyExportV1(
        "rev", None, (_record(0, (1, 2, 3)), _record(1, (7, 8))), {0: "domain0", 1: "domain1"}
    )
    return export if selected is None else export.with_chart_band(None, selected, alpha)


def _fake_exports(monkeypatch, outcome):
    """Метрика целого патча отказывает `outcome`; метрика-полоса (политика названа) собирается и считается."""

    calls = []

    def build(topology_export, patch_id, profile=None):
        banded = topology_export.chart_band is not None
        calls.append((int(patch_id), banded))
        if not banded:
            raise EnvelopeHostAdapterError(outcome, "the whole patch is refused", patch_domain_id="domain0")
        return EnvelopePatchMetricExportV1(
            topology_export.source_revision_value,
            int(patch_id),
            topology_export.patch_domain_id_by_patch[int(patch_id)],
            None,
            SimpleNamespace(),
            topology_export.developable_stretch_budget,
            band_key_of(topology_export, patch_id),
        )

    monkeypatch.setattr(session, "build_envelope_patch_metric_export", build)
    return calls


def test_the_band_key_holds_the_reach_cap_and_only_the_selected_edges_of_the_patch():
    assert band_key_of(_export(), 0) is None
    key = band_key_of(_export({2, 8, 99}), 0)
    assert key == (Fraction(1, 2), frozenset({2}))
    assert band_key_of(_export({2, 8, 99}), 1) == (Fraction(1, 2), frozenset({8}))


def test_a_whole_patch_refusal_from_the_triggers_leads_to_one_band_per_selection(monkeypatch):
    calls = _fake_exports(monkeypatch, EnvelopeDebugHostOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED)
    controller = session.EnvelopeDebugSessionController()
    first = controller.get_patch_metric(_export({2}), 0)
    assert first.band_key == (Fraction(1, 2), frozenset({2}))
    assert controller.get_patch_metric(_export({2}), 0) is first
    # Выбор в ДРУГОМ патче полосу этого патча не пересобирает.
    assert controller.get_patch_metric(_export({2, 8}), 0) is first
    assert calls == [(0, False), (0, True)]
    second = controller.get_patch_metric(_export({3}), 0)
    assert second is not first and second.band_key[1] == frozenset({3})
    # Метрика целого патча при смене выделения не пересобирается: отказ запомнен под ключом без выделения.
    assert calls == [(0, False), (0, True), (0, True)]
    assert controller.build_counts["PATCH_METRIC"] == 3


def test_a_refusal_outside_the_triggers_or_without_a_band_policy_stands(monkeypatch):
    calls = _fake_exports(monkeypatch, EnvelopeDebugHostOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED)
    controller = session.EnvelopeDebugSessionController()
    with pytest.raises(EnvelopeHostAdapterError) as refused:
        controller.get_patch_metric(_export({2}), 0)
    assert refused.value.outcome is EnvelopeDebugHostOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    assert calls == [(0, False)]
    calls = _fake_exports(monkeypatch, EnvelopeDebugHostOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED)
    with pytest.raises(EnvelopeHostAdapterError) as unbanded:
        session.EnvelopeDebugSessionController().get_patch_metric(_export(), 0)
    assert unbanded.value.outcome is EnvelopeDebugHostOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert calls == [(0, False)]


def test_a_band_refusal_is_remembered_under_the_band_key_and_raised_by_name(monkeypatch):
    def build(topology_export, patch_id, profile=None):
        outcome = (
            EnvelopeDebugHostOutcome.CHART_REACH_SHORT_OF_CAP
            if topology_export.chart_band is not None
            else EnvelopeDebugHostOutcome.DEVELOPABLE_CHART_SELF_OVERLAP
        )
        raise EnvelopeHostAdapterError(outcome, "refused", patch_domain_id="domain0")

    monkeypatch.setattr(session, "build_envelope_patch_metric_export", build)
    controller = session.EnvelopeDebugSessionController()
    for _ in range(2):
        with pytest.raises(EnvelopeHostAdapterError) as refused:
            controller.get_patch_metric(_export({2}), 0)
        assert refused.value.outcome is EnvelopeDebugHostOutcome.CHART_REACH_SHORT_OF_CAP
    assert controller.build_counts["PATCH_METRIC"] == 2


def test_the_domain_geometry_of_a_band_is_cached_apart_from_the_whole_patch_geometry(monkeypatch):
    _fake_exports(monkeypatch, EnvelopeDebugHostOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED)
    controller = session.EnvelopeDebugSessionController()
    one = controller.get_patch_metric(_export({2}), 0)
    two = controller.get_patch_metric(_export({3}), 0)
    assert controller.get_domain_geometry(one) is not controller.get_domain_geometry(two)
    assert controller.get_domain_geometry(one) is controller.get_domain_geometry(one)


def test_chart_points_name_a_vertex_beyond_the_reach_and_leave_ordinary_frames_plain(monkeypatch):
    class Certificate:
        pass

    banded = SimpleNamespace(planarity_certificate=Certificate())
    ordinary = SimpleNamespace(planarity_certificate=object())
    monkeypatch.setattr(chart_band, "band_certificate_type", lambda: Certificate)
    points = {"a": (0, 0)}
    assert chart_band.chart_points(points, ordinary) is points
    charted = chart_band.chart_points(points, banded)
    assert charted["a"] == (0, 0)
    with pytest.raises(chart_band.BeyondChartReach):
        charted["b"]
    with pytest.raises(KeyError) as plain:
        points["b"]
    assert not isinstance(plain.value, chart_band.BeyondChartReach)


# --------------------------------------------------------------------------
# Кольцо с разрезом: имена карты у вершин пути
# --------------------------------------------------------------------------


def _ring_frame():
    """Рамка кольца с разрезом: вершина `v` на пути, правая копия `v|cut:R`, ребро обода `v -> w` стоит на правой копии."""

    from cftuv_envelope.contracts.metric import BandBoundaryRoleV1, BandCutCornerV1
    from cftuv_envelope.ids import SourceVertexId as Vertex, SurfaceTriangleId as Triangle

    def side(role, start, end, use="use"):
        return SimpleNamespace(role=role, start_vertex_id=Vertex(start), end_vertex_id=Vertex(end), chain_use_id=use)

    cut = SimpleNamespace(
        path_vertex_ids=(Vertex("v"), Vertex("p")),
        right_corners=frozenset({BandCutCornerV1(Triangle("t_right"), Vertex("v"))}),
    )
    certificate = SimpleNamespace(
        cut=cut,
        strip_boundary=(
            side(BandBoundaryRoleV1.RIM, "v|cut:R", "w"),
            side(BandBoundaryRoleV1.CUT_LEFT, "v", "p", None),
            side(BandBoundaryRoleV1.RIM, "u", "v"),
        ),
    )
    return SimpleNamespace(planarity_certificate=certificate), Vertex, Triangle


def test_an_edge_at_the_ring_cut_names_its_own_copy_of_the_cut_vertex():
    frame, Vertex, _Triangle = _ring_frame()
    # Ребро обода из вершины разреза начинается на правой копии, а входящее в неё кончается на левой.
    assert chart_band.chart_edge_ends(frame, Vertex("v"), Vertex("w")) == (Vertex("v|cut:R"), Vertex("w"))
    assert chart_band.chart_edge_ends(frame, Vertex("w"), Vertex("v")) == (Vertex("w"), Vertex("v|cut:R"))
    assert chart_band.chart_edge_ends(frame, Vertex("u"), Vertex("v")) == (Vertex("u"), Vertex("v"))
    plain = SimpleNamespace(planarity_certificate=SimpleNamespace(cut=None))
    assert chart_band.chart_edge_ends(plain, Vertex("v"), Vertex("w")) == (Vertex("v"), Vertex("w"))
    assert chart_band.cut_path_vertices(frame) == frozenset({Vertex("v"), Vertex("p")})
    assert chart_band.cut_path_vertices(plain) == frozenset()


def test_a_face_cycle_on_the_right_of_the_ring_cut_reads_the_right_copy_of_the_vertex():
    frame, Vertex, Triangle = _ring_frame()
    kernel_vertex = {1: Vertex("v"), 2: Vertex("w"), 3: Vertex("u")}
    triangle_ids = {10: Triangle("t_right"), 11: Triangle("t_left")}
    coordinates = {Vertex("v"): "left", Vertex("v|cut:R"): "right", Vertex("w"): "w", Vertex("u"): "u"}
    right = SimpleNamespace(vertex_cycle=(1, 2, 3), triangle_ids=(10,))
    left = SimpleNamespace(vertex_cycle=(1, 2, 3), triangle_ids=(11,))
    assert chart_band.chart_face_points(coordinates, frame, kernel_vertex, triangle_ids, right) == ("right", "w", "u")
    assert chart_band.chart_face_points(coordinates, frame, kernel_vertex, triangle_ids, left) == ("left", "w", "u")
    plain = SimpleNamespace(planarity_certificate=SimpleNamespace(cut=None))
    assert chart_band.chart_face_points(coordinates, plain, kernel_vertex, triangle_ids, right) == ("left", "w", "u")


# --------------------------------------------------------------------------
# Суженная досягаемость: сессия пересобирает полосу ОДИН раз под `alpha * (1 + b)`
# --------------------------------------------------------------------------

SEAM = EnvelopeDebugHostOutcome.PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED
STRETCH = EnvelopeDebugHostOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED


def test_the_policy_carries_alpha_and_the_tightened_cap_and_the_band_key_tells_the_tightened_map_apart():
    export = _export({2}, Fraction(1, 4))
    assert export.chart_band.alpha == Fraction(1, 4) and export.chart_band.tightened_reach_cap is None
    # alpha в ключ полосы запроса НЕ входит: ползунок не пересобирает кэш метрик карты под досягаемостью запроса.
    request_key = (Fraction(1, 2), frozenset({2}))
    assert band_key_of(export, 0) == band_key_of(_export({2}, Fraction(1, 2)), 0) == band_key_of(_export({2}), 0) == request_key
    narrow = export.with_band_tightened(Fraction(3, 10), SEAM.value)
    assert band_key_of(narrow, 0) == request_key + (("tightened", Fraction(3, 10)),)
    assert export.chart_band.tightened_reach_cap is None
    assert narrow.with_chart_band(None, {2}, Fraction(1, 4)).chart_band.tightened_reach_cap is None
    with pytest.raises(ValueError):
        _export().with_band_tightened(Fraction(3, 10), SEAM.value)
    assert chart_band.policy_alpha(0.25) == Fraction(1, 4) and chart_band.policy_alpha("not a number") is None


def test_the_tightened_export_is_the_decals_own_reach_and_only_after_a_refusal_that_a_narrower_support_cures():
    export = _export({2}, Fraction(1, 4))
    narrow = chart_band.tightened_export(export, SEAM)
    assert narrow.chart_band.tightened_reach_cap == Fraction(3, 10) and narrow.chart_band.tightened_after == SEAM.value
    assert chart_band.tightened_export(narrow, SEAM) is None  # пересборка одна
    assert chart_band.tightened_export(export, STRETCH).chart_band.tightened_reach_cap == Fraction(3, 10)
    assert chart_band.tightened_export(export, EnvelopeDebugHostOutcome.CHART_REACH_SHORT_OF_CAP) is None
    assert chart_band.tightened_export(_export({2}, Fraction(1, 2)), SEAM) is None  # 3/5 не уже запрошенной 1/2
    assert chart_band.tightened_export(_export({2}), SEAM) is None  # alpha неизвестна
    assert chart_band.tightened_export(_export(), SEAM) is None  # политики полосы нет
    wide = export.with_developable_stretch_budget(Fraction(1, 2))  # допуск запроса: 1/4 * 3/2
    assert chart_band.tightened_export(wide, SEAM).chart_band.tightened_reach_cap == Fraction(3, 8)


def test_the_band_request_of_a_tightened_policy_is_built_under_it_and_records_the_requested_reach():
    import cftuv_envelope as kernel

    edge_ids, chains, uses = _chain_objects()
    domain = kernel.PatchDomainId("domain")
    policy = ChartBandPolicyV1(Fraction(1, 2), frozenset({1}), Fraction(1, 4), Fraction(3, 10), SEAM.value)
    band = chart_band.chart_band_request(policy, edge_ids, chains, uses, domain)
    assert band.reach_cap == Fraction(3, 10) and band.requested_reach_cap == Fraction(1, 2)
    assert band.tightened_after == SEAM.value
    plain = chart_band.chart_band_request(ChartBandPolicyV1(Fraction(1, 2), frozenset({1})), edge_ids, chains, uses, domain)
    assert plain.reach_cap == Fraction(1, 2) and plain.requested_reach_cap is None and plain.tightened_after is None


def _seam_exports(monkeypatch, still_refused=None):
    """Целый патч отказывает растяжением, полоса под досягаемостью запроса - швом; суженная собирается (либо отказывает)."""

    calls = []

    def build(topology_export, patch_id, profile=None):
        policy = topology_export.chart_band
        tight = None if policy is None else policy.tightened_reach_cap
        calls.append((int(patch_id), policy is not None, tight))
        if policy is None:
            raise EnvelopeHostAdapterError(STRETCH, "the whole patch is refused", patch_domain_id="domain0")
        if tight is None:
            raise EnvelopeHostAdapterError(SEAM, "the seam residual is over", patch_domain_id="domain0")
        if still_refused is not None:
            raise EnvelopeHostAdapterError(still_refused, "the tightened band is refused too", patch_domain_id="domain0")
        return EnvelopePatchMetricExportV1(
            topology_export.source_revision_value,
            int(patch_id),
            topology_export.patch_domain_id_by_patch[int(patch_id)],
            None,
            SimpleNamespace(),
            topology_export.developable_stretch_budget,
            band_key_of(topology_export, patch_id),
        )

    monkeypatch.setattr(session, "build_envelope_patch_metric_export", build)
    return calls


def test_a_band_refused_by_the_seam_is_rebuilt_once_under_the_decals_own_reach(monkeypatch):
    calls = _seam_exports(monkeypatch)
    controller = session.EnvelopeDebugSessionController()
    metric = controller.get_patch_metric(_export({2}, Fraction(1, 4)), 0)
    assert calls == [(0, False, None), (0, True, None), (0, True, Fraction(3, 10))]
    assert metric.band_key == (Fraction(1, 2), frozenset({2}), ("tightened", Fraction(3, 10)))
    assert controller.get_domain_geometry(metric).band_key == metric.band_key
    # Тот же alpha: ни одной сборки, тот же объект.
    assert controller.get_patch_metric(_export({2}, Fraction(1, 4)), 0) is metric
    assert len(calls) == 3 and controller.build_counts["PATCH_METRIC"] == 3


def test_an_alpha_change_rebuilds_the_tightened_band_under_the_new_reach_and_never_reuses_the_stale_one(monkeypatch):
    calls = _seam_exports(monkeypatch)
    controller = session.EnvelopeDebugSessionController()
    first = controller.get_patch_metric(_export({2}, Fraction(1, 4)), 0)
    wider = controller.get_patch_metric(_export({2}, Fraction(2, 5)), 0)
    # Отказ карты запроса от alpha не зависит и не пересобирается; суженная - под новой точной досягаемостью.
    assert calls[3:] == [(0, True, Fraction(12, 25))]
    assert wider is not first and wider.band_key[-1] == ("tightened", Fraction(12, 25))
    assert controller.get_patch_metric(_export({2}, Fraction(1, 4)), 0) is first  # досягаемость - функция alpha, не истории
    assert len(calls) == 4


def test_an_alpha_whose_own_reach_is_not_narrower_than_the_requested_one_keeps_the_named_refusal(monkeypatch):
    calls = _seam_exports(monkeypatch)
    controller = session.EnvelopeDebugSessionController()
    for alpha in (Fraction(1, 2), None):
        with pytest.raises(EnvelopeHostAdapterError) as refused:
            controller.get_patch_metric(_export({2}, alpha), 0)
        assert refused.value.outcome is SEAM
    assert calls == [(0, False, None), (0, True, None)]


def test_a_tightened_band_that_is_refused_too_keeps_its_named_refusal_and_is_not_tried_again(monkeypatch):
    calls = _seam_exports(monkeypatch, still_refused=EnvelopeDebugHostOutcome.CHART_REACH_SHORT_OF_CAP)
    controller = session.EnvelopeDebugSessionController()
    for _ in range(2):
        with pytest.raises(EnvelopeHostAdapterError) as refused:
            controller.get_patch_metric(_export({2}, Fraction(1, 4)), 0)
        assert refused.value.outcome is EnvelopeDebugHostOutcome.CHART_REACH_SHORT_OF_CAP
        assert str(refused.value).startswith("[tightened: reach cap 0.5 m -> 0.3 m after PERIODIC_CUT_SEAM_RESIDUAL_EXCEEDED]")
    assert calls == [(0, False, None), (0, True, None), (0, True, Fraction(3, 10))]


def test_the_preparations_of_a_domain_leave_the_session_when_its_tightened_reach_changes(monkeypatch):
    _seam_exports(monkeypatch)
    controller = session.EnvelopeDebugSessionController()
    request = _request()
    selected = frozenset({2})
    controller.get_patch_metric(_export({2}, Fraction(1, 4)), 0)
    controller.get_patch_metric(_export({7}, Fraction(1, 4)), 1)
    controller.get_conveyor_preparation("rev", "domain0", selected, request, lambda: "prepared0")
    controller.get_conveyor_preparation("rev", "domain1", frozenset({7}), request, lambda: "prepared1")
    # Тот же alpha - подготовки на месте; чужая alpha у ДРУГОГО домена его подготовку не трогает.
    controller.get_patch_metric(_export({2}, Fraction(1, 4)), 0)
    assert controller.peek_conveyor_preparation("rev", "domain0", selected, request) == "prepared0"
    controller.get_patch_metric(_export({2}, Fraction(2, 5)), 0)
    assert controller.peek_conveyor_preparation("rev", "domain0", selected, request) is None
    assert controller.peek_conveyor_preparation("rev", "domain1", frozenset({7}), request) == "prepared1"


def _tightened_certificate(cap):
    class Certificate:
        reach_cap = SimpleNamespace(numerator=cap.numerator, denominator=cap.denominator)
        excluded_triangle_count = 4
        tightened = object()

    class Plain(Certificate):
        tightened = None

    return Certificate, Plain


def _snapshot_of(certificate):
    return SimpleNamespace(
        surface_metric_descriptors=(
            SimpleNamespace(planarity_certificate=certificate(), patch_domain_id=SimpleNamespace(value="domain0")),
        )
    )


def test_a_tightened_map_is_current_only_under_the_reach_its_own_alpha_asks_for(monkeypatch):
    tightened, plain = _tightened_certificate(Fraction(3, 10))
    monkeypatch.setattr(chart_band, "band_certificate_type", lambda: tightened)
    stored = _snapshot_of(tightened)
    assert chart_band.tightened_cap_of(stored) == Fraction(3, 10) and chart_band.tightened_cap_of(_snapshot_of(plain)) is None
    assert chart_band.tightening_is_current(_export({2}, Fraction(1, 4)), stored)
    # alpha выросла (карта короче нужной), упала (карта шире нужной: другой носитель, другой ответ) либо суженной уже нет.
    for alpha in (Fraction(2, 5), Fraction(1, 5), Fraction(1, 2), None):
        assert not chart_band.tightening_is_current(_export({2}, alpha), stored)
    assert not chart_band.tightening_is_current(_export(), stored)
    # Карта под досягаемостью запроса от alpha не зависит: годится любой.
    for export in (_export({2}, Fraction(2, 5)), _export({2}), _export()):
        assert chart_band.tightening_is_current(export, _snapshot_of(plain))


def _content_run(controller, alpha):
    from cftuv.envelope_production_export import PRODUCTION_TOPOLOGY_LAW, PRODUCTION_UV_POLICY

    return SimpleNamespace(
        controller=controller,
        topology_export=_export({2}, Fraction(alpha)),
        revision="rev",
        density=2,
        alpha=float(Fraction(alpha)),
        alpha_text=str(float(Fraction(alpha))),
        uv_policy_id=PRODUCTION_UV_POLICY,
        topology_law=PRODUCTION_TOPOLOGY_LAW,
        backend="PYTHON",
        backend_id="PYTHON",
        registered=[],
    )


def test_a_stored_preparation_on_a_map_tightened_for_another_alpha_is_dropped_not_reused(monkeypatch):
    import cftuv.envelope_content_key as content_key
    from cftuv import envelope_production_export as production

    tightened, _plain = _tightened_certificate(Fraction(3, 10))
    monkeypatch.setattr(chart_band, "band_certificate_type", lambda: tightened)
    monkeypatch.setattr(content_key, "domain_content_key", lambda export, selected, band, backend: "content-key")
    controller = session.EnvelopeDebugSessionController()
    prepared = SimpleNamespace(context=SimpleNamespace(snapshot=_snapshot_of(tightened)))

    def entry(alpha):
        return production._content_entry(_content_run(controller, alpha), 0, "domain0", frozenset({2}), object())

    controller.content_store.register_preparation("content-key", prepared, object())
    same = entry(Fraction(1, 4))
    assert same.reuse == "preparation" and same.prepared is prepared and same.failure is None
    # alpha другая: ключ содержимого тот же, но карта сужена под прежнюю alpha - запись убирается, домен считается заново.
    for alpha in (Fraction(2, 5), Fraction(1, 5)):
        controller.content_store.register_preparation("content-key", prepared, object())
        stale = entry(alpha)
        assert stale.prepared is None and not stale.reuse and stale.content_key == "content-key"
        assert controller.content_store.find("content-key") is None


def test_a_new_preparation_on_another_tightened_map_replaces_the_stored_one_and_an_equal_one_leaves_it(monkeypatch):
    from cftuv import envelope_production_export as production

    class Certificate:
        tightened = object()
        excluded_triangle_count = 0

        def __init__(self, cap):
            self.reach_cap = SimpleNamespace(numerator=cap.numerator, denominator=cap.denominator)

    def prepared(cap):
        metric = SimpleNamespace(planarity_certificate=Certificate(cap), patch_domain_id=SimpleNamespace(value="domain0"))
        return SimpleNamespace(context=SimpleNamespace(snapshot=SimpleNamespace(surface_metric_descriptors=(metric,))))

    monkeypatch.setattr(chart_band, "band_certificate_type", lambda: Certificate)
    controller = session.EnvelopeDebugSessionController()
    store = controller.content_store
    request = _request()
    run = _content_run(controller, Fraction(2, 5))
    entry = SimpleNamespace(content_key="content-key", patch_id=0, domain_id="domain0", selected=frozenset({2}))
    result = SimpleNamespace(labels=object(), outcome="MATERIALIZED")
    old, new, same = prepared(Fraction(3, 10)), prepared(Fraction(12, 25)), prepared(Fraction(12, 25))
    store.register_preparation("content-key", old, object())
    controller.get_conveyor_preparation("rev", "domain0", frozenset({2}), request, lambda: new)
    production._register_content(run, entry, request, result)
    assert store.find("content-key").prepared is new  # запись лежала на карте другой досягаемости: заменена
    controller.get_conveyor_preparation("rev", "domain0", frozenset({2}), request, lambda: same)  # кэш сессии держит `new`
    production._register_content(run, entry, request, result)
    assert store.find("content-key").prepared is new  # та же досягаемость: запись остаётся


def test_a_full_session_reset_forgets_the_embedding_memo_but_a_revision_change_keeps_it():
    from fractions import Fraction as F

    import cftuv_envelope as kernel
    from cftuv_envelope import _embedding

    vertices = [kernel.SourceVertexId(f"v{index}") for index in range(3)]
    positions = {item: (F(index), F(index * index), F(0)) for index, item in enumerate(vertices)}
    with _embedding.embedding_memo_limit(4):
        _embedding.build_source_snap_embedding_certificate(
            before=positions, after=positions, faces=(), intended_corners=(), snapping_law=kernel.GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1
        )
        controller = session.EnvelopeDebugSessionController()
        controller._invalidate_revision_scoped()
        assert _embedding.embedding_memo_stats()["entries"] == 1
        controller.clear()
        assert _embedding.embedding_memo_stats()["entries"] == 0
