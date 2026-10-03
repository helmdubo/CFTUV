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
from cftuv.envelope_topology_export import EnvelopeTopologyExportV1  # noqa: E402


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
    for name in ("DEVELOPABLE_BAND_SUPPORT_DISCONNECTED", "CHART_REACH_SHORT_OF_CAP", "DEVELOPABLE_BAND_BOUNDARY_UNRESOLVED"):
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
    policy = SimpleNamespace(selected_physical_edge_ids=frozenset({1}), reach_cap=Fraction(1, 2))
    band = chart_band.chart_band_request(policy, edge_ids, chains, uses, domain)
    assert band.selected_chain_use_ids == frozenset({kernel.ChainUseId("use0")})
    assert len(band.rim_edges) == 2 and len(band.boundary_uses) == 4
    assert band.reach_cap == Fraction(1, 2)
    nothing = SimpleNamespace(selected_physical_edge_ids=frozenset({99}), reach_cap=Fraction(1, 2))
    assert chart_band.chart_band_request(nothing, edge_ids, chains, uses, domain) is None
    assert chart_band.chart_band_request(None, edge_ids, chains, uses, domain) is None
    other_domain = kernel.PatchDomainId("other")
    assert chart_band.chart_band_request(policy, edge_ids, chains, uses, other_domain) is None


def test_an_alpha_beyond_the_reach_cap_of_a_band_chart_is_a_named_host_refusal(monkeypatch):
    class Certificate:
        reach_cap = SimpleNamespace(numerator=1, denominator=2)

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


# --------------------------------------------------------------------------
# Кэш метрики: целый патч от выделения не зависит, полоса зависит
# --------------------------------------------------------------------------


def _record(patch_id, edges):
    return SimpleNamespace(patch_id=patch_id, canonical_edge_ids=tuple(edges))


def _export(selected=None):
    export = EnvelopeTopologyExportV1(
        "rev", None, (_record(0, (1, 2, 3)), _record(1, (7, 8))), {0: "domain0", 1: "domain1"}
    )
    return export if selected is None else export.with_chart_band(None, selected)


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
