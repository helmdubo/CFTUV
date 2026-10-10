"""Exact per-Patch metric and domain-geometry export.

Both records are SourceRevision/PatchDomain scoped and independent of request
alpha.  They may therefore be reused by the explicit debug session.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from typing import TYPE_CHECKING

from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from .envelope_topology_export import (
    AnalysisBundleIdView,
    EnvelopeTopologyExportV1,
    build_analysis_bundle_id_view,
)

if TYPE_CHECKING:
    import cftuv_envelope as envelope_kernel


@dataclass(frozen=True, slots=True)
class EnvelopePatchMetricExportV1:
    source_revision_value: str
    patch_id: int
    patch_domain_id: str
    analysis_view: AnalysisBundleIdView
    snapshot: envelope_kernel.AnalysisSnapshotV1
    #: Допуск растяжения, под которым записан сертификат развёртки снапшота (ключ кэшей сессии).
    developable_stretch_budget: Fraction | None = None
    #: Ключ ПОЛОСЫ (`None` - метрика целого патча): карта полосы зависит от выделения и досягаемости запроса, поэтому
    #: ключ кэшей сессии для неё шире `(ревизия, домен, допуск)`.
    band_key: tuple | None = None
    #: Закон выбора масштаба решётки, под которым построена метрика (`None` - умолчание ядра): повторная попытка после отказа лотереи
    #: привязки пишет сюда `PLANE_PRESERVING_V1`, и ключи кэшей сессии несут его (`envelope_topology_export.metric_law_key`).
    grid_scale_law: str | None = None

    @property
    def metric_descriptor(self):
        return next(iter(self.snapshot.metric_descriptors))


@dataclass(frozen=True, slots=True)
class EnvelopeDomainGeometryExportV1:
    source_revision_value: str
    patch_id: int
    patch_domain_id: str
    snapshot: envelope_kernel.AnalysisSnapshotV1
    developable_stretch_budget: Fraction | None = None
    band_key: tuple | None = None
    grid_scale_law: str | None = None


def band_key_of(topology_export: EnvelopeTopologyExportV1, patch_id: int) -> tuple | None:
    """Ключ полосы патча: досягаемость и выбранные рёбра ЭТОГО патча, либо `None`, если политики полосы нет.

    Выбранные рёбра других патчей в ключ не входят: они не меняют полосу этого, а кэш метрики патча не должен
    пересобираться из-за чужого выделения. Суженная карта (`tightened_reach_cap`: сессия пересобрала полосу под собственной
    шириной декали) - другая карта: её ключ несёт метку и точную досягаемость, поэтому она не занимает ключ карты запроса.
    """

    policy = topology_export.chart_band
    if policy is None:
        return None
    # Рёбра патча - из индекса экспорта (один проход по цепочкам на экспорт), а не из прохода по всем цепочкам на каждый вызов:
    # ключ зависит ровно от политики полосы и цепочек экспорта, а обе части неизменяемы у одного объекта.
    own = topology_export.patch_edge_ids(patch_id)
    key = (policy.reach_cap, frozenset(policy.selected_physical_edge_ids) & own)
    return key if policy.tightened_reach_cap is None else key + (("tightened", policy.tightened_reach_cap),)


def build_envelope_patch_metric_export(
    topology_export: EnvelopeTopologyExportV1,
    patch_id: int,
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
) -> EnvelopePatchMetricExportV1:
    """Export one exact Patch metric without copying the AnalysisBundle."""

    from .envelope_request_export import build_envelope_analysis_snapshot

    patch_id = int(patch_id)
    domain_id = topology_export.patch_domain_id_by_patch[patch_id]
    analysis_view = build_analysis_bundle_id_view(
        topology_export.analysis_bundle,
        frozenset({patch_id}),
    )
    if profile is None:
        snapshot = build_envelope_analysis_snapshot(
            topology_export.analysis_bundle,
            included_patch_ids=frozenset({patch_id}),
            topology_export=topology_export,
            analysis_view=analysis_view,
        )
    else:
        with profile.measure("PATCH_METRIC_EXPORT", domain_id):
            snapshot = build_envelope_analysis_snapshot(
                topology_export.analysis_bundle,
                included_patch_ids=frozenset({patch_id}),
                profile=profile,
                topology_export=topology_export,
                analysis_view=analysis_view,
            )
    return EnvelopePatchMetricExportV1(
        topology_export.source_revision_value,
        patch_id,
        domain_id,
        analysis_view,
        snapshot,
        topology_export.developable_stretch_budget,
        band_key_of(topology_export, patch_id),
        getattr(topology_export, "grid_scale_law", None),
    )


def build_envelope_domain_geometry_export(
    metric_export: EnvelopePatchMetricExportV1,
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
) -> EnvelopeDomainGeometryExportV1:
    """Publish the cached domain snapshot without rebuilding geometry."""

    if profile is None:
        return EnvelopeDomainGeometryExportV1(
            metric_export.source_revision_value,
            metric_export.patch_id,
            metric_export.patch_domain_id,
            metric_export.snapshot,
            metric_export.developable_stretch_budget,
            metric_export.band_key,
            metric_export.grid_scale_law,
        )
    with profile.measure(
        "DOMAIN_GEOMETRY_EXPORT",
        metric_export.patch_domain_id,
    ):
        return EnvelopeDomainGeometryExportV1(
            metric_export.source_revision_value,
            metric_export.patch_id,
            metric_export.patch_domain_id,
            metric_export.snapshot,
            metric_export.developable_stretch_budget,
            metric_export.band_key,
            metric_export.grid_scale_law,
        )


__all__ = (
    "EnvelopeDomainGeometryExportV1",
    "EnvelopePatchMetricExportV1",
    "band_key_of",
    "build_envelope_domain_geometry_export",
    "build_envelope_patch_metric_export",
)
