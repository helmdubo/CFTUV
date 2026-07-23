"""Exact per-Patch metric and domain-geometry export.

Both records are SourceRevision/PatchDomain scoped and independent of request
alpha.  They may therefore be reused by the explicit debug session.
"""

from __future__ import annotations

from dataclasses import dataclass
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

    @property
    def metric_descriptor(self):
        return next(iter(self.snapshot.metric_descriptors))


@dataclass(frozen=True, slots=True)
class EnvelopeDomainGeometryExportV1:
    source_revision_value: str
    patch_id: int
    patch_domain_id: str
    snapshot: envelope_kernel.AnalysisSnapshotV1


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
        )


__all__ = (
    "EnvelopeDomainGeometryExportV1",
    "EnvelopePatchMetricExportV1",
    "build_envelope_domain_geometry_export",
    "build_envelope_patch_metric_export",
)
