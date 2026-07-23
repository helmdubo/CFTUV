"""Stage-specific admission checks for planar geometry evaluation."""

from __future__ import annotations

from ..contracts.analysis import (
    AnalysisSnapshotV1,
    PlanarPatchFrameV1,
    SurfaceRegime,
)
from ..contracts.surface import SurfacePayloadMode
from ..ids import PatchDomainId
from .contracts import (
    ReferenceDiagnosticSeverity,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)


def validate_reference_geometry_payload(
    snapshot: AnalysisSnapshotV1, patch_domain_id: PatchDomainId
) -> tuple[PlanarPatchFrameV1 | None, tuple[ReferenceEvaluationDiagnosticV1, ...]]:
    def issue(outcome: ReferenceOutcome, message: str):
        return (
            None,
            (
                ReferenceEvaluationDiagnosticV1(
                    outcome=outcome,
                    severity=ReferenceDiagnosticSeverity.UNSUPPORTED,
                    message=message,
                ),
            ),
        )

    if snapshot.surface_ir.payload_mode is not SurfacePayloadMode.FULL_HOST_SURFACE:
        return issue(
            ReferenceOutcome.REFERENCE_GEOMETRY_PAYLOAD_REQUIRED,
            "coordinate-free EC0 fixtures cannot be evaluated as geometry",
        )
    domain = next(
        (item for item in snapshot.patch_domains if item.patch_domain_id == patch_domain_id),
        None,
    )
    if domain is None or domain.surface_regime is not SurfaceRegime.PLANAR:
        return issue(
            ReferenceOutcome.REFERENCE_PLANAR_REGIME_REQUIRED,
            "reference evaluator supports SurfaceRegime.PLANAR only",
        )
    frames = tuple(
        item
        for item in snapshot.surface_metric_descriptors
        if isinstance(item, PlanarPatchFrameV1)
        and item.patch_domain_id == patch_domain_id
    )
    if not frames:
        return issue(
            ReferenceOutcome.REFERENCE_PLANAR_FRAME_REQUIRED,
            "PlanarPatchFrameV1 is required",
        )
    if len(frames) != 1:
        return issue(
            ReferenceOutcome.REFERENCE_PLANAR_FRAME_AMBIGUOUS,
            "exactly one PlanarPatchFrameV1 must own the domain",
        )
    frame = frames[0]
    if not frame.planarity_certificate.exact:
        return issue(
            ReferenceOutcome.REFERENCE_PLANAR_FRAME_REQUIRED,
            "v1 reference evaluation requires an exact planarity certificate",
        )
    return frame, ()
