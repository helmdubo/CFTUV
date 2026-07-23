"""Stage-specific admission checks for planar geometry evaluation."""

from __future__ import annotations

import sympy as sp

from ..contracts.analysis import (
    AnalysisSnapshotV1,
    CertifiedPlanarSupportDirectionV1,
    CertifiedReflexAngleMeasureV1,
    PatchFrameHandedness,
    PlanarPatchFrameV1,
    SurfaceRegime,
    TurnOrientation,
)
from ..contracts.metric import (
    CertifiedAffineSupportDirectionV2,
    RationalAffinePlanarMetricV2,
)
from ..contracts.surface import SurfacePayloadMode
from ..ids import PatchDomainId
from ..numeric import IntervalEndpointKind, LocalPoint3V1
from ..validation import validate_rational_affine_planar_metric
from .contracts import (
    ReferenceDiagnosticSeverity,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)
from .metric import ExactPlanarMetric
from .planar_types import (
    CertifiedPredicateUndecidable,
    ExactPlanarVector,
    exact_sign,
)


def _scalar(value: float) -> sp.Rational:
    return sp.Rational(str(value))


def _point3(value) -> tuple[sp.Expr, sp.Expr, sp.Expr]:
    return _scalar(value.x), _scalar(value.y), _scalar(value.z)


def _dot3(left, right) -> sp.Expr:
    return sp.factor(sum(a * b for a, b in zip(left, right, strict=True)))


def _cross3(left, right) -> tuple[sp.Expr, sp.Expr, sp.Expr]:
    return (
        sp.factor(left[1] * right[2] - left[2] * right[1]),
        sp.factor(left[2] * right[0] - left[0] * right[2]),
        sp.factor(left[0] * right[1] - left[1] * right[0]),
    )


def _diagnostic(outcome: ReferenceOutcome, message: str):
    return ReferenceEvaluationDiagnosticV1(
        outcome=outcome,
        severity=ReferenceDiagnosticSeverity.UNSUPPORTED,
        message=message,
    )


def _interval_contains_oriented_support_delta(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
    orientation: TurnOrientation,
    interval,
) -> bool:
    incoming_unit = metric.unit_g(incoming)
    outgoing_unit = metric.unit_g(outgoing)
    cross_value = metric.oriented_cross(incoming_unit, outgoing_unit)
    expected_sign = (
        1
        if orientation is TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
        else -1
    )
    if exact_sign(cross_value) != expected_sign:
        return False
    cosine = metric.dot_g(incoming_unit, outgoing_unit)
    lower_cosine = sp.cos(sp.pi * sp.Rational(str(interval.lower)))
    upper_cosine = sp.cos(sp.pi * sp.Rational(str(interval.upper)))
    lower_cmp = exact_sign(cosine - lower_cosine)
    upper_cmp = exact_sign(cosine - upper_cosine)
    lower_ok = (
        lower_cmp <= 0
        if interval.lower_kind is IntervalEndpointKind.CLOSED
        else lower_cmp < 0
    )
    upper_ok = (
        upper_cmp >= 0
        if interval.upper_kind is IntervalEndpointKind.CLOSED
        else upper_cmp > 0
    )
    return lower_ok and upper_ok


def _support_direction(payload) -> ExactPlanarVector | None:
    if isinstance(payload, CertifiedPlanarSupportDirectionV1):
        return ExactPlanarVector.from_values(
            _scalar(payload.direction_in_domain.x),
            _scalar(payload.direction_in_domain.y),
        )
    if isinstance(payload, CertifiedAffineSupportDirectionV2):
        return ExactPlanarVector.from_values(
            sp.Rational(
                payload.direction_in_domain.x.numerator,
                payload.direction_in_domain.x.denominator,
            ),
            sp.Rational(
                payload.direction_in_domain.y.numerator,
                payload.direction_in_domain.y.denominator,
            ),
        )
    return None


def _validate_angle_certificates(
    snapshot: AnalysisSnapshotV1,
    patch_domain_id: PatchDomainId,
    metric: ExactPlanarMetric,
) -> tuple[ReferenceEvaluationDiagnosticV1, ...]:
    sector_by_id = {
        item.owner_sector_id: item
        for item in snapshot.angular_owner_sectors
        if item.patch_domain_id == patch_domain_id
    }
    certificate_by_id = {
        item.certificate_id: item
        for item in snapshot.reflex_angle_certificates
    }
    for relation in sorted(
        (
            item
            for item in snapshot.corner_relations
            if item.owner_sector_id in sector_by_id
        ),
        key=lambda item: item.corner_relation_id.value,
    ):
        sector = sector_by_id[relation.owner_sector_id]
        certificate = certificate_by_id.get(
            relation.reflex_angle_certificate_id
        )
        if certificate is None or not isinstance(
            certificate.measure_payload, CertifiedReflexAngleMeasureV1
        ):
            continue
        incoming = _support_direction(
            sector.incoming_support_ref.direction_payload
        )
        outgoing = _support_direction(
            sector.outgoing_support_ref.direction_payload
        )
        if incoming is None or outgoing is None:
            continue
        try:
            matches = _interval_contains_oriented_support_delta(
                metric,
                incoming,
                outgoing,
                sector.turn_orientation,
                certificate.measure_payload.reflex_excess_over_pi,
            )
        except CertifiedPredicateUndecidable:
            matches = False
        if not matches:
            return (
                _diagnostic(
                    ReferenceOutcome.REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH,
                    "certified reflex excess does not contain the exact oriented "
                    f"support turn for {relation.corner_relation_id}",
                ),
            )
    return ()


def validate_reference_geometry_certificates(
    snapshot: AnalysisSnapshotV1,
    patch_domain_id: PatchDomainId,
) -> tuple[ReferenceEvaluationDiagnosticV1, ...]:
    """Prove that authoritative planar and angle records describe one geometry."""

    if snapshot.surface_ir.payload_mode is not SurfacePayloadMode.FULL_HOST_SURFACE:
        return ()
    domain = next(
        (
            item
            for item in snapshot.patch_domains
            if item.patch_domain_id == patch_domain_id
        ),
        None,
    )
    metrics = tuple(
        item
        for item in snapshot.surface_metric_descriptors
        if isinstance(item, (PlanarPatchFrameV1, RationalAffinePlanarMetricV2))
        and item.patch_domain_id == patch_domain_id
    )
    if domain is None or len(metrics) != 1:
        return ()
    frame = metrics[0]
    if isinstance(frame, RationalAffinePlanarMetricV2):
        issues = validate_rational_affine_planar_metric(frame)
        if issues:
            return (
                _diagnostic(
                    ReferenceOutcome.REFERENCE_PLANAR_METRIC_CERTIFICATE_MISMATCH,
                    "RationalAffinePlanarMetricV2 is invalid: "
                    + "; ".join(issue.message for issue in issues),
                ),
            )
        return _validate_angle_certificates(
            snapshot,
            patch_domain_id,
            ExactPlanarMetric.from_descriptor(frame),
        )
    origin = _point3(frame.origin)
    axis_u = _point3(frame.axis_u)
    axis_v = _point3(frame.axis_v)
    normal = _point3(frame.normal)
    basis_checks = (
        _dot3(axis_u, axis_u) - 1,
        _dot3(axis_v, axis_v) - 1,
        _dot3(normal, normal) - 1,
        _dot3(axis_u, axis_v),
        _dot3(axis_u, normal),
        _dot3(axis_v, normal),
    )
    expected_normal = (
        normal
        if frame.handedness is PatchFrameHandedness.RIGHT_HANDED_U_V_NORMAL
        else tuple(-item for item in normal)
    )
    handedness_checks = tuple(
        actual - expected
        for actual, expected in zip(
            _cross3(axis_u, axis_v), expected_normal, strict=True
        )
    )
    try:
        basis_valid = all(
            exact_sign(item) == 0 for item in (*basis_checks, *handedness_checks)
        )
    except CertifiedPredicateUndecidable:
        basis_valid = False
    if not basis_valid:
        return (
            _diagnostic(
                ReferenceOutcome.REFERENCE_PLANAR_FRAME_CERTIFICATE_MISMATCH,
                "PlanarPatchFrame basis is not orthonormal or contradicts handedness/normal",
            ),
        )

    source_by_id = {item.vertex_id: item for item in snapshot.source_vertices}
    coordinate_by_id = {
        item.source_vertex_id: item.domain_coordinate
        for item in frame.source_vertex_coordinates
    }
    required_vertex_ids = {
        vertex_id
        for face in snapshot.surface_ir.source_faces
        if face.patch_id == domain.owner_patch_id
        for vertex_id in face.vertex_cycle
    }
    for vertex_id in sorted(required_vertex_ids, key=lambda item: item.value):
        source = source_by_id.get(vertex_id)
        coordinate = coordinate_by_id.get(vertex_id)
        if (
            source is None
            or coordinate is None
            or not isinstance(source.position, LocalPoint3V1)
        ):
            return (
                _diagnostic(
                    ReferenceOutcome.REFERENCE_PLANAR_FRAME_CERTIFICATE_MISMATCH,
                    "full-host planar geometry lacks a total 3D/2D coordinate "
                    f"pair for {vertex_id}",
                ),
            )
        position = _point3(source.position)
        relative = tuple(
            item - base for item, base in zip(position, origin, strict=True)
        )
        checks = (
            _dot3(relative, normal),
            _dot3(relative, axis_u) - _scalar(coordinate.x),
            _dot3(relative, axis_v) - _scalar(coordinate.y),
        )
        try:
            consistent = all(exact_sign(item) == 0 for item in checks)
        except CertifiedPredicateUndecidable:
            consistent = False
        if not consistent:
            return (
                _diagnostic(
                    ReferenceOutcome.REFERENCE_PLANAR_FRAME_CERTIFICATE_MISMATCH,
                    "source position, planar frame, and domain coordinate disagree "
                    f"for {vertex_id}",
                ),
            )

    return _validate_angle_certificates(
        snapshot,
        patch_domain_id,
        ExactPlanarMetric.from_descriptor(frame),
    )


def validate_reference_geometry_payload(
    snapshot: AnalysisSnapshotV1, patch_domain_id: PatchDomainId
) -> tuple[
    PlanarPatchFrameV1 | RationalAffinePlanarMetricV2 | None,
    tuple[ReferenceEvaluationDiagnosticV1, ...],
]:
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
        if isinstance(item, (PlanarPatchFrameV1, RationalAffinePlanarMetricV2))
        and item.patch_domain_id == patch_domain_id
    )
    if not frames:
        return issue(
            ReferenceOutcome.REFERENCE_PLANAR_METRIC_REQUIRED,
            "PlanarPatchFrameV1 or RationalAffinePlanarMetricV2 is required",
        )
    if len(frames) != 1:
        return issue(
            ReferenceOutcome.REFERENCE_PLANAR_METRIC_AMBIGUOUS,
            "exactly one planar metric descriptor must own the domain",
        )
    frame = frames[0]
    if (
        isinstance(frame, PlanarPatchFrameV1)
        and not frame.planarity_certificate.exact
    ):
        return issue(
            ReferenceOutcome.REFERENCE_PLANAR_FRAME_REQUIRED,
            "v1 reference evaluation requires an exact planarity certificate",
        )
    certificate_diagnostics = validate_reference_geometry_certificates(
        snapshot, patch_domain_id
    )
    if certificate_diagnostics:
        return None, certificate_diagnostics
    return frame, ()
