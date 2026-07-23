from __future__ import annotations

from dataclasses import replace
from decimal import Decimal

from cftuv_envelope import (
    CertifiedDecimalIntervalV1,
    ChainUseId,
    IntervalEndpointKind,
    LocalPoint2V1,
    LocalVector3V1,
    OwnerSectorId,
    PatchSectorV1,
)
from cftuv_envelope.reference import (
    REFERENCE_BOUNDARY_CAPABILITIES_V1,
    ReferenceOutcome,
    compile_reference_envelopes,
    evaluate_reference_raw_coverage,
)

from reference_factories import angular_snapshot, straight_snapshot


SQUARE = (((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)),)


def _straight():
    return straight_snapshot(
        faces=SQUARE,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (10.0, 0.0))},
        ),
        alpha="2",
    )


def test_public_compile_rejects_partial_selected_use_resolution():
    snapshot, request = _straight()
    request = replace(
        request,
        selected_chain_use_ids=request.selected_chain_use_ids
        | {ChainUseId("missing-use")},
    )

    result = compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID
    assert result.compilation is None


def test_public_compile_rejects_invalid_snapshot_before_scope_resolution():
    snapshot, request = _straight()
    snapshot = replace(snapshot, physical_chains=frozenset())

    result = compile_reference_envelopes(snapshot, request)

    assert result.outcome is ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID
    assert result.compilation is None


def test_planar_reference_v1_rejects_multisector_chain_use_named():
    snapshot, request = _straight()
    domain = next(iter(snapshot.patch_domains))
    sector = next(iter(domain.sectors))
    second = PatchSectorV1(
        OwnerSectorId("second-sector"),
        sector.chain_use_id,
        True,
        sector.multiplicity_reason,
    )
    snapshot = replace(
        snapshot,
        patch_domains=frozenset(
            {replace(domain, sectors=domain.sectors | {second})}
        ),
    )

    result = compile_reference_envelopes(snapshot, request)

    assert (
        result.outcome
        is ReferenceOutcome.REFERENCE_MULTISECTOR_CHAIN_USE_UNSUPPORTED
    )
    assert result.compilation is None


def test_planar_frame_basis_must_match_its_exact_certificate():
    snapshot, request = _straight()
    frame = next(iter(snapshot.surface_metric_descriptors))
    snapshot = replace(
        snapshot,
        surface_metric_descriptors=frozenset(
            {replace(frame, axis_u=LocalVector3V1(2.0, 0.0, 0.0))}
        ),
    )

    result = compile_reference_envelopes(snapshot, request)

    assert (
        result.outcome
        is ReferenceOutcome.REFERENCE_PLANAR_FRAME_CERTIFICATE_MISMATCH
    )


def test_planar_domain_coordinate_must_equal_authoritative_projection():
    snapshot, request = _straight()
    frame = next(iter(snapshot.surface_metric_descriptors))
    coordinate = next(iter(frame.source_vertex_coordinates))
    changed_coordinate = replace(
        coordinate,
        domain_coordinate=LocalPoint2V1(
            coordinate.domain_coordinate.x + 1,
            coordinate.domain_coordinate.y,
        ),
    )
    snapshot = replace(
        snapshot,
        surface_metric_descriptors=frozenset(
            {
                replace(
                    frame,
                    source_vertex_coordinates=(
                        frame.source_vertex_coordinates - {coordinate}
                    )
                    | {changed_coordinate},
                )
            }
        ),
    )

    result = compile_reference_envelopes(snapshot, request)

    assert (
        result.outcome
        is ReferenceOutcome.REFERENCE_PLANAR_FRAME_CERTIFICATE_MISMATCH
    )


def test_angle_certificate_must_contain_exact_ordered_support_turn():
    snapshot, request = angular_snapshot(0)
    certificate = next(iter(snapshot.reflex_angle_certificates))
    payload = certificate.measure_payload
    incompatible_delta = CertifiedDecimalIntervalV1(
        Decimal("0.80"),
        Decimal("0.90"),
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        Decimal("0.10"),
    )
    incompatible_phi = CertifiedDecimalIntervalV1(
        Decimal("1.80"),
        Decimal("1.90"),
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        Decimal("0.10"),
    )
    snapshot = replace(
        snapshot,
        reflex_angle_certificates=frozenset(
            {
                replace(
                    certificate,
                    measure_payload=replace(
                        payload,
                        phi_over_pi=incompatible_phi,
                        reflex_excess_over_pi=incompatible_delta,
                    ),
                )
            }
        ),
    )

    result = compile_reference_envelopes(snapshot, request)

    assert (
        result.outcome
        is ReferenceOutcome.REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH
    )
    assert result.compilation is None


def test_concave_ngon_owner_side_comes_from_oriented_face_cycle():
    concave = (
        (
            (0.0, 0.0),
            (4.0, 0.0),
            (6.0, -2.0),
            (8.0, 0.0),
            (8.0, 6.0),
            (0.0, 6.0),
        ),
    )
    snapshot, request = straight_snapshot(
        faces=concave,
        source_routes=(
            {"name": "source", "points": ((0.0, 0.0), (4.0, 0.0))},
        ),
        alpha="1",
    )

    compiled = compile_reference_envelopes(snapshot, request)
    evaluated = evaluate_reference_raw_coverage(
        compiled.compilation, Decimal("1")
    )

    assert compiled.outcome is ReferenceOutcome.EXACT
    assert evaluated.outcome is ReferenceOutcome.EXACT


def test_boundary_capability_table_is_variant_explicit():
    assert dict(REFERENCE_BOUNDARY_CAPABILITIES_V1) == {
        "StripEnvelope": "EXACT_COMPONENT_CONTACT_AND_NAMED_CAPACITY",
        "AngularEnvelope": "EXACT_CLIP_IF_NO_CONTACT_ELSE_NAMED_UNPROVEN",
        "CapEnvelope": "DERIVED_PHYSICAL_TERMINAL_WITH_INCIDENT_STRIP_CAPACITY",
        "JunctionEnvelope": "GEOMETRY_LAW_UNPROVEN_FAIL_CLOSED",
    }
