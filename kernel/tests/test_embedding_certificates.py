"""P0-4: embedding is certified separately from the near-planar budget."""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction
import os
from pathlib import Path
import subprocess
import sys

import cftuv_envelope
import pytest
from cftuv_envelope.contracts.metric import (
    ExactPoint2V1,
    ExactRationalV1,
    ExactVector3V1,
    GridSnappingLawV1,
    PlanarityAdmissionLawV1,
    ProjectionAnchorSelectionLawV1,
    PRODUCT_SKIRT_ABSOLUTE_BUDGET,
    NearPlanarProjectionEmbeddingCertificateV1,
    SourceSnapEmbeddingCertificateV1,
)
from cftuv_envelope.contracts.surface import SourceFaceV1
from cftuv_envelope.ids import (
    LineageId,
    PatchDomainId,
    PatchId,
    PlanarityCertificateId,
    PhysicalEdgeId,
    ReferenceMetricId,
    SourceFaceId,
    SourceVertexId,
    SourceRevision,
)
from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1
from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.codec import (
    AnalysisSnapshotCodecV1,
    ContractCodecError,
    EmbeddingCertifiedRationalAffinePlanarMetricCodecV1,
    canonical_json_bytes,
)
from cftuv_envelope.planar_metric import (
    PlanarMetricAdmissionError,
    build_embedding_certified_rational_affine_planar_metric,
    build_rational_affine_planar_metric,
)
from cftuv_envelope.validation_metric import (
    validate_embedding_certified_rational_affine_planar_metric,
    validate_rational_affine_planar_metric,
)
from cftuv_envelope._metric_wire import (
    positive_scaled_normal_compatibility_policy,
)
from cftuv_envelope._embedding import (
    _canonical_cycle,
    _fan_rotation,
    build_projection_embedding_certificate,
    build_source_snap_embedding_certificate,
    projection_violations,
    source_snap_violations,
)


def test_the_two_embedding_certificate_types_are_additive_public_contracts():
    assert cftuv_envelope.SourceSnapEmbeddingCertificateV1 is (
        SourceSnapEmbeddingCertificateV1
    )
    assert cftuv_envelope.NearPlanarProjectionEmbeddingCertificateV1 is (
        NearPlanarProjectionEmbeddingCertificateV1
    )


def test_embedding_does_not_take_authority_over_the_product_skirt_budget():
    assert PRODUCT_SKIRT_ABSOLUTE_BUDGET == Fraction(1, 80)


def _point(x, y, z=0):
    return Fraction(x), Fraction(y), Fraction(z)


def _rational(value):
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def _face(name, vertices):
    return SourceFaceV1(
        face_id=SourceFaceId(name),
        patch_id=PatchId("patch"),
        vertex_cycle=tuple(vertices),
        edge_cycle=tuple(
            PhysicalEdgeId(f"{name}:e{index}") for index in range(len(vertices))
        ),
        polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
        triangle_ids=(),
    )


def _face_with_edges(name, vertices, edge_names):
    return replace(
        _face(name, vertices),
        edge_cycle=tuple(PhysicalEdgeId(item) for item in edge_names),
    )


def _ids(prefix, count):
    return tuple(SourceVertexId(f"{prefix}{index}") for index in range(count))


def _metric_inputs(points):
    vertices = _ids("m", len(points))
    source_vertices = tuple(
        SourceVertexV1(
            vertex_id=vertex_id,
            position=LocalPoint3V1(float(x), float(y), float(z)),
        )
        for vertex_id, (x, y, z) in zip(vertices, points, strict=True)
    )
    return vertices, source_vertices, (_face("metric-face", vertices),)


def _build_wrapper(points, *, near=False, grid=GridSnappingLawV1.UNSNAPPED_EXACT_V1):
    _, vertices, faces = _metric_inputs(points)
    record = build_embedding_certified_rational_affine_planar_metric(
        source_revision=SourceRevision("revision"),
        patch_domain_id=PatchDomainId("domain"),
        owner_patch_id=PatchId("patch"),
        source_vertices=vertices,
        source_faces=faces,
        planarity_policy=(
            PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1
            if near
            else PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
        ),
        grid_policy=grid,
    )
    return record, vertices, faces


def _validate_wrapper(
    record,
    vertices,
    faces,
    *,
    source_revision=SourceRevision("revision"),
    patch_domain_id=PatchDomainId("domain"),
    source_lineage=frozenset(),
):
    return validate_embedding_certified_rational_affine_planar_metric(
        record,
        source_vertices=vertices,
        source_faces=faces,
        owner_patch_id=PatchId("patch"),
        expected_source_revision=source_revision,
        expected_patch_domain_id=patch_domain_id,
        expected_source_lineage=source_lineage,
    )


def test_source_snap_names_injection_edge_collapse_and_intended_degeneration():
    vertices = _ids("v", 4)
    face = _face("f", vertices)
    before = dict(
        zip(
            vertices,
            (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1)),
            strict=True,
        )
    )
    after = dict(before)
    after[vertices[1]] = after[vertices[0]]
    certificate = build_source_snap_embedding_certificate(
        before=before,
        after=after,
        faces=(face,),
        intended_corners=((vertices[3], vertices[0], vertices[1]),),
        snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    )
    assert {
        NamedOutcome.SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED,
        NamedOutcome.SOURCE_SNAP_NONZERO_EDGE_COLLAPSED,
        NamedOutcome.SOURCE_SNAP_INTENDED_RIGHT_CORNER_DEGENERATED,
    } <= set(source_snap_violations(certificate))


def test_source_snap_names_a_new_nonadjacent_intersection():
    vertices = _ids("x", 6)
    faces = (_face("left", vertices[:3]), _face("right", vertices[3:]))
    before = dict(
        zip(
            vertices,
            (
                _point(-3, 0),
                _point(-2, 0),
                _point(-5, -2),
                _point(2, -3),
                _point(2, -2),
                _point(5, -5),
            ),
            strict=True,
        )
    )
    after = dict(before)
    after[vertices[0]], after[vertices[1]] = _point(-1, 0), _point(1, 0)
    after[vertices[3]], after[vertices[4]] = _point(0, -1), _point(0, 1)
    certificate = build_source_snap_embedding_certificate(
        before=before,
        after=after,
        faces=faces,
        intended_corners=(),
        snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    )
    assert NamedOutcome.SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION in (
        source_snap_violations(certificate)
    )


def test_source_snap_checks_an_internal_physical_edge_not_only_the_boundary():
    vertices = _ids("internal", 4)
    first = _face_with_edges(
        "internal-left",
        (vertices[0], vertices[1], vertices[2]),
        ("outer-01", "shared-12", "outer-20"),
    )
    second = _face_with_edges(
        "internal-right",
        (vertices[2], vertices[1], vertices[3]),
        ("shared-12", "outer-13", "outer-32"),
    )
    before = dict(
        zip(
            vertices,
            (_point(0, 0), _point(1, 0), _point(0, 1), _point(1, 1)),
            strict=True,
        )
    )
    after = dict(before)
    after[vertices[2]] = after[vertices[1]]
    certificate = build_source_snap_embedding_certificate(
        before=before,
        after=after,
        faces=(first, second),
        intended_corners=(),
        snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    )
    assert certificate.source_edge_count == 5
    assert certificate.collapsed_nonzero_source_edge_count == 1
    assert NamedOutcome.SOURCE_SNAP_NONZERO_EDGE_COLLAPSED in (
        source_snap_violations(certificate)
    )


def test_reused_physical_edge_id_with_different_endpoints_is_a_contract_error():
    vertices = _ids("bad-edge", 6)
    faces = (
        _face_with_edges("bad-edge-a", vertices[:3], ("same", "a1", "a2")),
        _face_with_edges("bad-edge-b", vertices[3:], ("same", "b1", "b2")),
    )
    positions = {
        vertex: _point(index, index % 2) for index, vertex in enumerate(vertices)
    }
    with pytest.raises(ValueError, match="inconsistent endpoints"):
        build_source_snap_embedding_certificate(
            before=positions,
            after=positions,
            faces=faces,
            intended_corners=(),
            snapping_law=GridSnappingLawV1.UNSNAPPED_EXACT_V1,
        )


def test_projection_names_injection_collapse_crossing_and_overlap_separately():
    vertices = _ids("p", 6)
    faces = (_face("a", vertices[:3]), _face("b", vertices[3:]))
    before = {
        vertices[0]: _point(-1, 0, 0),
        vertices[1]: _point(1, 0, 0),
        vertices[2]: _point(0, -2, 0),
        vertices[3]: _point(0, -1, 1),
        vertices[4]: _point(0, 1, 1),
        vertices[5]: _point(2, 2, 1),
    }
    projected = {vertex: (point[0], point[1]) for vertex, point in before.items()}
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=projected,
        faces=faces,
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_NEW_NONADJACENT_EDGE_INTERSECTION in (
        projection_violations(certificate)
    )

    collapsed = dict(projected)
    collapsed[vertices[1]] = collapsed[vertices[0]]
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=collapsed,
        faces=faces,
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    failures = set(projection_violations(certificate))
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_INJECTIVITY_VIOLATED in failures
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_NONZERO_BOUNDARY_EDGE_COLLAPSED in failures
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED not in failures

    overlap = dict(projected)
    overlap[vertices[3]], overlap[vertices[4]] = _point(-1, 0)[:2], _point(1, 0)[:2]
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=overlap,
        faces=faces,
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_NEW_COLLINEAR_EDGE_OVERLAP in (
        projection_violations(certificate)
    )


def test_projection_names_component_merge_but_not_a_degree_two_turn_change():
    left = _ids("component-left", 3)
    right = _ids("component-right", 3)
    faces = (_face("component-a", left), _face("component-b", right))
    before = {
        left[0]: _point(-2, -1, 0),
        left[1]: _point(-1, -1, 0),
        left[2]: _point(-1, 0, 0),
        right[0]: _point(1, -1, 1),
        right[1]: _point(2, -1, 1),
        right[2]: _point(1, 0, 1),
    }
    projected = {vertex: point[:2] for vertex, point in before.items()}
    projected[right[0]] = (Fraction(-3, 2), Fraction(-2))
    projected[right[1]] = (Fraction(-3, 2), Fraction(1))
    projected[right[2]] = (Fraction(0), Fraction(0))
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=projected,
        faces=faces,
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    failures = set(projection_violations(certificate))
    assert certificate.source_boundary_component_count == 2
    assert certificate.projected_boundary_component_count == 1
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED in failures

    vertices = _ids("cyclic", 5)
    before = dict(
        zip(
            vertices,
            (
                _point(0, 0),
                _point(4, 0),
                _point(4, 4),
                _point(2, 5),
                _point(0, 4),
            ),
            strict=True,
        )
    )
    projected = {vertex: point[:2] for vertex, point in before.items()}
    projected[vertices[3]] = (Fraction(2), Fraction(2))
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=projected,
        faces=(_face("cyclic", vertices),),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    failures = set(projection_violations(certificate))
    assert certificate.orientation_mismatch_count == 0
    assert certificate.source_boundary_component_count == (
        certificate.projected_boundary_component_count
    )
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_CYCLIC_ORDER_CHANGED not in failures


def test_projection_rejects_a_mirrored_loop_by_the_orientation_axis():
    vertices = _ids("mirror", 4)
    before = dict(
        zip(
            vertices,
            (_point(0, 0), _point(2, 0), _point(2, 1), _point(0, 1)),
            strict=True,
        )
    )
    projected = {
        vertex: (-point[0], point[1]) for vertex, point in before.items()
    }
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=projected,
        faces=(_face("mirror", vertices),),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    failures = set(projection_violations(certificate))
    assert certificate.source_cyclic_order_sha256 == (
        certificate.projected_cyclic_order_sha256
    )
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED in failures


def test_projection_names_a_real_fan_identity_change_with_fixed_boundary():
    outer = _ids("fan-outer", 4)
    center = SourceVertexId("fan-center")
    faces = (
        _face_with_edges("fan-0", (outer[0], outer[1], center), ("b01", "r1", "r0")),
        _face_with_edges("fan-1", (outer[1], outer[2], center), ("b12", "r2", "r1")),
        _face_with_edges("fan-2", (outer[2], outer[3], center), ("b23", "r3", "r2")),
        _face_with_edges("fan-3", (outer[3], outer[0], center), ("b30", "r0", "r3")),
    )
    before = {
        outer[0]: _point(0, 0),
        outer[1]: _point(2, 0),
        outer[2]: _point(2, 2),
        outer[3]: _point(0, 2),
        center: _point(1, 1),
    }
    projected = {vertex: point[:2] for vertex, point in before.items()}
    projected[center] = (Fraction(3), Fraction(1))
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=projected,
        faces=faces,
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    failures = set(projection_violations(certificate))
    assert certificate.source_cyclic_order_sha256 == (
        certificate.projected_cyclic_order_sha256
    )
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED in failures


def test_projection_anchor_law_uses_the_resolved_plane_and_rejects_substitution():
    vertices = _ids("anchor", 4)
    before = {
        vertices[0]: _point(0, 0, 0),
        vertices[1]: _point(1, 0, 0),
        vertices[2]: _point(2, 0, 1),
        vertices[3]: _point(0, 1, 0),
    }
    projected = {
        vertices[0]: (Fraction(0), Fraction(0)),
        vertices[1]: (Fraction(1), Fraction(0)),
        vertices[2]: (Fraction(2), Fraction(0)),
        vertices[3]: (Fraction(0), Fraction(1)),
    }
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=projected,
        faces=(_face("anchor", vertices),),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    assert certificate.anchor_selection_law is (
        ProjectionAnchorSelectionLawV1.
        CANONICAL_SOURCE_VERTEX_BASIS_ON_RESOLVED_PLANE_V1
    )
    resolved_anchor = (
        vertices[0],
        vertices[1],
        vertices[3],
    )
    assert certificate.source_anchor_vertex_ids == resolved_anchor
    assert certificate.projected_anchor_vertex_ids == resolved_anchor
    assert projection_violations(certificate) == ()
    forged = replace(
        certificate,
        projected_anchor_vertex_ids=tuple(reversed(resolved_anchor)),
    )
    assert set(projection_violations(forged)) == {
        NamedOutcome.NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED
    }


def test_projection_anchor_law_rejects_an_unknown_literal_through_the_codec():
    record, _, _ = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1, Fraction(1, 10**9))),
        near=True,
    )
    projection = record.near_planar_projection_embedding_certificate
    forged = replace(
        record,
        near_planar_projection_embedding_certificate=replace(
            projection,
            anchor_selection_law="UNKNOWN_PROJECTION_ANCHOR_LAW",
        ),
    )
    payload = EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(forged)
    with pytest.raises(ContractCodecError):
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(payload)


def test_projection_names_a_resolved_plane_without_a_canonical_basis():
    vertices = _ids("basis-unavailable", 4)
    before = dict(
        zip(
            vertices,
            (_point(0, 0), _point(1, 0), _point(2, 0), _point(3, 0)),
            strict=True,
        )
    )
    certificate = build_projection_embedding_certificate(
        before=before,
        projected={vertex: point[:2] for vertex, point in before.items()},
        faces=(_face("basis-unavailable", vertices),),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    assert certificate.resolved_plane_basis_unavailable_count == 1
    assert (
        NamedOutcome.NEAR_PLANAR_PROJECTION_RESOLVED_PLANE_BASIS_UNAVAILABLE
        in projection_violations(certificate)
    )


def test_projection_names_three_disconnected_components_merging_to_one():
    """Normative form of the old helper bug: three components, not a pinch.

    Separate components require separate vertex occurrences as well as unique
    physical edges.  The old helper's shared vertex IDs plus unique edge IDs
    described a non-manifold pinch, not three honest boundary components.
    """

    faces = []
    before = {}
    projected = {}
    for row in range(3):
        vertices = _ids(f"separate-{row}-", 4)
        faces.append(_face(f"separate-{row}", vertices))
        points = (
            _point(0, row, row),
            _point(1, row, row),
            _point(1, row + 1, row),
            _point(0, row + 1, row),
        )
        before.update(dict(zip(vertices, points, strict=True)))
        projected.update(
            {vertex: point[:2] for vertex, point in zip(vertices, points, strict=True)}
        )
    certificate = build_projection_embedding_certificate(
        before=before,
        projected=projected,
        faces=tuple(faces),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    failures = set(projection_violations(certificate))
    assert certificate.source_boundary_component_count == 3
    assert certificate.projected_boundary_component_count == 1
    assert certificate.coincident_boundary_occurrence_pair_count == 4
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED in failures
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_INJECTIVITY_VIOLATED in failures


def test_projection_refuses_a_branched_boundary_instead_of_forging_loops():
    vertices = _ids("branch", 5)
    faces = (
        _face("branch-a", (vertices[0], vertices[1], vertices[2])),
        _face("branch-b", (vertices[0], vertices[3], vertices[4])),
    )
    positions = {
        vertex: _point(index % 3, index // 3)
        for index, vertex in enumerate(vertices)
    }
    with pytest.raises(ValueError, match="disjoint union of directed cycles"):
        build_projection_embedding_certificate(
            before=positions,
            projected={vertex: point[:2] for vertex, point in positions.items()},
            faces=faces,
            normal=_point(0, 0, 1),
            expected_orientation_sign=1,
        )


def test_projection_checks_the_nonfirst_face_and_outer_hole_nesting():
    outer = _ids("o", 4)
    inner = _ids("h", 4)
    positions = {
        **dict(zip(outer, (_point(0, 0), _point(4, 0), _point(4, 4), _point(0, 4)), strict=True)),
        **dict(zip(inner, (_point(1, 1), _point(3, 1), _point(3, 3), _point(1, 3)), strict=True)),
    }
    projected = {vertex: point[:2] for vertex, point in positions.items()}
    certificate = build_projection_embedding_certificate(
        before=positions,
        projected=projected,
        faces=(_face("outer", outer), _face("inner", inner)),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    assert certificate.orientation_mismatch_count == 1
    assert certificate.nesting_mismatch_count == 0
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED in (
        projection_violations(certificate)
    )

    reversed_inner = tuple(reversed(inner))
    certificate = build_projection_embedding_certificate(
        before=positions,
        projected=projected,
        faces=(_face("outer", outer), _face("inner", reversed_inner)),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    assert certificate.projected_face_orientation_signs == (1, -1)
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED in (
        projection_violations(certificate)
    )


def test_projection_names_a_real_containment_parent_change():
    outer = _ids("contain-outer", 4)
    inner = _ids("contain-inner", 4)
    hole = tuple(reversed(inner))
    positions = {
        **dict(zip(outer, (_point(0, 0), _point(4, 0), _point(4, 4), _point(0, 4)), strict=True)),
        **dict(zip(inner, (_point(1, 1), _point(3, 1), _point(3, 3), _point(1, 3)), strict=True)),
    }
    projected = {vertex: point[:2] for vertex, point in positions.items()}
    for vertex in inner:
        projected[vertex] = (projected[vertex][0] + 10, projected[vertex][1])
    certificate = build_projection_embedding_certificate(
        before=positions,
        projected=projected,
        faces=(_face("contain-outer", outer), _face("contain-inner", hole)),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    assert certificate.source_boundary_loop_nesting_depths != (
        certificate.projected_boundary_loop_nesting_depths
    )
    assert NamedOutcome.NEAR_PLANAR_PROJECTION_OUTER_HOLE_NESTING_CHANGED in (
        projection_violations(certificate)
    )


def test_projection_identity_claims_each_have_a_named_negative_control():
    vertices = _ids("i", 3)
    positions = dict(
        zip(vertices, (_point(0, 0), _point(1, 0), _point(0, 1)), strict=True)
    )
    certificate = build_projection_embedding_certificate(
        before=positions,
        projected={vertex: point[:2] for vertex, point in positions.items()},
        faces=(_face("identity", vertices),),
        normal=_point(0, 0, 1),
        expected_orientation_sign=1,
    )
    controls = (
        (
            replace(certificate, projected_boundary_component_count=0),
            NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED,
        ),
        (
            replace(certificate, projected_cyclic_order_sha256="forged"),
            NamedOutcome.NEAR_PLANAR_PROJECTION_CYCLIC_ORDER_CHANGED,
        ),
        (
            replace(certificate, projected_anchor_vertex_ids=tuple(reversed(vertices))),
            NamedOutcome.NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED,
        ),
        (
            replace(certificate, projected_fan_identity_sha256="forged"),
            NamedOutcome.NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED,
        ),
    )
    for forged, expected in controls:
        assert expected in projection_violations(forged)


def test_cyclic_and_fan_rotation_laws_reject_permutation_and_reflection():
    cycle = ("e0", "e1", "e2", "e3")
    assert _canonical_cycle(cycle) == _canonical_cycle(cycle[2:] + cycle[:2])
    assert _canonical_cycle(cycle) != _canonical_cycle(("e0", "e2", "e1", "e3"))
    assert _canonical_cycle(cycle) != _canonical_cycle(tuple(reversed(cycle)))

    source, _, source_ambiguity = _fan_rotation(
        (
            ("e0", (Fraction(1), Fraction(0))),
            ("e1", (Fraction(0), Fraction(1))),
            ("e2", (Fraction(-1), Fraction(0))),
            ("e3", (Fraction(0), Fraction(-1))),
        )
    )
    permuted, _, target_ambiguity = _fan_rotation(
        (
            ("e0", (Fraction(1), Fraction(0))),
            ("e1", (Fraction(-1), Fraction(0))),
            ("e2", (Fraction(0), Fraction(1))),
            ("e3", (Fraction(0), Fraction(-1))),
        )
    )
    assert source_ambiguity == target_ambiguity == 0
    assert source["rotation"] != permuted["rotation"]

    grouped, _, ambiguity = _fan_rotation(
        (
            ("e0", (Fraction(1), Fraction(0))),
            ("e1", (Fraction(2), Fraction(0))),
            ("e2", (Fraction(0), Fraction(1))),
        )
    )
    assert ambiguity == 1
    assert grouped["same_direction_pairs"] == [("e0", "e1")]


def test_additive_wrapper_keeps_v2_bytes_and_requires_projection_only_when_used():
    points = (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1))
    record, vertices, faces = _build_wrapper(points)
    assert record.near_planar_projection_embedding_certificate is None
    old_entry = build_rational_affine_planar_metric(
        source_revision=SourceRevision("revision"),
        patch_domain_id=PatchDomainId("domain"),
        owner_patch_id=PatchId("patch"),
        source_vertices=vertices,
        source_faces=faces,
    )
    assert old_entry == record.metric
    encoded = EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(record)
    assert EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(encoded) == record

    near_record, near_vertices, near_faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1, Fraction(1, 10**9))),
        near=True,
    )
    assert near_record.near_planar_projection_embedding_certificate is not None
    assert _validate_wrapper(near_record, near_vertices, near_faces) == ()


@pytest.mark.parametrize(
    "grid_law",
    (
        GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        GridSnappingLawV1.UNSNAPPED_EXACT_V1,
    ),
)
def test_bent_slope_preserves_rotation_system_and_exact_3d_source_anchors(grid_law):
    slope = -0.4525 / 0.8918
    rows = (0.0, 0.9137, 1.8271, 2.7)
    vertex_ids = _ids("slope-v", 8)
    vertices = []
    for column, x in enumerate((0.0, 3.3125)):
        for row, y in enumerate(rows):
            index = column * len(rows) + row
            height = slope * y + (0.01 if column and row else 0.0)
            vertices.append(
                SourceVertexV1(
                    vertex_id=vertex_ids[index],
                    position=LocalPoint3V1(x, y, height),
                )
            )
    faces = tuple(
        _face_with_edges(
            f"slope-face-{row}",
            (
                vertex_ids[row],
                vertex_ids[row + 1],
                vertex_ids[len(rows) + row + 1],
                vertex_ids[len(rows) + row],
            ),
            (
                f"slope-left-{row}",
                f"slope-cross-{row + 1}",
                f"slope-right-{row}",
                f"slope-cross-{row}",
            ),
        )
        for row in range(len(rows) - 1)
    )
    record = build_embedding_certified_rational_affine_planar_metric(
        source_revision=SourceRevision("slope-revision"),
        patch_domain_id=PatchDomainId("slope-domain"),
        owner_patch_id=PatchId("patch"),
        source_vertices=tuple(vertices),
        source_faces=faces,
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        grid_policy=grid_law,
    )
    projection = record.near_planar_projection_embedding_certificate
    assert projection_violations(projection) == ()
    assert projection.source_anchor_vertex_ids == (
        vertex_ids[0],
        vertex_ids[1],
        vertex_ids[2],
    )
    assert projection.projected_anchor_vertex_ids == (
        vertex_ids[0],
        vertex_ids[1],
        vertex_ids[2],
    )


def test_validator_recomputes_instead_of_trusting_a_zero_cost_claim():
    record, vertices, faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1, Fraction(1, 10**9))),
        near=True,
    )
    projection = record.near_planar_projection_embedding_certificate
    forged = replace(
        record,
        near_planar_projection_embedding_certificate=replace(
            projection,
            exact_pair_test_count=projection.exact_pair_test_count + 1,
        ),
    )
    issues = _validate_wrapper(forged, vertices, faces)
    assert any("differs from exact recomputation" in item.message for item in issues)


def test_validator_recomputes_the_source_grid_certificate():
    record, vertices, faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1)),
    )
    grid = record.metric.grid_certificate
    forged = replace(
        record,
        metric=replace(
            record.metric,
            grid_certificate=replace(
                grid,
                patch_extent=_rational(
                    Fraction(grid.patch_extent.numerator, grid.patch_extent.denominator)
                    + 1
                ),
            ),
        ),
    )
    issues = _validate_wrapper(forged, vertices, faces)
    assert any("grid certificate differs" in item.message for item in issues)


@pytest.mark.parametrize("near", (False, True))
def test_validator_rejects_a_zero_plane_normal_after_codec_roundtrip(near):
    points = (
        _point(0, 0),
        _point(1, 0),
        _point(1, 1),
        _point(0, 1, Fraction(1, 10**9) if near else 0),
    )
    record, vertices, faces = _build_wrapper(points, near=near)
    zero = ExactRationalV1(0, 1)
    forged = replace(
        record,
        metric=replace(
            record.metric,
            planarity_certificate=replace(
                record.metric.planarity_certificate,
                exact_plane_normal=ExactVector3V1(zero, zero, zero),
            ),
        ),
    )
    with pytest.raises(ContractCodecError, match="ZERO_NORMAL_FORBIDDEN"):
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
            canonical_json_bytes(forged)
        )
    issues = _validate_wrapper(forged, vertices, faces)
    assert any("plane normal must be non-zero" in item.message for item in issues)
    assert any("normal differs from exact source" in item.message for item in issues)


@pytest.mark.parametrize("near", (False, True))
def test_positive_scaled_normal_is_read_only_compatibility(near):
    record, vertices, faces = _build_wrapper(
        (
            _point(0, 0),
            _point(1, 0),
            _point(1, 1),
            _point(0, 1, Fraction(1, 10**9) if near else 0),
        ),
        near=near,
    )
    normal = record.metric.planarity_certificate.exact_plane_normal
    scale = Fraction(7, 3)
    scaled = ExactVector3V1(
        _rational(scale * Fraction(normal.x.numerator, normal.x.denominator)),
        _rational(scale * Fraction(normal.y.numerator, normal.y.denominator)),
        _rational(scale * Fraction(normal.z.numerator, normal.z.denominator)),
    )
    forged = replace(
        record,
        metric=replace(
            record.metric,
            planarity_certificate=replace(
                record.metric.planarity_certificate,
                exact_plane_normal=scaled,
            ),
        ),
    )
    direct_issues = (
        *validate_rational_affine_planar_metric(forged.metric),
        *_validate_wrapper(forged, vertices, faces),
    )
    assert any(
        "POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED" in item.message
        for item in direct_issues
    )
    with pytest.raises(
        ContractCodecError,
        match="POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED",
    ):
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
            canonical_json_bytes(forged)
        )
    loaded = (
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.
        loads_with_compatibility_receipt(canonical_json_bytes(forged))
    )
    assert loaded.compatibility_receipt.positive_scaled_normal_compat_hits == 1
    assert loaded.record.metric.planarity_certificate.exact_plane_normal == normal
    assert _validate_wrapper(loaded.record, vertices, faces) == ()
    with pytest.raises(
        ContractCodecError,
        match="POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED",
    ):
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(forged)
    rewritten = EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads_with_compatibility_receipt(
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(loaded.record)
    )
    assert rewritten.compatibility_receipt.positive_scaled_normal_compat_hits == 0
    assert rewritten.record.metric.planarity_certificate.exact_plane_normal == normal


@pytest.mark.parametrize("near", (False, True))
@pytest.mark.parametrize("kind", ("negative", "nonparallel"))
def test_normal_wire_policy_rejects_negative_and_nonparallel(kind, near):
    record, vertices, faces = _build_wrapper(
        (
            _point(0, 0),
            _point(1, 0),
            _point(1, 1),
            _point(0, 1, Fraction(1, 10**9) if near else 0),
        ),
        near=near,
    )
    normal = record.metric.planarity_certificate.exact_plane_normal
    values = tuple(
        Fraction(item.numerator, item.denominator)
        for item in (normal.x, normal.y, normal.z)
    )
    changed = (
        tuple(-item for item in values)
        if kind == "negative"
        else (values[0] + 1, values[1], values[2])
    )
    forged = replace(
        record,
        metric=replace(
            record.metric,
            planarity_certificate=replace(
                record.metric.planarity_certificate,
                exact_plane_normal=ExactVector3V1(
                    *(_rational(item) for item in changed)
                ),
            ),
        ),
    )
    expected = "NEGATIVE_NORMAL_SCALE" if kind == "negative" else "NONPARALLEL_NORMAL"
    with pytest.raises(ContractCodecError, match=expected):
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
            canonical_json_bytes(forged)
        )
    issues = (
        *validate_rational_affine_planar_metric(forged.metric),
        *_validate_wrapper(forged, vertices, faces),
    )
    assert any(expected in item.message for item in issues)


@pytest.mark.parametrize("near", (False, True))
def test_normal_wire_policy_names_a_degenerate_a_cross_b(near):
    record, vertices, faces = _build_wrapper(
        (
            _point(0, 0),
            _point(1, 0),
            _point(1, 1),
            _point(0, 1, Fraction(1, 10**9) if near else 0),
        ),
        near=near,
    )
    forged = replace(record, metric=replace(record.metric, exact_basis_b=record.metric.exact_basis_a))
    with pytest.raises(ContractCodecError, match="DEGENERATE_AFFINE_BASIS"):
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
            canonical_json_bytes(forged)
        )
    issues = (
        *validate_rational_affine_planar_metric(forged.metric),
        *_validate_wrapper(forged, vertices, faces),
    )
    assert any("DEGENERATE_AFFINE_BASIS" in item.message for item in issues)


def test_positive_scaled_fixture_ratchet_and_downstream_primitive_form():
    fixture = (
        Path(__file__).parents[1]
        / "fixtures/building_patch10_density4_v1/analysis_snapshot.json"
    )
    loaded = AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        fixture.read_bytes()
    )
    remaining = (
        positive_scaled_normal_compatibility_policy().
        positive_scaled_fixtures_remaining
    )
    assert remaining <= 1
    assert loaded.compatibility_receipt.positive_scaled_normal_compat_hits == remaining
    second = AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        AnalysisSnapshotCodecV1.dumps(loaded.record)
    )
    assert second.compatibility_receipt.positive_scaled_normal_compat_hits == 0


def test_positive_scaled_fixture_ratchet_rejects_n_plus_one_atomically():
    fixture = (
        Path(__file__).parents[1]
        / "fixtures/building_patch10_density4_v1/analysis_snapshot.json"
    )
    snapshot = AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        fixture.read_bytes()
    ).record
    metric = next(iter(snapshot.surface_metric_descriptors))
    normal = metric.planarity_certificate.exact_plane_normal

    def scaled(item, suffix):
        declared = ExactVector3V1(
            *(
                _rational(2 * Fraction(value.numerator, value.denominator))
                for value in (normal.x, normal.y, normal.z)
            )
        )
        domain = PatchDomainId(f"compat-domain-{suffix}")
        certificate = replace(
            item.planarity_certificate,
            certificate_id=PlanarityCertificateId(f"compat-certificate-{suffix}"),
            patch_domain_id=domain,
            exact_plane_normal=declared,
        )
        return replace(
            item,
            reference_metric_id=ReferenceMetricId(f"compat-metric-{suffix}"),
            patch_domain_id=domain,
            planarity_certificate=certificate,
        )

    over_budget = replace(
        snapshot,
        surface_metric_descriptors=frozenset(
            (scaled(metric, "a"), scaled(metric, "b"))
        ),
    )
    payload = canonical_json_bytes(over_budget)
    for load in (
        AnalysisSnapshotCodecV1.loads,
        AnalysisSnapshotCodecV1.loads_with_compatibility_receipt,
    ):
        with pytest.raises(
            ContractCodecError,
            match="POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED",
        ):
            load(payload)
    with pytest.raises(
        ContractCodecError,
        match="POSITIVE_SCALED_COMPATIBILITY_EXHAUSTED",
    ):
        AnalysisSnapshotCodecV1.dumps(over_budget)


def test_validator_recomputes_all_near_planar_numeric_facts_after_codec_roundtrip():
    record, vertices, faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1, Fraction(1, 10**9))),
        near=True,
    )
    certificate = record.metric.planarity_certificate
    assert certificate.max_residual_squared.numerator != 0
    zero = ExactRationalV1(0, 1)
    forged = replace(
        record,
        metric=replace(
            record.metric,
            planarity_certificate=replace(
                certificate,
                max_residual_squared=zero,
                max_coordinate_ulp=zero,
                planar_extent=zero,
            ),
        ),
    )
    decoded = EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(forged)
    )
    messages = {
        item.message for item in _validate_wrapper(decoded, vertices, faces)
    }
    assert "max_residual_squared differs from exact source recomputation" in messages
    assert "max_coordinate_ulp differs from exact source recomputation" in messages
    assert "planar_extent differs from exact source recomputation" in messages


def test_validator_recomputes_metric_and_certificate_ids_after_codec_roundtrip():
    record, vertices, faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1)),
    )
    forged = replace(
        record,
        metric=replace(
            record.metric,
            reference_metric_id=ReferenceMetricId("arbitrary-metric"),
            planarity_certificate=replace(
                record.metric.planarity_certificate,
                certificate_id=PlanarityCertificateId("arbitrary-certificate"),
            ),
        ),
    )
    decoded = EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(forged)
    )
    messages = {
        item.message for item in _validate_wrapper(decoded, vertices, faces)
    }
    assert "reference metric ID differs from deterministic source recomputation" in messages
    assert "planarity certificate ID differs from deterministic recomputation" in messages


def test_validator_binds_coordinated_identity_claims_to_caller_authority():
    points = (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1))
    _, vertices, faces = _metric_inputs(points)
    forged_lineage = frozenset((LineageId("forged-lineage"),))
    forged = build_embedding_certified_rational_affine_planar_metric(
        source_revision=SourceRevision("forged-revision"),
        patch_domain_id=PatchDomainId("forged-domain"),
        owner_patch_id=PatchId("patch"),
        source_vertices=vertices,
        source_faces=faces,
        source_lineage=forged_lineage,
    )
    decoded = EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.loads(
        EmbeddingCertifiedRationalAffinePlanarMetricCodecV1.dumps(forged)
    )
    messages = {
        item.message for item in _validate_wrapper(decoded, vertices, faces)
    }
    assert "source_revision differs from caller-owned source identity" in messages
    assert "patch_domain_id differs from caller-owned source identity" in messages
    assert "source_lineage differs from caller-owned source identity" in messages


def test_validator_cannot_omit_the_external_identity_authority():
    record, vertices, faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1)),
    )
    with pytest.raises(TypeError, match="expected_source_revision"):
        validate_embedding_certified_rational_affine_planar_metric(
            record,
            source_vertices=vertices,
            source_faces=faces,
            owner_patch_id=PatchId("patch"),
        )


def test_validator_rejects_a_uniform_domain_coordinate_translation():
    record, vertices, faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1, Fraction(1, 10**9))),
        near=True,
    )
    shifted_coordinates = frozenset(
        replace(
            item,
            domain_coordinate=ExactPoint2V1(
                _rational(Fraction(item.domain_coordinate.x.numerator, item.domain_coordinate.x.denominator) + 7),
                item.domain_coordinate.y,
            ),
        )
        for item in record.metric.exact_source_vertex_coordinates
    )
    forged = replace(
        record,
        metric=replace(
            record.metric,
            exact_source_vertex_coordinates=shifted_coordinates,
        ),
    )
    issues = _validate_wrapper(forged, vertices, faces)
    assert any("domain coordinates differ" in item.message for item in issues)


def test_validator_does_not_return_early_when_projection_claim_is_absent():
    record, vertices, faces = _build_wrapper(
        (_point(0, 0), _point(1, 0), _point(1, 1), _point(0, 1))
    )
    changed = tuple(
        replace(vertex, position=LocalPoint3V1(0.0, 1.0, 0.001))
        if vertex.vertex_id == SourceVertexId("m3")
        else vertex
        for vertex in vertices
    )
    issues = _validate_wrapper(record, changed, faces)
    assert any("certificate presence differs" in item.message for item in issues)


def test_planar_metric_has_a_clean_fresh_subprocess_import():
    source_root = Path(__file__).resolve().parents[1] / "src"
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(source_root)
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            "from cftuv_envelope.planar_metric import "
            "build_rational_affine_planar_metric",
        ],
        cwd=source_root.parent,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr


def test_preexisting_unclassifiable_corner_is_diagnostic_not_snap_failure():
    _, source_vertices, faces = _metric_inputs(
        (_point(0, 0), _point(1, 0), _point(2, 0), _point(0, 1))
    )
    record = build_embedding_certified_rational_affine_planar_metric(
        source_revision=SourceRevision("revision"),
        patch_domain_id=PatchDomainId("domain"),
        owner_patch_id=PatchId("patch"),
        source_vertices=source_vertices,
        source_faces=faces,
    )
    snap = record.source_snap_embedding_certificate
    assert snap.unclassifiable_source_corner_count == 1
    assert snap.unchanged_unclassifiable_source_corner_count == 1
    assert source_snap_violations(snap) == ()


def test_true_intended_corner_degeneration_has_no_injectivity_side_failure():
    vertices = _ids("true-corner", 3)
    a = Fraction(500001, 10**6)
    b = Fraction(499999, 10**6)
    before = {
        vertices[0]: _point(a, b),
        vertices[1]: _point(0, 0),
        vertices[2]: _point(-a, b),
    }
    after = {
        vertices[0]: _point(1, 0),
        vertices[1]: _point(0, 0),
        vertices[2]: _point(-1, 0),
    }
    certificate = build_source_snap_embedding_certificate(
        before=before,
        after=after,
        faces=(_face("true-corner", vertices),),
        intended_corners=((vertices[0], vertices[1], vertices[2]),),
        snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    )
    assert certificate.newly_coincident_vertex_pair_count == 0
    assert certificate.collapsed_nonzero_source_edge_count == 0
    assert certificate.degenerated_intended_right_corner_count == 1
    assert source_snap_violations(certificate) == (
        NamedOutcome.SOURCE_SNAP_INTENDED_RIGHT_CORNER_DEGENERATED,
    )


def test_product_path_checks_a_nonfirst_boundary_loop_orientation():
    first = _ids("first", 4)
    second = _ids("second", 4)
    points = {
        **dict(
            zip(
                first,
                (_point(0, 0), _point(4, 0), _point(4, 4), _point(0, 4)),
                strict=True,
            )
        ),
        **dict(
            zip(
                second,
                (
                    _point(10, 0),
                    _point(10, 1),
                    _point(11, 1, Fraction(1, 10**9)),
                    _point(11, 0),
                ),
                strict=True,
            )
        ),
    }
    vertices = tuple(
        SourceVertexV1(
            vertex_id=vertex_id,
            position=LocalPoint3V1(*(float(item) for item in point)),
        )
        for vertex_id, point in points.items()
    )
    faces = (_face("f0", first), _face("f1", second))
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        build_embedding_certified_rational_affine_planar_metric(
            source_revision=SourceRevision("revision"),
            patch_domain_id=PatchDomainId("domain"),
            owner_patch_id=PatchId("patch"),
            source_vertices=vertices,
            source_faces=faces,
            planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        )
    assert failure.value.outcome is (
        NamedOutcome.NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED
    )
