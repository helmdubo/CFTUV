"""Отказы вложения: первая строка для консоли (исход, счёт, первая находка, не длиннее 240 знаков) и полная запись после неё.

Полевой случай `cover.008` (патч 18): прежде отказ нёс `repr` сертификата на 2.2 КБ и исход `ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED`.
Здесь тот же отказ воспроизведён на малом меше с теми же номерами и тем же зазором (5.7e-06 м, ячейка 2^-12): строка называет
исход, счёт контактов, пару рёбер и расстояние, а полный сертификат лежит второй строкой без потерь.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope._embedding import build_projection_embedding_certificate, build_source_snap_embedding_certificate
from cftuv_envelope.contracts.metric import GridSnappingLawV1
from cftuv_envelope.contracts.surface import SourceFaceV1
from cftuv_envelope.embedding_refusal import (
    COMPACT_LINE_LIMIT,
    RECORD_MARKER,
    projection_refusal,
    refusal_message,
    short_name,
    source_snap_refusal,
)
from cftuv_envelope.ids import PatchId, PhysicalEdgeId, SourceFaceId, SourceVertexId
from cftuv_envelope.numeric import LocalVector3V1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import PlanarMetricAdmissionError
from cftuv_envelope.source_grid import resolve_source_grid


def _vertex(number: int) -> SourceVertexId:
    return SourceVertexId(f"host-vertex:host-source:e65179df503e61d2e6c232cf2343e75d04748ffb1cc0e4e2c43d731b7407dbb8:cover.008:{number}")


def _face(name: str, numbers) -> SourceFaceV1:
    return SourceFaceV1(
        face_id=SourceFaceId(name),
        patch_id=PatchId("patch"),
        vertex_cycle=tuple(_vertex(number) for number in numbers),
        edge_cycle=tuple(PhysicalEdgeId(f"{name}:e{index}") for index in range(len(numbers))),
        polygon_normal=LocalVector3V1(0.0, 1.0, 0.0),
        triangle_ids=(),
    )


def _patch_with_a_shared_stretch(offset: float):
    """Две грани вдоль общей прямой без общих вершин (T-стык): нижняя 100-103, верхняя 104-107 на `offset` по y."""

    points = {
        100: (0.0, 0.0, -1.0), 101: (0.0, 0.0, 0.0), 102: (10.0, 0.0, 0.0), 103: (10.0, 0.0, -1.0),
        104: (3.0, offset, 0.0), 105: (13.0, offset, 0.0), 106: (13.0, offset, 1.0), 107: (3.0, offset, 1.0),
    }
    positions = {_vertex(number): tuple(Fraction(axis) for axis in point) for number, point in points.items()}
    return positions, (_face("f46", (100, 101, 102, 103)), _face("f54", (104, 105, 106, 107)))


def _refusal_of(positions, faces):
    with pytest.raises(PlanarMetricAdmissionError) as raised:
        resolve_source_grid(positions=positions, faces=faces, snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1)
    return raised.value


def test_the_field_refusal_is_one_readable_line_with_the_full_record_after_it():
    """Зазор в 5.7 мкм на габарите 13 м: привязка к 2^-12 замыкает его в контакт; строка называет рёбра, расстояние и ячейку."""

    error = _refusal_of(*_patch_with_a_shared_stretch(5.7e-06))
    assert error.outcome is NamedOutcome.SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION
    first, record = str(error).split("\n", 1)
    assert first == (
        "SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION: 3 contacts after snap to 2^-12; "
        "first edge 101-102 × 104-105, 5.7e-06 m apart"
    )
    assert len(first) <= COMPACT_LINE_LIMIT
    assert record.startswith(RECORD_MARKER + "SourceSnapEmbeddingCertificateV1(")
    assert "SOURCE_ONLY_GRID_SNAP_V1" in record and "host-vertex:host-source:" in record


def _certificate(before, after, faces, corners=()):
    return build_source_snap_embedding_certificate(
        before=before, after=after, faces=faces, intended_corners=corners, snapping_law=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1
    )


def _square():
    vertices = [_vertex(number) for number in (10, 11, 12, 13)]
    before = dict(zip(vertices, (tuple(map(Fraction, point)) for point in ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)))))
    return vertices, before, (_face("sq", (10, 11, 12, 13)),)


def test_each_snap_outcome_names_its_first_finding_in_the_numbers_of_the_host():
    vertices, before, faces = _square()
    after = dict(before)
    after[vertices[1]] = after[vertices[0]]
    corner = ((vertices[3], vertices[0], vertices[1]),)
    certificate = _certificate(before, after, faces, corner)
    expected = {
        NamedOutcome.SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED: "1 vertex pair merged after snap to 2^-12; first 10 and 11, 1 m apart",
        NamedOutcome.SOURCE_SNAP_NONZERO_EDGE_COLLAPSED: "1 edge collapsed after snap to 2^-12; first edge 10-11, 1 m long",
        NamedOutcome.SOURCE_SNAP_INTENDED_RIGHT_CORNER_DEGENERATED: "1 intended right corner degenerated after snap to 2^-12; first at vertex 10 (13-10-11)",
    }
    for outcome, summary in expected.items():
        message = source_snap_refusal(outcome, certificate, before=before, after=after, faces=faces, corners=corner, scale=4096)
        first, record = message.split("\n", 1)
        assert first == f"{outcome.value}: {summary}", first
        assert record == RECORD_MARKER + repr(certificate)


def test_a_new_contact_between_two_far_apart_triangles_is_found_by_the_certificates_own_predicate():
    left = [_vertex(number) for number in (1, 2, 3)]
    right = [_vertex(number) for number in (4, 5, 6)]
    faces = (_face("left", (1, 2, 3)), _face("right", (4, 5, 6)))
    before = dict(zip(left + right, (tuple(map(Fraction, point)) for point in (
        (-3, 0, 0), (-2, 0, 0), (-5, -2, 0), (2, -3, 0), (2, -2, 0), (5, -5, 0)))))
    after = dict(before)
    after[left[0]], after[left[1]] = (Fraction(-1), Fraction(0), Fraction(0)), (Fraction(1), Fraction(0), Fraction(0))
    after[right[0]], after[right[1]] = (Fraction(0), Fraction(-1), Fraction(0)), (Fraction(0), Fraction(1), Fraction(0))
    certificate = _certificate(before, after, faces)
    message = source_snap_refusal(
        NamedOutcome.SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION, certificate,
        before=before, after=after, faces=faces, corners=(), scale=2,
    )
    first = message.split("\n", 1)[0]
    assert first.startswith(
        f"SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION: {certificate.new_nonadjacent_edge_intersection_count} contact"
    )
    assert "after snap to 2^-1; first edge 1-2 × 4-5, 4.5 m apart" in first, first


def test_the_first_line_never_exceeds_the_limit_and_an_overlong_summary_is_cut_with_an_ellipsis():
    message = refusal_message(NamedOutcome.GRID_WINDOW_CLOSED, "x" * 400, {"record": 1})
    first, record = message.split("\n", 1)
    assert len(first) == COMPACT_LINE_LIMIT and first.endswith("…")
    assert record == RECORD_MARKER + repr({"record": 1})
    assert COMPACT_LINE_LIMIT == 240


def test_short_name_is_the_host_number_after_the_last_colon():
    assert short_name(_vertex(133)) == "133"
    assert short_name("plain") == "plain"


def test_every_projection_outcome_has_a_one_line_summary_and_the_record():
    vertices = [_vertex(number) for number in (1, 2, 3)]
    faces = (_face("tri", (1, 2, 3)),)
    before = dict(zip(vertices, (tuple(map(Fraction, point)) for point in ((0, 0, 0), (2, 0, 0), (0, 2, 0)))))
    projected = {vertex: point[:2] for vertex, point in before.items()}
    certificate = build_projection_embedding_certificate(
        before=before, projected=projected, faces=faces, normal=(Fraction(0), Fraction(0), Fraction(1)), expected_orientation_sign=1
    )
    families = [item for item in NamedOutcome if item.value.startswith("NEAR_PLANAR_PROJECTION_")]
    assert len(families) >= 13
    for outcome in families:
        first, record = projection_refusal(outcome, certificate).split("\n", 1)
        assert first.startswith(f"{outcome.value}: ") and first.endswith("after projection onto the patch plane"), first
        assert len(first) <= COMPACT_LINE_LIMIT
        assert record.startswith(RECORD_MARKER + "NearPlanarProjectionEmbeddingCertificateV1(")
