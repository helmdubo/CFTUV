"""NEAR_PLANAR V2, ступень 3: закон укладки на поверхность судит по ширине.

Под `SOURCE_TRIANGLES_V1` судят ДВА свидетеля, а абсолютная невязка плоскости
(1.25 см) — записанная диагностика:

* искажение ширины (`NearPlanarWidthDistortionCertificateV1`): `min cos² ≥ 2500/2601`;
* вложение проекции (`NearPlanarProjectionEmbeddingCertificate`), прежнее.

Бюджеты РАЗНЫЕ и не смешиваются: гребень в 1 см высотой на 5 см ширины лежит
внутри абсолютного бюджета и отвергается по ширине, а дуга цилиндра 20° с
прогибом 7.6 см лежит за абсолютным бюджетом и принимается. Оба направления —
исполняемые.
"""

from __future__ import annotations

import math
from dataclasses import replace
from fractions import Fraction

import pytest

from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.contracts.metric import (
    ExactRationalV1,
    GridSnappingLawV1,
    NearPlanarLiftLawV1,
    NearPlanarProjectionCertificateV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.contracts.surface import SourceFaceV1, SurfaceTriangleV1
from cftuv_envelope.ids import (
    PatchDomainId,
    PatchId,
    PhysicalEdgeId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from cftuv_envelope.materialize.admit import (
    MaterializationOutcome,
    _lift_refusal,
)
from cftuv_envelope.numeric import LocalPoint3V1, LocalVector3V1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import (
    PlanarMetricAdmissionError,
    build_rational_affine_planar_metric,
)
from cftuv_envelope.validation_metric import validate_rational_affine_planar_metric

from materialize_factories import near_planar_domain

NEAR = PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1
EXACT = PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
UNSNAPPED = GridSnappingLawV1.UNSNAPPED_EXACT_V1
ON_PLANE = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
ON_SURFACE = NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1
REVISION = SourceRevision("surface-law-revision")
DOMAIN = PatchDomainId("surface-law-domain")
PATCH = PatchId("surface-law-patch")
THRESHOLD = Fraction(2500, 2601)


# --------------------------------------------------------------------------
# Входы: полоса из колец (по две точки в кольце), квады между кольцами.
# --------------------------------------------------------------------------


def _strip(rings):
    """Полоса: кольцо — две точки (колонки 0 и 1); квады между соседними кольцами."""

    ids = [
        [SourceVertexId(f"s{ring:02d}{column}") for column in range(2)]
        for ring in range(len(rings))
    ]
    vertices = tuple(
        SourceVertexV1(ids[ring][column], LocalPoint3V1(*(float(a) for a in point)))
        for ring, points in enumerate(rings)
        for column, point in enumerate(points)
    )

    def edge(first, second):
        return PhysicalEdgeId(
            "e:" + ":".join(sorted((first.value, second.value)))
        )

    faces = []
    for ring in range(len(rings) - 1):
        cycle = (ids[ring][0], ids[ring][1], ids[ring + 1][1], ids[ring + 1][0])
        faces.append(
            SourceFaceV1(
                face_id=SourceFaceId(f"quad{ring:02d}"),
                patch_id=PATCH,
                vertex_cycle=cycle,
                edge_cycle=tuple(
                    edge(cycle[index], cycle[(index + 1) % 4]) for index in range(4)
                ),
                polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
                triangle_ids=(),
            )
        )
    triangles = tuple(
        SurfaceTriangleV1(
            triangle_id=SurfaceTriangleId(f"{face.face_id.value}:t{index}"),
            source_face_id=face.face_id,
            vertex_ids=(
                face.vertex_cycle[0],
                face.vertex_cycle[index],
                face.vertex_cycle[index + 1],
            ),
            physical_edge_ids=(None, None, None),
            triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
        )
        for face in faces
        for index in (1, 2)
    )
    return vertices, tuple(faces), triangles


def _arc(total_degrees, step_degrees, radius=5.0, width=1.0):
    """Дуга цилиндра: кольцо = образующая; колонки — концы образующей."""

    count = int(total_degrees / step_degrees) + 1
    rings = []
    for index in range(count):
        theta = math.radians(-total_degrees / 2 + index * step_degrees)
        x, z = radius * math.sin(theta), radius * (1 - math.cos(theta))
        rings.append(((x, 0.0, z), (x, width, z)))
    return rings


# Гребень: 1 см высотой, по 2.5 см в каждую сторону (скат ~21.8°), ширина 5 см.
RIDGE = (
    ((-0.025, 0.0, 0.0), (-0.025, 0.05, 0.0)),
    ((0.0, 0.0, 0.01), (0.0, 0.05, 0.01)),
    ((0.025, 0.0, 0.0), (0.025, 0.05, 0.0)),
)


def _metric(rings, law, *, policy=NEAR, triangles=True):
    vertices, faces, surface = _strip(rings)
    return build_rational_affine_planar_metric(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=vertices,
        source_faces=faces,
        planarity_policy=policy,
        grid_policy=UNSNAPPED,
        surface_triangles=surface if triangles else None,
        near_planar_lift_law=law,
    )


def _refusal(rings, law):
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _metric(rings, law)
    return failure.value


# --------------------------------------------------------------------------
# Цилиндр: малый шаг проходит, 90° отказывает по ширине.
# --------------------------------------------------------------------------


def test_a_cylinder_strip_with_small_steps_passes_the_width_budget():
    metric = _metric(_arc(20, 5), ON_SURFACE)
    certificate = metric.planarity_certificate
    assert type(certificate) is NearPlanarProjectionCertificateV1
    assert certificate.lift_law is ON_SURFACE
    sigma = certificate.width_distortion
    measured = Fraction(sigma.min_cos_squared.numerator, sigma.min_cos_squared.denominator)
    # Худший квад — крайний, наклон 7.5°: cos² = 0.9830.
    assert measured > THRESHOLD
    assert float(measured) == pytest.approx(math.cos(math.radians(7.5)) ** 2, abs=2e-3)
    assert validate_rational_affine_planar_metric(metric) == ()


def test_a_ninety_degree_arc_is_refused_by_width_with_numbers():
    error = _refusal(_arc(90, 15), ON_SURFACE)
    assert error.outcome is NamedOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED
    text = str(error)
    for fragment in ("min_cos_squared=", "threshold=9.611687812e-01", "worst_triangle=quad"):
        assert fragment in text, text
    # Крайний квад наклонён на 37.5°: cos² = 0.629.
    measured = float(text.split("min_cos_squared=")[1].split()[0])
    assert measured == pytest.approx(math.cos(math.radians(37.5)) ** 2, abs=2e-2)


# --------------------------------------------------------------------------
# Бюджеты не смешиваются.
# --------------------------------------------------------------------------


def test_the_three_budgets_do_not_mix():
    """Невязка плоскости, ширина и вложение — три судьи, и ни один не подменяет другого.

    * Гребень 1 см (невязка 1.0 см < 1.25 см): прежний закон ПРИНИМАЕТ его, а
      закон поверхности отвергает по ширине — скат 21.8°, `cos²` = 0.862.
    * Дуга 20° (прогиб 7.6 см > 1.25 см): прежний закон ОТВЕРГАЕТ по невязке,
      закон поверхности принимает — наклон 7.5°, `cos²` = 0.983.
    * Домен, принятый по ширине, но с развёрнутой гранью поверх первой
      (`cos²` = 1 у обеих, ориентация обратная), отвергает вложение.
    """

    ridge_on_plane = _metric(RIDGE, ON_PLANE)
    assert ridge_on_plane.planarity_certificate.lift_law is ON_PLANE
    ridge_sigma = ridge_on_plane.planarity_certificate.width_distortion
    assert Fraction(ridge_sigma.min_cos_squared.numerator, ridge_sigma.min_cos_squared.denominator) < THRESHOLD
    assert (
        _refusal(RIDGE, ON_SURFACE).outcome
        is NamedOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED
    )

    arc = _arc(20, 5)
    assert (
        _refusal(arc, ON_PLANE).outcome
        is NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    )
    admitted = _metric(arc, ON_SURFACE).planarity_certificate
    residual = Fraction(admitted.max_residual_squared.numerator, admitted.max_residual_squared.denominator)
    budget = Fraction(admitted.residual_budget.numerator, admitted.residual_budget.denominator)
    assert residual > budget * budget  # записана и НЕ судила


def test_the_embedding_certificate_still_judges_under_the_surface_law():
    """σ у обеих граней 1 (плоские), но вторая обходится против первой и накрывает её."""

    ids = [SourceVertexId(f"f{index}") for index in range(6)]
    positions = (
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 0.0),
        (0.0, 2.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 2.0, 0.002),
    )
    vertices = tuple(
        SourceVertexV1(vertex_id, LocalPoint3V1(*point))
        for vertex_id, point in zip(ids, positions, strict=True)
    )

    def face(name, cycle):
        return SourceFaceV1(
            face_id=SourceFaceId(name),
            patch_id=PATCH,
            vertex_cycle=tuple(ids[item] for item in cycle),
            edge_cycle=tuple(
                PhysicalEdgeId(
                    "e:" + ":".join(sorted((ids[cycle[i]].value, ids[cycle[(i + 1) % len(cycle)]].value)))
                )
                for i in range(len(cycle))
            ),
            polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
            triangle_ids=(),
        )

    faces = (face("a", (0, 1, 2, 3)), face("b", (2, 1, 4, 5)))
    triangles = tuple(
        SurfaceTriangleV1(
            triangle_id=SurfaceTriangleId(f"{item.face_id.value}:t{index}"),
            source_face_id=item.face_id,
            vertex_ids=(item.vertex_cycle[0], item.vertex_cycle[index], item.vertex_cycle[index + 1]),
            physical_edge_ids=(None, None, None),
            triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
        )
        for item in faces
        for index in (1, 2)
    )
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        build_rational_affine_planar_metric(
            source_revision=REVISION,
            patch_domain_id=DOMAIN,
            owner_patch_id=PATCH,
            source_vertices=vertices,
            source_faces=faces,
            planarity_policy=NEAR,
            grid_policy=UNSNAPPED,
            surface_triangles=triangles,
            near_planar_lift_law=ON_SURFACE,
        )
    assert failure.value.outcome.value.startswith("NEAR_PLANAR_PROJECTION_"), failure.value.outcome


# --------------------------------------------------------------------------
# Закон записан; невязка не судит; планарные байты прежние.
# --------------------------------------------------------------------------


def test_a_bend_beyond_the_old_budget_is_admitted_and_the_law_is_recorded():
    bend = (
        ((0.0, 0.0, 0.0), (0.0, 2.0, 0.0)),
        ((2.0, 0.0, 0.0), (2.0, 2.0, 0.0)),
        ((4.0, 0.0, 0.0), (4.0, 2.0, 0.05)),
    )
    assert _refusal(bend, ON_PLANE).outcome is NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    certificate = _metric(bend, ON_SURFACE).planarity_certificate
    assert certificate.lift_law is ON_SURFACE
    assert certificate.width_distortion is not None


def test_the_surface_law_needs_the_surface_triangles_by_name():
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _metric(_arc(20, 5), ON_SURFACE, triangles=False)
    assert (
        failure.value.outcome
        is NamedOutcome.NEAR_PLANAR_OWNER_SURFACE_TRIANGLES_UNAVAILABLE
    )


def test_a_degenerate_snapped_triangle_is_refused_closed_under_the_surface_law():
    ids = [SourceVertexId(f"d{index}") for index in range(5)]
    positions = ((0, 0, 0), (1, 0, 0), (2, 0, 0), (2, 1, 0), (0, 1, 0.002))
    vertices = tuple(
        SourceVertexV1(vertex_id, LocalPoint3V1(*(float(a) for a in point)))
        for vertex_id, point in zip(ids, positions, strict=True)
    )
    cycle = tuple(ids)
    face = SourceFaceV1(
        face_id=SourceFaceId("pent"),
        patch_id=PATCH,
        vertex_cycle=cycle,
        edge_cycle=tuple(PhysicalEdgeId(f"pent:e{index}") for index in range(5)),
        polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
        triangle_ids=(),
    )
    triangles = tuple(
        SurfaceTriangleV1(
            triangle_id=SurfaceTriangleId(f"pent:t{index}"),
            source_face_id=face.face_id,
            vertex_ids=(cycle[0], cycle[index], cycle[index + 1]),
            physical_edge_ids=(None, None, None),
            triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
        )
        for index in (1, 2, 3)
    )

    def build(law):
        return build_rational_affine_planar_metric(
            source_revision=REVISION,
            patch_domain_id=DOMAIN,
            owner_patch_id=PATCH,
            source_vertices=vertices,
            source_faces=(face,),
            planarity_policy=NEAR,
            grid_policy=UNSNAPPED,
            surface_triangles=triangles,
            near_planar_lift_law=law,
        )

    # Под плоскостью — запись (счёт в сертификате), под поверхностью — отказ.
    assert build(ON_PLANE).planarity_certificate.width_distortion.degenerate_triangle_count == 1
    with pytest.raises(PlanarMetricAdmissionError) as failure:
        build(ON_SURFACE)
    assert failure.value.outcome is NamedOutcome.NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE


def test_an_exactly_planar_domain_is_byte_identical_under_both_laws():
    flat = tuple(
        ((float(x), 0.0, 0.0), (float(x), 1.0, 0.0)) for x in range(3)
    )
    assert canonical_json_bytes(_metric(flat, ON_PLANE, policy=EXACT)) == (
        canonical_json_bytes(_metric(flat, ON_SURFACE, policy=EXACT))
    )


# --------------------------------------------------------------------------
# Валидатор и допуск материализатора.
# --------------------------------------------------------------------------


def _with(metric, **changes):
    return replace(
        metric,
        planarity_certificate=replace(metric.planarity_certificate, **changes),
    )


def test_the_validator_follows_the_declared_law():
    arc = _metric(_arc(20, 5), ON_SURFACE)
    # Невязка за бюджетом, закон поверхности: замечаний нет.
    assert validate_rational_affine_planar_metric(arc) == ()
    # Тот же сертификат с объявленным старым законом: невязка судит и отвергает.
    forged = _with(arc, lift_law=ON_PLANE)
    assert any(
        "recorded residual exceeds the recorded budget" in item.message
        for item in validate_rational_affine_planar_metric(forged)
    )
    # Закон поверхности без σ или с σ за бюджетом — замечание.
    assert any(
        "must carry the width-distortion record" in item.message
        for item in validate_rational_affine_planar_metric(_with(arc, width_distortion=None))
    )
    ridge = _metric(RIDGE, ON_PLANE)
    admitted_as_surface = _with(ridge, lift_law=ON_SURFACE)
    assert any(
        "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED" in item.message
        for item in validate_rational_affine_planar_metric(admitted_as_surface)
    )


def test_the_materializer_admission_judges_the_law_against_the_certificate():
    ridge = _metric(RIDGE, ON_PLANE).planarity_certificate
    arc = _metric(_arc(20, 5), ON_SURFACE).planarity_certificate

    # Поверхность запрошена, а σ домена за бюджетом: судья свой, до работы.
    refusal = _lift_refusal(ridge, ON_SURFACE)
    assert refusal.outcome is MaterializationOutcome.NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED
    assert "min_cos_squared=" in refusal.detail
    # Поверхность запрошена, σ в бюджете — допуск.
    assert _lift_refusal(arc, ON_SURFACE) is None
    # Плоскость запрошена, а домен принят под поверхностью с невязкой за бюджетом.
    refusal = _lift_refusal(arc, ON_PLANE)
    assert refusal.outcome is MaterializationOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    assert "max_residual_squared=" in refusal.detail
    # Плоскость запрошена, домен принят под плоскостью: допуск прежний.
    assert _lift_refusal(ridge, ON_PLANE) is None
    # Поверхность запрошена, а записи ширины нет вовсе.
    assert (
        _lift_refusal(replace(ridge, width_distortion=None), ON_SURFACE).outcome
        is MaterializationOutcome.SURFACE_LIFT_UNAVAILABLE
    )


def test_a_domain_admitted_for_the_surface_materializes_on_both_laws_when_flat_enough():
    """Малый прогиб: невязка внутри бюджета, поэтому плоскость остаётся законной."""

    from materialize_factories import affine_domain, prepare_and_cover
    from materialize_factories import SKEW_BOTTOM, SKEW_FACE
    from cftuv_envelope.ids import PolicyId
    from cftuv_envelope.materialize.domain import materialize_domain

    snapshot, request = affine_domain(
        faces=(SKEW_FACE,),
        routes=({"name": "source", "points": SKEW_BOTTOM},),
        planarity_policy=NEAR,
        lift={3: 0.002},
        near_planar_lift_law=ON_SURFACE,
    )
    prepared, coverage = prepare_and_cover(snapshot, request)
    certificate = prepared.context.frame.planarity_certificate
    assert certificate.lift_law is ON_SURFACE
    request = replace(prepared.compilation.decal_request, uv_policy_id=PolicyId("UV_DIRECT_STRIP_V1"))
    on_plane = materialize_domain(prepared, coverage, request=request, near_planar_lift_law=ON_PLANE)
    on_surface = materialize_domain(prepared, coverage, request=request, near_planar_lift_law=ON_SURFACE)
    assert on_plane.is_materialized and on_surface.is_materialized
    assert on_plane.content_digest != on_surface.content_digest
    # Число продолжений в записи диагностики — то же, что в счётчике подъёма.
    counted = dict(on_surface.counters)["MATERIALIZE_SURFACE_LIFT_EXTRAPOLATED_POINTS"]
    (note,) = (
        line for line in on_surface.diagnostics if "NEAR_PLANAR_LIFT_ONTO_SOURCE" in line
    )
    assert f"extrapolated_points={counted} " in note
    assert near_planar_domain is not None
