"""Near-planar политика: проекция объявляется, а не случается молча.

Карта N0 требует: ни один солвер не сглаживает и не проецирует по факту
собственного отказа, и каждая неточная политика записывает масштаб, метод,
бюджет невязки и ревизию источника.

Найдено при проверке: `planar_metric.py` требовал `_dot3(p - o, n) != 0`, то
есть побитово точный ноль без допуска. Поэтому ядро отвергало любой патч,
вершины которого не компланарны в точности — а это практически любой
повёрнутый, отмасштабированный или отредактированный меш. `building.002`
проходил только потому, что он осевой с перпендикулярными углами.
"""

from __future__ import annotations

import pytest

from cftuv_envelope.contracts.analysis import SourceVertexV1
from cftuv_envelope.contracts.metric import (
    NearPlanarResidualBudgetLawV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.contracts.surface import SourceFaceV1
from cftuv_envelope.ids import (
    PatchDomainId,
    PatchId,
    PhysicalChainId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
)
from cftuv_envelope.numeric import LocalPoint3V1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.planar_metric import (
    PlanarMetricAdmissionError,
    build_rational_affine_planar_metric,
)


EXACT = PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1
NEAR = PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1


def _unit_quad_metric(off_plane_height: float, policy: PlanarityAdmissionLawV1):
    """Единичный квад, у которого одна вершина приподнята над плоскостью."""

    vertex_ids = [SourceVertexId(f"v{index}") for index in range(4)]
    positions = (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, off_plane_height),
    )
    vertices = [
        SourceVertexV1(vertex_id=vertex_id, position=LocalPoint3V1(*position))
        for vertex_id, position in zip(vertex_ids, positions, strict=True)
    ]
    face = SourceFaceV1(
        face_id=SourceFaceId("f0"),
        patch_id=PatchId("p0"),
        vertex_cycle=tuple(vertex_ids),
        edge_cycle=tuple(PhysicalChainId(f"e{index}") for index in range(4)),
        polygon_normal=LocalPoint3V1(0.0, 0.0, 1.0),
        triangle_ids=(),
    )
    return build_rational_affine_planar_metric(
        source_revision=SourceRevision("rev"),
        patch_domain_id=PatchDomainId("d0"),
        owner_patch_id=PatchId("p0"),
        source_vertices=vertices,
        source_faces=[face],
        planarity_policy=policy,
    )


def test_exactly_planar_source_is_unaffected_by_the_new_policy():
    """Прежнее поведение обязано остаться побитово прежним."""

    certificate = _unit_quad_metric(0.0, EXACT).planarity_certificate
    assert certificate.admission_law is EXACT
    assert certificate.exact is True


def test_near_planar_source_is_still_refused_under_the_exact_policy():
    """Никакого автоматического сглаживания по факту отказа — карта N0."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _unit_quad_metric(1e-12, EXACT)
    assert (
        failure.value.outcome
        is NamedOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED
    )


def test_requested_near_planar_policy_admits_and_records_the_projection():
    certificate = _unit_quad_metric(1e-12, NEAR).planarity_certificate
    assert certificate.admission_law is NEAR
    assert certificate.exact is False
    assert certificate.residual_budget_law is (
        NearPlanarResidualBudgetLawV1.RELATIVE_EXTENT_OR_ULP_V1
    )
    # Записан масштаб, метод, бюджет и ревизия источника.
    assert certificate.source_revision == SourceRevision("rev")
    assert certificate.planar_extent.numerator > 0
    assert certificate.residual_budget.numerator > 0
    assert certificate.coordinate_ulp_multiplier == 64
    # И поимённо — какие вершины были сдвинуты.
    assert certificate.projected_source_vertex_ids == {SourceVertexId("v3")}


def test_source_beyond_the_budget_is_refused_by_name():
    """Это уже не шум представления, а другая геометрия."""

    with pytest.raises(PlanarMetricAdmissionError) as failure:
        _unit_quad_metric(0.05, NEAR)
    assert (
        failure.value.outcome
        is NamedOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED
    )


def test_projected_source_reconstructs_exactly():
    """Проекция рациональная, поэтому точность ниже по конвейеру не теряется.

    Построитель проверяет `origin + u*A + v*B == position` точным равенством;
    если бы проекция вносила приближение, построение упало бы здесь.
    """

    metric = _unit_quad_metric(1e-12, NEAR)
    assert len(metric.exact_source_vertex_coordinates) == 4


@pytest.mark.parametrize("height", [1e-15, 1e-12, 1e-9, 1e-8])
def test_budget_admits_representation_noise(height: float):
    """Шум порядка ULP и мельче обязан приниматься, иначе меши не пройдут."""

    assert _unit_quad_metric(height, NEAR).planarity_certificate.exact is False
