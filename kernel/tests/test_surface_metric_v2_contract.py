"""Контракт `SurfaceMetricDescriptorV2`: власть, две оси и fail-closed двери."""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction

import pytest

from cftuv_envelope.contracts.analysis import SurfaceRegime
from cftuv_envelope.contracts.metric import GridSnappingLawV1
from cftuv_envelope.contracts.surface_adjacency import (
    SurfaceAdjacencyScopeV1,
    surface_adjacency_digest,
)
from cftuv_envelope.contracts.surface_metric_v2 import (
    SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA,
    ConeAngleMeasureLawV1,
    ConeAngleVerdictV1,
    NamedEpsilonV1,
    SurfaceBarrierRoleV1,
    SurfaceBarrierV1,
    SurfaceMetricDescriptorV2,
    SurfaceMetricPrecisionTierV1,
    SurfacePeriodicTopologyV1,
    VertexConeAngleV1,
)
from cftuv_envelope.ids import LawId, PatchDomainId
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from cftuv_envelope.surface_adjacency_compat import read_surface_adjacency
from cftuv_envelope.surface_cone_angle import (
    angle_bounds,
    certified_cone_angle,
    cosine_bounds,
    two_pi_bounds,
)
from cftuv_envelope.surface_metric_build import build_surface_metric_v2

from surface_adjacency_factories import (
    coincident_positions,
    edge,
    three_coincident_components,
    two_triangle_quad,
    vertex,
)


DOMAIN = PatchDomainId("synthetic:domain")


def _positions():
    return {
        vertex(0): (Fraction(0), Fraction(0), Fraction(0)),
        vertex(1): (Fraction(1), Fraction(0), Fraction(0)),
        vertex(2): (Fraction(1), Fraction(1), Fraction(0)),
        vertex(3): (Fraction(0), Fraction(1), Fraction(0)),
    }


def _metric(**overrides):
    surface = overrides.pop("surface", two_triangle_quad())
    positions = overrides.pop("positions", _positions())
    adjacency = read_surface_adjacency(
        surface, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    kwargs = {
        "patch_domain_id": DOMAIN,
        "surface": surface,
        "adjacency": adjacency,
        "source_positions": positions,
        "surface_regime": SurfaceRegime.GENERAL_CURVED,
        "snapping_law": GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
    }
    kwargs.update(overrides)
    return build_surface_metric_v2(**kwargs)


def _rebuild(metric, **overrides):
    fields = {
        "schema_version": metric.schema_version,
        "patch_domain_id": metric.patch_domain_id,
        "source_revision": metric.source_revision,
        "surface_regime": metric.surface_regime,
        "evaluation_law": metric.evaluation_law,
        "precision_tier": metric.precision_tier,
        "named_epsilon": metric.named_epsilon,
        "grid_certificate": metric.grid_certificate,
        "snapped_source_positions": metric.snapped_source_positions,
        "intrinsic": metric.intrinsic,
        "adjacency_ref": metric.adjacency_ref,
        "barriers": metric.barriers,
        "vertex_cone_angles": metric.vertex_cone_angles,
        "periodic_topology": metric.periodic_topology,
    }
    fields.update(overrides)
    return SurfaceMetricDescriptorV2(**fields)


# --------------------------------------------------------------------------
# Положительная сторона
# --------------------------------------------------------------------------


def test_flat_quad_records_rational_lengths_and_an_exact_interior_cone():
    metric = _metric()
    assert metric.schema_version == SURFACE_METRIC_DESCRIPTOR_V2_SCHEMA
    assert metric.periodic_topology is SurfacePeriodicTopologyV1.NON_PERIODIC_V1
    assert not metric.intrinsic.degenerate_triangles

    lengths = {
        item.triangle_id: (item.side_0, item.side_1, item.side_2)
        for item in metric.intrinsic.triangle_squared_lengths
    }
    assert len(lengths) == 2
    for sides in lengths.values():
        # Единичный квадрат: две стороны единичные, диагональ — два.
        assert sorted(
            Fraction(item.numerator, item.denominator) for item in sides
        ) == [Fraction(1), Fraction(1), Fraction(2)]

    verdicts = {
        item.vertex_id: item.verdict for item in metric.vertex_cone_angles
    }
    assert set(verdicts.values()) == {ConeAngleVerdictV1.STRICTLY_LESS}


def test_snapped_positions_are_recorded_explicitly_for_every_vertex():
    """Счёт ROADMAP закрыт: метрика несёт позиции, из которых её выводят."""

    metric = _metric()
    named = {item.source_vertex_id for item in metric.snapped_source_positions}
    assert named == {vertex(index) for index in range(4)}


def test_adjacency_ref_names_the_table_by_digest():
    surface = two_triangle_quad()
    adjacency = read_surface_adjacency(
        surface, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    metric = _metric(surface=surface)
    assert metric.adjacency_ref.adjacency_digest == surface_adjacency_digest(
        adjacency
    )
    assert metric.adjacency_ref.scope is SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH


def test_grid_certificate_is_the_shared_one_not_a_copy():
    from cftuv_envelope.contracts.metric import IntegerGridCertificateV1

    metric = _metric()
    assert isinstance(metric.grid_certificate, IntegerGridCertificateV1)
    assert metric.grid_certificate.snapping_law.snaps_source


# --------------------------------------------------------------------------
# Отрицательные контроли
# --------------------------------------------------------------------------


def test_n7_planar_regime_is_refused():
    """N-7: у PLANAR есть решётко-точная метрика, и подменять её нельзя."""

    metric = _metric()
    with pytest.raises(ValueError, match="PLANAR"):
        _rebuild(metric, surface_regime=SurfaceRegime.PLANAR)


def test_n8_general_curved_is_never_labelled_exact():
    """N-8: general-curved никогда не EXACT."""

    metric = _metric()
    with pytest.raises(ValueError, match="GENERAL_CURVED"):
        _rebuild(
            metric,
            precision_tier=SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL,
            named_epsilon=None,
        )


def test_n8b_the_two_precision_axes_stay_consistent():
    metric = _metric()
    with pytest.raises(ValueError, match="ε"):
        _rebuild(metric, named_epsilon=None)
    with pytest.raises(ValueError, match="exact"):
        _rebuild(
            metric,
            surface_regime=SurfaceRegime.DEVELOPABLE,
            precision_tier=SurfaceMetricPrecisionTierV1.REFERENCE_EXACT_SMALL,
        )


def test_n9_periodic_topology_other_than_non_periodic_fails_closed():
    """N-9: поле заведено с рождения, значение до S1 одно."""

    metric = _metric()
    with pytest.raises(ValueError, match="не поддержан до S1"):
        _rebuild(
            metric,
            periodic_topology=(
                SurfacePeriodicTopologyV1.PERIODIC_UNIVERSAL_COVER_V1
            ),
        )


def test_n11_exact_two_pi_cannot_be_claimed_from_an_enclosure():
    """N-11: 2π иррационально, интервал равенства не доказывает."""

    low, high = two_pi_bounds()
    enclosure = certified_cone_angle([(low, high)])
    with pytest.raises(ValueError, match="EXACT_TWO_PI"):
        VertexConeAngleV1(
            vertex_id=vertex(0),
            enclosure=enclosure,
            verdict=ConeAngleVerdictV1.EXACT_TWO_PI,
            measure_law=ConeAngleMeasureLawV1.CERTIFIED_INTERVAL_ENCLOSURE_V1,
        )


def test_missing_fan_can_only_produce_the_fail_closed_verdict():
    enclosure = CertifiedDecimalIntervalV1(
        lower=Decimal("1"),
        upper=Decimal("2"),
        lower_kind=IntervalEndpointKind.CLOSED,
        upper_kind=IntervalEndpointKind.CLOSED,
        absolute_error_bound=Decimal("1"),
    )
    with pytest.raises(ValueError, match="UNDECIDED_FAIL_CLOSED"):
        VertexConeAngleV1(
            vertex_id=vertex(0),
            enclosure=enclosure,
            verdict=ConeAngleVerdictV1.STRICTLY_LESS,
            measure_law=ConeAngleMeasureLawV1.FAN_UNAVAILABLE_V1,
        )


def test_one_physical_edge_carries_exactly_one_barrier_role():
    metric = _metric()
    doubled = frozenset(
        {
            SurfaceBarrierV1(
                physical_edge_id=edge(0), role=SurfaceBarrierRoleV1.BARRIER
            ),
            SurfaceBarrierV1(
                physical_edge_id=edge(0),
                role=SurfaceBarrierRoleV1.BOUNDARY_CONDITION,
            ),
        }
    )
    with pytest.raises(ValueError, match="одна роль"):
        _rebuild(metric, barriers=doubled)


def test_n10_three_coincident_components_share_geometry_but_no_adjacency():
    """N-10 со стороны метрики: одинаковые длины, ни одной склейки."""

    surface = three_coincident_components()
    metric = _metric(surface=surface, positions=coincident_positions())
    lengths = {
        (item.side_0, item.side_1, item.side_2)
        for item in metric.intrinsic.triangle_squared_lengths
    }
    assert len(metric.intrinsic.triangle_squared_lengths) == 3
    assert len(lengths) == 1

    adjacency = read_surface_adjacency(
        surface, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )
    from cftuv_envelope.contracts.surface_adjacency import TriangleSideRefV1

    assert not [
        side
        for side in adjacency.triangle_sides
        if isinstance(side.opposite, TriangleSideRefV1)
    ]


# --------------------------------------------------------------------------
# Сертифицированная оболочка угла
# --------------------------------------------------------------------------


def test_cosine_bounds_enclose_the_algebraic_value_exactly():
    """Прямой угол 3-4-5: косинус ровно ноль, и оболочка это видит."""

    low, high = cosine_bounds(Fraction(9), Fraction(16), Fraction(25))
    assert low <= 0 <= high


def test_angle_bounds_prove_the_right_angle_without_a_single_float_authority():
    low, high = angle_bounds(Fraction(9), Fraction(16), Fraction(25))
    two_pi_low, _ = two_pi_bounds()
    assert low < two_pi_low / 4 < high
    assert high - low < Fraction(1, 10**12)


def test_four_right_angles_enclose_two_pi_and_the_sum_is_certified():
    angles = [angle_bounds(Fraction(1), Fraction(1), Fraction(2))] * 4
    enclosure = certified_cone_angle(angles)
    two_pi_low, two_pi_high = two_pi_bounds()
    assert Fraction(enclosure.lower) <= two_pi_low
    assert two_pi_high <= Fraction(enclosure.upper)
    assert enclosure.absolute_error_bound == enclosure.upper - enclosure.lower


def test_named_epsilon_is_the_measured_width_not_a_promise():
    metric = _metric()
    assert metric.named_epsilon is not None
    assert metric.named_epsilon.name == LawId(
        "SURFACE_CONE_ANGLE_INTERVAL_ENCLOSURE_V1"
    )
    widest = max(
        item.enclosure.absolute_error_bound for item in metric.vertex_cone_angles
    )
    assert metric.named_epsilon.absolute_bound == widest
    assert isinstance(metric.named_epsilon.absolute_bound, Decimal)


def test_named_epsilon_rejects_a_negative_bound():
    with pytest.raises(ValueError):
        NamedEpsilonV1(name=LawId("x"), absolute_bound=Decimal("-1"))
