import pytest
from mathutils import Vector

from cftuv.decal_voronoi import (
    DecalSurfaceDomain,
    _insert_intrinsic_triangle_stations,
)


def test_c5_arrangement_edges_split_at_intrinsic_triangle_boundaries():
    domain = DecalSurfaceDomain(
        patch_id=77,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(
            ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),
            ((1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
        ),
    )
    polygon = (
        (0.2, 0.4),
        (0.8, 0.4),
        (0.8, 0.6),
        (0.2, 0.6),
    )

    rebuilt, inserted = _insert_intrinsic_triangle_stations(
        (polygon,), domain, 1e-8
    )

    assert inserted == 2
    assert any(point == pytest.approx((0.6, 0.4)) for point in rebuilt[0])
    assert any(point == pytest.approx((0.4, 0.6)) for point in rebuilt[0])


def test_c5_collinear_boundary_adds_only_missing_source_vertices():
    domain = DecalSurfaceDomain(
        patch_id=78,
        kind="INTRINSIC",
        origin=Vector((0.0, 0.0, 0.0)),
        reference_normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
        boundary_triangles=(
            ((0.0, 0.0), (0.5, 0.0), (0.0, 1.0)),
            ((0.5, 0.0), (1.0, 0.0), (0.0, 1.0)),
        ),
    )

    rebuilt, inserted = _insert_intrinsic_triangle_stations(
        (((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)),),
        domain,
        1e-8,
    )

    assert inserted == 1
    assert any(point == pytest.approx((0.5, 0.0)) for point in rebuilt[0])
