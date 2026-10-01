"""DEVELOPABLE (S1), срез C2: материализатор кладёт меш развёрнутого домена на треугольники источника.

Домен с сертификатом развёртки проходит тот же путь, что и near-planar на поверхности:
допуск (`admit`) судит растяжение заново, подъём (`surface_lift_of`) берёт позиции из
сертификата, а направление смещения над поверхностью — нормаль ВЕРШИНЫ (закон
`SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`), потому что плоскости источника у развёртки нет.
"""

from __future__ import annotations

import math
from dataclasses import replace
from fractions import Fraction
from types import SimpleNamespace

import pytest

from cftuv_envelope.contracts.metric import DevelopableUnfoldCertificateV1, ExactRationalV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1, exact_work_budget
from cftuv_envelope.materialize.admit import (
    MaterializationOutcome,
    PlanarityKind,
    _developable_refusal,
)
from cftuv_envelope.materialize.lift import plane_lift_of, plane_normal_binary64
from cftuv_envelope.materialize.lift_surface import surface_lift_of
from cftuv_envelope.materialize.offset_normal import (
    OFFSET_NORMAL_LAW,
    blend,
    source_vertex_normals,
)
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.numeric import LocalPoint3V1
from cftuv_envelope.outcomes import NamedOutcome

import developable_factories as factories
from developable_route import build_metric, materialize_developable

CASES = {
    "fold-strip": (factories.fold_strip, ("r0a", "r0b"), "1.5"),
    "bevel": (lambda: factories.bevel_strip(4), ("r0a", "r0b"), "2.5"),
    "quarter-cylinder": (factories.quarter_cylinder, ("r0a", "r0b"), "0.8"),
    "cone-sector": (lambda: factories.cone(8, boundary_apex=True), ("apex", "b0"), "0.5"),
}


@pytest.fixture(scope="module", params=sorted(CASES), ids=lambda name: name)
def materialized(request):
    make, route, alpha = CASES[request.param]
    parts = make()
    result, prepared = materialize_developable(parts, route, alpha=alpha)
    return request.param, parts, result, prepared


def _snapped(certificate):
    return {
        item.source_vertex_id: tuple(
            float(Fraction(axis.numerator, axis.denominator))
            for axis in (item.position.x, item.position.y, item.position.z)
        )
        for item in certificate.snapped_source_positions
    }


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _distance_to_triangle_surface(point, corners) -> float | None:
    """Расстояние от точки до плоскости треугольника, если её проекция внутри (иначе `None`)."""

    a, b, c = corners
    normal = _cross(_sub(b, a), _sub(c, a))
    length = math.sqrt(_dot(normal, normal))
    along = _dot(_sub(point, a), normal) / length
    foot = tuple(p - along * n / length for p, n in zip(point, normal))
    area = _dot(normal, normal)
    weights = (
        _dot(_cross(_sub(c, b), _sub(foot, b)), normal) / area,
        _dot(_cross(_sub(a, c), _sub(foot, c)), normal) / area,
        _dot(_cross(_sub(b, a), _sub(foot, a)), normal) / area,
    )
    if min(weights) < -1e-9:
        return None
    return abs(along)


def _surface_distance(point, triangles, positions) -> float:
    return min(
        distance
        for distance in (
            _distance_to_triangle_surface(
                point, tuple(positions[vertex] for vertex in triangle.vertex_ids)
            )
            for triangle in triangles
        )
        if distance is not None
    )


# --------------------------------------------------------------------------
# Материализация: фикстуры 1-4
# --------------------------------------------------------------------------


def test_the_unfolded_domain_is_materialized_onto_the_source_triangles(materialized):
    name, parts, result, prepared = materialized
    assert result.is_materialized, (name, result.outcome, result.detail)
    certificate = prepared.context.frame.planarity_certificate
    assert type(certificate) is DevelopableUnfoldCertificateV1
    counters = dict(result.counters)
    assert counters["MATERIALIZE_VERTICES"] >= 4
    positions = _snapped(certificate)
    worst = max(
        _surface_distance(
            (vertex.position.x, vertex.position.y, vertex.position.z),
            prepared.context.snapshot.surface_ir.surface_triangles,
            positions,
        )
        for vertex in result.batch.vertices
    )
    # Вершины меша лежат НА треугольниках источника: одно округление точной величины.
    assert worst < 1e-14, (name, worst)


def test_the_chart_lattice_snaps_nothing_and_no_point_is_extrapolated(materialized):
    _name, _parts, result, _prepared = materialized
    counters = dict(result.counters)
    assert counters["MATERIALIZE_SURFACE_LIFT_CHART_VERTICES_SNAPPED"] == 0
    assert counters["MATERIALIZE_SURFACE_LIFT_EXTRAPOLATED_POINTS"] == 0
    assert counters["MATERIALIZE_SURFACE_LIFT_DEGENERATE_PROJECTIONS"] == 0
    assert counters["MATERIALIZE_SURFACE_LIFT_COLLAPSED_BY_SNAPPING"] == 0


def test_the_batch_names_the_unfolded_lift_with_its_numbers(materialized):
    _name, _parts, result, _prepared = materialized
    lines = [item for item in result.diagnostics if item.startswith("DEVELOPABLE_LIFT_ONTO_UNFOLDED")]
    assert len(lines) == 1
    assert "worst_band_squared<=" in lines[0]
    assert f"offset_normal_law={OFFSET_NORMAL_LAW}" in lines[0]
    assert "previous_refusals=['NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED']" in lines[0]
    assert not any(item.startswith("NEAR_PLANAR_LIFT") for item in result.diagnostics)


def test_every_vertex_of_the_batch_carries_a_front_facing_unit_offset_normal(materialized):
    _name, parts, result, prepared = materialized
    assert result.offset_normal_law == OFFSET_NORMAL_LAW
    keys = {item.vert_key.value for item in result.batch.vertices}
    normals = dict(result.vertex_normals)
    assert set(normals) == keys
    for normal in normals.values():
        assert math.sqrt(_dot(normal, normal)) == pytest.approx(1.0, abs=1e-12)
    # Меш обходится так же, как источник: нормаль каждой его грани смотрит вместе с нормалью смещения.
    position = {item.vert_key.value: item.position for item in result.batch.vertices}
    for face in result.batch.faces:
        points = [position[key.value] for key in face.ordered_vert_keys]
        triangle = _cross(
            _sub((points[1].x, points[1].y, points[1].z), (points[0].x, points[0].y, points[0].z)),
            _sub((points[2].x, points[2].y, points[2].z), (points[0].x, points[0].y, points[0].z)),
        )
        mean = tuple(
            sum(normals[key.value][axis] for key in face.ordered_vert_keys) for axis in range(3)
        )
        assert _dot(triangle, mean) > 0


def test_the_materialization_is_reproducible_bit_for_bit():
    make, route, alpha = CASES["quarter-cylinder"]
    first, _ = materialize_developable(make(), route, alpha=alpha)
    second, _ = materialize_developable(make(), route, alpha=alpha)
    assert first.content_digest == second.content_digest
    assert first.vertex_normals == second.vertex_normals


def test_a_decal_crossing_a_fold_lifts_vertices_onto_both_planes():
    result, _prepared = materialize_developable(factories.fold_strip(), ("r0a", "r0b"), alpha="1.5")
    xs = {round(vertex.position.x, 9) for vertex in result.batch.vertices}
    zs = {round(vertex.position.z, 9) for vertex in result.batch.vertices}
    # Декаль перешла со стола на стену: вершины есть и на плоскости z = 0, и на x = 1.
    assert 0.0 in zs and any(z > 0.0 for z in zs)
    assert 1.0 in xs or any(abs(x - 1.0) < 1e-9 for x in xs)


def test_the_unfolded_frame_has_no_plane_to_lift_on_or_to_take_a_normal_from():
    record = build_metric(factories.fold_strip())
    with pytest.raises(ValueError, match="no source plane"):
        plane_lift_of(record.metric, 1)
    with pytest.raises(ValueError, match="no source plane"):
        plane_normal_binary64(record.metric)


# --------------------------------------------------------------------------
# Допуск: сертификат судится заново
# --------------------------------------------------------------------------


def test_admit_judges_the_stretch_certificate_again_and_names_the_refusal():
    certificate = build_metric(factories.bevel_strip(3)).metric.planarity_certificate
    assert _developable_refusal(certificate) is None
    liar = replace(
        certificate,
        stretch=replace(
            certificate.stretch,
            triangles_outside_budget=1,
            first_outside_triangle_id=certificate.stretch.worst_triangle_id,
        ),
    )
    refusal = _developable_refusal(liar)
    assert refusal.outcome is MaterializationOutcome.DEVELOPABLE_STRETCH_BUDGET_EXCEEDED
    assert "developable stretch" in refusal.detail
    flipped = replace(
        certificate,
        stretch=replace(
            certificate.stretch,
            chart_flipped_triangle_count=1,
            first_flipped_triangle_id=certificate.stretch.worst_triangle_id,
        ),
    )
    assert (
        _developable_refusal(flipped).outcome
        is MaterializationOutcome.DEVELOPABLE_CHART_TRIANGLE_FLIPPED
    )
    overlapping = replace(certificate, chart_boundary_overlap_count=2)
    assert (
        _developable_refusal(overlapping).outcome
        is MaterializationOutcome.DEVELOPABLE_CHART_SELF_OVERLAP
    )


def test_the_planarity_kind_of_an_unfolded_domain_is_named(materialized):
    from cftuv_envelope.materialize.admit import admit_domain

    _name, _parts, result, prepared = materialized
    from cftuv_envelope.materialize.admit import materialization_request

    request = materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1")
    admission = admit_domain(
        prepared, SimpleNamespace(outcome=SimpleNamespace(value="EXACT"), preparation=None), request
    )
    assert admission.planarity is PlanarityKind.DEVELOPABLE_UNFOLDED
    assert admission.lift_law.value == "SOURCE_TRIANGLES_V1"


# --------------------------------------------------------------------------
# Нормаль смещения: закон по вершине, а не по плоскости
# --------------------------------------------------------------------------


def _triangles_and_positions(parts):
    vertices, _faces, triangles = parts
    return triangles, {
        item.vertex_id: tuple(Fraction(a) for a in (item.position.x, item.position.y, item.position.z))
        for item in vertices
    }


def test_the_vertex_normal_on_a_ninety_degree_fold_bisects_the_two_planes():
    triangles, positions = _triangles_and_positions(factories.fold_strip())
    normals = source_vertex_normals(triangles, positions)
    on_fold = normals[next(v for v in positions if v.value == "v:r1a")]
    # Вершина складки лежит между полом (нормаль -z по обходу полосы) и стеной (+x): биссектриса.
    expected = (math.sqrt(0.5), 0.0, -math.sqrt(0.5))
    assert on_fold == pytest.approx(expected, abs=1e-12)
    assert normals[next(v for v in positions if v.value == "v:r0a")] == pytest.approx(
        (0.0, 0.0, -1.0), abs=1e-12
    )


def test_a_back_to_back_pair_has_no_offset_normal_and_is_refused_by_name():
    points = {
        "a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0), "c": (0.0, 1.0, 0.0),
    }
    triangles, positions = _triangles_and_positions(
        factories.surface(
            {**points, "d": (0.0, -1.0, 0.0)}, [["a", "b", "c"], ["b", "a", "d"]]
        )
    )
    # Плоский лист даёт нормаль `+z` по обоим треугольникам — это законный случай.
    assert source_vertex_normals(triangles, positions)
    folded_back = {
        key: (value if key != "d" else (0.5, 0.0, 0.0)) for key, value in
        {"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0), "c": (0.0, 1.0, 0.0), "d": (0.0, -1.0, 0.0)}.items()
    }
    del folded_back
    # Складка на 180°: два треугольника лицом к лицу, их нормали противоположны.
    triangles, positions = _triangles_and_positions(
        factories.surface(
            {"a": (0.0, 0.0, 0.0), "b": (1.0, 0.0, 0.0), "c": (0.0, 1.0, 0.0), "d": (0.0, 1.0, 0.0)},
            [["a", "b", "c"], ["b", "a", "d"]],
        )
    )
    with pytest.raises(MaterializationRefusal) as failure:
        source_vertex_normals(triangles, positions)
    assert failure.value.outcome is MaterializationOutcome.SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE


def test_a_blended_normal_is_the_normalized_barycentric_mixture():
    mixture = blend((0.5, 0.5, 0.0), ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
    assert mixture == pytest.approx((math.sqrt(0.5), 0.0, math.sqrt(0.5)), abs=1e-12)
    with pytest.raises(MaterializationRefusal):
        blend((0.5, 0.5, 0.0), ((0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)))


def test_planar_and_near_planar_domains_carry_no_vertex_normals():
    from materialize_factories import near_planar_domain
    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.domain import materialize_domain

    prepared, coverage, _request = near_planar_domain()
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
    )
    assert result.is_materialized
    assert result.vertex_normals == ()
    assert result.offset_normal_law == ""


# --------------------------------------------------------------------------
# Шов: подъём на общем ребре источника у планарного и развёрнутого соседа
# --------------------------------------------------------------------------


def _vault_with_end_wall():
    """Свод (четверть цилиндра, развёрнутый патч B) и торцевая стена (near-planar патч A).

    Общее ребро источника — образующая `r0a-r0b` на торце свода. Стена слегка непланарна
    (вершины выведены из плоскости на сантиметры, больше её ячейки решётки), чтобы быть
    near-planar, а не точной плоскостью. Концы общего ребра лежат на любой степени двойки
    (`(0, 0, 0)`, `(0, 1, 0)`), поэтому привязка у соседей даёт им одни и те же позиции.
    """

    from cftuv_envelope.contracts.analysis import SourceVertexV1
    from cftuv_envelope.ids import PatchId

    patch_a = PatchId("end-wall")
    vault = factories.quarter_cylinder(8)
    vertices, faces, triangles = vault
    wall_points = {
        "w0": (-1.0, 0.0, 0.05),
        "w1": (-1.0, 1.0, 0.09),
    }
    ids = {item.vertex_id.value[2:]: item.vertex_id for item in vertices}
    from cftuv_envelope.ids import SourceVertexId

    for name in wall_points:
        ids[name] = SourceVertexId(f"v:{name}")
    extra_vertices = tuple(
        SourceVertexV1(ids[name], LocalPoint3V1(*point)) for name, point in wall_points.items()
    )
    wall_cycle = (ids["w0"], ids["r0a"], ids["r0b"], ids["w1"])
    from cftuv_envelope.contracts.surface import SourceFaceV1, SurfaceTriangleV1
    from cftuv_envelope.ids import PhysicalEdgeId, SourceFaceId, SurfaceTriangleId
    from cftuv_envelope.numeric import LocalVector3V1

    face = SourceFaceV1(
        face_id=SourceFaceId("wall"),
        patch_id=patch_a,
        vertex_cycle=wall_cycle,
        edge_cycle=tuple(
            PhysicalEdgeId("e:" + ":".join(sorted((wall_cycle[i].value, wall_cycle[(i + 1) % 4].value))))
            for i in range(4)
        ),
        polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
        triangle_ids=(),
    )
    wall_triangles = tuple(
        SurfaceTriangleV1(
            triangle_id=SurfaceTriangleId(f"wall:t{index}"),
            source_face_id=face.face_id,
            vertex_ids=(wall_cycle[0], wall_cycle[index], wall_cycle[index + 1]),
            physical_edge_ids=(None, None, None),
            triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
        )
        for index in (1, 2)
    )
    return (
        (*vertices, *extra_vertices),
        (*faces, face),
        (*triangles, *wall_triangles),
        ids,
        patch_a,
    )


def test_the_lift_on_a_shared_source_edge_is_the_same_from_a_planar_and_a_developable_neighbour():
    """Шов планарного и развёрнутого соседей: общее ребро источника поднимается побитово одинаково."""

    from cftuv_envelope.contracts.metric import (
        GridSnappingLawV1,
        NearPlanarLiftLawV1,
        PlanarityAdmissionLawV1,
    )
    from cftuv_envelope.planar_metric import build_rational_affine_planar_metric

    vertices, faces, triangles, ids, patch_a = _vault_with_end_wall()
    snapshot = SimpleNamespace(
        surface_ir=SimpleNamespace(source_faces=faces, surface_triangles=triangles)
    )
    guard = exact_work_budget(stage="SEAM_TEST", domain_id="seam")
    wall = build_rational_affine_planar_metric(
        source_revision=factories.REVISION,
        patch_domain_id=factories.DOMAIN,
        owner_patch_id=patch_a,
        source_vertices=vertices,
        source_faces=faces,
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        surface_triangles=triangles,
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
    )
    vault = build_metric((vertices, faces, triangles), owner_patch_id=factories.PATCH).metric
    assert type(wall.planarity_certificate).__name__ == "NearPlanarProjectionCertificateV1"
    assert type(vault.planarity_certificate) is DevelopableUnfoldCertificateV1
    wall_scale, vault_scale = 1 << 10, 1
    lifts = {
        "wall": surface_lift_of(wall, snapshot, patch_a, wall_scale).bind(guard),
        "vault": surface_lift_of(vault, snapshot, factories.PATCH, vault_scale).bind(guard),
    }

    def exact_position(metric, vertex):
        certificate = metric.planarity_certificate
        records = (
            certificate.snapped_source_positions
            if type(certificate) is DevelopableUnfoldCertificateV1
            else certificate.width_distortion.snapped_source_positions
        )
        item = next(entry for entry in records if entry.source_vertex_id == vertex)
        return tuple(
            Fraction(axis.numerator, axis.denominator)
            for axis in (item.position.x, item.position.y, item.position.z)
        )

    s0, s1 = ids["r0a"], ids["r0b"]
    assert exact_position(wall, s0) == exact_position(vault, s0)
    assert exact_position(wall, s1) == exact_position(vault, s1)

    def chart_end(name, metric, scale, vertex):
        item = next(
            entry for entry in metric.exact_source_vertex_coordinates if entry.source_vertex_id == vertex
        )
        from cftuv_envelope.robust.grid import GridSpecV1, snap_value

        grid = GridSpecV1(scale=scale)
        return tuple(
            Fraction(snap_value(Fraction(axis.numerator, axis.denominator) * scale / scale, grid))
            for axis in (item.domain_coordinate.x, item.domain_coordinate.y)
        )

    start, end = exact_position(vault, s0), exact_position(vault, s1)
    ends = {
        "wall": (
            chart_end("wall", wall, wall_scale, s0),
            chart_end("wall", wall, wall_scale, s1),
        ),
        "vault": (
            chart_end("vault", vault, vault_scale, s0),
            chart_end("vault", vault, vault_scale, s1),
        ),
    }
    for parameter in (Fraction(0), Fraction(1, 4), Fraction(1, 2), Fraction(3, 4), Fraction(1)):
        expected = tuple((1 - parameter) * start[axis] + parameter * end[axis] for axis in range(3))
        lifted = {}
        for name, lift in lifts.items():
            (x0, y0), (x1, y1) = ends[name]
            point = (
                SqrtSumV1.rational((1 - parameter) * x0 + parameter * x1),
                SqrtSumV1.rational((1 - parameter) * y0 + parameter * y1),
            )
            exact = lift.lift_exact(point)
            assert tuple(item.as_rational() for item in exact) == expected, (name, parameter)
            lifted[name] = lift.lift(point)
        assert lifted["wall"] == lifted["vault"]
