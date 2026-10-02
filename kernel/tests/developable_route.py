"""Входы тестов лестницы метрики: публичный построитель в режиме хоста и полный путь домена."""

from __future__ import annotations

import dataclasses

import cftuv_envelope as kernel
from cftuv_envelope.contracts.metric import (
    CurvatureLadderPolicyV1,
    GridSnappingLawV1,
    NearPlanarFramePolicyV1,
    NearPlanarLiftLawV1,
    PlanarityAdmissionLawV1,
)
from cftuv_envelope.planar_metric import (
    build_embedding_certified_rational_affine_planar_metric,
)

from developable_factories import DOMAIN, PATCH, REVISION


def build_metric(parts, *, ladder=CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1, **overrides):
    """Метрика через публичный построитель в режиме хоста: near-planar на поверхность + лестница."""

    vertices, faces, triangles = parts
    options = dict(
        source_revision=REVISION,
        patch_domain_id=DOMAIN,
        owner_patch_id=PATCH,
        source_vertices=vertices,
        source_faces=faces,
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        surface_triangles=triangles,
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
        near_planar_frame_policy=NearPlanarFramePolicyV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1,
        curvature_ladder=ladder,
    )
    options.update(overrides)
    return build_embedding_certified_rational_affine_planar_metric(**options)


def developable_domain(parts, route_names, *, alpha="0.2"):
    """Снапшот и запрос домена-развёртки: честные 3D-позиции, метрика через лестницу.

    Помощник `straight_snapshot` берёт плоские многоугольники и разделяет грани по
    ТОЧКАМ, поэтому вершины поверхности здесь подписаны координатами их собственной
    развёртки (метры, дроби по степеням двойки): тогда петли и обход плоского
    каркаса согласованы с картой, а позиции в 3D подставляются поверх, и
    метрика строится построителем ядра из НИХ, как у хоста.
    """

    from materialize_factories import with_affine_metric
    from reference_factories import straight_snapshot

    from developable_factories import developable_chart

    vertices, faces, _triangles = parts
    chart = developable_chart(parts)
    scale = chart.chart_scale
    label = {
        vertex_id: (chart.nodes[vertex_id][0] / scale, chart.nodes[vertex_id][1] / scale)
        for vertex_id in chart.nodes
    }
    by_name = {item.vertex_id.value[2:]: item.vertex_id for item in vertices}
    cycles = tuple(tuple(label[vertex] for vertex in face.vertex_cycle) for face in faces)
    snapshot, request = straight_snapshot(
        faces=cycles,
        source_routes=(
            {"name": "source", "points": tuple(label[by_name[name]] for name in route_names)},
        ),
        alpha=alpha,
    )
    order = []
    for cycle in cycles:
        for point in cycle:
            if point not in order:
                order.append(point)
    point_vertex = {label[vertex]: vertex for vertex in label}
    position = {item.vertex_id: item.position for item in vertices}
    snapshot = dataclasses.replace(
        snapshot,
        source_vertices=frozenset(
            dataclasses.replace(
                item, position=position[point_vertex[order[int(item.vertex_id.value[1:])]]]
            )
            for item in snapshot.source_vertices
        ),
    )
    snapshot = with_affine_metric(
        snapshot,
        grid_policy=GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1,
        planarity_policy=PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        near_planar_lift_law=NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1,
        near_planar_frame_policy=NearPlanarFramePolicyV1.REDUCED_INTEGER_PLANE_LATTICE_BASIS_V1,
        curvature_ladder=CurvatureLadderPolicyV1.NEAR_PLANAR_THEN_DEVELOPABLE_UNFOLD_V1,
    )
    return snapshot, request


def materialize_developable(parts, route_names, *, alpha="0.2", **kwargs):
    """`(MaterializationV1, подготовка)` домена-развёртки продуктовым путём ядра.

    `kwargs` — остальные именованные параметры `materialize_domain` (закон топологии, закон
    укладки `near_planar_lift_law`, по умолчанию `SOURCE_TRIANGLES_V1`).
    """

    from materialize_factories import prepare_and_cover

    from cftuv_envelope.materialize.admit import materialization_request
    from cftuv_envelope.materialize.domain import materialize_domain

    snapshot, request = developable_domain(parts, route_names, alpha=alpha)
    prepared, coverage = prepare_and_cover(snapshot, request)
    kwargs.setdefault("near_planar_lift_law", NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1)
    result = materialize_domain(
        prepared,
        coverage,
        request=materialization_request(prepared, uv_policy_id="UV_DIRECT_STRIP_V1"),
        **kwargs,
    )
    return result, prepared
