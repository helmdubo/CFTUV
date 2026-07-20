from __future__ import annotations

import inspect

import pytest

from cftuv.decal_corner_model import (
    BandSide,
    CornerAnchor,
    CornerAnchorKind,
    CornerModel,
    CornerPointProvenance,
    CornerSeed,
    CornerStationRef,
    CornerStripVertex,
    CornerVertexRef,
    LocalClippedCornerStrip,
)
from cftuv.model import CornerJoinMode


def _provenance(edge_id, station_key):
    route_id = ("route", edge_id)
    return CornerPointProvenance(
        source_face_id=17,
        source_edge_ids=(edge_id,),
        route_ids=(route_id,),
        station_keys=(station_key,),
    )


def _model(join):
    vertex_provenance = CornerPointProvenance(
        source_face_id=17,
        source_edge_ids=(3, 5),
        route_ids=(("route", 3), ("route", 5)),
        station_keys=(("station", 3), ("station", 5)),
    )
    seed = CornerSeed(
        apex_ref=CornerVertexRef(
            key=("V", 11),
            chart_point=(0.0, 0.0),
            position=(0.0, 0.0, 0.125),
            provenance=vertex_provenance,
        ),
        corner_vertex_id=11,
        incident_site_ids=(3, 5),
        side=BandSide.POSITIVE,
        sector_id=None,
        owner_surface_id=17,
        join=join,
    )
    p1_station = CornerStationRef(
        route_id=("route", 3),
        site_id=3,
        source_edge_id=3,
        station_key=("station", 3),
        s=4.0,
        r=0.5,
        tangent_away=(1.0, 0.0),
    )
    p2_station = CornerStationRef(
        route_id=("route", 5),
        site_id=5,
        source_edge_id=5,
        station_key=("station", 5),
        s=4.0,
        r=0.5,
        tangent_away=(0.0, 1.0),
    )
    return CornerModel(
        seed=seed,
        p1=CornerAnchor(
            kind=CornerAnchorKind.P1,
            key=("P1", 11),
            chart_point=(-0.5, 0.5),
            provenance=_provenance(3, ("station", 3)),
            station_ref=p1_station,
        ),
        p2=CornerAnchor(
            kind=CornerAnchorKind.P2,
            key=("P2", 11),
            chart_point=(0.5, -0.5),
            provenance=_provenance(5, ("station", 5)),
            station_ref=p2_station,
        ),
    )


def test_corner_model_bevel_planar_isolated_oracle_is_one_triangle():
    model = _model(CornerJoinMode.BEVEL)

    derived = model.derive()

    assert derived.collision_boundary is derived.ownership_boundary
    assert derived.collision_boundary is derived.uv_boundary
    assert derived.collision_boundary is derived.projection_boundary
    assert derived.collision_boundary is derived.emission_boundary
    assert tuple(vertex.semantic_id for vertex in derived.emission_boundary) == (
        "V",
        "P1",
        "P2",
    )
    assert len(derived.emission_boundary) == 3
    faces = model.emit_isolated()
    assert len(faces) == 1
    assert len(faces[0].vertices) == 3
    assert len(faces[0].traces) == 3


def test_corner_model_miter_and_bevel_share_the_same_anchor_instances():
    miter = _model(CornerJoinMode.MITER)
    bevel = CornerModel(
        seed=CornerSeed(
            apex_ref=miter.seed.apex_ref,
            corner_vertex_id=miter.seed.corner_vertex_id,
            incident_site_ids=miter.seed.incident_site_ids,
            side=miter.seed.side,
            sector_id=miter.seed.sector_id,
            owner_surface_id=miter.seed.owner_surface_id,
            join=CornerJoinMode.BEVEL,
        ),
        p1=miter.p1,
        p2=miter.p2,
    )

    assert bevel.p1 is miter.p1
    assert bevel.p2 is miter.p2
    assert len(miter.derive().emission_boundary) == 4
    assert len(bevel.derive().emission_boundary) == 3


def test_corner_model_reads_ordered_anchors_from_local_strip_corners():
    expected = _model(CornerJoinMode.BEVEL)
    strips = tuple(
        LocalClippedCornerStrip(
            site_id=anchor.station_ref.site_id,
            vertices=(
                CornerStripVertex(
                    key=anchor.key,
                    chart_point=anchor.chart_point,
                    provenance=anchor.provenance,
                    station_ref=anchor.station_ref,
                ),
            ),
            outer_corner_key=anchor.key,
        )
        for anchor in (expected.p1, expected.p2)
    )

    model = CornerModel.from_local_strips(expected.seed, *strips)

    assert model.p1.key == strips[0].outer_corner.key
    assert model.p2.key == strips[1].outer_corner.key
    assert model.p1.provenance is strips[0].outer_corner.provenance
    assert model.p2.station_ref is strips[1].outer_corner.station_ref


def test_resolved_corner_view_reuses_anchors_and_traces_contour_vertices():
    model = _model(CornerJoinMode.BEVEL)
    vertices = tuple(
        (vertex.key, vertex.chart_point)
        for vertex in model.derive().emission_boundary
    )

    resolved = model.resolve(vertices)

    assert resolved.p1 is model.p1
    assert resolved.p2 is model.p2
    assert len(resolved.traces) == 3


def test_resolved_miter_view_reuses_the_same_derived_boundary():
    model = _model(CornerJoinMode.MITER)
    derived = model.derive()
    vertices = tuple(
        (vertex.key, vertex.chart_point)
        for vertex in derived.emission_boundary
    )

    resolved = model.resolve(vertices)

    assert resolved.derived.emission_boundary == derived.emission_boundary
    assert resolved.p1 is model.p1
    assert resolved.p2 is model.p2
    assert tuple(
        trace.contour_edge for trace in resolved.traces
    ) == (
        ("V", "P1"),
        ("V", "P1"),
        ("P1", "MITER_APEX"),
        ("MITER_APEX", "P2"),
    )


def test_rf28_fan_discretization_cannot_masquerade_as_bevel_chord():
    model = _model(CornerJoinMode.BEVEL)

    with pytest.raises(
        ValueError, match="CORNER_MATERIAL_VERTEX_OUTSIDE_SEMANTIC_CONTOUR"
    ):
        model.resolve((("legacy-fan-sample", (0.0, 0.4)),))


def test_rf28_join_branch_is_owned_only_by_corner_model():
    source = inspect.getsource(CornerModel._material_boundary)

    assert source.count("CornerJoinMode.BEVEL") == 1
    assert "CornerJoinMode.MITER" not in source
