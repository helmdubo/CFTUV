from __future__ import annotations

from dataclasses import replace
from math import acos, pi

import pytest
from mathutils import Vector

import cftuv.decal_chart_measurement as chart_measurement
from cftuv.decal_chart_admission import (
    _prepare_disk_topology,
    admit_intrinsic_strip_runtime,
)
from cftuv.decal_chart_measurement import measure_chart_width
from cftuv.decal_chart_parametrization import (
    parameterize_intrinsic_strip_width,
)
from cftuv.decal_charts import (
    ChartBuildFailure,
    _source_vertex_angle,
    build_intrinsic_strip_charts,
    unroll_intrinsic_strip_chart,
)

from decal_organic_fixtures import (
    low_poly_cylinder_fixture,
    low_poly_dome_fixture,
)

_build_intrinsic_strip_charts = build_intrinsic_strip_charts


def build_intrinsic_strip_charts(node, *args, **kwargs):
    kwargs["patch_surface"] = node._test_patch_surface
    return _build_intrinsic_strip_charts(node, *args, **kwargs)


def _angle_defect_values(chart):
    boundary_vertices = {
        vertex
        for edge in chart.boundary_edges
        for vertex in edge.source_edge
    }
    angle_sums = {}
    for triangle in chart.triangles:
        for vertex_index, vertex_id in enumerate(
            triangle.source_vertex_ids
        ):
            angle_sums[vertex_id] = angle_sums.get(vertex_id, 0.0) + (
                _source_vertex_angle(triangle, vertex_index)
            )
    return tuple(
        abs(2.0 * pi - angle_sum)
        for vertex_id, angle_sum in angle_sums.items()
        if vertex_id not in boundary_vertices
    )


def _source_position(triangle, weights):
    return sum(
        (
            Vector(position) * weight
            for position, weight in zip(triangle.positions, weights)
        ),
        Vector(),
    )


def _measurement_profile(chart):
    """W4 diagnostics: тот же width-oracle с per-station receipt."""

    graph = chart_measurement._relation_graph(chart)
    relation_lookup = {
        frozenset((relation.triangle_a, relation.triangle_b)): relation
        for relation in chart.adjacency
    }
    triangles = {
        triangle.triangle_id: triangle for triangle in chart.triangles
    }
    path_cache = {}
    samples = chart_measurement._station_samples(chart)
    result = []
    for station_index, (seed, owner, factor) in enumerate(samples):
        points = dict(zip(owner.source_vertex_ids, owner.chart_points))
        first = points[seed.source_vertex_ids[0]]
        second = points[seed.source_vertex_ids[1]]
        edge = chart_measurement._sub2(second, first)
        edge_length = chart_measurement._distance2(first, second)
        if edge_length <= 1e-15:
            continue
        station = (
            first[0] + edge[0] * factor,
            first[1] + edge[1] * factor,
        )
        station_weights = chart_measurement._barycentric(
            station, owner.chart_points
        )
        station_position = _source_position(owner, station_weights)
        directions = (
            (-edge[1] / edge_length, edge[0] / edge_length),
            (edge[1] / edge_length, -edge[0] / edge_length),
        )
        for side, direction in zip(("LEFT", "RIGHT"), directions):
            probe = chart_measurement._rail_probe(
                chart,
                station,
                direction,
                float(chart.alpha_budget),
            )
            if probe is None:
                result.append(
                    {
                        "station_index": station_index,
                        "station_fraction": (
                            station_index + 0.5
                        ) / len(samples),
                        "side": side,
                        "status": "NO_PROBE",
                    }
                )
                continue
            sampled_distance, _endpoint, located = probe
            target_id, target_weights = located
            cache_key = (owner.triangle_id, target_id)
            paths = path_cache.get(cache_key)
            if paths is None:
                paths = chart_measurement._candidate_paths(
                    graph, *cache_key
                )
                path_cache[cache_key] = paths
            candidates = []
            for path in paths:
                measurement = chart_measurement._unfolded_path_measurement(
                    chart,
                    path,
                    relation_lookup,
                    seed.source_vertex_ids,
                    factor,
                    target_weights,
                )
                if measurement is not None:
                    candidates.append((*measurement, path))
            if not candidates:
                result.append(
                    {
                        "station_index": station_index,
                        "station_fraction": (
                            station_index + 0.5
                        ) / len(samples),
                        "side": side,
                        "status": "NO_VALID_PATH",
                        "sampled_chart_distance": sampled_distance,
                        "target_triangle_id": target_id,
                        "candidate_path_count": len(paths),
                    }
                )
                continue
            geodesic_distance, normal_variation, path = min(
                candidates,
                key=lambda item: (item[0], item[1], item[2]),
            )
            target_position = _source_position(
                triangles[target_id], target_weights
            )
            result.append(
                {
                    "station_index": station_index,
                    "station_fraction": (station_index + 0.5)
                    / len(samples),
                    "side": side,
                    "status": "MEASURED",
                    "sampled_chart_distance": sampled_distance,
                    "hinge_unfolded_distance": geodesic_distance,
                    "relative_width_error": abs(
                        geodesic_distance - sampled_distance
                    )
                    / sampled_distance,
                    "world_chord_distance": (
                        target_position - station_position
                    ).length,
                    "normal_variation_rad": normal_variation,
                    "owner_triangle_id": owner.triangle_id,
                    "target_triangle_id": target_id,
                    "triangle_path": list(path),
                    "candidate_path_count": len(paths),
                }
            )
    return tuple(result)


def _fixture_geometry_contract(node, seeds, alpha):
    positions = {
        vertex_id: position
        for vertex_id, position in zip(
            node.mesh_vert_indices, node.mesh_verts
        )
    }
    path_vertices = [seeds[0].source_vertex_ids[0]] + [
        seed.source_vertex_ids[1] for seed in seeds
    ]
    radii = tuple(positions[vertex_id].length for vertex_id in path_vertices)
    polar_angles = tuple(
        acos(positions[vertex_id].z / radius)
        for vertex_id, radius in zip(path_vertices, radii)
    )
    path_length = sum(
        (positions[second] - positions[first]).length
        for first, second in zip(path_vertices, path_vertices[1:])
    )
    return {
        "polar_angle_min_rad": min(polar_angles),
        "polar_angle_max_rad": max(polar_angles),
        "radius_min": min(radii),
        "radius_max": max(radii),
        "selected_path_chord_length": path_length,
        "requested_half_width": float(alpha),
    }


def _measure_low_poly_fixture(fixture):
    node, seeds, alpha = fixture
    charts = build_intrinsic_strip_charts(
        node,
        seeds,
        alpha_budget=alpha,
    )
    assert len(charts) == 1
    chart = charts[0]
    unrolled = unroll_intrinsic_strip_chart(
        _prepare_disk_topology(chart),
        edge_relative_tolerance=1e-5,
    )
    raw_measurement = measure_chart_width(unrolled)
    parameterized = parameterize_intrinsic_strip_width(unrolled)
    measured = measure_chart_width(parameterized)
    profile = _measurement_profile(parameterized)
    parameterized = replace(
        parameterized,
        metrics=replace(parameterized.metrics, **measured),
    )
    defects = _angle_defect_values(parameterized)
    try:
        admitted = admit_intrinsic_strip_runtime(
            chart,
            initial_alpha=alpha,
            distortion_budget=0.02,
        )
        reason = f"ADMIT_{admitted.admission_tier}"
    except ChartBuildFailure as error:
        reason = error.code
    return {
        "defect_max": max(defects, default=0.0),
        "defect_mean": sum(defects) / len(defects) if defects else 0.0,
        "width_error": measured["max_width_error_sampled"],
        "raw_width_error": raw_measurement["max_width_error_sampled"],
        "normal_variation": measured["max_station_normal_variation"],
        "reason": reason,
        "measurement_mode": "W4_WIDTH_PARAMETERIZED_PRE_ADMISSION",
        "fallback_geometry_evaluated": False,
        "fixture_geometry": _fixture_geometry_contract(
            node, seeds, alpha
        ),
        "station_profile": profile,
    }


@pytest.mark.parametrize(
    ("fixture", "defect_range", "width_range", "reason"),
    (
        (
            low_poly_dome_fixture(16, 8, patch_id=116),
            (0.063, 0.065),
            (0.036, 0.038),
            "DISTORTION_BUDGET_EXCEEDED",
        ),
        (
            low_poly_dome_fixture(32, 16, patch_id=132),
            (0.015, 0.017),
            (0.009, 0.010),
            "DISTORTION_BUDGET_EXCEEDED",
        ),
        (
            low_poly_cylinder_fixture(12, patch_id=212),
            (0.0, 1e-12),
            (0.0, 1e-12),
            "ADMIT_EXACT",
        ),
        (
            low_poly_cylinder_fixture(16, patch_id=216),
            (0.0, 1e-12),
            (0.0, 1e-12),
            "ADMIT_EXACT",
        ),
        (
            low_poly_cylinder_fixture(24, patch_id=224),
            (0.0, 1e-12),
            (0.0, 1e-12),
            "ADMIT_EXACT",
        ),
    ),
)
def test_w4_low_poly_measurement_receipt(
    fixture,
    defect_range,
    width_range,
    reason,
):
    measured = _measure_low_poly_fixture(fixture)

    assert defect_range[0] <= measured["defect_max"] <= defect_range[1]
    assert width_range[0] <= measured["width_error"] <= width_range[1]
    assert measured["reason"] == reason


def test_w4_dome_measurement_is_deterministic():
    fixture = low_poly_dome_fixture(16, 8, patch_id=116)

    assert _measure_low_poly_fixture(fixture) == _measure_low_poly_fixture(
        fixture
    )


def test_g3_dome_recheck_uses_same_latitude_and_exposes_error_profile():
    coarse = _measure_low_poly_fixture(
        low_poly_dome_fixture(16, 8, patch_id=116)
    )
    fine = _measure_low_poly_fixture(
        low_poly_dome_fixture(32, 16, patch_id=132)
    )

    for measured in (coarse, fine):
        assert measured["measurement_mode"] == (
            "W4_WIDTH_PARAMETERIZED_PRE_ADMISSION"
        )
        assert measured["fallback_geometry_evaluated"] is False
        geometry = measured["fixture_geometry"]
        assert geometry["polar_angle_min_rad"] == pytest.approx(pi / 4.0)
        assert geometry["polar_angle_max_rad"] == pytest.approx(pi / 4.0)
        assert geometry["radius_min"] == pytest.approx(5.0)
        assert geometry["radius_max"] == pytest.approx(5.0)
        profile_errors = tuple(
            row["relative_width_error"]
            for row in measured["station_profile"]
            if row["status"] == "MEASURED"
        )
        assert max(profile_errors) == pytest.approx(
            measured["width_error"]
        )

    # W4 устраняет накопление holonomy: на fine fixture КАЖДАЯ станция
    # согласуется с локальным hinge-unfold внутри E2-бюджета.
    for row in fine["station_profile"]:
        if row["status"] != "MEASURED":
            continue
        assert abs(
            row["hinge_unfolded_distance"]
            - row["sampled_chart_distance"]
        ) <= 0.02


def test_w4_dome_resolution_sweep_converges_monotonically():
    errors = tuple(
        _measure_low_poly_fixture(
            low_poly_dome_fixture(
                azimuth_segments,
                latitude_segments,
                patch_id=300 + azimuth_segments,
            )
        )["width_error"]
        for azimuth_segments, latitude_segments in (
            (12, 6),
            (16, 8),
            (20, 10),
            (24, 12),
            (32, 16),
            (48, 24),
            (64, 32),
        )
    )

    assert all(
        following < previous
        for previous, following in zip(errors, errors[1:])
    )
    assert errors[0] < 0.09
    assert errors[-1] < 0.003
