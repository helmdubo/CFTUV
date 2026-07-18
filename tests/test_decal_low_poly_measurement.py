from __future__ import annotations

from dataclasses import replace
from math import pi

import pytest

from cftuv.decal_chart_admission import (
    _prepare_disk_topology,
    admit_intrinsic_strip_runtime,
)
from cftuv.decal_chart_measurement import measure_chart_width
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
    measured = measure_chart_width(unrolled)
    unrolled = replace(
        unrolled,
        metrics=replace(unrolled.metrics, **measured),
    )
    defects = _angle_defect_values(unrolled)
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
        "normal_variation": measured["max_station_normal_variation"],
        "reason": reason,
    }


@pytest.mark.parametrize(
    ("fixture", "defect_range", "width_range", "reason"),
    (
        (
            low_poly_dome_fixture(16, 8, patch_id=116),
            (0.063, 0.065),
            (0.060, 0.061),
            "DISTORTION_BUDGET_EXCEEDED",
        ),
        (
            low_poly_dome_fixture(32, 16, patch_id=132),
            (0.015, 0.017),
            (0.408, 0.410),
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
