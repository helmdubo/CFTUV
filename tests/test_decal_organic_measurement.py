import pytest

from cftuv.decal_chart_admission import _prepare_disk_topology
from cftuv.decal_chart_measurement import measure_chart_width
from cftuv.decal_charts import (
    build_intrinsic_strip_charts,
    unroll_intrinsic_strip_chart,
)

from decal_organic_fixtures import (
    cliff_fixture,
    crumple_fixture,
    intermediate_dome_fixture,
    saddle_fixture,
    sphere_cap_fixture,
    tight_sphere_fixture,
)

_build_intrinsic_strip_charts = build_intrinsic_strip_charts


def build_intrinsic_strip_charts(node, *args, **kwargs):
    kwargs["patch_surface"] = node._test_patch_surface
    return _build_intrinsic_strip_charts(node, *args, **kwargs)


def _measure(fixture):
    node, seeds, alpha = fixture
    chart = build_intrinsic_strip_charts(
        node, seeds, alpha_budget=alpha
    )[0]
    unrolled = unroll_intrinsic_strip_chart(
        _prepare_disk_topology(chart),
        edge_relative_tolerance=1e-5,
    )
    return unrolled.metrics, measure_chart_width(unrolled)


@pytest.mark.parametrize(
    ("fixture", "error_range", "overlap_expected", "foldover_expected"),
    (
        (sphere_cap_fixture(12.0), (0.0, 0.005), False, False),
        (cliff_fixture(), (0.0, 0.005), True, False),
        (saddle_fixture(12.0), (0.0, 0.005), True, False),
        (intermediate_dome_fixture(), (0.02, 0.05), False, False),
        (tight_sphere_fixture(), (0.10, 1.0), False, False),
        (crumple_fixture(), (0.10, 1.0), True, True),
    ),
)
def test_e0_b_organic_spike_matches_provisional_thresholds(
    fixture,
    error_range,
    overlap_expected,
    foldover_expected,
):
    metrics, measured = _measure(fixture)
    assert error_range[0] < measured["max_width_error_sampled"] < error_range[1]
    assert bool(metrics.triangle_overlap_count) is overlap_expected
    assert bool(measured["foldover_count"]) is foldover_expected


def test_e0_width_measurement_is_deterministic():
    fixture = intermediate_dome_fixture()
    first = _measure(fixture)
    second = _measure(fixture)

    assert first == second
