"""Подъём на плоскость и UV-закон `UV_DIRECT_STRIP_V1`: точно, одно округление."""

from __future__ import annotations

import math
from fractions import Fraction

from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import lift, uv_law
from cftuv_envelope.materialize.stations import chain_station_table

import materialize_factories as factories


def test_the_binary64_conversion_is_the_enclosure_midpoint_and_nothing_else():
    """Тот же перевод, что у отладочного хоста: формула, а не её копия."""

    value = SqrtSumV1.radical(3, 5, factories.budget())  # 3*sqrt(5)
    low, high = value.enclosure(64)
    assert lift.sqrt_sum_binary64(value) == float((low + high) / 2)
    assert abs(lift.sqrt_sum_binary64(value) - 3.0 * math.sqrt(5.0)) <= 4e-15
    # Рациональное число переводится без шума: середина оболочки точки — она.
    assert lift.sqrt_sum_binary64(SqrtSumV1.rational(Fraction(3, 8))) == 0.375
    assert lift.sqrt_sum_binary64(SqrtSumV1.zero()) == 0.0


def test_the_lift_of_a_source_node_is_the_source_position():
    """Сетка 1/4096 держит координаты 0, 6, 10: узлы вершин — сами вершины."""

    prepared, _coverage, _request = factories.two_edge_chain_domain()
    table = chain_station_table(prepared, factories.budget())
    plane = lift.plane_lift_of(prepared.context.frame, table.scale)
    nodes = {
        edge.start_vertex_id: edge.start for edge in table.edges.values()
    }
    point = plane.lift(
        (SqrtSumV1.rational(nodes["v1"][0]), SqrtSumV1.rational(nodes["v1"][1]))
    )
    assert (point.x, point.y, point.z) == (6.0, 0.0, 0.0)
    origin = plane.lift((SqrtSumV1.rational(0), SqrtSumV1.rational(0)))
    assert (origin.x, origin.y, origin.z) == (0.0, 0.0, 0.0)


def test_the_exact_lift_is_linear_before_any_rounding():
    """`lift(p + q) - lift(0) = (lift(p) - lift(0)) + (lift(q) - lift(0))` ТОЧНО."""

    prepared, _coverage, _request = factories.field_domain(
        "building_002_weighted_normals_v1"
    )
    plane = lift.plane_lift_of(
        prepared.context.frame, int(prepared.lattice.scale)
    )

    def exact(x, y):
        return plane.lift_exact((SqrtSumV1.rational(x), SqrtSumV1.rational(y)))

    zero = exact(0, 0)
    first, second, both = exact(3, 5), exact(11, -7), exact(14, -2)
    for axis in range(3):
        assert (both[axis] - zero[axis]) == (
            (first[axis] - zero[axis]) + (second[axis] - zero[axis])
        )


def test_uv_direct_strip_is_the_division_by_alpha():
    s = SqrtSumV1.rational(3)
    r = SqrtSumV1.rational(1)
    uv = uv_law.uv_direct_strip_v1(s, r, Fraction(2))
    assert (uv.u, uv.v) == (1.5, 0.5)
    # Шов: r = 0 даёт v = 0; фронт: r = alpha даёт v = 1 ТОЧНО.
    alpha = Fraction(131072)
    assert uv_law.uv_direct_strip_v1(s, SqrtSumV1.zero(), alpha).v == 0.0
    assert uv_law.uv_direct_strip_v1(s, SqrtSumV1.rational(alpha), alpha).v == 1.0


def test_only_the_declared_uv_policy_is_supported():
    assert uv_law.UV_DIRECT_STRIP_V1 == PolicyId("UV_DIRECT_STRIP_V1")
    assert uv_law.SUPPORTED_UV_POLICIES == {uv_law.UV_DIRECT_STRIP_V1}
    assert PolicyId("ENVELOPE_DEBUG_NO_UV_V1") not in uv_law.SUPPORTED_UV_POLICIES
