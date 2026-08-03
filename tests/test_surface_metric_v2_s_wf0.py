"""Приёмка V2 на девятке S-WF0: рациональные длины, конусы, ретриангуляция.

Девятка взята не для красоты. На полевых снапшотах всё планарно, и конусный
угол там не отличит плоскость от купола: приёмка была бы пустой. S-WF0 несёт
ровно те развилки, ради которых V2 существует, — гладкую положительную
кривизну (`half_sphere`), развёртываемую поверхность (`cylinder_one_seam`),
две диагонализации одного меша (`retriangulated_surface/a,b`) и два близких
листа, которые нельзя склеивать (`close_parallel_sheets`).
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.contracts.analysis import SurfaceRegime
from cftuv_envelope.contracts.surface_adjacency import (
    surface_adjacency_digest,
    validate_surface_adjacency,
)
from cftuv_envelope.contracts.surface_metric_v2 import (
    ConeAngleMeasureLawV1,
    ConeAngleVerdictV1,
)
from cftuv_envelope.ids import PatchDomainId
from cftuv_envelope.surface_metric_build import build_surface_metric_v2

from s_wf0_surface_corpus import (
    adjacency_for,
    load_fixtures,
    positions_for,
    surface_ir_for,
)


EXPECTED_KEYS = (
    "planar_l",
    "planar_tx",
    "concave_owner",
    "cylinder_one_seam",
    "half_sphere",
    "close_parallel_sheets",
    "retriangulated_surface/a",
    "retriangulated_surface/b",
    "fold_near_smooth",
)


def _regime(key: str) -> SurfaceRegime:
    if key.startswith(("planar_", "concave_", "close_parallel")):
        return SurfaceRegime.DEVELOPABLE
    return SurfaceRegime.GENERAL_CURVED


def _build(fixture):
    surface = surface_ir_for(fixture)
    adjacency = adjacency_for(fixture, surface)
    validate_surface_adjacency(adjacency, surface)
    metric = build_surface_metric_v2(
        patch_domain_id=PatchDomainId(f"s-wf0:{fixture.key}"),
        surface=surface,
        adjacency=adjacency,
        source_positions=positions_for(fixture),
        surface_regime=_regime(fixture.key),
    )
    return surface, adjacency, metric


@pytest.fixture(scope="module")
def built():
    return {fixture.key: _build(fixture) for fixture in load_fixtures()}


def test_the_nine_fixtures_are_the_spike_nine(built):
    assert tuple(sorted(built)) == tuple(sorted(EXPECTED_KEYS))


@pytest.mark.parametrize("key", EXPECTED_KEYS)
def test_every_squared_length_is_rational_and_positive(built, key):
    """Piecewise-flat метрика: длины квадратами, рационально и точно."""

    _surface, _adjacency, metric = built[key]
    assert metric.intrinsic.triangle_squared_lengths
    for item in metric.intrinsic.triangle_squared_lengths:
        for side in (item.side_0, item.side_1, item.side_2):
            value = Fraction(side.numerator, side.denominator)
            assert value > 0
            # Привязка к решётке 1/scale: знаменатель — её квадрат либо делитель.
            assert value.denominator <= metric.grid_certificate.source_scale ** 2
    assert not metric.intrinsic.degenerate_triangles


@pytest.mark.parametrize("key", EXPECTED_KEYS)
def test_gram_determinant_agrees_with_the_squared_lengths(built, key):
    """Грам и квадраты длин — одна геометрия, посчитанная двумя путями."""

    _surface, _adjacency, metric = built[key]
    grams = {item.triangle_id: item.gram for item in metric.intrinsic.triangle_gram_matrices}
    for item in metric.intrinsic.triangle_squared_lengths:
        gram = grams[item.triangle_id]
        g00 = Fraction(gram.m00.numerator, gram.m00.denominator)
        g01 = Fraction(gram.m01.numerator, gram.m01.denominator)
        g11 = Fraction(gram.m11.numerator, gram.m11.denominator)
        first = Fraction(item.side_0.numerator, item.side_0.denominator)
        third = Fraction(item.side_2.numerator, item.side_2.denominator)
        second = Fraction(item.side_1.numerator, item.side_1.denominator)
        # Сторона 0 — (v0,v1), сторона 1 — (v1,v2), сторона 2 — (v2,v0).
        assert g00 == first
        assert g11 == third
        # |v1v2|² = |v0v1|² − 2·(v0v1·v0v2) + |v0v2|²: теорема косинусов в
        # квадратах. Ни одного корня, ни одного округления.
        assert second == first - 2 * g01 + third


def test_half_sphere_cone_angles_are_strictly_less_than_two_pi(built):
    """Положительная кривизна: дефект есть у КАЖДОЙ вершины, и он доказан."""

    _surface, _adjacency, metric = built["half_sphere"]
    verdicts = [item.verdict for item in metric.vertex_cone_angles]
    assert verdicts
    assert set(verdicts) == {ConeAngleVerdictV1.STRICTLY_LESS}
    assert {item.measure_law for item in metric.vertex_cone_angles} == {
        ConeAngleMeasureLawV1.CERTIFIED_INTERVAL_ENCLOSURE_V1
    }


def test_planar_fixtures_prove_interior_cones_exactly(built):
    """Плоские веера дают EXACT_TWO_PI ТОЧНЫМ выводом, не оболочкой."""

    for key in ("planar_l", "planar_tx", "concave_owner", "close_parallel_sheets"):
        _surface, adjacency, metric = built[key]
        exact = [
            item
            for item in metric.vertex_cone_angles
            if item.verdict is ConeAngleVerdictV1.EXACT_TWO_PI
        ]
        assert exact, key
        assert all(
            item.measure_law is ConeAngleMeasureLawV1.EXACT_PLANAR_CLOSED_FAN_V1
            for item in exact
        )
        closed = [fan for fan in adjacency.vertex_fans if getattr(fan, "is_closed", False)]
        assert len(exact) == len(closed), key


def test_developable_snapped_cones_fail_closed_instead_of_claiming_flatness(built):
    """Цилиндр после привязки не плоский точно — и не называется плоским."""

    _surface, _adjacency, metric = built["cylinder_one_seam"]
    verdicts = {item.verdict for item in metric.vertex_cone_angles}
    assert ConeAngleVerdictV1.EXACT_TWO_PI not in verdicts
    assert ConeAngleVerdictV1.UNDECIDED_FAIL_CLOSED in verdicts


def test_close_parallel_sheets_stay_two_components(built):
    """Два листа в 4 см друг от друга не склеиваются ни одной стороной."""

    from cftuv_envelope.contracts.surface_adjacency import TriangleSideRefV1

    surface, adjacency, _metric = built["close_parallel_sheets"]
    glued = [
        (side.side.triangle_id, side.opposite.triangle_id)
        for side in adjacency.triangle_sides
        if isinstance(side.opposite, TriangleSideRefV1)
    ]
    components = _components(
        glued, {item.triangle_id for item in surface.surface_triangles}
    )
    assert len(components) == 2


def _components(glued, nodes):
    parent = {node: node for node in nodes}

    def find(node):
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    for left, right in glued:
        parent[find(left)] = find(right)
    return {find(node) for node in nodes}


def test_retriangulation_pair_shares_geometry_and_differs_only_in_adjacency(built):
    """a/b: позиции, барьеры и вердикты те же; таблицы смежности РАЗНЫЕ.

    «Одинаковые углы» здесь означает одинаковые ВЕРДИКТЫ и согласованные
    оболочки, а не побитовое равенство интервалов. Причина названа, а не
    обойдена: ширина сертифицированной оболочки — функция ЧИСЛА слагаемых,
    а диагональ меняет число треугольников при вершине. Требовать побитового
    равенства значило бы требовать, чтобы точность не зависела от того, сколько
    раз её складывали, — то есть печатать точность, которой нет.
    """

    _sa, adjacency_a, metric_a = built["retriangulated_surface/a"]
    _sb, adjacency_b, metric_b = built["retriangulated_surface/b"]

    assert surface_adjacency_digest(adjacency_a) != surface_adjacency_digest(
        adjacency_b
    )
    assert _positions(metric_a) == _positions(metric_b)
    assert metric_a.barriers == metric_b.barriers

    left = {_number(item.vertex_id): item for item in metric_a.vertex_cone_angles}
    right = {_number(item.vertex_id): item for item in metric_b.vertex_cone_angles}
    assert set(left) == set(right)
    bound = max(
        metric_a.named_epsilon.absolute_bound, metric_b.named_epsilon.absolute_bound
    )
    for key in left:
        assert left[key].verdict is right[key].verdict
        assert left[key].enclosure.lower <= right[key].enclosure.upper
        assert right[key].enclosure.lower <= left[key].enclosure.upper
        assert abs(left[key].enclosure.lower - right[key].enclosure.lower) <= bound
        assert abs(left[key].enclosure.upper - right[key].enclosure.upper) <= bound


def _number(identity):
    return int(str(identity.value).rsplit(":", 1)[1])


def _positions(metric):
    return sorted(
        (
            _number(item.source_vertex_id),
            (item.position.x.numerator, item.position.x.denominator),
            (item.position.y.numerator, item.position.y.denominator),
            (item.position.z.numerator, item.position.z.denominator),
        )
        for item in metric.snapped_source_positions
    )


@pytest.mark.parametrize("key", EXPECTED_KEYS)
def test_every_descriptor_names_its_table_and_its_epsilon(built, key):
    _surface, adjacency, metric = built[key]
    assert metric.adjacency_ref.adjacency_digest == surface_adjacency_digest(
        adjacency
    )
    assert metric.named_epsilon is not None
    assert metric.named_epsilon.absolute_bound > 0
