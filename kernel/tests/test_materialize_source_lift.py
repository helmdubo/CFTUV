"""Закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1`: вершина `src:` стоит в позиции вершины хоста.

Два слоя. Малый — сам закон на синтетических позициях (бюджет точно по границе, откат
перевернувшейся грани, независимость от закона топологии). Большой — полный путь настоящего
домена: каждая вершина `src:` либо в точной позиции исходника, либо названа среди
оставшихся, и счётчики сходятся.
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction

import pytest

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import domain
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.source_lift import (
    DISPLACED,
    FACES_OFF_PLANE,
    FACES_TRIANGULATED_AFTER_LIFT,
    FOLLOWED,
    LIFTED,
    ORIENTATION_KEPT,
    SOURCE_VERTEX_LIFT_BUDGET_CELLS,
    TRIANGLES_FLIPPED_BY_LIFT,
    UNAVAILABLE,
    lift_source_vertices,
    off_plane_distance,
    settle_emitted_faces,
    source_step_of,
)
from cftuv_envelope.materialize.tessellate import triangulate_exact
from cftuv_envelope.wavefront.faces import doubled_shoelace
from cftuv_envelope.numeric import LocalPoint3V1
from cftuv_envelope.outcomes import NamedOutcome

import materialize_factories as factories

STEP = Fraction(1, 16)
UV = PolicyId("UV_DIRECT_STRIP_V1")


def _point(x, y=0.0, z=0.0):
    return LocalPoint3V1(float(x), float(y), float(z))


def _bits(point):
    return tuple(axis.hex() for axis in (point.x, point.y, point.z))


# --------------------------------------------------------------------------
# Закон на синтетических позициях
# --------------------------------------------------------------------------


def test_a_vertex_within_the_budget_is_lifted_at_the_exact_host_position():
    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "src:a": _point(0.5, 1.0),
    }
    host = {"a": _point(0.5 + 0.01, 1.0 - 0.02, 0.005)}

    result = lift_source_vertices(positions, [("node:0", "node:1", "src:a")], host, STEP)

    assert _bits(result.positions["src:a"]) == _bits(host["a"])
    assert result.positions["node:0"] == positions["node:0"]
    assert result.positions["node:1"] == positions["node:1"]
    assert (result.lifted, result.moved, result.displaced) == (1, 1, 0)
    assert result.unavailable == result.kept_for_orientation == 0
    assert dict(result.counters())[LIFTED] == 1
    assert SOURCE_VERTEX_LIFT_BUDGET_CELLS == 1


def test_the_budget_is_exactly_one_source_cell_and_the_comparison_is_exact():
    """Расстояние РОВНО в ячейку — ещё в бюджете; на один бит больше — уже нет."""

    base = {"node:0": _point(0.0), "node:1": _point(1.0), "src:a": _point(0.5, 1.0)}
    polygon = [("node:0", "node:1", "src:a")]
    on_the_border = lift_source_vertices(
        base, polygon, {"a": _point(0.5 + float(STEP), 1.0)}, STEP
    )
    just_beyond = lift_source_vertices(
        base, polygon, {"a": _point(0.5 + float(STEP) + 2.0**-30, 1.0)}, STEP
    )

    assert on_the_border.lifted == 1 and on_the_border.displaced == 0
    assert just_beyond.lifted == 0 and just_beyond.displaced == 1


def test_a_vertex_beyond_the_budget_stays_and_is_named():
    positions = {"node:0": _point(0.0), "node:1": _point(1.0), "src:a": _point(0.5, 1.0)}
    host = {"a": _point(0.5 + 4 * float(STEP), 1.0)}

    result = lift_source_vertices(positions, [("node:0", "node:1", "src:a")], host, STEP)

    assert result.positions["src:a"] == positions["src:a"]
    assert (result.lifted, result.displaced) == (0, 1)
    key, distance = result.worst_displaced
    assert key == "src:a" and distance == pytest.approx(4 * float(STEP))
    assert "src:a" in result.displaced_note() and "farther than the budget" in result.displaced_note()
    assert dict(result.counters())[DISPLACED] == 1


def test_only_src_vertices_are_ever_moved():
    positions = {"node:0": _point(0.0), "node:1": _point(1.0), "src:a": _point(0.5, 1.0)}
    host = {"a": _point(0.5, 1.0), "0": _point(9.0), "node:0": _point(9.0)}

    result = lift_source_vertices(positions, [("node:0", "node:1", "src:a")], host, STEP)

    assert result.positions["node:0"] == positions["node:0"]
    assert result.total == 1


@pytest.mark.parametrize("missing", ("position", "cell"))
def test_an_unknown_host_position_or_cell_is_counted_and_nothing_moves(missing):
    positions = {"node:0": _point(0.0), "node:1": _point(1.0), "src:a": _point(0.5, 1.0)}
    host = {} if missing == "position" else {"a": _point(0.5, 1.0)}
    step = STEP if missing == "position" else None

    result = lift_source_vertices(positions, [("node:0", "node:1", "src:a")], host, step)

    assert result.positions == positions
    assert (result.lifted, result.unavailable) == (0, 1)
    assert dict(result.counters())[UNAVAILABLE] == 1


def test_a_thin_face_the_host_position_would_turn_over_moves_its_nodes_with_the_vertex():
    """Сливер высотой в доли ячейки: подъём вершины его перевернул бы, и его узлы сдвигаются вместе с ней."""

    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "src:c": _point(0.5, 0.00001),
    }
    host = {"c": _point(0.5, -0.00002)}
    triangle = ("node:0", "node:1", "src:c")

    result = lift_source_vertices(positions, [triangle], host, STEP)

    assert _bits(result.positions["src:c"]) == _bits(host["c"])
    assert (result.lifted, result.kept_for_orientation) == (1, 0)
    assert (result.followed, result.followed_drivers) == (2, 1)
    # Тот же вектор подвижки, покоординатно в binary64: треугольник перенесён жёстко.
    shift = (host["c"].x - positions["src:c"].x, host["c"].y - positions["src:c"].y)
    for key in ("node:0", "node:1"):
        assert result.positions[key].x == positions[key].x + shift[0]
        assert result.positions[key].y == positions[key].y + shift[1]
        assert result.positions[key].z == positions[key].z
    before = _area_z(*(positions[key] for key in triangle))
    after = _area_z(*(result.positions[key] for key in triangle))
    assert before * after > 0.0
    assert result.max_follow_displacement == pytest.approx(3e-5)
    assert result.max_follow_displacement <= result.budget
    assert dict(result.counters())[FOLLOWED] == 2
    assert dict(result.counters())[ORIENTATION_KEPT] == 0
    assert "moved rigidly" in result.followed_note()


def test_a_domain_where_nothing_follows_has_no_follow_counter():
    """Закон, который не сработал, ничего не пишет: счётчики прежних доменов остаются прежними."""

    positions = {"node:0": _point(0.0), "node:1": _point(1.0), "src:a": _point(0.5, 1.0)}
    result = lift_source_vertices(
        positions, [("node:0", "node:1", "src:a")], {"a": _point(0.5 + 0.01, 1.0)}, STEP
    )
    assert result.followed == 0
    assert FOLLOWED not in dict(result.counters())


def test_a_thin_face_without_a_free_node_keeps_the_lattice_lift():
    """Три вершины `src:`: узлов, которым можно следовать, нет, и перевернувшая грань вершина остаётся на узле."""

    positions = {
        "src:a": _point(0.0),
        "src:b": _point(1.0),
        "src:c": _point(0.5, 0.00001),
    }
    host = {"a": _point(0.0), "b": _point(1.0), "c": _point(0.5, -0.00002)}
    triangle = ("src:a", "src:b", "src:c")

    result = lift_source_vertices(positions, [triangle], host, STEP)

    assert result.positions == positions
    assert (result.lifted, result.kept_for_orientation, result.followed) == (2, 1, 0)
    assert dict(result.counters())[ORIENTATION_KEPT] == 1
    assert "turn a face contour over" in result.orientation_note()


def test_a_reverted_vertex_is_rechecked_against_its_neighbours_to_a_fixed_point():
    """Откат вершины меняет соседние грани: проверка идёт, пока что-то откатывается."""

    positions = {
        "src:n0": _point(0.0),
        "src:n1": _point(1.0),
        "src:a": _point(0.5, 0.00001),
        "src:b": _point(0.5, -1.0),
    }
    host = {
        "n0": _point(0.0),
        "n1": _point(1.0),
        "a": _point(0.5, -0.00002),
        "b": _point(0.5, -1.0 - 0.01),
    }
    faces = [("src:n0", "src:n1", "src:a"), ("src:n1", "src:n0", "src:b")]

    result = lift_source_vertices(positions, faces, host, STEP)

    assert result.kept_for_orientation >= 1
    assert result.positions["src:a"] == positions["src:a"]
    for first, second, third in faces:
        before = _area_z(positions[first], positions[second], positions[third])
        after = _area_z(*(result.positions[key] for key in (first, second, third)))
        assert before * after > 0.0, (first, second, third)


def _no_ear_turned(result, positions, faces):
    for keys in faces:
        before = _area_z(*(positions[key] for key in keys))
        after = _area_z(*(result.positions[key] for key in keys))
        assert before * after > 0.0, keys


def test_following_runs_along_a_chain_of_needles_to_a_fixed_point():
    """Узел, за которым сдвинулся узел, переворачивает ЕГО иголку: следует и тот, пока контур не устоит."""

    positions = {
        "node:0": _point(0.0, 0.0),
        "node:1": _point(0.5, -0.000003),
        "node:2": _point(0.25, -0.00002),
        "node:3": _point(0.1, -0.00002),
        "src:s": _point(1.0, 0.0),
    }
    host = {"s": _point(1.0, -0.00004)}
    faces = [
        ("node:0", "src:s", "node:1"),
        ("node:1", "node:2", "node:0"),
        ("node:2", "node:3", "node:0"),
    ]

    result = lift_source_vertices(positions, faces, host, STEP)

    assert (result.lifted, result.kept_for_orientation) == (1, 0)
    assert (result.followed, result.followed_drivers) == (4, 1)
    for key in ("node:0", "node:1", "node:2", "node:3"):
        assert result.positions[key].y == positions[key].y + (host["s"].y - positions["src:s"].y)
        assert result.positions[key].x == positions[key].x
    _no_ear_turned(result, positions, faces)


def test_a_node_that_needs_two_different_vectors_gives_the_vertices_back():
    """Узел в иголках двух подвинутых вершин не может следовать за обеими: обе возвращаются на узлы."""

    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "src:a": _point(0.5, 0.00001),
        "src:b": _point(0.5, -0.00001),
    }
    host = {"a": _point(0.5, -0.00002), "b": _point(0.5, 0.00002)}
    faces = [("node:0", "node:1", "src:a"), ("node:1", "node:0", "src:b")]

    result = lift_source_vertices(positions, faces, host, STEP)

    assert result.positions == positions
    assert (result.lifted, result.kept_for_orientation, result.followed) == (0, 2, 0)


def test_a_follower_that_breaks_a_fixed_neighbour_takes_its_driver_back_with_its_nodes():
    """Узлы, следовавшие за вершиной, ломают грань с неподвижной вершиной `src:`: откат убирает и вершину, и узлы."""

    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "src:a": _point(0.5, 0.00001),
        "src:u": _point(0.5, -0.000003),
    }
    host = {"a": _point(0.5, -0.00002), "u": _point(0.5, -0.000003)}
    faces = [("node:0", "node:1", "src:a"), ("src:u", "node:0", "node:1")]

    result = lift_source_vertices(positions, faces, host, STEP)

    assert result.positions == positions
    assert (result.lifted, result.kept_for_orientation, result.followed) == (1, 1, 0)
    assert FOLLOWED not in dict(result.counters())


def _area_z(a, b, c):
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)


def _chart(raw):
    """Точные точки карты `{ключ: (SqrtSumV1, SqrtSumV1)}` из `{ключ: (x, y)}`."""

    return {
        key: (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))
        for key, (x, y) in raw.items()
    }


def _chart_sign(points):
    return doubled_shoelace(tuple(points)).sign(budget=factories.budget())


def _canonical_ears(raw):
    """Канонические треугольники закона `TRIANGLES_V1` контура: точные уши по ключам."""

    keys = tuple(raw)
    chart = _chart(raw)
    ears = triangulate_exact([chart[key] for key in keys], factories.budget())
    return tuple(tuple(keys[index] for index in ear) for ear in ears)


def test_a_fan_from_the_first_vertex_is_not_a_triangulation_the_exact_ears_are():
    """Прямая вершина на ребре многоугольника: веер из первой вершины даёт треугольник нулевой
    площади, и любая её подвижка выглядела «разворотом» (молчаливая потеря сварки). Точные уши
    такого треугольника не имеют: вершина ложится в позицию хоста."""

    raw = {
        "node:0": (0, 0),
        "src:1": (2, 0),
        "node:2": (4, 0),
        "node:3": (4, 2),
        "node:4": (0, 2),
    }
    positions = {key: _point(x, y) for key, (x, y) in raw.items()}
    host = {"1": _point(2.0, 0.01, 0.002)}

    keys = tuple(raw)
    fan = [(keys[0], keys[index], keys[index + 1]) for index in range(1, len(keys) - 1)]
    by_ears = lift_source_vertices(positions, _canonical_ears(raw), host, STEP)
    by_fan = lift_source_vertices(positions, fan, host, STEP)

    assert (by_ears.lifted, by_ears.kept_for_orientation) == (1, 0)
    assert _bits(by_ears.positions["src:1"]) == _bits(host["1"])
    assert (by_fan.lifted, by_fan.kept_for_orientation) == (0, 1)


def test_the_lift_takes_canonical_triangles_so_the_positions_cannot_see_the_emitted_faces():
    """`lift_source_vertices` не знает о гранях закона: тот же набор треугольников — те же позиции."""

    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "node:2": _point(1.0, 1.0),
        "src:q": _point(0.0, 1.0),
        "src:t": _point(0.5, 0.00001),
    }
    host = {"q": _point(0.003, 1.004), "t": _point(0.5, -0.00002)}
    triangles = (
        ("src:q", "node:0", "node:1"),
        ("node:1", "node:2", "src:q"),
        ("node:0", "node:1", "src:t"),
    )

    first = lift_source_vertices(positions, triangles, host, STEP)
    again = lift_source_vertices(positions, iter(triangles), host, STEP)

    assert first.positions == again.positions
    # Сливер вершины `t` не перевернулся: его узлы сдвинулись вместе с ней, обе вершины в позиции хоста.
    assert (first.kept_for_orientation, first.lifted, first.followed) == (0, 2, 2)


# --------------------------------------------------------------------------
# Выпущенные грани после сдвига (`settle_emitted_faces`)
# --------------------------------------------------------------------------

SQUARE = {"a": (0, 0), "b": (4, 0), "c": (4, 4), "d": (0, 4)}
L_SHAPE = {
    "node:0": (0, 0),
    "node:1": (4, 0),
    "src:2": (4, 2),
    "node:3": (2, 2),
    "node:4": (2, 4),
    "node:5": (0, 4),
}


def _settle(raw, final_overrides, polygons, *, reverse=False):
    """`settle_emitted_faces` на синтетике: подъём — карта в плоскости `z = 0`, итог — с подменой позиций."""

    before = {key: _point(x, y) for key, (x, y) in raw.items()}
    final = {**before, **final_overrides}
    sourced = type("Sourced", (), {"positions": final})()
    settled, numbers = settle_emitted_faces(
        polygons, before, sourced, _chart(raw), factories.budget(), reverse
    )
    return settled, numbers


def test_off_plane_distance_is_the_exit_of_a_moved_vertex_from_the_face_plane_for_any_length():
    quad = (_point(0.0), _point(1.0), _point(1.0, 1.0), _point(0.0, 1.0))
    lifted = quad[:3] + (_point(0.0, 1.0, 0.0125),)
    assert off_plane_distance(quad, lifted) == pytest.approx(0.0125)
    # Начало цикла и обход меры не меняют.
    assert off_plane_distance(quad[2:] + quad[:2], lifted[2:] + lifted[:2]) == pytest.approx(0.0125)
    assert off_plane_distance(quad[::-1], lifted[::-1]) == pytest.approx(0.0125)
    hexagon = tuple(_point(x, y) for x, y in ((0, 0), (4, 0), (6, 2), (5, 5), (1, 5), (-1, 2)))
    assert off_plane_distance(hexagon, hexagon) == 0.0
    moved = hexagon[:3] + (_point(5, 5, 0.01),) + hexagon[4:]
    assert off_plane_distance(hexagon, moved) == pytest.approx(0.01)
    # Подвижка в плоскости грани плоскость не покидает.
    inside = hexagon[:3] + (_point(5.01, 5.02),) + hexagon[4:]
    assert off_plane_distance(hexagon, inside) == pytest.approx(0.0, abs=1e-12)
    # Грань нулевой площади до сдвига ничего не меряет.
    flat = tuple(_point(i) for i in range(4))
    assert off_plane_distance(flat, flat[:3] + (_point(3.0, 0.0, 1.0),)) == 0.0


def test_a_thin_face_does_not_inflate_the_measure_it_is_the_move_that_is_measured():
    """Полоса 1 м x 1 мм: плоскость «остальных вершин» у неё шумит, а уход вершины — нет."""

    strip = tuple(
        _point(x, y) for x, y in ((0, 0), (0.25, 0), (0.5, 0), (1, 0), (1, 0.001), (0, 0.001))
    )
    moved = strip[:3] + (_point(1, 0, 0.0001),) + strip[4:]
    assert off_plane_distance(strip, moved) == pytest.approx(0.0001)


def test_a_face_of_any_length_reports_its_off_plane_deviation_and_stays_whole():
    keys = tuple(L_SHAPE)
    settled, numbers = _settle(
        L_SHAPE, {"src:2": _point(4.01, 2.02, 0.0125)}, [(keys,)]
    )

    assert settled == [(keys,)]
    assert (numbers.triangulated, numbers.flipped_triangles) == (0, 0)
    assert numbers.max_off_plane == pytest.approx(0.0125)
    assert dict(numbers.counters())[FACES_OFF_PLANE] == round(numbers.max_off_plane * 10**9) > 0
    assert dict(numbers.counters())[FACES_TRIANGULATED_AFTER_LIFT] == 0
    assert dict(numbers.counters())[TRIANGLES_FLIPPED_BY_LIFT] == 0


def test_a_face_without_a_moved_vertex_is_left_alone_and_costs_nothing():
    keys = tuple(L_SHAPE)
    polygons = [(keys,), (("node:0", "node:1", "src:2"),)]
    settled, numbers = _settle(L_SHAPE, {}, polygons)

    assert settled is polygons
    assert (numbers.max_off_plane, numbers.triangulated, numbers.flipped_triangles) == (0.0, 0, 0)


@pytest.mark.parametrize("reverse", (False, True))
def test_a_face_whose_ear_turns_over_is_cut_into_its_ears_and_the_weld_stays(reverse):
    """Ухо квадрата `(b, c, d)` перевёрнуто подвижкой `c`; грань режется на `n - 2` уха, позиции те же."""

    keys = tuple(SQUARE) if not reverse else ("a", "d", "c", "b")
    moved = {"c": _point(-2.0, -2.0)}
    settled, numbers = _settle(SQUARE, moved, [(keys,)], reverse=reverse)

    (triangles,) = settled
    assert len(triangles) == 2 and all(len(item) == 3 for item in triangles)
    assert numbers.triangulated == 1
    assert dict(numbers.counters())[FACES_TRIANGULATED_AFTER_LIFT] == 1
    # Обход ушей — обход грани: против часовой на карте, а при `reverse` — по часовой.
    chart = _chart(SQUARE)
    after = {key: _point(x, y) for key, (x, y) in SQUARE.items()} | moved
    turned = 0
    for triangle in triangles:
        sign = _chart_sign(chart[key] for key in triangle)
        assert sign == (-1 if reverse else 1), triangle
        # Ухо, у которого знак 3D не тот, что на карте, перевёрнуто: ровно их и считает закон.
        turned += int(_area_z(*(after[key] for key in triangle)) * sign < 0.0)
    assert turned >= 1
    assert numbers.flipped_triangles == turned
    assert dict(numbers.counters())[TRIANGLES_FLIPPED_BY_LIFT] == turned
    # Все вершины грани остались в ушах, новых нет.
    assert {key for triangle in triangles for key in triangle} == set(keys)
    assert numbers.max_off_plane == 0.0


def test_an_emitted_triangle_that_loses_its_orientation_is_counted_not_silent():
    raw = {"node:0": (0, 0), "node:1": (4, 0), "src:2": (2, 3)}
    triangle = tuple(raw)
    settled, numbers = _settle(raw, {"src:2": _point(2.0, -1.0)}, [(triangle,)])

    assert settled == [(triangle,)]
    assert (numbers.triangulated, numbers.flipped_triangles) == (0, 1)


def test_a_straight_vertex_and_a_concave_corner_do_not_turn_a_whole_face_into_triangles():
    """Невыпуклая грань с прямой вершиной и подвижкой в плоскости остаётся одной гранью."""

    raw = {**L_SHAPE, "node:6": (1, 0)}
    keys = ("node:0", "node:6", "node:1", "src:2", "node:3", "node:4", "node:5")
    settled, numbers = _settle(raw, {"src:2": _point(4.0, 2.01)}, [(keys,)])

    assert settled == [(keys,)]
    assert numbers.triangulated == 0 and numbers.flipped_triangles == 0


# --------------------------------------------------------------------------
# Полный путь настоящего домена
# --------------------------------------------------------------------------

CASES = ("weighted", "point_contact", "full_selection")
FIELD = {
    "weighted": "building_002_weighted_normals_v1",
    "point_contact": "building_002_point_contact_v1",
    "full_selection": "building_002_full_selection_v1",
}


def _domain(name):
    return factories.field_domain(FIELD[name])


def _materialize(name, law=DecalTopologyLawV1.TRIANGLES_V1):
    prepared, coverage, request = _domain(name)
    result = materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=law,
    )
    return prepared, result


@pytest.mark.parametrize("name", CASES)
def test_every_source_vertex_of_a_field_domain_is_lifted_or_named(name):
    prepared, result = _materialize(name)
    assert result.is_materialized, result.detail
    host = {
        item.vertex_id.value: item.position for item in prepared.context.snapshot.source_vertices
    }
    counters = dict(result.counters)
    sources = [item for item in result.batch.vertices if item.vert_key.value.startswith("src:")]
    at_host = [
        item
        for item in sources
        if _bits(item.position) == _bits(host[item.vert_key.value[len("src:"):]])
    ]

    assert sources
    # Каждая вершина `src:` учтена ровно одним счётом, и в позиции хоста стоят ровно положенные.
    assert (
        counters[LIFTED]
        + counters[DISPLACED]
        + counters[UNAVAILABLE]
        + counters[ORIENTATION_KEPT]
        == len(sources)
    )
    assert len(at_host) == counters[LIFTED] > 0
    named = {item.outcome for item in result.batch.diagnostics}
    assert NamedOutcome.SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1 in named


def test_without_host_positions_the_batch_is_the_lattice_lift(monkeypatch):
    """Отрицательный контроль: закон без позиций хоста — нуль действий, батч прежний."""

    _prepared, lifted = _materialize("weighted")
    monkeypatch.setattr(domain, "host_positions_of", lambda snapshot: {})
    _prepared, plain = _materialize("weighted")

    assert plain.is_materialized
    assert dict(plain.counters)[LIFTED] == 0
    assert dict(plain.counters)[UNAVAILABLE] > 0
    assert plain.batch.vertices != lifted.batch.vertices
    assert plain.batch.semantic_digest != lifted.batch.semantic_digest
    assert not any(
        item.outcome is NamedOutcome.SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1
        for item in plain.batch.diagnostics
    )
    # Грани, UV и станции законом не тронуты: он двигает только позиции.
    assert plain.batch.faces == lifted.batch.faces
    assert plain.batch.station_facts == lifted.batch.station_facts
    moved = {
        item.vert_key.value
        for item in lifted.batch.vertices
        if item not in plain.batch.vertices
    }
    assert moved and all(key.startswith("src:") for key in moved)


@pytest.mark.parametrize("name", CASES)
def test_the_positions_agree_across_the_topology_laws(name):
    """Позиции (и дайджест смысла) — побитово одни у `TRIANGLES_V1`, `QUAD_STRIPS_V1` и `PLANAR_POLYGONS_V1`."""

    prepared, triangles = _materialize(name)
    cell = float(source_step_of(prepared.context.frame))
    results = [
        _materialize(name, law)[1]
        for law in (
            DecalTopologyLawV1.QUAD_STRIPS_V1,
            DecalTopologyLawV1.PLANAR_POLYGONS_V1,
        )
    ]

    own = dict(triangles.counters)
    assert own[FACES_OFF_PLANE] == 0 and own[FACES_TRIANGULATED_AFTER_LIFT] == 0
    assert own[TRIANGLES_FLIPPED_BY_LIFT] == 0
    for other in results:
        assert other.is_materialized, other.detail
        assert triangles.batch.vertices == other.batch.vertices
        assert triangles.batch.semantic_digest == other.batch.semantic_digest
        assert triangles.vertex_normals == other.vertex_normals
        counters = dict(other.counters)
        assert own[LIFTED] == counters[LIFTED] > 0
        assert own[ORIENTATION_KEPT] == counters[ORIENTATION_KEPT]
        assert own[DISPLACED] == counters[DISPLACED]
        assert counters[TRIANGLES_FLIPPED_BY_LIFT] == 0, name
        # Грань от четырёх вершин с подвинутой вершиной — запись закона: плоскость до ячейки.
        assert counters[FACES_OFF_PLANE] * 1e-9 <= SOURCE_VERTEX_LIFT_BUDGET_CELLS * cell, name


@pytest.mark.parametrize("name", CASES)
def test_the_law_is_fed_the_same_canonical_triangles_under_every_topology_law(name, monkeypatch):
    """Вход закона положения — канонические треугольники слитых граней, а не грани закона топологии."""

    seen = {}
    real = domain.lift_source_vertices

    def spy(positions, triangles, host_positions, step):
        triangles = [tuple(item) for item in triangles]
        seen[current[0]] = triangles
        return real(positions, triangles, host_positions, step)

    monkeypatch.setattr(domain, "lift_source_vertices", spy)
    current = [None]
    for law in DecalTopologyLawV1:
        current[0] = law
        _materialize(name, law)

    assert set(seen) == set(DecalTopologyLawV1)
    assert all(len(item) == 3 for item in seen[DecalTopologyLawV1.TRIANGLES_V1])
    reference = seen[DecalTopologyLawV1.TRIANGLES_V1]
    for law, triangles in seen.items():
        assert triangles == reference, law


#: Дайджесты малых случаев ДО закона (`test_materialize_domain.GOLDEN` на 89a7d89): с выключенным
#: законом батч обязан совпасть с ними побитово, то есть закон — единственное, что их сдвинуло.
PRE_LIFT_GOLDEN = {
    "weighted": (
        "a4360476d7143bb595dc2cfc839d394c270a03a57c05faf3173c0d35fd66175d",
        "6c9595622c6553df029c39e7de416eb99668d03507cc359ae209fbbdffba3fcb",
    ),
    "point_contact": (
        "13c9760ddaf4789159392c304703f648a4b48e25fdc4290cf2b0521ff36940be",
        "1df330928f921ffa0274e7345bafe3fedecb5cf12b745a191c503cebc026f90e",
    ),
    "two_edge": (
        "90759ea94fed3a6102a32fa6bda16a85b83da95dab4ff0c3b0c19a6b555580a0",
        "15c1a2b06d84c9a9030ee6e79e189360b79730601183583e35d7d3b0b9c54840",
    ),
    "straight3": (
        "2e9294e7de68f084672b0095c594fcb0da251bc407d5bc3feb1cc13e2f8f43a7",
        "f9988c58176be1d6d0dacdc12aaa86efcfc12116c7d8db26aaf3405e0760e7c1",
    ),
}


@pytest.mark.parametrize("name", sorted(PRE_LIFT_GOLDEN))
def test_with_the_law_off_the_batches_keep_their_pre_lift_digests(name, monkeypatch):
    from test_materialize_domain import _run

    monkeypatch.setattr(domain, "host_positions_of", lambda snapshot: {})
    result = _run(name)

    assert result.is_materialized, result.detail
    assert (result.batch.semantic_digest.value, result.content_digest) == PRE_LIFT_GOLDEN[name]
