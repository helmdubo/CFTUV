"""Закон топологии `PLANAR_POLYGONS_V1`: перекладины слитых пробегов и простые многоугольники лент.

Закон меняет СБОРКУ граней и больше ничего: вершины, их ключи, UV, станции,
цепи, диагностики, семантический дайджест и дайджест нормалей смещения
побитово те же, что у `TRIANGLES_V1` и `QUAD_STRIPS_V1`; число треугольников как
сумма `n - 2` по граням — то же. Разложение граней закона в треугольники НЕ
даёт прежнее мультимножество (слитый пробег режется по перекладинам, а не
триангулируется целиком), поэтому ворота здесь другие: точное замыкание
площади каждой грани, точная простота контура и аффинность UV (допуск
`PLANAR_AFFINE_UV_POLYGON_V1`: тогда любая триангуляция показа даёт ту же
поверхность и ту же UV-интерполяцию), допустимая триангуляция каждого
многоугольника по точным точкам, плоскостность в 3D, сумма `n - 2`.
"""

from __future__ import annotations

import dataclasses
import pickle
from collections import Counter
from fractions import Fraction
from functools import lru_cache
from types import SimpleNamespace

import pytest

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.materialize import domain
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.assemble import (
    CURVED_STRIP_FACES_TRIANGULATED,
    FAN_FACES_CUT_BY_NEIGHBOUR,
    FAN_FACES_NOT_STAR_FROM_APEX,
    FAN_FACES_TRIANGULATED_FROM_APEX,
    FAN_POLYGON_FACES_CONCAVE_EMITTED,
    FAN_POLYGON_FACES_EMITTED,
    MERGED_RUNS_KEPT_WHOLE,
    MERGED_RUNS_SPLIT_AT_RUNGS,
    POLYGON_FACES_CONCAVE_EMITTED,
    POLYGON_FACES_EMITTED,
    POLYGON_FACES_TRIANGULATED_NOT_SIMPLE,
    POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE,
    QUADS_REFUSED_NOT_CONVEX,
    QUADS_UV_BILINEAR,
    QUADS_UV_BILINEAR_MAX_MILLI_ALPHA,
    settle_topology,
    tessellate_faces,
)
from cftuv_envelope.materialize.audit import audit_batch
from cftuv_envelope.materialize.coalesce import (
    CoveredFaceV1,
    merge_same_chain_faces,
    point_key,
)
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.source_lift import (
    FACES_OFF_PLANE,
    FACES_TRIANGULATED_AFTER_LIFT,
    TRIANGLES_FLIPPED_BY_LIFT,
    off_plane_distance,
)
from cftuv_envelope.materialize.tessellate import (
    contour_is_simple,
    convex_polygon_ring,
    convex_quad_ring,
    counter_clockwise_ring,
    has_right_turn,
    triangulate_exact,
    uv_is_affine_in_chart,
)
from cftuv_envelope.validation import validate_geometry_batch
from cftuv_envelope.wavefront.faces import doubled_shoelace
from developable_route import materialize_developable
from wavefront_cases import named_corpus

import materialize_factories as factories
from test_decal_topology_law import (
    ALL_NAMES,
    DEVELOPABLE,
    DOMAINS,
    NORMAL,
    ON_SURFACE,
    SURFACE,
    UV,
    _both_laws,
    _near_planar_on_surface,
)

TRIANGLES = DecalTopologyLawV1.TRIANGLES_V1
QUADS = DecalTopologyLawV1.QUAD_STRIPS_V1
POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1

#: Числа закона: ровно эти имена, всегда, в том числе нулями.
LAW_NUMBERS = frozenset(
    (
        QUADS_REFUSED_NOT_CONVEX,
        "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES",
        "MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER",
        POLYGON_FACES_EMITTED,
        POLYGON_FACES_CONCAVE_EMITTED,
        POLYGON_FACES_TRIANGULATED_NOT_SIMPLE,
        POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE,
        CURVED_STRIP_FACES_TRIANGULATED,
        MERGED_RUNS_SPLIT_AT_RUNGS,
        MERGED_RUNS_KEPT_WHOLE,
        FAN_FACES_CUT_BY_NEIGHBOUR,
        FAN_POLYGON_FACES_EMITTED,
        FAN_POLYGON_FACES_CONCAVE_EMITTED,
        FAN_FACES_TRIANGULATED_FROM_APEX,
        FAN_FACES_NOT_STAR_FROM_APEX,
        # Закон `QUAD_UV_BILINEAR_V1` (перекладина угла JOIN): без угла JOIN оба нули.
        QUADS_UV_BILINEAR,
        QUADS_UV_BILINEAR_MAX_MILLI_ALPHA,
    )
)
#: Счётчики, которые считают ГРАНИ (зависят от закона, как у пары прежних законов).
FACE_COUNTERS = frozenset(
    (
        "MATERIALIZE_FACES_EMITTED",
        FACES_OFF_PLANE,
        FACES_TRIANGULATED_AFTER_LIFT,
        TRIANGLES_FLIPPED_BY_LIFT,
        "MATERIALIZE_QUADS",
        "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED",
        "MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE",
        "MATERIALIZE_TRIANGLES_UV_DEGENERATE",
        "MATERIALIZE_TRIANGLES_UV_REVERSED",
    )
)

BUDGET = factories.budget


def _points(raw):
    return tuple(
        (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))
        for x, y in raw
    )


def _area(points):
    """Положительная удвоенная площадь контура, точная."""

    total = doubled_shoelace(tuple(points))
    if total.sign(budget=BUDGET()) < 0:
        total = SqrtSumV1.zero() - total
    return total


# --------------------------------------------------------------------------
# Предикат: выпуклый контур без правых поворотов
# --------------------------------------------------------------------------

HEXAGON = ((0, 0), (4, 0), (6, 2), (5, 5), (1, 5), (-1, 2))


@pytest.mark.parametrize("clockwise", (False, True))
def test_a_convex_polygon_gives_its_counter_clockwise_ring_in_both_orientations(clockwise):
    raw = HEXAGON[::-1] if clockwise else HEXAGON
    ring = convex_polygon_ring(_points(raw), BUDGET())
    assert ring == (tuple(range(6)) if not clockwise else (5, 4, 3, 2, 1, 0))


def test_a_vertex_on_a_straight_side_keeps_the_polygon_convex():
    """Вершина на прямой несёт T-стык соседа: контур допустим, а не «не строго выпуклый»."""

    rectangle = ((0, 0), (2, 0), (4, 0), (4, 3), (0, 3))
    assert convex_quad_ring(_points(rectangle[:3] + rectangle[4:]), BUDGET()) is None
    assert convex_polygon_ring(_points(rectangle), BUDGET()) == (0, 1, 2, 3, 4)
    flat_quad = ((0, 0), (2, 0), (4, 0), (2, 3))
    assert convex_quad_ring(_points(flat_quad), BUDGET()) is None
    assert convex_polygon_ring(_points(flat_quad), BUDGET()) == (0, 1, 2, 3)


@pytest.mark.parametrize(
    "raw",
    (
        # Один правый поворот: «лента с вырезом» — то, что оставляет фронт соседа.
        ((0, 0), (6, 0), (6, 3), (4, 3), (3, 1), (2, 3), (0, 3)),
        ((0, 0), (4, 0), (4, 4), (2, 1), (0, 4)),
        # Нулевая площадь.
        ((0, 0), (1, 0), (2, 0), (3, 0), (4, 0)),
        # Не больше трёх точек: у закона для них своя дорога.
        ((0, 0), (4, 0), (4, 3)),
    ),
)
def test_a_contour_with_a_right_turn_or_no_area_is_not_a_convex_polygon(raw):
    assert convex_polygon_ring(_points(raw), BUDGET()) is None


def test_a_strictly_convex_quad_has_the_same_ring_under_both_predicates():
    ring = ((0, 0), (4, 0), (5, 3), (-1, 2))
    assert convex_polygon_ring(_points(ring), BUDGET()) == convex_quad_ring(
        _points(ring), BUDGET()
    )


# --------------------------------------------------------------------------
# Предикаты допуска: простота контура и аффинность UV
# --------------------------------------------------------------------------


def _affine_map(points):
    """Точные `(s, r) = (x + 2y, 3x - y)` по вершинам: аффинная функция положения на карте."""

    return [
        (x + y.scaled(Fraction(2)), x.scaled(Fraction(3)) - y) for x, y in points
    ]


NOTCHED = ((0, 0), (6, 0), (6, 3), (4, 3), (3, 1), (2, 3), (0, 3))
SELF_CROSSING = ((0, 0), (4, 0), (0, 4), (4, 4), (2, 6))
#: Пентаграмма: пять левых поворотов подряд (`has_right_turn` ложен), но контур пересекает сам себя.
PENTAGRAM = ((0, 0), (5, 3), (-1, 3), (4, 0), (2, 5))


def test_counter_clockwise_ring_follows_the_sign_of_the_area_and_a_zero_area_has_none():
    assert counter_clockwise_ring(_points(NOTCHED), BUDGET()) == tuple(range(7))
    assert counter_clockwise_ring(_points(NOTCHED[::-1]), BUDGET()) == tuple(range(6, -1, -1))
    assert counter_clockwise_ring(_points(((0, 0), (1, 0), (2, 0), (3, 0))), BUDGET()) is None


def test_a_right_turn_is_named_on_a_notched_ring_and_absent_on_a_convex_one():
    notched = _points(NOTCHED)
    convex = _points(HEXAGON)
    assert has_right_turn(notched, counter_clockwise_ring(notched, BUDGET()), BUDGET())
    assert not has_right_turn(convex, counter_clockwise_ring(convex, BUDGET()), BUDGET())


@pytest.mark.parametrize("clockwise", (False, True))
def test_a_notched_contour_is_simple_and_a_self_crossing_one_is_not(clockwise):
    notched = NOTCHED[::-1] if clockwise else NOTCHED
    assert contour_is_simple(_points(notched), BUDGET())
    assert not contour_is_simple(_points(SELF_CROSSING), BUDGET())
    assert not contour_is_simple(_points(((0, 0), (1, 0), (2, 0), (3, 0))), BUDGET())


def test_an_affine_uv_is_accepted_by_any_choice_of_the_three_base_vertices():
    # Первые три вершины на одной прямой: база берётся из следующих неколлинеарных.
    raw = ((0, 0), (2, 0), (4, 0), (4, 3), (0, 3))
    points = _points(raw)
    values = _affine_map(points)
    assert uv_is_affine_in_chart(points, values, BUDGET())
    assert uv_is_affine_in_chart(points[::-1], values[::-1], BUDGET())
    assert uv_is_affine_in_chart(_points(NOTCHED), _affine_map(_points(NOTCHED)), BUDGET())


@pytest.mark.parametrize("component", (0, 1))
@pytest.mark.parametrize("index", (0, 2, 3, 4))
def test_one_vertex_off_the_affine_map_makes_the_uv_not_affine(index, component):
    points = _points(((0, 0), (2, 0), (4, 0), (4, 3), (0, 3)))
    values = _affine_map(points)
    broken = list(values)
    shifted = list(broken[index])
    shifted[component] = shifted[component] + SqrtSumV1.rational(Fraction(1, 1000000))
    broken[index] = tuple(shifted)
    assert not uv_is_affine_in_chart(points, broken, BUDGET())


def test_the_affine_check_is_exact_on_irrational_chart_coordinates():
    root = SqrtSumV1.radical(1, Fraction(2), BUDGET())
    zero = SqrtSumV1.zero()
    half = Fraction(1, 2)
    points = (
        (zero, zero),
        (root, zero),
        (root, root),
        (root.scaled(half), root),
        (zero, root),
    )
    values = _affine_map(points)
    assert uv_is_affine_in_chart(points, values, BUDGET())
    values[3] = (values[3][0] + root.scaled(Fraction(1, 1000)), values[3][1])
    assert not uv_is_affine_in_chart(points, values, BUDGET())


def test_points_on_one_line_are_not_an_affine_map():
    points = _points(((0, 0), (1, 0), (2, 0), (3, 0)))
    assert not uv_is_affine_in_chart(points, _affine_map(points), BUDGET())


# --------------------------------------------------------------------------
# tessellate_faces под законом
# --------------------------------------------------------------------------


def _uv(*cycles):
    """`uv_values` закона: аффинные `(s, r)` по ключам вершин (тесселяция читает только их)."""

    values = {}
    for cycle in cycles:
        for (key, _point), value in zip(cycle, _affine_map([point for _key, point in cycle])):
            values[key] = value
    return lambda frame_face, key: values[key]


def _frame(points, *, fan=False, parts=()):
    keys = tuple(f"k{index}" for index in range(len(points)))
    face = SimpleNamespace(
        owner=(0, 0, 1, 1, 1) if fan else (0, 0, 1, 1),
        doubled_area=_area(points),
        parts=parts,
    )
    return SimpleNamespace(is_fan=fan, face=face), tuple(zip(keys, points))


def _shape(points, *, reverse=False, exact_plane=True, fan=False, uv=None):
    frame, cycle = _frame(points, fan=fan)
    tally = Counter()
    polygons = tessellate_faces(
        [frame],
        [cycle],
        BUDGET(),
        reverse,
        POLYGONS,
        exact_plane,
        tally,
        uv if uv is not None else _uv(cycle),
    )
    return polygons[0], tally, cycle


def _exact_triangulation_is_valid(points):
    """Допустимая триангуляция по точным точкам: `n - 2` невырожденных треугольников, площадь замкнута."""

    triangles = triangulate_exact(tuple(points), BUDGET())
    assert triangles is not None and len(triangles) == len(points) - 2
    total = SqrtSumV1.zero()
    for first, second, third in triangles:
        part = doubled_shoelace((points[first], points[second], points[third]))
        assert part.sign(budget=BUDGET()) > 0
        total = total + part
    assert (total - _area(points)).is_zero


@pytest.mark.parametrize("reverse", (False, True))
@pytest.mark.parametrize("clockwise", (False, True))
def test_a_convex_hexagon_strip_is_one_face_on_an_exact_plane(clockwise, reverse):
    points = _points(HEXAGON[::-1] if clockwise else HEXAGON)
    polygons, tally, cycle = _shape(points, reverse=reverse)
    assert [len(item) for item in polygons] == [6]
    assert tally[POLYGON_FACES_EMITTED] == 1
    assert not tally[POLYGON_FACES_CONCAVE_EMITTED]
    keys = tuple(key for key, _point in cycle)
    ring = keys if not clockwise else keys[::-1]
    # Первая вершина сохранена, обход — против часовой либо (при `reverse`) обратный.
    assert polygons[0] == ((ring[0], *reversed(ring[1:])) if reverse else ring)
    _exact_triangulation_is_valid(points)


def test_a_four_point_strip_with_a_straight_vertex_stays_one_face():
    """Строгая выпуклость нужна была четырёхграннику ради `fan_out`; многоугольнику она не нужна."""

    points = _points(((0, 0), (2, 0), (4, 0), (2, 3)))
    polygons, tally, _cycle = _shape(points)
    assert [len(item) for item in polygons] == [4]
    assert not tally[QUADS_REFUSED_NOT_CONVEX]
    quads = tessellate_faces(
        [_frame(points)[0]], [_frame(points)[1]], BUDGET(), False, QUADS
    )
    assert [len(item) for item in quads[0]] == [3, 3]
    _exact_triangulation_is_valid(points)


@pytest.mark.parametrize("reverse", (False, True))
@pytest.mark.parametrize("clockwise", (False, True))
def test_a_notched_strip_is_one_concave_face_when_it_is_simple_and_its_uv_is_affine(
    clockwise, reverse
):
    points = _points(NOTCHED[::-1] if clockwise else NOTCHED)
    polygons, tally, cycle = _shape(points, reverse=reverse)
    assert [len(item) for item in polygons] == [7]
    assert tally[POLYGON_FACES_EMITTED] == 1 and tally[POLYGON_FACES_CONCAVE_EMITTED] == 1
    assert not tally[POLYGON_FACES_TRIANGULATED_NOT_SIMPLE]
    assert not tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE]
    keys = tuple(key for key, _point in cycle)
    ring = keys if not clockwise else keys[::-1]
    assert polygons[0] == ((ring[0], *reversed(ring[1:])) if reverse else ring)
    _exact_triangulation_is_valid(points)


def test_a_concave_quad_on_an_exact_plane_is_one_face_too():
    dart = _points(((0, 0), (4, 0), (1, 1), (0, 4)))
    polygons, tally, _cycle = _shape(dart)
    assert [len(item) for item in polygons] == [4]
    assert tally[POLYGON_FACES_CONCAVE_EMITTED] == 1 and not tally[POLYGON_FACES_EMITTED]
    frame, cycle = _frame(dart)
    quads = tessellate_faces([frame], [cycle], BUDGET(), False, QUADS)
    assert [len(item) for item in quads[0]] == [3, 3]


@pytest.mark.parametrize("raw", (NOTCHED, HEXAGON))
def test_a_non_affine_uv_makes_the_strip_ear_clipped_and_named(raw):
    points = _points(raw)
    frame, cycle = _frame(points)
    clean = _uv(cycle)

    def bent(face, key):
        s, r = clean(face, key)
        return (s + SqrtSumV1.rational(Fraction(1)), r) if key == "k3" else (s, r)

    polygons, tally, _cycle = _shape(points, uv=bent)
    assert [len(item) for item in polygons] == [3] * (len(points) - 2)
    assert tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE] == 1
    assert not tally[POLYGON_FACES_EMITTED] and not tally[POLYGON_FACES_CONCAVE_EMITTED]


@pytest.mark.parametrize("flow_key", (None, "flow:u0"))
def test_a_non_affine_convex_quad_stays_whole_only_in_a_flow(flow_key):
    """`QUAD_UV_BILINEAR_V1` — закон ПОТОКА: вне потока неаффинное четырёхгранье режется, как раньше."""

    points = _points(((0, 0), (4, 0), (5, 3), (-1, 3)))
    frame, cycle = _frame(points)
    frame.flow_key = flow_key
    clean = _uv(cycle)

    def bent(face, key):
        s, r = clean(face, key)
        return (s + SqrtSumV1.rational(Fraction(1)), r) if key == "k3" else (s, r)

    tally = Counter()
    polygons = tessellate_faces(
        [frame], [cycle], BUDGET(), False, POLYGONS, True, tally, bent
    )[0]
    if flow_key is None:
        assert [len(item) for item in polygons] == [3, 3]
        assert tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE] == 1
        assert not tally[QUADS_UV_BILINEAR]
    else:
        assert [len(item) for item in polygons] == [4]
        assert tally[QUADS_UV_BILINEAR] == 1
        assert not tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE]


@pytest.mark.parametrize("raw", (SELF_CROSSING, PENTAGRAM))
def test_a_self_crossing_contour_is_never_emitted_whole_even_with_no_right_turn(raw):
    """Сплошные левые повороты не доказывают простоту: пентаграмма раньше шла целой гранью."""

    points = _points(raw)
    ring = counter_clockwise_ring(points, BUDGET())
    assert has_right_turn(points, ring, BUDGET()) == (raw is SELF_CROSSING)
    frame, cycle = _frame(points)
    tally = Counter()
    with pytest.raises(MaterializationRefusal) as refusal:
        tessellate_faces(
            [frame], [cycle], BUDGET(), False, POLYGONS, True, tally, _uv(cycle)
        )
    assert refusal.value.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    # Грани, которой нет в меше, счётчик «разрезана на треугольники» не помнит.
    assert not tally[POLYGON_FACES_TRIANGULATED_NOT_SIMPLE]
    assert not tally[POLYGON_FACES_EMITTED]


def test_a_contour_whose_vertex_repeats_is_not_one_face_and_is_counted_only_as_triangles():
    """Перетяжка (одна вершина дважды) трансверсальных пересечений не даёт, но и многоугольником не станет."""

    points = _points(HEXAGON)
    frame, cycle = _frame(points)
    pinched = tuple(
        ("k0" if index == 3 else key, point) for index, (key, point) in enumerate(cycle)
    )
    assert contour_is_simple(points, BUDGET())
    tally = Counter()
    polygons = tessellate_faces(
        [frame], [pinched], BUDGET(), False, POLYGONS, True, tally, _uv(cycle)
    )
    assert all(len(item) == 3 for item in polygons[0]) and len(polygons[0]) == 4
    assert tally[POLYGON_FACES_TRIANGULATED_NOT_SIMPLE] == 1
    assert not tally[POLYGON_FACES_EMITTED] and not tally[POLYGON_FACES_CONCAVE_EMITTED]


def test_a_zero_area_contour_is_a_refusal_and_no_counter_claims_triangles_for_it():
    flat = _points(((0, 0), (1, 0), (2, 0), (3, 0)))
    frame, cycle = _frame(flat)
    frame.face.doubled_area = SqrtSumV1.zero()
    tally = Counter()
    with pytest.raises(MaterializationRefusal) as refusal:
        tessellate_faces(
            [frame], [cycle], BUDGET(), False, POLYGONS, True, tally, _uv(cycle)
        )
    assert refusal.value.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    assert not +tally


def test_on_a_source_triangle_lift_only_a_strictly_convex_quad_stays_whole():
    hexagon = _points(HEXAGON)
    polygons, tally, _cycle = _shape(hexagon, exact_plane=False)
    assert [len(item) for item in polygons] == [3] * 4
    assert tally[CURVED_STRIP_FACES_TRIANGULATED] == 1
    assert not tally[POLYGON_FACES_EMITTED]

    quad = _points(((0, 0), (4, 0), (5, 3), (-1, 2)))
    polygons, tally, _cycle = _shape(quad, exact_plane=False)
    assert [len(item) for item in polygons] == [4]

    flat = _points(((0, 0), (2, 0), (4, 0), (2, 3)))
    polygons, tally, _cycle = _shape(flat, exact_plane=False)
    assert [len(item) for item in polygons] == [3, 3]
    assert tally[QUADS_REFUSED_NOT_CONVEX] == 1


def test_an_area_that_does_not_close_is_a_named_refusal_for_a_polygon():
    points = _points(HEXAGON)
    frame, cycle = _frame(points)
    frame.face.doubled_area = frame.face.doubled_area + SqrtSumV1.rational(Fraction(1))
    with pytest.raises(MaterializationRefusal) as refusal:
        tessellate_faces(
            [frame], [cycle], BUDGET(), False, POLYGONS, True, None, _uv(cycle)
        )
    assert refusal.value.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    assert "polygon areas differ" in refusal.value.detail


# --------------------------------------------------------------------------
# Перекладины слитого пробега
# --------------------------------------------------------------------------

CHAIN = "physical-chain:one"


def _covered(owner, raw, chain=CHAIN):
    contour = tuple(
        (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))
        for x, y in raw
    )
    return CoveredFaceV1(
        region_id="region",
        owner=owner,
        envelope_spec_id="",
        envelope_instance_id=None,
        points=contour,
        doubled_area=_area(contour),
        source_chain_id=chain,
    )


def _run():
    """Две грани одной цепи на одной прямой, слитые в один шестиугольник-пробег."""

    left = _covered((0, 0, 2, 0), ((0, 0), (2, 0), (2, 1), (0, 1)))
    right = _covered((2, 0, 4, 0), ((2, 0), (4, 0), (4, 1), (2, 1)))
    merged, stats = merge_same_chain_faces((left, right))
    assert len(merged) == 1 and stats.merged_separators == 1
    return left, right, merged[0]


def _merged_frame(merged, *, drop=None):
    cycle = tuple((f"k{index}", point) for index, point in enumerate(merged.points))
    if drop is not None:
        cycle = tuple(item for item in cycle if point_key(item[1]) != point_key(drop))
    return SimpleNamespace(is_fan=False, face=merged), cycle


def test_the_merge_keeps_its_parts_and_they_are_no_part_of_a_faces_identity():
    left, right, merged = _run()
    assert merged.parts == (left, right)
    assert merged.merged_owners == ((2, 0, 4, 0),)
    assert left.parts == ()
    assert dataclasses.replace(merged, parts=()) == merged


def test_a_merged_run_is_cut_back_into_the_faces_of_its_source_edges():
    left, right, merged = _run()
    frame, cycle = _merged_frame(merged)
    tally = Counter()
    polygons = tessellate_faces(
        [frame], [cycle], BUDGET(), False, POLYGONS, True, tally, _uv(cycle)
    )[0]
    assert [len(item) for item in polygons] == [4, 4]
    assert tally[MERGED_RUNS_SPLIT_AT_RUNGS] == 1 and not tally[MERGED_RUNS_KEPT_WHOLE]
    key = {point_key(point): name for name, point in cycle}
    expected = [
        {key[point_key(point)] for point in part.points} for part in (left, right)
    ]
    assert [set(item) for item in polygons] == expected
    # Перекладина — общее ребро в ПРОТИВОПОЛОЖНЫХ направлениях: внутреннее, не граница.
    edges = [
        {(item[index], item[(index + 1) % 4]) for index in range(4)} for item in polygons
    ]
    shared = {(a, b) for a, b in edges[0] if (b, a) in edges[1]}
    assert len(shared) == 1
    # Без частей тот же контур — один шестиугольник (вершины на прямой — в нём).
    plain, plain_cycle = _frame(merged.points)
    whole = tessellate_faces(
        [plain], [plain_cycle], BUDGET(), False, POLYGONS, True, None, _uv(plain_cycle)
    )[0]
    assert [len(item) for item in whole] == [6]


def test_a_run_whose_rung_holds_a_vertex_the_merged_contour_lacks_stays_whole_and_is_named():
    _left, right, merged = _run()
    frame, cycle = _merged_frame(merged, drop=right.points[3])
    tally = Counter()
    polygons = tessellate_faces(
        [frame], [cycle], BUDGET(), False, POLYGONS, True, tally, _uv(cycle)
    )[0]
    assert tally[MERGED_RUNS_KEPT_WHOLE] == 1 and not tally[MERGED_RUNS_SPLIT_AT_RUNGS]
    assert [len(item) for item in polygons] == [len(cycle)]


def test_the_parts_of_a_run_must_add_up_to_its_area():
    left, right, merged = _run()
    broken = dataclasses.replace(
        merged, doubled_area=merged.doubled_area + SqrtSumV1.rational(Fraction(1))
    )
    frame, cycle = _merged_frame(broken)
    with pytest.raises(MaterializationRefusal) as refusal:
        tessellate_faces(
            [frame], [cycle], BUDGET(), False, POLYGONS, True, None, _uv(cycle)
        )
    assert refusal.value.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    assert "run part areas differ" in refusal.value.detail


# --------------------------------------------------------------------------
# settle_topology под законом
# --------------------------------------------------------------------------


def test_the_law_names_its_numbers_even_when_they_are_zero():
    polygons, tally, cycle = _shape(_points(HEXAGON))
    sources = {key: (None, None) for key, _point in cycle}
    settled, numbers = settle_topology([None], [None], [polygons], sources, POLYGONS, tally)
    assert [len(item) for item in settled[0]] == [6]
    assert {name for name, _value in numbers} == LAW_NUMBERS
    assert dict(numbers)[POLYGON_FACES_EMITTED] == 1


def test_a_polygon_over_five_vertices_that_is_not_planar_in_3d_is_refused_not_split():
    polygons = [(("a", "b", "c", "d", "e"),)]
    sources = {key: ("t7", None) for key in "abcd"} | {"e": ("t8", None)}
    with pytest.raises(MaterializationRefusal) as refusal:
        settle_topology([None], [None], polygons, sources, POLYGONS, Counter())
    assert refusal.value.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    assert "POLYGON_NOT_PLANAR" in refusal.value.detail


def test_the_older_laws_keep_their_own_numbers_untouched():
    """Закон лент не получил ни одного нового имени: его ответ побитово прежний."""

    frame, cycle = _frame(_points(HEXAGON))
    for law in (TRIANGLES, QUADS):
        polygons = tessellate_faces([frame], [cycle], BUDGET(), False, law)
        sources = {key: (None, None) for key, _point in cycle}
        _settled, numbers = settle_topology([frame], [cycle], polygons, sources, law)
        assert [name for name, _value in numbers] == [
            "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX",
            "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES",
            "MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER",
            "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED",
        ]


# --------------------------------------------------------------------------
# Ворота на материализаторе: те же домены, три закона
# --------------------------------------------------------------------------


@lru_cache(maxsize=None)
def _polygons_law(name):
    if name in DEVELOPABLE:
        make, route, alpha = DEVELOPABLE[name]
        return materialize_developable(
            make(), route, alpha=alpha, decal_topology_law=POLYGONS
        )[0]
    if name in SURFACE:
        prepared, coverage, request = _near_planar_on_surface(*SURFACE[name])
        extra = {"near_planar_lift_law": ON_SURFACE}
    else:
        prepared, coverage, request = DOMAINS[name]()
        extra = {}
    return materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=POLYGONS,
        **extra,
    )


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_polygon_law_materializes_a_valid_audited_batch(name):
    result = _polygons_law(name)
    assert result.is_materialized, (name, result.detail)
    assert result.decal_topology_law is POLYGONS
    assert validate_geometry_batch(result.batch) == ()
    assert audit_batch(result.batch, NORMAL).problems() == ()


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_polygon_law_changes_no_vertex_no_uv_no_chain_and_no_digest_of_meaning(name):
    triangles, quads = _both_laws(name)
    result = _polygons_law(name)
    for other in (triangles, quads):
        left, right = other.batch, result.batch
        assert left.vertices == right.vertices
        assert left.station_facts == right.station_facts
        assert left.semantic_regions == right.semantic_regions
        assert left.boundary_chains == right.boundary_chains
        assert left.interface_chains == right.interface_chains
        assert left.diagnostics == right.diagnostics
        assert left.contract_versions == right.contract_versions
        assert left.semantic_digest == right.semantic_digest
        assert other.vertex_normals == result.vertex_normals
        assert other.offset_normals_digest == result.offset_normals_digest
        assert other.diagnostics == result.diagnostics


def _uv_by_region_and_vertex(batch):
    found = {}
    for face in batch.faces:
        for fact in face.uv_facts:
            slot = (face.semantic_region_id, fact.vert_key)
            assert found.setdefault(slot, fact.uv) == fact.uv
    return found


@pytest.mark.parametrize("name", ALL_NAMES)
def test_every_vertex_keeps_its_uv_in_its_region_across_the_rungs(name):
    """Перекладина — внутреннее ребро одного региона: UV по обе её стороны одни и те же."""

    triangles, _quads = _both_laws(name)
    assert _uv_by_region_and_vertex(_polygons_law(name).batch) == _uv_by_region_and_vertex(
        triangles.batch
    )


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_counters_agree_except_the_ones_that_count_faces(name):
    triangles, quads = _both_laws(name)
    result = _polygons_law(name)
    left, right, own = dict(triangles.counters), dict(quads.counters), dict(result.counters)
    # Ключи: прежний набор без счётчика-обманщика плюс числа закона.
    assert set(own) == (set(right) - {"MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED"}) | LAW_NUMBERS
    assert set(left) == set(right)
    for key in set(left) - FACE_COUNTERS - LAW_NUMBERS:
        if key.startswith("EXACT_WORK_"):
            continue
        assert own[key] == left[key], key
    sizes = Counter(len(face.ordered_vert_keys) for face in result.batch.faces)
    assert own["MATERIALIZE_FACES_EMITTED"] == len(result.batch.faces)
    assert own["MATERIALIZE_QUADS"] == sizes[4]
    # Сумма `n - 2` не зависит ни от диагонали, ни от перекладин: то же число у всех трёх законов.
    assert own["MATERIALIZE_TRIANGLES"] == sum((size - 2) * count for size, count in sizes.items())
    assert own["MATERIALIZE_TRIANGLES"] == left["MATERIALIZE_TRIANGLES"] == right["MATERIALIZE_TRIANGLES"]
    # Многоугольники лент — `POLYGON_FACES_EMITTED`; веерные клетки считаются отдельно
    # (`FAN_POLYGON_FACES_EMITTED`) и от пяти вершин входят в число граней длиннее четырёх.
    over_four = sum(count for size, count in sizes.items() if size > 4)
    assert own[POLYGON_FACES_EMITTED] <= over_four <= own[POLYGON_FACES_EMITTED] + own[FAN_POLYGON_FACES_EMITTED]
    assert own[FAN_FACES_CUT_BY_NEIGHBOUR] == (
        own[FAN_POLYGON_FACES_EMITTED]
        + own[FAN_FACES_TRIANGULATED_FROM_APEX]
        + own[FAN_FACES_NOT_STAR_FROM_APEX]
    )
    assert own[MERGED_RUNS_KEPT_WHOLE] == 0


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_polygon_law_never_emits_more_faces_than_the_quad_law(name):
    """Закон лишь склеивает и режет по перекладинам: контур даёт одну грань либо `n - 2` треугольника, как у закона лент."""

    _triangles, quads = _both_laws(name)
    result = _polygons_law(name)
    assert len(result.batch.faces) <= len(quads.batch.faces)


def _polygons_law_unlifted(name):
    """Закон многоугольников без закона `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` (позиций хоста нет)."""

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(domain, "host_positions_of", lambda snapshot: {})
        return _polygons_law.__wrapped__(name)


@pytest.mark.parametrize("name", ALL_NAMES)
def test_no_polygon_of_the_law_is_non_planar(name):
    """Точная плоскость многоугольника — свойство ПОДЪЁМА: вершины на носителе. Сдвиг хоста — ниже."""

    result = _polygons_law_unlifted(name)
    assert dict(result.counters)[FACES_OFF_PLANE] == 0
    batch = result.batch
    position = {item.vert_key: item.position for item in batch.vertices}
    for face in batch.faces:
        count = len(face.ordered_vert_keys)
        if count < 4:
            continue
        points = [position[key] for key in face.ordered_vert_keys]
        normal = [0.0, 0.0, 0.0]
        for index, current in enumerate(points):
            following = points[(index + 1) % count]
            normal[0] += (current.y - following.y) * (current.z + following.z)
            normal[1] += (current.z - following.z) * (current.x + following.x)
            normal[2] += (current.x - following.x) * (current.y + following.y)
        length = sum(item * item for item in normal) ** 0.5
        assert length > 0.0
        origin = points[0]
        scale = max(
            max(
                ((p.x - origin.x) ** 2 + (p.y - origin.y) ** 2 + (p.z - origin.z) ** 2) ** 0.5
                for p in points
            ),
            1e-9,
        )
        for point in points:
            distance = abs(
                normal[0] * (point.x - origin.x)
                + normal[1] * (point.y - origin.y)
                + normal[2] * (point.z - origin.z)
            ) / length
            assert distance / scale < 1e-12


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_lifted_polygons_are_planar_within_the_recorded_deviation(name):
    """ОГРАНИЧЕНИЕ ЗАКОНА: после сдвига вершины на `<= 1` ячейку грань от четырёх вершин плоская
    лишь в записанных пределах, а не точно; число — наибольший уход подвинутой вершины от плоскости
    грани до сдвига, по ВСЕМ длинам граней."""

    result = _polygons_law(name)
    plain = _polygons_law_unlifted(name)
    before = {item.vert_key: item.position for item in plain.batch.vertices}
    after = {item.vert_key: item.position for item in result.batch.vertices}
    recorded = dict(result.counters)[FACES_OFF_PLANE] * 1e-9
    measured = max(
        (
            off_plane_distance(
                tuple(before[key] for key in face.ordered_vert_keys),
                tuple(after[key] for key in face.ordered_vert_keys),
            )
            for face in result.batch.faces
            if len(face.ordered_vert_keys) >= 4
        ),
        default=0.0,
    )
    # Записано наибольшее отклонение граней с подвинутой вершиной; прочие плоские точно.
    assert measured == pytest.approx(recorded, abs=1e-9), (name, measured, recorded)


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_positions_and_the_digest_of_meaning_agree_across_all_three_laws(name):
    """Позиции вершин не зависят от закона топологии: ориентация сварки идёт по каноническим
    треугольникам слитых граней, а не по веерам и частям пробегов выпущенных граней."""

    triangles, quads = _both_laws(name)
    polygons = _polygons_law(name)
    for other in (quads, polygons):
        assert other.batch.vertices == triangles.batch.vertices
        assert other.batch.semantic_digest == triangles.batch.semantic_digest
        assert other.vertex_normals == triangles.vertex_normals
        assert other.offset_normals_digest == triangles.offset_normals_digest
        left, right = dict(triangles.counters), dict(other.counters)
        for key in (
            "MATERIALIZE_SOURCE_VERTICES_LIFTED_AT_HOST",
            "MATERIALIZE_SOURCE_VERTICES_DISPLACED_BY_LATTICE",
            "MATERIALIZE_SOURCE_VERTICES_LIFT_REFUSED_BY_FACE_ORIENTATION",
        ):
            assert left[key] == right[key], key


@pytest.mark.parametrize("name", sorted(DOMAINS) + sorted(SURFACE))
def test_every_polygon_of_the_law_is_exactly_simple_with_an_affine_uv_and_a_valid_triangulation(
    name, monkeypatch
):
    """Точные ворота: по контурам СЛИТЫХ граней и точным `(s, r)`, не по float-позициям батча."""

    captured = {}
    real = domain.settle_topology
    real_tessellate = domain.tessellate_faces
    real_settle_faces = domain.settle_emitted_faces

    def spy(frame_faces, cycles, polygons, sources, law, tally=None):
        settled, numbers = real(frame_faces, cycles, polygons, sources, law, tally)
        captured["cycles"] = cycles
        return settled, numbers

    def spy_tessellate(frame_faces, cycles, budget, **kwargs):
        captured["frames"], captured["uv"] = frame_faces, kwargs["uv_values"]
        return real_tessellate(frame_faces, cycles, budget, **kwargs)

    def spy_settle_faces(polygons, *rest):
        settled, numbers = real_settle_faces(polygons, *rest)
        captured["polygons"] = settled
        return settled, numbers

    monkeypatch.setattr(domain, "settle_topology", spy)
    monkeypatch.setattr(domain, "tessellate_faces", spy_tessellate)
    monkeypatch.setattr(domain, "settle_emitted_faces", spy_settle_faces)
    if name in SURFACE:
        prepared, coverage, request = _near_planar_on_surface(*SURFACE[name])
        extra = {"near_planar_lift_law": ON_SURFACE}
    else:
        prepared, coverage, request = DOMAINS[name]()
        extra = {}
    result = materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=POLYGONS,
        **extra,
    )
    assert result.is_materialized, result.detail
    point_of = {key: point for cycle in captured["cycles"] for key, point in cycle}
    final = [polygon for face in captured["polygons"] for polygon in face]
    assert [tuple(key.value for key in face.ordered_vert_keys) for face in result.batch.faces] == final
    concave = 0
    for frame, face_polygons in zip(captured["frames"], captured["polygons"]):
        for polygon in face_polygons:
            points = tuple(point_of[key] for key in polygon)
            _exact_triangulation_is_valid(points)
            if len(points) > 4:
                assert name not in SURFACE
            if len(points) < 4:
                continue
            ring = counter_clockwise_ring(points, BUDGET())
            assert contour_is_simple(points, BUDGET())
            # Другая тройка базовых вершин, чем у закона: проверка не повторяет его выбор.
            values = [captured["uv"](frame, key) for key in polygon]
            assert uv_is_affine_in_chart(points[::-1], values[::-1], BUDGET())
            if has_right_turn(points, ring, BUDGET()):
                assert name not in SURFACE
                concave += 1
            elif name not in SURFACE:
                assert convex_polygon_ring(points, BUDGET()) is not None
    counters = dict(result.counters)
    assert concave == counters[POLYGON_FACES_CONCAVE_EMITTED] + counters[FAN_POLYGON_FACES_CONCAVE_EMITTED]


def test_a_chain_of_two_source_edges_is_two_quads_and_not_one_six_gon_or_four_triangles():
    triangles, quads = _both_laws("two_edge")
    result = _polygons_law("two_edge")
    assert [len(face.ordered_vert_keys) for face in triangles.batch.faces] == [3] * 4
    assert [len(face.ordered_vert_keys) for face in quads.batch.faces] == [3] * 4
    assert [len(face.ordered_vert_keys) for face in result.batch.faces] == [4, 4]
    counters = dict(result.counters)
    assert counters[MERGED_RUNS_SPLIT_AT_RUNGS] == 1
    # Перекладина — не шов UV: интерфейсных цепей столько же, сколько у других законов.
    assert result.batch.interface_chains == triangles.batch.interface_chains
    # Обе грани — того же региона и той же огибающей, что и слитая.
    assert len({face.semantic_region_id for face in result.batch.faces}) == 1
    assert len({face.ownership_claim_id for face in result.batch.faces}) == 1
    assert len({face.provenance for face in result.batch.faces}) == 1


def test_a_straight_chain_of_three_edges_is_three_quads():
    result = _polygons_law("straight3")
    assert [len(face.ordered_vert_keys) for face in result.batch.faces] == [4, 4, 4]
    assert dict(result.counters)[MERGED_RUNS_SPLIT_AT_RUNGS] == 1


@pytest.mark.parametrize("name", sorted(SURFACE) + sorted(DEVELOPABLE))
def test_a_domain_on_source_triangles_emits_no_polygon_over_four_vertices(name):
    result = _polygons_law(name)
    counters = dict(result.counters)
    assert counters[POLYGON_FACES_EMITTED] == 0
    assert max(len(face.ordered_vert_keys) for face in result.batch.faces) <= 4


# --------------------------------------------------------------------------
# Корпус стенда: сборка от разбиения (плоская укладка)
# --------------------------------------------------------------------------


def test_the_polygon_law_over_the_corpus_changes_only_the_assembly_of_faces():
    seen = Counter()
    for name, polygon in named_corpus():
        for alpha in (Fraction(1), Fraction(3, 2)):
            left = factories.assemble_polygon_batch(polygon, alpha, law=TRIANGLES)
            right = factories.assemble_polygon_batch(polygon, alpha, law=POLYGONS)
            assert (left is None) == (right is None), (name, alpha)
            if left is None:
                continue
            batch, _frames = right
            assert validate_geometry_batch(batch) == (), (name, alpha)
            assert audit_batch(batch, NORMAL).problems() == (), (name, alpha)
            assert batch.vertices == left[0].vertices
            assert batch.semantic_digest == left[0].semantic_digest
            assert sum(len(item.ordered_vert_keys) - 2 for item in batch.faces) == len(
                left[0].faces
            ), (name, alpha)
            assert _uv_by_region_and_vertex(batch) == _uv_by_region_and_vertex(left[0])
            seen.update(len(item.ordered_vert_keys) for item in batch.faces)
    assert seen[4] > 0


def test_a_concave_strip_of_the_corpus_is_one_face_and_changes_nothing_else(monkeypatch):
    """`double_notch` на alpha 3/2: фронт соседа вырезает полосу — контур невыпуклый, а грань одна."""

    from cftuv_envelope.materialize import assemble

    real = assemble.tessellate_faces
    tally = Counter()

    def spy(*args, **kwargs):
        kwargs["tally"] = tally
        return real(*args, **kwargs)

    monkeypatch.setattr(assemble, "tessellate_faces", spy)
    polygon = dict(named_corpus())["double_notch"]
    alpha = Fraction(3, 2)
    left = factories.assemble_polygon_batch(polygon, alpha, law=TRIANGLES)[0]
    assert not +tally
    batch = factories.assemble_polygon_batch(polygon, alpha, law=POLYGONS)[0]
    assert tally[POLYGON_FACES_CONCAVE_EMITTED] == 1
    assert not tally[POLYGON_FACES_TRIANGULATED_NOT_SIMPLE]
    assert not tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE]
    assert validate_geometry_batch(batch) == ()
    assert audit_batch(batch, NORMAL).problems() == ()
    assert batch.vertices == left.vertices and batch.semantic_digest == left.semantic_digest
    assert sum(len(item.ordered_vert_keys) - 2 for item in batch.faces) == len(left.faces)
    assert _uv_by_region_and_vertex(batch) == _uv_by_region_and_vertex(left)
    assert len(batch.faces) < len(left.faces)


# --------------------------------------------------------------------------
# Результат
# --------------------------------------------------------------------------

#: Золотые содержательные дайджесты закона для малых случаев (семантические те же,
#: что у других законов). Меняются ТОЛЬКО осознанно: любое движение — смена состава граней.
#: `weighted`, `point_contact`, `two_edge` пересняты после слияния DECAL-WELD: закон
#: `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` сдвинул вершины `src:` в позиции хоста
#: (те же сдвинутые позиции, что у `GOLDEN_QUADS`: грани законов тут совпадают), состав
#: граней не менялся; `straight3` сдвига не имеет.
GOLDEN_POLYGONS = {
    "weighted": "61d88c1adce5ad5537917e754c0c6ef01c3a7beb7e87fdbb3775f206f482d89d",
    "point_contact": "a18d06885f2ae09de58e54807fd921f3eef52f3b7797b7a6b4e9ff5c81815113",
    "two_edge": "6f7f4f4e71ea11d22aab8c7ed53551b2607986e767814e10e3860a04c6f2352e",
    "straight3": "d77838183f8b36115e0bd985fdb966247e55974ff2ee91effb03542328a2b454",
}


@pytest.mark.parametrize("name", sorted(GOLDEN_POLYGONS))
def test_golden_content_digest_of_the_polygon_law(name):
    assert _polygons_law(name).content_digest == GOLDEN_POLYGONS[name]


def test_the_polygon_result_survives_a_pickle_and_records_its_law():
    result = _polygons_law("full_selection")
    restored = pickle.loads(pickle.dumps(result))
    assert restored == result and restored.decal_topology_law is POLYGONS


def test_a_refusal_records_the_polygon_law_that_was_asked_for():
    from cftuv_envelope.wavefront.conveyor import ConveyorOutcome

    prepared, coverage, request = DOMAINS["skew"]()
    refused = materialize_domain(
        prepared,
        dataclasses.replace(coverage, outcome=ConveyorOutcome.COVERAGE_DID_NOT_CLOSE),
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=POLYGONS,
    )
    assert refused.batch is None and refused.decal_topology_law is POLYGONS
