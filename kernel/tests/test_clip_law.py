"""Закон подъёма `SOURCE_TRIANGLES_CLIPPED_V1`: каждый кусок грани лежит в ОДНОМ треугольнике источника.

Ворота закона. Стадия (`materialize.clip`) проверяется на квадрате 4x4, разрезанном диагональю
(тот же, что у `test_materialize_lift_surface`): куски, площади, вершины `clip:`, T-стыки, свес,
невыпуклый контур, отказ доказательства. Домен целиком — на складке, фаске и четверти цилиндра:
куски лежат в замкнутых треугольниках источника (проверяется в 3D независимым путём), хорда через
складку исчезла, сетка вершин `node:` та же, семантика цепей та же (кроме вершин `clip:`), а счёт
`QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES` — нуль.
"""

from __future__ import annotations

import dataclasses
import math
from collections import Counter
from fractions import Fraction
from functools import lru_cache

import pytest

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1, exact_work_budget
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import clip
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.clip import ClipStageV1
from cftuv_envelope.materialize.coalesce import point_key
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1
from cftuv_envelope.outcomes import NamedOutcome
from cftuv_envelope.wavefront.faces import doubled_shoelace

import developable_factories as df
from developable_route import build_metric, materialize_developable
from test_materialize_lift_surface import (
    P00,
    P01,
    P10,
    P11,
    V00,
    V01,
    V10,
    V11,
    as_point,
    t0,
    t1,
    two_triangle_lift,
)

SURFACE = NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1
CLIPPED = NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1
POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
TRIANGLES = DecalTopologyLawV1.TRIANGLES_V1
QUADS = DecalTopologyLawV1.QUAD_STRIPS_V1


def budget():
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="clip")


def point(x, y):
    return SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y))


def floats(value):
    return float(value.as_rational())


# --------------------------------------------------------------------------
# Стадия на квадрате 4x4 с диагональю (0,0)-(4,4): `t0` под диагональю, `t1` над ней.
# --------------------------------------------------------------------------


def stage_for(faces):
    """`(стадия, циклы, многоугольники)` по граням `[[(x, y), ...], ...]`: ключи `p<k>` по порядку появления."""

    keys, points, cycles, polygons = {}, {}, [], []
    for face in faces:
        cycle = []
        for xy in face:
            if xy not in keys:
                keys[xy] = f"p{len(keys)}"
                points[keys[xy]] = point(*xy)
            cycle.append((keys[xy], points[keys[xy]]))
        cycles.append(cycle)
        polygons.append((tuple(key for key, _point in cycle),))
    stage = ClipStageV1(two_triangle_lift().bind(budget()), budget(), points)
    return stage, cycles, polygons, keys


def chart_area(result, keys, polygon):
    """Удвоенная ориентированная площадь грани на карте (float): вершины старые (`keys`) и `clip:`."""

    xy = {key: coordinates for coordinates, key in keys.items()}
    xy.update({key: tuple(floats(axis) for axis in value) for key, value in result.points.items()})
    pts = [xy[key] for key in polygon]
    return sum(
        pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
        for i in range(len(pts))
    )


def directed_edges(polygons):
    return Counter(
        (polygon[i], polygon[(i + 1) % len(polygon)])
        for face in polygons
        for polygon in face
        for i in range(len(polygon))
    )


def test_a_rectangle_across_the_diagonal_is_cut_into_a_quad_and_a_triangle():
    stage, cycles, polygons, keys = stage_for([[(1, 1), (3, 1), (3, 2), (1, 2)]])
    result = stage.run(cycles, polygons, POLYGONS)
    sizes = sorted(len(item) for item in result.polygons[0])
    assert sizes == [3, 4]
    # Одна новая вершина — на ребре `(3, 2)-(1, 2)` в точке `(2, 2)`; `(1, 1)` уже вершина.
    assert list(result.points) == ["clip:0"]
    assert tuple(floats(axis) for axis in result.points["clip:0"]) == (2.0, 2.0)
    counters = dict(result.counters)
    assert counters[clip.VERTICES_INSERTED] == 1
    assert counters[clip.FACES_CUT] == 1 and counters[clip.PIECES_EMITTED] == 2
    assert counters[clip.FACES_OVERHANG] == counters[clip.FACES_BOUNDARY_MISMATCH] == 0
    # Контур слитой грани несёт вершину ребра ровно на своём месте.
    assert [key for key, _point in result.cycles[0]] == ["p0", "p1", "p2", "clip:0", "p3"]
    assert "clip_vertices=1" in result.note and "pieces=2" in result.note
    # Площади кусков в сумме — площадь многоугольника (в float: 3 x 1 ... 2 * 2 = 4).
    total = sum(chart_area(result, keys, polygon) for polygon in result.polygons[0])
    assert total == pytest.approx(4.0)
    # Подъём новой вершины — независимое аффинное отображение треугольника (оно же с обеих сторон).
    position, (triangle, normal) = result.lifted["clip:0"]
    assert normal is None and triangle in ("t0", "t1")
    assert position == as_point(t0(2, 2)) == as_point(t1(2, 2))


def test_a_clockwise_polygon_is_cut_clockwise():
    stage, cycles, polygons, keys = stage_for([[(1, 2), (3, 2), (3, 1), (1, 1)]])
    result = stage.run(cycles, polygons, POLYGONS)
    assert sorted(len(item) for item in result.polygons[0]) == [3, 4]
    assert all(chart_area(result, keys, polygon) < 0 for polygon in result.polygons[0])


def test_a_polygon_inside_one_triangle_is_not_cut_and_gets_no_vertex():
    stage, cycles, polygons, keys = stage_for([[(2, 1), (3, 1), (3, 2)]])
    result = stage.run(cycles, polygons, POLYGONS)
    assert result.polygons[0] == (("p0", "p1", "p2"),)
    assert not result.points and not result.lifted
    counters = dict(result.counters)
    assert counters[clip.FACES_IN_ONE_TRIANGLE] == 1 and counters[clip.FACES_CUT] == 0


def test_two_neighbours_share_the_vertex_on_their_common_edge_and_close_the_mesh():
    """Нет T-стыков: ребро `(1, 2)-(3, 2)` несёт `(2, 2)` у обеих граней, границы сходятся с контурами."""

    stage, cycles, polygons, keys = stage_for(
        [[(1, 1), (3, 1), (3, 2), (1, 2)], [(1, 2), (3, 2), (3, 3), (1, 3)]]
    )
    result = stage.run(cycles, polygons, POLYGONS)
    # Диагональ пересекает общее ребро в `(2, 2)`; `(3, 3)` — уже вершина второй грани.
    assert list(result.points) == ["clip:0"]
    assert tuple(floats(axis) for axis in result.points["clip:0"]) == (2.0, 2.0)
    edges = directed_edges(result.polygons)
    assert max(edges.values()) == 1, "a directed edge is used twice"
    boundary = {edge for edge in edges if (edge[1], edge[0]) not in edges}
    chain = set()
    for contour in result.cycles:
        for i in range(len(contour)):
            chain.add((contour[i][0], contour[(i + 1) % len(contour)][0]))
    # Общее ребро граней внутреннее: его полурёбра встречные, граница меша — внешний контур.
    assert boundary == chain - {edge for edge in chain if (edge[1], edge[0]) in chain}
    shared = [key for key, _point in result.cycles[0] if key.startswith("clip:")]
    assert set(shared) & {key for key, _point in result.cycles[1]} == {"clip:0"}
    # Каждая вершина `clip:` входит не менее чем в две грани.
    uses = Counter(key for face in result.polygons for polygon in face for key in polygon)
    assert all(uses[key] >= 2 for key in result.points)


def test_a_polygon_overhanging_the_triangulation_stays_ears_and_is_named():
    stage, cycles, polygons, keys = stage_for([[(3, 1), (5, 1), (5, 2), (3, 2)]])
    result = stage.run(cycles, polygons, POLYGONS)
    assert len(result.polygons[0]) == 2 and all(len(item) == 3 for item in result.polygons[0])
    counters = dict(result.counters)
    assert counters[clip.FACES_OVERHANG] == 1 and counters[clip.PIECES_EMITTED] == 0
    # Ни один сосед не разрезан: рёбрам вершины не нужны, обрезков у границы нет.
    assert not result.points
    assert "overhang_faces=1" in result.note


def test_an_overhanging_neighbour_of_a_cut_polygon_carries_the_cut_vertex_on_the_shared_edge():
    stage, cycles, polygons, keys = stage_for(
        [[(1, 1), (3, 1), (3, 2), (1, 2)], [(1, 2), (3, 2), (3, 5), (1, 5)]]
    )
    result = stage.run(cycles, polygons, POLYGONS)
    assert list(result.points) == ["clip:0"]
    assert dict(result.counters)[clip.FACES_OVERHANG] == 1
    over = result.polygons[1]
    assert len(over) >= 2 and all(len(item) == 3 for item in over)
    assert any("clip:0" in item for item in over), "T-junction: the shared vertex is missing"
    edges = directed_edges(result.polygons)
    assert max(edges.values()) == 1


def test_a_concave_polygon_is_cut_by_ears_and_the_pieces_of_one_triangle_are_merged():
    """Г-образный контур: уши режутся треугольниками, куски одного треугольника складываются в один."""

    stage, cycles, polygons, keys = stage_for(
        [[(1, 1), (3, 1), (3, 2), (2, 2), (2, 3), (1, 3)]]
    )
    result = stage.run(cycles, polygons, POLYGONS)
    counters = dict(result.counters)
    assert counters[clip.FACES_CUT_BY_EARS] == 1
    assert counters[clip.PIECES_EMITTED] == 2
    assert counters[clip.PIECES_MERGED] >= 1 and counters[clip.PIECES_KEPT_SEPARATE] == 0
    assert len(result.polygons[0]) == 2
    total = sum(chart_area(result, keys, polygon) for polygon in result.polygons[0])
    assert total == pytest.approx(6.0)  # удвоенная площадь Г-образного контура (площадь 3)


@pytest.mark.parametrize("law", [TRIANGLES, QUADS])
def test_the_topology_law_still_decides_the_shape_of_the_pieces(law):
    stage, cycles, polygons, keys = stage_for([[(1, 1), (3, 1), (3, 2), (1, 2)]])
    result = stage.run(cycles, polygons, law)
    sizes = sorted(len(item) for item in result.polygons[0])
    # Куски — четырёхгранье и треугольник; под законом треугольников четырёхгранье режется по ушам.
    assert sizes == ([3, 3, 3] if law is TRIANGLES else [3, 4])


def test_a_piece_that_is_not_inside_its_triangle_is_a_named_refusal(monkeypatch):
    """Доказательство — не формальность: кусок, подписанный чужим треугольником, отказывает."""

    original = ClipStageV1._positive_pieces

    def relabelled(self, groups):
        return [(1 - ti if ti in (0, 1) else ti, nodes, area) for ti, nodes, area in original(self, groups)]

    monkeypatch.setattr(ClipStageV1, "_positive_pieces", relabelled)
    stage, cycles, polygons, keys = stage_for([[(3, 1), (4, 1), (4, 3), (3, 3)]])
    with pytest.raises(MaterializationRefusal) as refusal:
        stage.run(cycles, polygons, POLYGONS)
    assert refusal.value.outcome is MaterializationOutcome.CLIP_PIECE_LEFT_ITS_TRIANGLE
    assert "outside the closed source triangle" in refusal.value.detail


def test_clip_vertices_are_numbered_after_the_existing_ones_in_emission_order():
    faces = [[(1, 1), (3, 1), (3, 2), (1, 2)], [(1, 2), (3, 2), (3, 2.5), (1, 2.5)], [(1, 2.5), (3, 2.5), (3, 3.5), (1, 3.5)]]
    stage, cycles, polygons, keys = stage_for(faces)
    first = stage.run(cycles, polygons, POLYGONS)
    again_stage, again_cycles, again_polygons, _keys = stage_for(faces)
    second = again_stage.run(again_cycles, again_polygons, POLYGONS)
    assert first.polygons == second.polygons
    # `(2, 2)` на общем ребре, `(3, 3)` на правой стороне третьей грани, `(2.5, 2.5)` на её нижнем ребре.
    assert [key for key in first.points] == ["clip:0", "clip:1", "clip:2"]
    assert {key: point_key(value) for key, value in first.points.items()} == {
        key: point_key(value) for key, value in second.points.items()
    }


# --------------------------------------------------------------------------
# Домен целиком: складка 90°, фаска, четверть цилиндра.
# --------------------------------------------------------------------------

DOMAINS = {
    "fold": (df.fold_strip, ("r0a", "r0b"), "1.5"),
    "fold_wide": (df.fold_strip, ("r0a", "r0b"), "2.5"),
    "slant": (df.slant_fold, ("r0a", "r0b"), "1.2"),
    "slant_wide": (df.slant_fold, ("r0a", "r0b"), "2.0"),
    "bevel": (lambda: df.bevel_strip(4), ("r0a", "r0b"), "2.5"),
    "quarter": (df.quarter_cylinder, ("r0a", "r0b"), "0.8"),
}
#: Допуск «на поверхности», метры: точные на двоичной сетке и допуск на ячейку источника у остальных
#: (вершины `src:` встают в позицию хоста, не привязанную к решётке: закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1`).
SURFACE_TOLERANCE = {"fold": 1e-9, "fold_wide": 1e-9, "slant": 1e-9, "slant_wide": 1e-9, "bevel": 2e-4, "quarter": 2e-4}


@lru_cache(maxsize=None)
def pair(name, law=POLYGONS):
    """`(без резки, с резкой, части поверхности)` одного домена под одним законом топологии."""

    make, route, alpha = DOMAINS[name]
    parts = make()
    plain, _ = materialize_developable(parts, route, alpha=alpha, decal_topology_law=law)
    cut, _ = materialize_developable(
        parts, route, alpha=alpha, decal_topology_law=law, near_planar_lift_law=CLIPPED
    )
    return plain, cut, parts


def triangles_3d(parts):
    position = {item.vertex_id: item.position for item in parts[0]}
    return [
        tuple((position[v].x, position[v].y, position[v].z) for v in item.vertex_ids)
        for item in parts[2]
    ]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def in_closed_triangle(points, triangle, tolerance=1e-6):
    """Все точки — в плоскости треугольника (до допуска) и внутри него: независимый от ядра путь."""

    a, b, c = triangle
    normal = cross(sub(b, a), sub(c, a))
    length = math.sqrt(dot(normal, normal))
    for p in points:
        if abs(dot(sub(p, a), normal)) / length > tolerance:
            return False
        for first, second in ((a, b), (b, c), (c, a)):
            if dot(cross(sub(second, first), sub(p, first)), normal) / length < -tolerance:
                return False
    return True


def closest_distance(p, triangle):
    """Расстояние от точки до треугольника в 3D (Эриксон, «Real-Time Collision Detection»)."""

    a, b, c = triangle
    ab, ac, ap = sub(b, a), sub(c, a), sub(p, a)
    d1, d2 = dot(ab, ap), dot(ac, ap)
    if d1 <= 0 and d2 <= 0:
        q = a
    else:
        bp = sub(p, b)
        d3, d4 = dot(ab, bp), dot(ac, bp)
        cp = sub(p, c)
        d5, d6 = dot(ab, cp), dot(ac, cp)
        vc = d1 * d4 - d3 * d2
        vb = d5 * d2 - d1 * d6
        va = d3 * d6 - d5 * d4
        if d3 >= 0 and d4 <= d3:
            q = b
        elif vc <= 0 and d1 >= 0 and d3 <= 0:
            q = tuple(a[i] + ab[i] * d1 / (d1 - d3) for i in range(3))
        elif d6 >= 0 and d5 <= d6:
            q = c
        elif vb <= 0 and d2 >= 0 and d6 <= 0:
            q = tuple(a[i] + ac[i] * d2 / (d2 - d6) for i in range(3))
        elif va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
            w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
            q = tuple(b[i] + (c[i] - b[i]) * w for i in range(3))
        else:
            denom = 1.0 / (va + vb + vc)
            v, w = vb * denom, vc * denom
            q = tuple(a[i] + ab[i] * v + ac[i] * w for i in range(3))
    return math.sqrt(dot(sub(p, q), sub(p, q)))


def face_points(batch):
    position = {item.vert_key.value: (item.position.x, item.position.y, item.position.z) for item in batch.vertices}
    return [[position[key.value] for key in face.ordered_vert_keys] for face in batch.faces]


def centroid(points):
    return tuple(sum(p[i] for p in points) / len(points) for i in range(3))


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_every_face_of_the_clipped_domain_lies_in_one_closed_source_triangle(name):
    plain, cut, parts = pair(name)
    assert cut.is_materialized, cut.detail
    triangles = triangles_3d(parts)
    tolerance = max(SURFACE_TOLERANCE[name], 1e-6)
    for points in face_points(cut.batch):
        assert any(in_closed_triangle(points, triangle, tolerance) for triangle in triangles), points


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_the_clipped_domain_never_cuts_into_the_surface(name):
    """Хорда через складку исчезла: точки грани (вершины и центр) на поверхности источника."""

    plain, cut, parts = pair(name)
    triangles = triangles_3d(parts)

    def depth(batch):
        return max(
            min(closest_distance(point, triangle) for triangle in triangles)
            for points in face_points(batch)
            for point in (*points, centroid(points))
        )

    assert depth(cut.batch) <= SURFACE_TOLERANCE[name]
    if name == "slant":
        # Без резки лента через косую складку 90° режет в стену: измеримая, а не придуманная польза.
        assert depth(plain.batch) > 0.05


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_the_clipped_domain_covers_the_same_uv_area_and_keeps_every_node_vertex(name):
    plain, cut, _parts = pair(name)

    def uv_area(batch):
        total = 0.0
        for face in batch.faces:
            uv = [(fact.uv.u, fact.uv.v) for fact in face.uv_facts]
            total += abs(
                sum(
                    uv[i][0] * uv[(i + 1) % len(uv)][1] - uv[(i + 1) % len(uv)][0] * uv[i][1]
                    for i in range(len(uv))
                )
            )
        return total

    assert uv_area(cut.batch) == pytest.approx(uv_area(plain.batch), rel=1e-9)
    keys = lambda batch: {item.vert_key.value for item in batch.vertices}
    assert keys(plain.batch) <= keys(cut.batch)
    extra = keys(cut.batch) - keys(plain.batch)
    assert extra and all(key.startswith("clip:") for key in extra)
    assert {f"clip:{k}" for k in range(len(extra))} == extra
    assert len({item.position for item in cut.batch.vertices}) == len(cut.batch.vertices)


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_the_clipped_domain_counts_no_quad_split_and_names_its_numbers(name):
    plain, cut, _parts = pair(name)
    counters = dict(cut.counters)
    assert counters["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 0
    assert counters["MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER"] == 0
    assert counters[clip.VERTICES_INSERTED] == len(
        [item for item in cut.batch.vertices if item.vert_key.value.startswith("clip:")]
    )
    assert counters[clip.FACES_OVERHANG] == counters[clip.FACES_BOUNDARY_MISMATCH] == 0
    assert counters["MATERIALIZE_QUADS"] >= dict(plain.counters)["MATERIALIZE_QUADS"]
    lines = [line for line in cut.diagnostics if line.startswith("SOURCE_EDGES_LIFTED_ONTO_SURFACE")]
    assert len(lines) == 1 and f"clip_vertices={counters[clip.VERTICES_INSERTED]}" in lines[0]
    assert any(
        item.outcome is NamedOutcome.SOURCE_EDGES_LIFTED_ONTO_SURFACE for item in cut.batch.diagnostics
    )
    assert not [line for line in plain.diagnostics if line.startswith("SOURCE_EDGES")]


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_the_chains_of_the_clipped_domain_are_the_plain_chains_plus_clip_vertices(name):
    """Семантика без `clip:`: те же цепи, регионы и факты станций (ворота «дайджест по модулю clip»)."""

    plain, cut, _parts = pair(name)

    def stripped(chains):
        return {
            (item.semantic_boundary_id.value if hasattr(item, "semantic_boundary_id") else item.semantic_interface_id.value):
            tuple(key.value for key in item.ordered_vert_keys if not key.value.startswith("clip:"))
            for item in chains
        }

    assert stripped(cut.batch.boundary_chains) == stripped(plain.batch.boundary_chains)
    assert stripped(cut.batch.interface_chains) == stripped(plain.batch.interface_chains)
    assert cut.batch.semantic_regions == plain.batch.semantic_regions
    base_facts = {
        (fact.semantic_region_id.value, fact.vert_key.value): (fact.source_s, fact.source_r)
        for fact in plain.batch.station_facts
    }
    cut_facts = {
        (fact.semantic_region_id.value, fact.vert_key.value): (fact.source_s, fact.source_r)
        for fact in cut.batch.station_facts
    }
    assert {slot: value for slot, value in cut_facts.items() if not slot[1].startswith("clip:")} == base_facts
    assert any(slot[1].startswith("clip:") for slot in cut_facts)


@pytest.mark.parametrize("name", sorted(DOMAINS))
def test_the_clipped_domain_is_the_same_for_the_other_topology_laws_up_to_the_faces(name):
    """Закон топологии решает форму кусков: треугольники, четырёхгранья либо многоугольники; кусок остаётся в треугольнике.

    Вершины `clip:` у законов могут различаться: под `TRIANGLES_V1` режутся УШИ контура, а их диагонали
    тоже пересекают рёбра источника. Общее — доказательство (каждая грань в одном замкнутом треугольнике)
    и закрытая сетка (аудит ядра не отказал).
    """

    _plain, _cut, parts = pair(name)
    triangles = triangles_3d(parts)
    tolerance = max(SURFACE_TOLERANCE[name], 1e-6)
    for law in (TRIANGLES, QUADS):
        _plain, cut, _parts = pair(name, law)
        assert cut.is_materialized, (name, law, cut.detail)
        sizes = {len(face.ordered_vert_keys) for face in cut.batch.faces}
        assert sizes == {3} if law is TRIANGLES else sizes <= {3, 4}
        for points in face_points(cut.batch):
            assert any(in_closed_triangle(points, triangle, tolerance) for triangle in triangles)


def test_a_domain_on_a_fold_gets_quads_where_the_plain_law_had_only_triangles():
    plain, cut, _parts = pair("fold")
    assert Counter(len(face.ordered_vert_keys) for face in plain.batch.faces) == {3: 4}
    assert Counter(len(face.ordered_vert_keys) for face in cut.batch.faces) == {3: 3, 4: 1}
    wide = pair("slant_wide")
    assert Counter(len(face.ordered_vert_keys) for face in wide[1].batch.faces) == {4: 4}


def test_the_spread_of_offset_normals_inside_a_face_is_recorded_not_judged():
    """Грань кусков плоская ДО смещения; развод нормалей смещения внутри неё — число, у которого нет порога."""

    plain, cut, _parts = pair("quarter")
    spread = dict(cut.counters)["MATERIALIZE_FACES_MAX_OFFSET_NORMAL_ANGLE_MILLIDEG"]
    assert 0 < spread <= 90_000, "a piece of a curved wall has vertex normals that differ"
    # Без резки четырёхгранье с разными нормалями резалось (`QUADS_SPLIT_OFFSET_NORMALS_DIFFER`): развода в гранях нет.
    assert dict(plain.counters)["MATERIALIZE_FACES_MAX_OFFSET_NORMAL_ANGLE_MILLIDEG"] == 0
    assert dict(cut.counters)["MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER"] == 0
    # Число записано и там, где нормали у вершин куска равны (нулём, а не отсутствием).
    assert "MATERIALIZE_FACES_MAX_OFFSET_NORMAL_ANGLE_MILLIDEG" in dict(pair("fold")[1].counters)


def test_the_clipped_result_is_deterministic():
    make, route, alpha = DOMAINS["bevel"]
    first, _ = materialize_developable(make(), route, alpha=alpha, decal_topology_law=POLYGONS, near_planar_lift_law=CLIPPED)
    second, _ = materialize_developable(make(), route, alpha=alpha, decal_topology_law=POLYGONS, near_planar_lift_law=CLIPPED)
    assert first.content_digest == second.content_digest
    assert first.offset_normals_digest == second.offset_normals_digest


# --------------------------------------------------------------------------
# Закон укладки: допуск, метрика, плоскость.
# --------------------------------------------------------------------------


def test_the_metric_records_the_judging_law_and_is_the_same_under_the_clipped_request():
    parts = df.fold_strip()
    plain = build_metric(parts, near_planar_lift_law=SURFACE)
    clipped = build_metric(parts, near_planar_lift_law=CLIPPED)
    assert plain == clipped
    assert CLIPPED.judged_as is SURFACE and SURFACE.judged_as is SURFACE
    assert CLIPPED.onto_surface and SURFACE.onto_surface
    assert not NearPlanarLiftLawV1.CERTIFIED_PLANE_V1.onto_surface


def test_an_exact_plane_domain_is_not_touched_by_the_clipped_request():
    from materialize_factories import straight_chain_domain
    from cftuv_envelope.materialize.domain import materialize_domain

    prepared, coverage, request = straight_chain_domain()
    request = dataclasses.replace(request, uv_policy_id=PolicyId("UV_DIRECT_STRIP_V1"))
    plain = materialize_domain(prepared, coverage, request=request, decal_topology_law=POLYGONS)
    cut = materialize_domain(
        prepared, coverage, request=request, decal_topology_law=POLYGONS, near_planar_lift_law=CLIPPED
    )
    assert plain.content_digest == cut.content_digest
    assert plain.batch.semantic_digest == cut.batch.semantic_digest
    assert not any(name.startswith("MATERIALIZE_CLIP_") for name, _value in cut.counters)


# --------------------------------------------------------------------------
# Near-planar на поверхности (синтетика карты `SKEW`) в обычной и зеркальной карте.
# --------------------------------------------------------------------------


def surface_triangles_3d(prepared):
    ir = prepared.context.snapshot.surface_ir
    position = {item.vertex_id: item.position for item in prepared.context.snapshot.source_vertices}
    return [
        tuple((position[v].x, position[v].y, position[v].z) for v in item.vertex_ids)
        for item in ir.surface_triangles
    ]


def surface_result(mirrored, law):
    from test_decal_topology_law import UV, _mirrored, _near_planar_on_surface
    from cftuv_envelope.materialize.domain import materialize_domain

    make = _mirrored(lambda: _near_planar_on_surface("1")) if mirrored else (lambda: _near_planar_on_surface("1"))
    prepared, coverage, request = make()
    request = dataclasses.replace(request, uv_policy_id=UV)
    result = materialize_domain(
        prepared, coverage, request=request, near_planar_lift_law=CLIPPED, decal_topology_law=law
    )
    return prepared, result


@pytest.mark.parametrize("mirrored", [False, True], ids=["ccw", "cw"])
@pytest.mark.parametrize("law", [POLYGONS, TRIANGLES, QUADS])
def test_a_near_planar_surface_domain_is_clipped_in_either_chart_orientation(mirrored, law):
    prepared, result = surface_result(mirrored, law)
    assert result.is_materialized, result.detail
    triangles = surface_triangles_3d(prepared)
    for points in face_points(result.batch):
        assert any(in_closed_triangle(points, triangle, 1e-4) for triangle in triangles)
    counters = dict(result.counters)
    assert counters["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 0
    assert counters[clip.VERTICES_INSERTED] >= 1
    # Ориентация карты не меняет резку: то же число вершин и граней у зеркальной карты.
    other_prepared, other = surface_result(not mirrored, law)
    assert len(result.batch.vertices) == len(other.batch.vertices)
    assert len(result.batch.faces) == len(other.batch.faces)


def test_a_boundary_that_does_not_close_falls_back_to_ears_and_is_named(monkeypatch):
    """Куски с верной площадью, но границей не по контуру — не молчаливая сетка: уши и свой счётчик."""

    monkeypatch.setattr(ClipStageV1, "_boundary_is", lambda self, pieces, nodes: False)
    stage, cycles, polygons, keys = stage_for([[(1, 1), (3, 1), (3, 2), (1, 2)]])
    result = stage.run(cycles, polygons, POLYGONS)
    counters = dict(result.counters)
    assert counters[clip.FACES_BOUNDARY_MISMATCH] == 1 and counters[clip.FACES_OVERHANG] == 0
    assert all(len(item) == 3 for item in result.polygons[0])
    assert "boundary_mismatch_faces=1" in result.note


def test_the_triangles_of_emitted_pieces_are_the_ears_the_orientation_guard_reads():
    stage, cycles, polygons, keys = stage_for([[(1, 1), (3, 1), (3, 2), (1, 2)]])
    result = stage.run(cycles, polygons, POLYGONS)
    points = {**{key: point(*xy) for xy, key in keys.items()}, **result.points}
    triangles = clip.piece_triangles(result.polygons, points, budget())
    # Четырёхгранье даёт два уха, треугольник — себя: три треугольника, и каждый по ключам куска.
    assert len(triangles) == 3
    assert {key for triangle in triangles for key in triangle} == {*points}
    assert all(len(set(triangle)) == 3 for triangle in triangles)
