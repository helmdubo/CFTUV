"""Закон `FAN_FACE_TRIANGULATED_FROM_APEX_V1`: срезанный веер не зависит от начала контура.

Грань веера, обрезанная локусом соседа, раньше резалась отсечением ушей по порядку контура
(`coverage_at` отдаёт его с произвольной точки), и конгруэнтные углы окон получали то
диагональ мимо вершины веера, то через неё. Под `PLANAR_POLYGONS_V1` такая клетка на точной
плоскости — одна грань (проста, UV аффинна по построению), иначе она режется ОТ вершины
веера; по ушам — только когда вершина не видна, и это названо. Прежние законы не тронуты.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import sys
from collections import Counter
from fractions import Fraction
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace

import pytest

from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from cftuv_envelope.ids import VertexKey
from cftuv_envelope.materialize.assemble import (
    FAN_FACES_CUT_BY_NEIGHBOUR,
    FAN_FACES_NOT_STAR_FROM_APEX,
    FAN_FACES_TRIANGULATED_FROM_APEX,
    FAN_POLYGON_FACES_CONCAVE_EMITTED,
    FAN_POLYGON_FACES_EMITTED,
    POLYGON_FACES_TRIANGULATED_NOT_SIMPLE,
    POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE,
    tessellate_faces,
)
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.tessellate import triangulate_exact
from cftuv_envelope.wavefront.faces import orientation

from test_decal_topology_law import UV, _crowded_patch17_d2_domain, _cut_fans_domain, _mirrored
from test_planar_polygon_law import BUDGET, _area, _points, _uv

TRIANGLES = DecalTopologyLawV1.TRIANGLES_V1
QUADS = DecalTopologyLawV1.QUAD_STRIPS_V1
POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1

#: Клетки веера с вершиной в `(0, 0)`: выпуклая, невыпуклая (правый поворот в `(1, 1)`),
#: от которой вершина всё же видит все остальные, и «стрела» (вершина `(2, 1)` ей не видна).
CONVEX = ((0, 0), (4, 0), (3, 3), (0, 4))
DART = ((0, 0), (4, 0), (1, 1), (0, 4))
HIDDEN = ((0, 0), (4, 0), (4, 4), (2, 1))


def _cell(raw, *, owner=(0, 0, 1, 1, 1), start=0, mirror=False):
    """`(кадр, контур)` клетки веера: ключи `k<i>` по исходному порядку, контур сдвинут/развёрнут."""

    points = _points(raw)
    cycle = [(f"k{index}", point) for index, point in enumerate(points)]
    cycle = cycle[start:] + cycle[:start]
    if mirror:
        cycle = cycle[::-1]
    face = SimpleNamespace(owner=owner, doubled_area=_area(points), parts=())
    return SimpleNamespace(is_fan=True, face=face), tuple(cycle)


def _run(raw, *, exact_plane=True, reverse=False, law=POLYGONS, uv=None, **cell):
    frame, cycle = _cell(raw, **cell)
    tally = Counter()
    polygons = tessellate_faces(
        [frame], [cycle], BUDGET(), reverse, law, exact_plane, tally,
        uv if uv is not None else _uv(cycle),
    )
    return polygons[0], tally


def _faces(polygons):
    return {frozenset(polygon) for polygon in polygons}


def _accounted(tally):
    """Каждая срезанная грань веера названа ровно одним исходом."""

    return tally[FAN_FACES_CUT_BY_NEIGHBOUR] == (
        tally[FAN_POLYGON_FACES_EMITTED]
        + tally[FAN_FACES_TRIANGULATED_FROM_APEX]
        + tally[FAN_FACES_NOT_STAR_FROM_APEX]
    )


STARTS = [(start, mirror, reverse) for start in range(4) for mirror in (False, True) for reverse in (False, True)]


@pytest.mark.parametrize("raw", (CONVEX, DART))
@pytest.mark.parametrize("start,mirror,reverse", STARTS)
def test_a_cut_fan_on_an_exact_plane_is_one_face_whatever_the_contour_start(raw, start, mirror, reverse):
    polygons, tally = _run(raw, start=start, mirror=mirror, reverse=reverse)
    assert len(polygons) == 1 and set(polygons[0]) == {f"k{index}" for index in range(4)}
    assert tally[FAN_FACES_CUT_BY_NEIGHBOUR] == tally[FAN_POLYGON_FACES_EMITTED] == 1
    assert tally[FAN_POLYGON_FACES_CONCAVE_EMITTED] == int(raw is DART)
    assert not tally[FAN_FACES_TRIANGULATED_FROM_APEX] and not tally[FAN_FACES_NOT_STAR_FROM_APEX]
    assert _accounted(tally)


@pytest.mark.parametrize("exact_plane", (True, False))
def test_an_intact_triangle_fan_face_is_itself_and_counts_nothing(exact_plane):
    polygons, tally = _run(((0, 0), (4, 0), (0, 4)), exact_plane=exact_plane)
    assert polygons == (("k0", "k1", "k2"),)
    assert not +tally


@pytest.mark.parametrize("raw", (CONVEX, DART))
@pytest.mark.parametrize("start,mirror,reverse", STARTS)
def test_on_a_curved_lift_a_cut_fan_is_split_from_its_apex_whatever_the_contour_start(raw, start, mirror, reverse):
    polygons, tally = _run(raw, exact_plane=False, start=start, mirror=mirror, reverse=reverse)
    reference, _ = _run(raw, exact_plane=False)
    assert len(polygons) == 2 and all("k0" in triangle for triangle in polygons)
    assert _faces(polygons) == _faces(reference)
    assert tally[FAN_FACES_CUT_BY_NEIGHBOUR] == tally[FAN_FACES_TRIANGULATED_FROM_APEX] == 1
    assert not tally[FAN_POLYGON_FACES_EMITTED] and not tally[FAN_FACES_NOT_STAR_FROM_APEX]
    assert _accounted(tally)
    # Ориентация треугольников — точная: против часовой на карте, при `reverse` — по часовой.
    chart = dict(_cell(raw)[1])
    for triangle in polygons:
        sign = orientation(*(chart[key] for key in triangle), BUDGET())
        assert (sign < 0) == reverse and sign != 0


def test_a_non_affine_fan_is_split_from_its_apex_and_the_reason_is_named():
    frame, cycle = _cell(CONVEX)
    clean = _uv(cycle)

    def bent(face, key):
        s, r = clean(face, key)
        return (s + SqrtSumV1.rational(Fraction(1)), r) if key == "k2" else (s, r)

    polygons, tally = _run(CONVEX, uv=bent)
    assert len(polygons) == 2 and all("k0" in triangle for triangle in polygons)
    assert tally[FAN_FACES_TRIANGULATED_FROM_APEX] == 1 and not tally[FAN_POLYGON_FACES_EMITTED]
    assert tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE] == 1
    assert _accounted(tally)


@pytest.mark.parametrize(
    "raw,owner",
    ((HIDDEN, (0, 0, 1, 1, 1)), (CONVEX, (9, 9, 1, 1, 1))),
    ids=("apex-does-not-see-a-vertex", "apex-is-not-in-the-contour"),
)
def test_a_cell_that_is_not_a_star_from_its_apex_falls_back_to_ears_and_is_named(raw, owner):
    polygons, tally = _run(raw, exact_plane=False, owner=owner)
    ears = triangulate_exact(_points(raw), BUDGET())
    assert _faces(polygons) == {frozenset(f"k{index}" for index in ear) for ear in ears}
    assert tally[FAN_FACES_NOT_STAR_FROM_APEX] == 1
    assert not tally[FAN_FACES_TRIANGULATED_FROM_APEX] and not tally[FAN_POLYGON_FACES_EMITTED]
    assert _accounted(tally)


def test_a_cell_the_apex_does_not_see_keeps_the_reason_it_was_not_one_face():
    """Путь по ушам называет и причину, как лента: неаффинная UV у клетки, которую вершина не видит."""

    _frame, cycle = _cell(HIDDEN)
    clean = _uv(cycle)

    def bent(face, key):
        s, r = clean(face, key)
        return (s + SqrtSumV1.rational(Fraction(1)), r) if key == "k2" else (s, r)

    polygons, tally = _run(HIDDEN, uv=bent)
    assert len(polygons) == 2 and not tally[FAN_POLYGON_FACES_EMITTED]
    assert tally[FAN_FACES_NOT_STAR_FROM_APEX] == 1 == tally[POLYGON_FACES_TRIANGULATED_UV_NOT_AFFINE]
    assert not tally[FAN_FACES_TRIANGULATED_FROM_APEX] and not tally[POLYGON_FACES_TRIANGULATED_NOT_SIMPLE]
    assert _accounted(tally)


@pytest.mark.parametrize("law", (TRIANGLES, QUADS))
def test_the_older_laws_still_cut_a_fan_by_ears_and_count_nothing_new(law):
    frame, cycle = _cell(HIDDEN, start=2)
    polygons = tessellate_faces([frame], [cycle], BUDGET(), False, law)[0]
    ears = triangulate_exact(tuple(point for _key, point in cycle), BUDGET())
    assert polygons == tuple(tuple(cycle[index][0] for index in ear) for ear in ears)


# --------------------------------------------------------------------------
# Полевой домен: `2`, патч 0, срезанные веера окон
# --------------------------------------------------------------------------


@lru_cache(maxsize=None)
def _result(law, mirrored=False):
    prepared, coverage, request = (_mirrored(_cut_fans_domain)() if mirrored else _cut_fans_domain())
    return materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=law,
    )


@lru_cache(maxsize=None)
def _tool():
    path = Path(__file__).resolve().parents[2] / "tools" / "fan_congruence_check.py"
    spec = importlib.util.spec_from_file_location("fan_congruence_check", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _fans(law, mirrored=False):
    result = _result(law, mirrored)
    assert result.is_materialized, result.detail
    fans, skipped = _tool().fans_of(result.batch, "fixture")
    assert skipped == 0
    return fans


def test_every_cut_fan_of_the_field_domain_is_one_face_under_the_polygon_law():
    counters = dict(_result(POLYGONS).counters)
    assert counters[FAN_FACES_CUT_BY_NEIGHBOUR] == 12 > 0
    assert counters[FAN_POLYGON_FACES_EMITTED] == 12
    assert counters[FAN_FACES_TRIANGULATED_FROM_APEX] == counters[FAN_FACES_NOT_STAR_FROM_APEX] == 0
    shapes = Counter(_tool().shape_of(fan) for fan in _fans(POLYGONS))
    assert shapes == {"polygon": 12, "triangle": 12}


def test_the_older_laws_keep_the_ear_cut_on_the_same_domain():
    """Дефект существует там, где его не трогали: срезанный веер — два треугольника по уху."""

    for law in (TRIANGLES, QUADS):
        shapes = Counter(_tool().shape_of(fan) for fan in _fans(law))
        assert shapes["polygon"] == 0 and shapes["ear-fallback"] + shapes["from-apex"] == 12


def test_congruent_corners_get_congruent_fan_faces_and_one_class_per_group():
    tool = _tool()
    fans = _fans(POLYGONS)
    lines: list[str] = []
    groups, multi, extra = tool.report(fans, 0, 3e-3, out=lines.append)
    assert groups >= 1 and multi == 0 and extra == 0, lines
    # Грубые группы (число вершин обводки, угол, срезан ли): в каждой один класс разбиения.
    assert all(len(forms) == 1 for forms in tool.coarse_classes(fans).values())
    assert tool.slivers(fans) == 0


def test_a_mirrored_chart_gives_the_same_fan_faces_as_key_sets():
    def view(fans):
        return sorted(sorted(tuple(sorted(face)) for face in fan.faces) for fan in fans)

    assert view(_fans(POLYGONS, True)) == view(_fans(POLYGONS))


def test_positions_digest_and_normals_do_not_see_the_fan_law():
    triangles, polygons = _result(TRIANGLES), _result(POLYGONS)
    assert polygons.batch.vertices == triangles.batch.vertices
    assert polygons.batch.semantic_digest == triangles.batch.semantic_digest
    assert polygons.vertex_normals == triangles.vertex_normals
    assert polygons.offset_normals_digest == triangles.offset_normals_digest
    assert polygons.content_digest != triangles.content_digest
    assert len(polygons.batch.faces) < len(triangles.batch.faces)


# --------------------------------------------------------------------------
# Невыпуклая клетка из поля и приёмка, которая не молчит о непроверенных веерах
# --------------------------------------------------------------------------


@lru_cache(maxsize=None)
def _crowded_result():
    prepared, coverage, request = _crowded_patch17_d2_domain()
    return materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=POLYGONS,
    )


def _has_right_turn_3d(points):
    """Правый поворот в плоском многоугольнике: относительно нормали Ньюэлла, с запасом против шума."""

    count = len(points)
    normal = [0.0, 0.0, 0.0]
    for index, current in enumerate(points):
        following = points[(index + 1) % count]
        normal[0] += (current[1] - following[1]) * (current[2] + following[2])
        normal[1] += (current[2] - following[2]) * (current[0] + following[0])
        normal[2] += (current[0] - following[0]) * (current[1] + following[1])
    for index in range(count):
        a, b, c = points[index - 1], points[index], points[(index + 1) % count]
        u, w = [b[k] - a[k] for k in range(3)], [c[k] - b[k] for k in range(3)]
        cross = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        size = sum(item * item for item in u) ** 0.5 * sum(item * item for item in w) ** 0.5
        turn = sum(cross[k] * normal[k] for k in range(3))
        if turn < -1e-6 * size * sum(item * item for item in normal) ** 0.5:
            return True
    return False


def test_a_field_cut_fan_with_a_right_turn_is_one_concave_face():
    """`building` патч 17 d2: невыпуклая срезанная клетка поля (в свипе их 30 из 53) — одна грань, названная."""

    result = _crowded_result()
    assert result.is_materialized, result.detail
    counters = dict(result.counters)
    assert counters[FAN_FACES_CUT_BY_NEIGHBOUR] == counters[FAN_POLYGON_FACES_EMITTED] == 2
    assert counters[FAN_POLYGON_FACES_CONCAVE_EMITTED] == 1
    assert not counters[FAN_FACES_TRIANGULATED_FROM_APEX] and not counters[FAN_FACES_NOT_STAR_FROM_APEX]
    fans, skipped = _tool().fans_of(result.batch, "patch17")
    assert skipped == 0
    concave = [
        fan
        for fan in fans
        for face in fan.faces
        if len(face) >= 4 and _has_right_turn_3d([fan.vectors[i] for i in face])
    ]
    assert len(concave) == 1


def test_a_fan_with_an_interior_vertex_is_skipped_and_counted_and_fails_the_acceptance():
    """Внутренняя вершина — разбиение по индексам обводки её не описывает: веер не сравнивается и не молчит."""

    tool = _tool()
    batch = _result(POLYGONS).batch
    fans, skipped = tool.fans_of(batch, "fixture")
    assert skipped == 0
    regions = {
        fact.semantic_region_id
        for fact in batch.station_facts
        if fact.station_model_id.value == "CONSTANT_PHYSICAL_ENDPOINT_S"
    }
    target = next(
        index
        for index, face in enumerate(batch.faces)
        if face.semantic_region_id in regions and len(face.ordered_vert_keys) == 4
    )
    face = batch.faces[target]
    centre = VertexKey("node:interior")
    keys = face.ordered_vert_keys
    by_key = {fact.vert_key: fact for fact in face.uv_facts}
    pieces = []
    for number in range(4):
        ring = (centre, keys[number], keys[(number + 1) % 4])
        facts = tuple(
            dataclasses.replace(by_key[keys[0]], vert_key=centre) if key == centre else by_key[key]
            for key in ring
        )
        pieces.append(dataclasses.replace(face, ordered_vert_keys=ring, uv_facts=facts))
    doctored = dataclasses.replace(
        batch, faces=batch.faces[:target] + tuple(pieces) + batch.faces[target + 1 :]
    )
    kept, skipped = tool.fans_of(doctored, "doctored")
    assert skipped == 1 and len(kept) == len(fans) - 1
    assert tool.verdict(len(kept), skipped, 0, True) == 1
    assert tool.verdict(len(kept), skipped, 0, False) == 0
    assert tool.verdict(len(fans), 0, 0, True) == 0
    assert tool.verdict(0, 0, 0, True) == 1
