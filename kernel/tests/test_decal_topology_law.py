"""Закон топологии декали: `TRIANGLES_V1` и `QUAD_STRIPS_V1`.

Грань батча — многоугольник любой длины от трёх. Закон по умолчанию излучает
треугольники, как до оси; закон лент излучает строго выпуклые четырёхгранья.
Ворота обоих законов одни: равенство ПОСЛЕ разложения четырёхграней обратно в
треугольники (`fan_out`), то есть закон не меняет ни вершин, ни UV, ни
семантики — только то, как вершины собраны в грани.
"""

from __future__ import annotations

import dataclasses
import pickle
from collections import Counter
from fractions import Fraction
from functools import lru_cache
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.contracts.geometry_batch import (
    DecalTopologyLawV1,
    GeometryFaceV1,
)
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import domain
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.assemble import settle_topology, tessellate_faces
from cftuv_envelope.materialize.audit import audit_batch
from cftuv_envelope.materialize.frames import MaterializationRefusal
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.source_lift import (
    FACES_OFF_PLANE,
    FACES_TRIANGULATED_AFTER_LIFT,
    TRIANGLES_FLIPPED_BY_LIFT,
    off_plane_distance,
)
from cftuv_envelope.materialize.tessellate import (
    convex_quad_ring,
    fan_out,
    triangulate_exact,
)
from cftuv_envelope.validation import validate_geometry_batch
from cftuv_envelope.wavefront.conveyor import ConveyorOutcome
from cftuv_envelope.wavefront.faces import doubled_shoelace, orientation
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

import developable_factories
import materialize_factories as factories
from developable_route import materialize_developable
from wavefront_cases import named_corpus

TRIANGLES = DecalTopologyLawV1.TRIANGLES_V1
QUADS = DecalTopologyLawV1.QUAD_STRIPS_V1

UV = PolicyId("UV_DIRECT_STRIP_V1")
NORMAL = (0.0, 0.0, 1.0)


def _points(raw):
    return tuple(
        (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))
        for x, y in raw
    )


def _result(domain, **kwargs):
    prepared, coverage, request = domain
    return materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        **kwargs,
    )


# --------------------------------------------------------------------------
# fan_out: каноническое разбиение грани закона на треугольники
# --------------------------------------------------------------------------


def test_a_triangle_is_its_own_fan_out():
    assert fan_out(("a", "b", "c")) == (("a", "b", "c"),)


@pytest.mark.parametrize("clockwise", (False, True))
def test_the_quad_fan_out_is_the_first_ear_of_the_ear_clipping(clockwise):
    """Разбиение четырёхгранья — РОВНО два треугольника `triangulate_exact`."""

    ring = ((0, 0), (4, 0), (5, 3), (-1, 2))
    raw = ring[::-1] if clockwise else ring
    points = _points(raw)
    triangles = triangulate_exact(points, factories.budget())
    assert triangles is not None and len(triangles) == 2
    keys = tuple("abcd"[: len(points)])
    expected = tuple(tuple(keys[index] for index in item) for item in triangles)
    # Кольцо против часовой: у входа против часовой это сам контур, у входа по
    # часовой — контур в обратном порядке (так делает отсечение ушей).
    quad = keys if not clockwise else keys[::-1]
    assert fan_out(quad) == expected


def test_a_face_longer_than_a_quad_has_no_canonical_split():
    with pytest.raises(ValueError):
        fan_out(("a", "b", "c", "d", "e"))


# --------------------------------------------------------------------------
# Склейка треугольников в четырёхгранья — обратная к `fan_out` операция
# --------------------------------------------------------------------------


def merge_pairs_into_quads(batch):
    """Батч, где каждая пара соседних граней-треугольников вида `fan_out` слита в четырёхгранье.

    Не излучатель закона, а ВНЕШНИЙ построитель для ворот: он берёт готовый
    батч треугольников и склеивает ровно те пары, чьё разбиение ЕСТЬ `fan_out`
    четырёхгранья `(q0, q1, q2, q3)`: `(q3, q0, q1), (q1, q2, q3)` с общей
    диагональю `q1 q3`.
    """

    faces = list(batch.faces)
    merged = []
    index = 0
    while index < len(faces):
        face = faces[index]
        following = faces[index + 1] if index + 1 < len(faces) else None
        if following is not None and _is_quad_split(face, following):
            first = face.ordered_vert_keys
            second = following.ordered_vert_keys
            keys = (first[1], second[0], second[1], first[0])
            by_key = {
                fact.vert_key: fact for fact in (*face.uv_facts, *following.uv_facts)
            }
            merged.append(
                dataclasses.replace(
                    face,
                    face_id=type(face.face_id)(f"face:{len(merged)}"),
                    ordered_vert_keys=keys,
                    uv_facts=tuple(by_key[key] for key in keys),
                )
            )
            index += 2
            continue
        merged.append(
            dataclasses.replace(face, face_id=type(face.face_id)(f"face:{len(merged)}"))
        )
        index += 1
    return dataclasses.replace(batch, faces=tuple(merged))


def _is_quad_split(first: GeometryFaceV1, second: GeometryFaceV1) -> bool:
    a, b = first.ordered_vert_keys, second.ordered_vert_keys
    return (
        len(a) == len(b) == 3
        and a[2] == b[0]
        and a[0] == b[2]
        and a[1] != b[1]
        and first.semantic_region_id == second.semantic_region_id
    )


def _rotation_normal(keys):
    """Грань как цикл, без точки отсчёта: сдвиг к наименьшему ключу."""

    start = min(range(len(keys)), key=lambda index: keys[index])
    return keys[start:] + keys[:start]


def _face_signature(face):
    return (
        _rotation_normal(tuple(key.value for key in face.ordered_vert_keys)),
        face.semantic_region_id,
        face.ownership_claim_id,
    )


def fan_out_batch(batch):
    """Тот же батч, у которого каждая грань разложена `fan_out` в треугольники."""

    faces = []
    for face in batch.faces:
        for triangle in fan_out(face.ordered_vert_keys):
            by_key = {fact.vert_key: fact for fact in face.uv_facts}
            faces.append(
                dataclasses.replace(
                    face,
                    face_id=type(face.face_id)(f"face:{len(faces)}"),
                    ordered_vert_keys=triangle,
                    uv_facts=tuple(by_key[key] for key in triangle),
                )
            )
    return dataclasses.replace(batch, faces=tuple(faces))


#: Домены, чьи слитые контуры — ленты из выпуклых четырёхугольников: косая карта,
#: угол «Г» и кольцо.
QUAD_DOMAINS = ("skew_chain_domain", "l_chains_domain", "ring_domain")


@pytest.fixture(scope="module", params=QUAD_DOMAINS)
def triangle_batch(request):
    result = _result(getattr(factories, request.param)())
    assert result.outcome is MaterializationOutcome.MATERIALIZED, result.detail
    return result.batch


def test_the_merge_of_a_triangle_batch_has_quads_and_fans_back_out_to_it(triangle_batch):
    quads = merge_pairs_into_quads(triangle_batch)
    sizes = {len(face.ordered_vert_keys) for face in quads.faces}
    assert 4 in sizes and len(quads.faces) < len(triangle_batch.faces)
    back = fan_out_batch(quads)
    assert [_face_signature(item) for item in back.faces] == [
        _face_signature(item) for item in triangle_batch.faces
    ]
    assert sum(len(face.ordered_vert_keys) - 2 for face in quads.faces) == len(
        triangle_batch.faces
    )


def test_the_validator_and_the_semantic_digest_accept_a_batch_of_quads(triangle_batch):
    quads = merge_pairs_into_quads(triangle_batch)
    assert validate_geometry_batch(quads) == ()
    assert quads.semantic_digest == triangle_batch.semantic_digest


def test_the_audit_reads_a_quad_like_the_two_triangles_it_is(triangle_batch):
    quads = merge_pairs_into_quads(triangle_batch)
    base = audit_batch(triangle_batch, NORMAL)
    audit = audit_batch(quads, NORMAL)
    assert audit.problems() == ()
    assert audit.faces == len(quads.faces) < base.faces
    assert audit.boundary_edges == base.boundary_edges
    assert audit.boundary_chain_mismatch == 0
    assert audit.flipped_vs_source == base.flipped_vs_source == 0
    assert audit.uv_reversed == 0
    assert (audit.v_min, audit.v_max) == (base.v_min, base.v_max)


def test_the_audit_still_catches_a_crack_and_an_overlap_in_a_quad_batch(triangle_batch):
    quads = merge_pairs_into_quads(triangle_batch)
    quad_index = next(
        index for index, face in enumerate(quads.faces) if len(face.ordered_vert_keys) == 4
    )
    cracked = dataclasses.replace(
        quads, faces=quads.faces[:quad_index] + quads.faces[quad_index + 1 :]
    )
    assert "BOUNDARY_DOES_NOT_MATCH_CHAINS" in audit_batch(cracked, NORMAL).problems()
    doubled = dataclasses.replace(quads, faces=quads.faces + (quads.faces[quad_index],))
    assert "HALF_EDGE_DUPLICATED" in audit_batch(doubled, NORMAL).problems()


def test_the_audit_catches_a_reversed_quad_by_direction_normal_and_uv(triangle_batch):
    quads = merge_pairs_into_quads(triangle_batch)
    quad_index = next(
        index for index, face in enumerate(quads.faces) if len(face.ordered_vert_keys) == 4
    )
    face = quads.faces[quad_index]
    reversed_face = dataclasses.replace(
        face,
        ordered_vert_keys=face.ordered_vert_keys[::-1],
        uv_facts=face.uv_facts[::-1],
    )
    faces = list(quads.faces)
    faces[quad_index] = reversed_face
    audit = audit_batch(dataclasses.replace(quads, faces=tuple(faces)), NORMAL)
    assert audit.flipped_vs_source == 1
    if len(quads.faces) > 1:
        # Соседи этой грани идут против неё: обход и UV-знак уже не у большинства.
        assert "HALF_EDGE_DUPLICATED" in audit.problems()
        assert audit.uv_reversed >= 1


# --------------------------------------------------------------------------
# Закон на материализаторе
# --------------------------------------------------------------------------


def test_the_default_law_is_triangles_and_the_result_records_it():
    result = _result(factories.two_edge_chain_domain())
    assert result.decal_topology_law is DecalTopologyLawV1.TRIANGLES_V1
    counters = dict(result.counters)
    assert counters["MATERIALIZE_FACES_EMITTED"] == len(result.batch.faces)
    assert counters["MATERIALIZE_TRIANGLES"] == len(result.batch.faces)
    assert counters["MATERIALIZE_QUADS"] == 0
    assert all(len(face.ordered_vert_keys) == 3 for face in result.batch.faces)


def test_the_law_is_a_result_field_and_not_a_semantic_record():
    """Закон не входит в `contract_versions` и диагностики: они в семантическом дайджесте."""

    result = _result(factories.two_edge_chain_domain())
    assert all(
        "TOPOLOGY" not in item.value.upper() for item in result.batch.contract_versions
    )
    assert all("TOPOLOGY" not in line.upper() for line in result.diagnostics)


# --------------------------------------------------------------------------
# Строго выпуклый четырёхугольник: предикат закона
# --------------------------------------------------------------------------


def test_a_strictly_convex_quad_gives_its_counter_clockwise_ring_in_both_orientations():
    ring = ((0, 0), (4, 0), (5, 3), (-1, 2))
    budget = factories.budget()
    assert convex_quad_ring(_points(ring), budget) == (0, 1, 2, 3)
    assert convex_quad_ring(_points(ring[::-1]), budget) == (3, 2, 1, 0)


@pytest.mark.parametrize(
    "raw",
    (
        # «Стрела»: одна вершина вогнута.
        ((0, 0), (4, 0), (1, 1), (0, 4)),
        # Плоский угол: третья вершина лежит на отрезке соседей — не строго.
        ((0, 0), (2, 0), (4, 0), (2, 3)),
        # Нулевая площадь.
        ((0, 0), (1, 0), (2, 0), (3, 0)),
        # «Бабочка»: самопересечение, повороты разных знаков.
        ((0, 0), (4, 4), (4, 0), (0, 4)),
        # Не четыре точки.
        ((0, 0), (4, 0), (4, 3)),
        ((0, 0), (4, 0), (5, 2), (4, 4), (0, 3)),
    ),
)
def test_a_contour_that_is_not_strictly_convex_is_not_a_quad(raw):
    assert convex_quad_ring(_points(raw), factories.budget()) is None


# --------------------------------------------------------------------------
# tessellate_faces: что закон оставляет четырёхгранью
# --------------------------------------------------------------------------


def _frame(points, *, fan=False):
    """Грань с кадром ровно в том, что читает тесселяция: точки, площадь, признак веера."""

    total = doubled_shoelace(points)
    if total.sign(budget=factories.budget()) < 0:
        total = SqrtSumV1.zero() - total
    keys = tuple(f"k{index}" for index in range(len(points)))
    frame = SimpleNamespace(
        is_fan=fan, face=SimpleNamespace(owner=(0, 0, 1, 1), doubled_area=total)
    )
    return frame, tuple(zip(keys, points))


def _rotation_normal_triangle(triangle):
    return _rotation_normal(tuple(triangle))


@pytest.mark.parametrize("clockwise", (False, True))
@pytest.mark.parametrize("reverse", (False, True))
def test_a_quad_fans_back_out_to_exactly_the_triangles_of_the_other_law(clockwise, reverse):
    raw = ((0, 0), (4, 0), (5, 3), (-1, 2))
    points = _points(raw[::-1] if clockwise else raw)
    frame, cycle = _frame(points)
    budget = factories.budget()
    quads = tessellate_faces([frame], [cycle], budget, reverse, QUADS)
    triangles = tessellate_faces([frame], [cycle], budget, reverse, TRIANGLES)
    assert [len(item) for item in quads[0]] == [4]
    assert len(triangles[0]) == 2
    split = fan_out(quads[0][0])
    assert [_rotation_normal_triangle(item) for item in split] == [
        _rotation_normal_triangle(item) for item in triangles[0]
    ]


def test_a_fan_face_stays_triangles_even_with_four_points():
    points = _points(((0, 0), (4, 0), (5, 3), (-1, 2)))
    frame, cycle = _frame(points, fan=True)
    result = tessellate_faces([frame], [cycle], factories.budget(), False, QUADS)
    assert [len(item) for item in result[0]] == [3, 3]


def test_a_reflex_four_point_strip_stays_triangles_and_is_named():
    points = _points(((0, 0), (4, 0), (1, 1), (0, 4)))
    frame, cycle = _frame(points)
    result = tessellate_faces([frame], [cycle], factories.budget(), False, QUADS)
    assert [len(item) for item in result[0]] == [3, 3]
    sources = {key: (None, None) for key, _point in cycle}
    settled, numbers = settle_topology([frame], [cycle], result, sources, QUADS)
    assert settled == result
    assert dict(numbers) == {
        "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX": 1,
        "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES": 0,
        "MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER": 0,
        "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED": 0,
    }


def test_a_merged_run_is_triangulated_and_counted_under_the_quad_law_only():
    points = _points(((0, 0), (2, 0), (4, 0), (4, 3), (0, 3)))
    frame, cycle = _frame(points)
    for law, expected in ((QUADS, 1), (TRIANGLES, 0)):
        result = tessellate_faces([frame], [cycle], factories.budget(), False, law)
        assert [len(item) for item in result[0]] == [3, 3, 3]
        sources = {key: (None, None) for key, _point in cycle}
        numbers = dict(settle_topology([frame], [cycle], result, sources, law)[1])
        assert numbers["MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED"] == expected


def test_an_area_that_does_not_close_is_a_named_refusal_for_a_quad_too():
    points = _points(((0, 0), (4, 0), (5, 3), (-1, 2)))
    frame, cycle = _frame(points)
    frame.face.doubled_area = frame.face.doubled_area + SqrtSumV1.rational(Fraction(1))
    with pytest.raises(MaterializationRefusal) as refusal:
        tessellate_faces([frame], [cycle], factories.budget(), False, QUADS)
    assert refusal.value.outcome is MaterializationOutcome.TESSELLATION_DID_NOT_CLOSE
    assert "areas differ" in refusal.value.detail


# --------------------------------------------------------------------------
# Закон QUAD_IN_ONE_SOURCE_TRIANGLE_V1
# --------------------------------------------------------------------------


def _one_quad(tags):
    """Четырёхгранье и записи подъёма по его вершинам: `tags` — `(треугольник, нормаль)` на вершину."""

    points = _points(((0, 0), (4, 0), (5, 3), (-1, 2)))
    frame, cycle = _frame(points)
    polygons = tessellate_faces([frame], [cycle], factories.budget(), False, QUADS)
    return frame, cycle, polygons, {key: tags[index] for index, (key, _p) in enumerate(cycle)}


UP = (0.0, 0.0, 1.0)
TILTED = (0.0, 0.6, 0.8)


def test_a_quad_with_all_four_vertices_in_one_source_triangle_is_kept():
    frame, cycle, polygons, sources = _one_quad([("t7", None)] * 4)
    settled, numbers = settle_topology([frame], [cycle], polygons, sources, QUADS)
    assert settled == polygons
    assert dict(numbers)["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 0


@pytest.mark.parametrize("odd", (0, 1, 2, 3))
def test_a_quad_with_one_vertex_in_another_source_triangle_is_split_canonically(odd):
    tags = [("t7", None)] * 4
    tags[odd] = ("t8", None)
    frame, cycle, polygons, sources = _one_quad(tags)
    settled, numbers = settle_topology([frame], [cycle], polygons, sources, QUADS)
    assert settled == [fan_out(polygons[0][0])]
    found = dict(numbers)
    assert found["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 1
    assert found["MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER"] == 0


def test_a_plane_names_no_triangles_and_every_quad_stays():
    frame, cycle, polygons, sources = _one_quad([(None, None)] * 4)
    assert settle_topology([frame], [cycle], polygons, sources, QUADS)[0] == polygons


def test_a_quad_in_one_triangle_with_equal_offset_normals_stays_planar_and_is_kept():
    frame, cycle, polygons, sources = _one_quad([("t7", UP)] * 4)
    settled, numbers = settle_topology([frame], [cycle], polygons, sources, QUADS)
    assert settled == polygons
    assert dict(numbers)["MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER"] == 0


@pytest.mark.parametrize("odd", (0, 1, 2, 3))
def test_a_quad_whose_offset_normals_differ_is_split_and_named_apart(odd):
    """Смещённая по разным нормалям грань не плоская, даже если вершины лежат в одном треугольнике."""

    tags = [("t7", UP)] * 4
    tags[odd] = ("t7", TILTED)
    frame, cycle, polygons, sources = _one_quad(tags)
    settled, numbers = settle_topology([frame], [cycle], polygons, sources, QUADS)
    assert settled == [fan_out(polygons[0][0])]
    found = dict(numbers)
    assert found["MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER"] == 1
    assert found["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 0


def test_another_triangle_and_other_normals_are_counted_as_the_triangle_split_only():
    tags = [("t7", UP), ("t8", TILTED), ("t7", UP), ("t7", UP)]
    frame, cycle, polygons, sources = _one_quad(tags)
    found = dict(settle_topology([frame], [cycle], polygons, sources, QUADS)[1])
    assert found["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 1
    assert found["MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER"] == 0


# --------------------------------------------------------------------------
# Ворота: закон меняет сборку граней и больше ничего
# --------------------------------------------------------------------------

ON_SURFACE = NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1


def _near_planar_on_surface(alpha):
    snapshot, request = factories.affine_domain(
        faces=(factories.SKEW_FACE,),
        routes=({"name": "source", "points": factories.SKEW_BOTTOM},),
        alpha=alpha,
        planarity_policy=kernel.PlanarityAdmissionLawV1.NEAR_PLANAR_PROJECTION_V1,
        lift={3: 0.002},
        near_planar_lift_law=ON_SURFACE,
    )
    prepared, coverage = factories.prepare_and_cover(snapshot, request)
    return prepared, coverage, prepared.compilation.decal_request


def _mirrored(make):
    """Тот же домен в зеркальной карте: порядок обхода граней разворачивается."""

    def build():
        prepared, coverage, request = make()
        frame = prepared.context.frame
        mirrored = dataclasses.replace(
            frame,
            chart_orientation=type(frame.chart_orientation)(
                "COORDINATE_CW_MATCHES_OWNER_PATCH"
            ),
        )
        context = dataclasses.replace(prepared.context, frame=mirrored)
        return dataclasses.replace(prepared, context=context), coverage, request

    return build


@lru_cache(maxsize=None)
def _cut_fans_domain():
    """`2`, патч 0: веера окон, срезанные локусом соседа (`FAN_FACE_TRIANGULATED_FROM_APEX_V1`)."""

    return factories.field_domain("mesh2_patch0_cut_fans_v1")


#: Корпус ворот: полевые домены, синтетика, near-planar на плоскости и на
#: поверхности, зеркальные карты. Развёртки идут отдельной таблицей (`DEVELOPABLE`).
DOMAINS = {
    "weighted": lambda: factories.field_domain("building_002_weighted_normals_v1"),
    "point_contact": lambda: factories.field_domain("building_002_point_contact_v1"),
    "full_selection": lambda: factories.field_domain("building_002_full_selection_v1"),
    "cut_fans": _cut_fans_domain,
    "two_edge": factories.two_edge_chain_domain,
    "straight3": factories.straight_chain_domain,
    "skew": factories.skew_chain_domain,
    "l_chains": factories.l_chains_domain,
    "ring": factories.ring_domain,
    "near_planar": factories.near_planar_domain,
    "skew_mirrored": _mirrored(factories.skew_chain_domain),
    "l_chains_mirrored": _mirrored(factories.l_chains_domain),
    "ring_mirrored": _mirrored(factories.ring_domain),
    "full_selection_mirrored": _mirrored(
        lambda: factories.field_domain("building_002_full_selection_v1")
    ),
}
SURFACE = {
    "near_planar_surface_a0.5": ("0.5",),
    "near_planar_surface_a1": ("1",),
    "near_planar_surface_a2": ("2",),
}
DEVELOPABLE = {
    "fold-strip": (developable_factories.fold_strip, ("r0a", "r0b"), "1.5"),
    "bevel": (lambda: developable_factories.bevel_strip(4), ("r0a", "r0b"), "2.5"),
    "quarter-cylinder": (developable_factories.quarter_cylinder, ("r0a", "r0b"), "0.8"),
    "cone-sector": (
        lambda: developable_factories.cone(8, boundary_apex=True),
        ("apex", "b0"),
        "0.5",
    ),
}
ALL_NAMES = (*DOMAINS, *SURFACE, *DEVELOPABLE)


@lru_cache(maxsize=None)
def _both_laws(name):
    """`(треугольники, четырёхгранья)` одного домена: тот же вход, разные законы."""

    if name in DEVELOPABLE:
        make, route, alpha = DEVELOPABLE[name]
        return tuple(
            materialize_developable(make(), route, alpha=alpha, decal_topology_law=law)[0]
            for law in (TRIANGLES, QUADS)
        )
    if name in SURFACE:
        prepared, coverage, request = _near_planar_on_surface(*SURFACE[name])
        request = dataclasses.replace(request, uv_policy_id=UV)
        extra = {"near_planar_lift_law": ON_SURFACE}
    else:
        prepared, coverage, request = DOMAINS[name]()
        request = dataclasses.replace(request, uv_policy_id=UV)
        extra = {}
    return tuple(
        materialize_domain(
            prepared, coverage, request=request, decal_topology_law=law, **extra
        )
        for law in (TRIANGLES, QUADS)
    )


def _both_laws_unlifted(name):
    """То же, что `_both_laws`, но без закона `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` (позиций хоста нет)."""

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(domain, "host_positions_of", lambda snapshot: {})
        return _both_laws.__wrapped__(name)


#: Счётчики, которые считают ГРАНИ и потому зависят от закона (или названы им).
LAW_COUNTERS = frozenset(
    (
        "MATERIALIZE_FACES_EMITTED",
        FACES_OFF_PLANE,
        FACES_TRIANGULATED_AFTER_LIFT,
        TRIANGLES_FLIPPED_BY_LIFT,
        "MATERIALIZE_QUADS",
        "MATERIALIZE_QUADS_REFUSED_NOT_CONVEX",
        "MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES",
        "MATERIALIZE_QUADS_SPLIT_OFFSET_NORMALS_DIFFER",
        "MATERIALIZE_MERGED_RUN_FACES_TRIANGULATED",
        "MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE",
        "MATERIALIZE_TRIANGLES_UV_DEGENERATE",
        "MATERIALIZE_TRIANGLES_UV_REVERSED",
    )
)


def _face_view(face):
    """Грань без точки отсчёта и без номера: цикл ключей, UV по ключам, всё остальное."""

    uv = {fact.vert_key.value: fact.uv for fact in face.uv_facts}
    cycle = _rotation_normal(tuple(key.value for key in face.ordered_vert_keys))
    return (
        cycle,
        tuple(uv[key] for key in cycle),
        face.semantic_region_id,
        face.ownership_claim_id,
        face.provenance,
        face.material_id,
    )


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_quad_law_materializes_and_fans_back_out_to_the_triangle_batch(name):
    triangles, quads = _both_laws(name)
    assert triangles.is_materialized and quads.is_materialized, (name, quads.detail)
    assert validate_geometry_batch(quads.batch) == ()
    assert audit_batch(quads.batch, NORMAL).problems() == ()
    back = fan_out_batch(quads.batch)
    # Те же грани в том же порядке, с теми же UV, с точностью до начала цикла...
    assert [_face_view(item) for item in back.faces] == [
        _face_view(item) for item in triangles.batch.faces
    ]
    # ...а значит и как множество.
    assert Counter(_face_view(item) for item in back.faces) == Counter(
        _face_view(item) for item in triangles.batch.faces
    )


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_quad_law_changes_no_vertex_no_uv_no_chain_and_no_digest_of_meaning(name):
    triangles, quads = _both_laws(name)
    left, right = triangles.batch, quads.batch
    assert left.vertices == right.vertices
    assert left.station_facts == right.station_facts
    assert left.semantic_regions == right.semantic_regions
    assert left.boundary_chains == right.boundary_chains
    assert left.interface_chains == right.interface_chains
    assert left.diagnostics == right.diagnostics
    assert left.contract_versions == right.contract_versions
    assert left.semantic_digest == right.semantic_digest
    assert triangles.vertex_normals == quads.vertex_normals
    assert triangles.offset_normals_digest == quads.offset_normals_digest
    assert triangles.diagnostics == quads.diagnostics
    assert quads.decal_topology_law is QUADS and triangles.decal_topology_law is TRIANGLES
    # Содержание меняется по построению (там лежат грани), и только если в нём есть четырёхгранья.
    quad_count = dict(quads.counters)["MATERIALIZE_QUADS"]
    assert (triangles.content_digest != quads.content_digest) == bool(quad_count)


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_counters_agree_except_the_ones_that_count_faces(name):
    triangles, quads = _both_laws(name)
    left, right = dict(triangles.counters), dict(quads.counters)
    assert left.keys() == right.keys()
    for key in left:
        if key in LAW_COUNTERS or key.startswith("EXACT_WORK_"):
            continue
        # Подъём (`LOCATIONS`, `PREDICATES`), станции, вершины, цепи, регионы —
        # побитово те же: закон не прибавляет ни одного нахождения.
        assert left[key] == right[key], key
    sizes = Counter(len(face.ordered_vert_keys) for face in quads.batch.faces)
    assert right["MATERIALIZE_FACES_EMITTED"] == len(quads.batch.faces)
    assert right["MATERIALIZE_QUADS"] == sizes[4]
    # Сумма `n - 2` не зависит от диагонали: это то же число, что у закона треугольников.
    assert right["MATERIALIZE_TRIANGLES"] == sum(
        (size - 2) * count for size, count in sizes.items()
    )
    assert right["MATERIALIZE_TRIANGLES"] == left["MATERIALIZE_TRIANGLES"]
    # Каждое сохранённое четырёхгранье заменило ровно два треугольника.
    assert left["MATERIALIZE_FACES_EMITTED"] - right["MATERIALIZE_FACES_EMITTED"] == right[
        "MATERIALIZE_QUADS"
    ]
    assert left["MATERIALIZE_QUADS"] == 0
    assert set(sizes) <= {3, 4}


@pytest.mark.parametrize("name", ALL_NAMES)
def test_no_quad_of_the_law_is_non_planar(name):
    """Четырёхгранья закона плоские в 3D (с точностью до одного округления позиции).

    Это свойство ПОДЪЁМА: вершины лежат на носителе. Закон
    `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` кладёт вершины `src:` в позиции хоста, и ниже
    плоскость держится уже только в записанных пределах, поэтому точная проверка идёт при
    выключенном законе.
    """

    _triangles, quads = _both_laws_unlifted(name)
    assert_every_quad_is_planar(quads.batch)
    assert dict(quads.counters)[FACES_OFF_PLANE] == 0


@pytest.mark.parametrize("name", ALL_NAMES)
def test_the_lifted_quads_are_planar_within_the_recorded_deviation(name):
    """С законом позиций хоста четырёхгранье плоское в пределах числа, которое закон записал.

    Число — наибольший уход подвинутой вершины от плоскости её грани ДО сдвига; «до» — позиции
    того же домена без закона положения (точный подъём), «после» — с ним.
    """

    _triangles, quads = _both_laws(name)
    _plain_triangles, plain = _both_laws_unlifted(name)
    before = {item.vert_key: item.position for item in plain.batch.vertices}
    after = {item.vert_key: item.position for item in quads.batch.vertices}
    recorded = dict(quads.counters)[FACES_OFF_PLANE] * 1e-9
    measured = max(
        (
            off_plane_distance(
                tuple(before[key] for key in face.ordered_vert_keys),
                tuple(after[key] for key in face.ordered_vert_keys),
            )
            for face in quads.batch.faces
            if len(face.ordered_vert_keys) == 4
        ),
        default=0.0,
    )
    # Записано наибольшее отклонение четырёхгранников с подвинутой вершиной, остальные — точно плоские.
    assert measured == pytest.approx(recorded, abs=1e-9), (name, measured, recorded)


def assert_every_quad_is_planar(batch, tolerance=1e-12):
    position = {item.vert_key: item.position for item in batch.vertices}
    for face in batch.faces:
        if len(face.ordered_vert_keys) != 4:
            continue
        a, b, c, d = (position[key] for key in face.ordered_vert_keys)
        ab = (b.x - a.x, b.y - a.y, b.z - a.z)
        ac = (c.x - a.x, c.y - a.y, c.z - a.z)
        ad = (d.x - a.x, d.y - a.y, d.z - a.z)
        normal = (
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        )
        length = sum(item * item for item in normal) ** 0.5
        scale = max(sum(item * item for item in ad) ** 0.5, 1e-9)
        assert abs(sum(n * v for n, v in zip(normal, ad))) / length / scale < tolerance


@pytest.mark.parametrize("name", sorted(SURFACE))
def test_a_quad_across_two_source_triangles_is_split_and_named(name):
    _triangles, quads = _both_laws(name)
    counters = dict(quads.counters)
    assert counters["MATERIALIZE_QUADS_SPLIT_ACROSS_SOURCE_TRIANGLES"] == 1
    assert counters["MATERIALIZE_QUADS"] == 0
    assert all(len(face.ordered_vert_keys) == 3 for face in quads.batch.faces)


def test_without_the_law_the_split_quad_would_be_non_planar(monkeypatch):
    """Отрицательный контроль: проверка плоскостности видит то, от чего закон бережёт."""

    monkeypatch.setattr(
        domain,
        "settle_topology",
        lambda ff, cy, polygons, sources, law, tally=None: (polygons, ()),
    )
    prepared, coverage, request = _near_planar_on_surface("1")
    quads = materialize_domain(
        prepared,
        coverage,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=QUADS,
        near_planar_lift_law=ON_SURFACE,
    )
    assert quads.is_materialized
    with pytest.raises(AssertionError):
        assert_every_quad_is_planar(quads.batch)


@pytest.mark.parametrize("name", ("weighted", "point_contact", "full_selection"))
def test_the_fans_of_the_field_cases_stay_triangles(name):
    triangles, quads = _both_laws(name)
    fans = dict(quads.counters)["MATERIALIZE_FAN_FACES"]
    assert fans == dict(triangles.counters)["MATERIALIZE_FAN_FACES"]
    if fans:
        assert any(len(face.ordered_vert_keys) == 3 for face in quads.batch.faces)


def test_the_quad_law_over_the_corpus_fans_back_out_to_the_triangle_batch():
    """Корпус стенда: 22 формы на двух alpha, сборка от разбиения (плоская укладка)."""

    quads_seen = 0
    for name, polygon in named_corpus():
        for alpha in (Fraction(1), Fraction(3, 2)):
            left = factories.assemble_polygon_batch(polygon, alpha, law=TRIANGLES)
            right = factories.assemble_polygon_batch(polygon, alpha, law=QUADS)
            assert (left is None) == (right is None), (name, alpha)
            if left is None:
                continue
            batch, _frames = right
            assert validate_geometry_batch(batch) == (), (name, alpha)
            assert audit_batch(batch, NORMAL).problems() == (), (name, alpha)
            assert batch.vertices == left[0].vertices
            assert batch.semantic_digest == left[0].semantic_digest
            assert [_face_view(item) for item in fan_out_batch(batch).faces] == [
                _face_view(item) for item in left[0].faces
            ], (name, alpha)
            quads_seen += sum(1 for item in batch.faces if len(item.ordered_vert_keys) == 4)
    assert quads_seen > 0


# --------------------------------------------------------------------------
# Результат
# --------------------------------------------------------------------------

#: Золотые содержательные дайджесты закона лент для малых случаев (семантические
#: те же, что у треугольников, — они в `test_materialize_domain.GOLDEN`). Меняются
#: ТОЛЬКО осознанно: любое движение — смена состава граней закона.
GOLDEN_QUADS = {
    "weighted": "61d88c1adce5ad5537917e754c0c6ef01c3a7beb7e87fdbb3775f206f482d89d",
    "point_contact": "a18d06885f2ae09de58e54807fd921f3eef52f3b7797b7a6b4e9ff5c81815113",
    "two_edge": "d184f24e9f85eca087ac1404329acc8a777700757c601b53c7d815dcbf10b827",
    "straight3": "f9988c58176be1d6d0dacdc12aaa86efcfc12116c7d8db26aaf3405e0760e7c1",
}


@pytest.mark.parametrize("name", sorted(GOLDEN_QUADS))
def test_golden_content_digest_of_the_quad_law(name):
    _triangles, quads = _both_laws(name)
    assert quads.content_digest == GOLDEN_QUADS[name]


def test_the_quad_result_survives_a_pickle_and_records_its_law():
    _triangles, quads = _both_laws("full_selection")
    restored = pickle.loads(pickle.dumps(quads))
    assert restored == quads and restored.decal_topology_law is QUADS


def test_a_refusal_records_the_law_that_was_asked_for():
    prepared, coverage, request = DOMAINS["skew"]()
    broken = dataclasses.replace(
        coverage, outcome=ConveyorOutcome.COVERAGE_DID_NOT_CLOSE
    )
    refused = materialize_domain(
        prepared,
        broken,
        request=dataclasses.replace(request, uv_policy_id=UV),
        decal_topology_law=QUADS,
    )
    assert refused.batch is None
    assert refused.decal_topology_law is QUADS


def test_the_quad_law_over_the_weighted_building_has_the_expected_shape():
    """Полевая стена: 8 треугольников — это 4 четырёхгранья, веера отсутствуют."""

    triangles, quads = _both_laws("weighted")
    assert len(triangles.batch.faces) == 8 and len(quads.batch.faces) == 4
    assert dict(quads.counters)["MATERIALIZE_QUADS"] == 4
