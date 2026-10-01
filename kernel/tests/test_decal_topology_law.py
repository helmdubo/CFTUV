"""Закон топологии декали: `TRIANGLES_V1` и `QUAD_STRIPS_V1`.

Грань батча — многоугольник любой длины от трёх. Закон по умолчанию излучает
треугольники, как до оси; закон лент излучает строго выпуклые четырёхгранья.
Ворота обоих законов одни: равенство ПОСЛЕ разложения четырёхграней обратно в
треугольники (`fan_out`), то есть закон не меняет ни вершин, ни UV, ни
семантики — только то, как вершины собраны в грани.
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction

import pytest

from cftuv_envelope.contracts.geometry_batch import (
    DecalTopologyLawV1,
    GeometryFaceV1,
)
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize.admit import MaterializationOutcome
from cftuv_envelope.materialize.audit import audit_batch
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.tessellate import fan_out, triangulate_exact
from cftuv_envelope.validation import validate_geometry_batch
from cftuv_envelope.wavefront.faces import doubled_shoelace, orientation
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

import materialize_factories as factories

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
