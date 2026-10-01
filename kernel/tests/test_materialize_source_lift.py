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
from cftuv_envelope.ids import PolicyId
from cftuv_envelope.materialize import domain
from cftuv_envelope.materialize.domain import materialize_domain
from cftuv_envelope.materialize.source_lift import (
    DISPLACED,
    LIFTED,
    ORIENTATION_KEPT,
    QUAD_OFF_PLANE,
    SOURCE_VERTEX_LIFT_BUDGET_CELLS,
    UNAVAILABLE,
    lift_source_vertices,
)
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


def test_a_thin_face_the_host_position_would_turn_over_keeps_its_nodes():
    """Сливер высотой в доли ячейки: подъём вершины в хостовую позицию его перевернул бы."""

    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "src:c": _point(0.5, 0.00001),
    }
    host = {"c": _point(0.5, -0.00002)}
    triangle = ("node:0", "node:1", "src:c")

    result = lift_source_vertices(positions, [triangle], host, STEP)

    assert result.positions == positions
    assert (result.lifted, result.kept_for_orientation) == (0, 1)
    assert dict(result.counters())[ORIENTATION_KEPT] == 1
    assert "turn a face contour over" in result.orientation_note()


def test_a_reverted_vertex_is_rechecked_against_its_neighbours_to_a_fixed_point():
    """Откат вершины меняет соседние грани: проверка идёт, пока что-то откатывается."""

    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "src:a": _point(0.5, 0.00001),
        "src:b": _point(0.5, -1.0),
    }
    host = {"a": _point(0.5, -0.00002), "b": _point(0.5, -1.0 - 0.01)}
    faces = [("node:0", "node:1", "src:a"), ("node:1", "node:0", "src:b")]

    result = lift_source_vertices(positions, faces, host, STEP)

    assert result.kept_for_orientation >= 1
    assert result.positions["src:a"] == positions["src:a"]
    for first, second, third in faces:
        before = _area_z(positions[first], positions[second], positions[third])
        after = _area_z(*(result.positions[key] for key in (first, second, third)))
        assert before * after > 0.0, (first, second, third)


def _area_z(a, b, c):
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)


def test_a_quad_with_a_lifted_vertex_reports_its_off_plane_deviation():
    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "node:2": _point(1.0, 1.0),
        "src:q": _point(0.0, 1.0),
    }
    host = {"q": _point(0.0, 1.0, 0.0125)}
    quad = ("node:0", "node:1", "node:2", "src:q")

    result = lift_source_vertices(positions, [quad], host, STEP)

    assert result.lifted == 1
    assert result.max_quad_deviation == pytest.approx(0.0125)
    assert dict(result.counters())[QUAD_OFF_PLANE] == round(result.max_quad_deviation * 10**9)
    assert dict(result.counters())[QUAD_OFF_PLANE] > 0


def test_the_positions_do_not_depend_on_the_topology_law():
    """Четырёхгранье и его два канонических треугольника дают ОДНИ позиции."""

    positions = {
        "node:0": _point(0.0),
        "node:1": _point(1.0),
        "node:2": _point(1.0, 1.0),
        "src:q": _point(0.0, 1.0),
        "src:t": _point(0.5, 0.00001),
    }
    host = {"q": _point(0.003, 1.004), "t": _point(0.5, -0.00002)}
    quad = ("node:0", "node:1", "node:2", "src:q")
    triangles = (("src:q", "node:0", "node:1"), ("node:1", "node:2", "src:q"))
    sliver = ("node:0", "node:1", "src:t")

    as_quad = lift_source_vertices(positions, [quad, sliver], host, STEP)
    as_triangles = lift_source_vertices(positions, [*triangles, sliver], host, STEP)

    assert as_quad.positions == as_triangles.positions
    assert as_quad.kept_for_orientation == as_triangles.kept_for_orientation


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
    _prepared, triangles = _materialize(name)
    _prepared, quads = _materialize(name, DecalTopologyLawV1.QUAD_STRIPS_V1)

    assert triangles.batch.vertices == quads.batch.vertices
    assert dict(triangles.counters)[LIFTED] == dict(quads.counters)[LIFTED]
    assert dict(triangles.counters)[ORIENTATION_KEPT] == dict(quads.counters)[ORIENTATION_KEPT]
    assert dict(triangles.counters)[QUAD_OFF_PLANE] == 0


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
