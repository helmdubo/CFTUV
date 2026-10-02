"""SOURCE_EDGE_ZERO_LENGTH: ребро с совпавшими концами называется ДО любой ступени.

Полевой случай (`wall_noise_top`): вершины 6 и 19 лежат в одной точке и
соединены швом. Мир увидел это тремя разными отказами: нулевая площадь
треугольников (`NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE`, лестница метрики встала
и не дошла до `DEVELOPABLE`), нулевая касательная угла («incident support is
degenerate») и холостой веер. Все три верны и все три вводят в заблуждение:
чинить надо ребро. Снапшот называет его сам.

Допуск — ТОЧНОЕ равенство координат. «Короче шага решётки» — другой класс
(шаг известен только после выбора закона решётки) и здесь не ловится.
"""

from __future__ import annotations

from dataclasses import replace
from math import nextafter

import pytest

from cftuv_envelope import (
    ContractValidationError,
    LocalPoint3V1,
    SourceVertexV1,
    SurfaceCoordinateUnavailableReason,
    UnavailableSourcePositionV1,
    ValidationCode,
    validate_analysis_snapshot,
)
from cftuv_envelope.reference import ReferenceOutcome, compile_reference_envelopes
from cftuv_envelope.validation import raise_for_issues

from factories import full_host_snapshot
from reference_factories import angular_snapshot


def _first_edge(snapshot):
    return min(snapshot.surface_ir.source_edges, key=lambda item: str(item.edge_id))


def _coincide(snapshot, edge, *, nudge: bool = False):
    """Снапшот, где конец `b` ребра лежит там же, где `a` (или на один ulp рядом)."""

    by_id = {item.vertex_id: item.position for item in snapshot.source_vertices}
    target = by_id[edge.vertex_a_id]
    if nudge:
        target = LocalPoint3V1(nextafter(target.x, float("inf")), target.y, target.z)
    return replace(
        snapshot,
        source_vertices=frozenset(
            SourceVertexV1(item.vertex_id, target)
            if item.vertex_id == edge.vertex_b_id
            else item
            for item in snapshot.source_vertices
        ),
    )


def _zero_length(issues):
    return [item for item in issues if item.code is ValidationCode.SOURCE_EDGE_ZERO_LENGTH]


def test_a_clean_snapshot_has_no_zero_length_issue():
    assert validate_analysis_snapshot(full_host_snapshot()) == ()


def test_an_edge_with_coinciding_endpoints_is_named_first_with_its_vertices():
    snapshot = full_host_snapshot()
    edge = _first_edge(snapshot)

    issues = validate_analysis_snapshot(_coincide(snapshot, edge))

    found = _zero_length(issues)
    assert [item.path for item in found] == [
        ("surface_ir", "source_edges", str(edge.edge_id), "length")
    ]
    assert issues[0] is found[0], "the name leads: it is the cause of what follows"
    message = found[0].message
    assert message.startswith("SOURCE_EDGE_ZERO_LENGTH:")
    assert str(edge.vertex_a_id) in message and str(edge.vertex_b_id) in message
    assert "Merge by Distance" in message


def test_the_refusal_text_carries_the_name_in_every_rendering():
    snapshot = full_host_snapshot()
    broken = _coincide(snapshot, _first_edge(snapshot))
    with pytest.raises(ContractValidationError) as raised:
        raise_for_issues(validate_analysis_snapshot(broken))
    assert str(raised.value).startswith("SOURCE_EDGE_ZERO_LENGTH@surface_ir")


def test_one_ulp_apart_is_not_zero_length():
    """Допуск нулевой: «почти ноль» — другой класс, и эвристики тут нет."""

    snapshot = full_host_snapshot()
    nudged = _coincide(snapshot, _first_edge(snapshot), nudge=True)
    assert not _zero_length(validate_analysis_snapshot(nudged))


def test_a_coordinate_free_vertex_is_never_called_zero_length():
    snapshot = full_host_snapshot()
    edge = _first_edge(snapshot)
    hidden = UnavailableSourcePositionV1(
        next(iter(SurfaceCoordinateUnavailableReason))
    )
    blind = replace(
        snapshot,
        source_vertices=frozenset(
            SourceVertexV1(item.vertex_id, hidden)
            if item.vertex_id in {edge.vertex_a_id, edge.vertex_b_id}
            else item
            for item in snapshot.source_vertices
        ),
    )
    assert not _zero_length(validate_analysis_snapshot(blind))


def test_the_compile_refusal_names_the_edge_instead_of_a_downstream_stage():
    snapshot, request = angular_snapshot(0)
    assert compile_reference_envelopes(snapshot, request).outcome is ReferenceOutcome.EXACT

    compiled = compile_reference_envelopes(
        _coincide(snapshot, _first_edge(snapshot)), request
    )

    assert compiled.outcome is ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID
    assert "SOURCE_EDGE_ZERO_LENGTH" in compiled.diagnostics[0].message
