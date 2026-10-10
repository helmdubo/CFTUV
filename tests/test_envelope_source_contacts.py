"""Предполёт контактов источника хоста: T-вершина и самопересечение грани названы номерами BMesh ДО ядра.

Геометрия патчей 18 и 285 полевого случая `cover.008` (замороженная копия `buildings2_2.blend`) взята из замера координат (девять знаков), а не
придумана: вершины 127 и 359 стоят в 6-7 мкм от рёбер соседней грани, грань 1497 — «бабочка» с рёбрами 2257-1438 и 1437-1520.
"""

from __future__ import annotations

import types
from dataclasses import replace

import pytest

import cftuv_envelope.embedding_refusal as kernel_text
from cftuv.envelope_host_outcomes import METRIC_STAGE_OUTCOMES, EnvelopeDebugHostOutcome
from cftuv.envelope_production_report import CONSOLE_DETAIL_LIMIT, console_detail, production_console_lines
from cftuv.envelope_request_export import EnvelopeHostAdapterError, build_envelope_analysis_snapshot
from cftuv.envelope_source_contacts import patch_contact_refusal, source_contact_refusal
from cftuv.surface_ir import AnalysisBundle
from test_envelope_host_adapter import _single_patch_bundle

PATCH_18 = {
    100: (-4.022137642, 22.162029266, 4.026507854),
    101: (-4.022140503, 22.162029266, 0.644262493),
    125: (4.375988007, 22.162031174, 7.589482307),
    126: (3.128871918, 22.162029266, 7.00512886),
    127: (3.128871918, 22.162027359, 4.026504993),
    128: (4.375988007, 22.162029266, 4.02650547),
    359: (3.532581329, 22.162021637, 4.026502609),
    362: (3.532577515, 22.162021637, 0.644257665),
}
PATCH_18_FACES = {180: (359, 362, 101, 100), 54: (128, 127, 126, 125)}

PATCH_285 = {
    2257: (-19.472175598, 23.8372612, 4.215972424),
    1438: (-12.104334831, 25.52388382, 4.215973377),
    1437: (-14.675764084, 25.52388382, 4.215973377),
    1520: (-18.853733063, 23.90820694, 4.215972424),
}


def _surface(positions, faces_by_patch):
    faces = tuple(
        types.SimpleNamespace(face_id=face, patch_id=patch, vertex_cycle=cycle)
        for patch, cycles in faces_by_patch.items()
        for face, cycle in cycles.items()
    )
    vertices = tuple(types.SimpleNamespace(vertex_id=number, position=point) for number, point in positions.items())
    return types.SimpleNamespace(faces=faces, vertices=vertices)


def test_patch_18_is_named_as_t_vertices_with_the_numbers_an_artist_sees_in_blender():
    refusal = patch_contact_refusal(_surface(PATCH_18, {18: PATCH_18_FACES}), 18)
    assert refusal.outcome is EnvelopeDebugHostOutcome.SOURCE_T_VERTEX
    first, record = refusal.message.split("\n", 1)
    assert first.startswith("SOURCE_T_VERTEX: vertex 127 lies 5.7 µm from edge 100-359 — merge or dissolve it"), first
    assert "(patch 18: 2 such vertices: 127, 359)" in first
    assert len(first) <= CONSOLE_DETAIL_LIMIT
    assert refusal.vertex_ids == (127, 359) and refusal.face_ids == () and refusal.patch_id == 18
    assert record.startswith("record: patch 18, extent 8.39813 m, contact gap 5.88e-05 m; 2 T-vertices, 0 face self-intersections")
    assert "vertex 127 lies 5.72e-06 m from edge 100-359, at 0.9466 of its length" in record, record


def test_patch_285_is_named_as_a_self_intersecting_face_with_its_edges():
    refusal = patch_contact_refusal(_surface(PATCH_285, {285: {1497: (2257, 1438, 1437, 1520)}}), 285)
    assert refusal.outcome is EnvelopeDebugHostOutcome.SOURCE_FACE_SELF_INTERSECTION
    first, record = refusal.message.split("\n", 1)
    assert first.startswith(
        "SOURCE_FACE_SELF_INTERSECTION: face 1497 crosses itself, its edges 2257-1438 and 1437-1520 pass "
    ), first
    assert first.endswith("µm apart — rebuild the face"), first
    assert refusal.face_ids == (1497,) and refusal.vertex_ids == (1437, 1438, 1520, 2257)
    assert "face 1497: edges 2257-1438 and 1437-1520 pass" in record


def test_a_clean_patch_and_the_neighbour_patch_of_a_defective_one_are_not_refused():
    surface = _surface({**PATCH_18, **PATCH_285}, {18: PATCH_18_FACES, 285: {1497: (2257, 1438, 1437, 1520)}})
    flat = {number: (float(number), 0.0, 0.0) for number in (1, 2, 3, 4)}
    flat[3], flat[4] = (2.0, 1.0, 0.0), (0.0, 1.0, 0.0)
    clean = _surface({**flat}, {7: {70: (1, 2, 3, 4)}})
    assert patch_contact_refusal(clean, 7) is None
    # Грань чужого патча в поверхности не вменяется патчу: у 285 нет рёбер патча 18.
    assert patch_contact_refusal(surface, 285).outcome is EnvelopeDebugHostOutcome.SOURCE_FACE_SELF_INTERSECTION
    assert patch_contact_refusal(surface, 18).outcome is EnvelopeDebugHostOutcome.SOURCE_T_VERTEX
    assert patch_contact_refusal(surface, 99) is None


def test_a_domain_set_is_refused_by_its_lowest_defective_patch_and_the_crossing_outranks_the_t_vertex():
    surface = _surface({**PATCH_18, **PATCH_285}, {18: PATCH_18_FACES, 285: {1497: (2257, 1438, 1437, 1520)}})
    assert source_contact_refusal(surface, [285, 18]).patch_id == 18
    assert source_contact_refusal(surface, {285}).patch_id == 285
    assert source_contact_refusal(surface, []) is None
    both = {**PATCH_18, **PATCH_285}
    mixed = _surface(both, {5: {**PATCH_18_FACES, 1497: (2257, 1438, 1437, 1520)}})
    assert patch_contact_refusal(mixed, 5).outcome is EnvelopeDebugHostOutcome.SOURCE_FACE_SELF_INTERSECTION


def test_many_findings_stay_on_one_console_line_and_the_rest_goes_to_the_record():
    positions = {}
    faces = {}
    for index in range(12):
        base, y = index * 6, float(index) * 10
        positions[base], positions[base + 1], positions[base + 2] = (0.0, y, 0.0), (10.0, y, 0.0), (5.0, y, -1.0)
        positions[base + 3], positions[base + 4], positions[base + 5] = (5.0, y + 4e-06, 0.0), (4.0, y + 4e-06, 1.0), (6.0, y + 4e-06, 1.0)
        faces[index * 2], faces[index * 2 + 1] = (base, base + 1, base + 2), (base + 3, base + 4, base + 5)
    refusal = patch_contact_refusal(_surface(positions, {3: faces}), 3)
    first, record = refusal.message.split("\n", 1)
    assert refusal.outcome is EnvelopeDebugHostOutcome.SOURCE_T_VERTEX
    assert ", ...)" in first and len(first) <= CONSOLE_DETAIL_LIMIT, first
    assert record.count(" lies ") == 12 and len(refusal.vertex_ids) == 12


def test_the_hook_refuses_a_request_scoped_export_by_name_and_leaves_the_full_export_alone(monkeypatch):
    bundle = _single_patch_bundle()
    swapped = tuple(
        replace(vertex, position=bundle.patch_surface.vertices[5 - vertex.vertex_id].position) if vertex.vertex_id in (2, 3) else vertex
        for vertex in bundle.patch_surface.vertices
    )
    bowtie = AnalysisBundle(bundle.source_revision, bundle.patch_graph, replace(bundle.patch_surface, vertices=swapped), bundle.capabilities)
    with pytest.raises(EnvelopeHostAdapterError) as raised:
        build_envelope_analysis_snapshot(bowtie, included_patch_ids=frozenset({0}))
    assert raised.value.outcome is EnvelopeDebugHostOutcome.SOURCE_FACE_SELF_INTERSECTION
    assert raised.value.outcome in METRIC_STAGE_OUTCOMES
    assert raised.value.patch_domain_id and "patch-domain" in raised.value.patch_domain_id
    assert str(raised.value).startswith("SOURCE_FACE_SELF_INTERSECTION: face 0 crosses itself")

    assert build_envelope_analysis_snapshot(bundle, included_patch_ids=frozenset({0})) is not None
    monkeypatch.setattr(
        "cftuv.envelope_request_export.source_contact_refusal",
        lambda *_a, **_k: pytest.fail("the full-bundle export must not run the per-domain preflight"),
    )
    assert build_envelope_analysis_snapshot(bundle) is not None


def test_the_console_prints_the_first_line_of_a_refusal_and_the_receipt_keeps_the_whole_text():
    refusal = patch_contact_refusal(_surface(PATCH_18, {18: PATCH_18_FACES}), 18)
    result = types.SimpleNamespace(
        patch_id=18, domain_id="host-v0:patch-domain:2e67fe", outcome=refusal.outcome.value, detail=refusal.message, is_materialized=False
    )
    refused_line = production_console_lines([result])[0]
    first_line = refusal.message.split("\n", 1)[0]
    assert refused_line.endswith(first_line.removeprefix("SOURCE_T_VERTEX: ")) and "\n" not in refused_line
    assert refused_line.count("SOURCE_T_VERTEX") == 1, refused_line
    assert "record:" not in refused_line and "record:" in result.detail
    assert len(console_detail("x" * 500)) == CONSOLE_DETAIL_LIMIT and console_detail("x" * 500).endswith("…")
    assert console_detail("one\ntwo") == "one"
    assert console_detail("NAME: text", "NAME") == "text" and console_detail("NAMES: text", "NAME") == "NAMES: text"
    assert CONSOLE_DETAIL_LIMIT == kernel_text.COMPACT_LINE_LIMIT == 240
