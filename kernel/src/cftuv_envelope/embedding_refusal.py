"""Отказы вложения (привязка источника и проекция): строка для консоли и полная запись одним текстом.

Прежде отказ нёс `repr` сертификата целиком: на патче в 13 вершин это 2.2 КБ строки вида
`SourceSnapEmbeddingCertificateV1(snapping_law=<...>, source_vertex_ids=(SourceVertexId(value='host-vertex:host-source:e65179...:
cover.008:100'), ...`, и из неё нельзя было прочесть, ЧТО именно сломалось. Теперь текст отказа — две части:

* ПЕРВАЯ СТРОКА (не длиннее `COMPACT_LINE_LIMIT` = 240 знаков) называет исход, счёт и ПЕРВУЮ находку терминами художника:
  `SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION: 4 contacts after snap to 2^-12; first edge 100-359 × 127-128, 6.8e-06 m apart`.
  Вершины названы номерами хоста (последний знак после `:` в идентичности вершины ядра — номер BMesh), рёбра — парами вершин;
* ПОЛНАЯ ЗАПИСЬ — после первой строки, с маркером `record: ` и `repr` сертификата без потерь. Консоль печатает только первую
  строку (`envelope_production_report.console_detail`), а JSON-свидетельство, квитанция и расписка несут текст целиком.

Первая находка ищется ТЕМ ЖЕ предикатом и в том же порядке, каким сертификат (`_embedding`) считает счёт: обход пар и рёбер
повторён, а не придуман заново, поэтому «первая» — это первая из тех, что вошли в счёт. Ничего не чинится и не скрывается:
число в строке — число сертификата.
"""

from __future__ import annotations

import math

from ._embedding import (
    _NONE,
    _corner_degenerated,
    _nonadjacent_pairs,
    _segment_relation3,
    _source_edge_occurrences,
)
from .outcomes import NamedOutcome
from .source_defects import segment_distance_squared

#: Длина первой строки отказа, не больше; хост печатает в консоль ровно её (тот же предел в `envelope_production_report`).
COMPACT_LINE_LIMIT = 240

RECORD_MARKER = "record: "


def short_name(identifier) -> str:
    """Номер хоста из идентичности ядра: всё после последнего `:` (`host-vertex:<ревизия>:133` -> `133`)."""

    return str(getattr(identifier, "value", identifier)).rsplit(":", 1)[-1]


def _metres(squared) -> str:
    return f"{math.sqrt(float(squared)):.2g}"


def _distance_squared(left, right):
    return sum((a - b) * (a - b) for a, b in zip(left, right))


def _grid_step(scale) -> str:
    if scale and scale & (scale - 1) == 0:
        return f"2^-{scale.bit_length() - 1}"
    return f"1/{scale}" if scale else "the source grid"


def _count(number: int, noun: str) -> str:
    return f"{number} {noun}" + ("" if number == 1 else "s")


def _edge_name(edge) -> str:
    return f"{short_name(edge.start)}-{short_name(edge.end)}"


def refusal_message(outcome, summary: str, record) -> str:
    """Первая строка `ИСХОД: сводка` (обрезана до предела) и `record: <repr>` второй строкой."""

    line = f"{outcome.value}: {summary}"
    if len(line) > COMPACT_LINE_LIMIT:
        line = line[: COMPACT_LINE_LIMIT - 1] + "…"
    return f"{line}\n{RECORD_MARKER}{record!r}"


def _first_merged_pair(before, after):
    ordered = sorted(before, key=lambda item: item.value)
    for index, left in enumerate(ordered):
        for right in ordered[index + 1 :]:
            if before[left] != before[right] and after[left] == after[right]:
                return left, right
    return None


def _first_collapsed_edge(before, after, faces):
    for edge in _source_edge_occurrences(faces):
        if before[edge.start] != before[edge.end] and after[edge.start] == after[edge.end]:
            return edge
    return None


def _first_new_contact(before, after, faces):
    for left, right in _nonadjacent_pairs(_source_edge_occurrences(faces)):
        prior = _segment_relation3(before[left.start], before[left.end], before[right.start], before[right.end])
        snapped = _segment_relation3(after[left.start], after[left.end], after[right.start], after[right.end])
        if prior == _NONE and snapped != _NONE:
            return left, right
    return None


def source_snap_refusal(outcome, embedding, *, before, after, faces, corners, scale) -> str:
    """Текст отказа привязки источника: исход, счёт сертификата, первая находка и расстояние до привязки (метры)."""

    where = f"after snap to {_grid_step(scale)}"
    first = "first: not located"
    if outcome is NamedOutcome.SOURCE_SNAP_VERTEX_INJECTIVITY_VIOLATED:
        pair = _first_merged_pair(before, after)
        if pair is not None:
            first = f"first {short_name(pair[0])} and {short_name(pair[1])}, {_metres(_distance_squared(before[pair[0]], before[pair[1]]))} m apart"
        summary = f"{_count(embedding.newly_coincident_vertex_pair_count, 'vertex pair')} merged {where}; {first}"
    elif outcome is NamedOutcome.SOURCE_SNAP_NONZERO_EDGE_COLLAPSED:
        edge = _first_collapsed_edge(before, after, faces)
        if edge is not None:
            first = f"first edge {_edge_name(edge)}, {_metres(_distance_squared(before[edge.start], before[edge.end]))} m long"
        summary = f"{_count(embedding.collapsed_nonzero_source_edge_count, 'edge')} collapsed {where}; {first}"
    elif outcome is NamedOutcome.SOURCE_SNAP_NEW_NONADJACENT_EDGE_INTERSECTION:
        contact = _first_new_contact(before, after, faces)
        if contact is not None:
            left, right = contact
            gap = segment_distance_squared(before[left.start], before[left.end], before[right.start], before[right.end])
            first = f"first edge {_edge_name(left)} × {_edge_name(right)}, {_metres(gap)} m apart"
        summary = f"{_count(embedding.new_nonadjacent_edge_intersection_count, 'contact')} {where}; {first}"
    else:
        corner = next((item for item in corners if _corner_degenerated(after, item)), None)
        if corner is not None:
            first = f"first at vertex {short_name(corner[1])} ({short_name(corner[0])}-{short_name(corner[1])}-{short_name(corner[2])})"
        summary = f"{_count(embedding.degenerated_intended_right_corner_count, 'intended right corner')} degenerated {where}; {first}"
    return refusal_message(outcome, summary, embedding)


_PROJECTION_SUMMARIES = {
    NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_INJECTIVITY_VIOLATED: lambda c: f"{_count(c.coincident_boundary_occurrence_pair_count, 'boundary vertex pair')} coincided",
    NamedOutcome.NEAR_PLANAR_PROJECTION_NONZERO_BOUNDARY_EDGE_COLLAPSED: lambda c: f"{_count(c.collapsed_boundary_edge_occurrence_count, 'boundary edge')} collapsed",
    NamedOutcome.NEAR_PLANAR_PROJECTION_NEW_NONADJACENT_EDGE_INTERSECTION: lambda c: f"{_count(c.new_nonadjacent_edge_intersection_count, 'new edge contact')}",
    NamedOutcome.NEAR_PLANAR_PROJECTION_NEW_COLLINEAR_EDGE_OVERLAP: lambda c: f"{_count(c.new_nonadjacent_collinear_overlap_count, 'new collinear edge overlap')}",
    NamedOutcome.NEAR_PLANAR_PROJECTION_LOOP_ORIENTATION_CHANGED: lambda c: f"{_count(c.orientation_mismatch_count, 'loop')} changed orientation",
    NamedOutcome.NEAR_PLANAR_PROJECTION_BOUNDARY_COMPONENT_COUNT_CHANGED: lambda c: f"boundary components {c.source_boundary_component_count} -> {c.projected_boundary_component_count}",
    NamedOutcome.NEAR_PLANAR_PROJECTION_OUTER_HOLE_NESTING_CHANGED: lambda c: f"{_count(c.nesting_mismatch_count, 'loop')} changed outer/hole nesting",
    NamedOutcome.NEAR_PLANAR_PROJECTION_SOURCE_ANCHOR_IDENTITY_CHANGED: lambda c: "the anchor vertices of the chart changed",
    NamedOutcome.NEAR_PLANAR_PROJECTION_RESOLVED_PLANE_BASIS_UNAVAILABLE: lambda c: f"{_count(c.resolved_plane_basis_unavailable_count, 'face')} without a plane basis",
    NamedOutcome.NEAR_PLANAR_PROJECTION_FAN_IDENTITY_CHANGED: lambda c: "the order of faces around a vertex changed",
    NamedOutcome.NEAR_PLANAR_PROJECTION_VERTEX_INJECTIVITY_VIOLATED: lambda c: f"{_count(c.coincident_projected_vertex_pair_count, 'vertex pair')} coincided",
    NamedOutcome.NEAR_PLANAR_PROJECTION_FACE_POLYGON_NOT_SIMPLE: lambda c: f"{_count(c.nonsimple_projected_face_count, 'face')} crossed themselves",
    NamedOutcome.NEAR_PLANAR_PROJECTION_INTERIOR_OVERLAP: lambda c: f"{_count(c.overlapping_projected_triangle_pair_count, 'triangle pair')} overlapped",
}


def projection_refusal(outcome, embedding) -> str:
    """Текст отказа проекции на плоскость патча: исход и счёт сертификата (первую находку проекция не хранит)."""

    describe = _PROJECTION_SUMMARIES.get(outcome)
    what = "the embedding was not preserved" if describe is None else describe(embedding)
    return refusal_message(outcome, f"{what} after projection onto the patch plane", embedding)


__all__ = (
    "COMPACT_LINE_LIMIT",
    "RECORD_MARKER",
    "projection_refusal",
    "refusal_message",
    "short_name",
    "source_snap_refusal",
)
