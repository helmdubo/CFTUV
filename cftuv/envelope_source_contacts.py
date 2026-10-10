"""Предполёт контактов источника: T-вершина и самопересечение грани названы ДО ядра, номерами BMesh.

Полевой случай `cover.008` (замороженная копия `buildings2_2.blend`): патч 18 и патч 285 отказывали ядру отказом привязки
к решётке, а причина лежала в источнике. У патча 18 вершины 127, 133 и 359 стоят в 6-7 мкм от рёбер, которым не принадлежат
(две грани делят полметра общей границы без общих вершин); у патча 285 грань 1497 — «бабочка»: её рёбра 2257-1438 и 1437-1520
пересекаются в плане и разведены по высоте на 4e-08 м. Привязка превращает такой зазор в контакт, и отказ приходил как счёт
«4 контакта после привязки» там, где чинить надо две вершины, а не решётку.

Адаптер ОТОБРАЖАЕТ контракт (AGENTS.md) и геометрию не чинит: хост передаёт ядру позиции и циклы граней патча, ядро находит
контакты (`cftuv_envelope.source_defects`, зазор — запись реестра `SOURCE_CONTACT_GAP_V1`, AUTHORING_INTENT), а здесь они
переводятся в именованный отказ домена с номерами вершин, рёбер и граней, которые художник видит в Blender. Ничего не
сваривается, не двигается и не выделяется: исправляет владелец (`Merge by Distance`, растворить вершину, пересобрать грань).

Область — патч домена (его грани): контакты с соседними патчами идут через швы и ядру не мешают. Отказ один на домен:
самопересечение грани — первым (оно тяжелее), иначе T-вершина; полный перечень находок лежит в записи (вторая и следующие
строки текста), консоль печатает только первую строку (`envelope_production_report.console_detail`).

Модуль — лист хоста без Blender: поверхность — любой объект с `faces` (`patch_id`, `face_id`, `vertex_cycle`) и `vertices`
(`vertex_id`, `position`), ядро берётся лениво.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass

from .envelope_host_outcomes import EnvelopeDebugHostOutcome

#: Сколько номеров вершин (граней) перечисляется в первой строке; остальные — в записи.
LISTED_IN_THE_FIRST_LINE = 4


@dataclass(frozen=True, slots=True)
class SourceContactRefusalV1:
    """Отказ предполёта: исход, текст (первая строка + запись), патч и номера, которые художник найдёт в Blender."""

    outcome: EnvelopeDebugHostOutcome
    message: str
    patch_id: int
    vertex_ids: tuple
    face_ids: tuple


def _length(metres: float) -> str:
    if metres < 1e-3:
        return f"{metres * 1e6:.2g} µm"
    return f"{metres * 1e3:.2g} mm"


def _edge(edge) -> str:
    return f"{edge[0]}-{edge[1]}"


def _listed(numbers) -> str:
    shown = ", ".join(str(item) for item in numbers[:LISTED_IN_THE_FIRST_LINE])
    return shown + (", ..." if len(numbers) > LISTED_IN_THE_FIRST_LINE else "")


def _distance(squared) -> float:
    return float(squared) ** 0.5


def _record(patch_id, found, lines) -> str:
    head = (
        f"record: patch {patch_id}, extent {float(found.extent):.6g} m, contact gap {float(found.gap):.3g} m; "
        f"{len(found.t_vertices)} T-vertices, {len(found.face_crossings)} face self-intersections"
    )
    return "\n".join([head, *lines])


def _t_vertex_refusal(patch_id, found) -> SourceContactRefusalV1:
    nearest = found.t_vertices[0]
    vertices = tuple(sorted({item.vertex for item in found.t_vertices}))
    first = (
        f"{EnvelopeDebugHostOutcome.SOURCE_T_VERTEX.value}: vertex {nearest.vertex} lies {_length(_distance(nearest.distance_squared))} "
        f"from edge {_edge(nearest.edge)} — merge or dissolve it"
    )
    if len(vertices) > 1:
        first += f" (patch {patch_id}: {len(vertices)} such vertices: {_listed(vertices)})"
    lines = [
        f"  vertex {item.vertex} lies {_distance(item.distance_squared):.3g} m from edge {_edge(item.edge)}, at {float(item.parameter):.4f} of its length"
        for item in found.t_vertices
    ]
    return SourceContactRefusalV1(
        EnvelopeDebugHostOutcome.SOURCE_T_VERTEX, first + "\n" + _record(patch_id, found, lines), patch_id, vertices, ()
    )


def _crossing_refusal(patch_id, found) -> SourceContactRefusalV1:
    nearest = found.face_crossings[0]
    faces = tuple(sorted({item.face for item in found.face_crossings}))
    left, right = nearest.edges
    first = (
        f"{EnvelopeDebugHostOutcome.SOURCE_FACE_SELF_INTERSECTION.value}: face {nearest.face} crosses itself, its edges "
        f"{_edge(left)} and {_edge(right)} pass {_length(_distance(nearest.distance_squared))} apart — rebuild the face"
    )
    if len(faces) > 1:
        first += f" (patch {patch_id}: {len(faces)} such faces: {_listed(faces)})"
    lines = [
        f"  face {item.face}: edges {_edge(item.edges[0])} and {_edge(item.edges[1])} pass {_distance(item.distance_squared):.3g} m apart"
        for item in found.face_crossings
    ]
    vertices = tuple(sorted({vertex for item in found.face_crossings for edge in item.edges for vertex in edge}))
    return SourceContactRefusalV1(
        EnvelopeDebugHostOutcome.SOURCE_FACE_SELF_INTERSECTION, first + "\n" + _record(patch_id, found, lines), patch_id, vertices, faces
    )


def patch_contact_refusal(surface, patch_id: int):
    """Отказ по ОДНОМУ патчу либо `None`: грани патча в ядро, находки — в именованный исход."""

    faces = [(int(item.face_id), tuple(int(v) for v in item.vertex_cycle)) for item in surface.faces if int(item.patch_id) == int(patch_id)]
    if not faces:
        return None
    used = {vertex for _face, cycle in faces for vertex in cycle}
    positions = {int(item.vertex_id): item.position for item in surface.vertices if int(item.vertex_id) in used}
    if len(positions) != len(used):
        return None
    defects = importlib.import_module("cftuv_envelope.source_defects")
    grid = importlib.import_module("cftuv_envelope.source_grid")
    found = defects.find_source_contact_defects(positions, faces, extent=grid.source_extent(positions))
    if found.face_crossings:
        return _crossing_refusal(int(patch_id), found)
    if found.t_vertices:
        return _t_vertex_refusal(int(patch_id), found)
    return None


def source_contact_refusal(surface, patch_ids):
    """Первый (по номеру патча) отказ среди `patch_ids` либо `None`."""

    for patch_id in sorted(int(item) for item in patch_ids):
        refusal = patch_contact_refusal(surface, patch_id)
        if refusal is not None:
            return refusal
    return None


__all__ = (
    "LISTED_IN_THE_FIRST_LINE",
    "SourceContactRefusalV1",
    "patch_contact_refusal",
    "source_contact_refusal",
)
