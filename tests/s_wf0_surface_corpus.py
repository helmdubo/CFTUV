"""Девятка S-WF0 как вход ядра: те же меши, их же кодом.

Меши строит `research/s_wf0/fixtures.py` — ровно тот код, которым спайк S-WF0
получил числа решения о курсе кривых поверхностей. Здесь они только
переводятся в контракты ядра: одна грань на треугольник, физические рёбра —
все рёбра меша, диагоналей тесселяции нет по построению (треугольная грань
своих диагоналей не имеет).

Почему это законный вход. Приёмка V2 проверяет метрику, а не экспортёр: ей
нужен меш с известной кривизной, известной границей и известной парой
ретриангуляций. Полевые снапшоты этого не дают — они планарны, и на них
конусный угол не отличил бы гладкую поверхность от плоской.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope.contracts.surface import (
    PatchSurfaceIRV1,
    SourceEdgeV1,
    SourceFaceV1,
    SurfacePayloadMode,
    SurfaceTriangleV1,
)
from cftuv_envelope.contracts.surface_adjacency import SurfaceAdjacencyScopeV1
from cftuv_envelope.ids import (
    PatchId,
    PhysicalEdgeId,
    SourceFaceId,
    SourceRevision,
    SourceVertexId,
    SurfaceTriangleId,
)
from cftuv_envelope.numeric import LocalVector3V1
from cftuv_envelope.surface_adjacency_compat import read_surface_adjacency


PATCH_SURFACE_IR_SCHEMA = "cftuv.envelope.patch_surface_ir.v1"


def load_fixtures():
    """Девятка S-WF0. Пропуск — только если research-зависимостей нет."""

    pytest.importorskip("numpy", reason="S-WF0 fixtures pin numpy")
    from research.s_wf0.fixtures import all_fixtures

    return all_fixtures()


def _identity(name: str):
    revision = SourceRevision(f"s-wf0:{name}")
    return revision, {
        "vertex": lambda index: SourceVertexId(f"s-wf0:{name}:vertex:{index}"),
        "edge": lambda index: PhysicalEdgeId(f"s-wf0:{name}:edge:{index}"),
        "face": lambda index: SourceFaceId(f"s-wf0:{name}:face:{index}"),
        "triangle": lambda index: SurfaceTriangleId(
            f"s-wf0:{name}:triangle:{index}"
        ),
    }


def _edge_index(vertices, first, second):
    return vertices[frozenset((first, second))]


def _edges_of(faces):
    pairs = {}
    for face in faces:
        for index in range(3):
            pair = frozenset((int(face[index]), int(face[(index + 1) % 3])))
            pairs.setdefault(pair, len(pairs))
    return pairs


def surface_ir_for(fixture) -> PatchSurfaceIRV1:
    """Треугольный меш фикстуры как `PatchSurfaceIRV1` без единой диагонали."""

    revision, mint = _identity(fixture.key.replace("/", "-"))
    faces = [tuple(int(value) for value in face) for face in fixture.faces]
    edge_index = _edges_of(faces)
    patch_id = PatchId(f"s-wf0:{fixture.key}")
    source_edges = frozenset(
        SourceEdgeV1(
            edge_id=mint["edge"](index),
            vertex_a_id=mint["vertex"](min(pair)),
            vertex_b_id=mint["vertex"](max(pair)),
        )
        for pair, index in edge_index.items()
    )
    triangles = []
    source_faces = []
    for number, face in enumerate(faces):
        triangle_id = mint["triangle"](number)
        vertex_ids = tuple(mint["vertex"](value) for value in face)
        edge_cycle = tuple(
            mint["edge"](
                _edge_index(edge_index, face[index], face[(index + 1) % 3])
            )
            for index in range(3)
        )
        triangles.append(
            SurfaceTriangleV1(
                triangle_id=triangle_id,
                source_face_id=mint["face"](number),
                vertex_ids=vertex_ids,
                # Ядерный закон: `physical_edge_ids[i]` описывает пару
                # `(v[i], v[i+1])` — ту же, что `edge_cycle[i]`.
                physical_edge_ids=edge_cycle,
                triangle_normal=LocalVector3V1(0.0, 0.0, 1.0),
            )
        )
        source_faces.append(
            SourceFaceV1(
                face_id=mint["face"](number),
                patch_id=patch_id,
                vertex_cycle=vertex_ids,
                edge_cycle=edge_cycle,
                polygon_normal=LocalVector3V1(0.0, 0.0, 1.0),
                triangle_ids=(triangle_id,),
            )
        )
    used = sorted({value for face in faces for value in face})
    return PatchSurfaceIRV1(
        schema_version=PATCH_SURFACE_IR_SCHEMA,
        source_revision=revision,
        payload_mode=SurfacePayloadMode.FULL_HOST_SURFACE,
        source_vertex_ids=frozenset(mint["vertex"](value) for value in used),
        source_edges=source_edges,
        source_faces=frozenset(source_faces),
        surface_triangles=frozenset(triangles),
    )


def positions_for(fixture) -> dict:
    """Точные рациональные позиции: `Fraction(float)` без единого округления."""

    revision, mint = _identity(fixture.key.replace("/", "-"))
    used = sorted({int(value) for face in fixture.faces for value in face})
    return {
        mint["vertex"](index): tuple(
            Fraction(float(component)) for component in fixture.vertices[index]
        )
        for index in used
    }


def adjacency_for(fixture, surface):
    """Смежность девятки выводится совместимым читателем: меши ПОЛНЫЕ."""

    return read_surface_adjacency(
        surface, scope=SurfaceAdjacencyScopeV1.FULL_SOURCE_MESH
    )


__all__ = (
    "adjacency_for",
    "load_fixtures",
    "positions_for",
    "surface_ir_for",
)
