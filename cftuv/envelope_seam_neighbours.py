"""Грани соседа шва у внутренних вершин цепей: факт хоста для закона `CHAIN_STATION_PLAN_V1`.

АДАПТЕР ОТОБРАЖАЕТ КОНТРАКТ (AGENTS.md): здесь ничего не решается и не чинится. Снапшот запроса несёт поверхность только патчей запроса
(`AnalysisBundleIdView`), а цепь шва двух патчей читают оба домена; решение по её внутренней вершине обязано видеть обе стороны, и ядро
берёт чужую сторону из `AnalysisSnapshotV1.seam_neighbour_faces`. Срез поверхности (`_PatchSurfaceIdView.neighbour_faces`) уже несёт грани
патчей вне запроса у вершин граней запроса; здесь из них берутся те, что касаются ВНУТРЕННЕЙ вершины цепи, чей сосед — патч.

Один и тот же отбор нужен дважды: снапшоту (`seam_neighbour_faces`) и лёгкому входу воркера (`narrowed_surface`): ключ содержимого домена
кодирует вход воркера целиком, и кольцо граней, которого снапшот не читает (грань патча, касающегося домена лишь вершиной), без отбора
меняло бы ключ домена от правки, которой его снапшот не видит.
"""

from __future__ import annotations

import importlib
from dataclasses import replace

from .model_enums import ChainNeighborKind


def interior_vertices(host_chains) -> frozenset:
    """Номера вершин хоста, у которых закон плана станций (`CHAIN_STATION_PLAN_V1`) может решать: внутренние вершины кусков цепей, чей
    сосед — патч (замкнутый кусок — все вершины), и стыки — вершины, где кончаются ровно два таких куска."""

    found: set = set()
    ends: dict = {}
    for record in host_chains:
        if record.chain.neighbor_kind is ChainNeighborKind.PATCH:
            vertices = record.canonical_vertex_ids
            found.update(vertices if record.chain.is_closed else vertices[1:-1])
            if not record.chain.is_closed:
                for vertex in {vertices[0], vertices[-1]}:
                    ends[vertex] = ends.get(vertex, 0) + 1
    found.update(vertex for vertex, count in ends.items() if count == 2)
    return frozenset(found)


def narrowed(ring, host_chains) -> tuple:
    """Грани кольца (`NeighbourFaceV1`), касающиеся внутренней вершины цепи шва, в порядке кольца."""

    interior = interior_vertices(host_chains)
    return tuple(face for face in ring if not interior.isdisjoint(face.vertex_cycle)) if interior else ()


def narrowed_surface(surface, host_chains):
    """Срез поверхности с кольцом, отобранным по цепям: то, что снапшот прочтёт, и ничего сверх."""

    ring = getattr(surface, "neighbour_faces", ())
    return surface if not ring else replace(surface, neighbour_faces=narrowed(ring, host_chains))


def seam_neighbour_faces(kernel, revision: str, analysis_bundle, host_chains) -> frozenset:
    """`frozenset[SeamNeighbourFaceV1]`: грани патчей вне среза у внутренних вершин цепей, граничащих с патчем.

    Срез без граней соседей (полная выгрузка, где патчи в снапшоте все) даёт пустое множество: чужая сторона там — `surface_ir`.
    """

    ring = narrowed(getattr(analysis_bundle.patch_surface, "neighbour_faces", ()), host_chains)
    if not ring:
        return frozenset()
    record_type = importlib.import_module("cftuv_envelope.contracts.analysis").SeamNeighbourFaceV1
    return frozenset(
        record_type(
            kernel.SourceFaceId(f"host-face:{revision}:{face.face_id}"),
            kernel.PatchId(f"host-patch:{revision}:{face.patch_id}"),
            tuple(kernel.SourceVertexId(f"host-vertex:{revision}:{vertex}") for vertex in face.vertex_cycle),
            tuple(kernel.LocalPoint3V1(*point) for point in face.positions),
        )
        for face in ring
    )
