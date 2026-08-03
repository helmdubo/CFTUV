"""Полевой корпус смежности: одиннадцать снапшотов, оба вывода, один язык.

Модуль — не тест, а общий вход двух приёмок. Он делает ровно три вещи:

1. Находит одиннадцать полевых снапшотов и их `host_mesh.json`.
2. Восстанавливает хостовый `PatchSurfaceIR` из байтов снапшота. Восстановление
   законно, потому что идентичность в снапшоте ЯВНО несёт хостовый номер
   последним сегментом (`host-triangle:<revision>:<n>`), и обратная разборка
   ничего не выдумывает — она читает то, что записал экспортёр.
3. Отдаёт карты идентичности ровно теми объектами, что лежат в снапшоте,
   поэтому сравнение двух выводов сравнивает СМЕЖНОСТЬ, а не именование.
"""

from __future__ import annotations

import json
from pathlib import Path

import cftuv_envelope as kernel
from cftuv_envelope.contracts.surface import PatchSurfaceIRV1

from cftuv.surface_ir import (
    PatchSurfaceIR,
    SourceEdge,
    SourceFace,
    SourceRevision as HostSourceRevision,
    SourceVertex,
    SurfaceTriangle,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_ROOT = REPO_ROOT / "kernel" / "fixtures"

FIELD_SNAPSHOT_NAMES = (
    "building_002_point_contact_v1",
    "building_002_full_selection_v1",
    "building_002_weighted_normals_v1",
    "building_patch10_density4_v1",
    "sem_clb_02_lost_domains_v1/cases/building_001_single_edge_patch_000_named_outcome_v1",
    "sem_clb_02_lost_domains_v1/cases/building_002_single_edge_patch_000_named_outcome_v1",
    "sem_clb_02_lost_domains_v1/cases/building_003_single_edge_patch_000_named_outcome_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_001_lost_resolved_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_006_lost_resolved_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_011_lost_resolved_v1",
    "sem_clb_02_lost_domains_v1/cases/building_all_seams_patch_105_lost_resolved_v1",
)


def _host_number(identity) -> int:
    """Хостовый номер, записанный экспортёром последним сегментом id."""

    return int(str(identity.value).rsplit(":", 1)[1])


def field_snapshot_paths() -> tuple[Path, ...]:
    return tuple(FIXTURE_ROOT / name for name in FIELD_SNAPSHOT_NAMES)


def load_snapshot(folder: Path):
    return kernel.AnalysisSnapshotCodecV1.loads(
        (folder / "analysis_snapshot.json").read_bytes()
    )


def load_edge_face_counts(folder: Path):
    """`{номер ребра: сколько граней ПОЛНОГО меша его держат}` или `None`."""

    path = folder / "host_mesh.json"
    if not path.exists():
        return None
    mesh = json.loads(path.read_text(encoding="utf-8"))
    edge_by_pair = {
        frozenset(int(value) for value in edge["vertex_indices"]): int(edge["index"])
        for edge in mesh["edges"]
    }
    counts = {int(edge["index"]): 0 for edge in mesh["edges"]}
    for face in mesh["faces"]:
        cycle = [int(value) for value in face["vertex_cycle"]]
        for index, first in enumerate(cycle):
            pair = frozenset((first, cycle[(index + 1) % len(cycle)]))
            edge_index = edge_by_pair.get(pair)
            if edge_index is not None:
                counts[edge_index] += 1
    return counts


def kernel_identity(surface: PatchSurfaceIRV1):
    """Карты `хостовый номер -> id снапшота` для треугольников, вершин, рёбер."""

    triangles = {
        _host_number(item.triangle_id): item.triangle_id
        for item in surface.surface_triangles
    }
    vertices = {
        _host_number(item): item for item in surface.source_vertex_ids
    }
    edges = {_host_number(item.edge_id): item.edge_id for item in surface.source_edges}
    return triangles, vertices, edges


def host_patch_surface(snapshot) -> PatchSurfaceIR:
    """Хостовый `PatchSurfaceIR`, разобранный обратно из байтов снапшота."""

    surface = snapshot.surface_ir
    positions = {
        _host_number(item.vertex_id): item.position
        for item in snapshot.source_vertices
    }
    vertices = tuple(
        SourceVertex(
            vertex_id=number,
            position=(
                (0.0, 0.0, 0.0)
                if not hasattr(positions.get(number), "x")
                else (
                    positions[number].x,
                    positions[number].y,
                    positions[number].z,
                )
            ),
        )
        for number in sorted(_host_number(item) for item in surface.source_vertex_ids)
    )
    faces_by_number = {
        _host_number(face.face_id): face for face in surface.source_faces
    }
    edge_faces: dict[int, set[int]] = {}
    for number, face in faces_by_number.items():
        for edge in face.edge_cycle:
            edge_faces.setdefault(_host_number(edge), set()).add(number)
    edges_by_number = {
        _host_number(edge.edge_id): edge for edge in surface.source_edges
    }
    edges = tuple(
        SourceEdge(
            edge_id=number,
            vertex_ids=tuple(
                sorted(
                    (
                        _host_number(edges_by_number[number].vertex_a_id),
                        _host_number(edges_by_number[number].vertex_b_id),
                    )
                )
            ),
            source_face_ids=tuple(sorted(edge_faces.get(number, ()))),
        )
        for number in sorted(edges_by_number)
    )
    faces = tuple(
        SourceFace(
            face_id=number,
            patch_id=_host_number(faces_by_number[number].patch_id),
            vertex_cycle=tuple(
                _host_number(item) for item in faces_by_number[number].vertex_cycle
            ),
            edge_cycle=tuple(
                _host_number(item) for item in faces_by_number[number].edge_cycle
            ),
            polygon_normal=(
                faces_by_number[number].polygon_normal.x,
                faces_by_number[number].polygon_normal.y,
                faces_by_number[number].polygon_normal.z,
            ),
            triangle_ids=tuple(
                _host_number(item) for item in faces_by_number[number].triangle_ids
            ),
        )
        for number in sorted(faces_by_number)
    )
    triangles = tuple(
        SurfaceTriangle(
            triangle_id=_host_number(item.triangle_id),
            source_face_id=_host_number(item.source_face_id),
            vertex_ids=tuple(_host_number(value) for value in item.vertex_ids),
            # Обратный поворот экспортёра: он пишет `kernel[k] = host[k + 2]`
            # (`envelope_request_export.py`: `[2], [0], [1]`), значит
            # `host[j] = kernel[j + 1]`. Без поворота хостовый IR получил бы
            # ядерную нумерацию под хостовым именем.
            physical_edge_ids=tuple(
                None
                if item.physical_edge_ids[(ordinal + 1) % 3] is None
                else _host_number(item.physical_edge_ids[(ordinal + 1) % 3])
                for ordinal in range(3)
            ),
            triangle_normal=(
                item.triangle_normal.x,
                item.triangle_normal.y,
                item.triangle_normal.z,
            ),
        )
        for item in sorted(
            surface.surface_triangles,
            key=lambda entry: _host_number(entry.triangle_id),
        )
    )
    return PatchSurfaceIR(
        source_revision=HostSourceRevision(
            source_name=str(surface.source_revision.value), digest=""
        ),
        vertices=vertices,
        edges=edges,
        faces=faces,
        triangles=triangles,
    )


__all__ = (
    "FIELD_SNAPSHOT_NAMES",
    "FIXTURE_ROOT",
    "field_snapshot_paths",
    "host_patch_surface",
    "kernel_identity",
    "load_edge_face_counts",
    "load_snapshot",
)
