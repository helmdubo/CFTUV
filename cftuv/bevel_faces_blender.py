"""Нажатие «Build Decal Mesh» по выделенным граням бевела (BEVEL_FACES_INWARD_OUTWARD_V1): чтение bmesh и пакет анализа.

Решения принимает `bevel_faces` (чистый Python); здесь только Blender:
- локальный вид меша вокруг выделения (циклы граней, координаты, номера рёбер, атрибут Face Strength);
- пакет анализа ПОДМНОЖЕСТВА граней (`build_analysis_bundle(bm, scope, ...)`): грань вне анализа для него не существует, и граница
  выделения становится открытой границей меша (`_neighbor_for_side` даёт MESH_BORDER) - «как будто другой части нет»;
- разрез «узел | полоса» INWARD (BEVEL_NODE_STRIP_SPLIT_V1) - флаги шва на рёбрах разреза ТОЛЬКО на время анализа, в `finally`
  возвращаются прежние значения. Тот же приём, что у самого анализа (`analysis_classification._unwrap_patch_faces_for_loop_classification`
  гасит швы на время развёртки): копия bmesh не годится, потому что анализ многопетлевого патча разворачивает ИМЕННО edit-mesh объекта;
- отпечаток анализа - отпечаток меша плюс сторона, грани и разрез (`scope_digest`): кэши сессии не путают его с нажатием по рёбрам,
  а живая ширина сверяет с мешем ОТПЕЧАТОК МЕША (`mesh_digest` записи кнопки).
Путь по рёбрам сюда не заходит и не меняется.
"""

from __future__ import annotations

from dataclasses import dataclass

from .bevel_faces import (
    BEVEL_FACES_NO_SOURCE,
    FACE_STRENGTH_ATTRIBUTE,
    INWARD,
    OUTWARD,
    BevelFacesPlanV1,
    BevelFacesViewV1,
    plan_faces,
    scope_digest,
)
from .surface_ir import SourceRevision

FACE_SIDE_ITEMS = (
    (INWARD, "Inward", "Selected bevel faces: every strip and node is filled from its whole boundary inward, as if the other faces were absent"),
    (OUTWARD, "Outward", "Faces around the selection: the edges the selected bevel faces leave are the sources, as if the selection were absent"),
)


class BevelFacesRefused(RuntimeError):
    """Нажатие по граням отказано по имени (`outcome`): меш декали не тронут."""

    def __init__(self, outcome: str, detail: str):
        self.outcome = outcome
        super().__init__(f"{outcome}: {detail}")


def face_side_of(context, mesh_settings) -> str | None:
    """Сторона нажатия по граням либо `None` (путь по рёбрам): только режим выделения граней без режима рёбер."""

    mode = tuple(context.tool_settings.mesh_select_mode)
    if mode[1] or not mode[2]:
        return None
    return str(getattr(mesh_settings, "face_side", INWARD))


def read_view(bm) -> BevelFacesViewV1:
    """Выделенные грани, их соседи по рёбрам, координаты, номера рёбер выделенных граней и Face Strength выделенных граней."""

    bm.faces.ensure_lookup_table()
    selected = [face for face in bm.faces if face.select]
    faces: dict[int, tuple[int, ...]] = {}
    edge_ids = {}
    for face in selected:
        for edge in face.edges:
            edge_ids[frozenset(int(vertex.index) for vertex in edge.verts)] = int(edge.index)
            for linked in edge.link_faces:
                if int(linked.index) not in faces:
                    faces[int(linked.index)] = tuple(int(loop.vert.index) for loop in linked.loops)
    bm.verts.ensure_lookup_table()
    coords = {
        index: tuple(float(value) for value in bm.verts[index].co)
        for cycle in faces.values()
        for index in cycle
    }
    layer = bm.faces.layers.int.get(FACE_STRENGTH_ATTRIBUTE)
    strength = None if layer is None else {int(face.index): int(face[layer]) for face in selected}
    return BevelFacesViewV1(coords, faces, frozenset(int(face.index) for face in selected), edge_ids, strength)


@dataclass(frozen=True, slots=True)
class FaceBuildV1:
    """Нажатие по граням: план, отпечаток меша и отпечаток анализа."""

    plan: BevelFacesPlanV1
    mesh_revision: SourceRevision
    revision: SourceRevision

    def build_bundle(self, bm, obj):
        """Пакет анализа граней `scope`; рёбра разреза - швы только на время анализа."""

        from .analysis import build_analysis_bundle

        bm.edges.ensure_lookup_table()
        edges = [bm.edges[index] for index in sorted(self.plan.cut_edges)]
        previous = [bool(edge.seam) for edge in edges]
        try:
            for edge in edges:
                edge.seam = True
            return build_analysis_bundle(bm, self.plan.scope, obj, self.revision)
        finally:
            for edge, seam in zip(edges, previous):
                edge.seam = seam
            if edges and getattr(obj, "mode", "") == "EDIT":
                import bmesh

                bmesh.update_edit_mesh(obj.data)  # анализ многопетлевого патча сбрасывал edit-mesh с разрезом: сброс без него

    def sources(self, bundle) -> frozenset[int]:
        """Рёбра-источники: OUTWARD - рёбра, которые оставляет выделение; INWARD - вся граница каждого патча анализа."""

        if self.plan.side == OUTWARD:
            found = self.plan.outward_sources
        else:
            found = frozenset(
                int(edge_id)
                for node in bundle.patch_graph.nodes.values()
                for loop in node.boundary_loops
                for chain in loop.chains
                for edge_id in chain.edge_indices
            )
        if not found:
            raise BevelFacesRefused(BEVEL_FACES_NO_SOURCE, "the analysed faces have no boundary edge")
        return found


def plan_face_build(bm, obj, side: str) -> FaceBuildV1:
    """План нажатия, отпечаток меша (тот же, что у нажатия по рёбрам) и отпечаток анализа; отказ плана - `BevelFacesRefused`."""

    from .analysis_surface import source_revision_from_bmesh

    bm.faces.ensure_lookup_table()
    mesh_revision = source_revision_from_bmesh(bm, obj, tuple(face.index for face in bm.faces))
    plan = plan_faces(read_view(bm), side, (int(face.index) for face in bm.faces))
    if plan.refusal:
        raise BevelFacesRefused(plan.refusal, plan.detail)
    revision = SourceRevision(source_name=mesh_revision.source_name, digest=scope_digest(mesh_revision.digest, plan))
    return FaceBuildV1(plan, mesh_revision, revision)


def restore_face_selection(obj, face_ids) -> None:
    """Выделение граней владельца после анализа (анализ многопетлевого патча переставлял выделение)."""

    import bmesh
    import bpy

    if obj is None or obj.name not in bpy.data.objects or obj.mode != "EDIT":
        return
    wanted = {int(index) for index in face_ids}
    bm = bmesh.from_edit_mesh(obj.data)
    for element in (*bm.faces, *bm.edges, *bm.verts):
        element.select = False
    for face in bm.faces:
        if int(face.index) in wanted:
            face.select = True
    bm.select_flush_mode()
    bmesh.update_edit_mesh(obj.data)


__all__ = (
    "BevelFacesRefused",
    "FACE_SIDE_ITEMS",
    "FaceBuildV1",
    "face_side_of",
    "plan_face_build",
    "read_view",
    "restore_face_selection",
)
