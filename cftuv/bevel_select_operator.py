"""Кнопка «Select Bevel Faces» (BEVEL_SELECT_FACES_V1): предложение выделения граней бевела перед нажатием по граням.

Что выделять, решают `bevel_faces.select_bevel_faces` (выделенных граней нет - весь меш: атрибут Face Strength All - точно; иначе
геометрия под своим именем; ничего - `BEVEL_SELECT_NONE_FOUND`, выделение не тронуто) и `bevel_seed.select_from_seeds` (кнопка «Grow Bevel», `grow`, BEVEL_SELECT_GROW_EXPLICIT_V1: выделенные
грани - семена, BEVEL_SELECT_SEED_V1: группа бевела, в которой они лежат, и ничего вне её; параметры - из ряда семени, не из «Max
Bevel Width»; семя не грань бевела - `BEVEL_SEED_NOT_BEVEL`, ни одного годного - выделение не тронуто); чтение и запись bmesh -
`bevel_faces_blender`. Владелец правит предложение руками, нажатие «Build Decal Mesh» по граням считает выделение правдой и не
меняется. Регистрирует `envelope_production_operator` (`_CLASSES`).
"""

from __future__ import annotations

import bmesh
import bpy
from bpy.props import BoolProperty, FloatProperty

from .bevel_faces import (
    BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS,
    BEVEL_SELECT_MAX_WIDTH,
    BEVEL_SELECT_NONE_FOUND,
    select_bevel_faces,
    selection_console_lines,
    seam_edges,
    selection_line,
)
from .bevel_faces_blender import apply_face_selection, read_mesh_view, read_view
from .bevel_seed import BEVEL_SEED_NOT_BEVEL, select_from_seeds


class HOTSPOTUV_OT_SelectBevelFaces(bpy.types.Operator):
    """BEVEL_SELECT_FACES_V1: предложение выделения граней бевела перед нажатием по граням; владелец правит его руками, нажатие считает выделение правдой."""

    bl_idname = "hotspotuv.select_bevel_faces"
    bl_label = "Select Bevel Faces"
    bl_description = (
        "Select the bevel faces (strips and nodes) of every mesh in Edit Mode: exact from Bevel Face Strength 'All', "
        "otherwise a named geometric guess. The whole mesh is scanned, the current selection is ignored; Shift-click extends it. "
        "Grow (the Grow Bevel button) instead takes the selected faces as seeds: only their bevel group is selected"
    )
    bl_options = {"REGISTER", "UNDO"}

    extend: BoolProperty(
        name="Extend",
        description="Add the bevel faces to the selection instead of replacing it",
        default=False, options={"SKIP_SAVE"},
    )
    grow: BoolProperty(
        name="Grow From Selection",
        description="BEVEL_SELECT_GROW_EXPLICIT_V1: the selected faces are seeds and only their bevel group is selected (the seeds stay "
                    "selected); off - the whole mesh is scanned and the selection is ignored",
        default=False, options={"SKIP_SAVE"},
    )
    max_width: FloatProperty(
        name="Max Bevel Width",
        description="Whole-mesh geometric guess only: the widest bevel band (scene units); a wider band of narrow faces is a wall or a "
                    "ledge. Grown from seeds, the width comes from the seed's own bevel",
        default=BEVEL_SELECT_MAX_WIDTH, min=1e-4, soft_max=1.0, subtype="DISTANCE",
    )

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == "MESH" and obj.mode == "EDIT"

    def invoke(self, context, event):
        if event.shift:
            self.extend = True
        return self.execute(context)

    def _select(self, obj):
        """Предложение для объекта: «Grow» - от выделенных граней (семян), иначе весь меш; режим явный (BEVEL_SELECT_GROW_EXPLICIT_V1)."""

        view = read_mesh_view(bmesh.from_edit_mesh(obj.data), obj.matrix_world)
        return select_from_seeds(view) if self.grow else select_bevel_faces(view, self.max_width)

    def execute(self, context):
        objects = [obj for obj in getattr(context, "objects_in_mode_unique_data", ()) if obj.type == "MESH"] or [context.active_object]
        found = [(obj, self._select(obj)) for obj in objects]
        total = [selection for _obj, selection in found if selection.faces]
        if total:
            context.tool_settings.mesh_select_mode = (False, False, True)
            for obj, selection in found:
                if selection.faces:  # BEVEL_SELECT_NONE_FOUND / BEVEL_SEED_NOT_BEVEL: выделение этого объекта не тронуто
                    apply_face_selection(bmesh.from_edit_mesh(obj.data), selection.faces | selection.seeds, self.extend)
                    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
        for obj, selection in found:
            for line in selection_console_lines(obj.name, selection):
                print(line, flush=True)
        strips = sum(len(selection.strip) for selection in total)
        nodes = sum(len(selection.node) for selection in total)
        seeds = sum(len(selection.seeds) for _obj, selection in found)
        line = (
            f"Select Bevel Faces{' (extend)' if self.extend and not self.grow else ''}{f' grown from {seeds} seeds' if self.grow else ''}: "
            f"{strips + nodes} faces ({strips} strip / {nodes} node) | "
            + "; ".join(selection_line(obj.name, selection) for obj, selection in found)
        )
        print(f"[CFTUV][BevelSelect] {line}", flush=True)
        guessed = any(
            selection.outcome in (BEVEL_SELECT_NONE_FOUND, BEVEL_SEED_NOT_BEVEL) or selection.source == BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS
            or any(name == BEVEL_SEED_NOT_BEVEL for name, _count in selection.events)
            for _obj, selection in found
        )
        self.report({"WARNING" if guessed else "INFO"}, line)
        return {"FINISHED"} if total else {"CANCELLED"}


class HOTSPOTUV_OT_SeamBevel(bpy.types.Operator):
    """BEVEL_SEAM_MARK_V1: швы разметки по выделенным граням бевела - граница выделения и разрез «узел | полоса»."""

    bl_idname = "hotspotuv.seam_bevel"
    bl_label = "Seam Bevel"
    bl_description = (
        "Mark UV seams around the selected bevel faces: the selection boundary (rails, node edges to the original faces) and every "
        "node | strip edge, so each node becomes its own patch and the rail seams meet the node seams at junction points. "
        "A single selected node gets seams on its whole boundary. Nodes and strips: exact from Bevel Face Strength 'All', "
        "otherwise a named geometric guess. Existing seams are kept"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == "MESH" and obj.mode == "EDIT"

    def execute(self, context):
        objects = [obj for obj in getattr(context, "objects_in_mode_unique_data", ()) if obj.type == "MESH"] or [context.active_object]
        parts, marked = [], 0
        for obj in objects:
            bm = bmesh.from_edit_mesh(obj.data)
            view = read_view(bm)
            if not view.selected:
                continue
            edges, split = seam_edges(view)
            bm.edges.ensure_lookup_table()
            added = [bm.edges[index] for index in sorted(edges) if not bm.edges[index].seam]
            for edge in added:
                edge.seam = True
            if added:
                bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
            marked += len(added)
            events = ", ".join(f"{name} {count}" for name, count in split.events)
            parts.append(
                f"{obj.name}: {len(edges)} seam edges ({len(added)} new), {len(split.strip)} strip / {len(split.node)} node"
                f"{f', {len(split.unresolved)} unresolved' if split.unresolved else ''} ({split.source}{f'; {events}' if events else ''})"
            )
        if not parts:
            self.report({"WARNING"}, "Seam Bevel: BEVEL_FACES_EMPTY_SELECTION - select the bevel faces (or a node) first")
            return {"CANCELLED"}
        line = f"Seam Bevel: {marked} new seam edges | " + "; ".join(parts)
        print(f"[CFTUV][BevelSeam] {line}", flush=True)
        self.report({"INFO"}, line)
        return {"FINISHED"}


__all__ = ("HOTSPOTUV_OT_SeamBevel", "HOTSPOTUV_OT_SelectBevelFaces")
