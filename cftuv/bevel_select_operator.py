"""Кнопка «Select Bevel Faces» (BEVEL_SELECT_FACES_V1): предложение выделения граней бевела перед нажатием по граням.

Что выделять, решает `bevel_faces.select_bevel_faces` (атрибут Face Strength All - точно; иначе геометрия под своим именем; ничего -
`BEVEL_SELECT_NONE_FOUND`, выделение не тронуто); чтение и запись bmesh - `bevel_faces_blender`. Владелец правит предложение руками,
нажатие «Build Decal Mesh» по граням считает выделение правдой и не меняется. Регистрирует `envelope_production_operator` (`_CLASSES`).
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
    selection_line,
)
from .bevel_faces_blender import apply_face_selection, read_mesh_view


class HOTSPOTUV_OT_SelectBevelFaces(bpy.types.Operator):
    """BEVEL_SELECT_FACES_V1: предложение выделения граней бевела перед нажатием по граням; владелец правит его руками, нажатие считает выделение правдой."""

    bl_idname = "hotspotuv.select_bevel_faces"
    bl_label = "Select Bevel Faces"
    bl_description = (
        "Select the bevel faces (strips and nodes) of every mesh in Edit Mode: exact from Bevel Face Strength 'All', "
        "otherwise a named geometric guess. Shift-click extends the selection"
    )
    bl_options = {"REGISTER", "UNDO"}

    extend: BoolProperty(name="Extend", description="Add the bevel faces to the selection instead of replacing it", default=False, options={"SKIP_SAVE"})
    max_width: FloatProperty(
        name="Max Bevel Width",
        description="Geometric guess only: the widest bevel band (scene units); a wider band of narrow faces is a wall or a ledge",
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

    def execute(self, context):
        objects = [obj for obj in getattr(context, "objects_in_mode_unique_data", ()) if obj.type == "MESH"] or [context.active_object]
        found = [(obj, select_bevel_faces(read_mesh_view(bmesh.from_edit_mesh(obj.data), obj.matrix_world), self.max_width)) for obj in objects]
        total = [selection for _obj, selection in found if selection.faces]
        if total:
            context.tool_settings.mesh_select_mode = (False, False, True)
            for obj, selection in found:
                if selection.faces:  # BEVEL_SELECT_NONE_FOUND: выделение этого объекта не тронуто
                    apply_face_selection(bmesh.from_edit_mesh(obj.data), selection.faces, self.extend)
                    bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)
        for obj, selection in found:
            for line in selection_console_lines(obj.name, selection):
                print(line, flush=True)
        strips = sum(len(selection.strip) for selection in total)
        nodes = sum(len(selection.node) for selection in total)
        line = (
            f"Select Bevel Faces{' (extend)' if self.extend else ''}: {strips + nodes} faces ({strips} strip / {nodes} node) | "
            + "; ".join(selection_line(obj.name, selection) for obj, selection in found)
        )
        print(f"[CFTUV][BevelSelect] {line}", flush=True)
        guessed = any(selection.outcome == BEVEL_SELECT_NONE_FOUND or selection.source == BEVEL_SELECT_FACE_STRENGTH_AMBIGUOUS for _obj, selection in found)
        self.report({"WARNING" if guessed else "INFO"}, line)
        return {"FINISHED"} if total else {"CANCELLED"}


__all__ = ("HOTSPOTUV_OT_SelectBevelFaces",)
