"""Общее для проб GP_RENDER: аддон из рабочего дерева и выделение швов."""

from __future__ import annotations

import sys
from pathlib import Path

import addon_utils
import bmesh
import bpy


def bootstrap(worktree: Path):
    """Аддон из рабочего дерева: установленная копия отключается и забывается."""

    if "cftuv" in bpy.context.preferences.addons:
        addon_utils.disable("cftuv", default_set=False)
    for name in tuple(sys.modules):
        if name == "cftuv" or name.startswith("cftuv.") or name.startswith(
            "cftuv_envelope"
        ):
            del sys.modules[name]
    for path in (worktree, worktree / "kernel" / "src"):
        if str(path) in sys.path:
            sys.path.remove(str(path))
        sys.path.insert(0, str(path))
    import cftuv

    assert Path(cftuv.__file__).resolve().is_relative_to(worktree), cftuv.__file__
    cftuv.register()
    return cftuv


def select_seams(obj):
    """Все швы меша в edit mode, как `tools/blender_field_sweep.py`."""

    bpy.context.view_layer.objects.active = obj
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    wanted = 0
    for edge in bm.edges:
        edge.select_set(False)
    for edge in bm.edges:
        if edge.seam:
            edge.select_set(True)
            wanted += 1
    bmesh.update_edit_mesh(obj.data)
    return wanted
