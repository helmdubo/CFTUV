"""CFTUV Operators — Blender UI, operators, settings.

Thin wrappers only. No geometry logic here (max 5 lines math).
All heavy work delegated to analysis.py, solve.py, debug.py.
"""

import bpy
import bmesh
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)

from .analysis import (
    build_patch_graph,
    format_patch_graph_report,
    format_solver_input_preflight_report,
    validate_solver_input_mesh,
)
from .constants import GP_DEBUG_PREFIX
from .debug import apply_layer_visibility, clear_visualization, create_frontier_visualization, create_visualization
from .model import UVSettings
from .solve import (
    build_root_scaffold_map,
    build_solver_graph,
    execute_phase1_preview,
    format_root_scaffold_report,
    format_solve_plan_report,
    plan_solve_phase1,
)


# ============================================================
# UI SETTINGS
# ============================================================

class HOTSPOTUV_Settings(bpy.types.PropertyGroup):
    target_texel_density: IntProperty(
        name="Target Texel Density (px/m)", default=512, min=1
    )
    texture_size: IntProperty(name="Texture Size", default=2048, min=1)
    uv_scale: FloatProperty(name="Custom Scale Multiplier", default=1.0, min=0.0001)
    uv_range_limit: IntProperty(name="UV Range Limit (Tiles)", default=16, min=0)

    # Debug state
    dbg_active: BoolProperty(
        name="Analyze", default=False, description="Debug analysis mode"
    )
    dbg_source_object: StringProperty(name="Debug Source", default="")

    # Group toggles
    dbg_grp_patches: BoolProperty(name="Patches", default=True)
    dbg_grp_loops: BoolProperty(name="Loop Types", default=True)
    dbg_grp_overlay: BoolProperty(name="Overlay", default=True)

    # Patches group
    dbg_patches_wall: BoolProperty(name="Wall", default=True)
    dbg_patches_floor: BoolProperty(name="Floor", default=True)
    dbg_patches_slope: BoolProperty(name="Slope", default=True)

    # Loop Types group
    dbg_loops_chains: BoolProperty(name="Chains", default=True)
    dbg_loops_holes: BoolProperty(name="Holes", default=True)


# ============================================================
# OPERATOR HELPERS
# ============================================================

def _build_debug_settings(settings: HOTSPOTUV_Settings) -> dict:
    """Собирает dict debug visibility из panel toggles."""
    loops_visible = bool(settings.dbg_grp_loops)
    overlay_visible = bool(settings.dbg_grp_overlay)
    return {
        'patches_wall': bool(settings.dbg_grp_patches and settings.dbg_patches_wall),
        'patches_floor': bool(settings.dbg_grp_patches and settings.dbg_patches_floor),
        'patches_slope': bool(settings.dbg_grp_patches and settings.dbg_patches_slope),
        'loops_chains': bool(loops_visible and settings.dbg_loops_chains),
        'loops_boundary': loops_visible,
        'loops_holes': bool(loops_visible and settings.dbg_loops_holes),
        'overlay_basis': overlay_visible,
        'overlay_centers': overlay_visible,
        'frontier_path': True,
    }


def _print_console_report(title, lines, summary=None):
    """Печатает отчёт в System Console."""
    print('=' * 60)
    print(title)
    print('=' * 60)
    for line in lines:
        print(line)
    if summary:
        print('-' * 60)
        print(summary)
    print('=' * 60)


def _capture_face_selection(bm):
    """Сохраняет текущее выделение faces."""
    return [face.index for face in bm.faces if face.select]


def _restore_face_selection(bm, selected_indices):
    """Восстанавливает выделение faces."""
    selected_set = set(selected_indices)
    for face in bm.faces:
        face.select = face.index in selected_set


def _prepare_patch_graph(context, require_selection=True, validate_for_solver=False):
    """Prepare PatchGraph from the current context.

    Returns: (obj, bm, patch_graph, original_mode, selected_face_indices)
    """
    obj = context.active_object
    if obj is None or obj.type != 'MESH':
        raise ValueError('Select a mesh object')

    original_mode = obj.mode
    if obj.mode != 'EDIT':
        bpy.ops.object.mode_set(mode='EDIT')

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()

    selected_face_indices = _capture_face_selection(bm)

    try:
        if original_mode == 'OBJECT':
            face_indices = [face.index for face in bm.faces]
        else:
            face_indices = list(selected_face_indices)
            if require_selection and not face_indices:
                raise ValueError('No faces selected in Edit Mode')
            if not face_indices:
                face_indices = [face.index for face in bm.faces]

        if validate_for_solver:
            preflight = validate_solver_input_mesh(bm, face_indices)
            if not preflight.is_valid:
                lines, summary = format_solver_input_preflight_report(preflight, mesh_name=obj.name)
                _print_console_report('CFTUV Solver Preflight', lines, summary)
                raise ValueError(summary)

        patch_graph = build_patch_graph(bm, face_indices, obj)
        return obj, bm, patch_graph, original_mode, selected_face_indices
    except Exception:
        _restore_mode_and_selection(obj, original_mode, selected_face_indices)
        raise

def _restore_mode_and_selection(obj, original_mode, selected_face_indices):
    """Восстанавливает mode и выделение после операции."""
    if obj is None or obj.name not in bpy.data.objects:
        return

    if obj.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        _restore_face_selection(bm, selected_face_indices)
        bmesh.update_edit_mesh(obj.data)

    if original_mode == 'OBJECT' and obj.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')


def _build_solve_state(context):
    """Полный solve state: PatchGraph + SolverGraph + SolvePlan + UVSettings."""
    obj, bm, patch_graph, original_mode, sel = _prepare_patch_graph(
        context, require_selection=True, validate_for_solver=True
    )
    solver_graph = build_solver_graph(patch_graph)
    solve_plan = plan_solve_phase1(patch_graph, solver_graph)
    settings = UVSettings.from_blender_settings(context.scene.hotspotuv_settings)
    return obj, bm, patch_graph, solver_graph, solve_plan, settings, original_mode, sel


# ============================================================
# DEBUG MODE
# ============================================================

def _enter_debug_mode(context, obj):
    """Входит в debug mode: строит PatchGraph, создаёт GP visualization."""
    s = context.scene.hotspotuv_settings

    try:
        obj, bm, patch_graph, _om, _sel = _prepare_patch_graph(
            context, require_selection=not (obj.mode == 'OBJECT')
        )
    except ValueError:
        if obj.mode != 'EDIT':
            bpy.ops.object.mode_set(mode='EDIT')
        return None

    bpy.ops.object.mode_set(mode='OBJECT')

    dbg_settings = _build_debug_settings(s)
    gp_obj = create_visualization(patch_graph, obj, dbg_settings)

    obj.hide_viewport = True
    bpy.ops.object.select_all(action='DESELECT')
    gp_obj.select_set(True)
    context.view_layer.objects.active = gp_obj

    s.dbg_active = True
    s.dbg_source_object = obj.name

    lines, summary = format_patch_graph_report(patch_graph, mesh_name=obj.name)
    _print_console_report('CFTUV PatchGraph Analyze', lines, summary)

    # Scaffold + Frontier visualization поверх GP
    try:
        solver_graph = build_solver_graph(patch_graph)
        solve_plan = plan_solve_phase1(patch_graph, solver_graph)
        settings = UVSettings.from_blender_settings(s)
        scaffold_map = build_root_scaffold_map(patch_graph, solve_plan, settings.final_scale)
        scaffold_lines, scaffold_summary = format_root_scaffold_report(patch_graph, scaffold_map, mesh_name=obj.name)
        _print_console_report('CFTUV Scaffold (Analyze)', scaffold_lines, scaffold_summary)
        create_frontier_visualization(patch_graph, scaffold_map, obj, dbg_settings)
    except Exception as exc:
        print(f"[CFTUV][Analyze] Scaffold failed: {exc}")

    return summary


def _exit_debug_mode(context):
    """Выходит из debug mode: удаляет GP, показывает mesh."""
    s = context.scene.hotspotuv_settings
    source_name = s.dbg_source_object

    if context.active_object and context.active_object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    if source_name and source_name in bpy.data.objects:
        source_obj = bpy.data.objects[source_name]
        clear_visualization(source_obj)
        source_obj.hide_viewport = False
        bpy.ops.object.select_all(action='DESELECT')
        source_obj.select_set(True)
        context.view_layer.objects.active = source_obj

    s.dbg_active = False
    s.dbg_source_object = ""


# ============================================================
# ACTIVE OPERATORS
# ============================================================

class HOTSPOTUV_OT_DebugAnalysis(bpy.types.Operator):
    bl_idname = "hotspotuv.debug_analysis"
    bl_label = "Debug: Toggle Analysis"
    bl_description = "Toggle debug analysis mode ON/OFF"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        s = context.scene.hotspotuv_settings
        if s.dbg_active:
            return True
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        s = context.scene.hotspotuv_settings

        if s.dbg_active:
            _exit_debug_mode(context)
            self.report({"INFO"}, "Debug analysis OFF")
        else:
            obj = context.active_object
            if not obj or obj.type != 'MESH':
                self.report({"WARNING"}, "Select a mesh object")
                return {"CANCELLED"}
            try:
                report = _enter_debug_mode(context, obj)
            except Exception as exc:
                self.report({"ERROR"}, f"Debug analysis failed: {exc}")
                return {"CANCELLED"}
            if report is None:
                self.report({"WARNING"}, "No faces selected in Edit Mode")
                return {"CANCELLED"}
            self.report({"INFO"}, report)

        return {"FINISHED"}


class HOTSPOTUV_OT_DebugClear(bpy.types.Operator):
    bl_idname = "hotspotuv.debug_clear"
    bl_label = "Debug: Force Clear"
    bl_description = "Force remove all debug visualization"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        s = context.scene.hotspotuv_settings

        if context.active_object and context.active_object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        source_name = s.dbg_source_object
        if source_name and source_name in bpy.data.objects:
            source_obj = bpy.data.objects[source_name]
            clear_visualization(source_obj)
            source_obj.hide_viewport = False

        obj = context.active_object
        if obj:
            if obj.name.startswith(GP_DEBUG_PREFIX):
                src_name = obj.name[len(GP_DEBUG_PREFIX):]
                src = bpy.data.objects.get(src_name)
                if src:
                    clear_visualization(src)
                    src.hide_viewport = False
            elif obj.type == 'MESH':
                clear_visualization(obj)

        s.dbg_active = False
        s.dbg_source_object = ""
        self.report({"INFO"}, "Debug cleared")
        return {"FINISHED"}


class HOTSPOTUV_OT_DebugToggleLayer(bpy.types.Operator):
    """Toggle visibility of a GP debug layer."""
    bl_idname = "hotspotuv.debug_toggle_layer"
    bl_label = "Toggle Layer"
    bl_options = {"REGISTER", "UNDO"}

    layer_name: StringProperty()

    def execute(self, context):
        s = context.scene.hotspotuv_settings
        source_name = s.dbg_source_object
        if not source_name:
            return {"CANCELLED"}

        gp_name = GP_DEBUG_PREFIX + source_name
        gp_obj = bpy.data.objects.get(gp_name)
        if gp_obj is None or gp_obj.type != 'GPENCIL':
            return {"CANCELLED"}

        gp_data = gp_obj.data
        if self.layer_name in gp_data.layers:
            layer = gp_data.layers[self.layer_name]
            layer.hide = not layer.hide

        return {"FINISHED"}


class HOTSPOTUV_OT_DebugToggleGroup(bpy.types.Operator):
    """Toggle visibility of a debug layer group."""
    bl_idname = "hotspotuv.debug_toggle_group"
    bl_label = "Toggle Group"
    bl_options = {"REGISTER", "UNDO"}

    group_name: StringProperty()

    _GROUP_LAYERS = {
        'patches': ['Patches_WALL', 'Patches_FLOOR', 'Patches_SLOPE'],
        'loops': ['Loops_Chains', 'Loops_Boundary', 'Loops_Holes'],
        'overlay': ['Overlay_Basis', 'Overlay_Centers'],
    }

    def execute(self, context):
        s = context.scene.hotspotuv_settings
        source_name = s.dbg_source_object
        if not source_name:
            return {"CANCELLED"}

        prop_name = f"dbg_grp_{self.group_name}"
        new_val = not getattr(s, prop_name, True)
        setattr(s, prop_name, new_val)

        gp_name = GP_DEBUG_PREFIX + source_name
        gp_obj = bpy.data.objects.get(gp_name)
        if gp_obj is None or gp_obj.type != 'GPENCIL':
            return {"FINISHED"}

        gp_data = gp_obj.data
        for layer_name in self._GROUP_LAYERS.get(self.group_name, []):
            if layer_name in gp_data.layers:
                gp_data.layers[layer_name].hide = not new_val

        return {"FINISHED"}


class HOTSPOTUV_OT_FlowDebug(bpy.types.Operator):
    bl_idname = "hotspotuv.flow_debug"
    bl_label = "Flow Debug"
    bl_description = "Build SolverGraph + SolvePlan, print to System Console"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        try:
            obj, _bm, pg, sg, sp, _s, om, sel = _build_solve_state(context)
            lines, summary = format_solve_plan_report(pg, sg, sp, mesh_name=obj.name)
            _print_console_report('CFTUV Flow Debug', lines, summary)
            self.report({"INFO"}, summary)
            return {"FINISHED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Flow Debug failed: {exc}")
            return {"CANCELLED"}
        finally:
            if 'obj' in locals() and 'om' in locals() and 'sel' in locals():
                _restore_mode_and_selection(obj, om, sel)


class HOTSPOTUV_OT_ScaffoldDebug(bpy.types.Operator):
    bl_idname = "hotspotuv.scaffold_debug"
    bl_label = "Scaffold Debug"
    bl_description = "Build ScaffoldMap, print to System Console"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        try:
            obj, _bm, pg, _sg, sp, settings, om, sel = _build_solve_state(context)
            scaffold_map = build_root_scaffold_map(pg, sp, settings.final_scale)
            lines, summary = format_root_scaffold_report(pg, scaffold_map, mesh_name=obj.name)
            _print_console_report('CFTUV Scaffold Debug', lines, summary)

            # Frontier path visualization поверх существующего GP
            s = context.scene.hotspotuv_settings
            dbg_settings = _build_debug_settings(s)
            source_name = s.dbg_source_object
            if source_name and source_name in bpy.data.objects:
                create_frontier_visualization(pg, scaffold_map, bpy.data.objects[source_name], dbg_settings)

            self.report({"INFO"}, summary)
            return {"FINISHED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Scaffold Debug failed: {exc}")
            return {"CANCELLED"}
        finally:
            if 'obj' in locals() and 'om' in locals() and 'sel' in locals():
                _restore_mode_and_selection(obj, om, sel)


class HOTSPOTUV_OT_FrontierReplay(bpy.types.Operator):
    bl_idname = "hotspotuv.frontier_replay"
    bl_label = "Frontier Replay"
    bl_description = "Animate scaffold build step-by-step. Hides mesh, shows only frontier path"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        s = context.scene.hotspotuv_settings
        if s.dbg_active and s.dbg_source_object:
            return True
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        try:
            obj, _bm, pg, _sg, sp, settings, om, sel = _build_solve_state(context)
            scaffold_map = build_root_scaffold_map(pg, sp, settings.final_scale)

            # Frontier visualization
            s = context.scene.hotspotuv_settings
            dbg_settings = _build_debug_settings(s)
            source_name = s.dbg_source_object
            if source_name and source_name in bpy.data.objects:
                source_obj = bpy.data.objects[source_name]
                create_frontier_visualization(pg, scaffold_map, source_obj, dbg_settings)

                # Прячем mesh, показываем только GP
                source_obj.hide_set(True)

                # Прячем все GP слои кроме Frontier_Path
                gp_name = GP_DEBUG_PREFIX + source_name
                gp_obj = bpy.data.objects.get(gp_name)
                if gp_obj and gp_obj.type == 'GPENCIL':
                    for layer in gp_obj.data.layers:
                        layer.hide = (layer.info != 'Frontier_Path')

                # Запускаем playback
                context.scene.frame_current = 0
                bpy.ops.screen.animation_play()

            self.report({"INFO"}, "Frontier Replay started — press Esc to stop")
            return {"FINISHED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Frontier Replay failed: {exc}")
            return {"CANCELLED"}
        finally:
            if 'obj' in locals() and 'om' in locals() and 'sel' in locals():
                _restore_mode_and_selection(obj, om, sel)


class HOTSPOTUV_OT_SolvePhase1Preview(bpy.types.Operator):
    bl_idname = "hotspotuv.solve_phase1_preview"
    bl_label = "Solve Phase 1 Preview"
    bl_description = "Write ScaffoldMap-driven preview UVs"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        try:
            obj, bm, pg, _sg, sp, settings, om, sel = _build_solve_state(context)
            stats = execute_phase1_preview(context, obj, bm, pg, settings, sp)
            summary = (
                f"Phase1 quilts:{stats.get('quilts', 0)} "
                f"roots:{stats.get('supported_roots', 0)} "
                f"children:{stats.get('attached_children', 0)} "
                f"invalid:{stats.get('invalid_scaffold_patches', 0)} "
                f"unresolved:{stats.get('unresolved_scaffold_points', 0)} "
                f"missing:{stats.get('missing_uv_targets', 0)} "
                f"conflicts:{stats.get('conflicting_uv_targets', 0)}"
            )
            print(f"[CFTUV][Phase1] Summary: {summary}")
            self.report({"INFO"}, summary)
            return {"FINISHED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Solve Phase 1 Preview failed: {exc}")
            return {"CANCELLED"}
        finally:
            if 'obj' in locals() and 'om' in locals() and 'sel' in locals():
                _restore_mode_and_selection(obj, om, sel)


# ============================================================
# LEGACY OPERATORS — DISABLED
# Будут пересобраны на ScaffoldMap в Phase 5.
# ============================================================

class HOTSPOTUV_OT_UnwrapFaces(bpy.types.Operator):
    bl_idname = "hotspotuv.unwrap_faces"
    bl_label = "UV Unwrap Faces"
    bl_description = "[DISABLED] Pending rebuild on ScaffoldMap pipeline (Phase 5)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        self.report(
            {"WARNING"},
            "UV Unwrap Faces is disabled — pending rebuild on chain-first pipeline"
        )
        return {"CANCELLED"}


class HOTSPOTUV_OT_ManualDock(bpy.types.Operator):
    bl_idname = "hotspotuv.manual_dock"
    bl_label = "Manual Dock Islands"
    bl_description = "[DISABLED] Pending rebuild on ScaffoldMap pipeline (Phase 5)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        self.report(
            {"WARNING"},
            "Manual Dock is disabled — pending rebuild on chain-first pipeline"
        )
        return {"CANCELLED"}


class HOTSPOTUV_OT_SelectSimilar(bpy.types.Operator):
    bl_idname = "hotspotuv.select_similar"
    bl_label = "Select Similar Islands"
    bl_description = "[DISABLED] Pending rebuild on ScaffoldMap pipeline (Phase 5)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        self.report(
            {"WARNING"},
            "Select Similar is disabled — pending rebuild"
        )
        return {"CANCELLED"}


class HOTSPOTUV_OT_StackSimilar(bpy.types.Operator):
    bl_idname = "hotspotuv.stack_similar"
    bl_label = "Stack Similar Islands"
    bl_description = "[DISABLED] Pending rebuild on ScaffoldMap pipeline (Phase 5)"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        self.report(
            {"WARNING"},
            "Stack Similar is disabled — pending rebuild"
        )
        return {"CANCELLED"}


# ============================================================
# PANEL
# ============================================================

class HOTSPOTUV_PT_Panel(bpy.types.Panel):
    bl_label = "Hotspot UV"
    bl_idname = "HOTSPOTUV_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Hotspot UV"

    def draw(self, context):
        layout = self.layout
        s = context.scene.hotspotuv_settings

        # --- Settings ---
        col = layout.column(align=True)
        col.prop(s, "target_texel_density")
        col.prop(s, "texture_size")
        col.prop(s, "uv_scale")
        col.prop(s, "uv_range_limit")

        # --- Face Tools (disabled) ---
        layout.separator()
        col = layout.column(align=True)
        col.label(text="Face Tools:")
        row = col.row(align=True)
        row.enabled = False
        row.operator("hotspotuv.unwrap_faces", text="UV Unwrap Faces", icon="UV")

        # --- Edge Tools (disabled) ---
        layout.separator()
        col = layout.column(align=True)
        col.label(text="Edge Tools:")
        row = col.row(align=True)
        row.enabled = False
        row.operator("hotspotuv.manual_dock", text="Manual Dock Islands", icon="SNAP_ON")

        # --- Utility Tools (disabled) ---
        layout.separator()
        col = layout.column(align=True)
        col.label(text="Utility Tools:")
        row = col.row(align=True)
        row.enabled = False
        row.operator("hotspotuv.select_similar", text="Select Similar", icon="RESTRICT_SELECT_OFF")
        row = col.row(align=True)
        row.enabled = False
        row.operator("hotspotuv.stack_similar", text="Stack Similar", icon="ALIGN_CENTER")

        # --- Debug ---
        layout.separator()
        col = layout.column(align=True)
        col.label(text="Debug:")
        col.operator("hotspotuv.flow_debug", text="Flow Debug", icon="SORTSIZE")
        col.operator("hotspotuv.scaffold_debug", text="Scaffold Debug", icon="MESH_GRID")
        col.operator("hotspotuv.frontier_replay", text="Frontier Replay", icon="PLAY")
        col.operator(
            "hotspotuv.solve_phase1_preview", text="Solve Phase 1 Preview", icon="UV"
        )

        # --- Analyze toggle ---
        if s.dbg_active:
            col.operator(
                "hotspotuv.debug_analysis",
                text="Analyze: ON",
                icon="PAUSE",
                depress=True,
            )
        else:
            col.operator(
                "hotspotuv.debug_analysis", text="Analyze: OFF", icon="VIEWZOOM"
            )

        # --- Layer controls (only when debug active) ---
        if s.dbg_active and s.dbg_source_object:
            gp_name = GP_DEBUG_PREFIX + s.dbg_source_object
            gp_obj = bpy.data.objects.get(gp_name)
            has_gp = gp_obj is not None and gp_obj.type == 'GPENCIL'

            if has_gp:
                gp_data = gp_obj.data
                _draw_debug_group(
                    col, s, gp_data, "patches", "Patches", "MESH_GRID",
                    [('Patches_WALL', 'Wall'), ('Patches_FLOOR', 'Floor'),
                     ('Patches_SLOPE', 'Slope')],
                )
                _draw_debug_group(
                    col, s, gp_data, "loops", "Loop Types", "CURVE_BEZCIRCLE",
                    [('Loops_Chains', 'Chains'), ('Loops_Boundary', 'Boundary'),
                     ('Loops_Holes', 'Holes')],
                )
                _draw_debug_group(
                    col, s, gp_data, "overlay", "Overlay", "ORIENTATION_LOCAL",
                    [('Overlay_Basis', 'Basis (U/V/N)'),
                     ('Overlay_Centers', 'Centers')],
                )

            col.separator()
            col.operator("hotspotuv.debug_clear", text="Force Clear", icon="X")


def _draw_debug_group(col, settings, gp_data, group_name, label, icon, layers):
    """Рисует collapsible debug group в panel."""
    grp_prop = f"dbg_grp_{group_name}"
    is_expanded = getattr(settings, grp_prop, True)

    box = col.box()
    row = box.row(align=True)
    arrow_icon = 'TRIA_DOWN' if is_expanded else 'TRIA_RIGHT'
    op = row.operator("hotspotuv.debug_toggle_group", text="", icon=arrow_icon, emboss=False)
    op.group_name = group_name
    row.label(text=label, icon=icon)

    if is_expanded:
        for layer_name, layer_label in layers:
            if layer_name in gp_data.layers:
                layer = gp_data.layers[layer_name]
                row = box.row(align=True)
                row.separator(factor=2.0)
                vis_icon = 'HIDE_OFF' if not layer.hide else 'HIDE_ON'
                op = row.operator(
                    "hotspotuv.debug_toggle_layer", text="", icon=vis_icon
                )
                op.layer_name = layer_name
                row.label(text=layer_label)


# ============================================================
# REGISTRATION
# ============================================================

classes = (
    HOTSPOTUV_Settings,
    # Active operators
    HOTSPOTUV_OT_DebugAnalysis,
    HOTSPOTUV_OT_DebugClear,
    HOTSPOTUV_OT_DebugToggleLayer,
    HOTSPOTUV_OT_DebugToggleGroup,
    HOTSPOTUV_OT_FlowDebug,
    HOTSPOTUV_OT_ScaffoldDebug,
    HOTSPOTUV_OT_FrontierReplay,
    HOTSPOTUV_OT_SolvePhase1Preview,
    # Legacy stubs (disabled)
    HOTSPOTUV_OT_UnwrapFaces,
    HOTSPOTUV_OT_ManualDock,
    HOTSPOTUV_OT_SelectSimilar,
    HOTSPOTUV_OT_StackSimilar,
    # Panel
    HOTSPOTUV_PT_Panel,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.hotspotuv_settings = PointerProperty(type=HOTSPOTUV_Settings)


def unregister():
    # Cleanup GP debug objects
    for obj in list(bpy.data.objects):
        if obj.name.startswith(GP_DEBUG_PREFIX):
            bpy.data.objects.remove(obj, do_unlink=True)
    if hasattr(bpy.types.Scene, "hotspotuv_settings"):
        del bpy.types.Scene.hotspotuv_settings
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
