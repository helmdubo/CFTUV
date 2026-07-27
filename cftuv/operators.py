"""CFTUV Operators — Blender UI, operators, settings.

Thin wrappers only. No geometry logic here (max 5 lines math).
All heavy work delegated to analysis.py, solve.py, debug.py.
"""

from dataclasses import replace
from math import pi
from pathlib import Path

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
    build_analysis_bundle,
    build_straighten_structural_support,
    format_patch_graph_report,
    format_patch_graph_snapshot_report,
    format_solver_input_preflight_report,
    validate_solver_input_mesh,
)
from .analysis_surface import source_revision_from_bmesh
from .constants import GP_DEBUG_PREFIX
from .decal_modal import (
    DECAL_DRAG_DISTANCE as _DECAL_DRAG_DISTANCE,
    DECAL_SIZE_MIN as _DECAL_SIZE_MIN,
    DecalDragTarget as _DecalDragTarget,
    decal_drag_target_value as _decal_drag_target_value,
    decal_drag_targets as _decal_drag_targets,
    format_decal_drag_value as _format_decal_drag_value,
    warp_decal_drag_cursor as _warp_decal_drag_cursor,
)
from .decal_transform import (
    metric_context_for_source,
)
from .decal_session import CapturedDecalRequest, DecalSessionController
from .debug import (
    ENVELOPE_DEBUG_GP_PREFIX,
    GreasePencilDebugWriter,
    apply_layer_visibility,
    clear_decal_rail_visualization,
    clear_visualization,
    create_decal_rail_visualization,
    create_frontier_visualization,
    create_visualization,
    get_gp_layer,
    gp_layer_name,
    is_gp_debug_object,
)
from .decals import (
    DecalGenerationResult,
    DecalPreviewState,
    PreviewStatus,
    chain_refs_for_edge_indices,
    compile_manual_seam_decal_plan,
    decal_compile_alpha_budget,
    decal_mode_archived_reason,
    evaluate_manual_seam_faces_local,
    generate_decal_result_local,
    remove_decal_preview_object,
)
from .decal_gpu_preview import create_controller as create_gpu_preview
from .model import DecalSettings, MeshPreflightReport, UVSettings
from .solve import (
    build_root_scaffold_map,
    build_solver_graph,
    execute_phase1_preview,
    format_regression_snapshot_report,
    format_root_scaffold_report,
    format_solve_plan_report,
    plan_solve_phase1,
)
from .version_info import resolve_addon_build_info

ADDON_PACKAGE = __package__ or Path(__file__).resolve().parent.name
ADDON_BUILD_INFO = resolve_addon_build_info(Path(__file__).resolve())


def _coerce_decal_generation_result(value):
    """Временный adapter старых list-based mocks/callers к A6 result."""

    if isinstance(value, DecalGenerationResult):
        return value
    names = list(value or ())
    if not names:
        return DecalGenerationResult(
            PreviewStatus.EMPTY,
            None,
            reason="generation produced no objects",
        )
    return DecalGenerationResult(
        PreviewStatus.UPDATED,
        str(names[0]),
        topology_changed=True,
    )


def _generation_result_names(result):
    return [result.object_name] if result.object_name else []


class HOTSPOTUV_OT_CleanNonManifoldEdges(bpy.types.Operator):
    bl_idname = "hotspotuv.clean_non_manifold_edges"
    bl_label = "Clean Non-Manifold Edges"
    bl_description = "Find NON_MANIFOLD_EDGE issues and split faces and edges by vertices"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        obj = context.active_object
        if obj is None or obj.type != 'MESH':
            self.report({"ERROR"}, "Select a mesh object")
            return {"CANCELLED"}

        original_mode = obj.mode
        try:
            if obj.mode != 'EDIT':
                bpy.ops.object.mode_set(mode='EDIT')

            bm = bmesh.from_edit_mesh(obj.data)
            bm.faces.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()

            preflight = validate_solver_input_mesh(bm, [face.index for face in bm.faces])
            non_manifold_report = _filter_preflight_issues(preflight, "NON_MANIFOLD_EDGE")
            if non_manifold_report.is_valid:
                if original_mode == 'OBJECT':
                    bpy.ops.object.mode_set(mode='OBJECT')
                self.report({"INFO"}, "No NON_MANIFOLD_EDGE issues found")
                return {"FINISHED"}

            edge_indices = sorted(
                {
                    edge_index
                    for issue in non_manifold_report.issues
                    for edge_index in issue.edge_indices
                    if 0 <= edge_index < len(bm.edges)
                }
            )
            selection_message = _highlight_solver_preflight_issues(
                context,
                obj,
                bm,
                non_manifold_report,
            )
            bpy.ops.mesh.edge_split(type='VERT')

            bm = bmesh.from_edit_mesh(obj.data)
            bm.faces.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            bm.edges.ensure_lookup_table()
            bmesh.update_edit_mesh(obj.data)

            remaining_preflight = validate_solver_input_mesh(bm, [face.index for face in bm.faces])
            remaining_non_manifold = _filter_preflight_issues(remaining_preflight, "NON_MANIFOLD_EDGE")
            remaining_count = len(remaining_non_manifold.issues)
            remaining_breakdown = _format_solver_preflight_breakdown(remaining_non_manifold)
            breakdown_suffix = f" Remaining: {remaining_breakdown}" if remaining_breakdown else ""
            message = (
                f"Edge Split by vertices applied to {len(edge_indices)} non-manifold edges. "
                f"Remaining non-manifold issues: {remaining_count}.{breakdown_suffix}"
            )
            print(f"[CFTUV][Cleanup] {selection_message}")
            print(f"[CFTUV][Cleanup] {message}")
            self.report({"WARNING"} if remaining_count > 0 else {"INFO"}, message)
            return {"FINISHED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Clean Non-Manifold Edges failed: {exc}")
            return {"CANCELLED"}


# ============================================================
# UI SETTINGS
# ============================================================


class HOTSPOTUV_AddonPreferences(bpy.types.AddonPreferences):
    bl_idname = ADDON_PACKAGE

    clear_pins_after_phase1: BoolProperty(
        name="Clear Pins After Solve Phase 1",
        default=True,
        description="Remove UV pins after Solve Phase 1 Preview. Disable to keep pins visible for debug inspection",
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "clear_pins_after_phase1")


def _update_envelope_debug_visibility(settings, _context):
    """Обновляет только видимость уже построенных Envelope GP layers."""

    source_name = str(settings.envelope_debug_source_object).strip()
    if not source_name:
        return
    try:
        from .envelope_debug_renderer import apply_envelope_debug_visibility

        apply_envelope_debug_visibility(source_name, settings)
    except (ImportError, RuntimeError, TypeError, ValueError) as exc:
        print(f"[CFTUV][EnvelopeDebug] Visibility update failed: {exc}")


def _update_envelope_debug_alpha(settings, context):
    """Ползунок alpha на движке QUEUE: покрытие и слои, без единой компиляции.

    Ничего не считает, пока нет тёплой подготовки: без неё ответ «ждём кнопки»,
    а не «посчитаем сейчас» — подготовка стоит десятки миллисекунд на домен.
    """

    from .envelope_queue_export import ENVELOPE_DEBUG_ENGINE_QUEUE

    if str(settings.envelope_debug_engine) != ENVELOPE_DEBUG_ENGINE_QUEUE:
        return
    source_name = str(settings.envelope_debug_source_object).strip()
    controller = (
        getattr(
            context.window_manager,
            "_cftuv_envelope_debug_session",
            None,
        )
        if context is not None and context.window_manager is not None
        else None
    )
    if not source_name or controller is None:
        return
    try:
        from .envelope_debug_renderer import update_queue_alpha

        status = update_queue_alpha(
            controller,
            source_name,
            float(settings.envelope_debug_alpha),
            settings=settings,
        )
    except (ImportError, KeyError, RuntimeError, TypeError, ValueError) as exc:
        settings.envelope_debug_queue_timing = (
            f"QUEUE alpha update failed: {type(exc).__name__}"
        )
        print(f"[CFTUV][EnvelopeDebug] QUEUE alpha update failed: {exc}")
        return
    if status is not None:
        settings.envelope_debug_queue_timing = status


class HOTSPOTUV_Settings(bpy.types.PropertyGroup):
    target_texel_density: IntProperty(
        name="Target Texel Density (px/m)", default=512, min=1
    )
    texture_size: IntProperty(name="Texture Size", default=2048, min=1)
    uv_scale: FloatProperty(name="Custom Scale Multiplier", default=1.0, min=0.0001)
    uv_range_limit: IntProperty(name="UV Range Limit (Tiles)", default=16, min=0)

    # Decals
    decal_width_corner: FloatProperty(
        name="Corner Width",
        default=0.20,
        min=_DECAL_SIZE_MIN,
        description="Total width of corner decal strips (world units)",
    )
    decal_width_seam: FloatProperty(
        name="Seam Width",
        default=0.15,
        min=_DECAL_SIZE_MIN,
        description="Total width of flat seam decal strips (world units)",
    )
    decal_height_trim: FloatProperty(
        name="Trim Height",
        default=0.25,
        min=_DECAL_SIZE_MIN,
        description="Height of top/bottom trim decal strips (world units)",
    )
    decal_offset: FloatProperty(
        name="Surface Offset",
        default=0.02,
        min=0.0,
        description="Decal offset from the source surface to avoid z-fighting",
    )
    decal_seam_network: BoolProperty(
        name="Deprecated Seam Network Toggle",
        default=True,
        description=(
            "Compatibility-only saved property; Decal Seams always uses the "
            "strict rail/Patch runtime and never enables legacy geometry"
        ),
    )
    decal_chart_distortion_budget: FloatProperty(
        name="Chart Distortion Budget",
        default=0.02,
        min=0.005,
        soft_max=0.05,
        max=0.10,
        subtype="FACTOR",
        description=(
            "Maximum measured intrinsic decal width error; compile-time "
            "setting, applied when the decal operator starts"
        ),
    )
    decal_dynamic_corner_bands: BoolProperty(
        name="Experimental Dynamic Corner Bands",
        default=False,
        description=(
            "Enable the experimental A11 corner-band grammar; the stable "
            "default keeps collision-resolved decal fronts fixed"
        ),
    )
    decal_corner_join_mode: EnumProperty(
        name="Corner Join",
        items=(
            (
                "MITER",
                "Miter",
                "Preserve the apex at convex ordinary corners",
            ),
            (
                "BEVEL",
                "Bevel",
                "Use the CornerModel chord at eligible exterior corners",
            ),
        ),
        default="MITER",
        description="Compile-time CornerModel join style",
    )
    decal_corner_acute_split_angle: FloatProperty(
        name="Split Angle",
        subtype="ANGLE",
        default=pi / 3.0,
        min=pi / 180.0,
        max=pi * 179.0 / 180.0,
        description=(
            "Boundary between FAN and ACUTE_SPLIT corner bands; evaluated "
            "without rebuilding the Voronoi diagram"
        ),
    )
    decal_corner_miter_angle: FloatProperty(
        name="Miter Angle",
        subtype="ANGLE",
        default=2.0 * pi / 3.0,
        min=0.0,
        max=pi,
        description="Boundary between continuous MITER and KITE bands",
    )
    decal_corner_kite_angle: FloatProperty(
        name="Kite Angle",
        subtype="ANGLE",
        default=pi / 2.0,
        min=0.0,
        max=pi,
        description="Boundary between KITE and FAN corner bands",
    )
    decal_corner_hairpin_angle: FloatProperty(
        name="Hairpin Angle",
        subtype="ANGLE",
        default=pi / 6.0,
        min=0.0,
        max=pi,
        description="Boundary between ACUTE_SPLIT and HAIRPIN bands",
    )
    decal_corner_miter_limit: FloatProperty(
        name="Apex Limit",
        default=8.0,
        min=1.0,
        soft_max=16.0,
        description=(
            "Maximum removed-apex distance as a multiple of half decal "
            "width; evaluated at runtime for miter, kite and acute corners"
        ),
    )
    decal_preview_display: EnumProperty(
        name="Preview Display",
        items=(
            (
                "GPU_TEXTURED",
                "GPU Textured",
                "GPU overlay with the source object's first image texture",
            ),
            (
                "GPU",
                "GPU Solid",
                "GPU overlay colored by decal component kind",
            ),
            (
                "MESH",
                "Mesh",
                "Persistent preview mesh (debug fallback)",
            ),
        ),
        default="GPU_TEXTURED",
        description=(
            "Modal drag display adapter; confirm always uses exact BMesh "
            "materialization"
        ),
    )
    # Exact-planar Envelope diagnostic bridge.
    envelope_debug_engine: EnumProperty(
        name="Engine",
        items=(
            (
                "LEGACY",
                "Legacy: union + interactions",
                "Exact reference path: raw union, then pairwise interaction "
                "resolution and ownership",
            ),
            (
                "QUEUE",
                "Queue: skeleton + owners",
                "Wavefront conveyor: bridge, straight skeleton, faces and "
                "owner-tagged coverage without any union",
            ),
        ),
        default="LEGACY",
        description=(
            "Which evaluator the staged Envelope debug build runs. LEGACY "
            "keeps the accepted reference path unchanged"
        ),
    )
    envelope_debug_alpha: FloatProperty(
        name="Alpha",
        default=0.25,
        min=0.0,
        description="Exact propagation alpha for a complete selected PhysicalChain",
        update=_update_envelope_debug_alpha,
    )
    envelope_debug_queue_timing: StringProperty(
        name="Envelope Queue Timing",
        default="",
    )
    envelope_debug_source_object: StringProperty(
        name="Envelope Debug Source",
        default="",
    )
    envelope_debug_status: StringProperty(
        name="Envelope Debug Status",
        default="Not built",
    )
    envelope_debug_outcome: StringProperty(
        name="Envelope Debug Outcome",
        default="",
    )
    envelope_debug_stage_summary: StringProperty(
        name="Envelope Stage Summary",
        default="",
    )
    envelope_debug_domain_status: StringProperty(
        name="Envelope Domain Status",
        default="",
    )
    envelope_debug_show_domains: BoolProperty(
        name="Domains",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_chains: BoolProperty(
        name="Chains",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_supports: BoolProperty(
        name="Supports",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_envelopes: BoolProperty(
        name="Envelopes",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_raw: BoolProperty(
        name="Raw",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_readings: BoolProperty(
        name="Readings",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_equality: BoolProperty(
        name="Equality",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_resolved: BoolProperty(
        name="Resolved",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_diagnostics: BoolProperty(
        name="Diagnostics",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_labels: BoolProperty(
        name="Labels",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    envelope_debug_show_queue: BoolProperty(
        name="Queue",
        default=True,
        update=_update_envelope_debug_visibility,
    )
    # Debug state
    dbg_active: BoolProperty(
        name="Analyze", default=False, description="Debug analysis mode"
    )
    dbg_source_object: StringProperty(name="Debug Source", default="")
    dbg_replay_active: BoolProperty(name="Replay Active", default=False)
    dbg_replay_source_object: StringProperty(name="Replay Source", default="")
    dbg_replay_cleanup_pending: BoolProperty(name="Replay Cleanup Pending", default=False)
    dbg_verbose_console: BoolProperty(
        name="Verbose Console",
        default=False,
        description="Print full reports and trace diagnostics to System Console",
    )
    dbg_show_advanced_debug: BoolProperty(
        name="Advanced Debug",
        default=False,
        description="Show rarely used developer debug tools",
    )
    phase1_make_seams_by_sharp: BoolProperty(
        name="Make Seams by Sharp",
        default=False,
        description="Before Phase 1 preview, mark sharp edges as seam",
    )
    straighten_strips: BoolProperty(
        name="Straighten Strips",
        default=False,
        description="Place structurally strong FREE chains as straight frame lines (inherited from neighbor)",
    )

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

    # Overlay group
    dbg_overlay_labels: BoolProperty(name="Chain Labels", default=True)


# ============================================================
# OPERATOR HELPERS
# ============================================================


class _SolverPreflightSelectionError(ValueError):
    """Blocking solver preflight error after selecting offending topology."""

    def __init__(self, summary, selection_message=None):
        super().__init__(summary)
        self.summary = summary
        self.selection_message = selection_message or summary


def _clear_pins_after_phase1_enabled(context) -> bool:
    addon = getattr(context.preferences, "addons", {}).get(ADDON_PACKAGE)
    preferences = getattr(addon, "preferences", None)
    return bool(getattr(preferences, "clear_pins_after_phase1", True))


def _build_scaffold_map_with_straighten(graph, solve_plan, settings):
    """Build scaffold map, optionally applying straighten strips from structural analysis."""
    straighten = getattr(settings, 'straighten_strips', False)
    # Shape classification always runs (independent of straighten toggle).
    # Straighten-specific data (inherited roles, structural summaries) only
    # feeds into the solver when the straighten UI toggle is active.
    inherited_map, patch_structural_summaries, patch_shape_classes, straighten_chain_refs, band_spine_data = build_straighten_structural_support(graph)
    return build_root_scaffold_map(
        graph, solve_plan, settings.final_scale,
        straighten_enabled=straighten,
        inherited_role_map=inherited_map if straighten else None,
        patch_structural_summaries=patch_structural_summaries if straighten else None,
        patch_shape_classes=patch_shape_classes,
        straighten_chain_refs=straighten_chain_refs if straighten else None,
        band_spine_data=band_spine_data if straighten else None,
    )


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
        'overlay_labels': bool(overlay_visible and settings.dbg_overlay_labels),
        'frontier_path': True,
    }


def _refresh_debug_layers(context, patch_graph, source_obj, scaffold_map=None):
    """Refresh all visible debug layers from the same current PatchGraph."""
    dbg_settings = _build_debug_settings(context.scene.hotspotuv_settings)
    create_visualization(patch_graph, source_obj, dbg_settings)
    if scaffold_map is not None:
        create_frontier_visualization(patch_graph, scaffold_map, source_obj, dbg_settings)


def _find_screen_override():
    for window in bpy.context.window_manager.windows:
        screen = window.screen
        for area in screen.areas:
            if area.type not in {'VIEW_3D', 'DOPESHEET_EDITOR', 'TIMELINE'}:
                continue
            region = next((reg for reg in area.regions if reg.type == 'WINDOW'), None)
            if region is None:
                continue
            return {
                'window': window,
                'screen': screen,
                'area': area,
                'region': region,
            }
    return None


def _stop_animation_playback():
    override = _find_screen_override()
    if override is not None:
        with bpy.context.temp_override(**override):
            try:
                bpy.ops.screen.animation_cancel(restore_frame=False)
                return
            except Exception:
                try:
                    bpy.ops.screen.animation_play()
                    return
                except Exception:
                    pass
    try:
        bpy.ops.screen.animation_cancel(restore_frame=False)
    except Exception:
        try:
            bpy.ops.screen.animation_play()
        except Exception:
            pass


def _force_clear_debug_state(context, source_obj=None, reset_timeline=False):
    s = context.scene.hotspotuv_settings

    if context.active_object and context.active_object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    resolved_source = source_obj
    if resolved_source is None:
        source_name = s.dbg_replay_source_object or s.dbg_source_object
        if source_name and source_name in bpy.data.objects:
            resolved_source = bpy.data.objects[source_name]
        else:
            obj = context.active_object
            if obj:
                if obj.name.startswith(GP_DEBUG_PREFIX):
                    src_name = obj.name[len(GP_DEBUG_PREFIX):]
                    resolved_source = bpy.data.objects.get(src_name)
                elif obj.type == 'MESH':
                    resolved_source = obj

    if resolved_source is not None and resolved_source.name in bpy.data.objects:
        resolved_source.hide_viewport = False
        resolved_source.hide_set(False)
        clear_visualization(resolved_source)
        bpy.ops.object.select_all(action='DESELECT')
        resolved_source.select_set(True)
        context.view_layer.objects.active = resolved_source

    if reset_timeline:
        context.scene.frame_current = context.scene.frame_start

    s.dbg_active = False
    s.dbg_source_object = ""
    s.dbg_replay_active = False
    s.dbg_replay_source_object = ""
    s.dbg_replay_cleanup_pending = False


def _finish_frontier_replay(scene_name):
    scene = bpy.data.scenes.get(scene_name)
    if scene is None:
        return None
    settings = scene.hotspotuv_settings
    if not settings.dbg_replay_cleanup_pending:
        return None

    _stop_animation_playback()
    scene.frame_current = scene.frame_start
    source_obj = None
    source_name = settings.dbg_replay_source_object or settings.dbg_source_object
    if source_name and source_name in bpy.data.objects:
        source_obj = bpy.data.objects[source_name]
    _force_clear_debug_state(bpy.context, source_obj=source_obj, reset_timeline=False)
    return None


def _frontier_replay_frame_handler(scene, _depsgraph=None):
    settings = getattr(scene, 'hotspotuv_settings', None)
    if settings is None or not settings.dbg_replay_active or settings.dbg_replay_cleanup_pending:
        return
    if scene.frame_current < scene.frame_end:
        return
    settings.dbg_replay_cleanup_pending = True
    bpy.app.timers.register(lambda: _finish_frontier_replay(scene.name), first_interval=0.0)


def _print_console_report(title, lines, summary=None, show_lines=True):
    """Печатает отчёт в System Console."""
    print('=' * 60)
    print(title)
    print('=' * 60)
    if show_lines:
        for line in lines:
            print(line)
        if summary:
            print('-' * 60)
            print(summary)
    else:
        if lines and str(lines[0]).startswith("Mesh:"):
            print(lines[0])
            if summary:
                print('-' * 60)
        if summary:
            print(summary)
    print('=' * 60)


def _safe_snapshot_name(label: str) -> str:
    safe = ''.join(ch if ch.isalnum() or ch in {'-', '_', '.'} else '_' for ch in label.strip())
    return safe or 'mesh'


def _regression_snapshot_path(obj) -> Path:
    blend_path = bpy.data.filepath
    if blend_path:
        base_dir = Path(bpy.path.abspath("//"))
        blend_stem = Path(blend_path).stem
    else:
        base_dir = Path(__file__).resolve().parents[1]
        blend_stem = "unsaved_blend"

    snapshot_dir = base_dir / "_cftuv_snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)
    filename = f"{_safe_snapshot_name(blend_stem)}__{_safe_snapshot_name(obj.name)}.md"
    return snapshot_dir / filename


def _capture_face_selection(bm):
    """Сохраняет текущее выделение faces."""
    return [face.index for face in bm.faces if face.select]


def _capture_selected_seam_edges(bm):
    """Выбранные seam edges как seed для ручного decal-режима."""
    return [edge.index for edge in bm.edges if edge.select and edge.seam]


def _capture_selected_physical_edges(bm):
    """Выбранные physical edges для whole-chain Envelope debug request."""
    return [edge.index for edge in bm.edges if edge.select]


def _is_edge_select_mode(context, obj):
    return (
        obj is not None
        and obj.mode == "EDIT"
        and bool(context.tool_settings.mesh_select_mode[1])
    )


def _restore_face_selection(bm, selected_indices):
    """Восстанавливает выделение faces."""
    selected_set = set(selected_indices)
    for face in bm.faces:
        face.select = face.index in selected_set


def _restore_edge_selection(obj, selected_indices):
    if obj is None or obj.name not in bpy.data.objects or obj.mode != "EDIT":
        return
    selected_set = {int(index) for index in selected_indices}
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    for edge in bm.edges:
        edge.select = edge.index in selected_set
    bmesh.update_edit_mesh(obj.data)


def _format_solver_preflight_breakdown(preflight):
    issue_counts = {}
    for issue in preflight.issues:
        issue_counts[issue.code] = issue_counts.get(issue.code, 0) + 1
    if not issue_counts:
        return ""
    return ", ".join(f"{code}:{issue_counts[code]}" for code in sorted(issue_counts.keys()))


def _filter_preflight_issues(preflight, code):
    return MeshPreflightReport(
        checked_face_indices=tuple(preflight.checked_face_indices),
        issues=[issue for issue in preflight.issues if issue.code == code],
    )


def _mark_sharp_edges_as_seams(bm):
    changed = 0
    for edge in bm.edges:
        if edge.seam or edge.smooth:
            continue
        edge.seam = True
        changed += 1
    return changed


def _highlight_solver_preflight_issues(context, obj, bm, preflight):
    """Leave the blocking topology selected for quick manual inspection."""

    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()

    edge_indices = sorted(
        {
            edge_index
            for issue in preflight.issues
            for edge_index in issue.edge_indices
            if 0 <= edge_index < len(bm.edges)
        }
    )
    face_indices = sorted(
        {
            face_index
            for issue in preflight.issues
            for face_index in issue.face_indices
            if 0 <= face_index < len(bm.faces)
        }
    )
    vert_indices = sorted(
        {
            vert_index
            for issue in preflight.issues
            for vert_index in issue.vert_indices
            if 0 <= vert_index < len(bm.verts)
        }
    )

    for face in bm.faces:
        face.select = False
    for edge in bm.edges:
        edge.select = False
    for vert in bm.verts:
        vert.select = False
    issue_breakdown = _format_solver_preflight_breakdown(preflight)
    breakdown_suffix = f" ({issue_breakdown})" if issue_breakdown else ""

    if edge_indices:
        context.tool_settings.mesh_select_mode = (False, True, False)
        for edge_index in edge_indices:
            bm.edges[edge_index].select = True
        selection_message = f"Solver preflight failed — selected {len(edge_indices)} offending edges in Edit Mode"
    elif face_indices:
        context.tool_settings.mesh_select_mode = (False, False, True)
        for face_index in face_indices:
            bm.faces[face_index].select = True
        selection_message = f"Solver preflight failed — selected {len(face_indices)} offending faces in Edit Mode"
    elif vert_indices:
        context.tool_settings.mesh_select_mode = (True, False, False)
        for vert_index in vert_indices:
            bm.verts[vert_index].select = True
        selection_message = f"Solver preflight failed — selected {len(vert_indices)} offending verts in Edit Mode"
    else:
        selection_message = "Solver preflight failed — offending topology could not be selected"

    if breakdown_suffix:
        selection_message = f"{selection_message}{breakdown_suffix}"

    bm.select_flush_mode()
    bmesh.update_edit_mesh(obj.data)
    return selection_message


def _report_solver_preflight_selection(operator, exc):
    message = getattr(exc, "selection_message", str(exc))
    operator.report({"WARNING"}, message)


def _prepare_patch_graph(
    context,
    require_selection=True,
    validate_for_solver=False,
    make_seams_by_sharp=False,
    use_all_faces=False,
):
    """Prepare an atomic AnalysisBundle from the current context.

    Returns: (obj, bm, analysis_bundle, original_mode, selected_face_indices)
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

    if make_seams_by_sharp:
        seam_count = _mark_sharp_edges_as_seams(bm)
        if seam_count > 0:
            bmesh.update_edit_mesh(obj.data)
            print(f"[CFTUV][Phase1] Make Seams by Sharp: marked {seam_count} sharp edges")

    selected_face_indices = _capture_face_selection(bm)

    try:
        if original_mode == 'OBJECT' or use_all_faces:
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
                report = format_solver_input_preflight_report(preflight, mesh_name=obj.name)
                _print_console_report('CFTUV Solver Preflight', report.lines, report.summary)
                selection_message = _highlight_solver_preflight_issues(context, obj, bm, preflight)
                raise _SolverPreflightSelectionError(report.summary, selection_message)

        analysis_bundle = build_analysis_bundle(bm, face_indices, obj)
        return obj, bm, analysis_bundle, original_mode, selected_face_indices
    except _SolverPreflightSelectionError:
        raise
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


def _prepare_decal_generation(context, mode="SEAMS"):
    """Собирает immutable state для immediate или modal decal generation."""

    decal_settings = context.scene.hotspotuv_settings
    source_obj = context.active_object
    metric_context = metric_context_for_source(source_obj)
    manual_selection = _is_edge_select_mode(context, source_obj)
    selected_edge_indices = []
    if manual_selection:
        source_bm = bmesh.from_edit_mesh(source_obj.data)
        source_bm.edges.ensure_lookup_table()
        selected_edge_indices = _capture_selected_seam_edges(source_bm)
        if not selected_edge_indices:
            raise ValueError(
                "Edge Select Mode: select one or more seam-marked edges"
            )

    obj, _bm, analysis_bundle, original_mode, selected_faces = _prepare_patch_graph(
        context,
        require_selection=not manual_selection,
        use_all_faces=manual_selection,
    )
    try:
        chain_refs = None
        if manual_selection:
            chain_refs = chain_refs_for_edge_indices(
                analysis_bundle.patch_graph, selected_edge_indices
            )
            if not chain_refs:
                raise ValueError(
                    "Selected seam edges do not belong to PatchGraph boundary chains"
                )
        settings = DecalSettings.from_blender_settings(decal_settings)
        return CapturedDecalRequest(
            mode=mode,
            source_obj=obj,
            analysis_bundle=analysis_bundle,
            world_settings=settings,
            metric_context=metric_context,
            chain_refs=chain_refs,
            manual_selection=manual_selection,
            selected_edge_indices=tuple(selected_edge_indices),
        )
    finally:
        _restore_mode_and_selection(obj, original_mode, selected_faces)
        if manual_selection:
            _restore_edge_selection(obj, selected_edge_indices)


def _build_solve_state(context, make_seams_by_sharp=False):
    """Полный solve state: PatchGraph + SolverGraph + SolvePlan + UVSettings."""
    obj, bm, analysis_bundle, original_mode, sel = _prepare_patch_graph(
        context,
        require_selection=True,
        validate_for_solver=True,
        make_seams_by_sharp=make_seams_by_sharp,
    )
    patch_graph = analysis_bundle.patch_graph
    settings = UVSettings.from_blender_settings(context.scene.hotspotuv_settings)
    solver_graph = build_solver_graph(
        patch_graph,
        straighten_enabled=settings.straighten_strips,
    )
    solve_plan = plan_solve_phase1(patch_graph, solver_graph)
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

    report = format_patch_graph_report(patch_graph, mesh_name=obj.name)
    _print_console_report(
        'CFTUV PatchGraph Analyze',
        report.lines,
        report.summary,
        show_lines=bool(s.dbg_verbose_console),
    )

    # Scaffold + Frontier visualization поверх GP
    try:
        solver_graph = build_solver_graph(
            patch_graph,
            straighten_enabled=bool(getattr(s, 'straighten_strips', False)),
        )
        solve_plan = plan_solve_phase1(patch_graph, solver_graph)
        settings = UVSettings.from_blender_settings(s)
        scaffold_map = _build_scaffold_map_with_straighten(patch_graph, solve_plan, settings)
        scaffold_report = format_root_scaffold_report(patch_graph, scaffold_map, mesh_name=obj.name)
        _print_console_report(
            'CFTUV Scaffold (Analyze)',
            scaffold_report.lines,
            scaffold_report.summary,
            show_lines=bool(s.dbg_verbose_console),
        )
        create_frontier_visualization(patch_graph, scaffold_map, obj, dbg_settings)
    except Exception as exc:
        print(f"[CFTUV][Analyze] Scaffold failed: {exc}")

    return report.summary


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
        _stop_animation_playback()
        _force_clear_debug_state(context, reset_timeline=True)
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
        if not is_gp_debug_object(gp_obj):
            return {"CANCELLED"}

        gp_data = gp_obj.data

        # Chain Labels — управляем коллекцией, а не GP layer
        if self.layer_name == 'Overlay_Labels':
            from .debug import _LABEL_COLLECTION_NAME
            if _LABEL_COLLECTION_NAME in bpy.data.collections:
                col = bpy.data.collections[_LABEL_COLLECTION_NAME]
                col.hide_viewport = not col.hide_viewport
                s.dbg_overlay_labels = not col.hide_viewport
            return {"FINISHED"}

        layer = get_gp_layer(gp_data, self.layer_name)
        if layer is not None:
            layer.hide = not layer.hide
            visible = not layer.hide
            layer_setting_map = {
                'Patches_WALL': 'dbg_patches_wall',
                'Patches_FLOOR': 'dbg_patches_floor',
                'Patches_SLOPE': 'dbg_patches_slope',
                'Loops_Chains': 'dbg_loops_chains',
                'Loops_Holes': 'dbg_loops_holes',
            }
            prop_name = layer_setting_map.get(self.layer_name)
            if prop_name and hasattr(s, prop_name):
                setattr(s, prop_name, visible)

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
        if not is_gp_debug_object(gp_obj):
            return {"FINISHED"}

        gp_data = gp_obj.data
        for layer_name in self._GROUP_LAYERS.get(self.group_name, []):
            layer = get_gp_layer(gp_data, layer_name)
            if layer is not None:
                layer.hide = not new_val

        # Overlay group также управляет коллекцией labels
        if self.group_name == 'overlay':
            from .debug import _LABEL_COLLECTION_NAME
            if _LABEL_COLLECTION_NAME in bpy.data.collections:
                bpy.data.collections[_LABEL_COLLECTION_NAME].hide_viewport = not new_val

        return {"FINISHED"}


class HOTSPOTUV_OT_FlowDebug(bpy.types.Operator):
    bl_idname = "hotspotuv.flow_debug"
    bl_label = "Flow Debug"
    bl_description = "Build SolverGraph + SolvePlan, print to System Console"
    bl_options = {"REGISTER", "INTERNAL"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        try:
            obj, _bm, pg, sg, sp, _s, om, sel = _build_solve_state(context)
            report = format_solve_plan_report(pg, sg, sp, mesh_name=obj.name)
            _print_console_report('CFTUV Flow Debug', report.lines, report.summary)
            self.report({"INFO"}, report.summary)
            return {"FINISHED"}
        except _SolverPreflightSelectionError as exc:
            _report_solver_preflight_selection(self, exc)
            return {"CANCELLED"}
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
    bl_options = {"REGISTER", "INTERNAL"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        try:
            obj, _bm, pg, _sg, sp, settings, om, sel = _build_solve_state(context)
            scaffold_map = _build_scaffold_map_with_straighten(pg, sp, settings)
            report = format_root_scaffold_report(pg, scaffold_map, mesh_name=obj.name)
            _print_console_report('CFTUV Scaffold Debug', report.lines, report.summary)

            # Frontier path visualization поверх существующего GP
            s = context.scene.hotspotuv_settings
            source_name = s.dbg_source_object
            if source_name and source_name in bpy.data.objects:
                _refresh_debug_layers(context, pg, bpy.data.objects[source_name], scaffold_map)

            self.report({"INFO"}, report.summary)
            return {"FINISHED"}
        except _SolverPreflightSelectionError as exc:
            _report_solver_preflight_selection(self, exc)
            return {"CANCELLED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Scaffold Debug failed: {exc}")
            return {"CANCELLED"}
        finally:
            if 'obj' in locals() and 'om' in locals() and 'sel' in locals():
                _restore_mode_and_selection(obj, om, sel)


class HOTSPOTUV_OT_SaveRegressionSnapshot(bpy.types.Operator):
    bl_idname = "hotspotuv.save_regression_snapshot"
    bl_label = "Save Regression Snapshot"
    bl_description = "Serialize a stable scaffold baseline report to a markdown file"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        try:
            obj, bm, pg, _sg, sp, settings, om, sel = _build_solve_state(context)
            scaffold_map = _build_scaffold_map_with_straighten(pg, sp, settings)
            patch_graph_report = format_patch_graph_snapshot_report(
                pg,
                mesh_name=obj.name,
            )
            report = format_regression_snapshot_report(
                bm,
                pg,
                sp,
                scaffold_map,
                mesh_name=obj.name,
            )
            output_path = _regression_snapshot_path(obj)
            content = "\n".join([
                "# CFTUV Regression Snapshot",
                "",
                "## PatchGraph Snapshot",
                "",
                *patch_graph_report.lines,
                "",
                patch_graph_report.summary,
                "",
                "## Scaffold Snapshot",
                "",
                *report.lines,
                "",
                "---",
                report.summary,
                "",
            ])
            output_path.write_text(content, encoding="utf-8")
            _print_console_report(
                'CFTUV Regression Snapshot',
                report.lines,
                report.summary,
                show_lines=bool(context.scene.hotspotuv_settings.dbg_verbose_console),
            )
            self.report({"INFO"}, f"Snapshot saved: {output_path}")
            return {"FINISHED"}
        except _SolverPreflightSelectionError as exc:
            _report_solver_preflight_selection(self, exc)
            return {"CANCELLED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Save Regression Snapshot failed: {exc}")
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
        if s.dbg_replay_active:
            return True
        if s.dbg_active and s.dbg_source_object:
            return True
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and obj.mode in {'EDIT', 'OBJECT'}

    def execute(self, context):
        s = context.scene.hotspotuv_settings

        if s.dbg_replay_active:
            _stop_animation_playback()
            _force_clear_debug_state(context, reset_timeline=True)
            self.report({"INFO"}, "Frontier Replay stopped")
            return {"FINISHED"}

        # В debug mode mesh скрыт, активен GP — нужно переключиться
        source_name = s.dbg_source_object if s.dbg_active else ""
        need_restore_debug = False

        try:
            if source_name and source_name in bpy.data.objects:
                source_obj = bpy.data.objects[source_name]
                # Временно показываем mesh и делаем активным
                source_obj.hide_viewport = False
                source_obj.hide_set(False)
                if context.active_object and context.active_object.mode != 'OBJECT':
                    bpy.ops.object.mode_set(mode='OBJECT')
                bpy.ops.object.select_all(action='DESELECT')
                source_obj.select_set(True)
                context.view_layer.objects.active = source_obj
                need_restore_debug = True

            obj, _bm, pg, _sg, sp, settings, om, sel = _build_solve_state(context)
            scaffold_map = _build_scaffold_map_with_straighten(pg, sp, settings)
            _restore_mode_and_selection(obj, om, sel)

            # Frontier visualization
            target_name = source_name or obj.name
            if target_name in bpy.data.objects:
                target_obj = bpy.data.objects[target_name]
                _refresh_debug_layers(context, pg, target_obj, scaffold_map)

                # Прячем mesh
                target_obj.hide_viewport = True

                # Показываем только Frontier_Path
                gp_name = GP_DEBUG_PREFIX + target_name
                gp_obj = bpy.data.objects.get(gp_name)
                if is_gp_debug_object(gp_obj):
                    for layer in gp_obj.data.layers:
                        layer.hide = (gp_layer_name(layer) != 'Frontier_Path')
                    # Активируем GP
                    if context.active_object and context.active_object.mode != 'OBJECT':
                        bpy.ops.object.mode_set(mode='OBJECT')
                    bpy.ops.object.select_all(action='DESELECT')
                    gp_obj.select_set(True)
                    context.view_layer.objects.active = gp_obj

                # Запускаем playback
                s.dbg_replay_active = True
                s.dbg_replay_source_object = target_name
                s.dbg_replay_cleanup_pending = False
                context.scene.frame_current = 0
                bpy.ops.screen.animation_play()

            self.report({"INFO"}, "Frontier Replay started — click again to stop")
            return {"FINISHED"}
        except _SolverPreflightSelectionError as exc:
            s.dbg_replay_active = False
            s.dbg_replay_source_object = ""
            s.dbg_replay_cleanup_pending = False
            _report_solver_preflight_selection(self, exc)
            return {"CANCELLED"}
        except Exception as exc:
            s.dbg_replay_active = False
            s.dbg_replay_source_object = ""
            s.dbg_replay_cleanup_pending = False
            self.report({"ERROR"}, f"Frontier Replay failed: {exc}")
            return {"CANCELLED"}


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
            scene_settings = context.scene.hotspotuv_settings
            obj, bm, pg, _sg, sp, settings, om, sel = _build_solve_state(
                context,
                make_seams_by_sharp=bool(scene_settings.phase1_make_seams_by_sharp),
            )
            stats = execute_phase1_preview(
                context,
                obj,
                bm,
                pg,
                settings,
                sp,
                keep_pins=not _clear_pins_after_phase1_enabled(context),
            )
            s = context.scene.hotspotuv_settings
            source_name = s.dbg_source_object
            if source_name and source_name in bpy.data.objects:
                scaffold_map = _build_scaffold_map_with_straighten(pg, sp, settings)
                _refresh_debug_layers(context, pg, bpy.data.objects[source_name], scaffold_map)
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
        except _SolverPreflightSelectionError as exc:
            _report_solver_preflight_selection(self, exc)
            return {"CANCELLED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Solve Phase 1 Preview failed: {exc}")
            return {"CANCELLED"}
        finally:
            if 'obj' in locals() and 'om' in locals() and 'sel' in locals():
                _restore_mode_and_selection(obj, om, sel)


class HOTSPOTUV_OT_GenerateDecals(bpy.types.Operator):
    bl_idname = "hotspotuv.generate_decals"
    bl_label = "Generate Decals"
    bl_description = (
        "Generate mesh decal strips (trims / corners / seams) from patch topology"
    )
    bl_options = {"REGISTER", "UNDO", "BLOCKING"}

    mode: EnumProperty(
        name="Mode",
        items=[
            (
                "TOP",
                "Trim Top (Archived)",
                "Unavailable until a mode-specific engine plan is implemented",
            ),
            (
                "BOTTOM",
                "Trim Bottom (Archived)",
                "Unavailable until a mode-specific engine plan is implemented",
            ),
            (
                "CORNERS",
                "Corners (Archived)",
                "Unavailable until a mode-specific engine plan is implemented",
            ),
            ("SEAMS", "Seams", "Flat strips along coplanar WALL-WALL seams"),
        ],
        default="BOTTOM",
    )

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return (
            obj is not None
            and obj.type == "MESH"
            and obj.mode in {"EDIT", "OBJECT"}
        )

    def _stop_gpu_preview(self):
        controller = getattr(self, "_modal_gpu_preview", None)
        if controller is None:
            return
        try:
            controller.stop()
        except Exception:
            pass

    def _generate_gpu_frame(self, request, local_request, plan, preview):
        """GPU display adapter: те же evaluated faces без BMesh writes."""

        controller = getattr(self, "_modal_gpu_preview", None)
        if controller is None or controller.failed or not preview:
            return None
        try:
            result = evaluate_manual_seam_faces_local(
                local_request.settings,
                plan,
                preview=True,
            )
        except Exception:
            controller.update((), "ERROR")
            raise
        if not result.faces:
            controller.update((), "EMPTY")
            return DecalGenerationResult(
                PreviewStatus.EMPTY,
                None,
                reason="evaluation produced no faces",
                evaluation_ms=result.evaluation_ms,
            )
        outcome = controller.update(result.geometry_batch, "UPDATED")
        if outcome == "FAILED":
            self._modal_gpu_preview = None
            return None
        return DecalGenerationResult(
            PreviewStatus.UPDATED,
            None,
            policy_counts=result.policy_counts,
            evaluation_ms=result.evaluation_ms,
        )

    def _evaluate_session(
        self,
        context,
        request,
        local_request,
        plan,
        preview=False,
    ):
        """Blender display/materialization port для session controller."""

        if preview and getattr(self, "_modal_gpu_preview", None) is not None:
            gpu_result = self._generate_gpu_frame(
                request,
                local_request,
                plan,
                preview,
            )
            if gpu_result is not None:
                return gpu_result
        session = getattr(self, "_decal_session", None)
        preview_state = (
            session.preview_state if session is not None else None
        )
        return generate_decal_result_local(
            request.analysis_bundle,
            request.source_obj,
            local_request.settings,
            request.mode,
            scene=context.scene,
            chain_refs=request.chain_refs,
            selected_edge_indices=request.selected_edge_indices,
            preview=preview,
            decal_plan=plan,
            preview_state=preview_state,
        )

    def _compile_session_plan(self, request, local_request):
        """Compile-port; план возвращается controller'у, не хранится в UI."""

        if (
            request.mode == "SEAMS"
            and request.manual_selection
            and request.selected_edge_indices
        ):
            local_settings = local_request.settings
            return compile_manual_seam_decal_plan(
                request.analysis_bundle,
                local_settings,
                request.selected_edge_indices,
                alpha_budget=decal_compile_alpha_budget(local_settings),
            )
        return None

    def _clear_session_preview(self):
        """CLEAR policy projection для обоих preview adapters."""

        gpu = getattr(self, "_modal_gpu_preview", None)
        if gpu is not None:
            try:
                gpu.update((), "ERROR")
            except Exception:
                pass
        session = getattr(self, "_decal_session", None)
        if session is None:
            return
        try:
            remove_decal_preview_object(
                session.request.mode,
                session.request.source_obj,
                session.preview_state,
            )
        except Exception as exc:
            reporter = getattr(self, "report", None)
            if callable(reporter):
                reporter({"ERROR"}, f"Preview clear failed: {exc}")

    def _create_session(self, context, request, *, preview_state):
        session = DecalSessionController(
            request,
            compile_plan=lambda local: self._compile_session_plan(
                request, local
            ),
            evaluate=lambda local, plan, preview: _coerce_decal_generation_result(
                self._evaluate_session(
                    context,
                    request,
                    local,
                    plan,
                    preview=preview,
                )
            ),
            clear_preview=self._clear_session_preview,
            cleanup=lambda restore: self._discard_session_resources(
                context,
                restore_scene=restore,
            ),
            preview_state=preview_state,
        )
        self._decal_session = session
        return session

    def _report_created(self, created, request, suffix=""):
        summary = "Created: " + ", ".join(created)
        if request.manual_selection:
            summary += (
                f" | chains:{len(request.chain_refs)} edges:{request.edge_count}"
            )
        session = getattr(self, "_decal_session", None)
        decal_plan = session.compiled_plan if session is not None else None
        backend_summary = getattr(decal_plan, "backend_summary", "")
        if backend_summary:
            summary += " | " + backend_summary
        if suffix:
            summary += " | " + suffix
        self.report({"INFO"}, summary)

    def _active_modal_drag_target(self):
        target = getattr(self, "_modal_drag_target", None)
        if target is not None:
            return target
        return _DecalDragTarget(
            "W",
            getattr(self, "_modal_label", "Size"),
            getattr(self, "_modal_settings_field", "width_seam"),
            getattr(self, "_modal_property", "decal_width_seam"),
            _DECAL_DRAG_DISTANCE,
            _DECAL_SIZE_MIN,
            None,
            0.01,
        )

    def _configure_modal_drag_targets(self, settings):
        targets = _decal_drag_targets(self.mode)
        session = getattr(self, "_decal_session", None)
        decal_plan = session.compiled_plan if session is not None else None
        live_corner_controls = bool(
            getattr(decal_plan, "supports_live_corner_controls", False)
        )
        if live_corner_controls:
            block_reason = ""
        elif decal_plan is None:
            block_reason = (
                "Live corner controls require Patch Voronoi-only Seams"
            )
        elif getattr(decal_plan, "rejected_edges", ()):
            block_reason = "Live corner controls require Failed:0"
        else:
            block_reason = (
                "Live corner controls require Patch Voronoi-only Seams"
            )
        self._modal_corner_targets_enabled = live_corner_controls
        self._modal_corner_target_block_reason = block_reason
        self._modal_drag_targets = {
            target.key: target
            for target in targets
            if target.key == "W" or live_corner_controls
        }
        self._modal_disabled_drag_targets = {
            target.key: target
            for target in targets
            if target.key != "W" and not live_corner_controls
        }
        self._modal_drag_target = targets[0]

    def _rebase_modal_drag(self, key, mouse_x, precise):
        targets = getattr(self, "_modal_drag_targets", {})
        target = targets.get(key)
        if target is None:
            target = self._active_modal_drag_target()
            if target.key != key:
                return False
        session = getattr(self, "_decal_session", None)
        if session is None:
            return False
        settings = session.current_world_settings
        value = getattr(settings, target.settings_field)
        self._modal_drag_target = target
        self._modal_property = target.scene_property
        self._modal_settings_field = target.settings_field
        self._modal_label = target.label
        self._modal_base_value = value
        self._modal_current_value = value
        self._modal_start_mouse = mouse_x
        self._modal_last_mouse_x = mouse_x
        self._modal_precise_mode = bool(precise)
        return True

    def _modal_value_text(self, value=None):
        if value is None:
            value = self._modal_current_value
        return _format_decal_drag_value(
            self._active_modal_drag_target(), value
        )

    def _modal_corner_diagnostics_text(self):
        if not getattr(self, "_modal_corner_targets_enabled", False):
            return ""
        session = getattr(self, "_decal_session", None)
        generation = session.last_valid_result if session is not None else None
        if generation is None:
            return ""
        counts = dict(getattr(generation, "policy_counts", ()) or ())
        evaluation_ms = float(getattr(generation, "evaluation_ms", 0.0))
        return (
            f"MITER:{int(counts.get('MITER', 0))} "
            f"BEVEL:{int(counts.get('BEVEL', 0))} "
            f"KITE:{int(counts.get('KITE', 0))} "
            f"FAN:{int(counts.get('FAN', 0))} "
            f"SPLIT:{int(counts.get('ACUTE_SPLIT', 0))} "
            f"HAIRPIN:{int(counts.get('HAIRPIN', 0))} | "
            f"{evaluation_ms:.1f} ms"
        )

    def _restore_modal_scene_targets(self, scene_settings):
        session = getattr(self, "_decal_session", None)
        initial = session.request.world_settings if session is not None else None
        targets = tuple(getattr(self, "_modal_drag_targets", {}).values())
        for target in targets:
            if initial is None or not hasattr(initial, target.settings_field):
                continue
            setattr(
                scene_settings,
                target.scene_property,
                getattr(initial, target.settings_field),
            )

    def _set_modal_header(self, value, reason=""):
        area = getattr(self, "_modal_area", None)
        if area is None:
            return
        target = self._active_modal_drag_target()
        label = "Acute Split" if target.key == "A" else target.label
        parts = [f"{label}: {self._modal_value_text(value)}"]
        diagnostics = self._modal_corner_diagnostics_text()
        if diagnostics:
            parts.append(diagnostics)
        text = " | ".join(parts) + (
            " | "
            "W: size | A: acute | M: apex | Move Left/Right | "
            "Shift: precise | LMB/Enter: confirm | Esc/RMB: cancel"
        )
        if not reason:
            reason = getattr(self, "_modal_corner_target_block_reason", "")
        if reason:
            text += f" | {reason}"
        area.header_text_set(text)

    def _clear_modal_header(self):
        area = getattr(self, "_modal_area", None)
        if area is not None:
            area.header_text_set(None)

    def _show_modal_rail_visualization(self):
        """Один раз показывает compile-static rail plan текущего modal."""

        if (
            getattr(self, "_modal_rail_visualization_shown", False)
            or getattr(self, "_modal_rail_visualization_cleared", False)
        ):
            return
        self._modal_rail_visualization_shown = True
        session = getattr(self, "_decal_session", None)
        request = session.request if session is not None else None
        source_obj = request.source_obj if request is not None else None
        if (
            self.mode != "SEAMS"
            or request is None
            or not request.manual_selection
        ):
            return
        decal_plan = session.compiled_plan
        rail_plan = getattr(decal_plan, "rail_plan", None)
        if source_obj is not None:
            create_decal_rail_visualization(rail_plan, source_obj)

    def _clear_modal_rail_visualization(self):
        """Идемпотентно удаляет rail overlay независимо от mesh preview."""

        if getattr(self, "_modal_rail_visualization_cleared", False):
            return
        self._modal_rail_visualization_cleared = True
        session = getattr(self, "_decal_session", None)
        source_obj = (
            session.request.source_obj if session is not None else None
        )
        if source_obj is None:
            return
        try:
            clear_decal_rail_visualization(source_obj)
        except Exception as exc:
            reporter = getattr(self, "report", None)
            if callable(reporter):
                reporter({"ERROR"}, f"Rail overlay cleanup failed: {exc}")

    def _discard_session_resources(self, context, restore_scene):
        """Blender cleanup-port; terminal ordering задаёт controller."""

        self._stop_gpu_preview()
        if not getattr(self, "_modal_resources_discarded", False):
            session = getattr(self, "_decal_session", None)
            source_obj = (
                session.request.source_obj if session is not None else None
            )
            if source_obj is not None:
                try:
                    remove_decal_preview_object(
                        self.mode,
                        source_obj,
                        session.preview_state,
                    )
                except Exception as exc:
                    reporter = getattr(self, "report", None)
                    if callable(reporter):
                        reporter(
                            {"ERROR"}, f"Preview cleanup failed: {exc}"
                        )
            self._modal_resources_discarded = True
        self._clear_modal_rail_visualization()
        if restore_scene:
            scene = getattr(context, "scene", None)
            scene_settings = getattr(scene, "hotspotuv_settings", None)
            if scene_settings is not None:
                self._restore_modal_scene_targets(scene_settings)
        self._clear_modal_header()

    def cancel(self, context):
        """Blender callback при принудительном завершении modal/window."""

        session = getattr(self, "_decal_session", None)
        if session is not None:
            session.cancel()

    def invoke(self, context, event):
        self._decal_session = None
        self._modal_rail_visualization_shown = False
        self._modal_rail_visualization_cleared = False
        self._modal_resources_discarded = False
        archived_reason = decal_mode_archived_reason(self.mode)
        if archived_reason:
            self.report({"ERROR"}, archived_reason)
            return {"CANCELLED"}
        source_obj = context.active_object
        interactive_mode = self.mode in {"TOP", "BOTTOM", "CORNERS", "SEAMS"}
        edge_select_is_immediate = (
            context.window is not None
            and self.mode != "SEAMS"
            and _is_edge_select_mode(context, source_obj)
        )
        if (
            context.window is None
            or not interactive_mode
            or edge_select_is_immediate
        ):
            return self.execute(context)

        try:
            request = _prepare_decal_generation(context, self.mode)
            preview_state = DecalPreviewState()
            self._modal_area = context.area
            self._modal_gpu_preview = None
            session = self._create_session(
                context,
                request,
                preview_state=preview_state,
            )
            decal_plan = session.compile()
            if (
                self.mode == "SEAMS"
                and decal_plan is not None
                and getattr(decal_plan, "backend_partitions", None)
            ):
                display_mode = str(
                    getattr(
                        context.scene.hotspotuv_settings,
                        "decal_preview_display",
                        "GPU_TEXTURED",
                    )
                )
                controller = create_gpu_preview(
                    request.source_obj,
                    display_mode,
                )
                if controller is not None and controller.start():
                    self._modal_gpu_preview = controller
            self._configure_modal_drag_targets(request.world_settings)
            evaluation = session.begin()
            generation = evaluation.result
            if generation.status != PreviewStatus.UPDATED:
                level = (
                    {"ERROR"}
                    if generation.status == PreviewStatus.ERROR
                    else {"WARNING"}
                )
                session.cancel()
                self.report(
                    level,
                    generation.reason
                    or "No decals generated (check selection / wall geometry)",
                )
                return {"CANCELLED"}

            self._show_modal_rail_visualization()
            self._modal_created = _generation_result_names(generation)
            drag_anchor = _warp_decal_drag_cursor(
                context,
                request.source_obj,
                event,
            )
            self._rebase_modal_drag(
                "W", drag_anchor[0], bool(getattr(event, "shift", False))
            )
            self._set_modal_header(self._modal_current_value)
            context.window_manager.modal_handler_add(self)
            return {"RUNNING_MODAL"}
        except ValueError as exc:
            session = getattr(self, "_decal_session", None)
            if session is not None:
                session.cancel()
            self.report({"WARNING"}, str(exc))
            return {"CANCELLED"}
        except Exception as exc:
            session = getattr(self, "_decal_session", None)
            if session is not None:
                session.cancel()
            self.report({"ERROR"}, f"Generate Decals failed: {exc}")
            return {"CANCELLED"}

    def modal(self, context, event):
        event_type = event.type
        event_mouse_x = getattr(
            event,
            "mouse_x",
            getattr(
                self,
                "_modal_last_mouse_x",
                getattr(self, "_modal_start_mouse", 0),
            ),
        )
        self._modal_last_mouse_x = event_mouse_x
        precise_mode = bool(
            getattr(
                event,
                "shift",
                getattr(self, "_modal_precise_mode", False),
            )
        )
        targets = getattr(self, "_modal_drag_targets", {})
        disabled_targets = getattr(
            self, "_modal_disabled_drag_targets", {}
        )
        if (
            event_type in disabled_targets
            and getattr(event, "value", "PRESS") == "PRESS"
        ):
            self._set_modal_header(
                self._modal_current_value,
                getattr(self, "_modal_corner_target_block_reason", ""),
            )
            return {"RUNNING_MODAL"}
        if (
            event_type in targets
            and getattr(event, "value", "PRESS") == "PRESS"
        ):
            self._rebase_modal_drag(event_type, event_mouse_x, precise_mode)
            self._set_modal_header(self._modal_current_value)
            return {"RUNNING_MODAL"}
        if precise_mode != getattr(self, "_modal_precise_mode", False):
            target = self._active_modal_drag_target()
            self._rebase_modal_drag(target.key, event_mouse_x, precise_mode)
            self._set_modal_header(self._modal_current_value)
            return {"RUNNING_MODAL"}

        if event.type == "MOUSEMOVE":
            session = getattr(self, "_decal_session", None)
            if session is None:
                self.report({"ERROR"}, "DECAL_SESSION_MISSING")
                return {"CANCELLED"}
            target = self._active_modal_drag_target()
            new_value = _decal_drag_target_value(
                target,
                self._modal_base_value,
                event.mouse_x - self._modal_start_mouse,
                precise=getattr(self, "_modal_precise_mode", False),
            )
            if (
                abs(new_value - self._modal_current_value) < 1e-9
                and not session.last_preview_failure
            ):
                return {"RUNNING_MODAL"}
            settings = replace(
                session.current_world_settings,
                **{target.settings_field: new_value},
            )
            # Raw MOUSEMOVE никогда не компилирует: controller вызывает
            # только evaluate-port текущего compile-static plan.
            evaluation = session.preview(settings)
            generation = evaluation.result
            if generation.status != PreviewStatus.UPDATED:
                reason = generation.reason or "generation was not updated"
                self._set_modal_header(
                    (
                        new_value
                        if session.pending_recompile_request is not None
                        else self._modal_current_value
                    ),
                    f"{generation.status.value}: {reason}",
                )
                return {"RUNNING_MODAL"}
            setattr(
                context.scene.hotspotuv_settings,
                target.scene_property,
                new_value,
            )
            self._modal_created = _generation_result_names(generation)
            self._modal_current_value = new_value
            self._set_modal_header(new_value)
            return {"RUNNING_MODAL"}

        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            session = getattr(self, "_decal_session", None)
            if session is not None:
                session.cancel()
            return {"CANCELLED"}

        if event.type in {"LEFTMOUSE", "RET", "NUMPAD_ENTER"} and event.value == "PRESS":
            session = getattr(self, "_decal_session", None)
            if session is None:
                self.report({"ERROR"}, "DECAL_SESSION_MISSING")
                return {"CANCELLED"}
            self._stop_gpu_preview()
            self._modal_gpu_preview = None
            blocked_reason = (
                session.last_preview_failure
                if not session.is_confirmable
                else ""
            )
            controlled_recompile = session.pending_recompile_request is not None
            evaluation = session.confirm()
            generation = evaluation.result
            if generation.status != PreviewStatus.UPDATED:
                reason = generation.reason or generation.status.value
                message = (
                    "Current SEAMS preview failed; confirmation cancelled: "
                    + blocked_reason
                    if blocked_reason
                    else "Final decal rebuild failed: " + reason
                )
                self.report({"ERROR"}, message)
                return {"CANCELLED"}
            if controlled_recompile:
                target = self._active_modal_drag_target()
                final_value = getattr(
                    evaluation.world_settings,
                    target.settings_field,
                )
                setattr(
                    context.scene.hotspotuv_settings,
                    target.scene_property,
                    final_value,
                )
                self._modal_current_value = final_value
            self._modal_created = _generation_result_names(generation)
            self._report_created(
                self._modal_created,
                session.request,
                f"{self._modal_label}:{self._modal_value_text()}",
            )
            return {"FINISHED"}

        if event.type in {"MIDDLEMOUSE", "WHEELUPMOUSE", "WHEELDOWNMOUSE"}:
            return {"PASS_THROUGH"}
        return {"RUNNING_MODAL"}

    def execute(self, context):
        self._decal_session = None
        archived_reason = decal_mode_archived_reason(self.mode)
        if archived_reason:
            self.report({"ERROR"}, archived_reason)
            return {"CANCELLED"}
        try:
            request = _prepare_decal_generation(context, self.mode)
            session = self._create_session(
                context,
                request,
                preview_state=None,
            )
            generation = session.execute().result
            if generation.status != PreviewStatus.UPDATED:
                level = (
                    {"ERROR"}
                    if generation.status == PreviewStatus.ERROR
                    else {"WARNING"}
                )
                self.report(
                    level,
                    generation.reason
                    or "No decals generated (check selection / wall geometry)",
                )
                return {"CANCELLED"}
            created = _generation_result_names(generation)
            self._report_created(created, request)
            return {"FINISHED"}
        except ValueError as exc:
            self.report({"WARNING"}, str(exc))
            return {"CANCELLED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Generate Decals failed: {exc}")
            return {"CANCELLED"}


# ============================================================
# ENVELOPE DEBUG — EXACT PLANAR, STATIC
# ============================================================

def _envelope_debug_outcome_value(value):
    return str(value.value) if hasattr(value, "value") else str(value)


def _envelope_stage_summary_text(profile):
    summary = profile.stage_summary()
    return (
        f"Topology: {summary['topology']}/{summary['total']} | "
        f"Metric: {summary['metric']}/{summary['total']} | "
        f"Raw: {summary['raw']}/{summary['total']} | "
        f"Resolved: {summary['resolved']}/{summary['total']}"
    )


def _envelope_runtime_key(value):
    as_pointer = getattr(value, "as_pointer", None)
    return int(as_pointer()) if callable(as_pointer) else id(value)


def _envelope_debug_session(context):
    from .envelope_debug_session import EnvelopeDebugSessionController

    window_manager = context.window_manager
    controller = getattr(
        window_manager,
        "_cftuv_envelope_debug_session",
        None,
    )
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(
            window_manager,
            "_cftuv_envelope_debug_session",
            controller,
        )
    return controller


def _draw_envelope_debug_box(layout, s):
    """Панель Envelope Debug. Вынесена из `draw`: она растёт своей жизнью."""

    layout.separator()
    envelope_box = layout.box()
    envelope_box.label(
        text="Envelope Debug (Staged)",
        icon="GREASEPENCIL",
    )
    envelope_box.label(
        text="Edit Mode / Edge Select / whole chain",
        icon="INFO",
    )
    envelope_box.prop(s, "envelope_debug_engine")
    envelope_box.prop(s, "envelope_debug_alpha")
    envelope_box.operator(
        "hotspotuv.build_envelope_topology_debug",
        text="Build Topology Debug",
        icon="OUTLINER_DATA_MESH",
    )
    envelope_box.operator(
        "hotspotuv.build_exact_reference_envelope_debug",
        text="Build Exact Reference Envelope Debug",
        icon="PLAY",
    )
    row = envelope_box.row(align=True)
    row.operator(
        "hotspotuv.clear_envelope_debug",
        text="Clear Envelope Debug",
        icon="X",
    )
    envelope_box.label(text=s.envelope_debug_status)
    if s.envelope_debug_stage_summary:
        envelope_box.label(text=s.envelope_debug_stage_summary)
    if s.envelope_debug_domain_status:
        envelope_box.label(text=s.envelope_debug_domain_status)
    if s.envelope_debug_queue_timing:
        envelope_box.label(text=s.envelope_debug_queue_timing)
    if s.envelope_debug_outcome:
        envelope_box.label(
            text=f"Outcome: {s.envelope_debug_outcome}",
        )
    visibility = envelope_box.grid_flow(
        row_major=True,
        columns=2,
        even_columns=True,
        align=True,
    )
    visibility.prop(s, "envelope_debug_show_domains")
    visibility.prop(s, "envelope_debug_show_chains")
    visibility.prop(s, "envelope_debug_show_supports")
    visibility.prop(s, "envelope_debug_show_envelopes")
    visibility.prop(s, "envelope_debug_show_raw")
    visibility.prop(s, "envelope_debug_show_readings")
    visibility.prop(s, "envelope_debug_show_equality")
    visibility.prop(s, "envelope_debug_show_resolved")
    visibility.prop(s, "envelope_debug_show_diagnostics")
    visibility.prop(s, "envelope_debug_show_labels")
    visibility.prop(s, "envelope_debug_show_queue")


def _remember_queue_session(
    controller,
    source_name,
    topology_scene,
    exact_scenes,
    evaluation,
):
    """Запомнить подготовки очереди, чтобы ползунок нашёл их тёплыми."""

    from .envelope_debug_session import QueueSessionStateV1
    from .envelope_queue_export import build_queue_scene

    queue_domains = evaluation.queue_domains
    if not queue_domains:
        return None
    controller.remember_queue_session(
        QueueSessionStateV1(
            str(source_name),
            topology_scene,
            tuple(exact_scenes),
            tuple(
                (item.patch_id, item.patch_domain_id, item.preparation)
                for item in queue_domains
                if item.preparation is not None
            ),
            evaluation.receipts,
        )
    )
    return build_queue_scene(queue_domains)


class _EnvelopeDebugBuildBase:
    exact_reference = False
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return (
            obj is not None
            and obj.type == "MESH"
            and obj.mode == "EDIT"
            and bool(context.tool_settings.mesh_select_mode[1])
        )

    def execute(self, context):
        settings = context.scene.hotspotuv_settings
        source_obj = context.active_object
        if source_obj is None or source_obj.type != "MESH":
            self.report({"ERROR"}, "Select a mesh object")
            return {"CANCELLED"}

        from .envelope_debug_profile import (
            EnvelopeDebugProfileBuilderV1,
            EnvelopeDomainStage,
        )
        from .envelope_debug_renderer import (
            clear_envelope_debug,
            render_envelope_topology_debug_scene,
            render_staged_envelope_debug,
            visibility_from_settings,
        )

        previous_source = str(settings.envelope_debug_source_object).strip()
        for source_name in {
            previous_source,
            source_obj.name,
        }:
            if source_name:
                clear_envelope_debug(source_name)
        settings.envelope_debug_source_object = source_obj.name
        settings.envelope_debug_status = "Building topology..."
        settings.envelope_debug_outcome = ""
        settings.envelope_debug_stage_summary = ""
        settings.envelope_debug_domain_status = ""
        settings.envelope_debug_queue_timing = ""
        engine = str(settings.envelope_debug_engine)

        source_bm = bmesh.from_edit_mesh(source_obj.data)
        source_bm.edges.ensure_lookup_table()
        selected_edge_indices = _capture_selected_physical_edges(source_bm)
        if not selected_edge_indices:
            outcome = "ENVELOPE_DEBUG_EMPTY_SELECTION"
            settings.envelope_debug_status = (
                "Failed: select a whole PhysicalChain"
            )
            settings.envelope_debug_outcome = outcome
            self.report({"WARNING"}, outcome)
            return {"CANCELLED"}

        profile = EnvelopeDebugProfileBuilderV1(
            source_obj.name,
            (
                ("EXACT_REFERENCE" if engine == "LEGACY" else engine)
                if self.exact_reference
                else "TOPOLOGY"
            ),
        )
        controller = _envelope_debug_session(context)
        queue_scene = None
        source_object_key = _envelope_runtime_key(source_obj)
        source_data_key = _envelope_runtime_key(source_obj.data)
        source_bm.faces.ensure_lookup_table()
        face_indices = tuple(face.index for face in source_bm.faces)
        source_revision = source_revision_from_bmesh(
            source_bm,
            source_obj,
            face_indices,
        )
        try:
            analysis_bundle = controller.get_analysis_bundle(
                source_object_key,
                source_data_key,
                source_revision,
                lambda: build_analysis_bundle(
                    source_bm,
                    face_indices,
                    source_obj,
                ),
                profile=profile,
            )
            if self.exact_reference:
                from .envelope_host_adapter import (
                    evaluate_envelope_debug_staged,
                )

                evaluation = evaluate_envelope_debug_staged(
                    analysis_bundle,
                    frozenset(selected_edge_indices),
                    float(settings.envelope_debug_alpha),
                    profile=profile,
                    controller=controller,
                    source_object_key=source_object_key,
                    source_data_key=source_data_key,
                    engine=engine,
                )
                topology_scene = evaluation.topology_scene
                exact_scenes = evaluation.exact_debug_scenes
                receipts = evaluation.receipts
                queue_scene = _remember_queue_session(
                    controller,
                    source_obj.name,
                    topology_scene,
                    exact_scenes,
                    evaluation,
                )
            else:
                from .envelope_host_adapter import (
                    build_envelope_topology_debug_scene,
                )

                topology_export = controller.get_topology_export(
                    analysis_bundle,
                    source_object_key,
                    source_data_key,
                    profile=profile,
                )
                topology_scene = build_envelope_topology_debug_scene(
                    analysis_bundle,
                    frozenset(selected_edge_indices),
                    profile=profile,
                    topology_export=topology_export,
                )
                exact_scenes = ()
                receipts = profile.snapshot().receipts
        except Exception as exc:
            clear_envelope_debug(source_obj)
            settings.envelope_debug_status = "Topology build failed"
            settings.envelope_debug_outcome = type(exc).__name__
            self.report({"ERROR"}, f"Envelope topology failed: {exc}")
            return {"CANCELLED"}
        finally:
            _restore_edge_selection(source_obj, selected_edge_indices)

        try:
            if self.exact_reference:
                summary = render_staged_envelope_debug(
                    topology_scene,
                    exact_scenes,
                    source_obj,
                    visibility_by_layer=visibility_from_settings(settings),
                    profile=profile,
                    queue_scene=queue_scene,
                )
            else:
                summary = render_envelope_topology_debug_scene(
                    topology_scene,
                    source_obj,
                    visibility_by_layer=visibility_from_settings(settings),
                    profile=profile,
                )
        except Exception as exc:
            clear_envelope_debug(source_obj)
            settings.envelope_debug_status = "Failed during GP render"
            settings.envelope_debug_outcome = type(exc).__name__
            self.report({"ERROR"}, f"Envelope Debug render failed: {exc}")
            return {"CANCELLED"}

        final_profile = profile.snapshot()
        settings.envelope_debug_stage_summary = (
            _envelope_stage_summary_text(final_profile)
        )
        missing = tuple(
            item
            for item in receipts
            if item.stage
            not in {
                EnvelopeDomainStage.RESOLVED,
                EnvelopeDomainStage.QUEUE_RESOLVED,
            }
        )
        if queue_scene is not None:
            from .envelope_queue_export import queue_timing_text

            settings.envelope_debug_queue_timing = queue_timing_text(
                queue_scene
            )
        if not self.exact_reference:
            settings.envelope_debug_status = (
                f"Topology built: {summary.stroke_count} strokes"
            )
            settings.envelope_debug_outcome = "TOPOLOGY_READY"
            settings.envelope_debug_domain_status = (
                "Exact Envelope not requested"
            )
        elif missing:
            first = missing[0]
            settings.envelope_debug_status = (
                "Topology built; Exact Envelope unavailable "
                f"for {len(missing)}/{len(receipts)} domains"
            )
            settings.envelope_debug_outcome = first.outcome
            settings.envelope_debug_domain_status = (
                f"Patch {first.patch_id}: {first.stage.value}"
            )
        else:
            settings.envelope_debug_status = (
                f"Topology and Exact Envelope built: "
                f"{summary.stroke_count} strokes"
            )
            settings.envelope_debug_outcome = "RESOLVED"
            settings.envelope_debug_domain_status = (
                f"All {len(receipts)} domains resolved"
            )

        report_level = {"WARNING"} if missing and self.exact_reference else {"INFO"}
        self.report(
            report_level,
            (
                f"Envelope Debug: {summary.stroke_count} strokes, "
                f"{summary.point_count} points, {len(receipts)} domains"
            ),
        )
        return {"FINISHED"}


class HOTSPOTUV_OT_BuildEnvelopeTopologyDebug(
    _EnvelopeDebugBuildBase,
    bpy.types.Operator,
):
    bl_idname = "hotspotuv.build_envelope_topology_debug"
    bl_label = "Build Topology Debug"
    bl_description = (
        "Render Patch, BoundaryLoop, PhysicalChain, and directed ChainUse "
        "host facts without loading the exact kernel or SymPy"
    )


class HOTSPOTUV_OT_BuildExactReferenceEnvelopeDebug(
    _EnvelopeDebugBuildBase,
    bpy.types.Operator,
):
    bl_idname = "hotspotuv.build_exact_reference_envelope_debug"
    bl_label = "Build Exact Reference Envelope Debug"
    bl_description = (
        "Keep topology visible while evaluating exact reference Envelope "
        "stages independently for every selected PatchDomain"
    )
    exact_reference = True


class HOTSPOTUV_OT_BuildEnvelopeDebug(
    _EnvelopeDebugBuildBase,
    bpy.types.Operator,
):
    """Compatibility alias for saved scripts from the V0-B debug slice."""

    bl_idname = "hotspotuv.build_envelope_debug"
    bl_label = "Build Exact Reference Envelope Debug"
    exact_reference = True


class HOTSPOTUV_OT_ClearEnvelopeDebug(bpy.types.Operator):
    bl_idname = "hotspotuv.clear_envelope_debug"
    bl_label = "Clear Envelope Debug"
    bl_description = "Remove Envelope Debug GP object and JSON sidecar"
    bl_options = {"REGISTER"}

    def execute(self, context):
        settings = context.scene.hotspotuv_settings
        active_obj = context.active_object
        source_names = {
            str(settings.envelope_debug_source_object).strip(),
            (
                active_obj.name
                if active_obj is not None and active_obj.type == "MESH"
                else ""
            ),
        }
        from .envelope_debug_renderer import clear_envelope_debug

        for source_name in source_names:
            if source_name:
                clear_envelope_debug(source_name)
        settings.envelope_debug_source_object = ""
        settings.envelope_debug_status = "Cleared"
        settings.envelope_debug_outcome = ""
        settings.envelope_debug_stage_summary = ""
        settings.envelope_debug_domain_status = ""
        settings.envelope_debug_queue_timing = ""
        self.report({"INFO"}, "Envelope Debug cleared")
        return {"FINISHED"}


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

        build_box = layout.box()
        build_box.label(
            text=f"Branch: {ADDON_BUILD_INFO.branch}", icon="INFO"
        )
        build_box.label(
            text=f"Code commit: {ADDON_BUILD_INFO.short_commit}"
        )

        # --- Settings ---
        col = layout.column(align=True)
        col.prop(s, "target_texel_density")
        col.prop(s, "texture_size")
        col.prop(s, "uv_scale")
        col.prop(s, "uv_range_limit")

        # --- Preview ---
        layout.separator()
        col = layout.column(align=True)
        col.label(text="Preview:")
        row = col.row(align=True)
        row.prop(s, "phase1_make_seams_by_sharp", text="")
        row.prop(s, "straighten_strips", text="", icon="SNAP_EDGE")
        row.operator(
            "hotspotuv.solve_phase1_preview", text="Solve Phase 1 Preview", icon="UV"
        )
        col.operator(
            "hotspotuv.clean_non_manifold_edges",
            text="Clean Non-Manifold Edges",
            icon="MESH_DATA",
        )

        # --- Envelope Debug ---
        _draw_envelope_debug_box(layout, s)

        # --- Decals ---
        layout.separator()
        col = layout.column(align=True)
        col.label(text="Decals:")
        col.prop(s, "decal_width_corner")
        col.prop(s, "decal_width_seam")
        col.prop(s, "decal_height_trim")
        col.prop(s, "decal_offset")
        col.prop(s, "decal_chart_distortion_budget")
        row = col.row(align=True)
        row.prop(s, "decal_corner_join_mode")
        col.prop(s, "decal_dynamic_corner_bands")
        if s.decal_dynamic_corner_bands:
            col.prop(s, "decal_corner_miter_angle")
            col.prop(s, "decal_corner_kite_angle")
        col.prop(s, "decal_corner_acute_split_angle")
        if s.decal_dynamic_corner_bands:
            col.prop(s, "decal_corner_hairpin_angle")
        col.prop(s, "decal_corner_miter_limit")
        col.prop(s, "decal_preview_display")
        row = col.row(align=True)
        row.enabled = False
        op = row.operator(
            "hotspotuv.generate_decals",
            text="Decal Top (Archived)",
            icon="TRIA_UP",
        )
        op.mode = "TOP"
        row = col.row(align=True)
        row.enabled = False
        op = row.operator(
            "hotspotuv.generate_decals",
            text="Decal Bottom (Archived)",
            icon="TRIA_DOWN",
        )
        op.mode = "BOTTOM"
        row = col.row(align=True)
        row.enabled = False
        op = row.operator(
            "hotspotuv.generate_decals",
            text="Decal Corners (Archived)",
            icon="MOD_BEVEL",
        )
        op.mode = "CORNERS"
        op = col.operator(
            "hotspotuv.generate_decals", text="Decal Seams", icon="MOD_EDGESPLIT"
        )
        op.mode = "SEAMS"

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
        row = col.row(align=True)
        row.prop(s, "dbg_verbose_console", text="Verbose Console", toggle=True)
        row.prop(s, "dbg_show_advanced_debug", text="Advanced Tools", toggle=True)
        col.operator(
            "hotspotuv.save_regression_snapshot",
            text="Save Regression Snapshot",
            icon="TEXT",
        )
        if s.dbg_replay_active:
            col.operator(
                "hotspotuv.frontier_replay",
                text="Frontier Replay: ON",
                icon="PAUSE",
                depress=True,
            )
        else:
            col.operator("hotspotuv.frontier_replay", text="Frontier Replay: OFF", icon="PLAY")
        if s.dbg_show_advanced_debug:
            col.operator("hotspotuv.flow_debug", text="Flow Debug", icon="SORTSIZE")

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
            has_gp = is_gp_debug_object(gp_obj)

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
                     ('Overlay_Centers', 'Centers'),
                     ('Overlay_Labels', 'Chain Labels')],
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
            # Chain Labels — коллекция, а не GP layer
            if layer_name == 'Overlay_Labels':
                from .debug import _LABEL_COLLECTION_NAME
                if _LABEL_COLLECTION_NAME in bpy.data.collections:
                    label_col = bpy.data.collections[_LABEL_COLLECTION_NAME]
                    row = box.row(align=True)
                    row.separator(factor=2.0)
                    vis_icon = 'HIDE_OFF' if not label_col.hide_viewport else 'HIDE_ON'
                    op = row.operator(
                        "hotspotuv.debug_toggle_layer", text="", icon=vis_icon
                    )
                    op.layer_name = layer_name
                    row.label(text=layer_label)
                continue

            layer = get_gp_layer(gp_data, layer_name)
            if layer is not None:
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
    HOTSPOTUV_AddonPreferences,
    HOTSPOTUV_Settings,
    # Active operators
    HOTSPOTUV_OT_DebugAnalysis,
    HOTSPOTUV_OT_DebugClear,
    HOTSPOTUV_OT_DebugToggleLayer,
    HOTSPOTUV_OT_DebugToggleGroup,
    HOTSPOTUV_OT_FlowDebug,
    HOTSPOTUV_OT_ScaffoldDebug,
    HOTSPOTUV_OT_SaveRegressionSnapshot,
    HOTSPOTUV_OT_FrontierReplay,
    HOTSPOTUV_OT_SolvePhase1Preview,
    HOTSPOTUV_OT_CleanNonManifoldEdges,
    HOTSPOTUV_OT_GenerateDecals,
    HOTSPOTUV_OT_BuildEnvelopeTopologyDebug,
    HOTSPOTUV_OT_BuildExactReferenceEnvelopeDebug,
    HOTSPOTUV_OT_BuildEnvelopeDebug,
    HOTSPOTUV_OT_ClearEnvelopeDebug,
    # Legacy stubs (disabled)
    HOTSPOTUV_OT_UnwrapFaces,
    HOTSPOTUV_OT_ManualDock,
    HOTSPOTUV_OT_SelectSimilar,
    HOTSPOTUV_OT_StackSimilar,
    # Panel
    HOTSPOTUV_PT_Panel,
)


def register():
    from .envelope_debug_session import (
        register_window_manager_session_attribute,
    )

    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.hotspotuv_settings = PointerProperty(type=HOTSPOTUV_Settings)
    register_window_manager_session_attribute()
    if _frontier_replay_frame_handler not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(_frontier_replay_frame_handler)


def unregister():
    from .envelope_debug_session import (
        unregister_window_manager_session_attribute,
    )

    if _frontier_replay_frame_handler in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(_frontier_replay_frame_handler)
    # Cleanup GP debug objects
    for obj in list(bpy.data.objects):
        if obj.name.startswith(ENVELOPE_DEBUG_GP_PREFIX):
            GreasePencilDebugWriter.clear_object(obj.name)
    for obj in list(bpy.data.objects):
        if (
            obj.name.startswith(GP_DEBUG_PREFIX)
            and not obj.name.startswith(ENVELOPE_DEBUG_GP_PREFIX)
        ):
            bpy.data.objects.remove(obj, do_unlink=True)
    for text in list(bpy.data.texts):
        if (
            text.name.startswith("CFTUV_EnvelopeDebug_")
            or text.name.startswith("CFTUV_EnvelopeProfile_")
        ):
            bpy.data.texts.remove(text)
    for window_manager in bpy.data.window_managers:
        controller = getattr(
            window_manager,
            "_cftuv_envelope_debug_session",
            None,
        )
        if controller is not None:
            controller.clear()
            delattr(
                window_manager,
                "_cftuv_envelope_debug_session",
            )
    unregister_window_manager_session_attribute()
    if hasattr(bpy.types.Scene, "hotspotuv_settings"):
        del bpy.types.Scene.hotspotuv_settings
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
