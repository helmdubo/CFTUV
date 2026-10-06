"""Envelope Debug panel layout, isolated from operator execution."""

from __future__ import annotations

from .envelope_alpha_preview_gp import draw_alpha_preview_row
from .envelope_kernel_backend import draw_kernel_backend_row
from .envelope_width_live import draw_decal_width_rows
from .envelope_worker_python import draw_worker_python_row


def draw_decal_mesh_rows(layout) -> None:
    """Продуктовый меш: кнопка, настройки и строки статуса (они лежат в сцене).

    Сцена берётся из `bpy.context`: у `draw_envelope_debug_box` контекста нет, а
    группа настроек оператора (`envelope_production_operator`) — свойство сцены,
    не `hotspotuv_settings`.
    """

    import bpy

    mesh_settings = getattr(bpy.context.scene, "hotspotuv_decal_mesh", None)
    if mesh_settings is None:
        return
    layout.separator()
    layout.operator(
        "hotspotuv.build_envelope_decal_mesh",
        text="Build Decal Mesh",
        icon="MESH_DATA",
    )
    draw_decal_width_rows(layout)
    row = layout.row(align=True)
    row.prop(mesh_settings, "offset")
    row.prop(mesh_settings, "material_name", text="")
    draw_kernel_backend_row(layout, mesh_settings)
    if mesh_settings.status:
        layout.label(text=mesh_settings.status)
    if mesh_settings.timing:
        layout.label(text=mesh_settings.timing)


def draw_envelope_debug_box(layout, settings) -> None:
    """Рисует диагностическую панель без владения геометрией и исполнением."""

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
    envelope_box.prop(settings, "envelope_debug_engine")
    envelope_box.prop(settings, "envelope_debug_fan_density")
    envelope_box.prop(settings, "envelope_debug_max_stretch")
    envelope_box.prop(settings, "envelope_debug_dissolve_uv_tolerance")
    envelope_box.prop(settings, "envelope_debug_workers")
    draw_worker_python_row(envelope_box)
    envelope_box.prop(settings, "envelope_debug_alpha")
    draw_alpha_preview_row(envelope_box)
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
    envelope_box.label(text=settings.envelope_debug_status)
    if settings.envelope_debug_stage_summary:
        envelope_box.label(text=settings.envelope_debug_stage_summary)
    if settings.envelope_debug_domain_status:
        envelope_box.label(text=settings.envelope_debug_domain_status)
    if settings.envelope_debug_queue_timing:
        envelope_box.label(text=settings.envelope_debug_queue_timing)
    if settings.envelope_debug_outcome:
        envelope_box.label(
            text=f"Outcome: {settings.envelope_debug_outcome}",
        )
    visibility = envelope_box.grid_flow(
        row_major=True,
        columns=2,
        even_columns=True,
        align=True,
    )
    for property_name in (
        "envelope_debug_show_domains",
        "envelope_debug_show_chains",
        "envelope_debug_show_supports",
        "envelope_debug_show_envelopes",
        "envelope_debug_show_raw",
        "envelope_debug_show_readings",
        "envelope_debug_show_equality",
        "envelope_debug_show_resolved",
        "envelope_debug_show_diagnostics",
        "envelope_debug_show_refused",
        "envelope_debug_show_labels",
        "envelope_debug_show_queue",
    ):
        visibility.prop(settings, property_name)
    draw_decal_mesh_rows(envelope_box)


__all__ = ("draw_decal_mesh_rows", "draw_envelope_debug_box")
