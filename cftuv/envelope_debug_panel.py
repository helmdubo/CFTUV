"""Envelope Debug panel layout, isolated from operator execution."""

from __future__ import annotations


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
    envelope_box.prop(settings, "envelope_debug_alpha")
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


__all__ = ("draw_envelope_debug_box",)
