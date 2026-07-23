"""Blender runtime projection of immutable EnvelopeDebugSceneV1 records."""

from __future__ import annotations

import json
from dataclasses import dataclass

import bpy
from mathutils import Vector

from .debug import (
    ENVELOPE_DEBUG_GP_PREFIX,
    GreasePencilDebugWriter,
)


ENVELOPE_DEBUG_TEXT_PREFIX = "CFTUV_EnvelopeDebug_"
ENVELOPE_DEBUG_LABEL_LAYER = "ENV_LABELS"

ENVELOPE_DEBUG_LAYER_STYLES = {
    "ENV_00_PATCH_DOMAIN": ((0.90, 0.90, 0.90, 1.0), 5),
    "ENV_01_HOLES": ((0.20, 0.45, 1.00, 1.0), 6),
    "ENV_02_BARRIERS": ((1.00, 0.15, 0.15, 1.0), 8),
    "ENV_10_PHYSICAL_CHAINS": ((1.00, 0.80, 0.05, 1.0), 7),
    "ENV_11_CHAIN_USES": ((1.00, 0.45, 0.05, 1.0), 4),
    "ENV_20_SOURCE_SUPPORTS": ((0.05, 0.85, 0.95, 1.0), 5),
    "ENV_21_MOVING_FRONTS": ((0.20, 1.00, 0.45, 1.0), 5),
    "ENV_22_HIDDEN_SUPPORTS": ((0.85, 0.20, 1.00, 1.0), 7),
    "ENV_30_ENVELOPE_INSTANCES": ((0.10, 0.95, 0.70, 1.0), 4),
    "ENV_40_RAW_COVERAGE": ((0.15, 0.80, 1.00, 1.0), 8),
    "ENV_50_INTERACTION_COMPONENTS": ((0.80, 0.45, 1.00, 1.0), 4),
    "ENV_51_FRONT_READINGS": ((1.00, 0.30, 0.80, 1.0), 6),
    "ENV_52_EQUALITY_LOCI": ((1.00, 1.00, 1.00, 1.0), 10),
    "ENV_60_RESOLVED_COVERAGE": ((0.10, 1.00, 0.15, 1.0), 9),
    "ENV_61_RETAINED_REGIONS": ((0.35, 1.00, 0.35, 1.0), 5),
    "ENV_62_REMOVED_REGIONS": ((1.00, 0.25, 0.25, 1.0), 5),
    "ENV_70_EVENT_ANCHORS": ((1.00, 1.00, 0.10, 1.0), 9),
    "ENV_80_DIAGNOSTICS": ((1.00, 0.05, 0.05, 1.0), 11),
    ENVELOPE_DEBUG_LABEL_LAYER: ((1.00, 1.00, 1.00, 1.0), 4),
}

_VISIBILITY_GROUPS = {
    "envelope_debug_show_domains": (
        "ENV_00_PATCH_DOMAIN",
        "ENV_01_HOLES",
        "ENV_02_BARRIERS",
    ),
    "envelope_debug_show_chains": (
        "ENV_10_PHYSICAL_CHAINS",
        "ENV_11_CHAIN_USES",
    ),
    "envelope_debug_show_supports": (
        "ENV_20_SOURCE_SUPPORTS",
        "ENV_21_MOVING_FRONTS",
        "ENV_22_HIDDEN_SUPPORTS",
    ),
    "envelope_debug_show_envelopes": ("ENV_30_ENVELOPE_INSTANCES",),
    "envelope_debug_show_raw": ("ENV_40_RAW_COVERAGE",),
    "envelope_debug_show_readings": (
        "ENV_50_INTERACTION_COMPONENTS",
        "ENV_51_FRONT_READINGS",
    ),
    "envelope_debug_show_equality": (
        "ENV_52_EQUALITY_LOCI",
        "ENV_70_EVENT_ANCHORS",
    ),
    "envelope_debug_show_resolved": (
        "ENV_60_RESOLVED_COVERAGE",
        "ENV_61_RETAINED_REGIONS",
        "ENV_62_REMOVED_REGIONS",
    ),
    "envelope_debug_show_diagnostics": ("ENV_80_DIAGNOSTICS",),
    "envelope_debug_show_labels": (ENVELOPE_DEBUG_LABEL_LAYER,),
}


@dataclass(frozen=True, slots=True)
class EnvelopeDebugRenderSummaryV1:
    object_name: str
    text_name: str
    layer_count: int
    stroke_count: int


def envelope_debug_object_name(source_obj_or_name) -> str:
    name = (
        source_obj_or_name
        if isinstance(source_obj_or_name, str)
        else source_obj_or_name.name
    )
    return ENVELOPE_DEBUG_GP_PREFIX + str(name)


def envelope_debug_text_name(source_obj_or_name) -> str:
    name = (
        source_obj_or_name
        if isinstance(source_obj_or_name, str)
        else source_obj_or_name.name
    )
    return ENVELOPE_DEBUG_TEXT_PREFIX + str(name) + ".json"


def visibility_from_settings(settings) -> dict[str, bool]:
    result = {name: True for name in ENVELOPE_DEBUG_LAYER_STYLES}
    for property_name, layer_names in _VISIBILITY_GROUPS.items():
        visible = bool(getattr(settings, property_name, True))
        for layer_name in layer_names:
            result[layer_name] = visible
    return result


def apply_envelope_debug_visibility(source_obj_or_name, settings) -> bool:
    return GreasePencilDebugWriter.set_layer_visibility(
        envelope_debug_object_name(source_obj_or_name),
        visibility_from_settings(settings),
    )


def clear_envelope_debug(source_obj_or_name) -> None:
    GreasePencilDebugWriter.clear_object(
        envelope_debug_object_name(source_obj_or_name),
        material_prefix="CFTUV_EnvelopeDebug_",
    )
    text = bpy.data.texts.get(envelope_debug_text_name(source_obj_or_name))
    if text is not None:
        bpy.data.texts.remove(text)


def _vector3(value) -> Vector:
    return Vector((float(value.x), float(value.y), float(value.z)))


def _identity_value(value) -> str:
    return value.value if hasattr(value, "value") else str(value)


def _lineage_values(record) -> list[str]:
    return sorted(
        _identity_value(item)
        for item in getattr(record, "source_lineage_ids", ())
    )


def _stage_value(record) -> str:
    return _identity_value(record.stage)


def _kind_value(record) -> str:
    return _identity_value(record.kind)


def _lift_point(point, frame, layer_ordinal, sympy) -> Vector:
    x = float(sympy.sympify(point.x_expression))
    y = float(sympy.sympify(point.y_expression))
    origin = _vector3(frame.origin)
    axis_u = _vector3(frame.axis_u)
    axis_v = _vector3(frame.axis_v)
    normal = _vector3(frame.normal)
    lift = 0.002 + 0.00035 * float(layer_ordinal)
    return origin + axis_u * x + axis_v * y + normal * lift


def _marker_segments(anchor, frame, layer_ordinal, sympy):
    center = _lift_point(anchor, frame, layer_ordinal, sympy)
    axis_u = _vector3(frame.axis_u)
    axis_v = _vector3(frame.axis_v)
    size = 0.025
    return (
        (center - axis_u * size, center + axis_u * size),
        (center - axis_v * size, center + axis_v * size),
    )


def _record_mapping(record, layer_name, stroke_index):
    return {
        "layer": layer_name,
        "stroke_index": stroke_index,
        "semantic_id": _identity_value(record.semantic_id),
        "stage": _stage_value(record),
        "kind": _kind_value(record),
        "source_lineage_ids": _lineage_values(record),
        "label": record.label,
    }


def _write_sidecar(text_name, payload) -> None:
    text = bpy.data.texts.get(text_name)
    if text is None:
        text = bpy.data.texts.new(text_name)
    else:
        text.clear()
    text.write(
        json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
        )
    )


def render_envelope_debug_scene(
    scene,
    source_obj,
    *,
    visibility_by_layer=None,
) -> EnvelopeDebugRenderSummaryV1:
    """Render one immutable DebugScene without changing semantic geometry."""

    import cftuv_envelope
    import sympy

    object_name = envelope_debug_object_name(source_obj)
    text_name = envelope_debug_text_name(source_obj)
    clear_envelope_debug(source_obj)
    writer = GreasePencilDebugWriter(source_obj, object_name)
    visibility = {
        name: True for name in ENVELOPE_DEBUG_LAYER_STYLES
    }
    visibility.update(visibility_by_layer or {})
    for layer_name, (color, _line_width) in ENVELOPE_DEBUG_LAYER_STYLES.items():
        writer.ensure_layer(
            layer_name,
            color,
            visible=visibility.get(layer_name, True),
        )

    frame_by_domain = {
        item.patch_domain_id: item for item in scene.patch_frames
    }
    layer_ordinals = {
        name: ordinal
        for ordinal, name in enumerate(ENVELOPE_DEBUG_LAYER_STYLES)
    }
    stroke_map = []

    def frame_for(record):
        return frame_by_domain[record.patch_domain_id]

    def add_path_record(record, exact_points, *, cyclic):
        layer_name = record.style_key
        frame = frame_for(record)
        points = [
            _lift_point(
                point,
                frame,
                layer_ordinals[layer_name],
                sympy,
            )
            for point in exact_points
        ]
        stroke_index = writer.add_path(
            layer_name,
            points,
            line_width=ENVELOPE_DEBUG_LAYER_STYLES[layer_name][1],
            cyclic=cyclic,
        )
        if stroke_index is not None:
            stroke_map.append(
                _record_mapping(record, layer_name, stroke_index)
            )

    for record in scene.paths:
        add_path_record(record, record.exact_points, cyclic=record.closed)
    for record in scene.loops:
        add_path_record(record, record.exact_points, cyclic=True)
    for record in scene.regions:
        add_path_record(record, record.outer_exact_points, cyclic=True)
        for hole in record.hole_exact_point_loops:
            add_path_record(record, hole, cyclic=True)

    for record in scene.points:
        frame = frame_for(record)
        for segment in _marker_segments(
            record.exact_point,
            frame,
            layer_ordinals[record.style_key],
            sympy,
        ):
            stroke_index = writer.add_path(
                record.style_key,
                segment,
                line_width=ENVELOPE_DEBUG_LAYER_STYLES[record.style_key][1],
            )
            stroke_map.append(
                _record_mapping(record, record.style_key, stroke_index)
            )

    for record in scene.labels:
        frame = frame_for(record)
        for segment in _marker_segments(
            record.exact_anchor,
            frame,
            layer_ordinals[ENVELOPE_DEBUG_LABEL_LAYER],
            sympy,
        ):
            stroke_index = writer.add_path(
                ENVELOPE_DEBUG_LABEL_LAYER,
                segment,
                line_width=ENVELOPE_DEBUG_LAYER_STYLES[
                    ENVELOPE_DEBUG_LABEL_LAYER
                ][1],
            )
            stroke_map.append(
                _record_mapping(
                    record,
                    ENVELOPE_DEBUG_LABEL_LAYER,
                    stroke_index,
                )
            )

    diagnostics = []
    for diagnostic in scene.diagnostics:
        diagnostic_payload = {
            "diagnostic_id": _identity_value(diagnostic.diagnostic_id),
            "patch_domain_id": (
                _identity_value(diagnostic.patch_domain_id)
                if diagnostic.patch_domain_id is not None
                else None
            ),
            "outcome": diagnostic.outcome,
            "severity": _identity_value(diagnostic.severity),
            "message": diagnostic.message,
            "source_lineage_ids": _lineage_values(diagnostic),
        }
        diagnostics.append(diagnostic_payload)
        if (
            diagnostic.exact_anchor is None
            or diagnostic.patch_domain_id not in frame_by_domain
        ):
            continue
        frame = frame_by_domain[diagnostic.patch_domain_id]
        for segment in _marker_segments(
            diagnostic.exact_anchor,
            frame,
            layer_ordinals["ENV_80_DIAGNOSTICS"],
            sympy,
        ):
            stroke_index = writer.add_path(
                "ENV_80_DIAGNOSTICS",
                segment,
                line_width=ENVELOPE_DEBUG_LAYER_STYLES[
                    "ENV_80_DIAGNOSTICS"
                ][1],
            )
            stroke_map.append(
                {
                    **diagnostic_payload,
                    "layer": "ENV_80_DIAGNOSTICS",
                    "stroke_index": stroke_index,
                }
            )

    request_ids = [_identity_value(item) for item in scene.request_ids]
    domain_ids = [_identity_value(item) for item in scene.patch_domain_ids]
    stage_counts = {}
    for record in (
        *scene.points,
        *scene.paths,
        *scene.loops,
        *scene.regions,
        *scene.labels,
    ):
        stage = _stage_value(record)
        stage_counts[stage] = stage_counts.get(stage, 0) + 1
    stage_counts["DIAGNOSTIC"] = len(scene.diagnostics)

    gp_obj = writer.object
    gp_obj["kernel_version"] = cftuv_envelope.__version__
    gp_obj["source_revision"] = _identity_value(scene.source_revision)
    gp_obj["decal_request_ids"] = json.dumps(request_ids)
    gp_obj["patch_domain_ids"] = json.dumps(domain_ids)
    gp_obj["requested_alpha"] = str(scene.requested_alpha.value)
    gp_obj["raw_coverage_digests"] = json.dumps(
        list(scene.raw_coverage_digests)
    )
    gp_obj["resolved_coverage_digests"] = json.dumps(
        list(scene.resolved_coverage_digests)
    )
    gp_obj["stage_outcomes"] = json.dumps(
        stage_counts,
        sort_keys=True,
    )

    sidecar = {
        "schema": "cftuv.envelope.debug_scene_runtime_sidecar.v1",
        "object_name": object_name,
        "kernel_version": cftuv_envelope.__version__,
        "source_revision": _identity_value(scene.source_revision),
        "decal_request_ids": request_ids,
        "patch_domain_ids": domain_ids,
        "requested_alpha": str(scene.requested_alpha.value),
        "raw_coverage_digests": list(scene.raw_coverage_digests),
        "resolved_coverage_digests": list(
            scene.resolved_coverage_digests
        ),
        "stage_outcomes": stage_counts,
        "strokes": stroke_map,
        "diagnostics": diagnostics,
    }
    _write_sidecar(text_name, sidecar)
    return EnvelopeDebugRenderSummaryV1(
        object_name,
        text_name,
        len(ENVELOPE_DEBUG_LAYER_STYLES),
        writer.stroke_count(),
    )


__all__ = (
    "ENVELOPE_DEBUG_LABEL_LAYER",
    "ENVELOPE_DEBUG_LAYER_STYLES",
    "EnvelopeDebugRenderSummaryV1",
    "apply_envelope_debug_visibility",
    "clear_envelope_debug",
    "envelope_debug_object_name",
    "envelope_debug_text_name",
    "render_envelope_debug_scene",
    "visibility_from_settings",
)
