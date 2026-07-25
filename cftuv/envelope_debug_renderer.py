"""Blender runtime projection of immutable EnvelopeDebugSceneV1 records."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

import bpy
from mathutils import Vector

from .debug import (
    ENVELOPE_DEBUG_GP_PREFIX,
    GreasePencilDebugWriter,
)
from .envelope_debug_profile import EnvelopeDomainStage


ENVELOPE_DEBUG_TEXT_PREFIX = "CFTUV_EnvelopeDebug_"
ENVELOPE_DEBUG_PROFILE_TEXT_PREFIX = "CFTUV_EnvelopeProfile_"
ENVELOPE_DEBUG_LABEL_LAYER = "ENV_LABELS"

ENVELOPE_DEBUG_LAYER_STYLES = {
    "ENV_00_PATCH_DOMAIN": ((0.90, 0.90, 0.90, 1.0), 5),
    "ENV_01_HOLES": ((0.20, 0.45, 1.00, 1.0), 6),
    "ENV_02_BARRIERS": ((1.00, 0.15, 0.15, 1.0), 8),
    "ENV_10_PHYSICAL_CHAINS": ((1.00, 0.80, 0.05, 1.0), 7),
    "ENV_11_CHAIN_USES": ((1.00, 0.45, 0.05, 1.0), 4),
    "ENV_12_SELECTED_SOURCES": ((0.05, 1.00, 0.95, 1.0), 9),
    "ENV_13_PATCH_PAIRS": ((0.95, 0.55, 0.10, 1.0), 8),
    "ENV_14_SEAM_SELF_PAIRS": ((1.00, 0.15, 0.80, 1.0), 8),
    "ENV_20_SOURCE_SUPPORTS": ((0.05, 0.85, 0.95, 1.0), 5),
    "ENV_21_MOVING_FRONTS": ((0.20, 1.00, 0.45, 1.0), 5),
    "ENV_22_HIDDEN_SUPPORTS": ((0.85, 0.20, 1.00, 1.0), 7),
    "ENV_30_ENVELOPE_INSTANCES": ((0.10, 0.95, 0.70, 1.0), 4),
    "ENV_40_RAW_COVERAGE": ((0.15, 0.80, 1.00, 1.0), 8),
    "ENV_53_POINT_CONTACTS": ((1.00, 0.85, 0.05, 1.0), 11),
    "ENV_54_BOUNDARY_OCCURRENCES": ((0.95, 0.55, 1.00, 1.0), 6),
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
        "ENV_12_SELECTED_SOURCES",
        "ENV_13_PATCH_PAIRS",
        "ENV_14_SEAM_SELF_PAIRS",
    ),
    "envelope_debug_show_supports": (
        "ENV_20_SOURCE_SUPPORTS",
        "ENV_21_MOVING_FRONTS",
        "ENV_22_HIDDEN_SUPPORTS",
    ),
    "envelope_debug_show_envelopes": ("ENV_30_ENVELOPE_INSTANCES",),
    "envelope_debug_show_raw": (
        "ENV_40_RAW_COVERAGE",
        "ENV_53_POINT_CONTACTS",
        "ENV_54_BOUNDARY_OCCURRENCES",
    ),
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
    point_count: int


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


def envelope_debug_profile_text_name(source_obj_or_name) -> str:
    name = (
        source_obj_or_name
        if isinstance(source_obj_or_name, str)
        else source_obj_or_name.name
    )
    return ENVELOPE_DEBUG_PROFILE_TEXT_PREFIX + str(name) + ".json"


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
    profile_text = bpy.data.texts.get(
        envelope_debug_profile_text_name(source_obj_or_name)
    )
    if profile_text is not None:
        bpy.data.texts.remove(profile_text)


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


def _topology_lift(record, layer_ordinal: int) -> list[Vector]:
    normal = Vector(record.display_normal)
    if normal.length_squared > 0.0:
        normal.normalize()
    lift = 0.0015 + 0.00035 * float(layer_ordinal)
    return [
        Vector(point) + normal * lift
        for point in record.local_points
    ]


def _topology_cross(center: Vector, normal: Vector, size: float = 0.025):
    if normal.length_squared == 0.0:
        normal = Vector((0.0, 0.0, 1.0))
    else:
        normal = normal.normalized()
    helper = (
        Vector((1.0, 0.0, 0.0))
        if abs(normal.x) < 0.9
        else Vector((0.0, 1.0, 0.0))
    )
    axis_u = normal.cross(helper).normalized()
    axis_v = normal.cross(axis_u).normalized()
    return (
        (center - axis_u * size, center + axis_u * size),
        (center - axis_v * size, center + axis_v * size),
    )


def _topology_record_mapping(record, layer_name, stroke_index):
    return {
        "layer": layer_name,
        "stroke_index": stroke_index,
        "semantic_id": record.semantic_id,
        "stage": "TOPOLOGY",
        "kind": record.kind.value,
        "patch_domain_id": record.patch_domain_id,
        "physical_chain_id": record.physical_chain_id,
        "chain_use_id": record.chain_use_id,
        "boundary_loop_id": record.boundary_loop_id,
        "host_vertex_ids": list(record.host_vertex_ids),
        "host_edge_ids": list(record.host_edge_ids),
        "selected": record.selected,
        "directed": record.directed,
        "label": record.label,
    }


def _render_topology_scene(
    scene,
    writer,
    layer_ordinals,
    stroke_map,
) -> tuple[list[dict], int]:
    point_count = 0
    path_by_physical_chain = {}
    for record in scene.paths:
        layer_name = record.style_key
        points = _topology_lift(record, layer_ordinals[layer_name])
        stroke_index = writer.add_path(
            layer_name,
            points,
            line_width=ENVELOPE_DEBUG_LAYER_STYLES[layer_name][1],
            cyclic=record.closed,
        )
        point_count += len(points)
        if stroke_index is not None:
            stroke_map.append(
                _topology_record_mapping(
                    record,
                    layer_name,
                    stroke_index,
                )
            )
        if (
            record.kind.value == "PHYSICAL_CHAIN"
            and record.physical_chain_id is not None
        ):
            path_by_physical_chain[record.physical_chain_id] = (
                points,
                Vector(record.display_normal),
            )

        if record.directed and not record.closed and len(points) >= 2:
            tangent = points[-1] - points[-2]
            if tangent.length_squared > 0.0:
                tangent.normalize()
                normal = Vector(record.display_normal)
                if normal.length_squared == 0.0:
                    normal = Vector((0.0, 0.0, 1.0))
                side = normal.cross(tangent)
                if side.length_squared == 0.0:
                    side = Vector((0.0, 1.0, 0.0))
                else:
                    side.normalize()
                for endpoint in (
                    points[-1] - tangent * 0.035 + side * 0.018,
                    points[-1] - tangent * 0.035 - side * 0.018,
                ):
                    marker = (points[-1], endpoint)
                    marker_index = writer.add_path(
                        layer_name,
                        marker,
                        line_width=ENVELOPE_DEBUG_LAYER_STYLES[
                            layer_name
                        ][1],
                    )
                    point_count += 2
                    if marker_index is not None:
                        stroke_map.append(
                            _topology_record_mapping(
                                record,
                                layer_name,
                                marker_index,
                            )
                        )

    pair_payloads = []
    for pair in scene.pairs:
        pair_payload = {
            "pair_id": pair.pair_id,
            "kind": pair.kind.value,
            "physical_chain_id": pair.physical_chain_id,
            "chain_use_ids": list(pair.chain_use_ids),
            "patch_domain_ids": list(pair.patch_domain_ids),
            "host_edge_ids": list(pair.host_edge_ids),
        }
        pair_payloads.append(pair_payload)
        carrier = path_by_physical_chain.get(pair.physical_chain_id)
        if carrier is None:
            continue
        points, normal = carrier
        center = points[len(points) // 2]
        layer_name = (
            "ENV_13_PATCH_PAIRS"
            if pair.kind.value == "PATCH"
            else "ENV_14_SEAM_SELF_PAIRS"
        )
        for segment in _topology_cross(center, normal):
            stroke_index = writer.add_path(
                layer_name,
                segment,
                line_width=ENVELOPE_DEBUG_LAYER_STYLES[layer_name][1],
            )
            point_count += 2
            if stroke_index is not None:
                stroke_map.append(
                    {
                        **pair_payload,
                        "layer": layer_name,
                        "stroke_index": stroke_index,
                        "stage": "TOPOLOGY",
                    }
                )
    return pair_payloads, point_count


def _render_exact_scene(
    scene,
    writer,
    layer_ordinals,
    stroke_map,
    sympy,
) -> tuple[list[dict], dict[str, int], int]:
    frame_by_domain = {
        item.patch_domain_id: item for item in scene.patch_frames
    }
    point_count = 0

    def frame_for(record):
        return frame_by_domain[record.patch_domain_id]

    def add_path_record(record, exact_points, *, cyclic):
        nonlocal point_count
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
        point_count += len(points)
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
            point_count += 2
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
            point_count += 2
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
            point_count += 2
            stroke_map.append(
                {
                    **diagnostic_payload,
                    "layer": "ENV_80_DIAGNOSTICS",
                    "stroke_index": stroke_index,
                }
            )

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
    return diagnostics, stage_counts, point_count


# Столбцы масштабирования: по ним видно, какая величина растёт нелинейно у
# большого патча против мелкого. Порядок — как в конвейере: сколько сегментов
# домена, сколько принесли огибающие до и после клипа, сколько пар отсеял
# broadphase, сколько пересечений нашлось, во что это развернулось на выходе.
_SCALING_COUNTER_COLUMNS = (
    ("dom", "ARRANGEMENT_DOMAIN_SEGMENTS"),
    ("clip>", "CLIP_SEGMENTS_IN"),
    ("clip<", "CLIP_SEGMENTS_OUT"),
    ("in", "ARRANGEMENT_INPUT_SEGMENTS"),
    ("allpair", "ARRANGEMENT_ALL_POSSIBLE_PAIRS"),
    ("broad", "ARRANGEMENT_BROADPHASE_CANDIDATE_PAIRS"),
    ("hits", "ARRANGEMENT_INTERSECTIONS"),
    ("atomic", "ARRANGEMENT_ATOMIC_SEGMENTS"),
    ("faces", "ARRANGEMENT_OUTPUT_REGIONS"),
    ("scans", "ARRANGEMENT_POINT_LOCATION_SCANS"),
    ("chars", "ARRANGEMENT_MAX_COORDINATE_CHARS"),
    ("cands", "INTERACTION_CANDIDATES"),
    # Последним и намеренно: именно этот столбец объяснил полевое время, когда
    # все предыдущие его не объяснили.
    ("algebra", "ALGEBRAIC_CANONICALIZATIONS"),
)


def _counter_text(value) -> str:
    return str(int(value)) if float(value).is_integer() else f"{value:g}"


def _domain_text(patch_domain_id) -> str:
    # Хвост, а не начало: у всех доменов общий префикс `host-v0:patch-domain:`,
    # и обрезка слева давала одинаковые строки. Живой след стадий уже печатает
    # последние три символа, и владелец называет патчи именно так («bf6»).
    return (patch_domain_id or "-")[-24:]


def _print_counters(profile) -> None:
    """Счётчики по каждому домену: сводка масштабирования, затем всё остальное.

    Сводка отвечает на вопрос, ради которого счётчики и заведены. Полный список
    печатается следом и намеренно: счётчик, для которого не завели столбца, иначе
    выглядел бы как несделанное измерение, а не как невыведенное.
    """

    by_domain: dict[str, dict[str, float]] = {}
    for item in profile.counters:
        by_domain.setdefault(_domain_text(item.patch_domain_id), {})[
            item.name
        ] = item.value
    scaled = {
        domain: values
        for domain, values in by_domain.items()
        if any(name in values for _, name in _SCALING_COUNTER_COLUMNS)
    }
    if scaled:
        header = "".join(f"{title:>9}" for title, _ in _SCALING_COUNTER_COLUMNS)
        print(f"  Scaling  {'Domain':<24}{header}")
        for domain, values in sorted(scaled.items()):
            row = "".join(
                f"{_counter_text(values[name]) if name in values else '-':>9}"
                for _, name in _SCALING_COUNTER_COLUMNS
            )
            print(f"  {'':<9}{domain:<24}{row}")
    print("  Counter                                  Domain                   value")
    for domain, values in sorted(by_domain.items()):
        for name in sorted(values):
            print(
                f"  {name:<40} {domain:<24} "
                f"{_counter_text(values[name]):>8}"
            )


def _print_profile(profile) -> None:
    print(
        f"[CFTUV][EnvelopeProfile] {profile.source_name} "
        f"{profile.build_kind}"
    )
    print("  Stage                         Domain                     ms")
    for timing in profile.timings:
        domain = _domain_text(timing.patch_domain_id)
        print(
            f"  {timing.stage:<29} {domain:<24} "
            f"{timing.elapsed_seconds * 1000.0:9.3f}"
        )
    dominant = profile.dominant_stage
    if dominant is not None:
        print(
            f"  Dominant: {dominant[0]} "
            f"{dominant[1] * 1000.0:.3f} ms"
        )
    summary = profile.stage_summary()
    print(
        "  Receipts: "
        f"Topology {summary['topology']}/{summary['total']} | "
        f"Metric {summary['metric']}/{summary['total']} | "
        f"Raw {summary['raw']}/{summary['total']} | "
        f"Resolved {summary['resolved']}/{summary['total']}"
    )
    # «Resolved 6/7» само по себе не говорит ни какой домен упал, ни почему.
    # Именованный отказ, который надо искать в JSON, — это почти тихий отказ.
    for receipt in sorted(
        (
            item
            for item in profile.receipts
            if item.stage is not EnvelopeDomainStage.RESOLVED
        ),
        key=lambda item: item.patch_domain_id,
    ):
        print(
            f"  REJECTED {_domain_text(receipt.patch_domain_id)} "
            f"{receipt.stage.value}: {receipt.outcome}"
        )
        print(f"    {receipt.message}")
    cache_parts = []
    for layer in (
        "ANALYSIS_BUNDLE",
        "TOPOLOGY_EXPORT",
        "PATCH_METRIC",
        "DOMAIN_GEOMETRY",
    ):
        hits = sum(
            int(item.value)
            for item in profile.counters
            if item.name == f"{layer}_CACHE_HIT"
        )
        misses = sum(
            int(item.value)
            for item in profile.counters
            if item.name == f"{layer}_CACHE_MISS"
        )
        if hits or misses:
            cache_parts.append(f"{layer} H{hits}/M{misses}")
    if cache_parts:
        print("  Cache: " + " | ".join(cache_parts))
    _print_counters(profile)


def render_staged_envelope_debug(
    topology_scene,
    exact_scenes,
    source_obj,
    *,
    visibility_by_layer=None,
    profile=None,
) -> EnvelopeDebugRenderSummaryV1:
    """Render topology plus any admitted exact-domain scenes atomically."""

    exact_scenes = tuple(exact_scenes)
    cftuv_envelope = None
    sympy = None
    if exact_scenes:
        import cftuv_envelope as cftuv_envelope_module
        import sympy as sympy_module

        cftuv_envelope = cftuv_envelope_module
        sympy = sympy_module

    render_started = time.perf_counter()
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

    layer_ordinals = {
        name: ordinal
        for ordinal, name in enumerate(ENVELOPE_DEBUG_LAYER_STYLES)
    }
    stroke_map = []
    pair_payloads = []
    point_count = 0
    if topology_scene is not None:
        pair_payloads, topology_points = _render_topology_scene(
            topology_scene,
            writer,
            layer_ordinals,
            stroke_map,
        )
        point_count += topology_points

    diagnostics = []
    stage_counts = {}
    request_ids = []
    domain_ids = list(
        topology_scene.patch_domain_ids
        if topology_scene is not None
        else ()
    )
    raw_digests = []
    resolved_digests = []
    requested_alpha = ""
    source_revision = (
        topology_scene.source_revision
        if topology_scene is not None
        else ""
    )
    for scene in exact_scenes:
        scene_diagnostics, scene_counts, scene_points = _render_exact_scene(
            scene,
            writer,
            layer_ordinals,
            stroke_map,
            sympy,
        )
        diagnostics.extend(scene_diagnostics)
        point_count += scene_points
        for stage, count in scene_counts.items():
            stage_counts[stage] = stage_counts.get(stage, 0) + count
        request_ids.extend(
            _identity_value(item) for item in scene.request_ids
        )
        domain_ids.extend(
            _identity_value(item) for item in scene.patch_domain_ids
        )
        raw_digests.extend(scene.raw_coverage_digests)
        resolved_digests.extend(scene.resolved_coverage_digests)
        requested_alpha = str(scene.requested_alpha.value)
        source_revision = _identity_value(scene.source_revision)
    if topology_scene is not None:
        stage_counts["TOPOLOGY"] = len(topology_scene.paths)

    request_ids = sorted(set(request_ids))
    domain_ids = sorted(set(domain_ids))
    raw_digests = sorted(set(raw_digests))
    resolved_digests = sorted(set(resolved_digests))
    kernel_version = (
        cftuv_envelope.__version__
        if cftuv_envelope is not None
        else "NOT_LOADED_TOPOLOGY_ONLY"
    )
    profile_snapshot = profile.snapshot() if profile is not None else None
    receipt_payload = (
        profile_snapshot.to_payload()["receipts"]
        if profile_snapshot is not None
        else []
    )

    gp_obj = writer.object
    gp_obj["kernel_version"] = kernel_version
    gp_obj["source_revision"] = source_revision
    gp_obj["decal_request_ids"] = json.dumps(request_ids)
    gp_obj["patch_domain_ids"] = json.dumps(domain_ids)
    gp_obj["requested_alpha"] = requested_alpha
    gp_obj["raw_coverage_digests"] = json.dumps(raw_digests)
    gp_obj["resolved_coverage_digests"] = json.dumps(resolved_digests)
    gp_obj["stage_outcomes"] = json.dumps(stage_counts, sort_keys=True)
    gp_obj["stage_receipts"] = json.dumps(
        receipt_payload,
        sort_keys=True,
    )

    if profile is not None:
        profile.add_timing(
            "GP_RENDER",
            time.perf_counter() - render_started,
        )
        profile.set_counter("GP_STROKES", writer.stroke_count())
        profile.set_counter("GP_POINTS", point_count)

    sidecar = {
        "schema": "cftuv.envelope.staged_debug_runtime_sidecar.v1",
        "object_name": object_name,
        "kernel_version": kernel_version,
        "source_revision": source_revision,
        "decal_request_ids": request_ids,
        "patch_domain_ids": domain_ids,
        "requested_alpha": requested_alpha,
        "raw_coverage_digests": raw_digests,
        "resolved_coverage_digests": resolved_digests,
        "stage_outcomes": stage_counts,
        "stage_receipts": receipt_payload,
        "topology_pairs": pair_payloads,
        "topology_selection_diagnostics": (
            [
                {
                    "code": item.code,
                    "message": item.message,
                    "patch_domain_id": item.patch_domain_id,
                    "physical_chain_id": item.physical_chain_id,
                }
                for item in topology_scene.selection_diagnostics
            ]
            if topology_scene is not None
            else []
        ),
        "strokes": stroke_map,
        "diagnostics": diagnostics,
    }
    sidecar_started = time.perf_counter()
    _write_sidecar(text_name, sidecar)
    if profile is not None:
        profile.add_timing(
            "SIDECAR_JSON",
            time.perf_counter() - sidecar_started,
        )
        final_profile = profile.snapshot()
        _write_sidecar(
            envelope_debug_profile_text_name(source_obj),
            final_profile.to_payload(),
        )
        _print_profile(final_profile)
    return EnvelopeDebugRenderSummaryV1(
        object_name,
        text_name,
        len(ENVELOPE_DEBUG_LAYER_STYLES),
        writer.stroke_count(),
        point_count,
    )


def render_envelope_topology_debug_scene(
    scene,
    source_obj,
    *,
    visibility_by_layer=None,
    profile=None,
) -> EnvelopeDebugRenderSummaryV1:
    return render_staged_envelope_debug(
        scene,
        (),
        source_obj,
        visibility_by_layer=visibility_by_layer,
        profile=profile,
    )


def render_envelope_debug_scene(
    scene,
    source_obj,
    *,
    visibility_by_layer=None,
) -> EnvelopeDebugRenderSummaryV1:
    """Compatibility renderer for one immutable exact DebugScene."""

    return render_staged_envelope_debug(
        None,
        (scene,),
        source_obj,
        visibility_by_layer=visibility_by_layer,
    )


__all__ = (
    "ENVELOPE_DEBUG_LABEL_LAYER",
    "ENVELOPE_DEBUG_LAYER_STYLES",
    "EnvelopeDebugRenderSummaryV1",
    "apply_envelope_debug_visibility",
    "clear_envelope_debug",
    "envelope_debug_object_name",
    "envelope_debug_profile_text_name",
    "envelope_debug_text_name",
    "render_envelope_debug_scene",
    "render_envelope_topology_debug_scene",
    "render_staged_envelope_debug",
    "visibility_from_settings",
)
