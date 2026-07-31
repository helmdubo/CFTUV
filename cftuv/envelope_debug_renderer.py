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
from .envelope_queue_export import (
    QUEUE_LAYER_STYLES,
    QUEUE_OWNER_LAYERS,
    QUEUE_OWNER_PALETTE_WRAPPED,
    QUEUE_SKELETON_LAYER,
    QUEUE_VISIBILITY_PROPERTY,
    QUEUE_WALL_LAYER,
    queue_scene_payload,
)


ENVELOPE_DEBUG_TEXT_PREFIX = "CFTUV_EnvelopeDebug_"
ENVELOPE_DEBUG_PROFILE_TEXT_PREFIX = "CFTUV_EnvelopeProfile_"
ENVELOPE_DEBUG_LABEL_LAYER = "ENV_LABELS"

# Слой отказавших доменов. Отказ домена был виден ТОЛЬКО строкой статуса, и
# владелец читал «Resolved 1/2» как «строит с ошибкой на одну сторону»: во
# вьюпорте отказавшая сторона выглядела ровно как несуществующая. Здесь она
# получает контур.
ENVELOPE_DEBUG_REFUSED_LAYER = "ENV_93_QUEUE_REFUSED"

#: Стадия штрихов отказа в sidecar. Отдельная от `TOPOLOGY` и `QUEUE`: это не
#: результат движка, а объявление того, что результата у домена нет.
ENVELOPE_DEBUG_REFUSED_STAGE = "REFUSED"

#: Построение, у которого стадия `TOPOLOGY_READY` — это УСПЕХ, а не отказ.
#: Без этого имени чисто топологический прогон красил бы красным все домены.
ENVELOPE_DEBUG_TOPOLOGY_BUILD_KIND = "TOPOLOGY"

ENVELOPE_DEBUG_RESOLVED_STAGES = frozenset(
    {
        EnvelopeDomainStage.RESOLVED,
        EnvelopeDomainStage.QUEUE_RESOLVED,
    }
)

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
    # Слои движка QUEUE дописаны В КОНЕЦ намеренно: порядковый номер слоя
    # задаёт высоту подъёма над поверхностью, и вставка в середину сдвинула бы
    # подъём всех последующих слоёв эталонного пути. На движке LEGACY эти слои
    # не создаются вовсе (см. `_active_layer_styles`).
    **QUEUE_LAYER_STYLES,
    # Тот же красный, что у барьеров: смысл здесь тот же — «сюда нельзя», — и
    # новый цвет в палитре означал бы новый смысл. Слой создаётся ОБОИМИ
    # движками: METRIC_REJECTED случается и на LEGACY.
    ENVELOPE_DEBUG_REFUSED_LAYER: ((1.00, 0.15, 0.15, 1.0), 9),
}

QUEUE_LAYER_NAMES = frozenset(QUEUE_LAYER_STYLES)

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
    "envelope_debug_show_refused": (ENVELOPE_DEBUG_REFUSED_LAYER,),
    "envelope_debug_show_labels": (ENVELOPE_DEBUG_LABEL_LAYER,),
    QUEUE_VISIBILITY_PROPERTY: (
        QUEUE_SKELETON_LAYER,
        QUEUE_WALL_LAYER,
        *QUEUE_OWNER_LAYERS,
    ),
}


def _active_layer_styles(queue_scene):
    """Слои этого прогона. Слои очереди создаются только при её результате.

    Иначе движок LEGACY получал бы в GP-объекте десять пустых слоёв очереди, и
    «ничего не изменилось» пришлось бы доказывать чтением, а не сравнением
    набора слоёв.
    """

    if queue_scene is not None and queue_scene.domains:
        return ENVELOPE_DEBUG_LAYER_STYLES
    return {
        name: style
        for name, style in ENVELOPE_DEBUG_LAYER_STYLES.items()
        if name not in QUEUE_LAYER_NAMES
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


def _lift_plane_point(x: float, y: float, frame, layer_ordinal) -> Vector:
    """Точка карты патча в 3D хоста. Один кадр на все слои одного домена."""

    origin = _vector3(frame.origin)
    axis_u = _vector3(frame.axis_u)
    axis_v = _vector3(frame.axis_v)
    normal = _vector3(frame.normal)
    lift = 0.002 + 0.00035 * float(layer_ordinal)
    return origin + axis_u * x + axis_v * y + normal * lift


def _lift_point(point, frame, layer_ordinal, sympy) -> Vector:
    return _lift_plane_point(
        float(sympy.sympify(point.x_expression)),
        float(sympy.sympify(point.y_expression)),
        frame,
        layer_ordinal,
    )


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


def _pair_marker_point(points, *, closed: bool) -> Vector:
    """Куда ставится крестик пары: середина СРЕДИННОГО ПО ДЛИНЕ сегмента цепи.

    Стояло `points[len(points) // 2]` — срединная ВЕРШИНА полилинии. На цепи из
    двух рёбер это внутренняя вершина, на цепи из одного — вообще её конец, и
    владелец читал pair-запись как junction-пометку: крестик садился ровно туда,
    где смысл имеют отношения углов, а не цепей. Место маркера при этом
    определялось СЧЁТОМ вершин, то есть тем, где цепь оказалась разбита, — а не
    её геометрией.

    Место выбирается длиной: берётся сегмент, внутри которого лежит середина
    дуги, и маркер ставится в СЕРЕДИНУ ЭТОГО СЕГМЕНТА.

    Почему не сама середина дуги, хотя интерполяция внутри сегмента дала бы
    именно её. У цепи из двух РАВНЫХ рёбер — а это обычный случай, ребро шва,
    разбитое вершиной пополам, — середина дуги совпадает с той самой внутренней
    вершиной, и дефект не был бы вылечен вовсе. Отличать этот случай пришлось бы
    сравнением накопленных длин с нулём, то есть порогом на float'ах. Середина
    сегмента же не совпадает с вершиной НИ ПРИ КАКОЙ длине — по построению, а не
    по удаче, и никакого сравнения для этого не нужно.

    Замкнутая цепь считается вместе со своим замыкающим сегментом: длина её
    дуги — периметр, а не путь до последней вершины.

    Семантика самих pair-записей не двигается: меняется только точка, в которой
    рисуется их крестик.
    """

    spans = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        spans.append((points[-1], points[0]))
    measured = [
        (start, end, (end - start).length)
        for start, end in spans
        if (end - start).length_squared > 0.0
    ]
    if not measured:
        return Vector(points[0]) if points else Vector((0.0, 0.0, 0.0))
    target = sum(length for _, _, length in measured) / 2.0
    walked = 0.0
    chosen = measured[-1]
    for span in measured:
        walked += span[2]
        if walked > target:
            chosen = span
            break
    return (chosen[0] + chosen[1]) * 0.5


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
                record.closed,
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
        points, normal, closed = carrier
        center = _pair_marker_point(points, closed=closed)
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


def refused_receipts(profile_snapshot) -> tuple:
    """Домены, не дошедшие до покрытия, в порядке их идентификаторов.

    На чисто топологическом построении таких НЕТ: там `TOPOLOGY_READY` — это
    терминальный успех, и красить его отказом значило бы объявлять отказавшими
    все домены каждого топологического прогона.
    """

    if profile_snapshot is None:
        return ()
    if profile_snapshot.build_kind == ENVELOPE_DEBUG_TOPOLOGY_BUILD_KIND:
        return ()
    return tuple(
        sorted(
            (
                item
                for item in profile_snapshot.receipts
                if item.stage not in ENVELOPE_DEBUG_RESOLVED_STAGES
            ),
            key=lambda item: item.patch_domain_id,
        )
    )


def render_refused_domains(
    topology_scene,
    receipts,
    writer,
    layer_ordinals,
    stroke_map,
) -> int:
    """Контур каждого отказавшего домена красным, с именем исхода в штрихе.

    Петли домена уже посчитаны сценой топологии (`PATCH_OUTER_LOOP` и
    `PATCH_HOLE_LOOP`), поэтому здесь не появляется второго источника геометрии
    домена: тот же путь рисуется вторым слоем.
    """

    if topology_scene is None or not receipts:
        return 0
    receipt_by_domain = {item.patch_domain_id: item for item in receipts}
    ordinal = layer_ordinals[ENVELOPE_DEBUG_REFUSED_LAYER]
    line_width = ENVELOPE_DEBUG_LAYER_STYLES[ENVELOPE_DEBUG_REFUSED_LAYER][1]
    point_count = 0
    for record in topology_scene.paths:
        if record.kind.value not in {"PATCH_OUTER_LOOP", "PATCH_HOLE_LOOP"}:
            continue
        receipt = receipt_by_domain.get(record.patch_domain_id)
        if receipt is None:
            continue
        points = _topology_lift(record, ordinal)
        stroke_index = writer.add_path(
            ENVELOPE_DEBUG_REFUSED_LAYER,
            points,
            line_width=line_width,
            cyclic=record.closed,
        )
        point_count += len(points)
        if stroke_index is None:
            continue
        stroke_map.append(
            {
                "layer": ENVELOPE_DEBUG_REFUSED_LAYER,
                "stroke_index": stroke_index,
                "stage": ENVELOPE_DEBUG_REFUSED_STAGE,
                "kind": record.kind.value,
                "patch_domain_id": record.patch_domain_id,
                "boundary_loop_id": record.boundary_loop_id,
                "domain_stage": receipt.stage.value,
                "outcome": receipt.outcome,
                "message": receipt.message,
                "label": (
                    f"{record.label} REFUSED "
                    f"{receipt.stage.value}: {receipt.outcome}"
                ),
            }
        )
    return point_count


def queue_frames_by_domain(exact_scenes) -> dict:
    """Кадр патча по строковому идентификатору домена, для слоёв очереди."""

    return {
        _identity_value(frame.patch_domain_id): frame
        for scene in exact_scenes
        for frame in scene.patch_frames
    }


def _queue_stroke_mapping(domain, layer_name, stroke_index, kind, payload):
    return {
        "layer": layer_name,
        "stroke_index": stroke_index,
        "stage": "QUEUE",
        "kind": kind,
        "patch_domain_id": domain.patch_domain_id,
        **payload,
    }


def render_queue_scene(
    queue_scene,
    frame_by_domain,
    writer,
    layer_ordinals,
    stroke_map,
) -> int:
    """Слои очереди: скелет, стены и грани покрытия по цвету владельца.

    Домен без кадра патча пропускается НЕ молча: он остаётся в sidecar своим
    исходом, а здесь для него просто нет системы координат, в которой его
    можно было бы поднять в 3D.
    """

    point_count = 0
    for domain in queue_scene.domains:
        frame = frame_by_domain.get(domain.patch_domain_id)
        if frame is None:
            continue
        for segment in domain.segments:
            ordinal = layer_ordinals[segment.layer]
            points = [
                _lift_plane_point(x, y, frame, ordinal)
                for x, y in segment.points
            ]
            stroke_index = writer.add_path(
                segment.layer,
                points,
                line_width=ENVELOPE_DEBUG_LAYER_STYLES[segment.layer][1],
            )
            point_count += len(points)
            if stroke_index is not None:
                stroke_map.append(
                    _queue_stroke_mapping(
                        domain,
                        segment.layer,
                        stroke_index,
                        segment.label,
                        {"region_id": segment.region_id},
                    )
                )
        for face in domain.faces:
            layer_name = queue_scene.layer_of(face.envelope_spec_id)
            ordinal = layer_ordinals[layer_name]
            points = [
                _lift_plane_point(x, y, frame, ordinal)
                for x, y in face.points
            ]
            stroke_index = writer.add_path(
                layer_name,
                points,
                line_width=ENVELOPE_DEBUG_LAYER_STYLES[layer_name][1],
                cyclic=True,
            )
            point_count += len(points)
            if stroke_index is not None:
                stroke_map.append(
                    _queue_stroke_mapping(
                        domain,
                        layer_name,
                        stroke_index,
                        "QUEUE_COVERAGE_FACE",
                        {
                            "region_id": face.region_id,
                            "owner": list(face.owner),
                            "envelope_spec_id": face.envelope_spec_id,
                            "envelope_instance_id": (
                                face.envelope_instance_id
                            ),
                            "palette_slot": queue_scene.slot_of(
                                face.envelope_spec_id
                            ),
                            "doubled_area": face.doubled_area_text,
                            # Цепь владельца и слитые владельцы — здесь, а не
                            # только в счётчике: число отвечает «сколько», а
                            # найти подавленный разделитель на меше можно лишь
                            # по именам тех граней, которые в штрих вошли.
                            "source_chain_id": face.source_chain_id,
                            "merged_owners": [
                                list(item) for item in face.merged_owners
                            ],
                        },
                    )
                )
    return point_count


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


def _print_diagnostics(diagnostics) -> None:
    """Все диагностики ядра, не только отказы.

    Печаталось только `REJECTED`, поэтому именованное событие в поле было
    невидимо: в прошлом срезе нельзя было сказать, формировался ли
    `MULTIWAY_MEET_RESOLVED_AS_ONE_EVENT` вообще. Тот же класс дефекта, который
    уже чинили печатью отказов, — исход, который надо искать в JSON, почти
    неотличим от несостоявшегося.
    """

    rows = [item for item in diagnostics if _diagnostic_severity(item) != "EXACT"]
    if not rows:
        return
    print(f"  Diagnostics ({len(rows)}):")
    for item in rows:
        outcome = getattr(item, "outcome", "")
        outcome = getattr(outcome, "value", outcome)
        domain = _domain_text(getattr(item, "patch_domain_id", None))
        print(f"    {_diagnostic_severity(item):<12} {domain:<24} {outcome}")
        message = getattr(item, "message", "")
        if message:
            print(f"      {message}")


def _diagnostic_severity(item) -> str:
    severity = getattr(item, "severity", "")
    return str(getattr(severity, "value", severity) or "")


def _write_gp_properties(gp_obj, sidecar) -> None:
    """Свойства GP-объекта — из того же payload, что и sidecar.

    Второй источник тех же полей рано или поздно разошёлся бы с первым, и
    расхождение читалось бы как расхождение движка с самим собой.
    """

    gp_obj["kernel_version"] = sidecar["kernel_version"]
    gp_obj["source_revision"] = sidecar["source_revision"]
    gp_obj["requested_alpha"] = sidecar["requested_alpha"]
    for key in (
        "decal_request_ids",
        "patch_domain_ids",
        "raw_coverage_digests",
        "resolved_coverage_digests",
        "stage_outcomes",
        "stage_receipts",
    ):
        gp_obj[key] = json.dumps(sidecar[key], sort_keys=True)


def _staged_sidecar(
    object_name,
    kernel_version,
    source_revision,
    identities,
    stage_counts,
    receipt_payload,
    pair_payloads,
    topology_scene,
    stroke_map,
    diagnostics,
    queue_scene,
) -> dict:
    """Sidecar постадийного прогона. Раздел очереди появляется вместе с ней."""

    request_ids, domain_ids, raw_digests, resolved_digests, alpha = identities
    payload = {
        "schema": "cftuv.envelope.staged_debug_runtime_sidecar.v1",
        "object_name": object_name,
        "kernel_version": kernel_version,
        "source_revision": source_revision,
        "decal_request_ids": request_ids,
        "patch_domain_ids": domain_ids,
        "requested_alpha": alpha,
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
    if queue_scene is not None and queue_scene.domains:
        payload["queue"] = queue_scene_payload(queue_scene)
    return payload


def _accumulate_exact_scenes(
    topology_scene,
    exact_scenes,
    writer,
    layer_ordinals,
    stroke_map,
    sympy,
) -> tuple[list, dict, tuple, str, int]:
    """Отрисовать точные сцены доменов и собрать их сводные идентичности.

    Идентичности возвращаются уже отсортированным множеством: собирать их в
    вызывающем значило бы держать второй экземпляр той же сводки.
    """

    diagnostics = []
    stage_counts: dict[str, int] = {}
    request_ids: list[str] = []
    domain_ids = list(
        topology_scene.patch_domain_ids if topology_scene is not None else ()
    )
    raw_digests: list[str] = []
    resolved_digests: list[str] = []
    requested_alpha = ""
    source_revision = (
        topology_scene.source_revision if topology_scene is not None else ""
    )
    point_count = 0
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
    identities = (
        sorted(set(request_ids)),
        sorted(set(domain_ids)),
        sorted(set(raw_digests)),
        sorted(set(resolved_digests)),
        requested_alpha,
    )
    return diagnostics, stage_counts, identities, source_revision, point_count


def render_staged_envelope_debug(
    topology_scene,
    exact_scenes,
    source_obj,
    *,
    visibility_by_layer=None,
    profile=None,
    queue_scene=None,
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
    active_styles = _active_layer_styles(queue_scene)
    visibility = {name: True for name in active_styles}
    visibility.update(visibility_by_layer or {})
    for layer_name, (color, _line_width) in active_styles.items():
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

    (
        diagnostics,
        stage_counts,
        identities,
        source_revision,
        exact_points,
    ) = _accumulate_exact_scenes(
        topology_scene,
        exact_scenes,
        writer,
        layer_ordinals,
        stroke_map,
        sympy,
    )
    point_count += exact_points
    if topology_scene is not None:
        stage_counts["TOPOLOGY"] = len(topology_scene.paths)

    # Отказ домена рисуется ДО сборки sidecar: его штрихи обязаны попасть в тот
    # же перечень, что и всё остальное, иначе имя исхода снова окажется только
    # в строке статуса.
    profile_snapshot = profile.snapshot() if profile is not None else None
    refused = refused_receipts(profile_snapshot)
    point_count += render_refused_domains(
        topology_scene,
        refused,
        writer,
        layer_ordinals,
        stroke_map,
    )
    stage_counts[ENVELOPE_DEBUG_REFUSED_STAGE] = len(refused)
    if queue_scene is not None and queue_scene.domains:
        point_count += render_queue_scene(
            queue_scene,
            queue_frames_by_domain(exact_scenes),
            writer,
            layer_ordinals,
            stroke_map,
        )
        stage_counts["QUEUE"] = sum(
            len(item.faces) for item in queue_scene.domains
        )

    kernel_version = (
        cftuv_envelope.__version__
        if cftuv_envelope is not None
        else "NOT_LOADED_TOPOLOGY_ONLY"
    )
    receipt_payload = (
        profile_snapshot.to_payload()["receipts"]
        if profile_snapshot is not None
        else []
    )

    sidecar = _staged_sidecar(
        object_name,
        kernel_version,
        source_revision,
        identities,
        stage_counts,
        receipt_payload,
        pair_payloads,
        topology_scene,
        stroke_map,
        diagnostics,
        queue_scene,
    )
    _write_gp_properties(writer.object, sidecar)

    if profile is not None:
        profile.add_timing(
            "GP_RENDER",
            time.perf_counter() - render_started,
        )
        profile.set_counter("GP_STROKES", writer.stroke_count())
        profile.set_counter("GP_POINTS", point_count)
        if queue_scene is not None:
            profile.set_counter(
                QUEUE_OWNER_PALETTE_WRAPPED,
                queue_scene.palette_wrapped,
            )

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
        _print_diagnostics(diagnostics)
    return EnvelopeDebugRenderSummaryV1(
        object_name,
        text_name,
        len(active_styles),
        writer.stroke_count(),
        point_count,
    )


def redraw_envelope_queue_layers(
    source_obj_or_name,
    exact_scenes,
    queue_scene,
    *,
    visibility_by_layer=None,
    profile=None,
) -> EnvelopeDebugRenderSummaryV1 | None:
    """Лёгкая перерисовка ТОЛЬКО слоёв очереди на готовом GP-объекте.

    Полный проход удаляет объект и строит его заново; при перетаскивании
    ползунка alpha это и лишняя работа, и удаление объекта прямо внутри
    update-callback свойства. Здесь объект тот же, а переписываются ровно
    десять слоёв очереди.

    `None` — GP-объекта нет (кнопку ещё не нажимали либо результат очищен).
    Это ответ вызывающему, а не тихое бездействие.
    """

    render_started = time.perf_counter()
    object_name = envelope_debug_object_name(source_obj_or_name)
    writer = GreasePencilDebugWriter.attach(object_name)
    if writer is None:
        return None
    visibility = {name: True for name in QUEUE_LAYER_NAMES}
    visibility.update(
        {
            name: value
            for name, value in (visibility_by_layer or {}).items()
            if name in QUEUE_LAYER_NAMES
        }
    )
    for layer_name in QUEUE_LAYER_STYLES:
        writer.ensure_layer(
            layer_name,
            QUEUE_LAYER_STYLES[layer_name][0],
            visible=visibility.get(layer_name, True),
        )
    layer_ordinals = {
        name: ordinal
        for ordinal, name in enumerate(ENVELOPE_DEBUG_LAYER_STYLES)
    }
    stroke_map: list[dict] = []
    point_count = render_queue_scene(
        queue_scene,
        queue_frames_by_domain(tuple(exact_scenes)),
        writer,
        layer_ordinals,
        stroke_map,
    )
    text_name = envelope_debug_text_name(source_obj_or_name)
    text = bpy.data.texts.get(text_name)
    if text is not None:
        payload = json.loads(text.as_string())
        payload["strokes"] = [
            item
            for item in payload.get("strokes", ())
            if item.get("stage") != "QUEUE"
        ] + stroke_map
        payload["queue"] = queue_scene_payload(queue_scene)
        _write_sidecar(text_name, payload)
    writer.object["requested_alpha"] = (
        queue_scene.domains[0].alpha if queue_scene.domains else ""
    )
    if profile is not None:
        profile.add_timing(
            "QUEUE_REDRAW",
            time.perf_counter() - render_started,
        )
        profile.set_counter(
            QUEUE_OWNER_PALETTE_WRAPPED,
            queue_scene.palette_wrapped,
        )
    return EnvelopeDebugRenderSummaryV1(
        object_name,
        text_name,
        len(QUEUE_LAYER_STYLES),
        writer.stroke_count(),
        point_count,
    )


def update_queue_alpha(
    controller,
    source_name,
    alpha,
    *,
    density,
    settings=None,
    profile=None,
) -> str | None:
    """Пересчёт покрытия очереди на ТЁПЛОЙ подготовке плюс перерисовка слоёв.

    `None` — тёплого кэша нет (кнопку ещё не нажимали, движок другой либо
    источник сменился). Тогда ползунок не считает ВООБЩЕ: подготовка стоит
    десятки миллисекунд на домен, и запускать её на каждое движение мыши
    значило бы подвесить интерфейс.
    """

    from .envelope_queue_export import (
        queue_timing_text,
        recompute_queue_coverage,
    )
    from .envelope_request_policy import normalize_envelope_fan_density

    session = getattr(controller, "queue_session", None)
    if (
        session is None
        or str(session.source_object_name) != str(source_name)
        or not session.entries
    ):
        return None
    if session.density != normalize_envelope_fan_density(density):
        return "Fan Density changed; press Build"
    started = time.perf_counter()
    scene = recompute_queue_coverage(
        session.entries,
        str(float(alpha)),
        profile=profile,
    )
    summary = redraw_envelope_queue_layers(
        source_name,
        session.exact_scenes,
        scene,
        visibility_by_layer=(
            visibility_from_settings(settings)
            if settings is not None
            else None
        ),
        profile=profile,
    )
    if summary is None:
        return None
    elapsed = (time.perf_counter() - started) * 1000.0
    return f"{queue_timing_text(scene)} | alpha redraw {elapsed:.0f} ms"


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
    "ENVELOPE_DEBUG_REFUSED_LAYER",
    "ENVELOPE_DEBUG_REFUSED_STAGE",
    "QUEUE_LAYER_NAMES",
    "EnvelopeDebugRenderSummaryV1",
    "apply_envelope_debug_visibility",
    "clear_envelope_debug",
    "envelope_debug_object_name",
    "envelope_debug_profile_text_name",
    "envelope_debug_text_name",
    "queue_frames_by_domain",
    "redraw_envelope_queue_layers",
    "refused_receipts",
    "render_envelope_debug_scene",
    "render_envelope_topology_debug_scene",
    "render_queue_scene",
    "render_refused_domains",
    "render_staged_envelope_debug",
    "update_queue_alpha",
    "visibility_from_settings",
)
