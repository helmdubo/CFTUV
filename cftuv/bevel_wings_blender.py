"""Тонкий слой Blender крыльев UV декали Both (BEVEL_WINGS_V1): запись нажатия и UV меша декали. Решает `bevel_wings` (numpy, без Blender).

* Запись крыльев - свойство НОВОГО меша `cftuv_bevel_wings` (домены граней бевела), её кладёт кнопка Both; каждая запись UV этого меша (кнопка,
  подтверждение и кадры живой ширины) проходит через `map_mesh_wings` из `envelope_production_mesh._map_hotspot`: превью и точный кадр
  отображаются одной функцией от одних чисел (UV ядра, вершины углов, домен и владелец граней), как отображение Hotspot.
* Крылья строятся только под Straight strips и без Hotspot: Uniform trim строит `u` веера иначе (BEVEL_WINGS_OFF_UNDER_UNIFORM), а Hotspot
  отображает UV ядра в свою метку (BEVEL_WINGS_OFF_UNDER_HOTSPOT) - меш Both строится, UV остаются законом ядра (или метки), исход назван.
"""

from __future__ import annotations

import numpy as np

from .bevel_faces import BOTH
from .bevel_wings import MESH_PROPERTY, WingsRecordV1, wing_uvs, wings_from_json, wings_to_json
from .envelope_request_policy import ENVELOPE_UV_POLICY_DIRECT_STRIP

BEVEL_WINGS_OFF_UNDER_HOTSPOT = "BEVEL_WINGS_OFF_UNDER_HOTSPOT"
BEVEL_WINGS_OFF_UNDER_UNIFORM = "BEVEL_WINGS_OFF_UNDER_UNIFORM"
BEVEL_WINGS_RECORD_INVALID = "BEVEL_WINGS_RECORD_INVALID"
#: Числа последнего отображения по `session_uid` меша (строки консоли кнопки и тесты).
LAST_COUNTERS: dict = {}
LAST_COUNTERS_SIZE = 8


def inner_patches(face_build, bundle) -> tuple:
    """Патчи анализа нажатия, все грани которых выделены (полосы и узлы бевела)."""

    selected = face_build.plan.selected
    return tuple(sorted(int(patch) for patch, node in bundle.patch_graph.nodes.items() if set(int(face) for face in node.face_indices) <= selected))


def press_record(face_build, bundle, uv_policy_id: str, hotspot) -> tuple:
    """`(запись крыльев JSON | None, исход отказа крыльев | "")` нажатия по граням: запись есть только у Both под Straight strips без Hotspot."""

    if face_build is None or face_build.plan.side != BOTH:
        return None, ""
    if hotspot is not None:
        return None, BEVEL_WINGS_OFF_UNDER_HOTSPOT
    if uv_policy_id != ENVELOPE_UV_POLICY_DIRECT_STRIP:
        return None, BEVEL_WINGS_OFF_UNDER_UNIFORM
    return wings_to_json(WingsRecordV1(inner_patches(face_build, bundle))), ""


def _read(collection, name: str, dtype, size: int = 1):
    out = np.empty(len(collection) * size, dtype=dtype)
    collection.foreach_get(name, out)
    return out


def _uv_store(mesh, layer_name: str):
    attribute = mesh.attributes.get(layer_name)
    if attribute is not None and attribute.data_type == "FLOAT2" and attribute.domain == "CORNER":
        return attribute.data, "vector"
    return mesh.uv_layers[layer_name].data, "uv"


def map_mesh_wings(mesh, layer_name: str = "UVMap") -> dict:
    """UV ядра в слое `layer_name` меша -> UV крыльев по записи `cftuv_bevel_wings` меша. Числа отображения; они же в `LAST_COUNTERS`.

    Запись не читается - UV ядра остаются, исход `BEVEL_WINGS_RECORD_INVALID` назван строкой консоли.
    """

    try:
        record = wings_from_json(str(mesh[MESH_PROPERTY]))
    except (KeyError, ValueError, TypeError) as exc:
        counters = {BEVEL_WINGS_RECORD_INVALID: 1}
        print(f"[CFTUV][BevelWings] {BEVEL_WINGS_RECORD_INVALID}: mesh {mesh.name!r}: {exc}", flush=True)
        _remember(mesh, counters)
        return counters
    corner = mesh.attributes.get(".corner_vert")
    vertices = _read(corner.data, "value", np.int32) if corner is not None and corner.domain == "CORNER" else _read(mesh.loops, "vertex_index", np.int32)
    totals = _read(mesh.polygons, "loop_total", np.int32)
    group = np.zeros(len(totals), dtype=np.int64)
    for name, scale in (("cftuv_domain", 1 << 20), ("cftuv_owner", 1)):
        attribute = mesh.attributes.get(name)
        if attribute is not None and attribute.domain == "FACE" and len(attribute.data) == len(totals):
            group += _read(attribute.data, "value", np.int32).astype(np.int64) * scale
    store, field = _uv_store(mesh, layer_name)
    kernel = _read(store, field, np.float32, 2).reshape(-1, 2)
    mapped, counters = wing_uvs(kernel, vertices.astype(np.int64), np.repeat(group, totals), record.inner_domains)
    store.foreach_set(field, mapped.astype(np.float32).reshape(-1))
    _remember(mesh, counters)
    return counters


def _remember(mesh, counters) -> None:
    key = getattr(mesh, "session_uid", id(mesh))
    LAST_COUNTERS.pop(key, None)
    if len(LAST_COUNTERS) >= LAST_COUNTERS_SIZE:
        LAST_COUNTERS.pop(next(iter(LAST_COUNTERS)))
    LAST_COUNTERS[key] = counters


def counter_lines(object_name) -> list:
    """Строки консоли кнопки: числа последнего отображения крыльев меша объекта `object_name` (нет записи - пусто)."""

    import bpy

    decal = bpy.data.objects.get(object_name or "")
    mesh = None if decal is None else decal.data
    if mesh is None or mesh.get(MESH_PROPERTY) is None:
        return []
    counters = LAST_COUNTERS.get(mesh.session_uid, {})
    return ["[CFTUV][BevelWings] " + ", ".join(f"{name}={value}" for name, value in sorted(counters.items()))]


__all__ = (
    "BEVEL_WINGS_OFF_UNDER_HOTSPOT",
    "BEVEL_WINGS_OFF_UNDER_UNIFORM",
    "BEVEL_WINGS_RECORD_INVALID",
    "LAST_COUNTERS",
    "counter_lines",
    "inner_patches",
    "map_mesh_wings",
    "press_record",
)
