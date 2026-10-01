"""Детерминированный дамп Grease Pencil объекта для доказательства равенства.

Работает на обеих версиях GP-API: GPENCIL (v2, 4.1 и legacy-коллекция 4.3) и
GREASEPENCIL v3 (4.3+). Дамп двух уровней:

* штрихи в порядке объекта (индекс материала, число точек, позиции с
  округлением 1e-6, radius/pressure/strength/opacity, флаги cyclic/line_width);
* для v3 ещё и ВСЕ атрибуты drawing целиком (имя, домен, тип, значения и
  побитовый отпечаток float32) — так виден и лишний, и недостающий атрибут,
  который по штрихам не заметен.

Слои, кадры и материалы — обобщённым обходом скалярных RNA-свойств, чтобы
дамп не зависел от того, какие именно поля в данной версии Blender есть.
"""

from __future__ import annotations

import array
import hashlib
import json
import re

import bpy


ROUND = 6
_MS = re.compile(r"\d+(?:\.\d+)? ms")
_SCALAR_TYPES = {"BOOLEAN", "INT", "FLOAT", "STRING", "ENUM"}
_SKIP_PROPS = {"rna_type", "name_full", "session_uid", "users", "is_evaluated"}

# тип атрибута -> (поле значения, компонент, код array)
_ATTRIBUTE_FIELDS = {
    "FLOAT": ("value", 1, "f"),
    "INT": ("value", 1, "i"),
    "INT8": ("value", 1, "i"),
    "BOOLEAN": ("value", 1, "i"),
    "FLOAT_VECTOR": ("vector", 3, "f"),
    "FLOAT2": ("vector", 2, "f"),
    "FLOAT_COLOR": ("color", 4, "f"),
    "BYTE_COLOR": ("color", 4, "f"),
    "QUATERNION": ("value", 4, "f"),
}


def _rounded(value):
    if isinstance(value, float):
        return round(value, ROUND) + 0.0
    if isinstance(value, (tuple, list)):
        return [_rounded(item) for item in value]
    if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
        return [_rounded(item) for item in value]
    return value


def _scalar_props(struct):
    """Скалярные RNA-свойства структуры: имя -> значение (округлённое)."""

    result = {}
    for prop in struct.bl_rna.properties:
        if prop.identifier in _SKIP_PROPS or prop.type not in _SCALAR_TYPES:
            continue
        try:
            value = getattr(struct, prop.identifier)
        except (AttributeError, RuntimeError, TypeError):
            continue
        if isinstance(value, set):
            value = sorted(value)
        result[prop.identifier] = _rounded(value)
    return result


def _bits(values, code):
    return hashlib.sha256(array.array(code, values).tobytes()).hexdigest()


def _attribute_values(attribute):
    field, components, code = _ATTRIBUTE_FIELDS[attribute.data_type]
    count = len(attribute.data)
    buffer = array.array("f" if code == "f" else "i", [0] * (count * components))
    attribute.data.foreach_get(field, buffer)
    return components, code, buffer


def _dump_v3_attributes(drawing):
    attributes = {}
    for attribute in sorted(drawing.attributes, key=lambda item: item.name):
        if attribute.data_type not in _ATTRIBUTE_FIELDS:
            attributes[attribute.name] = {
                "domain": attribute.domain,
                "data_type": attribute.data_type,
                "unsupported": True,
            }
            continue
        components, code, buffer = _attribute_values(attribute)
        attributes[attribute.name] = {
            "domain": attribute.domain,
            "data_type": attribute.data_type,
            "length": len(attribute.data),
            "bits_sha256": _bits(buffer, code),
            "values": _rounded(list(buffer)) if code == "f" else list(buffer),
        }
    return attributes


def _dump_v3_strokes(drawing):
    strokes = []
    for stroke in drawing.strokes:
        points = stroke.points
        strokes.append(
            {
                "material_index": stroke.material_index,
                "cyclic": bool(stroke.cyclic),
                "point_count": len(points),
                "positions": [_rounded(tuple(p.position)) for p in points],
                "radius": [_rounded(p.radius) for p in points],
                "opacity": [_rounded(p.opacity) for p in points],
            }
        )
    return strokes


def _dump_v2_strokes(frame):
    strokes = []
    for stroke in frame.strokes:
        points = stroke.points
        strokes.append(
            {
                "material_index": stroke.material_index,
                "line_width": stroke.line_width,
                "cyclic": bool(stroke.use_cyclic),
                "point_count": len(points),
                "positions": [_rounded(tuple(p.co)) for p in points],
                "pressure": [_rounded(p.pressure) for p in points],
                "strength": [_rounded(p.strength) for p in points],
            }
        )
    return strokes


def _dump_frame(frame):
    record = {"props": _scalar_props(frame)}
    drawing = getattr(frame, "drawing", None)
    if drawing is not None:
        record["api"] = "v3"
        record["attributes"] = _dump_v3_attributes(drawing)
        record["strokes"] = _dump_v3_strokes(drawing)
    else:
        record["api"] = "v2"
        record["strokes"] = _dump_v2_strokes(frame)
    return record


def _dump_layers(gp_data):
    layers = []
    for layer in gp_data.layers:
        layers.append(
            {
                "props": _scalar_props(layer),
                "frames": [_dump_frame(frame) for frame in layer.frames],
            }
        )
    return layers


def _dump_material(material):
    if material is None:
        return None
    return {
        "name": material.name,
        "grease_pencil": (
            _scalar_props(material.grease_pencil)
            if material.grease_pencil
            else None
        ),
    }


def _sidecar_strokes(source_name):
    """Перечень штрихов sidecar (layer/stroke_index/...) без секунд."""

    text = bpy.data.texts.get(f"CFTUV_EnvelopeDebug_{source_name}.json")
    if text is None:
        return None
    payload = json.loads(text.as_string())
    normalised = _MS.sub("<ms>", json.dumps(payload.get("strokes", []), sort_keys=True))
    return {
        "count": len(payload.get("strokes", [])),
        "sha256": hashlib.sha256(normalised.encode("utf-8")).hexdigest(),
    }


def dump_gp_object(obj, source_name=None):
    gp_data = obj.data
    dump = {
        "object": {
            "name": obj.name,
            "type": obj.type,
            "matrix_world": _rounded([list(row) for row in obj.matrix_world]),
            "hide_render": obj.hide_render,
            "show_in_front": obj.show_in_front,
            "custom_property_keys": sorted(obj.keys()),
        },
        "data": {"name": gp_data.name, "props": _scalar_props(gp_data)},
        "materials": [_dump_material(slot) for slot in gp_data.materials],
        "layers": _dump_layers(gp_data),
        "sidecar_strokes": (
            _sidecar_strokes(source_name) if source_name else None
        ),
    }
    return dump


def canonical_json(dump) -> str:
    return json.dumps(dump, sort_keys=True, ensure_ascii=False, separators=(",", ":"))


def summary(dump) -> dict:
    """Короткая сводка: слои, штрихи, точки и отпечаток канонического JSON."""

    layers = []
    strokes_total = 0
    points_total = 0
    for layer in dump["layers"]:
        strokes = sum(len(frame["strokes"]) for frame in layer["frames"])
        points = sum(
            stroke["point_count"]
            for frame in layer["frames"]
            for stroke in frame["strokes"]
        )
        strokes_total += strokes
        points_total += points
        layers.append(
            [
                layer["props"].get("name", layer["props"].get("info")),
                strokes,
                points,
            ]
        )
    return {
        "layers": layers,
        "strokes": strokes_total,
        "points": points_total,
        "sha256": hashlib.sha256(
            canonical_json(dump).encode("utf-8")
        ).hexdigest(),
    }


def write_dump(obj, path, source_name=None) -> dict:
    dump = dump_gp_object(obj, source_name)
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(canonical_json(dump))
    return summary(dump)
