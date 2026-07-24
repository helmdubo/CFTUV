"""JSON Schema generation directly from public contract types."""

from __future__ import annotations

import types
from dataclasses import fields, is_dataclass
from decimal import Decimal
from enum import Enum
from typing import Any, Union, get_args, get_origin, get_type_hints

from .ids import OpaqueId


class ContractSchemaError(TypeError):
    pass


def json_schema_for(root_type: type[Any], schema_id: str) -> dict[str, Any]:
    definitions: dict[str, Any] = {}
    building: set[str] = set()

    def schema_for(annotation: Any) -> dict[str, Any]:
        origin = get_origin(annotation)
        if origin in (Union, types.UnionType):
            return {"oneOf": [schema_for(choice) for choice in get_args(annotation)]}
        if origin is tuple:
            args = get_args(annotation)
            if len(args) == 2 and args[1] is Ellipsis:
                return {"type": "array", "items": schema_for(args[0])}
            return {
                "type": "array",
                "prefixItems": [schema_for(item) for item in args],
                "minItems": len(args),
                "maxItems": len(args),
            }
        if origin is frozenset:
            (item_type,) = get_args(annotation)
            return {"type": "array", "items": schema_for(item_type), "uniqueItems": True}
        if annotation is type(None):
            return {"type": "null"}
        if annotation is Decimal:
            return {
                "type": "string",
                "pattern": "^-?(?:0|[1-9][0-9]*)(?:\\.[0-9]+)?$",
            }
        if annotation is str:
            return {"type": "string"}
        if annotation is bool:
            return {"type": "boolean"}
        if annotation is int:
            return {"type": "integer"}
        if annotation is float:
            return {"type": "number"}
        if isinstance(annotation, type) and issubclass(annotation, Enum):
            return {"type": "string", "enum": [item.value for item in annotation]}
        if isinstance(annotation, type) and issubclass(annotation, OpaqueId):
            ensure_id(annotation)
            return {"$ref": f"#/$defs/{annotation.__name__}"}
        if isinstance(annotation, type) and is_dataclass(annotation):
            ensure_record(annotation)
            return {"$ref": f"#/$defs/{annotation.__name__}"}
        raise ContractSchemaError(f"unsupported public annotation: {annotation!r}")

    def ensure_id(identity_type: type[OpaqueId]) -> None:
        name = identity_type.__name__
        if name in definitions:
            return
        definitions[name] = {
            "type": "object",
            "properties": {
                "$id_type": {"const": name},
                "value": {"type": "string", "minLength": 1},
            },
            "required": ["$id_type", "value"],
            "additionalProperties": False,
        }

    def ensure_record(record_type: type[Any]) -> None:
        name = record_type.__name__
        if name in definitions or name in building:
            return
        building.add(name)
        hints = get_type_hints(record_type)
        properties: dict[str, Any] = {"$type": {"const": name}}
        required = ["$type"]
        for field in fields(record_type):
            properties[field.name] = schema_for(hints[field.name])
            required.append(field.name)
        definitions[name] = {
            "type": "object",
            "properties": properties,
            "required": required,
            "additionalProperties": False,
        }
        building.remove(name)

    root_schema = schema_for(root_type)
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": schema_id,
        "title": root_type.__name__,
        **root_schema,
        "$defs": {name: definitions[name] for name in sorted(definitions)},
    }

