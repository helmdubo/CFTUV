"""Версионированные deterministic JSON codecs для публичных contracts."""

from __future__ import annotations

import json
import types
from dataclasses import fields, is_dataclass
from decimal import Decimal
from enum import Enum
from typing import Any, Generic, TypeVar, Union, get_args, get_origin, get_type_hints

from . import ids, numeric
from .contracts import analysis, coverage, envelopes, events, geometry_batch
from .contracts import (
    debug,
    metric,
    ownership,
    plan,
    request,
    seeds,
    surface,
    tessellation,
)
from .ids import OpaqueId


T = TypeVar("T")


class ContractCodecError(ValueError):
    pass


_PUBLIC_MODULES = (
    ids,
    numeric,
    analysis,
    coverage,
    envelopes,
    events,
    geometry_batch,
    metric,
    ownership,
    plan,
    request,
    seeds,
    surface,
    tessellation,
    debug,
)


def _build_type_registry() -> dict[str, type[Any]]:
    registry: dict[str, type[Any]] = {}
    for module in _PUBLIC_MODULES:
        for value in vars(module).values():
            if isinstance(value, type) and is_dataclass(value):
                existing = registry.get(value.__name__)
                if existing is not None and existing is not value:
                    raise RuntimeError(f"duplicate public record name: {value.__name__}")
                registry[value.__name__] = value
    return registry


_TYPE_REGISTRY = _build_type_registry()


def _canonical_decimal(value: Decimal) -> str:
    if not value.is_finite():
        raise ContractCodecError("non-finite Decimal is forbidden")
    if value.is_zero():
        return "0"
    normalized = value.normalize()
    text = format(normalized, "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text


def _json_sort_key(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def to_canonical_data(value: Any) -> Any:
    if isinstance(value, OpaqueId):
        return {"$id_type": type(value).__name__, "value": value.value}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Decimal):
        return _canonical_decimal(value)
    if is_dataclass(value) and not isinstance(value, type):
        result: dict[str, Any] = {"$type": type(value).__name__}
        for field in fields(value):
            result[field.name] = to_canonical_data(getattr(value, field.name))
        return result
    if isinstance(value, tuple):
        return [to_canonical_data(item) for item in value]
    if isinstance(value, frozenset):
        encoded = [to_canonical_data(item) for item in value]
        return sorted(encoded, key=_json_sort_key)
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise ContractCodecError("canonical mappings require string keys")
        return {key: to_canonical_data(item) for key, item in value.items()}
    if isinstance(value, float):
        if value != value or value in (float("inf"), float("-inf")):
            raise ContractCodecError("NaN and infinity are forbidden")
        return value
    if value is None or isinstance(value, (str, int, bool)):
        return value
    raise ContractCodecError(f"unsupported canonical value: {type(value).__name__}")


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        to_canonical_data(value),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _reject_constant(value: str) -> None:
    raise ContractCodecError(f"non-finite JSON number is forbidden: {value}")


def _decode_union(data: Any, annotation: Any) -> Any:
    choices = get_args(annotation)
    if data is None and type(None) in choices:
        return None
    if isinstance(data, dict):
        tagged_name = data.get("$type") or data.get("$id_type")
        if isinstance(tagged_name, str):
            tagged_type = _TYPE_REGISTRY.get(tagged_name)
            if tagged_type is None:
                raise ContractCodecError(f"unknown tagged type: {tagged_name}")
            for choice in choices:
                if isinstance(choice, type) and issubclass(tagged_type, choice):
                    return _decode_as(data, choice)
    errors: list[str] = []
    for choice in choices:
        if choice is type(None):
            continue
        try:
            return _decode_as(data, choice)
        except (ContractCodecError, TypeError, ValueError) as exc:
            errors.append(str(exc))
    raise ContractCodecError("value does not match union: " + "; ".join(errors))


def _decode_as(data: Any, annotation: Any) -> Any:
    origin = get_origin(annotation)
    if origin in (Union, types.UnionType):
        return _decode_union(data, annotation)
    if origin is tuple:
        if not isinstance(data, list):
            raise ContractCodecError("tuple field must be a JSON array")
        args = get_args(annotation)
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_decode_as(item, args[0]) for item in data)
        if len(args) != len(data):
            raise ContractCodecError("fixed tuple length mismatch")
        return tuple(_decode_as(item, item_type) for item, item_type in zip(data, args))
    if origin is frozenset:
        if not isinstance(data, list):
            raise ContractCodecError("frozenset field must be a JSON array")
        (item_type,) = get_args(annotation)
        return frozenset(_decode_as(item, item_type) for item in data)
    if annotation is Decimal:
        if not isinstance(data, str):
            raise ContractCodecError("Decimal field must be a canonical JSON string")
        value = Decimal(data)
        if not value.is_finite():
            raise ContractCodecError("non-finite Decimal is forbidden")
        return value
    if isinstance(annotation, type) and issubclass(annotation, OpaqueId):
        if not isinstance(data, dict):
            raise ContractCodecError(f"{annotation.__name__} must have an ID tag")
        if data.get("$id_type") != annotation.__name__:
            raise ContractCodecError(
                f"expected {annotation.__name__}, got {data.get('$id_type')}"
            )
        if set(data) != {"$id_type", "value"}:
            raise ContractCodecError("unexpected opaque ID fields")
        return annotation(data["value"])
    if isinstance(annotation, type) and issubclass(annotation, Enum):
        try:
            return annotation(data)
        except ValueError as exc:
            raise ContractCodecError(str(exc)) from exc
    if isinstance(annotation, type) and is_dataclass(annotation):
        if not isinstance(data, dict):
            raise ContractCodecError(f"{annotation.__name__} must be a JSON object")
        if data.get("$type") != annotation.__name__:
            raise ContractCodecError(
                f"expected record {annotation.__name__}, got {data.get('$type')}"
            )
        expected = {field.name for field in fields(annotation)} | {"$type"}
        extra = set(data) - expected
        missing = expected - set(data)
        if extra or missing:
            raise ContractCodecError(
                f"{annotation.__name__} field mismatch; extra={sorted(extra)}, missing={sorted(missing)}"
            )
        hints = get_type_hints(annotation)
        kwargs = {
            field.name: _decode_as(data[field.name], hints[field.name])
            for field in fields(annotation)
        }
        return annotation(**kwargs)
    if annotation is bool:
        if type(data) is not bool:
            raise ContractCodecError("expected bool")
        return data
    if annotation is int:
        if type(data) is not int:
            raise ContractCodecError("expected int")
        return data
    if annotation is float:
        if type(data) not in (int, float):
            raise ContractCodecError("expected finite number")
        value = float(data)
        if value != value or value in (float("inf"), float("-inf")):
            raise ContractCodecError("non-finite float is forbidden")
        return value
    if annotation is str:
        if not isinstance(data, str):
            raise ContractCodecError("expected string")
        return data
    raise ContractCodecError(f"unsupported annotation: {annotation!r}")


class ContractCodecV1(Generic[T]):
    root_type: type[T]

    @classmethod
    def dumps(cls, record: T) -> bytes:
        if type(record) is not cls.root_type:
            raise ContractCodecError(
                f"{cls.__name__} expects {cls.root_type.__name__}, got {type(record).__name__}"
            )
        return canonical_json_bytes(record)

    @classmethod
    def loads(cls, payload: bytes | str) -> T:
        text = payload.decode("utf-8") if isinstance(payload, bytes) else payload
        try:
            data = json.loads(text, parse_constant=_reject_constant)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ContractCodecError(str(exc)) from exc
        record = _decode_as(data, cls.root_type)
        if type(record) is not cls.root_type:
            raise ContractCodecError("decoded root type mismatch")
        return record


class AnalysisSnapshotCodecV1(ContractCodecV1[analysis.AnalysisSnapshotV1]):
    root_type = analysis.AnalysisSnapshotV1


class DecalRequestCodecV1(ContractCodecV1[request.DecalRequestV1]):
    root_type = request.DecalRequestV1


class CompiledPlanCodecV1(ContractCodecV1[plan.CompiledPatchEvaluationPlanV1]):
    root_type = plan.CompiledPatchEvaluationPlanV1


class EvaluationGeometryBindingCodecV1(
    ContractCodecV1[plan.EvaluationGeometryBindingV1]
):
    root_type = plan.EvaluationGeometryBindingV1


class ChainStraightEvaluationGeometryBindingCodecV2(
    ContractCodecV1[plan.ChainStraightEvaluationGeometryBindingV2]
):
    root_type = plan.ChainStraightEvaluationGeometryBindingV2


class GeometryBatchCodecV1(ContractCodecV1[geometry_batch.GeometryBatchV1]):
    root_type = geometry_batch.GeometryBatchV1


class EnvelopeDebugSceneCodecV1(
    ContractCodecV1[debug.EnvelopeDebugSceneV1]
):
    root_type = debug.EnvelopeDebugSceneV1


class RationalAffinePlanarMetricCodecV2(
    ContractCodecV1[metric.RationalAffinePlanarMetricV2]
):
    root_type = metric.RationalAffinePlanarMetricV2


class RuntimePlanarMetricCodecV1(
    ContractCodecV1[metric.RuntimePlanarMetricV1]
):
    root_type = metric.RuntimePlanarMetricV1
