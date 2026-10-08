"""Версионированные deterministic JSON codecs для публичных contracts."""

from __future__ import annotations

import json
import types
from dataclasses import dataclass, fields, is_dataclass
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
from .schema import is_wire_default_field
from ._metric_wire import (
    MetricNormalWireDispositionV1,
    MetricWireCompatibilityReceiptV1,
    canonicalize_metric_wire_record,
    positive_scaled_normal_compatibility_policy,
)


T = TypeVar("T")


class ContractCodecError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class ContractCodecLoadResultV1(Generic[T]):
    record: T
    compatibility_receipt: MetricWireCompatibilityReceiptV1


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


#: Кодировщик тех же настроек, что у `json.dumps(..., ensure_ascii=False, allow_nan=False, sort_keys=True,
#: separators=(",", ":"))`: `dumps` с такими ключами строит НОВЫЙ `JSONEncoder` на каждый вызов, а ключ
#: сортировки множества считается на каждый его элемент (на батче `building` — тысячи вызовов). Байты те же.
_ENCODER = json.JSONEncoder(
    ensure_ascii=False,
    allow_nan=False,
    sort_keys=True,
    separators=(",", ":"),
)


def _build_text_encoder():
    """`value -> str` тех же настроек без объекта `JSONEncoder` на каждый вызов.

    `JSONEncoder.encode` собирает C-кодировщик (`c_make_encoder`) заново на КАЖДЫЙ вызов, а ключей
    сортировки на батче тысячи (семь микросекунд на вызов, четверть времени дайджеста). Здесь тот же
    C-кодировщик собирается один раз с теми же параметрами, что `JSONEncoder` отдаёт ему при
    `ensure_ascii=False, allow_nan=False, sort_keys=True, check_circular=False, separators=(",", ":")`;
    круговой ссылки в каноническом дереве из `dict`/`list`/`str`/`int` быть не может. Нет C-версии (не
    CPython) — прежний путь через `_ENCODER`.
    """

    try:
        from json.encoder import c_make_encoder, encode_basestring
    except ImportError:  # pragma: no cover - не CPython
        return _ENCODER.encode
    if c_make_encoder is None:  # pragma: no cover - не CPython
        return _ENCODER.encode

    def unsupported(value):  # pragma: no cover - дерево собирает `to_canonical_data`
        raise TypeError(f"Object of type {value.__class__.__name__} is not JSON serializable")

    iterencode = c_make_encoder(
        None, unsupported, encode_basestring, None, ":", ",", True, False, False
    )

    def encode(value) -> str:
        return "".join(iterencode(value, 0))

    return encode


_encode_text = _build_text_encoder()

#: `класс -> (имена его полей, {имя: умолчание})` для записей, которые идут обычным путём (не `OpaqueId`,
#: не `Enum`, не `Decimal`): `dataclasses.fields` строит кортеж на каждый вызов, а записей в батче тысячи.
#: Словарь умолчаний — поля «опускается на проводе, пока равно умолчанию» (`schema.wire_default_field`):
#: быстрый путь пропускает их тем же условием, что и обычный.
_RECORD_FIELDS: dict[type, tuple[tuple[str, ...], dict[str, Any]]] = {}

#: Классы идентификаторов (`OpaqueId` и наследники), уже встреченные обычным путём: почти каждое поле
#: записи — идентификатор, и `isinstance` по цепочке типов на каждом из них — лишняя цена.
_OPAQUE_KINDS: set[type] = set()


def _json_sort_key(value: Any) -> bytes:
    return _encode_text(value).encode("utf-8")


def to_canonical_data(value: Any) -> Any:
    kind = type(value)
    known = _RECORD_FIELDS.get(kind)
    if known is not None:
        names, wire_defaults = known
        result: dict[str, Any] = {"$type": kind.__name__}
        if wire_defaults:
            for name in names:
                held = getattr(value, name)
                if name in wire_defaults and held == wire_defaults[name]:
                    continue
                result[name] = to_canonical_data(held)
        else:
            for name in names:
                result[name] = to_canonical_data(getattr(value, name))
        return result
    if kind is str or kind is int or kind is bool or value is None:
        return value
    if kind in _OPAQUE_KINDS:
        return {"$id_type": kind.__name__, "value": value.value}
    # Точные `tuple`/`frozenset` не бывают ни `OpaqueId`, ни `Enum`, ни `Decimal`, ни записью: ветки ниже
    # для них те же, а четыре проверки типа до них не нужны.
    if kind is tuple:
        return [to_canonical_data(item) for item in value]
    if kind is frozenset:
        return sorted([to_canonical_data(item) for item in value], key=_json_sort_key)
    if isinstance(value, OpaqueId):
        _OPAQUE_KINDS.add(kind)
        return {"$id_type": type(value).__name__, "value": value.value}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Decimal):
        return _canonical_decimal(value)
    if is_dataclass(value) and not isinstance(value, type):
        result = {"$type": type(value).__name__}
        for field in fields(value):
            held = getattr(value, field.name)
            if is_wire_default_field(field) and held == field.default:
                continue
            result[field.name] = to_canonical_data(held)
        _RECORD_FIELDS[kind] = (
            tuple(field.name for field in fields(value)),
            {field.name: field.default for field in fields(value) if is_wire_default_field(field)},
        )
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
    return _encode_text(to_canonical_data(value)).encode("utf-8")


def _reject_constant(value: str) -> None:
    raise ContractCodecError(f"non-finite JSON number is forbidden: {value}")


class _ReadPlan:
    """Что чтение записи берёт из определения её класса, посчитанное ОДИН раз на класс.

    `get_type_hints` разбирает строковые аннотации (`from __future__ import annotations`) через `eval` на КАЖДЫЙ вызов,
    а `dataclasses.fields` строит кортеж на каждый вызов; запись читается тысячами раз на снапшоте (разбор подсказок был
    около 60 % времени чтения снапшота `building`). Определение класса за жизнь процесса не меняется, поэтому ни план, ни
    подсказки не пересчитываются. Подсказки разрешаются ЛЕНИВО, при первом чтении записи этого класса и ПОСЛЕ проверки
    её полей — ровно там, где их звали раньше: класс с неразрешимой ссылкой отказывает тем же `NameError` на каждом
    чтении, а отказ не запоминается.
    """

    __slots__ = ("names", "expected", "required", "wire_defaults", "_hints")

    def __init__(self, annotation: type) -> None:
        record_fields = fields(annotation)
        self.names = tuple(item.name for item in record_fields)
        self.expected = frozenset(self.names) | {"$type"}
        self.required = frozenset(
            item.name for item in record_fields if not is_wire_default_field(item)
        ) | {"$type"}
        self.wire_defaults = tuple(
            (item.name, item.default) for item in record_fields if is_wire_default_field(item)
        )
        self._hints: dict[str, Any] | None = None

    def hints(self, annotation: type) -> dict[str, Any]:
        if self._hints is None:
            self._hints = get_type_hints(annotation)
        return self._hints


#: `класс записи -> _ReadPlan`: класс попадает сюда, только пройдя ветку записи в `_decode_as` (не `OpaqueId`, не `Enum`).
_READ_PLANS: dict[type, _ReadPlan] = {}


def _read_plan(annotation: type) -> _ReadPlan:
    plan = _READ_PLANS.get(annotation)
    if plan is None:
        plan = _READ_PLANS[annotation] = _ReadPlan(annotation)
    return plan


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
        plan = _read_plan(annotation)
        extra = set(data) - plan.expected
        missing = plan.required - set(data)
        if extra or missing:
            raise ContractCodecError(
                f"{annotation.__name__} field mismatch; extra={sorted(extra)}, missing={sorted(missing)}"
            )
        hints = plan.hints(annotation)
        kwargs = {
            name: _decode_as(data[name], hints[name])
            for name in plan.names
            if name in data
        }
        for name, default in plan.wire_defaults:
            if name in kwargs and kwargs[name] == default:
                raise ContractCodecError(
                    f"{annotation.__name__}.{name} equals its default and must be omitted on the wire"
                )
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
        # Writer side is strict: compatibility exists only on the explicit
        # receipt-bearing read boundary below.
        canonical, receipt = canonicalize_metric_wire_record(record)
        _require_accepted_metric_normal_wire(receipt)
        return canonical_json_bytes(canonical)

    @classmethod
    def _decode_record(cls, payload: bytes | str) -> T:
        text = payload.decode("utf-8") if isinstance(payload, bytes) else payload
        try:
            data = json.loads(text, parse_constant=_reject_constant)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ContractCodecError(str(exc)) from exc
        record = _decode_as(data, cls.root_type)
        if type(record) is not cls.root_type:
            raise ContractCodecError("decoded root type mismatch")
        return record

    @classmethod
    def loads_with_compatibility_receipt(
        cls, payload: bytes | str
    ) -> ContractCodecLoadResultV1[T]:
        record = cls._decode_record(payload)
        canonical, receipt = canonicalize_metric_wire_record(
            record,
            policy=positive_scaled_normal_compatibility_policy(),
        )
        _require_accepted_metric_normal_wire(receipt)
        return ContractCodecLoadResultV1(canonical, receipt)

    @classmethod
    def loads(cls, payload: bytes | str) -> T:
        record = cls._decode_record(payload)
        canonical, receipt = canonicalize_metric_wire_record(record)
        _require_accepted_metric_normal_wire(receipt)
        return canonical


def _require_accepted_metric_normal_wire(
    receipt: MetricWireCompatibilityReceiptV1,
) -> None:
    accepted = {
        MetricNormalWireDispositionV1.CANONICAL_PRIMITIVE,
        MetricNormalWireDispositionV1.POSITIVE_SCALED_COMPATIBILITY_APPLIED,
    }
    rejected = next(
        (item for item in receipt.dispositions if item not in accepted),
        None,
    )
    if rejected is not None:
        raise ContractCodecError(
            f"{rejected.value}: planar-normal wire record rejected"
        )


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


class EmbeddingCertifiedRationalAffinePlanarMetricCodecV1(
    ContractCodecV1[metric.EmbeddingCertifiedRationalAffinePlanarMetricV1]
):
    root_type = metric.EmbeddingCertifiedRationalAffinePlanarMetricV1


class RuntimePlanarMetricCodecV1(
    ContractCodecV1[metric.RuntimePlanarMetricV1]
):
    root_type = metric.RuntimePlanarMetricV1
