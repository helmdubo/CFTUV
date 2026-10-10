"""Прежний (до таблицы типов и планов записи) кодировщик ключа содержимого: СВЕРКА, а не продукт.

Копия кода `886bc7c4` слово в слово: рекурсивный обход с цепочкой проверок типа и `_field` на каждое поле. Продуктовый кодировщик
(`cftuv/envelope_content_key.py`) выбирает запись по точному типу таблицей, пишет однородные кортежи целых и вещественных одним `join` и ведёт
датакласс по плану класса, но обязан давать ТЕ ЖЕ байты на любом значении, в том числе то же исключение с тем же текстом на значении, которое не
кодируется: ключ - адрес результата в хранилище, и ключ, изменившийся без изменения входа, молча обнулил бы хранилище (или, хуже, свёл два разных
входа к одному). `tests/test_envelope_content_key_encoder.py` сравнивает оба на фикстурах, записанном полевом слепке и на случайных деревьях.
"""

from __future__ import annotations

import dataclasses
import hashlib
from enum import Enum
from fractions import Fraction

from cftuv.envelope_content_key import (
    CONTENT_KEY_SCHEMA,
    EXCLUDED_FIELDS,
    ContentKeyUnsupported,
    _normalized_budget,
    _patch_ranks,
    _policy_constants,
    _request_policy_signature,
)
from cftuv.envelope_kernel_backend import DEFAULT_KERNEL_BACKEND

_REVISION_MARK = "\x00R"
_PATCH_FIELDS = frozenset({"patch_id", "neighbor_patch_id"})
_PATCH_MAP_FIELD = "nodes"


def _fields(cls: type) -> tuple[str, ...]:
    return tuple(item.name for item in dataclasses.fields(cls))


class LegacyEncoder:
    """Детерминированная запись значения: тип и значение каждого узла, множества - по порядку записи."""

    __slots__ = ("revision", "patches")

    def __init__(self, revision: str, patches: dict[int, int]) -> None:
        self.revision = revision
        self.patches = patches

    def patch(self, number) -> str:
        if type(number) is not int:
            raise ContentKeyUnsupported(f"a patch number must be an int, got {type(number).__name__}")
        if number < 0:
            return f"p{number};"
        rank = self.patches.get(number)
        if rank is None:
            raise ContentKeyUnsupported(f"patch {number} is not the domain or one of its neighbours")
        return f"p{rank};"

    def encode(self, value) -> str:
        kind = type(value)
        if kind is int:
            return f"i{value};"
        if kind is float:
            return f"f{value.hex()};"
        if kind is str:
            if self.revision in value:
                value = value.replace(self.revision, _REVISION_MARK)
            return f"s{len(value)}:{value};"
        if kind is bool:
            return "T;" if value else "F;"
        if value is None:
            return "N;"
        if kind is tuple or kind is list:
            return "(" + "".join(self.encode(item) for item in value) + ")"
        if kind is frozenset or kind is set:
            return "{" + "".join(sorted(self.encode(item) for item in value)) + "}"
        if kind is dict:
            pairs = sorted((self.encode(key), self.encode(item)) for key, item in value.items())
            return "<" + "".join(key + item for key, item in pairs) + ">"
        if kind is Fraction:
            return f"q{value.numerator}/{value.denominator};"
        if isinstance(value, Enum):
            return f"e{kind.__qualname__}.{value.name};"
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return self._record(value, kind)
        raise ContentKeyUnsupported(f"{kind.__qualname__} is not encodable in a content key")

    def _record(self, value, kind: type) -> str:
        if kind.__qualname__ == "SourceRevision":
            return "\x00SRC;"
        return f"D{kind.__qualname__}[{''.join(self._field(value, name) for name in _fields(kind))}]"

    def _field(self, owner, name: str) -> str:
        item = getattr(owner, name)
        if name in _PATCH_FIELDS:
            return f"{name}={self.patch(item)}"
        if name == _PATCH_MAP_FIELD and type(item) is dict:
            pairs = sorted((self.patch(key), self.encode(entry)) for key, entry in item.items())
            return f"{name}=<" + "".join(key + entry for key, entry in pairs) + ">"
        return f"{name}={self.encode(item)}"


def legacy_domain_content_key(export, selected_edge_ids, band_key=None, backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None) -> str:
    """`domain_content_key` на прежнем кодировщике: те же политики, поля, нормализации и порядок частей."""

    from cftuv.envelope_request_policy import normalize_envelope_fan_density

    encoder = LegacyEncoder(export.source_revision_value, _patch_ranks(export))
    parts = [CONTENT_KEY_SCHEMA, encoder.encode(_policy_constants(backend, skeleton_backend, embedding_backend))]
    for name in _fields(type(export)):
        if name in EXCLUDED_FIELDS:
            continue
        value = getattr(export, name)
        if name == "grid_scale_law" and value is None:
            continue
        if name == "density":
            try:
                value = normalize_envelope_fan_density(value)
            except (TypeError, ValueError) as exc:
                raise ContentKeyUnsupported(f"density is not canonical: {exc}") from exc
        elif name == "developable_stretch_budget":
            value = _normalized_budget(value)
        parts.append(f"{name}={encoder.encode(value)}")
    parts.append("policy=" + encoder.encode(_request_policy_signature(export.density, export.developable_stretch_budget)))
    parts.append("selected=" + encoder.encode(tuple(sorted(int(item) for item in selected_edge_ids))))
    parts.append("band=" + encoder.encode(band_key))
    return hashlib.sha256("\x1e".join(parts).encode("utf-8")).hexdigest()
