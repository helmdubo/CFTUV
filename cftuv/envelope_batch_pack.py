"""Упаковка батча отложенного результата (COMMIT_BATCH_PACK_V1): граф `(GeometryBatchV1, нормали вершин)`, который воркер шлёт родителю на COMMIT.

Раньше это был `pickle.dumps` графа записей ядра: у замороженного slots-датакласса пикл зовёт питоновский `_dataclass_getstate` (и `dataclasses.fields`)
на КАЖДОЙ записи, и на `cover008` это 1.5 с ЦП воркеров на каждый COMMIT. Теперь граф пакует Rust (`cftuv_native.graph_pack`, `graph_pack.rs`) одним
проходом, а родитель распаковывает его в ТОТ ЖЕ граф, что дал бы `pickle.loads`: те же классы и значения (числа с плавающей точкой побитово), тот же
порядок вставки каждого `frozenset`, то же разделение объектов (доказательство - `tests/test_native_graph_pack.py`: пикл распакованного графа - байты
пикла распикленного).

`CFTUV_BATCH_PACK` (`NATIVE` | `PICKLE`, по умолчанию `NATIVE`) заказывает упаковку. Отказ порта (граф, которого он не несёт; расширения нет) не тихий:
назван в `FALLBACKS`, напечатан один раз на причину, и батч пиклится как раньше. Распаковка узнаёт формат по первым байтам (`MAGIC` либо пикл), так
что заказ на стороне родителя ничего не решает; буфер порта без расширения распаковывает эталон формата `reference_unpack` (тот же граф).
"""

from __future__ import annotations

import os
import pickle

#: Первые байты буфера порта (`graph_pack.rs`): пикл протокола 5 начинается с `\\x80\\x05`, их не спутать.
MAGIC = b"CGP1"
PICKLE_PROTOCOL = 5
BATCH_PACK_ENVIRONMENT = "CFTUV_BATCH_PACK"
BATCH_PACK_NATIVE = "NATIVE"
BATCH_PACK_PICKLE = "PICKLE"
DEFAULT_BATCH_PACK = BATCH_PACK_NATIVE
#: `{(исход, операция): [число, текст первой причины]}` откатов упаковки на пикл за процесс.
FALLBACKS: dict = {}


def batch_pack_order() -> str:
    """Заказ среды процесса; неизвестное имя - `ValueError`."""

    text = os.environ.get(BATCH_PACK_ENVIRONMENT, "").strip().upper() or DEFAULT_BATCH_PACK
    if text not in (BATCH_PACK_NATIVE, BATCH_PACK_PICKLE):
        raise ValueError(f"unknown {BATCH_PACK_ENVIRONMENT} {text!r}: expected NATIVE or PICKLE")
    return text


def _native(operation: str, argument) -> tuple:
    from cftuv_envelope.backend import graph_pack_compute

    done, answer, fallbacks = graph_pack_compute(operation, argument)
    for key, (count, detail) in fallbacks.items():
        slot = FALLBACKS.get(key)
        if slot is None:
            FALLBACKS[key] = [count, detail]
            print(f"[CFTUV][Production] BATCH PACK: {key[1]} by pickle ({key[0]}: {detail})")
        else:
            slot[0] += count
    return done, answer


def pack(value) -> bytes:
    """Байты графа `value`: буфер порта либо (заказан пикл, откат назван) пикл протокола 5."""

    if batch_pack_order() == BATCH_PACK_NATIVE:
        done, answer = _native("graph_pack", value)
        if done:
            return answer
    return pickle.dumps(value, protocol=PICKLE_PROTOCOL)


def unpack(data: bytes):
    """Граф из байтов `pack`: буфер порта - портом (без расширения - эталоном формата), пикл - `pickle.loads`."""

    if not data.startswith(MAGIC):
        return pickle.loads(data)
    done, answer = _native("graph_unpack", data)
    return answer if done else reference_unpack(data)


# --------------------------------------------------------------------------
# Эталон формата (тот, что пишет `graph_pack.rs`): распаковка без расширения
# --------------------------------------------------------------------------

_T_NONE, _T_FALSE, _T_TRUE, _T_INT, _T_FLOAT, _T_STR, _T_BYTES, _T_TUPLE, _T_FROZENSET, _T_DECIMAL, _T_ENUM, _T_RECORD, _T_REF, _T_EMPTY_TUPLE = range(14)
_KIND_RECORD, _KIND_ENUM = 0, 1
_DEPTH_LIMIT = 200


class _Reader:
    def __init__(self, data: bytes):
        self.data, self.at = data, len(MAGIC)

    def take(self, count: int) -> bytes:
        if self.at + count > len(self.data):
            raise ValueError("COMMIT_BATCH_PACK_V1: the buffer ends early")
        found = self.data[self.at : self.at + count]
        self.at += count
        return found

    def varint(self) -> int:
        value = shift = 0
        while True:
            byte = self.take(1)[0]
            value |= (byte & 0x7F) << shift
            if not byte & 0x80:
                return value
            shift += 7

    def text(self) -> str:
        return self.take(self.varint()).decode("utf-8")


def reference_unpack(data: bytes):
    """Эталон формата: тот же граф, что строит `graph_unpack` (и `pickle.loads` пикла исходного графа)."""

    import dataclasses
    import decimal
    import importlib
    import struct

    if not data.startswith(MAGIC):
        raise ValueError("COMMIT_BATCH_PACK_V1: not a COMMIT_BATCH_PACK_V1 buffer")
    reader = _Reader(data)
    classes = []
    for _ in range(reader.varint()):
        kind, module, qualname = reader.take(1)[0], reader.text(), reader.text()
        names = tuple(reader.text() for _ in range(reader.varint()))
        found = importlib.import_module(module)
        for part in qualname.split("."):
            found = getattr(found, part)
        if kind == _KIND_RECORD and tuple(item.name for item in dataclasses.fields(found)) != names:
            raise ValueError(f"COMMIT_BATCH_PACK_V1: the fields of {module}.{qualname} differ from the buffer's {names}")
        classes.append((kind, found, names))
    memo: list = []

    def value(depth: int):
        if depth > _DEPTH_LIMIT:
            raise ValueError("COMMIT_BATCH_PACK_V1: nesting deeper than the packer's limit")
        tag = reader.take(1)[0]
        if tag in (_T_NONE, _T_FALSE, _T_TRUE):
            return (None, False, True)[tag]
        if tag == _T_INT:
            return struct.unpack("<q", reader.take(8))[0]
        if tag == _T_FLOAT:
            return struct.unpack("<d", reader.take(8))[0]
        if tag == _T_EMPTY_TUPLE:
            return ()
        if tag == _T_REF:
            return memo[reader.varint()]
        if tag == _T_RECORD:
            kind, cls, names = classes[reader.varint()]
            if kind != _KIND_RECORD:
                raise ValueError("COMMIT_BATCH_PACK_V1: a record of an enum class")
            found = object.__new__(cls)
            memo.append(found)
            for name in names:
                object.__setattr__(found, name, value(depth + 1))
            return found
        if tag == _T_STR:
            found = reader.text()
        elif tag == _T_BYTES:
            found = reader.take(reader.varint())
        elif tag in (_T_TUPLE, _T_FROZENSET):
            items = [value(depth + 1) for _ in range(reader.varint())]
            found = tuple(items) if tag == _T_TUPLE else frozenset(items)
        elif tag == _T_DECIMAL:
            found = decimal.Decimal(reader.text())
        elif tag == _T_ENUM:
            kind, cls, _names = classes[reader.varint()]
            if kind != _KIND_ENUM:
                raise ValueError("COMMIT_BATCH_PACK_V1: an enum member of a record class")
            found = cls(value(depth + 1))
        else:
            raise ValueError(f"COMMIT_BATCH_PACK_V1: an unknown tag {tag}")
        memo.append(found)
        return found

    found = value(0)
    if reader.at != len(data):
        raise ValueError("COMMIT_BATCH_PACK_V1: bytes after the value")
    return found


__all__ = (
    "BATCH_PACK_ENVIRONMENT",
    "BATCH_PACK_NATIVE",
    "BATCH_PACK_PICKLE",
    "DEFAULT_BATCH_PACK",
    "FALLBACKS",
    "MAGIC",
    "batch_pack_order",
    "pack",
    "reference_unpack",
    "unpack",
)
