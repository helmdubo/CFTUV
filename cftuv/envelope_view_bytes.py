"""Вид домена одной строкой байтов (VIEW_BYTES_V1): воркер пишет, родитель разбирает лениво в ТЕ ЖЕ кортежи, что у `build_domain_view`.

ЗАЧЕМ. Вид домена (`DomainViewV1`) ехал от воркера пиклом графа кортежей: на `cover.008` w12 разбор ответов стоил родителю ~0.22 с процессора на полном нажатии,
и ~70 % этого - разворот датаклассов со слотами (`_dataclass_setstate` -> `fields()` на каждый объект). Теперь вид едет одним полем байтов (`view_bytes`), разбор - одна
функция (`decode_view`) при первом чтении `view`, а токен структуры (`structure_token`) воркер кладёт в заголовок: родитель его не считает.

ТОЧНОСТЬ ПО ПОСТРОЕНИЮ. Байты повторяют вид без потерь: числа - IEEE-754 binary64 (знак нуля сохранён), индексы - целые, строки - UTF-8 дословно (`location:src:<id>`
не разбирается в число), порядок - тот же. Вид, который байты не повторили бы точно (число не `float`, индекс не `int`, контейнер не кортеж), не кодируется вовсе
(`ViewBytesUnsupported`): воркер шлёт его объектом, как прежде. Поэтому все читатели вида (`build_mesh_arrays`, составной образец, сдвиг, перенос, `view_hash`)
получают побитово тот же вид, где бы его ни считали.

РАСКЛАДКА (little-endian). Заголовок `_HEADER`; затем `float64`: позиции (3V), нормали смещения (3V), UV (2L); `int64`: владельцы граней (F); `uint32`: начала граней (F+1),
углы граней, швы (2S), ссылки вершин (V: 0 - `None`, иначе номер строки + 1), начала стен (W+1), вершины стен, имена цепей (C), начала цепей (C+1), ключи цепей,
смещения строк (T+1); затем строки UTF-8 подряд. У вида-отказа (`failure`) чисел нет: заголовок и строки (ревизия, исход, деталь).
"""

from __future__ import annotations

import struct
import sys
from array import array

VIEW_BYTES_VERSION = 1
_MAGIC = b"CFVB"
_FAILED = 1
#: magic, версия, флаги, patch_id, V, F, углы, L (пары UV), S, W, вершины стен, C, ключи цепей, T (строки), байты строк, ревизия, исход, деталь, токен структуры
_HEADER = struct.Struct("<4sHHq14I20s")
_TOKEN_BYTES = 20


class ViewBytesUnsupported(ValueError):
    """Вид, который байты не повторили бы точно, либо байты не этой версии: такой вид едет объектом."""


def _float_rows(rows, width: int) -> array:
    if type(rows) is not tuple or not all(type(row) is tuple and len(row) == width for row in rows):
        raise ViewBytesUnsupported(f"rows of {width} numbers are not tuples")
    flat = [value for row in rows for value in row]
    if not all(type(value) is float for value in flat):
        raise ViewBytesUnsupported("a coordinate is not a float")
    return array("d", flat)


def _index_rows(rows) -> tuple:
    """`(начала, плоские номера)` кортежа кортежей целых."""

    if type(rows) is not tuple or not all(type(row) is tuple for row in rows):
        raise ViewBytesUnsupported("index rows are not tuples")
    flat = [value for row in rows for value in row]
    if not all(type(value) is int for value in flat):
        raise ViewBytesUnsupported("an index is not an int")
    starts = [0]
    for row in rows:
        starts.append(starts[-1] + len(row))
    return array("I", starts), array("I", flat)


def encode_view(view, token: str) -> bytes:
    """Байты вида (`DomainViewV1`) и токена его структуры; `ViewBytesUnsupported` - вид, который байты не повторили бы точно."""

    try:
        return _encode(view, token)
    except ViewBytesUnsupported:
        raise
    except (OverflowError, TypeError, ValueError) as exc:  # номер вне uint32/int64, строка не в UTF-8, отказ не пара
        raise ViewBytesUnsupported(f"{type(exc).__name__}: {exc}") from None


def _encode(view, token: str) -> bytes:
    if sys.byteorder != "little":
        raise ViewBytesUnsupported("the host is big-endian: the view travels as an object")
    strings: dict[str, int] = {}

    def number_of(text) -> int:
        if type(text) is not str:
            raise ViewBytesUnsupported("a text of the view is not a str")
        found = strings.get(text)
        if found is None:
            found = strings[text] = len(strings)
        return found

    revision = number_of(view.source_revision)
    if view.failure is not None:
        if type(view.failure) is not tuple or len(view.failure) != 2:
            raise ViewBytesUnsupported("the failure is not an (outcome, detail) pair")
        outcome, detail = view.failure
        return _pack(0, _FAILED, (0,) * 10, revision, number_of(outcome), number_of(detail), "", (), strings)
    vertices = view.vertices
    if type(vertices.patch_id) is not int:
        raise ViewBytesUnsupported("the patch id is not an int")
    positions = _float_rows(vertices.positions, 3)
    normals = _float_rows(vertices.normals, 3)
    uvs = _float_rows(view.uvs, 2)
    count = len(vertices.positions)
    if len(vertices.normals) != count or type(vertices.refs) is not tuple or len(vertices.refs) != count:
        raise ViewBytesUnsupported("the vertex rows differ in length")
    if type(view.owners) is not tuple or not all(type(value) is int for value in view.owners) or len(view.owners) != len(view.faces):
        raise ViewBytesUnsupported("the face owners are not one int per face")
    owners = array("q", view.owners)
    face_starts, corners = _index_rows(view.faces)
    seam_starts, seams = _index_rows(view.seams)
    if any(b - a != 2 for a, b in zip(seam_starts, seam_starts[1:])):
        raise ViewBytesUnsupported("a seam is not a pair")
    refs = array("I", (0 if ref is None else number_of(ref) + 1 for ref in vertices.refs))
    wall_starts, wall_items = _index_rows(vertices.walls)
    chains = view.boundary_chains
    if type(chains) is not tuple or not all(type(chain) is tuple and len(chain) == 2 and type(chain[1]) is tuple for chain in chains):
        raise ViewBytesUnsupported("the boundary chains are not (name, keys) tuples")
    chain_names = array("I", (number_of(name) for name, _keys in chains))
    chain_starts = array("I", [0])
    chain_items = array("I")
    for _name, keys in chains:
        chain_items.extend(number_of(key) for key in keys)
        chain_starts.append(len(chain_items))
    if len(token.encode("ascii")) != _TOKEN_BYTES:
        raise ViewBytesUnsupported("the structure token is not 20 characters")
    counts = (count, len(view.faces), len(corners), len(view.uvs), len(view.seams), len(vertices.walls), len(wall_items), len(chains), len(chain_items), 0)
    body = (positions, normals, uvs, owners, face_starts, corners, seams, refs, wall_starts, wall_items, chain_names, chain_starts, chain_items)
    return _pack(vertices.patch_id, 0, counts, revision, 0, 0, token, body, strings)


def _pack(patch_id: int, flags: int, counts: tuple, revision: int, outcome: int, detail: int, token: str, body: tuple, strings: dict) -> bytes:
    texts = [text.encode("utf-8") for text in strings]
    offsets = array("I", [0])
    for text in texts:
        offsets.append(offsets[-1] + len(text))
    blob = b"".join(texts)
    head = _HEADER.pack(_MAGIC, VIEW_BYTES_VERSION, flags, patch_id, *counts[:9], len(texts), len(blob), revision, outcome, detail, token.encode("ascii").ljust(_TOKEN_BYTES, b"\0"))
    return b"".join((head, *(part.tobytes() for part in body), offsets.tobytes(), blob))


def put_view(state: dict, result) -> None:
    """Вид результата домена в состояние его копии (`deferred_result`, `lean_result`): байтами (`view_bytes`; `view` разберёт первое чтение), а вид, который байты
    не повторили бы точно, - объектом (`view`), как прежде: читатели получают тот же вид.

    VIEW_BYTES_NATIVE_V1: байты считает порт (`native_view_bytes`), эталон - этот же путь питона; отказ порта (вид-отказ писателя, вход, который порт не читает как питон)
    назван в журнале сборки меша, и вид считает питон.
    """

    from .envelope_production_view import build_domain_view, structure_token

    data = native_view_bytes(result)
    if data is not None:
        state["view_bytes"] = data
        state.pop("view", None)
        return
    view = build_domain_view(result)
    try:
        state["view_bytes"] = encode_view(view, "" if view.failure is not None else structure_token(view))
    except ViewBytesUnsupported:
        state["view"] = view
        return
    state.pop("view", None)


#: VIEW_BYTES_NATIVE_V1: операция порта (`cftuv_native.domain_view_bytes`, `native/cftuv-python/src/domain_view.rs`); эталон - `build_domain_view`, `structure_token` и `encode_view`.
VIEW_BYTES_OPERATION = "domain_view_bytes"


def native_view_bytes(result) -> bytes | None:
    """Байты вида порта либо `None` (заказан эталон, расширения нет, порт отказал вход по имени: откат записан в журнал сборки меша, вид считает питон)."""

    from .envelope_mesh_assembly import native_answer

    return native_answer(
        VIEW_BYTES_OPERATION, result.batch, result.normal, getattr(result, "vertex_normals", ()), getattr(result, "source_normal", None), result.patch_id
    )


def cached_view(result):
    """`view` результата с видом байтами: разобран при первом чтении и запомнен на результате (второе чтение не разбирает); `None` - байтов нет."""

    data = result.__dict__.get("view_bytes")
    if data is None:
        return None
    view = decode_view(data)
    object.__setattr__(result, "view", view)
    return view


def decode_view(data: bytes):
    """`DomainViewV1` из байтов `encode_view`: те же кортежи и строки, тот же токен структуры (`token`)."""

    from .envelope_production_view import DomainViewV1
    from .envelope_production_weld import DomainVerticesV1

    header = _HEADER.unpack_from(data, 0)
    if header[0] != _MAGIC or header[1] != VIEW_BYTES_VERSION:
        raise ViewBytesUnsupported(f"not VIEW_BYTES_V{VIEW_BYTES_VERSION} bytes")
    (_magic, _version, flags, patch_id, count, faces, corners, loops, seams, walls, wall_items, chains, chain_items, strings, blob, revision, outcome, detail, token) = header
    offset = _HEADER.size

    def take(code: str, size: int) -> array:
        nonlocal offset
        values = array(code)
        end = offset + values.itemsize * size
        values.frombytes(data[offset:end])
        offset = end
        return values

    if flags & _FAILED:
        texts = _texts(take("I", strings + 1), data[offset:offset + blob])
        return DomainViewV1((texts[outcome], texts[detail]), None, (), (), (), (), (), texts[revision])
    positions = iter(take("d", 3 * count).tolist())
    normals = iter(take("d", 3 * count).tolist())
    uvs = iter(take("d", 2 * loops).tolist())
    owners = tuple(take("q", faces).tolist())
    face_starts = take("I", faces + 1).tolist()
    face_corners = take("I", corners).tolist()
    seam_pairs = iter(take("I", 2 * seams).tolist())
    refs = take("I", count)
    wall_starts = take("I", walls + 1).tolist()
    wall_flat = take("I", wall_items).tolist()
    chain_names = take("I", chains).tolist()
    chain_starts = take("I", chains + 1).tolist()
    chain_flat = take("I", chain_items).tolist()
    texts = _texts(take("I", strings + 1), data[offset:offset + blob])
    lookup = (None, *texts)
    face_rings = tuple(tuple(face_corners[a:b]) for a, b in zip(face_starts, face_starts[1:]))
    vertices = DomainVerticesV1(
        patch_id,
        tuple(zip(positions, positions, positions)),
        tuple(zip(normals, normals, normals)),
        tuple(map(lookup.__getitem__, refs)),
        tuple(tuple(wall_flat[a:b]) for a, b in zip(wall_starts, wall_starts[1:])),
        face_rings,
    )
    return DomainViewV1(
        None,
        vertices,
        face_rings,
        tuple(zip(uvs, uvs)),
        owners,
        tuple(zip(seam_pairs, seam_pairs)),
        tuple((texts[name], tuple(map(texts.__getitem__, chain_flat[a:b]))) for name, a, b in zip(chain_names, chain_starts, chain_starts[1:])),
        texts[revision],
        token.rstrip(b"\0").decode("ascii"),
    )


def view_parts(data: bytes):
    """Что читает составной образец у вида байтами, БЕЗ разбора в кортежи (`decode_view`): `(число вершин, UV (L, 2) float64, владельцы граней int64, ссылки вершин, токен структуры)`;
    `None` - вид-отказ. Числа - те же binary64 и целые (окна `np.frombuffer` на те же байты, только чтение), ссылки - те же строки (`None` - без ссылки)."""

    import numpy as np

    header = _HEADER.unpack_from(data, 0)
    if header[0] != _MAGIC or header[1] != VIEW_BYTES_VERSION:
        raise ViewBytesUnsupported(f"not VIEW_BYTES_V{VIEW_BYTES_VERSION} bytes")
    (_magic, _version, flags, _patch_id, count, faces, corners, loops, seams, walls, wall_items, chains, chain_items, strings, blob, _revision, _outcome, _detail, token) = header
    if flags & _FAILED:
        return None
    at = _HEADER.size + 8 * 6 * count
    uvs = np.frombuffer(data, dtype="<f8", count=2 * loops, offset=at).reshape(-1, 2)
    at += 16 * loops
    owners = np.frombuffer(data, dtype="<i8", count=faces, offset=at)
    at += 8 * faces + 4 * (faces + 1 + corners + 2 * seams)
    refs = np.frombuffer(data, dtype="<u4", count=count, offset=at)
    at += 4 * (count + walls + 1 + wall_items + chains + chains + 1 + chain_items)
    offsets = np.frombuffer(data, dtype="<u4", count=strings + 1, offset=at)
    text_at = at + 4 * (strings + 1)
    lookup: dict = {0: None}
    for number in set(refs.tolist()) - {0}:
        lookup[number] = data[text_at + int(offsets[number - 1]) : text_at + int(offsets[number])].decode("utf-8")
    return int(count), uvs, owners, tuple(map(lookup.__getitem__, refs.tolist())), token.rstrip(b"\0").decode("ascii")


def view_bytes_failed(data: bytes) -> bool:
    """Байты вида-отказа (`failure`): флаг заголовка, без разбора; не байты этой версии - `ViewBytesUnsupported`."""

    header = _HEADER.unpack_from(data, 0)
    if header[0] != _MAGIC or header[1] != VIEW_BYTES_VERSION:
        raise ViewBytesUnsupported(f"not VIEW_BYTES_V{VIEW_BYTES_VERSION} bytes")
    return bool(header[2] & _FAILED)


def _texts(offsets: array, blob: bytes) -> list:
    return [blob[a:b].decode("utf-8") for a, b in zip(offsets, offsets[1:])]


__all__ = ("VIEW_BYTES_VERSION", "ViewBytesUnsupported", "cached_view", "decode_view", "encode_view", "put_view", "view_bytes_failed", "view_parts")
