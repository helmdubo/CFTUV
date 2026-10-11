"""Проверка «меш источника тот же, что при кнопке» для шага ширины: отпечаток массивов меша вместо обхода bmesh на каждом шаге.

ЗАЧЕМ. Каждый шаг живой ширины (`envelope_width_live._begin`) сверял отпечаток меша источника с записью кнопки: `bmesh.from_mesh` и `source_revision_from_bmesh`
(sha256 по шестнадцатеричным записям каждой вершины, ребра и грани) стоили 4 мс на `building` и 36 мс на `cover.008` на КАЖДОМ шаге, хотя меш за шаги не менялся.

ПОЧЕМУ ЭТО ДОКАЗАНО, А НЕ УГАДАНО. Отпечаток ревизии - чистая функция содержимого bmesh: позиций вершин (`vertex.co`), пар вершин рёбер и их шва, циклов вершин граней. Всё это
bmesh получает из массивов меша (`bm.from_mesh`): `vertices.co`, `edges.vertices` и `edges.use_seam`, `loops.vertex_index` с началами и длинами граней (`polygons.loop_start`,
`polygons.loop_total`). Эти массивы читаются за доли миллисекунды (`foreach_get`) и хэшируются побайтно. Равные байты всех шести массивов - равный bmesh - равный отпечаток
ревизии: поэтому отпечаток, посчитанный полным путём один раз, отдаётся для любого меша с теми же байтами, а любой иной байт любого из шести массивов (и режим Edit, чьи данные в
массивах меша ещё не лежат) отправляет шаг полным путём. Ключ строже функции (порядок пары вершин ребра, например, на отпечаток ревизии не влияет): лишний промах возможен, лишнее
попадание - нет.

ЧТО НЕ ЗАЩИЩАЕТ. Объект (матрица, имя), модификаторы и всё вне шести массивов в отпечаток ревизии и не входят: они ему безразличны и здесь тоже.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class SourceCheckV1:
    """Отпечаток массивов меша и отпечаток ревизии, которые полный путь дал для него."""

    fingerprint: bytes
    digest: str


def _bytes_of(collection, name: str, dtype, size: int = 1) -> bytes:
    buffer = np.empty(len(collection) * size, dtype=dtype)
    collection.foreach_get(name, buffer)
    return buffer.tobytes()


def mesh_fingerprint(source_obj) -> bytes | None:
    """Байты шести массивов меша источника под одним хэшем, либо `None` (в режиме Edit данные ещё не в массивах меша; объект без меша)."""

    if getattr(source_obj, "mode", "OBJECT") == "EDIT":
        return None
    mesh = getattr(source_obj, "data", None)
    try:
        parts = (
            (len(mesh.vertices), len(mesh.edges), len(mesh.loops), len(mesh.polygons)),
            _bytes_of(mesh.vertices, "co", np.float32, 3),
            _bytes_of(mesh.edges, "vertices", np.int32, 2),
            _bytes_of(mesh.edges, "use_seam", np.bool_),
            _bytes_of(mesh.loops, "vertex_index", np.int32),
            _bytes_of(mesh.polygons, "loop_start", np.int32),
            _bytes_of(mesh.polygons, "loop_total", np.int32),
        )
    except (AttributeError, TypeError, RuntimeError, ValueError):
        return None  # не меш Blender (или его массивы не читаются): шаг идёт полным путём
    digest = hashlib.blake2b(digest_size=32)
    digest.update(repr(parts[0]).encode("ascii"))
    for part in parts[1:]:
        digest.update(part)
    return digest.digest()


def source_digest_now(controller, source_obj, compute) -> str:
    """Отпечаток ревизии меша источника СЕЙЧАС: из записи сессии, если байты шести массивов те же, иначе `compute(source_obj)` (полный путь) и новая запись.

    Запись одна на сессию (`controller.width_source_check`): шаги одного перетаскивания идут по одному мешу, и правка меша между ними даёт другие байты.
    """

    fingerprint = mesh_fingerprint(source_obj)
    cached = controller.width_source_check
    if fingerprint is not None and cached is not None and cached.fingerprint == fingerprint:
        return cached.digest
    digest = compute(source_obj)
    controller.width_source_check = None if fingerprint is None else SourceCheckV1(fingerprint, digest)
    return digest


__all__ = ("SourceCheckV1", "mesh_fingerprint", "source_digest_now")
