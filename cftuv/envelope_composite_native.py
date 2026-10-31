"""COMPOSITE_SPLICE_NATIVE_V1: сварка группы составного образца и грани/швы перекладываемых доменов - нативно, прямо из байтов видов (VIEW_BYTES_V1; чистая, без Blender).

ЗАЧЕМ. Хвост потокового такта живого меша (`envelope_width_composite`) на `cover.008` стоил 40-60 мс питона под GIL: каждый обслуженный домен группы сдвигался в кортежи
(`rebased_view`), сварка читала их объектами, грани и швы перекладываемых доменов разворачивались циклами питона. Здесь те же входы - байты видов посчитанных доменов как есть и байты
видов прежних результатов обслуженных с наклонами их сертификатов - уходят одним нативным вызовом (`cftuv_native.mesh_splice_weld`, `mesh_splice_rebuilt`), сдвиг на ширину такта делает
порт теми же операциями binary64 и в том же порядке, ответ - побитово ответ питона (дифференциальные тесты `tests/test_native_composite_splice.py`).

ОТКАТ НАЗВАН. Заказан эталон (`CFTUV_MESH_ASSEMBLY_BACKEND=PYTHON`), у домена группы вид объектом, а не байтами, наклоны сертификата не байтами, порт отказал вход по имени - ответ
считает прежний путь питона (тот же ответ), причина записана в `envelope_mesh_assembly.FALLBACKS` и один раз напечатана.
"""

from __future__ import annotations

import sys
from array import array
from dataclasses import dataclass

import numpy as np

from .envelope_mesh_assembly import MESH_ASSEMBLY_NATIVE, mesh_assembly_backend, native_answer, record_fallback
from .envelope_production_weld import WELD_LAW
from .envelope_width_rebase import certificate_usable

SPLICE_WELD_OPERATION = "mesh_splice_weld"
SPLICE_REBUILT_OPERATION = "mesh_splice_rebuilt"
OUTCOME_VIEW_NOT_BYTES = "NATIVE_PORT_UNSUPPORTED"
#: Домен группы не двигается моделью и не посчитан (имя то же, что у пути питона: `envelope_width_composite.COMPOSITE_HELD_NOT_RECOMPUTED`).
HELD_NOT_RECOMPUTED = "INCREMENTAL_COMPOSITE_HELD_NOT_RECOMPUTED"


@dataclass(frozen=True, slots=True)
class NativeWeldV1:
    """Сварка группы нативно: позиции `(N, 3)`, номера вершин меша по доменам группы (`int64`) и соседи по стене - то, что пути перекладки читают из `WeldV1`."""

    positions: np.ndarray
    index: tuple
    partners: tuple


def _slope_bytes(slopes):
    """Наклоны сертификата байтами `float64` little-endian (так их читает `np.frombuffer` пути питона): байты как есть, `array("d")` - его байты; иное - `None`."""

    if type(slopes) is bytes:
        return slopes
    if type(slopes) is array and slopes.typecode == "d" and slopes.itemsize == 8 and sys.byteorder == "little":
        return slopes.tobytes()
    return None


def group_inputs(displayed, fresh: dict, ordered, operation: str = SPLICE_WELD_OPERATION):
    """`{номер домена: (байты вида, сдвиг | None)}` по `ordered` либо имя причины (домен группы не двигается моделью и не посчитан - как у пути питона), либо `None`: заказан эталон
    или у домена нет байтов (откат назван)."""

    found: dict = {}
    for number in ordered:
        if number in fresh:
            data = fresh[number][0].__dict__.get("view_bytes")
            step = None
        else:
            origin = displayed.origins.get(displayed.domains[number].patch_id)
            if origin is None or not certificate_usable(origin):
                return HELD_NOT_RECOMPUTED
            data = origin.__dict__.get("view_bytes")
            certificate = origin.parametric
            step = (_slope_bytes(certificate.position_slopes), _slope_bytes(certificate.loop_slopes), float(certificate.alpha))
            if step[0] is None or step[1] is None:
                record_fallback((OUTCOME_VIEW_NOT_BYTES, operation), 1, "the slopes of a certificate are neither bytes nor a float64 array")
                return None
        if data is None:
            record_fallback((OUTCOME_VIEW_NOT_BYTES, operation), 1, "a group domain carries its view as an object, not as VIEW_BYTES_V1 bytes")
            return None
        found[number] = (data, step)
    return found


def native_ordered() -> bool:
    return mesh_assembly_backend() == MESH_ASSEMBLY_NATIVE


def group_weld(inputs: dict, ordered, alpha: float, offset: float) -> NativeWeldV1 | None:
    """`weld_vertices` над видами группы `ordered` на ширине такта нативно либо `None` (откат назван)."""

    answer = native_answer(SPLICE_WELD_OPERATION, [inputs[number][0] for number in ordered], [inputs[number][1] for number in ordered], alpha, offset, WELD_LAW)
    if answer is None:
        return None
    positions, index, counts, partners = answer
    flat = np.frombuffer(index, dtype=np.uint32).astype(np.int64)
    bounds = np.zeros(len(counts) + 1, dtype=np.int64)
    np.cumsum(np.asarray(counts, dtype=np.int64), out=bounds[1:])
    return NativeWeldV1(np.frombuffer(positions, dtype=np.float64).reshape(-1, 3), tuple(flat[bounds[at] : bounds[at + 1]] for at in range(len(counts))), tuple(partners))


def rebuilt_pieces(inputs: dict, rebuilt, alpha: float, vertices_of: dict, patch_of):
    """`({номер: (длины граней, вершины граней в новой нумерации, владельцы вида int32)}, [швы видов], складки)` перекладываемых доменов нативно либо `None` (откат назван)."""

    numbers = sorted(rebuilt)
    answer = native_answer(
        SPLICE_REBUILT_OPERATION,
        [inputs[number][0] for number in numbers],
        [inputs[number][1] for number in numbers],
        alpha,
        [np.ascontiguousarray(vertices_of[number], dtype=np.int64).tobytes() for number in numbers],
        [int(patch_of(number)) for number in numbers],
    )
    if answer is None:
        return None
    face_counts, sizes, corners, owners, seams, folds = answer
    sizes, corners = np.frombuffer(sizes, dtype=np.int64), np.frombuffer(corners, dtype=np.int64)
    owners = np.frombuffer(owners, dtype=np.int64).astype(np.int32)
    pieces: dict = {}
    face_at = corner_at = 0
    for number, count in zip(numbers, face_counts):
        lengths = sizes[face_at : face_at + count]
        span = int(lengths.sum())
        pieces[number] = (lengths, corners[corner_at : corner_at + span], owners[face_at : face_at + count])
        face_at, corner_at = face_at + count, corner_at + span
    return pieces, [np.frombuffer(seams, dtype=np.int64).reshape(-1, 2)], np.frombuffer(folds, dtype=np.int64).reshape(-1, 2)


__all__ = (
    "NativeWeldV1",
    "SPLICE_REBUILT_OPERATION",
    "SPLICE_WELD_OPERATION",
    "group_inputs",
    "group_weld",
    "native_ordered",
    "rebuilt_pieces",
)
