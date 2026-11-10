"""Разрез отказанного патча Both на куски (BOTH_PATCH_SPLIT_V1). Без Blender: чистый Python.

ЗАЧЕМ. Ядро считает поле домена на ОДНОЙ развёртке всего патча. Шумный верх стены с узкими щелями бевела (полосы бевела шумовых складок
входят в верх и кончаются в нём) не разворачивается в одну карту: почти изометрическая карта накрывает себя в щелях
(`DEVELOPABLE_CHART_SELF_OVERLAP`), а карта без накрытия выходит за допуск растяжения (`DEVELOPABLE_STRETCH_BUDGET_EXCEEDED`). Это
свойство одной карты, а не поверхности: куски такого патча разворачиваются. Поэтому ХОСТ режет патч по рёбрам меша на куски, и каждый
кусок - свой патч и свой домен (ядро не меняется: ключ `(DecalRequestId, PatchDomainId)`, шов - факт хоста, как любой шов меша).

ЗАКОН РАЗРЕЗА (`patch_cut`): одна хорда за раунд - путь по ВНУТРЕННИМ рёбрам патча между двумя вершинами его границы, внутренние вершины
пути не на границе, разрез делит патч ровно на две части. Из всех хорд (кратчайший путь по длине рёбер между каждой парой вершин
границы) берётся та, у которой `длина / sqrt(меньшая площадь)` наименьшая: короткий разрез, отрезающий не мелочь, - узкая шейка патча
(у верха стены с щелями - от дна щели к другому рельсу), а у ровной полосы - поперёк посередине. Ничья - по длине, затем по номерам
вершин: разрез - функция меша и граней патча, повторное нажатие и живая ширина видят те же куски. Хорды нет (патч без двух разделимых
вершин границы) - `BOTH_PATCH_SPLIT_NO_CUT`, отказ куска остаётся как есть.

Рёбра разреза - швы анализа (как разрез «узел | полоса» Inward) и НЕ источники: источники - рёбра, которые оставляет выделение.
"""

from __future__ import annotations

import heapq
import math
import re
from collections.abc import Mapping, Sequence

#: Отказы внешней части Both, после которых патч режется: одна карта патча не годится, куски - могут.
SPLIT_TRIGGER_OUTCOMES = frozenset({"DEVELOPABLE_CHART_SELF_OVERLAP", "DEVELOPABLE_STRETCH_BUDGET_EXCEEDED"})
#: Раундов разреза на нажатие не больше: каждый раунд режет каждый ещё отказанный кусок одной хордой (до 2^N кусков у патча).
SPLIT_MAX_ROUNDS = 4

BOTH_PATCH_SPLIT = "BOTH_PATCH_SPLIT"
BOTH_PATCH_SPLIT_NO_CUT = "BOTH_PATCH_SPLIT_NO_CUT"
BOTH_PATCH_SPLIT_PIECE_REFUSED = "BOTH_PATCH_SPLIT_PIECE_REFUSED"


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _area(coords, cycle) -> float:
    """Площадь грани по Ньюэллу (половина длины суммы векторных произведений)."""

    x = y = z = 0.0
    for index, vertex in enumerate(cycle):
        a, b = coords[vertex], coords[cycle[(index + 1) % len(cycle)]]
        x += (a[1] - b[1]) * (a[2] + b[2])
        y += (a[2] - b[2]) * (a[0] + b[0])
        z += (a[0] - b[0]) * (a[1] + b[1])
    return 0.5 * math.sqrt(x * x + y * y + z * z)


def _edge_faces(faces: Mapping[int, Sequence[int]]) -> dict[tuple[int, int], list[int]]:
    table: dict[tuple[int, int], list[int]] = {}
    for face_id in sorted(faces):
        cycle = faces[face_id]
        for index, vertex in enumerate(cycle):
            other = cycle[(index + 1) % len(cycle)]
            table.setdefault((min(vertex, other), max(vertex, other)), []).append(face_id)
    return table


def _shortest_chords(start, boundary, adjacency):
    """Кратчайшие пути от вершины границы `start` до других вершин границы, не проходящие через границу: `{b: (длина, путь)}`."""

    best = {start: (0.0, (start,))}
    heap = [(0.0, (start,))]
    found = {}
    while heap:
        length, path = heapq.heappop(heap)
        vertex = path[-1]
        if best.get(vertex, (math.inf,))[0] < length or (vertex != start and vertex in boundary):
            continue
        for other, step in adjacency.get(vertex, ()):
            if other in path:
                continue
            item = (length + step, (*path, other))
            if other in boundary:
                if other != start and (other not in found or item < found[other]):
                    found[other] = item
                continue
            if other not in best or item < best[other]:
                best[other] = item
                heapq.heappush(heap, item)
    return found


def _parts(faces, table, cut) -> list[set[int]]:
    """Части патча, связные по внутренним рёбрам вне разреза."""

    neighbours: dict[int, list[int]] = {face_id: [] for face_id in faces}
    for key, owners in table.items():
        if len(owners) == 2 and key not in cut:
            neighbours[owners[0]].append(owners[1])
            neighbours[owners[1]].append(owners[0])
    seen: set[int] = set()
    parts = []
    for face_id in sorted(faces):
        if face_id in seen:
            continue
        part, stack = set(), [face_id]
        while stack:
            item = stack.pop()
            if item in part:
                continue
            part.add(item)
            stack.extend(neighbours[item])
        seen |= part
        parts.append(part)
    return parts


def overlap_pair(detail: str) -> tuple[tuple[int, int], tuple[int, int]] | None:
    """Первая пара граничных рёбер, которые карта накрыла (`DEVELOPABLE_CHART_SELF_OVERLAP`: `first: a->b vs c->d`), номерами вершин меша
    (хвост имени вершины хоста `host-vertex:...:<номер>`), либо `None` - в тексте отказа пары нет."""

    found = _OVERLAP_PAIR.search(detail or "")
    if found is None:
        return None
    a, b, c, d = (int(item) for item in found.groups())
    return (min(a, b), max(a, b)), (min(c, d), max(c, d))


#: Имя объекта в имени вершины может нести пробелы и двоеточия: номер - цифры перед `->`, ` vs ` или `)`.
_OVERLAP_PAIR = re.compile(r"first: host-vertex:.*?:(\d+)->host-vertex:.*?:(\d+) vs host-vertex:.*?:(\d+)->host-vertex:.*?:(\d+)\)")


def _separates(parts, table, apart) -> bool:
    """Оба ребра пары `apart` - рёбра патча, и разрез кладёт их в разные части (нет пары либо ребра нет в патче - не мешает)."""

    if apart is None or any(len(table.get(edge, ())) != 1 for edge in apart):
        return True
    owners = [table[edge][0] for edge in apart]
    return not any(owners[0] in part and owners[1] in part for part in parts)


def patch_cut(faces: Mapping[int, Sequence[int]], coords, apart=None) -> tuple[tuple[int, int], ...]:
    """Рёбра (пары вершин по возрастанию) хорды, режущей патч `faces` надвое по закону модуля, либо `()` - хорды нет.

    `apart` - пара граничных рёбер (`overlap_pair`), которые хорда обязана развести по разным частям: накрытие карты - между ними, и
    разрез, оставивший их в одном куске, этот отказ не снял бы. Хорды, разводящей их, нет - закон без пары.
    """

    table = _edge_faces(faces)
    if apart is not None and any(len(table.get(edge, ())) != 1 for edge in apart):
        apart = None
    boundary = {vertex for key, owners in table.items() if len(owners) == 1 for vertex in key}
    adjacency: dict[int, list[tuple[int, float]]] = {}
    for (a, b), owners in table.items():
        if len(owners) == 2:
            step = math.dist(coords[a], coords[b])
            adjacency.setdefault(a, []).append((b, step))
            adjacency.setdefault(b, []).append((a, step))
    for items in adjacency.values():
        items.sort()
    areas = {face_id: _area(coords, faces[face_id]) for face_id in faces}
    best = None
    for start in sorted(boundary):
        for end, (length, path) in sorted(_shortest_chords(start, boundary, adjacency).items()):
            if end < start:
                continue
            cut = frozenset((min(a, b), max(a, b)) for a, b in zip(path, path[1:]))
            parts = _parts(faces, table, cut)
            if len(parts) != 2 or not _separates(parts, table, apart):
                continue
            smaller = min(sum(areas[item] for item in part) for part in parts)
            if smaller <= 0.0:
                continue
            key = (length / math.sqrt(smaller), length, path)
            if best is None or key < best[0]:
                best = (key, cut)
    if best is None:
        return () if apart is None else patch_cut(faces, coords)
    return tuple(sorted(best[1]))


def split_summary(patches: int, pieces: int, rounds: int, refused: Mapping[str, int], no_cut: int) -> str:
    """Хвост строки статуса: сколько патчей разрезано, на сколько кусков, за сколько раундов, отказы кусков по имени и патчи без хорды."""

    text = f"{BOTH_PATCH_SPLIT} {patches} -> {pieces} pieces ({rounds} rounds)"
    if refused:
        text += f", {BOTH_PATCH_SPLIT_PIECE_REFUSED} " + ", ".join(f"{name} {count}" for name, count in sorted(refused.items()))
    return text + (f", {BOTH_PATCH_SPLIT_NO_CUT} {no_cut}" if no_cut else "")


__all__ = (
    "BOTH_PATCH_SPLIT",
    "BOTH_PATCH_SPLIT_NO_CUT",
    "BOTH_PATCH_SPLIT_PIECE_REFUSED",
    "SPLIT_MAX_ROUNDS",
    "SPLIT_TRIGGER_OUTCOMES",
    "overlap_pair",
    "patch_cut",
    "split_summary",
)
