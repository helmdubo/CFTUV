"""Индекс поверхности и графа патчей: срез по набору патчей без прохода по всей поверхности.

Вид пакета анализа по номерам патчей (`envelope_topology_export.AnalysisBundleIdView`) строился фильтром ПО ВСЕЙ поверхности на каждый
домен: грани, рёбра, вершины, треугольники, кольцо соседей и рёбра графа - O(доменов x поверхность). На `cover.008` (1051 домен) это
22.8 млн вызовов генераторов, 4.8 с родителя до первой задачи пула и столько же на каждом шаге ширины, пока родитель не отдал пулу
ни одного домена. Здесь поверхность и граф индексируются ОДИН раз, а срез домена собирается по индексу: позиции в исходных
кортежах, отсортированные по возрастанию.

ОТВЕТ ТОТ ЖЕ, ПОБИТОВО. Содержимое среза - то же, что давал фильтр, а ПОРЯДОК - порядок исходной поверхности (граней, вершин, рёбер,
треугольников) и исходного графа: он питает дайджесты снапшота, и перестановка была бы молчаливой правкой ответа. Равенство со
старым фильтром закрыто тестом `tests/test_surface_index.py` на каждом домене синтетической поверхности, фикстур выпущенных снапшотов и
записанного полевого слепка `building`.

ЖИЗНЬ ИНДЕКСА - ЖИЗНЬ ОБЪЕКТА. Поверхность неизменяема (замороженные записи в кортежах), поэтому индекс лежит в реестре под
слабой ссылкой на сам объект поверхности (или графа) и уходит вместе с ним: пересборка пакета анализа даёт новый объект и новый индекс,
занятое тождество другому не отдаётся (запись проверяется `is`). Объект, на который слабую ссылку не взять (срез воркера - класс со
`slots`), индексируется на каждый вызов: он мал (грани одного домена и его кольца), и копии реестра ему не нужно. Индекс
сверяется с объектом перед выдачей (те же кортежи по `is`, те же словари графа и их длины): подмену поля или добавление патча в граф
реестр не переживает, индекс строится заново.

Модуль не знает ни Blender, ни ядра: записи поверхности читаются по именам полей (`SourceFace`, `SourceVertex`, ...).
"""

from __future__ import annotations

import weakref
from itertools import chain


class SurfaceIndexV1:
    """Позиции записей поверхности: грани по патчу и по вершине, вершины, рёбра и треугольники по номеру.

    Части строятся при первом обращении (`patch_faces` анализа не платит за треугольники и вершины), каждая - одним проходом.
    """

    __slots__ = (
        "faces",
        "vertices",
        "edges",
        "triangles",
        "ring",
        "face_patch",
        "faces_by_patch",
        "memo",
        "_faces_at_vertex",
        "_vertex_positions",
        "_position_of",
        "_edge_positions",
        "_triangle_positions",
    )

    def __init__(self, surface) -> None:
        self.faces = faces = surface.faces
        self.vertices = surface.vertices
        self.edges = surface.edges
        self.triangles = surface.triangles
        #: Кольцо граней соседей, которое срез воркера уже несёт (`_PatchSurfaceIdView.neighbour_faces`); у полной поверхности его нет.
        self.ring = getattr(surface, "neighbour_faces", ())
        face_patch = []
        faces_by_patch: dict[int, list[int]] = {}
        for position, face in enumerate(faces):
            patch = int(face.patch_id)
            face_patch.append(patch)
            faces_by_patch.setdefault(patch, []).append(position)
        self.face_patch = face_patch
        self.faces_by_patch = faces_by_patch
        #: Производные записи по позиции грани (кольцо соседей: `NeighbourFaceV1`), общие для всех срезов этой поверхности.
        self.memo: dict[int, object] = {}
        self._faces_at_vertex = self._vertex_positions = self._position_of = self._edge_positions = self._triangle_positions = None

    def matches(self, surface) -> bool:
        return (
            surface.faces is self.faces
            and surface.vertices is self.vertices
            and surface.edges is self.edges
            and surface.triangles is self.triangles
            and getattr(surface, "neighbour_faces", ()) is self.ring
        )

    def faces_at_vertex(self) -> dict[int, list[int]]:
        found = self._faces_at_vertex
        if found is None:
            found = {}
            for position, face in enumerate(self.faces):
                for vertex in face.vertex_cycle:
                    found.setdefault(int(vertex), []).append(position)
            self._faces_at_vertex = found
        return found

    def vertex_positions(self) -> dict[int, list[int]]:
        found = self._vertex_positions
        if found is None:
            found, place = {}, {}
            for position, item in enumerate(self.vertices):
                vertex_id = int(item.vertex_id)
                found.setdefault(vertex_id, []).append(position)
                place[vertex_id] = item.position
            self._position_of = place
            self._vertex_positions = found
        return found

    def position_of(self) -> dict[int, object]:
        """Положение вершины по номеру (последняя запись с номером побеждает, как в словаре, который строил фильтр)."""

        self.vertex_positions()
        return self._position_of

    def edge_positions(self) -> dict[int, list[int]]:
        found = self._edge_positions
        if found is None:
            found = {}
            for position, item in enumerate(self.edges):
                found.setdefault(int(item.edge_id), []).append(position)
            self._edge_positions = found
        return found

    def triangle_positions(self) -> dict[int, list[int]]:
        found = self._triangle_positions
        if found is None:
            found = {}
            for position, item in enumerate(self.triangles):
                found.setdefault(int(item.source_face_id), []).append(position)
            self._triangle_positions = found
        return found

    def face_positions(self, included) -> list[int]:
        """Позиции граней патчей `included` по возрастанию (порядок поверхности)."""

        by_patch = self.faces_by_patch
        if len(included) == 1:
            (patch,) = included
            return by_patch.get(patch, [])
        return sorted(chain.from_iterable(by_patch.get(patch, ()) for patch in included))

    def vertex_item_positions(self, vertex_ids) -> list[int]:
        """Позиции вершин поверхности с номерами `vertex_ids` по возрастанию; номера, которых в поверхности нет, пропускаются."""

        found = self.vertex_positions()
        return sorted(chain.from_iterable(found[vertex] for vertex in vertex_ids if vertex in found))

    def edge_item_positions(self, edge_ids) -> list[int]:
        found = self.edge_positions()
        return sorted(chain.from_iterable(found[edge] for edge in edge_ids if edge in found))

    def triangle_item_positions(self, face_ids) -> list[int]:
        found = self.triangle_positions()
        return sorted(chain.from_iterable(found[face] for face in face_ids if face in found))

    def ring_positions(self, vertex_ids, included) -> list[int]:
        """Позиции граней ВНЕ `included`, касающихся хотя бы одной вершины из `vertex_ids`, по возрастанию."""

        at_vertex = self.faces_at_vertex()
        touching: set[int] = set()
        for vertex in vertex_ids:
            found = at_vertex.get(vertex)
            if found:
                touching.update(found)
        face_patch = self.face_patch
        return sorted(position for position in touching if face_patch[position] not in included)


class GraphIndexV1:
    """Порядок узлов и рёбер графа патчей: срез по набору патчей без прохода по всему графу."""

    __slots__ = (
        "nodes",
        "edges",
        "node_count",
        "edge_count",
        "node_keys",
        "node_position",
        "node_ids",
        "edge_keys",
        "edge_ends",
        "edges_at_patch",
    )

    def __init__(self, graph) -> None:
        self.nodes = nodes = graph.nodes
        self.edges = edges = graph.edges
        self.node_count = len(nodes)
        self.edge_count = len(edges)
        self.node_keys = tuple(nodes)
        self.node_position = {int(key): position for position, key in enumerate(self.node_keys)}
        self.node_ids = frozenset(self.node_position)
        self.edge_keys = tuple(edges)
        ends = []
        at_patch: dict[int, list[int]] = {}
        for position, key in enumerate(self.edge_keys):
            seam = edges[key]
            first, second = int(seam.patch_a_id), int(seam.patch_b_id)
            ends.append((first, second))
            at_patch.setdefault(first, []).append(position)
            if second != first:
                at_patch.setdefault(second, []).append(position)
        self.edge_ends = ends
        self.edges_at_patch = at_patch

    def matches(self, graph) -> bool:
        return (
            graph.nodes is self.nodes
            and graph.edges is self.edges
            and len(self.nodes) == self.node_count
            and len(self.edges) == self.edge_count
        )

    def nodes_of(self, graph, included) -> dict:
        """Узлы патчей `included` в порядке графа, по номеру патча; номера обязаны входить в `node_ids`."""

        position, keys, table = self.node_position, self.node_keys, graph.nodes
        return {patch: table[keys[position[patch]]] for patch in sorted(included, key=position.__getitem__)}

    def edges_of(self, graph, included) -> dict:
        """Рёбра графа, оба конца которых лежат в `included`, в порядке графа, по ключу ребра."""

        candidates: set[int] = set()
        for patch in included:
            found = self.edges_at_patch.get(patch)
            if found:
                candidates.update(found)
        ends, keys, table = self.edge_ends, self.edge_keys, graph.edges
        chosen = sorted(
            position for position in candidates if ends[position][0] in included and ends[position][1] in included
        )
        return {keys[position]: table[keys[position]] for position in chosen}


#: Реестр индексов по `id` объекта: запись - `(слабая ссылка на объект, индекс)`; уходит вместе с объектом.
_INDEXES: dict[int, tuple] = {}


def _forgetter(key: int):
    def forget(_dead) -> None:
        _INDEXES.pop(key, None)

    return forget


def _indexed(owner, build, current):
    key = id(owner)
    entry = _INDEXES.get(key)
    if entry is not None and entry[0]() is owner and current(entry[1], owner):
        return entry[1]
    index = build(owner)
    try:
        reference = weakref.ref(owner, _forgetter(key))
    except TypeError:
        return index
    _INDEXES[key] = (reference, index)
    return index


def surface_index_of(surface) -> SurfaceIndexV1:
    """Индекс поверхности: из реестра, пока жив её объект, либо построенный заново."""

    return _indexed(surface, SurfaceIndexV1, SurfaceIndexV1.matches)


def graph_index_of(graph) -> GraphIndexV1:
    """Индекс графа патчей: из реестра, пока жив его объект и словари узлов и рёбер те же, либо построенный заново."""

    return _indexed(graph, GraphIndexV1, GraphIndexV1.matches)


def patch_faces_of(surface, patch_id: int) -> tuple:
    """Грани патча в порядке поверхности: то же, что давал фильтр `face.patch_id == int(patch_id)` по всем граням."""

    index = surface_index_of(surface)
    faces = index.faces
    return tuple(faces[position] for position in index.faces_by_patch.get(int(patch_id), ()))


__all__ = (
    "GraphIndexV1",
    "SurfaceIndexV1",
    "graph_index_of",
    "patch_faces_of",
    "surface_index_of",
)
