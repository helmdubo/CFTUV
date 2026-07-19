"""Compile-static rail topology для surface-native decal runtime.

R0 намеренно не материализует геометрию. Модуль читает только сериализованный
PatchGraph, строит детерминированные edge-loop маршруты и отдаёт immutable IR
для preview. Ни Blender API, ни Voronoi/chart backend здесь не участвуют.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, replace
from enum import Enum
from math import sqrt

from .constants import DECAL_COPLANAR_DOT


class RailStationKind(str, Enum):
    """Способ привязки станции к source topology."""

    VERTEX = "VERTEX"
    EDGE = "EDGE"


class RailTermination(str, Enum):
    """Каноническая причина завершения route."""

    ALPHA = "ALPHA"
    DAM = "DAM"
    PCHAIN = "PCHAIN"
    POLE = "POLE"
    MERGE = "MERGE"


class RailEventKind(str, Enum):
    """События, видимые в rail preview и regression-отчёте."""

    DAM = "DAM"
    POLE = "POLE"
    MERGE = "MERGE"
    MARK = "MARK"
    PCHAIN = "PCHAIN"
    ALPHA = "ALPHA"


class RailStartSectorKind(str, Enum):
    """RR1b-классификация веерного сектора в routing IR."""

    ROUTE = "ROUTE"
    DIRECT = "DIRECT"
    DAM = "DAM"
    CORNER = "CORNER"


@dataclass(frozen=True, order=True)
class RailSideKey:
    """Топологическая сторона spine-вершины без геометрического выбора."""

    spine_vertex_id: int
    start_edge_id: int
    source_face_ids: tuple[int, ...] = ()


@dataclass(frozen=True, order=True)
class RailRouteKey:
    """Стабильный ключ route, независимый от порядка перечисления."""

    side: RailSideKey


@dataclass(frozen=True)
class RailSourceVertex:
    vertex_id: int
    position: tuple[float, float, float]


@dataclass(frozen=True)
class RailSourceEdge:
    edge_id: int
    vertex_ids: tuple[int, int]
    face_indices: tuple[int, ...]
    length: float
    is_pchain: bool = False
    is_spine: bool = False
    is_marked: bool = False
    is_fold: bool = False


@dataclass(frozen=True)
class RailSourceFace:
    face_id: int
    vertex_ids: tuple[int, ...]
    edge_ids: tuple[int, ...]
    normal: tuple[float, float, float]
    planarity_min_dot: float


@dataclass(frozen=True)
class RailChainUse:
    chain_ref: tuple[int, int, int]
    oriented_vertex_ids: tuple[int, int]
    chain_edge_index: int
    chain_is_closed: bool = False


@dataclass(frozen=True)
class RailSpineUse:
    edge_id: int
    vertex_ids: tuple[int, int]
    chain_uses: tuple[RailChainUse, ...] = ()


@dataclass(frozen=True)
class RailStation:
    station_index: int
    distance: float
    kind: RailStationKind
    source_vertex_id: int | None = None
    source_edge_id: int | None = None
    edge_parameter: float | None = None


@dataclass(frozen=True)
class RailRouteSegment:
    """Полная provenance одного station interval."""

    edge_id: int
    from_station_index: int
    to_station_index: int
    source_face_ids: tuple[int, ...]


@dataclass(frozen=True)
class RailRoute:
    route_id: int
    key: RailRouteKey
    stations: tuple[RailStation, ...]
    segments: tuple[RailRouteSegment, ...]
    termination: RailTermination


@dataclass(frozen=True)
class RailEvent:
    kind: RailEventKind
    route_id: int
    station_index: int


@dataclass(frozen=True)
class RailStartSector:
    """Топологический сектор между spine/boundary-делимитерами."""

    sector_id: int
    kind: RailStartSectorKind
    spine_vertex_id: int
    delimiter_edge_ids: tuple[int, ...]
    internal_edge_ids: tuple[int, ...]
    source_face_ids: tuple[int, ...]


@dataclass(frozen=True)
class DecalRailPlan:
    """Immutable R0 ledger; evaluator не имеет права читать его до R1."""

    vertices: tuple[RailSourceVertex, ...]
    edges: tuple[RailSourceEdge, ...]
    faces: tuple[RailSourceFace, ...]
    spine_uses: tuple[RailSpineUse, ...]
    routes: tuple[RailRoute, ...]
    events: tuple[RailEvent, ...]
    start_sectors: tuple[RailStartSector, ...]
    alpha_budget: float

    @property
    def ledger(self):
        """Debug layer читает тот же канонический объект, без копии схемы."""

        return self

    @property
    def event_counts(self):
        counts = defaultdict(int)
        for event in self.events:
            counts[event.kind.value] += 1
        return tuple(sorted(counts.items()))


@dataclass(frozen=True)
class RailCompileFailure:
    reason: str
    edge_indices: tuple[int, ...] = ()
    vertex_indices: tuple[int, ...] = ()
    details: tuple[tuple[str, object], ...] = ()


@dataclass(frozen=True)
class RailCompileAttempt:
    plan: DecalRailPlan | None
    failures: tuple[RailCompileFailure, ...] = ()


class _RailCompileError(RuntimeError):
    def __init__(
        self,
        reason,
        *,
        edge_indices=(),
        vertex_indices=(),
        details=(),
    ):
        super().__init__(str(reason))
        self.failure = RailCompileFailure(
            reason=str(reason),
            edge_indices=tuple(sorted(int(index) for index in edge_indices)),
            vertex_indices=tuple(
                sorted(int(index) for index in vertex_indices)
            ),
            details=tuple(details),
        )


@dataclass(frozen=True)
class _RailTopology:
    vertices: tuple[RailSourceVertex, ...]
    edges: tuple[RailSourceEdge, ...]
    faces: tuple[RailSourceFace, ...]
    vertex_by_id: dict
    edge_by_id: dict
    face_by_id: dict
    vertex_edges: dict
    edge_faces: dict
    face_failures: dict


def _point_tuple(value):
    try:
        coords = tuple(float(component) for component in value)
    except (TypeError, ValueError) as exc:
        raise _RailCompileError("INVALID_SOURCE_VERTEX") from exc
    if len(coords) < 3:
        coords = coords + (0.0,) * (3 - len(coords))
    return coords[:3]


def _edge_length(vertex_a, vertex_b):
    delta = tuple(a - b for a, b in zip(vertex_a.position, vertex_b.position))
    return sqrt(sum(component * component for component in delta))


def _triangle_edge_vertices(triangle, opposite_slot):
    """mesh_tri_edge_indices индексируется противоположной вершиной."""

    return (
        int(triangle[(opposite_slot + 1) % 3]),
        int(triangle[(opposite_slot + 2) % 3]),
    )


def _cross3(vector_a, vector_b):
    return (
        vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1],
        vector_a[2] * vector_b[0] - vector_a[0] * vector_b[2],
        vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0],
    )


def _sub3(point_a, point_b):
    return tuple(a - b for a, b in zip(point_a, point_b))


def _dot3(vector_a, vector_b):
    return sum(a * b for a, b in zip(vector_a, vector_b))


def _normalized3(vector):
    length = sqrt(_dot3(vector, vector))
    if length <= 0.0:
        return None
    return tuple(component / length for component in vector)


def _cycle_normal(vertex_cycle, vertex_positions):
    """Newell normal сохраняет winding исходной polygon face."""

    normal = [0.0, 0.0, 0.0]
    for index, vertex_id in enumerate(vertex_cycle):
        point = vertex_positions[vertex_id]
        other = vertex_positions[vertex_cycle[(index + 1) % len(vertex_cycle)]]
        normal[0] += (point[1] - other[1]) * (point[2] + other[2])
        normal[1] += (point[2] - other[2]) * (point[0] + other[0])
        normal[2] += (point[0] - other[0]) * (point[1] + other[1])
    return _normalized3(tuple(normal))


def _canonical_face_cycle(
    face_id,
    edge_ids,
    edge_vertices,
    vertex_positions,
    oriented_normal,
):
    adjacency = defaultdict(list)
    edge_for_pair = {}
    for edge_id in sorted(edge_ids):
        vert_a, vert_b = edge_vertices[edge_id]
        adjacency[vert_a].append(vert_b)
        adjacency[vert_b].append(vert_a)
        edge_for_pair[frozenset((vert_a, vert_b))] = edge_id
    if len(adjacency) < 3 or any(len(neighbors) != 2 for neighbors in adjacency.values()):
        raise _RailCompileError(
            "SOURCE_FACE_CYCLE_INVALID",
            edge_indices=edge_ids,
            vertex_indices=adjacency,
            details=(("face_id", int(face_id)),),
        )

    start = min(adjacency)
    cycles = []
    for first in sorted(adjacency[start]):
        cycle = [start]
        previous = start
        current = first
        while current != start and len(cycle) <= len(adjacency):
            cycle.append(current)
            next_vertices = [
                vertex for vertex in adjacency[current] if vertex != previous
            ]
            if len(next_vertices) != 1:
                break
            previous, current = current, next_vertices[0]
        if current == start and len(cycle) == len(adjacency):
            cycles.append(tuple(cycle))
    if not cycles:
        raise _RailCompileError(
            "SOURCE_FACE_CYCLE_INVALID",
            edge_indices=edge_ids,
            details=(("face_id", int(face_id)),),
        )
    if oriented_normal is None:
        # Provisional winding нужен до определения rail-footprint. Он не
        # легализует грань: именованный отказ хранится отдельно и применяется
        # только если materialization действительно коснётся этой грани.
        vertex_cycle = min(cycles)
    else:
        oriented_cycles = [
            cycle
            for cycle in cycles
            if (
                _cycle_normal(cycle, vertex_positions) is not None
                and _dot3(
                    _cycle_normal(cycle, vertex_positions),
                    oriented_normal,
                ) > 0.0
            )
        ]
        if len(oriented_cycles) != 1:
            raise _RailCompileError(
                "SOURCE_FACE_ORIENTATION_INVALID",
                edge_indices=edge_ids,
                details=(("face_id", int(face_id)),),
            )
        vertex_cycle = oriented_cycles[0]
    ordered_edges = tuple(
        edge_for_pair[
            frozenset((vertex_cycle[index], vertex_cycle[(index + 1) % len(vertex_cycle)]))
        ]
        for index in range(len(vertex_cycle))
    )
    return vertex_cycle, ordered_edges


def _source_chain_records(graph):
    records = defaultdict(list)
    for patch_id in sorted(getattr(graph, "nodes", {})):
        node = graph.nodes[patch_id]
        for loop_index, boundary_loop in enumerate(
            getattr(node, "boundary_loops", ())
        ):
            for chain_index, chain in enumerate(
                getattr(boundary_loop, "chains", ())
            ):
                edge_ids = tuple(
                    int(edge_id)
                    for edge_id in getattr(chain, "edge_indices", ())
                    if int(edge_id) >= 0
                )
                vertex_ids = tuple(
                    int(vertex_id)
                    for vertex_id in getattr(chain, "vert_indices", ())
                )
                for edge_index, edge_id in enumerate(edge_ids):
                    records[edge_id].append(
                        (
                            (int(patch_id), loop_index, chain_index),
                            bool(getattr(chain, "is_closed", False)),
                            edge_index,
                            edge_ids,
                            vertex_ids,
                        )
                    )
    return records


def _oriented_chain_edge(record, endpoints):
    _chain_ref, is_closed, edge_index, edge_ids, vertex_ids = record
    if len(vertex_ids) == len(edge_ids) + 1:
        candidate = (vertex_ids[edge_index], vertex_ids[edge_index + 1])
        if frozenset(candidate) == frozenset(endpoints):
            return candidate, edge_index
    if is_closed and len(vertex_ids) == len(edge_ids):
        candidate = (
            vertex_ids[edge_index],
            vertex_ids[(edge_index + 1) % len(vertex_ids)],
        )
        if frozenset(candidate) == frozenset(endpoints):
            return candidate, edge_index
    candidates = []
    for index in range(max(0, len(vertex_ids) - 1)):
        candidate = (vertex_ids[index], vertex_ids[index + 1])
        if frozenset(candidate) == frozenset(endpoints):
            candidates.append((candidate, index))
    if is_closed and len(vertex_ids) > 1:
        candidate = (vertex_ids[-1], vertex_ids[0])
        if frozenset(candidate) == frozenset(endpoints):
            candidates.append((candidate, len(vertex_ids) - 1))
    unique = tuple(sorted(set(candidates), key=lambda item: item[1]))
    return unique[0] if len(unique) == 1 else None


def _build_source_topology(
    graph,
    selected_edge_ids,
    marked_edge_ids,
):
    if not hasattr(graph, "nodes"):
        raise _RailCompileError("PATCH_GRAPH_REQUIRED")

    vertex_positions = {}
    edge_vertices = {}
    edge_face_ids = defaultdict(set)
    face_edge_ids = defaultdict(set)
    face_triangle_normals = defaultdict(list)
    face_triangle_vertices = defaultdict(set)

    for patch_id in sorted(graph.nodes):
        node = graph.nodes[patch_id]
        local_vertex_ids = tuple(
            int(vertex_id) for vertex_id in getattr(node, "mesh_vert_indices", ())
        )
        local_positions = tuple(getattr(node, "mesh_verts", ()))
        triangles = tuple(getattr(node, "mesh_tris", ()))
        triangle_faces = tuple(getattr(node, "mesh_tri_face_indices", ()))
        triangle_edges = tuple(getattr(node, "mesh_tri_edge_indices", ()))
        if not triangles:
            continue
        if len(local_vertex_ids) != len(local_positions):
            raise _RailCompileError(
                "SOURCE_VERTEX_PROVENANCE_MISSING",
                details=(("patch_id", int(patch_id)),),
            )
        if not (
            len(triangles) == len(triangle_faces) == len(triangle_edges)
        ):
            raise _RailCompileError(
                "SOURCE_TRIANGLE_PROVENANCE_MISSING",
                details=(("patch_id", int(patch_id)),),
            )

        for local_index, vertex_id in enumerate(local_vertex_ids):
            position = _point_tuple(local_positions[local_index])
            existing = vertex_positions.get(vertex_id)
            if existing is not None and existing != position:
                raise _RailCompileError(
                    "SOURCE_VERTEX_POSITION_CONFLICT",
                    vertex_indices=(vertex_id,),
                )
            vertex_positions[vertex_id] = position

        for triangle, face_id, physical_edges in zip(
            triangles,
            triangle_faces,
            triangle_edges,
        ):
            if len(triangle) != 3 or len(physical_edges) != 3:
                raise _RailCompileError(
                    "SOURCE_TRIANGLE_INVALID",
                    details=(("patch_id", int(patch_id)),),
                )
            global_triangle = tuple(local_vertex_ids[int(index)] for index in triangle)
            face_id = int(face_id)
            point_a, point_b, point_c = (
                vertex_positions[vertex_id]
                for vertex_id in global_triangle
            )
            face_triangle_vertices[face_id].update(global_triangle)
            triangle_normal = _normalized3(
                _cross3(
                    _sub3(point_b, point_a),
                    _sub3(point_c, point_a),
                )
            )
            # RV2: нулевая fan-тройка внутри валидного polygon face — всего
            # лишь артефакт pivot-триангуляции. Нормаль грани собирается из
            # остальных треугольников; отказ допустим лишь если вся грань
            # действительно имеет нулевую площадь.
            if triangle_normal is not None:
                face_triangle_normals[face_id].append(
                    (tuple(global_triangle), triangle_normal)
                )
            for opposite_slot, edge_id in enumerate(physical_edges):
                edge_id = int(edge_id)
                if edge_id < 0:
                    continue
                local_a, local_b = _triangle_edge_vertices(
                    global_triangle,
                    opposite_slot,
                )
                endpoints = tuple(sorted((local_a, local_b)))
                existing = edge_vertices.get(edge_id)
                if existing is not None and existing != endpoints:
                    raise _RailCompileError(
                        "SOURCE_EDGE_PROVENANCE_CONFLICT",
                        edge_indices=(edge_id,),
                    )
                edge_vertices[edge_id] = endpoints
                edge_face_ids[edge_id].add(face_id)
                face_edge_ids[face_id].add(edge_id)

    missing_selected = sorted(set(selected_edge_ids).difference(edge_vertices))
    if missing_selected:
        raise _RailCompileError(
            "SELECTED_SPINE_EDGE_MISSING",
            edge_indices=missing_selected,
        )
    missing_marked = sorted(set(marked_edge_ids).difference(edge_vertices))
    if missing_marked:
        raise _RailCompileError(
            "RAIL_MARK_EDGE_MISSING",
            edge_indices=missing_marked,
        )

    faces = []
    face_failures = {}
    for face_id in sorted(face_edge_ids):
        triangle_normals = tuple(
            normal
            for _triangle_key, normal in sorted(
                face_triangle_normals[face_id],
                key=lambda item: item[0],
            )
        )
        try:
            provisional_cycle, provisional_edges = _canonical_face_cycle(
                face_id,
                face_edge_ids[face_id],
                edge_vertices,
                vertex_positions,
                None,
            )
        except _RailCompileError as exc:
            face_failures[face_id] = exc.failure
            provisional_cycle = tuple(
                sorted(
                    {
                        vertex_id
                        for edge_id in face_edge_ids[face_id]
                        for vertex_id in edge_vertices[edge_id]
                    }
                )
            )
            provisional_edges = tuple(sorted(face_edge_ids[face_id]))

        if triangle_normals:
            normal_sum = tuple(
                sum(normal[axis] for normal in triangle_normals)
                for axis in range(3)
            )
            oriented_normal = _normalized3(normal_sum)
            if oriented_normal is None:
                face_failures.setdefault(
                    face_id,
                    RailCompileFailure(
                        reason="SOURCE_FACE_ORIENTATION_INVALID",
                        edge_indices=tuple(sorted(face_edge_ids[face_id])),
                        details=(("face_id", int(face_id)),),
                    ),
                )
                oriented_normal = triangle_normals[0]
        else:
            oriented_normal = None
            face_failures.setdefault(
                face_id,
                RailCompileFailure(
                    reason="SOURCE_TRIANGLE_DEGENERATE",
                    edge_indices=tuple(sorted(face_edge_ids[face_id])),
                    vertex_indices=tuple(
                        sorted(face_triangle_vertices[face_id])
                    ),
                    details=(("face_id", int(face_id)),),
                ),
            )

        if oriented_normal is None:
            vertex_cycle = provisional_cycle
            ordered_edges = provisional_edges
            source_normal = (0.0, 0.0, 0.0)
        else:
            try:
                vertex_cycle, ordered_edges = _canonical_face_cycle(
                    face_id,
                    face_edge_ids[face_id],
                    edge_vertices,
                    vertex_positions,
                    oriented_normal,
                )
            except _RailCompileError as exc:
                face_failures.setdefault(face_id, exc.failure)
                vertex_cycle = provisional_cycle
                ordered_edges = provisional_edges
            source_normal = oriented_normal
        faces.append(
            RailSourceFace(
                face_id=int(face_id),
                vertex_ids=vertex_cycle,
                edge_ids=ordered_edges,
                normal=source_normal,
                planarity_min_dot=(
                    min(
                        _dot3(source_normal, triangle_normal)
                        for triangle_normal in triangle_normals
                    )
                    if triangle_normals
                    else 0.0
                ),
            )
        )

    chain_records = _source_chain_records(graph)
    pchain_edges = set(chain_records)
    face_normal_by_id = {face.face_id: face.normal for face in faces}
    vertex_by_id = {
        vertex_id: RailSourceVertex(vertex_id, vertex_positions[vertex_id])
        for vertex_id in sorted(vertex_positions)
    }
    edges = []
    for edge_id in sorted(edge_vertices):
        endpoints = edge_vertices[edge_id]
        incident_faces = tuple(sorted(edge_face_ids[edge_id]))
        is_fold = (
            len(incident_faces) == 2
            and _dot3(
                face_normal_by_id[incident_faces[0]],
                face_normal_by_id[incident_faces[1]],
            )
            <= DECAL_COPLANAR_DOT
        )
        edges.append(
            RailSourceEdge(
                edge_id=edge_id,
                vertex_ids=endpoints,
                face_indices=incident_faces,
                length=_edge_length(
                    vertex_by_id[endpoints[0]],
                    vertex_by_id[endpoints[1]],
                ),
                is_pchain=edge_id in pchain_edges,
                is_spine=edge_id in selected_edge_ids,
                is_marked=edge_id in marked_edge_ids,
                is_fold=is_fold,
            )
        )
    edge_by_id = {edge.edge_id: edge for edge in edges}
    face_by_id = {face.face_id: face for face in faces}
    vertex_edges = defaultdict(set)
    for edge in edges:
        for vertex_id in edge.vertex_ids:
            vertex_edges[vertex_id].add(edge.edge_id)

    return _RailTopology(
        vertices=tuple(vertex_by_id.values()),
        edges=tuple(edges),
        faces=tuple(faces),
        vertex_by_id=vertex_by_id,
        edge_by_id=edge_by_id,
        face_by_id=face_by_id,
        vertex_edges={
            vertex_id: tuple(sorted(edge_ids))
            for vertex_id, edge_ids in vertex_edges.items()
        },
        edge_faces={
            edge_id: frozenset(face_ids)
            for edge_id, face_ids in edge_face_ids.items()
        },
        face_failures=face_failures,
    )


def _validate_rail_footprint(topology, face_ids):
    """RV1: применяет отложенные source-face отказы только в footprint."""

    for face_id in sorted(set(face_ids)):
        failure = topology.face_failures.get(face_id)
        if failure is None:
            continue
        raise _RailCompileError(
            failure.reason,
            edge_indices=failure.edge_indices,
            vertex_indices=failure.vertex_indices,
            details=failure.details,
        )


def _other_vertex(edge, vertex_id):
    vert_a, vert_b = edge.vertex_ids
    if vertex_id == vert_a:
        return vert_b
    if vertex_id == vert_b:
        return vert_a
    raise _RailCompileError(
        "EDGE_VERTEX_INCIDENCE_INVALID",
        edge_indices=(edge.edge_id,),
        vertex_indices=(vertex_id,),
    )


def _vertex_station(index, distance, vertex_id):
    return RailStation(
        station_index=index,
        distance=float(distance),
        kind=RailStationKind.VERTEX,
        source_vertex_id=int(vertex_id),
    )


def _edge_station(index, distance, edge, from_vertex_id, fraction):
    canonical_fraction = (
        fraction if from_vertex_id == edge.vertex_ids[0] else 1.0 - fraction
    )
    return RailStation(
        station_index=index,
        distance=float(distance),
        kind=RailStationKind.EDGE,
        source_edge_id=edge.edge_id,
        edge_parameter=float(canonical_fraction),
    )


def _start_side_groups(topology, spine_vertex_id, spine_edges):
    """Разделяет RR1-кандидаты по независимым face-sector сторонам."""

    if len(spine_edges) != 2:
        return ()
    faces_a = topology.edge_faces.get(spine_edges[0], frozenset())
    faces_b = topology.edge_faces.get(spine_edges[1], frozenset())
    groups = defaultdict(list)
    for edge_id in topology.vertex_edges.get(spine_vertex_id, ()):
        if edge_id in spine_edges:
            continue
        edge_faces = topology.edge_faces.get(edge_id, frozenset())
        touch_a = edge_faces.intersection(faces_a)
        touch_b = edge_faces.intersection(faces_b)
        if touch_a and touch_b:
            side_faces = tuple(sorted(touch_a.union(touch_b)))
            groups[side_faces].append(edge_id)
    return tuple(
        (side_faces, tuple(sorted(edge_ids)))
        for side_faces, edge_ids in sorted(groups.items())
    )


def _faces_are_one_planar_region(topology, face_ids):
    """RP2: тот же DECAL_COPLANAR_DOT, без rail-local epsilon."""

    face_ids = tuple(sorted(set(face_ids)))
    if len(face_ids) < 2:
        return True
    reference = topology.face_by_id[face_ids[0]].normal
    return all(
        _dot3(reference, topology.face_by_id[face_id].normal)
        > DECAL_COPLANAR_DOT
        for face_id in face_ids[1:]
    )


def _start_sector_groups(
    topology,
    spine_vertex_id,
    spine_edges,
    boundary_edges,
):
    """RR1a: режет веер вершины spine/boundary-делимитерами на сектора."""

    incident_edges = tuple(
        sorted(topology.vertex_edges.get(spine_vertex_id, ()))
    )
    incident_faces = {
        face_id
        for edge_id in incident_edges
        for face_id in topology.edge_faces.get(edge_id, ())
    }
    delimiters = set(spine_edges).union(boundary_edges)
    neighbors = {face_id: set() for face_id in incident_faces}
    for edge_id in incident_edges:
        if edge_id in delimiters:
            continue
        edge_faces = tuple(
            sorted(
                set(topology.edge_faces.get(edge_id, ())).intersection(
                    incident_faces
                )
            )
        )
        for index, face_id in enumerate(edge_faces):
            neighbors[face_id].update(edge_faces[:index])
            neighbors[face_id].update(edge_faces[index + 1 :])

    sectors = []
    pending = set(incident_faces)
    while pending:
        seed = min(pending)
        sector = set()
        frontier = [seed]
        while frontier:
            face_id = frontier.pop()
            if face_id in sector:
                continue
            sector.add(face_id)
            frontier.extend(
                sorted(neighbors[face_id].difference(sector), reverse=True)
            )
        pending.difference_update(sector)
        internal_edges = tuple(
            edge_id
            for edge_id in incident_edges
            if edge_id not in delimiters
            and topology.edge_faces.get(edge_id, frozenset())
            and set(topology.edge_faces[edge_id]).issubset(sector)
        )
        sector_delimiters = tuple(
            edge_id
            for edge_id in sorted(delimiters)
            if set(topology.edge_faces.get(edge_id, ())).intersection(sector)
        )
        sectors.append(
            (tuple(sorted(sector)), internal_edges, sector_delimiters)
        )
    return tuple(sorted(sectors))


def _automatic_continuation(topology, vertex_id, incoming_edge_id):
    incident_edges = topology.vertex_edges.get(vertex_id, ())
    incident_faces = {
        face_id
        for edge_id in incident_edges
        for face_id in topology.edge_faces.get(edge_id, ())
    }
    regular_quad = len(incident_edges) == 4 and all(
        len(topology.face_by_id[face_id].edge_ids) == 4
        for face_id in incident_faces
    )
    if not regular_quad:
        return None
    incoming_faces = topology.edge_faces.get(incoming_edge_id, frozenset())
    candidates = [
        edge_id
        for edge_id in incident_edges
        if edge_id != incoming_edge_id
        and topology.edge_by_id[edge_id].is_fold
        and not topology.edge_faces.get(edge_id, frozenset()).intersection(
            incoming_faces
        )
    ]
    return candidates[0] if len(candidates) == 1 else None


def _chain_continuation(record, incoming_endpoints, vertex_id):
    """Возвращает следующее ребро того же pChain в заданной вершине."""

    oriented = _oriented_chain_edge(record, incoming_endpoints)
    if oriented is None:
        return None
    (from_vertex_id, to_vertex_id), edge_index = oriented
    _chain_ref, is_closed, _record_index, edge_ids, _vertex_ids = record
    if vertex_id == to_vertex_id:
        next_index = edge_index + 1
        if next_index >= len(edge_ids):
            next_index = 0 if is_closed else None
    elif vertex_id == from_vertex_id:
        next_index = edge_index - 1
        if next_index < 0:
            next_index = len(edge_ids) - 1 if is_closed else None
    else:
        return None
    if next_index is None:
        return None
    return edge_ids[next_index]


def _pchain_continuation(topology, chain_records, vertex_id, incoming_edge_id):
    """RR8: boundary-rail продолжает только собственную seam-цепочку."""

    incoming = topology.edge_by_id[incoming_edge_id]
    candidates = {
        continuation
        for record in chain_records.get(incoming_edge_id, ())
        for continuation in (
            _chain_continuation(record, incoming.vertex_ids, vertex_id),
        )
        if continuation is not None
        and continuation != incoming_edge_id
        and continuation in topology.edge_by_id
        and topology.edge_by_id[continuation].is_pchain
    }
    return next(iter(candidates)) if len(candidates) == 1 else None


def _trace_route(
    topology,
    route_id,
    key,
    alpha_budget,
    barrier_vertices,
    spine_vertices,
    chain_records,
):
    start_vertex_id = key.side.spine_vertex_id
    current_edge_id = key.side.start_edge_id
    boundary_route = topology.edge_by_id[current_edge_id].is_pchain
    stations = [_vertex_station(0, 0.0, start_vertex_id)]
    segments = []
    events = []
    distance = 0.0
    current_vertex_id = start_vertex_id
    visited_states = set()
    if topology.edge_by_id[current_edge_id].is_marked:
        events.append(RailEvent(RailEventKind.MARK, route_id, 0))

    def finish(termination, event_kind):
        event = RailEvent(event_kind, route_id, len(stations) - 1)
        return (
            RailRoute(
                route_id,
                key,
                tuple(stations),
                tuple(segments),
                termination,
            ),
            events + [event],
        )

    def record_segment(edge):
        segments.append(
            RailRouteSegment(
                edge_id=edge.edge_id,
                from_station_index=len(stations) - 2,
                to_station_index=len(stations) - 1,
                source_face_ids=edge.face_indices,
            )
        )

    while True:
        edge = topology.edge_by_id[current_edge_id]
        if edge.length <= 0.0:
            return finish(RailTermination.DAM, RailEventKind.DAM)
        remaining = alpha_budget - distance
        if remaining < edge.length:
            distance = alpha_budget
            stations.append(
                _edge_station(
                    len(stations),
                    distance,
                    edge,
                    current_vertex_id,
                    remaining / edge.length,
                )
            )
            record_segment(edge)
            return finish(RailTermination.ALPHA, RailEventKind.ALPHA)

        next_vertex_id = _other_vertex(edge, current_vertex_id)
        distance += edge.length
        stations.append(
            _vertex_station(len(stations), distance, next_vertex_id)
        )
        record_segment(edge)
        if boundary_route and next_vertex_id in spine_vertices:
            return finish(RailTermination.PCHAIN, RailEventKind.PCHAIN)
        if not boundary_route and next_vertex_id in barrier_vertices:
            return finish(RailTermination.PCHAIN, RailEventKind.PCHAIN)
        if distance >= alpha_budget:
            return finish(RailTermination.ALPHA, RailEventKind.ALPHA)

        state = (next_vertex_id, current_edge_id)
        if state in visited_states:
            return finish(RailTermination.DAM, RailEventKind.DAM)
        visited_states.add(state)

        if boundary_route:
            continuation = _pchain_continuation(
                topology,
                chain_records,
                next_vertex_id,
                current_edge_id,
            )
            if continuation is None:
                return finish(RailTermination.PCHAIN, RailEventKind.PCHAIN)
            current_vertex_id = next_vertex_id
            current_edge_id = continuation
            continue

        marked = [
            edge_id
            for edge_id in topology.vertex_edges.get(next_vertex_id, ())
            if edge_id != current_edge_id
            and topology.edge_by_id[edge_id].is_marked
            and topology.edge_by_id[edge_id].is_fold
            and not topology.edge_by_id[edge_id].is_pchain
        ]
        if len(marked) == 1:
            events.append(
                RailEvent(
                    RailEventKind.MARK,
                    route_id,
                    len(stations) - 1,
                )
            )
            current_vertex_id = next_vertex_id
            current_edge_id = marked[0]
            continue
        if len(marked) > 1:
            return finish(RailTermination.DAM, RailEventKind.DAM)
        continuation = _automatic_continuation(
            topology,
            next_vertex_id,
            current_edge_id,
        )
        if continuation is None or topology.edge_by_id[continuation].is_pchain:
            return finish(RailTermination.DAM, RailEventKind.DAM)
        current_vertex_id = next_vertex_id
        current_edge_id = continuation


def _start_dam_route(route_id, side_key):
    station = _vertex_station(0, 0.0, side_key.spine_vertex_id)
    key = RailRouteKey(side_key)
    return (
        RailRoute(route_id, key, (station,), (), RailTermination.DAM),
        [RailEvent(RailEventKind.DAM, route_id, 0)],
    )


def _apply_poles_and_merges(routes, events):
    """Полюс определяется до merge, затем общие хвосты принадлежат одному route."""

    raw_endpoints = defaultdict(list)
    for route in routes:
        if route.termination != RailTermination.DAM or not route.stations:
            continue
        station = route.stations[-1]
        if station.source_vertex_id is not None:
            raw_endpoints[station.source_vertex_id].append(route.route_id)
    pole_vertices = {
        vertex_id
        for vertex_id, route_ids in raw_endpoints.items()
        if len(route_ids) >= 3
    }

    event_by_route_station = {
        (event.route_id, event.station_index): event for event in events
    }
    normalized = []
    occupied_vertices = {}
    for route in routes:
        stations = list(route.stations)
        termination = route.termination
        if (
            stations
            and stations[-1].source_vertex_id in pole_vertices
            and termination == RailTermination.DAM
        ):
            termination = RailTermination.POLE
            event_by_route_station[(route.route_id, len(stations) - 1)] = RailEvent(
                RailEventKind.POLE,
                route.route_id,
                len(stations) - 1,
            )

        merge_index = None
        merge_limit = (
            len(stations) - 1
            if termination == RailTermination.PCHAIN
            else len(stations)
        )
        for station_index, station in enumerate(
            stations[1:merge_limit],
            1,
        ):
            vertex_id = station.source_vertex_id
            if vertex_id is None or vertex_id in pole_vertices:
                continue
            if vertex_id in occupied_vertices:
                merge_index = station_index
                break
        if merge_index is not None:
            stations = stations[: merge_index + 1]
            termination = RailTermination.MERGE
            event_by_route_station[(route.route_id, merge_index)] = RailEvent(
                RailEventKind.MERGE,
                route.route_id,
                merge_index,
            )
            event_by_route_station = {
                key: event
                for key, event in event_by_route_station.items()
                if key[0] != route.route_id or key[1] <= merge_index
            }
        for station_index, station in enumerate(stations):
            if station.source_vertex_id is not None:
                occupied_vertices.setdefault(
                    station.source_vertex_id,
                    (route.route_id, station_index),
                )
        normalized.append(
            replace(
                route,
                stations=tuple(stations),
                segments=route.segments[: max(0, len(stations) - 1)],
                termination=termination,
            )
        )
    normalized_events = tuple(
        event_by_route_station[key]
        for key in sorted(event_by_route_station)
    )
    return tuple(normalized), normalized_events


def compile_decal_rail_plan(
    graph,
    selected_edge_indices,
    *,
    alpha_budget,
    rail_mark_edge_indices=(),
):
    """Компилирует RR1-RR8 один раз; drag получает только готовые станции."""

    alpha_budget = float(alpha_budget)
    if not alpha_budget > 0.0:
        raise _RailCompileError("RAIL_ALPHA_BUDGET_INVALID")
    selected_edge_ids = tuple(
        sorted({int(edge_id) for edge_id in selected_edge_indices or ()})
    )
    marked_edge_ids = tuple(
        sorted({int(edge_id) for edge_id in rail_mark_edge_indices or ()})
    )
    topology = _build_source_topology(
        graph,
        frozenset(selected_edge_ids),
        frozenset(marked_edge_ids),
    )
    chain_records = _source_chain_records(graph)
    if not selected_edge_ids:
        return DecalRailPlan(
            topology.vertices,
            topology.edges,
            topology.faces,
            (),
            (),
            (),
            (),
            alpha_budget,
        )

    selected_by_vertex = defaultdict(list)
    spine_uses = []
    for edge_id in selected_edge_ids:
        edge = topology.edge_by_id[edge_id]
        chain_uses = []
        for record in chain_records.get(edge_id, ()):
            oriented = _oriented_chain_edge(record, edge.vertex_ids)
            if oriented is None:
                continue
            oriented_vertex_ids, chain_edge_index = oriented
            chain_uses.append(
                RailChainUse(
                    chain_ref=record[0],
                    oriented_vertex_ids=oriented_vertex_ids,
                    chain_edge_index=chain_edge_index,
                    chain_is_closed=record[1],
                )
            )
        spine_uses.append(
            RailSpineUse(
                edge_id,
                edge.vertex_ids,
                tuple(sorted(set(chain_uses), key=lambda use: (
                    use.chain_ref,
                    use.chain_edge_index,
                    use.oriented_vertex_ids,
                ))),
            )
        )
        for vertex_id in edge.vertex_ids:
            selected_by_vertex[vertex_id].append(edge_id)
    barrier_vertices = {
        vertex_id
        for edge in topology.edges
        if edge.is_pchain
        for vertex_id in edge.vertex_ids
    }
    spine_vertices = frozenset(selected_by_vertex)
    rr_face_ids = {
        face_id
        for spine_vertex_id in spine_vertices
        for edge_id in topology.vertex_edges.get(spine_vertex_id, ())
        for face_id in topology.edge_faces.get(edge_id, ())
    }

    route_seeds = []
    start_dams = []
    start_sectors = []
    for spine_vertex_id in sorted(selected_by_vertex):
        spine_edges = tuple(sorted(selected_by_vertex[spine_vertex_id]))
        spine_faces = {
            face_id
            for edge_id in spine_edges
            for face_id in topology.edge_faces.get(edge_id, ())
        }
        boundary_faces = set()
        boundary_edges = tuple(
            edge_id
            for edge_id in topology.vertex_edges.get(spine_vertex_id, ())
            if edge_id not in spine_edges
            and topology.edge_by_id[edge_id].is_pchain
        )
        for edge_id in boundary_edges:
            edge_faces = set(topology.edge_by_id[edge_id].face_indices)
            shared_faces = tuple(sorted(edge_faces.intersection(spine_faces)))
            source_faces = shared_faces or tuple(sorted(edge_faces))
            boundary_faces.update(shared_faces)
            route_seeds.append(
                RailRouteKey(
                    RailSideKey(spine_vertex_id, edge_id, source_faces)
                )
            )
        claimed_faces = set(boundary_faces)
        marked_incident = tuple(
            edge_id
            for edge_id in topology.vertex_edges.get(spine_vertex_id, ())
            if edge_id not in spine_edges
            and topology.edge_by_id[edge_id].is_marked
            and not topology.edge_by_id[edge_id].is_pchain
        )
        used_candidates = set(boundary_edges)
        sectorized_start = (
            len(spine_edges) == 2
            or (len(spine_edges) == 1 and bool(boundary_edges))
        )
        if sectorized_start:
            side_groups = _start_sector_groups(
                topology,
                spine_vertex_id,
                spine_edges,
                boundary_edges,
            )
        else:
            side_groups = tuple(
                (side_faces, candidates, ())
                for side_faces, candidates in _start_side_groups(
                    topology,
                    spine_vertex_id,
                    spine_edges,
                )
            )
        for side_faces, candidates, delimiters in side_groups:
            used_candidates.update(candidates)
            source_faces = tuple(
                face_id for face_id in side_faces if face_id not in claimed_faces
            )
            if not source_faces:
                continue
            if _faces_are_one_planar_region(topology, side_faces):
                # RP1: внутренняя triangulation плоскости не рождает ни
                # route, ни station-0 DAM. Геометрию ведёт in-plane path R1.
                claimed_faces.update(source_faces)
                continue
            spine_delimiters = tuple(
                edge_id for edge_id in delimiters if edge_id in spine_edges
            )
            corner_sector = (
                sectorized_start
                and len(spine_delimiters) == 2
                and len(candidates) != 1
            )
            if corner_sector:
                start_sectors.append(
                    RailStartSector(
                        sector_id=len(start_sectors),
                        kind=RailStartSectorKind.CORNER,
                        spine_vertex_id=spine_vertex_id,
                        delimiter_edge_ids=delimiters,
                        internal_edge_ids=candidates,
                        source_face_ids=tuple(sorted(side_faces)),
                    )
                )
                claimed_faces.update(source_faces)
                continue
            # RR1a: spine+boundary сектор без внутреннего ребра уже
            # является прямым RM7-руслом между делимитерами.
            if sectorized_start and not candidates:
                start_sectors.append(
                    RailStartSector(
                        sector_id=len(start_sectors),
                        kind=RailStartSectorKind.DIRECT,
                        spine_vertex_id=spine_vertex_id,
                        delimiter_edge_ids=delimiters,
                        internal_edge_ids=(),
                        source_face_ids=tuple(sorted(side_faces)),
                    )
                )
                claimed_faces.update(source_faces)
                continue
            allowed = tuple(
                edge_id
                for edge_id in candidates
                if not topology.edge_by_id[edge_id].is_pchain
                and topology.edge_by_id[edge_id].is_fold
            )
            marked = tuple(
                edge_id
                for edge_id in allowed
                if topology.edge_by_id[edge_id].is_marked
            )
            chosen = None
            if len(allowed) == 1:
                chosen = allowed[0]
            elif len(marked) == 1:
                chosen = marked[0]
            if chosen is None:
                if sectorized_start:
                    start_sectors.append(
                        RailStartSector(
                            sector_id=len(start_sectors),
                            kind=RailStartSectorKind.DAM,
                            spine_vertex_id=spine_vertex_id,
                            delimiter_edge_ids=delimiters,
                            internal_edge_ids=candidates,
                            source_face_ids=tuple(sorted(side_faces)),
                        )
                    )
                dam_source_faces = tuple(
                    face_id
                    for face_id in source_faces
                    if face_id in spine_faces
                ) or source_faces
                start_dams.append(
                    RailSideKey(spine_vertex_id, -1, dam_source_faces)
                )
            else:
                if sectorized_start:
                    start_sectors.append(
                        RailStartSector(
                            sector_id=len(start_sectors),
                            kind=RailStartSectorKind.ROUTE,
                            spine_vertex_id=spine_vertex_id,
                            delimiter_edge_ids=delimiters,
                            internal_edge_ids=candidates,
                            source_face_ids=tuple(sorted(side_faces)),
                        )
                    )
                route_seeds.append(
                    RailRouteKey(
                        RailSideKey(spine_vertex_id, chosen, source_faces)
                    )
                )
            claimed_faces.update(source_faces)

        # Явная разметка может определить отсутствующий RR1-sector, но не
        # заменяет уже найденную противоположную сторону.
        for edge_id in marked_incident:
            if edge_id in used_candidates:
                continue
            if not topology.edge_by_id[edge_id].is_fold:
                continue
            source_faces = tuple(
                face_id
                for face_id in topology.edge_by_id[edge_id].face_indices
                if face_id not in claimed_faces
            )
            if not source_faces:
                continue
            route_seeds.append(
                RailRouteKey(
                    RailSideKey(
                        spine_vertex_id,
                        edge_id,
                        source_faces,
                    )
                )
            )
            claimed_faces.update(source_faces)
        # RP1: свободный endpoint planar-ленты закрывается аналитической
        # поперечной in-plane границей; topology-only DAM здесь запрещён.

    routes = []
    events = []
    next_route_id = 0
    for key in sorted(set(route_seeds)):
        route, route_events = _trace_route(
            topology,
            next_route_id,
            key,
            alpha_budget,
            barrier_vertices,
            spine_vertices,
            chain_records,
        )
        routes.append(route)
        events.extend(route_events)
        next_route_id += 1
    for side_key in sorted(set(start_dams)):
        route, route_events = _start_dam_route(next_route_id, side_key)
        routes.append(route)
        events.extend(route_events)
        next_route_id += 1

    routes, events = _apply_poles_and_merges(tuple(routes), tuple(events))
    footprint_face_ids = set(rr_face_ids)
    for route in routes:
        footprint_face_ids.update(route.key.side.source_face_ids)
        for segment in route.segments:
            footprint_face_ids.update(segment.source_face_ids)
    _validate_rail_footprint(topology, footprint_face_ids)
    return DecalRailPlan(
        vertices=topology.vertices,
        edges=topology.edges,
        faces=topology.faces,
        spine_uses=tuple(spine_uses),
        routes=routes,
        events=events,
        start_sectors=tuple(start_sectors),
        alpha_budget=alpha_budget,
    )


def compile_decal_rail_attempt(
    graph,
    selected_edge_indices,
    *,
    alpha_budget,
    rail_mark_edge_indices=(),
):
    """Локализует geometry/provenance отказ в compile receipt."""

    try:
        plan = compile_decal_rail_plan(
            graph,
            selected_edge_indices,
            alpha_budget=alpha_budget,
            rail_mark_edge_indices=rail_mark_edge_indices,
        )
    except _RailCompileError as exc:
        return RailCompileAttempt(None, (exc.failure,))
    except Exception as exc:
        # R0 preview не имеет права разрушить прежний materialization path.
        failure = RailCompileFailure(
            reason="RAIL_COMPILE_INTERNAL_ERROR",
            details=(
                ("exception_type", type(exc).__name__),
                ("message", str(exc)),
            ),
        )
        return RailCompileAttempt(None, (failure,))
    return RailCompileAttempt(plan, ())


__all__ = [
    "DecalRailPlan",
    "RailCompileAttempt",
    "RailCompileFailure",
    "RailEvent",
    "RailEventKind",
    "RailRoute",
    "RailRouteKey",
    "RailRouteSegment",
    "RailChainUse",
    "RailSideKey",
    "RailSourceEdge",
    "RailSourceFace",
    "RailSourceVertex",
    "RailSpineUse",
    "RailStartSector",
    "RailStartSectorKind",
    "RailStation",
    "RailStationKind",
    "RailTermination",
    "compile_decal_rail_attempt",
    "compile_decal_rail_plan",
]
