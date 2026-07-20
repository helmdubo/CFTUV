"""Compile-static rail topology для surface-native decal runtime.

R0 намеренно не материализует геометрию. Модуль читает только atomic
AnalysisBundle: chain-топологию из PatchGraph и исходные polygon cycles /
нормали из PatchSurfaceIR. Ни Blender API, ни Voronoi/chart backend здесь
не участвуют.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, replace
from enum import Enum
from math import isfinite, sqrt

from .constants import DECAL_COPLANAR_DOT
from .surface_ir import AnalysisBundle, AnalysisSchemaError


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


class RailCompetitionKind(str, Enum):
    """Структурный источник compile-static freeze-локуса."""

    THREAD_DUAL_READING = "THREAD_DUAL_READING"
    TERMINAL_ROUTE_PAIR = "TERMINAL_ROUTE_PAIR"


class RailStartSectorKind(str, Enum):
    """RR1b-классификация веерного сектора в routing IR."""

    ROUTE = "ROUTE"
    DIRECT = "DIRECT"
    DAM = "DAM"
    CORNER = "CORNER"


class RailTerminalKind(str, Enum):
    """RR9-решение торцевого среза, опубликованное routing IR."""

    ROUTE = "ROUTE"
    SNAP_TIE_DAM = "SNAP_TIE_DAM"
    IN_PLANE_EMPTY = "IN_PLANE_EMPTY"
    IN_PLANE_AMBIGUOUS = "IN_PLANE_AMBIGUOUS"


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


@dataclass(frozen=True, order=True)
class RailRouteReading:
    """RC1: расстояния одного потока на канонических станциях route."""

    route_id: int
    chain_ref: tuple[int, int, int]
    origin_vertex_id: int
    source_face_ids: tuple[int, ...]
    station_distances: tuple[float, ...]


@dataclass(frozen=True, order=True)
class RailFreezeLocus:
    """RC3: compile-static граница встречи двух чтений одной нити."""

    route_id: int
    chain_refs: tuple[tuple[int, int, int], tuple[int, int, int]]
    owner_chain_ref: tuple[int, int, int]
    canonical_distance: float
    kind: RailStationKind
    source_vertex_id: int | None = None
    source_edge_id: int | None = None
    edge_parameter: float | None = None
    route_ids: tuple[int, int] = ()
    arrival_distances: tuple[float, float] = ()
    competition_kind: RailCompetitionKind = (
        RailCompetitionKind.THREAD_DUAL_READING
    )

    def __post_init__(self):
        route_ids = self.route_ids or (self.route_id, self.route_id)
        arrivals = self.arrival_distances or (
            self.canonical_distance,
            self.canonical_distance,
        )
        if len(route_ids) != 2 or len(arrivals) != 2:
            raise ValueError("RAIL_FREEZE_READING_PAIR_REQUIRED")
        arrivals = tuple(float(value) for value in arrivals)
        if any(not isfinite(value) or value < 0.0 for value in arrivals):
            raise ValueError("RAIL_FREEZE_ARRIVAL_INVALID")
        object.__setattr__(
            self,
            "route_ids",
            tuple(int(value) for value in route_ids),
        )
        object.__setattr__(
            self,
            "arrival_distances",
            arrivals,
        )
        object.__setattr__(
            self,
            "competition_kind",
            RailCompetitionKind(self.competition_kind),
        )


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


@dataclass(frozen=True, order=True)
class RailTerminalUse:
    """Единственный источник выбора terminal rail для materialization."""

    spine_vertex_id: int
    spine_edge_id: int
    source_face_ids: tuple[int, ...]
    kind: RailTerminalKind
    candidate_edge_ids: tuple[int, ...] = ()
    route_edge_id: int | None = None
    route_id: int | None = None


@dataclass(frozen=True)
class DecalRailPlan:
    """Immutable rail ledger для compile-static routing и competition."""

    vertices: tuple[RailSourceVertex, ...]
    edges: tuple[RailSourceEdge, ...]
    faces: tuple[RailSourceFace, ...]
    spine_uses: tuple[RailSpineUse, ...]
    routes: tuple[RailRoute, ...]
    route_readings: tuple[RailRouteReading, ...]
    freeze_loci: tuple[RailFreezeLocus, ...]
    events: tuple[RailEvent, ...]
    start_sectors: tuple[RailStartSector, ...]
    terminal_uses: tuple[RailTerminalUse, ...]
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

    @property
    def terminal_fallback_counts(self):
        counts = defaultdict(int)
        for use in self.terminal_uses:
            if use.kind != RailTerminalKind.ROUTE:
                counts[use.kind.value] += 1
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
    face_patch_ids: dict
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


def _sub3(point_a, point_b):
    return tuple(a - b for a, b in zip(point_a, point_b))


def _dot3(vector_a, vector_b):
    return sum(a * b for a, b in zip(vector_a, vector_b))


def _normalized3(vector):
    length = sqrt(_dot3(vector, vector))
    if length <= 0.0:
        return None
    return tuple(component / length for component in vector)


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
    analysis_bundle,
    selected_edge_ids,
    marked_edge_ids,
):
    if not isinstance(analysis_bundle, AnalysisBundle):
        raise AnalysisSchemaError(
            "DECAL_ANALYSIS_SCHEMA_UNSUPPORTED",
            "rail compiler requires AnalysisBundle",
        )
    analysis_bundle.capabilities.require_supported()
    graph = analysis_bundle.patch_graph
    surface = analysis_bundle.patch_surface

    vertex_positions = {
        int(vertex.vertex_id): _point_tuple(vertex.position)
        for vertex in surface.vertices
    }
    edge_vertices = {
        int(edge.edge_id): tuple(sorted(int(value) for value in edge.vertex_ids))
        for edge in surface.edges
    }
    edge_face_ids = {
        int(edge.edge_id): set(int(value) for value in edge.source_face_ids)
        for edge in surface.edges
    }
    face_patch_ids = {
        int(face.face_id): int(face.patch_id) for face in surface.faces
    }
    triangle_by_id = surface.triangle_by_id

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
    for source_face in sorted(surface.faces, key=lambda item: item.face_id):
        face_id = int(source_face.face_id)
        vertex_cycle = tuple(int(value) for value in source_face.vertex_cycle)
        ordered_edges = tuple(int(value) for value in source_face.edge_cycle)
        triangle_normals = tuple(
            _point_tuple(triangle_by_id[triangle_id].triangle_normal)
            for triangle_id in source_face.triangle_ids
            if _normalized3(
                _point_tuple(triangle_by_id[triangle_id].triangle_normal)
            )
            is not None
        )
        source_normal = _normalized3(_point_tuple(source_face.polygon_normal))
        if source_normal is None or not triangle_normals:
            face_failures[face_id] = RailCompileFailure(
                reason="SOURCE_TRIANGLE_DEGENERATE",
                edge_indices=ordered_edges,
                vertex_indices=tuple(sorted(vertex_cycle)),
                details=(("face_id", face_id),),
            )
            source_normal = source_normal or (0.0, 0.0, 0.0)
        faces.append(
            RailSourceFace(
                face_id=face_id,
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
        face_patch_ids=face_patch_ids,
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


def _is_terminal_rail_edge(edge):
    """RR9: pChain, настоящий fold либо структурный mesh border."""

    return edge.is_pchain or edge.is_fold or len(edge.face_indices) == 1


def _terminal_snap_measure(
    topology,
    spine_vertex_id,
    spine_edge_id,
    candidate_edge_id,
):
    """RR9a: точная 3D-мера старого волнового направления, без epsilon."""

    origin = topology.vertex_by_id[spine_vertex_id].position
    spine_edge = topology.edge_by_id[spine_edge_id]
    candidate_edge = topology.edge_by_id[candidate_edge_id]
    spine_direction = _normalized3(
        _sub3(
            topology.vertex_by_id[
                _other_vertex(spine_edge, spine_vertex_id)
            ].position,
            origin,
        )
    )
    candidate_direction = _normalized3(
        _sub3(
            topology.vertex_by_id[
                _other_vertex(candidate_edge, spine_vertex_id)
            ].position,
            origin,
        )
    )
    if spine_direction is None or candidate_direction is None:
        raise _RailCompileError(
            "RAIL_TERMINAL_SNAP_DIRECTION_INVALID",
            edge_indices=(spine_edge_id, candidate_edge_id),
            vertex_indices=(spine_vertex_id,),
        )
    return abs(_dot3(candidate_direction, spine_direction))


def _select_terminal_pchain(
    topology,
    spine_vertex_id,
    spine_edge_id,
    candidate_edge_ids,
):
    """RR9a: mark > уникальный argmin; точная ничья оставляет DAM."""

    candidate_edge_ids = tuple(sorted(candidate_edge_ids))
    marked = tuple(
        edge_id
        for edge_id in candidate_edge_ids
        if topology.edge_by_id[edge_id].is_marked
    )
    if len(marked) == 1:
        return marked[0]
    if len(marked) > 1:
        return None
    measured = tuple(
        (
            _terminal_snap_measure(
                topology,
                spine_vertex_id,
                spine_edge_id,
                edge_id,
            ),
            edge_id,
        )
        for edge_id in candidate_edge_ids
    )
    minimum = min(measure for measure, _edge_id in measured)
    winners = tuple(
        edge_id for measure, edge_id in measured if measure == minimum
    )
    return winners[0] if len(winners) == 1 else None


def _terminal_side_partition(
    topology,
    chain_records,
    spine_vertex_id,
    spine_edge_id,
):
    """RR9b: стороны endpoint, guide-кандидаты и continuation-делимитеры."""

    incident_edges = tuple(topology.vertex_edges.get(spine_vertex_id, ()))
    incident_faces = {
        face_id
        for edge_id in incident_edges
        for face_id in topology.edge_faces.get(edge_id, ())
    }
    spine_faces = set(topology.edge_faces.get(spine_edge_id, ()))
    delimiters = {
        edge_id
        for edge_id in incident_edges
        if edge_id == spine_edge_id
        or _is_terminal_rail_edge(topology.edge_by_id[edge_id])
    }
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

    sector_records = []
    pending = set(spine_faces)
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
        sector_candidate_edge_ids = tuple(
            edge_id
            for edge_id in sorted(delimiters)
            if edge_id != spine_edge_id
            and set(topology.edge_faces.get(edge_id, ())).intersection(sector)
        )
        patch_ids = tuple(
            sorted(
                {
                    topology.face_patch_ids[face_id]
                    for face_id in sector.intersection(spine_faces)
                }
            )
        )
        sector_records.append(
            (
                tuple(sorted(sector)),
                sector_candidate_edge_ids,
                patch_ids,
            )
        )

    patch_side_counts = defaultdict(int)
    for _sector, _candidates, patch_ids in sector_records:
        for patch_id in patch_ids:
            patch_side_counts[patch_id] += 1

    side_patch_keys = tuple(record[2] for record in sector_records)
    patch_keys_are_unique = len(set(side_patch_keys)) == len(side_patch_keys)
    boundary_chain_continuation = (
        _pchain_continuation(
            topology,
            chain_records,
            spine_vertex_id,
            spine_edge_id,
        )
        if len(spine_faces) == 1
        else None
    )
    continuation_edge_ids = set()
    for edge_id in sorted(delimiters.difference({spine_edge_id})):
        edge_faces = set(topology.edge_faces.get(edge_id, ()))
        if (
            edge_id == boundary_chain_continuation
            and len(edge_faces) == 1
        ):
            # RR9b: у одностороннего border-spine нет второй стороны,
            # поэтому продолжение его source-pChain — делимитер, а не гид.
            # Чужое border-ребро и RF21 остаются в обычной иерархии RR9.
            continuation_edge_ids.add(edge_id)
            continue
        if patch_keys_are_unique:
            edge_patch_ids = {
                topology.face_patch_ids[face_id] for face_id in edge_faces
            }
            side_hits = sum(
                bool(edge_patch_ids.intersection(patch_ids))
                for patch_ids in side_patch_keys
            )
        else:
            # SEAM_SELF: patch id общий у двух сторон, поэтому различителем
            # остаются уже построенные face-sector веера.
            side_hits = sum(
                bool(edge_faces.intersection(sector))
                for sector, _candidates, _patch_ids in sector_records
            )
        if side_hits > 1:
            continuation_edge_ids.add(edge_id)

    groups = []
    for sector, sector_candidate_edge_ids, patch_ids in sector_records:
        pchain_edge_ids = tuple(
            edge_id
            for edge_id in sector_candidate_edge_ids
            if edge_id not in continuation_edge_ids
            if topology.edge_by_id[edge_id].is_pchain
            and not topology.edge_by_id[edge_id].is_spine
        )
        # Обычная PATCH-seam имеет разные patch id по сторонам spine: тогда
        # RR9a берёт ВСЕ внешние pChains данного patch, даже если проигравшая
        # цепочка отделена внутри endpoint-веера. Для SEAM_SELF один patch id
        # встречается с обеих сторон; sector остаётся структурным различителем.
        if len(patch_ids) == 1 and patch_side_counts[patch_ids[0]] == 1:
            side_patch_id = patch_ids[0]
            pchain_edge_ids = tuple(
                edge_id
                for edge_id in incident_edges
                if edge_id != spine_edge_id
                and edge_id not in continuation_edge_ids
                and topology.edge_by_id[edge_id].is_pchain
                and not topology.edge_by_id[edge_id].is_spine
                and any(
                    topology.face_patch_ids.get(face_id) == side_patch_id
                    for face_id in topology.edge_faces.get(edge_id, ())
                )
            )
        fallback_edge_ids = tuple(
            edge_id
            for edge_id in sector_candidate_edge_ids
            if edge_id not in continuation_edge_ids
            if not topology.edge_by_id[edge_id].is_pchain
        )
        eligible = tuple(sorted(pchain_edge_ids or fallback_edge_ids))
        marked = tuple(
            edge_id
            for edge_id in eligible
            if topology.edge_by_id[edge_id].is_marked
        )
        chosen_edge_id = None
        if pchain_edge_ids:
            chosen_edge_id = _select_terminal_pchain(
                topology,
                spine_vertex_id,
                spine_edge_id,
                pchain_edge_ids,
            )
        elif len(eligible) == 1:
            chosen_edge_id = eligible[0]
        elif len(marked) == 1:
            # RR9: fold/border сохраняют прежний structural mark-bridge;
            # RR9a-snap к ним не применяется.
            chosen_edge_id = marked[0]
        if chosen_edge_id is not None:
            kind = RailTerminalKind.ROUTE
        elif pchain_edge_ids:
            kind = RailTerminalKind.SNAP_TIE_DAM
        elif eligible:
            kind = RailTerminalKind.IN_PLANE_AMBIGUOUS
        else:
            kind = RailTerminalKind.IN_PLANE_EMPTY
        groups.append(
            (
                tuple(sorted(sector)),
                tuple(sorted(eligible)),
                chosen_edge_id,
                kind,
            )
        )
    return tuple(sorted(groups)), tuple(sorted(continuation_edge_ids))


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


def _terminal_continuation(topology, vertex_id, incoming_edge_id):
    """RR9: terminal rail продолжается только при топологической уникальности."""

    candidates = tuple(
        edge_id
        for edge_id in topology.vertex_edges.get(vertex_id, ())
        if edge_id != incoming_edge_id
        and not topology.edge_by_id[edge_id].is_spine
        and not topology.edge_by_id[edge_id].is_pchain
        and _is_terminal_rail_edge(topology.edge_by_id[edge_id])
    )
    marked = tuple(
        edge_id
        for edge_id in candidates
        if topology.edge_by_id[edge_id].is_marked
    )
    if len(candidates) == 1:
        return candidates[0]
    return marked[0] if len(marked) == 1 else None


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


def _boundary_contour_continuation(
    topology,
    chain_records,
    vertex_id,
    incoming_edge_id,
    visited_edge_ids,
):
    """RR8c: chain — сегмент; station route читает физический контур."""

    visited_edge_ids = frozenset(visited_edge_ids)
    chain_edge_id = _pchain_continuation(
        topology,
        chain_records,
        vertex_id,
        incoming_edge_id,
    )
    if chain_edge_id is not None:
        if chain_edge_id in visited_edge_ids:
            return None, RailTermination.DAM, None
        return chain_edge_id, None, None

    contour_edge_ids = tuple(
        edge_id
        for edge_id in topology.vertex_edges.get(vertex_id, ())
        if edge_id != incoming_edge_id
        and not topology.edge_by_id[edge_id].is_spine
        and (
            topology.edge_by_id[edge_id].is_pchain
            or len(topology.edge_by_id[edge_id].face_indices) == 1
        )
    )
    candidates = tuple(
        edge_id
        for edge_id in contour_edge_ids
        if edge_id not in visited_edge_ids
    )
    if len(candidates) == 1:
        return candidates[0], None, None

    marked = tuple(
        edge_id
        for edge_id in candidates
        if topology.edge_by_id[edge_id].is_marked
    )
    if len(marked) == 1:
        return marked[0], None, RailEventKind.MARK
    if candidates or contour_edge_ids:
        return None, RailTermination.DAM, None
    return None, RailTermination.PCHAIN, None


def _trace_route(
    topology,
    route_id,
    key,
    alpha_budget,
    barrier_vertices,
    spine_vertices,
    chain_records,
    terminal_route_keys=(),
):
    start_vertex_id = key.side.spine_vertex_id
    current_edge_id = key.side.start_edge_id
    boundary_route = topology.edge_by_id[current_edge_id].is_pchain
    terminal_route = (
        key.side.spine_vertex_id,
        key.side.start_edge_id,
    ) in terminal_route_keys
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
            continuation, stop_reason, continuation_event = (
                _boundary_contour_continuation(
                    topology,
                    chain_records,
                    next_vertex_id,
                    current_edge_id,
                    (segment.edge_id for segment in segments),
                )
            )
            if continuation_event is not None:
                events.append(
                    RailEvent(
                        continuation_event,
                        route_id,
                        len(stations) - 1,
                    )
                )
            if continuation is None:
                if stop_reason == RailTermination.DAM:
                    return finish(RailTermination.DAM, RailEventKind.DAM)
                return finish(RailTermination.PCHAIN, RailEventKind.PCHAIN)
            current_vertex_id = next_vertex_id
            current_edge_id = continuation
            continue

        if terminal_route:
            continuation = _terminal_continuation(
                topology,
                next_vertex_id,
                current_edge_id,
            )
            if continuation is None:
                return finish(RailTermination.DAM, RailEventKind.DAM)
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


def _spine_chain_refs_by_vertex(spine_uses):
    refs_by_vertex = defaultdict(set)
    for spine_use in spine_uses:
        for vertex_id in spine_use.vertex_ids:
            refs_by_vertex[vertex_id].update(
                chain_use.chain_ref for chain_use in spine_use.chain_uses
            )
    return {
        vertex_id: tuple(sorted(chain_refs))
        for vertex_id, chain_refs in refs_by_vertex.items()
    }


def _thread_chain_pair(route, topology, chain_refs_by_vertex):
    """RR10(e): распознаёт уже трассированный route между двумя chains."""

    if route.termination != RailTermination.PCHAIN or not route.segments:
        return None
    if any(
        topology.edge_by_id[segment.edge_id].is_pchain
        for segment in route.segments
    ):
        return None
    start_station = route.stations[0]
    end_station = route.stations[-1]
    if (
        start_station.source_vertex_id is None
        or end_station.source_vertex_id is None
    ):
        return None
    patch_ids = {
        topology.face_patch_ids[face_id]
        for face_id in route.key.side.source_face_ids
        if face_id in topology.face_patch_ids
    }
    if len(patch_ids) != 1:
        return None
    patch_id = next(iter(patch_ids))
    start_refs = tuple(
        chain_ref
        for chain_ref in chain_refs_by_vertex.get(
            start_station.source_vertex_id,
            (),
        )
        if chain_ref[0] == patch_id
    )
    end_refs = tuple(
        chain_ref
        for chain_ref in chain_refs_by_vertex.get(
            end_station.source_vertex_id,
            (),
        )
        if chain_ref[0] == patch_id
    )
    if len(start_refs) != 1 or len(end_refs) != 1:
        return None
    if start_refs[0] == end_refs[0]:
        return None
    return start_refs[0], end_refs[0]


def _thread_physical_key(route, chain_pair):
    edge_ids = tuple(segment.edge_id for segment in route.segments)
    reverse_edge_ids = tuple(reversed(edge_ids))
    return tuple(sorted(chain_pair)), min(edge_ids, reverse_edge_ids)


def _canonicalize_thread_routes(
    routes,
    events,
    topology,
    spine_uses,
):
    """RR10(c): удаляет обратное второе представление одной нити."""

    chain_refs_by_vertex = _spine_chain_refs_by_vertex(spine_uses)
    groups = defaultdict(list)
    for route in routes:
        chain_pair = _thread_chain_pair(
            route,
            topology,
            chain_refs_by_vertex,
        )
        if chain_pair is None:
            continue
        groups[_thread_physical_key(route, chain_pair)].append(route)

    owner_by_route_id = {route.route_id: route.route_id for route in routes}
    for grouped_routes in groups.values():
        if len(grouped_routes) < 2:
            continue
        owner = min(grouped_routes, key=lambda route: route.key)
        owner_edge_ids = tuple(segment.edge_id for segment in owner.segments)
        for route in grouped_routes:
            edge_ids = tuple(segment.edge_id for segment in route.segments)
            if edge_ids not in (owner_edge_ids, tuple(reversed(owner_edge_ids))):
                continue
            owner_by_route_id[route.route_id] = owner.route_id

    kept_routes = tuple(
        route
        for route in routes
        if owner_by_route_id[route.route_id] == route.route_id
    )
    dense_id_by_old = {
        route.route_id: dense_id
        for dense_id, route in enumerate(kept_routes)
    }
    dense_id_by_old.update(
        {
            route_id: dense_id_by_old[owner_id]
            for route_id, owner_id in owner_by_route_id.items()
            if owner_id in dense_id_by_old
        }
    )
    canonical_routes = tuple(
        replace(route, route_id=dense_id_by_old[route.route_id])
        for route in kept_routes
    )
    canonical_events = tuple(
        replace(event, route_id=dense_id_by_old[event.route_id])
        for event in events
        if owner_by_route_id[event.route_id] == event.route_id
    )
    route_start_aliases = {
        (route.key.side.spine_vertex_id, route.key.side.start_edge_id): (
            dense_id_by_old[route.route_id]
        )
        for route in routes
    }
    return canonical_routes, canonical_events, route_start_aliases


def _freeze_locus_for_route(route, chain_pair, topology):
    total_distance = route.stations[-1].distance
    canonical_distance = total_distance / 2.0
    for station in route.stations:
        if station.distance == canonical_distance:
            return RailFreezeLocus(
                route_id=route.route_id,
                chain_refs=tuple(sorted(chain_pair)),
                owner_chain_ref=min(chain_pair),
                canonical_distance=canonical_distance,
                kind=station.kind,
                source_vertex_id=station.source_vertex_id,
                source_edge_id=station.source_edge_id,
                edge_parameter=station.edge_parameter,
                route_ids=(route.route_id, route.route_id),
                arrival_distances=(
                    canonical_distance,
                    canonical_distance,
                ),
            )
    for segment in route.segments:
        start = route.stations[segment.from_station_index]
        end = route.stations[segment.to_station_index]
        if not start.distance < canonical_distance < end.distance:
            continue
        edge = topology.edge_by_id[segment.edge_id]
        if start.source_vertex_id is None:
            raise _RailCompileError(
                "RAIL_FREEZE_SEGMENT_START_INVALID",
                edge_indices=(segment.edge_id,),
            )
        fraction = (
            (canonical_distance - start.distance)
            / (end.distance - start.distance)
        )
        edge_parameter = (
            fraction
            if start.source_vertex_id == edge.vertex_ids[0]
            else 1.0 - fraction
        )
        return RailFreezeLocus(
            route_id=route.route_id,
            chain_refs=tuple(sorted(chain_pair)),
            owner_chain_ref=min(chain_pair),
            canonical_distance=canonical_distance,
            kind=RailStationKind.EDGE,
            source_edge_id=segment.edge_id,
            edge_parameter=float(edge_parameter),
            route_ids=(route.route_id, route.route_id),
            arrival_distances=(canonical_distance, canonical_distance),
        )
    raise _RailCompileError(
        "RAIL_FREEZE_LOCUS_UNRESOLVED",
        edge_indices=tuple(segment.edge_id for segment in route.segments),
    )


def _station_edge_parameter(station, edge):
    """Точный параметр station на физическом source edge."""

    if (
        station.source_edge_id == edge.edge_id
        and station.edge_parameter is not None
    ):
        return float(station.edge_parameter)
    if station.source_vertex_id == edge.vertex_ids[0]:
        return 0.0
    if station.source_vertex_id == edge.vertex_ids[1]:
        return 1.0
    return None


def _route_origin_chain_ref(route, topology, chain_refs_by_vertex):
    """RC1 tie-break authority одного route без геометрической эвристики."""

    if not route.stations or route.stations[0].source_vertex_id is None:
        return None
    patch_ids = {
        topology.face_patch_ids[face_id]
        for face_id in route.key.side.source_face_ids
        if face_id in topology.face_patch_ids
    }
    if len(patch_ids) != 1:
        return None
    patch_id = next(iter(patch_ids))
    refs = tuple(
        chain_ref
        for chain_ref in chain_refs_by_vertex.get(
            route.stations[0].source_vertex_id,
            (),
        )
        if chain_ref[0] == patch_id
    )
    return refs[0] if len(refs) == 1 else None


def _route_pair_freeze_locus(
    route_a,
    route_b,
    topology,
    chain_refs_by_vertex,
):
    """RC1: точная встреча двух terminal routes на общем source edge.

    Функция вызывается только на compile. Runtime получает готовые station
    distances и source-key локуса; повторно решать равенство запрещено RC3.
    """

    stations_a = {
        station.station_index: station for station in route_a.stations
    }
    stations_b = {
        station.station_index: station for station in route_b.stations
    }
    segments_b = defaultdict(list)
    for segment in route_b.segments:
        segments_b[segment.edge_id].append(segment)
    candidates = []
    unresolved_equal_intervals = []
    for segment_a in route_a.segments:
        edge = topology.edge_by_id.get(segment_a.edge_id)
        if edge is None:
            continue
        station_a0 = stations_a[segment_a.from_station_index]
        station_a1 = stations_a[segment_a.to_station_index]
        parameter_a0 = _station_edge_parameter(station_a0, edge)
        parameter_a1 = _station_edge_parameter(station_a1, edge)
        if (
            parameter_a0 is None
            or parameter_a1 is None
            or parameter_a0 == parameter_a1
        ):
            continue
        for segment_b in segments_b.get(segment_a.edge_id, ()):
            station_b0 = stations_b[segment_b.from_station_index]
            station_b1 = stations_b[segment_b.to_station_index]
            parameter_b0 = _station_edge_parameter(station_b0, edge)
            parameter_b1 = _station_edge_parameter(station_b1, edge)
            if (
                parameter_b0 is None
                or parameter_b1 is None
                or parameter_b0 == parameter_b1
            ):
                continue
            if (
                (parameter_a1 - parameter_a0)
                * (parameter_b1 - parameter_b0)
                >= 0.0
            ):
                continue
            low = max(
                min(parameter_a0, parameter_a1),
                min(parameter_b0, parameter_b1),
            )
            high = min(
                max(parameter_a0, parameter_a1),
                max(parameter_b0, parameter_b1),
            )
            if low > high:
                continue

            def arrival(station_0, station_1, parameter_0, parameter_1, value):
                factor = (value - parameter_0) / (
                    parameter_1 - parameter_0
                )
                return float(station_0.distance) + factor * float(
                    station_1.distance - station_0.distance
                )

            difference_low = (
                arrival(
                    station_a0,
                    station_a1,
                    parameter_a0,
                    parameter_a1,
                    low,
                )
                - arrival(
                    station_b0,
                    station_b1,
                    parameter_b0,
                    parameter_b1,
                    low,
                )
            )
            difference_high = (
                arrival(
                    station_a0,
                    station_a1,
                    parameter_a0,
                    parameter_a1,
                    high,
                )
                - arrival(
                    station_b0,
                    station_b1,
                    parameter_b0,
                    parameter_b1,
                    high,
                )
            )
            if difference_low == 0.0 and difference_high == 0.0:
                unresolved_equal_intervals.append(int(edge.edge_id))
                continue
            if difference_low == 0.0:
                parameter = low
            elif difference_high == 0.0:
                parameter = high
            elif difference_low * difference_high < 0.0:
                parameter = low - difference_low * (high - low) / (
                    difference_high - difference_low
                )
            else:
                continue
            arrival_a = arrival(
                station_a0,
                station_a1,
                parameter_a0,
                parameter_a1,
                parameter,
            )
            arrival_b = arrival(
                station_b0,
                station_b1,
                parameter_b0,
                parameter_b1,
                parameter,
            )
            candidates.append(
                (
                    max(arrival_a, arrival_b),
                    min(route_a.key, route_b.key),
                    max(route_a.key, route_b.key),
                    int(edge.edge_id),
                    float(parameter),
                    float(arrival_a),
                    float(arrival_b),
                )
            )
    if unresolved_equal_intervals:
        raise _RailCompileError(
            "RAIL_COMPETITION_METRIC_UNRESOLVED",
            edge_indices=tuple(sorted(set(unresolved_equal_intervals))),
            details=(
                (
                    "route_ids",
                    tuple(sorted((route_a.route_id, route_b.route_id))),
                ),
            ),
        )
    if not candidates:
        return None

    winner = min(candidates)
    edge_id = winner[3]
    edge_parameter = winner[4]
    arrival_a = winner[5]
    arrival_b = winner[6]
    chain_a = _route_origin_chain_ref(
        route_a, topology, chain_refs_by_vertex
    )
    chain_b = _route_origin_chain_ref(
        route_b, topology, chain_refs_by_vertex
    )
    if chain_a is None or chain_b is None:
        raise _RailCompileError(
            "RAIL_COMPETITION_CHAIN_ID_UNRESOLVED",
            edge_indices=(edge_id,),
            details=(
                (
                    "route_ids",
                    tuple(sorted((route_a.route_id, route_b.route_id))),
                ),
            ),
        )
    edge = topology.edge_by_id[edge_id]
    source_vertex_id = None
    kind = RailStationKind.EDGE
    if edge_parameter == 0.0:
        kind = RailStationKind.VERTEX
        source_vertex_id = edge.vertex_ids[0]
    elif edge_parameter == 1.0:
        kind = RailStationKind.VERTEX
        source_vertex_id = edge.vertex_ids[1]
    route_ids = (route_a.route_id, route_b.route_id)
    arrivals = (arrival_a, arrival_b)
    if route_ids[1] < route_ids[0]:
        route_ids = tuple(reversed(route_ids))
        arrivals = tuple(reversed(arrivals))
    chain_pair = tuple(sorted((chain_a, chain_b)))
    return RailFreezeLocus(
        route_id=route_ids[0],
        route_ids=route_ids,
        chain_refs=chain_pair,
        owner_chain_ref=min(chain_pair),
        canonical_distance=max(arrivals),
        arrival_distances=arrivals,
        kind=kind,
        source_vertex_id=source_vertex_id,
        source_edge_id=(None if source_vertex_id is not None else edge_id),
        edge_parameter=(
            None if source_vertex_id is not None else edge_parameter
        ),
        competition_kind=RailCompetitionKind.TERMINAL_ROUTE_PAIR,
    )


def _compile_route_competition(
    routes,
    topology,
    spine_uses,
    terminal_route_ids=(),
):
    """RC1-RC3: два чтения и freeze для каждой RR10-нити."""

    chain_refs_by_vertex = _spine_chain_refs_by_vertex(spine_uses)
    readings = []
    freeze_loci = []
    for route in routes:
        chain_pair = _thread_chain_pair(
            route,
            topology,
            chain_refs_by_vertex,
        )
        if chain_pair is None:
            continue
        start_chain_ref, end_chain_ref = chain_pair
        total_distance = route.stations[-1].distance
        readings.extend(
            (
                RailRouteReading(
                    route_id=route.route_id,
                    chain_ref=start_chain_ref,
                    origin_vertex_id=route.stations[0].source_vertex_id,
                    source_face_ids=route.key.side.source_face_ids,
                    station_distances=tuple(
                        station.distance for station in route.stations
                    ),
                ),
                RailRouteReading(
                    route_id=route.route_id,
                    chain_ref=end_chain_ref,
                    origin_vertex_id=route.stations[-1].source_vertex_id,
                    source_face_ids=route.segments[-1].source_face_ids,
                    station_distances=tuple(
                        total_distance - station.distance
                        for station in route.stations
                    ),
                ),
            )
        )
        freeze_loci.append(
            _freeze_locus_for_route(route, chain_pair, topology)
        )
    terminal_routes = tuple(
        route
        for route in routes
        if route.route_id in frozenset(terminal_route_ids)
        and route.segments
    )
    for index, route_a in enumerate(terminal_routes):
        for route_b in terminal_routes[index + 1 :]:
            if (
                route_a.key.side.spine_vertex_id
                == route_b.key.side.spine_vertex_id
            ):
                continue
            locus = _route_pair_freeze_locus(
                route_a,
                route_b,
                topology,
                chain_refs_by_vertex,
            )
            if locus is not None:
                freeze_loci.append(locus)
    return tuple(sorted(readings)), tuple(sorted(freeze_loci))


def _apply_poles_and_merges(routes, events, protected_route_ids=()):
    """Полюс определяется до merge, затем общие хвосты принадлежат одному route."""

    protected_route_ids = set(protected_route_ids)
    raw_endpoints = defaultdict(list)
    for route in routes:
        if route.route_id in protected_route_ids:
            continue
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
        if route.route_id in protected_route_ids:
            normalized.append(route)
            continue
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
    analysis_bundle,
    selected_edge_indices,
    *,
    alpha_budget,
    rail_mark_edge_indices=(),
):
    """Компилирует RR1-RR10/RC1-RC3; drag читает готовые станции."""

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
        analysis_bundle,
        frozenset(selected_edge_ids),
        frozenset(marked_edge_ids),
    )
    graph = analysis_bundle.patch_graph
    chain_records = _source_chain_records(graph)
    if not selected_edge_ids:
        return DecalRailPlan(
            vertices=topology.vertices,
            edges=topology.edges,
            faces=topology.faces,
            spine_uses=(),
            routes=(),
            route_readings=(),
            freeze_loci=(),
            events=(),
            start_sectors=(),
            terminal_uses=(),
            alpha_budget=alpha_budget,
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
    terminal_uses = []
    terminal_route_keys = set()
    for spine_vertex_id in sorted(selected_by_vertex):
        spine_edges = tuple(sorted(selected_by_vertex[spine_vertex_id]))
        spine_faces = {
            face_id
            for edge_id in spine_edges
            for face_id in topology.edge_faces.get(edge_id, ())
        }
        terminal_side_groups = ()
        terminal_delimiter_edges = ()
        if len(spine_edges) == 1:
            terminal_side_groups, terminal_delimiter_edges = (
                _terminal_side_partition(
                    topology,
                    chain_records,
                    spine_vertex_id,
                    spine_edges[0],
                )
            )
        boundary_faces = set()
        boundary_edges = tuple(
            edge_id
            for edge_id in topology.vertex_edges.get(spine_vertex_id, ())
            if edge_id not in spine_edges
            and edge_id not in terminal_delimiter_edges
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
        if len(spine_edges) == 1:
            spine_edge_id = spine_edges[0]
            for (
                side_faces,
                candidate_edge_ids,
                chosen_edge_id,
                terminal_kind,
            ) in terminal_side_groups:
                terminal_uses.append(
                    RailTerminalUse(
                        spine_vertex_id=spine_vertex_id,
                        spine_edge_id=spine_edge_id,
                        source_face_ids=side_faces,
                        kind=terminal_kind,
                        candidate_edge_ids=candidate_edge_ids,
                        route_edge_id=chosen_edge_id,
                    )
                )
                if chosen_edge_id is None:
                    continue
                route_seeds.append(
                    RailRouteKey(
                        RailSideKey(
                            spine_vertex_id,
                            chosen_edge_id,
                            side_faces,
                        )
                    )
                )
                terminal_route_keys.add((spine_vertex_id, chosen_edge_id))
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

    route_seed_faces = defaultdict(set)
    for key in route_seeds:
        route_seed_faces[
            (key.side.spine_vertex_id, key.side.start_edge_id)
        ].update(key.side.source_face_ids)
    canonical_route_seeds = tuple(
        RailRouteKey(
            RailSideKey(
                spine_vertex_id,
                start_edge_id,
                tuple(sorted(source_face_ids)),
            )
        )
        for (spine_vertex_id, start_edge_id), source_face_ids in sorted(
            route_seed_faces.items()
        )
    )

    routes = []
    events = []
    next_route_id = 0
    for key in canonical_route_seeds:
        route, route_events = _trace_route(
            topology,
            next_route_id,
            key,
            alpha_budget,
            barrier_vertices,
            spine_vertices,
            chain_records,
            terminal_route_keys,
        )
        routes.append(route)
        events.extend(route_events)
        next_route_id += 1
    for side_key in sorted(set(start_dams)):
        route, route_events = _start_dam_route(next_route_id, side_key)
        routes.append(route)
        events.extend(route_events)
        next_route_id += 1

    routes, events, route_start_aliases = _canonicalize_thread_routes(
        tuple(routes),
        tuple(events),
        topology,
        tuple(spine_uses),
    )
    protected_route_ids = {
        route.route_id
        for route in routes
        if (
            route.key.side.spine_vertex_id,
            route.key.side.start_edge_id,
        ) in terminal_route_keys
    }
    routes, events = _apply_poles_and_merges(
        tuple(routes),
        tuple(events),
        protected_route_ids,
    )
    route_by_start = {
        (route.key.side.spine_vertex_id, route.key.side.start_edge_id): route
        for route in routes
        if route.key.side.start_edge_id >= 0
    }
    route_by_id = {route.route_id: route for route in routes}
    for start_key, route_id in route_start_aliases.items():
        route = route_by_id.get(route_id)
        if route is not None:
            route_by_start.setdefault(start_key, route)
    terminal_uses = tuple(
        replace(
            use,
            route_id=(
                route_by_start[
                    (
                        use.spine_vertex_id,
                        use.route_edge_id,
                    )
                ].route_id
                if use.kind == RailTerminalKind.ROUTE
                and use.route_edge_id is not None
                else None
            ),
        )
        for use in terminal_uses
    )
    footprint_face_ids = set(rr_face_ids)
    for route in routes:
        footprint_face_ids.update(route.key.side.source_face_ids)
        for segment in route.segments:
            footprint_face_ids.update(segment.source_face_ids)
    # RV1/RV2 имеют приоритет над RC4: конкуренция не должна маскировать
    # именованный дефект source-геометрии внутри уже известного footprint.
    _validate_rail_footprint(topology, footprint_face_ids)
    route_readings, freeze_loci = _compile_route_competition(
        routes,
        topology,
        tuple(spine_uses),
        terminal_route_ids=tuple(
            use.route_id
            for use in terminal_uses
            if use.route_id is not None
        ),
    )
    return DecalRailPlan(
        vertices=topology.vertices,
        edges=topology.edges,
        faces=topology.faces,
        spine_uses=tuple(spine_uses),
        routes=routes,
        route_readings=route_readings,
        freeze_loci=freeze_loci,
        events=events,
        start_sectors=tuple(start_sectors),
        terminal_uses=terminal_uses,
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
    "RailCompetitionKind",
    "RailEvent",
    "RailEventKind",
    "RailFreezeLocus",
    "RailRoute",
    "RailRouteKey",
    "RailRouteReading",
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
    "RailTerminalKind",
    "RailTerminalUse",
    "RailTermination",
    "compile_decal_rail_attempt",
    "compile_decal_rail_plan",
]
